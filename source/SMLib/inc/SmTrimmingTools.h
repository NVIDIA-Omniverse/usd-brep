// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTrimmingTools.h
* PURPOSE: Header file for SmTrimmingTools object.
**********************************************************************/

#ifndef __SMTRIMMINGTOOLS_H__
#define __SMTRIMMINGTOOLS_H__

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

/*******************************************************************//**
   First some helper classes, to facilitate handling of loops of curves.
   Note, these also deal with intersections on surfaces, to aid with handling surface creases.
***********************************************************************/

/*******************************************************************//**
PURPOSE: A little class for keeping track of an iso-curve's intersections with Loop curves.
***********************************************************************/
class SmCurveIntersections
{
public:
  SmCurve              * m_pIsoCurve = NULL ;  // IsoParamCurve being intersected
  SmSolutionArray        m_sSolArray ;         // Array of IsoParamCurve/Curve XSects
  SmSurfParamType        m_eSrfDir ;           // one of SM_SP_U or SM_SP_V The Const param of the IsoParamCurve
  double                 m_dIsoParam ;         // Iso Curve constant param value.  For SM_SP_U a u value, for SM_SP_V a v value.
  SmCurveIntersections * m_pNext = NULL;       // Linked list

  // Constructor:
  SmCurveIntersections(SmCurve              * pIsoCurve,    // in : IsoParamCurve being intersected
                       SmSurfParamType        eSrfDir,      // in : one of SM_SP_U or SM_SP_V The Const param of the IsoParamCurve
                       double                 dIsoParam,    // in : Iso Curve constant param value.  For SM_SP_U a u value, for SM_SP_V a v value.
                       SmCurveIntersections * pNextCrvInts) // in : Linked list's next SmCurveIntersections object
                     : m_pIsoCurve ( pIsoCurve    ),
                       m_eSrfDir   ( eSrfDir      ),
                       m_dIsoParam ( dIsoParam    ),
                       m_pNext     ( pNextCrvInts )
                     { }

}; // end class SmCurveIntersections

/*******************************************************************//**
PURPOSE: A class to represent a segment of a cross curve with a curve loop.
         The curve loop has already been split, so 
         the intersections are at the junctures between curve loops.

NOTES:
***********************************************************************/
class SmIsoXSectSegment
{
 public:
  SmCurve         * m_pIsoCurve ;                // m_pIsoCurve points to one original Curve being inserted into a CurveLoop data structure.  
                                                 //   It gets copied twiced, once for for each segment added to CurveLoop data array.  
                                                 //   m_pIsoCurve is not owned - the copied curves are owned.
  SmExtent1d        m_sInterval ;                // The interval on m_pIsoCurve between Enter and Leave
  SmOrientType      m_eOrient     = SM_OT_SAME;  // Orientation of m_pIsoCurve.
  ULONG             m_lStartEnter = 9999 ;       // Indices into the Curves array of the entering and leaving curves
  ULONG             m_lStartLeave = 9999 ;       //   at the start and end of the segment of interest.
  ULONG             m_lEndEnter   = 9999 ;       //   Usually, 'Leave' == 'Enter+1', unless they straddle
  ULONG             m_lEndLeave   = 9999 ;       //   the 'seam' of a loop.

  // Constructor
  SmIsoXSectSegment( SmCurve *pIsoCurve ) : m_pIsoCurve( pIsoCurve ) {}

  // Destructor - nothing owned.
  ~SmIsoXSectSegment() { }

  // Convenience routines
  ULONG GetMinIndex()  { SM_ASSERT( m_lStartLeave == m_lStartEnter + 1  || m_lStartEnter == 9999 );
                         SM_ASSERT( m_lEndLeave   == m_lEndEnter   + 1  || m_lEndEnter   == 9999 );
                         return smos_Min( m_lStartEnter, m_lEndEnter );
                       }

  ULONG GetMaxIndex()  { SM_ASSERT( m_lStartLeave == m_lStartEnter + 1  || m_lStartEnter == 9999 );
                         SM_ASSERT( m_lEndLeave   == m_lEndEnter   + 1  || m_lEndEnter   == 9999 );
                         return smos_Max( m_lStartLeave, m_lEndLeave );
                       }

} ; // end class SmIsoXSectSegment

/*******************************************************************//**
PURPOSE: Helper class, to encapsulate handling of curve loops.

NOTES: Curve Loops are implemented as three arrays:
   - Curves:  List of all the curves in all loops
   - Orients: List of orientations for each of the curves
   - Loops:   List of the counts of curves in each Loop

   The number of entries in the Loops array is the number of separate Loops.

   Currently, the curves must be ordered end to end, and loops must be closed.
   This class could be enhanced to order the curves into closed loops.

   The curves can be 2d (uv) or 3d.

***********************************************************************/
class SmCurveLoops
{
 public:

  SmTArray<SmCurve*>     m_sCurves ;      // List of all the curves in all loops
  SmTArray<SmOrientType> m_sOrients ;     // List of orientations for each of the curves
  SmTArray<ULONG>        m_sLoopCounts ;  // List of the counts of curves in each Loop, m_sLoopCounts.GetSize() = number of loops
  SmBoolean              m_bIsValid;      // Flag set by constructor
                                          //   TRUE = constructor input ok
                                          //   FALSE= constructor input included neither UVTrimCurves nor 3D Curves - must have some curves
                           
  // Empty Constructor
  SmCurveLoops() { m_bIsValid = TRUE; } 

  // constructor
  SmCurveLoops(const SmTArray<ULONG>           & crCurveLoops,          // in : List of the counts of curves in each Loop, m_sLoopCounts.GetSize() = number of loops
               const SmTArray<SmBSplineCurve*> * cpOptUVCurves,         // in : opt list of UVTrimCurves to copy, one of cpOptUVCurves or cpOpt3DCurves must be NonNULL
               const SmTArray<SmCurve*>        * cpOpt3DCurves,         // in : opt List of 3d curves to copy,    only used when cpOptUVCurves is NULL, one of cpOptUVCurves or cpOpt3DCurves must be NonNULL 
               const SmTArray<SmOrientType>    & crCurveOrientations);  // in : List of orientations for each of the curves   
                                               
   // destructor
  ~SmCurveLoops() { } 

  // clear data structs
  SmStatus Reset() ;

  // data access
  SmBoolean IsValid()                          const { return m_bIsValid; }
  ULONG     GetNumLoops()                      const { return m_sLoopCounts.GetSize(); }
  SmStatus  GetLoopStartIndex(ULONG   lLoopIndx,              // in : Loop number: Index into m_sLoopCounts    
                              ULONG & rlStart) const ;        // out: Index of the first curve in lIndx's loop  
  SmStatus  GetLoopStartEnd  (ULONG   lCurveIndx,             // in : Index of the curve in question         
                              ULONG & rlStart,                // out: Index of the first curve in lIndx's loop      
                              ULONG & rlEnd ) const;          // out: Index of the last  curve in lIndx's loop      
            
            

  // queries
  
  // Find the adjacent loop curve, given the start or end of a tgt loop curve.
  SmStatus FindAdjacentLoopCurve(SmCurve * pCrv,        // in : Pointer to the curve in question        
                                 double    dParam,      // in : Parameter on pCrv: indicates start/end  
                                 ULONG   & rlAdjIndx) ; // out: Index of the adjacent curve in the loop 

  // given a consecutive pair of curves in a loop - find which is entering and which is leaving
  SmStatus FindEnteringAndLeaving(ULONG   lCurveIndx1,     // in : index in the LoopCurves array of one of the two connecting Loop curves
                                  ULONG   lCurveIndx2,     // in : index of the other intersecting Loop curve
                                  ULONG & rlEnterCurve,    // in : Whichever of lIndx1 or lIndx2 that is first
                                  ULONG & rlLeaveCurve) ;  // in : Whichever of lIndx1 or lIndx2 that is second
  
  // sample points near param on TgtCrv.  When dparam is on Curve end sample pt on adjacent curve in the loop
  SmStatus SampleNearbyPoints(SmCurve   * pCrv,     // in : Pointer to the curve in question        
                              double      dParam,   // in : Parameter on pCrv: indicates start/end  
                              SmPoint3d & rPt1,     // out: A point on the loop just before pCrv/dParam 
                              SmPoint3d & rPt2) ;   // out: A point on the loop just after pCrv/dParam  

  // Given a loop curve and its start/end, decide whether the intersection is crossing or same-side (possibly tangent).
  SmBoolean IsSameSideIntersection(SmCurve              * pCrv,         // in : Pointer to the curve in question        
                                   double                 dParam,       // in : Parameter on pCrv: indicates start/end  
                                   SmCurveIntersections * pCurveInts,   // in : An isocurve/loop curve intersection 
                                   const SmSurface      * cpSrf,        // in : Surface containing the curves 
                                   const SmExtent2d     & crSrfDomain,  // in : Domain of interest on cpSrf  
                                   SmZoneTol3d          & rZoneTol3d) ; // in : 3d tol for dropping points to pSrf 

  // edits
  
  // Append a Loop from another SmCurveLoops object onto this one
  SmStatus AppendLoop               (const SmCurveLoops       & crOther,           // in : Another SmCurveLoops                
                                     ULONG                      lOtherLoopIndex) ; // in : Index into m_sLoopCounts in crOther 

  // Insert a curve into the curve loop structure. Used when a loop curve is split.
  SmStatus InsertIntoLoops          (SmCurve                  * pNewCrv,           // in : The new curve to be inserted 
                                     SmCurve                  * pLoopCrv) ;        // in : Existing curve to insert after 

  // insert IsoCurve segment already XSected with LoopCurves into a loop - splits loop
  SmStatus InsertIsoXSectSegment(SmIsoXSectSegment & rIntseg) ;

  // update XSect Solution objects to approriate child curve after splitting a curve 
  SmStatus UpdateSolutionObjects( SmCurveIntersections * pCrvInts,   // in : linked list of all the CurveInts 
                                  SmCurve              * pOrigCurve, // in : the curve that got split (reused as one child) 
                                  SmCurve              * pNewCurve,  // in : the new curve, (the other child) split off from pOrigCurve 
                                  SmZoneTol3d          & rTol3d );   // in : siZe of tolerant neighborhood about each curve
} ; // end class SmCurveLoops

/*******************************************************************//**
PURPOSE: This class provides tools for creating trimmed surfaces in
   an existing Brep given data which is not in the standard SMLib
   format.

   It also allows access to some of the routines used to do this, as
   they are also useful when importing BReps.

NOTES: The methods provided by this class should be used when
    you need to import data from systems that do not follow all of the
    SMLib conventions.  It also enables the conversion of periodic
    trimmed surfaces where curves cross the seam of the surface.

***********************************************************************/
class SM_EXPORT SmTrimmingTools
{
public:
  static SmStatus TrimSurfaceWithModelSpaceCurves    ( SmContext const              & crContext,                         // in : new object construction                                                                               
                                                       SmRegion                     * pRegion,                           // in : Target region in existing Brep to receive new trimmed surface.                                        
                                                       double                         d3DTolerance,                      // in : Tolerance assigned to Brep and passed to SplitAtSeams().                                              
                                                       const SmTArray<ULONG>        & crCurveLoops,                      // in : crCurveLoops.GetSize() = number of loops to make                                                      
                                                                                                                         //    : crCurveLoops[i]        = number of curves in ith loop                                                 
                                                       const SmTArray<SmCurve*>     & cp3DCurves,                        // in : Curve array ordered by loop and neighbor.                                                             
                                                                                                                         //    : 1st loop is outer loop, subsequent optLoops are inner loops.                                          
                                                                                                                         //    : outer loop orientation = counter clockwise                                                            
                                                                                                                         //    : inner loop orientation = clockwise                                                                    
                                                                                                                         //    : These curves are consumed by this method.                                                             
                                                       const SmTArray<SmOrientType> & crCurveOrientations,               // in : SM_OT_SAME     = curve's orientation in its loop is the same as it's parametric orientation           
                                                                                                                         //    : SM_OT_OPPOSITE = curve's orientation in its loop is the oppositie of it's parametric orientation      
                                                       const SmTArray<SmPoint3d>    & crLoopPoints,                      // in : Points within the outer loop to become vertex loops.                                                  
                                                       SmSurface                    * pSurface,                          // in : The surface to be trimmed. It's consumed by this operation.                                           
                                                       const SmExtent2d             & crUVDomain,                        // in : Domain of the surface used by the face.                                                               
                                                       SmOrientType                   eSurfaceOrientation,               // in : SM_OT_SAME     = face oriented with the surface normal                                                
                                                                                                                         //    : SM_OT_OPPOSITE = face oriented against the surface normal (negates the CurveOrientation requirements) 
                                                       SmBoolean                      bFirstLoopIsOuterLoop,             // in : Not used in this function.                                                                            
                                                       SmBoolean                      bLoopsAreOrientedCorrectly,        // in : TRUE =                                                                                                
                                                                                                                         //    : FALSE=                                                                                                
                                                       SmBoolean                      bLoopMayCrossASeam,                // in : TRUE =                                                                                                
                                                                                                                         //    : FALSE=                                                                                                
                                                       SmTArray<SmFace*>            * rpNewFaces = NULL) ;               // out: Optional pointer to array of new faces
                                                       
  static SmStatus TrimSurfaceWithModelSpaceCurves    ( SmContext const              & crContext,                         // in :        
                                                       double                         d3DTolerance,                      // in :        
                                                       const SmTArray<ULONG>        & crCurveLoops,                      // in :        
                                                       const SmTArray<SmCurve*>     & cp3DCurves,                        // in :        
                                                       const SmTArray<SmOrientType> & crCurveOrientations,               // in :        
                                                       const SmTArray<SmPoint3d>    & crLoopPoints,                      // in :        
                                                       SmSurface                    * pSurface,                          // in :        
                                                       const SmExtent2d             & crUVDomain,                        // in :        
                                                       SmOrientType                   eSurfaceOrientation,               // in :        
                                                       SmBoolean                      bFirstLoopIsOuterLoop,             // in :        
                                                       SmBoolean                      bLoopsAreOrientedCorrectly,        // in :        
                                                       SmBoolean                      bLoopMayCrossASeam,                // in :        
                                                       SmBrep                      *& rpBrep) ;                          // out:        
                                                     
  static SmStatus TrimSurfaceWithParameterSpaceCurves( SmContext const                   & crContext,                    // NotUsed-in :                                                                              
                                                       SmRegion                          * pRegion,                      // in : Region in an existing brep in which new trimmed surface is to be created.    
                                                       double                              d3DTolerance,                 // in :                                                                              
                                                       const SmTArray<ULONG>             & crCurveLoops,                 // in :                                                                              
                                                       const SmTArray<SmBSplineCurve*>   & cpUVCurves,                   // in : Parameter space curves which are consumed in this operation.                 
                                                       const SmTArray<SmOrientType>      & crCurveOrientations,          // in :                                                                              
                                                       const SmTArray<SmPoint3d>         & crLoopPoints,                 // in :                                                                              
                                                       SmSurface                         * pSurface,                     // in : Surface which is consumed in this operation.                                 
                                                       const SmExtent2d                  & crUVDomain,                   // in :                                                                              
                                                       SmOrientType                        eSurfaceOrientation,          // in :                                                                              
                                                       SmBoolean                           bFirstLoopIsOuterLoop,        // in :                                                                              
                                                       SmBoolean                           bLoopsAreOrientedCorrectly) ; // in :                                                                              
                                                       

  static SmStatus TrimSurfaceWithParameterSpaceCurves( SmContext const                   & crContext,                    // in :                                                               
                                                       double                              d3DTolerance,                 // in :                                                               
                                                       const SmTArray<ULONG>             & crCurveLoops,                 // in :                                                               
                                                       const SmTArray<SmBSplineCurve*>   & cpUVCurves,                   // in : Parameter space curves which are consumed in this operation.  
                                                       const SmTArray<SmOrientType>      & crCurveOrientations,          // in :                                                               
                                                       const SmTArray<SmPoint3d>         & crLoopPoints,                 // in :                                                               
                                                       SmSurface                         * pSurface,                     // in : Surface which is consumed in this operation                   
                                                       const SmExtent2d                  & crUVDomain,                   // in :                                                               
                                                       SmOrientType                        eSurfaceOrientation,          // in :                                                               
                                                       SmBoolean                           bFirstLoopIsOuterLoop,        // in :                                                               
                                                       SmBoolean                           bLoopsAreOrientedCorrectly,   // in :                                                               
                                                       SmBrep                           *& rpBrep) ;                     // out:                                                               
                                                       
public:

  static SmStatus SplitAtSeams                       (const SmContext   & crContext,                                     // NotUsed-in : context for new object construction                          
                                                      SmFace            * pFace,                                         // in : target face                                                  
                                                      SmTArray<SmFace*> & rNewFaces,                                     // out: array of new faces appended to by this result                
                                                      SmBoolean           bAlsoSplitAtMiddleOfClosedSurfaces = FALSE) ;  // NotUsed-in : TRUE = when surface is closed, also split face               
                                                                                                                         //    :        at midSurface isoParameterCurve.                      
                                                                                                                         //    : FALSE= just split face at closed boundaries                  
                                                                                                                         
  static SmStatus SplitAtSeams                       (SmContext const   & crContext,                                     // in : context for new object construction                                              
                                                      SmBrep            * pBrep,                                         // in : target Brep                                                                      
                                                      SmTArray<SmFace*> & rpNewFaces,                                    // out: resulting faces created by operation                                             
                                                      SmBoolean           bAlsoSplitAtMiddleOfClosedSurfaces = FALSE) ;  // in : TRUE = when surface is closed, also split face at midSurface isoParameterCurve.  
                                                                                                                         //    : FALSE= just split face at closed boundaries                                      
                                                        
                                                     
  static SmStatus SurfaceContinuityCheck             (SmSurface const * pSurface,                                        // in :     
                                                      SmBoolean       & rbSurfaceIsAtLeastG1) ;                          // in :     

  // decided not to implement this method - not yet needed for the PRC<->SMB project
  // - kind of overlaps the method SmCurve::FixChainCurvesList()
  // extract all loops made from a connected network of edges.
//static SmStatus FindTrimmingLoopsInConnectedCurves(SmSurface              & rSurface,                     // in : Surface to trim                            
//                                                   SmTArray<SmCurve*>     & rCurvesInOneLoop,             // in : Loop to trim surface:                      
//                                                   SmTArray<SmOrientType> & rCurveOrientationsInOneLoop,  // i/o: associated Trim curve orientations         
//                                                                                                          //    : oneof: SM_OT_SAME, SM_OT_OPPOSITE          
//                                                   SmOrientType           & rOrientationWithSurface) ;    // in : SM_OT_SAME     = loop on surface up side   
//                                                                                                          // in : SM_OT_OPPOSITE = loop on surface down side 
//                                                                                                          // in : SM_OT_OPPOSITE = loop on surface down side 
                                                                                                            
  // set orientation for every LoopCurve by walking a given head-to-tail set of curves
  static SmStatus FindLoopCurveOrientations  ( SmSurface              & rSurface,            // in : Surface to trim                                            
                                               SmTArray<SmCurve*>     & rCurvesInOneLoop,    // i/o: curves in loop-sequenced that form a surface loop          
                                               SmOrientType             eFirstCurveOrient,   // in : SM_OT_SAME     = walk with 1st crv parameterization dir    
                                                                                             //    : SM_OT_OPPOSITE = walk against 1st crv parameterization dir 
                                               SmTArray<SmCurve*>     & rOrderedCurves,      // out: Curves oredered head to tail                               
                                               SmTArray<SmOrientType> & rCurveOrients) ;     // out: associated Trim curve orientations                         
                                                                                             //    : oneof: SM_OT_SAME, SM_OT_OPPOSITE                          
                                                                                                               
  static SmStatus ComputeLoopOrientation    ( SmTArray<SmBSplineCurve *>  const & crpAllUVCurves,    // in :                                           
                                              SmTArray<SmOrientType>      const & creAllCurveSenses, // in :                                           
                                              ULONG                               iIndexBegin,       // in : indices of this loop in the input arrays  
                                              ULONG                               iIndexEnd,         // out:                                           
                                              SmOrientType                      & rOrientation) ;    
                                                                                                              
  static SmStatus FixTrimLoopTinyParameterSpaceGaps( SmLoop    * pLoop,          // in : target loop                                   
                                                     double      d2DTolerance,   // in : max allowed UVTrimCurve-UVTrimCurve gap       
                                                     SmBoolean & rbRepairMade) ; // out: TRUE = UVTrimCurves modified to close gaps    
                                                                                 //    : FALSE= NO UVTrimCurves were modified          
                                                                                                               
  static SmStatus FixTrimLoopTinyParameterSpaceGaps( SmFace    * pFace,          // in :    
                                                     double      d2DTolerance,   // in :    
                                                     SmBoolean & rbRepairMade) ; // in :    
                                                                                 
                                                                                 
  static SmStatus FixBackwardsOuterLoopOnSeam      ( SmFace    * pFace,          // in :    
                                                     SmBoolean & rbRepairMade) ; // in :    
                                                                                 
                                                                                     
  static SmStatus FixTrimLoopOrientation           ( SmFace    * pFace,              // in :     
                                                     SmLoop    * pLoop,              // in :     
                                                     SmBoolean   bOuterLoop,         // in :     
                                                     SmBoolean & rbLoopReoriented) ; // out:     
                                                                                                               
                                                                                                               
  static SmStatus FixTrimLoopOrientation           ( SmFace *pFace,                     // in :     
                                                     ULONG  &riNumberLoopsReoriented) ; // out:     
                                                                                                               
                                                                                                               
  static SmStatus FixTrimLoopOrder                 ( SmFace    *pFace,              // in :     
                                                     SmBoolean &rbLoopsReordered) ; // out:     
                                                                                                              
  static SmStatus OrderCurvesIntoLoops             ( const SmTArray<SmCurve*> & rsCurves,       // in : target curves to order                                   
                                                     double                     d3dTol,         // in : maximum length for a degenerate curve and                
                                                                                                //    : lower bound on maximum allowed curve/curve gap size      
                                                                                                //    : max dist from plane                                      
                                                     SmTArray<SmCurve*>       & rOrderedCurves, // out: Curves reordered into loop sequences (not copied)        
                                                     SmTArray<ULONG>          & rsLoopCounts,   // out: number of output loops       = rsLoopCounts.GetSize()    
                                                                                                //    : number of curves in ith loop = rsLoopCounts[i],          
                                                     SmTArray<SmOrientType>   & rsOrients,      // out: a SM_OT_SAME/SM_OT_OPPOSITE value for each OrderedCurve  
                                                     SmPoint3d                & rsPlanePoint,   // out: Pt on output plane containing all input curves           
                                                     SmVector3d               & rsPlaneNormal,  // out: Normal to output plane containing all input curves       
                                                     SmExtent3d               & rsCurvesBBox) ; // out: Bounding box containing all input curves                 
                                                                                                              
                                                   
  static SmStatus FindOuterLoops                   ( SmTArray<SmCurve*>       & rsInCurves,                // i/o: consecutive Loop CrvSets. (CrvSet = Curves of one loop ordered 1st to last)   
                                                                                                           //    : in = Loops not yet ordered                                                       
                                                                                                           //    : out= Sequence of[OuterLoop CCW CrvSet followed by its InnerLoop CW CrvSets]      
                                                                                                           //    : sized:[NumCrvs]                                                                  
                                                      SmTArray<ULONG   >      & rsInLoops,                 // i/o: Num of Crvs in each Loop, sized:[NumLoops]                                    
                                                      SmTArray<SmOrientType>  & rsInOrients,               // i/o: assoc orientation of each Crv in its Loop, sized:[NumCrvs]                    
                                                      SmVector3d              & rsPlaneNormal,             // in : Normal to plane containing curves                                                
                                                      double                    d3DTol,                    // in : min dist between distinct points                                                 
                                                      SmTArray<ULONG>         & rsOuterLoopStartIndices) ; // out: Num of Loops in each outerLoop, sized:[NumOuterLoops]                            
                                                                                                           //    : ex: rsOuterLoopCounts[i] = 3 => 2 innerLoops inside OuterLoop[i]                 
                                                                                                                  
                                                                                                                  
  static SmStatus CheckLoopForMiniHourglass         ( SmLoop const     * pLoop,                   // in :                                                                                   
                                                      SmAssertArray    * pAList = NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                            
                                                      SmAssertTestLevel  eTestLevel = SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                        
                                                                                                  //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                    
                                                                                                  //    : default:[SM_LEVEL_0]                                                              
                                                      SmTArray<ULONG>  * pTestRequests = NULL) ;  // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]  
                                                                                                  
                                                                                                  
  static SmStatus CheckTessellation                ( SmFace const     * pFace,                    // in : target face                                                                      
                                                     SmAssertArray    * pAList = NULL,            // NotUsed-i/o: Accumulating list of failed Asserts, NULL to ignore                           
                                                     SmAssertTestLevel  eTestLevel = SM_LEVEL_0,  // NotUsed-in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                       
                                                                                                  //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                   
                                                                                                  //    : default:[SM_LEVEL_0]                                                             
                                                     SmTArray<ULONG>  * pTestRequests = NULL) ;   // NotUsed-in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL] 
                                                   
                                                   
  static SmStatus CheckFace                       ( SmFace const     * pFace,                     // in : target face                                                                        
                                                    double             d3DTolerance,              // in : Given Max SurfaceGap Size                                                          
                                                    SmAssertArray    * pAList = NULL,             // i/o: Accumulating list of failed Asserts, NULL to ignore                             
                                                    SmAssertTestLevel  eTestLevel = SM_LEVEL_0,   // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                         
                                                                                                  //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                     
                                                                                                  //    : default:[SM_LEVEL_0]                                                               
                                                    SmTArray<ULONG>  * pTestRequests = NULL,      // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                                                    SmBoolean          bSkipCheckLoop = FALSE) ;  // in : TRUE = skip CheckLoop (caller will run it from Loop::AssertValid) 
                                                                                                  
                                                                                                        
  static SmStatus CheckFaces                      ( SmTArray<SmFace *> const & crpFaces,                             
                                                    double                     d3DTolerance,            // in : list of target faces                                                              
                                                    SmAssertArray            * pAList = NULL,           // in : Tolerance used to check loop gaps                                                 
                                                    SmAssertTestLevel          eTestLevel = SM_LEVEL_0, // i/o: Accumulating list of failed Asserts, NULL to ignore                            
                                                                                                        // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                        
                                                                                                        //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                    
                                                    SmTArray<ULONG>          * pTestRequests = NULL) ;  //    : default:[SM_LEVEL_0]                                                              
                                                                                                        // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]  
                                                                                                        
                                                                                                        
  static SmStatus CheckLoop                       ( SmSurface const  * pSurface,                        // in : target loop                                                                        
                                                    SmLoop const     * pLoop,                           // in : max distance allowed between loop edges                                            
                                                    double             d3DTolerance,                    // out: 0: The uv-curves form a closed loop.                                               
                                                    ULONG            & rlClosure,                       //    : 1: The uv-curves are closed except at singularities.                               
                                                                                                        //    : 2: The uv-curves are not closed, even with singularities.                          
                                                                                                        //    : 3: The loop is not closed in 3d.                                                   
                                                    SmAssertArray    * pAList = NULL,                   // i/o: Accumulating list of failed Asserts, NULL to ignore                             
                                                    SmAssertTestLevel  eTestLevel = SM_LEVEL_0,         // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                         
                                                                                                        //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                     
                                                                                                        //    : default:[SM_LEVEL_0]                                                               
                                                                                                        // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]   
                                                    SmTArray<ULONG>  * pTestRequests = NULL) ;
                                                    
                                                    
  static SmStatus CheckFaceUVSeamJumps            ( const SmFace           * pFace,                     // in :
                                                    SmTArray< SmEdgeuse* > & raBadEUs,                  // out:                                                                                  
                                                    SmAssertArray          * pAList = NULL,             // NotUsed-i/o: Accumulating list of failed Asserts, NULL to ignore                           
                                                    SmAssertTestLevel        eTestLevel = SM_LEVEL_0,   // NotUsed-in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                       
                                                                                                        //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                   
                                                                                                        //    : default:[SM_LEVEL_0]                                                             
                                                    SmTArray<ULONG>        * pTestRequests = NULL) ;    // NotUsed-in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL] 
                                                                                                           
                                                                                                           
  static SmStatus RecreateUVTrimCurves            ( SmTArray< SmEdgeuse* > & rEdgeuses) ;                  
                                                                                                           
  static SmStatus GetEdgeUVExtremes               ( SmEdgeuse const * pEdgeuse,                         // in : target edgeuse                       
                                                    SmPoint2d       & rStart,                           // out: edgeuse->UVTrimCurve Natural Start   
                                                    double          & rdStartWeight,                    // out: associated weight                    
                                                    SmPoint2d       & rEnd,                             // out: edgeuse->UVTrimCurve Natural Start   
                                                    double          & rdEndWeight) ;                    // out: associated weight                    
                                                                                                        
  static SmStatus GetEdgeUVExtremes               ( SmEdgeuse const * pEdgeuse,                         // in : target edgeuse                  
                                                    SmPoint2d       & rStart,                           // out: edgeuse->StartUVPoint           
                                                    SmPoint2d       & rEnd) ;                           // out: edgeuse->EndUVPoint             
                                                    
                                                    
  static SmStatus ApproximateLengthThroughSurface ( SmBSplineCurve const * cpUVCurve,                   // in :    
                                                    SmSurface      const * pSurface,                    // in :    
                                                    ULONG                  iNumberSamples,              // in :    
                                                    double               & rdApproximateLength) ;       // out:    
                                                    
                                                                                                        
  static SmStatus IsCurveOnSurfaceDegenerate      ( SmBSplineCurve * cpUVCurve,                         // in :      
                                                    SmSurface      * pSurface,                          // in :      
                                                    double           d3DTolerance,                      // in :      
                                                    SmBoolean      & rbCurveIsDegenerate) ;             // out:      
                                                                                                     
                                                                                                     
  static SmBoolean IsIsoCurve                     ( const SmCurve   * cpUVCurve,                        // in : uv curve to test                        
                                                    const SmSurface * cpSurf,                           // in : surface in which cpUVCurve is defined   
                                                    double            dTol3d,                           // in : 3d tolerance                            
                                                    SmSurfParamType & reWhichDir,                       // out: SM_SP_U or _V                           
                                                    double          & rdParamValue) ;                   // out: constant u- or v- parameter             
                                                                                                        
                                                                                                        
  static SmBoolean IsIsoCurve                     ( SmBSplineCurve const * cpUVCurve,                   // in : uv curve to test   
                                                    SmSurfParamType      & rSide,                       // in :                    
                                                    double               & rdParameter) ;               // in :                    
                                                                                                        
                                                                                                        
  static SmBoolean IsIsoCurve                     ( SmBSplineCurve const * cpUVCurve,                   // in :          
                                                    SmSurfParamType        eSide,                       // in :          
                                                    double                 dLow,                        // in :          
                                                    double                 d2DTolerance) ;              // in :          
                                                    
                                                    
  static SmBoolean IsIsoCurveOnSeam               ( SmSurface const      * cpSurface,                   // in :          
                                                    SmBSplineCurve const * cpUVCurve,                   // in :          
                                                    SmSurfParamType      & rSide,                       // in :          
                                                    double               & rdParameter) ;               // in :          
                                                    
                                                    
  static SmStatus CopyNonDegenerate               ( const SmTArray<ULONG>             & crCurveLoops,         // in :          
                                                    SmTArray<SmBSplineCurve*>         & cpUVCurves,           // in :          
                                                    const SmTArray<SmOrientType>      & crCurveOrientations,  // in :          
                                                    SmSurface                         * pSurface,             // in :          
                                                    double                              d3DTolerance,         // in :          
                                                    SmTArray<ULONG>                   & loop_curve_counts,    // in :          
                                                    SmTArray<SmBSplineCurve*>         & uv_curves,            // in :          
                                                    SmTArray<SmOrientType>            & curve_orientations) ; // in :          
                                                    
                                                    
  static SmStatus OrientUVLoops                  ( SmTArray<ULONG> const     & loop_curve_counts,        // in :           
                                                   SmTArray<SmBSplineCurve*> & uv_curves,                // in :           
                                                   SmTArray<SmOrientType>    & curve_orientations) ;     // in :           
                                                   
                                                   
  static SmStatus EnsureLastEdgeuseUVCorrect     ( SmSurface const      * pSurface,                      // in :             
                                                   SmTArray<SmEdgeuse*> & rpEdgeuses,                    // out:             
                                                   SmBoolean            & rbAllEdgeusesCorrect) ;        // out:             
                                                   
                                                   
  static SmStatus MirrorCurve                    ( SmBSplineCurve * pCurve,
                                                   SmSurfParamType  eSide,                               // in :            
                                                   double           dLow,                                // in :            
                                                   double           dHigh,                               // in :            
                                                   double           dTolerance) ;                        // NotUsed-in :            
                                                   
                                                   
  static SmBoolean AreUVCurvesPresent( SmLoop const *pLoop) ;
  static SmBoolean AreUVCurvesPresent( SmFace const *pFace) ;
  static SmBoolean AreUVCurvesPresent( SmBrep const *pBrep) ;

  // Questionable if the following should be const pointers
  static SmStatus CreateUVTrimCurves( SmFace const *pFace) ;
  static SmStatus CreateUVTrimCurves( SmBrep const *pBrep) ;

  // Questionable if the following should be const pointers
  static void ClearUVTrimCurves( SmFace const *pFace) ;
  static void ClearUVTrimCurves( SmBrep const *pBrep) ;

  // Shouldn't these be const if the above are?
  static SmStatus RedropUVTrimCurves( SmFace *pFace) ;
  static SmStatus RedropUVTrimCurves( SmBrep *pBrep) ;

  static double Min( double a, double b) ;

public:
  // SM_COMMON functions - can't use the macro when class not derived from SmObject
  SM_COMMON_ONLY_BASE( SmTrimmingTools, SmTrimmingTools_TYPE) ;

  //              SM_TYPE   GetType()       const     { return     SmTrimmingTools_TYPE ; }    
  // const        TCHAR *   GetTypeString() const     { return _T("SmTrimmingTools_TYPE") ; } 
  // static       SM_TYPE   GetClassType()            { return     SmTrimmingTools_TYPE ; }   
  // static const TCHAR *   GetClassTypeString()      { return _T("SmTrimmingTools_TYPE") ; } 
  //              SmBoolean IsKindOf(SM_TYPE t) const { return ((SmTrimmingTools_TYPE == t) ? TRUE : FALSE); } 

}; // end class SmTrimmingTools

#endif // !__SMTRIMMINGTOOLS_H__

