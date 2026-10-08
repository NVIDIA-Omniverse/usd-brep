// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCurve.h
* PURPOSE: Header file for Curve class.
**********************************************************************/

#ifndef __SMCURVE_H__
#define __SMCURVE_H__

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMOS_TYPES_H_
#include <SmTopoTypes.h>
#endif

#ifndef __SMAOBJECT_H__
#include <SmAObject.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMLOCALSOLVE1D_H__
#include <SmLocalSolve1d.h>
#endif

#ifndef __SMSOLUTIONARRAY_H__
#include <SmSolutionArray.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMPERIODICEXTENT1D_H__
#include <SmPeriodicExtent1d.h>
#endif
 

// forward declarations
class SmDisplayParameters;
class SmEdge ;
// Remove Composites
// class SmCEdge ;
class SmFace ;
class SmBrep ;
class SmEdgeuse ;
class SmSolutionEnd ;
class SmDatabaseIO ;
class SmGfxArraySet ;
class SmPlane ; 
enum  SmObjectCacheType;

// GWC: Not yet ready to use  SM_TOLERANT_XSECT code
#ifdef SM_TOLERANT_XSECT
#undef SM_TOLERANT_XSECT
#endif

/*******************************************************************//**
PURPOSE: The curve is an abstract base class for all curve types.
    Most of the numerical methods are implemented on this class.

NOTES: 
***********************************************************************/
class SM_EXPORT SmCurve : public SmAObject
{
protected:
  ULONG      m_lDim   = SM_UNDEF_ULONG ;  // Dimension of the curve - typically 2 or 3
  SmObject * m_pOwner = NULL ;            // Pointer to an SmEdge object when used by a single Edge.
                                          //        The standalone Edge will point to this curve.
                                          // // Remove Composites
                                          // // Pointer to an SmCEdge object when used by a composite Edge.
                                          // //        The CompositeEdge and each of its sibling Edges
                                          // //        will point to this curve.
                                          // Pointer to SmFace for 2d UVTrimCurves which are not associated
                                          //        with an edge, e.g. a cross-hatch line
                                          // Pointer to SmEdgeuse for 2d UVTrimCurves which are associated
                                          //        with an edge, e.g. a trim-boundary line
                                          // Pointer to SmCurve when used as a Curve GenCurve, ex: SmCrvInVolume
                                          // usually NULL when not used by an Edge.
#ifdef SM_DEBUG_CACHE_H
public:
 ULONG m_lCurveCacheCount ; 
#endif

public:
  // Constructor
  SmCurve(ULONG lDim) ;     

  // copy constructor
  SmCurve(const SmCurve & crSourceCurve) ;

  //destructor
  virtual ~SmCurve() ;
    
  // equality operator
  virtual SmBoolean operator==(const SmCurve & crOther) const ;
     
  // Select a different section of the curve
  virtual SmStatus AdjustSTEPInterval(const SmExtent1d & crNewSTEPInterval) 
                                     { SM_REF1(crNewSTEPInterval) ; SE(SM_ERR); return SM_ERR; }

  SmStatus AnalyticCurveIntersect(const SmExtent1d & crInterval,                // in : line interval in Nurb Domain                          
                                  const SmCurve    & crOtherCurve,              // in : other curve to intersect                              
                                  const SmExtent1d & crOtherInterval,           // in : other curve interval                                  
                                  double             dDistanceTolerance,        // in : Find points where curves are within this 3D distance  
                                  SmBoolean        & rbNeedsMoreIntersections,  // out: TRUE = pass call to general curve/curve intersector   
                                                                                //    : FALSE= intersections found here                       
                                  SmSolutionArray  & rSolutions)                // out: solutions                                             
                                 const ;

  // Approximate a curve with a B-Spline using a hermite based approximation algorithm.
  virtual SmStatus ApproximateCurve(const SmContext        & crContext,                           // in : memory context for new object                                                             
                                    const SmTArray<double> & crBreakParams,                       // in : input curve params exactly interpolated by approx curve.                                  
                                                                                                  //    : Always representing Interval end points                                                   
                                                                                                  //    : and commonly also representing orig curve discontinuities.                                
                                    double                   d3DApproxTol,                        // in : Max dist allowed between approx and original curves.                                      
                                    double                 & rdMaxGap3d,                          // out: Actual Max dist between approx and original curves.                                       
                                                                                                  //    : note: value is not exact, only based on sampling.                                         
                                    SmBSplineCurve        *& rpNewBSplineCurve,                   // out: the new BSpline (always Dim==3)                                                           
                                    SmBoolean                bOptCreateAnalytics = TRUE,          // in : TRUE = create derived type analytics (SmLine, SmCircle, ...) when possible                
                                                                                                  //    : FALSE= don't, default:[TRUE]                                                              
                                    SmBoolean                bOptMatchParameterization = FALSE,   // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values     
                                                                                                  //    : FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)                      
                                                                                                  //    : default:[FALSE], FALSE produces lower control point count curves for slightly more cost.  
                                    SmBoolean                bCopyBSplines = FALSE)               // in : TRUE = if BSplineCurve or derived from BSplineCurve, just copy it (preserves Crv_TYPE and CrvParams)                                 
                                                                                                  //    : FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated  
                                   const ;                                                              
                                                                                                        
  // note: default of bOptCreateAnalytics = TRUE gets overridden if USE_ANALYTICS is not set

  // Approximate to tol this curve constrained to trim curves and trim points while minimizing control point count
  virtual SmStatus ApproximateConstrainedCurve(const SmContext     & crContext,                   // in : context for new object construction                                              
                                               double                dApproxTol,                  // in : maximum allowed distance between Approx and original curves                      
                                               const SmExtent1d    & crInterval,                  // in : Curve interval to approximate                                                    
                                               double              & dAchievedTol,                // out: actual max distance between approx and original curves                           
                                               SmBSplineCurve     *& rpNewBSplineCurve,           // out: The new curve when successful, else NULL.                                        
                                                                                                  //    : expected to be NULL on input - otherwise can cause a memory leak                 
                                               SmTArray<SmPoint3d> * pOpt3DTrimPoints=NULL,       // in : optional array of 3d Point constraints, NULL to ignore                           
                                               SmTArray<double>    * pOptTParamTrimPoints=NULL) ; // in : associated curve TParam values made close to each 3DTrimPoint, NULL to ignore    
                                             
  // return polygonal length between equally domain spaced SamplePoints
  virtual double ApproximateLength(const SmExtent1d & crInterval,              // in : curve interval to query                                                
                                   ULONG              lSampleCnt,              // in : number of smp pts including end points, 5 is a reasonable number       
                                   double           * pOptUVTurnAngDeg = NULL) // out: 2d Curves only. Optional UV Space turning angle. 0.0 for 3dCurves      
                                  const ;
                                  
  // bounding box based on fine tessellation sample points
  //   SmBSplineCurve uses controlPolygon of containing spans
  virtual SmStatus CalculateBoundingBox(const SmExtent1d & crInterval,          // in : target interval of interest - this is used                                  
                                        SmExtent3d       * pNormalBox = NULL,   // out: Axis alligned box, NULL to ignore, default:[NULL]                           
                                        SmPseudoBox      * pPseudoBox = NULL,   // out: Non-axis aligned box, NULL to ignore, default:[NULL]                        
                                        SmPolarBox       * pPolarBox = NULL,    // out: Curve tangent vector field bounding box, NULL to ignore, default:[NULL]     
                                        SmBoolean          bExpandBox = TRUE)   // in : TRUE = expand BBoxes prior to return                                        
                                                                                //    : FALSE= don't                                                                
                                       const ;                                  

  // Calculate the precise bounding box.
  virtual SmStatus CalculateTightBoundingBox(const SmExtent1d & crInterval,        // in : target interval of interest - this is used   
                                             SmExtent3d       * pNormalBox = NULL) // out: Axis alligned box                            
                                            const ;
    
  // Calculate the precise pseudo bounding box given 3 independent unit basis directions
  virtual SmStatus CalculateTightPseudoBox(const SmExtent1d & crInterval,      // in : target interval of interest - this is used     
                                           SmVector3d const & crUnitBasis0,    // in : unit basis 0 of pseudo box                     
                                           SmVector3d const & crUnitBasis1,    // in : unit basis 1 of pseudo box                     
                                           SmVector3d const & crUnitBasis2,    // in : unit basis 2 of pseudo box                     
                                           SmPseudoBox      & rPseudoBox)      // out: basis alligned pseudo box                      
                                          const ;
    
  // polar bounding box of curve's tangents based even paramStep point sampling 
  SmStatus ApproximatePolarBox(const SmExtent1d & crInterval,        // in : target interval of interest - this is used                
                               SmPolarBox       & rPolarBox,         // out: Surface normal vector field bounding box                  
                               ULONG              lPointCount = 5)   // in : number of sample points                                   
                              const ;                                //    :(incremented by 1 to make it odd when it is given as even) 

  // used by SmCacheMgr::GetOrCreateObjectCache()
  // create a new or fetch an existing ObjectCache - place ObjectCache in m_pCacheObj
  SmStatus CacheMakeOrValidate(SmObjectCacheType   eObjectCacheType,     // in : oneof SM_OC_CURVE                                
                                                                         //              SM_OC_SURFACE                              
                                                                         //              SM_OC_TRIMSRF                              
                                                                         //              SM_OC_BREP                                 
                               const SmAObject   * pObject,              // in : target object                                    
                               SmCacheObj        * pOldCache,            // in : existing target Object's ObjectCache or NULL     
                               SmCacheObj       *& rpNewCache)           // out: ptr to target object's ObjectCache                 
                              const ;

  virtual SmStatus CalculateContinuities (SmContinuityType           & reMinContinuityInCurve,  // out: min continuity of all internal knots      
                                          SmTArray<SmContinuityType> & rContinuitiesAtKnots,          // out: continuity at every knot value for curve  
                                          double dContinuityAngleTol = SM_CONTINUITY_ANGLE) const     // in :                                           
                                        { SM_REF3(reMinContinuityInCurve, rContinuitiesAtKnots, dContinuityAngleTol) ; ERR(SM_ERR) ; return SM_ERR; }

  // Compute approximate normal from loop of (nearly) planar curves. 
  static SmStatus  ComputeCrvsNormal(SmTArray <SmCurve*> & cr3DCurves,        // in : Input array of planar curves   
                                     SmVector3d          & crCrvsNormal) ;    // out: Resulting normal               
    
  virtual SmStatus ConvertTFromNURBSToSTEP(double   dNURBSParam,   // in :      
                                           double & rdSTEPParam)   // out:      
                                          const ;

  virtual SmStatus ConvertTFromSTEPToNURBS(double   dSTEPParam,    // in :    
                                           double & rdNURBSParam)  // out:    
                                          const ;

  SmStatus ConvertIvlFromSTEPToNURBS(const SmExtent1d     & crAnalDomain,  // in :    
                                     SmTArray<SmExtent1d> & rNurbDomain)   // out:    
                                    const ;

  SmStatus ConvertIvlFromNURBSToSTEP(const SmExtent1d & crNurbDomain,   // in :       
                                     SmExtent1d       & rAnalDomain)    // out:       
                                    const ;

  virtual SmStatus ConvertTo2D() { SE_MSG(SM_ERR,_T("Called pure virtual method")); return SM_ERR; }                                                  
  virtual SmStatus ConvertTo3D() { SE_MSG(SM_ERR,_T("Called pure virtual method")); return SM_ERR; }

  // make exact BSpline equivalent curve when possible - when Curve is already a BSpline no new curve is built
  virtual SmStatus MakeExactBSplineIfPossible (SmBSplineCurve *& rpNewBSplineCurve) const 
                                             { rpNewBSplineCurve = NULL ; return(SM_SUCCESS) ; } 

  // make an exact copy of any curve
  virtual SmStatus Copy(const SmContext & crContext,    // in :         
                        SmCurve        *& rpNewCurve)   // out:         
                       const ;

  // when possible make a SmBSpline Copy of an Analytic Curve - else set output to NULL
  virtual SmStatus CopyAnalyticAsNurb(const SmContext & crContext,        // in :      
                                      SmCurve        *& rpNewNurbCurve)   // out:      
                                     const ;

  // when possible make an analytic copy of a BSplineCurve - else make exact copy                          
  virtual SmStatus CreateAnalyticCurve (const SmContext  & crContext,     // in :        
                                        const SmExtent1d & crInterval,       // in :        
                                        SmCurve         *& rpNewCurve) const // out:        
                                      { rpNewCurve = NULL; SM_REF2(crInterval, crContext) ; return SM_SUCCESS; }

  // create mirrored copy of this curve. rpMirrorCurve derived type same as 'this' curve
  virtual SmStatus CreateMirrorCurve (const SmContext & crContext,              // in :     
                                      const SmAxis2Placement & crMirrorPlane,         // in :     
                                      SmCurve               *& rpMirrorCurve) /*const*/   // out:     
                                    { SE(SM_ERR); SM_REF3(crContext, crMirrorPlane, rpMirrorCurve) ; return SM_ERR; }
                                           
  // Create a curve by projecting an existing curve into a plane using either parallel or perspective projection.
  virtual SmStatus CreatePlaneProjection (const SmContext  & crContext,                 // in : context for new object construction                                                 
                                          SmProjectionType   eProjectionType,           // in : oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE                                             
                                          const SmPoint3d  & rProjectionPlanePoint,     // in : Point on target plane                                                               
                                          const SmVector3d & rProjectionPlaneNormal,    // in : Normal of target plane                                                              
                                          const SmVector3d & rProjDirOrCenterOfProj,    // in : When eProjectionType == SM_PT_PARALLEL, vector is projection direction.             
                                                                                        //    : When eProjectionType == SM_PT_PERSPECTIVE, vector is 'eye' point of the projection. 
                                          SmCurve         *& rpProjectedCurve) const    // out:                                                                                     
                                        { SM_REF6(crContext, eProjectionType, rProjectionPlanePoint, rProjectionPlaneNormal, rProjDirOrCenterOfProj, rpProjectedCurve) ;
                                          SE(SM_ERR);
                                          return SM_ERR;
                                        }

  // Split this curve into two pieces editing 'this' into the 1st curve portion and creating rpNewCurve for the 2nd portion.
  SmStatus CreateBySplitCurve(const SmContext & crContext,          // in :   
                              double            dSplitParam,        // in :   
                              SmCurve        *& rpNewCurve) ;      // out:   

  // Interpolate a set of points and vectors given their knot parameters
  static SmStatus CreateByInterpolateWithKnots(const SmContext            & crContext,               // in :       
                                               const SmTArray<SmPoint3d>  & crPointsToInterpolate,   // in :       
                                               const SmTArray<SmVector3d> & crVectorsToInterpolate,  // in :       
                                               const SmTArray<double>     & crKnotValues,            // in :       
                                               SmBSplineCurve            *& rpNewBSplineCurve) ;     // out:       
                                              
  // create a degenerate curve equivalent to a point ( builds linear BSplineCurve with two coincident control points) 
  static SmStatus CreateDegenerateCurve(const SmContext  & crContext,               // in :      
                                        ULONG              lDimensionOfResult,      // in :      
                                        const SmPoint3d  & crPoint,                 // in :      
                                        SmCurve         *& rpNewBSplineCurve) ;   // out:      

  virtual SmStatus CurveMaxDistanceBetween(const SmExtent1d & crInterval,                          // in : interval limit for this curve                                             
                                           const SmCurve    & crOtherCurve,                        // in : other curve to test                                                       
                                           double             dOtherCurveParameterAtStartOfThis,   // in : OtherCurve param mapping to ThisCurve Interval.Min value                  
                                           double             dOtherCurveParameterAtEndOfThis,     // in : OtherCurve param mapping to ThisCurve Interval.Max value                  
                                           ULONG              lNumSamples,                         // in : Min number of samples to take - it measures at least this many points     
                                                                                                   //    : at a uniform spacing on the otherCurve finding the                        
                                                                                                   //    : corresponding ThisCurve closest points.                                   
                                                                                                   //    : If 0 is given it will do its best to perform                              
                                                                                                   //    : a precise measurement and will be much slower.                            
                                           double           * pdOptMaxDistanceNeeded,              // in : Max allowed gap.  Quit searching once this value is exceeded.             
                                                                                                   //    : NULL to ignore.   Never quit search when NULL.                            
                                           double           & rdMaxDistanceFound,                  // out: Set to max gap size seen.                                                 
                                                                                                   //    : a. When lNumSamples == 0 This is the max curve/curve gap.                 
                                                                                                   //    : b. When pdOptMaxDistanceNeeded this is either the max sampled             
                                                                                                   //    :    curve/curve gap which is less than pdOptMaxDistanceNeeded or           
                                                                                                   //    :    the 1st gap seen larger than pdOptMaxDistanceNeeded.                   
                                           double           * pOptCurveT = NULL,                   // out: Curve param for returned MaxDistance Found, NULL to ignore.               
                                           double           * pOptOtherT = NULL,                   // out: OtherCurve param for returned MaxDistance Found, NULL to ignore.          
                                           SmBoolean        * pbOptIncorrectDir = NULL)            // out: corresponding point found in incorrect order                              
                                        
                                          const ;

  virtual SmStatus SurfaceMaxDistanceBetween(const SmExtent1d & crInterval,                          // in : interval limit for this curve                                              
                                             const SmSurface  & crSurf,                              // in : surface to test                                                            
                                             const SmExtent2d & crSurfDomain,                        // in : surface domain of interest                                                 
                                             ULONG              lNumSamples,                         // in : Min number of samples to take.  See Notes above                            
                                             double           * pdOptMaxDistanceNeeded,              // in : Max allowed gap.  See Notes above.                                         
                                             double           & rdMaxDistanceFound,                  // out: Set to max gap size seen.  See Notes above.                                
                                             double           * pOptCurveT = NULL,                   // out: Curve param for returned MaxDistance Found, NULL to ignore.                
                                             SmPoint2d        * pOptSurfParam = NULL)                // out: Surface param for returned MaxDistance Found, NULL to ignore.              
                                            const ;
                                           
  virtual SmStatus CreateNonRationalNurb(const SmContext & crContext,             // in : context for new object construction                                          
                                         double            dThisApproxTol3d,      // in : max distance between input and approx curves                                 
                                         SmBSplineCurve *& rpNewNurbCurve)        // out: newly allocated approx curve (exact copy for non-rational BSPline inputs)    
                                        const ;

  // curve interval copies on positive side of an infinite plane (could be one entire curve copy or any number of trimmed curve copies)
  SmStatus CutWithPlane(const SmContext    & crContext,          // in : context for new object construction                                   
                        const SmPoint3d    & crPlanePoint,       // in : point on infinite plane                                               
                        const SmVector3d   & crPlaneNormal,      // in : Normal to infinite plane (keep positive side curve portions)          
                        SmTArray<SmCurve*> & rCutCurves)         // out: Portions of ThisCurve on pos side of ininfite plane                   
                       const ; 

  virtual SmStatus DropPointSTEP(const SmExtent1d & crInterval,            // in : target curve allowed domain                                                      
                                 const SmPoint3d  & crPointToDrop,         // in : Point to drop to curve                                                           
                                 double             dDistTol,              // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.                     
                                                                           //    : sXSectTol3d = dDistTol ;      SM_SO_INTERSECT                                    
                                                                           //    : sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE                    
                                 const double     * pdOptGuessParameter,   //    : sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT   
                                 SmBoolean        & rbSuccess,             // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()    
                                 double           & rdDroppedParameter,    // out: TRUE = found a drop point                                                        
                                 double           & rdDistanceToCurve)     // out: found drop curve param                                                           
                                const ;

  // find CurvePoint closest (or within tolerance) of given point. Calls DropPointFast on analytics.
  virtual SmStatus DropPoint(const SmExtent1d    & crInterval,                       // in : target curve allowed domain                                                            
                             const SmPoint3d     & crPointToDrop,                    // in : Point to drop to curve                                                                 
                             const SmVector3d    * cpVecToCrvInside,                 // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.  
                                                                                     //    : when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.    
                                                                                     //    : Vec handles ambiguities and makes sure that curves are not just touching at the ends.                                                                      
                             double                dDistTol,                         // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.                           
                                                                                     //    : sXSectTol3d = dDistTol ;      SM_SO_INTERSECT                                          
                                                                                     //    : sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE                          
                                                                                     //    : sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT         
                                                                                     //    : DropPTs kept when DropDist < sXSectTol3d                                               
                             const double        * pOptGuessParam,                   // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()          
                             SmBoolean           & rbSuccess,                        // out: TRUE = found a drop point                                                              
                             double              & rdDroppedParameter,               // out: found drop curve param                                                                 
                             double              & rdDistanceToCurve,                // out: found drop distance                                                                    
                             SmSolverOperationType eOperationType = SM_SO_MINIMIZE)  // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints                                  
                                                                                     //    : SM_SO_INTERSECT= like Minimize but point must be within dDistTol                       
                                                                                     //    : SM_SO_NORMALIZE= exclude nonNormal drops near endPoints                                
                                                                                     // out: Portions of ThisCurve on pos side of ininfite plane                                                      
                            const ;

  // Helper function for DropPoint. try to find a fast way to drop points, (drop vector is either perp to curve or to nearest curve EndPoint)
  virtual SmStatus DropPointFast (const SmExtent1d      & crInterval,              // in : Nurb Domain of curve to search for solutions                                          
                                  SmSolverOperationType   eSolverOperation,        // in : oneof: SM_SO_MINIMIZE =find closest point (more than one for closed curves)           
                                                                                   //    :        to curve or if cpdOptTargetDistance is given                                   
                                                                                   //    :        point within pdOptTargetDistance + dDistanceTolerance.                         
                                                                                   //    :        SM_SO_INTERSECT=find closest point within dDistanceTolerance.                  
                                  const SmPoint3d       & crTestPoint,             // in : target point                                                                          
                                  const SmVector3d      * cpOptInPointingVector,   // in : specifies end (EndTang = cpOptInPointingVector) saved for drops on closed curve seams 
                                  double                  dDistanceTolerance,      // in : Skip Solutions whose drop distance is too far away                                    
                                                                                   //    : operation == MINIMIZE save solution if cpdOptTargetDistance == NULL                   
                                                                                   //    :                       or DropDist < cpdOptTargetDistance + dDistanceTolerance         
                                                                                   //    : operation == INTERSECT save solution if DropDist < dDistanceTolerance                 
                                  const double          * cpdOptTargetDistance,    // in : only used for operation Minimize.  When given                                         
                                                                                   //    : skip solutions whose dropDist > cpdOptTargetDistance + dDistanceTolerance.            
                                                                                   //    : else keep all solutions.                                                              
                                  SmSolutionRequestedType eSolutionRequested,      // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                           
                                  SmSolutionArray       & rSolutions) const        // out: array of problem solutions reported as Curve parameter values                         
                                { SM_REF4(crInterval, eSolverOperation, crTestPoint, cpOptInPointingVector) ;
                                  SM_REF4(dDistanceTolerance, cpdOptTargetDistance, eSolutionRequested, rSolutions) ;
                                  return SM_ERR;
                                }      

  SmStatus EuclidianStepOff(const SmExtent1d & crInterval,       // in : range of curve to investigate                                             
                            double             dStartParam,      // in : initial point                                                             
                            SmOrientType       eStepDirection,   // in : SM_OT_SAME    =search increasing param values                             
                                                                 //    : SM_OT_OPPOSITE=search decreasing param values                             
                            double             dStepOffDistance, // in : desired TgtDist chord:(Curve(StartParam), Curve(EndParam)]                
                            double             d3DTolerance,     // in : Stop solving when reach this accuracy                                     
                            SmBoolean        & bClampped,        // out: TRUE= stopped because search steped out of interval before converging.    
                            double           & rdEndParam)       // out: found point on curve dStepOffDistance from initial point                  
                           const ;

  virtual SmStatus Evaluate (double     dParameter,                   // in : tgt param
                             ULONG      lNumDerivatives,              // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .                                         
                             SmBoolean  bFromLeft,                    // in : if P is on interval boundary                                                     
                                                                      //    : TRUE  = evaluate P in upper interval where P is on the left of the interval      
                                                                      //    : FALSE = evaluate P in lower interval where P is on the right of the interval     
                             SmVector3d aPointAndDerivatives[],       // out: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]                                  
                             SmBoolean  bNonZeroTangents=TRUE)        // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors     
                            const                                     //    : FALSE= return exact tangent values                                               
                                                                      //    : note: Surprisingly TRUE is the common choice because most tangent uses           
                                                                      //    :       are for their direction (Binorm, SurfNorm comps), but when the             
                                                                      //    :       tangent is being used for its magnitude (like an arc-length comp)          
                                                                      //    :       then set this to FALSE.                                                    
                           { SM_REF5(dParameter, lNumDerivatives, bFromLeft, aPointAndDerivatives, bNonZeroTangents) ;
                             SE(SM_ERR);
                             return SM_ERR;
                           }

  virtual SmStatus EvaluatePoint (double dParameter,                  // in : Input parameter on curve     
                                  SmPoint3d & rPoint) const      // out: Resulting point on curve     
                                { SM_REF2(dParameter, rPoint) ; SE(SM_ERR); return SM_ERR; }

  virtual SmStatus EvaluateSTEP(double    dSTEPParameter,            // in : appropriate step parameter for derived class, often angle in degrees or distance    
                                ULONG     lNumDerivatives,           // in :                                                                                     
                                SmBoolean bFromLeft,                 // in : if P is on interval boundary                                                        
                                                                     //    : TRUE  = evaluate P in upper interval where P is on the left of the interval         
                                                                     //    : FALSE = evaluate P in lower interval where P is on the right of the interval        
                                SmVector3d aPointAndDerivatives[],   // out:                                                                                     
                                SmZoneTol3d dZoneTol3d=0.0)          // NotUsed: in :                                                                                     
                               const ;

  virtual SmStatus EvaluateSTEPPoint(double      dSTEPParameter,         // in :    
                                     SmPoint3d & rPoint,                 // out:    
                                     SmZoneTol3d dZoneTol3d=0.0)         // NotUsed: in :    
                                    const ;

  SmStatus EvaluateGeometric(double     dParameter,                    // in : curve parameter to query                                                         
                             ULONG      lNumGeometricVectors,          // in : rtn, 0 = position, 1=tangent, 2=curvature_Normal, 3=Torsion_BiNormal             
                             SmBoolean  bFromLeft,                     // in : if P is on interval boundary                                                     
                                                                       //    : TRUE  = evaluate P in upper interval where P is on the left of the interval      
                                                                       //    : FALSE = evaluate P in lower interval where P is on the right of the interval     
                             SmVector3d aPointAndFrenetFrameVectors[]) // out: a[0] = position                                                                  
                                                                       //    : a[1] = 1st derivative,   Mag = Speed,   direction = tangent vector               
                                                                       //    : a[2] = Curvature Vector, Mag = K,       direction = normal vector                
                                                                       //    : a[3] = Torsion Vector,   Mag = Torsion, direction = binormal vector              
                                                                       //    : note: Osculating circle center: center = pos + RadOfCurv * NormalVector          
                                                                       //    : center = a[0] + a[2]/a[2].LengthSquared(), when a[2].LengthSquared() is not zero.
                            const ;

  SmStatus EvaluateContinuity(double             dThisParam,                                  // in : target parameter to check                                                                    
                              SmBoolean          bThisFromLeft,                               // in : TRUE = evaluate from Left                                                                    
                              const SmCurve    & rOtherCurve,                                 // in : other target curve                                                                           
                              double             dOtherParam,                                 // in : other target parameter to check                                                              
                              SmBoolean          bOtherFromLeft,                              // in : TRUE = evaluate from left                                                                    
                              SmContinuityType & rContinuityType,                             // out: oneof SM_CT_DISCONTINUOUS                                                                    
                                                                                              //    :       SM_CT_C0                                                                               
                                                                                              //    :       SM_CT_G1                                                                               
                                                                                              //    :       SM_CT_G1_G2                                                                            
                                                                                              //    :       SM_CT_G1_G2_G3                                                                         
                                                                                              //    :       SM_CT_C1                                                                               
                                                                                              //    :       SM_CT_C1_G2                                                                            
                                                                                              //    :       SM_CT_C1_G2_G3                                                                         
                                                                                              //    :       SM_CT_C1_C2                                                                            
                                                                                              //    :       SM_CT_C1_C2_G3                                                                         
                                                                                              //    :       SM_CT_C1_C2_C3                                                                         
                              SmVector3d         pOptThisEval[4] = NULL,                      // out: optional [position, 1stDerivative, 2ndDerivative, 3rdDerivative] array for this evaluation   
                              SmVector3d         pOptOtherEval[4] = NULL,                     // out: optional [position, 1stDerivative, 2ndDerivative, 3rdDerivative] array for other evaluation  
                              double             dContinuityAngleTol = SM_CONTINUITY_ANGLE)   // in : max angle between G1 continuity, default:[SM_CONTINUITY_ANGLE]                               
                             const ;

  SmStatus FindParameterAtArcLength(double      dStartParam,            // in : location on curve at which to start               
                                    double      dArcLength,             // in : how far (+/-) from dStartParam location to move   
                                    double    & rdEndParam,             // out: param at dArcLength away from dStartParam         
                                    SmBoolean * pbResultPastEnd = NULL) // out: (optional)                                        
                                   const ;

  virtual ULONG            GetDegree()              const { return 3; }
  virtual ULONG            GetDim()                 const { return m_lDim; }
  virtual SmExtent1d       GetNaturalInterval()     const { ERR(SM_ERR); return SmExtent1d(0,0); }
  virtual SmExtent1d       GetSTEPInterval()        const { return GetNaturalInterval() ; }
  virtual SmExtent1d       GetMaxAnalyticDomain()   const { ERR(SM_ERR); return SmExtent1d(0,0); }
  virtual ULONG            GetNumberNaturalKnots()  const { return 2; }
  virtual ULONG            GetNumberControlPoints() const { return 0; }
  virtual SmObject       * GetOwner()               const { return m_pOwner; }
  virtual SmBSplineCurve * GetRootCurve()           const { return(NULL) ; }

  virtual SmStatus GetKnots(SmTArray<double> & rKnots,                       // out: Unique knot vector                                           
                            SmTArray<ULONG>  * pKnotMultiplicities = NULL,   // out: multiplicity value for each knot                             
                            const SmExtent1d * pOptIvl = NULL)               // in : interval of interest, NULL=Natural Interval, default:[NULL]  
                           const ;

  virtual void GetEnds(SmPoint3d        & rStartPoint,                  // out: Curve StartPoint       
                       SmPoint3d        & rEndPoint,                    // out: Curve EndPoint         
                       double           * pdOptStartW = NULL,           // out: Curve StartWeight      
                       double           * pdOptEndW = NULL)             // out: Curve EndWeight        
                      const ;

  const SmEdge      *GetEdge()       const ; // when crv is owned by edge or when crv is genCurve for a crv owned by an edge, rtn edge
// Remove Composites
  // const SmCEdge     *GetCEdge()      const { return( m_pOwner && m_pOwner->IsKindOf(SmCEdge_TYPE)   ? (SmCEdge*)m_pOwner : NULL) ; }
  const SmFace      *GetFace()       const { return( m_pOwner && m_pOwner->IsKindOf(SmFace_TYPE)    ? (SmFace*)m_pOwner : NULL) ; }
  const SmEdgeuse   *GetEdgeuse()    const { return( m_pOwner && m_pOwner->IsKindOf(SmEdgeuse_TYPE) ? (SmEdgeuse*)m_pOwner : NULL) ; }
  const SmCurve     *GetOwnerCurve() const { return( m_pOwner && m_pOwner->IsKindOf(SmCurve_TYPE)   ? (SmCurve*)m_pOwner : NULL) ; }
  const SmBrep      *GetBrep()       const ;

#ifdef SM_TOLERANT_XSECT  
  SmStatus IntervalCoincidenceChecker(const SmExtent1d  & crInterval,               // in : 'this' curve interval to examine                                     
                                      const SmCurve     & crOtherCurve,             // in : other curve                                                          
                                      const SmExtent1d  & crOtherInterval,          // in : other-curve interval to examine                                      
                                      double              d3dTolerance,             // in : max separation distances                                             
                                      const SmPseudoBox * cpOptPseudoBox,           //    : not yet used - part of new args to make compatible with SimpleCC     
                                      const SmPseudoBox * cpOptPseudoBoxOther,      //    : not yet used - part of new args to make compatible with SimpleCC     
                                      SmBoolean         & rbAdditionalWorkNeeded,   // out: TRUE = may be additional intersections between                       
                                                                                    //    :        the curve.                                                    
                                                                                    //    : FALSE= all of one of the curves has been used in the                 
                                                                                    //    :        coincidence and no more solutions can exist unless            
                                                                                    //    :        there is something strange like a self intersection.          
                                      SmSolutionArray   & rSolutions,               // out: array of coincident segments                                         
                                      SmBoolean           bSnapToEnds = TRUE)       // in : TRUE = snap to ends within tolerance                                 
                                                                                    //    : FALSE= don't snap near end solutions to endPoints                    
                                     const ;

  SmStatus CoincidenceCheckerAndValidate(const SmExtent1d  & crInterval,               // in : 1st curve target interval                                            
                                         const SmCurve     & crOtherCurve,             // in : 2nd target curve                                                     
                                         const SmExtent1d  & crOtherInterval,          // in : 2nd curve target interval                                            
                                         double              d3dTolerance,             // in : Max Distance allowed for solution devaitions                         
                                         const SmPseudoBox * cpOptPseudoBox,           // in : Optional 1st interval BoundingBox                                    
                                         const SmPseudoBox * cpOptPseudoBoxOther,      // in : Optional 2nd interval BoundingBox                                    
                                         SmBoolean         & rbAdditionalWorkNeeded,   // out: TRUE = may be additional intersections between the curve.            
                                                                                       //    : FALSE= all of one of the curves has been used in the                 
                                                                                       //    :        coincidence and no more solutions can exist unless            
                                                                                       //    :        there is something strange like a self intersection.          
                                         SmSolutionArray   & rSolutions,               // out: array of coincident segments                                         
                                         SmBoolean           bSnapToEnds = TRUE)       // in : TRUE = snap to ends within tolerance                                 
                                                                                       //    : FALSE= don't snap near end solutions to endPoints                    
                                        const ;

  SmStatus SimpleCoincidenceChecker(const SmExtent1d  & crInterval,               // in : 1st curve target interval                                            
                                    const SmCurve     & crOtherCurve,             // in : 2nd target curve                                                     
                                    const SmExtent1d  & crOtherInterval,          // in : 2nd curve target interval                                            
                                    double              d3dTolerance,             // in : Max Distance allowed for solution devaitions                         
                                    const SmPseudoBox * cpOptPseudoBox,           // in : Optional 1st interval BoundingBox                                    
                                    const SmPseudoBox * cpOptPseudoBoxOther,      // in : Optional 2nd interval BoundingBox                                    
                                    SmBoolean         & rbAdditionalWorkNeeded,   //    : not yet used - part of new arg list                                  
                                    SmSolutionArray   & rSolutions,               //    : not yet used - part of new arg list                                  
                                    SmBoolean           bSnapToEnds = TRUE)       //    : not yet used - part of new arg list                                  
                                   const ;

  SmStatus GlobalCoincidenceChecker(const SmExtent1d  & crInterval,               // in : 'this' curve interval to examine                                      
                                    const SmCurve     & crOtherCurve,             // in : other curve                                                           
                                    const SmExtent1d  & crOtherInterval,          // in : other-curve interval to examine                                       
                                    double              d3dTolerance,             // in : max separation distances                                              
                                    const SmPseudoBox * cpOptPseudoBox,           //    : not yet used - part of new args to make compatible with SimpleCC      
                                    const SmPseudoBox * cpOptPseudoBoxOther,      //    : not yet used - part of new args to make compatible with SimpleCC      
                                    SmBoolean         & rbAdditionalWorkNeeded,   // out: TRUE = may be additional intersections between the curve.             
                                                                                  //    : FALSE= all of one of the curves has been used in the                  
                                                                                  //    :        coincidence and no more solutions can exist unless             
                                                                                  //    :        there is something strange like a self intersection.           
                                    SmSolutionArray   & rSolutions,               // out: array of coincident segments                                          
                                    SmBoolean           bSnapToEnds = TRUE)       // in : TRUE = snap to ends within tolerance                                  
                                                                                  //    : FALSE= don't snap near end solutions to endPoints                     
                                   const ;
                     
#else  // NO SM_TOLERANT_XSECT

  SmStatus SimpleCoincidenceChecker(const SmExtent1d  & crInterval,               // in : 1st curve target interval                                             
                                    const SmCurve     & crOtherCurve,             // in : 2nd target curve                                                      
                                    const SmExtent1d  & crOtherInterval,          // in : 2nd curve target interval                                             
                                    double              d3dTolerance,             // in : Max Distance allowed for solution deviations                          
                                    const SmPseudoBox * cpOptPseudoBox,           // in : Optional 1st interval BoundingBox                                     
                                    const SmPseudoBox * cpOptPseudoBoxOther,      // in : Optional 2nd interval BoundingBox                                     
                                    ULONG             & rlNumFound,               // out: number of solutions                                                   
                                    SmSolution          aSolutions[2])            // out: Solutions                                                             
                                   const ;

  SmStatus GlobalCoincidenceChecker(const SmExtent1d  & crInterval,               // in : 'this' curve interval to examine                                       
                                    const SmCurve     & crOtherCurve,             // in : other curve                                                            
                                    const SmExtent1d  & crOtherInterval,          // in : other-curve interval to examine                                        
                                    double              d3dTolerance,             // in : max separation distances                                               
                                    SmBoolean         & rbAdditionalWorkNeeded,   // out: TRUE = may be additional intersections between the curve.              
                                                                                  //    : FALSE= all of one of the curves has been used in the                   
                                                                                  //    :        coincidence and no more solutions can exist unless              
                                                                                  //    :        there is something strange like a self intersection.            
                                    SmSolutionArray   & rSolutions,               // out: array of coincident segments                                           
                                    SmBoolean           bSnapToEnds = TRUE)       // in : TRUE = snap to ends within tolerance                                   
                                                                                  //    : FALSE= don't snap near end solutions to endPoints                      
                                   const ;
                     
#endif // NO SM_TOLERANT_XSECT

  // See if intersecting coincident range is really a tangent intersection point 
  SmStatus ClassifyIntersectingRangeSolution(const SmCurve    & crOtherCurve,     // in : Curve being intersected                                  
                                             const SmSolution & crSol,            // in : A ThisCurve/Otherurve intersection solution              
                                             double             dTol3D ,          // NotUsed: in : intersection 3d tolerance                                
                                             SmBoolean        & bIsDegenerate,    // out: TRUE = Intersection is a Tangent Point Intersection      
                                                                                  //    : FALSE= Intersection is a coincident segment              
                                                                                  // out: set to a Point Tangent Solution when                     
                                             SmSolutionEnd    & rSolPoint)        //    : bIsDegenerate == TRUE                                    
                                                                                  //    : else left unmodified.                                    
                                            
                                            const ;

  virtual SmStatus GlobalPropertyAnalysis(const SmExtent1d    & crInterval,             // in : ThisCurve target interval                                                 
                                          SmCurvePropertyType   eCurveProperty,         // in : The property of the curve to extract or find on the curve                 
                                          const double        * cpdPropertyValue,       // in : Value used to specify a particular property.                              
                                          const SmVector3d    * cpOptVectors,           // in : Vectors or points used to define a particular property.                   
                                                                                        //    : If there is a point, that should be the first value in the array          
                                                                                        //    : If there is a normal (projection), it will come before all other vectors  
                                          double                dDistanceTolerance,     // in : 3D Tolerance used to determine when two answers are equivalent            
                                          SmSolutionArray     & rSolutions,             // out:                                                                           
                                          SmTArray<SmPoint3d> * pOptCombPoints = NULL,  // out: opt check comb: curve sample points                                           
                                          SmTArray<SmPoint3d> * pOptCombVecs = NULL)    // out: opt check comb: tines - lengths proportional to property being zeroed.        
                                         const ;

  virtual SmStatus GlobalCurveIntersect(const SmExtent1d & crInterval,                            // in : thisCurve's interval for intersection                           
                                        const SmCurve    & crOtherCurve,                          // in : target OtherCurve                                               
                                        const SmExtent1d & crOtherInterval,                       // in : OtherCurve's interval for intersection                          
                                        double             dDistanceTolerance,                    // in : max 3d distance between intersecting points                     
                                        SmSolutionArray  & rSolutions,                            // out: Array of Found Solutions: sSol.m_vStart[0] = thisCurve param    
                                                                                                  //    :                           sSol.m_vStart[1] = otherCurve param   
                                        SmBoolean          bSkipGlobalCoincidenceCheck = FALSE)   // in : TRUE = Skip global coincidence check, default:[FALSE]           
                                       const ;
    
  virtual SmStatus GlobalCurveSelfIntersect(const SmExtent1d & crInterval,            // in : thisCurve's interval for intersection                              
                                            double             dDistanceTolerance,    // in : max 3d distance between intersecting points                        
                                            SmSolutionArray  & rSolutions)            // out: Array of Found Solutions: sSol.m_vStart[0] = thisCurve param       
                                                                                      //    :                           sSol.m_vStart[1] = otherCurve param      
                                           const ;                                    
                                              
  virtual SmStatus GlobalCurveSolve(const SmExtent1d      & crInterval,             // in : Interval on curve where solution will be searched for                
                                    const SmCurve         & crOtherCurve,           // in : target other curve                                                   
                                    const SmExtent1d      & crOtherInterval,        // in : Interval on other curve where solution will be searched for          
                                    SmSolverOperationType   eSolverOperation,       // in : Which solver operation to perform                                    
                                    double                  dDistTol,               // in : Basically the distance tolerance sets up a range for the             
                                                                                    //    : distance measurements where additional answers may exist.            
                                                                                    //    : For example if the distance between two local minima/maxima          
                                                                                    //    : is less than this tolerance, both answers will be returned.          
                                    const double          * cpdOptTargetDistance,   // in : If not NULL it will either be the target distance for the            
                                                                                    //    : SM_SO_AT_DISTANCE operation, or it will be the corresponding         
                                                                                    //    : limit to a minimize/maximize operation.  In other words, it will     
                                                                                    //    : ask the solver to find a minimum value only if it is less than       
                                                                                    //    : the target distance or maximum value only if it is greater than      
                                                                                    //    : the target distance                                                  
                                    const SmVector3d      * cpOptVectors,           // in : See SmSolverOperationType documentation for corresponding meaning    
                                                                                    //    : of these vectors.                                                    
                                    SmSolutionRequestedType eSolutionRequested,     // in : oneof SM_SR_SINGLE, SM_SR_ALL, SM_SR_FIND_AMBIGUITIES, SM_SR_NODES   
                                    SmSolutionArray       & rSolutions)             // out:                                                                      
                                   const ;
    
  virtual SmStatus GlobalPointSolve(const SmExtent1d      & crInterval,             // in : search curve interval                                                                 
                                    SmSolverOperationType   eSolverOperation,       // in : which solver operation to perform                                                     
                                    const SmPoint3d       & crTestPoint,            // in : Euclidean target point                                                                
                                    double                  dDistTol,               // in : Basically the distance tolerance sets up a range for the                              
                                                                                    //    : distance measurements where additional answers may exist.                             
                                                                                    //    : For example if the distance between two local minima/maxima                           
                                                                                    //    : is greater than this tolerance, both answers will be returned.                        
                                                                                    //    : For DropPointFast: Skip Solutions whose drop distance is too far away                 
                                                                                    //    : operation == MINIMIZE save solution if cpdOptTargetDistance == NULL                   
                                                                                    //    :                   or DropDist < cpdOptTargetDistance + dDistTol                       
                                                                                    //    : operation == INTERSECT save solution if DropDist < dDistTol                           
                                    const double          * cpdOptTargetDistance,   // in : If not NULL it will be:                                                               
                                                                                    //    : SM_SO_AT_DISTANCE       = target distance,                                            
                                                                                    //    : SM_SO_MINIMIZE/MAXIMIZE = distance limit.                                             
                                                                                    //    : For example, to find a minimum value only if it less than                             
                                                                                    //    : the target distance or maximum value only if it is greater than the target distance   
                                    const SmVector3d      * cpOptVectors,           // in : Vectors used in some of the solvers.                                                  
                                                                                    //    : SM_SO_RAYFIRE,                  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE                     
                                                                                    //    : SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_DIRECTED_MINIMIZE                               
                                                                                    //    : SM_SO_DIRECTED_MAXIMIZE,        SM_SO_PROJECTED_MINIMIZE                              
                                                                                    //    : SM_SO_PROJECTED_MAXIMIZE                                                              
                                                                                    //    : SM_SO_PROJECTED_TANGENT_THROUGH_POINT                                                 
                                    SmSolutionRequestedType eSolutionRequested,     // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                           
                                    SmSolutionArray       & rSolutions)             // out: array of problem solutions reported as curve parameter values                         
                                   const ;

  virtual SmStatus GlobalPointSolveSTEP(const SmExtent1d      & crSTEPInterval,         // in : Interval on curve where solution will be searched for                
                                        SmSolverOperationType   eSolverOperation,       // in : Which solver operation to perform                                    
                                        const SmPoint3d       & crTestPoint,            // in : Point to be used as other object in solver                           
                                        double dDistTol,                                // in : Basically the distance tolerance sets up a range for the             
                                                                                        //    : distance measurements where additional answers may exist.            
                                                                                        //    : For example if the distance between two local minima/maxima          
                                                                                        //    : is less than this tolerance, both answers will be returned.          
                                        const double          * cpdOptTargetDistance,   // in : If not NULL it will either be the target distance for the            
                                                                                        //    : SM_SO_AT_DISTANCE operation, or it will be the corresponding         
                                                                                        //    : limit to a minimize/maximize operation.  In otherwords, it will      
                                                                                        //    : ask the solver to find a minimum value only if it less than          
                                                                                        //    : the target distance or maximum value only if it is greater than      
                                                                                        //    : the target distance                                                  
                                        const SmVector3d      * cpOptVectors,           // in : Vectors used in some of the solvers.                                 
                                        SmSolutionRequestedType eSolutionRequested,     // in :                                                                      
                                        SmSolutionArray       & rSolutions) ;           // out:

  // currently only BSplines - return param pairs bounding intervals of zero 1st derivative values
  virtual SmStatus GetNullSegments(SmTArray<double> & rSegments) const { rSegments.ReSet(); return SM_SUCCESS; }

  virtual SmBoolean HasNullSegments() const { return FALSE; }

  virtual SmStatus IntersectWithEllipse(const SmExtent1d & crInterval,                  // in : line interval                                          
                                        const SmEllipse  & crOtherCurve,                // in : other curve to intersect                               
                                        const SmExtent1d & crOtherInterval,             // in : other curve interval                                   
                                        double             dDistanceTolerance,          // in : Find points where curves are within this 3D distance   
                                        SmBoolean        & rbNeedsMoreIntersections,    // out: TRUE = pass call to general curve/curve intersector    
                                        SmSolutionArray  & rSolutions)                  //    : FALSE= intersections found here                        
                                       const ;                                          // out: solutions                                              

  virtual SmStatus IntersectWithLine(const SmExtent1d & crInterval,                  // in : curve interval in Nurb Domain                           
                                     const SmLine     & crOtherCurve,                // in : other line to intersect                                 
                                     const SmExtent1d & crOtherInterval,             // in : other line interval in Nurb Domain                      
                                     double             dDistanceTolerance,          // in : Find points where curves are within this 3D distance    
                                     SmBoolean        & rbNeedsMoreIntersections,    // out: TRUE = pass call to general curve/curve intersector     
                                                                                     //        FALSE= intersections found here                         
                                     SmSolutionArray  & rSolutions)                  // out: solutions: sSol[ii].m_vStart[0] = thisCurve param       
                                                                                     //                 sSol[ii].m_vStart[1] = Line param              
                                    const ;                                                 
                                   
  virtual SmBoolean       IsAnalytic      ()                                                            const { return FALSE; } // default
  virtual SmBoolean       IsArc(ULONG, double, SmAxis2Placement&, double&, double&, double&) const ;                            
  virtual SmBoolean       IsBounded       ()                                                            const { return TRUE ; } // TRUE = BSplineCurves and finite analytic curves,
                                                                                                                                // FALSE= semi-bounded or infinite analytic curves
  virtual SmBoolean       IsClosed        (const SmExtent1d & crInterval, double dTolerance=0.0)        const ; 
  virtual SmBoolean       IsDegenerate    (double d3DTol=SM_EFF_ZERO, const SmExtent1d *pInterval=NULL) const ; 
  virtual SmSurfParamType IsDomainBoundary(const SmExtent2d &rDomain, double dTol=SM_EFF_ZERO)          const ; 
  virtual SmBoolean       IsLine          (ULONG        lNumSamples,                                            // in : number of sample points to generate and test                        
                                           double       dTol,                                                   // in : max deviation allowed for any samplePoint from ideal line           
                                           SmPoint3d  & rLinePoint,                                             // out: line's base point                                                   
                                           SmVector3d & rLineVector)                                    const ;   // out: line's direction vector                                             
  SmBoolean               IsLinear        (double      dTol3d = SM_EFF_ZERO,                                      // in : max allowed deviation from Line                       
                                           SmPoint3d  *pOptLinePoint = NULL,                                      // out: Point on Line                                         
                                           SmVector3d *pOptLineTangent = NULL,                                    // out: Unit Curve tangent                                    
                                           double     *pOptActualDeviation = NULL)                      const ; // out: Max deviation from linearity seen                     
  SmBoolean               IsPlanar        (double      dTol3d = SM_EFF_ZERO,                                    // in : max allowed deviation from Plane                    
                                           SmPoint3d  *pOptPlanePoint = NULL,                                   // out: Point on Plane                                      
                                           SmVector3d *pOptPlaneNormal = NULL,                                  // out: Unit Surface Normal                                 
                                           double     *pOptActualDeviation = NULL)                      const ;   // out: Max ControlPoint/Plane distance seen                
  SmBoolean               AreCoPlanar     (const SmTArray<SmCurve *> & crCurves,                                  // in : set of curves to check                                   
                                           double                      dTol3d = SM_EFF_ZERO,                      // in : max allowed deviation from Plane                         
                                           SmPoint3d                 * pOptPlanePoint = NULL,                     // out: Point on Plane                                           
                                           SmVector3d                * pOptPlaneNormal = NULL,                    // out: Unit Surface Normal                                      
                                           double                    * pOptActualDeviation = NULL)     const ;  // out: Max ControlPoint/Plane distance seen when return is TRUE 
  virtual SmStatus        IsOnPlane       (const SmPoint3d  & crPlanePointArg,                                  // in : point on plane                                                      
                                           const SmVector3d & crPlaneNormalArg,                                 // in : plane normal direction                                              
                                           double             d3DToleranceArg,                                  // out: max allowed deviation between curve and plane                       
                                           SmBoolean        & rbIsOnPlaneArg,                                   // out: TRUE = all curve points are within tol of plane                     
                                           double           & rdMaxDistToPlaneArg,                              // out: max distance between the curve and the given plane                  
                                           const SmExtent1d * pOptInterval = NULL)                     const ;    // in : Curve Ivl to examine, NULL to use Natural Interval, default:[NULL]  
  SmBoolean               AreOnPlane      (const SmTArray<SmCurve *>    & crCurves,                               // in : set of curves to check                                              
                                           double                         dTol3d = SM_EFF_ZERO,                   // in : max allowed deviation from Plane                                    
                                           const SmPoint3d              & crPlanePoint = NULL,                    // out: Point on Plane                                                      
                                           const SmVector3d             & crPlaneNormal = NULL,                   // out: Unit Surface Normal                                                 
                                           double                       * pOptActualDeviation = NULL,             // out: Max ControlPoint/Plane distance seen when return is TRUE            
                                           const SmTArray<SmExtent1d>   * pOptIntervals = NULL)         const ; // in : optional assoicated crCurves intervals, NULL = use NaturalInterval  
  SmBoolean                HasPlaneCrossings                   (SmTArray<SmPlane *> sPlanes)            const ; 
  SmBoolean                IsOnPositiveSideOfPlanes            (SmTArray<SmVector3d> &rPlanePoints,               // in : array of points on plane
                                                                SmTArray<SmVector3d> &rPlaneNormals)    const ; // in : associated array of plane normals
  virtual SmBoolean        HasRepeatedEndControlPoints         (double dTol=SM_ZONE_TOL_3D)             const { SM_REF1(dTol) ; return FALSE ; } 
  virtual SmBoolean        HasInternalPole                     (double dTol=SM_ZONE_TOL_3D)             const { SM_REF1(dTol) ; return FALSE ; }
  virtual SmBoolean        HasKnotMultiplicityGreaterThanDegree()                                       const { return FALSE ; } 
  virtual SmBoolean        IsPeriodic                          (const SmExtent1d & crInterval)          const ;
  virtual SmBoolean        IsRational                          ()                                       const { SE_MSG(SM_ERR,_T("Called pure virtual method")); return FALSE; }

  virtual SmStatus Length(const SmExtent1d & crInterval,          // in :       
                          double             dDesiredAccuracy,    // in :       
                          double           & rdLength)            // out:       
                         const ;

  virtual SmStatus LocalPropertyAnalysis(const SmExtent1d   & crInterval,        // in : target interval                                                    
                                         SmCurvePropertyType  eCurveProperty,    // in : specify Curve Property to find per above table                     
                                         double               dGuessParameter,   // in : Start parameter of curve for local iteration.                      
                                         const double       * cpdPropertyValue,  // in : Constant for some curve properties                                 
                                                                                 //    :  SM_CP_RADIUS_OF_CURVATURE     = Desired Radius                    
                                                                                 //    :  SM_CP_FIRST_DERIVATIVE_LENGTH = Desired Length                    
                                                                                 //    :  SM_CP_PLANE_INTERSECTION      = D of ax+by+cd-D=0                 
                                         const SmVector3d   * cpOptVectors,      // in : Vectors and/or Points for some curve properties                    
                                                                                 //    : SM_CP_SILHOUETTE_VECTOR       : [0] = vector                       
                                                                                 //    : SM_CP_SILHOUETTE_POINT        : [0] = point                        
                                                                                 //    : SM_CP_PLANE_INTERSECTION      : [0] = planeNormal                  
                                                                                 //    : SM_CP_CYLINDER_INTERSECTION   : [0] = CylCenter                    
                                                                                 //    :                                 [1] = CylAxis                      
                                                                                 //    : SM_CP_MINIMIZE_ANGLE_TO_PLANE : [0] = PlanePoint                   
                                                                                 //    :                                 [1] = planeNormal                  
                                                                                 //    :                                 [2] = ???                          
                                                                                 //    : SM_CP_MINIMIZE_CCW_ANGLE      : [0] = PlanePoint                   
                                                                                 //    :                                 [1] = planeNormal                  
                                                                                 //    : SM_CP_MINIMIZE_DIRECTED_ANGLE : [0] = PlanePoint                   
                                                                                 //    :                                 [1] = planeNormal                  
                                                                                 //    :                                 [2] = ???                          
                                                                                 //    : SM_CP_PROJECTED_POINT_MINIMIZE: [0] = PlanePoint                   
                                                                                 //    :                                 [1] = planeNormal                  
                                                                                 //    : SM_CP_PROJECTED_POINT_MAXIMIZE: [0] = PlanePoint                   
                                                                                 //    :                                 [1] = planeNormal                  
                                                                                 //    : SM_SO_PROJECTED_TANGENT_THROUGH_POINT: [0] = PlanePoint            
                                                                                 //    :                                        [1] = planeNormal           
                                         SmBoolean          & rbFoundAnswer,     // out: TRUE = PropertyValue within tolerance of Zero                      
                                                                                 //    : for cases: SM_CP_INFLECTION_POINTS                                 
                                                                                 //    :            SM_CP_RADIUS_OF_CURVATURE                               
                                                                                 //    :            SM_CP_FIRST_DERIVATIVE_LENGTH                           
                                                                                 //    : FALSE otherwise.                                                   
                                         SmSolution         & rSolution)         // out: specification of a found solution                                  
                                        const ;

  virtual SmStatus LocalCurveIntersect(const SmExtent1d & crInterval,                // in : ThisCurve intersection limits                               
                                       const SmCurve     & crOtherCurve,             // in : other curve                                                 
                                       const SmExtent1d  & crOtherInterval,          // in : other curve intersection limits                             
                                       double              dDistTol,                 // in : Passed to SmLocalSolveNd::SolveIt() and                     
                                                                                     //    : LocalCurveSolve()                                           
                                       double              dMyGuessParameter,        // in : ThisCurve GuessPoint                                        
                                       double              dOtherGuessParameter,     // in : OtherCurve GuessPoint                                       
                                       SmBoolean         & rbFoundIntersection,      // out: TRUE = curves intersect                                     
                                       double            & rdMyParameter,            // out: ThisCurve intersection point                                
                                       double            & rdOtherParameter,         // out: OtherCurve intersection point                               
                                       double            & rdDeviation)              // out: Intersection Gap Vector size                                
                                      const ;
 
  virtual SmStatus LocalCurveSolve(const SmExtent1d     & crInterval,            // in : this Curve interval to query                                                  
                                   const SmCurve        & crOtherCurve,          // in : other Curve                                                                   
                                   const SmExtent1d     & crOtherInterval,       // in : Other Curve interval to query                                                 
                                   SmSolverOperationType  eSolverOperation,      // in : solver operation to perform                                                   
                                   double                 dDistTol3d,            // in : min dist between two distinct answers                                         
                                   const double         * cpdOptTargetDistance,  // in : for SM_SO_AT_DISTANCE = target distance,                                      
                                                                                 //    : for min and max ops = limit distance, i.e. only find                          
                                                                                 //    : sols whose mins are less or whose maxs are more than this limit dist.         
                                   const SmVector3d     * cpOptVectors,          // in : see SmSolverOperationType definition for uses for each eSolverOperation:      
                                                                                 //    : For projected min/max, 1st vector = a projection or reference plane normal.   
                                                                                 //    : For directed min/max,  1st vector = normal to the reference plane,            
                                                                                 //    :                      2nd vector = direction vector.                           
                                                                                 //    : For rot projected,     1st vector = pt on rotation axis                       
                                                                                 //    :                      2nd vector = dir of rotation axis                        
                                                                                 //    : In these cases, the reference plane is a theoretical plane into which         
                                                                                 //    : the curves are theoretically projected prior to solving the                   
                                                                                 //    : minimization/maximization.  In reality we do not need to project              
                                                                                 //    : the curves we simply build the projection into the solver.                    
                                   double                 dMyGuessParameter,     // in : Starting parameter on curve for iterations - must be in crInterval            
                                   double                 dOtherGuessParameter,  // in : Starting parameter on other curve for iterations - must be in crOtherInterval 
                                   SmBoolean            & rbFoundAnswer,         // out: TRUE = found a solution, FALSE = didn't                                       
                                   SmSolution           & rSolution)             // out: the found solution                                                            
                                  const ;
                                                         
  virtual SmStatus LocalPointSolve(const SmExtent1d     & crInterval,              // in : search interval                                                             
                                   SmSolverOperationType  eSolverOperation,        // in : specify specific operation to optimize                                      
                                   const SmPoint3d      & crTestPoint,             // in : point specializing this search                                              
                                   const double         * cpdOptDistanceTolerance, // in : Opt Max allowed solution distance (NULL to ignore)                          
                                                                                   //    : for SM_SO_RAYFIRE                                                           
                                                                                   //    : SM_SO_3D_SIGNED_DIRECTED_MINIMIZE                                           
                                   const double         * cpdOptTargetDistance,    // in : required curve/TestPoint desired dist (NULL when not used                   
                                                                                   //    : for SM_SO_AT_DISTANCE                                                       
                                   const SmPoint3d      * cpOptVectors,            // in : required Vector direction (NULL when not used)                              
                                                                                   //    : for SM_SO_RAYFIRE                                                           
                                                                                   //    : SM_SO_3D_SIGNED_DIRECTED_MINIMIZE                                           
                                   double                 dGuessParameter,         // in : search starting parameter                                                   
                                   SmBoolean            & rbFoundAnswer,           // out: TRUE=converged,FALSE=didn't                                                 
                                   SmSolution           & rSolution)               // out: solution container for solver                                               
                                  const ;

  virtual SmStatus MakeNurb() { SE_MSG(SM_ERR,_T("Called pure virtual method")); return SM_ERR; }

  virtual void Notify (SmNotifyOperation eNotifyOperation,   //       event                | caller      |  pData1  | pData2                | pData3                     
                       SmObject        * pData1,             //----------------------------+-------------+----------+-----------------------+-------------------------   
                       SmObject        * pData2,             // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL   
                       SmObject        * pData3) ;           // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                     
                                                             // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                       
                                                             // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                                             // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB                      
                                                             // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL    
                                                             // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL            
                                                             // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL           
                                                             // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL                       
                                                             // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL    
                                                             // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                       
                                                             // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                       
                                                             // SM_NO_SPLIT                | SplitObj    | Child1   | Child2                | SplitObj's Owner or NULL   
                                                             // SM_NO_MERGE                | MergeObj    | OrigObj1 | OrigObj2              | MergeObj's Owner or NULL   
                                                             // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg         
                                                             // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                      
                      
  // reverse curve parameterization while preserving its NaturalInterval range
  // e.g. NewPoint(sNatIvl.Min) == OldPoint(sNatIvl.Max) with negated tangents.
  virtual SmStatus ReverseParameterization (const SmExtent1d & crOldInterval, // in : interval of interest, may be a subset of natural interval
                                            SmExtent1d       & rNewInterval)  // out: new domain for crOldInterval on modified curve   
                                          { SM_REF2(crOldInterval, rNewInterval) ; SE(SM_ERR); return SM_ERR; }

  // Change NURB parameterization of curve to given extent range - no Analytic-STEP range change
  virtual SmStatus EditParameterization (const SmExtent1d & crNewParameterization,
                                         SmBoolean bNotify=TRUE)
                                       { SM_REF2(crNewParameterization, bNotify) ; SE(SM_ERR); return SM_ERR; }

  SmObject *SetOwner(SmObject *pNewOwner); // eff: set owner value and return previous value
    
  // snap to nearest knot within dUVSnapTolerance distance
  SmStatus SnapToKnots(double   dOriginalParameter,  // in : input Curve param to snap to nearby knot values                            
                       double   dUVSnapTolerance,    // in : Max Param snap dist, large value forces snap to nearest existing knot
                       double & rdSnappedParameter)  // out: after snapping ParamVal (same as input Param when no snapping occurred)
                      const ;

  virtual SmStatus STEPInversion (const SmPoint3d      & crPointOnCurve,        // in : Target Point                                              
                                  double               & rdAnalyticParameter,     // out: STEPParameter closest to Target Point                     
                                  SmCurveLocationType  * pOptLoc = NULL,          // out: Point's classification to curve; NULL to ignore           
                                  SmZoneTol3d            dZoneTol3d = 0.0) const // in : Unused                                                    
                                { SM_REF4(crPointOnCurve, rdAnalyticParameter, pOptLoc, dZoneTol3d) ;
                                  SE(SM_ERR);
                                  return SM_ERR;
                                }

  virtual SmStatus Tessellate(const SmExtent1d     & crInterval,                // in : interval to tessellate                                  
                              double                 dChordHeightTolerance,     // in : max height/length ratio between tess pts, 0=ignore      
                              double                 dAngleTolDeg,              // in : max angle between tess tangents, 0=ignore               
                              ULONG                  lMinimumNumberOfSegments,  // in : Minimum number of tesselation segments, 0=ignore        
                              SmTArray<double>     * pParameters = NULL,        // out: array of parameters                                     
                              SmTArray<SmPoint3d>  * pPoints = NULL,            // out: array of sample points                                  
                              SmTArray<SmVector3d> * pOptTangents = NULL)       // out: Optional array of tangent points for each sample point. 
                             const ;

  SmStatus TessellateByBisection(const SmExtent1d     & rInterval,                 // in : interval to tessellate                                    
                                 double                 dMaxChordHeight,           // in : max height/length ratio between tess pts, 0=ignore        
                                 double                 dMaxAngDeg,                // in : max angle between tess tangents, 0=ignore                 
                                 double                 dMaxDist3d,                // in : max distance between tess pts, 0=ignore                   
                                 SmTArray<double>     * pParameters,               // out: array of split parameter values, NULL=ignore              
                                 SmTArray<SmPoint3d>  * pPoints,                   // out: associated 3D points, NULL=ignore                         
                                 SmTArray<SmVector3d> * pOptTangents = NULL)       // out: Optional array of tangent points for each sample point    
                                const ;

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL)   // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling
                             { SM_REF2(crRotateNMove, cpOptScale) ; SE(SM_ERR); return SM_ERR; }

  // Deform curve smoothly to force its ends to interpolate input EndPt and EndTan tgts. Meant for gap healing and small tol-sized moves
  virtual SmStatus DeformToEndPointTargets(SmPoint3d      * pTgtStartPt,                    // in :  NotNULL = Curve StartPt new Tgt position 
                                                                                            //       NULL    = leave as is
                                           SmPoint3d      * pTgtEndPt,                      // in :  NotNULL = Curve EndPt new Tgt position
                                                                                            //       NULL    = leave as is
                                           SmCurve       ** ppOptNewCurve    = NULL,        // out: Ptr to new curve when editing shape changes this curve's type
                                                                                            //      Not used when this curve can be edited in place without changing its type.
                                                                                            //      default:[NULL]. Returns Err if not given when its needed
                                           SmVector3d     * pOptTgtStartTan  = NULL,        // in : optional Start 1stDir tangent dir (NonUnit but only uses dir, not speed)
                                           SmInValueType    eTgtStartTanType = SM_IV_SAME,  // in : oneof SM_IV_SPECIFIED    : set Start1stDir = pTgtStartTan dir
                                                                                            //            SM_IV_SAME         : set Start1stDir = Init Start1stDir
                                                                                            //            SM_IV_UNCONSTRAINED: set Start1stDir = Unspecified
                                                                                            //      default:[SM_IV_SAME]
                                           SmVector3d     * pOptTgtEndTan    = NULL,        // in : optional End 1stDir tangent dir (NonUnit but only uses dir, not speed)
                                           SmInValueType    eTgtEndTanType   = SM_IV_SAME,  // in : oneof SM_IV_SPECIFIED    : set End1stDir = pTgtEndTan dir
                                                                                            //            SM_IV_SAME         : set End1stDir = Init End1stDir
                                                                                            //            SM_IV_UNCONSTRAINED: set End1stDir = Unspecified
                                                                                            //      default:[SM_IV_SAME]
                                           SmSurface      * pOptSurfCrvOnSurf = NULL,       // in : Optional surface for upon which the target points are define. CrvOnSurf editing. If the CrvOnSurf surf agrees with pOptSurfCrvOnSurf
                                                                                            //      then we edit the underlying UV curve instead of approximating. Default is NULL.
                                           SmOrientType   * pOptCrvOrientation = NULL);     // in : orientation for when we have a crv on surf and will adjust UV points.

  virtual SmStatus Trim (SmExtent1d & crTrimInterval,          // [i/o]: desired new Trim Ivl - can be snapped by tol to existing knots 
                         SmBoolean    bNotify=TRUE,            // in : internal use: use default value     
                         SmBoolean    bSkipDebugCheck=FALSE)   // in : internal use: use default value     
                        { SM_REF3(crTrimInterval, bNotify, bSkipDebugCheck) ; SE(SM_ERR); return SM_ERR; }

  // CurveChain Healing Tools
  // LoopHeal::FixChainCurvesList : order curve array into a curve chain - optionally deleted duplicate curves
  // LoopHeal::CrvsCleanup        : remove short curves and close resulting gaps
  // LoopHeal::CrvsFixGaps        : move SmBSplineCurve EndPts to close small gaps in a head to toe curve sequence
  static ULONG  FixChainCurvesList    (SmTArray <SmCurve *> & cr3DCurves,                  // [in, out]: curves to order into chains (curve directions possibly reversed, duplicates possibly deleted)  
                                       SmTArray <ULONG>     & ChainStart,                  // out: index of each chain start (chain includes curves from this index to next index or end of array)    
                                                                                           //      rChainStart.GetSize() == number of chains in r3DCurves.                                              
                                                                                           // in : dTol*dTol = max distance between matching curve endPoints                                          
                                       double                 dTol = 0.0001,               // in : TRUE = Delete duplicate curves - set Curve's r3DCurves entry to NULL                               
                                       SmBoolean              bDeleteDuplicates = TRUE) ;  //      FALSE= Don't (using duplicate curves may confuse the algorithm)                                      
                                      
  static ULONG  CrvsCleanup           (SmTArray <SmCurve *> & rpCrvs,                      // [in, out]: an ordered chain of target curves             
                                       double                 dTol,                        // in : maximum size for a curve to be considered tiny    
                                       SmBoolean              bDeleteCrv = FALSE) ;        // in : if True, delete any curves removed from sCrvs     
  
  static ULONG CrvsFixGaps            (SmTArray <SmCurve *> & rpCrvs, double dTol);        // in :

  // End Tools for healing Loop Curve Chains

  static void  RemoveDuplicateCurves  (SmTArray <SmCurve *> & rpCrvs);

  SmStatus  RemoveCoincidentSections  (); 

  // Separate duplicate end Control points on non-degenerate curves
  virtual SmBoolean FixRepeatedEndControlPoints() { return FALSE ; } 



  // write a curve to file
  SmStatus WriteToFile(const TCHAR * cOutputFileName,           // in : target file name                                    
                       SmBoolean     bSkipHeaderWrite = FALSE,  // NotUsed: in : TRUE = Omit "Surface Type: TYPE" header label       
                       SmBoolean     bNewFile = FALSE,          // in : TRUE = open file and rewrite contents               
                                                                //        FALSE= open file and append to end                  
                       SmBoolean     bWriteAttributes = FALSE)  // in : TRUE=Write attributes, FALSE=don't                  
                      const ;   
    
  // write array of curves to file                                          
  static SmStatus WriteArrayToFile(const TCHAR               * cOutputFileName,          // in : target file name                           
                                   const SmTArray<SmCurve *> & rCrvArr,                  // in : array of curves to write                
                                   SmBoolean                   bWriteAttributes=FALSE) ; // in : TRUE=Write attributes, FALSE=don't      

  // read a curve from file
  static SmStatus ReadFromFile(const SmContext & crContext,                // in : context for new object construction                   
                               const TCHAR     * cInputFileName,           // in : target file                                           
                               SmCurve        *& rpNewCurve,               // out: the read curve with newly allocated memory            
                               const ULONG       lFileOffsetInBytes = 0) ; // NotUsed: in : Number of characters in file to skip before reading   

  // read array of curves from file
  static SmStatus ReadArrayFromFile(const SmContext     & crContext,                // in : context for new object construction                         
                                    const TCHAR         * cInputFileName,           // in : target file                                                 
                                    SmTArray<SmCurve *> & rNewCurves,               // out: the read curve with newly allocated memory                  
                                    const ULONG           lFileOffsetInBytes = 0,   // NotUsed: in : Number of characters in file to skip before reading)        
                                    SmTArray<SmCurve *> * pTestCurves = NULL) ;     // in : for debug only - the array of curves expected to be read    

  // I/O assist methods
  virtual SmStatus WriteToDB (SmDatabaseIO & rDB,               // in : target output stream                                
                              ULONG          lDBVersionNumber) const   // in : database version to get proper sequence of writes   
                            { SM_REF2( &rDB, lDBVersionNumber ) ; return SM_ERR ; }

  static  SmStatus ReadFromDB(SM_TYPE           lType,              // in : Object type to be read                                                       
                              SmDatabaseIO    & rDB,                 // in : target output stream                                                         
                              ULONG             lDim,                // in : curve image space dim, 2 or 3                                                                                    
                              const SmContext & crContext,           // in : context for new object construction                                          
                              SmCurve        *& rpNewCurve,          // out: NULL on input = new object allocated in this routine built from stream data  
                                                                     //        NotNULL on input = pointer to an empty object to be filled by this routine   
                              ULONG             lDBVersionNumber) ;  // in : database version to get proper sequence of writes                            

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmCurve,SmAObject,SmCurve_TYPE);

  SmStatus OutputGraphics(const SmExtent1d          & crInterval,             // in : curve Interval to display                                                      
                         const SmDisplayParameters & crDisp,                 // in : current display parameters                                                     
                         void                      * pGraphicsNurb,          // NotUsed: in : kept for API compatibility
                         SmPlane                   * pOptOutPlane = NULL,    // in : For 2d curves: Draw on this plane, NULL=draw on z=0 plane, default:[NULL]      
                         SmGfxArraySet             * pOptGfxSet = NULL)      // [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                        const ;

  // Add Curve Graphics to new or open Display List
  virtual SmDisplayList * Draw(const SmExtent1d * pInterval = NULL,            // in : Target Interval, NULL = Use Natural Interval, default:[NULL]                        
                               SmBoolean          bAddToUIPickList = FALSE,    // in : TRUE = Add this Curve to UI pick interface for debugging, default:[FALSE]           
                               SmPlane          * pOptOutPlane = NULL,         // in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                          
                               SmGfxArraySet    * pOptGfxSet = NULL)           // in : used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.             
                              const ;

  SmDisplayList * DrawAt(double          dParameter,               // in : target curve parameter to display                                                
                         ULONG           lNumDeriv = 0,            // in : number of derivatives to display                                                 
                         const double  * pOptVicinityDist = NULL,  // in : notNULL=Draw curve in detail on either side of dParameter                        
                                                                   //    : to a 3d distance of +/- *pVicinity3d.  Intended for micro-graphics               
                                                                   //    : NULL to ignore, default:[NULL]                                                   
                         SmPoint3d     * pOptVicinityPoint = NULL, // in : Vicinity Scaling Center, used when pOptVicinityDist != NULL,                     
                                                                   //    : NULL = use Curve(dParam) as scaling center.                                      
                         double          dVicinityScale = 1000.0,  // in : Vicinity Scale, used when pOptVicinityDist != NULL, default:[1000] ;             
                         SmPlane       * pOptOutPlane = NULL,      // in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                       
                         SmGfxArraySet * pOptGfxSet = NULL)        // [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                        const ;

  SmDisplayList * DrawWDeriv(const SmExtent1d & crIntervalToDraw,    // in : target interval                                                                   
                             ULONG              lDerivative = 0,     // in : 0=Draw Curve                                                                      
                                                                     //    : 1=Draw Curve+1stDeriv comb                                                        
                             SmPlane          * pOptOutPlane = NULL, // in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                        
                             SmGfxArraySet    * pOptGfxSet = NULL)   // [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.   
                            const ;

  SmDisplayList * DrawCurvature(double           dScale =-25.0,       // in : scale applied to curvature vectors                                                 
                                ULONG            lNumber = 35,        // in : number of sample points                                                            
                                SmExtent1d     * pInterval = NULL,    // in : Curve interval to draw,NULL = use NaturalInterval                                  
                                SmPlane        * pOptOutPlane = NULL, // in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                         
                                SmGfxArraySet  * pOptGfxSet = NULL)   // [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.    
                               const ;         

  SmDisplayList * DrawSpeed(double           dScale = -1.0,        // in : scale applied to curvature vectors, default:[-1.0]                                 
                            ULONG            lNumber = 35,         // in : number of sample points, default:[35]                                              
                            SmExtent1d     * pInterval = NULL,     // in : Curve interval to draw, NULL = use NaturalInterval, d3efault:[NULL]                
                            SmPlane        * pOptOutPlane = NULL,  // in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                         
                            SmGfxArraySet  * pOptGfxSet = NULL)    // [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.    
                           const ;         

  // Draw curve and ControlPoint Net
  virtual SmDisplayList * DrawPolygon   (SmGfxArraySet    * pOptGfxSet=NULL) const ;

  // Draw curve and Control Points
  SmDisplayList * DrawControlPoints     (SmGfxArraySet    * pOptGfxSet=NULL) const ;  

  // Draw curve and Curve and Knot Points
  SmDisplayList * DrawWithKnots(const SmExtent1d *pInterval = NULL,   // in : Curve interval to draw                                                               
                                                                       //    : NULL = use NaturalInterval                                                           
                                SmGfxArraySet    * pOptGfxSet = NULL)  // [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.           
                               const ;   

  // Draw sequenced Param Points increasing in
  // size and changing color from green to blue
  SmDisplayList * DrawParams(const SmExtent1d * pInterval=NULL,    // in : Curve interval to draw                                                               
                                                                    //    : NULL = use NaturalInterval                                                           
                             SmGfxArraySet    * pOptGfxSet = NULL)  // [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.      
                            const ;   

  SmDisplayList * DrawCrvGapFunction(const SmCurve    & crOtherCurve,              // in : Other curve                                                                   
                                     ULONG              lOptSampleCnt = 100,       // in : Number of samples, default:[100]                                              
                                     const SmExtent1d * pOptThisCurveIvl = NULL,   // in : this Curve Ivl to draw, NULL=NaturalIvl, default:[NULL]                       
                                     double             dOptTol = .00001,          // in : Minimum distance between two distinct points, default:[.00001]                
                                     SmVector3d       * pOptInTolGapColor = NULL,  // in : GapVector color less than tol, NULL=Orange, default:[NULL]                    
                                     SmVector3d       * pOptOutTolGapColor = NULL, // in : GapVector color greater than tolm NULL=Grey, default:[NULL]                   
                                     SmGfxArraySet    * pOptGfxSet = NULL)         // [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.    
                                    const ;

  SmDisplayList * DrawSrfGapFunction(const SmSurface  & crOtherSurface,
                                     ULONG              lOptSampleCnt = 100,       // in : Other surfac                                                               
                                     const SmExtent1d * pOptThisCurveIvl = NULL,   // in : Number of samples, default:[100]                                           
                                     double             dOptTol = .00001,          // in : this Curve Ivl to draw, NULL=NaturalIvl, default:[NULL]                    
                                     SmVector3d       * pOptInTolGapColor = NULL,  // in : Minimum distance between two distinct points, default:[.00001]             
                                     SmVector3d       * pOptOutTolGapColor = NULL, // in : GapVector color less than tol, NULL=Orange, default:[NULL]                 
                                     SmGfxArraySet    * pOptGfxSet = NULL)         // in : GapVector color greater than tolm NULL=Grey, default:[NULL]                
                                    const ;                                        // [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls. 

  SmDisplayList * DrawInspectTwoCurves(const SmCurve * pCrv2,              // in : drop dense sequence of pts from this curve to pCrv2               
                                       ULONG           lSampleCnt=100,     // in : number of equally spaced ThisParam pts dropped to pCrv2           
                                       ULONG           lOutputIndex=2,     // in : 0 = print sample/dropPt params and 3d distances                   
                                                                           //    : 1 = and draw sample/drop points and                               
                                                                           //    : 2 = and draw vector between sample and drop points                
                                       SmGfxArraySet    * pOptGfxSet=NULL)
                                      const ;

  SmDisplayList * DrawInspectUVTrimCurve(const SmSurface  & crSurface,          // in : check gaps between curves and surface                                           
                                         const SmCurve    & crUVTrimCurve,      // in : UVTrimCurve (expected to share parameterization with this curve)                                                                 
                                         const SmExtent1d * pOptIvl=NULL,       // in : Ivl to draw, NULL=NaturalIvl, default:[NULL]                                    
                                         ULONG              lSampleCnt=100,     // in : Number of samples, default:[100]                                                
                                         SmBoolean          bDrawUVPlane=FALSE, // in : TRUE = Display UVTrimCurves on a UVplane icon near the surface                  
                                         ULONG              lOutputIndex=0,     // in : 0=this pts (blue)                                                               
                                                                                //    : & 1=add TrimPts and Normal (red)                                                
                                                                                //    : & 2=add DropToTrimPts and Normal (green)                                        
                                                                                //    : & 4=add DropToSrfPts and Normal  (cyan) (Debug mode only)                       
                                                                                //    : & 8=add Dropped(dropSrfPt)ToUVTrimPts and Normal (orange) (Debug mode only)     
                                                                                //    : &16=add vectors between drawn points                                            
                                         SmGfxArraySet    * pOptGfxSet=NULL)
                                        const ;

  // get memory used for curve 
  virtual ULONG GetMemoryUsed (ULONG & rlMemoryAllocated,           // out: bigger size of all allocated memory in bytes    
                               SmMarkType eMarkType = SM_MT_NOMARK) // in : uses without increment eMarkType value          
                              const
                             { SM_REF1(eMarkType) ;
                               ULONG lUsed(0), lThisAllocated ;
                               
                               // + cache memory
                               if(m_pCacheObj)
                                 { lUsed = m_pCacheObj->GetMemoryUsed( lThisAllocated );
                                   rlMemoryAllocated += lThisAllocated;
                                 }
                               return sizeof( *this ) + lUsed;
                             }

  virtual SmBoolean PassesValidityCheck (SmValidityCheckType   eChecks,
                                         SmValidityCheckType & reCheckFailed) const  
                                       { SM_REF2(eChecks, reCheckFailed) ; return TRUE; }

  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // [in,out]: Accumulating list of failed Asserts, NULL to ignore                                           
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       
                                                                          //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   
                                                                          //    : default:[SM_LEVEL_0]                                                                             
                                SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 
                               const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  virtual void Dump( const TCHAR * message)  const ;
  virtual void Dump( ULONG)  const ;
  virtual void Dump( SmBoolean bAbbrev) const;

  void DumpTopology(ULONG lWalkDepth=0) const ;  // NotUsed: in : lWalkDepth[0] = no walk, [99] = walk to bottom

} ; // end class SmCurve

// Exported functions

SM_EXPORT ULONG smcurv_GetCurveEvalCount();

/*******************************************************************//**
PURPOSE: A class to make curve evaluation convenient.

NOTES:
 - The methods return references to (not copies of) our actual data,
   so that data could be changed from outside.  Not recommended.
***********************************************************************/
// ToDo:
// - add 3rd derivs
// - don't always eval 2 derivs:
//   - requires NumEvaluated, not just Boolean m_bSet.
//   - and an arg to Eval().
// - allow out-of-bounds evaluation (if desired).

class SM_EXPORT SmCurveEval
{
protected:
  double     m_dParam = SM_UNDEF_DOUBLE ;
  SmVector3d m_sPtDerivs[3] ;

  const SmCurve *m_pCurve; // We do not own this.

  SmBoolean m_bSet;  // Allow for lazy evaluation.

public:
  // Constructors
  SmCurveEval( const SmCurve *pCurve = NULL )
      : m_pCurve( pCurve ),
        m_bSet( FALSE )
      { } 
  SmCurveEval( const SmCurve *pCurve, double dParam )
      : m_pCurve( pCurve ),
        m_bSet( FALSE )
      { Set( dParam ); } 

  // Assignment operator
  SmCurveEval & operator=( const SmCurveEval &crOther )
    { m_pCurve = crOther.GetCurve(); m_dParam = crOther.Param(); m_bSet = FALSE; return *this; }

  // Destructor
  ~SmCurveEval() { m_pCurve = NULL; }

  // simple data access

  SmStatus Set( double dParam ) { m_dParam = dParam; m_bSet = FALSE; return SM_SUCCESS; }

  const SmCurve * GetCurve() const { return m_pCurve; }

  void SetCurve( const SmCurve * pCurve ) { m_pCurve = pCurve; }

  const double & Param() const { return m_dParam;  }
  SmPoint3d  & Pos (SmBoolean bNonZeroTangents=TRUE) { Eval(bNonZeroTangents); return m_sPtDerivs[0]; }
  SmVector3d & Ct  (SmBoolean bNonZeroTangents=TRUE) { Eval(bNonZeroTangents); return m_sPtDerivs[1]; }
  SmVector3d & Ctt (SmBoolean bNonZeroTangents=TRUE) { Eval(bNonZeroTangents); return m_sPtDerivs[2]; }

private:
  SmStatus Eval(SmBoolean bNonZeroTangents=TRUE);

}; // end class SmCurveEval

// Inline methods

/*******************************************************************//**
PURPOSE: Constructor which takes a dimension.

NOTES: 
***********************************************************************/
inline SmCurve::SmCurve(ULONG lDim) 
 : m_lDim(lDim),
   m_pOwner(NULL) 
{
#ifdef SM_DEBUG_CACHE_H
  m_lCurveCacheCount  = 0 ;
#endif

  // derived classes do not duplicate this call
  SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL) ; 

} // end SmCurve::SmCurve constructor

/*******************************************************************//**
PURPOSE: Copy constructor for the curve.

NOTES: 
***********************************************************************/
inline SmCurve::SmCurve(const SmCurve & crSourceCurve) 
 : SmAObject(crSourceCurve) 
{ m_lDim   = crSourceCurve.m_lDim ; 
  m_pOwner = NULL ;
#ifdef SM_DEBUG_CACHE_H
  m_lCurveCacheCount  = 0 ;
#endif

  // derived classes do not duplicate these calls
  Notify(SM_NO_CONSTRUCTION, this, (SmCurve *)&crSourceCurve, NULL) ; 

} // end SmCurve::SmCurve copy constructor

/*******************************************************************//**
PURPOSE: Default destructor.

NOTES: 
***********************************************************************/
inline SmCurve::~SmCurve() 
{
  // derived classes do not duplicate this call
  Notify(SM_NO_DESTRUCTION, this, NULL, NULL) ; 

  m_pOwner = NULL ;

} // end SmCurve::~SmCurve destructor

/*******************************************************************//**
PURPOSE: Copy this curve.

NOTES: 
***********************************************************************/
inline SmStatus SmCurve::Copy(const SmContext & , SmCurve *& ) const
{ 
  SE(SM_ERR); return SM_ERR; 

  // derived classes must make sure that every copy sequence generates
  //  NewObj->SmObject::Notify(SM_NO_CONSTRUCTION, pNewObj, pFromObj, NULL) ;   // from a copy constructor
  // or
  //  NewObj->SmObject::Notify(SM_NO_CONSTRUCTION, pNewObj, NULL, NULL) ;       // from a regular constructor
  //  FromObj->Notify(SM_NO_COPY, pToObj, pFromObj, SM_NO_GET_OWNER(pToObj), SM_NO_GET_OWNER(pFromObj)) ;
  // The SM_NO_CONSTRUCTION is usually already called by constructors
  // The SM_NO_COPY call is not needed if the copy is done through the copy constructor,
  //   otherwise the derived class will have to make sure the call is made whereever
  //   is apporpriate in the method sequence.  The Copy methods named IsNurbType() are
  //   already making the SM_NO_COPY calls.

} // end SmCurve::Copy

/*******************************************************************//**
PURPOSE: Copy this curve as a SmBSplineCurve if possible - else
            set output to NULL.

NOTES: The default behavior is to just copy the curve.
***********************************************************************/
inline SmStatus SmCurve::CopyAnalyticAsNurb
(
  const SmContext &,     
  SmCurve        *& rpCurve
) const
{ 
  rpCurve = NULL ; 

  // derived classes must make sure that every copy sequence generates
  //  NewObj->SmObject::Notify(SM_NO_CONSTRUCTION, pNewObj, pFromObj, NULL) ;  // from a copy constructor   
  // or                                                                                                     
  //  NewObj->SmObject::Notify(SM_NO_CONSTRUCTION, pNewObj, NULL, NULL) ;      // from a regular constructor
  //  FromObj->Notify(SM_NO_COPY, pToObj, pFromObj, SM_NO_GET_OWNER(pToObj), SM_NO_GET_OWNER(pFromObj)) ;
  // The SM_NO_CONSTRUCTION is usually already called by constructors
  // The SM_NO_COPY call is not needed if the copy is done through the copy constructor,
  //   otherwise the derived class will have to make sure the call is made whereever
  //   is apporpriate in the method sequence.  The Copy methods named IsNurbType() are
  //   already making the SM_NO_COPY calls.

  return SM_SUCCESS;

} // end SmCurve::CopyAnalyticAsNurb

/*******************************************************************//**
PURPOSE: A class for finding the min- and/or max- separation between two curves.

NOTES: 
    The curves are parameterized the same: the presumed use is for
    the two rail curves of a variable-radius fillet surface;
    this will find the min and max radius values.

    This is in the .h file because it is used by SmSurface.
***********************************************************************/
class SmFindCurveDistanceExtremaEFO : public SmEvalFunctionObject
{
protected:
  const SmCurve       * m_pCurve1;   // target curve      
  const SmCurve       * m_pCurve2;   // target curve      
        SmExtent1d      m_sDomain;   // domain of interest on the curve
        double          m_dTol;      // stopping tolerance for iteration

public:
  // Constructor:
  SmFindCurveDistanceExtremaEFO(const SmCurve *pCurve1,
                                const SmCurve *pCurve2,
                                SmExtent1d & rsDomain,
                                double   dTol)            : m_pCurve1( pCurve1 ),
                                                            m_pCurve2( pCurve2 ),
                                                            m_sDomain( rsDomain ),
                                                            m_dTol   ( dTol )
                                                          { }
  // assignment operator
  SmFindCurveDistanceExtremaEFO& operator=(SmFindCurveDistanceExtremaEFO const &obj)
                                                          { if(&obj == this) return *this ;
                                                            m_pCurve1 = obj.m_pCurve1 ;
                                                            m_pCurve2 = obj.m_pCurve2 ;
                                                            m_sDomain = obj.m_sDomain ;
                                                            m_dTol    = obj.m_dTol ;
                                                            return *this ;  
                                                          }

  // destructor
  virtual ~SmFindCurveDistanceExtremaEFO() { }

  // evaluate
  virtual SmStatus Evaluate(double      dT,                   // in : target T value to query
                            double    & rdFOfT,               // out: F(T)     = Function value for given dT value
                            double    & rdFPrimeOfT,          // out: dF(T)/dT = Function derivative value for given dT value
                            SmBoolean & rbFoundAnswer,        // out: TRUE = Function value is within tolerance of zero, FALSE=Not
                            SmBoolean   bSignalErrors=TRUE) ; // NotUsed: in : TRUE = signal errors, FALSE=errors anticipated, don't signal, default:[TRUE]

}; // end class SmFindCurveDistanceExtremaEFO

/*******************************************************************//**
PURPOSE: MACRO to temporarily allow a curve of type
  SmBSplineCurve to make out-of-bounds evaluations.

  GWC:TODO - investigate moving this extension concept up to 
             the SmCurve class.

NOTES: 
  1. pCurve is a pointer to a SmCurve which is checked for type
  2. n is a unique integer needed to create a tmp variable to allow
      the SmTemporaryChangeValue mechanism to be used for more than
      one SmCurve in a single name scope without generating a name
      clash
  3. Files using this macro must 
     #include <SmBSplineCurve.h>
***********************************************************************/

#define SM_CURVE_ENABLE_OUTOFBOUNDS(pCurve,n)                  \
  SmBoolean tmpC_##n = 0 ;                                     \
  SmTemporaryChangeValue<SmBoolean> sCStack_##n                \
       (   (pCurve)->IsKindOf(SmBSplineCurve_TYPE)             \
        ? ((SmBSplineCurve *)pCurve)->GetOutOfBoundsEnabled()  \
        : tmpC_##n,                                            \
        TRUE) ;

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    extern SmCurve * dbgCurve1 ;
//    extern SmCurve * dbgCurve2 ;
//    
//    #endif // SM_DEBUG_CODE

#endif // !__SMCURVE_H__


