// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCrvOnSurf.h
* PURPOSE: Header file for implicit 3D curve defined as a 2D
*    parameter space curve and the surface in which it is defined.
**********************************************************************/

#ifndef __SMCRVONSURF_H__
#define __SMCRVONSURF_H__

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMEXTENT2D_H__
#include <SmExtent2d.h>
#endif

/*******************************************************************//**
PURPOSE: The SmCrvOnSurf class defines a 3d curve as a 2D curve traced
    along the parameter space of a surface.

NOTES: 
***********************************************************************/
class SM_EXPORT SmCrvOnSurf : public SmCurve
{
protected:
  SmCurve        * m_pUVCurve = NULL ;              // Parameter space curve  
  SmSurface      * m_pSurface = NULL ;              // Base surface
  SmExtent2d       m_vUVDomain;                     // Domain of interest on the surface
  SmBoolean        m_bNeedBreaks = TRUE ;           // TRUE = m_vBreaks not yet computed, FALSE = m_vBreaks is computed
  SmTArray<double> m_vBreaks;                       // List of curve discontinuities = Curve Knots + Curve/SurfaceDiscontinuity intersections
  ULONG            m_lOwnerFlag = 0 ;               // Constructor default:[0]
                                                    //      0 = deletes nothing when destructed
                                                    //      1 = copies curve and deletes copy when destructed
                                                    //      2 = copies both inputs, deletes both copies when destructed
                                                    //      3 = saves both inputs, deletes both originals when destructed
  SmBSplineCurve * m_pBSApproxCurve = NULL ;        // Cached BSpline approximation to the 3d curve. Lazy evaluation, as needed.
  double           m_dMaxGap3d = SM_BIG_DOUBLE;     // Max deviation between approximation and actual 3d curve 
  SmBoolean m_bParametrizationMatches = FALSE;      // Flag for whether the 3d approximation parameterization matches the 3d curve.
  public:

  // constructor
  SmCrvOnSurf(SmCurve          & rUVCurve,            // in : Parameter space curve                                          
              SmSurface        & rSurface,            // in : Base surface                                                   
              const SmExtent2d * cpUVDomain = NULL,   // in : Domain of interest on the surface, NULL = NaturalUVDomain                              
              ULONG              lOwnerFlag = 0,      // in : 0 = deletes nothing when destructed                            
                                                      //    : 1 = copies curve and deletes copy when destructed              
                                                      //    : 2 = copies both inputs, deletes both copies when destructed    
                                                      //    : 3 = saves both inputs, deletes both originals when destructed  
              const SmContext  * cpContext = NULL)    // in : must be given for automatic variables, optional for            
                                                      //    : stack variables built with overloaded new.                     
             : SmCurve( 3 )
             { SetAll( rUVCurve, rSurface, cpUVDomain, lOwnerFlag, cpContext) ; }

  // Constructor - UVLineSegment on a Surface
  SmCrvOnSurf(SmPoint2d        & rStartUV,            // in : Start point of pUVCurve UVline
              SmPoint2d        & rEndUV,              // in : End   point of pUVCurve UVline
              SmSurface        & rSurface,            // in : Base Surface
              const SmExtent2d * cpUVDomain = NULL,   // in : Domain of interest on the surface, NULL = NaturalUVDomain  
              ULONG              lOwnerFlag = 1,      // in : 1 = saves rSurface and owns m_pUVCurve and only deletes m_pUVCurve when destructed
                                                      //      2 = copies rSurface and deletes m_pSurface and m_pUVCurve when destructed
                                                      //      3 = saves rSurface and deletes rSurface and m_pUVCurve when destructed
              const SmContext  * cpContext  = NULL) ; // in : must be given for automatic variables, optional for            
                                                      //    : stack variables built with overloaded new.

  // empty constructor for I/O
  SmCrvOnSurf() : SmCurve( 3 ),
                  m_pUVCurve( NULL ),
                  m_pSurface( NULL ),
                  m_bNeedBreaks( TRUE ),
                  m_lOwnerFlag( 0 ),
                  m_pBSApproxCurve( NULL ),
                   m_dMaxGap3d(SM_BIG_DOUBLE)
                { }

  // copy constructor
  SmCrvOnSurf( const SmCrvOnSurf & crCurveToCopy) ;

  // copy
  virtual SmStatus Copy(const SmContext & crContext,
                        SmCurve        *& crNewCurve)
                       const;

  // destructor
  virtual ~SmCrvOnSurf() ;

  // equality operator
  virtual SmBoolean operator==( const SmCurve &crOther ) const ;

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

  // Change NURB parameterization of curve to given extent range - no Analytic-STEP range change
  virtual SmStatus EditParameterization(const SmExtent1d & crNewParameterization,       ///< [in] : new parameter range for curve                    <br>
                                        SmBoolean          bNotify = TRUE) ;            ///< [in] : TRUE  = make notify calls (previous behavior)    <br>
                                                                                        ///<      : FALSE = Skip notify call                         <br>

  // evaluate surface(curve(dParameter)) for position and derivatives
  virtual SmStatus Evaluate(double     dParameter,                ///< [in] : tgt param                                                                        <br>
                            ULONG      lNumDerivatives,           ///< [in] : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .                                         <br>
                            SmBoolean  bFromLeft,                 ///< [in] : if P is on interval boundary                                                     <br>
                                                                  ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval      <br>
                                                                  ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval     <br>
                            SmVector3d aPointAndDerivatives[],    ///< [out]: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]                                  <br>
                            SmBoolean  bNonZeroTangents = TRUE)   ///< [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors     <br>
                           const ;                                ///<      : FALSE= return exact tangent values                                               <br>
                                                                  ///<      : note: Surprisingly TRUE is the common choice because most tangent uses are for   <br>
                                                                  ///<      : their direction (Binorm, SurfNorm comps), but when the tangent is being          <br>
                                                                  ///<      : used for its magnitude (like an arc-length comp) then set this to FALSE.         <br>   

  // evaluate Pt3d = surface(curve(dParameter)) for position
  virtual SmStatus EvaluatePoint(double      dParameter,
                                 SmPoint3d & rPoint )
                                const ;

  // evaluate SurfPtUV = curve(dParameter) for position
  SmStatus EvaluateUVPoint(double      dParameter,
                           SmPoint2d & rUVPoint )
                          const ;

  // evaluate Surface values for given UVTrimcurve parameter value
  SmStatus EvaluateSurface(double      dParameter,              ///< [in] : tgt param                                                                                <br>
                           ULONG       lHighestUDeriv,          ///< [in] : number of U derivatives                                                                  <br>
                           ULONG       lHighestVDeriv,          ///< [in] : number of V derivatives to compute                                                       <br>
                           SmBoolean   bFromLeft,               ///< [in] : if P is on interval boundary                                                             <br>
                                                                ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval              <br>
                                                                ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval             <br>
                           SmBoolean   bOnlyUpperHalf,          ///< [in] : TRUE=compute upper half of matrix only                                                   <br>
                                                                ///<      : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value            <br>
                                                                ///<      :           [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)       <br>
                                                                ///<      :                         [Dvv  ---   ---]                                                 <br>
                           SmVector2d  aUV[2],                  ///< [out]: UVTrimCurve UVposition and UVtangent                                                     <br>
                           SmVector3d *aDerivatives,            ///< [out]: matrix of evaluations values                                                             <br>
                                                                ///<      : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                               <br>
                                                                ///<      : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)    <br>
                                                                ///<      :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )    <br>
                                                                ///<      :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                       <br>
                                                                ///<      :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                       <br>
                                                                ///<      : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...] <br>
                           SmBoolean   bNonZeroTangents = TRUE) ///< [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors             <br>
                          const ;                               ///<      : FALSE= return exact tangent values                                                       <br>
                                                                ///<      : note: Surprisingly TRUE is the common choice because most tangent uses are for           <br>
                                                                ///<      : their direction (Binorm, SurfNorm comps), but when the tangent is being used             <br>
                                                                ///<      : for its magnitude (like an arc-length comp) then set this to FALSE.                      <br>

// evaluate Surface Normal for given UVTrimcurve parameter value
  SmStatus EvaluateSurfaceNormal(double       dParameter,       ///< [in] : tgt param                                                                       <br>
                                 SmBoolean    bFromLeft,        ///< [in] : if P is on interval boundary                                                    <br>
                                                                ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
                                                                ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
                                 SmVector2d   aUV[2],           ///< [out]: UVTrimCurve UVposition and UVtangent                                            <br>
                                 SmVector3d & rSurfaceNormal) ; ///< [out]: Surface normal                                                                  <br>

  // Evaluate derivatives of the surface's 1st derivatives w.r.t. our parameter.
  // Note, currently does only one derivative, if higher derivatives are
  // requested they are set to zero.
  SmStatus EvaluateSurface1stDerivDerivs(double      dT,                ///< [in] : tgt param                                             <br>
                                         ULONG       lNumDers,          ///< [in] : number of derivatives (w.r.t. t) required.            <br>
                                         SmBoolean   bFromLeft,         ///< [in] : as above                                              <br>
                                         SmVector3d *aSurfaceUDers,     ///< [out]: Derivs of Su w.r.t. t                                 <br>
                                         SmVector3d *aSurfaceVDers) ;  ///< [out]: Derivs of Sv w.r.t. t                                 <br>

  // simple data access
  virtual SmExtent1d GetNaturalInterval() const { return m_pUVCurve->GetNaturalInterval(); }

  virtual SmStatus CalculateBoundingBox(const SmExtent1d& crInterval, // in : target interval of interest -
                                                                                  // this is used
                                       SmExtent3d* pBBox,       // out: Axis alligned box, NULL to ignore, default:[NULL]
                                       SmPseudoBox* pPseudoBox, // out: Non-axis aligned box, NULL to ignore,
                                                                // default:[NULL]
                                       SmPolarBox* pPolarBox,   // out: Curve tangent vector field bounding box, NULL to
                                                                // ignore, default:[NULL]
                                       SmBoolean bExpandBox = TRUE)    // in : TRUE = expand BBoxes prior to return
                                                      const ;   //      FALSE= don't
                                                                //      default:[TRUE]

  // report all discontinuities due to the propogation of both the curve and surface discontinuities as SM_CT_C1
  virtual SmStatus CalculateContinuities(SmContinuityType           & reMinContinuityInCurve,    ///< [out]: min continuity of all internal knots         <br>
                                         SmTArray<SmContinuityType> & rContinuitiesAtKnots,      ///< [out]: continuity at every knot value for curve     <br>
                                         double dContinuityAngleTol = SM_CONTINUITY_ANGLE)       ///< [in] :                                              <br>
                                        const ;

  // create mirrored copy of this curve. rpMirrorCurve derived type same as 'this' curve
  virtual SmStatus CreateMirrorCurve(const SmContext&        crContext,      ///< [in] :     <br>
                                     const SmAxis2Placement& crMirrorPlane,  ///< [in] :     <br>
                                           SmCurve*&         rpMirrorCurve)  ///< [out]:     <br>
                                    /*const*/ ;

  // Create a curve by projecting an existing curve into a plane using either parallel or perspective projection.
  SmStatus CreatePlaneProjection(const SmContext  & crContext,                 ///< [in] : context for new object construction                                                 <br>
                                 SmProjectionType   eProjectionType,           ///< [in] : oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE                                             <br>
                                 const SmPoint3d  & rProjectionPlanePoint,     ///< [in] : Point on target plane                                                               <br>
                                 const SmVector3d & rProjectionPlaneNormal,    ///< [in] : Normal of target plane                                                              <br>
                                 const SmVector3d & rProjDirOrCenterOfProj,    ///< [in] : When eProjectionType == SM_PT_PARALLEL, vector is projection direction.             <br>
                                                                               ///<      : When eProjectionType == SM_PT_PERSPECTIVE, vector is 'eye' point of the projection. <br>
                                 SmCurve         *& rpProjectedCurve)          ///< [out]:                                                                                     <br>
                                const ;

  virtual SmStatus DropPoint(const SmExtent1d    & crInterval,                       ///< [in] : target curve allowed domain                                                            <br>
                           const SmPoint3d     & crPointToDrop,                    ///< [in] : Point to drop to curve                                                                 <br>
                           const SmVector3d    * cpVecToCrvInside,                 ///< [in] : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.  <br>
                                                                                   ///<      : when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.    <br>
                                                                                   ///<      : Vec handles ambiguities and makes sure that curves are not just touching at the ends.  <br>                                                                    
                           double                dDistTol,                         ///< [in] : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.                           <br>
                                                                                   ///<      : sXSectTol3d = dDistTol ;      SM_SO_INTERSECT                                          <br>
                                                                                   ///<      : sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE                          <br>
                                                                                   ///<      : sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT         <br>
                                                                                   ///<      : DropPTs kept when DropDist < sXSectTol3d                                               <br>
                           const double        * pOptGuessParam,                   ///< [in] : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()          <br>
                           SmBoolean           & rbSuccess,                        ///< [out]: TRUE = found a drop point                                                              <br>
                           double              & rdDroppedParameter,               ///< [out]: found drop curve param                                                                 <br>
                           double              & rdDistanceToCurve,                ///< [out]: found drop distance                                                                    <br>
                           SmSolverOperationType eOperationType = SM_SO_MINIMIZE)  ///< [in] : SM_SO_MINIMIZE = allow nonNormal drops near endPoints                                  <br>
                                                                                   ///<      : SM_SO_INTERSECT= like Minimize but point must be within dDistTol                       <br>
                                                                                   ///<      : SM_SO_NORMALIZE= exclude nonNormal drops near endPoints                                <br>
                                                                                   ///< [out]: Portions of ThisCurve on pos side of ininfite plane                                    <br>                  
                          const ;

  // report all discontinuities due to the propogation of both the curve and surface discontinuities as knots
  //  on a degree 3 surface with multiplicity 2
  virtual SmStatus  GetKnots(SmTArray<double> & rKnots,                               ///< [out]: unique knot list                                               <br>
                             SmTArray<ULONG>  * pKnotMultiplicities = NULL,           ///< [out]: multiplicity reported for each knot                            <br>
                             const SmExtent1d * pOptIvl = NULL)                       ///< [in] : interval of interest, NULL=Natural Interval, default:[NULL]    <br>
                            const ;

  // report all discontinuities due to the propogation of both the curve and surface discontinuities as knots
  //  on a degree 3 surface with multiplicity 2
  virtual ULONG            GetNumberNaturalKnots() const;
  virtual SmBSplineCurve * GetRootCurve()          const { return(m_pUVCurve->GetRootCurve()); }

  SmStatus GetOrCreateBSApproxPointer
                        (SmBSplineCurve *&  pBSCurve,                               ///< [out]: Pointer to bspline approximation to 3d curve.
                         double             dApproxTol = SM_APPROX_TOL_3D,          ///< [in] : Approximation tolerance. Default is 3d tolerance.
                         SmTArray<double> * pOptBreaks = NULL,                      ///< [in] : Optional array of breaks. If null, breaks are computed.
                                                                                    ///         Default is null.
                         SmBoolean        * pOptParameterizationMatches = NULL);    /// <[in] : Optional flag to match parameterization for approximation.
  // simple data access  
  const SmContext * FindContext()    const;
  const SmCurve   * GetUVCurve()     const { return m_pUVCurve; }
  const SmSurface * GetBaseSurface() const { return m_pSurface; }
  const SmExtent2d& GetUVDomain()    const { return m_vUVDomain; }
  const double      GetBSApproxMaxGap3d() const { return m_dMaxGap3d; }

  void              SetUVCurve( SmCurve *cpUVCurve );
  void              SetBaseSurface( SmSurface *cpBaseSurface ) { m_pSurface = cpBaseSurface; }
  void              SetUVDomain(const SmExtent2d& rUVDomain)   { m_vUVDomain = rUVDomain; }
  void              SetOwnerFlag( ULONG lOwnerFlag ) // in : lOwnerFlag: 0 = deletes nothing when destructed
                                                     //                  1 = deletes m_pUVCurve when destructed
                                                     //                  2 = deletes m_pUVCurve and m_pSurface when destructed
                                                     //                  3 = deletes m_pUVCurve and m_pSurface when destructed
                                                     { m_lOwnerFlag = lOwnerFlag ; }
  void              SetContext(const SmContext * cpContext)
                                                     { m_cpContext = cpContext ;
                                                       if (m_lOwnerFlag > 1 ) m_pSurface->SetContext(cpContext);
                                                       if (m_lOwnerFlag > 0 ) m_pUVCurve->SetContext(cpContext); }

  void              SetAll
  ( 
    SmCurve          & rUVCurve,          ///< [in] : Parameter space curve                                           <br>
    SmSurface        & rSurface,          ///< [in] : Base surface                                                    <br>
    const SmExtent2d * cpUVDomain = NULL, ///< [in] : Domain of interest on the surface                               <br>
    ULONG              lOwnerFlag = 0,    ///< [in] : 0 = deletes nothing when destructed                             <br>
                                          ///<      : 1 = copies curve and deletes copy when destructed               <br>
                                          ///<      : 2 = copies both inputs, deletes both copies when destructed     <br>
                                          ///<      : 3 = saves both inputs, deletes both originals when destructed   <br>
    const SmContext  * cpContext = NULL   ///< [in] : must be given for automatic variables, optional for             <br>
                                          ///<      : stack variables built with overloaded new.                      <br>
  );

  virtual SmBoolean IsAnalytic()     const { return FALSE; }
  virtual SmBoolean IsBounded()      const { return m_pUVCurve->IsBounded(); } // TRUE=finite (FALSE=infinite) parameter range

  // 
  virtual SmStatus ReverseParameterization(const SmExtent1d & crOldInterval,      ///< [in] : current curve interval             <br>
                                           SmExtent1d       & rNewInterval) ;     ///< [out]: curve interval after reversal      <br>

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  virtual SmStatus Trim(SmExtent1d & crTrimInterval,            ///< [i/o] : desired new Trim Ivl - can be snapped by tol to existing knots <br>
                        SmBoolean    bNotify = TRUE,            ///< [in] : internal use: use default value    <br>
                        SmBoolean    bSkipDebugCheck = FALSE) ; ///< [in] : internal use: use default value    <br>

  SmStatus SurfaceCurveMaxDistanceBetween(const SmExtent1d & crInterval,                     ///< [in] : interval limit for this curve                                                    <br>
                                          const SmCurve    & crOtherCurve,                   ///< [in] : other curve to test                                                              <br>
                                          double   dOtherCurveParameterAtStartOfThis,        ///< [in] : OtherCurve param mapping to ThisCurve Interval.Min value                         <br>
                                          double   dOtherCurveParameterAtEndOfThis,          ///< [in] : OtherCurve param mapping to ThisCurve Interval.Max value                         <br>
                                          ULONG    lNumSamples,                              ///< [in] : Min number of samples to take - it measures at least this many points            <br>
                                                                                             ///<      : at a uniform spacing on the otherCurve finding the                               <br>
                                                                                             ///<      : corresponding ThisCurve closest points.                                          <br>
                                                                                             ///<      : If 0 is given it will do its best to perform                                     <br>
                                                                                             ///<      : a precise measurement and will be much slower.                                   <br>
                                          double   dDropTolerance3d,                         ///< [in] : Distance where nearby DropPoint solutions will be considered the same solution   <br>
                                          double * pdOptMaxDistNeeded,                       ///< [in] : Max allowed gap, either Dist or Dev.                                             <br>
                                                                                             ///<      : Quit searching once this value is exceeded.                                      <br>
                                                                                             ///<      : NULL to ignore.   Never quit search when NULL.                                   <br>
                                          double & rdMaxDistFound,                           ///< [out]: Set to signed max distance seen along the surface normal.                        <br>
                                                                                             ///<      : a. When lNumSamples == 0 This is the max curve/surface gap.                      <br>
                                                                                             ///<      : b. When pdOptMaxDistanceNeeded this is either the max sampled                    <br>
                                                                                             ///<      :    curve/surface gap which is less than pdOptMaxDistanceNeeded or                <br>
                                                                                             ///<      :    the 1st gap seen larger than pdOptMaxDistanceNeeded.                          <br>
                                          double & rdMaxDevFound,                            ///< [out]: Set to max lateral deviation seen.                                               <br>
                                                                                             ///<      : a. When lNumSamples == 0 This is the max curve/curve gap                         <br>
                                                                                             ///<      :   in the surface tangent plane.                                                  <br>
                                                                                             ///<      : b. When pdOptMaxDistanceNeeded this is either the max sampled                    <br>
                                                                                             ///<      :   curve/curve gap which is less than pdOptMaxDistanceNeeded or                   <br>
                                                                                             ///<      :   the 1st gap seen larger than pdOptMaxDistanceNeeded.                           <br>
                                          double * pOptCurveDistT = NULL,                    ///< [out]: Curve param for returned MaxDist Found, NULL to ignore.                          <br>
                                          double * pOptOtherDistT = NULL,                    ///< [out]: OtherCurve param for returned MaxDist Found, NULL to ignore.                     <br>
                                          double * pOptCurveDevT = NULL,                     ///< [out]: Curve param for returned MaxDev Found, NULL to ignore.                           <br>
                                          double * pOptOtherDevT = NULL)                     ///< [out]: OtherCurve param for returned MaxDev Found, NULL to ignore.                      <br>
                                         const ;

  // get Curve Memory size 
  virtual ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,        ///< [out]: bigger size of all allocated memory in bytes    <br>
                              SmMarkType eMarkType = SM_MT_NOMARK) ///< [in] : uses without increment eMarkType value          <br>
                             const ;
                            
  virtual SmStatus WriteToDB(SmDatabaseIO & rDB,             ///< [in] : target output stream                                 <br>
                             ULONG lDBVersionNumber)         ///< [in] : database version to get proper sequence of writes    <br>
                            const ;  

  static  SmStatus ReadFromDB(SM_TYPE           lType,              ///< NotUsed: [in] : Object type to be read                                                      <br>
                              SmDatabaseIO    & rDB,                ///< [in] : target output stream                                                        <br>
                              ULONG             lDim,               ///< NotUsed: [in] : curve image space dim, 2 or 3                                               <br>                                    
                              const SmContext & crContext,          ///< [in] : context for new object construction                                         <br>
                              SmCurve         *&rpNewCurve,         ///< [out]: NULL on input = new object allocated in this routine built from stream data <br>
                                                                    ///<      : NotNULL on input = pointer to an empty object to be filled by this routine  <br>
                              ULONG             lDBVersionNumber) ; ///< [in] : database version to get proper sequence of writes                           <br>

  virtual SmBoolean AssertValid(SmAssertArray    * pAList = NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                           <br>
                                SmAssertTestLevel  eTestLevel = SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       <br>
                                                                            ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   <br>
                                                                            ///<      : default:[SM_LEVEL_0]                                                                             <br>
                                SmAssertWalking    eWalkTree = SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] <br>
                                SmTArray<ULONG>  * pTestRequests = NULL)    ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 <br>
                               const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList );

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON( SmCrvOnSurf, SmCurve, SmCrvOnSurf_TYPE );

}; // end class SmCrvOnSurf

#endif // !__SMCRVONSURF_H__

