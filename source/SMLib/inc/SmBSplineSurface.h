// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBSplineSurface.h
* PURPOSE: Header file for B-Spline Surface class.
**********************************************************************/

#ifndef __SMBSPLINESURFACE_H__
#define __SMBSPLINESURFACE_H__

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

#ifndef __SMGFX_EXTERN_H__
#include <SmGraphicsExtern.h>
#endif

#ifndef __Sm_Nurbs_H__
#include <SmNurbs.h>
#endif

class SmGfxArraySet ;

/*******************************************************************//**
PURPOSE: This object is a surface which represents a NURBS surface.

NOTES: 
***********************************************************************/
class SM_EXPORT SmBSplineSurface : public SmSurface
{
    friend class SmSurfaceCache;
    friend class SmTrimSrfCache;
    friend class SmTree;

protected:
    gw_SURFACE*           m_pNurb;                 // ptr to Bspline data of type gw_SURFACE
    SmBoolean             m_bNurbIsBorrowed;       // borrowed Nurb pointers are on the stack and
                                                   // can't do global operations
    SmBoolean             m_bOutOfBoundsEnabled ;  // GWC:OUTOFBOUNDS MODIFICATION
                                                   // TRUE=RETURN EVALUATIONS FOR UV POINTS BEYOND DOMAIN
                                                   // default:[FALSE]
    SmKnotType            m_eKnotType;
    SmBSplineSurfaceForm  m_eBSplineSurfaceForm;

public:
    // constructor - from BSpline components
    SmBSplineSurface
    (
      ULONG       n,                  // in : high index of Pw (U)                                    
      ULONG       m,                  // in : high index of Pw (V)                                    
      short       p,                  // in : degree in u                                             
      short       q,                  // in : degree in v                                             
      ULONG       r,                  // in : high index of U                                         
      ULONG       s,                  // in : high index of V                                         
      gw_CPOINT **Pw,                 // in : Control polygon  data in NLib format  NOT checked here  
      double     *U,                  // in : knots in u       data in NLib format  NOT checked here  
      double     *V                   // in : knots in v                                              
     // SmBoolean   bAssertHeal=TRUE    // NotUsed: in : TRUE = run AssertHeal sequence on new BSplineSurfaces   
    );
    
    // constructor - from gw_Surface to copy
    SmBSplineSurface
    (
      const gw_SURFACE * cpGwNurb             // in : Std Constructor, copies gw_SURFACE NURB object        
      // SmBoolean          bAssertHeal=TRUE     // NotUsed: in : TRUE = run AssertHeal sequence on new BSplineSurfaces 
    );

    // constructor - from gw_Surface to contain - (typically for internal Cache management) 
    SmBSplineSurface
    (
      gw_SURFACE      * pAlreadyAllocatedNurb,   // in : Fast constructor, stores pAlreadyAllocatedNurb NURB object   
      SmBoolean         bNurbIsBorrowed,         // in : TRUE = Don't delete pAlreadyAllocatedNurb when destructed    
      const SmContext * pContext                 // in : set this->m_cpContext when given                             
      // SmBoolean         bAssertHeal              // NotUsed: in : TRUE = run AssertHeal sequence on new BSplineSurfaces        
    );

    // empty constructor for IO
    SmBSplineSurface() ;

    // Copy Constructor
    SmBSplineSurface(const SmBSplineSurface & crSourceSurface) ;                              

    // destructor
    virtual ~SmBSplineSurface();
    
    // equality operator
    virtual SmBoolean operator==(const SmSurface &crOther) const;

    // Public methods
    virtual SmStatus AdjustSTEPUVDomain(const SmExtent2d & crNewSTEPUVDomain);

    SmStatus AdjustBoundaryCurve
    (
      const SmBSplineCurve * cpNewBoundaryCurve,  // in :                                     
      SmSurfParamType        eSurfDir,            // in : u- or v-direction                   
      SmBoolean              bIsNewBdryAtMin,     // in : TRUE - u(v)-min FALSE - u(v)-max    
      double                 dPercentAffected     // in :                                     
    );

    // build SmBSplineSurface approximation of exact offset of a BSplineSurface with N_SrfOffset() 
    SmStatus ApproximateOffsetSurface
    (
      const SmContext   & crContext,       // in : context for new object construction            
      double              dOffsetDistance, // in : offset distance along normal (neg for insets)  
      double              dApproxTol3d,    // in : max allowed approximation distance             
      SmBSplineSurface *& rpNewSurf,       // out: newly constructed approximation surface        
      int                 iParamType = 0   // in : specify approximation test method              
                                           //    : 0 = INHERITED(default)                         
                                           //    : 1 = NL_CHORDLENGTH,                               
                                           //    : 2 = NL_CENTRIPETAL                                
                                           //    : 3 = NL_UNIFORM                                    
    );

    // approximate a regular RxS sized grid[i][j] of sample points with a Nurbs surface. 
    // without a tol, calls N_FitSrfLstSqApprox() to return a RxS sized controlPoint surface 
    // with    a tol, calls N_FitSrfApproxTol()   to return a RxS sized controlPoint surface after checking for knot removal
    static SmStatus ApproximatePoints
    (
      const SmContext           & crContext,            // in : New Object Context                                                       
      const SmTArray<SmPoint3d> & crPoints,             // in : Tgt Points arranged in a sample grid sized:[lNumRows x lNumCols] points                                                      
                                                        //    : ordered:P[row][col] = p[row*lNumCols + col]                              
      ULONG                       lNumRows,             // in : Number of rows in crPoints array                                         
      ULONG                       lNumCols,             // in : Number of cols in crPoints array                                         
      ULONG                       lUDegree,             // in : Output surface degree U                                                  
      ULONG                       lVDegree,             // in : Output surface degree V                                                  
      double                    * pdOptTolerance,       // in : NULL    = use N_FitSrfLstSqApprox() to approximate                       
                                                        //    : a (lNumRows)x(lNumCols) controlPoint surface to crPoints                 
                                                        //    : NotNULL = use N_FitSrfApproxTol() to approximate                         
                                                        //    : a possible smaller controlPoint count surfact to crPoints                
                                                        //    : to within specified tolerance.  Larger tolerances yield                  
                                                        //    : smaller control point surfaces.                                          
      SmBSplineSurface         *& rpNewSurf,            // out: Constructed surface                                                      
      SmBoolean                   bParamOption = true   // in : when using N_FitSrfApproxTol(), as selected by pdOptTolerance value,     
                                                        //    : specify method used to reduce knot counts.                               
                                                        //    : TRUE  = knots computed by knot reduction of last iteration surface       
                                                        //    : FALSE = knots computed by clumping sample point param values             
    );                                                  

    // approx a regular grid of position and normal sample points with N_FitPtsNormals which builds a
    // bicubic piecewise bezier Nurbs surface with about 9 times as many control points as sample points
    static SmStatus ApproximatePointNormals
    (
      const SmContext      & crContext,       // in : new object context                                            
      const SmTArray<SmPoint3d>  & crPoints,  // in : Grid of Sample Point Positions, sized:[lNumRows][lNumCols]    
                                              //    : ordered:P[row][col] = p[row*lNumCols + col]                   
      const SmTArray<SmVector3d> & crNormals, // in : Grid of Sample Point Normals, sized:[lNumRows][lNumCols]      
                                              //    : ordered:P[row][col] = p[row*lNumCols + col]                   
      ULONG                        lNumRows,  // in : number of rows in Sample Point grid                           
      ULONG                        lNumCols,  // in : number of cols in Sample Point grid                           
      SmBSplineSurface          *& rpNewSurf  // out: approximating surface                                         
    );

    // fit a surface to a cloud of point poisitions and optional natural boundary curves and their cross-derivatives.
    //   assigns UV values to each point  
    //       1. with cpOptBdryCurves, by projecting to a base coons approximate surface
    //       2. without cpOptBdryCurves, by projecting points to a plane.   
    //   Calls N_FitSrfLstSqBoundary() 
    static SmStatus ApproximateRandomPoints
    (
      const SmContext                 & crContext,      // in : New Object Context                                               
      const SmTArray<SmPoint3d>       & crPoints,       // in : target Points                                                    
      ULONG                             lMaxUCtrlPnts,  // in : Output surface controlPoints U maximum count                     
      ULONG                             lMaxVCtrlPnts,  // in : Output surface controlPoints V maximum count                     
      ULONG                             lUDegree,       // in : Output surface degree U                                          
      ULONG                             lVDegree,       // in : Output surface degree V                                          
      const SmTArray<SmBSplineCurve*> * cpOptBdryCurves,// in : The first two will be U-boundaries (v-min & v-max) and           
                                                        //    : the last  two will be V-boundaries (u-min & u-max)               
                                                        //    : Will increase lMaxUCtrlPnts and lMaxVCtrlPnts values if needed.  
      ULONG                             lComplexity,    // in : 1 for simple patches (cheap)                                     
                                                        //    : 2 when internal details are required (expensive), default:[2]    
                                                        //    : 3 for more points and smoother surfaces (inbetween)              
      SmBSplineSurface               *& rpNewSurf       // out: approximated surface                                             
    );

    // fit a surface to a cloud of point poisitions and optional natural boundary curves and their cross-derivatives.
    //   assigns UV values to each point. Unlike ApproximateRandomPoints this uses LU Decomposition and alpha and beta parameters to control the result  
    //       1. with cpOptBdryCurves, by projecting to a base coons approximate surface
    //       2. without cpOptBdryCurves, by projecting points to a plane.   
    //   Calls N_FitSrfLstSqBoundaryDeformablePeriodic() 
    static SmStatus ApproximateRandomPointsDeformable
    (
      const SmContext                 & crContext,      // in : New Object Context                                                     
      const SmTArray<SmPoint3d>       & crPoints,       // in : target Points                                                          
      ULONG                             lMaxUCtrlPnts,  // in : Output surface controlPoints U maximum count                           
      ULONG                             lMaxVCtrlPnts,  // in : Output surface controlPoints V maximum count                           
      ULONG                             lUDegree,       // in : Output surface degree U                                                
      ULONG                             lVDegree,       // in : Output surface degree V                                                
      const SmTArray<SmBSplineCurve*> * cpOptBdryCurves,// in : The first two will be U-boundaries (v-min & v-max) and                 
                                                        //    : the last  two will be V-boundaries (u-min & u-max)                     
                                                        //    : Will increase lMaxUCtrlPnts and lMaxVCtrlPnts values if needed.        
      SmBoolean bPeriodicU,                             //    :
      SmBoolean bPeriodicV,                             //    :
      ULONG                             lComplexity,    // in : 1 for simple patches (cheap)                                           
                                                        //    : 2 when internal details are required (expensive), default:[2]          
                                                        //    : 3 for more points and smoother surfaces (inbetween)                    
      double dAlpha,                                    // in : Resistance to stretch weight term                                      
      double dBeta,                                     // in : Resistance to bending weight term                                      
      double dStiffness,                                // in : Scales stiffness matrix. Higher value lessens impact of alpha and beta 
      SmBSplineSurface                 *& rpNewSurf     // out: approximated surface                                                   
    );

    // Fit a surface to a closed curve described as either an ordered set of point positions or a BSPlineCurve.  
    //   assigns UV values to each point by projecting points to a plane.
    //   Calls N_FitRandomPN() with point position and normal samples computed from the input curve. 
    static SmStatus ApproximateRandomPoints
    (
      const SmContext     & crContext,      // in : New Object Context                              
      SmTArray<SmPoint3d> * cpOpt3DPoints,  // in : Optional target Points                          
      SmBSplineCurve     *& cpOpt3DCurve,   // in : Optional boundary curve through crPoints        
      ULONG                 lMaxUCtrlPnts,  // in : Output surface controlPoints U maximum count    
      ULONG                 lMaxVCtrlPnts,  // in : Output surface controlPoints V maximum count    
      ULONG                 lUDegree,       // in : Output surface degree U                         
      ULONG                 lVDegree,       // in : Output surface degree V                         
      SmBSplineSurface   *& rpNewSurf       // out: approximated surface                            
    );

    // Fit a surface to a cloud of point positions and normals
    //   assigns UV values to each point by projecting points to a plane.
    //   Calls N_FitRandomPNWithPlaneToTol()
    static SmStatus ApproximateRandomPointsNormals
    (
      const SmContext            & crContext,                  // in : new object context                               
      const SmTArray<SmPoint3d>  & crPoints,                   // in : sample points                                    
      const SmTArray<SmVector3d> & crNormals,                  // in : associated surface normals                       
      ULONG                        lMaxUCtrlPnts,              // in : output control point count U                     
      ULONG                        lMaxVCtrlPnts,              // in : output control point count V                     
      ULONG                        lUDegree,                   // in : output degree U                                  
      ULONG                        lVDegree,                   // in : output degree V                                  
      SmBSplineSurface          *& rpNewSurf,                  // out: approximating surface                            
      SmAxis2Placement           * pOptProjectionPlane = NULL, // in : Opt projection plane, NULL to ignore             
                                                               //    : default:[NULL]                                   
      double                       dOptTolerance = 0.0         // in : Max distance allowed between approx and points,  
                                                               //    : 0.0 to ignore, default:[0.0]                     
    );

    // approximate this rational BSplineSurface with a degree 3 nonRational BSplineSurface
    SmStatus ApproximateRationalSurface
    (
      const SmContext   & crContext,        // in :    
      double              dThisApproxTol3d, // in :    
      SmBSplineSurface *& rpNewSurf         // out:    
    );

    // create surface from family of cross-section curves 
    static SmStatus ApproximateSkinnedSurface
    (
      const SmContext                  & crContext,                 // in : context for new geometry construction                                                            
      const SmTArray<SmBSplineCurve *> & crCrossSectionCurves,      // in : CrossSectionCurves in order of occurrence on surface                                             
      SmBoolean                          bSyncronized,              // in : TRUE  = CrossSectionCurves have same degree and are synchronized                                 
      SmBoolean                         bPeriodic,                  // in : TRUE  = create NL_C1 smoothly closed surface if crCrossSectionCurves[0]=crCrossSectionCurves[k]  
                                                                    //    : FALSE = do not force NL_C1 closure if crCrossSectionCurves[0]=crCrossSectionCurves[k]            
      SmSurfParamType                    eCrossSectionSurfParam,    // in : SM_SP_U/SM_SP_V = CrossSectionCurves become surface U/V isoparameter curves.                     
      double                             dThisApproxTol3d,          // in : Approximation tolerance for surface                                     
      const SmTArray<double>           * pCrossSectionParams,       // in : Optional parameter value for each CrossSection Curve 
      SmBSplineSurface                *& rpNewBSplineSurface,       // out: The skinned surface                                                                              
      ULONG                              lOptDegree = 3             // in : sweep direction degree, default:[3]                                                              
    );

    // approximate a subsection of this surface bounded by 4 given curves to act as the natural boundaries of the new surface 
    SmStatus ApproximateSubSurface
    (
      const SmContext                 & crContext,      // in :    
      const SmTArray<SmBSplineCurve*> & cr3DCurves,     // in :    
      const SmTArray<SmBSplineCurve*> & crUVCurves,     // in :    
      SmBSplineSurface               *& rpNewSurf       // out:    
    );

    // Exposes N_FitSrfApproxRemoveKnots
    SmStatus ApproximateSurfaceRemoveKnots
    (
        const SmContext   & crContext,
        const SmTArray<double>  & crUParams,
        const SmTArray<double>  &crVParams,
        SmSurfParamType     eDir,
        const SmApproxTol3d & crApproxTol,
        SmBSplineSurface *& prNewSurface
    );

    // Exposes N_FitSurfApproxPoints()
    SmStatus ApproximateRefit
    (
      const SmTArray<SmPoint3d> & sApproxPts,           // in :     
      SmBoolean                   bFixEdgesU = FALSE,   // in :     
      SmBoolean                   bFixEdgesV = FALSE,   // in :     
      double                      dAlpha0 = -1,         // in :     
      double                      dAlpha1 = -1,         // in :     
      SmTArray< double >        * pErrors = NULL        // in :     
    );

    // fit surface to a grid of points sampled from this surface.
    //  Calls SmBSplineSurafce::ApproximatePoints() and returns a degree 3 nonRational BSplineSurface.
    SmStatus RebuildSurface
    (
      const SmContext   & crContext,      // in : new object context                                           
      double            * dOptTol,        // in : NULL = use N_FitSrfLstSqApprox() to approximate              
                                          //    : a (lNumRows)x(lNumCols) controlPoint surface to crPoints     
                                          //    : NotNULL = use N_FitSrfApproxTol() to approximate             
                                          //    : a possible smaller controlPoint count surfact to crPoints    
                                          //    : to within specified tolerance.  Larger tolerances yield      
                                          //    : smaller control point surfaces.                              
      double            * dMaxDiff,       // out: max deviation of result from input surface                   
      SmBSplineSurface *& rpNewSurf,      // out: new approx surface                                           
      ULONG               lNumPtsU = 21,  // in : number of control points in the u direction                  
      ULONG               lNumPtsV = 21   // in : number of control points in the v direction                  
    );                                   

    virtual SmStatus CalculateBoundingBox
    (
      const SmExtent2d  & crUVDomain,                   // in : only whole surfaces are bounded (except for polarBoxes of analytic surfaces)         
      SmExtent3d        * pNormalBox = NULL,            // out: Axis alligned box                                                                    
      SmPseudoBox       * pPseudoBox = NULL,            // out: Non-axis aligned box                                                                 
      SmPolarBox        * pPolarBox = NULL,             // out: Surface normal vector field bounding box                                             
      const SmPseudoBox * pOptPseudoBasisGuess = NULL,  // in : guess for PseudoBox basis vectors - used unless another better orientation is found  
      const SmPolarBox  * pOptPolarBasisGuess = NULL,         // in : guess for PolarBox basis vectors - used unless another better orientation is found   
      SmBoolean           bExpandPosBoxesByZoneTol3d = TRUE)  // in : TRUE = returned Normal & Pseudo BBoxes = BBox->ExpandAbsoluate(ZoneTol3d)
     const;                                                   //    : FALSE= returned Normal & Pseudo BBoxes = BBox with no expansion 
                                                              //      default:[TRUE] = previous behavior
    
    SmStatus CalculateCrossBoundaryDerivative
    (
      const SmContext & crContext,      // in : context for new object construction                  
      SmSurfParamType   eSurfDir,       // in : SM_SP_U,  NL_LEFT  : derivative across u=umin        
                                        //    :           NL_RIGHT : derivative across u=umax        
                                        //    : SM_SP_V,  NL_BOTTOM: derivative across v=vmin        
                                        //    :           NL_TOP   : derivative across v=vmax        
      SmBoolean         bIsAtMin,       // in : TRUE  - SM_SP_U=>NL_LEFT  and SM_SP_V=>NL_BOTTOM     
                                        //    : FALSE - SM_SP_U=>NL_RIGHT and SM_SP_V=>NL_TOP        
      SmBSplineCurve *& rpDerivCurve    // out: CrossDeriv curve                                     
    );

    virtual SmStatus CalculatePartialNetValues
    (
      const SmExtent2d & crUVDomain,                  // in : target domain               
      SmPoint3d          aCorners[2][2],              // out: aCorners[1][0] = U=1, V=0   
      SmExtent3d       * pOptNormalBox = NULL,        // out:                             
      SmPseudoBox      * pOptPseudoBox = NULL,        // out:                             
      SmPoint2d        * pOptUVChordHeight = NULL,    // out:                             
      SmPoint2d        * pOptUVAngleTolDegree = NULL  // out:                             
    ) const;

    SmStatus ConvertToPowerBasis();

    // Surface is already a BSpline - make and return an exact copy
    virtual SmStatus MakeExactBSplineIfPossible(SmBSplineSurface *& rpNewBSplineSurface) const 
    { 
      SmStatus sRtn  = Copy(*GetContext(), (SmSurface *&)rpNewBSplineSurface) ; 
      return(sRtn) ;    
    } 

    // make an exact copy of any surface
    virtual SmStatus Copy
    (
      const SmContext & crContext,    // in :
      SmSurface      *& rpNewSurface  // out:
    ) const;

    // Copy of a SubPatch of BSplineSurface 
    SmStatus CopySubPatch
    (
      const SmContext   & crContext,         // in : Context for new object construction                               
      const SmExtent2d  & crUVDomain,        // in : Domain of NewSurface (snapped to existing knots within 1.0e-10)   
      SmBSplineSurface *& rpNewSurface       // out: NewSurface = ThisSurface over crUVDomain                          
    ) const;

    // when possible make a SmBSpline Copy of an Analytic Surface - else set output to NULL
    virtual SmStatus CopyAnalyticAsNurb
    (
      const SmContext & crContext,            // in : context for new object construction              
      SmSurface      *& rpNewNurbSurface      // out: newly copied SmBSplineSurface of appropriate     
                                              //    : analytic derived type when analytic              
    ) const ;
    
    // when possible make an analytic copy of a BSplineSurface - else just make an exact copy                          
    virtual SmStatus CopyAndAddAnalytics
    (
      const SmContext & crContext,            // in : context for new object construction                
      SmSurface      *& rpNewSurface          // out: newly copied SmBSplineSurface of appropriate       
                                              //    : analytic derived type when analytic                
    ) const;

    static SmStatus CreateBilinearSurface
    (
      const SmContext   & crContext,          // in : context for new object construction                                
      const SmPoint3d   & crU0V0Corner,       // in : ControlPoint[0][0] of 4 corner points to interpolate               
      const SmPoint3d   & crU1V0Corner,       // in : ControlPoint[1][0] of 4 corner points to interpolate               
      const SmPoint3d   & crU0V1Corner,       // in : ControlPoint[0][1] of 4 corner points to interpolate               
      const SmPoint3d   & crU1V1Corner,       // in : ControlPoint[1][1] of 4 corner points to interpolate               
      SmBSplineSurface *& rpNewBSplineSurface // out: NonRational Bilinear BSplineSurface interpolating given corners    
    );

    static SmStatus CreateCanonical
    (
      const SmContext           & crContext,              // in : context for new object construction                            
      ULONG                       lUDegree,               // in : U dir polynomial degree. supported range: [1 through 32]       
      ULONG                       lVDegree,               // in : V dir polynomial degree. supported range: [1 through 32]       
      const SmTArray<SmPoint3d> & crControlPointsList,    // in : control points,  (Euclidian coordinates)                       
      SmBSplineSurfaceForm        eBSplineSurfaceForm,    // in : oneof SM_SF_PLANE_SURF,   SM_SF_CYLINDRICAL_SURF,              
                                                          //    :       SM_SF_CONICAL_SURF, SM_SF_SPHERICAL_SURF,                
                                                          //    :       SM_SF_RULED_SURF,   SM_SF_SURF_OF_REVOLUTION,            
                                                          //    :       SM_SF_QUADRIC_SURF, SM_SF_GENERALIZED_CONE,              
                                                          //    :       SM_SF_UNSPECIFIED,  SM_SF_SURF_OF_LINEAR_EXTRUSION,           
                                                          //    :       SM_SF_POLYNOMIAL,   SM_SF_HELICAL_SWEEP                  
      const SmTArray<ULONG>     & crUMultiplicities,      // in : U dir multiplicities for each associated knot                  
      const SmTArray<ULONG>     & crVMultiplicities,      // in : v dir multiplicities for each associated knot                  
      const SmTArray<double>    & crUKnots,               // in : U dir unique knot values                                       
      const SmTArray<double>    & crVKnots,               // in : v dir unique knot values                                       
      SmKnotType                  eKnotType,              // in : oneof SM_KT_UNIFORM_KNOTS, SM_KT_QUASI_UNIFORM_KNOTS,          
                                                          //    :       SM_KT_UNSPECIFIED,   SM_KT_PIECEWISE_BEZIER_KNOTS        
      const SmTArray<double>    * cpOptWeights,           // in : optional associated ControlPoint weights, NULL for nonRational 
      const SmExtent2d          * cpOptUVDomain,          // in : optional domain whose min/max values are used to overwrite     
                                                          //    : the input crUKnots and crVKnots first and last values.         
                                                          //    : NUll to ignore.                                                
      SmBSplineSurface         *& rpNewBSplineSurface     // out: The created BSplineSurface                                     
    );

    static SmStatus CreateConePatch
    (
      const SmContext        & crContext,                 // in :                                                                                     
      const SmAxis2Placement & crReferenceFrame,          // in :                                                                                     
      double                   dBottomRadius,             // in :                                                                                     
      double                   dTopRadius,                // in : If equal to dBottomRadius a cylinder is created                                     
      double                   dStartAngleDeg,            // in :                                                                                     
      double                   dEndAngleDeg,              // in :                                                                                     
      double                   dHeight,                   // in : Perpendicular height measured along the axis of cylinder/cone                       
      SmNurbCircleParam        eParameterization,         // in : SM_CO_QUADRATIC - produces a surface using a quadratic (degree 2) parameterization. 
                                                          //    : SM_CO_QUINTIC - produces a surface using a quintic (degree 5) parametrization.      
      SmBSplineSurface      *& rpNewBSplineSurface        // out: Resulting new cone surface                                                          
    );

    static SmStatus CreateConeThroughBox
    (
      const SmContext        & crContext,                 // in :                                                                                       
      const SmAxis2Placement & crReferenceFrame,          // in :                                                                                       
      double                   dRadius,                   // in : Radius of cone corresponding to a plane through the origin of the reference frame.    
      double                   dSemiAngleDeg,             // in : Angle between cone axis and surface of cone.                                          
      double                   dStartAngleDeg,            // in :                                                                                       
      double                   dEndAngleDeg,              // in :                                                                                       
      SmNurbCircleParam        eParameterization,         // in : SM_CO_QUADRATIC - produces a surface using a quadratic (degree 2) parameterization.   
                                                          //    : SM_CO_QUINTIC - produces a surface using a quintic (degree 5) parametrization.        
      const SmExtent3d       & crBoundingBox,             // in :                                                                                       
      SmBSplineSurface      *& rpNewBSplineSurface        // out:                                                                                       
    );

    static SmStatus CreateCoonsPatch
    (
      const SmContext                 & crContext,            // in : context for new object construction                             
      double                            dTol,                 // in : max allowed patch/boundary curve deviation                      
      SmBoolean                         bCreateBilinearPatch, // in : TRUE = Create bilinear patch (ignore crDerivCrvs)               
                                                              //    : FALSE= Create bicubic patch through curves and crossTangents    
      const SmTArray<SmBSplineCurve*> & crBdryCrvs,           // in : ordered array of 4 boundary curves                              
      const SmTArray<SmBSplineCurve*> & crDerivCrvs,          // in : associated array of 4 crossTangent functions                    
      SmBSplineSurface               *& rpCoonsSurface        // out: the new coons patch                                             
    );

    static SmStatus CreateCornerBlend
    (
      const SmContext                   & crContext,                    // in : context for new object construction                                                     
      const SmTArray<SmCurve*>          & cr3DCurves,                   // in : ordered array of 3d boundary curves                                                     
      const SmTArray<SmOrientType>      & crCurveOrients,               // in : SM_OT_SAME     = associated cr3DCurve parameterized in direction of Curve order         
                                                                        //    : SM_OT_OPPOSITE = associated cr3DCurve parameterized opposite to direction of Curve order
      const SmTArray<SmBSplineCurve*>   & crUVCurves,                   // in : associated UVTrimCurves                                                                 
                                                                        //    : In the future we may allow NULL in this and the                                         
                                                                        //    : surfaces array to indicate that there is no tangency                                    
                                                                        //    : for this edge, for now they are required                                                
      const SmTArray<SmSurface*>        & crSurfaces,                   // in : associated Surfaces                                                                     
      double                              dThisApproxTol3d,             // in : max allowed 3D deviation from blendSurfaces to 3DCurves                                 
      double                              dTangencyTolRadians,          // in : max allowed angle deviation along tangent edges                                         
      SmTArray<SmSurface*>              & rBlendSurfaces,               // out: blended surface set                                                                     
      SmBoolean                           bCreateBilinearPatch = FALSE  // in : TRUE = Create bilinear patch (ignore crDerivCrvs)                                       
                                                                        //    : FALSE= Create bicubic patch through curves and crossTangents                            
    );

    static SmStatus CreateCylinderThroughBox
    (
      const SmContext        & crContext,           // in :                                                                                      
      const SmAxis2Placement & crReferenceFrame,    // in :                                                                                      
      double                   dRadius,             // in :                                                                                      
      double                   dStartAngle,         // in :                                                                                      
      double                   dEndAngle,           // in :                                                                                      
      SmNurbCircleParam        eParameterization,   // in : SM_CO_QUADRATIC - produces a surface using a quadratic (degree 2) parameterization.  
                                                    // in : SM_CO_QUINTIC - produces a surface using a quintic (degree 5) parametrization.       
      const SmExtent3d       & crBoundingBox,       // in :                                                                                      
      SmBSplineSurface      *& rpNewBSplineSurface  // out:                                                                                      
    );                                                                                                                                            

    SmStatus CreateExtendedSurface
    (
        const SmContext   & crContext,              // in : context for new object construction                      
        double              dDist,                  // in : Distance of extension from each side                     
        SmContinuityType    eExtensionContinuity,   // in : OneOf: SM_CT_G1 - linear extension                       
                                                    //    :      SM_CT_G1R -                                         
                                                    //    :      SM_CT_G1_G2 - extension with second derivative      
                                                    //    :      SM_CT_CINFINITY - infinite continuity               
        SmSurface        *& rpExtended              // out: the newly constructed surface (NULL on input)            
    );

    SmStatus CreateExtendedSurface
    (
        const SmContext   & crContext,              // in : context for new object construction                       
        SmSurfParamType     eExtDirection,          // in : Either extend in SM_SP_U or SM_SP_V direction             
                                                    //    : or SM_SP_UMIN/VMIN/UMAX/VMAX/BOTH                         
        double              dDist,                  // in : Distance of extension from each side                      
        SmContinuityType    eExtensionContinuity,   // in : oneof: SM_CT_G1 - linear extension                        
                                                    //    :      SM_CT_G1R -                                          
                                                    //    :      SM_CT_G1_G2 - extension with second derivative       
                                                    //    :      SM_CT_CINFINITY - infinite continuity                
        SmSurface        *& rpExtended              // out: the newly constructed surface (NULL on input)
    );

    SmStatus JoinSurface           
    (
      const SmContext  & crContext,                 // in : context for new object construction                 
      SmBSplineSurface  * pSurfaceToJoin,           // in : Surf to join to 'this' surf (neither surf modified) 
      SmSurfParamType     eDirFlag,                 // in: Direction of parameter flow across join.
                                                    //     SM_SP_U = join this->MaxV to pSurfaceToJoin->MinV isocurve (surfs made compatible in v param)
                                                    //     SM_SP_V = join this->MaxU to pSurfaceToJoin->MinU isocurve (surfs made compatible in u param)
      double              dTolerance,               // in : Knot removal tolerance.  The knot corresponding     
                                                    //    : to the merged boundary has multiplicity equal       
                                                    //    : to the degree.  Knot removal will be attempted      
                                                    //    : using this tolerance.                               
      SmBSplineSurface *& rpJoinedSurface           // out: joined surface (newly allocated), NULL on input
    );

    static SmStatus CreateFromBoundary
    (
      const SmContext                 & crContext,        // in :                                
      const SmTArray<SmPoint3d>       & InteriorPoints,   // in : sample interior points         
      const SmTArray<SmBSplineCurve*> & crUCurves,        // in : 2 u surface boundary curves    
      const SmTArray<SmBSplineCurve*> & crVCurves,        // in : 2 v surface boundary curves    
      const double                      dTol,             // in : fit tolerance for surface      
      SmBSplineSurface               *& rpNewSurf         // out: output surface                 
    );

    static SmStatus CreateGordonSurface
    (
      const SmContext                 & crContext,        // in :            
      const SmTArray<SmBSplineCurve*> & crUCurves,        // in :            
      const SmTArray<SmBSplineCurve*> & crVCurves,        // in :            
      SmBSplineSurface               *& rpNewSurf         // out:            
    );

  virtual SmStatus CreateIsoParametricCurve
  (
    const SmContext  & crContext,               // in : context for created objects                                                                  
    SmSurfParamType    eSurfParam,              // in : Defines which Nurb parameter direction on surface to extract curve from                      
                                                //    : SM_SP_U = create constant u isoParameter curve                                               
                                                //    : SM_SP_V = create constant v isoParameter curve                                               
    double             dIsoParameter,           // in : Defines Nurb parametric value at which to extract the curve.                                 
                                                //    : If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V                         
                                                //    : then this is the V parameter                                                                 
    SmApproxTol3d      sApproxTol3d,            // NotUsed: in : passed to ApproximateCurve() when approximation is required.                                 
                                                //    : If set to 0.0, tolerance is set by system: old[curve length * 1.0e-4] new[GetApproxTol3d()]  
    SmBSplineCurve  *& rpNewIsoCurve,           // out: 3d IsoParameterCurve                                                                         
    const SmExtent2d * pOptDomain = NULL,       // in : optional trim bound for IsoParameterCurve, NULL to ignore, default:[NULL]                    
    double           * pOptMaxGap3d = NULL,     // out: opt achieved max gap, NULL to ignore, default:[NULL]                                         
    SmCurve         ** pOptUVIsoCurve = NULL    // out: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]              
  ) const;
                                                            
    SmStatus CreateIsoParametricDerivCurve
    (
      const SmContext  & crContext,             // in : context for created objects                                               
      ULONG              lDerCnt_U,             // in : desired derivative in U direction (0=none,1=1st,2=2nd,...).               
                                                //    : less than degree p for NUBs and less than p-1 for NURBs.                  
      ULONG              lDerCnt_V,             // in : desired derivative in V direction (0=none,1=1st,2=2nd,...).               
                                                //    : less than degree q for NUBs and less than q-1 for NURBs.                  
      SmSurfParamType    eSurfParam,            // in : Defines which parameter direction on surface to extract curve from        
                                                //    : SM_SP_U = create constant u isoParameter curve                            
                                                //    : SM_SP_V = create constant v isoParameter curve                            
      double             dIsoParameter,         // in : Defines parametric value at which to extract the curve.                   
                                                //    : If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V      
                                                //    : then this is the V parameter                                              
      double             d3DTolerance,          // in : d3DTolerance = Not used in this method                                    
      SmBSplineCurve  *& rpNewIsoDerivCurve,    // out: 3d IsoParameterCurve                                                      
      const SmExtent2d * pOptDomain = NULL      // in : optional trim bound for IsoParameterCurve
    ) const;                     

    static SmStatus CreateLinearSweep
    (
      const SmContext      & crContext,           // in : context for new object construction              
      const SmBSplineCurve & crCurveToSweep,      // in : target Curve                                     
      const SmVector3d     & crSweepVector,       // in : Defines magnitude and direction of the sweep     
      SmBSplineSurface    *& rpNewBSplineSurface  // out: The swept surface with new domain                
                                                  //    : UVDomainMin = { (0.0, CurveToSweep_Ivl->Min),    
                                                  //    :                 (1.0, CurveToSweep_Ivl->Max) }   
    );

    virtual SmStatus CreateMirrorSurface
    (
      const SmContext        & crContext,         // in :       
      const SmAxis2Placement & crMirrorPlane,     // in :       
      SmSurface             *& rpMirrorSurface    // out:       
    ) const;

    static SmStatus CreateNSidedPatch
    (
      const SmContext                 & crContext,                // in : context for new object construction                                       
      const SmTArray<SmBSplineCurve*> & crBdryCrvs,               // in : NON-RATIONAL  boundary curves; cur[0],...,cur[k]                          
                                                                  //    : must be  input  consecutively along  the n-sided                          
                                                                  //    : boundary and each curve parameterized in the same direction               
      const SmTArray<SmBSplineCurve*> & crDerivCrvs,              // in : NON-RATIONAL compatible corresponding cross-derivative curves.            
                                                                  //    : der=NULL:No curves specified, else der[i]=NULL:just der[i] not specified. 
                                                                  //    : specified der values approximated to eps angle in degrees                 
      double                            dAngleTol,                // in : MaxAngDeg allowed between NSidedPatch cross-derivative                    
                                                                  //    : vectors and associated input crDerivCrvs values.                          
      SmTArray<SmSurface*>            & rSurfaces,                // out: (k+1) surfaces, interpolating boundary curves, and                        
                                                                  //    :  approximating cross-boundary derivatives                                 
      SmPoint3d                       * pOptCenterPoint = NULL,   // in : Point on plane tangent to NSidedPatch in its center.                      
      SmVector3d                      * pOptCenterNormal = NULL   // in : Normal of plane tangent to NSidedPatch in its center.                     
    ) ;

    // create surface approximation of the surface offset - Out Surf->Domain(s) may be trimmed but not scaled
    virtual SmStatus CreateOffsetSurface
    (
      const SmContext      & crContext,               // in : context for new obj construction                                              
      double                 dSignedOffsetDistance,   // in : offset dist, (neg val = Offset dir opposite surface normal)                   
      SmApproxTol3d          sApproxTol3d,            // in : Max Dist between ApproxOffsetSurface and ideal offset shape                   
      SmSurface* &           rOffsetSurface           // out: Offset Surf Approx, result may have self-intersections                        
    ) const;

    // build SmBSplineSurface approximation to exact offset surface
    SmStatus CreateOffsetWithPolygon
    (
      const SmContext      & crContext,               // in : context for new object construction                             
      double                 dSignedOffsetDistance,   // in : offset distance (negative for insets)                           
      double                 dApproxTol,              // in : max allowed approximation distance                              
      ULONG                  lSubdivisionLevel,       // in : How many times to split spans before giving up                  
                                                      //    : 0-no subdivisions,                                              
                                                      //    : 1-split spans a max of 1 time, (1 patch becomes 4)              
                                                      //    : 2-split spans a max of 2 times (1 patch becomes 8) . .          
      SmTArray<SmSurface*> & rOffsetSurfaces          // out: Contains 1 approximation surface with the smallest number       
                                                      //    : of subdivisions possible that is within tolerance               
                                                      //    : of exact offset surface.                                        
                                                      //    : Contains no surfaces when finest allowed subdivision surface    
                                                      //    : is still out of tolerance                                       
    ) const;

    // create surface as linear interpolation between common parameter points of two g1 section curves
    static SmStatus CreateRuledSurface
    (
      const SmContext      & crContext,            // in : context for new object construction                           
      const SmBSplineCurve & crCurve1,             // in : Shape for Min U or V surface isoparam curve                   
      const SmBSplineCurve & crCurve2,             // in : Shape for Max U or V surface isoparam curve                   
      SmSurfParamType        eLinearSurfParam,     // in : SM_SP_U = Surface U dir is linear, input curves vary in V     
                                                   //    : SM_SP_V = Surface V dir is linear, input curves vary in U     
      SmBSplineSurface    *& rpNewBSplineSurface   // out: The new ruled surface                                         
    );

    // Create Ruled surface after modifying section curves - curve knots are dampened and then made compatible
    static SmStatus CreateRuledSurfaceSafe
    (
      const SmContext   & crContext,            // in : context for new object construction                             
      SmBSplineCurve    & crCurve1,             // in : Shape for Min U or V surface isoparam curve                     
      SmBSplineCurve    & crCurve2,             // in : Shape for Max U or V surface isoparam curve                     
      SmSurfParamType     eLinearSurfParam,     // in : SM_SP_U = Surface U dir is linear, input curves vary in V       
                                                //    : SM_SP_V = Surface V dir is linear, input curves vary in U       
      SmBSplineSurface *& rpNewBSplineSurface   // out: The new ruled surface                                           
    );

    // create surface from family of cross-section curve and optional 1 or 2 rail curves
    static SmStatus CreateSkinnedSurface
    (
      const SmContext                  & crContext,                 // in : context for new geometry construction                                               
      const SmTArray<SmBSplineCurve *> & crCrossSectionCurves,      // in : CrossSectionCurves in order of occurrence on surface                                
      SmBoolean                          bSyncronized,              // in : TRUE    = CrossSectionCurves have same degree and are synchronized                  
      SmSurfParamType                    eCrossSectionSurfParam,    // in : SM_SP_U/SM_SP_V = CrossSectionCurves become surface U/V isoparameter curves.        
      double                             dThisApproxTol3d,          // in : 0.0 = skinning interpolates crossSections, else approximate.                        
      const SmBSplineCurve             * pRail1,                    // in : Min param sideBoundary curve of surface. Null to ignore.                            
      const SmBSplineCurve             * pRail2,                    // in : Max param sideBoundary curve of surface. Null to ignore.                            
      SmBoolean                          bRail1UsedAsSpineCurve,    // in : TRUE = use pRail1 as spine curve. (don't use pRail2)                                
                                                                    //    : Note: When both rails are specified they must be syncronized.                       
                                                                    //    : Note: CrossSections positioned along rails at pIntersectionsWithRails param values  
      const SmTArray<double>           * pIntersectionsWithRails,   // in : Rail params of CrossSectionCurve/Rail intersections.                                
      SmBSplineSurface                 * pDerivSurf[2],             // in : pDerivSurf[0] = skinned surface start CrossBoundary derivatives. NULL to ignore.    
                                                                    //    : pDerivSurf[1] = skinned surface end CrossBoundary derivatives. NULL to ignore.      
                                                                    //    : not used when bRail1UsedAsSpineCurve == TRUE                                        
      SmBSplineSurface                *& rpNewBSplineSurface,       // out: The skinned surface                                                                 
      ULONG                              lOptDegree = 3             // in : sweep direction degree, default:[3]                                                 
    );

    static SmStatus CreateSphereFromArcs
    (
      const SmContext                 & crContext,                  // in : context for new object construction                                
      const SmTArray<SmBSplineCurve*> & crArcs,                     // in : boundary curves to fill with a Spherical Surface - if possible     
      double                            d3DTolerance,               // in : Max allowed gap between NewSphere and input Arcs                   
      SmTArray<SmCurve*>              & rCCWArcs,                   // out: These are the same curves as passed in                             
                                                                    //    : by crArcs ordered CCW about the sphere                             
      SmTArray<SmOrientType>          & rOrients,                   // out: Orientation of individual curves relative                          
                                                                    //    : to the CCW orientation about the surface.                          
      SmBSplineSurface               *& rpNewSphere                 // out: SurfaceOfRevolution with small gaps to crArcs, else NULL           
    );
    
    // create surface by sweeping a section curve around an axis of rotation                                     
    static SmStatus CreateSurfOfRevolution
    (
      const SmContext        & crContext,             // in : new object context                                              
      const SmBSplineCurve   * pGenCurve,             // in : Generating curve                                                
      const SmPoint3d        & crOrigin,              // in : Origin of the axis of revoluation                               
      const SmVector3d       & crAxisDir,             // in : Direction vector of axis of revolution                          
      double                   dAngleSpanDeg,         // in : Angle of revolution (> 0.0 && <= 360.0) in degrees              
      SmBSplineSurface      *& rpNewBSplineSurface,   // out: The new surface                                                 
      SmBoolean                bSwapUV = FALSE,       // in : TRUE = Degree 2 circular U direction (constant V iso curves)    
                                                      //    : FALSE= Degree 2 circular V direction (constant U iso curves)    
                                                      //    : default:[FALSE]                                                 
      const SmCircle         * pOptSweepCurve = NULL  // in : Opt Surface SweepDirection Parameterization specification.      
                                                      //    : Circle must be centered upon and normal to the Revolution axis. 
    ) ;

    // create surface by sweeping a curve along a spiral path
    static SmStatus CreateHelicalSweep
    (
      const SmContext        & crContext,             // in : context for new object construction                                                         
      const SmAxis2Placement & crReferenceFrame,      // in : Helix centered on Z Axis starting at Z=0 on X Axis, running to Z=Height                     
      const SmBSplineCurve   & crPlanarCurve,         // in : Planar BSplineCurve built in the RefFrame's Z=0 plane                                       
      double                   dHeight,               // in : Helix Length along RefFrame Z Axis                                                          
      double                   dRadiusStart,          // in : Helix Radius at RefFrame Z = 0      (linearly interpolated from 0 to Height)                
      double                   dRadiusEnd,            // in : Helix Radius at RefFrame Z = Height (linearly interpolated from 0 to Height)                
      double                   dNumTurns,             // in : Num of 360 deg Helix Turns in H=[0,Height] (Fractions okay), PeriodLength:[Height/NumTurns] 
      SmBoolean                bRightHanded,          // in : TRUE = spiral is right handed, FALSE=left handed                                            
      SmApproxTol3d            sApproxTol3d,          // in : Max allowed deviation between NewBSplineCurve and theoretical Helix                         
      SmBSplineSurface      *& rpNewBSplineSurface    // out: The new Surface = Helical Sweep of input Surface.                                           
    );

    // create surface by sweeping (with optional scaling) a section curve along a trajectory curve
    static SmStatus CreateSweepSurface
    (
      const SmContext      & crContext,               // in : context for new object construction                                      
      const SmBSplineCurve * cpSectionCurve,          // in : u direction curve for new Surface                                        
      const SmBSplineCurve * cpTrajectoryCurve,       // in : v direction curve for new Surface                                        
      const SmBSplineCurve * cpOptScaleCurve,         // in : x,y,z scale values for SectionCurve for each v value                     
                                                      //    : ScaleCurve->NaturalInterval must equal TrajectoryCurve->NaturalInterval  
      SmBoolean              bDoTranslationalSweep,   // in : TRUE = NewSurf(u,v) = SectionCurve(u) + TrajectoryCurve(v)               
                                                      //    :        this is an exact surface.                                         
                                                      //    :        (cpOptScaleCurve and dThisApproxTol3d not used)                   
                                                      //    : FALSE= NewSurf(u,v) =   A(v) * ScaleCurve(v) * SectionCurve(u)           
                                                      //    :                     + TrajectoryCurve(v)                                 
                                                      //    :        this is an approximate surface.                                   
      double                 dThisApproxTol3d,        // in : Max distance between exact surface and newSurf approximation             
      SmBSplineSurface    *& rpNewSurf                // out: new sweptSurface
    );

    // create surface by sweeping a series of section curves along
    // trajectory curves -- currently, two.
    static SmStatus CreateSweepSurface
    ( 
      const SmContext                   & crContext,             // in :                          
      const SmTArray< SmBSplineCurve* > & rProfiles,             // in :                          
      const SmTArray< SmBSplineCurve* > & rRails,                // in :                          
      double                              dTol,                  // in : optional.  See notes.    
      SmBSplineSurface                 *& rpNewSurf,             // out:                          
      ULONG                               lNumBetweenProfiles=4, // in : optional.  See notes.    
      ULONG                               lBlendLevel = 2        // in : optional.  See notes.    
    );

    // create generalized SurfOfRotation replacing the circular sweep with a general trajectory curve
    static SmStatus CreateSwungSurface
    (
      const SmContext      & crContext,             // in :              
      const SmBSplineCurve * cpSectionCurve,        // in : XZ plane     
      const SmBSplineCurve * cpTrajectoryCurves,    // in : XY Plane     
      double                 dScaleFactor,          // out:              
      SmBSplineSurface    *& rpNewSurf              // out:              
    );

    static SmStatus CreateTrimmedPlaneFromRuled
    (
      const SmContext         & crContext,          // in :       
      const SmBSplineCurve    & crCurve1,           // in :       
      const SmBSplineCurve    & crCurve2,           // in :       
      SmTArray<SmCurve*>      & r3DTrimmingCurves,  // in :       
      SmTArray<SmOrientType>  & rTrimOrientations,  // in :       
      SmBSplineSurface       *& rpPlanarNurb        // out:       
    );                       
    
    virtual SmStatus DegreeElevate
    (
      SmSurfParamType eDirectionToElevate,  // in : u- or v-direction    
      ULONG           lNewDegree            // in :                      
    );

    virtual SmStatus DegreeReduction
    (
      double          dTolerance,                     // in :                                                                        
      SmBoolean       bMaxReduce = TRUE,              // in : If TRUE, will reduce the degree as much as possible (both u- & v-dirs) 
      SmSurfParamType eDirectionToReduce = SM_SP_U    // in : Specify u- or v-direction if reduce by only 1 degree                   
    );

    virtual SmStatus Evaluate
    (
      const SmPoint2d & crUV,                  // in : param value to evaluate                                                                  
      ULONG             lHighestUDeriv,        // in : number of U derivatives                                                                  
      ULONG             lHighestVDeriv,        // in : number of V derivatives to compute                                                       
      SmBoolean         bUFromLeft,            // in : if P is on U interval boundary                                                           
                                               //    : TRUE  = evaluate P in upper interval where P is on the left of the interval              
                                               //    : FALSE = evaluate P in lower interval where P is on the right of the interval             
      SmBoolean         bVFromLeft,            // in : if P is on V interval boundary                                                           
                                               //    : TRUE  = evaluate P in upper interval where P is on the left of the interval              
                                               //    : FALSE = evaluate P in lower interval where P is on the right of the interval             
      SmBoolean         bOnlyUpperHalf,        // in : TRUE=compute upper half of matrix only                                                   
                                               //    : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value            
                                               //    :           [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)       
                                               //    :                       [Dvv  ---   ---]                                                   
      SmVector3d      * aDerivatives,          // out: matrix of evaluations values                                                             
                                               //    : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                               
                                               //    : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)    
                                               //    :              [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )     
                                               //    :              [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                        
                                               //    :              [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                        
                                               //    : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...] 
      SmBoolean         bNonZeroTangents=TRUE, // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors             
                                               //    : FALSE= return exact tangent values                                                       
                                               //    : note: Surprisingly TRUE is the common choice because most tangent uses                   
                                               //    :     are for their direction (Binorm, SurfNorm comps), but when the                       
                                               //    :     tangent is being used for its magnitude (like an arc-length comp)                    
                                               //    :     then set this to FALSE.                                                              
                                               //    : default:[TRUE]                                                                           
      SmBoolean         bDoZeroSampling=TRUE   // in : for internal use only, always set to TRUE, default:[TRUE]                                
    ) const ;

    virtual SmStatus EvaluateSimple
    (
      const SmPoint2d & crUV,             // in : target surface point                                                                     
      ULONG             lHighestUDeriv,   // in : Requested highest U derivative                                                           
      ULONG             lHighestVDeriv,   // in : Requested highest V derivative                                                           
      SmBoolean         bUFromLeft,       // in : if P is on U interval boundary                                                           
                                          //    : TRUE  = evaluate P in upper interval where P is on the left of the interval              
                                          //    : FALSE = evaluate P in lower interval where P is on the right of the interval             
      SmBoolean         bVFromLeft,       // in : if P is on V interval boundary                                                           
                                          //    : TRUE  = evaluate P in upper interval where P is on the left of the interval              
                                          //    : FALSE = evaluate P in lower interval where P is on the right of the interval             
      SmBoolean         bOnlyUpperHalf,   // in : TRUE=compute upper half of matrix only                                                   
                                          //    : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value            
                                          //    :           [Dv --]       [Dv   Duv   ---]                                                 
                                          //    :                         [Dvv  ---   ---]                                                 
      SmVector3d      * aDerivatives      // out: matrix of evaluations values                                                             
                                          //    : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                               
                                          //    : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]                                       
                                          //    :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]                                       
                                          //    :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                       
                                          //    :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                       
                                          //    : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...] 
    ) const;

    virtual SmStatus EvaluatePoint
    (
      const SmPoint2d & crUV,             // in : Target Nurb UV Point      
      SmPoint3d       & rPoint            // out: Resulting Image Point     
    ) const;

    SmStatus EvaluatePointOnExtendedSurface
    (
      const SmPoint2d & crUV, // in :       
      SmPoint3d       & rPnt  // out:       
    ) const;

    // extend boundary span domain to expand NaturalUVDomain - only use for tolerance sized extensions
    SmStatus ExpandBoundarySpan
    (
      double          dNewUorV,               // in : desired new U or V NaturalDomain boundary value    
      SmSurfParamType eCrossSectionSurfParam  // in : SM_SP_U = setting a U boundary value               
                                              //      SM_SP_V = setting a V boundary value               
    ) ;

    SmStatus FindSpans
    (
      const SmPoint2d & crUV,
      SmBoolean         bUFromLeft,       // in : if P is on U interval boundary                                                 
                                          //    : TRUE  = evaluate P in upper interval where P is on the left of the interval    
                                          //    : FALSE = evaluate P in lower interval where P is on the right of the interval   
      SmBoolean         bVFromLeft,       // in : if P is on V interval boundary                                                 
                                          //    : TRUE  = evaluate P in upper interval where P is on the left of the interval    
                                          //    : FALSE = evaluate P in lower interval where P is on the right of the interval   
      ULONG           & rUIndex,          // out:                                                                                
      ULONG           & rVIndex           // out:                                                                                
    ) const;

    // Separate duplicate end Control points on non-degenerate ControlPolygon rows and cols
    virtual SmBoolean FixRepeatedEndControlPoints() ;

    // simple data access
    virtual ULONG              GetDegree               (SmSurfParamType eSurfParam) const;
    virtual SmExtent2d         GetNaturalUVDomain      () const;
    virtual SmExtent2d         GetMaxAnalyticDomain    () const;
    ULONG                      GetNumberNaturalKnots   (SmSurfParamType eSurfParam) const;
    virtual ULONG              GetNumberControlPoints  (SmSurfParamType eSurfParam) const;
    SmBSplineSurfaceForm       GetBSplineSurfaceForm   () const { return m_eBSplineSurfaceForm; }
    SmBoolean &                GetOutOfBoundsEnabled   ()       { return m_bOutOfBoundsEnabled; }
    gw_SURFACE *               GetGwNurbPointer        () const { return m_pNurb ; }  // retrieve the NLIB NURB struct
    gw_SURFACE *               GetOrCreateGwNurbPointer()       { if(m_pNurb==NULL) {MakeNurb() ; } 
                                                                  SM_ASSERT(m_pNurb != NULL) ;
                                                                  return m_pNurb ;
                                                                }

    virtual SmBSplineSurface * GetRootSurface() const           { return (SmBSplineSurface *)this ; }

    virtual SmStatus GetKnots                
    (
      SmSurfParamType    eSurfParam,                  // in : SM_SP_U = get U Knot Vector                                  
                                                      //    : SM_SP_V = get V knot Vector                                  
      SmTArray<double> & rUniqueKnots,                // out: unique knot values in requested dimension                    
      SmTArray<ULONG>  * pKnotMultiplicities = NULL,  // out: associated multiplicity for every knot                       
      const SmExtent1d * pOptIvl = NULL               // in : interval of interest, NULL=Natural Interval, default:[NULL]  
    ) const;

    SmStatus GetControlPointNet      
    (
      ULONG               & rlUCount,                 // out: U controlPoint count                                                   
      ULONG               & rlVCount,                 // out: V controlPoint count                                                   
      SmTArray<SmPoint3d> & rControlPointsList,       // out: Always produced in Euclidian coordinates                               
      SmTArray<double>    & rWeights                  // out: The Weights array will have size zero if the surface is non-rational.  
    ) const ;         

    SmStatus GetControlPoint         
    (
      SmControlPointFormType eCtrlPointForm,          // in : SM_CP_NON_RATIONAL         = do perspective projection and        
                                                      //    :                              set W=1.0 if it is rational.         
                                                      //    : SM_CP_HOMOGENEOUS_RATIONAL = don't do division and return W       
                                                      //    : SM_CP_EUCLIDIAN_RATIONAL   = do division and return W             
      ULONG                  lUIndex,                 // in : TgtCPoint U index value                                           
      ULONG                  lVIndex,                 // in : TgtCPoint V index value                                           
      SmPoint3d            & rControlPoint,           // out: rControlPoint = (  SM_CP_NON_RATIONAL         ? [X   Y   Z  ]     
                                                      //    :                  : SM_CP_HOMOGENEOUS_RATIONAL ? [X*W Y*W Z*W]     
                                                      //    :                  : SM_CP_EUCLIDIAN_RATIONAL   ? [X   Y   Z  ] ) ; 
      double               & rdWeight                 // out: rdWeight = (  SM_CP_NON_RATIONAL         ? 1.0                    
                                                      //    :             : SM_CP_HOMOGENEOUS_RATIONAL ?  W                     
                                                      //    :             : SM_CP_EUCLIDIAN_RATIONAL   ?  W ) ;                 
    ) const;

    SmStatus GetControlPointsPointer 
    (
      ULONG   & lControlPointCountU,      // out: number of control points in each U row                        
      ULONG   & lControlPointCountV,      // out: number of control points in each V row                        
      double *& pControlPoints            // out: array of controlPoints stored as doubles                      
                                          //    : [ x00, y00, z00, w00,                                         
                                          //    :   x01, y01, z01, w01, ..                                      
                                          //    :   . . .          w0VCount-1,                                  
                                          //    :   x10, y10, z10, w10, x11... ]                                
                                          //    :   For 2d control Point   z == NL_NOZ or 0.0                   
                                          //    :   for nonRational points w == NL_NOW                          
                                          //    :   for Rational points x,y,z are stored in homogeneous space   
                                          //    :   i.e. CartesianX = x/w                                       
                                          //    :   CartesianY = y/w                                            
                                          //    :   CartesianZ = (z != NL_NOZ) ? z/w : NL_NOZ ;                 
    ) const ;

    SmStatus GetKnotsPointers        
    (
      ULONG   & lKnotCountU,    // out: number of knots in U direction      
      ULONG   & lKnotCountV,    // out: number of knots in V direction      
      double *& pKnotsU,        // out: ptr to array of U-knots             
      double *& pKnotsV         // out: ptr to array of V-knots             
    ) const;

    SmStatus GetMeasures             
    (
      double & rdAverageLengthU,    // out:   
      double & rdAverageLengthV,    // out:   
      double & rdEstimatedAreaBound // out:   
    );

    SmStatus GetKnotsExpert          
    (
      SmSurfParamType eSurfParam,   // in :    
      ULONG           lStartIndex,  // in :    
      ULONG           lEndIndex,    // in :    
      double        * adKnots       // out:    
    ) const;

    SmStatus GetControlPointsExpert  
    (
      SmControlPointFormType eCtrlPointForm,      // in : Output array layout = specify data written to output array for each fetched control point.                    
                                                  //    : oneof SM_CP_NON_RATIONAL          = output euclidean coords only                                              
                                                  //    :     SM_CP_HOMOGENEOUS_RATIONAL  = output homogeneous coords and weights                                       
                                                  //    :     SM_CP_EUCLIDIAN_RATIONAL    = output euclidean coords and weights                                         
                                                  //    : controls the number of doubles per point in the output array.                                                 
                                                  //    : PtSize = ((SM_CP_NON_RATIONAL) ? 3 : 4)                                                                       
      ULONG                  lStartUIndex,        // in : First NlibArray U control point index to be copied                                                            
      ULONG                  lEndUIndex,          // in : Last  NlibArray U control point index to be copied                                                            
      ULONG                  lStartVIndex,        // in : First NlibArray V control point index to be copied                                                            
      ULONG                  lEndVIndex,          // in : Last  NlibArray V control point index to be copied                                                            
      ULONG                  lCtrlPointUStride,   // in : Output array layout = number of doubles to leave between fetched U Control Points.                            
                                                  //    : lCtrlPointUStride = M * PtSize, M range:[any positive integer]                                                
                                                  //    : M > 1 means M-1 uninitialized columns are built into the output array                                         
                                                  //    :                 between every fetched column.                                                                 
      ULONG                  lCtrlPointVStride,   // in : Output array layout = number of doubles to leave between fetched V Control Points.                            
                                                  //    : lCtrlPointVStride = N * ((lEndUIndex - lStartUIndex + 1) * lCtrlPointUStride), N range:[any positive integer] 
                                                  //    : N > 1 means N-1 uninitialized rows are built into the output array                                            
                                                  //    :                 between every fetched row.                                                                    
      double               * adControlPoints      // out: SM_CP_NON_RATIONAL         && 2d ? ordered:[X Y          X Y          ...]                                    
                                                  //    : SM_CP_HOMOGENEOUS_RATIONAL && 2d ? ordered:[X Y W        X Y W        ...]                                    
                                                  //    : SM_CP_EUCLIDIAN_RATIONAL   && 2d ? ordered:[X/W Y/W W    X/W Y/W W    ...]                                    
                                                  //    : SM_CP_NON_RATIONAL         && 3d ? ordered:[X Y Z        X Y Z        ...]                                    
                                                  //    : SM_CP_HOMOGENEOUS_RATIONAL && 3d ? ordered:[X Y Z W      X Y Z W      ...]                                    
                                                  //    : SM_CP_EUCLIDIAN_RATIONAL   && 3d ? ordered:[X/W Y/W Z/W  X/W Y/W Z/W  ...]                                    
                                                  //    :                                                                                                               
                                                  //    : assumed sized on input - watch for boundary errors!                                                           
                                                  //    : sized:[ lCtrlPointVStride * (lEndVIndex - lStartVIndex + 1)]                                                  
    ) const;

    double GetGrevilleAbscissa     
    (
      SmSurfParamType eSurfParam,                 // in :      
      ULONG           lCtrlPointIndex             // in :      
    ) const;

    SmStatus GetCanonical            
    (
      ULONG                & rlUDegree,               // out: Surface polynomial U degree                               
      ULONG                & rlVDegree,               // out: Surface polynomial V degree                               
      SmTArray<SmPoint3d>  & rControlPointsList,      // out: Euclidian form of the control points.                     
      SmBSplineSurfaceForm & reBSplineSurfaceForm,    //    : SM_SF_PLANE_SURF,         SM_SF_CYLINDRICAL_SURF,         
                                                      //    : SM_SF_CONICAL_SURF,       SM_SF_SURF_OF_REVOLUTION,       
                                                      //    : SM_SF_SPHERICAL_SURF,     SM_SF_RULED_SURF,               
                                                      //    : SM_SF_GENERALIZED_CONE,                                   
                                                      //    : SM_SF_QUADRIC_SURF,       SM_SF_SURF_OF_LINEAR_EXTRUSION, 
                                                      //    : SM_SF_UNSPECIFIED,                                        
                                                      //    : SM_SF_POLYNOMIAL,         SM_SF_HELICAL_SWEEP             
      SmTArray<ULONG>      & rUKnotMultiplicities,    // out: multiplicity value for each U knot                        
      SmTArray<ULONG>      & rVKnotMultiplicities,    // out: multiplicity value for each V knot                        
      SmTArray<double>     & rUKnots,                 // out: Unique U knot vector                                      
      SmTArray<double>     & rVKnots,                 // out: Unique V knot vector                                      
      SmKnotType           & reKnotType,              //    : SM_KT_UNIFORM_KNOTS,                                      
                                                      //    : SM_KT_UNSPECIFIED,                                        
                                                      //    : SM_KT_QUASI_UNIFORM_KNOTS,                                
                                                      //    : SM_KT_PIECEWISE_BEZIER_KNOTS                              
      SmTArray<double>     & rWeights                 // out: associated weight for every ControlPoint,                 
                                                      //    : rWeigths.GetSize() == 0 for non-rational curves           
    ) const;
                               
    // Simple Queries                            
    SmBoolean         IsRational             () const;
    SmBoolean         IsNurbBorrowed         () const { return m_bNurbIsBorrowed; }
    SmBoolean         IsOutOfBoundsEnabled   () const { return m_bOutOfBoundsEnabled; }
    virtual SmBoolean IsBilinear             () const;
    virtual SmBoolean IsParallelToVector     (const SmVector3d & crVector) const;
    virtual SmBoolean IsDegeneratePoint      (double             d3dTolerance=SM_ZONE_TOL_3D/10.0,               // rtn: TRUE = surface is a pointto tol
                                              const SmExtent2d * pOptUVDomain=NULL) const ;
    virtual SmBoolean IsDegenerateCurve      (double             dAngleTolDeg=SM_EFF_ZERO_DEG,  // rtn: TRUE = surface is a curve to tol but not a point
                                              const SmExtent2d * pOptUVDomain=NULL,
                                              double             d3dTolerance=SM_ZONE_TOL_3D/10.0) const ;
    SmBoolean         HasSameParameterization(const SmBSplineSurface * cpOtherSurface) const ;
    SmBoolean         HasSameParameterization(SmSurfParamType        eAnalyticDirection, 
                                              SmSurfParamType        eNurbDirection, 
                                              const SmBSplineCurve * cpOtherCurve) const ;

    virtual SmBoolean HasRepeatedEndControlPoints (double dTol=SM_ZONE_TOL_3D) const ;
    virtual SmBoolean HasInternalPole             (double dTol=SM_ZONE_TOL_3D) const ;
    virtual SmBoolean HasDegenerateControlPointRow(double dTol=SM_ZONE_TOL_3D) const ;
    virtual SmBoolean HasKnotMultiplicityGreaterThanDegree() const ;

    SmStatus InsertOneKnot
    (
      double          dKnot,                // in :      
      ULONG           lNumKnotInsertions,   // in :      
      SmSurfParamType eSurfParam            // in :      
    );

    SmStatus InsertKnots
    ( 
      SmSurfParamType    eSurfParam,        // in : one of SM_SP_U or SM_SP_V, knot vector to receive more knots
      SmTArray<double> & rNewKnots          // in : array of knot values to add                                     
    );

    SmStatus InsertKnots
    ( 
      SmSurfParamType eSurfParam,           // in :      
      int             iNumToAdd,            // in :      
      SmExtent1d    * pOptInterval = NULL   // in :      
    );

    SmStatus InsertKnotsByDistance
    (
      SmSurfParamType eSurfParam,           // in :      
      double dDistance                      // in :      
    );

    static SmStatus InterpolatePoints
    (
      const SmContext                 & crContext,          // in :                                                                         
      const SmTArray<SmPoint3d>       & crPoints,           // in : Contains lNumRows x lNumCols points                                     
      ULONG                             lNumRows,           // in :                                                                         
      ULONG                             lNumCols,           // in :                                                                         
      ULONG                             lUDegree,           // in :                                                                         
      ULONG                             lVDegree,           // in :                                                                         
      const SmTArray<SmBSplineCurve*> * cpOptBdryCurves,    // in : The first two will be U-boundaries and the last two will be V-boundaries
      const SmTArray<double>          * cpOptUParams,       // in :                                                                         
      const SmTArray<double>          * cpOptVParams,       // in :                                                                         
      SmInterpolationType               eParameterization,  // in : SM_IT_UNIFORM:                                                          
                                                            //    : SM_IT_CENTRIPETAL:                                                      
                                                            //    : SM_IT_CHORDLENGTH:                                                      
      SmBSplineSurface               *& rpNewSurf           // out:                                                                         
    );

    SmStatus MakeNonRational();
    SmStatus MakeSingular
    (
      ULONG       lSingularSide,         // in : target side: one of SM_SS_UMIN SM_SS_UMAX                 
                                         //    :                   SM_SS_VMIN SM_SS_VMAX                   
      SmPoint3d * pOptSingularPt=NULL    // in : Euclidean Point to become the singularity location        
                                         //    : NULL = Set SingularPt = Centroid of Boundary ControlPts   
    );                                                                                                     

    // Return TRUE when surface has both STEP and NURB representations that are equivalent
    virtual SmBoolean AreSTEPAndNURBCurrent() const    { return TRUE ; }
    virtual SmStatus  RebuildSTEPFromNURBParameters()  { return SM_SUCCESS ; } // also update GenCurve and PolarCurve data as needed
    virtual SmStatus  RebuildNURBFromSTEPParameters()  { return(MakeNurb()) ; }
    virtual SmStatus  MakeNurb();

    SmStatus OffsetCurveAlongSurface
    (
      const SmContext & crContext,      // in :                                                 
      SmBSplineCurve  * pCrv,           // in : curve must lie on or near surface               
      double            dOffset,        // in : + or - for which side of crv                    
      double            dTol,           // in : tolerance used for intersection ( use 0,001 )   
      SmBSplineCurve *& pOffsetCrv      // out: offset curve                                    
    );

    virtual SmBoolean PassesValidityCheck
    (
      SmValidityCheckType   eChecks,      // in :        
      SmValidityCheckType & reCheckFailed // in :        
    ) const;

    static SmStatus PrepCurvesForSurfacing 
    (
      const SmContext                 & crContext,                        // in :                               
      const SmTArray<SmBSplineCurve*> & crUCurves,                        // in :                               
      const SmTArray<SmBSplineCurve*> & crVCurves,                        // in :                               
      NL_CURVE                           * curU[],                           // out:                               
      NL_CURVE                           * curV[],                           // out:                               
      NL_STACKS                          & SG,                               // in :                               
      double                            dTol = -1.0,                      // in : default: -1 (not specified)   
      SmBoolean                         bMakeNonRational = FALSE,         // in : default False                 
      ULONG                             lNumPtsBetweenU = 0,              // in : default 0                     
      ULONG                             lNumPtsBetweenV = 0,              // in : default 0                     
      SmBoolean                         bCommonParamRanges = TRUE,        // in : default True                  
      SmBoolean                         bCommonParameterization = TRUE,   // in : default True                  
      double                          * u[] = NULL,                       // in : default NULL                  
      double                          * v[] = NULL,                       // in : default NULL                  
      double                         ** uu[] = NULL,                      // in : default NULL                  
      double                         ** vv[] = NULL                       // in : default NULL
    );

    SmStatus RefineSurface
    (
      const SmTArray<double> & crNewKnots,  // in :   
      SmSurfParamType          eSurfParam   // in :   
    );

    SmStatus RemoveKnots
    (
      double                   dThisApproxTol3d,        // in : max allowed Current/New Curve deviation          
      SmBoolean                bRemoveUKnots,           // in : TRUE = Constrain end derivatives, FALSE = don't  
      SmBoolean                bRemoveVKnots,           // in : TRUE = Constrain end derivatives, FALSE = don't  
      const SmTArray<double> * cpOptKeptUKnots = NULL,  // out: u-knots that will NOT be removed                 
      const SmTArray<double> * cpOptKeptVKnots = NULL   // out: v-knots that will NOT be removed                 
    );

    SmStatus RemoveOneKnot
    (
      double          dKnot,               // in : Knot value to be removed                  
      ULONG           lNumKnotsRemoval,    // in : number of times to remove the knot        
      SmSurfParamType eSurfParam,          // in : SM_SP_U = Remove in u-direction           
      double          dThisApproxTol3d,    //    : SM_SP_V = Remove in v-direction           
      ULONG         & rlNumKnotsRemoved    // in : max allowed Current/New Curve deviation   
                                           // out: Number of knots removed                   
    );

    // change surface parameterization with no change in surface shape
    virtual SmStatus Reparameterize(const SmExtent2d & crNewDomain);

    SmStatus ReparametrizeWithArcLength();

    SmStatus Reverse(SmSurfParamType eSurfParam);

    SmStatus SetCanonical
    (
      ULONG                       lUDegree,                 // in : u Degree of this SmBSplineCurve   
      ULONG                       lVDegree,                 // in : v Degree of this SmBSplineCurve   
      const SmTArray<SmPoint3d> & crControlPointsList,      // in : control points                    
      SmBSplineSurfaceForm        eBSplineSurfaceForm,      // in :                                   
      const SmTArray<ULONG>     & crUKnotMultiplicities,    // in :                                   
      const SmTArray<ULONG>     & crVKnotMultiplicities,    // in :                                   
      const SmTArray<double>    & crUKnots,                 // in :                                   
      const SmTArray<double>    & crVKnots,                 // in :                                   
      SmKnotType                  eKnotType,                // in :                                   
      const SmTArray<double>   * cpOptWeights               // in :                                   
    );

    SmStatus SetControlPoint
    (
      SmControlPointFormType eCtrlPointForm,    // in : Either SM_CP_EUCLIDIAN_RATIONAL with a weight    
                                                //    : or SM_CP_NON_RATIONAL which ignores the weight   
      ULONG                  lUIndex,           // in :                                                  
      ULONG                  lVIndex,           // in :                                                  
      const SmPoint3d      & crControlPoint,    // in :                                                  
      double                 dWeight            // in :
    );

    SmStatus SetExpert
    (
      ULONG                  lUDegree,              // in :     
      ULONG                  lVDegree,              // in :     
      SmBSplineSurfaceForm   eBSplineSurfaceForm,   // in :     
      SmEndKnotFormType      eEndKnotForm,          // in :     
      ULONG                  lUNumKnots,            // in :     
      const double         * cadUKnots,             // in :     
      ULONG                  lVNumKnots,            // in :     
      const double         * cadVKnots,             // in :     
      SmControlPointFormType eCtrlPointForm,        // in :     
      ULONG                  lCtrlPointUStride,     // in :     
      ULONG                  lCtrlPointVStride,     // in :     
      const double         * cadCtrlPoints          // in :     
    );

    virtual SmStatus SetFromGwNurb
    (
      ULONG,                        // in : Not used       
      gw_SURFACE * pGwNurbSurface   // in :                
    );

    void SetBSplineSurfaceForm(SmBSplineSurfaceForm eBSplineSurfaceForm) { m_eBSplineSurfaceForm = eBSplineSurfaceForm; }

    virtual void SetOutOfBoundsEnabled(SmBoolean bOutOfBoundsEnabled)    { m_bOutOfBoundsEnabled = bOutOfBoundsEnabled; }

    // Move Control points in two surfaces so that the surfaces meet across a G1 common boundary.
    static SmStatus SmoothJoin
    (
      SmBSplineSurface * pSurf1,              // in : Input surface1                             
      SmSurfParamType    eWhichEdge1,         // in : SM_SP_UMIN, _VMIN, _UMAX, or _VMAX.        
      SmBSplineSurface * pSurf2,              // in : Input surface2                             
      SmSurfParamType    eWhichEdge2,         // in : SM_SP_UMIN, _VMIN, _UMAX, or _VMAX.        
      double             dPreserveEndFactor   // in : see Usage Notes.  Default 0.5.             
    );

    // Build left and right new surfaces from the original unmodified surface and a specified split parameter
    SmStatus SplitAt
    (
      const SmContext & crContext,        // in : context for new object construction           
      double            dParam,           // in : split parameter                               
      SmSurfParamType   eSurfParam,       // in : one of SM_SP_U = split u domain at dParam
                                          //    :       SM_SP_V = split v domain at dParam      
      SmSurface      *& rpLeftSurface,    // out: Split surface result, Ivl=[MinParam, TgtParam]
      SmSurface      *& rpRightSurface    // out: Split surface result, Ivl=[TgtParam, MaxParam]
    );

    virtual SmStatus STEPInversion
    (
      const SmExtent2d & crAnalUVDomain,      // in : domain limit for successful inversions               
      const SmPoint3d  & crPointOnSurf,       // in : Target Point - must be on Surface within Tolerance   
      double             dDistanceTolerance,  // in : max allowed dist between point and surface           
      SmPoint2d        & rdAnalUVParameter,   // out: Nurb Surface Point[U,V] for input point              
      SmLocationType   & reLocation,          // out: SM_LT_POLE,                                          
                                              //    : SM_LT_U_SEAM,                                        
                                              //    : SM_LT_V_SEAM,                                        
                                              //    : SM_LT_UV_SEAM,                                       
                                              //    : SM_LT_INTERIOR                                       
      SmPoint2d        * pUVGuess = NULL      // in : guess parameter.                                     
    ) const;

    SmStatus SwapUV();
    virtual void      ToggleSwapUVBit()  { }  // only called by SwapUV(). Every Surf with a m_bSwapUV member (the analytics) must implement.
    virtual SmBoolean GetSwapUVBit()     { return FALSE ; }

    SmStatus TestDegenerate
    (
      double  Tolerance, 
      ULONG & rNu, 
      ULONG & rNv
    ) const;

    SmStatus TestCornerNormals(double Tol) const;

    SmStatus TestForFilletSurface
    (
      const SmContext & crContext,                // in : context for new object construction                                                
      double            d3DTolerance,             // in : max allowed distance between this curve and a filletCrossSection Shape             
      SmBoolean       & rbIsFilletSurface,        // out: TRUE = surface's naturalDomain isoParameter curves have Fillet CrossSection shapes 
                                                  //    : FALSE= curves natural domain boundaries do not have fillet crossSection shapes     
      ULONG           & rlFilletCrossSection,     // out: fillet crossSection type    0 - circular,                                          
                                                  //    :                           1 - Approx Circular,                                     
                                                  //    :                           2 - linear                                               
                                                  //    :                           3 - G1,                                                  
                                                  //    :                           4 - G2,                                                  
                                                  //    :                           5 - G3                                                   
      ULONG           & rlFilletSolverType,       // out: 0 - Constant Radius                                                                
                                                  //    : 1 - Constant Distance,                                                             
                                                  //    : 2 - Variable Radius                                                                
      SmSurfParamType & reFilletRailDirection,    // out: Direction in which profiles are isoparms                                           
                                                  //    : SM_SP_U - profiles are constant U curves                                           
      SmBoolean       & rbNormalIsOutward,        // out: TRUE = If surface normal points toward outward part of surface                     
      double          * pdRadius = NULL,          // out: Radius, if constant radius                                                         
      double          * pdDist = NULL,            // out: Cross-section distance, if constant distance                                       
      SmExtent1d      * paMinMaxRadii = NULL      // out: [optional; default NULL]: Min and max radii, if variable radius                    
    ) const;

    SmStatus TestForFilletSurfaceUOrV
    (
      const SmContext & crContext,                // in : context for new object construction                                                
      SmSurfParamType   eDirToTest,               // in : U- or V-direction, which cross sections to test.                                   
      double            d3DTolerance,             // in : max allowed distance between this curve and a filletCrossSection Shape             
      SmBoolean       & rbIsFilletSurface,        // out: TRUE = surface's naturalDomain isoParameter curves have Fillet CrossSection shapes 
                                                  //    : FALSE= curves natural domain boundaries do not have fillet crossSection shapes     
      ULONG           & rlFilletCrossSection,     // out: fillet crossSection type    0 - circular,                                          
                                                  //    :                             1 - Approx Circular,                                   
                                                  //    :                             2 - linear                                             
                                                  //    :                             3 - G1,                                                
                                                  //    :                             4 - G2,                                                
                                                  //    :                             5 - G3                                                 
      ULONG           & rlFilletSolverType,       // out:  0 - Constant Radius                                                               
                                                  //    :  1 - Constant Distance,                                                            
                                                  //    :  2 - Variable Radius                                                               
      SmBoolean       & rbNormalIsOutward,        // out:  TRUE = If surface normal points toward outward part of surface                    
      double          * pdRadius = NULL,          // out: [optional; default NULL]: Radius, if constant radius                               
      double          * pdDist = NULL,            // out: [optional; default NULL]: Cross-section distance, if constant distance             
      SmExtent1d      * paMinMaxRadii = NULL      // out: [optional; default NULL]: Min and max radii, if variable radius                    
    ) const;

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

    virtual SmStatus TrimWithDomain(SmExtent2d & rTrimDomain);


    // for backward compatibility - define these read/write methods which have moved to SmSurface

    // write a surface to file
    SmStatus WriteToFile
    (
      const TCHAR * cOutputFileName,              // in :                                       
      SmBoolean     bSkipHeaderWrite=FALSE,       // in :                                       
      SmBoolean     bNewFile=FALSE,               // in :                                       
      SmBoolean     bWriteAttributes=FALSE        // in : TRUE=Write attributes, FALSE=don't    
    ) const 
    { 
      return(SmSurface::WriteToFile(cOutputFileName, bSkipHeaderWrite, bNewFile, bWriteAttributes)) ; 
    }   
    
    // write array of surfaces to file                                          
    static SmStatus WriteArrayToFile
    (
      const TCHAR               * cOutputFileName,                 
      const SmTArray<SmBSplineSurface *> & rSrfArr,           
      SmBoolean                   bWriteAttributes=FALSE
    ) 
    { 
      ULONG ii ;
      SmTArray<SmSurface *> sWriteSurfaces ;
      for(ii=0;ii<rSrfArr.GetSize();ii++) 
      { 
        sWriteSurfaces.Add((SmSurface*)rSrfArr[ii]) ; 
      }
      return(SmSurface::WriteArrayToFile(cOutputFileName, sWriteSurfaces, bWriteAttributes)) ; 
    }

    // read a surface from file
    static SmStatus ReadFromFile
    (
      const SmContext & crContext,
      const TCHAR     * cInputFileName,
      SmBSplineSurface  *& rpNewSurface,
      const ULONG       lFileOffsetInBytes = 0
    ) 
    { 
      SmSurface *pReadSurface = NULL ;
      SmStatus eStat = SmSurface::ReadFromFile(crContext, cInputFileName, pReadSurface, lFileOffsetInBytes) ; 
      rpNewSurface = SM_CAST_PTR(SmBSplineSurface, pReadSurface) ;
      return eStat ;
    }

    // read array of surfaces from file
    static SmStatus ReadArrayFromFile
    (
      const SmContext              & crContext,
      const TCHAR                  * cInputFileName,        
      SmTArray<SmBSplineSurface *> & rNewSurfaces,
      const ULONG                    lFileOffsetInBytes = 0,
      SmTArray <SmSurface *>       * pTestSurfaces = NULL
    ) 
    { 
      SmTArray<SmSurface *> sReadSurfaces ;
      SmStatus eStat = SmSurface::ReadArrayFromFile(crContext, cInputFileName, sReadSurfaces, lFileOffsetInBytes, pTestSurfaces) ; 
      ULONG ii ;
      for(ii=0;ii<sReadSurfaces.GetSize();ii++) 
      { 
        rNewSurfaces.Add((SmBSplineSurface*)sReadSurfaces[ii]) ; 
      }
      return eStat ;
    }

    // I/O assist
    virtual SmStatus WriteToDB
    (
      SmDatabaseIO & rDB,                   // in : target output stream                                     
      ULONG          lDBVersionNumber       // in : database version to get proper sequence of writes                                                                       
    ) const;

    static  SmStatus ReadFromDB
    (
      SM_TYPE           lType,              // NotUsed: in : Object type to be read                                                         
      SmDatabaseIO    & rDB,                // in : target output stream                                                           
      const SmContext & crContext,          // in : context for new object construction                                            
      SmSurface      *& rpNewSurface,       // out: NULL on input = new object allocated in this routine built from stream data    
                                            //    : NotNULL on input = pointer to an empty object to be filled by this routine     
      ULONG             lDBVersionNumber    // in : database version to get proper sequence of writes                              
    );

    // output surface Polygons through SmPolygonOutputCallback.OutputPolygon call
    SmStatus OutputPolygons
    (
      double                    dSurfaceChordHeightTolerance,   // in : max distance between polygon and surface                                         
      double                    dCurveChordHeightTolerance,     // in : mas distance between polygon edge and surface                                    
      SmBoolean                 bReverseNormals,                // in : TRUE = Reverse polygon normals prior to display                                  
                                                                //    : FALSE= don't                                                                     
      SmPolygonOutputCallback & rPolygonOutput,                 // in : Chooses how and where to output the polygons                                     
      SmGfxArraySet           * pOptGfxSet = NULL               // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.  
    ) const;

    // in : Chooses how and where to output the polygons
    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.

    // get memory used for curve but not its attributes
    virtual ULONG GetMemoryUsed
    (
      ULONG    & rlMemoryAllocated,      // out: bigger size of all allocated memory in bytes
      SmMarkType eMarkType=SM_MT_NOMARK  // in : uses without increment eMarkType value
    ) const ;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmBSplineSurface,SmSurface,SmBSplineSurface_TYPE);

    virtual SmBoolean AssertValid
    (
      SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                                             
      SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         
                                                //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     
                                                //      default:[SM_LEVEL_0]                                                                               
      SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   
      SmTArray<ULONG>  * pTestRequests=NULL     // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   
    ) const ;

    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

    virtual void Dump(const TCHAR * message) const;
    virtual void Dump(ULONG) const;
    virtual void Dump(SmBoolean bAbbrev) const;

protected:
    void SwapNurbPointers(SmBSplineSurface *pTgt) 
    {
      gw_SURFACE *pTmp = m_pNurb ; 
      m_pNurb          = pTgt->m_pNurb ;
      pTgt->m_pNurb    = pTmp ; 
    }

} ; // end class SmBSplineSurface



/***********************************************************//**
* PURPOSE - convenience macro for managing m_pNurb construction
*
****************************************************************/
#define  SM_ENSURE_SURFACE_MPNURB(a)  \
if( (a)->IsKindOf(SmBSplineSurface_TYPE) )  \
  { ((SmBSplineSurface *)(a))->GetOrCreateGwNurbPointer() ; }


// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmBSplineSurface*) ;

#endif // !__SMBSPLINESURFACE_H__

