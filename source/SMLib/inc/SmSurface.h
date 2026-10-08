// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurface.h
* PURPOSE: Header file for Surface class. 
**********************************************************************/

#ifndef __SMSURFACE_H__
#define __SMSURFACE_H__

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMEXTENT2D_H__
#include <SmExtent2d.h>
#endif

#ifndef __SMAOBJECT_H__
#include <SmAObject.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMLOCALSOLVE1D_H__
#include <SmLocalSolve1d.h> // for SmTerminationReasonType
#endif

#include <SmGraphicsExtern.h>
#include <SmConfig.h>
#include <SmTol.h>

//#ifndef __MUTEX__
//#define __MUTEX__
//#include <mutex>
//#endif

enum SmPinCushionType ;
class SmDerivSurfDefinition ;
class SmGfxArraySet ;
class SmPlane ;

// Space Deformations (e.g. SpaceBend, SpaceUnbend, etc.)
// under development - make sure the following is not defined - not yet complete or ready for use
#define SM_LOOSE_SPACEDEF_APPROXS

/*******************************************************************//**
PURPOSE: enum list of failure and recovery paths for CreateUVTrimCurve

NOTES:
***********************************************************************/
enum SmCreateUVTrimCurvePathType 
    {
      SM_CTC_UNKNOWN,
      SM_CTC_OKAY,                              // CreateUVTrimCurve: Curve dropped and used as expected - no recoveries
      SM_CTC_FROM_CRVONSURF,                    // CreateUVTrimCurve: lowWork - Curve = CrvOnSurf
      SM_CTC_REVERSEDROP_RECOVERY,              // CreateUVTrimCurve: DropCurve Fail recovery - DropReverseCurve
      SM_CTC_TWOSINGULARENDPTS_RECOVERY,        // CreateUVTrimCurve: DropCurve Fail recovery - Both endPts are singularities
      SM_CTC_SHORTPIECE_SEAMCROSS_RECOVERY,     // CreateUVTrimCurve: CrossSeam recovery - ShortSegment deleted - LongSegment used 
      SM_CTC_PICKONESEAM_RECOVERY,              // CreateUVTrimCurve: DropToSeam - Pick one DropCurve - Discard the other 
      SM_CTC_10PERCENT_SEAMCROSS_RECOVERY,      // CreateUVTrimCurve: CrossSeam recovery - 10% Segment deleted - LongSegment used 
      SM_CTC_1PERCENT_UVTRIMCURVETRIM_RECOVERY1,// CreateUVTrimCurve: mismatched UVTrimCurve/Curve param ranges - resolved by Trimming UVTrimCurve
      SM_CTC_1PERCENT_UVTRIMCURVETRIM_RECOVERY2,// CreateUVTrimCurve: mismatched UVTrimCurve/Curve param ranges - resolved by Trimming UVTrimCurve
      SM_CTC_1PERCENT_EDGETRIM_RECOVERY1,       // CreateUVTrimCurve: mismatched UVTrimCurve/Curve param ranges - resolved by Trimming Edge
      SM_CTC_1PERCENT_EDGETRIM_RECOVERY2,       // CreateUVTrimCurve: mismatched UVTrimCurve/Curve param ranges - resolved by Trimming Edge
      SM_CTC_1PERCENT_CURVETRIM_RECOVERY,       // CreateUVTrimCurve: mismatched UVTrimCurve/Curve param ranges - resolved by Trimming Curve
      SM_CTC_101POINTDROP_DEGENCURVE_RECOVERY,  // CreateUVTrimCurve: UVTRIMCURVE FAILURE - resolved by one successful drop of 101 drop points
      SM_CTC_101POINTDROP_FITCURVE_RECOVERY,    // CreateUVTrimCurve: UVTRIMCURVE FAILURE - resolved by several successful drops of 101 drop points    
      SM_CTC_ALLRECOVERIES_FAIL,                // CreateUVTrimCurve: All UVTRIMCURVE recovery attempts failed

      SM_CTC_DROPCURVE_OKAY,                    // DropCurve: algorithm worked with no problems
      SM_CTC_DEGENSURFACE_FAIL,                 // DropCurve: no work - input Surface is Degenerate 
      SM_CTC_DROPDOMAIN_UNCONTAINED_FAIL,       // DropCurve: no work - input DropDomain outside of Surface UVDomain
      SM_CTC_FIRSTDROP_UNCONTAINED_FAIL,        // DropCurve: no work - First DropPoint is not over surface
      SM_CTC_BAD_FIND_DEGENPARAM_WARNING,       // DropCurve: FindDegenParamDirection for drop to singular point failed
      SM_CTC_KEPT_BEYOND_TOL_DROP,              // DropCurve: Kept a DropCurve beyond Drop Tolerances because bKeepAllDropCurves == TRUE
      SM_CTC_DROPSTARTPOINT_FAIL,               // DropCurve: At lease one DropCurve StartPoints failed to yield a DropCurve
      SM_CTC_SPLIT_DROPSTEP_FAIL,               // DropCurve: Shortening the DropStep distance fails to yield next DropPoint      

      SM_CTC_DROPANDTRIMCURVE_OKAY              // DropAndTrimCurve: algorithm worked with no problems

    } ;

/*******************************************************************//**
PURPOSE: Helper data container class for DropCurve failures

NOTES: 
 1. This object is not used when DropCurve succeeds .
 2. This object is only used when DropCurve fails.  It contains
    copies of any data DropCurve was able to generate prior to the 
    DropCurve failure.
 3. It was created to help SmEdgeuse::CreateUVTrimCurve() manage DropCurve
    failures.  (DropCurve failures commonly indicate database problems 
    where an Edge-connected-to-a-face crosses a missing seam or is not 
    close to the Face's Surface. Such problems should be identified
    and fixed either by running an appropriate heal function while the
    part is being read, or by finding and fixing the bit of SMLib
    code that has the bug that generates the bad database.)
***********************************************************************/
class SM_EXPORT SmDropCurveFail
{
  public:
   SmStatus           m_sFailStatus = SM_UNDEF_ULONG ;              // mem 1 :contains copy of return Status value
   ULONG              m_lFromCall = SM_UNDEF_ULONG ;                // mem 2 :0=Uninit, 1=DropCurve(), 1=DropAndTrimCurve()
   SmCreateUVTrimCurvePathType m_eCreatePathType = SM_CTC_UNKNOWN ; // mem 3


                                                    
   // DropCurve(Input args)                                  
   const SmContext           * m_cpContext = NULL ;                    // mem 4 : pointer to external state - could go stale     
   SmExtent2d                  m_sUVDomain ;                           // mem 5 : 
   const SmSurface           * m_cp3dSurface = NULL ;                  // mem 6 : pointer to external state - could go stale
   const SmCurve             * m_cp3dCurve   = NULL ;                  // mem 7 : pointer to external state - could go stale     
   SmExtent1d                  m_sInterval ;                           // mem 8 : 
   SmApproxTol3d               m_sApproxTol3d ;                        // mem 9 : 
   double                      m_dMaxDropToSurfArg = SM_UNDEF_DOUBLE ; // mem 10: 
   double                      m_dMaxApproxDev     = SM_UNDEF_DOUBLE ; // mem 11: 
   SmTArray<SmBSplineCurve*> * m_pUVCurvesArg = NULL ;                 // mem 12: pointer to external state - could go stale      
   SmBoolean                   m_bKeepAllDropCurves = FALSE ;          // mem 13: 
                                                           
   // DropCurve Degen Geom classification                   
   SmBoolean          m_bDegenCurve = FALSE ;              // mem 14:TRUE = cr3DCurve is degenerate to a point in 3Space
   SmBoolean          m_bDegenSurfU = FALSE ;              // mem 15:TRUE = this Surf sUVDomain.XLength is degenerate
   SmBoolean          m_bDegenSurfV = FALSE ;              // mem 16:TRUE = this Surf sUVDomain.YLength is degenerate
                                                           
   // DropCurve input Domain check                         
   SmBoolean          m_bDropDomainIsContained = FALSE ;   // mem 17:TRUE = Given UVDropDomain is bigger than Surf NaturalUVDomain
                                                    
   // DropCurve internal result accumulators                 
   double             m_dMaxDropToSurf = SM_UNDEF_DOUBLE ; // mem 18:Max DropToSurface distance seen prior to failure
   SmSolutionArray    m_sSolutions ;                       // mem 19:Array of StartCurve drop points - one curve to be traced for each solution
   SmTArray<SmBSplineCurve*> m_sUVCurves ;                 // mem 20:Copies of any curves or curve pieces that were made before the failure
                                                            
   // DropCurve surface classification                       
   SmBoolean           m_bClosedU    = FALSE ;             // mem 21:TRUE = Surface is closed in U
   SmBoolean           m_bClosedV    = FALSE ;             // mem 22:TRUE = Surface is closed in V
   SmSurfParamType     m_eSrfClosure = SM_SP_UNKNOWN ;     // mem 23:oneof, SM_SP_BOTH, SM_SP_U, SM_SP_V, SM_SP_NEITHER
   SmTArray<SmPoint3d> m_sSrfPolePoints ;                  // mem 24:PolePoint array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
   SmTArray<SmPoint3d> m_sSrfPoleNormals ;                 // mem 25:associated Pole Normals
   ULONG               m_lSingularities       = SM_UNDEF_ULONG ; // mem 26:Tol:[SM_EFF_ZERO], SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
   ULONG               m_lApproxSingularities = SM_UNDEF_ULONG ; // mem 27:Tol:[ApproxTol3d], SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX

   // last set of Trace Curve locals                        
   SmTArray<SmPoint3d> m_sCtrlPoly ;                // mem 28:ordered UV ctrlPoint array for all previously dropped span ivls (stored in SmPoint3d array - z coords ignored)
   SmTArray<double>    m_sKnots = {0} ;             // mem 29:ordered param values for all previously dropped span ivls
   SmBrep            * m_pBrep = NULL ;             // mem 30:gathered when possible - used for graphics

  public:
    SmDropCurveFail() { UnInit() ; m_pBrep = NULL ; }
    SmDropCurveFail(const SmDropCurveFail & crOther) ; 
    SmDropCurveFail & operator=(const SmDropCurveFail &crOther) ; 
    SmBoolean         operator==(const SmDropCurveFail &crOther) ; 
   ~SmDropCurveFail() { UnInit() ; }

    // copy
    SmStatus Copy(SmDropCurveFail *& rpCopy) const { rpCopy = new SmDropCurveFail(*this) ; NER(rpCopy) ;
                                                     return SM_SUCCESS ;
                                                   }
    // ReSet
    void UnInit() ;

    // Set 
    void Set(SmStatus  sFailStatus,                            // arg 1
             ULONG     lFromCall,                              // arg 2, 0=Uninit, 1=DropCurve(), 2=DropAndTrimCurve()
                                                               //        3=CreateUVTrimCurve() for SmCrvOnSurf
             SmCreateUVTrimCurvePathType eCreatePathType,      // arg 3                                                    
             const SmContext           * cpContext,            // arg 4
             SmExtent2d                  sUVDomain,            // arg 5
             const SmSurface           * cp3dSurface,          // arg 6
             const SmCurve             * cp3dCurve,            // arg 7
             SmExtent1d                  sInterval,            // arg 8
             SmApproxTol3d               sApproxTol3d,         // arg 9
             double                      dMaxDropToSurfArg,    // arg 10
             double                      dMaxApproxDev,        // arg 11
             SmTArray<SmBSplineCurve*> * pUVCurvesArg,         // arg 12
             SmBoolean                   bKeepAllDropCurves,   // arg 13 
                                                               
             SmBoolean bDegenCurve,                            // arg 14
             SmBoolean bDegenSurfU,                            // arg 15
             SmBoolean bDegenSurfV,                            // arg 16
             SmBoolean bDropDomainIsContained,                 // arg 17
             double    dMaxDropToSurf,                         // arg 18
             SmBoolean            bClosedU=UNSURE,             // arg 19
             SmBoolean            bClosedV=UNSURE,             // arg 20
             SmSurfParamType      eSrfClosure=SM_SP_UNKNOWN,   // arg 21     
             SmTArray<SmPoint3d> *pSrfPolePoints=NULL,         // arg 22
             SmTArray<SmPoint3d> *pSrfPoleNormals=NULL,        // arg 23
             ULONG                lSingularities=SM_SS_NONE,   // arg 24
             ULONG                lApproxSingularities=SM_SS_NONE, // arg 25
             SmSolutionArray           *pSolutions=NULL,           // arg 26
             SmTArray<SmPoint3d>       *pCtrlPoly=NULL,            // arg 27
             SmTArray<double>          *pKnots=NULL,               // arg 28
             SmTArray<SmBSplineCurve*> *pUVCurves=NULL,            // arg 29
             SmBrep                    *pBrep=NULL) ;              // NotUsed: arg 30
                                                                       
  SmBoolean IsUnInit() { return(m_sFailStatus == SM_ERR_UNKNOWN) ; }

  void Dump(SmEdgeuse *pOptEdgeuse=NULL) ;    // NotUsed: in : pOptEdgeuse
  SmDisplayList * Draw(SmGfxArraySet * pOptGfxSet= NULL) const ;

} ; // end SmDropCurveFail

// various DropCurve macros to support failures at different scopes with various combinations of data available - (a mess - apologies)
#define SM_SET1_DROP_CURVE_FAIL(status,ePathType) pDropCurveFail->Set(status, 1, ePathType,\
                                                            &crContext, crUVDomain, this, &cr3dCurve, crInterval, sApproxTol3d, \
                                                            rdMaxDropToSurf, rdMaxApproxDev, &rUVCurves, bKeepAllDropCurves, \
                                                            bDegenCurve, bDegenSurfU, bDegenSurfV, bDropDomainIsContained, dMaxDropToSurf) ;  
#define SM_SET2_DROP_CURVE_FAIL(status, ePathType) pDropCurveFail->Set(status, 1, ePathType,\
                                                            &crContext, crUVDomain, this, &cr3dCurve, crInterval, sApproxTol3d, \
                                                            rdMaxDropToSurf, rdMaxApproxDev, &rUVCurves, bKeepAllDropCurves, \
                                                            bDegenCurve, bDegenSurfU, bDegenSurfV, bDropDomainIsContained, dMaxDropToSurf, \
                                                            bClosedU, bClosedV, eSrfClosure, &sSrfPolePoints, &sSrfPoleNormals, \
                                                            lSingularities, lApproxSingularities) ;

#define SM_SET3_DROP_CURVE_FAIL(status, ePathType) pDropCurveFail->Set(status, 1, ePathType,\
                                                            &crContext, crUVDomain, this, &cr3dCurve, crInterval, sApproxTol3d, \
                                                            rdMaxDropToSurf, rdMaxApproxDev, &rUVCurves, bKeepAllDropCurves, \
                                                            bDegenCurve, bDegenSurfU, bDegenSurfV, bDropDomainIsContained, dMaxDropToSurf, \
                                                            bClosedU, bClosedV, eSrfClosure, &sSrfPolePoints, &sSrfPoleNormals, \
                                                            lSingularities, lApproxSingularities, \
                                                            &sSolutions, \
                                                            &sCtrlPoly, &sKnots, &rUVCurves) ;

#define SM_SET4_DROP_CURVE_FAIL(status, ePathType) pDropCurveFail->Set(status, 1, ePathType,\
                                                            &crContext, crUVDomain, this, &cr3dCurve, crInterval, sApproxTol3d, \
                                                            rdMaxDropToSurf, rdMaxApproxDev, &rUVCurves, bKeepAllDropCurves, \
                                                            bDegenCurve, bDegenSurfU, bDegenSurfV, bDropDomainIsContained, dMaxDropToSurf, \
                                                            bClosedU, bClosedV, eSrfClosure, &sSrfPolePoints, &sSrfPoleNormals, \
                                                            lSingularities, lApproxSingularities, \
                                                            &sSolutions, \
                                                            NULL, NULL, &rUVCurves) ;

// DropAndTrim generates only the rUVCurves and does not generate intermediate drop results on failure
#define SM_SET_DROP_AND_TRIM_CURVE_FAIL(status, ePathType) pDropCurveFail->Set(status, 2, ePathType,\
                                                             &crContext, sUVDomain, this, &cr3dCurve, crInterval, sApproxTol3d, \
                                                             rdMaxDropToSurf, rdMaxApproxDev, &rUVCurves, bKeepAllDropCurves, \
                                                             UNSURE, UNSURE, UNSURE, UNSURE, rdMaxDropToSurf, \
                                                             UNSURE, UNSURE, SM_SP_UNKNOWN, NULL, NULL, \
                                                             0, 0, \
                                                             NULL, \
                                                             NULL, NULL, NULL) ;

// CreateUVTrimCurve exits on low-work conditions - document those                                                             
#define SM_SET_CREATE_UVTRIMCURVE_FAIL(status, ePathType) pDropCurveFail->Set(status, 3, ePathType,\
                                                            pContext, pSurface->GetNaturalUVDomain(), pSurface, pCurve, pCurve->GetNaturalInterval(), SmTol::GetApproxTol3d(), \
                                                            0.0, 0.0, NULL, UNSURE, \
                                                            UNSURE, UNSURE, UNSURE, UNSURE, 0.0) ;  



/*******************************************************************//**
PURPOSE: This is the superclass of all surfaces.  Most of the numerical
   methods on surfaces reside here.

NOTES: 
***********************************************************************/
class SM_EXPORT SmSurface : public SmAObject
{
public:
  SmObject * m_pOwner;    // SmFace ptr when used by a single Face.
                          //        The standalone face will point to this surface.
// Remove Composites
//                          // SmCFace ptr when used by a composite Face.
//                          //        The CompositeFace and each of its sibling Faces
//                          //        will point to this surface.
                          // SmSurface ptr when used as a GenSurface for another Surface, ex: SmSrfInVolume, SmOffsetSurface
                          // SmCurve ptr when used as a GenSurface for a Curve, ex: SmCrvOnSurf
                          // NULL when not used by a Face, CFace, or as a GenSurface. 
                          // Cache Control: All surfaces are cached in SmTrimSrfCache objs
                          //                whether used by a face or not.

  // mutable std::recursive_mutex * mCacheMutex; // Mutex for tessellation parallelization JLMCC removed to address compiler warnings. Revisit.
#ifdef SM_DEBUG_CACHE_H
public:
  ULONG m_lSurfaceCacheCount ;
  ULONG m_lTrimSrfCacheCount ;
#endif                           

//cbi Temporary: start:
  SmBoolean m_bUseNewPointSolvers;
public:
  SmBoolean SetUseNewPointSolvers( SmBoolean bUseNew = TRUE ) { SmBoolean bRet = m_bUseNewPointSolvers;
                                                                m_bUseNewPointSolvers = bUseNew;
                                                                return bRet;
                                                              }

  SmBoolean IsUsingNewPointSolvers() { return m_bUseNewPointSolvers; }

//cbi Temporary: end.
      
public:

  // Empty Constructor for I/O
  SmSurface() ;

  // copy constructor
  SmSurface(const SmSurface& crSourceSurface) ;

  // equality operator
  virtual SmBoolean operator==(const SmSurface &crOther) const ;

  // make an exact copy of any surface
  virtual SmStatus Copy(const SmContext & crContext,          // in :
                        SmSurface      *& rpNewSurface)       // out:
                       const ;

  // when possible make a SmBSpline Copy of an Analytic Surface - else set output to NULL
  virtual SmStatus CopyAnalyticAsNurb(const SmContext & crContext,          // in :   
                                      SmSurface      *& rpNewNurbSurface)   // out:   
                                     const ;

  // when possible make an analytic copy of a BSplineSurface - else make exact copy                          
  virtual SmStatus CopyAndAddAnalytics(const SmContext & crContext,          // in :   
                                       SmSurface      *& rpNewSurface)       // out:   
                                      const ;

  // destructor
  virtual ~SmSurface() { // derived classes do not duplicate this call
                         Notify( SM_NO_DESTRUCTION, this, NULL, NULL ) ;
                         m_pOwner = NULL;
                       }

  // general methods

  SmBoolean AdjustPeriodicityUVPoints(SmExtent2d          & rUVDomain,  // i/o:   
                                      SmTArray<SmPoint2d> & rUVPoints)  // i/o:   
                                     const ;

  // Set UVDomain from given STEP UVDomain and rebuild associated m_pNurb.
  virtual SmStatus AdjustSTEPUVDomain(const SmExtent2d & crNewSTEPUVDomain)    { SM_REF1( crNewSTEPUVDomain ) ;
                                                                                 SE( SM_ERR ) ;
                                                                                 return SM_ERR;
                                                                               }

  // Set UVDomain from given Nurb UVDomain and don't rebuild associated m_pNurb.
  virtual SmStatus UpdateAnalyticalDomain(const SmExtent2d & crNewNurbsDomain) { SM_REF1(crNewNurbsDomain) ;
                                                                                 return SM_SUCCESS;
                                                                               }

  // Approximate to tolerance any surface as a piece-wise bezier surface (large control point counts)
  SmStatus ApproximateSurface(const SmContext   & crContext,                 // in : context for new object construction
                              SmApproxTol3d       sApproxTol3d,              // in : max allowed approximation distance                                        
                              double            & dAchievedTol,              // out: Max tolerance achieved.                                                   
                              ULONG               lSubdivisionLevel,         // in : How many times to split spans before giving up                            
                                                                             //    : Splits this many times in each direction (u/v).                           
                              SmBSplineSurface *& rpNewBSplineSurface,       // out: approx surf with the smallest number of                                   
                                                                             //    :  subdivisions possible within tol of exact surf.                          
                                                                             //    : NULL = finest allowed subdivision surf still out of tol                   
                              SmBoolean           bJustCopyBSplines=FALSE)   // in : TRUE = if this Surface is a BSpline, copy it FALSE= approximate the surface
                             const ;                                                

  // Approximate to tol this surface constrained to trim curves and trim points while minimizing control point count
  virtual SmStatus ApproximateConstrainedSurface(const SmContext            & crContext,                // in : context for new object construction                                                        
                                                 double                       dApproxTol,               // in : maximum allowed distance between Approx and original surfaces                              
                                                 double                     & dAchievedTol,             // out: actual max distance between approx and original surfaces                                   
                                                 SmBSplineSurface          *& rpNewBSplineSurface,      // out: The new surface when successful, else NULL.                                                
                                                                                                        //    : expected to be NULL on input - otherwise can cause a memory leak                           
                                                 SmTArray<SmPoint3d>        * pOpt3DTrimPoints=NULL,    // in : optional array of 3d Point constraints, NULL to ignore, default:[NULL]                     
                                                 SmTArray<SmPoint2d>        * pOptUVTrimPoints=NULL,    // in : associated surface UVPoints made close to each 3DTrimPoint, NULL to ignore, default:[NULL] 
                                                 SmTArray<SmCurve *>        * pOpt3DTrimCurves=NULL,    // in : optional array of 3d Curve constraints, NULL to ignore, default:[NULL]                     
                                                 SmTArray<SmBSplineCurve *> * pOptUVTrimCurves=NULL,    // in : associated UVTrimCurve for each given 3d Curve, NULL to ignore, default:[NULL]             
                                                 SmTArray<SmExtent1d>       * pOptTrimIntervals=NULL) ; // in : associated trim intervals for each given 3d Curve, NULL to ignore, default:[NULL]          
                                                      
  SmStatus ApproximateSurfaceLeastSqs(SmBSplineSurface *& rpNewBSplineSurface) ; 
  
  // return polygon length and width between 5 equally spaced U and V SamplePoints of the 
  SmStatus ApproximateSize(SmVector2d &rLWSize,              // out: rLWSize.x = approx length in U direction                  
                           SmExtent2d * pOptDomain = NULL)   //    : rLWSize.y = approx width in V direction                   
                                                             // in : optional sub domain to query, NULL = use NaturalUVDomain  
                          const ;                           

  virtual SmVector2d ApproxDerivativeLengths( const SmExtent2d & crUVDomain)  const ;

  virtual SmStatus   Area(const SmExtent2d & crUVDomain,
                          double             dDesiredAccuracy, 
                          double           & rdArea)
                         const ;

  // used by SmCacheMgr::GetOrCreateObjectCache()
  // create a new or fetch an existing ObjectCache - place ObjectCache in m_pCacheObj
  SmStatus CacheMakeOrValidate(SmObjectCacheType   eObjectCacheType,      // in : oneof SM_OC_CURVE                               
                                                                          //    :       SM_OC_SURFACE                             
                                                                          //    :       SM_OC_TRIMSRF                                
                                                                          //    :       SM_OC_BREP                                           
                               const SmAObject   * pObject,               // in : target object                                   
                               SmCacheObj        * pOldCache,             // in : existing target Object's ObjectCache or NULL    
                               SmCacheObj       *& rpNewCache)            // out: ptr to target object's ObjectCache              
                              const ;

  virtual SmStatus CalculateBoundingBox
   (const SmExtent2d  & crUVDomain,                         // NotUsed: in : returns whole surface bounding box
                                        SmExtent3d        * pNormalBox = NULL,            // out: Axis aligned box                                                                          
                                        SmPseudoBox       * pPseudoBox = NULL,            // out: Non-axis aligned box                                                                      
                                        SmPolarBox        * pPolarBox = NULL,             // out: Surface normal vector field bounding box                                                  
                                        const SmPseudoBox * pOptPseudoBasisGuess = NULL,  // NotUsed: in : guess for basis vectors - used unless another better orientation is found NULL to ignore  
    const SmPolarBox  * pOptPolarBasisGuess = NULL,         // NotUsed: in : guess for basis vectors - used unless another better orientation is found NULL to ignore  
    SmBoolean           bExpandPosBoxesByZoneTol3d = TRUE)  // in : TRUE = returned Normal & Pseudo BBoxes = BBox->ExpandAbsoluate(ZoneTol3d)
   const ;                                                  //    : FALSE= returned Normal & Pseudo BBoxes = BBox with no expansion 
                                                            //      default:[TRUE] = previous behavior
                                       

  virtual SmStatus CalculateTightBoundingBox(const SmExtent2d & crUVDomain,               // in : domain of interest on the surface.  
                                             SmExtent3d       * pNormalBox = NULL)        // out: Axis aligned box                    
                                            const ;

  virtual SmStatus CalculateContinuities(SmSurfParamType              eSurfParam,                     // in : SM_SP_U or SM_SP_V                                                                      
                                         SmContinuityType           & reMinContinuityInSurfaceKnot,   // out: minimum continuity over all interior knots                                              
                                         SmTArray<SmContinuityType> & rContinuitiesAtKnots)           // out: Continuity value for each unique knot value end knot continuities = SM_CT_DISCONTINUOUS 
                                        const ;

  SmStatus CalculateApproxIsoCurveDistance(SmSurfParamType    eSurfParam,           // in : oneof SM_SP_U or SM_SP_V
                                           const SmExtent2d & crDomain,             // in : This domain limits the length of the isoCurve sample points                              
                                                                                    //    : eSurfParam = SM_SP_U: minV = crDomain.Min.y, maxV = crDomain.Max.y                       
                                                                                    //    : eSurfParam = SM_SP_V: minU = crDomain.Min.x, maxU = crDomain.Max.x                       
                                           const SmExtent1d & crInterval,           // in : Min and Max values of this interval define the                                           
                                                                                    //    : isoCurves being measured                                                                 
                                                                                    //    : eSurfParam = SM_SP_U: max dist between Curve(crInterval.Min,v) and Curve(crInterval.Max,v)
                                                                                    //    : eSurfParam = SM_SP_V: max dist between Curve(u,crInterval.Min) and Curve(u,crInterval.Max)
                                           ULONG              lNumSamplesAlong,     // in : number of places along each isocurve - checked for distance (max dist returned)          
                                           ULONG              lNumSamplesBetween,   // in : numer of samples between isocurves used to compute distance at each along point          
                                           double           & rdDistance)           // out: max distance found between the two isocurves measued at every sample along               
                                          const ;

  virtual SmStatus CalculateFilletRadius(SmSurfParamType eRailDir,                // in : see Usage Notes                                       
                                         double          dParam,                  // in : u- or v- parameter                                    
                                         double        & dRadius,                 // out:                                                       
                                         SmBoolean     * pbIsCircular = NULL,     // out: True iff the cross section is circular                
                                         double        * dTol = NULL)             // in : tolerance for circular check                          
                                                                                  //    : default: NULL, in which case we use SM_EFF_ZERO_SQRT  
                                        const ;                                   
                                             
  // see if these surfaces are coincident or if either of the surfaces currently or can be extended to contain the other
  virtual SmStatus CoincidenceCheck(const SmSurface & crSurface,             // in : target surface to compare                                                              
                                    double            d3DTolerance,          // in : max allowed distance between coincident surfaces                                       
                                    SmBoolean       & rbAreCoincident,       // out: TRUE = surfaces are the same to within tolerance                                       
                                    double          & rMaxDistanceBetween,   // out: For total coincidence check, max distance seen between matched sample surface points.  
                                                                             //    : Not used for partial overlap check.                                                    
                                    SmBoolean         bAllowExtensions,      // in : TRUE = Test whether surfaces are totally coincident                                    
                                                                             //    : FALSE= Test whether there is any coincident overlap                                    
                                    SmSurface      *& rpContainingSurface)   // out: (Not used.)                                                                            
                                   const ;

  SmStatus ComputeCubicBezierApprox(const SmExtent2d     & crUVDomain,
                                    SmCubicBezierSurface & rCB)
                                   const ;

  SmStatus ComputeCubicBezierSplit(const SmExtent2d           & crUVDomain,         // in :      
                                   SmSurfParamType              eSplitDirection,    // in :      
                                   const SmCubicBezierSurface & crCB,               // in :      
                                   SmExtent2d                 & rUVDomain1,         // in :      
                                   SmExtent2d                 & rUVDomain2,         // in :      
                                   SmCubicBezierSurface       & rCB1,               // out:      
                                   SmCubicBezierSurface       & rCB2)               // out:      
                                  const ;

  // compute parameter direction for given 3D tangent from chain rule - output vec is not unitized
  SmStatus ComputeParamDirection(const SmPoint2d  & crUV,        // in : UV Point to evaluate                                              
                                 const SmVector3d & crVec,       // in : 3D vector to project to UV space                                  
                                 SmVector2d       & rUVDir)      // out: UV parameter direction that maps to crVec Vector is not unitized  
                                const ;

  // approximate directional derivatives with finite differences                                        
  SmStatus ComputeHigherOrderDerivs(const SmPoint2d  & crUV,                 // in : Point where derivatives are to be computed                         
                                    const SmVector2d & crG1Deriv,            // in : First order derivative of a curve on the surface at the crUV point.
                                    SmVector3d       & rG2Deriv,             // out: Computed second order derivative of the curve                      
                                    SmVector3d       * pOptG3Deriv = NULL,   // out: Computed third order derivative of the curve                       
                                    SmVector3d       * pOptG4Deriv = NULL)   // out: Computed fourth order derivative of the curve                      
                                   const ;

  // get directional unit 1st-deriv in the UV direction to the domain center 
  SmStatus ComputeInwardVector(const SmPoint2d & crUV,                   // in :       
                               SmVector3d      & rInwardVector)          // out:       
                              const ;

  // compute X=1st directinal derivative, Y=SurfaceNormal cross X
  SmStatus ComputeLocalCurveFrame(double            dParam,       // in :      
                                  const SmCurve   & crUVCurve,    // in :      
                                  SmOrientType      eOrientation, // in :      
                                  SmPoint3d       & rOrigin,      // out:      
                                  SmVector3d      & rXAxis,       // out:      
                                  SmVector3d      & rYAxis)       // out:      
                                 const ;
  
  double ComputeMaxAspectRatio (void) const ;


  virtual SmStatus ConvertUVFromSTEPToNURBS(const SmPoint2d & crSTEPUV,            // in : STEP Domain UV point     
                                            SmPoint2d       & rNURBSUV)            // out: Nurb Domain UV point     
                                           const ;                                 
                                                                                   
  virtual SmStatus ConvertUVFromNURBSToSTEP(const SmPoint2d & crNURBSUV,           // in : 
                                            SmPoint2d       & rSTEPUV)             // out: 
                                           const ;                                 
                                                                                   
  SmStatus         ConvertDomainFromSTEPToNURBS(const SmExtent2d & crAnalDomain,       // in : analytic domain to translate    
                                                SmExtent2d       & rNurbDomain)        // out: Nurb domain to specify          
                                               const ;                             
                                                                                   
  SmStatus         ConvertDomainFromNURBSToSTEP(const SmExtent2d & crNurbDomain,       // in : analytic domain to translate    
                                                SmExtent2d       & rAnalDomain)        // out: Nurb domain to specify          
                                               const ;

  // make exact BSpline equivalent surface when possible - when Surface is already a BSpline no new curve is built
  virtual SmStatus MakeExactBSplineIfPossible(SmBSplineSurface *& rpNewBSplineSurface) const
                     { rpNewBSplineSurface = NULL; return(SM_SUCCESS) ; }

  // See whether this surface can contain the other, possibly expanding this.
  virtual SmStatus CoverCoincidentSurface(const SmSurface  * pOther,             // in : other surface                                 
                                          double       dTol,                     // in : max separation allowed                        
                                          SmBoolean  & rbSuccess,                // out: pOther will fit inside this; see notes above  
                                          double     & rdMaxDist,                // out: max distance found; see notes above           
                                          SmExtent2d * pOtherDomain = NULL,      // in : optional: use this subset of pOther
                                          SmExtent2d * pOptCoveredSTEPDomain = NULL) ; // out: covered subset in this surface's STEP parameters
                                                                                       //      valid when rbSuccess is TRUE

  // See whether either surface can contain the other, possibly expanding it.
  virtual SmStatus CoverOtherCoincidentSurface(SmSurface   * pOther,                  // in : other surface                                             
                                               double        dTol,                    // in : max separation allowed                                    
                                               double      & rdMaxDist,               // out: max distance found between covering and covered surfaces  
                                               SmSurface  *& pCoveringSurf,           // out: covering surface, NULL if not possible
                                               const SmExtent2d * cpOptThisDomain = NULL,  // in : subset of this in NURBS parameters; NULL uses natural domain
                                               const SmExtent2d * cpOptOtherDomain = NULL, // in : subset of pOther in NURBS parameters; NULL uses natural domain
                                               SmExtent2d * pOptMergedDomain = NULL) ;    // out: union in covering surface's final NURBS parameters
                                                                                          //      valid only when pCoveringSurf is not NULL

  virtual SmStatus CreateIsoParametricCurve(const SmContext  & crContext,              // in : context for created objects
                                            SmSurfParamType    eSurfParam,             // in : Defines which Nurb parameter direction on surface to extract curve from                   
                                                                                       //    : SM_SP_U = create constant u isoParameter curve                                            
                                                                                       //    : SM_SP_V = create constant v isoParameter curve                                            
                                            double             dIsoParameter,          // in : Defines Nurb parametric value at which to extract the curve.                              
                                                                                       //    : If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V                      
                                                                                       //    : then this is the V parameter                                                              
                                            SmApproxTol3d      sApproxTol3d,           // in : passed to ApproximateCurve() when approximation is required.                              
                                                                                       //    : If set to 0.0, tolerance is set by system: old[curve length * 1.0e-4] new[GetApproxTol3d()]
                                            SmBSplineCurve  *& rpNewIsoCurve,          // out: 3d IsoParameterCurve                                                                      
                                            const SmExtent2d * pOptDomain = NULL,      // in : optional trim bound for IsoParameterCurve, NULL to ignore, default:[NULL]                 
                                            double           * pOptMaxGap3d = NULL,    // out: opt achieved max gap, NULL to ignore, default:[NULL]                                      
                                            SmCurve         ** pOptUVIsoCurve = NULL)  // out: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]           
                                           const ;

  // create a CrvOnSurf SeamCurve for closed surfaces only                                             
  SmStatus CreateSeamCurve(const SmContext  & crContext,           // in : context for new Obj construction
                           SmSurfParamType    eSurfParam,          // in : SM_SP_U = ConstU, VaryV, SM_SP_V = VaryU, ConstV                            
                           SmCrvOnSurf     *& rpNewSeamCurve,      // out: 3DSeam curve:[SmCrvOnSurf_TYPE], ParamIvl:[0,1] - NULL when Srf is not closed, NULL on input
                                                                   //    : new memory - to be deleted by caller                                        
                           SmBoolean          bOptIsClosed=UNSURE) // in : TRUE = save some time, Surf known to be closed                              
                                                                   //    : FALSE or UNSURE = chk Surf for closure, default:[UNSURE]                    
                          const ;          

  // create a UVLine CrvOnSurf Curve given two UVPoints
  SmStatus CreateLineOnSurf(const SmContext  & crContext,          // in : context for new Obj construction               
                            SmPoint2d        & rStartUV,           // in : StartPt of UVLine(StartPt,EndPt)               
                            SmPoint2d        & rEndUV,             // in : EndPt of UVLine(StartPt,EndPt)                 
                            SmCrvOnSurf     *& rpNewLineOnSurf)    // out: 3DProj of UVLine(StartPt,EndPt), NULL on input 
                           const ;
                          
  // create CrvOnSurf with UVCurve an arc created from a UVCenter pt, UVRadius, and UVStart and UVStop angles
  // and surf = this surface.  (input args can be found in the SmVertexuse::ComputeUVSector() output making it
  //                            convenient to create sector graphics with arcs marking sector interiors)
  SmStatus CreateArcOnSurf(const SmContext  & crContext,       // in : context for new Obj construction                            
                           SmPoint2d        & rCenterUV,       // in : CenterPt of UVArc(CenterPt,XAxis,YAxis,IvlDeg,radius)       
                           SmVector2d       & rBegUVTan,       // in : StartPt  of UVArc(CenterPt,XAxis,YAxis,IvlDeg,radius)       
                           double             dSectorAngDeg,   // in : CCW rotation from BegUVTan to EndUVTan                      
                           double             dRadius,         // in : radius   of UVArc(CenterPt,XAxis,YAxis,IvlDeg,radius)       
                           SmBoolean        & rOutsideDomain,  // out: TRUE = rtn arc runs outside defined surface domain          
                                                               //    : FALSE= rtn arc contained inside defined surface domain      
                           SmCurve         *& rpNewArcOnSurf)  // out: 3DProj of UVArc(CenterPt,StartPt,EndPt), Type:[SmCrvOnSurf], NULL on input
                          const ;

  // build a blend a pair of boundary curves and their specified cross-derivatives
  SmStatus CreateBlendSurface(const SmContext        & crContext,          // in : context for new object construction                              
                              const SmCurve          & crCurve1,           // in : Boundary Curve on this Surface starting the blend                
                              SmExtent1d             & rIvl1,              // in : interval of Curve1 to blend                                      
                              SmBoolean                bOffToTheRight,     // in : TRUE = begin blend moving to the right hand side of curve1       
                              const SmSurface        & crSurface2,         // in : Surface of Boundary Curve2                                       
                              const SmCurve          & crCurve2,           // in : Boundary Curve on Surface2 ending the blend                      
                              SmExtent1d             & rIvl2,              // in : interval of Curve2 to blend                                      
                              SmBoolean                bInFromTheLeft,     // in : TRUE = end blend moving into the curves from the left hand side  
                              double                   dTolerance,         // in : 0.0 = interpolate sampled cross-tangent values, else approximate 
                              SmDerivSurfDefinition  * pOptDSDef,          // in : Container for blend generation options, NULL = use default values
                              SmSurface             *& rpNewBlend) ;       // out: The blend surface                                                

  SmStatus CreateIsoBoundaries(const SmContext & crContext,        // in : context for new object construction              
                               SmSurfParamType   eSurfParam,       // in : oneof SM_SP_U (u=const), SM_SP_V(v=const)        
                               SmApproxTol3d     sApproxTol3d,     // in : max allowed distance between curves and surface  
                               SmBSplineCurve *& rpMinIsoCurve,    // out: min iso parameter boundary                       
                               SmBSplineCurve *& rpMaxIsoCurve)    // out: max iso parameter boundary                       
                              const ;

  virtual SmStatus CreateExtendedSurface(const SmContext   & crContext,             // in : context for new object construction                  
                                         double              dDist,                 // in : Distance of extension from each side                 
                                         SmContinuityType    eExtensionContinuity,  // in : OneOf: SM_CT_G1 - linear extension                   
                                                                                    //    :        SM_CT_G1R -                                   
                                                                                    //    :        SM_CT_G1_G2 - extension with second derivative
                                                                                    //    :        SM_CT_CINFINITY - infinite continuity         
                                         SmSurface *& rpExtended)                   // out: the newly constructed surface (NULL on input)        
                                                                                    { SE( SM_ERR ) ;
                                                                                      SM_REF4(crContext, dDist, eExtensionContinuity, rpExtended) ;
                                                                                      return SM_ERR;
                                                                                    }

  virtual SmStatus CreateExtendedSurface(const SmContext   & crContext,             // in : context for new object construction                  
                                         SmSurfParamType     eExtDirection,         // in : Either extend in SM_SP_U or SM_SP_V direction        
                                                                                    //    : or SM_SP_UMIN/VMIN/UMAX/VMAX/BOTH                    
                                         double              dDist,                 // in : Distance of extension from each side                 
                                         SmContinuityType    eExtensionContinuity,  // in : oneof: SM_CT_G1 - linear extension                   
                                                                                    //    :        SM_CT_G1R -                                   
                                                                                    //    :        SM_CT_G1_G2 - extension with second derivative
                                                                                    //    :        SM_CT_CINFINITY - infinite continuity         
                                         SmSurface *& rpExtended)                   // out: the newly constructed surface (NULL on input)        
                                                                                    { SER( SM_ERR ) ;
                                                                                      SM_REF5(crContext, eExtDirection, dDist, eExtensionContinuity, rpExtended) ;
                                                                                      return SM_ERR;
                                                                                    }

  virtual SmStatus CreateMirrorSurface(const SmContext        & crContext,         // in :     
                                       const SmAxis2Placement & crMirrorPlane,     // in :     
                                       SmSurface             *& rpMirrorSurface )  // out:     
                                      const                                        { SM_REF3( crContext, crMirrorPlane, rpMirrorSurface ) ;
                                                                                     SE( SM_ERR ) ; return SM_ERR;
                                                                                   }

  // create UVTrimCurve/3D curve pairs in counterclockwise sequence for input UVDomain (Natural Curves when UVDomain == UVNaturalDomain)
  SmStatus CreateNaturalUVTrimCurves(const SmContext           & crContext,                // in : context for new object construction                                            
                                     const SmExtent2d          & crUVDomain,               // in : desired Nurb domain, may be a sub-domain of the Surface Natural Trim Domain                                                            
                                     SmSurfParamType             eTgtDomainSide,           // in : SM_SP_U == go (0,1)->(0,0)->(1,0)->(1,1)->(0,1)                                
                                                                                           //    : else       go (0,0)->(1,0)->(1,1)->(0,1)->(0,0)                                
                                     SmBoolean                   bRemoveDegenerateCurves,  // in : TRUE = remove degenerate curves.                                               
                                     SmTArray<SmCurve*>        & r3DTrimmingCurves,        // out: 3D Trimming Curves                                                             
                                     SmTArray<SmBSplineCurve*> & rUVTrimmingCurves,        // out: UV Trimming Curves                                                             
                                     SmTArray<SmOrientType>    & rOrients,                 // out: associated Orientation of the trimming curves.                                 
                                     SmTArray<double>          * pOptMaxGap3d = NULL)      // out: optional achieved max gaps for each TrimmingCurve NULL to ignore, default:[NULL]
                                    const ;                                                
                                                                                           

  // create one CrvOnSurf curve with defining SmLine UVTrimCurve running in CounterClockwise direction on the specified border of the given UVDomain                                    
  SmStatus CreateBoundaryCrvOnSurf(const SmContext   & crContext,      // in : context for new object construction                                
                                   const SmExtent2d  & crUVDomain,     // in : desired Nurb domain                                                
                                   SmSurfParamType     eTgtDomainSide, // in : oneof: SM_SP_UMIN= rtn:UVTrimCurve for corners (0,1) -> (0,0)      
                                                                       //    :        SM_SP_VMIN= rtn:UVTrimCurve for corners (0,0) -> (1,0)      
                                                                       //    :        SM_SP_UMAX= rtn:UVTrimCurve for corners (1,0) -> (1,1)      
                                                                       //    :        SM_SP_VMAX= rtn:UVTrimCurve for corners (1,1) -> (0,1)      
                                   SmCrvOnSurf      *& pCrvOnSurf,     // out: UV Curve bordering crUVDomain- NULL on input                       
                                   SmOrientType      & rOrient)        // out: associated Orientation of the trimming curves.  NULL to ignore     
                                  const ;                             
  
  static SmStatus CreatePlaneFromBBox(const SmContext  & crContext,        // in : context for new object construction                               
                                      const SmExtent3d & crBoundingBox,    // in : Bounding box to create plane through.                             
                                      const SmPoint3d  & crPlanePoint,     // in : Defines base point of an infinite plane                           
                                      const SmVector3d & crPlaneNormal,    // in : Defines the normal of an infinite plane                           
                                      SmSurface       *& rpNewPlane,       // out: newly constructed plane limited by plane/BBox intersection        
                                                                           //    : or NULL when plane does not intersect BBox                        
                                      double             dOptTol = 0.0) ;  // in : max distance between BBOx and intersecting plane                  
                                                                           //    : 0.0 = scale tol to size of BoundingBox and Position of PlanePoint 

  static SmStatus CreatePlaneWith3DCurves(const SmContext             & crContext,                 // in : context for new object construction                                            
                                          const SmTArray<SmCurve*>    & crInputCurves,             // in : array of 3d Curves                                                             
                                          double                        d3DTol,                    // in : Tol for detecting degenerate curves                                            
                                          SmSurface                  *& rpNewPlanarSurface,        // out: New Plane Surface                                                              
                                          SmTArray<ULONG>             & sLoopCounts,               // out: number of curves in each loop, sized:[NumLoops]                                
                                          SmTArray<SmCurve*>          & sOrderedCurves,            // out: curves ordered into loops (not copied) sized:[NumCrvs] as                      
                                                                                                   //    : [Sequence of[OuterLoop CCW CrvSet followed by its InnerLoop CW CrvSets]]       
                                          SmTArray<SmOrientType>      & sOrients,                  // out: orientation for each curve in its loop                                         
                                          SmTArray<ULONG>             * pOuterLoopCounts = NULL) ; // out: Num of Loops in each outerLoop, sized:[NumOuterLoops]                          
                                                                                                   //    : ex: pOuterLoopCounts[i]=3 => 2 innerLoops inside OuterLoop[i] NULL to ignore   

  // create offset surface - Approximate When necessary - Out Surf->Domain(s) may be trimmed but not scaled
  virtual SmStatus CreateOffsetSurface(const SmContext      & crContext,               // in : context for new obj construction                                           
                                       double                 dSignedOffsetDistance,   // in : offset dist, (neg val = Offset dir opposite surface normal)                
                                       SmApproxTol3d          sApproxTol3d,            // NotUsed: in : Max Dist between ApproxOffsetSurface and ideal offset shape                
                                       SmSurface* &           rOffsetSurface)          // out: Offset Surf Approx, may have self-intersections                            
                                      const ; 

  virtual SmStatus CreateParallelProjectionCurves(const SmContext      & crContext,                 // in : context for new object construction                        
                                                  const SmExtent2d     & crUVDomain,                // in : surface domain limit                                       
                                                  const SmBSplineCurve & crCurveToProject,          // in : Curve being projected                                      
                                                  const SmVector3d     & crProjectionVector,        // in : Direction vector in with projection occurs                 
                                                  const double         * cpdOptProjectionDistance,  // in : If specified defines the distance of the projection        
                                                  const SmApproxTol3d  * pOptApproxTol3d,           // in : If specified defines the accuracy of the projection curve  
                                                                                                    //    : relative to the exact projection of the curve              
                                                  const double         * cpdOptAngleTolRadians,     // in : If specified defines the maximum angle of a single segment 
                                                                                                    //    : of the resulting approximating curve.                      
                                                  SmTArray<SmCurve*>   * pOpt3DProjectedCurves,     // out: 3D curves approximating the projection                     
                                                  SmTArray<SmCurve*>   * pOptUVProjectedCurves)     // out: UV curves in the surface approximating the projection      
                                                 const ;

  virtual SmStatus CreatePlanarSectionCurves(const SmContext     & crContext,              // in : context for new object construction                       
                                             const SmExtent2d    & crUVDomain,             // in : surface domain of interest                                
                                             const SmPoint3d     & crPlanePoint,           // in : Defines base point of an infinite sectioning plane        
                                             const SmVector3d    & crPlaneNormal,          // in : Defines the unit normal of an infinite sectioning plane   
                                             const SmApproxTol3d * pOptApproxTol3d,        // in :                                                           
                                             const double        * cpdOptAngleTolRadians,  // in :                                                           
                                             SmTArray<SmCurve*>  * pOpt3DSectionCurves,    // out: 3D curves produced by sectioning operation                
                                             SmTArray<SmCurve*>  * pOptUVSectionCurves)    // out: this surface UVTrimcurves produced by sectioning operation
                                            const ;

  virtual SmStatus CreateSilhouetteCurves(const SmContext     & crContext,                 // in : context for new object construction                   
                                          const SmExtent2d    & crUVDomain,                // in : target domain                                         
                                          const SmPoint3d     & crEyePointOrViewVector,    // in : If parallel projection this is a view vector.         
                                                                                           //    : If perspective projection this is the eye point.      
                                          SmBoolean             bPerspective,              // in : TRUE  = perspective projection,                       
                                                                                           //    : FALSE = parallel projection                           
                                          const SmApproxTol3d * pOptApproxTol3d,           // in :                                                       
                                          const double        * cpdOptAngleTolRadians,     // in :                                                       
                                          SmTArray<SmCurve*>  * pOpt3DSilhouetteCurves,    // out: 3D silhouette curves produced by silhouette operation 
                                          SmTArray<SmCurve*>  * pOptUVSilhouetteCurves)    // out: UV silhouette curves produced by silhouette operation 
                                         const ;
  
  virtual SmStatus CreateSurfaceRay(const SmContext  & crContext,                          // in : Context for new objects                                                      
                                    const SmExtent2d & crUVDomain,                         // in : Surface domain to consider                                                   
                                    const SmPoint2d  & crStartPoint,                       // in : Ray Start Point on Surface                                                   
                                    SmBoolean          bCurveInParameterSpace,             // in : TRUE  = build 2d Curve, FALSE = build 3d curve                               
                                    SmBoolean          bTowardUpperBoundaryOfDomain,       // in : TRUE  = direct ray up   FALSE = direct ray down                              
                                    SmSurfParamType    eSurfParam,                         // in : parameter held constant                                                      
                                    SmApproxTol3d      sApproxTol3d,                       // in : Tol passed to CreateIsoParametricCurve() when bCurveInParameterSpace == FALSE
                                    SmBSplineCurve  *& rpRayCurve)                         // out: Ray                                                                          
                                   const ;

  // Find the maximum closest approach between a curve and this surface.
  // Just calls the SmCurve method.
  virtual SmStatus CurveMaxDistanceBetween(const SmExtent2d & crSurfaceDomain,        // in : surface domain of interest                              
    const SmCurve    & crCurve,                // in : curve to test                                           
    const SmExtent1d & crCurveInterval,        // in : curve domain of interest                                
    ULONG              lNumSamples,            // in : 0 for precise measurement                               
    double           * pdOptMaxDistanceNeeded, // in : If specified, quit sampling as soon as this is exceeded
    double           & rdMaxDistanceFound,     // out:                                                         
    double           * pOptCurveT=NULL,        // out: curve parameter of biggest gap
    SmPoint2d        * pOptSurfParam=NULL)     // out: surface parameter of biggest gap
   const                                       { return crCurve.SurfaceMaxDistanceBetween(crCurveInterval,
                                                                                         *this,
                                                                                          crSurfaceDomain,
                                                                                          lNumSamples,
                                                                                          pdOptMaxDistanceNeeded,
                                                                                          rdMaxDistanceFound,
                                                                                          pOptCurveT,
                                                                                          pOptSurfParam) ;
                                               }

  // make 1 UVTrimCurve for 3dCurve when curve drops completely within surface, else make no curve
  //   special case: (When 3dCurve drops to closed surface seam curve - make 2 UVTrimCurve) 
  virtual SmStatus DropCurve(const SmContext           & crContext,                  // in : context for new object construction                                                         
                             const SmExtent2d          & crUVDomain,                 // in : domain of interest for this surface                                                         
                             const SmCurve             & cr3dCurve,                  // in : Curve to project onto the surface                                                           
                             const SmExtent1d          & crInterval,                 // in : interval of interest for target curve                                                       
                             SmApproxTol3d               sApproxTol3d,               // in : max allowed distance between drop point and m_SrfNormal line at drop point                  
                             double                    & rdMaxDropToSurf,            // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.        
                             double                    & rdMaxApproxDev,             // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0          
                             SmTArray<SmBSplineCurve*> & rUVCurves,                  // out: 1 (or 2) curves constructed by projection.                                                    
                                                                                     //    : (2 curves for closed surfaces when cr3dCurve is coincident with seam)                         
                             SmBoolean                   bKeepAllDropCurves=TRUE,    // in : TRUE = return all drop curves that stay over the surface (those that wander off do not drop)
                                                                                     //    : FALSE= only return drop curves with drop distances less than dApproxTol                     
                             SmDropCurveFail            * pOptDropCurveFail=NULL,    // out: Optional data container of a DropCurve fail, not used otherwise                             
                                                                                     //    : NUll to ignore, default:[NULL]                                                              
                             SmBoolean                   bCreatingUVTrimCurve=TRUE)  // NotUsed: in :                                                                                             
                            const ;                       

  // make UVTrimCurves for each portion of the 3dCurve completely within the surface
  //   special case: (when 3dCurve drops to closed surface seam curve - makes pairs of UVTrimCurves)
  SmStatus DropAndTrimCurve(const SmContext           & crContext,                // in : context for newly created geometry                                                    
                            const SmExtent2d          & crUVDomain,               // in : Domain limits for accepting the dropped curve                                         
                            const SmCurve             & cr3dCurve,                // in : Curve to project onto the surface                                                     
                            const SmExtent1d          & crInterval,               // in : cr3dCurve interval to drop                                                            
                            SmApproxTol3d               sApproxTol3d,             // in : max allowed drop distance. The projected curve is broken up into                      
                                                                                  //    : more than one piece everytime the projection curve goes outside the                   
                                                                                  //    : surface boundary by more than this amount.                                            
                            double                    & rdMaxDropToSurf,          // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.  
                            double                    & rdMaxApproxDev,           // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
                            SmTArray<SmBSplineCurve*> & rUVCurves,                // out: Projected UVTrimCurves                                                                
                            SmBoolean                 bKeepAllDropCurves=TRUE,    // in : TRUE = keep all drop curves                                                           
                                                                                  //    : FALSE=Only keep curves when rdMaxDropToSurf <= sApproxTol3d, default:[TRUE]           
                            SmBoolean                 bOptCurveIsOnSurface=FALSE, // in : TRUE=cr3dCurve is known to lie on the Surface default:[FALSE]                         
                            SmDropCurveFail           * pOptDropCurveFail=NULL)   // out: Optional data container of a DropCurve fail, not used otherwise
                                                                                  //    : NUll to ignore, default:[NULL]
                           const ;

  // make a UVLine when 3dCurve happens to drop to a surface isoParameter curve, else make no curve
  SmStatus DropIsoCurve(const SmContext      & crContext,              // in : context for new object construction                                      
                        const SmExtent2d     & crUVDomain,             // in : This Surface domain of interest                                          
                        const SmCurve        & cr3dCurve,              // in : Curve to drop                                                            
                        const SmExtent1d     & crInterval,             // in : Curve interval of interest                                               
                        const SmPoint2d      & crStartUVPoint,         // in : Surface UVPoint corresponding to start of3d curve.                       
                        SmApproxTol3d          sApproxTol3d,           // in : max allowed distance between drop point and surfNormal line at drop point
                        SmBoolean            & rbSuccessfulIsoDrop,    // out: TRUE = dropped a curve                                                   
                        double               & rdMaxDropToSurf,        // out: DropDist = max sample distance found between cr3dCurve and Surface       
                                                                       //    : taken at cr3dCurve.NumberOfKnot evenly spaced samples                    
                        SmBSplineCurve      *& rpIsoUVCurve,           // out: Pointer to newly allocated UVLine curve or                               
                                                                       //    : NULL if no curve were constructed                                        
                        double               * pdMaxApproxDev = NULL)  // out: Max CurveToDrop smplPoint to DropSurfNormLine dist                       
                       const ;

 // classify a curve known to be on the surface into in/on/out segments
 SmStatus CurveOnClassify(SmCurveClassification & rCurveClassification) const ;

 SmStatus DropPoint(const SmPoint3d      & crPtToDrop,                    // in : target point to drop                                        
                    const SmExtent2d     & crDomain,                      // in : target surface domain                                       
                    const SmPoint2d      * pOptUVGuess,                   // in : When given, uses only local solves                          
                    SmBoolean            & rbSuccess,                     // out: TRUE=Point dropped successfully, FALSE=didn't               
                    SmPoint2d            & rResultUV,                     // out: drop result UVPoint                                         
                    double               & rdGap,                         // out: distance of found point to Pt to drop                       
                    SmBoolean            & rbIsMultivalued,               // out: TRUE = solution is multivalued, FALSE = Single valued
                    SmSolverOperationType  eOperationType=SM_SO_MINIMIZE, // in : SM_SO_MINIMIZE = allow nonNormal drops near boundaries      
                                                                          //    : SM_SO_NORMALIZE= exclude nonNormal drops near boundaries    
                                                                          //    : SM_SO_INTERSECT= point must be on surface, to Tol                                                    
                    SmVector3d           * pOptTgtSeamSide=NULL  ,        // in  Selects uv point from the seam side whose binormal points    
                                                                          //    : in the same dir as the pOptTgtSeamSide vector, ie. the      
                                                                          //    : pOptTgtSeamSide vec pts into the desired side of the face.  
                                                                          //    : when pOptTgtSeamSide=NULL: pick uv pt closest to pOptUVGuess
                                                                          //    : when pOptUVGuess    =NULL: pick uv pt with min drop dist    
                                                                          //    : NULL to ignore, default:[NULL]                              
                    SmSolutionArray      * pOptSolutionArray = NULL)      // out: Array of solutions. NULL to ignore, default:[NULL]  
                   const ;

 virtual SmStatus DropPointFast(const SmExtent2d       & crUVDomain,            // in :     
                                SmSolverOperationType    eSolverOperation,      // in :     
                                const SmPoint3d        & crTestPoint,           // in :     
                                const SmVector3d       * cpOptInPointingVector, // in :     
                                double                   dDistanceTolerance,    // in :     
                                const double           * cpdOptTargetDistance,  // in :     
                                SmSolutionRequestedType  eSolutionRequested,    // in :     
                                SmSolutionArray        & rSolutions)            // out:     
                               const                                            { SM_REF5( crUVDomain, eSolverOperation, crTestPoint, cpOptInPointingVector, dDistanceTolerance ) ;
                                                                                  SM_REF3( cpdOptTargetDistance, eSolutionRequested, rSolutions ) ;
                                                                                  return SM_ERR;
                                                                                }

  virtual SmStatus DropPointsToTangentPlane(const SmPoint2d & crUV,               // in : Point on surface at which to construct a tangent plane.                        
                                            SmBoolean         bUFromLeft,         // in : if P is on U interval boundary                                                 
                                                                                  //    : TRUE  = evaluate P in upper interval where P is on the left of the interval    
                                                                                  //    : FALSE = evaluate P in lower interval where P is on the right of the interval   
                                            SmBoolean         bVFromLeft,         // in : if P is on V interval boundary                                                 
                                                                                  //    : TRUE  = evaluate P in upper interval where P is on the left of the interval    
                                                                                  //    : FALSE = evaluate P in lower interval where P is on the right of the interval   
                                            ULONG             lNumPointsToDrop,   // in : How many points are in the list to be dropped.                                 
                                            const SmPoint3d * caPointsToDrop,     // in : Points to drop to tangent plane.                                               
                                            SmPoint2d       * aResultingUVPoints) // out: Resulting projection of points onto tangent plane in terms                     
                                                                                  //    : of the UV coordinate system.                                                   
                                           const ;

  virtual SmStatus DropVectors(const SmPoint2d  & crUV,                // in : Target UV Point                                                                    
                               SmBoolean          bUFromLeft,          // in : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval 
                               SmBoolean          bVFromLeft,          // in : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval 
                               ULONG              lNumVectorsToDrop,   // in : Number of vectors in following array                                               
                               const SmVector3d * caVectorsToDrop,     // in : Vectors to drop into parameter space.                                              
                               SmVector2d       * aResultingUVVectors) // out: Corresponding parameter space vectors produced by drop                             
                              const ;

  SmStatus Evaluate1stDerivatives(const SmPoint2d & crUV,              // in : target UVPoint to query                                                      
                                  SmBoolean         bUFromLeft,        // in : if P is on U interval boundary                                               
                                                                       //    : TRUE  = evaluate P in upper interval where P is on the left of the interval  
                                                                       //    : FALSE = evaluate P in lower interval where P is on the right of the interval 
                                  SmBoolean         bVFromLeft,        // in : if P is on V interval boundary                                               
                                                                       //    : TRUE  = evaluate P in upper interval where P is on the left of the interval  
                                                                       //    : FALSE = evaluate P in lower interval where P is on the right of the interval 
                                  SmPoint3d       & rPoint,            // out: Surface position                                                             
                                  SmVector3d      & rDU,               // out: U direction tangent vector                                                   
                                  SmVector3d      & rDV)               // out: V direction tangent vector                                                   
                                 const ;        

  SmStatus Evaluate2ndDerivatives(const SmPoint2d & crUV,
                                  SmBoolean         bUFromLeft,        // in : if P is on U interval boundary                                                
                                                                       //    : TRUE  = evaluate P in upper interval where P is on the left of the interval   
                                                                       //    : FALSE = evaluate P in lower interval where P is on the right of the interval  
                                  SmBoolean         bVFromLeft,        // in : if P is on V interval boundary                                                
                                                                       //    : TRUE  = evaluate P in upper interval where P is on the left of the interval   
                                                                       //    : FALSE = evaluate P in lower interval where P is on the right of the interval  
                                  SmPoint3d       & rPoint,            // out:                                                                               
                                  SmVector3d      & rDU,               // out:                                                                               
                                  SmVector3d      & rDV,               // out:                                                                               
                                  SmVector3d      & rDUV,              // out:                                                                               
                                  SmVector3d      & rDUU,              // out:                                                                               
                                  SmVector3d      & rDVV)              // out:                                                                               
                                 const ; 
                                                  
  virtual SmStatus EvaluateNormal(const SmPoint2d & crUV,              // in : surface point within surface domain                                                  
                                  SmBoolean         bUFromLeft,        // in : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval   
                                  SmBoolean         bVFromLeft,        // in : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval   
                                  SmVector3d      & rSurfaceNormal)    // out: unit-vect normal = crossProduct(Wu,Wv)                                               
                                 const ;                                                                                                                    

  // Sizing and Indexing for Evaluate() output arrays by partial deriv cnts.
  // examples: aDerivs[SM_SSIZE(MaxDeriv)], lHighestDeriv=MaxDeriv, 
  //     sW   = aDerivs[SM_SI(0,0)],  - position                            // when MaxDeriv is named 'lHighestDeriv'
  //     sWuu = aDerivs[SM_SI(2,0)],  - 2nd deriv in U direction            // 
  //     sWv  = aDerivs[SM_SI(0,1)],  - 1st deriv in V direction = TangentS // 
  //     sWuw = aDerivs[SM_SI(1,0)],  - cross deriv in UW direction         // 
  //  or sW   = aDerivs[SM_SM(0,0,MaxDeriv)],                               // when MaxDeriv is not named 'lHighestDeriv'
  //     sWv  = aDerivs[SM_S1(0,1)],                                        // when MaxDeriv is known to equal 1
  //     sWuu = aDerivs[SM_S2(2,0)],                                        // when MaxDeriv is known to equal 2
  //     sWuw = aDerivs[SM_S3(1,1)],  - cross deriv in UV direction         // when MaxDeriv is known to equal 3

  // Size of output array for Evaluate() methods
  #define SM_SSIZE(MaxDeriv)    (((MaxDeriv)+1)*((MaxDeriv)+1))

  #define SM_SI(u,v)          ((u)*(lHighestDeriv+1)+(v))    // when MaxDeriv is named 'lHighestDeriv'
  #define SM_SM(u,v,MaxDeriv) ((u)*((MaxDeriv)+1)+(v))       // when MaxDeriv is not named 'lHighestDeriv'
  #define SM_S1(u,v)          ((u)*(2)+(v))        // when MaxDeriv = 1
  #define SM_S2(u,v)          ((u)*(3)+(v))        // when MaxDeriv = 2
  #define SM_S3(u,v)          ((u)*(4)+(v))        // when MaxDeriv = 3
                                                              
  virtual SmStatus Evaluate(const SmPoint2d & crUV,            // in : param value to evaluate                                                                 
                            ULONG            lHighestUDeriv,   // in : number of U derivatives                                                                 
                            ULONG            lHighestVDeriv,   // in : number of V derivatives to compute                                                      
                            SmBoolean        bUFromLeft,       // in : if P is on U interval boundary                                                          
                                                               //    : TRUE  = evaluate P in upper interval where P is on the left of the interval             
                                                               //    : FALSE = evaluate P in lower interval where P is on the right of the interval            
                            SmBoolean        bVFromLeft,       // in : if P is on V interval boundary                                                          
                                                               //    : TRUE  = evaluate P in upper interval where P is on the left of the interval             
                                                               //    : FALSE = evaluate P in lower interval where P is on the right of the interval            
                            SmBoolean        bOnlyUpperHalf,   // in : TRUE=compute upper half of matrix only                                                  
                                                               //    : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value           
                                                               //    :           [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)      
                                                               //    :                         [Dvv  ---   ---]                                                
                            SmVector3d     * aDerivatives,     // out: matrix of evaluations values                                                            
                                                               //    : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                              
                                                               //    : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)   
                                                               //    :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )   
                                                               //    :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                      
                                                               //    :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                      
                                                               //    : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]
                           SmBoolean bNonZeroTangents = TRUE,  // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors            
                                                               //    : FALSE= return exact tangent values                                                      
                                                               //    : note: Surprisingly TRUE is the common choice because most tangent uses                  
                                                               //    :       are for their direction (Binorm, SurfNorm comps), but when the                    
                                                               //    :       tangent is being used for its magnitude (like an arc-length comp)                 
                                                               //    :       then set this to FALSE.                                                           
                                                               //    : default:[TRUE]                                                                          
                           SmBoolean bDoZeroSampling = TRUE)   // in : for internal use only, always set to TRUE, default:[TRUE]                               
                          const                                { SM_REF5( crUV, lHighestUDeriv, lHighestVDeriv, bUFromLeft, bVFromLeft ) ;
                                                                 SM_REF4( bOnlyUpperHalf, aDerivatives, bNonZeroTangents, bDoZeroSampling ) ;
                                                                 SE( SM_ERR ) ; return SM_ERR;
                                                               }

  // evaluate surface without any adjustments for ZeroTangents
  virtual SmStatus EvaluateSimple(const SmPoint2d & crUV,                       // in :                                                                                   
                                  ULONG             lHighestUDeriv,             // in :                                                                                   
                                  ULONG             lHighestVDeriv,             // in :                                                                                   
                                  SmBoolean         bUFromLeft,                 // in : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval
                                  SmBoolean         bVFromLeft,                 // in : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval
                                  SmBoolean         bOnlyUpperHalf,             // in : Should be TRUE                                                                    
                                  SmVector3d      * aDerivatives)               // out:                                                                                   
                                 const ;

  virtual SmStatus EvaluateNormalAtSingularity(const SmPoint2d & crUV,                // in :  
                                               SmVector3d      & rSurfaceNormal,      // out:  
                                               double            d3dTol=SM_EFF_ZERO)  // in :  
                                              const ;

  virtual SmStatus EvaluateGeometric(const SmPoint2d & crUV,                       // in : Target Surface Parameter                                                            
                                     SmBoolean         bUFromLeft,                 // in : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval  
                                     SmBoolean         bVFromLeft,                 // in : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval  
                                     double          & rdGaussianCurvature,        // out: GaussianCurvature = K1*K2 (principal curvatures)                                    
                                     double          & rdNormalCurvature,          // out: MeanCurvature = 1/2(K1 + K2)                                                        
                                     double          & rdPrincipleCurvature1,      // out: 1st principal curvature value, K1 = 1/RadiusOfCurvature1                            
                                     double          & rdPrincipleCurvature2,      // out: 2nd principal curvature value, K2 = 1/RadiusOfCurvature1                            
                                     SmVector3d      & rEFGOfFirstFundForm,        // out: 1st fundamental form, [E, F, G]                                                     
                                     SmVector3d      & rLMNOfSecondFundForm,       // out: 2nd fundamental form, [L, M, N]                                                     
                                     SmVector3d      & rPrincipleCurvatureVector1, // out: 3d vector tangent to surface in 1st principal direction                             
                                     SmVector3d      & rPrincipleCurvatureVector2) // out: 3d vector tangent to surface in 2nd principal direction                             
                                    const ;

  // evaluate directional derivatives with chain rule
  SmStatus EvaluateDirectionalDerivs(const SmPoint2d  & crUV,           // in : UV Point to evaluate                                                               
                                     SmVector2d       & rUVDir,         // in : direction for derivatives, gets unitized                                           
                                     ULONG              lHighestDeriv,  // in : 1=1st deriv, 2=1st and 2nd derivs, 3=1st, 2nd, and 3rd derivs                      
                                     SmVector3d         aDirDerivs[],   // out: aDirDerivs[0] = position                                                           
                                                                        //    : aDirDerivs[1] = 1st directional derivative                                         
                                                                        //    : aDirDerivs[2] = 2nd directional derivative                                         
                                                                        //    : aDirDerivs[3] = 3rd directional derivative                                         
                                                                        //    : sized:[lHighestDeriv+1]                                                            
                                     SmBoolean         bUFromLeft=TRUE, // in : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval 
                                     SmBoolean         bVFromLeft=TRUE) // in : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval 
                                    const ;

  // Evaluate directional 2nd derivatives with respect to two directions
  SmStatus Evaluate2ndDirectionalDerivatives(const SmPoint2d  & crUV,        // in : uv value to evaluate at               
                                             const SmVector3d & crDir1,      // in : 3d vector in first direction          
                                             const SmVector3d & crDir2,      // in : 3d vector in second direction         
                                                   SmVector3d *aDerivs)      // out: SmSurface::Evaluate output:           
                                                                             //    : 2d organized: Du=[1][0], Dv=[0][1],   
                                                                             //    :   Duu=[2][0], Duv=[1][1],Dvv=[0][2]   
                                                                             //    : 1d organized: lHighestDeriv == 2      
                                                                             //    :   Du=[3] , Dv=[1]                     
                                                                             //    :   Duu=[6], Duv=[4], Dvv=[2]           
                                            const ;

  virtual SmStatus EvaluateNormalSection(const SmPoint2d  & crUV,                   // in : Target Point                                                                        
                                         SmBoolean          bUFromLeft,             // in : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval  
                                         SmBoolean          bVFromLeft,             // in : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval  
                                         const SmVector3d & crDirection,            // in : Direction which projects to the tangent of a curve on                               
                                                                                    //    : the surface at which the evaluation is being done. Non-unit is OK.                  
                                         SmVector3d       & rTangentPlaneDirection, // out: Unit-Vector in direction of crDirection projected into tangent plane.               
                                         double           & rdCurvatureValue,       // out: Curvature value of the theoretical surface curve whose tangent                      
                                                                                    //    : direction is rTangentPlaneDirection.                                                
                                         SmVector3d       & rSurfaceNormalVector)   // out: unit-Normal vector to the surface at that point                                     
                                        const ;                                       

  virtual SmStatus EvaluatePoint(const SmPoint2d & crUV, SmPoint3d & rPoint) const 
                     { SM_REF2(crUV, rPoint) ; SE(SM_ERR) ; return SM_ERR; }

  virtual SmStatus EvaluateSTEP(const SmPoint2d & crUV,                    // in : target surface point                                                                      
                                ULONG             lHighestUDeriv,          // in : Requested highest U derivative                                                            
                                ULONG             lHighestVDeriv,          // in : Requested highest V derivative                                                            
                                SmBoolean         bUFromLeft,              // in : if P is on U interval boundary                                                            
                                                                           //    : TRUE  = evaluate P in upper interval where P is on the left of the interval               
                                                                           //    : FALSE = evaluate P in lower interval where P is on the right of the interval              
                                SmBoolean         bVFromLeft,              // in : if P is on V interval boundary                                                            
                                                                           //    : TRUE  = evaluate P in upper interval where P is on the left of the interval               
                                                                           //    : FALSE = evaluate P in lower interval where P is on the right of the interval              
                                SmBoolean         bOnlyUpperHalf,          // in : TRUE=compute upper half of matrix only                                                    
                                                                           //    : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value             
                                                                           //    :           [Dv --]       [Dv   Duv   ---]                                                  
                                                                           //    :                         [Dvv  ---   ---]                                                  
                                SmVector3d      * aDerivatives)            // out: matrix of evaluations values                                                              
                                                                           //    : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                                
                                                                           //    : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]                                        
                                                                           //    :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]                                        
                                                                           //    :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                        
                                                                           //    :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                        
                                                                           //    : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]  
                               const ;

  virtual SmStatus EvaluateSTEPPoint(const SmPoint2d & crUV, SmPoint3d & rPoint) const ;

  // At a singularity, find DegenParamValue whose NonDegenTangent lines up with a given direction.
  SmStatus FindDegenParamForDirection(SmPoint2d         & rUV,                // i/o: a point on the surface singularity                                       
                                      SmVector3d        & rDir,               // in : the direction to match with the NonSingular Surface tangent               
                                      SmBoolean           bDirFromPole,       // in : TRUE  = Dir to match is from the pole heading out, as in starting a curve 
                                                                              //    : FALSE = Dir to match ends at the pole heading in, as in ending a curve    
                                      SmSurfParamType     eSingularDirection, // in : oneof SM_SP_U: Surf(si,v) == Surf(sj,v) where si, sj are any valid u values
                                                                              //    :       SM_SP_V: Surf(u,si) == Surf(u,sj) where si, sj are any valid v values
                                      double              dTol)               // in : a loose tolerance; will try for a tight one.                              
                                     const ;

  virtual SmStatus FindFilletMinMaxRadii(SmSurfParamType eRailDir,            // in : see Usage Notes      
                                         SmExtent1d & sRadii,                 // out:                      
                                         double dTol = SM_EFF_ZERO)           // in : default:[SM_EFF_ZERO]
                                        const ;
 
  // get param values for IsoParamLines with specified discontinutities 
  virtual SmStatus FindKnotsByMaxContinuity(SmContinuityType   eMaxContinuity, // in : oneof:     SM_CT_DISCONTINUOUS = find srf pos discontinuities
                                                                               //                 SM_CT_CO = find C0 and if (!bAtContinuity) pos discs 
                                                                               //             ... SM_CT_C1 = find C1 and if (!bAtContinuity) C0, pos discs
                                                                               //             ... SM_CT_C2 = find C2 and if (!bAtContinuity) C1, C1, pos discs
                                                                               //             ... and all other SmContinuityType values                               
                                            SmBoolean          bAtContinuity,  // in : TRUE = only report knots with given continuity   
                                                                               //    : FALSE= report knots with given or less continuity
                                            SmTArray<double> & rUKnots,        // out: All U knots matching search criteria             
                                            SmTArray<double> & rVKnots)        // out: All V knots matching search criteria             
                                           const ;
  
  // Get singular classification for each UVNatural boundary at corners, see also GetSingularities() 
  //      note: - Singular = TolBetweenSmpPts < scaled IW_EFF_ZERO
  SmStatus FindSingularities(SmBoolean & rbUMin,   // out: TRUE = surface UMin Natural Bndry is a Pole
                             SmBoolean & rbUMax,   // out: TRUE = surface UMax Natural Bndry is a Pole
                             SmBoolean & rbVMin,   // out: TRUE = surface VMin Natural Bndry is a Pole
                             SmBoolean & rbVMax)   // out: TRUE = surface VMax Natural Bndry is a Pole
                            const ;

  static SmStatus FindUVCurveIvlInDomain(const SmBSplineCurve & crUVCurve,      // in : TgtCurve to be classified                                                    
                                         const SmExtent1d     & crInterval,     // in : TgtCurve Ivl of interest                                                     
                                         double                 dParamInside,   // in : TgtCurve Param that's inside or on crTrimDomain                              
                                         const SmExtent2d     & crTrimDomain,   // in : Find TgtCurve->crInterval portion containing dParamInside inside crTrimDomain.
                                         SmExtent1d           & rTrimIvl) ;     // out: Resulting interval inside crTrimDomain containing dParamInside               

  // find surface tessellation points by running TessellateByBisection() on isoParameter curves.
  SmStatus FindTessellationSplits(SmSurfParamType    eSurfParam,               // in : oneof: SM_SP_U, SM_SP_V                                                               
                                  const SmExtent2d & crUVDomain,               // in : surface domain                                                                        
                                  double             dChordHeightTolerance,    // in : max height/length allowed ratio, 0=ignore                                             
                                  double             dAngleTolDeg,             // in : max angle between tessellation pt tangents, 0=ignore                                  
                                  double             dMaximumSideLength,       // in : max length between tessellation pts, 0=ignore                                         
                                  SmTArray<double> & rSplits,                  // out: array of tessellation pts (including end-pts, so size >= 2 always)                    
                                  SmBoolean          bCheckDiagonals = FALSE)  // in : TRUE = when U and V dirs are linear tesselate diagonal isoparameter curves FALSE=don't
                                 const ;

  // Separate duplicate end Control points on non-degenerate ControlPolygon rows and cols
  virtual SmBoolean FixRepeatedEndControlPoints() { return FALSE ; }
   
  virtual SmObject         * GetOwner              ()                           const { return m_pOwner ; }
          SmFace           * GetFace               ()                           const ; // Srfs and Srf->Owners of Type==Srf can be owned by a Face
          SmBrep           * GetBrep               ()                           const ;
  virtual SmExtent2d         GetNaturalUVDomain    ()                           const { ERR(SM_ERR) ; return SmExtent2d(SmPoint2d(0,0),SmPoint2d(0,0)) ; }
  virtual SmExtent2d         GetSTEPUVDomain       ()                           const { return GetNaturalUVDomain() ; }
  virtual ULONG              GetDegree             (SmSurfParamType eSurfParam) const { SM_REF1(eSurfParam) ; ERR(SM_ERR) ; return SM_ERR ; }
  virtual ULONG              GetNumberControlPoints(SmSurfParamType eSurfParam) const { SM_REF1(eSurfParam) ; return 0 ; }
  virtual SmBSplineSurface * GetRootSurface        ()                           const { return NULL ; }
  virtual SmExtent2d         GetMaxAnalyticDomain  ()                           const { ERR(SM_ERR) ; SmExtent2d sRet;  return sRet; }

  virtual SmStatus     GetKnots(SmSurfParamType    eSurfParam,                     // in : SM_SP_U = get U Knot Vector                                  
                                                                                   //    : SM_SP_V = get V knot Vector                                  
                                SmTArray<double> & rKnots,                         // out: unique knot values in requested dimension                    
                                SmTArray<ULONG>  * pKnotMultiplicities = NULL,     // out: associated multiplicity for every knot                       
                                const SmExtent1d * pOptIvl = NULL)                 // in : interval of interest, NULL=Natural Interval, default:[NULL]  
                               const ;         

  // rtn: orof: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX for sides whose SamplePts are within IW_EFF_ZERO of being a pole
  //      note: - Singular       = TolBetweenSmpPts < scaled IW_EFF_ZERO.   
  //            - ApproxSingular = TolBetweenSmpPts < Approx3dTol. note: (ApproxSingular & ~Singular) = bit array of sides needing healing where (scaled_IW_EFF_ZERO < TolBetweenSmpPts < Approx3dTol)
  virtual ULONG        GetSingularities(SmTArray<SmPoint3d>  * pPolePoints =NULL,          // out: opt array[4] of [UMinPole, VMinPole, UMaxPole, VmaxPole], nonSingular points set to Uninit  
                                        SmTArray<SmVector3d> * pPoleNormals=NULL,          // out: opt array[4] of [UMinNrml, VMinNrml, UMaxNrml, VmaxNrml], nonSingular points set to UnInit  
                                        ULONG                * pbApproxSingularities=NULL) // out: sides whose SampleTps are within ApproxTol3d of being a pole.                                         
                                                                                           //    : SM_SS_NONE or an orof: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX                       
                                      const ; 
  // rtn: SM_FC_NONE or an orof: SM_FC_UMIN_VMIN, SM_FC_UMIN_VMAX, SM_FC_UMAX_VMIN, SM_FC_UMAX_VMAX
  virtual ULONG        GetFlatCorners(SmTArray<SmPoint3d>  * pFlatCornerPoints=NULL,     // out: opt array[4] of FlatCorner locs[UMinVMin, UMinVax, UMaxVMin, UMaxVmax]                      
                                                                                         //    : NonFlatCorner points set to Uninit                                                          
                                      double dAngleTolDeg=SM_EFF_ZERO_DEG)               // in : Parallel Vec Angle Tol in Degrees, default:[SM_EFF_ZERO_DEG]                                
                                     const ;  

  // get some subdomain containing the crUV point that is everywhere at least eMinContinuity continuous 
  virtual SmExtent2d  GetDomainWithContinuity(SmContinuityType  eMinContinuity, // in : min continuity in desired subDomain                                
                                              const SmPoint2d  & crUV,          // in : subDomain must contain this point                                  
                                              const SmVector2d & crUVVector)    // in : If crUV is on a discontinuity - specify desired side for subDomain 
                                             const ;                                                                    

  // Get lower limits on acceptable first derivatives and normal.
  static double GetDerivativeLimit(const SmPoint3d &rPos ) ;
  static double GetNormalLimit() ;

  virtual SmStatus GlobalCurveIntersect(const SmExtent2d & crUVDomain,          // in : Domain of the surface to intersect            
                                        const SmCurve    & crCurve,             // in : Curve to intersect with surface               
                                        const SmExtent1d & crInterval,          // in : Interval of curve to intersect                
                                        double             dDistanceTolerance,  // in : 3D distance tolerance to use in intersection  
                                        SmSolutionArray  & rSolutions)          // out: Contains results of intersection              
                                                                                //    : rSol.m_vStart[0] = Curve Param                
                                                                                //    : rSol.m_vStart[1] = Surface Param U            
                                                                                //    : rSol.m_vStart[2] = Surface Param V            
                                       const ;   
                                                                                
  virtual SmStatus GlobalCurveSolve    (const SmExtent2d       & crUVDomain,               // in : Domain of the surface to be used in solve                                          
                                        const SmCurve          & crCurve,                  // in : Curve to solve with                                                                
                                        const SmExtent1d       & crInterval,               // in : Curve interval to use in solve                                                     
                                        SmSolverOperationType    eSolverOperation,         // in :                                                                                    
                                        double                   dDistanceTolerance,       // in : 3D distance tolerance to be used for solve                                         
                                        const double           * cpdOptTargetDistance,     // in : The cpdOptTargetDistance, if not NULL, will be the corresponding limit             
                                                                                           //    : to a minimize/maximize operations.  In other words, it will ask the solver         
                                                                                           //    : to find a minimum value only if it is less than the target distance                
                                                                                           //    : or maximum value only if it is greater than the target distance.                   
                                        const SmVector3d       * cpOptVectors,             // in : See SmSolverOperationType documentation for corresponding meaning of these vectors.
                                        SmSolutionRequestedType  eSolutionRequested,       // in : What kind of a solution do you want                                                
                                        SmSolutionArray        & rSolutions)               // out: The output of the solve                                                            
                                       const ;
  
  virtual SmStatus GlobalLineIntersect (const SmExtent2d & crUVDomain,                     // in : Surface domain to intersect                              
                                        const SmPoint3d  & crLinePoint,                    // in : Point on the infinite line                               
                                        const SmVector3d & crLineVector,                   // in : Direction vector of the infinite line                    
                                        const SmExtent1d * cpOptLineInterval,              // in : If specified, bounds the line to a specific segment      
                                                                                           // in : If TRUE specifies that the line is bounded only at the   
                                        SmBoolean          bFireRay,                       //    : start point and proceeds along the vector to infinity.   
                                        double             dDistanceTolerance,             // in : min distance between distinct 3d points                  
                                        SmSolutionArray  & rSolutions)                     // out: Solution array                                           
                                       const ;

  virtual SmStatus GlobalPointSolve    (const SmExtent2d      & crUVDomain,                // in : Domain of surface to search for solutions                                                       
                                        SmSolverOperationType   eSolverOperation,          // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT                         
                                        const SmPoint3d       & crTestPoint,               // in : Target point for the solve operation                                                            
                                        SmZoneTol3d             sSrcZoneTol3d,             // in : Obj ZoneTol3d assoc with Target Point, if none, use: SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL)  
                                        const double          * cpdOptTargetDistance,      // in : Max/Min Drop distance for min/max and normalize operations. NULL to ignore.                     
                                        SmSolutionRequestedType eSolutionRequested,        // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                                     
                                        SmSolutionArray       & rSolutions)                // out: array of problem solutions reported as surface UV parameter values                              
                                       const ;

// for internal use only - temp internal access to old Surface GlobalPoint algorithm.
//   Set m_bUseNewPointSolvers to switch between solvers
  virtual SmStatus GlobalPointSolve_0  (const SmExtent2d      & crUVDomain,                 // in :     
                                        SmSolverOperationType   eSolverOperation,           // in :     
                                        const SmPoint3d       & crTestPoint,                // in :     
                                        double                  dDistanceTolerance,         // in :     
                                        const double          * cpdOptTargetDistance,       // in :     
                                        SmSolutionRequestedType eSolutionRequested,         // in :     
                                        SmSolutionArray       & rSolutions)                 // out:     
                                       const ;

// for internal use only - temp internal access to new Surface GlobalPoint algorithm.
//   Set m_bUseNewPointSolvers to switch between solvers
  virtual SmStatus GlobalPointSolve_1  (const SmExtent2d      & crUVDomain,                  // in :     
                                        SmSolverOperationType   eSolverOperation,            // in :     
                                        const SmPoint3d       & crTestPoint,                 // in :     
                                        double                  dDistanceTolerance,          // in :     
                                        const double          * cpdOptTargetDistance,        // in :     
                                        SmSolutionRequestedType eSolutionRequested,          // in :     
                                        SmSolutionArray       & rSolutions)                  // out:     
                                       const ;

  virtual SmStatus GlobalPointSolveSTEP(const SmExtent2d      & crSTEPUVDomain,              // in : STEP-based Domain of surface   
                                        SmSolverOperationType   eSolverOperation,            // in :                                
                                        const SmPoint3d       & crTestPoint,                 // in :                                
                                        double                  dDistanceTolerance,          // in :                                
                                        const double          * cpdOptTargetDistance,        // in :                                
                                        SmSolutionRequestedType eSolutionRequested,          // in :                                
                                        SmSolutionArray       & rSolutions) ;                // out: step domain parameters         

  virtual SmStatus GlobalSurfaceIntersect(const SmContext            & crContext,             // in : Context for new object construction                                                      
                                          const SmExtent2d           & crUVDomain,            // in : this Surface intersection limits                                                         
                                          const SmSurface            & crOtherSurface,        // in : target 2nd intersecting surface                                                          
                                          const SmExtent2d           & crOtherUVDomain,       // in : 2nd Surface intersection limits                                                          
                                          const SmBoolean              bUseSurfaceEdges[2],   // in : TRUE = Find intersection curve start points by intersecting                              
                                                                                              //    :        the edges of one surface with the other.                                          
                                                                                              //    : Normally both are TRUE unless you know                                                   
                                                                                              //    : that the edges of one surface do not intersect                                           
                                                                                              //    : the other surface.  It is a slight optimization                                          
                                                                                              //    : to set the flag to FALSE                                                                 
                                          const SmApproxTol3d        * pOptApproxTol3d,       // in : If not given it uses 1/1000 of surface size as approximation tolerance                   
                                          const double               * pdOptAngTolRad,        // in : If not given it uses 30 degrees                                                          
                                          SmTArray<SmCurve*>         * pOpt3DCurves,          // out: 3D curves produced by intersection                                                       
                                          SmTArray<SmCurve*>         * pOptSurface1UVCurves,  // out: UV curves on this surface produced by intersection                                       
                                          SmTArray<SmCurve*>         * pOptSurface2UVCurves,  // out: UV curves on crOtherSurface produced by intersection                                     
                                          SmTArray<SmTsectCurveType> * pOptCurveTypes,        // out: What type of curve is produced -                                                         
                                                                                              //    : oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)                
                                                                                              //    :        SM_TC_CROSSING   - curve intersection (surf norms not parallel)                   
                                                                                              //    :        SM_TC_TANGENT    - curve intersection (surf norms parallel)                       
                                                                                              //    :        SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal)
                                                                                              //    :        SM_TC_NEAR_TANGENT    - curve has small angle of intersection                     
                                                                                              //    :        SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident 
                                          SmTArray<double>           * pOptDeviations)        // out: produced Curve Deviations from each UVCurve to true xSect, NULL to ignore                
                                         const ;

#ifdef SM_VALIDATE_INTERSECTORS
  SmStatus GlobalSurfaceIntersectAndValidate(const SmContext            & crContext,                 // in : Context for new object construction                                                       
                                             const SmExtent2d           & crUVDomain,                // in : this Surface intersection limits                                                          
                                             const SmSurface            & crOtherSurface,            // in : target 2nd intersecting surface                                                           
                                             const SmExtent2d           & crOtherUVDomain,           // in : 2nd Surface intersection limits                                                           
                                             const SmBoolean              bUseSurfaceEdges[2],       // in : TRUE = Find intersection curve start points by intersecting                               
                                                                                                     //    :        the edges of one surface with the other.                                           
                                                                                                     //    : Normally both are TRUE unless you know                                                    
                                                                                                     //    : that the edges of one surface do not intersect                                            
                                                                                                     //    : the other surface.  It is a slight optimization                                           
                                                                                                     //    : to set the flag to FALSE                                                                  
                                             const SmApproxTol3d        * pOptApproxTol3d,           // in : If not given it uses 1/1000 of surface size approximation tolerance                       
                                                                                                     // in : If not given it uses 30 degrees                                                           
                                             const double               * pdOptAngTolRad,            // out: 3D curves produced by intersection                                                        
                                             SmTArray<SmCurve*>         * pOpt3DCurves,              // out: UV curves on this surface produced by intersection                                        
                                             SmTArray<SmCurve*>         * pOptSurface1UVCurves,      // out: UV curves on crOtherSurface produced by intersection                                      
                                             SmTArray<SmCurve*>         * pOptSurface2UVCurves,      // out: What type of curve is produced -                                                          
                                             SmTArray<SmTsectCurveType> * pOptCurveTypes,            //    : oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)                 
                                                                                                     //    :        SM_TC_CROSSING   - curve intersection (surf norms not parallel)                    
                                                                                                     //    :        SM_TC_TANGENT    - curve intersection (surf norms parallel)                        
                                                                                                     //    :        SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal) 
                                                                                                     //    :        SM_TC_NEAR_TANGENT    - curve has small angle of intersection                      
                                                                                                     //    :        SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident  
                                             SmTArray<double>           * pOptDeviations)            // out:                                                                                           
                                            const ;
#endif // SM_VALIDATE_INTERSECTORS

  virtual SmStatus GlobalSurfaceSolve(const SmExtent2d       & crUVDomain,                    // in :                                                                   
                                      const SmSurface        & crOtherSurface,                // in :                                                                   
                                      const SmExtent2d       & crOtherUVDomain,               // in :                                                                   
                                      SmSolverOperationType    eSolverOperation,              // in :                                                                   
                                      double                   dDistanceTolerance,            // in :                                                                   
                                      const double           * cpdOptTargetDistance,          // in : The cpdOptTargetDistance, if not NULL, will be the corresponding  
                                                                                              //    : limit to a minimize/maximize operations.  In otherwords, it will  
                                                                                              //    : ask the solver to find a minimum value only if it is less than    
                                                                                              //    : the target distance or maximum value only if it is greater than   
                                                                                              //    : the target distance.                                              
                                      const SmVector3d       * cpOptVectors,                  // in : See SmSolverOperationType documentation for corresponding meaning 
                                                                                              //    : of these vectors.                                                 
                                      SmSolutionRequestedType  eSolutionRequested,            // in :                                                                   
                                      SmSolutionArray        & rSolutions)                    // out:
                                     const ;

  SmStatus IntersectSurfaceBoundaries(const SmContext     & crContext,                        // in : context for new object construction                
                                      const SmExtent2d    & crUVDomain,                       // in : This surface intersection NurbDomain               
                                      const SmSurface     & crOtherCone,                      // in : other surf                                         
                                      const SmExtent2d    & crOtherUVDomain,                  // in : other surf intersection NurbDomain                 
                                      const SmApproxTol3d & crApproxTol3d,                    // in : intersection 3d tol or                             
                                      SmTArray<SmCurve*>  * pOpt3DCurves,                     // out: required array of output intersections             
                                      SmTArray<SmCurve*>  * pOptSurface1UVCurves,             // out: optional associated surface1 UVTrimCurves          
                                      SmTArray<SmCurve*>  * pOptSurface2UVCurves,             // out: optional associated surface2 UVTrimCurves          
                                      SmTArray<SmTsectCurveType> * pOptCurveTypes,            // out: optional assocaited intersection types             
                                      SmTArray<double>    * pOptDeviations)                   // out: optional associated deviations for each xSect Curve
                                     const ;

  virtual SmStatus InvertPointNearSingularity(const SmPoint3d               & crTestPt,       // in :  
                                              const SmExtent2d              & crUVDomain,     // in :  
                                                    SmPoint2d               & rGuessUV,       // in :  
                                                    double                    dTol,           // in :  
                                                    SmTerminationReasonType & reReason)       // out:  
                                             const ;

  virtual SmStatus LiftCurve(const SmContext      & crContext,                       // in : context for new object construction                          
                             const SmExtent2d     & crUVDomain,                      // in : this surface limit                                           
                             const SmBSplineCurve & crUVCurve,                       // in : 2d Parameter curve defined in surface parameter space to lift
                             const SmExtent1d     & crInterval,                      // in : curve segment to lift                                        
                             SmApproxTol3d          sApproxTol3d,                    // in : max allowed distance between output curve and Surface        
                             double               & rdMaxDistToSurf,                 // out: max dist from output curve to surface                        
                             SmBSplineCurve      *& rp3dCurve,                       // out: 3d Curve = Surface(crUVCurveToLift(crInterval))              
                             SmBoolean              bSkipContinuityChecks = FALSE)   // in : TRUE = this Surface is at least C1 continuous and            
                                                                                     //    :        internal surface C0 continuity checks are skipped.    
                                                                                     //    : FALSE= check Surface for C0 discontinuities -                
                                                                                     //    :        break lifted curve at each such point                 
                            const ;
                                            
  virtual SmStatus LocalCurveIntersect(const SmExtent2d & crUVDomain,             // in : surface sub-domain to check                                            
                                       const SmCurve    & crCurve,                // in : curve to xsect with surface                                            
                                       const SmExtent1d & crInterval,             // in : curve sub-domain to check                                              
                                       double             dDistanceTolerance,     // in : How close do curve and surface need to be to constitute an intersection
                                       const SmPoint2d  & crUVGuess,              // in : surface parameter guess                                                
                                       double             dTGuess,                // in : curve parameter guess                                                  
                                       SmBoolean        & rbFoundIntersection,    // out: TRUE=found an xsect, FALSE=didn't                                      
                                       SmPoint2d        & rUVFound,               // out: xsect surface param value                                              
                                       double           & rdTFound,               // out: xsect curve param value                                                
                                       double           & rdDeviation)            // out: distance between curve and surface xsect points                        
                                      const ;

  virtual SmStatus LocalCurveSolve(const SmExtent2d     & crUVDomain,               // in : Target Surface domain limit                                                        
                                   const SmCurve        & crCurve,                  // in : Target Curve                                                                       
                                   const SmExtent1d     & crInterval,               // in : Target Curve interval                                                              
                                   SmSolverOperationType  eSolverOperation,         // in : SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE or SM_SO_3D_SIGNED_DIRECTED_MINIMIZ
                                   double                 dDistanceTolerance,       // in :                                                                                    
                                   const double         * cpdOptTargetDistance,     // NotUsed: in :                                                                                    
                                   const SmVector3d     * cpOptVectors,             // in :                                                                                    
                                   const SmPoint2d      & crUVGuess,                // in : Surface Guess Point                                                                
                                   double                 dTGuess,                  // in : Curve Guess Point                                                                  
                                   SmBoolean            & rbFoundAnswer,            // out: TRUE = Found a solution                                                            
                                   SmSolution           & rSolution)                // out: the curve/surface solution                                                         
                                  const ;

  virtual SmStatus LocalLineIntersect(const SmExtent2d & crUVDomain,                   // in : surface domain: clamps to this.                                       
                                      const SmPoint3d  & crLinePoint,                  // in :                                                                       
                                      const SmPoint3d  & crLineVector,                 // in :                                                                       
                                      const SmExtent1d & crInterval,                   // in : line interval.  Not used in the main iteration.                       
                                      double             dDistanceTolerance,           // in : How close do curve and surface need to be to constitute an intersection
                                      double             dAccuracyTolerance,           // in : This is the accuracy we use to stop iterations                        
                                      const SmPoint2d  & crUVGuess,                    // in : should be a good guess.                                               
                                      SmBoolean        & rbFoundIntersection,          // out: If TRUE an intersection is found                                      
                                      SmPoint2d        & rUVFound,                     // out: on surface                                                            
                                      double           & rdTFound,                     // out: on line                                                               
                                      double           & rdDeviation,                  // out: distance between final line point and surface point.                  
                                      SmPoint3d        & rPoint,                       // out: intersection point                                                    
                                      SmVector3d       & rNormal)                      // out: Note: not unitized.                                                   
                                     const ;

  // find surface param point that solves desired operation locally
  // returns 1 best solution that may be bounded.
  virtual SmStatus LocalPointSolve(const SmExtent2d    & crUVDomain,                // in : Surface Domain limit                      
                                   SmSolverOperationType eSolverOperation,          // in : one of SM_SO_MINIMIZE,  |                 
                                                                                    //    :        SM_SO_MAXIMIZE,  | - Same Behavior 
                                                                                    //    :        SM_SO_NORMALIZE, |                 
                                                                                    //    :        SM_SO_INTERSECT.                   
                                   const SmPoint3d     & crTestPoint,               // in : Target Point                              
                                   const SmPoint2d     & crUVGuess,                 // in : UV Guess Point                            
                                   SmBoolean           & rbFoundAnswer,             // out: TRUE = Found a Point                      
                                   SmSolution          & rSolution)                 // out: Contains UV Found Point                   
                                  const ;

  virtual SmStatus LocalPointSolve_0(const SmExtent2d        & crUVDomain,            // in : Surface Domain limit                     
                                     SmSolverOperationType     eSolverOperation,      // in : one of SM_SO_MINIMIZE,  |                
                                                                                      //    :        SM_SO_MAXIMIZE,  | - Same Behavior
                                                                                      //    :        SM_SO_NORMALIZE, |                
                                                                                      //    :        SM_SO_INTERSECT.                  
                                     const SmPoint3d         & crTestPoint,           // in : Target Point                             
                                     const SmPoint2d         & crUVGuess,             // in : UV Guess Point                           
                                     SmBoolean               & rbFoundAnswer,         // out: TRUE = Found a Point                     
                                     SmSolution              & rSolution,             // out: Contains UV Found Point                  
                                     SmTerminationReasonType & eReason, 
                                     int                     & IterCount)
                                    const ;

  virtual SmStatus LocalPointSolve_1(const SmExtent2d        & crUVDomain,             // in : Surface Domain limit            
                                     SmSolverOperationType     eSolverOperation,       // in : one of SM_SO_MINIMIZE,          
                                                                                       //    :        SM_SO_MAXIMIZE,          
                                                                                       //    :        SM_SO_NORMALIZE,         
                                                                                       //    :        SM_SO_INTERSECT.         
                                     const SmPoint3d         & crTestPoint,            // in : Target Point                    
                                     const SmPoint2d         & crUVGuess,              // in : UV Guess Point                  
                                     SmBoolean               & rbFoundAnswer,          // out: TRUE = Found a Point            
                                     SmSolution              & rSolution,              // out: Contains UV Found Point         
                                     SmTerminationReasonType & eReason,                // out:                                 
                                     int                     & IterCount)              // out:                                 
                                    const ;

  // find the intersection point between a plane and 2 surfaces
  SmStatus LocalPlaneSurfaceIntersect(const SmExtent2d & crUVDomain,       // in : this surface domain of interest                                
                                      const SmSurface  & crOtherSurface,   // in : other surface                                                  
                                      const SmExtent2d & crOtherUVDomain,  // in : other surface domain of interest                               
                                      double             dXSectTol3d,      // in : Max allowed distance between found Surface intersection points 
                                      const SmPoint2d  & crUVGuess,        // in : this Surface initial UV guess                                  
                                      const SmPoint2d  & crOtherUVGuess,   // in : other Surface initial UV guess                                 
                                      const SmVector3d & crPlaneOrigin,    // in : origin of Plane(origin, normal)                                
                                      const SmVector3d & crPlaneNormal,    // in : normal of Plane(origin, normal)                                
                                      SmBoolean        & rbFoundAnswer,    // out: TRUE = found a solution                                        
                                      SmSolution       & rSolution)        // out: when rbFoundAnser==TRUE, the solution                          
                                     const ; 

  virtual SmStatus LocalSurfaceIntersect(const SmExtent2d & crUVDomain,          // in : ThisSurface intersection domain                                 
                                         const SmSurface  & crOtherSurface,      // in : OtherSurface                                                    
                                         const SmExtent2d & crOtherUVDomain,     // in : OtherSurface intersection domain                                
                                         double             dDistanceTolerance,  // in : min dist between distinct points                                
                                                                                 //    : Nearby Solutions larger than this will be rejected              
                                         const SmPoint2d  & crUVGuess,           // in : Best Guess for Surface0 solution                                
                                         const SmPoint2d  & crOtherUVGuess,      // in : Best Guess for Surface1 solution                                
                                         const SmVector3d * cpOptPlaneNormal,    // in : If specified, solution will lie on plane defined                
                                                                                 //    : Surf1Point/Surf2Point avg and this Normal vector.               
                                                                                 //    : If bDoBoundaryTesting == TRUE this plane may be moved           
                                                                                 //    : to a found boundary point.                                      
                                                                                 //    : If this vector is not specified, this method will try           
                                                                                 //    : to calculate its own planeNormal by crossing the surface normals.
                                         const SmPoint2d * cpOptPreviousUV,      // in : Previous UV if we are stepping.  We will use this               
                                                                                 //    : to prevent stepping back to same point during convergence.      
                                         SmBoolean bDoBoundaryTesting,           // in : TRUE = Look for Surface/SurfaceBoundary intersections           
                                         SmBoolean & rbFoundAnswer,              // out: TRUE = found a solution                                         
                                         SmSolution & rSolution)                 // out: The solution containing Surface/OtherSurface UVPoints           
                                        const ;
  
  virtual SmStatus LocalSurfaceSolve(const SmExtent2d    & crUVDomain,            // in : 1st surface intersection domain                       
                                     const SmSurface     & crOtherSurface,        // in : 2nd surface                                           
                                     const SmExtent2d    & crOtherUVDomain,       // in : 2nd surface intersection domain                       
                                     SmSolverOperationType eSolverOperation,      // in : Specify solution type                                 
                                     double                dDistanceTolerance,    // in : dDistanceTolerance                                    
                                     const double        * cpdOptTargetDistance,  // NotUsed: in :                                                       
                                     const SmVector3d    * cpOptVectors,          // in : SM_SO_3D_SIGNED_DIRECTED_MINIMIZE -                   
                                                                                  //    :       cpOptVectors[0] = direction to minimize         
                                                                                  //    :       solution = signed Surf1Point/Surf2Point gap size
                                                                                  //    : ALL OTHER CASES -                                     
                                     const SmPoint2d     & crUVGuess,             // in : 1st Surface GuessPoint                                
                                     const SmPoint2d     & crOtherUVGuess,        // in : 2nd Surface GuessPoint                                
                                     SmBoolean           & rbFoundAnswer,         // out: TRUE = found a solution                               
                                     SmSolution          & rSolution)             // out: the solution                                          
                                    const ;

  virtual SmStatus LocalSurfaceSurfaceIntersect(const SmExtent2d & crUVDomain1,              // in : surface domain limit for solution search                          
                                                const SmSurface  & crSurface2,               // in : 2nd surface                                                       
                                                const SmExtent2d & crUVDomain2,              // in : 2nd surface domain limit for solution search                      
                                                const SmSurface  & crSurface3,               // in : 3rd surface                                                       
                                                const SmExtent2d & crUVDomain3,              // in : 3rd surface domain limit for solution search                      
                                                const SmPoint2d  & crUVGuess1,               // in : surface start-pt                                                  
                                                const SmPoint2d  & crUVGuess2,               // in : 2nd surface start-pt                                              
                                                const SmPoint2d  & crUVGuess3,               // in : 3rd surface start-pt                                              
                                                SmBoolean & rbFoundAnswer,                   // out: 1=found 1 answer, 0=found no answers                              
                                                SmSolution & rSolution)                      // out: rSolution.m_vStart.m_dSolutionValue = max dist between surface pts
                                                                                             //    : rSolution.m_vStart = [u0, v0, u1, v1, u2, v2]                     
                                                                                             //    : where u0, v0 = Solution's Surface param pt                        
                                                                                             //    : where u1, v1 = Solution's 2nd Surface param pt                    
                                                                                             //    : where u2, v2 = Solution's 3rd Surface param pt                    
                                               const ;    

  virtual SmStatus LocalSurfaceSurfaceIntersect(const SmExtent2d & crUVDomain1,              // in : surface domain limit for solution search                                                
                                                const SmSurface  & crSurface2,               // in : 2nd surface                                                                             
                                                const SmExtent2d & crUVDomain2,              // in : 2nd surface domain limit for solution search                                            
                                                const SmSurface  & crSurface3,               // in : 3rd surface                                                                             
                                                const SmExtent2d & crUVDomain3,              // in : 3rd surface domain limit for solution search                                            
                                                const SmPoint3d  & crXYZGuess1,              // in : image-space guess point                                                                 
                                                const SmPoint3d  * optXYZGuess2,             //    : opt: 2nd guess point (used if 1st guess pt fails to project to surfaces), NULL to ignore
                                                SmBoolean & rbFoundAnswer,                   // out: 1=found 1 answer, 0=found no answers                                                    
                                                SmSolution & rSolution)                      // out: rSolution.m_vStart.m_dSolutionValue = max dist between surface pts                      
                                                                                             //    : rSolution.m_vStart = [u0, v0, u1, v1, u2, v2]                                           
                                                                                             //    : where u0, v0 = Solution's Surface param pt                                              
                                                                                             //    : where u1, v1 = Solution's 2nd Surface param pt                                          
                                                                                             //    : where u2, v2 = Solution's 3rd Surface param pt                                          
                                               const ;

  // Whole Surface Predicates                                                                                   
          SmBoolean IsAnalytic             () const ;
  virtual SmBoolean IsBilinear             () const { return FALSE; }

  virtual SmBoolean IsPlanar               (double             dTol3d =SM_EFF_ZERO,       // in : max allowed deviation from Plane      
                                            SmPoint3d        * pOptPlanePoint = NULL,     // out: Point on Plane                        
                                            SmVector3d       * pOptPlaneNormal = NULL,    // out: Unit Surface Normal                   
                                            double           * pOptActDeviation = NULL)   // out: Max ControlPoint/Plane distance seen  
                                           const ;

  virtual SmBoolean IsDegeneratePoint      (double             dTol3d = SM_ZONE_TOL_3D/10.0,    // in : min distance between unique points,                                        
                                                                                                //    : 0.0 = use dScaledZero = 1000 * SM_EFF_ZERO*(1+GetMaxDimension(PtOnSurface))
                                                                                                //    : default:[SM_ZONE_TOL_3D/10.0=1.0e-6]                                       
                                            const SmExtent2d * pOptUVDomain = NULL)             // in : domain to examine, NULL = use Natural Interval                             
                                           const ;

  virtual SmBoolean IsDegenerateCurve      (double             dAngleTolDeg =SM_EFF_ZERO_DEG,      // in : min angle between parallel vectors, default:[SM_EFF_ZERO_DEG]             
                                            const SmExtent2d * pOptUVDomain = NULL,                // in : domain to examine, NULL = use Natural Domain, default:[NULL]              
                                            double             dOpt3DTol = SM_ZONE_TOL_3D / 10.0)  // in : Min distance between distinct points, default:[SM_ZONE_TOL_3D/10.0=1.0e-6]
                                           const ;

  virtual SmBoolean IsBounded ()           const   { return( GetSTEPUVDomain().IsBounded() ) ; }       // TRUE=finite (FALSE=infinite) parameter range

  virtual SmBoolean IsPeriodic             (const SmExtent2d & crUVDomain,                    
                                            SmSurfParamType    eSurfParam)                        // in : oneof SM_SP_U = [check PosTan[Umin,v] == PosTan[Umax,v] for v smpls
                                                                                                  //    :       SM_SP_V = [check PosTan[u,Vmin] == PosTan[u,Vmax] for u smpls
                                           const ; 
                                                                                                                                     
  virtual SmBoolean IsClosed               (const SmExtent2d & crUVDomain,                   // in :                                                                   
                                            SmSurfParamType    eSurfParam,                   // in : oneof SM_SP_U = [check Pos[Umin,v] == Pos[Umax,v] for v samples   
                                                                                             //    :   SM_SP_V = [check Pos[u,Vmin] == Pos[u,Vmax] for u samples       
                                                                                             //    :   SM_SP_BOTH                                                      
                                            double           * pdOptTolerance =NULL,         // in : min distance between distinct points                              
                                            SmContinuityType * peOptContinuity=NULL)         // out: oneof SM_CT_DISCONTINUOUS                                         
                                                                                             //    :   SM_CT_C0      SM_CT_C1                                          
                                                                                             //    :   SM_CT_G1      SM_CT_C1_G2                                       
                                                                                             //    :   SM_CT_G1_G2   SM_CT_C1_C2                                       
                                                                                             //    : NULL to ignore and only check for C0 continuity, default:[NULL]   
                                           const ;

  virtual SmBoolean IsParallelToVector (const SmVector3d & crVector) const 
                                       { SM_REF1(crVector) ; return FALSE ; }
                                                                                     
  // Surface Point Predicates                                                        
  SmBoolean IsPlanarPoint (const SmPoint2d  & crUV) const ; // rtns FALSE when crUV is outside of surface domain

  virtual SmBoolean IsSingularity          (const SmPoint2d  & crUVToTest,                   // in : UVpoint to test                                                                      
                                            SmSurfParamType  & reSingularDirection,          // out: SM_SP_U:[Su=0(U varies, V const)] or SM_SP_V:[Sv=0(U const, V Varies)] or SM_SP_BOTH 
                                            double             dTol3d =SM_EFF_ZERO,          // in : min dist between distinct 3d points                                                  
                                            SmBoolean          bPtTestOnly=FALSE)            // in : TRUE = Pt tests only                                                                 
                                                                                             //    : FALSE= Pt and Surf tests                                                             
                                                                                             //    : default:[FALSE] typical behavior except for some debug cases                         
                                           const ;

  virtual SmBoolean IsSingularity(const SmPoint2d           & crUVToTest,          // in : UVpoint to test                                                                      
                                  ULONG                       lSingularities,      // in : SM_SS_NONE or one of: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX, SM_SS_UNKNOWN
                                  const SmTArray<SmPoint3d> * sSrfPolePoints,      // in : PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                                                                   //    : NonSingular side values set to SmPoint3d::SetUninitialized(),
                                  SmSurfParamType           & reSingularDirection, // out: SM_SP_U:[Su=0(U varies, V const)] or SM_SP_V:[Sv=0(U const, V Varies)] or SM_SP_BOTH 
                                  double                      dTol3d =SM_EFF_ZERO, // in : min dist between distinct 3d points                                                  
                                  SmBoolean                   bPtTestOnly=FALSE)   // in : TRUE = Pt tests only                                                                 
                                                                                   //    : FALSE= Pt and Surf tests                                                             
                                                                                   //    : default:[FALSE] typical behavior except for some debug cases                         
                                 const ;
                                                                                             
  // rtn: TRUE = Su parallel to Sv at crUV is degenerate
  SmBoolean IsFlatCorner  (const SmPoint2d & crUV,                       
                           double dAngleTolDeg=SM_EFF_ZERO_DEG)
                          const ;

  // check UVPoint is one Surface Natural Boundary
  SmBoolean IsOnBoundary  (const SmExtent2d & crUVDomain,                    // in : domain to check                                                   
                           const SmPoint2d  & crUVPoint,                     // in : Surface UVPoint to test                                           
                           SmSurfParamType  & rParamSide,                    // out: SM_SP_U point is on uMin or uMax const U boundary                 
                                                                             //    : SM_SP_V point is on vMin or vMax const V boundary                 
                                                                             //    : SM_SP_BOTH point is on a corner                                   
                                                                             //    : SM_SP_NEITHER point is not on boundary                            
                           SM_NEWTOL_LINE const SmTol3d * pOptTol3d = NULL,  // in : max 3d deviation allowed, [NULL = SmTol::GetZoneTol3d(this))]     
                           double           * pOptOtherU=NULL,               // out: When OnBoundary, Other End Value, NULL to ignore, default:[NULL]  
                           double           * pOptOtherV=NULL)               // out: When OnBoundary, Other End Value, NULL to ignore, default:[NULL]  
                          const ;                                    
                                                                                      
  // check UVCurve is on Surface Natural Boundary with sampling               
  SmBoolean IsOnUVBoundary(const SmExtent2d & crUVDomain,                    // in : domain to check                                               
                           const SmCurve    & crUVTrimCurve,                 // in : uv curve to test                                              
                           ULONG              nSamplePts,                    // in : recommended = 5, if 0 and Curve is BSpline, use GlobalSolve   
                           SM_NEWTOL_LINE SmTol3d * pOptTol3d=NULL)          // in : max 3d deviation allowed, [NULL = SmTol::GetZoneTol3d(this))] 
                          const ;                                    

  // check UVCurve is on Surface Natural Boundary with global intersection solve (not yet implemented - pass call to sampling IsOnUVBoundary)
  SmBoolean IsOnUVBoundary(const SmExtent2d & crUVDomain,                    // in : domain to check                                                
                           const SmCurve    & crUVTrimCurve,                 // in : uv curve to test                                               
                           SM_NEWTOL_LINE SmTol3d * pOptTol3d = NULL)        // in : max 3d deviation allowed, [NULL = SmTol::GetZoneTol3d(this))]  
                          const ;
                          
  // check UV Point is both on Boundary and Boundary is a Seam, Tol3d val is projected to Tol2d in each seam direction
  virtual SmBoolean IsOnSeam(const SmPoint2d     & crUVPoint,             // in : UV Point to check                                                          
                             const SmTol3d       * pOptTol3d=NULL,        // in : max 3d deviation allowed, [NULL = SmTol::GetZoneTol3d(this))]              
                             SmSurfParamType     * eOptSeamDir=NULL,      // out: When IsOnSeam, Dir of Seam picked by crUVPoint (Not a Surf classification),
                                                                          //    : oneof SM_SP_U, SM_SP_V, SM_SP_BOTH, SM_SP_NEITHER, NULL to ignore          
                             SmTArray<SmPoint2d> * pOptCrossSeamUVs=NULL) // out: when IsOnSeam rtns TRUE, list of UVPts that map to same 3dPt,              
                                                                          //    : When SeamDir == SM_SP_U or SM_SP_V, there will be one pt on this list      
                                                                          //    : When SeamDir == SM_SP_BOTH, there will be three pts on this list,          
                                                                          //    : NULL to ignore, default:[NULL]                                             
                            const ;             

  SmBoolean IsInDomain(const SmExtent2d & crUVDomain,            // in : Tgt Domain
                       const SmPoint2d  & crUVToTest,            // in : Tgt UV Point to test                                           
                       SM_NEWTOL_LINE SmTol3d * pOptTol3d=NULL)  // in : max 3d deviation allowed, [NULL = SmTol::GetZoneTol3d(this))]  
                      const ;


  // Surface Problem Predicates (some are BSplineSurface specific)
  virtual SmBoolean HasInternalPole                     (double dTol=SM_ZONE_TOL_3D)  const { SM_REF1(dTol) ; return FALSE ; }
  virtual SmBoolean HasKnotMultiplicityGreaterThanDegree()                            const { return FALSE ; } 
  virtual SmBoolean HasRepeatedEndControlPoints         (double dTol=SM_ZONE_TOL_3D)  const { SM_REF1(dTol) ; return FALSE ; } 
  virtual SmBoolean HasDegenerateControlPointRow        (double dTol=SM_ZONE_TOL_3D)  const { SM_REF1(dTol) ; return FALSE ; }
          SmBoolean HasPlaneCrossings                   (SmTArray<SmPlane *> sPlanes) const ; 
          SmBoolean HasFlatCorners                      (double dAngleTolDeg=SM_EFF_ZERO_DEG) const ;

  virtual void Notify(SmNotifyOperation eNotifyOperation, // in : 
                      SmObject * pData1,                  // in : 
                      SmObject * pData2,                  // in : 
                      SmObject * pData3) ;                // in : 

  virtual SmStatus IntersectWithPlane(const SmContext     & crContext,                // in : context for new object construction                                                   
                                      const SmExtent2d    & crUVDomain,               // in : intersection limit for this surface                                                   
                                      const SmPlane       & crOtherPlane,             // in : target intersection plane                                                             
                                      const SmExtent2d    & crOtherUVDomain,          // in : intersection limit for target plane                                                   
                                      const SmBoolean       bUseSurfaceEdges[2],      // in : TRUE = find xsect curve start points from boundaryCurve/surface xsects                
                                                                                      //    : typically these values are TRUE - its a small savings if you know                     
                                                                                      //    : the boundaries of one surface don't intersect the other surface                       
                                      const SmApproxTol3d * pApproxTol3d,             // in :                                                                                       
                                      const double        * pdOptAngTolRad,           // in :                                                                                       
                                      SmBoolean           & rbNeedsMoreIntersections, // out: TRUE = special case intersection failed-use general intersection                      
                                      SmTArray<SmCurve*>  * pOpt3DCurves,             // out: Intersection 3DCurves, NULL to ignore                                                 
                                      SmTArray<SmCurve*>  * pOptSurface1UVCurves,     // out: associated UVTrimCurves on this surface, NULL to ignore                               
                                      SmTArray<SmCurve*>  * pOptSurface2UVCurves,     // out: associated UVTrimCurves on plane, NULL to ignore                                      
                                      SmTArray<SmTsectCurveType> * pOptCurveTypes,    // out: oneof for each 3DCurve, NULL to ignore                                                
                                                                                      //    : SM_TC_TOUCHING        - single point intersection (surf norms parallel)               
                                                                                      //    : SM_TC_CROSSING        - curve intersection (surf norms not parallel)                  
                                                                                      //    : SM_TC_TANGENT         - curve intersection (surf norms parallel)                      
                                                                                      //    : SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)
                                                                                      //    : SM_TC_NEAR_TANGENT    - curve has small angle of intersection                         
                                                                                      //    : SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident     
                                      SmTArray<double>    * pOptDeviations)           // out: associated max 3DCurve to surface distance, NULL to ignore                            
                                     const
                                     {
                                       SM_REF7( crContext, crUVDomain, crOtherPlane, crOtherUVDomain, bUseSurfaceEdges, pApproxTol3d, pdOptAngTolRad ) ;
                                       SM_REF6( rbNeedsMoreIntersections, pOpt3DCurves, pOptSurface1UVCurves, pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations ) ;
                                       return SM_ERR;
                                     }


  // currently a stub function                      
  virtual SmBoolean PassesValidityCheck(SmValidityCheckType eChecks,           // NotUsed: in :
                                        SmValidityCheckType & reCheckFailed)   // NotUsed: out :     
                                       const ;

  // Version that takes 3d points:
  SmStatus PointBasedSurfaceIntersect(const SmContext     & crContext,                  // in : for curve creation                         
                                      const SmExtent2d    & crUVDomain,                 // in : this surface's domain of interest          
                                      const SmSurface     & crOtherSurface,             // in : other surface to intersect with            
                                      const SmExtent2d    & crOtherUVDomain,            // in : other surface's domain of intersext        
                                      const SmTArray<SmPoint3d> & crSurfacePoints,      // in : ordered 3d points                          
                                      SmBoolean             bExtendBeforeStart,         // in : if more than two points given              
                                      SmBoolean             bExtendAfterEnd,            // in : if more than two points given              
                                      const SmVector3d    * pOptStartDirection,         // in :                                            
                                      const SmVector3d    * pOptEndDirection,           // in :                                            
                                      const SmApproxTol3d * pdOptApproxTol3d,           // in : default 1/1000 of surface size             
                                      const double        * pdOptAngTolRad,             // in : default 30 degrees                         
                                      SmBSplineCurve     *& rp3DCurve,                  // out:                                            
                                      SmBSplineCurve     *& rpSurface1UVCurve,          // out:                                            
                                      SmBSplineCurve     *& rpSurface2UVCurve,          // out:                                            
                                      SmTsectCurveType    & reCurveType,                // out:                                            
                                      double              & rdDeviation) ;              // out: max distance between 3d curve and surfaces 

  // Version that takes 2d uv parameter points:
  SmStatus PointBasedSurfaceIntersect(const SmContext           & crContext,             // in : Context for curve construction                                  
                                      const SmExtent2d          & crUVDomain,            // in : surf1 intersection domain limit                                 
                                      const SmSurface           & crOtherSurface,        // in : surf2                                                           
                                      const SmExtent2d          & crOtherUVDomain,       // in : surf2 intersection domain limit                                 
                                      const SmTArray<SmPoint2d> & crSurface1Points,      // in : Surface1 UV points on the intersection                          
                                                                                         //    : Current limit is 20 points.                                     
                                      const SmTArray<SmPoint2d> & crSurface2Points,      // in : Surface2 UV points on the intersection                          
                                                                                         //    : corresponding to crSurface1Points                               
                                      SmBoolean                   bExtendBeforeStart,    // in : If more than two points                                         
                                                                                         //    : are specified this will allow extension of the                  
                                                                                         //    : intersection curve prior to the start point                     
                                      SmBoolean                   bExtendAfterEnd,       // in : If more than two points                                         
                                                                                         //    : are specified this will allow the extension of the              
                                                                                         //    : intersection curve after the last point.                        
                                      const SmVector3d          * pOptStartDirection,    // in : expected direction of xsect curve leaving the 1st given point   
                                      const SmVector3d          * pOptEndDirection,      // in : end direction                                                   
                                      const SmApproxTol3d       * pdOptApproxTol3d,      // in : If not given it uses 1/1000 of surface size                     
                                                                                         //    : approximation tolerance                                         
                                      const double              * pdOptAngTolRad,        // in : If not given it uses 30 degrees                                 
                                      SmBSplineCurve           *& rp3DCurve,             // out: Resulting curve - note that this will be NULL                   
                                                                                         //    : if we are unable to find a curve that satisfies the             
                                                                                         //    : input requirements and passes through all given points.         
                                      SmBSplineCurve           *& rpSurface1UVCurve,     // out: Surf1 UVTrimCurve                                               
                                      SmBSplineCurve           *& rpSurface2UVCurve,     // out: Surf2 UVTrimCurve                                               
                                      SmTsectCurveType          & reCurveType,           // out: type of intersection on intersection curve                      
                                      double                    & rdDeviation) ;         // out: max distance between rp3DCurve and surfaces                     
                                             
  virtual SmStatus Reverse(SmSurfParamType eSurfParam)                 
                          { SM_REF1(eSurfParam) ; SE(SM_ERR) ; return SM_ERR; }

  virtual SmStatus Reparameterize(const SmExtent2d & crNewDomain)      
                                 { SM_REF1(crNewDomain) ; SE(SM_ERR) ; return SM_ERR; }

  SmObject *SetOwner(SmObject *pNewOwner) ; 
  
  virtual SmStatus SimpleCoincidenceChecker(const SmExtent2d & crUVDomain,                      // in : Surface domain of interest                                      
                                            const SmCurve    & crCurve,                         // in : target curve to check                                           
                                            const SmExtent1d & crInterval,                      // in : curve interval of interest                                      
                                            double             d3dTolerance,                    // in : max allowed curve/surface deviation for coincidence             
                                            SmBoolean        & rbFoundCoincidence,              // out: TRUE = curve is coincident with surface                         
                                                                                                //    : FALSE= Otherwise                                                
                                            SmSolution       & rSolution,                       // out: Solution: a Range solution.  Contains the                       
                                                                                                //    : farthest-apart coincident points found.  If                     
                                                                                                //    : PartialCoincidence is NOT ok, then this will be                 
                                                                                                //    : the whole curve (if coincident) or nothing (if                  
                                                                                                //    : not).  But if PartialCoinc is ok, then all we                   
                                                                                                //    : know for sure is that both Start and End are                    
                                                                                                //    : coincident, and it's probably coincident                        
                                                                                                //    : in between.                                                     
                                            SmBoolean          bPartialCoincidenceOK = FALSE)   // in : TRUE = find cases where at least part of the curve is coincident
                                                                                                //    : FALSE = find only cases where all sample points are coincident  
                                                                                                //    : default:[FALSE] = previous behavior                             
                                           const ;

  SmStatus SnapToKnots(const SmPoint2d & crUVOriginal,     // in :       
                       double            dUVSnapTolerance, // in :       
                       SmPoint2d       & rUVSnapped)       // out:       
                      const ;

  virtual SmStatus SplitAt(const SmContext & crContext,        // in : context for new object construction           
                           double            dParam,           // in : split parameter                               
                           SmSurfParamType   eSurfParam,       // in : oneof SM_SP_U = split u domain at dParam      
                                                               //    :       SM_SP_V = split v domain at dParam      
                           SmSurface      *& rpLeftSurface,    // out: Split surface result, Ivl=[MinParam, TgtParam]
                           SmSurface      *& rpRightSurface)   // out: Split surface result, Ivl=[TgtParam, MaxParam]
                          {
                            SM_REF5( crContext, dParam, eSurfParam, rpLeftSurface, rpRightSurface ) ;
                            SE( SM_ERR ) ; return SM_ERR;
                          }

  // given a point on the surface find its corresponding analytic UV-parameter
  virtual SmStatus STEPInversion(const SmExtent2d & crAnalUVDomain,                    // in :    
                                 const SmPoint3d  & crPointOnSurf,                     // in :    
                                 double             dDistanceTolerance,                // in :    
                                 SmPoint2d        & rdAnalUVParameter,                 // in :    
                                 SmLocationType   & reLocation,                        // in :    
                                 SmPoint2d        * pUVGuess = NULL)                   // in :    
                                const
                                {
                                  SM_REF6( crAnalUVDomain, crPointOnSurf, dDistanceTolerance, rdAnalUVParameter, reLocation, pUVGuess ) ;
                                  return SM_ERR;
                                }

  virtual SmStatus SwapUV()                                            
                         { SE(SM_ERR) ; return SM_ERR; }

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL)   // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling
                            { SM_REF2(crRotateNMove, cpOptScale) ; SE(SM_ERR) ; return SM_ERR; }

  virtual SmStatus TrimWithDomain(SmExtent2d & crTrimUVDomain)         
                                 { SM_REF1(crTrimUVDomain) ; SE(SM_ERR) ; return SM_ERR; }
  

  // See if intersecting coincident range is really a tangent intersection point 
  SmStatus ClassifyIntersectingRangeSolution(const SmCurve    & crCurve,        // in : Curve being intersected                            
                                             const SmSolution & crSol,          // in : A ThisSurface/OtherCurve intersection solution     
                                             double             dTol3D,         // NotUsed: in : intersection 3d tolerance                          
                                             SmBoolean        & bIsDegenerate,  // out: TRUE = Intersection is a Tangent Point Intersection
                                                                                //    : FALSE= Intersection is a coincident segment        
                                             SmSolutionEnd    & rSolPoint)      // out: set to a Point Tangent Solution when               
                                                                                //    : bIsDegenerate == TRUE                              
                                                                                //    : else left unmodified.                              
                                            const ;
                                           
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmSurface,SmAObject,SmSurface_TYPE) ;

  // write surface to file
  SmStatus WriteToFile(const TCHAR * cOutputFileName,          // in :                                        
                       SmBoolean     bSkipHeaderWrite=FALSE,   // NotUsed: in :                                        
                       SmBoolean     bNewFile=FALSE,           // in :                                        
                       SmBoolean     bWriteAttributes=FALSE)   // in : TRUE=Write attributes, FALSE=don't     
                      const ;   
  
  // write array of surfaces to file                                          
  static SmStatus WriteArrayToFile(const TCHAR                 * cOutputFileName,          // in : target file name                          
                                   const SmTArray<SmSurface *> & rSrfArr,                  // in : array of surfaces to write                
                                   SmBoolean                     bWriteAttributes=FALSE) ; // in : TRUE=Write attributes, FALSE=don't        

  // read a surface from file
  static SmStatus ReadFromFile(const SmContext & crContext,                // in : context for new object construction                  
                               const TCHAR     * cInputFileName,           // in : target file                                          
                               SmSurface      *& rpNewSurface,             // out: the read surface with newly allocated memory         
                               const ULONG       lFileOffsetInBytes = 0) ; // NotUsed: in : Number of characters in file to skip before reading) 

  // read array of surfaces from file
  static SmStatus ReadArrayFromFile(const SmContext       & crContext,                // in : context for new object construction                                        
                                    const TCHAR           * cInputFileName,           // in : target file                                                                
                                    SmTArray<SmSurface *> & rNewSurfaces,             // out: the read surface with newly allocated memory                               
                                    const ULONG             lFileOffsetInBytes = 0,   // NotUsed: in : Number of characters in file to skip before reading)                       
                                    SmTArray<SmSurface *> * pTestSurfaces = NULL) ;   // in : for debug only - the array of curves expected to be read  NULL to ignore   

  // I/O assist methods
  virtual SmStatus WriteToDB(SmDatabaseIO & rDB,                     // in : target output stream                                
                              ULONG          lDBVersionNumber)        // in : database version to get proper sequence of writes   
                             const                                                                                                                             
                            { SM_REF2( &rDB, lDBVersionNumber ) ; return SM_ERR ; }
                           
                            static  SmStatus ReadFromDB(SM_TYPE           lType,              // in : Object type to be read                                                        
                              SmDatabaseIO    & rDB,                // in : target output stream                                                          
                              const SmContext & crContext,          // in : context for new object construction                                           
                              SmSurface      *& rpNewSurface,       // out: NULL on input = new object allocated in this routine built from stream data   
                                                                    //    : NotNULL on input = pointer to an empty object to be filled by this routine    
                              ULONG             lDBVersionNumber) ; // in : database version to get proper sequence of writes                             

  // add various isoParameterCurves and surfaceNormal Vector to open displayList
  virtual SmStatus OutputGraphics(const SmDisplayParameters & crDisp,               // in : display control parameters                                                                     
                                  void                      * pGraphicNurb,         // NotUsed: in : not used?                                                                                      
                                  const SmExtent2d          * pOptUVDomain = NULL,  // in : UVDomain to crossHatch                                                                         
                                                                                    //    : NULL=NaturalUVDomain                                                                           
                                  SmGfxArraySet             * pOptGfxSet = NULL)    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore 
                                 const ;
                                
  // get memory used for surface but not its attributes
  virtual ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,      // out: bigger size of all allocated memory in bytes   
                              SmMarkType eMarkType=SM_MT_NOMARK) // in : uses without increment eMarkType value         
                             const 
                             { SM_REF2(rlMemoryAllocated, eMarkType) ; return sizeof(*this) ; }

  // add surface graphics for display Parameters to new or open displayList
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                         
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                     
                                                                          //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                 
                                SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't  
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order               
                               const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
  
  virtual void Dump(SmBoolean bAbbrev) const ;
  virtual void Dump(ULONG) const ;

  // Draw Methods
  virtual SmDisplayList * Draw(SmBoolean       bAddToUIPickList=FALSE,  // in : TRUE=Add to UI pick list, FALSE=don't       
                               SmGfxArraySet * pOptGfxSet=NULL)         // in : When given output GfxVertexArrays not GL calls.  
                              const ;  

  SmDisplayList * DrawSTEP(SmGfxArraySet * pOptGfxSet=NULL) const ;

  // add surfacePoint and optional derivative vectors to new or open displayList
  SmDisplayList * DrawAt(const SmPoint2d & sUV,                         // in : target param                                           
                         ULONG             lNumDeriv=0,                 // in : 0 = pos, 1 = pos + 1stDerivs, 2 = pos + 2ndDerivs      
                         SmGfxArraySet   * pOptGfxSet=NULL)             // in : When given output GfxVertexArrays not GL calls.        
                        const ;     
                                                                        
  // add surface isoparameter curve and optional cross-derivatives to new or open displayList
  SmDisplayList * DrawAlong(SmSurfParamType eConstantParamType,         // in : SM_SP_U or SM_SP_V to Draw constant U or V IsoParamCurve                           
                            double          dConstantParam,             // in : constant param value                                                       
                            ULONG           lNumDeriv=0,                // in : 0 = draw isoCurve                                                          
                                                                        //    : 1 = draw isoCurve and 1st cross-derivative vectors (dW/du or dW/dv)        
                                                                        //    : 2 = draw isoCurve and 2nd cross-derivative vectors (d2W/duu or d2W/dvv).   
                                                                        //    : 3 = draw isoCurve and 3rd cross-derivative vectors (d3W/duuu or d3W/dvvv). 
                            SmGfxArraySet * pOptGfxSet=NULL)            // i/o: When given output GfxVertexArrays not GL calls.                         
                           const ;        

  // add surfacePoint and optional Normals and 1st Derivatives vectors to new or open displayList
  SmDisplayList * DrawVectorField(const SmExtent2d   crUVDomain,                      // in : surface domain                                                      
                                  ULONG              lNumUPoints,                     // in : number of U points                                                  
                                  ULONG              lNumVPoints,                     // in : number of V point                                                   
                                  SmPinCushionType   eType,                           // in : oneof: SM_DM_POINTS                                                 
                                                                                      //    :        SM_DM_U_NATURAL                                              
                                                                                      //    :        SM_DM_V_NATURAL                                              
                                                                                      //    :        SM_DM_UV_NATURAL                                             
                                                                                      //    :        SM_DM_UNIT_NORMAL                                            
                                                                                      //    :        SM_DM_U_SCALED                                               
                                                                                      //    :        SM_DM_V_SCALED                                               
                                                                                      //    :        SM_DM_UV_SCALED                                              
                                                                                      //    :        SM_DM_SCALED_NORMAL                                          
                                  SmGfxArraySet    * pOptGfxSet=NULL)                 // i/o: When given output GfxVertexArrays not GL calls.
                                 const ;          

  // add UVTrimCurve drawn on surface graphics to new or open displayList
  SmDisplayList * DrawUVCurve(const SmCurve & rUVTrimCurve,             // in : UVTrimCurve to draw on surface                                                 
                              SmBoolean       bAddNormals=FALSE,        // in : TRUE = draw Normal comb, FALSE = don't                                                                                                 
                              double          dNormalScale=1.0,         // in : NormalComb scale factor, 1.0 = Draw Unit Normal Vectors                                                                                   
                              SmBoolean       bDrawUVPlane=FALSE,       // in : TRUE = Display UVTrimCurves on a UVplane icon near the surface, FALSE = don't                                                          
                              SmGfxArraySet * pOptGfxSet=NULL)          // i/o: When given output GfxVertexArrays not GL calls. NULL to ignore              
                             const ;                    
                                                                                                                                                                      
  // add UVPolyLine (3d - ignore z coords) drawn on surface graphics to new or open displayList
  SmDisplayList * DrawUVPolyline(const SmTArray<SmPoint3d> & rUVPnts,                // in : UVPolyline to draw on surface (ignores z coord values)                      
                                 SmBoolean                   bAddNormals=FALSE,      // in : TRUE = draw Normal comb, FALSE = don't                                                                                                   
                                 double                      dNormalScale=1.0,       // in : NormalComb scale factor, 1.0 = Draw Unit Normal Vectors                                                                                 
                                 SmBoolean                   bDrawUVPlane=FALSE,     // in : TRUE = Display UVTrimCurves on a UVplane icon near the surface, FALSE = don't                                                              
                                 SmGfxArraySet             * pOptGfxSet=NULL)        // i/o: When given output GfxVertexArrays not GL calls.                          
                                const ;       

  // add UVPolyLine (2d) drawn on surface graphics to new or open displayList
  SmDisplayList * DrawUVPolyline(const SmTArray<SmPoint2d> & rUVPnts,                // in : UVPolyline to draw on surface (ignores z coord values)                       
                                 SmBoolean                   bAddNormals=FALSE,      // in : TRUE = draw Normal comb, FALSE = don't                                                                                                   
                                 double                      dNormalScale=1.0,       // in : NormalComb scale factor, 1.0 = Draw Unit Normal Vectors                                                                                    
                                 SmBoolean                   bDrawUVPlane=FALSE,     // in : TRUE = Display UVTrimCurves on a UVplane icon near the surface, FALSE = don't                                                            
                                 SmGfxArraySet             * pOptGfxSet=NULL)        // i/o: When given output GfxVertexArrays not GL calls.                           
                                const ;       
                               
  // Draw and Return a UVPlane upon which UVCurves can be drawn as a UnitSquare icon.  Optionally, draw 3d Bound
  SmDisplayList * DrawUVDomain  (const SmExtent2d & sUVDomain,                       // in : target domain                                                                 
                                 SmBoolean          bDrawUVPlane=FALSE,              // in : TRUE = Display UVTrimCurves on a UVplane icon near the surface, FALSE = don't                                                        
                                 SmGfxArraySet    * pOptGfxSet=NULL)                 // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.    
                                const ;                
                                                               
  SmDisplayList * DrawUVDomain  (const SmTArray<SmCurve *> * cpOptUVCurves=NULL,     // in : Array of UVCurves to Draw in UVPlane 3d icon                                
                                 const SmExtent2d          * cpOptUVDomain=NULL,     // in : Subdomain only used to size UVPlane Graphic size - always draw full surface 
                                                                                     //    : ignored when NULL or when pOptUVCurves is given                             
                                 SmBoolean                   bDraw2dBoundaries=TRUE, // in : TRUE = Draw 2d UVPlane icon graphics                                        
                                 SmBoolean                   bDraw3dBoundaries=TRUE, // in : TRUE = Draw 3d Surface Natural Boundary Curves, FALSE = don't               
                                 SmBoolean                   bAddNormals=FALSE,      // in : TRUE = draw Normal comb, FALSE = don't                                      
                                 double                      dNormalScale=1.0,       // in : NormalComb scale factor, 1.0 = Draw Unit Normal Vectors                     
                                 const SmContext           * cpContext=NULL,         // in : context used when allocating rpUVPlane                                      
                                 SmPlane                  ** ppOptUVPlane=NULL,      // out: UVPlane upon which UVTrimCurves can be drawn as a unitSquare icon.          
                                 SmGfxArraySet             * pOptGfxSet=NULL)        // i/o: When given output GfxVertexArrays not GL calls.                          
                                const ; 

  // Create a SmPlane to use for drawing UV data on a plane near the surface
  void GetNearPlaneForUVDraw(SmPlane         ** ppUVPlane,          // out: Should be NULL on input, needs to be freed by caller
                             const SmExtent2d * cpOptUVDomain=NULL) // in : Subdomain to size UVPlane Graphic size - always draw full surface
                                                                    //    : NULL=Use NaturalUVDomain
                            const ;                                  

  // convenience draw functions
  // draw surface and crossHatch lines, or NumBetweenU == NumBetweenV == 999: Draw as 4 corners connected by lines
  virtual SmDisplayList * DrawUV (ULONG              lNumBetweenU=8,             // in : number of U IsoParameter lines between knots      
                                  ULONG              lNumBetweenV=8,             // in : number of V IsoParameter lines between knots                             
                                  SmBoolean          bVaryCrossHatchColor=FALSE, // in : TRUE = Draw U Lines in ObjectColor                
                                                                                 //    :        Draw V lines in m_VaryCrossHatchColor      
                                                                                 //    : FALSE= Draw both U and V Lines in ObjectColor     
                                  const SmExtent2d * pOptUVDomain=NULL,          // in : UVDomain to crossHatch, NULL=use NaturalUVDomain  
                                  SmBoolean          bAddToUIPickList=FALSE,     // in : TRUE=Add to UI pick list, FALSE=don't        
                                  SmGfxArraySet    * pOptGfxSet=NULL)            // i/o: When given output GfxVertexArrays not GL calls.
                                 const ;     
                                                                                                                
  // draw surface and crossHatch lines with curvature combs
  SmDisplayList * DrawCurvature(double             dCurvatureGain=-25,         // in : CurvatureVec gain, Use neg values.                       
                                ULONG              lSamplePointCount=150,      // in : CurvatureVec cnt per isoparam curve curvature comb       
                                ULONG              lNumBetweenU=8,             // in : number of U IsoParameter lines between knots             
                                ULONG              lNumBetweenV=8,             // in : number of V IsoParameter lines between knots                             
                                SmBoolean          bVaryCrossHatchColor=FALSE, // in : TRUE = Draw U Lines in ObjectColor                       
                                                                               //    :        Draw V lines in m_VaryCrossHatchColor             
                                                                               //    : FALSE= Draw both U and V Lines in ObjectColor            
                                const SmExtent2d * pOptUVDomain=NULL,          // in : UVDomain to crossHatch, NULL=use NaturalUVDomain         
                                SmBoolean          bAddToUIPickList=FALSE,     // in : TRUE=Add to UI pick list, FALSE=don't               
                                SmGfxArraySet    * pOptGfxSet=NULL)            // i/o: When given output GfxVertexArrays not GL calls.       
                               const ;     
                                                                                  
  virtual SmDisplayList * DrawPolygon      (SmGfxArraySet * pOptGfxSet=NULL) const ; // eff: draw surface and controlNet
  virtual SmDisplayList * DrawControlPoints(SmGfxArraySet * pOptGfxSet=NULL) const ; // eff: draw surface and controlPoints
  virtual SmDisplayList * DrawSeams        (SmGfxArraySet * pOptGfxSet=NULL) const ; // eff: draw only Seam IsoParam Curves
  virtual SmDisplayList * DrawPoles        (SmGfxArraySet * pOptGfxSet=NULL) const ; // eff: draw only point icon for every surface singular edge
  virtual SmDisplayList * DrawSingularities(SmGfxArraySet * pOptGfxSet=NULL) const { return DrawPoles(pOptGfxSet) ; }
  virtual SmDisplayList * DrawFlatCorners  (SmGfxArraySet * pOptGfxSet=NULL) const ; // eff: draw graphics for FlatCorners
  
  virtual SmDisplayList * DrawNet          (SmGfxArraySet * pOptGfxSet=NULL) const ; // eff: OldName draw surface and controlNet

  SmDisplayList         * DrawParams(const SmExtent2d * pUVDomain=NULL,              // draw Udir IsoCurve (Green->Blue, ConstV=Min & Max) 
                                     SmGfxArraySet    * pOptGfxSet=NULL) const ;      //      Vdir Isocurve (Green->Red,  ConstU=Min & Max)                                                                            
  SmDisplayList         * DrawUVBox  (SmExtent2d    & rBBox,                            
                                      SmGfxArraySet * pOptGfxSet=NULL) const ;          // draw rBBox projected through surface

  SmDisplayList         * DrawInspectUVTrimCurve(const SmCurve    & crUVTrimCurve,      // in : UVTrimCurve (expected to share parameterization with this curve)                                                               
                                                 const SmCurve    & cr3dCurve,          // in : 3dCurve mated to the UVTrimCurve                                             
                                                 const SmExtent1d * pOptIvl=NULL,       // in : Ivl to draw, NULL=NaturalIvl, default:[NULL]                                 
                                                 ULONG              lSampleCnt=100,     // in : Number of samples, default:[100]                                             
                                                 SmBoolean          bDrawUVPlane=FALSE, // in : TRUE = Display UVTrimCurves on a UVplane icon near the surface               
                                                 ULONG              lOutputIndex=0,     // in :   0=this pts (blue)                                                          
                                                                                        //    : & 1=add TrimPts and Normal (red)                                             
                                                                                        //    : & 2=add DropToTrimPts and Normal (green)                                     
                                                                                        //    : & 4=add DropToSrfPts and Normal  (cyan) (Debug mode only)                    
                                                                                        //    : & 8=add Dropped(dropSrfPt)ToUVTrimPts and Normal (orange) (Debug mode only)  
                                                                                        //    : &16=add vectors between drawn points                                         
                                                 SmGfxArraySet    * pOptGfxSet=NULL)    // NotUsed: i/o :
                                                const ;
                        
  void DrawInspectTwoSurfaces ( const SmSurface *pSrf2,  SmGfxArraySet * pOptGfxSet=NULL) const ;  // Inspect two surfaces up close.
  void DrawInspectCurveSurface( const SmCurve   *pCrv,   SmGfxArraySet * pOptGfxSet=NULL) const ;  // Inspect a curve and a surface up close.
  void DrawInspectSurfaceCurve( const SmCurve   *pUVCrv, SmGfxArraySet * pOptGfxSet=NULL) const ;  // Inspect a surface along a uv curve. NotUsed: in : pOptGfxSet

  ULONG GetSurfaceNumberInBrep( )const ;
  void  DumpTopology(ULONG lWalkDepth=0) const ;  // NotUsed: in : lWalkDepth[0] = no walk, [99] = walk to bottom

} ; // end class SmSurface

// GWC:BIND_TEMPLATES_MOVE  SM_TARRAY_TEMPLATE_PREDECLARATION(SmSurface*) ;

/*******************************************************************//**
PURPOSE: A class to make surface evaluation convenient.

NOTES:
 - The methods return references to our actual data,
   so that data could be changed from outside.  Not recommended.
***********************************************************************/
// ToDo:
// - allow 3rd derivs
// - don't always eval 2 derivs:
//   - requires NumEvaluated, not just Boolean m_bUVSet.
//   - and an arg to Eval().
// - allow out-of-bounds evaluation (if desired).

class SM_EXPORT SmSurfaceEval
{
protected:
  SmBoolean  m_bNonZeroTangents = TRUE ;          // TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                                  // FALSE= return exact tangent values                                          
                                                  // note: There are reasons to use this flag both ways. When tangents are being 
                                                  //       used for their direction (Binorm, SurfNorm comps) set this to TRUE,
                                                  //       but when the tangent is being used for its magnitude (like an arc-length comp)     
                                                  //       then set this to FALSE.                                               
                                                  // default:[TRUE]    
                                                                                            
  SmPoint2d  m_sUV;                               // independent, Surface UV position
  SmBoolean  m_bUFromLeft = TRUE ;                // independent, if P is on U interval boundary
                                                  //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                                  //      FALSE = evaluate P in lower interval where P is on the right of the interval
                                                  // default:[TRUE]                                                              
  SmBoolean  m_bVFromLeft = TRUE ;                // independent, if P is on V interval boundary
                                                  //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                                  //      FALSE = evaluate P in lower interval where P is on the right of the interval
                                                  // default:[TRUE]                                                              
                                                  
  SmVector3d m_sSectDir;                          // independent, Vector direction defining NormalCrossSection of surface
                                                  //              can't be parallel to SurfaceNormal, N.
                                                  
  SmVector3d m_sPVV[3][3] ;                       // position: S   = m_sPVV[0][0]   // dependent on m_sUV : set when m_bUVSet == TRUE  
                                                  // TangentU: Su  = m_sPVV[1][0]   // dependent on m_sUV : set when m_bUVSet == TRUE
                                                  // TangentV: Sv  = m_sPVV[0][1]   // dependent on m_sUV : set when m_bUVSet == TRUE
                                                  // 2ndDerUU: Suu = m_sPVV[2][0]   // dependent on m_sUV : set when m_bUVSet == TRUE
                                                  // 2ndDerUV: Suv = m_sPVV[1][1]   // dependent on m_sUV : set when m_bUVSet == TRUE
                                                  // 2ndDerVV: Svv = m_sPVV[0][2]   // dependent on m_sUV : set when m_bUVSet == TRUE
                                                  // Normal  : N   = m_sPVV[2][2]   // dependent on m_bUVSet: set when m_bNormalSet == TRUE
  SmVector3d m_sSectTangent ;                     // Surface unit-Tangent in CrossSection Direction     // dependent on m_bUVSet and m_bNormalSet: Set when m_bSectDirSet == TRUE
  double     m_dSectCurvature = SM_UNDEF_DOUBLE ; // Surface Signed Curvature in CrossSection Direction // dependent on m_bUVSet and m_bNormalSet: Set when m_bSectDirSet == TRUE
                                   
  //      SmPoint3d  m_sPos;                                 // dependent on m_sUV: m_bUVSet
  //      SmVector3d m_sSu, m_sSv, m_sSuu, m_sSuv, m_sSvv ;  // dependent on m_sUV: m_bUVSet
  //      SmVector3d m_sNormal;                              // dependent on m_bUVSet: m_bNormalSet

  const SmSurface *m_pSrf; // We do not own this.

  // Allow for lazy evaluation.
  SmBoolean  m_bUVSet      = FALSE ;   // FALSE: after m_sUV change,               TRUE: pos,tangents,2ndDers are ready
  SmBoolean  m_bNormalSet  = FALSE ;   // FALSE: after m_sUV change,               TRUE: Normal is ready
  SmBoolean  m_bSectDirSet = FALSE ;   // FALSE: after m_sUV or m_sSectDir change, TRUE: m_sSectTangent,m_dSectCurvature are ready
  SmBoolean  m_bFailsEval  = FALSE ;   // FALSE: okay, TRUE: call to Eval Fails.
                                       //                    can happen forOutOfBoundary Evaluations, and poorly defined surfaces

public:
  // Constructors
  SmSurfaceEval( const SmSurface *pSurf = NULL ) 
               : m_bNonZeroTangents(TRUE),
                 m_bUFromLeft( TRUE ),
                 m_bVFromLeft( TRUE ),                     
                 m_pSrf( pSurf ),
                 m_bUVSet( FALSE ),
                 m_bNormalSet( FALSE ),
                 m_bSectDirSet( FALSE)
              { } 
              
  SmSurfaceEval( const SmSurface *pSurf, double dU, double dV )                                           
               : m_bNonZeroTangents(TRUE),
                 m_bUFromLeft( TRUE ),
                 m_bVFromLeft( TRUE ),  
                 m_pSrf( pSurf ),
                 m_bUVSet( FALSE ),
                 m_bNormalSet( FALSE ),
                 m_bSectDirSet( FALSE)
               { SetUV( dU, dV ) ; } 

  SmSurfaceEval( const SmSurface *pSurf,  double dU, double dV, const SmVector3d &rSectDir )    
               : m_bNonZeroTangents(TRUE),
                 m_bUFromLeft( TRUE ),
                 m_bVFromLeft( TRUE ),
                 m_pSrf( pSurf ),
                 m_bUVSet( FALSE ),
                 m_bNormalSet( FALSE ),
                 m_bSectDirSet( FALSE)
               { SetUV( dU, dV ) ; SetSectDir( rSectDir) ; } 

  // Assignment operator
  SmSurfaceEval & operator=( const SmSurfaceEval &crOther )
                           { if(this == &crOther) return *this;
                             m_bNonZeroTangents = crOther.m_bNonZeroTangents ;
                             m_sUV.Copy(crOther.m_sUV) ; 
                             m_bUFromLeft       = crOther.m_bUFromLeft ;
                             m_bVFromLeft       = crOther.m_bVFromLeft ;
                             m_sSectDir.Copy(crOther.m_sSectDir) ; 
                             for(ULONG ii=0;ii<3;ii++)
                               for(ULONG jj=0;jj<3;jj++)
                                 { m_sPVV[ii][jj].Copy(crOther.m_sPVV[ii][jj]) ; }
                             m_sSectTangent.Copy(crOther.m_sSectTangent) ;
                             m_dSectCurvature  = crOther.m_dSectCurvature ;
                             m_pSrf            = crOther.m_pSrf ;
                             m_bUVSet          = crOther.m_bUVSet      ;
                             m_bNormalSet      = crOther.m_bNormalSet  ;
                             m_bSectDirSet     = crOther.m_bSectDirSet ;
                             m_bFailsEval      = crOther.m_bFailsEval  ;
                             return *this;
                           }

  SmBoolean    operator==(const SmSurfaceEval &crOther )
                         { SmBoolean bRtn = TRUE ;
                           bRtn &= m_bNonZeroTangents == crOther.m_bNonZeroTangents ;
                           bRtn &= m_sUV              == crOther.m_sUV ;
                           bRtn &= m_bUFromLeft       == crOther.m_bUFromLeft ;
                           bRtn &= m_bVFromLeft       == crOther.m_bVFromLeft ;
                           bRtn &= m_sSectDir         == crOther.m_sSectDir ;
                           for(ULONG ii=0;ii<3;ii++)
                             for(ULONG jj=0;jj<3;jj++)
                               { if(m_sPVV[ii][jj].IsInitialized()) { bRtn &= m_sPVV[ii][jj] == crOther.m_sPVV[ii][jj] ; }
                               }
                           bRtn &= m_sSectTangent    == crOther.m_sSectTangent ;
                           bRtn &= m_dSectCurvature  == crOther.m_dSectCurvature ;
                           bRtn &= m_pSrf            == crOther.m_pSrf ;
                           bRtn &= m_bUVSet          == crOther.m_bUVSet      ;
                           bRtn &= m_bNormalSet      == crOther.m_bNormalSet  ;
                           bRtn &= m_bSectDirSet     == crOther.m_bSectDirSet ;
                           bRtn &= m_bFailsEval      == crOther.m_bFailsEval  ;
                           return bRtn ;
                         }
  // Destructor
  ~SmSurfaceEval() { m_pSrf = NULL; }

  // Set Values
  void SetSurface( const SmSurface * pSurface )
                 {
                   m_pSrf = pSurface;
                   m_bUVSet = m_bNormalSet = m_bSectDirSet = FALSE;
                   m_bFailsEval = FALSE;
                 }

  SmStatus SetNonZeroTangents( SmBoolean bNonZeroTangents )
                             {
                               if(bNonZeroTangents != m_bNonZeroTangents)
                               {
                                 m_bNonZeroTangents = bNonZeroTangents;
                                 m_bUVSet = m_bNormalSet = m_bSectDirSet = FALSE;
                                 m_bFailsEval = FALSE;
                               } return SM_SUCCESS;
                             }

  SmStatus SetUV( const SmPoint2d & crUV )
                {
                  if(m_sUV.x != crUV.x || m_sUV.y != crUV.y)
                  {
                    m_sUV = crUV;
                    m_bUVSet = m_bNormalSet = m_bSectDirSet = FALSE;
                    m_bFailsEval = FALSE;
                  }
                  return SM_SUCCESS;
                }

  SmStatus SetUV( const double dU, const double dV )
                {
                  if(m_sUV.x != dU || m_sUV.y != dV)
                  {
                    m_sUV.Set( dU, dV ) ;
                    m_bUVSet = m_bNormalSet = m_bSectDirSet = FALSE;
                    m_bFailsEval = FALSE;
                  }
                  return SM_SUCCESS;
                }

  SmStatus SetUFromLeft( SmBoolean bUFromLeft )
                       {
                         if(m_bUFromLeft != bUFromLeft)
                         {
                           m_bUFromLeft = bUFromLeft;
                           m_bUVSet = m_bNormalSet = m_bSectDirSet = FALSE;
                           m_bFailsEval = FALSE;
                         }
                         return SM_SUCCESS;
                       }

  SmStatus SetVFromLeft( SmBoolean bVFromLeft )
                       {
                         if(m_bVFromLeft != bVFromLeft)
                         {
                           m_bVFromLeft = bVFromLeft;
                           m_bUVSet = m_bNormalSet = m_bSectDirSet = FALSE;
                         }
                         return SM_SUCCESS;
                       }

  SmStatus SetSectDir( const SmVector3d &rSectDir )
                     {
                       m_sSectDir = rSectDir;
                       m_bSectDirSet = FALSE;
                       return SM_SUCCESS;
                     }

  // Get independent Values
  const SmSurface  * GetSurface()   const { return m_pSrf ; }
  const SmPoint2d  & GetUV()        const { return m_sUV ;  }
  SmBoolean          GetUFromLeft() const { return m_bUFromLeft ; }
  SmBoolean          GetVFromLeft() const { return m_bVFromLeft ; }
  const SmVector3d & GetSectDir()   const { return m_sSectDir; }

  // Get dependent Values - may cause lazy evaluations

  // use FailsEval() when evaluating outside the DomainBoundary and possible failures are expected and handled
  SmBoolean FailsEval(SmBoolean bSignalFails=FALSE)    
                     { Eval(bSignalFails) ; return m_bFailsEval ; }

  // gwc: design flaw - when FailsEval() == TRUE, these methods return uninitialized values, check FailsEval().
  SmPoint3d    & Pos()           { SM_ASSERT_MSG(FailsEval() == FALSE, _T("SmSurfaceEval::Pos() returning uninit value because m_pSrf->Evaluate() failed")) ; 
                                   Eval() ; return m_sPVV[0][0] ; }
  SmVector3d   & Su ()           { SM_ASSERT_MSG(FailsEval() == FALSE, _T("SmSurfaceEval::Su () returning uninit value because m_pSrf->Evaluate() failed")) ;  
                                   Eval() ; return m_sPVV[1][0] ; }
  SmVector3d   & Sv ()           { SM_ASSERT_MSG(FailsEval() == FALSE, _T("SmSurfaceEval::Sv () returning uninit value because m_pSrf->Evaluate() failed")) ;  
                                   Eval() ; return m_sPVV[0][1] ; }
  SmVector3d   & Suu()           { SM_ASSERT_MSG(FailsEval() == FALSE, _T("SmSurfaceEval::Suu() returning uninit value because m_pSrf->Evaluate() failed")) ;  
                                   Eval() ; return m_sPVV[2][0] ; }
  SmVector3d   & Suv()           { SM_ASSERT_MSG(FailsEval() == FALSE, _T("SmSurfaceEval::Suv() returning uninit value because m_pSrf->Evaluate() failed")) ;  
                                   Eval() ; return m_sPVV[1][1] ; }
  SmVector3d   & Svv()           { SM_ASSERT_MSG(FailsEval() == FALSE, _T("SmSurfaceEval::Svv() returning uninit value because m_pSrf->Evaluate() failed")) ;  
                                   Eval() ; return m_sPVV[0][2] ; }
  SmVector3d   & Normal()        { SM_ASSERT_MSG(FailsEval() == FALSE, _T("SmSurfaceEval::Normal() returning uninit value because m_pSrf->Evaluate() failed")) ;  
                                   EvalNormal() ; return m_sPVV[2][2] ; }
  SmVector3d   & SectTangent()   { SM_ASSERT_MSG(FailsEval() == FALSE, _T("SmSurfaceEval::SectTangent() returning uninit value because m_pSrf->Evaluate() failed")) ;  
                                   EvalNormalSection() ; return m_sSectTangent ; }
  double         SectCurvature() { SM_ASSERT_MSG(FailsEval() == FALSE, _T("SmSurfaceEval::SectCurvature() returning uninit value because m_pSrf->Evaluate() failed")) ;  
                                   EvalNormalSection() ; return m_dSectCurvature ; }

  // pretty print
  void Dump() ;

private:
  SmStatus Eval(SmBoolean bSignalFails=TRUE) ;
  SmStatus EvalNormal() ;
  SmStatus EvalNormalSection() ;

} ; // end class SmSurfaceEval

/*******************************************************************//**
PURPOSE: This enum controls how derivative surfaces are constructed.       
    Their values are described with the SmDerivSurfDefinition declaration, below.

NOTES:
***********************************************************************/
enum SmTangencyType
{
  // when building DerivSurface from a surface boundary
  //       compute D,Dv,Dvv,... values from Surface directionalDerivatives
  SM_TT_NO_TANGENCY,
  SM_TT_TAN_TO_EDGES,
  SM_TT_MATCH_NEIGHBORS,
  SM_TT_DIR_TO_REFERENCE_CURVE,

  // when building DerivSurface from a BaseCurve
  //       compute D,Dv, values from BaseCurve and input properties
  SM_TT_G1_FROM_CURVE,
  SM_TT_ALONG_VECTOR,
  SM_TT_PERPENDICULAR_TO_VECTOR,
  SM_TT_PARALLEL_TO_CURVATURE,
  SM_TT_PERPENDICULAR_TO_CURVATURE

} ; // end enum SmTangencyType

/*******************************************************************//**
PURPOSE: This enum controls the continuity between the derivative surface
  being created and the original surface being extended. As an extension
  of that, it also controls the continuity of new blend surfaces.
    
NOTES: 
***********************************************************************/
enum SmCurvatureType
{
  SM_CT_NO_CURVATURE    = 0, // ruled surface

 // when building DerivSurface from a surfaceBoundary
  SM_CT_G2_FROM_SURFACE = 1, // G2 continuity between original and deriv surfaces
  SM_CT_G3_FROM_SURFACE = 2, // G3 continuity between original and deriv surfaces

 // when building DerivSurface from a basecurve
  SM_CT_G2_FROM_CURVE   = 3,
  SM_CT_FROM_PARAMETERS = 4

} ; // end enum SmCurvatureType

/*******************************************************************//**
PURPOSE: This enum controls the connections between a new blend surface
  and the original surfaces being blended. 

NOTES: The connection can either be G* or D*, i.e. smooth or cusped. 
  For a cusped connection the two surfaces have the same normal (up to
  scaling) at the shared edge.

***********************************************************************/
enum SmBlendEndType
  {
  SM_BE_NO_CUSP,  // smooth continuity between original and blend surfaces
  SM_BE_1ST_CUSP, // 1st edge connection of blend surface is cusped
  SM_BE_2ND_CUSP, // 2nd edge connection of blend surface is cusped
  SM_BE_BOTH_CUSP // Both edges are cusped
  } ; // end enum SmBlendEndType

// For both SmCurve and SmSurface implementations of FixRepeatedEndControlPoints().
// The normalized distance a repeated end CPt is moved between its neighbors.
#define SM_FR_NORMALIZED_PARAM  .99  // a number in the range 0 < NP < 1
                                     // numbers closer to 1 leave moved CPT closer to its 
                                     // original position

/*******************************************************************//**
PURPOSE: Option storage container for controlling how DerivSurfaces are built
              in SmEdgeuse::CreateDerivativeSurface() and
                 SmDerivSurfDefinition::CreateDerivativeSurface().

NOTES: DerivSurfaces are used as input or to generate the input
  to blend and filleting functions.

  They represent the cross-boundary derivative information along an edge of a
  surface.  They represent position, cross-tangent, cross-2ndDeriv and possibly
  higher-order derivatives in the crossing direction.  This information is stored
  in a surface and retrieved from the surface by making surface evaluations along
  the v=0 isoparameter curve as

     D(u,0) = Pos(u)
     Dv(u,0) = 1stCrossDeriv(u)
     Dvv(u,0) = 2ndCrossDeriv(u)
     Dvvv(u,0) = 3rdCrossDeriv(u)

  DerivSurfaces are built in 3 steps:
    1. construct D, Dv, Dvv, . . . for a sequence of sample points along a
       curve or along a curve on a surface. When working with a curve on a surface
       the D, Dv, Dvv, . . . values are computed as directional derivatives.
       For each sample point, there are many options for selecting the direction
       of the directional derivative.  When working with just a curve there are
       also many options specifying how to compute the D, Dv, Dvv, ... values.
       The flags that control these options are all stored in the SmDerivSurfDefintion.

    2. For each sample point create a single cross-section curve whose start
       pos, 1stDeriv, 2ndDeriv, ... values equal the sampled D, Dv, Dvv, ... values.

    3. The sequence of sample cross-section curves is skinned producing a BSplineSurface
       that is the Deriv surface, that is evaluations along the v=0 isoparameter
       value yield functions of D(u), Dv(u), Dvv(u), ...

  Each step is encapsulated in a method so that the sequence can be used
    to build deriv surfaces and blend surfaces as well. The intended use for blending
    is shown in outline in the following G3 blend example.

    {
      // output
      SmBSplineSurface *pBlendSurface ;

      // locals
      SmDerivSurfDefintion sDS ;
      SmTArray<double>       sParams ;
      SmTArray<SmVector3d> & sPos1,       & sPos2 ;
      SmTArray<SmVector3d> & s1stDerivs1, & s1stDerivs2 ;
      SmTArray<SmVector3d> & s2ndDerivs1, & s2ndDerivs2 ;
      SmTArray<SmVector3d> & s3rdDerivs1, & s3rdDerivs2 ;
      SmTArray<SmBSplineCurve *>     sCrossSections ;
      SmObjsDelete<SmBSplineCurve *> sCleanup(&sCrossSections) ;

      // set Blend constraints
      . . .

      // in debug mode: check for consistent blend constraints
      SM_ASSERT_VALID(sDS) ;

      // Get sample points on 1st side for G3 blend
      sDS.m_pTangentReference = sCurve2 ;
      sDS.GetSampleValues(sSurface1, sCurve1, sCurveIvl1, TRUE, sParams, sPos1, s1stDerivs1, &s2ndDerivs1, &s3rdDerivs1) ;

      // Get sample points on 2nd side for G3 blend
      sDS.m_pTangentReference = sCurve1 ;
      sDS.GetSampleValues(sSurface2, sCurve2, sCurveIvl2, FALSE, sParams, sPos2, s1stDerivs2, &s2ndDerivs2, &s3rdDerivs2) ;

      // build blend curves between sample points for G3 blend
      sDS.CreateCrossSectionCurves(crContext, sCrossSections,
                                   sPos1, s1stDerivs1, &s2ndDerivs1, &s3rdDerivs1,
                                   sPos2, s1stDerivs2, &s2ndDerivs2, &s3rdDerivs2) ;

      // Skin cross sections to build output surface for G3 blend
      SkinDerivSurface(crContext, rpDerivSurface,
                       sCrossSections, sParams, sCurve1, &sCurve2, dTolerance) ;
    }


Descriptions of option parameters:

m_lNumSamples: Number of sample points along a CurveOnSurface whose
               cross-derivative values define the DerivativeSurface being built.
               default: 100


Tangent direction and magnitude at take-off -- first cross-boundary derivative:

m_eTangencyType: Defines the take-off angle for isoparameter lines in the blend surface.

  When creating DerivSurfaces from a single base curve, cross-tangent dir is:

    SM_TT_G1_FROM_CURVE              : vector( TangentReferencePoint - CurvePoint )
    SM_TT_ALONG_VECTOR               : given m_vVector
    SM_TT_PERPENDICULAR_TO_VECTOR    : perp to CurveNormal and given m_vVector
    SM_TT_PARALLEL_TO_CURVATURE      : CurveNormal
    SM_TT_PERPENDICULAR_TO_CURVATURE : CurveBiNormal
      Cross-curvature dir will be cross(cross-tangent-dir, curve-tangent-dir).

  When creating DerivSurfaces from a surface curve, dir for directionalDerivatives is:

    SM_TT_NO_TANGENCY            : binormal direction (out from surface).
    SM_TT_TAN_TO_EDGES           : linear interp between neighbor end-UV-tangents.
    SM_TT_MATCH_NEIGHBORS        : linear interp between avg of this and neighbor end-UV-tangents.
    SM_TT_DIR_TO_REFERENCE_CURVE : vector( TangentReferenceCurvePoint-CurvePoint),
                                   projected onto surface tangent plane.
  default: SM_TT_MATCH_NEIGHBORS

  Currently for internal use when creating DerivSurfaces from a surface boundary
      and m_eTangencyType == SM_TT_TAN_TO_EDGES or SM_TT_MATCH_NEIGHBORS
  - in SmEdgeuse::CreateDerivativeSurface() :
    - set to meet SM_TT_TAN_TO_EDGES and SM_TT_MATCH_NEIGHBORS constraint
  - in SmDerivSurfDefinition::GetSampleValues() :
    - used for    SM_TT_TAN_TO_EDGES and SM_TT_MATCH_NEIGHBORS to linear interpolate
      directions for directionalDerivatives.

  Note, SM_TT_MATCH_NEIGHBORS is designed so that when two adjacent, non-tangent
  edges both have blend surfaces built from them, the blend surfaces can join
  each other smoothly.

m_dStartDerivDirAngleDeg:
m_dEndDerivDirAngleDeg  :
  These define the rotation of the take-off direction.  The angle is a linear
  interpolation from m_dStartDerivDirAngleDeg to m_dEndDerivDirAngleDeg:
    DirectionalDerivAng = (1-param)*m_dStartDerivDirAngleDeg + (param)*m_dEndDerivDirAngleDeg
  where DirectionalDerivAng is measured in the surface UV coordinate system
  as the CounterClockwise angle from the binormal UVDir.
  These are applied when m_eTangencyType is SM_TT_TAN_TO_EDGES or SM_TT_MATCH_NEIGHBORS.

m_pTangentReference: When m_eTangencyType is SM_TT_DIR_TO_REFERENCE_CURVE,
  this is the curve used to define the take-off direction: the direction is
  the vector from this curve to m_pTangentReference, evaluated at corresponding
  parameters, and projected to the surface tangent plane if appropriate.
  The magnitude of the vector is the distance between the two curve evaluations,
  scaled by  m_dStartTanLength and m_dEndTanScale if given.
  When blending between two curves or between to surface edges,
  m_pTangentReference is generally the other curve or edge.
  default: NULL.

m_dStartTanLength:
m_dEndTanScale   :
  These are used to scale the magnitude of the take-off vector.
  m_dStartTanLength is the scaling at the beginning of the blend surface,
  and the scaling is multiplied by m_dEndTanScale at the end, and linearly
  interpolated in between:
    scaling = (1-param)*m_dStartTanLength + (param)*m_dStartTanLength*m_dEndTanScale.
  Defaults: 1.0.

So the length of the cross-tangent take-off vectors is:
- If m_pTangentReference curve is given, use the distance to that.
- If length scaling is given (m_dStartTanLength and m_dEndTanScale > 0),
  scale the vectors according to those values, as described above.
- If neither is given, then
  - If building from a base curve, use 10.0.
  - Else (building from a surface), use the magnitude of the underlying
    surface derivatives in that direction.

m_dStartAngleDeg:
m_dEndAngleDeg  :
  These define a rotation that may be applied to the take-off direction.
  Rotation is about the base curve or edge, and is linearly interpolated
  from m_dStartAngleDeg at the start of the blend to m_dEndAngleDeg at the end.
  Defaults: 0.0.

m_vVector: Used to define the take-off direction.
  Used only when building from a base curve.
  When m_eTangencyType is:
    SM_TT_G1_FROM_CURVE              : not used
    SM_TT_ALONG_VECTOR               : take-off dir = m_vVector
    SM_TT_PERPENDICULAR_TO_VECTOR    : take-off dir = perp(m_vVector,CurveNormal)
    SM_TT_PARALLEL_TO_CURVATURE      : not used
    SM_TT_PERPENDICULAR_TO_CURVATURE : not used
  default: [0,0,0]


Cross-curvature at take-off -- higher-order cross boundary derivatives:

m_eCurvatureType: Higher order derivative control where the DerivSurface
  takes off from the base curve or surface edge -- the v=0 isoparameter.
  Position (D) and tangent direction (Dv) are always specified.
  Values are:
    SM_CT_NO_CURVATURE = Use D and Dv values only.
  when building from a basecurve:
    SM_CT_G2_FROM_CURVE   = Use D,Dv,Dvv and let mag(Dvv) = Dist(BaseCurve(s),CurvatureRef(s))
    SM_CT_FROM_PARAMETERS = Use D,Dv,Dvv and let mag(Dvv) = LinearInterp(StartRadius,EndRadius)
  when building from a surface boundary:
    SM_CT_G2_FROM_SURFACE = Use D,Dv,Dvv      values.
    SM_CT_G3_FROM_SURFACE = Use D,Dv,Dvv,Dvvv values.
  default: [SM_CT_G2_FROM_SURFACE]

m_dStartRadius:
m_dEndRadScale:
  These are used to scale the radius of curvature, when m_eCurvatureType
  is SM_CT_FROM_PARAMETERS.  (These are the parameters.)
  Note that this is the case only when working from a base curve.
  m_dStartRadius is the scaling at the beginning of the blend surface,
  and the scaling is multiplied by m_dEndRadScale at the end, and linearly
  interpolated in between:
    scaling = (1-param)*m_dStartRadius + (param)*m_dStartRadius*m_dEndRadScale.
 Defaults: 0.0


m_pCurvatureReference:
  This is used to scale the radius of curvature, when m_eCurvatureType
  is SM_CT_G2_FROM_CURVE.  (This is the curve.)
  Note that this is the case only when working from a base curve.
  RadiusOfCurvatuve = Dist( CurvePos, CurvatureReferencePos ).
  default: Null.

m_bFlipCurvature: if TRUE, negate the final curvature vector computation.
  default: FALSE.

m_eBlendEndType: Choose between a smooth or cusp boundary between each surface
  and the new blend surface. In SMLib nomenclature this is a choice of G* or D*
  boundaries. For use with SmPrimitiveCreation::CreateBlendPrimitive().
  Values are:
    SM_BE_NO_CUSP   = Both boundaries are smooth
    SM_BE_1ST_CUSP  = Boundary at first surface is a cusp
    SM_BE_2ND_CUSP  = Boundary at second surface is a cusp
    SM_BE_BOTH_CUSP = Boundary between blend and original surfaces are cusps 
  default: [SM_BE_NO_CUSP]
***********************************************************************/
class SM_EXPORT SmDerivSurfDefinition
{
public:
 // Options for Creating Derivative Surfaces.
 // These are described above.

  ULONG             m_lNumSamples;
  SmTangencyType    m_eTangencyType;

  double            m_dStartDerivDirAngleDeg;
  double            m_dEndDerivDirAngleDeg;

  const SmCurve   * m_pTangentReference;
  double            m_dStartTanLength;
  double            m_dEndTanScale;

  double            m_dStartAngleDeg;
  double            m_dEndAngleDeg;
  SmVector3d        m_vVector;

  SmCurvatureType   m_eCurvatureType;
  SmBlendEndType    m_eBlendEndType;

  double            m_dStartRadius;
  double            m_dEndRadScale;
  SmCurve         * m_pCurvatureReference;
  SmBoolean         m_bFlipCurvature;

public:
  // constructor
  SmDerivSurfDefinition() ;

  // destructor
  ~SmDerivSurfDefinition() ;

  // branch predicate
  SmBoolean IsFromBaseCurve() { return (m_eTangencyType == SM_TT_G1_FROM_CURVE
                                                                    || m_eTangencyType == SM_TT_ALONG_VECTOR
                                                                    || m_eTangencyType == SM_TT_PERPENDICULAR_TO_VECTOR
                                                                    || m_eTangencyType == SM_TT_PARALLEL_TO_CURVATURE
                                                                    || m_eTangencyType == SM_TT_PERPENDICULAR_TO_CURVATURE) ;
                              }


  SmBoolean IsFromSurfaceBoundaryCurve()
                                      {
                                        return (m_eTangencyType == SM_TT_NO_TANGENCY
                                             || m_eTangencyType == SM_TT_TAN_TO_EDGES
                                             || m_eTangencyType == SM_TT_MATCH_NEIGHBORS
                                             || m_eTangencyType == SM_TT_DIR_TO_REFERENCE_CURVE) ;
                                      }

  // convenience function: Build DerivSurface from base curve
  SmStatus CreateDerivativeSurface(const SmContext      & crContext,           // in : context for new object construction                
                                   const SmBSplineCurve & rBaseCurve,          // in : boundary for derivative surface                    
                                   double                 dTolerance,          // in : 0.0 = interpolate, else approximate                
                                   SmBSplineSurface    *& rpDerivSurface)      // out: Evaluate(u,0) yields D(u,0), Dv(u,0), Dvv(u,0), ...
                                  const ;

  // convenience function: Build DerivSurface from Surface Boundary
  SmStatus CreateDerivativeSurface(const SmContext      & crContext,           // in : Context for new object construction                                           
                                   const SmSurface      & rSurface,            // in : Target Surface                                                                
                                   const SmBSplineCurve & rBoundaryCurve,      // in : Boundary on that Surface, the closer CurveBoundary is to rSurface the better  
                                   SmExtent1d           & rBoundaryCurveIvl,   // in : target Boundary Curve interval                                                
                                   SmBoolean              bOffToTheRight,      // in : Specify how the tangents come off the curve, When walking along the direction 
                                                                               //    : of the curve with one's head in the surface normal's direction,               
                                                                               //    : TRUE = Tangents point to the curve's right hand side.                         
                                                                               //    : FALSE= Tangents point to the curve's left hand side.                          
                                   SmBoolean              bStartAtBoundary,    // in : Only used when m_eTangencyType == SM_TT_DIR_TO_REFERENCE_CURVE                
                                                                               //    : TRUE = direction dirs selected to start at BoundaryCurve and go to ReferencePt
                                                                               //    : FALSE= direction dirs selected to start at ReferencePt and go to BoundaryCurve
                                   double                 dTolerance,          // in : 0.0 = interpolate, else approximate                                           
                                   SmBSplineSurface    *& rpDerivSurface)      // out: The Derivative Surface                                                        
                                  const ;

  // Build D, Dv, Dvv sample value arrays from a BaseCurve
  SmStatus GetSampleValues(const SmCurve        & rBaseCurve,              // in : target curve                                                            
                           SmTArray<double>     & sParams,                 // out: param along Surface Boundary Curve for every output pos and crossDeriv  
                           SmTArray<SmVector3d> & rPos,                    // out: positions along rBaseCurve                                              
                           SmTArray<SmVector3d> & r1stDerivs,              // out: 1stDerivs along rBaseCurve                                              
                           SmTArray<SmVector3d> * pOpt2ndDerivs = NULL)    // out: Opt 2ndDerivs along rBaseCurve, NULL to ignore                          
                          const ;

  // Build D, Dv, Dvv sample value arrays from a Curve bounding a surface
  SmStatus GetSampleValues(const SmSurface   & rSurface,                   // in : target surface                                                                 
                           const SmCurve     & rCurveBoundary,             // in : Boundary on that Surface, the closer CurveBoundary is to rSurface the better   
                           SmExtent1d        & rBoundaryCurveIvl,          // in : Target Boundary Curve interval                                                 
                           SmBoolean           bOffToTheRight,             // in : Specify how the tangents come off the curve, When walking along the direction  
                                                                           //    : of the curve with one's head in the surface normal's direction,                
                                                                           //    : TRUE = Tangents point to the curve's right hand side.                          
                                                                           //    : FALSE= Tangents point to the curve's left hand side.                           
                           SmBoolean           bStartAtBoundary,           // in : Only used when m_eTangencyType == SM_TT_DIR_TO_REFERENCE_CURVE                 
                                                                           //    : TRUE = direction dirs selected to start at BoundaryCurve and go to ReferencePt 
                                                                           //    : FALSE= direction dirs selected to start at ReferencePt and go to BoundaryCurve 
                           SmTArray<double>     & sParams,                 // out: param along Surface Boundary Curve for every output pos and crossDeriv         
                           SmTArray<SmVector3d> & rPos,                    // out: positions                                                                      
                           SmTArray<SmVector3d> & r1stDerivs,              // out: cross1stDerivs                                                                 
                           SmTArray<SmVector3d> * pOpt2ndDerivs = NULL,    // out: cross2ndDerivs, NULL to ignore, default:[NULL]                                 
                           SmTArray<SmVector3d> * pOpt3rdDerivs = NULL)    // out: cross3rdDerivs, NULL to ignore, default:[NULL]                                 
                          const ;

  // Create PosCurve(u) = BSplineCurve whose values are the position values of the DerivSurf for all values u
  static SmStatus ExtractPosCurve(SmContext         & rContext,            // in : context for new object construction                                        
                                  SmBSplineSurface  & rDerivSurface,       // in : DerivSurface - probably made by one of the methods of this class           
                                  SmBSplineCurve   *& rpPosCurve) ;        // out: New BSplineCurve whose values are the positions                            
                                                                           //    : of the Deriv Surface for all valid u values.                               
                                                                           //    : The output is allocated in this function and must be deleted by the caller 
                                                                           //    : rpPosCurve should equal NULL on input                                      

  // Create CrossDerivCurve(u) = BSplineCurve whose values are the crossDerivative values of the DerivSurf for all values u
  static SmStatus ExtractCrossDerivCurve(SmContext         & rContext,            // in : context for new object construction                                        
                                         SmBSplineSurface  & rDerivSurface,       // in : DerivSurface - probably made by one of the methods of this class           
                                         SmBSplineCurve   *& rpCrossDerivCurve) ; // out: New BSplineCurve showe values are the positions                            
                                                                                  //    : of the Deriv Surface for all valid u values.                               
                                                                                  //    : The output is allocated in this function and must be deleted by the caller 
                                                                                  //    : rpPosCurve should equal NULL on input                                      

  // Create CrossDerivCurve(u) = BSplineCurve whose values are the cross2ndDerivative values of the DerivSurf for all values u
  static SmStatus Extract2ndDerivCurve(SmContext         & rContext,              // in :     
                                       SmBSplineSurface  & rDerivSurface,         // out:     
                                       SmBSplineCurve   *& rpCrossDerivCurve) ;   // out:     

  // simple access
  ULONG GetNumDerivs() { return   m_eCurvatureType == SM_CT_NO_CURVATURE ? 1
                                : m_eCurvatureType == SM_CT_G2_FROM_SURFACE ? 2
                                : m_eCurvatureType == SM_CT_G3_FROM_SURFACE ? 3
                                : m_eCurvatureType == SM_CT_G2_FROM_CURVE ? 2
                                : m_eCurvatureType == SM_CT_FROM_PARAMETERS ? 2 : 1;
                       }

  // build cross-section curves from sample values so that
  //  Curve(0) = StartPos(rStartPos), Curve'(0) = Start1stDeriv(rStartPos, . . . 
  //  and optionally Curve(1) = EndPos(*pOptEndPos),  Curve'(1) = End1stDeriv(*pOptEndPos), . . .
  SmStatus CreateCrossSectionCurves(const SmContext            & crContext,                   // in : context for new object construction                      
                                    SmTArray<SmBSplineCurve *> & sCrossSections,              // out: array of new SmBSpline bezier curves with Ivls = [0,1]   
                                    SmTArray<SmVector3d>       & rStartPos,                   // in : start position                                           
                                    SmTArray<SmVector3d>       & rStart1stDerivs,             // in : start 1st deriv                                          
                                    SmTArray<SmVector3d>       * pOptStart2ndDerivs = NULL,   // in : optional start 2nd deriv, NULL to ignore                 
                                    SmTArray<SmVector3d>       * pOptStart3rdDerivs = NULL,   // in : optional start 3rd dervi, NULL to ignore                 
                                    SmTArray<SmVector3d>       * pOptEndPos = NULL,           // in : optional end position                                    
                                    SmTArray<SmVector3d>       * pOptEnd1stDerivs = NULL,     // in : optional end 1st deriv                                   
                                    SmTArray<SmVector3d>       * pOptEnd2ndDerivs = NULL,     // in : optional end 2nd deriv                                   
                                    SmTArray<SmVector3d>       * pOptEnd3rdDerivs = NULL)     // in : optional end 3rd deriv                                   
                                   const ;

  // Build DerivSurface from CrossSection curves
  SmStatus SkinDerivSurface(const SmContext            & crContext,                   // in : context for new object construction                                                              
                            SmBSplineSurface          *& rpDerivSurface,              // out: The skinned surface                                                                              
                            SmTArray<SmBSplineCurve *> & sCrossSections,              // in : array of cross-sections to skin                                                                  
                            SmTArray<double>           & sParams,                     // in : param value for every cross-section                                                              
                            const SmBSplineCurve       & rStartRailCurve,             // in : Surface iso-parameter curve running through all sCrossSection start points                       
                            const SmBSplineCurve       * pOptEndRailCurve = NULL,     // in : Optional Surface iso-parameter curve running through all sCrossSection end points. NULL to ignore
                            double                       dTolerance = 0.0)            // in : 0.0 = interpolate cross-sections, else approximate                                               
                           const ;

  // return TRUE when all stored input values are self consistent and reasonable
  SmBoolean AssertValid(SmAssertArray    * pAList=NULL,             // i/o: Accumulating list of failed Asserts, NULL to ignore                          
                        SmAssertTestLevel  eTestLevel=SM_LEVEL_0,   // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                      
                                                                    //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                  
                        SmAssertWalking    eWalkTree=SM_WALK,       // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't   
                        SmTArray<ULONG>  * pTestRequests=NULL)      // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order,               
                       const ;

  SM_TYPE       GetType()            const { return(SmDerivSurfDefinition_TYPE) ; }
  const TCHAR  *GetTypeString()      const { return(_T("SmDerivSurfDefinition_TYPE")) ; }
  const TCHAR  *GetClassString()     const { return(_T("SmDerivSurfDefinition")) ; }
  SM_TYPE       GetClassType()       const { return(SmDerivSurfDefinition_TYPE) ; }
  const TCHAR  *GetClassTypeString() const { return(_T("SmDerivSurfDefinition_TYPE")) ; }

} ; // end class SmDerivSurfDefinition


// Some standalone surface functions.

/*******************************************************************//**
PURPOSE: Compute 2nd derivative of CrvOnSurf(s) = S(C(s)) 
         using the chain rule given C'(s) and C''(s) 
         and all the S(u,v) 1st and 2nd derivatives .

NOTES:

RETURN: directional 2nd derivative 3d vector
***********************************************************************/
SM_EXPORT SmVector3d smsurf_LiftSecondDerivative( const SmVector3d & crCs,     // in : Curve 1st deriv           
                                                  const SmVector3d & crCss,    // in : Curve 2nd deriv           
                                                  const SmVector3d & crDU,     // in : Surface 1st U deriv       
                                                  const SmVector3d & crDV,     // in : Surface 1st V deriv       
                                                  const SmVector3d & crDUU,    // in : Surface 2nd UU deriv      
                                                  const SmVector3d & crDUV,    // in : Surface 2nd UV deriv      
                                                  const SmVector3d & crDVV) ;  // in : Surface 2nd UW deriv      

/*******************************************************************//**
PURPOSE: Compute 3rd derivative of CrvOnSurf(s) = S(C(s)) 
         using the chain rule given C'(s), C''(s), and C'''(s) 
         and all the S(u,v,w) 1st, 2nd, and 3rd derivatives .

NOTES:  

RETURN: directional 3rdderivative 3d vector
***********************************************************************/
SM_EXPORT SmVector3d smsurf_LiftThirdDerivative( const SmVector3d & crCs,       // in : Curve 1st deriv             
                                                 const SmVector3d & crCss,      // in : Curve 2nd deriv             
                                                 const SmVector3d & crCsss,     // in : Curve 3rd deriv             
                                                 const SmVector3d & crSu,       // in : Surface 1st U deriv         
                                                 const SmVector3d & crSv,       // in : Surface 1st V deriv         
                                                 const SmVector3d & crSuu,      // in : Surface 2nd UU deriv        
                                                 const SmVector3d & crSuv,      // in : Surface 2nd UV deriv        
                                                 const SmVector3d & crSvv,      // in : Surface 2nd UW deriv        
                                                 const SmVector3d & crSuuu,     // in : Surface 3rd UUU deriv       
                                                 const SmVector3d & crSuuv,     // in : Surface 3rd UUV deriv       
                                                 const SmVector3d & crSuvv,     // in : Surface 3rd UVV deriv       
                                                 const SmVector3d & crSvvv) ;   // in : Surface 3rd VVV deriv       

/*******************************************************************//**
PURPOSE: Get or create SmTrimSrfCache for this surface whether
         it has a SmFace m_pOwner or not. 

NOTES: The returned SmSurfaceCache object is placed into the global cache stacks. 
***********************************************************************/
SM_EXPORT SmSurfaceCache* smsurf_GetSurfaceCache(const SmSurface *cpSurface) ;

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
// Convert STEP derivatives computed per radian into derivatives per degree for
// the angular parameters (U and/or V). Entry [i*(lHighestVDeriv+1)+j] is the
// (i,j) derivative and is scaled by (PI/180)^i for an angular U and (PI/180)^j
// for an angular V, so derivatives match the degree-valued STEP parameters.
// Only the pure derivatives and the triangular mixed derivatives authored by
// the STEP evaluators are scaled; unused rectangular-array slots are untouched.
SM_EXPORT void smsurf_ScaleSTEPAngularDerivatives( SmVector3d * aDerivatives,
                                                   ULONG        lHighestUDeriv,
                                                   ULONG        lHighestVDeriv,
                                                   SmBoolean    bUIsAngular,
                                                   SmBoolean    bVIsAngular );

SM_EXPORT SmStatus smsurf_EvaluateGeometric( SmVector3d aEval[3][3],                    // in : surface evaluation matrix:
                                                                                        //    : [0][0] = position, [0][1] = d/dv, [0][2] = d^2/dvv
                                                                                        //    : [1][0] = d/du,     [1][1] = d^2/duv
                                                                                        //    : [2][0] = d^2/duu,                 [2][2] = SurfaceNormal
                                             double     & rdGaussianCurvature,          // out: GaussianCurvature = K1*K2 (principal curvatures)              
                                             double     & rdNormalCurvature,            // out: MeanCurvature = 1/2(K1 + K2)                                  
                                             double     & rdPrincipleCurvature1,        // out: 1st principal curvature value, K1                             
                                             double     & rdPrincipleCurvature2,        // out: 2nd principal curvature value, K2                             
                                             SmVector3d & rEFGOfFirstFundForm,          // out: 1st fundamental form, [E, F, G]                               
                                             SmVector3d & rLMNOfSecondFundForm,         // out: 2nd fundamental form, [L, M, N]                               
                                             SmVector3d & rPrincipleCurvatureVector1,   // out: 3d unit-vector tangent to surface in 1st principal direction  
                                             SmVector3d & rPrincipleCurvatureVector2) ; // out: 3d unit-vector tangent to surface in 2nd principal direction  

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SM_EXPORT SmStatus smsurf_EvaluateNormalSection( SmVector3d aEval[3][3],                   // in : Surface Point, its 1st and 2nd Derivatives, and SurfNorm           
                                                 const SmVector3d & crDirection,           // in : Direction specifying the crossSection Direction, non-unit is OK.   
                                                 SmVector3d & rTangentPlaneDirection,      // out: Surface unit-Tangent in CrossSection Direction                     
                                                 double & rdCurvatureValue) ;              // out: Surface Curvature in CrossSection Direction                        

/*******************************************************************//**
PURPOSE: Check whether two surfaces have the same shape at a point.

NOTES: Returns True if surface evaluations match position, tangent plane,
   and two normal curvatures.
***********************************************************************/
SM_EXPORT SmStatus smsurf_CheckShapeMatch( SmVector3d sPVV0[3][3],     // in : First surface evaluation matrix, two derivatives.   
                                           SmVector3d sPVV1[3][3],     // in : same for other surface                              
                                           double d3dTol,              // in : 3d tolerance                                        
                                           double dAngTolRad,          // in : angle tolerance, radians                            
                                           SmBoolean &rbMatch) ;       // out:                                                     

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SM_EXPORT SmStatus smsurf_DropVectors( const SmVector3d & sDU,                // in : Surface U_dir tangent                                                         
                                       const SmVector3d & sDV,                // in : Surface V_dir tangent                                                         
                                       ULONG lNumVectorsToDrop,               // in : Number of vectors to drop                                                     
                                       const SmVector3d * caVectorsToDrop,    // in : vectors to drop, sized:[lNumVectorsToDrop]                                    
                                       SmVector2d * aResultingUVVectors) ;    // out: resulting UVVectors sized:[lNumVectorsToDrop] where                           
                                                                              //    : DropVec3D = u*rDU + v*rDV                                                     
                                                                              //    : DropVec3D = vector generated by droping input vec to rDU/rDV plane            
                                                                              //    : when rDU or rDV are very close to zero - they are replaced by unitized vectors.

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SM_EXPORT SmLocalSurfaceType smsurf_ComputeSurfacePointType( const SmVector3d  & crNormal,                 // in : surface Normal                                                
                                                             const SmVector3d  & crDUU,                    // in : Surface U dir 2nd derivative                                  
                                                             const SmVector3d  & crDVV,                    // in : Surface V dir 2nd derivative                                  
                                                             SmSurfaceSideType * pOptSurfSideDUU=NULL,     // out: U Dir isoParameter curvature orientation                      
                                                                                                           //    : SM_SS_ON    = isoParam curve is locally planar,               
                                                                                                           //    : SM_SS_ABOVE = isoParam curve bends up towards normal vector,  
                                                                                                           //    : SM_SS_BELOW = isoParam curve bend down away from Normal vector
                                                             SmSurfaceSideType * pOptSurfSideDVV = NULL) ; // out: V Dir isoParameter curvature orientation                      

/*******************************************************************//**
PURPOSE: Given a surface uv position and a 3d space point, determine
   whether the point is on the surface normal.

USAGE NOTES:
   1. For testPoints more than unit distant from the surface (as measured
   by the project distance onto the test surfNorm line) the AppoxTol is
   scaled by the dist to the surface.  
   
   So ApproxTol for points remote from the surface really specifies
   an angle.  It's the angle made between the SurfNorm and a testPoint that
   is exactly ApproxTol distance away from the line at exactly one unit
   away from the surface.  Any TestPoint within the cone centerd on the surfNorm
   line and this angle is considered IsPointOnNormal.

     dScaledTolerance = dDistToSurf > 1 ? (dDistToSurf * dApproxTol) : dDistToSurf ;
   
   2. When a surface uv domain is passed in, the test2D point is clamped to it.
   This might be good, because in this function, a test3D point outside the
     surface natural domain will never be classified as good.
***********************************************************************/
SmStatus smsurf_IsPointOnNormal(const SmSurface  & crSurface,                  // in : target surface                                                               
                                const SmExtent2d * cpUVDomain,                 // in : clamp domain: if present, clamp crUVToTest to this domain.                   
                                                                               //    : NULL to ignore                                                               
                                const SmPoint2d  & crUVToTest,                 // in : Surface test 2Dpoint                                                         
                                const SmPoint3d  & cr3DPointToTest,            // in : input   test 3DPoint                                                         
                                const SmVector3d * cp3D1stDeriv,               // in : associated tangent direction for cr3DPointToTest                             
                                SmBoolean        & rbFromLeftU,                // out: TRUE=eval srf u from left at aSrfSpanEndUV[0], else FALSE                    
                                SmBoolean        & rbFromLeftV,                // out: TRUE=eval srf v from left at aSrfSpanEndUV[0], else FALSE                                       
                                double             dApproxTol,                 // in : Max dist allowed between unique points                                       
                                SmBoolean        & rbGoodPoint,                // out: TRUE=TestPoint within tol of SurfaceNormal line, FALSE=not                   
                                SmStatus         & reReasonIsBad,              // out: SM_ERR_OUTSIDE_OF_DOMAIN    = input testPoint is not within domain           
                                                                               //    : SM_ERR_BAD_SURFACE_POINT    = Surface.Evaluate() fails - testing impossible  
                                                                               //    : SM_ERR_NOT_WITHIN_TOLERANCE = test run - Point not on normal                 
                                                                               //    : SM_ERR_UNKNOWN              = test run - Point is on normal                  
                                double           & rdDropToSurf,               // out: when rbGoodPoint == TRUE, set to SurfPt/TestPt dist                          
                                                                               //    :   rbGoodPoint == FALSE, set to 0.0                                           
                                double           & rdApproxDev) ;              // out: deviation between 3dPoint and srfNormalLine                                  

#endif // __SMSURFACE_H__
