// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfaceIntersector.h
* PURPOSE: Header file for SmSurfaceIntersector object.
**********************************************************************/

#ifndef __SMSURFACEINTERSECTOR_H__
#define __SMSURFACEINTERSECTOR_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMGLOBALSOLVER_H__
#include <SmGlobalSolver.h>
#endif

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

#ifndef __SMTLIST_H__
#include <SmTList.h>
#endif


#ifndef __SMSURFACECACHE_H__
#include <SmSurfaceCache.h>
#endif

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif
   
#define SM_MAX_USER_DOUBLES 4


/*******************************************************************//**
PURPOSE: This enum defines the type of an intersection point 
     (SmTsectPnt). 

NOTES:
  An IntersectionPoint = a point in 3dSpace where two surfaces
                         coincide. Each IntersectionPoint is associated
                         with 2 SurfacePoints, one on each surface.

  The surf/surf intersections in the neighborhood of an intersection point are
  determined by the relative orientation and curvature of the surfaces
  at the intersection point.

  At an intersection point When surfacePoint->Normals are not parallel,
    the intersection type is SM_IP_CROSSING and there are two rays of intersections
    emitting from the intersection points in opposite directions. The direction
    is equal to the crossProduct of the two surface normals.

  At an intersection point When surfacePoint->Normals are parallel to one another there
    are 4 different intersection cases distinguished by how the
    surfaces intersect one another in the neighborhood of the
    intersection point.  Intersection cases are determined by
    by the relative surfacePoint->curvature properties between the
    two surfaces. The cases are:

 SM_IP_TANGENT_POINT, example: a bowl just kissing a plane 
                            - no intersectionPoint in point neighborhood 
 SM_IP_TANGENT_CURVE, example: cylinder touching a plane  
                            - 2 rays of intersections emit from point in opposite directions 
 SM_IP_SINGULARITY,   example: a saddle intersecting a plane  
                            - 4 rays of intersections emit from point along 
                              two asymptotic directions in opposite directions
                      example: cylinder tangent with a torus along an entire circle
                            - 6 rays of intersections emit from point closest to torus center 
                               - 2 for for the tangent circle and 
                               - 2 more for each of 2 intersection 'lobes' that start and start 
                                   at this intersection singularity point.
 SM_IP_COINCIDENCE    The surface points have the same position, surfaceNormal,
                               and curvatures at the intersection point. 
                      example: any two coincident surfaces.
                            - every point in the neighborhood of the intersection point is also an intersection 
***********************************************************************/
enum SmIntersectionPointType {
  SM_IP_UNDEFINED,     // clear value for deleted objects
  SM_IP_UNKNOWN,       // Unknown intersection point
  SM_IP_CROSSING,      // Surfaces have normal crossing intersection at this point
  SM_IP_TANGENT_POINT, // Singular tangent point (surfaces touch at this point)
  SM_IP_TANGENT_CURVE, // Point is on tangent curve
  SM_IP_SINGULARITY,   // Singular Intersection point at nexus of 4 or 6 intersection curves
  SM_IP_COINCIDENCE,   // Point on surfaces where second fundamental forms match
  SM_IP_TOUCHING       // Point is on the natural boundaries of two surfaces so only one point of xsect exists
} ;

// note 1: don't change the order of these values. Some code depends
//         on the relative rank of the different IntersectionPointTypes.

// note 2: watch out for confusion on the word 'SINGULARITY' it's used by SMLib in two ways.
//         1. For surface representations, it's used for points of surface degeneracy where an
//             entire curve of surface points map to one 3d point, e.g. the poles of a sphere, the
//             degenerate edge of a BSplineSurface collapsed down to a triangle, or the tip of a cone.
//         2. For intersections, it's used for intersection points that are the nexus of multiple intersection curves.
//         There is no conceptual connection between surface and intersection singularities.
//            As such cases can be constructed in which intersection singularities can happen
//               free of or coincident with surface singularities and inversely surface singularities
//               can be free of or coincident with any kind of intersection type including crossing, 
//               tangentPoint, TangentCurve, Singular, and Coincident.

/*******************************************************************//**
PURPOSE: Provide the storage for an intersection point.

NOTES: static class object - don't add virtual methods to SmTsectPnt. 
***********************************************************************/
class SM_EXPORT SmTsectPnt : public SmListNode
{
public:
    SmIntersectionPointType  m_ePointType; // oneof:
                                           //  SM_IP_CROSSING,      - normal crossing intersection point
                                           //  SM_IP_TANGENT_POINT, - surfaces touch at point
                                           //  SM_IP_TANGENT_CURVE, - point on tangent curve
                                           //  SM_IP_SINGULARITY,   - point at nexus of 4 or 6 intersect curves
                                           //  SM_IP_COINCIDENCE    - matching 2nd fundamental forms

    SmExtentPointType m_eUVParamType[2][2] ; // m_eUVParamType[0][0] = Srf0 U Param Classification (pt is Umin, Umax or between)
                                             // m_eUVParamType[0][1] = Srf0 V Param Classification (pt is Vmin, Vmax or between) 
                                             // m_eUVParamType[1][0] = Srf1 U Param Classification (pt is Umin, Umax or between)
                                             // m_eUVParamType[1][1] = Srf1 V Param Classification (pt is Vmin, Vmax or between) 
public:

    SmVector2d  m_vUVCurvePV[2][2];    // 2d pos and tangent in each surface domain
                                       // [lsrf][0] = uv pos
                                       // [lsrf][1] = uv XSectCurve_tang_dir

    SmVector3d  m_vSurfacePV[2][3][3]; // evaluation matrices for both surfaces
                                       // [lsrf][0][0] = 3d pos
                                       // [lsrf][1][0] = du = tang_udir
                                       // [lsrf][0][1] = dv = tang_vdir
                                       // [lsrf][2][0] = duu
                                       // [lsrf][1][1] = duv
                                       // [lsrf][0][2] = dvv
                                       // [lsrf][2][2] = surf_norm

                                       // [lsrf][1][2] = Unused, uninitialized
                                       // [lsrf][2][1] = Unused, uninitialized

    SmVector3d  m_v3DCurvePV[2];       // 3d xsect curve pos, tang
                                       //   when used in fillets
                                       //    BaseSurface(uvPnt)   = points on rails
                                       //    OffsetSurface(uvPnt) = point on fillet center offsetSurf/Surf intersection curve

public:

    double      m_dCurveParameter;       // intersection curve parameter
    double      m_dTangentPlaneAngleRad; // angle between surface normals at intersection point in radians
    double      m_dDeviation;            // max dist to each surface at intersection point

    double      m_adUserDoubles[SM_MAX_USER_DOUBLES];
                    // used by different derived types differently at different times:
                    // class SmConstantDistanceFS
                    //     m_adUserDoubles[0] = surface offset distance 
                    //                          rail curves are xSect between surface and otherSurfaceOffsetSurface
                    // class SmVariableRadiusFS
                    //     m_adUserDoubles[0] = variable radius law parameter for this point
                    // stores Surface UV Points as [surf0.u surf0.v surf1.u surf1.v] in methods 
                    //         SmFilletSolver::PointOnPlaneSolve 
                    //         SmFilletSolver::RailRailIntersect   
                    //         SmFilletSolver::StepSolve
                    //         SmConstantRadiusAssistedFS::GuessPointOnPlaneSolve
                    //         SmConstantRadiusAssistedFS::PointOnPlaneSolve       
                    // stores curve currentParam and nextParam values as [currentParam nextParam] in methods
                    //         SmSurfaceIntersector::ComputeLawPointValues
                    //         SmSurfaceIntersector::SplitSpan

    void        * m_apUserPointer[2]; // NOT USED in SMLib - available for UserApplications

    // constructor, destructor
    SmTsectPnt();
    SmTsectPnt(const SmTsectPnt &crTsectPnt) ;
    SmTsectPnt & operator= (const SmTsectPnt &crTsectPnt) ;
    ~SmTsectPnt() { SmListNode::ReSet() ;   // clear Next/Prev pointers
                    m_ePointType            = SM_IP_UNDEFINED ; 
                    m_dCurveParameter       = SM_UNDEF_DOUBLE ;
                    m_dTangentPlaneAngleRad = SM_UNDEF_DOUBLE ;
                    m_dDeviation            = SM_UNDEF_DOUBLE ;
                    m_eUVParamType[0][0]    = SM_EP_UNKNOWN ;
                    m_eUVParamType[0][1]    = SM_EP_UNKNOWN ;
                    m_eUVParamType[1][0]    = SM_EP_UNKNOWN ;
                    m_eUVParamType[1][1]    = SM_EP_UNKNOWN ;
                  }

    // predicates and side effects

    // Return TRUE when either UVPoint sits on its surface's natural UVDomain boundary
    SmBoolean IsBounded() const ; 

    // Set m_eUVParamType with given uv points for given surfaces
    void ClassifyPointParams
    (
      SmPoint2d         aUVs[2],                ///< [in ]: Given UVPoints on SmSurfaceIntersector::m_cpSurface[0 and 1]               <br>
      const SmSurface * apSurface[2],           ///< [in ]: The SmSurfaceIntersector::m_cpSurface array                                <br>
      const SmExtent2d  aUVDomain[2],           ///< [in ]: Given target UVDomains for SmSurfaceIntersector::m_cpSurface[0 and 1]      <br>
      double            dTol3d = SM_EFF_ZERO    ///< [in ]: min distance between unique 3d points                                      <br>
    ) ;

    // Convenience access.  Note, these methods return references,
    // so they can be used to modify this class's data members.
    SmIntersectionPointType & PointType()       { return m_ePointType ; }
    SmExtentPointType & UParamType( ULONG lSrf) { return m_eUVParamType[lSrf][0]; } // lSrf UV Point U param is minU, maxU, or insideU
    SmExtentPointType & VParamType( ULONG lSrf) { return m_eUVParamType[lSrf][1]; } // lSrf UV Point V param is minV, maxV, or insideV 
    SmPoint2d  & UVPos    ( ULONG lSrf ) { return m_vUVCurvePV[lSrf][0]; }
    SmVector2d & UVDeriv  ( ULONG lSrf ) { return m_vUVCurvePV[lSrf][1]; }
    SmPoint3d  & SrfPos   ( ULONG lSrf ) { return m_vSurfacePV[lSrf][0][0]; }
    SmVector3d & SrfDu    ( ULONG lSrf ) { return m_vSurfacePV[lSrf][1][0]; }
    SmVector3d & SrfDv    ( ULONG lSrf ) { return m_vSurfacePV[lSrf][0][1]; }
    SmVector3d & SrfDuu   ( ULONG lSrf ) { return m_vSurfacePV[lSrf][2][0]; }
    SmVector3d & SrfDuv   ( ULONG lSrf ) { return m_vSurfacePV[lSrf][1][1]; }
    SmVector3d & SrfDvv   ( ULONG lSrf ) { return m_vSurfacePV[lSrf][0][2]; }
    SmVector3d & SrfNorm  ( ULONG lSrf ) { return m_vSurfacePV[lSrf][2][2]; }
    SmPoint3d  & CrvPos   (            ) { return m_v3DCurvePV[0]; }
    SmVector3d & CrvDeriv (            ) { return m_v3DCurvePV[1]; }
    SmVector3d * SrfEvalMatrix( ULONG lSrf ) { return &( m_vSurfacePV[lSrf][0][0] ); }

    // obsolete
    // void CopySrfEvalMatrix( ULONG lSrf, SmVector3d aMat[3][3] );

    void Dump() const ;
    SmDisplayList *Draw() const ;

} ; // end class SmTsectPnt

SM_TLIST_TEMPLATE_PREDECLARATION(SmTsectPnt) ;

/*******************************************************************//**
PURPOSE: The surface intersector class provides an interface and mechansim
    to intersect two surfaces.  This basic surface intersector only handles
    nicely behaved crossing intersections.  Tangency and singularities are
    handled by the advanced surface intersector which is a subclass of this
    object and available in the Advanced Surface Plug-In product.  

NOTES: Surface/surface intersection requires G1 surfaces.
***********************************************************************/
class SM_EXPORT SmSurfaceIntersector : public SmGlobalSolver
{
protected:
  const SmContext   * m_cpContext;
  const SmSurface   * m_cpSurface[2];                // source intersection surfaces
  SmExtent2d          m_vUVDomain[2];                // domain for each surface
  SmBoolean           m_bClosedU[2];                 // m_bClosedU[i] : TRUE = Surface[i] is closed in U direction (closed isoU parameter curves)
  SmBoolean           m_bClosedV[2];                 // m_bClosedV[i] : TRUE = Surface[i] is closed in V direction (closed isoV parameter curves)
  SmBoolean           m_bUseSurfaceEdges[2];         // TRUE = Use SurfaceBoundaryCurve/OtherSurface xSects to find StartPoints 
                                                     // FALSE= don't
                                                     // default:[TRUE]
  double              m_dThisAngTolRad;              // max change in xsect curve tangent direction
                                                     //     allowed between xsect curve sample points
  double              m_dThisApproxTol3d;            // max dist between through points and xsect curve
                                                     //     2*max dist between approx surfUV points/ and approx xsect curve elsewhere
  double              m_dSteppingParallelTolRadians; // Cutoff for tangent-surfaces case during stepping.   // GWCTangentChange
                                                     // See discussion in constructor.
  double              m_dStartParallelTolRadians;    // Cutoff for tangent-surfaces case for start points.  // GWCTangentChange
                                                     // See discussion in constructor 
  double              m_dCurveTraceDirection;        // +1 = trace in direction of crossProduct(surf0,surf1)
                                                     // -1 = trace in opposite direction 
  double              m_dLastThroughParmeter;        
  double              m_dExtensionDistance;          // max parameter distance for doing extensionClipping
  SmBoolean           m_bDoingExtensionClipping;     // TRUE = set m_dLastThroughParmeter to last through point
                                                     //        parameter value when checking for through points on xsect curves
  SmBoolean           m_bCurveIsClosed;              // TRUE = traced xsect curve was found to be closed
  SmBoolean           m_bCheckForInteriorCurves;     // NO Longer used: used to be
                                                     //    TRUE = skip search for intersections that don't start or stop on surface boundary curves             
  SmBoolean           m_bDoingSelfIntersection;
  SmBoolean           m_bClippedByThroughPoint;      // set to TRUE when a xsect curve segment hits a given Through point
  SmBoolean           m_bDoBoundaryPointOnCurveTest;
  SmMemBlockMgr       m_vTSPntMgr;                   // manages blocks or SmTsectPnts to be used in m_vCurvePoints
  SmTsectPnt*         m_pStartPoint;                 // location to start curve Trace
  SmTsectPnt*         m_pEndPoint;                   // location to end curve Trace
  SmTList<SmTsectPnt> m_vCurvePoints;                // xsect curve sample points - contains TsectPnts allocaed by m_vTSPntMgr
  SmTList<SmTsectPnt> m_vStartPoints;                // list of start points to start/stop curve traces
  SmTList<SmTsectPnt> m_vThroughPoints;              // list of points to include in curve trace
    
public:
  // constructor
  SmSurfaceIntersector
  (
    const SmSurface  & crSurface1,                      ///< [in ]: Target Surf 1                                                     <br>
    const SmExtent2d & crUVDomain1,                     ///< [in ]: Target Surf 1 Domain                                              <br>
    const SmSurface  & crSurface2,                      ///< [in ]: Target Surf 2                                                     <br>
    const SmExtent2d & crUVDomain2,                     ///< [in ]: Target Surf 2 Domain                                              <br>
    SmBoolean          bFromFilletIntersector = FALSE   ///< [in ]: TRUE = from FilletIntersector: use UVDomains as given             <br>
                                                        ///<      : FALSE= reduce given UVDomains to XSect(UVDomain,Surf->UVNatDom)   <br>
  );

  // destructor
  virtual ~SmSurfaceIntersector() {};

  double GetThisApproxTol3d() const { SM_ASSERT_TOL(m_dThisApproxTol3d) ; return m_dThisApproxTol3d; } 
  double GetThisAngTolRad  () const { SM_ASSERT_TOL(m_dThisAngTolRad) ; return m_dThisAngTolRad; } 

  SmStatus ComputeLawPointValues
  (
    double              dParameter,              ///< [in ]: Param for crPointCurve eval                                                           <br>
    const SmPoint2d   & crUV0,                   ///< [in ]: guess uv point on m_cpSurface[0]                                                      <br>
    const SmPoint2d   & crUV1,                   ///< [in ]: guess uv point on m_cpSurface[1]                                                      <br>
    double              dDeltaStep,              ///< [in ]: param increment used to estimate crPointCurve tangent direction                       <br>
    const SmVector3d    caNormals[2],            ///< [in ]: when bAverageNormals = TRUE, PlaneNormals at crPointCurve crPointCurveInterval bounds <br>
    SmBoolean           bAverageNormals,         ///< [in ]: TRUE = PlaneNormals(param) = LinearInterp(Param,Ivl) of caNormals values              <br>
                                                 ///<      : FALSE= PlaneNormals(param) = crPointCurve(Param) tangents                             <br>
    const SmFilletLaw & crLawCurve,              ///< [in ]: FilletLaw to compute fillet-radius for every Curve param                              <br>
    SmBoolean           bLawOrient,              ///< [in ]: bReverseOrientation: TRUE = parameters run from interval end to interval start        <br>
    const SmCurve     & crPointCurve,            ///< [in ]: Should be Arc Length Parameterized                                                    <br>
    const SmExtent1d  & crPointCurveInterval,    ///< [in ]: interval defining range of fillet edge                                                <br>
    double              dSurfaceOrientations[2], ///< [in ]: Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.                     <br>
    SmBoolean           bSkipComputingUVS,       ///< [in ]: TRUE = use rTSectPnt, UVPos(0), UVPos(1), and Param values                            <br>
                                                 ///<      : FALSE= refine rTSectPnt values to Surf/Surf/Plane xsect before using                  <br>
    SmTsectPnt        & rTsectPnt                ///< [in,out]: in : bSkipComputingUVS== TRUE  rTsectPnt.UVPos(0);                                 <br>
                                                 ///<      :                                rTsectPnt.UVPos(1);                                    <br>
                                                 ///<      :                                rTsectPnt.m_adUserDoubles[0];                          <br>
                                                 ///<      : out: bSkipComputingUVS== FALSE rTsectPnt.UVPos(0);                                    <br>
                                                 ///<      :                                rTsectPnt.UVPos(1);                                    <br>
                                                 ///<      :                                rTsectPnt.m_adUserDoubles[0];                          <br>
                                                 ///<      :      m_ePointType etc.                                                                <br>
  );

  virtual SmStatus ComputePointValuesDelta
  (
    SmPoint2d [2],  ///< [in ]: TgtParam      rail Surf UVPt values to evaluate                                      <br>
    SmPoint2d [2],  ///< [in ]: TgtDeltaParam rail Surf UVPt values to evaluate                                      <br>
    SmTsectPnt & ,  ///< [out]: container for TgtParam surface and Crv 3dPt, UVPos and derivative values             <br>
    SmTsectPnt &    ///< [out]: container for TgtDeltaParam surface and Crv 3dPt, UVPos and derivative values        <br>
  )
  { return SM_ERR; }

  SmStatus DoIntersection
  (
    const SmContext            & crContext,                 ///< [in ]: context for new object construction                                            <br>
    const SmApproxTol3d        * pdOptApproxTol3d,          ///< [in ]: specify dist tol, NULL = use m_dThisApproxTol3d value                          <br>
    const double               * pdOptAngTolRad,            ///< [in ]: specify ang tol,  NULL = use m_dThisAngTolRad    value                         <br>
    SmTArray<SmCurve*>         * p3DCurves,                 ///< [out]: 3dCurve intersections                                                          <br>
    SmTArray<SmCurve*>         * pSurface1UVCurves,         ///< [out]: associated UVTrimCurves on m_cpSurface[0]                                      <br>
    SmTArray<SmCurve*>         * pSurface2UVCurves,         ///< [out]: associated UVTrimCurves on m_cpSurface[1]                                      <br>
    SmTArray<SmTsectCurveType> * pCurveTypes,               ///< [out]: oneof for each 3dCurve                                                         <br>
                                                            ///<      : SM_TC_TOUCHING,       // Curve is a degenerate point which represents a        <br>
                                                            ///<      :                       // single point where surfaces touch.                    <br>
                                                            ///<      : SM_TC_CROSSING,       // Curve represents a crossing intersection where        <br>
                                                            ///<      :                       // surface normals are not parallel.                     <br>
                                                            ///<      : SM_TC_TANGENT,        // Curve represents a tangent curve where surfaces touch <br>
                                                            ///<      :                       // along a curve                                         <br>
                                                            ///<      : SM_TC_COINCIDENT,     // Curve represents a point where the two surfaces       <br>
                                                            ///<      :                       // are coincident.  Usually this curve corresponds to    <br>
                                                            ///<      :                       // a boundary curve from one of the surfaces.            <br>
                                                            ///<      :                       // Along this curve, both surfaces are contiguous,       <br>
                                                            ///<      :                       // have parallel surface normals, and have a             <br>
                                                            ///<      :                       // cross-tangent direction in which the surface          <br>
                                                            ///<      :                       // curvatures are the same.                              <br>
                                                            ///<      : SM_TC_NEAR_TANGENT,   // Curve has a relatively small angle of intersection    <br>
                                                            ///<      : SM_TC_REGION_BOUNDARY // Curve bounds a region, within which the surfaces      <br>
                                                            ///<      :                       // are coincident                                        <br>
    SmTArray<double>           * pDeviations                ///< [out]: associated max 3DCurve to Surface distance                                     <br>
  );

  virtual SmStatus EvaluateLawPoint
  (
    double              dParameter,              ///< [in ]: filletCurve parameter - used to compute current fillet radius                    <br>
    const SmPoint2d   & crUV0,                   ///< [in ]: guess uv point on m_cpSurface[0]                                                 <br>
    const SmPoint2d   & crUV1,                   ///< [in ]: guess uv point on m_cpSurface[1]                                                 <br>
    const SmFilletLaw & crLawCurve,              ///< [in ]: FilletLaw to compute fillet-radius for every Curve param                         <br>
    SmBoolean           bLawOrient,              ///< [in ]: bReverseOrientation: TRUE = parameters run from interval end to interval start   <br>
    const SmExtent1d  & crPointCurveInterval,    ///< [in ]: crCurveInterval = =interval defining range of fillet edge                        <br>
    double              dSurfaceOrientations[2], ///< [in ]: Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.                <br>
    const SmPoint3d   & rPlaneOrigin,            ///< [in ]: Origin of Plane(origin, normal)                                                  <br>
    const SmVector3d  & rPlaneNormal,            ///< [in ]: Normal of Plane(origin, normal)                                                  <br>
    double            & rdCurveParam,            ///< [out]: Curve Param                                                                      <br>
    SmPoint2d           sUVs[2]                  ///< [out]: Surf Params of m_pSurface[0]/m_pSurface[0]/plane XSect result                    <br>
  );
                                                
  SmStatus DoLawIntersection
  (
    const SmContext             & crContext,                     ///< NotUsed: [in ]:                                                                  <br>
    const SmTArray<SmTsectPnt*> & crTsectPoints,                 ///< [in ]:                                                                  <br>
    const SmFilletLaw           & crLawCurve,                    ///< [in ]:                                                                  <br>
    SmBoolean                     bLawOrient,                    ///< [in ]:                                                                  <br>
    const SmCurve               & crPointCurve,                  ///< [in ]: Should be Arc Length Parameterized                               <br>
    const SmExtent1d            & crPointCurveInterval,          ///< [in ]:                                                                  <br>
    double                        dSurfaceOrientations[2],       ///< [in ]: Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values.<br>
    SmBoolean                     bUseNormalPlaneAveraging,      ///< [in ]:                                                                  <br>
    SmBoolean                     bUniformSteps,                 ///< NotUsed: [in ]:                                                                  <br>
    const SmApproxTol3d         * pOptApproxTol3d,               ///< [in ]:                                                                  <br>
    const double                * pdOptAngTolRad,                ///< NotUsed: [in ]:                                                                  <br>
    SmBSplineCurve             *& rp3DCurve,                     ///< [out]:  Resulting curve - note that this will be NULL                   <br>
                                                                 ///<      : if we are unable to find a curve that satisfies                  <br>
                                                                 ///<      : the input requirements and passes through all given points.      <br>
    SmBSplineCurve             *& rpSurface1UVCurve,             ///< [out]:                                                                  <br>
    SmBSplineCurve             *& rpSurface2UVCurve,             ///< [out]:                                                                  <br>
    SmTsectCurveType            & reCurveType,                   ///< [out]:                                                                  <br>
    double                      & rdDeviation                    ///< [out]:
  );

  // find Surface/Surface intersection curve through 1, 2, or more given points
  SmStatus DoPointIntersection
  (
    const SmContext             & crContext,                   ///< [in ]: context for new object construction                           <br>
    const SmTArray<SmTsectPnt*> & crTsectPoints,               ///< [in ]: Points on the intersection                                    <br>
    SmBoolean                     bExtendBeforeStart,          ///< [in ]: If more than two points                                       <br>
                                                               ///<      : are specified this will allow extension of the                <br>
                                                               ///<      : intersection curve prior to the start point                   <br>
    SmBoolean                     bExtendAfterEnd,             ///< [in ]: If more than two points                                       <br>
                                                               ///<      : are specified this will allow the extension of the            <br>
                                                               ///<      : intersection curve after the last point.                      <br>
    const SmVector3d            * pOptStartDirection,          ///< [in ]: Specifies direction to start the intersection marching        <br>
    const SmVector3d            * pOptEndDirection,            ///< [in ]: Specifies direction to end the intersection marching          <br>
    const SmApproxTol3d         * pOptApproxTol3d,             ///< [in ]: specify dist tol, NULL = use m_dThisApproxTol3d value         <br>
    const double                * pdOptAngTolRad,              ///< [in ]: specify ang tol,  NULL = use m_dThisAngTolRad    value        <br>
    SmBSplineCurve             *& rp3DCurve,                   ///< [out]: Resulting 3Dcurve - note that this will be NULL               <br>
                                                               ///<      : if we are unable to find a curve that satisfies               <br>
                                                               ///<      : the input requirements and passes through all given points.   <br>
    SmBSplineCurve             *& rpSurface1UVCurve,           ///< [out]: Resulting UVTrimCurve on Surface1                             <br>
    SmBSplineCurve             *& rpSurface2UVCurve,           ///< [out]: Resulting UVTrimCurve on Surface2                             <br>
    SmTsectCurveType            & reCurveType,                 ///< [out]: specify the kind of intersection curve found                  <br>
    double                      & rdDeviation                  ///< [out]: max distance from through points to source surfaces           <br>
  );
                                                   
  SmStatus FindBoundaryStartPoints
  (
    const SmExtent3d           & crIntersectionBox,            ///< [in ]: bound on surface/surface intersections                      <br>
    SmTArray<SmCurve*>         & r3DCurves,                    ///< [in,out]: currently found intersections                            <br>
                                                               ///<      : augmented whenever a boundary curve                         <br>
                                                               ///<      : has a coincident range with the other surface               <br>
    SmTArray<SmCurve*>         & rSurface1UVCurves,            ///< [out]: associated Surface1 UVTrimCurves                            <br>
    SmTArray<SmCurve*>         & rSurface2UVCurves,            ///< [out]: associated Surface2 UVTrimCurves                            <br>
    SmTArray<SmTsectCurveType> & rCurveTypes,                  ///< [out]: associated Curve Type: oneof                                <br>
                                                               ///<      : SM_TC_TOUCHING                                              <br>
                                                               ///<      : SM_TC_CROSSING                                              <br>
                                                               ///<      : SM_TC_TANGENT                                               <br>
                                                               ///<      : SM_TC_COINCIDENT                                            <br>
                                                               ///<      : SM_TC_NEAR_TANGENT                                          <br>
                                                               ///<      : SM_TC_REGION_BOUNDARY                                       <br>
    SmTArray<double>           & rDeviations                   ///< [out]: associated deviations                                       <br>
  );

  SmStatus DoBoundaryPlaneTest
  (
    SmPatchBoundaryBoundingPlanes & rBPlanes1,                 ///< [in ]: BoundaryPlanes from SurfaceCache1                                     <br>
    SmPatchBoundaryBoundingPlanes & rBPlanes2,                 ///< [in ]: BoundaryPlanes from SurfaceCache2                                     <br>
    double                          d3DTol,                    ///< [in ]: max distance between distinct intersection points                     <br>
    SmExtent3d                    & rBBox,                     ///< [in ]: intersection bounding box limit                                       <br>
    SmTArray<SmCurve*>            & r3DCurves,                 ///< [out]: augmented with bndryCrv/bndryCrv xSects for grazind cases             <br>
    SmTArray<SmTsectCurveType>    & rCurveTypes,               ///< [out]:                                                                       <br>
    SmTArray<double>              & rDeviations,               ///< [out]:                                                                       <br>
    SmBoundaryPlaneTestResult     & rTestResult                ///< [out]: oneof SM_BR_GRAZE one pair of boundary curve bounding planes is       <br>
                                                               ///<      :               coincident, and the surfaces are on the opposite        <br>
                                                               ///<      :               sides of this plane                                     <br>
                                                               ///<      :   SM_BR_DISJOINT The surfaces are definitely disjoint: one pair of    <br>
                                                               ///<      :               boundary curve planes are parallel, and the surfaces    <br>
                                                               ///<      :               are OFF the region enclosed by the parallel planes.     <br>
                                                               ///<      :   SM_BR_UNKNOWN  Intersection possible: more work needed              <br>
  );

  const SmSurface *GetSurface(ULONG lIndex) const { return m_cpSurface[lIndex] ; }

  SmStatus GrazingBoxProcess
  (
    const SmContext            & crContext,                   ///< [in ]: context for new object construction                    <br>
    const SmExtent3d           & crIntersectionBBox,          ///< [in ]: Bounding Box containing intersection                   <br>
    const SmExtent2d           & crUVDomain1,                 ///< [in ]: Surface0 UVDomain containing intersection              <br>
    const SmExtent2d           & crUVDomain2,                 ///< [in ]: Surface1 UVDomain containing intersection              <br>
    SmBoolean                  & rbNoFurtherWorkNeeded,       ///< [out]: TRUE = Send to general Surf/Surf intersector           <br>
                                                              ///<      : FALSE= All done - no intersections                     <br>
    SmTArray<SmCurve*>         * p3DCurves,                   ///< [out]: 3D intersection curves                                 <br>
    SmTArray<SmCurve*>         * pSurface1UVCurves,           ///< [out]: associated Surface1 Curve                              <br>
    SmTArray<SmCurve*>         * pSurface2UVCurves,           ///< [out]: Associated Surface2 Curve                              <br>
    SmTArray<SmTsectCurveType> * pCurveTypes,                 ///< [out]: associated surf/surf intersection type                 <br>
    SmTArray<double>           * pDeviations                  ///< [out]: intersection deviation                                 <br>
  );

  SmStatus ReverseCurveDirection();

  // Trace one xsect curve from startPoint storing a sequence of SmTsectPnts in m_vCurvePoints.
  SmStatus TraceCurve
  (
    SmTsectPnt         & rStartPoint,      ///< [in ]: Location to start next xsect curve trace                <br>
    SmTArray<SmCurve*> & r3DCurves         ///< [in ]: accumulation of all xsect curves used to                <br>
                                           ///<      : prevent tracing out an already found intersection.      <br>
  );

  SmStatus StepBackFromSingularity
  (
    SmSurfParamType eSingDir,             ///< [in ]:                                                     <br>
    ULONG           lSurface,             ///< [in ]:                                                     <br>
    ULONG           bDirection,           ///< [in ]: 0 - step off rPrevPnt 1 - step off rCurrPnt         <br>
    SmTsectPnt    & rPrevPnt,             ///< [in ]:                                                     <br>
    SmTsectPnt    & rCurrPnt,             ///< [in ]:                                                     <br>
    double          dStepSize             ///< [in ]:
  );

  virtual SmStatus FlushCurve
  (
    SmTArray<SmCurve*>         & r3DCurves,                 ///< [out]: 3D intersection curve
    SmTArray<SmCurve*>         & rSurface1UVCurves,         ///< [out]: 1st surface UVTrimCurve                                                                          <br>
    SmTArray<SmCurve*>         & rSurface2UVCurves,         ///< [out]: 2nd surface UVTrimCurve                                                                          <br>
    SmTArray<SmTsectCurveType> & rCurveTypes,               ///< [out]: oneof: SM_TC_TOUCHING   = 2 surfs touch at 1 pt                                                  <br>
                                                            ///<      :        SM_TC_CROSSING   = 2 surfs xsect along curve; surf normals are not parallel               <br>
                                                            ///<      :        SM_TC_TANGENT    = 2 surfs xsect along curve tangent to one another                       <br>
                                                            ///<      :        SM_TC_COINCIDENT = 2 surfs are contiguous and share normals and cross-tangents along curve<br>
                                                            ///<      :        SM_TC_NEAR_TANGENT = curve has small angle of xsect                                       <br>
                                                            ///<      :        SM_TC_REGION_BOUNDARY = curve bounds a region within which surfs are coincident           <br>
    SmTArray<double>           & rDeviations                ///< [out]: max 3DCurve through point deviation from source surfaces                                         <br>
                                                            ///<      : NOTE: the deviation between through points may be larger than this value.                        <br>
  );

  virtual SmStatus FindInteriorCurves
  (
    SmTArray<SmCurve*>         & r3DCurves,                  ///< [out]: 3d intersection curve array                            <br>
    SmTArray<SmCurve*>         & rSurface1UVCurves,          ///< [out]: associated UVTrimCurves on Surface 1                   <br>
    SmTArray<SmCurve*>         & rSurface2UVCurves,          ///< [out]: associated UVTrimCurves on Surface 2                   <br>
    SmTArray<SmTsectCurveType> & rCurveTypes,                ///< [out]: associated CurveType, oneof:                           <br>
                                                             ///<      : SM_TC_TOUCHING,                                        <br>
                                                             ///<      : SM_TC_CROSSING,                                        <br>
                                                             ///<      : SM_TC_TANGENT,                                         <br>
                                                             ///<      : SM_TC_COINCIDENT,                                      <br>
                                                             ///<      : SM_TC_NEAR_TANGENT,                                    <br>
                                                             ///<      : SM_TC_REGION_BOUNDARY                                  <br>
    SmTArray<double>           & rDeviations                 ///< [out]: associated max distance between 3DCurve and surfaces   <br>
  );

  SmStatus ComputeStepSize
  (
    SmTsectPnt & rTsectPnt,                                 ///< NotUsed: [in ]: last xsect point          <br>
    double       dOldStepSize,                              ///< [in ]: last step size            <br>
    double     & rdNewStepSize                              ///< [out]: next step size            <br>
  );

  virtual SmStatus ComputePointValues
  (
    SmPoint2d    aUVValues[2],                               ///< [in ]: UV values on each surface for point                                  <br>
    SmTsectPnt & rTsectPnt,                                  ///< [out]: TsectPnt 3D and UV pos, tangent, and type values                     <br>
    SmTsectPnt * pOptPreviousPnt = NULL,                     ///< [in ]: Last intersection point on curve being stepped out, NULL to ignore   <br>
                                                             ///<      : When supplied used to handle singularity cases and                   <br>
                                                             ///<      : insure continuity of trace direction.                                <br>
    double     * pdStepSize = NULL                           ///< [in ]: distance to step back from singularities to try and find             <br>
                                                             ///<      : a nearby neighbor to use to computePointValues, NULL to ignore       <br>
  );

  virtual SmStatus ComputeSingularityPoint                   ///< currently just a stub functions <br>
  (
    SmPoint2d    aUVValues[2],                               ///< [in ]: Surface intersection UVPoints                         <br>
    SmTsectPnt & rTsectPnt,                                  ///< [out]: SurfacePoint values                                   <br>
    SmTsectPnt * pOptPreviousPnt                             ///< [in ]: last successfully classified intersectionCurve Point  <br>
  );

  virtual SmStatus ComputeNextPoint
  (
    SmTsectPnt & rTsectPnt,                                  ///< [in ]: last computed xSect point positions and tangents          <br>
    double       dStepSize,                                  ///< [in,out]: size for next step (reduced when a boundary is hit)    <br>
    SmTsectPnt & rNextTSP,                                   ///< [out]: next xsect point                                          <br>
    SmBoolean  & rbFoundGoodPoint,                           ///< [out]: TRUE = Found next point passes angle and dist checks      <br>
                                                             ///<      : FALSE= doesn't                                            <br>
    double     & rdDeviationFound,                           ///< [out]: max approx_3Dcurve/approx_surfTrimCurve dist              <br>
                                                             ///<      : of 5 test points between TsectPnt and NextTSP             <br>
                                                             ///< [out]: angleBetween(TsectPnt.3DTangent, NextTSP.3DTangent)       <br>
    double     & rdAngleFoundRad,                            ///< [out]: TRUE=proposed stepSize was reduced to                     <br>
    SmBoolean  & rbClipped,                                  ///<      : force NextTSP to be in both Surfaces                      <br>
                                                             ///< [out]: TRUE=TsectPnt is already on a boundary                    <br>
    SmBoolean  & rbBoundaryHit                               ///<      : and the NextTSP direction steps off one of the surfaces.  <br>

  );

  // Refine a near surf/surf xSect Guess to a surf/surf intersection
  SmStatus RefinePoint
  (
    const SmPoint2d        & sUV0,                ///< [in ]: m_cpSurface[0] guess point                                       <br>
     const SmPoint2d        & sUV1,               ///< [in ]: m_cpSurface[1] guess point                                       <br>
     SmIntersectionPointType  eIPType,            ///< [in ]: Expected Type of surf/surf intersection                          <br>
     const SmVector3d       * cpOptPlaneNormal,   ///< [in ]: if given, Force Solution to plane [guessAverage, Normal]         <br>
     const SmVector2d       * cpOptPreviousUV,    ///< [in ]: if given, prevent stepping back to same solution                 <br>
     SmBoolean                bDoBoundaryTesting, ///< [in ]: TRUE = Look for Surface/SurfaceBoundary intersections            <br>
     SmBoolean              & rbFoundPoint,       ///< [out]: TRUE = refinement succeeded                                      <br>
     SmVector2d               sUVs[2]) ;          ///< [out]: refined UVPoints clamped to Surface Boundaries                   <br>
    
  SmStatus AddPointToCurve(SmTsectPnt & rNextTSP);

  SmStatus AddStartPoint(SmTsectPnt & rStartTSP);

  SmStatus AddThroughPoint(SmTsectPnt & rStartTSP);

  void SetExtensionDistance(double dExtensionDistance) { m_dExtensionDistance = dExtensionDistance; }

  SmStatus SplitSpan
  (
    const SmTsectPnt  * pCurr,                         ///< [in ]:                  <br>
    const SmTsectPnt  * pNext,                         ///< [in ]:                  <br>
    double              dDeltaStep,                    ///< [in ]:                  <br>
    const SmVector3d    caNormals[2],                  ///< [in ]:                  <br>
    SmBoolean           bAverageNormals,               ///< [in ]:                  <br>
    const SmFilletLaw & crLawCurve,                    ///< [in ]:                  <br>
    SmBoolean           bLawOrient,                    ///< [in ]:                  <br>
    const SmCurve     & crPointCurve,                  ///< [in ]:                  <br>
    const SmExtent1d  & crPointCurveInterval,          ///< [in ]:                  <br>
    double              dSurfaceOrientations[2],       ///< [in ]:                  <br>
    SmTsectPnt       *& pNewTsect                      ///< [out]:                  <br>
  );

  SmStatus TestPoint
  (
    SmTsectPnt & rNextTSP,          ///< [in,out]: point to test - gets replaced by through/end points when appropriate    <br>
    SmBoolean & rbDone              ///< [out]: TRUE = NextTSP is now one of the m_pStartPoint                             <br>
                                    ///<      : or the m_pEndPoint                                                         <br>
                                    ///<      :    or one of the m_pThroughPoints when m_bDoingExtensionClipping == TRUE   <br>
                                    ///<      :    or rNextTSP.m_ePointType == SM_IP_SINGULARITY                           <br>
                                    ///<      :    or point is on a domain boundary                                        <br>
                                    ///<      :       and pushing outward (leaving domain).                                <br>
  );

  virtual SmStatus TestSpanAccuracy
  (
    SmTsectPnt & rTsectPnt,               ///< [in ]: last trace point                                        <br>
    SmTsectPnt & rNextTSP,                ///< [in ]: current trace point                                     <br>
    SmTsectPnt * pOptMidPnt,              ///< [out]: pOptMidPoint = NOT USED                                 <br>
    SmBoolean  & rbFoundGoodPoint,        ///< [out]: TRUE  = next point passes both angle and dist checks    <br>
                                          ///<      : FALSE =    angleFoundRad > m_dThisAngTolRad             <br>
                                          ///<      :         or max Curve/Surf dist > m_dThisApproxTol3d/2.0 <br>
    double     & rdDeviationFound,        ///< [out]: max approx_3Dcurve/approx_surfTrimCurve dist            <br>
                                          ///<      : of 5 test points between TsectPnt and NextTSP           <br>
    double     & rdAngleFoundRad          ///< [out]: angleBetween(TsectPnt.3DTangent, NextTSP.3DTangent)     <br>
  );

  SmBoolean IsPointOnCurve
  (
    const SmPoint3d          & crPointToTest,         ///< [in ]: point to test                         <br>
    const SmTArray<SmCurve*> & cr3DCurves,            ///< [in ]: curves to test                        <br>
    ULONG                    * pOptSkipIndex = NULL   ///< [in ]: skip this curve, NULL to ignore       <br>
  ) const;

  SmBoolean IsPointOnStartPoint
  (
    const SmTsectPnt          & crPointToTest,          ///< [in ]: point to test                        <br>
    const SmTList<SmTsectPnt> & crStartPoints,          ///< [in ]: StartPoints to test                  <br>
    ULONG                     * pOptSkipIndex = NULL    ///< [in ]: skip this index, NULL to ignore      <br>
  ) const ;

    
  SmStatus ClassifyBoundaryStartPointIntersection
  (
    SmTsectPnt &pTSP,                     ///< [in ]: Target Point with UVs, XSectCurvePoint, and SurfacePoint data        <br>
    SmBoolean  &bIsDegenerate             ///< [out]: TRUE = intersection point is not part of an intersection curve       <br>
                                          ///<      :        due to tangent intersections or boundary effects              <br>
                                          ///<      : FALSE= Point is part of an intersection curve                        <br>
  ) ;

  void SetUseSurfaceEdges(SmBoolean bUseSurface1Edges, SmBoolean bUseSurface2Edges) 
  { m_bUseSurfaceEdges[0] = bUseSurface1Edges; m_bUseSurfaceEdges[1] = bUseSurface2Edges; }

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmSurfaceIntersector, SmGlobalSolver, SmSurfaceIntersector_TYPE) ;

} ; // end class SmSurfaceIntersector

#endif // __SMSURFACEINTERSECTOR_H__
