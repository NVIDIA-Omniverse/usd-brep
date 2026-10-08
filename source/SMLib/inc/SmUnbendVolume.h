// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmUnbendVolume.h
* PURPOSE: Header file for the Unbend Volume class.
* Oct 2006 GWC Author
**********************************************************************/

#ifndef __SMUNBENDVOLUME_H__
#define __SMUNBENDVOLUME_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#include <SmVolume.h>

//      #ifndef __SMTARRAY_H__
//      #include <SmTArray.h>
//      #endif

class SmBSplineCurve ;
class SmDisplayList ;

/*******************************************************************//**
PURPOSE: This derived SmVolume object represents an Unbend Mapping to mimic
  the behavior of flattening a piece of sheet metal once bent in a break press.

NOTES: Unbending consists of sending the points of an object's
  shape through the SmUnbend's 3d InSpace to 3d OutSpace mapping.

  The SmUnbendVolume class implements the SmVolume virtual methods that define the
    ParamSpace -> ProjSpace
  piece of the SmVolume's general mapping of
    InSpace -> ParamSpace -> ProjSpace -> OutSpace,

  It's the reverse effect of the SmBendVolume object.

 The SmUnbendVolume is derived from the SmVolume class and is
 defined by the same information, an unbend NeutralDist and an unbend
 unit-normal coordinate system.

  o. The UnbendOrigin OutSpace point is an point on the unbend's axis.
  o. The UnbendAxis OutSpace vector points along the length of the unbend.
  o. The UnbendMidDir OutSpace vector points to the symmetric center of the unbend from the UnbendAxis.
       In other words, the unbend mapping is symmetric about the plane spanned by the
       UnbendAxis and the UnbendMidDir vectors.
  o. A UnbendBinormal OutSpace direction is defined as UnbendBinormal = UnbendAxis * UnbendMidDir.
  o. The Unbend's OutSpace NeutralDistance defines the distance from the UnbendAxis to the unbend's neutral plane.
  o. The Unbend's OutSpace NeutralPlane is defined as the plane normal to the UnbendMidDir
       and offset from the UnbendOrigin by the UnbendNeutralDistance.

  The Unbend mapping is defined over all space with a 1/2 infinite discontinuity plane
  V = 0 and U < 0 and a singularity along the W axis.
  If we worked in cylindrical coordinates the Unbend domain would be a bounding box defined by
    r > 0, singular at r = 0
    Pi > Theta > -Pi, double valued at the boundaried Theta=[-Pi, Pi] same as 1/2 plane V=0 and U<0
    Z unbounded

  In ParamSpace, the unbend is built about the ParamSpace coordinate system as,

    ParamSpace UnbendOrigin    = ParamPt (0,0,0)
    ParamSpace UnbendAxis      = ParamVec(0,0,1)
    ParamSpace UnbendMidDir    = ParamVec(1,0,0)
    ParamSpace UnbendBinormal  = ParamVec(0,1,0)

  In InSpace, the unbend is oriented and positioned by the Orient() mapping
  between orthogonal coordinate systems so that,

    InSpace    UnbendOrigin    = GetUnbendOrigin()    (stored in m_pOrientMap->GetToDisp())
    InSpace    UnbendAxis      = GetUnbendAxis()      (stored in m_pOrientMap->GetToZAxis())
    InSpace    UnbendMidDir    = GetUnbendMidDir() (stored in m_pOrientMap->GetToXAxis())
    InSpace    UnbendBinormal  = UnbendAxis * UnbendMidDir

  In OutSpace, the unbend has the same orientation and location as it does in InSpace so that,

    OutSpace   UnbendOrigin    = GetUnbendOrigin()    (stored in m_pOrientMap->GetToDisp())
    OutSpace   UnbendAxis      = GetUnbendAxis()      (stored in m_pOrientMap->GetToZAxis())
    OutSpace   UnbendMidDir    = GetUnbendMidDir() (stored in m_pOrientMap->GetToXAxis())
    OutSpace   UnbendBinormal  = UnbendAxis * UnbendMidDir

 (The Unbend and Bend mappings have the same descriptions, positions, and orientations.)

 THE UNBEND MAP (Same as the SmBendVolume Map Inversion)

   In the InSpace->OutSpace Unbend mapping:
     o. InSpace circular arcs normal to and centered on the UnbendAxis map to OutSpace lines parallel to the UnbendBiNormal.
         - ArcLength change depends on the radius of the arc
           o. less    than NeutralDistance, ArcLength lengthens
           o. equal   to   NeutralDistance, ArcLength constant
           o. greater than NeutralDistance, ArcLength shortens
     o. InSpace lines parallel to the UnbendAxis                map to OutSpace lines parallel the UnbendAxis.
     o. InSpace lines normal to and intersecting the UnbendAxis map to OutSpace lines parallel to the UnbendBinormal.

  In the ParamSpace->ProjSpace portion of the unbend mapping.
    o. ParamSpace circular arcs normal to and centered on the W Axis map to ProjSpace lines parallel to the Y Axis.
    o. ParamSpace lines parallel to the W Axis          map to ProjSpace lines parallel to the Z Axis.
    o. ParamSpace normal to and intersecting the W Axis map to ProjSpace lines parallel to the X Axis.


  The bend mappings between InSpace (xIn,yIn,zIn), ParamSpace (u,v,w),
  ProjSpace (xProj,yProj,zProj), and OutSpace (xOut,yOut,zOut) spaces are
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Unbend Mapping    | FromSpace | ToSpace | mapping                                               |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | EvaluateSimple    | u         | xProj   | r     = sqrt(u**2 + v**2) ;                           |
  |                   | v         | yProj   | theta = ArcTan(v,u) ; (where ThetaIvl:[-Pi Pi])       |
  |                   | w         | zProj   | d     = UnbendNeutralDist ;                           |
  |                   |           |         | xProj = r ;                                           |
  |                   |           |         | yProj = d * theta ;                                   |
  |                   |           |         | zProj = w ;                                           |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvEvaluateSimple | xProj     | u       | r     = xProj ;                                       |
  |                   | yProj     | v       | theta = yProj/d ;                                     |
  |                   | zProj     | w       | d     = UnbendNeutralDist ;                           |
  |                   |           |         | u     = r*Cos(theta) ;                                |
  |                   |           |         | v     = r*Sin(theta) ;                                |
  |                   |           |         | w     = zProj ;                                       |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Orient            | u         | xIn     | Pin = UnbendCenter + u*UnbendMidDir                   |
  |                   | v         | yIn     |                    + v*UnbendBinormal                 |
  |                   | w         | zIn     |                    + w*UnbendAxis ;                   |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvOrient         | xIn       | u       | u = (PxyzIn - UnbendCenter).Dot(UnbendMidDir)         |
  |                   | yIn       | v       | v = (PxyzIn - UnbendCenter).Dot(UnbendBinormal)       |
  |                   | zIn       | w       | w = (PxyzIn - UnbendCenter).Dot(UnbendAxis)           |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Evaluate          | u         | xOut    | r     = sqrt(u**2 + v**2) ;                           |
  |                   | v         | yOut    | theta = ArcTan(v,u) ;                                 |
  |                   | w         | zOut    | d     = UnbendNeutralDist ;                           |
  |                   |           |         | Pout  = UnbendCenter + r * UnbendMidDir               |
  |                   |           |         |                    + d * theta * UnbendBinormal       |
  |                   |           |         |                    + w * UnbendAxis ;                 |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvEvaluate       | xOut      | u       | r     = (PxyzOut - UnbendCenter).Dot(UnbendMidDir) ;  |
  |                   | yOut      | v       | theta = (PxyzOut - UnbendCenter).Dot(UnbendBinormal)/d|
  |                   | zOut      | w       | d     = UnbendNeutralDist ;                           |
  |                   |           |         | u     = r*Cos(theta) ;                                |
  |                   |           |         | v     = r*Sin(theta) ;                                |
  |                   |           |         | w     = (PxyzOut - UnbendCenter).Dot(UnbendAxis) ;    |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Map               | xIn       | xOut    | u     = (PxyzIn - UnbendCenter).Dot(UnbendMidDir) ;   |
  |                   | yIn       | yOut    | v     = (PxyzIn - UnbendCenter).Dot(UnbendBinormal) ; |
  |                   | zIn       | zOut    | w     = (PxyzIn - UnbendCenter).Dot(UnbendAxis) ;     |
  |                   |           |         | r     = sqrt(u**2 + v**2) ;                           |
  |                   |           |         | theta = ArcTan(v,u) ;                                 |
  |                   |           |         | d     = UnbendNeutralDist ;                           |
  |                   |           |         | Pout  = UnbendCenter + r * UnbendMidDir               |
  |                   |           |         |                      + d * theta * UnbendBinormal     |
  |                   |           |         |                      + w * UnbendAxis ;               |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvMap            | xOut      | xIn     | r     = (PxyzOut - UnbendCenter).Dot(UnbendMidDir) ;  |
  |                   | yOut      | yIn     | theta = (PxyzOut - UnbendCenter).Dot(UnbendBinormal)/d|
  |                   | zOut      | zIn     | zProj = (PxyzOut - UnbendCenter).Dot(UnbendAxis)      |
  |                   |           |         | d     = UnbendNeutralDist ;                           |
  |                   |           |         | Pin   = UnbendCenter + r*Cos(theta)*UnbendMidDir      |
  |                   |           |         |                      + r*Sin(theta)*UnbendBinormal    |
  |                   |           |         |                      + zProj       *UnbendAxis ;      |
  |                   |           |         |                                                       |
  |                   |           |         |                                                       |
  +-------------------+-----------+---------+-------------------------------------------------------+

***********************************************************************/
class SM_EXPORT SmUnbendVolume : public SmVolume
{
protected:
  double  m_dUnbendNeutralDist = SM_UNDEF_DOUBLE ; // distance to UnbendNeutralPlane along UnbendMidDir from UnbendOrigin.
  double  m_dSingularityTol    = SM_UNDEF_DOUBLE ; // min dist between NonSingular pts and Volume's singularity at U=0,
                                                   // dSingularityTol must be >= SM_ZONE_TOL_3D, default:[SM_ZONE_TOL_3D]
  // inherited from SmVolume::
  //   // UnbendOrigin    = m_pOrient->GetToDisp()   // InSpace (and OutSpace) point on the bend center line
  //   // UnbendAxis      = m_pOrient->GetToZAxis()  // InSpace (and OutSpace) unit-vector direction of the bend center line
  //   // UnbendBinormal  = m_pOrient->GetToYAxis()  // InSpace (and OutSpace) unit-vector = UnbendAxis * UnbendMidDir
  //   // UnbendMidDir    = m_pOrient->GetToXAxis()  // InSpace (and OutSpace) unit-vector orthoganal to UnbendAxis marking start of Unbend Coordinate system
                                                     //  note: The UnbendMidDir/UnbendBinormal plane is the symmetric plane
                                                     //        of the Unbend mapping
  //   // UnbendNeutralDist = m_dUnbendNeutralDist ; // InSpace (and OutSpace) distance along UnbendMidDir between UnbendOrigin and UnbendNeutralPlane.
  // note: for SmUnbendVolume: UnbendVolume is the inverse of UnbendVolume and two have the same geometry

  // Inherited SmVolume virtual methods:
  //   These virtual methods are implemented by derived classes computing only the EvaluateSimple mapping.
  //   Derived classes are isolated from the complications of both compounding and orientation.
  //     Methods to be implemented for derived Classes
  //
  //      // standard structure
  //        Destructor                           // req implementation: derived class destructors must be virtual
  //        operator==                           // req implementation: deep compare predicate
  //        Copy()                               // req implementation: deep virtual copy operator
  //        Notify()                             // opt implementation: Notify mechanism. Default:[pass Notify to compounded volumes and base class]
  //        GetNaturalParamDomain()              // req implementation: return SmExtent3d ParamSpace domain
  //
  //     // look like a BSplineVolume - gwc change: not all volumes are BSplines - these don't need to be here just in SmBSplineVolume
  //        Reparameterize()                     // opt implementation: reparam bounded domain limits - unbounded vols return SM_ERR. Default:[signal err and return SM_ERR]
  //        GetDegree()                          // opt implementation: return paramDirection degree. Default:[signal error and return 3]
  //        GetKnots()                           // opt implementation: return paramDirection knot vector. Default:[init output and return SM_ERR]
  //        GetNumberControlPoints()             // opt implementation: return paramDirection CPoint cnt. Default:[signal error and return 0]
  //
  //     // EvaluateSimple Interface to fit within the Oriented and compounded structure of the SmVolume base class
  //        EvaluateBoundingBoxSimple()             // opt implementation: map ParamBoundingBoxes to ProjBoundingBoxes
  //        EvaluateIsoParametricCurveSimple()      // req implementation: create ProjSpace IsoCurve for ParamSpace IsoLine
  //        EvaluateIsoParametricSurfaceSimple()    // req implementation: create ProjSpace IsoSurface for ParamSpace IsoPlane
  //        EvaluateSimple()                        // req implementation: compute ProjPoint = EvaluateSimple(ParamPoint)
  //        InvEvaluateGuessPointSimple()           // req implementation: get approximate ParamSpace location for given ProjSpace point
  //        GlobalPointSolveSimple()                // opt implementation: Point Solver without a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  //        LocalPointSolveSimple()                 // opt implementation: Point Solver with a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  //        FindParamIntervalForInSpaceLineSimple() // Opt implementation: Find InSpace Line interval that maps to the NaturalParamDomain, default rTrimIvl = crCurrentIvl
  //        TrimParamBoundingBoxSimple()            // opt implementation: Trim ParamBBox to natural ParamDomain, default pTrimBox = pGivenBox
// GWC - excluded from first release:    //        FindParamExtentForInSpacePlaneSimple()  // Opt implementation: Find InSpace Plane extent that's within the NaturalParamDomain, default rTrimIvl = crCurrentIvl
//     // GWC - removed from interface because it made no sense: Even very simple cases of trimming a BBox to a long skinny PseduoBox showed
//     //       that the inscribed Box could be very small and non-unique.  It won't necessarily capture the notion of a good
//     //       bounding box designed to help narrow down the space of options for functions that need to check that.  In general,
//     //       check wether individual InSpace and ProjPace Points map back to legal ParamDomain Points, one point at a time.
//     // TrimInSpaceBoundingBoxSimple()          // opt implementation: Trim InSpaceBBox to map within the NaturalParamSpaceDomain, default pTrimBox = pGivenBox
  //
  //     // Derived class EvaluateSimple map modifications
  //        // design issue - derived and default behaviors are not exactly the same.
  //        //                Default behavior modifies the volume maps after being oriented, i.e. concatenates another NextMap object.
  //        //                Derived behavior modifies the volume map before being oriented, i.e. modifies EvaluateSimple() by modifying derived map parameters.
  //        //                If at all possible, derived classes should implement this set of virtual methods.
  //        MirrorSimple()                       // opt implementation: Mirror EvaluateSimple map.                 Default:[by concatenating a Mirror        Transform as NextMap]
  //        ScaleSimple()                        // opt implementation: Scale EvaluateSimple map.                  Default:[by concatenating a Scale         Transform as NextMap]
  //        TranslateSimple()                    // opt implementation: Translate EvaluateSimple map.              Default:[by concatenating a Translate     Transform as NextMap]
  //        RotateAboutAxisSimple()              // opt implementation: RotateAboutAxis EvaluateSimple map.        Default:[by concatenating a Rotate        Transform as NextMap]
  //        RotateAboutAxisAtPointSimple()       // opt implementation: RotateAboutAxisAtPoint EvaluateSimple map. Default:[by concatenating a RotateAtPoint Transform as NextMap]
  //        TransformSimple()                    // opt implementation: Transform Volume.                          Default:[by concatenating a general       Transform as NextMap]
  //
  //     // predicates
  //        IsBoundedSimple()                    // opt implementation: Bounded  ParamSpace Predicate. Default:[return FALSE]
  //        IsClosedSimple()                     // opt implementation: Closed   ParamSpace Predicate. Default:[OutSpace Point Sample test]
  //        IsPeriodicSimple()                   // opt implementation: Periodic ParamSpace Predicate. Default:[OutSpace Point Sample test]
  //        IsSingularitySimple()                // opt implementation: EvaluateSingularity Predicate. Default:[OutSpace Zero Tangent check]
  //        IsOnBoundarySimple()                 // opt implementation: ParamPt on ParamBnd Predicate. Default:[ParamSpace dist to ParamBoundindBox check]
  //        IsPointInParamDomainSimple()         // opt: Return TRUE if ParamPoint is within the map's paramSpace Natural domain, default:[return TRUE]
  //        IsPointInProjDomainSimple()          // opt: Return TRUE if projPoint can be inverted uniquely to ParamSpace, default:[return TRUE]
  //        IsLineInParamDomainSimple()          // opt: rtn TRUE if ParamLine is in map's NaturalParamDomain. default:[if bnded(GetNaturalDomain().Contains(ParamPt)) else TRUE], implement if:[ParamDomain is bounded not rectilinear]
  //   ToBe IsLineInProjDomainSimple()           // opt: rtn TRUE if ProjLine drops within map's NaturalParamDomain. default:[if bnded(InvEvaluatePointSimple()) else TRUE], implement if:[ParamDomain is bounded not rectilinear]
  //
  //        IsCurveInParamDomainSimple()         // opt: return TRUE when TgtCurve does NOT cross any Volume external or internal discontinuity boundary
  //        IsSurfaceInParamDomainSimple()       // opt: return TRUE when TgtSurface does NOT cross any Volume external or internal discontinuity boundary
  //        HasDiscontinuitiesSimple()           // opt: Return TRUE if Simple map has internal C1 discontinuities, default:[return FALSE]
  //        CalculateContinuitiesSimple()        // opt: Return list of all Simple map internal discontinuities, default:[none]
  //
  //     // Persistence and reporting
  //        WriteToDB()                          // req implementation: virtual nested write i/o
  //        static ReadFromDB()                  // req implementation: static nested read i/o
  //        GetMemoryUsed()                      // opt implementation: required when derived class has nay members. Default:[this + NextMap + OrientMap + InvOrientMap]
  //        Dump()                               // req implementation: nested pretty print
  //        Draw()                               // opt implementation:
  //        DrawMesh()                           // opt implementation:
  //        DrawControlPoints()                  // opt implementation:
  //        DrawUVW()                            // opt implementation:
  //        AssertValid()                        // req implementation:

public:

  // Constructor
  SmUnbendVolume
  (
    const SmContext  & crContext,              ///< [in ]: for new OrientMap construction                                                                          <br>
    const SmPoint3d  & rUnbendCenter,          ///< [in ]: InSpace (and OutSpace) point on the bend center line                                                    <br>
    const SmVector3d & rUnbendAxis,            ///< [in ]: InSpace (and OutSpace) direction of the bend center line marking W axis of Unbend Coordinate System     <br>
    const SmVector3d & rUnbendStartDir,        ///< [in ]: InSpace (and OutSpace) vector orthoganal to UnbendAxis marking U axis of Unbend Coordinate System       <br>
    double             dUnbendNeutralDist,     ///< [in ]: InSpace (and OutSpace) distance along UnbendMidDir between UnbendOrigin and NeutralPlane.               <br>
    double             dSingularityTol = 0.01  ///< [in ]: min dist between NonSingular pts and Volume's singularity at U=0,                                       <br>
                                               ///<      : dSingularityTol must be >= SM_ZONE_TOL_3D, gwc: 0.01 works better                                       <br>
                                               ///<      : note: The UnbendMidDir/UnbendBinormal plane is the symmetric plane of the Unbend mapping                <br>
  );

  // copy constructors
  // SmUnbendVolume(const SmUnbendVolume & crSourceVolume)   ///< [in ]: SourceVolume to copy (copies all compounding volumes)
  //                                                         { SmUnbendVolume::SmUnbendVolume(crSourceVolume, FALSE) ; }

  SmUnbendVolume
  (
    const SmUnbendVolume & crSourceVolume,   ///< [in ]: SourceVolume to copy                                             <br>
    SmBoolean bSimpleMapOnly = FALSE         ///< [in ]: TRUE = Copy this Volume omitting any compounding volumes         <br>
                                             ///<      : FALSE= Copy this Volumes with any compounding volumes            <br>
  )
    : SmVolume( crSourceVolume, bSimpleMapOnly ),
    m_dUnbendNeutralDist( crSourceVolume.m_dUnbendNeutralDist ),
    m_dSingularityTol( crSourceVolume.m_dSingularityTol )
  {}

  // empty constructor for I/O
  SmUnbendVolume( const SmContext *cpContext = NULL ) : m_dUnbendNeutralDist( 0.0 )
  {
    if(cpContext) { SetContext( cpContext ); }
  }

  // make an exact copy of any volume with or without any compounding volumes
  virtual SmStatus Copy
  (
    const SmContext & crContext,            ///< [in ]: context for new object construction                          <br>
    SmVolume       *& rpNewVolume,          ///< [out]: The copied Volume                                            <br>
    SmBoolean         bSimpleMapOnly=FALSE  ///< [in ]: TRUE = Copy this Volume omitting any compounding volumes     <br>
                                            ///<      : FALSE= Copy this Volumes with any compounding volumes        <br>  
  ) const;

  // assignment operator
  SmUnbendVolume &operator=(const SmUnbendVolume &crUnbendVolume) ;

  // destructor
  virtual ~SmUnbendVolume() { }

  // equality operator
  virtual SmBoolean operator==(const SmVolume&) const;


  static SmStatus CreateCanonical
  (
    const SmContext & crContext,                      ///< [in ]: context for new object construction                                                                    <br>
    SmPoint3d       & rUnbendCenter,                  ///< [in ]: InSpace (and OutSpace) point on the bend center line                                                   <br>
    SmVector3d      & rUnbendAxis,                    ///< [in ]: InSpace (and OutSpace) direction of the bend center line marking W axis of Unbend Coordinate System    <br>
    SmVector3d      & rUnbendStartDir,                ///< [in ]: InSpace (and OutSpace) vector orthoganal to UnbendAxis marking U axis of Unbend Coordinate System      <br>
    double            dUnbendNeutralDist,             ///< [in ]: InSpace (and OutSpace) distance along rUnbendMidDir between UnbendOrigin and NeutralPlane.             <br>
    SmUnbendVolume *& rpNewVolume,                    ///< [out]: New SmUnbendVolume, NULL on input                                                                      <br>
    double            dSingularityTol = 0.01          ///< [in ]: min dist between NonSingular pts and Volume's singularity at U=0,                                      <br>
                                                      ///<      : dSingularityTol must be >= SM_ZONE_TOL_3D                                                              <br>
  );

  SmStatus        GetCanonical
  (
    SmPoint3d  & rUnbendCenter,                       ///< [out]: InSpace (and OutSpace) point on the bend center line                                                   <br>
    SmVector3d & rUnbendAxis,                         ///< [out]: InSpace (and OutSpace) direction of the bend center line marking W axis of Unbend Coordinate System    <br>
    SmVector3d & rUnbendMidDir,                       ///< [out]: InSpace (and OutSpace) vector orthoganal to UnbendAxis marking U axis of Unbend Coordinate System      <br>
    double     & rdUnbendNeutralDist,                 ///< [out]: InSpace (and OutSpace) distance along UnbendMidDir between UnbendOrigin and NeutralPlane.              <br>
    double     & rdSingularityTol                     ///< [out]: min ProjSpace dist between NonSingular pts and the Volume's singularity at the ParameterPlane U=0.     <br>
  ) const ;

  SmStatus        SetCanonical
  (
    SmPoint3d  & rUnbendCenter,                       ///< [in ]: InSpace (and OutSpace) point on the bend center line                                                   <br>
    SmVector3d & rUnbendAxis,                         ///< [in ]: InSpace (and OutSpace) direction of the bend center line marking W axis of Unbend Coordinate System    <br>
    SmVector3d & rUnbendMidDir,                       ///< [in ]: InSpace (and OutSpace) vector orthoganal to UnbendAxis marking U axis of Unbend Coordinate System      <br>
    double       dUnbendNeutralDist,                  ///< [in ]: InSpace (and OutSpace) distance along UnbendMidDir between UnbendOrigin and NeutralPlane.              <br>
    double       dSingularityTol = SM_ZONE_TOL_3D     ///< [in ]: min ProjSpace dist between NonSingular pts and the Volume's singularity at the ParameterPlane U=0.     <br>
           
  );

  // simple data access

  // Natural Domain in ParamSpace and InSpace
  virtual SmExtent3d GetNaturalParamDomain() const
  {
    return SmExtent3d( -SM_INFINITE_PARAMETER, -SM_INFINITE_PARAMETER, -SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER );
  }

  // return a bounded ParamDomain that avoids all internal discontinuities - used for graphics
  virtual SmExtent3d GetDiscontinuityFreeParamDomain() const
  {
    return SmExtent3d( m_dSingularityTol, -SM_INFINITE_PARAMETER, -SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER );
  }


  // Min U distance to singularity plane at U = 0
  double             GetLegalMinU()      const { return ( m_dSingularityTol - SM_EFF_ZERO) ; }
  double             GetSingularityTol() const { return ( m_dSingularityTol) ; }

  // Unbend Coordinate System in InSpace (and OutSpace) directions
  SmVector3d  GetUnbendOrigin()      const    { return ( GetOrientOrigin() ) ; }
  SmVector3d  GetUnbendAxis()        const    { return ( GetOrientZAxis() ) ; }
  SmVector3d  GetUnbendMidDir()      const    { return ( GetOrientXAxis() ) ; }
  SmVector3d  GetUnbendBinormal()    const    { return ( GetOrientYAxis() ) ; }     // UnbendBinormal = UnbendAxis * UnbendMidDir ;
  double      GetUnbendNeutralDist() const    { return ( m_dUnbendNeutralDist ) ; }

  // convenience functions to look like a SmBSplineVolume for cache algorithm - other classes use base default behaviors
  virtual ULONG    GetDegree(SmVolumeParamType eParam) const { SM_REF1(eParam) ; return 3 ; }

  virtual SmStatus GetKnots
  (
    SmVolumeParamType  eParam,            ///< [in ]: one of SM_VP_U, SM_VP_V, SM_VP_W                                   <br>
    SmTArray<double> & rKnots,            ///< [out]: ParamSpace knot list                                               <br>
    SmTArray<ULONG>  * pKnotMults = NULL, ///< [out]: associated knot multipliticies. NULL to ignore. default:[NULL]     <br>
    const SmExtent1d * pOptIvl = NULL     ///< NotUsed: [in ]: interval of interest, NULL=Natural Interval, default:[NULL]        <br>
  ) const ;

  virtual ULONG    GetNumberControlPoints( SmVolumeParamType eParam ) const
  {
    SM_REF1( eParam ); return 4;
  }

  // make exact BSplineCurve projection of InputSpace Curve to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutCurve
  (
    SmVolumeSpaceTYPE eInputSpace,       ///< [in ]: SM_VS_IN_SPACE   = InputCurve is in InSpace                        <br>
                                         ///<      : SM_VS_PARAM_SPACE= InputCurve is in ParamSpace                     <br>
    const SmCurve   & rInputCurve,       ///< [in ]: Curve to project to 1st OutSpace                                   <br>
    SmBSplineCurve *& rpNewCurve         ///< [out]: 1st OutSpace projection or NULL for not possible                   <br>
  ) const ;

  // make exact BSplineCurve projection of InputSpace Surface to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutSurface
  (
    SmVolumeSpaceTYPE   eInputSpace,   ///< [in ]: SM_VS_IN_SPACE   = InputSurface is projected from InSpace             <br>
                                       ///<      : SM_VS_PARAM_SPACE= InputSurface is projected from ParamSpace          <br>
    const SmSurface   & rInputSurface, ///< [in ]: Surface to project to Last OutSpace                                   <br>
    SmBSplineSurface *& rpNewSurface   ///< [out]: 1st OutSpace projection or NULL for not possible                      <br>
  ) const ;

  // VOLUME MAPPING DIAGRAM
  //   +---------------+                                 +------------------+
  //   |   InSpace     | -------------'Map'------------> |     OutSpace     |
  //   | [xIn yIn zIn] | <----------'InvMap'------------ | [xOut yOut zOut] |
  //   +---------------+                              >  +------------------+
  //       ^     |                             >      <        ^     |
  //       | 'InvOrient'     'Evaluate' >      <               | 'InvOrient'
  //   'Orient'  |               >      <                  'Orient'  |
  //       |     v        >      < 'InvEvaluate'               |     v
  //   +---------------+  <                              +------------------+
  //   |  ParamSpace   | -------'EvaluateSimple'-------> |    ProjSpace     |
  //   |   [u v w]     | <-----'InvEvaluateSimple'------ |                  |
  //   +---------------+  [  GlobalPointSolveSimple   ]  +------------------+
  //                      [InvEvaluateGuessPointSimple]
  //                      [   LocalPointSolveSimple   ]

  // Compute OutPoint = Evaluate(ParamPoint) (in final outspace with compounding)
  virtual SmStatus Evaluate
  (
    const SmPoint3d & crParamPoint,    ///< [in ]: ParamSpace point to OutSpace point                                                  <br>
    ULONG             lHighestDeriv,   ///< [in ]: 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                            <br>
    SmBoolean         bUFromLeft,      ///< NotUsed: [in ]: if P is on U, V, or W interval boundary                                             <br>
    SmBoolean         bVFromLeft,      ///< NotUsed: [in ]: TRUE  = evaluate P in upper interval where P is on the left of the interval         <br>
    SmBoolean         bWFromLeft,      ///< NotUsed: [in ]: FALSE = evaluate P in lower interval where P is on the right of the interval        <br>
    SmVector3d      * aDerivatives,    ///< [out]: matrix of OutSpace evaluations values                                               <br>
                                       ///<      : 3d organized: D[u][v][w]                                                            <br>
                                       ///<      : 1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3                  <br>
                                       ///<      : sized       : [n+1][n+1][n+1], where n=lHighesDeriv                                 <br>
                                       ///<      : lHghDrv = 0,   sized: [1],                                                          <br>
                                       ///<      :   i=0          order: [D]                                                           <br>
                                       ///<      : lHghDrv = 1,   sized: [8]                                                           <br>
                                       ///<      :   i=u*4+v*2+w  order: [D  Dw  Dv  ---                                               <br>
                                       ///<      :                        Du --- --- ---]                                              <br>
                                       ///<      : lHghDrv = 2,   sized: [27]                                                          <br>
                                       ///<      :   i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---                          <br>
                                       ///<      :                        Du  Duw --- Duv --- --- --- --- ---                          <br>
                                       ///<      :                        Duu --- --- --- --- --- --- --- ---]                         <br>
                                       ///<      : lHghDrv = 3,   sized: [81]                                                          <br>
                                       ///<      :   i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw            <br>
                                       ///<      :                        --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---            <br>
                                       ///<      :                        Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---            <br>
                                       ///<      :                        --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---            <br>
                                       ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  Duuu ---            <br>
                                       ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---            <br>
                                       ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---            <br>
                                       ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---            <br>
                                       ///<      :                        --- --- ]                                                    <br>
    SmBoolean bNonZeroTangents=TRUE,   ///< NotUsed: [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors        <br>
                                       ///<      : FALSE= return exact tangent values, default:[TRUE]                                  <br>
                                       ///<      : note: Surprisingly TRUE is the common choice because most tangent uses              <br>
                                       ///<      :       are for their direction (Binorm, SurfNorm comps), but when the                <br>
                                       ///<      :       tangent is being used for its magnitude (like an arc-length comp)             <br>
                                       ///<      :       then set this to FALSE.                                                       <br>
                                       ///<      : default:[TRUE]                                                                      <br>
    SmBoolean bDoZeroSampling=TRUE,    ///< NotUsed: [in ]: for internal use only, always set to TRUE, default:[TRUE]                           <br>
    SmBoolean bWithCompounding=TRUE    ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]           <br>
  ) const ;

  // SmVolume base class SimpleMap interface

  // VOLUME MAPPING DIAGRAM
  //   +---------------+                                 +------------------+
  //   |   InSpace     | -------------'Map'------------> |     OutSpace     |
  //   | [xIn yIn zIn] | <----------'InvMap'------------ | [xOut yOut zOut] |
  //   +---------------+                              >  +------------------+
  //       ^     |                             >      <        ^     |
  //       | 'InvOrient'     'Evaluate' >      <               | 'InvOrient'
  //   'Orient'  |               >      <                  'Orient'  |
  //       |     v        >      < 'InvEvaluate'               |     v
  //   +---------------+  <                              +------------------+
  //   |  ParamSpace   | -------'EvaluateSimple'-------> |    ProjSpace     |
  //   |   [u v w]     | <-----'InvEvaluateSimple'------ |                  |
  //   +---------------+  [  GlobalPointSolveSimple   ]  +------------------+
  //                      [InvEvaluateGuessPointSimple]
  //                      [   LocalPointSolveSimple   ]

  // compute ProjPoint = EvaluateSimple(ParamPoint) - requires derived implementation
  virtual SmStatus EvaluateSimple
  (
    const SmPoint3d & crParamPoint,         ///< [in ]: ParamSpace point to map to ProjSpace point                                         <br>
    ULONG             lHighestDeriv,        ///< [in ]: 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                           <br>
    SmBoolean         bUFromLeft,           ///< NotUsed: [in ]: if P is on U, V, or w interval boundary                                            <br>
    SmBoolean         bVFromLeft,           ///< NotUsed: [in ]: TRUE  = evaluate P in upper interval where P is on the left of the interval        <br>
    SmBoolean         bWFromLeft,           ///< NotUsed: [in ]: FALSE = evaluate P in lower interval where P is on the right of the interval       <br>
    SmVector3d      * aDerivatives,         ///< [out]: matrix of ParamSpace evaluations values                                            <br>
                                            ///<      :  3d organized: D[u][v][w]                                                          <br>
                                            ///<      :  1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3                <br>
                                            ///<      :  sized       : [n+1][n+1][n+1], where n=lHighesDeriv                               <br>
                                            ///<      :  lHghDrv = 0,   sized: [1],                                                        <br>
                                            ///<      :    i=0          order: [D]                                                         <br>
                                            ///<      :  lHghDrv = 1,   sized: [8]                                                         <br>
                                            ///<      :    i=u*4+v*2+w  order: [D  Dw  Dv  ---                                             <br>
                                            ///<      :                         Du --- --- ---]                                            <br>
                                            ///<      :  lHghDrv = 2,   sized: [27]                                                        <br>
                                            ///<      :    i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---                        <br>
                                            ///<      :                         Du  Duw --- Duv --- --- --- --- ---                        <br>
                                            ///<      :                         Duu --- --- --- --- --- --- --- ---]                       <br>
                                            ///<      :  lHghDrv = 3,   sized: [81]                                                        <br>
                                            ///<      :    i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw          <br>
                                            ///<      :                         --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---          <br>
                                            ///<      :                         Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---          <br>
                                            ///<      :                         --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---          <br>
                                            ///<      :                         --- ---  ---   ---   ---  ---  ---  ---  Duuu ---          <br>
                                            ///<      :                         --- ---  ---   ---   ---  ---  ---  ---  ---- ---          <br>
                                            ///<      :                         --- ---  ---   ---   ---  ---  ---  ---  ---- ---          <br>
                                            ///<      :                         --- ---  ---   ---   ---  ---  ---  ---  ---- ---          <br>
                                            ///<      :                         --- --- ]                                                  <br>
    SmBoolean bNonZeroTangents=TRUE,        ///< NotUsed: [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors       <br>
                                            ///<      : FALSE= return exact tangent values, default:[TRUE]                                 <br>
                                            ///<      : note: Surprisingly TRUE is the common choice because most tangent uses             <br>
                                            ///<      :       are for their direction (Binorm, SurfNorm comps), but when the               <br>
                                            ///<      :       tangent is being used for its magnitude (like an arc-length comp)            <br>
                                            ///<      :       then set this to FALSE.                                                      <br>
                                            ///<      : default:[TRUE]                                                                     <br>
    SmBoolean bDoZeroSampling=TRUE          ///< NotUsed: [in ]: for internal use only, always set to TRUE, default:[TRUE]                          <br>
  ) const ;

  // compute ProjSpace BBoxes for given ParamSpace Box
  virtual SmStatus EvaluateBoundingBoxSimple
  (
    const SmExtent3d  & crParamDomain,
    SmPseudoBox       * pOptPseudoParamBox = NULL,                  ///< [in ]: ParamSpace BBox to project to Project Space                                                             <br>
                                                                    ///< [in ]: optional ParamSpace PseudoBox used to set output PseudoBox orientations,                                <br>
                                                                    ///<      : NULL   : ProjPseudoBox Basis = Project X Y Z ParamVecs to ProjSpace at crParamBox Center                <br>
                                                                    ///<      : NotNULL: ProjPseudoBox Basis = Project pOptParamPseudoBox BasisVecs to ProjSpace at crParamBox Center   <br>
    SmExtent3d        * pNormalProjBox = NULL,                      ///< [out]: ProjectSpace Axis aligned box                                                                           <br>
    SmPseudoBox       * pPseudoProjBox = NULL                       ///< [out]: ProjectSpace Non-axis aligned box                                                                       <br>
                                                                
  ) const ;

  // Find portion of InSpace Line interval that maps to the NaturalParamDomain
  // rtn: SM_ERR for NULL line, else SM_SUCCESS
  virtual SmStatus FindParamIntervalForInSpaceLineSimple                
  (
    const SmPoint3d      & crInSpacePoint,   ///< [in ]: LinePoint       of Line(s) = LinePoint + s * LineVector                 <br>
    const SmVector3d     & crInSpaceVector,  ///< [in ]: unit LineVector of Line(s) = LinePoint + s * LineVector                 <br>
    const SmExtent1d     & crCurrentIvl,     ///< [in ]: current limits on s interval                                            <br>
    SmTArray<SmExtent1d> & rTrimIvls         ///< [out]: Interval of line that maps legally within the NaturalParamDomains       <br>
  ) const ;

  // GWC: exclude FindParamExtentForInSpacePlaneSimple from first release. Code is written but I don't
  //      see the bug in SmUnbendVolume::FindParamExtentForInSpacePlaneSimple().  I'll fix that later.
  //      // Find the largest square extent of an InSpace plane that maps inside the NaturalParamDomain.
  //      virtual SmStatus FindParamExtentForInSpacePlaneSimple                 // rtn: SM_ERR for NULL plane, else SM_SUCCESS
  //                                  (const SmPoint3d      & crInSpacePoint,   ///< [in ]: LinePoint          of Line(s) = LinePoint + u * LineVecU + v * LineVecV
  //                                   const SmVector3d     & crInSpaceVecU,    ///< [in ]: scaled LineVectorU of Line(s) = LinePoint + u * LineVecU + v * LineVecV
  //                                   const SmVector3d     & crInSpaceVecV,    ///< [in ]: scaled LineVectorV of Line(s) = LinePoint + u * LineVecU + v * LineVecV
  //                                   const SmExtent2d     & crCurrentUV,      ///< [in ]: current limits on UV extent
  //                                   SmTArray<SmExtent2d> & rTrimUVs)         ///< [out]: UVExtent of plane that maps legally within the NaturalParamDomains
  //                                  const ;

  // Trim ParamSpace BBox to Natural Parameter Domain
  // rtn: SM_ERR when ParamBox trims to empty set, else SM_SUCCESS
  virtual SmStatus TrimParamBoundingBoxSimple                     
  (
    const SmExtent3d & crParamBox,              ///< [in ]: Tgt Param Box to trim                    <br>
    SmExtent3d       & rParamTrimBox            ///< [in ]: Box trimmed to Natural Param Domain      <br>
  ) const ;

  // Create ProjSpace IsoCurve from a ParamSpace IsoParamLine
  virtual SmStatus EvaluateIsoParametricCurveSimple
  (
    const SmContext     & crContext,            ///< [in ]: context for created objects                              <br>
    SmVolumeParamsType    eConstantParams,      ///< [in ]: oneof: SM_VPS_UV_IN,                                     <br>
                                                ///<      :        SM_VPS_UW_IN,                                     <br>
                                                ///<      :        SM_VPS_VW_IN.                                     <br>
    double                dIsoParam1,           ///< [in ]: 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value    <br>
    double                dIsoParam2,           ///< [in ]: 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value    <br>
    double                d3DTolerance,         ///< NotUsed: [in ]: Max ApproxCurve to IdealCurve deviation                  <br>
    SmCurve            *& rpNewIsoCurve,        ///< [out]: the ProjSpace IsoCurve                                   <br>
    const SmExtent3d    * pOptParamDomain=NULL  ///< [in ]: limiting domain, NULL to ignore. default:[NULL]          <br>
  ) const ;

  // Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane
  virtual SmStatus EvaluateIsoParametricSurfaceSimple
  (
    const SmContext   & crContext,              ///< [in ]: context for created objects                              <br>
    SmVolumeParamType   eConstantParam,         ///< [in ]: oneof: SM_VP_U_IN,                                       <br>
                                                ///<      :        SM_VP_V_IN,                                       <br>
                                                ///<      :        SM_VP_W_IN                                        <br>
    double              dIsoParam,              ///< [in ]: constant param value                                     <br>
    double              d3DTolerance,           ///< NotUsed: [in ]: Max ApproxSurface to IdealSurface deviation              <br>
    SmSurface        *& rpNewIsoSurface,        ///< [out]: the ProjSpace IsoSurface                                 <br>
    const SmExtent3d  * pOptParamDomain=NULL    ///< [in ]: limiting domain, NULL to ignore. default:[NULL]          <br>
  ) const ;

  // get approximate ParamSpace location for given ProjSpace point for upcoming newton raphson search
  virtual SmStatus InvEvaluateGuessPointSimple
  (
    const SmPoint3d     & crProjPoint,        ///< [in ]: ProjSpace Point to map back to ParamSpace Point                    <br>
    SmTArray<SmPoint3d> & rGuessParamPoints   ///< [out]: ParamSpace Point near Target Point actual map back to ParamSpace   <br>
  ) const ;

  // General Point Solver without a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  virtual SmStatus GlobalPointSolveSimple
  (
    const SmPoint3d  & crProjPoint,          ///< [in ]: ProjSpace Point to map back to ParamSpace                                            <br>
    SmBoolean        & rbFoundAnswer,        ///< [out]: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point                     <br>
    SmSolutionArray  & rSolutions,           ///< [out]: Contains ParamSpace Found Point                                                      <br>
                                             ///<      : rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                          <br>
                                             ///<      : rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint); <br>
                                             ///<      : rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location         <br>
                                             ///<      : rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location         <br>
                                             ///<      : rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location         <br>
    const SmExtent3d * pOptParamDomain=NULL  ///< [in ]: ParamSpace domain over which to search for the inverse point                         <br>
                                             ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                       <br>
  ) const ;                                  

  // Local Point Solver with a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  virtual SmStatus  LocalPointSolveSimple
  (
    const SmPoint3d  & crProjPoint,          ///< [in ]: ProjSpace Point to map back to ParamSpace                                             <br>
    const SmPoint3d  & crParamPointGuess,    ///< NotUsed: [in ]: ParamSpace point guess, the closer to the actual ParamSpace point the better          <br>
    SmBoolean        & rbFoundAnswer,        ///< [out]: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point                      <br>
    SmSolution       & rSolution,            ///< [out]: Contains ParamSpace Found Point                                                       <br>
                                             ///<      : rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                           <br>
                                             ///<      : rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);  <br>
                                             ///<      : rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location          <br>
                                             ///<      : rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location          <br>
                                             ///<      : rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location          <br>
    const SmExtent3d * pOptParamDomain=NULL  ///< [in ]: ParamSpace domain over which to search for the inverse point                          <br>
                                             ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                        <br>
  ) const ;                                   

  // return TRUE if any of the compound mappings are bounded
  virtual SmBoolean IsBoundedSimple    () const ;                            

  virtual SmBoolean IsClosedSimple     
  (
    SmBoolean        & rbClosedU,                ///< [out]: TRUE = closed in U direction, [check Pos[Umin,v,w] == Pos[Umax,v,w] for v,w samples     <br>
    SmBoolean        & rbClosedV,                ///< [out]: TRUE = closed in V direction, [check Pos[u,Vmin,w] == Pos[u,Vmax,w] for w,u samples     <br>
    SmBoolean        & rbClosedW,                ///< [out]: TRUE = closed in W direction, [check Pos[u,v,Wmin] == Pos[u,v,Wmax] for u,v samples     <br>
    double           * pdOptTolerance = NULL,    ///< NotUsed: [out]: pdOptTolerance = Tolerance to allow for check                                           <br>
    const SmExtent3d * pOptParamDomain = NULL,   ///< NotUsed: [in ]: ParamSpace domain over which to search for the inverse point                            <br>
                                                 ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                          <br>
    SmContinuityType * peOptContinuityU = NULL,  ///< [out]: U dir Continuity when closed                                                            <br>
    SmContinuityType * peOptContinuityV = NULL,  ///< [out]: V dir Continuity when closed                                                            <br>
    SmContinuityType * peOptContinuityW = NULL   ///< [out]: W dir Continuity when closed                                                            <br>
                                                 ///<      : oneof SM_CT_DISCONTINUOUS                                                               <br>
                                                 ///<      :       SM_CT_C0                                                                          <br>
                                                 ///<      :       SM_CT_G1                                                                          <br>
                                                 ///<      :       SM_CT_G1_G2                                                                       <br>
                                                 ///<      :       SM_CT_G1_G2_G3                                                                    <br>
                                                 ///<      :       SM_CT_C1                                                                          <br>
                                                 ///<      :       SM_CT_C1_G2                                                                       <br>
                                                 ///<      :       SM_CT_C1_G2_G3                                                                    <br>
                                                 ///<      :       SM_CT_C1_C2                                                                       <br>
                                                 ///<      :       SM_CT_C1_G3                                                                       <br>
                                                 ///<      :       SM_CT_C1_C3                                                                       <br>
  ) const ;

  virtual SmBoolean IsPeriodicSimple
  (
    SmBoolean        & rbPeriodicU,            ///< [out]: TRUE = G1 or better in U Dir                                   <br>
    SmBoolean        & rbPeriodicV,            ///< [out]: TRUE = G1 or better in V Dir                                   <br>
    SmBoolean        & rbPeriodicW,            ///< [out]: TRUE = G1 or better in W Dir                                   <br>
    const SmExtent3d * pOptParamDomain = NULL  ///< NotUsed: [in ]: ParamSpace domain over which to search for the inverse point   <br>
                                               ///<      : NULL = use Map's NaturalDomain, default:[NULL]                 <br>
  ) const;

  // return TRUE if point evaluation is singular in any parametric direction
  virtual SmBoolean IsSingularitySimple
  (
    const SmPoint3d  & crParamPoint,         
    SmBoolean        & rbSingularU,          ///< [out]: TRUE = TargetPoint on RAxis boundary:[r=0] (the WAxis)                 <br>
    SmBoolean        & rbSingularV,          ///< [out]: TRUE = TargetPoint on theta boundary:[-pi pi] (1/2 plane V=0 and U<0)  <br>
    SmBoolean        & rbSingularW,          ///< [out]: Always FALSE                                                           <br>
    double             d3dTol=SM_EFF_ZERO    ///< [in ]: min OutSpace distance between distinct points                          <br>
  ) const ;

  virtual SmBoolean IsOnBoundarySimple 
  (
    const SmPoint3d  & crParamPoint,         ///< [in ]: Volume UVWPoint to test                                                 <br>
    SmBoolean        & rbOnU,                ///< [out]: TRUE = TargetPoint on RAxis boundary:[r=0] (the WAxis)                  <br>
    SmBoolean        & rbOnV,                ///< [out]: TRUE = TargetPoint on theta boundary:[-pi pi] (1/2 plane V=0 and U<0)   <br>
    SmBoolean        & rbOnW,                ///< [out]: TRUE = TargetPoint on Z boundary: Z is unbounded (always FALSE)         <br>
    double           * pdOptTolerance=NULL,  ///< [in ]: max deviation allowed for point on seam                                 <br>
                                             ///<      : NULL = use SM_EFF_ZERO * 1000 * (1 + maxDimension())                    <br>
    const SmExtent3d * pOptParamDomain=NULL  ///< NotUsed: [in ]: ParamSpace domain over which to search for the inverse point            <br>
                                             ///<      : NULL = use Map's NaturalDomain, default:[NULL]                          <br>
  ) const;

  // rtn: TRUE if ParamPoint is Not on a discontinuity
  virtual SmBoolean IsPointInParamDomainSimple
  (
    const SmPoint3d  & crParamPoint
  ) const
  {
    SmBoolean bB = false;
    return(!IsOnBoundarySimple( crParamPoint, bB, bB, bB ));
  }

  // rtn: TRUE if projPoint can be inverted uniquely to DomainSpace
  virtual SmBoolean IsPointInProjDomainSimple
  ( 
    const SmPoint3d  & crProjPoint,    
    SmPoint3d   * pOptParamPoint = NULL  ///< [out]: if the point is invertible, go ahead and get the inverse  <br>
  )  const
  {
    SM_REF1(pOptParamPoint) ;
    double dD = m_dUnbendNeutralDist;
    return(crProjPoint.x > GetLegalMinU()
           && crProjPoint.y >= -dD * SM_PI
           && crProjPoint.y <= dD * SM_PI);
  }

  // rtn: TRUE if ParamLine is within the maps paramSpace Natural domain
  virtual SmBoolean IsLineInParamDomainSimple
  (
    const SmPoint3d  & crStartParamPoint,      ///< [in ]: ParamSpace Line StartPoint to test for inclusion      <br>
    const SmPoint3d  & crEndParamPoint         ///< [in ]: ParamSpace Line EndPoint to test for inclusion        <br>
  ) const ;

  // ToBe implemented: base implementations on projecting line to ParamSpace and testing IsCurveInParamDomainSimple
  //  virtual SmBoolean IsLineInProjDomainSimple(const SmPoint3d  & crStartProjPoint,    // rtn: TRUE if projPoint can be inverted uniquely to ParamSpace
  //                                             const SmPoint3d  & crEndProjPoint,      ///< [in ]: LineSeg End projPoint
  //                                             SmPoint3d        * pOptStartParamPoint, ///< [out]: if the point is invertible, go ahead and get the inverse
  //                                             SmPoint3d        * pOptEndParamPoint)   ///< [out]: if the point is invertible, go ahead and get the inverse
  //                                            const

  virtual SmBoolean IsCurveInParamDomainSimple(const SmCurve & crParamCurve) const ;

  virtual SmBoolean IsSurfaceInParamDomainSimple(const SmSurface & crParamSurface) const ;

public:
  // utilities

  // I/O assist
  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,             ///< [in ]: target output stream                                <br>
    ULONG lDBVersionNumber          ///< [in ]: database version to get proper sequence of writes   <br>
  ) const ; 

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,              ///< NotUsed: [in ]: Object type to be read                                                      <br>
    SmDatabaseIO    & rDB,                ///< [in ]: target output stream                                                        <br>
    const SmContext & crContext,          ///< [in ]: context for new object construction                                         <br>
    SmVolume        *&rpNewVolume,        ///< [out]: NULL on input = new object allocated in this routine built from stream data <br>
                                          ///<      : NotNULL on input = pointer to an empty object to be filled by this routine  <br>
    ULONG             lDBVersionNumber    ///< [in ]: database version to get proper sequence of writes                           <br>
  ) ;                                                                                                                         

  // get memory used for volume and its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes    <br>
    SmMarkType eMarkType=SM_MT_NOMARK  ///< [in ]: uses without increment eMarkType value          <br>
  ) const ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmUnbendVolume, SmVolume, SmUnbendVolume_TYPE) ;

  // pretty print SmVolume
  virtual void Dump(SmBoolean bAbbrev, ULONG lIndentCnt) const ;
  virtual SmDisplayList * Draw
  (
    SmBoolean       bShowOutSpace=TRUE,            ///< [in ]: TRUE = draw point = Evaluate(sParam)), FALSE=don't                             <br>
    SmBoolean       bShowInSpace = FALSE,          ///< [in ]: TRUE = draw point = Orient(sParam), FALSE=don't                                <br>
    SmExtent3d    * pOptParamDomain = NULL,        ///< [in ]: optional ParamDomain to Draw, NULL=Draw Approximated Unbounded Region          <br>
    SmGfxArraySet * pOptGfxSet = NULL              ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.<br>
  ) const ;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                             <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                               <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

} ; // end class SmUnbendVolume


// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmUnbendVolume*)

#endif // !__SMUNBENDVOLUME_H__

