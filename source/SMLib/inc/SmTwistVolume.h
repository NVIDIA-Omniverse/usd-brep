// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmTwistVolume.h
* PURPOSE: Header file for the Twist Volume class.
* Oct 2006 GWC Author
**********************************************************************/

#ifndef __SMTWISTVOLUME_H__
#define __SMTWISTVOLUME_H__

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
PURPOSE: The SmTwistVolume derived SmVolume object represents a Twist, mapping
         rotating Z-Normal planes along the Z axis at a constant rate.
         The behavior can be used to mimic the shape of a twist in a bar of
         steal or a piece of molten glass.

 EvaluateSimple Map:
 Map UVW ParamSpace point to xyz ProjSpace point and iso-derivatives.

  EvaluateSimple : d      = TwistPeriodDistance (negative for left hand twist)
                   a      = 2Pi * W/d
                   Dw(a)  = 2Pi/d ;
                   Dww(a) = 0
  Position
    PProj       = [xProj] = [ cos(a) * U + -sin(a) * V]
                  [yProj] = [ sin(a) * U +  cos(a) * V]
                  [zProj] = [ W                       ]
                
  1st Derivs
    Du(PProj)   = [ cos(a),  sin(a), 0]
    Dv(PProj)   = [-sin(a),  cos(a), 0]
    Dw(PProj)   = [2Pi/d*(-sin(a)*U + -cos(a)*V)]
                  [2Pi/d*( cos(a)*U + -sin(a)*V)]
                  [           1                 ]
                
  2nd Derivs    
    Duu(PProj)  =  0 ;
    Duv(PProj)  =  0 ;
    Duw(PProj)  = [2Pi/d* -sin(a)]
                  [2Pi/d*  cos(a)]
                  [  0           ]
    Dvv(PProj)  = 0 ;
    Dvw(PProj)  = [2Pi/d* -cos(a)]
                  [2Pi/d* -sin(a)]
                  [  0           ]
    Dww(PProj)  = [(2Pi/d)*(2Pi/d)*(-cos(a)*U +  sin(a)*V)]
                  [(2Pi/d)*(2Pi/d)*(-sin(a)*U + -cos(a)*V)]
                  [  0                                        ]
                
  3rd Derivs
    Duuu(PProj) = 0 ;
    Duuv(PProj) = 0 ;
    Duuw(PProj) = 0 ;
    Duvv(PProj) = 0 ;
    Duvw(PProj) = 0 ;
    Duww(PProj) = [(2Pi/d)*(2Pi/d)* -cos(a)]
                  [(2Pi/d)*(2Pi/d)* -sin(a)]
                  [  0                     ]
    Dvvv(PProj) = 0 ;
    Dvvw(PProj) = 0 ;
    Dvww(PProj) = [(2Pi/d)*(2Pi/d)*  sin(a)]
                  [(2Pi/d)*(2Pi/d)* -cos(a)]
                  [  0                     ]
    Dwww(PProj) = [(2Pi/d)*(2Pi/d)*(2Pi/d)*( sin(a)*U +  cos(a)*V)]
                  [(2Pi/d)*(2Pi/d)*(2Pi/d)*(-cos(a)*U +  sin(a)*V)]
                  [  0                                            ]

 EvaluateSimple InverseMap:
 Map xyz ProjSpace point UVW ParamSpace point 

  EvaluateSimple : d      = TwistPeriodDistance (negative for left hand twist)
                   a      = 2Pi * W/d
  Position
     [U] = [ cos(a) * xProj +  sin(a) * yProj]
     [V] = [-sin(a) * xProj +  cos(a) * yProj]
     [W] = [ zProj                           ]


NOTES:  The Twist's axis is aligned with the ParamSpace W Axis.
        The twist domain maps all of UVW ParamSpace to all of xyz ProjSpace.

        ParameterSpace planes perpendicular to W Axis map to ProjSpace planes

  The SmTwistVolume class implements the SmVolume virtual methods that
    define the ParamSpace -> ProjSpace portion of the overall
    InSpace -> ParamSpace -> ProjSpace -> OutSpace mapping.

  Twisting consists of sending the points of an object's
  shape through the overall InSpace to Outpace mapping.

  When TwistPeriodDistance > 0, positive twist about the W axis
       TwistPeriodDistance < 0, negative twist about the W axis
       TwistPeriodDistance = 0 asks for an infinite twist rate and is not allowed.

       larger TwistPeriodDistance values have slow twist rates.
       The inverse of a Twist(TwistPeriodDistance = val) space warp 
                    is Twist(TwistPeriodDistance = -val) space warp.

  A twist is completely defined by the one parameter, TwistPeriodDistance which
  is the distance traveled along the W axis for one complete revolution of twist.

  The Twist is oriented in InSpace and OutSpace by The ortho normal coordinate set
        TwistOrigin,
        TwistMidDir,
        TwistBinormal,
        TwistAxis.
  Twist in InSpace and Outspace are about the TwistAxis vector.

  o. The TwistOrigin InSpace and OutSpace points are coincident and are on the twist's axis.
  o. The TwistAxis InSpace and OutSpace vectors point along the length of the twist.
  o. The TwistMidDir InSpace and OutSpace vectors define the 'X' direction of the local twist coordinate system.
  o. The TwistBinormal InSpace and OutSpace vectors define the 'Y' direction of the local twist coordinate system.
  o. The TwistAxis InSpace and OutSpace vectors define the 'Z' direction of the local twist coordinate system.
  o. The Twist's TwistPeriodDistance defines the distance along the TwistAxis for one full revolution.

  In ParamSpace, the twist is built about the ParamSpace coordinate system as,

    ParamSpace TwistOrigin    = ParamPt (0,0,0)
    ParamSpace TwistAxis      = ParamVec(0,0,1)
    ParamSpace TwistMidDir    = ParamVec(1,0,0)
    ParamSpace TwistBinormal  = ParamVec(0,1,0)

  In InSpace, the twist is oriented and positioned by the Orient() mapping
  between orthogonal coordinate systems so that,

    InSpace    TwistOrigin    = GetTwistOrigin()    (stored in m_pOrientMap->GetToDisp())
    InSpace    TwistAxis      = GetTwistAxis()      (stored in m_pOrientMap->GetToZAxis())
    InSpace    TwistMidDir    = GetTwistMidDir()    (stored in m_pOrientMap->GetToXAxis())
    InSpace    TwistBinormal  = TwistAxis * TwistMidDir

  In OutSpace, the twist has the same orientation and location as it does in InSpace so that,

    OutSpace   TwistOrigin    = GetTwistOrigin()    (stored in m_pOrientMap->GetToDisp())
    OutSpace   TwistAxis      = GetTwistAxis()      (stored in m_pOrientMap->GetToZAxis())
    OutSpace   TwistMidDir    = GetTwistMidDir()    (stored in m_pOrientMap->GetToXAxis())
    OutSpace   TwistBinormal  = TwistAxis * TwistMidDir

  In the InSpace->OutSpace twist mapping:
    o. Inspace planes perpendicular to the TwistAxis map to rotated planes in OutSpace.
    o. Any planar shape in a plane perpendicular to the TwistAxis maps to a rotated shape in OutSpace.

  As an interesting detail, 
    geometry points on the planes W*2pi/TwistPeriodDist = i*2pi where i is any integer,
    are not moved by the twist mapping.
  
  The twist mappings between InSpace (xIn,yIn,zIn), ParamSpace (u,v,w),
  ProjSpace (xProj,yProj,zProj), and OutSpace (xOut,yOut,zOut) spaces are
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Twist Mapping     | FromSpace | ToSpace | mapping                                               |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | EvaluateSimple    | u         | xProj   | d     = TwistPeriodDistance                           |
  |                   | v         | yProj   | a     = 2Pi * W/d; Dw(a) = 2Pi/d; Dww(a) = 0 ;        |
  |                   | w         | zProj   | xProj =  cos(a) * U + sin(a) * V                      |
  |                   |           |         | yProj = -sin(a) * U + cos(a) * V                      |
  |                   |           |         | zProj =  W                                            |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvEvaluateSimple | xProj     | u       |  u    =  cos(-a) * xProj + sin(-a) * yProj            |
  |                   | yProj     | v       |  v    = -sin(-a) * xProj + cos(-a) * yProj            |
  |                   | zProj     | w       |  w    =  zProj                                        |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Orient            | u         | xIn     | Pin = TwistOrigin + u*TwistMidDir                     |
  |                   | v         | yIn     |                   + v*TwistBinormal                   |
  |                   | w         | zIn     |                   + w*TwistAxis ;                     |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvOrient         | xIn       | u       | u = (PxyzIn - TwistOrigin).Dot(TwistMidDir)           |    
  |                   | yIn       | v       | v = (PxyzIn - TwistOrigin).Dot(TwistBinormal)         |    
  |                   | zIn       | w       | w = (PxyzIn - TwistOrigin).Dot(TwistAxis)             |    
  +-------------------+-----------+---------+-------------------------------------------------------+    
  | Evaluate          | u         | xOut    | d     = TwistPeriodDistance                           |    
  |                   | v         | yOut    | a     = 2Pi * W/d; Dw(a) = 2Pi/d; Dww(a) = 0 ;        |    
  |                   | w         | zOut    | POut  = TwistOrigin                                   |    
  |                   |           |         |   +  cos(a)*TwistMidDir*u + sin(a)*TwistBinormal*v    |    
  |                   |           |         |   + -sin(a)*TwistMidDir*u + cos(a)*TwistBinormal*v    |    
  |                   |           |         |   +  TwistAxis*w                                      |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvEvaluate       | xOut      | u       | xTwistOut = (PxyzOut-TwistOrigin).Dot(TwistMidDir) ;  |
  |                   | yOut      | v       | yTwistOut = (PxyzOut-TwistOrigin).Dot(TwistBinormal) ;|
  |                   | zOut      | w       | zTwistOut = (PxyzOut-TwistOrigin).Dot(TwistAxis) ;    |
  |                   |           |         | u     =  cos(-a) * xTwistOut + sin(-a) * yTwistOut    |     
  |                   |           |         | v     = -sin(-a) * xTwistOut + cos(-a) * yTwistOut    |     
  |                   |           |         | w     =  zTwistOut                                    |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Map               | xIn       | xOut    | xTwistIn = (PxyzIn-TwistOrigin).Dot(TwistMidDir) ;    |
  |                   | yIn       | yOut    | yTwistIn = (PxyzIn-TwistOrigin).Dot(TwistBinormal)/d ;|
  |                   | zIn       | zOut    | zTwistIn = (PxyzIn-TwistOrigin).Dot(TwistAxis) ;      |
  |                   |           |         | POut  =    TwistOrigin                                |
  |                   |           |         |         +  cos(a)*TwistMidDir  *xTwistIn              |                 
  |                   |           |         |         +  sin(a)*TwistBinormal*yTwistIn              |
  |                   |           |         |         + -sin(a)*TwistMidDir  *xTwistIn              |                 
  |                   |           |         |         +  cos(a)*TwistBinormal*yTwistIn              |
  |                   |           |         |         +  TwistAxis*zTwistIn                         |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvMap            | xOut      | xIn     | xTwistOut = (PxyzOut-TwistOrigin).Dot(TwistMidDir) ;  |
  |                   | yOut      | yIn     | yTwistOut = (PxyzOut-TwistOrigin).Dot(TwistBinormal) ;|
  |                   | zOut      | zIn     | zTwistOut = (PxyzOut-TwistOrigin).Dot(TwistAxis) ;    |
  |                   |           |         | PIn  =    TwistOrigin                                 |
  |                   |           |         |        +  cos(-a)*TwistMidDir  *xTwistOut             |                 
  |                   |           |         |        +  sin(-a)*TwistBinormal*yTwistOut             |
  |                   |           |         |        + -sin(-a)*TwistMidDir  *xTwistOut             |                 
  |                   |           |         |        +  cos(-a)*TwistBinormal*yTwistOut             |
  |                   |           |         |        +  TwistAxis*zTwistOut                         |       
  +-------------------+-----------+---------+-------------------------------------------------------+

***********************************************************************/
class SM_EXPORT SmTwistVolume : public SmVolume
{
protected:
  double  m_dTwistPeriodDist = SM_UNDEF_DOUBLE ; // distance along the TwistAxis for one complete Twist revolution
                                                 // TwistPeriodDistance > 0, positive twist about the W axis
                                                 // TwistPeriodDistance < 0, negative twist about the W axis
                                                 // TwistPeriodDistance = 0 asks for an infinite twist rate and is not allowed.
                                                 // must be nonZero

  double  m_dPeriodMinDist   = SM_UNDEF_DOUBLE ; // min allowed value for smos_fabs(m_dTwistPeriodDist)

  // inherited from SmVolume::
  //   // TwistOrigin    = m_pOrient->GetToDisp()   // InSpace (and OutSpace) point on the twist center line
  //   // TwistAxis      = m_pOrient->GetToZAxis()  // InSpace (and OutSpace) unit-vector direction of the twist center line
  //   // TwistBinormal  = m_pOrient->GetToYAxis()  // InSpace (and OutSpace) unit-vector = TwistAxis * TwistMidDir
  //   // TwistMidDir    = m_pOrient->GetToXAxis()  // InSpace (and OutSpace) unit-vector orthoganal to TwistAxis marking start of Twist Coordinate system
                                                   //  note: The TwistMidDir/TwistBinormal plane is the symmetric plane
                                                   //        of the Twist mapping

  // Inherited SmVolume members
  //       SmObject    * m_pOwner ;            // not used
  //       SmVolume    * m_pNextMap ;          // linked list of concatenated SmVolume transformations
  //       SmTransform * m_pOrientMap ;        // for space deformations to orient this mapping's ParamSpace to a target InSpace.
  //       SmTransform * m_pInvOrientMap ;     // cached m_pOrientMap Inverse.
  //       ULONG         m_lNextOwnerFlag ;    // owner state for m_pNextMap, needed for destructor
  //       ULONG         m_lOrientOwnerFlag ;  // owner state for m_pOrientMap, needed for destructor

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
  // SmVolume default behavior:        FindParamIntervalForInSpaceLineSimple() // Opt implementation: Find portion of InSpace Line interval that maps to the NaturalParamDomain, default rTrimIvl = crCurrentIvl
// GWC - excluded from first release:  //        FindParamExtentForInSpacePlaneSimple()  // Opt implementation: Find InSpace Plane extent that's within the NaturalParamDomain, default rTrimIvl = crCurrentIvl
  //        TrimParamBoundingBoxSimple()            // opt implementation: Trim ParamBBox to natural ParamDomain, default pTrimBox = pGivenBox
//     // GWC - removed from interface because it made no sense: Even very simple cases of trimming a BBox to a long skinny PseduoBox showed
//     //       that the inscribed Box could be very small and non-unique.  It won't necessarily capture the notion of a good
//     //       bounding box designed to help narrow down the space of options for functions that need to check that.  In general,
//     //       check wether individual InSpace and ProjPace Points map back to legal ParamDomain Points, one point at a time.
//     //   //        TrimInSpaceBoundingBoxSimple()          // opt implementation: Trim InSpaceBBox to map within the NaturalParamSpaceDomain, default pTrimBox = pGivenBox
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
  SmTwistVolume
  (
    const SmContext  & crContext,                        ///< [in ]: context for new OrientMap construction
    const SmPoint3d  & rTwistOrigin,                     ///< [in ]: InSpace (and OutSpace) point on the twist center line
    const SmVector3d & rTwistAxis,                       ///< [in ]: InSpace (and OutSpace) direction of the twist center line marking W axis of Twist Coordinate System
    const SmVector3d & rTwistMidDir,                     ///< [in ]: InSpace (and OutSpace) vector orthoganal to TwistAxis marking U axis of Twist Coordinate System
    double             dTwistPeriodDist,                 ///< [in ]: distance along the TwistAxis for one complete Twist revolution
    double             dPeriodMinDist=SM_ZONE_TOL_3D     ///< [in ]: min allowed value dTwistPeriodDist
                                                         ///<      : dPeriodMinDist must be >= SM_ZONE_TOL_3D
  );

  // copy constructor  // copy constructors
  SmTwistVolume
  (
    const SmTwistVolume & crSourceVolume,                ///< [in ]: SourceVolume to copy
     SmBoolean             bSimpleMapOnly=FALSE          ///< [in ]: TRUE = Copy this Volume omitting any compounding volumes
                                                         ///<      : FALSE= Copy this Volume with any compounding volumes
  ) ; 
                                                              

  // empty constructor for I/O
  SmTwistVolume( const SmContext * cpContext = NULL ) : m_dTwistPeriodDist( 0.0 )
  {
    if(cpContext) { SetContext( cpContext ); }
  }

  // make an exact copy of any volume with or without any compounding volumes
  virtual SmStatus Copy
  (
    const SmContext & crContext,              ///< [in ]: context for new object construction                          <br>
    SmVolume       *& rpNewVolume,            ///< [out]: The copied Volume                                            <br>
    SmBoolean         bSimpleMapOnly = FALSE  ///< [in ]: TRUE = Copy this Volume omitting any compounding volumes     <br>
                                              ///<      : FALSE= Copy this Volumes with any compounding volumes        <br>
                                              ///<      : default:[FALSE]                                              <br>
  ) const;

  // assignment operator
  SmTwistVolume & operator=(const SmTwistVolume &crTwistVolume) ;

  // equality operator
  virtual SmBoolean operator==(const SmVolume&) const ;

  // destructor
  virtual ~SmTwistVolume() { }

  static SmStatus CreateCanonical
  (
    const SmContext & crContext,                 ///< [in ]: context for new object construction                                                                   <br>
    SmPoint3d       & rTwistOrigin,              ///< [in ]: InSpace (and OutSpace) point on the twist center line                                                 <br>
    SmVector3d      & rTwistAxis,                ///< [in ]: InSpace (and OutSpace) direction of the twist center line marking W axis of Twist Coordinate System   <br>
    SmVector3d      & rTwistMidDir,              ///< [in ]: InSpace (and OutSpace) vector orthoganal to TwistAxis marking U axis of Twist Coordinate System       <br>
    double            dTwistPeriodDist,          ///< [in ]: distance along the TwistAxis for one complete Twist revolution                                        <br>
    SmTwistVolume   *& rpNewVolume,              ///< [out]: New SmTwistVolume, NULL on input                                                                      <br>
    double      dPeriodMinDist=SM_ZONE_TOL_3D    ///< [in ]: min allowed value dTwistPeriodDist                                                                    <br>
                                                 ///<      : dPeriodMinDist must be >= SM_ZONE_TOL_3D                                                              <br>
  );

  SmStatus GetCanonical
  (
    SmPoint3d  & rTwistOrigin,              ///< [out]: InSpace (and OutSpace) point on the twist center line                                                      <br>
    SmVector3d & rTwistAxis,                ///< [out]: InSpace (and OutSpace) direction of the twist center line marking W axis of Twist Coordinate System        <br>
    SmVector3d & rTwistMidDir,              ///< [out]: InSpace (and OutSpace) vector orthoganal to TwistAxis marking U axis of Twist Coordinate System            <br>
    double     & rdTwistPeriodDist,         ///< [out]: distance along the TwistAxis for one complete Twist revolution                                             <br>
    double     & rdPeriodMinDist            ///< [out]: min allowed value dTwistPeriodDist                                                                         <br>
  ) const ;

  SmStatus SetCanonical
  (
    SmPoint3d  & rTwistOrigin,              ///< [in ]: InSpace (and OutSpace) point on the twist center line                                                        <br>
    SmVector3d & rTwistAxis,                ///< [in ]: InSpace (and OutSpace) direction of the twist center line marking W axis of Twist Coordinate System          <br>
    SmVector3d & rTwistMidDir,              ///< [in ]: InSpace (and OutSpace) vector orthoganal to TwistAxis marking U axis of Twist Coordinate System              <br>
    double       dTwistPeriodDist,          ///< [in ]: distance along the TwistAxis for one complete Twist revolution                                               <br>
    double dPeriodMinDist=SM_ZONE_TOL_3D    ///< [in ]: min allowed value dTwistPeriodDist                                                                           <br>
  ) ; 

  // Simple Data Access

  // Natural Domain in ParamSpace and InSpace - Mapping is defined for all ParamSpace and InSpace
  virtual SmExtent3d GetNaturalParamDomain()   const
  {
    return SmExtent3d( -SM_INFINITE_PARAMETER, -SM_INFINITE_PARAMETER, -SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER );
  }

  // Min Period distance 
  double      GetPeriodMinDist()    const    { return ( m_dPeriodMinDist) ; }

  // Twist Coordinate System in InSpace (and OutSpace) directions
  SmVector3d  GetTwistOrigin()      const    { return ( GetOrientOrigin() ) ; }    // ZeroHeightPoint on helix Z Axis 
  SmVector3d  GetTwistAxis()        const    { return ( GetOrientZAxis() ) ; }     // helix Z Axis
  SmVector3d  GetTwistMidDir()      const    { return ( GetOrientXAxis() ) ; }     // helix X Axis
  SmVector3d  GetTwistBinormal()    const    { return ( GetOrientYAxis() ) ; }     // helix Y Axis: TwistBinormal = TwistAxis * TwistMidDir ;
  double      GetTwistPeriodDist()  const    { return ( m_dTwistPeriodDist ) ; }

  // convenience functions to look like a SmBSplineVolume for cache algorithm - other classes use base default behaviors
  virtual ULONG    GetDegree(SmVolumeParamType eParam) const { SM_REF1(eParam) ; return 3 ; }

  virtual SmStatus GetKnots
  (
    SmVolumeParamType  eParam,                ///< [in ]: one of SM_VP_U, SM_VP_V, SM_VP_W                                    <br>
    SmTArray<double> & rKnots,                ///< [out]: ParamSpace knot list                                                <br>
    SmTArray<ULONG>  * pKnotMults = NULL,     ///< [out]: associated knot multipliticies. NULL to ignore. default:[NULL]      <br>
    const SmExtent1d * pOptIvl = NULL         ///< [in ]: interval of interest, NULL=Natural Interval, default:[NULL]         <br>
  ) const ;

  virtual ULONG    GetNumberControlPoints( SmVolumeParamType eParam ) const
  {
    SM_REF1( eParam ); return 4;
  }

  // make exact BSplineCurve projection of InputSpace Curve to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutCurve
  (
    SmVolumeSpaceTYPE eInputSpace,       ///< [in ]: SM_VS_IN_SPACE   = InputCurve is projected from InSpace      <br>
                                         ///<      : SM_VS_PARAM_SPACE= InputCurve is projected from ParamSpace   <br>
    const SmCurve   & rInputCurve,       ///< [in ]: Curve to project to 1st OutSpace                             <br>
    SmBSplineCurve *& rpNewCurve         ///< [out]: 1st OutSpace projection or NULL for not possible             <br>
  ) const ;

  // make exact BSplineCurve projection of InputSpace Surface to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutSurface
  (
    SmVolumeSpaceTYPE   eInputSpace,   ///< [in ]: SM_VS_IN_SPACE   = InputSurface is projected from InSpace       <br>
                                       ///<      : SM_VS_PARAM_SPACE= InputSurface is projected from ParamSpace    <br>
    const SmSurface   & rInputSurface, ///< [in ]: Surface to project to 1st OutSpace                              <br>
    SmBSplineSurface *& rpNewSurface   ///< [out]: 1st OutSpace projection or NULL for not possible                <br>
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
    const SmPoint3d & crParamPoint,    ///< [in ]: ParamSpace point to OutSpace point                                                 <br>
    ULONG             lHighestDeriv,   ///< [in ]: 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                           <br>
    SmBoolean         bUFromLeft,      ///< NotUsed: [in ]: if P is on U, V, or W interval boundary                                            <br>
    SmBoolean         bVFromLeft,      ///< NotUsed: [in ]: TRUE  = evaluate P in upper interval where P is on the left of the interval        <br>
    SmBoolean         bWFromLeft,      ///< NotUsed: [in ]: FALSE = evaluate P in lower interval where P is on the right of the interval       <br>
    SmVector3d      * aDerivatives,    ///< [out]: matrix of OutSpace evaluations values                                              <br>
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
    SmBoolean bNonZeroTangents=TRUE,   ///< NotUsed: [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors       <br>
                                       ///<      : FALSE= return exact tangent values, default:[TRUE]                                 <br>
                                       ///<      : note: Surprisingly TRUE is the common choice because most tangent uses             <br>
                                       ///<      :       are for their direction (Binorm, SurfNorm comps), but when the               <br>
                                       ///<      :       tangent is being used for its magnitude (like an arc-length comp)            <br>
                                       ///<      :       then set this to FALSE.                                                      <br>
                                       ///<      : default:[TRUE]                                                                     <br>
    SmBoolean bDoZeroSampling=TRUE,    ///< NotUsed: [in ]: for internal use only, always set to TRUE, default:[TRUE]                          <br>
    SmBoolean bWithCompounding=TRUE    ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]          <br>
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

  // compute ProjPoint = EvaluateSimple(ParamPoint) - requires derived implementation
  virtual SmStatus EvaluateSimple
  (
    const SmPoint3d & crParamPoint,         ///< [in ]: ParamSpace point to map to ProjSpace point                                       <br>
    ULONG             lHighestDeriv,        ///< [in ]: 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                         <br>
    SmBoolean         bUFromLeft,           ///< NotUsed: [in ]: if P is on U, V, or w interval boundary                                          <br>
    SmBoolean         bVFromLeft,           ///< NotUsed: [in ]: TRUE  = evaluate P in upper interval where P is on the left of the interval      <br>
    SmBoolean         bWFromLeft,           ///< NotUsed: [in ]: FALSE = evaluate P in lower interval where P is on the right of the interval     <br>
    SmVector3d      * aDerivatives,         ///< [out]: matrix of ParamSpace evaluations values                                          <br>
                                            ///<      : 3d organized: D[u][v][w]                                                         <br>
                                            ///<      : 1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3               <br>
                                            ///<      : sized       : [n+1][n+1][n+1], where n=lHighesDeriv                              <br>
                                            ///<      : lHghDrv = 0,   sized: [1],                                                       <br>
                                            ///<      :   i=0          order: [D]                                                        <br>
                                            ///<      : lHghDrv = 1,   sized: [8]                                                        <br>
                                            ///<      :   i=u*4+v*2+w  order: [D  Dw  Dv  ---                                            <br>
                                            ///<      :                        Du --- --- ---]                                           <br>
                                            ///<      : lHghDrv = 2,   sized: [27]                                                       <br>
                                            ///<      :   i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---                       <br>
                                            ///<      :                        Du  Duw --- Duv --- --- --- --- ---                       <br>
                                            ///<      :                        Duu --- --- --- --- --- --- --- ---]                      <br>
                                            ///<      : lHghDrv = 3,   sized: [81]                                                       <br>
                                            ///<      :   i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw         <br>
                                            ///<      :                        --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---         <br>
                                            ///<      :                        Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---         <br>
                                            ///<      :                        --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---         <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  Duuu ---         <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---         <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---         <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---         <br>
                                            ///<      :                        --- --- ]                                                 <br>
    SmBoolean bNonZeroTangents=TRUE,        ///< NotUsed: [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors     <br>
                                            ///<      : FALSE= return exact tangent values, default:[TRUE]                               <br>
                                            ///<      : note: Surprisingly TRUE is the common choice because most tangent uses           <br>
                                            ///<      :       are for their direction (Binorm, SurfNorm comps), but when the             <br>
                                            ///<      :       tangent is being used for its magnitude (like an arc-length comp)          <br>
                                            ///<      :       then set this to FALSE.                                                    <br>
                                            ///<      : default:[TRUE]                                                                   <br>
    SmBoolean bDoZeroSampling=TRUE          ///< NotUsed: [in ]: for internal use only, always set to TRUE, default:[TRUE]
  ) const ;

  // compute ProjSpace BBoxes for given ParamSpace Box
  virtual SmStatus EvaluateBoundingBoxSimple
  (
    const SmExtent3d  & crParamDomain,                   ///< [in ]: ParamSpace BBox to project to Project Space                                                           <br>
    SmPseudoBox       * pOptPseudoParamBox = NULL,       ///< NotUsed: [in ]: optional ParamSpace PseudoBox used to set output PseudoBox orientations,                              <br>
                                                         ///<      : NULL   : ProjPseudoBox Basis = Project X Y Z ParamVecs to ProjSpace at crParamBox Center              <br>
                                                         ///<      : NotNULL: ProjPseudoBox Basis = Project pOptParamPseudoBox BasisVecs to ProjSpace at crParamBox Center <br>
                                                         ///<      : default:[NULL]                                                                                        <br>
    SmExtent3d        * pNormalProjBox = NULL,           ///< [out]: ProjectSpace Axis aligned box                                                                         <br>
    SmPseudoBox       * pPseudoProjBox = NULL            ///< [out]: ProjectSpace Non-axis aligned box
  ) const ;

  // Create ProjSpace IsoCurve from a ParamSpace IsoParamLine - requires derived implementation
  virtual SmStatus EvaluateIsoParametricCurveSimple
  (
    const SmContext     & crContext,            ///< [in ]: context for created objects                                <br>
    SmVolumeParamsType    eConstantParams,      ///< [in ]: oneof: SM_VPS_UV_IN,                                       <br>
                                                ///<      :        SM_VPS_UW_IN,                                       <br>
                                                ///<      :        SM_VPS_VW_IN.                                       <br>
    double                dIsoParam1,           ///< [in ]: 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value      <br>
    double                dIsoParam2,           ///< [in ]: 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value      <br>
    double                d3DTolerance,         ///< [in ]: Max ApproxCurve to IdealCurve deviation                    <br>
    SmCurve            *& rpNewIsoCurve,        ///< [out]: the ProjSpace IsoCurve                                     <br>
    const SmExtent3d    * pOptParamDomain=NULL  ///< [in ]: limiting domain, NULL to ignore. default:[NULL]            <br>
  ) const ;

  // Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane - requires derived implementation
  virtual SmStatus EvaluateIsoParametricSurfaceSimple
  (
    const SmContext   & crContext,              ///< [in ]: context for created objects                          <br>
    SmVolumeParamType   eConstantParam,         ///< [in ]: oneof: SM_VP_U_IN,                                   <br>
                                                ///<      :        SM_VP_V_IN,                                   <br>
                                                ///<      :        SM_VP_W_IN                                    <br>
    double              dIsoParam,              ///< [in ]: constant param value                                 <br>
    double              d3DTolerance,           ///< [in ]: Max ApproxSurface to IdealSurface deviation          <br>
    SmSurface        *& rpNewIsoSurface,        ///< [out]: the ProjSpace IsoSurface                             <br>
    const SmExtent3d  * pOptParamDomain=NULL    ///< [in ]: limiting domain, NULL to ignore. default:[NULL]      <br>
  ) const ;

  // get approximate ParamSpace location for given ProjSpace point for upcoming newton raphson search
  virtual SmStatus InvEvaluateGuessPointSimple
  (
    const SmPoint3d     & crProjPoint,        ///< [in ]: ProjSpace Point to map back to ParamSpace Point                      <br>
    SmTArray<SmPoint3d> & rGuessParamPoints   ///< [out]: ParamSpace Point near Target Point actual map back to ParamSpace     <br>
  ) const ;

  // General Point Solver without a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  virtual SmStatus GlobalPointSolveSimple
  (
    const SmPoint3d  & crProjPoint,          ///< [in ]: ProjSpace Point to map back to ParamSpace                                             <br>
    SmBoolean        & rbFoundAnswer,        ///< [out]: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point                      <br>
    SmSolutionArray  & rSolutions,           ///< [out]: Contains ParamSpace Found Point                                                       <br>
                                             ///<      : rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                           <br>
                                             ///<      : rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);  <br>
                                             ///<      : rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location          <br>
                                             ///<      : rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location          <br>
                                             ///<      : rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location          <br>
    const SmExtent3d * pOptParamDomain=NULL  ///< [in ]: ParamSpace domain over which to search for the inverse point                          <br>
                                             ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                        <br>
  ) const;

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
                                             ///<      :NULL = use Map's NaturalDomain, default:[NULL]                                         <br>
  ) const ;                                   

  // IsBounded = TRUE because NaturalParamDomain is limited to U>0 halfspace
  virtual SmBoolean IsBoundedSimple      () const ;

  // IsClosed = !ClosedU, ClosedV, !ClosedW. TwistVolumes are periodic over v = [-d*Pi, +d*Pi ] - a wedge, not a box as is typical
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

  // IsPeriodic = !PeriodicU, PeriodicV, !PeriodicW. TwistVolumes are periodic over v = [-d*Pi, +d*Pi ] - a wedge, not a box as is typical
  virtual SmBoolean IsPeriodicSimple   
  (
    SmBoolean        & rbPeriodicU,            ///< [out]: TRUE = G1 or better in U Dir                                      <br>
    SmBoolean        & rbPeriodicV,            ///< [out]: TRUE = G1 or better in V Dir                                      <br>
    SmBoolean        & rbPeriodicW,            ///< [out]: TRUE = G1 or better in W Dir                                      <br>
    const SmExtent3d * pOptParamDomain = NULL  ///< NotUsed: [in ]: ParamSpace domain over which to search for the inverse point      <br>
                                               ///<      : NULL = use Map's NaturalDomain, default:[NULL]                    <br>
  ) const ;     

  virtual SmBoolean IsSingularitySimple
  (
    const SmPoint3d   & crParamPoint,       ///< NotUsed: [in ]: UVpoint to test                                     <br>
    SmBoolean         & rbSingularU,        ///< [out]: TRUE = [Su=0]                                       <br>
    SmBoolean         & rbSingularV,        ///< [out]: TRUE = [Sv=0]                                       <br>
    SmBoolean         & rbSingularW,        ///< [out]: TRUE = [Sw=0]                                       <br>
    double              d3dTol=SM_EFF_ZERO  ///< NotUsed: [in ]: min 3d distance between distinct points) const;     <br>
  ) const ;

  virtual SmBoolean IsOnBoundarySimple 
  (
    const SmPoint3d  & crParamPoint,           ///< NotUsed: [in ]: Volume UVWPoint to test                                     <br>
    SmBoolean        & rbOnU,                  ///< [out]: TRUE = TargetPoint on UMin or UMax                          <br>
    SmBoolean        & rbOnV,                  ///< [out]: TRUE = TargetPoint on VMin or VMax                          <br>
    SmBoolean        & rbOnW,                  ///< [out]: TRUE = TargetPoint on WMin or WMax                          <br>
    double           * pdOptTolerance = NULL,  ///< NotUsed: [in ]: max deviation allowed for point on seam                     <br>
                                               ///<      : NULL = use SM_EFF_ZERO * 1000 * (1 + maxDimension())        <br>
    const SmExtent3d * pOptParamDomain = NULL  ///< NotUsed: [in ]: ParamSpace domain over which to search for the inverse point<br>
                                               ///<      : NULL = use Map's NaturalDomain, default:[NULL]              <br>
  ) const ;                                                                                                     

  // rtn: TRUE if ParamPoint is within the maps paramSpace Natural domain
  virtual SmBoolean IsPointInParamDomainSimple(const SmPoint3d  & crParamPoint) const ; // NotUsed: in : crParamPoint

  // rtn: TRUE if projPoint can be inverted uniquely to DomainSpace
  virtual SmBoolean IsPointInProjDomainSimple
  (
    const SmPoint3d  & crProjPoint,           ///< [in ]: project space point to test for inversion                    <br>
    SmPoint3d   * pOptParamPoint = NULL       ///< [out]: if the point is invertible, go ahead and get the inverse     <br>
  ) const ;

  // rtn: TRUE if ParamLine is within the maps paramSpace Natural domain
  virtual SmBoolean IsLineInParamDomainSimple
  (
    const SmPoint3d  & crStartParamPoint,      ///< NotUsed: [in ]: ParamSpace Line StartPoint to test for inclusion     <br>
    const SmPoint3d  & crEndParamPoint         ///< NotUsed: [in ]: ParamSpace Line EndPoint to test for inclusion       <br>
  ) const ;

  // ToBe implemented: base implementations on projecting line to ParamSpace and testing IsCurveInParamDomainSimple
  //  virtual SmBoolean IsLineInProjDomainSimple(const SmPoint3d  & crStartProjPoint,    // rtn: TRUE if projPoint can be inverted uniquely to ParamSpace
  //                                             const SmPoint3d  & crEndProjPoint,      ///< [in ]: LineSeg End projPoint
  //                                             SmPoint3d        * pOptStartParamPoint, ///< [out]: if the point is invertible, go ahead and get the inverse
  //                                             SmPoint3d        * pOptEndParamPoint)   ///< [out]: if the point is invertible, go ahead and get the inverse
  //                                            const

  virtual SmBoolean IsCurveInParamDomainSimple(const SmCurve & crParamCurve) const ; // NotUsed: in : crParamCurve

  virtual SmBoolean IsSurfaceInParamDomainSimple(const SmSurface & crParamSurface) const ; // NotUsed: in : crParamSurface

public:
  // utilities

  // I/O assist
  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,             ///< [in ]: target output stream                                 <br>
    ULONG lDBVersionNumber          ///< [in ]: database version to get proper sequence of writes    <br>
  ) const ; 

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,              ///< NotUsed: [in ]: Object type to be read                                                       <br>
    SmDatabaseIO    & rDB,                ///< [in ]: target output stream                                                         <br>
    const SmContext & crContext,          ///< [in ]: context for new object construction                                          <br>
    SmVolume        *&rpNewVolume,        ///< [out]: NULL on input = new object allocated in this routine built from stream data  <br>
                                          ///<      : NotNULL on input = pointer to an empty object to be filled by this routine   <br>
    ULONG             lDBVersionNumber    ///< [in ]: database version to get proper sequence of writes                            <br>
  ) ; 

  // get memory used for volume and its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,         ///< [out]: bigger size of all allocated memory in bytes  <br>
    SmMarkType eMarkType=SM_MT_NOMARK     ///< [in ]: uses without increment eMarkType value        <br>
  ) const ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmTwistVolume, SmVolume, SmTwistVolume_TYPE) ;

  // pretty print SmVolume
  virtual void Dump(SmBoolean bAbbrev, ULONG lIndentCnt) const ;

  // AssertValid
  virtual SmDisplayList * Draw
  (
    SmBoolean       bShowOutSpace=TRUE,        ///< [in ]: TRUE = draw point = Evaluate(sParam)), FALSE=don't                               <br>
    SmBoolean       bShowInSpace = FALSE,      ///< [in ]: TRUE = draw point = Orient(sParam), FALSE=don't                                  <br>
    SmExtent3d    * pOptParamDomain = NULL,    ///< [in ]: optionally output an XYPlane subDomain graphic, NULL to ignore                   <br>
    SmGfxArraySet * pOptGfxSet = NULL          ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.  <br>
  ) const;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                           <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                             <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 <br>
  ) const ;                                                                                                                                                

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

} ; // end class SmTwistVolume


// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmTwistVolume*)

#endif // !__SMTWISTVOLUME_H__


