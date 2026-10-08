// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBendVolume.h
* PURPOSE: Header file for the Bend Volume class.
* Oct 2006 GWC Author
**********************************************************************/

#ifndef __SMBENDVOLUME_H__
#define __SMBENDVOLUME_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMVOLUME_H__
#include <SmVolume.h>
#endif

class SmBSplineCurve ;
class SmDisplayList ;

/*******************************************************************//**
PURPOSE: The SmBendVolume derived SmVolume object represents a Bend, mapping
         positive U=const planes into Cylinders centered on the Z axis.
         The behavior can be used to mimic the shape of the bend in
         a piece of sheet metal that has been bent in a break press.

  NOTICE: this mapping is multivalued and locally degenerate.
    1. multivalued       : points separated by 2Pi*d (d=BendNeutralDist) on line segments
                           parallel to the BendBinormal map to the same point.
    2. locally degenerate: all points on the u = 0 plane, map to points on the bend axis line,
                           but with undefined derivatives.  For that plane there is no concept
                           of how many times a v-length line-segment wraps around a zero radius
                           cylinder; perhaps an infinite number. As such the domain of this mapping
                           is limited to the 1/2 space defined by u > 0.
                           Objects mapped through the BendVolume should fit within
                           one period (2Pi*d) along the V axis.
 EvaluateSimple Map:
 Map UVW ParamSpace point to xyz ProjSpace point and iso-derivatives.

  EvaluateSimple : r      = u ;
                   theta  = v/d ;
                   d      = BendNeutralDist ;
  Position
    PProj  = BendCenter + u*Cos(v/d)*BendMidDir
                        + u*Sin(v/d)*BendBinormal
                        + w*BendAxis ;

  1st Derivs
    Du(PProj)  =      Cos(v/d)*BendMidDir +     Sin(v/d)*BendBinormal ;
    Dv(PProj)  = u/d*-Sin(v/d)*BendMidDir + u/d*Cos(v/d)*BendBinormal ;
    Dw(PProj)  = BendAxis ;

  2nd Derivs
    Duu(PProj) = 0 ;
    Duv(PProj) =   -1/d*Sin(v/d)*BendMidDir +    1/d*Cos(v/d)*BendBinormal ;
    Duw(PProj) = 0 ;
    Dvv(PProj) = -u/d/d*Cos(v/d)*BendMidDir + -u/d/d*Sin(v/d)*BendBinormal ;
    Dvw(PProj) = 0 ;
    Dww(PProj) = 0 ;

  3rd Derivs
    Duuu = 0 ;
    Duuv = 0 ;
    Duuw = 0 ;
    Duvv =   -1/d/d*Cos(v/d)*BendMidDir +   -1/d/d*Sin(v/d)*BendBinormal ;
    Duvw = 0 ;
    Duww = 0 ;
    Dvvv =  u/d/d/d*Sin(v/d)*BendMidDir + -u/d/d/d*Cos(v/d)*BendBinormal ;
    Dvvw = 0 ;
    Dvww = 0 ;
    Dwww = 0 ;

NOTES:  The Bend's axis is aligned with the ParamSpace W Axis.
        The Bend's NeutralPlane is offset in the U direction by the BendNeutralDistance.
        The Bend's NeutralPlane is parallel to the ParamSpace V/W plane.

        ParameterSpace planes parallel to the NeutralPlane are mapped to ProjSpace
        Z-axis-centered cylinders.

  The SmBendVolume class implements the SmVolume virtual methods that
    define the ParamSpace -> ProjSpace portion of the overall
    InSpace -> ParamSpace -> ProjSpace -> OutSpace mapping.

  Bending consists of sending the points of an object's
  shape through the overall InSpace to Outpace mapping.

  It's the reverse effect of the SmUnbendVolume object.

  A bend is defined by a unit-normal bend coordinate system (containing a
  BendOrigin point, a BendAxis vector, and a BendMidDirection vector) and
  a BendNeutralDistance distance. The BendAxis and the BendMidDir are
  perpendicular to one another both before and after the mapping.

  o. The BendOrigin OutSpace point is a point on the bend's axis.
  o. The BendAxis OutSpace vector points along the length of the bend.
  o. The BendMidDir OutSpace vector points to the symmetric center of the bend from the BendAxis.
       In other words, the bend mapping is symmetric about the plane spanned by the
       BendAxis and the BendMidDir vectors.
  o. A BendBinormal OutSpace direction is defined as BendBinormal = BendAxis * BendMidDir.
  o. The Bend's OutSpace NeutralDistance defines the distance from the BendAxis to the bend's neutral plane.
  o. The Bend's OutSpace NeutralPlane is defined as the plane normal to the BendMidDir
       and offset from the BendOrigin by the BendNeutralDistance.

  The Bend mapping is only defined over a half-space.
    In ParamSpace, all points where the U coordinate is greater than zero.
    In InSpace, all points from the BendAxis in the general direction of the BendMidDir.

  In ParamSpace, the bend is built about the ParamSpace coordinate system as,

    ParamSpace BendOrigin    = ParamPt (0,0,0)
    ParamSpace BendAxis      = ParamVec(0,0,1)
    ParamSpace BendMidDir    = ParamVec(1,0,0)
    ParamSpace BendBinormal  = ParamVec(0,1,0)

  In InSpace, the bend is oriented and positioned by the Orient() mapping
  between orthogonal coordinate systems so that,

    InSpace    BendOrigin    = GetBendOrigin()    (stored in m_pOrientMap->GetToDisp())
    InSpace    BendAxis      = GetBendAxis()      (stored in m_pOrientMap->GetToZAxis())
    InSpace    BendMidDir    = GetBendMidDir()    (stored in m_pOrientMap->GetToXAxis())
    InSpace    BendBinormal  = BendAxis * BendMidDir

  In OutSpace, the bend has the same orientation and location as it does in InSpace so that,

    OutSpace   BendOrigin    = GetBendOrigin()    (stored in m_pOrientMap->GetToDisp())
    OutSpace   BendAxis      = GetBendAxis()      (stored in m_pOrientMap->GetToZAxis())
    OutSpace   BendMidDir    = GetBendMidDir()    (stored in m_pOrientMap->GetToXAxis())
    OutSpace   BendBinormal  = BendAxis * BendMidDir

 (The Bend and unBend mappings have the same descriptions, positions, and orientations.)

 THE BEND MAP (Same as the SmUnbendVolume Map Inversion)

  In the InSpace->OutSpace bend mapping:
    o. InSpace lines parallel to the BendBinormal map to OutSpace circular arcs normal to and centered on the BendAxis.
       - ArcLength change depends on distance from the BendCenter
         o. less    than NeutralDistance, ArcLength shortens
         o. equal   to   NeutralDistance, ArcLength constant
         o. greater than NeutralDistance, ArcLength lengthens
    o. InSpace lines parallel to the BendAxis map to OutSpace lines parallel to the BendAxis.
    o. InSpace lines parallel to the BendMidDir map to OutSpace lines that radiate out
         radially from the BendAxis.

  In the ParamSpace->ProjSpace portion of the bend mapping.
    o. ParamSpace varying V isoLines map to circular arcs centered on the ProjSpace Z Axis
    o. ParamSpace varying W isoLines map to ProjSpace varying Z isolines.
    o. ParamSpace varying U isoLines map to ProjSpace lines radiating radially from the ProjSpace Z Axis.

  As an interesting detail, geometry points on BendAxis/BendMidDir plane are not moved
  by the bend mapping.

  The bend mappings between InSpace (xIn,yIn,zIn), ParamSpace (u,v,w),
  ProjSpace (xProj,yProj,zProj), and OutSpace (xOut,yOut,zOut) spaces are
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Bend Mapping      | FromSpace | ToSpace | mapping                                               |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | EvaluateSimple    | u         | xProj   | r      = u ;                                          |
  |                   | v         | yProj   | theta  = v/d ;                                        |
  |                   | w         | zProj   | d      = BendNeutralDist ;                            |
  |                   |           |         | xProj  = r*Cos(theta) ;                               |
  |                   |           |         | yProj  = r*Sin(theta) ;                               |
  |                   |           |         | zProj  = w ;                                          |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvEvaluateSimple | xProj     | u       | r     = sqrt(xProj**2 + yProj**2) ;                   |
  |                   | yProj     | v       | theta = ArcTan(yProj,xProj) ; (ThetaIvl:[-Pi Pi])
  +|
  |                   | zProj     | w       | d     = BendNeutralDist ;                             |
  |                   |           |         | u     = r ;                                           |
  |                   |           |         | v     = d * theta ;                                   |
  |                   |           |         | w     = zProj ;                                       |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Orient            | u         | xIn     | Pin = BendCenter + u*BendMidDir                       |
  |                   | v         | yIn     |                  + v*BendBinormal                     |
  |                   | w         | zIn     |                  + w*BendAxis ;                       |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvOrient         | xIn       | u       | u = (PxyzIn - BendCenter).Dot(BendMidDir)             |
  |                   | yIn       | v       | v = (PxyzIn - BendCenter).Dot(BendBinormal)           |
  |                   | zIn       | w       | w = (PxyzIn - BendCenter).Dot(BendAxis)               |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Evaluate          | u         | xOut    | r     = u ;                                           |
  |                   | v         | yOut    | theta = v/d ;                                         |
  |                   | w         | zOut    | d     = BendNeutralDist ;                             |
  |                   |           |         | POut  = BendCenter + r*Cos(theta)*BendMidDir          |
  |                   |           |         |                    + r*Sin(theta)*BendBinormal        |
  |                   |           |         |                    + w*BendAxis ;                     |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvEvaluate       | xOut      | u       | xProj = (PxyzOut - BendCenter).Dot(BendMidDir) ;      |
  |                   | yOut      | v       | yProj = (PxyzOut - BendCenter).Dot(BendBinormal) ;    |
  |                   | zOut      | w       | zProj = (PxyzOut - BendCenter).Dot(BendAxis) ;        |
  |                   |           |         | r     = sqrt(xProj**2 + yProj**2)                     |
  |                   |           |         | theta = ArcTan(yProj,xProj) ;                         |
  |                   |           |         | d     = BendNeutralDist ;                             |
  |                   |           |         | u     = r ;                                           |
  |                   |           |         | v     = d * theta ;                                   |
  |                   |           |         | w     = zProj ;                                       |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | Map               | xIn       | xOut    |  r      = (PxyzIn - BendCenter).Dot(BendMidDir) ;     |
  |                   | yIn       | yOut    |  theta  = (PxyzIn - BendCenter).Dot(BendBinormal)/d ; |
  |                   | zIn       | zOut    |  w      = (PxyzIn - BendCenter).Dot(BendAxis) ;       |
  |                   |           |         |  d      = BendNeutralDist ;                           |
  |                   |           |         |  pOut   = BendCenter + r*Cos(theta)*BendMidDir        |
  |                   |           |         |                      + r*Sin(theta)*BendBinormal      |
  |                   |           |         |                      + w*BendAxis ;                   |
  +-------------------+-----------+---------+-------------------------------------------------------+
  | InvMap            | xOut      | xIn     | xProj = (PxyzOut - BendCenter).Dot(BendMidDir) ;      |
  |                   | yOut      | yIn     | yProj = (PxyzOut - BendCenter).Dot(BendBinormal) ;    |
  |                   | zOut      | zIn     | zProj = (PxyzOut - BendCenter).Dot(BendAxis) ;        |
  |                   |           |         | r     = sqrt(xProj**2 + yProj**2)                     |
  |                   |           |         | theta = ArcTan(yProj,xProj) ;                         |
  |                   |           |         | d     = BendNeutralDist ;                             |
  |                   |           |         | PIn   = BendCenter + (r)         * BendMidDir         |
  |                   |           |         |                    + (d * theta) * BendBinormal       |
  |                   |           |         |                    + (zProj)     * BendAxis ;         |
  +-------------------+-----------+---------+-------------------------------------------------------+

  WARNINGS: this mapping is
  1. multivalued       : points separated by 2Pi*d on line segments parallel
                         to the BendBinormal map to the same point.
  2. locally degenerate: all points on the u = 0 plane, map to points on the bend axis line,
                         but with undefined derivatives.  For that plane there is no concept
                         of how many times a v-length line-segment wraps around a zero radius
                         cylinder; perhaps an infinite number. As such the domain of this mapping
                         is limited to the 1/2 space defined by u > 0.
***********************************************************************/
class SM_EXPORT SmBendVolume : public SmVolume
{
protected:
  double  m_dBendNeutralDist = SM_UNDEF_DOUBLE ; // distance to BendNeutralPlane along BendMidDir from BendOrigin.
  double  m_dSingularityTol  = SM_UNDEF_DOUBLE ; // min dist between NonSingular pts and Volume's singularity at U=0,
                                                 // dSingularityTol must be >= SM_ZONE_TOL_3D, default:[SM_ZONE_TOL_3D]

  // inherited from SmVolume::
  //   // BendOrigin    = m_pOrient->GetToDisp()   // InSpace (and OutSpace) point on the bend center line
  //   // BendAxis      = m_pOrient->GetToZAxis()  // InSpace (and OutSpace) unit-vector direction of the bend center line
  //   // BendBinormal  = m_pOrient->GetToYAxis()  // InSpace (and OutSpace) unit-vector = BendAxis * BendMidDir
  //   // BendMidDir    = m_pOrient->GetToXAxis()  // InSpace (and OutSpace) unit-vector orthoganal to BendAxis marking start of Bend Coordinate system
                                                   //  note: The BendMidDir/BendBinormal plane is the symmetric plane
                                                   //        of the Bend mapping

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
  //        FindParamIntervalForInSpaceLineSimple() // Opt implementation: Find portion of InSpace Line interval that maps to the NaturalParamDomain, default rTrimIvl = crCurrentIvl
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
  SmBendVolume(const SmContext  & crContext,                        // in : context for new OrientMap construction
               const SmPoint3d  & rBendCenter,                      // in : InSpace (and OutSpace) point on the bend center line
               const SmVector3d & rBendAxis,                        // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System
               const SmVector3d & rBendMidDir,                      // in : InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System
               double             dBendNeutralDist,                 // in : InSpace (and OutSpace) distance along BendMidDir from BendOrigin to NeutralPlane.
               double             dSingularityTol=SM_ZONE_TOL_3D) ; // in : min dist between NonSingular pts and Volume's singularity at U=0,
                                                                    //      dSingularityTol must be >= SM_ZONE_TOL_3D
                                                                    //      note: The BendMidDir/BendBinormal plane is the symmetric plane of the Bend mapping

  // copy constructor  // copy constructors
  // SmBendVolume(const SmBendVolume & crSourceVolume)   // in : SourceVolume to copy (copies all compounding volumes)
  //                                                     //      { SmBendVolume::SmBendVolume(crSourceVolume, FALSE) ; }

  SmBendVolume(const SmBendVolume & crSourceVolume,         // in : SourceVolume to copy
               SmBoolean            bSimpleMapOnly=FALSE) ; // in : TRUE = Copy this Volume omitting any compounding volumes
                                                            //      FALSE= Copy this Volumes with any compounding volumes

  // empty constructor for I/O
  SmBendVolume(const SmContext * cpContext=NULL) : m_dBendNeutralDist(0.0)
                                                 { if(cpContext) { SetContext(cpContext) ; } }

  // make an exact copy of any volume with or without any compounding volumes
  virtual SmStatus Copy(const SmContext & crContext,            // in : context for new object construction
                        SmVolume       *& rpNewVolume,          // out: The copied Volume
                        SmBoolean         bSimpleMapOnly=FALSE  // in : TRUE = Copy this Volume omitting any compounding volumes
                       ) const ;                                //      FALSE= Copy this Volumes with any compounding volumes
                                                 

  // assignment operator
  SmBendVolume &operator=(const SmBendVolume &crBendVolume) ;

  // destructor
  virtual ~SmBendVolume() { }

  // equality operator
  virtual SmBoolean operator==(const SmVolume&) const ;

  static SmStatus CreateCanonical(const SmContext & crContext,                        // in : context for new object construction
                                  SmPoint3d       & rBendCenter,                      // in : InSpace (and OutSpace) point on the bend center line
                                  SmVector3d      & rBendAxis,                        // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System
                                  SmVector3d      & rBendMidDir,                      // in : InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System
                                  double            dBendNeutralDist,                 // in : InSpace (and OutSpace) distance along rBendMidDir between BendOrigin and NeutralPlane.
                                  SmBendVolume   *& rpNewVolume,                      // out: New SmBendVolume, NULL on input
                                  double            dSingularityTol=SM_ZONE_TOL_3D) ; // in : min dist between NonSingular pts and Volume's singularity at U=0,
                                                                                      //      dSingularityTol must be >= SM_ZONE_TOL_3D

  SmStatus        GetCanonical   (SmPoint3d  & rBendCenter,                           // out: InSpace (and OutSpace) point on the bend center line
                                  SmVector3d & rBendAxis,                             // out: InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System
                                  SmVector3d & rBendMidDir,                           // out: InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System
                                  double     & rdBendNeutralDist,                     // out: InSpace (and OutSpace) distance along BendMidDir between BendOrigin and NeutralPlane.
                                  double     & rdSingularityTol                       // out: min ProjSpace dist between NonSingular pts and the Volume's singularity at the ParameterPlane U=0.
                                 ) const ;

  SmStatus        SetCanonical   (SmPoint3d  & rBendCenter,                           // in : InSpace (and OutSpace) point on the bend center line
                                  SmVector3d & rBendAxis,                             // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System
                                  SmVector3d & rBendMidDir,                           // in : InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System
                                  double       dBendNeutralDist,                      // in : InSpace (and OutSpace) distance along BendMidDir between BendOrigin and NeutralPlane.
                                  double       dSingularityTol=SM_ZONE_TOL_3D) ;      // in : min ProjSpace dist between NonSingular pts and the Volume's singularity at the ParameterPlane U=0.

  // Simple Data Access

  // Natural Domain in ParamSpace and InSpace
  virtual SmExtent3d GetNaturalParamDomain()   const ;

  // Min U distance to singularity plane at U = 0
  double      GetLegalMinU()       const { return ( m_dSingularityTol - SM_EFF_ZERO) ; }
  double      GetSingularityTol()  const { return ( m_dSingularityTol) ; }

  // Bend Coordinate System in InSpace (and OutSpace) directions
  SmVector3d  GetBendOrigin()      const { return ( GetOrientOrigin() ) ; }
  SmVector3d  GetBendAxis()        const { return ( GetOrientZAxis() ) ; }
  SmVector3d  GetBendMidDir()      const { return ( GetOrientXAxis() ) ; }
  SmVector3d  GetBendBinormal()    const { return ( GetOrientYAxis() ) ; }     // BendBinormal = BendAxis * BendMidDir ;
  double      GetBendNeutralDist() const { return ( m_dBendNeutralDist ) ; }

  // convenience functions to look like a SmBSplineVolume for cache algorithm - other classes use base default behaviors
  virtual ULONG    GetDegree(SmVolumeParamType eParam) const { SM_REF1(eParam) ; return 3 ; }

  virtual SmStatus GetKnots(SmVolumeParamType  eParam,             // in : one of SM_VP_U, SM_VP_V, SM_VP_W
                            SmTArray<double> & rKnots,             // out: ParamSpace knot list
                            SmTArray<ULONG>  * pKnotMults = NULL,  // out: associated knot multipliticies. NULL to ignore. 
                            const SmExtent1d * pOptIvl = NULL)     // NotUsed: in : interval of interest, NULL=Natural Interval
                           const ;

  virtual ULONG    GetNumberControlPoints(SmVolumeParamType eParam) const { SM_REF1(eParam) ; return 4 ; }


  // make exact BSplineCurve projection of InputSpace Curve to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutCurve(SmVolumeSpaceTYPE eInputSpace,   // in : SM_VS_IN_SPACE   = InputCurve is projected from InSpace
                                                                                //      SM_VS_PARAM_SPACE= InputCurve is projected from ParamSpace
                                                const SmCurve   & rInputCurve,  // in : Curve to project to 1st OutSpace
                                                SmBSplineCurve *& rpNewCurve)   // out: 1st OutSpace projection or NULL for not possible
                                               const ;

  // make exact BSplineCurve projection of InputSpace Surface to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutSurface(SmVolumeSpaceTYPE   eInputSpace,    // in : SM_VS_IN_SPACE   = InputSurface is projected from InSpace
                                                                                     //      SM_VS_PARAM_SPACE= InputSurface is projected from ParamSpace
                                                  const SmSurface   & rInputSurface, // in : Surface to project to 1st OutSpace
                                                  SmBSplineSurface *& rpNewSurface)  // out: 1st OutSpace projection or NULL for not possible
                                                 const ;

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
  virtual SmStatus Evaluate(const SmPoint3d & crParamPoint,    // in : ParamSpace point to OutSpace point
                            ULONG             lHighestDeriv,   // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3
                            SmBoolean         bUFromLeft,      // NotUsed: in : if P is on U, V, or W interval boundary
                            SmBoolean         bVFromLeft,      // NotUsed: in : TRUE  = evaluate P in upper interval where P is on the left of the interval
                            SmBoolean         bWFromLeft,      // NotUsed: in : FALSE = evaluate P in lower interval where P is on the right of the interval
                            SmVector3d      * aDerivatives,    // out: matrix of OutSpace evaluations values
                                                               //      3d organized: D[u][v][w]
                                                               //      1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3
                                                               //      sized       : [n+1][n+1][n+1], where n=lHighesDeriv
                                                               //      lHghDrv = 0,   sized: [1],
                                                               //      i=0          order: [D]
                                                               //      lHghDrv = 1,   sized: [8]
                                                               //        i=u*4+v*2+w  order: [D  Dw  Dv  ---
                                                               //                           Du --- --- ---]
                                                               //      HghDrv = 2,   sized: [27]
                                                               //       i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---
                                                               //                            Du  Duw --- Duv --- --- --- --- ---
                                                               //                            Duu --- --- --- --- --- --- --- ---]
                                                               //      HghDrv = 3,   sized: [81]
                                                               //       i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw
                                                               //                            --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---
                                                               //                            Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---
                                                               //                            --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---
                                                               //                            --- ---  ---   ---   ---  ---  ---  ---  Duuu ---
                                                               //                            --- ---  ---   ---   ---  ---  ---  ---  ---- ---
                                                               //                            --- ---  ---   ---   ---  ---  ---  ---  ---- ---
                                                               //                            --- ---  ---   ---   ---  ---  ---  ---  ---- ---
                                                               //                            --- --- ]
                            SmBoolean bNonZeroTangents=TRUE,   // NotUsed: in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                                               //      FALSE= return exact tangent values, default:[TRUE]
                                                               //      note: Surprisingly TRUE is the common choice because most tangent uses
                                                               //            are for their direction (Binorm, SurfNorm comps), but when the
                                                               //            tangent is being used for its magnitude (like an arc-length comp)
                                                               //            then set this to FALSE.
                            SmBoolean bDoZeroSampling=TRUE,    // NotUsed: in : for internal use only, always set to TRUE, default:[TRUE]
                            SmBoolean bWithCompounding=TRUE)   // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
                          const ;

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
    const SmPoint3d & crParamPoint,      // in : ParamSpace point to map to ProjSpace point
    ULONG             lHighestDeriv,     // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3
    SmBoolean         bUFromLeft,        // NotUsed: in : if P is on U, V, or w interval boundary
    SmBoolean         bVFromLeft,        // NotUsed: in : TRUE  = evaluate P in upper interval where P is on the left of the interval
    SmBoolean         bWFromLeft,        // NotUsed: in : FALSE = evaluate P in lower interval where P is on the right of the interval
    SmVector3d      * aDerivatives,      // out: matrix of ParamSpace evaluations values
                                         //      3d organized: D[u][v][w]
                                         //      1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3
                                         //      sized       : [n+1][n+1][n+1], where n=lHighesDeriv
                                         //      lHghDrv = 0,   sized: [1],
                                         //        i=0          order: [D]
                                         //      lHghDrv = 1,   sized: [8]
                                         //        i=u*4+v*2+w  order: [D  Dw  Dv  ---
                                         //                             Du --- --- ---]
                                         //      lHghDrv = 2,   sized: [27]
                                         //        i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---
                                         //                             Du  Duw --- Duv --- --- --- --- ---
                                         //                             Duu --- --- --- --- --- --- --- ---]
                                         //      lHghDrv = 3,   sized: [81]
                                         //        i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw
                                         //                             --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---
                                         //                             Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---
                                         //                             --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---
                                         //                             --- ---  ---   ---   ---  ---  ---  ---  Duuu ---
                                         //                             --- ---  ---   ---   ---  ---  ---  ---  ---- ---
                                         //                             --- ---  ---   ---   ---  ---  ---  ---  ---- ---
                                         //                             --- ---  ---   ---   ---  ---  ---  ---  ---- ---
                                         //                             --- --- ]
    SmBoolean bNonZeroTangents=TRUE,     // NotUsed: in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                         //      FALSE= return exact tangent values, default:[TRUE]
                                         //      note: Surprisingly TRUE is the common choice because most tangent uses
                                         //          are for their direction (Binorm, SurfNorm comps), but when the
                                         //          tangent is being used for its magnitude (like an arc-length comp)
                                         //          then set this to FALSE.
    SmBoolean bDoZeroSampling=TRUE)      // NotUsed: in : for internal use only, always set to TRUE, default:[TRUE]
   const ;

  // compute ProjSpace BBoxes for given ParamSpace Box
  virtual SmStatus EvaluateBoundingBoxSimple              (const SmExtent3d  & crParamDomain,              // in : 
                                                           SmPseudoBox       * pOptPseudoParamBox = NULL,  // NotUsed: in : 
                                                           SmExtent3d        * pNormalProjBox     = NULL,  // out: 
                                                           SmPseudoBox       * pPseudoProjBox     = NULL)  // out: 
                                                           const ;

  // Find portion of InSpace Line interval that maps to the NaturalParamDomain
  // rtn: SM_ERR for NULL line, else SM_SUCCESS
  virtual SmStatus FindParamIntervalForInSpaceLineSimple  (const SmPoint3d      & crInSpacePoint,   ///< [in] : LinePoint       of Line(s) = LinePoint + s * LineVector
                                                           const SmVector3d     & crInSpaceVector,  ///< [in] : unit LineVector of Line(s) = LinePoint + s * LineVector
                                                           const SmExtent1d     & crCurrentIvl,     ///< [in] : current limits on s interval
                                                           SmTArray<SmExtent1d> & rTrimIvls)         ///< [out]: Interval of line that maps legally within the NaturalParamDomains
                                                          const ;

  // GWC: exclude FindParamExtentForInSpacePlaneSimple from first release. Code is written but I don't
  //      see the bug in SmBendVolume::FindParamExtentForInSpacePlaneSimple().  I'll fix that later.
  //      // Find the largest square extent of an InSpace plane that maps inside the NaturalParamDomain.
  //      virtual SmStatus FindParamExtentForInSpacePlaneSimple                 // rtn: SM_ERR for NULL plane, else SM_SUCCESS
  //                                  (const SmPoint3d      & crInSpacePoint,   ///< [in] : LinePoint          of Line(s) = LinePoint + u * LineVecU + v * LineVecV
  //                                   const SmVector3d     & crInSpaceVecU,    ///< [in] : scaled LineVectorU of Line(s) = LinePoint + u * LineVecU + v * LineVecV
  //                                   const SmVector3d     & crInSpaceVecV,    ///< [in] : scaled LineVectorV of Line(s) = LinePoint + u * LineVecU + v * LineVecV
  //                                   const SmExtent2d     & crCurrentUV,      ///< [in] : current limits on UV extent
  //                                   SmTArray<SmExtent2d> & rTrimUVs)         // out: UVExtent of plane that maps legally within the NaturalParamDomains
  //                                  const ;

  // Trim ParamSpace BBox to Natural Parameter Domain
  // rtn: SM_ERR when ParamBox trims to empty set, else SM_SUCCESS
  virtual SmStatus TrimParamBoundingBoxSimple (const SmExtent3d & crParamBox,       // in : Tgt Param Box to trim
                                               SmExtent3d       & rParamTrimBox)    // in : Box trimmed to Natural Param Domain
                                              const ;

//     // GWC - removed from interface because it made no sense: Even very simple cases of trimming a BBox to a long skinny PseduoBox showed
//     //       that the inscribed Box could be very small and non-unique.  It won't necessarily capture the notion of a good
//     //       bounding box designed to help narrow down the space of options for functions that need to check that.  In general,
//     //       check wether individual InSpace and ProjPace Points map back to legal ParamDomain Points, one point at a time.
//     // Trim an InSpace BBox so that it maps totally within the Natural ParamSpace Domain
       // virtual SmStatus TrimInSpaceBoundingBoxSimple                   // rtn: SM_ERR when InSpaceBox trims to empty set, else SM_SUCCESS
       //                (const SmExtent3d & crInSpaceBox,                ///< [in] : Tgt InSpace Box to trim
       //                 SmExtent3d       & rInSpaceTrimBox)             ///< [in] : Box trimmed so that it maps totally within the Natural ParamSpace Domain
       //                const ;

  // Create ProjSpace IsoCurve from a ParamSpace IsoParamLine - requires derived implementation
  virtual SmStatus EvaluateIsoParametricCurveSimple (const SmContext     & crContext,            // in : context for created objects
                                                     SmVolumeParamsType    eConstantParams,      // in : oneof: SM_VPS_UV_IN,
                                                                                                 //           SM_VPS_UW_IN,
                                                                                                 //           SM_VPS_VW_IN.
                                                     double                dIsoParam1,           // in : 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value
                                                     double                dIsoParam2,           // in : 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value
                                                     double                d3DTolerance,         // NotUsed: in : Max ApproxCurve to IdealCurve deviation
                                                     SmCurve            *& rpNewIsoCurve,        // out: the ProjSpace IsoCurve
                                                     const SmExtent3d    * pOptParamDomain=NULL) // in : limiting domain, NULL to ignore.
                                                    const ;

  // Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane - requires derived implementation
  virtual SmStatus EvaluateIsoParametricSurfaceSimple (const SmContext   & crContext,              // in : context for created objects
                                                       SmVolumeParamType   eConstantParam,         // in : oneof: SM_VP_U_IN,
                                                                                                   //           SM_VP_V_IN,
                                                                                                   //           SM_VP_W_IN
                                                       double              dIsoParam,              // in : constant param value
                                                       double              d3DTolerance,           // NotUsed: in : Max ApproxSurface to IdealSurface deviation
                                                       SmSurface        *& rpNewIsoSurface,        // out: the ProjSpace IsoSurface
                                                       const SmExtent3d  * pOptParamDomain=NULL)   // in : limiting domain, NULL to ignore.
                                                      const ;

  // get approximate ParamSpace location for given ProjSpace point for upcoming newton raphson search
  virtual SmStatus InvEvaluateGuessPointSimple( const SmPoint3d     & crProjPoint,        // in : ProjSpace Point to map back to ParamSpace Point
                                                SmTArray<SmPoint3d> & rGuessParamPoints)  // out: ParamSpace Point near Target Point actual map back to ParamSpace
                                               const ;

  // General Point Solver without a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  virtual SmStatus GlobalPointSolveSimple (const SmPoint3d  & crProjPoint,          // in : ProjSpace Point to map back to ParamSpace
                                           SmBoolean        & rbFoundAnswer,        // out: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point
                                           SmSolutionArray  & rSolutions,           // out: Contains ParamSpace Found Point
                                                                                    //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE
                                                                                    //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);
                                                                                    //      rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location
                                                                                    //      rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location
                                                                                    //      rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location
                                           const SmExtent3d * pOptParamDomain=NULL) // NotUsed: in : ParamSpace domain over which to search for the inverse point
                                                                                    //      NULL = use Map's NaturalDomain
                                          const ;                                  

  // Local Point Solver with a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  virtual SmStatus  LocalPointSolveSimple (const SmPoint3d  & crProjPoint,          // in : ProjSpace Point to map back to ParamSpace
                                           const SmPoint3d  & crParamPointGuess,    // in : ParamSpace point guess, the closer to the actual ParamSpace point the better
                                           SmBoolean        & rbFoundAnswer,        // out: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point
                                           SmSolution       & rSolution,            // out: Contains ParamSpace Found Point
                                                                                    //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE
                                                                                    //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);
                                                                                    //      rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location
                                                                                    //      rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location
                                                                                    //      rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location
                                           const SmExtent3d * pOptParamDomain=NULL) // in : ParamSpace domain over which to search for the inverse point
                                                                                    //      NULL = use Map's NaturalDomain
                                          const ;                                  

  // IsBounded = TRUE because NaturalParamDomain is limited to U>0 halfspace
  virtual SmBoolean IsBoundedSimple      () const ;

  // IsClosed = !ClosedU, ClosedV, !ClosedW. BendVolumes are periodic over v = [-d*Pi, +d*Pi ] - a wedge, not a box as is typical
  virtual SmBoolean IsClosedSimple (SmBoolean        & rbClosedU,               // out: TRUE = closed in U direction, [check Pos[Umin,v,w] == Pos[Umax,v,w] for v,w samples
                                    SmBoolean        & rbClosedV,               // out: TRUE = closed in V direction, [check Pos[u,Vmin,w] == Pos[u,Vmax,w] for w,u samples
                                    SmBoolean        & rbClosedW,               // out: TRUE = closed in W direction, [check Pos[u,v,Wmin] == Pos[u,v,Wmax] for u,v samples
                                    double           * pdOptTolerance = NULL,   // out: pdOptTolerance = Tolerance to allow for check
                                    const SmExtent3d * pOptParamDomain = NULL,  // in : ParamSpace domain over which to search for the inverse point
                                                                                //      NULL = use Map's NaturalDomain, default:[NULL]
                                    SmContinuityType * peOptContinuityU = NULL, // out: U dir Continuity when closed 
                                    SmContinuityType * peOptContinuityV = NULL, // out: V dir Continuity when closed      
                                    SmContinuityType * peOptContinuityW = NULL) // out: W dir Continuity when closed
                                                                                //      oneof SM_CT_DISCONTINUOUS      
                                                                                //          SM_CT_C0                 
                                                                                //          SM_CT_G1                 
                                                                                //          SM_CT_G1_G2              
                                                                                //          SM_CT_G1_G2_G3              
                                                                                //          SM_CT_C1                 
                                                                                //          SM_CT_C1_G2        
                                                                                //          SM_CT_C1_G2_G3        
                                                                                //          SM_CT_C1_C2        
                                                                                //          SM_CT_C1_G3        
                                                                                //          SM_CT_C1_C3        
                                   const ;

  // IsPeriodic = !PeriodicU, PeriodicV, !PeriodicW. BendVolumes are periodic over v = [-d*Pi, +d*Pi ] - a wedge, not a box as is typical
  virtual SmBoolean IsPeriodicSimple    (SmBoolean        & rbPeriodicU,             // out: TRUE = G1 or better in U Dir 
                                         SmBoolean        & rbPeriodicV,             // out: TRUE = G1 or better in V Dir
                                         SmBoolean        & rbPeriodicW,             // out: TRUE = G1 or better in W Dir
                                         const SmExtent3d * pOptParamDomain = NULL)  // NotUsed: in : ParamSpace domain over which to search for the inverse point
                                        const ;                                      //      NULL = use Map's NaturalDomain, default:[NULL]                                          
                                                                                      
  virtual SmBoolean IsSingularitySimple (const SmPoint3d   & crParamPoint,           // in]: UVpoint to test
                                         SmBoolean         & rbSingularU,            // out: TRUE = [Su=0]
                                         SmBoolean         & rbSingularV,            // out: TRUE = [Sv=0]
                                         SmBoolean         & rbSingularW,            // out: TRUE = [Sw=0]
                                         double              d3dTol = SM_EFF_ZERO)   // NotUsed: in : min 3d distance between distinct points) const;
                                        const;                                       
                                                                                     
  virtual SmBoolean IsOnBoundarySimple  (const SmPoint3d  & crParamPoint,            // in : Volume UVWPoint to test
                                         SmBoolean        & rbOnU,                   // out: TRUE = TargetPoint on UMin or UMax
                                         SmBoolean        & rbOnV,                   // out: TRUE = TargetPoint on VMin or VMax
                                         SmBoolean        & rbOnW,                   // out: TRUE = TargetPoint on WMin or WMax
                                         double           * pdOptTolerance = NULL,   // NotUsed: in : max deviation allowed for point on seam
                                                                                     //      NULL = use SM_EFF_ZERO * 1000 * (1 + maxDimension())
                                         const SmExtent3d * pOptParamDomain = NULL)  // NotUsed: in : ParamSpace domain over which to search for the inverse point
                                        const ;                                      //      NULL = use Map's NaturalDomain, default:[NULL]

  // rtn: TRUE if ParamPoint is within the maps paramSpace Natural domain
  virtual SmBoolean IsPointInParamDomainSimple(const SmPoint3d  & crParamPoint) const ;

  // rtn: TRUE if projPoint can be inverted uniquely to DomainSpace
  virtual SmBoolean IsPointInProjDomainSimple (const SmPoint3d  & crProjPoint,       // in : project space point to test for inversion                
                                               SmPoint3d   * pOptParamPoint = NULL)  // out: if the point is invertible, go ahead and get the inverse
                                              const ;

  // rtn: TRUE if ParamLine is within the maps paramSpace Natural domain
  virtual SmBoolean IsLineInParamDomainSimple (const SmPoint3d  & crStartParamPoint,  // in : ParamSpace Line StartPoint to test for inclusion
                                               const SmPoint3d  & crEndParamPoint)    // in : ParamSpace Line EndPoint to test for inclusion
                                              const ;

  // ToBe implemented: base implementations on projecting line to ParamSpace and testing IsCurveInParamDomainSimple
  //  virtual SmBoolean IsLineInProjDomainSimple(const SmPoint3d  & crStartProjPoint,    // rtn: TRUE if projPoint can be inverted uniquely to ParamSpace
  //                                             const SmPoint3d  & crEndProjPoint,      // in : LineSeg End projPoint
  //                                             SmPoint3d        * pOptStartParamPoint, // out: if the point is invertible, go ahead and get the inverse
  //                                             SmPoint3d        * pOptEndParamPoint)   // out: if the point is invertible, go ahead and get the inverse
  //                                            const

  virtual SmBoolean IsCurveInParamDomainSimple  (const SmCurve & crParamCurve) const ;

  virtual SmBoolean IsSurfaceInParamDomainSimple(const SmSurface & crParamSurface) const ;

public:
  // utilities

  // I/O assist
  virtual SmStatus WriteToDB (SmDatabaseIO & rDB,                    // in : target output stream
                              ULONG lDBVersionNumber)                // in : database version to get proper sequence of writes
                             const ;                                      
                                                                          
  static  SmStatus ReadFromDB (SM_TYPE           lType,              // NotUsed: in : Object type to be read
                               SmDatabaseIO    & rDB,                // in : target output stream
                               const SmContext & crContext,          // in : context for new object construction
                               SmVolume       *& rpNewVolume,        // out: NULL on input = new object allocated in this routine built from stream data
                                                                     //      NotNULL on input = pointer to an empty object to be filled by this routine
                               ULONG             lDBVersionNumber) ; // in : database version to get proper sequence of writes
                                                                         
  // get memory used for volume and its attributes                       
  virtual ULONG GetMemoryUsed (ULONG    & rlMemoryAllocated,         // out: bigger size of all allocated memory in bytes
                               SmMarkType eMarkType=SM_MT_NOMARK)    // in : uses without increment eMarkType value
                              const ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmBendVolume, SmVolume, SmBendVolume_TYPE) ;

  // pretty print SmVolume
  virtual void Dump(SmBoolean bAbbrev, ULONG lIndentCnt) const ;

  // AssertValid
  virtual SmDisplayList * Draw (SmBoolean       bShowOutSpace=TRUE,    // in :
                                SmBoolean       bShowInSpace=FALSE,    // in :
                                SmExtent3d    * pOptParamDomain=NULL,  // in : optionally output a subDomain graphic, NULL to ignore
                                SmGfxArraySet * pOptGfxSet=NULL)
                               const;

  virtual SmBoolean AssertValid (SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                 SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                                                           //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                 SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
                                 SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                                const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

} ; // end class SmBendVolume


// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmBendVolume*)

#endif // !__SMBENDVOLUME_H__


