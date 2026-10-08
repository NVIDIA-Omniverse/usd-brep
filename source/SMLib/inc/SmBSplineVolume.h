// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBSplineVolume.h
* PURPOSE: Header file for B-Spline Volume class.
* Oct 2006 GWC Author
**********************************************************************/

#ifndef __SMBSPLINEVOLUME_H__
#define __SMBSPLINEVOLUME_H__

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif

#ifndef __SMVOLUME_H__
#include <SmVolume.h>
#endif

#include <SmGraphicsExtern.h>

// forward declarations
//      typedef struct gw_volume gw_VOLUME ;     // NLib NURB volume
//      typedef struct volume VOLUME ;           // NLib NURB volume
//      typedef struct gw_cpoint gw_CPOINT;      // NLib NURB control polygon array

/*******************************************************************//**
PURPOSE: This object is a volume which represents a NURBS volume.

NOTES: The SmBSplineVolume is a mapping from a UVW ParamSpace to a XYZ ProjSpace as
       ProjPoint = EvaluateSimple(ParamPoint) ;

       The ParamSpace shape of the BSplineVolume's bounding box is an axis aligned SmExtent3d box.
       The InSpace shape of the BSplineVolume's bounding box is PseudoBox.
       The InSpace PseudoBox is an exact Orientation Mapping of the Bounding BoundingBox.
       so,
          Orient(ParamSpace BoundingBox) = InSpace PseudoBox and
          InvOrient(InSpace PseudoBox)   = ParamSpace BoundingBox

       This relationship is unique to BSplineVolumes and is used to manage
       the UserInterface to BSplineVolume Space Deformations.  The 
       Orient Mapping allows the user to position the BSplineVolume over
       a desired chunk of Inspace and then the BSplineVolume Map
       can deform all the geometry in that chunk.
       
***********************************************************************/
class SM_EXPORT SmBSplineVolume : public SmVolume
{
protected:
  gw_VOLUME  * m_pNurb;                         // ptr to Bspline data of type gw_VOLUME
  SmBoolean    m_bNurbIsBorrowed;               // borrowed Nurb pointers are on the stack and
                                                // can't do global operations
  SmBoolean    m_bOutOfBoundsEnabled ;          // GWC:OUTOFBOUNDS MODIFICATION
                                                // TRUE=RETURN EVALUATIONS FOR UV POINTS BEYOND DOMAIN
                                                // default:[FALSE]
  // Inherited SmVolume members
  //      SmObject    * m_pOwner ;            // not used
  //       SmVolume    *m_pNextMap ;          // linked list of concatenated SmVolume transformations
  //       SmTransform *m_pOrientMap ;        // for space deformations to orient this mapping's ParamSpace to a target InSpace.
  //       SmTransform *m_pInvOrientMap ;     // cached m_pOrientMap Inverse.
  //       ULONG        m_lNextOwnerFlag ;    // owner state for m_pNextMap, needed for destructor
  //       ULONG        m_lOrientOwnerFlag ;  // owner state for m_pOrientMap, needed for destructor

  // Inherited SmVolume virtual methods: 
  //     A key Volume design feature is the isolation of derived classes from the complications 
  //     of both compounding and orientation.  A derived Volume class only implements the 'EvaluateSimple' 
  //     set of virtual methods and inherits compounding and orientation from the base SmVolume class.
  //     
  //     Virtual Methods to be implemented for derived Classes
  //
  //      // standard structure
  //        constructors   [as needed]           // req: make sure to call base constructors
  //        Copy()                               // req: deep virtual copy operator
  //        operator=()    [not virtual]         // req: deep copy operator
  //        operator==()                         // req: deep compare predicate 
  //        Notify()                             // opt: Notify mechanism. req:[if Derived class caches data]. Default:[pass Notify to compounded volumes and base class]
  //        Destructor()                         // req: derived class destructors must be virtual
  //        GetNaturalParamDomain()              // req: return SmExtent3d ParamSpace domain
  //                                             
  //     // look like a BSplineVolume - obsolete
  //     // gwc change: Volumes are no longer required to look like BSplines - these only need to be in SmBSplineVolume
  //        GetGwNurbPointer()                   // opt: retrieve the NLIB NURB struct           
  //        Reparameterize()                     // opt: reparameterize the param space domain intervals
  //        GetDegree()                          // opt: return paramDirection degree.      Default:[signal error and return 3]
  //        GetKnots()                           // opt: return paramDirection knot vector. Default:[init output and return SM_ERR]
  //        GetNumberControlPoints()             // opt: return paramDirection CPoint cnt.  Default:[signal error and return 0]  
  //                                             
  //     // EvaluateSimple Interface to fit within the Oriented and compounded structure of the SmVolume base class
  //        EvaluateBoundingBoxSimple()             // opt: map ParamBoundingBoxes to ProjBoundingBoxes
  //        EvaluateIsoParametricCurveSimple()      // req: create ProjSpace IsoCurve for ParamSpace IsoLine
  //        EvaluateIsoParametricSurfaceSimple()    // req: create ProjSpace IsoSurface for ParamSpace IsoPlane
  //        EvaluateSimple()                        // req: compute ProjPoint = EvaluateSimple(ParamPoint)
  //        InvEvaluateGuessPointSimple()           // req: get approximate ParamSpace location for given ProjSpace point
  //        GlobalPointSolveSimple()                // opt: Point Solver without a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  //        LocalPointSolveSimple()                 // opt: Point Solver with a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  //        FindParamIntervalForInSpaceLineSimple() // opt: Find InSpace Line interval that maps to the NaturalParamDomain, default rTrimIvl = crCurrentIvl
  //        TrimParamBoundingBoxSimple()            // opt: Trim ParamBBox to natural ParamDomain, default pTrimBox = pGivenBox
// GWC - excluded from first release:  //        FindParamExtentForInSpacePlaneSimple()  // opt: Find InSpace Plane extent that's within the NaturalParamDomain, default rTrimIvl = crCurrentIvl
//     // GWC - removed from interface because it made no sense: Even very simple cases of trimming a BBox to a long skinny PseduoBox showed
//     //       that the inscribed Box could be very small and non-unique.  It won't necessarily capture the notion of a good
//     //       bounding box designed to help narrow down the space of options for functions that need to check that.  In general,
//     //       check wether individual InSpace and ProjPace Points map back to legal ParamDomain Points, one point at a time. 
//     //   TrimInSpaceBoundingBoxSimple()          // opt: Trim InSpaceBBox to map within the NaturalParamSpaceDomain, default pTrimBox = pGivenBox
  //                                            
  //     // Derived class EvaluateSimple map modifications
  //        // design issue - derived and default behaviors are not exactly the same.  
  //        //                Default behavior modifies the volume maps after being oriented, i.e. concatenates another NextMap object.
  //        //                Derived behavior modifies the volume map before being oriented, i.e. modifies EvaluateSimple() by modifying derived map parameters.
  //        //                If at all possible, derived classes should implement this set of virtual methods.
  //        MirrorSimple()                       // opt: Mirror EvaluateSimple map.                 implement:[if map can mirror],           Default:[by concatenating a Mirror        Transform as NextMap]
  //        ScaleSimple()                        // opt: Scale EvaluateSimple map.                  implement:[if map can scale],            Default:[by concatenating a Scale         Transform as NextMap]
  //        TranslateSimple()                    // opt: Translate EvaluateSimple map.              implement:[if map can translate],        Default:[by concatenating a Translate     Transform as NextMap]
  //        RotateAboutAxisSimple()              // opt: RotateAboutAxis EvaluateSimple map.        implement:[if map can RotAboutAxis],     Default:[by concatenating a Rotate        Transform as NextMap]
  //        RotateAboutAxisAtPointSimple()       // opt: RotateAboutAxisAtPoint EvaluateSimple map. implement:[if map can RotAboutAxisAtPt], Default:[by concatenating a RotateAtPoint Transform as NextMap]  
  //        TransformSimple()                    // opt: Transform EvaluateSimple map.              implement:[if map can Transform],        Default:[by concatenating a general       Transform as NextMap]
  //                                             
  //     // predicates                           
  //        IsBoundedSimple()                    // opt: Bounded  ParamSpace Predicate. Default:[return FALSE]
  //        IsClosedSimple()                     // opt: Closed   ParamSpace Predicate. Default:[OutSpace Point Sample test]
  //        IsPeriodicSimple()                   // opt: Periodic ParamSpace Predicate. Default:[OutSpace Point Sample test]
  //        IsSingularitySimple()                // opt: EvaluateSingularity Predicate. Default:[OutSpace Zero Tangent check]
  //        IsOnBoundarySimple()                 // opt: ParamPt on ParamBnd Predicate. Default:[ParamSpace dist to ParamBoundindBox check]
  //        IsPointInParamDomainSimple()         // opt: Return TRUE if ParamPoint is within the map's paramSpace Natural domain, default:[return TRUE]                                   
  //        IsPointInProjDomainSimple()          // opt: Return TRUE if projPoint can be inverted uniquely to ParamSpace, default:[return TRUE]                                   
  //        IsLineInParamDomainSimple()          // opt: rtn TRUE if ParamLine is in map's NaturalParamDomain. default:[if bnded(GetNaturalDomain().Contains(ParamPt)) else TRUE], implement if:[ParamDomain is bounded not rectilinear]                                   
  //  ToBe IsLineInProjDomainSimple()            // opt: rtn TRUE if ProjLine drops within map's NaturalParamDomain. default:[if bnded(InvEvaluatePointSimple()) else TRUE], implement if:[ParamDomain is bounded not rectilinear]
  //               
  //        IsCurveInParamDomainSimple()         // opt: return TRUE when TgtCurve does NOT cross any Volume external or internal discontinuity boundary
  //        IsSurfaceInParamDomainSimple()       // opt: return TRUE when TgtSurface does NOT cross any Volume external or internal discontinuity boundary
  //        HasDiscontinuitiesSimple()           // opt: Return TRUE if Simple map has internal C1 discontinuities, default:[return FALSE]
  //        CalculateContinuitiesSimple()        // opt: Return list of all Simple map internal discontinuities, default:[none]
  //
  //     // Persistence and reporting            
  //        WriteToDB()                          // req: virtual nested write i/o
  //        static ReadFromDB()                  // req: static nested read i/o
  //        GetMemoryUsed()                      // opt: required when derived class has nay members. Default:[this + NextMap + OrientMap + InvOrientMap]
  //        Dump()                               // req: nested pretty print
  //        Draw()                               // opt: 
  //        DrawMesh()                           // opt: 
  //        DrawControlPoints()                  // opt: 
  //        DrawUVW()                            // opt: 
  //        AssertValid()                        // req: 

public: 
                                        
  // Empty Constructor for I/O
  SmBSplineVolume(const SmContext * cpContext=NULL); // Construct Volume with no NURB object 

  // constructor
  SmBSplineVolume
  (
    const SmContext * cpContext, ///< [in] : context for this new object                                        <br>
    ULONG             m,         ///< [in] : high index of Pw (U)                                               <br>
    ULONG             n,         ///< [in] : high index of Pw (V)                                               <br>
    ULONG             o,         ///< [in] : high index of Pw (W)                                               <br>
    short             p,         ///< [in] : degree in u                                                        <br>
    short             q,         ///< [in] : degree in v                                                        <br>
    short             r,         ///< [in] : degree in w                                                        <br>
    ULONG             ir,        ///< [in] : high index of U                                                    <br>
    ULONG             is,        ///< [in] : high index of V                                                    <br>
    ULONG             it,        ///< [in] : high index of W                                                    <br>
    gw_CPOINT     *** Pw,        ///< [in] : Control mesh to copy  // data in NLib format, NOT checked here     <br>
    double            *U,        ///< [in] : knots in u to copy    // data in NLib format, NOT checked here     <br>
    double            *V,        ///< [in] : knots in v to copy    // data in NLib format, NOT checked here     <br>
    double            *W         ///< [in] : knots in w to copy    // data in NLib format, NOT checked here     <br>
  );
  
  // Fast constructor - stores given gw_VOLUME       
  SmBSplineVolume
  (
    gw_VOLUME       * pAlreadyAllocatedNurb,   ///< [in] : stores pAlreadyAllocatedNurb NURB object                    <br>
    SmBoolean         bNurbIsBorrowed,         ///< [in] : TRUE = Don't delete pAlreadyAllocatedNurb when destructed   <br>
    const SmContext * cpContext=NULL           ///< [in] : set this->m_cpContext when given                            <br>
  );

  // Std constructor - copies given gw_VOLUME
  SmBSplineVolume
  (
    const SmContext * cpContext,               ///< [in] : context for this new object, may be NULL for temp objects     <br>
    const gw_VOLUME * cpGwNurb                 ///< [in] :copies gw_VOLUME NURB object                                   <br>
  );

  // copy constructors
  // SmBSplineVolume(const SmBSplineVolume & crSourceVolume)   ///< [in] : SourceVolume to copy (copies all compounding volumes)
  //                                                           { SmBSplineVolume::SmBSplineVolume(crSourceVolume, FALSE) ; }

  SmBSplineVolume
  (
    const SmBSplineVolume & crSourceVolume,      ///< [in] : SourceVolume to copy                                         <br>
    SmBoolean               bSimpleMapOnly=FALSE ///< [in] : TRUE = Copy this Volume omitting any compounding volumes     <br>
                                                 ///<        FALSE= Copy this Volumes with any compounding volumes        <br>
  );
                                                 
  // make an exact copy of any volume with or without any compounding volumes
  virtual SmStatus Copy
  (
    const SmContext & crContext,           ///< [in] : context for new object construction                          <br>
    SmVolume       *& rpNewVolume,         ///< [out]: The copied Volume                                            <br>
    SmBoolean         bSimpleMapOnly=FALSE ///< [in] : TRUE = Copy this Volume omitting any compounding volumes     <br>
                                           ///<        FALSE= Copy this Volumes with any compounding volumes        <br>
  ) const ;                                  
                                                               
  
  static SmStatus CreateCanonical
  (
    const SmContext & crContext,                         ///< [in] : context for new object construction                              <br>
    ULONG lUDegree,                                      ///< [in] : U dir Degree                                                     <br>
    ULONG lVDegree,                                      ///< [in] : V dir Degree                                                     <br>
    ULONG lWDegree,                                      ///< [in] : W dir Degree                                                     <br>
    const SmTArray<SmPoint3d> & crControlPointsList,     ///< [in] : coords : Euclidian                                               <br>
                                                         ///<        sized  : [  lNumUCPts = UKnotTotalCnt - lUDegree - 1             <br>
                                                         ///<                  * lNumVCPts = VKnotTotalCnt - lVDegree - 1             <br>
                                                         ///<                  * lNumWCPts = WKnotTotalCnt - lWDegree - 1]            <br>
                                                         ///<        ordered: for Puvw - w varies fastest, then v, then u as:         <br>
                                                         ///<                [P000, P001, ... P00o, with: n = lNumUCPts - 1           <br>
                                                         ///<                 P010, P011, ... P01o,       m = lNumVCPts - 1           <br>
                                                         ///<                 . . .                       o = lNumWCPts - 1           <br>
                                                         ///<                 P0m0, P0m1, ... P0mo,                                   <br>
                                                         ///<                 P100, P101, ... P10o,                                   <br>
                                                         ///<                 . . .                                                   <br>
                                                         ///<                 Pnm0, Pnm1, ... Pnmo]                                   <br>
    const SmTArray<ULONG> & crUMultiplicities,           ///< [in] : each crUKnot[i] multiplicity, sized:[crUKnots.GetSize()]         <br>
    const SmTArray<ULONG> & crVMultiplicities,           ///< [in] : each crVKnot[j] multiplicity, sized:[crVKnots.GetSize()]         <br>
    const SmTArray<ULONG> & crWMultiplicities,           ///< [in] : each crWKnot[k] multiplicity, sized:[crWKnots.GetSize()]         <br>
    const SmTArray<double> & crUKnots,                   ///< [in] : unique or complete U Knot vals, as coordinated with Mults arrays <br>
    const SmTArray<double> & crVKnots,                   ///< [in] : unique or complete V Knot vals, as coordinated with Mults arrays <br>
    const SmTArray<double> & crWKnots,                   ///< [in] : unique or complete W Knot vals, as coordinated with Mults arrays <br>
    const SmTArray<double> * cpOptWeights,               ///< [in] : optional weight for each ControlPoint, NULL to ignore            <br>
    const SmExtent3d       * cpOptUVWDomain,             ///< [in] : Will trim volume to smaller domain                               <br>
    SmBSplineVolume       *& rpNewBSplineVolume          ///< [out]: New SmBSplineVolume, NULL on input                               <br>
  );

  // equality operator
  virtual SmBoolean operator==(const SmVolume&) const;

  // assignment operator
  SmBSplineVolume &operator=(const SmBSplineVolume &crBSplineVolume) ;

  // Notify - use inherited Base behavior

  // destructor
  virtual ~SmBSplineVolume() ;
  
  // Edit BSpline Representation
  SmStatus Reparameterize(const SmExtent3d & crNewDomain);

  SmStatus DegreeElevate(SmVolumeParamType eDirectionToElevate, 
                         ULONG             lNewDegree);

  SmStatus InsertOneKnot
  (
    double dKnot,                    ///< [in] : parameter value to insert               <br>
    ULONG lNumKnotInsertions,        ///< [in] : Number of times to insert the knot      <br>
    SmVolumeParamType eVolumeParam   ///< [in] : oneof: SM_VP_U, SM_VP_V, SM_VP_W        <br>
  ) ;

  // gwc: a function to consider adding later on
  //      SmStatus RefineVolume(const SmTArray<double> & crNewKnots,
  //                             SmVolumeParamType eVolumeParam) ;

  // gwc: a function to consider adding later on
  //      SmStatus RemoveKnots(double dThisApproxTol3d,
  //                           SmBoolean bRemoveUKnots,
  //                           SmBoolean bRemoveVKnots,
  //                           SmBoolean bRemoveWKnots,
  //                           const SmTArray<double> * cpOptKeptUKnots = NULL,
  //                           const SmTArray<double> * cpOptKeptVKnots = NULL,
  //                           const SmTArray<double> * cpOptKeptWKnots = NULL) ;

  // gwc: a function to consider adding later on
  //      SmStatus RemoveOneKnot(double dKnot,
  //                             ULONG lNumKnotsRemoval,
  //                             SmVolumeParamType eVolumeParam,
  //                             double dThisApproxTol3d,
  //                             ULONG & rlNumKnotsRemoved) ;

  // gwc: a function to consider adding later on
  //      SmStatus ReparametrizeWithArcLength() ;

  // gwc: a function to consider adding later on
  //      SmStatus SplitAt(const SmContext    & crContext,       // in : context for new object construction           
  //                       double               dParam,          // in : split parameter                               
  //                       SmVolumeParamType    eVolumeParam,    // in : oneof SM_SP_U = split u domain at dParam      
  //                                                             //    :       SM_SP_V = split v domain at dParam      
  //                       SmVolume          *& rpLeftVolume,    // out: Split surface result, Ivl=[MinParam, TgtParam]
  //                       SmVolume          *& rpRightVolume) ; // out: Split surface result, Ivl=[TgtParam, MaxParam]

  // gwc: a function to consider adding later on
  //      virtual SmStatus TrimWithDomain(SmExtent3d & rTrimDomain) ;

  // data access
  ULONG                GetDegree              (SmVolumeParamType eVolumeParam) const ;
  ULONG                GetNumberNaturalKnots  (SmVolumeParamType eVolumeParam) const ;
  virtual SmExtent3d   GetNaturalParamDomain  ( )                              const ;
  ULONG                GetNumberControlPoints (SmVolumeParamType eVolumeParam) const ;
  SmBoolean &          GetOutOfBoundsEnabled  ( )                                    { return m_bOutOfBoundsEnabled; }
  gw_VOLUME *          GetGwNurbPointer       ( )                              const { return m_pNurb ; }  // retrieve the NLIB NURB struct
  gw_VOLUME *          GetOrCreateGwNurbPointer( )                                  { return m_pNurb ; }

  SmStatus GetKnots
  (
    SmVolumeParamType    eVolumeParam,                  ///< [in] : oneof: SM_VP_U, SM_VP_V, SM_VP_W                               <br>
    SmTArray<double>   & rKnots,                        ///< [out]: unique knot values in requested dimension                      <br>
    SmTArray<ULONG>    * pKnotMultiplicities = NULL,    ///< [out]: associated multiplicity for every knot                         <br>
    const SmExtent1d   * pOptIvl = NULL                 ///< [in] : interval of interest, NULL=Natural Interval, default:[NULL]    <br>
  ) const;

  SmStatus GetControlPointMesh    
  (
    ULONG               & rlUCount,              ///< [out]: U controlPoint count                                                 <br>
    ULONG               & rlVCount,              ///< [out]: V controlPoint count                                                 <br>
    ULONG               & rlWCount,              ///< [out]: W controlPoint count                                                 <br>
    SmTArray<SmPoint3d> & rControlPointsList,    ///< [out]: Always produced in Euclidian coordinates                             <br>
    SmTArray<double>    & rWeights               ///< [out]: The Weights array will have size zero if the volume is non-rational. <br>
  ) const ;
                                                               
  SmStatus GetControlPoint        
  (
    SmControlPointFormType  eCtrlPointForm,      ///< [in] : SM_CP_NON_RATIONAL         - do perspective projection and      <br>
                                                 ///<                                     set W=1.0 if it is rational.       <br>
                                                 ///<        SM_CP_HOMOGENEOUS_RATIONAL - don't do division and return W     <br>
                                                 ///<        SM_CP_EUCLIDIAN_RATIONAL   - do division and return W           <br>
    ULONG                   lUIndex,             ///< [in] : target ControlPoint 1st index                                   <br>
    ULONG                   lVIndex,             ///< [in] : target ControlPoint 2nd index                                   <br>
    ULONG                   lWIndex,             ///< [in] : target ControlPoint 3rd index                                   <br>
    SmPoint3d             & rControlPoint,       ///< [out]: ControlPoint position - in requested format                     <br>
    double                & rdWeight             ///< [out]: assocaited weight - in requested format                         <br>
  ) const ;

  SmStatus GetControlPointsPointer
  (
    ULONG   & lControlPointCountU,                ///< [out]: number of control points in each U row                    <br>
    ULONG   & lControlPointCountV,                ///< [out]: number of control points in each V row                    <br>
    ULONG   & lControlPointCountW,                ///< [out]: number of control points in each W row                    <br>
    double *& pControlPoints                      ///< [out]: array of controlPoints stroed as doubles                  <br>
                                                  ///<        with Pijk = [x y z w]                                     <br>
                                                  ///<        ordered:[P000, P001, .... P00W,                           <br>
                                                  ///<                 P010, P011, .... P01W,                           <br>
                                                  ///<                 . . .                                            <br>
                                                  ///<                 P0V0, P0V1, .... P0VW,                           <br>
                                                  ///<                 P10W, P101, .... P10W,                           <br>
                                                  ///<                 . . .                                            <br>
                                                  ///<                 PUV0, PUV1, .... PUVW]                           <br>
                                                  ///<        For 2d control Point   z == NL_NOZ or 0.0                 <br>
                                                  ///<        for nonRational points w == NL_NOW                        <br>
                                                  ///<        for Rational points x,y,z are stored in homogeneous space <br>
                                                  ///<          i.e. CartesianX = x/w                                   <br>
                                                  ///<               CartesianY = y/w                                   <br>
                                                  ///<               CartesianZ = (z != NL_NOZ) ? z/w : NL_NOZ ;        <br>
  ) const ;


  // gwc: a function to consider adding later on
  //      SmStatus             GetMeasures            (double & rdAverageLengthU,
  //                                                   double & rdAverageLengthV,
  //                                                   double & rdAverageLengthW,
  //                                                   double & rdEstimatedVolumeBound) ;

  // copy a subset of the knot vector for specified parameter direction
  SmStatus GetKnotsExpert          
  (
    SmVolumeParamType eVolumeParam,     ///< [in] : oneof: SM_VP_U, SM_VP_V, SM_VP_W                            <br>
    ULONG lStartIndex,                  ///< [in] : 1st knot index to extract                                   <br>
    ULONG lEndIndex,                    ///< [in] : last knot index to extract                                  <br>
    double *adKnots                     ///< [out]: specified knot values,  sized:[lEndIndex-lStartIndex+1]     <br>
  ) const;

  // copy a subset of the control point 3d array into an ordered 1d array (may be spaced out in the output)
  SmStatus GetControlPointsExpert  
  (
    SmControlPointFormType eCtrlPointForm, ///< [in] : SM_CP_NON_RATIONAL           - output euclidean coords only                     <br>
                                           ///<        SM_CP_HOMOGENEOUS_RATIONAL   - output homogeneous coords and weights            <br>
                                           ///<        SM_CP_EUCLIDIAN_RATIONAL     - output euclidean coords and weights              <br>
    ULONG              lStartUIndex,       ///< [in] : 1st UIndex to copy                                                              <br>
    ULONG              lEndUIndex,         ///< [in] : last UIndex to copy                                                             <br>
    ULONG              lStartVIndex,       ///< [in] : 1st VIndex to copy                                                              <br>
    ULONG              lEndVIndex,         ///< [in] : last VIndex to copy                                                             <br>
    ULONG              lStartWIndex,       ///< [in] : 1st WIndex to copy                                                              <br>
    ULONG              lEndWIndex,         ///< [in] : last WIndex to copy                                                             <br>
    ULONG              lCtrlPointUStride,  ///< [in] : lCtrlPointUStride = M * PtSize, M > 1 for U output spaces                       <br>
    ULONG              lCtrlPointVStride,  ///< [in] : lCtrlPointVStride = N * ((lEndUIndex - lStartUIndex + 1) * lCtrlPointUStride)   <br>
    ULONG              lCtrlPointWStride,  ///< [in] : lCtrlPointWStride = O * ((lEndVIndex - lStartVIndex + 1) * lCtrlPointVStride)   <br>
    double           * adControlPoints     ///< [in] : copied control points optionally spaced out by M, N, O > 1 values               <br>
                                           ///<       sized:[lCtrlPointWStride * (lEndWIndex - lStartWIndex + 1)]                      <br>
   ) const;
  
  // extract all BSplineVolume shape parameters
  SmStatus GetCanonical            
  (
    ULONG & rlUDegree,                          ///< [out]: degree in U direction                                                   <br>
    ULONG & rlVDegree,                          ///< [out]: degree in V direction                                                   <br>
    ULONG & rlWDegree,                          ///< [out]: degree in W direction                                                   <br>
    SmTArray<SmPoint3d> & rControlPointsList,   ///< [out]: Always produced in Euclidian coordinates                                <br>
    SmTArray<ULONG> & rUKnotMultiplicities,     ///< [out]: U Knot multiplicities                                                   <br>
    SmTArray<ULONG> & rVKnotMultiplicities,     ///< [out]: V Knot multiplicities                                                   <br>
    SmTArray<ULONG> & rWKnotMultiplicities,     ///< [out]: W Knot multiplicities                                                   <br>
    SmTArray<double> & rUKnots,                 ///< [out]: U unique knot array                                                     <br>
    SmTArray<double> & rVKnots,                 ///< [out]: V unique knot array                                                     <br>
    SmTArray<double> & rWKnots,                 ///< [out]: W unique knot array                                                     <br>
    SmTArray<double> & rWeights                 ///< [out]: The Weights array will have size zero if the volume is non-rational.    <br>
  ) const;

  // set data
  SmStatus SetCanonical
  (
    ULONG                       lUDegree,                ///< [in] : U dir Degree                                              <br>
    ULONG                       lVDegree,                ///< [in] : V dir Degree                                              <br>
    ULONG                       lWDegree,                ///< [in] : W dir Degree                                              <br>
    const SmTArray<SmPoint3d> & crControlPointsList,     ///< [in] : Euclidian coordinates                                     <br>
    const SmTArray<ULONG>     & crUKnotMultiplicities,   ///< [in] : multiplicity of each knot in crUKnots                     <br>
    const SmTArray<ULONG>     & crVKnotMultiplicities,   ///< [in] : multiplicity of each knot in crVKnots                     <br>
    const SmTArray<ULONG>     & crWKnotMultiplicities,   ///< [in] : multiplicity of each knot in crWKnots                     <br>
    const SmTArray<double>    & crUKnots,                ///< [in] : Knots in U                                                <br>
    const SmTArray<double>    & crVKnots,                ///< [in] : Knots in V                                                <br>
    const SmTArray<double>    & crWKnots,                ///< [in] : Knots in W                                                <br>
    const SmTArray<double>    * cpOptWeights = NULL      ///< [in] : optional weight for each ControlPoint, same size & order  <br>
  );

  SmStatus SetControlPoint
  (
    SmControlPointFormType eCtrlPointForm,     ///< [in] : SM_CP_EUCLIDIAN_RATIONAL = convert point to homogeneous form               <br>
                                               ///<        SM_CP_NON_RATIONAL       = store point in euclidean space                  <br>
    ULONG                  lUIndex,            ///< [in] : 1st index of Target Mesh CPoint to modify                                  <br>
    ULONG                  lVIndex,            ///< [in] : 2nd index of Target Mesh CPoint to modify                                  <br>
    ULONG                  lWIndex,            ///< [in] : 3rd index of Target Mesh CPoint to modify                                  <br>
    const SmPoint3d      & crControlPoint,     ///< [in] : euclidian space point                                                      <br>
    double                 dWeight             ///< [in] : associated weight only used when eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL<br>
  );

  SmStatus SetExpert
  (
    ULONG                  lUDegree,              ///< [in] : U dir degree                                                                             <br>
    ULONG                  lVDegree,              ///< [in] : V dir degree                                                                             <br>
    ULONG                  lWDegree,              ///< [in] : W dir degree                                                                             <br>
    SmEndKnotFormType      eEndKnotForm,          ///< [in] : SM_EK_CLAMPPED   = knot arrays contain unused extra endKnot                              <br>
                                                  ///<        SM_EK_UNCLAMPPED = knot arrays missing unused extra endKnot                              <br>
    ULONG                  lUNumKnots,            ///< [in] : U knot count                                                                             <br>
    const double          *cadUKnots,             ///< [in] : U Knot array                                                                             <br>
    ULONG                  lVNumKnots,            ///< [in] : V knot count                                                                             <br>
    const double          *cadVKnots,             ///< [in] : V Knot array                                                                             <br>
    ULONG                  lWNumKnots,            ///< [in] : W knot count                                                                             <br>
    const double          *cadWKnots,             ///< [in] : W Knot array                                                                             <br>
    SmControlPointFormType eCtrlPointForm,        ///< [in] : SM_CP_NON_RATIONAL         = PointSize = 3                                               <br>
                                                  ///<        SM_CP_HOMOGENEOUS_RATIONAL = PointSize = 4                                               <br>
                                                  ///<        SM_CP_EUCLIDIAN_RATIONAL   = PointSize = 4                                               <br>
    ULONG                  lCtrlPointUStride,     ///< [in] : Stride in cadCtrlPoints for each UIndex increment                                        <br>
    ULONG                  lCtrlPointVStride,     ///< [in] : Stride in cadCtrlPoints for each VIndex increment                                        <br>
    ULONG                  lCtrlPointWStride,     ///< [in] : Stride in cadCtrlPoints for each WIndex increment                                        <br>
    const double          *cadCtrlPoints          ///< [in] : Control Points Array, sized:[PointSize * (lUPointCount * lVPointCount * lWPointCount)]   <br>
                                                  ///<        ordered so that                                                                          <br>
                                                  ///<        P[i][j][k].x = cadCtrlPoints[  i * lCtrlPointUStride                                     <br>
                                                  ///<                                     + j * lCtrlPointVStride                                     <br>
                                                  ///<                                     + k * lCtrlPointWStride]                                    <br>
  );

  SmStatus SetFromGwNurb(ULONG, gw_VOLUME * pGwNurbVolume) ;

  void SetOutOfBoundsEnabled (SmBoolean bOutOfBoundsEnabled) { m_bOutOfBoundsEnabled = bOutOfBoundsEnabled; }

  // Queries                            
  SmBoolean  IsRational()           const ;
  SmBoolean  IsNurbBorrowed()       const { return m_bNurbIsBorrowed ? TRUE : FALSE ; }
  SmBoolean  IsOutOfBoundsEnabled() const { return m_bOutOfBoundsEnabled ? TRUE : FALSE ; }

  SmStatus   FindSpans
  (
    const SmPoint3d & crParamPoint,         ///< [in] :                                                                               <br>
    SmBoolean         bUFromLeft,           ///< [in] : if P is on U, V, or w interval boundary                                       <br>
    SmBoolean         bVFromLeft,           ///<        TRUE  = evaluate P in upper interval where P is on the left of the interval   <br>
    SmBoolean         bWFromLeft,           ///<        FALSE = evaluate P in lower interval where P is on the right of the interval  <br>
    ULONG            & rUIndex,             ///< [out]:                                                                               <br>
    ULONG            & rVIndex,             ///< [out]:                                                                               <br>
    ULONG            & rWIndex              ///< [out]:                                                                               <br>
  ) const ;

  // make exact BSplineCurve projection of InputSpace Curve to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutCurve
  (
    SmVolumeSpaceTYPE eInputSpace,       ///< [in] : SM_VS_IN_SPACE   = InputCurve is projected from InSpace      <br>
                                         ///<        SM_VS_PARAM_SPACE= InputCurve is projected from ParamSpace   <br>
    const SmCurve   & rInputCurve,       ///< [in] : Curve to project to 1st OutSpace                             <br>
    SmBSplineCurve *& rpNewCurve         ///< [out]: 1st OutSpace projection or NULL for not possible             <br>
  )  const
  {
    SM_REF2( rInputCurve, eInputSpace );
    rpNewCurve = NULL;
    return(SM_SUCCESS);
  }

  // make exact BSplineCurve projection of InputSpace Surface to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutSurface
  (
    SmVolumeSpaceTYPE   eInputSpace,   ///< [in] : SM_VS_IN_SPACE   = InputSurface is projected from InSpace         <br>
                                       ///<        SM_VS_PARAM_SPACE= InputSurface is projected from ParamSpace      <br>
    const SmSurface   & rInputSurface, ///< [in] : Surface to project to Last OutSpace                               <br>
    SmBSplineSurface *& rpNewSurface   ///< [out]: 1st OutSpace projection or NULL for not possible                  <br>
  ) const
  {
    SM_REF2( eInputSpace, rInputSurface );
    rpNewSurface = NULL;
    return(SM_SUCCESS);
  }

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

  // compute ProjSpace BBoxes for given ParamSpace Box
  virtual SmStatus EvaluateBoundingBoxSimple
  (
    const SmExtent3d  & crParamDomain,               ///< [in] : ParamSpace BBox to project to Project Space                                                            <br>
    SmPseudoBox       * pOptPseudoParamBox = NULL,   ///< [in] : optional ParamSpace PseudoBox used to set output PseudoBox orientations,                               <br>
                                                     ///<        NULL   : ProjPseudoBox Basis = vecs connecting crParamBox corner projections into ProjSpace            <br>
                                                     ///<        NotNULL: ProjPseudoBox Basis = Project pOptParamPseudoBox BasisVecs to ProjSpace at crParamBox Center  <br>
    SmExtent3d        * pNormalProjBox = NULL,       ///< [out]: ProjectSpace Axis aligned box                                                                          <br>
    SmPseudoBox       * pPseudoProjBox = NULL        ///< [out]: ProjectSpace Non-axis aligned box                                                                      <br>
  ) const ;

  // Create ProjSpace IsoCurve from a ParamSpace IsoParamLine
  virtual SmStatus EvaluateIsoParametricCurveSimple
  (
    const SmContext     & crContext,             ///< [in] : context for created objects                             <br>
    SmVolumeParamsType    eConstantParams,       ///< [in] : oneof: SM_VPS_UV_IN,                                    <br>
                                                 ///<               SM_VPS_UW_IN,                                    <br>
                                                 ///<               SM_VPS_VW_IN.                                    <br>
    double                dIsoParam1,            ///< [in] : 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value   <br>
    double                dIsoParam2,            ///< [in] : 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value   <br>
    double                d3DTolerance,          ///< NotUsed: [in] : Max ApproxCurve to IdealCurve deviation                 <br>
    SmCurve            *& rpNewIsoCurve,         ///< [out]: the ProjSpace IsoCurve                                  <br>
    const SmExtent3d    * pOptParamDomain=NULL   ///< [in] : limiting domain, NULL to ignore. default:[NULL]         <br>
  ) const ;

  // Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane
  virtual SmStatus EvaluateIsoParametricSurfaceSimple
  (
    const SmContext   & crContext,             ///< [in] : context for created objects                       <br>
    SmVolumeParamType   eConstantParam,        ///< [in] : oneof: SM_VP_U_IN,                                <br>
                                               ///<               SM_VP_V_IN,                                <br>
                                               ///<               SM_VP_W_IN                                 <br>
    double              dIsoParam,             ///< [in] : constant param value                              <br>
    double              d3DTolerance,          ///< NotUsed: [in] : Max ApproxSurface to IdealSurface deviation       <br> 
    SmSurface        *& rpNewIsoSurface,       ///< [out]: the ProjSpace IsoSurface                          <br>
    const SmExtent3d  * pOptParamDomain=NULL   ///< [in] : limiting domain, NULL to ignore. default:[NULL]   <br>
  ) const ;

  // evaluate BSpline volume map from ParamSpace to ProjSpace
  virtual SmStatus EvaluateSimple                  
  (
    const SmPoint3d & crParamPoint,     ///< [in] : ParamSpace point to map to ProjSpace point                                       <br>
    ULONG             lHighestDeriv,    ///< [in] : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                         <br>
    SmBoolean         bUFromLeft,       ///< [in] : if P is on U, V, or w interval boundary                                          <br>
    SmBoolean         bVFromLeft,       ///<       TRUE  = evaluate P in upper interval where P is on the left of the interval       <br>
    SmBoolean         bWFromLeft,       ///<       FALSE = evaluate P in lower interval where P is on the right of the interval      <br>
    SmVector3d      * aDerivatives,     ///< [out]: matrix of ParamSpace evaluations values                                          <br>
                                        ///<        3d organized: D[u][v][w]                                                         <br>
                                        ///<        1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3               <br>
                                        ///<        sized       : [n+1][n+1][n+1], where n=lHighesDeriv                              <br>
                                        ///<        lHghDrv = 0,   sized: [1],                                                       <br>
                                        ///<          i=0          order: [D]                                                        <br>
                                        ///<        lHghDrv = 1,   sized: [8]                                                        <br>
                                        ///<          i=u*4+v*2+w  order: [D  Dw  Dv  ---                                            <br>
                                        ///<                               Du --- --- ---]                                           <br>
                                        ///<        lHghDrv = 2,   sized: [27]                                                       <br>
                                        ///<          i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---                       <br>
                                        ///<                               Du  Duw --- Duv --- --- --- --- ---                       <br>
                                        ///<                               Duu --- --- --- --- --- --- --- ---]                      <br>
                                        ///<        lHghDrv = 3,   sized: [81]                                                       <br>
                                        ///<          i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw         <br>
                                        ///<                               --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---         <br>
                                        ///<                               Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---         <br>
                                        ///<                               --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---         <br>
                                        ///<                               --- ---  ---   ---   ---  ---  ---  ---  Duuu ---         <br>
                                        ///<                               --- ---  ---   ---   ---  ---  ---  ---  ---- ---         <br>
                                        ///<                               --- ---  ---   ---   ---  ---  ---  ---  ---- ---         <br>
                                        ///<                               --- ---  ---   ---   ---  ---  ---  ---  ---- ---         <br>
                                        ///<                               --- --- ]                                                 <br>
    SmBoolean bNonZeroTangents=TRUE,    ///< [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors     <br>
                                        ///<        FALSE= return exact tangent values, default:[TRUE]                               <br>
                                        ///<        note: Surprisingly TRUE is the common choice because most tangent uses           <br>
                                        ///<              are for their direction (Binorm, SurfNorm comps), but when the             <br>
                                        ///<              tangent is being used for its magnitude (like an arc-length comp)          <br>
                                        ///<              then set this to FALSE.                                                    <br>
    SmBoolean bDoZeroSampling=TRUE      ///< [in] : for internal use only, always set to TRUE                                        <br>
  ) const ;                            

  // get approximate ParamSpace location for given ProjSpace point for upcoming newton raphson search
  virtual SmStatus InvEvaluateGuessPointSimple           
  (
    const SmPoint3d     & crProjPoint,         ///< [in] : ProjSpace Point to map back to ParamSpace Point                     <br>
    SmTArray<SmPoint3d> & rGuessParamPoints    ///< [out]: ParamSpace Point near Target Point actual map back to ParamSpace    <br>
  ) const ;                             
  
  // no local implementations of solvers: defauilt base behavior of inverting with NewtonRaphson is used 
  //    GlobalPointSolveSimple()    - use base default behavior
  //    LocalPointSolveSimple()     - use base default behavior

  // GWC: TODO - decide if inherited or local behavior is needed for following 3 methods
  //        FindParamIntervalForInSpaceLineSimple() // Opt implementation: Find InSpace Line interval that maps to the NaturalParamDomain, default rTrimIvl = crCurrentIvl
  //        TrimParamBoundingBoxSimple()            // opt implementation: Trim ParamBBox to natural ParamDomain, default pTrimBox = pGivenBox
  //        TrimInSpaceBoundingBoxSimple()          // opt implementation: Trim InSpaceBBox to map within the NaturalParamSpaceDomain, default pTrimBox = pGivenBox
                                          
  // Modify SimpleMap params so: Orient(ModifiedSimpleMap(uvw)) = Transform(Orient(SimpleMap(uvw)))                                                                  
  virtual SmStatus  TransformSimple
  (
    const SmAxis2Placement & crOutRotateNMove,     ///< [in] : OutSpace Rotate and Move Transform                               <br>
    const SmVector3d       * cpOptOutScale = NULL  ///< [in] : Optional OutSpace scale factors applied after Rotate and Move    <br>
  ) ;

  // Orient(ModSimpleMap(uvw)) = Translate(Orient(SimpleMap(uvw)))
  virtual SmStatus  TranslateSimple(const SmVector3d & crOutTranslate) ;           
  
  // Orient(ModSimpleMap(uvw)) = Mirror   (Orient(SimpleMap(uvw)))
  virtual SmStatus  MirrorSimple   
  (
    const SmPoint3d  & crPlaneOutPt,       ///< [in] : Pt on Mirror Plane given in ProjSpace         <br>
    const SmVector3d & crPlaneOutNormal    ///< [in] : Normal to Mirror Plane given in ProjSpace     <br>
  ) ;  

  // Orient(ModSimpleMap(uvw)) = Scale    (Orient(SimpleMap(uvw)))
  virtual SmStatus  ScaleSimple    
  ( 
    const SmVector3d & crScaleOutVec,          ///< [in] : OutSpace scale factors                    <br>
    const SmPoint3d  * cpOptOutCenter = NULL   ///< [in] : Optional OutSpace scaling center point    <br>
  ) ;

  // Orient(ModSimpleMap(uvw)) = RotateAboutAxis(Orient(SimpleMap(uvw)))
  virtual SmStatus  RotateAboutAxisSimple
  (
    double             dAngRad,      ///< [in] : OutSpace rotation AngRad     <br>    
    const SmVector3d & crOutAxis     ///< [in] : OutSpace rotation axis       <br>
  ) ;    

  // Orient(ModSimpleMap(uvw)) = RotateAboutAxisAtPoint(Orient(SimpleMap(uvw)))
  virtual SmStatus  RotateAboutAxisAtPointSimple 
  (
    double             dAngRad,       ///< [in] : OutSpace rotation AngRad           <br>
    const SmPoint3d  & crOutOrigin,   ///< [in] : OutSpace point on rotation axis    <br>
    const SmVector3d & crOutAxis      ///< [in] : OutSpace rotation axis direction   <br>
  ) ;

  // predicates

  // rtn: TRUE if this simple volume's ParamSpace is bounded
  virtual SmBoolean IsBoundedSimple() const { return(TRUE) ; }      

  // for these predicates - use default point sampling methods
  //      virtual SmBoolean IsClosedSimple ;
  //      virtual SmBoolean IsPeriodicSimple ;
  //      virtual SmBoolean IsSingularitySimple ;
  //      virtual SmBoolean IsOnBoundarySimple ;

  // for these predicates - use default behavior because ParamSpace domain is a BBox
  //        IsPointInParamDomainSimple()          // opt implementation: Return TRUE if ParamPoint is within the map's paramSpace Natural domain, default:[return TRUE]                                   
  //        IsPointInProjDomainSimple()           // opt implementation: Return TRUE if projPoint can be inverted uniquely to ParamSpace, default:[return TRUE]                                   
  //        IsLineInParamDomainSimple()           // opt: rtn TRUE if ParamLine is in map's NaturalParamDomain. default:[if bnded(GetNaturalDomain().Contains(ParamPt)) else TRUE], implement if:[ParamDomain is bounded not rectilinear]                                   
  //   ToBe IsLineInProjDomainSimple()            // opt: rtn TRUE if ProjLine drops within map's NaturalParamDomain. default:[if bnded(InvEvaluatePointSimple()) else TRUE], implement if:[ParamDomain is bounded not rectilinear]

  virtual SmBoolean IsCurveInParamDomainSimple(const SmCurve & crParamCurve) const ;

  virtual SmBoolean IsSurfaceInParamDomainSimple(const SmSurface & crParamSurface) const ;

  // rtn: TRUE if Simple map has internal C1 discontinuities
  virtual SmBoolean HasDiscontinuitiesSimple                        
                      
  (
    SmDiscontinuities3d *pOptDisconts,       ///< [out]: list of discontinuities and summary data                                              <br>
    SmBoolean bCalcGeometric=TRUE            ///< [in] : TRUE = expensive - use geometric testing to compute actal geometric discontinuities   <br>
                                             ///<        FALSE= cheap - report representational discontinuites                                 <br>
  ) const ;                                       
                                                                    
  virtual SmStatus CalculateContinuitiesSimple      
  (
    SmVolumeParamType            eVolumeParam,        ///< [in] : SM_VP_U, SM_VP_V, or SM_VP_W                                                        <br>
    SmContinuityType           & reMinContinuity,     ///< [out]: minimum continuity over all interior knots                                          <br>
    SmTArray<double>           & rParams,             ///< [out]: param values marking discontinuity                                                  <br>
    SmTArray<SmContinuityType> & rConts,              ///< [out]: assocaited continuity type for each rParams value                                   <br>
                                                      ///<        end param continuities = SM_CT_DISCONTINUOUS                                        <br>
    SmBoolean                    bCalcGeometric=TRUE  ///< [in] : TRUE = expensive - use geometric testing to compute actal geometric discontinuities <br>
                                                      ///<        FALSE= cheap - report representational discontinuites                               <br>
  )  const ;                                            

  // I/O assist
  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,                 ///< [in] : target output stream                               <br>
    ULONG          lDBVersionNumber     ///< [in] : database version to get proper sequence of writes  <br>
  ) const ;                                                                        

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,              ///< NotUsed: [in] : Object type to be read                                                        <br>
    SmDatabaseIO    & rDB,                ///< [in] : target output stream                                                          <br>
    const SmContext & crContext,          ///< [in] : context for new object construction                                           <br>
    SmVolume       *& rpNewVolume,        ///< [out]: NULL on input = new object allocated in this routine built from stream data   <br>
                                          ///<        NotNULL on input = pointer to an empty object to be filled by this routine    <br>
    ULONG             lDBVersionNumber    ///< [in] : database version to get proper sequence of writes                             <br>
  ) ; 

  // gwc: a function to consider adding later on
  //      // output volume Polygons through SmPolygonOutputCallback.OutputPolygon call
  //      SmStatus OutputPolygons(double dVolumeChordHeightTolerance,
  //                              double dCurveChordHeightTolerance,
  //                              SmBoolean bReverseNormals, 
  //                              SmPolygonOutputCallback & rPolygonOutput) const;

  // parameters for rendering
  SmStatus CalculatePartialMeshValues
  (
    const SmExtent3d & crParamDomain,              ///< [in] : ParamSpace domain to query                                                   <br>
    SmPoint3d     aCorners[2][2][2],               ///< [out]: ParamDomain corners mapped to ProjSpace, ordered:[aCorners[u][v][w]]         <br>
    SmExtent3d  * pOptNormalBox = NULL,            ///< [out]: ProjSpace bounding box, NULL to ignore, default:[NULL]                       <br>
    SmPseudoBox * pOptPseudoBox = NULL,            ///< [out]: ProjSpace pseudo box, NULL to ignore, default:[NULL]                         <br>
    SmPoint3d   * pOptUVWChordHeight = NULL,       ///< [out]: Max projX,Y,Z Chord height along any single row of varying uvw CPoints       <br>
                                                   ///<        NULL to ignore, default:[NULL]                                               <br>
    SmPoint3d   * pOptUVWAngleTolDegree = NULL     ///< [out]: Max ProjSpace tangAngle change along any single row of varying uvw CPoints   <br>
                                                   ///<        NULL to ignore, default:[NULL]                                               <br>
  ) const;

  // get memory used for volume and its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes
    SmMarkType eMarkType=SM_MT_NOMARK  ///< [in] : uses without increment eMarkType value
  ) const ;

  // Declare GetType(), GetTypeString(), GetClassType(), GetClassTypeString, IsKindOf(), Dump() - requires Dump() implementation.
  SM_COMMON(SmBSplineVolume,SmVolume,SmBSplineVolume_TYPE);

  //      void              Dump()                  const ;  // Dump(abbrev=FALSE, lIndentCnt=0)
  virtual void              Dump(TCHAR * message,   ULONG lIndentCnt=0) const ;  // Dump(abbrev=FALSE) preceded by a one line message
  virtual void              Dump(ULONG i,           ULONG lIndentCnt=0) const ;  // Dump(abbrev=FALSE) preceded by an integer (ULONG)
  virtual void              Dump(SmBoolean bAbbrev, ULONG lIndentCnt=0) const ;  // Dump abbrev: TRUE = output corner control pts, FALSE = all pts

  SmBoolean PassesValidityCheck
  (
    SmValidityCheckType   eChecks,
    SmValidityCheckType & reCheckFailed
  ) const;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out] : Accumulating list of failed Asserts, NULL to ignore                       <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                    <br>
                                              ///<        SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order              <br>
  ) const ;

  // gwc: a function to consider adding later on
  //      // output volume Polygons through SmPolygonOutputCallback.OutputPolygon call
  //      SmStatus OutputPolygons(double    dVolumeChordHeightTolerance,
  //                              double    dCurveChordHeightTolerance,
  //                              SmBoolean bReverseNormals, 
  //                              SmPolygonOutputCallback & rPolygonOutput) ///< [in] : Chooses how and where to output the polygons
  //                            const;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // rtn SM_SUCCESS when no control points are coincident, else rtn SM_ERR
  SmStatus TestDegenerate
  (
    double dTol,            ///< [in] : Min 3d distance between distinct ControlPoints       <br>
    ULONG  &rNu,            ///< [out]: Index of 1st ControlPoint with a duplicate or 0      <br>
    ULONG  &rNv,            ///< [out]: Index of 1st ControlPoint with a duplicate or 0      <br>
    ULONG  &rNw             ///< [out]: Index of 1st ControlPoint with a duplicate or 0      <br>
  ) const ;      

  SmStatus TestCornerVolumes(double dTol) const ;   ///< [in] : ScaledZero for length of a vector      <br>
                                                    ///<        degenerate when Volume <= Tol*Tol*Tol  <br>

} ; // end class SmBSplineVolume

/***********************************************************//**
* PURPOSE - convenience macro for managing m_pNurb construction
*
****************************************************************/
#define  SM_ENSURE_VOLUME_MPNURB(a)  \
if( (a)->IsKindOf(SmBSplineVolume_TYPE)) \
  { ((SmBSplineVolume *)(a))->GetOrCreateGwNurbPointer() ; }



// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmBSplineVolume*)

#endif // !__SMBSPLINEVOLUME_H__

