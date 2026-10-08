// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSpaceUnbend.h
* PURPOSE: Header file for SmSpaceUnbend object.
**********************************************************************/

#ifndef __SMSPACEUNBEND_H__
#define __SMSPACEUNBEND_H__

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#include <SmSurfTypes.h>
#include <SmCoreTypes.h>
#include <SmCurveTypes.h>
#include <SmVector3d.h>
#include <SmTopoTypes.h>
#include <SmCurveTypes.h>
#include <SmTopologyIntersector.h>
#include <SmEdge.h>
#include <SmVertex.h>
class SmUnbendVolume ;
class SmSrfInVolume ;
class SmSpaceBend ;

extern SmBendOrientTYPE   MapType(SmUnbendOrientTYPE) ;
extern SmUnbendOrientTYPE MapType(SmBendOrientTYPE) ;

/*******************************************************************//**
PURPOSE: The SmSpaceUnbend object directs the unbending of a sheet metal Brep model.

NOTES:
 language: (same shape as the SmBendSpace operator with an inverse shape change)
  The Unbend description:
                 Post Unbend line                           -+-------------+-------> UnbendAxis
                            +----*----*------+               | UnbendRadius|
                                 |____|<-UnbendInterval      v             | NeutralRadius in UnbendDirection
                                                         +-*-------------|---------+
                              +          +               | ___ ___ ___ __v ___ ___ |
                               \<------>/<--UnbendAngle  |                         |
                Pre Unbend line \      /                 |                         |
                                 *-__-*                  +-------------------------+
                                                          Unbend Block cross section
                                                      at center of unbend perp to the page


    UnbendAngle      = Extent of the unbend from -180 to +180 degrees.
    UnbendRadius     = distance between the UnbendAxis and the sheet metal's inside surface.
    NeutralRadius    = distance between the UnbendAxis and the NeutralAxis.

  The Unbend orientation:
    UnbendAxisPoint  = the origin of the UnbendCenterLine = UnbendAxisPoint + s * UnbendAxis
    UnbendAxis       = unbend's geometric center line located parallel and above the sheet metal inside surface.
                        (The unbend's Z sxis - in the Neutral plane)
    UnbendDirection  = a vector from the NeutralPlane to UnbendAxis specifying the inside part of the unbend.
                        (The unbend's X axis - perpendicular to both the UnbendAxis and the NeutralPlane)
    UnbendBinormal   = defined as UnbendAxis * UnbendDirection
                        (The unbend's Y axis - in the Neutral Plane)
    UnbendInterval   = the length of the UnbendSection which is sized and positioned by the segment
                       UnbendSegment(s) = UnbendAxisPoint + UnbendInterval.Evaluate(s) * NeutralAxis
                          for s values from 0.0 to 1.0.
    UnbendSegment    = Portion of the UnbendAxis within the UnbendRegion
  The Unbend region:
    UnbendRegion     = the region in which the unbend is applied. The UnbendRegion is a rectilinear
                       solid aligned with the NeutralPlane and the Neutral Axis.
                         height = Thickness with the KFactor determining where the
                                    UnbendRegion sits in relation to the NeutralPlane.
                         width  = NeutralRadius * UnbendAngleRad,
                                    which is centered on the NeutralAxis.
                         length = length of the NeutralAxis vector and its length is positioned
                                   by the segment = UnbendAxisPoint + UnbendInteral.Evaluate(s) * NeutralAxis
                                   for s values from 0.0 to 1.0.
                       Only those brep geometry pieces completely within the UnbendRegion are bent.
  The Unbend NeutralPlane geometry:
    NeutralPoint    = UnbendAxisPoint + UnbendRadius * UnbendDirection
    NeutralAxis     = line parallel to UnbendAxis in the NeutralPlane.
                      NeutralAxis(s) = UnbendAxisPoint + s * UnbendAxis
    NeutralPlane    = plane within the sheet metal that maintains its length through the unbend operation.
                      NeutralPlane(u,s) = UnbendAxisPoint + u * UnbendBinormal + s * UnbendAxis

  The Sheet metal:
    KFactor         = location of the neutral plane represented as the normalized distance (ranging from 0.0 to 1.0)
                         between the sheet metal inside and outside surfaces.
    Thickness       = distance between the sheet metal inside and outside surfaces.

  The Part final position options: Ways to specify the after unbend orientation of the Brep.
    SM_UO_FIXED_START_BENDING = The portion of the Brep before the unbend
                                (in the direction of Cross(UnbendDirection,NeutralAxis))
                                remains fixed throughout the unbend and the Brep portion
                                after the unbend is rotated by the UnbendAngle.
    SM_UO_CENTER_BENDING      = The portions of the Brep before and after the unbend are both
                                rotated about the unbend by one half the UnbendAngle.
    SM_UO_FIXED_END_BENDING   = The portion of the Brep after the unbend
                                (in the direction opposite of Cross(UnbendDirection,NeutralAxis))
                                remains fixed throughout the unbend and the Brep portion
                                before the unbend is rotated by the UnbendAngle.
 Notes and Rules:
  1 : NeutralRadius = UnbendInsideRadius + KFactor * SheetThickness
  2 : NeutralRadius >= kFactor * SheetThickness
  3 : It's assumed the sheet metal inside and outside surfaces and the unbend's UnbendAxis, NeutralAxis,
        and NeutralPlane are all parallel to one another.
  4 : The Brep's before unbend geometry is all the geometry which connects to the before unbend side which
       is in the -UnbendBinormal direction).
  5 : Similarly, the Brep's after unbend geometry connects to the after unbend side which is in the
       +UnbendBinormal direction).
  6 : Pieces of geometry COMPLETELY within the UnbendRegion are bent.
  7 : Pieces of geometry COMPLETELY within the Before UnbendRegion are rotated and moved.
  8 : Pieces of geometry COMPLETELY within the After UnbendRegion are rotated and moved.
  9 : No check is made for geometry moved to self-intersecting positions.

 note: A curve (or surface) bent by AngleDeg can be restored
       to its original shape by an unbend of AngleDeg as long as
       the SmSpaceUnbend object has the same shape and position
       as the SmSpaceBend object used to bend the shape.

 NOTE: The SmSpaceBend and SmSpaceUnbend classes duplicate most of their methods (yuk).
       If you debug one class - duplicate the debug change in the other class.

***********************************************************************/
class SM_EXPORT SmSpaceUnbend : public SmObject
{
protected:
  // the target
  SmBrep            * m_pBrep ;               // Brep to unbend (containing Context used for new obj construction)

  // the unbend
  double              m_dUnbendAngDeg ;       // Extent of the unbend in degrees from -180 to + 180.
                                              //    Often just [30, 45, 60, 90, 120, 135, 150, 180]

  double              m_dNeutralRadius ;      // Unbend radius measured at the neutral axis, not the inside face of the sheet
                                              //  NeutralRadius = UnbendRadius + KFactor * SheetThickness

  // unbend orientation
  SmPoint3d           m_vUnbendAxisPoint ;    // A point on UnbendAxis.
                                              //   UnbendAxisPoint = UnbendPoint + NeutralRadius * UnbendDir
  SmVector3d          m_vUnbendAxis ;         // unit vector pointing along the unbend axis.
                                              //  (Unbend Z Axis)
  SmVector3d          m_vUnbendDirection ;    // unit vector pointing from the UnbendAxisPoint to the center of the NeutralArc.
                                              //  (Unbend X Axis - perpendicular to both UnbendAxis and NeutralPlane)

  // unbend linear extent
  SmExtent1d          m_vUnbendInterval ;     // Subinterval of the Line(s) = UnbendAxisPoint + s*UnbendAxis
                                              //   which is bent.  This allows subsections of a Brep to
                                              //     be bent while ignoring other pieces.
                                              // IsInit(m_vUnbendInterval) == TRUE, means the interval is infinite - unbend
                                              //     all pieces of the Brep on unbend plane
  // the sheet metal
  double              m_dSheetThickness ;     // Sheet metal thickness.
  double              m_dKFactor ;            // Locates the neutral axis between the inside and outside sheet surfaces.
                                              //   0.0 = neutral plane is the inside unbend surface.
                                              //   1.0 = neutral plane is the outside unbend surface.
                                              //   0.5 = neutral plane is midway between the inside and outside unbend surfaces.
                                              //  default:[0.44]

  // tolerance when not using exact geometry
  double              m_dApproxTol ;          // Max distance between approx BSpline and exact unbend shapes
                                              // default:[1.0e-5] - for now SmBrep::ApproximateWithBSplines() has trouble hitting
                                              //                    this tolerance - use 1.0e-4 or larger.

  // options for final part orientation
  SmUnbendOrientTYPE  m_eUnbendOrient ;       // selects the final orientation of the bent part
                                              // SM_EB_FIXED_START_BENDING = topology in front of unbend is fixed - after topology is rotated into position
                                              // SM_EB_CENTER_BENDING      = topology on either side of unbend is rotated in equal and opposite directions
                                              // SM_EB_FIXED_END_BENDING   = topology after unbend is fixed - before topology is rotated into position

  // chached data
  SmBrep            * m_pUnbendDomain ;       // A Brep representing the region of space being unbent. It's the geometric region
                                              //   selecting all m_pBrep geometry to unbend. m_pBendDomain is not a part of m_pBrep.
                                              // default:[NULL]
  SmBrep            * m_pStartUnbendBoundary; // A Brep representing the sheet start-unbend boundary of m_pUnbendDomain
  SmBrep            * m_pStopUnbendBoundary;  // A Brep representing the sheet stop-unbend boundary of m_pUnbendDomain

  double              m_dBrepSize ;           // used when Unbend Axis is unbounded (m_vUnbendInterval is set to Init() values)
                                              // default:[SM_UNDEF_DOUBLE]
public:
  // constructor
  SmSpaceUnbend
  (
    SmBrep           & rBrep,                 ///< [in ]: target Brep to unbend (containing Context used for new obj construction)                                     <br>
    double             dUnbendAngDeg,         ///< [in ]: unbend angle in degrees                                                                                      <br>
    double             dNeutralRadius,        ///< [in ]: unbend radius at the neutral plane                                                                           <br>
    const SmPoint3d  & rUnbendAxisPoint,      ///< [in ]: UnbendAxisPoint   of unbend Centerline = UnbendAxisPoint + s * UnbendAxisUnitVec.                            <br>
    const SmVector3d & rUnbendAxis,           ///< [in ]: UnbendAxisUnitVec of unbend Centerline = UnbendAxisPoint + s * UnbendAxisUnitVec,                            <br>
                                              ///<      : "the unbend's Z Axis."                                                                                       <br>
    const SmVector3d & rUnbendDirection,      ///< [in ]: unit-vector from UnbendAxisPoint towards NeutralPlane center,                                                <br>
                                              ///<      : perpendicular to the UnbendAxis and perpendicular to the neutral plane.                                      <br>
                                              ///<      : unbend's X Axis.                                                                                             <br>
    const SmExtent1d & rUnbendInterval,       ///< [in ]: active segment of unbend Centerline = UnbendAxisPoint + s * UnbendAxis.                                      <br>
                                              ///<      : use UnbendInterval.Init() for an unbounded unbend line.                                                      <br>
    double             dSheetThickness,       ///< [in ]: SheetThickness of NeutralRadius = InsideRadius + KFactor * SheetThickness                                    <br>
    double             dKFactor,              ///< [in ]: KFactor        of NeutralRadius = InsideRadius + KFactor * SheetThickness                                    <br>
                                              ///<      :  neutral plane normalized param between inside and outside sheet surfaces, commonly [.44]                    <br>
    SmUnbendOrientTYPE eUnbendOrient,         ///<      : selects the final orientation of the bent part. oneof:                                                       <br>
                                              ///<      : SM_UO_FIXED_BEFORE_BENDING = topology in front of unbend is fixed - after topology is rotated into position  <br>
                                              ///<      : SM_UO_CENTER_BENDING       = topology on either side of unbend is rotated in equal and opposite directions   <br>
                                              ///<      : SM_UO_FIXED_AFTER_BENDING  = topology after unbend is fixed - before topology is rotated into position       <br>
    double             dApproxTol = 1.0e-5    ///< [in ]: when not using exact geometry: max distance between approx BSpline and exact unbend shapes                   <br>
                                              ///<      :  default:[1.0e-5] - for now SmBrep::ApproximateWithBSplines() has trouble hitting                            <br>
                                              ///<      :                     this tolerance - currently bounded by 1.0e-4 or larger.                                  <br>
  )
    : SmObject( rBrep ),
    m_pBrep( &rBrep ),
    m_dUnbendAngDeg( dUnbendAngDeg ),
    m_dNeutralRadius( dNeutralRadius ),
    m_vUnbendAxisPoint( rUnbendAxisPoint ),
    m_vUnbendAxis( rUnbendAxis ),
    m_vUnbendDirection( rUnbendDirection ),
    m_vUnbendInterval( rUnbendInterval ),
    m_dSheetThickness( dSheetThickness ),
    m_dKFactor( dKFactor ),
    m_dApproxTol( dApproxTol > 1.0e-4 ? dApproxTol : 1.0e-4 ),
    m_eUnbendOrient( eUnbendOrient ),
    m_pUnbendDomain( NULL ),
    m_pStartUnbendBoundary( NULL ),
    m_pStopUnbendBoundary( NULL ),
    m_dBrepSize( SM_UNDEF_DOUBLE )

  { // work ahead - build needed preCache m_pUnbendDomain
    GetUnbendDomain();
  }

  // destructor
  virtual ~SmSpaceUnbend()
  {
    if(m_pUnbendDomain != NULL) { delete m_pUnbendDomain; m_pUnbendDomain = NULL; }
    if(m_pStartUnbendBoundary != NULL) { delete m_pStartUnbendBoundary; m_pStartUnbendBoundary = NULL; }
    if(m_pStopUnbendBoundary != NULL) { delete m_pStopUnbendBoundary; m_pStopUnbendBoundary = NULL; }
  }

  // create SpaceUnbend inverse for a given SpaceBend object
  static SmStatus CreateUnbendInverse
  (
    const SmSpaceBend & crSpaceBend,      ///< [in ]: the SpaceBend to invert                            <br>
    SmBrep            & rBrep,            ///< [in ]: Brep target for new rpSaceUnbend                   <br>
    SmSpaceUnbend    *& rpSpaceUnbend     ///< [out]: the SpaceUnbend inverse to the input SpaceBend.    <br>
  );

  // Create on the heap and return a SpaceUnbend defining UnbendVolume
  SmUnbendVolume * CreateUnbendVolume(const SmContext &crContext) const ;

  // execute unbend on m_pBrep, m_pBrep is modified in place
  SmStatus DoSpaceUnbend(SmBoolean bDoApproximations = TRUE) ;

  // internal assist methods

  // common derived geometric properties
  //   BrepSize         = MaxDimension of Brep BoundingBox
  //   InsideRadius     = UnbendRadius at sheet metal inside surface
  //   OutsideRadius    = UnbendRadius at sheet metal outside surface
  //   UnbendSegment      = The active portion along the unbend NeutralLine
  //                        along which m_pBrep geometry will be bent.  m_pBrep geometry
  //                        on the UnbendLine outside the UnbendSegment will not be bent.
  //   UnbendSegmentStart = Unbend NeutralAxis segment start point
  //   UnbendSegmentStop  = Unbend NeutralAxis segment end point
  //   UnbendLength       = Unbend NeutralAxis segment length
  //   PreUnbendRegion    = rectilinear region centered on the NeutralAxis segment
  //                        containing all geometyry to unbend in the preUnbend Brep
  //   PreUnbendCorner    = PreUnbendRegion start corner, a point on the unbend's outside surface
  //                      offset from the neutralAxis segment start point by (NeutralRadius * UnbendAngRad)

  SmBrep * GetBrep() const { return( m_pBrep ) ; }

  double               GetBrepSize()
  {
    if(m_dBrepSize == SM_UNDEF_DOUBLE)
    {
      SmExtent3d sSize;
      m_pBrep->CalculateBoundingBox( sSize, TRUE );
      m_dBrepSize = sSize.GetMaxDimension();
    }
    return(m_dBrepSize);
  }

  double               GetApproxTol()            const { return( m_dApproxTol ) ; }
  double               GetUnbendAngDeg()         const { return( m_dUnbendAngDeg ) ; }
  double               GetInsideRadius()         const { return( m_dNeutralRadius - m_dKFactor * m_dSheetThickness ) ; }
  double               GetOutsideRadius()        const { return( m_dNeutralRadius + (1.0 - m_dKFactor) * m_dSheetThickness ) ; }
  double               GetNeutralRadius()        const { return( m_dNeutralRadius ) ; }
  SmExtent1d           GetUnbendInterval()       const { return( m_vUnbendInterval ) ; }
  double               GetSheetThickness()       const { return( m_dSheetThickness ) ; }
  double               GetKFactor()              const { return( m_dKFactor ) ; }
  SmUnbendOrientTYPE   GetUnbendOrient()         const { return( m_eUnbendOrient ) ; }

  SmVector3d GetUnbendDirection()      const { return( m_vUnbendDirection ) ; }                 // The Unbend unit-vector X Axis
  SmVector3d GetUnbendBinormal()       const { return( m_vUnbendAxis * m_vUnbendDirection ) ; } // The Unbend unit-vector Y Axis
  SmVector3d GetUnbendAxis()           const { return( m_vUnbendAxis ) ; }                      // The Unbend unit-vector Z Axis
  SmPoint3d  GetUnbendAxisPoint()      const { return( m_vUnbendAxisPoint ) ; }                 // The Unbend origin

  // Points on the PreUnbend Neutral Arc
  SmPoint3d  GetNeutralBegPoint()      const { return( GetNeutralMidPoint().RotatePtAboutLine(m_vUnbendAxisPoint, m_vUnbendAxis, SM_DEG2RAD(-m_dUnbendAngDeg) / 2.0)) ; }
  SmPoint3d  GetNeutralMidPoint()      const { return( m_vUnbendAxisPoint + m_dNeutralRadius * m_vUnbendDirection ) ; }
  SmPoint3d  GetNeutralEndPoint()      const { return( GetNeutralMidPoint().RotatePtAboutLine(m_vUnbendAxisPoint, m_vUnbendAxis, SM_DEG2RAD( m_dUnbendAngDeg) / 2.0)) ; }

  // points on the UnbendAxis
  SmPoint3d  GetUnbendSegmentStart()   const
  {
    if(m_vUnbendInterval.IsInit()) { return(m_vUnbendAxisPoint - ((SmSpaceUnbend*)this)->GetBrepSize() * m_vUnbendAxis); }
    else { return(m_vUnbendAxisPoint + m_vUnbendInterval.Evaluate( 0.0 ) * m_vUnbendAxis); }
  }

  SmPoint3d  GetUnbendSegmentStop()    const
  {
    if(m_vUnbendInterval.IsInit()) { return(m_vUnbendAxisPoint + ((SmSpaceUnbend*)this)->GetBrepSize() * m_vUnbendAxis); }
    else { return(m_vUnbendAxisPoint + m_vUnbendInterval.Evaluate( 1.0 ) * m_vUnbendAxis); }
  }

  double     GetUnbendSegmentLength()  const
  {
    if(m_vUnbendInterval.IsInit()) { return(2.0 * ((SmSpaceUnbend*)this)->GetBrepSize()); }
    else { return(m_vUnbendInterval.GetLength()); }
  }

  // Unbend NeturalArc length
  double     GetPostUnbendWidth()      const
  {
    return(m_dNeutralRadius * SM_DEG2RAD( m_dUnbendAngDeg ));
  }

  // internal assist methods

  // Get (build when needed) preUnbendRegion represented as a Brep
  SmBrep * GetUnbendDomain() const ;
  SmBrep * GetStartUnbendBoundary() const { GetUnbendDomain() ; return(m_pStartUnbendBoundary) ; }
  SmBrep * GetStopUnbendBoundary()  const { GetUnbendDomain() ; return(m_pStopUnbendBoundary) ; }

  SmAxis2Placement GetBeforeUnbendTransform(SmUnbendOrientTYPE eOptOrientType=SM_UO_ORIENT_UNKNOWN) const ;  // NotUsed: in : eOptOrientType
  SmAxis2Placement GetInsideUnbendTransform(SmUnbendOrientTYPE eOptOrientType=SM_UO_ORIENT_UNKNOWN) const ;  // NotUsed: in : eOptOrientType
  SmAxis2Placement GetAfterUnbendTransform (SmUnbendOrientTYPE eOptOrientType=SM_UO_ORIENT_UNKNOWN) const ;  // NotUsed: in : eOptOrientType

  // internal unit-test for UnbendTransforms
  SmBoolean TestUnbendTransforms() const ;

  // apply rigid body translate and move transformation to listed vertices, edges, and faces
  SmStatus TransformTopology
  (
    const SmAxis2Placement    & crRotateNMove,  ///< [in ]: Rigid body transform to apply                     <br>
    SmBrep                    & rBrep,          ///< [in ]: Brep containing all topology to be transformed    <br>
    const SmTArray<SmFace*>   & crFaces,        ///< [in ]: faces to transform                                <br>
    const SmTArray<SmEdge*>   & crEdges,        ///< [in ]: edges to transform                                <br>
    const SmTArray<SmVertex*> & crVertices      ///< [in ]: vertices to transform                             <br>
  );

  // apply Unbend mapping to listed vertices, edges, and faces
  SmStatus UnbendTopology
  (
    const SmUnbendVolume      & crUnbendVolume, ///< [in ]: Unbend mapping to apply                           <br>
    SmBrep                    & rBrep,          ///< [in ]: Brep containing all topology to be transformed    <br>
    const SmTArray<SmFace*>   & crFaces,        ///< [in ]: faces to transform                                <br>
    const SmTArray<SmEdge*>   & crEdges,        ///< [in ]: edges to transform                                <br>
    const SmTArray<SmVertex*> & crVertices      ///< [in ]: vertices to transform                             <br>
  );

  // a specialized version of SmBrep::ReplaceSurface to save some copies only for SmSpaceUnbend internal use
  SmStatus ReplaceSurface
  (
     SmFace            * pFace,         ///< [in ]: Target Face to get new Surface                        <br>
     SmSrfInVolume     * pNewSurface    ///< [in ]: Target Surface to replace current pFace->Surface      <br>
  );

  // utilities
  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                             <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                               <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   <br>
  )  const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  SmDisplayList   * Draw
  (
    SmBoolean       bDrawBrep=FALSE,            ///< [in ]: TRUE = Draw target m_pBrep, default:[FALSE]                                          <br>
    SmBoolean       bAddToUIPickList=FALSE,     ///< NotUsed: [in ]: TRUE = Add target m_pBrep to UI pick interface for debugging, default:[FALSE]        <br>
    SmBoolean       bDrawPreUnbendRegion=FALSE, ///< [in ]: TRUE = Draw preUnbendRegion, default:[FALSE]                                         <br>
    SmGfxArraySet * pOptGfxSet=NULL             ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.      <br>
  ) const;

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmSpaceUnbend, SmObject, SmSpaceUnbend_TYPE) ;

} ; // end class SmSpaceUnbend

#endif // __SMSPACEUNBEND_H__


