// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSpaceBend.h
* PURPOSE: Header file for SmSpaceBend object.
**********************************************************************/

#ifndef __SMSPACEBEND_H__
#define __SMSPACEBEND_H__

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

class SmBendVolume;
class SmSrfInVolume ;
class SmSpaceUnbend ;

extern SmBendOrientTYPE   MapType(SmUnbendOrientTYPE) ;
extern SmUnbendOrientTYPE MapType(SmBendOrientTYPE) ;

/*******************************************************************//**
PURPOSE: The SmSpaceBend object directs the bending of a sheet metal Brep model.

NOTES:
 language: (same shape as the SmUnbendSpace operator with an inverse shape change)
  The Bend description:
                    PreBend line                          -+-------------+-------> BendAxis
                            +----*----*------+             | BendRadius  |
                                 |____|<-BendInterval      v             | NeutralRadius in BendDirection
                                                         +-*-------------|---------+
                              +          +               | ___ ___ ___ __v ___ ___ |
                               \<------>/<--BendAngle    |                         |
                 AfterBend line \      /                 |                         |
                                 *-__-*                  +-------------------------+
                                                         PreBend Block cross section
                                                      at center of bend perp to the page

    BendAngle        = Extent of the bend from -180 to +180 degrees. Bend a straight line segment in the middle.
                       The angle between the two ends of the line is the BendAngle.
    BendRadius       = distance between the BendAxis and the sheet metal's inside surface.
    NeutralRadius    = distance between the BendAxis and the NeutralAxis.

  The Bend orientation:
    BendAxisPoint    = the origin of the BendCenterLine = BendAxisPoint + s * BendAxis
    BendAxis         = bend's geometric center line located parallel and above the sheet metal inside surface.
                        (The bend's Z sxis - in the Neutral plane)
    BendDirection    = a vector from the NeutralPlane to BendAxis specifying the inside part of the bend.
                        (The bend's X axis - perpendicular to both the BendAxis and the NeutralPlane)
    BendBinormal     = defined as BendAxis * BendDirection
                        (The bend's Y axis - in the Neutral Plane)
    BendInterval     = the length of the BendSection which is sized and positioned by the segment
                       BendSegment(s) = BendAxisPoint + BendInterval.Evaluate(s) * NeutralAxis
                          for s values from 0.0 to 1.0.
    BendSegment      = Portion of the BendAxis within the BendRegion
  The Bend region:
    BendRegion       = the region in which the bend is applied. The BendRegion is a rectilinear
                       solid aligned with the NeutralPlane and the Neutral Axis.
                         height = Thickness with the KFactor determining where the
                                    BendRegion sits in relation to the NeutralPlane.
                         width  = NeutralRadius * BendAngleRad,
                                    which is centered on the NeutralAxis.
                         length = length of the NeutralAxis vector and its length is positioned
                                   by the segment = BendAxisPoint + BendInteral.Evaluate(s) * NeutralAxis
                                   for s values from 0.0 to 1.0.
                       Only those brep geometry pieces completely within the BendRegion are bent.
  The Bend NeutralPlane geometry:
    NeutralPoint     = BendAxisPoint + BendRadius * BendDirection
    NeutralAxis      = line parallel to BendAxis in the NeutralPlane.
                       NeutralAxis(s) = BendAxisPoint + s * BendAxis
    NeutralPlane     = plane within the sheet metal that maintains its length through the bend operation.
                       NeutralPlane(u,s) = BendAxisPoint + u * BendBinormal + s * BendAxis

  The Sheet metal:
    KFactor          = location of the neutral plane represented as the normalized distance (ranging from 0.0 to 1.0)
                          between the sheet metal inside and outside surfaces.
    Thickness        = distance between the sheet metal inside and outside surfaces.

  The Part final position options: Ways to specify the after bend orientation of the Brep.
    SM_BO_FIXED_START_BENDING = The portion of the Brep before the bend
                                (in the direction of Cross(BendDirection,NeutralAxis))
                                remains fixed throughout the bend and the Brep portion
                                after the bend is rotated by the BendAngle.
    SM_BO_CENTER_BENDING      = The portions of the Brep before and after the bend are both
                                rotated about the bend by one half the BendAngle.
    SM_BO_FIXED_END_BENDING   = The portion of the Brep after the bend
                                (in the direction opposite of Cross(BendDirection,NeutralAxis))
                                remains fixed throughout the bend and the Brep portion
                                before the bend is rotated by the BendAngle.
 Notes and Rules:
  1 : NeutralRadius = BendInsideRadius + KFactor * SheetThickness
  2 : NeutralRadius >= kFactor * SheetThickness
  3 : It's assumed the sheet metal inside and outside surfaces and the bend's BendAxis, NeutralAxis,
        and NeutralPlane are all parallel to one another.
  4 : The Brep's before bend geometry is all the geometry which connects to the before bend side which
       is in the -BendBinormal direction).
  5 : Similarly, the Brep's after bend geometry connects to the after bend side which is in the
       +BendBinormal direction).
  6 : Pieces of geometry COMPLETELY within the BendRegion are bent.
  7 : Pieces of geometry COMPLETELY within the Before BendRegion are rotated and moved.
  8 : Pieces of geometry COMPLETELY within the After BendRegion are rotated and moved.
  9 : No check is made for geometry moved to self-intersecting positions.

 NOTE: The SmSpaceBend and SmSpaceUnbend classes duplicate most of their methods (yuk).
       If you debug one class - duplicate the debug change in the other class.

***********************************************************************/
class SM_EXPORT SmSpaceBend : public SmObject
{
protected:
  // the target
  SmBrep            * m_pBrep ;               // Brep to bend (containing Context used for new obj construction)

  // the bend
  double              m_dBendAngDeg ;         // Extent of the bend in degrees from -180 to + 180.
                                              //    Often just [30, 45, 60, 90, 120, 135, 150, 180]
  double              m_dNeutralRadius ;      // Bend radius measured at the neutral axis, not the inside face of the sheet
                                              //  NeutralRadius = BendRadius + KFactor * SheetThickness

  // bend orientation
  SmPoint3d           m_vBendAxisPoint ;      // A point on BendAxis.
                                              //   BendAxisPoint = BendPoint + NeutralRadius * BendDir
  SmVector3d          m_vBendAxis ;           // unit vector pointing along the bend axis.
                                              //  (Bend Z Axis)
  SmVector3d          m_vBendDirection ;      // unit vector pointing from the BendAxisPoint to the center of the NeutralPlane.
                                              //  (Bend X Axis - perpendicular to both BendAxis and NeutralPlane)

  // bend linear extent
  SmExtent1d          m_vBendInterval ;       // Subinterval of the Line(s) = BendAxisPoint + s*BendAxis
                                              //   which is bent.  This allows subsections of a Brep to
                                              //     be bent while ignoring other pieces.
                                              // IsInit(m_vBendInterval) == TRUE, means the interval is infinite - bend
                                              //     all pieces of the Brep on bend plane
  // the sheet metal
  double              m_dSheetThickness ;     // Sheet metal thickness.
  double              m_dKFactor ;            // Locates the neutral axis between the inside and outside sheet surfaces.
                                              //   0.0 = neutral plane is the inside bend surface.
                                              //   1.0 = neutral plane is the outside bend surface.
                                              //   0.5 = neutral plane is midway between the inside and outside bend surfaces.
                                              //  default:[0.44]

  // tolerance when not using exact geometry
  double              m_dApproxTol ;          // Max distance between approx BSpline and exact bend shapes
                                              // default:[1.0e-5] - for now SmBrep::ApproximateWithBSplines() has trouble hitting
                                              //                    this tolerance - use 1.0e-4 or larger.

  // options for final part orientation
  SmBendOrientTYPE    m_eBendOrient ;         // selects the final orientation of the bent part
                                              // SM_EB_FIXED_START_BENDING = topology in front of bend is fixed - after topology is rotated into position
                                              // SM_EB_CENTER_BENDING      = topology on either side of bend is rotated in equal and opposite directions
                                              // SM_EB_FIXED_END_BENDING   = topology after bend is fixed - before topology is rotated into position

  // chached data
  SmBrep            * m_pBendDomain ;         // A Brep representing the region of space being bent. It's the geometric region
                                              //   selecting all m_pBrep geometry to bend. m_pBendDomain is not a part of m_pBrep.
                                              // default:[NULL]
  SmBrep            * m_pStartBendBoundary ;  // A Brep representing the sheet start-bend boundary of m_pBendDomain
  SmBrep            * m_pStopBendBoundary ;   // A Brep representing the sheet stop-bend boundary of m_pBendDomain

  double              m_dBrepSize ;           // used when Bend Axis is unbounded (m_vBendInterval is set to Init() values)
                                              // default:[SM_UNDEF_DOUBLE]
public:
  // constructor
  SmSpaceBend
  (
    SmBrep           & rBrep,               ///< [in ]: target Brep to bend (containing Context used for new obj construction)                                       <br>
    double             dBendAngDeg,         ///< [in ]: bend angle in degrees                                                                                        <br>
    double             dNeutralRadius,      ///< [in ]: bend radius at the neutral plane                                                                             <br>
    const SmPoint3d   & rBendAxisPoint,     ///< [in ]: BendAxisPoint   of bend Centerline = BendAxisPoint + s * BendAxisUnitVec.                                    <br>
    const SmVector3d  & rBendAxis,          ///< [in ]: BendAxisUnitVec of bend Centerline = BendAxisPoint + s * BendAxisUnitVec,                                    <br>
                                            ///<      : "the bend's Z Axis."                                                                                         <br>
    const SmVector3d  & rBendDirection,     ///< [in ]: unit-vector from BendAxisPoint towards NeutralPlane center,                                                  <br>
                                            ///<      : perpendicular to the BendAxis and perpendicular to the neutral plane.                                        <br>
                                            ///<      : bend's X Axis.                                                                                               <br>
    const SmExtent1d  & rBendInterval,      ///< [in ]: active segment of bend Centerline = BendAxisPoint + s * BendAxis.                                            <br>
                                            ///<      : use BendInterval.Init() for an unbounded bend line.                                                          <br>
    double             dSheetThickness,     ///< [in ]: SheetThickness of NeutralRadius = InsideRadius + KFactor * SheetThickness                                    <br>
    double             dKFactor,            ///< [in ]: KFactor        of NeutralRadius = InsideRadius + KFactor * SheetThickness                                    <br>
                                            ///<      : neutral plane normalized param between inside and outside sheet surfaces, commonly [.44]                     <br>
    SmBendOrientTYPE   eBendOrient,         ///<      : selects the final orientation of the bent part. oneof:                                                       <br>
                                            ///<      : SM_BO_FIXED_BEFORE_BENDING = topology in front of bend is fixed - after topology is rotated into position    <br>
                                            ///<      : SM_BO_CENTER_BENDING       = topology on either side of bend is rotated in equal and opposite directions     <br>
                                            ///<      : SM_BO_FIXED_AFTER_BENDING  = topology after bend is fixed - before topology is rotated into position         <br>
    double             dApproxTol = 1.0e-5  ///< [in ]: when not using exact geometry: max distance between approx BSpline and exact bend shapes                     <br>
                                            ///<      : default:[1.0e-5] - for now SmBrep::ApproximateWithBSplines() has trouble hitting                             <br>
                                            ///<      :                    this tolerance - currently bounded by 1.0e-4 or larger.                                   <br>
  )
    : SmObject( rBrep ),
    m_pBrep( &rBrep ),
    m_dBendAngDeg( dBendAngDeg ),
    m_dNeutralRadius( dNeutralRadius ),
    m_vBendAxisPoint( rBendAxisPoint ),
    m_vBendAxis( rBendAxis ),
    m_vBendDirection( rBendDirection ),
    m_vBendInterval( rBendInterval ),
    m_dSheetThickness( dSheetThickness ),
    m_dKFactor( dKFactor ),
    m_dApproxTol( dApproxTol > 1.0e-4 ? dApproxTol : 1.0e-4 ),
    m_eBendOrient( eBendOrient ),
    m_pBendDomain( NULL ),
    m_pStartBendBoundary( NULL ),
    m_pStopBendBoundary( NULL ),
    m_dBrepSize( SM_UNDEF_DOUBLE )

  { // work ahead - build needed cached BendDomain = region of space being bent
    GetBendDomain();
  }

  // destructor
  virtual ~SmSpaceBend()
  {
    if(m_pBendDomain != NULL) { delete m_pBendDomain; m_pBendDomain = NULL; }
    if(m_pStartBendBoundary != NULL) { delete m_pStartBendBoundary; m_pStartBendBoundary = NULL; }
    if(m_pStopBendBoundary != NULL) { delete m_pStopBendBoundary; m_pStopBendBoundary = NULL; }
  }

  // create SpaceBend inverse to a given SpaceUnbend object
  static SmStatus CreateBendInverse
  (
    const SmSpaceUnbend & crSpaceUnbend, ///< [in ]: the SpaceUnbend to invert                               <br>
    SmBrep              & rBrep,         ///< [in ]: Brep target for new rpSaceBend                          <br>
    SmSpaceBend        *& rpSpaceBend    ///< [out]: the new SpaceBend inverse to the input SpaceUnbend.     <br>
  );

  // Create on the heap and return a SpaceBend defining BendVolume
  SmBendVolume * CreateBendVolume(const SmContext &crContext) const ;

  // execute bend on m_pBrep, m_pBrep is modified in place
  SmStatus DoSpaceBend(SmBoolean bDoApproximations = TRUE) ;

  // internal assist methods

  // common derived geometric properties
  //   BrepSize         = MaxDimension of Brep BoundingBox
  //   InsideRadius     = BendRadius at sheet metal inside surface
  //   OutsideRadius    = BendRadius at sheet metal outside surface
  //   BendSegment      = The active portion along the bend NeutralLine
  //                        along which m_pBrep geometry will be bent.  m_pBrep geometry
  //                        on the BendLine outside the BendSegment will not be bent.
  //   BendSegmentStart = Bend NeutralAxis segment start point
  //   BendSegmentStop  = Bend NeutralAxis segment end point
  //   BendLength       = Bend NeutralAxis segment length
  //   PreBendRegion    = rectilinear region centered on the NeutralAxis segment
  //                        containing all geometyry to bend in the preBend Brep
  //   PreBendCorner    = PreBendRegion start corner, a point on the bend's outside surface
  //                      offset from the neutralAxis segment start point by (NeutralRadius * BendAngRad)

  SmBrep* GetBrep() const { return( m_pBrep ) ; }

  double             GetBrepSize()
  {
    if(m_dBrepSize == SM_UNDEF_DOUBLE)
    {
      SmExtent3d sSize;
      m_pBrep->CalculateBoundingBox( sSize, TRUE );
      m_dBrepSize = sSize.GetMaxDimension();
    }
    return(m_dBrepSize);
  }

  double             GetApproxTol()          const { return( m_dApproxTol ) ; }
  double             GetBendAngDeg()         const { return( m_dBendAngDeg ) ; }
  double             GetInsideRadius()       const { return( m_dNeutralRadius - m_dKFactor * m_dSheetThickness ) ; }
  double             GetOutsideRadius()      const { return( m_dNeutralRadius + (1.0 - m_dKFactor) * m_dSheetThickness ) ; }
  double             GetNeutralRadius()      const { return( m_dNeutralRadius ) ; }
  SmExtent1d         GetBendInterval()       const { return( m_vBendInterval ) ; }
  double             GetSheetThickness()     const { return( m_dSheetThickness ) ; }
  double             GetKFactor()            const { return( m_dKFactor ) ; }
  SmBendOrientTYPE   GetBendOrient()         const { return( m_eBendOrient ) ; }

  SmVector3d GetBendDirection()      const { return( m_vBendDirection ) ; }                 // The Bend X Axis
  SmVector3d GetBendBinormal()       const { return( m_vBendAxis * m_vBendDirection ) ; }   // The Bend Y Axis
  SmVector3d GetBendAxis()           const { return( m_vBendAxis ) ; }                      // The Bend Z Axis
  SmPoint3d  GetBendAxisPoint()      const { return( m_vBendAxisPoint ) ; }                 // The Bend origin

  // points on the PreBend Neutral line in plane perp to BendOrigin
  SmPoint3d  GetNeutralBegPoint()    const { return( m_vBendAxisPoint + m_dNeutralRadius * m_vBendDirection - GetPreBendWidth()/2.0 * GetBendBinormal() ) ; }
  SmPoint3d  GetNeutralMidPoint()    const { return( m_vBendAxisPoint + m_dNeutralRadius * m_vBendDirection ) ; }
  SmPoint3d  GetNeutralEndPoint()    const { return( m_vBendAxisPoint + m_dNeutralRadius * m_vBendDirection + GetPreBendWidth()/2.0 * GetBendBinormal() ) ; }

  // points on the PreBend Neutral line in plane perp to BendInterval begin
  SmPoint3d  GetPreBendBeginCorner() const
  {
    return(m_vBendAxisPoint
           + GetInsideRadius()             * m_vBendDirection
           - GetPreBendWidth() / 2.0         * (m_vBendAxis * m_vBendDirection)
           + m_vBendInterval.Evaluate( 0.0 ) * m_vBendAxis);
  }

  SmPoint3d  GetPreBendEndCorner()   const
  {
    return(m_vBendAxisPoint
           + GetInsideRadius()             * m_vBendDirection
           + GetPreBendWidth() / 2.0         * (m_vBendAxis * m_vBendDirection)
           + m_vBendInterval.Evaluate( 0.0 ) * m_vBendAxis);
  }

  // points on the Bend Axis
  SmPoint3d  GetBendSegmentStart()   const
  {
    if(m_vBendInterval.IsInit()) { return(m_vBendAxisPoint - ((SmSpaceBend*)this)->GetBrepSize() * m_vBendAxis); }
    else { return(m_vBendAxisPoint + m_vBendInterval.Evaluate( 0.0 ) * m_vBendAxis); }
  }

  SmPoint3d  GetBendSegmentStop()    const
  {
    if(m_vBendInterval.IsInit()) { return(m_vBendAxisPoint + ((SmSpaceBend*)this)->GetBrepSize() * m_vBendAxis); }
    else { return(m_vBendAxisPoint + m_vBendInterval.Evaluate( 1.0 ) * m_vBendAxis); }
  }

  double     GetBendSegmentLength()  const
  {
    if(m_vBendInterval.IsInit()) { return(2.0 * ((SmSpaceBend*)this)->GetBrepSize()); }
    else { return(m_vBendInterval.GetLength()); }
  }


  // Bend Neutral line width
  double     GetPreBendWidth()       const { return( m_dNeutralRadius * SM_DEG2RAD(m_dBendAngDeg) ) ; }

  // internal assist methods

  // Get (build when needed) preBendRegion represented as a Brep
  SmBrep * GetBendDomain() const ;  // does builds internally stored Bend Brep shapes
  SmBrep * GetStartBendBoundary() const { GetBendDomain() ; return(m_pStartBendBoundary) ; }
  SmBrep * GetStopBendBoundary()  const { GetBendDomain() ; return(m_pStopBendBoundary) ; }

  SmAxis2Placement GetBeforeBendTransform(SmBendOrientTYPE eOptOrientType=SM_BO_ORIENT_UNKNOWN) const ;
  SmAxis2Placement GetInsideBendTransform(SmBendOrientTYPE eOptOrientType=SM_BO_ORIENT_UNKNOWN) const ;
  SmAxis2Placement GetAfterBendTransform (SmBendOrientTYPE eOptOrientType=SM_BO_ORIENT_UNKNOWN) const ;

  // internal unit-test for BendTransforms
  SmBoolean TestBendTransforms() const ;

  // apply rigid body translate and move transformation to listed vertices, edges, and faces
  SmStatus TransformTopology
  (
    const SmAxis2Placement    & crRotateNMove,  ///< [in ]: Rigid body transform to apply                      <br>
    SmBrep                    & rBrep,          ///< [in ]: Brep containing all topology to be transformed     <br>
    const SmTArray<SmFace*>   & crFaces,        ///< [in ]: faces to transform                                 <br>
    const SmTArray<SmEdge*>   & crEdges,        ///< [in ]: edges to transform                                 <br>
    const SmTArray<SmVertex*> & crVertices      ///< [in ]: vertices to transform                              <br>
  );

  // apply Bend mapping to listed vertices, edges, and faces
  SmStatus BendTopology
  (
    const SmBendVolume        & crBendVolume,   ///< [in ]: Bend mapping to apply                             <br>
    SmBrep                    & rBrep,          ///< [in ]: Brep containing all topology to be transformed    <br>
    const SmTArray<SmFace*>   & crFaces,        ///< [in ]: faces to transform                                <br>
    const SmTArray<SmEdge*>   & crEdges,        ///< [in ]: edges to transform                                <br>
    const SmTArray<SmVertex*> & crVertices      ///< [in ]: vertices to transform                             <br>
  );

  // a specialized version of SmBrep::ReplaceSurface to save some copies only for SmSpaceBend internal use
  SmStatus ReplaceSurface
  (
    SmFace            * pFace,         ///< [in ]: Target Face to get new Surface                     <br>
    SmSrfInVolume     * pNewSurface    ///< [in ]: Target Surface to replace current pFace->Surface   <br>
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
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  SmDisplayList   * Draw
  (
    SmBoolean       bDrawBrep=FALSE,          ///< [in ]: TRUE = Draw target m_pBrep, default:[FALSE]                                     <br>
    SmBoolean       bAddToUIPickList=FALSE,   ///< NotUsed: [in ]: TRUE = Add target m_pBrep to UI pick interface for debugging, default:[FALSE]   <br>
    SmBoolean       bDrawPreBendRegion=FALSE, ///< [in ]: TRUE = Draw preBendRegion, default:[FALSE]                                      <br>
    SmGfxArraySet * pOptGfxSet=NULL           ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. <br>
  ) const;

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmSpaceBend, SmObject, SmSpaceBend_TYPE) ;

} ; // end class SmSpaceBend

#endif // __SMSPACEBEND_H__


