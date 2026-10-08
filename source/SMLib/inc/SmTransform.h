// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmTransform.h
* PURPOSE: Header file for Axis2Placement object
**********************************************************************/

#ifndef __SMTRANSFORM_H__
#define __SMTRANSFORM_H__

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#include <SmVolume.h> 

class SmDisplayList ;
class SmAxis2Placment ;

/*******************************************************************//**
PURPOSE: This object represents a general linear transform from
    a paramUVW coordinate system to an projXYZ coordinate system.

NOTES: SmTransform is implemented as the standard 4x4 transform matrix
       (like OpenGL) with the transform mapping defined
       as a premultiplication so that this class acts just like
       SMLib's SmAxis2Placment class (different than OpenGL).

    1. The forward transform from paramUVW space to projXYZ space is defined as: 

       [ProjPoint.x ProjPoint.y ProjPoint.z 1] = [ParamPoint.u ParamPoint.v ParamPoint.w 1] * T
         
          where T = 4x4 matrix

    2. Transforms concatenate.  Projecting a point through a sequence
       of transforms is the same as projecting a point through a
       concatenation of the transform squence.

           Example: When Tconcat = T1 * T2 * T3 ;

                    Pproj1 = Pparam * Tconcat ;
                    Pproj2 = (((Pparam * T1) * T2) * T3) ;
                and Pproj1 == Pproj2.

       Typical uses of this class are 
         a. create a unity Transform, T
         b. Concatenate T with any squence of Rotate, Translate, Mirror, Scale and project operations
         c1. Apply the concatenated transform, T, to move points from the original paramUVW space to the final projXYZ space.
         c2. Apply the concatenated transform, T, to orient a Volume's ParamSpace within a target InSpace for space deformation.

     3. Matrix forms
       3a. Unity Transform.          ProjPt = ParamPt
             Tunity  = [ 1  0  0  0]    
                       [ 0  1  0  0]    In words: Applying Tunity to an object is like multiplying by 1,
                       [ 0  0  1  0]              the tranformation returns the object exactly as it was input.  
                       [ 0  0  0  1]

       3b. Rotation Transform.       Rotate points between a pair of coordinate systems sharing the same origin.
             Trotate = [X0 X1 X2  0]    Where   : X, Y, Z are the orthogonal unit-vectors of the projXYZ coordinate system.
                       [Y0 Y1 Y2  0]    In words: Applying Trotate to an object rotates the object about the origin
                       [Z0 Z1 Z2  0]              so that its paramX, paramY, and paramZ axes becomes aligned with 
                       [ 0  0  0  1]              the X, Y, and Z axes embedded within the transform matrix.
                                  
       3c. Translation Transform.    Move points between a pair of parallel coordinate systems with different origins.
             Ttrans  = [ 1  0  0  0]    Where   : V = Translate vectors 
                       [ 0  1  0  0]    In words: Applying Ttrans to an object moves it by the translation vector, V,
                       [ 0  0  1  0]              without rotating the object.
                       [Vx Vy Vz  1]

       3d. Scaling Transform.        Scale each coordinate of an paramUVW point by a scale factor about the origin. 
             Tscale  = [Sx  0  0  0]    
                       [ 0 Sy  0  0]    In words: Applying Tscale to an object scales its size about the origin
                       [ 0  0 Sz  0]              without rotating the object. Uniform scaling happens when Sx = Sy = Sz. 
                       [ 0  0  0  1]              NonUniform scaling shears parts, they won't be rotated but the angles
                                                   formed at the corners of an object will be changed.

       3e. Mirror Transform.         Mirror points about a plane.
             Tmirror =     
        [1-2NxNx  -2NxNy  -2NxNz  0]    Where   : D = PlanePt.Dot(PlaneNormal) ;                        
        [ -2NyNy 1-2NyNy  -2NyNz  0]              N = PlaneNormal              
        [ -2NzNx  -2NzNy 1-2NzNz  0]    In words: Applying Tmirror to an object mirrors the object about the given plane.
        [  2D*Nx   2D*Ny   2D*Nz  1]    Method  : Pproj = Pparam - 2.0 * (Pparam-Pplane).Dot(Pnormal) * Pnormal                                                       
                                                  Pproj = Pparam + 2.0 * Pplane.Dot(Pnormal)* Pnormal - 2.0 * Pparam * [Pnormal o Pnormal]
                                                  Pproj = [PparamU PparamV PparamW 1 ] [1-2NxNx  -2NxNy  -2NxNz  0]                          
                                                                                       [ -2NyNy 1-2NyNy  -2NyNz  0]                          
                                                                                       [ -2NzNx  -2NzNy 1-2NzNz  0]                          
                                                                                       [  2D*Nx   2D*Ny   2D*Nz  1]  
       3f. Project Transform.       Project points to a plane.  
             Tproject =     
           [1-NxNx  -NxNy  -NxNz  0]    Where   : D = PlanePt.Dot(PlaneNormal) ;                        
           [ -NyNy 1-NyNy  -NyNz  0]              N = PlaneNormal              
           [ -NzNx  -NzNy 1-NzNz  0]    In words: Applying Tproject to an object drops points down the PlaneNormal to a Plane.
           [  D*Nx   D*Ny   D*Nz  1]    
                                        
                                        
     4. Method overview:

       Use the following methods to build a desired transformation as a sequence of simple pure transformations.

         // constructors and init
         Empty  Constructor()             alloc ThisT = unity transform ;
         Orient Constructor(Orig,X,Y,Y)   alloc ThisT = Transform from ParamSpace to ProjSpace with Orig, and XYZ axes
         FromTo Constructor(FromOrig,
                            FromX,FromY,
                            ToO,ToX,ToY)  alloc ThisT = Transform from FromXYZ to ToXYZ coordinate system
         Init()                           set   ThisT = unity transform ; 

         // Concatentation operations
         Transform(T2) ;                    // build ThisT = ThisT * T2       = add Transform T2 to end of a transform sequence
         Translate(Vec) ;                   // build ThisT = ThisT * Ttrans   = add a Translate  to end of a transform sequence
         RotateAboutAxis(AngRad, Axis) ;    // build ThisT = ThisT * Trotate  = add a Rotate     to end of a transform sequence
         RotateAboutAxisAtPoint(AngRad,                                                      
                                Pt, Axis) ; // build ThisT = ThisT * Trotate  = add a Rotate     to end of a transform sequence
         Scale(ScaleVec) ;                  // build ThisT = ThisT * Tscale   = add a Scale      to end of a transform sequence
         Mirror(PlanePt, PlaneNormal) ;     // build ThisT = ThisT * Tmirror  = add a Mirror     to end of a transform sequence
         Project(PlanePt, PlaneNormal) ;    // build ThisT = ThisT * TProject = add a Project    to end of a transform sequence

       Use the following to map points and vectors between paramUVW and projXYZ spaces.

         TransformPoint  (ParamPt,  ProjPt) ;    // Map ParamPoint  from paramUVW  to projXYZ space
         TransformVector (ParamVec, ProjVec) ;   // Map ParamVec    from paramUVW  to projXYZ space
         InvTransformPoint (ProjPt, ParamPt) ;     // Map ProjPoint from projXYZ to paramUVW space (inverse of TransformPoint)
         InvTransformVector(ProjPt, ParamPt) ;     // Map ProjVec   from projXYZ to paramUVW space (inverse of TransformVector)

       Many More methods are defined below.
***********************************************************************/
class SM_EXPORT SmTransform : public SmVolume
{
 protected:
   double     m_adT[4][4] = {0.0} ;  // the 4x4 transform matrix in POut = [Pparam 1] * T
                                     //
                                     // for I/O sake, the parts of the transform matrix are named as:
                                     //  [X0 X1 X2 | 0] = [ ToXAxis |     ]   where  ToXAxis = [X0 X1 X2]
                                     //  [Y0 Y1 Y2 | 0]   [ ToYAxis | Ext ]          ToYAxis   [Y0 Y1 Y2]
                                     //  [Z0 Z1 Z2 | 0]   [ ToZAxis |     ]          ToZAxis   [Z0 Z1 Z2]
                                     //  [D0 D1 D2 | 1]   [ ToDisp  |     ]          ToDisp    [D0 D1 D2]
                                     //                                              Ext     = [0 0 0 1]T

  // Inherited SmVolume members
  //      SmObject    * m_pOwner ;            // not used
  //       SmVolume    *m_pNextMap ;          // linked list of concatenated SmVolume transformations
  //       SmTransform *m_pOrientMap ;        // for space deformation to orient this mapping's ParamSpace to a target InSpace.
  //       SmTransform *m_pInvOrientMap ;     // cached m_pOrientMap Inverse.
  //       ULONG        m_lNextOwnerFlag ;    // owner state for m_pNextMap, needed for destructor
  //       ULONG        m_lOrientOwnerFlag ;  // owner state for m_pOrientMap, needed for destructor

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
//     //   TrimInSpaceBoundingBoxSimple()          // opt implementation: Trim InSpaceBBox to map within the NaturalParamSpaceDomain, default pTrimBox = pGivenBox
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
  //        IsPointInParamDomainSimple()         // opt: rtn TRUE if ParamPoint is in map's NaturalParamDomain. default:[if bnded(GetNaturalDomain().Contains(ParamPt)) else TRUE], implement if:[ParamDomain is bounded not rectilinear]                                   
  //        IsPointInProjDomainSimple()          // opt: rtn TRUE if ProjPoint drops within map's NaturalParamDomain. default:[if bnded(InvEvaluatePointSimple()) else TRUE], implement if:[ParamDomain is bounded not rectilinear]
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
  // orient constructor = map Current paramUVW space to SmTransform coords 
   SmTransform
   (
     const SmContext  * cpContext,        ///< [in ]: context for this new object                                            <br>
     const SmPoint3d  & crToOrigin,       ///< [in ]: ProjSpace origin = ToDisp                                              <br>      
     const SmVector3d & crToXAxis,        ///< [in ]: ProjSpace X Axis                                                       <br>
     const SmVector3d & crToYAxis,        ///< [in ]: ProjSpace Y Axis                                                       <br>
     const SmVector3d * cpToZAxis = NULL  ///< [in ]: opt ProjSpace Z Axis, NULL = let ProjSpaceZ = cross(ToXAxis, ToYAxis)  <br>
   )     
     : SmVolume( cpContext )
   {
     SetCanonical( crToOrigin, crToXAxis, crToYAxis, cpToZAxis );
   }

  // orient constructor from doubles for convenience
   SmTransform
   (
     const SmContext * cpContext,                                                   ///< [in ]: context for this new object                                            <br>
     double  dToOriginX, double  dToOriginY, double  dToOriginZ,                    ///< [in ]: ProjSpace origin = ToDisp                                              <br>     
     double  dToXAxisX, double  dToXAxisY, double  dToXAxisZ,                       ///< [in ]: ProjSpace X Axis                                                       <br>
     double  dToYAxisX, double  dToYAxisY, double  dToYAxisZ,                       ///< [in ]: ProjSpace Y Axis                                                       <br>
     double *pToZAxisX = NULL, double *pToZAxisY = NULL, double *pToZAxisZ = NULL   ///< [in ]: opt ProjSpace Z Axis, NULL = let ProjSpaceZ = cross(ToXAxis, ToYAxis)  <br>
   )
     : SmVolume( cpContext )
   {
     SmVector3d *pToZAxis = NULL, sToZAxis;
     if(pToZAxisX && pToZAxisY && pToZAxisZ)
     {
       sToZAxis.Set( *pToZAxisX, *pToZAxisY, *pToZAxisZ );
       pToZAxis = &sToZAxis;
     }

     SetCanonical( SmPoint3d( dToOriginX, dToOriginY, dToOriginZ ),
                  SmVector3d( dToXAxisX, dToXAxisY, dToXAxisZ ),
                  SmVector3d( dToYAxisX, dToYAxisY, dToYAxisZ ),
                  pToZAxis );
   }

  // from/to constructor = map FromXYZ coords to ToXYZ coords
   SmTransform
   (
     const SmContext  * cpContext,       ///< [in ]: context for this new object       <br>
     const SmPoint3d  & crFromOrigin,    ///< [in ]: FromSpace origin                  <br>
     const SmVector3d & crFromXAxis,     ///< [in ]: FromSpace X Axis                  <br>
     const SmVector3d & crFromYAxis,     ///< [in ]: FromSpace Y Axis                  <br>
     const SmVector3d & crFromZAxis,     ///< [in ]: FromSpace Z Axis                  <br>
     const SmVector3d & crToOrigin,      ///< [in ]: ProjSpace origin                  <br>
     const SmVector3d & crToXAxis,       ///< [in ]: ProjSpace X Axis                  <br>
     const SmVector3d & crToYAxis,       ///< [in ]: ProjSpace Y Axis                  <br>
     const SmVector3d & crToZAxis )      ///< [in ]: ProjSpace Z Axis                  <br>
     : SmVolume( cpContext )
   {
     SmTransform sFrom( cpContext, crFromOrigin, crFromXAxis, crFromYAxis, &crFromZAxis );
     SmTransform sTo( cpContext, crToOrigin, crToXAxis, crToYAxis, &crToZAxis );
     sFrom.InvertSimple();
     this->Init();
     this->ConcatTransform( sFrom );
     this->ConcatTransform( sTo );
   }

  // matrix constructor = set 4x4 matrix directly
   SmTransform
   (
     const SmContext  * cpContext,      ///< [in ]: context for this new object
     const double dT[4][4] ) : SmVolume( cpContext )
   {
     m_adT[0][0] = dT[0][0]; m_adT[0][1] = dT[0][1]; m_adT[0][2] = dT[0][2]; m_adT[0][3] = dT[0][3];
     m_adT[1][0] = dT[1][0]; m_adT[1][1] = dT[1][1]; m_adT[1][2] = dT[1][2]; m_adT[1][3] = dT[1][3];
     m_adT[2][0] = dT[2][0]; m_adT[2][1] = dT[2][1]; m_adT[2][2] = dT[2][2]; m_adT[2][3] = dT[2][3];
     m_adT[3][0] = dT[3][0]; m_adT[3][1] = dT[3][1]; m_adT[3][2] = dT[3][2]; m_adT[3][3] = dT[3][3];
   }

  // empty constructor = identity transform for I/O
   SmTransform( const SmContext *cpContext = NULL )
   {
     if(cpContext) { SetContext( cpContext ); }
     Init();
   }
  
  // Copy constructors
  // SmTransform(const SmTransform & crSourceVolume)  ///< [in ]: SourceVolume to copy (copies all compounding volumes)
  //                                                  { SmTransform::SmTransform(crSourceVolume, FALSE) ; }

  SmTransform
  (
    const SmTransform &crFromObject,        ///< [in ]: SourceVolume to copy                                         <br>
    SmBoolean          bSimpleMapOnly=FALSE ///< [in ]: TRUE = Copy this Volume omitting any compounding volumes     <br>
                                            ///<      : FALSE= Copy this Volumes with any compounding volumes        <br>
  );

  // make an exact copy of any volume with or without any compounding volumes
  virtual SmStatus Copy
  (
    const SmContext & crContext,               ///< [in ]: context for new object construction                        <br>
    SmVolume       *& rpNewVolume,             ///< [out]: The copied Volume                                          <br>
    SmBoolean         bSimpleMapOnly = FALSE   ///< [in ]: TRUE = Copy this Volume omitting any compounding volumes   <br>
                                               ///<      : FALSE= Copy this Volumes with any compounding volumes      <br>
  ) const
  {
    rpNewVolume = new (crContext) SmTransform( *this, bSimpleMapOnly ); NER( rpNewVolume );
    return SM_SUCCESS;
  }

  // assignment operator
  SmTransform & operator=(const SmTransform &crTransform) ;

  // destructor
  virtual ~SmTransform() { }

  // equality operator
  virtual SmBoolean operator==(const SmVolume &crOther) const ;

  // Simple Data Access

  // Natural Domain in ParamSpace and InSpace - Mapping is defined for all ParamSpace and InSpace
  virtual SmExtent3d  GetNaturalParamDomain() const
  {
    return SmExtent3d( -SM_INFINITE_PARAMETER, -SM_INFINITE_PARAMETER, -SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER );
  }

  // Get ProjSpace coordinate vectors given in ParamSpace UVW coordinates
  SmPoint3d  & GetToOrigin()              const   { return (SmVector3d &)m_adT[3][0] ; }
  SmVector3d & GetToXAxis()               const   { return (SmVector3d &)m_adT[0][0] ; }
  SmVector3d & GetToYAxis()               const   { return (SmVector3d &)m_adT[1][0] ; }        
  SmVector3d & GetToZAxis()               const   { return (SmVector3d &)m_adT[2][0] ; } 

  SmVector3d & GetToAxis( ULONG iIndx )   const
  {
    switch(iIndx)
    {
      case 0: return (SmVector3d &)m_adT[0][0];
      case 1: return (SmVector3d &)m_adT[1][0];
      case 2: return (SmVector3d &)m_adT[2][0];
      default: SE_MSG( SM_ERR, _T( "SmTransform::GetToAxis given invalid index value" ) );  // error case
        return (SmVector3d &)m_adT[0][0];
    }
  }

  SmPoint3d  & GetToDisp()                const   { return GetToOrigin() ; }

  double     * operator[](ULONG lIndex)           { return m_adT[lIndex] ; }
                                                                                 
  // Set ProjSpace coordinate vectors given in ParamSpace UVW coordinates
  void InitExt()                                  { m_adT[0][3] = 0; m_adT[2][3] = 0; m_adT[1][3] = 0;  m_adT[3][3] = 1; }

  void SetToOrigin(const SmVector3d &rVec)        { m_adT[3][0] = rVec.x ; m_adT[3][1] = rVec.y ; m_adT[3][2] = rVec.z ; }

  void SetToXAxis (const SmVector3d &rVec)        { m_adT[0][0] = rVec.x ; m_adT[0][1] = rVec.y ; m_adT[0][2] = rVec.z ; }

  void SetToYAxis (const SmVector3d &rVec)        { m_adT[1][0] = rVec.x ; m_adT[1][1] = rVec.y ; m_adT[1][2] = rVec.z ; }

  void SetToZAxis (const SmVector3d &rVec)        { m_adT[2][0] = rVec.x ; m_adT[2][1] = rVec.y ; m_adT[2][2] = rVec.z ; }

  void SetToDisp  (const SmVector3d &rVec)        { SetToOrigin(rVec) ; }
                                       
  void SetToOrigin(double dX, double dY, double dZ) { m_adT[3][0] = dX ; m_adT[3][1] = dY ; m_adT[3][2] = dZ ; }

  void SetToXAxis (double dX, double dY, double dZ) { m_adT[0][0] = dX ; m_adT[0][1] = dY ; m_adT[0][2] = dZ ; }

  void SetToYAxis (double dX, double dY, double dZ) { m_adT[1][0] = dX ; m_adT[1][1] = dY ; m_adT[1][2] = dZ ; }

  void SetToZAxis (double dX, double dY, double dZ) { m_adT[2][0] = dX ; m_adT[2][1] = dY ; m_adT[2][2] = dZ ; }

  void SetToDisp  (double dX, double dY, double dZ) { SetToOrigin(dX, dY, dZ) ; }

  void GetMatrix( double adT[4][4] )
  {
    adT[0][0] = m_adT[0][0]; adT[0][1] = m_adT[0][1]; adT[0][2] = m_adT[0][2]; adT[0][3] = m_adT[0][3];
    adT[1][0] = m_adT[1][0]; adT[1][1] = m_adT[1][1]; adT[1][2] = m_adT[1][2]; adT[1][3] = m_adT[1][3];
    adT[2][0] = m_adT[2][0]; adT[2][1] = m_adT[2][1]; adT[2][2] = m_adT[2][2]; adT[2][3] = m_adT[2][3];
    adT[3][0] = m_adT[3][0]; adT[3][1] = m_adT[3][1]; adT[3][2] = m_adT[3][2]; adT[3][3] = m_adT[3][3];
  }
  
  void SetMatrix( double adT[4][4] )
  {
    m_adT[0][0] = adT[0][0]; m_adT[0][1] = adT[0][1]; m_adT[0][2] = adT[0][2]; m_adT[0][3] = adT[0][3];
    m_adT[1][0] = adT[1][0]; m_adT[1][1] = adT[1][1]; m_adT[1][2] = adT[1][2]; m_adT[1][3] = adT[1][3];
    m_adT[2][0] = adT[2][0]; m_adT[2][1] = adT[2][1]; m_adT[2][2] = adT[2][2]; m_adT[2][3] = adT[2][3];
    m_adT[3][0] = adT[3][0]; m_adT[3][1] = adT[3][1]; m_adT[3][2] = adT[3][2]; m_adT[3][3] = adT[3][3];
  }
  
  // set transform = identity tranformation
  void     Init()
  {
    SetToXAxis( 1, 0, 0 );
    SetToYAxis( 0, 1, 0 );
    SetToZAxis( 0, 0, 1 );
    SetToDisp( 0, 0, 0 );
    InitExt();
  }

  static SmStatus CreateCanonical
  (
    const SmContext  & crContext,     ///< [in ]: context for new object construction                                       <br>       
    const SmVector3d & crToOrigin,    ///< [in ]: ProjSpace origin                                                          <br>
    const SmVector3d & crToXAxis,     ///< [in ]: ProjSpace XAxis                                                           <br>
    const SmVector3d & crToYAxis,     ///< [in ]: ProjSpace YAxis                                                           <br>
    const SmVector3d * cpToZAxis,     ///< [in ]: opt ProjSpace ZAxis, NULL: let ProjSpaceZ = cross(ProjXAxis, ProjYAxis)   <br>
    SmTransform     *& rpNewVolume    ///< [out]: New SmBendVolume, NULL on input                                           <br>
  );

  void     SetCanonical
  (
    const SmVector3d & crToOrigin,    ///< [in ]: ProjSpace origin                                                          <br>
    const SmVector3d & crToXAxis,     ///< [in ]: ProjSpace XAxis                                                           <br>
    const SmVector3d & crToYAxis,     ///< [in ]: ProjSpace YAxis                                                           <br>
    const SmVector3d * cpToZAxis      ///< [in ]: opt ProjSpace ZAxis, NULL: let ProjSpaceZ = cross(ProjXAxis, ProjYAxis)   <br>
  ) ;   
                                                         

  void     GetCanonical
  (
    SmVector3d & rToDisp,             ///< [out]: ProjSpace displacement      <br>
    SmVector3d & rToXAxis,            ///< [out]: ProjSpace XAxis             <br>
    SmVector3d & rToYAxis,            ///< [out]: ProjSpace YAxis             <br>
    SmVector3d & rToZAxis             ///< [out]: ProjSpace ZAxis             <br>
  ) const;

  // forward evals: map from paramUVW space to projXYZ space
  void TransformPoint (const SmPoint3d  & crParamPoint, SmPoint3d  & rProjPoint) const;
  void TransformVector(const SmVector3d & crParamVec,   SmVector3d & rProjVec) const;

  // inverse evals: map projSpace XYZ coords back to paramSpace UVW coords
  //   note       : inverse mapping is expensive, it's cheaper to Invert the transform and use the Transform methods

  // rtn: SM_ERR when transform is not invertable
  SmStatus InvTransformPoint 
  ( 
    const SmPoint3d  & crProjPoint,      ///< [in ]: point existing in XYZ ProjSpace                <br>
    SmPoint3d  & rParamPoint             ///< [out]: point inversely transformed to UVW ParamSpace  <br>
  ) const;  

  // rtn: SM_ERR when transform is not invertable
  SmStatus InvTransformVector
  ( 
    const SmVector3d & crProjVec,        ///< [in ]: vector existing in XYZ ProjSpace                 <br>
    SmVector3d & rParamVec               ///< [out]: vector inversely transformed to UVW ParamSpace   <br>
  ) const;    

  // post-concatenate a general transform onto this transform                
  SmStatus          ConcatTransform           (const SmTransform & crTransform) ;

  // convenience concatenation methods - 
  //   note: use these methods to build a single transformation that's equivalent to a sequence of transformations
  //       e.g. pTransform->Init() ; 
  //            pTransform->Mirror() ; 
  //            pTransform->Translate() ;
  //            builds a single transformtion map that's equivalent to
  //              mirroring a point, followed by translating those mirrored points.
  // MirrorSimple                  = Mirror points about given plane
  // ScaleSimple                   = Scale point locations about a given point  
  // TranslateSimple               = translate points by given vector  
  // RotateAboutAxisSimple         = rotate points about an axis that runs through the origin  
  // RotateAboutAxisAtPointSimple  = rotate points about an axis that runs through a given point 
  // TransformSimple               = move points from current to new coordinate system specified by an SmAxis2Placement object  
  // ProjectToPlaneSimple                 = project all points down to a given plane  
  // InvertSimple                  = invert the current transformation that will map ProjPoints back to ParamPoints  

  // rtn: SM_ERR when PlaneNormal is zero length
  virtual SmStatus  MirrorSimple                 
  (
    const SmPoint3d   & crPlaneProjPt,     ///< [in ]: Point on plane       <br>
    const SmVector3d & crPlaneProjNormal   ///< [in ]: plane's normal       <br>
  ) ; 

  virtual SmStatus  ScaleSimple                  
  (
    const SmVector3d  & crScaleProjVec,         ///< [in ]: ScaleVec coordinates define xyz scaling factors.                    <br>
    const SmPoint3d  * cpOptProjCenter = NULL   ///< [in ]: Sole point at which the scaled location equals the input location   <br>
                                                ///<      : NULL=(0,0,0), default:[NULL]                                        <br>
  );                                           

  virtual SmStatus  TranslateSimple              (const SmVector3d  & crProjTranslate) ;

  virtual SmStatus  RotateAboutAxisSimple        
  (
    double dAngRad,                           ///< [in ]: rotation angle (radians)    <br>
    const SmVector3d & crRotateAxis           ///< [in ]: rotation axis               <br>
  ) ;

  virtual SmStatus  RotateAboutAxisAtPointSimple 
  (
    double dAngRad,                           ///< [in ]: rotation angle radians                 <br>
    const SmPoint3d  & crRotateOrigin,        ///< [in ]: ProjSpace Point on rotation axis       <br>
    const SmVector3d & crRotateAxis           ///< [in ]: ProjSpace direction of rotation axis   <br>
  ) ;

  virtual SmStatus  TransformSimple              
  (
    const SmAxis2Placement & crRotateNMove,       ///< [in ]: 1st transform added to end of current transform sequence   <br>
    const SmVector3d       * cpOptScale = NULL    ///< [out]: 2nd transform added to end of current transform sequence   <br>
  ) ;

  // rtn: SM_ERR when PlaneNormal is zero length
  SmStatus          ProjectToPlaneSimple         
  (
    const SmPoint3d   & crPlanePt,               ///< [in ]: ProjSpace Point on plane  <br>
    const SmVector3d &crPlaneNormal              ///< [in ]: ProjSpace plane's normal  <br>
  ) ; 
  
  // rtn: SM_ERR when axes submatrix is invertible
  SmStatus          InvertSimple                 () ; 

  // predicates
  
  // rtn: TRUE = axes sumMatrix is RightHanded, Unit, and Orthonormal
  SmBoolean IsUnitOrthonormal() const ;  

  // work with SmAxis2Placement objects 

  // rtn: SM_ERR when Transform is not equivalent to a SmAxis2Placement
  SmStatus  LoadAxis2Placement(SmAxis2Placement &rAxis2Placement) const ;  

  // eff: set m_adT from rAxis2Placement data
  void      SetAxis2Placement (const SmAxis2Placement &rAxis2Placement) ;  
  
  //  the SmTransform 4x4 matrix is the transpose to the tranformation matrix used by NLib and OpenGL
  void      Transpose() ; 

  // work with NLib (Nlib 4x4) = Transpose(SMLib 4x4) ;

  // eff: set adNLibMat = transpose(m_adT)
  SmStatus  LoadNLibTransform(double **adNLibMat) const ; 

  // eff: set m_adT = transpose(adNLibMat)
  void      SetNLibTransform (double **adNLibMat) ;          

  // work with sequence of rotation angles

  // rtn: SM_ERR when Transform is not equivalent to a rotated unit-ortho coordinate system
  SmStatus  DecomposeToAngles
  (
    double & rdXRotRad,            ///< [out]: 1st rotation about X Axis     <br>
    double & rdYRotRad,            ///< [out]: 2nd rotation about Y Axis     <br>
    double & rdZRotRad             ///< [out]: 3rd rotation about Z Axis     <br>
  ) const ; 

  void      ComposeFromAngles
  (
    double    dXRotRad,             ///< [in ]: 1st rotation about X Axis     <br>
    double    dYRotRad,             ///< [in ]: 2nd rotation about Y Axis     <br>
    double    dZRotRad,             ///< [in ]: 3rd rotation about Z Axis     <br>
    SmPoint3d *pOptCenter = NULL    ///< [in ]: Center, NULL=leave at origin  <br>
  ) ;

  // base class SmVolume virtual method implementations

  // convenience functions for SmBSplineVolume - other classes use base default behaviors
  virtual ULONG    GetDegree( SmVolumeParamType eParam ) const
  {
    SM_REF1( eParam ); return 1;
  }

  virtual SmStatus GetKnots
  (
    SmVolumeParamType  eParam,              ///< NotUsed: [in ]: one of SM_VP_U, SM_VP_V, SM_VP_W                                <br>
    SmTArray<double> & rKnots,              ///< [out]: ParamSpace knot list                                            <br>
    SmTArray<ULONG>  * pKnotMults = NULL,   ///< [out]: associated knot multipliticies. NULL to ignore. default:[NULL]  <br>
    const SmExtent1d * pOptIvl = NULL       ///< NotUsed: [in ]: interval of interest, NULL=Natural Interval, default:[NULL]     <br>
  ) const ;

  virtual ULONG    GetNumberControlPoints( SmVolumeParamType eParam ) const
  {
    SM_REF1( eParam ); return 2;
  }

  // SmVolume base class SimpleMap interface

  // make exact BSplineCurve projection of InputSpace Curve to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutCurve  
  (
    SmVolumeSpaceTYPE eInputSpace,          ///< [in ]: one of SM_VS_PARAM_SPACE = InputCurve is mapped from ParamSpace  <br>
                                            ///<      :        SM_VS_IN_SPACE    = InputCurve is mapped from InSpace     <br>
    const SmCurve   & rInputCurve,          ///< [in ]: Target InSpace Curve                                             <br>
    SmBSplineCurve   *& rpNewCurve          ///< [out]: NonNULL when exact surface is built, else NULL                   <br>
  ) const ;

  // make exact BSplineSurface projection of InputSpace Surface to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutSurface
  (
    SmVolumeSpaceTYPE eInputSpace,          ///< [in ]: SM_VS_IN_SPACE   = InputSurface is projected from InSpace       <br>
                                            ///<      : SM_VS_PARAM_SPACE= InputSurface is projected from ParamSpace    <br>
    const SmSurface & rInputSurface,        ///< [in ]: Surface to project to Last OutSpace                             <br>
    SmBSplineSurface *& rpNewSurface        ///< [out]: 1st OutSpace projection or NULL for not possible                <br>
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
    const SmPoint3d & crParamPoint,         ///< [in ]: ParamSpace point to map to ProjSpace point                                      <br>
    ULONG             lHighestDeriv,        ///< [in ]: 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                        <br>
    SmBoolean         bUFromLeft,           ///< NotUsed: [in ]: if P is on U, V, or w interval boundary                                         <br>
    SmBoolean         bVFromLeft,           ///< NotUsed: [in ]: TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
    SmBoolean         bWFromLeft,           ///< NotUsed: [in ]: FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
    SmVector3d      * aDerivatives,         ///< [out]: matrix of ParamSpace evaluations values                                         <br>
                                            ///<      : 3d organized: D[u][v][w]                                                        <br>
                                            ///<      : 1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3              <br>
                                            ///<      : sized       : [n+1][n+1][n+1], where n=lHighesDeriv                             <br>
                                            ///<      : lHghDrv = 0,   sized: [1],                                                      <br>
                                            ///<      :   i=0          order: [D]                                                       <br>
                                            ///<      : lHghDrv = 1,   sized: [8]                                                       <br>
                                            ///<      :   i=u*4+v*2+w  order: [D  Dw  Dv  ---                                           <br>
                                            ///<      :                        Du --- --- ---]                                          <br>
                                            ///<      : lHghDrv = 2,   sized: [27]                                                      <br>
                                            ///<      :   i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---                      <br>
                                            ///<      :                        Du  Duw --- Duv --- --- --- --- ---                      <br>
                                            ///<      :                        Duu --- --- --- --- --- --- --- ---]                     <br>
                                            ///<      : lHghDrv = 3,   sized: [81]                                                      <br>
                                            ///<      :   i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw        <br>
                                            ///<      :                        --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---        <br>
                                            ///<      :                        Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---        <br>
                                            ///<      :                        --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---        <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  Duuu ---        <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---        <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---        <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---        <br>
                                            ///<      :                        --- --- ]                                                <br>
    SmBoolean bNonZeroTangents=TRUE,        ///< NotUsed: [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors    <br>
                                            ///<      : FALSE= return exact tangent values, default:[TRUE]                              <br>
                                            ///<      : note: Surprisingly TRUE is the common choice because most tangent uses          <br>
                                            ///<      :       are for their direction (Binorm, SurfNorm comps), but when the            <br>
                                            ///<      :       tangent is being used for its magnitude (like an arc-length comp)         <br>
                                            ///<      :       then set this to FALSE.                                                   <br>
                                            ///<      : default:[TRUE]                                                                  <br>
    SmBoolean bDoZeroSampling=TRUE          ///< NotUsed: [in ]: for internal use only, always set to TRUE, default:[TRUE]                       <br>
  ) const ;

  // compute ProjSpace BBoxes for given ParamSpace Box
  virtual SmStatus EvaluateBoundingBoxSimple
  (
    const SmExtent3d  & crParamDomain,                  ///< [in ]: ParamSpace BBox to project to Project Space                                                            <br>
    SmPseudoBox       * pOptParamPseudoBox = NULL,      ///< [in ]: optional ParamSpace PseudoBox used to set output PseudoBox orientations,                               <br>
                                                        ///<      : NULL   : ProjPseudoBox Basis = Project X Y Z ParamVecs to ProjSpace at crParamBox Center               <br>
                                                        ///<      : NotNULL: ProjPseudoBox Basis = Project pOptParamPseudoBox BasisVecs to ProjSpace at crParamBox Center  <br>
                                                        ///<      : default:[NULL]                                                                                         <br>
    SmExtent3d        * pNormalProjBox = NULL,          ///< [out]: ProjectSpace Axis aligned box                                                                          <br>
    SmPseudoBox       * pPseudoProjBox = NULL           ///< [out]: ProjectSpace Non-axis aligned box                                                                      <br>
  ) const ;

  // Create ProjSpace IsoCurve from a ParamSpace IsoParamLine - requires derived implementation
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

  // Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane - requires derived implementation
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
    const SmPoint3d  & crProjPoint,          ///< [in ]: ProjSpace Point to map back to ParamSpace                                             <br>
    SmBoolean        & rbFoundAnswer,        ///< [out]: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point                      <br>
    SmSolutionArray  & rSolutions,           ///< [out]: Contains ParamSpace Found Point                                                       <br>
                                             ///<      : rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                           <br>                    
                                             ///<      : rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);  <br>                    
                                             ///<      : rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location          <br>          
                                             ///<      : rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location          <br>          
                                             ///<      : rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location          <br>          
    const SmExtent3d * pOptParamDomain=NULL  ///< NotUsed: [in ]: ParamSpace domain over which to search for the inverse point                          <br>
  ) const ;                                  

  // Local Point Solver with a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  virtual SmStatus  LocalPointSolveSimple
  (
    const SmPoint3d  & crProjPoint,          ///< [in ]: ProjSpace Point to map back to ParamSpace                                            <br>
    const SmPoint3d  & crParamPointGuess,    ///< NotUsed: [in ]: ParamSpace point guess, the closer to the actual ParamSpace point the better         <br>
    SmBoolean        & rbFoundAnswer,        ///< [out]: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point                     <br>
    SmSolution       & rSolution,            ///< [out]: Contains ParamSpace Found Point                                                      <br>
                                             ///<      : rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                          <br>                   
                                             ///<      : rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint); <br>                   
                                             ///<      : rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location         <br>         
                                             ///<      : rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location         <br>         
                                             ///<      : rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location         <br>         
    const SmExtent3d * pOptParamDomain=NULL  ///< [in ]: ParamSpace domain over which to search for the inverse point                         <br>
                                             ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                       <br>
  ) const ;                                   

  // implement cheap predicates to avoid expensive default point sampling behavior
  virtual SmBoolean IsBoundedSimple    () const { return FALSE ; }

  SmBoolean         IsOrthogonal       () const ;

  // rtn: TRUE = bases are orthogonal and parallel to XYZ.
  SmBoolean         IsAxisAligned 
  (
    long alAxisMap[3]                    ///< [out]: alAxisMap[0] = index+1 of basis parallel to X, neg = in negative direction   <br>
                                         ///<      : alAxisMap[1] = index+1 of basis parallel to Y, neg = in negative direction   <br>
                                         ///<      : alAxisMap[2] = index+1 of basis parallel to Z, neg = in negative direction   <br>
  ) const ; 
                                                                   
  virtual SmBoolean IsClosedSimple     
  (
    SmBoolean        & rbClosedU,                  ///< [out]: TRUE = closed in U direction, [check Pos[Umin,v,w] == Pos[Umax,v,w] for v,w samples       <br>
    SmBoolean        & rbClosedV,                  ///< [out]: TRUE = closed in V direction, [check Pos[u,Vmin,w] == Pos[u,Vmax,w] for w,u samples       <br>
    SmBoolean        & rbClosedW,                  ///< [out]: TRUE = closed in W direction, [check Pos[u,v,Wmin] == Pos[u,v,Wmax] for u,v samples       <br>
    double           * pdOptTolerance = NULL,      ///< NotUsed: [out]: pdOptTolerance = Tolerance to allow for check                                             <br>
    const SmExtent3d * pOptParamDomain = NULL,     ///< NotUsed: [in ]: ParamSpace domain over which to search for the inverse point                              <br>
                                                   ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                            <br>
    SmContinuityType * peOptContinuityU = NULL,    ///< [out]: U dir Continuity when closed                                                              <br>
    SmContinuityType * peOptContinuityV = NULL,    ///< [out]: V dir Continuity when closed                                                              <br>
    SmContinuityType * peOptContinuityW = NULL     ///< [out]: W dir Continuity when closed                                                              <br>
                                                   ///<      : oneof SM_CT_DISCONTINUOUS                                                                 <br>
                                                   ///<      :       SM_CT_C0                                                                            <br>
                                                   ///<      :       SM_CT_G1                                                                            <br>
                                                   ///<      :       SM_CT_G1_G2                                                                         <br>
                                                   ///<      :       SM_CT_G1_G2_G3                                                                      <br>
                                                   ///<      :       SM_CT_C1                                                                            <br>
                                                   ///<      :       SM_CT_C1_G2                                                                         <br>
                                                   ///<      :       SM_CT_C1_G2_G3                                                                      <br>
                                                   ///<      :       SM_CT_C1_C2                                                                         <br>
                                                   ///<      :       SM_CT_C1_G3                                                                         <br>
                                                   ///<      :       SM_CT_C1_C3                                                                         <br>
  ) const;                                                                                                                                       

  virtual SmBoolean IsPeriodicSimple   
  (
    SmBoolean        & rbPeriodicU,             ///< [out]: TRUE = G1 or better in U Dir                                   <br>
    SmBoolean        & rbPeriodicV,             ///< [out]: TRUE = G1 or better in V Dir                                   <br>
    SmBoolean        & rbPeriodicW,             ///< [out]: TRUE = G1 or better in W Dir                                   <br>
    const SmExtent3d * pOptParamDomain = NULL   ///< NotUsed: [in ]: ParamSpace domain over which to search for the inverse point   <br>
                                                ///<      : NULL = use Map's NaturalDomain, default:[NULL]                 <br>
  )  const ;

  virtual SmBoolean IsSingularitySimple
  (
    const SmPoint3d   & crUVWToTest,           ///< NotUsed: [in ]: UVpoint to test                                       <br>
    SmBoolean         & rbSingularU,           ///< [out]: TRUE = [Su=0]                                         <br>
    SmBoolean         & rbSingularV,           ///< [out]: TRUE = [Sv=0]                                         <br>
    SmBoolean         & rbSingularW,           ///< [out]: TRUE = [Sw=0]                                         <br>
    double              d3dTol=SM_EFF_ZERO     ///< NotUsed: [in ]: min 3d distance between distinct points) const;       <br>
  ) const ;

  virtual SmBoolean IsOnBoundarySimple 
  (
    const SmPoint3d  & crParamPoint,           ///< NotUsed: [in ]: Volume UVWPoint to test                                       <br>
    SmBoolean        & rbOnU,                  ///< [out]: TRUE = TargetPoint on UMin or UMax                            <br>
    SmBoolean        & rbOnV,                  ///< [out]: TRUE = TargetPoint on VMin or VMax                            <br>
    SmBoolean        & rbOnW,                  ///< [out]: TRUE = TargetPoint on WMin or WMax                            <br>
    double           * pdOptTolerance = NULL,  ///< NotUsed: [in ]: max deviation allowed for point on seam                       <br>
                                               ///<      : NULL = use SM_EFF_ZERO * 1000 * (1 + maxDimension())          <br>
    const SmExtent3d * pOptParamDomain = NULL  ///< NotUsed: [in ]: ParamSpace domain over which to search for the inverse point  <br>
                                               ///<      : NULL = use Map's NaturalDomain, default:[NULL]                <br>
  ) const ;

  virtual SmBoolean IsCurveInParamDomainSimple( const SmCurve & crParamCurve ) const
  {
    SM_REF1(crParamCurve) ;
    return TRUE;
  }

  virtual SmBoolean IsSurfaceInParamDomainSimple( const SmSurface & crParamSurface ) const
  {
    SM_REF1(crParamSurface) ;
    return TRUE;
  }

  // rtn: TRUE if Simple map has internal C1 discontinuities
  virtual SmBoolean HasDiscontinuitiesSimple
  (
    SmDiscontinuities3d * pOptDisconts = NULL,              ///< [out]: list of ParamSpace discontinuities and summary data                                   <br>
    SmBoolean             bCalcGeometric = TRUE             ///< [in ]: TRUE = expensive - use geometric testing to compute actal geometric discontinuities   <br>
                                                            ///<      : FALSE= cheap - report representational discontinuites                                 <br>
                                                            ///<      : default:[TRUE]                                                                        <br>
  ) const
  {
    SM_REF1( bCalcGeometric );
    if(pOptDisconts) pOptDisconts->Init();
    return FALSE;
  }

  virtual SmStatus  CalculateContinuitiesSimple
  (
    SmVolumeParamType            eVolumeParam,               ///< [in ]: SM_VP_U, SM_VP_V, or SM_VP_W
    SmContinuityType           & reMinContinuity,            ///< [out]: minimum continuity over all interior knots                                         <br>
    SmTArray<double>           & rParams,                    ///< [out]: param values marking discontinuity                                                 <br>
    SmTArray<SmContinuityType> & rConts,                     ///< [out]: assocaited continuity type for each rParams value                                  <br>
                                                             ///<      : end param continuities = SM_CT_DISCONTINUOUS                                       <br>
    SmBoolean                    bCalcGeometric = TRUE       ///< [in ]: TRUE = expensive - use geometric testing to compute actal geometric discontinuities<br>
                                                             ///<      : FALSE= cheap - report representational discontinuites                              <br>
  ) const
  {
    SM_REF2( eVolumeParam, bCalcGeometric );
    reMinContinuity = SM_CT_CINFINITY;
    rParams.ReSet();
    rConts.ReSet();
    return(SM_SUCCESS);
  }


  // utilities

  // I/O assist methods
  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,             ///< [in ]: target output stream                                  <br>
    ULONG lDBVersionNumber          ///< [in ]: database version to get proper sequence of writes     <br>
  ) const ;                                                                        

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,              ///< NotUsed: [in ]: Object type to be read                                                       <br>
    SmDatabaseIO    & rDB,                ///< [in ]: target output stream                                                         <br>
    const SmContext & crContext,          ///< [in ]: context for new object construction                                          <br>
    SmVolume        *&rpNewVolume,        ///< [out]: NULL on input = new object allocated in this routine built from stream data  <br>
                                          ///<      : NotNULL on input = pointer to an empty object to be filled by this routine   <br>
    ULONG             lDBVersionNumber    ///< [in ]: database version to get proper sequence of writes                            <br>
  );
 
  // get memory used for volume and its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes      <br>
    SmMarkType eMarkType=SM_MT_NOMARK  ///< [in ]: uses without increment eMarkType value            <br>
  ) const ; 

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmTransform, SmVolume, SmTransform_TYPE) ;

  // pretty print SmVolume
  virtual void Dump(SmBoolean bAbbrev, ULONG lIndentCnt) const ;

  // Draw, AssertValid
  SmDisplayList * DrawAxis
  (
    SmExtent2d    * pOptUVDomain=NULL, 
    SmGfxArraySet * pOptGfxSet   =NULL
  ) const ;

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

} ; // end class SmTransform 
    
#endif // !__SMTRANSFORM_H__


