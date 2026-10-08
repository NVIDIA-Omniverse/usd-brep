// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmAxis2Placement.h
* PURPOSE: Header file for Axis2Placement object
**********************************************************************/

#ifndef __SMAXIS2PLACEMENT_H__
#define __SMAXIS2PLACEMENT_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

class SmDisplayList ;

/*******************************************************************//**
PURPOSE: This object represents a 3D coordinate system as a point,
    an X Axis and a Y Axis.  The implied Z Axis is equal to X cross Y.
    Like a transformation matrix it can move points and vectors from one
    coordinate system to another except that it does not allow scaling 
    and perspective transformations.  It allows any sequence of rotations 
    and translations and one last mirror about the XY plane.

NOTES: intent is that m_vXAxis and m_vYAxis are unit vectors
    perpendicular to one another.  However, the SetCanonical() method
    can be used to set nonOrthogonal values while signalling an error.

    0. The SmAxis2Placement object is a coordinate system representation
       and not a general transformation representation.  Sometimes this
       class looks like a transformation object because its eval methods can be
       used to project points from one right handed coordinate system to another.

       To manage constructing desirable IsAxis2Placement objects, 
       methods exist to transform one coordinate system into another
       by the application of any arbitrary sequence of rotations and translations.
       
       So the class does have many of the properties of a general transformation class.  
       However, this class does not support any mirroring or scaling transformations.
       For convenience, mirror evaluate methods are implemented that will allow
       one to mirror points and vectors about the XY plane of an SmAxis2Placement object.
       There are no scalling methods.

    1. The equivalent 4x4 matrix representation of the SmAxis2Placment object is
       [X0 X1 X2 0]
       [Y0 Y1 Y2 0]   where X = m_vXAxis
       [Z0 Z1 Z2 0]         Y = m_vYAxis
       [O0 O1 O2 1]         Z is not stored but computed as Z = cross_product(X,Y)
                        and O = Origin

    2. Outpoint = [InPoint.x InPoint.y InPoint.z 1] * [X0 X1 X2 0]
                                                      [Y0 Y1 Y2 0]
                                                      [Z0 Z1 Z2 0]
                                                      [O0 O1 O2 1]
       which is the same as: OutPoint = Origin + InPoint.x * XAxis
                                               + InPoint.y * YAxis
                                               + InPoint.z * ZAxis

         this maps the unit x point [1,0,0] to m_vOrigin + m_vXAxis
         this maps the unit y point [0,1,0] to m_vOrigin + m_vYAxis
         this maps the unit z point [0,0,1] to m_vOrigin + m_vZAxis

       use: TransformPoint(InPt, OutPt)    // Map InPoint from Current XYZ to SmAxis2Placement XYZ
            TransformVector(InVec, OutVec) // Map InVec   from Current XYZ to SmAxis2Placement XYZ
            InvTransformPoint(OutPt, InPt)   // Map OutPoint from SmAxis2Placement XYZ to Current XYZ (inverse of TransformPoint)
            InvTransformVector(OutPt, InPt)  // Map OutVec   from SmAxis2Placement XYZ to Current XYZ (inverse of TransformVector)

    3.  In words, the SmAxis2Placement transformation has two parts, 
        a rotation followed by a translation. Points are first rotated 
        about the origin and then translated.

        A pure rotation about the origin can be had by setting m_vOrigin = [0,0,0]
        A pure translation can be had by setting X, Y, Z parallel to the global axes as
             X = [1,0,0], y = [0,1,0], and Z = [0,0,1]
    
    4. Transformations concatenate.  Projecting a point through a sequence
       of transformations is the same as projecting a point through a
       concatenation of all the squences.

           Example: When Tconcat = T1 * T2 * T3 ;

                    Vout = Vin * Tconcat
                         = (((Vin * T1) * T2) * T3) ;
       This means that a single SmAxis2Placement object can be used
         to accumulate the effects of a sequence of rotate and translate transformations
         and then used just once to apply that transformation to
         a set of points or a whole Brep model. 

       Use the following methods to start and concatenate desired transformations
         Empty  Constructor()             alloc    ThisT = unity transform ;
         Orient Constructor(Orig,X,Y)     alloc    ThisT = Transform from Global to Orig centered rotated coordinates
         FromTo Constructor(FromOrig,
                            FromX,FromY,
                            ToO,ToX,ToY)  alloc    ThisT = Trasfrom from From to To centered and rotated coordinates
         Init()                           to set   ThisT = unity transform ; 
         TransformAxis2Placement(T2, T12) to build T12   = ThisT * T2 ;     // T2     = GeneralTransformation
         Translate(Vec)                   to build ThisT = ThisT * Ttrans ; // Ttrans = Translate only transform
         RotateAboutAxis(AngRad, Axis)    to build ThisT = ThisT * Trot ;   // Trot   = Rotate about origin only transform
         RotateAboutAxisAtPoint(AngRad, 
                                Pt, Axis) to built ThisT = ThisT * Trot ;   // Trot   = Rotate about pt only transform
    
    5. There are no Mirroring or Scaling transformation.
***********************************************************************/
class SM_EXPORT SmAxis2Placement
{
 protected:
   SmPoint3d  m_vOrigin; // Origin of the 3D coordinate system
   SmVector3d m_vXAxis;  // X axis (should be unit length and perp to Y axis)
   SmVector3d m_vYAxis;  // Y axis (should be unit length and perp to X axis)
                         // implied Z axis = crossProduct(X,Y)
 public:
   // empty constructor = identity transform
   SmAxis2Placement() { Init() ; } 
   
   // copy constructor
  SmAxis2Placement(const SmAxis2Placement & crSource) : m_vOrigin(crSource.GetOriginRef()),
                                                        m_vXAxis (crSource.GetXAxisRef()),
                                                        m_vYAxis (crSource.GetYAxisRef())
                                                      { }                     
  
  // orient constructor(vecs) = map Current XYZ coords to SmAxis2Placement coords 
  SmAxis2Placement(const SmPoint3d  & rOrigin,        
                   const SmVector3d & rXAxis,         
                   const SmVector3d & rYAxis) { SetCanonical(rOrigin, rXAxis, rYAxis) ; }
  
  // Orient constructor(coords) = map Current XYZ coords to SmAxis2Placement coords
  SmAxis2Placement(double dOriginX, double dOriginY, double dOriginZ,    
                   double dXAxisX,  double dXAxisY,  double dXAxisZ,
                   double dYAxisX,  double dYAxisY,  double dYAxisZ) { SmPoint3d  sOrigin(dOriginX,dOriginY,dOriginZ) ;
                                                                       SmVector3d sX(dXAxisX, dXAxisY, dXAxisZ) ;
                                                                       SmVector3d sY(dYAxisX, dYAxisY, dYAxisZ) ;
                                                                       SetCanonical(sOrigin, sX, sY) ;                                                 
                                                                     }
   // plane constructor - make plane: dot(N,x) + D = 0
  SmAxis2Placement(const SmVector3d & crNormal,        
                   double             dD) ;
  
  // from/to constructor = map FromXYZ coords to ToXYZ coords
  SmAxis2Placement(const SmPoint3d  &crFromOrigin,     
                   const SmVector3d &crFromXAxis,      
                   const SmVector3d &crFromYAxis,      
                   const SmVector3d &crToOrigin,       
                   const SmVector3d &crToXAxis,        
                   const SmVector3d &crToYAxis) { SmAxis2Placement sFrom(crFromOrigin, crFromXAxis, crFromYAxis) ;
                                                  SmAxis2Placement sTo(crToOrigin, crToXAxis, crToYAxis) ;
                                                  sFrom.Invert(*this) ;
                                                  this->TransformAxis2Placement(sTo, *this) ;
                                                }
                                                
  // Simple Data Access
  const SmPoint3d  &GetOriginRef(void)                       const { return m_vOrigin ; }
  const SmVector3d &GetXAxisRef (void)                       const { return m_vXAxis ; }
  const SmVector3d &GetYAxisRef (void)                       const { return m_vYAxis ; }
  SmPoint3d         GetOrigin   (void)                       const { return m_vOrigin ; }
  SmVector3d        GetXAxis    (void)                       const { return m_vXAxis ; }
  SmVector3d        GetYAxis    (void)                       const { return m_vYAxis ; }
  SmVector3d        GetZAxis    (void)                       const { SmVector3d sZ = m_vXAxis * m_vYAxis ; 
                                                                     sZ.Unitize() ; 
                                                                     return sZ ;
                                                                   }
  void              GetMatrix      (double * const * padMat) const ;
  void              GetMirrorMatrix(double * const * padMat) const ;
  void              GetCanonical   (SmPoint3d  & crOrigin,         
                                    SmVector3d & crXAxis,          
                                    SmVector3d & crYAxis)    const ;
  
  // forward evals: map current XYZ coords to SmAxisToPlacement XYZ coords
  void TransformPoint    (const SmPoint3d  & crUVW,           ///< [in] : point to transform     <br>
                          SmPoint3d        & rXYZ)            ///< [out]: transformed point      <br>
                         const;
                         
  void TransformVector   (const SmVector3d & crUVW,           ///< [in] :     <br>
                          SmVector3d       & rXYZ)            ///< [out]:     <br>
                         const;
                         
  // inverse evals: map SmAxisToPlacement XYZ coords back to global XYZ coords
  void InvTransformPoint (const SmPoint3d & crXYZ,            ///< [in] : point to inversely transform   <br>
                          SmPoint3d       & rUVW)             ///< [out]: point inversely transformed    <br>
                         const ;                              
                                                              
  void InvTransformVector(const SmVector3d & crXYZ,           ///< [in] :     <br>
                          SmVector3d       & rUVW)            ///< [out]:     <br>
                         const ;
  
  // mirror evals: map a point about X-Y plane of placement
  void MirrorPoint       (const SmPoint3d & crPointToMirror,  ///< [in] : point to mirror  <br>
                          SmPoint3d       & rMirror)          ///< [in] : mirrored point   <br>
                         const ;                              
                                                              
  void MirrorVector      (const SmVector3d & crPointToMirror, ///< [in] : vector to mirror <br>
                          SmPoint3d        & rMirror)         ///< [out]: mirrored vector  <br>
                         const ;
   
  // set transform methods
  void     Init()          { m_vOrigin.Set(0,0,0) ;  m_vXAxis.Set (1,0,0) ; m_vYAxis.Set (0,1,0) ; }
  
  SmStatus SetCanonical       (const SmPoint3d  & crOrigin,           ///< [in] : coordinate origin                             <br>
                               const SmVector3d & crXAxis,            ///< [in] : coordinate X Axis (should be perp to Y axis)  <br>
                               const SmVector3d & crYAxis) ;          ///< [in] : coordinate Y Axis (should be perp to X axis)  <br>
  SmStatus SetSTEPCanonical   (const SmVector3d & crOrigin,           ///< [in] : coordinate origin   <br>
                               const SmVector3d & crZAxis,            ///< [in] : coordinate Z Axis   <br>
                               const SmVector3d & crXRefDirection) ;  ///< [in] : coordinate Y Axis   <br>
                              
  // concatenation methods - post-concatenate a transform onto existing transformation
  void Translate              (const SmVector3d & crTranslation);
  void RotateAboutAxis        (double             dAngleRadians,      ///< [in] :   <br>
                               const SmVector3d & crAxis) ;           ///< [in] :   <br>
  void RotateAboutAxisAtPoint (double             dAngleRadians,      ///< [in] :   <br>
                               const SmPoint3d  & crOrigin,           ///< [in] :   <br>
                               const SmVector3d & crAxis) ;           ///< [in] :   <br>
  void TransformAxis2Placement(const SmAxis2Placement & crInput,      ///< [in] :   <br>
                               SmAxis2Placement       & rResult)      ///< [out]:   <br>
                              const ;
  
  // no mirror transformation because SmAxis2Placement does not represent left handed coord systems
  
  // invert transform to map SmAxisToPlacement XYZ coords back to global XYZ coords
  void Invert(SmAxis2Placement & rResult) const ;
  
  // work with 4x4 transformation matrices
  void Load4x4   (double ad4x4Matrix[16]) const ; // load ad4x4Matrix from stored origin, XAxis, and YAxis values
  void SetFrom4x4(double ad4x4Matrix[16]) ;       // Set Origin, XAxis, YAxis values from 1d4x4Matrix
  
  // work with sequence of rotation angles
  void DecomposeToAngles(double & rdXRotRad, double & rdYRotRad, double & rdZRotRad) const;
  void ComposeFromAngles(double    dXRotRad, double    dYRotRad, double    dZRotRad, SmPoint3d *pOptCenter=NULL) ;
  
  // equality operator
  SmBoolean operator==(const SmAxis2Placement&) const ;
  
  // Predicates
  SmBoolean IsIdentity(double dTol = SM_EFF_ZERO) const ;
  SmBoolean AreCoaxial(const SmAxis2Placement & crOther,
           SmApproxTol3d      sTol3d,
                       double             dSizeScale ) const ;
  
  // utilities
  SmDisplayList * Draw       (SmExtent2d *pOptUVDomain=NULL) const ;
  SmBoolean       AssertValid(SmAssertArray    * pAList=NULL,           ///< [in,out] : Accumulating list of failed Asserts, NULL to ignore                       <br>
                              SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                    <br>
                                                                        ///<        SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                <br>
                              SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't <br>
                              SmTArray<ULONG>  * pTestRequests=NULL)    ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order              <br>
                             const ;
                              
  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON_ONLY_BASE(SmAxis2Placement, SmAxis2Placement_TYPE) ;

} ; // end class SmAxis2Placement 
    
#endif // !__SMAXIS2PLACEMENT_H__


