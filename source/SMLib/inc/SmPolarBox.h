// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPolarBox.h
* PURPOSE: Header file for SmPolarBox class.
**********************************************************************/

#ifndef __SMPOLARBOX_H__
#define __SMPOLARBOX_H__

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMEXTENT2D_H__
#include <SmExtent2d.h>
#endif

/*******************************************************************//**
PURPOSE: This enum defines the current state of a polar box.

NOTES: member order must run from most degenerate to most general.
***********************************************************************/
enum SmPolarBoxState {
    SM_PS_UNINITIALIZED,    // No vectors yet registered with polar box
    SM_PS_POINT_WITH_BASIS, // (0,0) vector registered - and basis vectors set
                            //   only used within AddVector3d().
    SM_PS_SINGLE_POINT,     // Vector space maps to single vector on unit sphere  
    SM_PS_ARC,              // Vector space maps to an arc on the unit sphere
    SM_PS_REGION,           // Vector space maps to a square region of unit sphere
    SM_PS_UNBOUNDED         // Unable to determine a bound for the vector space.
                            // A polar box is labeled SM_PS_UNBOUNDED
                            //   whenever a vector added to a polar box
                            //   is more than 90 degrees from basis[0] vector
};

class SmPolarBox;

/*******************************************************************//**
PURPOSE: This object represents a bounding box of polar
    coordinates on the unit sphere.  

NOTES: This object is used primarily to bound vector fields.

   The SmPolarBox angles are not spherical angles - instead each angle 
   of the SmPolar box defines a great circle on the Sphere.  Each
   pair of min/max angles defines a lune (a lemon slice shape of the unit sphere.)
   The SmPolarBox is the surface of the intersection of the Basis[1] and a 
   Basis[2] oriented lunes.
   
   The SmPolarBox angles are defined as follows in relationship to the 
   basis vectors.  The basis vectors are an ortho-normal triad called 
   (v0, v1, v2).

     dAngle1 = angle between v0 and the projection of a vector onto the
               v0/v2 plane.  Allowed values range from -PI to PI. The angle 
               is measured CCW from the V0 basis vector when viewed from the tip of the
               V1 basis vector. (positive angles move into the -v2 dirction.)
               The lune defined by the min/max dAngle1 values
               has a base which runs along the v1 basis vector.

     dAngle2 = angle between v0 and the projection of a vector onto the
               v0/v1 plane.  Allowed values range from -PI to PI. The angle 
               is measured CCW f(positive angles move into the +v1 dirction.) 
               rom the v0 basis vector when viewed from the tip of the
               V2 basis vector. The lune defined by the min/max dAngle2 values
               has a base which runs along the v2 basis vector.

   note: This angle scheme does not represent angles on the Basis[1]/Basis[2] plane uniquely.
         All one can tell for those vectors is the quadrant in which the vector lies, i.e
         for all vectors on the Basis[1]/Basis[2] plane, the angles dAngle1 and dAngle2
         become +/- Pi/2.

         The intent here is to represent small extent polar boxes.  So to prevent
         this from becoming a problem, whenever a vector is added to a polar
         box outside of the hemisphere centered on Basis[0]the polarBox's state
         is set to SM_PS_UNBOUNDED meaning all vectors are considered to fall
         within the bounding box.

   SmPolarBox objects are created empty and then expanded by adding
   vectors with SmPolarBox::AddVector3d function. As vectors are added
   the polar box's basis vectors, min/max bounding angle values,
   and PolarBoxState are set as follows:
     1. The first vector added is used to set the m_aBasis[0] (v0) vector
        and promotes the state from SM_PS_UNINITIALIZED to SM_PS_SINGLE_POINT.
        The min/max Domain is set to [0:0, 0:0]
     2. The next vector added, that is not parallel to the first vector,
        promotes the state from SM_PS_SINGLE_POINT to SM_PS_ARC.
        sets the m_aBasis[1] (v1) and m_aBasis[2] (v2) vectors so 
        that v1 is perpendicular to both input vectors and
        the v0/v2 plane contains both input vectors. The min/max
        Domain is set in the first angle to include both vectors as: [a:b, 0:0]
     3. The next vector added, that is not in the v0/v2 plane 
        promotes the state from SM_PS_ARC to SM_PS_REGION. The min/max
        domain is increased in both angles to include all input vectors
        as [a:b, c:d]
     4. when a vector is added that is more than 90 degrees from
        the v0 basis vector, the state is promoted to SM_PS_UNBOUNDED.
        The domain values are no longer valid since they are limited to
        values that run from -PI to +PI measured from the v0 basis vector.

   To maximize an SmPolarBox's effectiveness make sure the first
   vector inserted into the SmPolarBox is in the middle of the expected
   vector directions.

Compare SmPolar Angles to Spherical Coordinates

  Spherical Coordinates for Point P in 
    coordinate system with center O and basis vectors xyz:
      
    Rho   = distance from point P to origin O. (radius of sphere)
    Phi   = Angle between Z Axis and vector P-O. 
    Theta = Angle between Z Axis and projection of vector P-O onto xy plane.
      
    Cartesian Coordinates from Spherical Angles:

      x = Rho * sin(Phi) * cos(theta)
      y = Rho * sin(Phi) * sin(Theta)
      z = Rho * cos(Phi)
      
  SmPolar Coordinates for Point P, a point on the unit sphere centered on
    coordinate system with center O and basis vectors 012.
    Let Basis[0] = Z, Basis[1] = x, Basis[2] = y
    
    a1  = Angle between Basis[0]=z and vector Q-O.
    a2  = Angle between Basis[0]=z and vector S-0.
    Rho = 1.0, distance from P to origin O

    where Q = projection of point P onto yz=02 plane
          S = projection of point P onto xz=01 plane

    Cartesian Coordinates from SmPolar Angles:  
      
      Consider P = x*Basis[1] + y*Basis[2] + z*Basis[0], decomposition of P into Basis coordinates

      with q = |Q-O|, length of vector Q-O
           s = |A-O|, length of vector S-O
      
        Then  x = s *  sin(a2), distance along Basis[1] of P decomposition
              y = q * -sin(a1), distance along Basis[2] of P decomposition
              z = q *  cos(a1), distance along Basis[0] of P decomposition
         also z = s *  cos(a2), distance along Basis[0] of P decomposition

      compute q and s from a1 and a2

            x**2       +     y**2       +     z**2       = 1
        (s*sin(a2))**2 + (q*sin(a1))**2 + (q*cos(a1))**2 = 1 
        (s*sin(a2))**2 + (q*sin(a1))**2 + (s*cos(a2))**2 = 1
        
        [ sin(a2)**2     1      ][s**2] = [1]
        [     1      sin(a1)**2 ][q**2]   [1]

        [s**2] = [ sin(a1)**2    -1      ][1] = [-sin(a1)**2     1      ][1] = [-sin(a1)**2     1      ][1]
        [q**2]   [    -1      sin(a2)**2 ][1]   [     1     -sin(a2)**2 ][1]   [     1     -sin(a2)**2 ][1]
                 ----------------------------   ----------------------------   --------------------------------------------
                   (sin(a2)*sin(a1))**2 - 1      1 - (sin(a2)*sin(a1))**2       (1 - sin(a1)*sin(a2))*(1 + sin(a1)*sin(a2))   
        s = sqrt( (1-sin(a1)**2) / (1 - sin(a1)*sin(a2))*(1 + sin(a1)*sin(a2)) )
        q = sqrt( (1-sin(a2)**2) / (1 - sin(a1)*sin(a2))*(1 + sin(a1)*sin(a2)) )

        s/q = sqrt(cos(a12)**2 / cos(a2)**2)
            = cos(a1)/cos(a2)

        so    Vec(a1,a2) =   q * cos(a1) * Basis[0] 
                           - q * sin(a1) * Basis[2]
                           + s * sin(a2) * Basis[1]

                             (   cos(a1) * Basis[0]      )   (   cos(a1) * Basis[0]                  )
        and   Vec(a1,a2)/q = ( - sin(a1) * Basis[2]      ) = ( - sin(a1) * Basis[2]                  )
                             ( + s/q * sin(a2) * Basis[1])   ( + sin(a2) * cos(a1)/cos(a2) * Basis[1])

                                                           (   cos(a1) * Basis[0]                  )
        and   Vec(a1,a2) = Unitize(Vec(a1,a2)/q)  = Unitize( - sin(a1) * Basis[2]                  )
                                                           ( + sin(a2) * cos(a1)/cos(a2) * Basis[1])

***********************************************************************/
class SM_EXPORT SmPolarBox
{
protected:
    SmVector3d m_aBasis[3];           // a Polar coordinate system
                                      //  m_aBasis[0] = 'Z' or 'polar axis' axis  
                                      //  m_aBasis[0] is set equal to the 1st vector added to a PolarBox
                                      //  m_aBasis[2] is set so the m_Basis[0]/m_Basis[2] plane contains 2nd Vector added to PolarBox 
                                      //  m_aBasis[1] is set perpendicular to m_Basis[0] and m_Basis[2]

    SmExtent2d m_sPolarDomain;        // min/max angles from polar axis in radians 
                                      //   x coords are CCW angle measured from m_basis[0] about m_basis[1] axis 
                                      //       of projection of vector onto m_Basis[0]/m_Basis[2] plane 
                                      //   y coords are CCW angle measured from m_basis[0] about m_basis[2] axis 
                                      //       of projection of vector onto m_Basis[0]/m_Basis[1] plane 

    SmPolarBoxState m_ePolarBoxState; // oneof:
                                      //   SM_PS_UNINITIALIZED,    // No vectors yet registered with polar box 
                                      //   SM_PS_POINT_WITH_BASIS, // vector (0,0) registered with a basis set          
                                      //   SM_PS_SINGLE_POINT,     // Vector space maps to single vector on unit sphere   (eg: tangents from a line) 
                                      //   SM_PS_ARC,              // Vector space maps to an arc on the unit sphere      (eg: tangents from a planar curve) 
                                      //   SM_PS_REGION,           // Vector space maps to a square region of unit sphere (eg: tangents from a non-planar curve)
                                      //   SM_PS_UNBOUNDED         // Unable to determine a bound for the vector space - 
                                      //                           //   Vectors added to PolarBox not contained within +/- Pi of polar axis 
     
public:
    // constructors, assignment operator, destructor
    SmPolarBox() : m_ePolarBoxState(SM_PS_UNINITIALIZED) { }
    SmPolarBox(const SmVector3d & crInitialVector);
    SmPolarBox &operator = (SmPolarBox const &obj);
    ~SmPolarBox() { }

    // grow PolarBox: 1st vector sets polar axis - subsequent vectors increase min/max points and State
    SmStatus        AddVector3d(const SmVector3d & crVectorToAdd, const SmVector3d *pOptBasis2 = NULL) ;
    SmStatus        Union(const SmPolarBox & crOther, SmPolarBox & crResult) const ;
    void            ExpandAbsolute(double dExpansion) ;
    SmStatus        Cross(const SmPolarBox & crOther, SmPolarBox & rCrossBox) const ;

    // clear PolarBox by taking it back to its uninitialized state. 
    void            ReSet() { m_ePolarBoxState = SM_PS_UNINITIALIZED ; }

    // simple data access
    SmExtent2d         GetDomain        ()                            const { return m_sPolarDomain ; }
    SmPolarBoxState    GetPolarBoxState ()                            const { return m_ePolarBoxState ; }
    SmPoint2d          ComputePolarCoord(const SmVector3d & crVector) const ;
    double             GetPolarArea     ()                            const ;
    const SmVector3d * GetBasis(ULONG lIndex)                         const { SM_ASSERT(lIndex <= 2) ;
                                                                              return( & m_aBasis[lIndex] ) ;
                                                                            }
    // predicates      
    SmBoolean          AreEqual          (const SmPolarBox & crOther)  const ;
    SmBoolean          IsContainedBy     (const SmPolarBox & crOther, 
                                          double dTol=SM_EFF_ZERO_RAD) const ; 
    SmBoolean          AreDisjoint       (const SmPolarBox & crOther)  const ;
    SmBoolean          IsDisjointWithCone(const SmVector3d & crConeVec,
                                          double dConeAngleInRadians)  const ;
    SmBoolean          IsPerpendicularDisjointWithCone(const SmVector3d & crConeVec,
                                                       double dConeAngleInRadians) const ;
    SmBoolean          HasPerpendicularToVector(const SmVector3d & crTestVector)   const ;
    SmBoolean          HasLargerSpan           (const SmPolarBox & crOther)        const ;
                       
    SmVector3d EvaluateNormalized(double dAlpha, double dBeta) const ;
    SmStatus   GetBoundaryVectors(ULONG & rNumVectors, SmVector3d sVectors[4]) const ;

public:

    SmBoolean  AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                           SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                     //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                     //      default:[SM_LEVEL_0] 
                           SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                           SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                          const ;
    SmStatus        TestTransform(SmBoolean &rbPassedTheTest) const ;
    void            Dump(void) const;
    SmDisplayList * Draw(const SmPoint3d & crSphereCenter) const;

    SM_TYPE       GetType()            const { return(SmPolarBox_TYPE) ; }
    const TCHAR  *GetTypeString()      const { return(_T("SmPolarBox_TYPE")) ; }
    const TCHAR  *GetClassString()     const { return(_T("SmPolarBox")) ; }
    SM_TYPE       GetClassType()       const { return(SmPolarBox_TYPE) ; }
    const TCHAR  *GetClassTypeString() const { return(_T("SmPolarBox_TYPE")) ; }
 
} ; // end class SmPolarBox

#endif // !__SMPOLARBOX_H__
