// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmRegion.h
* PURPOSE: Header file for SmRegion class.
**********************************************************************/

#ifndef __SMREGION_H__
#define __SMREGION_H__

#ifndef __SMOWNINGTOPOLOGY_H__
#include <SmOwningTopology.h>
#endif

/*******************************************************************//**
PURPOSE: This class is used to represent a topological region (3D Volume). 

NOTES: 
***********************************************************************/
class SM_EXPORT SmRegion : public SmOwningTopology
{
    friend class SmBrep;
    friend class SmShell;
    friend class SmBrepConstructor;
    friend class SmObjsDelete<SmRegion*>;

protected:
    // inherited:
    // SmTopology::m_pListOwner       - used to store pointer to owning SmBrep
    // SmTopology::m_pNext            - used to store doubly linked list of
    // SmTopology::m_pLast                   SmRegion objects belonging to SmBrep
    // SmOwningTopology::m_pList      - pointer to head of Regions's Shell link-list 
    // SmOwningTopology::m_lListSize  - number of Shells in this region

  SmBoolean   m_bIsVoidFlag;        // TRUE = region is a void. 
                                    //    1. In a manifold world this marks a region that is
                                    //       outside the modeled part.
                                    //       1a. The infinite region is a void
                                    //       1b. all inner shells (holes in the solid) are voids
                                    //    2. In a non-manifold world voids are set explicitly
protected:
    SmRegion() 
    {
      m_bIsVoidFlag      = FALSE ;
      // report construction at SmObject::Notify level - skip other levels
      SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL) ;
    }

    virtual ~SmRegion() 
    { 
      Notify(SM_NO_DESTRUCTION, this, NULL, NULL); 
      m_bIsVoidFlag = UNSURE; 
    }

public:

    SmStatus CalculateBoundingBox
    ( 
      SmExtent3d & rBBox,             ///< [out]: Bounding Box containing Region                                            <br>
      SmBoolean    bTight = FALSE     ///< [in] : TRUE = compute minimal box for each contained face and edge (expensive)   <br>
                                      ///<      : FALSE= compute any box larger than each contained face and edge (cheaper) <br>
    ) const; 

    // compute area, volume, center of gravity and moments using a tessellated approximation
    SmStatus ComputeProperties
    (
      double dEdgeTessTol,                            ///< [in] :                                                                   <br>
      double dFaceTessTol,                            ///< [in] :                                                                   <br>
      const SmPoint3d & crOriginOfComputation,        ///< [in] :                                                                   <br>
      double & rdArea,                                ///< [out]:                                                                   <br>
      double & rdVolume,                              ///< [out]:                                                                   <br>
      SmPoint3d & rBarycenter,                        ///< [out]:                                                                   <br>
      SmVector3d aMoments[2],                         ///< [out]: aMoments[2]  The following moments are currently supported:       <br>
                                                      ///<      : aMoments[0] Contains Ixx, Iyy, Izz                                <br>
                                                      ///<      : aMoments[1] Contains Ixy, Iyz, Izx                                <br>
      SmMassPropertiesType eWhichProps = SM_MPT_ALL   ///< [in] : SM_MPT_AREA                                                       <br>
                                                      ///<      : SM_MPT_VOLUME                                                     <br>
                                                      ///<      : SM_MPT_CENTROID                                                   <br>
                                                      ///<      : SM_MPT_MOMENTS                                                    <br>
                                                      ///<      : SM_MPT_ALL.                                                       <br>
                                                      ///<      : These are sequenctial, in this order: specifying any one causes   <br>
                                                      ///<      : that one and all earlier ones to be calculated.                   <br>
                                                      ///<      : Note SM_MPT_MOMENTS is( currently ) the same as SM_MPT_ALL.       <br>
    ) const;


    // compute area, volume, center of gravity and moments using analytic techniques
    SmStatus ComputePreciseProperties
    (
      double dRelativeAccuracy,                       ///< [in] : Relative accuracy between                                                    <br>
                                                      ///< [in] : 1.0e-1 and 1.0e-4 appears to be the best.  You can go                        <br>
                                                      ///< [in] : tighter but it gets really slow if you have complex                          <br>
                                                      ///< [in] : trim curves.                                                                 <br>
      const SmPoint3d & crOriginOfComputation,        ///< [in] :                                                                              <br>
      double dEstimatedArea,                          ///< [in] : If not given then the area will be estimated by a bounding box of the SmBrep <br>
      double & rdArea,                                ///< [out]:                                                                              <br>
      double & rdVolume,                              ///< [out]:                                                                              <br>
      SmTArray<SmVector3d> & rMoments                 ///< [out]: [0] Ix, Iy, Iz    - Area static (first) moments:       integral of (x, y, z) dA                <br>
                                                      ///< [out]: [1] Ixx, Iyy, Izz - Area second moments about planes:   integral of (x^2, y^2, z^2) dA          <br>
                                                      ///< [out]: [2] Iyz, Izx, Ixy - Area products of inertia:           integral of (yz, zx, xy) dA             <br>
                                                      ///< [out]: [3] Ixx, Iyy, Izz - Area moments of inertia about axes: integral of (y^2+z^2, x^2+z^2, x^2+y^2) dA <br>
                                                      ///< [out]: [4] Ix, Iy, Iz    - Volume static (first) moments:      integral of (x, y, z) dV                <br>
                                                      ///< [out]: [5] Ixx, Iyy, Izz - Volume second moments about planes: integral of (x^2, y^2, z^2) dV          <br>
                                                      ///< [out]: [6] Iyz, Izx, Ixy - Volume products of inertia:         integral of (yz, zx, xy) dV             <br>
                                                      ///< [out]: [7] Ixx, Iyy, Izz - Volume moments of inertia about axes: integral of (y^2+z^2, x^2+z^2, x^2+y^2) dV <br>
    ) const;

    inline void GetShells
    (          
      SmTArray< SmShell* >  & rShells,  ///< [out]: list of objects                                              <br>
      ULONG *pOptAttributeId = NULL     ///< [in] : only include objects containing an attribute with this id    <br>
    ) const;                     

    SmStatus  GetAdjacentRegions( SmTArray< SmRegion* > & rRegions ) const;

    inline SmBrep*   GetBrep() const;

    SmBoolean IsVoid          ()                          const { return m_bIsVoidFlag ; } // TRUE = region is a void not the inside of a part
    void      SetIsVoid       (SmBoolean bIsVoidFlag);                                     // In non-manifold models, voids are specified explicitly
    SmBoolean IsDegenerate    (double d3DTol=SM_EFF_ZERO) const ;                          // TRUE = all Region geometries fit within BBox.MaxDim < d3DTol 
    SmBoolean IsInfiniteRegion()                          const ;

    SmStatus  OrderShells();  // Make sure our shells follow SMLib topology rules.

    // Draw Faceuse-Shells with colored Faceuse normals to show Infinite, solid, and void regions ()
    SmDisplayList* Draw
    (
      double dNormalGain=0.0,           ///< [in] : [0.0] = guess from Shell BBox Size  <br>
      SmGfxArraySet * pOptGfxSet=NULL   ///< [in] :                                     <br>
    ) const;

    virtual SmBoolean AssertValid
    (
      SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                             <br>
      SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         <br>
                                                ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     <br>
      SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   <br>
      SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   <br>
    )  const ;

    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmRegion,SmOwningTopology,SmRegion_TYPE);

    void DumpTopology(ULONG lWalkDepth=0) const ;  // lWalkDepth[0] = no walk, [99] = walk to bottom
    
} ; // end class SmRegion

/*******************************************************************//**
PURPOSE: Get the Shells in a region.

NOTES: 
***********************************************************************/
inline void SmRegion::GetShells
(
  SmTArray<SmShell*> & rShells,          ///< [out]: list of objects                                              <br>
  ULONG              * pOptAttributeId   ///< [in] : only include objects containing an attribute with this id    <br>
                                         ///<      : NULL to ignore, default:[NULL]                               <br>
) const 
{ 
  GetAll(SM_REINTERPRET_CAST(SmTArray<class SmTopology*>&,rShells), pOptAttributeId); 
}

/*******************************************************************//**
PURPOSE: Get the brep of a region.

NOTES: 
***********************************************************************/
inline SmBrep* SmRegion::GetBrep() const 
{ 
  SM_ASSERT(m_pListOwner != NULL); 
  return (SmBrep*)m_pListOwner; 
}

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    extern SmRegion * dbgRegion1 ;
//    extern SmRegion * dbgRegion2 ;
//    
//    #endif // SM_DEBUG_CODE

#endif // !__SMREGION_H__
