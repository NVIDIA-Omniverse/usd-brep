// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmShell.h
* PURPOSE: Header file for SmShell class.
**********************************************************************/

#ifndef __SMSHELL_H__
#define __SMSHELL_H__

#ifndef __SMOWNINGTOPOLOGY_H__
#include <SmOwningTopology.h>
#endif

#ifndef __SMREGION_H__
#include <SmRegion.h>
#endif

#ifndef __SMVERTEXUSE_H__
#include <SmVertexuse.h>
#endif

#ifndef __SMPOINTCLASSIFICATION_H__
#include <SmPointClass.h>
#endif

/*******************************************************************//**
PURPOSE: This class represents topological Shells.

NOTES: 
***********************************************************************/
class SM_EXPORT SmShell : public SmOwningTopology
{
    friend class SmBrep;
    friend class SmRegion;
    friend class SmBrepConstructor;
    friend class SmPointClassification;
    friend class SmObjsDelete<SmShell *>;
protected:
    // inherited:
    // SmTopology::m_pListOwner       - used to store pointer to owning SmRegion
    // SmTopology::m_pNext            - used to store doubly linked list of
    // SmTopology::m_pLast                   SmShell objects belonging to SmRegion
    // SmOwningTopology::m_pList      - ShellType == SmFaceuse_TYPE
    //                                     pointer to head of Shell's Faceuse link-list
    //                                  ShellType == SmEdgeuse_TYPE
    //                                     pointer to Edgeuse
    //                                  ShellType == SmVertexuse_Type
    //                                     pointer to Vertexuse 
    // SmOwningTopology::m_lListSize  - number of Faceuses on a SmFaceuse_TYPE Shell or
    //                                  number of Edgeuses on a SmEdgeuse_TYPE Shell 
    //                                        (always 2 even when wire edge is connected to other wires
    //                                         because EU next/last ptrs are used for radial list not loop neighbors)
    //                                  number of Vertexuses on a SmVerteuse_TYPE Shell 
    //                                        (always 1) 

    SM_TYPE m_tShellType;    // Type of object contained by shell and stored in m_pList
                             //    SmVertexuse_TYPE (16006) = VertexShells,                                 
                             //    SmEdgeuse_TYPE   (16005) = WireEdges, or                                 
                             //    SmFaceuse_TYPE   (16022) = FaceShells with or without attached WireEdges 
protected:
    SmShell() ;
    virtual ~SmShell();
public:

    SmStatus CalculateBoundingBox
    ( 
      SmExtent3d & rBBox,            // out: Bounding Box containing Region
      SmBoolean    bTight = FALSE    // in : TRUE = compute minimal box for each contained face and edge (expensive)      
                                     //      FALSE= compute any box larger than each contained face and edge (cheaper)
    ) const; 

    // fire a ray against the Faceuses of this shell - side effect: increments unused Mark value
    SmStatus RayFire
    (
      const SmPoint3d       & crRayStartPoint,          // in : Point location to classify
      const SmVector3d      & crRayVector,              // in : ray direction
      SmZoneTol3d             sRayZoneTol3d,            // in : RayZoneTol3d - sizes XSectTol3d dist for finding objects
      SmRegion             *& rpRegionOfPoint,          // out: Region containing point or 
                                                        //      NULL when ray does not intersect this Shell
      SmPointClassification & rRayIntersection,         // out: Point classification of ray intersection
                                                        //      classification = SM_PC_UNKNOWN when ray
                                                        //      does not intersect this shell.
      double                * pOptSphereBound = NULL    // in : Optional spherical bound to avoid BBox construction (this method can be called many times in succession for the same brep).
    ) const;

    void             GetFaceuses (SmTArray<SmFaceuse*> & rFaceuses)  const; // get list of faceuses in this shell
    SmStatus         GetWireEdges(SmTArray<SmEdge*>    & rWireEdges) const; // get list of wireEdges in this shell
    SmRegion       * GetRegion()       const;
    virtual SmBrep * GetBrep()         const;
    SmVertex       * GetVertex()       const;  // get the vertex of a vertex shell
    SmEdge         * GetWireEdge()     const;  // Get first edge of edgeuse shell
    SmFaceuse      * GetFirstFaceuse() const;  // get first faceuse of a faceuse shell 
    SmStatus         GetMatedShells(SmTArray<SmShell *> &rMatedShells)  const;  // increments an unused Mark value

    SmBoolean IsVertexShell()  const;  // shell is one vertex  ; m_pList points to Vertexuse
    SmBoolean IsWireShell()    const;  // shell is set of wires; m_pList points to Edgeuse
    SmBoolean IsFaceuseShell() const;  // shell is set of faces; m_pList points to Faceuse
    SmBoolean IsUnknownShell() const;  // shell is not yet used; m_pList points to NULL


    SmBoolean IsDegenerate(double d3DTol=SM_EFF_ZERO) const ; // TRUE = all Shell geometries fit within BBox.MaxDim < d3DTol 
    SmBoolean IsClosed() const; // Separates two (or more) regions

    SmStatus  IsInnerShell( SmBoolean &bIsInner ) const;  // Meaningful only on Closed Shells.

    // note: increments unused Mark value
    SmBoolean IsInnerShellOf(const SmShell *pOptOuterShell) const;  // pOptOuterShell == NULL for infinite region
                                                                    // pOptOuterShell != NULL outer shell to test for containing inner shell

    void SetShellType(SM_TYPE tShellType); 

    // transfer all children of this to the recipient
    SmStatus TransferChildrenTo(SmShell * pRecipient,
                                SmTArray<SmEdge*> * pWireEdgesInBrep);

    // Draw Faceuse-Shells with colored Faceuse normals to show Infinite, solid, and void regions (dNormalGain:[0.0] = guess from Shell BBox Size)
    SmDisplayList * Draw(double dNormalGain=0.0, SmGfxArraySet * pOptGfxSet=NULL) const;

    // return one shell point representative of this shell's location
    SmPoint3d GetGraphicsPoint() const ; 

    virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                  SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                            //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                            //      default:[SM_LEVEL_0] 
                                  SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                  SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                                 const ;
    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
    virtual SmBoolean TypicalListOwnerUse() const;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmShell,SmOwningTopology,SmShell_TYPE);
    void DumpTopology(ULONG lWalkDepth=0) const ;  // lWalkDepth[0] = no walk, [99] = walk to bottom

} ; // end class SmShell

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    extern SmShell * dbgShell1 ;
//    extern SmShell * dbgShell2 ;
//    
//    #endif // SM_DEBUG_CODE

#endif // !__SMSHELL_H__
