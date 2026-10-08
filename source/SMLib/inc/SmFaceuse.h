// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFaceuse.h
* PURPOSE: Header file for SmFaceuse class.
**********************************************************************/

#ifndef __SMFACEUSE_H__
#define __SMFACEUSE_H__

#include <SmOwningTopology.h>
#include <SmGraphicsExtern.h>
#include <SmShell.h>

/*******************************************************************//**
PURPOSE: This class represents topological Faceuses.

NOTES: 
***********************************************************************/
class SM_EXPORT SmFaceuse : public SmOwningTopology
{
    friend class SmBrep;
    friend class SmFace;
    friend class SmLoopuse;
    friend class SmShell;
    friend class SmEdgeuse;
    friend class SmBrepConstructor;
    friend class SmHealData ; 
protected:
    // inherited:
    // SmTopology::m_pListOwner       - used to store pointer to owning SmShell
    // SmTopology::m_pNext            - used to store doubly linked list of
    // SmTopology::m_pLast                   SmFaceuse objects belonging to SmShell
    // SmOwningTopology::m_pList      - pointer to head of face's Loopuse link-list (first Loop is outer loop)
    // SmOwningTopology::m_lListSize  - number of Loopuses on this Faceuse

    SmFace       * m_pF;           // this Faceuse's Face
    SmFaceuse    * m_pFUMate;      // Face's other face use
    SmOrientType   m_eOrientation; // SM_OT_SAME     = this is an upward faceuse, Faceuse Normal =  Face->m_pSurface->Normal
                                   // SM_OT_OPPOSITE = this is a  down faceuse.   Faceuse Normal = -Face->m_pSurface->Normal
                 
protected:
    SmFaceuse() ;
    virtual ~SmFaceuse() { Notify(SM_NO_DESTRUCTION, this, NULL, NULL);
                           m_pF           = NULL;                 
                           m_pFUMate      = NULL;           
                           m_eOrientation = SM_OT_UNKNOWN;
                         } 
public:
    // gather faces connected like a shell through their Edgeuse radial ptrs.
    // Don't go around lamina edges so collecting on a partial shell (one with a hole in it)
    //   will only collect Faceuses on one side of the partial shell.
    SmStatus CollectAdjacentFaceuses(SmTArray<SmFaceuse*> & rFaceuses, double * pOptAngleThreshold = NULL) const;

    void                       GetEdgeuses (SmTArray<SmEdgeuse*> & rEdgeuses, SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const;
    void                       GetLoopuses (SmTArray<SmLoopuse*> & rLoopuses, SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const;
    void                       GetNeighbors(SmTArray<SmFaceuse*> & rFaceuses, SmBoolean bSkipLaminaEdges = FALSE) const;
    virtual SmBrep           * GetBrep()         const { SM_ASSERT(m_pListOwner != NULL) ; return GetShell()->GetBrep(); }
    SmShell                  * GetShell()        const { SM_ASSERT(m_pListOwner != NULL) ; return (SmShell*)m_pListOwner ; }
    SmRegion                 * GetRegion()       const { SM_ASSERT(m_pListOwner != NULL) ; return( m_pListOwner ? ((SmShell*)m_pListOwner)->GetRegion() : NULL) ; }
    SmFace                   * GetFace()         const { SM_ASSERT(m_pF         != NULL) ; return m_pF ; }
    SmFaceuse                * GetMate()         const { SM_ASSERT(m_pFUMate    != NULL) ; return m_pFUMate ; }
    SmFaceuse                * GetOtherFaceuse() const { SM_ASSERT(m_pFUMate    != NULL) ; return m_pFUMate ; } // backward compatible
    SmLoopuse                * Get1stLoopuse()   const { return((SmLoopuse *)GetList()) ; } 
    inline  SmOrientType       GetOrientation()  const { return m_eOrientation; }
    virtual SmAObject        * GetAOwner()       const { return ((SmAObject*)GetFace()); } // get attribute inheritance owner
    SmStatus                   GetPointsInFace
                               (ULONG lPointCount, SmTArray<SmPoint2d> &rUV, SmTArray<SmPoint3d> &rPV) const;

    // draw faceGraphics with normal vector pointing to Faceuse side and colorCodedForRegionProperty: Infinite:[green], Solid:[Blue], Void:[Red]
    SmDisplayList   * Draw(SmDrawModeType  lDrawingMode=SM_DM_CURRENT, 
                           ULONG           lUHatch=8, 
                           ULONG           lVHatch=8,
                           double          dNormalGain=0.0, 
                           SmGfxArraySet * pOptGfxSet=NULL) const ;           // draw face with mode and optional xHatch counts
    virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                  SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                            //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                            //      default:[SM_LEVEL_0] 
                                  SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                  SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                                 const ;
    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmFaceuse,SmOwningTopology,SmFaceuse_TYPE);

} ; // end class SmFaceuse

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    extern SmFaceuse * dbgFaceuse1 ;
//    extern SmFaceuse * dbgFaceuse2 ;
//    
//    #endif // SM_DEBUG_CODE

#endif // !__SMFACEUSE_H__

