// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCurveCache.h
* PURPOSE: Header file for SmCurveCache object.
**********************************************************************/

#ifndef __SMCURVECACHE_H__
#define __SMCURVECACHE_H__

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMMEMBLOCKMGR_H__
#include <SmMemBlockMgr.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMPSEUDOBOX_H__
#include <SmPseudoBox.h>
#endif

#ifndef __SMPOLARBOX_H__
#include <SmPolarBox.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMCACHEMGR_H__
#include <SmCacheMgr.h>
#endif

#ifndef __Sm_Nurbs_H__
#include <SmNurbs.h>
#endif

#ifndef __SMTREE_H__
#include <SmTree.h>
#endif

// gwc: added following dependency when the type hiding in SmBezierPatch was removed.
//      If we modify SmBeizierSpan to just have a pointer to a gw_C rather
//      than contain one, we can remove this compile dependency.
//#ifndef _NURBS_H_INCLUDED
//#include <nurbs.h>
//#endif


// forward declarations
//      typedef struct gw_curve gw_CURVE ;

/*******************************************************************//**
PURPOSE: This object contains all the extra information stored
            in the parent nodes of a curve spatial decomposition

NOTES: static class object - don't add virtual methods to SmBezierAux1d
***********************************************************************/ 
class SM_EXPORT SmBezierAux1d : public SmTreeNodeData
{

public:
  // Cached Bezier Span properties computed for the Bezier Span below
  ULONG             m_lLeft = 0;               // index of leaf node Bezier span furthest to left within this one
                                               //    note: ParentLeft    = LeftChildLeft
                                               //          LeafNodeLeft  = Its own index

  ULONG             m_lRight = 0;              // index of leaf node Bezier span furthest to right within this one
                                               //    note: ParentRight   = RightChildRight
                                               //          LeafNodeRight = its own index

  SmContinuityType  m_aeConts[2];              // continuity between this Span and its decomposition neighbors
                                               //    note: ParentConts[0] = LeftChildConts[0], 
                                               //          ParentConts[1] = RightChildConts[1]

  double            m_dMaxLength;              // control-polygon BaseLine distance, 
                                               //    note: ParentMaxLength = Sum(ChildrenMaxLength)
                                                     
  double            m_dMaxChordHeightSquared;  // max control-polygon vertex/BaseLine distance
                                               //    note: only computed for internal nodes when SmCurveCache::m_bMakeFullTree == FALSE

  double            m_dMaxTurningAngleDeg;     // accumulated control-polygon vertex angle value, 
                                               //    note: ParentMaxTurningAngle =   Sum(ChildrenMaxTurningAngle)
                                               //                                  + TurningAngle at Children join point

  SmExtent1d        m_sIvl ;                   // interval for this span

public:
  SmExtent1d  & GetInterval()                  { return m_sIvl ; }
  
public:

  void GetStart 
  (
    const SmCurveCache & crCurveCache,    ///< [in] : owner of this span                       <br>
    SmPoint3d &rStart,                    ///< [out]: Image Space Start Point                  <br>
    double &dStart                        ///< [out]: Domain Space Span Interval start         <br>
  ) const ;

  void GetEnd   
  (
    const SmCurveCache & crCurveCache,     ///< [in] : owner of this span                      <br>
    SmPoint3d &rEnd,                       ///< [out]: Image Space End Point                   <br>
    double &dEnd                           ///< [out]: Domain Space Span Interval end          <br>
  )   const ;

  void GetEnds  
  (
    const SmCurveCache & crCurveCache,      ///< [in] : owner of this span                      <br>
    SmPoint3d    & rStart,                  ///< [out]: Image Space Start Point                 <br>
    SmPoint3d    & rEnd,                    ///< [out]: Image Space End Point                   <br>
    SmExtent1d   & rIvl                     ///< [out]: Domain Space Span Interval              <br>
  ) const;

  void          GetDegree(const SmCurveCache & crCurveCache) const ;

  void          Dump     (const SmCurveCache &crCurveCache, int lDepth=0) const ;

  // no virtual functions allowed by memory block manager

} ; // end class SmBezierAux1d

/*******************************************************************//**
PURPOSE: This object contains all the data stored for the leaf
            nodes of a curve spatial tree decomposition.

NOTES: 
***********************************************************************/ 
class SM_EXPORT SmBezierSpan : public SmBezierAux1d
{

public:

  SmPseudoBox    m_sPseudoBox;                // A tight bounding box - parallapiped

  SmPseudoBox & GetPseudoBox()                { return m_sPseudoBox; }

  // no virtual functions allowed by memory block manager

} ; // end class SmBezierSpan

/*******************************************************************//**
PURPOSE: The curve cache object contains a Bezier decomposition of 
    a curve.  The decomposition element is an SmBezierSpan.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmCurveCache : public SmCacheObj
{
    friend class SmTree;
private:                                                     
    SmTree                     * m_pTree;                    // binary decomposition tree
                                                             //   node memory stored in m_pTree::m_sNodeMgr.     
    const SmCurve              & m_crCurve;                  // owner curve
                                                             //  note: m_crCurve may be any type of SmCurve, however
                                                             //        m_crCurve is derived from SmBSplineCurve if m_pTree != NULL     
    SmExtent1d                   m_sInterval;                // Interval covered by m_pTree     
    SmTArray<SmContinuityType> * m_pContinuities;            // continuity value for every continuity boundary in m_crCurve

    SmBoolean                    m_bTriedToCreateAnalytical; // TRUE = tried (may or may not have succeeded) to make analytic    
    SmCurve                    * m_pAnalyticalCurve;         // when possible, analytic rep for this curve  
       
    ULONG                        m_lBezierSize;              // size of one bezier gw_CURVE Nurb curve sub-division span, set in constructor   
    ULONG                        m_lTotalBezSize;            // size of SmBezierSpan (cached gw_CURVE properties),        set in constructor 

    SmMemBlockMgr                m_sBezMgr;                  // sub-division tree leaf-node   data memory (SmBezierSpan objects), one SmBezierSpan per Curve sub-division span allocated in Tessellate().
    SmMemBlockMgr                m_sParentMgr;               // sub-division tree parent-node data memory (SmBezierAux1d objects), one SmBezierAux1d per node, only allocated when m_bComputeAuxTreeData == TRUE.
                                                             //    allocated in BuildTree().

    SmBoolean                    m_bComputeAuxTreeData;      // in SmCurveCache::BuildTree
                                                             //   TRUE = Compute Parent Properties from children properties after tree subdivision
                                                             //          based on initial knot spans and subsequent tessellation properties is complete.
                                                             //   FALSE= don't

    SmBoolean                    m_bMakeFullTree;            // when m_bComputeAuxTreeData == TRUE
                                                             // in SmCurveCache::BuildTree
                                                             //   TRUE = Save every child and internal subdivision node.
                                                             //   FALSE= Simplify tree when possible. 
                                                             //          When parents pass tessellation tests, remove children pointers to make 
                                                             //             parent a leaf node.


    double                       m_dChordHeightTolerance;    // max leaf node control-polygon vertex to baseLine distance      - 0 = ignore     
    double                       m_dAngleToleranceDeg;       // max leaf node control-polygon accumulated vertex angle value   - 0 = ignore    
    double                       m_dOffAxisTolerance;        // max leaf node control-polygon bbox 2nd largest dimension       - 0 = ignore
                                                             //     of nonlinear segments, or linear segments not aligned with an x, y or z axis

    double                       m_dMax3DDistance;           // max LeafNode 3d size                                           - 0 = ignore     
    double                       m_dMinParamRatio;           // (Leaf node interval)/(total interval) limit, stops subdivision - 0 = ignore
    ULONG                        m_lMinSegNumber;            // min number of leaf nodes - 0=ignore                            - 0 = ignore

#ifdef SM_DEBUG_CACHE_H
    ULONG                        m_lCurveCacheCount ;        //      
#endif

public:
    // constructor - builds cache object without tessellation
    SmCurveCache
    (
      const SmCurve    & crCurve,                            ///< [in] : target curve                                                             <br>
      const SmExtent1d & crInterval,                         ///< [in] : target interval                                                          <br>
      SmBoolean          bMakeFullTree            = FALSE,   ///< [in] : only used when bComputeAuxTreeData == TRUE                               <br>
                                                             ///<      : FALSE = Simplify Tree where Parent Node's pass all tessellation tests    <br>
                                                             ///<      : TRUE  = don't                                                            <br>
      double             dChordHeightTolerance    = 0.0,     ///< [in] : max leafNode control-polygon vertex to baseline distance, 0 = ignore     <br>
      double             dAngleTolDeg             = 0.0,     ///< [in] : max leafNode control-polygon vertex angle sum,            0 = ignore     <br>
      double             dOffAxisTolerance        = 0.0,     ///< [in] : max leafNode off axis control-polygon BBox size,          0 = ignore     <br>
                                                             ///<      : subdivides leaves into axis aligned near-linear segments.                <br>
      ULONG              lMinimumNumberOfSegments = 0,       ///< [in] : limits max element size, 0 = ignore                                      <br>
      SmBoolean          bComputeAuxTreeData      = FALSE,   ///< [in] : TRUE = propagate child properties up to parent nodes.                    <br>
                                                             ///<      : FALSE= don't                                                             <br>
      double             dMax3DDistance           = 0.0,     ///< [in] : max leafNode 3d control-polygon baseline size,  0 = ignore               <br>
      double             dMinimumParametricRatio  = 0.0001   ///< [in] : min (leafNode Interval)/(Curve Interval) ratio, 0 = ignore               <br>
    );            

    // call immediately after construction to build subdivision tree for SmBSplineCurves
    SmStatus Tessellate();

    // destructor
    virtual ~SmCurveCache();

    // simple data access
    ULONG GetSpanCount() const
    { 
      return m_sBezMgr.GetNumActiveElements();    // will be zero if Curve is not derived from SmBSplineCurve
    }
                                                 
    SmBezierSpan * GetAt( ULONG lIndex ) const
    {
      SM_ASSERT( lIndex < GetSpanCount() );
      return (SmBezierSpan*)((SmCurveCache*)this)->m_sBezMgr.GetAt( lIndex );
    }
                                                 
    SmBezierAux1d * GetParentAt( ULONG lIndex )
    {
      SM_ASSERT( lIndex < m_sParentMgr.GetNumActiveElements() );
      return (SmBezierAux1d*)m_sParentMgr.GetAt( lIndex );
    }

    SmStatus GetTessellation
    (
      SmTArray<double>    *pParameters=NULL,      ///< [in] :
      SmTArray<SmPoint3d> *pPoints    =NULL       ///< [in] :
    );

    SmTree        * GetTree()         const { return m_pTree; }

    const SmCurve * GetAnalyticCurve();

    SmBrep        * GetBrep()         const ;
    const SmCurve * GetCurve()        const { return( &m_crCurve) ; }

    // return m_crCurve cast to SmBSpline or NULL when Curve is not derived from SmBSPlineCurve
    const SmBSplineCurve * GetBSplineCurve() const ;

    const SmTArray<SmContinuityType> & GetContinuitiesArray()   const { return *m_pContinuities; }

    // internals
    SmStatus BuildTree(SmTArray<gw_CURVE *> &rLeafBeziers);

    gw_CURVE *OutputSegment
    (
      gw_CURVE        *pBezCurve,               ///< [in] : bezier curve to output as a segment - in working or scratch memory   <br>
      SmMemBlockMgr   *pSegmentBeziersMgr,      ///< [in] : working memory, needed when pBezCurve is in scratch memory           <br>
      double           dMaxLength,              ///< [in] : baseline length of curve's control-polygon                           <br>
      double           dMaxChordHeightSquared,  ///< [in] : max vertex to baseline length of curve's control-polygon             <br>
      double           dMaxTurningAngle,        ///< [in] : sum of vertex angles for curve's control-polygon                     <br>
      SmContinuityType eLeftContinuity,         ///< [in] : continuity to left neighbor span                                     <br>
      SmContinuityType eRightContinuity         ///< [in] : continuity to right neighbor shan                                    <br>
    );

    void     SetComputeAuxTreeData(SmBoolean bComputeAuxData)    { m_bComputeAuxTreeData = bComputeAuxData; }

    ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const ;
    virtual void Draw(void) const;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmCurveCache,SmObject,SmCurveCache_TYPE);

} ; // end class SmCurveCache

#endif // !__SMCURVECACHE_H__


