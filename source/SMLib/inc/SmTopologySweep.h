// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTopologySweeep.h
* PURPOSE   --- Header file for the class. 
**********************************************************************/

#ifndef __SMTOPOLOGYSWEEP_H__
#define __SMTOPOLOGYSWEEP_H__


#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMMAPPTRTOPTR_H__
#include <SmMapPtrToPtr.h>
#endif
   // member

#ifndef __SMSWEEPGEOMETRYCREATION_H__
#include <SmSweepGeometryCreation.h>
#endif

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifndef __SMSTITCH_H__
#include <SmStitch.h>
#endif



/*******************************************************************//**
PURPOSE: This object is the high level interface to Non-manifold 
    sweeping.  To use this object to create a new sweep type, you can 
    create a subclass of SmSweepGeometryCreation that creates the corresponding
    geometry and put it into the constructor for this operation. 

NOTES: > It is assumed that the input is a valid Brep that
                  has no geometric coincidences of separate topological items.
                  If this is not true, it may get corrected during the stitching
                  phase of the sweep, and the maps of the sweep may not 
                  be correct.
                  
                  Please note that the above must hold for the given sweep
                  tolerance, thus it must be selected in such a way that
                  this assumption holds.
***********************************************************************/
class SM_EXPORT SmTopologySweep : public SmObject
{
private:
    SmSweepGeometryCreation * m_pGeometryCreation = NULL ; // subclassed: currently linear or rotational sweeps
                                                           //   Note: not owned, not deleted on destruction.
    SmBoolean m_bDoMerge  = FALSE ;                        // TRUE = Do Piecewise merge after sweep - expensive but finds self intersections
    SmBoolean m_bDoStitching = FALSE ;                        // TRUE = if(m_bDoMerge == FALSE) Stitch Brep after sweep is complete - glues coincident geometry
                                                           
    SmBoolean m_bDoTagging = FALSE ;                       // TRUE = Add an SmTagAttribute to every face and edge
    long      m_lTagID = SM_UNDEF_ULONG ;                  // m_lPrimaryID value stored with every sweep geometry tag
                                                           
    ULONG     m_nRepetitions = SM_UNDEF_ULONG ;            // Repeate sweep number of times
                                                           
    SmBoolean m_bDoSelfIntersectionCheck = FALSE ;         // TRUE = Not used - if needed try m_bDoMerge = TRUE
                             

    SmTopologySweep() { SE(SM_ERR); }  // Do not call constructor with no arguments.

public:
    // constructor
    SmTopologySweep
    (
      SmSweepGeometryCreation & rGeometryCreation, ///< [in ]: currently either linear or rotation sweep                                                             <br>
      SmBoolean bDoStitching           = TRUE,        ///< [in ]: TRUE = stitch geometry after sweep to glue coincident edges                                              <br>
                                                   ///<      : not used when bDoMerge is TRUE                                                                        <br>
      SmBoolean bClosedSweep        = FALSE,       ///< [in ]: not used                                                                                              <br>
      SmBoolean bDoSelfIntersection = FALSE,       ///< [in ]: not used; if needed try bDoMerge = TRUE                                                               <br>
      SmBoolean bDoMerge            = FALSE,       ///< [in ]: TRUE = piecewise merge geometry after sweep, corrects for self-intersections and coincident geometry  <br>
      SmBoolean bDoTagging          = FALSE,       ///< [in ]: TRUE = add tags to faces and edges                                                                    <br>
      long      lTagID              = 0            ///< [in ]: primaryID stored with every face and edge tag                                                         <br>
    );

    ~SmTopologySweep() {} // Does not delete m_pGeometryCreation.


    // top level interface to sweep operator
    // If all six array arguments are Null, sweep the whole Brep.
    SmStatus DoSweep
    (
      SmBrep              * pBrepToSweep,                     ///< [in ]: target Brep with topology to sweep, always required                             <br>
      SmRegion            * pRegionToSweepInto,               ///< [in ]: Region to contain sweep geometry. Its Brep can be BrepToSweep or other Brep,    <br>
                                                              ///<      : When NULL, swept geometry is merged into pBrepToSweep.                          <br>
      SmTArray<SmFace*>   * pOptFacesToSweepHigher   =NULL,   ///< [in ]: sweep these faces into solids, NULL to ingore                                   <br>
      SmTArray<SmEdge*>   * pOptEdgesToSweepHigher   =NULL,   ///< [in ]: sweep these edges into faces, NULL to ignore                                    <br>
      SmTArray<SmVertex*> * pOptVerticesToSweepHigher=NULL,   ///< [in ]: sweep these vertices into edges, NULL to ignore                                 <br>
      SmTArray<SmFace*>   * pOptFacesToSweepSame     =NULL,   ///< [in ]: copy these faces to their swept position, NULL to ignore                        <br>
      SmTArray<SmEdge*>   * pOptEdgesToSweepSame     =NULL,   ///< [in ]: copy these edges to their swept position, NULL to ignore                        <br>
      SmTArray<SmVertex*> * pOptVerticesToSweepSame  =NULL    ///< [in ]: copy these vertices to their swept position, NULL to ignore                     <br>
    );

    // note: increments unlocked mark value
    // Whole-Brep envelope sweep
    SmStatus DoAdvSweep
    (
      SmBrep   * pBrepToSweep,
      SmRegion * pRegionToSweepInto
    );

    // Data access
    SmBoolean GetDoMerge()         { return(m_bDoMerge); }    

    SmBoolean GetDoStitching()        { return(m_bDoStitching) ; }

    ULONG GetRepetitions()         { return(m_nRepetitions) ; }

    SmBoolean GetTagging( long &lTagID )
    {
      lTagID = m_lTagID;
      return(m_bDoTagging);
    }

    const SmSweepGeometryCreation* GetSweepGeometryCreation() const
    {
      return m_pGeometryCreation;
    }

    void SetDoMerge(SmBoolean bDoMerge)                { m_bDoMerge = bDoMerge; }  

    void SetDoStitching(SmBoolean bDoStitching)              { m_bDoStitching = bDoStitching; }

    void SetRepetitions(ULONG nRepetitions) ;

    void SetTagging( SmBoolean bDoTagging, long lTagID )
    {
      m_bDoTagging = bDoTagging;
      m_lTagID = lTagID;
    }

protected:
    // create copied vertex at sweep destination
    SmStatus DoSweepVerticesSame
    (
      SmRegion* pRegion,                                     ///< [in ]:    <br>
      const SmTArray<SmVertex*> & crVerticesToSweepSame      ///< [in ]:    <br>
    );

    // create sweep vertex and edge
    SmStatus DoSweepVerticesHigher
    (
      SmRegion* pRegion,                                      ///< [in ]:     <br>
      const SmTArray<SmVertex*> & crVerticesToSweepHigher     ///< [in ]:     <br>
    );

    // create copied edge at sweep destination
    SmStatus DoSweepEdgesSame
    (
      SmRegion* pRegion,
      const SmTArray<SmEdge*> & crEdgesToSweepSame
    );

    // create sweep edge and face (and sweep edge-end vertices if not already done)
    SmStatus DoSweepEdgesHigher
    (
      SmRegion* pRegion,                                ///< [in ]:   <br>
      const SmTArray<SmEdge*> & crEdgesToSweepHigher    ///< [in ]:   <br>
    );

    // create copied face at sweep destination
    SmStatus DoSweepFacesSame
    (
      SmRegion* pRegion,
      const SmTArray<SmFace*> & crFacesToSweepSame
    );

    // create copied face at sweep destination and sweep boundary edges and vertices if not already done)
    SmStatus DoSweepFacesHigher
    (
      SmRegion* pRegion,
      const SmTArray<SmFace*> & crFacesToSweepHighe);

    // copy face at sweep destination
    SmStatus SweepFaceSame
    (
      SmRegion* pRegion,                 ///< [in ]: Region to contain new topology objects   <br>
      SmFace * pFaceArg                  ///< [in ]: Face to be swept                         <br>
    );

    // copy vertex at sweep destination
    SmStatus SweepVertexSame
    (
      SmRegion* pRegionArg,
      SmVertex* pVertexArg, 
      SmVertex*& pSweptVertexArg
    );

    // copy edge at swepp destination
    SmStatus SweepEdgeSame
    (
      SmRegion  *  pRegionArg,
      SmEdgeuse *  pEUArg,
      SmEdgeuse *& rpSweptEdgeuseArg,
      SmEdge    *& rpExistingSweptEdgeArg        ///< [out]: Will be set only if there already exists a swept edge for this       <br>
                                                 ///<      : original edge (pEUArg->GetEdge()) -- otherwise will be set to NULL   <br>
    );

    //
    SmStatus SweepEdgeHigh
    (
      SmRegion*  pRegionArg,                    ///< [in ]: region to contain new topology objects                   <br>
      SmEdge  *  pEdgeArg,                      ///< [in ]: Edge to sweep into a face                                <br>
      SmFace  *& rpNewFaceArg,                  ///< [out]: the few face connected to following boundary topology    <br>
      SmEdge  *& rpCopiedEdgeArg,               ///< [out]: the copy of the original edge                            <br>
      SmEdge  *& rpNewStartEdgeArg,             ///< [out]: the new edge emanating from the start vtx                <br>
      SmEdge  *& rpNewEndEdgeArg,               ///< [out]: the new edge emanating from the end vtx                  <br>
      SmEdge  *& rpNewEdgeArg,                  ///< [out]: copy of original edge swept to final position            <br>
      SmVertex*& rpSweptStartVtxArg,            ///< [out]: copy of original start vertex swept to final position    <br>
      SmVertex*& rpSweptEndVtxArg               ///< [out]: copy of original end vertex swept to final position      <br>
    );

    // add tags to faces and edges, copy edge->curve tags if possible, vertices not currently tagged
    SmStatus TagProfile
    (
      SmTArray<SmFace*>   * pOptFacesHigher,      ///< [in ]: Faces and their edges to tag        <br>
      SmTArray<SmEdge*>   * pOptEdgesHigher,      ///< [in ]: Edges to tag                        <br>
      SmTArray<SmVertex*> * pOptVerticesHigher,   ///< NotUsed: [in ]: pOptVerticesHigher                            <br>
      SmTArray<SmFace*>   * pOptFacesSame,        ///< [in ]: more Faces and their edges to tag   <br>
      SmTArray<SmEdge*>   * pOptEdgesSame,        ///< [in ]: more Edges to tag                   <br>
      SmTArray<SmVertex*> * pOptVerticesSame      ///< NotUsed: [in ]: pOptVerticesSame                            <br>
    );

    // map relationship from original topology objects to swept/copied topology objects
    SmMapPtrToPtr<SmTopology, SmTopology>  m_vMapToSame;       // map orig vertices, edges, and faces to sweptCopied vertices, edges, and faces
    SmMapPtrToPtr<SmTopology, SmTopology>  m_vMapToHigher;     // map orig vertices to swept edges and orig edges to swept faces
    SmMapPtrToPtr<SmTopology, SmTopology>  m_vMapFromSame;     // inverse of m_vMapToSame
    SmMapPtrToPtr<SmTopology, SmTopology>  m_vMapFromLower;    // inverse of m_vMapToHigher

public:

    SmMapPtrToPtr<SmTopology, SmTopology>* GetMapToSame()    { return & m_vMapToSame;    }
    SmMapPtrToPtr<SmTopology, SmTopology>* GetMapToHigher()  { return & m_vMapToHigher;  }
    SmMapPtrToPtr<SmTopology, SmTopology>* GetMapFromSame()  { return & m_vMapFromSame;  }
    SmMapPtrToPtr<SmTopology, SmTopology>* GetMapFromLower() { return & m_vMapFromLower; }

    SmTopology* GetToSamePriv(SmTopology* pFrom)    { return m_vMapToSame.At(pFrom);    }
    SmTopology* GetFromSamePriv(SmTopology* pFrom)  { return m_vMapFromSame.At(pFrom);  }
    SmTopology* GetToHigherPriv(SmTopology* pFrom)  { return m_vMapToHigher.At(pFrom);  }
    SmTopology* GetFromLowerPriv(SmTopology* pFrom) { return m_vMapFromLower.At(pFrom); }

    void RemoveFromToSamePriv(SmTopology* pFrom)   { m_vMapToSame.Remove(pFrom);   }
    void RemoveFromFromSamePriv(SmTopology* pFrom) { m_vMapFromSame.Remove(pFrom); }

    void SetSame(SmTopology *pFrom, SmTopology *pTo);
    SmVertex * GetToSame(SmVertex *pFrom) { return SM_REINTERPRET_CAST(SmVertex*,GetToSamePriv(pFrom)); }
    SmEdge   * GetToSame(SmEdge *pFrom)   { return SM_REINTERPRET_CAST(SmEdge*,GetToSamePriv(pFrom)); }
    SmFace   * GetToSame(SmFace *pFrom)   { return SM_REINTERPRET_CAST(SmFace*,GetToSamePriv(pFrom)); }

    SmVertex * GetFromSame(SmVertex *pTo) { return SM_REINTERPRET_CAST(SmVertex*,GetFromSamePriv(pTo)); }
    SmEdge   * GetFromSame(SmEdge *pTo)   { return SM_REINTERPRET_CAST(SmEdge*,GetFromSamePriv(pTo)); }
    SmFace   * GetFromSame(SmFace *pTo)   { return SM_REINTERPRET_CAST(SmFace*,GetFromSamePriv(pTo)); }
    
    void SetHigher(SmTopology *pFrom, SmTopology *pTo);

    SmEdge   * GetToHigher(SmVertex *pFrom) { return SM_REINTERPRET_CAST(SmEdge*,GetToHigherPriv(pFrom)); }
    SmFace   * GetToHigher(SmEdge *pFrom)   { return SM_REINTERPRET_CAST(SmFace*,GetToHigherPriv(pFrom)); }
    SmRegion * GetToHigher(SmFace *pFrom)   { return SM_REINTERPRET_CAST(SmRegion*,GetToHigherPriv(pFrom)); }

    SmVertex * GetFromLower(SmEdge *pTo)    { return SM_REINTERPRET_CAST(SmVertex*,GetFromLowerPriv(pTo)); }
    SmEdge   * GetFromLower(SmFace *pTo)    { return SM_REINTERPRET_CAST(SmEdge*,GetFromLowerPriv(pTo)); }
    SmFace   * GetFromLower(SmRegion *pTo)  { return SM_REINTERPRET_CAST(SmFace*,GetFromLowerPriv(pTo)); }
    
} ; // end SmTopologySweep

/*******************************************************************//**
PURPOSE: This call back function decides which entities to keep
    and which to delete, when gluing during the Stitch operation.

NOTES: 
    Generally, keep the entities that are recorded in our entity maps.
***********************************************************************/
class SM_EXPORT SmSweepStitchCallback : public SmStitchCallback
{
protected:
  SmTopologySweep * m_pTopologySweep;

public:
  SmSweepStitchCallback(SmTopologySweep *pTopologySweep) : m_pTopologySweep( pTopologySweep )
                                                      { }
  virtual ~SmSweepStitchCallback() { }

  virtual SmStatus SelectTopologyToKeep(SmTopology * cpTopologyElement1,    ///< [in ]:
                                        SmTopology * cpTopologyElement2,    ///< [in ]:
                                        double       dDistBetween,          ///< NotUsed: [in ]: max dist between coincident objects          <br>
                                        double     & rdMaxDistance,         ///< [out]: max dist found between two target objects    <br>
                                        SmBoolean  & rbKeepElement1) ;      ///< [in ]: TRUE: keep Elem1, FALSE: keep Elem2.         <br>

} ; // end class SmSweepStitchCallback

#endif // !__SMTOPOLOGYSWEEP_H__

