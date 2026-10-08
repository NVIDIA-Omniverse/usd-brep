// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmProto.h
* PURPOSE: Header file for SmProto< topology > objects.
* Contains classes:
*   SmVertexDefinition  SmEdgeDefinition  SmProtoVertex  SmProtoEdge  SmProtoFace  SmProtoTopologyManager
**********************************************************************/

#ifndef __SMPROTOTOPO_H__
#define __SMPROTOTOPO_H__

#include <SmTopology.h>
#include <SmTArray.h>

class SmContext;

class SmProtoVertex;
class SmProtoEdge;
class SmProtoTopologyManager;

class SmTopologyIntersector;

/*******************************************************************//**
PURPOSE: Classes for ProtoTopology.

USAGE NOTES---- 
   1, This class hierarchy supports the IntersectInsertandRelate() (IIR()) portion of the Boolean algorithm implemented in the SmMerge class.
      The Boolean combines a pair of Breps for a variety of Boolean operations including: Intersect, Union, Merge, and Difference operations.  
   2. The two input Breps to the Boolean operation are called pBrep1 and pBrep2 in the ProtoTopology code.
   3. The IIR() portion of the Boolean operation in which all the topology objects of pBrep1 are intersected with 
      all the topology object of pBrep2 is the same for all the different Boolean operation types.
   4. The ProtoTopology idea is to organize the IIR() portion of the Boolean algorithm as:
        a. Compute and hold all the intersections found between all the topology object pairs in pBrep1 and pBrep without modifying the input Breps.
        b. Gather all the individual intersections that may be intended to represent the same intersection geometry into lists
             (For example three planes connected to one another with edges coming together to form a single corner of a cube will have 7 different
              intersections that define the corner's vertex:
                 1 FaceSurface/FaceSurface/FaceSurface intersection point
                 3 EdgeCurve/EdgeCurve intersection points
                 3 EdgeCurve/FaceSurface points
              The point locations for all 7 intersections may vary by tolerant amounts due to tolerances and numerical roundoff.  Making things even more complicated,
              it's possible this corner location happens to be near tolerance to another corner where the point positions of the 7 intersections that define the 2nd corner
              may spread out and overlap the 7 point locations of the 1st corner's intersections. The ProtoTopology idea is to gather all the intersections
              that may redundantly define the location of a new corner before analyzing those intersections to pick whether no, one, or more new Vertices are 
              needed and which location to give new Vertices to represent new corner(s) in the merged input Breps.  
              
              This idea differs from the previous implementation of the Boolean operator in which the location of a new corner vertex was
              set and inserted into the input Breps by the first intersection computed that defined its position and then depended 
              only on distance measures of the subsequent intersections to decide when an intersection was redundant or a unique new corner.
              The old algorithm mostly worked but often made mistakes when the intersections for one corner spread out over a distance larger then
              tolerance or tried to create two corners that were close enough that some of their intersection locations were within tolerance 
              of one another. Another problem the old implementation is that Merge(Brep1, Brep2) would not always equal Merge(Brep2, Brep1) because
              sometimes the different order of intersections made by the two calls could result in different first intersections creating 
              different output topology.  

   5. Each intersection between topology pairs of the input Breps is represented by one SmTopologyDefinition object.  
      Intersections that might be related to one another by distance or the connections between the topology objects are gathered together in
      a list of related SmTopologyDefinition objects that gets contained in the SmProtoTopology classes.
   5. The classes for ProtoTopology include:
        SmTopologyMgr
        SmProtoVertex with a list of SmVertexDefinitions representing a set of point intersections between topology pairs that are related 
                                                         by being within tolerance dist of one another or by sharing source topology objects
                                                         that connect to one another inb appropriate manners. The point intersections within a single
                                                         SmProtoVertex are most commonly analyzed to discover they all represent a single corner
                                                         and end up being combined to define a single Vertex that needs to be added
                                                         to represent a new corner in the output Brep. But other cases exist in which the analysis
                                                         of the SmProtoVertex's set of intersections concludes that no or multiple vertices should
                                                         be added to the input Breps.
        SmProtoEdge   with a list of SmEdgeDefinitions   representing a set of curve intersections between topology pairs of the input Breps that
                                                         are related by being within tolerance dist of one another or by sharing common connected
                                                         topology elements.  The curve intersections within a single SmProtoEdge commonly represent
                                                         a single edge that need to be added to the input Breps.  But other cases exist where
                                                         no or multiple edges end up being needed to be added to the input Breps. 
        SmProtoFace   with a list of SmFaceDefinitions   representing a set of Face/Face coincidences between Face pairs of the input Breps that
                                                         will end up being combined to define the Faces that need to be added
                                                         to the input Breps to enable completing the Boolean operation.
   6. Each ProtoTopology is analyzed by a Resolve() method to decide what topology is to be inserted into the input Breps as new topology objects
      from the information found within the list of intersections stored within the ProtoTopology object's TopologyDefinitions array. 
      (In the example of 3 planar faces coming together to form a corner, the SmProtoVertex's Resolve() method will decide that the 7 intersections that are 
      in that one ProtoVertex will result in adding one new Vertex to the input Breps.) The SmProtoTopology->Resolve() methods have been coded to handle many cases
      including simple geometries that intersect cleanly, geometry with seams, coincident geometry, and tangent geometry.  The Resolve() methods even
      have cases where the right thing to do is to create topology objects that are within tolerance of one another. 
        (For example when importing data from a system that hasn't yet stitched its geometry together, an edge between a pair 
         of faces may be represented by a pair of coincidnet lamina edges. If that kind of Brep is Booleaned with another its possible that 
         the two coincident edges end up needing a pair of coincident vertices to form a valid Brep topology graph where where the lamina edges 
         intersect another face to form a new corner.  Each new vertex is needed to consistently bound one of the coincident pair of lamina edge where they
         intersect a face to form a new corner. The output of this Boolean operation would be a valid Brep model that could be 
         sent to the SmBrep->Stitch() method which would find and glue all the coincident geometry into a simple manifold topology structure without 
         coincident vertices or lamina edges.)
   7. Once the SmProtoTopology->Resolve() functions are run, the IIR() portion of the Boolean operator is complete, the use of the ProtoTopology is done, and the Boolean
      operator moves on to its next phase placing the topology objects of the merged input Breps into the appropriate save and delete lists to use to generate
      the final output Brep for the various possible Boolean operations of Union, Intersect, Difference, Merge, and the like.

IMPLEMENTATION NOTES---- 
   Design ideas.
   - Derive from SmTopology.  That inherits:
      Context; 
      SmObjDelete (SmObject);
      Attributes (SmAObject);
      linked lists; 
      Marks;
      overloaded 'new()';
    That is a bit more convenient.

   Specific Design Decisions:
   'DD1' : For the ProtoVertex's list of ProtoEdge's: if the PE is closed,
           list it twice, even though they're the same.
   'DD2' : Change from VertexDef's and EdgeDef's pointing to each other,
           to VD -> PV and ED -> PE.  PE's and PV's point to each other.
   'DD3' : EdgeDefs on Edges: if two are on the same Edge, and intervals overlap,
           then they have to be dealt with: combined in some way.
           They should be EdgeDefs in the same ProtoEdge.
   'DD4' : In Imprint(), don't change the PCs' class/object to the final Brep topology,
           leave it as the original Brep topology.  We already store the final Brep
           topology in the SmVertexDefinition and SmEdgeDefinition objects.

cbi Todo:
   > cache ?

***********************************************************************/


/*******************************************************************//**
PURPOSE: Class for a Vertex Definition

USAGE NOTES---- 
   This class stores the data for one point intersection between a pair of topology
   objects from the input Breps of a Boolean operator call.

   The class helps enable the Boolean operator to compute and store all the 
   intersection results between its two input Breps before deciding which 
   intersections to imprint into the Boolean input Breps to generate the 
   Boolean output Brep.

   The class stores the intersection's position, XSectTol3d, and Gap3d values 
   and the two topology objects (one from each Brep) whose geometry intersected 
   to create the intersection point. If this object gets imprinted it will be 
   used to ensure that each Brep contains one of a mapped pair of vertices located at this 
   position and connected to the point intersection source topology objects.

   A ProtoVertex contains a list of one or more of these SmVertexDefinitions.
   When the ProtoVertex is Resolved, it decides which of its VertexDefinitions
   belong in the same final Brep Vertex and combines them.  After being Resolved,
   every VertexDefinition will become a SmVertex in each Brep, even if the SmVertexes
   are coincident.  This can happen for example when a pair of unstitched, coincident
   lamina Edges in one Brep intersect a Face in the other Brep: that situation must
   create coincident SmVertexes.

   A SmVertexDefinition is always contained in a ProtoVertex.
   To create a VertexDefinition, call SmProtoTopologyManager::FindOrCreateProtoVertex().
   That will first check to see whether a ProtoVertex already exists at
   that location, and if so, will add the VertexDefinition to that ProtoVertex,
   and if not, will create a new ProtoVertex with that VertexDef.
***********************************************************************/
class SM_EXPORT SmVertexDefinition : public SmTopology
{
 protected:
  SmPointClassification   m_sPC1 ;               // Description of Brep1 topo and geom at this point XSection
  SmPointClassification   m_sPC2 ;               // Description of Brep2 topo and geom at this point XSection
  // in SmPointClassifications :                 
  // SmTopology         * m_pSrcTopo1            // ( == m_sPC1.GetObject() )  = source Vertex, Edge, or Face in brep1 for this XSection
  // SmTopology         * m_pSrcTopo2            // ( == m_sPC2.GetObject() )  = source Vertex, Edge, or Face in brep2 for this XSection
  // double               m_dBrep1T,  m_dBrep2T  // ( == m_sPC*.GetTParam() )  = assoc Params on source Edges of this XSection
  // SmPoint2d            m_sBrep1UV, m_sBrep2UV // ( == m_sPC*.GetUVParam() ) = assoc Params on source Faces of this XSection
                                                 
  SmPoint3d               m_sPosition ;          // this XSection 3d Position
  double                  m_dTol ;               // this XSection XSectTol3d (soursce topo objects are within this dist to one another)
  double                  m_dGap ;               // this XSecgtion Gap3d between source objects: less than SM_EFF_ZERO = Exact XSect, greater than SM_EFF_ZERO = Tolerant XSect
                                                 
  SmProtoVertex         * m_pMyProtoVertex ;     // Back-pointer to owning ProtoVertex
                                                 
  SmVertex              * m_pFinalVtx1 ;         // Vertex in brep1 for this XSection after resolve and imprint
  SmVertex              * m_pFinalVtx2 ;         // Vertex in brep2 for this XSection after resolve and imprint

public:
  // Constructor, Destructor:
  SmVertexDefinition( const SmContext    * pCtx, 
                      const SmPointClass * cpPC1,
                      const SmPointClass * cpPC2,
                      const SmPoint3d    & rPos,
                      double               dTol,
                      double               dGap = -1.0,
                      SmProtoVertex      * pOwner = NULL) ;

  ~SmVertexDefinition();

  // Note: in the following methods the argument iWhichBrep is either 1 or 2.

  // Simple Access
  SmPoint3d                 & GetPosition            ()                     { return m_sPosition; }
  SmZoneTol3d                 GetTolerance           ()               const { return m_dTol; }
  double                      GetGap()                                const { return m_dGap; }
                                                     
  SmVertex                  * GetFinalVertex         (int iWhichBrep) const { return ( iWhichBrep==1 ) ? m_pFinalVtx1 : m_pFinalVtx2; }

  ULONG                       GetNumProtoEdges       ()               const ;
  SmProtoVertex             * GetProtoVertex         ()               const { return m_pMyProtoVertex; }
  SmProtoTopologyManager    * GetProtoTopologyManager()               const ;

  SmTopology                * GetBrep1Topo           ()               const ;
  SmPointClassificationType   GetBrep1TopoType       ()               const { return m_sPC1.GetPointClass(); }

  SmTopology                * GetBrep2Topo           ()               const ;
  SmPointClassificationType   GetBrep2TopoType       ()               const { return m_sPC2.GetPointClass(); }

  SmTopology                * GetSrcTopo            (int iWhichBrep) const { return ( iWhichBrep==1 ) ? GetBrep1Topo() : GetBrep2Topo(); }
  SmPointClassificationType   GetSrcTopo_TYPE        (int iWhichBrep) const { return ( iWhichBrep==1 ) ? GetBrep1TopoType() : GetBrep2TopoType(); }

  double                      GetBrepTParam          (int iWhichBrep) const { return ( iWhichBrep==1 ) ? m_sPC1.GetTParam() : m_sPC2.GetTParam(); }
  SmPoint2d                   GetBrepUVParam         (int iWhichBrep) const { return ( iWhichBrep==1 ) ? m_sPC1.GetUVParam() : m_sPC2.GetUVParam(); }
  SmPointClassification     * GetBrepPointClassif    (int iWhichBrep)       { return ( iWhichBrep==1 ) ? &m_sPC1 : &m_sPC2; }
                           
  void     SetPosition       (SmPoint3d      sPosition)           { m_sPosition = sPosition ; }
  void     SetProtoVertex    (SmProtoVertex *pPV)                 { m_pMyProtoVertex = pPV; }
  SmStatus SetPointParameters() ;
  SmStatus SetSrcTopo        (int iWhichBrep, 
                              SmPointClassificationType eType, 
                              SmTopology *pTopo) ;
  SmStatus SetFinalVertex    (int iWhichBrep, SmVertex *pVertex ) { (( iWhichBrep==1 ) ? m_pFinalVtx1 : m_pFinalVtx2 ) = pVertex; 
                                                                    return SM_SUCCESS ; 
                                                                  }

  // Predicates
  double DistanceTo( const SmPoint3d &crPosition ) const  { return (m_sPosition.DistanceBetween( crPosition )) ; }

  // Working methods:
// DD2:  void     AddEdgeDefinition( SmEdgeDefinition *pED) ;
// DD2:  SmStatus RemoveEdgeDefinition( SmEdgeDefinition *pED) ;

  SmStatus Resolve(int & riStatus) ;
  SmStatus Imprint() ;

  // utilities
  void         Dump(int iDumpLevel, ULONG * lIdx = NULL ) const ;
  virtual void Dump() const { Dump( -1, NULL) ; }
  void         Draw() ;

} ; // end class SmVertexDefinition

/*******************************************************************//**
PURPOSE: Class for an EdgeDefinition

USAGE NOTES---- 
   This class stores the data for one curve intersection between a pair of topology
   objects from the input Breps of a Boolean operator call.

   The class helps enable the Boolean operator to compute and store all the 
   intersection results between its two input Breps before deciding which 
   intersections to imprint into the Boolean input Breps to generate the 
   Boolean output Brep.

   This class stores the intersection curve's Curve3d, UVTrimCurves, intervals 
   and XSectTol3d values and the two topology objects (one from each Brep) 
   whose geometry intersected to create the intersection curve.
   
   A ProtoEdge contains a list of one or more of these SmEdgeDefinitions.
   When the ProtoEdge is Resolved, it decides which of its EdgeDefinitions
   belong in the same final Brep Edge and combines them.  After being Resolved,
   every EdgeDefinition will become a SmEdge in each Brep, even if the SmEdges
   are coincident.  This can happen for example when a pair of unstitched, coincident
   lamina Edges in one Brep intersect a Face in the other Brep: that situation must
   create coincident SmEdgess.

   When an SmEdgeDefinition is defined by a pair of Faces, the curve is the
   intersection curve returned from FaceIntersect() in the CurveClass objects. 
   If it is defined by a pair of Edges or a Face and an Edge, the Edge is 
   coincident with an existing edge in one or both of the input Breps. That 
   intersection curve is not the same curve that's in the SmEdge, it's a copy.
***********************************************************************/
class SM_EXPORT SmEdgeDefinition : public SmTopology
{
 protected:
  SmCurve      * m_pXSectCurve3d ;   // the 3d intersection curve
  SmCurve      * m_pXSectUVCurve1 ;  // associated UVTrimCurve Curve on a Face in pBrep1 or NULL
  SmCurve      * m_pXSectUVCurve2 ;  // associated UVTrimCurve Curve on a Face in pBrep2 or NULL

  SmExtent1d     m_pXSectInterval ;  // XSectCurve3d domain interval
                                     // note: For the case of partially coincident SmEdges forming
                                     //       a XSectCurve3d, this interval might not 
                                     //       match the originating SmEdges' domain intervals.
                 
  SmTopology   * m_pSrcTopo1 ;       // source XSecting Topology object in pBrep1. Usually SmFace but could be SmEdge.
  SmTopology   * m_pSrcTopo2 ;       // source XSecting Topology object in pBrep2. Usually SmFace but could be SmEdge.

  SM_TYPE        m_pSrcTopo1_TYPE ;  // The topo type of m_pSrcTopo1. OneOf: SmFace_TYPE or SmEdge_TYPE. Cached because m_pSrcTopo1 may get deleted during Imprint. Used in the destructor.
  SM_TYPE        m_pSrcTopo2_TYPE ;  // The topo type of m_pSrcTopo2. OneOf: SmFace_TYPE or SmEdge_TYPE. Cached because m_pSrcTopo2 may get deleted during Imprint. Used in the destructor.

  SmProtoEdge  * m_pProtoEdge ;    // Back-pointer to the owning ProtoEdge.

  double         m_dTol;             // most likely the XSectTol3d value used for this intersection

  SmEdge       * m_pFinalEdge1 ;     // The resulting Brep1->SmEdge created after Resolve() and Imprint().
  SmEdge       * m_pFinalEdge2 ;     // The resulting Brep2->SmEdge created after Resolve() and Imprint()

 public:
  // Constructor
  SmEdgeDefinition(SmCurve     * pXSectCurve3d,   // in : Intersection 3d curve
                   SmExtent1d  & pXSectInterval,  // in : XSectCurve3d domain interval
                   SmTopology  * pSrcTopo1,       // in : source intersecting pBrep1 topology object (an SmFace or SmEdge)
                   SmTopology  * pSrcTopo2,       // in : source intersecting pBrep1 topology object (an SmFace or SmEdge)
                   double        dTol,            // in : probably the SmXSectTol3d value used by the intersector to generate this intersection curve
                   SmCurve     * pXSectUVCurve1,  // in : associated UVTrimCurve on a SmFace in pBrep1, or NULL
                   SmCurve     * pXSectUVCurve2,  // in : associated UVTrimCurve on a SmFace in pBrep2, or NULL
                   SmProtoEdge * pMyProtoEdge )   // in : Back pointer to the SmProtoEdge that owns this SmEdgeDefinition
                 : m_pXSectCurve3d ( pXSectCurve3d ),
                   m_pXSectUVCurve1( pXSectUVCurve1 ),
                   m_pXSectUVCurve2( pXSectUVCurve2 ),
                   m_pXSectInterval( pXSectInterval ),
                   m_pSrcTopo1     ( pSrcTopo1 ),
                   m_pSrcTopo2     ( pSrcTopo2 ),
                   m_pProtoEdge  ( pMyProtoEdge ),
                   m_dTol          ( dTol )
                 {
                   m_pSrcTopo1_TYPE = pSrcTopo1->GetType();
                   m_pSrcTopo2_TYPE = pSrcTopo2->GetType();
                 }

  // copy constructor
  SmEdgeDefinition( const SmEdgeDefinition &crObjToCopy )
                  : m_pXSectCurve3d ( crObjToCopy.m_pXSectCurve3d ),
                    m_pXSectUVCurve1( crObjToCopy.m_pXSectUVCurve1 ),
                    m_pXSectUVCurve2( crObjToCopy.m_pXSectUVCurve2 ),
                    m_pXSectInterval( crObjToCopy.m_pXSectInterval ),
                    m_pSrcTopo1     ( crObjToCopy.m_pSrcTopo1 ),
                    m_pSrcTopo2     ( crObjToCopy.m_pSrcTopo2 ),
                    m_pSrcTopo1_TYPE( crObjToCopy.m_pSrcTopo1_TYPE),
                    m_pSrcTopo2_TYPE( crObjToCopy.m_pSrcTopo2_TYPE),
                    m_pProtoEdge  ( crObjToCopy.m_pProtoEdge ),
                    m_dTol          ( crObjToCopy.m_dTol )
                  { }

  // destructor. The destructor does not delete the geometry because he geometry is normally consumed when imprinting.
  ~SmEdgeDefinition() ;

  // Delete contained geometry when appropriate. Needed because the destructor does not delete the geometry.
  SmStatus Destruct();

  // Simple Data Access
  SmCurve                   * GetXSectCurve3d        ()               const { return m_pXSectCurve3d; }
  SmExtent1d                  GetXSectInterval       ()               const { return m_pXSectInterval; }
  SmZoneTol3d                 GetTolerance           ()               const { return m_dTol; }
  SmCurve                   * GetXSectUVCurve        (int iWhichBrep) const { return ( iWhichBrep==1 ) ? m_pXSectUVCurve1 : m_pXSectUVCurve2; }
  SmTopology                * GetSrcTopo             (int iWhichBrep) const { return ( iWhichBrep==1 ) ? m_pSrcTopo1 : m_pSrcTopo2; }
  SmPointClassificationType   GetSrcTopo_TYPE        (int iWhichBrep) const ;
  SmProtoVertex             * GetStartProtoVertex    ()               const ;
  SmProtoVertex             * GetEndProtoVertex      ()               const ;
  SmProtoEdge               * GetProtoEdge           ()               const { return m_pProtoEdge; }
  SmEdge                    * GetFinalEdge           (int iWhichBrep) const { return ( iWhichBrep==1 ) ? m_pFinalEdge1 : m_pFinalEdge2; }
  SmProtoTopologyManager    * GetProtoTopologyManager()               const ;
  SmEdge                    * GetStartBrepEdge       (int iWhichBrep) const ; // Get Brep topology object from the start ProtoVertex if present.
  SmEdge                    * GetEndBrepEdge         (int iWhichBrep) const ; // Get Brep topology object from the end   ProtoVertex if present.

  void SetXSectCurve3d ( SmCurve *pCrv )                                                             { m_pXSectCurve3d = pCrv; }
  void SetXSectCurveUV ( int iWhichBrep, SmCurve *pCrv )                                             { ( iWhichBrep==1 ? m_pXSectUVCurve1 : m_pXSectUVCurve2 ) = pCrv ; }
  void SetProtoEdge    ( SmProtoEdge * pProtoEdge )                                                  { m_pProtoEdge = pProtoEdge; }
  void SetXSectInterval( const SmExtent1d & crXSectInterval )                                        { m_pXSectInterval = crXSectInterval ; }
  void SetXSectInterval( double dMin, double dMax) ;                                                 // Allows reversal, if dMin > dMax
  void SetSrcTopo      ( int iWhichBrep, SmPointClassificationType /* eType */ , SmTopology *pTopo ) { (iWhichBrep==1 ? m_pSrcTopo1      : m_pSrcTopo2)      = pTopo ; 
                                                                                                       (iWhichBrep==1 ? m_pSrcTopo1_TYPE : m_pSrcTopo2_TYPE) = pTopo->GetType() ; 
                                                                                                     }

  // edit methods
  SmStatus Reparameterize( const SmExtent1d &rNewDomain, SmBoolean bReverse = FALSE) ;
  SmStatus ReverseOrientation();

  // "Evaluation" methods
  SmStatus GetEndPosition( SmBoolean      bAtStart, SmPoint3d & rPos ) const ; // rtn XSectCurve3d end Pos3d. bAtStart:[TRUE=Start3d, FALSE=end3d]
  SmStatus GetEndPosition( SmProtoVertex *pWhichPV, SmPoint3d & rPos ) const ; // rtn XSectCurve3d end Pos3d bounded by pWhichPV. Rtn SM_ERR when pWhichPV does not bound this ProtoEdge
  SmStatus Evaluate      ( double         dParam  , SmPoint3d * pPos=NULL, SmVector3d * pTan=NULL ) const ;
  SmStatus EvaluateMid   (                          SmPoint3d * pPos=NULL, SmVector3d * pTan=NULL ) const ;

  // Working methods:
  SmStatus Imprint() ;

  // Split a SmEdgeDefinition at the given parameter after splitting the owning ProtoEdge
  SmStatus Split( double                dT,        // in : Split parameter on ProtoEdge to be used to split this EdgeDefinition
                  SmProtoVertex       * pMidPV,    // in : pMidPV = ProtoVertex at ProtoEdge split point
                  SmProtoEdge         * pTopPE,    // in : pTopPE = new ProtoEdge for ProtoEdge UpperSplitChild
                  SmVertexDefinition *& rpNewVD,   // out: new VertexDefinition at Split point. Gets added to pMidPV
                  SmEdgeDefinition   *& rpNewED) ; // out: new Split UpperChild EdgeDefinition. Gets added to pTopPE
                                                   // note: this EdgeDefinition gets reused as Split LowerChild EdgeDefinition
                                                   // 
  // return true when XSecting Interval has same Shape as this EdgeDefinition. Set bIsIdentical = TRUE when SrcTopos are also the same
  SmBoolean IsSameEdgeDefinition                              // rtn: TRUE = TgtXSectCrv has same Shape as a contained EdgeDefinition
              (SmProtoVertex               * pPV1,            // in : Tgt Start ProtoVertex
               SmProtoVertex               * pPV2,            // in : Tgt End   ProtoVertex
               const SmPointClassification & crMidPC1,        // in : MidPt Classification for face1->CrvClass->TgtCrvInterval
               const SmPointClassification & crMidPC2,        // in : MidPt Classification for face2->CrvClass->TgtCrvInterval
               const SmCurve               * pCurve3d,        // in : shared XSectCurve3d being classified against face1/face2 pair
               const SmExtent1d            & rCurveDomain,    // in : TgtCrvInterval domain
               SmBoolean                   & rbIsIdentical)   // out: TRUE = this and TgtXSectCrv->Interval->MidPt classifications
              const ;                                         //             map to the same pair of Brep1 and Brep2 Src topology objects.
  SmBoolean IsInteriorPoint( const SmPoint3d & crPosition,
                             SmXSectTol3d      sTol3d,
                             SmBoolean         bAtStart,
                             double          & rdParam) const ;

  // utilities
  void         Dump( int iDumpLevel, ULONG * lIdx = NULL ) const;
  virtual void Dump() const { Dump( -1, NULL) ; }
  void         Draw() ;

} ; // end class SmEdgeDefinition

/*******************************************************************//**
PURPOSE: Class for an SmProtoVertex.

USAGE NOTES---- 
   A ProtoVertex represents a collection of VertexDefinitions located at
   the same position.  A single ProtoVertex represents all Brep-Brep
   topology intersections that would result in a Vertex at this location.
   When the ProtoVertex is Resolved, it decides which of its VertexDefinitions
   belong in the same Vertex and combines them.  It results in usually one
   but possibly more VertexDefinitions, each of which will be imprinted as a
   Vertex in each Brep.

   To create a ProtoVertex, call SmProtoTopologyManager::FindOrCreateProtoVertex().
   That will first check to see whether a ProtoVertex already exists at
   that location, and if so, will add the VertexDefinition to that ProtoVertex,
   and if not, will create a new ProtoVertex with that VertexDefinition.
***********************************************************************/
class SM_EXPORT SmProtoVertex : public SmTopology
{
 protected:
  SmPoint3d                       m_sPosition ;           // resolved Vertex position
  double                          m_dTol ;                // vertex tolerance
                                                          //   uses: SmZoneTol3d in SmProtoVertex::ContainsPoint

  SmTArray<SmVertexDefinition *> m_sVertexDefinitions ;   // list of VertexDefinitions
  SmTArray<SmProtoEdge *>        m_sProtoEdges ;          // array of ProtoEdges connected to this ProtoVertex
  SmProtoTopologyManager       * m_pProtoTopoMgr;         // Back-pointer to owning object.

 public:
  // constructor with 1 VD
  SmProtoVertex( const SmPointClassification & crPC1,     // in : Face1 PtClassification of XSectPt3d solution
                 const SmPointClassification & crPC2,     // in : Face2 PtClassification of XSectPt3d solution
                 const SmPoint3d             & rPosition, // in : new ProtoVertex Position (commonly from a XSectPt3d solution)
                 double                        dTol,      // in : new ProtoVertex ZoneTol3d
                 SmProtoTopologyManager      * pOwner) ;  // in : new ProtoVertex owning ProtoManager

  // constructor with 1 VD
  SmProtoVertex( SmVertexDefinition          * pVtxDef,   // in : new ProtoVertex 1st VertexDef     
                 SmProtoTopologyManager      * pOwner) ;  // in : new ProtoVertex owning ProtoManager

  // constructor with no VDs
  SmProtoVertex( const SmPoint3d             & rPosition, // in : new ProtoVertex Position (commonly from a XSectPt3d solution)
                 double                        dTol,      // in : new ProtoVertex ZoneTol3d
                 SmProtoTopologyManager      * pOwner) ;  // in : new ProtoVertex owning ProtoManager

  // destructor
 ~SmProtoVertex() ;

  // simple data access
  SmPoint3d                       & GetPosition               ()                                        { return m_sPosition ; }
  SmZoneTol3d                       GetTolerance              ()                                  const { return m_dTol ; }
  SmVertexDefinition              * GetClosestVertexDefinition(const SmPoint3d & crPosition)      const ;
  double                            GetCaptureDistance        ()                                  const ;
  SmProtoTopologyManager          * GetProtoTopologyManager   ()                                  const { return m_pProtoTopoMgr ; }
  SmVertex                        * GetFinalVertex            (int iWhichBrep,                                        // in : 
                                                               const SmEdgeDefinition * pEdgeDef,         // in : 
                                                               SmBoolean bAtStart )               const ; // in :
  SmEdge                          * GetBrepEdge               (int iWhichBrep,                            // in : 
                                                               const SmEdgeDefinition * pEdgeDef,         // in : 
                                                               SmBoolean bAtStart )               const ; // NotUsed: in : 
  ULONG                             GetNumVertexDefinitions   ()                                  const { return m_sVertexDefinitions.GetSize() ; }
  ULONG                             GetNumProtoEdges          ()                                  const { return m_sProtoEdges.GetSize() ; }
  SmTArray< SmVertexDefinition* > & GetVertexDefinitions      ()                                        { return m_sVertexDefinitions ; }
  SmTArray< SmProtoEdge*        > & GetProtoEdges             ()                                        { return m_sProtoEdges ;  }

  // predicates
  SmBoolean            ContainsVertexDefinition    (SmVertexDefinition *pVD ) { return m_sVertexDefinitions.IsIn( pVD) ; }
  SmVertexDefinition * FindMatchingVertexDefinition(const SmTopology *pTopo1,    // rtn: VertexDefinition matching Tgt SrcTopo objs or NULL for none
                                                    const SmTopology *pTopo2) ;
  SmBoolean            IsOverlappingPEGroupMember() const ;

  // Queries/Predicates
  SmBoolean ContainsPoint( const SmPoint3d &crPosition) ;

  double    DistanceTo   ( const SmPoint3d &crPosition )  { return ( m_sPosition.DistanceBetween( crPosition )) ; }

  // Working methods:
  void     ClearVertexDefinitions() { m_sVertexDefinitions.ReSet(); }

  // add input VertexDefinition to ProtoVertex->Definition list
  SmStatus AddVertexDefinition   ( SmVertexDefinition *pNewVD) ;

  // Find or Create a VertexDefinition for input PtClass pair
  SmStatus AddVertexDefinition   ( SmPointClassification  & rPC1,                  // in : Brep1 PointClassification for Brep1/Brep2 XSectPt3d
                                   SmPointClassification  & rPC2,                  // in : Brep2 PointClassification for Brep1/Brep2 XSectPt3d
                                   SmVertexDefinition    *& pNewVtxDef,            // out: default [NULL]
                                   SmBoolean              * pbWasCreated = NULL) ; // out: optional flag: TRUE = rpNewVD was created, NULL to ignore, default:[NULL]
                                                                                   //                     FALSE= rpNewVD was found
  // Find or Create a VertexDefinition for input PtClass pair and assign it an input Pos3d 
  SmStatus AddVertexDefinition   ( SmTopology          * pTopo1,                   // in : Brep1 PointClassification for Brep1/Brep2 XSectPt3d
                                   SmTopology          * pTopo2,                   // in : Brep2 PointClassification for Brep1/Brep2 XSectPt3d
                                   const SmPoint3d     & crPos,                    // in : Pos3d to assign to new or found VertexDef
                                   SmVertexDefinition *& pNewVtxDef,               // out: default [NULL]
                                   SmBoolean           * pbWasCreated = NULL) ;    // out: optional flag: TRUE = rpNewVD was created, NULL to ignore, default:[NULL]
                                                                                   //                     FALSE= rpNewVD was found

  void     AddProtoEdge          ( SmProtoEdge * pPE ) { m_sProtoEdges.Add( pPE) ; } // DD1: Even if already there.
  SmStatus RemoveVertexDefinition( SmVertexDefinition *pVD) ;
  SmStatus RemoveProtoEdge       ( SmProtoEdge *pPE) ;
  void     CombineProtoVertex    ( SmProtoVertex *pOtherPV) ;
  SmStatus Resolve               () ;

  // utilities
  void         Dump( int iDumpLevel, ULONG * lIdx = NULL ) const;
  virtual void Dump() const { Dump( -1, NULL) ; }
  void         Draw();

} ; // end class SmProtoVertex

/*******************************************************************//**
PURPOSE: Class for a ProtoEdge.

   A ProtoEdge represents a collection of EdgeDefinitions located at
   the same position.  A single ProtoEdge represents all Brep-Brep
   topology intersections that would result in a Edge at this location.
   When the ProtoEdge is Resolved, it decides which of its EdgeDefinitions
   belong in the same Edge and combines them.  It results in usually one
   but possibly more EdgeDefinitions, each of which will be imprinted as a
   Edge in each Brep.

   To create a ProtoEdge, call SmProtoTopologyManager::FindOrCreateProtoEdge().
   That will first check to see whether a ProtoEdge already exists at
   that location, and if so, will add the EdgeDefinition to that ProtoEdge,
   and if not, will create a new ProtoEdge with that EdgeDefinition.

USAGE NOTES---- 
***********************************************************************/
class SM_EXPORT SmProtoEdge : public SmTopology
{
 protected:
  // We have to ensure that all ED's have same parameterization.
  SmExtent1d                    m_pXSectInterval ;  // If we're on an SmEdge, then we're coincident.
                                                    // This is the original range.
                                                    // If we're on an SmFace, this is the domain of the intersection curve.
                                                   
  SmProtoVertex                * m_pStartPV ;        // ProtoVertex bounding the start of this ProtoEdge
  SmProtoVertex                * m_pEndPV ;          // ProtoVertex bounding the end   of this ProtoEdge
  SmTArray< SmEdgeDefinition *>  m_sEdgeDefs ;       // List of EdgeDefinitions 
                                                    
  double                         m_dTol ;            // probably the XSectTol3d value used for these intersections
                                                     // use: XSectTol3d in SmProtoEdge::Split
  SmProtoTopologyManager       * m_pProtoTopoMgr ;   // Back-pointer to owning object.

  SmTArray< SmProtoEdge *>       m_sOverlappingPEs ; // List of ProtoEdges generated in the Face/Face XSect phase of IntersectTopology()
                                                     // that were created from XSect interval solutions that classify to an existing Brep
                                                     // Edge in one or both of the Boolean input Breps that 'overlap' with this ProtoEdge. That
                                                     // happens when an Edge in one Brep is coincident with an Edge or Face in the other Brep. 
                                                     // The problem is that multiple Face/Face XSect combinations will generate XSect 
                                                     // solutions of the same coincident geometry but represented in differing interval 
                                                     // sets that can overlap one another. This list is used by the Resolve() method  
                                                     // to gather and replace entire sets of overlapping ProtoEdges representing 
                                                     // a single coincident edge with a clean set of non-overlapping ProtoEdges. 
                                                     // The clean, nonOverlapping ProtoEdges are not made from XSect results but 
                                                     // rather from the homogenized intervals of a pair of new CrvClassifications made by 
                                                     // classifying the now-known-to-be-coincident curve geometry against appropriate target 
                                                     // faces in each Brep. 
  SmTArray< SmProtoFace *>       m_sCoincPFs ;       // ProtoFaces representing coincident Brep1/Brep2 face pairs
                                                     // bounded by this ProtoEdge.

public:
  // constructor
  SmProtoEdge( SmProtoVertex          * pStartPV,    // in : ProtoVertex bounding the start of this ProtoEdge
               SmProtoVertex          * pEndPV,      // in : ProtoVertex bounding the end   of this ProtoEdge
               const SmExtent1d       & sIvl,        // in : Interval of SmEdgeDefinition XSectCrvs represented by this ProtoEdge
               SmProtoTopologyManager * pOwner) ;    // in : Back-pointer to owning SmProtoTopologyManager object

//  SmProtoEdge( SmCurve                * p3dCurve,
//               SmExtent1d             & rIvl,
//               SmTopology             * pBrep1Topo,
//               SmTopology             * pBrep2Topo,
//               SmProtoVertex          * pStartPV,
//               SmProtoVertex          * pEndPV,
//               double                   dTol,
//               SmCurve                * pUVCurve1,
//               SmCurve                * pUVCurve2,
//               SmProtoTopologyManager * pOwner) ;

  // copy constructor
  SmProtoEdge(const SmProtoEdge &crObjToCopy)
             : m_pXSectInterval( crObjToCopy.m_pXSectInterval ),
               m_pStartPV      ( crObjToCopy.m_pStartPV ),
               m_pEndPV        ( crObjToCopy.m_pEndPV ),
               m_dTol          ( crObjToCopy.m_dTol ),
               m_pProtoTopoMgr ( crObjToCopy.m_pProtoTopoMgr )
             { m_sEdgeDefs.ReSet() ;
               m_sOverlappingPEs.ReSet() ;
               m_sCoincPFs.ReSet() ; 
             }

  // destructor
 ~SmProtoEdge() ;

  // simple data access
  SmExtent1d                      GetXSectInterval       ()                                            const { return m_pXSectInterval; }
  SmZoneTol3d                     GetTolerance           ()                                            const { return m_dTol; }
  SmProtoVertex                 * GetStartProtoVertex    ()                                            const { return m_pStartPV; } ;
  SmProtoVertex                 * GetEndProtoVertex      ()                                            const { return m_pEndPV;   } ;
  SmTArray<SmProtoEdge *>       & GetOverlappingPEs      ()                                                  { return m_sOverlappingPEs ; } 
  void                            GetOverlappingPEs      (SmTArray<SmProtoEdge *> & rOverlappingPEs)   const { rOverlappingPEs = m_sOverlappingPEs; }
  void                            GetOverlappingPEGroup  (SmTArray<SmProtoEdge *> & rOverlappingGroup) const ; // include case of A overlaps B and C, while B and C are disjoint                                           
  ULONG                           GetNumEdgeDefinitions  ()                                            const { return m_sEdgeDefs.GetSize(); }
  SmTArray<SmEdgeDefinition *>  & GetEdgeDefinitions     ()                                                  { return m_sEdgeDefs; }
  SmProtoTopologyManager        * GetProtoTopologyManager()                                            const { return m_pProtoTopoMgr; }
  SmTArray<SmProtoFace *>       * GetCoincProtoFaces     ()                                                  { return &m_sCoincPFs; }
                                                                                                       
  double                          GetCaptureDistance     ()                                            const ;

  void SetXSectInterval   (const SmExtent1d crIvl ) { m_pXSectInterval = crIvl; }
  void SetStartProtoVertex(SmProtoVertex * pPV )    { m_pStartPV = pPV; }
  void SetEndProtoVertex  (SmProtoVertex * pPV )    { m_pEndPV   = pPV; }

  // actions
  void RelateCoincidentPF  (SmProtoFace * pCoincPF ) ;
  void AddOverlappingPE    (SmProtoEdge *& prOverlappingPE )                { m_sOverlappingPEs.Add( prOverlappingPE) ; }
  void AppendOverlappingPEs(SmTArray< SmProtoEdge* > & rOverlappingPEs )    { m_sOverlappingPEs.Append( rOverlappingPEs) ; }
  void ClearEdgeDefinitions()                                               { m_sEdgeDefs.ReSet(); }
  void ReplaceProtoVertex  (SmProtoVertex * pOldPV, SmProtoVertex * pNewPV) ; // If ProtoEdge points to pOldPV ProtoVertex, replace it with pNewPV 

  // Return ProtoEdge->EdgeDefinition that matches a proposed EdgeDefinition's XSectCrv Brep1 and Brep2 MidInterval classifications or NULL for none
  SmEdgeDefinition * FindMatchingEdgeDefinition(const SmCurveInterval & crCI1,   // in : proposed EdgeDefinitions Brep1 XSectCrv's midInterval classification
                                                const SmCurveInterval & crCI2,   // in : proposed EdgeDefinitions Brep2 XSectCrv's midInterval classification
                                                SmBoolean             & rbRev) ; // in : [not used] //cbi probably deal with this.

  // "Evaluation" methods:
  SmStatus GetEndPosition(SmProtoVertex *pWhichPV, SmPoint3d & rPos) const ;  // set rPos = (pWhichPV == m_pStartPV) ? Avg(EdgeDefinition starts) : (pWhichPV == m_pStartPV) ? Avg(EdgeDefinition ends)
  SmStatus GetEndPosition(SmBoolean      bAtStart, SmPoint3d & rPos) const ;  // set rPos = bAtStart ? Avg(EdgeDefinition starts) : Avg(EdgeDefinition ends)
  SmStatus Evaluate      (double         dParam,   SmPoint3d * pPos=NULL, SmVector3d * pTan=NULL) ;
  SmStatus EvaluateMid   (                         SmPoint3d * pPos=NULL, SmVector3d * pTan=NULL) ;

  // return true when XSecting Interval has same Shape as a contained EdgeDefinition. Set bIsIdentical = TRUE when SrcTopos are also the same
  SmBoolean IsSameProtoEdge                                  // rtn: TRUE = XSecting Interval has same Shape as a contained EdgeDefinition
              (SmProtoVertex               * pPV1,           // in : Tgt Start ProtoVertex
               SmProtoVertex               * pPV2,           // in : Tgt End ProtoVertex
               const SmPointClassification & crMidPC1,       // in : face1->CrvClass1->TgtCrvInterval->Mid PtClassification
               const SmPointClassification & crMidPC2,       // in : face2->CrvClass2->TgtCrvInterval->Mid PtClassification
               const SmCurve               * pCurve3d,       // in : shared XSectCurve3d being classified against face1/face2 pair
               const SmExtent1d            & rCurveDomain,   // in : XSectCrv domain for this TgtCrvInterval
               SmBoolean                   & rbIsIdentical)  // out: TRUE = the same-shape EdgeDefinition also shares the same SrcTopos as the input XSect Interval
              const ;                                        //      FALSe= the same-shape EdgeDefinition comes from a different SrcTopo XSection

  // Working methods:
  SmStatus Resolve() ;

  // Split ProtoEdge at given Param value - also splits ProtoEdge->EdgeDefinitions and updates all topological connection relationships
  SmStatus Split  (double           dT,        // in : Tgt Split param, checked to be interior value
                   SmProtoVertex *& rpNewPV,   // out:  When dT is interior, new ProtoVertex at Split point
                   SmProtoEdge   *& rpNewPE) ; // out:  When dT is interior, new Upper Child Split ProtoEdge:[dT, IvlMax]
                                               // note: When dT is interior, this PE reused for Lower Child split ProtoEdge:[IvlMin, dT]

  SmStatus AddEdgeDefinition   (SmEdgeDefinition * pEdgeDef) ;                                  // eff: Add EdgeDef to this ProtoEdge's EdgeDef list
  SmStatus AddEdgeDefinition   (const SmCurveInterval & crCI1, const SmCurveInterval & crCI2) ; // eff: Create EdgeDef and add to this ProtoEdge's EdgeDef list
  void     CombineProtoEdge    (SmProtoEdge * pOtherPE) ;
  SmStatus RemoveEdgeDefinition(SmEdgeDefinition * pEdgeDef) ;
  SmStatus RemoveProtoVertex   (SmProtoVertex * pPV) ;

public:
  // utilities
  void         Dump(int iDumpLevel, ULONG * lIdx = NULL) const;
  virtual void Dump() const { Dump( -1, NULL) ; }
  void         Draw();

} ; // end class SmProtoEdge

/*******************************************************************//**
PURPOSE: Class for a ProtoFace representing a coincident Brep1/Brep2 face pair.

USAGE NOTES---- 
   This is different from SmProtoEdge and SmProtoVertex in that there are
   no intersections.  It's a way to record coincident Faces.
***********************************************************************/
class SM_EXPORT SmProtoFace : public SmTopology
{
  SmFace                   * m_pBrep1Face ;    // Brep1->Face of the coincident Brep1/Brep2 face pair.
  SmFace                   * m_pBrep2Face ;    // Brep2->Face of the coincident Brep1/Brep2 face pair.
  SmTArray<SmProtoEdge *>    m_sCoincPEs ;     // list of ProtoEdges bounding this ProtoFace
                                               //   note: before imprinting the ProtoEdge SrcTopo Types might be
                                               //         SM_PC_FACE, SM_PC_EDGE happens for partial coincidence
                                               //         SM_PC_EDGE, SM_PC_EDGE happens for total coincidence along this one boundary edge
                                               //         after imprinting both Brep's will have an Edge for each one of these PEs
                                               //         meaning their SrcTopo types will end as SM_PC_EDGE, SM_PC_EDGE. 
  SmProtoTopologyManager   * m_pProtoTopoMgr ; // Back-pointer to owning object.

public:
  // constructor
  SmProtoFace( SmFace *pBrep1Face,
               SmFace *pBrep2Face,
               const SmTArray< SmProtoEdge* > & crCoincPEs,
               SmProtoTopologyManager *pOwner )
             : m_pBrep1Face( pBrep1Face ),
               m_pBrep2Face( pBrep2Face ),
               m_pProtoTopoMgr( pOwner )
             {
               m_sCoincPEs = crCoincPEs;  // Assignment operator
             }

  // destructor
 ~SmProtoFace() { }

  // simple data access
  SmTopology               * GetSrcTopo             ( int iWhichBrep )                       const { return ( iWhichBrep==1 ) ? m_pBrep1Face : m_pBrep2Face; }
  SmPointClassificationType  GetSrcTopo_TYPE        ( int /* iWhichBrep */ )                 const { return SM_PC_FACE; }
  SmProtoTopologyManager   * GetProtoTopologyManager()                                       const { return m_pProtoTopoMgr; }
  SmTArray<SmProtoEdge *>  * GetCoincProtoEdges     ()                                             { return &m_sCoincPEs; }
  void                       GetCoincProtoEdges     ( SmTArray< SmProtoEdge* > & rCoincPEs ) const { rCoincPEs = m_sCoincPEs; }
  void                       RelateCoincidentPF     ( SmProtoEdge * pCoincPE )                     { m_sCoincPEs.AddUnique( pCoincPE) ; 
                                                                                                     pCoincPE->GetCoincProtoFaces()->AddUnique(this) ;
                                                                                                   }
  void SetSrcTopo(int iWhichBrep, 
                  SmPointClassificationType /* eType */, 
                  SmTopology *pTopo )                     { ( iWhichBrep==1 ? m_pBrep1Face : m_pBrep2Face ) = SM_CAST_PTR( SmFace, pTopo) ; }

  // utilities
  void         Dump(int iDumpLevel, ULONG * lIdx = NULL) const ;
  virtual void Dump() const { Dump( -1, NULL) ; }
  void         Draw() ;

} ; // end class SmProtoFace

/*******************************************************************//**
PURPOSE: A container to collect and manage SmProtoEdges, SmProtoVertices, and SmProtoVertices.

USAGE NOTES---- Used to gather IntersectTopology results as part of the Boolean process
***********************************************************************/
class SM_EXPORT SmProtoTopologyManager : public SmTopology
{
  SmBrep                  * m_pBrep1 ;           // 1st Brep being booleaned
  SmBrep                  * m_pBrep2 ;           // 2nd Brep being booleaned
  double                    m_dTol ;             // seems to be an override XSectTol3d = max dist between coincident points
                                                 //   pos     : Intersect TopoObjs to this XSectTol3d value
                                                 //   0 or neg: m_dtol = if(set) m_pTI->ApproxTol3d value
                                                 //             m_dtol = else    (Brep1->Tol + Brep2->Tol + MinEdgeSize / 2.0) ;

                                                 // GWC Note: 1. the name of m_dTol needs to change to reflect its use, I'm guessing that will be become
                                                 //              SmXSectTol3d m_sXSectTol3d ;
                                                 //           2. the method SetTolerance() calls m_dTol ZoneTol3d.
                                                 //           3. the initialization of m_dTol in the contsructor is confused,
                                                 //                It either stores a given value, uses the m_pTI->ApproxTol3d value or computes
                                                 //                a new value as (Brep1->GetTolerance() + Brep2->GetToleranc() + MinEdgeSize / 2.0) ;
                                                 //              which mixes ApproxTol and XSectTol concepts and implements its own function for an
                                                 //              XSectTol3d value.  This needs to be reviewed and made consistent with the new topology model. 
                                                 //           4. This one value mixes together the concepts of ApproxTol3d, ZoneTol3d, and XSectTol3d and 
                                                 //              implements its own function for an XSectTol3d value.  
                                                 //           5. The definition and use of m_dTol needs to be reviewed and made consistent with the new topology model
                                                 // found uses: XSectTol3d in SmProtoTopologyManager::SplitIntersectingProtoEdges 

  double                    m_dCaptureDistance ; // max dist allowed between same shaped curves, used in IsSameEdgeDefinition()
                                                 // a value typically larger than XSectTol3d distance currently being used to account to tol errors.
                                                 // This may not be needed once the new Tol model gets deployed

  SmTArray<SmProtoVertex *> m_sPVs ;             // array of ProtoVertices marking m_pBrep1/m_pBrep2 XSect points
  SmTArray<SmProtoEdge *>   m_sPEs ;             // array of ProtoEdges marking m_pBrep1/m_pBrep2 XSect curves
  SmTArray<SmProtoFace *>   m_sPFs ;             // array of ProtoFacess marking m_pBrep1/m_pBrep2 Coincident surfaces
                                                 
  SmTopologyIntersector   * m_pTI;               // contains other info about the IIR phase of the Boolean operation

public:
  // constructor
  SmProtoTopologyManager(SmBrep                * pBrep1,                // in : 1st Brep of Boolean arg list
                         SmBrep                * pBrep2,                // in : 2nd Brep of Boolean arg list
                         SmTopologyIntersector * pTI,                   // in : TopologyIntersector obj being used for the Boolean operation
                         double                  dCaptureDistance,      // in :
                         double                  dOverrideTol = 0.0) ;  // seems to be an override XSectTol3d = max dist between coincident points
                                                                        //   pos     : Intersect TopoObjs to this XSectTol3d value
                                                                        //   0 or neg: m_dtol = if(set) m_pTI->ApproxTol3d value
                                                                        //             m_dtol = else    (Brep1->Tol + Brep2->Tol + MinEdgeSize / 2.0) ;
  // destructor
  ~SmProtoTopologyManager() ;

  // set ProtoArray sizes to zero
  void Reset() ;

  // Simple Access:
  SmBrep                     * GetWhichBrep      (int iWhichBrep)      const { return ( iWhichBrep==1 ) ? m_pBrep1 : m_pBrep2 ; }
  virtual SmZoneTol3d          GetTolerance      ()                    const { return((SmZoneTol3d)m_dTol); }
  double                       GetCaptureDistance()                    const { return m_dCaptureDistance ; }
  SmTArray< SmProtoVertex* > & GetProtoVertices  ()                          { return m_sPVs ; }
  SmTArray< SmProtoEdge  * > & GetProtoEdges     ()                          { return m_sPEs ; }
  SmTArray< SmProtoFace  * > & GetProtoFaces     ()                          { return m_sPFs ; }
  ULONG                        GetProtoVertexId  (const SmProtoVertex * pPV) const ; // map PV ptr to ProtoTopoMgr list item indx
  ULONG                        GetProtoEdgeId    (const SmProtoEdge   * pPE) const ; // map PE ptr to ProtoTopoMgr list item indx
  ULONG                        GetProtoFaceId    (const SmProtoFace   * pPF) const ; // map PF ptr to ProtoTopoMgr list item indx
  void                         GetAllVertexDefinitions(SmTArray< SmVertexDefinition* > & rVtxDefs ) ;
  void                         GetAllEdgeDefinitions  (SmTArray< SmEdgeDefinition*   > & rEdgeDefs) ;

  void         SetCaptureDistance(double      dNewCaptureDist)         { m_dCaptureDistance = dNewCaptureDist; }
  virtual void SetTolerance      (SmZoneTol3d sNewZoneTol3d, 
                                  SmBoolean   bUpdateOnlyIfLarger=TRUE,
                                  SmBoolean   bCascadeToVertices=TRUE) { SM_REF2(bUpdateOnlyIfLarger, bCascadeToVertices) ; 
                                                                         m_dTol = (double)sNewZoneTol3d ; 
                                                                       }
  // Working methods:
  void AddProtoVertex(SmProtoVertex * pPV ) { m_sPVs.Add(pPV) ; }
  void AddProtoEdge  (SmProtoEdge   * pPE ) { m_sPEs.Add(pPE) ; }
  void AddProtoFace  (SmProtoFace   * pPF ) { m_sPFs.Add(pPF) ; }

  // Find existing VertexDefinition within a ProtoVertex near TgtPos and defined by the input Topo pair 
  SmVertexDefinition * FindVertexDefinition(const SmVector3d & sPos,    // in : Tgt Position - used to find nearby ProtoVertices
                                            const SmTopology * pTopo1,  // in : Topo1 of VertexDefinition TopoPair to be found in ProtoVertex within tol of crPos
                                            const SmTopology * pTopo2,  // in : Topo2 of VertexDefinition TopoPair to be found in ProtoVertex within tol of crPos
                                            double dTol) ;              // NotUsed: in : max search distance for ProtoVertex search

  // Find closest SmProtoVertex within m_dTol of a Tgt position
  SmProtoVertex    * FindProtoVertex(const SmPoint3d & rPos,            // in : Target position
                                     double          * pdDist = NULL) ; // out: Deviation from found ProtoVertex to TgtPos, NULL to ignore

  // Find the SmEdgeDefinition that refers to a TgtEdge
  SmEdgeDefinition * FindEdgeDefinitionFromEdge(SmEdge *pTargetEdge,    // in : TgtEdge referenced by desired EdgeDefinition
                                                int     iWhichBrep) ;   // in : iWhichBrep == 1 ? m_Brep1 : m_Brep2

  SmStatus FindOrCreateVertexDefinition(SmTopology          * pTopo1,
                                        SmTopology          * pTopo2,
                                        const SmPoint3d     & crPt,
                                        SmVertexDefinition *& rpNewVD,
                                        SmBoolean           * pbWasCreated = NULL) ;

  SmStatus CreateVertexDefinition(SmPointClass        & rPC1,
                                  SmPointClass        & rPC2,
                                  SmVertexDefinition *& rpNewVD) ;

  SmStatus CreateVertexDefinition(SmTopology          * pTopo1,
                                  SmTopology          * pTopo2,
                                  const SmPoint3d     & crPt,
                                  SmVertexDefinition *& rpNewVD) ;

  SmStatus CreateProtoVertex(SmPointClassification & rPC1,
                             SmPointClassification & rPC2,
                             SmProtoVertex        *& rpNewPV) ;

  // Find or create a ProtoVertex from two SmPointClassifications
  SmStatus FindOrCreateProtoVertex(SmPointClassification & rPC1,                 // in : Brep1 PointClassification for face pair XSectPt3d
                                   SmPointClassification & rPC2,                 // in : Brep2 PointClassification for face pair XSectPt3d
                                   SmProtoVertex        *& rpNewPV,              // out: SmProtoVertex for face pair XSectPt3d
                                   SmBoolean             * pWasCreated = NULL) ; // out: TRUE = Created output NewPV
                                                                                 //      FALSE= Found   output NewPV
//cbi Not currently used:
//  SmStatus FindOrCreateProtoVertex(const SmPoint3d  & crPos,
//                                   const SmTopology * pTopo1,
//                                   const SmTopology * pTopo2,
//                                   SmProtoVertex   *& rpNewPV,
//                                   SmBoolean        * pWasCreated = NULL) ;

  // return PE with a ED->XSectCrv3d within CaptureDistance of TgtCurve3d or NULL. 
  // Set rbIsIdentical == TRUE when returned PE's same-shape ED also shares the same SrcTopos as the input XSect Interval
  SmProtoEdge   * FindProtoEdge(SmProtoVertex               * pTgtStartPV,   // in : Tgt Start ProtoVertex
                                SmProtoVertex               * pTgtEndPV,     // in : Tgt End   ProtoVertex
                                const SmPointClassification & crPC1,         // in : Tgt MidPt Classification for face1->CrvClass->TgtCrvInterval
                                const SmPointClassification & crPC2,         // in : Tgt MidPt Classification for face2->CrvClass->TgtCrvInterval
                                const SmCurve               * pTgtCurve3d,   // in : shared Tgt XSectCurve3d being classified against face1/face2 pair
                                const SmExtent1d            & rTgtCurveIvl,  // in : TgtCrvInterval domain
                                SmBoolean                   & rbIdentical)   // out: TRUE  = returned PE->ED also shares the same SrcTopos as the input XSect Interval
                               const ;                                       //      FALSE = returned PE->ED only has the same shape as input XSectInterval but different SrcTopos

  // create and add to this ProtoTopologyManager a ProtoEdge defined by XSect SmCurveIntervals and bounded by given ProtoVertices
  SmStatus CreateProtoEdge(SmProtoVertex         * pStartPV, // in : start ProtoVertex
                           SmProtoVertex         * pEndPV,   // in : end ProtoVertex
                           const SmCurveInterval & crCI1,    // in : CurveIvl from SrcTopo 1
                           const SmCurveInterval & crCI2,    // in : CurveIvl from SrcTopo 2
                           SmProtoEdge          *& pNewPE);  // out: new ProtoEdge

  // Find or create ProtoVertex at given Pos3d
  SmStatus FindOrCreateProtoVertex(const SmPoint3d  & crPos,                // in : TgtPosition
                                   SmProtoVertex   *& rpNewPV,              // out: either an existing ProtoVertex within Tol of crPos, or a new one created at crPos
                                   SmBoolean        * pWasCreated = NULL) ; // out: optional output, TRUE =rpNewPV is a new ProtoVertex, 
                                                                            //                       FALSE=rpNewPV is an existing one, 
                                                                            //      NULL to ignore, default:[NULL]
  // Find or create a ProtoEdge from two SmCurveIntervals
  SmStatus FindOrCreateProtoEdge  (SmCurveInterval & rCI1,                // in : Face1 CrvInterval classification from a face pair XSection
                                   SmCurveInterval & rCI2,                // in : Face2 CrvInterval classification from a face pair XSection
                                   SmProtoEdge    *& rpNewPE,             // out: new or found ProtoEdge
                                   SmBoolean       * pbWasCreated=NULL) ; // out: optional, TRUE  = rpNewPE was created, NULL to ignore, default:[NULL]
                                                                          //                FALSE = rpNewPE was found

  SmStatus CreateProtoFace(SmFace                   * pF1,        // in : Brep1->face of Brep1/Brep2 coincident face pair
                           SmFace                   * pF2,        // in : Brep2->face of Brep1/Brep2 coincident face pair
                           SmTArray< SmProtoEdge* > & crCoinPEs,  // in : array Brep1/Brep2 Coincident edge pairs that bound pF1 and pF2 repsectively
                           SmProtoFace             *& rpNewPF) ;  // out: The new SmProtoFace object

  // Remove but don't delete TgtProtoVertex from m_PVs list
  SmStatus RemoveProtoVertex(SmProtoVertex * pPV) ;

  // Remove but don't delete TgtProtoEdge from m_PVs list
  SmStatus RemoveProtoEdge( SmProtoEdge * pPE) ;

  // Remove but don't delete VertexDefinition from its ownerPV->m_sVertexDefs list
  SmStatus RemoveVertexDefinition(SmVertexDefinition * pVD,
                                  SmBoolean            bRemoveOwnerIfEmpty) ;

  // Remove but don't delete SmEdgeDefinition from its ownerPE->m_sEdgeDefs list
  SmStatus RemoveEdgeDefinition(SmEdgeDefinition * pED,                   // in : EdgeDef to remove
                                SmBoolean          bRemoveOwnerIfEmpty) ; // in : TRUE  = remove and delete PEs with no other EdgeDefs from ProtoMgr
                                                                          //      FALSE = don't

  // Gather list of ProtoEdges whose Edgedefinition->CurveIntervals overlap with the input CurveIntervalPair->CurveInterval
  SmStatus CheckOverlappingProtoEdges(const SmCurveInterval  & crCI1,             // in : Brep 1 CurveInterval Classification 
                                      const SmCurveInterval  & crCI2,             // in : Brep 2 CurveInterval Classification 
                                      SmTArray<SmProtoEdge*> & rOverlappingPEs) ; // out: List of SmProtoEdges that overlap with this CurveInterval

  SmStatus GlueCoincidentBrepVertices();

  // See whether two VertexDefinitions can be combined into one (uses VertexDefinition->GetTolerance() as ZoneTol3d values)
  SmStatus CanCombineVertexDefinitions(SmVertexDefinition * pVD1,
                                       SmVertexDefinition * pVD2,
                                       ULONG              & rlCanDelete) ;

  SmStatus GlueVertexDefinitions(SmVertexDefinition * pVD1,
                                 SmVertexDefinition * pVD2) ;

  // See whether two EdgeDefinitions can be combined into one
  SmStatus CanCombineEdgeDefinitions( SmEdgeDefinition * pED1,
                                      SmEdgeDefinition * pED2,
                                      ULONG            & rlCanDelete) ;

  SmStatus GlueEdgeDefinitions(SmEdgeDefinition * pED1,
                               SmEdgeDefinition * pED2) ;

  // Split intersecting ProtoEdges at the XSect points while updating ProtoTopo and TopoDefinition data structures 
  SmStatus SplitIntersectingProtoEdges(SmProtoEdge * pPE1,   // in : 1st Tgt of ProtoEdge/ProtoEdge XSection
                                       SmProtoEdge * pPE2) ; // in : 2nd Tgt of ProtoEdge/ProtoEdge XSection

  // Remove NoEdge ProtoVertices coincident with other ProtoEdges
  SmBoolean RemoveCoincidentNoEdgeProtoVertex(SmProtoEdge   * pPE,   // in : Tgt ProtoEdge   of ProtoEdge/ProtoVertex XSection
                                              SmProtoVertex * pPV) ; // in : Tgt ProtoVertex of ProtoEdge/ProtoVertex XSection

  // Update all ProtoTopology that sat on a Brep Edge that was split
  SmStatus UpdateEdges(int                         iWhichBrep,          // in : 1 = From Brep1, 2 = from Brep2
                       SmEdge                    * pThisEdge,           // in : original Edge
                       const SmTArray< SmEdge* > & crNewEdges,          // in : result of splitting with new Vertex
                       SmVertexDefinition        * pCallingVD = NULL) ; // in : Optional VertexDefinition being imprinted, NULL to ignore, default:[NULL]

  // Update all ProtoTopology that sat on a Brep Face that was split
  SmStatus UpdateFaces(int                iWhichBrep,          // in : 1 = From Brep1, 2 = from Brep2
                       SmFace           * pThisFace,           // in : Original Face that was split. Currently reused as SplitFace->Child1
                       SmFace           * pNewFace,            // in : New Face that was split from pOldFace. SplitFace->Child2
                       SmEdge           * pSplittingEdge,      // in : The Edge that split the Faces.
                       SmEdge           * pOtherEdge,          // in : Corresponding Edge in other Brep.
                       SmEdgeDefinition * pCallingED = NULL) ; // in : optional calling EdgeDef: don't have to process this one. NULL to ignore. default:[NULL]

  // replace overlappingPE groups from Edge coincidences with new NonOverlapping PEs from cleaner Edge->Curve Face classifications
  SmStatus ResolveOverlappingProtoEdges() ;

  // utilities

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmProtoTopologyManager, SmTopology, SmProtoTopologyManager_TYPE) ;

  void Dump( int iDumpLevel) const ;
  void Draw() const ;
  void DrawDebug(SmBoolean bDoDebugDraw = TRUE, ULONG lFaceDrawSize=1) const ;

} ;  // end class SmProtoTopologyManager

#endif // __SMPROTOTOPO_H__
