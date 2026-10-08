// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmVertex.h
* PURPOSE: Header file for SmVertex class.
**********************************************************************/

#ifndef __SMVERTEX_H__
#define __SMVERTEX_H__

#ifndef __SMOWNINGTOPOLOGY_H__
#include <SmOwningTopology.h>
#endif

#ifndef __SMTLIST_H__
#include <SmTList.h>
#endif

class SmGapArray ;
class SmPolyVertex ;

/*******************************************************************//**
PURPOSE: This object is used to represent a PolyVertex list in the 
   SmVertex class.

NOTES: static class object - don't add virtual methods to SmPolyVertexList 
***********************************************************************/
class SM_EXPORT SmPolyVertexList : public SmListNode
{
public:
  // inherited:
  // SmListNode::m_pNext;
  // SmListNode::m_pPrev;
  SmPolyVertex * m_pPolyVertex ;

  // constructor
  SmPolyVertexList(SmPolyVertex *pPolyVertex) : m_pPolyVertex(pPolyVertex) { }
                                              
  // destructor                               
  ~SmPolyVertexList()
  {
    SmListNode::ReSet();   // clear Next/Prev pointers
    m_pPolyVertex = NULL;
  }

  // utilities
  void Dump() const ;
                                              
} ; // end class SmPolyVertexList

SM_TLIST_TEMPLATE_PREDECLARATION(SmPolyVertexList) ;

/*******************************************************************//**
PURPOSE: This class represents topological Vertices.

NOTES: 
***********************************************************************/
class SM_EXPORT SmVertex : public SmOwningTopology
{
  friend class SmBrep;
  friend class SmFace;
  friend class SmVertexuse;
  friend class SmBrepConstructor;
  friend class SmObjsDelete<SmVertex*>;

protected:
  // inherited:
  // SmTopology::m_pListOwner - when in a Brep - SmBrep::m_pVertexListHead
  // SmTopology::m_pNext      - Next member of vertex list owned by SmBrep::m_pVertexListHead
  // SmTopology::m_pLast      - Last member of vertex list owned by SmBrep::m_pVertexListHead 
  //
  // SmOwningTopology::m_pList     - used to store pointer to primary Vertexuse (head of connected vertexuse link-list)
  // SmOwningTopology::m_lListSize - used to store number of Vertexuses for this vertex
  SM_NEWTOL_LINE // SmTopology::m_bIsSmallTopology - used by Face, Edge, Vertex (rarely needed - leave defaulted 99.99% of the time)
                  //                                - TRUE = Intended small geometry size (Pinhole in Battleship) - gets tighter tolerances
                  //                                - FALSE= Typical size - gets typical tolerances
                  //                                -   default:[FALSE] 
     
  SmPoint3d    m_vPoint ;                   // Euclidian point associated with the vertex
                             
  SM_OLDTOL_LINE SmZoneTol3d m_sZoneTol3d ; // Stored ZoneTol3d for this object

  SmPolyVertex * m_pPolyVertex3D  = NULL ;  // temp (during tessellation) backPointer to 3d PolyVertex object
                                            //   constructed in SmTess::m_p3DPolyBrep.  
                                            // note: The tessellation process builds one PolyBrep per Brep->Face
                                            //   stored in Face->Surface => SmTessSrfCache->mTS_PolyBrep which is a 
                                            //   UVDomain tessellation of the Brep->Face. A single Brep->Vertex may generate 
                                            //   multiple PolyVertex2d objects in multiple surface => pSC->mTS_PolyBrep objects.
                                            //   (a SmVertex at a Pole will end up mapped to multiple PolyVertex2d objects)
                                            //   Each derived PolyVertex2d object stores a backPointer to the SmVertex object
                                            //   that generated them.
                                            //   Later all the mTS_PolyBrep UVDomain Face tessellations can be used
                                            //   to build a single watertight 3d PolyBrep tessellation built and stored
                                            //   in SmTess::m_p3DPolyBrep.  When building m_p3DPolyBrep, every
                                            //   PolyVertex2d generated from this SmVertex is used to generate one
                                            //   PolyVertex3d.  To prevent creating multiple coincident PolyVertex3d 
                                            //   objects, this temp BackPointer is set to the to the first PolyVertex3d 
                                            //   object made from any of the derived PolyVertex2d objects.  Subsequent
                                            //   calls to generate PolyVertex3d objects from other derived PolyVertex2d objects
                                            //   reuse the temp stored PolyVertex3d value rather than creating a new
                                            //   duplicate coincident one.
#ifdef SM_TOLERANT_WATCH
  SmBoolean    m_bExact;      // FALSE= vertex represents two or more exact point locations
                              //        within tolerance of one another like the intersection
                              //        of 4 surfaces where the surfaces don't exactly come to just one point.
                              // TRUE = vertex represents just one exact location
                              //        like the intersection of 3 surfaces.
#endif // SM_TOLERANT_WATCH

#ifdef USE_DEBUG_COUNTER
  ULONG        m_lDebugCount; // For Debugging purposes
#endif // USE_DEBUG_COUNTER

protected:
  // constructor
  SmVertex()                  : m_vPoint(0,0,0) 
               SM_OLDTOL_LINE , m_sZoneTol3d(0.0)        
                              {
#ifdef SM_TOLERANT_WATCH      
                                m_bExact = TRUE ;
#endif // SM_TOLERANT_WATCH                       
#ifdef USE_DEBUG_COUNTER      
                                m_lDebugCount = 0; 
#endif // USE_DEBUG_COUNTER                      
                                // report construction at SmObject::Notify level - skip other levels
                                SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL);
                              }  // end SmVertex constructor
  // destructor               
  virtual ~SmVertex()         { Notify(SM_NO_DESTRUCTION, this, NULL, NULL);
                                m_pPolyVertex3D = NULL ; 
                              }

public:
  // find position defined by faces connected to this vertex (currently only a surf/surf/surf intersection)
  SmStatus CalcCornerFromFaces
  (
    SmBoolean & rbFoundPoint, ///< [out]: TRUE = found a Point on connected faces, FALSE=didn't
    double    & rdMaxGap3d,   ///< [out]: when bFoundPoint == TRUE, Max VertexFace gap from current vertex position
    SmFace   *& rpMaxGapFace, ///< [out]: when bFoundPoint == TRUE, Face of max VertexFace gap
    SmPoint3d & rsBestPoint   ///< [out]: When bFoundPoint == TRUE, Best possible vertex position to minimize Vertex/Face gaps  otherwise set to uninitialized
  ) const ;      

  // :--------------------------------------------------------------------------------------------:
  // :                          UpDim, DownDim, and SameDim Gaps                                  :
  // :----------+--------------------------+---------------------------+--------------------------+            
  // : Topology : UpDim Gap types          : DownDim Gap types         : SameDim Gap types        :      
  // :----------+--------------------------+---------------------------+--------------------------+       
  // :  Vertex  : Vertex/Edge, Vertex/Face : none                      : none                     :       
  // :  Edge    : Edge/Face                : Vertex/Edge               : none                     :       
  // :  Face    : none                     : Vertex/Face, Edge/Face    : none                     :       
  // :  Loop    : none                     : none                      : EdgeEnd/EdgeEnd LoopGaps :       
  // :----------+--------------------------+---------------------------+--------------------------+   
  //        MaxGap3d = Max( Vertex/Edge Vertex/Face Gap ) 
  virtual const SmGap * GetMaxGap3d       (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const { return GetMaxUpDimGap3d(pOptGapArray, pOptTol3d) ; }
  virtual const SmGap * GetMaxUpDimGap3d  (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const ; // Max(VertexEdge VertexFace) Gap

  SmPoint3d           GetPoint             () const  { return(m_vPoint) ; } 
  virtual SmBrep    * GetBrep              () const;
  SmAObject         * GetAOwner            () const  { return ((SmAObject*)GetBrep()); } // Get the attribute inheritance owner for this object.
  void                GetShells            (SmTArray<SmShell*>     & rShells)       const ;
  void                GetRegions           (SmTArray<SmRegion*>    & rRegions)     const ;
  SmStatus            GetShellInRegion     (const SmRegion         * cpRegion,           
                                            SmShell               *& rpShell)      const ; 
  void                GetFaces             (SmTArray<SmFace*>      & rFaces)       const ;
  void                GetFaceuses          (SmTArray<SmFaceuse*>   & rFaceuses)     const ;
  void                GetEdges             (SmTArray<SmEdge*>      & rEdges,
                                            SmFace                 * pOptFace=NULL) const ;
  void                GetCommonEdges       (SmVertex               * pOtherVertex,       
                                            SmTArray<SmEdge*>      & rCommonEdges,
                                            SmFace                 * pOptFace=NULL) const ;
  void                GetVertexuses        (SmTArray<SmVertexuse*> & rVertexuses, ULONG * pOptAttributeId=NULL) const ;
  SmVertexuse       * GetVertexuseOfFace   (const SmFace           * pFace)        const ;
  SmVertexuse       * GetVertexuseOfEdge   (const SmEdge           * pEdge)        const ;
  void                GetVertexusesOfFace  (const SmFace           * cpFace, SmTArray<SmVertexuse*> & rVertexuses, const SmTopology *pOptConnectTgt=NULL) const ;
  void                GetVertexusesOfEdge  (const SmEdge           * cpEdge, SmTArray<SmVertexuse*> & rVertexuses, const SmTopology *pOptConnectTgt=NULL) const ;
  SmLoop            * GetLoopOfFace        (const SmFace           * cpFace)        const ;
  ULONG               GetVertexNumberInBrep() const ;
  SmVertexuse       * GetPrimaryVertexuse  () const ;
  SmPolyVertex      * GetPolyVertex        () const { return( m_pPolyVertex3D ) ; }
  //SmVertexProps     * GetVertexProps       () const { return( m_pVertexProps ) ; }

  // predicates
  SmBoolean IsShellVertex        () const; // vertex connects directly to region, not connected to any edges or faces 
  SmBoolean IsLoopVertex         () const; // vertex connected to just one LoopVertex: VU1->LU1(->CommonLoop)->FU1->CommonFace, 
                                           //                                          VU2->LU2(->CommonLoop)->FU2->CommonFace
  SmBoolean IsEdgeVertex         () const; // vertex connects directly to one or more edges
  SmBoolean IsWireVertex         () const; // vertex connects to one or more wire edges

  SmBoolean HasWireEdge          () const; // vertex connects to at least one Region as part of a WireEdge
  SmBoolean HasLoopEdge          () const; // vertex connects to at least one face as part of an EdgeLoop
  SmBoolean HasLoopVertex        () const; // vertex connects to at least one face as a LoopVertex
  SmBoolean HasShellVertex       () const; // vertex connects to just one Region as a ShellVertex

  SmBoolean IsLaminaVertex       () const; // connected to one or more lamina edges
  SmBoolean IsTopologicalVertex  () const; // vertex splits a single edge into two sharing the same curve
  SmBoolean IsOnPole             () const; // vertex marks a pole on one of the face->Surfaces to which it connects
  SmBoolean IsOnSeam             () const; // vertex marks a seam on one of the face->Surfaces to which it connects
  SmBoolean IsConnectedToEdge    (const SmEdge * cpEdge) const ;  // TRUE = vertex connects to target edge
  SmBoolean IsConnectedToFace    (const SmFace * cpFace) const ;  // TRUE = vertex connects to edge that connects to target face
  SmBoolean IsZoneTol3dConsistent(SmZoneTol3d * pOptZoneTol3d=NULL) const ; // TRUE = SM_ARE_SAME(m_sZoneTol3d == SmTol::GetZoneTol3d(this,0,TRUE))
  virtual SmBoolean IsConnectedTo(const SmTopology * cpConnectTgt) const ; 

  SmBoolean IsWithinXSectTol3dOfFaces
  (
    double    * pOptMaxGap3d=NULL,         ///< [out]: optional max Vertex/Face gap3d found
    SmFace   ** pOptMaxGapFace = NULL,     ///< [out]: optional associated Face for found max Edge/Face gap3d
    SmBoolean   bForceCalc = FALSE         ///< [in ]: TRUE = force gap evaluation, FALSE=use cache if available
  ) const ; 

  SmBoolean IsWithinXSectTol3dOfEdges
  (
    double    * pOptMaxGap3d=NULL,         ///< [out]: optional max Vertex/Edge gap3d found
    SmBoolean   bForceCalc = FALSE         ///< [in ]: TRUE = force gap evaluation, FALSE=use cache if available
  ) const ; 

  SmBoolean IsOnFaceCorner       
  (
    SmBoolean * pbOptFoundPoint=NULL,     ///< [out]: TRUE = found a Point on connected faces, FALSE=didn't
    double    * pdOptMaxGap3d=NULL,       ///< [out]: when bFoundPoint == TRUE, Max VertexFace gap from current vertex position
    SmPoint3d * pOptBestPoint=NULL        ///< [out]: When bFoundPoint == TRUE, Best possible vertex position to minimize Vertex/Face gaps
  ) const ;                           

  SmBoolean AreEdgesTangent      
  (
    const SmEdge * pE1,                   ///< [in ]:
    const SmEdge * pE2,                   ///< [in ]:
    double         dAngleTolDeg           ///< [in ]:
  ) const;

  SmBoolean HasParallelFaces     
  (
    double    * pOptAngTolDeg=NULL,              ///< [in ]: optional max angle deg between parallel vectors,
                                                 ///<      :  NULL=SM_ANG_TOL_DEG, default:[NULL]
    double    * pOptMinDihedralAngDeg = NULL,    ///< [out]: Min Dihedral AngDeg seen between Faces
    double    * pOptMaxParallelFaceGap = NULL,   ///< [out]: when Rtn == TRUE, Max gap between parallel face pairs
    SmFace   ** pOptFace1 = NULL,                ///< [out]: when Rtn == TRUE, optional face1 of MaxParallelFace/Face Gap pair
    SmFace   ** pOptFace2 = NULL,                ///< [out]: when Rtn == TRUE, optional face2 of MaxParallelFace/Face Gap pair
    SmPoint2d * pOptUV1 = NULL,                  ///< [out]: when Rtn == TRUE, optional face1 UVPt of MaxParallelFace/Face Gap pair
    SmPoint2d * pOptUV2 = NULL                   ///< [out]: when Rtn == TRUE, optional face2 UVPt of MaxParallelFace/Face Gap pair
  )const ;

  // TRUE = every use of this Vertex as a VertexLoop has consistent pointers
  SmBoolean CheckPointers() const ;  

  // TRUE  = vertex is part of a tolerant corner where more than 3 faces come together not at a single point.
  SmBoolean IsComplexCornerMember                                                        
  (
    SmTArray<SmTopology*> *pOptComplexCornerElements=NULL,      ///< [out]: optional list of vertices and edges making up the corner
                                                                ///<      : NULL to ignore, default:[NULL].
    SmTArray<SmFace*>     *pOptComplexCornerFaces=NULL          ///< [out]: optional list of faces meeting at this corner
                                                                ///<      : NULL to ignore, default:[NULL].
  ) const;

#ifdef SM_TOLERANT_WATCH
  SmBoolean IsExact() const               { return m_bExact ; }
  void      SetExact(SmBoolean bExact)    { m_bExact = bExact ; }
#else // no SM_TOLERANT_WATCH
  SmBoolean IsExact() const               { return TRUE ; }
  void      SetExact(SmBoolean)           { }
#endif // no SM_TOLERANT_WATCH

  // modify data access
  void     SetPoint     (const SmPoint3d & crPoint) ;
  void     SetPolyVertex(SmPolyVertex *pPolyVertex) ;

  SM_OLDTOL_LINE // local tolerance management
  SM_OLDTOL_LINE virtual SmZoneTol3d GetTolerance() const { return(m_sZoneTol3d) ; }   // old Tol ZoneTol3d stored on the object
  SM_OLDTOL_LINE virtual void        SetTolerance(SmZoneTol3d sNewZoneTol3d,          // in : Desired New ZoneTol3d Value
                                                  SmBoolean bUpdateOnlyIfLarger=TRUE, // in : default:[TRUE] = set tolerance only if sZoneTol3d > m_sZoneTol3d
                                                                                      //      FALSE          = always set tolerance (allow shrinking TolValues)
                                                  SmBoolean bCascadeToBndries=TRUE) ; // NotUsed: in : not used here: default:[TRUE] = Chg VtxTol to be >= EdgeTol (previous behavior)
                                                                                      //                     FALSE          = never change a VtxTol

  // Assign Edge and Faces connected to Edge consistent tolerance values
  SM_OLDTOL_LINE SmStatus            RefreshTolerance(SmBoolean bNestedRefreshes=TRUE) ;  

  SM_NEWTOL_LINE // local tolerance management 
  SM_NEWTOL_LINE //   inherited from SmTopology 
  SM_NEWTOL_LINE //     tolerance model: for pinholes in battleships (rarely used) - default:[FALSE] (99.9% of the time - leave it that way)
  SM_NEWTOL_LINE //        SmBoolean   IsSmallTopology() const ;
  SM_NEWTOL_LINE //        void        SetIsSmallTopology(SmBoolean bIsSmall) ;
  SM_NEWTOL_LINE //        
  SM_NEWTOL_LINE //     tolerance model: obsolete old-style compatible - instead use SmTol::GetZoneTol3d(this) ; 
  SM_NEWTOL_LINE //        SmZoneTol3d GetTolerance() const                   { return SmTol::GetZoneTol3d(this) ; } // GWC: obsolete
  SM_NEWTOL_LINE //        void        SetTolerance(SmZoneTol3d, SmBoolean)   { /* no action - only for backward compatibility */ ; }

  // Set Point location based on surf/surf/surf intersections if possible
  SmStatus RefineGeometry
  (
    SmMarkType eMarkType,               ///< [in ]: SM_MT_NOMARK = ignore marks else skip vertices which are currently marked
                                        ///<      :  mark all other vertices
    SmBoolean &bMadeChange              ///< [out]: TRUE = Changed VertexPoint to a surf/surf/surf intersection point
                                        ///<      :     and modified associated tolerance.
                                        ///<      : FALSE= no changes made
  ) ;

  // remove connection to specified edge
  SmVertexuse * FindAndRemoveVertexuseByEdge(SmEdge * pEdge) ;

  virtual void Notify
  (
    SmNotifyOperation eNotifyOperation,
    SmObject        * pData1,
    SmObject        * pData2,
    SmObject        * pData3
  ) ;

  // utilities
  SmStatus OutputGraphics(SmGfxArraySet * pOptGfxSet=NULL) const;

  virtual SmDisplayList * Draw         (SmGfxArraySet * pOptGfxSet=NULL) const ;
  SmDisplayList         * DrawNeighbors(SmGfxArraySet * pOptGfxSet=NULL) const ;
  SmDisplayList         * DrawMicro    (SmGfxArraySet * pOptGfxSet=NULL) const ;
                                                      
  void            Dump         (const TCHAR * message) const ;
  void            Dump         (ULONG)                 const ;
  void            Dump         (SmBoolean bAbbrev)     const ;  ///< NotUsed: [in ]: TRUE = Vertex ptr, Num Edges, Num Faces, point
                                                                //      FALSE= pluse Vertexuse, connectedEdge and connectedFace list reports
  void                    DumpTopology (ULONG lWalkDepth=0)    const ;  // NotUsed: in : lWalkDepth[0] = no walk, [99] = walk to bottom

  // get memory used for vertex, all its vertexuses and their attributes
  ULONG                   GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes
    SmMarkType eMarkType=SM_MT_NOMARK  ///< [in ]: uses without increment eMarkType value
  ) const ;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                              //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                              //      default:[SM_LEVEL_0] 
    SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
                         
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmVertex,SmOwningTopology,SmVertex_TYPE);

} ; // end class SmVertex

// GWC:BIND_TEMPLATES_MOVE  SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertex*) ;

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    extern SmVertex * dbgVertex1 ;
//    extern SmVertex * dbgVertex2 ;
//    
//    #endif // SM_DEBUG_CODE


#endif // !__SMVERTEX_H__

