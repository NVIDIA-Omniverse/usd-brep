// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmVertexuse.h
* PURPOSE: Header file for SmVertexuse class.
**********************************************************************/

#ifndef __SMVERTEXUSE_H__
#define __SMVERTEXUSE_H__

#ifndef __SMTOPOLOGY_H__
#include <SmTopology.h>
#endif

#include <SmGap.h>

 /*******************************************************************//**
PURPOSE: convenience class containing all ComputeUVSector optional output arguments

NOTES: Only set in call SmVertexuse::ComputUVSector().
       if this class ever gets used elsewhere - it'll probably need 
       constructor, destructor, copy operator, equal operator.  For now
       it doesn't.
***********************************************************************/
class SM_EXPORT SmUVSectorIO
 {
  public:
   const SmVertexuse * m_pVertexuse = NULL ;               // in : vertexuse defining this sector

   SmEdgeuse         * m_pBegEdgeuse = NULL ;              // out: pBegEdgeuse   = sector BegEdgeuse (NotNULL for EdgeVertex, NULL for VertexLoops and VertexShells)
   SmEdgeuse         * m_pEndEdgeuse = NULL ;              // out: pEndEdgeuse   = sector EndEdgeuse (NotNULL for EdgeVertex, NULL for VertexLoops and VertexShells)
   double              m_dBegEdgeParam = SM_UNDEF_DOUBLE ; // out: dBegEdgeParam = sector BegEdge corner Param
   double              m_dEndEdgeParam = SM_UNDEF_DOUBLE ; // out: dEndEdgeParam = sector EndEdge corner Param

   SmPoint2d           m_sBegEdgeUVPoint ;                 // out: sBegEdgeUVPoint = UVPosition of Vertexuse marking the beg of this sector
   SmPoint2d           m_sEndEdgeUVPoint ;                 // out: sEndEdgeUVPoint = UVPosition of VertexUse marking the end of this sector
                                                           //                  = if(GetVertexuseType() == SmEdgeuse_TYPE)
                                                           //                      if(bVertexOnClosed)          Other UVPoint mapping to same 3dPoint
                                                           //                      else if(bVertexOnSingulariy) Other end of the Singularity bndry Ivl 
                                                           //                                                    mapping to this vertex
                                                           //                      else                         Should be same UVPoint as UVBegEdgePoint.
                                                           //                      else                         SetUninitialized()
   SmPoint2d           m_sBegEdgeUVTan ;                   // out: sBegEdgeUV1stDir = UV unitized tangent from the SectorPoint along BegSector arm
   SmPoint2d           m_sEndEdgeUVTan ;                   // out: sEndEdgeUV1stDir = UV unitized tangent from the SectorPoint along EndSector arm
   double              m_dSectorAngDeg ;                   // out: UVSpace rotation from BegTan to EndTan in the UVplane corner, range:[0 360]                                                              

   SmBoolean           m_bVertexOnClosedU    = FALSE ;     // out: bVertexOnClosedU:     TRUE = SectorVtx on closedU bndry, Pos[Umin,v] == Pos[Umax,v] for v smps
   SmBoolean           m_bVertexOnClosedV    = FALSE ;     // out: bVertexOnClosedV:     TRUE = SectorVtx on closedV bndry, Pos[u,Vmin] == Pos[u,Vmax] for u smps
   SmBoolean           m_bVertexOnPole       = FALSE ;     // out: bVertexOnSingularity: TRUE = SectorVtx on Singularity
                                                              
   SmBoolean           m_bVertexOnFlatCorner = FALSE ;     // out: bVertexOnFlatCorner: TRUE = Surface TangU parallel to TangV
   SmBoolean           m_bSectorMissingSeamU = FALSE ;     // out: bSectorMissingSeamU: TRUE = Sector contains a missing Udir Seam 
   SmBoolean           m_bSectorMissingSeamV = FALSE ;     // out: bSectorMissingSeamV: TRUE = Sector contains a missing Vdir Seam 

   void Dump() const ; 

 } ; // end SmUVSectorIO

/*******************************************************************//**
PURPOSE: This class represents topological Vertexuses.

NOTES: 
  for EdgeuseTYPE vertexuses:
   Sectors: When a Vertexuse connects to an Edgeuse and the Edgeuse connects to a Loopuse,
            the vertex use represents a FaceSector. 
     FaceSector = Part of the Face inside the corner marked by 
                   1. Vertexuse->Vertex
                   2. Start Vec pointing from Vertex into Face
                   3. Stop  Vec pointing from Vertex into Face
                   4. SectorAngDeg measured from StartVec to EndVec about associated Faceuse->NormalVec
     (think of pie slices - bigger SectorAngDeg values, bigger slices of pie from this corner)
     The start Vertexuse->Edgeuse runs from the vertex into the face
     The end   Edgeuse = Vertexuse->Edgeuse->CWEU runs from the face into the vertex

     If the loop is an outer counterclockwise oriented loop (VU->EU->LU->m_eOrientation == SM_OT_SAME)
       then a person walking the loop in the direction of the head to tail edgeuses would
         arrive at this vertex sector walking along the EndEdgeuse and
         leave this vertex sector walking along the BegEdgeuse
     and the inside of the face is this vertexuses sector which would always lie to the
     left hand side of the walking person.

     If the loop is an inner clockwise oriented loop (VU->EU->LU->m_eOrientation == SM_OT_OPPOSITE)
       then a person walking the loop in the direction of the head to tail edgeuses would
         arrive at this vertex sector walking along the BegEdgeuse and
         leave this vertex sector walking along the EndEdgeuse
     and the inside of the face is this vertexuses sector which would always lie to the
     left hand side of the walking person.

    The vertex sectors for SM_OT_SAME and SM_OT_OPPOSITE oriented loops share the same
    Beg and End UVTan vectors but the sectors are compliments of one another,
      so SM_OT_SAME Loop VertexSectorAngle + SM_OT_OPPOSITE loop VertexSectorAngle = 360 degrees.

    A person walking along the Edgeuse from Start to End on the positive side
    of the faceuse will have the interior of the face to his lefthand side.
    That means Sector StartEU = pVU->EU and
                      EndEU   = pVU->EU->CWEU
***********************************************************************/
class SM_EXPORT SmVertexuse : public SmTopology
{
  friend class SmBrep;
  friend class SmVertex;
  friend class SmEdge;
  friend class SmShell;
  friend class SmLoop;
  friend class SmBrepConstructor;

protected:
  // inherited:
  // SmTopology::m_pListOwner - used to store pointer to owning SmVertex
  // SmTopology::m_pNext - doubly linked-list of Vertexuses that belong to SmVertex
  // SmTopology::m_pLast

  SM_TYPE          m_tVertexuseType;   // oneof: SmShell_TYPE   (16013 connects to Region thru Shell - ShellVertex in region)
                                       //        SmLoopuse_TYPE (16004 connects to Face thru Loopuse - LoopVertex in face)
                                       //        SmEdgeuse_TYPE (16005 connects to Edge thru Edgeuse - EdgeVertex bounding edge in face
                                       //                                                      or EdgeVertex bounding edge in region)
                                         
  SmTopology     * m_pSorLUorEU;       // when m_tVertexuseType == SmShell_TYPE   ptr to SmShell,     (vertex in a region)
                                       // when m_tVertexuseType == SmLoopuse_TYPE        SmLoopuse or (vertex in a face)    
                                       // when m_tVertexuseType == SmEdgeuse_TYPE        SmEdgeuse    (vertex part of a loop in a face or a region)

  mutable SmVertexEdgeGap  m_sVertexEdgeGap3d; // only for SmEdgeuse_TYPE
                                               //   Gap between Vertex->Point and Edge->Curve->EndPoint
                                               //   m_sVertexEdgeGap3d->m_pVertexuse == NULL
                                               //   for uninitialized gaps and Vertexuses not connected to Edge

  mutable SmVertexFaceGap  m_sVertexFaceGap3d; // only for SmEdgeuse_TYPE and SmLoopuse_TYPE
                                               //   Gap between Vertex->Point and Face->Surface
                                               //   m_sVertexFaceGap3d->m_pVertexuse == NULL 
                                               //   for uninitialized gaps and Vertexuses not connected to Face

  SmPoint2d m_sUVPoint = SmPoint2d(SM_BIG_DOUBLE, SM_BIG_DOUBLE);

  // note: mutable SmVertexFaceTrimCurveGap m_sVertexFaceTrimCurve3d ;
  //       Gap3d is not cached because it is not part of the consistent tolerance model.
  //       if the Vertex/FaceTrimCurveGap becomes part of the consistent tolerance model - cache it.
public:
  // constructor                                       
  SmVertexuse() : m_tVertexuseType(SmUnknown_TYPE), 
                  m_pSorLUorEU(NULL),
                  m_sUVPoint(SmPoint2d(SM_BIG_DOUBLE, SM_BIG_DOUBLE))
                { 
                  // report construction at SmObject::Notify level - skip other levels
                   SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL); 
                }

  // destructor
  virtual ~SmVertexuse() { Notify( SM_NO_DESTRUCTION, this, NULL, NULL );
                           m_tVertexuseType = SmUnknown_TYPE;
                           m_pSorLUorEU = NULL;
                           // destructor automatically called on 
                           //   m_sVertexEdgeGap3d
                           //   m_sVertexFaceGap3d
                           // which calls ReSet() before deallocating.
                         }

  void SetUVPoint(SmPoint2d sUVPoint) { this->m_sUVPoint = sUVPoint; }

  // drop vertex point onto face->surface, rtn SM_SUCCESS=Vertex connects to Face else rtn SM_ERR (Shell and WireEdge vertices)
  SmStatus ComputeUVPoint(SmPoint2d & rUVPoint,                 // out: Projection of Vertexuse->Vertex->Point onto                 
                                                                //    : Vertexuse->Edgeuse->Loopuse->Faceuse->Face->Surface         
                          SmBoolean   bOKToMakeUVTrimCurve=TRUE // in : TRUE  = Use LocalSolve with guesses from UVTrimCurve        
                                                                //    : FALSE = don't make UVTrimCurve, if missing use GlobalSolve  
                         ) const;                                   

  // drop Vertex point and connected EdgeTangents onto face->Surface, rtn: SM_SUCCESS for EdgeVerts connected to face, else rtn SM_ERR
  SmStatus ComputeUVSector(SmUVSectorIO & rUVSectorIO,               // out: Optional Sector Properties.  NULL to ignore, default:[NULL]
                                                                     //      contains proper UVStartPt and UVStartPt vals for Sectors on Poles.
                           SmBoolean      bOKToMakeUVTrimCurve=TRUE) // in : TRUE  = Use LocalSolve with guesses from UVTrimCurve                       
                                                                     //      FALSE = don't make UVTrimCurve, if missing use GlobalSolve, default:[TRUE] 
                          const ;                                    //                                                                                 
                           // where SmUVSectorIO * pOptIO contains:  // 
                           //   SmEdgeuse *    m_pBegEdgeuse           // out: pBegEdgeuse = sector BegEdgeuse
                           //   SmEdgeuse *    m_pEndEdgeuse           // out: pEndEdgeuse = sector EndEdgeuse
                           //   double         m_dBegParam             // out: dBegParam   = sector BegEdge corner Param 
                           //   double         m_dEndParam             // out: dEndParam   = sector EndEdge corner Param
                           // 
                           //   SmPoint2d      m_sBegEdgeUVPoint ;    // out: sBegEdgeUVPoint = UVPosition of Vertexuse marking the beg of this sector
                           //   SmPoint2d      m_sEndEdgeUVPoint ;    // out: sEndEdgeUVPoint = UVPosition of VertexUse marking the end of this sector
                           //                                         //                  = if(GetVertexuseType() == SmEdgeuse_TYPE)
                           //                                         //                      if(bVertexOnClosed)          Other UVPoint mapping to same 3dPoint
                           //                                         //                      else if(bVertexOnSingulariy) Other end of the Singularity bndry Ivl 
                           //                                         //                                                    mapping to this vertex
                           //                                         //                      else                         Should be same UVPoint as UVBegEdgePoint.
                           //                                         //                      else                         SetUninitialized()
                           //   SmPoint2d      m_sBegEdgeUVTan ;   // out: sBegEdgeUV1stDir = UV 1st Derivative from the SectorPoint along BegSector arm
                           //   SmPoint2d      m_sEndEdgeUVTan ;   // out: sEndEdgeUV1stDir = UV 1st Derivative from the SectorPoint along EndSector arm
                           //   double         m_dSectorAngDeg ;      // out: UVSpace rotation from BegTan to EndTan in the UVplane corner, range:[0 360]                                                              
                           //
                           //   double         m_bVertexOnClosedU      // out: bVertexOnClosedU:     TRUE = SectorVtx on closedU bndry, Pos[Umin,v] == Pos[Umax,v] for v smps
                           //   double         m_bVertexOnClosedV      // out: bVertexOnClosedV:     TRUE = SectorVtx on closedV bndry, Pos[u,Vmin] == Pos[u,Vmax] for u smps
                           //   SmBoolean      m_vVertexOnPole  // out: bVertexOnSingularity: TRUE = SectorVtx on Singularity
                           //   SmBoolean      m_bSectorMissingSeamU   // out: bSectorMissingSeamU: TRUE = Sector contains a missing Udir Seam 
                           //   SmBoolean      m_bSectorMissingSeamV   // out: bSectorMissingSeamV: TRUE = Sector contains a missing Vdir Seam 
                                                                        
  // Sector Turning AngDeg [-180 to 180]
  double   GetTurningAngDeg() const ;

  // get vertexuse->edgeuse->edge->EndPointParam that should connect to vertexuse->vertex->Point
  SmStatus ComputeTPoint(double &dT) const ;

  // simple access
  SM_TYPE             GetVertexuseType() const { return (m_tVertexuseType) ; }

  SmPoint2d           GetUVPoint() const { return (m_sUVPoint) ; }

  SmEdgeuse         * GetEdgeuse()       const; // Edgeuse_type: Connected Edgeuse  (Vertex connects to 1 or more Faces)
                                                // Loopuse_type: NULL               (Vertex connects to 1 Face)
                                                // Shell_type  : NULL               (Vertex not connected to Face)

  SmFaceuse         * GetFaceuse()       const; // Edgeuse_type: Connected Edgeuse->GetFaceuse() = Faceuse of FaceSector started by this Vertexuse. (Vertex connects to 1 or more Faces) 
                                                // Loopuse_type: Connected Loopuse->GetFaceuse(). (Vertex connects to 1 Face)
                                                // Shell_type  : NULL.                            (Vertex not connected to Face)

  SmLoopuse         * GetLoopuse()       const; // Edgeuse_type: Connected Edgeuse->GetLoopuse()
                                                // Loopuse_type: Connected Loopuse
                                                // Shell_type  : NULL

  virtual SmShell   * GetShell()         const; // Edgeuse_type: Connected Edgeuse->GetShell()
                                                // Loopuse_type: Connected Loopuse->GetShell()
                                                // Shell_type  : Connected Shell

  virtual SmBrep    * GetBrep()          const; // Edgeuse_type: Connected Edgeuse->GetShell()->GetBrep()
                                                // Loopuse_type: Connected Loopuse->GetShell()->GetBrep
                                                // Shell_type  : Connected Shell->GetBrep()

  SmVertex          * GetVertex()        const { return (SmVertex*)GetOwner(); }

  // Vertex/Edge Gaps
  // Vertex/Face Gaps
  SmVertexEdgeGap  * GetVertexEdgeGap    (SmBoolean bForceCalc=FALSE) const ; // rtn: Vertex/EdgeEnd (possibly updated) cached value 
  SmVertexFaceGap  * GetVertexFaceGap    (SmBoolean bForceCalc=FALSE) const ; // rtn: Vertex/SurfaceDrop (possibly updated) cached value

  SmBoolean          HasVertexEdgeGap    () const ;
  SmBoolean          IsVertexEdgeGapInit () const                 { return( m_sVertexEdgeGap3d.IsInit()) ; }
  SmBoolean          IsVertexEdgeGapFresh(SmVertexEdgeGap & rVertexEdgeGap, 
                                          SmVertexEdgeGap & rStoredVertexEdgeGap) const ;
  SmBoolean          IsVertexEdgeGapSet  () const { return( m_sVertexEdgeGap3d.GetVertexuse() != NULL) ; }
  void               SetVertexEdgeGap    (const SmVertexEdgeGap & rGap) const { m_sVertexEdgeGap3d = rGap ; }
  void               InitVertexEdgeGap   () const { m_sVertexEdgeGap3d.ReSet() ; }

  SmBoolean          HasVertexFaceGap    () const ;
  SmBoolean          IsVertexFaceGapInit() const { return(m_sVertexFaceGap3d.IsInit()); }
  SmBoolean          IsVertexFaceGapFresh(SmVertexFaceGap & rVertexFaceGap,  SmVertexFaceGap & rStoredVertexFaceGap) const ;
  SmBoolean          IsVertexFaceGapSet  () const { return( m_sVertexFaceGap3d.GetVertexuse() != NULL) ; }
  virtual SmBoolean  IsConnectedTo       (const SmTopology *cpConnectTgt) const ;

  void               SetVertexFaceGap    (const SmVertexFaceGap & rGap) const { m_sVertexFaceGap3d = rGap ; }
  void               InitVertexFaceGap   () const { m_sVertexFaceGap3d.ReSet() ; }

  // Vertex/FaceTrimCurve gaps
  SmBoolean                HasVertexFaceTrimCurveGap() const ;
  SmVertexFaceTrimCurveGap GetVertexFaceTrimCurveGap(SmBSplineCurve * pOptUVTrimCurve=NULL) const; ///< [in ]: for internal use only // note: computes Vertex/FaceTrimCurveEnd gap value everytime
                                                                             

  SmStatus RefreshGaps() const
  {
    SmStatus sRtn1 = CalcVertexEdgeGap( NULL, TRUE );
    SmStatus sRtn2 = CalcVertexFaceGap( NULL, TRUE );
    return((sRtn1 == SM_SUCCESS && sRtn2 == SM_SUCCESS) ? SM_SUCCESS : SM_ERR);
  }

  // compute and optionally cache Vertex gaps (to edge and to face), for internal use - call Get...Gap() methods instead
  SmStatus CalcVertexEdgeGap(SmVertexEdgeGap * pOptVertexEdgeGap=NULL, SmBoolean bCache=TRUE) const ; // vertex/EdgeEnd gap
  SmStatus CalcVertexFaceGap(SmVertexFaceGap * pOptVertexFaceGap=NULL, SmBoolean bCache=TRUE) const ; // vertex/FaceDrop gap

  // predicates
  SmBoolean IsLoopVertexuse()  const { return(m_tVertexuseType == SmLoopuse_TYPE) ; } // vertex connects directly to face;   m_pSorLUorEU points to Loopuse  
  SmBoolean IsShellVertexuse() const { return(m_tVertexuseType == SmShell_TYPE)   ; } // vertex connects directly to region; m_pSorLUorEU points to Shell
  SmBoolean IsEdgeVertexuse()  const { return(m_tVertexuseType == SmEdgeuse_TYPE) ; } // vertex connects to edge;            m_pSorLUorEU points to Edgeuse
  SmBoolean IsStrutVertexuse() const; // vertex connects to edge which is not connected to another edge at this end. 
  SmBoolean IsOnPole()         const; // vertex connects to a pole on one of the surfaces to which it connects
  SmBoolean IsOnSeam()         const; // vertex connects to a seam on one of the surfaces to which it connects
  SmBoolean IsSectorMissingSeam(SmBoolean *pOptSectorMissingSeamU=NULL, 
                                SmBoolean *pOptSectorMissingSeamV=NULL,
                                SmBoolean  bUVSpaceOkay=TRUE                 // [in] : default:[TRUE] = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) <br>
                                                                             //        FALSE = don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE] <br>
                                ) const ; // valid Brep always returns FALSE
  SmBoolean IsSectorGapFreeAndOriented() const ;

  // maintenance
  virtual void Notify(SmNotifyOperation eNotifyOperation,
                      SmObject        * pData1,
                      SmObject        * pData2,
                      SmObject        * pData3) ;

  SmDisplayList * Draw     (SmGfxArraySet * pOptGfxSet=NULL) const ;
  SmDisplayList * DrawMicro(SmGfxArraySet * pOptGfxSet=NULL) const ;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                             <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                               <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmVertexuse,SmTopology,SmVertexuse_TYPE);

} ; // end class SmVertexuse

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    extern SmVertexuse * dbgVertexuse1 ;
//    extern SmVertexuse * dbgVertexuse2 ;
//    
//    #endif // SM_DEBUG_CODE

#endif // !__SMVERTEXUSE_H__

