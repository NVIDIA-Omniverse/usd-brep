// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmEdgeuse.h
* PURPOSE: Header file for SmEdgeuse class. 
**********************************************************************/

#ifndef __SMEDGEUSE_H__
#define __SMEDGEUSE_H__

#ifndef __SMTOPOLOGY_H__
#include <SmTopology.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMLOOPUSE_H__
#include <SmLoopuse.h>
#endif

#include <SmGap.h>
#include <SmSurface.h> // added only to gain access to the SmDropCurveFail object - this could be cleaned up

class SmDerivSurfDefinition;
class SmVertex;

/*******************************************************************//**
PURPOSE: This class represents topological edgeuses within a Brep.

NOTES: 
***********************************************************************/
class SM_EXPORT SmEdgeuse : public SmTopology
{
  friend class SmBrep;
  friend class SmEdge;
  friend class SmShell;
  friend class SmFace;
  friend class SmLoop;
  friend class SmLoopuse;
  friend class SmVertexuse;
  friend class SmBrepConstructor;
  friend class SmFaceRemoval;
  friend class SmBrepValidationTestAccess;

protected:
  // inherited:
  // SmTopology::m_pListOwner - used to store pointer to owning SmEdge
  // SmTopology::m_pNext      - used to store doubly linked list of
  // SmTopology::m_pLast            mate and radial Edgeuse partners
  //                                in Radial order using the right hand rule.
  // note: The Edgeuse linked list for SmLoopuse_TYPE edgeuses -
  //    1.  if(m_eOrientation == OwningEdge->PrimaryEdgeUse->m_eorientation)
  //             {mate = m_pNext ; radial = m_pLast ; }
  //        else {mate = m_pLast ; radial = m_pNext ; }
  //    2. The m_pNext linked list of Edgeuses is ordered as 
  //          [PrimaryEdgeuse -> Mate -> Radial -> Mate -> . . . -> Radial -> PrimaryEdgeuse]
  //       So, for Edge->EdgeuseList:  elems [0 1] [2 3] ... [n-2 n-1] are mated pairs on the same Face and
  //         where: n=Edge->ListSize:  elems [1 2] [3 4] ... [n-1  0 ] are radial pairs between Faces bounding Edge Sectors within one Shell
  //                 i.e., List[(2*ii)%ListSize]  <- Mate ->List[(2*ii+1)%ListSize]  = mated pairs
  //                 i.e., List[(2*ii+1)%ListSize]<-Radial->List[(2*ii+2)%ListSize]  = radial pairs                             
  //    3. Every mated  pair of Edgeuses share a common Face and 
  //       Every radial pair of Edgeuses share a common Shell as  
  //        mated pair : Face1  == Face2,  Face1 ->Faceuse-Loopuse->Edgeuse- Mate -Edgeuse->Loopuse->Faceuse->Face2 ; 
  //        radial pair: Shell1 == Shell2, Shell1->Faceuse-Loopuse->Edgeuse-Radial-Edgeuse->Loopuse->Faceuse->Shell2; 
  //    5. The radial ordered edgeuse list of orientations always follows a
  //          SAME-OPPOSITE-SAME-... pattern starting on either SAME or OPPOSITE
  //    6. EdgeuseStartParam  = m_eOrientation == SAME ? Edge->m_vInterval->GetMin : Edge->m_vInterval->GetMax ;
  //       EdgeuseStartVertex = m_eOrientation == SAME ? m_pVU->Vertex             : Mate->m_pVU->Vertex (or CCWEdgeuse->m_pVU->Vertex) ;
  //       EdgeuseEndParam    = m_eOrientation == SAME ? Edge->m_vInterval->GetMax : Edge->m_vInterval->GetMin ;
  //       EdgeuseEndVertex   = m_eOrientation == SAME ? Mate->m_pVU->Vertex (or CCWEdgeuse->m_pVU->Vertex) : m_pVU->Vertex ; 

  SM_TYPE                  m_tEdgeuseType ;          // oneof: SmShell_TYPE    (16013 = connects to Region thru Shell - wire in Region)
                                                     //        SmLoopuse_TYPE  (16004 = connects to Face thru Loop    - edge in Face)
  SmTopology             * m_pSorLU ;                // Pointer to either shell or loopuse
  SmEdgeuse              * m_pCCW ;                  // Counter Clockwise Edgeuse in the loop, NULL for wire edges
  SmEdgeuse              * m_pCW ;                   // Clockwise Edgeuse in the loop, NULL for wire edges
                                                     //   thisEdgeuse->StartPt == CWEdgeuse->EndPt (with tolerances)
                                                     //   thisEdgeuse->EndPt   == CCWEdgeuse->StartPt (with tolerances)
  SmVertexuse            * m_pVU ;                   // SmVertexuse attached to this edgeuse
                                                       
  SmOrientType             m_eOrientation ;          // SM_OT_SAME     = Edgeuse interval = [Edge->m_vInterval->Getmin,Edge->m_vInterval->Getmax]
                                                     // SM_OT_OPPOSITE = Edgeuse interval = [Edge->m_vInterval->Getmax,Edge->m_vInterval->Getmin]
                                                       
  mutable SmBSplineCurve * m_pUVTrimCurve ;          // only for SmLoopuse_TYPE: m_eOrientation == SM_OT_SAME
                                                     //   2D curve in parameter space - must be oriented same
                                                     //   as corresponding 3D curve attached to the edge.
                                                     //   The curve is stored once for every pair of Edgeuse mates
                                                     //   on the Edgeuse whose orientation == SM_OT_SAME.
                                                       
  mutable SmEdgeFaceGap    m_sMaxEdgeFaceGap3d ;     // only for SmLoopuse_TYPE: (gwc: not currently part of the tol model - but could be)
                                                     //   MaxGap3d between Edge->Curve and Face->Surface 
                                                     //   m_sMaxEdgeFaceGap3d->m_pEdgeuse == NULL for uninitialized gaps

  mutable SmEdgeEdgeGap    m_sEdgeCCWEdgeGap3d ;     // only for SmLoopuse_TYPE: (gwc: not currently part of the tol model - but could be)
                                                     //   Gap3d between EdgeuseEndPt->Edge->Curve->Pt and CCWEdgeuseStartPt->Edge->Curve->Pt
                                                     //   m_sMaxEdgeCCWEdgeGap3d->m_pEdgeuse0 == NULL for uninitialized gaps

  // GWC some kind of flag to communicate that CreateUVTrimCurve failed - cleared with any call to SetUVTrimCurve()
  mutable SmDropCurveFail * m_pDropCurveFail ;       // a nonpersistent object containing state from the last 
                                                     // CreateUVTrimCurve call - ReSet() with call to SetUVTrimCurve()

protected:
  SmEdgeuse();            

public:
  virtual ~SmEdgeuse();

  // create UVTrimCurve and its UVTrimCurve/Curve tolerance but do not attach to this Edgeuse.
  SmStatus CreateUVTrimCurve
  (
    double          & rdMaxDistToSurf,            // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.     
    SmBSplineCurve *& rpNewUVCurve,               // out: created UVTrimCurve                                                                      
    double          * pdMaxDeviation=NULL,        // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0  
    SmDropCurveFail * pOptDropCurveFail=NULL      // out: Opt state at time of DropCurve fail or success                                           
  ) const;

  // UVTrimCurve management: UVtrimCurve stored once for every Edgeuse/Mate pair (on Edgeuse with orientation == SM_OT_SAME) 
  SmBSplineCurve  * GetUVTrimCurve  (SmEdgeuse **pOptOwner=NULL) const ;             // rtn: UVTrimCurve from Edgeuse or Mate. Return NULL if none attached
  SmDropCurveFail * GetDropCurveFail(SmEdgeuse **pOptOwner=NULL) const ;             // rtn: State Data saved when UVTrimCurve was created
  SmStatus          GetOrCreateUVTrimCurve
  (
    SmCurve *& rpUVTrimCurve,                 // out: UVTrimCurve attached to Edgeuse/Mate pair                                                                           
    double   * dOptMaxDistToSurf=NULL,        // out: for New UVTrimCurve: MaxDropDist(Edge->CrvPt, SrfDropPt), found by sampling, may be slightly less than actual max.  
                                              //    : for Old UVTrimCurve: -1.0                                                                                           
    double   * dOptDeviation =NULL,           // out: for New UVTrimCurve: MaxDropDist(SrfDropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0 
                                              //    : for Old UVTrimCurve: -1.0                                                                                           
    SmBoolean  bOKToMakeUVTrimCurve=TRUE      // in : default:[TRUE] = okay to build UVTrimCurve if it does not exist 
   ) ;                                        //    : FALSE= return NULL when UVTrimCurve does not exist because during healing expect DropCurve to Fail 


  SmStatus          GetOrCreateUVTrimCurve
  (
    SmBSplineCurve *& rpBSC,                  // out: UVTrimCurve attached to Edgeuse/Mate pair                                                                           
    double * dOptMaxDistToSurf=NULL,          // out: for New UVTrimCurve: MaxDropDist(Edge->CrvPt, SrfDropPt), found by sampling, may be slightly less than actual max.  
                                              //    : for Old UVTrimCurve: -1.0                                                                                           
    double  * dOptDeviation =NULL,            // out: for New UVTrimCurve: MaxDropDist(SrfDropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0 
                                              //    : for Old UVTrimCurve: -1.0                                                                                           
    SmBoolean bOKToMakeUVTrimCurve=TRUE       // in : default:[TRUE] = okay to build UVTrimCurve if it does not exist 
   ) ;                                        //    : FALSE= return NULL when UVTrimCurve does not exist because during healing expect DropCurve to Fail 

  // Set UVTrimCurve, and UVTrimCurve->Owner, increase EdgeTol as needed
  // pUVTrimCurve Ownership changes to SmEdgeuse, caller should not delete 
 void              SetUVTrimCurve
  (
    SmBSplineCurve * pUVTrimCurve,            // in : New uv curve                                                                   
    double           dMaxDistToSurf,          // in : MaxDropDist(Edge->CrvPt, SrfDropPt), 0.0 to ignore                             
    SmBoolean        bDeleteFlag=FALSE,       // in : TRUE = delete existing UVTrimCurve, FALSE=don't                                
    SmDropCurveFail *pOptDropCurveFail=NULL   // in : DropCurve() data when UVTrimCurve was created, NULL to ignore, default:[NULL]  
                                              //    : Makes a deep copy - owner needs to manage the memory for this object.          
  ) ;                                                                                                                                   

  // Set UVTrimCurve only if(MaxDropDist(Edge->CrvPt, SrfDropPt) < 100 * BrepTol 
  SmStatus          CheckAndSetUVTrimCurve
  (
    SmBSplineCurve  * pUVTrimCurve,           // in : New uv curve                                                                  
    SmBoolean         bAdjustTolerances,      // in : TRUE=Adjust Edgeuse->Edge->Tol >= 2 MaxDistToSurf                             
    double          & rdMaxDistToSurf,        // out: found MaxDropDist(Edge->CrvPt, SrfDropPt)                                     
    SmBoolean       & rbTrimReplaced,         // out: TRUE = pUVCurve attaced to Edgeuse as UVTrimCurve                             
                                              //    : FALSE= pUVCurve was not good enough, no changes made                          
    SmDropCurveFail * pOptDropCurveFail=NULL  // in : DropCurve() data when UVTrimCurve was created, NULL to ignore, default:[NULL] 
                                              //    : Makes a deep copy - owner needs to manage the memory for this object.         
  ) ;                                                                                                                                   
  void             DeleteUVTrimCurve()  { SetUVTrimCurve(NULL, 0.0, TRUE, NULL) ; } // eff: remove and delete existing UVTrimCurve
  SmStatus         RebuildUVTrimCurve() ;                                           // eff: replace existing UVTrimCurve with a newly built one
  SmStatus         RefreshUVTrimCurve() { return RebuildUVTrimCurve() ; }           // eff: backward compatible name for RebuildUVTrimCurve()

  // Connect this mated pair of Edgeuses with UVTrimCurve. (for internal use: otherwise see SetUVTrimCurve())
  void SetMemberUVTrimCurve
  (
    SmBSplineCurve  * pUVTrimCurve,             // in : ptr to place on this mated pair owner, no memory management 
    SmDropCurveFail * pOptDropCurveFail = NULL  // in : ptr to place on this mated pair owner, no memory management 
  )
  {
    SmEdgeuse * pOwner = (m_eOrientation == SM_OT_SAME) ? this : GetMate();
    pOwner->m_pUVTrimCurve = pUVTrimCurve;
    if(pOptDropCurveFail) { pOwner->m_pDropCurveFail = pOptDropCurveFail; }
    pUVTrimCurve->SetOwner( pOwner );
  }

  // create DerivSurface from surface boundary that captures pos and cross-deriv data along edgeuse.
  SmStatus CreateDerivativeSurface
  (
    SmDerivSurfDefinition & crDS,               // in : Construction parameters.                                                               
    SmBoolean               bIntoFace,          // in : TRUE = Derivs go into the surface, FALSE = they go out                                 
    SmBSplineSurface     *& rpDerivSurface      // out: the DerivSurface, use DerivSurface->Evaluate(s,0) to get D, Dv, Dvv, and Dvvv values.  
  );                                            

  // Get vector pointing towards inner part of face at point on edge, 
  // binormal = CrossProduct(SurfNormal(EdgePoint), EdgeuseTangent (EdgeTangent adjusted for orientation)
  //   A person walking along the Edgeuse from Start to End on the positive side 
  //   of the faceuse will have the interior of the face to his lefthand side.
  SmStatus EvaluateBinormal
  (
    double       dParameter,                 // in : value within Edge->m_vInterval. 
    SmBoolean    bUse3DOnly,                 // in : TRUE  = when pUVTrimCurve==NULL, proj Edge pts to Surface else use pUVTrimCurve 
                                             //      FALSE = get/create pUVTrimCurve to calc Surface points 
    SmPoint3d  & rBinormalPoint,             // out: 3D pt on edge 
    SmVector3d & rBinormal,                  // out: unit-vector pointing to SmFace interior from rBinormalPoint 
    SmVector3d * pOptEdgeuseTangent = NULL,  // out: opt non-unit Edgeuse tangent at rBinormalPoint, NULL to ignore, default:[NULL]                                     
    SmVector3d * pOptFaceuseNormal  = NULL,  // out: opt unit Faceuse normal, NULL to ignore, default:[NULL]                                                            
    SmPoint2d  * pOptUVParameter    = NULL,  // out: opt Surface param value at edge point, NULL to ignore, default:[NULL]                                              
    SmVector2d * pOptUVDir          = NULL   // out: opt Surface Binormal param direction at edge point, not unitized, NULL to ignore, default:[NULL]                   
  )  const ;

  // Get vector pointing towards inner part of face at point on the face a given distance from edge,
  // binormal = CrossProduct(SurfNormal(OffsetPoint), EdgeTangent)
  //   A person walking along the Edgeuse from Start to End on the positive side 
  //   of the faceuse will have the interior of the face to his lefthand side.
  SmStatus EvaluateBinormalStepOff
  (
    double dParameter,                           // in : curve parameter                                                    
    double dStepOffDistance,                     // in : 3d distance to step (0.0 is ok)                                    
    SmPoint3d  & rBinormalPoint,                 // out: Surface position for given param and StepOffDistance               
    SmVector3d & rBinormal,                      // out: rBinormal = CrossProduct(sNormal,sTangent at stepoff point         
    SmVector3d * pOptEdgeuseTangent = NULL,      // out: curve tangent at param, NULL to ignore                             
    SmVector3d * pOptEdgeuseFaceNormal = NULL,   // out: surface normal at stepOffPoint, NULL to ignore                     
    SmPoint3d  * pOptCenterPoint = NULL,         // in : surface point at step off distance, NULL to ignore                 
                                                 //    : default:[NULL] = use surfacePoint nearest EdgePoint at given param 
    SmVector3d * pOptPlaneNormal = NULL,         // in : define plane to measure step off distance, NULL to ignore          
                                                 //    : default:[NULL] = use EdgeuseTangent at given param                 
    SmVector2d * pOptStepOffUV = NULL,           // out: found surface StepOff UVPoint, NULL to ignore                      
    SmBoolean    bOKToMakeUVTrimCurve=TRUE       // in : default:[TRUE] = okay to build UVTrimCurves when missing  
                                                 //    : FALSE= Only use existing UVTrimCurves or work in 3d because UVTrimCurve Creation is dodgy during healing 
  ) const;

  // Evaluate surfaceUV or XYZ pts for Normalized edgeuse parameters
  SmStatus NormalizedEvaluate
  (
    double       dNormalizedParameter,     // in : 0 to 1 (start to end) target param value (accounts for orientation)                                   
    SmBoolean    bParameterSpaceEval,      // in : TRUE = Eval Edgeuse->UVTrimCurve for a Surface UV Point                                               
                                           //    : FALSE= Eval Edgeuse->Edge->Curve for a 3d Point                                                       
    SmPoint3d  & rPoint,                   // out: UV or XYZ value along Edgeuse for param (accounts for orientation)                                    
    SmVector3d * pOptFirstDeriv=NULL,      // out: Optional, 1st UV or XYZ deriv along Edgeuse at param point (accounts for orientation), NULL to ignore 
    SmBoolean    bOKToMakeUVTrimCurve=TRUE // in : default:[TRUE] = okay to build UVTrimCurves when missing                                              
                                           //    : FALSE= Only use existing UVTrimCurves or work in 3d because UVTrimCurve Creation is dodgy during healing  
  ) const ;                 

  // Evaluate surfaceUV pts for Normalized edgeuse parameters without creating UVTrimCurves
  SmStatus NormalizedEvaluateAndDropPt
  (
    double       dNormalizedParameter,  // in : 0 to 1 (start to end) target param value (accounts for orientation)                                   
    SmPoint2d  & rPointUV,              // out: UV value along Edgeuse for param (accounts for orientation)                                    
    SmVector2d * pOptFirstDerivUV=NULL  // out: Optional, 1st UV deriv along Edgeuse at param point (accounts for orientation), NULL to ignore 
  ) const ;                 

  SM_TYPE             GetEdgeuseType()           const { return (m_tEdgeuseType) ; }
  virtual SmBrep    * GetBrep()                  const ;
  SmShell           * GetShell()                 const ;
  SmVertexuse       * GetVertexuse()             const ;
  SmEdgeuse         * GetMate()                  const ;
  SmEdgeuse         * GetRadial()                const ;
  SmFace            * GetFace()                  const ;
  SmEdge            * GetEdge()                  const { return (SmEdge*)GetOwner() ; }
  double              GetEdgeParam(double dNormalizedParameter) const ; 
  SmFaceuse         * GetFaceuse()               const ;
  SmLoopuse         * GetLoopuse()               const ;
  SmBSplineCurve    * GetUVTrimCurvePointer()    const ;
  SmOrientType        GetOrientation()           const { return m_eOrientation ; }
  SmEdgeuse         * GetCWEdgeuse()             const { SM_ASSERT(m_pCW  == NULL || IsLoopEdgeuse()) ; return m_pCW ;  }
  SmEdgeuse         * GetCCWEdgeuse()            const { SM_ASSERT(m_pCCW == NULL || IsLoopEdgeuse()) ; return m_pCCW ; }
  double              GetStartPointUVLoopGap()   const ;
  double              GetEndPointUVLoopGap  ()   const ;
  SmEdgeuse         * GetCornerMateEdgeuse(const SmVertex *pVertex)  const;
  double              GetTurningAngDeg()         const ; // rtn AngDeg:[-180 to 180]

  SmStatus            GetConnectedEdgeuses
  (
    SmTArray<SmEdgeuse *> & rArray,      // out: list of connected edgeuses (not marked)                                                    
    SmMarkType             eMarkType     // in : only used for wires: passed to CollectWireEdges (not incremented - should be a NewMark)    
  ) const; 

  // Return continuity of dihedral sector based on 5 point samples
  SmContinuityType    GetSectorContinuity  (double dAngTolDeg=SM_CONTINUITY_ANGLE) const; // NotUsed: in :

  // Edge/Face   Gaps  
  // Edge/Vertex Gaps 
  // Edge/Edge   LoopGaps 
  SmEdgeFaceGap   * GetMaxEdgeFaceGap    (SmBoolean bForceCalc=FALSE) const ; // rtn: stored max gap between Edge->Curve and Face->Surface
  SmEdgeEdgeGap   * GetEdgeCCWEdgeGap    (SmBoolean bForceCalc=FALSE) const ; // rtn: stored gap between Edge->Curve and CCWEdge->Curve
  SmEdgeEdgeGap   * GetEdgeCWEdgeGap     (SmBoolean bForceCalc=FALSE) const ; // rtn: stored gap between Edge->Curve and CCWEdge->Curve, // NotUsed: in :
  SmVertexEdgeGap * GetVertexEdgeGap     (SmBoolean bForceCalc=FALSE) const ; // rtn: gap between edgeuse->edgeStartPoint/edgeuse->StartVertexPoint
  SmVertexEdgeGap * GetOtherVertexEdgeGap(SmBoolean bForceCalc=FALSE) const ; // rtn: gap between edgeuse->edgeEndPoint/edgeuse->EndVertexPoint

  SM_OLDTOL_LINE // local tolerance management
  SM_OLDTOL_LINE virtual SmZoneTol3d GetTolerance() const ;   // old Tol ZoneTol3d inherited from val stored on owning edge

  // Does Edgeuse have Gaps
  SmBoolean       HasEdgeFaceGap         () const { return IsLoopEdgeuse() ; }
  SmBoolean       HasEdgeCCWEdgeGap      () const { return IsLoopEdgeuse() ; }
  SmBoolean       HasEdgeFaceTrimCurveGap() const { return IsLoopEdgeuse() ; }

  // Is Cached Gap init                  
  SmBoolean       IsMaxEdgeFaceGapInit() const { return( m_sMaxEdgeFaceGap3d.IsInit() ) ; }
  SmBoolean       IsEdgeCCWEdgeGapInit() const { return( m_sEdgeCCWEdgeGap3d.IsInit() ) ; }
  void            InitMaxEdgeFaceGap()   const { m_sMaxEdgeFaceGap3d.ReSet() ; }
  void            InitEdgeCCWEdgeGap()   const { m_sEdgeCCWEdgeGap3d.ReSet() ; }

  // check Cached Gap value against Freshly Calced one
  SmBoolean       IsMaxEdgeFaceGapFresh(SmEdgeFaceGap & rCalcMaxEdgeFaceGap, SmEdgeFaceGap & rStoredMaxEdgeFaceGap) const ;
  SmBoolean       IsEdgeCCWEdgeGapFresh(SmEdgeEdgeGap & rCalcEdgeCCWEdgeGap, SmEdgeEdgeGap & rStoredEdgeCCWEdgeGap) const ;

  virtual SmBoolean IsConnectedTo        (const SmTopology *cpConnectTgt) const ;
  SmBoolean       IsEdgeuseSideInSurface
  (
    ULONG       lSmpCount=6,                // in : Number of sample points along the Edgeuse->Edge->Curve, default:[6]            
    SmBoolean * pOptPeriodicU=NULL,         // in : TRUE=Surface U Periodic, FALSE=Not, NULL to ignore, default:[NULL]             
    SmBoolean * pOptPeriodicV=NULL,         // in : TRUE=Surface U Periodic, FALSE=Not, NULL to ignore, default:[NULL]             
    SmBoolean * pOptOnNaturalBoundary=NULL  // in : TRUE=edgeuse is on SurfNatBndry, FALSE=Not, NULL to ignore, default:[NULL]     
  ) const ; 

  void            SetMaxEdgeFaceGap( const SmEdgeFaceGap & rMaxGap ) const 
      { m_sMaxEdgeFaceGap3d = rMaxGap; }

  // Recalc and cache Max Edge/Face gap (no longer cache Max Edge/FaceTrimCurve gap)
  SmStatus        RefreshGaps() const    
      { return( CalcMaxEdgeFaceGap(NULL, TRUE) ) ; }
                         
  // GetGap support methods for internal use - compute and optionally cache Edge Gaps (to Face, to CCWEdge, and to FaceTrimCurve)
  SmStatus CalcMaxEdgeFaceGap         (SmEdgeFaceGap          * pOptMaxEdgeFaceGap=NULL, SmBoolean bCache=TRUE) const ;
  SmStatus CalcEdgeCCWEdgeGap         (SmEdgeEdgeGap          * pOptEdgeCCWEdgeGap=NULL, SmBoolean bCache=TRUE) const ;

  SmStatus CalcMaxEdgeFaceTrimCurveGap
  (
    SmEdgeFaceTrimCurveGap & rMaxEdgeFaceTrimCurveGap,      // out: set to calculated Edge/FaceTrimCurve gap, NULL to ignore, default:[NULL]   
    SmBSplineCurve         * pOptUVTrimCurve = NULL         // in : for internal use only - stops infinite loop                                
  ) const ;

  // Returns the sector angle to the Radial EU at dParam, in radians
  SmStatus CalcSectorAngle ( 
    double    dParam,     // in :Edge parameter at which angle is calculated  
    double & rdAngle      // out:Sector angle at dParam in radians            
  );
    
  SmBoolean        IsLoopEdgeuse()  const;          // rtn: TRUE = edge connects to face           ; m_pSorLU points to Loopuse
  SmBoolean        IsShellEdgeuse() const;          // rtn: TRUE = edge connects directly to region; m_pSorLU points to Shell
  SmBoolean        IsStrut()        const;          // rtn: TRUE = edge does not connect at this end, but connects to loop at other end.

  SmBoolean        IsTangentSector                  // rtn: TRUE = dihedral angle between this and radial partner binormals is less tol
  (
    double    dTangencyTolDeg=SM_CONTINUITY_ANGLE,  // in : max deviation in degrees between two parallel vectors                     
    SmBoolean bCoincidenceTest=FALSE,               // in : TRUE = Test for coincidence, FALSE = Test for tangency                    
    SmBoolean bOKToMakeUVTrimCurve=TRUE             // in : TRUE=Make UVTrimCurve as needed, FALSE=use 3d only if UVTrimCurve==NULL   
  ) const ;    

  // rtn: TRUE = dihedral angle between this and radial partner binormals
  SmBoolean        IsConvexRadialSector                                  
  (
    ULONG     lNumSamples,                          // in : is convex along the entire length of the edge to TolDeg   
    SmBoolean bOptStepOff=FALSE,                    // in :                                                           
    double    dTolDeg = 0.0                         // in :                                                           
  ) const;           

  // rtn: TRUE=UVTrimCurve ControlPolygon turns and follows a surface singularity boundary 
  SmBoolean        HasDogLegNearPole (SmBoolean * pOptIndx=FALSE) const;  // out: CPt Index at frist turn, NULL to ignore, default:[NULL]

  void SetOrientation(SmOrientType eOrientation) 
      { m_eOrientation = eOrientation; }

  virtual void Notify
  (
    SmNotifyOperation eNotifyOperation,
    SmObject        * pData1,
    SmObject        * pData2,
    SmObject        * pData3
  ) ;

  // draw 80% of edge beginning at edgeuse start, 
  //   add a binormal vector pointing into its connected face, and
  //   add a surface normal vector showing this edgeuse's sector.
  SmDisplayList * Draw(double          dScale = 1.0,           // in : Set size of BiNorm and Norm Vectors
                                                               //      BiNormLength = .2 * CurveLength * dScale
                                                               //      NormLength   = .1 * CurveLength * dScale
                                                               //      default:[1.0]
                       SmBoolean       bDrawUVTrimCurves=TRUE, // in : TRUE = Draw UVTrimCurves when present, FALSE=don't
                                                               //        note: when UVTrimCurves are good - they draw on top of the edges and aren't seen
                                                               //              when UVTrimCurves are bad - they vary widely from the edges and indicate debugging is needed
                                                               //      default:[TRUE]
                       SmGfxArraySet * pOptGfxSet=NULL)        // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                      const ;                                  //      NULL to ignore. default:[NULL]
  SmDisplayList * DrawEdgeFaceTrimCurveGap(double dScale = 1.0, SmGfxArraySet * pOptGfxSet=NULL) const;
  SmDisplayList * DrawMicro(double * pOptMicroParam=NULL, SmGfxArraySet * pOptGfxSet=NULL) const ;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                         
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                     
                                              //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                 
    SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't  
    SmTArray<ULONG>  * pTestRequests=NULL     // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order               
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmEdgeuse,SmTopology,SmEdgeuse_TYPE);

} ; // end class SmEdgeuse

#endif // !__SMEDGEUSE_H__

