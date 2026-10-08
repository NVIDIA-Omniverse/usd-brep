// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFilletGeom.h 
* PURPOSE: Header file for SmFilletGeom objects.
**********************************************************************/

#ifndef __SMFILLETGEOM_H__
#define __SMFILLETGEOM_H__

#ifndef __SMOWNINGTOPOLOGY_H__
#include <SmOwningTopology.h>
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

#ifndef __SMEDGEUSE_H__
#include <SmEdgeuse.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifndef __SMVERTEXUSE_H__
#include <SmVertexuse.h>
#endif

#ifndef __SMSURFACEINTERSECTOR_H__
#include <SmSurfaceIntersector.h>
#endif

class SmFilletSolver;
class SmFilletCorner;
class SmFilletBrep;
class SmFilletGeom;

/*******************************************************************//**
PURPOSE: Fillet Error Type enum

NOTES: 
***********************************************************************/
enum SmFilletErrorType
{
  SM_FILERR_OK,
  SM_FILERR_EXEC_UNINITIALIZED,
  SM_FILERR_INVALID_INPUT,
  SM_FILERR_VERTEX_PROBLEM,
  SM_FILERR_EDGE_PROBLEM,
  SM_FILERR_BAD_INT,
  SM_FILERR_BAD_TRIM,
  SM_FILERR_BAD_MERGE
        // 6 - (Add more as needed)
        // 0 - Unknown
        // 1 - Rail/Rail Intersection
        // 2 - Rail Edgeuse Intersection
        // 3 - Surface Generation
        // 4 - Self Intersection
        // 5 - Unhandled Large Radius Case, (Add more as needed)
    // The remaining error codes should be a sequential display of events
    // showing where the errors started to occur.
} ;

/*******************************************************************//**
PURPOSE: Fillet Corner types

NOTES: 
***********************************************************************/
enum SmFilletCornerType 
{
  SM_FCR_UNKNOWN,
  SM_FCR_OPEN,
  SM_FCR_DEGENERATE,
  SM_FCR_1_x_1,
  SM_FCR_2_x_2,
  SM_FCR_3_x_2_MIXED,
  SM_FCR_4_x_3,
  SM_FCR_N_x_1_CLOSED,
  SM_FCR_N_x_1,
  SM_FCR_N_x_2,
  SM_FCR_N_x_N,
  SM_FCR_N_x_N_CONVEX,
  SM_FCR_N_x_N_CONCAVE
};

/*******************************************************************//**
PURPOSE: Filleting Status

NOTES: 
***********************************************************************/
enum SmFilStatus
{
  SM_FIL_UNPROCESSED,
  SM_FIL_PROCESSED,
  SM_FIL_FAILURE,
  SM_FIL_SURF_INT_FAILURE,
  SM_FE_END_PNT_DROP_FAILURE,
  SM_FE_TO_BE_SQUEEZED,
  SM_FE_CUBIC_RAIL_RAIL_INTERPOLATE,
  SM_FV_NO_INT_RAIL_EU,                    // 'Unable to intersect fillet rail curve with side Edge'
  SM_FV_SOLVER_NOT_CONVERGE,
  SM_FV_TO_BE_DELETED,
  SM_FS_SURF_TRACING_FAILURE,
  SM_FS_SELF_INT_HANDLING_FAILURE,
  SM_FS_SURF_SKINNING_FAILURE
};

/*******************************************************************//**
PURPOSE: FilletVertex type enum

NOTES: marks how to compute FilletVertex locations as used in 
  SmFilletVertex::CalcCornerVertGeom() to switch to proper vertex 
  position calculator
***********************************************************************/
enum SmFilletVertexType 
{
  SM_FV_FILLET_X_EDGEUSE,         //
  SM_FV_FILLET_X_FILLET,          // vertex is connected to another vertex on opposite railCurve - the edge between them
                                  //   will be determined by a fillet/fillet intersection
  SM_FV_FILLET_X2_FILLETS,        // vertex at intersection of 3 fillets at corner
  SM_FV_MATE,                     // vertex is mated to another vertex on opposite railCurve - the edge between them
                                  //   will be the same shape as the FilletSurface crossSection 
  SM_FV_ON_VERTEX,                // vertex on existing vertex
  SM_FV_ON_CROSS_SECTION,         // vertex is on the cross-sectional plane at the filleted vertex
  SM_FV_CLIFF_RAIL_X_EDGE,        //
  SM_FV_RAIL_END,                 // 
  SM_FV_RAIL_X_RAIL,              // vertex at intersection of two railcurves
  SM_FV_RAIL_X_EDGEUSE,           // vertex at intersection of a railcurve and an edge->Curve
  SM_FV_RAIL_X_VERTEX,            //
  SM_FV_RAIL_X_EXTENDED_EDGEUSE,  //
  SM_FV_SETBACK,                  //
  SM_FV_UNKNOWN
};

/*******************************************************************//**
PURPOSE: Describes the type of trimming to perform at the ends of 
         fillet surfaces.  Primarily used in Surface-Based Filleting.

NOTES: 
***********************************************************************/
enum SmBoundaryTrimmingType
{
  SM_BT_NONE,    // No fillet triming just generate entire fillet
  SM_BT_MINIMAL, // Trim using ISO curves to minimal intersections on face boundaries
  SM_BT_MAXIMAL, // Trim using ISO curves to maximal intersections on face boundaries
  SM_BT_BEVEL,   // Create a bevel (line in parameter space) between minimal and maximal
                 // intersections on each end of fillet.
  SM_BT_BLEND    // Create a blend (Hermite in parameter space) between minimal and 
                 // maximal intersections on each end of fillet.
};

/*******************************************************************//**
PURPOSE: A Fillet Vertexuse represents a vertexuse of a vertex 
         on a fillet. Typically, it contains the UV data of its 'use'

NOTES: 
***********************************************************************/
class SM_EXPORT SmFilletVertexuse : public SmVertexuse
{
protected:
  SmPoint2d   m_vUVPoint;     // The UV param on filletSurface
  SmTsectPnt  m_vTsectPnt;    // Computed result from fillet solver

public:
  // constructor
  SmFilletVertexuse();

  // destructor
  virtual ~SmFilletVertexuse() {}

  // simple access
  SmTsectPnt & GetTsectPnt()      { return m_vTsectPnt; }
  SmPoint2d    GetUVPoint() const { return m_vUVPoint; }

  // simple assignment
  void SetUVPoint(SmPoint2d &rUVPoint)           { m_vUVPoint = rUVPoint; }
  void SetTsectPnt(const SmTsectPnt & rTsectPnt) { m_vTsectPnt = rTsectPnt; }
  void SetProperty(SmTopology * pEU)             { m_tVertexuseType = SmEdgeuse_TYPE;
                                                   m_pSorLUorEU = pEU;
                                                 }


  // utilities
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                          //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
  SM_COMMON(SmFilletVertexuse,SmVertexuse,SmFilletVertexuse_TYPE);
    
} ; // end class SmFilletVertexuse

/*******************************************************************//**
PURPOSE: A Fillet Vertex represents a vertex on a fillet.

NOTES: 
***********************************************************************/
class SM_EXPORT SmFilletVertex : public SmVertex
{
protected:
  SmFilletVertexType     m_eType;        // Type of this vertex
  SmPointClassification  m_vPointClass;  // Map to original brep
  SmFilletVertex       * m_pMate[2];     // When specified, pointing to MATE(s) - matching vertices on other railCurves
  SmFilStatus            m_eStatus;      // Status of vertex computation
  SmFilletCorner       * m_cpCorner;     // If not NULL, will point to its belonging corner

public:
  // constructor
  SmFilletVertex(SmFilletCorner * cpCorner = NULL);

  // copy operator
  void CopyPointClass(SmPointClassification & rPC) { m_vPointClass = rPC; }

  // destructor
  virtual ~SmFilletVertex();

  // modifiers
  virtual SmStatus CalcCornerVertGeom        ();
  SmStatus         CalcFilletIntFillet       ();
  SmStatus         CalcFilletInt2Fillets     ();
  SmStatus         CalcRailIntEdgeuse        (SmFilletGeom * pOptFG = NULL);
  SmStatus         CalcRailIntExtendedEdgeuse();
  SmStatus         CalcRailIntRail           ();
  SmStatus         CalcSetBackVert           ();
  SmStatus         CalcVertOnVert            ();
  SmStatus         CalcVertOnCrossSection    (double * pOptEdgeParam = NULL);
  SmStatus         CalcMateGeom              ();
  
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmStatus         CalcCliffRailIntEdge      (SmBSplineSurface * pFillet,
  // rm :                                             double             dThisApproxTol3d);
  SmStatus         CalcCliffRailIntEdge      (SM_FILLETSURF_TYPE * pFillet,
                                              double             dThisApproxTol3d);

  SmStatus UpdateSplitFace( SmFace *pOrigFace, SmFace *pNewFace );

  // simple access
  virtual void                 GetName               (TCHAR * pcMyName, size_t lMyNameAllocLen);
  SmFilletVertex             * GetMate               (ULONG lIndex)        const { return m_pMate[lIndex]; }
  SmFilStatus                  GetStatus             ()                    const { return m_eStatus; }

  SmPoint3d                    GetPoint              ()                    const { return m_vPoint; }
  SmPointClassification      & GetPointClassification()                          { return m_vPointClass; }
  double                       GetOriginalTParam     ()                    const { return m_vPointClass.GetTParam(); }
  SmPoint2d                    GetOriginalUV         ()                    const { return m_vPointClass.GetUVParam(); }
  SmPointClassificationType    GetPointClass         ()                    const { return m_vPointClass.GetPointClass(); }
  SmObject                   * GetPointClassObject   ()                    const ;

  SmFilletCorner             * GetFilletCorner       ()                    const { return m_cpCorner; }
  void                         GetFilletGeoms        (SmTArray<SmFilletGeom*> & rGeoms);
  virtual SmFilletVertexType   GetFilletVertexType   ()                    const { return m_eType; }
  SmFilletVertexuse          * GetVUAtRailEnd        (SmFilletGeom      * pOptFilletGeom = NULL,
                                                      SmFilletVertexuse * pOptExcludeVU  = NULL,
                                                      SmFilletEdge      * pOptFilletRail = NULL);
  ULONG                        FindIndexInCorner     () const;

  // predicates                                                      
  SmBoolean IsProcessed() const { return m_eStatus == SM_FIL_PROCESSED; }

  // simple assignment
  void SetMate            (ULONG lIndex, 
                           SmFilletVertex * pMate)               { m_pMate[lIndex] = pMate; }
  void SetStatus          (SmFilStatus eStatus)                  { m_eStatus = eStatus; }
  void SetFilletCorner    (SmFilletCorner * pCorner)             { m_cpCorner = pCorner; }
  void SetFilletVertexType(SmFilletVertexType eType)             { m_eType = eType; }
  void SetOriginalTParam  (double dT)                            { m_vPointClass.SetTParam(dT); }
  void SetOriginalUV      (const SmPoint2d & crUVParam)          { m_vPointClass.SetUVParam(crUVParam); }
  void SetPointClass      (SmPointClassificationType eType, 
                           SmObject * pObj)                      { m_vPointClass.SetClassObject(eType, pObj); }

  // utilities
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                          //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  SM_COMMON(SmFilletVertex,SmVertex,SmFilletVertex_TYPE);
  void DumpLevel(int           iDebugLevel,           // NotUsed: in : 
                 const TCHAR * pcMsg = NULL ) const;  // in : 

#ifdef SM_DEBUG_CODE
  int DebugLevel();
#endif // SM_DEBUG_CODE
    
} ; // end class SmFilletVertex

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletVertex*) ;


/*******************************************************************//**
PURPOSE: FilletEdge type enum

NOTES: 
***********************************************************************/
enum SmFilletEdgeType 
{
  SM_FE_BLENDING_RAIL,
  SM_FE_CROSS_SECTION,             // FilletEdge shape is same as filletSurface crossSection
  SM_FE_CLIFF_RAIL,
  SM_FE_FILLET_END,
  SM_FE_FILLET_X_FILLET,
  SM_FE_FILLET_X_SIDE_FACE,
  SM_FE_ON_EDGE,                   // filletEdge is defined by an Original Brep SideEdge
  SM_FE_ON_EXTEND_EDGE,
  SM_FE_PRECOMPUTED,               // Edge will be computed during creation, NOT a general type
  SM_FE_RAIL,
  SM_FE_RAIL_EXTENSION,
  SM_FE_RAIL_RAIL_INTERPOLATION,
  SM_FE_SETBACK_RAIL,
  SM_FE_SPLIT_FACE,
  SM_FE_UNKNOWN
} ;

/*******************************************************************//**
PURPOSE: A Fillet Edgeuse represents an edgeuse of a fillet edge.

NOTES: 
***********************************************************************/
class SM_EXPORT SmFilletEdgeuse : public SmEdgeuse
{
protected:
  SmFilletGeom * m_pFilletGeom;       // owner of this FilletEdgeuse 

public:
  // constructor
  SmFilletEdgeuse(SmFilletBrep *pPseudoBrep);

  // destructor
  virtual ~SmFilletEdgeuse() {}

  // simple access
  SmFilletGeom * GetFilletGeom     () const                          { return m_pFilletGeom; }

  // simple assignment
  void           SetFilletGeom     (SmFilletGeom * pFilletGeom) ;
  void           SetFilletVertexuse(SmFilletVertexuse * pFilVU)      { m_pVU = pFilVU; }
  void           SetUVCurve        (SmBSplineCurve * pUVTrimCurve,            // in : NewUVTrimCurve value
                                    SmBoolean        bDeleteOldCurve=FALSE,   // in : TRUE = delete current m_pCurve if not NULL
                                                                              //      FALSE= don't delete current m_pCurve
                                                                              //      default:[FALSE] = previous behavior
                                    SmBoolean        bDbgWarnLeaks=TRUE) ;    // in : TRUE = in Debug Mode - Warn when NonNULL m_pCurve is overwritten
                                                                              //      FALSE= don't, caller knows that might happen and it's not a problem
                                                                              //      default:[TRUE]

  // utilities
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                          //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

#ifdef SM_DEBUG_CODE
  int DebugLevel();
#endif // SM_DEBUG_CODE

  SM_COMMON(SmFilletEdgeuse,SmEdgeuse,SmFilletEdgeuse_TYPE);
    
} ; // end class SmFilletEdgeuse

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletEdgeuse*) ;

/*******************************************************************//**
PURPOSE: A Fillet Edge represents an edge of a fillet.

NOTES: gwc: During construction a SmFilletEdge has two Edgeuses 
         SmFilletEdge->PrimaryEU            = connects FilletEdge to New Faces built for the Fillet
         SmFilletEdge->PrimaryEU->GetMate() = connects FilletEdge to OrigFaces touching the new Fillets
***********************************************************************/
class SM_EXPORT SmFilletEdge : public SmEdge
{
protected:
  SmFilletEdgeType        m_eType;             // Fillet Edge type
  double                  m_dDeviation = 0.0;  // Deviation of curve
  SmEdge                * m_pOrigEdge = NULL;  // Original m_pTargetBrep->Edge this curve is on
  SmFace                * m_pOrigFace = NULL;  // Original m_pTargetBrep->Face this curve is attaching to, 
                                               //    which might have to be extended to make fillet EndSurfaces
                                               //    and which might be changed if the Face is split.
  SmTArray< SmFace *>     m_apSplitFaces;      // When pOrigFace is split by this FilletEdge, these are the new Faces
  SmCurveClassification * m_pCurveClass;       // Contains classification of the rail
  SmFilletCorner        * m_cpCorner;          // FilletCorner that constructed this SmFilletEdge. 
                                               //    NULL = Not from a FilletCorner
  SmFilStatus             m_eStatus;           // Status of edge computation

public:
  SmEdge                * m_pFilletBrepEdge1;  // An m_pFilletBrep->Edge that matches this Edge geometrically.
                                               // Only used when 'this' FilletEdge->GetBrep() == m_pPseudoBrep 
                                               //     (e.g. was made by a FilletCorner or FilletSolver in m_pPseudoBrep)
                                               // The m_pFilletBrep->Edge is to be merged into the
                                               // original m_pTargetBrep. Typically a lamina edge always in m_pFilletBrep.
  SmEdge                * m_pFilletBrepEdge2;  // One other m_pFilletBrep->Edge which might associate
                                               // with 'this' if splitting of 'this' occurred.
                            
public:
  // constructor                     
  SmFilletEdge(SmFilletBrep *pPseudoBrep, SmFilletCorner * cpCorner = NULL);

  // destructor
  virtual ~SmFilletEdge();

  // modifiers
  SmStatus         CalcBlendingRail         (const SmContext & crContext);
  virtual SmStatus CalcCornerEdgeGeom       ();
  SmStatus         AdjustCrossSection       (const SmContext & crContext, SmFilletCorner * pCorner, SmVertex * pFVOfDegenerateCurve);
  SmStatus         CalcCrossSection         (const SmContext & crContext, SmFilletCorner * pOptCorner = NULL);
  SmStatus         CalcEdgeOnEdge           (const SmContext & crContext, SmFilletCorner * pOptCorner = NULL);
  SmStatus         CalcEdgeOnExtendedEdge   (const SmContext & crContext, SmFilletCorner * pOptCorner = NULL);
  SmStatus         CalcFilletEnd            (const SmContext & crContext, SmFilletCorner * pOptCorner = NULL);
  SmStatus         CalcFilletIntFillet      (const SmContext & crContext, SmFilletCorner * pOptCorner = NULL);   // NotUsed: in :
  SmStatus         CalcFilletIntSideFace    (const SmContext & crContext, SmFilletCorner * pOptCorner = NULL);
  SmStatus         CalcRailRailInterpolation(const SmContext & crContext, SmFilletCorner * pOptCorner = NULL);
  SmStatus         CalcSetBackRail          (const SmContext & crContext, SmFilletCorner * pOptCorner = NULL);
  SmStatus         ClearFilletBrepEdges     (const SmTArray<SmEdge*> & crEdges );
  SmStatus         FindFilletBrepEdge       (const SmTArray<SmEdge*> & crEdges, SmBoolean bFindMultipleEdges = FALSE);
  static SmStatus  MakeFilletEdge           (SmFilletBrep   * pPseudoBrep,       // in : target Brep to receive new topology objects
                                             SmFilletVertex * pStartVertex,      // in : start of new FilletEdge
                                             SmFilletVertex * pEndVertex,        // in : end   of new FilletEdge
                                             SmFilletEdge  *& rpNewFilletEdge,   // out: newly allocated FilletEdge - no Curve or UVTrimCurve data
                                             SmFilletCorner * cpCorner = NULL);  // in : NewFilletEdge's filletCorner, gets stored in rpNewEdge->m_cpCorner
                                                                                 //      NULL to ignore, default:[NULL]
  SmBoolean        ShouldSwapFaces( SmFace *pOrigFace, SmFace *pNewFace );
  SmStatus         UpdateSplitFace( SmFace *pOrigFace, SmFace *pNewFace );

  // simple access
  SmCurveClassification * GetCurveClass     ()                           const { return m_pCurveClass; }
  SmEdge                * GetFilletBrepEdge ()                           const { return m_pFilletBrepEdge1; }
  SmStatus                GetFilletBrepEdges(SmTArray<SmEdge*> & rEdges) const;
  SmFilletCorner        * GetFilletCorner   ()                           const { return m_cpCorner; }
  SmFilletEdgeType        GetFilletEdgeType ()                           const { return m_eType; }
  virtual void            GetName           (TCHAR * pcMyName, size_t lMyNameAllocLen) ;
  SmEdge                * GetOriginalEdge   ()                           const { return m_pOrigEdge; }
  SmFace                * GetOriginalFace   ()                           const { return m_pOrigFace; }
  const SmTArray< SmFace *> & GetSplitFaces ()                           const { return m_apSplitFaces; }
  SmBrep                * GetOriginalBrep   ()                           const { return m_pOrigEdge ? m_pOrigEdge->GetBrep() : m_pOrigFace ? m_pOrigFace->GetBrep() : NULL ; }
  SmBSplineCurve        * GetOriginalUVCurve()                           const { return GetPrimaryEdgeuse()->GetMate()->GetUVTrimCurvePointer(); }
  SmFilStatus             GetStatus         ()                           const { return m_eStatus; }
  ULONG                   FindIndexInCorner ()                           const ;

  // simple assignment
  void SetCurveClass    (SmCurveClassification * pCurveClass) { m_pCurveClass = pCurveClass; }
  void SetFilletEdgeType(SmFilletEdgeType eType)              { m_eType       = eType; }
  void SetFilletCorner  (SmFilletCorner * pCorner)            { m_cpCorner    = pCorner; }
  void SetOriginalEdge  (SmEdge * pEdge)                      { m_pOrigEdge   = pEdge; }
  void SetOriginalFace  (SmFace * pFace)                      { m_pOrigFace   = pFace; }
  void AddSplitFace     (SmFace * pFace)                      { m_apSplitFaces.AddUnique( pFace ); }
  void SetStatus        (SmFilStatus eStatus)                 { m_eStatus     = eStatus; }

  // utilities
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                          //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
  SM_COMMON(SmFilletEdge,SmEdge,SmFilletEdge_TYPE);
  void DumpLevel( int iDebugLevel, const TCHAR * pcMsg = NULL );
  SmDisplayList * Draw(SmGfxArraySet * pOptGfxSet=NULL) const;

#ifdef SM_DEBUG_CODE
  int DebugLevel();
#endif // SM_DEBUG_CODE

} ; // end class SmFilletEdge

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletEdge*) ;

/*******************************************************************//**
PURPOSE: FilletGeom type enum

NOTES: 
***********************************************************************/
enum SmFilletGeomType 
{
  SM_FG_DEFAULT,
  SM_FG_BLENDS,
  SM_FG_CLIFF_ROLLOVER,
  SM_FG_TANGENT_ROLLOVER,
  SM_FG_CLIFF_SIDE_PATCH,
  SM_FG_GAP_FILLER,
  SM_FG_CORNER_FILLET
};

/*******************************************************************//**
PURPOSE: This class contains the geometry for a single fillet surface.
    Each FilletGeom contains two rail curves and the center line
    (if defined, such as in 'Rolling-ball' models)

NOTES: 
***********************************************************************/
class SM_EXPORT SmFilletGeom
{
  friend class SmFilletSolver;
  friend class SmConstantRadiusFS;
  friend class SmFilletSurfaceGenerator;
  friend class SmFilletIntersector;
  friend class SmFilletCorner;
  friend class SmFilletExecutive;
  friend class SmSurfaceSurfaceFS;
protected:
  SmFilletGeomType           m_eType;            // Fillet Geom type
  SmFilletSolver           * m_pFilletSolver;    // Fillet Solver defining implicit geometry
                                                 // contains context for all new Geometry construction
  SmFilletEdge             * m_vRails[2];        // Corresponding 3D Rails (commonly built as SM_SP_V IsoParameter curves of m_pFilletSurface)
  SmTArray<SmFilletEdge*>    m_vOtherRails1;     // Array of rail edges appended to m_vRails[0]
                                                 // Mostly, it will be used when original rail is split into multiple segments
                                                 // when rollover occurred.
  SmTArray<SmFilletEdge*>    m_vOtherRails2;     // Array of rail edges appended to m_vRails[1]

  SmBSplineCurve           * m_pCenterLineCurve; // Curve corresponding to a center line
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface         * m_pFilletSurface;   // Generated fillet surface
  SM_FILLETSURF_TYPE       * m_pFilletSurface;   // Generated fillet surface
                                                 // For right now we assume that the start of the fillet surface corresponds
                                                 // to the minimum U value of its domain and the end of the surface corresponds
                                                 // to the maximum U value.  Rail 1 (on m_pSurface[0]) corresponds the minimum 
                                                 // V value and Rail 2 (on m_pSurface[1]) corresponds to the maximum V value.  
  SmOffsetSurface          * m_pOffSurfs[2];     // These may differ from our FilletSolver's.
  double                     m_dOrientations[2]; // Offset directions: +- 1.0.
  double                     m_dDeviations[2];   // Deviation of 3D Rail Curve from surface.
  SmBoolean                  m_bCapped[2];       // Has this end of the surface required capping -
                                                 // i.e. extension to the domain boundary.
  SmBoolean                  m_bNeedSplit;       // Does the fillet surface need to be split
  SmFilStatus                m_eStatus;          // Status of fillet computation
  SmTArray<SmFilletEdge*>    m_vEdges;           // Array of edges 'this' owned (other than rails)
  SmTArray<SmFilletVertex*>  m_vVertices;        // Array of vertices 'this' owned
  SmTArray<SmFilletEdgeuse*> m_vRefEUs;          // Collection of all referenced edgeuses,
  SmTArray<SmFilletEdgeuse*> m_vSideEUs;         // Collection of all side curves, excluding rail curves.

public:
  // constructor
  SmFilletGeom(SmFilletSolver *pFilletSolver,
               SmFilletGeomType eType = SM_FG_DEFAULT);

  // copy constructor
  SmFilletGeom(SmFilletGeom *pFilletGeomToCopy);

  // copy operator
  SmStatus Copy(SmFilletGeom *pFilletGeomToCopy);

  // destructor
  virtual ~SmFilletGeom();

  // modifiers
  void     AddReferencedFilletEdgeuse(SmFilletEdgeuse * pRefEdgeuse)  { m_vRefEUs.AddUnique(pRefEdgeuse); }
  void     AddSideFilletEdgeuse      (SmFilletEdgeuse * pSideEdgeuse) ;
  SmStatus CalcBlendingGeom          ();
  SmStatus CreateEndCurve            (ULONG lEndIndex,
                                      SmBoundaryTrimmingType eTrimType,
                                      SmFilletBrep * pPseudoBrep);
  SmStatus InsertIntersectionTopology();

  // Create face in pBrep from m_pFilletSurface with call m_pFilletBrep->MakeFaceWithCurves
  SmStatus MakeFaceBrep              (SmBrep * pBrep);

  SmStatus MakeGapFiller             ();
  SmStatus MakeRailEdge              (SmFilletBrep  * pPseudoBrep,
                                      ULONG           lRailIndex,
                                      SmFilletEdge *& rpNewEdge);
  SmStatus RefineFilletPoint         (ULONG lRailIndex,   // NotUsed: in :
                                      ULONG lEndIndex);   // NotUsed: in :
  SmStatus ReCalcFilletGeom          (SmBoolean bIsAnalyticFillet = FALSE,
                                      SmVector3d * pOptMarchDir = NULL,
                                      SmCurve * pOptCurve = NULL);
  SmStatus RailEdgeuseIntersect      (ULONG        lRailIndex,          // in : target rail index
                                      SmEdgeuse  * pEdgeuse,            // in : target edgeuse->edge->curve to intersect
                                      double       dGuessEdgeParameter, // in : Edgeuse->curve guess parameter
                                      SmBoolean  & rbFoundIntersection, // out: TRUE=found an intersection
                                      SmTsectPnt & rTsectPnt,           // out: Contains solution
                                      double     & rdEdgeuseParameter); // out: edgeuse Parameter of intersection
                                     
  // make topology to connect FilletVertex to RailEdge as Vertex->Vertexuse->Edgeuse->Edge
  SmStatus MakeRailEdgeuse(SmFilletBrep      * pPseudoBrep,
                           ULONG               lRailIndex,
                           SmFilletVertex    * pFilVertex,
                           SmFilletEdgeuse  *& rpNewEdgeuse,
                           SmOrientType        eOrient = SM_OT_UNKNOWN,
                           SmFilletEdge      * pOptBelongingRail = NULL);

  SmStatus UpdateSplitFace( SmFace *pOrigFace, SmFace *pNewFace );

  // split topology of a single parent filletGeom into two connected FilletGeom children
  SmStatus TopologySplit(SmFilletGeom    *& rpNewFilletGeom,
                         SmFilletGeomType   eNewGeomType = SM_FG_DEFAULT);
  SmStatus Trim          ( SmExtent1d &crTrimInterval );
  SmStatus TrimCliffRails();
  SmStatus TrimRailCurves();

  // Trim the original surfaces in cases where the fillet edge has split the face.
  SmStatus TrimOriginalSurfaces(SmTArray<SmFace*> & rFilletBrepFacesKept,    // out: list of faces in m_pFilletBrep to keep 
                                SmTArray<SmFace*> & rOriginalFacesDelete,    // out: list of faces from OrigBrep(s) to delete
                                SmTArray<SmNewMarkAndLock *> & rMarkLocks) ; // in : list of all marks for all contexts used in this fillet
                                                                             //      the first entry is for SmFilletExecutive::m_crContext
                                                                             // note: Checks and sets topology mark values, does not increment context mark values
  // simple access
  SmFilletGeomType    GetFilletGeomType()  const      { return m_eType; }
  SmFilletSolver    * GetFilletSolver  ()  const      { return m_pFilletSolver; }
  SmFilStatus         GetStatus        ()  const      { return m_eStatus; }
  SmBoolean           GetSplitFlag     ()  const      { return m_bNeedSplit; }

  void                GetFilletEdges    (SmTArray<SmFilletEdge*>   & rEdges) const { rEdges.Append(m_vEdges) ; }
  void                GetFilletVerts    (SmTArray<SmFilletVertex*> & rVerts) const { rVerts.Append(m_vVertices) ; }

  SmBSplineCurve    * GetCenterLineCurve()                                   const { return m_pCenterLineCurve; }
                     
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : SmBSplineSurface  * GetFilletSurface  ()                                   const { return m_pFilletSurface; }
  SM_FILLETSURF_TYPE * GetFilletSurface  ()                                   const { return m_pFilletSurface; }

  SmFilletEdge      * GetRail           (ULONG lRailIndex)                   const { return m_vRails[lRailIndex] ; }
  SmFilletVertex    * GetRailVertex     (ULONG lRailIndex, ULONG lEndIndex)  const ;
  SmFilletVertexuse * GetRailVertexuse  (ULONG lRailIndex, ULONG lEndIndex)  const ;
  SmOffsetSurface   * GetOffsetSurface  (ULONG lRailIndex )                  const ;

  SmStatus            GetRailIndexOfSideEU   (SmEdgeuse         *pEU, ULONG &lIdx ) const ;
  SmStatus            GetRailIndexOfVertexuse(SmFilletVertexuse *pVU, ULONG &lIdx ) const ;
  SmStatus            GetRailIndexOfVertex   (SmFilletVertex    *pVU, ULONG &lIdx ) const ;
  SmStatus            FindEdgeTangentAtPoint (ULONG lRailIndex,
                                              ULONG lEndIndex,
                                              SmBoolean & rbFoundTangent,
                                              SmVector3d & rTangent) const;
                      
  // simple assignment
  void     SetCenterLineCurve(SmBSplineCurve   * pCenterLine) { m_pCenterLineCurve = pCenterLine; }

  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
  // rm : void     SetFilletSurface  (SmBSplineSurface * pSurface)    { m_pFilletSurface   = pSurface; }
  void     SetFilletSurface  (SM_FILLETSURF_TYPE * pSurface){ m_pFilletSurface   = pSurface; }

  void     SetFilletGeomType (SmFilletGeomType eType)       { m_eType            = eType; }
  SmStatus SetOffsetSurface  (ULONG             lRailIndex, 
                              SmOffsetSurface * pOffSurf ) ;
  void     SetRail           (ULONG          lIndex, 
                              SmFilletEdge * pRailE)        { m_vRails[lIndex]   = pRailE; }
  void     SetSplitFlag      (SmBoolean bNeedSplit)         { m_bNeedSplit       = bNeedSplit; }
  void     SetStatus         (SmFilStatus eStatus)          { m_eStatus          = eStatus; }
  SmStatus SetUpRailPoint    (ULONG                   lRailIndex,
                              ULONG                   lEndIndex,
                              double                  dParameter,
                              SmPointClassification * pOptPointClass = NULL);

  //utilities
  void RecordFilletError(SmFilletErrorType  eType,
                         SmEdge           * pFilletedEdge,
                         SmFace           * pTargetFace,
                         SmFilletEdge     * pRail,
                         SmSurface        * pFilletSurface,
                         const TCHAR      * errMsg = NULL) ;

  SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                        SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                  //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                  //      default:[SM_LEVEL_0] 
                        SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                        SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                       const                                      { SM_REF4(pAList, eTestLevel, eWalkTree, pTestRequests) ;return TRUE ; }
  // obsolete
  // SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) { SM_REF2(rAReport, pAList) ;return TRUE ; }

  void DumpLevel( int iDebugLevel, const TCHAR * pcMsg = NULL ) const ;
  void Draw() const ;

#ifdef SM_DEBUG_CODE
  int DebugLevel();
  void DumpFilletVUs();
#endif // SM_DEBUG_CODE

  ULONG FindIndexInSolver() const ;

  SM_COMMON_BASE(SmFilletGeom, SmFilletGeom_TYPE) ;
  // const TCHAR * GetTypeString() const { return _T("SmFilletGeom") ; }

} ;  // end class SmFilletGeom 
                             
// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletGeom*) ;

/*******************************************************************//**
PURPOSE: This is the pseudo brep which contains the fillet edges
    and fillet vertices.

NOTES: 
***********************************************************************/
class SM_EXPORT SmFilletBrep : public SmBrep
{
  // inherited:
  // SmTopology::m_pListOwner            -
  // SmTopology::m_pNext                 -
  // SmTopology::m_pLast                 -
  // SmOwningTopology::m_pList           -
  // SmOwningTopology::m_lListSize       -
  // SmSAGObject::m_lDisplayListName     -
  // SmSAGObject::m_vDisplayParameters   -

public:
  // constructor
  SmFilletBrep();

  // destructor
  virtual ~SmFilletBrep();

  // modifiers
  void AddEdge   (SmFilletEdge *pFilletEdge) { m_pEdgeListHead->PostInsert(pFilletEdge); }
  void RemoveEdge(SmFilletEdge *pFilletEdge) { m_pEdgeListHead->Remove(pFilletEdge); }

  // utilities
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                          //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // no drawing for FilletBrep 
  virtual SmDisplayList * Draw(SmBoolean bAddToUIPickList=FALSE, SmGfxArraySet * pOptGfxSet=NULL) const { SM_REF2(bAddToUIPickList, pOptGfxSet) ; return NULL ; }
    
} ; // end class SmFilletBrep

//
SmStatus sm_SrfSrfIntersection
  (const SmContext & crContext,                            // in : context for new object construction
   SmVertex        * pStartV,                              // in : 1st point known to be on intersection curve
   SmVertex        * pEndV,                                // in : 2nd point known to be on intersection curve,
                                                           //      NULL to ignore
   SmVector3d      * pStartDirection,                      // in : expected general direction of intersection curve from 1st point
   SmVector3d      * pEndDirection,                        // in   expected intersection end direction
   SmSurface       * pSurf1,                               // in : 1st intersecting surface
   SmSurface       * pSurf2,                               // in : 2nd intersecting surface
   SmApproxTol3d     dApproxTol,                           // in : max allowed distance between xSect Curve and surfaces
   double            dAngleTol,                            // in : max allowed angle between consecutive xSect curve segment tangents
   SmBSplineCurve *& rp3DCurve,                            // out: 3d intersection curve
   SmBSplineCurve *& rpUVCurve1,                           // out: associated UVTrimCurve on 1st surface
   SmBSplineCurve *& rpUVCurve2,                           // out: associated UVTrimCurve on 2nd surface
   SmBoolean         bOptSkipTwoPntsIntersection = FALSE,  // in : FALSE= try cheap sm_TwoPntsIntersection() before
                                                           //             resorting to expensive general surf/surf xSect solver
   SmPoint3d       * pOptRefPoint = NULL,                  // in : reference point
   int               iDebugLevel = 0);                     // in : iDebugLevel, 0 = No Debug output

#endif // !__SMFILLETGEOM_H__

