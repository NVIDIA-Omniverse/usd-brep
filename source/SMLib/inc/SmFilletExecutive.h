// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFilletExecutive.h
* PURPOSE: Header file for SmFilletExecutive object.
**********************************************************************/

#ifndef __SMFILLETEXECUTIVE_H__
#define __SMFILLETEXECUTIVE_H__

#ifndef __SMFILLETSOLVER_H__
#include <SmFilletSolver.h>
#endif

#ifndef __SMFILLETCORNER_H__
#include <SmFilletCorner.h>
#endif

#ifndef __SMMAPPTRTOPTR_H__
#include <SmMapPtrToPtr.h>
#endif

#ifndef __SMMERGE_H__
#include <SmMerge.h>
#endif

#ifndef __SM_TRACKTOPOLOGYCHANGES_H__
#include <SmTrackTopologyChanges.h>
#endif

// GWC:BIND_TEMPLATE_MOVE   SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletErrorType) ;
// due to UNIX problems...leave it here
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletErrorType) ;

class SM_EXPORT SmFilletErrorInfo
{
private:
    SmFilletErrorType m_eErrorCode;
    SmFilStatus       m_eFilletStatus;

    // Geometry and topology involved in the error:
public:   //cbi: maybe make private, but maybe not, because they don't have to 
          //cbi  be kept in sync like the arrays.

    SmFilletExecutive * m_pFilExec;

    SmPoint3d m_vLocation;  // point where the error occurred

    // These are in the Target Brep:
    const SmVertex * m_pFilletedVertex;
    const SmEdge   * m_pFilletedEdge;
    const SmFace   * m_pSideFace;
    const SmFace   * m_pEndFace;

    SmFilletCornerType m_eCornerType;
    SmFilletEdgeType   m_eEdgeType;

    // These are in the Fillet Brep:
    const SmFilletVertex           * m_pFilletVertex1;
    const SmFilletVertex           * m_pFilletVertex2;
    SmTArray<const SmSurface*>       m_pFilletSurfaces;
    SmTArray<const SmFilletEdge*>    m_pFilletEdges;
    SmTArray<const SmBSplineCurve *> m_pFilletCurves;  // could be spine of fillet.

    TCHAR m_cComment[SM_TBLOCK_SIZE];
    TCHAR m_cEntityName[SM_TBLOCK_SIZE];

    public:

    // Constructors
    SmFilletErrorInfo();

    SmFilletErrorInfo( SmFilletExecutive *pFilletExec );

    // Destructor
    ~SmFilletErrorInfo() {}

    // Assignment operator
    SmFilletErrorInfo& operator=( const SmFilletErrorInfo & src );

private:
    void set();
    
public:
    // Using the class:

    SmFilletErrorType GetErrorCode()             { return m_eErrorCode;  }
    void SetErrorCode( SmFilletErrorType lCode ) { m_eErrorCode = lCode; }

    // A generic message:
    void NoteError(
        SmFilletErrorType eErrorCode,
        const TCHAR * cComment = NULL
    );

    //cbi: for problem merging fillet edge into target face:
    void NoteError
    (
        SmFilletErrorType eErrorCode,
        SmFilStatus eFilletStatus,
        const SmEdge *pFilletedEdge,
        const SmFace *pTargetFace,
        const SmFilletEdge *pFilletEdge,
        const SmSurface *pFilletSurface,
        const TCHAR * cComment = NULL
    );

    //cbi: for problem intersecting two fillet surfaces:
    void NoteError
    (
        SmFilletErrorType  eErrorType,
        SmFilletCornerType eCornerType,
        SmFilStatus eFilletStat,
        TCHAR     * sCornerName,
        const SmVertex  * pFilletedVertex,
        const SmSurface * pFilletSurface1,
        const SmSurface * pFilletSurface2,
        const SmFilletVertex * pStartFV,
        const SmFilletVertex * pEndFV,
        const TCHAR * cComment = NULL
    );

    void NoteError
    (
        const SmVertex *pBadVert,     // in :
        SmFilStatus eVertexStatus,    // NotUsed: in :
        SmFilletErrorType eErrorCode, // NotUsed: in :
        const TCHAR * cComment = NULL // NotUsed: in :
    );

    void NoteError
    (
        const SmEdge *pBadEdge,
        SmFilStatus eVertexStatus,
        SmFilletErrorType eErrorCode,
        const TCHAR * cComment = NULL
    );

    void NoteError
    (
        SmFilletErrorType eErrorType,
        SmFilStatus eFilletStat,
        SmFilletEdgeType eFEType,
        const SmFilletEdge *pFilletEdge,
        const TCHAR *cEdgeName,
        const TCHAR *cComment
    );

    // For problem intersecting base surfaces:
    void NoteError
    (
        SmFilletErrorType eErrorType,
        SmFilStatus eFilletStat,
        const SmEdge *pFilletedEdge,
        // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
        // rm : SmTArray<SmBSplineSurface*> & rapFilSurfs,
        SmTArray<SM_FILLETSURF_TYPE*>        & rapFilSurfs,
        SmTArray<SmBSplineCurve*> & rapLeftRails,
        SmTArray<SmFilletEdge*> & rapRightRails,
        SmPoint3d &rTsectPos,
        const TCHAR * cComment = NULL
    );

    void Report();

}; // end class SmFilletErrorInfo

/*******************************************************************//**
PURPOSE: Fillet Executive that is the top level controlling object.

NOTES:
***********************************************************************/
class SM_EXPORT SmFilletExecutive : public SmMerge
{
  friend class SmFilletGeom;
  friend class SmFilletSolver;
  friend class SmFilletCorner;
protected:
  const SmContext &         m_crContext;            // Context for all temporary and permanent object allocation.

  SmBrep *                  m_pTargetBrep;          // Brep to be filleted (can be NULL). Set in SmFilletExecutive constructor.
                                                    //   In DoFilleting() becomes m_vTI::m_pBrep for DoPiecewiseMerge() and IntersectInsertRelate() calls.

  SmBrep *                  m_pFilletBrep;          // Intermediate Brep made in CreateFilletBrep() made in pieces by calls to:
                                                    //     SmFilletCorner::CalcCornerGeom()  - For every FilletCorner: Any faces constructed are placed in m_pFilletBrep
                                                    //     SmFilletGeom::MakeFaceBrep()      - for every FilletSolver: use SmFilletGeom::MakeFaceBrep to place FilletSurface Face in m_pFilletBrep
                                                    //            (context == m_crContext)
                                                    //   It's initially a set of unconnected corner and edge fillet complexes (one for eace FilletCorner with a face and each FilletSolver->FilletGeom; 
                                                    //      many coincident edges and vertices) ultimately stitched together with one SmBrep::StitchFaces() call.
                                                    //   It's a stand-alone model of the Fillet shape to insert into TargetBrep. 
                                                    //   It's the Union of all FilletCorner and FilletSolver Faces where each Face
                                                    //      is bound by FilletEdges and FilletVertices cached in the m_pPseudoBrep 
                                                    //      by the FilletCorners and FilletSolvers.
                                                    //   In DoFilleting() becomes m_vTI::m_pOther for DoPiecewiseMerge() and IntersectInsertRelate() calls.

  SmFilletBrep *            m_pPseudoBrep;          // Intermediate Brep to hold all SmFilletEdge and SmFilletVertex topology created by
                                                    //    SmFilletEdge::MakeFilletEdge() from SmFilletCorner::MakeVariableRadiusFillet()
                                                    //                                        SmFilletCorner::MakConstantRadiusFillet()
                                                    //    SmFilletCorner::MakeCornerToplogy()
                                                    //    SmFilletCorner::AdjustTopology()
                                                    // The PseudoBrep often times has topology that has yet to be assigned geometry.
                                                    // The m_pPseudoBrep is a bag of unconnected complete corner and edge fillet complexes of
                                                    // SmFilletEdges and SmFilletVertices needing to be connected to one another.  
                                                    // Copies of these model bits are trimmed, extended, and connected in m_pFilletBrep
                                                    // to create one consistent sheet model of the entire Fillet geometry.
                                                    //            (context == m_crContext)

  SmTArray<SmFilletSolver*> m_vFilletSolvers;       // one FilletSolver for each m_pTargetBrep edge to fillet
  SmTArray<SmFilletCorner*> m_vFilletCorners;       // one FilletCorner for each m_pTargetBrep vertex to fillet

  SmTArray<SmVertex*>       m_vFilletedVertices;    // Vertices of target brep where corners were built
                                                    //    includes vertices attached to filletedEdges
  SmTArray<SmEdge*>         m_vTopoEdges;           // Container of all topological fillet edges
  SmTArray< SmTArray< SmEdge* > > m_sLawSequenceEdgeList;  // Lists of Edges that are in a Law sequence, created by CreateSequenceOfLaws().

  SmMapPtrToPtr<SmFace, SmSurface> m_vExtendedSurfacesMap;      // Array of surfaces of original brep that have been extended
  SmMapPtrToPtr<SmVertex, SmObject> m_vVertexCornerMap;         // Holds a mapping between
                                                                //    1. vertices and array of Edgeuses being filleted that are connected to that vertex
                                                                //    2. vertices and SmFilletCorner objects
  SmMapPtrToPtr<SmEdgeuse, SmFilletSolver> m_vEdgeuseSolverMap; // Holds a mapping between edgeuses and SmFilletSolvers for every
                                                                //   fillet edge->edgeuse being filleted

  SmSelfIntersectionHandler * m_pSelfIntersectionHandler; // This object processes problems with self intersecting fillet surfaces.  
                                                          // default = returns an error.

  // gwc: Only one of m_bDoPiecewiseMerge, m_bDoTopologyInsertion, and m_bDoGlobalMerge should be TRUE in any SmFilletExecutive.
  //      They are 3 different ways to do the same thing; move m_pFilletBrep Topology and Geometry into m_pTargetBrep. 

  SmBoolean                 m_bDoPiecewiseMerge;    // TRUE = Do piecewise merge of each m_pFilletBrep->FilletFace into the original m_pTargetBrep
                                                    //        to catch Fillet/Fillet intersections.
                                                    // FALSE= don't do piecewise merges.

  SmBoolean                 m_bDoTopologyInsertion; // TRUE = insert Fillet/OrigBrep XSects into the original m_pTargetBrep.  
                                                    // FALSE= don't insert intersection geometry into m_pTargetBrep

  SmBoolean                 m_bDoGlobalMerge;       // TRUE = use slower Global Merge (m_vTI.IntersectInsertRelate())
                                                    //        to insert m_pFilletBrep into the original m_pTargetBrep.  
                                                    //        This catches large radius cases where original Brep pieces get cut away.
                                                    // FALSE= use faster local operations that might miss some large radius cases.

  SmBoolean                 m_bDoTrimming;          // TRUE = Remove original m_pTargetBrep pieces trimmed away by the filleting operation.
                                                    // FALSE= no trimming; Only imprint FilletGeometry onto original m_pTargetBrep

  SmBoolean                 m_bDoClassification;    // TRUE = classify Fillet Curves to see if they intersect any other edges.
                                                    //        This is required for the Large Radius filleting.

  SmBoolean                 m_bIsInitialized;       // TRUE = ready to do filleting because Setup() has been called.
                                                    // FALSE= not ready, need to call Setup().

  SmAttribute             * m_pAttributeE;          // Attribute placed on all m_pFilletBrep edges

  SmTArray< SmFilletSurfaceGenerator * > m_vFSGs; // These may be stored here if they have to be deleted.
                                                  // Commonly, setup and execution are done in the same
                                                  // subroutine, and the FSG is deleted automatically.
                                                  // But if the FSG's are not automatic, their pointers can
                                                  // be stored here for deletion.

  SmFilletErrorInfo         m_sErrorInfo;         // Object to hold info about
                                                  //   fillet errors

public:
  // constructor
  SmFilletExecutive
  ( 
    const SmContext & crContext,                 ///< [in] : context for new object construction
    SmBrep          * pTargetBrep = NULL         ///< [in] : optional : Brep to be filleted
  );

  // destructor
  virtual ~SmFilletExecutive();

  // User level:
  // Same parameters on each edge.
  SmStatus SetFilletParameters
  (
    SmBrep                      * pBrep,                          ///< [in] :                                                     <br>
    SmTArray< SmEdge* >         & rEdgePtrs,                      ///< [in] :                                                     <br>
    SmFilletSurfaceGeneratorType  eXSectType,                     ///< [in] :                                                     <br>
    SmFilletSolverType            eRadiusType,                    ///< [in] :                                                     <br>
    double                        dRadius,                        ///< [in] :                                                     <br>
    SmFilletLaw                 * pOptVarRadLaw,                  ///< [in] :                                                     <br>
    double                        dApproxTol,                     ///< [in] : default (if <= 0): 100 * Brep tol.                  <br>
    double                        dAngTolDeg,                     ///< [in] : default (if <= 0): 20 degrees                       <br>
    double                        dTanTolDeg,                     ///< [in] : default (if <= 0): 10 degrees                       <br>
    double                        dXSectTol,                      ///< [in] : default (if <= 0): 0.05                             <br>
    ULONG                         lContinuity=1,                  ///< [in] : 1, 2, or 3 for G1, G2, or G3                        <br>
                                                                  ///< [in] : Used only when eXSectType == SM_FSG_BLEND_CURVE     <br>
    double                        dThumbweight=1.0                ///< [in] : Used only when eXSectType == SM_FSG_BLEND_CURVE     <br>
  ); 

  // Different parameters on each edge.
  SmStatus SetFilletParameters
  (
    SmBrep                                   * pBrep,             ///< [in] :                                                      <br>
    SmTArray< SmEdge* >                      & rEdgePtrs,         ///< [in] :                                                      <br>
    SmTArray< SmFilletSurfaceGeneratorType > & rXSectTypes,       ///< [in] :                                                      <br>
    SmTArray< SmFilletSolverType >           & rRadiusTypes,      ///< [in] :                                                      <br>
    SmTArray< double >                       & rRadiusValues,     ///< [in] :                                                      <br>
    SmTArray< SmFilletLaw *>                 * pOptVarRadLaws,    ///< [in] :                                                      <br>
    double                                     dApproxTol,        ///< [in] : default (if <= 0): 100 * Brep tol.                   <br>
    double                                     dAngTolDeg,        ///< [in] : default (if <= 0): 20 degrees                        <br>
    double                                     dTanTolDeg,        ///< [in] : default (if <= 0): 10 degrees                        <br>
    double                                     dXSectTol,         ///< [in] : default (if <= 0): 0.05                              <br>
    ULONG                                      lContinuity=1,     ///< [in] : 1, 2, or 3 for G1, G2, or G3                         <br>
                                                                  ///< [in] : Used only when eXSectType == SM_FSG_BLEND_CURVE      <br>
    double                                     dThumbweight=1.0   ///< [in] : Used only when eXSectType == SM_FSG_BLEND_CURVE      <br>
  );  

  // Same parameters on each edge, taking edge numbers instead of pointers.
  SmStatus SetFilletParameters
  (                                    
    SmBrep                     * pBrep,                           ///< [in] :                                                       <br>
    SmTArray< ULONG >          & rEdgeNums,                       ///< [in] :                                                       <br>
    SmFilletSurfaceGeneratorType eXSectType,                      ///< [in] :                                                       <br>
    SmFilletSolverType           eRadiusType,                     ///< [in] :                                                       <br>
    double                       dRadius,                         ///< [in] :                                                       <br>
    SmFilletLaw                * pOptVarRadLaw,                   ///< [in] :                                                       <br>
    double                       dApproxTol,                      ///< [in] : default (if <= 0): 100 * Brep tol.                    <br>
    double                       dAngTolDeg,                      ///< [in] : default (if <= 0): 20 degrees                         <br>
    double                       dTanTolDeg,                      ///< [in] : default (if <= 0): 10 degrees                         <br>
    double                       dXSectTol,                       ///< [in] : default (if <= 0): 0.05                               <br>
    ULONG                        lContinuity=1,                   ///< [in] : 1, 2, or 3 for G1, G2, or G3                          <br>
                                                                  ///< [in] : Used only when eXSectType == SM_FSG_BLEND_CURVE       <br>
    double                       dThumbweight=1.0                 ///< [in] : Used only when eXSectType == SM_FSG_BLEND_CURVE       <br>
  );     

  // Different parameters on each edge, taking edge numbers instead of pointers.
  SmStatus SetFilletParameters
  (
    SmBrep                                   * pBrep,             ///< [in] :                                                       <br>
    SmTArray< ULONG >                        & rEdgeNums,         ///< [in] :                                                       <br>
    SmTArray< SmFilletSurfaceGeneratorType > & rXSectTypes,       ///< [in] :                                                       <br>
    SmTArray< SmFilletSolverType >           & rRadiusTypes,      ///< [in] :                                                       <br>
    SmTArray< double >                       & rRadiusValues,     ///< [in] :                                                       <br>
    SmTArray< SmFilletLaw *>                 * pOptVarRadLaws,    ///< [in] :                                                       <br>
    double                                     dApproxTol,        ///< [in] : default (if <= 0): 100 * Brep tol.                    <br>
    double                                     dAngTolDeg,        ///< [in] : default (if <= 0): 20 degrees                         <br>
    double                                     dTanTolDeg,        ///< [in] : default (if <= 0): 10 degrees                         <br>
    double                                     dXSectTol,         ///< [in] : default (if <= 0): 0.05                               <br>
    ULONG                                      lContinuity=1,     ///< [in] : 1, 2, or 3 for G1, G2, or G3                          <br>
                                                                  ///< [in] : Used only when eXSectType == SM_FSG_BLEND_CURVE       <br>
    double                                     dThumbweight=1.0   ///< [in] : Used only when eXSectType == SM_FSG_BLEND_CURVE       <br>
  ); 

  // User level, for edge filleting: Creates a FilletCorner in m_vFilletCorners for every m_vFilletedVertices vertex and 
  //                                adds [vertex,SmFilletCorner] entry pair to m_vVertexCornerMap and then 
  //                                calls DoFilleting() after SetFilletParameters() call.
  SmStatus DoFillet();

  // User level, for surface-surface filleting: sets up and then calls DoSurfaceFilleting()
  // note: It is possible to both mirror and complement circular cross sections.
  SmStatus SurfaceSurfaceFillet
  (
    const SmContext      & crContext,                              ///< [in] : Creation context for the new brep that contains the results.                             <br>
    SmSurface            * pSur1,                                  ///< [in] : Surface1 to be filleted (can be from different Breps)                                    <br>
    SmSurface            * pSur2,                                  ///< [in] : Surface2 to be filleted (can be from different Breps)                                    <br>
    double                 dFilletRadius1,                         ///< [in] : signed radius from pSur1 (FilletCenter = Surf/Surf XSect of pSur1Offset(SignedDist1))    <br>
    double                 dFilletRadius2,                         ///< [in] : signed radius from pSur2 (FilletCenter = Surf/Surf XSect of pSur2Offset(SignedDist2))    <br>
    double                 dTolerance,                             ///< [in] : Tolerance: max dist from rails to base surfaces                                          <br>
    SmBrep              *& rpResult,                               ///< [out]: Brep containing the new fillet surface(s)                                                <br>
    ULONG                  lXSectType    =2,                       ///< [in] : 0=linear, 1=approx circular, 2=circular (rational)                                       <br>
    double                 dXSectAccuracy=0.1,                     ///< [in] : Relative accuracy of approx circular                                                     <br>
    SmBoundaryTrimmingType eFilTrimType=SM_BT_MAXIMAL,             ///< [in] : SM_BT_NONE,   = No fillet triming just generate entire fillet                            <br>
                                                                   ///<      : SM_BT_MINIMAL = Trim using ISO curves to minimal intersections on face boundaries        <br>
                                                                   ///<      : SM_BT_MAXIMAL = Trim using ISO curves to maximal intersections on face boundaries        <br>
                                                                   ///<      : SM_BT_BEVEL,  = Create a bevel (line in parameter space) between minimal and maximal     <br>
                                                                   ///<      :                 intersections on each end of fillet.                                     <br>
                                                                   ///<      : SM_BT_BLEND   = Create a blend (Hermite in parameter space) between minimal and          <br>
                                                                   ///<      :                 maximal intersections on each end of fillet.                             <br>
    ULONG                  lBaseTrimType=0,                        ///< [in] : base surface trimming: 0=none; 1=normal; 2=reverse trim.                                 <br>
    SmBoolean              bMirror    =FALSE,                      ///< [in] : TRUE = mirror fillet to produce an inside out fillet, FALSE=don't                        <br>
    SmBoolean              bComplement=FALSE                       ///< [in] : TRUE = complement circular and approx circular cross sections, FALSE=don't.              <br>                     
  );    

  SmStatus GetPreviewFilletSurfaces
  ( 
    SmTArray<SmSurface*>& rFilletSurfaces                         ///< [out] :        <br>
  );

                                                       
  // Internal:
  void     AddFilletSurfaceGenerator (SmFilletSurfaceGenerator * pFSG) { m_vFSGs.Add( pFSG ); }
  void     AddTopoEdges              (SmFilletEdge * pTopoEdge)        { m_vTopoEdges.Add(pTopoEdge); }
  SmStatus CreateFilletBrep          (SmFilletErrorInfo * pOptFilletErrorInfo = NULL);
  SmStatus CreateFilletCorners       (SmFilletErrorInfo * pOptFilletErrorInfo = NULL);
  SmStatus DoFilleting               (SmFilletErrorInfo * pOptFilletErrorInfo = NULL); // note: increments unlocked mark value
  SmStatus DoSurfaceFilleting        (SmFilletErrorInfo * pOptFilletErrorInfo = NULL); // note: increments unlocked mark value
  SmStatus DoPiecewiseMerge          ();                                               // note: increments unlocked mark value
  SmStatus LoadFilletSolver          (SmFilletSolver * pFS);
  SmStatus RemoveTopoEdges           ();
  void     RemoveFromTopoEdges( SmFilletEdge * pTopoEdge )
  {
    ULONG lIndex;
    if(m_vTopoEdges.FindElement( pTopoEdge, lIndex ))
    {
      m_vTopoEdges.RemoveAt( lIndex );
    }
  }

  SmStatus TestAndConvertBigRadFillet( SmFilletGeom * pFG, SmBoolean    & rbConvertedToCorner );

  SmStatus UpdateTopologyChanges( SmAttribute  *pFaceAttr, SmFace *pOrigFace );

  // simple data access
  const SmContext           * GetContext()            { return &m_crContext; };
  SmBrep                    * GetTargetBrep()         { return m_pTargetBrep; }
  SmBrep                    * GetFilletBrep()         { return m_pFilletBrep; }
  SmFilletBrep              * GetPseudoBrep()         { return m_pPseudoBrep; }

  SmBoolean                   GetDoClassification()   { return m_bDoClassification; }
  SmBoolean                   GetDoGlobalMerge()      { return m_bDoGlobalMerge; }

  SmTArray<SmFilletCorner*> & GetFilletCorners()      { return m_vFilletCorners; }
  SmTArray<SmFilletSolver*> & GetFilletSolvers()      { return m_vFilletSolvers; }
  SmFilletErrorInfo         * GetFilletErrorInfo()    { return &m_sErrorInfo; }

  SmFilletCorner            * GetFilletCornerOfVertex (SmVertex             * pVertex)     const;
  SmFilletSolver            * GetFilletSolverOfEdgeuse(SmEdgeuse            * pEdgeuse)    const;
  SmFilletSolver            * GetFilletSolverOfEdge   (SmEdge               * pEdge   )    const;

  ULONG                       FindCornerIndex( const SmFilletCorner * pCorner )    const;
  ULONG                       FindSolverIndex( const SmFilletSolver * pSolver )    const;

  SmStatus                    GetRailOfEdgeuse        
  (
    SmEdgeuse            * pEdgeuse,
    SmFilletSolver      *& rpFilletSolver,
    ULONG                & rlRailIndex
  ) const;

  

  // simple assignments
  void     SetDoClassification       (SmBoolean bDoClassification) { m_bDoClassification    = bDoClassification; }
  void     SetDoPiecewiseMerge       (SmBoolean bDoPiecewiseMerge) { m_bDoPiecewiseMerge    = bDoPiecewiseMerge; }
  void     SetDoTopologyInsertion    (SmBoolean bDoInsert)         { m_bDoTopologyInsertion = bDoInsert; }
  void     SetDoGlobalMerge          (SmBoolean bDoGlobalMerge)    { m_bDoGlobalMerge       = bDoGlobalMerge; }
  void     SetDoTrimming             (SmBoolean bDoTrim)           { m_bDoTrimming          = bDoTrim; }
  SmStatus SetBevelCorner            (SmVertex * pCornerVert);
  SmStatus SetFilletBrepEdgeMap      ();
  void     SetSelfIntersectionHandler(SmSelfIntersectionHandler *pSelfIntersectionHandler)
              { m_pSelfIntersectionHandler = pSelfIntersectionHandler; }

  // Other processing
  SmStatus ProcessSmoothEdgeSequences();
  SmStatus CollectTangentEdgeSequence( SmTArray< SmFilletSolver* > & rFilletSolvers,
                                       SmTArray< SmFilletSolver* > & rSmoothSequence );
  SmStatus CreateSmoothEdgeSequence  ( SmTArray< SmFilletSolver* > & rSmoothSequence );
  SmStatus CreateSequenceOfLaws
  (
    const SmContext & crContext,
    SmFilletLaw *pLaw,                         ///< [in] : the single Law to be copied on each Edge     <br>
    SmTArray< SmEdge* > & rEdges,              ///< [in] : the Edge sequence, ordered                   <br>
    SmTArray< SmFilletLaw* > & rLaws           ///< [out]: one Law for each input Edge                  <br>
  );

  // Error info handling
  void ReportErrors() { m_sErrorInfo.Report(); }

  void NoteFilletError
  (
    SmFilletErrorType      eErrorType,
    SmFilletEdge         * pFilletEdge,
    const TCHAR          * cComment=NULL
  );

  void NoteFilletError
  ( 
    SmFilletErrorType      eErrorCode,
    SmFilStatus            eFilletStatus,
    const SmEdge         * pFilletedEdge,
    const SmFace         * pTargetFace,
    const SmFilletEdge   * pFilletEdge,
    const SmSurface      * pFilletSurface,
    const TCHAR          * cComment = NULL 
  )
  {
    GetFilletErrorInfo()->NoteError( eErrorCode, eFilletStatus,
                                     pFilletedEdge, pTargetFace,
                                     pFilletEdge, pFilletSurface,
                                     cComment );
  }

  void NoteFilletError
  ( 
    SmFilletErrorType      eErrorType,
    SmFilletCornerType     eCornerType,
    SmFilStatus            eFilletStat,
    TCHAR                * sCornerName,
    const SmVertex       * pFilletedVertex,
    const SmSurface      * pFilletSurface1,
    const SmSurface      * pFilletSurface2,
    const SmFilletVertex * pStartFV,
    const SmFilletVertex * pEndFV,
    const TCHAR          * cComment = NULL 
  )
  {
    GetFilletErrorInfo()->NoteError( eErrorType, eCornerType,
                                     eFilletStat, sCornerName,
                                     pFilletedVertex, pFilletSurface1,
                                     pFilletSurface2, pStartFV,
                                     pEndFV, cComment );
  }


  void NoteFilletError
  (
    SmFilletErrorType             eErrorType,
    SmFilStatus                   eFilletStat,
    const SmEdge                * pFilletedEdge,
    // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 1 line
    // rm : SmTArray<SmBSplineSurface*> & rapFilSurfs,
    SmTArray<SM_FILLETSURF_TYPE*> & rapFilSurfs,
    SmTArray<SmBSplineCurve*>   & rapFilCurvs,
    SmTArray<SmFilletEdge*>     & rapFilEdges,
    SmPoint3d                   & vTsectPos,
    const TCHAR                 * cComment = NULL )
  {
    GetFilletErrorInfo()->NoteError( eErrorType, 
                                     eFilletStat,
                                     pFilletedEdge, 
                                     rapFilSurfs,
                                     rapFilCurvs, 
                                     rapFilEdges,
                                     vTsectPos, 
                                     cComment );
  }

// This must be outside SM_DEBUG_CODE to be initialized by some compilers
private:
  int m_iDebugLevel; // Set this in the debugger.

public:
#ifdef SM_DEBUG_CODE
  int DebugLevel() { return m_iDebugLevel; }
#else
  int DebugLevel() { return 0; }
#endif // SM_DEBUG_CODE

}; // end class SmFilletExecutive


#define FILEXEC_ERR(msg) { \
    TCHAR buff[SM_TBLOCK_SIZE]; \
    smos_sprintf(buff,_T("%s\n"),(msg)); \
    smos_WriteBuffer(buff); \
    if (TRUE) { return SM_ERR; } \
}

#define FILEXEC_VERT_ERR(vert,msg) { \
    TCHAR name[SM_TBLOCK_SIZE]; \
    (vert)->GetName(name, SM_TBLOCK_SIZE); \
    TCHAR buff[SM_TBLOCK_SIZE]; \
    smos_sprintf(buff,_T("^^^^ %s -- %s\n"), name, (msg)); \
    smos_WriteBuffer(buff); \
    if (TRUE) { return SM_ERR; } \
}

#define FILEXEC_CORNER_ERR(corner,msg) { \
    TCHAR name[SM_TBLOCK_SIZE]; \
    (corner)->GetName(name, SM_TBLOCK_SIZE); \
    TCHAR buff[SM_TBLOCK_SIZE]; \
    SmPoint3d pnt; \
    if( corner->GetFilletedVertex() ) { \
    pnt = (corner)->GetFilletedVertex()->GetPoint(); \
    } \
    smos_sprintf(buff,_T("## %.256s at (%.4lf, %.4lf, %.4lf) -- %.256s\n"), \
    name, pnt.x, pnt.y, pnt.z, (msg)); \
    smos_WriteBuffer(buff); \
    if (TRUE) { return SM_ERR; } \
}

#define FILEXEC_SOLVER_ERR(solver,msg) { \
    SmEdge * edge = (solver)->GetEdgeuse(0)->GetEdge(); \
    SmTArray<SmEdge*> edges; \
    edge->GetBrep()->GetEdges(edges); \
    ULONG index = 0; \
    edges.FindElement(edge,index); \
    TCHAR name[SM_TBLOCK_SIZE]; \
    (solver)->GetName(name, SM_TBLOCK_SIZE); \
    TCHAR buff[SM_TBLOCK_SIZE]; \
    smos_sprintf(buff,_T("%.256s of edge# %ld -- %.256s\n"), \
    name, index, (msg)); \
    smos_WriteBuffer(buff); \
    if (TRUE) { return SM_ERR; } \
}


/*******************************************************************//**
PURPOSE: A class to temporarily change a variable-radius function.

NOTES:
   When a variable-radius fillet is defined on an edge that meets smoothly
   with another edge, it must have the same radius as the fillet on the
   smoothly-joining edge.  The fillets must match where they meet, which
   is not necessarily at the vertex (which is how they are defined), it
   will be where the fillets hit a side edge -- the third edge coming into
   the vertex that connects the two filleted edges.

   When intersecting the fillet rail with the side edge, the variable-radius
   function must have its end value, wherever it ends up intersecting.
   The way to do that is to pretend that the var-rad function is constant,
   at its end value.  So when doing that intersection, we replace the
   var-rad function with a constant.

   This class currently will also act on any variable-radius function,
   even if it doesn't meet another fillet smoothly.  For example, when a
   linear fillet meets two other fillets at a corner, the specified end
   radius value will obtain wherever the var-rad fillet ends, at the
   corner blend.  Without this, the specified end radius value would not
   be realized anywhere.  It is possible that we might want to revert this
   behavior: invoke this only if the corner has exactly one other fillet,
   and the edges meet smoothly. [Reg iter:306; B570]

   This class behaves like SmObjDelete: its constructor does the work to
   set it up, and the destructor puts things back where they should be.

   If m_pSavedBSp is not Null, then m_pFS contains an SmBSplineFilletLaw,
   and m_pSavedBSp gets swapped back in to the FilletSolver upon destruction.
   Else if m_dStartRad and m_dEndRad are positive, then m_pFS contains an
   SmLinearFilletLaw, and they get swapped back into that.
   Otherwise, nothing was changed, and the destructor does nothing.
   That happens for constant-radius fillets, for example.

***********************************************************************/
class SmTemporaryRadiusChange
{
  SmBSplineCurve *m_pSavedBSp;  // For SmBSplineFilletLaw.  See Notes above.
  double          m_dStartRad;  // For SmLinearFilletLaw.   See Notes above.
  double          m_dEndRad;
  SmEdgeuse      *m_pSideEU;    // Edgeuse of the other filleted Edge, whose fillet we're intersecting
  SmFilletSolver *m_pFS;        // This FilletSolver
  ULONG m_lRailIdx;             // (Not used.)

public:
  SmStatus SetUp();
  SmStatus Revert();

  // Constructor:
  SmTemporaryRadiusChange( SmFilletSolver *pFS, ULONG lRailIndex, SmEdgeuse *pSideEU )
    : m_pSavedBSp(NULL),
      m_dStartRad(0),
      m_dEndRad(0),
      m_pSideEU(pSideEU),
      m_pFS       ( pFS ),
      m_lRailIdx  ( lRailIndex )
  {
    this->SetUp();
  }

  // Destructor:
  ~SmTemporaryRadiusChange()
  {
      this->Revert();
  }
};  // end class SmTemporaryRadiusChange


//
// Below are all the SmFilletLaw classes, moved here from SmSurfaceIntersector.cpp
// And they could go to their own file one day.
//

/*******************************************************************//**
PURPOSE: Law function used to specify fillet radius and distances.

NOTES:
   m_vEdgeMap: This is normally [0,1], which means the extent of the
   radius function as given.  The radius function can be extended, but
   the 0-1 range will correspond to the radius function as given, which
   matches the extent of the final fillet.

   An exception is when a single radius function is specified to extend
   over more than one consecutive edge.  In that case, the [0-1] range
   still specifies the radius function as given, but only a portion of it
   maps to the current edge.  That should be set up proportionally to the
   arc lengths of the edges in the sequence.  For example, if the radius
   function spanned two edges with lengths 2 and 3, then the EdgeMaps
   would be [0.0, 0.4] and [0.4, 1.0].

   So generally, pass in [0,1], or nothing.  'Nothing' is specified by
   either a Null pointer, or a zero-length interval.
***********************************************************************/
class SM_EXPORT SmFilletLaw : public SmObject
{
public:
  SmExtent1d       m_vEdgeMap;         // Subset of the fillet-radius function which maps to the edge.  
                                       // See notes above.
  double           m_dEdgeArcLength;   //
  SmBSplineCurve * m_pExtended;        // Extended Edge Curve

                                       // constructor, destructor
  SmFilletLaw();
  SmFilletLaw(const SmFilletLaw & crOther);
  virtual ~SmFilletLaw();

  // Assignment operator
  virtual SmFilletLaw & operator=(const SmFilletLaw & crOther);

  // Copy method.  (Only way to make a virtual copy operator.)
  virtual SmStatus Copy
   (const SmContext & crContext,       // in :
    SmFilletLaw    *& rpNewFilletLaw)  // in :
   const;

  // get fillet radius and its derivatives at a point along a fillet edge
  virtual SmStatus Evaluate
  (
    double,              ///< [in] : dParameter = target parameter - range:[crCurveInterval.Min,Max]                          <br>
    const SmExtent1d &,  ///< [in] : crCurveInterval = =interval defining range of fillet edge                                <br>
    SmBoolean,           ///< [in] : bReverseOrientation: TRUE = parameters run from interval end to interval start           <br>
    double[3]            ///< [out]: dValues[0] = fillet-radius at dParameter value                                           <br>
                         ///<      : dValues[1] = 1st derivative: d(fillet-radius)/d(Parameter)                               <br>
                         ///<      : dValues[2] = 2nd derivative: d2(fillet-radius)/d(Parameter)**2                           <br>
  ) const                                      
  {
      SE(SM_ERR); return SM_ERR;
  }

  SmExtent1d       GetEdgeMap()                         { return m_vEdgeMap; }
  SmExtent1d       GetEdgeArcLength()                   { return m_dEdgeArcLength; }
  SmBSplineCurve * GetExtendedEdgeCurve()               { return m_pExtended; }
                                                        
  void SetEdgeMap(SmExtent1d &rEdgeMap)                 { m_vEdgeMap = rEdgeMap; }
  void SetEdgeArcLength(double dEdgeArcLength)          { m_dEdgeArcLength = dEdgeArcLength; }
  void SetExtendedEdgeCurve(SmBSplineCurve * pExtended) { m_pExtended = pExtended; }

  virtual SmBoolean IsLinearLaw() const                 { return FALSE; }

  // Use the following method to apply a single radius function along a sequence of
  // smoothly-meeting edges.  See Notes in the method's header file.
  // Note, this is no longer a user-level call: users should call SmFilletExecutve::CreateSequenceOfLaws().
  static SmStatus CreateLawSequence
  (
    const SmContext & crContext,
    SmFilletLaw *pLaw,                         ///< [in] : the single Law to be copied on each Edge                     <br>
    SmTArray< SmEdge* > & rEdges,              ///< [in] : the Edge sequence, ordered                                   <br>
    SmTArray< SmFilletLaw* > & rLaws           ///< [out]: one Law for each input Edge                                  <br>
  );

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmFilletLaw, SmObject, SmFilletLaw_TYPE);

}; // end class SmFilletLaw

/*******************************************************************//**
PURPOSE: Linear law function used to specify radius as a linear function.

NOTES:
***********************************************************************/
class SM_EXPORT SmLinearFilletLaw : public SmFilletLaw
{
protected:
  double m_dStartValue;     // start fillet-radius value 
  double m_dEndValue;       // end   fillet-radius value

public:
  SmLinearFilletLaw
  (
    double dStartValue,                  ///< [in] : radius value at fillet edge start    <br>
    double dEndValue,                    ///< [in] : radius value at fillet edge end      <br>
    SmExtent1d * pOptEdgeMap = NULL      ///< [in] :                                      <br>
  );

  SmLinearFilletLaw(const SmLinearFilletLaw & crOther);

  using SmFilletLaw::operator=;
  virtual SmLinearFilletLaw & operator=(const SmLinearFilletLaw & crOther);

  // Copy method.  (Only way to make a virtual copy operator.)
  virtual SmStatus Copy
  (
    const SmContext & crContext,
    SmFilletLaw    *& rpNewFilletLaw
  ) const;

  double GetStartRadius()           { return m_dStartValue; }
  double GetEndRadius()             { return m_dEndValue; }

  void SetStartRadius(double dRad)  { m_dStartValue = dRad; }
  void SetEndRadius(double dRad)    { m_dEndValue = dRad; }

  virtual SmStatus Evaluate
  (
    double dParameter,                       ///< [in] : target parameter - range:[crCurveInterval.Min,Max]             <br>
    const SmExtent1d & crCurveInterval,      ///< [in] : interval defining range of fillet edge                         <br>
    SmBoolean bReverseOrientation,           ///< [in] : TRUE = parameters run from interval end to interval start      <br>
    double dValues[3]                        ///< [out]: dValues[0] = fillet-radius at dParameter value                 <br>
                                             ///<      : dValues[1] = 1st derivative: d(fillet-radius)/d(Parameter)     <br>
                                             ///<      : dValues[2] = 2nd derivative: d2(fillet-radius)/d(Parameter)**2 <br>
  ) const;

  virtual SmBoolean IsLinearLaw() const { return TRUE; }

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmLinearFilletLaw, SmFilletLaw, SmLinearFilletLaw_TYPE);

}; // end class SmLinearFilletLaw 

/*******************************************************************//**
PURPOSE: A B-Spline curve used to specify radius.  The constructor consumes the curve.

NOTES: The radius function is represented as the x-coordinate
       of the SmBSplineCurve; the y- and z-coordinates are ignored.
***********************************************************************/
class SM_EXPORT SmBSplineFilletLaw : public SmFilletLaw
{
protected:
  SmBSplineCurve * m_pLawCurve;        // the fillet-radius function represented as a BSpline curve
  SmBSplineCurve * m_pExtendedLaw;     // a G1/G2 extension to m_pLawCurve created whenever Evaluate
                                       // is called with a parameter value outside the m_pLawCurve parameter range

public:
  SmBSplineFilletLaw
  (
    SmBSplineCurve * pCurve,                     ///< [in] : fillet-radius function                                                     <br>
    const SmExtent1d & crEdgeMap                 ///< [in] : Subset of the curve which maps to the edge.                                <br>
                                                 //      That way we have control of what happens outside of the interval of the edge.  <br>
  );

  SmBSplineFilletLaw
  (
    const SmContext & crContext,                 ///< [in] : context for new object construction                                                     <br>
    const SmTArray<double> & crParameters,       ///< [in] : given parameter values                                                                  <br>
    const SmTArray<double> & crRadii,            ///< [in] : corresponding fillet-radii values                                                       <br>
    ULONG lLawBlendingType,                      ///< [in] : 0 - Point Interpolation.                                                                <br>
                                                 ///<      : 1 - Points and Parallel Derivatives                                                     <br>
    const SmExtent1d & crEdgeMap,                ///< [in] : Subset of the curve which maps to the edge.                                             <br>
                                                 ///<      : That way we have control of what happens outside of the interval of the edge.           <br>
                                                 ///<      : If passed in Null, will use the domain of the B-Spline law curve that we create here.   <br>
    SmBoolean & bError                           ///< [out]: TRUE = failed to build pLawCurve                                                        <br>
                                                 ///<      :    FALSE= OK                                                                            <br>
  );

 

  SmBSplineFilletLaw
  (
    const SmContext & crContext,                 ///< [in] :
    double dStartRadius,                         ///< [in] : Start Radius of Blending Function                                    <br>
    double dEndRadius,                           ///< [in] : End Radius of Blending Function                                      <br>
    double dOuterRadius,                         ///< [in] : Outer Radius of Blending Function - usually the largest radius       <br>
    double dAngleSpan,                           ///< [in] : Angle span of arc that has dOuterRadius                              <br>
    double dStartDeriv,                          ///< NotUsed: [in] : Start Derivative if we don't start with a constant radius thing.     <br>
    double dEndDeriv,                            ///< NotUsed: [in] :
    SmExtent1d * pOptEdgeMap = NULL              ///< [in] :
  );

  SmBSplineFilletLaw
  (
    const SmContext & crContext,                  ///< [in] :            <br>
    double dStartValue,                           ///< [in] :            <br>
    double dEndValue,                             ///< [in] :            <br>
    double dStartDeriv,                           ///< [in] :            <br>
    double dEndDeriv,                             ///< [in] :            <br>
    SmExtent1d * pOptEdgeMap = NULL               ///< [in] :            <br>
  );

  SmBSplineFilletLaw(const SmBSplineFilletLaw & crOther);

  virtual ~SmBSplineFilletLaw();

  using SmFilletLaw::operator=;
  virtual SmBSplineFilletLaw & operator=(const SmBSplineFilletLaw & crOther);

  // Copy method.  (Only way to make a virtual copy operator.)
  virtual SmStatus Copy
  (
    const SmContext & crContext,            ///< [in] :                  <br>
    SmFilletLaw    *& rpNewFilletLaw        ///< [out]:                  <br>
  ) const;

  virtual SmStatus Evaluate
  (
    double dParameter,                       ///< [in] : target parameter - range:[crCurveInterval.Min,Max]                 <br>
    const SmExtent1d & crCurveInterval,      ///< [in] : interval defining range of fillet edge                             <br>
    SmBoolean bReverseOrientation,           ///< [in] : TRUE = parameters run from interval end to interval start          <br>
    double dValues[3]                        ///< [out]: dValues[0] = fillet-radius at dParameter value                     <br>
                                             ///<      : dValues[1] = 1st derivative: d(fillet-radius)/d(Parameter)         <br>
                                             ///<      : dValues[2] = 2nd derivative: d2(fillet-radius)/d(Parameter)**2     <br>
  ) const;

  virtual SmBoolean IsLinearLaw() const
  {
    if(m_pLawCurve->GetDegree() == 1) { return TRUE; }
    return FALSE;
  }

  SmBSplineCurve * GetLawCurve()                { return m_pLawCurve; }
  SmBSplineCurve * GetExtendedCurve()           { return m_pExtendedLaw; }

  void SetLawCurve(SmBSplineCurve *pBSPl)       { m_pLawCurve = pBSPl; }
  void SetExtendedCurve(SmBSplineCurve *pBSPl)  { m_pExtendedLaw = pBSPl; }

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmBSplineFilletLaw, SmFilletLaw, SmBSplineFilletLaw_TYPE);

}; // end class SmBSplineFilletLaw

#endif // !__SMFILLETEXECUTIVE_H__
