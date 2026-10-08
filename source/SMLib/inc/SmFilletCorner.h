// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFilletCorner.h
* PURPOSE: Header file for SmFilletCorner object.
**********************************************************************/

#ifndef __SMFILLETCORNER_H__
#define __SMFILLETCORNER_H__

#ifndef __SMFILLETGEOM_H__
#include <SmFilletGeom.h>
#endif

#ifndef __SMTANGENTFIELD_H__
#include <SmTangentField.h>
#endif

#ifndef __SMBSPLINESURFACE_H__
#include <SmBSplineSurface.h>
#endif

#ifndef __SMFILLETEXECUTIVE_H__
#include <SmFilletExecutive.h>
#endif

/*******************************************************************//**
PURPOSE: The fillet corner object defines and controls what happens
     at a vertex in terms of combining adjacent edge fillets.
     Class SmFilletCorner is an abstract class.  The various types
     (corresponding to the values of the enum SmFilletCornerType)
     are derived (possibly indirectly) from SmFilletCorner.

NOTES:
***********************************************************************/
class SM_EXPORT SmFilletCorner : public SmObject
{
  friend class SmFilletExecutive;

protected:
  const SmContext         & m_crContext;             // Context for all temporary and permanent object allocation.
  SmFilletExecutive       * m_pExecutive;            // Executive that controls global filleting operations
  SmFilletBrep            * m_pPseudoBrep;           // Ptr to m_pExecutive->m_pPseudoBrep; Container of all topo entities
                                                     
  const SmVertex          * m_cpVertex;              // Vertex of original brep
  SmTArray<SmEdgeuse*>    * m_pSolverEUs;            // All (outward)solver-edgeuses surrounding the corner (edges to be filleted)
  SmTArray<SmEdgeuse*>    * m_pAllEUs;               // All (outward)edgeuses surrounding the corner

  SmTArray<SmFilletVertex*> m_vVertices;             // Ordered array of fillet vertices - vertices added as result of filleting
  SmTArray<SmFilletEdge*>   m_vEdges;                // Ordered array of fillet edges around corner as result of filleting (not the rails)
  SmTArray<SmSurface*>      m_vSurfaces;             // New surfaces constructed to fill in endFaces and 'holes' at this filletCorner
                                                     
  SmBoolean                 m_bBevel;                // TRUE = a 'bevel' corner (see comments below)
  SmBoolean                 m_bBlending;             // TRUE = a 'blending' corner
  SmBoolean                 m_bChamfer;              // TRUE = a chamfer(i.e. 'planar')
  SmBoolean                 m_bDegenerate;           // TRUE=all rail curves meet at one point and
                                                     //   m_vVertices.GetSize() == 1
                                                     //   m_vEdges.GetSize()    == 0
                                                     // FALSE= rail curves terminate at a variety of
                                                     //   filletVertices offset from origVertex
                                                     
  SmBoolean                 m_bCalcAsConstRad;       // See notes below.

  SmBoolean                 m_bTopologyAdjusted;     // TRUE = corner topology has been altered
  SmBoolean                 m_bExtendedSurfacePatch; // TRUE = EndFace->Surface is defined by sideFace->ExtendedSurface
                                                     // FALSE=EndFace->Surface is defined by interpolating filletEdges
                                                     // default:[TRUE]
                            
  double                    m_dSetBackDist;          // Distance of setback (from the filleted vertex).
                                                     // No setbacks if m_dSetBackDist = 0.0
                                                     // Note, this makes no sense with Bevel.
  double                    m_dThisApproxTol3d;   // 3D tolerance for corner patch creation
  double                    m_dTangencyTolRadians;   // Angle tol for tangent field approximation
                            
  SmFilStatus               m_eStatus;               // Status of corner computation

// Notes:
// m_bBlending and m_bBevel:  A bevel is a non-smooth intersection between
// two fillet surfaces, and a blend refers to an extra surface put in as a
// 'vertex fillet' at a corner.  Picture a simple 3x3 blend on a box: such
// a fillet could be either beveled or blended at the corner.  If beveled,
// the fillet surfaces are run long and intersected with each other.  The
// result is three fillet-fillet intersections, that meet at a point in the
// middle. If blended, a 3-sided corner patch is added, which joins smoothly
// with the ends of the three fillets.
//
// There is another case, where two edges to be filleted meet smoothly at
// the corner.  If they have the same radii (the fillet probably won't
// work if they don't), and the adjoining faces also meet smoothly, then
// the fillet surfaces will meet smoothly, end to end, and no extra surface
// patch is created.  In practice, this is also called a Blending corner.
//
// So if m_bBlending is true, then the fillet surfaces will just be
// ended at some cross section, and another surface (either a new cap
// surface or another fillet surface) will meet it smoothly, but if
// m_bBevel is true, the fillet surfaces will be extended and intersected
// with each other.  Also, in practice, the two variables are mutually
// exclusive: one or the other must be set.
//
// The values are set by the algorithms, according to what makes
// more sense.  However, the user can specify which is done, via
// SmFilletCorner::SetBevel( SmBoolean ).
//

//
// m_bCalcAsConstRad: If TRUE, then artificially treat a variable-radius fillet coming into
// this corner as having a constant radius, which is the specified radius where it ends at
// this Corner.  This is done to calibrate a radius function so that it will end with the
// specified end radius value, before we're sure exactly where the fillet surface will end.
// See class SmTemporaryRadiusChange.
//

public:
  // Constructor
  SmFilletCorner(const SmContext      & crContext,
                 const SmVertex       * cpVertex,
                 SmTArray<SmEdgeuse*> * pSolverEUs,
                 SmTArray<SmEdgeuse*> * pAllEUs,
                 SmFilletExecutive    * pExec,
                 double                 dApproxTol3d,
                 double                 dTangencyTolRadians);

  // static constructor
  static SmFilletCorner * Create(const SmContext      & crContext,
                                 const SmVertex       * cpVertex,
                                 SmTArray<SmEdgeuse*> * pSolverEUs,
                                 SmFilletExecutive    * pExec);
  // destructor
  virtual ~SmFilletCorner();

  // key SmFilletCorner methods driven by SmFilletExecutive::CreateFilletBrep() to Fillet a Brep
  virtual SmStatus MakeCornerTopology   ()=0 ;                 // 1st : Add FilletVertices (topo only) connected by FilletEdges (topo only)
  SmStatus         CalcCornerVertGeom   () ;                   // 2nd : Set FilletVertex point positions
  SmBoolean        CheckAndFixDegeneracy() ;                   // 3rd : check for and deal with degenerate corner
  virtual SmStatus AdjustTopology       (){return SM_SUCCESS;} // 4th : Adjust topo for NonDegenerate cases depending on part geometry
                                                               // 5th : SmFilletSolver::CalcFilletGeom()
                                                               //        Compute untrimmed filletSurface and railCurve geometry 
  virtual SmStatus AdjustTangentCorner  (){return SM_SUCCESS;} // 6th : Similar to AdjustTopology - uses more geometry checks
                                                               // 7th : SmFilletSolver::TestRollOver() XSect RailCurves with OrigFace->Bndry
                                                               //         to find if railCurve "RolledOver" other faces. If so, adjust fillets.
  SmStatus         CalcCornerEdgeGeom   () ;                   // 8th : if needed, Recompute FilletEdges of AdjustedCorners because of RollOver.
  SmStatus         CalcCornerGeom       () ;                   // 9th : if needed, call CalcCornerPatch() or CreateEndPatch()
                                                               //       gwc: removed CalcCornerGeom() virtual attribute - didn't see any derived class implementations
  virtual SmStatus   CreateCornerPatch  () ;                   //     : calc corner SurfaceFill patch and store in m_vSurfaces, call MakeFaceBrep()
  SmBoolean          MakeAnalyticCornerSurface() ;             //     : rtn TRUE when analytic Surface created and added to m_vSurfaces
  SmStatus           CreateEndPatch     () ;                   //     : calc corner EndCap patch and store in m_vSurfaces,      call MakeFaceBrep()
  SmStatus           MakeFaceBrep       () ;                   //     : Create face (and Edges and Verts) in m_pFilletBrep from m_vSurfaces with call m_pFilletBrep->MakeFaceWithCurves
                                                               //       gwc: removed MakeFaceBrep() virtual attribute - didn't see any derived class implementations                     

                                                               // notes:
                                                               // 10th: SmFilletGeom::MakeFaceBrep(m_pFilletBrep)
                                                               //          Make EdgePatch SmFace in m_pFilletBrep for every m_vFilletGeoms[i]->FilletSurface
                                                               // 11th: m_pFilletBrep->StitchFaces()
  // other supporting SmFilletCorner methods

  // Register the OriginalFace/ExtSurface pair in the SmFilletExecutive.
  void                    AddExtendedOriginalSurface(SmFace    * pOriginalFace,
                                                     SmSurface * pExtSurface);
  // For FilletVertex's created during filleting:
  void                    AddFilletVertex( SmFilletVertex *pFV ) { m_vVertices.Add( pFV ); }
  SmStatus                InsertIntersectionTopology();
  // gwc: I think pSideSurface no longer has to be a BBSplineSurface - replace 2 line
  // rm : SmStatus                MakeConstantRadiusFillet(SmBSplineSurface *& rpSurface);
  // rm : SmStatus                MakeVariableRadiusFillet(SmBSplineSurface *& rpSurface);
  SmStatus                MakeConstantRadiusFillet(SM_FILLETSURF_TYPE *& rpSurface);
  SmStatus                MakeVariableRadiusFillet(SM_FILLETSURF_TYPE *& rpSurface);
  void                    ReInitializeCornerEdges();
  SmStatus                UpdateSplitFace( SmFace* pOrigFace, SmFace *pSplitFace );
  SmStatus                SqueezeEdge(SmFilletEdge *pEdgeToSqueeze);
  SmStatus                BuildTangentField(ULONG             lCurveIndex,
                                            SmBoolean         bIsIsoDir,
                                            SmTangentField *& pTanField);
  // simple data access
  ULONG               FindVertexIndex( const SmFilletVertex *pFilVtx  ) const;
  ULONG               FindEdgeIndex  ( const SmFilletEdge   *pFilEdge ) const;
  ULONG               FindIndexInExec() const;
  virtual SmFilletCornerType GetCornerType() const=0;           // always implemented by derived classes
  virtual void               GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const=0; // always implemented by derived classes
  double              GetThisApproxTol3d() const                       { SM_ASSERT_TOL(m_dThisApproxTol3d) ; return m_dThisApproxTol3d; }
  const SmContext   & GetContext()                                     { return m_crContext; }
  SmSurface         * GetExtendedSurface(SmFace * pOriginalFace);
  SmFilletExecutive * GetFilletExecutive()                             { return m_pExecutive; }
  void                GetFilletEdges(SmTArray<SmFilletEdge*> & rEdges) { rEdges.Append(m_vEdges); }
  void                GetFilletVertices(SmTArray<SmFilletVertex*> & rVerts) { rVerts.Append(m_vVertices); }
  const SmVertex    * GetFilletedVertex()                              { return m_cpVertex; }
  double              GetSetBackDist()                                 { return m_dSetBackDist; }
  SmFilStatus         GetStatus()                                      { return m_eStatus; }
  double              GetTangencyTolRadians()                          { SM_ASSERT_TOL(m_dTangencyTolRadians) ; return m_dTangencyTolRadians; }

  // Other data access
  void                     GetFilletSolvers( SmTArray<SmFilletSolver*> & rSolvers );
  virtual SmFilletSolver * GetTangentFilletSolver( const SmFilletSolver *pFS ) { SM_REF1(pFS) ; return NULL; }

  // simple assignments
  void SetBevel         ( SmBoolean bIsBevel )  { m_bBevel          = bIsBevel; }
  void SetSetBackDist   ( double dSetBackDist ) { m_dSetBackDist    = dSetBackDist; }
  void SetCalcAsConstRad( SmBoolean bConst )    { m_bCalcAsConstRad = bConst; }
  void SetStatus        ( SmFilStatus eStatus ) { m_eStatus         = eStatus; }

  // predicates
  SmBoolean IsBevelCorner()                    const { return m_bBevel; }
  SmBoolean IsBlendingCorner()                 const { return m_bBlending; }
  SmBoolean IsDegenerateCorner()               const { return m_bDegenerate; }
  SmBoolean IsTopologyAdjusted()               const { return m_bTopologyAdjusted; }
  SmBoolean IsExtendedSurfacePatch()           const { return m_bExtendedSurfacePatch; }
  SmBoolean IsCalcAsConstRad()                 const { return m_bCalcAsConstRad; }
//cbi551 ???  SmBoolean IsEdgeuseSolverEU(SmEdgeuse * pEU) const;
  virtual SmBoolean IsTangentCorner()          const { return FALSE; }

  // utilities
  void RecordFilletError(SmFilletErrorType  eErrorType,
                         SmSurface        * pFilletSurface1,
                         SmSurface        * pFilletSurface2,
                         SmFilletVertex   * pStartFV,
                         SmFilletVertex   * pEndFV,
                         const TCHAR      * cComment = NULL);
#ifdef SM_DEBUG_CODE
  int DebugLevel();
#endif // SM_DEBUG_CODE

  SM_COMMON(SmFilletCorner,SmObject,SmFilletCorner_TYPE);
  virtual void    DumpLevel( int iDebugLevel, const TCHAR * cpMsg = NULL ) const;
  SmDisplayList * Draw(SmGfxArraySet * pOptGfxSet=NULL) const ;

} ; // end class SmFilletCorner

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletCorner*) ;

/*******************************************************************//**
PURPOSE: Create a 1x1 corner. Typically, when only one (closed & non-
    lamina) edge is connected to this corner vertex such that it separates
    two adjacent surfaces, a closed-fillet surface can be defined by rolling
    along this edge. (e.g. when two tori form an '8' shape, only one
    vertex and one edge are defined at the intersection)

NOTES: Will only handle the tangency cases for now.
***********************************************************************/
class SM_EXPORT SmFillet1x1Corner : public SmFilletCorner
{
public:
    SmFillet1x1Corner(const SmContext      & crContext,
                      const SmVertex       * cpVertex,
                      SmTArray<SmEdgeuse*> * pSolverEUs,
                      SmTArray<SmEdgeuse*> * pAllEUs,
                      SmFilletExecutive    * pExec,
                      double                 dApproxTol3d,
                      double                 dTangencyTolRadians);

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_1_x_1; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const {smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFillet1x1Corner")); }

    virtual SmFilletSolver * GetTangentFilletSolver( const SmFilletSolver *pFS );

    virtual SmStatus MakeCornerTopology();

} ; // end class SmFillet1x1Corner

/*******************************************************************//**
PURPOSE: Create a 2x2 corner. When two edges are connected to this
    corner vertex with G1-continuity, a cross section can be defined to
    separate two G1-continuous fillets at this corner

NOTES: Will only handle the tangency cases for now.
***********************************************************************/
class SM_EXPORT
SmFillet2x2Corner : public SmFillet1x1Corner
{
public:
    SmFillet2x2Corner(const SmContext      & crContext,
                      const SmVertex       * cpVertex,
                      SmTArray<SmEdgeuse*> * pSolverEUs,
                      SmTArray<SmEdgeuse*> * pAllEUs,
                      SmFilletExecutive    * pExec,
                      double                 dApproxTol3d,
                      double                 dTangencyTolRadians);

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_2_x_2; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFillet2x2Corner")); }

    virtual SmFilletSolver * GetTangentFilletSolver( const SmFilletSolver *pFS );

    virtual SmStatus MakeCornerTopology();

} ; // end class SmFillet2x2Corner

/*******************************************************************//**
PURPOSE: Create a Nx1 closed corner. Typically, this might happen when we
    have a corner with 2 or 3 edges (such as in cylindrical solids)
    and one 'closed' edge is filleted.

NOTES: Will only handle the tangency cases for now.
***********************************************************************/
class SM_EXPORT SmFilletNx1ClosedCorner : public SmFilletCorner
{
public:
    SmBoolean  m_bG1Case;  // Is the closed edge also G1?

    SmFilletNx1ClosedCorner(const SmContext      & crContext,
                            const SmVertex       * cpVertex,
                            SmTArray<SmEdgeuse*> * pSolverEUs,
                            SmTArray<SmEdgeuse*> * pAllEUs,
                            SmFilletExecutive    * pExec,
                            double                 dApproxTol3d,
                            double                 dTangencyTolRadians);

    virtual SmStatus AdjustTangentCorner();

    virtual SmFilletSolver * GetTangentFilletSolver( const SmFilletSolver *pFS );

    virtual SmBoolean IsTangentCorner() const { return m_bG1Case; }

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_N_x_1_CLOSED; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFilletNx1ClosedCorner")); }

    virtual SmStatus MakeCornerTopology();

} ; // end SmFilletNx1ClosedCorner

/*******************************************************************//**
PURPOSE: Create Nx1 corner by specifying the boundary of the incoming
    fillet. If the filleted edge is a concave edge, an 'end' surface will
    be created to fill the hole of the side face. When a convex edge of
    a concave corner is filleted, a new side surface will also be created

NOTES: Corners with N > 3 are not yet implemented.

***********************************************************************/
class SM_EXPORT SmFilletNx1Corner : public SmFilletCorner
{
public:
    SmFilletNx1Corner(const SmContext      & crContext,
                      const SmVertex       * cpVertex,
                      SmTArray<SmEdgeuse*> * pSolverEUs,
                      SmTArray<SmEdgeuse*> * pAllEUs,
                      SmFilletExecutive    * pExec,
                      double                 dApproxTol3d,
                      double                 dTangencyTolRadians);

    static SmBoolean CheckNx1Degeneracy(const SmVertex * pCornerVert,
                                        SmEdgeuse * pFilEdgeuse);

    virtual SmStatus CreateCornerPatch();

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_N_x_1; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFilletNx1Corner")); }

    virtual SmStatus MakeCornerTopology();

} ; // end class SmFilletNx1Corner

/*******************************************************************//**
PURPOSE: Create an open-end corner (i.e. corner with lamina edge) to
    determine the trimming boundary of fillet at this corner.
    Typically, this will be needed when we do shell-filleting.

NOTES: For now, it will only handle 3x1 case.
***********************************************************************/
class SM_EXPORT SmFilletDegenerateCorner : public SmFilletCorner
{
public:
    SmFilletDegenerateCorner(const SmContext      & crContext,
                             const SmVertex       * cpVertex,
                             SmTArray<SmEdgeuse*> * pSolverEUs,
                             SmTArray<SmEdgeuse*> * pAllEUs,
                             SmFilletExecutive    * pExec,
                             double                 dApproxTol3d,
                             double                 dTangencyTolRadians);

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_DEGENERATE; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFilletDegenerateCorner")); }

    virtual SmStatus MakeCornerTopology();

} ; // end class SmFilletDegenerateCorner

/*******************************************************************//**
PURPOSE: Create an open-end corner (i.e. corner with lamina edge) to
    determine the trimming boundary of fillet at this corner.
    Typically, this will be needed when we do shell-filleting.

NOTES: For now, it will only handle 3x1 case.
***********************************************************************/
class SM_EXPORT
SmFilletOpenCorner : public SmFilletNx1Corner
{
public:
    SmFilletOpenCorner(const SmContext      & crContext,
                       const SmVertex       * cpVertex,
                       SmTArray<SmEdgeuse*> * pSolverEUs,
                       SmTArray<SmEdgeuse*> * pAllEUs,
                       SmFilletExecutive    * pExec,
                       double                 dApproxTol3d,
                       double                 dTangencyTolRadians);

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_OPEN; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFilletOpenCorner")); }

    virtual SmStatus MakeCornerTopology();

} ; // end SmFilletOpenCorner

/*******************************************************************//**
PURPOSE: A 3x2 corner whose two filleted edges has different edge convexity
    such as in L-shaped box case.


NOTES:

***********************************************************************/
class SM_EXPORT SmFillet3x2MixedConvexityCorner : public SmFilletCorner
{
public:
    SmFillet3x2MixedConvexityCorner(const SmContext      & crContext,
                                    const SmVertex       * cpVertex,
                                    SmTArray<SmEdgeuse*> * pSolverEUs,
                                    SmTArray<SmEdgeuse*> * pAllEUs,
                                    SmFilletExecutive    * pExec,
                                    double                 dApproxTol3d,
                                    double                 dTangencyTolRadians);

    virtual SmStatus CreateCornerPatch();

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_3_x_2_MIXED; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFillet3x2MixedConvexityCorner")); }

    virtual SmStatus MakeCornerTopology();

} ; // end class SmFillet3x2MixedConvexityCorner

/*******************************************************************//**
PURPOSE: An Nx2 corner represents a corner with N adjacent
    edges and two of them are filleted. If N = 3, typically, a bevel corner
    (i.e. without corner surface) will be created for convex and
    tangential corners, and a blending corner patch will be generated
    for concave filleted edges. If N = 4, the two filleted edges
    should not share the same face and the corner will consists of only
    one intersection curve between two fillets just as in 3x2 case.


NOTES: Corners with N > 4 are not yet implemented.

***********************************************************************/
class SM_EXPORT SmFilletNx2Corner : public SmFilletCorner
{
public:
    SmBoolean  m_bG1Case;  // Do the two filleted edges meet G1?

    SmFilletNx2Corner(const SmContext      & crContext,
                      const SmVertex       * cpVertex,
                      SmTArray<SmEdgeuse*> * pSolverEUs,
                      SmTArray<SmEdgeuse*> * pAllEUs,
                      SmFilletExecutive    * pExec,
                      double                 dApproxTol3d,
                      double                 dTangencyTolRadians);

    SmStatus AdjustConcaveCornerTopology();

    virtual SmStatus AdjustTangentCorner();

    virtual SmStatus AdjustTopology();

    virtual SmStatus CreateCornerPatch();

    virtual SmFilletSolver * GetTangentFilletSolver( const SmFilletSolver *pFS );

    virtual SmBoolean IsTangentCorner() const { return m_bG1Case; }

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_N_x_2; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFilletNx2Corner")); }

    virtual SmStatus MakeCornerTopology();

    SmStatus Make4x2ConcaveCornerTopology();

} ; // end class SmFilletNx2Corner

/*******************************************************************//**
PURPOSE: A 4x3 corner represents a corner with 4 adjacent
    edges and three of them are filleted. Typically, a blending corner
    patch will be generated.

NOTES:
***********************************************************************/
class SM_EXPORT SmFillet4x3Corner : public SmFilletCorner
{
public:
    SmFillet4x3Corner(const SmContext      & crContext,
                      const SmVertex       * cpVertex,
                      SmTArray<SmEdgeuse*> * pSolverEUs,
                      SmTArray<SmEdgeuse*> * pAllEUs,
                      SmFilletExecutive    * pExec,
                      double                 dApproxTol3d,
                      double                 dTangencyTolRadians);

    virtual SmStatus AdjustTopology() { return SM_SUCCESS; }

//    virtual SmStatus CreateCornerPatch();

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_4_x_3; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFillet4x3Corner")); }

    virtual SmStatus MakeCornerTopology();

} ; // end class SmFillet4x3Corner

/*******************************************************************//**
PURPOSE: An NxN corner is a corner where all incident edges are filleted.
    This default corner surface could be a 3-, 4- or N-sided patch which
    blends smoothly between adjacent fillets

NOTES: This is an abstract class.
***********************************************************************/
class SM_EXPORT SmFilletNxNCorner : public SmFilletCorner
{
public:
    SmFilletNxNCorner(const SmContext      & crContext,
                      const SmVertex       * cpVertex,
                      SmTArray<SmEdgeuse*> * pSolverEUs,
                      SmTArray<SmEdgeuse*> * pAllEUs,
                      SmFilletExecutive    * pExec,
                      double                 dApproxTol3d,
                      double                 dTangencyTolRadians)
   : SmFilletCorner(crContext,cpVertex,pSolverEUs,pAllEUs,pExec,
                    dApproxTol3d,dTangencyTolRadians) {}

    virtual SmStatus AdjustTopology()      { return SM_SUCCESS; }

    virtual SmFilletCornerType GetCornerType() const = 0;

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFilletNxNCorner")); }

    virtual SmStatus MakeCornerTopology() = 0;

} ; // end class SmFilletNxNCorner

/*******************************************************************//**
PURPOSE: A convex NxN corner is an NxN corner where all edges have
    the same convexity.

NOTES: This same-convexity corner is easier than the mixed-convexity
    corner (SmFilletConcaveNxNCorner) because the rails of adjacent fillets
    will intersect each other within the ranges of the edges.

    This default corner surface could be a 3-, 4- or N-sided patch which
    blends smoothly between adjacent fillets
***********************************************************************/
class SM_EXPORT
SmFilletConvexNxNCorner : public SmFilletNxNCorner
{
public:
    SmFilletConvexNxNCorner(const SmContext      & crContext,
                            const SmVertex       * cpVertex,
                            SmTArray<SmEdgeuse*> * pSolverEUs,
                            SmTArray<SmEdgeuse*> * pAllEUs,
                            SmFilletExecutive    * pExec,
                            double                 dApproxTol3d,
                            double                 dTangencyTolRadians);

    virtual SmStatus AdjustTopology();

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_N_x_N_CONVEX; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const {
        smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFilletConvexNxNCorner")); }

    SmBoolean IsChamferCorner() { return m_bChamfer; }

    virtual SmStatus MakeCornerTopology();

    void SetChamfer(SmBoolean bIsChamferCorner) { m_bChamfer = bIsChamferCorner; }

} ; // end class SmFilletConvexNxNCorner

/*******************************************************************//**
PURPOSE: A concave NxN corner is an NxN corner where not all of the
    edges have the same convexity.

NOTES: In this configuration, the two rails in the face that is
    between two same-convexity corners will not intersect with their edges'
    parameter ranges.  They must be extended, and possibly blended.

    This default corner surface could be a 3-, 4- or N-sided patch which
    blends smoothly between adjacent fillets with one extra boundary edge
    which interpolates two rails between the same-convexity edges.
***********************************************************************/
class SM_EXPORT
SmFilletConcaveNxNCorner : public SmFilletConvexNxNCorner
{
public:
    SmFilletConcaveNxNCorner(const SmContext      & crContext,
                             const SmVertex       * cpVertex,
                             SmTArray<SmEdgeuse*> * pSolverEUs,
                             SmTArray<SmEdgeuse*> * pAllEUs,
                             SmFilletExecutive    * pExec,
                             double                 dApproxTol3d,
                             double                 dTangencyTolRadians);

    virtual SmFilletCornerType GetCornerType() const { return SM_FCR_N_x_N_CONCAVE; }

    virtual void GetName(TCHAR * pcMyName, size_t lMyNameAllocLen) const { smos_WStrCpy(pcMyName, lMyNameAllocLen, _T("SmFilletConcaveNxNCorner")); }

    virtual SmStatus MakeCornerTopology();

    SmBoolean TestRailRailInterpolationCurve(SmCurve * pCurve);

} ; // end class SmFilletConcaveNxNCorner


#endif // !__SMFILLETCORNER_H__

