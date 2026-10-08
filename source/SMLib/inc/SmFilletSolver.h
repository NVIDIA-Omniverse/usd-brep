// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFilletSolver.h 
* PURPOSE: Header file for SmFilletSolver object.
**********************************************************************/

#ifndef __SMFILLETSOLVER_H__
#define __SMFILLETSOLVER_H__

#ifndef __SMADVSURFACEINTERSECTOR_H__
#include <SmAdvSurfaceIntersector.h>
#endif 

#ifndef __SMMATRIX_H__
#include <SmMatrix.h>
#endif

#ifndef __SmExtentNd_H__
#include <SmExtentNd.h>
#endif

#ifndef __SMOFFSETSURFACE_H__
#include <SmOffsetSurface.h>
#endif

#ifndef __SMFILLETGEOM_H__
#include <SmFilletGeom.h>
#endif

#ifndef __SMHERMITECURVE_H__
#include <SmHermiteCurve.h>
#endif

#ifndef __SMLOCALSOLVEND_H__
#include <SmLocalSolveNd.h>
#endif

class SmFilletSurfaceGenerator;
class SmLinearCrossSectionFSG;
class SmBlendCurveCrossSectionFSG;
class SmFilletExecutive;
class SmSelfIntersectionHandler;

// Type of different solvers
enum SmFilletSolverType 
{
  SM_FS_UNKNOWN,
  SM_FS_SURF_SURF,
  SM_FS_CONST_RADIUS,
  SM_FS_VARIABLE_RADIUS,
  SM_FS_CONST_DIST,
  SM_FS_CURVE_BASED,
  SM_FS_CONST_RADIUS_ASSISTED
};

// Type of different fillet surface generators
enum SmFilletSurfaceGeneratorType 
{
  SM_FSG_UNKNOWN,
  SM_FSG_LINEAR,
  SM_FSG_CIRCULAR,
  SM_FSG_BLEND_CURVE
};

/*******************************************************************//**
PURPOSE: Class contains original solver-specific data before roll over

NOTES: 
***********************************************************************/
class SmFilletSolverOffsetsData
{
public:
  SmOffsetSurface * m_pSurfaces[2] ;     //Original offset surfaces
  double            m_dOrientations[2] ; //Orientations in which to offset
                                         // either -1.0 or 1.0 values.

  // constructor
  SmFilletSolverOffsetsData() { m_pSurfaces[0]     = m_pSurfaces[1]     = NULL ;
                                m_dOrientations[0] = m_dOrientations[1] = 1.0 ;
                              }

} ; // end class SmFilletSolverOffsetsData

/*******************************************************************//**
PURPOSE: The Fillet Solver object is the abstract object which 
     presents a consistent interface to filleting operations to the
     high level fillet framework.

NOTES: 
***********************************************************************/
class SM_EXPORT SmFilletSolver : public SmObject                    
{
  friend class SmFilletSurfaceGenerator;
  friend class SmFilletIntersector;
  friend class SmFilletCorner;
  friend class SmFilletExecutive;
  friend class SmFilletGeom;

protected:
  SmEdgeuse         * m_pEdgeuses[2];            // Two radial edgesuses corresponding to sector to be filleted.
                                                 // Note: Rail 0 is on surface of the first edgeuse.
                                                 //       Rail 1 is on surface of the 2nd edgeuse.
  SmFilletExecutive * m_pExecutive;              // Executive that controls global filleting operations
  const SmContext   & m_crContext;               // Context for all temporary and permanent object
                                                 // allocation.
  double              m_dThisApproxTol3d;        // Tolerance used to determine how close fillet
                                                 // edges approximate the surfaces on which they lie.
  double              m_dThisAngTolRad;          // Angle tolerance for stepping in radians
  double              m_dTangencyTolerance;      // Angle tolerance for surface tangency in radians
  double              m_dEdgeBlendFactor;        // Blending factor for edges
  SmOffsetSurface   * m_pSurfaces[2];            // offset surfaces of the two surfaces connected to the filletEdge
  double              m_dOrientations[2];        // Current orientations in which to offset, either -1.0 or 1.0 values.
  SmFilletGeom      * m_pFGForSurfaces;          // If not Null, use the offset surfaces stored
                                                 // in this FG, if present.
  double              m_dSavedOffsets[2];        // Temporary place to save offsets of each surface
                                                 //  used when offset is temporarily set to 0.0.
  double              m_dSurfaceExtensionFactor; // Scale factor for surface UV domains, and curve 
                                                 // intervals during extension.
                                                 // An extension factor of 1.0 will add 1 length to each end tripling the size.
                                                 // An extension factor of 0.5 will add .5 length to each end doubling the size.
  double              m_dConversionTol;          // Tolerance indicating when conversion happens - represents
                                                 // a given number of decimal places.
  SmBoolean           m_bReverseTrim;            // If TRUE then we will take keep the opposite side
                                                 // of the surface or solid.
  SmBoolean           m_bExtendBefore;           // If TRUE then we will allow the extension of the
                                                 // fillet before the start.
  SmBoolean           m_bExtendAfter;            // If TRUE then we will allow the extension of the
                                                 // fillet after the end.
  SmFilletSurfaceGenerator * m_pFSG;             // Fillet surface generator to contain method CreateSurface().
  SmTArray<SmFilletGeom*>    m_vFilletGeoms;     // Fillet geometries ordered in the order in which they
                                                 // occur as walking along the fillet edge (more or less).
  SmFilStatus         m_eStatus;                 // Status of solver computation
  SmTArray<double>    m_vG1Knots;                // Internal knots of the edge-curve that are only G1-continuous

  SmStatus FinishConstruction();                 // Private method for base class, called by
                                                 // derived classes at end of constructors.

//    virtual SmStatus Copy(const SmContext & crContext,
//                          SmFilletSolver *& rpNewFS) const;

public:
  // constructor
  SmFilletSolver(const SmContext & crContext,
                 double            dApproxTol3d,
                 double            dAngTolRad,
                 double            dTangencyTolerance);
  
  // destructor
  virtual ~SmFilletSolver();

  // simple access
  virtual SmFilletSolverType  GetSolverType() = 0;
  virtual void                GetName                    (TCHAR * pcMyName, size_t lMyNameAllocLen)=0 ;
  SmFilStatus                 GetStatus                  ()                 const { return m_eStatus; }
  SmBoolean                   GetReverseTrim             ()                       { return m_bReverseTrim; }

  const SmContext           & GetCreationContext         ()                       { return m_crContext; }
  SmFilletExecutive         * GetFilletExecutive         () const                 { return m_pExecutive; }
  SmFilletSurfaceGenerator  * GetFilletSurfaceGenerator  ()                       { return m_pFSG; }
  SmSelfIntersectionHandler * GetSelfIntersectionHandler ()                 const ; 
  virtual void                GetOffsetsData             (SmFilletSolverOffsetsData & rOffsetsData);
                            
  double                      GetThisApproxTol3d         ()                 const { SM_ASSERT_TOL(m_dThisApproxTol3d) ; return m_dThisApproxTol3d; }
  double                      GetThisAngTolRad           ()                 const { SM_ASSERT_TOL(m_dThisAngTolRad) ; return m_dThisAngTolRad; }   // radians
  double                      GetConversionTol           ()                 const { return m_dConversionTol; }
  double                      GetTangencyTolerance       ()                 const { return m_dTangencyTolerance; }

  SmVertex                  * GetVertex                  (ULONG lIndex)     const ;
  SmEdgeuse                 * GetEdgeuse                 (ULONG lIndex)     const { return m_pEdgeuses[lIndex]; }
  SmFilletCorner            * GetFilletCorner            (ULONG lIndex)     const ;
  SmFilletCorner            * GetOtherFilletCorner       (const SmFilletCorner * ) const ;
  void                        GetFilletGeoms             (SmTArray<SmFilletGeom*> & rFilletGeoms);
  SmFilletGeom              * GetFilletGeomForSurfaces   ()                       { return m_pFGForSurfaces; }
  SmFilletGeom              * GetFirstFilletGeom         ()                       { return m_vFilletGeoms[0]; }
  SmFilletGeom              * GetLastFilletGeom          ()                       { return m_vFilletGeoms.GetLast(); }
                            
  ULONG                       FindIndexOfRailOnFace      (SmFace          * pOrigFace) ;
  ULONG                       FindIndexOfRailXSideEdgeuse(SmEdgeuse       * pSideEU) ;
  SmOffsetSurface           * GetSurface                 (ULONG             lRailIndex) const ;
  double                      GetOrientation             (ULONG             lRailIndex) { return m_dOrientations[lRailIndex]; }
  double                      GetExtensionFactor         () { return m_dSurfaceExtensionFactor; }
 
  ULONG                       FindFilGeomIndex           ( const SmFilletGeom *pFilGeom ) const;
  ULONG                       FindIndexInExec            () const;
                            
  virtual ULONG               GetJacobianSize() const { SE_MSG(SM_ERR,_T("Called pure virtual method")); return 0 ; }

  // simple assignment
  void             SetStatus                (SmFilStatus eStatus)                                { m_eStatus = eStatus; }
  void             SetReverseTrim           (SmBoolean bReverseTrim)                             { m_bReverseTrim = bReverseTrim; }
  virtual void     SetExtensionFlags        () ;
  virtual void     SetExtensionFactor       (double dExtFactor)                                  { m_dSurfaceExtensionFactor = dExtFactor; }
  virtual void     SetFilletExecutive       (SmFilletExecutive * pFilExecutive)                  { m_pExecutive = pFilExecutive; }
  void             SetFilletSurfaceGenerator(SmFilletSurfaceGenerator *pFSG)                     { m_pFSG = pFSG; } // sets fillet cross-section shapes 
  void             SetFilletGeomForSurfaces (SmFilletGeom *pFG )                                 { m_pFGForSurfaces = pFG; }
  void             SetSurface               (ULONG lRailIndex, SmOffsetSurface *pOffsetSurface ) ;

  virtual void     SetSurfacesToZeroOffset  () ;
  virtual SmStatus SetupOffsetValues        (const SmTsectPnt & rTsectPnt, double * pOffsetDist=NULL) { SM_REF2(rTsectPnt,pOffsetDist) ; SE_MSG(SM_ERR,_T("Called pure virtual method")); return SM_ERR ;}
  virtual void     SetOffsetsData           (SmFilletSolverOffsetsData & rOffsetsData);
  virtual SmStatus SetOffSurfsToFilletGeom  (SmFilletGeom *pFG ) ;

  // predicates                  
  virtual SmBoolean CanMakeAnalytic()      { return ! OffsetRadiiCanChange(); } // default behavior.
  virtual SmBoolean OffsetRadiiCanChange() { return TRUE; }

  // side effects
  virtual SmStatus  CalcFilletGeom           (); // after SmFilletCorner:: Builds topology, Compute filletSurface and railCurve geometry
  virtual SmStatus  CalcCylinderFilletGeom   (SmTArray<SmTsectPnt*> & rTsectPnts, SmFilletGeom * pFG = NULL);
  virtual SmStatus  CalcConeFilletGeom       (SmTArray<SmTsectPnt*> & rTsectPnts, SmFilletGeom * pFG = NULL);
  virtual SmStatus  CalcTorusFilletGeom      (SmTArray<SmTsectPnt*> & rTsectPnts, SmFilletGeom * pFG = NULL);

  SmStatus          ComputeSurfaceValues     (SmPoint2d               aUVValues[2], SmTsectPnt  & rTsectPnt);
  SmStatus          ComputePointValues       (SmPoint2d          aUVValues[2],
                                              const SmExtent2d & crUVDomain1,
                                              const SmExtent2d & crUVDomain2,
                                              double             dCurveTraceDirection,
                                              SmTsectPnt       & rTsectPnt,
                                              SmTsectPnt       * pOptPreviousPnt);
  SmStatus          ComputeUVVectors         (SmTsectPnt       & rTsectPnt);

  SmStatus          CreateCliffRollOverSideFG(SmFilletEdge     * pAdjRail,
                                              SmEdge           * pSideEdge,
                                              SmFace           * pAdjSideFace,
                                              SmBoolean          bTrimSideFaces,
                                              double             dExtensionDist,
                                              SmFilletGeom    *& rpSideFG);
  
  // include slightly different FilletSolver function sets in a variety of derived SmEvalNFunctionsObjects
  virtual SmStatus LoadJacobian(ULONG &                  /* in : rlNumEquations  */, 
                                ULONG &                  /* in : rlNumParameters */,
                                ULONG                    /* in : lRailIndex      */,
                                SmSurface *&             /* in : rpSurface1      */,
                                ULONG &                  /* in : rlSurf1Offset   */,
                                SmSurface *&             /* in : rpSurface2      */,
                                ULONG &                  /* in : rlSurf2Offset   */,
                                const SmTArray<double> & /* in : crX             */, 
                                SmTArray<double> &       /* out: rF              */, 
                                SmMatrix *               /* out: pOptJacobian    */,
                                SmBoolean &              /* out: rbFoundAnswer   */) { SE(SM_ERR); return SM_ERR; }

  // Note: first argument is ULONG & for historical reasons.
  // It could be changed to SmBoolean.
  virtual SmStatus LoadInitialValues(ULONG &,                 // in : rbDoSurf2Calcs 
                                     ULONG,                   // in : lRailIndex     
                                     const SmSurface &,       // in : crSurface1     
                                     ULONG,                   // in : lSurface1Index 
                                     SmSurface *&,            // in : pSurface2      
                                     ULONG &,                 // in : lSurface2Index 
                                     SmExtentNd &,            // in : rExtents       
                                     SmTArray<SmBoolean> &,   // in : rPeriodicities 
                                     SmTArray<double> &)      // in : rGuessT        
                                   { SE(SM_ERR); return SM_ERR; }

  // given point on and plane perp to offset-surface xsect curve - find pair of rail-curve points
  virtual SmStatus PointOnPlaneSolve(const SmPoint3d  & crPlaneOrig,
                                     const SmVector3d & crPlaneNormal,
                                     const SmExtent2d & crUVDomain1,
                                     const SmExtent2d & crUVDomain2,
                                     const SmVector2d & rUV1,
                                     const SmVector2d & rUV2,
                                     SmBoolean        & rbFoundSolution,
                                     SmTsectPnt       & rTsectPnt);

  SmStatus ProcessFullRollOver(SmFilletGeom * pFilletGeom, ULONG lRollOverRailIndex);
  SmStatus ProcessRollOver    (SmFilletGeom * pFilletGeom, SmTArray<SmFilletCorner*> & rAdjustedCorners);

  SmStatus RailEdgeuseIntersect(ULONG        lRailIndex,
                                SmEdgeuse  * pEdgeuse,
                                double       dGuessEdgeParameter,
                                SmBoolean  & rbFoundIntersection,
                                SmTsectPnt & rTsectPnt,
                                double     & rdEdgeuseParameter);

  SmStatus RailRailIntersect(const SmPoint2d & crUVGuess,
                             ULONG             lRailIndex,
                             SmFilletSolver  * pOtherFillet,
                             ULONG             lOtherRailIndex,
                             SmBoolean       & rbFoundIntersection,
                             SmTsectPnt      & rThisTsectPnt,
                             SmTsectPnt      & rOtherTsectPnt);

  virtual void ReloadSurfaceOffsets();
  virtual void SetupOffsetExtension(const SmSurface * pSurf, SmExtent2d & rDomain) ;
  SmStatus     SplitAndTrimFilletSurface(ULONG lFilletGeomIndex,
                                         SmBoundaryTrimmingType eTrimType);
               
  // Split a topological fillet geom at a side-edge / rail intersection.
  SmStatus     SplitFilletGeomAtSideEdge (SmFilletGeom      *  pFilletGeom,
                                          SmFilletVertexuse *  pSplitFilVU,
                                          ULONG                lWhichEnd,
                                          SmFilletGeom      * *ppRetFilletGeom = NULL);
               
  // Split a surface-surface fillet geom into separate segments.
  SmStatus     SplitFilletGeom(ULONG                         lFilletGeomIndex,
                               SmBoundaryTrimmingType        eTrimType,
                               const SmTArray<SmExtent1d>  & crIvls1,
                               const SmTArray<SmExtent1d>  & crIvls2,
                               const SmCurveClassification & crCC1,
                               const SmCurveClassification & crCC2);
                                       
               
  SmStatus     StepSolve(const SmTsectPnt & crCurrentPoint,
                         double           & rdStep,
                         const SmExtent2d & crUVDomain1,
                         const SmExtent2d & crUVDomain2,
                         SmBoolean        & rbFoundGoodPoint,
                         SmBoolean        & rbClipped,
                         SmBoolean        & rbBoundaryHit,
                         SmTsectPnt       & rNextPoint);
               
  SmStatus     TestRollOver(SmTArray<SmFilletCorner*> & rAdjustedCorners);
  SmStatus     TrimRails();

  // utilities
  void         RecordFilletError(SmFilletErrorType       eType,
                                 SmTArray<SmTsectPnt*> & rapTSPs,
                                 const TCHAR           * cComment);
               
  SM_COMMON(SmFilletSolver,SmObject,SmFilletSolver_TYPE);
  void         DumpLevel( int iDebugLevel, const TCHAR * cpMsg = NULL ) const;
  virtual void Draw();

#ifdef SM_DEBUG_CODE
  int DebugLevel();
#endif // SM_DEBUG_CODE

} ; // end class SmFilletSolver

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletSolver*) ;

/*******************************************************************//**
PURPOSE: This object generates a surface of a corresponding type
    given a centerline curve, rail curves, and TsectPnts.

NOTES: 
***********************************************************************/
class SM_EXPORT SmFilletSurfaceGenerator : public SmObject
{
public:
  // constructor
  SmFilletSurfaceGenerator() {}

  // destructor
  virtual ~SmFilletSurfaceGenerator() {}

  virtual SmStatus CreateSurface(SmFilletSolver              & /* rFilletSolver  */,
                                 SmBSplineCurve              * /* pCenterLine    */,
                                 SmBSplineCurve              * /* pRail1Curve    */,
                                 SmBSplineCurve              * /* pRail2Curve    */,
                                 const SmTArray<SmTsectPnt*> & /* crFilletPoints */,
                                 SmSurface                   * /* pSurf1         */,
                                 SmBSplineCurve              * /* pUV1           */,
                                 SmSurface                   * /* pSurf2         */,
                                 SmBSplineCurve              * /* pUV2           */,
                                 SmBSplineSurface           *& /*rpFilletSurface */)
  { SE(SM_ERR); return SM_ERR; }

  virtual SmFilletSurfaceGeneratorType GetFilletSurfaceGeneratorType()
  { return SM_FSG_UNKNOWN; }

} ; // end class SmFilletSurfaceGenerator

/*******************************************************************//**
PURPOSE: Create a fillet surface with a linear cross section.  This
    is simply a ruled surface or in some special cases a plane or a cone.

NOTES: 
***********************************************************************/
class SM_EXPORT SmLinearCrossSectionFSG : public SmFilletSurfaceGenerator
{
public:
  SmLinearCrossSectionFSG() {}

  virtual ~SmLinearCrossSectionFSG() {}

  virtual SmStatus CreateSurface(SmFilletSolver              & rFilletSolver,     // in : 
                                 SmBSplineCurve              * pCenterLine,       // NotUsed: in :
                                 SmBSplineCurve              * pRail1Curve,       // in : 
                                 SmBSplineCurve              * pRail2Curve,       // in : 
                                 const SmTArray<SmTsectPnt*> & crFilletPoints,    // NotUsed: in :
                                 SmSurface                   * pSurf1,            // NotUsed: in :
                                 SmBSplineCurve              * pUV1,              // NotUsed: in :
                                 SmSurface                   * pSurf2,            // NotUsed: in :
                                 SmBSplineCurve              * pUV2,              // NotUsed: in :
                                 SmBSplineSurface           *& rpFilletSurface) ; // in : 

  virtual SmFilletSurfaceGeneratorType GetFilletSurfaceGeneratorType()
  { return SM_FSG_LINEAR; }

} ; // end class SmLinearCrossSectionFSG

/*******************************************************************//**
PURPOSE: Create a fillet surface with a circular cross section.  This
    is a lofted surface or in some special cases a cylinder or a torus.

NOTES: 
***********************************************************************/
class SM_EXPORT SmCircularCrossSectionFSG : public SmFilletSurfaceGenerator
{
protected:
  SmBoolean   m_bApproximateArcs;     // TRUE = interpolate rail curves with as circular arcs as possible
                                      // FALSE= circular cross-sections even when rail curves are
                                      //        not aligned on a circle centered on the centerCurve
                                      // default:[TRUE]
                                               
  double      m_dTolPercentRadius;    // max allowed percentage deviation of output curve from circular arc
                                      // used when m_bApproximateArcs == TRUE
                                      // default:[0.05] 
                                            
  SmBoolean   m_bMirror;              // TRUE = CrossSections are mirrored about RailCurve Chord
                                      // default:[FALSE] 
                                          
  SmBoolean   m_bComplement;          // TRUE = Fillet Surface made with larger half of circular arcs
                                      // FALSE= Fillet Surface made with smaller half of circular arcs     
                                      // default:[FALSE]
public:
  SmCircularCrossSectionFSG(SmBoolean bApproximateArcs=TRUE,
                            double    dTolPercentRadius=0.05,
                            SmBoolean bMirror=FALSE,
                            SmBoolean bComplement=FALSE)      : m_bApproximateArcs(bApproximateArcs), 
                                                                m_dTolPercentRadius(dTolPercentRadius), 
                                                                m_bMirror(bMirror),
                                                                m_bComplement(bComplement)
                                                              { }

  virtual ~SmCircularCrossSectionFSG() {}

  double    GetTolPercentRadius() { return m_dTolPercentRadius; }
  SmBoolean GetApproximateArcs()  { return m_bApproximateArcs; }

  virtual SmStatus CreateSurface(SmFilletSolver   & rFilletSolver,
                                 SmBSplineCurve   * pCenterLine,
                                 SmBSplineCurve   * pRail1Curve,
                                 SmBSplineCurve   * pRail2Curve,
                                 const SmTArray<SmTsectPnt*> & crFilletPoints,
                                 SmSurface         * pSurf1,  
                                 SmBSplineCurve    * pUV1,
                                 SmSurface         * pSurf2,
                                 SmBSplineCurve    * pUV2,
                                 SmBSplineSurface *& rpFilletSurface);

  virtual SmStatus CreateSurfaceFromCurves(const SmContext   & crContext,
                                           SmBSplineCurve    & rCenterLine,
                                           SmBSplineCurve    & rRail1Curve,
                                           SmBSplineCurve    & rRail2Curve,
                                           double              dThisApproxTol3d,
                                           SmBoolean           bNeedG1Continuity,
                                           SmSurface         * pSurf1,  
                                           SmBSplineCurve    * pUV1,
                                           SmSurface         * pSurf2,
                                           SmBSplineCurve    * pUV2,
                                           SmBSplineSurface *& rpFilletSurface);

  virtual SmFilletSurfaceGeneratorType GetFilletSurfaceGeneratorType()
  { return SM_FSG_CIRCULAR; }

} ; // end class SmCircularCrossSectionFSG

/*******************************************************************//**
PURPOSE: Create a fillet surface with a circular cross section.  This
    is a lofted surface or in some special cases a cylinder or a torus.

NOTES: 
***********************************************************************/
class SM_EXPORT SmBlendCurveCrossSectionFSG : public SmFilletSurfaceGenerator
{
protected:
  ULONG  m_lContinuity; // 1, 2, or 3 for G1, G2 or G3
  double m_dBlendScale;
public:
  SmBlendCurveCrossSectionFSG(double dBlendScale=1.5, ULONG lContinuity=1)
     : m_lContinuity(lContinuity), 
       m_dBlendScale(dBlendScale)
  {}

  virtual ~SmBlendCurveCrossSectionFSG() {}

  virtual SmStatus CreateSurface(SmFilletSolver    & rFilletSolver,
                                 SmBSplineCurve    * pCenterLine,
                                 SmBSplineCurve    * pRail1Curve,
                                 SmBSplineCurve    * pRail2Curve,
                                 const SmTArray<SmTsectPnt*> & crFilletPoints,
                                 SmSurface         * pSurf1,  
                                 SmBSplineCurve    * pUV1,
                                 SmSurface         * pSurf2,
                                 SmBSplineCurve    * pUV2,
                                 SmBSplineSurface *& rpFilletSurface);

  virtual SmStatus CreateSurfaceFromCurves(const SmContext   & crContext,
                                           SmBSplineCurve    & rCenterLine,
                                           SmBSplineCurve    & rRail1Curve,
                                           SmBSplineCurve    & rRail2Curve,
                                           double              dThisApproxTol3d,
                                           SmBoolean           bNeedG1Continuity,
                                           SmSurface         * pSurf1,  
                                           SmBSplineCurve    * pUV1,
                                           SmSurface         * pSurf2,
                                           SmBSplineCurve    * pUV2,
                                           SmBSplineSurface *& rpFilletSurface);

  double GetBlendScale() { return m_dBlendScale; }

  ULONG GetContinuity() { return m_lContinuity; }

  virtual SmFilletSurfaceGeneratorType GetFilletSurfaceGeneratorType()
  { return SM_FSG_BLEND_CURVE; }

} ; // end class SmBlendCurveCrossSectionFSG

/*******************************************************************//**
PURPOSE: Describes the type of operation to perform when self-
    intersections were detected for fillet creations.

NOTES: 
***********************************************************************/
enum SmHandlingAlgorithmType
{
    SM_HA_UNKNOWN,  // Unknown type of algorithm
    SM_HA_SMOOTHING,// Smooth out self intersections of fillets
    SM_HA_BLENDING  // Insert a blending patch for self intersections
};

/*******************************************************************//**
PURPOSE: This is the default handler for self-intersecting fillet
    surfaces.  By default we do nothing but error return.

NOTES: 
***********************************************************************/
class SM_EXPORT SmSelfIntersectionHandler : public SmObject
{
public:
  SmSelfIntersectionHandler() {}

  virtual ~SmSelfIntersectionHandler() {}

  // not completely implemented - currently always returns bSelfIntersecting = FALSE
  virtual SmStatus IsSelfIntersecting
            (const SmContext        & crContext,           // in : context for temporary geometry
             const SmBSplineSurface * cpFilletSurface,     // in : fillet surface to examine, NULL to ignore
             double                   dThisApproxTol3d,    // in : tol used to build pFilletSurface, pFilletGeom->GetFilletSolver()->GetThisApproxTol3d()
             const SmBSplineCurve   * cpCenterLine,        // in : centerline used to define fillet surface 
             const SmBSplineCurve   * cpRailCurve1,        // in : min constant_v value cpFilletSurface iso-parameter line
             const SmBSplineCurve   * cpRailCurve2,        // in : max constant_v value cpFilletSurface iso-parameter line
             double                   dFilletRadius,       // in : radius used to define fillet surface
             SmBoolean              & bSelfIntersecting,   // out: TRUE=is self-intersecting, FALSE = not
             SmPoint2d              & rStartPoint,         // NotUsed: out: start of surface self-intersecting region 
             SmPoint2d              & rEndPoint) ;         // NotUsed: out: end   of surface self-intersecting

  // trim out and blend over a self intersecting portion of a fillet surface
  //   input: prev section in pOrigFilletGeom, 
  //          next section in input arguments pNewFilletSurface, pNewFillet, ...
  //   output: pOrigFilletGeom set with geometry for prev section trimmed back to OK section
  //           pBlendFilletGeom set with geometry for blend section covering problem trimmed out section
  //           pNewFilletGeom set with geometry for prev section trimmed back to OK section 
  // same algorithm as SmoothOutIntersection - but sets outputs into geometry of 3 different fillet geoms and updates fillet surface
  virtual SmStatus CreateBlends
            (SmFilletGeom     * pOrigFilletGeom,           // i/o: orig FilletGeom - now beg of output FilletSequence                  
             SmFilletGeom     * pBlendFilletGeom,          // i/o: FilletGeom after 1 TopologySplit() - mid of output FilletSequence   
             SmFilletGeom     * pNewFilletGeom,            // i/o: FilletGeom after 2 topologySplits() - end of output FilletSequence  
             SmBSplineSurface * pNewFilletSurface,         // in : Fillet surface with potential self-intersections                    
             SmBSplineCurve   * pCenterLine,               // in : Fillet surface center line                                          
             SmBSplineCurve   * pRailCurve1,               // in : 1st 3d rail curve                                                   
             SmBSplineCurve   * pRailCurve2,               // in : 2nd 3d rail curve                                                   
             SmBSplineCurve   * pUVCurve1,                 // in : 1st UV rail curve                                                   
             SmBSplineCurve   * pUVCurve2) = 0;            // in : 2nd UV rail curve                                                   

  // trim out and blend over a self intersecting portion of a fillet surface
  //   input: prev section in pFilletGeom, 
  //          next section in input arguments
  //   output: pFilletGeom curve geometry set with joined prev, blend, and next sections
  // same algorithm as CreateBlends - but sets outputs into geometry of 1 fillet geom and does not update fillet surface
  virtual SmStatus SmoothOutIntersection
            (SmFilletGeom * pFilletGeom,                   // i/o: in: fillet prev section, out: fillet joined section
             SmTArray<SmBSplineCurve*> & r3DCurves,        // i/o: [0] = next centerline,   out: joined centerline
                                                           //      [1] = next Rail3d Surf0  out: joined Rail3d Surf0
                                                           //      [2] = next Rail3d Surf1  out: joined Rail3d Surf1
             SmTArray<SmBSplineCurve*> & rUVCurves1,       // i/o: [0] = next RailUV Surf0  out: joined RailUV Surf0
             SmTArray<SmBSplineCurve*> & rUVCurves2) = 0;  // i/o: [0] = next RailUV Surf0  out: joined RailUV Surf0

  virtual SmHandlingAlgorithmType GetAlgorithmType() = 0;

} ; // end class SmSelfIntersectionHandler

/*******************************************************************//**
PURPOSE: This is the fillet surface self intersection handler that
    will do a little step back and then just make a blend between the
    curves in the area of a self intersection.  

NOTES: Note that the resulting surface in the area of the blend
    will not have a circular cross seciton.  It will be a blend of some
    sort if you are doing a circular cross section fillet.
***********************************************************************/
class SM_EXPORT SmMakeCurveBlendSIH : public SmSelfIntersectionHandler
{
protected:
  double m_dStepBackDistFactor;   // Factor of fillet radius to determine
  // how far to step back from intersection before making blends between
  // curves
public:
  SmMakeCurveBlendSIH(double dStepBackDistFactor)
      : m_dStepBackDistFactor(dStepBackDistFactor) {}

  virtual ~SmMakeCurveBlendSIH() {}

  // dummy virtual function
  virtual SmStatus CreateBlends(SmFilletGeom *,      // i/o: orig FilletGeom - now beg of output FilletSequence              
                                SmFilletGeom *,      // i/o: FilletGeom after 1 TopologySplit() - mid of output FilletSequence 
                                SmFilletGeom *,      // i/o: FilletGeom after 2 topologySplits() - end of output FilletSequence
                                SmBSplineSurface *,  // in : Fillet surface with potential self-intersections                
                                SmBSplineCurve *,    // in : Fillet surface center line                                      
                                SmBSplineCurve *,    // in : 1st 3d rail curve                                               
                                SmBSplineCurve *,    // in : 2nd 3d rail curve                                               
                                SmBSplineCurve *,    // in : 1st UV rail curve                                               
                                SmBSplineCurve *)    // in : 2nd UV rail curve                                               
          { return SM_SUCCESS; }                 
                                                   
  // trim out and blend over a self intersecting portion of a fillet surface
  //   input: prev section in pFilletGeom, 
  //          next section in input arguments
  //   output: pFilletGeom curve geometry set with joined prev, blend, and next sections
  // same algorithm as CreateBlends - but sets outputs into geometry of 1 fillet geom and does not update fillet surface
  virtual SmStatus SmoothOutIntersection
           (SmFilletGeom * pFilletGeom,              // i/o: in: fillet prev section, out: fillet joined section
                                                     // i/o: [0] = next centerline,   out: joined centerline    
                                                     //      [1] = next Rail3d Surf0  out: joined Rail3d Surf0  
            SmTArray<SmBSplineCurve*> & r3DCurves,   //      [2] = next Rail3d Surf1  out: joined Rail3d Surf1  
            SmTArray<SmBSplineCurve*> & rUVCurves1,  // i/o: [0] = next RailUV Surf0  out: joined RailUV Surf0  
            SmTArray<SmBSplineCurve*> & rUVCurves2); // i/o: [0] = next RailUV Surf0  out: joined RailUV Surf0  

  virtual SmHandlingAlgorithmType GetAlgorithmType() { return SM_HA_SMOOTHING; }

} ; // end class SmMakeCurveBlendSIH

/*******************************************************************//**
PURPOSE: This is the fillet surface self intersection handler that
    will do a step back and make a blending surface in the area of a
    self intersection.  

NOTES: Note that the resulting surface in the area of the blend
    will not have a circular cross section.  It will be a blend of some
    sort if you are doing a circular cross section fillet.
***********************************************************************/
class SM_EXPORT SmMakeSurfaceBlendSIH : public SmSelfIntersectionHandler
{
protected:
  double m_dStepBackDistFactor;   // Factor of fillet radius to determine
  // how far to step back from intersection before making blends between
  // fillet surfaces.  
public:
  SmMakeSurfaceBlendSIH(double dStepBackDistFactor)
      : m_dStepBackDistFactor(dStepBackDistFactor) {}

  virtual ~SmMakeSurfaceBlendSIH() {}

  // trim out and blend over a self intersecting portion of a fillet surface
  //   input: prev section in pOrigFilletGeom, 
  //          next section in input arguments pNewFilletSurface, pNewFilletSurface, ...
  //   output: pOrigFilletGeom set with geometry for prev section trimmed back to OK section
  //           pBlendFilletGeom set with geometry for blend section covering problem trimmed out section
  //           pNewFilletGeom set with geometry for prev section trimmed back to OK section 
  // same algorithm as SmoothOutIntersection - but sets outputs into geometry of 3 different fillet geoms and updates fillet surface
  virtual SmStatus CreateBlends
            (SmFilletGeom     * pOrigFilletGeom,     // i/o: orig FilletGeom - now beg of output FilletSequence                
             SmFilletGeom     * pBlendFilletGeom,    // i/o: FilletGeom after 1 TopologySplit() - mid of output FilletSequence 
             SmFilletGeom     * pNewFilletGeom,      // i/o: FilletGeom after 2 topologySplits() - end of output FilletSequence
             
             SmBSplineSurface * pNewFilletSurface,   // in : Fillet surface with potential self-intersections                  
             SmBSplineCurve   * pCenterLine,         // in : Fillet surface center line                                        

             SmBSplineCurve   * pRailCurve1,         // in : 1st 3d rail curve                                                 
             SmBSplineCurve   * pRailCurve2,         // in : 2nd 3d rail curve                                                 
             SmBSplineCurve   * pUVCurve1,           // in : 1st UV rail curve                                                 
             SmBSplineCurve   * pUVCurve2);          // in : 2nd UV rail curve                                                 

  // dummy virtual function
  virtual SmStatus SmoothOutIntersection
           (SmFilletGeom              * pFilletGeom, // i/o: in: fillet prev section, out: fillet joined section
                                                     // i/o: [0] = next centerline,   out: joined centerline    
                                                     //      [1] = next Rail3d Surf0  out: joined Rail3d Surf0  
            SmTArray<SmBSplineCurve*> & r3DCurves,   //      [2] = next Rail3d Surf1  out: joined Rail3d Surf1  
            SmTArray<SmBSplineCurve*> & rUVCurves1,  // i/o: [0] = next RailUV Surf0  out: joined RailUV Surf0  
            SmTArray<SmBSplineCurve*> & rUVCurves2)  // i/o: [0] = next RailUV Surf0  out: joined RailUV Surf0  
          { SM_REF4(pFilletGeom, r3DCurves, rUVCurves1, rUVCurves2) ; return SM_SUCCESS; }

  virtual SmHandlingAlgorithmType GetAlgorithmType()  { return SM_HA_BLENDING; }

}; // end class SmMakeSurfaceBlendSIH

/*******************************************************************//**
PURPOSE: Compute the fillet values at given distance from existing point

NOTES: 
***********************************************************************/
class SmFilletSphereSolveENFO : public SmEvalNFunctionsObject
{
protected:
  SmPoint3d              m_vSphereCenter;   // obsolete approach - center of intersection sphere
  double                 m_dSphereRadius;   // obsolete approach - radius of intersection sphere
  SmPoint3d              m_vPlaneABC;       // new approach - intersection plane normal direction
  double                 m_dPlaneD;         // new approach - plane distance from origin
  SmBoolean              m_bPlaneAdjusts;   // TRUE = Recompute Plane from last solve values
  SmFilletSolver       & m_crFilletSolver;  // Container for surfaces and edge being filleted
public:

  // constructor, destructor
  SmFilletSphereSolveENFO(const SmPoint3d & crSphereCenter,
                          double dSphereRadius,
                          SmFilletSolver & crFilletSolver)
    : m_vSphereCenter(crSphereCenter), 
      m_dSphereRadius(dSphereRadius),
      m_vPlaneABC(crSphereCenter), 
      m_dPlaneD(dSphereRadius),
      m_bPlaneAdjusts(FALSE),
      m_crFilletSolver(crFilletSolver) 
      { }
  virtual ~SmFilletSphereSolveENFO() {}

  //
  void SetPlaneAdjusts(SmBoolean bPlaneAdjusts) { m_bPlaneAdjusts = bPlaneAdjusts; }
  virtual SmStatus Evaluate(const SmTArray<double> & crX,             // in : x of Ax=F     
                            SmTArray<double>       & rF,              // out: F of Ax=F function values,                     
                            SmMatrix               * pOptJacobian,    // out: Partial derivatives of the functions.
                            SmBoolean              & rbFoundAnswer);  // out: Not always used, when used
                                                                      //      TRUE = converged (F members are within tolerance of 0.0
                                                                      //      FALSE= Not Used or Not Converged
} ; // end SmFilletSphereSolveENFO

/*******************************************************************//**
PURPOSE: Find intersection of fillet rail and UV curve on surface.

NOTES:
***********************************************************************/
class SmRailUVCurveIntersectENFO : public SmEvalNFunctionsObject
{
protected:
  const SmBSplineCurve & m_crUVCurve;
  SmExtent1d             m_vInterval;
  const SmSurface      & m_crSurfaceOfCurve;
  ULONG                  m_lRailIndex;
  SmFilletSolver       & m_crFilletSolver;
public:
// constructor, destructor
SmRailUVCurveIntersectENFO
  (const SmBSplineCurve & crUVCurve,
   const SmExtent1d & crInterval,
   const SmSurface & crSurfaceOfCurve,
   ULONG            lRailIndex,
   SmFilletSolver & crFilletSolver)
 : m_crUVCurve(crUVCurve), m_vInterval(crInterval),
   m_crSurfaceOfCurve(crSurfaceOfCurve),
   m_lRailIndex(lRailIndex), m_crFilletSolver(crFilletSolver)
  { }
virtual ~SmRailUVCurveIntersectENFO() {}

// virtual function
virtual SmStatus Evaluate(const SmTArray<double> & crX,             // in : x of Ax=F
                          SmTArray<double>       & rF,              // out: F of Ax=F function values,
                          SmMatrix               * pOptJacobian,    // out: Partial derivatives of the functions.
                          SmBoolean              & rbFoundAnswer);  // out: Not always used, when used
                                                                    //      TRUE = converged (F members are within tolerance of 0.0
                                                                    //      FALSE= Not Used or Not Converged
} ; // end class SmRailUVCurveIntersectENFO

#endif // !__SMFILLETSOLVER_H__

