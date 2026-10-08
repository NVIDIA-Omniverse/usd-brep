// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmOffsetExecutive.h 
* PURPOSE: Header file for SmOffsetExecutive object.
**********************************************************************/

#ifndef __SMOFFSETEXECUTIVE_H__
#define __SMOFFSETEXECUTIVE_H__

#ifndef __SMRELATION_H__
#include <SmRelation.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMOFFSETGEOMETRYCREATION_H__
#include <SmOffsetGeometryCreation.h>
#endif

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

#ifndef __SMATTRIBUTE_H__
#include <SmAttribute.h>
#endif

SM_RELATION_TEMPLATE_PREDECLARATION(SmFaceuse,SmFace)
SM_RELATION_TEMPLATE_PREDECLARATION(SmEdgeuse,SmEdgeuse)
SM_RELATION_TEMPLATE_PREDECLARATION(SmEdge,SmFace)
SM_RELATION_TEMPLATE_PREDECLARATION(SmVertex,SmFace)
SM_RELATION_TEMPLATE_PREDECLARATION(SmFace,SmFace)
SM_RELATION_TEMPLATE_PREDECLARATION(SmSurface,SmSurface)

// not being used
/*******************************************************************//**
PURPOSE: What type of face is this.

NOTES: 
*********************************************************************** 
enum SmFaceDescriptorType {
    SM_CD_STANDARD,      // Standard face
    SM_CD_CORNER_BLEND,  // Corner Blend Face
    SM_CD_FILLET         // Fillet or blend 

};
*/ // end not being used

// not being used
/*******************************************************************//**
PURPOSE: This class contains a reverse engineered and possibly offset
   description of a blend or fillet face.  We can use this information to 
   increase the quality of offsetting algorithms.
   THIS IS NOT USED (SMLib V6.5)

NOTES: 
   See also:
    SmBSplineSurface::TestForFilletSurface()
    SmSurface::CalculateFilletRadius()
    SmSurface::FindFilletMinMaxRadii()
    SmFace::IdentifyFillet()
    SmFace::ExtractFillet()
**********************************************************************
class
SmFaceDescriptor : public SmAttribute
{
public:
    SmFaceDescriptorType    m_eFaceDescriptor;  // What kind of a face is this

    // The following are relevant if it is a fillet type
    ULONG                   m_lFilletCrossSection;
    ULONG                   m_lFilletSolverType;
    SmSurfParamType         m_eFilletRailDirection;
    SmBoolean               m_bNormalIsOutward;
    SmBSplineCurve        * m_pMinRail;
    SmBSplineCurve        * m_pMaxRail;
    SmFace                * m_pMinFace;
    SmFace                * m_pMaxFace;

    // The following are relevant if it is a blend type
    SmTArray<SmCurve*>          m_v3DCurves; // 0-Left, 1-Bottom, 2-Right, 3-Top
    // For 3 sided case the Top will be non-existant or a Point Curve
    SmTArray<SmOrientType>      m_vCurveOrients;
    SmTArray<SmBSplineCurve*>   m_vUVCurves;
    SmTArray<SmSurface*>        m_vSurfaces;

public:
};
*/ // end not being used

/*******************************************************************//**
PURPOSE: This type controls where the results of the offset are to
   be placed, either in a new Brep or the original Brep.

NOTES: 
***********************************************************************/
enum SmOffsetOperationType
{
    SM_OO_SOLID_OFFSET,    // Straight volumetric offset into a New Brep
    SM_OO_NMT_OFFSET,      // Non-manifold offset 
    SM_OO_SHELL,           // Offset into original Brep
    SM_OO_LOCAL_OPERATION, // Local Operation  
    SM_OO_MACHINE_OFFSET   // Doing a Machining Operation Offset
};

/*******************************************************************//**
PURPOSE: The Offset Executive is the top level object which controls
   the topological offset and shelling operations.

NOTES: There is room for enhancements to this functionality.
   If you have specific requests, please contact Support.
***********************************************************************/
class SM_EXPORT SmOffsetExecutive
{
protected:                        
  const SmContext               & m_crContext; 
                                                        // Context for the creation of all temporary and permanent objects
  SmOffsetOperationType           m_eOffsetOperation;   // What type of offsetting operation is being done - 
                                                        //   Solid, NMT, Shell, Local Operation, offset for Machining.
  SmOffsetGeometryCreation      & m_rGeometryCreation;  // Creates offset geometry for individual topology objects.
  SmBrep                        * m_pOriginalBrep;      // Original Brep to be Offset
  SmBrep                        * m_pOffsetBrep;        // A temporary, used internally
  SmBoolean                       m_bInset;             // If TRUE do an inset into a solid
                                                        //  not an offset outside of the solid.
  SmBoolean                       m_bExtendConvexEdges; // If TRUE we do an extension/intersect
                                                        //  instead of a fillet of the convex 
                                                        //  edges. (DoExtendedOffset()) 
  SmBoolean                       m_bMergeResults;      // If TRUE then we don't 
                                                        //  stitch but we merge results 
  SmBoolean                       m_bCreateOffsetSolid; // If TRUE it will make ruled surfaces
                                                        //  between lamina edges of original and offset.
  SmTArray<SmFace*>             * m_pShellFaces;        // These faces are to be shelled not offset:
                                                        //  they just get copied instead of offset
                                                        //  into the result.
  SmTArray<SmFace*>               m_sClosingFaces;      // Faces created by BuildSideWallOfEdgeuse(), connecting
                                                        //  a lamina edge with its offset, to close off the final Brep.
  SmRelation<SmFaceuse,SmFace>    m_vFUToF;             // map InfiniteRegion bounding OriginalBrep->faceuses to new OffsetBrep faces
  SmRelation<SmEdgeuse,SmEdgeuse> m_vEUToEU;            // map of all m_pOriginalBrep->face->edgeuses to m_pOffsetBrep->face->edgeuses
  SmRelation<SmEdge,SmFace>       m_vEToF;              // map origEdges to CapFaces created to fill gaps created by offsetting surfaces in convex sectors
  SmRelation<SmVertex,SmFace>     m_vVToF;              // map origVertices to CapFaces created to fill gaps created by offsetting surfaces from the vertex
  SmRelation<SmFace,SmFace>       m_vBlendToFaces;
  SmMapPtrToPtr<SmSurface, SmSurface> & m_vSurfExtSurf;

public:

  // Constructor, destructor:
  SmOffsetExecutive(const SmContext & crContext,
                    SmOffsetOperationType eOperationType,
                    SmOffsetGeometryCreation & rGeometryCreation,
                    SmBrep *pOriginalBrep);

  virtual ~SmOffsetExecutive();


  // High-level interface:
  static SmStatus ShellBrep(const SmContext & crContext,
                            SmBrep *pBrepToShell,
                            double dOffsetDistance,
                            SmBoolean bDoExtendedOffset,
                            SmBoolean bDoSelfInt,
                            SmBoolean bCreateOffsetSolid,
                            const SmTArray<SmFace*> & crFacesToShell,
                            SmBrep *& crShellBrep);

  // Same operation as the 8-argument ShellBrep() overload, with explicit
  // ownership reporting for adapters that pass a private working copy.  TRUE
  // means the input was either deleted by the operation or returned as
  // crShellBrep; FALSE means it remains a separate allocation owned by the
  // caller.
  static SmStatus ShellBrep(const SmContext & crContext,
                            SmBrep *pBrepToShell,
                            double dOffsetDistance,
                            SmBoolean bDoExtendedOffset,
                            SmBoolean bDoSelfInt,
                            SmBoolean bCreateOffsetSolid,
                            const SmTArray<SmFace*> & crFacesToShell,
                            SmBrep *& crShellBrep,
                            SmBoolean & rbInputOwnershipTransferred);

  static SmStatus OffsetBrep(const SmContext & crContext,
                             SmBrep *pBrepToOffset,
                             double dOffsetDistance,
                             SmBoolean bDoExtendedOffset,
                             SmBoolean bDoSelfInt,
                             SmBrep *& crOffsetBrep);

  virtual SmStatus DoSolidOffset(SmBrep *& rpResult, SmBoolean bSkipStitchMerge=FALSE); // note: increments unlocked mark value

  SmStatus DoExtendedOffset(SmBrep *& rpResult); // note: increments unlocked mark value


  // Member access:
  SmBoolean             GetInset              () const { return m_bInset; }
  SmOffsetOperationType GetOffsetOperation    () const { return m_eOffsetOperation ; }
  double                GetTangencyTolDegrees ()       { return m_rGeometryCreation.GetTangencyTolDegrees(); }
  SmBoolean             GetExtendConvexEdges  () const { return m_bExtendConvexEdges; }
  SmBrep              * GetOffsetBrep         ()       { return m_pOffsetBrep; }
  void GetShellFaces(SmTArray<SmFace*> & rFaces) const 
    { rFaces.ReSet(); 
      if (m_pShellFaces) 
        { rFaces.Append(*m_pShellFaces); }
    } 

  void SetInset             (SmBoolean bInset)              { m_bInset = bInset; }
  void SetOffsetOperation   (SmOffsetOperationType eOpType) { m_eOffsetOperation = eOpType; }
  void SetCreateOffsetSolid (SmBoolean bCreateOffsetSolid)  { m_bCreateOffsetSolid = bCreateOffsetSolid; }
  void SetExtendConvexEdges (SmBoolean bExtendConvexEdges)  { m_bExtendConvexEdges = bExtendConvexEdges; }
  void SetMergeResults      (SmBoolean bMergeResults)       { m_bMergeResults = bMergeResults; }
  void SetShellFaces        (const SmTArray<SmFace*> & rFaces) 
    { if (m_pShellFaces) 
        { delete m_pShellFaces; m_pShellFaces = NULL ; }
      m_pShellFaces = new (m_crContext) SmTArray<SmFace*>(m_crContext);
      m_pShellFaces->Append(rFaces);
    }


  // Internals:

  SmStatus BuildSideWallOfEdgeuse(SmEdgeuse *pEdgeuse);

  virtual SmStatus BuildOffsetOfEdgeuses(SmEdgeuse *pEdgeuse,
                                         SmEdgeuse *pOtherEdgeuse,
                                         SmMarkType eMarkType) ;   // in : uses without increment eMarkType value

  SmStatus BuildFilletOffsetsOfFaceuse(SmFaceuse *pFU); // (Not implemented.)  // NotUsed: in : pFU

  SmStatus BuildOffsetOfFaceuse
  (
    SmFaceuse *  pFU, 
    SmSurface* & rSurface,
    SmSSIData &  rSelfIntersections
  );

  virtual SmStatus BuildOffsetOfVertex(SmVertex               * pVertex,
                                       SmTArray<SmCurve*>     & r3DTrimCurves,
                                       SmTArray<SmOrientType> & rCurveOrientations,
                                       SmSurface             *& rpNewSurface);

  SmStatus FillGapEdgeuses(SmEdge *pEdge,
                           const SmTArray<SmEdgeuse*> & crGapEdgeuses,
                           const SmTArray<SmFace*> & crCanidateFaces);

  SmStatus GetCreateExtendedSurface(SmSurface *pBase,
                                    SmExtent2d & rUVDomainOfOffset,
                                    SmSurface *& rpExtSurface);

  SmEdgeuse * GetOffsetEdgeuse(SmEdgeuse *cpOrigEu);

  SmStatus MapEdgeuses(SmFaceuse * pOriginalFU,
                       SmFace * pOffsetFace);

  SmStatus RemoveExcessFaces(SmBrep * pResult);

  // Replace all SmOffsetSurfaces with approximate SmBSplineSurfaces in target Brep.
  static SmStatus ReplaceImplicitOffsets(SmBrep * pBrepToFix,
                                         double dThisApproxTol3d);

  SmStatus SyncronizeOffsetEdgeuses(SmEdgeuse *pOrigEdgeuse,
                                    SmEdgeuse *pOtherEdgeuse,
                                    SmTArray<SmEdgeuse*> & rOffsetEdgeuses,
                                    SmTArray<SmEdgeuse*> & rOtherOffsetEdgeuses,
                                    SmTArray<SmEdgeuse*> & rOffGapEdgeuses,
                                    SmTArray<SmEdgeuse*> & rOtherOffGapEdgeuses);

  SmStatus PiecewiseMerge(SmBrep *& rpResult);


  // Utility:
  SmDisplayList * Draw() const ;  // draw face with mode and optional xHatch counts
  void Dump(SmBoolean bDumpMapObjects=FALSE) const;  // NotUsed: in : bDumpMapObjects
    
} ; // end class SmOffsetExecutive

#endif // !__SMOFFSETEXECUTIVE_H__
