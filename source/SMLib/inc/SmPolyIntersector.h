// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPolyIntersector.h
* PURPOSE: Header file for SmPolyIntersector object.
**********************************************************************/

#ifndef __SMPOLYINTERSECTOR_H__
#define __SMPOLYINTERSECTOR_H__

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

//#ifndef __SMEXTENT3D_H__
//#include <SmExtent3d.h>
//#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

//#ifndef __SMSARRAY_H__
//#include <SmSArray.h>
//#endif

//#ifndef __SMGLOBALSOLVER_H__
//#include <SmGlobalSolver.h>
//#endif

#ifndef __SMMAPPTRTOPTR_H__
#include <SmMapPtrToPtr.h>
#endif

#ifndef __SMPOLY_H__
#include <SmPoly.h>
#endif

#ifndef __SMLINESEGCLASS_H__
#include <SmLineSegClass.h>
#endif


/*******************************************************************//**
PURPOSE: The Poly intersector object provides the ability 
    to create edges and vertices corresponding to intersections
    in each Brep.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmPolyIntersector
{
  friend class SmPolyMerge;
public:
  SmBoolean             m_bPolyDeleted;              // If TRUE then some topological objects have been
                                                     // deleted during subsequent operations.
                                                     
private:                                             
  const SmContext     & m_crContext;                 // Context in which to create temporary data strucutres
                                                     // used by the topology intersector.
  SmPolyBrep          * m_pBrep;                     // Brep which receives the merge
  SmPolyBrep          * m_pOther;                    // Brep to be merged into result
  SmMapPtrToPtr<SmObject, SmObject> * m_pBToO;       // Maps Brep objects to Other objects
  SmMapPtrToPtr<SmObject, SmObject> * m_pOToB;       // Maps Other objects to Brep objects
  SmBoolean             m_bSwappedOrder;             // If TRUE we have swapped the order because
                                                     // of the size of one of the objects.
  double                m_dThisApproxTol3d;          // Tolerance used in creating Edges.
  double                m_dThisAngTolRad;            // Angular tolerance used in creating curves - degrees.
  SmTArray<SmPolyFace*> m_vDeletedFaces;
  SmTArray<SmPolyFace*> m_vOtherDeletedFaces;
  SmBoolean             m_bTrimWithPlane;            // If TRUE, m_pBrep will be trimmed by a plane
  SmPoint3d             m_vTrimPlanePoint;
  SmVector3d            m_vTrimPlaneNormal;

public:
  ~SmPolyIntersector();

  SmPolyIntersector(const SmContext & crContext);

  SmPolyIntersector(const SmContext & crContext,
                    SmPolyBrep *pBrep, 
                    SmPolyBrep *pOther,
                    double d3DApproximationTolerance,
                    double dThisAngTolRad);

  SmStatus CheckForSqueezeVertices
  (
      SmPolyVertex            * pBrepVertex,     // in : PolyVertex from m_pBrep
      SmPolyVertex            * pOtherVertex,    // in : PolyVertex from m_pOther
      SmTArray<SmPolyVertex*> & rDeletedV,       // i/o: accumulating list of stale PolyVertices
      SmTArray<SmPolyVertex*> & rSurvivingV,     // i/o: associated list of PolyVertices replacing stale PolyVertices (NULL=no replacement)
      SmTArray<SmPolyEdge*>   & rDeletedE,       // i/o: accumulating list of stale PolyEdges
      SmTArray<SmPolyEdge*>   & rSurvivingE,     // i/o: associated list of PolyEdges replacing stale PolyEdges(NULL=no replacement)
      SmBoolean                 bCheckCloserMate // in : TRUE  = Check if current BrepVertex and OtherVertex mates are closer than 
                                                 //              than BrepVertex and OtherVertex are to eachother
                                                 //      FALSE = Don't.
                                                 //      Use FALSE when squeezing degenerate edge
  ); 


  SmStatus DeleteFaces(const SmTArray<SmPolyFace*> & crPolyFaces);

  SmStatus FixGaps(SmTArray<SmPolyVertex*> & crSingleVertices,
                   SmBoolean & rbModifiedTopology);

  SmStatus FixRelationsAtVertex(SmPolyVertex * pVertex,      // in : Brep vertex   of relationship(pVertex, pOtherVertex)
                                SmPolyVertex * pOtherVertex, // in : Other vertex  of relationship(pVertex, pOtherVertex)
                                SmBoolean    & rbFixed);     // out: TRUE = made changes, FALSE = didn't

  SmStatus IntersectInsertRelate();

  SmStatus IIRFaces(SmPolyFace *pFace, 
                    SmPolyFace *pOtherFace, 
                    const SmExtent3d * cpIntersectionBBox = NULL);

  SmStatus Relate(SmObject *pBrepTopo, SmObject *pOtherTopo);

  SmPolyBrep * GetBrep           () const { return m_pBrep ; }
  SmPolyBrep * GetOther          () const { return m_pOther ; }
  double       GetThisApproxTol3d() const { SM_ASSERT_TOL(m_dThisApproxTol3d) ; return m_dThisApproxTol3d; } 
  double       GetThisAngTolRad  () const { SM_ASSERT_TOL(m_dThisAngTolRad) ; return m_dThisAngTolRad; } 

  void GetCommonEdges(SmTArray<SmPolyEdge*> & rEdges, 
                      SmTArray<SmPolyEdge*> & rOtherEdges) const;

  void GetCommonVertices(SmTArray<SmPolyVertex*> & rVertices, 
                         SmTArray<SmPolyVertex*> & rOtherVertices) const;

//    SmStatus FixupClassifications(SmPolyEdgeClass & rEdgeClassBrep,
//                                  SmPolyEdgeClass & rEdgeClassOther);

//    SmStatus FixMidPointClass(SmPointClassification & rPCBrep,
//                              SmPointClassification & rPCOther);

//    SmStatus FixEndPointClass(ULONG lPointClassIndex,
//                              SmPolyEdgeClass & rEdgeClassBrep,
//                              SmPolyEdgeClass & rEdgeClassOther,
//                              const SmPoint3d & crPoint,
//                              SmBoolean & rbModified);
                                               
  SmObject * GetOtherMate(SmObject * pBrepEntity) const;

  SmObject * GetBrepMate(SmObject * pOtherEntity) const;

  SmStatus MergeCoincidentFaces(SmPolyFace * pBFace,   // increments unlocked marks in pBFace and pOFace contexts
                                SmPolyFace * pOFace); 

  SmStatus MergeLineSegClasses(SmLineSegClassification & rClass1,
                               SmLineSegClassification & rClass2,

                               SmBoolean & rbDeletedTopology);

  SmStatus MergeLineSegOnFaces(SmPolyFace * pFace,
                               SmPolyFace * pOtherFace,
                               double dTolerance,
                               const SmPoint3d & crLinePnt,
                               const SmVector3d & crLineVec,
                               SmPolyEdge * pBrepEdge,
                               SmPolyEdge * pOtherEdge,
                               SmBoolean & rbDeletedPoly);
  
  SmStatus RemoveRelationship(SmTopology * pBrepEntity,
                              SmTopology * pOtherEntity);

  void SetPrimaryBrep(SmPolyBrep *pBrep) { m_pBrep  = pBrep; }
  void SetOtherBrep (SmPolyBrep *pOther) { m_pOther = pOther; }

  SmStatus SplitEdgesAroundVertex(SmPolyVertex            * pVertex,      // in : PolyVertex to check
                                  SmTArray<SmPolyVertex*> & rNewVertices, // out: New Vertices created by splitting edges
                                  SmTArray<SmPolyEdge*>   & rNewEdges) ;  // out: New Edges created by splitting edges

  void Dump(SmBoolean bDumpMapObjects=FALSE) const;  // NotUsed: in : bDumpMapObjects
  void Draw( SmVector3d *pOffset1=NULL,
             SmVector3d *pOffset2=NULL,
             SmVector3d *pColor1=NULL,
             SmVector3d *pColor2=NULL
    );
  void DrawRelatedObjects(); 
  
  void Validate();
};


#endif // !__SMPOLYINTERSECTOR_H__
