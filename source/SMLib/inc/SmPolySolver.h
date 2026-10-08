// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPolySolver.h
* PURPOSE: Header file for SmPolySolver object.
**********************************************************************/

#ifndef __SMPOLYSOLVER_H__
#define __SMPOLYSOLVER_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

// #ifndef __SMMAPPTRTOPTR_H__
// #include <SmMapPtrToPtr.h>
// #endif

#ifndef __SMGLOBALSOLVER_H__
#include <SmGlobalSolver.h>
#endif

// forward declarations
class SmPolySolver;

/*******************************************************************//**
PURPOSE:  The PolySolver class uses the PolyLocalSolver classes
    to solve the variety of global problems between pairs of Poly objects.

    Solutions are available for all the SmSolverOperationType values:

NOTES: 
***********************************************************************/
// contains: classes SmPolySolver            SmPolyLocalSolver           
//                      SmPolyPlaneSolver       SmPLSEdgeEdge             
//                                              SmPLSEdgeFace             
//                                              SmPLSPlane                
//                                                 SmPLSPlaneVertex        
//                                                 SmPLSPlaneEdge          
//                                              SmPLSPoint                
//                                                 SmPLSPointVertex        
//                                                    SmPLSVertexVertex
//                                                 SmPLSPointEdge          
//                                                    SmPLSVertexEdge       
//                                                 SmPLSPointFace          
//                                                    SmPLSVertexFace 

// SmSolution uses:
//  SmPLSPointVertex and SmPLSVertexVertex::SolverIt 
//  SmPLSPointEdge   and SmPLSVertexEdge::SolverIt 
//  SmPLSPointFace   and SmPLSVertexFace::SolverIt 
//  SmPLSEdgeEdge::SolveIt
//  SmPLSEdgeFace::SolveIt
//  SmPLSPlaneVertex::SolveIt  (Only uses m_vStart[0,1,2] to store the vertex point - not the plane point)
//  SmPLSPlaneEdge::SolveIt    (only uses m_vStart[0,1,2] to store the edge ponit = not the plane point)
//      // set output
//      sSol.m_eSolutionType = SM_ST_SINGLE_VALUE;
//      sSol.m_lNumVariables = 6;
//      sSol.m_vStart.m_dSolutionValue = dSign*smos_Sqrt(dDistSq);
//      sSol.m_vStart[0] = Pnt1.x;       // object 1 point
//      sSol.m_vStart[1] = Pnt1.y;
//      sSol.m_vStart[2] = Pnt1.z;
//      sSol.m_vStart[3] = Pnt2.x;       // object 2 (or point) point
//      sSol.m_vStart[4] = Pnt2.y;
//      sSol.m_vStart[5] = Pnt2.z;
//      sSol.m_lNumObjects = 2 for Vertex/Vertex problems  Vertex/Edge
//                           1 for Vertex/Point problems   Edge/Point
       
/*******************************************************************//**
PURPOSE: These classes provides the interface for local solutions of
    two objects.  

NOTES: These classes are used only by SmPolySolver.
***********************************************************************/
class SmPolyLocalSolver
{
 protected:
    SmPolySolver  & m_rPolySolver;
    SmBoolean       m_bSwapOrder;
 public:
    // constructor
    SmPolyLocalSolver(SmPolySolver & rPolySolver, 
                      SmBoolean      bSwapOrder)
                    : m_rPolySolver(rPolySolver), 
                      m_bSwapOrder (bSwapOrder) 
                    { }
    // destructor
    virtual ~SmPolyLocalSolver() { }

    // solver
    virtual  SmStatus SolveIt(const SmObject * /*pObj1*/, 
                              const SmObject * /*pObj2*/) { SE(SM_ERR) ; return SM_ERR; }

} ; // end class SmPolyLocalSolver

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSPoint : public SmPolyLocalSolver
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
 protected:
       SmPoint3d             m_vPoint;
       const SmPolyVertex  * m_pPointVertex;

 public:
     // constructor
     SmPLSPoint(SmPolySolver & rPolySolver, 
                SmBoolean      bSwapOrder = FALSE)
              : SmPolyLocalSolver(rPolySolver, bSwapOrder), 
                m_pPointVertex   (NULL) 
              { }
     // destructor
     virtual ~SmPLSPoint() { }

     // access
     void SetPoint(const SmPoint3d & crPoint)             { m_vPoint       = crPoint; }
     void SetPointVertex(SmPolyVertex * pPointVertex)     { m_pPointVertex = pPointVertex; }

     // solver
     virtual SmStatus SolveIt(const SmObject * /*pObj1*/, 
                              const SmObject * /*pObj2*/) { SE(SM_ERR) ; return SM_ERR; }
} ; // end class SmPLSPoint

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSPointVertex : public SmPLSPoint
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
  // inherited from SmPLSPoint
  //   SmPoint3d             m_vPoint;
  //   const SmPolyVertex  * m_pPointVertex;

 public:
    // constructor
    SmPLSPointVertex(SmPolySolver & rPolySolver, 
                     SmBoolean      bSwapOrder = FALSE)
                   : SmPLSPoint(rPolySolver,bSwapOrder) 
                   { }
    // destructor
    virtual ~SmPLSPointVertex() { }

    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSPointVertex

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSVertexVertex : public SmPLSPointVertex
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
  // inherited from SmPLSPoint
  //   SmPoint3d             m_vPoint;
  //   const SmPolyVertex  * m_pPointVertex;

 public:
    // constructor
    SmPLSVertexVertex(SmPolySolver & rPolySolver, 
                     SmBoolean       bSwapOrder = FALSE)
                   : SmPLSPointVertex(rPolySolver,bSwapOrder) 
                   { }
    // destructor
    virtual ~SmPLSVertexVertex() { }
    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSVertexVertex

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSPointEdge : public SmPLSPoint
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
  // inherited from SmPLSPoint
  //   SmPoint3d             m_vPoint;
  //   const SmPolyVertex  * m_pPointVertex;

 public:
    // constructor
    SmPLSPointEdge(SmPolySolver & rPolySolver, 
                   SmBoolean      bSwapOrder = FALSE)
                 : SmPLSPoint(rPolySolver,bSwapOrder) 
                 { }
    // destructor
    virtual ~SmPLSPointEdge() { }
    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSPointEdge


/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSVertexEdge : public SmPLSPointEdge
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
  // inherited from SmPLSPoint
  //   SmPoint3d             m_vPoint;
  //   const SmPolyVertex  * m_pPointVertex;
 public:
    // constructor
    SmPLSVertexEdge(SmPolySolver & rPolySolver, 
                   SmBoolean       bSwapOrder = FALSE)
                 : SmPLSPointEdge(rPolySolver,bSwapOrder) 
                 { }
    // destructor
    virtual ~SmPLSVertexEdge() { }
    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSVertexEdge

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSEdgeEdge : public SmPolyLocalSolver
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
 public:
    // constructor
    SmPLSEdgeEdge(SmPolySolver & rPolySolver, 
                  SmBoolean      bSwapOrder = FALSE)
                : SmPolyLocalSolver(rPolySolver,bSwapOrder) 
                { }
    // destructor
    virtual ~SmPLSEdgeEdge() { }
    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSEdgeEdge 

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSEdgeFace : public SmPolyLocalSolver
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
 public:
    // constructor
    SmPLSEdgeFace(SmPolySolver & rPolySolver, 
                  SmBoolean      bSwapOrder = FALSE)
                : SmPolyLocalSolver(rPolySolver,bSwapOrder) 
                { }
    // destructor
    virtual ~SmPLSEdgeFace() { }
    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSEdgeFace 

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSPointFace : public SmPLSPoint
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
  // inherited from SmPLSPoint
  //   SmPoint3d             m_vPoint;
  //   const SmPolyVertex  * m_pPointVertex;
 public:
    // constructor
    SmPLSPointFace(SmPolySolver & rPolySolver, 
                   SmBoolean      bSwapOrder = FALSE)
                 : SmPLSPoint(rPolySolver,bSwapOrder) 
                 { }
    // destructor
    virtual ~SmPLSPointFace() { }
    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSPointFace 

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSVertexFace : public SmPLSPointFace
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
  // inherited from SmPLSPoint
  //   SmPoint3d             m_vPoint;
  //   const SmPolyVertex  * m_pPointVertex;
 public:
    // constructor
    SmPLSVertexFace(SmPolySolver & rPolySolver, 
                   SmBoolean       bSwapOrder = FALSE)
                 : SmPLSPointFace(rPolySolver,bSwapOrder) 
                 { }
    // destructor
    virtual ~SmPLSVertexFace() { }
    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSVertexFace 

/*******************************************************************//**
PURPOSE: The Poly solver class provides an interface and mechansim
    to solve global problems using Poly objects.

NOTES: 
***********************************************************************/
class SM_EXPORT SmPolySolver
{
    friend class SmTree;
    friend class SmPLSPointVertex;
    friend class SmPLSPointEdge;
    friend class SmPLSPointFace;
    friend class SmPLSEdgeEdge;
    friend class SmPLSEdgeFace;
    friend class SmPLSPlaneVertex;
    friend class SmPLSPlaneEdge;
 protected:
    SmPolyLocalSolver         * m_pLocalSolver;
    SmSolverOperationType       m_eSolverOperation;
    SmOperationCategory         m_eOperationCategory;
    SmBoolean                   m_bProjectedOperation;
    SmSolutionRequestedType     m_eSolutionRequested;
    ULONG                       m_lNumTrees;
    ULONG                       m_lNumVariables;
    SmTree                    * m_apTrees[SM_GS_MAX_TREES];
    double                      m_d3dTolerance;
    double                      m_dBestAnswerSoFarSq;
    double                      m_dAtDistance;
    double                      m_dAtDistanceSq;  
    const SmVector3d          * m_cpOptVectors;              // Vectors used by some solvers 
    SmSolutionArray           * m_pSolutions;                // Resulting answers 

 public:
    // constructor
    SmPolySolver(SmSolverOperationType   eSolverOperation,
                 SmSolutionRequestedType eSolutionRequested,
                 double                  d3dTolerance,
                 double                  dBestAnswerSoFarSq,
                 const SmVector3d      * cpOptVectors,
                 SmSolutionArray       & rSolutions) ;

    // destructor
    virtual ~SmPolySolver() { }

    // PolyBrep solutions to a plane: currently only SM_SO_MINIMIZE is implemented.
    //  plans for: SM_SO_MAXIMIZE, IS_SO_INTERSECT, SM_SO_NORMALIZE
    //  returns solutions for the distances of vertices and edges to the plane
    static SmStatus PolyBrepPlaneSolve(const SmPolyBrep       * cpPolyBrep, 
                                       const SmPoint3d        & crPlanePoint,
                                       const SmVector3d       & crPlaneNormal,
                                       SmSolverOperationType    eSolverOperation,
                                       SmSolutionRequestedType  eSolutionRequested,
                                       double                   d3dTolerance,
                                       double                   dBestAnswerSoFarSq,
                                       const SmVector3d       * cpOptVectors,
                                       SmSolutionArray        & rSolutions) ;

    // PolyBrep solutions to a point, valid operations include: 
    //      SM_SO_MINIMIZE,                  SM_SO_MAXIMIZE,
    //      SM_SO_PROJECTED_MINIMIZE,        SM_SO_PROJECTED_MAXIMIZE, 
    //      SM_SO_DIRECTED_MINIMIZE,         SM_SO_DIRECTED_MAXIMIZE,  
    //      SM_SO_SIGNED_DIRECTED_MINIMIZE,  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE.
    // returns solutions for polyvertexes, polyedges, and polyfaces for the given point
    static SmStatus PolyBrepPointSolve(const SmPolyBrep       * cpPolyBrep, 
                                       const SmPoint3d        & crPoint,
                                       SmSolverOperationType    eSolverOperation,
                                       SmSolutionRequestedType  eSolutionRequested,
                                       double                   d3dTolerance,
                                       double                   dBestAnswerSoFarSq,
                                       const SmVector3d       * cpOptVectors,
                                       SmSolutionArray        & rSolutions) ;

    // PolyBrep to PolyBrep solutions, valid operations include:
    //      SM_SO_MINIMIZE,                 SM_SO_MAXIMIZE,
    //      SM_SO_PROJECTED_MINIMIZE,       SM_SO_PROJECTED_MAXIMIZE, 
    //      SM_SO_DIRECTED_MINIMIZE,        SM_SO_DIRECTED_MAXIMIZE, 
    //      SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_3D_SIGNED_DIRECTED_MINIMIZE.
    // Looks for all Vertex/Vertex, Vertex/Edge, Vertex/Face, Edge/Edge, and Edge/Face soluctions.
    //   note: no Face/Face solutions.
    static SmStatus PolyBrepPolyBrepSolve(const SmPolyBrep       * cpPolyBrep1, 
                                          const SmPolyBrep       * cpPolyBrep2, 
                                          SmSolverOperationType    eSolverOperation,
                                          SmSolutionRequestedType  eSolutionRequested,
                                          double                   d3dTolerance,
                                          double                   dBestAnswerSoFarSq,
                                          const SmVector3d       * cpOptVectors,
                                          SmSolutionArray        & rSolutions) ;

    // intersects the PolyBrep with the rectangle and returns any one 
    //   vertex/vertex, vertex/edge, vertex/face, edge/edge, edge/face point intersection
    static SmStatus PolyBrepRectangleIntersect(const SmPolyBrep       * cpPolyBrep, 
                                               const SmAxis2Placement & crRecPlacement,
                                               const SmExtent2d       & crRecDomain,
                                               double                   d3dTolerance,
                                               SmSolutionArray        & rSolutions) ;
    // internal solver piece
    void AddSortedSolution(const SmSolution & crSolution,
                           SmSortKeyType      eSortKey) ;

    // internal solver piece
    virtual SmBoolean BranchMayContainAnswers(SmTreeNode * apBranch[SM_GS_MAX_TREES]) ;

    // internal solver piece
    SmStatus FindBestSubdivisionNode(SmTreeNode * apBranch[SM_GS_MAX_TREES],
                                     ULONG & rlBestSubdivisionIndex,
                                     SmBoolean & rbSwapTraversalOrder) const;

    // internal solver piece
    virtual SmBoolean IdenticalSolutions(const SmSolution & crSolution,
                                         const SmSolution & crSol2,
                                         const SmSolution *&pPreferredSol) const;

    // internal solver piece
    virtual SmStatus LocalSolve(SmTreeNode * apBranch[SM_GS_MAX_TREES], 
                                SmBoolean & rbNeedsMoreSubdivision) ;

    // internal solver piece
    virtual SmBoolean MayContainAnswer(const SmExtent3d & crBox1, 
                                       const SmExtent3d & crBox2) const;

    //
    static SmStatus CachePointSolve(SmBrepCache *pBC, 
                                    const SmPoint3d & crPoint,
                                    SmSolverOperationType eSolverOperation,
                                    SmSolutionRequestedType eSolutionRequested,
                                    double d3dTolerance,
                                    double dBestAnswerSoFarSq,
                                    const SmVector3d * cpOptVectors,
                                    SmSolutionArray & rSolutions) ;

    // internal solver piece
    SmStatus SolveBranch(SmTreeNode * apBranch[SM_GS_MAX_TREES]) ;

    // internal solver piece
    SmStatus Subdivide(SmTreeNode* apOriginal[SM_GS_MAX_TREES],
                       SmTreeNode * apChildBranch1[SM_GS_MAX_TREES],
                       SmTreeNode * apChildBranch2[SM_GS_MAX_TREES]) ;

    // internal solver piece
    SmStatus SolveTrees(SmTree & rTree1,
                        SmTree & rTree2,
                        SmPolyLocalSolver & rLocalSolve) ;

    // internal solver piece
    SmStatus SolveTwoTrees(SmTree & rTree1,
                           SmTree & rTree2,
                           SmPolyLocalSolver & rLocalSolve) ;
    
    //
    void GetBestAnswerSoFar(double & rdBestAnwser, double * & rpdBestAnswer) ;

} ; // end class SmPolySolver 

/*******************************************************************//**
PURPOSE:  Solver for plane-polybrep solve

NOTES: 
***********************************************************************/
class SM_EXPORT SmPolyPlaneSolver : public SmPolySolver
{
    friend class SmPLSPlane;
 // inherited from SmPolySolver
 //   SmPolyLocalSolver        * m_pLocalSolver;
 //   SmSolverOperationType      m_eSolverOperation;
 //   SmOperationCategory        m_eOperationCategory;
 //   SmBoolean                  m_bProjectedOperation;
 //   SmSolutionRequestedType    m_eSolutionRequested;
 //   ULONG                      m_lNumTrees;
 //   ULONG                      m_lNumVariables;
 //   SmTree                   * m_apTrees[SM_GS_MAX_TREES];
 //   double                     m_d3dTolerance;
 //   double                     m_dBestAnswerSoFarSq;
 //   double                     m_dAtDistance;
 //   double                     m_dAtDistanceSq;  
 //   const SmVector3d         * m_cpOptVectors;              // Vectors used by some solvers 
 //   SmSolutionArray          * m_pSolutions;                // Resulting answers 

 protected:
      SmPoint3d                  m_vPlanePoint;
      SmVector3d                 m_vPlaneNormal;

 public:
    // constructor
    SmPolyPlaneSolver(SmSolverOperationType     eSolverOperation,
                      const SmPoint3d         & crPlanePoint,
                      const SmVector3d        & crPlaneNormal,
                      SmSolutionRequestedType   eSolutionRequested,
                      double                    d3dTolerance,
                      double                    dBestAnswerSoFarSq,
                      const SmVector3d        * cpOptVectors,
                      SmSolutionArray         & rSolutions) ;

    // destructor
    virtual ~SmPolyPlaneSolver() { }

    //
    virtual SmBoolean BranchMayContainAnswers(SmTreeNode * apBranch[SM_GS_MAX_TREES]) ;

    //
    virtual SmStatus LocalSolve(SmTreeNode * apBranch[SM_GS_MAX_TREES], 
                                SmBoolean & rbNeedsMoreSubdivision) ;

    //
    virtual SmBoolean MayContainAnswer(const SmExtent3d & crBox1, 
                                       const SmExtent3d & crBox2) const;

    //
    SmStatus SolveTree(SmTree & rTree,
                       SmPolyLocalSolver & rLocalSolve) ;

} ; // end class SmPolyPlaneSolver

/*******************************************************************//**
PURPOSE:  Local solvers for plane-polybrep solve

NOTES: 
***********************************************************************/
class SmPLSPlane : public SmPolyLocalSolver
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
 protected:
       SmPoint3d             m_vPlanePoint;
       SmVector3d            m_vPlaneNormal;
 public:
    // constructor
    SmPLSPlane(SmPolyPlaneSolver & rPolySolver)
             : SmPolyLocalSolver((SmPolySolver&)rPolySolver,FALSE),
               m_vPlanePoint(rPolySolver.m_vPlanePoint),
               m_vPlaneNormal(rPolySolver.m_vPlaneNormal) 
             { }
    // destructor
    virtual ~SmPLSPlane() { }
    // solver
    virtual SmStatus SolveIt(const SmObject * /*pObj1*/, 
                             const SmObject * /*pObj2*/) { SE(SM_ERR) ; return SM_ERR; }
} ; // end class SmPLSPlane 

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSPlaneVertex : public SmPLSPlane
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
  // inherited from SmPLSPlane
  //  SmPoint3d             m_vPlanePoint;
  //  SmVector3d            m_vPlaneNormal;
 public:
    // constructor
    SmPLSPlaneVertex(SmPolyPlaneSolver & rPolySolver)
                   : SmPLSPlane(rPolySolver) { }
    // destructor
    virtual ~SmPLSPlaneVertex() { }
    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSPlaneVertex 

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmPLSPlaneEdge : public SmPLSPlane
{
  // inherited from SmPolyLocalSolver
  //   SmPolySolver        & m_rPolySolver;
  //   SmBoolean             m_bSwapOrder; 
  // inherited from SmPLSPlane
  //  SmPoint3d             m_vPlanePoint;
  //  SmVector3d            m_vPlaneNormal;
 public:
    // constructor
    SmPLSPlaneEdge(SmPolyPlaneSolver & rPolySolver)
                 : SmPLSPlane(rPolySolver) { }
    // destructor
    virtual ~SmPLSPlaneEdge() { }
    // solver
    virtual SmStatus SolveIt(const SmObject *pObj1, 
                             const SmObject *pObj2) ;
} ; // end class SmPLSPlaneEdge 


#endif // no __SMPOLYSOLVER_H__
