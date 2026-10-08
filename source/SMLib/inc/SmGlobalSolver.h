// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmGlobalSolver.h
* PURPOSE:  Header file for SmGlobalSolver object.
**********************************************************************/

#ifndef __SMGLOBALSOLVER_H__
#define __SMGLOBALSOLVER_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMMEMBLOCKMGR_H__
#include <SmMemBlockMgr.h>
#endif

#ifndef _INC_TIME
#include <time.h>
#endif

#ifndef __ATOMIC__
#define __ATOMIC__
#include <atomic>
#endif

#define SM_GS_MAX_TREES 3

// SmGlobalSolver::LocalSove Tolerance Note
//   SmAdvSurfaceIntersector::LocalSolve
//      m_eSolverOperation == SM_SO_INTERSECT        : If(AreDisjoint(BBox1.ExpandAbsolute, BBox2)) No Solution
//      m_eSolverOperation == SM_SO_INTERSECTION_TEST: If(AreDisjoint(BBox1.ExpandAbsolute, BBox2)) No Solution
//   SmCCIGlobalSolver::LocalSolve
//      all SM_SO_... cases                          : if (sExpandedBox.AreDisjoint(sPseudoBox2))
//      m_eSolverOperation == SM_SO_INTERSECTION_TEST: get union of all XSecting BBoxes
//      m_eSolverOperation != SM_SO_INTERSECTION_TEST: if(   sLineVec1.LengthSquared() < SM_EFF_ZERO_SQ   // check for degeneracy
//                                                        || sLineVec2.LengthSquared() < SM_EFF_ZERO_SQ)  // check for degeneracy
//                                                       assume segment chords are perpendicular
//                                                     bOnlyOneIntersectionPossible = ( dMaxChange > m_d3dTolerance );
//                                                     // when endPoints are within tolerance of each other
//                                                     if (sSol.m_vStart.m_dSolutionValue < m_d3dTolerance)
/*******************************************************************//**
PURPOSE: The Global Solver is the basic mechanism used to search
   for tennative solutions to global problems.  It is a highly 
   adaptable object which can be subclassed and modified to suit
   many applications of global search problems.

NOTES: 
***********************************************************************/
class SM_EXPORT SmGlobalSolver
{
protected:
  // Data used to obtain answers
  SmSolverOperationType   m_eSolverOperation;         // specify behavior to be used by LocalSolver calls

  // convenience summaries of m_eSolverOperation used to control
  // processing within various derived SolveIt() and LocalSolve() functions.
  SmOperationCategory     m_eOperationCategory;       // oneof: SM_OPERATION_MINIMIZE = Some kind of minimization
                                                      //        SM_OPERATION_MAXIMIZE = Some kind of maximization
                                                      //        SM_OPERATION_SIGNED   = Signed operations (eg. SM_SO_SIGNED_ANGLE_MINIMIZE)
                                                      //        SM_OPERATION_OTHER    = All other kinds of operations

  SmBoolean               m_bProjectedOperation;      // TRUE for SM_SO_DIRECTED_MINIMIZE : SM_SO_SIGNED_DIRECTED_MINIMIZE:
                                                      //          SM_SO_DIRECTED_MAXIMIZE : SM_SO_SIGNED_ANGLE_MINIMIZE:   
                                                      //                                    SM_SO_SIGNED_PIVOT_MINIMIZE: 
                                                      //          SM_SO_PROJECTED_MINIMIZE: 
                                                      //          SM_SO_PROJECTED_MAXIMIZE: 
    
  // control which solutions are saved in AddSortedSolution()                                                 
  SmSolutionRequestedType m_eSolutionRequested;       // oneof: SM_SR_SINGLE = Only produce one solution - the best                     
                                                      //        SM_SR_ALL    = Find all solutions that satisfy the function to with     
                                                      //                       the given tolerance                                      
                                                      //        SM_SR_NODES  = Only find the nodes in the tree where solutions may exist

  ULONG                   m_lNumTrees;                // number of geometry trees to traverse, always 1 or 2
  ULONG                   m_lNumVariables;            // How many total variables in this solution.  This defines how
                                                      // many total parameters exist in m_vStart and/or m_vEnd.  For
                                                      // example, if we are itersecting a curve and a surface the number
                                                      // of variables is 3 - m_vStart[0] - curve parameter of the intersection
                                                      // m_vStart[1] - U parameter of the surface, m_vStart[2] - V parameter
                                                      // of the surface.  In most cases you will know the number of variables
                                                      // based on the types of input given. 
  SmTree                * m_apTrees[SM_GS_MAX_TREES]; // array of trees to traverse, sized:[m_lNumTrees]
  double                  m_d3dTolerance;             // 3d distance tolerance, used in many ways:
                                                      //  dist measure for coincidence, size used for near hits
                                                      //  on bounding boxes, size used to size vector lengths, ...
  double                  m_dBestAnswerSoFarSq;       // best scalar value found by solver to date
  double                  m_dAtDistance;              // distance used for case: SM_SO_AT_DISTANCE
                                                      // This case is used to find a pair of points on
                                                      // a pair of lines that are equal distant from one another.
  double                  m_dAtDistanceSq;            // the square of (m_dAtDistance) for convenience
  const SmVector3d      * m_cpOptVectors;             // Vectors used by some solvers 
                                                      // examples:
                                                      //  SM_SO_RAYFIRE:                     vec[0] = ray direction
                                                      //  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE: vec[0] = optimization direction to optimize
                                                      //  SM_SO_SIGNED_DIRECTED_MINIMIZE:    vec[1] = optimization direction vector 
                                                      //  SM_SO_DIRECTED_MINIMIZE:           vec[1] = optimization direction vector                                            
                                                      //  SM_SO_DIRECTED_MAXIMIZE:           vec[1] = optimization direction vector                                                  
                                                      //  SM_SO_SIGNED_ANGLE_MINIMIZE:       vec[1] = the reference vector           
                                                      //  SM_SO_PROJECTED_INTERSECT:         vec[0] = projection plane unit-normal 
                                                      //  SM_SO_ROTATED_PROJECTED_INTERSECT: vec[0] = pt on rotation axis
                                                      //                                     vec[1] = rotation axis unit-vector   
                                                      //                                     vec[2] = unit-XAxis of rotation plane
                                                      //  SM_SO_PERSPECTIVE_INTERSECT:       vec[0] = pt on view plane
                                                      //                                     vec[1] = view plane unit normal
                                                      //                                     vec[2] = eye pt               
                                                      //  SM_SO_SIGNED_PIVOT_MINIMIZE:       vec[1] = the pivot point          
                                                      //                                     vec[2] = reference direction
                                                                                                          
  // Resulting answers 
  SmSolutionArray       * m_pSolutions;               // array of sorted solutions

public:
  SmGlobalSolver();
  SmGlobalSolver(SmTree *pTree1, SmTree *pTree2, ULONG lNumVariables);
  SmGlobalSolver(SmTree *pTree2, ULONG lNumVariables);
  virtual ~SmGlobalSolver();

  //
  SmStatus SolveIt(SmSolverOperationType   eSolverOperation,
                   SmSolutionRequestedType eSolutionRequested,
                   double                  d3dTolerance,        // in: loaded into m_d3dTolerance
                   double                  dBestAnswerSoFarSq,
                   const SmVector3d      * cpOptVectors,
                   SmSolutionArray       & rSolutions);

  // simple data access
  SmSolverOperationType GetSolverOperationType()   { return(m_eSolverOperation) ; }
  SmVector3d            GetOptVector(ULONG iIndex) { return(m_cpOptVectors[iIndex]) ; }

  void SetSolverOperation  (SmSolverOperationType eSolverOp)        { m_eSolverOperation   = eSolverOp; }
  void SetSolutionRequested(SmSolutionRequestedType eSolReq)        { m_eSolutionRequested = eSolReq; }
  void Set3DTolerance      (double d3DTolerance)                    { m_d3dTolerance       = d3DTolerance; }
  void SetBestAnswerSoFarSq(double dBestAnswerSoFarSq)              { m_dBestAnswerSoFarSq = dBestAnswerSoFarSq; }
  void SetNumTrees         (ULONG lNumTrees)                        { m_lNumTrees          = lNumTrees; }
  void SetNumVariables     (ULONG lNumVariables)                    { m_lNumVariables      = lNumVariables; }
  void SetOperationCategory(SmOperationCategory eOperationCategory) { m_eOperationCategory = eOperationCategory; }
  void SetSolutions        (SmSolutionArray * pSolutions)           {  m_pSolutions        = pSolutions; }
  void SetAtDistance       (double dAtDistance)                     { m_dAtDistance   = dAtDistance; 
                                                                      m_dAtDistanceSq = dAtDistance*dAtDistance; 
                                                                    }

  // accumulate unique solutions with specified sort into SmGlobalSolver::m_pSolutions
  void AddSortedSolution(const SmSolution & crSolution,     
                         SmSortKeyType      eSortKey,
                         SmBoolean          bCombineContiguousRanges=TRUE);            
                                                              
protected:                                                       
  virtual SmStatus  SolveBranch            (SmTreeNode * apBranch[SM_GS_MAX_TREES]);
  virtual SmBoolean BranchMayContainAnswers(SmTreeNode * apBranch[SM_GS_MAX_TREES]); // cull Solve cases based on ObjBBox sizes, operation, and tolerances
  virtual SmBoolean AreReadyForLocalSolve  (SmTreeNode * apBranch[SM_GS_MAX_TREES]) const; // no derived implementations yet 
  virtual SmStatus  FindBestSubdivisionNode(SmTreeNode * apBranch[SM_GS_MAX_TREES],
                                            ULONG      & rlBestSubdivisionIndex,
                                            SmBoolean  & rbSwapTraversalOrder) const;
  virtual SmStatus  Subdivide              (SmTreeNode * apOriginal[SM_GS_MAX_TREES],      // no derived implementations yet
                                            SmTreeNode * apChildBranch1[SM_GS_MAX_TREES],
                                            SmTreeNode * apChildBranch2[SM_GS_MAX_TREES]);
  virtual SmStatus  LocalSolve             (SmTreeNode * apBranch[SM_GS_MAX_TREES], 
                                            SmBoolean  & rbNeedsMoreSubdivision);

  // return TRUE when two solutions are identical
  virtual SmSolutionPairType IdenticalSolutions(const SmSolution & crSol1,
                                                const SmSolution & crSol2,
                                                const SmSolution *&pPreferredSol) const;

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON_BASE(SmGlobalSolver, SmGlobalSolver_TYPE) ;

} ; // end class SmGlobalSolver

#endif // !__SMGLOBALSOLVER_H__


