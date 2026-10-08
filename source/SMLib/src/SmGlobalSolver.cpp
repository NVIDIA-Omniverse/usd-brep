// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmGlobalSolver.cpp
* PURPOSE: Source file for SmGlobalSolver methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmGlobalSolver.h>
#include <SmTree.h>
#include <SmCurveCache.h>
#include <SmExtent3d.h>
#include <SmGraphicsExtern.h>
#include <SmSolutionArray.h>
#include <SmCrvOnSurf.h>
#include <SmGeomUtility.h>
#include <math.h>                 // floor()

#ifdef SM_DEBUG_CODE
#include <SmSurface.h>         // for curve and surface dumps and draws
#include <SmCurve.h>
#endif

/*******************************************************************//**
PURPOSE: Constructor which nothing and initializes the
   global solver.

NOTES:
***********************************************************************/
SmGlobalSolver::SmGlobalSolver()
: m_eSolverOperation(SM_SO_INTERSECT),
  m_eOperationCategory(SM_OPERATION_OTHER),
  m_bProjectedOperation(FALSE),
  m_eSolutionRequested(SM_SR_ALL),
  m_lNumTrees(0),
  m_lNumVariables(0),
//m_apTrees[SM_GS_MAX_TREES] - set below
  m_d3dTolerance(SM_UNDEF_DOUBLE),
  m_dBestAnswerSoFarSq(0.0),
  m_dAtDistance(0.0),
  m_dAtDistanceSq(0.0),
  m_cpOptVectors(NULL),
  m_pSolutions(NULL)
{
  ULONG ii ;
  for(ii=0;ii<SM_GS_MAX_TREES;ii++)
    {
      m_apTrees[ii] = NULL ;
    }
} // end SmGlobalSolver::SmGlobalSolver default constructor

/*******************************************************************//**
PURPOSE: Constructor which takes one tree and initializes the
   global solver.

NOTES:
***********************************************************************/
SmGlobalSolver::SmGlobalSolver
  (SmTree *pTree,
   ULONG lNumVariables)
 : m_eSolverOperation(SM_SO_INTERSECT),
   m_eOperationCategory(SM_OPERATION_OTHER),
   m_bProjectedOperation(FALSE),
   m_eSolutionRequested(SM_SR_ALL),
   m_lNumTrees(1),
   m_lNumVariables(lNumVariables),
   m_dBestAnswerSoFarSq(0.0),
   m_dAtDistance(0.0),
   m_dAtDistanceSq(0.0)
{
    m_apTrees[0] = pTree;
    m_d3dTolerance = 0.0;
}

/*******************************************************************//**
PURPOSE: Constructor which takes two trees and initializes the
   global solver.

NOTES:
***********************************************************************/
SmGlobalSolver::SmGlobalSolver
  (SmTree *pTree1, SmTree *pTree2,
   ULONG lNumVariables)
 : m_eSolverOperation(SM_SO_INTERSECT),
   m_eOperationCategory(SM_OPERATION_OTHER),
   m_bProjectedOperation(FALSE),
   m_eSolutionRequested(SM_SR_ALL),
   m_lNumTrees(2),
   m_lNumVariables(lNumVariables),
   m_dBestAnswerSoFarSq(0.0),
   m_dAtDistance(0.0),
   m_dAtDistanceSq(0.0)
{
    m_apTrees[0] = pTree1;
    m_apTrees[1] = pTree2;
    m_d3dTolerance = 0.0;
}

/*******************************************************************//**
PURPOSE: Destructor for global solver.

NOTES:
***********************************************************************/
SmGlobalSolver::~SmGlobalSolver()
{}

/*******************************************************************//**
PURPOSE: Execute solver to get answers.

NOTES:
  The GlobalSolver traverses the subdivision trees for the target geometry.
  For every pair of leaf nodes that have intersecting bounding boxes it calls
  a Local Solver that actually finds solutions.  The behavior of each local
  solver is controlled by the eSolverOperation value. The accumulation of
  solutions in the rSolutions array is controlled by the value of
  eSolutionRequested.

  eSolverOperation = specifies the behavior to be executed by the Local Solver calls.

  eSolutionRequested = one of
    SM_SR_SINGLE, // Only produce one solution - the best
    SM_SR_ALL,    // Find all solutions that satisfy the function to within
                  // the given tolerance
    SM_SR_NODES   // Only find the nodes in the tree where solutions may exist


***********************************************************************/
SmStatus SmGlobalSolver::SolveIt
  (SmSolverOperationType   eSolverOperation,    // in : specify the solver problem
   SmSolutionRequestedType eSolutionRequested,  // in : one of SM_SR_SINGLE - get best solution
                                                //             SM_SR_ALL    - get all  solutions
                                                //             SM_SR_NODES  - find nodes in tree where solution may exist
   double                  d3dTolerance,        // in : Size distance to satisfy, loaded into m_d3dTolerance
   double                  dBestAnswerSoFarSq,  // in : Best scalar value found to date
   const SmVector3d      * cpOptVectors,        // in : Different vectors required for different solvers: examples are
                                                //  SM_SO_RAYFIRE                      : vec[0] = ray direction
                                                //  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE  : vec[0] = optimization direction to optimize for
                                                //  SM_SO_PROJECTED_...  and
                                                //  SM_SO_SIGNED_...                   : vec[0] = projection plane for all projected solvers
                                                //  SM_SO_SIGNED_DIRECTED_MINIMIZE and
                                                //  SM_SO_DIRECTED_MINIMIZE and
                                                //  SM_SO_DIRECTED_MAXIMIZE            : vec[1] = optimization direction vector for
                                                //  SM_SO_SIGNED_ANGLE_MINIMIZE        : vec[1] = the reference vector for
                                                //  SM_SO_SIGNED_PIVOT_MINIMIZE        : vec[1] = the pivot point for
                                                //  SM_SO_SIGNED_PIVOT_MINIMIZE        : vec[2] = reference direction for
   SmSolutionArray        & rSolutions)         // out: array of SmSolution objects
{
  // m_eOperationCategory value from eSolverOperation
  switch(eSolverOperation)
    {
      case SM_SO_SIGNED_DIRECTED_MINIMIZE   : // note: The ProjectedIntersect solvers don't use this feature - they
      case SM_SO_DIRECTED_MINIMIZE          : //       do their solves by projecting the curve to common planes and then 
      case SM_SO_PROJECTED_MINIMIZE         : //       solving for regular intersections.
      case SM_SO_SIGNED_ANGLE_MINIMIZE      : 
      case SM_SO_DIRECTED_MAXIMIZE          :  
      case SM_SO_PROJECTED_MAXIMIZE         : 
      case SM_SO_SIGNED_PIVOT_MINIMIZE      :   
      case SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE:                                                             
               m_bProjectedOperation = TRUE ;  
                 break ;

      default: m_bProjectedOperation = FALSE ; 
               SM_ASSERT(   eSolverOperation == SM_SO_MINIMIZE || eSolverOperation == SM_SO_ANGLE_MINIMIZE
                         || eSolverOperation == SM_SO_RAYFIRE  || eSolverOperation == SM_SO_MAXIMIZE                   
                         || eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                         
                         || eSolverOperation == SM_SO_INTERSECT             || eSolverOperation == SM_SO_INTERSECT_WIREFRAME
                         || eSolverOperation == SM_SO_INTERSECTION_TEST     || eSolverOperation == SM_SO_NORMALIZE
                         || eSolverOperation == SM_SO_AT_DISTANCE           || eSolverOperation == SM_SO_FIND
                         || eSolverOperation == SM_SO_PROJECTED_INTERSECT   || eSolverOperation == SM_SO_ROTATED_PROJECTED_INTERSECT
                         || eSolverOperation == SM_SO_PERSPECTIVE_INTERSECT || eSolverOperation == SM_SO_PROJECTED_TANGENCY
                         || eSolverOperation == SM_SO_SURFACE_PROJECTED_INTERSECT
                         || eSolverOperation == SM_SO_PROJECTED_TANGENT_THROUGH_POINT);
                break ;

    } // end m_bProjectedOperation classification 

  // m_eOperationCategory value from eSolverOperation
  switch (eSolverOperation)
    {
      case SM_SO_DIRECTED_MINIMIZE  : 
      case SM_SO_PROJECTED_MINIMIZE : 
      case SM_SO_MINIMIZE           : 
      case SM_SO_ANGLE_MINIMIZE     : 
      case SM_SO_RAYFIRE            :
                m_eOperationCategory  = SM_OPERATION_MINIMIZE;
                  break;

      case SM_SO_DIRECTED_MAXIMIZE  :  
      case SM_SO_PROJECTED_MAXIMIZE :
      case SM_SO_MAXIMIZE           : 
                m_eOperationCategory  = SM_OPERATION_MAXIMIZE;
                  break;

      case SM_SO_SIGNED_DIRECTED_MINIMIZE    : 
      case SM_SO_SIGNED_ANGLE_MINIMIZE       :
      case SM_SO_SIGNED_PIVOT_MINIMIZE       :
      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE : 
      case SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE :
                m_eOperationCategory  = SM_OPERATION_SIGNED;
                  break;

      default : m_eOperationCategory  = SM_OPERATION_OTHER;
                SM_ASSERT(   eSolverOperation == SM_SO_INTERSECT
                          || eSolverOperation == SM_SO_INTERSECT_WIREFRAME
                          || eSolverOperation == SM_SO_INTERSECTION_TEST
                          || eSolverOperation == SM_SO_NORMALIZE
                          || eSolverOperation == SM_SO_AT_DISTANCE
                          || eSolverOperation == SM_SO_FIND
                          || eSolverOperation == SM_SO_PROJECTED_INTERSECT
                          || eSolverOperation == SM_SO_ROTATED_PROJECTED_INTERSECT
                          || eSolverOperation == SM_SO_PERSPECTIVE_INTERSECT
                          || eSolverOperation == SM_SO_PROJECTED_TANGENCY
                          || eSolverOperation == SM_SO_SURFACE_PROJECTED_INTERSECT
                          || eSolverOperation == SM_SO_PROJECTED_TANGENT_THROUGH_POINT);
                break;
    } // end m_eOperationCategory classification

  // load input values into this SmGlobalSolver member values
  m_eSolverOperation   = eSolverOperation ;
  m_eSolutionRequested = eSolutionRequested ;
  m_dBestAnswerSoFarSq = dBestAnswerSoFarSq ;
  m_cpOptVectors       = cpOptVectors ;
  m_d3dTolerance       = d3dTolerance ;

  // special case dBestAnswerSoFarSq
  if (    m_dBestAnswerSoFarSq == SM_BIG_DOUBLE
      && (   eSolverOperation  == SM_SO_MAXIMIZE
          || eSolverOperation  == SM_SO_DIRECTED_MAXIMIZE))
    {
      m_dBestAnswerSoFarSq = 0.0;
    }

  // store pointer to output array
  m_pSolutions = &rSolutions ;

  // init start branches with tree top nodes
  SmTreeNode *sStartBranch[SM_GS_MAX_TREES] ;
  for (ULONG i=0; i<m_lNumTrees; i++)
    {
      sStartBranch[i] = m_apTrees[i]->GetTopNode() ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        { m_apTrees[i]->Dump() ;
          m_apTrees[i]->DrawNodeBoundingBoxes() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif
    } // end iter every tree branch - usually 2, sometimes 1, can be up to 3

  // execute the solver tree/tree iteration
  SER(SolveBranch(sStartBranch));

  // all done
  return SM_SUCCESS;

} // end SmGlobalSolver::SolveIt

/*******************************************************************//**
PURPOSE: Solves a subset of the global problem by solving for a
   given branch of a tree.

NOTES: This is a recursive method which traverses down the
   sub-division tree stored in apBranch[0]
   until it can find a leaf node set which may contain a solution
   at which time it calls SmGlobalSolver::LocalSolve
   on that leaf node.
***********************************************************************/
SmStatus SmGlobalSolver::SolveBranch
  (SmTreeNode * apBranch[SM_GS_MAX_TREES])
{
  // no more work - these nodes no longer contain possible answer
  if (!BranchMayContainAnswers(apBranch))
    { return SM_SUCCESS; }

  // check for leaf nodes
  if (AreReadyForLocalSolve(apBranch))
    {
      SmBoolean bNeedsMoreSubdivision;

#ifdef SM_DEBUG_CODE
static int iDebugMe = FALSE ;
      if(iDebugMe>0)
      {
          if(iDebugMe>1)
            { m_pSolutions->Dump() ; }

          const SmObject  *pObj1     = (apBranch[0]) ? apBranch[0]->GetOwnerObject() : NULL;
          const SmObject  *pObj2     = (apBranch[1]) ? apBranch[1]->GetOwnerObject() : NULL;
          const SmSurface *pSurface1 = (pObj1 && pObj1->IsKindOf(SmSurface_TYPE)) ? (const SmSurface *)pObj1 : NULL ;
          const SmSurface *pSurface2 = (pObj2 && pObj2->IsKindOf(SmSurface_TYPE)) ? (const SmSurface *)pObj2 : NULL ;
          const SmCurve   *pCurve1   = (pObj1 && pObj1->IsKindOf(SmCurve_TYPE)) ? (const SmCurve *)pObj1 : NULL ;
          const SmCurve   *pCurve2   = (pObj2 && pObj2->IsKindOf(SmCurve_TYPE)) ? (const SmCurve *)pObj2 : NULL ;
          sm_GraphicsLoop() ;

          if ( iDebugMe > 5 ) {
              if(apBranch[0]) apBranch[0]->Dump() ;
              if(apBranch[1]) apBranch[1]->Dump() ;

              smgfx_Erase() ;
              if ( iDebugMe > 10 )
              {
                  smgfx_SetLook(1,2, 0,0,1) ; if(pSurface1) pSurface1->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 0,1,0) ; if(pSurface2) pSurface2->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
              }
              else
              {
                  smgfx_SetLook(1,2, 0,0,1) ; if(pSurface1) pSurface1->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(2,3, 0,1,0) ; if(pSurface2) pSurface2->Draw() ; sm_GraphicsLoop() ;
              }
              smgfx_SetLook(1,2, 0,0,1) ; if(pCurve1)   pCurve1->Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; if(pCurve2)   pCurve2->Draw(NULL,TRUE) ; sm_GraphicsLoop() ;
          }

          smgfx_SetLook(1,2, 1,0,1) ; if(apBranch[0]) { apBranch[0]->DrawBoundingBox(); } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(apBranch[1]) { apBranch[1]->DrawBoundingBox(); } sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE

      // make the leafNode/leafNode call to Local solver.
      //  note: when working with spatial trees - geometry is
      //  stored in both leaf and internal nodes.
      //  bNeedsMoreSubdivision is set TRUE in such cases
      //  until all apBranch[i] entries map to leaf nodes.
      SER(LocalSolve(apBranch, bNeedsMoreSubdivision));

#ifdef SM_DEBUG_CODE
      if(iDebugMe>1)
        { m_pSolutions->Dump() ;

          // add solution image to graphics drawn above
          smgfx_SetLook(4,5, 1,0,0) ; m_pSolutions->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // done with this branch when LocalSolver is done
      if (!bNeedsMoreSubdivision)
        {
          return SM_SUCCESS;
        }
    } // end AreReadyForLocalSolve(apBranch) check

  // allocate two branch solverChildren nodes
  SmTreeNode* sChild1[SM_GS_MAX_TREES];
  SmTreeNode* sChild2[SM_GS_MAX_TREES];

  // set pointer arrays to NULL - okay to use smos_MemSet() on pointer arrays
  smos_MemSet(sChild1,0,SM_GS_MAX_TREES*sizeof(SmTreeNode*));
  smos_MemSet(sChild2,0,SM_GS_MAX_TREES*sizeof(SmTreeNode*));

  // get the children to this solverNode
  SER(Subdivide(apBranch, sChild1, sChild2));

  // recurse for each childNode
  SER(SolveBranch(sChild1));
  SER(SolveBranch(sChild2));

  // all done
  return SM_SUCCESS;

} // end SmGlobalSolver::SolveBranch

/*******************************************************************//**
PURPOSE: Determine if the given branch may contain answers.  In other
   words can we utilize existing information (i.e. bounding boxes) to
   eliminate the possibility of any answers in this branch.

NOTES:
***********************************************************************/
SmBoolean SmGlobalSolver::BranchMayContainAnswers
  (SmTreeNode * apBranch[SM_GS_MAX_TREES])
{
  // No natural optimization for single tree operations.
  // You'll have to do this by subclassing.
  if (m_lNumTrees==1) return TRUE;

#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smgfx_Erase();
      sm_GraphicsLoop();
      apBranch[0]->m_sBBox.Draw(NULL);
      apBranch[1]->m_sBBox.Draw(NULL);
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // tolerance note for each operation - Dist between Obj BBoxes are checked against m_d3dTolerance,and other constants to cull no solution branches 
  // case SM_SO_INTERSECT:
  // case SM_SO_INTERSECTION_TEST: AreDisjoint(BBox1+m_d3dTolerance,BBox2) => return(FALSE)
  //  case SM_SO_MINIMIZE: MinDistSq(BBox1,BBox2) - m_d3dTolerance > m_dBestAnswerSoFarSq => return(FALSE)
  //  case SM_SO_MAXIMIZE: MaxDistSq(BBox1,BBox2) + m_d3dTolerance < m_dBestAnswerSoFarSq => return(FALSE)
  //  case SM_SO_AT_DISTANCE: MaxDist(BBox1,BBox2) - m_d3dTolerance < smos_Sqrt(m_dAtDistanceSq) => return(FALSE)
  //                          MinDist(BBox1,BBox2) + m_d3dTolerance < smos_Sqrt(m_dAtDistanceSq) => return(FALSE) 
  // case SM_SO_PROJECTED_MINIMIZE:          (BoxRadius1+m_d3Tolerance + BoxRadius2)**2 TestedAgainst m_dBestAnswerSoFarSq
  // case SM_SO_PROJECTED_MAXIMIZE:          (BoxRadius1+m_d3Tolerance + BoxRadius2)**2 TestedAgainst m_dBestAnswerSoFarSq
  // case SM_SO_PROJECTED_INTERSECT:         (BoxRadius1+m_d3Tolerance + BoxRadius2)**2 TestedAgainst m_dBestAnswerSoFarSq
  // case SM_SO_PERSPECTIVE_INTERSECT:       (BoxRadius1+m_d3Tolerance + BoxRadius2)**2 TestedAgainst m_dBestAnswerSoFarSq
  // case SM_SO_ROTATED_PROJECTED_INTERSECT: (BoxRadius1+m_d3Tolerance + BoxRadius2)**2 TestedAgainst m_dBestAnswerSoFarSq
  //  case SM_SO_DIRECTED_MINIMIZE:          Dist(BoxSphereCenters)>SumRadius+SM_EFF_ZERO_SQRT => FALSE, No use of m_d3Tolerance (maybe a bug?)
  //  case SM_SO_SIGNED_DIRECTED_MINIMIZE:   Dist(BoxSphereCenters)>SumRadius+SM_EFF_ZERO_SQRT => FALSE, No use of m_d3Tolerance (maybe a bug?)
  //  case SM_SO_DIRECTED_MAXIMIZE:          Dist(BoxSphereCenters)>SumRadius+SM_EFF_ZERO_SQRT => FALSE, No use of m_d3Tolerance (maybe a bug?)
  //  case SM_SO_SIGNED_PIVOT_MINIMIZE:      Dist(ProjDistToPivot(BoxSphereCenters)) - m_d3dTolerance > dSumRadius => FALSE
  // case SM_SO_NORMALIZE:  No Culling - all cases return TRUE
  // case SM_SO_FIND:       No Culling - all cases return TRUE 
  // case SM_SO_RAYFIRE:   Dist(BBoxSphereCenter,Ray) > BBoxSphereRadius + md3dTolerance => return FALSE
  // case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE: Dist(BBoxSphereCenter,DirectedLine) > SumBBoxSphereRadius + md3dTolerance => return FALSE
  // case SM_SO_SURFACE_PROJECTED_INTERSECT: AreDisjoint(Expand(ProjBBox1,2*m_d3dTolerance),ProjBBox2)
  // SM_SO_PROJECTED_TANGENT_THROUGH_POINT:  No Culling - all cases return TRUE

  // for each kind of Solver operation - return either TRUE=There may be solutions in this branch or FALSE=No Solutions
  switch (m_eSolverOperation)
    {
      case SM_SO_INTERSECT:
      case SM_SO_INTERSECTION_TEST:
        {
          for (ULONG i=0; i+1<m_lNumTrees; i++)  // note: can't say m_lNumTrees-1
            {
              SmExtent3d sBBox = apBranch[i]->m_sBBox;
              sBBox.ExpandAbsolute(m_d3dTolerance);

              for (ULONG j=i+1; j<m_lNumTrees; j++)
                {
                  SmExtent3d *pBBox2 = &apBranch[j]->m_sBBox;
                  if (sBBox.AreDisjoint(*pBBox2))
                    {
                      return FALSE; // No answers in this branch
                    }
                }
            }
          return TRUE;  // possible answers in this branch
        }

      case SM_SO_MINIMIZE:
        {
            SM_ASSERT(m_lNumTrees == 2);
            double dMinDistSq = apBranch[0]->m_sBBox.MinimumDistanceSquared(apBranch[1]->m_sBBox);
            if (dMinDistSq > m_dBestAnswerSoFarSq)
              {
                if(smos_Sqrt(dMinDistSq) - m_d3dTolerance > smos_Sqrt(m_dBestAnswerSoFarSq) )
                  {
                    return FALSE; // no answers in this branch
                  }
              }
            return TRUE; // possible answers in this branch
        }

      case SM_SO_MAXIMIZE:
        {
            SM_ASSERT(m_lNumTrees == 2);
            double dMaxDistSq = apBranch[0]->m_sBBox.MaximumDistanceSquared(apBranch[1]->m_sBBox);
            if (dMaxDistSq < m_dBestAnswerSoFarSq)
              {
                if(smos_Sqrt(dMaxDistSq) + m_d3dTolerance < smos_Sqrt(m_dBestAnswerSoFarSq) )
                  {
                    return FALSE; // no answers in this branch
                  }
              }
            return TRUE; // possible answers in this branch
        }

      case SM_SO_AT_DISTANCE:
        {
            SM_ASSERT(m_lNumTrees == 2);
            double dMaxDistSq = apBranch[0]->m_sBBox.MaximumDistanceSquared(apBranch[1]->m_sBBox);
            if (dMaxDistSq < m_dAtDistanceSq)
              {
                if(smos_Sqrt(dMaxDistSq) - m_d3dTolerance < smos_Sqrt(m_dAtDistanceSq) )
                  {
                    return FALSE; // no answers in this branch
                  }
              }
            double dMinDistSq = apBranch[0]->m_sBBox.MinimumDistanceSquared(apBranch[1]->m_sBBox);
            if (dMinDistSq > m_dAtDistanceSq)
              {
                if(smos_Sqrt(dMinDistSq) + m_d3dTolerance > smos_Sqrt(m_dAtDistanceSq) )
                  {
                    return FALSE; // no answers in this branch
                  }
              }
            return TRUE; // may be answers
        }

      case SM_SO_PROJECTED_MINIMIZE:
      case SM_SO_PROJECTED_MAXIMIZE:
      case SM_SO_PROJECTED_INTERSECT:
      case SM_SO_ROTATED_PROJECTED_INTERSECT:
      case SM_SO_PERSPECTIVE_INTERSECT:
        {
          // get Sphere bounds for the branch bboxes
          SmPoint3d sSph1Center, sSph2Center;
          double    dSph1Radius=0.0, dSph2Radius=0.0;
          apBranch[0]->m_sBBox.ComputeSphereBound(sSph1Center,dSph1Radius);
          apBranch[1]->m_sBBox.ComputeSphereBound(sSph2Center,dSph2Radius);

          // Factor 3D tolerance by adding it to just one sphere radii.
          dSph1Radius = dSph1Radius + m_d3dTolerance;

          // project SphereCenters to common plane circles that can be quickly checked no intersections
          SmPoint3d sProjPnt1, sProjPnt2 ;
          if(m_eSolverOperation == SM_SO_ROTATED_PROJECTED_INTERSECT)
            {
              // m_cpOptVectors[1] = unitRotAxis, m_cpOptVectors[2] = unitXAxis
              SM_ASSERT( SM_IS_ZERO(m_cpOptVectors[1].Length() - 1.0) ) ;
              SM_ASSERT( SM_IS_ZERO(m_cpOptVectors[2].Length() - 1.0) ) ;
              SM_ASSERT( SM_IS_ZERO(m_cpOptVectors[1].Dot(m_cpOptVectors[2]))) ;

              // rotate project Sph1 and Sph2 CenterPts to same plane
              //  (rather than rotate Sph2Center to Sph1Center plane we rotate both to a common plane
              //   to avoid making all checks and spaecial cases if Sph1Center happens to be on the rotation axis)
              sProjPnt1 = sSph1Center.RotateProjectPointToPlane(m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;
              sProjPnt2 = sSph2Center.RotateProjectPointToPlane(m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;

            } // end SolerOperation == RotatedProjectedIntersect branch
          else if(m_eSolverOperation == SM_SO_PERSPECTIVE_INTERSECT)
            {
              // m_cpOptVectors[0] = point on view plane
              // m_cpOptVectors[1] = unit normal to view plane
              // m_cpOptVectors[2] = eye point
              SM_ASSERT( SM_IS_ZERO(m_cpOptVectors[1].Length() - 1.0) ) ;

              // perspective project Sph1 and Sph2 CenterPts to same plane
              sProjPnt1 = sSph1Center.PerspectiveProjectPointToPlane(m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;
              sProjPnt2 = sSph2Center.PerspectiveProjectPointToPlane(m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;

              // spheres project to ellipses - find max radius of sphere projections
              SmVector3d sEyeVec1 = sSph1Center - m_cpOptVectors[2] ;
              SmVector3d sDir1    = sEyeVec1 * m_cpOptVectors[1] ; // vec perp to eyevec and view norm
              sDir1.Unitize() ;
              SmVector3d sDir2    = sDir1 * sEyeVec1 ;             // vec perp to eyevec in view normal plane
              sDir2.Unitize() ;
              SmVector3d sRadVec ;
              double dNewRad1MaxSq = 0.0, dNewRadSq ;
              ULONG ii ;
              dSph1Radius -= m_d3dTolerance ;
              for(ii=0;ii<4;ii++)
                {
                  sRadVec   = sSph1Center + (  ii == 0 ?  sDir1 * dSph1Radius
                                             : ii == 1 ? -sDir1 * dSph1Radius
                                             : ii == 2 ?  sDir2 * dSph1Radius
                                             :           -sDir2 * dSph1Radius) ;
                  sRadVec   = sRadVec.PerspectiveProjectToPlane(sSph1Center, m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;
                  dNewRadSq = sRadVec.LengthSquared();
                  if(dNewRadSq > dNewRad1MaxSq) dNewRad1MaxSq = dNewRadSq ;
                }

              SmVector3d sEyeVec2 = sSph2Center - m_cpOptVectors[2] ;
              sDir1    = sEyeVec2 * m_cpOptVectors[1] ; // vec perp to eyevec and view norm
              sDir1.Unitize() ;
              sDir2    = sDir1 * sEyeVec2 ;             // vec perp to eyevec in view normal plane
              sDir2.Unitize() ;
              double dNewRad2MaxSq = 0.0 ;
              for(ii=0;ii<4;ii++)
                {
                  sRadVec   = sSph2Center + (  ii == 0 ?  sDir1 * dSph1Radius
                                             : ii == 1 ? -sDir1 * dSph1Radius
                                             : ii == 2 ?  sDir2 * dSph1Radius
                                             :           -sDir2 * dSph1Radius) ;
                  sRadVec   = sRadVec.PerspectiveProjectToPlane(sSph1Center, m_cpOptVectors[0], m_cpOptVectors[1], m_cpOptVectors[2]) ;
                  dNewRadSq = sRadVec.LengthSquared();
                  if(dNewRadSq > dNewRad2MaxSq) dNewRad2MaxSq = dNewRadSq ;
                }

              // arrive here when sProjPnt1 and sProjPnt2 are the projected centers of the spheres and
              //   dNewRad1Max and dNewRad2Max are the maximum radii of the sphere projections onto the viewing plane
              // update variables
              dSph1Radius = smos_Sqrt(dNewRad1MaxSq) + m_d3dTolerance ;
              dSph2Radius = smos_Sqrt(dNewRad2MaxSq) ;
            } // end SolerOperation == SM_SO_PERSPECTIVE_INTERSECT branch
          else
            {
              // project Sph2 CenterPt to same plane as Sph1 CenterPt
              SER(smgu_PointProjectToPlane(sSph2Center,        // in : target point
                                           sSph1Center,        // in : point on plane
                                           m_cpOptVectors[0],  // in : plane surface normal
                                           sProjPnt2));         // out: target point projected to plane
              sProjPnt1 = sSph1Center ;
            }  // end SolerOperation != RotatedProjectedIntersect branch

          // for each operation - see if spheres are properly positioned to exclude a possible solution
          switch(m_eSolverOperation)
            {
              case SM_SO_PROJECTED_INTERSECT:
              case SM_SO_ROTATED_PROJECTED_INTERSECT:
              case SM_SO_PERSPECTIVE_INTERSECT:
                {
                  double dMinProjDist =   sProjPnt1.DistanceBetween(sProjPnt2)
                                        - dSph1Radius
                                        - dSph2Radius;
                  if (dMinProjDist > 0.0)
                    {
                      return FALSE; // No answers in this branch
                    }
                } break ;

              case SM_SO_PROJECTED_MINIMIZE:
                {
                  double dMinProjDist =   sProjPnt1.DistanceBetween(sProjPnt2)
                                        - dSph1Radius
                                        - dSph2Radius;
                  if (   dMinProjDist > 0.0
                      && dMinProjDist * dMinProjDist > m_dBestAnswerSoFarSq)
                    {
                      return FALSE; // No answers in this branch
                    }
                } break ;

              case SM_SO_PROJECTED_MAXIMIZE:
                {
                  double dMaxProjDist =   sProjPnt1.DistanceBetween(sProjPnt2)
                                        + dSph1Radius
                                        + dSph2Radius;
                  if (dMaxProjDist * dMaxProjDist < m_dBestAnswerSoFarSq)
                    {
                      return FALSE; // No answers in this branch
                    }
                } break ;
              default:
                  break;
            } // end switch on m_eSolverOperation

          // If we were unable to eliminate using above sphere tests then
          // we just skip out here with a TRUE.
          return TRUE; // possible answers in this branch
        }


      // If we are doing a directed minimization we can use the bounding
      // box information to eliminate additional comparisons
      case SM_SO_DIRECTED_MINIMIZE:
      case SM_SO_SIGNED_DIRECTED_MINIMIZE:
      case SM_SO_DIRECTED_MAXIMIZE:
        {
            // This optimization can be done by creating a plane through the
            // center of one bounding box whose normal is perpendicular to the
            // direction vector and seeing if the distance to the
            // center of the other bounding box is greater than the sum of
            // the two radii.
            SmVector3d sPlaneNorm = m_cpOptVectors[1] * m_cpOptVectors[0];
            SmPoint3d  sSph1Cent,   sSph2Cent;
            double     dSph1Radius = 0.0, dSph2Radius = 0.0;
            apBranch[0]->m_sBBox.ComputeSphereBound(sSph1Cent,dSph1Radius);
            apBranch[1]->m_sBBox.ComputeSphereBound(sSph2Cent,dSph2Radius);

            SmPoint3d sProjPnt;
            SER(smgu_PointProjectToPlane(sSph2Cent,
                                         sSph1Cent,
                                         sPlaneNorm,
                                         sProjPnt));
            double dSumRadius = dSph1Radius + dSph2Radius;
            if (sProjPnt.DistanceBetweenSquared(sSph2Cent) >
                dSumRadius*dSumRadius+SM_EFF_ZERO_SQRT)
              {
                return FALSE; // These two nodes can not solve this directed minimization
              }

            // Now take projected point and project it again up to projection plane
            // as defined going through sSph1Center
            SER(smgu_PointProjectToPlane(sProjPnt,
                                         sSph1Cent,
                                         m_cpOptVectors[0],
                                         sProjPnt));

            if (m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE)
              {
                SmVector3d sVec = sProjPnt - sSph1Cent;
                double dDist = sVec.Length();
                if (sVec.Dot(m_cpOptVectors[1]) < 0)
                  {
                    dDist = - dDist;
                  }
                double dMinProjDist = dDist - dSph1Radius - dSph2Radius;
                if (dMinProjDist*smos_Fabs(dMinProjDist) > m_dBestAnswerSoFarSq)
                  {
                    return FALSE; // No answers in this branch
                  }
              }

            if (m_eSolverOperation == SM_SO_DIRECTED_MINIMIZE)
              {
                double dMinProjDist = sProjPnt.DistanceBetween(sSph1Cent) -
                    dSph1Radius - dSph2Radius;
                if (dMinProjDist > 0.0 && dMinProjDist*dMinProjDist > m_dBestAnswerSoFarSq)
                  {
                    return FALSE; // No answers in this branch
                  }
              }

            if (m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE)
              {
                double dMaxProjDist = sProjPnt.DistanceBetween(sSph1Cent) +
                    dSph1Radius + dSph2Radius;
                if (dMaxProjDist*dMaxProjDist < m_dBestAnswerSoFarSq)
                  {
                    return FALSE; // No answers in this branch
                  }
              }

            return TRUE; // possible answers in this branch
        }

      case SM_SO_SIGNED_PIVOT_MINIMIZE:
        {
            // First, find projected distance from pivot pt to each center of bounding spheres.
            // If the difference of those two distances is greater than the sum of
            // the two radii, we can determine that no solutions will come from these two nodes.
            // PlaneNorm: m_cpOptVectors[0];
            // Pivot: m_cpOptVectors[1];
            SmPoint3d sSph1Cent, sSph2Cent;
            double dSph1Radius = 0.0, dSph2Radius = 0.0;
            apBranch[0]->m_sBBox.ComputeSphereBound(sSph1Cent,dSph1Radius);
            apBranch[1]->m_sBBox.ComputeSphereBound(sSph2Cent,dSph2Radius);
            SmVector3d sVec1 = m_cpOptVectors[0]*(sSph1Cent-m_cpOptVectors[1])*m_cpOptVectors[0];
            SmVector3d sVec2 = m_cpOptVectors[0]*(sSph2Cent-m_cpOptVectors[1])*m_cpOptVectors[0];
            double dSumRadius = dSph1Radius + dSph2Radius;
            if(smos_Fabs(sVec1.Length() - sVec2.Length()) - m_d3dTolerance > dSumRadius)
              {
                return FALSE; // These two nodes can not solve this pivot minimization
              }

            return TRUE; // possible answers in this branch
        }

      case SM_SO_NORMALIZE:
        // No optimization here yet - should be added in virtual method
        // for lower level object.
        break;

      case SM_SO_FIND:
        // No optimization for plain searching - should be added as
        // a virtual method
        break;

      // Doing a ray firing - minimization of intersections
      // along a vector.
      case SM_SO_RAYFIRE:
        {
            SmVector3d sLinePnt = apBranch[0]->m_sBBox.GetMin();
            SmPoint3d sLineVec = m_cpOptVectors[0];
            SmPoint3d sSph1Cent;
            double dSph1Radius = 0.0;
            apBranch[1]->m_sBBox.ComputeSphereBound(sSph1Cent,dSph1Radius);
            double dParam = 0.0;
            SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sSph1Cent,dParam));
            SmPoint3d sClPnt = sLinePnt + dParam * sLineVec;
            double dDist = sClPnt.DistanceBetween(sSph1Cent);
            // See if sphere does not intersect the infinite line
            if (dDist > dSph1Radius + m_d3dTolerance)
              {
                return FALSE; // No answers in this branch
              }
            // See if we are on wrong side of ray start point
            if (dParam + dSph1Radius < 0.0)
              {
                return FALSE; // No answers in this branch
              }

            // See if we are further out on the ray than the best answer so
            // far.
            double dMinPossibleRayDistance = dParam - dSph1Radius - m_d3dTolerance;
            if (dMinPossibleRayDistance > 0.0)
              {
                if (dMinPossibleRayDistance*dMinPossibleRayDistance > m_dBestAnswerSoFarSq)
                  {
                    return FALSE; // No answers in this branch
                  }
              }
            // Do some additional tests to see if the ray does not intersect with the
            // box of the object.
            double dLeng = 2.0 * (smos_Fabs(dParam) + dSph1Radius);
            SmPoint3d sEndPnt = sLinePnt +  dLeng * sLineVec;
            SmPoint3d sStartPnt = sLinePnt - dLeng * sLineVec;
            SmExtent3d sRayBox(sStartPnt);
            sRayBox.AddPoint3d(sEndPnt);
            sRayBox.ExpandAbsolute(m_d3dTolerance);
            if (sRayBox.AreDisjoint(apBranch[1]->m_sBBox))
              {
                return FALSE; // No answers in this branch
              }
            return TRUE; // possible answers in this branch
        }
      // If we are doing a 3D directed minimization we can use the bounding
      // box information to eliminate additional comparisons
      case SM_SO_3D_SIGNED_DIRECTED_MINIMIZE:
        {
            SmPoint3d sSph1Cent, sSph2Cent;
            double dSph1Radius = 0.0, dSph2Radius = 0.0;
            apBranch[0]->m_sBBox.ComputeSphereBound(sSph1Cent,dSph1Radius);
            apBranch[1]->m_sBBox.ComputeSphereBound(sSph2Cent,dSph2Radius);
            SmVector3d sLinePnt = sSph1Cent;
            SmPoint3d sLineVec = m_cpOptVectors[0];
            double dParam;
            SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sSph2Cent,dParam));
            SmPoint3d sClPnt = sLinePnt + dParam * sLineVec;
            double dDist = sClPnt.DistanceBetween(sSph2Cent);
            double dSumRadius = dSph1Radius + dSph2Radius;
            // If two spheres don't intersect as projected along the vector
            // return FALSE.
            if (dDist > dSumRadius + m_d3dTolerance)
              {
                return FALSE; // No answers in this branch
              }

            SmVector3d sVec = sClPnt - sSph1Cent;
            double dDistTo = sVec.Length();
            if (sVec.Dot(m_cpOptVectors[0]) < 0)
              {
                dDistTo = - dDistTo;
              }
            double dMinDist = dDistTo - dSph1Radius - dSph2Radius - m_d3dTolerance;
            if (dMinDist*smos_Fabs(dMinDist) > m_dBestAnswerSoFarSq)
              {
                return FALSE; // No answers in this branch
              }

            return TRUE; // possible answers in this branch
        }

      case SM_SO_SURFACE_PROJECTED_INTERSECT:
        {
          // If the boxes actually intersect, then we can return True.
          // If they don't, we have to look at the surface offsets.
          SmExtent3d sBBox1 = apBranch[0]->m_sBBox;
          sBBox1.ExpandAbsolute(m_d3dTolerance);

          SmExtent3d sBBox2 = apBranch[1]->m_sBBox;

          double dMinDist = sBBox1.MinimumDistance( sBBox2 );
          // If the min dist is 0, then they overlap.
          if ( dMinDist < m_d3dTolerance )
            { return TRUE; }

          // Now we have to see about offsetting sBBox2 along the surface normal.
          // We can find a reasonable offset distance by comparing BBoxes.
          double dMaxDist = sBBox1.MaximumDistance( sBBox2 );

          // If we create a new box by offsetting the surface-curve points
          // along the normqal by both the min and max distances,
          // then we will catch any case that could 'intersect'.
          // But we have to sample the surface curve.
          const SmObject *pObj2 = (apBranch[1]) ? apBranch[1]->GetOwnerObject() : NULL;

          SmCrvOnSurf *pCOS = SM_CAST_PTR( SmCrvOnSurf, pObj2 );
          if ( pCOS == NULL )
            { return FALSE; }

          // Get the vector from the surface box to the 3d curve box.
          // We'll use that to decide which surface normal (+/-) to use.
          SmVector3d sDiffVec( sBBox1.GetMid() - sBBox2.GetMid() );

          SmBezierSpan *pBezSpan2 = (SmBezierSpan*)apBranch[1]->m_pData;
          if ( pBezSpan2 == NULL ) { return FALSE; }
          SmExtent1d sIvl2 = pBezSpan2->GetInterval();

          SmExtent3d sOffBox;
          SmPoint3d sSrfPt;
          SmVector3d sNorm;
          SmVector2d sUVPtDer[2];

          ULONG ii, lNumSamples = 10;
          double dFrac, dT, dNumSamples = lNumSamples;
          for ( ii = 0; ii <= lNumSamples; ii++ )
            {
              dFrac = ii / dNumSamples;
              dT = sIvl2.Evaluate( dFrac );
              pCOS->EvaluatePoint( dT, sSrfPt );
              pCOS->EvaluateSurfaceNormal( dT, FALSE, sUVPtDer, sNorm );
              if ( sNorm.Dot( sDiffVec ) < 0.0 )
                { sNorm *= -1.0; }

              sOffBox.AddPoint3d( sSrfPt + dMinDist * sNorm );
              sOffBox.AddPoint3d( sSrfPt + dMaxDist * sNorm );
            }

          // Expand offset box to account for sampling.
          sOffBox.ExpandAbsolute( 2*m_d3dTolerance );

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) 
            {
              if ( FALSE ) 
                {
                  const SmSurface * pSrf     = pCOS->GetBaseSurface();
                  const SmObject  * pObj1    = (apBranch[0]) ? apBranch[0]->GetOwnerObject() : NULL;
                  const SmCurve   * p3dCurve = SM_CAST_PTR( SmCurve, pObj1 );
                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,1,1 ); pSrf->DrawUV( 4,4 ); sm_GraphicsLoop();
                  smgfx_SetLook( 3,5, 0,1,0 ); if(p3dCurve) p3dCurve->Draw(); sm_GraphicsLoop(); 
                  smgfx_SetLook( 3,5, 0,0,1 ); pCOS->Draw(); sm_GraphicsLoop();
                }
              smgfx_SetLook( 2,4, 0,1,0 ); sBBox1.Draw();  sm_GraphicsLoop();
              smgfx_SetLook( 2,4, 0,0,1 ); sBBox2.Draw();  sm_GraphicsLoop();
              smgfx_SetLook( 2,4, 1,0,0 ); sOffBox.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          if ( sOffBox.AreDisjoint( sBBox1 ))
            { return FALSE; }

          return TRUE;  // possible answers in this branch
        }

      case SM_SO_PROJECTED_TANGENT_THROUGH_POINT:
        {
          // The point could be arbitrarily far from the curve, so no kind of
          // boxing will work.  We would have to evaluate the curve at various
          // points along the interval...
          return TRUE;
        }

      default:
        {
          ERR(SM_ERR);  // not handled yet
        }
    } // end switch on SolverOperation

  return TRUE;  // No special knowledge: can't rule out solutions in this branch.

} // end SmGlobalSolver::BranchMayContainAnswers

/*******************************************************************//**
PURPOSE: Determine if the current branch is ready to try for a local
    solver solution.

NOTES: Right now just test for all nodes being leaf nodes.
***********************************************************************/
SmBoolean SmGlobalSolver::AreReadyForLocalSolve(SmTreeNode * apBranch[SM_GS_MAX_TREES]) const
{
  for (ULONG i=0; i<m_lNumTrees; i++)
    {
      // If we have children then are not ready
      if (apBranch[i]->m_pChild1 != NULL)
        {
          // False for all tree types except for surfaces
          //  which can be true when their children are gwc???
          if (   apBranch[i]->m_eAuxDataType            != SM_AD_BEZIER_SURFACE
              || apBranch[i]->m_pChild1->m_eAuxDataType == SM_AD_BEZIER_SURFACE)
             { return FALSE; }
        }
    }
  return TRUE; // all must been leaf nodes
}


static ULONG sm_FindLargestNonLeaf(ULONG lNumTrees, SmTreeNode * apBranch[SM_GS_MAX_TREES])
{
    ULONG lFound = 99999;
    double dMaxSize = 0;
    for (ULONG i=0; i<lNumTrees; i++) {
        if (apBranch[i]->m_pChild1 == NULL) continue;
        if (apBranch[i]->m_eAuxDataType == SM_AD_BEZIER_SURFACE &&
            apBranch[i]->m_pChild1->m_eAuxDataType != SM_AD_BEZIER_SURFACE) {
            continue;
        }
        SmVector3d sVec = apBranch[i]->m_sBBox.GetMax() -
                apBranch[i]->m_sBBox.GetMin();
        double dSize = sVec.LengthSquared();
        if (dSize >= dMaxSize) {
            dMaxSize = dSize;
            lFound = i;
        }
    }
    SM_ASSERT (lFound != 99999);
    return lFound;
}


/*******************************************************************//**
PURPOSE: Find which node should be subdivided and see if we need to
    swap the traversal order to achieve more optimal minimization and
    maximization.

NOTES:
***********************************************************************/
SmStatus SmGlobalSolver::FindBestSubdivisionNode
 (SmTreeNode * apBranch[SM_GS_MAX_TREES],
  ULONG      & rlBestSubdivisionIndex,
  SmBoolean  & rbSwapTraversalOrder)
 const
{
    rbSwapTraversalOrder = FALSE;
    rlBestSubdivisionIndex = sm_FindLargestNonLeaf(m_lNumTrees,apBranch);
    // check for failure
    if( rlBestSubdivisionIndex == 99999 )
        return SM_ERR;

    if (m_eSolverOperation == SM_SO_RAYFIRE) {
        SmExtent3d *pSub1 = &apBranch[1]->m_pChild1->m_sBBox;
        SmExtent3d *pSub2 = &apBranch[1]->m_pChild2->m_sBBox;

        // Use mid point of box to determine which one is best choice
        SmPoint3d sPMid1 = pSub1->Evaluate(0.5,0.5,0.5);
        SmPoint3d sPMid2 = pSub2->Evaluate(0.5,0.5,0.5);

        SmVector3d sLinePnt = apBranch[0]->m_sBBox.GetMin();
        SmPoint3d sLineVec = m_cpOptVectors[0];

        double dParam1, dParam2;
        SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sPMid1,dParam1));
        SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sPMid2,dParam2));

        if (dParam2 < dParam1) {
            // Swap nodes in child branches
            rbSwapTraversalOrder = TRUE;
        }
        return SM_SUCCESS;
    }

    if (m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE) {
        SM_ASSERT(m_lNumTrees == 2);
        ULONG lOtherIndex = 1 - rlBestSubdivisionIndex;
        SmExtent3d *pOther = &apBranch[lOtherIndex]->m_sBBox;
        SmExtent3d *pSub1 = &apBranch[rlBestSubdivisionIndex]->m_pChild1->m_sBBox;
        SmExtent3d *pSub2 = &apBranch[rlBestSubdivisionIndex]->m_pChild2->m_sBBox;

        // Use mid point of box to determine which one is best choice
        SmPoint3d sPMid1 = pSub1->Evaluate(0.5,0.5,0.5);
        SmPoint3d sPMid2 = pSub2->Evaluate(0.5,0.5,0.5);

        SmVector3d sLinePnt = pOther->Evaluate(0.5,0.5,0.5);
        SmPoint3d sLineVec = m_cpOptVectors[0];

        double dParam1, dParam2;
        SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sPMid1,dParam1));
        SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sPMid2,dParam2));

        if (dParam2 < dParam1) {
            // Swap nodes in child branches
            rbSwapTraversalOrder = TRUE;
        }
        return SM_SUCCESS;
    }


    if (m_lNumTrees == 1) return SM_SUCCESS;

    // See if we can optimize based on distance between boxes

    switch (m_eSolverOperation) {

    case SM_SO_MINIMIZE:
    case SM_SO_DIRECTED_MINIMIZE:
    case SM_SO_SIGNED_DIRECTED_MINIMIZE:
    case SM_SO_PROJECTED_MINIMIZE:
        {
            SM_ASSERT(m_lNumTrees == 2);
            ULONG lOtherIndex = 1 - rlBestSubdivisionIndex;
            SmExtent3d *pOther = &apBranch[lOtherIndex]->m_sBBox;
            SmExtent3d *pSub1 = &apBranch[rlBestSubdivisionIndex]->m_pChild1->m_sBBox;
            SmExtent3d *pSub2 = &apBranch[rlBestSubdivisionIndex]->m_pChild2->m_sBBox;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
            if (bDebugMe) {
                if (FALSE) {
                    double dNewXCent = 0.0;    double dNewYCent = 0.0;    double dNewSize = 1.0;
                    smgfx_ZoomBox(dNewXCent,dNewYCent,dNewSize);
                }
                smgfx_SetColor(1,0,0); pOther->Draw(NULL); sm_GraphicsLoop() ;
                smgfx_SetColor(0,1,0); pSub1->Draw(NULL);  sm_GraphicsLoop() ;
                smgfx_SetColor(0,0,0); pSub2->Draw(NULL);  sm_GraphicsLoop() ;
                sm_GraphicsLoop() ;
            }
#endif
            // Use mid point of box to determine which one is best choice
            SmPoint3d sPMid1 = pSub1->Evaluate(0.5,0.5,0.5);
            SmPoint3d sPMid2 = pSub2->Evaluate(0.5,0.5,0.5);
            SmPoint3d sPOther = pOther->Evaluate(0.5,0.5,0.5);
            SmVector3d sV1 = sPMid1 - sPOther;
            SmVector3d sV2 = sPMid2 - sPOther;
            if (sV2.LengthSquared() < sV1.LengthSquared()) {
                // Swap nodes in child branches
                rbSwapTraversalOrder = TRUE;
            }
        }
        break;

    case SM_SO_MAXIMIZE:
    case SM_SO_DIRECTED_MAXIMIZE:
    case SM_SO_PROJECTED_MAXIMIZE:
        {
            SM_ASSERT(m_lNumTrees == 2);
            ULONG lOtherIndex = 1 - rlBestSubdivisionIndex;
            SmExtent3d *pOther = &apBranch[lOtherIndex]->m_sBBox;
            SmExtent3d *pSub1 = &apBranch[rlBestSubdivisionIndex]->m_pChild1->m_sBBox;
            SmExtent3d *pSub2 = &apBranch[rlBestSubdivisionIndex]->m_pChild2->m_sBBox;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
            if (bDebugMe) {
                smgfx_SetColor(1,0,0); pOther->Draw(NULL); sm_GraphicsLoop() ;
                smgfx_SetColor(0,1,0); pSub1->Draw(NULL);  sm_GraphicsLoop() ;
                smgfx_SetColor(0,0,0); pSub2->Draw(NULL);  sm_GraphicsLoop() ;
                sm_GraphicsLoop() ;
            }
#endif
            // Use mid point of box to determine which one is best choice
            SmPoint3d sPMid1 = pSub1->Evaluate(0.5,0.5,0.5);
            SmPoint3d sPMid2 = pSub2->Evaluate(0.5,0.5,0.5);
            SmPoint3d sPOther = pOther->Evaluate(0.5,0.5,0.5);
            SmVector3d sV1 = sPMid1 - sPOther;
            SmVector3d sV2 = sPMid2 - sPOther;
            if (sV2.LengthSquared() > sV1.LengthSquared()) {
                // Swap nodes in child branches
                rbSwapTraversalOrder = TRUE;
            }
        }
        break;

    default:
        break;

    }

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Subdivide a branch into two sub-branches.

NOTES: For now just try the one with the largest box first.
   If that node is a leaf continue on until find one which is able
   to be subdivided.
***********************************************************************/
SmStatus SmGlobalSolver::Subdivide
 (SmTreeNode * apOriginal[SM_GS_MAX_TREES],
  SmTreeNode * apChildBranch1[SM_GS_MAX_TREES],
  SmTreeNode * apChildBranch2[SM_GS_MAX_TREES])
{
    ULONG lBestSubdivisionIndex;
    SmBoolean bSwapTraversalOrder;
    SER(FindBestSubdivisionNode(apOriginal,lBestSubdivisionIndex,bSwapTraversalOrder));

    // Do default subdivision
    for (ULONG i=0; i<m_lNumTrees; i++) {
        apChildBranch1[i] = apOriginal[i];
        apChildBranch2[i] = apOriginal[i];
    }
    // Now make two subbranches take the children of the subdivision node.
    apChildBranch1[lBestSubdivisionIndex] =
        apOriginal[lBestSubdivisionIndex]->m_pChild1;
    apChildBranch2[lBestSubdivisionIndex] =
        apOriginal[lBestSubdivisionIndex]->m_pChild2;

    // If we need to swap the traversal order to take Child2 first
    if (bSwapTraversalOrder) {
        SmTreeNode *pTmp = apChildBranch1[lBestSubdivisionIndex];
        apChildBranch1[lBestSubdivisionIndex] = apChildBranch2[lBestSubdivisionIndex];
        apChildBranch2[lBestSubdivisionIndex] = pTmp;
    }


    return SM_SUCCESS;
} // end SmGlobalSolver::Subdivide

/*******************************************************************//**
PURPOSE: Once we have arrived here we try to do a local solve.  In this
   case where we have no contextual information, the best we can do is to
   register the tree nodes where we end up.  Most of the time this function
   will be overridden by subclassing SmGlobalSolver.

NOTES:
***********************************************************************/
SmStatus SmGlobalSolver::LocalSolve
  (SmTreeNode * apBranch[SM_GS_MAX_TREES],
   SmBoolean  & rbNeedsMoreSubdivision)
{
  SmSolution sSolution;
  sSolution.m_eSolutionType = SM_ST_NODES_ONLY;
  for (ULONG i=0; i<m_lNumTrees; i++)
    {
      sSolution.m_apNodes[i] = apBranch[i];
    }
  m_pSolutions->Add(sSolution);
  rbNeedsMoreSubdivision = FALSE;  // How can we with no context

  // all done
  return SM_SUCCESS;

} // end SmGlobalSolver::LocalSolve

/*******************************************************************//**
PURPOSE: Combine two solutions into one when appropriate

NOTES: Only combines solutions to the same objects,
   e.g. won't combine a vertex with a face solution.

   when bCombineContiguousRanges == FALSE won't combine contiguous range solutions
     but will continue to combine overlapping and contained interval solutions
***********************************************************************/
static SmBoolean sm_CombineMinMax          // rtn: TRUE = solutions combined
  (SmSolverOperationType eSolverOperation,   // in : Solver operation
   SmSolution & crMin,                       // in : 1st solution to compare
   SmSolution & crMax,                       // in : 2nd solution
   SmSolution & rCombinedResult,             // out: combined solution - one of modified crMin or crMax
   SmBoolean bCombineContiguousRanges)       // in : TRUE = Do combine contiguous intervals - needed for global intersector where
                                             //              solutions are found in small pieces and added together
                                             //      FALSE = Don't.  Used when building solutions from CurveClassifications
                                             //              where range solution is known to terminate on vertex or curve
                                             //              boundaries and where we don't want to lose those by joining
                                             //              two segments together.
{
  // when 1st solution is rangeOfValues
  if (crMin.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
    {
      // and 2nd solution is a point - crMax->Start to crMin
      if (crMax.m_eSolutionType == SM_ST_SINGLE_VALUE)
        {
          // get classification
          //    0 - matches single value or is inside of a range of values
          //   -1 - matches the minimum end of a range of values
          //   +1 - matches the maximum end of a range of values
          //   -2 - less than the minimum
          //   +2 - greater than the maximum
          //   +3 - there is an inconsistency in the relationship - no classification possible
          long lClass1 = crMin.ClassifySolutionEnd(crMax,TRUE);

          // don't combine incompatible solutions
          if(lClass1 == 3)
            {
              return FALSE ;
            }

          // merge points into ranges when appropriate
          if (   lClass1 == -1
              || lClass1 == 1
              || (lClass1 == 0 && eSolverOperation != SM_SO_INTERSECT))
            {
              rCombinedResult = crMin;
              return TRUE;
            }
          else if(lClass1 == 0 && eSolverOperation == SM_SO_INTERSECT)
            {
              // arrive here when inserting a intersection point solution into a range solution

              // when point solution is tight -
              // convert two solutions into two contiguous range solutions
              double dPtDev  = crMax.m_vStart.m_dSolutionValue ;
              double dMinDev = crMin.m_vStart.m_dSolutionValue ;
              double dMaxDev = crMin.m_vEnd.m_dSolutionValue ;

              // when point solution is better than range endPts
              if(   dPtDev < dMinDev * 100.0
                 || dPtDev < dMaxDev * 100.0)
                {
                  // change point solution into a range that starts at the point solution
                  crMax.m_eSolutionType = SM_ST_RANGE_OF_VALUES ;
                  crMax.m_vEnd          = crMin.m_vEnd ;

                  // change min solution into a range that stops at the point solution
                  crMin.m_vEnd          = crMax.m_vStart ;
                  return FALSE;
                }
              else // combine point into range
                {
                  rCombinedResult = crMin ;
                  return TRUE ;
                }
            }
        } // end MaxSolution is a point branch

      else // MaxSolution is a range
        {
          // classify Max Start and End against Min
          long lClass1 = crMin.ClassifySolutionEnd(crMax,TRUE);  // TRUE  = classify Max Start against Min interval
          long lClass2 = crMin.ClassifySolutionEnd(crMax,FALSE); // FALSE = classify Max End against Min interval

          // don't combine incompatible solutions
          if(lClass1 == 3 || lClass2 == 3)
            {
              return FALSE ;
            }

          // don't combine disjoint intervals
          if (   lClass1 >  1
              || lClass2 < -1)
            {
              return FALSE;
            }

          // don't combine contiguous intervals unless asked
          if(   bCombineContiguousRanges == FALSE
             && (   (   lClass1 == 1
                     && lClass2 == 2)
                 || (   lClass1 == -2
                     && lClass2 == -1)))
            {
              return FALSE;
            }

          // Max is contained within Min
          if (lClass1 == 0 && lClass2 == 0)
            {
              rCombinedResult = crMin;
              return TRUE;
            }

          // The remaining cases require the combination of the intervals

          // No: Splitting this Range solution results in spurious Vertices. [Reg_060124_...]
          //// Prevent from combining and losing a closer intersection
          //if (eSolverOperation == SM_SO_INTERSECT && lClass2 > 1)
          //  {
          //    double dStartDev = crMin.m_vStart.m_dSolutionValue;
          //    double dEndDev   = crMin.m_vEnd.m_dSolutionValue;
          //    if (   dStartDev > SM_EFF_ZERO
          //        && dStartDev > dEndDev * 100.0 )
          //      {
          //        return FALSE;
          //      }
          //    double dStartDev2 = crMax.m_vStart.m_dSolutionValue;
          //    double dEndDev2   = crMax.m_vEnd.m_dSolutionValue;
          //    if (   dEndDev2 > SM_EFF_ZERO
          //        && dEndDev2 > dStartDev2 * 100.0 )
          //      {
          //         return FALSE;
          //      }
          //  }

          // modify MinSolution
          rCombinedResult = crMin;
          if (lClass1 < -1)
            {
              SmSolutionEnd sTmp = crMax.m_vStart;
              rCombinedResult.m_vStart = sTmp;
            }
          if (lClass2 > 1)
            {
              SmSolutionEnd sTmp = crMax.m_vEnd;
              rCombinedResult.m_vEnd = sTmp;
            }
          return TRUE;
        } // end 2nd solution is a range of values branch
    } // end 1st solution is a range of values branch

  // The only remaining case to worry about is the case of min being a single
  // value which falls into the crMax interval.
  if (crMax.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
    {
      long lClass1 = crMax.ClassifySolutionEnd(crMin,TRUE);
      if (smos_Fabs(lClass1) <= 1)
        {
          rCombinedResult = crMax;
          return TRUE;
        }
      if (   lClass1 == -1
          || lClass1 ==  1
          || (lClass1 == 0 && eSolverOperation != SM_SO_INTERSECT))
        {
          rCombinedResult = crMax;
          return TRUE;
        }
      else if(lClass1 == 0 && eSolverOperation == SM_SO_INTERSECT)
        {
          // arrive here when inserting a intersection point solution into a range solution

          // when point solution is tight -
          // convert two solutions into two contiguous range solutions
          double dPtDev  = crMin.m_vStart.m_dSolutionValue ;
          double dMinDev = crMax.m_vStart.m_dSolutionValue ;
          double dMaxDev = crMax.m_vEnd.m_dSolutionValue ;

          // when point solution is better than range endPts
          if(   dPtDev < dMinDev * 100.0
             || dPtDev < dMaxDev * 100.0)
            {
              // change point solution into a range that starts at the point solution
              crMin.m_eSolutionType = SM_ST_RANGE_OF_VALUES ;
              crMin.m_vEnd          = crMax.m_vEnd ;

              // change min solution into a range that stops at the point solution
              crMax.m_vEnd          = crMin.m_vStart ;
              return FALSE;
            }
          else // combine point into range
            {
              rCombinedResult = crMax ;
              return TRUE ;
            }
        }
    } // end Max range solution check

  // all done - no combining
  return FALSE;

} // end sm_CombineMinMax

/*******************************************************************//**
PURPOSE: Add a solution to the SmGlobalSolver::m_pSolutions list
            using an insertion sort and a given sort key.

METHOD ---
  1. Don't add solutions to the solution array when
     a. SolutionValue == SM_BIG_DOUBLE
     b. duplicate solutions
     c. Not best solutions when looking for best solution.
  2. Add Solution into array in its sorted position
  3. After adding an ordered solution
     - try combining all solutions
        o. union overlapping and contiguous ranges
        o. absorb point solutions within range solutions when
               tolerance is less than range end-points.
        o. split range solutions into pairs when adding point
            solution that has tighter tolerance than range end point
            solitions.
  4. When looking for a single answer or a best answer -
     just keep that best answer.
***********************************************************************/
void SmGlobalSolver::AddSortedSolution  // eff: Add solution to m_pSolutions if appropriate
  (const SmSolution & crSolution,       // in : target solution
   SmSortKeyType eSortKey,              // in : sort by one of
                                        //      SM_SK_BY_SOLUTION_VALUE       = sort by value
                                        //      SM_SK_BY_FIRST_PARAMETER      = sort by first parameter
                                        //      SM_SK_BY_VALUE_WITH_PARAMETER = Sort by value - using parameters to resolve ties
                                        //      SM_SK_BY_FIRST_TWO_PARAMETERS = Sort by first two parameters - second used to resolve ties
                                        //      SM_SK_BY_ALL_PARAMETERS       = Use all parameters up to point of resolving ties
   SmBoolean bCombineContiguousRanges)  // in : TRUE = Do combine contiguous intervals - needed for global intersector where
                                        //              solutions are found in small pieces and added together
                                        //      FALSE = Don't.  Used when building solutions from CurveClassifications
                                        //              where range solution is known to terminate on vertex or curve
                                        //              boundaries and where we don't want to lose those by joining
                                        //              two segments together.
                                        //      default:[TRUE]
{
  // skip BigSolutions
  if (crSolution.m_vStart.m_dSolutionValue == SM_BIG_DOUBLE)
      return;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smos_WriteBuffer(_T("Current Solution Array ------------------------------\n"));
      m_pSolutions->Dump();
      smos_WriteBuffer(_T("\n\nSolution Being Considered +++++++++++++++++++++++++++++++++\n"));
      crSolution.Dump();
    }
#ifdef GWC
  SM_ASSERT(   (   crSolution.m_lNumObjects  != 0
                && crSolution.m_apObjects[0] != NULL)
            || (   crSolution.m_lNumObjects   > 1      // GlobalLineIntersect - has a legal NULL object value for m_apObjects[0]
                && crSolution.m_apObjects[1] != NULL)) ;
#endif // GWC
#endif // SM_DEBUG_CODE

  //

  // locals
  double dSolValue  = crSolution.m_vStart.m_dSolutionValue ;
  double dThisSolSq = dSolValue * dSolValue  ;
  double dBestAnsWithTol, dSign ;
  SmBoolean bBetterThanBest = FALSE ;
  SmBoolean bFarFromBest    = FALSE ;

  // Note, using the squared value for BestAnswer (to avoid sqrt operations?)
  // causes several extra sqrt operations below, so do only one here:
  double dBestSoFar = smos_Sqrt( smos_Fabs( m_dBestAnswerSoFarSq ));

  // When looking for a best solution - compare this solution to best
  if(m_eOperationCategory == SM_OPERATION_SIGNED)
    {
      // remember if solution is better than the current best
      bBetterThanBest = (smos_Sgn(dSolValue) * dThisSolSq) < m_dBestAnswerSoFarSq ;

      // remember if solution is worse than current best + tolerance
      dSign           = (m_dBestAnswerSoFarSq < 0.0) ? -1.0 : 1.0 ;
      dBestAnsWithTol = dSign * dBestSoFar + m_d3dTolerance ;
      bFarFromBest    = dSolValue > dBestAnsWithTol;
    }

  // remember Best solution values - cull solutions not within tol of best solution
  if (bBetterThanBest)   { m_dBestAnswerSoFarSq = dThisSolSq;
                         }
  else if (bFarFromBest) { return;
                         }

  switch(m_eSolverOperation)
    {
      case SM_SO_MINIMIZE          :
      case SM_SO_DIRECTED_MINIMIZE :
      case SM_SO_PROJECTED_MINIMIZE:
      case SM_SO_RAYFIRE           :

        bBetterThanBest = dThisSolSq < m_dBestAnswerSoFarSq ;
        dBestAnsWithTol = dBestSoFar + m_d3dTolerance;
        bFarFromBest    = dSolValue  > dBestAnsWithTol;
        // Do not do this.  Example: two valid solutions, 1e-30 and 1e-14.
        //bFarFromBest |= dBestSoFar > 0 && dSolValue > dBestSoFar * 1000;
        break ;

      case SM_SO_MAXIMIZE          :
      case SM_SO_DIRECTED_MAXIMIZE :
      case SM_SO_PROJECTED_MAXIMIZE:
        bBetterThanBest = dThisSolSq > m_dBestAnswerSoFarSq ;
        dBestAnsWithTol = dBestSoFar - m_d3dTolerance;
        bFarFromBest    = dSolValue < dBestAnsWithTol;
        //bFarFromBest |= dSolValue > 0 && dSolValue < dBestSoFar / 1000;
        break ;

      default:
        break ;
    } // end switch on m_eOperationCategory

  // remember Best solution values - cull solutions not within tol of best solution
  if (bBetterThanBest)   { m_dBestAnswerSoFarSq = dThisSolSq;
                         }
  else if (bFarFromBest) { return;
                         }

  // Now do insertion sort
  SmBoolean bFoundPlace = FALSE;
  ULONG lInsertionIndex = m_pSolutions->GetSize();

  // for every current solution
  ULONG ii, lNumSols = m_pSolutions->GetSize();
  for (ii=0; ii<lNumSols; ii++)
    {
      SmSolution & rTestGS = (*m_pSolutions)[ii];
      const SmSolution *pPreferredSol = NULL ;

      // don't add duplicate solutions (SolValues within ZeroSqrt and ParamValues within ScaledZero)

      if (SM_SP_IDENTICAL == IdenticalSolutions(crSolution, rTestGS, pPreferredSol))
        {
          // No: don't swap in crSolution if it's better.
          // There could be other reasons why the original is preferred.
          // Example, if the point is both a pole and a seam,
          // then points on the ends of the degenerate domain are preferred.
          // [B98; B115; my_test_csi_analy()]
          // But unfortunately, that causes other problems.
          // And since at this level we do not have enough information
          // to know that it might be a pole/seam or whatever, we have to
          // leave this in and deal with it at a higher level of the code.
          // [B125]

          // save the preferred solution if possible, else better solution - improve tolerances
          if(pPreferredSol != NULL)
            {
              rTestGS = *pPreferredSol ;
            }
          else if( fabs(crSolution.m_vStart.m_dSolutionValue) < fabs(rTestGS.m_vStart.m_dSolutionValue))
            {
              rTestGS = crSolution ;
            } // end new solution better than the stored solution check

          return;
        } // end identical solutions check

      //
      switch (eSortKey)
        {
          case SM_SK_BY_SOLUTION_VALUE:
              if (   m_eSolverOperation == SM_SO_MAXIMIZE
                  || m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE
                  || m_eSolverOperation == SM_SO_PROJECTED_MAXIMIZE)
                {
                  // Best answer is biggest
                  if (rTestGS.m_vStart.m_dSolutionValue < crSolution.m_vStart.m_dSolutionValue)
                    {
                      lInsertionIndex = ii;
                      bFoundPlace = TRUE;
                    }
                }
              else // Best answer is the smallest
                {
                  if (rTestGS.m_vStart.m_dSolutionValue > crSolution.m_vStart.m_dSolutionValue)
                    {
                      lInsertionIndex = ii;
                      bFoundPlace = TRUE;
                    }
                }
              break;

          case SM_SK_BY_FIRST_TWO_PARAMETERS:

              if (rTestGS.m_vStart[0] > crSolution.m_vStart[0])
                {
                  if (rTestGS.m_vStart[1] > crSolution.m_vStart[1])
                    {
                      lInsertionIndex = ii;
                      bFoundPlace = TRUE;
                    }
                }
              break;

          case SM_SK_BY_FIRST_PARAMETER:

              if (rTestGS.m_vStart[0] > crSolution.m_vStart[0])
                {
                  lInsertionIndex = ii;
                  bFoundPlace = TRUE;
                }

              // Do additional sorting based on intervals not just start and end values
              // This should put single values in front of intervals with same start
              // This will help us to process closed curves better
              {
                double dScale = (1.0 + rTestGS.m_vStart[0]) * SM_EFF_ZERO;
                if (smos_Fabs(rTestGS.m_vStart[0]-crSolution.m_vStart[0]) < dScale )
                  {
                    if (rTestGS.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
                      {
                        if (crSolution.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
                          {
                            // This does actually happen when a curve is coincident
                            // with the seam of a surface
                          }
                        lInsertionIndex = ii;
                        bFoundPlace = TRUE;
                      }
                  }
              }
              break;

          case SM_SK_BY_VALUE_WITH_PARAMETER:
              // when values are different - sort by value else sort by parameter
              if (!SM_IS_ZERO(rTestGS.m_vStart.m_dSolutionValue-crSolution.m_vStart.m_dSolutionValue))
                {
                  // Just use standard by value sorting
                  if (   m_eSolverOperation == SM_SO_MAXIMIZE
                      || m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE
                      || m_eSolverOperation == SM_SO_PROJECTED_MAXIMIZE)
                    {  // Best answer is biggest
                      if (rTestGS.m_vStart.m_dSolutionValue < crSolution.m_vStart.m_dSolutionValue)
                        {
                          lInsertionIndex = ii;
                          bFoundPlace = TRUE;
                        }
                    }
                  else // best answer is smallest
                    {
                      if (rTestGS.m_vStart.m_dSolutionValue > crSolution.m_vStart.m_dSolutionValue)
                        {
                          lInsertionIndex = ii;
                          bFoundPlace = TRUE;
                        }
                    }
                  break; // Only break if solution values are distinct otherwise sort by parameters
               } // end values are NOT the same check

             // else fall through to next case and sort by parameters

          case SM_SK_BY_ALL_PARAMETERS:
              {
                // The following should sort consecutive things together even if there
                // is tangency and periodic cases.  Basically it says that if a tie
                // exists then look at the next parameter to settle the deal.
                SmBoolean bFoundOne = FALSE;
                for (ULONG j=0; j<rTestGS.m_lNumVariables; j++)
                  {
                    // Take default answer for this parameter
                    if (!bFoundOne)
                      { // keep first answer
                        if (rTestGS.m_vStart[j] > crSolution.m_vStart[j])
                          {
                            bFoundOne = TRUE;
                            lInsertionIndex = ii;
                            bFoundPlace = TRUE;
                          }
                        else if (rTestGS.m_vStart[j] < crSolution.m_vStart[j])
                          {
                            bFoundOne = TRUE;
                            bFoundPlace = FALSE;
                          }
                      }

                    // This tol is bigger than the IdenticalSolutions() tolerance.
                    //  All that means is that the final array may not be sorted lowest.
                    //  to highest for solutions within this tolerance distance
                    // Consider: making the IdenticalSolutions() tolerance the same as this one.
                    // GWC TODO: after current work is running - change tol in  IdenticalSolutions()
                    //           and test in prog_test
                    double dScale = (1.0 + rTestGS.m_vStart[j]) * SM_EFF_ZERO_SQRT;
                    if (smos_Fabs(rTestGS.m_vStart[j]-crSolution.m_vStart[j]) > dScale)
                      {
                        // If answers are close to being same continue looking until you find
                        // an answer with acceptable deviation or until we have compared all of them
                        break;
                      }

                    // If we make it here then we have a tie so far.
                    // Look to see if the next one exists differentiates the answers
                    if (j+1 < rTestGS.m_lNumVariables)
                      {
                        dScale = (1.0 + rTestGS.m_vStart[j+1]) * SM_EFF_ZERO_SQRT;
                        if (smos_Fabs(rTestGS.m_vStart[j+1]-crSolution.m_vStart[j+1]) > dScale)
                          {
                            if (rTestGS.m_vStart[j+1] > crSolution.m_vStart[j+1])
                              {
                                // answer differentiates and should be inserted
                                lInsertionIndex = ii;
                                bFoundPlace = TRUE;
                                break;
                              }
                            else
                              { // answer differentiates and should not be inserted
                                bFoundPlace = FALSE;
                                break;
                              }
                          }
                      }
                  } // for
              }
              break;

          default:
              ERR(SM_ERR);
        } // Switch

      // If found an answer get out of loop
      if (bFoundPlace) break;

    } // end iter every solution seeking new solution place

  if (bFoundPlace) { // Insert it
                     m_pSolutions->InsertAt(lInsertionIndex,crSolution,1);
                   }
  else             { // Just add it to the end
                     m_pSolutions->Add(crSolution);
                   }

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      m_pSolutions->Dump();
    }
#endif

  // Once done go back and combine sequential solutions
  // Note that because things are not necessarily sequential
  // we have to compare every thing in the list to every other
  // thing in the list.
  SmBoolean bDone = FALSE;
  while (   !bDone
         && m_pSolutions->GetSize() < 100
         && m_pSolutions->GetSize() > 0)
    {
      bDone = TRUE;

      // For every solution
      for (ULONG k=0; k+1<m_pSolutions->GetSize(); k++)  // note: can't say m_pSolutions->GetSize()-1
        {
          const SmSolution & rMin = (*m_pSolutions)[k];

          // for all other solutions
          for (ULONG j=k+1; j<m_pSolutions->GetSize(); j++)
            {
              const SmSolution & rMax = (*m_pSolutions)[j];

              // try to combine the solutions.
              //   contiguous and overlapping intervals get unioned.
              //   points contained in intervals get absorbed.
              //   disjoint intervals remain disjoint.
              //   point solutions remain distinct.
              SmSolution sCombined;
              if (sm_CombineMinMax(m_eSolverOperation,(SmSolution &)rMin,(SmSolution &)rMax,sCombined,bCombineContiguousRanges))
                {
                  m_pSolutions->SetAt(k,sCombined);
                  m_pSolutions->RemoveAt(j,1);
                  bDone = FALSE;
                  break;
                }
            } // end iter every other solution

          // start over after combining solutions
          if (!bDone) break;

        } // end iter every solution
    } // while not done

  // If requesting only a single answer just keep the best
  if (m_eSolutionRequested == SM_SR_SINGLE)
    {
      // This assumes that we sorted by a meaningful
      // value and that the first answer is the best.
      m_pSolutions->SetSize(1);
    }

  // If we are minimizing or maximizing only keep those
  // solutions which are within tolerance of the best.
  if (   m_eSolverOperation == SM_SO_MINIMIZE
      || m_eSolverOperation == SM_SO_MAXIMIZE
      || m_eSolverOperation == SM_SO_RAYFIRE
      || m_eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
      || m_eSolverOperation == SM_SO_DIRECTED_MINIMIZE
      || m_eSolverOperation == SM_SO_DIRECTED_MAXIMIZE
      || m_eSolverOperation == SM_SO_PROJECTED_MINIMIZE
      || m_eSolverOperation == SM_SO_PROJECTED_MAXIMIZE
      || m_eSolverOperation == SM_SO_SIGNED_DIRECTED_MINIMIZE
      || m_eSolverOperation == SM_SO_SIGNED_PIVOT_MINIMIZE)
    {
      const SmSolution & rFirst = (*m_pSolutions)[0];
      SmSolution       * aData  = m_pSolutions->GetDataArray();
      SmSolution       * pLast  = &aData[m_pSolutions->GetSize()-1];

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          m_pSolutions->Dump();
        }
#endif // SM_DEBUG_CODE

      double dThisTol = m_d3dTolerance ;

      // This is a key location for the correct handling of tolerances.
      //  We'll have to revisit this bit of code and decide on the correct
      //  behavior given our model of tolerances for every operation type.  For
      //  now we'll just address the SM_SO_RAYFIRE operation.  This compare is deciding if
      //  two points on two different geometries are the same point given
      //  tolerances.  To do this we need to add together the tolerances
      //  of the associated Topology Objects that have been intersected by the ray.
      //  Even for this case there remains a question.  Should the locations of
      //  the points on topology object be compared, or the locations of the
      //  points on the ray.  To match old behavior, we check the distance between
      //  intersection points on the ray.  However, it's unclear which
      //  choice is the correct one here.
      if( m_eSolverOperation == SM_SO_RAYFIRE )
        {
          double dFirstTol = rFirst.GetObjectTolerance(0) ;
          double dLastTol  = pLast->GetObjectTolerance(0) ;
          dThisTol = smos_Max(m_d3dTolerance, dFirstTol + dLastTol) ;
        }

      while (   m_pSolutions->GetSize() > 1
             &&   smos_Fabs(rFirst.m_vStart.m_dSolutionValue - pLast->m_vStart.m_dSolutionValue)
                > dThisTol)
        {
          m_pSolutions->SetSize(m_pSolutions->GetSize()-1);
          pLast = &aData[m_pSolutions->GetSize()-1];
        }
    }

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      m_pSolutions->Dump();
    }
#endif // SM_DEBUG_CODE

} // end SmGlobalSolver::AddSortedSolution

/*******************************************************************//**
PURPOSE: Test to see if two solutions are identical.

NOTES: This
   prevents us from adding more then one solution which is the
   same.

   Note: the default definition of an identical solution is
         that the number of Objects, number of values, and solution types
         are the same and that the parameter values for point solutions
         and the parameter values at both ends of a range solution are the same.

   Note: individual solvers may override this
         method if they wish to change the way IdenticalSolutions
         are determined.  Mostly they look for multiple solutions
         on surface singularity boundaries watching out for seams.
***********************************************************************/
SmSolutionPairType SmGlobalSolver::IdenticalSolutions // rtn: oneof SM_SP_IDENTICAL
                                                      //            SM_SP_DIFFCOUNTS
                                                      //            SM_SP_DIFFOBJECTS
                                                      //            SM_SP_DIFFPARAMETERS
  (const SmSolution & crSol1,                         // in : 1st target solution
   const SmSolution & crSol2,                         // in : 2nd target solution
   const SmSolution *&pPreferredSol)                  // out: When identical, set to the preferred solution or NULL for none.
                                                      //      NULL to ignore. default:[NULL]
  const
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      smos_WriteBuffer(_T("\n\nSolution 1 +++++++++++++++++++++++++++++++++\n"));
      crSol1.Dump();
      smos_WriteBuffer(_T("Solution 2 --------------------------------\n"));
      crSol2.Dump();
    }
#endif
  // init output
  pPreferredSol = NULL ;

  // different ObjectCounts, VariableCounts, SolutionValues, SolutionTypes
  if (   (crSol1.m_lNumObjects   != crSol2.m_lNumObjects)
      || (crSol1.m_lNumVariables != crSol2.m_lNumVariables)
      || (crSol1.m_eSolutionType != crSol2.m_eSolutionType)
      || (SM_EFF_ZERO_SQRT < smos_Fabs(  crSol1.m_vStart.m_dSolutionValue
                                       - crSol2.m_vStart.m_dSolutionValue)))
    {
      return SM_SP_DIFFCOUNTS;
    }

  // unequal ObjectPointers
  for (ULONG j=0; j<crSol1.m_lNumObjects; j++)
    {
      if (crSol1.m_apObjects[j] != crSol2.m_apObjects[j])
        {
          return SM_SP_DIFFOBJECTS;
        }
    }

  SmExtent1d sIvl1, sIvl2 ;

  // ParameterValues off by more than ScaledZero or more than 1/10000th of the parameter length
  for (ULONG i=0; i<crSol1.m_lNumVariables; i++)
    {
      SmStatus sRtn1 = crSol1.GetParameterInterval(i, sIvl1) ;
      SmStatus sRtn2 = crSol2.GetParameterInterval(i, sIvl2) ;
      double dLength1 = (sRtn1 == SM_SUCCESS) ? .0001 * sIvl1.GetLength() : 0.0 ;
      double dLength2 = (sRtn2 == SM_SUCCESS) ? .0001 * sIvl2.GetLength() : 0.0 ;

      // (Note, SM_EFF_ZERO is too small to use here.)
      // [my_test_surface_curve_intersect()]
      // GWC TODO: after current work is running - change tol in here from 100*SM_EFF_ZERO to SM_EFF_ZERO_SQRT
      //           and test in prog_test
      double dScaledZero = smos_3Max(smos_Fabs(1.0 + smos_Fabs(crSol1.m_vStart[i])) * 100*SM_EFF_ZERO,
                                     dLength1,
                                     dLength2) ;

      if (   (   smos_Fabs(crSol1.m_vStart[i]-crSol2.m_vStart[i]) > dScaledZero)
          || (   crSol1.m_eSolutionType == SM_ST_RANGE_OF_VALUES
              && smos_Fabs(crSol1.m_vEnd[i]-crSol2.m_vEnd[i]) > dScaledZero))
        {
          return SM_SP_DIFFPARAMETERS;
        }
    }

  // All tests passed - identical Solutions

  // prefer the solution with the tighter solution
  pPreferredSol =   (crSol1.m_vStart.m_dSolutionValue < crSol2.m_vStart.m_dSolutionValue)
                  ? &crSol1
                  : &crSol2 ;

  return SM_SP_IDENTICAL;

} // end SmGlobalSolver::IdenticalSolutions

/*******************************************************************//**
PURPOSE: Pretty print

NOTES:
***********************************************************************/
void SmGlobalSolver::Dump() const
{
  // TCHAR sBuff[SM_TBLOCK_SIZE];

  // start
  smos_WriteBuffer(_T("\nBegin SmGlobalSolver::Dump()")) ;

  // pretty print values

  // end
  smos_WriteBuffer(_T("\nEnd SmGlobalSolver::Dump()\n")) ;

} // end SmGlobalSolver::Dump
