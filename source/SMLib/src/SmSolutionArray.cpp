// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSolutionArray.cpp
* PURPOSE: Source file for implementation of SmSolutionArray methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmSolutionArray.h>
#include <SmSurface.h>
#include <SmCurve.h>
#include <SmAssertArray.h>
#include <SmVertex.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <SmVolume.h>
// Remove Composites
//  #include <SmCEdge.h>
// Remove Composites
//  #include <SmCFace.h>
#include <SmPoly.h>

// for Draw() method
#include <SmGraphicsOutput.h>

// Initial capacity of an SmSolutionArray. Most hold one or two solutions, and each
// SmSolution is a few hundred bytes, all zeroed on allocation.
#define SM_SOLUTION_ARRAY_MIN_SIZE 4

/*******************************************************************//**
PURPOSE: Return TRUE when 3 real values are in ascending or
            descending order, else return FALSE.

NOTES:
***********************************************************************/
static SmBoolean sm_AreSequential
  (double a,     // in :  1st arg
   double b,     // in :  2nd arg
   double c)     // in :  3rd arg
{
  return(  (   (a < b && b < c) 
            || (a > b && b > c)) ? TRUE : FALSE) ;

} // end sm_AreSequential

/**************************************************************
PURPOSE: Assignment Operator

NOTES:
**************************************************************/
SmSolutionEnd & SmSolutionEnd::operator=  // eff: assignment operator
 (const SmSolutionEnd & crSolution)    // in : object to copy
{
  if(this == &crSolution) return *this ;

  m_dSolutionValue = crSolution.m_dSolutionValue ;
  // okay to use smos_MemCpy on base (double) objects.
  SE(smos_MemCpy(&m_adParameters, &crSolution.m_adParameters, sizeof(double) * 2 * SM_SA_MAX_TREES, sizeof(double) * 2 * SM_SA_MAX_TREES));

  return *this ;

} // end SmSolutionEnd::operator= assignment operator

/*******************************************************************//**
PURPOSE: Equality operator for SmSolutionEnd

NOTES: This only does a shallow check - so only pointer values
       and not objects are compared when comparing a pair of
       SmTArrays of pointers.
***********************************************************************/
SmBoolean SmSolutionEnd::operator==
  (const SmSolutionEnd & crOther) 
 const
{
  // low work - same objects
  if(&crOther == this)
    { return TRUE ; }

  // locals
  ULONG ii ;

  // compare
  SmBoolean bRtn = SM_ARE_SAME(m_dSolutionValue, crOther.m_dSolutionValue) ;
  for(ii=0;ii<SM_SA_MAX_TREES*2 && bRtn;ii++)
    {
      bRtn &= SM_ARE_SAME(m_adParameters[ii], crOther.m_adParameters[ii]) ;
    }

  // all done
  return(bRtn) ; 

} // end SmSolutionEnd::operator==

/**************************************************************
PURPOSE: SmSolution Copy Constructor

NOTES:
**************************************************************/
SmSolution::SmSolution(const SmSolution & crSrc) 
{

  m_eSolutionType = crSrc.m_eSolutionType ;
  m_lNumVariables = crSrc.m_lNumVariables ;   
  m_vStart        = crSrc.m_vStart ;   
  m_vEnd          = crSrc.m_vEnd ;   
  m_lNumObjects   = crSrc.m_lNumObjects ; 
  
  // okay to use smos_MemCpy on base (pointer) objects.
  SE(smos_MemCpy(&m_apObjects, &crSrc.m_apObjects, sizeof(SmObject *)   * SM_SA_MAX_TREES, sizeof(SmObject *)   * SM_SA_MAX_TREES));
  SE(smos_MemCpy(&m_apNodes,   &crSrc.m_apNodes  , sizeof(SmTreeNode *) * SM_SA_MAX_TREES, sizeof(SmTreeNode *) * SM_SA_MAX_TREES));

} // end SmSolution::SmSolution Copy Constructor

/**************************************************************
PURPOSE: Assignment Operator

NOTES:
**************************************************************/
SmSolution & SmSolution::operator=  // eff: assignment operator
 (const SmSolution & crSrc)    // in : object to copy
{
  if(this == &crSrc) return *this ;

  m_eSolutionType = crSrc.m_eSolutionType ;
  m_lNumVariables = crSrc.m_lNumVariables ;   
  m_vStart        = crSrc.m_vStart ;   
  m_vEnd          = crSrc.m_vEnd ;   
  m_lNumObjects   = crSrc.m_lNumObjects ; 
  
  // okay to use smos_MemCpy on base (pointer) objects.
  SE(smos_MemCpy(&m_apObjects, &crSrc.m_apObjects, sizeof(SmObject *)   * SM_SA_MAX_TREES, sizeof(SmObject *)   * SM_SA_MAX_TREES));
  SE(smos_MemCpy(&m_apNodes,   &crSrc.m_apNodes  , sizeof(SmTreeNode *) * SM_SA_MAX_TREES, sizeof(SmTreeNode *) * SM_SA_MAX_TREES));

  return *this ;

} // end SmSolution::operator= assignment operator

/*******************************************************************//**
PURPOSE: SmSolution Equality operator

NOTES: This only does a shallow check - so only pointer values
       and not objects are compared when comparing a pair of
       SmTArrays of pointers.
***********************************************************************/
SmBoolean SmSolution::operator==
  (const SmSolution & crOther) 
 const
{
  // low work - same objects
  if(&crOther == this)
    { return TRUE ; }

  // locals
  ULONG ii ;

  // compares
  SmBoolean bRtn = (   m_eSolutionType == crOther.m_eSolutionType
                    && m_lNumVariables == crOther.m_lNumVariables
                    && m_vStart        == crOther.m_vStart
                    && m_vEnd          == crOther.m_vEnd
                    && m_lNumObjects   == crOther.m_lNumObjects) ;
  for(ii=0;ii<m_lNumObjects && bRtn;ii++)
    {
      bRtn &= m_apObjects[ii] == crOther.m_apObjects[ii] ;
    }

  // all done
  return(bRtn) ;

} // end SmSolution::operator==

/*******************************************************************//**
PURPOSE: Classify the solution end relative to the solution given.

Example:
   The following classifications are valid return values:
    0 - matches single value solution or is inside of a range of values solution
   -1 - matches the minimum end of a range of values
   +1 - matches the maximum end of a range of values
   -2 - less than the minimum 
   +2 - greater than the maximum
   +3 - there is an inconsistency in the relationship - no classification possible
           Different NumVariables, NumObjects, or ObjectPointers
           Unequal Solution Values at matching parameter locations
           A rangeOfValues solution interval is degenerate

NOTES: 
***********************************************************************/
long SmSolution::ClassifySolutionEnd
  (const SmSolution & crOtherSolution,  // in : Solution to compare to thisSolution
   SmBoolean bCompareStart)             // in : TRUE = compare OtherSolution.Start to thisSolution
                                        //      FALSE= compare OtherSolution.End   to thisSolution
  const
{
  // can't compare different variable or object counts
  if (   m_lNumVariables != crOtherSolution.m_lNumVariables
      || m_lNumObjects   != crOtherSolution.m_lNumObjects) 
    {
      return 3;
    }

  // can't compare different solution objects
  for (ULONG k=0; k<m_lNumObjects; k++) 
    {
      if (m_apObjects[k] != crOtherSolution.m_apObjects[k])
          return 3;
    }

  // Get OtherSolution target end
  const SmSolutionEnd * pEndToClassify =  (   bCompareStart 
                                           || crOtherSolution.m_eSolutionType == SM_ST_SINGLE_VALUE)
                                         ? &crOtherSolution.m_vStart
                                         : &crOtherSolution.m_vEnd ;

  // When thisSolution is a point
  if (m_eSolutionType == SM_ST_SINGLE_VALUE) 
    {
      long lRet = 3;

      // can't compare unequal solutionValues
      if (SM_EFF_ZERO_SQRT < smos_Fabs(  m_vStart.m_dSolutionValue
                                       - pEndToClassify->m_dSolutionValue)) 
        {
          return lRet ;
        }

      // for every variable
      for (ULONG i=0; i<m_lNumVariables; i++) 
        {
          double dScaledZero = SM_EFF_ZERO * (1.0 + smos_Fabs(m_vStart[i])) ;

          // when parameters equal - continue to next param after setting classification as needed
          if (smos_Fabs(m_vStart[i]-(*pEndToClassify)[i]) < dScaledZero) 
            {
              if (lRet != 3 && lRet != 0) continue;
              lRet = 0;  // Match values
              continue;
            }

          // else parameters are different

          // had a match and lost it
          if (lRet == 0) return 3;  

          // remember if this is lower or higher
          if ((*pEndToClassify)[0] < m_vStart[0]) 
            {
              lRet = -2;  // Less than
              break;
            }
          else 
            {
              lRet = 2;  // Greater than
              break;
            }
        }
      return lRet;
    
    } // end thisSolution is a point branch

  SM_ASSERT(m_eSolutionType == SM_ST_RANGE_OF_VALUES) ;

  // else this Solution is an interval solution
  long lRet = 3;

  // for every Variable
  for (ULONG i=0; i<m_lNumVariables; i++) 
    {
      double dScaledZero = smos_Fabs(1.0 + smos_Fabs((*pEndToClassify)[i])) * SM_EFF_ZERO;

      // thisSolutionVariable Param range is degenerate
      if (   smos_Fabs(m_vStart[i]-(*pEndToClassify)[i]) < dScaledZero
          && smos_Fabs(m_vEnd[i]  -(*pEndToClassify)[i]) < dScaledZero) 
        {
          // Ambiguous situation just continue to next one
          continue;
        }

      // ThisSolution->Start = OtherSolution->End
      if (smos_Fabs(m_vStart[i]-(*pEndToClassify)[i]) < dScaledZero) 
        {
          // when current parameter classification conflicts with previous ones - return inconsitency
          if (lRet == -2 || lRet == 2 || lRet == 1) return 3;
          
          // don't change unlabeled and nonMinumum classifications
          if (lRet != 3 && lRet != -1) continue;

          // Solutions at same point must be the same
          if (smos_Fabs(m_vStart.m_dSolutionValue - pEndToClassify->m_dSolutionValue) > SM_EFF_ZERO_SQRT) 
            {
              return 3;
            }
          lRet = -1; // Match start
          continue;
        }

      // ThisSolution->End = OtherSolution->End
      if (smos_Fabs(m_vEnd[i]-(*pEndToClassify)[i]) < dScaledZero) 
        {
          // when current parameter classification conflicts with previous ones - return inconsitency
          if (lRet == -2 || lRet == 2 || lRet == -1) return 3;
          
          // don't change unlabeled and nonMaximum classifications
          if (lRet != 3 && lRet != 1) continue;
                    
          // Solutions at same point must be the same
          if (smos_Fabs(m_vEnd.m_dSolutionValue - pEndToClassify->m_dSolutionValue) > SM_EFF_ZERO_SQRT) 
            {
              return 3;
            }
          lRet = 1; // Match end
          continue;
        }

      // Ascending [This->Start, Other->End, This->End] and matching solution values
      if (sm_AreSequential(m_vStart[i],(*pEndToClassify)[i],m_vEnd[i])) 
        {
          // when current solution parameters classification is not the first 
          // or not the same as previous ones - return inconsistent relationship.
          if (lRet != 3 && lRet != 0) return 3;

          //      // When End SolutionValue is much better than Start and End SolutionValues - don't merge the solution
          //      if(   (m_vStart.m_dSolutionValue - pEndToClassify->m_dSolutionValue) > SM_EFF_ZERO_SQRT 
          //         && (m_vEnd.m_dSolutionValue - pEndToClassify->m_dSolutionValue) > SM_EFF_ZERO_SQRT) 
          //        {
          //          return 3;
          //        }

          // remember that this parameter is between start and end
          lRet = 0; // Between start and end
          continue;
        }

      // Below Start, [Other->End, This->Start, This->End] 
      if (sm_AreSequential((*pEndToClassify)[i],m_vStart[i],m_vEnd[i])) 
        {
          // when current solution parameters classification is not the first 
          // or not the same as previous ones - return inconsistent relationship.
          if (lRet != 3 && lRet != -2) return 3;

          // remember that this parameter is Below Start
          lRet = -2; // Below Start
          continue;
        }
      
      // Above End, [This->Start, This->End, Other->End] 
      if (sm_AreSequential(m_vStart[i],m_vEnd[i],(*pEndToClassify)[i])) 
        {
          // when current solution parameters classification is not the first 
          // or not the same as previous ones - return inconsistent relationship.
          if (lRet != 3 && lRet != 2) return 3;
          
          // remember that this parameter is Above End
          lRet = 2; // Above End
          continue;
        }

      // If we make it to here then we have some inconsistency 
      return 3;
    }

  return lRet;

} // end SmSolution::ClassifySolutionEnd

/*******************************************************************//**
PURPOSE:  get Solution object's object and param indices

NOTES:
 
***********************************************************************/
SmStatus SmSolution::GetIndex
  (const SmObject *pTargetObject,  // in : target object to query
   SmBoolean bObjectFlag,          // in : TRUE = get index for given pObject
                                   //      FALSE= get index for 1st object that is not pObject
   ULONG &lObjectIndex,            // out: Index of requested object
   ULONG &lParamIndex)             // out: Index of 1st solution parameter for requested object 
 const
{
  // init output
  lObjectIndex = m_lNumObjects ;
  lParamIndex  = 0 ;

  // figure out the index for [pObject,bThisFlag]
  for(ULONG ii=0; ii<m_lNumObjects; ii++)
    {
      SmObject *pObject = m_apObjects[ii];

      // check for a hit
      if(   ( bObjectFlag && pTargetObject == pObject)
         || (!bObjectFlag && pTargetObject != pObject)) 
         { 
           lObjectIndex = ii ;
           return(SM_SUCCESS) ;
         }
      else // not the desired object
         {
            // This is not the index we want, move down to the next start
            // parameter based on the type of the object.
            if      (pObject->IsKindOf(SmCurve_TYPE))   { lParamIndex++; }
            else if (pObject->IsKindOf(SmEdge_TYPE))    { lParamIndex++; }
            else if (pObject->IsKindOf(SmFace_TYPE))    { lParamIndex += 2; }
            else if (pObject->IsKindOf(SmSurface_TYPE)) { lParamIndex += 2; }
         } // end not desired object branch
                  
    } // end iter every object

  // arrive here with no hits
  return(SM_ERR) ;

} // end SmSolution::GetIndex

/*******************************************************************//**
PURPOSE: Given a solution from a topology solve (SmTopologySolver),
    parse it and determine the 3D solution point generated by either
    target object or the other object.

NOTES:
    Returns the start point for a range of solutions
 
***********************************************************************/
SmStatus SmSolution::GetPoint
  (const SmObject *pObject,    // in : target object to query
   SmBoolean bObjectFlag,      // in : TRUE = get point on given pObject
                               //      FALSE= get point on 1st object that is not pObject
   SmPoint3d &rSolPoint,       // out: Solution Point for the lIndex object
   SmPoint3d *pOptSolParam,    // out: Solution Parameter for the lIndex object,
                               //      when SolObjects is
                               //        SmVertex or SmPoint  : set to: [0 0 0]
                               //        SmEdge   or SmCurve  :         [u 0 0]
                               //        SmFace   or SmSurface:         [u v 0]
                               //        SmVolume             :         [u v w]
                               //      NULL to ignore
                               //      default:[NULL]
   SmBoolean bGetEnd)          // in, opt: if True, get the End of a Range solution.
                               //      If not a range solution, just return the Start.
                               //      default:[FALSE]
 const
{
  // locals
  ULONG lIndex = 0 ;
  ULONG lParamIndex = 0 ;

  // figure out the index for [pObject,bThisFlag]
  if(SM_SUCCESS != GetIndex(pObject, bObjectFlag, lIndex, lParamIndex))
    {
      rSolPoint.SetUninitialized() ;
      return(SM_ERR) ;
    }

  // pass the call along
  GetPoint( lIndex, rSolPoint, pOptSolParam, bGetEnd );
  return(SM_SUCCESS) ;

} // end SmSolution::GetPoint

/*******************************************************************//**
PURPOSE: Given a solution from a topology solve (SmTopologySolver),
    parse it and determine the 3D point generated by the i-th (lIndex)
    object.

NOTES:
    Returns the start point for a range of solutions

***********************************************************************/
SmStatus SmSolution::GetPoint
  (ULONG      lIndex,          // in : object index to query
   SmPoint3d &rSolPoint,       // out: Solution Point for the lIndex object
   SmPoint3d *pOptSolParam,    // out: Soloution Parameter for the lIndex object,
                               //      when SolObjects is
                               //        SmVertex or SmPoint  : set to: [0 0 0]
                               //        SmEdge   or SmCurve  :         [u 0 0]
                               //        SmFace   or SmSurface:         [u v 0]
                               //        SmVolume             :         [u v w]
                               //      NULL to ignore
                               //      default:[NULL]
   SmBoolean bGetEnd)          // in, opt: if True, get the End of a Range solution.
                               //      If not a range solution, just return the Start.
                               //      default:[FALSE]
 const
{
  // init output
  rSolPoint.SetUninitialized() ;
  if(pOptSolParam) pOptSolParam->Set(0.0, 0.0, 0.0) ;

  // Not an error if bGetEnd is passed True for a point solution.
  if ( m_eSolutionType != SM_ST_RANGE_OF_VALUES )
    { bGetEnd = FALSE; }

  // check state - lIndex out of range
  if(lIndex >= m_lNumObjects)
    { SER(SM_ERR) ; }

  // local - parameter index associated with the lIndex object
  ULONG lStart = 0;

  double dParam;

  // for every object in this solution
  for (ULONG i=0; i<=lIndex; i++)
    {
      // Get the object from the solution.
      SmObject *pObject = m_apObjects[i];

      // convert object index into parameter index
      if (i != lIndex)
        {
          // This is not the index we want, move down to the next start
          // parameter based on the type of the object.
          if (pObject->IsKindOf(SmCurve_TYPE)) { lStart++; }
          if (pObject->IsKindOf(SmEdge_TYPE))  { lStart++; }
          if (pObject->IsKindOf(SmFace_TYPE))  { lStart += 2; }

        } // end counting object parameters for each object branch

      else // found the lIndex object
        {
          // This is the object at the index we are looking for.
          // There are five possibilities:
          // 1 - Object is a vertex - get the point from the vertex.
          // 2 - Object is an edge - get the 3-D curve from the edge
          //        and evaluate it at the parameter found in the solution.
          // 3 - Object is a curve - evaluate it at the parameter
          //       found in the solution.
          // 4 - Object is a face - get the surface from the face and
          //       evaluate the surface using two parameters from the solution.
          // 5 - Object is a surface - evaluate it using two parameters
          //       from the solution.
          // 6 - Object is a PolyVertex - get the point from the PolyVertex.
          // 7 - Object is a PolyEdge - recurse on this function using the other object
          // 8 - Object is a PolyFace - recurse on this function using the other object
          if ( pObject->IsKindOf( SmVertex_TYPE ))
            {
              SmVertex *pV = (SmVertex*) pObject;
              rSolPoint    = pV->GetPoint();
              return( SM_SUCCESS );
            }
          else if ( pObject->IsKindOf( SmEdge_TYPE ))
            {
              SmEdge  *pE     = (SmEdge*)pObject;
              SmCurve *pCurve = pE->GetCurve();
              // Evaluate the curve at the current parameter
              dParam = ( bGetEnd ) ? m_vEnd[ lStart ] : m_vStart[ lStart ];
              SE( pCurve->EvaluatePoint( dParam, rSolPoint ));
              if ( pOptSolParam ) { pOptSolParam->Set( dParam, 0.0, 0.0 ); }
              return( SM_SUCCESS );
            }
          else 

            if ( pObject->IsKindOf( SmCurve_TYPE ))
            {
              SmCurve *pCurve = (SmCurve*)pObject;
              // Evaluate the curve at the current parameter
              dParam = ( bGetEnd ) ? m_vEnd[ lStart ] : m_vStart[ lStart ];
              SE( pCurve->EvaluatePoint(  dParam, rSolPoint ));
              if ( pOptSolParam ) { pOptSolParam->Set( dParam, 0.0, 0.0 ); }
              return( SM_SUCCESS );
            }
          else if ( pObject->IsKindOf( SmFace_TYPE ))
            {
              SmFace *pF = (SmFace*)pObject;
              SmSurface *pSurface = pF->GetSurface();
              // Evaluate the surface using two parameters from the solution.
              SmPoint2d sUV( m_vStart[ lStart ], m_vStart[ lStart+1 ] );
              if ( bGetEnd )
                { sUV.Set( m_vEnd[ lStart ], m_vEnd[ lStart+1 ] ); }
              SE( pSurface->EvaluatePoint( sUV, rSolPoint ));
              if ( pOptSolParam ) { pOptSolParam->Set( sUV.x,  sUV.y, 0.0 ); }
              return( SM_SUCCESS );
            }
          else if ( pObject->IsKindOf( SmPolyVertex_TYPE ))
            {
              SmPolyVertex *pPV = (SmPolyVertex*) pObject;
              rSolPoint = pPV->GetPoint();
              return( SM_SUCCESS );
            }
          else if ( pObject->IsKindOf( SmSurface_TYPE ))
            {
              SmSurface *pSurface = (SmSurface*)pObject;
              // Evaluate the surface using two parameters from the solution.
              SmPoint2d sUV( m_vStart[ lStart ], m_vStart[ lStart+1 ] );
              if ( bGetEnd )
                { sUV.Set( m_vEnd[ lStart ], m_vEnd[ lStart+1 ] ); }
              SE( pSurface->EvaluatePoint( sUV, rSolPoint ));
              if ( pOptSolParam ) { pOptSolParam->Set( sUV.x,  sUV.y, 0.0 ); }
              return( SM_SUCCESS );
            }
          else if ( pObject->IsKindOf( SmPolyEdge_TYPE )
                 || pObject->IsKindOf( SmPolyFace_TYPE ))
            {
              // PolyEdges and PolyFaces do not have parameterized geometry.
              // So just request the other object to compute the point.

              // the only solutions that make it to here
              // are line/polyedge or line/polyface intersections
              SM_ASSERT(lIndex == 1) ;

              // However, we want the point exactly on the Poly topology,
              // not a point exactly on the pick line.
              // So work from the stored coordinates.
              //   GetPoint(0, rSolPoint, pOptSolParam, bGetEnd) ;
              rSolPoint.Set( m_vStart[3], m_vStart[4], m_vStart[5] );
              if ( pOptSolParam != NULL )
                { pOptSolParam->Set( m_vStart[0], 0, 0 ); }

              return( SM_SUCCESS );
            }
        } // end found lIndex object branch
    } // end iter every object in this solution

  // arrive here with an unhandled type or bad index count error
  SE(SM_ERR);
  return(SM_ERR);

} // end SmSolution::GetPoint

/*******************************************************************//**
PURPOSE: Compute the Object Index and Object DomainDir for the given
            Solution Paremeter Index.

NOTES: Returns SmErr when lParameterIndex is not within the
                bounds of the current Solution.

                Assumes m_apObjects are set
***********************************************************************/
SmStatus SmSolution::GetObjectIndexForParameterIndex
  (ULONG  lParameterIndex,   // in : Target Parameter Index
   ULONG &rlObjectIndex,     // out: Index of Object associated with ParameterIndex
   ULONG &rlDomainDir)       // out: Domain index associated with ParameterIndex
                             //      0 = s or u for curves, surfaces, and volumes
                             //      1 = v for surfaces and volumes
                             //      2 = w for volumes
  const
{
  // init output
  rlObjectIndex = 0 ;
  rlDomainDir   = 0 ;

  // check input
  if(   lParameterIndex >= m_lNumVariables
     || m_lNumObjects   == 0)
    {
      return(SM_ERR);
    }

  // locals
  ULONG lPIndex = 0 ;

  // else iter through all the domains
  for(ULONG ii=0;ii<m_lNumObjects;ii++)
    {
      SmObject *pObject = m_apObjects[rlObjectIndex] ;

      // some solvers actually don't store objects for every parameter value.
      if(pObject == NULL)
        {
          // no way to figure out the proper alignment of parameter index to object
          return(SM_ERR) ;
        }

      // things with 1 parameter
      if(        pObject->IsKindOf(SmEdge_TYPE)
              || pObject->IsKindOf(SmCurve_TYPE))
        {
          if( lPIndex + 1 > lParameterIndex){ rlDomainDir = 0;
                                              rlObjectIndex = ii;
                                              break;
                                            }
          lPIndex += 1 ;
        }

      // things with 2 parameters
      else if(   pObject->IsKindOf(SmFace_TYPE)   
              || pObject->IsKindOf(SmSurface_TYPE))
        {
          if( lPIndex + 2 > lParameterIndex){ rlDomainDir = lParameterIndex - lPIndex ;  
                                              rlObjectIndex = ii ;
                                              break ;
                                            }
          lPIndex += 2 ;
       }

      // things with 3 parameters
      else if(   pObject->IsKindOf(SmVolume_TYPE))
        {
          if( lPIndex + 3 > lParameterIndex){ rlDomainDir = lParameterIndex - lPIndex ;   
                                              rlObjectIndex = ii ;
                                              break ;
                                            }
          lPIndex += 3 ;
        }
      
      // things without parameters
      else if (   pObject->IsKindOf(SmVertex_TYPE)
               || pObject->IsKindOf(SmPolyBrep_TYPE)
               || pObject->IsKindOf(SmPolyRegion_TYPE)
               || pObject->IsKindOf(SmPolyShell_TYPE)
               || pObject->IsKindOf(SmPolyLoop_TYPE)
               || pObject->IsKindOf(SmCPolyFace_TYPE)
               || pObject->IsKindOf(SmPolyFace_TYPE)
               || pObject->IsKindOf(SmPolyEdge_TYPE)
               || pObject->IsKindOf(SmPolyVertex_TYPE))
        {
          continue;
        }

      // error
      else
        {
          return(SM_ERR) ;
        }
    } // end iter to lParameterIndex

  // all done
  return(SM_SUCCESS) ;

} // end SmSolution::GetObjectIndexForParameterIndex

/*******************************************************************//**
PURPOSE: Get the associated parameter interval for the given
            Solution Paremeter Index.

NOTES: Returns SmErr when lParameterIndex is not within the
                bounds of the current Solution.

                Assumes m_apObjects are set
***********************************************************************/
SmStatus SmSolution::GetParameterInterval
  (ULONG  lParameterIndex,   // in : Target Parameter Index
   SmExtent1d &rIvl)         // out: interval associated with lParameterIndex
  const
{
  // init output
  rIvl.Init() ;

  // locals

  // get the object and associated domain dir
  ULONG lObjectIndex = 0 ;
  ULONG lDomainDir   = 0 ;
  SmStatus sRtn = GetObjectIndexForParameterIndex(lParameterIndex, lObjectIndex, lDomainDir) ;

  // no work - Don't have objects to search or something is wrong
  if(sRtn != SM_SUCCESS)
    {
      return(sRtn) ;
    }

  SmObject *pObject = m_apObjects[lObjectIndex] ;

  // Get the parameter interval from pObject

  // gwc:note - this is a hack.
  //   We could create a virtual function SmObject::GetDomainInterval(ULONG lDomainDir) and
  //   implement all the appropriate derived methods.  But since this is only getting 
  //   a single use at this time, I'll leave it this way for now.

// Remove Composites
// // things with 1 parameter
//  // Note: derived types before parent types, for IsKindOf().
//  if ( pObject->IsKindOf( SmCEdge_TYPE ))
//    {
//      SmTArray<SmEdge*> sEdges;
//      ((SmCEdge*)pObject)->GetEdges( sEdges );
//      rIvl = sEdges[0]->GetInterval();
//      for ( ULONG ii=1; ii<sEdges.GetSize(); ii++ )
//        {
//          rIvl.Union( sEdges[ii]->GetInterval(), rIvl );
//        }
//    }
//  else if ( pObject->IsKindOf( SmEdge_TYPE ))
  if ( pObject->IsKindOf( SmEdge_TYPE ))
    {
      rIvl = ((SmEdge*)pObject)->GetInterval();
    }
  else if ( pObject->IsKindOf( SmCurve_TYPE ))
    {
      rIvl = ((SmCurve*)pObject)->GetNaturalInterval();
    }
  // Note: derived types before parent types, for IsKindOf().
// Remove Composites
//  else if ( pObject->IsKindOf( SmCFace_TYPE ))
//    {
//      SmTArray<SmFace*> sFaces;
//      SmExtent1d sTmpIvl; // For a Borland compiler issue.
//      ((SmCFace*)pObject)->GetFaces( sFaces );
//      rIvl = lDomainDir == 0
//             ? sFaces[0]->GetUVDomain().GetUInterval()
//             : sFaces[0]->GetUVDomain().GetVInterval();
//      for ( ULONG ii=1; ii<sFaces.GetSize(); ii++ )
//        {
//          sTmpIvl = lDomainDir == 0
//                    ? sFaces[ii]->GetUVDomain().GetUInterval()
//                    : sFaces[ii]->GetUVDomain().GetVInterval();
//          rIvl.Union( sTmpIvl, rIvl );
//        }
//    }
  else if ( pObject->IsKindOf( SmFace_TYPE ))
    {
      rIvl = lDomainDir == 0
             ? ((SmFace*)pObject)->GetUVDomain().GetUInterval()
             : ((SmFace*)pObject)->GetUVDomain().GetVInterval();
    }

  else if ( pObject->IsKindOf( SmSurface_TYPE ))
    {
      rIvl = lDomainDir == 0
             ? ((SmSurface*)pObject)->GetNaturalUVDomain().GetUInterval()
             : ((SmSurface*)pObject)->GetNaturalUVDomain().GetVInterval();
    }
  else if ( pObject->IsKindOf( SmVolume_TYPE ))
    {
      rIvl =   lDomainDir == 0 ? ((SmVolume*)pObject)->GetNaturalParamDomain().GetUInterval()
             : lDomainDir == 1 ? ((SmVolume*)pObject)->GetNaturalParamDomain().GetVInterval()
             :                   ((SmVolume*)pObject)->GetNaturalParamDomain().GetWInterval();
    }
  else
    {
      // includes things without parameters: SmVertex_TYPE, SmPolyBrep_TYPE, SmPolyRegion_TYPE, SmPolyShell_TYPE, SmPolyLoop_TYPE, SmCPolyFace_TYPE, SmPolyFace_TYPE, SmPolyEdge_TYPE, SmPolyVertex_TYPE
      return( SM_ERR );
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmSolution::GetParameterInterval

/*******************************************************************//**
PURPOSE: When Object exists for given ObjectIndex and it has a 
            tolerance (Breps, Faces, Edges, and Vertices) return that
            tolerance else return 0.0

NOTES: 
***********************************************************************/
double SmSolution::GetObjectTolerance
  (ULONG lObjectIndex)   // in : Target index for Object stored in m_apObjects array
 const 
{
  // no tolerance for invalid indices
  if(lObjectIndex >= m_lNumObjects) 
    { return(0.0) ; }

  // target object
  SmObject *pObj = m_apObjects[lObjectIndex] ;
  
  // pass the call along
  if(pObj) { return( SmTol::GetZoneTol3d(pObj) ) ; }
  else     { return( 0.0 ) ; }

} // end SmSolution::GetObjectTolerance

/*******************************************************************//**
PURPOSE: Test to see if two solutions are identical.  This 
   prevents us from adding more then one solution which is the
   same.  Note that the default definition of an identical 
   solution is that the values are the same and that the 
   interval values are the same.

NOTES: 
***********************************************************************/
SmBoolean SmSolution::IsEqual
  (const SmSolution & crSolution2)  // in : 2nd target solution
  const
{
  // lcoals
  const SmSolution &crSolution1 = *this ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smos_WriteBuffer(_T("\n\nSolution 1 +++++++++++++++++++++++++++++++++\n"));
      crSolution1.Dump();
      smos_WriteBuffer(_T("Solution 2 --------------------------------\n"));
      crSolution2.Dump();
    }
#endif

  // different ObjectCounts, VariableCounts, SolutionValues, SolutionTypes
  if (   (crSolution1.m_lNumObjects   != crSolution2.m_lNumObjects) 
      || (crSolution1.m_lNumVariables != crSolution2.m_lNumVariables)
      || (crSolution1.m_eSolutionType != crSolution2.m_eSolutionType)
      || (SM_EFF_ZERO_SQRT < smos_Fabs(  crSolution1.m_vStart.m_dSolutionValue
                                       - crSolution2.m_vStart.m_dSolutionValue)))
    { 
      return FALSE;
    }

  // unequal ObjectPointers or parameterValues
  for (ULONG j=0; j<crSolution1.m_lNumObjects; j++) 
    {
      if (crSolution1.m_apObjects[j] != crSolution2.m_apObjects[j]) 
        { 
          return FALSE;
        }
    }

  SmExtent1d sIvl1, sIvl2 ;

  // ParameterValues off by more than ScaledZero or more than 1/10000th of the parameter length
  for (ULONG i=0; i<crSolution1.m_lNumVariables; i++) 
    {
      SmStatus sRtn1 = crSolution1.GetParameterInterval(i, sIvl1) ;
      SmStatus sRtn2 = crSolution2.GetParameterInterval(i, sIvl2) ;
      double dLength1 = (sRtn1 == SM_SUCCESS) ? .0001 * sIvl1.GetLength() : 0.0 ;
      double dLength2 = (sRtn2 == SM_SUCCESS) ? .0001 * sIvl2.GetLength() : 0.0 ;

      double dScaledZero = smos_3Max(smos_Fabs(1.0 + smos_Fabs(crSolution1.m_vStart[i])) * SM_EFF_ZERO,
                                     dLength1,
                                     dLength2) ;

      if (   (   smos_Fabs(crSolution1.m_vStart[i]-crSolution2.m_vStart[i]) > dScaledZero)
          || (   crSolution1.m_eSolutionType == SM_ST_RANGE_OF_VALUES 
              && smos_Fabs(crSolution1.m_vEnd[i]-crSolution2.m_vEnd[i]) > dScaledZero)) 
        {
          return FALSE;
        }
    }
  
  // All tests passed - identical Solutions
  return TRUE;

} // end SmSolution::IsEqual

/*******************************************************************//**
PURPOSE: Constructor for an array allows the initial input of a data
    array.  

NOTES: 
    This method allows you to declare the data for an array on
    stack and input it to the construction.  Note that if the size grows
    greater than the initial size, a new data array will be allocated
    and used insead of the one put in by the constructor.  In general
    the size of the one input by the constructor should be big enough
    to cover most cases to achieve optimal performance.  Note that if the
    borrowed data is not specified than the memory is allocated from the
    heap.
  
    See the following for an example:
    - SmSolution sData[100];
    - SmSolutionArray sSolutions(100,sData);  // Use data input
    - SmSolutionArray sSolutions2(100);  // No data input
***********************************************************************/
SmSolutionArray::SmSolutionArray
  (ULONG        nDataSize,         // in : Size of OptBorrowedData
   SmSolution * pOptBorrowedData)  // in : the borrowed array
{ 
  m_cpContext = NULL;
  if (pOptBorrowedData) 
    {
      m_lSize       = 0;
      m_lMaxSize    = nDataSize;
      m_pData       = pOptBorrowedData;
      m_bIsBorrowed = TRUE;
      for (ULONG i=0; i<m_lMaxSize; i++) 
        {
          m_pData[i].m_lNumVariables = 0;
          m_pData[i].m_lNumObjects = 0;
          smos_MemSet(&pOptBorrowedData[i].m_vStart, 0, sizeof(SmSolutionEnd));
          smos_MemSet(&pOptBorrowedData[i].m_vEnd, 0, sizeof(SmSolutionEnd));
        }
    }
  else 
    {
      m_lSize       = 0 ;
      m_lMaxSize    = 0;
      m_pData       = NULL;
      m_bIsBorrowed = FALSE;
      SetSize(nDataSize); 
      m_lSize = 0; 
    }

} // end SmSolutionArray::SmSolutionArray constructor for borrowed arrays

/*******************************************************************//**
PURPOSE: Copy Constructor

NOTES:
***********************************************************************/
SmSolutionArray::SmSolutionArray(const SmSolutionArray & crSrc)
{
  m_cpContext   = NULL;
  m_lSize       = 0;
  m_lMaxSize    = 0;
  m_pData       = NULL;
  m_bIsBorrowed = FALSE;
  Append(crSrc);

} // end SmSolutionArray::SmSolutionArray copy constructor

/*******************************************************************//**
PURPOSE: Assignment operator

NOTES: 
***********************************************************************/
SmSolutionArray & SmSolutionArray::operator=
  (const SmSolutionArray & crSrc)
{
  if(&crSrc == this) 
    { return *this ; }

  // base members
  m_cpContext = crSrc.m_cpContext ;

  // member values
  ReSet() ;
  Append(crSrc) ;
  return *this;

} // end SmSolutionArray::operator=

/*******************************************************************//**
PURPOSE: SmSolutionArray Equality operator 

NOTES: This only does a shallow check - so only pointer values
       and not objects are compared when comparing a pair of
       SmTArrays of pointers.
***********************************************************************/
SmBoolean SmSolutionArray::operator==
  (const SmSolutionArray & crOther) 
 const
{
  // low work - same objects
  if(&crOther == this)
    { return TRUE ; }

  // locals
  ULONG ii ;
  SmBoolean bRtn = (m_lSize == crOther.m_lSize) ;

  if(bRtn)
    {
      // check equivalence of these objects
      for(ii=0;ii<m_lSize && bRtn;ii++)
        {
          bRtn &= (m_pData[ii] == crOther.m_pData[ii]) ;
        }  
    }

  // all done
  return bRtn ;

} // end SmSolutionArray::operator==

/*******************************************************************//**
PURPOSE: Destructor for the array.

NOTES: 
***********************************************************************/
SmSolutionArray::~SmSolutionArray()
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID_NO_STREAM(this);                
    }
#endif // SM_DEBUG_CODE 

  if (m_pData && !m_bIsBorrowed) 
    {
      smos_Free(m_pData);
      m_pData = NULL;
    }

} // end SmSolutionArray::~SmSolutionArray destructor

/*******************************************************************//**
PURPOSE: Set the size of the array to a new size and allocate or free
   memory as necessary.

NOTES: 
***********************************************************************/
void SmSolutionArray::SetSize
  (ULONG nNewSize)
{
  if (nNewSize == 0)
    {
      // shrink to nothing
      if (m_pData && !m_bIsBorrowed) smos_Free(m_pData);
      m_bIsBorrowed = FALSE;
      m_pData = NULL;
      m_lSize = m_lMaxSize = 0;
    }
  else if (m_pData == NULL)
    {
      ULONG nNewMaxSize = SM_SOLUTION_ARRAY_MIN_SIZE;
      while (nNewMaxSize < nNewSize) 
        { nNewMaxSize = nNewMaxSize * 2; }

      // okay to use smos_Calloc on static class (SmSolution and SmSolutionEnd) objects.
      m_pData = (SmSolution*) smos_Calloc(1, nNewMaxSize * sizeof(SmSolution));

      m_lSize = nNewSize;
      m_lMaxSize = nNewMaxSize;
    }
  else if (nNewSize <= m_lMaxSize)
    {
      // first free up old memory

      if (nNewSize > m_lSize)
        {
          // initialize the new elements
          // okay to use smos_MemSet on static class (SmSolution and SmSolutionEnd) objects.
          smos_MemSet(&m_pData[m_lSize], 0, (nNewSize-m_lSize) * sizeof(SmSolution));
        }
      m_lSize = nNewSize;
    }
  else
    {
      // otherwise, grow array
      ULONG nNewMaxSize = SM_SOLUTION_ARRAY_MIN_SIZE;
      while (nNewMaxSize < nNewSize) 
        { nNewMaxSize = nNewMaxSize * 2; }

      SM_ASSERT(nNewMaxSize >= m_lMaxSize);  // no wrap around

      // okay to use smos_Calloc on static class (SmSolution and SmSolutionEnd) objects.
      SmSolution* pNewData = (SmSolution*) smos_Calloc(1, nNewMaxSize * sizeof(SmSolution));

      // copy new data from old
      // okay to use smos_MemCpy on static class (SmSolution and SmSolutionEnd) objects.
      SE(smos_MemCpy(pNewData, m_pData, m_lSize * sizeof(SmSolution), m_lSize * sizeof(SmSolution)));

      // construct remaining elements
      SM_ASSERT(nNewSize > m_lSize);

      // get rid of old stuff (note: no destructors called)
      if (!m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
      m_bIsBorrowed = FALSE;
      m_pData = pNewData;
      m_lSize = nNewSize;
      m_lMaxSize = nNewMaxSize;
    }

} // end SmSolutionArray::SetSize

/*******************************************************************//**
PURPOSE: Set the data size of the array to a new size and allocate or
    free memory as necessary.

NOTES: 
***********************************************************************/
void SmSolutionArray::SetDataSize
  (ULONG nNewSize)
{
  if (nNewSize == m_lMaxSize) return;
  if (nNewSize < m_lMaxSize) 
    {
      // okay to use smos_Calloc on static class (SmSolution and SmSolutionEnd) objects.
      SmSolution* pNewData = (SmSolution*) smos_Calloc(1, nNewSize * sizeof(SmSolution));
            
      // copy new data from old
      m_lSize = smos_Min(m_lSize,nNewSize);
      // okay to use smos_MemCpy on static class (SmSolution and SmSolutionEnd) objects.
      SE(smos_MemCpy(pNewData, m_pData, m_lSize * sizeof(SmSolution), m_lSize * sizeof(SmSolution)));
    
      if (!m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
      m_bIsBorrowed = FALSE;
      m_pData = pNewData;    
      m_lMaxSize = nNewSize;
    }
  else if (nNewSize > m_lMaxSize) 
    {
      // okay to use smos_Calloc on static class (SmSolution and SmSolutionEnd) objects.
      SmSolution* pNewData = (SmSolution*) smos_Calloc(1, nNewSize * sizeof(SmSolution));
            
      if (m_pData) {
          // copy new data from old
          // okay to use smos_MemCpy on static class (SmSolution and SmSolutionEnd) objects.
          SE(smos_MemCpy(pNewData, m_pData, m_lSize * sizeof(SmSolution), m_lSize * sizeof(SmSolution)));
          if (!m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
          m_bIsBorrowed = FALSE;
      }
      m_pData = pNewData;    
      m_lMaxSize = nNewSize;
    }

} // end SmSolutionArray::SetDataSize

/*******************************************************************//**
PURPOSE: set m_lSize, don't realloc memory unless
                   m_lMaxSize < nNewSize
NOTES: 
***********************************************************************/
void SmSolutionArray::SetSizeValue
  (ULONG nNewSize)
{
  // only allocate memory to ensure m_lMaxSize >= nNewSize
  if(m_lMaxSize < nNewSize)
    { SetDataSize(nNewSize) ; }

  // save the value
  m_lSize = nNewSize ;

} // end SmSolutionArray::SetSizeValue

/*******************************************************************//**
PURPOSE: Append an array to the end of another array.

NOTES: 
***********************************************************************/
ULONG SmSolutionArray::Append
  (const SmSolutionArray& crSource)
{
  SM_ASSERT(this != &crSource);   // cannot append to itself

  ULONG nOldSize = m_lSize;
  SetSize(m_lSize + crSource.m_lSize);

  // okay to use smos_MemCpy on static class (SmSolution and SmSolutionEnd) objects.
  SE(smos_MemCpy(m_pData + nOldSize, crSource.m_pData, crSource.m_lSize * sizeof(SmSolution), crSource.m_lSize * sizeof(SmSolution)));

  return nOldSize;

} // end SmSolutionArray::Append
  
/*******************************************************************//**
PURPOSE: Add a solution to the SmSolutionArray list 
            using an insertion sort and a given sort key.

METHOD ---  
  1. Don't add solutions to the solution array when
     a. SolutionValue == SM_BIG_DOUBLE
     b. duplicate solutions
     Return SM_BIG_ULONG
  2. Add Solution into array in its sorted position.
  3. smaller values always come first
     so if you want the biggest value - look at the end of the array
  4. Solutions are not combined, for example
       adding a point SingleValue solution and a spanning RangeOfValues solution
       that happens to contain the point solution parameter results in
       two distinct entries in the Array.  These solutions are not merged
       into one.
***********************************************************************/
ULONG SmSolutionArray::AddSortedSolution // rtn: index of added Solution or SM_BIG_ULONG for rejected solutions
  (const SmSolution & crNewSolution,     // in : target solution
   SmSortKeyType eSortKey)               // in : sort by one of 
                                         //      SM_SK_BY_SOLUTION_VALUE       = sort by value
                                         //      SM_SK_BY_FIRST_PARAMETER      = sort by first parameter
                                         //      SM_SK_BY_FIRST_TWO_PARAMETERS = Sort by first two parameters - second used to resolve ties
                                         //      SM_SK_BY_VALUE_WITH_PARAMETER = Sort by value - using parameters to resolve ties
                                         //      SM_SK_BY_ALL_PARAMETERS       = Sort by first parameter, then 2nd and on up to point of resolving ties
{
  // locals
  SmSolutionArray &rSolutions = *this ;

#ifdef GWC
  // SM_ASSERT(crNewSolution.m_lNumObjects != 0) ;
  { SmBoolean bBool = (crNewSolution.m_lNumObjects != 0) ;
    for(ULONG ii=0;ii<crNewSolution.m_lNumObjects;ii++) { bBool &= (crNewSolution.m_apObjects[ii] != NULL) ; }  
    if (!bBool) 
      { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,NULL,NULL,0,FUNC_NAME); }  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
      crNewSolution.Dump() ;
    }
#endif // SM_DEBUG_CODE
  }

#endif // GWC

  // skip BigSolutions - unitialized values
  if(crNewSolution.m_vStart.m_dSolutionValue == SM_BIG_DOUBLE)
    { return(SM_BIG_ULONG) ; }

  // Now do insertion sort
  SmBoolean bFoundPlace = FALSE;
  ULONG lInsertionIndex = rSolutions.GetSize();

  // for every current solution
  for (ULONG ii=0; ii<rSolutions.GetSize() && bFoundPlace == FALSE; ii++) 
    {
      SmSolution & rOldSolution = rSolutions[ii];

      // don't add duplicate solutions (SolValues within ZeroSqrt and ParamValues within ScaledZero)
      if (rOldSolution.IsEqual(crNewSolution)) 
        { 
          // save the better solution - improve tolerances
          if(  fabs(crNewSolution.m_vStart.m_dSolutionValue)
             < fabs( rOldSolution.m_vStart.m_dSolutionValue))
            {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
              if (bDebugMe) 
                {
                  smos_WriteBuffer(_T("\n\nSolution Being Considered +++++++++++++++++++++++++++++++++\n"));
                  rOldSolution.Dump();
                  smos_WriteBuffer(_T("Current Solution Vector --------------------------------\n"));
                  crNewSolution.Dump();
                }
#endif
              rOldSolution = crNewSolution ;
            }

          return(SM_BIG_ULONG) ;
        } // end solutions are equal check

      // switch on sort key
      switch (eSortKey) 
        {
           case SM_SK_BY_SOLUTION_VALUE:      
               { if(rOldSolution.m_vStart.m_dSolutionValue > crNewSolution.m_vStart.m_dSolutionValue) 
                   {
                     lInsertionIndex = ii;
                     bFoundPlace = TRUE;
                   }
               }
             break;

          case SM_SK_BY_FIRST_PARAMETER:
              { if (rOldSolution.m_vStart[0] > crNewSolution.m_vStart[0]) 
                  {
                    lInsertionIndex = ii;
                    bFoundPlace = TRUE;
                  }
              }
              break;

          case SM_SK_BY_FIRST_TWO_PARAMETERS: 
              { 
                double dScale = (1.0 + rOldSolution.m_vStart[0]) * SM_EFF_ZERO_SQRT;

                if(   (rOldSolution.m_vStart[0] > crNewSolution.m_vStart[0] + dScale)
                   || (   fabs(rOldSolution.m_vStart[0] - crNewSolution.m_vStart[0]) < dScale
                       && rOldSolution.m_vStart[1] > crNewSolution.m_vStart[1]))
                  {
                    lInsertionIndex = ii;
                    bFoundPlace = TRUE;
                  }
              }
              break;


          case SM_SK_BY_VALUE_WITH_PARAMETER:
              { 
                double dScale = (1.0 + rOldSolution.m_vStart.m_dSolutionValue) * SM_EFF_ZERO_SQRT;

                // when sSolutionValues are unique
                if (fabs(rOldSolution.m_vStart.m_dSolutionValue-crNewSolution.m_vStart.m_dSolutionValue) > dScale) 
                  {
                    // Just use standard by value sorting
                    if (rOldSolution.m_vStart.m_dSolutionValue > crNewSolution.m_vStart.m_dSolutionValue) 
                      {
                        lInsertionIndex = ii;
                        bFoundPlace = TRUE;
                      }
                    break; // Only break if solution values are distinct otherwise sort by parameters
                  }
               }    
                // else SolutionValues are the same within tolerance
                // fall through to next case and sort by parameters

          case SM_SK_BY_ALL_PARAMETERS:
              {
                for (ULONG jj=0; jj<rOldSolution.m_lNumVariables; jj++) 
                  {
                    double dScale = (1.0 + rOldSolution.m_vStart[jj]) * SM_EFF_ZERO_SQRT;

                    // if old param is bigger by tolerance - insert here
                    if(rOldSolution.m_vStart[jj] > crNewSolution.m_vStart[jj] + dScale)
                      {
                        lInsertionIndex = ii;
                        bFoundPlace = TRUE;
                        break ;
                      }

                    // else if old param is smaller by tolerance - go onto next tgt
                    else if(rOldSolution.m_vStart[jj] < crNewSolution.m_vStart[jj] - dScale)
                      {
                        break ;
                      }
                  
                    // else old param is equal by tolerance to new param
                    //  - go onto next param if there is one or else go onto next tgt

                  } // end iter every param looking for a deciding parameter difference
              }
              break;
          
          default:
              ERR(SM_ERR);
        } // end switch on sort key - looking to set bFoundPlace == TRUE

    } // end iter every solution seeking new solution place

  // insert the solution in its spot or at the end of the array
  if (bFoundPlace) { InsertAt(lInsertionIndex,crNewSolution,1); }
  else             { lInsertionIndex = Add(crNewSolution); }
                   
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if (bDebugMe) 
    {
     Dump();
    }
#endif
  
  // all done
  return(lInsertionIndex) ;

} // end SmSolutionArray::AddSortedSolution

/*******************************************************************//**
PURPOSE: Copy the contents of one array to the other.

NOTES: 
***********************************************************************/
void SmSolutionArray::Copy(const SmSolutionArray& crSource)
{
  SM_ASSERT(this != &crSource);   // cannot append to itself

  SetSize(crSource.m_lSize);

  // okay to use smos_MemCpy on static class (SmSolution and SmSolutionEnd) objects.
  SE(smos_MemCpy(m_pData, crSource.m_pData, crSource.m_lSize * sizeof(SmSolution), crSource.m_lSize * sizeof(SmSolution)));

} // end SmSolutionArray::Copy

/////////////////////////////////////////////////////////////////////////////

/*******************************************************************//**
PURPOSE: Set the value of the array at a given index and grow if necessary.

NOTES: 
***********************************************************************/
void SmSolutionArray::SetAtGrow
  (ULONG nIndex, 
   const SmSolution & crNewSolution)
{
    if (nIndex >= m_lSize)
        SetSize(nIndex+1);
    m_pData[nIndex] = crNewSolution;

} // end SmSolutionArray::SetAtGrow

/*******************************************************************//**
PURPOSE: Insert a value into the array at a given position a given
   number of times.

NOTES: 
***********************************************************************/
void SmSolutionArray::InsertAt
  (ULONG nIndex, 
   const SmSolution & crNewSolution, 
   ULONG nCount)
{
    if (nIndex >= m_lSize)
      {
        // adding after the end of the array
        SetSize(nIndex + nCount);  // grow so nIndex is valid
      }
    else
      {
        // inserting in the middle of the array
        ULONG nOldSize = m_lSize;
        SetSize(m_lSize + nCount);  // grow it to new size

        // shift old data up to fill gap
        // okay to use smos_MemMove on static class (SmSolution and SmSolutionEnd) objects.
        smos_MemMove(&m_pData[nIndex+nCount], 
                     &m_pData[nIndex],
                     (nOldSize-nIndex) * sizeof(SmSolution));

        // re-init slots we copied from
        // okay to use smos_MemSet on static class (SmSolution and SmSolutionEnd) objects.
        smos_MemSet(&m_pData[nIndex], 0, nCount * sizeof(SmSolution));

      }

    // insert new value in the gap
    SM_ASSERT(nIndex + nCount <= m_lSize);
    while (nCount--)
        m_pData[nIndex++] = crNewSolution;

} // end SmSolutionArray::InsertAt

/*******************************************************************//**
PURPOSE: Remove one or more values of an array at a given index.

NOTES: 
***********************************************************************/
void SmSolutionArray::RemoveAt(ULONG nIndex, ULONG nCount)
{
  SM_ASSERT(nIndex + nCount <= m_lSize);

  // just remove a range
  if (m_lSize > nIndex + nCount) 
    {
      ULONG nMoveCount = m_lSize - (nIndex + nCount);
      for (ULONG i=0; i<nMoveCount; i++) 
        {
          m_pData[i+nIndex] = m_pData[i+nIndex+nCount];
        }
//        smos_Memmove(&m_pData[nIndex], &m_pData[nIndex + nCount], nMoveCount * sizeof(TYPE));
    }

  // adjust the size
  m_lSize -= (nCount <= m_lSize) ? nCount : m_lSize ;

} // end SmSolutionArray::RemoveAt

/*******************************************************************//**
PURPOSE: Remove duplicate solutions from the array.

NOTES:  The array of solutions is traversed to check for intersections
        with the same parameter value on a specific object. 
        This may happen when an curve intersects a closed curve 
        through the start/end, resulting in two intersections when
        there is only one 3d intersection. This may happen when
        two closed curves intersect at their start/end.
        It is not necessary to actually evaluate and check the 3d data
***********************************************************************/
void SmSolutionArray::RemoveDuplicateSolutions()
{
    SmSolution sSol1, sSol2;
    double sParam;

    // Look for duplicate parameter values on first object
    ULONG ii ;
    for( ii = 0; ii < GetSize(); ii++ ) {
        sSol1 = this->GetAt( ii );
        sParam = sSol1.m_vStart[0];
        
        for( ULONG jj = ii + 1; jj < GetSize(); jj++ ) {
            sSol2 = this->GetAt( jj );
            if( sParam == sSol2.m_vStart[0] )
                this->RemoveAt( jj );
        }
    }

    // Look for duplicate parameter values on second object
    for( ii = 0; ii < GetSize(); ii++ ) {
        sSol1 = this->GetAt( ii );
        sParam = sSol1.m_vStart[1];
        
        for( ULONG jj = ii + 1; jj < GetSize(); jj++ ) {
            sSol2 = this->GetAt( jj );
            if( sParam == sSol2.m_vStart[1] )
                this->RemoveAt( jj );
        }
    }

}


/*******************************************************************//**
PURPOSE: Swap two SmSolution objects within the existing array

NOTES: 
***********************************************************************/
void SmSolutionArray::Swap
 (ULONG nIndex1, 
  ULONG nIndex2)
{
  SM_ASSERT(nIndex1 < m_lSize);
  SM_ASSERT(nIndex2 < m_lSize);

  // when needed, swap the solutions
  if(nIndex1 != nIndex2)
    {
      SmSolution sTmp  = m_pData[nIndex1] ;
      m_pData[nIndex1] = m_pData[nIndex2] ;
      m_pData[nIndex2] = sTmp ;
    }

} // end SmSolutionArray::Swap

/*******************************************************************//**
PURPOSE: Insert the contents of one array into an other at the given
   start index.

NOTES: 
***********************************************************************/
void SmSolutionArray::InsertAt
 (ULONG                   nStartIndex, 
  const SmSolutionArray & crArrayToInsert)
{
  if (crArrayToInsert.GetSize() > 0)
    {
      InsertAt(nStartIndex, crArrayToInsert.GetAt(0), crArrayToInsert.GetSize());
      for (ULONG i = 0; i < crArrayToInsert.GetSize(); i++)
          SetAt(nStartIndex + i, crArrayToInsert.GetAt(i));
    }

} // end SmSolutionArray::InsertAt


/////////////////////////////////////////////////////////////////////////////
// Diagnostics

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmSolution::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // value and parameters
  if (m_eSolutionType == SM_ST_SINGLE_VALUE) 
    {
      smos_sprintf(sBuff,_T("Type - SM_ST_SINGLE_VALUE, Value - %16.16lf\n"),m_vStart.m_dSolutionValue);
      smos_WriteBuffer(sBuff);
      smos_WriteBuffer(_T("\t        Start Parameters - "));
      for (ULONG j=0; j<m_lNumVariables; j++) 
        {
          smos_sprintf(sBuff, _T("[%ld] = %16.16lf, "),j,m_vStart[j]);
          smos_WriteBuffer(sBuff);
        }
      smos_WriteBuffer(_T("\n"));
    }
  else if(m_eSolutionType == SM_ST_RANGE_OF_VALUES)
    {
      smos_sprintf(sBuff, _T("Type - SM_ST_RANGE_OF_VALUES, Start Value - %16.16lf,  End Value - %16.16lf\n"),
          m_vStart.m_dSolutionValue,m_vEnd.m_dSolutionValue);
      smos_WriteBuffer(sBuff);
      smos_WriteBuffer(_T("\t        Start Parameters -"));
      for (ULONG j=0; j<m_lNumVariables; j++) 
        {
          smos_sprintf(sBuff, _T("[%ld] = %16.16lf, "),j,m_vStart[j]);
          smos_WriteBuffer(sBuff);
        }
      smos_WriteBuffer(_T("\n"));
      smos_WriteBuffer(_T("\t        End   Parameters -"));
      for (ULONG jj=0; jj<m_lNumVariables; jj++) 
        {
          smos_sprintf(sBuff, _T("[%ld] = %16.16lf, "),jj,m_vEnd[jj]);
          smos_WriteBuffer(sBuff);
        }
      smos_WriteBuffer(_T("\n"));
    }
  else if(m_eSolutionType == SM_ST_POLY_SINGLE_VALUE)
    {
      smos_sprintf(sBuff,_T("Type - SM_ST_POLY_SINGLE_VALUE, Value - %16.16lf\n"),m_vStart.m_dSolutionValue);
      smos_WriteBuffer(sBuff);
      smos_sprintf(sBuff, _T("\t        Object1 Position - [%16.16lf, %16.16lf, %16.16lf]"),
                 m_vStart[0], m_vStart[1], m_vStart[2]);
      smos_WriteBuffer(sBuff);
      if(m_lNumVariables > 3)
        {
          smos_sprintf(sBuff, _T("\n\t        %s Position - [%16.16lf, %16.16lf, %16.16lf]\n"),
                     (m_lNumObjects > 1) ? _T("Object2") : _T("Second "),
                     m_vStart[3], m_vStart[4], m_vStart[5]);
          smos_WriteBuffer(sBuff);
        }
      smos_WriteBuffer(_T("\n"));
    }
  else if(m_eSolutionType == SM_ST_NODES_ONLY)   
    {
      smos_sprintf(sBuff,_T("%s"),_T("Type - SM_ST_NODES_ONLY\n"));
      smos_WriteBuffer(sBuff);

      // TODO: add appropriate output here
    }
  else
    {
      // SM_ASSERT(SM_ST_UNKNOWN == SM_ST_NODES_ONLY) ;     // THIS LOOKED LIKE A TYPO. Please review
      SM_ASSERT(m_eSolutionType == SM_ST_UNKNOWN);
      smos_sprintf(sBuff,_T("%s"),_T("Type - SM_ST_UNKNOWN, The uninitialized value\n"));
      smos_WriteBuffer(sBuff);
    }

  // objects
  smos_sprintf(sBuff,       _T("          %ld Objects    = "), m_lNumObjects) ;
  smos_WriteBuffer(sBuff);
  ULONG ii ;
  for(ii=0;ii<m_lNumObjects;ii++) 
    {
      smos_sprintf(sBuff,       _T("0x%p[%s], "), 
                 m_apObjects[ii],
                 m_apObjects[ii] ? m_apObjects[ii]->GetTypeString() 
                                 //        (  m_apObjects[ii]->IsKindOf(SmFace_TYPE)    ? _T("SmFace") 
                                 //         : m_apObjects[ii]->IsKindOf(SmEdge_TYPE)    ? _T("SmEdge")
                                 //         : m_apObjects[ii]->IsKindOf(SmVertex_TYPE)  ? _T("SmVertex")
                                 //         : m_apObjects[ii]->IsKindOf(SmCurve_TYPE)   ? _T("SmCurve")
                                 //         : m_apObjects[ii]->IsKindOf(SmSurface_TYPE) ? _T("SmSurface")
                                 //         : m_apObjects[ii]->IsKindOf(SmVolume_TYPE)  ? _T("SmVolume")
                                 //         : m_apObjects[ii]->IsKindOf(SmPolyBrep_TYPE  ) ? _T("SmPolyBrep")
                                 //         : m_apObjects[ii]->IsKindOf(SmPolyRegion_TYPE) ? _T("SmPolyRegion")
                                 //         : m_apObjects[ii]->IsKindOf(SmPolyShell_TYPE ) ? _T("SmPolyShell")
                                 //         : m_apObjects[ii]->IsKindOf(SmCPolyFace_TYPE ) ? _T("SmCPolyFace")
                                 //         : m_apObjects[ii]->IsKindOf(SmPolyFace_TYPE  ) ? _T("SmPolyFace")
                                 //         : m_apObjects[ii]->IsKindOf(SmPolyLoop_TYPE  ) ? _T("SmPolyLoop")
                                 //         : m_apObjects[ii]->IsKindOf(SmPolyEdge_TYPE  ) ? _T("SmPolyEdge")
                                 //         : m_apObjects[ii]->IsKindOf(SmPolyVertex_TYPE) ? _T("SmPolyVertex")
                                 //         : _T("UnExpected"))
                                 : _T("NULL")) ;
      smos_sprintf(sBuffForFile,_T("%s[%s], "), 
                 m_apObjects[ii] ? _T("notNULL") : _T("NULL"),
                 m_apObjects[ii] ? m_apObjects[ii]->GetTypeString()
                                  //       (  m_apObjects[ii]->IsKindOf(SmFace_TYPE)    ? _T("SmFace") 
                                  //        : m_apObjects[ii]->IsKindOf(SmEdge_TYPE)    ? _T("SmEdge")
                                  //        : m_apObjects[ii]->IsKindOf(SmVertex_TYPE)  ? _T("SmVertex")
                                  //        : m_apObjects[ii]->IsKindOf(SmCurve_TYPE)   ? _T("SmCurve")
                                  //        : m_apObjects[ii]->IsKindOf(SmSurface_TYPE) ? _T("SmSurface")
                                  //        : m_apObjects[ii]->IsKindOf(SmVolume_TYPE)  ? _T("SmVolume")
                                  //        : m_apObjects[ii]->IsKindOf(SmPolyBrep_TYPE  ) ? _T("SmPolyBrep")
                                  //        : m_apObjects[ii]->IsKindOf(SmPolyRegion_TYPE) ? _T("SmPolyRegion")
                                  //        : m_apObjects[ii]->IsKindOf(SmPolyShell_TYPE ) ? _T("SmPolyShell")
                                  //        : m_apObjects[ii]->IsKindOf(SmCPolyFace_TYPE ) ? _T("SmCPolyFace")
                                  //        : m_apObjects[ii]->IsKindOf(SmPolyFace_TYPE  ) ? _T("SmPolyFace")
                                  //        : m_apObjects[ii]->IsKindOf(SmPolyLoop_TYPE  ) ? _T("SmPolyLoop")
                                  //        : m_apObjects[ii]->IsKindOf(SmPolyEdge_TYPE  ) ? _T("SmPolyEdge")
                                  //        : m_apObjects[ii]->IsKindOf(SmPolyVertex_TYPE) ? _T("SmPolyVertex")
                                  //        : _T("UnExpected"))
                                 : _T("NULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile);
    }
  smos_WriteBuffer(_T("\n"));

  // tree nodes  
  smos_sprintf(sBuff,       _T("          %ld Tree Nodes = "), m_lNumObjects) ;
  smos_WriteBuffer(sBuff);
  for(ii=0;ii<m_lNumObjects;ii++) 
    {
      smos_sprintf(sBuff,       _T("0x%p, "), m_apNodes[ii]) ;
      smos_sprintf(sBuffForFile,_T("%s, "), m_apNodes[0] ? _T("notNULL") : _T("NULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile);
    }
  smos_WriteBuffer(_T("\n"));

} // end SmSolution::Dump

/*******************************************************************//**
PURPOSE: This is the version of the SmSolution::Draw
            method which displays curve/point graphics for solutions.

NOTES: The method references classes SmFace and SmEdge to 
  generate graphics.

***********************************************************************/
SmDisplayList * SmSolution::Draw
 (ULONG l1stObject1stParamIndex)  // in : in almost all cases this will be 0
                                  //      for ray firing solutions in which the
                                  //       ray is not stored as its own object
                                  //       but the m_vStart[0] = ray param value
                                  //       and the m_vStart[2] = u param for surf,
                                  //       then this value needs to be set to 1.
                                  //      default:[0]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor());

  // output graphics locals
  ULONG lParamCount = l1stObject1stParamIndex ;
  double dParam1, dParam2 ;
  SmObject  *pObject ;
  SmEdge    *pEdge ;   // not allowed in GSLib - all pEdge references removed in the GSLib version of this draw function
  SmFace    *pFace ;   // not allowed in GSLib - all pFace references removed in the GSLib version of this draw function
  SmCurve   *pCurve ;
  SmSurface *pSurface ;

  // when solution is SingleValue
  if (m_eSolutionType == SM_ST_SINGLE_VALUE) 
    {
      SmPoint2d sPoint2d ;
      SmPoint3d sPoint3d ;

      // for every Solution Object - output a point graphic
      for(ULONG ii=0;ii<m_lNumObjects;ii++)
        {
          // get edge/face pointer if possible
          pObject = m_apObjects[ii] ;
          if(pObject == NULL) { // assume NULL object was a tmp line (since that's the only place that sets this value to NULL right now)
                                lParamCount++ ;
                                continue ;
                              }
// Remove Composites
//          pEdge = (   pObject->IsKindOf(SmEdge_TYPE) || pObject->IsKindOf(SmCEdge_TYPE)) ? (SmEdge *)pObject : NULL ;  
//          pFace = (   pObject->IsKindOf(SmFace_TYPE) || pObject->IsKindOf(SmCFace_TYPE)) ? (SmFace *)pObject : NULL ;  
          pEdge = (pObject->IsKindOf(SmEdge_TYPE)) ? (SmEdge *)pObject : NULL ;  
          pFace = (pObject->IsKindOf(SmFace_TYPE)) ? (SmFace *)pObject : NULL ;  

          // When Object is an Edge/Curve Pointer
          if(pEdge || pObject->IsKindOf(SmCurve_TYPE))
            {
              // skip NULL curves
              pCurve = pEdge ? pEdge->GetCurve() : (SmCurve *)pObject ;
              if(!pCurve) continue ;

              // get CurveParam and CurvePoint
              dParam1 = m_vStart[lParamCount] ; lParamCount++ ;
              pCurve->EvaluatePoint(dParam1,sPoint3d) ;

              // draw CurvePoint
              if(pCurve->GetDim() == 2) { sPoint2d.Set(sPoint3d.x, sPoint3d.y) ;
                                          sPoint2d.Draw() ;
                                        }
              if(pCurve->GetDim() == 3) { sPoint3d.Draw() ;
                                        }
            } // end Edge or Curve branch 

          // When Object is a Face/Surface Pointer
          if(pFace || pObject->IsKindOf(SmSurface_TYPE))
            {
              // skip NULL surfaces
              pSurface = pFace ? pFace->GetSurface() : (SmSurface *)pObject ;
              if(!pSurface) continue ;

              // get SurfaceParam and SurfacePoint
              dParam1 = m_vStart[lParamCount] ; lParamCount++ ;
              dParam2 = m_vStart[lParamCount] ; lParamCount++ ;
              sPoint2d.Set(dParam1, dParam2) ;
              pSurface->EvaluatePoint(sPoint2d,sPoint3d) ;
              
              // draw SurfacePoint
              sPoint3d.Draw() ;
            } // end Face or Surface branch 
        } // end iter every object
    } // end SM_ST_SINGLE_VALUE branch 
  else if(m_eSolutionType == SM_ST_RANGE_OF_VALUES) // SM_ST_RANGE_OF_VALUES
    {
      // for every Solution Object - output a point graphic
      for(ULONG ii=0;ii<m_lNumObjects;ii++)
        {
          // get edge/face pointer if possible
          pObject = m_apObjects[ii] ;
          if(pObject == NULL) { // assume NULL object was a tmp line (since that's the only place that sets this value to NULL right now)
                                lParamCount++ ;
                                continue ;
                              } 

// Remove Composites
//          pEdge = (pObject->IsKindOf(SmEdge_TYPE ) || pObject->IsKindOf(SmCEdge_TYPE)) ? (SmEdge *)pObject : NULL ;
//          pFace = (pObject->IsKindOf(SmFace_TYPE ) || pObject->IsKindOf(SmCFace_TYPE)) ? (SmFace *)pObject : NULL ;
          pEdge = (pObject->IsKindOf(SmEdge_TYPE )) ? (SmEdge *)pObject : NULL ;
          pFace = (pObject->IsKindOf(SmFace_TYPE )) ? (SmFace *)pObject : NULL ;

          // When Object is an Edge/Curve Pointer
          if(pEdge || pObject->IsKindOf(SmCurve_TYPE))
            {
              // skip NULL curves
              pCurve = pEdge ? pEdge->GetCurve() : (SmCurve *)pObject ;
              if(!pCurve) continue ;

              // get CurveParam Interval
              dParam1 = m_vStart[lParamCount] ; 
              dParam2 = m_vEnd[lParamCount] ;   
              lParamCount++ ;
              SmExtent1d sIvl(smos_Min(dParam1, dParam2),
                              smos_Max(dParam1, dParam2)) ;

              // draw CurveInterval
              pCurve->DrawWDeriv(sIvl) ;

            } // end Edge or Curve branch 

          // When Object is a Face/Surface Pointer
          if(pFace || pObject->IsKindOf(SmSurface_TYPE))
            {
              // skip NULL surfaces
              pSurface = pFace ? pFace->GetSurface() : (SmSurface *)pObject ;
              if(!pSurface) continue ;

              // get SurfaceParam and SurfacePoint
              SmPoint2d sUV1 = SmPoint2d(m_vStart[lParamCount],
                                         m_vStart[lParamCount+1]) ; 
              SmPoint2d sUV2 = SmPoint2d(m_vEnd[lParamCount],
                                         m_vEnd[lParamCount+1]) ;
              SmVector3d sSurfPoint1 ;
              SmVector3d sSurfPoint2 ;
              pSurface->EvaluatePoint(sUV1, sSurfPoint1) ; 
              pSurface->EvaluatePoint(sUV2, sSurfPoint2) ; 
           
              // increment the parameter space              
              lParamCount += 2 ;
              //SmPoint2d sMaxPoint2d(dParam1, dParam2) ;
           
              // draw Surface and solution end points
              pSurface->DrawUV() ;
              sSurfPoint1.Draw() ;
              sSurfPoint2.Draw() ;

            } // end Face or Surface branch 
        } // end iter every object
    } // end SM_ST_RANGE_OF_VALUES branch
  else if (m_eSolutionType == SM_ST_POLY_SINGLE_VALUE) 
    {
      SmPoint3d sPoint3d ;

      // for every Solution Object - output a point graphic
      for(ULONG ii=0;ii<m_lNumObjects;ii++)
        {
          SM_ASSERT(ii*3+2 < m_lNumVariables) ;
          // set and draw the point
          sPoint3d.Set(m_vStart[ii*3+0], m_vStart[ii*3+1], m_vStart[ii*3+2]) ;
          sPoint3d.Draw() ;

        } // end iter every Object
    } // end SM_ST_POLY_SINGLE_VALUE branch 

  // end display list
  pRtn = smgfx_Close() ;

#else
  SM_REF1(l1stObject1stParamIndex);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmSolution::Draw version (references SmEdge and SmFace)


/*******************************************************************//**
PURPOSE: Return the point value at a given index in the array.  

NOTES: The index must be less than the current size.
***********************************************************************/
SmSolution SmSolutionArray::GetAt( ULONG nIndex ) const
{
  SM_ASSERT_BREAK( nIndex < m_lSize );
  return m_pData[nIndex];
}

/*******************************************************************//**
PURPOSE: Set the solution value at a given index in the array.

NOTES: The index must be less than the current size.  To add
    elements to the end of the list see SmSolutionArray::Add.
***********************************************************************/
void SmSolutionArray::SetAt
( 
  ULONG              nIndex,
  const SmSolution & crNewSolution 
)
{
  SM_ASSERT_BREAK( nIndex < m_lSize );
  if(&crNewSolution != &(m_pData[nIndex]))
  {
    m_pData[nIndex] = crNewSolution;
  }
}

/*******************************************************************//**
PURPOSE: Append a new solution to the end of the array and increase the
   size of the allocated data array if necessary.

NOTES: 
***********************************************************************/
ULONG SmSolutionArray::Add( const SmSolution & crNewSolution )
{
  ULONG nIndex = m_lSize;
  if(nIndex < m_lMaxSize) { m_pData[nIndex] = crNewSolution; m_lSize++; }
  else { SetAtGrow( nIndex, crNewSolution ); }
  return nIndex;
}

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSolutionArray::IsKindOf( SM_TYPE t ) const
{
  return ((SmSolutionArray_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmSolutionArray::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\nSmSolutionArray size=%ld, alloc size=%ld  "),m_lSize,m_lMaxSize);
  smos_WriteBuffer(sBuff);
  SmObject::Dump();
  for (ULONG i = 0; i < m_lSize; i++) 
    {
      smos_sprintf(sBuff,_T("  [%ld] = "),i);
      smos_WriteBuffer(sBuff);
      m_pData[i].Dump();
    }

} // end SmSolutionArray::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmDisplayList * SmSolutionArray::Draw() const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(this));

  // draw every solution
  for (ULONG i = 0; i < m_lSize; i++) 
    {
      m_pData[i].Draw();
    }

  pRtn = smgfx_Close() ;

#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmSolutionArray::Draw

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertSolutionArray_list[] =
{
  {SM_AT_SIZE, _T("Bad SIZE"), _T("m_pData is NULL and m_lSize is zero - should be greater than zero") },
  {SM_AT_SIZE, _T("Bad SIZE"), _T("m_pData is NULL and m_lMaxSize is zero - should be greater than zero") },
  {SM_AT_SIZE, _T("Bad SIZE"), _T("m_pData is NotNULL and m_lSize greater than m_lMaxSize - should be less than or equal") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmSolutionArray::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  SmBoolean bRtn = TRUE ;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmObject::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ; 

  // gwc note: these checks could be duplicated for class SmTArray
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pData != NULL || m_lSize    == 0), _T("") ) ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (m_pData != NULL || m_lMaxSize == 0), _T("") ) ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (m_pData == NULL || m_lSize <= m_lMaxSize), _T("") ) ;

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmSolutionArray::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmSolutionArray::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport 
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return ( SmObject::AssertHeal(rAReport, pAList) ) ;
//     }
//    
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ; 
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//                rAReport.m_pHealMessage = _T("SmSolutionArray::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmSolutionArray::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Self test for classes SmSolutionArray, SmSolution, and SmSolutionEnd

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmSolutionArray::SelfTest()
{
  // locals
  ULONG ii ; 
  SmBoolean bRtn = TRUE ; 
  SmObject sObj1, sObj2, sObj3 ; 

  // class SmSolutionEnd SelfTest
  SmSolutionEnd sSolutionEnd1, sSolutionEnd2 ;
  bRtn &= sSolutionEnd1.m_dSolutionValue == 0.0 ;
  for(ii=0;ii<SM_SA_MAX_TREES*2;ii++)
    {
      bRtn &= sSolutionEnd1.m_adParameters[ii] == 0.0 ; 
    }

  sSolutionEnd2.m_dSolutionValue  = 7.0 ; 
  sSolutionEnd2.m_adParameters[0] = 1.0 ; 
  sSolutionEnd2.m_adParameters[1] = 2.0 ; 
  sSolutionEnd2.m_adParameters[2] = 3.0 ; 

  bRtn &= !(sSolutionEnd1 == sSolutionEnd2) ; 

  sSolutionEnd1 = sSolutionEnd2 ;
  bRtn &= (sSolutionEnd1 == sSolutionEnd2) ;
  
  SmSolutionEnd sSolutionEnd3(sSolutionEnd1) ;
  bRtn &= (sSolutionEnd1 == sSolutionEnd3) ;
   
  // SmSolution SelfTest

  // assign values
  sSolutionEnd1.m_dSolutionValue  = 4.0 ; 
  sSolutionEnd1.m_adParameters[0] = 1.0 ; 
  sSolutionEnd1.m_adParameters[1] = 2.0 ; 
  sSolutionEnd1.m_adParameters[2] = 3.0 ; 

  sSolutionEnd2.m_dSolutionValue  = 7.0 ; 
  sSolutionEnd2.m_adParameters[0] = 4.0 ; 
  sSolutionEnd2.m_adParameters[1] = 5.0 ; 
  sSolutionEnd2.m_adParameters[2] = 6.0 ; 

  sSolutionEnd3.m_dSolutionValue  = 11.0 ; 
  sSolutionEnd3.m_adParameters[0] = 7.0 ; 
  sSolutionEnd3.m_adParameters[1] = 8.0 ; 
  sSolutionEnd3.m_adParameters[2] = 9.0 ; 

  SmSolution sSolution1, sSolution2 ; 

  bRtn &= sSolution1.m_eSolutionType == SM_ST_UNKNOWN ;
  bRtn &= sSolution1.m_lNumVariables == 0 ;
  bRtn &= sSolution1.m_vStart.m_dSolutionValue == 0.0 ;
  for(ii=0;ii<SM_SA_MAX_TREES*2;ii++)
    {
      bRtn &= sSolution1.m_vStart.m_adParameters[ii] == 0.0 ; 
    }
  bRtn &= sSolution1.m_vEnd.m_dSolutionValue == 0.0 ;
  for(ii=0;ii<SM_SA_MAX_TREES*2;ii++)
    {
      bRtn &= sSolution1.m_vEnd.m_adParameters[ii] == 0.0 ; 
    }
  bRtn &= sSolution1.m_lNumObjects   == 0 ;
  for(ii=0;ii<SM_SA_MAX_TREES;ii++)
    {
      bRtn &= sSolution1.m_apObjects[ii] == NULL ; 
      bRtn &= sSolution1.m_apNodes[ii]   == NULL ; 
    }

  // assign values
  sSolution1.m_lNumVariables = 3 ; 
  sSolution1.m_lNumObjects   = 2 ; 
  sSolution1.m_vStart = sSolutionEnd1 ;
  sSolution1.m_vEnd   = sSolutionEnd2 ;
  sSolution1.m_apObjects[0] = &sObj1 ;
  sSolution1.m_apObjects[1] = &sObj2 ;
  sSolution1.m_apObjects[2] = &sObj3 ;

  bRtn &= !(sSolution1 == sSolution2) ;
  
  sSolution2 = sSolution1 ; 
  bRtn &= (sSolution1 == sSolution2) ;

  SmSolution sSolution3(sSolution1) ;
  bRtn &= (sSolution1 == sSolution3) ;

  // SmSolutionArray selftest
  SmSolution sSolData[3] ;
  SmSolutionArray sSolutionArray1, sSolutionArray2(3, sSolData) ;

  bRtn &= sSolutionArray1.m_lSize       == 0 ;
  bRtn &= sSolutionArray1.m_lMaxSize    == 0 ;
  bRtn &= sSolutionArray1.m_pData       == NULL ;
  bRtn &= sSolutionArray1.m_bIsBorrowed == FALSE ;

  bRtn &= sSolutionArray2.m_lSize       == 0 ;
  bRtn &= sSolutionArray2.m_lMaxSize    == 3 ;
  bRtn &= sSolutionArray2.m_pData       == sSolData ;
  bRtn &= sSolutionArray2.m_bIsBorrowed == TRUE ;

  // assign values
  sSolution1.m_vStart = sSolutionEnd1 ;
  sSolution1.m_vEnd   = sSolutionEnd2 ;

  sSolution2.m_vStart = sSolutionEnd2 ;
  sSolution2.m_vEnd   = sSolutionEnd3 ;

  sSolution3.m_vStart = sSolutionEnd3 ;
  sSolution3.m_vEnd   = sSolutionEnd1 ;


  // assign values
  sSolutionArray1.Add(sSolution1) ;
  sSolutionArray1.Add(sSolution2) ;
  sSolutionArray1.Add(sSolution3) ;

  bRtn &= !(sSolutionArray1 == sSolutionArray2) ;

  // assign values
  sSolutionArray2 = sSolutionArray1 ; 
  bRtn &= (sSolutionArray1 == sSolutionArray2) ;

  // copy constructor
  SmSolutionArray sSolutionArray3(sSolutionArray1) ;
  bRtn &= (sSolutionArray1 == sSolutionArray3) ;

  // all done
  return(bRtn) ;

} // end SmSolutionArray::SelfTest


