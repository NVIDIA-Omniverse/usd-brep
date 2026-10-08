// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCurveCache.cpp
* PURPOSE: Source file for SmCurveCache methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmCurveCache.h>
#include <SmBSplineCurve.h>
#include <nurbs.h>
#include <SmNurbsCrv.h>
#include <SmGeomUtility.h>
#include <SmTree.h>
#include <SmAssertArray.h>
#include <SmEdge.h>

/*******************************************************************//**
PURPOSE: Get the Bezier polygon for a range of Bezier segments in the
    cache.

NOTES: This function is only used during the construction of the
                SmCurveCache::m_pTree and creates no need to store the
                Bezier Spans after the tree is constructed.
***********************************************************************/
static SmStatus sm_GetBezierPolygon
  (SmTArray<gw_CURVE *> &rLeafBeziers, // in : ordered list of leaf node Bezier curves
   ULONG lLeftSpan,                    // in : index of left most subDivision CurveSpan stored in rLeafBeziers 
   ULONG lRightSpan,                   // in : index of right most subDivision CurveSpan stored in rLeafBeziers
   SmTArray<SmPoint3d> & rPolygon)     // out: Array of all Control Points for all Bezier Spans
{
  SmPoint3d sEuclid;

  gw_CURVE     *pBez     = rLeafBeziers.GetAt(lLeftSpan) ;

  // init rPolygon with the 1st CPOINT from the 1st span
  gw_CPOINT *pCpoint = &pBez->pol->Pw[0];
  TO_EUCLID(*pCpoint,sEuclid);
  rPolygon.Add(sEuclid);

  // add nonoverlapping control-polygon vertices for the rest of the requested spans
  for (ULONG i=lLeftSpan; i<=lRightSpan; i++) 
    {
      pBez     = rLeafBeziers.GetAt(i) ;

      // add in all control-polygon vertices but the first (avoids duplication) 
      for (int j=1; j<=pBez->p; j++) 
        {
          gw_CPOINT *pCpnt = &pBez->pol->Pw[j];
          TO_EUCLID(*pCpnt,sEuclid);
          rPolygon.Add(sEuclid);
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmCurveCache::GetBezierPolygon or sm_GetBezierPolygon

/*******************************************************************//**
PURPOSE: set the SmBezierSpan Parent member values from the children
  values.  This is a propagate-up method.

NOTES: 
  1. Requires: The gwCURVE part of the parent and children are complete.
  2. Requires: The children Bezier Spans are complete.
  3. Sets all parent properties except
     3a. Does not set m_dMaxTurningAngleDeg when SmCurveCache::m_bMakeFullTree == TRUE
     3b. Does not set m_sPseudoBox 
***********************************************************************/
SmStatus sm_ComputeParentData
  (SmCurveCache *pCurveCache,             // in : containing cache
   SmBezierAux1d *pParent,                // i/o: Parent Object whose values are to be set
   SmBezierAux1d *pLeft,                  // in : left child data
   SmBezierAux1d *pRight,                 // in : right child data
   SmTArray<gw_CURVE *> &rLeafBeziers,    // in : ordered list of leaf node bezier curves
   SmBoolean bMakeFullTree)               // in : TRUE = skip computing parent->m_dMaxChordHeightSquared
{
  // propagate simple values from children to parents
  pParent->m_lLeft                  =   pLeft->m_lLeft;
  pParent->m_lRight                 =   pRight->m_lRight;
  pParent->m_aeConts[0]             =   pLeft->m_aeConts[0];
  pParent->m_aeConts[1]             =   pRight->m_aeConts[1];
  pParent->m_dMaxLength             =   pLeft->m_dMaxLength 
                                      + pRight->m_dMaxLength;
  pParent->m_dMaxTurningAngleDeg    =   pLeft->m_dMaxTurningAngleDeg
                                      + pRight->m_dMaxTurningAngleDeg;
  pParent->m_dMaxChordHeightSquared = 0.0;

  pParent->m_sIvl.SetMinMax(pLeft->m_sIvl.GetMin(), pRight->m_sIvl.GetMax()) ;
  
  // When NOT making a full tree - compute parent MaxChordHeightSquared 
  //  we need it later to decide when two children nodes can be collapsed into one parent 
  if (!bMakeFullTree) 
    {
      SmPoint3d sPData[256];
      SmTArray<SmPoint3d> sPolygon(256,sPData);

      // get control-polygon for all beziers from left to right spans
      SER(sm_GetBezierPolygon(rLeafBeziers, pParent->m_lLeft, pParent->m_lRight, sPolygon));

      // get parent interval, startPoint, endPoint values
      SmPoint3d sStart, sEnd;
      SmExtent1d sIvl;
      pParent->GetEnds(*pCurveCache,sStart,sEnd,sIvl);

      // define span chord for chord height test
      SmVector3d sLineVec = sEnd-sStart;

      // unitize chord vector - protect against zero lengths
      if (sLineVec.LengthSquared() < SM_EFF_ZERO_SQRT) 
        { sLineVec.Set(1,0,0); // Chose any old vector
        }
      else 
        { sLineVec.Unitize(); 
        }

      // find max chord height to polygon vertices
      double dMaxChordSq = 0.0;
      for (ULONG i=0; i<sPolygon.GetSize(); i++) 
        {
          // find closest point on baseline to current polygon vertex
          double      dParam;
          SmPoint3d & rPnt      = sPolygon[i];
          SER(smgu_LineClosestPoint(sStart,sLineVec,rPnt,dParam));

          // get chord height for this polygon vertex
          SmPoint3d   sFoundPnt = sStart + dParam * sLineVec;
          double      dDistSq   = sFoundPnt.DistanceBetweenSquared(rPnt);

          // save max chord height
          if (dDistSq > dMaxChordSq) 
            { dMaxChordSq = dDistSq; }
        } // end iter every polygon vertex

      // set parent m_dMaxChordHeightSquared value
      pParent->m_dMaxChordHeightSquared = dMaxChordSq;

    } // end need to compute parentbMakeFullTree == FALSE check

  // when child-child continuity is not C1 or better
  // increment parent->m_dMaxTurningAngleDeg by angle at the children joint point
  if (pLeft->m_aeConts[1] < SM_CT_G1) 
    {
      // We have a derivative discontinuity
      // Need to add angle between tangents at end positions
      SmBezierSpan *pBezSpanRightOfLeft = pCurveCache->GetAt(pLeft->m_lRight);
      SmBezierSpan *pBezSpanLeftOfRight = pCurveCache->GetAt(pRight->m_lLeft);

      SmExtent1d            sExt1, sExt2 ;     
      const SmBSplineCurve *pLeftCurve, *pRightCurve ;

      sExt1       = pBezSpanRightOfLeft->GetInterval();
      sExt2       = pBezSpanLeftOfRight->GetInterval();
      pLeftCurve  = (const SmBSplineCurve *)pCurveCache->GetCurve() ; 
      pRightCurve = pLeftCurve ;
      SM_ASSERT(SM_ARE_SAME(sExt1.Evaluate(1.0), sExt2.Evaluate(0.0))) ;

      // evaluate tangents at same positions
      // Note, the left/right flags are significant here.
      SmPoint3d sPVLeft[2], sPVRight[2];
      SER(pLeftCurve->Evaluate (sExt1.Evaluate(1.0), 1, FALSE, sPVLeft, TRUE));   // nonZeroTangents
      SER(pRightCurve->Evaluate(sExt2.Evaluate(0.0), 1, TRUE, sPVRight, TRUE));   // nonZeroTangents
      double dAngle = 0;

      // add in change in tangent angle
      //SER(sPVLeft[1].AngleBetween(sPVRight[1],dAngle));
      SM_ASSERT(sPVRight[0].CloserThan(SM_EFF_ZERO, sPVLeft[0])) ;
      if(   sPVLeft[1].LengthSquared()  > SM_EFF_ZERO_SQ     // in case Curve has double control points on the end
         && sPVRight[1].LengthSquared() > SM_EFF_ZERO_SQ
         && SM_SUCCESS == sPVLeft[1].AngleBetween( sPVRight[1], dAngle ))
        {
          pParent->m_dMaxTurningAngleDeg += dAngle * 180.0 / SM_PI;
        }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(pLeftCurve) ;
          SM_DUMP_AND_ASSERT_VALID(pRightCurve) ;
        }
#endif // SM_DEBUG_CODE
    }

  return SM_SUCCESS;

} // end sm_ComputeParentData

/*******************************************************************//**
PURPOSE: Get (create if necessary) an analytical curve (SmLine or SmCircle)
         for this curve cache.

NOTES: Returns NULL when the curve is not represented by an analytic
***********************************************************************/
const SmCurve * SmCurveCache::GetAnalyticCurve()
{
  if(m_crCurve.IsAnalytic())
    { return &m_crCurve ; }

  if(    m_pAnalyticalCurve==NULL 
     && !m_bTriedToCreateAnalytical) 
    {
      // See if SmBSplineCurve can be represented by a SmLine or a SmCircle to with SM_EFF_ZERO tolerance
      if(SM_SUCCESS != m_crCurve.CreateAnalyticCurve(*m_cpContext,m_sInterval,m_pAnalyticalCurve)) 
        {
          m_pAnalyticalCurve = NULL;
        }
      m_bTriedToCreateAnalytical = TRUE;
    }

  // all done
  return m_pAnalyticalCurve;

} // end SmCurveCache::GetAnalyticCurve

/*******************************************************************//**
PURPOSE: Get curve for this curve cache.

NOTES: 
***********************************************************************/
const SmBSplineCurve * SmCurveCache::GetBSplineCurve()      
  const 
{ 
  SmBSplineCurve *pBSC = SM_CAST_NONNULL_PTR(SmBSplineCurve, &m_crCurve) ;

  SM_ASSERT(pBSC == NULL || m_pTree != NULL) ;

  return pBSC; 

} // end SmCurveCache::GetBSplineCurve

/*******************************************************************//**
PURPOSE: Get Brep for this curve or return NULL

NOTES: 
***********************************************************************/
SmBrep * SmCurveCache::GetBrep()      
  const 
{ 
    const SmCurve *pCurve = GetCurve() ;

    return (SmBrep *)(  pCurve->GetOwner() 
                    ? (  pCurve->GetOwner()->IsKindOf(SmEdge_TYPE)
                       ? ((SmEdge *)pCurve->GetEdge())->GetBrep()
                       : NULL)
                    : NULL ) ;

} // end SmCurveCache::GetBrep

//      /*******************************************************************//**
//      PURPOSE: Transform a curve cache.
//      
//      NOTES: 
//        gwc: This function does not seem to get called and its not
//             virtual and it depends on the large curve cache scheme.
//             So, its being removed
//      
//      ***********************************************************************/
//      SmStatus SmCurveCache::Transform
//        (const SmAxis2Placement & crRotateNMove, // in : affine rotate and move transformation      
//         const SmVector3d       * cpOptScale)    // in : optional scaling about current origin point before RotateNMove
//                                                 //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
//                                                 //      other geom types only support isoptropic scaling
//      {
//        if (m_pAnalyticalCurve) 
//          {
//            m_pAnalyticalCurve->Transform(crRotateNMove,cpOptScale);
//          }
//      
//        NL_STACKS SC;
//        RMATRIX rma;
//        SmStackHandler sStackKp(&SC);
//        N_InitRealMatrix(&rma);
//        N_SetRealMatrix(&rma,3,3,full,3,&SC);
//        gw_REAL  **RM = rma.RM;
//      
//        crRotateNMove.GetMatrix(RM);
//      
//        SmVector3d sScale(1,1,1);
//        if (cpOptScale) sScale = *cpOptScale;
//      
//        RM[0][0] *= sScale.x;
//        RM[0][1] *= sScale.y;
//        RM[0][2] *= sScale.z;
//      
//        RM[1][0] *= sScale.x;
//        RM[1][1] *= sScale.y;
//        RM[1][2] *= sScale.z;
//      
//        RM[2][0] *= sScale.x;
//        RM[2][1] *= sScale.y;
//        RM[2][2] *= sScale.z;
//      
//        // First transform leaf nodes and then work bounding boxes up the tree.
//        ULONG lCrvDim    = m_crCurve.GetDim();
//        ULONG lSpanCount = GetSpanCount();
//        for (ULONG i=0; i<lSpanCount; i++) 
//          {
//            SmTreeNode   *pNode    = (SmTreeNode*)m_pTree->m_sNodeMgr.GetAt(i);
//            SmBezierSpan *pBezSpan = this->GetAt(i);
//            pBezSpan->m_bPseudoBoxComputed = FALSE;
//            gw_CURVE *crv = pBezSpan->GetBezierPtr();
//            N_CrvTransform(crv,&rma);
//            SmBSplineCurve sBSP(crv,lCrvDim,TRUE);
//            sBSP.CalculateBoundingBox(sBSP.GetNaturalInterval(),&pNode->m_sBBox,NULL);
//          }
//      
//        SM_PTR_ARRAY(sNodes,SmTreeNode,256);
//        if (!m_pTree) SER(SM_ERR);
//        m_pTree->GetAllTreeNodes(sNodes);
//      
//        // Initialize BBoxes on upper level nodes.
//        for (ULONG j=lSpanCount; j<sNodes.GetSize(); j++) 
//          {
//            sNodes[j]->m_sBBox.Init();
//          }
//      
//        // Now propagate Boxes to top of tree.
//        for (ULONG k=0; k<sNodes.GetSize(); k++) 
//          {
//            SmTreeNode *pNode = sNodes[k];
//            SmTreeNode *pParent = pNode->m_pParent;
//            if (pParent) 
//              {
//                pParent->m_sBBox.Union(pNode->m_sBBox,pParent->m_sBBox);
//              }
//          }
//        
//        return SM_SUCCESS;
//      
//      } // end SmCurveCache::Transform

/*******************************************************************//**
PURPOSE: Constructor which creates a tree and its node elements from
            the children subdivision segments created for a curve 
            in a curve cache object.

NOTES:
  1. Don't call this function directly.  Instead call
     SmCurveCache::Tessellate() which calls this function as its last step.
  
  2. When called by Tessellate(), 
     a. The curve has been sub-divided into a set of spans each of which
        passes all the tessellation control tests.
     b. Each curve span is associated with a SmBezierSpan stored in the
        m_sBezMgr object.  
     c. Each SmBezierSpan stores a Bezier curve and some curve properties.


SIDE EFFECTS ---
  1. allocates and store m_pTree
  2. allocates nodes (twice the number of spans in m_sBezMgrnodes) in m_pTree->m_sNodeMgr
  3. when
***********************************************************************/
SmStatus SmCurveCache::BuildTree
  (SmTArray<gw_CURVE *> &rLeafBeziers)       // in : ordered array of leaf bezier curve pointers
{
  // allocate and store SmTree Object
  m_pTree = new (*m_crCurve.GetContext()) SmTree(this);

  // locals
  SmCurveCache * pCurveCache = this;
  SmTree       * pTree       = m_pTree;
  ULONG          lCrvDim     = pCurveCache->m_crCurve.GetDim();

  // get number of SmBezierSpan segments in m_sBezMgr.  
  //   Each of these SmBezierSpans came from sub-dividing the original unique 
  //   knot spans of the curve until each span passed all the 
  //   tessellation tests.
  ULONG lSpanCount  = pCurveCache->GetSpanCount();
  if (lSpanCount == 0) 
    { SER(SM_ERR); }

  // decomposition control parameters
  double dChordHeightToleranceSquared = m_dChordHeightTolerance * m_dChordHeightTolerance;
  double dAngleToleranceInDegrees     = m_dAngleToleranceDeg;

  // Compute the maximum allowable step size
  double dMaxAllowableStep =   (m_lMinSegNumber > 1)
                             ? (m_sInterval.GetMax() - m_sInterval.GetMin()) / m_lMinSegNumber
                             : (m_sInterval.GetMax() - m_sInterval.GetMin()) ;

  // tell Node memory manager sizeof a node and number of nodes per single contiguous memory block allocation
  pTree->m_sNodeMgr.Initialize(ALIGN_SIZE(sizeof(SmTreeNode)), // in : size of one element
                               2 * lSpanCount) ;               // in : number of elements per block = total number of nodes in this tree
  
  // two local node lists
  SmTreeNode           *sData1[100],        *sData2[100];
  SmTArray<SmTreeNode*> sNodes1(100,sData1), sNodes2(100,sData2);
  sNodes1.SetSize(lSpanCount);
  sNodes2.SetSize(lSpanCount);

  // Current Generation and Next Generation Array pointers
  SmTArray<SmTreeNode*> * pCurrArray = &sNodes1; 
  SmTArray<SmTreeNode*> * pNextArray = &sNodes2; 

  // Build child generation of tree nodes
  //   - for every sub-division curve SmBezierSpan in m_sBezMgr
  //       make and init one LeafNode from memory in m_sNodeMgr.
  long lNodeCount = 0;
  for (ULONG i=0; i<lSpanCount; i++) 
    {
      // fetch next SmTreeNode from m_sNodeMgr
      SmTreeNode    *pNode    = (SmTreeNode*)pTree->m_sNodeMgr.GetAt(lNodeCount++);

      // fetch next SmBezierSpan from Span array
      SmBezierSpan  *pBezSpan = pCurveCache->GetAt(i);

      // compute pNode->BoundingBox = BezSpan->BoundingBox
      //         and PseudoBox      = BezSpan->PseudoBox
      SmBSplineCurve sBSP(rLeafBeziers.GetAt(i), lCrvDim, TRUE);
      sBSP.CalculateBoundingBox(sBSP.GetNaturalInterval(),&pNode->m_sBBox,&pBezSpan->GetPseudoBox());

      // init other node values
      pNode->m_pTree        = pTree;
      pNode->m_pParent      = NULL;
      pNode->m_pChild1      = NULL;
      pNode->m_pChild2      = NULL;
      // pNode->m_sBBox     = already set in previous CaculateBoundingBox() call.
      pNode->m_eAuxDataType = SM_AD_BEZIER_CURVE;
      pNode->m_pData        = pBezSpan;
      pNode->m_eGeomType    = SM_NG_DEFAULT;

      // Set current node generation Array pointer
      (*pCurrArray)[i]      = pNode;
    
    } // end iter every child BezierSpan building an associated child Tree Node

  // arrive here when: 
  //   one SmTreeNode in m_pTree->m_sNodeMgr has been initialized for every SmBezierSpan in m_sBezMgr.
  //   scratch Pointers to these nodes are listed in the pCurrArray.
  //   These nodes are the leafNodes of the tree being built.

  // Now build tree for leaf nodes.
  //   leaf     nodes aux data all come from m_sBezMgr.
  //   internal nodes aux data all come from m_sParentMgr.

  // Tell m_sParentMgr size of each internal Node
  // For now the auxilliary data will not contain Bezier and Pseudo box data
  pCurveCache->m_sParentMgr.Initialize(ALIGN_SIZE(  sizeof(SmBezierAux1d)),   // in: size of one element
                                       lSpanCount);                            // in : number of elements per block

  // when using them, have m_sParentMgr pre-allocate all the SmParentSpans needed for internal nodes
  if(pCurveCache->m_bComputeAuxTreeData) 
    {
      pCurveCache->m_sParentMgr.GetAt(lSpanCount-1);
    }

  // arrive here when pCurrArray points to the 1st generation of tree nodes

  // while there is more than 1 node in the current generation
  //   - build a parent node for every pair of current generation nodes.
  //   - Add the parent to pNextArray, the growing list of nodes in the next generation.
  //   - if(m_bComputeAuxTreeData)
  //        - compute parent node Aux data
  //        - if(m_bMakeFullTree)
  //              - clear child pointers when parent passes all tessellation tests
  ULONG lAuxIndex=0;
  while (pCurrArray->GetSize() > 1) 
    {
      pNextArray->ReSet();

      // for every node in the current generation - taken two at a time,
      //   Build a parent node and add it to pNextArray, the list of next generation nodes 
      for (ULONG j=0; j+1<pCurrArray->GetSize(); j+=2)  // note: can't say pCurrArray->GetSize()-1
        {
          // fetch next m_pTree->Node to be used as Parent of current target nodes
          SmTreeNode *pNode = (SmTreeNode*)pTree->m_sNodeMgr.GetAt(lNodeCount++);

          // init general Node member values
          pNode->m_pTree   = pTree;
          pNode->m_pParent = NULL;

          // set parent/child node family pointer values
          pNode->m_pChild1 = (SmTreeNode*)(*pCurrArray)[j];
          pNode->m_pChild2 = (SmTreeNode*)(*pCurrArray)[j+1];
          pNode->m_pChild1->m_pParent = pNode;
          pNode->m_pChild2->m_pParent = pNode;

          // Parent BoundingBox = Union(children bounding boxes)
          pNode->m_pChild1->m_sBBox.Union(pNode->m_pChild2->m_sBBox,pNode->m_sBBox);

          // init Node AuxData to none
          pNode->m_eAuxDataType = SM_AD_NONE;
          pNode->m_pData = NULL; 

          // add parent node to growing NextArray list of nodes
          pNextArray->Add(pNode);
                 
          // When asked, Build up parent node auxillary data
          //   note: when m_bMakeFullTree == FALSE - two children nodes can be simplified into one parent node
          if (pCurveCache->m_bComputeAuxTreeData) 
            {
              // Fetch and store next m_sParentMgr truncated SmBezierSpan data into parent Node
              pNode->m_eAuxDataType          = SM_AD_CURVE_DATA;
              SmBezierAux1d *pAuxSpanParent  = pCurveCache->GetParentAt(lAuxIndex++);
              pNode->m_pData                 = pAuxSpanParent;

              // fetch child SmBezierSpans
              SmBezierSpan *pBezLeft         = (SmBezierSpan*)pNode->m_pChild1->m_pData;
              SmBezierSpan *pBezRight        = (SmBezierSpan*)pNode->m_pChild2->m_pData;

              // compute Parent properties from Children 
              //   note: m_bMakeFullTree == FALSE, compute parent->m_dMaxChordHeightSquared, TRUE=don't
              sm_ComputeParentData(pCurveCache,        // in : Containing Curve Cache
                                   pAuxSpanParent,     // i/o: target Parent Span to update
                                   pBezLeft,           // in : Child 1 Span
                                   pBezRight,          // in : Child 2 Span
                                   rLeafBeziers,       // in : ordered list of leafNode Bezier curves
                                   m_bMakeFullTree);   // in : TRUE =
                                                      //      FALSE=

              // when(    m_bMakeFullTree == FALSE
              //      and parent passes all tessellation requirements)
              //   clear child pointers turning parent back into a leaf node
              if (!m_bMakeFullTree) 
                {
                  // parent locals
                  SmExtent1d sInterval;
                  sInterval = pAuxSpanParent->GetInterval() ;

                  // classify the parent against the tessellation parameter limits
                  SmBoolean bPassTests = (   (pAuxSpanParent->m_dMaxTurningAngleDeg > 90.0)
                                          || (m_dChordHeightTolerance != 0.0 && pAuxSpanParent->m_dMaxChordHeightSquared > dChordHeightToleranceSquared)
                                          || (m_dAngleToleranceDeg    != 0.0 && pAuxSpanParent->m_dMaxTurningAngleDeg    > dAngleToleranceInDegrees)
                                          || (m_dMax3DDistance        != 0.0 && pAuxSpanParent->m_dMaxLength             > m_dMax3DDistance)
                                          || (sInterval.GetLength() > dMaxAllowableStep))
                                         ? FALSE : TRUE ;
                  
                  // Limit subdivision by the minimum parametric ratio
                  if (m_dMinParamRatio != 0.0 && sInterval.GetLength() < m_sInterval.GetLength()*m_dMinParamRatio) 
                    {
                      bPassTests = TRUE;
                    }
                  
                  if (bPassTests) 
                    {
                      pNode->m_pChild1 = NULL;
                      pNode->m_pChild2 = NULL;
                    }
                } // end m_bMakeFullTree == FALSE test
            }

        } // end iter pCurrArray members two at a time

      // Now pick up any left overs and add it to the next generation
      if (pCurrArray->GetSize() % 2 == 1) 
        {
          pNextArray->Add( pCurrArray->GetAt(pCurrArray->GetSize()-1) );
        }

      // Now swap curr and next array references
      SmTArray<SmTreeNode*> * pTmpAr = pCurrArray;
      pCurrArray = pNextArray;
      pNextArray = pTmpAr;
    
    } // end while current generation of nodes has more than 1 node.

  // Now store root node
  pTree->m_pTopNode = (SmTreeNode*)(*pCurrArray)[0];

  // all done
  return SM_SUCCESS;

} // end SmCurveCache::BuildTree

/*******************************************************************//**
PURPOSE: Get the start of a Bezier span.

NOTES: 
***********************************************************************/
void SmBezierAux1d::GetStart
  (const SmCurveCache & crCurveCache,  // in : owner of this span
   SmPoint3d          & rStart,        // out: Image Space Start Point
   double             & dStart)        // out: Domain Space Span Interval start
  const 
{
  // set output
  dStart = m_sIvl.GetMin() ;
  crCurveCache.GetCurve()->EvaluatePoint(dStart, rStart) ;

} // end SmBezierAux1d::GetStart

/*******************************************************************//**
PURPOSE: Get the end point of a Bezier span.

NOTES: 
***********************************************************************/
void SmBezierAux1d::GetEnd
  (const SmCurveCache & crCurveCache,  // in : owner of this span
   SmPoint3d          & rEnd,          // out: Image Space End Point
   double             & dEnd)          // out: Domain Space Span Interval end
  const 
{
  // set output
  dEnd = m_sIvl.GetMax() ;
  crCurveCache.GetCurve()->EvaluatePoint(dEnd, rEnd) ;

} // end SmBezierAux1d::GetEnd

/*******************************************************************//**
PURPOSE: Get the end points of a Bezier span.

NOTES: 
***********************************************************************/
void SmBezierAux1d::GetEnds
  (const SmCurveCache & crCurveCache,  // in : owner of this span
   SmPoint3d          & rStart,        // out: Image Space Start Point
   SmPoint3d          & rEnd,          // out: Image Space End Point
   SmExtent1d         & rIvl)          // out: Domain Space Span Interval
  const 
{
  // set output

  rIvl = m_sIvl ;
  crCurveCache.GetCurve()->EvaluatePoint(rIvl.GetMin(), rStart) ;
  crCurveCache.GetCurve()->EvaluatePoint(rIvl.GetMax(), rEnd) ;

} // end SmBezierAux1d::GetEnds

/*******************************************************************//**
PURPOSE: Get Degree.

NOTES: 
***********************************************************************/
void SmBezierAux1d::GetDegree
  (const SmCurveCache & crCurveCache)  // in : owner of this span
  const
{
  crCurveCache.GetCurve()->GetDegree() ;
}

/*******************************************************************//**
PURPOSE: Debug formatted print 

NOTES: 
***********************************************************************/
void SmBezierAux1d::Dump
  (const SmCurveCache &crCurveCache,
   int lDepth)         // in : Recursive depth - used to add spacing to align
                       //      with SmTreeNode::Dump indented list diplay.
                       //      default:[0]
  const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sIndent[SM_TBLOCK_SIZE];
  TCHAR sContType[14][32]  = { _T("SM_NOT_USED"),
                               _T("SM_CT_DISCONTINUOUS"),
                               _T("SM_CT_C0"),           
                               _T("SM_CT_G1"),           
                               _T("SM_CT_G1R"),          
                               _T("SM_CT_G1_G2"),
                               _T("SM_CT_G1_G2_G3"),        
                               _T("SM_CT_C1"),           
                               _T("SM_CT_C1_G2"),        
                               _T("SM_CT_C1_G2_G3"),        
                               _T("SM_CT_C1_C2"),
                               _T("SM_CT_C1_C2_G3"),
                               _T("SM_CT_C1_C2_C3"),        
                               _T("SM_CT_CINFINITY") } ;    

  // set indent string equal to depth of recursive call
  sIndent[0] = sIndent[1] = sIndent[2] = sIndent[3] = ' ' ;
  int i, i2;
  for(i=0,i2=4;i<=lDepth;i++,i2+=2)
    {
      sIndent[i2  ] = ' ' ;
      sIndent[i2+1] = ' ' ;
    }
  sIndent[i2] = '\0' ;

  // locals
  SmExtent1d sIvl ;
  SmPoint3d sStart, sEnd ;
  GetEnds(crCurveCache, sStart, sEnd, sIvl) ;

  // output Span Summary
  smos_sprintf(sBuff,_T("\n%s         Interval[%16.16lf, %16.16lf], m_lLeft = [%lu], m_lRight = [%lu]"), 
             sIndent,
             sIvl.GetMin(),
             sIvl.GetMax(), 
             m_lLeft, 
             m_lRight) ;
  smos_WriteBuffer(sBuff) ;

  smos_sprintf(sBuff,_T("\n%s         BoundaryContinuities = [%s, %s]"), 
             sIndent, 
             sContType[m_aeConts[0]], 
             sContType[m_aeConts[1]]) ;
  smos_WriteBuffer(sBuff) ;

  
} // end SmBezierAux1d::Dump

/*******************************************************************//**
PURPOSE: Constructor for SmCurveCache.  It allows input of data used
    during tessellation of the cache.  Note that this constructor only
    inputs values.  The actual creation of cache data is done in the 
    Tessellate method.

NOTES: All arguments except the curve and interval are optional.
***********************************************************************/
SmCurveCache::SmCurveCache
  (const SmCurve    & crCurve,           // in : target curve
   const SmExtent1d & crInterval,        // in : target interval
   SmBoolean bMakeFullTree,              // in : only used when bComputeAuxTreeData == TRUE
                                         //      FALSE = Simplify Tree where Parent Node's pass all tessellation tests
                                         //      TRUE  = don't 
   double    dChordHeightTolerance,      // in : max leafNode control-polygon vertex to baseline distance, 0 = ignore
   double    dAngleTolDeg,               // in : max leafNode control-polygon vertex angle sum,            0 = ignore
   double    dOffAxisTolerance,          // in : max leafNode off axis control-polygon BBox size,          0 = ignore
                                         //        subdivides leaves into axis aligned near-linear segments.
   ULONG     lMinimumNumberOfSegments,   // in : limits max element size, 0 = ignore
   SmBoolean bComputeAuxTreeData,        // in : TRUE = propagate child properties up to parent nodes.
                                         //      FALSE= don't 
   double    dMax3DDistance,             // in : max leafNode 3d control-polygon baseline size,  0 = ignore
   double    dMinimumParametricRatio)    // in : min (leafNode Interval)/(Curve Interval) ratio, 0 = ignore
 : m_pTree        (NULL), 
   m_crCurve      (crCurve),
   m_sInterval    (crInterval),
   m_pContinuities(NULL),

   m_bTriedToCreateAnalytical(FALSE),
   m_pAnalyticalCurve        (NULL), 
    
   m_lBezierSize        (0),  
   m_lTotalBezSize      (0),
   m_bComputeAuxTreeData(bComputeAuxTreeData),
   m_bMakeFullTree      (bMakeFullTree), 

   m_dChordHeightTolerance(dChordHeightTolerance),
   m_dAngleToleranceDeg(dAngleTolDeg),
   m_dOffAxisTolerance(dOffAxisTolerance),

   m_dMax3DDistance(dMax3DDistance),
   m_dMinParamRatio(dMinimumParametricRatio),
   m_lMinSegNumber(lMinimumNumberOfSegments)
{
#ifdef SM_DEBUG_CACHE_CPP
static ULONG lCount = 0 ; lCount++ ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // count caches made and inform the debugging public
  ((SmCurve &)crCurve).m_lCurveCacheCount++ ;
  smos_sprintf(sBuff, _T("\nConstructed Curve Cache [Curve = 0x%p, rep = %ld] iter: %ld"), 
                    &crCurve, 
                    crCurve.m_lCurveCacheCount, 
                    lCount) ;
  MYPRINTF(sBuff) ;
#endif

  // get size of one bezier span
  m_lBezierSize =   crCurve.IsKindOf(SmBSplineCurve_TYPE)
                  ? sm_ComputeNurbCurveSize(crCurve.GetDegree(),
                                               crCurve.GetDegree()*2+1)
                  : sm_ComputeNurbCurveSize(3,
                                               3*2+1) ;

  // Note that we will add extra data to the Bezier to speed up processing.
  m_lTotalBezSize = ALIGN_SIZE(sizeof(SmBezierSpan)) ;

  // init internal arrays
  m_pContinuities = new (*crCurve.GetContext()) SmTArray<SmContinuityType>(*crCurve.GetContext());

} // end SmCurveCache::SmCurveCache constructor

/*******************************************************************//**
PURPOSE: Destructor for the curve cache object.

NOTES: 
***********************************************************************/
SmCurveCache::~SmCurveCache()
{
#ifdef SM_DEBUG_CACHE_CPP
static ULONG lCount = 0 ; lCount++ ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  smos_sprintf(sBuff, _T("\nDestructed Curve Cache [Curve = 0x%p] iter: %ld"), 
          &m_crCurve, lCount) ;
  MYPRINTF(sBuff) ;
#endif

  if (m_pTree)            { delete m_pTree;            m_pTree            = NULL ; }
  if (m_pContinuities)    { delete m_pContinuities;    m_pContinuities    = NULL ; }
  if (m_pAnalyticalCurve) { delete m_pAnalyticalCurve; m_pAnalyticalCurve = NULL ; }

} // end SmCurveCache::~SmCurveCache


/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the curve cache.

NOTES: currently does not count memory for the optional
  m_pAnalyticalCurve.
***********************************************************************/
ULONG SmCurveCache::GetMemoryUsed
 (ULONG &rlMemoryAllocated) 
 const
{ 
  ULONG lBezAlloc=0, lAuxAlloc=0, lContinuitiesAlloc=0, lTreeAlloc=0 ;
  ULONG lUsed =   sizeof(SmCurveCache) 
                + m_sBezMgr.GetMemoryUsed(lBezAlloc) 
                + m_sParentMgr.GetMemoryUsed(lAuxAlloc);

  // get continuities memory
  ULONG lContinuitiesUsed = m_pContinuities ? m_pContinuities->GetMemoryUsed(lContinuitiesAlloc) : 0 ;

  // get tree memory
  ULONG lTreeUsed         = m_pTree ? m_pTree->GetMemoryUsed(lTreeAlloc) : 0 ;

  // get Analytic Curve memory
  // gwc todo: something for m_pAnalyticCurve

  // set output
  rlMemoryAllocated =   sizeof(SmCurveCache)
                      + lBezAlloc
                      + lAuxAlloc
                      + lContinuitiesAlloc
                      + lTreeAlloc ;
  // all done
  return lUsed + lContinuitiesUsed + lTreeUsed ;

} // end SmCurveCache::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Compute control-polygon properties while optionally checking for tolerance violations.
  Tolerances are only checked when given usable tolerance values (0.0 always means don't check).
  Tolerance violations reports are hidden in the output values of this function.
     When a tol is violated processing stops and not all outputs are set.


NOTES:

  always reported:
    rdMaxTurningAngleDeg    = sum of all control-polygon vertex angles
    rdMaxChordHeightSquared = max control-polygon vertex to base-line distance
    rdMaxLength             = control-polygon baseline length

  only for tolerance violation reporting:
    rdOffAxisDistance       = 2nd largest control-polygon BBox size (an indication of a non-line shape
                                                                     or a line-not-aligned with the principle axes shape)  
    rbNullCurve             = TRUE when BBOx of control-polygon is smaller than SM_EFF_ZERO 


 Use of tolerances: When a tolerance is given - time is saved by returning
                    as soon as any part of the control-polygon is found that
                    exceeds that value, in which case only the output parameter
                    associated with the violated tolerance is set.

  when dCHTolSq        not 0.0,  set rdMaxChordHeightSquared on 1st failure and return
       dCrvAngleTolDeg not 0.0,  set rdMaxTurningAngleDeg    on 1st failure and return
       dOffAxisTol     not 0.0,  set rdOffAxisDistance       on 1st failure and return
       d3DDistanceTol  not 0.0,  set rdMaxLength             on 1st failure and return

       all vertices are within SM_EFF_ZERO dist of one another, rbNullCurve is set to TRUE and system returns
***********************************************************************/
static SmStatus sm_ComputeConstants
  (      SmBSplineCurve & rBez,                      // in : Target Curve
   const SmCurve        & rOriginalCurve,            // in : Curve that is being partitioned
   SmExtent1d             sInterval,                 // in : Interval of interest
   SmTArray<SmPoint3d>  & rPolygon,                  // out: rBez control polygon vertex locations
   double                 dCHTolSq,                  // in : when not 0.0, check control-polygon vertex to baseLine distances. 
   double                 dCrvAngleTolDeg,           // in : when not 0.0, check accumulated
                                                     //           control-polygon vertex angle value.
   double                 dOffAxisTol,               // in : when not 0.0, check that control-polygon BBOx
                                                     //           is a line to tolerance that is aligned with an x,y,or z axis
   double                 d3DDistanceTol,            // in : when not 0.0, check rBez endPoints are 
                                                     //           separated by more than d3DDistanceTol,
   SmApproxTol3d        & rApproxTol,                // in : Approximation tolerance between Bez and OriginalCurve
   double               & rdMaxLength,               // out: control-polygon baseline length
   double               & rdMaxChordHeightSquared,   // out: max control-polygon vertex to baseLine distance
   double               & rdMaxTurningAngleDeg,      // out: sum control-polygon vertex angles
   double               & rdOffAxisDistance,         // out: err value when dOffAxisTol not 0.0 and control-polygon BBox not linear and x,y, or z aligned
   double               & rdCurveDistance,           // out: err value when distance between curves is greater than rApproxTol
   SmBoolean            & rbNullCurve)               // out: err value when control-polygon vertices are all within SM_EFF_ZERO distance of one another
{
  // init output
  rdMaxLength             = 0.0;
  rdMaxChordHeightSquared = 0.0;
  rdMaxTurningAngleDeg    = 0.0; 
  rdOffAxisDistance       = 0.0;
  rdCurveDistance         = 0.0;
  rbNullCurve             = FALSE;

  // tolerance
  double dAngleTolRad = dCrvAngleTolDeg * SM_PI/180.0;

  // Get the Euclidian control polygon of a B-Spline curve
  SER(rBez.GetPolygon( rBez.GetNaturalInterval(), rPolygon ));

  if (rPolygon.GetSize() < 2) 
  {
	//FS
	rbNullCurve = TRUE;
	return SM_ERR;
  }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetColor(1,1,0); rBez.DrawPolygon(); sm_GraphicsLoop();
      rPolygon.Dump();
      sm_GraphicsLoop();
    }
#endif

  // locals
  SmVector3d sLastVec(0,0,0);
  double     dTotalDist      = 0.0;
  double     dTotalAngle     = 0.0;
  double     dMaxSegmentCHSq = 0.0;                            // largest control-polygon vertex to 1st/last line segment distance
  ULONG      lPolySize       = rPolygon.GetSize();
  SmPoint3d  sLinePnt        = rPolygon[0];
  SmPoint3d  sLastPoint      = rPolygon[lPolySize-1];
  SmVector3d sLineVec        = sLastPoint - sLinePnt;

  // set BaseLine length
  rdMaxLength = sLineVec.Length();

  // when asked - quit when interval is too big
  if(   d3DDistanceTol != 0.0 
     && rdMaxLength > d3DDistanceTol)  
    { return SM_SUCCESS; }

  // for closed curves - find a control-polygon vertex not coincident with the 1st vertex
  ULONG lTryIndex = lPolySize - 2;
  while (sLineVec.LengthSquared() < SM_EFF_ZERO*SM_EFF_ZERO) 
    {
      // If first and last point coincide try a middle point
      if (lTryIndex == 0) 
        {
          rbNullCurve = TRUE;
          return SM_SUCCESS;
        }
      SmPoint3d sTryPoint = rPolygon[lTryIndex--];  
      sLineVec = sTryPoint - sLinePnt;
    } // end while looking for non-coincident vertex


  // Check whether it's a convex polygon.
  // Do this trick only for cubics.
  SmBoolean bConvexPoly = TRUE;  // Special only if not convex.

// (Not ready for prime time, needs more testing.)
//  if ( lPolySize == 4 )
//    {
//      SmVector3d sVec1( rPolygon[1] - rPolygon[0] );
//      SmVector3d sVec2( rPolygon[2] - rPolygon[1] );
//      SmVector3d sVec3( rPolygon[3] - rPolygon[2] );
//      if ( ( sVec1*sVec2 ).Dot( sVec2*sVec3 ) <= 0.0 )
//        { bConvexPoly = FALSE; }
//    }

  SmVector3d sDiffVector(0,0,0);
  SmExtent3d sBBox;
  sBBox.AddPoint3d(rPolygon[0]);

  // for every control-polygon vertex - but the first
  //   build A. control-polygon BBox 
  //         B. Control-polygon length
  //   and check vertices for
  //     1. if(dOffAxisTol) non linear and principle axis aligned BBox
  //     2. if(dCHTolSq)    exceeded vertex to baseline distance
  //     3. if(dTotalAngle) exceeded control-polygon vertex angle sum
  for (ULONG i=1; i<lPolySize; i++) 
    {
      SmPoint3d sCurr = rPolygon[i-1];
      SmPoint3d sNext = rPolygon[i];

      // build control-polygon bounding box
      sBBox.AddPoint3d(sNext);

      // when given a dOffAxisTol - return when control-polygon BBOX becomes
      //   non linear and not aligned with a principle axis
      if (dOffAxisTol != 0.0) 
        {
          SmVector3d sDiff = sBBox.GetMax() - sBBox.GetMin();
          ULONG lCountOver = 0;
          if (sDiff.x > dOffAxisTol) lCountOver++;
          if (sDiff.y > dOffAxisTol) lCountOver++;
          if (sDiff.z > dOffAxisTol) lCountOver++;
          if (lCountOver > 1) 
            {
              if (sDiff.x >= sDiff.y && sDiff.x >= sDiff.z) 
                {
                  rdOffAxisDistance = smos_Max(sDiff.y,sDiff.z);
                }
              else if (sDiff.y >= sDiff.x && sDiff.y >= sDiff.z) 
                {
                  rdOffAxisDistance = smos_Max(sDiff.x,sDiff.z);
                }
              else 
                {
                  rdOffAxisDistance = smos_Max(sDiff.x,sDiff.y);
                }
              return SM_SUCCESS;
            }
        } // end dOffAxisTol existence check

      // get current vertex dist to 1st/last line segment
      double dLineParam;
      SER(smgu_LineClosestPoint(sLinePnt,sLineVec,sNext,dLineParam));
      SmPoint3d sPntOnLine = sLinePnt + (sLineVec * dLineParam);
      SmVector3d sCHVec = sNext - sPntOnLine;
      double dCHSq = sCHVec.LengthSquared();

      // when given dCHTolSq and vertex distance to endPoint line is exceeded - return
      if (dCHSq > dMaxSegmentCHSq) dMaxSegmentCHSq = dCHSq;
      if (dCHTolSq != 0.0 && dMaxSegmentCHSq > dCHTolSq) 
        {
          // Quit as soon as we know we don't qualify
          rdMaxChordHeightSquared = dMaxSegmentCHSq;
          return SM_SUCCESS;
        }

      // get Next/Current vector and accumulate control-polygon length
      SmVector3d sCurrVec = sNext - sCurr;
      double     dDist    = sCurrVec.Length();
      double     dScale   = (1.0 + sNext.GetMaxDimension());
      dTotalDist         += dDist;

      // when possible normalize Next/Current Vector
      if (dDist < SM_EFF_ZERO * dScale) 
        { continue; }
      sCurrVec            = sCurrVec / dDist;

      // accumulate control-polygon angles
      if (sLastVec.LengthSquared() > SM_EFF_ZERO_SQ) 
        {
          // use positive value from 0.0 to SM_PI.
          double dCosAngle  = sCurrVec.Dot( sLastVec );
          double dThisAngle = (dCosAngle >=  1.0 ) ? 0.0 :
                              (dCosAngle <= -1.0 ) ? SM_PI :
                                                     smos_ArcCosine(dCosAngle);
          if ( bConvexPoly )
          {
              dTotalAngle += dThisAngle;  // add in positive value.
          }
          else
          {
              dTotalAngle = smos_Max( dTotalAngle, dThisAngle );
          }
        }

      // when given dCrvAngleTolDeg - return when accumulated control-polygon vertex angle exceeds tolerance
      if (dTotalAngle != 0.0 && dTotalAngle > dAngleTolRad) 
        {
          // Quit as soon as we know we don't qualify
          rdMaxTurningAngleDeg = dTotalAngle * 180 / SM_PI;
          return SM_SUCCESS;
        }

      // set for next iteration
      sLastVec = sCurrVec;

    } // end iter every control-polygon vertex


  // set output
  rdMaxChordHeightSquared = dMaxSegmentCHSq;
  rdMaxTurningAngleDeg    = dTotalAngle * 180 / SM_PI;

  SmFindCurveDistanceExtremaEFO sEFO( &rBez, &rOriginalCurve, sInterval, 10.*SM_EFF_ZERO );
  SmLocalSolve1d sLS( sEFO, sInterval, FALSE );

  // Step along the curve: when the function changes sign,
  // call the local solver.
  ULONG lNumSteps = 20;
  double dFoundT, dParam, dFrac, dDenom( lNumSteps );
  double dF = 0, dF1 = 0;
  double dApproxTolSquared = rApproxTol * rApproxTol;
  SmPoint3d sPt1, sPt2;

  // Get function value at start of curve.
  double dPrevF = 0;
  double dPrevT = sInterval.GetMin();
  SmBoolean bFoundSolution;
  sEFO.Evaluate( dPrevT, dPrevF, dF1, bFoundSolution );

  for ( ULONG ii = 1; ii <= lNumSteps; ++ii )
    {
      dFrac  = (double)ii / dDenom;
      dParam = sInterval.Evaluate( dFrac );

      sEFO.Evaluate( dParam, dF, dF1, bFoundSolution );
      if ( bFoundSolution )
        {
          rBez.EvaluatePoint( dParam, sPt1 );
          rOriginalCurve.EvaluatePoint( dParam, sPt2 );
          double dDistanceSquared = sPt1.DistanceBetweenSquared( sPt2 );
          if ( dDistanceSquared > dApproxTolSquared )
          { 
              rdCurveDistance = smos_Sqrt(dDistanceSquared); 
              break;
          }
        }
      else if ( dF * dPrevF < 0. )
        {
          // ... could do a secant for the guess, but this should do fine:
          double dGuessParam = dPrevT - dPrevF * ( dParam - dPrevT ) / ( dF - dPrevF );
          if ( sLS.SolveIt( dGuessParam, 10.*SM_EFF_ZERO, bFoundSolution, dFoundT ) == SM_SUCCESS )
            {
              rBez.EvaluatePoint( dParam, sPt1 );
              rOriginalCurve.EvaluatePoint( dParam, sPt2 );
              double dDistanceSquared = sPt1.DistanceBetweenSquared( sPt2 );
              if ( dDistanceSquared > dApproxTolSquared )
              { 
                  rdCurveDistance = smos_Sqrt( dDistanceSquared );
                  break;
              }
            }
        } // end else

      dPrevT = dParam;
      dPrevF = dF;

    } // end stepping along curve

#ifdef SM_DEBUG_CODE
  if (FALSE) 
    {
      SmPoint3d sPV1[2], sPV2[2];
      SmExtent1d sIvl = rBez.GetNaturalInterval();
      SER(rBez.Evaluate(sIvl.GetMin(),1,TRUE,sPV1));
      SER(rBez.Evaluate(sIvl.GetMax(),1,TRUE,sPV2));
      double dAngle;
      SER(sPV1[1].AngleBetween(sPV2[1],dAngle));
      if (dAngle > dTotalAngle + SM_ZONE_TOL_3D/10.0e-6) 
        { 
          smgfx_SetColor(1,1,0);
          rBez.DrawPolygon();
          ERR(SM_ERR); 
        }
    }
#endif

  // all done
  return SM_SUCCESS;

} // end sm_ComputeConstants

#undef ARC_COS_TABLE_SIZE
#undef HALF_ARC_COS_TABLE_SIZE

/*******************************************************************//**
PURPOSE: Calculate one BezierSpan (represented as a gw_CURVE) from
   pCurve for every non-zero span indicated by NonZeroSpanCount.

NOTES: 
  Memory is already allocated, in rBeziers.
  The results are cubic.
  Each span is a separate cubic Bezier curve.
      
***********************************************************************/
static SmStatus sm_BuildBezierSpans(
        const SmCurve       * pCurve,  // in : curve to be approximated.
        SmTArray< double >  & rKnots,  // in : knots to use.
        SmTArray<gw_CURVE*> & rBeziers // out: already allocated; we'll fill in.
    )
{
    ULONG lNumKnots = rKnots.GetSize();

    if ( lNumKnots < 2 )
      { SER( SM_ERR ); }

    ULONG i, j;
    SmVector3d sPV[2];
    SmVector3d sCtrlPts[4];
    double dPrevParam;
    double dDeltaT;

    double dParam = rKnots[0];
    pCurve->Evaluate( dParam, 1, TRUE, sPV );  // first eval: FromLeft == TRUE.

    for ( i = 1; i < lNumKnots; i++ )
    {
        // Calculate control points.
        dPrevParam = dParam;
        dParam = rKnots[i];
        dDeltaT = dParam - dPrevParam;

        sCtrlPts[0] = sPV[0];
        sCtrlPts[1] = sCtrlPts[0] + sPV[1] * dDeltaT / 3.0;

        pCurve->Evaluate( dParam, 1, FALSE, sPV );
        sCtrlPts[3] = sPV[0];
        sCtrlPts[2] = sCtrlPts[3] - sPV[1] * dDeltaT / 3.0;

        // Install knots and control points into NLib structures.
        gw_CURVE * pBezSpan = rBeziers[i - 1];

        for ( j = 0; j <= 3; j++ ) { rBeziers[i - 1]->knt->U[j] = dPrevParam; }
        for ( j = 4; j <= 7; j++ ) { rBeziers[i - 1]->knt->U[j] = dParam; }

        for ( j = 0; j <= 3; j++ )
        {
            N_CPtSetX( sCtrlPts[j].x, &( pBezSpan->pol->Pw[j] ) );
            N_CPtSetY( sCtrlPts[j].y, &( pBezSpan->pol->Pw[j] ) );
            N_CPtSetZ( sCtrlPts[j].z, &( pBezSpan->pol->Pw[j] ) );
            N_CPtSetW( NL_NOW, &( pBezSpan->pol->Pw[j] ) );
        }
    } // end for on knot spans.

    return SM_SUCCESS;

} // end sm_BuildBezierSpans

/*******************************************************************//**
PURPOSE: Build one BezierSpan (represented as a gw_CURVE) for every
     non-zero span in pBSplCrv.  All gw_Curve memory is allocated as a
     single contiguous block within the rBezierMgr block.

NOTES: 
  1. Memory for one BezierSpan per curve span is allocated in one contiguous block
      as a set of contiguous gw_CURVE objects inside of rBezierMgr.
  2. All the NLib style internal gw_CURVE pointers are set so that
      the SMLib contiguous memory NURB can be used by the NLib functions.
  3. NLib is called to set all the knot and control point values to build
      a BezierSpan for each Curve Span
      
***********************************************************************/
static SmStatus sm_CreateBezierSegmentBlock
  (gw_CURVE            * pBSplCrv,   // in : target NLib BSpline curve
   const SmCurve       * pCurve,     // in : target SmCurve if pBSplCurve is not given.
   SmMemBlockMgr       & rBezierMgr, // out: allocated memory to hold the newly built gw_CURVE BezierSpans
   SmTArray<gw_CURVE*> & rBeziers    // out: ptr to a BezierSpan for each pBSplCrv non-zero span
  )
{
  // One or the other must be non-null.
  if ( pBSplCrv == NULL && pCurve == NULL )
    { NER( NULL ); }

  NL_DEGREE lDegree = 3; // default; but if pBSplCrv is given, use its degree.

  SmTArray< double > sKnots; // used only if pBSplCrv is Null.

  // get Curve's non-zero span count
  gw_INDEX NonZeroSpanCount;   // NonZeroSpanCount is number of non-zero ( NE knots)  spans

  if ( pBSplCrv != NULL )
  {
      lDegree = pBSplCrv->p;
      gw_KNOTVECTOR *knt = pBSplCrv->knt;
      N_BasisGetSpanCount( knt, lDegree, &NonZeroSpanCount );
  }
  else
  {
      // Note, if not linear, don't use 1 in case of closed periodic curve.

      // Have to use knot vector, because caller uses CalculateContinuities(),
      // which is assumed to match up with knot vector.
      pCurve->GetKnots( sKnots );

//cbi TODO: make deg 1 if linear ... must tell my new routines about it, they're cubic.

      NonZeroSpanCount = sKnots.GetSize() - 1;
  }

  // check state: curve must have spans
  if  (NonZeroSpanCount < 1 )
    { SER(SM_ERR); }
  
  // get gw_CURVE byte size to store a BezierSpan of the curve's degree
  ULONG lBezSize = sm_ComputeNurbCurveSize( lDegree, lDegree*2 + 1 );

  // tell mem manager to allocate spanCount+4 gw_Curves at a time
  rBezierMgr.Initialize( ALIGN_SIZE(lBezSize), NonZeroSpanCount+4) ;

  // allocate one gw_Curve pointer for each span
  rBeziers.SetSize( NonZeroSpanCount );

  // allocate the gw_Curve memory and set the internal NLib style pointer values
  for ( long i=0; i<NonZeroSpanCount; i++ )
    {
      // get ptr to memory allocated in rBezierMgr for next BezierSpan
      gw_CURVE *pSpanCurve = SM_REINTERPRET_CAST(gw_CURVE*,rBezierMgr.GetAt(i));

      // set the NLib style internal gw_CURVE pointer values 
      sm_InitNurbCurveMemory( pSpanCurve, lDegree, lDegree, lDegree*2+1 );

      // remember this location
      rBeziers[i] = pSpanCurve;
    } // end iter

  // set the gw_Curve controlPoint and Knot values to build a
  // Bezier Span for every span
  if ( pBSplCrv != NULL )
  {
      if(NL_YES == sm_CreateBezSegments( pBSplCrv, rBeziers ))
        { SER(SM_ERR); }
      // GW_SER( sm_CreateBezSegments( pBSplCrv, rBeziers ));
  }
  else
  {
      SER( sm_BuildBezierSpans( pCurve, sKnots, rBeziers ));
  }

  // all done
  return SM_SUCCESS;

} // end sm_CreateBezierSegmentBlock

/*******************************************************************//**
PURPOSE: Split a Bezier span at its midpoint, using the original curve.

NOTES: All Bezier segments are nonrational cubics.
***********************************************************************/
static SmStatus sm_SplitSegment(
              gw_CURVE   *pBez,      // in : to be split
        const SmCurve    *cpCurve,   // in : original curve, being approximated
              gw_CURVE   *pBezMin,   // out: 1st half of pBez
              gw_CURVE   *pBezMax    // out: 2nd half of pBez
    )
{
    // Cubic, being split in half.
    // The very far end points are the same;
    // the next points in of the half-spans are the midpoints of
    // the far end and the next point in.
    // The four interior points (last two of first half and
    // first two of second half) come from the evaluation of the original curve.
    // (Note, this assumes that the full span starts and ends right on
    // the original curve.)

    // Do the knots first.

    NL_REAL *pKnotsFull = pBez->knt->U;
    NL_REAL *pKnots1st  = pBezMin->knt->U;
    NL_REAL *pKnots2nd  = pBezMax->knt->U;

    SmExtent1d sSpanIvl( pKnotsFull[0], pKnotsFull[7] );
    double dMidParam = sSpanIvl.GetMid();
    int i;
    for ( i = 0; i < 4; i++ )
    {
        pKnots1st[i  ] = pKnotsFull[i];
        pKnots1st[i+4] = dMidParam;
        pKnots2nd[i  ] = dMidParam;
        pKnots2nd[i+4] = pKnotsFull[i+4];
    }

    // Now the control points.
    SmVector3d sOrigEvalMid[2];
    cpCurve->Evaluate( dMidParam, 1, FALSE, sOrigEvalMid );

    // Derivative multiplier: parameter span, divided by 3 for cubic.
    double dScale = ( dMidParam - pKnotsFull[0] ) / 3.0;

    NL_CPOINT *pCPointsFull = pBez->pol->Pw;
    NL_CPOINT *pCPointsHalf = pBezMin->pol->Pw;

    // First point of 1st half: same
    N_CPtSetX( pCPointsFull[0].x, &( pCPointsHalf[0] ));
    N_CPtSetY( pCPointsFull[0].y, &( pCPointsHalf[0] ));
    N_CPtSetZ( pCPointsFull[0].z, &( pCPointsHalf[0] ));
    N_CPtSetW( pCPointsFull[0].w, &( pCPointsHalf[0] ));

    // Second point of 1st half: midpoint.
    N_CPtSetX( ( pCPointsFull[0].x + pCPointsFull[1].x )/2, &( pCPointsHalf[1] ));
    N_CPtSetY( ( pCPointsFull[0].y + pCPointsFull[1].y )/2, &( pCPointsHalf[1] ));

    // Careful: do not add NL_NOZ or NL_NOW values together.
    pCPointsHalf[1].z =  (pCPointsFull[0].z == NL_NOZ) ? NL_NOZ :  (pCPointsFull[0].z + pCPointsFull[1].z )/2;
    pCPointsHalf[1].w =  (pCPointsFull[0].w == NL_NOW) ? NL_NOW :  (pCPointsFull[0].w + pCPointsFull[1].w )/2;

    // Third point of 1st half: Curve eval minus scaled tangent.
    N_CPtSetX( ( sOrigEvalMid[0].x - sOrigEvalMid[1].x * dScale ), &( pCPointsHalf[2] ));
    N_CPtSetY( ( sOrigEvalMid[0].y - sOrigEvalMid[1].y * dScale ), &( pCPointsHalf[2] ));
    pCPointsHalf[2].z =  ( sOrigEvalMid[0].z == NL_NOZ )
              ? NL_NOZ
              : sOrigEvalMid[0].z - sOrigEvalMid[1].z * dScale;
    N_CPtSetW(              NL_NOW                                 , &( pCPointsHalf[2] ));

    // Fourth point of 1st half: Curve eval.
    N_CPtSetX( ( sOrigEvalMid[0].x ), &( pCPointsHalf[3] ));
    N_CPtSetY( ( sOrigEvalMid[0].y ), &( pCPointsHalf[3] ));
    N_CPtSetZ( ( sOrigEvalMid[0].z ), &( pCPointsHalf[3] ));
    N_CPtSetW(       NL_NOW           , &( pCPointsHalf[3] ));

    // Move to 2nd half.
    pCPointsHalf = pBezMax->pol->Pw;

    // First point of 2nd half: Curve eval.
    N_CPtSetX( ( sOrigEvalMid[0].x ), &( pCPointsHalf[0] ));
    N_CPtSetY( ( sOrigEvalMid[0].y ), &( pCPointsHalf[0] ));
    N_CPtSetZ( ( sOrigEvalMid[0].z ), &( pCPointsHalf[0] ));
    N_CPtSetW(       NL_NOW           , &( pCPointsHalf[0] ));

    // Second point of 1st half: Curve eval plus scaled tangent.
    N_CPtSetX( ( sOrigEvalMid[0].x + sOrigEvalMid[1].x * dScale ), &( pCPointsHalf[1] ));
    N_CPtSetY( ( sOrigEvalMid[0].y + sOrigEvalMid[1].y * dScale ), &( pCPointsHalf[1] ));
    pCPointsHalf[1].z =  ( sOrigEvalMid[0].z == NL_NOZ )
              ? NL_NOZ
              : sOrigEvalMid[0].z + sOrigEvalMid[1].z * dScale;
    N_CPtSetW(              NL_NOW                                 , &( pCPointsHalf[1] ));

    // Third point of 2nd half: midpoint.
    N_CPtSetX( ( pCPointsFull[2].x + pCPointsFull[3].x )/2, &( pCPointsHalf[2] ));
    N_CPtSetY( ( pCPointsFull[2].y + pCPointsFull[3].y )/2, &( pCPointsHalf[2] ));
    pCPointsHalf[2].z =  (pCPointsFull[2].z == NL_NOZ) ? NL_NOZ :  (pCPointsFull[2].z + pCPointsFull[3].z )/2;
    pCPointsHalf[2].w =  (pCPointsFull[2].w == NL_NOW) ? NL_NOW :  (pCPointsFull[2].w + pCPointsFull[3].w )/2;

    // Fourth point of 2nd half: same.
    N_CPtSetX( pCPointsFull[3].x, &( pCPointsHalf[3] ));
    N_CPtSetY( pCPointsFull[3].y, &( pCPointsHalf[3] ));
    N_CPtSetZ( pCPointsFull[3].z, &( pCPointsHalf[3] ));
    N_CPtSetW( pCPointsFull[3].w, &( pCPointsHalf[3] ));

    return SM_SUCCESS;

} // end sm_SplitSegment

/*******************************************************************//**
PURPOSE: Tessellate a curve using any combination of a variety of 
    subDivision control parameters to build a binary decomposition tree
    of the curve.

NOTES: 
   1. Usually called immediately after constructing a SmCurveCache object.

   2. If one of the tolerances are zero then that tolerance is
      not used in the calculation.  At least one of the tolerances must be non-zero.
        m_dChordHeightTolerance;    // max leaf node control-polygon vertex to baseLine distance      - 0 = ignore     
        m_dAngleToleranceDeg;       // max leaf node control-polygon accumulated vertex angle value   - 0 = ignore    
        m_dOffAxisTolerance;        // max leaf node control-polygon bbox 2nd largest dimension       - 0 = ignore
                                    //     of nonlinear segments, or linear segments not aligned with an x, y or z axis

        m_dMax3DDistance;           // max LeafNode 3d size                                           - 0 = ignore     
        m_dMinParamRatio;           // (Leaf node interval)/(total interval) limit, stops subdivision - 0 = ignore
        m_lMinSegNumber;            // min number of leaf nodes - 0=ignore                            - 0 = ignore



ASSUMES --- The following are all set in the SmCurveCache::constructor.
  1. m_crCurve             is set and is a BSpline and has non-zero spans,             
  2. m_sInterval           is set to interval to be tessellated,                       
  3. m_lBezierSize         is set  (size in bytes of one gw_CURVE bezier span object), 
  4. m_lTotalBezSize       is set  (size in bytes of one SmBezierSpan object = one gw_CURVE cached curve property set), 
  5. m_bComputeAuxTreeData is set, TRUE = compute Parent node SmBSplineSpan data from children data, 
                                   FALSE= don't
  6. m_bMakeFullTree       is set, Only used when m_bComputeAuxTreeData == TRUE
                                   TRUE = do no truncation (don't compute parent->m_dMaxChordHeightSquared values)
                                   FALSE= Simplify Tree when possible.
                                          When parent nodes pass all tessellation tests, remove children and make
                                          parent a leaf node.
  7. all tessellation control parameters are set.

SIDE EFFECTS:
        1. inits m_sBezMgr size params
        2. if(m_bMakeFullTree) set m_pContinuities array to store continuity type of every Curve continuity boundary
        3. 
***********************************************************************/
SmStatus SmCurveCache::Tessellate()
{

#ifdef SM_DEBUG_CODE 
  SmBoolean bDebugMe = FALSE;
  static ULONG lDebugCount = 0;
  static ULONG lCount = 1; lCount++;
  if(bDebugMe || lDebugCount == lCount)
  {
    m_crCurve.Dump();
  }
#endif // SM_DEBUG_CODE

  // check state: need at least one stored tessellation parameter 
  if(SM_IS_ZERO( m_dChordHeightTolerance )
     && SM_IS_ZERO( m_dAngleToleranceDeg ))
  {
    SER( SM_ERR_INVALID_INPUT );
  }

  // To tessellate: work with the geometric properties of the control polygons
  // of a sequence of bezier subdivision spans, if they are available,
  // rather than the shape of the curve, to bound the chord height and angle.

  // local tessellation params
  double dChordHeightToleranceSquared = m_dChordHeightTolerance * m_dChordHeightTolerance;
  double dAngleToleranceInDegrees = m_dAngleToleranceDeg;

  // locals
  SmBSplineCurve    * pBSC = SM_CAST_NONNULL_PTR( SmBSplineCurve, &m_crCurve) ;   // Curve cast to BSpline
  SmPoint3d           saDataPnts[100];
  SmTArray<SmPoint3d> sPolygon( 100, saDataPnts );  // tessellation point array - being built
  SmMemBlockMgr       sBezStack( m_lBezierSize );  // BezierSpan processing stack
  SmPseudoBox         sPBox;
  SmApproxTol3d dApproxTol = SmTol::GetApproxTol3d( &m_crCurve );
  if ( dApproxTol < SM_EFF_ZERO )
  { dApproxTol = SmTol::GetApproxTol3d( m_cpContext ); }

  // force curve to have a NURB representation
  if(pBSC != NULL && pBSC->m_pNurb == NULL)
  {
    SER( pBSC->MakeNurb() );
  }

  // Temporary being built - 
  //   array of ordered bezier curve segments partitioning
  //   the curve that each pass the tessellation tests
  gw_CURVE           *saDataLeafBeziers[100];
  SmTArray<gw_CURVE*> sLeafBeziers( 100, &saDataLeafBeziers[0] );        // pointer array to final decomposition leaf nodes

  // When not simplifying the decomposition-tree
  if(m_bMakeFullTree)
  {
    SmContinuityType eMinContinuity;
    SER( m_crCurve.CalculateContinuities( eMinContinuity, *m_pContinuities ) );
  }

  // 1. build a gw_CURVE BezierSpan for each pBSC unique-knot span.
  //      - bezier memory in sSegmentBeziersMgr,
  //      - pointed to by sBeziers pointer list.
  SmMemBlockMgr       sSegmentBeziersMgr;                              // growable List of BezierCurve objects
  gw_CURVE           *saDataBeziers[100];
  SmTArray<gw_CURVE*> sBeziers( 100, &saDataBeziers[0] );                 // array of BezierSpan (gw_CURVE) pointers
  gw_CURVE *pNurb = (pBSC == NULL) ? NULL : pBSC->m_pNurb;


  // allocate memory and set BezierSpans
  SER( sm_CreateBezierSegmentBlock( pNurb,
       &m_crCurve,
       sSegmentBeziersMgr,
       sBeziers ) );

  // allocate one more bezier curve as local working memory
  gw_CURVE *pWorkBez = SM_REINTERPRET_CAST( gw_CURVE*, sSegmentBeziersMgr.GetNewElement() );

  // m_lBezierSize(degree) set in constructor and GetElementSize(degree) set in sm_CreateBezierSegmentBlock
  SM_ASSERT( m_lBezierSize == sSegmentBeziersMgr.GetElementSize() );

  // max step size
  double dMaxAllowableStep = (m_lMinSegNumber > 1)
    ? (m_sInterval.GetMax() - m_sInterval.GetMin()) / m_lMinSegNumber
    : (m_sInterval.GetMax() - m_sInterval.GetMin());

  // tell SmBezierSpan manager to alloc 25 objects at a time
  m_sBezMgr.Initialize( ALIGN_SIZE( m_lTotalBezSize ), 25) ;

  // 2. for every unique curve parameter span
  //     2a. Trim to tessellation interval
  //     
  //     2b. Test for compliance with tessellation control parameters
  //     2c. If passed - Output a segment
  //         If failed - split and place children in the tessellation stack
  //     2d. While tessellation stack has members
  //         2d1. test top-of-stack for tesselation compliance
  //         2d2. if passed - Output a segment
  //              else split and place children in the tessellation stack
  ULONG i;
  for(i = 0; i < sBeziers.GetSize(); i++)
  {
    // get BezierSpan and its interval built for this Curve span
    gw_CURVE   * pBez = sBeziers[i];                // pBez points to sSegmentBeziersMgr working memory
    SmExtent1d   sSpanIvl( pBez->knt->U[0],
                          pBez->knt->U[pBez->knt->m] );

    // skip intervals outside the tessellation range
    if(m_sInterval.AreDisjoint( sSpanIvl ))
    {
      continue;
    }

    // build new bezier span trimmed to the tessellation interval when needed
    if(!sSpanIvl.IsContainedBy( m_sInterval, SM_EFF_ZERO ))
    {
      // trim curve interval to tessellation interval
      SER( m_sInterval.Intersect( sSpanIvl, sSpanIvl ) );

      // skip intervals too small to process
      double dScaledTol = SM_EFF_ZERO * (1.0 + smos_Fabs( sSpanIvl.GetMax() ) + smos_Fabs( sSpanIvl.GetMin() ));
      if(sSpanIvl.GetMax() - sSpanIvl.GetMin() < dScaledTol)
      {
        continue;
      }

      // build a new SmBezierSpan for the trimmed interval in the next
      //   available spot within the sSegmentBeziersMgr's gw_Curve block.
      //   Note: sSegmentBeziersMgr currently stores one bezier per original curve span.
      //         Now we are adding another bezier curve to the end of that list
      //         to represent the trimmed span.
      gw_CURVE *pTrimmed = SM_REINTERPRET_CAST( gw_CURVE*, sSegmentBeziersMgr.GetNewElement() );
      sm_InitNurbCurveMemory( pTrimmed, pBez->p, pBez->p, pBez->p * 2 + 1 );
      GW_SER( sm_CrvTrim( pBez,
                          sSpanIvl.GetMin(),
                          sSpanIvl.GetMax(),
                          pTrimmed,
                          pTrimmed ) );

      // change working bez ptr to this trimmed bezier span
      pBez = pTrimmed;                 // pBez points to sSegmentBeziersMgr working memory

    } // end need to trim interval check

  // tessellation locals
    double    dMaxLength, dMaxChordHeightSquared, dMaxTurningAngleDeg;
    SmBoolean bNullCurve;
    SmPoint3d sLastPoint;
    double    dOffAxisDistance, dCurveDistance;

    // convert current gw_CURVE Bezier curve into an SmBSplineCurve
    SmBSplineCurve sBSPTmp( pBez, m_crCurve.GetDim(), TRUE );

    // get control-polygon vertex chord height and accumulated vertex angle.
    //   while checking all non-zero tolerance properties
    SER( sm_ComputeConstants( sBSPTmp,                      // in : target SmBSplineCurve     
          m_crCurve,
                              sSpanIvl,
         sPolygon,                     // out: sBSPTmp control-polygon vertex locations
         dChordHeightToleranceSquared, // in : 
         m_dAngleToleranceDeg,         // in : 
         m_dOffAxisTolerance,          // in : 
         m_dMax3DDistance,             // in : 
                               dApproxTol,
         dMaxLength,                   // out: control-polygon baseLine length, only set when m_dMax3DDistance != 0.0
         dMaxChordHeightSquared,       // out: max control-polygon vertex to baseLine distance
         dMaxTurningAngleDeg,          // out: Sum control-polygon vertex angles
         dOffAxisDistance,             // out: Not 0.0 = m_dOffAxisTolerance exceeded
                              dCurveDistance,
         bNullCurve ) );                 // out: TRUE = span 3d-size less than SM_EFF_ZER0

// skip short spans - unless its the last span and no other spans have yet to be output
// gwc: Is this a source of tolerance errors?
//      A Snapping concept should be used here so that the sequence
//      of spans is continuous across the original curve's domain.
    if(bNullCurve
       && (GetSpanCount() != 0 || i < sBeziers.GetSize() - 1))
    {
      continue;
    }

    // analyze sm_ComputeConstants() output for span failures
    //   bPassTests: TRUE  = make this a leaf node in the decomposition tree
    //               FALSE = split this node and continue
    SmBoolean bPassTests1 = ((dMaxTurningAngleDeg > 90.0)
                            || (sSpanIvl.GetLength() > dMaxAllowableStep)
                            || (m_dChordHeightTolerance != 0.0 && dMaxChordHeightSquared > dChordHeightToleranceSquared)
                            || (m_dAngleToleranceDeg != 0.0 && dMaxTurningAngleDeg > dAngleToleranceInDegrees)
                            || (m_dOffAxisTolerance != 0.0 && dOffAxisDistance > m_dOffAxisTolerance)
                            || (m_dMax3DDistance != 0.0 && dMaxLength > m_dMax3DDistance)
                            || (dCurveDistance > dApproxTol))
      ? FALSE
      : TRUE;

    // don't split - small bezier spans when asked,
    //             - NULLCurves included to have at least one span per curve
    if((m_dMinParamRatio != 0.0
        && sSpanIvl.GetLength() / 2.0 < m_sInterval.GetLength() * m_dMinParamRatio)
       || (bNullCurve))
    {
      bPassTests1 = TRUE;
    }


    // when this BezierSpan is choosen to be a leaf node in the decomposition tree
    if(bPassTests1)
    {
      // get this span's neighbor continuities
      SmContinuityType eLeftCont = (m_bMakeFullTree) ? (*m_pContinuities)[i] : SM_CT_C1_C2;
      SmContinuityType eRightCont = (m_bMakeFullTree) ? (*m_pContinuities)[i + 1] : SM_CT_C1_C2;

      // fetch memory for the next SmBezierSpan from the m_sBezMgr, and
      //  1. set all SmBezierSpan data.
      //  2. Place SmBezierSpan into ordered LeafArray
      sLeafBeziers.Add( OutputSegment( pBez,                 // pBez points to sSegmentBeziersMgr working memory
                        NULL,
                        dMaxLength,
                        dMaxChordHeightSquared,
                        dMaxTurningAngleDeg,
                        eLeftCont,
                        eRightCont ) );
      continue;
    } // end found a leaf-node check

  // arrive here when bezier span needs to be subdivided

  // Begin Iterative splitting and refining of intervals 

  // Split pBez into two equal halves -
  //   Build the children into the 1st two gw_CURVE spots in the sBezStack scratch memory 
    ULONG lTopOfStack = 0;
    gw_CURVE *pBezMax = SM_REINTERPRET_CAST( gw_CURVE*, sBezStack.GetAt( lTopOfStack ) ); lTopOfStack++;
    gw_CURVE *pBezMin = SM_REINTERPRET_CAST( gw_CURVE*, sBezStack.GetAt( lTopOfStack ) ); lTopOfStack++;
    sm_InitNurbCurveMemory( pBezMin, pBez->p, pBez->p, pBez->p * 2 + 1 );
    sm_InitNurbCurveMemory( pBezMax, pBez->p, pBez->p, pBez->p * 2 + 1 );

    if(pBSC != NULL)
    {
      // The Bezier segment is an exact representation, so we can work from it.
      GW_SER( sm_CrvSplit( pBez,
              sSpanIvl.GetMid(), // split parameter value
              pBezMin,
              pBezMax,
              pBezMin,
              pBezMax ) );
    }
    else
    {
      // Split using the original curve.
      SER( sm_SplitSegment( pBez, &m_crCurve, pBezMin, pBezMax ) );
    }

    // Keep going until entire sBezStack is used up.
    //  Stack items are bezier gw_CURVE segments.
    //  In each iteration the top stack item is popped and checked to see if it passes all tessellation requirements.
    //  When it passes - it is used to output a SmBezierSpan
    //  When it fails  - it is split in half and its children gw_CURVEs are placed on the stack
    SmBoolean bFirstInSeg = TRUE;
    while(lTopOfStack != 0)
    {
      // Pop the top stack item off the stack
      pBez = SM_REINTERPRET_CAST( gw_CURVE*, sBezStack.GetAt( lTopOfStack - 1 ) ); // pBez points to sBezStack scratch memory
      SmBSplineCurve sBSP( pBez, m_crCurve.GetDim(), TRUE );
      lTopOfStack--;

      SmExtent1d sInterval = sBSP.GetNaturalInterval();

      // get current bezierSpan's control-polygon parameters
    SER( sm_ComputeConstants( sBSP,                      // in : target SmBSplineCurve     
          m_crCurve,
                              sInterval,
         sPolygon,                     // out: sBSPTmp control-polygon vertex locations
         dChordHeightToleranceSquared, // in : 
         m_dAngleToleranceDeg,         // in : 
         m_dOffAxisTolerance,          // in : 
         m_dMax3DDistance,             // in : 
                               dApproxTol,
         dMaxLength,                   // out: control-polygon baseLine length, only set when m_dMax3DDistance != 0.0
         dMaxChordHeightSquared,       // out: max control-polygon vertex to baseLine distance
         dMaxTurningAngleDeg,          // out: Sum control-polygon vertex angles
         dOffAxisDistance,             // out: Not 0.0 = m_dOffAxisTolerance exceeded
                              dCurveDistance,
         bNullCurve ) );                 // out: TRUE = span 3d-size less than SM_EFF_ZER0

      // skip very small curves
      if(bNullCurve)
        continue;

      // analyze sm_ComputeConstants() output for span failures
      //   bPassTests: TRUE  = make this a leaf node in the decomposition tree
      //               FALSE = split this node and continue
      SmBoolean bPassTests2 = ((dMaxTurningAngleDeg > 90.0)
                              || (sInterval.GetLength() > dMaxAllowableStep)
                              || (m_dChordHeightTolerance != 0.0 && dMaxChordHeightSquared > dChordHeightToleranceSquared)
                              || (m_dAngleToleranceDeg != 0.0 && dMaxTurningAngleDeg > dAngleToleranceInDegrees)
                              || (m_dOffAxisTolerance != 0.0 && dOffAxisDistance > m_dOffAxisTolerance)
                              || (m_dMax3DDistance != 0.0 && dMaxLength > m_dMax3DDistance)
                              || (dCurveDistance > dApproxTol))
        ? FALSE
        : TRUE;

      // don't split bezier spans below minimum size when asked
      if(m_dMinParamRatio != 0.0
         && sInterval.GetLength() / 2.0 < this->m_sInterval.GetLength() * m_dMinParamRatio)
      {
        bPassTests2 = TRUE;
      }

      // don't split bezier spans below zero size
      if(sInterval.GetLength() < SM_EFF_ZERO_SQRT)
      {

#ifdef SM_DEBUG_CODE
        //                gwc:note - This case should only occur for near zero length curves or
        //                           if one of the bPassTests is always failing forcing a regular
        //                           size curve to be tessellated with near zero length pieces.
        //                           To debug you should check which of the bPassTests checks is failing
        //                             and figure out how this case is making that happen.
        //                ERR_MSG(_T("Very Small Step Size in Tessellation"));
        if(bDebugMe)
        {
          m_crCurve.Dump();
          sBSP.Dump();
          sm_GraphicsLoop();
          smgfx_SetColor( 1, 0, 0 );
          sBSP.DrawWDeriv( sInterval, 0 );
          smgfx_SetColor( 0, 0, 1 );
          m_crCurve.DrawWDeriv( m_crCurve.GetNaturalInterval(), 0 );
          sm_GraphicsLoop();
        }
#endif

        bPassTests2 = TRUE;
      }

      // when this BezierSpan is choosen to be a leaf node in the decomposition tree
      if(bPassTests2)
      {
        SmContinuityType eLeftCont = SM_CT_UNDEFINED;
        if(m_bMakeFullTree && bFirstInSeg)
        {
          bFirstInSeg = FALSE;
          eLeftCont = (*m_pContinuities)[i];
        }
        else
        {
          eLeftCont = SM_CT_C1_C2;
        }

        SmContinuityType eRightCont = (m_bMakeFullTree && lTopOfStack == 0) ? (*m_pContinuities)[i + 1] : SM_CT_C1_C2;

        // fetch memory for the next SmBezierSpan from the m_sBezMgr, and
        //  1. set all SmBezierSpan data.
        //  2. Place3 SmBezierSpan into ordered LeafArray
        sLeafBeziers.Add( OutputSegment( pBez,                    // pBez points to sBezStack scratch memory
                          &sSegmentBeziersMgr,     // copy gw_CURVE to this bit of working memory
                          dMaxLength,
                          dMaxChordHeightSquared,
                          dMaxTurningAngleDeg,
                          eLeftCont,
                          eRightCont ) );
        continue;
      } // end found a leaf-node check

    // arrive here when current BezierSpan needs to be subdivided again to pass tessellation tests

    // Split the current BezierSpan in half placing the children in the sBezStack for further processing
      sm_CopyNurbCurve( pBez, pWorkBez );   // copy is required.  pBez points to sBezStack scratch memory.
                                            //   pWorkBez points to sSegmentBeziersMgr work memory.
                                            //   The upcoming split builds the children Beziers into the
                                            //   sBezStack scratch memory over the current pBez location. 
                                            //   This copy moves the parent values to temp memory 
                                            //   where it lives long enough to finish building the children.

      pBezMax = SM_REINTERPRET_CAST( gw_CURVE*, sBezStack.GetAt( lTopOfStack ) ); lTopOfStack++;
      pBezMin = SM_REINTERPRET_CAST( gw_CURVE*, sBezStack.GetAt( lTopOfStack ) ); lTopOfStack++;
      sm_InitNurbCurveMemory( pBezMin, pWorkBez->p, pWorkBez->p, pWorkBez->p * 2 + 1 );
      sm_InitNurbCurveMemory( pBezMax, pWorkBez->p, pWorkBez->p, pWorkBez->p * 2 + 1 );

      if(pBSC != NULL)
      {
        // The Bezier segment is an exact representation, so we can work from it.
        GW_SER( sm_CrvSplit( pWorkBez,
                sInterval.GetMid(),  // split parameter value
                pBezMin, pBezMax,
                pBezMin,
                pBezMax ) );
      }
      else
      {
        // Split using the original curve.
        SER( sm_SplitSegment( pWorkBez, &m_crCurve, pBezMin, pBezMax ) );
      }

    } // end iterative subdivision of span until all segments pass the tessellation tests

  } // end iter every m_crCurve non-zero span 

// arrive here when curve has been sub-divided into a set of leaf-nodes
// that each pass the tessellation tests.  Each curve span is associated
// with an SmBezierSpan object pointed to in sequence by the sLeafBeziers pointer array.
// The SmBezierSpan objects are stored in an unordered fashion within sSegmentBeziersMgr.

// Now combine the ordered SmBezierSpans pointed to in sLeafBeziers into a binary tree
  SER( BuildTree( sLeafBeziers ) );

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
  {
    m_pTree->Dump();
  }
#endif

  // all done
  return SM_SUCCESS;

} // end SmCurveCache::Tessellate

/*******************************************************************//**
PURPOSE: Get tessellation boundary points in an ordered array

NOTES: output either or both boundary point params or 3d locations.
***********************************************************************/

SmStatus SmCurveCache::GetTessellation
  (SmTArray<double>    *pParameters,    // out: optional array of tessellation span boundary parameter values, NULL to ignore
   SmTArray<SmPoint3d> *pPoints)        // out: optional array of tessellation span boundary 3d Point locations, NULL to ignore
{
  // check input
  if(   pParameters == NULL 
     && pPoints     == NULL) 
    { SER(SM_ERR); }

  // init output
  // locals
  ULONG ii, lSpanCount = m_sBezMgr.GetNumActiveElements();
  SmPoint3d sStart, sEnd ;
  double    dStart, dEnd ;
  SM_ASSERT(lSpanCount > 0) ;

  // get start of 1st span
  SmBezierSpan *pBezierSpan = GetAt(0) ;
  pBezierSpan->GetStart(*this, sStart, dStart) ;

  // load the output arrays
  if(pParameters && pPoints) 
    { // init output
      pParameters->SetSize(lSpanCount+1); 
      pParameters->ReSet();
      pPoints->SetSize(lSpanCount+1);     
      pPoints->ReSet(); 

      // 1st span start
      pPoints->Add(sStart) ;  
      pParameters->Add(dStart) ;
     
      // all span ends  
      for(ii=0;ii<lSpanCount;ii++)
        {
          pBezierSpan->GetEnd(*this, sEnd, dEnd) ;
          pPoints->Add(sEnd) ;  
          pParameters->Add(dEnd) ; 
        } // end iter every span
     } // end both pParaemters and pPoints

  else if(pParameters)       
    { // init output
     pParameters->SetSize(lSpanCount+1); 
     pParameters->ReSet();
    
     // 1st span start
     pParameters->Add(dStart) ;
     
     // all span ends  
     for(ii=0;ii<lSpanCount;ii++)
       {
         pBezierSpan->GetEnd(*this, sEnd, dEnd) ;
         pParameters->Add(dEnd) ; 
       } // end iter every span
    } // end just pParaemters

  else                       
    { // init output
     pPoints->SetSize(lSpanCount+1);     
     pPoints->ReSet(); 

     // 1st span start
     pPoints->Add(sStart) ;  
     
     // all span ends  
     for(ii=0;ii<lSpanCount;ii++)
       {
         pBezierSpan->GetEnd(*this, sEnd, dEnd) ;
         pPoints->Add(sEnd) ;  
       } // end iter every span
    } // end just pPoints 

//        SmBoolean bFirst = TRUE ;
//      
//        // local stack data
//        SmTreeNode *pNData[64];
//        SmTArray<SmTreeNode*> sStack(64,pNData);
//        SmTreeNode *pNode = m_pTree->GetTopNode();
//        SmPoint3d sStart, sEnd;
//        double    dStart, dEnd ; 
//      
//        // init stack with top node
//        sStack.Add(pNode);
//        
//        // iter until treeNode stack is empty
//        while (sStack.GetSize() > 0) 
//          {
//            pNode = sStack.GetLast();
//            sStack.RemoveLast();
//      
//            // if the node is a parent node
//            if (pNode->m_pChild1) 
//              {
//                // place the children on the stack
//                if (pNode->m_pChild2) { sStack.Add(pNode->m_pChild2); }
//                sStack.Add(pNode->m_pChild1);
//              }
//            else // node is a leaf node
//              {
//                // get Node's geometry summary
//                SmBezierSpan * pBezSpan = (SmBezierSpan*)pNode->m_pData; NER(pBezSpan);
//      
//                if(bFirst) { pBezSpan->GetStart(*this, sStart, dStart) ;
//                             if(pPoints)     { pPoints->Add(sStart) ; }
//                             if(pParameters) { pParameters->Add(dStart) ; }
//                             bFirst = FALSE ; 
//                           }
//      
//                pBezSpan->GetEnd(*this, sEnd, dEnd) ;
//                if(pPoints)     { pPoints->Add(sEnd) ; }
//                if(pParameters) { pParameters->Add(dEnd) ; }
//      
//              } // end leaf node branch
//          } // end iter treeNode stack until empty

  // all done
  return SM_SUCCESS;

} // end SmStatus SmCurveCache::GetTessellation


/*******************************************************************//**
PURPOSE:  load the next leaf-node's SmBezierSpan data from
             a source gw_CURVE bezier curve.

NOTES: 
  1. Fetch next SmBezierSpan from the m_sBezMgr
  2. When pSegmentBeziersMgr is not NULL,
         - Copy bezier curve into controlled memory
  3. set all other SmBezierSpan member values 
  
***********************************************************************/
gw_CURVE *SmCurveCache::OutputSegment
  (gw_CURVE        *pBezCurve,                // in : bezier curve to output as a segment - in working or scratch memory
   SmMemBlockMgr   *pSegmentBeziersMgr,       // in : working memory, needed when pBezCurve is in scratch memory
   double           dMaxLength,               // in : baseline length of curve's control-polygon
   double           dMaxChordHeightSquared,   // in : max vertex to baseline length of curve's control-polygon
   double           dMaxTurningAngleDeg,      // in : sum of vertex angles for curve's control-polygon
   SmContinuityType eLeftContinuity,          // in : continuity to left neighbor span
   SmContinuityType eRightContinuity)         // in : continuity to right neighbor shan
{
  ULONG         lNumSpan         = GetSpanCount();
  SmBezierSpan* pNew             = (SmBezierSpan*)m_sBezMgr.GetNewElement();

  pNew->m_lLeft                  = lNumSpan;     // a leaf node is its own left and right most bezier component
  pNew->m_lRight                 = lNumSpan;     
  pNew->m_aeConts[0]             = eLeftContinuity;
  pNew->m_aeConts[1]             = eRightContinuity;
  pNew->m_dMaxLength             = dMaxLength;
  pNew->m_dMaxChordHeightSquared = dMaxChordHeightSquared;
  pNew->m_dMaxTurningAngleDeg    = dMaxTurningAngleDeg;

  // note: m_sPseudoBox is not computed here. It gets set later in  
  //       SmCurveCache::BuildTree() when this SmBezierSpan is associated
  //       with an SmTreeNode and there is a place to store the regular
  //       bounding box as well.  Its efficient to compute the regular
  //       and the pseudo boxes at the same time.

  pNew->m_sIvl.SetMinMax(pBezCurve->knt->U[0], pBezCurve->knt->U[pBezCurve->knt->m]) ;

  // when asked copy the pBezCurve to working memory
  if(pSegmentBeziersMgr)
    {
      gw_CURVE *pWorkBez = SM_REINTERPRET_CAST(gw_CURVE*,pSegmentBeziersMgr->GetNewElement());
      sm_CopyNurbCurve(pBezCurve, pWorkBez);
      return(pWorkBez) ;
    }
  else
    {
      return( pBezCurve ) ;
    }  

} // end SmCurveCache::OutputSegment

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCurveCache::IsKindOf( SM_TYPE t ) const
{
  return ((SmCurveCache_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmCurveCache::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,       _T("SmCurveCache = 0x%p, "),this);
  smos_sprintf(sBuffForFile,_T("SmCurveCache = %s, "), _T("notNULL") );
  smos_WriteBuffer(sBuff, sBuffForFile);
  smos_WriteBuffer(_T("\n"));

  // for every span
  for (ULONG i=0; i<GetSpanCount(); i++) 
    {
      SmBezierSpan *pBezElem = (SmBezierSpan*)((SmCurveCache*)this)->m_sBezMgr.GetAt(i);

      smos_sprintf(sBuff,_T("   Span[%ld], CH=%16.16lf, Ang=%16.16lf, Dist=%16.16lf  Ivl=  "),
                          i,
                          smos_Sqrt(pBezElem->m_dMaxChordHeightSquared),
                          pBezElem->m_dMaxTurningAngleDeg,
                          pBezElem->m_dMaxLength);
      smos_WriteBuffer(sBuff);

      SmPoint3d sStart, sEnd ;
      SmExtent1d sIvl ;
      pBezElem->GetEnds(*this, sStart, sEnd, sIvl) ;

      sIvl.Dump();
      smos_WriteBuffer(_T("\n       Start = ")); 
      sStart.Dump();
      smos_WriteBuffer(_T("       End   = ")); 
      sEnd.Dump();
      smos_WriteBuffer(_T("\n"));
    }

} // end SmCurveCache::Dump

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmCurveCache::Draw(void) const
{
#ifdef SM_GFX_CODE
  // for every span
  for (ULONG i=0; i<GetSpanCount(); i++) 
    {
      SmBezierSpan         * pBezElem = (SmBezierSpan*)((SmCurveCache*)this)->m_sBezMgr.GetAt(i);

      const SmBSplineCurve * pCurve ;
      SmExtent1d             sIvl ; 

      pCurve   = (const SmBSplineCurve *)GetCurve() ;
      sIvl     = pBezElem->GetInterval() ;
      
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        { pCurve->Dump() ;
          pBezElem->GetPseudoBox().Dump() ;
          smgfx_Erase() ; 
        }
#endif // SM_DEBUG_CODE

      smgfx_SetColor(1,0,0); pBezElem->GetPseudoBox().Draw(GetContext());
      smgfx_SetColor(0,1,0); pCurve->Draw(&sIvl);

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { sm_GraphicsLoop() ; }
#endif // SM_DEBUG_CODE

    } // end iter every span

#endif // SM_GFX_CODE

} // end SmCurveCache::Draw


