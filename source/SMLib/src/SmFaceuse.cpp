// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmFaceuse.cpp
* PURPOSE: Source file for SmFaceuse class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmFaceuse.h>

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

#ifndef __SMEDG_H__
#include <SmEdge.h>

#endif
#ifndef __SMEDGEUSE_H__
#include <SmEdgeuse.h>
#endif

#include <SmGraphicsOutput.h>
#include <SmAssertArray.h>

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    SmFaceuse * dbgFaceuse1 = NULL ;
//    SmFaceuse * dbgFaceuse2 = NULL ;
//    
//    #endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Default SmFaceuse Constructor 

NOTES: 
***********************************************************************/
SmFaceuse::SmFaceuse()
 : m_pF(NULL),                  
   m_pFUMate(NULL)              
{
  m_eOrientation = SM_OT_UNKNOWN ;

  // report construction at SmObject::Notify level - skip other levels
  SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL);

} // end SmFaceuse::SmFaceuse Default Constructor

/*******************************************************************//**
PURPOSE: Get the loopuses of a faceuse.

NOTES: 
***********************************************************************/
void SmFaceuse::GetLoopuses
 (SmTArray<SmLoopuse*> & rLoopuses,      // out: Faceuse Loopuses
  SmBoolean                 bTgtLoopsOnly, // in : TRUE           = Only get Loopuses from pTgtLoops loops
                                           //    : default:[FALSE]= get Loopuses from all Face->Loops
  const SmTArray<SmLoop*> * pOptTgtLoops   // in : When bTgtLoopsOnly == TRUE                                
                                           //    : default:[NULL or Size=0] = Only use Face's OuterLoop
                                           //    : Size>0                   = Only use these Loops
 )const 
{ 
  if(bTgtLoopsOnly)   
{ 
      if(pOptTgtLoops == NULL || pOptTgtLoops->GetSize() == 0)
        { // Only use OuterLoop branch
          rLoopuses.ReSet() ;
                       if(m_pList) { rLoopuses.Add((SmLoopuse *)m_pList) ; }
                     }
      else // Use given Loops branch
        { 
          ULONG ii ;
          rLoopuses.SetSize(m_lListSize) ;
          rLoopuses.ReSet() ;

          // for nonNULL m_pList pointers
          for(ii=0;ii<pOptTgtLoops->GetSize();ii++)
            {
              if(pOptTgtLoops->GetAt(ii)->GetLoopuse()->GetFaceuse() == this) 
                { rLoopuses.Add(pOptTgtLoops->GetAt(ii)->GetLoopuse()) ; }
              else if(pOptTgtLoops->GetAt(ii)->GetLoopuse()->GetOtherLoopuse()->GetFaceuse() == this) 
                { rLoopuses.Add(pOptTgtLoops->GetAt(ii)->GetLoopuse()->GetOtherLoopuse()) ; }
            } // end iter given TgtLoops
        } // end given TgtLoops branch
    } // end TgtLoopsOnly is TRUE check
  else // include all loopuses branch              
    { 
      GetAll(SM_REINTERPRET_CAST(SmTArray<class SmTopology*>&,rLoopuses)) ; 
    }

} // end SmFaceuse::GetLoopuses

/*******************************************************************//**
PURPOSE: Get the edgeuses associated with this faceuse.

NOTES: 
***********************************************************************/
void SmFaceuse::GetEdgeuses
  (SmTArray<SmEdgeuse*> & rEdgeuses,     // out: all edgeuses connected to this faceuse
  SmBoolean                 bTgtLoopsOnly, // in : TRUE           = Only get Edgeuses from pTgtLoops loops
                                           //    : default:[FALSE]= get Edgeuses from all Face->Loops
  const SmTArray<SmLoop*> * pOptTgtLoops   // in : When bTgtLoopsOnly == TRUE                                
                                           //    : default:[NULL or Size=0] = Only use Face's OuterLoop
                                           //    : Size>0                   = Only use these Loops
 )const
{
  // init output
  rEdgeuses.ReSet();

  // locals
  SM_PTR_ARRAY(sLoopuses, SmLoopuse, 20) ;
  SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 20) ;

  // Loopuses (filtered as requested by input args)
  GetLoopuses(sLoopuses, bTgtLoopsOnly, pOptTgtLoops);

  // for every faceuse->loopuse
  for (ULONG i=0; i<sLoopuses.GetSize(); i++) 
    {
      // add all unique loopuse->edgeuses to output array
      SmLoopuse *pLU = sLoopuses[i];
      pLU->GetEdgeuses(sEdgeuses);
      for (ULONG j=0; j<sEdgeuses.GetSize(); j++) 
        {
          rEdgeuses.AddUnique(sEdgeuses[j]);
        }
    }

} // end SmFaceuse::GetEdgeuses

/*******************************************************************//**
PURPOSE: Get the faceuses connected to this faceuse through edgeuses
            as faceuse->edgeuse->radialMate->faceuse

NOTES: 
***********************************************************************/
void SmFaceuse::GetNeighbors
  (SmTArray<SmFaceuse*> & rFaceuses, // out: all faceuses connected through an edgeuse
                                     //      this faceuse is not added to output list
                                     //      unless its its own radial partner
                                     //      which happens when this->Face is closed.
   SmBoolean bSkipLaminaEdges)       // in : TRUE = skip lamina edges, FALSE=don't
                                     //      default:[FALSE]
 const
{
  // init output
  rFaceuses.ReSet();

  // locals
  SmLoopuse            * pLUData[20];
  SmEdgeuse            * pEUData[20];
  SmTArray<SmLoopuse*> sLoopuses(20,pLUData);
  SmTArray<SmEdgeuse*> sEdgeuses(20,pEUData);

  // for every faceuse->loopuse
  GetLoopuses(sLoopuses);
  for (ULONG i=0; i<sLoopuses.GetSize(); i++) 
    {
      // add all unique loopuse->edgeuses to output array
      SmLoopuse *pLU = sLoopuses[i];
      pLU->GetEdgeuses(sEdgeuses);
      for (ULONG j=0; j<sEdgeuses.GetSize(); j++) 
        {
          // for this edgeuse
          SmEdgeuse *pEdgeuse  = sEdgeuses[j] ;
          if(pEdgeuse == NULL) continue ;

          SmEdgeuse *pRadial   = pEdgeuse->GetRadial() ;
          if(pRadial == NULL) continue ;

          SmFaceuse *pNeighbor = pRadial->GetFaceuse() ;
          if(pNeighbor == NULL) continue ;

          // skip lamina edges when asked
          if(   bSkipLaminaEdges
             && pNeighbor->GetFace() == pEdgeuse->GetFaceuse()->GetFace())
            { continue ; }
          
          // else add unique neighbors to the list 
          rFaceuses.AddUnique(pNeighbor) ; 

        } // end iter every loopuse->edgeuse
    } // end iter every loopuse

} // end SmFaceuse::GetNeighbors

/*************************************************************
PURPOSE:  Find and return sampled points guaranteed to
  be within the interior of a faceuse.  The points are sampled
  in an unspecified manner and the UV, XYZ position, and
  Surface Normal for each sampled point is returned.  The
  direction of the Surface Normal is set to point into the
  region bounded by this faceuse.

RETURNS --- 
  SM_SUCCESS when the desired number of points are found, else
  SM_ERR     when input domain is degenerate (surface is a line or point)
             or when no internal points could be found by arbitrary sampling.
**************************************************************/
SmStatus  SmFaceuse::GetPointsInFace
  (ULONG lPointCount,             // in : number of scattered points from face interior to return
   SmTArray<SmPoint2d> &rUV,      // out: found UV Points 
   SmTArray<SmPoint3d> &rPV)      // out: associated positions and surface normals for found UV points
 const                            //      rUV[i] maps to rPV[2*i  ] = XYZ position
                                  //                     rPV[2*i+1] = Surface normal
{
  // pass the call along
  SmStatus sRtn = GetFace()->GetPointsInFace(lPointCount, rUV, rPV) ;

  // make sure the surface normal is in the direction of the Faceuse
  if(   sRtn           == SM_SUCCESS 
     && m_eOrientation == SM_OT_OPPOSITE)
    {
      ULONG ii ;
      for(ii=1;ii<rPV.GetSize();ii+=2)
        { 
          rPV[ii] = -rPV[ii] ;
        }
    }

  // all done
  return(sRtn) ;

} // end SmFaceuse::GetPointsInFace

/*******************************************************************//**
PURPOSE: Collect a set of faceuses which are on the same side as the
   input faceuse.  

NOTES: Do not go around lamina edges when doing this traversal.
***********************************************************************/
SmStatus SmFaceuse::CollectAdjacentFaceuses
  (SmTArray<SmFaceuse*> & rFaceuses,
   double *               pOptAngleThreshold)
 const
{
    // define a faceuse stack and an edgeuse array
    SmFaceuse *sFUData[64];
    SmTArray<SmFaceuse*> sFaceuseStack(64,sFUData);
    SmEdgeuse *sEUData[64];
    SmTArray<SmEdgeuse*> sEdgeuses(64,sEUData);

    // init faceuse stack
    sFaceuseStack.Add(SM_CONST_CAST(SmFaceuse*,this));

    // while there are faceuses on the stack
    while (sFaceuseStack.GetSize() > 0) 
      {
        // pull 1 faceuse off the stack
        SmFaceuse *pFU = sFaceuseStack.GetLast();
        sFaceuseStack.RemoveLast();
        pFU->GetEdgeuses(sEdgeuses);

        // for every face's edgeuse
        for (ULONG i=0; i<sEdgeuses.GetSize(); i++) 
          {
            SmEdgeuse *pEU = sEdgeuses[i];

            // skip lamina edges
            if (pEU->GetEdge()->IsLamina()) continue;

            // add the radial partner's face to output list and faceuse stack
            SmEdgeuse *pEURad = pEU->GetRadial();
            SmFaceuse *pFURad = pEURad->GetFaceuse();

            if (pOptAngleThreshold)
            {
                SmPoint3d sEUBinormalPoint;
                SmPoint3d sEURadBinormalPoint;
                SmVector3d sEUBinormal, sEUFUNormalMax, sEUFUNormalMin, sEUFUNormalMid;
                SmVector3d sEURadBinormal, sEURadFUNormalMax, sEURadFUNormalMin, sEURadFUNormalMid;

                double sAngle = 0.0;

                SmExtent1d sInt = pEU->GetEdge()->GetInterval();

                double sMinParam = sInt.GetMin();
                double sMidParam = sInt.GetMid();
                double sMaxParam = sInt.GetMax();

                pEU->EvaluateBinormal(sMinParam, TRUE, sEUBinormalPoint, sEUBinormal, NULL, &sEUFUNormalMin);
                pEURad->EvaluateBinormal(sMinParam, TRUE, sEURadBinormalPoint, sEURadBinormal, NULL, &sEURadFUNormalMax);
                SM_ASSERT(sEUBinormalPoint.DistanceBetween(sEURadBinormalPoint) < SM_EFF_ZERO);

                sEUFUNormalMin.AngleBetween(sEURadFUNormalMax, sAngle);

                if (sAngle > *pOptAngleThreshold) continue;

                pEU->EvaluateBinormal(sMidParam, TRUE, sEUBinormalPoint, sEUBinormal, NULL, &sEUFUNormalMid);
                pEURad->EvaluateBinormal(sMidParam, TRUE, sEURadBinormalPoint, sEURadBinormal, NULL, &sEURadFUNormalMid);
                SM_ASSERT(sEUBinormalPoint.DistanceBetween(sEURadBinormalPoint) < SM_EFF_ZERO);

                sEUFUNormalMid.AngleBetween(sEURadFUNormalMid, sAngle);

                if (sAngle > *pOptAngleThreshold) continue;

                pEU->EvaluateBinormal(sMaxParam, TRUE, sEUBinormalPoint, sEUBinormal, NULL, &sEUFUNormalMax);
                pEURad->EvaluateBinormal(sMaxParam, TRUE, sEURadBinormalPoint, sEURadBinormal, NULL, &sEURadFUNormalMin);
                SM_ASSERT(sEUBinormalPoint.DistanceBetween(sEURadBinormalPoint) < SM_EFF_ZERO);

                sEUFUNormalMax.AngleBetween(sEURadFUNormalMin, sAngle);

                if (sAngle > *pOptAngleThreshold) continue;
            }

            ULONG lFoundIndex;
            if (   pFURad != this 
                && !rFaceuses.FindElement(pFURad,lFoundIndex)) 
              {
                sFaceuseStack.Add(pFURad);
                rFaceuses.Add(pFURad);
              }
          } // end iter every edgeuse
      } // end while faces on the stack

    return SM_SUCCESS;

} // end SmFaceuse::CollectAdjacentFaceuses

/****************************************************************
PURPOSE: convenience function:
        Add Face graphics, with specified mode and hatch line counts,
        to new or open DisplayList added to global displayList array.

USAGE_NOTES --- 

METHOD ---
  1. Get GlobalDisplayParameters
  2. SetDisplayParameters(eMode)
  3. Do the Draw
****************************************************************/
SmDisplayList * SmFaceuse::Draw
  (SmDrawModeType  eMode,       // in : any SmDrawModeType; SM_DM_CURRENT = keep the current global display parameters,
                                //      SM_DM_FACEUSENEIGHBORS = also crosshatch the neighboring faceuses
   ULONG           lUHatch,     // in : evenly spaced U IsoParameter line count drawn on Face->Surface
   ULONG           lVHatch,     // in : evenly spaced V IsoParameter line count drawn on Face->Surface
   double          dNormalGain, // in : Amount to scale the Normal vector, 0.0 = Guess from Shell BBox Size, default:[0.0]
   SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // locals: global display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp);

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this, sDisp.GetShadedColorRule()), NULL, NULL, 0, pOptGfxSet);

  // Set SurfaceNormals to point into InfiniteRegion
  SmBoolean bSwapNormals =   (GetOrientation() == SM_OT_OPPOSITE) 
                           ? TRUE 
                           : FALSE ;

  // override global display parameters
  SmDrawModeType eLastDrawModeType = sDisp.m_eLastMode ;
  if(eMode != SM_DM_CURRENT)
    {
      smgfx_SetDisplayParameters(eMode, sDisp) ;
      sDisp.m_lCrossHatchUCount = lUHatch ;
      sDisp.m_lCrossHatchVCount = lVHatch ;
    }

  // always draw surface normal for this Faceuse
  sDisp.m_bDrawNormals = TRUE ;
  sDisp.m_bDrawFaceuse = TRUE ;
  sDisp.m_pFaceuse     = this ;
  smgfx_SetFaceuseNormalGain(dNormalGain) ;

  // output Face graphics
  GetFace()->OutputGraphics(sDisp, bSwapNormals, NULL, pOptGfxSet) ;

  // clear global display parameter DrawFaceuse data
  sDisp.m_bDrawFaceuse = FALSE ;
  sDisp.m_pFaceuse     = NULL ;

  // when asked - get neighbors
  if(eMode == SM_DM_FACEUSENEIGHBORS)
    {
      SmTArray<SmFaceuse *> sFaceuses ;
      GetNeighbors(sFaceuses) ;

      // draw each neigbhbor
      for(ULONG ii=0;ii<sFaceuses.GetSize();ii++)
        {
          SmFaceuse *pFaceuse = sFaceuses[ii] ;
          pFaceuse->Draw(SM_DM_CROSSHATCH, lUHatch, lVHatch, dNormalGain, pOptGfxSet) ;

        } // end iter every neighbor
    } // end draw neighbors check

  // restore the global display parameters to previous drawing mode
  smgfx_SetDrawingMode(eLastDrawModeType) ;
  smgfx_SetFaceuseNormalGain(0.0) ;

  // end new displayList
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF5(eMode, lUHatch, lVHatch, dNormalGain, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // SmFaceuse::Draw(Mode)

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertFaceuse_list[] =
{
  {SM_AT_POINTER, _T("Context"), _T("m_pF shares the same context") },
  {SM_AT_POINTER, _T("Context"), _T("m_pFUMate shares the same context") }
} ;


/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmFaceuse::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmOwningTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // SmFaceuse and the objects it attaches to need to share common contexts
  if(m_pF)      { bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (GetContext() == m_pF->GetContext()), _T("") ) ; }
  if(m_pFUMate) { bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (GetContext() == m_pFUMate->GetContext()), _T("") ) ; }

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmFaceuse::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmFaceuse::AssertHeal
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
//       return ( SmOwningTopology::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmFaceuse::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmFaceuse::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmFaceuse::IsKindOf( SM_TYPE t ) const
{
  return ((SmFaceuse_TYPE == t) ? TRUE : SmOwningTopology::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmFaceuse::Dump(void) const
{
    smos_WriteBuffer(_T("SmFaceuse object\n"));
}
