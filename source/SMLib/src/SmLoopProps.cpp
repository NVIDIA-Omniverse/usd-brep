// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmLoopProps.cpp
* PURPOSE: Source file for implementation of SmFaceProps methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmLoopProps.h>
#include <SmFaceProps.h>
#include <SmEdgeProps.h>
#include <SmVertexProps.h>
#include <SmFace.h>
#include <SmTol.h>
#include <SmLoop.h>
#include <SmVertex.h>
#include <SmGraphicsOutput.h>
#include <SmCurveClass.h>
#ifdef SM_DEBUG_CODE
  #include <SmBrep.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmClassifyLoopIO::Dump
  (ULONG lLabel)  // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\nBegin SmClassifyLoopIO Dump " ) ) ; }
  else                         { smos_sprintf(sBuff, _T("\nBegin SmClassifyLoopIO Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

  // header
  smos_sprintf(sBuff,        _T("\n SmClassifyLoopIO:[0x%p], Loop:[0x%p]"), this, m_pLoop) ;
  smos_sprintf(sBuffForFile, _T("\n SmClassifyLoopIO:[%s], Loop:[%s]"), _T("notNULL"), m_pLoop ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  smos_WriteBuffer( _T("\nEnd SmVertexProps Dump ")) ;

  // LoopuseType
  smos_sprintf(sBuff, _T("\n   Properties: LoopuseType:[%s]"), 
                    m_tLoopuseType == SmUnknown_TYPE   ? _T("UnInit") 
                  : m_tLoopuseType == SmVertexuse_TYPE ? _T("VertexLoop")
                  : m_tLoopuseType == SmEdgeuse_TYPE   ? _T("EdgeLoop")
                  : _T("Unknown Value")) ; 
  smos_WriteBuffer(sBuff) ;              

  // Orientation
  smos_sprintf(sBuff, _T("\n               m_eOrient:[%s]"), 
                    m_tLoopuseType == SM_OT_UNKNOWN   ? _T("UnInit") 
                  : m_tLoopuseType == SM_OT_SAME      ? _T("Same (OuterLoop or NoArea_BotLoop")
                  : m_tLoopuseType == SM_OT_OPPOSITE  ? _T("Opposite (InnerLoop or NoArea_TopLoop")
                  : _T("Unknown Value")) ; 
  smos_WriteBuffer(sBuff) ; 
  
  // Degenerate
  smos_sprintf(sBuff, _T("\n               m_bDegen_Loop:[%s]"), 
                    m_bDegen_Loop == UNSURE ? _T("UnInit") 
                  : m_bDegen_Loop == TRUE   ? _T("TRUE")
                  : m_bDegen_Loop == FALSE  ? _T("FALSE")
                  : _T("Unknown Value")) ; 
  smos_WriteBuffer(sBuff) ; 

  // Area_Loop
  smos_sprintf(sBuff, _T("\n               m_bArea_Loop:[%s]"), 
                    m_bArea_Loop == UNSURE ? _T("UnInit") 
                  : m_bArea_Loop == TRUE   ? _T("TRUE")
                  : m_bArea_Loop == FALSE  ? _T("FALSE")
                  : _T("Unknown Value")) ; 
  smos_WriteBuffer(sBuff) ; 

  // ContainmentType
  smos_sprintf(sBuff, _T("\n               m_eContainmentType:[%s]"), 
                    m_eContainmentType == SM_CMT_UNKNOWN           ? _T("UnInit") 
                  : m_eContainmentType == SM_CMT_OUTERLOOP         ? _T("OUTERLOOP")      
                  : m_eContainmentType == SM_CMT_INNERLOOP         ? _T("INNERLOOP")      
                  : m_eContainmentType == SM_CMT_NESTEDLOOP_EVEN   ? _T("NESTEDLOOP_EVEN")
                  : m_eContainmentType == SM_CMT_NESTEDLOOP_ODD    ? _T("NESTEDLOOP_ODD") 
                  : m_eContainmentType == SM_CMT_BOTLOOP           ? _T("BOTLOOP")        
                  : m_eContainmentType == SM_CMT_TOPLOOP           ? _T("TOPLOOP")        
                  : m_eContainmentType == SM_CMT_WIRELOOP          ? _T("WIRELOOP")       
                  : m_eContainmentType == SM_CMT_ERROR             ? _T("ERROR")          
                  : _T("Unknown Value")) ; 
  smos_WriteBuffer(sBuff) ; 

  // all done
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\nEnd   SmClassifyLoopIO Dump ") ) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd   SmClassifyLoopIO Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

} // end SmClassifyLoopIO::Dump

/*******************************************************************//**
PURPOSE: SmLoopProps Method implementations
***********************************************************************/

/*******************************************************************//**
PURPOSE: SmLoopProps Method implementations
***********************************************************************/

/*******************************************************************//**
 PURPOSE:
 NOTES:
***********************************************************************/
SmBoolean SmLoopProps::IsKindOf( SM_TYPE t ) const
{
  return ((SmLoopProps_TYPE == t) ? TRUE : SmObjProps::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: SmLoopProps Default Constructor

NOTES: 
***********************************************************************/
SmLoopProps::SmLoopProps(const SmContext * cpContext) 
{ 
  m_cpContext = cpContext ;
  ReSet() ; 

} // end SmLoopProps::SmLoopProps default constructor

/*******************************************************************//**
PURPOSE: SmLoopProps Constructor

NOTES: 
***********************************************************************/
SmLoopProps::SmLoopProps(const SmLoop * pLoop, const SmContext * cpContext) 
{ 
  m_cpContext = cpContext ;
  ReSet() ; 
  m_cpLoop = pLoop ; 
  //if(m_cpLoop) { m_cpLoop->SetLoopProps(this) ; }

} // end SmLoopProps::SmLoopProps default constructor

/*******************************************************************//**
PURPOSE: Deep Copy SmCurveClassification helper macro

NOTES: 
***********************************************************************/
#define SM_CRVCLASS_DEEP_COPY(To,From,cpCText) \
  { if(From) { if((To) == NULL)                                     \
                 { (To) = new (cpCText) SmCurveClassification() ; } \
               *(To) = *(From) ;                                    \
             }                                                      \
    else     { if(To) { delete (To) ; }                             \
               (To) = NULL ;                                        \
             }                                                      \
  }

/*******************************************************************//**
PURPOSE: SmLoopProps Copy Constructor

NOTES: 
***********************************************************************/
SmLoopProps::SmLoopProps(const SmLoopProps & crOther) 
{
  // locals
  m_cpContext = crOther.m_cpContext ; 

  m_cpLoop             = crOther.m_cpLoop ;
  m_tLoopuseType       = crOther.m_tLoopuseType ;
  m_bVertexLoopOnPole  = crOther.m_bVertexLoopOnPole ;
  m_lEdgeCnt           = crOther.m_lEdgeCnt ;
  m_lEdgeuseCnt        = crOther.m_lEdgeuseCnt ;
  m_bClosed3d          = crOther.m_bClosed3d ; 
  m_bClosedPtrs        = crOther.m_bClosedPtrs ; 
  m_bGoodClosed        = crOther.m_bGoodClosed ;       

  m_lSeamCrossingCntU  = crOther.m_lSeamCrossingCntU ;      
  m_lSeamWindingCntU   = crOther.m_lSeamWindingCntU ;      
  m_eWindingOrientU    = crOther.m_eWindingOrientU ;      

  m_lSeamCrossingCntV  = crOther.m_lSeamCrossingCntV ; 
  m_lSeamWindingCntV   = crOther.m_lSeamWindingCntV ; 
  m_eWindingOrientV    = crOther.m_eWindingOrientV ; 

  m_bDegen_Loop        = crOther.m_bDegen_Loop ; 
  m_bArea_Loop         = crOther.m_bArea_Loop ;            
  m_eContainmentType   = crOther.m_eContainmentType ; 
  m_eLoopOrient        = crOther.m_eLoopOrient ;   
  m_eDesiredLoopOrient = crOther.m_eDesiredLoopOrient ;
  m_bGoodOrient        = crOther.m_bGoodOrient ;       

} // end SmLoopProps::SmLoopProps copy constructor

/*******************************************************************//**
PURPOSE: SmLoopProps assignment operator

NOTES: 
***********************************************************************/
SmLoopProps & SmLoopProps::operator= (const SmLoopProps &crOther)
{
  // no work - same object
  if(this == &crOther)
    { return *this ; }

  m_cpContext          = crOther.m_cpContext ; 
                       
  m_cpLoop             = crOther.m_cpLoop ; 
  m_tLoopuseType       = crOther.m_tLoopuseType ;
  m_bVertexLoopOnPole  = crOther.m_bVertexLoopOnPole ;
  m_lEdgeCnt           = crOther.m_lEdgeCnt ;
  m_lEdgeuseCnt        = crOther.m_lEdgeuseCnt ;
  m_bClosed3d          = crOther.m_bClosed3d   ; 
  m_bClosedPtrs        = crOther.m_bClosedPtrs ; 
  m_bGoodClosed        = crOther.m_bGoodClosed ;       

  m_lSeamCrossingCntU  = crOther.m_lSeamCrossingCntU ;      
  m_lSeamWindingCntU   = crOther.m_lSeamWindingCntU ;      
  m_eWindingOrientU    = crOther.m_eWindingOrientU ;      

  m_lSeamCrossingCntV  = crOther.m_lSeamCrossingCntV ; 
  m_lSeamWindingCntV   = crOther.m_lSeamWindingCntV ; 
  m_eWindingOrientV    = crOther.m_eWindingOrientV ; 

  m_bArea_Loop         = crOther.m_bArea_Loop ;            
  m_bDegen_Loop        = crOther.m_bDegen_Loop ; 
  m_eContainmentType   = crOther.m_eContainmentType ;
  m_eLoopOrient        = crOther.m_eLoopOrient ;           
  m_eDesiredLoopOrient = crOther.m_eDesiredLoopOrient ;
  m_bGoodOrient        = crOther.m_bGoodOrient ;       

  // all done
  return(*this) ;
  
} // end SmLoopProps::operator= assignment operator

/*******************************************************************//**
PURPOSE: SmLoopProps equality operator

NOTES: 
***********************************************************************/
SmBoolean SmLoopProps::operator== (const SmLoopProps &crOther) const
{
  // locals
  SmBoolean bRtn = TRUE ; 

  // no work - same object
  if(this == &crOther)
    { return bRtn ; }

  bRtn &= m_cpLoop             == crOther.m_cpLoop ; 
  bRtn &= m_tLoopuseType       == crOther.m_tLoopuseType ;
  bRtn &= m_bVertexLoopOnPole  == crOther.m_bVertexLoopOnPole ;
  bRtn &= m_lEdgeCnt           == crOther.m_lEdgeCnt ;
  bRtn &= m_lEdgeuseCnt        == crOther.m_lEdgeuseCnt ;
  bRtn &= m_bClosed3d          == crOther.m_bClosed3d   ; 
  bRtn &= m_bClosedPtrs        == crOther.m_bClosedPtrs ; 
  bRtn &= m_bGoodClosed        == crOther.m_bGoodClosed ;       

  bRtn &= m_lSeamCrossingCntU  == crOther.m_lSeamCrossingCntU ;      
  bRtn &= m_lSeamWindingCntU   == crOther.m_lSeamWindingCntU ;      
  bRtn &= m_eWindingOrientU    == crOther.m_eWindingOrientU ;      

  bRtn &= m_lSeamCrossingCntV  == crOther.m_lSeamCrossingCntV ; 
  bRtn &= m_lSeamWindingCntV   == crOther.m_lSeamWindingCntV ; 
  bRtn &= m_eWindingOrientV    == crOther.m_eWindingOrientV ; 

  bRtn &= m_bArea_Loop         == crOther.m_bArea_Loop ;            
  bRtn &= m_bDegen_Loop        == crOther.m_bDegen_Loop ; 
  bRtn &= m_eContainmentType   == crOther.m_eContainmentType ;
  bRtn &= m_eLoopOrient        == crOther.m_eLoopOrient ; 
  bRtn &= m_eDesiredLoopOrient == crOther.m_eDesiredLoopOrient ;
  bRtn &= m_bGoodOrient        == crOther.m_bGoodOrient ;       

  // all done
  return(bRtn) ;
  
} // end SmLoopProps::operator== equality operator

/*******************************************************************//**
PURPOSE: initialize SmLoopProps all member values

NOTES: This method resets newly allocated SmEdgeProps and previously
       used SmEdgeProps objects back to the same uninitialized state.

       Leaves the m_cpContext unchanged. 
       (Simplifies reusing an SmLoopProps object)
***********************************************************************/
void SmLoopProps::ReSet()
{
  // nothing to do for Context

  // member values
  //if(m_cpLoop) { m_cpLoop->SetLoopProps(NULL) ; }
  m_cpLoop             = NULL ;

  m_tLoopuseType       = SmUnknown_TYPE ;  // [16999]
  m_bVertexLoopOnPole  = UNSURE ;
  m_lEdgeCnt           = SM_UNDEF_ULONG ;
  m_lEdgeuseCnt        = SM_UNDEF_ULONG ;
  m_bClosed3d          = UNSURE ;
  m_bClosedPtrs        = UNSURE ;
  m_bGoodClosed        = UNSURE ;       

  m_lSeamCrossingCntU  = SM_UNDEF_ULONG ; 
  m_lSeamWindingCntU   = SM_UNDEF_ULONG ; 
  m_eWindingOrientU    = SM_OT_UNKNOWN ; 

  m_lSeamCrossingCntV  = SM_UNDEF_ULONG ; 
  m_lSeamWindingCntV   = SM_UNDEF_ULONG ; 
  m_eWindingOrientV    = SM_OT_UNKNOWN ; 

  m_bArea_Loop         = UNSURE ;
  m_bDegen_Loop        = UNSURE ; 
  m_eDesiredLoopOrient = SM_OT_UNKNOWN ;
  m_eLoopOrient        = SM_OT_UNKNOWN ;
  m_eContainmentType   = SM_CMT_UNKNOWN ;
  m_bGoodOrient        = UNSURE ;
                     
} // end SmLoopProps::ReSet

/*******************************************************************//**
PURPOSE: Set all following SmLoopProps vals marked with '*'.
           (Vals marked with '##' = props set later in SmFaceProps::SetProps_Stage3())
         set SmLoopProps::m_cpLoop *              m_tLoopuseType *
                        ::m_bVertexLoopOnPole *
                        ::m_lEdgeCnt *            m_lEdgeuseCnt *
                        ::m_bClosed3d *           m_bClosedPtrs *
                        ::m_bGoodClosed *         
                        ::m_lSeamCrossingCntU *   m_lSeamCrossingCntV *
                        ::m_lSeamWindingCntU *    m_lSeamWindingCntV *
                        ::m_eWindingOrientU *     m_eWindingOrientV *  - set to SM_OT_UNKNOWN for Loops that don't cross seams
                        ::m_bDegen_Loop * - NotYetDone       
                        ::m_bArea_Loop *          
                        ::m_eContainmentType ##   m_eLoopOrient ##  - set later in SmFaceProps::SetProps_Stage3()      
                        ::m_eDesiredLoopOrient ## m_bGoodOrient ##
           * = value set in SmLoopProps::SetProps
           ## = value set later in SmFaceProps::SetProps_Stage3() 
NOTES:     
***********************************************************************/
SmStatus SmLoopProps::SetProps
  (const SmLoop * cpLoop,      // in : tgt Loop
   SmFaceProps  & rFaceProps   // in : Owner Face's FaceProps processed with SetProps_Stage2()
  )
{
  // init all values
  ReSet() ;

  // locals
  SmTArray<SmEdge*>    sEdges ;
  SmTArray<SmEdgeuse*> sEdgeuses ;
  SmTArray<SmVertex*>  sVerticesU ;
  SmTArray<SmVertex*>  sVerticesV ;  // Lists of Vertices where Loop crosses a Missing Seam
                           
  // gather data
  cpLoop->GetEdges(sEdges) ;
  cpLoop->GetEdgeuses(sEdgeuses) ;
            
  // set member values
  m_cpLoop       = cpLoop ;
  //cpLoop->SetLoopProps(this) ;
  m_tLoopuseType = cpLoop->GetLoopuse()->GetLoopuseType() ; // type of loop: SmVertexuse_TYPE (16006)   
                                                            //               SmEdgeuse_TYPE   (16005)   
                                                            //      default:[SmUnknown_TYPE   (16999)] ;
  // Get Loop Properties
  if(m_tLoopuseType == SmVertexuse_TYPE)
    {
      SmPoint3d sPoint3d  = cpLoop->GetLoopuse()->GetVertexuse()->GetVertex()->GetPoint() ; 
      m_bVertexLoopOnPole = rFaceProps.IsPoint3dOnPole(sPoint3d) ;
    }
  m_lEdgeCnt          = sEdges.GetSize() ;
  m_lEdgeuseCnt       = sEdgeuses.GetSize() ;
  m_bClosed3d         = cpLoop->IsClosed3d() ; 
  m_bClosedPtrs       = cpLoop->IsClosedPtrs() ;
  m_bGoodClosed       = m_bClosed3d == m_bClosedPtrs ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if (bDebugMe) 
    {
      if(m_bClosed3d == FALSE)
        {
          TCHAR sBuff[SM_TBLOCK_SIZE] ;
          SmEdgeuse   * pEdgeuse ;
          double        dMaxGap ;
          SmXSectTol3d  sXSectTol3d ; 
          m_bClosed3d = cpLoop->IsClosed3d( TRUE, &pEdgeuse, &dMaxGap, &sXSectTol3d) ;
          smos_sprintf(sBuff, _T("SmLoopuse::AssertValid - found loop with out of tol of Gap/XSectTol3d:[%16.16lf/%16.16lf] starting on Edge:[0x%p] "),
                              dMaxGap,
                              sXSectTol3d,
                              pEdgeuse->GetEdge()) ;
          smos_WriteBuffer(sBuff) ;
        }
    }
  #endif // SM_DEBUG_CODE

  // valid loops don't cross seams - they use them as part of the natrually trimmed outer-loop boundary
  // an invalid loop crossing a SurfaceSeam is missing a SeamEdge.  Area_Loops crossing a seam get fixed in two ways.
  // When possible the parameterization of the closed surface is changed so that the Area_Loop no longer crosses the seam.
  // When moving the seam won't fix the problem, a missing SeamEdge is created and inserted into the 
  // current Area_Loop.  That will split the loop into two child loops that will contain different sides of the
  // the added MissingEdge in their Loop-Edgeuse sequence. NoArea_Loops are organized into pairs and the MissingSeam edge
  // is created and inserted into the Brep which causes the paired NoArea_Loops to be connected 
  // creating a single valid AreaLoop that contains both sides of the inserted MissingSeam Edge in its edgeuse
  // sequence.

  SmTArray<SmTouchData> sThisTouchList ; 
  SmTArray<ULONG> sCrossingCounts ; 
  SmTArray<SmLoop*> sLoops ;
  sLoops.Add((SmLoop*)cpLoop) ; 

  // How many times does a Loop cross a U Seam
  if(rFaceProps.m_bClosedU == TRUE) 
       { // get this loop's touch list for the U Seam classify crv
         ULONG lTouchCnt     = SmTouchData::GetConnectedTouches((SmTopology *)cpLoop, rFaceProps.m_sDoneTouchListU, &sThisTouchList) ; 
          m_lSeamCrossingCntU = lTouchCnt/2 ;
         m_lSeamWindingCntU  = SmTouchData::GetWindingCount(sThisTouchList, m_eWindingOrientU) ;       
       }
  else { m_lSeamCrossingCntU = 0 ;
         m_lSeamWindingCntU  = 0 ;
         m_eWindingOrientU   = SM_OT_UNKNOWN ; 
       }

  // How many times does a Loop cross a V Seam
  if(rFaceProps.m_bClosedV == TRUE) 
       { // get this loop's touch list for the V Seam classify crv
         ULONG lTouchCnt     = SmTouchData::GetConnectedTouches((SmTopology *)cpLoop, rFaceProps.m_sDoneTouchListV, &sThisTouchList) ; 
         m_lSeamCrossingCntV = lTouchCnt / 2 ;
         m_lSeamWindingCntV  = SmTouchData::GetWindingCount(sThisTouchList, m_eWindingOrientV) ;
       }
  else { m_lSeamCrossingCntV = 0 ;
         m_lSeamWindingCntV  = 0 ; 
         m_eWindingOrientV   = SM_OT_UNKNOWN ;
       }

  // Is Loop a Area or NoArea_Loop - 
  //  for Lamina, SinglyPeriodic, and DoublyPeriodic surfaces
  //    NoArea_Loop = IsOdd(SeamCrossingCntU)  || IsOdd(SeamCrossingCntV)
  //    Area_Loop   = IsEven(SeamCrossingCntU) && IsEven(SeamCrossingCntV)
  //  VertexLoops are NoArea_Loops with zero SeamWindingCnts
  //  see white paper, "ClassifyLoops.docx"
  m_bArea_Loop =    smos_IsEven(m_lSeamCrossingCntU) 
                 && smos_IsEven(m_lSeamCrossingCntV) ;
#ifdef SM_DEBUG_CODE
  SmBoolean bArea_Loop2 = m_lSeamWindingCntU == 0 && m_lSeamWindingCntV == 0 ;
  SM_ASSERT_MSG(m_bArea_Loop == bArea_Loop2,
                _T("SmLoopProps::SetProps: unexpected bArea_Loop calc conflict between CrossingCnt and WindingCnt ideas - needs debug")) ;
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      rFaceProps.Dump() ;
      rFaceProps.m_pCrvClassU->Dump() ;
      rFaceProps.m_pCrvClassV->Dump() ;
      if(rFaceProps.m_bClosedU == TRUE) 
        {
          smos_sprintf(sBuff, _T("%s"),_T("Raw U: "));
          SmTouchData::DumpTouchList(sBuff, rFaceProps.m_sRawTouchListU, TRUE);
          smos_sprintf(sBuff, _T("%s"),_T("DoneU: "));
          SmTouchData::DumpTouchList(sBuff, rFaceProps.m_sDoneTouchListU, TRUE) ; 
        }
      if(rFaceProps.m_bClosedV == TRUE) 
        {
          smos_sprintf(sBuff, _T("%s"),_T("Raw V: "));
          SmTouchData::DumpTouchList(sBuff, rFaceProps.m_sRawTouchListV, TRUE);
          smos_sprintf(sBuff, _T("%s"),_T("DoneV: "));
          SmTouchData::DumpTouchList(sBuff, rFaceProps.m_sDoneTouchListV, TRUE) ; 
        }

      SmBrep    * pBrep = cpLoop->GetBrep() ;
      SmSurface * pSurface = cpLoop->GetFace()->GetSurface() ;
      
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ; 
      smgfx_SetLook(1,2, .3,.3,.3) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(cpLoop) cpLoop->Draw(3,FALSE,NULL,FALSE) ; sm_GraphicsLoop() ; 
      smgfx_SetLook(1,2, 0,1,1) ; rFaceProps.Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0) ; rFaceProps.m_pCrvClassU->Draw(FALSE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; rFaceProps.m_pCrvClassV->Draw(FALSE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmLoopProps::SetProps

/****************************************************************
PURPOSE: Return True when LoopProps has any problems

NOTES: 
****************************************************************/
SmBoolean SmLoopProps::HasProblems() const // rtn: TRUE=Obj has any problems
{
  // init rtn value
  SmBoolean bRtn = FALSE ;

  // check for problems
  bRtn |= m_bGoodClosed == FALSE ;
  bRtn |= m_bDegen_Loop == TRUE ;
  bRtn |= m_bArea_Loop  == FALSE ;
  bRtn |= (   m_lSeamCrossingCntU > 0                
           || m_lSeamCrossingCntV > 0) ;
  bRtn |= (   m_eContainmentType == SM_CMT_NESTEDLOOP_EVEN
           || m_eContainmentType == SM_CMT_NESTEDLOOP_ODD) ;
  bRtn |= m_bGoodOrient == FALSE ;

  // all done
  return(bRtn) ;

} // end SmLoopProps::HasProblems

/****************************************************************
PURPOSE: add graphics for problem Loops

NOTES: i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
       NULL to ignore. default:[NULL]
****************************************************************/
SmDisplayList * SmLoopProps::Draw
 ( SmVector3d sBadOrientColor,       // in : def:[.3,.3,.3]: Loop actual and desired orientations differ
   SmVector3d sBadDegen_LoopColor,   // in : def:[1.,.5, 0]: Loop has at least one degenerate dimension
   SmVector3d sBadNoArea_LoopColor,  // in : def:[ 0, 0, 0]: Loop is a NoArea_Loop       
   SmVector3d sBadContainmentColor,  // in : def:[.8,.2,.7]: Loop is a Nested Loop (i.e. inside an InnerLoop)
   SmVector3d sBadCrossingSeamColor, // in : def:[.2,.7,.5]: Loop crosses a Seam
   SmVector3d sBadClosed_LoopColor,  // in : def:[.4,.8,.2]: ClosedLoop3d != ClosedLoopPtrs (assumed intent is closed)
   SmGfxArraySet * pOptGfxSet        // in : When given, output GfxVertexArrays not GL calls.  
 ) const
{
  SM_REF1(sBadDegen_LoopColor);

  SmDisplayList *pRtn = NULL ;

  // skip healthy loops
  if(HasProblems() == FALSE)
    { return pRtn ; }

#ifdef SM_GFX_OUTPUT_CODE
  // locals: global display parameters
  // const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters();

  // start new displayList (unless one is already open)
  SmVector3d sFLoopPropsColor(0,1,0) ; 
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(sFLoopPropsColor, NULL, NULL, FALSE, pOptGfxSet);

  // output Problem Loop graphics

  // Bad Closed Loop Draw
  if(  m_bGoodClosed == FALSE)                 { smgfx_SetLook(3,5, sBadClosed_LoopColor) ; 
                                                 if(m_cpLoop) m_cpLoop->Draw(3, FALSE, NULL, FALSE, pOptGfxSet) ; // 3 = 0=NoObjuses, 1=Draw VUs, 2=Draw EUs, default:[3]=Draw VUs and EUs
                                               }
  // SeamCrossing Loop Draw
  if(   m_lSeamCrossingCntU > 0                
     || m_lSeamCrossingCntV > 0)               { smgfx_SetLook(6,8, sBadCrossingSeamColor) ; 
                                                 if(m_cpLoop) m_cpLoop->Draw(3, FALSE, NULL, FALSE, pOptGfxSet) ; // 3 = 0=NoObjuses, 1=Draw VUs, 2=Draw EUs, default:[3]=Draw VUs and EU
                                               }
  // Bad Degen_Loop Draw
  if(   m_bDegen_Loop == TRUE)                 { smgfx_SetLook(4,6, sBadNoArea_LoopColor) ; 
                                                 if(m_cpLoop) m_cpLoop->Draw(3, FALSE, NULL, FALSE, pOptGfxSet) ; // 3 = 0=NoObjuses, 1=Draw VUs, 2=Draw EUs, default:[3]=Draw VUs and EU
                                               }
  // Bad NoArea_Loop Draw
  if(   m_bArea_Loop == FALSE)                 { smgfx_SetLook(4,6, sBadNoArea_LoopColor) ; 
                                                 if(m_cpLoop) m_cpLoop->Draw(3, FALSE, NULL, FALSE, pOptGfxSet) ; // 3 = 0=NoObjuses, 1=Draw VUs, 2=Draw EUs, default:[3]=Draw VUs and EU
                                               }
  // Bad Containment Loop Draw
  if(   m_eContainmentType == SM_CMT_NESTEDLOOP_EVEN
     || m_eContainmentType == SM_CMT_NESTEDLOOP_ODD)
                                               { smgfx_SetLook(5,7, sBadContainmentColor) ; 
                                                 if(m_cpLoop) m_cpLoop->Draw(3, FALSE, NULL, FALSE, pOptGfxSet) ; // 3 = 0=NoObjuses, 1=Draw VUs, 2=Draw EUs, default:[3]=Draw VUs and EU
                                               }
  // Bad Orientation Loop Draw
  if(  m_bGoodOrient == FALSE)                 { smgfx_SetLook(3,5, sBadOrientColor) ; 
                                                 if(m_cpLoop) m_cpLoop->Draw(3, FALSE, NULL, FALSE, pOptGfxSet) ; // 3 = 0=NoObjuses, 1=Draw VUs, 2=Draw EUs, default:[3]=Draw VUs and EUs
                                               }
                                               
  // end new displayList
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF6(sBadOrientColor, sBadNoArea_LoopColor, sBadContainmentColor, sBadCrossingSeamColor, sBadClosed_LoopColor, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmLoopProps::Draw

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmLoopProps::Dump() const
{
  // pass the call along
  Dump(0) ;

}// end SmFaceProps::Dump()

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmLoopProps::Dump(ULONG lLabel)
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // optional label
  if(lLabel != SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("\n[%2lu] "),lLabel) ; }
  else                         { smos_sprintf(sBuff, _T("%s"), _T("\n")) ; } 
  smos_WriteBuffer(sBuff);

  // Begin
  smos_sprintf(sBuff,        _T("Begin SmLoopProps Dump: SmLoopProps[0x%p] for Loop:[0x%p] Dump "), this, m_cpLoop) ;
  smos_sprintf(sBuffForFile, _T("Begin SmLoopProps Dump: SmLoopProps[%s] for Loop:[%s] Dump "), _T("NotNULL"),  m_cpLoop ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // base class
  SmObjProps::Dump() ;

  // Properties: Closed, LoopType, SeamCrossing, Area_Loop, ContainmentType
  smos_sprintf(sBuff, _T("\n    Properties: Closed3d        :[%s] - %s"), (m_bClosed3d == TRUE)  ? _T("TRUE") 
                                                                      : (m_bClosed3d == FALSE) ? _T("FALSE") 
                                                                                               : _T("UNSURE"),
                                                                        (m_bGoodClosed == TRUE)  ? _T("Okay - Closed3d == ClosedPtrs") 
                                                                      : (m_bGoodClosed == FALSE) ? _T("Bad - Closed3d != ClosedPtrs") 
                                                                      :                            _T("Could be okay")) ; 
  smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n    Properties: ClosedPtrs      :[%s]"), (m_bClosedPtrs == TRUE) ? _T("TRUE") : (m_bClosedPtrs == FALSE) ? _T("FALSE") : _T("UNSURE")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n              : LoopType        :[%s]"), 
                      m_tLoopuseType == SmVertexuse_TYPE ? _T("VertexLoop")
                    : m_tLoopuseType == SmEdgeuse_TYPE   ? _T("EdgeuseLoop")
                    : m_tLoopuseType == SmUnknown_TYPE   ? _T("Unknow")
                    :                                      _T("Unexpected type")) ; 
  smos_WriteBuffer(sBuff) ;

  smos_sprintf(sBuff, _T("\n              : VertexLoopOnPole:[%s]"),  (m_bVertexLoopOnPole == TRUE)  ? _T("TRUE") 
                                                                  : (m_bVertexLoopOnPole == FALSE) ? _T("FALSE") 
                                                                                                   : _T("UNSURE")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n              : EdgeCnt         :[%2lu]"), m_lEdgeCnt) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n              : EdgeuseCnt      :[%2lu]"), m_lEdgeuseCnt) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n              : SeamCrossingCntU:[%2lu = %s]"), m_lSeamCrossingCntU, _T("number of loop crossings made by a LowParamToHighParam running ClassifyCrv parallel to Seam")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n              : SeamWindingCntU :[%2lu = %s]"), m_lSeamWindingCntU , _T("number of unmatched in/out loop crossings made by LowParamToHighParam running ClassifyCrv parallel to Seam")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n              : WindingOrientU  :[%s]"), 
               m_eWindingOrientU == SM_OT_UNKNOWN          ? _T("UNKNOWN = not yet computed or Loop is Area_Loop") 
             : m_eWindingOrientU == SM_OT_UPPERDOMAIN      ? _T("UpperDomain = UpperDomain is 'in' side of this NoArea_Loop")
             : m_eWindingOrientU == SM_OT_LOWERDOMAIN      ? _T("LowerDomain = LowerDomain is 'in' side of this NoArea_Loop")
             : _T("Unexpected Value")) ;
  smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n              : SeamCrossingCntV:[%2lu = %s]]"), m_lSeamCrossingCntV, _T("number of loop crossings made by a LowParamToHighParam running ClassifyCrv parallel to Seam")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n              : SeamWindingCntV :[%2lu = %s]]"), m_lSeamWindingCntV , _T("number of unmatched in/out loop crossings made by LowParamToHighParam running ClassifyCrv parallel to Seam")) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff, _T("\n              : WindingOrientV  :[%s]"), 
               m_eWindingOrientV == SM_OT_UNKNOWN         ? _T("UNKNOWN = not yet computed or Loop is Area_Loop") 
             : m_eWindingOrientV == SM_OT_UPPERDOMAIN      ? _T("UpperDomain = UpperDomain is 'in' side of this NoArea_Loop")
             : m_eWindingOrientV == SM_OT_LOWERDOMAIN      ? _T("LowerDomain = LowerDomain is 'in' side of this NoArea_Loop")
             : _T("Unexpected Value")) ; 
  smos_WriteBuffer(sBuff) ;

  // problems (mixed with data to report)
  SmBoolean bHasProbs = HasProblems() ; 

  // prob section header - and prob only output
  if(!bHasProbs) { smos_WriteBuffer( _T("\n Loop has no problems           : Okay")) ; 
                 }
  else           { smos_WriteBuffer( _T("\n Loop has problems              : Bad")) ;
                   smos_sprintf(sBuff, _T("\n              : Degen_Loop      :[%s"),  (m_bDegen_Loop == FALSE) ? _T("FALSE] - Okay") 
                                                                                  : (m_bDegen_Loop == TRUE)  ? _T("Degen_Loop] - Bad - No Fix Yet") 
                                                                                  :                            _T("Not Yet Checked] - could be okay")) ; 
                   smos_WriteBuffer(sBuff) ; 
                   smos_sprintf(sBuff, _T("\n              : Area_Loop       :[%s"),  (m_bArea_Loop == TRUE)  ? _T("TRUE] - Okay") 
                                                                                  : (m_bArea_Loop == FALSE) ? _T("NoArea_Loop] - Bad") 
                                                                                  :                           _T("Not Yet Checked] - could be okay")) ; 
                   smos_WriteBuffer(sBuff) ; 
                 }

  // data always printed with or without problems
  // m_eContainmentType
  smos_sprintf(sBuff, _T("\n              : ContainmentType :[%s"), 
               m_eContainmentType == SM_CMT_UNKNOWN         ? _T("UNKNOWN = not yet computed] - could be okay") 
             : m_eContainmentType == SM_CMT_OUTERLOOP       ? _T("OUTERLOOP = Area_Loop not in another, contains all UVPoints 'inside' loop] - Okay")
             : m_eContainmentType == SM_CMT_INNERLOOP       ? _T("INNERLOOP = Area_Loop in another, excludes all UVPoints 'inside' loop] - Okay")
             : m_eContainmentType == SM_CMT_NESTEDLOOP_EVEN ? _T("NESTEDLOOP_EVEN = Area_Loop (invalid) nested to even depth(0,2,4) in InnerLoop - heal to OuterLoop] - Bad")
             : m_eContainmentType == SM_CMT_NESTEDLOOP_ODD  ? _T("NESTEDLOOP_ODD = Area_Loop (invalid) nested to odd depth(1,3,5) in InnerLoop - heal to InnerLoop] - Bad")
             : m_eContainmentType == SM_CMT_BOTLOOP         ? _T("BOTLOOP = NoArea_Loop (invalid) contains upperdomain UVPoints - AddMisingSeam heals to OuterLoop] - Bad")
             : m_eContainmentType == SM_CMT_TOPLOOP         ? _T("TOPLOOP = NoArea_Loop (invalid) contains lowerdomain UVPoints - AddMisingSeam heals to OuterLoop] - Bad")
             : m_eContainmentType == SM_CMT_WIRELOOP        ? _T("WIRELOOP = A set of connected Edgeuses without a closed portion] - Okay")
             : m_eContainmentType == SM_CMT_ERROR           ? _T("ERROR = Error value assigned for unexpected cases] - Bad programming bug")
             : _T("Unexpected Value")) ; 
  smos_WriteBuffer(sBuff) ;
  
  // Current Orient
  smos_sprintf(sBuff, _T("\n              : CurrentOrient   :[%s] - %s"), 
               m_eLoopOrient == SM_OT_UNKNOWN  ? _T("UNKNOWN = not yet computed") 
             : m_eLoopOrient == SM_OT_SAME     ? _T("SAME = CCW (OuterLoop or UpperDomain)") 
             : m_eLoopOrient == SM_OT_OPPOSITE ? _T("OPPOSITE = CW (InnerLoop or LowerDomain)")
             : _T("Unexpected Value"),
               (m_bGoodOrient == TRUE)  ? _T("Okay") 
             : (m_bGoodOrient == FALSE) ? _T("Bad") 
             :                            _T("Could be okay")
             ) ; 
  smos_WriteBuffer(sBuff) ;

  // Desired Orient
  smos_sprintf(sBuff, _T("\n              : DesiredOrient   :[%s]"), 
               m_eDesiredLoopOrient == SM_OT_UNKNOWN  ? _T("UNKNOWN = not yet computed") 
             : m_eDesiredLoopOrient == SM_OT_SAME     ? _T("SAME = CCW (OuterLoop or UpperDomain)") 
             : m_eDesiredLoopOrient == SM_OT_OPPOSITE ? _T("OPPOSITE = CW (InnerLoop or LowerDomain)")
             : _T("Unexpected Value")) ; 
  smos_WriteBuffer(sBuff) ;

  // optional label
  if(lLabel != SM_UNDEF_ULONG) { SM_SPRINTF(sBuff, _T("\n[%2lu] "),lLabel) ; }
  else                         { SM_SPRINTF(sBuff, _T("%s"), _T("\n")) ; }
  smos_WriteBuffer(sBuff);

  // all done
  SM_SPRINTF(sBuff,        _T("End SmLoopProps Dump: SmLoopProps[0x%p]"), this) ;
  SM_SPRINTF(sBuffForFile, _T("End SmLoopProps Dump: SmLoopProps[%s]"), _T("NotNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

} // end SmLoopProps::Dump

/*******************************************************************//**
  BEGIN SmLooptreeItem Method implementations
***********************************************************************/

/*******************************************************************//**
PURPOSE: Area_Loop and Start NoArea_Loop pair constructor (See AddPartner() to manage NoArea_Loop pairs in pieces)

NOTES: 1. a NoArea_loop pair may be built in pieces
          a. Use this constructor to create a LooptreeItem with
             just the 1st of a NoArea_Loop pair.
          b. use AddPartner() to add the 2nd of a NoArea_Loop pair
             to this SmLooptreeItem object.
       2. leaves Child and Sibling pointers NULL.  
          Must be added to a tree by the caller.
***********************************************************************/
SmLooptreeItem::SmLooptreeItem
 (SmLoop           * pLoop,             
  SmLoopProps      * pLoopProps,
  SmClassifyLoopIO * pClassifyLoopIO) 
 { ReSetNoRecursion() ;
   m_bPoleVertex     = (pLoop == NULL) ;
   m_pLoop           = pLoop ;                                
   m_pLoopProps      = pLoopProps ;
   m_pClassifyLoopIO = pClassifyLoopIO ;

} // end SmLooptreeItem::SmLooptreeItem constructor(Area_Loop or 1st of NoArea_Loop pair)

/*******************************************************************//**
PURPOSE: NoArea_Loop pair constructor

NOTES: leaves Child and Sibling pointers NULL.  
       Must be added to a tree by the caller.
***********************************************************************/
SmLooptreeItem::SmLooptreeItem
 (SmLoop           * pLoop,                  // in : LowerParam NoArea_Loop or NULL=VertexPole
  SmLoopProps      * pLoopProps,             // in : associated LoopProps   or NULL=VertexPole
  SmClassifyLoopIO * pClassifyLoopIO,        // in : associated ClassifyLoopIO or NULL=VertexPole
  SmLoop           * pPartnerLoop,           // in : UpperParam NoArea_Loop or NULL=VertexPole
  SmLoopProps      * pPartnerLoopProps,      // in : associated LoopProps   or NULL=VertexPole
  SmClassifyLoopIO * pPartnerClassifyLoopIO) // in : associated ClassifyLoopIO or NULL=VertexPole
{ ReSetNoRecursion() ;
  m_bPoleVertex            = (pLoop == NULL) ;
  m_pLoop                  = pLoop ;                                
  m_pLoopProps             = pLoopProps ;  
  m_pClassifyLoopIO        = pClassifyLoopIO ;
  m_bPartnerPoleVertex     = (pPartnerLoop == NULL) ;
  m_pPartnerLoop           = pPartnerLoop ;                                
  m_pPartnerLoopProps      = pPartnerLoopProps ;
  m_pPartnerClassifyLoopIO = pPartnerClassifyLoopIO ;

} // end SmLooptreeItem::SmLooptreeItem constructor(NoArea_Loop pair)

/*******************************************************************//**
PURPOSE: deep copy construtor - with recursion

NOTES: duplicates this item and its entire descendants tree
***********************************************************************/
SmLooptreeItem::SmLooptreeItem
 (const SmLooptreeItem & crOther)  
{ m_bPoleVertex            = crOther.m_bPoleVertex ; 
  m_pLoop                  = crOther.m_pLoop ;
  m_pLoopProps             = crOther.m_pLoopProps ;
  m_pClassifyLoopIO        = crOther.m_pClassifyLoopIO ;
  m_bPartnerPoleVertex     = crOther.m_bPartnerPoleVertex ; 
  m_pPartnerLoop           = crOther.m_pPartnerLoop ;
  m_pPartnerLoopProps      = crOther.m_pPartnerLoopProps ;
  m_pPartnerClassifyLoopIO = crOther.m_pPartnerClassifyLoopIO ;
  if(crOther.m_pSibling) { m_pSibling = new SmLooptreeItem(*crOther.m_pSibling) ; }
  if(crOther.m_pChild)   { m_pChild   = new SmLooptreeItem(*crOther.m_pChild) ; }

} // end SmLooptreeItem::SmLooptreeItem Copy constructor - with recursion

/*******************************************************************//**
PURPOSE: init all member values to uninit values - this node only

NOTES: limited to this node only - does nothing to descendants
***********************************************************************/
void SmLooptreeItem::ReSetNoRecursion()                         
{ m_bPoleVertex     = m_bPartnerPoleVertex     = FALSE ;
  m_pLoop           = m_pPartnerLoop           = NULL ;
  m_pLoopProps      = m_pPartnerLoopProps      = NULL ;
  m_pClassifyLoopIO = m_pPartnerClassifyLoopIO = NULL ;
  m_pSibling        = NULL ; 
  m_pChild          = NULL ; 

} // end SmLooptreeItem::ReSetNoRecursion

/*******************************************************************//**
PURPOSE: assign all values except Sibling and Child Ptrs - those are set to NULL

NOTES: 1. Only this node is assigned, not the descendants
       2. to use SmTArray<SmLooptreeItem> must support operator=
***********************************************************************/
SmLooptreeItem & SmLooptreeItem::operator= 
 (const SmLooptreeItem &crOther)  
{ if(this == &crOther)
    { return *this ; }
  m_bPoleVertex            = crOther.m_bPoleVertex ; 
  m_pLoop                  = crOther.m_pLoop ;
  m_pLoopProps             = crOther.m_pLoopProps ;
  m_pClassifyLoopIO        = crOther.m_pClassifyLoopIO ;
  m_bPartnerPoleVertex     = crOther.m_bPartnerPoleVertex ; 
  m_pPartnerLoop           = crOther.m_pPartnerLoop ;
  m_pPartnerLoopProps      = crOther.m_pPartnerLoopProps ;
  m_pPartnerClassifyLoopIO = crOther.m_pPartnerClassifyLoopIO ;
  m_pSibling = NULL ;
  m_pChild   = NULL ;
  return(*this) ;

} // end SmLooptreeItem::operator=  - no recursion

/*******************************************************************//**
PURPOSE: LooptreeItem compare - node only

NOTES: 1. Only this node is compared, not the descendants
       2. to use SmTArray<SmLooptreeItem> must support operator==
***********************************************************************/
SmBoolean SmLooptreeItem::operator==
 (const SmLooptreeItem &crOther) 
 const  
{ SmBoolean bRtn = TRUE ;
  if(this == &crOther)
    { return bRtn ; }
  bRtn &= m_bPoleVertex            == crOther.m_bPoleVertex ; 
  bRtn &= m_pLoop                  == crOther.m_pLoop ;
  bRtn &= m_pLoopProps             == crOther.m_pLoopProps ;
  bRtn &= m_pClassifyLoopIO        == crOther.m_pClassifyLoopIO ;
  bRtn &= m_bPartnerPoleVertex     == crOther.m_bPartnerPoleVertex ; 
  bRtn &= m_pPartnerLoop           == crOther.m_pPartnerLoop ;
  bRtn &= m_pPartnerLoopProps      == crOther.m_pPartnerLoopProps ;
  bRtn &= m_pPartnerClassifyLoopIO == crOther.m_pPartnerClassifyLoopIO ;
  return(bRtn) ;

} // end SmLooptreeItem::operator==  - no recursion

/*******************************************************************//**
PURPOSE: destructor - with recursion

NOTES: this LooptreeItem and all its siblings and descendants are deleted
***********************************************************************/
SmLooptreeItem::~SmLooptreeItem()                                
{ m_bPoleVertex     = m_bPartnerPoleVertex     = FALSE ; 
  m_pLoop           = m_pPartnerLoop           = NULL ;
  m_pLoopProps      = m_pPartnerLoopProps      = NULL ;
  m_pClassifyLoopIO = m_pPartnerClassifyLoopIO = NULL ; 
  if(m_pSibling) { delete m_pSibling ; m_pSibling = NULL ; }
  if(m_pChild  ) { delete m_pChild   ; m_pChild   = NULL ; }
  // if(this)       { delete this ; }  // don't need a this recursion - causes an infinite loop and stack overflow

} // end SmLooptreeItem::~SmLooptreeItem destructor - with recursion

/*******************************************************************//**
PURPOSE: Add to array a list starting with this TreeItem and all its descendants

NOTES: 
***********************************************************************/
void SmLooptreeItem::GetDescendants  
 (SmTArray<SmLooptreeItem> & rDescendants,        // out: ordered:[this plus all descendants]
  SmTArray<SmLoop *>        & rLoopDescendants,    // out: Associated List of just Loop ptrs for convenience
  SmBoolean                   bReSet)              // in : default:[TRUE] = ReSet output rDescendants prior to gathering the descendants
                                                   //      FALSE= don't reset just append this and descendants
const
 {
  // when asked init return values
  if(bReSet)
    { 
      rDescendants.ReSet() ;
      rLoopDescendants.ReSet() ;
    }

  // augment OutputLists - include ParnterLoops in LoopDescendants when they exist
  rDescendants.Add(*this) ;
  rLoopDescendants.Add(this->m_pLoop) ;
  if(m_pPartnerLoop) { rLoopDescendants.Add(this->m_pPartnerLoop) ; }

  // child recursion
  if(m_pChild) 
    { m_pChild->GetDescendants(rDescendants, rLoopDescendants, FALSE) ; }

  // sibling recurse
  if(m_pSibling) 
    { m_pSibling->GetDescendants(rDescendants, rLoopDescendants, FALSE) ; }

} // end SmLooptreeItem::GetDescendants          

/*******************************************************************//**
PURPOSE: Find a Looptree Item

NOTES:
***********************************************************************/
SmLooptreeItem * SmLooptreeItem::FindItem  // rtn: Ptr to LooptreeItem with pLoopProps or NULL=Not Found
 (const SmLoop    * pLoop,                // in : Tgt pLoop to find in the Looptree, NULL = VertexPole
  SmLooptreeItem *& rpParent)              // out: If FoundItem, Parent of FoundItem or NULL= Parent is TreeRoot
 {
  // init return values
  rpParent              = NULL ;
  SmLooptreeItem * pRtn = NULL ;

  // Exit condition - found the item
  if(   (   (m_bPoleVertex == TRUE && pLoop == NULL)        // good for VertexPole_Loops
         || (m_pLoop       == pLoop))                       // good for Area_Loops and NoArea_Loops
     || (   (m_bPartnerPoleVertex == TRUE && pLoop == NULL) // good for partner VertexPole_Loops
         || (m_pPartnerLoop       == pLoop)))               // good for partner NoArea_Loops (there are no partner Area_Loops)
    { return(this) ; }

  // recurse Siblings
  if(m_pSibling != NULL)
    {
     pRtn = m_pSibling->FindItem(pLoop, rpParent) ;
     if(pRtn)
        { // found the item in sibling SubTree
          return( pRtn ) ; 
        }
    }

  // arrive here when pRtn == NULL - item not in Sibling SubTree

  // recurse Children
  if(m_pChild != NULL)
    { 
      pRtn = m_pChild->FindItem(pLoop, rpParent) ;
      if(pRtn)
        { // found the item in child SubTree
          if(rpParent == NULL) { rpParent = this ; }
          return( pRtn ) ; 
        }
    }

  // all done - not found here or in this Child and Sibling SubTrees
  return( NULL ) ;

} // end SmLooptreeItem::FindItem          

/*******************************************************************//**
PURPOSE: Pretty Print Looptree

NOTES:
***********************************************************************/
void SmLooptreeItem::Dump
 (ULONG             lGen,   // in : default:[0] = Root, 1 = Child, 2 = GrandChildren, . . .
  SmLooptreeItem * pParent) // in : Parent, default:[NULL] = this is RootNode   
{
  // locals
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sTab[SM_TBLOCK_SIZE] ;

  // build indent
  sTab[0] = '\0'; 
  for(ii=0;ii<=lGen;ii++) { smos_sprintf(sTab, _T("%s"), _T("  ") ) ; }

  // header
  if(lGen == 0)
    {
      smos_sprintf(sBuff, _T("%s"), _T("\n  Begin SmLooptree::Dump()") ) ;
      smos_WriteBuffer(sBuff) ;
    }

  /* Item report
     Gen( 0): Type[NoArea_Loop Start], Containment[SM_CMT_BOTLOOP         ], Loop[0x000000004F52E0A0], LoopProps[0x000000004F32B938], ClassifyIO[0x0000000000000000]
            : Type[NoArea_Loop Pair ], Containment[SM_CMT_TOPLOOP         ], Loop[0x000000004F52E0A0], LoopProps[0x000000004F32B8F0], ClassifyIO[0x0000000000000000]
              Parent[0x0000000000000000], Sibling[0x0000000000000000]

     Type Options:[NoArea_Loop Start], Containment Options:[UnKnown               ]
                  [NoArea_Loop Pair ]                      [SM_CMT_OUTERLOOP      ]
                  [Area_Loop        ]                      [SM_CMT_INNERLOOP      ]
                  [Pole_Loop   Start]                      [SM_CMT_NESTEDLOOP_EVEN]
                  [Pole_Loop   End  ]                      [SM_CMT_NESTEDLOOP_ODD ]
                  [Uninit           ]                      [SM_CMT_BOTLOOP        ]
                                                           [SM_CMT_TOPLOOP        ]
                                                           [SM_CMT_WIRELOOP       ]
                                                           [SM_CMT_UNKNOWN        ]
                                                           [SM_CMT_ERROR          ]
                                                           [Start VertexPole      ]
                                                           [End   VertexPole      ]
                                                           [Unavailable           ]
  */

  // Item Intro
  SmBoolean bArea_Loop = m_pLoopProps ? m_pLoopProps->m_bArea_Loop
                                      : m_pClassifyLoopIO ? m_pClassifyLoopIO->m_bArea_Loop
                                      : UNSURE ;

  // generation
  smos_sprintf(sBuff, _T("\n  %sGen(%2lu):"), sTab, lGen) ; 
  smos_WriteBuffer(sBuff) ;

  // Type
  smos_sprintf(sBuff, _T(" Type[%s]"),  (m_bPoleVertex == FALSE && m_pLoop && bArea_Loop == TRUE)  ? _T("Area_Loop        ")
                                    : (m_bPoleVertex == FALSE && m_pLoop && bArea_Loop == FALSE) ? _T("NoArea_Loop Start")
                                    : (m_bPoleVertex == TRUE)                                    ? _T("Pole_Loop   Start")
                                    :                                                              _T("Uninit           ")) ;
  smos_WriteBuffer(sBuff) ;

  // item Containment
  SmContainmentType eContainmentType = GetContainmentType() ;

  if(   m_bPoleVertex == FALSE
     && eContainmentType != SM_CMT_UNKNOWN) 
    { smos_sprintf(sBuff,_T(", Containment[%s]"),
        eContainmentType == SM_CMT_OUTERLOOP       ? _T("SM_CMT_OUTERLOOP      ")
      : eContainmentType == SM_CMT_INNERLOOP       ? _T("SM_CMT_INNERLOOP      ")
      : eContainmentType == SM_CMT_NESTEDLOOP_EVEN ? _T("SM_CMT_NESTEDLOOP_EVEN")
      : eContainmentType == SM_CMT_NESTEDLOOP_ODD  ? _T("SM_CMT_NESTEDLOOP_ODD ")
      : eContainmentType == SM_CMT_BOTLOOP         ? _T("SM_CMT_BOTLOOP        ")
      : eContainmentType == SM_CMT_TOPLOOP         ? _T("SM_CMT_TOPLOOP        ")
      : eContainmentType == SM_CMT_WIRELOOP        ? _T("SM_CMT_WIRELOOP       ")
      : eContainmentType == SM_CMT_UNKNOWN         ? _T("SM_CMT_UNKNOWN        ")
      : eContainmentType == SM_CMT_ERROR           ? _T("SM_CMT_ERROR          ")
      : _T("UnRecognized")) ;
    }
  else if(m_bPoleVertex == TRUE) { smos_sprintf(sBuff,_T("%.1024s"), ", Containment[Start VertexPole      ]" ) ; }
  else                           { smos_sprintf(sBuff,_T("%.1024s"), ", Containment[Unavailable           ]" ) ; }
  
  smos_WriteBuffer(sBuff) ;

  // item Loop
  if(m_pLoop != NULL) { smos_sprintf(sBuff,_T(", Loop[0x%p]"), m_pLoop) ; }
  else                { smos_sprintf(sBuff,_T("%s") , ", Loop[NULL]") ; }
  smos_WriteBuffer(sBuff) ;

  // item LoopProps
  if(m_pLoopProps != NULL) { smos_sprintf(sBuff,_T(", LoopProps[0x%p]"), m_pLoopProps) ; }
  else                     { smos_sprintf(sBuff,_T("%s") , ", LoopProps[NULL]") ; }
  smos_WriteBuffer(sBuff) ; 
  
  // item ClassifyIO
  if(m_pClassifyLoopIO != NULL) { smos_sprintf(sBuff,_T(", ClassifyIO[0x%p]"), m_pClassifyLoopIO) ; }
  else                          { smos_sprintf(sBuff,_T("%s") , ", ClassifyIO[NULL]" ); }
  smos_WriteBuffer(sBuff) ;  

  // NoArea_Loop pair
  if(IsNoArea_Pair())
    {
      // Partner intro
      smos_sprintf(sBuff, _T("\n  %s       : Type[%s]"), 
                 sTab, 
               (m_bPartnerPoleVertex == FALSE && m_pPartnerLoopProps && m_pPartnerLoopProps->m_bArea_Loop == FALSE) ? _T("NoArea_Loop Pair ")
             : (m_bPartnerPoleVertex == TRUE)                                                                       ? _T("Pole_Loop   End  ")
             :                                                                                                        _T("Uninit           ")) ;
      smos_WriteBuffer(sBuff) ;                                                                           

      // Partner Containment
      eContainmentType = GetPartnerContainmentType() ;

      if(   m_bPartnerPoleVertex == FALSE
         && m_pPartnerLoopProps) { smos_sprintf(sBuff,_T(", Containment[%s]"),
                                      eContainmentType == SM_CMT_OUTERLOOP       ? _T("SM_CMT_OUTERLOOP      ")
                                    : eContainmentType == SM_CMT_INNERLOOP       ? _T("SM_CMT_INNERLOOP      ")
                                    : eContainmentType == SM_CMT_NESTEDLOOP_EVEN ? _T("SM_CMT_NESTEDLOOP_EVEN")
                                    : eContainmentType == SM_CMT_NESTEDLOOP_ODD  ? _T("SM_CMT_NESTEDLOOP_ODD ")
                                    : eContainmentType == SM_CMT_BOTLOOP         ? _T("SM_CMT_BOTLOOP        ")
                                    : eContainmentType == SM_CMT_TOPLOOP         ? _T("SM_CMT_TOPLOOP        ")
                                    : eContainmentType == SM_CMT_WIRELOOP        ? _T("SM_CMT_WIRELOOP       ")
                                    : eContainmentType == SM_CMT_UNKNOWN         ? _T("SM_CMT_UNKNOWN        ")
                                    : eContainmentType == SM_CMT_ERROR           ? _T("SM_CMT_ERROR          ")
                                    :                                              _T("UnKnown               ")) ;
                                 }
      else if(m_bPartnerPoleVertex == TRUE) { smos_sprintf(sBuff,_T("%s"), _T(", Containment[End   VertexPole      ]") ) ; }
      else                                  { smos_sprintf(sBuff,_T("%s"), _T(", Containment[Unavailable           ]") ) ; }
      smos_WriteBuffer(sBuff) ;

      // item Partner Loop
      if(m_pPartnerLoop != NULL) { smos_sprintf(sBuff,_T(", Loop[0x%p]"), m_pPartnerLoop) ; }
      else                       { smos_sprintf(sBuff,_T("%.1024s"), _T(", Loop[NULL]") ) ; }
      smos_WriteBuffer(sBuff) ;

      // item Partner LoopProps
      if(m_pPartnerLoopProps != NULL) { smos_sprintf(sBuff,_T(", LoopProps[0x%p]"), m_pPartnerLoopProps) ; }
      else                            { smos_sprintf(sBuff,_T("%.1024s"), _T(", LoopProps[NULL]") )  ; }
      smos_WriteBuffer(sBuff) ; 
  
      // item Partner ClassifyIO
      if(m_pPartnerClassifyLoopIO != NULL) { smos_sprintf(sBuff,_T(", ClassifyIO[0x%p]"), m_pPartnerClassifyLoopIO) ; }
      else                                 { smos_sprintf(sBuff,_T("%.1024s"), _T(", ClassifyIO[NULL]") ) ; }
      smos_WriteBuffer(sBuff) ;  

    } // end NoArea_Loop pair check

  // parent
  if(pParent != NULL) { smos_sprintf(sBuff, _T("\n  %s       : Parent[0x%p]"), sTab, pParent) ; }
  else                { smos_sprintf(sBuff, _T("\n  %s       : Parent[NULL]"), sTab) ; }
  smos_WriteBuffer(sBuff) ;                                                                           

  // Sibling
  if(m_pSibling != NULL) { smos_sprintf(sBuff, _T(", Sibling[0x%p]"), m_pSibling) ; }
  else                   { smos_sprintf(sBuff, _T("%.1024s"), _T(", Sibling[NULL]") ) ; }
  smos_WriteBuffer(sBuff) ;                                                                           

  // Child
  if(m_pChild != NULL) { smos_sprintf(sBuff, _T(", Child[0x%p]"), m_pChild) ; }
  else                 { smos_sprintf(sBuff, _T("%s"), _T(", Child[NULL]") ) ; }
  smos_WriteBuffer(sBuff) ;                                                                           

  // Child and Sibling Recursion  (Child first is more human readable)
  if(m_pChild)   { m_pChild->Dump  (lGen+1, this) ; }
  if(m_pSibling) { m_pSibling->Dump(lGen,   pParent) ; }

  // Trailer
  if(lGen == 0)
    {
      smos_sprintf(sBuff, _T("%s"), _T("\n  End SmLooptree::Dump()") ) ;
      smos_WriteBuffer(sBuff);
    }

} // end SmLooptreeItem::Dump
    
