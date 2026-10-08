// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmVertexProps.cpp
* PURPOSE: Source file for implementation of SmEdgeProps methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmConfig.h>
#include <SmHealData.h>
#include <SmVertexProps.h>
#include <SmFaceProps.h>
#include <SmEdgeProps.h>
#include <SmVertex.h>
#include <SmEdge.h>
#include <SmTol.h>
#include <SmGraphicsOutput.h>
#ifdef SM_DEBUG_CODE
  #include <SmBrep.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: SmVertexProps Method implementations
***********************************************************************/

/*******************************************************************//**
 PURPOSE:
 NOTES:
***********************************************************************/
SmBoolean SmVertexProps::IsKindOf( SM_TYPE t ) const
{
  return ((SmVertexProps_TYPE == t) ? TRUE : SmObjProps::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: SmVertexProps Default Constructor

NOTES: 
***********************************************************************/
SmVertexProps::SmVertexProps(const SmContext * cpContext) 
{ 
  m_cpContext = cpContext ;
  ReSet() ; 

} // end SmVertexProps::SmVertexProps default constructor

/*******************************************************************//**
PURPOSE: SmVertexProps Constructor

NOTES: 
***********************************************************************/
SmVertexProps::SmVertexProps(SmVertex * pVertex, ULONG lVertexIndx, const SmContext * cpContext) 
{ 
  m_cpContext = cpContext ;
  ReSet() ; 
  m_pVertex     = pVertex ; 
  m_lVertexIndx = lVertexIndx ; 
  //if(m_pVertex) { m_pVertex->SetVertexProps(this) ; }

} // end SmVertexProps::SmVertexProps default constructor

/*******************************************************************//**
PURPOSE: SmVertexProps Copy Constructor

NOTES: 
***********************************************************************/
SmVertexProps::SmVertexProps(const SmVertexProps & crOther) 
{
  // locals
  m_cpContext = crOther.m_cpContext ; 

  // simple members
  m_pVertex               = crOther.m_pVertex ;
  m_lVertexIndx           = crOther.m_lVertexIndx ;
  m_sZoneTol3d            = crOther.m_sZoneTol3d ;       
  m_sOrigZoneTol3d        = crOther.m_sOrigZoneTol3d ;       
  m_pMaxGap3d_VtxEdge     = crOther.m_pMaxGap3d_VtxEdge ;            
  m_pMaxGap3d_VtxFace     = crOther.m_pMaxGap3d_VtxFace ; 
  m_bBadSmallZoneTol3d    = crOther.m_bBadSmallZoneTol3d ;            
  m_bBadLargeZoneTol3d    = crOther.m_bBadLargeZoneTol3d ; 
  m_bWarnLargeGap3d       = crOther.m_bWarnLargeGap3d ; 
  m_bBadCoincidentVertex  = crOther.m_bBadCoincidentVertex ;

} // end SmVertexProps::SmVertexProps copy constructor

/*******************************************************************//**
PURPOSE: SmVertexProps assignment operator

NOTES: 
***********************************************************************/
SmVertexProps & SmVertexProps::operator= (const SmVertexProps &crOther)
{
  // no work - same object
  if(this == &crOther)
    { return *this ; }

  // simple members
  m_pVertex               = crOther.m_pVertex ;
  m_lVertexIndx           = crOther.m_lVertexIndx ;
                          
  m_sZoneTol3d            = crOther.m_sZoneTol3d ;       
  m_sOrigZoneTol3d        = crOther.m_sOrigZoneTol3d ;       
  m_pMaxGap3d_VtxEdge     = crOther.m_pMaxGap3d_VtxEdge ;            
  m_pMaxGap3d_VtxFace     = crOther.m_pMaxGap3d_VtxFace ; 
  m_bBadSmallZoneTol3d    = crOther.m_bBadSmallZoneTol3d ;            
  m_bBadLargeZoneTol3d    = crOther.m_bBadLargeZoneTol3d ; 
  m_bWarnLargeGap3d       = crOther.m_bWarnLargeGap3d ; 
  m_bBadCoincidentVertex  = crOther.m_bBadCoincidentVertex ;

  // all done
  return(*this) ;
  
} // end SmVertexProps::operator= assignment operator

/*******************************************************************//**
PURPOSE: SmVertexProps equality operator

NOTES: 
***********************************************************************/
SmBoolean SmVertexProps::operator== (const SmVertexProps &crOther) const
{
  // locals
  SmBoolean bRtn = TRUE ; 

  // no work - same object
  if(this == &crOther)
    { return bRtn ; }

  // simple members
  bRtn &= m_pVertex               == crOther.m_pVertex ;  
  bRtn &= m_lVertexIndx           == crOther.m_lVertexIndx ;
  bRtn &= m_sZoneTol3d            == crOther.m_sZoneTol3d ;       
  bRtn &= m_sOrigZoneTol3d        == crOther.m_sOrigZoneTol3d ;       
  bRtn &= m_pMaxGap3d_VtxEdge     == crOther.m_pMaxGap3d_VtxEdge ;            
  bRtn &= m_pMaxGap3d_VtxFace     == crOther.m_pMaxGap3d_VtxFace ; 
  bRtn &= m_bBadSmallZoneTol3d    == crOther.m_bBadSmallZoneTol3d ;            
  bRtn &= m_bBadLargeZoneTol3d    == crOther.m_bBadLargeZoneTol3d ; 
  bRtn &= m_bWarnLargeGap3d       == crOther.m_bWarnLargeGap3d ; 
  bRtn &= m_bBadCoincidentVertex  == crOther.m_bBadCoincidentVertex ;

  // all done
  return(bRtn) ;
  
} // end SmVertexProps::operator== equality operator

/*******************************************************************//**
PURPOSE: initialize SmVertexProps all member values

NOTES: This method resets newly allocated SmVertexProps and previously
       used SmVertexProps objects back to the same uninitialized state.

       But leaves the m_cpContext unchanged. 
       (Simplifies reusing an SmVertexProps object)
***********************************************************************/
void SmVertexProps::ReSet()
{
  // nothing to do for Context

  // simple members
  //if(m_pVertex) { m_pVertex->SetVertexProps(NULL) ; }
  m_pVertex               = NULL ;
  m_lVertexIndx           = SM_UNDEF_ULONG ;
                          
  m_sZoneTol3d            = SM_UNDEF_DOUBLE ;
  m_sOrigZoneTol3d        = SM_UNDEF_DOUBLE ;
  m_pMaxGap3d_VtxEdge     = NULL ;
  m_pMaxGap3d_VtxFace     = NULL ;
  m_bBadSmallZoneTol3d    = UNSURE ; 
  m_bBadLargeZoneTol3d    = UNSURE ;
  m_bWarnLargeGap3d       = UNSURE ;
  m_bBadCoincidentVertex  = UNSURE ;
  
} // end SmVertexProps::ReSet

/*******************************************************************//**
PURPOSE: set all SmVertexProps member values

NOTES: This method sets most of the SmVertexProps member values.
         Sets   : m_pVertex       
                  m_lVertexIndx    
                  m_sZoneTol3d     
                  m_sOrigZoneTol3d 
                  m_pMaxGap3d_VtxEdge 
                  m_pMaxGap3d_VtxFace 
                  m_bBadSmallZoneTol3d   _ cleared in SmHealData::Fix_TolSizes
                  m_bBadLargeZoneTol3d   _ cleared in SmHealData::Fix_TolSizes
                  m_bWarnLargeGap3d      _ cleared in SmHealData::Fix_TolSizes

         Not Set: m_bBadCoincidentVertex - set in SmHealData::Cache_CoinVertices()
                                           cleared in SmHealData::Fix_CoinVertices()
***********************************************************************/
SmStatus SmVertexProps::SetProps
  (SmVertex        * cpVertex,                     // in : tgt Vertex
   ULONG             lVertexIndx,                  // in : associated Indx of cpVertex in managing SmHealData::m_sTgtVertices list
   SmTArray<ULONG> * pOptVertEdgeGap3d_Histogram,  // i/o: accumulating Vertex/Edge Gap3d histogram, NULL to ignore
   SmTArray<ULONG> * pOptVertFaceGap3d_Histogram,  // i/o: accumulating Vertex/Face Gap3d histogram, NULL to ignore
   ULONG             lOptLabel)                    // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
                                                   // sets: VertexProps: 
                                                   //        m_pVertex            m_lVertexIndx
                                                   //        m_sOrigZoneTol3d     m_sZoneTol3d
                                                   //        m_pMaxGap3d_VtxEdge  m_bBadSmallZoneTol3d  m_bWarnLargeGap3d
                                                   //        m_pMaxGap3d_VtxFace  m_bBadLargeZoneTol3d  
                                                   // note: m_bBadCoincidentVertex is not set in this call.
                                                   //       It's set elsewhere in SmHealData::Get_CoinVerts().
{                   
  // init all values except m_cpContext
  ReSet() ;

  // nothing to do here for m_cpContext

  // set property member values
  m_pVertex     = cpVertex ;         // forward ptr: VertexProps->Vertex
  m_lVertexIndx = lVertexIndx ;
  //cpVertex->SetVertexProps(this) ;  // back    ptr: Vertex->VertexProps
  m_sZoneTol3d = SmTol::GetZoneTol3d(m_pVertex) ;
  if(m_sOrigZoneTol3d == SM_UNDEF_DOUBLE)
    { m_sOrigZoneTol3d = m_sZoneTol3d ; }

  // Get MaxGap3ds from Vertex->Vertexuses Gap3d
  ULONG ii ; 
  SmTArray<SmVertexuse *> sVertexuses ;
  cpVertex->GetVertexuses(sVertexuses) ;

  // for every Vertex->Vertexuse - Save MaxGap3d seen
  for(ii=0;ii<sVertexuses.GetSize();ii++)
    {
      SmVertexuse     * pVertexuse     = sVertexuses[ii] ;

      // Vertex/Edge Gap - Loop and Shell Vertices don't have Vertex/Edge gaps
      SmVertexEdgeGap * pVertexEdgeGap = pVertexuse->GetVertexEdgeGap() ;
      if(pVertexEdgeGap)
        {
          // Max Vertex/Edge Gap
          if(   m_pMaxGap3d_VtxEdge == NULL
             || m_pMaxGap3d_VtxEdge->GetLength() < pVertexEdgeGap->GetLength())
            { m_pMaxGap3d_VtxEdge = pVertexEdgeGap ; }

          if(pOptVertEdgeGap3d_Histogram)
            { (*pOptVertEdgeGap3d_Histogram)[SM_HISTOGRAM_GAPSIZE_INDEX(pVertexEdgeGap->GetLength())]++ ; }
         } // end pVertexEdgeGap existence check

      // Vertex/Face Gap - Shell Vertices don't have Vertex/Face gaps
      SmVertexFaceGap * pVertexFaceGap = pVertexuse->GetVertexFaceGap() ;
      if(pVertexFaceGap)
        {
          // Max Vertex/Face Gap
          if(   m_pMaxGap3d_VtxFace == NULL
             || m_pMaxGap3d_VtxFace->GetLength() < pVertexFaceGap->GetLength())
            { m_pMaxGap3d_VtxFace = pVertexFaceGap ; }

          if(pOptVertFaceGap3d_Histogram)
            { (*pOptVertFaceGap3d_Histogram)[SM_HISTOGRAM_GAPSIZE_INDEX(pVertexFaceGap->GetLength())]++ ; }
        } // end pVertexFaceGap existence check

    } // end iter all Vertexuses looking for MaxGap3d values

  // locals for Problem gap checks
  SmZoneTol3d sDefZoneTol3d              = SmTol::GetZoneTol3d(m_cpContext) ;
  double      dVertexEdge_MaxGapLength3d = m_pMaxGap3d_VtxEdge ? m_pMaxGap3d_VtxEdge->GetLength() : 0.0 ;
  double      dVertexFace_MaxGapLength3d = m_pMaxGap3d_VtxFace ? m_pMaxGap3d_VtxFace->GetLength() : 0.0 ;
  SmZoneTol3d sCalcZoneTol3d             = SmTol::CalcVertexZoneTol3d(dVertexEdge_MaxGapLength3d,
                                                                      dVertexFace_MaxGapLength3d,
                                                                      NULL, // EdgeTol not yet set, so unreliable for this test
                                                                      m_cpContext) ;
  // ZoneTol3d for current Gap3d sizes    
  m_bBadSmallZoneTol3d = m_sOrigZoneTol3d <= sCalcZoneTol3d - SM_EFF_ZERO ; 
  m_bBadLargeZoneTol3d = m_sOrigZoneTol3d >= sCalcZoneTol3d + SM_EFF_ZERO ;

  // large gaps
  m_bWarnLargeGap3d = (   dVertexEdge_MaxGapLength3d >= sDefZoneTol3d
                       || dVertexFace_MaxGapLength3d >= sDefZoneTol3d) ;

  // Coin Vertices
  // m_bBadCoincidentVertex is not set here.  It's set in SmHealData::Cache_CoinVertices()

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE ; 
  if(bDebugMe2)
    {
      this->Dump(lOptLabel) ; 
      SmBrep * pBrep = m_pVertex ? m_pVertex->GetBrep() : NULL ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      this->Draw(TRUE) ; sm_GraphicsLoop() ; // sHighlightColor,         def:[ 1, 0, 0]
                                             // sBadSmallZoneTol3dColor, def:[ 0, 1, 1]
                                             // sBadLargeZoneTol3dColor, def:[ 1, 0, 1]
                                             // sWarnLargeGap3dColor,    def:[ 1,.5,.1]
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(lOptLabel);
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmVertexProps::SetProps

/*******************************************************************//**
PURPOSE: Return TRUE if member values have been set

NOTES: Member values are set with a call to SetProps() 
***********************************************************************/
SmBoolean SmVertexProps::IsSet() const
{
  SmBoolean bIsSet = TRUE ; 

  // simple members
  bIsSet &= m_pVertex              != NULL ;    
  bIsSet &= m_lVertexIndx          != SM_UNDEF_ULONG ;
  bIsSet &= m_sZoneTol3d           != SM_UNDEF_DOUBLE ;
  bIsSet &= m_sOrigZoneTol3d       != SM_UNDEF_DOUBLE ;
  bIsSet &= m_pMaxGap3d_VtxEdge    != NULL ;
  bIsSet &= m_pMaxGap3d_VtxFace    != NULL ;
  bIsSet &= m_bBadSmallZoneTol3d   != UNSURE ; 
  bIsSet &= m_bBadLargeZoneTol3d   != UNSURE ;
  bIsSet &= m_bWarnLargeGap3d      != UNSURE ;

  // m_bBadCoincidentVertex 
  //   set in SmHealData::Cache_CoinVertices() not in SmVertexProps::SetProps()
  //   omit check from IsSet():  bIsSet &= m_bBadCoincidentVertex != UNSURE ; 
  
  return(bIsSet) ;
                       
} // end SmVertexProps::IsSet

/*******************************************************************//**
PURPOSE: Return TRUE if obj has any problems

NOTES: Member values are set with a call to SetProps() 
***********************************************************************/
SmBoolean SmVertexProps::HasProblems() const
{
  // return value
  SmBoolean bHasProblems = FALSE ; 

  // check problem list
  bHasProblems |= m_bBadSmallZoneTol3d   == TRUE ;
  bHasProblems |= m_bBadLargeZoneTol3d   == TRUE ;
  bHasProblems |= m_bWarnLargeGap3d      == TRUE ;
  bHasProblems |= m_bBadCoincidentVertex == TRUE ; 

  // all done
  return(bHasProblems) ;
                       
} // end SmVertexProps::HasProblems

/****************************************************************
PURPOSE: add graphics for problem VertexVertexs

NOTES: i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
       NULL to ignore. default:[NULL]
****************************************************************/
SmDisplayList * SmVertexProps::Draw
 (SmBoolean bDrawAllCases,              // in : TRUE = output graphics for all Vertexs (with or without probs)
                                        //      default:[FALSE] = output graphics only for Vertexs with probs 
  SmVector3d sHighlightColor,           // in : def:[ 1, 0, 0]: If VertexHasProblems Draw VertexUV for highlight
  SmVector3d sBadSmallZoneTol3dColor,   // in : def:[ 0, 1, 1]: Vertices with too small ZoneTol3d Highlight Color    
  SmVector3d sBadLargeZoneTol3dColor,   // in : def:[ 1, 0, 1]: Vertices with too large ZoneTol3d Highlight Color
  SmVector3d sWarnLargeGap3dColor,      // in : def:[ 1,.5,.1]: Vertices with Gaps larger than DefZoneTol3d Highlight Color
  SmGfxArraySet * pOptGfxSet            // in : When given, output GfxVertexArrays not GL calls. 
 ) const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE
  // locals: global display parameters
  // const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters();

  // start new displayList (unless one is already open)
  SmVector3d sFVertexPropsColor(0,1,0) ; 
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(sFVertexPropsColor, NULL, NULL, FALSE, pOptGfxSet) ;

  // check need to draw
  if(   IsSet()
     && (   bDrawAllCases
         || HasProblems()))
    {
      SmPoint3d sVertexPoint = m_pVertex->GetPoint() ;

      // highlight - when asked
      if(bDrawAllCases) { smgfx_SetLook(2,4, sHighlightColor) ; 
                          sVertexPoint.Draw() ; 
                        }

      // Bad ZoneTol3d - too small
      if(m_bBadSmallZoneTol3d == TRUE) { smgfx_SetLook(4,6, sBadSmallZoneTol3dColor) ; 
                                         sVertexPoint.Draw() ; 
                                         if(m_pMaxGap3d_VtxEdge) m_pMaxGap3d_VtxEdge->Draw() ;
                                         if(m_pMaxGap3d_VtxFace) m_pMaxGap3d_VtxFace->Draw() ;
                                       }
      // Bad ZoneTol3d - too large
      if(m_bBadLargeZoneTol3d == TRUE) { smgfx_SetLook(5,7, sBadLargeZoneTol3dColor) ; 
                                         sVertexPoint.Draw() ; 
                                         if(m_pMaxGap3d_VtxEdge) m_pMaxGap3d_VtxEdge->Draw() ;
                                         if(m_pMaxGap3d_VtxFace) m_pMaxGap3d_VtxFace->Draw() ; 
                                       }

      // Warn Gap3d - too large
      if(m_bWarnLargeGap3d == TRUE)    { smgfx_SetLook(6,8, sWarnLargeGap3dColor) ; 
                                         sVertexPoint.Draw() ; 
                                         if(m_pMaxGap3d_VtxEdge) m_pMaxGap3d_VtxEdge->Draw() ;
                                         if(m_pMaxGap3d_VtxFace) m_pMaxGap3d_VtxFace->Draw() ; 
                                       }

      // Bad CoincidentVertex
      if(m_bBadCoincidentVertex == TRUE) { smgfx_SetLook(9,10, sWarnLargeGap3dColor) ; 
                                           sVertexPoint.Draw() ; 
                                         }
    } // end need to draw check

  // end new displayList
  SmVector3d sMyColor = smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF6(bDrawAllCases, sHighlightColor, sBadSmallZoneTol3dColor, sBadLargeZoneTol3dColor, sWarnLargeGap3dColor, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVertexProps::Draw

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmVertexProps::Dump() const
{
  // pass the call along
  Dump(SM_UNDEF_ULONG) ;

}// end SmVertexProps::Dump()

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmVertexProps::Dump
  (ULONG lLabel)  // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"),_T("\nBegin SmVertexProps Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nBegin SmVertexProps Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

  // base class
  SmObjProps::Dump() ;

  // no work - No contents - VertexProps_BeforeVertexSplit() has not yet been run 
  if(IsSet() == FALSE) 
    {
      smos_sprintf(sBuff,        _T("\n SmVertexProps   : [0x%p] - UnSet"), this) ;
      smos_sprintf(sBuffForFile, _T("\n SmVertexProps   : [%s] - UnSet"), _T("notNULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile);
      smos_WriteBuffer( _T("\nEnd SmVertexProps Dump ")) ;

      // all done
      return ;

    } // end BeforeVertexSplit Props not set branch

  // arrive here when SetProps() has run and IsSet() == TRUE

  // Header = PropsPtr, VertexPtr, VertexPoint, and XSectTol3d val.
  smos_sprintf(sBuff,        _T("\n SmVertexProps   : [0x%p] for Vertex:[0x%p], VertexIndx:[%4lu], ZoneTol3d:[%5.7lf], OrigZoneTol3d:[%5.7lf], Point:"),
             this,
             m_pVertex,
             m_lVertexIndx,
             m_sZoneTol3d.val,
             m_sOrigZoneTol3d.val);

  smos_sprintf(sBuffForFile, _T("\n SmVertexProps   :[%s] for Vertex:[%s], VertexIndx:[%4lu], ZoneTol3d:[%5.7lf], OrigZoneTol3d:[%5.7lf], Point:"),
             _T("notNULL"),
             m_pVertex  ? _T("notNULL") : _T("NULL"),
             m_lVertexIndx,
             m_sZoneTol3d.val,
             m_sOrigZoneTol3d.val) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;
  m_pVertex->GetPoint().Dump(); 

  // Properties: Max Vertex_Edge Gap3d
  smos_sprintf(sBuff, _T("\n   Properties: Max Vertex_Edge Gap3d:[%s%lf] %s"), 
                    m_pMaxGap3d_VtxEdge ? _T("") : _T("none="),
                    m_pMaxGap3d_VtxEdge ? m_pMaxGap3d_VtxEdge->GetLength() : 0,
                    m_pMaxGap3d_VtxEdge ? _T("") : m_pVertex == NULL 
                                                 ? _T("Err: Null m_pVertex Ptr")
                                                 : (  m_pVertex->IsShellVertex() ? _T("for ShellVertex")
                                                    : m_pVertex->IsLoopVertex()  ? _T("for LoopVertex")
                                                    : _T(""))) ; 
  smos_WriteBuffer(sBuff) ;              

  // Properties: Max Vertex_Face Gap3d
  smos_sprintf(sBuff, _T("\n   Properties: Max Vertex_Face Gap3d:[%s%lf] %s"), 
                    m_pMaxGap3d_VtxFace ? _T("") : _T("none="),
                    m_pMaxGap3d_VtxFace ? m_pMaxGap3d_VtxFace->GetLength() : 0,
                    m_pMaxGap3d_VtxFace ? _T("") : (  m_pVertex->IsLoopVertex()  ? _T("for LoopVertex")
                                                 : _T(""))) ; 
  smos_WriteBuffer(sBuff) ;

  // VertexProblems
  if(HasProblems() == FALSE)
    { 
      smos_WriteBuffer(_T("\n    Problems : [None]")) ; 
    } // end no problems branch

  else // Has Problems branch
    {
      // locals
      SmZoneTol3d sDefZoneTol3d  = SmTol::GetZoneTol3d() ;
      SmZoneTol3d sCalcZoneTol3d = SmTol::CalcVertexZoneTol3d(m_pMaxGap3d_VtxEdge ? m_pMaxGap3d_VtxEdge->GetLength() : 0.0,
                                                              m_pMaxGap3d_VtxFace ? m_pMaxGap3d_VtxFace->GetLength() : 0.0) ;

      // m_bBadSmallZoneTol3d             
      smos_sprintf(sBuff, _T("\n    Problems : BadZoneTol3d Too Small :[%s]"),
                   (m_bBadSmallZoneTol3d == TRUE)  ? _T("TRUE")
                 : (m_bBadSmallZoneTol3d == FALSE) ? _T("FALSE")
                 :                                   _T("UNSURE")) ;
      smos_WriteBuffer(sBuff) ;
      if(m_bBadSmallZoneTol3d == TRUE)
        { smos_sprintf(sBuff,_T("  Bad : Increase OrigZoneTol3d:[%lf] to SmTol::CalcVertexZoneTol3d:[%lf]"),
                           m_sOrigZoneTol3d.val,
                           sCalcZoneTol3d.val) ; 
          smos_WriteBuffer(sBuff) ;
        }
      else if(m_bBadSmallZoneTol3d == FALSE && m_sOrigZoneTol3d < m_sZoneTol3d)
        {
          smos_sprintf(sBuff,_T(" Okay: OrigZoneTol3d:[%lf] increased to ZoneTol3d:[%lf]"),
                           m_sOrigZoneTol3d.val,
                           m_sZoneTol3d.val) ; 
          smos_WriteBuffer(sBuff) ;
        }
      else
        { smos_WriteBuffer(_T(" Okay")) ; } 
      
      // m_bBadLargeZoneTol3d             
      smos_sprintf(sBuff, _T("\n             : BadZoneTol3d Too Large :[%s]"),
                   (m_bBadLargeZoneTol3d == TRUE)  ? _T("TRUE")
                 : (m_bBadLargeZoneTol3d == FALSE) ? _T("FALSE")
                 :                                   _T("UNSURE")) ;
      smos_WriteBuffer(sBuff) ;
      if(m_bBadLargeZoneTol3d == TRUE)
        { smos_sprintf(sBuff,_T("  Bad : Decrease OrigZoneTol3d:[%lf] to SmTol::CalcVertexZoneTol3d:[%lf]"),
                           m_sOrigZoneTol3d.val,
                           sCalcZoneTol3d.val) ; 
          smos_WriteBuffer(sBuff) ;
        }
      else if(m_bBadSmallZoneTol3d == FALSE && m_sOrigZoneTol3d > m_sZoneTol3d)
        {
          smos_sprintf(sBuff,_T(" Okay: OrigZoneTol3d:[%lf] decreased to ZoneTol3d:[%lf]"),
                           m_sOrigZoneTol3d.val,
                           m_sZoneTol3d.val) ; 
          smos_WriteBuffer(sBuff) ;
        }
      else
        { smos_WriteBuffer(_T(" Okay")) ; } 

      // m_bWarnLargeGap3d
      smos_sprintf(sBuff, _T("\n             : Warn LargeGap3d        :[%s]"),
                   (m_bWarnLargeGap3d == TRUE)  ? _T("TRUE")
                 : (m_bWarnLargeGap3d == FALSE) ? _T("FALSE")
                 :                                _T("UNSURE")) ;
      smos_WriteBuffer(sBuff) ;
      if(m_bWarnLargeGap3d == TRUE)
        { smos_sprintf(sBuff,_T("  Warn: Max Vert/Edge:[%lf] or Vert/Face:[%lf] Gap3d larger than DefZoneTol3d:[%lf]"),
                           m_pMaxGap3d_VtxEdge ? m_pMaxGap3d_VtxEdge->GetLength() : 0.0,
                           m_pMaxGap3d_VtxFace ? m_pMaxGap3d_VtxFace->GetLength() : 0.0,
                           sDefZoneTol3d.val) ; 
          smos_WriteBuffer(sBuff) ;
        }
      else
        { smos_WriteBuffer(_T(" Okay")) ; }       
      
      // m_bBadCoincidentVertex             
      smos_sprintf(sBuff, _T("\n             : Bad Coincident Vertex  :[%s]"),
                   (m_bBadLargeZoneTol3d == TRUE)  ? _T("TRUE")
                 : (m_bBadLargeZoneTol3d == FALSE) ? _T("FALSE")
                 :                                   _T("UNSURE")) ;
      smos_WriteBuffer(sBuff) ;

    } // end Vertex has Problems branch

  // all done
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"),_T("\nEnd   SmVertexProps Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd   SmVertexProps Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

} // end SmVertexProps::Dump
