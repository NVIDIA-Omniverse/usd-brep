// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmFaceGrid.cpp
* PURPOSE: Source file for implementation of SmFaceGrid methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmFaceGrid.h>
#include <SmCrvOnSurf.h>

/*******************************************************************//**
PURPOSE: SmFaceGrid Method implementations
***********************************************************************/

/*******************************************************************//**
PURPOSE: Helper function for LoadGrid

NOTES: Was made a separate function so it can call itself recursively.
       Assumes that CurveClass intervals are continuous and span the surface.
       Some tricky cases where CurveClass Intervals are collapsed can cause the
       Intervals to not span the surface. Since this is for graphics output, it is
       assumed we can have tolerance sized errors in favor of performance.

Method: Given a parameter, CurveClassification and an Index, determine if the
        Parameter is in CurveClassification[Index].
        If
            Param <  Ivl Min, return False
            Param =  Ivl Min, return (Classifies to Object == TRUE)
            Param in Ivl    , return (Classifies to Object == TRUE)
            Param =  Ivl Max, return (Classifies to Object == TRUE)
            Param >  Ivl Max, increment Index, call recursively
                                                                         
***********************************************************************/
SmBoolean sm_IsCurvePointInFace(double dParam, SmCurveClassification & rCurveClass, ULONG & rlIndex)
{
    // If rlIndex has grown to the number of intervals, then abort this process
    if (rlIndex == rCurveClass.GetSize())
    {
        // JGU: Curve classifications can collapse, causing them to have a shorter span than expected. Might show up here.
        //      I don't think it will be a problem if this happens, but added warning for review.
        SM_DBG_WARN(_T("Unexpected case. Review. CurveClassification interval doesn't span surface as expected."));
        return FALSE;
    }

    // Get local CurveInterval and Interval
    SmCurveInterval& rCrvIvl = rCurveClass[rlIndex];
    const SmExtent1d& rIvl = rCrvIvl.GetInterval();

    if (dParam < rIvl.GetMin()) // Param lower than interval
    {
        // JGU: Curve classifications can collapse, causing them to have a shorter span than expected. Might show up here.
        //      I don't think it will be a problem if this happens, but added warning for review.
        SM_DBG_WARN(_T("Unexpected case. Review. CurveClassification interval doesn't span surface as expected."));
        return FALSE;
    }
    else if (dParam == rIvl.GetMin()) //
    {
        if (rCrvIvl.m_vStart.GetObject())
        {
            return TRUE;
        }
    }
    else if (dParam < rIvl.GetMax())
    {
        if (rCrvIvl.m_vMid.GetObject())
        {
            return TRUE;
        }
    }
    else if (dParam == rIvl.GetMax())
    {
        if (rCrvIvl.m_vEnd.GetObject())
        {
            ++rlIndex;
            return TRUE;
        }
    }
    else
    {
        // Increment the index and call this function again.
        ++rlIndex;
        return sm_IsCurvePointInFace(dParam, rCurveClass, rlIndex);
    }

    return FALSE;
}

/*******************************************************************//**
PURPOSE: Eval and classify points in a regular UV Grid

NOTES: 
***********************************************************************/
// Load the Grid
void SmFaceGrid::LoadGrid()
{
    // locals
    ULONG ii, jj, lIndex;
    SmPoint2d sUV;
    SmPoint3d sOrigin(0, 0, 0);
    const SmContext& crContext = *m_cpFace->GetContext();
    SmZoneTol3d sZoneTol3d = SmTol::GetZoneTol3d();
    SmExtent1d sIvl(m_sUVDomain.GetUInterval());
    SmLine* pIsoCurve = new (m_cpFace->GetContext()) SmLine(2);
    SmSurface* pSupportSurf = NULL;
    SmCurveClassification sCurveClass;
    SmVector3d sVec(1.0, 0., 0.);

    m_cpSurface->Copy(crContext, pSupportSurf);
    SmCrvOnSurf* pCurveOnSurf = new (crContext) SmCrvOnSurf(*pIsoCurve, *pSupportSurf, &m_sUVDomain, 3);

    // Delete IsoCurve and SupportSurf when done
    SmObjDelete sCleanCurv(pCurveOnSurf);

    for (ii = 0; ii < m_lRowCnt * m_lColCnt; ++ii)
    { m_sGrid[ii] = SmFaceEval(); }

    // for every row make an IsoCurve to classify against the face
    for (ii = 0; ii < m_lRowCnt; ++ii)
    {
        // Loop locals
        lIndex = 0;

        // Set UV points for IsoCurve and this UV grid point
        sOrigin.y = sUV.y = m_sUVDomain.EvaluateV((double)ii / (double)(m_lRowCnt - 1));

        // Update curve geometry and clear cache
        pCurveOnSurf->Notify(SM_NO_PRE_EDIT, pCurveOnSurf, NULL, NULL);
        pIsoCurve->SetCanonical(sOrigin, sVec, 1.0, sIvl);
        pCurveOnSurf->Notify(SM_NO_POST_EDIT, pCurveOnSurf, NULL, NULL);

        // Classify curve against m_cpFace
        SmCurveInterval aData[20];
        sCurveClass.ReSet(pCurveOnSurf, &sIvl, pIsoCurve, sZoneTol3d, 20, aData, FALSE, TRUE, pSupportSurf);
        m_cpFace->CurveOnClassify(TRUE, sCurveClass, FALSE);

        for ( jj = 0; jj < m_lColCnt; ++jj )
        {
            sUV.x = m_sUVDomain.EvaluateU((double)jj / (double)(m_lColCnt - 1));

            // begin scope Eval Surface,
            //   note:SmSurfaceEval is a lazy evaluator - we just set the eval params here.
            //        the evaluation happens later whenever anyone looks for one of the SurfaceEval values.
            {
              SmSurfaceEval* pSurfaceEval = GetSurfaceEval(jj, ii);
              pSurfaceEval->SetSurface(m_cpSurface);
              pSurfaceEval->SetNonZeroTangents(TRUE);
              pSurfaceEval->SetUV(sUV);
              pSurfaceEval->SetUFromLeft(TRUE);
              pSurfaceEval->SetVFromLeft(TRUE);
            } // end scope Eval surface

            // Determine if point on curve is in the Face
            if (sm_IsCurvePointInFace(sUV.x, sCurveClass, lIndex))
            { SetIsInFace(jj, ii, TRUE); }
        }
    }
} // end SmFaceGrid::LoadGrid()

/*******************************************************************//**
PURPOSE: Draw FaceGrid data, surface and face

NOTES: 
***********************************************************************/
SmDisplayList* SmFaceGrid::Draw(SmGfxArraySet * pOptGfxSet) 
{
    SmDisplayList* pRtn = NULL;

#ifdef SM_GFX_CODE
    // Draw the surface with UV
    smgfx_SetLook(1, 2, .1, .1, 1.);
    m_cpSurface->DrawUV(8, 8, FALSE, NULL, FALSE, pOptGfxSet);

    // Draw the Face
    smgfx_SetLook(1, 2, 0., 0., 0.);
    m_cpFace->Draw(pOptGfxSet);

    // Draw the points that are in the face
    smgfx_SetLook(1, 2, 1., .1, .1);
    for (ULONG ii = 0; ii < m_lColCnt; ++ii)
        for (ULONG jj = 0; jj < m_lRowCnt; ++jj)
            IsInFace(ii, jj) ? m_cpSurface->DrawAt(GetUV(ii, jj), 0, pOptGfxSet) : NULL;

#else
  SM_REF1(pOptGfxSet);
#endif // end SM_GFX_CODE

    return pRtn;
    
} // end SmFaceGrid::Draw()

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES: Meta data
***********************************************************************/
void SmFaceGrid::DumpHeader() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // meta data

  // This FaceGrid pointer
  smos_sprintf(sBuff,        _T("\n FaceGrid     :[0x%p]"), this) ;
  smos_sprintf(sBuffForFile, _T("\n FaceGrid     :[0x%p]"), _T("notNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;      

  // TgtFace
  smos_sprintf(sBuff,        _T("\n TgtFace      :[0x%p]"), this) ;
  smos_sprintf(sBuffForFile, _T("\n TgtFace      :[0x%p]"), _T("notNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;      

  // Domain
  smos_sprintf(sBuff,        _T("%s"), _T("\n Domain       :")) ; 
  smos_WriteBuffer(sBuff) ;
  m_sUVDomain.Dump() ;

  // Size[row, col]
  smos_sprintf(sBuff,        _T("\n size[row,col]:[%4lu, %4lu]"), m_lRowCnt, m_lColCnt) ;
  smos_WriteBuffer(sBuff) ;

} // end SmFaceGrid::DumpHeader

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmFaceGrid::Dump
  (ULONG lLabel)  // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_ULONG]
{
  ULONG ii, jj ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  SmFaceEval * pFaceEval = GetDataArray() ;

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\nBegin SmFaceGrid Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nBegin SmFaceGrid Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

  // meta data
  DumpHeader() ;

  // Data

  // for every row
  for(ii=0;ii<m_lRowCnt;ii++)
    {
      smos_WriteBuffer(_T("\n")) ;

      // for every col
      for(jj=0;jj<m_lColCnt;jj++,pFaceEval++)
        {
          SmSurfaceEval         * pSurfaceEval         = pFaceEval->GetSurfaceEval() ;
          SmBoolean               bIsInFace            = pFaceEval->IsInFace();

          smos_sprintf(sBuff, _T("\n [%4lu, %4lu] : "), ii, jj) ;
          smos_WriteBuffer(sBuff) ;

          // SurfaceEval
          pSurfaceEval->Dump() ;

          // point classification to Face
          ( bIsInFace ) ? smos_WriteBuffer(_T("\n in/out Face:[In ]"))
                        : smos_WriteBuffer(_T("\n in/out Face:[Out]"));

        } // end iter jj, every col
    } // end iter ii, every row

  // all done
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\nEnd   SmFaceGrid Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd   SmFaceGrid Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);
  
} // end SmFaceGrid::Dump

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmFaceGrid::DumpFaceMap
  (ULONG lLabel)  // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_ULONG]
  const
{
  ULONG ii, jj ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  SmFaceEval * pFaceEval = GetDataArray() ;

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\nBegin SmFaceGrid FaceMap Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nBegin SmFaceGrid FaceMap Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

  // meta data
  DumpHeader() ;

  // Data

  // for every row
  for(ii=m_lRowCnt;ii>0;ii--)
    {
      smos_WriteBuffer(_T("\n  ")) ;

      // for every col
      for(jj=0;jj<m_lColCnt;jj++,pFaceEval++)
        {
          SmBoolean bIsInFace = pFaceEval->IsInFace();

          // point classification to Face
          (bIsInFace) ? SM_STRCAT(sBuff,_T("1"))
                      : SM_STRCAT(sBuff,_T("0")) ;

        } // end iter jj, every col

      // write the accumulated row characters
      smos_WriteBuffer(sBuff);

    } // end iter ii, every row

  // all done
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\n\nEnd   SmFaceGrid FaceMap Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\n\nEnd   SmFaceGrid FaceMap Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);
  
} // end SmFaceGrid::DumpFaceMap
