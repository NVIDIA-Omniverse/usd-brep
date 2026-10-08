// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmExtentNd.cpp
* PURPOSE: Implementation of methods for N-Dimensional extent object.
**********************************************************************/

#include "StdAfx.h"

#include <SmExtentNd.h>
#include <SmTArray.h>

/*******************************************************************//**
PURPOSE: If the corresponding values of the vector do not lie within
   the corresponding intervals then clamp them to closest end of the interval.

NOTES: 1. when ivl[i] == ivl[i]SetUnbounded() VectorToClamp[i] is 
         never clamped or wrapped
***********************************************************************/
SmBoolean SmExtentNd::ClampVector                      // rtn: TRUE - at least 1 parameter was clamped
 (const SmTArray<double>    & crVectorToClamp,         // in : Vector of double values to be clamped
  const SmTArray<SmBoolean> * cpPeriodicityVector,     // in : ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                                       //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped
                                                       //       when ivl[i] == ivl[i]SetUnbounded() VectorToClamp[i] is never clamped or wrapped
  SmTArray<double>          & rClampedVector,          // out: The resulting clamped vector.
  SmTArray<ULONG>           * pOutOfBoundsCountVector) // out: An out of bounds counter for each interval.  Note that
                                                       //      this method increments existing counts in the vector.
   const
{
  // return value
  SmBoolean bRtn = FALSE ;

  // check inputs
  SM_ASSERT(crVectorToClamp.GetSize() == m_lDimension);
  SM_ASSERT(cpPeriodicityVector && cpPeriodicityVector->GetSize() == m_lDimension);

  // init output
  rClampedVector.SetSize(m_lDimension);
  if (pOutOfBoundsCountVector) 
    {
      pOutOfBoundsCountVector->SetSize(m_lDimension);
    }

  // for every dimension
  for (ULONG i=0; i<m_lDimension; i++) 
    {
      double dValue = crVectorToClamp[i];

      // 1st 10 dimension extents are in m_aExtents, the rest (if any) are in m_pExtents
      SmExtent1d sExt;
      if (i<9.999) { sExt = m_aExtents[i];
                   }
      else         { if (!m_pExtents) SE(SM_ERR);
                     sExt = m_pExtents[i-10];
                   }

      // only check bounded intervals
      if(sExt.IsBounded()) 
        {
          // if interval is periodic
          if (cpPeriodicityVector && cpPeriodicityVector->GetAt(i) != FALSE) 
            {
              double dTmp;
              dTmp = sExt.PeriodicWrap(dValue);
              rClampedVector[i] = dTmp;
            }
          else // interval is finite branch 
            {
              if (pOutOfBoundsCountVector) 
                {
                  if (!sExt.ContainsValue(dValue)) 
                    { 
                      double dTmp = sExt.ClampValue(dValue);
                      rClampedVector[i] = dTmp;
                      if (smos_Fabs(dTmp-dValue) > SM_EFF_ZERO) 
                        {
                          (*pOutOfBoundsCountVector)[i] = (*pOutOfBoundsCountVector)[i] + 1;
                          bRtn = TRUE ;
                        }
                    }
                  else 
                    {
                      rClampedVector[i] = dValue;
                    }
                }
              else 
                {
                  double dTmp = sExt.ClampValue(dValue);
                  if (smos_Fabs(dTmp-dValue) > SM_EFF_ZERO) { bRtn = TRUE ; }
                  rClampedVector[i] = dTmp;
                }
            } // end finite interval branch
        } // end Bounded interval check
    } // end iter every dimension

  // all done
  return(bRtn) ;

} // end SmExtentNd::ClampVector

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmExtentNd::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  SM_SPRINTF(sBuff,       _T("SmExtentNd = 0x%p, Dimension = %ld\n"),this,m_lDimension);
  SM_SPRINTF(sBuffForFile,_T("SmExtentNd = %s, Dimension = %ld\n"),_T("notNULL"),m_lDimension);
  smos_WriteBuffer(sBuff, sBuffForFile);

  for (ULONG i=0; i<m_lDimension; i++) 
    {
      SM_SPRINTF(sBuff,_T("\t[%ld] = "),i);
      smos_WriteBuffer(sBuff);
      m_pExtents[i].Dump();
      smos_WriteBuffer(_T("\n"));
    }

} // end SmExtentNd::Dump
