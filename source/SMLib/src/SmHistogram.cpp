// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmHistogram.cpp
* PURPOSE:  Source file for histograms.
**********************************************************************/

/*******************************************************************/ /**
 PURPOSE: Histograms for data analysis and regression testing.

 NOTES: 
 ***********************************************************************/

#include "StdAfx.h"
#include <SmHistogram.h>
#include <SmTypes.h>

/*******************************************************************/ /**
 PURPOSE: PrettyPrint

 NOTES:

 1. The Histogram label is a block of 4 vertically aligned opitonal text strings placed
    at the front of the 4 lines of the bucket dump report.

 2. An optional text string of spaces can be used to align the columns between a set of similar histograms with different ranges

 SAMPLE:
  'LabelLine1' +-----------------------------------------------------------------------------------------------------------------------------+ 
  'LabelLine2' Count: [   1796 ] [      0 ] [      6 ] [      6 ] [      0 ] [      0 ] [      0 ] [      0 ] [      0 ] [      0 ] [      0 ] 
  'LabelLine3'  from: [  less  ] [1.00e-10] [1.00e-08] [1.00e-06] [1.00e-05] [1.00e-04] [1.00e-03] [1.00e-02] [1.00e-01] [1.00e+00] [1.00e+01] 
  'LabelLine4'    to: [1.00e-10] [1.00e-08] [1.00e-06] [1.00e-05] [1.00e-04] [1.00e-03] [1.00e-02] [1.00e-01] [1.00e+00] [1.00e+01] [  more  ]
 ***********************************************************************/
void SmHistogram::Dump
 (ULONG lOptLineMask)  // in: orof 1 = Print Top header line: "pOptLabel1      +---------...-------+'
 const                 //          2 = Print Count line     : "pOptLabel2 Count: [ cnt ] ... [ cnt ]
                       //          4 = Print bound lines    : "PoptLabel3  from: [ min ] ... [ min ]
                       //                                   : "PoptLabel4    to: [ max ] ... [ max ]
                       //     convenient printing several similar histograms next to one another.
                       //     default:[7] - print all lines
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffOffset[2][SM_TBLOCK_SIZE] ;
  size_t lLabelLength[5] ;
  lLabelLength[1] = SM_STRLEN(m_sLabelLine[0]) ;
  lLabelLength[2] = SM_STRLEN(m_sLabelLine[1]) ;
  lLabelLength[3] = SM_STRLEN(m_sLabelLine[2]) ;
  lLabelLength[4] = SM_STRLEN(m_sLabelLine[3]) ;
  lLabelLength[0] = smos_4Max(lLabelLength[1], lLabelLength[2], lLabelLength[3], lLabelLength[4]) ;

  // build sBuffOffset text strings of all blanks and all dashes
  smos_sprintf(sBuffOffset[0], _T("%s"), _T("")) ;
  smos_sprintf(sBuffOffset[1], _T("%s"), _T("")) ;
  for(ii=0;ii<m_lDataOffset;ii++)
    {
      size_t lLen0 = SM_STRLEN(sBuffOffset[0]) ;
      if (lLen0 + 2 < SM_TBLOCK_SIZE) { sBuffOffset[0][lLen0] = _T(' ') ; sBuffOffset[0][lLen0 + 1] = _T('\0') ; }
      size_t lLen1 = SM_STRLEN(sBuffOffset[1]) ;
      if (lLen1 + 2 < SM_TBLOCK_SIZE) { sBuffOffset[1][lLen1] = _T('-') ; sBuffOffset[1][lLen1 + 1] = _T('\0') ; }
    }

  // pad label lines with spaces to make them all the same length
  for(ii=0;ii<4;ii++)
    {
      size_t lPadCount = lLabelLength[0] - lLabelLength[ii + 1] ;
      for(size_t jj=0;jj<lPadCount;jj++)
        {
          size_t lLen = SM_STRLEN(m_sLabelLine[ii]) ;
          if (lLen + 2 < SM_TBLOCK_SIZE) { m_sLabelLine[ii][lLen] = _T(' ') ; m_sLabelLine[ii][lLen + 1] = _T('\0') ; }
        }
    }

  // header line - "m_sLabelLine[0] +----------+--...----+----------+"
  if(lOptLineMask & 1)
    {
      smos_sprintf(sBuff,_T("\n %.512s +--------------"),m_sLabelLine[0]) ;
      smos_WriteBuffer(sBuff) ;
      smos_WriteBuffer(sBuffOffset[1]) ;
      for(ii=0;ii<this->GetBucketSize(); ii++)
        {
          smos_sprintf    (sBuff, _T("%s"), _T("-----------")) ;
          smos_WriteBuffer(sBuff) ;
        }
      smos_sprintf    (sBuff,_T("%s"), _T("-+")) ;
      smos_WriteBuffer(sBuff) ;
    } // end lOptLineMask & 1 - print header line
     
  if(lOptLineMask & 2)
    {
      // histogram count values line - "m_sLabelLine[1] Count: [  cnt  ] [ cnt   ] ... "
      smos_sprintf    (sBuff, _T("\n %.256s Count: %.512s"),m_sLabelLine[1], sBuffOffset[0]) ;
      smos_WriteBuffer(sBuff) ;
      for(ii=0;ii<this->GetBucketSize(); ii++)
        {
          smos_sprintf    (sBuff, _T("[  %6lu  ] "), this->GetBucketCount(ii)) ;
          smos_WriteBuffer(sBuff) ;
        }
    } // end lOptLineMask & 2 - print count line

  if(lOptLineMask & 4)
    {
      // histogram from bound label line - "m_sLabelLine[2]  from: [ less ] [ bound0 ] [ bound 1 ] ... "
      smos_sprintf    (sBuff, _T("\n %.256s  from: %.512s[   less  ] "),m_sLabelLine[2], sBuffOffset[0]) ;
      smos_WriteBuffer(sBuff) ;
      for(ii=0;ii<this->GetBucketSize() - 1; ii++)
        {
          if (this->GetBound(ii) < 0) { smos_sprintf(sBuff, _T("[%4.2e] "), this->GetBound(ii)) ; }
          else                        { smos_sprintf(sBuff, _T("[ %4.2e] "), this->GetBound(ii)) ; }
          smos_WriteBuffer(sBuff) ;
        }
      
      // histogram to bound label line - "m_sLabelLine[2]    to: [ bound0 ] [ bound1 ] ... [ more ]"
      smos_sprintf(sBuff, _T("\n %.256s    to: %.512s"),m_sLabelLine[3], sBuffOffset[0]) ;
      smos_WriteBuffer(sBuff) ;
      for(ii=0;ii<this->GetBucketSize() - 1; ii++)
        {
          if (this->GetBound(ii) < 0) { smos_sprintf(sBuff, _T("[%4.2e] "), this->GetBound(ii)) ; }
          else                        { smos_sprintf(sBuff, _T("[ %4.2e] "), this->GetBound(ii)) ; }
          smos_WriteBuffer(sBuff) ;
        }
      smos_sprintf(sBuff, _T("%s"),_T("[   more  ]" )) ;
      smos_WriteBuffer(sBuff) ;
    } // end lOptLineMask & d - print bounds lines

} // SmHistogram::Dump
