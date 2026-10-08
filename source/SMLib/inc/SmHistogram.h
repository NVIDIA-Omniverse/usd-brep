// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmHistogram.h
* PURPOSE:  Header file for histograms.
**********************************************************************/

/*******************************************************************/ /**
 PURPOSE: Histograms for data analysis and regression testing.

 NOTES:
   1. Histogram structure
                       
         Buckets        :   ---bucket_0->|<--bucket_1-->|<--  . . .  -->|<--bucket_N---
         RealNumberLine :  <-------------+--------------+---- . . . ----+------------>
                                         |              |               |
         BucketBounds   :             bound_0        bound_1         bound_N-1

   1. A histogram divides the real number line into a set of intervals called buckets.
   2. The boundaries between buckets are called Bounds.
   3. Buckets have an index and an interval
       bucket_ii.Interval = [bound_ii-1, bound_ii)
      The first and last buckets are semi-infinite as:
        bucket_0.Interval all values <  bound_0
        bucket_N.Interval all values => bound_N-1
 ***********************************************************************/

#ifndef __SMHISTOGRAM_H__
#define __SMHISTOGRAM_H__

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

class SM_EXPORT SmHistogram
{
public:
  SmTArray<ULONG> m_sBuckets ;    // Number of buckets            = m_sBuckets.GetSize() 
                                  // Bucket_ii interval           = m_sBounds[ii-1] <= val < m_sBounds[ii]}
                                  // Number of items in Bucket_ii = m_sBuckets[ii]       
private:                          
  SmTArray<double> m_sBounds ;    // upper bound value for each bucket in m_sBuckets
                                  // sized:[m_sBuckets.GetSize() - 1]

  // label data for PrettyPrint
  mutable TCHAR m_sLabelLine[4][SM_TBLOCK_SIZE] = { _T("           "), // in : Line1 of 'label1234', spaces are added to make all 4 label lines the same length
                                                    _T("  Histogram"), //    : Line2 of 'label1234', spaces are added to make all 4 label lines the same length
                                                    _T("           "), //    : Line3 of 'label1234', spaces are added to make all 4 label lines the same length
                                                    _T("           ")  //    : Line4 of 'label1234', spaces are added to make all 4 label lines the same length
                                                  } ;                  //    : Label lines are output stacked vertically
  mutable ULONG m_lDataOffset=0 ;                                      // in : A string of spaces used to offset the bucket report.  Useful to align the buckets between different Histogram reports vertically. 

public:
  // default contructor: 2 buckets, 1 bound:[0.0], Divides real numbers into negatives and positives, 0 maps to positive bucket. Bounds and buckets can be adjusted later.
  SmHistogram(TCHAR *pOptLabelLine1=NULL, // in : Line1 of 'label1234', spaces are added to make all 4 label lines the same length
              TCHAR *pOptLabelLine2=NULL, // in : Line2 of 'label1234', spaces are added to make all 4 label lines the same length
              TCHAR *pOptLabelLine3=NULL, // in : Line3 of 'label1234', spaces are added to make all 4 label lines the same length
              TCHAR *pOptLabelLine4=NULL, // in : Line4 of 'label1234', spaces are added to make all 4 label lines the same length
              ULONG  lOptDataOffset=0)    // in : A string of spaces used to offset the bucket report.
                                          { m_sBuckets.SetSize(2) ;
                                            m_sBuckets.SetAll(0) ;
                                            m_sBounds.SetSize(1) ;
                                            m_sBounds[0] = 0.0 ;
                                            SetLabels(pOptLabelLine1, pOptLabelLine2, pOptLabelLine3, pOptLabelLine4, lOptDataOffset) ;
                                          }

  // copy constructor
  SmHistogram(const SmHistogram & crOriginal) : m_sBuckets(crOriginal.m_sBuckets),
                                                m_sBounds(crOriginal.m_sBounds) 
                                              { }

  // constructor for an array of given bound values
  SmHistogram(SmTArray<double> sBounds) // in : Array of bucket boundary values
                                        { SM_ASSERT_BREAK(sBounds.GetSize() > 0) ;
                                          for(ULONG ii=0;ii<sBounds.GetSize()-1;ii++)
                                            { SM_ASSERT_BREAK(sBounds[ii] < sBounds[ii + 1]) ; }
                                          m_sBuckets.SetSize(sBounds.GetSize() + 1) ;
                                          m_sBounds = sBounds ;
                                        }

  // constructor - for specified interval divided into even sized buckets                             
  SmHistogram(ULONG  lNumBuckets,       // in : number of buckets
              double dBottom,           // in : bottom boundary value
              double dTop)              // in : top boundary value
                                        { SM_ASSERT_BREAK(lNumBuckets > 2) ;
                                          SM_ASSERT_BREAK(dTop > dBottom) ;
                                          m_sBuckets.SetSize(lNumBuckets) ;
                                          m_sBounds.SetSize(lNumBuckets - 1) ;
                                          double dStepSize = (dTop - dBottom) / (double)(lNumBuckets - 1) ;
                                          
                                          m_sBounds[0]               = dBottom ;
                                          m_sBounds[lNumBuckets - 2] = dTop ;
                                          for(ULONG ii=1;ii<lNumBuckets-2;ii++)
                                            { m_sBounds[ii] = dBottom + ((double)ii * dStepSize) ; }
                                        }

  // constructor - for sequence of decade sized buckets, eg: bounds = {1.0e-3, 1e-2, 1.0e-1, 1.0e0, 1.0e1, 1.0e2 }
  SmHistogram(ULONG lNumBuckets,        // in : number of buckets
              int   dBottomPower)       // in : bottom boundary exponent value
                                        { SM_ASSERT_BREAK(lNumBuckets > 1) ;
                                          m_sBuckets.SetSize(lNumBuckets) ;
                                          m_sBounds.SetSize(lNumBuckets - 1) ;
                                          
                                          m_sBounds[0] = smos_Pow(10,dBottomPower) ;
                                          for(ULONG ii=0;ii<lNumBuckets-1;ii++)
                                            { m_sBounds[ii] = smos_Pow(10, dBottomPower+(int)ii) ; }
                                        }

  // set Histogram for Healer's default EdgeLength histogram                                     
  void SetHealerHistogramForEdgelengths() { m_sBuckets.SetSize(11) ;
                                            m_sBounds.SetSize(10) ;
                                         
                                            m_sBuckets.SetAll(0) ;
                                            m_sBounds[0] = 1e-5 ;
                                            m_sBounds[1] = 1e-4 ;
                                            m_sBounds[2] = 1e-3 ;
                                            m_sBounds[3] = 1e-2 ;
                                            m_sBounds[4] = 1e-1 ;
                                            m_sBounds[5] = 1e+0 ;
                                            m_sBounds[6] = 1e+1 ;
                                            m_sBounds[7] = 1e+2 ;
                                            m_sBounds[8] = 1e+3 ;
                                            m_sBounds[9] = 1e+4 ;
                                          }

  // set Histogram for Healer's default GapSize histogram
  void SetHealerHistogramForGapSizes()    { m_sBuckets.SetSize(11) ;
                                            m_sBounds.SetSize(10) ;
                                        
                                            m_sBuckets.SetAll(0) ;
                                            m_sBounds[0] = 1e-10 ;
                                            m_sBounds[1] = 1e-8 ;
                                            m_sBounds[2] = 1e-6 ;
                                            m_sBounds[3] = 1e-5 ;
                                            m_sBounds[4] = 1e-4 ;
                                            m_sBounds[5] = 1e-3 ;
                                            m_sBounds[6] = 1e-2 ;
                                            m_sBounds[7] = 1e-1 ;
                                            m_sBounds[8] = 1e+0 ;
                                            m_sBounds[9] = 1e+1 ;
                                          }
  // operator=
  SmHistogram & operator=(const SmHistogram & crHistogram)   { if(this == &crHistogram) { return *this ; }
                                                               m_sBuckets = crHistogram.m_sBuckets ;
                                                               m_sBounds  = crHistogram.m_sBounds ;
                                                               return *this ;
                                                             }
  
  // operator==
  SmBoolean operator==(const SmHistogram &crHistogram) const { if(this == &crHistogram) { return TRUE ; }
                                                               return (   m_sBounds  == crHistogram.m_sBounds  
                                                                       && m_sBuckets == crHistogram.m_sBuckets) ;
                                                             }
  // destructor                         
  ~SmHistogram() = default ;
  
  // simple data access                              
  SmTArray<double> & GetBounds     ()                    { return m_sBounds ; } 
  double             GetBound      (ULONG lIndex)  const { SM_ASSERT_BREAK(lIndex < this->GetBucketSize() - 1) ;
                                                           return m_sBounds[lIndex] ;
                                                         }
  SmTArray<ULONG>  & GetBuckets    ()                    { return m_sBuckets ; } 
  ULONG              GetBucketSize ()              const { return m_sBuckets.GetSize() ; } 
  void               ReSetBucketCounts()                 { m_sBuckets.SetAll(0) ; }
  ULONG              GetBucketCount(ULONG lIndex)  const { SM_ASSERT_BREAK(lIndex < this->GetBucketSize()) ;
                                                           return m_sBuckets[lIndex] ;
                                                         }
  ULONG              GetBucketIndex(double dValue) const { ULONG lBucketCount = this->GetBucketSize();
                                                           for(ULONG ii=0;ii<lBucketCount-1;ii++)
                                                             { if(dValue < m_sBounds[ii])
                                                                 { return ii ; }
                                                             }
                                                           return lBucketCount-1 ;
                                                         }
  // Add one value 
  void  Add(double dValue)                               { ULONG lIndex = this->GetBucketIndex(dValue) ;
                                                           m_sBuckets[lIndex]++ ;
                                                         }
  // add array of values                                 
  void  Add(const SmTArray<double> & rValues)            { for(ULONG ii=0;ii<rValues.GetSize();ii++)
                                                             { this->Add(rValues[ii]) ; }
                                                         }
  // PrettyPrint
  //  yields:
  //   'pOptLabelLine1'     +---------------------------------------------------------------------------------------------+
  //   'pOptLabelLine2'     :'pOptDataOffset'[   0] [   0] [    0] [   0] [   6] [  26] [ 255] [ 158] [    7] [   0] [   0]
  //   'pOptLabelLine3' from:'pOptDataOffset'[less] [1e-5] [.0001] [.001] [ .01] [  .1] [  1.] [ 10.] [ 100.] [1e+3] [1e+4]
  //   'pOptLabelLine4'   to:'pOptDataOffset'[1e-5] [1e-4] [.001 ] [.01 ] [ .1 ] [ 1.0] [ 10.] [100.] [1000.] [1e+4] [more]
  void SetLabels(TCHAR *pOptLabelLine1=NULL, // in : Line1 of 'label1234', spaces are added to make all 4 label lines the same length
                 TCHAR *pOptLabelLine2=NULL, // in : Line2 of 'label1234', spaces are added to make all 4 label lines the same length
                 TCHAR *pOptLabelLine3=NULL, // in : Line3 of 'label1234', spaces are added to make all 4 label lines the same length
                 TCHAR *pOptLabelLine4=NULL, // in : Line4 of 'label1234', spaces are added to make all 4 label lines the same length
                 ULONG  lOptDataOffset=0)    // in : A string of spaces used to offset the bucket report.
                                             //      Useful to align the buckets between different Histogram reports vertically. 
                                             { if(pOptLabelLine1) { smos_sprintf(m_sLabelLine[0], _T("%s "), pOptLabelLine1) ; }
                                               if(pOptLabelLine2) { smos_sprintf(m_sLabelLine[1], _T("%s "), pOptLabelLine2) ; }
                                               if(pOptLabelLine3) { smos_sprintf(m_sLabelLine[2], _T("%s "), pOptLabelLine3) ; }
                                               if(pOptLabelLine4) { smos_sprintf(m_sLabelLine[3], _T("%s "), pOptLabelLine4) ; }
                                               m_lDataOffset = lOptDataOffset ;
                                             }
  void Dump(ULONG lOptLineMask=7) const ;  // in: orof 1 = Print Top header line: "pOptLabel1      +---------...-------+'
                                           //          2 = Print Count line     : "pOptLabel2 Count: [ cnt ] ... [ cnt ]
                                           //          4 = Print bound lines    : "PoptLabel3  from: [ min ] ... [ min ]
                                           //                                   : "PoptLabel4    to: [ max ] ... [ max ]
                                           //     convenient printing several similar histograms next to one another.
  
} ; // end class SmHistogram

#endif // end __SMHISTOGRAM_H__
