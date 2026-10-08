// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmExtentNd.h
* PURPOSE: N-Dimensional extent object header file.
**********************************************************************/

#ifndef __SMEXTENTND_H__
#define __SMEXTENTND_H__

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMOS_MEMORY_H__
#include <SmMemory.h>
#endif

template<class TYPE>
class SmTArray;

/*******************************************************************//**
PURPOSE: This object represents an N-dimensional domain.  It contains
    an array of SmExtent1d objects.  

NOTES: Note that we have optimized memory allocation by 
    having 10 already allocated as part of the object.
***********************************************************************/
class SM_EXPORT SmExtentNd
{
protected:
    ULONG       m_lDimension = 0 ;  // dimension for this extent 
    SmExtent1d  m_aExtents[10] ;    // A 1d extent for 1st 10 dimensions
    SmExtent1d *m_pExtents = NULL ; // if needed, memory for more than 10 dimensions 
public:
    // constructor, destructor
    SmExtentNd(ULONG lDimension);
   ~SmExtentNd();

    // simple data access
    SmExtent1d   operator[] ( ULONG lIndex ) const;  // dim i range:[sExtentNd[i].GetMin() sExtentNd[i].GetMax()]
    SmExtent1d & operator[] ( ULONG lIndex );                               
    
    // Clamp vector elements to their individual bounds, return TRUE = at least 1 parameter was clamped.
    SmBoolean ClampVector(const SmTArray<double>    & crVectorToClamp,      // in : Vector of double values to be clamped
                          const SmTArray<SmBoolean> * cpPeriodicityVector,  // in : ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                                                            //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped
                                                                            //       when ivl[i] == ivl[i]SetUnbounded() VectorToClamp[i] is never clamped or wrapped
                          SmTArray<double>  & rClampedVector,               // out: The resulting clamped vector.
                          SmTArray<ULONG>   * pOutOfBoundsCountVector=NULL) // out: An out of bounds counter for each interval.  Note that
                         const ;                                            //      this method increments existing counts in the vector.

    void Dump(void) const;

    const TCHAR * GetTypeString() const { return _T("SmExtentNd") ; }

} ; // end class SmExtentNd

/*******************************************************************//**
PURPOSE: Construct an N-dimensional extent given the initial dimension.

NOTES: 
***********************************************************************/
inline SmExtentNd::SmExtentNd(ULONG lDimension) 
: m_lDimension(lDimension) 
{ 
  // init all the 1d extents
  m_aExtents[0].Init();
  m_aExtents[1].Init();
  m_aExtents[2].Init();

  if (lDimension > 2) 
    {
      m_aExtents[3].Init();
      m_aExtents[4].Init();
      m_aExtents[5].Init();
      m_aExtents[6].Init();
      m_aExtents[7].Init();
      m_aExtents[8].Init();
      m_aExtents[9].Init();
    }

  // memory for dimensions beyond the first 10
  if (lDimension > 10) 
    {
      // okay to use smos_Calloc on static class (SmExtent1d) objects.
      m_pExtents = (SmExtent1d*)smos_Calloc(1, sizeof(SmExtent1d)*(lDimension-10));
      
      ULONG ii ;
      for(ii=0;ii+10<lDimension;ii++)  // note: can't say lDimension-10
        {
          m_pExtents[ii].Init() ;
        } 
    }
  else 
    { m_pExtents = NULL; }

} // end SmExtentNd::SmExtentNd constructor

/*******************************************************************//**
PURPOSE: Destructor for the N-dimensional extent.

NOTES: 
***********************************************************************/
inline SmExtentNd::~SmExtentNd() 
{ 
  if (m_pExtents) { smos_Free(m_pExtents); 
                    m_pExtents = NULL ;
                  } 
}

/*******************************************************************//**
PURPOSE: Value based [] operator which returns the value of an extent
    at a given index in the N-dimensional extent.

NOTES: 
***********************************************************************/
inline SmExtent1d SmExtentNd::operator[] ( ULONG lIndex ) const 
{ 
    SM_ASSERT(lIndex<m_lDimension); 
    return ((lIndex >= 10) ? m_pExtents[lIndex-10] : m_aExtents[lIndex]);
}

/*******************************************************************//**
PURPOSE: Reference based [] operator which returns a reference to the
    extent at a given index in the N-dimensional extent.

NOTES: 
***********************************************************************/
inline SmExtent1d & SmExtentNd::operator[] ( ULONG lIndex )
{ 
    SM_ASSERT(lIndex<m_lDimension); 
    return ((lIndex >= 10) ? m_pExtents[lIndex-10] : m_aExtents[lIndex]);
}


#endif // !__SMEXTENTND_H__


