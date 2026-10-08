// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmFaceGrid.h
* PURPOSE: Header file for SmFaceGrid class.
**********************************************************************/

#ifndef __SMFACEGRIDEVAL_H__
#define __SMFACEGRIDEVAL_H__

 //#pragma warning(disable : 4291)   // no matching operator delete found; // restored these warnings to clean up errors on linux

#include <SmTypes.h>
#include <SmTArray.h>
#include <SmSurface.h>        // for class SmSurfaceEval
#include <SmFace.h>
#include <SmExtent2d.h>
#include <SmVector2d.h>

/*******************************************************************//**
PURPOSE: Container for a Face->Surface Eval and PointClassify for a
         common UV value.

NOTES:
***********************************************************************/
class SM_EXPORT SmFaceEval
{
 private:
  SmSurfaceEval m_sSurfaceEval ;
  SmBoolean     m_bInFace      = FALSE;

 public:
  // constructor
  SmFaceEval() { }

  // operators 
  SmFaceEval & operator= (const SmFaceEval &crOther ) { if(&crOther == this) return *this ;
                                                        m_sSurfaceEval = crOther.m_sSurfaceEval ;
                                                        m_bInFace = crOther.m_bInFace;
                                                        return *this ;
                                                      }
  SmBoolean    operator==(const SmFaceEval &crOther ) { if(&crOther == this) return TRUE ;
                                                        SmBoolean bRtn = m_sSurfaceEval == crOther.m_sSurfaceEval ;
                                                        bRtn &= m_bInFace == crOther.m_bInFace ;
                                                        return(bRtn) ;
                                                      }

  // simple data access
  SmSurfaceEval   * GetSurfaceEval()             { return &m_sSurfaceEval; }
  SmBoolean         IsInFace()                   { return m_bInFace; }
  void              SetInFace(SmBoolean bInFace) { m_bInFace = bInFace; }

} ; // end class SmFaceEval

SM_TARRAY_TEMPLATE_PREDECLARATION(SmFaceEval);

/*******************************************************************//**
PURPOSE: Regularly spaced UVSpace Face Surface and Classify Face samples

NOTES:
***********************************************************************/
class SM_EXPORT SmFaceGrid
{
 protected:
  // meta data
  const SmFace       * m_cpFace    = NULL ;     // Target Face
  const SmSurface    * m_cpSurface = NULL ;     //   Face->Surface
  SmExtent2d           m_sUVDomain;             //   Face->UVDomain
  ULONG                m_lRowCnt = 256;         // number of rows    = number of v samples
  ULONG                m_lColCnt = 256;         // number of columns = number of u samples
  SmTArray<SmFaceEval> m_sGrid;                 // array of SmFaceEval samples sized (m_lRowCnt * m_lColCnt)
                                                //  access Indx(u,v) = v*m_lColCnt + u 

  // Grid of Face Surface samples and classifications

public:
  // constructors and destructor
  SmFaceGrid(const SmFace * cpFace  = NULL,   // in : Target Face
             ULONG          lRowCnt = 256,    // in : number of rows    = number of v samples
             ULONG          lColCnt = 256)    // in : number of columns = number of u samples
    : m_cpFace(cpFace),
      m_cpSurface(cpFace->GetSurface()),
      m_sUVDomain(cpFace->GetUVDomain()),
      m_lRowCnt(lRowCnt),
      m_lColCnt(lColCnt),
      m_sGrid(m_lRowCnt * m_lColCnt, NULL, m_lRowCnt * m_lColCnt)
    { LoadGrid() ; }

  // Load the Grid - SmSurfaceEvals are lazy evals, CurveClassifications are evaluated here
  void LoadGrid() ;

  // simple access
  SmFaceEval            * GetDataArray          () const                       { return m_sGrid.GetDataArray() ; }
  SmFaceEval            * GetFaceEval           (ULONG lRow, ULONG lCol)       { SM_ASSERT_BREAK(lRow < m_lRowCnt) ;
                                                                                 SM_ASSERT_BREAK(lCol < m_lColCnt) ;
                                                                                 return &m_sGrid[lRow * m_lColCnt + lCol] ;
                                                                               }
  SmSurfaceEval         * GetSurfaceEval        (ULONG lRow, ULONG lCol)       { return GetFaceEval(lRow, lCol)->GetSurfaceEval() ; }
  SmVector2d              GetUV                 (ULONG lRow, ULONG lCol)       { return m_sUVDomain.Evaluate((double)lRow / (double)(m_lColCnt - 1),
                                                                                                             (double)lCol / (double)(m_lRowCnt - 1)) ;
                                                                               }
  void                    SetIsInFace           (ULONG lRow, ULONG lCol, SmBoolean bIsInFace)
                                                                               { GetFaceEval(lRow, lCol)->SetInFace(bIsInFace); }
  SmBoolean               IsInFace              (ULONG lRow, ULONG lCol)       { return GetFaceEval(lRow, lCol)->IsInFace(); }

  // Draw
  SmDisplayList* Draw(SmGfxArraySet* pOptGfxSet = NULL) ;

  // pretty print
  void Dump(ULONG lLabel=SM_UNDEF_ULONG) ; // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_ULONG]
  void DumpHeader() const ;
  void DumpFaceMap(ULONG lLabel=SM_UNDEF_ULONG) const ;   // output array of 1=InFace, 0=OutOfFace
  
  }; // end class SmFaceGrid

// add a SmTArray<SmLoopProps> template to the dll interface
// SM_TARRAY_TEMPLATE_PREDECLARATION(SmFaceGrid) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFaceGrid*);

#endif // !__SMFACEGRIDEVAL_H__

