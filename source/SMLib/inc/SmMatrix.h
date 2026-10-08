// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmMatrix.h
* PURPOSE: Header file for the generic matrix type.
**********************************************************************/

#ifndef __SMMATRIX_H__
#define __SMMATRIX_H__

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#define MAX_MATRIX_SIZE 1000

/*******************************************************************//**
PURPOSE: This object represents a multi-dimensional matrix of double
   precision values.  It is primarily used to solve equations.

NOTES: It has some internal memory preallocated to handle
   most common cases.
***********************************************************************/
class SM_EXPORT SmMatrix 
{
private:
  ULONG    m_lNumRows;        // matrix row count     
  ULONG    m_lNumColumns;     // matrix col count     
  double   m_adMat[36];       // enough room for a 6x6 matrix
  double  *m_apRows[6];       // enough room for a 6 row matrix
  double  *m_pdM;             // array of Matrix elements 
                              // organized:[row1, row2, row3, .., rowN]     
  double **m_ppdM;            // array of row pointers    

public:

  // constructors, destructor, SetSize()
  SmMatrix(ULONG lNumRows, ULONG lNumColumns);    // note: leaves mat values uninitialized
  SmMatrix(const SmMatrix & crSource);            // note: copy constructor, uses smos_MemCpy to copy mat values - okay for static class objects
  SmMatrix(const SmVector3d & crRow1,             // note: copies vec values into array
           const SmVector3d & crRow2,
           const SmVector3d & crRow3);
  void SetSize(ULONG lNumRows, ULONG lNumCols) ;  // note: size M, leave memory uninitialized
  ~SmMatrix();

  // simple access
  ULONG    GetNumRows()    const { return m_lNumRows; }
  ULONG    GetNumColumns() const { return m_lNumColumns; }
  double * operator[] (ULONG lRow) const;                    // rtn: pointer to lRow
  double *&operator[] (ULONG lRow);                          // rtn: ref to Pointer to lRow
 
  // row/col access
  void     SetAt(ULONG lRow, ULONG lCol, double dValue);
  void     AddTo(ULONG lRow, ULONG lCol, double dValToAdd);
  double   GetAt(ULONG lRow, ULONG lCol) const;              
 
  // index access, where: lIndex = row*Numcols + col
  void     Set(ULONG lIndex, double dValue) ;                
  void     Add(ULONG lIndex, double dValToAdd) ;
  double   Get(ULONG lIndex) ;                               

  double   GetMaxDiagonalDimension() ;  // rtn Max Fabs(diag_elem)
  double   GetMaxDimension() ;          // rtn Max Fabs(elem)

  // predicates
  SmBoolean IsSymmetric() const ;                                         // rtn: TRUE = square and symmetric, else FALSE
  SmBoolean IsEqual(const SmMatrix & crOther, double dTol=1.e-8) const ; // rtn: TRUE all elements are within Tol of one another

  // simple matrix math
  SmStatus Clear         ();                                       // set M = 0
  SmStatus MakeIdentity  ();                                       // set M = I
  void     operator=     (const SmMatrix & crOther) ;              // set M = crOther
  SmStatus Add           (const SmMatrix & crOther,                // set R = M + B
                                SmMatrix & rResult) const;         
  SmStatus Subtract      (const SmMatrix & crOther,                // set R = M - B
                                SmMatrix & rResult) const;         
  SmStatus Multiply      (const SmMatrix & crOther,                // set R = M * B  - Multiply by a Matrix
                                SmMatrix & rResult) const;         
  SmStatus Multiply      (const SmTArray<double> & pVector,        // set R = M * V  - Multiply by a vector
                                SmTArray<double> & rResult) const;                   
  SmStatus Multiply      (double dValue);                          // set M = d * M  - Multiply by a Scalar
  SmStatus Invert        ();                                       // set M = Inverse(M)
  SmStatus Transpose     ();                                       // set M = Transpose(M)

  // backward compatible old names
  SmStatus MultiplyBy    (double dValue)                           { return( Multiply(dValue) ) ; } ;
  SmStatus MultiplyVector(const SmTArray<double> & crVector, 
                                SmTArray<double> & rResult) const  { return( Multiply( crVector, rResult) ) ; }

  // matrix Ax=b solvers
  SmStatus SolveLinearSystem(const SmTArray<double> & crRightHandSide,        // single solve of Ax = b
                                   SmTArray<double> & rSolutionVector) const; // GWC: does not signal singular matrix errors

  SmStatus SolveWithLUMatrix(ULONG *laIdx,                                    // repeated Solves of Ax = b
                             const SmTArray<double> & crRightHandSide,        //    after 1 call to LUDecompose().
                                   SmTArray<double> & rSolutionVector) const; // GWC: does not signal singular matrix errors

  SmStatus LUDecompose(ULONG *plIndx);                                        // Setup function for SolveWithLUMatrix()

  SmStatus SolveWithSVDMatrices(const SmTArray<double> & crRightHandSide,     // Solve with SVD after call to SVDecompose()
                                      SmTArray<double> &  rW,
                                const SmMatrix         & crV,
                                      SmTArray<double> & rSolutionVector,
                                      double             dSVDTol=6.e-8,
                                      SmBoolean          bAdSingularValues=TRUE) const;

  SmStatus SVDecompose(SmTArray<double> &rW, SmMatrix &rVt) ;                 // Setup function for SolveWithSVDMatrices()

  SmStatus SolveZeroLinearSystem(SmTArray<double> & rSolutions) const;        // Solve Ax = 0

  ULONG    Rank() const;

  SmStatus MakeRowEchelon( ULONG &rRank, double dTol = SM_EFF_ZERO );         // Modifies 'this'.

  SmStatus SwapRows( ULONG lRow1, ULONG lRow2 );

  // get eigenvalues and eigenvectors for symmetric real matrices
  SmStatus EigenSystemJacobiSolver(SmTArray<double> &rEigenValues, SmMatrix &rEigenVecs, ULONG &rIterCnt) ;

  // Special 3x3 matrix functions
  SmStatus Compute3x3Determinant(double & rdDeterminant) const;
  SmStatus Compute3x3EigenValues(ULONG & rlNumEigenValues,
                                 double adEigenvalues[3]) const;

  SmStatus Compute3x3EigenVectors(ULONG & rlNumEigenVectors,           // NotUsed: out: 
                                  double adEigenValues[3],             // out: 
                                  SmVector3d aEigenVectors[3]) const;  // out: 

  // get memory used and allocated
  static SmBoolean UnitTest() ;
  ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const  
       { rlMemoryAllocated = sizeof(this) + (  (m_lNumRows > 6 || m_lNumColumns > 6) 
                                             ? (  sizeof(double )*m_lNumRows*m_lNumColumns
                                                + sizeof(double*)*m_lNumRows) 
                                             : 0) ; 
         return(rlMemoryAllocated) ;
       }

  // for possible forward compatability
  void Dump() const;

} ; // end class SmMatrix

/*******************************************************************//**
PURPOSE: element access

NOTES:
***********************************************************************/
inline double SmMatrix::GetAt
  (ULONG lRow, 
   ULONG lCol) 
  const
{
    SM_ASSERT(lRow < m_lNumRows && lCol < m_lNumColumns);
    return m_ppdM[lRow][lCol];

} // end SmMatrix::GetAt

/*******************************************************************//**
PURPOSE:  element set

NOTES:
***********************************************************************/
inline void SmMatrix::SetAt
  (ULONG lRow, 
   ULONG lCol, 
   double dValue)
{
    SM_ASSERT(lRow < m_lNumRows && lCol < m_lNumColumns);
    m_ppdM[lRow][lCol] = dValue;

} // end SmMatrix::SetAt

/*******************************************************************//**
PURPOSE:  element add

NOTES:
***********************************************************************/
inline void SmMatrix::AddTo
  (ULONG lRow, 
   ULONG lCol, 
   double dValToAdd)
{
    SM_ASSERT(lRow < m_lNumRows && lCol < m_lNumColumns);
    m_ppdM[lRow][lCol] += dValToAdd;

} // end SmMatrix::SetAt

/*******************************************************************//**
PURPOSE:  Set an element in a matrix treated as one long of vectors

NOTES: lIndex = ColCount * row + col
***********************************************************************/
inline void SmMatrix::Set
  (ULONG lIndex, 
   double dValue)
{
    SM_ASSERT(lIndex < m_lNumRows * m_lNumColumns);
    m_pdM[lIndex] = dValue;

} // end SmMatrix::Set

/*******************************************************************//**
PURPOSE:  Get an element from a matrix treated as one long vector

NOTES: lIndex = ColCount * row + col
***********************************************************************/
inline double SmMatrix::Get
  (ULONG lIndex)
{
    SM_ASSERT(lIndex < m_lNumRows * m_lNumColumns);
    return m_pdM[lIndex];

} // end SmMatrix::Get

/*******************************************************************//**
PURPOSE:  rtn Max Fabs(diag_elem)

NOTES: 
***********************************************************************/
inline double SmMatrix::GetMaxDiagonalDimension()
{
  ULONG ii, lIndex ;
  double dMaxDim = 0.0 ;

  // linear search
  for(ii=0,lIndex=0;ii<m_lNumRows;ii++,lIndex += m_lNumColumns+1)
    {
      SM_ASSERT(lIndex < m_lNumRows * m_lNumColumns);
      if(smos_Fabs(m_pdM[lIndex]) > dMaxDim) { dMaxDim = smos_Fabs(m_pdM[lIndex]) ; }
    }
  return(dMaxDim) ;

} // end SmMatrix::GetMaxDiagonalDimension

/*******************************************************************//**
PURPOSE:  rtn Max Fabs(elem)

NOTES: 
***********************************************************************/
inline double SmMatrix::GetMaxDimension()
{
  ULONG lIndex, lCnt = m_lNumRows * m_lNumColumns ;
  double dMaxDim = 0.0 ;

  // linear search
  for(lIndex=0;lIndex<lCnt;lIndex++)
    {
      if(smos_Fabs(m_pdM[lIndex]) > dMaxDim) { dMaxDim = smos_Fabs(m_pdM[lIndex]) ; }
    }
  return(dMaxDim) ;

} // end SmMatrix::GetMaxDimension

/*******************************************************************//**
PURPOSE:  add to an element in a matrix treated as one long vector

NOTES: lIndex = ColCount * row + col
***********************************************************************/
inline void SmMatrix::Add
  (ULONG lIndex, 
   double dValue)
{
    SM_ASSERT(lIndex < m_lNumRows * m_lNumColumns);
    m_pdM[lIndex] += dValue;

} // end SmMatrix::Add

/*******************************************************************//**
PURPOSE: Get Pointer to Row

NOTES:
***********************************************************************/
inline double * SmMatrix::operator[] (ULONG lRow) const
{ 
    SM_ASSERT(lRow < m_lNumRows);
    return m_ppdM[lRow];

} // end SmMatrix::operator

/*******************************************************************//**
PURPOSE: Get Reference to pointer to Row

NOTES:
***********************************************************************/
inline double *& SmMatrix::operator[] (ULONG lRow)
{ 
    SM_ASSERT(lRow < m_lNumRows);
    return m_ppdM[lRow];

} // end SmMatrix::operator

#endif // !__SMMATRIX_H__


