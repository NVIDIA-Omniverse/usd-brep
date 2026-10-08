// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmMatrix.cpp
* PURPOSE: Implementation of generic matrix methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmMatrix.h>
#include <SmPolynomial.h>
#include <SmGeomUtility.h>

/*******************************************************************//**
PURPOSE: Constructor for a matrix which inputs the sizes of the matrix.

NOTES: Leaves Mat values uninitialized
***********************************************************************/
SmMatrix::SmMatrix
  (ULONG lNumRows,                  // in : row count      
   ULONG lNumColumns)               // in : col count
 : m_lNumRows(lNumRows), 
   m_lNumColumns(lNumColumns),
   m_pdM(NULL),
   m_ppdM(NULL)
{
  // check state
  SM_ASSERT(lNumRows > 0 && lNumColumns > 0);

  // allocate large matrix memory
  if (lNumRows > 6 || lNumColumns > 6) 
    {
      // okay to use smos_Calloc on base(double, ptr,..) objects.
      m_pdM  = (double*) smos_Calloc(1,sizeof(double) *lNumRows*lNumColumns);
      m_ppdM = (double**)smos_Calloc(1,sizeof(double*)*lNumRows);
    }
  else 
    {
      m_pdM  = m_adMat;
      m_ppdM = m_apRows;
    }

  // set row pointers
  ULONG ii;
  for (ii=0; ii<lNumRows; ii++) 
    {
      m_ppdM[ii] = &m_pdM[ii*lNumColumns];
    }

} // end SmMatrix::SmMatrix constructor

/*******************************************************************//**
PURPOSE: Constructor a 3x3 matrix constructed using 3 vectors.

NOTES: 
***********************************************************************/
SmMatrix::SmMatrix
  (const SmVector3d & crRow1,       // in : row 1
   const SmVector3d & crRow2,       // in : row 2
   const SmVector3d & crRow3)       // in : row 3
  : m_lNumRows(3), 
    m_lNumColumns(3)
{
  m_pdM  = m_adMat;
  m_ppdM = m_apRows;

  ULONG ii;
  for (ii=0; ii<m_lNumRows; ii++) 
    {
      m_ppdM[ii] = &m_pdM[ii*m_lNumColumns];
    }
  m_ppdM[0][0] = crRow1.x;
  m_ppdM[0][1] = crRow1.y;
  m_ppdM[0][2] = crRow1.z;
  m_ppdM[1][0] = crRow2.x;
  m_ppdM[1][1] = crRow2.y;
  m_ppdM[1][2] = crRow2.z;
  m_ppdM[2][0] = crRow3.x;
  m_ppdM[2][1] = crRow3.y;
  m_ppdM[2][2] = crRow3.z;

} // end SmMatrix::SmMatrix constructor

/*******************************************************************//**
PURPOSE: Copy constructor for the SmMatrix.

NOTES: 
***********************************************************************/
SmMatrix::SmMatrix
  (const SmMatrix & crSource)        // in : Matrix to Copy
{
  // copy matrix size
  m_lNumRows    = crSource.GetNumRows();
  m_lNumColumns = crSource.GetNumColumns();

  // set up memory - fixed memory for less than 6x6 otherwise get memory from heap
  if (m_lNumRows > 6 || m_lNumColumns > 6) 
    {
      // okay to use smos_Calloc on base(double, ptr,..) objects.
      m_pdM  = (double*) smos_Calloc(1,sizeof(double) *m_lNumRows*m_lNumColumns);
      m_ppdM = (double**)smos_Calloc(1,sizeof(double*)*m_lNumRows);
    }
  else 
    {
      m_pdM  = m_adMat;
      m_ppdM = m_apRows;
    }
  
  // set the row pointers
  ULONG ii;
  for (ii=0; ii<m_lNumRows; ii++) 
    {
      m_ppdM[ii] = &m_pdM[ii*m_lNumColumns];
    }

  // copy the array
  // okay to use smos_MemCpy on base (double) objects.
  SE(smos_MemCpy(this->m_pdM, crSource.m_pdM, sizeof(double) * this->GetNumRows() * this->GetNumColumns(), sizeof(double) * this->GetNumRows() * this->GetNumColumns()));

} // end SmMatrix::SmMatrix

/*******************************************************************//**
PURPOSE: class destructor

NOTES:
***********************************************************************/
SmMatrix::~SmMatrix() 
{ 
  if (m_lNumRows > 6 || m_lNumColumns > 6) 
    {
    if(m_pdM && (m_pdM != m_adMat)) { smos_Free( m_pdM );  m_pdM = NULL; }
    if(m_ppdM && (m_ppdM != m_apRows)) { smos_Free( m_ppdM ); m_ppdM = NULL; }
    }

} // end SmMatrix::~SmMatrix destructor

/*******************************************************************//**
PURPOSE: Size array.

NOTES: memory left in uninitialized state
***********************************************************************/
void SmMatrix::SetSize
  (ULONG lNumRows,                  // in : row count      
   ULONG lNumColumns)               // in : col count
{
  // no work
  if(   lNumRows    == m_lNumRows
     && lNumColumns == m_lNumColumns)
    {
      return ;
    }
    
  // free existing memory if needed 
  if(m_lNumRows > 6 || m_lNumColumns > 6)
  {
    if(m_pdM && (m_pdM != m_adMat)) { smos_Free( m_pdM );  m_pdM = NULL; }
    if(m_ppdM && (m_ppdM != m_apRows)) { smos_Free( m_ppdM ); m_ppdM = NULL; }
  }

  // allocate new memory when needed
  if (lNumRows > 6 || lNumColumns > 6) 
    {
      // okay to use smos_Calloc on base(double, ptr,..) objects.
      m_pdM  = (double*) smos_Calloc(1,sizeof(double) *lNumRows*lNumColumns);
      m_ppdM = (double**)smos_Calloc(1,sizeof(double*)*lNumRows);
    }
  else 
    {
      m_pdM  = m_adMat;
      m_ppdM = m_apRows;
    }

  // set row pointers
  ULONG ii;
  for (ii=0; ii<lNumRows; ii++) 
    {
      m_ppdM[ii] = &m_pdM[ii*lNumColumns];
    }

  // set sizes
  m_lNumRows    = lNumRows ;
  m_lNumColumns = lNumColumns ;

} // end SmMatrix::SetSize

/*******************************************************************//**
PURPOSE: Utility: swap two rows.

NOTES: 
***********************************************************************/
SmStatus SmMatrix::SwapRows( ULONG lRow1, ULONG lRow2 )
{
  if ( lRow1 >= m_lNumRows || lRow2 >= m_lNumRows )
    { return SM_ERR; }

  if ( lRow1 == lRow2 )
    { return SM_SUCCESS; }

  // Just swap pointers in m_ppdM?  That would be fine if we could assume
  // that all access is through those pointers; probably not a safe assumption.
  // double *pTmpRowPtr = m_ppdM[ lRow1 ];
  // m_ppdM[ lRow1 ] = m_ppdM[ lRow2 ];
  // m_ppdM[ lRow2 ] = pTmpRowPtr;
  ULONG iCol;
  for ( iCol=0; iCol<m_lNumColumns; iCol++ )
    {
      double dTmp = m_ppdM[lRow1][iCol];
      m_ppdM[lRow1][iCol] = m_ppdM[lRow2][iCol];
      m_ppdM[lRow2][iCol] = dTmp;
    }

  return SM_SUCCESS;

} // end SmMatrix::SwapRows

/*******************************************************************//**
PURPOSE: return true when matrix is square and symmetric

NOTES:
***********************************************************************/
SmBoolean SmMatrix::IsSymmetric() const
{
  // return value
  SmBoolean bRtn = (   m_lNumRows == m_lNumColumns
                    && m_lNumRows > 0) ;

  if(bRtn)
    {
      // locals
      ULONG ii, jj ;

      // for every offdiagonal element
      for(ii=0;ii<m_lNumRows;ii++)
        {
          double *pRow = (*this)[ii] ;
           
          for(jj=ii+1;jj<m_lNumColumns;jj++)
            {
              // see if the element is symmetric
              if(!SM_IS_ZERO(pRow[jj] - GetAt(jj,ii)))
                {
                  bRtn = FALSE ;
                  break ;
                }
            } // end iter every upper diagonal row element

          // quit whenever a nonSymmetric element is found
          if(!bRtn)
            { break ; }
        } // end iter every row
    }

  // all done
  return (bRtn) ;    

} // end SmMatrix::IsSymmetric

/*******************************************************************//**
PURPOSE: return true when all elements are within tol of one another

NOTES:
***********************************************************************/
SmBoolean SmMatrix::IsEqual
 (const SmMatrix & crOther,   // in : target matrix for query
  double           dTol)      // in : max differance between equal elements, default:[1.0e-8]
 const
{
  // no work - same matrix
  if(this == &crOther)
    { return( TRUE ) ; }

  // no work - unequal sized matrices
  if(   m_lNumRows    != crOther.m_lNumRows
     || m_lNumColumns != crOther.m_lNumColumns) 
    { return( FALSE ) ; }

  // locals
  ULONG ii, lCnt = m_lNumRows * m_lNumColumns ;   
      
  for(ii=0;ii<lCnt;ii++)
    {   
      if(!SM_IS_ZERO_TO_TOL(m_pdM[ii] - crOther.m_pdM[ii], dTol)) 
        { return(FALSE) ; }
    }

  // all done
  return(TRUE) ; 

} // end SmMatrix::IsEqual

/*******************************************************************//**
PURPOSE: Set all matrix elements to zero.

NOTES: 
***********************************************************************/
SmStatus SmMatrix::Clear()
{
  // okay to use smos_MemSet on base (double and pointer) objects.
  smos_MemSet(m_pdM, 0 , m_lNumRows * m_lNumColumns * sizeof(double)) ;

  // for (i=0; i<m_lNumRows; i++) 
  //   {
  //     for (j=0; j<m_lNumRows; j++) 
  //       {
  //         (*this)[i][j] = 0.0;
  //       }
  //   }
  return SM_SUCCESS;

} // end SmMatrix::Clear

/*******************************************************************//**
PURPOSE: Make an NxN matrix into an identity matrix.

NOTES: 
***********************************************************************/
SmStatus SmMatrix::MakeIdentity()
{
  if (m_lNumRows != m_lNumColumns) 
    {
      SER(SM_ERR); // Requires square matrix to do inversion.
    }

  // init array to zero
  Clear() ;
  ULONG ii, iTgt ;

  // for every diagonal element
  for(ii=0, iTgt=0;ii<m_lNumRows;ii++,iTgt+=m_lNumRows+1)
    {
      m_pdM[iTgt] = 1.0 ; 
    }

  return SM_SUCCESS ;

  // ULONG ii, jj;
  // for (ii=0; ii<m_lNumRows; ii++) 
  //   {
  //     for (jj=0; jj<m_lNumRows; jj++) 
  //       {
  //         (*this)[ii][jj] = 0.0;
  //       }
  //     (*this)[ii][ii] = 1.0;
  //   }
  // return SM_SUCCESS;

} // end SmMatrix::MakeIdentity

/*******************************************************************//**
PURPOSE: Assignment operator

NOTES: Set this Matrix = crOther
***********************************************************************/
void SmMatrix::operator= (const SmMatrix &crOther)
{ 
  // no work
  if(this == &crOther)
    { return ; } 

  // locals
  ULONG lThisM  = GetNumRows() ;
  ULONG lThisN  = GetNumColumns() ;
  ULONG lOtherM = crOther.GetNumRows() ;
  ULONG lOtherN = crOther.GetNumColumns() ;

  // for unequally sized matrices
  if(   lThisM != lOtherM
     || lThisN != lOtherN)
    {
      // size M
      SetSize(lOtherM, lOtherN) ;

    } // end unequal sized matrices

  // copy all the elements
  // okay to use smos_MemCpy on base (double) objects.
  SE(smos_MemCpy((*this)[0], crOther[0], lOtherM*lOtherN * sizeof(double), lOtherM*lOtherN * sizeof(double)));

} // end SmMatrix::operator=

/*******************************************************************//**
PURPOSE: Matrix addition

NOTES: 
***********************************************************************/
SmStatus SmMatrix::Add
  (const SmMatrix & crOther,        // in : B of R = A + B
   SmMatrix       & rResult)        // out: R of R = A + B
  const
{
  const SmMatrix & rThis = *this;
  // First make sure all matrices have correct counts
  if (m_lNumColumns != crOther.m_lNumColumns) { SER(SM_ERR); }
  if (m_lNumRows    != crOther.m_lNumRows   ) { SER(SM_ERR); }
  if (m_lNumColumns != rResult.m_lNumColumns) { SER(SM_ERR); }
  if (m_lNumRows    != rResult.m_lNumRows   ) { SER(SM_ERR); }

  ULONG ii, iCnt = m_lNumRows * m_lNumColumns ;
  for (ii=0;ii<iCnt;ii++) 
    {
      rResult.m_pdM[ii] = rThis.m_pdM[ii] + crOther.m_pdM[ii] ;
    }

  return SM_SUCCESS;

} // end SmMatrix::Add

/*******************************************************************//**
PURPOSE: Matrix subtraction

NOTES: 
***********************************************************************/
SmStatus SmMatrix::Subtract
  (const SmMatrix & crOther,        // in : B of R = A + B
   SmMatrix       & rResult)        // out: R of R = A + B
  const
{
  const SmMatrix & rThis = *this;
  // First make sure all matrices have correct counts
  if (m_lNumColumns != crOther.m_lNumColumns) { SER(SM_ERR); }
  if (m_lNumRows    != crOther.m_lNumRows   ) { SER(SM_ERR); }
  if (m_lNumColumns != rResult.m_lNumColumns) { SER(SM_ERR); }
  if (m_lNumRows    != rResult.m_lNumRows   ) { SER(SM_ERR); }

  ULONG ii, iCnt = m_lNumRows * m_lNumColumns ;
  for (ii=0;ii<iCnt;ii++) 
    {
      rResult.m_pdM[ii] = rThis.m_pdM[ii] - crOther.m_pdM[ii] ;
    }

  return SM_SUCCESS;

} // end SmMatrix::Subtract

/*******************************************************************//**
PURPOSE: Matrix multiplication.

NOTES: rResult = 'this' * crOther
***********************************************************************/
SmStatus SmMatrix::Multiply
  (const SmMatrix & crOther,          // in : B of R = A * B
   SmMatrix       &  rResult)         // out: R of R = A * B
  const
{
  const SmMatrix & rThis = *this;
  SmMatrix       * pProd = &rResult ;
  SmTypedDelete<SmMatrix *> sClean ;

  // First make sure all matrices have correct counts
  if (m_lNumColumns         != crOther.m_lNumRows)    { SER(SM_ERR); }
  if (m_lNumRows            != rResult.m_lNumRows)    { SER(SM_ERR); }
  if (crOther.m_lNumColumns != rResult.m_lNumColumns) { SER(SM_ERR); }

  // extra work when rResult is the same object as 'this' or crOther object
  if(&rResult == this || &rResult == &crOther)
    {
      pProd = new SmMatrix(rResult.m_lNumRows, rResult.m_lNumColumns) ; 
      sClean.SetObj(pProd) ;
    }

  // init output to zero
  pProd->Clear() ;

  // do the multiply
  ULONG iRow, iCol, irc;
  for (iRow = 0; iRow<m_lNumRows; iRow++) 
    {
      double *pThisRow = rThis[iRow] ;

      for (irc=0; irc<m_lNumColumns; irc++) 
        {
           double *pOtherRow = crOther[irc] ;

           for (iCol=0; iCol<crOther.m_lNumColumns; iCol++) 
             {
               pProd->AddTo(iRow,iCol, pThisRow[irc] * pOtherRow[iCol]) ;
             }
        }
    }

  // extra work when rResult is the same object as 'this' or crOther object
  if(&rResult == this || &rResult == &crOther)
    {
      rResult = *pProd ;  
    }

  return SM_SUCCESS;

} // end SmMatrix::Multiply

/*******************************************************************//**
PURPOSE: Multiply a matrix by a vector and get the result as a vector.

NOTES:   R = M * V
***********************************************************************/
SmStatus SmMatrix::Multiply
  (const SmTArray<double> & rVector,      // in : v of R = M * V, sized:m_lNumColumns
   SmTArray<double>       & rResult)      // out: R of R = M * V
 const
{
  // First make sure all matrices have correct counts
  if (m_lNumColumns != rVector.GetSize())  { SER(SM_ERR); }
  if (m_lNumColumns != rResult.GetSize())  { rResult.SetSize(m_lNumRows) ; } 

  // locals
  ULONG ii, jj;
  const SmMatrix   & M = *this;
  SmTArray<double> * pProd = & rResult ;
  SmTArray<double>   sTmp ;

  // extra work when rResult is the same object rVector
  if(&rVector == &rResult)
    {
      sTmp.SetSize(m_lNumColumns) ;
      pProd = & sTmp ; 
    }

  // init result
  pProd->SetAll(0.0);
  double *pProdData = pProd->GetDataArray() ;

  for (ii=0; ii<this->GetNumRows(); ii++) 
    {
      double *pRow = M[ii] ;

      for (jj=0; jj<this->GetNumColumns(); jj++) 
        {
          pProdData[ii] += pRow[jj] * rVector[jj];
        }
    }    

  // extra work when rResult is the same object rVector
  if(&rVector == &rResult)
    {
      rResult = *pProd ;
    }

  return SM_SUCCESS;

} // end SmMatrix::Multiply

/*******************************************************************//**
PURPOSE: Multiply a matrix by a constant double value.

NOTES: 
***********************************************************************/
SmStatus SmMatrix::Multiply
  (double dValue)                   // in : d of A = d * A
{
  ULONG ii, lCnt = m_lNumRows * m_lNumColumns ;
  for (ii=0; ii<lCnt; ii++) 
    {
      m_pdM[ii] *= dValue ; 
    }    
  return SM_SUCCESS;

} // end SmMatrix::Multiply

/*******************************************************************//**
PURPOSE: Invert the matrix and store the result in the original.

NOTES: 
***********************************************************************/
SmStatus SmMatrix::Invert()
{
  if (m_lNumRows != m_lNumColumns) 
    {
      SER(SM_ERR); // Requires square matrix to do inversion.
    }

  SmMatrix sOrig(*this);

  double sDData[16];
  SmTArray<double> col(16,sDData);
  col.SetSize(m_lNumRows);
  ULONG laIndx[MAX_MATRIX_SIZE];

  if (sOrig.LUDecompose(laIndx) != SM_SUCCESS) 
    {
      return SM_ERR;
    }
  ULONG ii, jj, kk;
  for (jj=0; jj<m_lNumRows; jj++) 
    {
      for (ii=0; ii<m_lNumRows; ii++) 
        { col[ii] = 0.0; }
      col[jj] = 1.0;

      if (sOrig.SolveWithLUMatrix(laIndx,col,col) != SM_SUCCESS) 
        {
          return SM_ERR;
        }

      for (kk=0; kk<m_lNumRows; kk++) 
        {
          (*this)[kk][jj] = col[kk];
        }
    }

  return SM_SUCCESS;

} // end SmMatrix::Invert

/*******************************************************************//**
PURPOSE: M = Transpose(M)

NOTES: adjusts internal pointers for NonSquare matrices  
***********************************************************************/
SmStatus SmMatrix::Transpose()
{
  // locals
  ULONG ii, jj ;
  ULONG      m = GetNumRows() ;
  ULONG      n = GetNumColumns() ;
  SmMatrix &rM = *this ; 

  // for square matrices
  if(m == n)
    {
      for(ii=0;ii<m;ii++)
        {
          double *pRow = rM[ii] ;
          for(jj=ii+1;jj<m;jj++)
            {
              double dTemp = pRow[jj] ;
              pRow[jj]     = rM[jj][ii] ;
              rM[jj][ii]   = dTemp ;
            } // end iter every row
        } // end iter every col
    } // end square matrix branch
  else // matrix not square - requires memory changes
    {
      SmMatrix sMt(n, m) ;
      for(ii=0;ii<m;ii++)
        {
          double *pRow = rM[ii] ;
          for(jj=0;jj<n;jj++)
            {
              sMt[jj][ii] = pRow[jj] ; 
            } // end iter every row
        } // end iter every col
       
      rM = sMt ;
    }

  // all done
  return SM_SUCCESS;

} // end SmMatrix::Transpose

/*******************************************************************//**
PURPOSE: Do a Lower-Upper Decomposition of the matrix Utilizing 
  Crout's method as described in Numerical Recipies in C.

NOTES: 

RETURNS -- SM_ERR     = if matrix contains a zero row
           SM_ERR     = for singular matrices
           SM_SUCCESS = when matrix was decomposed in place
***********************************************************************/
SmStatus SmMatrix::LUDecompose
  (ULONG *plIndx)                 // out: Row Swapping index array
{
  if (GetNumRows() != GetNumColumns()) SER(SM_ERR_INVALID_INPUT);

  SmStatus sStatus = SM_SUCCESS;
  double dZeroTol  = SM_EFF_ZERO;  // Was SM_EFF_ZERO_SQ; that's too small. [bd 090330]
  double daScale[MAX_MATRIX_SIZE];


  double **M = m_ppdM;
  ULONG row, col, kk;

  // for every row - get 1.0/MaxTerm value - (return SM_ERR for any zero rows)
  for (row=0; row<GetNumRows(); row++) 
    {
      double dBig = 0.0;
      for (col=0; col<GetNumColumns(); col++) 
        {
          double dTmp = smos_Fabs(M[row][col]);
          if (dTmp > dBig) dBig = dTmp;
        }
      // check for bad rows
      if (dBig < dZeroTol )
        { return SM_ERR; }

      daScale[row] = 1.0/dBig;
    } // end iter every row getting max terms

  // for every col -
  for (col=0; col<GetNumColumns(); col++) 
    {
      // 
      for (row=0; row<col; row++) 
        {
          double dSum = M[row][col];
          for (kk=0; kk<row; kk++) 
            {
              dSum -= M[row][kk] * M[kk][col];
            }
          M[row][col] = dSum;
        }

      // search for largest pivot element
      double dBig = 0.0;  
      ULONG lMax = 0;
      for (row = col; row<GetNumRows(); row++) 
        {
          double dSum = M[row][col];
          for (kk=0; kk<col; kk++) 
            {
              dSum -= M[row][kk]*M[kk][col];
            }
          M[row][col] = dSum;

          double dDum = daScale[row] * smos_Fabs(dSum);
          if (dDum >= dBig) 
            {
              dBig = dDum;
              lMax = row;
            }
        }

      // See if we need to interchange rows
      if (col != lMax) 
        {
          for (kk=0; kk<GetNumRows(); kk++) 
            {
              double dTmp = M[lMax][kk];
              M[lMax][kk] = M[col][kk];
              M[col][kk]  = dTmp;
            }
          daScale[lMax] = daScale[col];
        }

      plIndx[col] = lMax;

      // In some cases we should return others we just
      // substitute SM_EFF_ZERO_SQ and continue.  For now do the
      // former and return warning.
      if (smos_Fabs(M[col][col]) < dZeroTol ) 
        {
          // Preserve the sign.  Sometimes this is just meaningless noise,
          // but often it's not, and if it's not meaningless,
          // then changing the sign will make Newton steps go backwards.
          M[col][col] = (M[col][col] > 0) ? SM_EFF_ZERO : -SM_EFF_ZERO;
          sStatus = SM_ERR_WARNING;
//          return SM_ERR;
        }

      // Scale rows 
      if (col != GetNumRows()-1) 
        {
          double dTmp = 1.0/M[col][col];
          for (kk=col+1;kk<GetNumColumns(); kk++) 
            {
              M[kk][col] *= dTmp;
            }
        }
    } // end iter every col

  // all done
  return sStatus;

} // end SmMatrix::LUDecompose

/*******************************************************************//**
PURPOSE: Solve the set of N linear equations using a LU decomposition
  matrix.

NOTES: 
***********************************************************************/
SmStatus SmMatrix::SolveWithLUMatrix
  (ULONG *laIndx,                              // in : row swapping index array
   const SmTArray<double> & crRightHandSide,   // in : b of Ax = b
   SmTArray<double> & rSolutionVector)         // out: x of Ax = b
  const
{
  double dZeroTol = SM_EFF_ZERO;  // Was SM_EFF_ZERO_SQ; that's too small. [bd 090330]

  // Now do forward substitution and backward elimination
  double **M = m_ppdM;

  ULONG lSizeRHS = crRightHandSide.GetSize();
  ULONG lNumRows = GetNumRows();

  for (ULONG ii=0; ii<lSizeRHS; ii++) 
    {
      rSolutionVector[ii] = crRightHandSide[ii];
    }

  // Forward substitution
  long mm=-1;
  for (ULONG row=0; row<lNumRows; row++) 
    {
      ULONG row_per = laIndx[row];
      // Unscramble row permutations
      double dSum = rSolutionVector[row_per];
      rSolutionVector[row_per] = rSolutionVector[row];
      if (mm >= 0) 
        {
          for (ULONG jj=mm; jj+1<=row; jj++)  // note: can't say row-1
            {
              dSum -= M[row][jj] * rSolutionVector[jj];
            }
        }
      else if (dSum != 0.0) 
        {
          mm = row;
        }
      rSolutionVector[row] = dSum;
    }

  // Backward elimination
  // Note when using ULONG must be careful about going below zero because
  // it never happens.
  for (ULONG jj=(long)lNumRows; jj>0; jj--) 
    {
      ULONG row = jj-1;
      double dSum = rSolutionVector[row];
      for (ULONG col=(ULONG)row+1; col<GetNumColumns(); col++) 
        {
          dSum -= M[row][col] * rSolutionVector[col];
        }
      double dDenom = M[row][row];
      if (smos_Fabs(dDenom) < dZeroTol ) 
        { return SM_ERR; }
      rSolutionVector[row] = dSum/dDenom;
    }

  // all done
  return SM_SUCCESS;

} // end SmMatrix::SolveWithLUMatrix

/*******************************************************************//**
PURPOSE: Solve a linear system of equations whose coefficients
  are defined by the matrix and whose right hand side is given.

NOTES: 
***********************************************************************/
SmStatus SmMatrix::SolveLinearSystem
  (const SmTArray<double> & crRightHandSide,     // in : b of Ax = b
   SmTArray<double>       & rSolutionVector)     // out: x of Ax = b
  const
{
  // check input
  if (GetNumRows()              != GetNumColumns()) SER(SM_ERR_INVALID_INPUT);
  if (crRightHandSide.GetSize() != GetNumRows())    SER(SM_ERR_INVALID_INPUT);

  // init output
  rSolutionVector.SetSize(crRightHandSide.GetSize());

  // copy matrix
  SmMatrix sTmp(*this);
  if (sTmp.GetNumRows() > MAX_MATRIX_SIZE) { SER(SM_ERR_INVALID_INPUT); }
  ULONG laIndx[MAX_MATRIX_SIZE];

  // Decompose the matrix
  SmStatus sStatus = sTmp.LUDecompose(laIndx);
  if(   sStatus != SM_SUCCESS 
     && sStatus != SM_ERR_WARNING) 
    {
      return SM_ERR;
    }

  // BackSubstitute for the error
  SER(sTmp.SolveWithLUMatrix(laIndx,crRightHandSide,rSolutionVector));

#ifdef SM_DEBUG_CODE
  // check err = Ax - b = 0
  ULONG ii, jj ;
  SmTArray<double> sTestB, sTestErr, sScaledZero ;
  sTestB.SetSize(crRightHandSide.GetSize());
  sTestErr.SetSize(crRightHandSide.GetSize());
  sScaledZero.SetSize(crRightHandSide.GetSize());

  // Get Err = Ax - b and a ScaledZero for each err term
  for(ii=0;ii<sTestB.GetSize();ii++)
    {
      sScaledZero[ii] = 0.0 ; 
      sTestB     [ii] = 0.0 ; 
      for(jj=0;jj<sTestB.GetSize();jj++)
        {
          double dTerm = GetAt(ii,jj) * rSolutionVector[jj] ;
          if(sScaledZero[ii] < smos_Fabs(dTerm)) sScaledZero[ii] = smos_Fabs(dTerm) ;
          sTestB[ii] += dTerm ;
        }
      sTestErr[ii]    = sTestB[ii] - crRightHandSide[ii] ;
      sScaledZero[ii] = SM_EFF_ZERO * (1.0 + sScaledZero[ii]) ;
    } // end Ax = b multiply

  // check the solution error
  for(ii=0;ii<crRightHandSide.GetSize();ii++) 
    { 
      double dDiff       = smos_Fabs(sTestErr[ii]) ;
      double dScaledZero = sScaledZero[ii] ;
      if(dDiff > dScaledZero)
        {
    // GWC:      SM_ASSERT_MSG(dDiff <= dScaledZero, _T("Bad Linear System Solution")) ;
        }
    }

SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      Dump();
      sTmp.Dump();
      rSolutionVector.Dump();
      crRightHandSide.Dump();
      sTestB.Dump();
    }

#endif

  // all done
  return sStatus;

} // end SmMatrix::SolveLinearSystem

/*******************************************************************//**
PURPOSE: Sort a given eigenvalue array and its associated given eigenvectors

NOTES: rtns SM_SUCCESS when rEigenValues.GetSize == rEigenVecs.GetNumRows
                       and rEigenVecs is a square matrix.
            SM_ERR     otherwise and does not touch the input

            This is an expensive sort routine - assuming small (less than 10 matrix sizes)
***********************************************************************/
SmStatus sm_SortEigenSystem
  (SmTArray<double> &rEigenValues,  // in : array of eigen values to sort in ascending order
   SmMatrix         &rEigenVecs)    // in : associated eigenVectors
{
  // locals
  ULONG i, j,       n = rEigenValues.GetSize() ;
  SmTArray<double> &d = rEigenValues ;
  SmMatrix         &v = rEigenVecs ; 
  
  // check input
  SER( (n == rEigenVecs.GetNumRows() && n == rEigenVecs.GetNumColumns()) ? SM_SUCCESS : SM_ERR) ;

  // for every eigenvalue
  for(i=0;i+1<n;i++) // note: can't say n-1
    {
      double dMin = d[i] ;
      ULONG  jMin = i ;

      // for every eigenvalue later in the list - find smallest element
      for(j=i+1;j<n;j++)
        {
          // remember when a next eigenValue is less than the current eigenValue
          if(d[j] < dMin) 
            { dMin = d[j] ; 
              jMin = j ;
            }
        }

      // when smallest remaining value is not the ith element - swap eigenvalues and eigenvectors
      if(dMin != d[i])
        {
          // swap eigenvalues
          double dTmp = d[jMin] ;
          d[jMin]     = d[i] ;
          d[i]        = dTmp ;

          // swap eigenvectors
          for(j=0;j<n;j++)
            {
              dTmp       = v[j][i] ;
              v[j][i]    = v[j][jMin] ;
              v[j][jMin] = dTmp ;
            }
        } // end found an unsorted eigenvector to swap
    } // end iter every eigenvalue

  // all done
  return(SM_SUCCESS) ;

} // end sm_SortEigenSystem

/*******************************************************************//**
PURPOSE: get eigenvalues and eigenvectors for symmetric real matrices

NOTES: 1. rtns SM_SUCCESS when matrix is square and symmteric on input
          rtns SM_ERR when matrix is not square and symmteric on input
       2. The algorithm is an expensive n**3, don't use this for
          larger matrices (let's say 10x10)  if you need that, you
          should type in a QR algorithm based on matrix tridiagonalization.
       3. This matrix is modified during the operation, but restored on output.

This algorithm was modified from the Numerical Recipes in C, jacobi method
in chapter 11. Eigensystems.
***********************************************************************/
SmStatus SmMatrix::EigenSystemJacobiSolver // eff: Find EigenVectors and Values for a symmetric real matrix     
  (SmTArray<double> & rEigenValues,        // out: EigenValues in ascending order
   SmMatrix         & rEigenVecs,          // out: EigenVectors: Col[ii] = associated EigenVector of EigenValue[ii]
   ULONG            & rIterCnt)            // out: number of iterations to achieve machine precision 
{
#define ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);a[k][l]=h+s*(g-h*tau);

  // check state
  if(!IsSymmetric())
    { return(SM_ERR) ; }

  // local names
  SmMatrix         & a = *this ;
  ULONG              n = GetNumRows() ;
  SmTArray<double> & d = rEigenValues ;
  SmMatrix         & v = rEigenVecs ;

  // locals 
  ULONG j, iq, ip, i ;
  double tresh, theta, tau, t, sm, s, h, g, c ;
  SmTArray<double> b(n,NULL,n) ;
  SmTArray<double> z(n,NULL,n) ;


  // init output
  d.SetSize(n) ;   d.SetAll(0) ;
  v.SetSize(n,n) ; v.MakeIdentity() ;
  rIterCnt = 0 ;

  // initialize initialize b and d to diagonal of a, and z = 0
  // z will accumulate terms of the form ta_pq as in equation (11.1.14) of Numerical Recipes in C.
  for(ip=0;ip<n;ip++)
    {
      b[ip] = a[ip][ip] ;
      d[ip] = a[ip][ip] ;
      z[ip] = 0.0 ; 
    }

  // for more than enough iterations to converge
  for(i=0;i<50;i++)
    {
      // sum upper diagonal elements (multiply by 2 to get all off diagonal terms - symmetric)
      for(sm=0.0,ip=0;ip+1<n;ip++) // note: can't say n-1
        {
          for(iq=ip+1;iq<n;iq++)
            {
              sm += smos_Fabs(a[ip][iq]) ;
            }
        }

      // EXIT POINT - normal return which relies on quadratic convergence 
      if(sm < SM_EFF_ZERO)
        { 
          // set output
          sm_SortEigenSystem(d, v) ;

          // restore A-matrix upper diagonal terms
          for(ip=0;ip+1<n;ip++) // note: can't say n-1
            {
              for(iq=ip+1;iq<n;iq++)
                {
                 a[ip][iq] = a[iq][ip] ;
                }
            }

#ifdef SM_DEBUG_CODE
          // check results [A - I*lambda[i]] * v[i] = 0
          SmMatrix dVTest(n,n) ;
          SmMatrix sMTest(n,n) ;

          double dMaxDimension = smos_Max(d.GetMaxValue(smgu_FabsDoubleLessThan), this->GetMaxDimension()) ;
          SmScaledZero dScaledZeroSqrt = smos_Sqrt(SmTol::GetScaledZero(dMaxDimension)) ;

          // for every eigenvalue
          for(ip=0;ip<n;ip++)
            {
              // build [A - I*lambda[ip]]
              sMTest.MakeIdentity() ;
              sMTest.Multiply(-d[ip]) ;
              sMTest.Add(*this,sMTest) ;

              // mulitply [A - I*lambda[ip]] * v[ip]
              SmTArray<double> sEigenV(n,NULL,n), sEigenTest(n,NULL,n) ;
              for(j=0;j<n;j++)
                { sEigenV[j] = v[j][ip] ; }

              sMTest.Multiply(sEigenV, sEigenTest) ;

              // check for zero
              double dSum = 0.0 ;
              // restore a matrix upper diagonal terms
              for(j=0;j<n;j++)
                { dSum += sEigenTest[j] ;
                }                              

              // if(dSum/n > SM_EFF_ZERO_SQRT * 10)
              //  { SM_ASSERT_MSG(dSum/n <= SM_EFF_ZERO_SQRT * 10, _T("EigenSystemJacobiSolver solution check failed")) ; }
              if ( smos_Fabs( dSum/10.0/n ) > dScaledZeroSqrt )
                { SM_ASSERT_MSG(SmTol::InTol(dSum/10.0/n, dScaledZeroSqrt), _T("EigenSystemJacobiSolver solution check failed")) ; }
            } // end iter ip, every eigenvector
#endif // SM_DEBUG_CODE

          // all done
          return SM_SUCCESS ;
        } // end all done check, off diagonal terms sum to zero with tolerance

      // for first 3 sweeps pick a nonzero threshold, after that set to 0.0
      tresh = (i < 3) ? 0.2*sm/(n*n) : 0.0 ;

      // iter every upper diagonal element looking for terms larger than threshold to rotate to 0.0
      for(ip=0;ip<n;ip++)
        {
          for(iq=ip+1;iq<n;iq++)
            {
              g = 100.0 * smos_Fabs(a[ip][iq]) ;

              // after four sweeps, skip the rotation if the off-diagonal element is small
              if(   i >= 3 
                 && SM_IS_ZERO((smos_Fabs(d[ip]) + g) - smos_Fabs(d[ip]))
                 && SM_IS_ZERO((smos_Fabs(d[iq]) + g) - smos_Fabs(d[iq])))
                {
                  // set off diagonal term to exactly 0.0
                  a[ip][iq] = 0.0 ;
                }

              else if(smos_Fabs(a[ip][iq]) > tresh)
                {
                  h = d[iq] - d[ip] ;

                  // set t for this iteration
                  if(SM_IS_ZERO((smos_Fabs(h) + g) - smos_Fabs(h)))
                    {
                      // t = 1/(2theta) for small off diagonal terms
                      t = (a[ip][iq]) / h ;
                    }
                  else
                    {
                      // numerical recipes in C Equation (11.1.10)
                      theta = h/2.0/(a[ip][iq]) ; 
                      t = 1.0 / (smos_Fabs(theta) + smos_Sqrt(1.0 + theta*theta)) ;
                      if(theta < 0.0)
                        {
                          t = -t ; 
                        }
                    } // end setting t
                  
                  //
                  c = 1.0/smos_Sqrt(1+t*t) ;
                  s=t*c ;
                  tau = s/(1.0+c) ;
                  h = t*a[ip][iq] ;
                  z[ip] -= h ;
                  z[iq] += h ;
                  d[ip] -= h ;
                  d[iq] += h ;
                  a[ip][iq] = 0.0 ;

                  // case of rotations 0 <= j < ip
                  for(j=0;j<ip;j++) 
                    {
                      ROTATE(a,j,ip,j,iq) ;
                    }
                  // case of rotations ip < j < iq
                  for(j=ip+1;j<iq;j++)
                    {
                      ROTATE(a,ip,j,j,iq) ;
                    }

                  // case of rotations iq < j < n
                  for(j=iq+1;j<n;j++)
                    {
                      ROTATE(a,ip,j,iq,j) ;
                    }

                  // rotate eigenVector matrix
                  for(j=0;j<n;j++)
                    {
                      ROTATE(v,j,ip,j,iq) ;
                    }

                  // count iterations
                  rIterCnt++ ;
                } // end found off diagonal element larger than threshold value check
            } // end iter iq - every upper diagonal term
        } // end iter ip - every row with upper diagonal terms

      // update d with sum of ta[p][q] and reinit z
      for(ip=0;ip<n;ip++)
        {
          b[ip] += z[ip] ;
          d[ip]  = b[ip] ;
          z[ip]  = 0.0 ;

        } // end iter all b, d, and z  elements

    } // end iter i looking for convergence

  // arrive here when jacobi rotation failed to diagonalize the a matrix
  SER_MSG(SM_ERR,_T("EigenSystemJacobiSolver: failed to converge - no eigen values or eigenvectors found")) ;

  // all done
  return SM_ERR ;

#undef ROTATE
} // end SmMatrix::EigenSystemJacobiSolver

/*******************************************************************//**
PURPOSE: Compute Determinant of a 3x3 matrix.

NOTES: 
***********************************************************************/
SmStatus SmMatrix::Compute3x3Determinant
  (double & rdDeterminant)                // out: matrix determinate 
  const
{
  if (m_lNumColumns != 3) SER(SM_ERR);
  if (m_lNumRows != 3) SER(SM_ERR);
  const SmMatrix & M = *this;
  rdDeterminant =   M[0][0]*M[1][1]*M[2][2]  -  M[0][0]*M[1][2]*M[2][1] 
                  - M[0][1]*M[1][0]*M[2][2]  +  M[0][1]*M[1][2]*M[2][0]
                  + M[0][2]*M[1][0]*M[2][1]  -  M[0][2]*M[1][1]*M[2][0] ; 
                    
  return SM_SUCCESS ;

} // end SmMatrix::Compute3x3Determinant

/*******************************************************************//**
PURPOSE: Compute EigenValues of a 3x3 matrix.  EigenValues are in
 ascending order.

NOTES: Only works for a 3x3 matrix.
***********************************************************************/
SmStatus SmMatrix::Compute3x3EigenValues
  (ULONG & rlNumEigenValues,                // out: number of real eigenvalues
   double adEigenValues[3])                 // out: Their values
  const
{
  rlNumEigenValues = 0;
  if (m_lNumColumns != 3) SER(SM_ERR);
  if (m_lNumRows != 3) SER(SM_ERR);
  SmMatrix M(*this);

  double dDeterminant = 0.0;
  SER(M.Compute3x3Determinant(dDeterminant));

  double adCoefficients[4];
  adCoefficients[3] = 1.0;  // Lambda^3
  adCoefficients[2] = - (M[0][0] + M[1][1] + M[2][2]);
  adCoefficients[1] =   M[2][2]*M[0][0] + M[2][2]*M[1][1] + M[0][0]*M[1][1]
                      - M[1][2]*M[2][1] - M[0][1]*M[1][0] - M[0][2]*M[2][0];
  adCoefficients[0] = - dDeterminant;

  double adSolutions[3];
  ULONG lNumSol;
  SER(SmPolynomial::SolveCubicEqn(adCoefficients,SM_EFF_ZERO,lNumSol,adSolutions));

  // save number of real solutions 
  rlNumEigenValues = lNumSol ;
          
  // set eigenvalues output in ascending order
  if      (lNumSol == 1) { adEigenValues[0] = adSolutions[0] ; }
  else if (lNumSol == 2)
    {
      if(adSolutions[0] <= adSolutions[1]) { adEigenValues[0] = adSolutions[0] ;
                                             adEigenValues[1] = adSolutions[1] ;
                                           }
      else                                 { adEigenValues[0] = adSolutions[1] ;
                                             adEigenValues[1] = adSolutions[0] ;
                                           }
    }
  else // 3 real solutions branch
    {
      if(adSolutions[0] <= adSolutions[1]) 
        {
          if (adSolutions[0] <= adSolutions[2]) { adEigenValues[0] = adSolutions[0] ;
                                                  adEigenValues[1] = smos_Min(adSolutions[1],adSolutions[2]) ;
                                                  adEigenValues[2] = smos_Max(adSolutions[1],adSolutions[2]) ;
                                                }
          else                                  { adEigenValues[0] = adSolutions[2] ;
                                                  adEigenValues[1] = adSolutions[0] ;
                                                  adEigenValues[2] = adSolutions[1] ;
                                                }
        }
      else // adSolutions[0] > adSolutions[1] branch
        { 
          if (adSolutions[1] <= adSolutions[2]) { adEigenValues[0] = adSolutions[1] ;
                                                  adEigenValues[1] = smos_Min(adSolutions[0],adSolutions[2]) ;
                                                  adEigenValues[2] = smos_Max(adSolutions[0],adSolutions[2]) ;
                                                }
          else                                  { adEigenValues[0] = adSolutions[2] ;
                                                  adEigenValues[1] = adSolutions[1] ;
                                                  adEigenValues[2] = adSolutions[0] ;
                                                }
        }
    } // end 3 real solutions branch

  // for now always check results, Det(Lambda_i * I - Mat) = 0
  ULONG ii;
  for (ii=0; ii<rlNumEigenValues; ii++) 
    {
      // build M2 = Lambda_i * I - Mat
      SmMatrix M2(*this);
      M2.MultiplyBy(-1.0);
      M2[0][0] += adEigenValues[ii];
      M2[1][1] += adEigenValues[ii];
      M2[2][2] += adEigenValues[ii];

      // get M2 determinate
      double dDet2 = 0.0;
      M2.Compute3x3Determinant(dDet2);

      // check det = 0
      if (smos_Fabs(dDet2) > SM_EFF_ZERO_SQRT) 
        {
          SE(SM_ERR);
        }
    } // end iter every real eigenvalue solution

  // all done
  return SM_SUCCESS;

} // end SmMatrix::Compute3x3EigenValues

/*******************************************************************//**
PURPOSE: Solve a linear system with all zeros as constants. 

NOTES: 
***********************************************************************/
SmStatus SmMatrix::SolveZeroLinearSystem
  (SmTArray<double> & rSolutions)         // out: x of Ax = 0
  const
{
  rSolutions.ReSet();
  if (m_lNumColumns != m_lNumRows) { SER(SM_ERR); }

//cbi264 TEMP: this is the new check I added.
//cbi264 You might want to disable it to fix the bug...
  // This will not work if the rank is not n-1.
  ULONG lRank = this->Rank();
  if ( lRank != m_lNumRows-1 )  // GWC: should also handle cases where rank is < n-1
    { return SM_ERR; }

  // Do this by choosing one of the variables to be a constant 1.0
  // moving it to the RHS and solving a smaller system.
  const SmMatrix & M = *this;
  SmMatrix S(m_lNumRows-1,m_lNumColumns-1);
  SmTArray<double> sRHS;
  SmTArray<double> sSolVec;
  ULONG i1, i2, jj, nn;

  // for every column - set sRHS = Reduced(-M.Column[i1]), without row i1
  //                    build S = Reduced(M,i1), without row or col i1
  for (i1=0; i1<m_lNumColumns; i1++) 
    {
      sRHS.ReSet();
      double dRHSTotal = 0.0;

      // for every let sRHS = -Col[i1]
      for (jj=0; jj<m_lNumRows; jj++) 
        { 
          // skip row == i1 (that row is being dropped from the eqn set)
          if (jj != i1) 
            {
              sRHS.Add(-M[jj][i1]);
              dRHSTotal += smos_Fabs(M[jj][i1]);
            }
        } // end iter every col building reduced sRHS

      // arrive here when sRHS = -M.Column[i1]
      
      // build S = Reduced(M,i1), without row or col iColS
      // for every column, 
      ULONG ic = 0; 
      for (i2=0; i2<m_lNumColumns; i2++) 
        {
          // skip column i1, that column is being dropped from eqns
          if (i2 == i1) continue;
          ULONG ir = 0;
          for (jj=0; jj<m_lNumRows; jj++) 
            {
              // skip row i1, that row is being dropped from eqns
              if (jj == i1) continue;
              S[ir][ic] = M[jj][i2];
              ir++;
            }
          ic++;
        } // end iter every row, col building Reduced(M)

//cbi264 TEMP:
//cbi: I don't think we should do this (call SolveZeroLinearSystem
//cbi  instead of SolveLinearSystem) if the RHS is zero):
//cbi: as per Bug 264, RHS is zero, and that's correct,
//cbi  the solution vector for this (x,z) should be (0,0).
//cbi  Let's go ahead and do it, but if it fails, don't quit,
//cbi  go on to the regular linear system solver.
//cbi264: But also check Bug 177.
      SmStatus eStat = SM_SUCCESS;

      // when Reduced(sRHS) is zero vector
      if (smos_Fabs(dRHSTotal) < SM_EFF_ZERO) 
        {
          // Pick a solution value for variable being eliminated from eqn set (any value will work, we choose 1)
          //  when M[i1][i1] != zero - the free variable must be zero
          double dCurrValue = (smos_Fabs(M[i1][i1]) > SM_EFF_ZERO) ? 0.0 : 1.0 ; 
          
          // ReducedMatrix size is 1x1
          if (S.GetNumRows() == 1) 
            {
              rSolutions.Add( dCurrValue ); // Undefined just give it a 1.0 or 0.0 [B177]
            }
          else // Reduced matrix size is bigger than 1x1 branch 
            {
              // try to solve the Reduced(M = 0) eqn set.  we know sRHS == 0.
              eStat = S.SolveZeroLinearSystem(sSolVec);
              if ( eStat == SM_SUCCESS )
                { rSolutions.Append(sSolVec); }
            }

          // arrive here when dCurrValue is set and we have a solution for the Reduced(M = 0) eqns

          // when call to SolveZeroLinearSystem() worked 
          if ( eStat == SM_SUCCESS )
            {
              // insert the eliminated variable back into the solution vector
              rSolutions.InsertAt(i1,dCurrValue) ;

              // all done
              return SM_SUCCESS;
            }
        } // end Reduced(sRHS) is zero check

      // arrive here when recursive use of SolveAeroLinearSystem() failed

      // try solving Reduced(M = sRHS) eqn set 
      if (S.SolveLinearSystem(sRHS,sSolVec) != SM_SUCCESS) 
        {
          continue;
        }

      // arrive here when solving Reduced(M = sRHS) eqn set succeeded

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          smos_WriteBuffer(_T("Matrix to zero solve\n"));
          Dump();
          smos_WriteBuffer(_T("Reduced Matrix and RHS\n"));
          S.Dump();
          sRHS.Dump();
        }   
#endif
      // check Reduced(solution vec) for large elements
      SmBoolean bSmallerThan1 = TRUE;
      for (nn=0; nn<sSolVec.GetSize(); nn++) 
        {
          if (smos_Fabs(sSolVec[nn]) > 2.0) 
            {
              bSmallerThan1 = FALSE;
            }
        }

      // when all Reduced(solution vec) elements are smallish
      if (bSmallerThan1) 
        {
          // build solution vec for original(Mv=0) eqns with a 1.0 value for removed variable
          //  GWC: should we use the same check as above for the inserted value, i.e.
          //       double dCurrValue = (smos_Fabs(M[i1][i1]) > SM_EFF_ZERO) ? 0.0 : 1.0 ;
          rSolutions.Append(sSolVec);
          rSolutions.InsertAt(i1,1.0);
          return SM_SUCCESS;
        }

    } // end iter i1 - trying a Reduced(Mv=0) for each element in v until a solution is found

  // arrive here when no solutions were found
  //   GWC: should this be SM_ERR because no solution was found?
  return SM_SUCCESS;

} // end SmMatrix::SolveZeroLinearSystem

/*******************************************************************//**
PURPOSE: Compute the Eigenvectors of a 3x3 matrix.  Eigenvectors
  will correspond to increasing values of EigenValues.

NOTES: 
***********************************************************************/
SmStatus SmMatrix::Compute3x3EigenVectors
  (ULONG     & rlNumEigenVectors,           // NotUsed: out: Number of real eigenvectors
   double      adEigenValues[3],            // out: eigen values
   SmVector3d  aEigenVectors[3])            // out: eigenvectors
  const
{
  SM_REF1(rlNumEigenVectors) ; 
  SmTArray<double> sEigenVals(3,adEigenValues,3) ;
  SmMatrix sV(3,3) ;
  ULONG sIterCnt ;

  ((SmMatrix *)this)->EigenSystemJacobiSolver(sEigenVals, sV, sIterCnt) ;

  // set output
  aEigenVectors[0].Set(sV[0][0], sV[1][0], sV[2][0]) ;
  aEigenVectors[1].Set(sV[0][1], sV[1][1], sV[2][1]) ;
  aEigenVectors[2].Set(sV[0][2], sV[1][2], sV[2][2]) ;

  // all done
  return(SM_SUCCESS) ;

//        rlNumEigenVectors = 0;
//        if (m_lNumColumns != 3) SER(SM_ERR);
//        if (m_lNumRows != 3) SER(SM_ERR);
//        // const SmMatrix & M = *this;
//      
//        double adEigenVals[3];
//        ULONG lNumEigenValues;
//        SER(Compute3x3EigenValues(lNumEigenValues,adEigenVals));
//        SmTArray<double> sSolutions;
//        ULONG ii;
//      
//        for (ii=0; ii<lNumEigenValues; ii++) 
//          {
//            SmMatrix S(*this);
//      
//            SER(S.MultiplyBy(-1.0));
//            S[0][0] = adEigenVals[ii] + S[0][0];
//            S[1][1] = adEigenVals[ii] + S[1][1];
//            S[2][2] = adEigenVals[ii] + S[2][2];
//      
//            SER(S.SolveZeroLinearSystem(sSolutions));
//      
//            aEigenVectors[ii].x = sSolutions[0];
//            aEigenVectors[ii].y = sSolutions[1];
//            aEigenVectors[ii].z = sSolutions[2];
//      
//            // Test results
//            SmVector3d sRes;
//            sRes.x = S[0][0]*aEigenVectors[ii].x + S[0][1]*aEigenVectors[ii].y + S[0][2]*aEigenVectors[ii].z;
//            sRes.y = S[1][0]*aEigenVectors[ii].x + S[1][1]*aEigenVectors[ii].y + S[1][2]*aEigenVectors[ii].z;
//            sRes.z = S[2][0]*aEigenVectors[ii].x + S[2][1]*aEigenVectors[ii].y + S[2][2]*aEigenVectors[ii].z;
//            if (sRes.Length() > SM_EFF_ZERO_SQRT) 
//              {
//                continue;
//              }
//            else 
//              {
//                adEigenValues[rlNumEigenVectors] = adEigenVals[ii];
//                aEigenVectors[rlNumEigenVectors] = aEigenVectors[ii];       
//                rlNumEigenVectors ++;
//              }
//          }
//      
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe) 
//          {
//            Dump();
//      
//          }
//      #endif
//      
//      
//        return SM_SUCCESS;

} // end SmMatrix::Compute3x3EigenVectors

/*******************************************************************//**
PURPOSE: Calculate the rank of this matrix.

NOTES:
***********************************************************************/
ULONG SmMatrix::Rank() const
{
  // Method: just call MakeRowEchelon(), which is the easiest way to calculate the rank.
  // But that modifies the matrix in place, and we don't want to do that, so make a copy.
  SmMatrix sTmpMat( *this );
  ULONG lRank = 0;
  SmStatus eStat = sTmpMat.MakeRowEchelon( lRank );

  return ( eStat == SM_SUCCESS ) ? lRank : 0;

} // end SmMatrix::Rank

/*******************************************************************//**
PURPOSE: Put this matrix into row-echelon form.

NOTES:
   Row-echelon form (REF), in general, means that a matrix has the shape
   which would result from Gaussian elimination on the rows: the leading
   coefficient of each row is strictly to the right of that of the row above.
   This implies that each entry in the column below the leading coefficient
   is zero.

   Sometimes it is also required that the leading coefficients are all 1.
   If that would help anything, it's easy enough to do.

   This is a relatively simple way to calculate the rank of a matrix.
***********************************************************************/
SmStatus SmMatrix::MakeRowEchelon( ULONG &rlRank, double dTol )
{
  // Limiting case, for recursion.
  if ( m_lNumRows == 1 && m_lNumColumns == 1 )
    {
      if ( smos_Fabs( GetAt( 0,0 ) ) > dTol )
        { rlRank++; }
      return SM_SUCCESS;
    }

  ULONG iRow, iCol;
  SmBoolean bFound = FALSE;

  // Find the first non-zero entry in the first column.
  for ( iRow = 0; iRow < m_lNumRows; iRow++ )
    {
      if ( smos_Fabs( m_ppdM[iRow][0] ) > dTol )
        {
          SwapRows( 0, iRow );
          bFound = TRUE;
          break;
        }
    }

  // Another check for end of recursion:
  if ( m_lNumColumns == 1 )
    {
      if ( bFound )
        {
          rlRank++;

          // Zero out the column below the first row.
          // (Already zero if not found.)
          for ( iRow=1; iRow<m_lNumRows; iRow++ )
            { m_ppdM[iRow][0] = 0; }
        }
      return SM_SUCCESS;
    }

  if ( bFound )
    {
      rlRank++;

      // Zero out the column below the first row.
      // Add to each row whatever multiple of the first row it takes
      // to zero out the first entry.
      for ( iRow=1; iRow<m_lNumRows; iRow++ )
        {
          double dMult = m_ppdM[iRow][0] / m_ppdM[0][0];
          m_ppdM[iRow][0] = 0;
          for ( iCol=1; iCol<m_lNumColumns; iCol++ )
            { m_ppdM[iRow][iCol] -= dMult * m_ppdM[0][iCol]; }
        }

      // Call recursively, leaving out 1st row and column.
      SmMatrix sSubMat( m_lNumRows-1, m_lNumColumns-1 );
      for ( iRow=1; iRow<m_lNumRows; iRow++ )
        {
          for ( iCol=1; iCol<m_lNumColumns; iCol++ )
            {
              sSubMat[iRow-1][iCol-1] = m_ppdM[iRow][iCol];
            }
        }

      SER( sSubMat.MakeRowEchelon( rlRank, dTol ));

      // Copy back into this.
      for ( iRow=1; iRow<m_lNumRows; iRow++ )
        {
          for ( iCol=1; iCol<m_lNumColumns; iCol++ )
            {
              m_ppdM[iRow][iCol] = sSubMat[iRow-1][iCol-1];
            }
        }
    }
  else
    {
      // A whole column of zeroes.  Do not increment the rank,
      // and make an n x m-1 submatrix.
      // Don't have to zero anything out.
      SmMatrix sSubMat( m_lNumRows, m_lNumColumns-1 );
      for ( iRow=0; iRow<m_lNumRows; iRow++ )
        {
          for ( iCol=1; iCol<m_lNumColumns; iCol++ )
            {
              sSubMat[iRow][iCol-1] = m_ppdM[iRow][iCol];
            }
        }

      SER( sSubMat.MakeRowEchelon( rlRank, dTol ));

      // Copy back into this.
      for ( iRow=0; iRow<m_lNumRows; iRow++ )
        {
          for ( iCol=1; iCol<m_lNumColumns; iCol++ )
            {
              m_ppdM[iRow][iCol] = sSubMat[iRow][iCol-1];
            }
        }
    }

  // all done
  return SM_SUCCESS;

} // end MakeRowEchelon

/*******************************************************************//**
PURPOSE: Put this matrix into SVD form.

NOTES: This math routine performs a single value decomposition on this 
  [mxn] matrix (m >=n), turning it into: A = U x W x T(V),  where 
  T(V) is the transpose of V. This A matrix is overwritten by U.

  Remember to copy this matrix prior to calling SVDecompose if access to the
  original A matrix is required later.
  
  See : "Numerical Recipes in C",  Chapter 2, for the original algorithm
     and the meaning of arrays: A, U, W, and V. 
    
  Indexing: 
  Original algorithm used 
     A[1..m][1..n], the diagonal of W as W[1..n], V[1..n][1..n].
  Current algorithm modified so that 
     A[0..m-1][0..n-1], W[0..n-1], V[0..n-1][0..n-1]

***********************************************************************/
SmStatus SmMatrix::SVDecompose // eff: decompose 'this' into U    of A[mxn] = U[mxn] X W[nxn] X Vt[nxn]
 (SmTArray<double> &rW,        // out: the diagonal of the diagonal matrix W
  SmMatrix         &rV)       // out: the V[nxn] matrix (not Vt) of A[mxn] = U[mxn] X W[nxn] X Vt[nxn]
                               // note: m >= n
{
  // locals
  SmMatrix &rA = *this ;
  ULONG      m = GetNumRows() ;
  ULONG      n = GetNumColumns() ;

  // init output
  rW.SetSize(n) ;
  rV.SetSize(n,n) ;

#ifdef SM_DEBUG_CODE
  // copy the input for later check
  SmMatrix sACopy(rA) ;
#endif // SM_DEBUG_CODE

  // locals
  int i_int, k_int, l_int ;
  ULONG flag, its, jj, kk, ll;
  ULONG i1, j1, lMaxIters = 30 ;
  double c, f, h, s ;
  double x, y, z ;
  double anorm = 0.0 ;
  double g     = 0.0 ;
  double scale = 0.0 ;
  SmTArray<double> rv1(n,NULL,n) ;

  // check input: m >= n
  if( m < n || m == 0 || n == 0)
    { return(SM_ERR) ; }

  // Householder reduction to bidiagonal form
  for(i1=0;i1<n;i1++)     // for(i1=0;i1<=n;i1++)
    {
      ll = i1 + 1;
      rv1[i1] = scale * g;
      g = s = scale = 0.0;

      if(i1<m)  // if(i1<=m)
        {
          for(kk=i1;kk<m;kk++)    // for(kk=i1;kk<=m;kk++)
            { scale += smos_Fabs( rA[kk][i1] ) ; }

          if( scale != 0.0 )
            {
              for(kk=i1;kk<m;kk++)  // for(kk=i1;kk<=m;kk++)
                {
                  rA[kk][i1] /= scale;
                  s += rA[kk][i1] * rA[kk][i1];
                }
                                                          
              f = rA[i1][i1];
              g = -( (f >= 0.0) ? smos_Sqrt(s) : -smos_Sqrt(s)) ; // -ST_SIGN( sqrt( s ), f );
              h = f * g - s;
              rA[i1][i1] = f - g;

              if( i1 != (n-1) )  // if( i1 != n )
                {
                  for(j1=ll;j1<n;j1++)   // for(j1=ll;j1<=n;j1++)
                    {
                      s = 0.0;

                      for(kk=i1;kk<m;kk++)  // for(kk=i1;kk<=m;kk++)
                          s += rA[kk][i1] * rA[kk][j1];
                      f = s / h;

                      for(kk=i1;kk<m;kk++) // for(kk=i1;kk<=m;kk++)
                          rA[kk][j1] += f * rA[kk][i1];
                    }
                }

              for(kk=i1;kk<m;kk++)   // for(kk=i1;kk<=m;kk++)
                  rA[kk][i1] *= scale;
            }
        }

      rW[i1] = scale * g;
      g = s = scale = 0.0;

      if( i1 < m && i1 != (n-1) )  // if( i1 < m && i1 != n )
        {
          for(kk=ll;kk<n;kk++)     // for(kk=ll;kk<=n;kk++)
            { scale += smos_Fabs( rA[i1][kk] ) ; }

          if( scale != 0.0 )
            {
              for(kk=ll;kk<n;kk++)  // for(kk=ll;kk<=n;kk++)
                {
                  rA[i1][kk] /= scale;
                  s += rA[i1][kk] * rA[i1][kk];
                }
                                          
              f = rA[i1][ll];
              g = -(f >= 0.0 ? smos_Sqrt(s) : -smos_Sqrt(s)) ;  // ST_SIGN( sqrt( s ), f );
              h = f * g - s;
              rA[i1][ll] = f - g;

              for(kk=ll;kk<n;kk++)  // for(kk=ll;kk<=n;kk++)
                { rv1[kk] = rA[i1][kk] / h ; }

              if(i1 != (m-1))  // if(i1 != m)
                {
                  for(j1=ll;j1<m;j1++)  // for(j1=ll;j1<=m;j1++)
                    {
                      s = 0.0;

                      for(kk=ll;kk<n;kk++)  // for(kk=ll;kk<=n;kk++)
                          s += rA[j1][kk] * rA[i1][kk];

                      for(kk=ll;kk<n;kk++)  // for(kk=ll;kk<=n;kk++)
                          rA[j1][kk] += s * rv1[kk];
                    }
                }

              for(kk=ll;kk<n;kk++)  // for(kk=ll;kk<=n;kk++)
                  rA[i1][kk] *= scale;
            }
        }

      anorm = smos_Max( anorm, (smos_Fabs( rW[i1] ) + smos_Fabs( rv1[i1] )) );
    }

  // Accumulation of right hand transformations 
  ll = 0 ; // init ll to any value to stop compiler warning - it'll get set to proper value prior to its next use
  for(i_int=(int)(n-1);i_int>=0;i_int--)   // for (i=n;i>=1;i--)  - fixed ULONG/int incompatibility
    {
      i1 = (ULONG)i_int ;
      if( i1 < (n-1) )  // if( i1 < n )
        {
          if( g != 0.0 )
            {
              for(j1=ll;j1<n;j1++)  // for(j1=ll;j1<=n;j1++)
                { rV[j1][i1] = (rA[i1][j1] / rA[i1][ll]) / g; }

              for(j1=ll;j1<n;j1++)  // for(j1=ll;j1<=n;j1++)
                {
                  s = 0.0;

                  for(kk=ll;kk<n;kk++)  // for(kk=ll;kk<=n;kk++)
                    { s += rA[i1][kk] * rV[kk][j1]; }

                  for(kk=ll;kk<n;kk++)  // for(kk=ll;kk<=n;kk++)
                    { rV[kk][j1] += s * rV[kk][i1]; }
                }
            }

          for(j1=ll;j1<n;j1++)  // for(j1=ll;j1<=n;j1++)
            { rV[i1][j1] = rV[j1][i1] = 0.0; }
        }

      rV[i1][i1] = 1.0;
      g = rv1[i1];
      ll = i1;
    }

  // Accumulation of left hand transformations 

  for(i_int=(int)(n-1);i_int>=0;i_int--)    // for(i=n;i>=1;i--) - fixed ULONG/int incompatibility
    {
      i1 = (ULONG)i_int ;
      ll = i1 + 1;
      g = rW[i1];

      if( i1 < (n-1) )  // if( i1 < n )
        {
          for(j1=ll;j1<n;j1++)  // for(j1=ll;j1<=n;j1++)
            { rA[i1][j1] = 0.0; }
        }

      if( g != 0.0 )
        {
          g = 1.0 / g;

          if( i1 != (n-1) )  // if( i1 != n )
            {
              for(j1=ll;j1<n;j1++)  // for(j1=ll;j1<=n;j1++)
                {
                  s = 0.0;

                  for(kk=ll;kk<m;kk++)  // for(kk=ll;kk<=m;kk++)
                    { s += rA[kk][i1] * rA[kk][j1]; }
                  f = (s / rA[i1][i1]) * g;

                  for(kk=i1;kk<m;kk++)  // for(kk=i1;kk<=m;kk++)
                    { rA[kk][j1] += f * rA[kk][i1]; }
                }
            }

          for(j1=i1;j1<m;j1++)  // for(j1=i1;j1<=m;j1++)
            { rA[j1][i1] *= g; }
        }
      else
        {
          for(j1=i1;j1<m;j1++)  // for(j1=i1;j1<=m;j1++)
            { rA[j1][i1] = 0.0; }
        }

      rA[i1][i1] += 1.0;
    }

  // Diagonalization of the bidiagonal form
  ULONG nm = 0;
  for(k_int=(int)(n-1);k_int>=0;k_int--)     // loop over singular values  // for(kk=n;kk>=1;kk--)  - fixed ULONG/int incompatibility     
    {
      kk = (ULONG)k_int ;
      for(its=0;its<lMaxIters;its++) // loop over allowed iterations   // for(its=1;its<=lMaxIters;its++)
        {
          flag = 1;

          for(l_int=k_int;l_int>=0;l_int--)  // for(ll=kk;ll>=1;ll--) // test for splitting 
            {
              ll = (ULONG)l_int ;
              nm = ll - 1;               // note that rv1[0] is always 0 

              if( smos_Fabs( rv1[ll] ) + anorm == anorm )
                {
                  flag = 0;
                  break;
                }

              if( smos_Fabs( rW[nm] ) + anorm == anorm )
                  break;
            }

          if( flag != 0 )
            { // cancellation of rv1[ll] if ll > 1
              c = 0.0;
              s = 1.0;

              for(i1=ll;i1<=kk;i1++)
                {
                  f = s * rv1[i1];

                  if( smos_Fabs( f ) + anorm != anorm )
                    {
                      g = rW[i1];
                      h = smos_Pythag(f,g) ;  // sqrt( f * f + g * g );
                      rW[i1] = h;
                      h = 1.0 / h;
                      c = g * h;
                      s = -f * h;

                      for(j1=0;j1<m;j1++)  // for(j1=1;j1<=m;j1++)
                        {
                          y = rA[j1][nm];
                          z = rA[j1][i1];
                          rA[j1][nm] = y * c + z * s;
                          rA[j1][i1] = z * c - y * s;
                        }
                    }
                }
            }

          z = rW[kk];

          if( ll == kk )
            {     // Convergence.  
              if( z < 0.0 )
                { // Singular value is made nonnegative  
                  rW[kk] = -z;

                  for(j1=0;j1<n;j1++)   // for(j1=1;j1<=n;j1++)
                      rV[j1][kk] = -rV[j1][kk];
                }
              break;
            }

          if( its == lMaxIters )
            { return(SM_ERR) ; }  //  error =  NL_CON_ERR 

          x = rW[ll];   // shift from bottom 2-by-2 minor 
          nm = kk - 1;
          y = rW[nm];
          g = rv1[nm];                               
          h = rv1[kk];                               
          f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 *h * y);
          g = sqrt( f * f + 1.0 );
          f = ((x - z) * (x + z) + h * ((y / (f + ( f >= 0.0 ? smos_Fabs(g) : -smos_Fabs(g)))) - h)) / x;

          c = s = 1.0; // next QR transformation: 

          for(j1=ll;j1<=nm;j1++)
            {
              i1 = j1 + 1;
              g = rv1[i1];
              y = rW[i1];
              h = s * g;
              g = c * g;
              z = smos_Pythag(f,h) ; // sqrt( f * f + h * h );
              rv1[j1] = z;
              c = f / z;
              s = h / z;
              f = x * c + g * s;
              g = g * c - x * s;
              h = y * s;
              y = y * c;

              for(jj=0;jj<n;jj++)   // for(jj=0;jj<=n;jj++)
                {
                  x = rV[jj][j1];
                  z = rV[jj][i1];
                  rV[jj][j1] = x * c + z * s;
                  rV[jj][i1] = z * c - x * s;
                }

              z = smos_Pythag(f,h) ;  // sqrt( f * f + h * h );
              rW[j1] = z; // rotation can be arbitrary if z=0 

              if( z != 0.0 )
                {
                  z = 1.0 / z;
                  c = f * z;
                  s = h * z;
                }

              f = (c * g) + (s * y);
              x = (c * y) - (s * g);

              for(jj=0;jj<m;jj++)     // for(jj=1;jj<=m;jj++)
                {
                  y = rA[jj][j1];
                  z = rA[jj][i1];
                  rA[jj][j1] = y * c + z * s;
                  rA[jj][i1] = z * c - y * s;
                }
            }

          rv1[ll] = 0.0;
          rv1[kk] = f;
          rW[kk] = x;
        }
    }

#ifdef SM_DEBUG_CODE
static constexpr int bDebugMe = 0;
  if ( bDebugMe )
    { 
      // check U*Ut = 1.0  and V*Vt = 1.0
      SmBoolean bCheck = 1 ;
      SmMatrix sUTest(n,n), sVTest(n,n), sI(n,n) ;
      SmMatrix sU  = rA ; 
      SmMatrix sUt = rA ; bCheck &= SM_SUCCESS == sUt.Transpose() ;
      SmMatrix sV  = rV ;
      SmMatrix sVt = rV ; bCheck &= SM_SUCCESS == sVt.Transpose() ;
      bCheck &= SM_SUCCESS == sUt.Multiply(sU,sUTest) ;
      bCheck &= SM_SUCCESS == sVt.Multiply(sV,sVTest) ;
      bCheck &= SM_SUCCESS == sI.MakeIdentity() ;
      bCheck &= sUTest.IsEqual(sI) ;
      bCheck &= sVTest.IsEqual(sI) ;
    
      // check A = U x W x Vt
      SmMatrix sACheck(m, n) ;
      SmMatrix sWVt(n, n) ;
      for(kk=0;kk<n;kk++)
        {
          for(ll=0;ll<n;ll++)
            {
              sWVt[kk][ll] = rW[kk] * rV[ll][kk] ; // rW * Transpose(V)
            }
        }
      bCheck &= SM_SUCCESS == rA.Multiply(sWVt, sACheck) ;

      bCheck &= sACheck.IsEqual(sACopy) ; // default tol = 1.0e-8
      SM_ASSERT_MSG(bCheck == TRUE, _T("SVDecompose failed decomposition")) ;

    }
#endif // SM_DEBUG_CODE

  // all done
  return (SM_SUCCESS);

} // end SmMatrix::SVDecompose

/******************************************************************//**
PURPOSE: Solve Ax=b when A has been decomposed into SVD matrices
                A = U.W.Vt
         See SmMatrix::SVDecompose()

NOTES: This  math routine  computes  the solution  of a system of  linear 
  equations.  It assumes the coefficient matrix has been  decomposed
  using SmMatrix::SVDecompose (Single Value Decomposition). 

  this is the [nxn] matrix U in                                 A = U x W x Vt
  W  is the diagonal elements of the [nxn] diagonal matrix W in A = U x W x Vt
  Vt is the [nxn] matrix in                                     A = U x W x Vt

  Ax=b  =>  Ut x (U x W x Vt) x = Ut x b             with Ut x U = 1.0
              InvW x (W X Vt) x = InvW * Ut * b      with InvW   = 1/W
                     V X (Vt) x = V * InvW * Ut * b  with V X Vt = 1.0
                              x = V * InvW * Ut * b

  See "Numerical Recipes in C", Chapter 2,
  for a description of the algorithm and the meaning of arrays: U,
  W and Vt.
***********************************************************************/
SmStatus SmMatrix::SolveWithSVDMatrices          // eff: Solve with SVD after call to SVDecompose()
 (const SmTArray<double> & crRightHandSide,      // in : b of Ax=b, sized:[m]
        SmTArray<double> &  rW,                  // i/o: diag of W of  A = U x W x Vt, sized:[n]
  const SmMatrix         & crV,                  // in : V of (not Vt) A = U x W x Vt, sized:[n,n]
        SmTArray<double> & rSolutionVector,      // out: x of Ax=b sized:[m]
  double                   dSVDTol,              // in : working limit for iterative solutions, default:[6.e-8]
  SmBoolean                bAdjSinglularValues)  // in : TRUE =set small values in rW to 0.0
                                                 //      FALSE=skip this step, only needed once, default:[TRUE]
 const
{
  // locals
  ULONG m = GetNumRows() ;
  ULONG n = GetNumColumns() ;

  // Check input, row count must be greater than or equal to column count 
  if( m < n || m == 0 || n == 0)
    { return (SM_ERR) ; }
  
  // check matrix sizes
  if(   rW.GetSize()        != n
     || crV.GetNumRows()    != n 
     || crV.GetNumColumns() != n 
     || crRightHandSide.GetSize() != m
     || rSolutionVector.GetSize() != m)
    { return (SM_ERR) ; }

  // locals
  ULONG  ii, jj;
  double dd ;
  const SmMatrix         & crU   = *this ;
  const SmTArray<double> &  rB   =  crRightHandSide ;
        SmTArray<double> &  rX   =  rSolutionVector ;
        SmTArray<double>    sTemp(n,NULL,n) ;

// NL_INDEX m ; 
// NL_INDEX n, 
// NL_VOID *crRHS, 
// NL_FLAG type, 
// NL_FLAG esvf, 
// NL_VOID *sol )

   // when asked - find and fix small values in W
   if( bAdjSinglularValues == TRUE )
     {
       double wmax = 0.0;

       for(ii=0;ii<n;ii++)
         { 
           if( rW[ii] > wmax ) { wmax = rW[ii]; }
         }

       if( wmax < dSVDTol )
         { return(SM_ERR) ; }

       dd = wmax * m * dSVDTol;

       for(ii=0;ii<n;ii++)
         {
           if( rW[ii] < dd ) { rW[ii] = 0.0; }
         }
     }

   // Now solve the system  
   
   // build sTemp = InvW x Ut x b 
   for(ii=0;ii<n;ii++)
     {
       dd = 0.0;

       // this check requries the above section for clearing singular values in W
       if( rW[ii] != 0.0 )
         {
           for(jj=0;jj<m;jj++)
             { dd += crU[jj][ii] * rB[jj]; }

           // could check for floating point divide errors here
           dd /= rW[ii];
         }
       sTemp[ii] = dd;
     }

   // build x = V x sTemp
   for(ii=0;ii<n;ii++)
     {
       rX[ii] = 0.0;
       double *pVRow = crV[ii] ;

       for(jj=0;jj<n;jj++)
         { rX[ii] += pVRow[jj] * sTemp[jj]; }
     }

#ifdef SM_DEBUG_CODE
static constexpr int bDebugMe = 0;
  if ( bDebugMe )
    { // check Ax = b

      // where A = U x W x Vt
      SmMatrix sACheck(m, n) ;
      SmMatrix sWVt(n, n) ;
      for(ii=0;ii<n;ii++)
        {
          for(jj=0;jj<n;jj++)
            {
              sWVt[ii][jj] = rW[ii] * crV[jj][ii] ;
            }
        }
      crU.Multiply(sWVt, sACheck) ;

      // Build Ax
      SmTArray<double> sAXCheck(m,NULL,m) ;
      sACheck.Multiply(rX, sAXCheck) ;

      SmBoolean bOK = TRUE ;
      for(ii=0;ii<m;ii++)  // GWC: this is not a good check because when m > n we expect a least squares rather than an exact solution
        { bOK &= (n != m) || SM_IS_ZERO_TO_TOL(sAXCheck[ii] - crRightHandSide[ii], dSVDTol) ; }
      SM_ASSERT_MSG(bOK == TRUE, _T("SolveWithSVDMatrices failed to solve properly")) ;

    }
#endif // SM_DEBUG_CODE

   // all done
   return (SM_SUCCESS);

} // end SmMatrix::SolveWithSVDMatrices

/*******************************************************************//**
PURPOSE: Pretty Print a matrix

NOTES:
***********************************************************************/
void SmMatrix::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,       _T("SmMatrix = 0x%p,  # Rows = %ld,  # Columns = %ld\n"),
             this,this->GetNumRows(),this->GetNumColumns());
  smos_sprintf(sBuffForFile,_T("SmMatrix = %s,  # Rows = %ld,  # Columns = %ld\n"),
             _T("notNULL"),this->GetNumRows(),this->GetNumColumns());
  smos_WriteBuffer(sBuffForFile);
  ULONG ii, jj;

  for (ii=0; ii<this->GetNumRows(); ii++) 
    {
      smos_sprintf(sBuff,_T("      [%ld] = "),ii);
      smos_WriteBuffer(sBuff);
      for (jj=0; jj<this->GetNumColumns(); jj++) 
        {
          smos_sprintf(sBuff,_T("%16.16lf   "),this->GetAt(ii,jj));
          smos_WriteBuffer(sBuff);
        }
      smos_WriteBuffer(_T("\n"));
    }

} // end SmMatrix::Dump

/*******************************************************************//**
PURPOSE: Unit Test for Matrix class

NOTES: Returns 1 for success, 0 for failure
***********************************************************************/
SmBoolean SmMatrix::UnitTest() 
{
  // init output
  SmBoolean bRtn = TRUE ; 

  // locals
  ULONG ii, jj ;

  { // begin simple tests
    // allocate a test matrix
    SmMatrix sMat3x3(3, 3) ;
    SmMatrix sMat5x7(5, 7) ;
    SmMatrix sMat5x7Copy(5, 7) ;
    SmMatrix sMat7x5(7, 5) ;
    bRtn &= (3 == sMat3x3.GetNumRows()) ;
    bRtn &= (5 == sMat5x7.GetNumRows()) ;
    bRtn &= (7 == sMat7x5.GetNumRows()) ;

    bRtn &= (3 == sMat3x3.GetNumColumns()) ;
    bRtn &= (7 == sMat5x7.GetNumColumns()) ;
    bRtn &= (5 == sMat7x5.GetNumColumns()) ;

    // 3x3 row vec constructor
    SmVector3d sX(1,0,0), sY(0,1,0), sZ(0,0,1) ;
    SmMatrix sMat3Vec(sX, sY, sZ) ;
    bRtn &= sMat3Vec.IsSymmetric() ;
  
    // copy constructor
    SmMatrix sMatCopy(sMat3Vec) ;
    bRtn &= sMatCopy.IsEqual(sMat3Vec, SM_EFF_ZERO) ;

    // init sMat3x3, Clear(), MakeIdentity(), operator=
    sMat3x3.SetAt(0,0, 1) ;
    sMat3x3.SetAt(0,1, 0) ;
    sMat3x3.SetAt(0,2, 0) ;
                         
    sMat3x3.SetAt(1,0, 0) ;
    sMat3x3.SetAt(1,1, 1) ;
    sMat3x3.SetAt(1,2, 0) ;
                         
    sMat3x3.SetAt(2,0, 0) ;
    sMat3x3.SetAt(2,1, 0) ;
    sMat3x3.SetAt(2,2, 1) ;
    bRtn &= sMat3x3.IsEqual(sMat3Vec, SM_EFF_ZERO) ;
    sMat3x3.Clear() ;
    for(ii=0;ii<9;ii++) { bRtn &= 0.0 == sMat3x3.Get(ii) ; }
    sMat3x3.MakeIdentity() ;
    for(ii=0;ii<3;ii++) 
      {
        for(jj=0;jj<3;jj++)
          { bRtn &=    (ii != jj && 0.0 == sMat3x3.GetAt(ii,jj))  
                    || (ii == jj && 1.0 == sMat3x3.GetAt(ii,jj)) ; 
          }
      }

    // operator=
    SmMatrix sMat3x3Copy(3, 3) ;
    sMat3x3Copy = sMat3x3 ; 
    bRtn &= sMat3x3.IsEqual(sMat3x3Copy, SM_EFF_ZERO) ;
  
    // simple access
    for(ii=0;ii<35;ii++) { ULONG lRow = ii / 7 ; 
                           ULONG lCol = ii % 7 ;

                           sMat5x7.Set(ii, ii) ;
                           sMat5x7Copy.SetAt(lRow,lCol, ii) ;

                           bRtn &= sMat5x7.GetAt(lRow,lCol) == (double) ii ; 
                           bRtn &= sMat5x7.Get(ii) == (double) ii ; 

                           double  * pRow  = sMat5x7[lRow] ;
                           double *& rpRow = sMat5x7[lRow] ;
                           bRtn &= pRow[lCol] == sMat5x7.GetAt(lRow,lCol) ;
                           bRtn &= pRow[lCol] == sMat5x7[lRow][lCol] ;

                           bRtn &= rpRow[lCol] == sMat5x7.GetAt(lRow,lCol) ;
                           bRtn &= rpRow[lCol] == sMat5x7[lRow][lCol] ;

                           sMat5x7.SetAt(lRow, lCol, 0) ;
                           sMat5x7.Add(ii, ii) ; 
                         
                           bRtn &= sMat5x7.GetAt(lRow,lCol) == (double) ii ; 
                           bRtn &= sMat5x7.Get(ii) == (double) ii ; 

                           sMat5x7.Set(ii, 0) ;
                           sMat5x7.AddTo(lRow,lCol, ii) ; 
                         
                           bRtn &= sMat5x7.GetAt(lRow,lCol) == (double) ii ; 
                           bRtn &= sMat5x7.Get(ii) == (double) ii ; 

                           sMat7x5.Set(ii, ii) ;
                        }

    bRtn &= sMat5x7.IsEqual(sMat5x7Copy, SM_EFF_ZERO) ;
    bRtn &= !(sMat5x7.IsSymmetric()) ;
    bRtn &= !(sMat5x7Copy.IsSymmetric()) ;
    bRtn &= !(sMat7x5.IsSymmetric()) ;

    // Matrix math
    SmTArray<double> sVec(3,NULL,3), sR(3,NULL,3) ;
    sVec.SetAt(0,1) ;
    sVec.SetAt(1,1) ;
    sVec.SetAt(2,1) ;

    // init some matrices
    SmMatrix sMat3x3Identity(3,3) ;
    bRtn &= SM_SUCCESS == sMat3x3Copy.MakeIdentity() ;
    bRtn &= SM_SUCCESS == sMat3x3Identity.MakeIdentity() ;

    for(ii=0;ii<9;ii++) { sMat3x3.Set(ii,ii) ; }

    bRtn &= SM_SUCCESS == sMat3x3.Add(sMat3x3, sMat3x3) ;
    for(ii=0;ii<9;ii++) { bRtn &= sMat3x3.Get(ii) == 2*ii ; }

    bRtn &= SM_SUCCESS == sMat3x3.Multiply(sMat3x3Copy, sMat3x3) ;
    for(ii=0;ii<9;ii++) { bRtn &= sMat3x3.Get(ii) == 2*ii ; }

    bRtn &= SM_SUCCESS == sMat3x3.Multiply(sVec, sR) ;
    for(ii=0;ii<3;ii++) { bRtn &= SM_IS_ZERO(sR[ii] - sMat3x3[ii][0] - sMat3x3[ii][1] - sMat3x3[ii][2]) ; } 

    bRtn &= SM_SUCCESS == sMat3x3.Multiply(3.0) ;
    for(ii=0;ii<9;ii++) { bRtn &= sMat3x3.Get(ii) == 6*ii ; }
  
    bRtn &= SM_SUCCESS == sMat3x3.Subtract(sMat3x3, sMat3x3) ;
    for(ii=0;ii<9;ii++) { bRtn &= sMat3x3.Get(ii) == 0.0 ; }

    sMat3x3[0][0] = 100 ;
    sMat3x3[0][1] = 2 ;
    sMat3x3[0][2] = 3 ;
  
    sMat3x3[1][0] = 1 ;
    sMat3x3[1][1] = 200 ;
    sMat3x3[1][2] = 3 ;
  
    sMat3x3[2][0] = 1 ;
    sMat3x3[2][1] = 2 ;
    sMat3x3[2][2] = 300 ;

    sMat3x3Copy   = sMat3x3 ;
    SmStatus sRtn = sMat3x3.Invert() ;
    if(sRtn == SM_SUCCESS) { sMat3x3.Multiply(sMat3x3Copy, sMat3x3Copy) ; 
                             bRtn &= sMat3x3Copy.IsEqual(sMat3x3Identity) ;
                           }
    else
      { bRtn = 0 ; }

  } // end simple test scope

  // Now the important part - test the solvers

  // need to add solver tests

  // SVD solver
  {
    SmMatrix sA(7,4), sACopy(7,4), sATest(7,4), sV(4,4), sWMat(4,4) ;
    SmTArray<double> sW(4,NULL,4) ; 

    sA[0][0] = 100 ;
    sA[0][1] =  2 ;
    sA[0][2] =  3 ;
    sA[0][3] =  4 ;

    sA[1][0] =  1 ;
    sA[1][1] = 200 ;
    sA[1][2] =  2 ;
    sA[1][3] =  3 ;

    sA[2][0] =  1 ;
    sA[2][1] =  2 ;
    sA[2][2] = 300 ;
    sA[2][3] =  4 ;

    sA[3][0] =  1 ;
    sA[3][1] =  2 ;
    sA[3][2] =  3 ;
    sA[3][3] = 400 ;

    sA[4][0] = 6 ;
    sA[4][1] = 15 ;
    sA[4][2] = 22 ;
    sA[4][3] = 30 ;

    sA[5][0] = 40 ;
    sA[5][1] = 30 ;
    sA[5][2] = 10 ;
    sA[5][3] = 20 ;

    sA[6][0] = 99 ;
    sA[6][1] = 12 ;
    sA[6][2] = 21 ;
    sA[6][3] = 99 ;

    sACopy = sA ;
    bRtn &= SM_SUCCESS == sA.SVDecompose(sW, sV) ;

    // check U*Ut = 1.0  and V*Vt = 1.0 
    SmMatrix sU = sA ; 
    SmMatrix sUt = sA ; bRtn &= SM_SUCCESS == sUt.Transpose() ;
    SmMatrix sVt = sV ; bRtn &= SM_SUCCESS == sVt.Transpose() ;
    SmMatrix sUTest(4,4), sVTest(4,4), sI4(4,4) ;
  
  
    bRtn &= SM_SUCCESS == sUt.Multiply(sU,sUTest) ;
    bRtn &= SM_SUCCESS == sVt.Multiply(sV,sVTest) ;
    bRtn &= SM_SUCCESS == sI4.MakeIdentity() ;
    bRtn &= sUTest.IsEqual(sI4) ;
    bRtn &= sVTest.IsEqual(sI4) ;

    // check   A = U X W X Vt
    sWMat.Clear() ;
    sWMat[0][0] = sW[0] ;
    sWMat[1][1] = sW[1] ;
    sWMat[2][2] = sW[2] ;
    sWMat[3][3] = sW[3] ;

    bRtn &= SM_SUCCESS == sWMat.Multiply(sVt,sWMat) ;
    bRtn &= SM_SUCCESS == sU.Multiply(sWMat,sATest) ;
    bRtn &= sACopy.IsEqual(sATest) ; 

    // check SVD solver on a square matrix
    // SVD solver
    sA.SetSize(4,4) ;
    sACopy.SetSize(4,4) ;
    SmTArray<double> sB(4,NULL,4), sX(4,NULL,4), sBTest(4,NULL,4) ;
    sA[0][0] = 100 ;
    sA[0][1] =  2 ;
    sA[0][2] =  3 ;
    sA[0][3] =  4 ;

    sA[1][0] =  1 ;
    sA[1][1] = 200 ;
    sA[1][2] =  2 ;
    sA[1][3] =  3 ;

    sA[2][0] =  1 ;
    sA[2][1] =  2 ;
    sA[2][2] = 300 ;
    sA[2][3] =  4 ;

    sA[3][0] =  1 ;
    sA[3][1] =  2 ;
    sA[3][2] =  3 ;
    sA[3][3] = 400 ;

    sB[0] = 1.0 ;
    sB[1] = 2.0 ;
    sB[2] = 3.0 ;
    sB[3] = 4.0 ;

    sACopy = sA ;
    bRtn &= SM_SUCCESS == sA.SVDecompose(sW, sV) ;
    bRtn &= SM_SUCCESS == sA.SolveWithSVDMatrices(sB, sW, sV, sX) ;

    // build Ax 
    for(ii=0;ii<4;ii++)
      {
        sBTest[ii] = 0.0 ; 
        for(jj=0;jj<4;jj++)
          {
            sBTest[ii] += sACopy[ii][jj] * sX[jj] ;
          }
      }

    // test Ax == b
    for(ii=0;ii<4;ii++)
      {
        bRtn &= SM_IS_ZERO_TO_TOL(sBTest[ii] - sB[ii], SM_EFF_ZERO) ;
      }
  } // end SVD tests

  // all done
  return(bRtn) ;

} // end SmMatrix::UnitTest()
