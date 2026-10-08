// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPolyDecimate.cpp
* PURPOSE: Implementation of Polygon decimater 
**********************************************************************/

#include "StdAfx.h"

#include <SmPolyDecimate.h>
#include <SmGraphicsExtern.h>
#include <SmGeomUtility.h>
#include <SmAttribute.h>
#include <SmPolySolver.h>
#include <SmSolutionArray.h>
#include <SmMatrix.h>
#include <SmTree.h>
#include <SmAssertArray.h>

//#define SM_VALIDATE_TOPOLOGY 1
//#define VALIDATE_QUADRIC 1

// class SmQuadricVertexNAttr was never used
//      /*******************************************************************//**
//      PURPOSE: Compute the total size of the memory used by SmQuadricVertexNAttr.
//      
//      NOTES:
//      ***********************************************************************/
//      ULONG SmQuadricVertexNAttr::GetMemoryUsed   // rtn: Total Memory being used by this object
//        (ULONG &rlMemoryAllocated)                // out: Total Memory allocated for this object
//       const
//      {
//        ULONG lThisAllocated ;
//      
//        // get base class allocation size - avoid counting contained objects twice
//        rlMemoryAllocated  = sizeof(this) ;
//        rlMemoryAllocated += m_A.GetMemoryUsed(lThisAllocated) - sizeof(SmMatrix) ;
//        rlMemoryAllocated += m_b.GetMemoryUsed(lThisAllocated) - sizeof(SmVectorNd) ;
//      
//        ULONG lUsed = m_vUsers.GetMemoryUsed(lThisAllocated) ;
//        rlMemoryAllocated += lThisAllocated - sizeof(SmTArray<SmAObject*>) ;
//      
//        // return used memory value
//        return(rlMemoryAllocated - lThisAllocated + lUsed) ; 
//      
//      } // end SmQuadricVertexNAttr::GetMemoryUsed
//      
//      /*******************************************************************//**
//      PURPOSE: Constructor for Quadric Vertex N attribute.
//      
//      NOTES: 
//      ***********************************************************************/
//      SmQuadricVertexNAttr::SmQuadricVertexNAttr
//       (ULONG lNumberValues)     // in : Size of the A matrix
//      : SmAttribute(SM_AI_QUADRIC_VERTEX_N_VALUES,SM_AB_COPY), 
//        m_A        (lNumberValues, lNumberValues), 
//        m_b        (6,m_dData)
//      {
//        m_b.SetSize(lNumberValues) ;
//      
//      } // end SmQuadricVertexNAttr::SmQuadricVertexNAttr constructor
//      
//      /*******************************************************************//**
//      PURPOSE: Add a weighted triangle with attributes to the quadric data.
//      
//      NOTES: The attributes should be normalized (between 0 and 1).
//      ***********************************************************************/
//      SmStatus SmQuadricVertexNAttr::AddWeightedTriangle
//       (SmPoint3d          sPnt1,
//        SmPoint3d          sPnt2,
//        SmPoint3d          sPnt3,
//        SmTArray<double> & crAttrPnt1,
//        SmTArray<double> & crAttrPnt2,
//        SmTArray<double> & crAttrPnt3)
//      {
//        // Make sure that the attributes all have the same size
//        if (   crAttrPnt1.GetSize() != crAttrPnt2.GetSize()
//          || crAttrPnt1.GetSize() != crAttrPnt3.GetSize())
//        {
//          SER(SM_ERR) ;
//        }
//        SM_VECND(sP1,16) ;
//        SM_VECND(sP2,16) ;
//        SM_VECND(sP3,16) ;
//      
//        sP1.SetValues(0,&sPnt1.x,3) ;
//        sP1.SetValues(3,crAttrPnt1.GetDataArray(),crAttrPnt1.GetSize()) ;
//        sP2.SetValues(0,&sPnt2.x,3) ;
//        sP2.SetValues(3,crAttrPnt2.GetDataArray(),crAttrPnt2.GetSize()) ;
//        sP3.SetValues(0,&sPnt3.x,3) ;
//        sP3.SetValues(3,crAttrPnt3.GetDataArray(),crAttrPnt3.GetSize()) ;
//      
//        //  Note area weighting may not be needed 
//        // double dWeight = dArea / 3.0 ;
//       
//        // Compute e1 and e2 ;
//        SM_VECND(sE1,16) ;
//        SM_VECND(sE2,16) ;
//      
//        sE1 = sP2-sP1 ;
//        if (sE1.LengthSquared() < SM_EFF_ZERO_SQ) {
//          SE(SM_ERR) ;
//          return SM_SUCCESS ;
//        }
//        SER(sE1.Unitize()) ;
//        SM_VECND(sK,16) ;
//        sK = sP3-sP1 ;
//      
//        // Compute e2 as an orthoginal vector to e1
//        SM_VECND(sVec,16) ;
//        sVec = (sE1.Dot(sK)) * sE1 ;
//        sVec = sK - sVec ;
//        double dDenom = sVec.Length() ;
//        if (dDenom < SM_EFF_ZERO) {
//          SE(SM_ERR) ;
//          return SM_SUCCESS ;
//        }
//        sE2 = sVec / dDenom ;
//      
//        double dPDotE1 = sP1.Dot(sE1) ;
//        double dPDotE2 = sP1.Dot(sE2) ;
//      
//        // Compute and add the constant c = p.p - (p.e1)^2 - (p.e2)^2
//        m_c += sP1.Dot(sP1) - dPDotE1*dPDotE1 - dPDotE1*dPDotE2 ;
//      
//        // Compute the vector b = (p.e1)e1 + (p.e2)e2 - p ;
//        m_b += sE1*dPDotE1 ;
//        m_b += sE2*dPDotE2 ;
//        m_b -= sP1 ;
//      
//        // Compute the matrix A = I - e1*e1^T - e2*e2^T
//        ULONG ii, jj, lNum = sE1.GetSize() ; 
//        for(ii=0 ;ii<lNum ;ii++){
//          for(jj=0 ;jj<lNum ;jj++){
//              if (ii==jj) {
//                  m_A[ii][jj] += 1.0 - sE1[ii]*sE1[jj] - sE2[ii]*sE2[jj] ;
//              }
//              else {
//                  m_A[ii][jj] += 0.0 - sE1[ii]*sE1[jj] - sE2[ii]*sE2[jj] ;
//              }
//          }
//        }
//      
//        return SM_SUCCESS ;
//      
//      } // end SmQuadricVertexNAttr::AddWeightedTriangle
//      
//      /*******************************************************************//**
//      PURPOSE: Combine two Quadric error forms into a single one as if
//         we are combining the two vertices.
//      
//      NOTES: This will work correctly even if the result the same
//         as one of the two input structures.
//      ***********************************************************************/
//      SmStatus SmQuadricVertexNAttr::Union
//       (const SmQuadricVertexNAttr & crOther,
//        SmQuadricVertexNAttr & rResult) 
//       const
//      {
//        SER(m_A.Add(crOther.m_A,rResult.m_A)) ;
//        rResult.m_b = m_b + crOther.m_b ;
//        rResult.m_c = m_c + crOther.m_c ;
//        return SM_SUCCESS ;
//      
//      } // end SmQuadricVertexNAttr::Union
//      
//      /*******************************************************************//**
//      PURPOSE: Compute the optimal possition of a new point given this
//        error matrix.
//      
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmQuadricVertexNAttr::ComputeOptimalPosition
//       (SmPoint3d        & rOptimal,
//        SmTArray<double> & rAttrOptimal,
//        SmBoolean        & rbFailsToCompute) const
//      {
//        SmMatrix sInv(m_A) ;
//        if (sInv.Invert() != SM_SUCCESS) {
//            rbFailsToCompute = TRUE ;
//            return SM_SUCCESS ;
//        }
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE ;
//        if (bDebugMe) {
//            SmMatrix sOrig(m_A) ;
//            SmMatrix sRes(m_A) ;
//            sOrig.Multiply(sInv,sRes) ;
//            sOrig.Dump() ;
//            sInv.Dump() ;
//            sRes.Dump() ;
//        }
//      #endif // SM_DEBUG_CODE
//      
//        double sOptimalData[16]  ;
//        SmTArray<double> sOptimal(16, sOptimalData) ;
//        SER( sInv.MultiplyVector( m_b.GetArray(), sOptimal )) ;
//      
//        rOptimal.x = - sOptimal[0] ;
//        rOptimal.y = - sOptimal[1] ;
//        rOptimal.z = - sOptimal[2] ;
//        rAttrOptimal.SetSize( sOptimal.GetSize()-3 ) ;
//        ULONG ii, lNum = sOptimal.GetSize() ;
//        for(ii=3 ;ii<lNum ;ii++){
//            rAttrOptimal[ii-3] = -sOptimal[ii] ;
//        }
//        rbFailsToCompute = FALSE ;
//        return SM_SUCCESS ;
//      
//      } // end SmQuadricVertexNAttr::ComputeOptimalPosition
//      
//      /*******************************************************************//**
//      PURPOSE: Compute the error.
//      
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmQuadricVertexNAttr::ComputeError
//       (const SmPoint3d        & crNewVertex,
//        const SmTArray<double> & crAttrVertex,
//        double & rdError) 
//       const
//      {
//        SM_VECND(sVec,16) ;
//        sVec.SetValues( 0, &crNewVertex.x, 3 ) ;
//        sVec.SetValues( 3, crAttrVertex.GetDataArray(), crAttrVertex.GetSize() ) ;
//      
//        rdError = sVec.Dot(m_b) + m_c ;
//      
//        return SM_SUCCESS ;
//      
//      } // end SmQuadricVertexNAttr::ComputeError
//      
//      /*******************************************************************//**
//      PURPOSE: Make a copy of this attribute in a virtual way such that
//        a general call can be made to copy all attributes.
//      
//      NOTES: 
//      ***********************************************************************/
//      SmAttribute * SmQuadricVertexNAttr::MakeCopy
//       (const SmContext & crContext) 
//       const
//      {
//        SmQuadricVertexNAttr *pRet = new (crContext) SmQuadricVertexNAttr(m_b.GetSize()) ;
//        NERN(pRet) ;
//        pRet->m_A = m_A ;
//        pRet->m_b = m_b ;
//        pRet->m_c = m_c ;
//        return pRet ;
//      
//      } // end SmQuadricVertexNAttr::MakeCopy
//
//      /*******************************************************************//**
//      PURPOSE: Dump routine for SmQuadricVertexNAttr
//      
//      NOTES: 
//      ***********************************************************************/
//      void SmQuadricVertexNAttr::Dump(void) const
//      {    
//        TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
//      
//        smos_sprintf(sBuff,       _T("SmQuadricVertexNAttr 0x%p - c = %16.16lf - b = "),this,m_c) ;
//        smos_sprintf(sBuffForFile,_T("SmQuadricVertexNAttr %s - c = %16.16lf - b = "),this ? _T("notNULL") : _T("NULL"),m_c) ;
//        smos_WriteBuffer(sBuff, sBuffForFile) ;
//      
//        m_b.Dump() ;
//      
//        smos_WriteBuffer(_T("\nA = ")) ;
//        m_A.Dump() ;
//      } // end SmQuadricVertexNAttr::Dump
//      
// end removed section because SmQuadricVertexNAttr was never used.

/*******************************************************************//**
PURPOSE: Add a weighted triangle to the quadric data.

NOTES: build SmQuadricVertexAttr member values as

  m_Q matrix += Area/3 * (Normal.Normalt),  Normal = [a b c], the triangle unit normal vector
  m_b vector += Area/3 * -(Normal . Center-Origin) * Normal
  m_c vector += Area/3 * (Center-Origin . Center-Origin)

***********************************************************************/
SmStatus SmQuadricVertexAttr::AddWeightedTriangle
 (const SmVector3d & crFaceNormal, // in : target triangle unit normal
  const SmPoint3d  & crCenter,     // in : target triangle center
  double             dArea)        // in : target triangle area
{
  double dWeight = dArea / 3.0 ;

  // triangle plane equation
  double        d = - crFaceNormal.Dot(crCenter) ;
  const double &a = crFaceNormal.x ;
  const double &b = crFaceNormal.y ;
  const double &c = crFaceNormal.z ;

  // Load the quadric error matrix
  // quadric error matrix = Sum(Area/3.0 N times N transpose) where N is the a,b,c of the plane equiation.
  m_Q[0].x += dWeight * a * a ;
  m_Q[0].y += dWeight * a * b ;
  m_Q[0].z += dWeight * a * c ;
  m_Q[1].x += dWeight * b * a ;
  m_Q[1].y += dWeight * b * b ;
  m_Q[1].z += dWeight * b * c ;
  m_Q[2].x += dWeight * c * a ;
  m_Q[2].y += dWeight * c * b ;
  m_Q[2].z += dWeight * c * c ;

  // b is d N - where d is d of plane equation and N is a,b,c of
  // plane equation.

  m_b += dWeight * d * crFaceNormal ;

  // The constant C is d*d ;
  m_c += dWeight * d * d ;

  // all done
  return SM_SUCCESS ;

} // end SmQuadricVertexAttr::AddWeightedTriangle

/*******************************************************************//**
PURPOSE: Combine two Quadric error forms into a single one as if
            we are combining the two vertices.  

NOTES: Sets the rResult values as
  rResult.m_Q = m_Q + Other.m_Q
  rResult.m_b = m_b + Other.m_b
  rResult.m_c = m_c + Other.m_c

  SmQuadricVertexAttr::Union() works correctly when 
  rResult is the same object as one of the input objects.
***********************************************************************/
SmStatus SmQuadricVertexAttr::Union
 (const SmQuadricVertexAttr & crOther, // in : other  of result.vals = this->vals + other.vals
  SmQuadricVertexAttr       & rResult) // out: result of result.vals = this->vals + other.vals
 const
{
  rResult.m_Q[0] = m_Q[0] + crOther.m_Q[0] ;
  rResult.m_Q[1] = m_Q[1] + crOther.m_Q[1] ;
  rResult.m_Q[2] = m_Q[2] + crOther.m_Q[2] ;
  rResult.m_b = m_b + crOther.m_b ;
  rResult.m_c = m_c + crOther.m_c ;
  return SM_SUCCESS ;

} // end SmQuadricVertexAttr::Union

/*******************************************************************//**
PURPOSE: Compute the optimal possition of a new point given the
                  current error matrix.

NOTES:  OptimalPt = -(Inv(m_Q) * m_b)

RETURNS --- the method always returns SM_SUCCESS
OUTPUT --- when rbFailsToCompute == TRUE  - there is no output
                                 == FALSE - output is good
***********************************************************************/
SmStatus SmQuadricVertexAttr::ComputeOptimalPosition
  (SmPoint3d & rOptimal,          // out: computed optimal point = -(Inv(m_Q) * m_b) 
   SmBoolean & rbFailsToCompute)  // out: TRUE = m_Q is not invertible - no result
                                  //      FALSE= result is good
 const
{
  // invert quadratic error matrix m_Q
  SmMatrix sInv(m_Q[0],m_Q[1],m_Q[2]) ;
  if (sInv.Invert() != SM_SUCCESS) 
    {
      rbFailsToCompute = TRUE ;
      return SM_SUCCESS ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if (bDebugMe) 
    {
      SmMatrix sOrig(m_Q[0],m_Q[1],m_Q[2]) ;
      SmMatrix sRes(3,3) ;
      sOrig.Multiply(sInv,sRes) ;
      sOrig.Dump() ;
      sInv.Dump() ;
      sRes.Dump() ;
    }
#endif // SM_DEBUG_CODE

  // build optimal point = -(Inv(m_Q) * m_b)
  rOptimal.x = sInv[0][0] * m_b.x + sInv[0][1] * m_b.y + sInv[0][2] * m_b.z ;
  rOptimal.y = sInv[1][0] * m_b.x + sInv[1][1] * m_b.y + sInv[1][2] * m_b.z ;
  rOptimal.z = sInv[2][0] * m_b.x + sInv[2][1] * m_b.y + sInv[2][2] * m_b.z ;

  rOptimal         = - rOptimal ;
  rbFailsToCompute = FALSE ;

  // all done
  return SM_SUCCESS ;

} // end SmQuadricVertexAttr::ComputeOptimalPosition

/*******************************************************************//**
PURPOSE: Compute error for new vertex from current QuadricVertex data
            values.

NOTES: error = NewVert . m_b + m_c
***********************************************************************/
SmStatus SmQuadricVertexAttr::ComputeError
 (const SmPoint3d & crNewVertex,  // in : target vertex
  double          & rdError)      // out: error = NewVert . m_b + m_c
 const
{
  rdError = crNewVertex.Dot(m_b) + m_c ;

  return SM_SUCCESS ;

} // end SmQuadricVertexAttr::ComputeError

/*******************************************************************//**
PURPOSE: Make a copy of this attribute in a virtual way such that
  a general call can be made to copy all attributes.

NOTES: 
***********************************************************************/
SmAttribute * SmQuadricVertexAttr::MakeCopy
 (const SmContext & crContext) 
 const
{
  SmQuadricVertexAttr *pRet = new (crContext) SmQuadricVertexAttr(m_lAttributeID,m_eBehavior) ;
  pRet->m_Q[0] = m_Q[0] ;
  pRet->m_Q[1] = m_Q[1] ;
  pRet->m_Q[2] = m_Q[2] ;
  pRet->m_b = m_b ;
  pRet->m_c = m_c ;
  return pRet ;

} // end SmQuadricVertexAttr::MakeCopy

/*******************************************************************//**
PURPOSE: Make a copy of this attribute in a virtual way such that
  a general call can be made to copy all attributes.

NOTES: 
***********************************************************************/
SmAttribute * SmQuadricEdgeAttr::MakeCopy
 (const SmContext & crContext) 
 const
{
  SmQuadricEdgeAttr *pRet = new (crContext) SmQuadricEdgeAttr(m_lAttributeID,m_eBehavior) ;
  pRet->m_lSortIndex = 0 ;
  pRet->m_dError     = m_dError ;
  pRet->m_vNewPoint  = m_vNewPoint ;

  return pRet ;

} // end SmQuadricEdgeAttr::MakeCopy

/*******************************************************************//**
PURPOSE: Make a copy of this attribute in a virtual way such that
  a general call can be made to copy all attributes.

NOTES: 
***********************************************************************/
SmAttribute * SmDecimateVertexAttr::MakeCopy
 (const SmContext & crContext) 
 const
{
  SmDecimateVertexAttr *pRet  = new (crContext) SmDecimateVertexAttr(m_lAttributeID,m_eBehavior) ;
  pRet->m_dMaxFaceError       = m_dMaxFaceError ;
  pRet->m_dVertexDistance     = m_dVertexDistance ;
  pRet->m_eVertClass          = m_eVertClass ;
  pRet->m_vAveragePlaneNormal = m_vAveragePlaneNormal ;

  return pRet ;

} // end SmDecimateVertexAttr::MakeCopy

/*******************************************************************//**
PURPOSE: Dump routine for SmDecimateVertexAttr

NOTES: 
***********************************************************************/
void SmDecimateVertexAttr::Dump(void) const
{    
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff,       _T("SmDecimateVertexAttr 0x%p - %d, %ld, %16.16lf, %16.16lf\n"),
      this,m_eVertClass,m_lSortIndex,m_dVertexDistance,m_dMaxFaceError) ;
  smos_sprintf(sBuffForFile,_T("SmDecimateVertexAttr %s - %d, %ld, %16.16lf, %16.16lf\n"),
      _T("notNULL"),m_eVertClass,m_lSortIndex,m_dVertexDistance,m_dMaxFaceError) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

} // end SmDecimateVertexAttr::Dump

/*******************************************************************//**
PURPOSE: Dump routine for SmQuadricEdgeAttr

NOTES: 
***********************************************************************/
void SmQuadricEdgeAttr::Dump(void) const
{    
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff,       _T("SmQuadricEdgeAttr 0x%p - %ld, %16.16lf\n"),
      this,m_lSortIndex,m_dError) ;
  smos_sprintf(sBuffForFile,_T("SmQuadricEdgeAttr %s - %ld, %16.16lf\n"),
      _T("notNULL"),m_lSortIndex,m_dError) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

} // end SmQuadricEdgeAttr::Dump

/*******************************************************************//**
PURPOSE: Dump routine for SmQuadricVertexAttr

NOTES: 
***********************************************************************/
void SmQuadricVertexAttr::Dump(void) const
{    
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff,       _T("SmQuadricVertexAttr 0x%p - c = %16.16lf - b = %16.16lf, %16.16lf, %16.16lf\n"),
      this,m_c,m_b.x,m_b.y,m_b.z) ;
  smos_sprintf(sBuffForFile,_T("SmQuadricVertexAttr %s - c = %16.16lf - b = %16.16lf, %16.16lf, %16.16lf\n"),
      _T("notNULL"), m_c,m_b.x,m_b.y,m_b.z) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  smos_WriteBuffer(_T("Q = ")) ;
  m_Q[0].Dump() ;

  smos_WriteBuffer(_T("\n    ")) ;
  m_Q[1].Dump() ;

  smos_WriteBuffer(_T("\n    ")) ;
  m_Q[2].Dump() ;

  smos_WriteBuffer(_T("\n\n")) ;

} // end SmQuadricVertexAttr::Dump

/*******************************************************************//**
PURPOSE: Constructor for SmPolyDecimate

NOTES: 
***********************************************************************/
SmPolyDecimate::SmPolyDecimate
  (SmPolyBrep * pPolyBrep,
   double       dPercentOfReduction,
   double       dMaximumError,
   SmBoolean    bUseEdgeLengthAsError,
   double       dMinFeatureAngDeg,
   double       dInteriorEdgeWeight,
   double       dBoundaryEdgeWeight)
: m_pPolyBrep            (pPolyBrep),
  m_dPercentOfReduction  (dPercentOfReduction),
  m_dMaximumError        (dMaximumError),
  m_dMinFeatureAngle     (dMinFeatureAngDeg),
  m_dInteriorEdgeWeight  (dInteriorEdgeWeight),
  m_dBoundaryEdgeWeight  (dBoundaryEdgeWeight),
  m_bUseEdgeLengthAsError(bUseEdgeLengthAsError),
  m_pEyeOrViewVector     (NULL), 
  m_bParallelProjection  (TRUE)
{ 

} // end SmPolyDecimate::SmPolyDecimate constructor

/*******************************************************************//**
PURPOSE: Determine the classification of SmPolyVertex for Decimation

NOTES: 
***********************************************************************/
SmPolyVertexClass SmPolyDecimate::FindVertexClass
 (SmPolyVertex          * cpPolyVertex,             // in : target vertex to classify
  SmTArray<SmPolyEdge*> * pFeatureOrBoundaryEdges,  // out: opt list of lamina/feature/silhouette edges connected to vertex.
                                                    //      NULL to ignore, default:[NULL]
  SmBoolean             * pbNewDVAttr)              // out: TRUE if a new DecimateVertexAttr was create for this PolyVertex
{
  // init output
  if (pFeatureOrBoundaryEdges) 
    { pFeatureOrBoundaryEdges->ReSet() ; }
  SmBoolean bNewDVAttr = FALSE;

  // get Vertex->SM_AI_VERTEX_DECIMATION_VALUES Attribute - make one when needed
  SmDecimateVertexAttr *pAttr = (SmDecimateVertexAttr*)((cpPolyVertex->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES))) ;
  if (pAttr == NULL) 
    {
      pAttr = new (*cpPolyVertex->GetContext()) SmDecimateVertexAttr(SM_AI_VERTEX_DECIMATION_VALUES, 
                                                                     SM_AB_STANDALONE_COPY) ;
      cpPolyVertex->AddAttribute(pAttr) ;    
      bNewDVAttr = TRUE;
    }
  if (pbNewDVAttr) *pbNewDVAttr = bNewDVAttr;

  // when m_eVertClass already computed and not looking for feature/boundary edges
  if(   pAttr->m_eVertClass != SM_PVC_NOT_COMPUTED 
     && pFeatureOrBoundaryEdges == NULL) 
    {
      // return cached classification
      return pAttr->m_eVertClass ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if (bDebugMe) 
    {
      smgfx_SetLook(2,8, 1,0,0) ; cpPolyVertex->GetPoint().Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // arrive here when we have to calculate m_eVertClass

  // locals
  ULONG ii ;
  SmPolyVertexClass eVertClass      = SM_PVC_UNKNOWN ;
  SmBoolean         bVertOnBoundary = FALSE ;   // becomes TRUE if vert is connected to lamina edge
  SmBoolean         bDone           = FALSE ;
  SM_PTR_ARRAY(sFeatureOrBoundaryEdges,SmPolyEdge,64) ; // SmTArray<SmPolyEdge*>
  SM_PTR_ARRAY(sCoinVEdges,SmPolyEdge,64) ;

  // get all polyEdges connected to polyVertex
  cpPolyVertex->GetStartingPolyEdges(sCoinVEdges) ;

  // for every polyEdge connected to polyVertex - 
  ULONG lNumCoin = sCoinVEdges.GetSize() ;
  for(ii=0 ;ii<lNumCoin ;ii++)
    {
      SmPolyEdge *pEdge = sCoinVEdges[ii] ;

      // if lamina, put edge in list and mark bVertOnBouneady
      if (pEdge->IsLamina()) 
        {
          sFeatureOrBoundaryEdges.Add(pEdge) ;
          bVertOnBoundary     = TRUE ;
          SmPolyEdge *pCWEdge = pEdge->GetCWPolyEdge() ;

          // check the edge that ends at Vertex as well
          if (pCWEdge->IsLamina()) 
            {
              sFeatureOrBoundaryEdges.Add(pCWEdge) ;
            }
        }
      // spine edges set eVertClass to SM_PVC_UNKNOWN
      else if (!pEdge->IsManifold()) 
        {
          eVertClass = SM_PVC_UNKNOWN ;
          bDone = TRUE ;
        }
      // edge is manifold
      else 
        {
          // check edge that ends on this vertex
          SmPolyEdge *pCWEdge = pEdge->GetCWPolyEdge() ;

          // if loop neighbor edge is lamina edges - Vert is on boundary
          if (pCWEdge->IsLamina()) 
            {
              sFeatureOrBoundaryEdges.Add(pCWEdge) ;
              bVertOnBoundary = TRUE ;
            }

          // else save feature edges - dihedral angle larger than m_dMinFeatureAngle
          else if (TestFeatureEdge(pEdge) == TRUE) 
            {
              sFeatureOrBoundaryEdges.Add(pEdge) ;
            }

          // else save silhouette edges - attached face normals lie on opposite sides of view vector
          //  GWC??? do we need to check for silhouette edges first to get bVertOnBoundary value?
          else if (TestSilhouetteEdge(pEdge) == TRUE) 
            {
              sFeatureOrBoundaryEdges.Add(pCWEdge) ;
              bVertOnBoundary = TRUE ;
            }

        } // end edge is manifold branch
    } // end iter every vertex->polyEdge

  // set output
  if (pFeatureOrBoundaryEdges) 
    {
      pFeatureOrBoundaryEdges->Append(sFeatureOrBoundaryEdges) ;
    }

  // check boundary verts for number of feature edges
  if (!bDone && bVertOnBoundary) 
    {
      // GWC: this seems wrong - a vertex at a corner of a box could be
      //      connected to 3 feature edges.
      if (sFeatureOrBoundaryEdges.GetSize() != 2) 
        {
          eVertClass = SM_PVC_UNKNOWN ; // Possibly non-manifold corner cases
          bDone = TRUE ;
        }
      else // vert really is on a boundary
        {
          eVertClass = SM_PVC_BOUNDARY ;
          bDone = TRUE ;
        }
    }

  // for unclassified vertices: classify based on number of Feature/boundary edges
  if (!bDone) 
    {
      ULONG lTotalFeatureEdges = sFeatureOrBoundaryEdges.GetSize() ;
      switch (lTotalFeatureEdges) 
        {
          // no feature edges = simple
          case 0 : 
          case 1 : eVertClass = SM_PVC_SIMPLE ;
                   break ;

          // 2 edges - Unknown or interior
          case 2 : if (sCoinVEdges.GetSize() == 2) 
                     {
                       eVertClass = SM_PVC_UNKNOWN ;
                     }
                   else 
                     {
                       eVertClass = SM_PVC_INTERIOR ;
                     }
                   break ; 

          // everythig else - corner
          default: eVertClass = SM_PVC_CORNER ;
                   break ;
        } // end swith on feature/boundary edge count
    } // end yet to classify branch

  // save the classification
  pAttr->m_eVertClass = eVertClass ;

  // all done
  return eVertClass ;

} // end SmPolyDecimate::FindVertexClass

/*******************************************************************//**
PURPOSE: Return TRUE when edge is a feature edge. 
  
NOTES: An edge is a feature when
  1. it's lamina
  2. its dihedral angle is larger than m_dMinFeatureAngle
***********************************************************************/
SmBoolean SmPolyDecimate::TestFeatureEdge
 (const SmPolyEdge * cpPolyEdge)
{
  // low work - lamina edges are features
  SmPolyEdge * pRadialEdge = cpPolyEdge->GetRadial() ;
  if ( pRadialEdge == NULL )
    { return TRUE ; }

  // locals
  SmVector3d sNormal, sNormal2 ;

  // This face->Normal
  SmPolyFace * pThisFace = cpPolyEdge->GetPolyFace() ; NER( pThisFace ) ;
  sNormal  = pThisFace->GetNormal() ;

  // Radial face->Normal
  SmPolyFace *pThatFace = pRadialEdge->GetPolyFace() ; NER( pThatFace ) ;
  sNormal2 = pThatFace->GetNormal() ;

  // dihedral angle in degrees
  double dAngleRad ;
  SER( sNormal.AngleBetween( sNormal2, dAngleRad )) ;
  double dAngleDeg = SM_RAD2DEG( dAngleRad ) ;

  // dihedral angle > MinFeatureAngle is a feature
  if ( smos_Fabs( dAngleDeg ) > m_dMinFeatureAngle )
    { return TRUE ; }

  // otherwise
  return FALSE ;

} // end TestFeatureEdge

/*******************************************************************//**
PURPOSE: Determine if the edge is a silhouette edge. An edge is called
  a silhouette edge if normals of two adjacent faces pointing 'OPPOSITE'
  way with respect to the view vector. 

NOTES: returns FALSE when view-based decimation is off (m_bEyeOrViewVector == NULL)

  An edge is silhouette if
    1. the edge lies on the eye point in perspective view
    2. the edge is attached to triangle perp to the view direction
    3. the edge is attached to two triangles whose normals are on opposited sides of the viewing direction

  An edge is not silhouette if
    1. view-based decimation is off (m_pEyeOrViewVector == NULL)
    2. the edge is lamina  (GWC: this seems wrong to me - but everywhere this method gets used, Lamina is its own special case)
    3. is not silhouette
***********************************************************************/
SmBoolean SmPolyDecimate::TestSilhouetteEdge
  (const SmPolyEdge * cpPolyEdge)  // in : target edge 
{
  // no view-based decimation - no silhouettes
  if (m_pEyeOrViewVector == NULL)
    { return FALSE ; }

  // Get View dir for parallel and perspective view modes
  SmVector3d sViewVec ;
  if ( m_bParallelProjection )
    {
      sViewVec = *m_pEyeOrViewVector ;
    }
  else // Perspective projection
    {
      SmVector3d sMidPnt ;
      SER( cpPolyEdge->EvaluatePoint( 0.5, sMidPnt )) ;
      sViewVec = sMidPnt - *m_pEyeOrViewVector ;

      // when eye is on the edge - it's a silhouette edge 
      if ( sViewVec.Unitize() != SM_SUCCESS )
        { return TRUE ; }
    }

  // SmVector3d sVec    = cpPolyEdge->GetEndPoint() - cpPolyEdge->GetStartPoint() ;
  // SmPolyEdge * pCCW  = cpPolyEdge->GetCCWPolyEdge() ;
  // SmVector3d sVecCCW = pCCW->GetEndPoint() - pCCW->GetStartPoint() ;
  // SmVector3d sNormal = sVec*sVecCCW ;
  // SER(sNormal.Unitize()) ;

  // locals
  SmPolyFace * pThisFace = cpPolyEdge->GetPolyFace() ; NER( pThisFace ) ;
  SmVector3d   sNormal   = pThisFace->GetNormal() ;

  // edges on triangles perp to the view are silhouette edges
  double dDot = sNormal.Dot(sViewVec) ;
  if ( smos_Fabs(dDot) < SM_EFF_ZERO ) 
    { return TRUE ; }

  // lamina edges are not Silhouette edges 
  // (GWC: this seems like it might be wrong - but everywhere this method gets used, Lamina is its own special case)
  SmPolyEdge * pRadialEdge = cpPolyEdge->GetRadial() ;
  if (pRadialEdge == NULL)
    { return FALSE ; }

  // SmVector3d sVec2    = pRadialEdge->GetEndPoint() - pRadialEdge->GetStartPoint() ;
  // SmPolyEdge * pCCW2  = pRadialEdge->GetCCWPolyEdge() ;
  // SmVector3d sVecCCW2 = pCCW2->GetEndPoint() - pCCW2->GetStartPoint() ;
  // SmVector3d sNormal2 = sVec2*sVecCCW2 ;
  // SER( sNormal2.Unitize() ) ;
  SmPolyFace * pThatFace = pRadialEdge->GetPolyFace() ; NER( pThatFace ) ;
  SmVector3d   sNormal2  = pThatFace->GetNormal() ;

  // edges on triangles perp to the view are silhouette edges
  double dDot2 = sNormal2.Dot( *m_pEyeOrViewVector ) ;
  if ( smos_Fabs( dDot2 ) < SM_EFF_ZERO ) 
    { return TRUE ; }

  // edges with faces on opposite sides of the view direction are silhouette edges
  if ( dDot*dDot2 < -SM_EFF_ZERO ) 
    { return TRUE ; }

  // else it's not a silhouette edge
  return FALSE ;

} // end SmPolyDecimate::TestSilhouetteEdge

/*******************************************************************//**
PURPOSE: Get normal, center & area data from a triangular faces
  making sure to generate output for degenerate triangles

NOTES: 
  normal = cross(Edge0Dir, -Edge2Dir), for degenerate triangles normal = [0,0,1]
  center = (Pt0 + Pt1 + Pt2) / 3.0 
  area = (dBaseLength * dHeight) / 2.0, for degenerate triangle area = 0.0
***********************************************************************/
SmStatus SmPolyDecimate::GetTriangularFaceData
 (const SmPolyFace * cpPolyFace,   // in : target polyface
  SmVector3d       & rFaceNormal,  // out: normal = unitized( cross(Edge0Dir, -Edge2Dir) ) (degNormal = [0,0,1])
  SmPoint3d        & rCenter,      // out: center = (Pt0 + Pt1 + Pt2) / 3.0 
  double           & rdArea)       // out: area   = (dBaseLength * dHeight) / 2.0; (degArea = 0.0)
{
  // locals
  SM_PTR_ARRAY(sPolyEdges, SmPolyEdge, 16) ; // SmTArray<SmPolyEdge*> 

  // every sPolyEdge
  cpPolyFace->GetNonDegeneratePolyEdges( sPolyEdges ) ;
  // used to be: cpPolyFace->GetPolyEdges( sPolyEdges ) ;

  // check input
  if ( sPolyEdges.GetSize() != 3 )
    { SER(SM_ERR) ; } // Non-triangular face  

  // triangle locals
  SmPolyEdge * pEdge1 = sPolyEdges[0] ;
  SmPolyEdge * pEdge2 = sPolyEdges[1] ;
  SmPoint3d    sPnt1  = pEdge1->GetStartPoint() ;
  SmPoint3d    sPnt2  = pEdge1->GetEndPoint() ;
  SmPoint3d    sPnt3  = pEdge2->GetEndPoint() ;
  SmVector3d   sVec1  = sPnt2 - sPnt1 ;
  SmVector3d   sVec2  = sPnt3 - sPnt1 ;
  SmVector3d   sUnitVec = sVec1 ;
  if ( sUnitVec.Unitize() != SM_SUCCESS ) 
    { sVec1.Set(1,0,0)  ; }

  // set normal = cross(Edge0Dir, -Edge2Dir)
  rFaceNormal = sUnitVec*sVec2 ;

  // for degenerate triangle let normal = [0,0,1]
  if ( rFaceNormal.Unitize() != SM_SUCCESS )
    {
      SmVector3d sUnit2 = sVec2 ;
      if ( sUnit2.Unitize() != SM_SUCCESS ) 
        {
          rFaceNormal.Set(0,0,1) ;
        }
      else 
        { 
          rFaceNormal = sUnitVec * sUnit2 ;
        }
      if (rFaceNormal.Unitize() != SM_SUCCESS)
        { rFaceNormal.Set(0,0,1) ; }
    } // end degenerate triangle check

  // set center
  rCenter = (sPnt1 + sPnt2 + sPnt3) / 3.0 ;

  // area locals
  double     dDist       = sVec2.Dot(sUnitVec) ;
  SmVector3d sHeightVec  = sVec2 - dDist * sUnitVec ;
  double     dHeight     = sHeightVec.Length() ;
  double     dBaseLength = sVec1.Length() ;

  // set area
  rdArea = (dBaseLength * dHeight) / 2.0 ;

  // all done
  return SM_SUCCESS ;

} // end SmPolyDecimate::GetTriangularFaceData

/*******************************************************************//**
PURPOSE: Triangulate a 'hole' face in the demination process. It will
  find a slit line between two vertices and then split it.

NOTES: 
***********************************************************************/
SmStatus SmPolyDecimate::SplitFace(SmPolyFace            * pFace,
                                   SmTArray<SmPolyFace*> & rNewFaces)
{
  rNewFaces.ReSet();
  NER( pFace );

  pFace->TriangulateNonPlanar( rNewFaces );
  return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Determine if new polygon is within error bounds

NOTES:

SIDE EFFECTS --- Adds a SM_AI_POLY_ERRORBOUNDS SmVector3dAttribute to 
  pNewFace whose x/y coordinate defines the error interval bounds for 
  the NewFace polygon.

  The error interval is defined by the min/max signed distance of a set of 
  polygon sample points to polyVertex/polyEdge/polyFace objects in pPolyCache 
***********************************************************************/
SmStatus SmPolyDecimate::CheckErrorBounds
  (SmPolyFace  * pNewFace,           // in : target face to evaluate and modify with a SM_AI_POLY_ERRORBOUNDS attribute
   SmBrepCache * pPolyCache,         // in : contains vertices/edges/faces to which NewFace sample points are projected
   SmBoolean   & rbWithinErrBounds)  // out: TRUE: m_dMaximumError == 0.0 or
                                     //      min/max NewFace sample distances to PolyCache objects < +/-m_dMaximumError
{
  // locals
  ULONG ii ;
  SmExtent1d sErrIvl(0.0,0.0) ; // becomes the union of all found object distances+deviError
  SmPoint3d aTestPnts[10] ; // [0] = Face->Edge[0]->Start
                            // [1] = Face->Edge[1]->Start
                            // [2] = Face->Edge[2]->Start
                            // [3] = Face->Edge[0]->MidPoint
                            // [4] = Face->Edge[1]->MidPoint
                            // [5] = Face->Edge[2]->MidPoint
                            // [6] = Face->SubTri[0]->MidPoint = ([0]+[3]+[5])/3
                            // [7] = Face->SubTri[1]->MidPoint = ([1]+[3]+[4])/3
                            // [8] = Face->SubTri[2]->MidPoint = ([2]+[4]+[5])/3
                            // [9] = Face->SubTri[3]->MidPoint = ([3]+[4]+[5])/3

  // Sample points from the input new face. Break triangle into four sub-triangles, at edge midpoints.
  SmPolyEdge * pE = pNewFace->GetOuterPolyLoop()->GetFirstPolyEdge() ;
  for(ii=0;ii<3;ii++)
    {
      aTestPnts[ii] = pE->GetStartPoint() ;
      pE->EvaluatePoint( 0.5, aTestPnts[ii+3] ) ;
      pE = SM_CAST_PTR(SmPolyEdge,pE->GetNext()) ;
    }

  // Midpoints of four sub-triangles:
  aTestPnts[6] = ( aTestPnts[0] + aTestPnts[3] + aTestPnts[5] ) / 3.0 ;
  aTestPnts[7] = ( aTestPnts[1] + aTestPnts[3] + aTestPnts[4] ) / 3.0 ;
  aTestPnts[8] = ( aTestPnts[2] + aTestPnts[4] + aTestPnts[5] ) / 3.0 ;
  aTestPnts[9] = ( aTestPnts[3] + aTestPnts[4] + aTestPnts[5] ) / 3.0 ;

  // for every nonvertex sample point
  for(ii=3;ii<10;ii++)
    {
      // Compute distance from test point to each original polygon
      SmSolution sSData[64] ;
      SmSolutionArray sSolutions(64,sSData) ;
      double dBestAnswer=SM_BIG_DOUBLE ;

      // find the closest vertex/edge/face in polyBrep to testpoint
      SER(SmPolySolver::CachePointSolve(pPolyCache,
                                        aTestPnts[ii],
                                        SM_SO_MINIMIZE,
                                        SM_SR_ALL,
                                        1.0e-8,
                                        dBestAnswer,NULL,sSolutions)) ;
      SM_ASSERT(sSolutions.GetSize() > 0) ;
      SmSolution & rSol = sSolutions[0] ;
      SmPoint3d sFoundPoint ;
      sFoundPoint.x = rSol.m_vStart[3] ; // position of testPoint projected to found object
      sFoundPoint.y = rSol.m_vStart[4] ;
      sFoundPoint.z = rSol.m_vStart[5] ;

      // get found object
      SmObject     * pTopoObj   = rSol.m_apObjects[0] ;
      SmPolyFace   * pFoundFace = NULL ;
      SmPolyEdge   * pFoundEdge = NULL ;
      SmPolyVertex * pFoundVert = NULL ;
      switch (pTopoObj->GetType()) 
        {
          case SmPolyFace_TYPE   : pFoundFace = SM_REINTERPRET_CAST(SmPolyFace*,pTopoObj) ;
                                   break ;

          case SmPolyEdge_TYPE   : pFoundEdge = SM_REINTERPRET_CAST(SmPolyEdge*,pTopoObj) ;
                                   pFoundFace = pFoundEdge->GetPolyFace() ;
                                   break ;

          case SmPolyVertex_TYPE : pFoundVert = SM_REINTERPRET_CAST(SmPolyVertex*,pTopoObj) ;
                                   pFoundEdge = pFoundVert->GetFirstPolyEdge() ;
                                   pFoundFace = pFoundEdge->GetPolyFace() ;
                                   break ;
          default:
              SER(SM_ERR) ;

        } // end switch on found object type

      // Get found object's face normal and signed distance to found object
      SmVector3d sNormal  = pFoundFace->GetNormal() ;
      SmVector3d sDistVec = aTestPnts[ii] - sFoundPoint ;
      double     dMinDist = sDistVec.Length() ;
      if (sDistVec.Dot(sNormal) < 0.0) 
        {
          dMinDist = -dMinDist ;
        }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if (bDebugMe) 
        {
          smgfx_SetLook(1,2, 1,0,0) ; sDistVec.Draw(&sFoundPoint) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // Increase 20% to the distance
      dMinDist *= 1.2 ;

      SmVector3d sDeviVec(0.0,0.0,0.0) ;
      SM_PTR_ARRAY(sAttr,SmAttribute,1) ; // SmTArray<SmAttribute*> 

      // find SmVector3dAttribute on pFoundFace
      // GWC??? this looks like a bug - the face may have any number of various attributes on it
      //         shouldn't we use pFoundFace->FindAttribute( SM_AI_POLY_ERRORBOUNDS )
      pFoundFace->GetAttributes(sAttr) ;
      if (sAttr.GetSize() > 0) 
        {
          SmVector3dAttribute *pAttr = SM_CAST_PTR(SmVector3dAttribute,sAttr[0]) ;
          sDeviVec = pAttr->GetValue() ;
        }

      // GWC: I think this adds in the deviation interval of the foundFace to the actual project distance
      //  ?? to me this has noe geometric meaning
      SmExtent1d sIvl(sDeviVec.x+dMinDist,sDeviVec.y+dMinDist) ;

      // accumulate all the error intervals
      sErrIvl.Union(sIvl,sErrIvl) ;

    } // end iter every nonVertex face sample point

  // arrive here when sErrIvl now spans all distance to polyBrep objects for each test object
  //  GWC ?? the distances are augmented with sDeviVec values which I don't understand

  rbWithinErrBounds = FALSE ;

  SmExtent1d sIvl( -m_dMaximumError, m_dMaximumError ) ;

  // when testing is turned off or sErrIvl is completely inside the +/-m_dMaximumError bounds
  if ( m_dMaximumError == 0.0 || sErrIvl.IsContainedBy( sIvl ) )
    {
      // this polygon is good
      rbWithinErrBounds = TRUE ;

      // store errIvl as x and y coordinate values in a SmVector3dAtribute
      SmVector3d sErrVec(sErrIvl.GetMin(),sErrIvl.GetMax(),0.0) ;
      const SmContext * cpContext = pNewFace->GetContext() ;
      SmVector3dAttribute *pAttr = new (*cpContext) SmVector3dAttribute(SM_AI_POLY_ERRORBOUNDS,sErrVec) ;

      // attach attribute to face - doesn't add duplicates
      pNewFace->AddAttribute(pAttr) ;

    } // end OK polygon check

  // all done
  return SM_SUCCESS ;

} // end SmPolyDecimate::CheckErrorBounds

/*******************************************************************//**
PURPOSE: Compute and weight chordheight between vertex and 
    a baseline running between the ends of the vertice's feature/boundary edges.


NOTES: The output rdDistanceToEdge =
      vert->IsBoundary && m_dBoundaryEdgeWeight <= 0.0 ? SM_BIG_DOUBLE
    : vert->isBoundary                                 ? chordheight/m_dBoundaryEdgeWeight + m_dMaxFaceError
    : vert->IsInterior && m_dInteriorEdgeWeight <= 0.0 ? SM_BIG_DOUBLE
    : vert->IsInterior                                 ? chordheight/m_dInteriorEdgeWeight + m_dMaxFaceError
    : SM_ERR
  m_dMaxFaceError = Max(Face[i]->Error)
  Face[i]        = the set of faces attached to pVert
  Face[i]->Error = a measure of the max distance to existing polyBrep of a newFace made by decimation.

SIDE EFFECTS --- pAttr->m_dVertexDistance = rdDistanceToEdge
***********************************************************************/
SmStatus SmPolyDecimate::FindDistanceToEdge
 (SmPolyVertex * pVert,             // in : target vertex 
  double       & rdDistanceToEdge)  // out:  vert->IsBoundary && m_dBoundaryEdgeWeight <= 0.0 ? SM_BIG_DOUBLE
                                    //     : vert->isBoundary                                 ? chordheight/m_dBoundaryEdgeWeight + m_dMaxFaceError
                                    //     : vert->IsInterior && m_dInteriorEdgeWeight <= 0.0 ? SM_BIG_DOUBLE
                                    //     : vert->IsInterior                                 ? chordheight/m_dInteriorEdgeWeight + m_dMaxFaceError
                                    //     : SM_ERR ? SM_BIG_DOUBLE
{
  // init output
  rdDistanceToEdge = SM_BIG_DOUBLE ;

  // locals
  double dDist ;
  
  // classify vertex and get attached feature/boundary edges. 
  // Those edges define the baseline for chordheight.
  //                 side effects: ensure vertex has SM_AI_VERTEX_DECIMATION_VALUES attribute
  //                               set pAttr->m_eVertClass
  SM_PTR_ARRAY(sFeatureOrBoundaryEdges, SmPolyEdge, 16) ; // SmTArray<SmPolyEdge*>
  FindVertexClass(pVert,&sFeatureOrBoundaryEdges) ;

  // get attrib
  SmDecimateVertexAttr *pAttr = (SmDecimateVertexAttr*)(pVert->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES)) ;

  // check state - must work with boundary of interior vertices
  if (   pAttr->m_eVertClass != SM_PVC_BOUNDARY
      && pAttr->m_eVertClass != SM_PVC_INTERIOR) 
    { SER(SM_ERR) ; }

  // check state
  SM_ASSERT(sFeatureOrBoundaryEdges.GetSize() == 2) ;

  // make sure attrib has current MaxFaceError value 
  SER( FindVertexAveragePlaneDistance( pVert, dDist )) ;

  // Determine the baseline running between the two feature edge endpoints
  SmPolyVertex * pStartV    =   (pVert != sFeatureOrBoundaryEdges[0]->GetStartPolyVertex()) 
                              ? sFeatureOrBoundaryEdges[0]->GetStartPolyVertex()
                              : sFeatureOrBoundaryEdges[0]->GetEndPolyVertex() ;
  SmPolyVertex * pEndV      =   (pVert != sFeatureOrBoundaryEdges[1]->GetStartPolyVertex())
                              ? sFeatureOrBoundaryEdges[1]->GetStartPolyVertex()
                              : sFeatureOrBoundaryEdges[1]->GetEndPolyVertex() ;
  SmPoint3d      sLineStart = pStartV->GetPoint() ;
  SmPoint3d      sLineEnd   = pEndV  ->GetPoint() ;
  double         dLengthSq  = sLineStart.DistanceBetweenSquared( sLineEnd ) ;

  // Get chordHeigth between Vertex and FeatureEdge baseline
  double dParameter ;
  double dChordHeight ;
  if ( dLengthSq < SM_EFF_ZERO_SQ ) 
    { // dist to start vertex
      dChordHeight = sLineStart.DistanceBetween( pVert->GetPoint() ) ;
    }
  else // dist to baseline
    {
      SER( smgu_SegmentPointDistance( sLineStart, sLineEnd, pVert->GetPoint(), dChordHeight, dParameter )) ;
    }

  // set distance based on eVertClass type, m_dInteriorEdgeWeight, and m_dBoundaryEdgeWeight values
  double dDistance = 0.0 ;

  // when Vert is on boundary
  if ( pAttr->m_eVertClass == SM_PVC_BOUNDARY )
    {
      // when asked - set distance so large that vertex is never decimated
      if ( this->m_dBoundaryEdgeWeight <= 0.0 )
        { dDistance = SM_BIG_DOUBLE ; }
      else // distance 
        { dDistance = dChordHeight/m_dBoundaryEdgeWeight + pAttr->m_dMaxFaceError ; }
    }
  // else when vert is an interior point
  else if (pAttr->m_eVertClass == SM_PVC_INTERIOR)
    {
      if (this->m_dInteriorEdgeWeight <= 0.0)
        { dDistance = SM_BIG_DOUBLE ; }
      else
        { dDistance = dChordHeight/m_dInteriorEdgeWeight + pAttr->m_dMaxFaceError ; }
    }

  // save distance
  pAttr->m_dVertexDistance = dDistance ;

  // set output
  rdDistanceToEdge = pAttr->m_dVertexDistance ;

  // all done
  return SM_SUCCESS ;

} // end SmPolyDecimate::FindDistanceToEdge

/*******************************************************************//**
PURPOSE: Retrieve or recompute project distance (augmented with a MaxFaceError)
            from the vertex to the vertex's average plane.

NOTES: When one of the surrounding faces is modified we must
                reset the attribute to force recomputation.

METHOD --- compute average plane of all faces attached to this vertex as
       Face[i]   = the set of faces attached to pVert
       AvgArea   = Sum_i( Face[i]->Area) 
       AvgCenter = Sum_i( Face[i]->Center * Face[i]->Area) / AvgArea
       AvgNormal = Sum_i( Face[i]->Normal * Face[i]->Area) / AvgArea

       DistToCenter = project distance of vertex to AvgPlane + MaxFaceError

       MaxFaceError = Max(Face[i]->Error)
       Face[i]->Error = a measure of the max distance to existing polyBrep of a newFace made by decimation.

SIDE EFFECTS --- 
    set pVert->SM_AI_VERTEX_DECIMATION_VALUES attribute values
        pAttr->m_dVertexDistance     = rdDistToCenter ;
        pAttr->m_dMaxFaceError       = dMaxFaceError ;
        pAttr->m_vAveragePlaneNormal = sAvgNormal ;
***********************************************************************/
SmStatus SmPolyDecimate::FindVertexAveragePlaneDistance
 (SmPolyVertex * pVert,           // in : target vertex
  double       & rdDistToCenter)  // out: projected distance to average plane of all faces about vertex
                                  //      augmented with a MaxFaceError value
{
  // locals
  SM_PTR_ARRAY(sCoinVEdges,SmPolyEdge,64) ;

  // Test to see if the decimation attribute exists and create it if it does not
  SmDecimateVertexAttr *pAttr = (SmDecimateVertexAttr*)(pVert->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES)) ;
  if (pAttr == NULL) 
    { SER(SM_ERR) ; }

  // low work - m_dVertexDistance already computed
  if (pAttr->m_dVertexDistance >= 0.0) 
    {
      rdDistToCenter = pAttr->m_dVertexDistance ;
      return SM_SUCCESS ;
    }

  // Calculate the average plane of all surrounding faces

  // locals
  ULONG ii ;
  SmVector3d sSumNormal(0.0,0.0,0.0) ;
  SmVector3d sSumCenter(0.0,0.0,0.0) ;
  double     dSumArea = 0.0 ;
  double dMaxFaceError = 0.0 ;

  // get polyEdges starting at vertex
  pVert->GetStartingPolyEdges(sCoinVEdges) ;

  // for every vertex->polyEdge - 
  ULONG lNumCoin = sCoinVEdges.GetSize() ;
  if ( lNumCoin==0 )
    {
      rdDistToCenter           = SM_BIG_DOUBLE ;
      pAttr->m_dVertexDistance = SM_BIG_DOUBLE ;
      return SM_SUCCESS ;
    }

  for(ii=0 ;ii<lNumCoin ;ii++)
    {
      SmPolyFace * pF = sCoinVEdges[ii]->GetPolyFace() ;

      //
      SmVector3dAttribute *pAttr3d = (SmVector3dAttribute*)(pF->FindAttribute(SM_AI_POLY_ERRORBOUNDS)) ;
      if (pAttr3d)
        {
          SmVector3d sVec = pAttr3d->GetValue() ;
          dMaxFaceError = smos_Max( dMaxFaceError, -sVec.x ) ;
          dMaxFaceError = smos_Max( dMaxFaceError,  sVec.y ) ;
        }

      // get face geometry
      SmVector3d sNormal ;
      SmPoint3d  sCenter ;
      double     dArea ;
      SER(SmPolyDecimate::GetTriangularFaceData( pF,         // in : target polyface                                                      
                                                 sNormal,    // out: normal = unitized( cross(Edge0Dir, -Edge2Dir) ) (degNormal = [0,0,1])
                                                 sCenter,    // out: center = (Pt0 + Pt1 + Pt2) / 3.0                                     
                                                 dArea )) ;  // out: area   = (dBaseLength * dHeight) / 2.0; (degArea = 0.0)              

      // accumulate geometric properties weighted by area
      sSumNormal = sSumNormal + sNormal*dArea ;
      sSumCenter = sSumCenter + sCenter*dArea ;
      dSumArea += dArea ;

    } // end iter every pVertex->polyEdge

  // turn accumulated values into average values
  SmVector3d sAvgNormal = sSumNormal/dSumArea ;
  SmPoint3d  sAvgCenter = sSumCenter/dSumArea ;

  // when average normal is zero - set output to large number and quit
  if ( sAvgNormal.LengthSquared() < SM_EFF_ZERO_SQ ) 
    {
      rdDistToCenter           = SM_BIG_DOUBLE ;
      pAttr->m_dVertexDistance = SM_BIG_DOUBLE ;
      return SM_SUCCESS ;
    }

  // unitize the average normal
  SER(sAvgNormal.Unitize()) ;
              
  // Find the project distance between this vertex and the average plane
  // add in MaxFaceError

  SmVector3d sDistVec          = pVert->GetPoint() - sAvgCenter ;
  double     dDistToCenter     = smos_Fabs(sDistVec.Dot(sAvgNormal)) ;
  rdDistToCenter               = dDistToCenter + dMaxFaceError ;
  pAttr->m_dVertexDistance     = rdDistToCenter ;
  pAttr->m_dMaxFaceError       = dMaxFaceError ;
  pAttr->m_vAveragePlaneNormal = sAvgNormal ;

  // all done
  return SM_SUCCESS ;

} // end SmPolyDecimate::FindVertexAveragePlaneDistance

/*******************************************************************//**
PURPOSE: Local routine to sort a given array of edges (end-to-end)
            so that it can form a loop

NOTES: 
***********************************************************************/
static SmStatus sm_SortPolyEdges(SmTArray<SmPolyEdge*> & rLoopEdges)
{
  // Sort rLoopEdges to make it a connected loop
  ULONG ii, jj, lSize = rLoopEdges.GetSize();
  SmPolyVertex * pCurrV = rLoopEdges[0]->GetEndPolyVertex();
  for(ii=1;ii+1<lSize;ii++) // note: can't say lSize-1
  {
      ULONG lFoundNext = 0;
      for(jj=ii;jj<lSize;jj++)
      {
          SmPolyEdge * pE = rLoopEdges[jj];
          SmPolyVertex * pV = pE->GetStartPolyVertex();
          if (pV == pCurrV) {
              lFoundNext = jj;
              break;
          }
      }
      if ( lFoundNext == 0 ) {
          return SM_ERR;
      }
      SmPolyEdge * pNextEdge = rLoopEdges[lFoundNext];
      if ( lFoundNext != ii )
      {
           rLoopEdges.SetAt( lFoundNext, rLoopEdges[ii] );
           rLoopEdges.SetAt( ii, pNextEdge );
      }
      pCurrV = pNextEdge->GetEndPolyVertex();
  }
  return SM_SUCCESS;
} // end sm_SortPolyEdges
 
/*******************************************************************//**
PURPOSE: Local Compare function used by quick sort to sort vertex attributes.

NOTES: 
***********************************************************************/
static int sm_CompareVertex(const void *pElem1, const void *pElem2) 
{
  if (*(void**)pElem1 == NULL) { return 1; }
  if (*(void**)pElem2 == NULL) { return 1; }
  if ((*((SmDecimateVertexAttr**)pElem1))->GetVertexDistance() > 
      (*((SmDecimateVertexAttr**)pElem2))->GetVertexDistance()) {
      return 1;
  }
  if ((*((SmDecimateVertexAttr**)pElem1))->GetVertexDistance() <
      (*((SmDecimateVertexAttr**)pElem2))->GetVertexDistance()) {
      return -1;
  }
  return 0;
} // end sm_CompareVertex


/*******************************************************************//**
PURPOSE: Compute or recompute the attributes of a vertex and append
  them to the array.  

NOTES: 
***********************************************************************/
SmStatus SmPolyDecimate::ComputeVertexAttributes(SmPolyVertex *pVert,
                                               SmDecimateVertexAttr *& rpNewAttr)
{
  rpNewAttr = NULL;
  SmBoolean bNewDVAttr;
  SmPolyVertexClass eVertClass = FindVertexClass(pVert, NULL, &bNewDVAttr);
             
  double dDistance = SM_BIG_DOUBLE;
  SmBoolean bGoodVertex;
  
  switch ( eVertClass ) {
    case SM_PVC_SIMPLE:
      if (FindVertexAveragePlaneDistance(pVert,dDistance) != SM_SUCCESS) {
          bGoodVertex = FALSE;
      }
      else {
          bGoodVertex = TRUE;
      }
      break;
    case SM_PVC_BOUNDARY:
      if (m_dBoundaryEdgeWeight > 0.0) {
          SER(FindDistanceToEdge(pVert,dDistance));
          bGoodVertex = TRUE;
      }
      else {
          bGoodVertex = FALSE;
      }
      break;
    case SM_PVC_INTERIOR:
      if (m_dInteriorEdgeWeight > 0.0) {
          SER(FindDistanceToEdge(pVert,dDistance));
          bGoodVertex = TRUE;
      }
      else {
          bGoodVertex = FALSE;
      }
      break;
    case SM_PVC_UNKNOWN:
    default:
      bGoodVertex = FALSE;
  }
  
  if (bGoodVertex && m_dMaximumError && dDistance > m_dMaximumError) {
      bGoodVertex = FALSE;
  }
  
  SmDecimateVertexAttr *pAttr = (SmDecimateVertexAttr *)pVert->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES);
  NER(pAttr);

  if (!bGoodVertex) {
      pVert->RemoveAttribute(pAttr);
      if ( bNewDVAttr ) delete pAttr;
  }
  else { // Keep this attribute and use it to decimate
      rpNewAttr = pAttr;
  }

  return SM_SUCCESS;

} // end ComputeVertexAttributes


/*******************************************************************//**
PURPOSE: This method fills the hole that will be caused by deletion
  of a vertex.  

NOTES: 
***********************************************************************/
SmStatus SmPolyDecimate::FillHole
 (SmPolyBrep                      * pPolyBrep,              // in : Hole filled in this brep
  SmPolyVertex                    * pVertToBeRemoved,       // in : Vertex to be removed that will leave hole.
  SmDecimateVertexAttr            * pAttr,                  // in : Decimation attributes of this vertex.
  SmBoolean                       & rbSuccess,              // out: Did we successfully fill the hole
  SmTArray<SmPolyFace*>           & rOrigPolys,             // in : Original polygons,
  SmTArray<SmPolyFace*>           & rNewPolys,              // in : New Polygons created to fill the hole
  SmTArray<SmPolyVertex*>         & rSurvivingLoopVertices, // in : Vertices which will survive the destruction of the rNewPolys
  SmTArray<SmDecimateVertexAttr*> & rSurvivingLoopAttr)     // out: Attributes corresponding to rSurvivingLoopVertices
{
  rbSuccess = FALSE;
  SM_PTR_ARRAY(sCoinVEdges,SmPolyEdge,64);
  SM_PTR_ARRAY(sLoopEdges,SmPolyEdge , 64);              // SmTArray<SmPolyEdge*>
  SM_PTR_ARRAY(sFeatureOrBoundaryEdges, SmPolyEdge, 64); // SmTArray<SmPolyEdge*>
  SM_PTR_ARRAY(sNewPolys,SmPolyFace, 4 );
  
  SmPolyShell * pShell = pVertToBeRemoved->GetFirstPolyEdge()->GetPolyFace()->GetPolyShell();
  NER(pShell);

  // Collect all boundary edges of the 'hole' which is going to
  // be re-triangulated
  pVertToBeRemoved->GetStartingPolyEdges(sCoinVEdges);
  ULONG ii, lNumCoin = sCoinVEdges.GetSize();
  for(ii=0;ii<lNumCoin;ii++)
  {
      SmPolyEdge *pE = sCoinVEdges[ii];
      SmPolyEdge *pCCWE = pE->GetCCWPolyEdge(); NER(pCCWE);
     sLoopEdges.Add(pCCWE);
  }
  
  sFeatureOrBoundaryEdges.ReSet();
  if (   pAttr->m_eVertClass == SM_PVC_BOUNDARY
      || pAttr->m_eVertClass == SM_PVC_INTERIOR )
  {
      FindVertexClass(pVertToBeRemoved,&sFeatureOrBoundaryEdges);
      if (pAttr->m_eVertClass == SM_PVC_BOUNDARY)
        { sLoopEdges.Append(sFeatureOrBoundaryEdges); }
  }
  
  // Sort sLoopEdges and make it a connected loop
  if ( sm_SortPolyEdges(sLoopEdges) != SM_SUCCESS )
    {}

  ULONG lSize = sLoopEdges.GetSize();
  // Collect original faces that surround this vertex
  rOrigPolys.ReSet();
  for(ii=0;ii<lSize;ii++){
      rOrigPolys.AddUnique(sLoopEdges[ii]->GetPolyFace());
  }
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe7 = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe7 || lCount == lDebugCount) {
      smgfx_Erase();
      SmTArray<SmPolyFace*> sPolyFaces;
      pVertToBeRemoved->GetPolyFaces( sPolyFaces );
      smgfx_SetLook(1,2, 0.2,1,0.2);
      for ( ii = 0; ii < sPolyFaces.GetSize(); ii++ ) {
          sPolyFaces[ii]->Draw(); sm_GraphicsLoop();
      }
      smgfx_SetLook(1,2, 1,0,0);
      for(ii=0;ii<sLoopEdges.GetSize();ii++){
          sLoopEdges[ii]->Draw(); sm_GraphicsLoop();
      }
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1);
      for(ii=0;ii<sFeatureOrBoundaryEdges.GetSize();ii++){
          sFeatureOrBoundaryEdges[ii]->Draw(); sm_GraphicsLoop();  
      }
      sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1); m_pPolyBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  ULONG lLineStartIndex = 0;
  ULONG lLineEndIndex = 0;
  SmPolyEdge * pIgnoredBoundaryEdge = NULL;
  switch (pAttr->m_eVertClass)
  {
    case SM_PVC_BOUNDARY:
      {
          // We are going to ignore one of the boundary edges
          // of sLoopEdges for the following two reasons:
          // (1) Two boundary edges will be replaced by just one new edge
          // (2) Only start vertex of each edge of sLoopEdges shall be used later.
          pIgnoredBoundaryEdge = sFeatureOrBoundaryEdges[0];
          SmPolyEdge *pKeptBoundaryEdge = sFeatureOrBoundaryEdges[1];
          if (pIgnoredBoundaryEdge->GetStartPolyVertex() != pVertToBeRemoved)
          {
              pIgnoredBoundaryEdge = sFeatureOrBoundaryEdges[1];
              pKeptBoundaryEdge = sFeatureOrBoundaryEdges[0];
          }
          ULONG lIndex;
          if (!sLoopEdges.FindElement(pIgnoredBoundaryEdge,lIndex)) {
              SER(SM_ERR);
          }
          sLoopEdges.RemoveAt(lIndex);
          lSize--;
          SmPolyEdge *pCCWOfIgnored = pIgnoredBoundaryEdge->GetCCWPolyEdge();
          if ( !sLoopEdges.FindElement( pKeptBoundaryEdge, lLineStartIndex )) { SER(SM_ERR); }
          if ( !sLoopEdges.FindElement( pCCWOfIgnored,     lLineEndIndex   )) { SER(SM_ERR); }
      }
      break;
    case SM_PVC_SIMPLE:
      {
          double dMinAspectRatio = 0.1;
          SmBoolean bSuccess;
          double rdMaxAspectRatio = 0.0;
          SER(SmPolyFace::FindSplitLine( sLoopEdges, pAttr->m_vAveragePlaneNormal, &dMinAspectRatio, NULL, bSuccess,
                                        rdMaxAspectRatio, lLineStartIndex, lLineEndIndex));
          if (!bSuccess) {
              rbSuccess = FALSE;
              return SM_SUCCESS;
          }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
          if (bDebugMe4 && bSuccess) {
              smgfx_Erase();
              sLoopEdges[lLineStartIndex]->Draw();
              sm_GraphicsLoop();
              sLoopEdges[lLineEndIndex]->Draw();
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE
                          
      }
      break;
    case  SM_PVC_INTERIOR:
      {
          if (   !sLoopEdges.FindElement( sFeatureOrBoundaryEdges[0]->GetCCWPolyEdge(), lLineStartIndex )
              || !sLoopEdges.FindElement( sFeatureOrBoundaryEdges[1]->GetCCWPolyEdge(), lLineEndIndex ))
          {
              SER(SM_ERR);
          }
          if (   ( lLineEndIndex   + 1) % lSize == lLineStartIndex
              || ( lLineStartIndex + 1) % lSize == lLineEndIndex )
          {
              rbSuccess = FALSE;
              return SM_SUCCESS;
          }
      }
      break;
    default:
      break;

  } // end switch
        
  if ( lSize <= 2 ) {
      // Just delete this face.  No need to retessellate because the
      // vertex going away is a boundary vertex.
      rNewPolys.ReSet();
      rbSuccess = TRUE;
      return SM_SUCCESS;
  }
  // Now, Let's make a new face by tracing the sorted edges
  // in reverse order to fill in the 'hole'
  SmPolyEdge * pStartVertexEdge = NULL;
  SmPolyEdge * pEndVertexEdge = NULL;
  double dTol = pPolyBrep->GetTolerance();

  // This face is often not planar. No need to estimate a normal for the new face. [my_test_decimation_regression]
  //SmPolyEdge * pFirstE   = sLoopEdges[0];
  //SmVector3d   sFirstVec = pFirstE->GetVec() ;
  //ULONG iCnt ;
  //for(iCnt=1;iCnt<sLoopEdges.GetSize();iCnt++)
  //  {
  //     SmPolyEdge * pSecondE     = sLoopEdges[iCnt] ;
  //     SmVector3d   sSecondVec   = pSecondE->GetVec() ;
  //     sFirstVec.NormalToPlane(sSecondVec, sPlaneNormal) ;
  //     if(sPlaneNormal.IsInitialized())
  //       { break ; }
  //  }

  SmPolyFace * pNewF = new ( pPolyBrep ) SmPolyFace( dTol,
                                                     NULL,
                                                     NULL,
                                                     pShell,
                                                     NULL );

  // SmPolyFace * pNewF = SmPolyFace::NewPolyFace (*pPolyBrep, dTol,NULL,NULL,pShell);
  NER(pNewF);
  SmPolyLoop * pNewL = new (pPolyBrep) SmPolyLoop(pNewF);
  // SmPolyLoop * pNewL = SmPolyLoop::NewPolyLoop (*pPolyBrep, pNewF);
  for(ii=0;ii<lSize;ii++)
  {
      SmPolyEdge * pE = sLoopEdges[ii];
      double dEdgeTol = pE->GetTolerance();
      SmPolyEdge * pNewE = new (pPolyBrep) SmPolyEdge(dEdgeTol); NER(pNewE);
      // SmPolyEdge * pNewE = SmPolyEdge::NewPolyEdge (*pPolyBrep, dEdgeTol); NER(pNewE);
      SmPoint3d sStartPnt = pE->GetStartPoint();
      pE->GetStartPolyVertex()->AddPolyEdge(pNewE);
      pNewL->PostInsert(pNewE);
      if (ii == lLineEndIndex)
        { pStartVertexEdge = pNewE; }
      else if (ii == lLineStartIndex)
        { pEndVertexEdge = pNewE; }
  }
  
  SmPolyEdge * pCurrE = pNewL->GetFirstPolyEdge();
  rSurvivingLoopVertices.ReSet();
  rSurvivingLoopAttr.ReSet();
  SM_PTR_ARRAY(sEdgesBetween,SmPolyEdge,16);
  for(ii=0;ii<lSize;ii++)
  {
      SmPolyEdge * pNextE = pCurrE->GetCCWPolyEdge();
      // Time to specify end vertex of current edge
      SmPolyEdge * pE = sLoopEdges[ii];
      SmPolyEdge * pRadialE = pE->GetRadial();
      if ( pRadialE )
      {
          // Glue (make radial partners) pRadialE and pCurrE without deleting any PolyEdge,
          //   glue endPVerts deleting pOtherEdge->PolyVerts          
          SER(pPolyBrep->GlueEdges( pRadialE,         // in : Target PolyEdge1
                                    pCurrE,           // in : Target PolyEdge2
                                    SM_OT_OPPOSITE,   // in : oneof SM_OT_SAME, SM_OT_OPPOSITE
                                    sEdgesBetween )); // out: PolyEdges between glued vertices (made zero length by gluing) 
                                                      // out: PolyVerts deleted when PolyVerts were glued (stale pointers), NULL to ignore, default:[NULL]
                                                      // out: associated surviving PolyVerts when PolyVerts were glued, NULL to ignore, default:[NULL]
          SmPolyVertex *pRadialV = pRadialE->GetStartPolyVertex(); NER(pRadialV);
          rSurvivingLoopVertices.Add(pRadialV);
          SmDecimateVertexAttr *pSurvAttr = (SmDecimateVertexAttr*)
              (pRadialV->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES));
          rSurvivingLoopAttr.Add( pSurvAttr ); // May be a NULL
      }
      pCurrE = pNextE;
  }
  pNewL->m_bClosed = TRUE;
  
  rbSuccess = TRUE;

  rNewPolys.ReSet();
  rNewPolys.Add( pNewF );
  if ( lSize > 3 && pAttr->m_eVertClass == SM_PVC_BOUNDARY )
  {
      // Split the resulting faces
      //SmBoolean bError = FALSE;
      if (SplitFace(pNewF,sNewPolys) != SM_SUCCESS) {
          rbSuccess = FALSE;
      }
      rNewPolys.Append( sNewPolys );
  }
  else if (lSize > 3)
  {
      // Now, split the face by the split line
      SmPolyEdge * pNewEdge2 = NULL;
      SmPolyLoop * pNewLoop2 = NULL;
      SmPolyFace * pNewFace2 = NULL;
      
      SmStatus eStat = pNewF->MakeManifoldEdge( pStartVertexEdge, pEndVertexEdge,
          pNewEdge2, pNewLoop2, pNewFace2 ); // NER( pNewFace2 );

      if ( eStat == SM_SUCCESS && pNewFace2 != NULL )
      {
          rNewPolys.Add(pNewFace2);
      }
      else
      {
          rbSuccess = FALSE;
      }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
      if (bDebugMe3) {
          smgfx_Erase();
          pStartVertexEdge->Draw();
          sm_GraphicsLoop();
          pEndVertexEdge->Draw();
          sm_GraphicsLoop();
          pNewEdge2->Draw();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE
      
      // Split the resulting faces
      //SmBoolean bError = FALSE;
      if ( SplitFace( pNewF, sNewPolys ) != SM_SUCCESS ) {
          rbSuccess = FALSE;
      }
      else {
          rNewPolys.Append( sNewPolys );
          if ( SplitFace( pNewFace2, sNewPolys ) != SM_SUCCESS ) {
              rbSuccess = FALSE;
          }
          rNewPolys.Append( sNewPolys );
      }
  } // end if lSize > 3

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
      if (bDebugMe4) {
          SmVector3d sNorm;
          SmPoint3d sCent;
          double dArea;
          for(ii=0;ii<rNewPolys.GetSize();ii++)
          {
              SmPolyFace * pF = rNewPolys[ii];
              SER( SmPolyDecimate::GetTriangularFaceData( pF,       // in : target polyface                                                      
                                                          sNorm,    // out: normal = unitized( cross(Edge0Dir, -Edge2Dir) ) (degNormal = [0,0,1])
                                                          sCent,    // out: center = (Pt0 + Pt1 + Pt2) / 3.0                                     
                                                          dArea )); // out: area   = (dBaseLength * dHeight) / 2.0; (degArea = 0.0)              
              if ( sNorm.LengthSquared() < SM_EFF_ZERO_SQ || dArea < SM_EFF_ZERO ) {
                  SE(SM_ERR);
              }
          }
      }
SmBoolean bDebugMe5 = FALSE;
      if (bDebugMe5) {
          for(ii=0;ii<rNewPolys.GetSize();ii++){
              SmPolyFace * pF = rNewPolys[ii];
              pF->Draw();
              sm_GraphicsLoop();
          }

      }
#endif // SM_DEBUG_CODE


  if ( !rbSuccess ) {
      for(ii=0;ii<rNewPolys.GetSize();ii++){
          SmPolyFace * pF = rNewPolys[ii];
          pPolyBrep->DeletePolyFace( pF );
      }
  }   

  return SM_SUCCESS;

} // end SmPolyDecimate::FillHole

/*******************************************************************//**
PURPOSE: Static Shell Sort Function for Sort for Ascending Order of Decimate Vertex Attributes

NOTES: 
***********************************************************************/
static void smgu_ShellSort_DecimateVertex( SmDecimateVertexAttr **aDecimateVertexAttr, ULONG lCount)
{
   int ii, bFlag = 1, lNumLength = lCount;
   int dd = lNumLength;
   SmDecimateVertexAttr *temp;
   while( bFlag || (dd > 1))      // boolean bFlag (true when not equal to 0)
   {
        bFlag = 0;           // reset bFlag to 0 to check for future swaps
        dd = (dd+1) / 2;
        for(ii=0;ii<(lNumLength-dd);ii++)
        {
            if ( aDecimateVertexAttr[ ii+dd ] == NULL || aDecimateVertexAttr[ii]==NULL )
              { continue; }

            if (  aDecimateVertexAttr[ ii+dd ]->GetVertexDistance()
                < aDecimateVertexAttr[ ii   ]->GetVertexDistance()) // ascending order test
            {
                    temp = aDecimateVertexAttr[ ii+dd ];      // swap positions ii+dd and ii
                    aDecimateVertexAttr[ ii+dd ] = aDecimateVertexAttr[ ii ];
                    aDecimateVertexAttr[ ii ] = temp;
                    bFlag = 1;                  // tells swap has occurred
            }
       }
   }
   return;

} // end smgu_ShellSort_DecimateVertex

/*******************************************************************//**
PURPOSE: Decimate operator which will reduce the total number of
  triangles of a given Brep by the Decimate criteria and maintain an
  error bound by using measurement. 

NOTES: This is a slow but very accurate way to reduce polygon
  counts.  For a faster method use DoQuadricDecimation.
***********************************************************************/
SmStatus SmPolyDecimate::DoMeasuredDecimation()
{
  SM_PTR_ARRAY(sOrigPolys, SmPolyFace,          512); // SmTArray<SmPolyFace*>
  SM_PTR_ARRAY(sNewPolys,  SmPolyFace,          512); // SmTArray<SmPolyFace*>
  SM_PTR_ARRAY(sVerts,     SmPolyVertex,         64); // SmTArray<SmPolyVertex*>
  SM_PTR_ARRAY(sAttr,      SmDecimateVertexAttr, 64); // SmTArray<SmDecimateVertexAttr*>
  SmObjsDelete<SmDecimateVertexAttr*> sCleanAttr(&sAttr);
  
  m_pPolyBrep->GetPolyVertices( sVerts );
  SmExtent1d sMinMax;
  //double dDistanceTotal = 0.0;
  ULONG lTotalHeadVertices = 0;

  // Get a list of head vertices and corresponding attributes
  ULONG lNumVerts = sVerts.GetSize();
  for(ULONG ii=0;ii<lNumVerts;ii++)
  {
      SmPolyVertex * pVert = sVerts[ii];
      
      lTotalHeadVertices++;
      
      SmDecimateVertexAttr *pNewAttr = NULL;
      SER( ComputeVertexAttributes( pVert, pNewAttr ));
      if ( pNewAttr )
        { sAttr.Add(pNewAttr); }
  }
  
  if ( lTotalHeadVertices < 1 )
    { return SM_SUCCESS; }  // nothing to do.
  
// Try going 300 at a time to see if sorting speed improves
#define SORT_GROUP 300
  ULONG lDecimationCount = 0;
  SM_PTR_ARRAY(sUsers,                 SmAObject,             16);         // SmTArray<SmAObject*>
  SM_PTR_ARRAY(sSurvivingLoopVertices, SmPolyVertex,          64);         // SmTArray<SmPolyVertex*>
  SM_PTR_ARRAY(sSurvivingLoopAttr,     SmDecimateVertexAttr,  64);         // SmTArray<SmDecimateVertexAttr*>
  SM_PTR_ARRAY(sWorkingGroup,          SmDecimateVertexAttr,  SORT_GROUP); // SmTArray<SmDecimateVertexAttr*>
  SmObjsDelete<SmDecimateVertexAttr*> sCleanWG(&sWorkingGroup);

  // Note, have to use GetSize() because sAttr changes size.
  for(ULONG ii=0;ii<sAttr.GetSize();ii+=SORT_GROUP)
  {
      // Now quicksort sort the attributes
      //ULONG lNumInSegment = SORT_GROUP;
      ULONG lEndOfSegment = ii+SORT_GROUP;
      if (lEndOfSegment > sAttr.GetSize()) {
          lEndOfSegment = sAttr.GetSize();
          //lNumInSegment = sAttr.GetSize()-ii;
      }
      
      smgu_ShellSort_DecimateVertex(&sAttr.GetDataArray()[ii], sAttr.GetSize()-ii );
      sWorkingGroup.ReSet();
      ULONG jj, lNumAttr = sAttr.GetSize();
      for(jj=ii;jj<lNumAttr;jj++)
      {
          sAttr[jj]->m_lSortIndex = jj;
          if ( sWorkingGroup.GetSize() < SORT_GROUP ) {
              sWorkingGroup.Add( sAttr[jj] );
              sAttr[jj] = NULL;
          }
      }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe7 = FALSE;
      if (bDebugMe7) {
          ULONG il ;
          for(il=0;il<sWorkingGroup.GetSize();il++){
              if (sWorkingGroup[il]) { sWorkingGroup[il]->Dump(); }
          }
      }
#endif // SM_DEBUG_CODE

      double dHighestValueInGroup = sWorkingGroup.GetLast()->m_dVertexDistance;
      if (lEndOfSegment == sAttr.GetSize()) {
          dHighestValueInGroup = SM_BIG_DOUBLE;
      }
      
      // Note, have to use GetSize() because sWorkingGroup can change.
      for(jj=0;jj<sWorkingGroup.GetSize();jj++)
      {
          SmDecimateVertexAttr *pAttr = sWorkingGroup[jj];

          pAttr->GetUsers(sUsers);
          if ( sUsers.GetSize() == 0 ) { continue; }

          // Assumes that the attribute is always on the head vertex.
          SmPolyVertex * pVertToBeRemoved = (SmPolyVertex*)sUsers[0];
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe /* || (ii==900 && jj == 47)*/ ) {
              m_pPolyBrep->ValidatePointers();
              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,0); pVertToBeRemoved->GetPoint().Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); m_pPolyBrep->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE
          
          double dRatio = (1.0*lDecimationCount)/lTotalHeadVertices;
          if (m_dPercentOfReduction != 0.0 && dRatio >= m_dPercentOfReduction) {
              break; // Stop because we are done using the m_dPercentOfReduction criteria
          }
          
          if (!(pAttr->m_eVertClass == SM_PVC_SIMPLE ||
              pAttr->m_eVertClass == SM_PVC_INTERIOR ||
              pAttr->m_eVertClass == SM_PVC_BOUNDARY ) ) {
              // This is a corner or something skip it
              continue;
          }

          //double dCurrentValue = pAttr->m_dVertexDistance;

#ifdef SM_DEBUG_CODE
          if ( bDebugMe )
          { SM_DUMP_AND_ASSERT_VALID( m_pPolyBrep ); }
#endif // SM_DEBUG_CODE
          
          SmBoolean bSuccess;
          SmStatus eStat = FillHole( m_pPolyBrep, pVertToBeRemoved, pAttr,
                                     bSuccess, sOrigPolys, sNewPolys,
                                     sSurvivingLoopVertices, sSurvivingLoopAttr );

          SE( eStat );
          if ( eStat != SM_SUCCESS ) 
            { continue; } // was SER.  Never hit.

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
          if (bDebugMe3 /*|| (ii==900 && jj == 47) */) {
              m_pPolyBrep->ValidatePointers();
              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,0); pVertToBeRemoved->GetPoint().Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); m_pPolyBrep->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

          if ( !bSuccess )
            { continue; }

          // Check error bounds for each polygon
          SmBrepCache * pPolyCache = new (*m_pPolyBrep->GetContext()) SmBrepCache(m_pPolyBrep);
          NER(pPolyCache);
          SmObjDelete sCleanup(pPolyCache);
          SER( pPolyCache->BuildPolyTrees( sOrigPolys ));

          SmBoolean bWithinErrBounds = TRUE;
          ULONG kk, ll, lNumNew = sNewPolys.GetSize();
          for(kk=0;kk<lNumNew;kk++)
          {
              SmPolyFace * pF = sNewPolys[kk];
              SER(CheckErrorBounds( pF, pPolyCache, bWithinErrBounds ));
              if ( !bWithinErrBounds )
                { break; }
          }
          
          if ( bWithinErrBounds )
          {
              // Delete original faces.
              // Make this vertex good now but set attributes to recompute.
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
              if (bDebugMe2) {
                  smgfx_Erase();
                  smgfx_SetLook(1,2, 0,0,1);
                  for(kk=0;kk<sOrigPolys.GetSize();kk++){
                      SmPolyFace * pF = sOrigPolys[kk];
                      pF->Draw();
                  }
                  sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 1,0,0);
                  ULONG kkk ;
                  for(kkk=0;kkk<sNewPolys.GetSize();kkk++){
                      sNewPolys[kkk]->Draw();
                  }
                  sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,0,1); m_pPolyBrep->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
#endif // SM_DEBUG_CODE
              
              lDecimationCount++;
              for(kk=0;kk<sOrigPolys.GetSize();kk++)
              {
                  SmPolyFace * pF = sOrigPolys[kk];
                  m_pPolyBrep->DeletePolyFace(pF);
              }

#ifdef SM_DEBUG_CODE
              if ( bDebugMe )
              { SM_DUMP_AND_ASSERT_VALID( m_pPolyBrep ); }
#endif // SM_DEBUG_CODE
              
              // Here let's update the surviving loop vertex attributes
              for(kk=0;kk<sSurvivingLoopAttr.GetSize();kk++)
              {
                  SmDecimateVertexAttr *pLoopAttr = sSurvivingLoopAttr[kk];
                  if (!pLoopAttr) continue;
                  
                  SmPolyVertex *pSurvLoopV = sSurvivingLoopVertices[kk];
                  
                  // Set values of attribute to force recomputation.
                  pLoopAttr->m_eVertClass = SM_PVC_NOT_COMPUTED;
                  pLoopAttr->m_dMaxFaceError = 0.0;
                  pLoopAttr->m_dVertexDistance = -SM_BIG_DOUBLE;
                  SmDecimateVertexAttr *pTmpAttr;

                  if ( ComputeVertexAttributes( pSurvLoopV, pTmpAttr ) != SM_SUCCESS )
                  {
                      m_pPolyBrep->GetPolyVertices(sVerts);
                      ULONG lNumVertices = sVerts.GetSize();
                      for(ll=0; ll<lNumVertices; ll++)
                      {
                          SmDecimateVertexAttr *pDVAttr =
                              (SmDecimateVertexAttr *)(sVerts[ll]->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES));
                          if (pDVAttr)
                            { sVerts[ll]->RemoveAttribute( pDVAttr ); }
                      }
                  }

                  if (pTmpAttr == NULL) {
                      pLoopAttr->m_dVertexDistance = SM_BIG_DOUBLE;
                  }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe5 = FALSE;
                  if (bDebugMe5) {
                      ULONG il ;
                      for(il=0;il<sWorkingGroup.GetSize();il++){
                          if (sWorkingGroup[il]) { sWorkingGroup[il]->Dump(); }
                          else { smos_WriteBuffer(_T("NULL\n")); }
                      }
                  }
#endif // SM_DEBUG_CODE
                  if ( pLoopAttr->m_lSortIndex >= sWorkingGroup.GetSize()+ii )
                    { continue; } // Just ignore vertices not in the group

                  if ( pLoopAttr->m_lSortIndex < ii+jj+1 )
                    { continue; } // Don't worry these have been left behind for some reason.

                  // If we make it to here the vertex is in the group we have to
                  // either determine how to resort it into the group or move it
                  // outside of the group to the end.  This should keep everything
                  // outside of the group non NULL.
                  if ( pLoopAttr->m_dVertexDistance < dHighestValueInGroup )
                  {
                      ULONG lStart, lEnd;
                      char pWork[ sizeof( SmDecimateVertexAttr* ) ];
                      SER( smgu_SortOneModified(
                              sWorkingGroup.GetDataArray(),
                              jj+1,
                              sWorkingGroup.GetSize()-1,
                              pLoopAttr->m_lSortIndex-ii,
                              sizeof( SmDecimateVertexAttr*),
                              sm_CompareVertex, pWork, lStart, lEnd ));

                      if ( lStart > sWorkingGroup.GetSize() || lEnd >= sWorkingGroup.GetSize() )
                        { SE( SM_ERR ); }

                      for(ll=lStart;ll<=lEnd;ll++){
                          if (sWorkingGroup[ll])
                            { sWorkingGroup[ll]->m_lSortIndex = ll+ii; }
                      }
                  }
                  else
                  {
                      // Remove element from working group and put it elsewhere
                      sWorkingGroup.RemoveAt( pLoopAttr->m_lSortIndex-ii, 1 );

                      for(ll=pLoopAttr->m_lSortIndex-ii;ll<sWorkingGroup.GetSize();ll++){
                          if (sWorkingGroup[ll]->m_lSortIndex != SM_BIG_ULONG) {
                              sWorkingGroup[ll]->m_lSortIndex --;
                          }
                          else {
                              SER(SM_ERR);
                          }
                      }
                      sAttr.Add(pLoopAttr); // Stick it on the end for now because it
                      pLoopAttr->m_lSortIndex = sAttr.GetSize() -1;
                      // falls outside of our group
                  }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe6 = FALSE;
                  if (bDebugMe6) {
                      ULONG il ;
                      for(il=0;il<sWorkingGroup.GetSize();il++){
                          if (sWorkingGroup[il]) { sWorkingGroup[il]->Dump(); }
                          else smos_WriteBuffer(_T("NULL\n"));
                      }
                  }
#endif // SM_DEBUG_CODE

              } // end 3rd nested loop (kk) on surviving loop vertex attributes

          } // end if ( bWithinErrBounds )

          else
          {
              // Keep original faces
              for(kk=0;kk<sNewPolys.GetSize();kk++){
                  SmPolyFace * pF = sNewPolys[kk];
                  m_pPolyBrep->DeletePolyFace(pF);
              }
          }
#ifdef SM_VALIDATE_TOPOLOGY
          m_pPolyBrep->ValidatePointers();
#endif

      } // For group elements

      // Move WorkingGroup attributes back to sAttr
      ULONG smg ;
      for(smg=0;smg<sWorkingGroup.GetSize();smg++){
          sAttr[smg+ii] = sWorkingGroup[smg];
          sWorkingGroup[smg] = NULL;
      }
  } // end big loop (ii) on sAttr: for each group 


  // All done.  Remove all attributes from vertices.
  m_pPolyBrep->GetPolyVertices( sVerts );
  lNumVerts = sVerts.GetSize();
  for(ULONG ii=0; ii<lNumVerts; ii++)
  {
      SmDecimateVertexAttr *pAttr = (SmDecimateVertexAttr *)sVerts[ii]->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES);
      // It's possible to have more than one.  [B178]
      while ( pAttr != NULL )
      {
          sVerts[ii]->RemoveAttribute( pAttr, TRUE );
          pAttr = (SmDecimateVertexAttr *)sVerts[ii]->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES);
      }
  }
        
  return SM_SUCCESS;

}  // end SmPolyDecimate::DoMeasuredDecimation

/*******************************************************************//**
PURPOSE: returns 1 when pElem1->GetError  > pElem2->GetError
                    0 when pElem1->GetError == pElem2->GetError
                   -1 when pElem1->GetError  < pElem2->GetError

NOTES:
***********************************************************************/
static int sm_CompareEdges
  (const void *pElem1,   // in : 
   const void *pElem2)   // in : 
{
  SmQuadricEdgeAttr *pQEA1 = *(SmQuadricEdgeAttr**)pElem1 ;
  SmQuadricEdgeAttr *pQEA2 = *(SmQuadricEdgeAttr**)pElem2 ;

  // check input
  if (pQEA1 == NULL) { return 1; }
  if (pQEA2 == NULL) { return 1; }

  // return value
  return(  (pQEA1->GetError() > pQEA2->GetError()) ?  1
         : (pQEA1->GetError() < pQEA2->GetError()) ? -1
         : 0) ;

  //      if ((*((SmQuadricEdgeAttr**)pElem1))->GetError() > 
  //          (*((SmQuadricEdgeAttr**)pElem2))->GetError()) 
  //        {
  //          return 1;
  //        }
  //      if ((*((SmQuadricEdgeAttr**)pElem1))->GetError() <
  //          (*((SmQuadricEdgeAttr**)pElem2))->GetError()) 
  //        {
  //          return -1;
  //        }
  //      return 0;

} // end sm_CompareEdges

/*******************************************************************//**
PURPOSE: Find the Quadric edge attribute on pEdge or any of its radial mates.

NOTES: 
***********************************************************************/
SmQuadricEdgeAttr * SmPolyDecimate::FindAttributeOnAnyEdge
 (SmPolyEdge *pEdge) 
 const
{
  SM_PTR_ARRAY(sRadEdges, SmPolyEdge, 16);         // SmTArray<SmPolyEdge*>

  // get all radial edges
  pEdge->GetAllRadials(sRadEdges) ;

  // for every radial edge mate - search for and return the SM_AI_QUADRIC_EDGE_VALUES attribute
  ULONG ii, lNumEdges = sRadEdges.GetSize() ;
  for(ii=0 ;ii<lNumEdges ;ii++)
    {
      SmPolyEdge *pE = sRadEdges[ii] ;
      SmQuadricEdgeAttr *pRet = (SmQuadricEdgeAttr *)pE->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;
      if (pRet) 
        { return pRet ; }
    }

  // did not find the attribute
  return NULL ;

} // end SmPolyDecimate::FindAttributeOnAnyEdge

/*******************************************************************//**
PURPOSE: Find and remove all the Quadric edge attributes on pEdge 
         and all of its radial mates.

NOTES: 
***********************************************************************/
void SmPolyDecimate::RemoveAttributeOnAnyEdge
 (SmPolyEdge *pPolyEdge)                         // in : target PolyEdge
 const
{
  // locals
  ULONG ii ;
  SM_PTR_ARRAY(sRadEdges, SmPolyEdge, 16);         // SmTArray<SmPolyEdge*>
   

  // get all radial edges
  pPolyEdge->GetAllRadials(sRadEdges) ;
  ULONG lNumEdges = sRadEdges.GetSize() ;

  // for every radial edge mate - search for and return the SM_AI_QUADRIC_EDGE_VALUES attribute
  for(ii=0;ii<lNumEdges;ii++)
    {
      SmPolyEdge        * pRadEdge  = sRadEdges[ii] ;
      SmQuadricEdgeAttr * pEdgeAttr = (SmQuadricEdgeAttr *)pRadEdge->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;

      // when pRadEdge has a SM_AI_QUADRIC_EDGE_VALUES attribute
      while(pEdgeAttr != NULL)
        {
          // remove it
          pRadEdge->RemoveAttribute(pEdgeAttr) ;
          pEdgeAttr = (SmQuadricEdgeAttr *)pRadEdge->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;

        } // end While pRadEdge has SM_AI_QUADRIC_EDGE_VALUES attributes - expect just 1
    } // end iter all Radial Partners

} // end SmPolyDecimate::RemoveAttributeOnAnyEdge

/*******************************************************************//**
PURPOSE: Find all the Quadric edge attributes on pEdge and all of its radial mates.

NOTES: This method is for checking state.
       Every Radial PolyEdge set is supposed to have just one SmQuadricEdgeAttr.
       More than one is a bug.
***********************************************************************/
ULONG SmPolyDecimate::FindAllAttributesOnAnyEdge
 (SmPolyEdge *pPolyEdge,                         // in : target PolyEdge
  SmTArray<SmQuadricEdgeAttr *> *pOptEdgeAttrs)  // out: optional list of all found attributes
                                                 //      default:[NULL]
 const
{
  // locals
  ULONG ii, jj  ;
  SM_PTR_ARRAY(sRadEdges, SmPolyEdge, 16);         // SmTArray<SmPolyEdge*>
  SM_PTR_ARRAY(sAttributes, SmAttribute, 16);      // SmTArray<SmAttribute*>
  SM_PTR_ARRAY(sEdgeAttrs, SmQuadricEdgeAttr, 4) ; // SmTArray<SmQuadricEdgeAttr*>
  SmTArray<SmQuadricEdgeAttr *> *pEdgeAttrs = pOptEdgeAttrs ? pOptEdgeAttrs : &sEdgeAttrs ;

  // init output
  pEdgeAttrs->ReSet() ;

  // get all radial edges
  pPolyEdge->GetAllRadials(sRadEdges) ;
  ULONG lNumEdges = sRadEdges.GetSize() ;

  // for every radial edge mate - search for and return the SM_AI_QUADRIC_EDGE_VALUES attribute
  for(ii=0;ii<lNumEdges;ii++)
    {
      SmPolyEdge *pRadEdge = sRadEdges[ii] ;
      pRadEdge->GetAttributes(sAttributes) ;

      // for every attribute
      for(jj=0;jj<sAttributes.GetSize();jj++)
        {
          SmAttribute *pAttribute = sAttributes[jj] ; 
          if (pAttribute->GetAttributeID() == SM_AI_QUADRIC_EDGE_VALUES)
            {
              pEdgeAttrs->Add((SmQuadricEdgeAttr*)pAttribute) ;
            }
        } // end iter every Attribute
    } // end iter all Radial Partners

  // all done
  return pEdgeAttrs->GetSize() ;

} // end SmPolyDecimate::FindAllAttributesOnAnyEdge

/*******************************************************************//**
PURPOSE: Set and return the edge->SM_AI_QUADRIC_EDGE_VALUES attribute
            containg newly computed optimal point and attribute error values.

NOTES: 
   rpNewAttribute is set on if pEdge and its radial partners did not have 
                  a quadric edge attribute and one had to be created,
                  otherwise it is returned Null.

   A newly created attribute is attached to pEdge.

   The edge->SM_AI_QUADRIC_VERTEX_VALUES attribute's 
                 optimal point and attribute error values are always computed
                 and set.

   Assumes: all Edge->Vertices have SM_AI_QUADRIC_VERTEX_VALUES attributes
***********************************************************************/
SmQuadricEdgeAttr * SmPolyDecimate::GetOrCreateQuadricEdgeAttr
 (SmPolyEdge         * pEdge,           // in : target edge
  SmQuadricEdgeAttr *& rpNewAttribute)  // out: attribute attached to pEdge
{
  // init output
  rpNewAttribute = NULL ;

  // locals
  SmPolyVertex *pVSt  = pEdge->GetStartPolyVertex() ;
  SmPolyVertex *pVEnd = pEdge->GetEndPolyVertex() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if (bDebugMe) 
    {
      smgfx_Erase() ;
      smgfx_SetLook(3,4, 0,0,1) ; pVSt->GetPoint().Draw() ;  sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; pVEnd->GetPoint().Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; pEdge->Draw() ;       sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_pPolyBrep->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
  
  // get vertex quadric error attributes 
  SmQuadricVertexAttr *pStAtt  = (SmQuadricVertexAttr *) pVSt->FindAttribute(SM_AI_QUADRIC_VERTEX_VALUES) ;
  NERN(pStAtt) ;
  SmQuadricVertexAttr *pEndAtt = (SmQuadricVertexAttr *) pVEnd->FindAttribute(SM_AI_QUADRIC_VERTEX_VALUES) ;
  NERN(pEndAtt) ;

  // combine polyEdge->polyVertex quadric error values to make an edge value
  SmQuadricVertexAttr sCombined(SM_AI_QUADRIC_VERTEX_VALUES,SM_AB_TEMP) ;
  pStAtt->Union(*pEndAtt,sCombined) ;

  // Use the combined edge quadric error vals to compute an optimal point
  //   old note: Here we may do some work to change point selection method.
  SmPoint3d sOptimalPoint ;
  SmBoolean bFails ;
  SE( sCombined.ComputeOptimalPosition( sOptimalPoint, bFails )) ;

  // when combined edge quadric error optimal point fails - use vertex point with smallest quadric error value
  if (bFails)
    {
      // get vertex quadric error values
      double dErrorSt  = SM_BIG_DOUBLE ;
      double dErrorEnd = SM_BIG_DOUBLE ;
      SE(sCombined.ComputeError(pVSt->GetPoint(), dErrorSt)) ;
      SE(sCombined.ComputeError(pVEnd->GetPoint(), dErrorEnd)) ;

      // let optimal point = smallest quadric error vertex->point
      if (dErrorSt < dErrorEnd) { sOptimalPoint = pVSt->GetPoint() ; }
      else                      { sOptimalPoint = pVEnd->GetPoint() ; }
    } // end polyEdge->optimal point computation faiure branch

  // set error to computed quadric error for edge and its newly computed optimal point
  double dError ;
  SE( sCombined.ComputeError( sOptimalPoint, dError )) ;

  // when asked, let error = dEdgeLength or
  if ( m_bUseEdgeLengthAsError ) 
    {
        double dEdgeLength = pVSt->GetPoint().DistanceBetween( pVEnd->GetPoint() ) ;
        dError = dEdgeLength ;
    }
  else // error = quadricError + weight * perimeter
    {
      SmPolyEdge * pCW = pEdge->GetCWPolyEdge() ;

      // GWC: this looked wrong - I have a proposal below for a replacement
      // old code
      double dEdgeLengthSq ;
      dEdgeLengthSq  = pVSt->GetPoint(). DistanceBetweenSquared(pVEnd->GetPoint() ) ;
      dEdgeLengthSq += pVSt->GetPoint(). DistanceBetweenSquared(pCW->GetStartPoint()) ;
      dEdgeLengthSq += pVEnd->GetPoint().DistanceBetweenSquared(pCW->GetStartPoint()) ;
      dError = dError + dEdgeLengthSq/1000.0 ;  // Some weight to total edge length of triangle

      // code to consider
      //      double dEdgeLength ;
      //      dEdgeLength  = pVSt->GetPoint(). DistanceBetween(pVEnd->GetPoint() ) ;
      //      dEdgeLength += pVSt->GetPoint(). DistanceBetween(pCW->GetStartPoint()) ;
      //      dEdgeLength += pVEnd->GetPoint().DistanceBetween(pCW->GetStartPoint()) ;
      //      dError = dError + dEdgeLength/30.0 ;  // Give some weight to total edge length of triangle
    }

  // get SM_AI_QUADRIC_EDGE_VALUES edge attribute from pEdge or its radial mate
  SmQuadricEdgeAttr *pAttr = FindAttributeOnAnyEdge( pEdge ) ;

  // when no attribute - add one 
  if (!pAttr) 
    {
      pAttr = new (*pEdge->GetContext()) 
          SmQuadricEdgeAttr(SM_AI_QUADRIC_EDGE_VALUES,SM_AB_STANDALONE_COPY) ;
      rpNewAttribute = pAttr ;
      pEdge->AddAttribute( pAttr ) ;
    }
  NERN(pAttr) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE ;
  if (bDebugMe2) 
    {
      smgfx_Erase() ;
      smgfx_SetLook(3,4, 0,0,0) ; pVSt->GetPoint().Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,0) ; pVEnd->GetPoint().Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,0,1) ; pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; sOptimalPoint.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_pPolyBrep->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // set attribute's optimal point and attribute error 
  pAttr->SetError(dError) ;
  pAttr->SetPoint(sOptimalPoint) ;

  // all done
  return pAttr ;

} // end SmPolyDecimate::GetOrCreateQuadricEdgeAttr

/*******************************************************************//**
PURPOSE: Squeeze an edge (make it zero length) out of the PolyBrep 
  and modify the topology graph as needed to remove degenerate 
  topology objects and preserve decimation attributes. The surviving vertex
  is assigned a new best position.

NOTES: 

GWC: I believe, conceptually, this method squeezes the edge length to zero,
     making faces connected to the edge degenerate and stretching the 
     neighbor faces to fill the hole.  Decimation attributes are moved
     and updated as needed and the topology graph is updated to remove all the degenerate
     objects, leaving a valid polyBrep behind. The squeezedEdge->Vertices are collapsed
     into a single vertex and that vertex is given a new position based
     on the quadric optimal point position rule built into the 
     SmQuadricVertexAttr and SmQuadricEdgeAttr classes.
                                                                
     +---------+---------+                                           +---------+---------+ 
      \       / \       /                                              \       |       /   
       \  1  /   \  2  /    ===> SqueezeEdge(D,pEdgeAttr) ===>           \  1  |  2  /     
        \   /  X  \   /     Delete EdgeD                                   \   |   /       
         \ /       \ /      Delete FaceX                                     \ | /         
          &----D----@       Delete FaceY                                       #     
         / \       / \      set Vertex# = StartVertex&                       / | \         
        /   \  Y  /   \     Delete EndVertex@                              /   |   \       
       /  3  \   /  4  \    set Vertex#.Point = pEdgeAttr->GetPoint      /  3  |  4  \     
      /       \ /       \   Connect Edges/ and Edges\ as appropriate   /       |       \   
     +---------+---------+  Move decimate attribs as appropriate     +---------+---------+ 
                                                                                                        
METHOD ---                  
  1. check that squeeze is ok.  Don't squeeze if squeeze makes any degenerate triangles.
  2. move decimation edge attributes from all edges about to be deleted to radial neighbors
  3. combines vertex quadric error attribute values into one vertex attribute (gets put on surviving vertex)
  4. Delete all faces attached to the edges between edgeToRemove->Start and End vertices
  5. When a vertex survives the squeeze
     5.a attach combined vertex attribute
     5.b build list of moved edges and attributes
***********************************************************************/
SmStatus SmPolyDecimate::SqueezeEdge
 (SmPolyEdge                   * pEdgeToBeRemoved, // in : target edge
  SmQuadricEdgeAttr            * pEdgeAttr,        // in : its SM_AI_QUADRIC_EDGE_VALUES attribute containing
                                                   //      the point loc to be given to the SqueezedEdge->SurvivingVertex
  SmBoolean                    & rbSuccess,        // out: TRUE = edge removed, FALSE = not removed
  SmTArray<SmPolyEdge*>        & rMovedEdges,      // out: list of edges moved
  SmTArray<SmQuadricEdgeAttr*> & rMovedEdgesAttrs) // out: their associated attributes
{
  // init output
  rbSuccess = FALSE ;
  rMovedEdges.ReSet() ;
  rMovedEdgesAttrs.ReSet() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
static ULONG lCount      = 1  ; lCount++  ;
static ULONG lDebugCount = 0  ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      smgfx_Erase() ;
      smgfx_SetLook(3,4, 0,0,1) ; pEdgeToBeRemoved->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; pEdgeAttr->GetPoint().Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_pPolyBrep->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // when asked - never remove boundary edges
  if(pEdgeToBeRemoved->IsLamina() && m_dBoundaryEdgeWeight == 0.0) 
    {
      return SM_SUCCESS ; 
    }

  // polyBrep Tol  GWC: is this correct? Should it be polyEdge tol
  double dTol = pEdgeToBeRemoved->GetPolyBrep()->GetTolerance() ;

  // First check to see that optimal point lies in a reasonable possition
  // relative to the surrounding polygons.

  // pEdgeToBeRemoved vertices
  SmPolyVertex * pStartV = pEdgeToBeRemoved->GetStartPolyVertex() ;
  SmPolyVertex * pEndV   = pEdgeToBeRemoved->GetEndPolyVertex() ;

  // polyEdges between StartV and EndV
  SM_PTR_ARRAY(sRemoveEdges, SmPolyEdge, 16);         // SmTArray<SmPolyEdge*>
  SER(pStartV->FindPolyEdgesBetween(pEndV, sRemoveEdges)) ;
      
  // set sEdgeFaces = All faces on edges between StartV and EndV
  SM_PTR_ARRAY(sEdgeFaces, SmPolyFace, 16);         // SmTArray<SmPolyFace*>
  ULONG ii, lNumEdges = sRemoveEdges.GetSize() ;
  for(ii=0 ;ii<lNumEdges ;ii++)
    {
      SmPolyEdge *pPolyEdge = sRemoveEdges[ii] ;
      sEdgeFaces.AddUnique( pPolyEdge->GetPolyFace() ) ;
    }

  // check all triangles being modified by removing the edge
  // if any will become degenerate due to the deletion, don't delete the edge

  // locals
  ULONG lIndex ;
  SM_PTR_ARRAY(sVertexEdges, SmPolyEdge, 32) ;  // SmTArraySmPolyEdge<*>
  SmPolyVertex * pSurvStartV = NULL ;

  // for every polyEdge attached to pStartV - check that removing edge won't make a degenerate triangle
  pStartV->GetStartingPolyEdges(sVertexEdges) ;
  SmQuadricVertexAttr * pStartAttr = (SmQuadricVertexAttr *)pStartV->FindAttribute(SM_AI_QUADRIC_VERTEX_VALUES) ;
  lNumEdges = sVertexEdges.GetSize() ;
  for(ii=0;ii<lNumEdges;ii++)
    {
      SmPolyEdge *pVertexEdge = sVertexEdges[ii] ;

      // skip Vertex->Edge whose face is connected to the delete Edge or its radial mate 
      if (sEdgeFaces.FindElement( pVertexEdge->GetPolyFace(), lIndex )) 
        {
          continue ;
        }

      // found an edge connected to vertex that won't be deleted

      // pEdgeToBeDeleted->StartVertex is a surviving vertex
      pSurvStartV = pStartV ;

      // vertex->edge->CCW edge locals
      SmPolyEdge *pCCW = pVertexEdge->GetCCWPolyEdge() ;
      SmVector3d  sBin = pCCW->ComputeBinormal() ;
      SmPoint3d   sPnt = pCCW->GetStartPoint() ;
      SmVector3d  sVec = pEdgeAttr->GetPoint() - sPnt ;

      // when new point is on the same line as the remaining neighbor edge - don't remove edge
      // GWC: I think this says - don't remove the edge if that will make a degenerate triangle 
      if ( sVec.Dot(sBin) < dTol ) 
        {
          rbSuccess = FALSE ;
          return SM_SUCCESS ;
        }
    } // end iter edges attached to pStartV

  // Now do end vertex
  SmPolyVertex *pSurvEndV = NULL ;
  pEndV->GetStartingPolyEdges(sVertexEdges) ;
  SmQuadricVertexAttr *pEndAttr = (SmQuadricVertexAttr *)pEndV->FindAttribute(SM_AI_QUADRIC_VERTEX_VALUES) ;
  lNumEdges = sVertexEdges.GetSize() ;
  for(ii=0;ii<lNumEdges;ii++)
    {
      SmPolyEdge *pPolyEdge = sVertexEdges[ii] ;
      
      // skip Vertex->Edge whose face is connected to the delete Edge or its radial mate
      if(sEdgeFaces.FindElement(pPolyEdge->GetPolyFace(),lIndex)) 
        {
          continue ;
        }

      // found an edge that won't be deleted

      // pEdgeToBeDeleted->EndVertex is a surviving vertex 
      pSurvEndV = pEndV ;

      // vertex->edge->CCW edge locals
      SmPolyEdge *pCCW = pPolyEdge->GetCCWPolyEdge() ;
      SmVector3d  sBin = pCCW->ComputeBinormal() ;
      SmPoint3d   sPnt = pCCW->GetStartPoint() ;
      SmVector3d  sVec = pEdgeAttr->GetPoint() - sPnt ;
      
      // when new point is on the same line as the remaining neighbor edge - don't remove edge
      // GWC: I think this says - don't remove the edge if that will make a degenerate triangle 
      if (sVec.Dot(sBin) < dTol) 
        {
          rbSuccess = FALSE ;
          return SM_SUCCESS ;
        }
    } // end iter edges attached to pEndV

  // arrive here when we can do the squeezing

  // for every edge connecting pStartV and pEndV 
  //  - remove neighbor edges from their radial lists and 
  //    move neighborEdge attributes to surviving neighborEdge radial neighbors
  lNumEdges = sRemoveEdges.GetSize() ;
  for(ii=0;ii<lNumEdges;ii++)
    {
      // locals - (since these are triangle there are only two other edges in each face->loop)
      SmPolyEdge        * pRemE     = sRemoveEdges[ii] ;
      SmPolyEdge        * pRemCCW   = pRemE->GetCCWPolyEdge() ;
      SmPolyEdge        * pRemCW    = pRemE->GetCWPolyEdge() ;
      
      // remove any edgeAttr from CCW and CW edges to move them to other edges
      SmQuadricEdgeAttr * pECCWAttr = FindAttributeOnAnyEdge(pRemCCW) ;
      SmQuadricEdgeAttr * pECWAttr  = FindAttributeOnAnyEdge(pRemCW) ;
      RemoveAttributeOnAnyEdge(pRemCCW) ;
      RemoveAttributeOnAnyEdge(pRemCW) ;

      // old code
      // SmQuadricEdgeAttr * pECCWAttr = (SmQuadricEdgeAttr *)pRemCCW->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;
      // SmQuadricEdgeAttr * pECWAttr  = (SmQuadricEdgeAttr *)pRemCW->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;
      // if (pECCWAttr) 
      //   { pRemCCW->RemoveAttribute(pECCWAttr) ; }
      // if (pECWAttr)  
      //   { pRemCW->RemoveAttribute(pECWAttr) ; }

      // when CCW edge is not lamina - 
      if (!pRemCCW->IsLamina()) 
        {
          SmPolyEdge *pCCWRad = pRemCCW->GetRadial() ;

          // gwc: ???, AddAttribute only allows one attribute of type to be added
          if (pECWAttr && pECCWAttr) pCCWRad->AddAttribute(pECCWAttr) ;
          SM_ASSERT_MSG(1 >= FindAllAttributesOnAnyEdge( pCCWRad ), _T("SmPolyDecimate::SqueezePolyEdge error: added SmQuadricEdgeAttr to PolyEdge radial set that has one already")) ; 

          if (pRemCW->IsLamina()) 
            { // Just remove pRemCCW from double linked list
              pRemCCW->m_pLastRadialE->m_pNextRadialE = pRemCCW->m_pNextRadialE ;
              pRemCCW->m_pNextRadialE->m_pLastRadialE = pRemCCW->m_pLastRadialE ;
            }
          else 
            { // Both are manifold or better
              pRemCCW->m_pNextRadialE->m_pLastRadialE = pRemCW->m_pLastRadialE ;
              pRemCW->m_pLastRadialE->m_pNextRadialE = pRemCCW->m_pNextRadialE ;
              pRemCCW->m_pLastRadialE->m_pNextRadialE = pRemCW->m_pNextRadialE ;
              pRemCW->m_pNextRadialE->m_pLastRadialE = pRemCCW->m_pLastRadialE ;
              pRemCW->m_pNextRadialE = pRemCW ; // Make it lamina
              pRemCW->m_pLastRadialE = pRemCW ;
            }
          pRemCCW->m_pLastRadialE = pRemCCW ; // Make it lamina
          pRemCCW->m_pNextRadialE = pRemCCW ;
        } // end CCW edge is not lamina branch
      else // pRemCCW is lamina just link around itself
        { 
          if (pECWAttr) pRemCW->m_pNextRadialE->AddAttribute(pECWAttr) ;
          SM_ASSERT_MSG(1 >= FindAllAttributesOnAnyEdge( pRemCW->m_pNextRadialE ), _T("SmPolyDecimate::SqueezePolyEdge error: added SmQuadricEdgeAttr to PolyEdge radial set that has one already")) ; 
          pRemCW->m_pLastRadialE->m_pNextRadialE = pRemCW->m_pNextRadialE ;
          pRemCW->m_pNextRadialE->m_pLastRadialE = pRemCW->m_pLastRadialE ;
          pRemCW->m_pNextRadialE = pRemCW ; // Make it lamina
          pRemCW->m_pLastRadialE = pRemCW ;
        }
    } // end iter every edge connecting pStartV and pEndV

  //// Remove Vertex Attributes temporarily until we see which vertex survives
  //if (pStartAttr) 
  //  { pStartV->RemoveAttribute(pStartAttr) ; }
  //if (pEndAttr) 
  //  { pEndV->RemoveAttribute(pEndAttr) ; }

  // Set the attributes as the union of both. Only one will survive
  SER(pStartAttr->Union(*pEndAttr,*pStartAttr)) ;
  *pEndAttr = *pStartAttr;

  // Now delete all faces connected to edges between pStartV and pEndV - this should delete face edges and vertices as well.
  ULONG lNumFaces = sEdgeFaces.GetSize() ;
  for(ii=0;ii<lNumFaces;ii++)
    {
      // Delete the Face, Edge, and vertices
      SmPolyFace *pF = sEdgeFaces[ii] ;
      m_pPolyBrep->DeletePolyFace(pF) ;
    }

  // make sure that pSurvStartV points to a surviving vertex if there is one.
  //   (a surviving vertex is attached to an edge that won't be deleted)

  // when both end vertices are attached to edges that are not deleted
  SM_PTR_ARRAY(sEdgesBetween,SmPolyEdge,16) ;
  if ( pSurvStartV && pSurvEndV ) 
    {
      // Glue endV to StartV: move starting edges from pEndV to pStartV
      //   - edges which become zero length are placed on sEdgesBetween.
      SER( m_pPolyBrep->GlueVertices( pStartV, pEndV, sEdgesBetween )) ;

      // GWC??? does the algorithm have to do something for the zero length polyEdges in sEdgesBetween
      //  these should be the Squeeze edge and its radial partner.  
      //  I would think that tese edges have already been deleted as part of the above DeletePolyFace sequence.
    }
  // when only startVertex survives
  else if(pSurvStartV && pSurvEndV == NULL) 
    {
      // EndV automatically deleted by delete face
      pSurvStartV = pStartV ;  // GWC: it already equals this??? Is this a bug??? It's a harmless no op
    }
  // when endVertex survives
  else if(pSurvStartV == NULL && pSurvEndV ) 
    {
      // StartV automatically deleted by del face - use end one instead.
      pSurvStartV = pSurvEndV ;
    }

  // Now that all of the dirty work is done - recompute the edge attributes

  // when a vertex survives the edge deletion
  if (pSurvStartV)
    {
      // SurvivingVertex pEdgeAttr->PointLocation
      pSurvStartV->SetPoint(pEdgeAttr->GetPoint()) ;  // GWC: This is where the surviving vertex is moved

      // for every edge attached to the surviving vertex
      pSurvStartV->GetPolyEdges(sVertexEdges) ;
      lNumEdges = sVertexEdges.GetSize() ;
      for(ii=0;ii<lNumEdges;ii++)
        {
          SmPolyEdge *pE = sVertexEdges[ii] ;
          SmPolyFace *pF = pE->GetPolyFace() ;

// No: this can change the model; don't do that in debug-only.
// #ifdef SM_DEBUG_CODE
//           pF->GetNormal(TRUE,    // in : FALSE = return m_vNormal when available, TRUE = always recompute m_vNormal
//                         FALSE) ; // in : TRUE = increase polyEdge and polyVertex tolerances when needed
// #endif // SM_DEBUG_CODE

          pF->m_dMaxDeviation = -SM_BIG_DOUBLE ;

          // skip edges without an SM_AI_QUADRIC_EDGE_VALUES attribute
          SmQuadricEdgeAttr *pAttr = (SmQuadricEdgeAttr *)pE->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;
          if (!pAttr) 
            {continue ; }

          // remove edge attributes from radial partners - only one edge attribute per radial set
          SmPolyEdge *pRad = pE->GetRadial() ;
          while (pRad != NULL)
            {
              SmQuadricEdgeAttr *pRadAttr = (SmQuadricEdgeAttr *)pRad->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;
              if (pRadAttr) 
                {
                  pRad->RemoveAttribute(pRadAttr) ;
                }
              pRad = pRad->GetRadial() ;
              if (pRad == pE) 
                { break ; }
            }

          // arrive here when an edge with an attribute starts at the surviving attribute
          // add the edge and attribute to the output moved lists.
          //  GWC??? is this logic what's desired.  It's the list of edges on surviving vertices that have attributes.
          //         that's not exactly the same thing as all moved edges, but it may be what the algorithm wants to
          //         track.
          rMovedEdges.Add(pE) ;
          rMovedEdgesAttrs.Add(pAttr) ;

        } // end iter every edge attached to the surviving vertex
    } // end when a vertex survives the edge delete check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE ;
  if (bDebugMe2) 
    {
      // does rMovedEdges contain the edges in sEdgesBetween?
      SmBoolean bYes = TRUE ;
      for(ii=0;ii<sEdgesBetween.GetSize();ii++)
        {
          ULONG nFoundIndex ;
          bYes &= rMovedEdges.FindElement(sEdgesBetween[ii], nFoundIndex) ;
        }
      sEdgesBetween.Dump() ;
      rMovedEdges.Dump() ; 
      
      m_pPolyBrep->ValidatePointers() ;
      smgfx_Erase() ;
      smgfx_SetLook(5,6, 1,0,0) ; pEdgeAttr->GetPoint().Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_pPolyBrep->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done    
  rbSuccess = TRUE ;
  return SM_SUCCESS ;

} // end SmPolyDecimate::SqueezeEdge

/*******************************************************************//**
PURPOSE: Checks PolyBrep during decimation making sure
  allpolyBrep pointers are OK,
  every polyEdge has a SM_AI_QUADRIC_EDGE_VALUES attribute, and
  every polyVertex has SM_AI_QUADRIC_VERTEX_VALUES attribute.

NOTES: returns SM_SUCCESS when everything is OK
                else return SM_ERR

METHOD ---
   checks
   1. m_pPolyBrep->ValidatePointers() ;
   2. every polyEdge
      2.a. polyEdge/Radial pair has a SM_AI_QUADRIC_EDGE_VALUES attribute
      2.b. ever polyEdge->StartVertex has a SM_AI_QUADRIC_VERTEX_VALUES attribute
***********************************************************************/
SmStatus SmPolyDecimate::ValidateQuadric()
{
  // validate PolyBrep
  SER(m_pPolyBrep->ValidatePointers()) ;

  // for every PolyEdge
  SM_PTR_ARRAY(sEdges, SmPolyEdge, 512);         // SmTArray<SmPolyEdge*>
  m_pPolyBrep->GetPolyEdges(sEdges) ;
  ULONG ii, lNumEdges = sEdges.GetSize() ;
  for(ii=0 ;ii<lNumEdges ;ii++)
    {
      SmPolyEdge *pE = sEdges[ii] ;

      // polyEdge/Radial pair has a SM_AI_QUADRIC_EDGE_VALUES attribute
      SmQuadricEdgeAttr *pAttr = (SmQuadricEdgeAttr*)pE->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;
      if (!pAttr) 
        {
          SmPolyEdge *pRadial = pE->GetRadial() ;
          if (!pRadial) {SER(SM_ERR) ;}
          pAttr = (SmQuadricEdgeAttr*)pRadial->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;
          if (!pAttr) {SER(SM_ERR) ;}
        }

      // polyEdge->StartVertes has a SM_AI_QUADRIC_VERTEX_VALUES attribute
      SmPolyVertex *pVHead = pE->GetStartPolyVertex() ;
      SmQuadricVertexAttr *pVAttr = (SmQuadricVertexAttr *)pVHead->FindAttribute(SM_AI_QUADRIC_VERTEX_VALUES) ;
      if (!pVAttr) 
        {
          SER(SM_ERR) ;
        }
      
    }

  // all done
  return SM_SUCCESS ;

} // end SmPolyDecimate::ValidateQuadric


/*******************************************************************//**
PURPOSE: Static Shell Sort Function for Sort for 
            Ascending Order of Quadric Edges Attributes

NOTES: Sorts the edges by the pEdge->GetError() value
***********************************************************************/
static void smgu_ShellSort_QuadricEdge
 (SmQuadricEdgeAttr **aQuadEdgeAttr, // in : array of attributes to sort
  ULONG               lCount )       // in : number of elements in array
{
  // locals
  int ii, bFlag = 1, lNumLength = lCount ;
  int dd = lNumLength ;
  SmQuadricEdgeAttr *temp ;

  // sorts remain
  while ( bFlag || (dd > 1) )  // boolean bFlag (true when not equal to 0)
    {
      bFlag = 0 ;           // reset bFlag to 0 to check for future swaps
      dd = (dd+1) / 2 ;
      for(ii=0 ;ii<(lNumLength-dd) ;ii++)
        {
          // skip NULL members
          if ( aQuadEdgeAttr[ ii+dd ] == NULL || aQuadEdgeAttr[ii] == NULL )
            { continue ; }

          // ascending order test
          if (aQuadEdgeAttr[ ii+dd ]->GetError() < aQuadEdgeAttr[ii]->GetError()) 
            {
               // swap out of order elements
               temp = aQuadEdgeAttr[ ii+dd ] ;      // swap positions ii+dd and ii
               aQuadEdgeAttr[ ii+dd ] = aQuadEdgeAttr[ii] ;
               aQuadEdgeAttr[ii] = temp ;
               bFlag = 1 ;                  // tells swap has occurred
            } // end out of order check
        } // end iter array at dd spacing
    } // end while dd spacing is bigger than 1 or elements still being swapped

  // all done
  return ;

} // end static void smgu_ShellSort_QuadricEdge

/*******************************************************************//**
PURPOSE: Very fast decimate operator based on edge contraction to
  reduce the polygon count of a given brep by the amount determined
  by the decimate criteria stored in the SmPolyDecimate object.  

NOTES: 
  For a more precise way to control error during decimation 
    use DoMeausuredDecimation.
  
  Vertex positions are moved.

  Users may obtain view-based decimation by specifing 'eye' or 'view' vector
  depending upon projection type (perspective or parallel projection.)

  increments unlocked mark value
METHOD --- Break every polygon up into triangles - only all-triangle polyBreps are decimated

           Assign every vertex and edge a quadric error as implemented in 
           SmQuadricVertexAttr and SmQuadricEdgeAttr.

           Order the edges based on error size from smallest to largest.
             For performance, a very clever scheme using a spatial decomposition tree
             and small working groups identifies a sequence of next lowest error edges.

           While(more edges should be decimated)
             { SqueezeEdge(EdgeWithlowestError)

***********************************************************************/
SmStatus SmPolyDecimate::DoQuadricDecimation
 (SmVector3d * pOptEyeOrViewVector,       // in : orientation data for view-based decimation, 
                                          //      NULL = no view-based decimation.
  SmBoolean    bOptIsParallelProjection)  // in : TRUE  = m_pEyeOrViewVector represents the view vector,
                                          //      FALSE = m_pEyeOrViewVector represents the eye position
                                          // note: increments unlocked mark value
{ 
  // side effect: store view-based decimation inputs
  m_pEyeOrViewVector    = pOptEyeOrViewVector ;
  m_bParallelProjection = bOptIsParallelProjection ;

  // when given, unitize m_pEyeOrViewVector 
  if ( m_pEyeOrViewVector && m_bParallelProjection ) 
    {
      SER(m_pEyeOrViewVector->Unitize()) ;
    }

  // locals
  ULONG ii, jj, kk ;
  SM_PTR_ARRAY(sPolyEdges,     SmPolyEdge, 512) ; // SmTArray<SmPolyEdge*> 
  SM_PTR_ARRAY(sPolyFaceEdges, SmPolyEdge, 512) ; // SmTArray<SmPolyEdge*> 
  SM_PTR_ARRAY(sOrigPolyFaces, SmPolyFace, 512) ; // SmTArray<SmPolyFace*> 
  SM_PTR_ARRAY(sNewPolyFaces,  SmPolyFace, 512) ; // SmTArray<SmPolyFace*> 
  ULONG lTotalHeadVertices = 0 ;    

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe  = FALSE ;
SmBoolean bDebugMe1 = FALSE ;
  ULONG il ;
#endif // SM_DEBUG_CODE

  // every PolyFace
  m_pPolyBrep->GetPolyFaces( sOrigPolyFaces ) ;
  ULONG lNumOrig = sOrigPolyFaces.GetSize() ;

  // for every PolyFaces - tessellate nonTriangles into triangles
  for(ii=0;ii<lNumOrig;ii++)
    {
      SmPolyFace *pOrigPolyFace = sOrigPolyFaces[ii] ;
      pOrigPolyFace->GetNonDegeneratePolyEdges(sPolyFaceEdges) ;
      // used to be: pOrigPolyFace->GetPolyEdges(sPolyFaceEdges) ;
      if (sPolyFaceEdges.GetSize() > 3) 
        {
          // tessellate by adding a sequence of 'best' split lines to maximize final triangle aspect ratios
          pOrigPolyFace->TriangulateNonPlanar(sNewPolyFaces) ;
        }
    } // end iter every polyface - breaking polygons into triangles

  // reload PolyFaces - all polyfaces are now triangles
  m_pPolyBrep->GetPolyFaces( sOrigPolyFaces ) ;
  lNumOrig = sOrigPolyFaces.GetSize() ;

  // now set up temporary quadric vertex error attributes
  SM_PTR_ARRAY(sVertAttr, SmQuadricVertexAttr, 16);         // SmTArray<SmQuadricVertexAttr*>
  SmObjsDelete<SmQuadricVertexAttr*> sCleanVertAttr(&sVertAttr) ;

  // for every polyface - add up quadric vertex error values
  for(ii=0;ii<sOrigPolyFaces.GetSize();ii++)
    {
      SmPolyFace *pPolyFace = sOrigPolyFaces[ii] ;

      // triangle locals
      SmVector3d sNorm ;
      SmPoint3d  sCent ;
      double     dArea ;
      SER( SmPolyDecimate::GetTriangularFaceData( pPolyFace, // in : target polyface                                                      
                                                  sNorm,     // out: normal = unitized( cross(Edge0Dir, -Edge2Dir) ) (degNormal = [0,0,1])
                                                  sCent,     // out: center = (Pt0 + Pt1 + Pt2) / 3.0                                     
                                                  dArea )) ; // out: area   = (dBaseLength * dHeight) / 2.0; (degArea = 0.0)              
      
      // every PolyFace->PolyEdge
      pPolyFace->GetNonDegeneratePolyEdges( sPolyFaceEdges ) ;
      // used to be: pPolyFace->GetPolyEdges( sPolyFaceEdges ) ;
      ULONG lNumEdges = sPolyFaceEdges.GetSize() ;

      // set up vertex attributes

      // for every triangle edge->StartVertex - add up quadric startVertex error data 
      // GWC: I think the quadric vertex error is a quality measure assigned to each vertex
      for(jj=0;jj<lNumEdges;jj++)
        {
          SmPolyEdge *pPolyEdge = sPolyFaceEdges[jj] ;

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              smgfx_Erase() ;
              smgfx_SetLook(3,4, 0,0,1) ; pPolyEdge->Draw(FALSE,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 1,0,0) ; pPolyFace->Draw(FALSE,TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; m_pPolyBrep->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          SmPolyVertex        * pPolyEdgeVertex = pPolyEdge->GetStartPolyVertex() ;
          SmQuadricVertexAttr * pAttr = (SmQuadricVertexAttr *) pPolyEdgeVertex->FindAttribute(SM_AI_QUADRIC_VERTEX_VALUES) ;

          // when vertex has no SM_AI_QUADRIC_VERTEX_VALUES attribute - add one
          if (!pAttr)
            {
              lTotalHeadVertices ++ ;
              pAttr = new (*pPolyEdgeVertex->GetContext()) SmQuadricVertexAttr(SM_AI_QUADRIC_VERTEX_VALUES,
                                                                               SM_AB_COPY) ; 
              NER(pAttr) ;
              pPolyEdgeVertex->AddAttribute(pAttr) ;
            }

          // when edge is lamina or is sihouette (view-based decimation) 
          if (    pPolyEdge->IsLamina()
              // || pPolyEdge->GetIndexValue() == SM_UNDEF_ULONG  // GWC: bug - m_lIndexValue not set in this routine - don't check it
               || TestSilhouetteEdge(pPolyEdge) == TRUE)
            {
              // Treat these boundary edges for constrained optimization differently.
              // Do this by adding in extra vertex quadric error values to both the start and end vertices.
              //   (error values based on square length of the edge)
              
              // edge locals
              double     dEdgeLengthSq = pPolyEdge->GetStartPoint().DistanceBetweenSquared(pPolyEdge->GetEndPoint()) ;
              SmVector3d sBinormal     = pPolyEdge->ComputeBinormal() ;
              SER(sBinormal.Unitize()) ;
              SmPolyVertex        * pPolyEdgeVertexEnd = pPolyEdge->GetEndPolyVertex() ;
              SmQuadricVertexAttr * pPolyEdgeEndAttr   = (SmQuadricVertexAttr *) pPolyEdgeVertexEnd->FindAttribute(SM_AI_QUADRIC_VERTEX_VALUES) ;

              // when endVertex has no SM_AI_QUADRIC_VERTEX_VALUES attribute - add one
              if (!pPolyEdgeEndAttr) 
                {
                  lTotalHeadVertices ++ ;
                  pPolyEdgeEndAttr = new (*pPolyEdgeVertexEnd->GetContext()) SmQuadricVertexAttr(SM_AI_QUADRIC_VERTEX_VALUES,
                                                                            SM_AB_COPY) ; 
                  NER(pPolyEdgeEndAttr) ;
                  pPolyEdgeVertexEnd->AddAttribute(pPolyEdgeEndAttr) ;
                }

              // Accumulate quadric error weights for end vertex
              pPolyEdgeEndAttr->AddWeightedTriangle( sNorm,     pPolyEdge->GetEndPoint(),   dArea ) ;
              pPolyEdgeEndAttr->AddWeightedTriangle( sBinormal, pPolyEdge->GetEndPoint(),   dEdgeLengthSq ) ;

              // Accumulate 2nd quadric error weight for start vertex
              pAttr->AddWeightedTriangle( sBinormal, pPolyEdge->GetStartPoint(), dEdgeLengthSq ) ;
            
            } // end edge is lamina, silhouette, or SM_BIG_ULONG index triangle check to add in extra error at start and end vertices

          // Accumulate error weight for start vertex
          pAttr->AddWeightedTriangle( sNorm, pPolyEdge->GetStartPoint(), dArea ) ;

        } // end iter every edge->StartVertex - adding up quadric start vertex error data
    } // end iter every polyface (all polyfaces known to be triangles) building quadric Vertex attribute data

  // Now set up temporary quadric edge Attributes
  SM_PTR_ARRAY(sEdgeAttrs, SmQuadricEdgeAttr, 16);         // SmTArray<SmQuadricEdgeAttr*>
  SmObjsDelete<SmQuadricEdgeAttr*> sCleanEdgeAttr(&sEdgeAttrs) ;

  // increment the context->m_lCurrentMark2 value
  SmNewMarkAndLock sMarkLock( m_pPolyBrep->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;

  // track the range of edge quadric error values
  SmExtent1d sErrIvl ;

  // for every PolyBrep->PolyEdge - compute and save a quadric optimal point and error value
  m_pPolyBrep->GetNonDegeneratePolyEdges( sPolyEdges ) ;
  ULONG lNumEdges = sPolyEdges.GetSize() ;
  for(ii=0;ii<lNumEdges;ii++)
    { 
      SmPolyEdge *pEdge = sPolyEdges[ii] ;

      // skip marked edges
      if (pEdge->IsMarked(eMarkType))
        { continue ; }

      // Compute pEdge's quadric error optimal point and error values
      //   (stored in the pEdge->SM_AI_QUADRIC_EDGE_VALUES attribute)
      SmQuadricEdgeAttr *pNewAttr ;
      SmQuadricEdgeAttr *pAttr = GetOrCreateQuadricEdgeAttr( pEdge, pNewAttr ) ;

      // bound giant errors
      if ( smos_Fabs( pAttr->GetError() ) > 1.0/SM_EFF_ZERO ) 
        {
          //  SE(SM_ERR) ;
          pAttr->SetError( 1.0 / SM_EFF_ZERO ) ;
        }

      // save the interval of error values
      sErrIvl.AddValue( pAttr->GetError() ) ;

      // when a new edge attribute was created - add it to the sEdgeAttrs list
      if(pNewAttr )
         { sEdgeAttrs.Add( pNewAttr ) ; }

      // mark pEdge and its radial partner
      pEdge->Mark( eMarkType ) ;
      SmPolyEdge *pRadial = pEdge->GetRadial() ;
      if ( pRadial )
        { pRadial->Mark( eMarkType ) ; }

    } // end iter every PolyBrep->PolyEdge computing and saving quadric optimal point and error values

  // arrive here after 
  //   every polyVertex has an SM_AI_QUADRIC_VERTEX_VALUES attribute with current
  //                    quadric error parameters (based on view orientation when asked)
  //   every polyEdge/radial partner pair have a SM_AI_QUADRIC_EDGE_VALUES attribute with current
  //                    optimal point and error values based on the edge's combined vertex quadric values.

  // Use a tree using just the x axis to sort the attributes by error value bucket sort.
  SmExtent3d sBBox(SmPoint3d( sErrIvl.GetMin()-SM_EFF_ZERO,        -SM_EFF_ZERO,-SM_EFF_ZERO),
                   SmPoint3d( sErrIvl.GetMax()+sErrIvl.GetLength(), SM_EFF_ZERO, SM_EFF_ZERO) ) ;
  SmExtent1d sTreeIvl( sBBox.GetMin().x, sBBox.GetMax().x ) ;
  ULONG      lNumPerBlock = smos_Max(500,sPolyEdges.GetSize()) / 10 ;

  // alloc and init a temporary tree
  SmTree    * pSortTree = new (*m_pPolyBrep->GetContext()) SmTree(sBBox,lNumPerBlock,lNumPerBlock/10) ;
  SmObjDelete sCleanTree(pSortTree) ;

  pSortTree->SetMaxElementsPerNode(400) ;
  pSortTree->SetMinSizeRatio(SM_BIG_DOUBLE) ; // Limitless subdivision
  pSortTree->SetNodeReduction(0.5) ;

  // for every pEdge->attribute - add attributes (as points in the SortTree) that are not too big
  ULONG lNumAttrs = sEdgeAttrs.GetSize() ;
  for(ii=0;ii<lNumAttrs;ii++)
    {
      SmQuadricEdgeAttr *pAttr = sEdgeAttrs[ii] ;

      // label attribute as unsorted
      pAttr->SetSortIndex(SM_BIG_ULONG) ; 

      double dErr = pAttr->GetError() ;

      // skip attributes when limiting max vertex error and
      //                      this error is bigger than the max allows
      if(   m_dMaximumError > 0.0 
         && dErr            > m_dMaximumError) 
        { continue ; }

      // Add a point representing the attribute into the tree - split nodes as needed
      SmPoint3d  sPnt(dErr,0,0) ;
      SmExtent3d sAttrBBox(sPnt,sPnt) ;
      pSortTree->AddToSpatialTree( sAttrBBox, pAttr ) ;

  } // end iter every attribute adding a point object to the sort tree

  // Now use the SortTree to do an efficient sorting and resorting of the edge attributes
  //  during decimation to quickly take the best target edge for decimation each time.

  // Try going 500 at a time to see if sorting speed improves
#define SORT_GROUP2 500

  // locals
  ULONG lDecimationCount = 0 ;
  SM_PTR_ARRAY(sWorkingGroup,    SmQuadricEdgeAttr, SORT_GROUP2); // SmTArray<SmQuadricEdgeAttr*>
  SM_PTR_ARRAY(sMovedEdges,      SmPolyEdge,        256);         // SmTArray<SmPolyEdge*>
  SM_PTR_ARRAY(sMovedEdgesAttrs, SmQuadricEdgeAttr, 256);         // SmTArray<SmQuadricEdgeAttr*>
  SM_PTR_ARRAY(sUsers,           SmAObject,         256);         // SmTArray<SmAObject*>
  SM_PTR_ARRAY(sObjects,         SmObjectList,      256) ;        // SmTArray<SmObjectList*>
  SM_PTR_ARRAY(sOutsideTreeAttr, SmQuadricEdgeAttr, 256) ;        // SmTArray<SmQuadricEdgeAttr*>

  // Loop over all nodes in pSortTree - squeezing edges until decimation hits a exit criteria 
  SmTreeNode *pNode = pSortTree->GetFirstLeafNode() ;
  while(pNode != NULL)
    {
      sWorkingGroup.ReSet() ;

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          pNode->m_sBBox.Dump() ;
        }
#endif // SM_DEBUG_CODE

      // load up the working group with 100 polyEdges with the smallest quadric errors.
      // This is done by visting the leafNodes in sequence looking at leafNode->Object->Attributes->errors. 
      //  Since the errors can change as decimation proceeds allow
      //  those objects to be resorted if their attr->errors fall outside
      //  of the current node's error max or outside the tree's error interval.

      // GWC_NEEDS_WORK NEXT_LINE_MAGIC_NUMBER_MAY_NEED_REPLACING GWC_LINE ;

      while(   (sWorkingGroup.GetSize() < 100)  // GWC: is this a bug? should this 100 be SORT_GROUP2?
            && (pNode!=NULL))
        {
          // when an Outside tree attributes are found
          if ( sOutsideTreeAttr.GetSize() > 0 )
            {
              // Rehash tree by adding outside attributes to SortTree 
              // and start from beginning be setting pNode back to GetFirstLeaf
              lNumAttrs = sOutsideTreeAttr.GetSize() ;
              for(ii=0;ii<lNumAttrs;ii++)
                {
                  SmQuadricEdgeAttr *pAttr = sOutsideTreeAttr[ii] ;
                  double dErr = pAttr->GetError() ;
                  SmPoint3d sPnt(dErr,0,0) ;
                  SmExtent3d sAttrBBox(sPnt,sPnt) ;
                  pSortTree->AddToSpatialTree(sAttrBBox,pAttr) ;
                }
              sOutsideTreeAttr.ReSet() ;
              pNode = pSortTree->GetFirstLeafNode() ;

            } // end outside tree attribute check

          // current node locals
          pNode->GetObjectList( sObjects ) ;
          pNode->m_pData = NULL ; // Clear object list for now
          SmExtent1d sErrInterval( pNode->m_sBBox.GetMin().x, pNode->m_sBBox.GetMax().x ) ;

          // for every node->Object - add object->Attribute to WorkingGroup, pSortTree, or sOutsideTreeAttr list
          ULONG lNumObjs = sObjects.GetSize() ;
          for(ii=0;ii<lNumObjs;ii++)
            {
              SmObjectList *pObjList = sObjects[ii] ;

              // extract the attribute and its Error
              SmQuadricEdgeAttr *pAttr    = (SmQuadricEdgeAttr*)pObjList->m_pObject ;
              double             dErr     = pAttr->GetError() ;

              // when dErr is less this node's ErrIvl max - add attr to working group
              if (dErr <= sErrInterval.GetMax())
                {
                  sWorkingGroup.Add(pAttr) ;
                }
              // else when dErr is in this tree's errIvl - add attribute spot to SortTree 
              else if ( sTreeIvl.ContainsValue(dErr) ) 
                {
                  pAttr->SetSortIndex( SM_BIG_ULONG ) ; // Means it is not sorted
                  SmPoint3d  sPnt( dErr, 0, 0 ) ;
                  SmExtent3d sAttrBBox( sPnt, sPnt ) ;
                  pSortTree->AddToSpatialTree( sAttrBBox, pAttr ) ;
                }
              else // found an error outside the current tree interval - add attr to sOutsideTreeAttr list
                {
                  // Outside of tree interval it will cause a resorting.
                  pAttr->SetSortIndex( SM_BIG_ULONG ) ; // Means it is not sorted
                  sOutsideTreeAttr.Add( pAttr ) ;
                }
            }  // end iter every object in current pNode

          // When all object->attributes were inside the existing tree ivl - move on to the next LeafNode
          if ( sOutsideTreeAttr.GetSize() == 0 ) 
            {
              pNode = pSortTree->GetNextLeafNode( pNode ) ;
            }

        } // end while working group < 100 and pNode != NULL

      // If there is nothing in this group just go to next node
      if ( sWorkingGroup.GetSize() == 0 ) 
        {
          continue ;
        }

      // Sort relatively small sWorkingGroup by member pAttrib->GetError() values
      smgu_ShellSort_QuadricEdge( sWorkingGroup.GetDataArray(), sWorkingGroup.GetSize() ) ;

      // for every attr in the working group - assign a SortIndex
      ULONG lNumGroup = sWorkingGroup.GetSize() ;
      for(ii=0;ii<lNumGroup;ii++)
        {
          sWorkingGroup[ii]->SetSortIndex(ii) ;
        }

#ifdef SM_DEBUG_CODE
      if (bDebugMe) 
        {
          for(il=0;il<sWorkingGroup.GetSize();il++)
            {
              if (sWorkingGroup[il]) sWorkingGroup[il]->Dump() ;
            }
        }
#endif // SM_DEBUG_CODE
      
      // get this working group's err interval - pNode is NULL when working group has all remaining attributes
      double dLowestValueInGroup  = sWorkingGroup[0]->GetError() ;
      double dHighestValueInGroup = (pNode != NULL) ? sWorkingGroup.GetLast()->GetError() : SM_BIG_DOUBLE ;
      
      // for every WorkingGroup attr - 
      //    note, must use GetSize() because sWorkingGroup can change.
      for(ii=0;ii<sWorkingGroup.GetSize();ii++)
        {
          SmQuadricEdgeAttr *pAttr = sWorkingGroup[ii] ;
          
          // skip attributes not attached to an edge
          pAttr->GetUsers( sUsers ) ;
          if ( sUsers.GetSize() == 0 ) 
            { continue ; }

          // Select edge to be removed
          SmPolyEdge * pEdgeToBeRemoved = (SmPolyEdge*)sUsers[0] ;
          
          //  Assumes that the attribute is always on the head edge.
          if(pAttr != FindAttributeOnAnyEdge( pEdgeToBeRemoved ))
            { SM_ASSERT(pAttr == FindAttributeOnAnyEdge( pEdgeToBeRemoved )) ; }

#ifdef SM_DEBUG_CODE
          if (bDebugMe1) 
            {
              sUsers.Dump() ;
              ValidateQuadric() ; // PolyBrep pointer ok, polyEdges and polyVerts all have appropriate quadric attributes

              smgfx_Erase() ;
              smgfx_SetLook(3,4, 1,0,0) ; pEdgeToBeRemoved->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,1) ; m_pPolyBrep->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE
          
          // current decimation ratio (based on total number of vertex attributes created)
          double dRatio = (double)lDecimationCount / (double)lTotalHeadVertices ;
          
          // exit case - decimated to specified reduction ratio
          if(m_dPercentOfReduction != 0.0 && dRatio >= m_dPercentOfReduction)
            {
              break ; // Stop because we are done using the m_dPercentOfReduction criteria
            }
          
          // exit case - using Length as an error and current error is bigger than maximum error
          double dCurrentValue = pAttr->GetError() ;
          if ( m_bUseEdgeLengthAsError && dCurrentValue > m_dMaximumError ) 
            {
              break ;
            }
          
          // Remove target edge when doing so will not create any degenerate triangles.
          //   The neighboring mesh is modified as if the target edge were squeezed to zero length
          //   pulling its end vertices and the neighbor edges attached to those vertices together.
          // The target edge, its radial mate, its endVertex, and the faces attached to those edges 
          //   are cut out of the mesh and deleted rather than being left to become degenerate.
          // The targetEdge->StartVertex survives and is moved (point = pAttr->GetPoint()).
          //   Decimate attributes are moved as needed to preserve them.
          SmBoolean bSuccess ;
          SER( SqueezeEdge( pEdgeToBeRemoved,     // in : target edge                                                   
                            pAttr,                // in : its SM_AI_QUADRIC_EDGE_VALUES attribute containing            
                                                  //      the point loc to be given to the SqueezedEdge->SurvivingVertex
                            bSuccess,             // out: TRUE = edge removed, FALSE = not removed                      
                            sMovedEdges,          // out: list of edges moved                                           
                            sMovedEdgesAttrs )) ; // out: their associated attributes                                   
#ifdef VALIDATE_QUADRIC
          ValidateQuadric() ; // check polyBrep pointers, every polyEdge and polyVertex has a decimate attribute
#endif
#ifdef SM_DEBUG_CODE
          if (bDebugMe1) 
            {
              ValidateQuadric() ; // PolyBrep pointer ok, polyEdges and polyVerts all have appropriate quadric attributes

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; m_pPolyBrep->Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // when an edge can't be squeezed - label it and move to next edge
          if (!bSuccess) 
            {
              pAttr->SetSortIndex(SM_BIG_ULONG) ; // label this as a bad edge and skip it
              // From now on.  
              continue ;
            }
          
          // count the removed edges
          lDecimationCount ++ ;

          // Here let's update the surviving edge attributes

          // for every edge moved with a decimate attribute
          ULONG lNumMoved = sMovedEdges.GetSize() ;
          for(jj=0;jj<lNumMoved;jj++)
            {
              SmPolyEdge *pMovedE = sMovedEdges[jj] ;

              // get old and new quadric errors
              SmQuadricEdgeAttr * pNewAttr ;
              SmQuadricEdgeAttr * pMovedEAttr = GetOrCreateQuadricEdgeAttr( pMovedE, pNewAttr ) ;
              if ( sMovedEdgesAttrs[jj] != pMovedEAttr )
                {
                  // SER(SM_ERR) ; // never hit.
                  SE(SM_ERR) ;
                  continue ;
                }
#ifdef SM_DEBUG_CODE
              double dOldError = sMovedEdgesAttrs[jj]->GetError();
              if (bDebugMe1) 
                {
                  double dNewError = pMovedEAttr->GetError() ;
                  smos_WriteDouble(_T("  Original Error = ,"),dOldError) ;
                  smos_WriteDouble(_T("  New Error = "),dNewError) ;
                  smos_WriteBuffer(_T("\n")) ;
                  
                  if (jj==0) 
                    { smgfx_Erase() ; 
                      smgfx_SetLook(1,2, 0,0,1) ; m_pPolyBrep->Draw() ; sm_GraphicsLoop() ;
                    }
                  smgfx_SetLook(3,4, 0,0,1) ; pMovedE->Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(5,6, 0,0,1) ; pMovedEAttr->GetPoint().Draw() ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE

              // Set values of attribute to force recomputation.
#ifdef SM_DEBUG_CODE    
              if (bDebugMe) 
                {
                  for(il=0;il<sWorkingGroup.GetSize();il++)
                    {
                      if (sWorkingGroup[il]) sWorkingGroup[il]->Dump() ;
                      else smos_WriteBuffer(_T("NULL\n")) ;
                    }
                }
#endif // SM_DEBUG_CODE

              // ignore vertices not in the group
              if(   pMovedEAttr->GetSortIndex() == SM_BIG_ULONG
                 || pMovedEAttr->GetSortIndex() < ii+1 
                 || pMovedEAttr->GetSortIndex() >= sWorkingGroup.GetSize()) 
                {
                  continue ; // Not in the working group skip it
                }

              // 
              if (pMovedEAttr->GetError() < dLowestValueInGroup) 
                {
//                    SE(SM_ERR) ; // might be impossible
                }

              // If we make it to here the vertex is in the group we have to
              // either determine how to resort it into the group or move it
              // outside of the group to the end.  This should keep everything
              // outside of the group non NULL.
              if(pMovedEAttr->GetError() < dHighestValueInGroup)
                {
                  ULONG lStart, lEnd ;
                  char pWork[ sizeof(SmQuadricEdgeAttr*) ] ;

                  // Restore the sort order after one element value has been modified. 
                  SER( smgu_SortOneModified( sWorkingGroup.GetDataArray(), // in : the ordered array with one unsorted elem                 
                                             ii+1,                         // in : index of first element of pbase array to use in sort.    
                                             sWorkingGroup.GetSize()-1,    // in : Size of array - must be greater than beggining           
                                             pMovedEAttr->GetSortIndex(),  // in : index of modified element.                               
                                             sizeof(SmQuadricEdgeAttr*),   // in : Size of element                                          
                                             sm_CompareEdges,              // in : ptr to compare function                                  
                                             pWork,                        // in : NO LONGER USED                                           
                                             lStart,                       // out: index Start of modified region                           
                                             lEnd )) ;                     // out: index End of modified region                             
                  // sorted range must be in the working group size
                  if(   lStart >  sWorkingGroup.GetSize()
                     || lEnd   >= sWorkingGroup.GetSize() )
                    { SE(SM_ERR) ; }

                  // reset the SortIndex values in the modified region of the Working Group
                  for(kk=lStart;kk<=lEnd;kk++)
                    {
                      if ( sWorkingGroup[kk] != NULL ) 
                        {
                           sWorkingGroup[kk]->m_lSortIndex = kk ;
                        }
                    }
                } // end movedEdge->Error < dHighestValueInGroup branch
              else // movedEdge->Error >= dHighestValueInGroup
                {
                  // Remove element from working group and put it elsewhere
                  sWorkingGroup.RemoveAt( pMovedEAttr->GetSortIndex(), 1 ) ;
                  ULONG lGroupSize = sWorkingGroup.GetSize() ;
                  for(kk=pMovedEAttr->GetSortIndex();kk<lGroupSize;kk++)
                    {
                      if (sWorkingGroup[kk]->m_lSortIndex != SM_BIG_ULONG) 
                        {
                          sWorkingGroup[kk]->m_lSortIndex -- ;
                        }
                    }
                  pMovedEAttr->SetSortIndex( SM_BIG_ULONG ) ; // Means it is not sorted
                  double dErr = pMovedEAttr->GetError() ;

                  // when movedEdge->Error is in TreeIvl - add it to the SortTree
                  if ( sTreeIvl.ContainsValue(dErr) ) 
                    {
                      SmPoint3d sPnt( dErr, 0, 0 ) ;
                      SmExtent3d sAttrBBox( sPnt, sPnt ) ;
                      pSortTree->AddToSpatialTree( sAttrBBox, pMovedEAttr ) ;
                    }
                  else // movedEdge->Error is out of TreeIvl - need to rehash the tree
                    {
                      sOutsideTreeAttr.Add( pMovedEAttr ) ;
                    }
                } // end movedEdge->Error >= dHighestValueInGroup branch

#ifdef SM_DEBUG_CODE
              if (bDebugMe) 
                {
                  for(il=0;il<sWorkingGroup.GetSize();il++)
                    {
                      if (sWorkingGroup[il]) { sWorkingGroup[il]->Dump() ; }
                      else { smos_WriteBuffer(_T("NULL\n")) ; }
                    }
                }
#endif // SM_DEBUG_CODE

            } // end iter(jj) each moved edge due to a Edge squeeze
        } // End iter(ii) each WorkingGroup element
    } // End while looping over all nodes in pSortTree.

  // All done.  Remove all attributes from edges and vertices.

  // for every polyVert
  SM_PTR_ARRAY(sVerts, SmPolyVertex, 512);         // SmTArray<SmPolyVertex*>
  m_pPolyBrep->GetPolyVertices( sVerts ) ;
  ULONG lCount = sVerts.GetSize() ;
  for(ii=0;ii<lCount;ii++)
    {
      SmQuadricVertexAttr *pAttr = (SmQuadricVertexAttr *) sVerts[ii]->FindAttribute(SM_AI_QUADRIC_VERTEX_VALUES) ;

      // It's possible to have more than one.  [B178]
      while ( pAttr != NULL )
        {
          // remove the Vert decimate attribute
          sVerts[ii]->RemoveAttribute(pAttr) ;
          pAttr = (SmQuadricVertexAttr *) sVerts[ii]->FindAttribute(SM_AI_QUADRIC_VERTEX_VALUES) ;
        }
    }

  // for every polyEdge
  m_pPolyBrep->GetPolyEdges( sPolyEdges ) ;
  lCount = sPolyEdges.GetSize() ;
  for(ii=0;ii<lCount;ii++)
    {
      SmQuadricEdgeAttr *pAttr = (SmQuadricEdgeAttr*)sPolyEdges[ii]->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;
      while ( pAttr != NULL )
        {
          // remove the Edge decimate attribute
          sPolyEdges[ii]->RemoveAttribute( pAttr ) ;
          pAttr = (SmQuadricEdgeAttr*)sPolyEdges[ii]->FindAttribute(SM_AI_QUADRIC_EDGE_VALUES) ;
        }
    }

  // all done
  return SM_SUCCESS ;

} // end SmPolyDecimate::DoQuadricDecimation

/*******************************************************************//**
PURPOSE: Subdivide a triangle bag until it is ready to be decimated.

NOTES: 
***********************************************************************/
SmStatus SmPolyDecimate::DivideAndDecimate
 (SmTriangleBag      * pTriangleBag,
  const SmExtent3d   & crBBox,
  SmPolygonSLAOutput & rSLP,
  ULONG                lTargetSize)
{
  // If we are small enought decimage it and write results out.
  if ( lTargetSize > pTriangleBag->GetNumberTriangles() )
  {
      SmObjDelete sClean2( pTriangleBag );
      SmPolyBrep *pPolyBrep;
      SER( pTriangleBag->CreatePolyBrep( *pTriangleBag->GetContext(), pPolyBrep )); NER(pPolyBrep);
      SmObjDelete sClean(pPolyBrep);
      {
          SmPolyDecimate sDec( pPolyBrep, m_dPercentOfReduction, m_dMaximumError, FALSE,
              m_dMinFeatureAngle, m_dInteriorEdgeWeight, m_dBoundaryEdgeWeight );
          sDec.DoQuadricDecimation();  // increments unlocked mark value
      }
      SM_PTR_ARRAY(sPolyFaces,SmPolyFace,256);
      SM_PTR_ARRAY(sEdges,SmPolyEdge,16);
      pPolyBrep->GetPolyFaces(sPolyFaces);
      SmPoint3d aPnts[3];

      ULONG ii, jj, lNumFaces = sPolyFaces.GetSize();
      for(ii=0;ii<lNumFaces;ii++)
      {
          SmPolyFace *pF = sPolyFaces[ii];
          pF->GetPolyEdges(sEdges);
          if (sEdges.GetSize() != 3) {
              SE(SM_ERR);
              continue;
          }
          for(jj=0;jj<3;jj++){
              aPnts[jj] = sEdges[jj]->GetStartPoint();
          }
          SER( rSLP.OutputPolygon( 0, 3, aPnts, NULL, NULL, NULL, NULL ));
      }
      return SM_SUCCESS;
  }

  SmExtent3d sBox1, sBox2;
  ULONG lDirection;
  crBBox.SubdivideLargestDirection( sBox1, sBox2, lDirection );
  SmTriangleBag *pBag1, *pBag2;
  SER( pTriangleBag->SplitBag( *pTriangleBag->GetContext(), sBox1, pBag1, pBag2 ));
  SM_ASSERT(pTriangleBag != NULL) ; delete pTriangleBag ; pTriangleBag = NULL ;
  SER( DivideAndDecimate( pBag1, pBag1->GetBBox(), rSLP, lTargetSize ));
  SER( DivideAndDecimate( pBag2, pBag2->GetBBox(), rSLP, lTargetSize ));
  
  return SM_SUCCESS;

} // end SmPolyDecimate::DivideAndDecimate

/*******************************************************************//**
PURPOSE: This is the decimation function that works with very large
  data sets.

NOTES: How small to subdivide before decimating.  10k is a good number to start with.
***********************************************************************/
SmStatus SmPolyDecimate::DoLargeDecimation
(
  const TCHAR * cInputSTLFileName, 
   SmFileType eInputFileType,
   const TCHAR * cOutputSTLFileName,
   SmFileType eOutputFileType,
   ULONG lTargetSize                            
)
{
  // This guy does the set up work to subdivide a polygonal database 
  // until it can be decimated quickly.
  SmContext sContext;
  SmTriangleBag *pTriangleBag;
  SER(SmTriangleBag::ReadFromSTLFile(sContext,cInputSTLFileName,pTriangleBag,eInputFileType));
//    SmObjDelete sClean1(pTriangleBag);

  SmBoolean bFailure;
  SmPolygonSLAOutput sSLP((TCHAR*)cOutputSTLFileName,eOutputFileType,0,bFailure);
  if (bFailure) SER(SM_ERR);

  this->m_dBoundaryEdgeWeight = 0.0; // Don't decimate things on the edges.

  SER( DivideAndDecimate( pTriangleBag, pTriangleBag->GetBBox(), sSLP, lTargetSize ));

  SER( sSLP.CompletedPolygonOutput() );
  return SM_SUCCESS;

} // end SmPolyDecimate::DoLargeDecimation


/*******************************************************************//**
PURPOSE: Determine if a face is degenerate 

NOTES: 
***********************************************************************/
SmBoolean SmPolyFace::IsDegenerate(SmBoolean bRecomputeNormal)
{
  SM_PTR_ARRAY(sEdges,SmPolyEdge,32);
  GetPolyEdges(sEdges);
  ULONG lTotalEdges = sEdges.GetSize();
  if (lTotalEdges < 3) {
      m_vNormal.Set(1,1,1);
      return TRUE;
  }

  SmBoolean bIsDegenerate;
  GetNormal( bRecomputeNormal, FALSE, &bIsDegenerate );

  return bIsDegenerate;

} // end SmPolyFace::IsDegenerate


/*******************************************************************//**
PURPOSE: Simple decimator to eliminate small edges given minimum
  edge length. In addition, users may also specify optional minimum
  aspect ratio(default value= 1:15) as the criteria to collapse skinny polygons

NOTES:  
***********************************************************************/
SmStatus SmPolyDecimate::DoMinimumEdgeLengthDecimation(double dMinEdgeLength,
                                                     double dOptMinAspectRatio)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      m_pPolyBrep->Dump();
      // if (0) {
      //     smgfx_Erase();
      // }
      m_pPolyBrep->Draw();
      sm_GraphicsLoop();
      m_pPolyBrep->ValidatePointers();
  }
#endif // SM_DEBUG_CODE


  SM_PTR_ARRAY(sVerts,SmPolyVertex,512);
  m_pPolyBrep->GetPolyVertices(sVerts);

  // Get a list of head vertices and corresponding attributes
  const SmContext * cpContext = m_pPolyBrep->GetContext(); 
  ULONG i;
  for(i=0;i<sVerts.GetSize();i++){
      SmPolyVertex * pVert = sVerts[i];
      SmDecimateVertexAttr *pAttr = (SmDecimateVertexAttr*)(pVert->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES));
      if (pAttr == NULL) {
          pAttr = new (*cpContext) SmDecimateVertexAttr(SM_AI_VERTEX_DECIMATION_VALUES, 
              SM_AB_STANDALONE_COPY);
          pVert->AddAttribute(pAttr);    
      }
      // Use m_lSortIndex as weight index when removing an edge
      pAttr->m_lSortIndex = 1;
  }

  // We are going to process decimation based on different
  // levels of edge length criteria:
  // First step, use 1/100 of dMinEdgeLength as the minimum length
  // Secondly, use 1/10th of dMinEdgeLength. Thirdly, eliminate all
  // edges that has length less than dMinEdgeLength
  ULONG lSteps = 3;
  double dCurrMinLength = dMinEdgeLength/1000.0;
  ULONG lCount;
  for(lCount=0;lCount<lSteps;lCount++)
  {
      dCurrMinLength *= 10.0;
      SM_PTR_ARRAY(sSmallEdges,SmPolyEdge,512);
      SM_PTR_ARRAY(sFaces,SmPolyFace,512);
      m_pPolyBrep->GetPolyFaces(sFaces);
      ULONG jj, lNumFaces = sFaces.GetSize();
      for(jj=0;jj<lNumFaces;jj++)
      {
          SmPolyFace *pPFace = sFaces[jj];
          SM_PTR_ARRAY(sFaceEdges,SmPolyEdge,4);
          pPFace->GetPolyEdges(sFaceEdges);
          double dMin = SM_BIG_DOUBLE;
          double dMax = -SM_BIG_DOUBLE;
          SmBoolean bFoundSmallEdge = FALSE;
          SmPolyEdge * pSmallestE = NULL;
          ULONG iii, lNumEdges = sFaceEdges.GetSize();
          for(iii=0;iii<lNumEdges;iii++)
          {
              SmPolyEdge *pPEdge = sFaceEdges[iii];
              double dLen = pPEdge->Length();
              if (dLen < dCurrMinLength) {
                  sSmallEdges.Add(pPEdge);
                  bFoundSmallEdge = TRUE;
                  break;
              }
              if (dLen < dMin) {
                  dMin = dLen;
                  pSmallestE = pPEdge;
              }
              if (dLen > dMax) {
                  dMax = dLen;
              }
          }
          if (bFoundSmallEdge) continue;

          // Determine if this edge is too small
          if (dMin < dMax/10.0) {
              sSmallEdges.Add(pSmallestE);
          }
      } // end for each face

      SM_PTR_ARRAY(sDegenerateFaces,SmPolyFace,64);
      ULONG ii, lNumSmall = sSmallEdges.GetSize();
      for(ii=0;ii<lNumSmall;ii++)
      {
          SmPolyEdge *pPEdge = sSmallEdges[ii];
          if (pPEdge == NULL) { continue; }
          if (pPEdge->Length() > dCurrMinLength) {
              continue;
          }
          SmPolyVertex *pVert = pPEdge->GetStartPolyVertex();
          SmPolyVertex *pOtherV = pPEdge->GetEndPolyVertex();
          SmPoint3d sPnt = pVert->GetPoint();
          SmPoint3d sOtherPnt = pOtherV->GetPoint();

#ifdef SM_DEBUG_CODE
          ULONG lDebugCount = 0;
          ULONG lDebugII = 0;
SmBoolean bDebugMe1 = FALSE;
          if (bDebugMe1) {
              if (lCount==lDebugCount && ii==lDebugII) {
                  smgfx_Erase();
                  SM_PTR_ARRAY(sLEdges,SmPolyEdge,8);
                  pVert->GetStartingPolyEdges(sLEdges);
                  ULONG aaa, lNumLEdges = sLEdges.GetSize();
                  for(aaa=0;aaa<lNumLEdges;aaa++){
                      SmPolyFace * pLF =sLEdges[aaa]->GetPolyFace();
                      pLF->Draw();
                      sm_GraphicsLoop();
                  }
                  pOtherV->GetStartingPolyEdges(sLEdges);
                  lNumLEdges = sLEdges.GetSize();
                  for(aaa=0;aaa<lNumLEdges;aaa++){
                      SmPolyFace * pLF =sLEdges[aaa]->GetPolyFace();
                      pLF->Draw();
                      sm_GraphicsLoop();
                  }
              }
          }
#endif // SM_DEBUG_CODE

          SmDecimateVertexAttr *pAttr = (SmDecimateVertexAttr*)(pVert->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES));
          if (pAttr == NULL) {
              // pVert may have been created by Stitch when edge was split
              pAttr = new (*cpContext) SmDecimateVertexAttr(SM_AI_VERTEX_DECIMATION_VALUES, 
                  SM_AB_STANDALONE_COPY);
              pVert->AddAttribute(pAttr);    
              pAttr->m_lSortIndex = 1;
          }
          SmDecimateVertexAttr *pOtherAttr = (SmDecimateVertexAttr*)(pOtherV->FindAttribute(SM_AI_VERTEX_DECIMATION_VALUES));
          if (pOtherAttr == NULL) {
              // pOtherV may have been created by Stitch when edge was splitted
              pOtherAttr = new (*cpContext) SmDecimateVertexAttr(SM_AI_VERTEX_DECIMATION_VALUES, 
                  SM_AB_STANDALONE_COPY);
              pOtherV->AddAttribute(pOtherAttr);    
              pOtherAttr->m_lSortIndex = 1;
          }
          ULONG lWeight = pAttr->m_lSortIndex;
          ULONG lOtherWeight = pOtherAttr->m_lSortIndex;
          double dTotalWeight = (double)lWeight+lOtherWeight;
          double dT = (double)lWeight / dTotalWeight;
          SmPoint3d sPntAverage = sPnt*dT + sOtherPnt*(1.0-dT);
          pVert->SetPoint(sPntAverage);

          pAttr->m_lSortIndex += pOtherAttr->m_lSortIndex;

          SM_PTR_ARRAY(sDelEdges,SmPolyEdge,8);
          SM_PTR_ARRAY(sDelFaces,SmPolyFace,4);
          // We are going to sqeeze out this edge and keep pV
          // But first, find all edges that will be removed in the process
          SM_PTR_ARRAY(sRadEdges,SmPolyEdge,4);
          pPEdge->GetAllRadials(sRadEdges);
          ULONG jjj, lNumRadE = sRadEdges.GetSize();
          for(jjj=0;jjj<lNumRadE;jjj++)
          {
              SmPolyEdge *pE = sRadEdges[jjj];
              SmPolyFace * pPFace = pE->GetPolyFace();
              SM_PTR_ARRAY(sFaceEdges,SmPolyEdge,4);
              pPFace->GetPolyEdges(sFaceEdges);
              if (sFaceEdges.GetSize() <= 3) {
                  sDelEdges.Append(sFaceEdges);
                  sDelFaces.AddUnique(pPFace);
              }
              else {
                  sDelEdges.Add(pE);
              }
          }
          ULONG kkk, lNumDelE = sDelEdges.GetSize();
          for(kkk=0;kkk<lNumDelE;kkk++)
          {
              SmPolyEdge *pE = sDelEdges[kkk];
              ULONG lIndex;
              if (sSmallEdges.FindElement(pE,lIndex)) {
                  sSmallEdges[lIndex] = NULL;
              }
          }

          // Now, sqeeze out this edge and keep pV
          SER(m_pPolyBrep->SqueezeEdge(pPEdge,pVert));

          ULONG lTotalDelFaces = sDelFaces.GetSize();
          if (lTotalDelFaces == 0) {
              continue;
          }

          SM_PTR_ARRAY(sStartEdges,SmPolyEdge,64);
          pVert->GetStartingPolyEdges(sStartEdges);

          ULONG nnn, lNumDelF = sDelFaces.GetSize();
          for(nnn=0;nnn<lNumDelF;nnn++)
          {
              SmPolyFace * pPFace = sDelFaces[nnn];
              m_pPolyBrep->DeletePolyFace(pPFace);
              ULONG lIndex;
              if (sDegenerateFaces.FindElement(pPFace,lIndex)) {
                  sDegenerateFaces[lIndex] = NULL;
              }
          }

          if (lTotalDelFaces == sStartEdges.GetSize()) {
              // In this case, pVert should have already been deleted
              // since it has no faces connected to it.
              continue;
          }

          sDelFaces.ReSet();
          pVert->GetStartingPolyEdges(sStartEdges);
          ULONG hhh, lNumStartE = sStartEdges.GetSize();
          for(hhh=0;hhh<lNumStartE;hhh++)
          {
              SmPolyEdge *pE = sStartEdges[hhh];
              if (pE->GetStartPolyVertex() == pE->GetEndPolyVertex()) {
                  SmPolyFace * pPFace = pE->GetPolyFace();
                  sDelFaces.AddUnique(pPFace);
                  SM_PTR_ARRAY(sFaceEdges,SmPolyEdge,4);
                  pPFace->GetPolyEdges(sFaceEdges);
                  ULONG kkkk, lNumFaceE = sFaceEdges.GetSize();
                  for(kkkk=0;kkkk<lNumFaceE;kkkk++){
                      ULONG lIndex;
                      if (sSmallEdges.FindElement(sFaceEdges[kkkk],lIndex)) {
                          sSmallEdges[lIndex] = NULL;
                      }
                  }
              }
          }

          lNumDelF = sDelFaces.GetSize();
          for(hhh=0;hhh<lNumDelF;hhh++)
          {
              SmPolyFace * pPFace = sDelFaces[hhh];
              m_pPolyBrep->DeletePolyFace(pPFace);
              ULONG lIndex;
              if (sDegenerateFaces.FindElement(pPFace,lIndex)) {
                  sDegenerateFaces[lIndex] = NULL;
              }
          }

          // Glue lamina edges together
          pVert->GetStartingPolyEdges( sStartEdges );
          lNumStartE = sStartEdges.GetSize();
          for(hhh=0;hhh<lNumStartE;hhh++)
          {
              SmPolyEdge *pE = sStartEdges[hhh];
              if (!pE->IsLamina()) continue;
              SmPolyVertex *pVEnd = pE->GetEndPolyVertex();
              SM_PTR_ARRAY(sEndEdges,SmPolyEdge,64);
              pVEnd->GetStartingPolyEdges(sEndEdges);
              ULONG iiii, lNumEndE = sEndEdges.GetSize();
              for(iiii=0;iiii<lNumEndE;iiii++)
              {
                  SmPolyEdge *pTestE = sEndEdges[iiii];
                  if (pTestE->GetEndPolyVertex() == pVert) {
                      if (!pTestE->IsLamina()) continue; 
                      SmOrientType eOrient;
                      if (!pE->IsCoincidentWith(dMinEdgeLength,pTestE,eOrient)) 
                        { SER(SM_ERR); }
                      else 
                        {
                          SM_PTR_ARRAY(sEdgesBetween,SmPolyEdge,16);
                          // Glue (make radial partners) pE and pTestE without deleting any PolyEdge,
                          //   glue endPVerts deleting pOtherEdge->PolyVerts          
                          SER(m_pPolyBrep->GlueEdges(pE,             // in : Target PolyEdge1
                                                     pTestE,         // in : Target PolyEdge2
                                                     eOrient,        // in : oneof SM_OT_SAME, SM_OT_OPPOSITE
                                                     sEdgesBetween));// out: PolyEdges between glued vertices (made zero length by gluing) 
                                                                     // out: PolyVerts deleted when PolyVerts were glued (stale pointers), NULL to ignore, default:[NULL]
                                                                     // out: associated surviving PolyVerts when PolyVerts were glued, NULL to ignore, default:[NULL]
                        }

                  }
              }
          }

          lNumStartE = sStartEdges.GetSize();
          for(hhh=0;hhh<lNumStartE;hhh++)
          {
              SmPolyFace * pPFace = sStartEdges[hhh]->GetPolyFace();
              if (pPFace == NULL) continue;
              SM_PTR_ARRAY(sFaceEdges,SmPolyEdge,4);
              pPFace->GetPolyEdges(sFaceEdges);
              ULONG lNumFaceEdges = sFaceEdges.GetSize();
              SM_PTR_ARRAY(sTestFaces,SmPolyFace,4);
              sTestFaces.Add(pPFace);
              if (lNumFaceEdges < 3) {
                  sDegenerateFaces.AddUnique(pPFace);
              }
              else if (lNumFaceEdges == 3) {
              SmBoolean bIsDegenerate;
              pPFace->GetNormal(TRUE,TRUE,&bIsDegenerate);
              if (bIsDegenerate) {
                  sDegenerateFaces.AddUnique(pPFace);
              }
          }
              else if (lNumFaceEdges > 3) {
                  SER(pPFace->TriangulateNonPlanar(sTestFaces));
              }
          }
      }

      ULONG kk, lNumDegen = sDegenerateFaces.GetSize();
      for(kk=0;kk<lNumDegen;kk++)
      {
          SmPolyFace * pPFace = sDegenerateFaces[kk];
          if (pPFace) {
              m_pPolyBrep->DeletePolyFace(pPFace);
          }
      }

      // Do some stitching
      ULONG lNumEdgesStitched = 0;
      ULONG lNumberLaminaRemaining = 0;
      double dTol = m_pPolyBrep->GetTolerance();
      //SER(m_pPolyBrep->Stitch(dCurrMinLength,TRUE,TRUE,FALSE,  ))
      SER(m_pPolyBrep->Stitch(dTol,                  // note: increments unlocked mark value
                           TRUE,TRUE,FALSE,
                           lNumEdgesStitched,
                           lNumberLaminaRemaining));

      SER(m_pPolyBrep->CleanSolidMesh());
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
      if (bDebugMe2) {
          m_pPolyBrep->Dump();
          smgfx_Erase();
          m_pPolyBrep->Draw();
          sm_GraphicsLoop();
          m_pPolyBrep->ValidatePointers();
      }
#endif // SM_DEBUG_CODE

      // Remove geometrically coincident faces 
      SER(m_pPolyBrep->RemoveCoincidentFaces());

  } // end for lCount < lSteps


  // At last, delete those skinny faces whose height is
  // relatively small (Note: each side might not be that small)
  SM_PTR_ARRAY(sDeleteFaces,SmPolyFace,512);
  SM_PTR_ARRAY(sFaces,SmPolyFace,512);
  m_pPolyBrep->GetPolyFaces(sFaces);

  // Note, must use GetSize() because sFaces can change.
  for(ULONG ii=0;ii<sFaces.GetSize();ii++)
  {
      SmBoolean bFaceSqueezed = FALSE;

      SmPolyFace *pPFace = sFaces[ii];

      if (pPFace == NULL) { continue; }

      SM_PTR_ARRAY(sFaceEdges,SmPolyEdge,4);
      pPFace->GetPolyEdges(sFaceEdges);

      lCount = sFaceEdges.GetSize();
      if (lCount < 3) {
          sDeleteFaces.AddUnique(pPFace);
          continue;
      }
      if (lCount > 3) {
          // We need to triangulate it
          SM_PTR_ARRAY(sNewPolyFaces, SmPolyFace, 64);         // SmTArray<SmPolyFace*>
          SER(pPFace->TriangulateNonPlanar(sNewPolyFaces));
          sFaces.Append(sNewPolyFaces);
          pPFace->GetPolyEdges(sFaceEdges);
      }

      // Double loop (jj, kk) over sFaceEdges.
      ULONG jj, kk, lNumFaceEdges = sFaceEdges.GetSize();
      for(jj=0;jj<lNumFaceEdges;jj++)
      {
          SmPolyEdge *pPEdge = sFaceEdges[jj];
          SmPolyVertex *pV = pPEdge->GetStartPolyVertex();
          SmPoint3d sVertPnt = pV->GetPoint();
          for(kk=0;kk<lNumFaceEdges;kk++)
          {
              if (kk == jj || (kk+1)%lCount == jj)
                { continue; }
              SmPolyEdge *pTargetE = sFaceEdges[kk];
              SmPoint3d sLinePnt;
              SmPoint3d sLineVec;
              SER(pTargetE->GetLine(sLinePnt,sLineVec));
              double dLineVecLengthSquared = sLineVec.LengthSquared();
              if (dLineVecLengthSquared < SM_EFF_ZERO_SQ) {
                  sDeleteFaces.AddUnique(pPFace);
                  break;
              }
              double dParam, dDistSq;
              SmVector3d sVecToPt = sVertPnt - sLinePnt;
              dParam = sVecToPt.Dot(sLineVec) / dLineVecLengthSquared;
              SmPoint3d sFoundPnt = sLinePnt + dParam * sLineVec;
              dDistSq = sFoundPnt.DistanceBetweenSquared(sVertPnt);
              // Try to collapse the face, here we collapse those triangles whose
              // aspect ratio (height/base) were less the the given minimum.
              if (   dParam >= 0.1
                  && dParam <= 0.9
                  && smos_Sqrt( dDistSq/dLineVecLengthSquared ) < dOptMinAspectRatio
                 )
              {
                  SmPoint3d sSplitPoint = sLinePnt + dParam*sLineVec;
                  pV->SetPoint(sSplitPoint);
                  
                  // Do edge splitting
                  if (!pTargetE->IsManifold()) {
                      SER(SM_ERR); // Nonmanifold edge found
                  }
                  SmPolyEdge * pRadialE = pTargetE->GetRadial();
                  SmPolyFace *pF = pRadialE->GetPolyFace(); NER(pF);
                  SmPolyEdge *pNewEdge = NULL;
                  SmPolyVertex *pNewVertex = NULL;
                  SER(pF->MakeVertexSplitPolyEdge(pRadialE,     // in : PolyEdge to split
                                                  sSplitPoint,  // in : Point split location
                                                  pNewEdge,     // out: new PolyEdge (and new radial partners)
                                                  pNewVertex)); // out: new PolyVertex
                  if (pNewEdge == NULL) 
                      break;


                  // Glue pRadialE & pNewEdge with
                  // pPEdge & pPEdge->GetCWPolyEdge();
                  SmPolyEdge * pGroup1E[] = { NULL, NULL };
                  SmPolyEdge * pGroup2E[] = { NULL, NULL };
                  pGroup1E[0] = pRadialE;
                  pGroup1E[1] = pNewEdge;
                  pGroup2E[0] = pPEdge->GetRadial();
                  pGroup2E[1] = pPEdge->GetCWPolyEdge()->GetRadial();

                  m_pPolyBrep->DeletePolyFace(pPFace);
                  SM_PTR_ARRAY(sEdgesBetween,SmPolyEdge,4);
                  SER(m_pPolyBrep->GlueVertices(pNewVertex,pV,sEdgesBetween));

                  ULONG eee, ggg;
                  for(eee=0;eee<2;eee++)
                    {
                      SmPolyVertex * pStartV = pGroup1E[eee]->GetStartPolyVertex();
                      SmPolyVertex * pEndV = pGroup1E[eee]->GetEndPolyVertex();
                      for(ggg=0;ggg<2;ggg++)
                        {
                          if (   pStartV == pGroup2E[ggg]->GetEndPolyVertex()
                              && pEndV   == pGroup2E[ggg]->GetStartPolyVertex())
                            {
                              // Glue (make radial partners) pGroup1E[eee] and pGroup2E[ggg] without deleting any PolyEdge,
                              //   glue endPVerts deleting pOtherEdge->PolyVerts          
                              SER(m_pPolyBrep->GlueEdges(pGroup1E[eee],   // in : Target PolyEdge1
                                                         pGroup2E[ggg],   // in : Target PolyEdge2
                                                         SM_OT_OPPOSITE,  // in : oneof SM_OT_SAME, SM_OT_OPPOSITE
                                                         sEdgesBetween)); // out: PolyEdges between glued vertices (made zero length by gluing) 
                                                                          // out: PolyVerts deleted when PolyVerts were glued (stale pointers), NULL to ignore, default:[NULL]
                                                                          // out: associated surviving PolyVerts when PolyVerts were glued, NULL to ignore, default:[NULL]
                              bFaceSqueezed = TRUE;
                              break;
                            }
                        }
                  }
                    
                  // It is possible, after splitting, the edges become too small
                  // then we want to eliminate small edge (and triangle) by squeezing it away
                  SmBoolean bSplitTriangle = TRUE;
                  for(eee=0;eee<2;eee++)
                  {
                      SmPolyEdge * pE = pGroup2E[eee];
                      if (pE->Length() < dMinEdgeLength) {
                          SmPolyFace * pPolyFace = pE->GetPolyFace();
                          SmPolyVertex *pSurvivingVertex = pE->GetStartPolyVertex();
                          if (pSurvivingVertex == pV) {
                              pSurvivingVertex = pE->GetEndPolyVertex();
                          }
                          SER(m_pPolyBrep->SqueezeEdge(pE,pSurvivingVertex));
                          SM_PTR_ARRAY(sFaceEdges2,SmPolyEdge,4);
                          pPolyFace->GetPolyEdges(sFaceEdges2);
                          if (sFaceEdges2.GetSize() == 2) {
                              // Deletye degenerate triangle
                              SmPolyEdge * pE1 = sFaceEdges2[0]->GetRadial(); NER(pE1);
                              SmPolyEdge * pE2 = sFaceEdges2[1]->GetRadial(); NER(pE2);

                              m_pPolyBrep->DeletePolyFace( pPolyFace );
                              // Glue (make radial partners) pE1 and pE2 without deleting any PolyEdge,
                              //   glue endPVerts deleting pOtherEdge->PolyVerts          
                              SER(m_pPolyBrep->GlueEdges(pE1,             // in : Target PolyEdge1
                                                         pE2,             // in : Target PolyEdge2
                                                         SM_OT_OPPOSITE,  // in : oneof SM_OT_SAME, SM_OT_OPPOSITE
                                                         sEdgesBetween)); // out: PolyEdges between glued vertices (made zero length by gluing) 
                                                                          // out: PolyVerts deleted when PolyVerts were glued (stale pointers), NULL to ignore, default:[NULL]
                                                                          // out: associated surviving PolyVerts when PolyVerts were glued, NULL to ignore, default:[NULL]
                              ULONG lIndex;
                              if (sFaces.FindElement( pPolyFace,lIndex)) {
                                  sFaces[lIndex] = NULL;
                              }
                          }
                          bSplitTriangle = FALSE;
                          break;
                      }
                  }

                  if (!bSplitTriangle) {
                      break;
                  }

                  // To keep all triangle, let's split pF
                  SmPolyEdge * pNewManifoldEdge = NULL;
                  SmPolyLoop * pNewLoop = NULL;
                  SmPolyFace * pNewFace = NULL;
                  
                  SmPolyEdge * pStartVertexEdge = pNewEdge;
                  SmPolyEdge * pEndVertexEdge = pRadialE->GetCWPolyEdge();
                  SER( pF->MakeManifoldEdge( pStartVertexEdge, pEndVertexEdge,
                      pNewManifoldEdge, pNewLoop, pNewFace ));

                  if ( pNewFace )
                  {
                      pNewFace->GetNormal( TRUE, TRUE );
                      sFaces.Add( pNewFace );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
                      if (bDebugMe3) {
                          smgfx_Erase();
                          smgfx_SetLook(1,2, 1,0,0); pNewEdge->Draw(); sm_GraphicsLoop();
                          smgfx_SetLook(1,2, 0,0,1); pF->Draw(); sm_GraphicsLoop();
                          pNewFace->Draw(); sm_GraphicsLoop();
                          sm_GraphicsLoop();
                      }
#endif // SM_DEBUG_CODE

                  }
                  break;

              } // end if aspect ratio less the the given minimum.

          } // end inner double loop (kk) over sFaceEdges

          if ( bFaceSqueezed )
            { break; }

      } // end outer double loop (jj) over sFaceEdges
  } // end big loop (ii) over sFaces
  
  if ( sDeleteFaces.GetSize() > 0 )
  {
      ULONG nn, lNumDelFaces = sDeleteFaces.GetSize();
      for(nn=0;nn<lNumDelFaces;nn++){
          m_pPolyBrep->DeletePolyFace( sDeleteFaces[nn] );
      }

      ULONG lNumEdgesStitched = 0;
      ULONG lNumberLaminaRemaining = 0;
      double dTol = 1.0e4*m_pPolyBrep->GetTolerance();
      SER ( m_pPolyBrep->Stitch( dTol,                     // note: increments unlocked mark value
                              TRUE, TRUE, FALSE,
                              lNumEdgesStitched, 
                              lNumberLaminaRemaining ));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
      if (bDebugMe4) {
          m_pPolyBrep->Dump();
          smgfx_Erase();
          m_pPolyBrep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();

          m_pPolyBrep->ValidatePointers();
      }
#endif // SM_DEBUG_CODE

      SER(m_pPolyBrep->CleanSolidMesh());
  }

  // Do the last thing which is to remove geometrically coincident faces 
  SER( m_pPolyBrep->RemoveCoincidentFaces() );

  // Recompute all face normals
  m_pPolyBrep->GetPolyFaces(sFaces);
  ULONG lNumFaces = sFaces.GetSize();
  for(ULONG ii=0;ii<lNumFaces;ii++)
  {
      SmPolyFace *pPFace = sFaces[ii];
      pPFace->GetNormal( TRUE, TRUE );
  }

  return SM_SUCCESS;

} // end SmPolyDecimate::DoMinimumEdgeLengthDecimation
