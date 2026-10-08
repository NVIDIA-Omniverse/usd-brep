// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPointSequence.cpp
* PURPOSE: Implementation of SmPointSequence methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmPointSet.h>
#include <SmTArray.h>
#include <SmVector3d.h>
#include <SmExtent3d.h>
#include <SmGeomUtility.h>
#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>
#include <SmPseudoBox.h>

/*******************************************************************//***
PURPOSE: Empty Constructorr

NOTES:
************************************************************************/
SmPointSet3d::SmPointSet3d
 (ULONG        nMemorySize,    // in : initial m_pData array size or size of given pOptPtrArray
  SmVector3d * pOptPtrArray,   // in : optional preallocated array to use for storage
  ULONG        nArraySize)     // in : initial array size, less than or equal to nMemorySize.
: SmTArray<SmPoint3d>(nMemorySize, pOptPtrArray, nArraySize),
  m_dDistTol3d(SM_EFF_ZERO),
  m_ePointSetType(SM_PST_UNKNOWN)
{
  ClearCache() ; // set

} // end SmPointSet3d::SmPointSet3d Empty Constructor

/*******************************************************************//***
PURPOSE: From SmTArray<SmPoint3d> array Constructor

NOTES:
************************************************************************/
SmPointSet3d::SmPointSet3d
 (SmTArray<SmPoint3d> & rTArray,    // in : array copied or converted to SmPointSet3d
  double                dTol3d,     // in : min dist between distinct points, default:[SM_EFF_ZERO]
  SmBoolean             bPlanar,    // NotUsed: in : TRUE = PointSequence is known to lie on a plane
  SmVector3d          * pOptNormal, // in : NULL   =system computes m_sNormal from best fit plane to points,
                                    //      NotNULL=User Given PlaneNormal when PointSet is known to be planar, default:[NULL]
  SmBoolean             bCopyData)  // in : TRUE=Copy sPoints into a new array - sPoints and this m_pData arrays are different
                                    //      FALSE=Share sPoints with new array - sPoints and this m_pData arrays are same
                                    //      default:[TRUE]
: SmTArray<SmPoint3d>(rTArray.GetDataSize(),   // eff: copy the base clasee
                      rTArray.GetDataArray(),
                      rTArray.GetSize()),
  m_dDistTol3d(dTol3d)
{
  SM_REF1(bPlanar) ;
  ClearCache() ;

  // Save user given normal - it never gets checked against the point set
  if(pOptNormal)
    {
      if(SM_SUCCESS != pOptNormal->Unitize())
        { SM_DBG_WARN( _T("SmPointSet3d constructor: ignoring given zero Length Normal, will compute one as needed")) ;
        }
      else
        { m_sNormal           = *pOptNormal ;
          m_bNormalSet        = TRUE ;
          m_ePointSetType     = SM_PST_PLANAR ;
          m_bPointSetTypeSet  = TRUE ;

        }
    } // end given Optional Normal check

  // copy data
  if(bCopyData == FALSE)
    { this->SetIsBorrowed(rTArray.GetIsBorrowed()) ;
      rTArray.SetIsBorrowed(TRUE) ;
    }
  else
    {
      this->SetIsBorrowed(TRUE) ;
      this->SetSize(0) ;
      this->Append(rTArray) ;
    }

} // end SmPointSet3d::SmPointSet3d From SmTArray<SmPoint3d> array Constructor

/*******************************************************************//***
PURPOSE: Copy Constructor

NOTES:
************************************************************************/
SmPointSet3d::SmPointSet3d
 (const SmPointSet3d &crOther)    // in : array copied or converted to SmPointSet3d
 : SmTArray<SmPoint3d>(crOther)   // eff: copy the base clasee
{
  // copy members
  m_dDistTol3d         = crOther.m_dDistTol3d;

  m_bPointSetTypeSet   = crOther.m_bPointSetTypeSet;
  m_ePointSetType      = crOther.m_ePointSetType;

  m_bCentroidSet       = crOther.m_bCentroidSet;
  m_sCentroid          = crOther.m_sCentroid;

  m_bAxesSet           = crOther.m_bAxesSet;
  m_sXAxis             = crOther.m_sXAxis;
  m_sYAxis             = crOther.m_sYAxis;

  m_bNormalSet         = crOther.m_bNormalSet;
  m_sNormal            = crOther.m_sNormal;

  m_bMaxCentroidDevSet = crOther.m_bMaxCentroidDevSet ;
  m_dMaxCentroidDev    = crOther.m_dMaxCentroidDev ;

  m_bMaxDevSet         = crOther.m_bMaxDevSet ;
  m_dMaxDev            = crOther.m_dMaxDev ;

  m_bBBoxSet           = crOther.m_bBBoxSet;
  m_bPBoxRefined       = crOther.m_bPBoxRefined ;
  m_sBBox              = crOther.m_sBBox;
  m_sPBox              = crOther.m_sPBox;

} // end SmPointSet3d::SmPointSet3d copy constructor

/*******************************************************************//***
PURPOSE: Assignment operator.

NOTES:
************************************************************************/
SmPointSet3d & SmPointSet3d::operator=( const SmPointSet3d &crOther )
{
  // no work condition
  if(&crOther == this) return *this ;

  // copy base values
  this->SmTArray<SmPoint3d>::operator=(crOther) ;

  // copy members
  m_dDistTol3d         = crOther.m_dDistTol3d;

  m_bPointSetTypeSet   = crOther.m_bPointSetTypeSet;
  m_ePointSetType      = crOther.m_ePointSetType;

  m_bCentroidSet       = crOther.m_bCentroidSet;
  m_sCentroid          = crOther.m_sCentroid;

  m_bAxesSet           = crOther.m_bAxesSet;
  m_sXAxis             = crOther.m_sXAxis;
  m_sYAxis             = crOther.m_sYAxis;

  m_bNormalSet         = crOther.m_bNormalSet;
  m_sNormal            = crOther.m_sNormal;

  m_bMaxCentroidDevSet = crOther.m_bMaxCentroidDevSet ;
  m_dMaxCentroidDev    = crOther.m_dMaxCentroidDev ;

  m_bMaxDevSet         = crOther.m_bMaxDevSet ;
  m_dMaxDev            = crOther.m_dMaxDev ;

  m_bBBoxSet           = crOther.m_bBBoxSet;
  m_bPBoxRefined       = crOther.m_bPBoxRefined ;
  m_sBBox              = crOther.m_sBBox;
  m_sPBox              = crOther.m_sPBox;

  // all done
  return *this;

} // end SmPointSet3d::operator=

/*******************************************************************//***
PURPOSE: Deep Equality operator

NOTES: compares the values of the stored points
   rather than their memory locations.
   (here so that we can have an SmTArray of them).
************************************************************************/
SmBoolean SmPointSet3d::operator==( const SmPointSet3d &crOther )
{
  ULONG ii ;

  // no work condition
  if(&crOther == this) return TRUE ;

  if( !(GetSize()            == crOther.GetSize()           )) { return FALSE; }
  for(ii=0;ii<GetSize();ii++)
    if( !(m_pData[ii]        == crOther.m_pData[ii]         )) { return FALSE ; }

  if( !(m_dDistTol3d         == (SmTol3d)crOther.m_dDistTol3d)) { return FALSE; }

  if( !(m_bPointSetTypeSet   == crOther.m_bPointSetTypeSet  )) { return FALSE; }
  if( !(m_ePointSetType      == crOther.m_ePointSetType     )) { return FALSE; }

  if( !(m_bCentroidSet       == crOther.m_bCentroidSet      )) { return FALSE; }
  if( !(m_sCentroid          == crOther.m_sCentroid         )) { return FALSE; }

  if( !(m_bAxesSet           == crOther.m_bAxesSet          )) { return FALSE; }
  if( !(m_sXAxis             == crOther.m_sXAxis            )) { return FALSE; }
  if( !(m_sYAxis             == crOther.m_sYAxis            )) { return FALSE; }

  if( !(m_bNormalSet         == crOther.m_bNormalSet        )) { return FALSE; }
  if( !(m_sNormal            == crOther.m_sNormal           )) { return FALSE; }

  if( !(m_bMaxCentroidDevSet == crOther.m_bMaxCentroidDevSet)) { return FALSE; }
  if( !(m_dMaxCentroidDev    == crOther.m_dMaxCentroidDev   )) { return FALSE; }

  if( !(m_bMaxDevSet         == crOther.m_bMaxDevSet        )) { return FALSE; }
  if( !(m_dMaxDev            == crOther.m_dMaxDev           )) { return FALSE; }

  if( !(m_bBBoxSet           == crOther.m_bBBoxSet          )) { return FALSE; }
  if( !(m_bPBoxRefined       == crOther.m_bPBoxRefined      )) { return FALSE; }
  if( !( m_sBBox             == crOther.m_sBBox             )) { return FALSE; }
//  if( !( m_sPBox          == crOther.m_sPBox          )) { return FALSE; }

  return TRUE;

} // end SmPointSet3d::operator==

/*******************************************************************//***
PURPOSE: Private method to set up a coordinate system for this point set.

NOTES:
   Sets, if not already set:
     m_bPointSized, m_bLinear, m_bPlanar
     m_sCentroid (which will be the origin)
     m_sXAxis
     m_sYAxis
     m_sNormal
   If not planar, then m_sXAxis and m_sNormal are not necessarily meaningful.
************************************************************************/
SmStatus SmPointSet3d::SetUpData
 (SmPointSetDataType eReason,  // oneof: SM_PSD_AXES                   = cache Normal, XAxis, and YAxis properties
                               //        SM_PSD_CENTROID               = cache centroid property
                               //        SM_PSD_POINTSET_TYPE          = cache SmPointSetType m_ePointSetType property
                               //        SM_PSD_MAX_DEVIATION          = cache Max Deviation from best fit plane, line, or point
                               //        SM_PSD_MAX_CENTROID_DEVIATION = cache Max Deviation from centroid
                               //        SM_PSD_BBOXES                 = cache Aligned and Pseudo bounding boxes
                               //        SM_PSD_REFINE_PBOX            = rotate Pseudo box X and Y axes to minize PseudoBox size
                               //        SM_PSD_ALL                    = cache all PointSet properties
  SmVector3d * pOptInputPBoxZ) // in : used to set the basis[2] direction of the pseudoBox.
                               //      NULL to ignore, default:[NULL]
{
  // no work - requested data already computed
  switch(eReason)
    { case SM_PSD_ALL                    : if(   m_bAxesSet
                                              && m_bMaxDevSet
                                              && m_bCentroidSet
                                              && m_bPointSetTypeSet
                                              && m_bMaxCentroidDevSet) { return SM_SUCCESS; }
      case SM_PSD_AXES                   : if(m_bAxesSet)              { return SM_SUCCESS; }
      case SM_PSD_MAX_DEVIATION          : if(m_bMaxDevSet)            { return SM_SUCCESS; }
      case SM_PSD_CENTROID               : if(m_bCentroidSet)          { return SM_SUCCESS; }
      case SM_PSD_POINTSET_TYPE          : if( m_bPointSetTypeSet)     { return SM_SUCCESS; }
      case SM_PSD_MAX_CENTROID_DEVIATION : if(m_bMaxCentroidDevSet)    { return SM_SUCCESS; }
      default:
          break;
    }

  // return value
  SmStatus sRtn = SM_SUCCESS ;

  // pass out the requests - these are order dependent
  if(eReason & SM_PSD_AXES)                                            { sRtn = SetUpAxes() ; }
  if(eReason & SM_PSD_CENTROID)                                        { sRtn = SetUpCentroid() ; }
  if(eReason & SM_PSD_POINTSET_TYPE)                                   { sRtn = SetUpPointSetType() ; }
  if(eReason & (SM_PSD_MAX_DEVIATION | SM_PSD_MAX_CENTROID_DEVIATION)) { sRtn = SetUpDeviation(eReason) ; }
  if(eReason & (SM_PSD_BBOXES        | SM_PSD_REFINE_PBOX) )           { sRtn = SetUpBBoxes(eReason, pOptInputPBoxZ) ; }

  // all done
  return(sRtn) ;

} // end SmPointSet3d::SetUpData

/*******************************************************************//***
PURPOSE: Private method to set up a coordinate system for this point set.

NOTES:  sets m_sNormal, m_sXAxis, m_sYAxis,
        marks m_bNormal and m_bAxes = TRUE

        When user specifies m_sNormal (m_bNormalSet == TRUE) use it
        else get normal from best fit plane to points which gets
        other data including: m_ePointSetType   and m_bPointSetTypeSet
                              m_sCentroid       and m_bCentroidSet
                              m_dMaxDev         and m_bMaxDevSet
                              m_dMaxCentroidDev and m_bMaxCentroidDevSet

        If not planar, then m_sXAxis and m_sNormal are not necessarily meaningful.
************************************************************************/
SmStatus SmPointSet3d::SetUpAxes()
{
  // no work - requested data already computed
  if(m_bAxesSet)
    { return SM_SUCCESS; }

  // branch on user given Normals
  if(m_bNormalSet && SM_IS_ZERO(1.0 - m_sNormal.LengthSquared()) )
    {
      // mark state
      m_bAxesSet = TRUE ;

      // when the points have been classified linear - set XAxis = tangent projected to Normal plane
      if(   m_bPointSetTypeSet == TRUE
         && m_ePointSetType    == SM_PST_LINEAR)
         {
           // find the tangent line direction
           SmVector3d sO = m_pData[0], sX ;
           double     dDist2, dDistTol3d2 = m_dDistTol3d * m_dDistTol3d ;

           // for the rest of the points
           for(ULONG ii=1;ii<m_lSize;ii++)
             {
               // seek first point to specify a line
               dDist2 = sO.DistanceBetweenSquared(m_pData[ii]) ;
               if(dDist2 > dDistTol3d2)
                 {
                   sX  = m_pData[ii] - sO ;
                   break ;
                 }
             } // end iter ii, every point seeking a line tangent direction

           // build the axes: Z=m_sNormal, X=Tangent Projected to Normal Plane, Y = Z * X ;
           m_sNormal.MakeUnitOrthoVectors(&sX, m_sNormal, m_sXAxis, m_sYAxis) ;
        }
      else // use given normal as Z and compute any X and Y axes.
        {
           m_sNormal.MakeUnitOrthoVectors(NULL, m_sNormal, m_sXAxis, m_sYAxis) ;
        }
      return(SM_SUCCESS) ;
    }
  else // compute Point Normal from best fit plane to points
    {
      // classify the point set, one of SM_PST_VOID, _POINTSIZED, _LINEAR, _PLANAR, _SCATTERED
      // set m_ePointSetType, m_sCentroid, m_dDeviation
      smgu_BestFitPlaneToPoints(GetPoints(),         // in : target point set
                                m_dDistTol3d,        // in : Min distance between distinct 3d points
                                m_ePointSetType,     // out: one of: SM_PST_VOID,          // empty point set
                                                     //              SM_PST_POINTSIZED,    // all pts within tol of BestCenterPt
                                                     //              SM_PST_LINEAR,        // all pts within tol of BestLine
                                                     //              SM_PST_PLANAR,        // all pts within tol of BestPlane
                                                     //              SM_PST_SCATTERED      // pts not within tol of target shape
                                m_sCentroid,         // out: found point on plane - geometric center of all crPoints
                                m_sNormal,           // out: for SM_PST_PLANAR     = found best plane normal
                                                     //          SM_PST_SCATTERED  = found best plane normal
                                                     //          SM_PST_LINEAR     = found best line tangent
                                                     //          SM_PST_POINTSIZED = not used set:[0 0 0]
                                                     //          SM_PST_VOID       = not used set:[0 0 0]
                                m_dMaxDev,           // out: max gap SM_PST_POINTSIZED: between crPoints[i] and rCenterPt
                                                     //              SM_PST_LINEAR    : between crPoints[i] and BestLine
                                                     //              SM_PST_PLANAR    : between crPoints[i] and BestPlane
                                                     //              SM_PST_SCATTERED : between crPoints[i] and BestPlane
                                m_dMaxCentroidDev) ; // out: max gap between crPoints[i] and rCenterPt

        // set axes
        switch(m_ePointSetType)
          {
            case SM_PST_VOID       :
            case SM_PST_POINTSIZED : m_sXAxis. Set(1,0,0) ;
                                     m_sYAxis. Set(0,1,0) ;
                                     m_sNormal.Set(0,0,1) ;
                                     break ;
            case SM_PST_LINEAR     : // currently m_sNormal = BestLine Tangent
                                     m_sNormal.MakeUnitOrthoVectors(NULL,m_sXAxis,m_sYAxis,m_sNormal) ;
                                     break ;
            case SM_PST_PLANAR     : // currently m_sNormal = BestPlane Normal
                                     m_sNormal.MakeUnitOrthoVectors(NULL,m_sNormal,m_sXAxis,m_sYAxis) ;
                                     break ;
            case SM_PST_SCATTERED  : // currently m_sNormal = BestPlane Normal
                                     m_sNormal.MakeUnitOrthoVectors(NULL,m_sNormal,m_sXAxis,m_sYAxis) ;
                                     break ;
            default:
                break;
          } // end swith on m_ePointSetType value

        // mark all saved values
        m_bAxesSet           = TRUE ;
        m_bNormalSet         = TRUE ;
        m_bPointSetTypeSet   = TRUE ;
        m_bCentroidSet       = TRUE ;
        m_bMaxDevSet         = TRUE ;
        m_bMaxCentroidDevSet = TRUE ;

    } // end eSetType switch - getting deviation

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; smgfx_DrawPlane(m_sCentroid, m_sNormal) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; m_sCentroid.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; m_sXAxis.Draw (&m_sCentroid) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; m_sYAxis.Draw (&m_sCentroid) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; m_sNormal.Draw(&m_sCentroid) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return (m_ePointSetType) ;

} // end SmPointSet3d::SetUpAxes

/*******************************************************************//***
PURPOSE: Private method to set centroid

NOTES: Sets  m_sCentroid
       Marks m_bCetroidSet
************************************************************************/
SmStatus SmPointSet3d::SetUpCentroid()
{
  // no work - requested data already computed
  if(m_bCentroidSet == TRUE)
    { return SM_SUCCESS ; }

  // locals
  ULONG ii, jj, idx, jCnt ;
  ULONG lBlockSize = 50 ;
  ULONG lExtras = m_lSize % lBlockSize ;
  ULONG iters   = m_lSize / lBlockSize + (lExtras == 0 ? 0 : 1) ;

  // init centroid
  m_sCentroid.Set(0,0,0) ;

  // to prevent adding very small numbers to very large one when m_lSize is large
  // get the accumulated average in blocks

  // for every block - first the extras, then blocks of lBlockSize
  for(ii=0,idx=0,jCnt=lExtras;ii<iters;ii++,jCnt=lBlockSize)
    {
      SmPoint3d sBlockSum(0,0,0) ;

      // for every point in this block
      for(jj=0;jj<jCnt;jj++,idx++)
        {
          sBlockSum += m_pData[idx] ;
        } // end iter jj, every point in this block

      // accumulate weighted block sums [B671]
      //m_sCentroid += ((double)jCnt)/((double)m_lSize) * sBlockSum ;
      m_sCentroid += sBlockSum / (double) m_lSize;

    } // end iter ii, every block

  // all done
  return(SM_SUCCESS) ;

} // end SmPointSet3d::SetUpCentroid

/*******************************************************************//***
PURPOSE: Private method to set m_ePointSetType

NOTES: Sets  m_ePointSetType
       Marks m_bPointSetTypeSet
************************************************************************/
SmStatus SmPointSet3d::SetUpPointSetType()
{
  // no work - requested data already computed
  if(m_bPointSetTypeSet == TRUE)
    { return SM_SUCCESS ; }

  // init output and mark state
   m_ePointSetType = SM_PST_VOID ;
   m_bPointSetTypeSet = TRUE ;

  // no work - no points
  if(m_lSize == 0)
    { return(SM_SUCCESS) ; }

  // the intent of user given normals is to specify the orientation of the
  // pseudo boxes and the best fit plane.  So, should the PointSetType
  // be done in the user given coordinate system or based just on the point
  // locations.  Consider a planar set of points not in the plane
  // defined by the user given Normal.  Is that planar or scattered?

  // For now I assume users will be careful to give normals that are aligned
  // with the data and so I'll just compute the properties directly from
  // the point locations.  I could use a walk the data approach or
  // call smgu_BestFitPlaneToPoints.  In either case I can't save the
  // side effect data because the classification coordinate system
  // (taken from the points) and the cached coordinate system (given by
  // the caller) may be different.

  // I choose the walk the data approach because without saving
  // the side-effect data, it's less work.

  // locals
  ULONG ii ;
  SmVector3d sO, sX, sY, sZ ;
  double     dDist, dDist2 ;
  double     dDistTol3d2 = m_dDistTol3d * m_dDistTol3d ;

  // init search
  sO = m_pData[0] ;
  m_ePointSetType = SM_PST_POINTSIZED ;

  // init output and mark state

  // for the rest of the points
  for(ii=1;ii<m_lSize;ii++)
    {
      switch(m_ePointSetType)
        {
          case SM_PST_POINTSIZED : // seek first point to specify a line
                                   { dDist2 = sO.DistanceBetweenSquared(m_pData[ii]) ;
                                     if(dDist2 > dDistTol3d2)
                                       {
                                         // data is not point sized
                                         sX  = m_pData[ii] - sO ;
                                         sX /= smos_Sqrt(dDist2) ;
                                         m_ePointSetType = SM_PST_LINEAR ;
                                       }
                                     break ;
                                   }
          case SM_PST_LINEAR     : // seek first point to specify a plane
                                   { smgu_LinePointDistance(sO, sX, m_pData[ii], dDist) ;
                                     if(dDist > m_dDistTol3d)
                                       {
                                         // data is not linear
                                         sY = m_pData[ii] - sO ;
                                         sX.MakeUnitOrthoVectors(&sY,sX,sY,sZ) ;
                                         m_ePointSetType = SM_PST_PLANAR ;
                                       }
                                     break ;
                                   }
          case SM_PST_PLANAR     : // seek first scattered point not on the plane
                                   { smgu_PlanePointDistance(sO, sZ, m_pData[ii], dDist) ;
                                     if(dDist > m_dDistTol3d)
                                       { m_ePointSetType = SM_PST_SCATTERED ;
                                         return(SM_SUCCESS) ;
                                       }
                                     break ;
                                   }
          default:
              break;
        } // end switch on m_ePointSetType
    } // end iter ii, all Points

  // arrive here Point set is classified

  // if Normal is set make sure the m_sXAxis is the line tangent projected to m_sNormal plane
  // note: we can't do more axes setting here because other than the linear case
  //       the axes built here are not fitted to the data.
  if(m_bAxesSet && m_ePointSetType == SM_PST_LINEAR)
    {
      m_sNormal.MakeUnitOrthoVectors(&sX, m_sNormal, m_sXAxis, m_sYAxis) ;
      m_bAxesSet = TRUE ;
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmPointSet3d::SetUpPointSetType

/*******************************************************************//***
PURPOSE: Private method to set bounding boxes, both axes aligned and pseudo

NOTES: Sets  m_sBBox and m_sPBox
       Marks m_bBBoxSet and possibly m_bPBoxRefined
************************************************************************/
SmStatus SmPointSet3d::SetUpBBoxes
 (SmPointSetDataType eReason,  // oneof: SM_PSD_BBOXES      = cache Aligned and Pseudo bounding boxes
                               //        SM_PSD_REFINE_PBOX = rotate Pseudo box X and Y axes to minize PseudoBox size
                               //        SM_PSD_ALL         = Cache and Refine BBoxes
  SmVector3d * pOptInputPBoxZ) // in : used to set the basis[2] direction of the pseudoBox.
                               //      NULL to ignore, default:[NULL]
{
  // prep
  if(pOptInputPBoxZ)
    { pOptInputPBoxZ->Unitize() ; }

  // no work - requested data already computed
  if(   (!(eReason & SM_PSD_BBOXES)      || m_bBBoxSet     == TRUE)
     && (!(eReason & SM_PSD_REFINE_PBOX) || m_bPBoxRefined == TRUE)
     && (  pOptInputPBoxZ == NULL || pOptInputPBoxZ->CloserThan(SM_EFF_ZERO, m_sPBox.GetBasis(2))))
    { return SM_SUCCESS ; }

  // init output
  SmStatus sRtn = SM_SUCCESS ;

  // when BBoxes need to be built
  if(   ((m_bBBoxSet == FALSE) && (eReason & (SM_PSD_BBOXES | SM_PSD_REFINE_PBOX)))
     || ( pOptInputPBoxZ != NULL && !pOptInputPBoxZ->CloserThan(SM_EFF_ZERO, m_sPBox.GetBasis(2))))
    {
      // mark state
      m_bBBoxSet = TRUE ;

      // Pseudo box uses the stored axes
      SetUpAxes() ;
      if(pOptInputPBoxZ == NULL)
        { m_sPBox.SetBasis(m_sXAxis, m_sYAxis, m_sNormal) ; }
      else
        { SmVector3d sX, sY ;
          pOptInputPBoxZ->MakeUnitOrthoVectors(NULL, *pOptInputPBoxZ, sX, sY) ;
          m_sPBox.SetBasis(sX, sY, *pOptInputPBoxZ) ;
        }

      // init box extents
      m_sBBox.Init() ;
      m_sPBox.Init() ;

      // for every point
      for(ULONG ii=0;ii<m_lSize;ii++)
        {
          m_sBBox.AddPoint3d(m_pData[ii]) ;
          m_sPBox.AddPoint3d(m_pData[ii]) ;
        }
    } // end need to build BBoxes check

  // when asked and if needed - refine PBox
  if((m_bPBoxRefined == FALSE) && (eReason & SM_PSD_REFINE_PBOX))
    {
      // rotate the X and Y axes around the normal vector to better fit the PseudoBox to the points
      sRtn = SetUpTightPseudoBox(m_bPointSetTypeSet && m_ePointSetType == SM_PST_PLANAR, pOptInputPBoxZ) ;

    } // end need to refine PBox check

  // all done
  return(sRtn) ;

} // end SmPointSet3d::SetUpBBoxes

/*******************************************************************//***
PURPOSE: Private method to tighten m_sPBox, i.e. Random search to find a
  good rotation of the planar X and Y axes to minimize the area of the
  PseudoBox xy plane.

NOTES: Modifies m_sPBox
       Marks    m_bPBoxRefined
************************************************************************/
SmStatus SmPointSet3d::SetUpTightPseudoBox
  (SmBoolean bDoExtraPlanarWork,  // in : TRUE = align Pseudo box so that two consecutive points are
                                  //             on the PseudoBox boundary.  Good for aligning PointSequence
                                  //             polygons with a PseudoBox boundary.
                                  //      FALSE= don't bother doing the extra work - the data is not planar
   SmVector3d * pOptInputPBoxZ)   // in : used to set the basis[2] direction of the pseudoBox.
                                  //      NULL to ignore, default:[NULL]

{
  // no work - already computed
  if(m_bPBoxRefined == TRUE)
    { return SM_SUCCESS ; }

  // set state
  m_bPBoxRefined = TRUE ;

  // ensure m_sPBox is built
  SetUpBBoxes(SM_PSD_BBOXES, pOptInputPBoxZ) ;

  // locals
  SmPoint3d   sPlanePoint  = m_sPBox.Evaluate(.5, .5, .5) ;
  SmVector3d  sPlaneX      = m_sPBox.GetBasis(0) ;
  SmVector3d  sPlaneY      = m_sPBox.GetBasis(1) ;
  SmVector3d  sPlaneZ      = m_sPBox.GetBasis(2) ;
  SmVector3d  sPlaneNormal = sPlaneX * sPlaneY ;
  sPlaneNormal.Unitize() ;
  double      dArea        = m_sPBox.GetXYArea() ;
  double      dThisArea ;
  SmPseudoBox sThisBox ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw input pseudo box
  if(bDebugMe)
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,4, 0,0,1) ; this->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; m_sPBox.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // for a few rotations
  ULONG ii, jj, iRotCnt = 5 ;

  for(ii=0;ii<iRotCnt;ii++)
    {
      double dAngRad = (ii+1) * (SM_PI/2.0/(iRotCnt+1)) ;

      // build pseudo box with rotated basis0 and basis1 vectors
      SmVector3d sRotX = sPlaneX.RotateVecAboutAxis(sPlaneNormal, dAngRad) ;
      SmVector3d sRotY = sPlaneY.RotateVecAboutAxis(sPlaneNormal, dAngRad) ;

      sThisBox.SetBasis(sRotX, sRotY, sPlaneZ) ;
      sThisBox.Init() ; // init intervals - leave basis vectors alone

      // size this pseudoBox to the data
      for(jj=0;jj<m_lSize;jj++) { sThisBox.AddPoint3d(m_pData[jj]) ; }

      // get XY plane area
      dThisArea = sThisBox.GetXYArea() ;

      // save the smallest area box
      if(dThisArea < dArea)
        {
          dArea = dThisArea ;
          m_sPBox = sThisBox ;
        }
    } // end iteration trial and test of rotated boxes

#ifdef SM_DEBUG_CODE
  // add potentially modified pseudobox to the graphics
  if(bDebugMe)
    {
      smgfx_SetLook(1,4,  0,0,1) ; this->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4,  1,0,0) ; m_sPBox.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // when asked - do extra work to try and align the box with a convex polygon line segment
  if(bDoExtraPlanarWork)
    {
      SmExtentPointType eBnd0, eBnd1, eBnd2 ;
      ULONG lFoundBeg0 = 0, lFoundEnd0 = 0 ;
      ULONG lFoundBeg1 = 0, lFoundEnd1 = 0 ;
      ULONG lBegIndex0 = 0, lEndIndex0 = 0;
      ULONG lBegIndex1 = 0, lEndIndex1 = 0;

      // find points on the min and max Basis[0] boundaries
      for(ii=0;ii<m_lSize;ii++)
        {
          // since we just made the pseudo box - expect some points to lie exactly on min/max boundaries
          m_sPBox.ClassifyPoint3d(m_pData[ii], eBnd0, eBnd1, eBnd2, SM_EFF_ZERO) ;

          // save min/max points - these are two points on the convex hull of the point set
          if(eBnd0 == SM_EP_START) { lBegIndex0 = ii ; lFoundBeg0++ ; }
          if(eBnd0 == SM_EP_END)   { lEndIndex0 = ii ; lFoundEnd0++ ; }
          if(eBnd1 == SM_EP_START) { lBegIndex1 = ii ; lFoundBeg1++ ; }
          if(eBnd1 == SM_EP_END)   { lEndIndex1 = ii ; lFoundEnd1++ ; }
        } // end iter search for min/max basis0 points

#ifdef SM_DEBUG_CODE
      // draw found min/max points
      if(bDebugMe)
        {
          sPlanePoint  = m_sPBox.Evaluate(.5, .5, .5) ;
          sPlaneX = m_sPBox.GetBasis(0) ;
          sPlaneY = m_sPBox.GetBasis(1) ;
          sPlaneZ = m_sPBox.GetBasis(2) ;

          smgfx_SetLook(1,4,  0,0,1) ; this->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4,  1,0,0) ; m_sPBox.Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5,  1,0,0) ; sPlaneX.Draw(&sPlanePoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5,  0,1,0) ; sPlaneY.Draw(&sPlanePoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5,  0,0,1) ; sPlaneZ.Draw(&sPlanePoint) ; sm_GraphicsLoop() ;

          smgfx_SetLook(7,8,  1,0,0) ; GetAt(lBegIndex0).Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(13,14,1,0,0) ; GetAt(lEndIndex0).Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(7,8,  0,1,0) ; GetAt(lBegIndex1).Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(13,14,0,1,0) ; GetAt(lEndIndex1).Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // no more work - some convex polygon line is already on a boundary
      if(   lFoundBeg0 > 1
         || lFoundEnd0 > 1
         || lFoundBeg1 > 1
         || lFoundEnd1 > 1)
        { return(SM_SUCCESS) ; }

      // update the pseudobox coordinate vectors
      sPlaneX = m_sPBox.GetBasis(0) ;
      sPlaneY = m_sPBox.GetBasis(1) ;
      sPlaneZ = m_sPBox.GetBasis(2) ;

      // note: to avoid tolerance problems on planar data and to do
      //       something reasonable on nonPlanar data for this next step,
      //       Project points to a common plane before testing angles.
      //       Project along basis[2].
      //       Measure angles from basis[1] about basis[0].cross(basis[1]) vector.
      SmPoint3d sBegPt0 = m_pData[lBegIndex0] ;
      SmPoint3d sEndPt0 = m_pData[lEndIndex0] ;
      SmPoint3d sBegPt1 = m_pData[lBegIndex1] ;
      SmPoint3d sEndPt1 = m_pData[lEndIndex1] ;
      sBegPt0.ProjectPointToPlane(sPlanePoint, sPlaneZ) ;
      sEndPt0.ProjectPointToPlane(sPlanePoint, sPlaneZ) ;
      sBegPt1.ProjectPointToPlane(sPlanePoint, sPlaneZ) ;
      sEndPt1.ProjectPointToPlane(sPlanePoint, sPlaneZ) ;
      double dBegAngRad0, dBegMinAngRad0 = SM_BIG_DOUBLE, dBegMaxAngRad0 = -SM_BIG_DOUBLE ;
      double dEndAngRad0, dEndMinAngRad0 = SM_BIG_DOUBLE, dEndMaxAngRad0 = -SM_BIG_DOUBLE ;
      double dBegAngRad1, dBegMinAngRad1 = SM_BIG_DOUBLE, dBegMaxAngRad1 = -SM_BIG_DOUBLE ;
      double dEndAngRad1, dEndMinAngRad1 = SM_BIG_DOUBLE, dEndMaxAngRad1 = -SM_BIG_DOUBLE ;

      // find the neighbor points with the smallest angles meaured from the boundary plane from the min/max points
      for(ii=0;ii<m_lSize;ii++)
        {
          SmPoint3d sThisPt = m_pData[ii] ;
          sThisPt.ProjectPointToPlane(sPlanePoint, sPlaneZ) ;

          // skip the bound points
          if(   lBegIndex0 == ii
             || lEndIndex0 == ii
             || lBegIndex1 == ii
             || lEndIndex1 == ii)
            { continue ; }

          // get the vectors
          SmVector3d sBegVec0 = sThisPt - sBegPt0 ;
          SmVector3d sEndVec0 = sThisPt - sEndPt0 ;
          SmVector3d sBegVec1 = sThisPt - sBegPt1 ;
          SmVector3d sEndVec1 = sThisPt - sEndPt1 ;

          // get the angles
          sPlaneNormal.CCWAngleBetween(sPlaneY, sBegVec0, dBegAngRad0) ;  // (negative)
          sPlaneNormal.CCWAngleBetween(sPlaneY, sEndVec0, dEndAngRad0) ;  // (positive)
          sPlaneNormal.CCWAngleBetween(sPlaneX, sBegVec1, dBegAngRad1) ;  // (positive)
          sPlaneNormal.CCWAngleBetween(sPlaneX, sEndVec1, dEndAngRad1) ;  // (negative)

#ifdef SM_DEBUG_CODE
          // draw found min/max points
          if(bDebugMe)
            {
              sPlanePoint  = m_sPBox.Evaluate(.5, .5, .5) ;
              smgfx_SetLook(1,4,  0, 0,1) ; this->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4,  1,.5,0) ; m_sPBox.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5,  0,0,1) ;  sPlaneX.Draw(&sPlanePoint) ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5,  0,1,0) ;  sPlaneY.Draw(&sPlanePoint) ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5,  1,0,0) ;  sPlaneZ.Draw(&sPlanePoint) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2,  0,1,1) ;  sBegVec0.Draw(&sBegPt0) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2,  0,1,1) ;  sEndVec0.Draw(&sEndPt0) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2,  0,1,1) ;  sBegVec1.Draw(&sBegPt1) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2,  0,1,1) ;  sEndVec1.Draw(&sEndPt1) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // save the min and max angles
          // note: expect all angles for BegPt0 to be from -Pi to 0  (negative)
          //       expect all angles for EndPt0 to be from   0 to Pi (positive)
          // note: expect all angles for BegPt1 to be from   0 to Pi (positive)
          //       expect all angles for EndPt1 to be from -Pi to 0  (negative)
          if(dBegAngRad0 < dBegMinAngRad0) { dBegMinAngRad0 = dBegAngRad0 ; }  // expected negative
          if(dBegAngRad0 > dBegMaxAngRad0) { dBegMaxAngRad0 = dBegAngRad0 ; }  // expected negative
          if(dEndAngRad0 < dEndMinAngRad0) { dEndMinAngRad0 = dEndAngRad0 ; }  // expected positive
          if(dEndAngRad0 > dEndMaxAngRad0) { dEndMaxAngRad0 = dEndAngRad0 ; }  // expected positive

          if(dBegAngRad1 < dBegMinAngRad1) { dBegMinAngRad1 = dBegAngRad1 ; }  // expected positive
          if(dBegAngRad1 > dBegMaxAngRad1) { dBegMaxAngRad1 = dBegAngRad1 ; }  // expected positive
          if(dEndAngRad1 < dEndMinAngRad1) { dEndMinAngRad1 = dEndAngRad1 ; }  // expected negative
          if(dEndAngRad1 > dEndMaxAngRad1) { dEndMaxAngRad1 = dEndAngRad1 ; }  // expected negative

        } // end iter search for basis0 min/max neighbor points

      // arrive here once min and max neighbors on the convex hull have been found
      // for both the beg and end points.

      // rotate the plane basis vectors in the xy plane the max amount to align the
      // pseudo box basis vectors so that at least 3 points on the convex hull lie on
      // the basis[0] boundaries without moving any pseudobox currently-inside points to the outside.
      double dPosAngRad0 = smos_Min(dEndMinAngRad0, dBegMinAngRad0 + SM_PI) ; // expected positive
      double dNegAngRad0 = smos_Max(dEndMaxAngRad0 - SM_PI, dBegMaxAngRad0) ; // expected negative

      double dPosAngRad1 = smos_Min(dEndMinAngRad1 + SM_PI, dBegMinAngRad1) ; // expected positive
      double dNegAngRad1 = smos_Max(dEndMaxAngRad1, dBegMaxAngRad1 - SM_PI) ; // expected negative

      double dAngRad     =  (smos_Min(dPosAngRad0, dPosAngRad1) > -smos_Max(dNegAngRad0, dNegAngRad1))
                           ? smos_Min(dPosAngRad0, dPosAngRad1)
                           : smos_Max(dNegAngRad0, dNegAngRad1) ;

      // rotate the basis by appropriate angle and rebuild box

      // build pseudo box with rotated basis0 and basis1 vectors
      SmVector3d sRotX = sPlaneX.RotateVecAboutAxis(sPlaneNormal, dAngRad) ;
      SmVector3d sRotY = sPlaneY.RotateVecAboutAxis(sPlaneNormal, dAngRad) ;

      sThisBox.SetBasis(sRotX, sRotY, sPlaneZ) ;
      sThisBox.Init() ; // init intervals - leave basis vectors alone

      // size this pseudoBox to the data
      for(ii=0;ii<m_lSize;ii++) {sThisBox.AddPoint3d(m_pData[ii]) ; }

      // get XY plane area
      dThisArea = sThisBox.GetXYArea() ;

#ifdef SM_DEBUG_CODE
      // draw found min/max points
      if(bDebugMe)
        {
          smgfx_SetLook(1,4,  0, 0,1) ; this->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4,  1,.5,0) ; sThisBox.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // save the smallest area box
      if(dThisArea < dArea)
        {
          dArea   = dThisArea ;
          m_sPBox = sThisBox ;
        }

    } // end DoExtraPlanarWork

#ifdef SM_DEBUG_CODE
      // draw found min/max points
      if(bDebugMe)
        {
          smgfx_SetLook(1,4,  0, 0,1) ; this->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4,  1,.5,0) ; m_sPBox.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end SmPointSet3d::SetUpTightPseudoBox

/*******************************************************************//***
PURPOSE: Private method to set Point deviations from best fit shape
 and the centroid

NOTES: Sets  m_dMaxDev     and m_dMaxCentroidDev
       Marks m_bMaxDevSet and m_bMaxCentroidDevSet
************************************************************************/
SmStatus SmPointSet3d::SetUpDeviation
 (SmPointSetDataType eReason)  // oneof: SM_PSD_MAX_DEVIATION          = cache Max Deviation from best fit plane, line, or point
                               //        SM_PSD_MAX_CENTROID_DEVIATION = cache Max Deviation from centroid
                               //        SM_PSD_ALL                    = Cache both Max Deviations
{
  // no work - requested data already computed
  if(   (!(eReason & SM_PSD_MAX_DEVIATION)          || m_bMaxDevSet         == TRUE)
     && (!(eReason & SM_PSD_MAX_CENTROID_DEVIATION) || m_bMaxCentroidDevSet == TRUE))
    { return SM_SUCCESS ; }

  // init output
  SmStatus sRtn = SM_SUCCESS ;

  // locals
  ULONG ii ;
  double dDist, dDist2 ;

  // ensure independent data is set
  SetUpAxes() ;
  SetUpCentroid() ;
  SetUpPointSetType() ;

  // note work to be done
  SmBoolean bMaxDevCalc   = (eReason & SM_PSD_MAX_DEVIATION) ;
  SmBoolean bCentroidCalc = (eReason & SM_PSD_MAX_CENTROID_DEVIATION) ;
  SmBoolean bEqualCalc    = (   m_ePointSetType == SM_PST_VOID
                             || m_ePointSetType == SM_PST_POINTSIZED) ;
  if(bEqualCalc) { bMaxDevCalc = FALSE ; }

  // init maxes as needed
  if(bMaxDevCalc  ) { m_dMaxDev         = 0.0 ; }
  if(bCentroidCalc) { m_dMaxCentroidDev = 0.0 ; }

  //   // to save run time at the expense of typing - pull all conditionals outside of the calc loop
  //   // note the following code could be a lot cleaner if the conditionals are inside the loops
  //   {
  //     // for every point
  //     for(ii=0;ii<m_lSize;ii++)
  //       { if(bCentroidCalc) { dDist2 = m_sCentroid.DistanceBetweenSquared(m_pData[ii]) ;
  //                             if(dDist2 > m_dMaxDev) {m_dMaxCentroidDev = dDist2 ; }
  //                           }
  //         if(bMaxDevCalc)   { if  (m_ePointSetType == SM_PST_LINEAR)
  //                               { smgu_LinePointDistance(m_sCentroid, m_sXAxis, m_pData[ii], dDist) ;
  //                                 if(dDist > m_dMaxDev) {m_dMaxDev = dDist ; }
  //                               }
  //                             else
  //                               { smgu_PlanePointDistance(m_sCentroid, m_sNormal, m_pData[ii], dDist) ;
  //                                           if(dDist > m_dMaxDev) {m_dMaxDev = dDist ; }
  //                               }
  //                           }
  //       } // end iter ii, every point
  //   }

  // when getting both deviations
  if(bMaxDevCalc && bCentroidCalc)
    { // get centroid and max deviation, switch between Linear and other (Planar and Scattered) case
      if(m_ePointSetType == SM_PST_LINEAR)
        { for(ii=0;ii<m_lSize;ii++)
            { // Max deiviation
              smgu_LinePointDistance(m_sCentroid, m_sXAxis, m_pData[ii], dDist) ;
              if(dDist > m_dMaxDev) {m_dMaxDev = dDist ; }

              // centroid deviation
              dDist2 = m_sCentroid.DistanceBetweenSquared(m_pData[ii]) ;
              if(dDist2 > m_dMaxCentroidDev) {m_dMaxCentroidDev = dDist2 ; }
            }
        } // end linear branch

      else // Plane or Scattered
        { for(ii=0;ii<m_lSize;ii++)
            { // Max deiviation
              smgu_PlanePointDistance(m_sCentroid, m_sNormal, m_pData[ii], dDist) ;
              if(dDist > m_dMaxDev) {m_dMaxDev = dDist ; }

              // centroid deviation
              dDist2 = m_sCentroid.DistanceBetweenSquared(m_pData[ii]) ;
              if(dDist2 > m_dMaxCentroidDev) {m_dMaxCentroidDev = dDist2 ; }
            }
        } // end Plane or Scattered branch
    } // end both devaitons branch

  else if(bMaxDevCalc)
    { // Get just MaxDev, switch between Linear and other (Planar and Scattered) case
      if(m_ePointSetType == SM_PST_LINEAR)
        { for(ii=0;ii<m_lSize;ii++)
            { smgu_LinePointDistance(m_sCentroid, m_sXAxis, m_pData[ii], dDist) ;
              if(dDist > m_dMaxDev) {m_dMaxDev = dDist ; }
            }
        } // end Linear branch

      else // Plane or Scattered
        { for(ii=0;ii<m_lSize;ii++)
            { smgu_PlanePointDistance(m_sCentroid, m_sNormal, m_pData[ii], dDist) ;
              if(dDist > m_dMaxDev) {m_dMaxDev = dDist ; }
            }
        } // end Plane or Scattered branch

    } // end get MaxDev branch

  else // get just centroid deviation
    { // get centroid deviation
      for(ii=0;ii<m_lSize;ii++)
        { dDist2 = m_sCentroid.DistanceBetweenSquared(m_pData[ii]) ;
          if(dDist2 > m_dMaxCentroidDev) {m_dMaxCentroidDev = dDist2 ; }
        }
    } // end GetCentroid branch

  // save cache state
  if(bMaxDevCalc)   { m_bMaxDevSet         = TRUE ; }
  if(bCentroidCalc) { m_dMaxCentroidDev    = smos_Sqrt(m_dMaxCentroidDev) ;
                      m_bMaxCentroidDevSet = TRUE ;
                    }
  if(bEqualCalc)    { m_dMaxDev            = m_dMaxCentroidDev ;
                      m_bMaxCentroidDevSet = TRUE ;
                    }

  // all done
  return(sRtn) ;

} // end SmPointSet3d::SetUpDeviation

/*******************************************************************//**
PURPOSE: Find minimum sized circumscribing sphere for point set

NOTES:
***********************************************************************/
SmStatus SmPointSet3d::CalculateBoundingSphere
  (SmTArray<ULONG> &rIndices,   // out: indices of points found to be on minimum radius sphere
   SmPoint3d       &rCenter,    // out: Center of minimum radius circumscribing sphere
   double          &dRadius)    // out: radius of minimum radius circumscribing sphere
 const
{
  // pass the call along
  return(smgu_BoundingSphereFromPointSet(*this, rIndices, rCenter, dRadius)) ;

} // end SmPointSet3d::CalculateBoundingSphere

/*******************************************************************//**
PURPOSE: Find list of coincident points

NOTES: reorders the Point array to reduce cost of method to
       less than N**2.  So becomes useful after point set size
       reaches more than about 25.

   GWC: This is way too slow - switch to Spatial Tree sort
        as done in the current Stitch function
***********************************************************************/
SmStatus SmPointSet3d::GetCoincidentPoints
 (SmTArray<SmPoint3d> &rCoinPoint1,        // runs in near linear time
  SmTArray<SmPoint3d> &rCoinPoint2)
{
  // init output
  rCoinPoint1.ReSet() ;
  rCoinPoint2.ReSet() ;

  // locals
  ULONG ii, jj ;
  SmPseudoBox sPBox = GetPBox() ;

  // no work - not enough points
  if(m_lSize < 2)
    { return SM_SUCCESS ; }

  // put points into a pseudo box to get best line for sorting
  SmVector3d sAxis = sPBox.GetBasis(0) ;

  // project every point to common line
  ShellSort(smgu_Point3dLessThan, &sAxis) ;

  // check neighbor points for possible coincidence
  // runs in nearly linear time
  SmScaledZero dScaledZero   = SmTol::GetScaledZero(m_pData[0]) ;
  SmScaledZero dScaledZeroSq = dScaledZero * dScaledZero ;
  for(ii=0;ii+1<m_lSize;ii++)  // note: can't say m_lSize-1
    {
      double dProjS1 = m_pData[ii].Dot(sAxis) ;
      for(jj=ii;jj<m_lSize;jj++)
        {
          double dProjS2 = m_pData[jj].Dot(sAxis) ;

          // move onto next point when rest of ordered points can't be coincident
          if(!SM_IS_ZERO_TO_TOL(dProjS2 - dProjS1, dScaledZero))
            { break ; }

          // check for coincidence
          double dDistSq = (m_pData[jj]-m_pData[ii]).LengthSquared() ;
          if(dDistSq < dScaledZeroSq)
            {
              rCoinPoint1.Add(m_pData[ii]) ;
              rCoinPoint2.Add(m_pData[jj]) ;
            }
        } // end iter jj, looking for coincident pair
    } // end iter ii, checking every point

  // all done
  return(SM_SUCCESS) ;

} // end SmPointSet3d::GetCoincidentPoints

//  /*******************************************************************//**
//  PURPOSE: Classify PointSet as oneof void, point, linear, planar, or scattered.
//
//  NOTES: pOptDevitiation = SM_PST_VOID      ? SM_UNDEF_DOUBLE
//                           SM_PST_POINTSIZE ? Max distance to centroid
//                           SM_PST_LINEAR    ? Max distance to Best Fit Line
//                           SM_PST_PLANAR    ? Max distance to Best Fit Plane
//                           SM_PST_SCATTERED ? Max distance to centroid
//  ***********************************************************************/
//  SmPointSetType SmPointSet3d::Classify
//    (double      dTol3d,              // in : min distance between distinct points, default:[SM_EFF_ZERO]
//     SmPoint3d  *pOptCentroid,        // out: geometric average, (undefined for void PointSets),
//                                      //      NULL to ignore, default:[NULL]
//     SmVector3d *pOptVector,          // out: for SM_PST_VOID      ? not set, InitTo:[0 0 0]
//                                      //          SM_PST_POINTSIZE ? not set, InitTo:[0 0 0]
//                                      //          SM_PST_LINEAR    ? Best Fit Line Tangent
//                                      //          SM_PST_PLANAR    ? Best Fit Plane Normal
//                                      //          SM_PST_SCATTERED ? Best Fit Plane Normal
//                                      //      NULL to ignore, default:[NULL]
//     double     *pOptMaxDev,          // out: for SM_PST_VOID      ? not set, InitTo:[0]
//                                      //          SM_PST_POINTSIZE ? Max dist to centroid
//                                      //          SM_PST_LINEAR    ? Max dist to Best Fit Line
//                                      //          SM_PST_PLANAR    ? Max dist to Best Fit Plane
//                                      //          SM_PST_SCATTERED ? Max dist to centroid
//                                      //      NULL to ignore, default:[NULL]
//     double      *pOptMaxCentroidDev, // out: max dist to centroid
//     SmPseudoBox *pOptPseudoBox)      // out: pseudobox built to classify the points, NULL to ignore
//                                      //      NULL to ignore, default:[NULL]
//  {
//    // Set new Tolerance when needed
//    SetTolerance(dTol3d) ;
//
//    // rebuild the cache if needed
//    SetUpAxes() ;
//
//    // set output
//    if(pOptCentroid)       { *pOptCentroid  = m_sCentroid ; }
//    if(pOptVector)         { *pOptVector    = m_ePointSetType == SM_PST_LINEAR ? m_sXAxis : m_sNormal ; }
//    if(pOptMaxDev)         { *pOptMaxDev    = m_dMaxDev ; }
//    if(pOptMaxCentroidDev) { *pOptMaxCentroidDev    = m_dMaxCentroidDev ; }
//    if(pOptPseudoBox)      { *pOptPseudoBox = m_sPBox ; }
//
//    // all done
//    return(m_ePointSetType) ;
//
//  } // end SmPointSet3d::Classify

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmPointSet3d::IsKindOf( SM_TYPE t ) const
{
  return ((SmPointSet3d_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print class object

NOTES:
***********************************************************************/
void SmPointSet3d::Dump()
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  smos_sprintf(sBuff,        _T("\nSmPointSet3d = 0x%p: dDistTol3d[%16.16lf]"),
             this,
             (double)m_dDistTol3d);
  smos_sprintf(sBuffForFile, _T("\nSmPointSet3d = 0x%p: dDistTol3d[%16.16lf]"),
             _T( "NotNULL" ),
             (double)m_dDistTol3d);
  smos_WriteBuffer(sBuff, sBuffForFile);

  // cached data
  if(!m_bPointSetTypeSet)    { smos_sprintf(sBuff,_T("%s"), _T("\n   PointSetType Not Yet Cached:")) ;
                               smos_WriteBuffer(sBuff);
                             }
  else                       { smos_sprintf(sBuff,        _T(", %s"),
                                          m_ePointSetType ==  SM_PST_VOID       ? _T("No Points")
                                         :m_ePointSetType ==  SM_PST_POINTSIZED ? _T("PontSized")
                                         :m_ePointSetType ==  SM_PST_LINEAR     ? _T("Linear")
                                         :m_ePointSetType ==  SM_PST_PLANAR     ? _T("Planar")
                                         :m_ePointSetType ==  SM_PST_SCATTERED  ? _T("Scattered")
                                         :                                        _T("UnKnown PointSetType"));
                               smos_WriteBuffer(sBuff);
                             }

  if(!m_bMaxDevSet)          { smos_sprintf(sBuff, _T("%s"),_T("\n   Max Deviation Not Yet Cached:")) ;
                               smos_WriteBuffer(sBuff);
                             }
  else                       { smos_sprintf(sBuff,        _T(", MaxDev:[%16.16lf]"),
                                          m_dMaxDev);
                               smos_WriteBuffer(sBuff);
                             }

  if(!m_bMaxCentroidDevSet)  { smos_sprintf(sBuff, _T("%s"),_T("\n   Max Deviation Not Yet Cached:")) ;
                               smos_WriteBuffer(sBuff);
                             }
  else                       { smos_sprintf(sBuff,        _T(", MaxCentroidDev:[%16.16lf]"),
                                          m_dMaxCentroidDev);
                               smos_WriteBuffer(sBuff);
                             }

  if(!m_bCentroidSet)        { smos_sprintf(sBuff, _T("%s"),_T("\n   Centroid Not Yet Cached:")) ;
                               smos_WriteBuffer(sBuff);
                             }
  else                       { smos_sprintf(sBuff, _T("\n   Centroid:[%16.16lf, %16.16lf, %16.16lf] "),
                                          m_sCentroid.x, m_sCentroid.y, m_sCentroid.z) ;
                               smos_WriteBuffer(sBuff);
                             }

  if(!m_bNormalSet)          { smos_sprintf(sBuff, _T("%s"),_T("\n   Normal Vector Not Yet Cached:")) ;
                               smos_WriteBuffer(sBuff);
                             }
  else                       { smos_sprintf(sBuff, _T("\n   Normal  :[%16.16lf, %16.16lf, %16.16lf] "),
                                            m_sNormal.x, m_sNormal.y, m_sNormal.z) ;
                               smos_WriteBuffer(sBuff);
                             }

  if(!m_bAxesSet)            { smos_sprintf(sBuff, _T("%s"),_T("\n   Data Centered Axes Not Yet Cached:")) ;
                               smos_WriteBuffer(sBuff);
                             }
  else                       { smos_sprintf(sBuff, _T("\n   XAxis   :[%16.16lf, %16.16lf, %16.16lf] "),
                                            m_sXAxis.x, m_sXAxis.y, m_sXAxis.z) ;
                               smos_WriteBuffer(sBuff);

                               smos_sprintf(sBuff, _T("\n   YAxis   :[%16.16lf, %16.16lf, %16.16lf] "),
                                            m_sYAxis.x, m_sYAxis.y, m_sYAxis.z) ;
                               smos_WriteBuffer(sBuff);
                             }


  if(!m_bBBoxSet)            { smos_sprintf(sBuff, _T("%s"),_T("\n   BBox Not Yet Cached:")) ;
                               smos_WriteBuffer(sBuff);
                             }
  else                       { smos_sprintf(sBuff, _T("\n   %s BBox Min:[%16.16lf, %16.16lf, %16.16lf],\n             BBox Max:[%16.16lf, %16.16lf, %16.16lf]"),
                                          m_bPBoxRefined ? _T("Refined  ") : _T("UnRefined"),
                                          m_sBBox.GetMin().x, m_sBBox.GetMin().y, m_sBBox.GetMin().z,
                                          m_sBBox.GetMax().x, m_sBBox.GetMax().y, m_sBBox.GetMax().z) ;
                               smos_WriteBuffer(sBuff);

                               smos_WriteBuffer( _T("\n   ")) ;
                               m_sPBox.Dump() ;
                             }

  // output the points
  SmTArray<SmPoint3d>::Dump() ;

} // end SmPointSet3d::Dump

/***************************************************************
PURPOSE:  Add point sequence graphics for global display Parameters
             to new DisplayList added to global DisplayList array.

NOTES:
***************************************************************/
SmDisplayList * SmPointSet3d::Draw
  (SmBoolean bAddToUIPickList,  // in : TRUE = Add this PointSequence to UI pick interface for debugging
                                //      default:[FALSE]
   SmBoolean bDrawParams,       // in : TRUE = annotate point sequence poly line
                                //      increasingly large and varying color point icons.
                                //      default:[FALSE]
   SmGfxArraySet *pOptGfxSet)

 const
{
  // init return value
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // when asked - add this SmPointSet3d to UI pick list
  if(bAddToUIPickList)
    { sm_GraphicsAddToBrepList(this) ; }

  // start new DisplayList (unless displayList is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this),NULL, NULL, FALSE, pOptGfxSet);

  // Draw Every Point
  for(ULONG ii=0;ii<m_lSize;ii++)
    {
      smgfx_OutputPoint(m_pData[ii].x, m_pData[ii].y, m_pData[ii].z, pOptGfxSet) ;
    }

  // when point parameterization
  if(bDrawParams)
    {
      ((SmPointSet3d *)this)->SetUpAxes() ;
      double dSize = m_sBBox.GetMaxDimension() * .75 ;

      // save current color and PointSize
      sColor = smgfx_OutputColor(0,0,0,pOptGfxSet) ;
      double dPointSize = smgfx_OutputPointSize(1.0,pOptGfxSet) ;

      // draw the coordinates
      if(m_bCentroidSet && m_bAxesSet)
        {
          smgfx_OutputColor(0.0, 0.0, 0.0, pOptGfxSet) ;
          smgfx_OutputLine(m_sCentroid.x,
                           m_sCentroid.y,
                           m_sCentroid.z,
                           m_sCentroid.x + dSize*m_sXAxis.x,
                           m_sCentroid.y + dSize*m_sXAxis.y,
                           m_sCentroid.z + dSize*m_sXAxis.z, pOptGfxSet) ;

          smgfx_OutputColor(0.0, 0.0, 0.0, pOptGfxSet) ;
          smgfx_OutputLine(m_sCentroid.x,
                           m_sCentroid.y,
                           m_sCentroid.z,
                           m_sCentroid.x + dSize*m_sYAxis.x,
                           m_sCentroid.y + dSize*m_sYAxis.y,
                           m_sCentroid.z + dSize*m_sYAxis.z, pOptGfxSet) ;
        } // end X and Y axes draw

      if(m_bCentroidSet && m_bNormalSet)
        {
          smgfx_OutputColor(1.0, 0.0, 0.0, pOptGfxSet) ;
          smgfx_OutputLine(m_sCentroid.x,
                           m_sCentroid.y,
                           m_sCentroid.z,
                           m_sCentroid.x + dSize*m_sNormal.x,
                           m_sCentroid.y + dSize*m_sNormal.y,
                           m_sCentroid.z + dSize*m_sNormal.z, pOptGfxSet) ;
        } // end Normal draw

      // restore current color and PointSize
      smgfx_OutputColor(sColor, pOptGfxSet) ;
      smgfx_OutputPointSize(dPointSize, pOptGfxSet) ;

    } // end drawing params

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(bAddToUIPickList, bDrawParams, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmPointSet3d::Draw

/*******************************************************************//**
  END SmPointSet3d
***********************************************************************/

/*******************************************************************//**
  BEGIN SmPointSequence
***********************************************************************/
/*******************************************************************//**
PURPOSE: Empty Object Constructor.

NOTES:
   The Closed flag is as specified, always.
***********************************************************************/
SmPointSequence::SmPointSequence
 (ULONG       nDataSize,        // in : initial m_pData array size or size of given pOptPtrArray, default:[0]
  SmPoint3d * pOptPtrArray,     // in : optional preallocated array to use for storage, default:[NULL]
  ULONG       lInitialSize)     // in : initial array size, less than or equal to nDataSize, default:[0]
: SmPointSet3d(nDataSize, pOptPtrArray, lInitialSize)
{
  // set all cache state booleans to FALSE - don't call ClearCache() because SmPointSet3d may have set some cache values
  m_bAreaSet       = FALSE ;
  m_bCoincidentSet = FALSE ;
  m_bMonotonicSet  = FALSE ;

} // end SmPointSequence::SmPointSequence Empty Constructor

/*******************************************************************//**
PURPOSE: Default Constructor.

NOTES:
   The Closed flag is as specified, always.
***********************************************************************/
SmPointSequence::SmPointSequence
 (SmTArray< SmPoint3d > & sPoints,     // in : array placed into PointSequence
  double                  dTol3d,      // in : min dist between distinct points
  SmBoolean               bClosed,     // in : TRUE = implied linear connection between 1st and last point
  SmBoolean               bPlanar,     // in : TRUE = PointSequence is known to lie on a plane
  SmVector3d            * pOptNormal,  // in : NULL=system computes m_sNormal, NotNULL=user given vector as Normal
  SmBoolean               bCopyData)   // in : TRUE=Copy sPoints into a new array - sPoints and returned array are two different lists
                                       //      FALSE=Convert sPoints to SmPointSequence - sPoints and returned array are same list
                                       //      default:[TRUE]
 : SmPointSet3d(sPoints, dTol3d, bPlanar, pOptNormal, bCopyData),
   m_bClosed   ( bClosed )
{
  // set all cache state booleans to FALSE - don't call ClearCache() because SmPointSet3d may have set some cache values
  m_bAreaSet       = FALSE ;
  m_bCoincidentSet = FALSE ;
  m_bMonotonicSet  = FALSE ;

} // end SmPointSequence::SmPointSequence Default Constructor

/*******************************************************************//***
PURPOSE: Copy Constructor

NOTES:
************************************************************************/
SmPointSequence::SmPointSequence
 ( const SmPointSequence &crOther ) // in : object to copy
: SmPointSet3d(crOther)             // eff: copy the base clasee
{
  // copy members
  m_bClosed        = crOther.m_bClosed;
  m_bAreaSet       = crOther.m_bAreaSet;
  m_dArea          = crOther.m_dArea;
  m_bCoincidentSet = crOther.m_bCoincidentSet ;
  m_bCoincident    = crOther.m_bCoincident ;
  m_bMonotonicSet  = crOther.m_bMonotonicSet ;
  m_bMonotonic     = crOther.m_bMonotonic ;

} // end SmPointSequence::SmPointSequence copy constructor

/*******************************************************************//***
PURPOSE: Assignment operator.

NOTES:
************************************************************************/
SmPointSequence & SmPointSequence::operator=( const SmPointSequence &crOther )
{
  // no work condition
  if(&crOther == this) return *this ;

  // copy base values
  this->SmPointSet3d::operator=(crOther) ;

  // copy members
  m_bClosed        = crOther.m_bClosed;
  m_bAreaSet       = crOther.m_bAreaSet;
  m_dArea          = crOther.m_dArea;
  m_bCoincidentSet = crOther.m_bCoincidentSet ;
  m_bCoincident    = crOther.m_bCoincident ;
  m_bMonotonicSet  = crOther.m_bMonotonicSet ;
  m_bMonotonic     = crOther.m_bMonotonic ;

  // all done
  return *this;

} // end SmPointSequence::operator=

/*******************************************************************//***
PURPOSE: Deep Equality operator

NOTES: compares the values of the stored points
   rather than their memory locations.
   (here so that we can have an SmTArray of them).
************************************************************************/
SmBoolean SmPointSequence::operator==( const SmPointSequence &crOther )
{
  // no work condition
  if(&crOther == this) return TRUE ;

  if( !this->SmPointSet3d::operator==(crOther) ) { return FALSE ; }

  if( !( m_bClosed        == crOther.m_bClosed       )     ) { return FALSE ; }
  if( !( m_bAreaSet       == crOther.m_bAreaSet      )     ) { return FALSE ; }
  if( !( m_dArea          == crOther.m_dArea         )     ) { return FALSE ; }
  if( !( m_bCoincidentSet == crOther.m_bCoincidentSet)     ) { return FALSE ; }
  if( !( m_bCoincident    == crOther.m_bCoincident   )     ) { return FALSE ; }
  if( !( m_bMonotonicSet  == crOther.m_bMonotonicSet )     ) { return FALSE ; }
  if( !( m_bMonotonic     == crOther.m_bMonotonic    )     ) { return FALSE ; }

  return TRUE;

} // end SmPointSequence::operator==

/*******************************************************************//***
PURPOSE: Cache the area enclosed by this point set.

NOTES: Might not be meaninful if not planar.
   If total failure (of smgu_PolygonArea), then area is set to -1.0.
   Also, treats point set as closed, whether it officially is or not.
************************************************************************/
SmStatus SmPointSequence::SetUpArea()
{
  // no work - already computed
  if(m_bAreaSet)
    {
      return SM_SUCCESS;
    }

  // Set state.
  m_bAreaSet = TRUE;

  // compute area on plane defined by PointSet Normal
  SmVector3d sNormal = GetNormal();
  SmStatus eStat = smgu_PolygonArea( m_pData,
                                     m_lSize,
                                     sNormal,
                                     m_dArea );
  if ( eStat != SM_SUCCESS )
    { m_dArea = -1.0 ; }

  // all done
  return eStat;

} // end SmPointSequence::SetUpArea

/*******************************************************************//***
PURPOSE: Cache wheter any of the coincident points are coincident

NOTES:
************************************************************************/
SmStatus SmPointSequence::SetUpCoincident()
{
  // no work - already computed
  if(m_bCoincidentSet)
    {
      return SM_SUCCESS;
    }

  // Set state
  m_bCoincidentSet = TRUE;
  m_bCoincident    = FALSE ;

  // locals
  ULONG i0, i1 ;
  if(m_bClosed) { i0 = m_lSize - 1 ;
                  i1 = 0 ;
                }
  else          { i0 = 0 ;
                  i1 = 1 ;
                }
  // For every pair of points
  for(;i1<m_lSize;i1++,i0=i1-1)
    {
      // when Two points are coincident to tolerance
      if( m_pData[i0].CloserThan(m_dDistTol3d, m_pData[i1]))
        {
          // the Point sequence is coincident
          m_bCoincident = TRUE ;
          break ;
        } // end found a coincident point pair
    } // end iter every pair of points

  // all done
  return SM_SUCCESS;

} // end SmPointSequence::SetUpCoincident

/*******************************************************************//***
PURPOSE: Cache whether the rows and column points are in a monotonic position

NOTES: Might not be meaninful if not planar.
   If total failure (of SetUpMonotonic) or when gris is closed,
   then Monotonic values are set to FALSE
************************************************************************/
SmStatus SmPointSequence::SetUpMonotonic()
{
  // no work - already computed
  if(m_bMonotonicSet)
    {
      return SM_SUCCESS;
    }

  // Set state.
  m_bMonotonicSet = TRUE;
  m_bMonotonic    = TRUE;

  // locals
  ULONG i0, i1 ;
  SmVector3d sVec0, sVec1 ;

  if(m_bClosed) { i0 = m_lSize - 1 ;
                  i1 = 0 ;
                }
  else          { i0 = 0 ;
                  i1 = 1 ;
                }

  // get first point pair difference
  sVec0 = m_pData[i1] - m_pData[i0] ;

  // for every pair of points in this row
  for(i1++,i0=i1-1;i1<m_lSize;i1++,i0=i1-1)
    {
      // get this point pair difference
      sVec1 = m_pData[i1] - m_pData[i0] ;

      // check for non-monotonic cases
      // GWC: we have a choice here
      //      1. Check next difference to last difference  - finds only large changes between neighbors
      //      2. Check next difference to first difference - finds changes acros whole row
      // 2 will find changes for grids on disks that are okay
      // so I'm going to initially use scheme 1.

      // look for change in direction
      double dDot = sVec1.Dot(sVec0) ;

      // when clearly not monotonic
      if(dDot < - SM_EFF_ZERO)
        {
          // mark that and move on to col checks
          m_bMonotonic = FALSE ;
          break ;
        }

      // GWC: exactly zero cases happen for coincident points and orthogonal turns
      //      let's assume that we only need to detect grossly non monotonic cases.

      // ready for next iteration - to switch to case 2 comment out next line
      sVec0 = sVec1 ;

    } // end iter every pair of row points

  // all done
  return SM_SUCCESS ;

} // end SmPointSequence::SetUpMonotonic

/*******************************************************************//**
PURPOSE: Return quadrant containing TestPt for coordinates
         crOrigin, m_sXAxis, m_sYAxis

NOTES: 1.) 0 = +x +y        m_sYAxis
           1 = -x +y      1 | 0
           2 = -x -y     ---+---> m_sXAxis
           3 = +x -y      2 | 3

       2.) Private method: local helper for ContainsUVPoint().
***********************************************************************/
int SmPointSequence::CalcQuadNum
 ( const SmPoint3d &crOrigin,
   const SmPoint3d &crTestPt )
{
  SmVector3d sVec( crTestPt - crOrigin );
  int        iRetVal = ( sVec.Dot( m_sXAxis ) > 0.0 ) ? ((sVec.Dot( m_sYAxis ) <= 0.0) ? 3 : 0)
                                                      : ((sVec.Dot( m_sYAxis )  > 0.0) ? 1 : 2) ;
  return iRetVal;

} // end SmPointSequence::CalcQuadNum

/*******************************************************************//***
PURPOSE: Determine whether a given point is inside or outside this point set.

NOTES: Works only for closed point sets.  If not closed, returns SM_POC_OUTSIDE.
  Similar to SmLoop::ContainsUVPoint(), but much simpler because the boundary
  is piecewise linear.  Also, this doesn't assume 2d.
************************************************************************/
SmPointObjectContainmentType SmPointSequence::PointContainment
 ( const SmPoint3d &crPoint )
{
  // First, if not closed, always outside.
  if ( ! m_bClosed )
    { return SM_POC_OUTSIDE; }

  // side effect - SetUpAxes() and set m_sBBox
  GetBBox() ;

  // low work - not in BBox (could also check the PBox
  if ( ! m_sBBox.ContainsPoint3d( crPoint, m_dDistTol3d ) )
    { return SM_POC_OUTSIDE; }

  // Finally, do the algorithm.
  // Winding number, of point set points around the given point.

  // locals
  ULONG ii;
  SmPoint3d sPrevPt, sThisPt = m_pData[ m_lSize-1 ];
  int       iThisQuad = CalcQuadNum( crPoint, sThisPt );
  int       iPrevQuad = iThisQuad;
  int       iAccum    = 0;
  int       iDelta    = 0;
  double    dDist, dParam;

  // for every point pair
  for(ii=0;ii<m_lSize;ii++)
    {
      sPrevPt = sThisPt;
      sThisPt = m_pData[ii];

      // First check 'on': coincident point.
      if ( sThisPt.CloserThan( m_dDistTol3d, crPoint ) )
        { return SM_POC_ON_BOUNDARY; }

      iThisQuad = CalcQuadNum( crPoint, sThisPt );
      iDelta = iThisQuad - iPrevQuad;

      if ( iDelta != 0 )
        {
          // First check 'on': on line segment.
          if ( smgu_IsPointOnSegment( sPrevPt, sThisPt, crPoint, m_dDistTol3d, dDist, dParam ) )
            { return SM_POC_ON_BOUNDARY; }

          if ( iDelta == 1 || iDelta == -3 )
            { iAccum++; }
          else if ( iDelta == -1 || iDelta == 3 )
            { iAccum--; }
          else
            {
              // +- 2: diagonal.
              // See whether we're heading CW or CCW around the test point:
              // use a cross product.
              // Note, we've already checked that the point is not on this segment,
              // so this segment does not pass right through the point.

              SmVector3d sCross = ( sPrevPt-crPoint ) * ( sThisPt - crPoint );
              double dDot = sCross.Dot( m_sNormal );
              if ( dDot > 0 )
                { iAccum += 2; } // CCW
              else
                { iAccum -= 2; } // CW
            }
        } // end if iDelta != 0

      iPrevQuad = iThisQuad;

    } // end loop on all points

  // The result depends on iAccum.
  if ( iAccum == 0 )
    { return SM_POC_OUTSIDE; }
  else if ( iAccum == -4 || iAccum == 4 )
    { return SM_POC_INSIDE; }

  // Still here? Allow for bit error in the quadrant classifications.
  if ( iAccum == -1 || iAccum == 1 )
    { return SM_POC_OUTSIDE; }
  else if (   iAccum == -3 || iAccum == 3
           || iAccum == -5 || iAccum == 5 )
    { return SM_POC_INSIDE; }

  // Should never get here.
  SM_DBG_WARN(_T("Error: SmPointSequence::PointContainment() Failed.\n") );
  return SM_POC_UNKNOWN;

} // end SmPointSequence::PointContainment

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmPointSequence::IsKindOf( SM_TYPE t ) const
{
  return ((SmPointSequence_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print class object

NOTES:
***********************************************************************/
void SmPointSequence::Dump()
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  smos_sprintf(sBuff,        _T("\nSmPointSequence = 0x%p: %s"),
             this,
             m_bClosed ? _T("   Closed") : _T("NotClosed"));
  smos_sprintf(sBuffForFile, _T("\nSmPointSequence = %s: %s"),
             _T( "NotNULL" ),
             m_bClosed ? _T("   Closed") : _T("NotClosed"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // cached Area
  if(!m_bAreaSet)   { smos_sprintf(sBuff, _T("%s"),_T(", Area Not Yet Cached")) ; }
  else              { smos_sprintf(sBuff, _T(", Area:[%16.16lf]"), m_dArea) ; }
  smos_WriteBuffer(sBuff);

  // cached Coincident
  if(!m_bCoincidentSet) { smos_sprintf(sBuff, _T("%s"),_T(", Coincident Not Yet Cached")) ; }
  else                  { smos_sprintf(sBuff, _T("%s"),_T(", %s Coincident Consecuctive Points"),
                                     m_bCoincident ? _T("HAS")
                                                   : _T("Does Not HAVE")) ; }
  smos_WriteBuffer(sBuff);

    // cached Monotonic
  if(!m_bMonotonicSet) { smos_sprintf(sBuff, _T("%s"),_T(", Monotonic Not Yet Cached")) ; }
  else                 { smos_sprintf(sBuff, _T("%s"),_T(", %s Monotonic Consecuctive Points"),
                                    m_bMonotonic ? _T("")
                                                 : _T("NON")) ; }
  smos_WriteBuffer(sBuff);

  // pretty print base class
  SmPointSet3d::Dump() ;

} // end SmPointSequence::Dump

/***************************************************************
PURPOSE:  Add point sequence graphics for global display Parameters
             to new DisplayList added to global DisplayList array.

NOTES:
***************************************************************/
SmDisplayList * SmPointSequence::Draw
 (SmBoolean bAddToUIPickList,  // in : TRUE = Add this PointSequence to UI pick interface for debugging
                               //      default:[FALSE]
  SmBoolean bDrawParams,       // in : TRUE = annotate point sequence poly line
                               //      increasingly large and varying color point icons.
                               //      default:[FALSE]
  SmGfxArraySet *pOptGfxSet)
 const
{
  // init return value
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // when asked - add this PointSequence to UI pick list
  if(bAddToUIPickList)
    { sm_GraphicsAddToBrepList(this) ; }

  // start new DisplayList (unless displayList is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // Add Curve Graphics using this object's color
  smgfx_OutputPolyline((double *)GetDataArray(), GetSize(), 3, pOptGfxSet);

  // when point parameterization
  if(bDrawParams)
    {
      ((SmPointSequence *)this)->SetUpAxes() ;
      double dSize = m_sBBox.GetMaxDimension() * .75 ;

      // save current color and PointSize
      sColor = smgfx_OutputColor(0,0,0) ;
      double dPointSize = smgfx_OutputPointSize(1.0, pOptGfxSet) ;

      if(m_bCentroidSet && m_bAxesSet)
        {
          // draw the coordinates
          smgfx_OutputColor(0.0, 0.0, 0.0, pOptGfxSet) ;
          smgfx_OutputLine(m_sCentroid.x,
                           m_sCentroid.y,
                           m_sCentroid.z,
                           m_sCentroid.x + dSize*m_sXAxis.x,
                           m_sCentroid.y + dSize*m_sXAxis.y,
                           m_sCentroid.z + dSize*m_sXAxis.z, pOptGfxSet) ;

          smgfx_OutputColor(0.0, 0.0, 0.0, pOptGfxSet) ;
          smgfx_OutputLine(m_sCentroid.x,
                           m_sCentroid.y,
                           m_sCentroid.z,
                           m_sCentroid.x + dSize*m_sYAxis.x,
                           m_sCentroid.y + dSize*m_sYAxis.y,
                           m_sCentroid.z + dSize*m_sYAxis.z, pOptGfxSet) ;
        } // end X and Y axes draw

      if(m_bCentroidSet && m_bNormalSet)
        {
          smgfx_OutputColor(1.0, 0.0, 0.0, pOptGfxSet) ;
          smgfx_OutputLine(m_sCentroid.x,
                           m_sCentroid.y,
                           m_sCentroid.z,
                           m_sCentroid.x + dSize*m_sNormal.x,
                           m_sCentroid.y + dSize*m_sNormal.y,
                           m_sCentroid.z + dSize*m_sNormal.z, pOptGfxSet) ;
        } // end Normal draw

      // draw BestFit
      if(m_bCentroidSet)
        {
          smgfx_OutputColor(0.0, 0.0, 1.0, pOptGfxSet) ;
          smgfx_OutputPoint(m_sCentroid.x,
                            m_sCentroid.y,
                            m_sCentroid.z, pOptGfxSet) ;
        }

      if(m_bPointSetTypeSet && m_ePointSetType == SM_PST_LINEAR)
        {
          smgfx_OutputLine(m_sCentroid.x - dSize*m_sXAxis.x,
                           m_sCentroid.y - dSize*m_sXAxis.y,
                           m_sCentroid.z - dSize*m_sXAxis.z,
                           m_sCentroid.x + dSize*m_sXAxis.x,
                           m_sCentroid.y + dSize*m_sXAxis.y,
                           m_sCentroid.z + dSize*m_sXAxis.z, pOptGfxSet) ;
        }

      if(   m_bPointSetTypeSet
         && (   m_ePointSetType == SM_PST_PLANAR
             || m_ePointSetType == SM_PST_SCATTERED))
        {
          smgfx_DrawPlane(m_sCentroid, m_sNormal, pOptGfxSet) ;
        }

      // for every Sample Point
      ULONG ii ;
      for(ii=0;ii<GetSize();ii++)
        {
          double dPar = (double)ii / (double)(GetSize()-1) ;

          // change color and point size
          smgfx_OutputColor(0.0, 1.0-dPar, dPar, pOptGfxSet) ;
          smgfx_OutputPointSize(dPointSize * (1.0+dPar), pOptGfxSet) ;

          // the point
          const SmPoint3d &rPt = GetAt(ii) ;

          // draw graphics for every sample point
          smgfx_OutputPoint(rPt.x, rPt.y, rPt.z, pOptGfxSet);

        } // end iter SamplePoint

      // restore current color and PointSize
      smgfx_OutputColor(sColor, pOptGfxSet) ;
      smgfx_OutputPointSize(dPointSize, pOptGfxSet) ;

    } // end drawing params

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(bAddToUIPickList, bDrawParams, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmPointSequence::Draw

/*******************************************************************//**
PURPOSE:  Display point sequence as a poly line annotated with
             a growing size and changing color icon for each point
             moving from the 1st point to the last.

NOTES:
***********************************************************************/
SmDisplayList * SmPointSequence::DrawParams
 (SmBoolean bAddToUIPickList,  // in : TRUE = Add this PointSequence to UI pick interface for debugging
                               //      default:[FALSE]
  SmGfxArraySet *pOptGfxSet)

 const
{
  // pass the call along
  return( Draw(bAddToUIPickList, TRUE, pOptGfxSet) ) ;

} // end SmPointSequence::DrawParams


/*******************************************************************//**
  END SmPointSequence
***********************************************************************/

/*******************************************************************//**
  BEGIN SmPointGrid
***********************************************************************/
/*******************************************************************//**
PURPOSE: Empty Object Constructor.

NOTES:
   The Closed flag is as specified, always.
***********************************************************************/
SmPointGrid::SmPointGrid
  (ULONG       nDataSize,        // in : initial m_pData array size or size of given pOptPtrArray, default:[0]
   SmPoint3d * pOptPtrArray,     // in : optional preallocated array to use for storage, default:[NULL]
   ULONG       lInitialSize)     // in : initial array size, less than or equal to nDataSize, default:[0]
 : SmPointSet3d(nDataSize, pOptPtrArray, lInitialSize),
   m_lRowSize(nDataSize)
{
  // set all cache state booleans to FALSE - don't call ClearCache() because SmPointSet3d may have set some cache values
  m_bCoincidentSet = FALSE ;
  m_bMonotonicSet  = FALSE ;

} // end SmPointGrid::SmPointGrid Empty Constructor

/*******************************************************************//**
PURPOSE: Default Constructor.

NOTES:
   The Closed flag is as specified, always.
***********************************************************************/
SmPointGrid::SmPointGrid
 (SmTArray< SmPoint3d >  & sPoints,     // in : array copied or converted into PointGrid
  ULONG                    lRowSize,    // in : number of points in each row of the grid
  double                   dTol3d,      // in : min dist between distinct points, default:[SM_EFF_ZERO]
  SmBoolean                bClosedCol,  // in : TRUE = 1st and last U point in each Column are connected, default:[FALSE]
  SmBoolean                bClosedRow,  // in : TRUE = 1st and last V point in each Row are connected, default:[FALSE]
  SmBoolean                bPlanar,     // in : TRUE = PointSequence is known to lie on a plane, default:[UNSURE]
  SmVector3d             * pOptNormal,  // in : NULL=system computes m_sNormal, NotNULL=user given vector as Normal, default:[NULL]
  SmBoolean                bCopyData)   // in : TRUE=Copy sPoints into a new array - sPoints and this m_pData arrays are different
                                        //      FALSE=Share sPoints with new array - sPoints and this m_pData arrays are same
                                        //      default:[TRUE]
 : SmPointSet3d(sPoints, dTol3d, bPlanar, pOptNormal, bCopyData),
   m_lRowSize(lRowSize),
   m_bClosedCol(bClosedCol),
   m_bClosedRow(bClosedRow)
{
  // set all cache state booleans to FALSE - don't call ClearCache() because SmPointSet3d may have set some cache values
  m_bCoincidentSet = FALSE ;
  m_bMonotonicSet  = FALSE ;

  // inform the public of bad row sizes
  if(!IsGoodRowSize())
    { WARN(_T("SmPointGrid constructor: Given RowSize does not break up PointArray into an even number of full rows")) ; }

} // end SmPointGrid::SmPointGrid Default Constructor

/*******************************************************************//***
PURPOSE: Copy Constructor

NOTES:
************************************************************************/
SmPointGrid::SmPointGrid
  ( const SmPointGrid &crOther ) // in : object to copy
 : SmPointSet3d(crOther)         // eff: copy the base class
{
  // copy members
  m_lRowSize       = crOther.m_lRowSize ;
  m_bClosedCol     = crOther.m_bClosedCol ;
  m_bClosedRow     = crOther.m_bClosedRow ;
  m_bCoincidentSet = crOther.m_bCoincidentSet ;
  m_bCoincidentCol = crOther.m_bCoincidentCol ;
  m_bCoincidentRow = crOther.m_bCoincidentRow ;
  m_bMonotonicSet  = crOther.m_bMonotonicSet ;
  m_bMonotonicCol  = crOther.m_bMonotonicCol ;
  m_bMonotonicRow  = crOther.m_bMonotonicRow ;

} // end SmPointGrid::SmPointGrid copy constructor

/*******************************************************************//***
PURPOSE: Assignment operator.

NOTES:
************************************************************************/
SmPointGrid & SmPointGrid::operator=( const SmPointGrid &crOther )
{
  // no work condition
  if(&crOther == this) return *this ;

  // copy base values
  this->SmPointSet3d::operator=(crOther) ;

  // copy members
  m_lRowSize       = crOther.m_lRowSize ;
  m_bClosedCol     = crOther.m_bClosedCol ;
  m_bClosedRow     = crOther.m_bClosedRow ;
  m_bCoincidentSet = crOther.m_bCoincidentSet ;
  m_bCoincidentCol = crOther.m_bCoincidentCol ;
  m_bCoincidentRow = crOther.m_bCoincidentRow ;
  m_bMonotonicSet  = crOther.m_bMonotonicSet ;
  m_bMonotonicCol  = crOther.m_bMonotonicCol ;
  m_bMonotonicRow  = crOther.m_bMonotonicRow ;

  // all done
  return *this;

} // end SmPointGrid::operator=

/*******************************************************************//***
PURPOSE: Deep Equality operator

NOTES: compares the values of the stored points
   rather than their memory locations.
   (here so that we can have an SmTArray of them).
************************************************************************/
SmBoolean SmPointGrid::operator==( const SmPointGrid &crOther )
{
  // no work condition
  if(&crOther == this) return TRUE ;

  if( !this->SmPointSet3d::operator==(crOther) ) { return FALSE ; }

  if( !(m_lRowSize      == crOther.m_lRowSize )       ) { return FALSE ; }
  if( !(m_bClosedCol    == crOther.m_bClosedCol)      ) { return FALSE ; }
  if( !(m_bClosedRow    == crOther.m_bClosedRow)      ) { return FALSE ; }
  if( !(m_bCoincidentSet == crOther.m_bCoincidentSet) ) { return FALSE ; }
  if( !(m_bCoincidentCol == crOther.m_bCoincidentCol) ) { return FALSE ; }
  if( !(m_bCoincidentRow == crOther.m_bCoincidentRow) ) { return FALSE ; }
  if( !(m_bMonotonicSet == crOther.m_bMonotonicSet)   ) { return FALSE ; }
  if( !(m_bMonotonicCol == crOther.m_bMonotonicCol)   ) { return FALSE ; }
  if( !(m_bMonotonicRow == crOther.m_bMonotonicRow)   ) { return FALSE ; }

  return TRUE;

} // end SmPointGrid::operator==

/*******************************************************************//***
PURPOSE: Cache whether any of the coincident points are coincident

NOTES: When any one row has coincident points m_bCoincidentRow == TRUE
       When any one col has coincident points m_bCoincidentCol == TRUE
************************************************************************/
SmStatus SmPointGrid::SetUpCoincident()
{
  // no work - already computed
  if(m_bCoincidentSet)
    {
      return SM_SUCCESS;
    }

  // Set state
  m_bCoincidentSet = TRUE ;
  m_bCoincidentCol = FALSE ;
  m_bCoincidentRow = FALSE ;

  // locals
  ULONG idx ;
  ULONG ii, i0, i1, jj, j0, j1 ;
  ULONG lColCnt = GetColCnt() ;
  ULONG lRowCnt = GetRowCnt() ;

  // for every row
  for(idx=0,ii=0;ii<lRowCnt && m_bCoincidentRow==FALSE;ii++)
    {
      // pick iteration bounds
      if(m_bClosedRow) { j0 = idx + m_lRowSize - 1 ;
                         j1 = idx ;
                       }
      else             { j0 = idx ;
                         j1 = idx+1 ;
                       }

      // next row start
      idx += m_lRowSize ;

      // for every pair of points in this row
      for(;j1<idx;j1++,j0=j1-1)
        {
          // when Two points are coincident to tolerance
          if( m_pData[j0].CloserThan(m_dDistTol3d, m_pData[j1]))
            {
              // the Point sequence is coincident
              m_bCoincidentRow = TRUE ;
              break ;
            } // end found a coincident point pair
        } // end iter every pair of row points
    } // end iter every row

  // for every col
  for(idx=0,jj=0;jj<lColCnt && m_bCoincidentCol==FALSE;jj++)
    {
      // pick iteration bounds
      if(m_bClosedCol) { i0 = idx + m_lSize - m_lRowSize ;
                         i1 = idx ;
                       }
      else             { i0 = idx  ;
                         i1 = idx + m_lRowSize ;
                       }

      // next col start
      idx ++ ;

      // for every pair of points in this col
      for(;i1<m_lSize;i1+=m_lRowSize,i0=i1-m_lRowSize)
        {
          // when Two points are coincident to tolerance
          if( m_pData[i0].CloserThan(m_dDistTol3d, m_pData[i1]))
            {
              // the Point sequence is coincident
              m_bCoincidentCol = TRUE ;
              break ;
            } // end found a coincident point pair
        } // end iter every pair of col points
    } // end iter every col

  // all done
  return SM_SUCCESS;

} // end SmPointGrid::SetUpCoincident

/*******************************************************************//***
PURPOSE: Cache whether the rows and column points are in a monotonic position

NOTES: Might not be meaninful if not planar.
   If total failure (of SetUpMonotonic) or when grid is closed,
   then Monotonic values are set to FALSE
************************************************************************/
SmStatus SmPointGrid::SetUpMonotonic()
{
  // no work - already computed
  if(m_bMonotonicSet)
    {
      return SM_SUCCESS;
    }

  // Set state.
  m_bMonotonicSet = TRUE;
  m_bMonotonicCol = TRUE;
  m_bMonotonicRow = TRUE;

  // locals
  ULONG idx ;
  ULONG ii, i0, i1, jj, j0, j1 ;
  ULONG lColCnt = GetColCnt() ;
  ULONG lRowCnt = GetRowCnt() ;
  SmVector3d sVec0, sVec1 ;

  // First the rows (constant i, varying j)

  // if we switch the test to check for global monotonicity rather than local
  // we could take the following short cut
  //    if(m_bClosedRow)
  //      { m_bMonotonicRow = FALSE ; }
  //    else

  // for every row - check for local monotonicity
  for(idx=0,ii=0;ii<lRowCnt && m_bMonotonicRow==TRUE;ii++)
    {
      // pick iteration bounds for closed and open cases
      if(m_bClosedRow) { j0 = idx + m_lRowSize - 1 ;
                         j1 = idx ;
                       }
      else             { j0 = idx ;
                         j1 = idx+1 ;
                       }

      // get first point pair difference
      sVec0 = m_pData[j1] - m_pData[j0] ;
      double dLenSq0 = sVec0.LengthSquared() ;

      // next row start
      idx += m_lRowSize ;

      // for every pair of points in this row
      for(j1++,j0=j1-1;j1<idx;j1++,j0=j1-1)
        {
          // get this point pair difference
          sVec1 = m_pData[j1] - m_pData[j0] ;
          double dLenSq1 = sVec1.LengthSquared() ;

          // check for non-monotonic cases
          // GWC: we have a choice here
          //      1. Check next difference to last difference  - finds only large changes between neighbors
          //      2. Check next difference to first difference - finds changes across whole row
          // scheme 2 will find false nonMonotonicities for grids on disks that are okay
          // so I'm going to initially use scheme 1.

          // skip coincident control points - gwc: could look for nonmonotonicity accross coincident CPts
          if(   dLenSq0 < SM_EFF_ZERO_SQ
             || dLenSq1 < SM_EFF_ZERO_SQ)
            {
              sVec0 = sVec1 ;
              dLenSq0 = dLenSq1 ;
              continue ;
            }

          // look for change in direction
          double dAngRad ;
          sVec1.AngleBetween(sVec0, dAngRad) ;  // out: 0 to Pi
          double dAngDeg = SM_RAD2DEG(dAngRad) ;
          // double dDot = sVec1.Dot(sVec0) ;
          // if(dDot < - SM_EFF_ZERO)

          // when clearly not monotonic
          // gwc: so here's an arbitrary choice.
          //      When (dDot < - SM_EFF_ZERO) the angle between sVec1 and sVec0 is greater than 90 degrees this code says there's a problem.
          //      However circles are made where the angle between sVec1 and sVec0 is exactly 90 degrees.
          //      We could use any arbitrary angle here, perhaps 120 degrees or 160?  We'll see how this works out.
          if(dAngDeg > 170.0)
            {
              // mark that and move on to col checks
              m_bMonotonicRow = FALSE ;
              break ;
            }

          // GWC: exactly zero cases happen for coincident points and orthogonal turns
          //      let's assume that we only need to detect grossly non monotonic cases.

          // ready for next iteration - to switch to case 2 comment out next line
          sVec0 = sVec1 ;

        } // end iter every pair of row points
    } // end iter every row

  // for every col
  for(idx=0,jj=0;jj<lColCnt && m_bMonotonicCol==TRUE;jj++)
    {
      // pick iteration bounds for closed and open cases
      if(m_bClosedCol) { i0 = idx + m_lSize - m_lRowSize ;
                         i1 = idx ;
                       }
      else             { i0 = idx  ;
                         i1 = idx + m_lRowSize ;
                       }

      // get first point pair difference
      sVec0 = m_pData[i1] - m_pData[i0] ;
      double dLenSq0 = sVec0.LengthSquared() ;

      // next col start
      idx ++ ;

      // for every pair of points in this col
      for(i1+=m_lRowSize,i0=i1-m_lRowSize;i1<m_lSize;i1+=m_lRowSize,i0=i1-m_lRowSize)
        {
          // get this point pair difference
          sVec1 = m_pData[i1] - m_pData[i0] ;
          double dLenSq1 = sVec1.LengthSquared() ;

          // skip coincident control points - gwc: could look for nonmonotonicity accross coincident CPts
          if(   dLenSq0 < SM_EFF_ZERO_SQ
             || dLenSq1 < SM_EFF_ZERO_SQ)
            {
              sVec0 = sVec1 ;
              dLenSq0 = dLenSq1 ;
              continue ;
            }

          // look for change in direction
          double dAngRad ;
          sVec1.AngleBetween(sVec0, dAngRad) ;  // out: 0 to Pi
          double dAngDeg = SM_RAD2DEG(dAngRad) ;
          // double dDot = sVec1.Dot(sVec0) ;
          // if(dDot < - SM_EFF_ZERO)

          // when clearly not monotonic
          // gwc: so here's an arbitrary choice.
          //      When (dDot < - SM_EFF_ZERO) the angle between sVec1 and sVec0 is greater than 90 degrees this code says there's a problem.
          //      However circles are made where the angle between sVec1 and sVec0 is exactly 90 degrees.
          //      We could use any arbitrary angle here, perhaps 120 degrees or 160?  We'll see how this works out.
          if(dAngDeg > 170.0)
            {
              // mark that and move on to col checks
              m_bMonotonicRow = FALSE ;
              break ;
            }

          // GWC: exactly zero cases happen for coincident points and orthogonal turns
          //      let's assume that we only need to detect grossly non monotonic cases.

          // ready for next iteration - to switch to case 2 comment out next line
          sVec0 = sVec1 ;

        } // end iter every pair of col points
    } // end iter every col

  // all done
  return SM_SUCCESS ;

} // end SmPointGrid::SetUpMonotonic

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmPointGrid::IsKindOf( SM_TYPE t ) const
{
  return ((SmPointGrid_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print class object

NOTES:
***********************************************************************/
void SmPointGrid::Dump()
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  smos_sprintf(sBuff,        _T("\nSmPointGrid = 0x%p: RowSize:[%ld], %s, %s"),
             this,
             m_lRowSize,
             m_bClosedCol ? _T("Closed Cols") : _T("Open Cols"),
             m_bClosedRow ? _T("Closed Rows") : _T("Open Rows"));
  smos_sprintf(sBuffForFile, _T("\nSmPointGrid = %s: RowSize:[%ld], %s, %s"),
             _T( "NotNULL" ),
             m_lRowSize,
             m_bClosedCol ? _T("Closed Cols") : _T("Open Cols"),
             m_bClosedRow ? _T("Closed Rows") : _T("Open Rows"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // cached
  if(!m_bCoincidentSet) { smos_sprintf(sBuff, _T("%s"),_T(", Coincident Not Yet Cached")) ; }
  else                  { smos_sprintf(sBuff, _T("%s"),_T(", %s Coincident Consecuctive Col Points"),
                                     m_bCoincidentCol ? _T("HAS")
                                                      : _T("NO")) ;
                          smos_WriteBuffer(sBuff);
                          smos_sprintf(sBuff, _T(", %s Coincident Consecuctive Row Points"),
                                     m_bCoincidentRow ? _T("HAS")
                                                   : _T("NO")) ;
                        }
  smos_WriteBuffer(sBuff);

  if(!m_bMonotonicSet)  { smos_sprintf(sBuff, _T("%s"),_T(", Monotonic Not Yet Cached")) ; }
  else                  { smos_sprintf(sBuff, _T("%s"),_T(", %s Monotonic Consecuctive Col Points"),
                                     m_bCoincidentCol ? _T("")
                                                      : _T("NON")) ;
                          smos_WriteBuffer(sBuff);
                          smos_sprintf(sBuff, _T("%s"),_T(", %s Monotonic Consecuctive Row Points"),
                                     m_bCoincidentRow ? _T("")
                                                      : _T("NON")) ;
                        }
  smos_WriteBuffer(sBuff);

  // pretty print base class
  SmPointSet3d::Dump() ;

} // end SmPointGrid::Dump

/***************************************************************
PURPOSE:  Add point sequence graphics for global display Parameters
             to new DisplayList added to global DisplayList array.

NOTES:
***************************************************************/
SmDisplayList * SmPointGrid::Draw
  (SmBoolean bAddToUIPickList,  // in : TRUE = Add this PointSequence to UI pick interface for debugging
                                //      default:[FALSE]
   SmBoolean bDrawParams,       // in : TRUE = annotate point sequence poly line
                                //      increasingly large and varying color point icons.
                                //      default:[FALSE]
   SmGfxArraySet *pOptGfxSet)
 const
{
  // init return value
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // locals
  ULONG ii, jj, idx ;
  ULONG lRowCnt = GetRowCnt() ;
  ULONG lColCnt = GetColCnt() ;

  // when asked - add this PointSequence to UI pick list
  if(bAddToUIPickList)
    { sm_GraphicsAddToBrepList(this) ; }

  // start new DisplayList (unless displayList is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // for every row draw a PolyLine
  for(ii=0,idx=0;ii<lRowCnt;ii++,idx+=m_lRowSize)
    {
      smgfx_OutputPolyline((double *)(GetDataArray()+idx), lColCnt, 3, pOptGfxSet);
    }

  // for every col draw a PolyLine
  for(jj=0;jj<lColCnt;jj++)
    {
      smgfx_OutputPolyline((double *)(GetDataArray()+jj), lRowCnt, 3*lColCnt, pOptGfxSet);
    }

  // for every point draw a Point Icon
  for(ii=0;ii<m_lSize;ii++)
    {
      smgfx_OutputPoint( m_pData[ii].x, m_pData[ii].y, m_pData[ii].z) ;
    }


  // when point parameterization
  if(bDrawParams)
    {
      ((SmPointGrid *)this)->SetUpAxes() ;
      double dSize = m_sBBox.GetMaxDimension() * .75 ;

      // save current color and PointSize
      sColor = smgfx_OutputColor(0,0,0) ;
      double dPointSize = smgfx_OutputPointSize(1.0, pOptGfxSet) ;

      if(m_bCentroidSet && m_bAxesSet)
        {
          // draw the coordinates
          smgfx_OutputColor(0.0, 0.0, 0.0, pOptGfxSet) ;
          smgfx_OutputLine(m_sCentroid.x,
                           m_sCentroid.y,
                           m_sCentroid.z,
                           m_sCentroid.x + dSize*m_sXAxis.x,
                           m_sCentroid.y + dSize*m_sXAxis.y,
                           m_sCentroid.z + dSize*m_sXAxis.z, pOptGfxSet) ;

          smgfx_OutputColor(0.0, 0.0, 0.0, pOptGfxSet) ;
          smgfx_OutputLine(m_sCentroid.x,
                           m_sCentroid.y,
                           m_sCentroid.z,
                           m_sCentroid.x + dSize*m_sYAxis.x,
                           m_sCentroid.y + dSize*m_sYAxis.y,
                           m_sCentroid.z + dSize*m_sYAxis.z, pOptGfxSet) ;
        } // end X and Y axes draw

      if(m_bCentroidSet && m_bNormalSet)
        {
          smgfx_OutputColor(1.0, 0.0, 0.0, pOptGfxSet) ;
          smgfx_OutputLine(m_sCentroid.x,
                           m_sCentroid.y,
                           m_sCentroid.z,
                           m_sCentroid.x + dSize*m_sNormal.x,
                           m_sCentroid.y + dSize*m_sNormal.y,
                           m_sCentroid.z + dSize*m_sNormal.z, pOptGfxSet) ;
        } // end Normal draw

      // draw BestFit
      if(m_bCentroidSet)
        {
          smgfx_OutputColor(0.0, 0.0, 1.0, pOptGfxSet) ;
          smgfx_OutputPoint(m_sCentroid.x,
                            m_sCentroid.y,
                            m_sCentroid.z, pOptGfxSet) ;
        }

      if(m_bPointSetTypeSet && m_ePointSetType == SM_PST_LINEAR)
        {
          smgfx_OutputLine(m_sCentroid.x - dSize*m_sXAxis.x,
                           m_sCentroid.y - dSize*m_sXAxis.y,
                           m_sCentroid.z - dSize*m_sXAxis.z,
                           m_sCentroid.x + dSize*m_sXAxis.x,
                           m_sCentroid.y + dSize*m_sXAxis.y,
                           m_sCentroid.z + dSize*m_sXAxis.z, pOptGfxSet) ;
        }

      if(   m_bPointSetTypeSet
         && (   m_ePointSetType == SM_PST_PLANAR
             || m_ePointSetType == SM_PST_SCATTERED))
        {
          smgfx_DrawPlane(m_sCentroid, m_sNormal, pOptGfxSet) ;
        }

      // restore current color and PointSize
      smgfx_OutputColor(sColor, pOptGfxSet) ;
      smgfx_OutputPointSize(dPointSize, pOptGfxSet) ;

    } // end drawing params

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(bAddToUIPickList, bDrawParams, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmPointGrid::Draw

/*******************************************************************//**
PURPOSE:  Display point sequence as a poly line annotated with
             a growing size and changing color icon for each point
             moving from the 1st point to the last.

NOTES:
***********************************************************************/
SmDisplayList * SmPointGrid::DrawParams
  (SmBoolean bAddToUIPickList,  // in : TRUE = Add this PointSequence to UI pick interface for debugging
                                //      default:[FALSE]
   SmGfxArraySet *pOptGfxSet)

 const
{
  // pass the call along
  return( Draw(bAddToUIPickList, TRUE, pOptGfxSet) ) ;

} // end SmPointGrid::DrawParams


/*******************************************************************//**
  END SmPointGrid
***********************************************************************/
