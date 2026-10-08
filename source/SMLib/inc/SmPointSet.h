// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- IsPointSet.h
* PURPOSE: Header file for SmPointSequence, SmPointSet2d, and SmPointSet3d classes.
**********************************************************************/

#ifndef __SMPOINTSET_H__
#define __SMPOINTSET_H__

#include <SmCoreTypes.h>
#include <SmVector3d.h>
#include <SmVector2d.h>
#include <SmExtent3d.h>
#include <SmExtent2d.h>
#include <SmTArray.h>
#include <SmPseudoBox.h>

class SmGfxArraySet ;

// template class SM_EXPORT SmTArray< SmVector3d >;

/*******************************************************************//**
PURPOSE: enum to control the lazy evaluation of SmPointSet3d cached properties

NOTES: These are assigned bit values so they can be treated as bit arrays internally
***********************************************************************/
enum SmPointSetDataType
{
  SM_PSD_AXES=1,                    // cache Normal, XAxis, and YAxis properties
  SM_PSD_CENTROID=2,                // cache centroid property
  SM_PSD_POINTSET_TYPE=4,           // cache SmPointSetType m_ePointSetType property
  SM_PSD_MAX_DEVIATION=8,           // cache Max Deviation from best fit plane, line, or point
  SM_PSD_MAX_CENTROID_DEVIATION=16, // cache Max Deviation from centroid
  SM_PSD_BBOXES=32,                 // cache Aligned and Pseudo bounding boxes
  SM_PSD_REFINE_PBOX=64,            // rotate Pseudo box X and Y axes to minize PseudoBox size
  SM_PSD_ALL=127,                   // cache all PointSet properties
} ;

/*******************************************************************//**
PURPOSE: A class to represent an unordered set of 3d points.

NOTES: 1.) PointSet3d Coordinate system [m_sCentroid, m_sXAxis, m_sYAxis, m_sNormal]:
         a.) The plane [m_sCentroid, m_sNormal] is either the best fit plane to the points 
             or user given when known in advance. When user given its not checked to see that                             
             it is the best fit plane.  For planar cases all points are with dDistTol3d of this plane.
         b.) m_sXAxis is the BestLine tangent direction for a linear PointSet.
***********************************************************************/
class SM_EXPORT SmPointSet3d : public SmTArray<SmPoint3d>
{
protected:
  // inherited from SmObject
  //   SmContext * m_pContext ;         // access to 'global' memory
  //
  // inherited from SmTArray<SmPoint3d>
  //   SmBoolean   m_bIsBorrowed;       // TRUE = m_pData is borrowed
  //   SmPoint3d * m_pData;             // the ordered array of SmPoint3d
  //   ULONG       m_lSize;             // GetSize() of m_pData     (number of SmPoint3d spots used)
  //   ULONG       m_nMemorySize;       // GetDataSize() of m_pData (Number of SmPoint3d spots allocated)

  SmTol3d          m_dDistTol3d;        // Minimum distance bewteen distinct points
                   
  SmBoolean        m_bPointSetTypeSet;  // TRUE = m_ePointSetType set, FALSE = awaiting lazy evaluation
  SmPointSetType   m_ePointSetType ;    // one of SM_PST_VOID,          // empty point set
                                        //        SM_PST_POINTSIZED,    // all points within tol of centroid
                                        //        SM_PST_LINEAR,        // all points within tol of a centroid intersecting line
                                        //        SM_PST_PLANAR,        // all points within tol of a centroid intersecting plane
                                        //        SM_PST_SCATTERED      // points are not within tolerance of a point, line or plane
                   
  SmBoolean        m_bCentroidSet;      // TRUE = m_sCentroid set, FALSE = awaiting lazy evaluation
  SmPoint3d        m_sCentroid;         // PointSet geometric center
                   
  SmBoolean        m_bAxesSet;          // TRUE = m_sXAxis, m_sYAxis, m_sNormal set, FALSE = awaiting lazy evaluation
  SmVector3d       m_sXAxis;            // perp to Y and n. When linear = BestLine tangent     
  SmVector3d       m_sYAxis;            // perp to X and n.  
                   
  SmBoolean        m_bNormalSet;        // TRUE  = m_sNormal set (by SetUpAxes() or user input), FALSE = awaiting lazy evaluation
  SmVector3d       m_sNormal;           // perp to m_sXAxis and m_sYAxis. When planar or scattered = BestPlane normal
  SmBoolean        m_bPlanar;           // TRUE  = points lie within tolerance of some plane.
                                        //         May be specifed by user through constructor, otherwise calculated.
                                      
  SmBoolean      m_bMaxCentroidDevSet;  // TRUE  = m_dMaxCentroidDev set, FALSE = awaiting lazy evaluation
  double         m_dMaxCentroidDev;     // max dist between crPoints[i] and Centroid                        
                   
  SmBoolean        m_bMaxDevSet;        // TRUE = m_dMaxDev set, FALSE = awaiting lazy evaluation
  double           m_dMaxDev;           // max dist SM_PST_POINTSIZED: between crPoints[i] and Centroid
                                        //          SM_PST_LINEAR    : between crPoints[i] and BestLine
                                        //          SM_PST_PLANAR    : between crPoints[i] and BestPlane 
                                        //          SM_PST_SCATTERED : between crPoints[i] and BestPlane  
                                       
  SmBoolean        m_bBBoxSet;          // TRUE = m_sBBox and m_sPBox set, FALSE = awaiting lazy evaluation
  SmBoolean        m_bPBoxRefined ;     // TRUE = m_sPBox defined by TightenPseudoBox call
  SmExtent3d       m_sBBox;             // Bounding Box
  SmPseudoBox      m_sPBox;             // Pseudo Box

  // Private methods:
  SmStatus SetUpData                               // eff: cache all data values
            (SmPointSetDataType eReason,           // oneof: SM_PSD_AXES                   = cache Normal, XAxis, and YAxis properties
                                                   //        SM_PSD_CENTROID               = cache centroid property
                                                   //        SM_PSD_POINTSET_TYPE          = cache SmPointSetType m_ePointSetType property
                                                   //        SM_PSD_MAX_DEVIATION          = cache Max Deviation from best fit plane, line, or point
                                                   //        SM_PSD_MAX_CENTROID_DEVIATION = cache Max Deviation from centroid
                                                   //        SM_PSD_BBOXES                 = cache Aligned and Pseudo bounding boxes
                                                   //        SM_PSD_REFINE_PBOX            = rotate Pseudo box X and Y axes to minize PseudoBox size
                                                   //        SM_PSD_ALL                    = cache all PointSet properties
              SmVector3d * pOptInputPBoxZ=NULL) ;  // in : used to set the basis[2] direction of the pseudoBox.
                                                   //      NULL to ignore, default:[NULL]

  SmStatus SetUpPointSetType() ;                   // eff: cache PointSetType
  SmStatus SetUpCentroid() ;                       // eff: cache Centroid
  SmStatus SetUpAxes() ;                           // eff: cache Normal, XAxis, YAxis
  SmStatus SetUpDeviation                          // eff: cache MaxCentroidDev, MaxDev
            (SmPointSetDataType eReason) ;         // oneof: SM_PSD_MAX_DEVIATION          = cache Max Deviation from best fit plane, line, or point
                                                   //        SM_PSD_MAX_CENTROID_DEVIATION = cache Max Deviation from centroid
                                                   //        SM_PSD_ALL                    = cach Max and MaxCentroid Deviations
  
  SmStatus SetUpBBoxes                             // eff: cache BBox, PBox
            (SmPointSetDataType eReason,           // oneof: SM_PSD_BBOXES      = cache axes aligned and Pseudo bounding boxes
                                                   //        SM_PSD_REFINE_PBOX = rotate Pseudo box X and Y axes to minize PseudoBox size
                                                   //        SM_PSD_ALL         = Cache and Refine BBoxes
             SmVector3d * pOptInputPBoxZ=NULL) ;   // in : used to set the basis[2] direction of the pseudoBox.
                                                   //      NULL to ignore, default:[NULL]

  SmStatus SetUpTightPseudoBox                     // eff: rotate PBox about Z axis to minimize extents containing points, not actual min but pretty good
            (SmBoolean bDoExtraPlanarWork,         // in : TRUE = align Pseudo box so that two consecutive points are
                                                   //             on the PseudoBox boundary.  Good for aligning PointSequence
                                                   //             polygons with a PseudoBox boundary.
                                                   //      FALSE= don't bother doing the extra work - the data is not planar
             SmVector3d * pOptInputPBoxZ=NULL) ;   // in : used to set the basis[2] direction of the pseudoBox.
                                                   //      NULL to ignore, default:[NULL]
public:
  // Empty Constructor
  SmPointSet3d(ULONG        nMemorySize  = 0,     // in : initial m_pData array size or size of given pOptPtrArray
               SmVector3d * pOptPtrArray = NULL,  // in : optional preallocated array to use for storage
               ULONG        nArraySize   = 0) ;   // in : initial array size, less than or equal to nMemorySize.
  
  // From SmTArray<SmPoint3d> array Constructor
  SmPointSet3d(SmTArray<SmPoint3d> & rTArray,            // in : array copied or converted to SmPointSet3d
               double                dTol3d=SM_EFF_ZERO, // in : min dist between distinct points, default:[SM_EFF_ZERO]                                    
               SmBoolean             bPlanar = UNSURE,   // NotUsed: in : TRUE = PointSequence is known to lie on a plane                          
               SmVector3d          * pOptNormal=NULL,    // in : NULL=system computes m_sNormal, NotNULL=user given vector as Normal
               SmBoolean             bCopyData=TRUE);    // in : TRUE=Copy sPoints into a new array - sPoints and this m_pData arrays are different
                                                         //      FALSE=Share sPoints with new array - sPoints and this m_pData arrays are same
                                                         //      default:[TRUE] 
  // Copy Constructor                                    
  SmPointSet3d ( const SmPointSet3d &crOther ) ; 

  // Assignment, equality operator
  SmPointSet3d & operator=  (const SmPointSet3d &crOther ) ;
  SmBoolean      operator== (const SmPointSet3d &crOther ) ;

  // clear the cached data - just in case a change was made to any of the contained points
  void ClearCache()                   { m_bAxesSet           = FALSE ;
                                        m_bNormalSet         = FALSE ;
                                        m_bCentroidSet       = FALSE ;
                                        m_bPointSetTypeSet   = FALSE ;
                                        m_bMaxDevSet         = FALSE ;
                                        m_bMaxCentroidDevSet = FALSE ;
                                        m_bBBoxSet           = FALSE ;
                                        m_bPBoxRefined       = FALSE ;
                                      }
  // simple access
  inline ULONG            Add( SmPoint3d sNewPoint3d ); // add Point3d to end of Point3d array
  SmTArray< SmPoint3d > & GetPoints()               { return *this; }
  SmTol3d                 GetTolerance() const      { return m_dDistTol3d ; }

  SmPointSetType          GetPointSetType()         { SetUpPointSetType() ; return m_ePointSetType ; }
  double                  GetMaxDev()               { SetUpDeviation(SM_PSD_MAX_DEVIATION) ;          return m_dMaxDev ; }
                          
  SmPoint3d               GetCentroid()             { SetUpCentroid() ; return m_sCentroid ; }
  double                  GetMaxCentroidDev()       { SetUpDeviation(SM_PSD_MAX_CENTROID_DEVIATION) ; return m_dMaxCentroidDev ; }

  SmVector3d              GetXAxis()                { SetUpAxes() ; return m_sXAxis ; }
  SmVector3d              GetYAxis()                { SetUpAxes() ; return m_sYAxis ; }
  SmVector3d              GetZAxis()                { SetUpAxes() ; return m_sNormal ; }
  SmVector3d              GetNormal()               { SetUpAxes() ; return m_sNormal ; }
                          
  SmExtent3d              GetBBox     (SmVector3d * pOptInputPBoxZ=NULL) { SetUpBBoxes(SM_PSD_BBOXES, pOptInputPBoxZ) ;      return m_sBBox ; }
  SmPseudoBox             GetPBox     (SmVector3d * pOptInputPBoxZ=NULL) { SetUpBBoxes(SM_PSD_BBOXES, pOptInputPBoxZ) ;      return m_sPBox ; }
  SmPseudoBox             GetTightPBox(SmVector3d * pOptInputPBoxZ=NULL) { SetUpBBoxes(SM_PSD_REFINE_PBOX, pOptInputPBoxZ) ; return m_sPBox ; }
  ULONG                   GetDimension()            // rtn: 2 = planar and parallel to the x-y plane, else 3
                                                    { SetUpAxes() ; SetUpPointSetType() ;
                                                      return(    m_ePointSetType == SM_PST_PLANAR
                                                              && m_sNormal.x < m_dDistTol3d
                                                              && m_sNormal.y < m_dDistTol3d ? 2 : 3) ;
                                                    }

  void            SetTolerance(SmTol3d &rTol) { if(m_dDistTol3d != rTol) { ClearCache() ; } m_dDistTol3d = rTol ; }

// to be obsoleted
  SmStatus        GetNormal   (SmVector3d & rNormal) ;    // rtn: TRUE for planar cases, else FALSE
  void            SetNormal   (SmVector3d sNormal)        { ClearCache() ; sNormal = m_sNormal ;
                                                            //  m_bNormalSet = TRUE ;
                                                          }
                  
  // predicates
  SmBoolean IsEmpty()      const { return (m_lSize == 0) ; }
  SmBoolean IsPointSized()       { SetUpPointSetType() ; return m_ePointSetType == SM_PST_POINTSIZED ; }
  SmBoolean IsLinear()           { SetUpPointSetType() ; return m_ePointSetType == SM_PST_LINEAR ; }
  SmBoolean IsPlanar()           { SetUpPointSetType() ; return m_ePointSetType == SM_PST_PLANAR ; }
  SmBoolean IsScattered()        { SetUpPointSetType() ; return m_ePointSetType == SM_PST_SCATTERED ; }

  // computations
  SmStatus  CalculateBoundingSphere(SmTArray<ULONG> &rIndices,        // out: indices of points found to be on minimum radius sphere
                                    SmPoint3d       &rCenter, 
                                    double          &dRadius) const ; 
  SmStatus  GetCoincidentPoints(SmTArray<SmPoint3d> &rCoinPoint1,        // runs in near linear time
                                SmTArray<SmPoint3d> &rCoinPoint2) ;      // reorder the Point array


//  // classify PointSet as oneof: void, point, linear, planar, or scattered
//  SmPointSetType Classify
//    (double        dTol3d             = SM_EFF_ZERO,  // in : min distance between unique points
//     SmPoint3d   * pOptCentroid       = NULL,         // out: geometric average (undefined for void PointSets), NULL to ignore
//     SmVector3d  * pOptVector         = NULL,         // out: Good Fit line tangent for linear PointSets,     
//                                                      //      Best Fit plane normal for planar PointSets,     
//                                                      //      else not used (set to undefined), NULL to ignore
//     double      * pOptDeviation      = NULL,         // out: Actual deviation, NULL to ignore
//     double      * pOptMaxCentroidDev = NULL,         // out: max dist to centroid
//     SmPseudoBox * pOptPseudoBox      = NULL) ;       // out: pseudo box built to classify the point set, NULL to ignore

  // simple access

  // Add Curve Graphics to new or open Display List
  SmDisplayList * Draw(SmBoolean bAddToUIPickList=FALSE, SmBoolean bDrawParams=TRUE, SmGfxArraySet *pOptGfxSet=NULL) const;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPointSet3d,SmObject,SmPointSet3d_TYPE);

}; // end class SmPointSet3d

/*******************************************************************//**
PURPOSE: Add a new Point to the end of the PointSet and
   expand the internal memory size of the PointSet as necessary.

NOTES: Note that
   this operation always causes the size to increase by one (GetSize).
***********************************************************************/
inline ULONG SmPointSet3d::Add
  (SmPoint3d sNewElement)
{
  ULONG nIndex = m_lSize;
  if (nIndex < m_lMaxSize)
    {
      m_pData[nIndex] = sNewElement;
      m_lSize++;
    }
  else
    {
      ULONG lErr = SetAtGrow(nIndex, sNewElement);
      if ( lErr >= SM_BIG_ULONG )
        { return SM_BIG_ULONG; }
    }

  // Centroid not set after adding a Point
  m_bCentroidSet = FALSE;

  return nIndex;

} // end SmPointSet3d::Add

/*******************************************************************//**
PURPOSE: A class to represent an ordered set of points with an implied
   linear connection between each pair of points.

NOTES: 
   The point set can be planar or not, and closed or not.  if appropriate
   it can find its normal (or that can be specified).  It can find its
   area if it's closed.  (If it's not planar, its normal and area can be
   ambiguous.)  It can report point containment (if closed).

   This can be used for 2d point sets, which could represent uv positions
   in the domain of a surface.

   The point array cannot be changed.  Make a new SmPointSequence object if desired.
***********************************************************************/
class SM_EXPORT SmPointSequence : public SmPointSet3d
{
private:
  // inherited from SmObject
  //   SmContext    * m_pContext ;   // access to 'global' memory
  //
  // inherited from SmTArray<SmPoint3d>
  //   SmBoolean      m_bIsBorrowed; // TRUE = m_pData is borrowed
  //   SmPoint3d    * m_pData;       // the ordered array of SmPoint3d
  //   ULONG          m_lSize;       // GetSize() of m_pData     (number of SmPoint3d spots used)
  //   ULONG          m_nMemorySize; // GetDataSize() of m_pData (Number of SmPoint3d spots allocated)
  // 
  // inherited from SmPointSet3d
  //   SmPointSetType m_ePointSetType ; 
  //   double         m_dDistTol3d;     
  //   
  //   SmPoint3d      m_sCentroid;       // PointSet geometric center
  //   SmVector3d     m_sXAxis;          // perp to Y and n. When linear = BestLine tangent     
  //   SmVector3d     m_sYAxis;          // perp to X and n.     
  //   SmVector3d     m_sNormal;         // perp to X and Y. When planar or scattered = BestPlane normal
  //                                     
  //   double         m_dMaxCentroidDev; // max dist between crPoints[i] and Centroid                        
  //   double         m_dMaxDev;         // max dist SM_PST_POINTSIZED: between crPoints[i] and Centroid
  //                                     //          SM_PST_LINEAR    : between crPoints[i] and BestLine
  //                                     //          SM_PST_PLANAR    : between crPoints[i] and BestPlane 
  //                                     //          SM_PST_SCATTERED : between crPoints[i] and BestPlane  
  //                                      
  //   SmExtent3d     m_sBBox;           // Bounding Box
  //   SmPseudoBox    m_sPBox;           // Pseudo Box
  // 

  SmBoolean m_bClosed;         // TRUE  = an implied linear connection between the 1st and last point
                               // FALSE = not
  SmBoolean m_bAreaSet;        // TRUE = m_dArea is calculated (Don't change the points),  FALSE = ready for lazy evaluation.
  double    m_dArea;           // Area enclsed by PointSequence, can be positive or negative.
                               //   If not planar, might not mean much.
                              
  SmBoolean m_bCoincidentSet ; // TRUE = m_bCoincident is calculated (Don't change the points),  FALSE = ready for lazy evaluation.
  SmBoolean m_bCoincident ;    // TRUE = Consecutive points in the sequence are coincident, FALSE = not

  SmBoolean m_bMonotonicSet ;  // TRUE = m_bMonotonicRow and m_bMonotonicCol calculat3ed, FALSE = ready for lazy evaluation.                              
  SmBoolean m_bMonotonic ;     // TRUE  = Every point increases monotonically
                               // FALSE = some points do not range monotonically

  // Private methods: helper methods
  SmStatus SetUpArea() ;
  SmStatus SetUpCoincident() ;
  SmStatus SetUpMonotonic() ;
  int CalcQuadNum( const SmPoint3d &crOrigin, const SmPoint3d &crTestPt );

public:
  // empty constructor
  SmPointSequence(ULONG        nMemorySize  = 0,        // in : initial m_pData array size or size of given pOptPtrArray
                  SmPoint3d * pOptPtrArray  = NULL,     // in : optional preallocated array to use for storage
                  ULONG        nArraySize   = 0) ;      // in : initial array size, less than or equal to nMemorySize.

  // constructor 
  SmPointSequence(SmTArray< SmPoint3d >  & sPoints,     // in : array copied or converted into PointSequence                                          
                  double       dTol3d    = SM_EFF_ZERO, // in : min dist between distinct points                                    
                  SmBoolean    bClosed   = TRUE,        // in : TRUE = implied linear connection between 1st and last point         
                  SmBoolean    bPlanar   = UNSURE,      // in : TRUE = PointSequence is known to lie on a plane                          
                  SmVector3d * pOptNormal= NULL,        // in : NULL=system computes m_sNormal, NotNULL=user given vector as Normal
                  SmBoolean    bCopyData = TRUE) ;      // in : TRUE=Copy sPoints into a new array - sPoints and this m_pData arrays are different
                                                        //      FALSE=Share sPoints with new array - sPoints and this m_pData arrays are same
                                                        //      default:[TRUE] 

  // Copy Constructor
  SmPointSequence ( const SmPointSequence &crOther ) ;      // in : object to copy

  // Assignment operator
  SmPointSequence & operator=( const SmPointSequence &crOther );

  // Equality operator
  SmBoolean operator==( const SmPointSequence &crOther );

  // clear the cached data - just in case a change was made to any of the contained points
  void ClearCache()                                         { SmPointSet3d::ClearCache() ;
                                                              m_bAreaSet = FALSE ;
                                                            }
  // simple access
  double     GetArea()        // rtn: positive for CCW and neg for CW sequences
                              { SetUpArea() ; return m_dArea ; }
  SmBoolean  GetCoincident()  { SetUpCoincident() ; return m_bCoincident ; }
  SmBoolean  GetMonotonic()   { SetUpMonotonic() ;  return m_bMonotonic ; }

  // inherited from SmPointSet simple access                                     
  //  SmTArray< SmPoint3d > & GetPoints() ;             
  //  SmZoneTol3d             GetTolerance() const ; 
  //  SmPointSetType          GetPointSetType() ;  
  //                                                   
  //  SmPoint3d               GetCentroid() ;           
  //  SmVector3d              GetXAxis() ;              
  //  SmVector3d              GetYAxis() ;              
  //  SmVector3d              GetZAxis() ;              
  //  SmVector3d              GetNormal() ;             
  //                                                   
  //  double                  GetMaxDev() ;             
  //  double                  GetMaxCentroidDev() ;
  //  ULONG                   GetDimension() ; // rtn: 2 = planar and parallel to x-y plane, else 3     
  //                                                   
  //  SmExtent3d              GetBBox() ;               
  //  SmPseudoBox             GetPBox() ;
  //  SmPseudoBox             GetTightPBox() ;               
                                           
  void SetClosed(SmBoolean bClosed)     { m_bClosed = bClosed ; }

  
  // predicates                                                            
  SmBoolean IsClosed()     const                            { return m_bClosed; }
  SmBoolean IsCounterClockwise(SmVector3d *pOptNormal=NULL) // rtn: TRUE = CounterClockwise with respect to pOptNormal or GetNormal()
                                                            { double dArea = GetArea() ; 
                                                              double dDotAreaNorm = dArea * (pOptNormal ? pOptNormal->Dot(m_sNormal) : 1.0) ; 
                                                              return(dDotAreaNorm > 0.0) ; 
                                                            }
  SmBoolean HasCoincidentPoints()                           { return( GetCoincident() ) ; }
  SmBoolean HasNonMonotonicPoints()                         { return( !GetMonotonic() ) ; }

  // inherited from SmPointSet Predicates
  //  SmBoolean IsEmpty() ;     
  //  SmBoolean IsPointSized() ;
  //  SmBoolean IsLinear() ;    
  //  SmBoolean IsPlanar() ;    
  //  SmBoolean IsScattered() ; 

  // Classify point as inside or outside this point set
  SmPointObjectContainmentType PointContainment( const SmPoint3d &crPoint );

  // Add Curve Graphics to new or open Display List
  SmDisplayList * Draw(SmBoolean bAddToUIPickList=FALSE, SmBoolean bDrawParams=FALSE, SmGfxArraySet *pOptGfxSet=NULL) const;
  SmDisplayList * DrawParams(SmBoolean bAddToUIPickList=FALSE, SmGfxArraySet *pOptGfxSet=NULL) const;   // Draw curve and increasing Param Points

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPointSequence,SmObject,SmPointSequence_TYPE);

}; // end class SmPointSequence

/*******************************************************************//**
PURPOSE: A class to represent an ordered array of points with implied
   linear connections between each point_ij and its four neighbor
   points_ij +/-1 .

NOTES: 
   The [i][j] grid of points are stored in a linear array and indexed
   as Index = i * RowCnt + j ;

   The point grid can be planar or not, and closed or not.  if appropriate
   it can find its normal (or that can be specified).  
   (If it's not planar, its normal and area can be ambiguous.)  
   It can detect when the point sequences double back on themselves

   The point array cannot be changed.  Make a new SmPointGrid object if desired.
***********************************************************************/
class SM_EXPORT SmPointGrid : public SmPointSet3d
{
private:
  // inherited from SmObject
  //   SmContext    * m_pContext ;   // access to 'global' memory
  //
  // inherited from SmTArray<SmPoint3d>
  //   SmBoolean      m_bIsBorrowed; // TRUE = m_pData is borrowed
  //   SmPoint3d    * m_pData;       // the ordered array of SmPoint3d
  //   ULONG          m_lSize;       // GetSize() of m_pData     (number of SmPoint3d spots used)
  //   ULONG          m_nMemorySize; // GetDataSize() of m_pData (Number of SmPoint3d spots allocated)
  // 
  // inherited from SmPointSet3d
  //   SmPointSetType m_ePointSetType ; 
  //   double         m_dDistTol3d;     
  //   
  //   SmPoint3d      m_sCentroid;       // PointSet geometric center
  //   SmVector3d     m_sXAxis;          // perp to Y and n. When linear = BestLine tangent     
  //   SmVector3d     m_sYAxis;          // perp to X and n.     
  //   SmVector3d     m_sNormal;         // perp to X and Y. When planar or scattered = BestPlane normal
  //                                     
  //   double         m_dMaxCentroidDev; // max dist between crPoints[i] and Centroid                        
  //   double         m_dMaxDev;         // max dist SM_PST_POINTSIZED: between crPoints[i] and Centroid
  //                                     //          SM_PST_LINEAR    : between crPoints[i] and BestLine
  //                                     //          SM_PST_PLANAR    : between crPoints[i] and BestPlane 
  //                                     //          SM_PST_SCATTERED : between crPoints[i] and BestPlane  
  //                                      
  //   SmExtent3d     m_sBBox;           // Bounding Box
  //   SmPseudoBox    m_sPBox;           // Pseudo Box
  // 

  ULONG     m_lRowSize ;     // number of points in each row of the grid
  SmBoolean m_bClosedCol ;   // TRUE  = an implied linear connection between the 1st and last U point in each Column
  SmBoolean m_bClosedRow ;   // TRUE  = an implied linear connection between the 1st and last V point in each Row
                             // FALSE = not

  SmBoolean m_bCoincidentSet ; // TRUE = m_bCoincidentCol and m_bCoincidentRow are calculated,  FALSE = ready for lazy evaluation.
  SmBoolean m_bCoincidentCol ; // TRUE = Consecutive points in a Col are coincident, FALSE = not
  SmBoolean m_bCoincidentRow ; // TRUE = Consecutive points in a Row are coincident, FALSE = not

  SmBoolean m_bMonotonicSet ;  // TRUE = m_bMonotonicRow and m_bMonotonicCol calculat3ed, FALSE = ready for lazy evaluation.                              
  SmBoolean m_bMonotonicCol ;  // TRUE  = Every point in in every Col (Varying i, Constant j) increases monotonically
                               // FALSE = some cols have points that do not range monotonically
  SmBoolean m_bMonotonicRow ;  // TRUE  = Every point in in every Row (Constant i, Varying j) increases monotonically
                               // FALSE = some rows have points that do not range monotonically
                                                  
  // Private methods: helper methods
  SmStatus SetUpCoincident() ;
  SmStatus SetUpMonotonic() ;

public:
  // empty constructor
  SmPointGrid(ULONG        nMemorySize  = 0,        // in : initial m_pData array size or size of given pOptPtrArray
              SmPoint3d * pOptPtrArray  = NULL,     // in : optional preallocated array to use for storage
              ULONG        nArraySize   = 0) ;      // in : initial array size, less than or equal to nMemorySize.

  // constructor 
  SmPointGrid(SmTArray< SmPoint3d >   & sPoints,     // in : array copied or converted into PointGrid                                         
              ULONG                     lRowSize,    // in : number of points in each row of the grid
              double       dTol3d     = SM_EFF_ZERO, // in : min dist between distinct points                                    
              SmBoolean    bClosedCol = FALSE,       // in : TRUE = 1st and last U point in each Column are connected        
              SmBoolean    bClosedRow = FALSE,       // in : TRUE = 1st and last V point in each Row are connected        
              SmBoolean    bPlanar    = UNSURE,      // in : TRUE = PointSequence is known to lie on a plane                          
              SmVector3d * pOptNormal = NULL,        // in : NULL=system computes m_sNormal, NotNULL=user given vector as Normal
              SmBoolean    bCopyData  = TRUE) ;      // in : TRUE=Copy sPoints into a new array - sPoints and this m_pData arrays are different
                                                     //      FALSE=Share sPoints with new array - sPoints and this m_pData arrays are same
                                                     //      default:[TRUE] 

  // Copy Constructor
  SmPointGrid ( const SmPointGrid &crOther ) ;      // in : object to copy

  // Assignment operator
  SmPointGrid & operator=( const SmPointGrid &crOther );

  // Equality operator
  SmBoolean operator==( const SmPointGrid &crOther );

  // clear the cached data - just in case a change was made to any of the contained points
  void ClearCache()                                         { SmPointSet3d::ClearCache() ;
                                                              m_bMonotonicSet = FALSE ;
                                                            }
  // simple access
  ULONG                       RowIndx(ULONG i)          const { return( i * m_lRowSize ) ; }
  ULONG                       Indx   (ULONG i, ULONG j) const { return( i * m_lRowSize + j ) ; }
  ULONG                       GetColCnt()               const { return m_lRowSize ; }
  ULONG                       GetRowCnt()               const { return m_lSize / m_lRowSize ; }
  SmBoolean                   GetCoincidentCol()              { SetUpCoincident() ; return m_bCoincidentCol ; }
  SmBoolean                   GetCoincidentRow()              { SetUpCoincident() ; return m_bCoincidentRow ; }
  SmBoolean                   GetMonotonicCol()               { SetUpMonotonic() ; return m_bMonotonicCol ; }
  SmBoolean                   GetMonotonicRow()               { SetUpMonotonic() ; return m_bMonotonicRow ; }

  // inherited from SmPointSet simple access                                     
  //  SmTArray< SmPoint3d > & GetPoints() ;             
  //  SmZoneTol3d             GetTolerance() const ; 
  //  SmPointSetType          GetPointSetType() ;  
  //                                                   
  //  SmPoint3d               GetCentroid() ;           
  //  SmVector3d              GetXAxis() ;              
  //  SmVector3d              GetYAxis() ;              
  //  SmVector3d              GetZAxis() ;              
  //  SmVector3d              GetNormal() ;             
  //                                                   
  //  double                  GetMaxDev() ;             
  //  double                  GetMaxCentroidDev() ;
  //  ULONG                   GetDimension() ; // rtn: 2 = planar and parallel to x-y plane, else 3     
  //                                                   
  //  SmExtent3d              GetBBox() ;               
  //  SmPseudoBox             GetPBox() ;
  //  SmPseudoBox             GetTightPBox() ;               
                                           
  void SetClosedCol(SmBoolean bClosedCol)     { m_bClosedCol = bClosedCol ; }
  void SetClosedRow(SmBoolean bClosedRow)     { m_bClosedRow = bClosedRow ; }

  
  // predicates              
  SmBoolean IsGoodRowSize()       const           { return (m_lSize == ((m_lSize / m_lRowSize) * m_lRowSize)) ; }                                               
  SmBoolean IsClosedCol()         const           { return m_bClosedCol; }
  SmBoolean IsClosedRow()         const           { return m_bClosedRow; }
  SmBoolean HasCoincidentPoints()                 { return( GetCoincidentCol() && GetCoincidentRow()) ; }
  SmBoolean HasNonMonotonicPoints()               { return( !(GetMonotonicCol() && GetMonotonicRow())) ; }

  // inherited from SmPointSet Predicates
  //  SmBoolean IsEmpty() ;     
  //  SmBoolean IsPointSized() ;
  //  SmBoolean IsLinear() ;    
  //  SmBoolean IsPlanar() ;    
  //  SmBoolean IsScattered() ; 

  // Add Curve Graphics to new or open Display List
  SmDisplayList * Draw(SmBoolean bAddToUIPickList=FALSE, SmBoolean bDrawParams=FALSE, SmGfxArraySet *pOptGfxSet=NULL) const;
  SmDisplayList * DrawParams(SmBoolean bAddToUIPickList=FALSE, SmGfxArraySet *pOptGfxSet=NULL) const;   // Draw curve and increasing Param Points

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPointGrid,SmObject,SmPointGrid_TYPE);

}; // end class SmPointGrid


#endif  // !__SMPOINTSET_H__
