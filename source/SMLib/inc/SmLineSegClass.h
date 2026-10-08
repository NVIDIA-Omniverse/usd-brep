// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmLineSegClass.h
* PURPOSE: Header file for LineSeg classification object.
**********************************************************************/

#ifndef __SMLINESEGCLASS_H__
#define __SMLINESEGCLASS_H__

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMTARRAY_H_
#include <SmTArray.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMTOLERANCE_H__
#include <smTol.h>
#endif

/*******************************************************************//**
PURPOSE: This enum defines where a point (parameter) on a LineSeg
   falls relative to an existing SmLineSegClassification and its subordinate 
   intervals.

NOTES: 
***********************************************************************/
enum SmPolyIntervalPosition 
{
  SM_PIP_START,      // Falls on start point of interval
  SM_PIP_INSIDE,     // Falls between start pont and end point of interval
  SM_PIP_END,        // Falls on end point of interval
  SM_PIP_OUTSIDE     // Falls outside of this interval
} ;

/*******************************************************************//**
PURPOSE: This enum defines the classification of a point - what does
    the point lie on.

NOTES: 
***********************************************************************/
#define SmPolyPointClassType SmPolyPointClassificationType

enum SmPolyPointClassificationType 
{
  SM_PPC_UNKNOWN, // Point classification is currently unknown
  SM_PPC_REGION,  // Point lines in a Region (SmPolyRegion)
  SM_PPC_FACE,    // Point lies on or near a poly face (SmPolyFace)
  SM_PPC_EDGE,    // Point lies on or near a poly edge (SmPolyEdge)
  SM_PPC_VERTEX,  // Point lies on or near a poly vertex (SmPolyVertex)
  SM_PPC_POINT    // Point lies on or near a geometric point (SmPoint3d)
} ; 

/*******************************************************************//**
PURPOSE: This object represents the classification of a point on either
    a geometric or topological object.  This object captures the type of
    object on which a point lies, a pointer to the object, parameters of
    the object, and deviation from the object.

NOTES: 
***********************************************************************/
#define SmPolyPointClass SmPolyPointClassification

class SM_EXPORT SmPolyPointClassification 
{
  friend SmLineSegClassification ; 
  friend SmTol ;

protected:
  SmPolyPointClassificationType   m_ePolyPointClass ;  // Type of classification for point
  SmObject                      * m_pObject ;          // Object on which the point lies or is near
  double                          m_dDeviation ;       // Distance from object to the point
  double                          m_dTParam ;
  SmZoneTol3d                     m_sSrcZoneTol3d ;    // When classifying a point against a PolyBrep - copy of Point's ZoneTol3d value
                                                       // When classifying a ray   against a PolyBrep - copy of Ray's   ZoneTol3d value
                                                       // When classifying a curve against a PolyBrep - copy of Curve's ZoneTol3d value
public:
  SmPolyPointClassification() : m_ePolyPointClass(SM_PPC_UNKNOWN),
                                m_pObject(NULL),
                                m_dDeviation(0.0),
                                m_dTParam(0.0),
                                m_sSrcZoneTol3d(0.0) 
                              { }

  ~SmPolyPointClassification() { }
  void ReSet()                 { m_ePolyPointClass = SM_PPC_UNKNOWN ;
                                 m_pObject        = NULL ;
                                 m_dDeviation     = 0.0 ;
                                 m_dTParam        = 0.0 ; 
                                 m_sSrcZoneTol3d  = 0.0 ; 
                               }

  // simple data access
  double                        GetDeviation()  const { return m_dDeviation ; }
  SmObject                    * GetObject()     const { return m_pObject ; }
  double                        GetTParam()     const { return m_dTParam ; }
  SmPolyPointClassificationType GetPointClass() const {return m_ePolyPointClass ; }

  void SetDeviation  (double dDeviation) { m_dDeviation = dDeviation ; }
  void SetTParam     (double dTParam)    { m_dTParam = dTParam ; }
  void SetClassObject(SmPolyPointClassificationType ePolyPointClass, SmObject *pObject)
    {
      m_ePolyPointClass = ePolyPointClass ; 
      m_pObject = pObject; 
      if ( pObject == NULL )
        { m_ePolyPointClass = SM_PPC_UNKNOWN; }   // Should be safer this way.
    }    
                     
  SM_OLDTOL_LINE SmZoneTol3d GetTolerance() const        { return(SmTol::GetObjZoneTol3d(this)) ; } 
private:
  SmZoneTol3d GetSrcZoneTol3d() const                    { SM_ASSERT_TOL(m_sSrcZoneTol3d) ; return m_sSrcZoneTol3d ; }
  void        SetSrcZoneTol3d(SmZoneTol3d sSrcZoneTol3d) { m_sSrcZoneTol3d = sSrcZoneTol3d ; }
public:

  SmPolyPointClassification Combine(const SmPolyPointClassification & crPointClass) const ;

  SmStatus Merge(const SmPoint3d       & sVertexPoint, 
                 double                  dTolerance,
                 SmTArray<SmPolyEdge*> * pOptNewEdges) ;

  // The following two methods have been removed.
  // Use SetClassObject() instead, to avoid type/object conflicts.
  //void SetObject(SmObject *pObject) { m_pObject = pObject; }
  //void SetPointClass(SmPolyPointClassificationType ePointClass) { m_ePolyPolyPointClass = ePolyPointClass; }

  void Draw() const ;
  void Dump() const ;

} ; // end class SmPolyPointClassification
    
/*******************************************************************//**
PURPOSE: This object represents the classification of a single 
    homogeneous interval along the LineSeg.  It is primarily an object
    used by the SmLineSegClassification.

NOTES: This class cannot have virtual methods since SmLineSegClassification
       manages an Array of SmLineSegIntervals using smos_Calloc() and smos_MemSet().
***********************************************************************/
class SM_EXPORT SmLineSegInterval
{
 friend SmTol ;

 public:
  SmLineSegClassification  * m_pLineSegClassification = NULL;
  SmExtent1d                 m_vInterval; // Parametric interval on curve
  SmPolyPointClassification  m_vStart;    // Point classification of start point of interval 
  SmPolyPointClassification  m_vMid;      // Point classification of interior of interval
  SmPolyPointClassification  m_vEnd;      // Point classificaiton of end point of interval

  SmLineSegInterval()                              { }
  SmLineSegInterval(const SmExtent1d & crInterval) : m_vInterval(crInterval)
                                                   { }
  void          ReSet()                            { m_vInterval.Init() ; 
                                                     m_vStart.ReSet() ;  
                                                     m_vMid.ReSet() ;    
                                                     m_vEnd.ReSet() ;    
                                                   }

  SmLineSegClassification * GetLineSegClassification() const { return m_pLineSegClassification ; }
  const SmExtent1d        & GetInterval()              const { return m_vInterval ; }

  void          Dump()          const ;
  const TCHAR * GetTypeString() const              { return _T("SmLineSegInterval") ; }

} ; // end class SmLineSegInterval

/*******************************************************************//**
PURPOSE: This object represents the classification of a LineSeg relative
    to either a geometric or topological object.   The classification of 
    a LineSeg is represented by a list of intervals on the LineSeg.  Each 
    interval being a homogeneous segment of the LineSeg relative to the
    object(s) being classified against.  

NOTES: Note that the current implementation may be somewhat
    inefficient for large numbers of LineSeg intervals.
***********************************************************************/
#define SmLineSegClass SmLineSegClassification

class SM_EXPORT SmLineSegClassification : public SmObject
{
  friend class SmPolyIntersector;
  friend class SmTol ;

protected:
  // LineSeg being classified
  const SmPoint3d     m_cLineStart;   
  const SmVector3d    m_cLineVec;      // Vector from start to end
  SmZoneTol3d         m_sSrcZoneTol3d; // Tolerance of LineSeg being classified
                                       // internal to this object.
  SmBoolean           m_bIsBorrowed;   // TRUE = data array borrowed, FALSE = data is internal
                                       //   destructor deletes m_pData when m_bIsBorrowed==FALSE
  SmLineSegInterval * m_pData;         // the actual array of data 
  ULONG               m_lSize;         // # of elements (upperBound - 1) 
  ULONG               m_lMaxSize;      // max allocated

  void SetAtGrow(ULONG nIndex, const SmLineSegInterval & crNewSolution);

public:
  // Constructor
  SmLineSegClassification(const SmPoint3d   & crLineStart,
                          const SmVector3d  & crLineVec,
                          SmZoneTol3d         sSrcZoneTol3d,
                          ULONG               nDataSize,    
                          SmLineSegInterval * pOptBorrowedData) ;
    
  // destructor
 ~SmLineSegClassification() ;
 
  // Attributes
  ULONG       GetSize()         const                 { return m_lSize; }

  SM_OLDTOL_LINE SmZoneTol3d GetTolerance() const     { return SmTol::GetSrcZoneTol3d(this) ; }
private:
  SmZoneTol3d GetSrcZoneTol3d() const                 { SM_ASSERT_TOL(m_sSrcZoneTol3d) ; return m_sSrcZoneTol3d; }
  void        SetTolerance(SmZoneTol3d sSrcZoneTol3d) { m_sSrcZoneTol3d = sSrcZoneTol3d; }
public:

  // Accessing elements
  SmLineSegInterval   GetAt      (ULONG nIndex) const { SM_ASSERT(nIndex < m_lSize); return m_pData[nIndex]; }
  SmLineSegInterval & operator[] (ULONG lIndex)       { SM_ASSERT(lIndex < m_lSize); return m_pData[lIndex]; }
  SmLineSegInterval & operator[] (ULONG lIndex) const { SM_ASSERT(lIndex < m_lSize); return m_pData[lIndex]; }

  // Editing 
  void InsertAt(ULONG nIndex, const SmLineSegInterval & crNewInterval);
  void RemoveAt(ULONG nIndex);
  void SetSize (ULONG nNewSize);

  // Topological operations
  SmBoolean AreCoincidentMaybe(const SmLineSegClassification & crOther,
                               ULONG                           lIntervalIndex,
                               SmPolyIntervalPosition        & rePositionOfCoincidentMaybe) const;

  SmStatus CleanupAndValidate(SmLineSegClassification & rOther);

  SmStatus CheckIntervalCoincidence( SmBoolean * pbWasModified=NULL );

  SmStatus ClassifyFaceInterval(ULONG              lIntervalIndex,
                                SmBoolean          bDoPointClassify,
                                const SmPolyFace * pFaceToClassify,
                                const SmPolyFace * pOldFace);

  SmStatus FindInterval(double                   dParameter, 
                        double                   dLastParameter,
                        SmZoneTol3d              sParamObjZoneTol3d,  // in : Obj assoc with dParameter's ZoneTol3d
                        ULONG                  & rlIndex, 
                        double                 & rdEndDist3d,
                        SmPolyIntervalPosition & rePosition) const;

  void GetLineSeg(SmPoint3d  & rLineStart,
                  SmVector3d & rLineVec) const { rLineStart = m_cLineStart; 
                                                 rLineVec = m_cLineVec; 
                                               }

  SmPolyIntervalPosition FindBestSqueezeEnd(ULONG lIndex) const;

  SmStatus Homogenize(SmLineSegClassification & rOther);
  SmStatus FixProblemsBySqueezing(SmLineSegClassification & rCurveClass,
                                  SmLineSegClassification & rCurveClassOther) ;

  SmStatus InsertLineSegInterval(const SmLineSegInterval & crLineSegInterval);

  SmStatus InsertPointClass(const SmPolyPointClassification & crPointClass,
                            double                            dParameter,
                            SmBoolean                         bForceInsertion,
                            SmBoolean                       & rbInsertionMade);

  SmStatus MergeClassifications(SmLineSegClassification & rOther,
                                SmTArray<SmPolyFace*>   * pOptNewFaces=NULL,
                                SmTArray<SmPolyFace*>   * pOptNewFacesOther=NULL,
                                SmTArray<SmPolyEdge*>   * pOptNewEdges=NULL,
                                SmTArray<SmPolyEdge*>   * pOptNewEdgesOther=NULL);

  SmStatus MergeInterval(ULONG                   lIntervalIndex,
                         SmTArray<SmPolyFace*> * pOptNewFaces=NULL,
                         SmTArray<SmPolyEdge*> * pOptNewEdges=NULL);

  SmStatus MergePointClass(ULONG                   lPointClassIndex,
                           SmTArray<SmPolyEdge*> * pOptNewEdges=NULL);

  SmPolyPointClassification * GetPointClassByIndex(ULONG    lPointClassIndex,
                                                   double * pOptParameter=NULL) const;

  SmPolyPointClassification * GetPointClassMateByIndex(ULONG lPointClassIndx) const;

  SmStatus ConnectIntervals(ULONG                  lIntervalIndex,
                            SmPolyIntervalPosition eEndToSqueeze,
                            SmBoolean              bAllowRemovalOfEnds);

//    SmStatus TestForPartialCoincidentEdges(SmLineSegClassification & rOther,
//                                           double & rdNewTolerance,
//                                           SmBoolean & rbCreateNewLine,
//                                           const SmPoint3d & crLinePnt,
//                                           const SmVector3d & crLineVec);

//    SM_COMMON(SmLineSegClassification,SmObject,SmLineSegClassification_TYPE);
  void Draw(const SmLineSegInterval & crLineSegIvl) const;
  void Draw() const;
  void Dump() const;

} ; // end class SmLineSegClassification

#endif // !__SMLINESEGCLASS_H__



