// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCurveClass.h
* PURPOSE: Header file for curve classification object.
*
* Contains
*   enum  SmIntervalPosition    - relate a point to an interval start, end, inside, outside
*   class SmCurveInterval       - the classification of a single homogeneous curve interval
*   class SmCurveClassification - the classification of a curve against another object, it 
*                                 represents all the homogeneous intervals of the curve
**********************************************************************/

#ifndef __SMCURVECLASSIFICATION_H__
#define __SMCURVECLASSIFICATION_H__

#ifndef __SMPOINTCLASSIFICATION_H__
#include <SmPointClass.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMTARRAY_H_
#include <SmTArray.h>
#endif

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

// forward declarations
class SmTopologyIntersector ;
class SmFaceProps ;            // currently only for optional Debug blocks
class SmClassifyLoopIO ;

/*******************************************************************//**
PURPOSE: This enum allows classification functions to report
the cause of any possibleProblems during a classification based
on geometric tests.  Calling functions can implement various recovery
strategies for various problems to increase the reliability of the 
classifications.

NOTES:
***********************************************************************/
enum SmClassifyProblem {
  SM_CP_NONE,                     // classification has no problems and can be trusted
  SM_CP_VERTEX_ON_SINGULARITY,     // Vertex being classified against marks a surface singularity - one bounding surface edge is compressed to a point
  SM_CP_TOGGLED_CLASSIFICATION,   // sequence of increasing step off distances changed classification once (probably ok) 
  SM_CP_SHORT_SECTOR_EDGEUSE1,    // sector's first edgeuse length shorter than attempted classify stepoff distance 
  SM_CP_SHORT_SECTOR_EDGEUSE2,    // sector's radial mate's edgeuse length shorter than attempted classify stepoff distance 
  SM_CP_SHORT_INTERVAL,           // Curve interval shorter than attempted classify stepoff distance
  SM_CP_SEESAW_CLASSIFICATION,    // sequence of increasing step off distances changed classification more than once (probably confused) 
  SM_CP_UNRESOLVED_CLASSIFICATION // iteration max reached before definitive classification criteria were met

} ; // end enum SmClassifyProblem

/*******************************************************************//**
PURPOSE: This object describes a ClassifyCrv's arrival or Departure direction  
         relationship to a Face at a SmPointClassification marking
         a ClassifyCrv/FaceBndry XSect Point.

NOTES: 
  1. Every ClassifyCrv SmPointClassification has an arriving and a departing side.
     The arriving and departing directions are defined as:
     Arriving  = ClassifyCrv direction heading into a ClassifyCrv/VertexOrEdge XSect
     Departing = ClassifyCrv direction heading out from a ClassifyCrv/VertexOrEdge XSect

  2. A single Touch = a Pair of TouchDirs as:
      Touch :== {ArriveTouchData  DepartTouchData}

  3. TouchDirLists are just SmTArray<SmTouchData> organized as
      [ArriveTouchDir0 DepartTouchDir0 ArriveTouchDir1 DepartTouchDir2 ...  DepartTouchDirN]

  4. TouchLists and Seams: The touchlist for a classifyCrv that starts and stops on a seam is handled in two ways:
       raw touch list:  The Seam Start/Stop touches are initially treated as a pair of touches one at diff ends of the param ivl
                        [ 0-Arr] Seam   (Param:[0.00000000]  <== note: param val says touch is at ivl beg
                           -Dep] Out/In (Param:[0.00000000]
                        [ n-Arr] In/Out (Param:[1.00000000]  <== note: param val says touch is at ivl end
                           -Dep] Seam   (Param:[1.00000000]
       done touch list: The Seam Start/Stop touches are compressed into a single seam-spanning touch arriving and departing on diff Ivl ends
                        [ 0-Arr] In/Out (Param:[1.00000000]  <== note: param val says touch arrives at inv end
                           -Dep] Out/In (Param:[0.00000000]  <==       param val says touch departs at inv beg

  5. Given a TouchDirList
      A ClassifyCrv's TouchCnt = TouchDirList.GetSize() / 2 ; (yup - that's going to cause a bug or two)

  6. This class cannot have virtual methods since SmCurveClassification
     manages an Array of SmCurveIntervals using smos_MemSet().
***********************************************************************/
class SM_EXPORT SmTouchData
{
 public:
  SmTopology            * m_pConnectedTo ;     // ptr to topology connected to PtClassify
  SmCrvDirOnFaceType      m_eLoopDir ;         // oneof: SM_CD_UNINIT SM_CD_IN   SM_CD_POLE    SM_CD_POLE_NO_VERTEX
                                               //                     SM_CD_ON   SM_CD_SEAM  
                                               //                     SM_CD_OUT  SM_CD_LAMINA
  ULONG                   m_lLoopIndx ;        // associated Loops Index for this m_pConnectedTo obj
  double                  m_dParam ;           // associated ClassifyCrv param            for this TouchDir
  ULONG                   m_lPtClassifyIndx ;  // associated ClassifyCrv PtClassify Index for this TouchDir
  SmPointClassification * m_pPtClassify ;      // associated ClassifyCrv PtClassify       for this TouchDir

  // maintenance
  SmBoolean operator==(const SmTouchData &crOther) const ;

  // Combine several SmTouchDataLists into one orderd by m_dParam values
  static void Homogenize(SmTArray< SmTArray<SmTouchData> > & rInLists, SmTArray<SmTouchData> & rOutList) ;

  // count and extract all the touches for a given ConnectedTo object, rtn number of ConnectedTouches
  //             note:  lCrossingCnt = GetConnectedTouches() / 2 ;
  static ULONG GetConnectedTouches(SmTopology * pConnectedTo, SmTArray<SmTouchData> & rInTouchList, SmTArray<SmTouchData> * pOptOutTouchList=NULL) ;
  static ULONG GetWindingCount(SmTArray<SmTouchData> & rTouchList,     // in : single TgtLoop TouchList to review, ordered:[m_dParam]
                               SmOrientType          & reOrientType) ; // out: oneof: SM_OT_UPPERDOMAIN = TouchList has unmatched Out/In Touches
                                                                       //             SM_OT_LOWERDOMAIN = TouchList has unmatched In/Out Touches
                                                                       //             SM_OT_UNKNOWN     = TouchList has no unmatched Out/In, InOut Touches

  // pretty print one line for an arriving or departing TouchDir obj
  void Dump(SmBoolean bNoDetails=TRUE) const ;

  // pretty print an entire touch list 
  static void DumpTouchList(const     TCHAR             * pLabel,            // in : string included in output header
                            const SmTArray<SmTouchData> & rTouchList,        // in : Touch list as :[ArriveTouchDir_0 DepartTouchDir_0, ArriveTouchDir_1 ...]
                                  SmBoolean               bNoDetails=TRUE) ; // in : TRUE =output only a string of eLoopDir values on one line
                                                                             //      FALSE=output one eLoopDir with all its associated data values per line
                                                                             //      default:[TRUE]
} ; // end class SmTouchData 

SM_TARRAY_TEMPLATE_PREDECLARATION(SmTouchData) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmTouchData*) ;

/*******************************************************************//**
PURPOSE: This object represents the classification of a single 
    homogeneous interval along the curve.  It is primarily an object
    used by the SmCurveClassification.

NOTES: This class cannot have virtual methods since SmCurveClassification
       manages an Array of SmCurveIntervals using smos_MemSet().
***********************************************************************/
class SM_EXPORT SmCurveInterval
{
  friend class SmCurveClassification ; // allows SmCurveClassification access to SetIntervalMin() and SetIntervalMax()
  friend class SmTol ;

public:
  SmCurveClassification * m_pCurveClassification ;  // Owner Object containing Curve being partitioned into intervals
  SmExtent1d              m_vInterval ;             // Parametric interval on curve
                                                    // note: sometimes the param values in m_vInterval have
                                                    //       been snapped.  When this is true then
                                                    //         m_vStart.m_dPreSnapParam != m_vInterval.GetMin() or
                                                    //         m_vEnd.m_dPreSnapParam != m_vInterval.GetMax()
  SmPointClassification   m_vStart ;                // Point classification of start point of interval 
  SmPointClassification   m_vMid ;                  // Point classification of interior of interval
  SmPointClassification   m_vEnd ;                  // Point classificaiton of end point of interval
                         
  // constructors
  SmCurveInterval(const SmContext             * cpContext = NULL,
                  const SmCurveClassification * cpOwner = NULL)
                 : m_pCurveClassification( SM_CONST_CAST( SmCurveClassification*, cpOwner ) ),
                   m_vInterval(),
                   m_vStart( SmTol::GetSrcZoneTol3d( cpOwner ), cpContext ),
                   m_vMid( SmTol::GetSrcZoneTol3d( cpOwner ), cpContext ),
                   m_vEnd( SmTol::GetSrcZoneTol3d( cpOwner ), cpContext )
                 { }

  SmCurveInterval(const SmExtent1d            & crInterval,
                  const SmContext             * cpContext,
                  const SmCurveClassification * cpOwner = NULL)
                 : m_pCurveClassification( SM_CONST_CAST( SmCurveClassification*, cpOwner ) ),
                   m_vInterval( crInterval ),
                   m_vStart( SmTol::GetSrcZoneTol3d( cpOwner ), cpContext ),
                   m_vMid( SmTol::GetSrcZoneTol3d( cpOwner ), cpContext ),
                   m_vEnd( SmTol::GetSrcZoneTol3d( cpOwner ), cpContext )
                 { }
                                                   
  // ReSet all member values to init values        
  void ReSet()   { m_vInterval.Init() ;  // m_cpContext value left alone by design
                   m_vStart.ReSet() ;
                   m_vMid.ReSet() ;
                   m_vEnd.ReSet() ;
                 }

  void SetInterval (const SmExtent1d & crInterval)
                   { m_vInterval = crInterval ; }


  // asssignment operator
  SmCurveInterval & operator=( const SmCurveInterval &crOther )
                             { if(this == &crOther) { return(*this) ; }
                               m_pCurveClassification = crOther.m_pCurveClassification;
                               m_vInterval = crOther.m_vInterval;
                               m_vStart = crOther.m_vStart;
                               m_vMid = crOther.m_vMid;
                               m_vEnd = crOther.m_vEnd;
                               return(*this) ;
                             }


private:
  // note: These two methods are private to force the use of methods
  //       SmCurveClassification::EditIntervalMin() and EditIntervalMax() 
  //       which coordinate the interval values within the sequence of intervals stored within SmCurveClassification.
  SmStatus SetIntervalMin(double dNewParam) { return(m_vInterval.SetMinMax(dNewParam, m_vInterval.GetMax())) ; }
  SmStatus SetIntervalMax(double dNewParam) { return(m_vInterval.SetMinMax(m_vInterval.GetMin(), dNewParam)) ; }

public:

  // data access
  SmCurveClassification * GetCurveClassification() const { return m_pCurveClassification ; }
  const SmCurve         * GetCurve()               const ;
  const SmExtent1d      & GetInterval()            const { return m_vInterval ; }
  double                  GetMin()                 const { return m_vInterval.GetMin() ; }
  double                  GetMax()                 const { return m_vInterval.GetMax() ; }
  SmStatus                GetMinUV(SmPoint2d &rUV) const ; // note: rtns SM_ERR when ClassifyCrv is NULL, uses DropPt when dim != 2
  SmStatus                GetMaxUV(SmPoint2d &rUV) const ; // note: rtns SM_ERR when ClassifyCrv is NULL, uses DropPt when dim != 2

  // return EdgeObj->Edgeuse bounding specified IvlEnd when ClassifyCrv is on Face and IvlEndPt classifies to EDGE, else rtn NULL
  SmEdgeuse * GetIvlBound_EdgeuseObj(SmBoolean     bDoMin,             // in : TRUE = Find Edgeuse Bounding m_vStart side of ClassifyIvl
                                                                     //      FALSE= Find Edgeuse Bounding m_vEnd side of ClassifyIvl
                                   SmFaceProps * pOptFaceProps=NULL) // in : For Debug only, NULL to ignore, default:[NULL]
                                  const ;

  // sets                                              
  void SetCurveClassifictaion(SmCurveClassification *pCrvClassif)
                             { m_pCurveClassification = pCrvClassif ; }

  // Return 3d chord distance between an Ivl's end points.
  SmStatus Length3d( double & rdLength ) const ;

  SmBrep * GetBrep(SmBrep *pSkipBrep=NULL) const ;

  // distance m_vInterval min was snapped or SM_BIG_DOUBLE when not snapped at all
  double GetStartSnap() const
                     { return(m_vStart.GetPreSnapParam() != SM_BIG_DOUBLE
                                                           ? smos_Fabs( m_vInterval.GetMin() - m_vStart.GetPreSnapParam() )
                                                           : SM_BIG_DOUBLE) ;
                     }
  
  // distance m_vInterval max was snapped or SM_BIG_DOUBLE when not snapped at all
  double GetEndSnap()   const
                   { return(m_vEnd.GetPreSnapParam() != SM_BIG_DOUBLE
                                                        ? smos_Fabs( m_vInterval.GetMax() - m_vEnd.GetPreSnapParam() )
                                                        : SM_BIG_DOUBLE) ;
                   }

  // for debug purposes
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                         
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                     
                                                                          //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                 
                                SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't  
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order               
                               const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON_ONLY_BASE(SmCurveInterval, SmCurveInterval_TYPE) ;

  // maintenance
  void Dump(ULONG            lCurveDim,               // in :
            const SmPoint3d *pStartPoint=NULL,        // in :
            const SmPoint3d *pEndPoint  =NULL,        // in :
            const SmSurface *pSurface1  =NULL,        // in :
            const SmSurface *pSurface2  =NULL)        // in :
           const;

  SmDisplayList * Draw(void) const;

} ; // end class SmCurveInterval

/*******************************************************************//**
PURPOSE: This object represents the classification of a curve relative
  to either a geometric or topological object.   The curve is broken 
  up into a set of elements and boundary points where it intersects 
  boundaries of the other object(s) creating a sequence of 
  homogeneous segments relative to the object(s) being classified against.

  This classification of the curve is stored as a list of 
  SmCurveInterval objects.  Each SmCurveInterval stores the interval's 
  parameter range and an SmPointClassification object for the interval's 
  end, start, and mid points.

  Each SmPointClassification represents the classification of a point
  against the geometry or topology object.  The SmPointClassification 
  stores a type classification (oneof REGION, FACE EDGE VERTEX, POINT, 
  CURVE, SURFACE, EDGEUSE, UNKNOWN), the object on which the point lies
  on or near, and tolerance and object parameter data.

  Each interval's inside region inherits its classification from the 
  information stored in the mid point SmPointClassification object.

NOTES: 
  The list of intervals spans the entire interval range of the
  curve and is stored in ascending order in m_pData so that

    m_pData[0].GetMin()         == m_vInterval.GetMin(),  (the curve's minimum parameter value)
    m_pData[i-1].GetMax()       == m_pData[i].GetMin()
    m_pData[m_lSize-1].GetMax() == m_vInterval.GetMax(),  (the curve's maximum parameter value)

  Note that the current implementation may be somewhat
  inefficient for large numbers of curve intervals because
  each interior interval boundary parameter is stored twice, not once.
***********************************************************************/
#define SmCurveClass SmCurveClassification
class SM_EXPORT SmCurveClassification : public SmObject
{
  friend class SmTol ;

protected:
  SmBoolean         m_bIsCurveOwned;        //          TRUE   = m_cpCurve is deleted by this destructor
                                            // default:[FALSE] = isn't
  const SmCurve   * m_cpCurve = NULL;       // Curve being classified, default:[NULL]
                                            //  when 3d             : works in 3d space, with 3d ZoneTol3d val
                                            //  when 2d with        : works in 2d Space, with 3d ZoneTol3d val 
                                            //    cpSurface1               (Which CurveClassification maps to 2d as needed)
                                            //  when 2d without     : works in 2d Space, with 2d ZoneTol3d val = SmTol::GetEffZeroParam
                                            //    cpSurface1    
                                            // Owned:[see m_bIsCurveOwned]

  SmZoneTol3d       m_sSrcZoneTol3d ;       // ZoneTol3d of m_cpCurve being classified (Its usually a SmTol::GetZoneTol3d(SrcObject) value)
                                            // +----------------------+----------------------------+-----------------------------------------+
                                            // |When ClassifyCurve is | Being Classified Against   | ZoneTol3d                               |
                                            // +----------------------+----------------------------+-----------------------------------------+
                                            // |  IntersectionCurve3d | Its intersection Faces     | smos_Max(SurfZoneTol3d1, SurfZoneTol3d2)|
                                            // |                      |                            |    (Set m_cpSurface1 and 2)             |
                                            // |  UVDropCurve         | Its DropFace or DropSurface| ZoneTol3d(DropEdgeOrDropCurve)          |
                                            // |                      |                            |    (Set m_cpSurface1)                   |
                                            // |  WireEdge->Curve     | A Brep                     | ZoneTol3d(WireEdge)                     |
                                            // |  Ray                 | A Brep                     | ZoneTol3d(User Selected Val)            |
                                            // |  UVCurve             | The Surf's UVDomain to get | SmTol::GetEffZeroParam()                |
                                            // |                      | UVTrimCurve/Knot crossings |                                         |
                                            // +----------------------+----------------------------+-----------------------------------------+
                                            // uninit:[SM_UNINIT_TOL]   (as returned by empty contstructor)
                                            // default:[SM_ZONE_TOL_3D] (as returned by ReSet() )

  const SmCurve * m_cpUVCurve1 = NULL;      // Corresponding Surface1 UV curve. NULL for analytic surfaces. default:[NULL]. NotOwned
  SmBoolean       m_bIsCurveCurrent = true; // default:[TRUE] =3DCurve and UVTrimCurves are current
                                            // FALSE=3DCurve has been edited and UVTrimCurves are obsolete 
                                            //  only set to FALSE by CHANGE_SSS code     
  const SmSurface * m_cpSurface1;           // opt: 1. When Curve is 3d and from a surf/surf XSect - ptr to 1st Surface
                                            //      2. surface for UVCurve, must be present when m_cpCurve->Dim is 2d
                                            //      default:[NULL]. NotOwned
  const SmSurface * m_cpSurface2;           // opt: 1. When Curve is 3d and from a surf/surf XSect - ptr to 2nd Surfacee
                                            //      default:[NULL]. NotOwned

                                                    // note: m_bDoingCrossHatch is being used to change how tolerances
                                                    //       are computed in IntersectTSBound().  This is a hack, in general
                                                    //       different problems (crosshatch, boolean, fillet, ...) will use 
                                                    //       tolerances differently.  For now: Crosshatch - use a specified 2d tol
                                                    //                                         Others     - use current Tol Model
                                                    // This flag and the general concept that different problems might use the same
                                                    // calculation tools with different ways for computing tolerances will
                                                    // be encapsulated in a 'smart tolerance' object. That object will
                                                    // be passed as argument values through the different calculation tools.
                                                    // The tools will ask that object what tolerance to use for the next computation.
                                                    // When the 'smart tolerance' object is built and used I expect the m_bDoingCrossHatch
                                                    // flag to become obsolete.  Maybe the m_bDoingFillets flag will be encapsulated
                                                    // in the 'smart tolerance' object as well.

  SmBoolean               m_bDoingCrossHatch;       //          TRUE  = m_cpCurve is a CrossHatch line calssifying against its Face, use special tol Rules 
                                                    // default:[FALSE]= default, use normal tolerance rules
  SmBoolean               m_bDoingFillets;          //          TRUE  = Use Fillet Package rules when Merging Coincident geometry into OriginalBreps
                                                    // default:[FALSE]= default, use normal Merge rules for Coincident geometry (for Booleans and all NonFillet packages)
  SmBoolean               m_bOKToSetTolerance ;     // default:[TRUE] = Adjust existing VertexTols during Classification to match XSect Results (Previous Behavior)
                                                    //          FALSE = Don't adjust existing Vertex Tolerances during Classification - can be adjusted during Merge
  SmBoolean               m_bTgtLoopsOnly ;         //          TRUE  = Only classify against Loops in m_sTgtLoops
                                                    // default:[FALSE]= classify against all Face->Loops
  SmTArray<SmLoop*>       m_sTgtLoops ;             // When m_bTgtLoopsOnly == TRUE
                                                    // default:[Empty]   = Classify against the Face's OuterLoop
                                                    //          NotEmpty = The only Face->Loops to Classify against
  SmCurveClassification * m_cpOtherClassification ; // opt: the paired classification when a single curve is classified against two Breps.
                                                    //      newly added, initially for use by fillet package but could be useful for Booleans. 
                                                    //      default:[NULL]
// GWC:CHANGE_SSS
  const SmFace    * m_cpFace ;                      // opt: when classification is against the face, the face being classified, default:[NULL]
  const SmCurve   * m_cpUVCurve2 = NULL;            // opt: Corresponding Surface2 UV curve (NULL for anlaytic surfaces), default:[NULL]
  const SmTopologyIntersector * m_cpTI = NULL;      // opt: TopologyIntersector managing a sequence
                                                    //      of surf/surf intersections for a boolean operator
                                                    //      Used to find faces attached to edges. 
                                                    //      During a boolean operator, edges can attach to faces
                                                    //      in two different Breps.  The m_cpTI object keeps
                                                    //      track of all those connections.  When m_cpTI == NULL
                                                    //      an edge is only connected to faces in the same Brep.
                                                    //      default:[NULL]
// end GWC:CHANGE_SSS

#ifdef SM_USE_NEWTOL
    SM_NEWTOL_LINE double m_dMaxGap3d ;       // MaxGap between m_cpCurve and m_pSurface1(m_cpUVCurve1) and m_pSurface2(m_cpUVCurve2)
#endif // SM_USE_NEWTOL

  SmExtent1d        m_vInterval;            // Total Interval of curve being classified, default:[0, 1]
  SmBoolean         m_bIsBorrowed;          //          TRUE   = data array borrowed, 
                                            // default:[FALSE] = data is internal
                                            //   destructor deletes m_pData when m_bIsBorrowed==FALSE
  SmCurveInterval * m_pData = NULL;         // array of ascending intervals spanning the entire curve domain.
                                            // sized:[m_lSize], default m_lSize:[1]
                                            //   uninit:[NULL]              (as returned by empty contstructor)
                                            //  default:[NotNULL,m_lSize=1] (as returned by ReSet() )
                                            //   For each segment store: 
                                            //     1. parameter range for each interval
                                            //     2. classification for each end_pt 
                                            //     3. classification for segment (mid_pt)
                                            //   m_pData[0].GetMin()         == m_vInterval.GetMin(),  (the curve's minimum parameter value)
                                            //   m_pData[i-1].GetMax()       == m_pData[i].GetMin()
                                            //   m_pData[m_lSize-1].GetMax() == m_vInterval.GetMax(),  (the curve's maximum parameter value)
                                            // Always has at least one Interval.  [1st ivl.begin, Last ivl.end] = m_vInterval
                                            // (SmPointClassification contained in SmCurveInterval is a dynamic class: don't use smos_Calloc etc.)
                                            
  ULONG             m_lSize = 0;            // number of sub-intervals stored in m_pData. default:[1]
  ULONG             m_lMaxSize;             // max allocated
  SmBoolean         m_bIgnoreTangency;      // TRUE = attempt to remove coincident-maybe cases which are the result 
                                            //        of tangencies by ignoring (not inserting) point or interval 
                                            //        classifications due to ClassificationCurve/Edge intersections 
                                            //        where the ClassificationCurve and Edge are nearly parallel 
                                            //        to one another.
                                            // Default:[FALSE] = no special detection or treatment of coincident-maybe cases.
public:
  // empty default constructor - use ReSet(args) to initialize
  SmCurveClassification() ;

  // copy constructor
  SmCurveClassification(const SmCurveClassification &crOther) ;

  // constructor - with all args
  SmCurveClassification(const SmCurve    * cpCurve,                   // in : either a 2d or a 3d curve                                                 
                        const SmExtent1d & crInterval,                // in : interval for cpCurve                                                      
                        const SmCurve    * cpUVCurve,                 // in : assoc UVTrimCurve (on Surface1 for intersection cases)  
                                                                      //    : NotOwned:[Not deleted by destructor] 
                        SmZoneTol3d        sSrcZoneTol3d,             // in : ZoneTol3d for m_cpCurve source: When m_cpCurve dim is                     
                                                                      //    : 3d                          : commonly a Curve or Edge ZoneTol3d          
                                                                      //    : TrimUVCurve with cpSurface1 : commonly a Surface or Face ZoneTol3d        
                                                                      //    : UVCurve without  cpSurface1 : 2d wire - SmTol::GetZoneTol3d(UVCurve)      
                        ULONG              lOptDataSize     = 0,      // in : SizeOf OptUserManaged m_pData array, default:[0]                          
                        SmCurveInterval  * pOptBorrowedData = NULL,   // in : OptUserManaged m_pData array, default:[NULL]                              
                        SmBoolean          bIsCurveOwned    = FALSE,  // in : TRUE=cpCurve is deleted by this destructor, FALSE=isn't, default:[FALSE]  
                        SmBoolean          bIsCurveCurrent  = TRUE,   // in : TRUE = Edge not modified since UVTrimCurve was built, default:[TRUE]      
                        const SmSurface  * cpSurface1       = NULL,   // in : Surf of cpUVCurve, or Surf1 of surf/surf XSection that made classify curve,
                                                                      //    : default:[NULL], NotOwned:[Not deleted by destructor] 
                        const SmSurface  * cpSurface2       = NULL,   // in : Surf2 of surf/surf XSection that made classify curve, default:[NULL]       
                                                                      //    : NotOwned:[Not deleted by destructor] 
                        const SmCurve    * cpUVCurve2       = NULL,   // in : UVCurve for Surface2, default:[NULL]                                       
                                                                      //    : NotOwned:[Not deleted by destructor] 
                        const SmTopologyIntersector * cpTI  = NULL) ; // in : notNULL = edges may connect through mates to faces in 2 different Breps    
                                                                      //    : NULL    = edges only connect to faces in 1 Brep                            

  // assignment operator
  SmCurveClassification & operator= (const SmCurveClassification &crOther ) ;

  // destructor
  virtual ~SmCurveClassification() ;

  // Simple Data Access
  ULONG                         GetSize()                         const { return m_lSize ; } // IvlCnt (EndPtCnt = IvlCnt + 1)
  SmBrep                      * GetBrep() ;
  SmBrep                      * GetBrep(SmBrep *pSkipBrep = NULL) const ;
  const SmFace                * GetFace()                         const { return m_cpFace ; }
  const SmSurface             * GetSurface1()                     const { return m_cpSurface1 ; }
  const SmSurface             * GetSurface2()                     const { return m_cpSurface2 ; }
  const SmCurve               * GetCurve()                        const { return m_cpCurve; }
  SmExtent1d                    GetInterval()                     const { return m_vInterval; }
  SmExtent1d                    GetSubInterval(ULONG lIndex)      const { SM_ASSERT_MSG(lIndex <= m_lSize, _T("Array Bounds Alert")) ;
                                                                          return (m_pData[lIndex].m_vInterval) ;
                                                                        }
  const SmCurve               * GetUVCurve1()                     const { return m_bIsCurveCurrent ? m_cpUVCurve1 : NULL ; }
  const SmCurve               * GetUVCurve2()                     const { return m_bIsCurveCurrent ? m_cpUVCurve2 : NULL ; }
  const SmCurve               * GetAnyUVCurve1()                  const { return m_cpUVCurve1 ; }
  const SmCurve               * GetAnyUVCurve2()                  const { return m_cpUVCurve2 ; }
  const SmTopologyIntersector * GetTopologyIntersector()          const { return m_cpTI ; }

  SmBoolean                     GetIsCurveCurrent()               const { return m_bIsCurveCurrent ; }
  SmBoolean                     GetIgnoreTangency()               const { return m_bIgnoreTangency ; }
  SmBoolean                     GetDoingCrossHatch()              const { return m_bDoingCrossHatch ; }
  SmBoolean                     GetDoingFillets()                 const { return m_bDoingFillets ; }
  SmCurveClassification       * GetOtherClassification()          const { return m_cpOtherClassification ; }
  SmBoolean                     GetOKToSetTolerance()             const { return m_bOKToSetTolerance ; }

  // TgtLoops Management   
  SmBoolean                     GetTgtLoopsOnly()                 const  { return m_bTgtLoopsOnly ; }
  void                          SetTgtLoopsOnly(SmBoolean bTgtLoopsOnly) { m_bTgtLoopsOnly    = bTgtLoopsOnly ; }
  const SmTArray<SmLoop*>     & GetTgtLoops    ()                 const  { return m_sTgtLoops ; }
  void                          AddTgtLoop     (SmLoop * pTgtLoop)       { m_sTgtLoops.Add(pTgtLoop) ;  }
  void                          ReSetTgtLoops  ()                        { m_sTgtLoops.ReSet() ; }

  // GetZoneTol3d() to become obsolete - use SmTol::GetSrcZoneTol3d(pCurveClassification)
  SmZoneTol3d                   GetZoneTol3d()                    const  { return SmTol::GetSrcZoneTol3d(this) ; } 
private: // for SmTol::Static methods only
  SmZoneTol3d                   GetSrcZoneTol3d()                 const  { SM_ASSERT_TOL(m_sSrcZoneTol3d) ; return m_sSrcZoneTol3d ; }  
  void                          SetSrcZoneTol3d(SmZoneTol3d sZoneTol3d)  { m_sSrcZoneTol3d = sZoneTol3d ; } 
public:
#ifdef SM_USE_NEWTOL     
  SM_NEWTOL_LINE double         GetMaxGap3d()                     const { return m_dMaxGap3d ; }
  SM_NEWTOL_LINE void           SetMaxGap3d(double dMaxGap3d)           { m_dMaxGap3d = dMaxGap3d ; }
#endif // SM_USE_NEWTOL

  // CurveInterval Access
  SmCurveInterval & GetAt(ULONG lIndex) const        { SM_ASSERT(lIndex  < m_lSize) ; return m_pData[lIndex]; }
  SmCurveInterval & GetLast() const                  { SM_ASSERT(m_lSize > 0) ;       return m_pData[m_lSize - 1] ; }
  SmCurveInterval & operator[] (ULONG lIndex)        { SM_ASSERT(lIndex  < m_lSize) ; return m_pData[lIndex]; }
  SmCurveInterval & operator[] (ULONG lIndex) const  { SM_ASSERT(lIndex  < m_lSize) ; return m_pData[lIndex]; }

  // Access by IvlBndry Indx
  SmPointClassification       * GetPointClassByIndex    (ULONG lPointClassIndx, double * pOptdCrvParam=NULL) const ;
  SmPointClassification       * GetPointClassMateByIndex(ULONG lPointClassIndx) const;
  SmZoneTol3d                   GetPointObjZoneTol3d    (ULONG lPointClassIndex) const ;
  // TRUE when any point object is a topology obj (vertex/edge/face) no longer live in cpBrep (dangling).
  SmBoolean                     HasDeadTopologyPointObject(const SmBrep * cpBrep) const ;
  SmPoint3d                     GetClassifyCrvLoc       (ULONG lPointClassIndex, double * pOptdCrvParam=NULL) const ;
  SmPoint3d                     GetClassifyCrvLoc       (const SmPointClassification *pPointClass, 
                                                        double dCurveParameter) const ;

  // List SegmentEndPt PtClassifications that connect to a TgtOwner: Ex: Get PtClassifies connected to a Loop, an Edge, a Vertex...
  SmStatus GetPointClassifiesConnectedTo(const SmTArray<SmTopology*> & crTgtOwners, // in : NotEmpty = Only get Edge and Vertex PtClassifications connected to pTgtObject, 
                                                                                    //    : Empty    = get all Edge and Vertex PtClassifications                           
                                         SmTArray<SmTouchData>       & rTouchData)  // out: For each SegmentEndPt PtClassification that passes the filter: 
                                        const ;                                     //    : stores:[SmTouchData::m_pPtClassify             ]             
                                                                                    //    :        [SmTouchData::m_lPtClassifyIndx         ]             
                                                                                    //    :        [SmTouchData::m_dParam                  ]             
                                                                                    //    :        [SmTouchData::m_eLoopDir == SM_CD_UNINIT]             
                                                                                    //    :        [SmTouchData::m_lLoopIndx               ]             
                                                                                    //    :        [SmTouchData::m_pConnectedTo            ]             
                                         
  ULONG GetPointClassifies (SmTArray<SmPointClassification*> & rPtClassifies,
                            SmTArray<double>                 & rClassifyCurveParam,
                            SmTopology                       * pOptConnectedTarget=NULL)
                           const ;
        
  ULONG FindVertexObjects  (SmTArray<SmVertex *> & rVertices,               // out:
                            const SmTopology     * pOptOwner=NULL,          // in :
                            SmTArray<double>     * pOptClassifyParams=NULL) // in :
                           const ; 
  ULONG FindEdgeObjects    (SmTArray<SmEdge *> & rEdges,                    // out:
                            const SmTopology   * pOptOwner=NULL,            // in :
                            ULONG                lWorkFlag=3)               // in : lWorkFlag = orof 1(CoincidentEdges) & 2(XSectingEdges)
                           const ;

  // count times ClassifyCrv crosses Loop in UVSpace: a valid loop is crossed an even number of times
  SmStatus GetLoopCrossings(const SmTArray<SmLoop*> & crLoops,              // in : TgtLoops
                            SmFaceProps             * pOptFaceProps,        // in : optional FaceProps, if already set by SmFaceProps::SetProps_Stage2() - saves time
                                                                            //    :                   else calls SmFaceProps::SetProps_Stage2().
                            SmTArray<ULONG>         * pCrossingCounts,      // out: One crossing count for each TgtLoop
                            SmTArray<SmTouchData>   * pOptRawTouchDirList,  // out: Optional list of raw ClassifyCrv/Loop touches described by arrive/depart pair values as [arrive/depart ...]
                                                                            //      Arrive/Depart eLoopDir vals oneof:[SM_CD_ IN ON OUT POLE SEAM LAMINA], SEAM entries may be duplicated
                            SmTArray<SmTouchData>   * pOptDoneTouchDirList) // out: Optional list of processed ClassifyCrv/Loop touches from which the LoopTouchCnt is computed
                                                                            //      Arrive/Depart eLoopDir vals oneof:[SM_CD_ IN ON OUT], Single SEAM entry - cyclic
                           const ;

   // Find the Interval containing parameter
  SmStatus FindInterval(double               dParameter,           // in : target parameter value                                                                 
                        double               dLastParameter,       // in : previous call Param to prevent double classifications, SM_BIG_DOUBLE to ignore         
                        SmZoneTol3d          sObjZoneTol3d,        // in : SmTol::GetZoneTol3d(pAssocObj), always 3d Tol even when SrcCurve->Dim == 2             
                        ULONG              & rlIndex,              // out: containing interval index                                                              
                        double             & rdEndDist3d,          // out: for SM_IP_START and SM_IP_END pts, dist3d (or dist2d) along m_cpCurve to ivl end point 
                        SmIntervalPosition & rePosition)           // out: oneof SM_IP_START, SM_IP_INSIDE, SM_IP_END, SM_IP_OUTSIDE dParameter classification    
                       const;    

  // Find Start/Mid/End PointClassification containing parameter
  SmStatus FindPointClass(double                  dParameter,          // in : m_cpCurve parameter to classify                                                               
                          SM_TOL_LINE SmZoneTol3d sParamObjZoneTol3d,  // in : for 3dCurve: ZoneTol3d of object associated with dParameter. For 2dCurve: SM_EFF_ZERO_PARAM   
                          SmPointClassification & rPointClass)         // out: Start, Mid, or End PointClassification within XSectTol3d of dParameter Point.                 
                         const;  
                                                                       
  // predicates
  SmBoolean IsCrossedByEdge() const ;                                // rtn: TRUE == the edge of any Edge/ClassifyCrv XSect crosses the ClassifyCrv (does not count EdgeEnd and TangentEdge XSects)
                                                                     //      GWC:TODO needs more special case Edge/ClassifyCrv tangent XSects handling - see notes in method
  SmBoolean IsCrossingFace (const SmFace * pOptTgtFace=NULL) const ; // rtn: TRUE == any IvlMid classifies to a Face 
                                                                     // in : NULL = count any Face Crossing, NotNULL = only count any pOptTgtFace Face Crossings
                                                                     //      note: Face Crossing ClassifyCrvs may split Face later when MergeInterval() adds edges to Face.                                     
  SmBoolean HasNearMisses  (SmBoolean bInSurfaceGaps=FALSE) const ;  // rtn: TRUE = any Pt or CoincidentEdge Classifies NearMiss-XSect (not exact) the ClassifyCrv.
                                                                     //      FALSE= all Pt and CoincidentEdge Classifies Exactly-XSect the ClassifyCrv.
                                                                     // in : TRUE = Only test GapComponent perp to m_pSurface1->SurfNorm at the connection
                                                                     //      FALSE= Test full gap size 
                                                                     //      default:[FALSE], when m_pSurface1==NULL full gaps sizes are checked

  void SetSurface1 (const SmSurface *cpSurface1) { m_cpSurface1 = cpSurface1 ; }                                                   
  void SetSurface2 (const SmSurface *cpSurface2) { m_cpSurface2 = cpSurface2 ; }   
  void SetUVCurve1 (const SmCurve   *cpUVCurve1, SmBoolean bDeleteOldUVCurve=FALSE) ;
  void SetUVCurve2 (const SmCurve   *cpUVCurve2, SmBoolean bDeleteOldUVCurve=FALSE) ;

  void DeleteUVCurve1() { SetUVCurve1( NULL, TRUE ) ; } 
  void DeleteUVCurve2() { SetUVCurve2( NULL, TRUE ) ; } 

  void SetFace               (const SmFace          * cpFace)               { m_cpFace            = cpFace ; }   
  void SetInterval           (const SmExtent1d      & vIvl)                 { m_vInterval.SetMinMax( vIvl.GetMin(), vIvl.GetMax() ) ; }
  void SetIgnoreTangency     (SmBoolean               bIgnoreTangency)      { m_bIgnoreTangency   = bIgnoreTangency; }
  void SetIsCurveOwned       (SmBoolean               bIsCurveOwned)        { m_bIsCurveOwned     = bIsCurveOwned; }
  void SetDoingCrossHatch    (SmBoolean               bDoingCrossHatch)     { m_bDoingCrossHatch  = bDoingCrossHatch ; }
  void SetDoingFillets       (SmBoolean               bDoingFillets)        { m_bDoingFillets     = bDoingFillets ; } 
  void SetOtherClassification(SmCurveClassification * pOtherClassification) { m_cpOtherClassification = pOtherClassification ; }
  void SetCurve              (const SmCurve         * cpCurve,                       
                              SmBoolean               bIsCurveOwned=FALSE) ;
  void SetIsCurveCurrent     (SmBoolean               bCurrent) ;           
  void SetOKToSetTolerance   (SmBoolean               bOKToSetTolerance)    { m_bOKToSetTolerance = bOKToSetTolerance ; }
  void SetIntersectData      (const SmSurface * cpSurface1,               // in : Surf1 of surf/surf intersection that produced curve to classify          
                              const SmSurface * cpSurface2,               // in : Surf2 of surf/surf intersection that produced curve to classify          
                              const SmCurve   * cpUVCurve2,               // in : UVCurve for Surface2                                                     
                              SmBoolean         bIsCurveCurrent=TRUE,     // in : TRUE = Edge not modified since UVTrimCurve was built                     
                              const SmTopologyIntersector *cpTI=NULL) ;   // in : notNULL = edges may connect through mates to faces in 2 different Breps  
                                                                          //    : NULL    = edges only connect to faces in 1 Brep                          
                                                     
  // Set DoingFillet data - OtherClassification is for the Fillet Brep to move Fillet Brep geometry to the Target Brep                                                   
  void SetFilletData(SmCurveClassification *pOtherClassification,
                     SmBoolean              bDoingFillets = TRUE )
                    { m_cpOtherClassification = pOtherClassification;
                      m_bDoingFillets         = bDoingFillets;
                    }

  // Editing 
  void InsertAt(ULONG nIndex, const SmCurveInterval & crNewInterval) ;
  void RemoveAt(ULONG nIndex) ;
  void SetSize (ULONG nNewSize) ;

  void ReSet(const SmCurve    * cpOptCurve    = NULL,        // in : when cpOptCurve && cpOptUVCurve are NULL - keep current curve, default:[NULL]   
             const SmExtent1d * cpOptInterval = NULL,        // in : interval of the curve,        default:[NULL]                                    
             const SmCurve    * cpOptUVCurve  = NULL,        // in : when cpOptCurve && cpOptUVCurve are NULL - keep current curve,                  
                                                             //    : default:[NULL], NotOwned                                                        
             SmZoneTol3d        sZoneTol3d = SM_ZONE_TOL_3D, // in : ZoneTol3d for m_cpCurve source: When m_cpCurve dim is                           
                                                             //    : 3d                          : commonly a Curve or Edge ZoneTol3d                
                                                             //    : TrimUVCurve with cpSurface1 : commonly a Surface or Face ZoneTol3d              
                                                             //    : UVCurve without  cpSurface1 : 2d wire - SmTol::GetZoneTol3d(UVCurve)            
             ULONG              nDataSize        = 0,        // in :  size of OptBorrowedData array, default:[0]                                     
             SmCurveInterval  * pOptBorrowedData = NULL,     // in : opt memory that can be used by callers that want to manage memory               
             SmBoolean          bIsCurveOwned    = FALSE,    // in : TRUE = Curve is deleted by this destructor, FALSE = isn't, default:[FAULT]      
             SmBoolean          bIsCurveCurrent  = TRUE,     // in : TRUE = Edge not modified since UVTrimCurve was built, default:[TRUE]            
             const SmSurface  * cpSurface1       = NULL,     // in : Surf of cpUVCurve, default:[NULL], NotOwned                                     
             const SmSurface  * cpSurface2       = NULL,     // in : Surf2 of surf/surf XSection that made classify curve, default:[NULL], NotOwned  
             const SmCurve    * cpUVCurve2       = NULL,     // in : UVCurve for Surface2, default:[NULL], NotOwned                                  
             const SmTopologyIntersector *cpTI   = NULL) ;   // in : notNULL = edges may connect through mates to faces in 2 different Breps    
                                                             //    : NULL    = edges only connect to faces in 1 Brep                            

  SmBoolean IsUnInit() const ;                                    // rtn: TRUE when all members have their uninit values

  // Set object parameter values along the curve in the PointClasses.
  SmStatus SetPointClassParameters( SmBoolean bSetMids=FALSE ) ;

  // return TRUE when pObject is referenced by one of the Classification interval->PointClassifications
  SmBoolean IsReferenced(const SmObject * pObject) const ;

  // connect two neighbor intervals to make one - delete the PtClassification data at the original interface
  SmStatus ConnectIntervals(ULONG              lIntervalIndex,        // in : interval to be removed (thought of as squeezed to zero length)
                            SmIntervalPosition eEndToSqueeze,         // in : interval endPoint to be removed                               
                            SmBoolean          bAllowRemovalOfEnds) ; // in : TRUE = OK to delete end interval and shorten m_vInterval      
                                                                      //    : FALSE= only merge end_intervals with their neighbors          
                                                                      
  // GWC TODO: cbiTol: tols should be consistent in:
  // - IntersectVertices()
  // - IntersectEdges()
  // - InsertPointClass()
  // - sm_InsertSolutions()
  // - MergeInterval()

  SmStatus InsertPointClass(const SmPointClassification & crPointClass,            // in : Point to insert or merge                                                   
                            double                        dCurveParameter,         // in : associated parameter value                                                 
                            double                        dLastCurveParameter,     // in : When inserting a sequence of pointClass objects                            
                                                                                   //    :is used to prevent two objects from both being                              
                                                                                   //    :classified to one interval endPoint.                                        
                                                                                   //    :Set to SM_BIG_DOUBLE to ignore.                                             
                            SmBoolean                   & rbInsertionMade,         // out: TRUE = inserted (IvlCnt+=1), FALSE=merged i.e. combined (IvlCnt=const)     
                            SmBoolean                     bForceInsertion=FALSE) ; // in : TRUE = force point insertion even when                                     
                                                                                   //    :        point is close to existing classification points                    
                                                                                   //    : FALSE = use internal heuristics to decide when to merge and when to insert 

  SmStatus InsertCurveInterval(const SmCurveInterval & crCurveInterval) ; 

  SmStatus RemovePointClass(ULONG              lIndex,    // in : index of interval to modify
                            SmIntervalPosition eIPos      // in : one of SM_IP_START or SM_IP_END
                          ) ;   

  SmStatus UpdatePointClass(ULONG                         lPointClassIndex,    // in : interval boundary index (not interval index)
                                                                               //    : range:[0<=lPointClassificationIndex<=SmCurveClassification::m_lSize]
                            const SmPointClassification & rNewValues) ;        // in : target pointClassification object to copy

  // Called by InsertPointClass() to prevent the creation of very short intervals
  // change an interval extent parameter value and update the neighbor intervals to be consistent.
  // When an interval is forced to zero or neg length by the edit, merge interval endPt classifications
  // and remove interval from m_pData array.
  SmStatus EditIntervalMin(ULONG lIndex, double dNewParam, SmBoolean bCollapseDegenIntervals = TRUE) ;
  SmStatus EditIntervalMax(ULONG lIndex, double dNewParam, SmBoolean bCollapseDegenIntervals = TRUE) ;

  // called by EditIntervalMin() and EditIntervalMax()
  // Remove a zero length interval from m_pData and clean up neighbor data.
  // Choose the best EndPt Classification to persist in the modified SmCurveClassification.
  SmStatus CollapseInterval(ULONG lIntervalIndex) ;
  
  // insert all curve/naturalBounds xSects into classification
  SmStatus IntersectNaturalBounds  (const SmSurface & crSurface) ;
  SmStatus IntersectNaturalUVBounds(const SmSurface & crSurface) ;

  // insert all UVCurve/SurfaceDiscontinuity xSects into classification (when m_cpCurve is 2d)
  SmStatus IntersectUVDiscontinuties(const SmSurface & crSurface, SmContext const & crContext) ;       

  // insert all 3DCurve/SurfaceDiscontinuity xSects into classification (when m_cpCurve is 3d)
  SmStatus Intersect3DDiscontinuties(const SmSurface & crSurface, SmContext const & crContext) ;       

  // insert all curve/VolumeDiscontinuity xSects into classification
  SmStatus Intersect3DDiscontinuities(const SmVolume  & crVolume,       // in : Target Volume to classify curve against       
                                      SmBoolean         bInParamSpace,  // in : TRUE  = m_pCurve is in Volume's ParamSpace    
                                                                        //    : FALSE = m_pCurve is in Volume's InSpace       
                                      SmContext const & crContext) ;    // in : Context for temporary object construction     

  // Return curve intervals from pair of CurveClassifications that classify to objects in both classifications
  SmStatus ExtractGeometry(const SmContext       & crContext,              // in : context for constructing new geometry            
                           SmCurveClassification & rOther,                 // in : other target CurveClassification                 
                           SmTArray<SmCurve*>    & r3DCurves,              // out: a curve for every common interval classified to topology in both input CrvClassifications  
                           SmTArray<SmCurve*>    * pOptUVCurves1 = NULL,   // out: optional associated UVCurves in pCrvClass1, NULL to ignore                                
                           SmTArray<SmCurve*>    * pOptUVCurves2 = NULL,   // out: optional associated UVCurves in pCrvClass2, NULL to ignore                                
                           SmTArray<SmPoint3d>   * pOpt3DPoints = NULL) ;  // out: optional singular points = neither adjacent curve has been found but it has a good classification, NULL to ignore                                

  // insert PointClassifications so that two CurveClassifications of one curve have common intervals.
  // Applies squeeze hueristics to edit the homogonized classification to remove common problems.
  SmStatus Homogenize(SmCurveClassification & rOther) ;

  // called indirectly by Homogenize() to merge a short interval with a longer neighbor
  SmIntervalPosition FindBestSqueezeEnd(ULONG lIndex) ;

  // Find and remove all CoincidentMaybe intervals
  SmStatus CleanupAndValidate(SmCurveClassification & rOther) ;

  // called by CleanupAndValidate() to test for short curves mislabeled as tangent to an edge due to tolerances but which are just Pt intersections.
  // rtn: TRUE=found a fixable Coincident-Maybe case. FALSE = Not Coincident-Maybe or not fixable. UNSURE=BadClassification.
  SmBoolean AreCoincidentMaybe(const SmCurveClassification & crOther,                     // in : homogenized partner to this CurveClassification to check two classifications at at time. 
                                                                                          //    : note: let crOther == this to check just one CurveClassification                          
                               ULONG                         lIntervalIndex,              // in : target interval                                                                          
                               SmIntervalPosition          & rePositionOfCoincidentMaybe, // out: interval end to remove when TRUE is returned.                                            
                                                                                          //    : one of SM_IP_START or SM_IP_END,                                                         
                               SmBoolean                     bReportProb=TRUE)            // in : TRUE = Report a message when CoinMaybeCase is found FALSE= don't                         
                              const ;     

  // a place to add huerisic modifications to fix common classification problems.
  // only called by SmTopologyIntersector::MergeCurveClasses() as part of the boolean operation.
  //  1. snap to nearby EdgeEndPoints.
  //  2. merge short intervals with long neighbors.
  //  3. if MidPoint classifies to an edge - make sure interval endPoints classify to edge or edge->Vertex
  // This is called before Homogenize().
  SmStatus FixupClassification() ;

  // Check for intervals entirely coincident to an Edge.
  SmStatus CheckIntervalCoincidence(SmCurveClassification & rOther, SmBoolean &rbWasModified) ;

  // called by FixupClassification() to snap end-points to nearby vertices and to merge short intervals with long neighbors.
  SmStatus FixEndPointClass(ULONG             lPointClassIndex,     // in : PointClassification to examine                                                 
                            const SmPoint3d & crPoint,              // in : rCurveClassBrep point location for lPointClassIndex                            
                            SmBoolean       & rbModified) ;         // out: TRUE = a short interval squeezed out of rCurveClassBrep and rCurveClassOther   
                                                                    //    : FALSE= no change to interval count in rCurveClassBrep and rCurveClassOther     

  // returns TRUE when entire interval classifies to the same edge
  SmBoolean IsCoincident(const SmExtent1d & rInterval,     // in : Interval from Edge being tested        
                         SmZoneTol3d        sObjZonetol3d) // in : Set to SmTol::GetZoneTol3d(pEdge)      
                        const ;

  SmStatus InsertTopologyIntersections(SmSolutionArray & rSolutions,             // in : an array of intersection solutions                                                     
                                       ULONG             lSolutionObjectIndex) ; // in : The index of the the CurveClassification object within the SmSolution objects.         
                                                                                 //    : Typically this is 0.  However to improve performance in SmFace::IsDegenerate() when    
                                                                                 //    : working with curve/curve coincidence we allow this number to be 1.                     

  SmStatus IntersectVertices(const SmTArray<SmVertex*> & crVertices,     // in : list of vertices to intersect with m_cpCurve                                                        
                             const SmSurface           & crSurface,      // in : Srf of Face being classified against m_cpCurve; Face connected to crVertices.                       
                                                                         //    : only used when bHasBigGaps == TRUE                                                                  
                             SmBoolean                   bHasBigGaps,         // in : TRUE = m_cpCurve and crVertices connect to Face = crSurface.GetOwner(),                             
                                                                         //    : where Face/Vertex gap are suspected being bigger than XSectTol3d.                                   
                                                                         //    : increase XSectTol3d to accomodate large Vertex/Face gaps                                            
                                                                         //    : FALSE= Expect all Face/Edge gaps to be within XSectTol3d, use standard SmTol::GetXSectTol3d tols.   
                             SmBoolean                   bUVSpaceOkay=TRUE) ; // in : TRUE = okay to use and create uv trim curves 
                                                                              //      FALSE= don't try to create UVTrimCurves because they are not known to be valid, default:[TRUE]
                             
  SmStatus IntersectEdges(const SmTArray<SmEdge*> & crEdges,          // in : list of edges to intersect with m_cpCurve                                                        
                          const SmFace            & crFace,           // in : Face being classified against m_cpCurve; Face connected to crEdges.                              
                                                                      //    : only used when bHasBigGaps == TRUE                                                               
                          SmBoolean                 bHasBigGaps,          // in : TRUE = m_cpCurve and crEdges connect to Face = crSurface.GetOwner(),                             
                                                                      //    : increase XSectTol3d to accomodate large Edge/Face gaps                                           
                                                                      //    : FALSE= Expect all Face/Edge gaps to be within XSectTol3d, use standard SmTol::GetXSectTol3d tols.
                          SmBoolean                 bUVSpaceOkay=TRUE) ;  // in : TRUE = okay to use and create uv trim curves 
                                                                          //      FALSE= don't try to create UVTrimCurves because they are not known to be valid, default:[TRUE]
                        
  // insert all 2d classificationCurve/face->vertex and face->edge intersections - increments an unlocked mark
  SmStatus IntersectTSBound(const SmFace *cpFace, SmBoolean bDoCheck3d) ; // in : TRUE = also check 3d dists for XSects     
                                                                          // in : FALSE= only check 2d dists for XSects     

  // top level merge entry point for boolean operator
  // merge classified 3DIntersectionCurve into two Breps that generated it.  
  SmStatus MergeClassifications(SmCurveClassification & rOther,                        // in : other target classification                                       
                                SmBoolean               bMergeSingularities,           // in : TRUE = merge each boundary PointClassifications                   
                                SmTArray<SmFace*>     * pOptNewFaces = NULL,           // out: optional list of thisBrep  new faces, NULL to ignore.             
                                                                                       //    : When an existing face is split by an interval into 2              
                                                                                       //    : the oldFace is reused and 1 newFace is constructed and            
                                                                                       //    : only the newFace is placed on this list.                          
                                SmTArray<SmFace*>     * pOptNewFacesOther = NULL,      // out: optional list of otherBrep new faces, NULL to ignore.             
                                                                                       //    : When an existing face is split by an interval into 2              
                                                                                       //    : the oldFace is reused and 1 newFace is constructed and            
                                                                                       //    : only the newFace is placed on this list.                          
                                SmTArray<SmEdge*>     * pOptNewEdges = NULL,           // out: optional list of thisBrep  new edges, NULL to ignore.             
                                                                                       //    : when an existing edge is split by a vertex it is reused           
                                                                                       //    : and 1 new edge is constructed - BOTH edges are put on this list.  
                                SmTArray<SmEdge*>     * pOptNewEdgesOther = NULL,      // out: optional list of otherBrep new edges, NULL to ignore.             
                                                                                       //    : when an existing edge is split by a vertex it is reused           
                                                                                       //    : and 1 new edge is constructed - BOTH edges are put on this list.  
                                SmTArray<SmVertex*>   * pOptNewVertices = NULL,        // out: optional list of thisBrep  new vertices, NULL to ignore.          
                                SmTArray<SmVertex*>   * pOptNewVerticesOther = NULL) ; // out: optional list of otherBrep new vertices, NULL to ignore.          

  // merge one classification interval into classification's brep
  SmStatus MergeInterval  (ULONG lIntervalIndex,                           // in : target interval index                               
                           SmTArray<SmFace*>   * pOptNewFaces=NULL,        // out: optional list of created faces, NULL to ignore.     
                                                                           //    : When an existing face is split into 2               
                                                                           //    : the oldFace is reused and 1 newFace is              
                                                                           //    : created and placed on this list.                    
                           SmTArray<SmEdge*>   * pOptNewEdges=NULL,        // out: optional list of created edges, NULL to ignore.     
                           SmTArray<SmVertex*> * pOptNewVertices=NULL,     // out: optional list of created vertices, NULL to ignore.  
                           SmBoolean             bVerticesOnly=FALSE) ;    // in : TRUE = Merge Vertices only                          
                                                                           //    : FALSE= Merge Vertices 1st and Edges 2nd             
                          
  // merge one classification point into classification's brep
  SmStatus MergePointClass(ULONG lPointClassIndex,                         // in : interval boundary index (not interval index)                              
                                                                           //    : range:[0<=lPointClassificationIndex<=SmCurveClassification::m_lSize]      
                           SmTArray<SmEdge*>   * pOptNewEdges = NULL,      // out: optional list of all children edges created by splitting existing edges   
                                                                           //    : note: some children reuse the parent edge data pointers but               
                                                                           //    :       are placed on the list anyway.                                      
                                                                           //    : NULL to ignore.                                                           
                           SmTArray<SmVertex*> * pOptNewVertices = NULL) ; // out: optional list of all new vertices created by merging                      
                                                                           //    : this pointClassification. NULL to ignore.                                 

  // merge all classification intervals into classification's brep
  SmStatus MergeIntervals(SmTArray<SmFace*>   * pOptNewFaces=NULL,         // out: list of new faces, NULL to ignore.                        
                                                                           //    : When an existing face is split into 2                     
                                                                           //    : the oldFace is reused and 1 newFace is                    
                                                                           //    : created and placed on this list.                          
                          SmTArray<SmEdge*>   * pOptNewEdges = NULL,       // out: list of new edges, NULL to ignore                         
                          SmTArray<SmVertex*> * pOptNewVertices = NULL,    // out: list of new vertices, NULL to ignore                      
                          SmFace              * pOrigFace = NULL,          // in : pointer to original face - used for debug display only    
                          SmBoolean             bVerticesOnly = FALSE) ;   // in : TRUE = Merge Vertices only                                
                                                                           //      FALSE= Merge Vertices 1st and Edges 2nd                   
                                                                           //      default:[FALSE]                                           
                        
  SmStatus ClassifyFaceInterval(ULONG          lIntervalIndex,             // in : index of interval in 'this' to classify                                                       
                                SmBoolean      bDoPointClassify,           // in : TRUE  = Do the work to classify MidPoint if interval ends don't touch anything                
                                                                           //    : FALSE = don't                                                                                 
                                const SmFace * pFaceToClassify,            // in : Face to classify tgtInterval against.                                                         
                                const SmFace * pOldFace,                   // in : Face to which interval may already be classified against.                                     
                                                                           //    : Used after a single face is split in two and the classification                               
                                                                           //    : has to be updated to decide to which of the two children faces                                
                                                                           //    : this interval belongs.                                                                        
                                SmBoolean      bUVSpaceOkay = TRUE,        // in : TRUE = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) 
                                                                           //    : FALSE= don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE]       
                                ULONG         lFaceIndx=SM_UNDEF_ULONG) ;  // in : When rFaceProps is available to caller, pass rFaceProps->m_lFaceIndx to debug a target face
                                                                           //      SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_ULONG]
                              
  // when two curveClassifications lie along two edges from two faces that are partially coincident 
  //  - make a new curve that is the coincident sub-interval 
  SmStatus TestForPartialCoincidentEdges(SmCurveClassification & rOther,                // in : 2nd target curve classification                                         
                                                                                        //    : this can be the same value as the 1st target curve classification       
                                         double                & rdMaxPartialCoinGap,   // out: 0.0     =                                                               
                                                                                        //    : notZero = max distance between coincident curve regions                 
                                         SmBSplineCurve       *& rpNewCurveToTest)      // out: NULL    = curve classification is coincident to existing edge           
                                                                                        //    : notNULL = curve is partially coincident to existing edge                
                                        const;



  // for debug purposes
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                         
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                     
                                                                          //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                 
                                SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't  
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order               
                               const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmCurveClassification, SmObject, SmCurveClassification_TYPE) ;

  // draw classification curve and interval boundary points (cyan)
  SmDisplayList * Draw(SmBoolean bDrawObjects=TRUE) const;

  // draw interval curves and boundary points (cyan), and their classify objects
  //  note: use smgfx_SetCurveClassificationObjectLook() to set report colors and sizes 
  SmDisplayList * DrawObjects() const;

} ; // end class SmCurveClassification

#endif // !__SMCURVECLASSIFICATION_H__



