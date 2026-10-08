// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCompositeCurve.h
* PURPOSE: Header file for composite curve.
**********************************************************************/

#ifndef __SMCOMPOSITECURVE_H__
#define __SMCOMPOSITECURVE_H__

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif

#ifndef __SMATTRIBUTE_H__
#include <SmAttribute.h>
#endif

#ifndef __SMSURFACECACHE_H__
#include <SmSurfaceCache.h>
#endif

#if 0
/*******************************************************************//**
PURPOSE: This object is the cache object which contains a composite
    curve region cache.  It is used to speed up operations on the 
    composite curve region such as intersection and point classification.

NOTES: 
***********************************************************************/
class SM_EXPORT SmCCRegionCache : public SmSurfaceCache
{
    friend class SmTree;
private:
    SmCompositeCurveRegion * m_pCCRegion;

public:
//    SmCCRegionCache(const SmFace * cpFace,
//                    double dChordHeightTolerance = 0.0,
//                    double dAngleTolDeg = 0.0,
//                    double dAspectRatio3D = 0.0,
//                    double dMaxSideLength3D = 0.0,
//                    double dMinSideLength3D = 0.0,
//                    double dMinSideLengthRatioUV = 0.001);
//
//    virtual ~SmCCRegionCache();

//    virtual SmStatus PointTest(const SmPoint2d & crUVPoint, SmBoolean & rbPointIsOk) const;

//    virtual SmStatus BuildTree();

//    SmStatus ImplantSegment(ULONG lSegment,
//                            SmBoolean bDoSubdivision,
//                            SmExtent2d & rFaceDomain);

//    SmStatus ImplantCorner(ULONG lSegment, SmExtent2d & rFaceDomain);

    virtual void Draw(SmBoolean bDrawTree=FALSE) const;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmCCRegionCache,SmSurfaceCache,SmCCRegionCache_TYPE);
} ; // end class SmCCRegionCache

#endif // #0

/*******************************************************************//**
PURPOSE: This object is an attribute which will contain information
   describing the origination of an offset curve.  It describes the
   pointers and indexes of the inputs to offsetting.

NOTES: 
***********************************************************************/
class SM_EXPORT SmOffsetMapAttribute : public SmAttribute
{
private:
  SmBoolean   m_bCornerOffset;    // If TRUE this curve is an offset
                                  // of a corner between two curves not the offset of an original curve
  SmBoolean   m_bLeftHandOffset;  // If TRUE this curve results from
                                  // a left handed offset.
  ULONG       m_lCompositeIndex;  // Index pointer to originating Composite Curve
                                  // relative to the order the composites are sent into BuildCompositesFromCurves()
  ULONG       m_lCurve;           // Index to originating curve as indexed in the SmCompositeCurve array
  ULONG       m_lOtherCurve;      // If this is corner offset this is the index of the other curve
  SmCurve *   m_pOriginator;
  double      m_dApproxErr;       // Max error of the offset approximation
public:
  SmOffsetMapAttribute(ULONG lAttributeID, 
                       SmBoolean bCornerOffset, 
                       SmBoolean bLeftHandOffset, 
                       ULONG lCompositeIndex, 
                       ULONG lCurve, 
                       ULONG lOtherCurve,
                       SmCurve *pOrigCurve, 
                       double dApproxErr )
    : SmAttribute(lAttributeID), 
      m_bCornerOffset(bCornerOffset),
      m_bLeftHandOffset(bLeftHandOffset), 
      m_lCompositeIndex(lCompositeIndex),
      m_lCurve(lCurve), 
      m_lOtherCurve(lOtherCurve), 
      m_pOriginator( pOrigCurve ),
      m_dApproxErr(dApproxErr) 
    { }

  SmOffsetMapAttribute(const SmOffsetMapAttribute & crOriginal) 
    : SmAttribute(crOriginal), 
      m_bCornerOffset(crOriginal.m_bCornerOffset),
      m_bLeftHandOffset(crOriginal.m_bLeftHandOffset), 
      m_lCompositeIndex(crOriginal.m_lCompositeIndex),
      m_lCurve(crOriginal.m_lCurve), 
      m_lOtherCurve(crOriginal.m_lOtherCurve),
      m_pOriginator( crOriginal.m_pOriginator ), 
      m_dApproxErr(crOriginal.m_dApproxErr) 
    { }

  virtual ~SmOffsetMapAttribute() {}

  virtual SmAttribute * MakeCopy(const SmContext & crContext) const;

  SmBoolean GetCornerOffset() { return m_bCornerOffset; }

  SmBoolean GetLeftHandOffset() { return m_bLeftHandOffset; }

  ULONG GetCompositeIndex() { return m_lCompositeIndex; }

  ULONG GetCurve() { return m_lCurve; }

  ULONG GetOtherCurve() { return m_lOtherCurve; }

  double GetApproxError() { return m_dApproxErr; }

  SmCurve *GetOrigCurve() { return m_pOriginator; }

  // get memory used and allocated
  virtual ULONG GetMemoryUsed          // rtn: memory actually used
    (ULONG &rlMemoryAllocated) const   // out: total memory allocated
                                       { ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed(lAllocated) ;
                                         rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<SmAObject*>) ;
                                         return(             sizeof(this) + lUsed      - sizeof(SmTArray<SmAObject*>)) ;
                                       }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmOffsetMapAttribute,SmAttribute,SmOffsetMapAttribute_TYPE);

} ; // end class SmOffsetMapAttribute

/*******************************************************************//**
PURPOSE: The composite curve segment defines the data for each segment
    of a composite curve.

NOTES: must remain a static class without virtual functions
       because SmCompositeCurve manages an array of SmCompositeCurveSegments
       using smos_Calloc()
***********************************************************************/
class SmCompositeCurveSegment
{
  friend class SmCompositeCurve;
public:                                                         
                                                                
  SmContinuityType m_eTransition;    // oneof: SM_CT_DISCONTINUOUS  SM_CT_C0        SM_CT_G1                  
                                     //        SM_CT_G1R            SM_CT_G1_G2     SM_CT_G1_G2_G3                     
                                     //        SM_CT_C1             SM_CT_C1_G2     SM_CT_C1_G2_G3     
                                     //        SM_CT_C1_C2          SM_CT_C1_C2_G3  SM_CT_C1_C2_C3  SM_CT_CINFINITY
                                     // note: not really used yet - but  intended to  record the 
                                     //       continuity from the end of this segment to the beginning of the next
  SmBoolean        m_bSameSense;     // TRUE = don't reverse curve in composite, FALSE = do                            
  SmCurve        * m_pParentCurve;   // ptr to curve for this segment (keep this name because it's in the user interface)                           
  double           m_dGapStart;      // gap from this curve's start to last curve's end                        
  double           m_dGapEnd;        // gap from this curve's end   to next curve's start  
  SmBoolean        m_bOwnsCurve;     // TRUE = deletes m_pParentCurve in destructor, FALSE = doesn't   

  // default constructor
  SmCompositeCurveSegment
  (
    SmCurve * pParentCurve = NULL,  ///< [in] : ptr to curve that defines segment shape 
    SmBoolean bSameSense   = TRUE,  ///< [in] : TRUE = don't reverse curve in composite, FALSE = do
    double    dGapStart    = 0.0,   ///< [in] : gap from this curve's start to last curve's end  
    double    dGapEnd      = 0.0    ///< [in] : gap from this curve's end   to next curve's start
  )  
    : m_eTransition (SM_CT_C0),
      m_bSameSense  (bSameSense),
      m_pParentCurve(pParentCurve),
      m_dGapStart   (dGapStart),
      m_dGapEnd     (dGapEnd),
      m_bOwnsCurve  (FALSE)
  { }

  // destructor
 ~SmCompositeCurveSegment() { if(m_bOwnsCurve && m_pParentCurve) { delete m_pParentCurve ; m_pParentCurve  = NULL ; } }

  // equality operator
  SmBoolean operator==(const SmCompositeCurveSegment &crOther) const;

  // simple access
  SmCurve   *GetCurve()                { return( m_pParentCurve) ; }
  SmBoolean  GetSense()                { return( m_bSameSense) ; }
  double     GetGapStart()             { return( m_dGapStart) ; }
  double     GetGapEnd()               { return( m_dGapEnd) ; }

  // utilities
  SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,                 ///< [in] : target output stream
    ULONG          lDBVersionNumber     ///< [in] : database version to get proper sequence of writes
  ) const ;                                                                        

  static  SmStatus ReadFromDB
  (
    SmDatabaseIO             & rDB,              ///< [in] : target output stream
    const SmContext          & crContext,        ///< [in] : context for new object construction
    SmCompositeCurveSegment *&rpNewSegment,      ///< [out]: NULL on input = new object allocated in this routine built from stream data
                                                 ///<        NotNull on input = assumed empty object already allocated filled here from stream data
    ULONG                    lDBVersionNumber    ///< [in] : database version to get proper sequence of writes
  ) ; 

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON_ONLY_BASE(SmCompositeCurveSegment, SmCompositeCurveSegment_TYPE) ;

  // prefix regular one-line dump with a linefeed and a numeric label
  void Dump(ULONG i) const ;


} ; // end class SmCompositeCurveSegment

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmCompositeCurve*) ; 

/*******************************************************************//**
PURPOSE: This composite curve region represends a 2D region in the
             X, Y plane.

NOTES: 
  Sets of closed composite curves can partition the X, Y planes into distinct in/out regions.

  An outer loop divides the infinite region from an inside region.
  An outerloop may contain any number of inner loops.
    Each inner loop divides the inside region from the infinite region.
  The composite curves are assumed to be non-coincident with one another.

  Outerloops traverse in a counter clockwise direction.
  Innerloops traverse in a clockwise direction.

  The object contains: a list of composite curves.
                       a list of indices listing which composite curves act as outer loops.
***********************************************************************/
class SM_EXPORT SmCompositeCurveRegion : public SmAObject
{
private:
  SmTArray<ULONG>             m_vOuterLoops;      // Index of each outer loop within m_vCompositeCurves.  
                                                  //   Non-outer loops are inner loops of the previous 
                                                  //   outer loop.
  SmTArray<SmCompositeCurve*> m_vCompositeCurves; // Composite curves that form the regions.
                                                  //   Closed outer loops must be counter clockwise.
                                                  //   Closed inner loops must be clockwise.  
                                                  //   Non-closed loops are not allowed. 

public:
  // constructor
  SmCompositeCurveRegion(const SmTArray<ULONG> & crOuterLoops,
                         const SmTArray<SmCompositeCurve*> & crCompositeCurves);

  // empty constructor for I/O
  SmCompositeCurveRegion() { }

  // memory usage report 
  // rtn: memory actually used
  ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,       // out: bigger size of all allocated memory in bytes
                      SmMarkType eMarkType=SM_MT_NOMARK)  // in : uses without increment eMarkType value
                     const  
                     {
                       SM_REF1(eMarkType) ;
                       ULONG lAllocated, lThisAllocated, lUsed ;
                       lUsed       = m_vOuterLoops.GetMemoryUsed(lAllocated) ; 
                       lUsed      += m_vCompositeCurves.GetMemoryUsed(lThisAllocated) ; 
                       lAllocated += lThisAllocated ;
                       rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<ULONG>) - sizeof(SmTArray<SmCompositeCurve*>) ;
                       // + cache memory
                       if ( m_pCacheObj )
                       {
                           lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
                           rlMemoryAllocated += lThisAllocated;
                       }
                       return(             sizeof(this) + lUsed      - sizeof(SmTArray<ULONG>) - sizeof(SmTArray<SmCompositeCurve*>) ) ;
                     }

  // graphics
  virtual void Draw() const ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmCompositeCurveRegion,SmAObject,SmCompositeCurveRegion_TYPE) ;

} ; // end class SmCompositeCurveRegion

/*******************************************************************//**
PURPOSE: The SmCompositeCurve class defines a 2D or 3D curve which is
    composed of end-to-end connected (within some tolerance) curves. 

NOTES:  Composite curves with just 1 segment are allowed.
***********************************************************************/
class SM_EXPORT SmCompositeCurve : public SmCurve
{
private:
  ULONG                     m_lNumSegments = SM_UNDEF_ULONG ; // number of segments in this composite curve
  SmCompositeCurveSegment * m_paSegments = NULL ;             // ptr to allocated block for Curve Segments.
                                                              // each segment stores curve shape, orientation bit, and end-gap sizes. 
                                                              //    BlockSize:[m_lNumSegments*Sizeof(SmCompositeCurveSegement)]
  SmBoolean                 m_bClosed = FALSE ;               // TRUE = last curve is connected to first curve, FALSE = not.

public:
  // functional interface
  static SmStatus BuildCompositesFromCurves
    (const SmContext                 & crContext,                 // in : context for new object construction
     const SmTArray<SmBSplineCurve*> & crCurves,                  // in : unordered curves to connect into composites                                                      
                                                                  //      Included curves move to the output composites;
                                                                  //      tiny omitted curves remain caller-owned.
     SmBoolean                         bMakeCurvesHomogeneous,    // in : (not yet used) bMakeCurvesHomogeneous : TRUE = Approx curves with deg 3 NUBs                     
     double                            dThisApproxTol3d,          // in : (not yet used) dThisApproxTol3d = tol used if an approximation is required.                      
     double                            dSamePointTolerance,       // in : if      gap < dSamePointTolerance, connect                                                       
     double                            dDistanceToAveragePoints,  // in : (not yet used) else if gap < dOptDistanceToAverage,    snap to avg and connect - not implemented 
     double                            dDistanceToExtendTrim,     // in : (not yet used) else if gap < dOptDistanceToExtendTrim, trim and connect        - not implemented 
     double                            dDistanceToCreateLine,     // in : else if gap < dOptDistanceToCreateLine, insert line seg and connect                              
     double                            dDistanceToCreateBlend,    // in : (not yet used) else if gap < dOptDistanceToCreateBlend, insert blend and connect                 
     SmTArray<SmCompositeCurve*>     & rComposites);              // out: Composite Curve using unordered crCurves[i] as segments                                          
                                                                  //      These objects own and delete the included input curves.
  // constructor
  SmCompositeCurve(ULONG                       lDimension,        // in : 3d or 2d, should match curve dimensions                 
                   const SmTArray<SmCurve*>  & crCurves,          // in : ordered array of curve segments                         
                   SmBoolean                   bIsClosed = TRUE,  // in : TRUE = last curve is connected to first, FALSE = open   
                   const SmTArray<SmBoolean> * cpSenses = NULL,   // in : bit for each curve, TRUE = reverse curve in composite   
                                                                  //      NULL = every curve assumed to be same sense.            
                   const SmTArray<double>    * cpGaps = NULL) ;   // in : gap for each curve from its end to next curve's start   
                                                                  //      NULL = every cached gap set to 0.0                      
  
  // empty constructor for I/O
  SmCompositeCurve(ULONG lDim) : SmCurve(lDim) { }
  
  // Copy Constructor - segment curves are DeepCopied placing ownership of the seg copies within this obj.
  SmCompositeCurve(const SmCompositeCurve & crSourceCurve) ;  
  
  // destructor
  virtual ~SmCompositeCurve();
  
  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const;
  
  // bounding box based on fine tessellation sample points
  //   SmBSplineCurve uses controlPolygon of containing spans
  virtual SmStatus CalculateBoundingBox(const SmExtent1d & crInterval,         // in : target interval of interest - this is used                              
                                        SmExtent3d       * pNormalBox = NULL,  // out: Axis alligned box, NULL to ignore, default:[NULL]                       
                                        SmPseudoBox      * pPseudoBox = NULL,  // out: Non-axis aligned box, NULL to ignore, default:[NULL]                    
                                        SmPolarBox       * pPolarBox = NULL,   // out: Curve tangent vector field bounding box, NULL to ignore, default:[NULL] 
                                        SmBoolean          bExpandBox = TRUE)  // in : TRUE = expand BBoxes prior to return  FALSE= don't                      
                                       const ;
  
  // returns effective natural interval of stacking all intervals together
  virtual SmExtent1d  GetNaturalInterval()  const;
  
  virtual SmStatus    GetKnots(SmTArray<double> & rKnots,                       // out: Unique knot vector                                           
                               SmTArray<ULONG>  * pKnotMultiplicities = NULL,   // out: multiplicity value for each knot                             
                               const SmExtent1d * pOptIvl = NULL)               // in : interval of interest, NULL=Natural Interval, default:[NULL]  
                              const ;        
                     
  // make an exact copy of any curve
  virtual SmStatus    Copy(const SmContext & crContext,
                           SmCurve        *& rpNewCurve)
                          const
                          {
                            rpNewCurve = new (crContext) SmCompositeCurve( *this );
                            NER( rpNewCurve );
                            return SM_SUCCESS;
                          }
  
  // append segment to curve
  SmStatus            AddCurve(SmCurve   *pCurve,                   // in : Note the curve is consumed by the composite.                 
                               SmBoolean  bSense,                   // in : Sense of the curve                                           
                               SmBoolean  bAddToBeginning,          // in : TRUE - add to beginning of composite                         
                                                                    //      FALSE - add to end of composite                              
                               SmBoolean  bMakesCompositeClosed) ;  // in : If TRUE this curve closes the composite into a closed loop.  
                      
  // build a single curve approximation of the composite curve - does not match parameterization
  virtual SmStatus    ApproximateCurve(const SmContext        & crContext,                              // in : memory context for new object                                                               <br>
                                       const SmTArray<double> & crBreakParams,                          // NotUsed: [in] : input curve params exactly interpolated by approx curve.                                    <br>
                                                                                                        //                 Always representing Interval end points                                                     <br>
                                                                                                        //                 and commonly also representing orig curve discontinuities.                                  <br>
                                       double                   d3DApproxTol,                           // in : Max dist allowed between approx and original curves.                                        <br>
                                       double                 & rdMaxGap3d,                             // out: Actual Max dist between approx and original curves.                                         <br>
                                                                                                        //      note: value is not exact, only based on sampling.                                           <br>
                                       SmBSplineCurve        *& rpNewBSplineCurve,                      // out: the new BSpline                                                                             <br>
                                       SmBoolean                bOptCreateAnalytics = TRUE,             // NotUsed: [in] : TRUE = create derived type analytics (SmLine, SmCircle, ...) when possible                  <br>
                                                                                                        //                 FALSE= don't, default:[TRUE]                                                                <br>
                                       SmBoolean                bOptMatchParameterization = FALSE,      // NotUsed: [in] : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values       <br>
                                                                                                        //                 FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)                        <br>
                                                                                                        //                 default:[FALSE], FALSE produces lower control point count curves for slightly more cost.    <br>
                                       SmBoolean                bCopyBSplines = FALSE)                  // NotUsed: [in] : TRUE = if this Curve is a BSpline just copy it                                              <br>
                                                                                                        //                 FALSE= approximate the curve                                                                <br>
                                      const ;
                       
  // Get the canonical representation of the composite curve.
  SmStatus            GetCanonical(ULONG               & rlDimension,      // out: 3d or 2d                                                        
                                   SmTArray<SmCurve*>  & rCurves,          // out: array of curves in the composite                                
                                   SmBoolean           & rbClosed,         // out: TRUE = last curve is connected to the first curve, FALSE = not  
                                   SmTArray<SmBoolean> * pSenses = NULL)   // out: optional sense bit for each curve, NULL to ignore               
                                  const ;
  
  //simple data access
  const SmCompositeCurveSegment * GetCurveSegment(ULONG lSegmentIndex) const ;
  ULONG                           GetNumSegments ()                    const { return m_lNumSegments ; }
  virtual SmBSplineCurve        * GetRootCurve   ()                    const { return(m_lNumSegments == 1 ? m_paSegments[0].m_pParentCurve->GetRootCurve() : NULL) ; }
  
  void SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                 if (m_lNumSegments != SM_UNDEF_ULONG) {
                                                     for (ULONG ii = 0; ii < m_lNumSegments; ++ii) {
                                                         if (m_paSegments[ii].m_bOwnsCurve)
                                                         { m_paSegments[ii].m_pParentCurve->SetContext(cpContext); }
                                                     }
                                                 }
                                               }
  // predicates
  virtual SmBoolean IsClosed(const SmExtent1d & , double =0.0) const { return m_bClosed; }
  
  // construct and return a NURB that spans all segments
  SmStatus          MakeCompositeNurb(const SmContext & crContext,            // in : context for new object construction          
                                      SmBSplineCurve *& rpCompositeNurb)      // out: new single Nurb curve spanning all segments  
                                     const;
                    
  static SmStatus   CreateOffsetsOfCurve(const SmContext           & crContext,           // in : context for new object construction                                                   
                                         const SmBSplineCurve      * cpCurveToOffset,     // in : Single curve to offset                                                                
                                         ULONG                       lOffsetDirection,    // in : Corresponding direction of each composite                                             
                                                                                          //      curve to offset.  1-LEFT, 2-RIGHT, 3-BOTH                                             
                                         double                      dXSectTol3d,         // in : min distance between distinct points                                                  
                                         double                      dApproxTol3d,        // in : max allowed distance allowed between a curve and its approximation                    
                                         const SmVector3d          & crOffsetPlaneNormal, // in : Defines, along with the curve's parameter direction,                                  
                                                                                          //      the right and left hand offset directions.                                            
                                         SmBoolean                   bTrimResults,        // in : TRUE = trim RawOffset to close offsets of closed lines and remove bowties             
                                                                                          //      FALSE= don't                                                                          
                                         double                      dOffsetDistance,     // in : distance curve is being offset                                                        
                                         SmTArray<SmBSplineCurve*> & rTrimmedOffsets,     // out: Resulting offset curves will have attribute attached                                  
                                                                                          //      describing origination of curve                                                       
                                         SmBoolean      bOptMatchParameterization=FALSE,  // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values 
                                                                                          //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)                  
                                                                                          //      note: FALSE produces lower control point count curves for slightly more cost.         
                                         SmTArray<double>          * pOptMaxGap3d=NULL) ; // out: optional array of MaxGap3d values for each TrimmedOffset                              
                                                      
                                                
  static SmStatus   ComputeProjectedLoopOrientation(const SmTArray<SmCurve*>  & crCurves,     // in : Ordered list of                                          
                                                                                              //      curves that go around the loop.                          
                                                    const SmTArray<SmBoolean> & crSenses,     // in : Specifies the orientation                                
                                                                                              //      of the curve relative to the loop.  TRUE means that the  
                                                                                              //      curve has the same orientation as the loop.              
                                                    const SmVector3d          & crNormal,     // in : Normal of the projection plane                           
                                                    SmOrientType              & reLoopOrient, // out: SM_OT_SAME     = CCW walk (outer loop)                   
                                                                                              //      SM_OT_OPPOSITE = CW  walk (inner loop)                   
                                                                                              //      SM_OT_UNKNOWN  = zero area, neither CW or CCW.           
                                                    SmExtent2d                * p2DBounds     // out: UVBoundingBox of all UVTrimCurve SamplePoints            
                                                                                              //      NULL to ignore.                                          
                                                  );
                                                             
  static SmStatus   CreateOffsetsOfManyCurves(const SmContext                   & crContext,             // in : context for new object construction                                                                                             
                                              const SmTArray<SmCompositeCurve*> & crCurvesToOffset,      // in : Array of composite curves to offset                                                                                             
                                              const SmTArray<ULONG>             & crOffsetDirections,    // in : Corresponding direction of each composite                                                                                       
                                                                                                         //      curve to offset.  1-LEFT, 2-RIGHT, 3-BOTH                                                                                       
                                              double                              dXSectTol3d,           // in : Minimum distance at which offset curve end-gaps are filled with corner curves                                                   
                                              double                              dApproxTol3d,          // in : Tolerance to which BSpline Approximations to exact offset curves are built                                                      
                                              const SmVector3d                  & crOffsetPlaneNormal,   // in : Defines, along with the curve's parameter direction,                                                                            
                                                                                                         //      the right and left hand offset directions.                                                                                      
                                              SmOffsetCornerType                  eOffsetCornerType,     // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.                                
                                                                                                         //      SM_OC_FILLET_CORNER: corner = fillet arc centered on crVertexPoint running to given end points                                  
                                                                                                         //      SM_OC_LINEAR_CHAMFER: corner = line between endpoints (result is actually within offset distance so bTrimResults must = false). 
                                              SmBoolean                           bTrimResults,          // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points                                       
                                                                                                         //      FALSE= skip trim step                                                                                                           
                                              double                              dOffsetDistance,       // in : offset distance (a negative value negates the offset direction)                                                                 
                                              SmTArray<SmBSplineCurve*>         & rTrimmedOffsets,       // out: Resulting offset curves will have attribute attached                                                                            
                                                                                                         //      describing origination of curve                                                                                                 
                                              SmBoolean               bOptMatchParameterization=FALSE,   // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)                                                                       
                                                                                                         //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated                                        
                                                                                                         //      default:[FALSE]                                                                                                                 
                                              SmTArray<double>                  * pOptMaxGaps3d=NULL) ;  // out: optional MaxGap for each output approximation
                    
  SmStatus           CreateTrimmedOffsets(const SmContext           & crContext,                         // in : context for new object construction                                                                                                     
                                          double                      dXSectTol3d,                       // in : Minimum distance at which offset curve end-gaps are filled with corner curves                                                           
                                          double                      dApproxTol3d,                      // in : Tolerance to which BSpline Approximations to exact offset curves are built                                                              
                                          const SmVector3d          & crOffsetPlaneNormal,               // in : Defines, along with the curve's parameter direction,                                                                                    
                                                                                                         //      the right and left hand offset directions.                                                                                              
                                          SmOffsetCornerType          eOffsetCornerType,                 // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.                                        
                                                                                                         //      SM_OC_FILLET_CORNER   : corner = fillet arc centered on crVertexPoint running to given end points                                       
                                                                                                         //      SM_OC_LINEAR_CHAMFER  : corner = line between given end points (result is actually within offset distance so bTrimResults must = false) 
                                          SmOffsetDirectionType       eOffsetDirection,                  // in : Corresponding direction of each composite                                                                                               
                                                                                                         //      curve member to offset.  1-LEFT, 2-RIGHT, 3-BOTH                                                                                        
                                          SmBoolean                   bTrimResults,                      // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points                                               
                                                                                                         //      FALSE= skip trim step                                                                                                                   
                                          double                      dOffsetDistance,                   // in : offset distance (a negative value negates the offset direction)                                                                         
                                          SmTArray<SmBSplineCurve*> & rTrimmedOffsets,                   // out: unordered array of offset curves will have attribute attached                                                                           
                                                                                                         //      describing origination of curve                                                                                                         
                                          SmBoolean                   bOptMatchParameterization=FALSE)   // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)                                                                               
                                                                                                         //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated                                                
                                         const ;                                                        
                                                        
  static SmStatus   IntersectAndTrimOffsets(const SmContext                & crContext,               // in : context for new object contsruction                            
                                            const SmTArray<SmCurve*>       & crOffsets,               // in : raw offsets of the Original Curves                             
                                            double                           dOffsetDistance,         // in : distance curves are being offset                               
                                            double                           dXSectTol3d,             // in : used as XSectTol3d and divided by 100 for a ShortLine length   
                                            const SmTArray<const SmCurve*> & crTotalOriginalCurves,   // in : to classify against                                            
                                            SmTArray<SmCurve*>             & rTrimmedOffsets) ;       // out: Trimmed actual offsets built from input raw offsets minus      
                                                                                                      //      those in the 'forbidden zone.' Those closer to some other      
                                                                                                      //      UnTrimmedCurve than dOffsetDistance.                           
                    
  virtual SmStatus  Transform(const SmAxis2Placement & crRotateNMove,
                              const SmVector3d * cpOptScale=NULL) ;
  
  // get CompositeCurve Used and Total Allocated Memory sizes (does not include any attribute memory)
  virtual ULONG     GetMemoryUsed(ULONG    & rlMemoryAllocated,        // out]: bigger size of all allocated memory in bytes
                                  SmMarkType eMarkType=SM_MT_NOMARK)   // in] : uses without increment eMarkType value      
                                 const ;
                    
  // maintenance
  virtual SmDisplayList * Draw      (const SmExtent1d * pInterval = NULL,           // NotUsed: in : Target Interval, NULL = Use Natural Interval, default:[NULL]      
                                     SmBoolean          bAddToUIPickList = FALSE,   // in : TRUE = Add this Curve to UI pick interface for debugging, default:[FALSE]  
                                     SmPlane          * pOptOutPlane = NULL,        // NotUsed: in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]        
                                     SmGfxArraySet    * pOptGfxSet = NULL)          // in : used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.    
                                    const ;
  
  SmDisplayList         * DrawParams(const SmExtent1d * pInterval=NULL,            // NotUsed: in : Draw curve and increasing Param Points
                                     SmBoolean          bAddToUIPickList=FALSE)    // in :
                                    const ;   
                        
                        
  virtual void            Dump( const TCHAR * message ) const;
  virtual void            Dump( ULONG ) const;
  
  // I/O assist
  virtual SmStatus        WriteToDB(SmDatabaseIO & rDB,                      // in : target output stream                              
                                    ULONG          lDBVersionNumber)         // in : database version to get proper sequence of writes 
                                   const ;   
                          
  static  SmStatus        ReadFromDB(SM_TYPE           lType,              // NotUsed: in: Object type to be read                                               
                                     SmDatabaseIO    & rDB,                // in : target output stream                                                        
                                     ULONG             lDim,               // in : curve image space dim, 2 or 3                                                                            
                                     const SmContext & crContext,          // in : context for new object construction                                         
                                     SmCurve         *&rpNewCurve,         // out: NULL on input = new object allocated in this routine built from stream data 
                                                                           //      NotNULL on input = pointer to an empty object to be filled by this routine  
                                     ULONG             lDBVersionNumber) ; // in : database version to get proper sequence of writes
                          
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmCompositeCurve,SmCurve,SmCompositeCurve_TYPE);

} ; // end class SmCompositeCurve

#endif // !__SMCOMPOSITECURVE_H__


