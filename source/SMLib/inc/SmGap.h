// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmGap.h
* PURPOSE: Header file for SmGap.
*
* CONTAINS --- 
* Class SmGapFunction          // base class representing the set of all gaps between a pair of topology objects
*                              // currently approximated by an array of SmGapSamples. The exact representation 
*                              // would be a function.
*         SmCrvPtGapFunction   //   GapFunction(s) from Crv to Pt,  defined over Crv param s                       
*         SmCrvCrvGapFunction  //   GapFunction(s) from Crv to Crv, defined over Crv param s
*         SmCrvSrfGapFunction  //   GapFunction(s) from Crv to Srf, defined over Crv param s
*         SmSrfPtGapFunction   //   GapFunction(u,v) from Srf to Pt,  defined over Srf param u,v  
*         SmSrfCrvGapFunction  //   GapFunction(u,v) from Srf to Crv, defined over Srf param u,v 
*         SmSrfSrfGapFunction  //   GapFunction(u,v) from Srf to Srf, defined over Srf param u,v 
*
* class SmGap           // the largest single gap between a pair of topology objects.
*                       // contains: gap topology type, gap 3d distance, topology objects and param values identifying the gap ends
*                       //  Gap Types are oneof: SM_GT_VERTEX_EDGE          (same as  SM_GT_EDGE_VERTEX)
*                       //                       SM_GT_VERTEX_FACETRIMCURVE (same as  SM_GT_FACETRIMCURVE_VERTEX)        
*                       //                       SM_GT_VERTEX_FACE          (same as  SM_GT_FACE_VERTEX)
*                       //                         
*                       //                       SM_GT_EDGE_FACETRIMCURVE   (same as  SM_GT_FACETRIMCURVE_EDGE)  
*                       //                       SM_GT_EDGE_FACE            (same as  SM_GT_FACE_EDGE)       
*                                   //                       SM_GT_NO_GAP               Uninit state
*         SmVertexEdgeGap           // from VertexPt to EdgeEndPt
*         SmVertexFaceTrimCurveGap  // from VertexPt to CrvOnSurf(UVTrimCurve,FaceSurf).EndPt
*         SmVertexFaceGap           // from VertexPt to FaceSurface_DropPt
*         SmEdgeEdgeGap             // From Edge1_EndPt to Edge2_EndPt
*         SmEdgeFaceTrimCurveGap    // MaxGap from EdgeCurve to CrvOnSurf(UVTrimCurve,FaceSurf))
*         SmEdgeFaceGap             // MaxGap from EdgeCurve to FaceSurface_DropCrv

*                       //                       
* class SmEdgeFaceGap           SmGapArray      // A list of SmGaps for a Topology Object and the largest of all gaps in the list.
* class SmGapSample     // gap geometry type (SM_GD_UNKNOWN  ), gap start and end point data (position and                        )
*                       //                   (SM_GD_NORMAL   )                               ( associated geometry parameter value)
*                       //                   (SM_GD_BOUNDARY )
*                       //                   (SM_GD_NO_DROP  )
* class SmLocalInterval // Interval marking the boundary points where one shape is within tol of another shape.
**********************************************************************/

#ifndef __SMGAP_H__
#define __SMGAP_H__

#include <SmVector2d.h>
#include <SmVector3d.h>
#include <SmCoreTypes.h>
#include <SmTopoTypes.h>
#include <SmTArray.h>
#include <SmCurveClass.h>

class SmVertexuse ;
class SmEdgeuse ;
class SmFaceuse ;
class SmLoopuse ;
class SmGapArray ;
class SmGapSample ;

/*******************************************************************//**
PURPOSE: This enum defines the set of gap types

NOTES:  Gaps are named by the pair of topology types that define them.
  For completeness, each gap type is given two gap names made
  by exchanging the order in which the type names appear within the gap name.  
***********************************************************************/
// backwards support for obsolete names
#define SM_GT_EDGE_VERTEX        SM_GT_VERTEX_EDGE
#define SM_GT_UVTRIMCURVE_VERTEX SM_GT_VERTEX_FACETRIMCURVE
#define SM_GT_FACE_VERTEX        SM_GT_VERTEX_FACE
#define SM_GT_UVTRIMCURVE_EDGE   SM_GT_EDGE_FACETRIMCURVE   
#define SM_GT_FACE_EDGE          SM_GT_EDGE_FACE
#define SM_GT_VERTEX_UVTRIMCURVE SM_GT_VERTEX_FACETRIMCURVE
#define SM_GT_EDGE_UVTRIMCURVE   SM_GT_EDGE_FACETRIMCURVE       

enum SmGapType 
{ SM_GT_UNKNOWN,               // gap type not specified
  SM_GT_VERTEX_EDGE,           // a 3d gap between a Vertex->Point and a point on an Edge->Curve
  SM_GT_VERTEX_FACETRIMCURVE,  // a 3d gap between a Vertex->Point and a point on a FaceTrimCurve 
  SM_GT_VERTEX_FACE,           // a 3d gap between a Vertex->Point and a point on Face->Surface
  SM_GT_EDGE_EDGE,             // a 3d Gap between a point on an Edge->Curve and a point on another Edge->Curve
  SM_GT_EDGE_FACETRIMCURVE,    // a 3d gap between a point on an Edge->Curve and a point on a FaceTrimCurve 
  SM_GT_EDGE_FACE,             // a 3d gap between a point on an Edge->Curve and a point on Face->Surface
  SM_GT_NO_GAP                 // cached gap types 
                               //      for Loop_TYPE  Vertexuse cached Vertex/Edge gap  (they don't have Vertex/Edge gaps)
                               //      for Shell_TYPE Vertexuse cached Vertex/Edge gap  (they don't have Vertex/Edge gaps)
                               //      for Shell_TYPE Vertexuse cached Vertex/Face gap  (they don't have Vertex/Face gaps)
                               //      for Shell_TYPE Edgeuse   cached Edge/Face gap    (they don't have Edge/Face   gaps)
} ;

/*******************************************************************//**
PURPOSE: This enum defines the set of GapSample types

NOTES: 
***********************************************************************/
enum SmGapSampleType
{   SM_GS_CRV_PT,            // sample from Curve to Point
    SM_GS_CRV_CRV,           // sample from Curve to Curve
    SM_GS_CRV_SRF,           // sample from Curve to Surface
    SM_GS_SRF_PT,            // sample from Surface to Point
    SM_GS_SRF_CRV,           // sample from Surface to Curve
    SM_GS_SRF_SRF            // sample from Surface to Surface
} ;

/*******************************************************************//**
PURPOSE: This enum defines the set of drop point classifications

NOTES: 
***********************************************************************/
enum SmGapDropType
{   SM_GD_UNKNOWN,           // Uninitialized value
    SM_GD_NORMAL,            // Gap runs from this geometry point along an other geometry Normal vector to Other geometry point
    SM_GD_BOUNDARY,          // Gap runs from this geometry point to neareast other geometry boundary point
    SM_GD_NO_DROP            // No Drop Point from this geometry point to other geometry was found
} ;

/*******************************************************************//**
PURPOSE: SmGap represents a gap between two points on two topology objects
  and is the base class for classes: SmVertexEdgeGap
                                     SmVertexFaceTrimCurveGap
                                     SmVertexFaceGap
                                     SmEdgeEdgeGap
                                     SmEdgeFaceTrimCurveGap
                                     SmEdgeFaceGap

NOTES: An SmGap represents one sampled 3d gap between a pair
  of connected topology objects and are commonly used to represent 
  min or max gaps between those objects.

  An SmGapSample represents one gap (of usually an infinite number of gaps) 
  sampled between a pair of geometry objects. 
  Ordered sets of SmGapSamples are used by the Gap functions to 
  approximate continuous GapFunctions as sampled discrete functions.

 Gaps store: 
     The gap Topology Objects (expected to be connected in a Brep Topology Graph)
     The gap end points (points located on topology objects by param point locations)
     The gap size            = 3d Dist between end points,
     The gap XSectTol3d size = SmTol::GetXSectTol3d(Object pair)

 The 4 kinds of Gap end points are:
     Vertex        -> Point
     Edge          -> Curve(T) for param value T
     FaceTrimCurve -> Face->Surface(UVTrimCurve(T)) for param value T
     Face          -> Surface(U,V) for param values U and V

 Gap topology objects are within intersecting distance at this gap when
     (GetGap() <= GetXSectTol3d()) == TRUE as returned by IsIntersecting()
 
 An SmGap object is the min sized version of the general SmGapSample object.  
 SmGaps       are used to cache Gap values and are minimally sized.
 SmGapSamples are used by SmGapFunction and store more complete data.
 
 A gap consists of two end points which are Points on various kinds of  
 topological objects and the distance between those end points.

 The SmGap Derived classes are named for the pair of objects making the gap as:

+------------------------+------------------------+--------------------------------------+---------------+-----------------+-------------------+
|CLASS                   | GAP DESCRIPTION        | MEMBERS                              | GetEdgeT(i=0) |GetUVTrimCurveT()|GetFaceUV()        |
+------------------------+------------------------+--------------------------------------+---------------+-----------------+-------------------+
|SmGap                   | Gap Base Class         | m_dGap3d                             |SM_UNDEF_DOUBLE| SM_UNDEF_DOUBLE |   NULL            |
+------------------------+------------------------+--------------------------------------+---------------+-----------------+-------------------+
|SmVertexEdgeGap         | Vertex to Edge         | m_pVertexuse, m_dEdgeT               |   m_dEdgeT    | SM_UNDEF_DOUBLE |[ SM_UNDEF_DOUBLE, |
|                        |                        |                                      |               |                 |  SM_UNDEF_DOUBLE] |
|SmVertexFaceTrimCurveGap| Vertex to FaceTrimCurve| m_pVertexuse, m_dUVTrimCurveT        |SM_UNDEF_DOUBLE| m_dUVTrimCurveT |m_pVertexuse       |
|                        |                        |                                      |               |                 | ->UVTrimCurve     |
|                        |                        |                                      |               |                 |  (m_dUVTrimCurveT)|                                        
|SmVertexFaceGap         | Vertex to Face         | m_pVertexuse, m_sFaceUV              |SM_UNDEF_DOUBLE| SM_UNDEF_DOUBLE |& m_sFaceUV        |
|SmEdgeEdgeGap           | EdgeEndPt to EdgeEndPt | m_pEdgeuse1, m_dEdgeT1,              | i=0, m_dEdgeT1| SM_UNDEF_DOUBLE |[ SM_UNDEF_DOUBLE, |
|                        |                        | m_pEdgeuse2, m_dEdgeT2               | i=1, m_dEdgeT2|                 |  SM_UNDEF_DOUBLE] |
|SmEdgeFaceTrimCurveGap  | Edge   to FaceTrimCurve| m_pEdgeuse, m_dEdgeT, m_dUVTrimCurveT|   m_dEdgeT    | m_dUVTrimCurveT |m_pVertexuse       |
|                        |                        |                                      |               |                 | ->UVTrimCurve     |
|                        |                        |                                      |               |                 |  (m_dUVTrimCurveT)|
|SmEdgeFaceGap           | Edge   to Face         | m_pEdgeuse, m_dEdgeT, m_sFaceUV      |   m_dEdgeT    | SM_UNDEF_DOUBLE |& m_sFaceUV        |
+------------------------+------------------------+--------------------------------------+---------------+-----------------+-------------------+

***********************************************************************/
class SM_EXPORT SmGap
{ public:
  // m_dGap3d is a cached value without a ready bit because
  //                            it becomes stale when the shape of the gap objects 
  //                            change and there is no way to map back from
  //                            an object change to all the gap objects that may exist.
  //                            It is computed in CacheGap() which is called by 
  //                            derived virtual Set() methods.
  // Algorithms using and storing gaps have to manage their own stale SmGap objects.

  double              m_dGap3d ;        // 3d gap between two topology objects
                                        // -SM_UNDEF_DOUBLE == undefined 

  // constructor, destructor, operator==, interchange SmGap and double                                       
  SmGap(double dVal = -SM_UNDEF_DOUBLE)           { SM_DBG_WARN_IF(dVal < 0.0 && dVal != -SM_UNDEF_DOUBLE, _T("SmGap constructor: tried to assign neg val to gap, changed to:[fabs(val)]")) ; 
                                                    m_dGap3d        = smos_Fabs(dVal) ;    // gwc: needed to interchange SmTol3d with double
                                                  }
  SmGap          & operator= (double dVal)        { SM_DBG_WARN_IF(dVal < 0.0 && dVal != -SM_UNDEF_DOUBLE, _T("tried to assign neg val to gap, changed to:[fabs(val)]")) ; 
                                                    m_dGap3d = smos_Fabs(dVal) ; 
                                                    return *this ; 
                                                  }

  virtual  ~SmGap()                               { m_dGap3d = -SM_UNDEF_DOUBLE ; }
  virtual SmGap & operator=(const SmGap & crGap)  { if(this == &crGap) { return *this ; }
#if SM_DEBUG_CODE
                                                    SmBoolean bType = GetType() == crGap.GetType();
                                                    AE_MSG( bType, _T("SmGap Operator= called with different derived object types. This is a bug.")) ;
#endif
                                                    m_dGap3d        = crGap.m_dGap3d ; 
                                                    return *this ; 
                                                  }
  virtual SmBoolean operator==(const SmGap & crGap) const ;
  SmBoolean operator>(const SmGap & crGap)  const { return m_dGap3d > crGap.m_dGap3d ; }
  SmBoolean operator<(const SmGap & crGap)  const { return m_dGap3d < crGap.m_dGap3d ; }
  SmBoolean operator>=(const SmGap & crGap) const { return m_dGap3d >= crGap.m_dGap3d ; }
  SmBoolean operator<=(const SmGap & crGap) const { return m_dGap3d <= crGap.m_dGap3d ; }
  virtual SmGap   * MakeCopy() const  { return new SmGap(*this) ; }
  virtual SmBoolean IsInit() const    { AE(0) ; return(FALSE) ; }
  virtual void      ReSet()           { AE(0) ; }

  // gap access
  operator const double&() const         { return m_dGap3d ; } // when SmGap is used as a double - it's its 3d Gap size 
  operator       double&()               { return m_dGap3d ; } // when SmGap is used as a double - it's its 3d Gap size 
  double       GetLength      () const { return m_dGap3d ; }
  SmStatus     GetTopoObjects (SmObject *& rpObj1,  SmObject *& rpObj2)  const ;
  SmStatus     GetGapEndPoints(SmPoint3d & rPoint1, SmPoint3d & rPoint2) const ;
  SmXSectTol3d GetXSectTol3d  () const { SmObject *pObj1, *pObj2 ; 
                                         GetTopoObjects(pObj1, pObj2) ; 
                                         return( SmTol::GetXSectTol3d(pObj1, pObj2) ) ; 
                                       }
  SmBoolean    IsIntersecting () const { return m_dGap3d <= GetXSectTol3d().val ; }

  // calc and cache GapSize value
  void        CacheGap        () { SmPoint3d sPoint1, sPoint2 ; 
                                   GetGapEndPoints(sPoint1, sPoint2) ;
                                   m_dGap3d = sPoint1.DistanceBetween(sPoint2) ; 
                                 } 
  // simple access
  virtual SmVertex    * GetVertex   ()              const { return NULL ; }                     // get vertex    when gap is bounded by one, else return NULL
  virtual SmVertexuse * GetVertexuse()              const { return NULL ; }                     // get vertexuse when gap is bounded by one, else return NULL
  virtual SmEdge      * GetEdge     (ULONG lIndx=0) const { SM_REF1(lIndx) ; return NULL ; }    // get edge      when gap is bounded by one, else return NULL
  virtual SmEdgeuse   * GetEdgeuse  (ULONG lIndx=0) const { SM_REF1(lIndx) ; return NULL ; }    // get edgeuse   when gap is bounded by one, else return NULL
  virtual SmFace      * GetFace     ()              const { return NULL ; }                     // get face      when gap is bounded by one, else return NULL
  
  //    parameter access return values for the various derived classes are
  //  +------------------------+---------------+-----------------+-------------------+
  //  |class                   | GetEdgeT(i=0) |GetUVTrimCurveT()|GetFaceUV()        |
  //  +------------------------+---------------+-----------------+-------------------+
  //  |SmVertexEdgeGap         |   m_dEdgeT    | SM_UNDEF_DOUBLE |[ SM_UNDEF_DOUBLE, |
  //  |                        |               |                 |  SM_UNDEF_DOUBLE] |
  //  |SmVertexFaceTrimCurveGap|SM_UNDEF_DOUBLE| m_dUVTrimCurveT |m_pVertexuse       |
  //  |                        |               |                 | ->UVTrimCurve     |
  //  |                        |               |                 |  (m_dUVTrimCurveT)|
  //  |SmVertexFaceGap         |SM_UNDEF_DOUBLE| SM_UNDEF_DOUBLE |& m_sFaceUV        |
  //  |SmEdgeEdgeGap           |i=0, m_dEdgeT0 | SM_UNDEF_DOUBLE |[ SM_UNDEF_DOUBLE, |
  //  |                        |i=1, m_dEdgeT1 |                 |  SM_UNDEF_DOUBLE] |
  //  |SmEdgeFaceTrimCurveGap  |   m_dEdgeT    | m_dUVTrimCurveT |m_pVertexuse       |
  //  |                        |               |                 | ->UVTrimCurve     |
  //  |                        |               |                 |  (m_dUVTrimCurveT)|
  //  |SmEdgeFaceGap           |   m_dEdgeT    | SM_UNDEF_DOUBLE |& m_sFaceUV        |
  //  +------------------------+---------------+-----------------+-------------------+
  virtual double    GetEdgeT(ULONG lIndx=0) const { SM_REF1(lIndx) ; return SM_UNDEF_DOUBLE ; } // edge->Curve param       when gap is bounded by an Edge, else return SM_UNDEF_DOUBLE
  virtual double    GetUVTrimCurveT()       const { return SM_UNDEF_DOUBLE ; }                  // edge->UVTrimCurve param when gap is bounded by a FaceTrimCurve, else return SM_UNDEF_DOUBLE
  virtual SmPoint2d GetFaceUV()             const { return SmPoint2d() ; }                      // face->Surface param     when gap is bounded by a Face, else return [SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE]

  // set Vertex/Edge or Vertex/FaceTrimCurve gap
  virtual void Set(const SmVertexuse *pVertexuse, double dEdgeOrUVTrimCurveT)      { SM_REF2(pVertexuse, dEdgeOrUVTrimCurveT) ; AE(0) ; }

  // set Vertex/Face gap
  virtual void Set(const SmVertexuse *pVertexuse, SmPoint2d *pFaceUV)              { SM_REF2(pVertexuse, pFaceUV) ; AE(0) ; }

  // set Edge/FaceTrimCurve gap
  virtual void Set(const SmEdgeuse *pEdgeuse, double dEdgeT, double dUVTrimCurveT) { SM_REF3(pEdgeuse, dEdgeT, dUVTrimCurveT) ; AE(0) ; }

  // set Edge/Face gap
  virtual void Set(const SmEdgeuse *pEdgeuse, double dEdgeT, SmPoint2d *pFaceUV)   { SM_REF3(pEdgeuse, dEdgeT, pFaceUV) ; AE(0) ; }

  // Set gap entity back pointers
  virtual void SetVertexuse(const SmVertexuse *pVertexuse) { SM_REF1(pVertexuse) ; AE(0) ; }
  virtual void SetEdgeuse  (const SmEdgeuse   *pEdgeuse, ULONG lIndx=0)   { SM_REF2(pEdgeuse, lIndx) ; AE(0) ; }

  // maintenance                             
  SmDisplayList *Draw(SmXSectTol3d    sXSectTol3d=SM_XSECT_TOL_3D,     // in : SM_USE_NEWTOL: not used
                                                                       //      OldTol       : override tol value, 0.0 to ignore. default:[SM_XSECT_TOL_3D]
                      SmGfxArraySet * pOptGfxSet =NULL)                // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                     const ;                                           //      NULL to ignore. default:[NULL]

  SmDisplayList *DrawNeighbors(SmGfxArraySet * pOptGfxSet=NULL)        // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                     const ;                                           //      NULL to ignore. default:[NULL]

  void           Dump(ULONG lLabel)  const ;
  void           GetDumpLine(TCHAR *sBuff, TCHAR *sBuffForFile) const ;

  // side effect - TODO this probably does not belong here and may have to change
  SmStatus RefineGeometry(SmGapArray & rAfterGaps,
                          SmBoolean  & bMadeChange) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON_BASE(SmGap, SmGap_TYPE);

} ; // end class SmGap

// // temporarily enable SmScaledZero to look like a double - these methods will be obsolete once SMLib is changed to use SmTol definitions
inline SmGap operator*= (const SmGap & rTolVal, double d) { return d * rTolVal.m_dGap3d ; }
inline SmGap operator/= (const SmGap & rTolVal, double d) { return d / rTolVal.m_dGap3d ; }
inline SmGap operator+= (const SmGap & rTolVal, double d) { return d + rTolVal.m_dGap3d ; }
inline SmGap operator-= (const SmGap & rTolVal, double d) { return d - rTolVal.m_dGap3d ; }
inline int     operator < (double d, const SmGap & rTolVal) { return smos_Fabs(d) < rTolVal.m_dGap3d ; }
inline int     operator > (double d, const SmGap & rTolVal) { return smos_Fabs(d) > rTolVal.m_dGap3d ; }

/*******************************************************************//**
PURPOSE: Derived class for Vertex->Point to Edge->Curve(T) gap

NOTES:
***********************************************************************/
class SM_EXPORT SmVertexEdgeGap : public SmGap
{
  const SmVertexuse * m_pVertexuse = NULL ;        // Vertexuse connecting a vertex to an edge
                                                   //  default:[NULL = uninit]

  double              m_dEdgeT = SM_UNDEF_DOUBLE ; // Edge->Curve parameter bounding Gap used as
                                                   //  m_pVertexuse->GetEdge->Evaluate(m_dEdgeT) ;
                                                   //  default:[SM_UNDEF_DOUBLE = uninit]
  public:                                          
  // Constructor 
  SmVertexEdgeGap(const SmVertexuse * pVertexuse= NULL, 
                  double              dEdgeT    = SM_UNDEF_DOUBLE)  { Set(pVertexuse, dEdgeT) ; }
  virtual SmGap   * MakeCopy() const                                { return new SmVertexEdgeGap(*this) ; }
  virtual SmBoolean IsInit() const                                  { return( m_pVertexuse != NULL) ; }
  virtual void      ReSet()                                         { m_dGap3d     = -SM_UNDEF_DOUBLE ;
                                                                      m_pVertexuse = NULL ;
                                                                      m_dEdgeT     = SM_UNDEF_DOUBLE ;
                                                                    }
  SmVertexEdgeGap & operator=(const SmVertexEdgeGap & crGap)  { return( (SmVertexEdgeGap&)SmVertexEdgeGap::operator=((const SmGap &)crGap) ) ; }
  virtual SmGap & operator=(const SmGap & crGap)              {  SmGap::operator=(crGap) ; 
                                                                 if(crGap.IsKindOf(SmVertexEdgeGap_TYPE)) 
                                                                   { m_pVertexuse = ((SmVertexEdgeGap&)crGap).m_pVertexuse ; 
                                                                     m_dEdgeT     = ((SmVertexEdgeGap&)crGap).m_dEdgeT ;
                                                                   }
                                                                 return *this ;
                                                              }
  virtual SmBoolean operator==(const SmGap &crGap) const      { if(   this == &crGap
                                                                   || (this->IsInit() && crGap.IsInit())) return TRUE ;
                                                                if(   this->IsInit() || crGap.IsInit()) return FALSE ;
                                                                SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dEdgeT) ;
                                                                return(   crGap.IsKindOf(SmVertexEdgeGap_TYPE)
                                                                       && SmGap::operator==(crGap)
                                                                       && (m_pVertexuse == crGap.GetVertexuse())
                                                                       && (SM_ARE_SAME_TO_TOL(m_dEdgeT, crGap.GetEdgeT(), sScaledZero)) ) ;
                                                              }
  // destructor                                    
  virtual ~SmVertexEdgeGap() { ReSet() ; }

  // from SmGap
  // double       GetLength      ()                                         const { return m_dGap3d ; }
  // SmStatus     GetTopoObjects (SmObject *& rpObj1,  SmObject *& rpObj2)  const ;
  // SmStatus     GetGapEndPoints(SmPoint3d & rPoint1, SmPoint3d & rPoint2) const ;
  // SmXSectTol3d GetXSectTol3d  ()                                         const { SmObject *pObj1, *pObj2 ; 
  //                                                                                GetTopoObjects(pObj1, pObj2) ; 
  //                                                                                return( SmTol::GetXSectTol3d(pObj1, pObj2) ) ; 
  //                                                                              }

  // simple access
  virtual SmVertex    * GetVertex()    const ; 
  virtual SmVertexuse * GetVertexuse() const ;  
  virtual SmEdge      * GetEdge(ULONG lIndx=0)    const ;  
  virtual SmEdgeuse   * GetEdgeuse(ULONG lIndx=0) const ;  
                                  
  virtual double        GetEdgeT(ULONG lIndx=0)   const { return(lIndx == 0 ? m_dEdgeT : SM_UNDEF_DOUBLE) ; } 

  virtual void SetVertexuse(const SmVertexuse * pVertexuse) { m_pVertexuse = pVertexuse ; }
  virtual void Set(const SmVertexuse * pVertexuse, 
                   double              dEdgeT)              { m_pVertexuse = pVertexuse ;
                                                              m_dEdgeT     = dEdgeT ;
                                                              if(m_dEdgeT != SM_UNDEF_DOUBLE) 
                                                                { CacheGap() ; }
                                                              else                            
                                                                { m_dGap3d = -SM_UNDEF_DOUBLE ; } 
                                                            }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmVertexEdgeGap, SmGap, SmVertexEdgeGap_TYPE);

} ; // end class SmVertexEdgeGap

/*******************************************************************//**
PURPOSE: Derived class for Vertex->Point to Face->Surface(Edge->UVTrimCurve(T)) gap

NOTES: 
***********************************************************************/
class SM_EXPORT SmVertexFaceTrimCurveGap : public SmGap
{
  const SmVertexuse * m_pVertexuse = NULL ;    // Vertexuse connecting a vertex to Face through an Edge
                                               //  default:[NULL = uninit]

  double              m_dUVTrimCurveT = SM_UNDEF_DOUBLE ; // Edge->UVTrimCurve parameter bounding gap used as
                                                          //  m_pVertexuse->GetFace->GetSurface->Evaluate
                                                          //     (m_pVertexuse->GetEdge->GetUVTrimCurve->Evaluate
                                                          //        (m_dUVTrimCurveT))) ;
                                                          //  default:[SM_UNDEF_DOUBLE = uninit]
  public:
  // Constructor
  SmVertexFaceTrimCurveGap(const SmVertexuse * pVertexuse   =  NULL, 
                           double              dUVTrimCurveT= SM_UNDEF_DOUBLE) { Set(pVertexuse, dUVTrimCurveT) ; }
                                                         
  // destructor                                          
  virtual ~SmVertexFaceTrimCurveGap()                    { ReSet() ; }      
  virtual SmGap   * MakeCopy() const                     { return new SmVertexFaceTrimCurveGap(*this) ; }
  virtual SmBoolean IsInit() const                       { return( m_pVertexuse != NULL) ; }
  virtual void      ReSet()                              { m_dGap3d        = -SM_UNDEF_DOUBLE ;
                                                           m_pVertexuse    = NULL ;
                                                           m_dUVTrimCurveT = SM_UNDEF_DOUBLE ;
                                                         }
  SmVertexFaceTrimCurveGap & operator=(const SmVertexFaceTrimCurveGap & crGap) { return( (SmVertexFaceTrimCurveGap&)SmVertexFaceTrimCurveGap::operator=((const SmGap &)crGap) ) ; }
  virtual SmGap & operator=(const SmGap & crGap)         {  SmGap::operator=(crGap) ;  
                                                            if(crGap.IsKindOf(SmVertexFaceTrimCurveGap_TYPE)) 
                                                              { m_pVertexuse    = ((SmVertexFaceTrimCurveGap&)crGap).m_pVertexuse ; 
                                                                m_dUVTrimCurveT = ((SmVertexFaceTrimCurveGap&)crGap).m_dUVTrimCurveT ;
                                                              }
                                                            return *this ;
                                                         }
  virtual SmBoolean operator==(const SmGap &crGap) const { if(   this == &crGap
                                                              || (this->IsInit() && crGap.IsInit())) return TRUE ;
                                                           if(   this->IsInit() || crGap.IsInit()) return FALSE ;
                                                           SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dUVTrimCurveT) ;
                                                           return(   crGap.IsKindOf(SmVertexFaceTrimCurveGap_TYPE)
                                                                  && SmGap::operator==(crGap)
                                                                  && (m_pVertexuse == crGap.GetVertexuse())
                                                                  && (SM_ARE_SAME_TO_TOL(m_dUVTrimCurveT, crGap.GetUVTrimCurveT(), sScaledZero)) ) ;
                                                         }
                                                      
  // from SmGap
  // double       GetLength      ()                                         const { return m_dGap3d ; }
  // SmStatus     GetTopoObjects (SmObject *& rpObj1,  SmObject *& rpObj2)  const ;
  // SmStatus     GetGapEndPoints(SmPoint3d & rPoint1, SmPoint3d & rPoint2) const ;
  // SmXSectTol3d GetXSectTol3d  ()                                         const { SmObject *pObj1, *pObj2 ; 
  //                                                                                GetTopoObjects(pObj1, pObj2) ; 
  //                                                                                return( SmTol::GetXSectTol3d(pObj1, pObj2) ) ; 
  //                                                                              }

  // simple access
  virtual SmVertex    * GetVertex()       const ;
  virtual SmVertexuse * GetVertexuse()    const ;
  virtual SmEdgeuse   * GetEdgeuse(ULONG lIndx=0) const ;  
  virtual SmFace      * GetFace()         const ;
                                                
  virtual double        GetUVTrimCurveT() const { return m_dUVTrimCurveT ; }
  virtual SmPoint2d     GetFaceUV()       const ;


  virtual void SetVertexuse(const SmVertexuse * pVertexuse) { m_pVertexuse = pVertexuse ; }
  virtual void Set(const SmVertexuse * pVertexuse, 
                   double              dUVTrimCurveT)       { m_pVertexuse    = pVertexuse ;
                                                              m_dUVTrimCurveT = dUVTrimCurveT ;
                                                              if(   m_pVertexuse
                                                                 && m_dUVTrimCurveT != SM_UNDEF_DOUBLE) 
                                                                { CacheGap() ; }
                                                              else                                      
                                                                { m_dGap3d = -SM_UNDEF_DOUBLE ; } 
                                                            }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmVertexFaceTrimCurveGap, SmGap, SmVertexFaceTrimCurveGap_TYPE);

} ; // end class SmVertexFaceTrimCurveGap

/*******************************************************************//**    
PURPOSE: Derived class for Vertex->Point to Face->Surface(U,V) gaps         
                                                                                                                                                        
NOTES:
***********************************************************************/
class SM_EXPORT SmVertexFaceGap : public SmGap
{
  const SmVertexuse * m_pVertexuse = NULL ; // Vertexuse connecting a vertex to a Face through an Edge
                                            //  default:[NULL = uninit]

  SmPoint2d           m_sFaceUV ;           // Face->Surface parameter bounding Gap used as
                                            //  m_pVertexuse->GetFaceuse->GetFace->GetSurface->Evaluate(m_sFaceUV)
                                            //  default:[SM_UNDEF_DOUBLE = uninit, SM_UNDEF_DOUBLE = uninit]
  public:
  // Construct Vertex/Face 
  SmVertexFaceGap(const SmVertexuse * pVertexuse= NULL, 
                  SmPoint2d         * pFaceUV   = NULL) { Set(pVertexuse, pFaceUV) ; }
  virtual SmGap   * MakeCopy() const                    { return new SmVertexFaceGap(*this) ; }
  virtual SmBoolean IsInit() const                      { return( m_pVertexuse != NULL) ; }
  virtual void      ReSet()                             { m_dGap3d        = -SM_UNDEF_DOUBLE ;
                                                          m_pVertexuse    = NULL ;
                                                          m_sFaceUV.SetUninitialized() ;
                                                        }
  SmVertexFaceGap & operator=(const SmVertexFaceGap & crGap) { return( (SmVertexFaceGap&)SmVertexFaceGap::operator=((const SmGap &)crGap) ) ; }
  virtual SmGap   & operator=(const SmGap & crGap)           {  SmGap::operator=(crGap) ;  
                                                                if(crGap.IsKindOf(SmVertexFaceGap_TYPE)) 
                                                                  { m_pVertexuse = ((SmVertexFaceGap&)crGap).m_pVertexuse ; 
                                                                    m_sFaceUV    = ((SmVertexFaceGap&)crGap).m_sFaceUV ;
                                                                  }
                                                                return *this ;
                                                             }
  virtual SmBoolean operator==(const SmGap &crGap) const     { if(   this == &crGap
                                                                  || (this->IsInit() && crGap.IsInit())) return TRUE ;
                                                               if(   this->IsInit() || crGap.IsInit()) return FALSE ;
                                                               SmScaledZero sScaledZero = SmTol::GetScaledZero(m_sFaceUV) ;
                                                               return(   crGap.IsKindOf(SmVertexFaceGap_TYPE)
                                                                      && SmGap::operator==(crGap)
                                                                      && (m_pVertexuse == crGap.GetVertexuse())
                                                                      && m_sFaceUV.CloserThan(sScaledZero.val, crGap.GetFaceUV())) ;
                                                             }

  // destructor, Makecopy, Reset, assignment operator, equality operator
  virtual ~SmVertexFaceGap()                { ReSet() ; }

  // from SmGap
  // double       GetLength      ()                                         const { return m_dGap3d ; }
  // SmStatus     GetTopoObjects (SmObject *& rpObj1,  SmObject *& rpObj2)  const ;
  // SmStatus     GetGapEndPoints(SmPoint3d & rPoint1, SmPoint3d & rPoint2) const ;
  // SmXSectTol3d GetXSectTol3d  ()                                         const { SmObject *pObj1, *pObj2 ; 
  //                                                                                GetTopoObjects(pObj1, pObj2) ; 
  //                                                                                return( SmTol::GetXSectTol3d(pObj1, pObj2) ) ; 
  //                                                                              }

  // simple access
  virtual SmVertex        * GetVertex()       const ;
  virtual SmVertexuse     * GetVertexuse()    const ;
  virtual SmFace          * GetFace()         const ;
                                                      
  virtual SmPoint2d         GetFaceUV()       const         { return m_sFaceUV ; }           

  virtual void SetVertexuse(const SmVertexuse * pVertexuse) { m_pVertexuse = pVertexuse ; }
  virtual void Set(const SmVertexuse * pVertexuse, 
                   SmPoint2d         * pFaceUV)             { m_pVertexuse = pVertexuse ;
                                                              if(   pVertexuse
                                                                 && pFaceUV
                                                                 && pFaceUV->IsInitialized()) 
                                                                { m_sFaceUV = *pFaceUV ;
                                                                  CacheGap() ;
                                                                }
                                                              else                            
                                                                { m_sFaceUV.SetUninitialized() ;
                                                                  m_dGap3d = -SM_UNDEF_DOUBLE ; 
                                                                }   
                                                            }                           

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmVertexFaceGap, SmGap, SmVertexFaceGap_TYPE);

} ; // end class SmVertexFaceGap

/*******************************************************************//**
PURPOSE: Derived class for Edge0->Curve(T0) to Edge1->Curve(T1) gap

NOTES: For FaceLoop EdgeEnd to EdgeEnd Gaps found within 
***********************************************************************/
class SM_EXPORT SmEdgeEdgeGap : public SmGap
{
  const SmEdgeuse * m_pEdgeuse0 = NULL ; // Edgeuse0 connecting an Edge0End to an Edge1End
                                         //  default:[NULL = uninit]
  const SmEdgeuse * m_pEdgeuse1 = NULL ; // Edgeuse1 connecting an Edge0End to an Edge1End
                                         //  default:[NULL = uninit]

  double        m_dEdgeT0 = SM_UNDEF_DOUBLE ; // Edge0->Curve parameter bounding Gap used as
                                              //  m_pEdgeuse0->GetEdge->Evaluate(m_dEdgeT0) ;
                                              //  default:[SM_UNDEF_DOUBLE = uninit]
  double        m_dEdgeT1 = SM_UNDEF_DOUBLE ; // Edge1->Curve parameter bounding Gap used as
                                              //  m_pEdgeuse1->GetEdge->Evaluate(m_dEdgeT1) ;
                                              //  default:[SM_UNDEF_DOUBLE = uninit]
  public:
  // Constructor 
  SmEdgeEdgeGap(const SmEdgeuse * pEdgeuse0= NULL,
                const SmEdgeuse * pEdgeuse1= NULL,
                double            dEdgeT0  = SM_UNDEF_DOUBLE,
                double            dEdgeT1  = SM_UNDEF_DOUBLE)       { SetEdgeEdgeGap(pEdgeuse0, pEdgeuse1, dEdgeT0, dEdgeT1) ; }
  virtual SmGap   * MakeCopy() const                                { return new SmEdgeEdgeGap(*this) ; }
  virtual SmBoolean IsInit() const                                  { return( m_pEdgeuse0 != NULL && m_pEdgeuse1 != NULL) ; }
  virtual void      ReSet()                                         { m_dGap3d     = -SM_UNDEF_DOUBLE ;
                                                                      m_pEdgeuse0 = NULL ;
                                                                      m_pEdgeuse1 = NULL ;
                                                                      m_dEdgeT0   = SM_UNDEF_DOUBLE ;
                                                                      m_dEdgeT1   = SM_UNDEF_DOUBLE ;
                                                                    }
  SmEdgeEdgeGap & operator=(const SmEdgeEdgeGap & crGap)      { return( (SmEdgeEdgeGap&)SmEdgeEdgeGap::operator=((const SmGap &)crGap) ) ; }
  virtual SmGap & operator=(const SmGap & crGap)              {  SmGap::operator=(crGap) ; 
                                                                 if(crGap.IsKindOf(SmEdgeEdgeGap_TYPE)) 
                                                                   { m_pEdgeuse0 = ((SmEdgeEdgeGap&)crGap).m_pEdgeuse0 ; 
                                                                     m_pEdgeuse1 = ((SmEdgeEdgeGap&)crGap).m_pEdgeuse1 ; 
                                                                     m_dEdgeT0    = ((SmEdgeEdgeGap&)crGap).m_dEdgeT0 ;
                                                                     m_dEdgeT1    = ((SmEdgeEdgeGap&)crGap).m_dEdgeT1 ;
                                                                   }
                                                                 return *this ;
                                                              }
  virtual SmBoolean operator==(const SmGap &crGap) const      { if(   this == &crGap
                                                                   || (this->IsInit() && crGap.IsInit())) return TRUE ;
                                                                if(   this->IsInit() || crGap.IsInit()) return FALSE ;
                                                                SmScaledZero sScaledZero = SmTol::GetScaledZero(smos_Max(m_dEdgeT0,m_dEdgeT1)) ;
                                                                return(   crGap.IsKindOf(SmEdgeEdgeGap_TYPE)
                                                                       && SmGap::operator==(crGap)
                                                                       && m_pEdgeuse0 == ((SmEdgeEdgeGap&)crGap).m_pEdgeuse0  
                                                                       && m_pEdgeuse1 == ((SmEdgeEdgeGap&)crGap).m_pEdgeuse1  
                                                                       && (SM_ARE_SAME_TO_TOL(m_dEdgeT0, crGap.GetEdgeT(0), sScaledZero))
                                                                       && (SM_ARE_SAME_TO_TOL(m_dEdgeT1, crGap.GetEdgeT(1), sScaledZero)) ) ;
                                                              }
  // destructor                                    
  virtual ~SmEdgeEdgeGap() { ReSet() ; }

  // from SmGap
  // double       GetLength      ()                                         const { return m_dGap3d ; }
  // SmStatus     GetTopoObjects (SmObject *& rpObj1,  SmObject *& rpObj2)  const ;
  // SmStatus     GetGapEndPoints(SmPoint3d & rPoint1, SmPoint3d & rPoint2) const ;
  // SmXSectTol3d GetXSectTol3d  ()                                         const { SmObject *pObj1, *pObj2 ; 
  //                                                                                GetTopoObjects(pObj1, pObj2) ; 
  //                                                                                return( SmTol::GetXSectTol3d(pObj1, pObj2) ) ; 
  //                                                                              }

  // simple access
  virtual SmEdge      * GetEdge(ULONG lIndx=0)    const ;  
  virtual SmEdgeuse   * GetEdgeuse(ULONG lIndx=0) const ;  
                                                  
  virtual double        GetEdgeT(ULONG lIndx=0)   const { return(lIndx==0 ? m_dEdgeT0 : m_dEdgeT1) ; } 

  virtual void SetEdgeuse(const SmEdgeuse * pEdgeuse, ULONG lIndx=0) { if(lIndx==0) { m_pEdgeuse0 = pEdgeuse ; }
                                                                       else         { m_pEdgeuse1 = pEdgeuse ; }
                                                                     }
  void SetEdgeEdgeGap(const SmEdgeuse * pEdgeuse0, 
           const SmEdgeuse * pEdgeuse1, 
           double            dEdgeT0,
           double            dEdgeT1)              { m_pEdgeuse0 = pEdgeuse0 ;
                                                             m_pEdgeuse1 = pEdgeuse1 ;
                                                             m_dEdgeT0   = dEdgeT0 ;
                                                             m_dEdgeT1   = dEdgeT1 ;
                                                              if(   m_dEdgeT0 != SM_UNDEF_DOUBLE
                                                                 && m_dEdgeT1 != SM_UNDEF_DOUBLE) 
                                                                { CacheGap() ; }
                                                              else                            
                                                                { m_dGap3d = -SM_UNDEF_DOUBLE ; } 
                                                            }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmEdgeEdgeGap, SmGap, SmEdgeEdgeGap_TYPE);

} ; // end class SmEdgeEdgeGap

/*******************************************************************//**
PURPOSE: Derived class for Edge->Curve(S) to Face->Surface(UVTrimCurve(T)) gaps

NOTES: A gap between an Edge and its projection to a Face as a UVTrimCurve
***********************************************************************/
class SM_EXPORT SmEdgeFaceTrimCurveGap : public SmGap
{
  const SmEdgeuse * m_pEdgeuse = NULL ;  // Edgeuse connecting an Edge to a Face 
                                         //  default:[NULL = uninit]

  double            m_dEdgeT = SM_UNDEF_DOUBLE ; // Edge->Curve parameter bounding Gap used as
                                                 //  Edgeuse->GetEdge->GetCurve->Evaluate(m_dEdgeT)
                                                 //  default:[SM_UNDEF_DOUBLE = uninit]

  double            m_dUVTrimCurveT = SM_UNDEF_DOUBLE ; // Edge->UVTrimCurve parameter bounding Gap used as 
                                                        //  Edgeuse->GetUVTrimCurve->Evaluate(m_dEdgeT)
                                                        //  default:[SM_UNDEF_DOUBLE = uninit]
  public:
  // Constructor
  SmEdgeFaceTrimCurveGap(const SmEdgeuse * pEdgeuse     = NULL,   
                         double            dEdgeT       = SM_UNDEF_DOUBLE, 
                         double            dUVTrimCurveT= SM_UNDEF_DOUBLE) { Set(pEdgeuse, dEdgeT, dUVTrimCurveT) ; }
  virtual SmGap   * MakeCopy() const                     { return new SmEdgeFaceTrimCurveGap(*this) ; }
  virtual SmBoolean IsInit() const                       { return( m_pEdgeuse != NULL) ; }
  virtual void      ReSet()                              { m_dGap3d        = -SM_UNDEF_DOUBLE ;
                                                           m_pEdgeuse      = NULL ;
                                                           m_dEdgeT        = SM_UNDEF_DOUBLE ;
                                                           m_dUVTrimCurveT = SM_UNDEF_DOUBLE ;
                                                         }
  SmEdgeFaceTrimCurveGap & operator=(const SmEdgeFaceTrimCurveGap & crGap) { return( (SmEdgeFaceTrimCurveGap&)SmEdgeFaceTrimCurveGap::operator=((const SmGap &)crGap) ) ; }
  virtual SmGap & operator=(const SmGap & crGap)         {  SmGap::operator=(crGap) ;  
                                                            if(crGap.IsKindOf(SmEdgeFaceTrimCurveGap_TYPE)) 
                                                              { m_pEdgeuse      = ((SmEdgeFaceTrimCurveGap&)crGap).m_pEdgeuse ; 
                                                                m_dEdgeT        = ((SmEdgeFaceTrimCurveGap&)crGap).m_dEdgeT ;
                                                                m_dUVTrimCurveT = ((SmEdgeFaceTrimCurveGap&)crGap).m_dUVTrimCurveT ;
                                                              }
                                                            return *this ;
                                                         }
  virtual SmBoolean operator==(const SmGap &crGap) const { if(   this == &crGap
                                                              || (this->IsInit() && crGap.IsInit())) return TRUE ;
                                                           if(   this->IsInit() || crGap.IsInit()) return FALSE ;
                                                           SmScaledZero sScaledZero0 = SmTol::GetScaledZero(m_dEdgeT) ;
                                                           SmScaledZero sScaledZero1 = SmTol::GetScaledZero(m_dUVTrimCurveT) ;
                                                           return(   crGap.IsKindOf(SmEdgeFaceTrimCurveGap_TYPE)
                                                                  && SmGap::operator==(crGap)
                                                                  && (m_pEdgeuse == crGap.GetEdgeuse())
                                                                  && (SM_ARE_SAME_TO_TOL(m_dEdgeT, crGap.GetEdgeT(), sScaledZero0)) 
                                                                  && (SM_ARE_SAME_TO_TOL(m_dUVTrimCurveT, crGap.GetUVTrimCurveT(), sScaledZero1))) ;
                                                         }

  // destructor
  virtual ~SmEdgeFaceTrimCurveGap()                      { ReSet() ; }
                                                         
  // from SmGap
  // double       GetLength      ()                                         const { return m_dGap3d ; }
  // SmStatus     GetTopoObjects (SmObject *& rpObj1,  SmObject *& rpObj2)  const ;
  // SmStatus     GetGapEndPoints(SmPoint3d & rPoint1, SmPoint3d & rPoint2) const ;
  // SmXSectTol3d GetXSectTol3d  ()                                         const { SmObject *pObj1, *pObj2 ; 
  //                                                                                GetTopoObjects(pObj1, pObj2) ; 
  //                                                                                return( SmTol::GetXSectTol3d(pObj1, pObj2) ) ; 
  //                                                                              }

  // simple access
  virtual SmEdge    * GetEdge(ULONG lIndx=0)    const ;          
  virtual SmEdgeuse * GetEdgeuse(ULONG lIndx=0) const ;          
  virtual SmFace    * GetFace()         const ;          
                                                         
  virtual double      GetEdgeT(ULONG lIndx=0)   const            { return(lIndx==0 ? m_dEdgeT : SM_UNDEF_DOUBLE) ; }
  virtual double      GetUVTrimCurveT() const            { return m_dUVTrimCurveT ; }
  virtual SmPoint2d   GetFaceUV()       const ;
                                                         
  virtual void SetEdgeuse(const SmEdgeuse * pEdgeuse, ULONG lIndx=0) { m_pEdgeuse = pEdgeuse ; SM_REF1(lIndx) ; }
  virtual void Set(const SmEdgeuse * pEdgeuse, 
                   double            dEdgeT, 
                   double            dUVTrimCurveT)      { m_pEdgeuse      = pEdgeuse ;
                                                           m_dEdgeT        = dEdgeT ; 
                                                           m_dUVTrimCurveT = dUVTrimCurveT ;
                                                           if(   m_pEdgeuse 
                                                              && m_dEdgeT        != SM_UNDEF_DOUBLE
                                                              && m_dUVTrimCurveT != SM_UNDEF_DOUBLE) 
                                                             { CacheGap() ; }
                                                           else 
                                                             { m_dGap3d = -SM_UNDEF_DOUBLE ; } 
                                                         }
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmEdgeFaceTrimCurveGap, SmGap, SmEdgeFaceTrimCurveGap_TYPE);

} ; // end class SmEdgeFaceTrimCurveGap

/*******************************************************************//**              
PURPOSE: Derived class for Edge->Curve(T) to Face->Surface(U,V) gaps

NOTES:
***********************************************************************/
class SM_EXPORT SmEdgeFaceGap : public SmGap
{
  const SmEdgeuse   * m_pEdgeuse = NULL ;          // Edgeuse connecting an Edge to a Face 
                                                   //  default:[NULL = uninit]

  double              m_dEdgeT = SM_UNDEF_DOUBLE ; // Edge->Curve parameter bounding Gap used as
                                                   //  Edgeuse->GetEdge->GetCurve->Evaluate(m_dEdgeT)
                                                   //  default:[SM_UNDEF_DOUBLE = uninit]

  SmPoint2d           m_sFaceUV ;                  // Face->Surface parameter bounding Gap used as
                                                   //  m_pVertexuse->GetFaceuse->GetFace->GetSurface->Evaluate(m_sFaceUV)
                                                   //  default:[SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE = uninit]
  public:
  // Construct Edge/Face    gap: SmGapType=SM_GT_EDGE_FACE
  SmEdgeFaceGap(const SmEdgeuse * pEdgeuse= NULL,   
                double      dEdgeT  = SM_UNDEF_DOUBLE, 
                SmPoint2d * pFaceUV = NULL)              { Set(pEdgeuse, dEdgeT, pFaceUV) ; }
  virtual SmGap   * MakeCopy() const                     { return new SmEdgeFaceGap(*this) ; }
  virtual SmBoolean IsInit() const                       { return( m_pEdgeuse != NULL) ; }
  virtual void      ReSet()                              { m_dGap3d   = -SM_UNDEF_DOUBLE ;
                                                           m_pEdgeuse = NULL ;
                                                           m_dEdgeT   = SM_UNDEF_DOUBLE ;
                                                           m_sFaceUV.SetUninitialized() ;
                                                         }
  SmEdgeFaceGap & operator=(const SmEdgeFaceGap & crGap) { return( (SmEdgeFaceGap&)SmEdgeFaceGap::operator=((const SmGap &)crGap) ) ; }
  virtual SmGap & operator=(const SmGap & crGap)         {  SmGap::operator=(crGap) ;  
                                                            if(crGap.IsKindOf(SmEdgeFaceGap_TYPE)) 
                                                              { m_pEdgeuse = ((SmEdgeFaceGap&)crGap).m_pEdgeuse ; 
                                                                m_dEdgeT   = ((SmEdgeFaceGap&)crGap).m_dEdgeT ;
                                                                m_sFaceUV  = ((SmEdgeFaceGap&)crGap).m_sFaceUV ;
                                                              }
                                                            return *this ;
                                                         }
  virtual SmBoolean operator==(const SmGap &crGap) const { if(   this == &crGap
                                                              || (this->IsInit() && crGap.IsInit())) return TRUE ;
                                                           if(   this->IsInit() || crGap.IsInit()) return FALSE ;
                                                           SmScaledZero sScaledZero0 = SmTol::GetScaledZero(m_dEdgeT) ;
                                                           SmScaledZero sScaledZero1 = SmTol::GetScaledZero(m_sFaceUV) ;
                                                           return(   crGap.IsKindOf(SmEdgeFaceGap_TYPE)
                                                                  && this->SmGap::operator==(crGap)
                                                                  && (m_pEdgeuse == crGap.GetEdgeuse())
                                                                  && (SM_ARE_SAME_TO_TOL(m_dEdgeT, crGap.GetEdgeT(), sScaledZero0.val)) 
                                                                  && m_sFaceUV.CloserThan(sScaledZero1.val, crGap.GetFaceUV())) ;
                                                         }

  // destructor
  virtual ~SmEdgeFaceGap()                            { ReSet() ; }

  // from SmGap
  // double       GetLength      ()                                         const { return m_dGap3d ; }
  // SmStatus     GetTopoObjects (SmObject *& rpObj1,  SmObject *& rpObj2)  const ;
  // SmStatus     GetGapEndPoints(SmPoint3d & rPoint1, SmPoint3d & rPoint2) const ;
  // SmXSectTol3d GetXSectTol3d  ()                                         const { SmObject *pObj1, *pObj2 ; 
  //                                                                                GetTopoObjects(pObj1, pObj2) ; 
  //                                                                                return( SmTol::GetXSectTol3d(pObj1, pObj2) ) ; 
  //                                                                              }

  // simple access
  virtual SmEdge          * GetEdge(ULONG lIndx=0)    const ;
  virtual SmEdgeuse       * GetEdgeuse(ULONG lIndx=0) const ;
  virtual SmFace          * GetFace()         const ;
                                                      
  virtual double            GetEdgeT(ULONG lIndx=0)   const   { return(lIndx==0 ? m_dEdgeT : SM_UNDEF_DOUBLE) ; }
  virtual SmPoint2d         GetFaceUV()       const   { return m_sFaceUV ; }           

  virtual void SetEdgeuse(const SmEdgeuse * pEdgeuse, ULONG lIndx=0) { m_pEdgeuse = pEdgeuse ; SM_REF1(lIndx) ; }
  virtual void Set(const SmEdgeuse * pEdgeuse, 
                   double            dEdgeT, 
                   SmPoint2d       * pFaceUV)         { m_pEdgeuse = pEdgeuse ;
                                                        m_dEdgeT   = dEdgeT ; 
                                                        if(   pEdgeuse
                                                           && dEdgeT != SM_UNDEF_DOUBLE
                                                           && pFaceUV
                                                           && pFaceUV->IsInitialized()) 
                                                          { m_sFaceUV = *pFaceUV ;
                                                            CacheGap() ;
                                                          }
                                                        else
                                                          { m_sFaceUV.SetUninitialized() ;
                                                            m_dGap3d = -SM_UNDEF_DOUBLE ; 
                                                          }
                                                      }
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmEdgeFaceGap, SmGap, SmEdgeFaceGap_TYPE);

} ; // end class SmEdgeFaceGap

/*******************************************************************//**
PURPOSE: class to store and report
            a set of topology Object gaps

NOTES: Gap Arrays contain any mix of the SmGap Hierarchy
       including any mix of objects of type: 
          SmVertexFaceGap
          SmVertexEdgeGap
          SmVertexFaceTrimCurveGap
          SmEdgeFaceTrimCurveGap
          SmEdgeFaceGap

  GapArrays own the objects they contain and will delete them when destructed.
  Objects added to GapArrays are copied before being stored.
  Objects removed from GapArrays are deleted.
***********************************************************************/
class SM_EXPORT SmGapArray
{
 protected:
  ULONG              m_lMaxGap3d ;  // max gap index of all gaps in m_sGaps, init to SM_BIG_ULONG
  SmTArray<SmGap *>  m_sGaps ;      // list of gaps (must be SmGap ptrs, not objects. SmGap is a dynamic class with virtual methods)
                                    // SmGaps are pointers to SmGap objects stored on the SmEdgeuse, SmVertexuse and SmLoopuse classes.

 public:
   // empty constructor
   SmGapArray()                            { m_lMaxGap3d   = SM_BIG_ULONG ; }

  SmGap * operator[](ULONG ii) const       { return( m_sGaps[ii] ) ; }   // no copy
  SmGap * GetAt(ULONG ii)      const       { return( m_sGaps[ii] ) ; }   // no copy
                                           
   // copy construtor - Deep copy - all SmGaps get copied
   SmGapArray(const SmGapArray &crGaps)    { m_lMaxGap3d   = crGaps.m_lMaxGap3d ;
                                                Append( (SmGapArray &)crGaps ) ; 
                                           }
   // destructor                           
   ~SmGapArray()                           { ReSet() ; }
                                          
   // ReSet m_sGaps.m_lSize back to zero, deletes all contained SmGap objects     
   void ReSet()                               { m_sGaps.ReSet() ; 
                                             m_lMaxGap3d = SM_BIG_ULONG ;
                                           }
   // assignment - deep copy gaps                       
   SmGapArray & operator=(SmGapArray &crGaps) { if(this == &crGaps) { return *this ; }
                                                ReSet() ;
                                                Append( crGaps ) ; 
                                                m_lMaxGap3d = crGaps.m_lMaxGap3d ;
                                                return *this ;
                                              }

  // simple Access
  const SmGap * GetMaxGap()       const    { return( m_lMaxGap3d == SM_BIG_ULONG ? NULL : m_sGaps[m_lMaxGap3d] ) ; }
  const double  GetMaxGapLength() const    { return( m_lMaxGap3d == SM_BIG_ULONG ? 0.0  : m_sGaps[m_lMaxGap3d]->GetLength() ) ; }
  ULONG         GetSize()         const    { return( m_sGaps.GetSize() ) ; }

  // Add SmGap copy to list 
  SmGap * Add(SmGap *cpGap)                   { if(   m_lMaxGap3d == SM_BIG_ULONG
                                                   || cpGap->GetLength() > m_sGaps[m_lMaxGap3d]->GetLength()) 
                                              { m_lMaxGap3d = m_sGaps.GetSize() ; }
                                                m_sGaps.Add(cpGap) ;
                                                return(cpGap) ;
                                              }
  
  void    Append(SmGapArray & rGapArray)      { for(ULONG ii=0;ii<rGapArray.GetSize();ii++)
                                                  { m_sGaps.Add(rGapArray.m_sGaps[ii]) ; }
                                           }

  void    Remove(ULONG lIndex)             { if(lIndex < m_sGaps.GetSize())
                                                  { m_sGaps.RemoveAt(lIndex,1) ;
                                               }
                                           }                                             
  // find elements by matches
  SmBoolean Find(const SmGap & crGap,  // in : target gap to match
                 ULONG       & lIndex) // out: found index
                const ;

  // 
  SmStatus  GetVerticesAndEdges(SmTArray<SmVertex *>  & rVertices,  // increments unlocked mark
                                 SmTArray<SmEdge *>    & rEdges,
                                SmTArray<SmEdgeuse *> & rEdgeuses) 
                               const ;
  // maintenance
  SmDisplayList *Draw(SmGfxArraySet * pOptGfxSet=NULL) const ;
  void           Dump() const ;

  const TCHAR * GetTypeString() const { return _T("SmGapArray") ; }

} ; // end class SmGapArray

/*******************************************************************//**
PURPOSE: One Gap Sample geometry

NOTES: includes GapDropType, 
                tolerance, 
                3d gap end points, and 
                associated gap end shape parameter points.

 static class object - don't add virtual methods to SmGapSample.  
 Virtual methods conflict with the methods in SmTArray<SmGapSample>
 that use memset() to clear memory - with SmGapSample virtual methods the
 virtual pointer tables get corrupted by SmTArray<SmGapSample>::ReSet() 
 calls and the like.
***********************************************************************/
class SM_EXPORT SmGapSample
{
 protected :
  SmGapDropType  m_eGapDropType = SM_GD_UNKNOWN ;  // oneof: SM_GD_UNKNOWN  - uninitialized value
                                                   //        SM_GD_NORMAL   - Gap runs along other geom Normal vector to other geometry point
                                                   //        SM_GD_BOUNDARY - Gap runs to nearest other geom boundary point
                                                   //        SM_GD_NO_DROP  - No Gap was found from this Geometry point to other geometry

  SmPoint3d      m_sThisPos  ;          // Gap start position- On the gapFunction from object
  SmPoint3d      m_sOtherPos ;          // Gap end   position- On the gapFunction other object
                                        //   note: gap is perp to from object's tangents at the start position.
  SmPoint3d      m_sTolPoint ;          // when m_bIsInTol == FALSE, the point along the gapVector that is m_sXSectTol3d from the ThisPos
                 
  SmPoint2d      m_sThisParam ;         // Gap parameter on the start object (only use x for curves, x and y for surfaces)
  SmPoint2d      m_sOtherParam ;        // Gap parameter on the end object   (only use x for curves, x and y for surfaces)
                
  SmXSectTol3d   m_sXSectTol3d ;        // Tolerance bound
  SmBoolean      m_bIsInTol = TRUE ;    // TRUE = gap length <= dXSectTol3d
                                        // FALSE= gap length >  dXSectTol3d
 public :

  // constructors
  SmGapSample(SmXSectTol3d sXSectTol3d=SM_XSECT_TOL_3D) : m_eGapDropType(SM_GD_UNKNOWN),  
                                                  m_sThisPos(),
                                                  m_sOtherPos(),
                                                  m_sTolPoint(),  
                                                  m_sThisParam(),
                                                  m_sOtherParam(), 
                                                  m_sXSectTol3d(sXSectTol3d),   
                                                  m_bIsInTol(FALSE)
                                                { }

  SmGapSample(SmPoint3d   & crThisPos,          // in : Point on From Geometry
              double        dThisU,             // in : 1st param of FromPoint for Geometry - used for Curves and Surfaces
              double        dThisV,             // in : 2nd param of FromPoint for Geometry - only used for Surfaces
              SmPoint3d   & crOtherPos,         // in : Point on To Geometry
              double        dOtherU,            // in : 1st param of ToPoint for Geometry - only used for Curves and Surfaces
              double        dOtherV,            // in : 2nd param of ToPoint for Geometry - only used for Surfaces
              SmGapDropType eGapDropType,       // in : one of SM_GD_NORMAL   - Gap runs along other geom Normal vector to other geometry point
                                                //             SM_GD_BOUNDARY - Gap runs to nearest other geom boundary point
                                                //             SM_GD_NO_DROP  - No Gap was found from this Geometry point to other geometry
              SmXSectTol3d  sXSectTol3d=SM_XSECT_TOL_3D) // in : Tol used to judge IsInTol gaps (also used for graphics)
                                                : m_eGapDropType(eGapDropType), 
                                                  m_sThisPos(crThisPos),  
                                                  m_sOtherPos(crOtherPos),
                                                  m_sThisParam(dThisU, dThisV), 
                                                  m_sOtherParam(dOtherU, dOtherV), 
                                                  m_sXSectTol3d(sXSectTol3d)
                                                { // calc and save m_bIsInTol and m_sTolPoint
                                                  SetTolPoint() ;
                                                }

  // copy constructor
  SmGapSample(const SmGapSample &crOther)       : m_eGapDropType(crOther.m_eGapDropType), 
                                                  m_sThisPos(crOther.m_sThisPos),     
                                                  m_sOtherPos(crOther.m_sOtherPos), 
                                                  m_sTolPoint(),  
                                                  m_sThisParam(crOther.m_sThisParam), 
                                                  m_sOtherParam(crOther.m_sOtherParam),      
                                                  m_sXSectTol3d(crOther.m_sXSectTol3d), 
                                                  m_bIsInTol(crOther.m_bIsInTol)
                                                { if(!m_bIsInTol) { m_sTolPoint = crOther.m_sTolPoint ; }
                                                }
  // operators
  SmGapSample & operator= (const SmGapSample &crOther) ;
  SmBoolean     operator==(const SmGapSample &crOther) const;

  // destructor
  ~SmGapSample() { }
  void SetUninitialized()  { m_eGapDropType = SM_GD_UNKNOWN ; 
                             m_sThisPos .SetUninitialized() ;  
                             m_sOtherPos.SetUninitialized() ;  
                             m_sTolPoint.SetUninitialized() ;  
                             m_sThisParam .SetUninitialized() ;  
                             m_sOtherParam.SetUninitialized() ;  
                             m_sXSectTol3d = SM_UNDEF_DOUBLE ;  
                             m_bIsInTol    = FALSE ;    
                           }

  // simple access
#ifdef SM_USE_OLDTOL
  // obsolete name - scheduled for removal
  SM_OLDTOL_LINE double         GetTol()           const       { return GetXSectTol3d() ; }
  SM_OLDTOL_LINE void           SetTol(double dXSectTol3d)     { SetXSectTol3d(dXSectTol3d) ; }
#endif // SM_USE_OLDTOL

  // simple data access
  SmXSectTol3d   GetXSectTol3d()    const       { return m_sXSectTol3d ; } 
  SmPoint3d    & GetThisPos()                   { return m_sThisPos ; }
  SmPoint3d    & GetOtherPos()                  { return m_sOtherPos ; }
  SmPoint3d    & GetTolPoint()                  { return m_sTolPoint ; }
  SmPoint2d    & GetThisParam()                 { return m_sThisParam ; }
  SmPoint2d    & GetOtherParam()                { return m_sOtherParam ; }
  double         GetThisParam(ULONG lIndx)  const { return(lIndx == 0 ? m_sThisParam.x : m_sThisParam.y) ; }
  double         GetOtherParam(ULONG lIndx) const { return(lIndx == 0 ? m_sOtherParam.x : m_sOtherParam.y) ; }
  double         GetLength()        const       { return m_sThisPos.DistanceBetween(m_sOtherPos) ; }
  double         GetLengthSquared() const       { return m_sThisPos.DistanceBetweenSquared(m_sOtherPos) ; }
  SmGapDropType  GetGapDropType()   const       { return m_eGapDropType ; }
  
  // simple data set                                              
  void SetCanonical(SmPoint3d   & rThisPos,     // in : this side Gap End Point
                    double        dThisU,       // in : this object ParamU (if any, use 0.0 for none)
                    double        dThisV,       // in : this object ParamV (if any, use 0.0 for none)
                    SmPoint3d   & rOtherPos,    // in : other side Gap End Point
                    double        dOtherU,      // in : other object ParamU (if any, use 0.0 for none)
                    double        dOtherV,      // in : other object ParamV (if any, use 0.0 for none)
                    SmGapDropType eGapDropType, // in : oneof: SM_GD_NORMAL   - Gap runs along other geom Normal vector to other geometry point
                                                //             SM_GD_BOUNDARY - Gap runs to nearest other geom boundary point
                                                //             SM_GD_NO_DROP  - No Gap was found from this Geometry point to other geometry
                    SmXSectTol3d  sXSectTol3d)  // in : XSectTol3d value
                                                {  m_eGapDropType  = eGapDropType ;
                                                   m_sThisPos      =  rThisPos ;       
                                                   m_sOtherPos     =  rOtherPos ;      
                                                   m_sThisParam.Set (dThisU,  dThisV) ;    
                                                   m_sOtherParam.Set(dOtherU, dOtherV) ; 
                                                   m_sXSectTol3d   = sXSectTol3d ; 
                                                   m_bIsInTol      = FALSE ;
                                                   SetTolPoint() ;
                                                }
  void SetGapDropType(SmGapDropType eDropType)  // oneof: SM_GD_UNKNOWN  - uninitialized value
                                                //        SM_GD_NORMAL   - Gap runs along other geom Normal vector to other geometry point
                                                //        SM_GD_BOUNDARY - Gap runs to nearest other geom boundary point
                                                //        SM_GD_NO_DROP  - No Gap was found from this Geometry point to other geometry
                                                { m_eGapDropType = eDropType ; }
  void SetXSectTol3d(SmXSectTol3d sXSectTol3d)  { if(m_sXSectTol3d != sXSectTol3d)
                                                    { m_sXSectTol3d = sXSectTol3d ;
                                                      SetTolPoint() ; 
                                                }   }
  void SetThisParam(double dT)                  { m_sThisParam.x  = dT ; m_sThisParam.y  = 0.0 ; }
  void SetThisParam(double dU, double dV)       { m_sThisParam.x  = dU ; m_sThisParam.y  = dV ; }
  void SetOtherParam(double dT)                 { m_sOtherParam.x = dT ; m_sOtherParam.y = 0.0 ; }
  void SetOtherParam(double dU, double dV)      { m_sOtherParam.x = dU ; m_sOtherParam.y = dV ; } 
  void SetThisPos(const SmPoint3d &crThisPos)   { if(m_sThisPos  != crThisPos)
                                                    { m_sThisPos  = crThisPos ;
                                                      SetTolPoint() ; 
                                                }   }
  void SetOtherPos(const SmPoint3d &crOtherPos) { if(m_sOtherPos != crOtherPos)
                                                    { m_sOtherPos = crOtherPos ;
                                                      SetTolPoint() ; 
                                                }   }
  // predicates
  SmBoolean IsInTol()         const             { return m_bIsInTol ;  }

  // internal
 protected:                        
  void SetTolPoint() ;

  // maintenance
 public:
  void            Dump(SmGapSampleType eType=SM_GS_SRF_SRF, SmBoolean bAbbrev=FALSE, ULONG *pOptLabel=NULL) const ;
  SmDisplayList * Draw(SmGfxArraySet * pOptGfxSet=NULL) const ;

} ; // end class SmGapSample

// add a SmTArray<SmGapSample> template to the dll interface
SM_TARRAY_TEMPLATE_PREDECLARATION(SmGapSample) ;

/*******************************************************************//**
PURPOSE: One Local interval marking the boundary points where
  one shape is within a specified tolerance of another shape.

NOTES: 
 static class object - don't add virtual methods to SmLocalInterval.  
 Virtual methods conflict with the methods in SmTArray<SmLocalInterval>
 that use memset() to clear memory - with SmLocalInterval virtual methods the
 virtual pointer tables get corrupted by SmTArray<SmLocalInterval>::ReSet() 
 calls and the like.

***********************************************************************/
class SM_EXPORT SmLocalInterval
{
 protected :
  SmGapSample m_sMin ;            // Interval min ThisParam, OtherParam, ThisPos, OtherPos, Tol, TolPoint, and bIsInTol values
  SmGapSample m_sMax ;            // Interval max ThisParam, OtherParam, ThisPos, OtherPos, Tol, TolPoint, and bIsInTol values
                                      
 public :

  // constructor
  SmLocalInterval(SmGapSample &rMin,
                  SmGapSample &rMax)            : m_sMin(rMin),
                                                  m_sMax(rMax)
                                                { }
  // copy constructor
  SmLocalInterval(const SmLocalInterval &crOther)   : m_sMin(crOther.m_sMin), 
                                                      m_sMax(crOther.m_sMax)
                                                    { }

  // empty constructor - intended for system use only
  SmLocalInterval()  { }

  // operators
  SmLocalInterval & operator= (const SmLocalInterval &crOther) { if(this == &crOther) return *this ;
                                                                 m_sMin = crOther.m_sMin ;
                                                                 m_sMax = crOther.m_sMax ;
                                                                 return *this ;
                                                               } 
  SmBoolean operator==(const SmLocalInterval &crOther) const { return(   m_sMin == crOther.m_sMin
                                                                      && m_sMax == crOther.m_sMax) ;
                                                             }
  // destructor
 ~SmLocalInterval() { }

  // simple access
  const SmGapSample &GetMin() const { return(m_sMin) ; }
  const SmGapSample &GetMax() const { return(m_sMax) ; }

  // convenience routines
  SmExtent1d GetThisUInterval()      { return(SmExtent1d(m_sMin.GetThisParam().x,  m_sMax.GetThisParam().x)) ; }
  SmExtent1d GetThisVInterval()      { return(SmExtent1d(m_sMin.GetThisParam().y,  m_sMax.GetThisParam().y)) ; }
  SmExtent1d GetOtherUInterval()     { return(SmExtent1d(m_sMin.GetOtherParam().x, m_sMax.GetOtherParam().x)) ; }
  SmExtent1d GetOtherVInterval()     { return(SmExtent1d(m_sMin.GetOtherParam().y, m_sMax.GetOtherParam().y)) ; }
  const SmPoint3d &GetThisMinPoint()   { return( m_sMin.GetThisPos()) ; }
  const SmPoint3d &GetThisMaxPoint()   { return( m_sMax.GetThisPos()) ; }
  const SmPoint3d &GetOtherMinPoint()  { return( m_sMin.GetOtherPos()) ; }
  const SmPoint3d &GetOtherMaxPoint()  { return( m_sMax.GetOtherPos()) ; }

  // maintenance
 public:
  void            Dump(SmBoolean bAbbrev=FALSE, ULONG *pOptLabel=NULL) const ;
  SmDisplayList * Draw(const SmCurve * pCurve=NULL,
                       SmGfxArraySet * pOptGfxSet=NULL) const ;

} ; // end class SmLocalInterval

// add a SmTArray<SmLocalInterval> template to the dll interface
SM_TARRAY_TEMPLATE_PREDECLARATION(SmLocalInterval) ;

/*******************************************************************//**
PURPOSE: class to represent the gap function between a pair of shapes
            moving from ThisShape to OtherShape.

NOTES: Between any two shapes there are two gap functions, 
     GapFunction(A,B) and GapFunction(B,A) which are usually different.
     
     This class represents only one gap function running from the ThisShape
     to the OtherShape.  Represent both gap functions between a single
     pair of shape objects with two SmGapFunction objects interchanging the
     this and other shape objects.
***********************************************************************/
class SM_EXPORT SmGapFunction
{
 protected:
  SmGapSampleType       m_eSampleType ;             // one of SM_GS_CRV_PT,   // sample from Curve to Point    
                                                    //        SM_GS_CRV_CRV,  // sample from Curve to Curve    
                                                    //        SM_GS_CRV_SRF,  // sample from Curve to Surface  
                                                    //        SM_GS_SRF_PT,   // sample from Surface to Point  
                                                    //        SM_GS_SRF_CRV,  // sample from Surface to Curve  
                                                    //        SM_GS_SRF_SRF   // sample from Surface to Surface
                                                    
  SmXSectTol3d          m_sXSectTol3d ;             // max dist between intersecting points
                                                    
  SmExtent1d            m_sSampleIvlU ;             // 1st param domain range for This Object samples: used for ThisCurves and ThisSurfaces
  SmExtent1d            m_sSampleIvlV ;             // 2nd param domain range for This Object samples: only used for ThisSurfaces
                                                    //    for curves - this will be set to [0 1]
  // lazy evaluation state                                                  
  SmBoolean             m_bAreSamplesSet ;          // TRUE  = m_sSampleGaps computed
                                                    // FALSE = m_sSampleGaps not computed
                                                    // UNSURE= some computed until an out-of-tol gap was found

  SmBoolean             m_bIsLocalNeighborhoodSet ; // LocalNeighborhoods currently defined only for ToGeom_TYPE == Curve
                                                    //   stored as SmCurveClassification contained in the Derived class 
                                                    // TRUE  = derived m_sLocalNeighborhood is     set - don't run BuildLocalNeighborhood()
                                                    // FALSE = derived m_sLocalNeighborhood is not set - do    run BuildLocalNeighborhood()

  // sample size
  ULONG                 m_lSampleCntU ;             // number of samples in the first dimension
  ULONG                 m_lSampleCntV ;             // number of samples in 2nd dimension (will be set to 1 for ThisCurve type GapFunctions)
  SmTArray<SmGapSample> m_sSampleGaps ;             // Array of gap samples stored in row major order
                                                    //   ordered:[00 01 02 10 11 12 20 21 22] for a 3x3 sample set

  // sample statistics
  ULONG                 m_lMaxSampleIndex ;         // index of max sample size in Sample array
  ULONG                 m_lMinSampleIndex ;         // index of min sample size in Sample array
  SmBoolean             m_bAllNormalGaps ;          // TRUE = all gaps are type SM_GD_NORMAL
  SmExtent1d            m_sOtherIvlU ;              // U Min/Max drop domain of all sampled points, used for curves and surfaces
  SmExtent1d            m_sOtherIvlV ;              // V Min/Max drop domain of all sampled points, used for surfaces
                                                    //   when unused set to UnInit().

  // constructor - build these with the derived classes
  SmGapFunction
  (
    SmGapSampleType      eType,           // in : Type so derived classes can mark their type here - yuk!
    const SmXSectTol3d & rXSectTol3d,     // in : Tol used to judge in/out property for a gap length.
    const SmExtent1d   & rSampleIvlU,     // in : 1st param domain range - used for Curves and Surfaces
    const SmExtent1d   & rSampleIvlV,     // in : 2nd param domain range - only used for Surfaces
    ULONG                lSampleCntU =20, // in : number of evenly spaced samples in 1st param dim - used for Curves and Surfaces      
    ULONG                lSampleCntV =1   // in : number of evenly spaced samples in 2nd param dim - only used for Surfaces  
  );

  // Copy Constructor from crOther to this                                  
  SmGapFunction(const SmGapFunction &crOther) ; 

  // virtual deep copy operator from this to rpNewGapFunction
  virtual SmStatus Copy(SmGapFunction  *& rpNewGapFunction) const { // should always be called by a derived class
                                                                    SM_REF1(rpNewGapFunction) ;
                                                                    ERR(SM_ERR) ; 
                                                                    return SM_ERR ;
                                                                  }
  SmGapFunction   & operator= (const SmGapFunction &crOther) ;
  virtual SmBoolean operator==(const SmGapFunction &crOther) const;

 public:
  virtual ~SmGapFunction()                   { m_bAreSamplesSet          = FALSE ;
                                               m_bIsLocalNeighborhoodSet = FALSE ;
                                               m_sSampleGaps.RemoveAll() ; 
                                             }
  // simple access
#ifdef SM_USE_OLDTOL
  // obsolete names - scheduled for removal
  SM_OLDTOL_LINE double  GetTol()                     { return GetXSectTol3d() ; }
  SM_OLDTOL_LINE void    SetTol(double dXSectTol3d)   { SetXSectTol3d(dXSectTol3d) ; }
#endif // SM_USE_OLDTOL

  SmXSectTol3d    GetXSectTol3d()               { return m_sXSectTol3d ; }
  SmGapSample   & GetGap(ULONG ii, ULONG jj=0); 
  ULONG           GetSampleCntU() const         { return m_lSampleCntU ; }
  ULONG           GetSampleCntV() const         { return m_lSampleCntV ; }
  ULONG           GetSize()       const         { return m_lSampleCntU * m_lSampleCntV ; }
  SmGapSampleType GetSampleType() const         { return m_eSampleType ; } // rtn: one of SM_GS_CRV_PT  SM_GS_SRF_PT
                                                                           //             SM_GS_CRV_CRV SM_GS_SRF_CRV
                                                                           //             SM_GS_CRV_SRF SM_GS_SRF_SRF
  void SetXSectTol3d(SmXSectTol3d sXSectTol3d) { if(sXSectTol3d != m_sXSectTol3d) 
                                                   { m_sXSectTol3d             = sXSectTol3d ;
                                                     m_bAreSamplesSet          = FALSE ;
                                                     m_bIsLocalNeighborhoodSet = FALSE ; 
                                               }   }
  void SetSampleCnt(ULONG lSampleCntU=20,
                    ULONG lSampleCntV=1) ;

  SmStatus                        GetMaxGapSample( SmGapSample *& prGapSample, SmBoolean bStopAfterBigGap = FALSE ) { ULONG indx; prGapSample = NULL;
                                                                                                                    SER( GetMaxIndexSample( bStopAfterBigGap, indx ) );
                                                                                                                    prGapSample = &m_sSampleGaps[indx]; 
                                                                                                                    return SM_SUCCESS; }
  SmStatus                        GetMinGapSample( SmGapSample *& prGapSample, SmBoolean bStopAfterBigGap = FALSE ) { ULONG indx; prGapSample = NULL;
                                                                                                                    SER( GetMinIndexSample( bStopAfterBigGap, indx ) ); 
                                                                                                                    prGapSample = &m_sSampleGaps[indx]; 
                                                                                                                    return SM_SUCCESS; }
  SmStatus                        GetMaxIndexSample( SmBoolean bStopAfterBigGap, ULONG & rlMaxIndx ) { SER( BuildSampleSet( bStopAfterBigGap ) );
                                                                                                       rlMaxIndx = m_lMaxSampleIndex;  
                                                                                                       return( SM_SUCCESS ); }
  SmStatus                        GetMinIndexSample( SmBoolean bStopAfterBigGap, ULONG & rlMinIndx ) { SER( BuildSampleSet( bStopAfterBigGap ) );
                                                                                                       rlMinIndx = m_lMinSampleIndex;
                                                                                                       return( SM_SUCCESS ); }

  virtual const SmObject        * GetThisObject()   const   { return NULL ; }
  virtual const SmObject        * GetOtherObject()  const   { return NULL ; }
  virtual const SmCurve         * GetThisCurve()    const   { return NULL ; }
  virtual const SmCurve         * GetOtherCurve()   const   { return NULL ; }

  const SmExtent1d              * GetThisIvlU()     const   { return(&m_sSampleIvlU) ; }
  const SmExtent1d              * GetThisIvlV()     const   { return(&m_sSampleIvlV) ; }

  virtual const SmSurface       * GetThisSurface()  const   { return NULL ; }
  virtual const SmSurface       * GetOtherSurface() const   { return NULL ; }

  virtual const SmPoint3d       * GetOtherPoint()   const   { return NULL ; }

  SmTArray<SmGapSample>         & GetSampleSet(SmBoolean bStopAfterBigGap=FALSE)   { BuildSampleSet(bStopAfterBigGap) ; return m_sSampleGaps ; }
  void                            GetLocalIntervals(SmTArray<SmLocalInterval> &rLocalIvls) const ;
  virtual const SmCurveClassification *GetAsIsLocalNeighborhood() const       { return NULL ; }
  virtual SmCurveClassification * GetLocalNeighborhood(SmBoolean bAsIs=FALSE) { SM_REF1(bAsIs) ; return NULL ; }
  SmBoolean                       GetAllNormalGaps() const  { return m_bAllNormalGaps ; }
  const SmExtent1d              * GetOtherIvlU()     const   { return(&m_sOtherIvlU) ; }
  const SmExtent1d              * GetOtherIvlV()     const   { return(&m_sOtherIvlV) ; }
                                              
  // predicates
  SmBoolean AreSamplesSet()                                 const   { return m_bAreSamplesSet ; }
  SmBoolean IsLocalNeighborhoodSet()                        const   { return m_bIsLocalNeighborhoodSet ; }
  SmBoolean IsInTol()                                               { SmGapSample *pMaxGap;
                                                                      GetMaxGapSample( pMaxGap, TRUE );
                                                                      return ( pMaxGap ) ? pMaxGap->GetLengthSquared() > m_sXSectTol3d*m_sXSectTol3d : FALSE; }
  SmBoolean AreGapsNormal(SmBoolean bStopAfterBigGap=FALSE) const   {((SmGapFunction*)this)->BuildSampleSet(bStopAfterBigGap) ; return(m_bAllNormalGaps) ; }

  // methods
  virtual SmStatus Evaluate(double       dThisU,                 // in : This Geom 1st param used for Curves and Surfaces
                            double       dThisV,                 // in : This Geom 2nd param only used for Surfaces
                            SmGapSample &rGapSample,             // out: Eval holder block
                            double      *pOptOtherGuessU=NULL,   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
                            double      *pOptOtherGuessV=NULL)   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
                                                                 { SM_REF5(dThisU, dThisV, rGapSample, pOptOtherGuessU, pOptOtherGuessV) ; 
                                                                   SE(SM_ERR) ; return SM_ERR ; 
                                                                 }

  // internal
 protected:
          SmStatus BuildSampleSet(SmBoolean bStopAfterBigGap=FALSE) ;
          SmStatus BuildLocalNeighborhood() ;
  virtual SmStatus EvaluateNeighborhoodBoundaries(SmSolutionArray &rSolutions) { rSolutions.ReSet() ; return(SM_SUCCESS) ; }

#ifdef SM_DEBUG_CODE
          SmStatus BuildSampleSet_OldDebug(SmBoolean bStopAfterBigGap=FALSE) ;
#endif // SM_DEBUG_CODE

  // maintenance
 public:
  virtual void            Dump(SmBoolean bAbbrev) const ;

  virtual SmDisplayList * Draw(SmBoolean       bShowGeometry = TRUE,      // NotUsed: in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
                               SmBoolean       bShowNeighborhoods=TRUE,   // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
                               SmBoolean       bShowSamples=FALSE,        // in : TRUE  = show all sample gaps, 
                                                                          //      UNSURE= Only show bigger-than-tol gaps, 
                                                                          //      FALSE = Show no sample gaps,
                                                                          //      default:[FALSE]
                               double          dDisplayScale=1000.0,      // in : Amount micro geometry is scaled for visualization, default:[1000]
                               SmGfxArraySet * pOptGfxSet=NULL)           // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                                                          //      NULL to ignore. default:[NULL]
                              const;

  SM_COMMON_BASE(SmGapFunction, SmGapFunction_TYPE) ; 

} ; // end class SmGapFunction

/*******************************************************************//**
PURPOSE: class to represent the gap function from a Curve to a Point

NOTES: 
***********************************************************************/
class SM_EXPORT SmCrvPtGapFunction : public SmGapFunction
{
 protected:
  const SmCurve       * m_pThisCurve ;
  SmPoint3d             m_sOtherPoint ;
  SmCurveClassification m_sLocalNeighborhood ;   // FromCurve entries and exits into the ToPoint LocalShape

 public:
  // constructor
  SmCrvPtGapFunction(SmXSectTol3d  & rXSectTol3d,
                     const SmCurve * pThisCurve,
                     SmExtent1d    & rThisIvl,
                     SmPoint3d     * pOtherPoint,
                     ULONG          lSampleCntU=20) : SmGapFunction(SM_GS_CRV_PT, 
                                                                    rXSectTol3d,
                                                                    rThisIvl, 
                                                                    rThisIvl,
                                                                    lSampleCntU, 
                                                                    1),
                                                      m_pThisCurve(pThisCurve),
                                                      m_sOtherPoint(*pOtherPoint),
                                                      // gwc: passing a XSectTol as a ZoneTol3d - needs fix
                                                      m_sLocalNeighborhood(pThisCurve, rThisIvl, NULL, (SmZoneTol3d &)(rXSectTol3d))
                                                    { m_sSampleIvlV.SetMinMax(0.0,1.0) ; }

  // Copy Constructor from crOther to this                                    
  SmCrvPtGapFunction(const SmCrvPtGapFunction &crOther) : SmGapFunction(crOther),
                                                          m_pThisCurve(crOther.m_pThisCurve),
                                                          m_sOtherPoint(crOther.m_sOtherPoint),
                                                          m_sLocalNeighborhood(crOther.m_sLocalNeighborhood)
                                                        { } 

  // virtual deep copy operator from this to rpNewGapFunction
  virtual SmStatus Copy(SmGapFunction  *& rpNewGapFunction) const { rpNewGapFunction = new SmCrvPtGapFunction(*this); NER(rpNewGapFunction) ;
                                                                    return SM_SUCCESS ;
                                                                  }
  SmGapFunction  & operator= (const SmCrvPtGapFunction &crOther)  { if(this == &crOther) return(*this) ;
                                                                    SmGapFunction::operator=(crOther) ;
                                                                    m_pThisCurve         = crOther. m_pThisCurve ;
                                                                    m_sOtherPoint        = crOther. m_sOtherPoint ;
                                                                    m_sLocalNeighborhood = crOther. m_sLocalNeighborhood ;
                                                                    return(*this) ;
                                                                  }
  virtual SmBoolean operator==(const SmGapFunction &crOther) const { if(this == &crOther) return(TRUE) ;
                                                                     if(!crOther.IsKindOf(SmCrvPtGapFunction_TYPE)) { return FALSE ; }
                                                                     SmBoolean bRtn = SmGapFunction::operator ==(crOther) ;
                                                                     bRtn &= m_pThisCurve         == ((SmCrvPtGapFunction&)crOther). m_pThisCurve ;
                                                                     bRtn &= m_sOtherPoint        == ((SmCrvPtGapFunction&)crOther). m_sOtherPoint ;
                                                               //      bRtn &= m_sLocalNeighborhood == ((SmCrvPtGapFunction&)crOther). m_sLocalNeighborhood ;
                                                                     return(bRtn) ;
                                                                   }
  // destructor
  virtual ~SmCrvPtGapFunction() { }

  // virtual functions                                                  
  virtual const SmObject              * GetThisObject()   const   { return (SmObject*) m_pThisCurve ; }
  virtual const SmObject              * GetOtherObject()  const   { return (SmObject*)&m_sOtherPoint ; }
  virtual const SmCurve               * GetThisCurve()         const  { return   m_pThisCurve ; }
  virtual const SmPoint3d             * GetOtherPoint()        const  { return & m_sOtherPoint ; }
  virtual const SmCurveClassification * GetAsIsLocalNeighborhood() const  { return(& m_sLocalNeighborhood) ; }
  virtual       SmCurveClassification * GetLocalNeighborhood
                                              (SmBoolean bAsIs=FALSE) { if(!m_bIsLocalNeighborhoodSet && !bAsIs) 
                                                                          { BuildLocalNeighborhood() ; }
                                                                        return( & m_sLocalNeighborhood) ;
                                                                      }
                                        
  virtual SmStatus Evaluate(double        dThisU,                 // in : This Geom 1st param used for Curves and Surfaces
                            double        dThisV,                 // NotUsed: in : This Geom 2nd param only used for Surfaces
                            SmGapSample & rGapSample,             // out: Eval holder block
                            double      * pOptOtherGuessU=NULL,   // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
                            double      * pOptOtherGuessV=NULL) ; // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
  virtual SmStatus EvaluateNeighborhoodBoundaries(SmSolutionArray &rSolutions) ;

  // maintenance
 public:
  virtual void            Dump(SmBoolean bAbbrev) const ;
  virtual SmDisplayList * Draw(SmBoolean       bShowGeometry=TRUE,      // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
                               SmBoolean       bShowNeighborhoods=TRUE, // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
                               SmBoolean       bShowSamples=FALSE,      // in : TRUE  = show all sample gaps, 
                                                                        //      UNSURE= Only show bigger-than-tol gaps, 
                                                                        //      FALSE = Show no sample gaps,
                                                                        //      default:[FALSE]
                               double          dDisplayScale=1000.0,    // in : Amount micro geometry is scaled for visualization, default:[1000]
                               SmGfxArraySet * pOptGfxSet=NULL)         // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                              const ;                                   //      NULL to ignore. default:[NULL]

  SM_COMMON(SmCrvPtGapFunction, SmGapFunction, SmCrvPtGapFunction_TYPE) ; 

} ; // end class SmCrvPtGapFunction

/*******************************************************************//**
PURPOSE: class to represent the gap function from a Curve to a Curve

NOTES: Use class IsCrvOnSurf to represent a Edge/UVTrimCurve gap function as

 SmCrvOnSurf s3dTrimCurve( sUVTrimCurve, sSurface) ;
 SmCrvCrvGapFunction sCrvCrvGF(sXSectTol3d, 
                               pEdge->GetCurve(), 
                               pEdge->GetCurve()->GetNaturalInterval(), 
                               s3dTrimCurve) ;
***********************************************************************/
class SM_EXPORT SmCrvCrvGapFunction : public SmGapFunction
{
 protected:
  const SmCurve       * m_pThisCurve ;
  const SmCurve       * m_pOtherCurve ;
  SmCurveClassification m_sLocalNeighborhood ;  // m_pThisCurve entries and exits into the m_pOtherCurve LocalShape

 public:
  // constructor
  SmCrvCrvGapFunction
  (
    const SmXSectTol3d  & rXSectTol3d,   // in : max allowed size for a within TOl gap
    const SmCurve       * pThisCurve,    // in : Gap base curve
    SmExtent1d          & rThisIvl,      // in : Gap base curve interval
    const SmCurve       * pOtherCurve,   // in : Gap Target Curve
    ULONG               lSampleCntU=20   // in : Number of samples to approximate GapFunctcion
  );

  // Copy Constructor from crOther to this                                    
  SmCrvCrvGapFunction(const SmCrvCrvGapFunction &crOther) : SmGapFunction(crOther),
                                                            m_pThisCurve(crOther.m_pThisCurve),
                                                            m_pOtherCurve(crOther.m_pOtherCurve),
                                                            m_sLocalNeighborhood(crOther.m_sLocalNeighborhood)
                                                          { } 

  // virtual deep copy operator from this to rpNewGapFunction
  virtual SmStatus Copy(SmGapFunction  *& rpNewGapFunction) const { rpNewGapFunction = new SmCrvCrvGapFunction(*this); NER(rpNewGapFunction) ;
                                                                    return SM_SUCCESS ;
                                                                  }
  SmGapFunction & operator=( const SmCrvCrvGapFunction &crOther ) { if(this == &crOther) return(*this) ;
                                                                    SmGapFunction::operator=(crOther) ;
                                                                    m_pThisCurve         = crOther. m_pThisCurve ;
                                                                    m_pOtherCurve        = crOther. m_pOtherCurve ;
                                                                    m_sLocalNeighborhood = crOther. m_sLocalNeighborhood ;
                                                                    return(*this) ;
                                                                  }
  virtual SmBoolean operator==(const SmGapFunction &crOther) const { if(this == &crOther) return(TRUE) ;
                                                                     if(!crOther.IsKindOf(SmCrvCrvGapFunction_TYPE)) { return FALSE ; }
                                                                     SmBoolean bRtn = SmGapFunction::operator ==(crOther) ;
                                                                     bRtn &= m_pThisCurve         == ((SmCrvCrvGapFunction&)crOther).m_pThisCurve ;
                                                                     bRtn &= m_pOtherCurve        == ((SmCrvCrvGapFunction&)crOther).m_pOtherCurve ;
                                                                  //   bRtn &= m_sLocalNeighborhood == ((SmCrvCrvGapFunction&)crOther).m_sLocalNeighborhood ;
                                                                     return(bRtn) ;
                                                                   }
  // destructor
  virtual ~SmCrvCrvGapFunction() { }

  // virtual functions
  virtual const SmObject              * GetThisObject()   const   { return (SmObject*) m_pThisCurve ; }
  virtual const SmObject              * GetOtherObject()  const   { return (SmObject*) m_pOtherCurve ; }
  virtual const SmCurve               * GetThisCurve()         const  { return   m_pThisCurve ; }
  virtual const SmCurve               * GetOtherCurve()        const  { return   m_pOtherCurve ; }
  virtual const SmCurveClassification * GetAsIsLocalNeighborhood() const  { return(& m_sLocalNeighborhood) ; }
  virtual       SmCurveClassification * GetLocalNeighborhood
                                              (SmBoolean bAsIs=FALSE) { if(!m_bIsLocalNeighborhoodSet && !bAsIs) 
                                                                          { BuildLocalNeighborhood() ; }
                                                                        return( & m_sLocalNeighborhood) ;
                                                                      }
                                        
  virtual SmStatus Evaluate(double        dThisU,                 // in : This Geom 1st param used for Curves and Surfaces
                            double        dThisV,                 // NotUsed: in : This Geom 2nd param only used for Surfaces
                            SmGapSample & rGapSample,             // out: Eval holder block
                            double      * pOptOtherGuessU=NULL,   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
                            double      * pOptOtherGuessV=NULL) ; // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
  virtual SmStatus EvaluateNeighborhoodBoundaries(SmSolutionArray &rSolutions) ;

  // maintenance
 public:
  virtual void            Dump(SmBoolean bAbbrev) const ;
  virtual SmDisplayList * Draw(SmBoolean       bShowGeometry=TRUE,      // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
                               SmBoolean       bShowNeighborhoods=TRUE, // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
                               SmBoolean       bShowSamples=FALSE,      // in : TRUE  = show all sample gaps, 
                                                                        //      UNSURE= Only show bigger-than-tol gaps, 
                                                                        //      FALSE = Show no sample gaps,
                                                                        //      default:[FALSE]
                               double          dDisplayScale=1000.0,    // in : Amount micro geometry is scaled for visualization, default:[1000]
                               SmGfxArraySet * pOptGfxSet=NULL)         // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                              const ;                                   //      NULL to ignore. default:[NULL]

  SM_COMMON(SmCrvCrvGapFunction, SmGapFunction, SmCrvCrvGapFunction_TYPE) ; 

} ; // end class SmCrvCrvGapFunction

/*******************************************************************//**
PURPOSE: class to represent the gap function from a Curve to a Surface

NOTES: 
***********************************************************************/
class SM_EXPORT SmCrvSrfGapFunction : public SmGapFunction
{
 protected:
  const SmCurve       * m_pThisCurve ;
  const SmSurface     * m_pOtherSurface ;
  SmCurveClassification m_sLocalNeighborhood ;  // m_pThisCurve entries and exits into the m_pOtherSurface LocalShape
                                                // SmCrvSrfGapFunction can't yet support LocalNeighborhoods
                                                //  because SmSurface::GlobalCurveSolve does not support SM_SO_AT_DISTANCE

 public:
  // constructor
  SmCrvSrfGapFunction
  (
    const SmXSectTol3d & rXSectTol3d,
    const SmCurve      * pThisCurve,
    SmExtent1d         & rThisIvl,
    const SmSurface    * pOtherSurface,
    ULONG                lSampleCntU=20
  )
    : SmGapFunction(SM_GS_CRV_SRF, 
                    rXSectTol3d,
                    rThisIvl, 
                    rThisIvl,
                    lSampleCntU, 
                    1),
       m_pThisCurve(pThisCurve),
       m_pOtherSurface(pOtherSurface),
       // gwc: passing a XSectTol as a ZoneTol3d - needs fix
       m_sLocalNeighborhood(pThisCurve, rThisIvl, NULL, (SmZoneTol3d &)rXSectTol3d)
                                                        {  m_sSampleIvlV.SetMinMax(0.0,1.0) ; }
  // Copy Constructor from crOther to this                                    
  SmCrvSrfGapFunction(const SmCrvSrfGapFunction &crOther) : SmGapFunction(crOther),
                                                            m_pThisCurve(crOther.m_pThisCurve),
                                                            m_pOtherSurface(crOther.m_pOtherSurface),
                                                            m_sLocalNeighborhood(crOther.m_sLocalNeighborhood)
                                                          { } 

  // virtual deep copy operator from this to rpNewGapFunction
  virtual SmStatus Copy(SmGapFunction  *& rpNewGapFunction) const { rpNewGapFunction = new SmCrvSrfGapFunction(*this); NER(rpNewGapFunction) ;
                                                                    return SM_SUCCESS ;
                                                                  }
  SmGapFunction & operator=( const SmCrvSrfGapFunction &crOther ) { if(this == &crOther) return(*this) ;
                                                                    SmGapFunction::operator=(crOther) ;
                                                                    m_pThisCurve         = crOther. m_pThisCurve ;
                                                                    m_pOtherSurface      = crOther. m_pOtherSurface ;
                                                                    m_sLocalNeighborhood = crOther. m_sLocalNeighborhood ;
                                                                    return(*this) ;
                                                                  }
  virtual SmBoolean operator==(const SmGapFunction &crOther) const { if(this == &crOther) return(TRUE) ;
                                                                     if(!crOther.IsKindOf(SmCrvSrfGapFunction_TYPE)) { return FALSE ; }
                                                                     SmBoolean bRtn = SmGapFunction::operator ==(crOther) ;
                                                                     bRtn &= m_pThisCurve         == ((SmCrvSrfGapFunction&)crOther).m_pThisCurve ;
                                                                     bRtn &= m_pOtherSurface      == ((SmCrvSrfGapFunction&)crOther).m_pOtherSurface ;
                                                                //     bRtn &= m_sLocalNeighborhood == ((SmCrvSrfGapFunction&)crOther).m_sLocalNeighborhood ;
                                                                     return(bRtn) ;
                                                                   }
  // destructor
  virtual ~SmCrvSrfGapFunction() { }

  // virtual functions
  virtual const SmObject      * GetThisObject()   const   { return (SmObject*) m_pThisCurve ; }
  virtual const SmObject      * GetOtherObject()  const   { return (SmObject*) m_pOtherSurface ; }
  virtual const SmCurve               * GetThisCurve()         const  { return   m_pThisCurve ; }
  virtual const SmSurface             * GetOtherSurface()      const  { return   m_pOtherSurface ; }
  virtual const SmCurveClassification * GetAsIsLocalNeighborhood() const  { return(& m_sLocalNeighborhood) ; }
  virtual       SmCurveClassification * GetLocalNeighborhood
                                              (SmBoolean bAsIs=FALSE)     { SM_REF1(bAsIs) ;
                                                                            // until SmCrvSrfGapFunction supports
                                                                            // LocalNeighborhoods when 
                                                                            // SmSurface::GlobalCurveSolve supports SM_SO_AT_DISTANCE
                                                                            return(NULL) ;
                                                                            // code for when SmSurface::GlobalCurveSolve() is extended
                                                                            // if(!m_bIsLocalNeighborhoodSet && !bAsIs) 
                                                                            //   { BuildLocalNeighborhood() ; }
                                                                            // return( & m_sLocalNeighborhood) ;
                                                                      }
                                        
  virtual SmStatus Evaluate(double        dThisU,                 // in : This Geom 1st param used for Curves and Surfaces
                            double        dThisV,                 // NotUsed: in : This Geom 2nd param only used for Surfaces
                            SmGapSample & rGapSample,             // out: Eval holder block
                            double      * pOptOtherGuessU=NULL,   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
                            double      * pOptOtherGuessV=NULL) ; // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 

  virtual SmStatus EvaluateNeighborhoodBoundaries(SmSolutionArray &rSolutions) ;

  // maintenance
 public:
  virtual void            Dump(SmBoolean bAbbrev) const ;
  virtual SmDisplayList * Draw(SmBoolean       bShowGeometry=TRUE,      // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
                               SmBoolean       bShowNeighborhoods=TRUE, // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
                               SmBoolean       bShowSamples=FALSE,      // in : TRUE  = show all sample gaps, 
                                                                        //      UNSURE= Only show bigger-than-tol gaps, 
                                                                        //      FALSE = Show no sample gaps,
                                                                        //      default:[FALSE]
                               double          dDisplayScale=1000.0,    // in : Amount micro geometry is scaled for visualization, default:[1000]
                               SmGfxArraySet * pOptGfxSet=NULL)         // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                              const ;                                   //      NULL to ignore. default:[NULL]

  SM_COMMON(SmCrvSrfGapFunction, SmGapFunction, SmCrvSrfGapFunction_TYPE) ; 

} ; // end class SmCrvSrfGapFunction

/*******************************************************************//**
PURPOSE: class to represent the gap function from a Surface to a Point

NOTES: 
***********************************************************************/
class SM_EXPORT SmSrfPtGapFunction : public SmGapFunction
{
 protected:
  const SmSurface * m_pThisSurface ;
  SmPoint3d         m_sOtherPoint ;

 public:
  // constructor
  SmSrfPtGapFunction(SmXSectTol3d    & rXSectTol3d,
                     const SmSurface * pThisSurface,
                     SmExtent2d      & rThisDomain,
                     SmPoint3d       * pOtherPoint,
                     ULONG             lSampleCntU=20) : SmGapFunction(SM_GS_SRF_PT, 
                                                                       rXSectTol3d,
                                                                       rThisDomain.GetUInterval(), 
                                                                       rThisDomain.GetVInterval(),
                                                                       lSampleCntU, 
                                                                       1),
                                                         m_pThisSurface(pThisSurface),
                                                         m_sOtherPoint(*pOtherPoint)
                                                       { }
  // Copy Constructor from crOther to this                                    
  SmSrfPtGapFunction(const SmSrfPtGapFunction &crOther) : SmGapFunction(crOther),
                                                          m_pThisSurface(crOther.m_pThisSurface),
                                                          m_sOtherPoint(crOther.m_sOtherPoint)
                                                        { } 

  // virtual deep copy operator from this to rpNewGapFunction
  virtual SmStatus Copy(SmGapFunction  *& rpNewGapFunction) const { rpNewGapFunction = new SmSrfPtGapFunction(*this); NER(rpNewGapFunction) ;
                                                                    return SM_SUCCESS ;
                                                                  }

  SmGapFunction & operator=( const SmSrfPtGapFunction &crOther ) { if(this == &crOther) return(*this) ;
                                                                    SmGapFunction::operator=(crOther) ;
                                                                    m_pThisSurface     = crOther.m_pThisSurface ;
                                                                    m_sOtherPoint      = crOther.m_sOtherPoint ;
                                                                    return(*this) ;
                                                                  }
  virtual SmBoolean operator==(const SmGapFunction &crOther) const { if(this == &crOther) return(TRUE) ;
                                                                     if(!crOther.IsKindOf(SmSrfPtGapFunction_TYPE)) { return FALSE ; }
                                                                     SmBoolean bRtn = SmGapFunction::operator ==(crOther) ;
                                                                     bRtn &= m_pThisSurface       == ((SmSrfPtGapFunction&)crOther).m_pThisSurface ;
                                                                     bRtn &= m_sOtherPoint        == ((SmSrfPtGapFunction&)crOther).m_sOtherPoint ;
                                                                     return(bRtn) ;
                                                                   }
  // destructor
  virtual ~SmSrfPtGapFunction() { }

  // virtual functions                                           
  virtual const SmObject    * GetThisObject()  const   { return (SmObject*) m_pThisSurface ; }
  virtual const SmObject    * GetOtherObject() const   { return (SmObject*)& m_sOtherPoint ; }
  virtual const SmSurface   * GetThisSurface() const   { return   m_pThisSurface ; }
  virtual const SmPoint3d   * GetOtherPoint()  const   { return & m_sOtherPoint ; }

  virtual SmStatus Evaluate(double       dThisU,                 // in : This Geom 1st param used for Curves and Surfaces
                            double       dThisV,                 // in : This Geom 2nd param only used for Surfaces
                            SmGapSample &rGapSample,             // out: Eval holder block
                            double      *pOptOtherGuessU=NULL,   // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
                            double      *pOptOtherGuessV=NULL) ; // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 

  // maintenance
 public:
  virtual void            Dump(SmBoolean bAbbrev) const ;
  virtual SmDisplayList * Draw(SmBoolean       bShowGeometry=TRUE,      // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
                               SmBoolean       bShowNeighborhoods=TRUE, // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
                               SmBoolean       bShowSamples=FALSE,      // in : TRUE  = show all sample gaps, 
                                                                        //      UNSURE= Only show bigger-than-tol gaps, 
                                                                        //      FALSE = Show no sample gaps,
                                                                        //      default:[FALSE]
                               double          dDisplayScale=1000.0,    // in : Amount micro geometry is scaled for visualization, default:[1000]
                               SmGfxArraySet * pOptGfxSet=NULL)         // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                              const ;                                   //      NULL to ignore. default:[NULL]

  SM_COMMON(SmSrfPtGapFunction, SmGapFunction, SmSrfPtGapFunction_TYPE) ; 

} ; // end class SmSrfPtGapFunction

/*******************************************************************//**
PURPOSE: class to represent the gap function from a Surface to a Curve

NOTES: 
***********************************************************************/
class SM_EXPORT SmSrfCrvGapFunction : public SmGapFunction
{
 protected:
  const SmSurface * m_pThisSurface ;
  const SmCurve   * m_pOtherCurve ;

 public:
  // constructor
  SmSrfCrvGapFunction(SmXSectTol3d    & rXSectTol3d,
                      const SmSurface * pThisSurface,
                      SmExtent2d      & rThisDomain,
                      const SmCurve   * pOtherCurve,
                      ULONG             lSampleCntU=20) : SmGapFunction(SM_GS_SRF_CRV, 
                                                                        rXSectTol3d,
                                                                        rThisDomain.GetUInterval(), 
                                                                        rThisDomain.GetVInterval(),
                                                                        lSampleCntU, 
                                                                        1),
                                                          m_pThisSurface(pThisSurface),
                                                          m_pOtherCurve(pOtherCurve)
                                                        { }
  // Copy Constructor from crOther to this                                    
  SmSrfCrvGapFunction(const SmSrfCrvGapFunction &crOther) : SmGapFunction(crOther),
                                                            m_pThisSurface(crOther.m_pThisSurface),
                                                            m_pOtherCurve(crOther.m_pOtherCurve)
                                                          { } 

  // virtual deep copy operator from this to rpNewGapFunction
  virtual SmStatus Copy(SmGapFunction  *& rpNewGapFunction) const { rpNewGapFunction = new SmSrfCrvGapFunction(*this); NER(rpNewGapFunction) ;
                                                                    return SM_SUCCESS ;
                                                                  }
  SmGapFunction & operator=( const SmSrfCrvGapFunction &crOther ) { if(this == &crOther) return(*this) ;
                                                                    SmGapFunction::operator=(crOther) ;
                                                                    m_pThisSurface = crOther.m_pThisSurface ;
                                                                    m_pOtherCurve  = crOther.m_pOtherCurve ;
                                                                    return(*this) ;
                                                                  }
  virtual SmBoolean operator==(const SmGapFunction &crOther) const { if(this == &crOther) return(TRUE) ;
                                                                     if(!crOther.IsKindOf(SmSrfCrvGapFunction_TYPE)) { return FALSE ; }
                                                                     SmBoolean bRtn = SmGapFunction::operator ==(crOther) ;
                                                                     bRtn &= m_pThisSurface       == ((SmSrfCrvGapFunction&)crOther).m_pThisSurface ;
                                                                     bRtn &= m_pOtherCurve        == ((SmSrfCrvGapFunction&)crOther).m_pOtherCurve ;
                                                                     return(bRtn) ;
                                                                   }
  // destructor
  virtual ~SmSrfCrvGapFunction() { }

  // virtual functions
  virtual const SmObject  * GetThisObject()  const   { return (SmObject*) m_pThisSurface ; }
  virtual const SmObject  * GetOtherObject() const   { return (SmObject*) m_pOtherCurve ; }
  virtual const SmSurface * GetThisSurface() const  { return   m_pThisSurface ; }
  virtual const SmCurve   * GetOtherCurve()  const  { return   m_pOtherCurve ; }

  virtual SmStatus Evaluate(double       dThisU,                 // in : This Geom 1st param used for Curves and Surfaces
                            double       dThisV,                 // in : This Geom 2nd param only used for Surfaces
                            SmGapSample &rGapSample,             // out: Eval holder block
                            double      *pOptOtherGuessU=NULL,   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
                            double      *pOptOtherGuessV=NULL) ; // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 

  // maintenance
 public:
  virtual void            Dump(SmBoolean bAbbrev) const ;
  virtual SmDisplayList * Draw(SmBoolean       bShowGeometry=TRUE,      // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
                               SmBoolean       bShowNeighborhoods=TRUE, // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
                               SmBoolean       bShowSamples=FALSE,      // in : TRUE  = show all sample gaps, 
                                                                        //      UNSURE= Only show bigger-than-tol gaps, 
                                                                        //      FALSE = Show no sample gaps,
                                                                        //      default:[FALSE]
                               double          dDisplayScale=1000.0,    // in : Amount micro geometry is scaled for visualization, default:[1000]
                               SmGfxArraySet * pOptGfxSet=NULL)         // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                              const ;                                   //      NULL to ignore. default:[NULL]

  SM_COMMON(SmSrfCrvGapFunction, SmGapFunction, SmSrfCrvGapFunction_TYPE) ; 

} ; // end class SmSrfCrvGapFunction

/*******************************************************************//**
PURPOSE: class to represent the gap function from a Surface to a Surface

NOTES: 
***********************************************************************/
class SM_EXPORT SmSrfSrfGapFunction : public SmGapFunction
{
 protected:
  const SmSurface * m_pThisSurface ;
  const SmSurface * m_pOtherSurface ;

 public:
  // constructor
  SmSrfSrfGapFunction
  (
    const SmXSectTol3d  & rXSectTol3d,
    const SmSurface     * pThisSurface,
    SmExtent2d          & rThisDomain,
    const SmSurface     * pOtherSurface,
    ULONG                 lSampleCntU=20,
    ULONG                 lSampleCntV=20
  ) : SmGapFunction(SM_GS_SRF_SRF, 
                    rXSectTol3d,
                    rThisDomain.GetUInterval(), 
                    rThisDomain.GetVInterval(),
                    lSampleCntU, 
                    lSampleCntV),
      m_pThisSurface(pThisSurface),
      m_pOtherSurface(pOtherSurface)
  { }

  // Copy Constructor from crOther to this                                    
  SmSrfSrfGapFunction(const SmSrfSrfGapFunction &crOther) : SmGapFunction(crOther),
                                                            m_pThisSurface(crOther.m_pThisSurface),
                                                            m_pOtherSurface(crOther.m_pOtherSurface)
                                                          { } 

  // virtual deep copy operator from this to rpNewGapFunction
  virtual SmStatus Copy(SmGapFunction  *& rpNewGapFunction) const { rpNewGapFunction = new SmSrfSrfGapFunction(*this); NER(rpNewGapFunction) ;
                                                                    return SM_SUCCESS ;
                                                                  }
  SmGapFunction & operator=( const SmSrfSrfGapFunction &crOther ) { if(this == &crOther) return(*this) ;
                                                                    SmGapFunction::operator=(crOther) ;
                                                                    m_pThisSurface  = crOther.m_pThisSurface ;
                                                                    m_pOtherSurface = crOther.m_pOtherSurface ;
                                                                    return(*this) ;
                                                                  }
  virtual SmBoolean operator==(const SmGapFunction &crOther) const { if(this == &crOther) return(TRUE) ;
                                                                     if(!crOther.IsKindOf(SmSrfSrfGapFunction_TYPE)) { return FALSE ; }
                                                                     SmBoolean bRtn = SmGapFunction::operator ==(crOther) ;
                                                                     bRtn &= m_pThisSurface       == ((SmSrfSrfGapFunction&)crOther).m_pThisSurface ;
                                                                     bRtn &= m_pOtherSurface      == ((SmSrfSrfGapFunction&)crOther).m_pOtherSurface ;
                                                                     return(bRtn) ;
                                                                   }
  // destructor
  virtual ~SmSrfSrfGapFunction() { }

  // virtual functions
  virtual const SmObject  * GetThisObject()   const  { return (SmObject*) m_pThisSurface ; }
  virtual const SmObject  * GetOtherObject()  const  { return (SmObject*) m_pOtherSurface ; }
  virtual const SmSurface * GetThisSurface()  const  { return m_pThisSurface ; }
  virtual const SmSurface * GetOtherSurface() const  { return m_pOtherSurface ; }

  virtual SmStatus Evaluate(double       dThisU,                 // in : This Geom 1st param used for Curves and Surfaces
                            double       dThisV,                 // in : This Geom 2nd param only used for Surfaces
                            SmGapSample &rGapSample,             // out: Eval holder block
                            double      *pOptOtherGuessU=NULL,   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
                            double      *pOptOtherGuessV=NULL) ; // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 

  // maintenance
 public:
  virtual void            Dump(SmBoolean bAbbrev) const ;
  virtual SmDisplayList * Draw(SmBoolean       bShowGeometry=TRUE,      // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
                               SmBoolean       bShowNeighborhoods=TRUE, // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
                               SmBoolean       bShowSamples=FALSE,      // in : TRUE  = show all sample gaps, 
                                                                        //      UNSURE= Only show bigger-than-tol gaps, 
                                                                        //      FALSE = Show no sample gaps,
                                                                        //      default:[FALSE]
                               double          dDisplayScale=1000.0,    // in : Amount micro geometry is scaled for visualization, default:[1000]
                               SmGfxArraySet * pOptGfxSet=NULL)         // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                              const ;                                   //      NULL to ignore. default:[NULL]

  SM_COMMON(SmSrfSrfGapFunction, SmGapFunction, SmSrfSrfGapFunction_TYPE) ; 

} ; // end class SmSrfSrfGapFunction

// BEGIN OBSOLETE1

//        virtual SmCurve   *GetThisCurve()          { return NULL ; }
//        virtual SmSurface *GetThisSurface()        { return NULL ; }
//      
//        virtual SmPoint3d *GetOtherPoint()         { return NULL ; }
//        virtual SmCurve   *GetOtherCurve()         { return NULL ; }
//        virtual SmSurface *GetOtherSurface()       { return NULL ; }
//        virtual SmStatus Evaluate(double       dThisU,       // in : This Geom 1st param used for Curves and Surfaces
//                                  double       dThisV,       // in : This Geom 2nd param only used for Surfaces
//                                  SmGapSample &rGapSample)   // out: Eval holder block
//                                                           { SM_REF3(dThisU, dThisV, rGapSample) ; 
//                                                             SE(SM_ERR) ; return SM_ERR ; 
//                                                           }



//      
//      /*******************************************************************//**
//      PURPOSE: A Vertex/Edge Gap
//      
//      NOTES: 
//      ***********************************************************************/
//      class SM_EXPORT SmVertexEdgeGap : public SmGap
//      {
//       public:
//        double        m_dVertexToEdgeGap ;  // 3d gap between this vertex and an edge-EndPoint
//        SmVertexuse * m_pVertexuseToEdge ;  // vertexuse connecting its vertex to an edge for this gap
//        double        m_dEdgeT ;            // Edge parameter connected to Gap
//      
//       public:
//        // constructor, copy constructor, destructor, assignement, virtual MakeCopy
//        SmVertexEdgeGap(double       dVertexToEdgeGap = SM_UNDEF_DOUBLE,      
//                        SmVertexuse *pVertexuseToEdge = NULL,
//                        double       dEdgeT           = SM_UNDEF_DOUBLE) ;
//      
//        SmVertexEdgeGap(const SmVertexEdgeGap &crGap) ;
//       virtual ~SmVertexEdgeGap()                           { ReSet() ; }                                  
//        virtual SmGap *   MakeCopy() const          { return( new SmVertexEdgeGap(*this)) ; }
//        virtual void      ReSet() ;
//        SmVertexEdgeGap & operator= (const SmVertexEdgeGap &crGap) ;
//        virtual SmBoolean operator==(const SmGap &crGap) ;
//      
//        // simple access
//       public:
//        virtual double    GetGap()    const    { return m_dVertexToEdgeGap ; }
//        virtual void      SetGap(double dGap)  { m_dVertexToEdgeGap = dGap ; }
//        
//        virtual SmVertex *GetVertex() const ;
//        virtual SmEdge   *GetEdge()   const ;
//        virtual double    GetEdgeT()  const    { return m_dEdgeT ; }
//      
//        // maintenance
//        virtual void           Dump() const ;
//        virtual SmDisplayList *Draw() const ;
//      
//      } ; // end class SmVertexEdgeGap
//      
//      /*******************************************************************//**
//      PURPOSE: A Vertex/Face Gap
//      
//      NOTES: 
//      ***********************************************************************/
//      class SM_EXPORT SmVertexFaceGap : public SmGap
//      {
//       public:
//        double        m_dVertexToFaceGap ;  // 3d gap between this vertex and a Face->Surface->DropPoint
//        SmVertexuse * m_pVertexuseToFace ;  // vertexuse connecting its vertex to a face for this gap
//                                            //   may be either  SmLoopuse_TYPE  (connects to Face thru Loopuse)
//                                            //                  SmEdgeuse_TYPE  (connects to Face thru Edgeuse->Loopuse)
//        SmPoint2d     m_sFaceUV ;           // Face parameter connected to Gap
//      
//       public:
//        // constructor, copy constructor, destructor, assignement, virtual MakeCopy
//        SmVertexFaceGap(double       dVertexToFaceGap = SM_UNDEF_DOUBLE,      
//                        SmVertexuse *pVertexuseToFace = NULL,
//                        SmPoint2d   *pFaceUV          = NULL) ;
//      
//        SmVertexFaceGap(const SmVertexFaceGap &crGap) ;
//       virtual ~SmVertexFaceGap()                           { ReSet() ; }         
//        virtual SmGap *   MakeCopy() const          { return( new SmVertexFaceGap(*this)) ; }
//        virtual void      ReSet() ;
//        SmVertexFaceGap & operator= (const SmVertexFaceGap &crGap) ;
//        virtual SmBoolean operator==(const SmGap &crGap) ;
//      
//        // simple access
//       public:
//        virtual double     GetGap()    const    { return m_dVertexToFaceGap ; }
//        virtual void       SetGap(double dGap)  { m_dVertexToFaceGap = dGap ; }
//       
//        virtual SmVertex  *GetVertex() const ;
//        virtual SmFace    *GetFace()   const ;
//        virtual SmPoint2d  GetFaceUV()          { return m_sFaceUV ; }
//      
//        // maintenance
//        virtual void           Dump() const ;
//        virtual SmDisplayList *Draw() const ;
//      
//      } ; // end class SmVertexFaceGap
//      
//      /*******************************************************************//**
//      PURPOSE: An Edge/Vertex Gap
//      
//      NOTES: 
//      ***********************************************************************/
//      class SM_EXPORT SmEdgeVertexGap : public SmGap
//      {
//       public:
//        double        m_dEdgeToVertexGap ;  // 3d gap between an edge-EndPoint and a vertex
//        SmEdgeuse   * m_pEdgeuseToVertex ;  // edgeuse connecting its edge to a vertex for this gap
//        double        m_dEdgeT ;            // Edge parameter connected to Gap
//      
//       public:
//        // constructor, copy constructor, destructor, assignement, virtual MakeCopy
//        SmEdgeVertexGap(double     dEdgeToVertexGap = SM_UNDEF_DOUBLE,      
//                        SmEdgeuse *pEdgeuseToVertex = NULL,
//                        double     dEdgeT           = SM_UNDEF_DOUBLE) ;
//      
//        SmEdgeVertexGap(const SmEdgeVertexGap &crGap) ;
//       virtual ~SmEdgeVertexGap()                                { ReSet() ; }    
//        virtual SmGap *   MakeCopy() const               { return( new SmEdgeVertexGap(*this)) ; }
//        virtual void      ReSet() ;
//        SmEdgeVertexGap & operator= (const SmEdgeVertexGap &crGap) ;
//        virtual SmBoolean operator==(const SmGap &crGap) ;
//      
//        // simple access
//       public:
//        virtual double    GetGap()    const    { return m_dEdgeToVertexGap ; }
//        virtual void      SetGap(double dGap)  { m_dEdgeToVertexGap = dGap ; }
//      
//        virtual SmEdge   *GetEdge()   const ;
//        virtual double    GetEdgeT()  const    { return m_dEdgeT ; }
//        virtual SmVertex *GetVertex() const ;
//      
//        // maintenance
//        virtual void           Dump() const ;
//        virtual SmDisplayList *Draw() const ;
//      
//      } ; // end class SmEdgeVertexGap
//      
//      /*******************************************************************//**
//      PURPOSE: A Edge/Face Gap
//      
//      NOTES: 
//      ***********************************************************************/
//      class SM_EXPORT SmEdgeFaceGap : public SmGap
//      {
//       public:
//        double        m_dEdgeToFaceGap ;  // max 3d gap between this edge its Face->UVTrimCurve
//        SmEdgeuse   * m_pEdgeuseToFace ;  // edgeuse connecting its edge to a face for this gap
//                                          //   must be of type SmLoopuse_TYPE
//        SmPoint2d     m_sFaceUV ;         // Face parameter connected to MaxGap
//        double        m_dEdgeT ;          // Edge parameter connected to MaxGap
//      
//       public:
//        // constructor, copy constructor, destructor, assignement, virtual MakeCopy
//        SmEdgeFaceGap(double       dEdgeToFaceGap = SM_UNDEF_DOUBLE,      
//                        SmEdgeuse *pEdgeuseToFace = NULL,
//                        SmPoint2d *pFaceUV        = NULL,
//                        double     dEdgeT         = SM_UNDEF_DOUBLE) ;
//      
//        SmEdgeFaceGap(const SmEdgeFaceGap &crGap) ;
//       virtual ~SmEdgeFaceGap()                                  { ReSet() ; }  
//        virtual SmGap *   MakeCopy() const               { return( new SmEdgeFaceGap(*this)) ; }
//        virtual void      ReSet() ;
//        SmEdgeFaceGap &   operator= (const SmEdgeFaceGap &crGap) ;
//        virtual SmBoolean operator==(const SmGap &crGap) ;
//      
//        // simple access
//       public:
//        virtual double     GetGap()    const    { return m_dEdgeToFaceGap ; }
//        virtual void       SetGap(double dGap)  { m_dEdgeToFaceGap = dGap ; }
//       
//        virtual SmEdge    *GetEdge()   const ;
//        virtual SmFace    *GetFace()   const ;
//        virtual SmPoint2d  GetFaceUV()          { return m_sFaceUV ; }
//        virtual double     GetEdgeT()  const    { return m_dEdgeT ; }
//      
//        // maintenance
//        virtual void           Dump() const ;
//        virtual SmDisplayList *Draw() const ;
//      
//      } ; // end class SmEdgeFaceGap
// end OBSOLETE1

// BEGIN OBSOLETE2
//  
//  
//  class SM_EXPORT SmGap
//  {
//   public:
//    // members
//    SmGapType     m_eGapType ;            // oneof SM_GT_VERTEX_EDGE
//                                          //       SM_GT_VERTEX_UVTRIMCURVE
//                                          //       SM_GT_VERTEX_FACE 
//                                          //       SM_GT_EDGE_UVTRIMCURVE
//                                          //       SM_GT_EDGE_FACE     (currently the same as SM_GT_EDGE_UVTRIMCURVE)
//                                          
//    double        m_dGap3d ;              // 3d gap between two topology objects
//                                          
//    SmEdgeuse   * m_pEdgeuse ;            // connecting an edge to a edgeuse or face, for gap 
//                                          //    types: SM_GT_EDGE_UVTRIMCURVE
//                                          //           SM_GT_EDGE_FACE 
//    SmVertexuse * m_pVertexuse ;          // connecting a vertex to an edge, edgeuse or face for gap
//                                          //    types: SM_GT_VERTEX_EDGE,
//                                          //           SM_GT_VERTEX_UVTRIMCURVE,
//                                          //           SM_GT_VERTEX_FACE
//                                          // RULE: only one of m_pEdgeuse or m_pVerteuse is nonNULL at a time.
//                                          
//    double        m_dEdgeT ;              // Edge->Curve parameter connected to Gap when gap connects to an edge
//                                          //  (SM_GT_VERTEX_EDGE, SM_GT_EDGE_UVTRIMCURVE, SM_GT_EDGE_FACE)
//                                          //  else set to SM_UNDEF_DOUBLE
//  
//    double        m_dUVTrimCurveT ;       // Edge->UVTrimCurve parameter connected to Gap when gap connects 
//                                          //  to the projection of an edge->UVtrimCurve through an edgeuse->Face->Surface
//                                          //  (SM_GT_EDGE_UVTRIMCURVE, SM_GT_VERTEX_UVTRIMCURVE)
//  
//    SmPoint2d     m_sFaceUV ;             // Face->Surface parameter connected to Gap when gap connects to a face
//                                          //  (SM_GT_VERTEX_FACE, SM_GT_EDGE_FACE)
//                                          //  else set to [SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE]
//                              
//   public:
//    // uninitizalized default constructor
//    SmGap()                               { ReSet() ; }
//  
//    // Construct Vertex/Edge        gap: SmGapType=SM_GT_VERTEX_EDGE
//    // Construct Vertex/UVTrimCurve gap: SmGapType=SM_GT_VERTEX_UVTRIMCURVE
//    SmGap(double dGap3d, SmVertexuse *pVertexuse, double dEdgeT, SmGapType=SM_GT_VERTEX_EDGE) ;
//  
//    // Construct Vertex/Face 
//    SmGap(double dGap3d, SmVertexuse *pVertexuse, SmPoint2d &rFaceUV);
//  
//    // Construct Edge/UVTrimCurve gap: SmGapType=SM_GT_EDGE_UVTRIMCURVE
//    SmGap(double dGap3d, SmEdgeuse   *pEdgeuse,   double dEdgeT, double dUVTrimCurveT) ;
//  
//    // Construct Edge/Face    gap: SmGapType=SM_GT_EDGE_FACE
//    SmGap(double dGap3d, SmEdgeuse   *pEdgeuse,   double dEdgeT, SmPoint2d &rFaceUV) ;
//  
//    // copy constructor
//    SmGap(const SmGap &crGap) ;
//  
//    // destructor, Makecopy, Reset, assignment operator, equality operator
//    virtual ~ SmGap()           { ReSet() ; }
//    SmGap   * MakeCopy() const  { return( new SmGap(*this)) ; }
//    void      ReSet() ;
//    SmGap   & operator= (const SmGap &crGap) ;
//    SmBoolean operator==(const SmGap &crGap) const ;
//  
//    // simple access
//   public:                                                         
//    SmGapType         GetType()                    const { return m_eGapType ; }
//    SmBoolean         IsKindOf(SmGapType eGapType) const { return( m_eGapType == eGapType) ; }  
//    double            GetGap()                     const { return m_dGap3d ; } 
//    void              SetGap(double dGap3d)              { m_dGap3d = dGap3d ; }    
//                                                         
//    SmVertex        * GetVertex()       const ;    // get vertex  only when gap is bounded by one, else return NULL
//    SmEdge          * GetEdge()         const ;    // get edge    only when gap is bounded by one, else return NULL
//    SmEdgeuse       * GetEdgeuse()      const ;    // get edgeuse only when gap is bounded by one, else return NULL
//    SmFace          * GetFace()         const ;    // get face    only when gap is bounded by one, else return NULL
//                                        
//    double            GetEdgeT()        const ;    // get edge->Curve param only when gap is bounded by an edge, else return SM_UNDEF_DOUBLE
//    double            GetUVTrimCurveT() const ;    // get edge->UVTrimCurve param only when gap is bounded by a SmCrvOnSurf(UVTrimCurve,Surf), else return SM_UNDEF_DOUBLE
//    const SmPoint2d   GetFaceUV()       const ;    // get face->Surface param only when gap is bounded by a face, else return [SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE]
//  
//    // Set() functions modify all internal values for a consistent representation
//    void Set(double dGap, SmVertexuse *pVertexuse, double dEdgeT, SmGapType=SM_GT_VERTEX_EDGE) ; // set Vertex/Edge or Vertex/Edgeuse gap
//    void Set(double dGap, SmVertexuse *pVertexuse, SmPoint2d &rFaceUV) ;                         // set Vertex/Face gap
//    void Set(double dGap, SmEdgeuse   *pEdgeuse,   double dEdgeT, double dUVTrimCurveT) ;        // set Edge/Edgeuse gap
//    void Set(double dGap, SmEdgeuse   *pEdgeuse,   double dEdgeT, SmPoint2d &rFaceUV) ;          // set Edge/Face gap
//    
//    // side effect
//    SmStatus RefineGeometry(SmGapArray &rRemainingGaps,
//                            SmBoolean  &bMadeChange) ;
//                                               
//    // maintenance                             
//    SmDisplayList *Draw(double          dTol=0.00001,
//                        SmGfxArraySet * pArraySet=NULL) const ;
//    SmDisplayList *DrawNeighbors(SmGfxArraySet * pArraySet=NULL) const ;
//    void           Dump(ULONG *pOptLabel=NULL)  const ;
//    void           GetDumpLine(TCHAR *sBuff, TCHAR *sBuffForFile) const ;
//                                           
//    const TCHAR * GetTypeString() const { return _T("SmGap") ; }
//  } ; // end SmGap
//  
//  // add a SmTArray<SmGap> template to the dll interface
//  SM_TARRAY_TEMPLATE_PREDECLARATION(SmGap);
//  
//  
// END OBSOLETE2

#endif // !__SMGAP_H__

