// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCutter.h
* PURPOSE: Header file for SmCutter object and some of its subclasses.
**********************************************************************/

#ifndef __SMCUTTER_H__
#define __SMCUTTER_H__

#include <SmBrep.h>
#include <SmTypes.h>

/*******************************************************************//**
PURPOSE: The cutter object defines something which can be used
    to cut away a portion of a brep.  It could be a plane, other 
    analytical surface, NURBS surface, or even another BREP.

NOTES: 
***********************************************************************/
class SM_EXPORT SmCutter
{
 protected:
  SmTArray<SmEdge*> m_apEdgesInCutter ; // list of Edges merged into TgtBrep from cutter during the 
                                        // merge cutter/TgtBrep XSect results into TgtBrep step.
 public:
  // default constructor
  SmCutter() { } ;
  
  // destructor
  virtual ~SmCutter() { };
  
  // Merge cutter/TgtBrep XSect results into TgtBrep. Pure virtual function
  virtual SmStatus MergeIntersection(SmBrep             * pTgtBrep,        // in : SmBrep to cut with this plane
                                     double               dXSectTol3d,     // in : Min dist between unique points
                                     SmTArray<SmFace *> * pOptSubsetFaces) // NotUsed: in : optional subset of faces, NULL to ignore, default:[NULL]
                                    { SM_REF3(pTgtBrep, dXSectTol3d, pOptSubsetFaces) ; SE( SM_ERR ); return SM_ERR ; }
  
  // Create lists of AfterCut Faces, Edge, and Vertices to delete. Pure virtual function
  virtual SmStatus Classification(SmBrep                * pTgtBrep,           // in : target Brep
                                  double                  dXSectTol3d,        // in : Min dist between unique points
                                  SmTArray < SmFace*>   & rFacesToRemove,     // out: Faces    to delete from TgtBrep
                                  SmTArray < SmEdge*>   & rEdgesToRemove,     // out: Edges    to delete from TgtBrep
                                  SmTArray < SmVertex*> & rVerticesToRemove)  // out: Vertices to delete from TgtBrep
                                 { SM_REF5(pTgtBrep, dXSectTol3d, rFacesToRemove, rEdgesToRemove, rVerticesToRemove) ; SE( SM_ERR ); return SM_ERR ; }
  
  // list of Edges merged into TgtBrep from cutter during the merge cutter/TgtBrep XSect results into TgtBrep step
  SmTArray< SmEdge* > & GetCutterEdges()    { return m_apEdgesInCutter; }

  // utilities
  virtual void          Draw         (SmBrep * pTgtBrfep=NULL) const ;  
  virtual SM_TYPE       GetType      ()                        const { return SmCutter_TYPE; }  
  virtual SmBoolean     IsKindOf     (SM_TYPE t)               const { return( SmCutter_TYPE == t) ; }
  virtual const TCHAR * GetTypeString()                        const { return _T("SmCutter") ; }
  
} ; // end class SmCutter

/*******************************************************************//**
PURPOSE: The plane cutter object defines a cutting object which is
    an infinite plane going through the given point with the given normal.
    The portion of the Brep which is cut away is the portion on the
    side of the plane normal.

NOTES: 
***********************************************************************/
class SM_EXPORT SmPlaneCutter : public SmCutter
{
 protected:
  // inherited: SmTArray<SmEdge*> m_apEdgesInCutter ; // list of Edges merged into TgtBrep from cutter during the 
  //                                                  // merge cutter/TgtBrep XSect results into TgtBrep step.
  SmPoint3d                       m_vPlanePoint ;     // point on infinite cutting plane
  SmVector3d                      m_vPlaneNormal ;    // infinite cutting plane normal - geometry on the positive normal side of the plane is removed
  
 public:
  // constructor
  SmPlaneCutter(const SmPoint3d & crPlanePoint,
                const SmVector3d & crPlaneVector);
  
  // destructor
  virtual ~SmPlaneCutter() { };
  
  // Merge cutter/TgtBrep XSect results into TgtBrep. Pure virtual function
  virtual SmStatus MergeIntersection(SmBrep             * pTgtBrep,          // in : SmBrep to cut with this plane
                                     double               dXSectTol3d,       // in : Min dist between unique points
                                     SmTArray<SmFace *> * pOptSubsetFaces) ; // in : optional subset of faces, NULL to ignore, default:[NULL]
  
  // Create lists of AfterCut Faces, Edge, and Vertices to delete. Pure virtual function
  virtual SmStatus Classification(SmBrep                * pTgtBrep,            // in : target Brep
                                  double                  dXSectTol3d,         // in : Min dist between unique points
                                  SmTArray < SmFace*>   & rFacesToRemove,      // out: Faces    to delete from TgtBrep
                                  SmTArray < SmEdge*>   & rEdgesToRemove,      // out: Edges    to delete from TgtBrep
                                  SmTArray < SmVertex*> & rVerticesToRemove) ; // out: Vertices to delete from TgtBrep
                    
  // data access
  SmPoint3d       & GetPlanePoint()  { return m_vPlanePoint ;  }
  SmVector3d      & GetPlaneNormal() { return m_vPlaneNormal ; }
  
  // utilities
  virtual void          Draw         (SmBrep *pTgtBrep=NULL) const ;
  virtual SM_TYPE       GetType      ()                      const { return SmPlaneCutter_TYPE; }  
  virtual SmBoolean     IsKindOf     (SM_TYPE t)             const { return( (SmPlaneCutter_TYPE == t) 
                                                                            ? TRUE 
                                                                            : SmCutter::IsKindOf(t)) ; 
                                                                   }                                 
  virtual const TCHAR * GetTypeString()                      const { return _T("SmPlaneCutter") ; }

} ; // end class SmPlaneCutter

/*******************************************************************//**
PURPOSE: The BiLinear surface cutter object defines a cutting object 
    The portion of the Brep which is cut away is the portion on the
    side that the normal points.

NOTES: 
***********************************************************************/
class SM_EXPORT SmSurfaceBiLinearCutter : public SmCutter
{
 protected:
  // inherited: SmTArray<SmEdge*> m_apEdgesInCutter ;  // list of Edges merged into TgtBrep from cutter during the 
  //                                                   // merge cutter/TgtBrep XSect results into TgtBrep step.
  SmPoint3d          m_vP0 ;                       // Pt0 corner of BiLinear surface{Pt0, Pt1, Pt2, Pt3}
  SmPoint3d          m_vP1 ;                       // Pt1 corner of BiLinear surface{Pt0, Pt1, Pt2, Pt3}
  SmPoint3d          m_vP2 ;                       // Pt2 corner of BiLinear surface{Pt0, Pt1, Pt2, Pt3}
  SmPoint3d          m_vP3 ;                       // Pt3 corner of BiLinear surface{Pt0, Pt1, Pt2, Pt3}
  SmBSplineSurface * m_vSrf;                       // Bilinear surface defined by points {Pt0, Pt1, Pt2, Pt3}
  SmAttribute      * m_pCutterAttribute ;          // atttribute placed on cutter face before boolean to track cutter face children in TgtBrep
  
public:
  // constructor
  SmSurfaceBiLinearCutter(const SmPoint3d & P0, 
                          const SmPoint3d & P1,
                          const SmPoint3d & P2, 
                          const SmPoint3d & P3) ;
  
  // destructor
  virtual ~SmSurfaceBiLinearCutter();
  
  // Merge cutter/TgtBrep XSect results into TgtBrep. Pure virtual function
  virtual SmStatus MergeIntersection(SmBrep             * pTgtBrep,          // in : SmBrep to cut with this plane
                                     double               dXSectTol3d,       // in : Min dist between unique points
                                     SmTArray<SmFace *> * pOptSubsetFaces) ; // NotUsed: in : optional subset of faces, NULL to ignore, default:[NULL]
  
  // Create lists of AfterCut Faces, Edge, and Vertices to delete. Pure virtual function
  virtual SmStatus Classification(SmBrep                * pTgtBrep,            // in : target Brep
                                  double                  dXSectTol3d,         // in : Min dist between unique points
                                  SmTArray < SmFace*>   & rFacesToRemove,      // out: Faces    to delete from TgtBrep
                                  SmTArray < SmEdge*>   & rEdgesToRemove,      // out: Edges    to delete from TgtBrep
                                  SmTArray < SmVertex*> & rVerticesToRemove) ; // out: Vertices to delete from TgtBrep
                    
  // data access                     
  SmBSplineSurface * GetSurface() { return m_vSrf ; } 

  // utilities
  virtual void          Draw         (SmBrep * pTgtBrep=NULL) const;
  virtual SM_TYPE       GetType      ()                       const { return SmSurfaceBiLinearCutter_TYPE; }  
  virtual SmBoolean     IsKindOf     (SM_TYPE t)              const { return( (SmSurfaceBiLinearCutter_TYPE == t) 
                                                                            ? TRUE 
                                                                            : SmCutter::IsKindOf(t)) ; 
                                                                    }                                 
  virtual const TCHAR * GetTypeString()                       const { return _T("SmSurfaceBiLinearCutter") ; }

} ; // end class SmSurfaceBiLinearCutter

#endif // !__SMCUTTER_H__


