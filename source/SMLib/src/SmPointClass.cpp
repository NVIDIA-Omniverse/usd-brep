// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPointClass.cpp
* PURPOSE: Implementation of point classification methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmPointClass.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#include <SmVolume.h>
#include <SmSurfaceCache.h>
#include <SmAssertArray.h>
#include <SmAttribute.h>

/*******************************************************************//**
PURPOSE: ReSet all members to init values

NOTES: leaves m_cpContext alone, rule: SmPointClassifications are constructed with a m_cpContext value.
***********************************************************************/
void SmPointClassification::ReSet
 (SmZoneTol3d *pOptZoneTol3d)  // in : optional ZoneTol3d if known, NULL=SM_UNINIT_TOL                   
 { 
   m_ePointClass         = SM_PC_UNKNOWN ;   
   m_cpFace              = NULL ; 
   m_pObject             = NULL ;  
   m_sSrcZoneTol3d       = pOptZoneTol3d ? *pOptZoneTol3d : (SmZoneTol3d)SM_UNINIT_TOL ; // copy of (Pt/Ray/CurveBeingClassified)->ZoneTol3d ;

   m_dGap3d              = 0.0 ;   // min gap between obj being classified and m_pObject ;
   m_dTParam             = 0.0 ;  
   m_vUVParam.Set (0.0,0.0) ; 
   m_vUVWParam.Set(0.0,0.0,0.0) ; 
   m_bParamSet           = FALSE;
                         
   m_dPreSnapParam       = SM_BIG_DOUBLE ; 
   m_bSSSPointFlag       = FALSE ; 
   m_vSSSPoint.Set(0.0,0.0,0.0) ; 

 } // end SmPointClassification::ReSet

/*******************************************************************//**
PURPOSE: assignment operator

NOTES: 
***********************************************************************/
SmPointClassification & SmPointClassification::operator=          
 (SmPointClassification const & crPC)    
{ 
  // no work - same object
  if(&crPC == this) return *this ;

  // base
  // m_cpContext     = crPC.m_cpContext ;

  // members
  m_ePointClass   = crPC.m_ePointClass ; 
  m_cpFace        = crPC.m_cpFace ;
      
  m_pObject             = crPC.m_pObject ; 
  m_sSrcZoneTol3d       = crPC.m_sSrcZoneTol3d ;       // copy of (Pt/Ray/CurveBeingClassified)->ZoneTol3d ;
                        
  m_dGap3d              = crPC.m_dGap3d ;              // min gap between obj being classified and m_pObject ;
  m_dTParam             = crPC.m_dTParam ;     
  m_vUVParam            = crPC.m_vUVParam ;  
  m_vUVWParam           = crPC.m_vUVWParam ;
  m_bParamSet           = crPC.m_bParamSet ;
                        
  m_dPreSnapParam       = crPC.m_dPreSnapParam ;
  m_bSSSPointFlag       = crPC.m_bSSSPointFlag ;
  m_vSSSPoint           = crPC.m_vSSSPoint ;

  // all done    
  return *this ;

} // end  SmPointClassification::operator=

/*******************************************************************//**
PURPOSE: equality operator

NOTES: 
***********************************************************************/
SmBoolean SmPointClassification::operator==          
 (SmPointClassification const & crPC)    
{
  SmBoolean bRtn = TRUE ; 
  // no work - same object
  if(&crPC == this) return bRtn ;

  // base
  // m_cpContext     = crPC.m_cpContext ;

  // members
  bRtn &= m_ePointClass   == crPC.m_ePointClass ; 
  bRtn &= m_cpFace        == crPC.m_cpFace ;
      
  bRtn &= m_pObject             == crPC.m_pObject ; 
  bRtn &= m_sSrcZoneTol3d       == crPC.m_sSrcZoneTol3d ;       // copy of (Pt/Ray/CurveBeingClassified)->ZoneTol3d ;
                        
  bRtn &= m_dGap3d              == crPC.m_dGap3d ;              // min gap between obj being classified and m_pObject ;
  bRtn &= m_dTParam             == crPC.m_dTParam ;     
  bRtn &= m_vUVParam            == crPC.m_vUVParam ;  
  bRtn &= m_vUVWParam           == crPC.m_vUVWParam ;
  bRtn &= m_bParamSet           == crPC.m_bParamSet ;
                        
  bRtn &= m_dPreSnapParam       == crPC.m_dPreSnapParam ;
  bRtn &= m_bSSSPointFlag       == crPC.m_bSSSPointFlag ;
  bRtn &= m_vSSSPoint           == crPC.m_vSSSPoint ;

  // all done    
  return bRtn ;

} // end  SmPointClassification::operator==

/*******************************************************************//**
PURPOSE: copy everything but the point data

NOTES: 
***********************************************************************/
SmPointClassification & SmPointClassification::CopyClassificationOnly
 (SmPointClassification const & crPC)    // in : object being copied
{ 
  // no work - same object
  if(&crPC == this) return *this ;

  // base
  // skip m_cpContext compares

  // members
  m_ePointClass   = crPC.m_ePointClass ; 
  m_cpFace        = crPC.m_cpFace ; 

  m_pObject             = crPC.m_pObject ;     
  m_sSrcZoneTol3d       = crPC.m_sSrcZoneTol3d ;      // copy of (Pt/Ray/CurveBeingClassified)->ZoneTol3d ;

  m_dGap3d              = crPC.m_dGap3d ;              // min gap between obj being classified and m_pObject ;
  m_dTParam             = crPC.m_dTParam ;     
  m_vUVParam            = crPC.m_vUVParam ; 
  m_vUVWParam           = crPC.m_vUVWParam ;
  m_bParamSet           = crPC.m_bParamSet ;
                        
  m_dPreSnapParam       = crPC.m_dPreSnapParam ;

  // don't copy the SSSPoint data
  
  // all done   
  return *this ;

} // end SmPointClassification::CopyClassificationOnly

/*******************************************************************//**
PURPOSE: Get the Brep associated with the particular object
    in the point class.

NOTES: 
***********************************************************************/
SmBrep *SmPointClassification::GetBrep
  (SmBrep *pSkipBrep)             // in : only return Breps which are not the given Brep,
                                  //      NULL to ignore, default:[NULL]
 const
{ 
  SmBrep *pBrep = NULL ;
  
  switch(m_ePointClass)
    {
      case SM_PC_VERTEX  : pBrep = ((SmVertex*)m_pObject)->GetBrep();  break ;
      case SM_PC_EDGE    : pBrep = ((SmEdge*)m_pObject)->GetBrep();    break ;
      case SM_PC_FACE    : pBrep = ((SmFace*)m_pObject)->GetBrep();    break ;
      case SM_PC_EDGEUSE : pBrep = ((SmEdgeuse*)m_pObject)->GetBrep(); break ; 
      case SM_PC_CURVE   : pBrep =   ((SmCurve*)m_pObject)->GetOwner()
                                   ? ((SmEdge*)((SmCurve*)m_pObject)->GetEdge())->GetBrep() 
                                   : NULL ; break ;
      case SM_PC_SURFACE : pBrep =   ((SmSurface*)m_pObject)->GetOwner()
                                   ? ((SmFace*)((SmSurface*)m_pObject)->GetFace())->GetBrep() 
                                   : NULL ; break ;
      case SM_PC_VOLUME  : // no Brep here
      case SM_PC_POINT   : // no Brep here
      case SM_PC_NOTHING : // no Brep here
      case SM_PC_UNKNOWN : // no Brep here
      default            : pBrep = NULL ;
    }

  // skip specified breps
  if(pSkipBrep && pSkipBrep == pBrep) pBrep = NULL ;

  // all done
  return pBrep ;

} // end SmPointClassification::GetBrep

/*******************************************************************//**
PURPOSE: Get the Loop associated with the particular m_pobject
    in the point class.

NOTES: Return NULL when when m_pObject is not connected to a Loop
       or is NULL
***********************************************************************/
SmLoop *SmPointClassification::GetLoop() const
{ 
  SmLoop       * pLoop = NULL ;
  const SmFace * pFace = GetFace() ;

  // check state - when PointClassification is not currently associated with a Face - return NULL
  if(pFace == NULL)
    { return NULL ; }
  
  switch(m_ePointClass)
    {
      case SM_PC_VERTEX  : pLoop = ((SmVertex*)m_pObject)->GetLoopOfFace(pFace);  break ;
      case SM_PC_EDGE    : pLoop = ((SmEdge*)m_pObject)->GetLoopOfFace(pFace);    break ;
      case SM_PC_EDGEUSE : pLoop =   ((SmEdgeuse*)m_pObject)->GetLoopuse() 
                                   ? ((SmEdgeuse*)m_pObject)->GetLoopuse()->GetLoop() : NULL ; 
                                   break ; 
      case SM_PC_CURVE   : pLoop =   ((SmCurve*)m_pObject)->GetOwner()
                                   ? ((SmEdge*)((SmCurve*)m_pObject)->GetEdge())->GetLoopOfFace(pFace) : NULL ; 
                                   break ;
      case SM_PC_FACE    : // no unique Loop here
      case SM_PC_SURFACE : // no unique Loop here
      case SM_PC_VOLUME  : // no Loop here
      case SM_PC_POINT   : // no Loop here
      case SM_PC_NOTHING : // no Loop here
      case SM_PC_UNKNOWN : // no Loop here
      default            : pLoop = NULL ;
    }

  // all done
  return pLoop ;

} // end SmPointClassification::GetLoop

/*******************************************************************//**
PURPOSE: Set the point class type and the Object.

NOTES: 
***********************************************************************/
void SmPointClassification::SetClassObject
 (SmPointClassificationType ePointClass,  // in :
  SmObject                * pObject)      // in :
{
#ifdef SM_DEBUG_CODE
  if(pObject != NULL)                             
    {                                             
      switch( ePointClass ) 
        {
          case SM_PC_REGION : if(!pObject->IsKindOf(SmRegion_TYPE )) { SM_ASSERT(SM_ERR); } break;   
          case SM_PC_FACE   : if(!pObject->IsKindOf(SmFace_TYPE   )) { SM_ASSERT(SM_ERR); } break;   
          case SM_PC_EDGE   : if(!pObject->IsKindOf(SmEdge_TYPE   )) { SM_ASSERT(SM_ERR); } break;   
          case SM_PC_VERTEX : if(!pObject->IsKindOf(SmVertex_TYPE )) { SM_ASSERT(SM_ERR); } break;   
          case SM_PC_POINT  : if(!pObject->IsKindOf(SmPoint3d_TYPE)) { SM_ASSERT(SM_ERR); } break;   
          case SM_PC_CURVE  : if(!pObject->IsKindOf(SmCurve_TYPE  )) { SM_ASSERT(SM_ERR); } break;   
          case SM_PC_SURFACE: if(!pObject->IsKindOf(SmSurface_TYPE)) { SM_ASSERT(SM_ERR); } break;   
          case SM_PC_VOLUME : if(!pObject->IsKindOf(SmVolume_TYPE )) { SM_ASSERT(SM_ERR); } break;   
          case SM_PC_EDGEUSE: if(!pObject->IsKindOf(SmEdgeuse_TYPE)) { SM_ASSERT(SM_ERR); } break; 
          case SM_PC_UNKNOWN: // no checks to do
          case SM_PC_NOTHING: // no checks to do 
          default :  
                    break;                                                                           
        }                                                                                            
    } // end pObject existence check
#endif // SM_DEBUG_CODE

  m_ePointClass = ePointClass; 
  m_pObject     = pObject;

  if ( pObject == NULL )
    { m_ePointClass = SM_PC_UNKNOWN; }   // Should be safer this way.

  if ( ePointClass == SM_PC_POINT || ePointClass == SM_PC_VERTEX )
    { m_bParamSet = TRUE; }
  else
    { m_bParamSet = FALSE; }
   
} // SmPointClassification::SetClassObject                      

/*******************************************************************//**
PURPOSE: Set PointClassify->Object Param value when Object is an
         Edge or Curve.

NOTES: When the classification object has no curve, return NULL.
***********************************************************************/
void SmPointClassification::SetTParam      
  (double    dTParam,      // in :
   SmBoolean bSnapToIvl)   // in : TRUE = snap dTParam to Ivl End point if needed
                           //      default:[FALSE] = No Snapping, output WARN when dTParam is outside of associated Ivl
{ 
#ifdef SM_DEBUG_CODE
  if(m_pObject)
    {
      SmBoolean bCheck = FALSE ; 
      SmExtent1d sIvl ;
      if     (m_ePointClass == SM_PC_EDGE ) { sIvl = ((SmEdge *)m_pObject)->GetInterval() ; bCheck = TRUE ; }
      else if(m_ePointClass == SM_PC_CURVE) { sIvl = ((SmCurve *)m_pObject)->GetNaturalInterval() ; bCheck = TRUE ; }

      if(bCheck)
        {
          if(FALSE == sIvl.ContainsValue(dTParam))
            {
              if(bSnapToIvl)
                { dTParam = sIvl.SnapValue(dTParam) ; }
              else
                { SM_ASSERT_BREAK_MSG(bCheck == FALSE || sIvl.ContainsValue(dTParam, SM_EFF_ZERO), 
                                      _T("SmPointClassification::SetTParam: Setting TParam to a value outside the Edge/Curve interval")) ; 
                }
            } // end dTParam not contained in Ivl check
        } // end associated Ivl existence check
    } // end associated obj existence check
#else
    SM_REF1(bSnapToIvl);
#endif // SM_DEBUG_CODE

   m_dTParam   = dTParam;    
   m_bParamSet = TRUE; 
} // end SmPointClassification::SetTParam

/*******************************************************************//**
PURPOSE: Set PointClassify->Object Param value when Object is an
         Edge or Curve.

NOTES: When the classification object has no curve, return NULL.
***********************************************************************/
void SmPointClassification::SetUVParam     
 (const SmPoint2d & crUVParam)           
{ 
#ifdef SM_DEBUG_CODE
  if(m_pObject)
    {
      SmBoolean bCheck = FALSE ; 
      SmExtent2d sUVDomain ;
      if     (m_ePointClass == SM_PC_FACE )   { sUVDomain = ((SmFace *)m_pObject)->GetUVDomain() ; bCheck = TRUE ; }
      else if(m_ePointClass == SM_PC_SURFACE) { sUVDomain = ((SmSurface *)m_pObject)->GetNaturalUVDomain() ; bCheck = TRUE ; }
      SM_ASSERT_BREAK_MSG(bCheck == FALSE || sUVDomain.ContainsPoint2d(crUVParam), _T("SmPointClassification::SetUVParam: Setting SetUVParam to a value outside the Face/Surface UVDomain")) ; 
    }
#endif // SM_DEBUG_CODE

  m_vUVParam  = crUVParam;  
  m_bParamSet = TRUE; 

} // end SmPointClassification::SetUVParam

/*******************************************************************//**
PURPOSE: Return the curve associated with the object to which this
  point is classified.

NOTES: When the classification object has no curve, return NULL.
***********************************************************************/
SmCurve *SmPointClassification::GetCurveObject()  
  const 
{
  return  (m_ePointClass == SM_PC_EDGE    && m_pObject) ? ((SmEdge*)m_pObject)->GetCurve()
         :(m_ePointClass == SM_PC_CURVE)                ? (SmCurve*)m_pObject
         :(m_ePointClass == SM_PC_EDGEUSE && m_pObject) ? ((SmEdgeuse*)m_pObject)->GetEdge()->GetCurve()
         : NULL ;            

} // end SmPointClassification::GetCurveObject

/*******************************************************************//**
PURPOSE: Return the surface associated with the object to which this
  point is classified.

NOTES: When the classification object has no surface, return NULL.
***********************************************************************/
SmSurface *SmPointClassification::GetSurfaceObject()  
  const 
{ 
  return  (m_ePointClass == SM_PC_FACE && m_pObject) ? ((SmFace*)m_pObject)->GetSurface()
         :(m_ePointClass == SM_PC_SURFACE)           ? (SmSurface*)m_pObject
         : NULL ;           

} // end SmPointClassification::GetSurfaceObject

/*******************************************************************//**
PURPOSE: Return the volume associated with the object to which this
  point is classified.

NOTES: When the classification object has no volume, return NULL.
***********************************************************************/
SmVolume *SmPointClassification::GetVolumeObject()  
  const 
{ 
  return (m_ePointClass == SM_PC_VOLUME) ? (SmVolume*)m_pObject : NULL ;

} // end SmPointClassification::GetVolumeObject

/*******************************************************************//**
PURPOSE: Return either this PointClass' Obj/ClassifyCrv full Gap3d size or
         or the size of the Gap3d component tangent to an optionally specified
         surface.
           

NOTES: When pOptSurface    NULL = rtn entire m_pGap3d size
            pOptSurface NotNULL = rtn size of m_pGap3d component perp to pOptSurface Normal
                                  at the PtClassify's DropToSurf Point.
                                  That's the 'InSurface' component of the gap vector
                                  when PtClassify is intended to lie on pOptSurface.
            default:[pOptSurface = NULL]
***********************************************************************/
double SmPointClassification::GetGap3d
 (const SmSurface * pOptSurface)  // in : pOptSurface    NULL = rtn entire m_pGap3d dist
                                  //      pOptSurface NotNULL = rtn component of m_pGap3d dist perp to pOptSurface Normal
                                  //      default:[NULL]
  const 
{ 
  // low work - full 3d gaps - no gap vector projection requested
  if(pOptSurface == NULL)
    { return m_dGap3d ; }

  // else - in surface gaps
  else
    {
      // Get Object Point
      SmExtent2d sUVDomain = pOptSurface->GetNaturalUVDomain() ;
      SmPoint3d sObjPt3d ; 
      FindObjPoint3d(sObjPt3d) ;
      SmZoneTol3d sObjZoneTol3d = GetObjZoneTol3d() ;

      // find SurfaceDropPt for ObjPt
      SmSolution sData[16];
      SmSolutionArray sSolutions(16,sData);
      SE(pOptSurface->GlobalPointSolve(sUVDomain,
                                       SM_SO_INTERSECT, 
                                       sObjPt3d, 
                                       sObjZoneTol3d, 
                                       NULL, 
                                       SM_SR_SINGLE, 
                                       sSolutions)) ;
     

      // when a distinct solution was not found
      if (sSolutions.GetSize() != 1) 
        {
          // find point/face->surface intersection point to 100 * face->Tolerance
          SE(pOptSurface->GlobalPointSolve(sUVDomain,
                                           SM_SO_INTERSECT, 
                                           sObjPt3d, 
                                           sObjZoneTol3d*100, // then, allow classified object gaps larger than XSectTol3d(SrcZoneTol3d,ToObject) value
                                           NULL, 
                                           SM_SR_SINGLE, 
                                           sSolutions)) ;

          // still no solutions - return an error
          if (sSolutions.GetSize() != 1) 
            {
              return SM_ERR;
            }
        }
      
      SmPoint2d sUV(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]) ;
      SmPoint3d sSurfPt3d ;
      SmPoint3d sSurfNorm ;
      pOptSurface->EvaluatePoint(sUV, sSurfPt3d) ;
      pOptSurface->EvaluateNormal(sUV, TRUE, TRUE, sSurfNorm) ;

      SmVector3d sGap3d       = sObjPt3d - sSurfPt3d ; 
      SmVector3d sInSurfGap3d = (sGap3d - sGap3d.Dot(sSurfNorm) * sSurfNorm) ;

      return sInSurfGap3d.Length() ; 
    } // end point classifies to a face branch

} // end SmPointClassification::GetGap3d

/*******************************************************************//**
PURPOSE: TRUE when ClassifyObject connects to ConnectedTarget through
            the topology graph
NOTES:
***********************************************************************/
SmBoolean SmPointClassification::IsConnectedTo
 (const SmTopology *cpConnectTgt)  
 const 
{ 
  // locals
  SmObject* pClassifyObject     = this->GetObject() ;
  ULONG     pClassifyObjectType = pClassifyObject ? pClassifyObject->GetType() : 0 ; 

  // for the first CurveInterval Start - add any PointClassification when
  SmBoolean bRtn = (   ((pClassifyObjectType == SmVertex_TYPE)    && (((SmVertex*)pClassifyObject)->IsConnectedTo(cpConnectTgt)))
                    || ((pClassifyObjectType == SmEdge_TYPE)      && (((SmEdge*)pClassifyObject)->IsConnectedTo(cpConnectTgt)))
                    || ((pClassifyObjectType == SmFace_TYPE)      && (((SmFace*)pClassifyObject)->IsConnectedTo(cpConnectTgt)))
                    || ((pClassifyObjectType == SmLoop_TYPE)      && (((SmLoop*)pClassifyObject)->IsConnectedTo(cpConnectTgt))) 
                    || ((pClassifyObjectType == SmVertexuse_TYPE) && (((SmVertexuse*)pClassifyObject)->IsConnectedTo(cpConnectTgt)))
                    || ((pClassifyObjectType == SmEdgeuse_TYPE)   && (((SmEdgeuse*)pClassifyObject)->IsConnectedTo(cpConnectTgt))) 
                    || ((pClassifyObjectType == SmFaceuse_TYPE)   && (((SmFaceuse*)pClassifyObject)->IsConnectedTo(cpConnectTgt)))
                    || ((pClassifyObjectType == SmLoopuse_TYPE)   && (((SmLoopuse*)pClassifyObject)->IsConnectedTo(cpConnectTgt))) 
                    ) ;
  // all done
  return(bRtn) ;

} // end SmPointClassification::IsConnectedTo

/*******************************************************************//**
PURPOSE: Return the PointClassification with the highest precidence.

NOTES:
  PointClassification precidence order:

      VERTEX with smallest m_dGap3d value 
      VERTEX
      EDGE
      FACE
      REGION
      ALL OTHERS (error case)

  The function does not generally really combine point Classifications.  It
  chooses one pointClassification object over the other based
  on the above precidence hierarchy.

SURF/SURF/SURF TOLERANCE REDUCTION HEURISTIC:
  When the prefered point classification bSSSPointFlag is not set
  and the rejected point classification bSSSPointFlag is set, 
  the bSSSPointFlag and vSSSPoint values are copied to the prefered 
  object, and if that object maps to a vertex is used to set the 
  vertex's location.
***********************************************************************/
const SmPointClassification & SmPointClassification::Combine
 (const SmPointClassification & crPointClass) 
{
  // locals
  SmBoolean bNewPointFlag = FALSE ;

  // save lowest dimension object or object with smallest gap

  // When Point is classified to a Topology object - see if NewPoint values should be saved
  //  rules: 1. save the vertex with the smallest gap
  //         2. save the lowest dim topology object seen
  //         3. ignore equal dim topology objects except for vertices
  //              GWC: These rules are based on supporting the Boolean operation in which an intersection
  //                   curve is classified against a Brep.  If that Intersection curve happens to intersect
  //                   a vertex (or Edge) it will also intersect all the edges and faces (or just faces) connected
  //                   to that vertex (or Edge).  In the set of all objects intersected by an XSectCurve at one
  //                   XSectCurve parameter value there will be one unique lowest-dimension object and 
  //                   any number of higher dimension objects.  For the Boolean to complete it only needs to
  //                   know that one lowest-dim topology object. The rules in SmPointClassification::Combine()
  //                   attempt to identify and return that single lowest-dimension topology object per
  //                   single SxectCurve param value intersecting Topology object set.
  //
  //                   That all works when there are no tolerances.  With tolerances, I see two different kinds
  //                   of problems that would confuse the Boolean operation.  First, the CurveParameter
  //                   at which the intersections happen for one set of connected topology objects may 
  //                   spread out making it hard to figure out which set of XSectCurve/Brep intersections 
  //                   belong together and worse yet, if the XSectCurve happens to intersect the Brep twice
  //                   at distances close to the XSectTol3d values, then the intersection results from those
  //                   two distinct intersections may be mixed together in unpredictable manners and can even
  //                   make it possible for there to be more than one lowest-dimension 
  //                   topology object in the set of XSectCurve/Brep intersections
  //                   at one XSectCurve param value.  
  //                  
  //                   Ex: imagine a curve being classified to a Brep as part of the Boolean operation.
  //                   An intersection curve passes between two unconnected edges that happen to be seperated by 
  //                   a distance not much larger than XSectTol but the intersection curve happens to 
  //                   lie wihtin XSectTol3d tol distance to both Edges.  I can imagine different cases
  //                   of this. case 1: The XSectCurve is close to two edges approaching a common vertex but
  //                   is beyond XSectTol3d distance to that common vertex.  Case 2:  The two edges within 
  //                   XSectTol3d of the XSectCurve at the same XSectCurve param value do not share any common vertices.
  //                   Those two edges just happen to get close to one another.  Additionally, under these
  //                   circumstances 
  //                   the right classification to make to allow the Boolean operation to complete successfully
  //                   might be one edge or the other depending on how the
  //                   Boolean operation was trying to connect up the resultBrep's topology graph.  Or, the Boolean
  //                   operator curve should connect the XSectCurve to both edges 
  //                   by adding a vertex on the XSectCurve's point of intersection
  //                   and connecting both Brep Edges in question to that vertex.  
  //
  //                   The point is: I don't think the information
  //                   being passed to SmPointClassification::Combine is enough to figure out how to resolve
  //                   that conflict. A better approach is needed here.
  bNewPointFlag =     ((m_ePointClass == SM_PC_VERTEX)  && (   crPointClass.m_ePointClass == SM_PC_VERTEX
                                                            && crPointClass.m_pObject     != m_pObject
                                                            && crPointClass.m_dGap3d      <  m_dGap3d)) 
                   || ((m_ePointClass == SM_PC_EDGEUSE) && (crPointClass.m_ePointClass == SM_PC_VERTEX))
                   || ((m_ePointClass == SM_PC_EDGE)    && (crPointClass.m_ePointClass == SM_PC_VERTEX))
                   || ((m_ePointClass == SM_PC_FACE)    && (   crPointClass.m_ePointClass == SM_PC_VERTEX 
                                                            || crPointClass.m_ePointClass == SM_PC_EDGE 
                                                            || crPointClass.m_ePointClass == SM_PC_EDGEUSE))
                   || ((m_ePointClass == SM_PC_REGION)  && (   crPointClass.m_ePointClass == SM_PC_VERTEX 
                                                            || crPointClass.m_ePointClass == SM_PC_EDGE 
                                                            || crPointClass.m_ePointClass == SM_PC_EDGEUSE
                                                            || crPointClass.m_ePointClass == SM_PC_FACE)) ;


  // when PointClass is classified to a geometry object - see if new point values should be saved
  bNewPointFlag |=    ((m_ePointClass == SM_PC_UNKNOWN))
                   || ((m_ePointClass == SM_PC_POINT)   && (   crPointClass.m_ePointClass == SM_PC_POINT
                             && crPointClass.m_pObject     != m_pObject
                                                            && crPointClass.m_dGap3d      <  m_dGap3d)) 
                   || ((m_ePointClass == SM_PC_CURVE)   && (   crPointClass.m_ePointClass == SM_PC_POINT))
                   || ((m_ePointClass == SM_PC_SURFACE) && (   crPointClass.m_ePointClass == SM_PC_POINT 
                                                            || crPointClass.m_ePointClass == SM_PC_CURVE))
                   || ((m_ePointClass == SM_PC_VOLUME)  && (   crPointClass.m_ePointClass == SM_PC_POINT 
                                                            || crPointClass.m_ePointClass == SM_PC_CURVE
                                                            || crPointClass.m_ePointClass == SM_PC_SURFACE)) ;

// GWC:CHANGE_SSS
  // see if SSSPointData should be propagated
  if(   bNewPointFlag                == FALSE
     && crPointClass.m_bSSSPointFlag == TRUE
     && this->m_bSSSPointFlag        == FALSE)
    {
      // copy the SSSPointData
      this->m_bSSSPointFlag = TRUE ;
      this->m_vSSSPoint     = crPointClass.m_vSSSPoint ;

      // set vertex position when appropriate
      if(this->m_ePointClass == SM_PC_VERTEX)
        {
          SmVertex *pVertex = (SmVertex *)m_pObject ;
          pVertex->SetPoint(m_vSSSPoint) ;
        }

    } // end need to propagate SSSpointData check
// end GWC:CHANGE_SSS

  // all done
  return(bNewPointFlag ? crPointClass : *this) ;

} // end SmPointClassification::Combine

/*******************************************************************//**
PURPOSE: project crPoint to m_pObject and save m_dGap3d 
            and m_dTParam, m_vUVParam, or m_vUVWParam 

NOTES: crPoint is intended to be the 3DPoint which
  classified to a face or edge and this function simplifies
  setting the rest of the internal values.

  modifies m_dGap3d              = min distance to m_pObject from crPoint
           m_dTParam     when m_ePointClass == SM_PC_EDGE or SM_PC_CURVE
           m_vUVParam    when m_ePointClass == SM_PC_FACE or SM_PC_SURFACE
           m_vUVWParam   when m_ePointClass == SM_PC_VOLUME
           m_bParamSet         Set to True after other values are set
***********************************************************************/
SmStatus SmPointClassification::ComputePointParameters
  (const SmPoint3d & crPoint,              // in : Point being classified to object 
   double            dOptClassifyCrvParam) // NotUsed: in : Optional ClassifyCrvParam when known
                                           //      SM_UNDEF_DOUBLE = not given, default:[SM_UNDEF_DOUBLE]
//   SmPoint2d sAssociatedSurfUV)
{
  SM_REF1(dOptClassifyCrvParam) ; 
  // init side effect values
  m_dGap3d     = 0.0 ;
  m_dTParam    = 0.0 ;
  m_vUVParam. Set(0, 0) ;
  m_vUVWParam.Set(0, 0, 0) ;

  // locals
  SmCurve   * pObjCurve   = NULL ;
  SmSurface * pObjSurface = NULL ;
  SmVolume  * pObjVolume  = NULL ;
  SM_TOL_LINE SmXSectTol3d sXSectTol3d    = SmTol::GetXSectTol3d(SmTol::GetZoneTol3d(m_pObject), m_sSrcZoneTol3d) ;
  SM_TOL_LINE SmXSectTol3d sXSectTol3d100 = sXSectTol3d*100 ;
  SmExtent1d    sIvl ;
  SmExtent2d    sUVDomain ;

  // branch on m_ePointClass to get geometry object and domain for upcoming point drop
  if     (m_ePointClass == SM_PC_CURVE)   { pObjCurve    = (SmCurve *)m_pObject ;
                                            sIvl      =  pObjCurve->GetNaturalInterval() ;                                      
                                          }
  else if(m_ePointClass == SM_PC_SURFACE) { pObjSurface  = (SmSurface *)m_pObject ;
                                            sUVDomain =  pObjSurface->GetNaturalUVDomain() ;
                                          }
  else if(m_ePointClass == SM_PC_VOLUME)  { pObjVolume   = (SmVolume *)m_pObject ; 
                                          }
  else if(m_ePointClass == SM_PC_EDGEUSE) { pObjCurve    = ((SmEdgeuse*)m_pObject)->GetEdge()->GetCurve() ;    
                                            sIvl      = ((SmEdgeuse*)m_pObject)->GetEdge()->GetInterval() ;
                                          }
  else if(m_ePointClass == SM_PC_EDGE)    { pObjCurve    = ((SmEdge*)m_pObject)->GetCurve() ;    
                                            sIvl      = ((SmEdge*)m_pObject)->GetInterval() ;
                                          }
  else if(m_ePointClass == SM_PC_FACE)    { pObjSurface  = ((SmFace*)m_pObject)->GetSurface() ;
                                            sUVDomain = ((SmFace*)m_pObject)->GetUVDomain() ;
                                          }
  else if(m_ePointClass == SM_PC_POINT)   { /* no work */ }
  else if(m_ePointClass == SM_PC_VERTEX)  { /* no work */ }

  // when point classifies to an edge or curve branch
  if(pObjCurve)
    {
      // project point to edge->Curve to get dGap3d, dParam, and dDist
      SmBoolean bSuccess;
      double    dParam, dDist;
      SER(pObjCurve->DropPoint(sIvl,            // in : target curve allowed domain
                            crPoint,         // in : Point to drop to curve
                            NULL,            // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                             //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                             //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                            sXSectTol3d100,  // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                             //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                             //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                             //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                            NULL,            // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                            bSuccess,        // out: TRUE = found a drop point
                            dParam,          // out: found drop curve param
                            dDist)) ;        // out: found drop distance
                                             // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                             //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                             //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                             //      default:[SM_SO_MINIMIZE] to preserve original behavior

      // If we fail then for now we'll error out in the
      // future we may want to do a minimization.
      if (!bSuccess) { return SM_ERR; }

      // save param and deviation
      m_dGap3d     = dDist ;
      m_dTParam    = dParam;
      m_bParamSet  = TRUE  ;

    } // end point classifies to an edge branch

  // when point classifies to a face or surface branch
  else if(pObjSurface) 
    {
      // find point/face->surface intersection point to face->Tolerance
      SmSolution sData[16];
      SmSolutionArray sSolutions(16,sData);
      SER(pObjSurface->GlobalPointSolve(sUVDomain,
                                     SM_SO_INTERSECT, 
                                     crPoint, 
                                     m_sSrcZoneTol3d,  // first look for intersecting surface solutions
                                     NULL, 
                                     SM_SR_SINGLE, 
                                     sSolutions));

      // when a distinct solution was not found
      if (sSolutions.GetSize() != 1) 
        {
          // find point/face->surface intersection point to 100 * face->Tolerance
          SER(pObjSurface->GlobalPointSolve(sUVDomain,
                                         SM_SO_INTERSECT, 
                                         crPoint, 
                                         m_sSrcZoneTol3d*100, // then, allow classified object gaps larger than XSectTol3d(SrcZoneTol3d,ToObject) value
                                         NULL, 
                                         SM_SR_SINGLE, 
                                         sSolutions));

          // still no solutions - return an error
          if (sSolutions.GetSize() != 1) 
            {
              return SM_ERR;
            }
        }
      
      // save param and deviation
      m_dGap3d     = sSolutions[0].m_vStart.m_dSolutionValue;
      m_vUVParam   = SmPoint2d(sSolutions[0].m_vStart[0],sSolutions[0].m_vStart[1]);
      m_bParamSet  = TRUE;

    } // end point classifies to a face branch

  // when point classifies to a volume branch
  else if(pObjVolume) 
    {
      // find point/face->surface intersection point to face->Tolerance
      SmBoolean bFoundAnswer = FALSE ;
      SmSolutionArray sSolutionArray ;

      SmExtent3d sParamDomain = pObjVolume->GetNaturalParamDomain();
      SER(pObjVolume->GlobalPointSolve(crPoint, bFoundAnswer, sSolutionArray, &sParamDomain));

      // when a distinct solution was not found
      if (bFoundAnswer == FALSE) 
        {
          return SM_ERR;
        }
      
      // save param and deviation
      SmSolution sSolution = sSolutionArray[0] ;
      m_dGap3d     = sSolution.m_vStart.m_dSolutionValue;
      m_vUVWParam  = SmPoint3d(sSolution.m_vStart[0],sSolution.m_vStart[1],sSolution.m_vStart[2]);
      m_bParamSet  = TRUE;

    } // end point classifies to a face branch

  // all done
  return SM_SUCCESS;

} // end SmPointClassification::ComputePointParameters

/*******************************************************************//**
PURPOSE: After ComputePointParameters() has run
         use m_pObect and the stored param values to compute
         and return m_pObject's 3d position to which this PointClassification
         object is classified.

NOTES: If not classified, return SM_ERR.
***********************************************************************/
SmStatus SmPointClassification::FindObjPoint3d
 (SmPoint3d & rPoint)  // out: Point to set
  const
{
  rPoint.SetUninitialized();

  if ( ! this->AreParametersSet() )
    { return SM_ERR; }

  // switch on classification type 
  switch( m_ePointClass )
    {
      case SM_PC_POINT  : { SmPoint3d * pPt  = (SmPoint3d*) m_pObject;              NER( pPt );
                            rPoint           = *pPt;
                            return SM_SUCCESS;
                          }
      case SM_PC_CURVE  : { SmCurve   * pCrv = SM_CAST_PTR( SmCurve, m_pObject );   NER( pCrv );
                            return pCrv->EvaluatePoint( m_dTParam, rPoint );
                          }
      case SM_PC_SURFACE: { SmSurface * pSrf = SM_CAST_PTR( SmSurface, m_pObject ); NER( pSrf );
                            return pSrf->EvaluatePoint( m_vUVParam, rPoint );
                          }
      case SM_PC_VOLUME : { SmVolume  * pVol = SM_CAST_PTR( SmVolume, m_pObject );  NER( pVol );
                            return pVol->EvaluatePoint( m_vUVWParam, rPoint );
                          }
      case SM_PC_VERTEX : { SmVertex  * pVtx = SM_CAST_PTR( SmVertex, m_pObject );  NER( pVtx );
                            rPoint           = pVtx->GetPoint();
                            return SM_SUCCESS;
                          }
      case SM_PC_EDGE   : { SmEdge    * pEdge = SM_CAST_PTR( SmEdge, m_pObject );   NER( pEdge );
                            SmCurve   * pCrv  = pEdge->GetCurve();                  NER( pCrv );
                            return pCrv->EvaluatePoint( m_dTParam, rPoint );
                          }
      case SM_PC_EDGEUSE: { SmEdgeuse * pEU   = SM_CAST_PTR( SmEdgeuse, m_pObject );NER( pEU );
                            SmCurve   * pCrv  = pEU->GetEdge()->GetCurve();         NER( pCrv );
                            return pCrv->EvaluatePoint( m_dTParam, rPoint );
                          }
      case SM_PC_FACE   : { SmFace    * pFace = SM_CAST_PTR( SmFace, m_pObject );   NER( pFace );
                            SmSurface * pSrf  = pFace->GetSurface();                NER( pSrf );
                            return pSrf->EvaluatePoint( m_vUVParam, rPoint );
                          }

      case SM_PC_UNKNOWN:
      case SM_PC_NOTHING:
      case SM_PC_REGION :
      default:
          SE( SM_ERR );
          return SM_ERR;

    } // end switch
} // end SmPointClassification::FindObjPoint3d

/*******************************************************************//**
PURPOSE: Merge (insert) this pointClassification's point into its
   m_pObject creating a vertex if one does not already exist.

   If a vertex was created, 
     set PointClassification->Object to equal new Vertex.  
     update m_sObjectZoneTol3d

   If a vertex was merged with this VertexPoint
     update m_pObject->ZoneTol3d
     update m_sObjectZoneTol3d

NOTES:
   When point is classified to 
   SM_PC_VERTEX: SmObject is SmVertex_TYPE.
                 Pos: Set Vertex new position = minimize new tol increase
                 Tol: Increase existing vertex ZoneTol3d if needed so that 
                       final vertex local neighborhood includes initial vertex and
                       SrcPoint's local neighborhoods.
   SM_PC_EDGE  : SmObject is SmEdge_TYPE.
                 Call SmBrep::MakeVertexSplitEdge() - create new vertex and edge pair.
                 Tol: Increase new vertex ZoneTol3d if needed so that 
                       final new vertex local neighborhood includes SrcPoint's local neighborhood.
   SM_PC_FACE  : SmObject is SmFace_TYPE.
                 Call SmBrep::MakeVerteLoop() - create new vertex and vertexLoop.
                 Tol: Increase new vertex ZoneTol3d if needed so that 
                       final new vertex local neighborhood includes SrcPoint's local neighborhood.
   SM_PC_REGION: SmObject is SmRegion_TYPE
                 Call SmBrep::MakeShellVertex()
                 Tol: Increase new vertex ZoneTol3d if needed so that 
                       final new vertex local neighborhood includes SrcPoint's local neighborhood.

   Vertices are placed at the precomputed m_vSSSPoint location when 
   the SmPointClassification has one and an attribute is attached to the
   newVertex so that its edges can be forced through the point at a
   later time.
***********************************************************************/
SmStatus SmPointClassification::MergeIntoObject
 (const SmPoint3d     & sSrcPoint,          // in : 3D loc on some source object to use for new (or combined) vertex locations 
  SmTArray<SmEdge*>   * pOptNewEdges,       // out: 2 edges from splitting existing edge (1 reused, 1 new)
  SmTArray<SmVertex*> * pOptNewVertices)    // out: New Vertex, if any
{
#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      this->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  const SmObject         * pObject       = this->GetObject(); NER(pObject) ;
  SmVertex               * pNewV         = NULL ;
  SM_TOL_LINE SmZoneTol3d  sSrcZoneTol3d = SmTol::GetSrcZoneTol3d(this) ; // When classifying a point against a Brep - copy of Point's ZoneTol3d value
                                                                          // When classifying a ray   against a Brep - copy of Ray's   ZoneTol3d value
                                                                          // When classifying a curve against a Brep - copy of Curve's ZoneTol3d value
                                                                          // uninit:[SM_UNINIT_TOL]

  // when the point is classified into a region - insert a NewVertex
  if (pObject->GetType() == SmRegion_TYPE)
    {
      // Put a single shell vertex into the middle of the region.
      SmRegion * pR    = SM_CAST_PTR( SmRegion, this->GetObject() );
      SmBrep   * pBrep = pR->GetBrep();
      SmShell  * pNewS = NULL;
      SER(pBrep->MakeShellVertex(pR,sSrcPoint,pNewS,pNewV));

      // save output
      if(pOptNewVertices) 
        { pOptNewVertices->Add(pNewV); }

    } // end point classified to a region branch 

  // when the point is classified to a face
  else if (pObject->GetType() == SmFace_TYPE)
    {
      // Put a single vertex loop into the middle of the face.
      SmFace   *pF    = SM_CAST_PTR( SmFace, this->GetObject() );
      SmBrep   *pBrep = pF->GetBrep();
      SmLoop   *pNewL = NULL;
      SER(pBrep->MakeVertexLoop(pF,sSrcPoint,pNewL,pNewV));

      // update output
      if (pOptNewVertices) { pOptNewVertices->Add(pNewV); }

    } // end point classified to a face branch

  // when the point is classified to an edge
  else if (pObject->GetType() == SmEdge_TYPE)
    {
      // Split the edge to make a vertex.
      SmEdge   * pE     = SM_CAST_PTR( SmEdge, this->GetObject() );
      SmBrep   * pBrep  = pE->GetBrep() ;
      double     dParam = GetTParam() ;    // note: sSrcPoint expected to = Edge->Evaluate(TParam) 
      SmEdge   * pNewE1 = NULL, * pNewE2 = NULL;

      // split edge when split point is not near edge
      if(SM_SUCCESS != pBrep->MakeVertexSplitEdge(pE,     // in : target edge
                                                  dParam, // in : target parameter
                                                  pNewE1, // out: new Edge1 (by chance == pEdgeToSplit) or NULL
                                                  pNewE2, // out: new Edge2 (by chance == newly allocated edge) or NULL
                                                  pNewV)) // out: new vertex (newly allocated) or nearby endVertex or NULL
        {
          // Handle case where vertex was too close to end to split edge
          if (!pNewV) 
            SER(SM_ERR);
          m_ePointClass = SM_PC_VERTEX;
          m_pObject     = pNewV;
          return SM_SUCCESS;
        }

      // save outputs
      if (pOptNewEdges)    { pOptNewEdges->Add(pNewE1);
                             pOptNewEdges->Add(pNewE2);
                           }
      if (pOptNewVertices) { pOptNewVertices->Add(pNewV);
                           }

    } // end point classified to an edge branch

  // when the point is classified to a Vertex
  else if (pObject->GetType() == SmVertex_TYPE)
    {
      // Almost no work to do. There is already a vertex here.
      // Next: Update Vertex Position and Tolerance so that final Vertex local neighborhood
      //       includes initial vertex and the SrcPoint's local neighborhoods.
      pNewV = SM_CAST_PTR( SmVertex, this->GetObject() ); NER( pNewV );

    } // end PointClass == SM_PC_VERTEX branch

  // else error - point is not classified to  Vertex, Edge, Face, or Region
  else
    {
      SER(SM_ERR);
    }

  // arrive here when pNewV = new or found Vertex that classifies to SrcPoint
  // next: update pNewV pos and tol so that SSSPoint locations are favored
  //              and pNewV final local neighborhood includes 
  //              the pNewV initial and sSrcPoint's local neighborhoods 

  // local
  SmVector3d  sInitVPoint = pNewV->GetPoint() ;

  // update this pointClassification
  SetClassObject( SM_PC_VERTEX, pNewV );

  // GWC:CHANGE_SSS
  // when the PointClassification has a SSSPoint
  if(m_bSSSPointFlag)
    {
      // set the vertex to the SSSPoint location
      pNewV->SetPoint(m_vSSSPoint) ;

#ifdef SM_DEBUG_CODE
      // remember vertex position came from SSSPoint - this is not persistent
      if(NULL == pNewV->FindAttribute(SM_AI_SSSPOINT))
        {
          SmAttribute *pAtt = new (*pNewV->GetContext()) SmAttribute(SM_AI_SSSPOINT, SM_AB_TEMP) ;
          pNewV->AddAttribute(pAtt) ;
        }
#endif // SM_DEBUG_CODE

    } // end m_vSSSPoint existence check

  // adjust vertex tolerance as needed
  SmVector3d               sNewVPoint     = pNewV->GetPoint() ;
  SM_TOL_LINE SmScaledZero sScaledZero    = SmTol::GetScaledZero(*pNewV) ;
  SM_TOL_LINE SmZoneTol3d  sNewVZoneTol3d = SmTol::GetZoneTol3d(pNewV) ;
  SmVector3d               sGap3d         = sNewVPoint - sSrcPoint ;
  double                   dGapSize       = sGap3d.Length() ;

#ifdef SM_DEBUG_CODE
  SM_TOL_LINE SmZoneTol3d sObjZoneTol3d = SmTol::GetObjZoneTol3d(this) ;
  SM_ASSERT_MSG(sNewVZoneTol3d >= sObjZoneTol3d, _T("SmPointClassification::MergeIntoObject - expected NewV Tolerance to at least as big as ObjTolerance")) ;
#endif // SM_DEBUG_CODE

  // remember when vertex is representing two different exact points
  pNewV->SetExact(dGapSize < sScaledZero) ; // TRUE = pNewVertex is representing just one point location
                                            // FALSE= pNewVertex is representing a pair of distince locations

  // adjust the Vertex Tol so final Vertex local neighborhood includes init vertex and srcPt local neighborhoods
  SmVector3d  sFinalPos ;
  SmZoneTol3d sFinalZoneTol3d ;
  if     (dGapSize < sScaledZero)                     { // use VertexPt and max ZoneTol3d
                                                        sFinalPos       = sNewVPoint ;
                                                        sFinalZoneTol3d = smos_Max(sSrcZoneTol3d, sNewVZoneTol3d) ; 
                                                      } 
  else if(sSrcZoneTol3d >= sNewVZoneTol3d + dGapSize) { // use SrcPt location    and ZoneTol3d
                                                        sFinalPos       = sSrcPoint ; 
                                                        sFinalZoneTol3d = sSrcZoneTol3d ; 
                                                      }
  else if(sNewVZoneTol3d >= sSrcZoneTol3d + dGapSize) { // use VertexPt location and ZoneTol3d  
                                                        sFinalPos       = sNewVPoint ; 
                                                        sFinalZoneTol3d = sNewVZoneTol3d ; 
                                                      }
  else                                                { // place final vertex location to minimize final ZoneTol3d size
                                                        //  while making final local neighborhood include both input local neighborhoods
                                                        sGap3d /= dGapSize ;
                                                        sFinalPos       = 0.5 * (  sSrcPoint  - sSrcZoneTol3d * sGap3d
                                                                                 + sNewVPoint + sNewVZoneTol3d * sGap3d) ;
                                                        sFinalZoneTol3d = 0.5 * (sSrcZoneTol3d + dGapSize + sNewVZoneTol3d) ; 
                                                      }
  // adjust the Vertex position and Tolerance
  pNewV->SetPoint(sFinalPos) ;
#ifdef SM_USE_OLDTOL
  SM_OLDTOL_LINE pNewV->SetTolerance(sFinalZoneTol3d) ;
#endif // SM_USE_OLDTOL

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      // draw Brep(blue), PointClass(cyan), VertexPoint(red), PointClass->Vertex(green)
      SmBrep *pBrep = GetBrep() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,1) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,5, 0,0,1) ; sSrcPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,7, 0,1,0) ; sInitVPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,9, 1,0,0) ; sNewVPoint.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmPointClassification::MergeIntoObject

/*******************************************************************//**
PURPOSE: Load the point class object from a solution which contains
    an intersection.    

NOTES: The input lOffset argument is used to determine where 
    to start in the solutionEnd.m_adParameters array to pick up 
    parameter values for the object, e.g TParam when cpObject is a curve and
    UVParams when cpObject is a surface.

    Can't use this function to load the this SmPointClassification object
    with point solutions because the class SmPoint3d (SmVector3d) is not
    derived from SmObject.  If you are loading a SmCurveClassification from
    GlobalPointSolutions, then just set the Deviation value from the m_dSolutionValue.
***********************************************************************/
SmStatus SmPointClassification::LoadFromIntersection
  (SmZoneTol3d           sSrcZoneTol3d,   // in : ZoneTol3d of Src in XSect(Src,Obj), e.g. the SmCurveClassification::SrcZoneTol3d val
   SmObject            * cpObject,        // in : Obj in the XSect(Src,Obj) call that produced crSolutionend
   const SmSolutionEnd & crSolutionEnd,   // in : contains solution and parameter values for intersection
   ULONG                 lOffset,         // in : index into crSolutionEnd.m_adParameters array for sol's 1st cpObject param 
   double                dPreSnapParam)   // in : sol val remembered when Point sol comes from a curve/object solution 
{
  // GWC: sadly, can't do the SmPoint3d case here because SmPoint3d is not derived from SmObject
  //      else if(cpObject->IsKindOf(SmPoint3d_TYPE)){ this->SetClassObject( SM_PC_POINT, cpObject );
  //                                                   this->SetGap3d( crSolutionEnd.m_dSolutionValue );
  //                                                 }

  // tolerance copy
  this->SetSrcZoneTol3d(sSrcZoneTol3d) ;
 
  if     (cpObject->IsKindOf(SmCurve_TYPE))  { this->SetClassObject( SM_PC_CURVE, cpObject );
                                               this->SetGap3d( crSolutionEnd.m_dSolutionValue );
                                               this->SetTParam( crSolutionEnd[lOffset] );
                                             }
 
  else if(cpObject->IsKindOf(SmSurface_TYPE)){ this->SetClassObject( SM_PC_SURFACE, cpObject );
                                               this->SetGap3d( crSolutionEnd.m_dSolutionValue );
                                               this->SetUVParam( SmPoint2d(crSolutionEnd[  lOffset],
                                                                           crSolutionEnd[1+lOffset] ));
                                             }
  else if(cpObject->IsKindOf(SmVolume_TYPE)) { this->SetClassObject( SM_PC_VOLUME, cpObject );
                                               this->SetGap3d( crSolutionEnd.m_dSolutionValue );
                                               this->SetUVWParam( SmPoint3d(crSolutionEnd[  lOffset],
                                                                            crSolutionEnd[1+lOffset],
                                                                            crSolutionEnd[2+lOffset] ));
                                             }
  else if(cpObject->IsKindOf(SmVertex_TYPE)) { this->SetClassObject( SM_PC_VERTEX, cpObject );
                                               this->SetGap3d( crSolutionEnd.m_dSolutionValue );
                                             }
  else if(cpObject->IsKindOf(SmEdge_TYPE))   { this->SetClassObject( SM_PC_EDGE, cpObject );
                                               this->SetGap3d( crSolutionEnd.m_dSolutionValue );
                                               this->SetTParam( crSolutionEnd[lOffset] );
                                             }
 
  else if(cpObject->IsKindOf(SmFace_TYPE))   { this->SetClassObject( SM_PC_FACE, cpObject );
                                               this->SetGap3d( crSolutionEnd.m_dSolutionValue );
                                               this->SetUVParam( SmPoint2d(crSolutionEnd[  lOffset],
                                                                           crSolutionEnd[1+lOffset] ));
                                             }
  else
    { SER(SM_ERR); }

  // save the preSnapParam value when Point solution comes from a curve/object solution
  this->SetPreSnapParam(dPreSnapParam) ;

  return SM_SUCCESS;

} // SmPointClassification::LoadFromIntersection

/*******************************************************************//**
PURPOSE: local implementation of Virtual SmObject::Dump() declaration 

NOTES: 
***********************************************************************/
void SmPointClassification::Dump() const
{
  // pass the call along using defaults
  Dump(3, SM_BIG_DOUBLE, NULL, NULL, NULL) ;

} // end SmPointClassification::Dump() of SmObject virtual Dump

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertPointClassification_list[] =
{
  /*  0 */ {SM_AT_VALUES,  _T("Bad m_ePointClass"),    _T("m_ePointClass == SM_PC_UNKNOWN when m_pObject != NULL") },
  /*  1 */ {SM_AT_VALUES,  _T("Uninit SrcZoneTol3d"),  _T("m_sSrcZoneTol3d is Uninit") },
  /*  2 */ {SM_AT_VALUES,  _T("Bad SrcZoneTol3d"),     _T("m_sSrcZoneTol3d is less than SM_EFF_ZERO") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
**********************************************************************/
SmBoolean SmPointClassification::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  SM_REF1(pTestRequests);

  // init rtn value
  SmBoolean bRtn  = TRUE ;

  // check simple members
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pObject == NULL || m_ePointClass != SM_PC_UNKNOWN), _T("") ) ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (m_sSrcZoneTol3d != SM_UNINIT_TOL), _T("") ) ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (m_sSrcZoneTol3d  > SM_EFF_ZERO || m_sSrcZoneTol3d == SM_UNINIT_TOL), _T("") ) ;

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmPointClassification::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmPointClassification::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // NotUsed: in : AssertArray holding rAReport
// {
//   SM_REF1(pAList) ;
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       ERR_MSG(_T("AssertHeal() error: virtual SmPointClassification::AssertHeal() was not the SmClass that generated this report - Logic Bug.")) ;
//        
//       return( FALSE ) ;
//     }
// 
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ;
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;
//                rAReport.m_pHealMessage = _T("SmCurveClassification::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmPointClassification::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmPointClassification::Dump
  (ULONG             lPointDim,      // in : Classification Point dimension, 2 or 3
   double            dClassifyParam, // in : SmCurveInterval Param associated with this Point, SM_BIG_DOUBLE to ignore
   const SmPoint3d * pPoint,         // in : Point being classified
   const SmSurface * pSurface1,      // in : 1st Surf of surf/surf xSect when Classification curve is an intersection curve
   const SmSurface * pSurface2)      // in : 2nd Surf of surf/surf xSect when Classification curve is an intersection curve
 const                               //      (NULL whenever lCurveDim == 2
                                     //       or Curve not defined by an intersection)
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  SM_ASSERT(lPointDim == 2 || lPointDim == 3) ;

  // locals for curvePoint/Surface1 distance
  SmSolutionArray sSolutions;
  double      dMaxDropDist  = 1.0 ;
  double      dDist1        = 0.0 ;
  double      dDist2        = 0.0 ;
  double      dGap3d        = m_dGap3d ;
  SmZoneTol3d sObjZoneTol3d = GetObjZoneTol3d() ;  

  // find Point/Surface1 distance
  if(lPointDim == 3 && pSurface1)
    {
      // get point/surf1 dist without checking trim boundaries
      sSolutions.ReSet() ; 
      SmSurfaceCache *pSurfaceCache = smsurf_GetSurfaceCache(pSurface1);
      SM_ASSERT_MSG(pSurfaceCache != NULL, _T("SmPointClassification::Dump - found a surface that didn't create a Surface cache- needs debugging")) ; 
      
      // Turn off point testing so GlobalPointSolve() will keep all point solutions
      //   without classifying the solution point against the trim boundaries.
      SmTemporaryChangeValue<SmBoolean> sChange(pSurfaceCache->m_bPointTestEnabled,FALSE);
      pSurface1->GlobalPointSolve(pSurface1->GetNaturalUVDomain(),
                                  SM_SO_MINIMIZE,
                                  *pPoint,
                                  dMaxDropDist,
                                  &dMaxDropDist,
                                  SM_SR_ALL,
                                  sSolutions) ;
      if(sSolutions.GetSize() == 0)
           { // only output error messages for intersection curves
             //   curves being merged from one brep to another due to
             //   coincident surfaces or fillets are often longer than
             //   the surface's domain in the other brep.  So not
             //   dropping under those circumstances is ok.
             if(pSurface1 && pSurface2)
               { SM_DBG_WARN(_T("XSectCurvePoint failed to drop to Surface1")) ; }
           }
      else { dDist1 = sSolutions[0].m_vStart.m_dSolutionValue ;
           }
    } // end pSurface1 existence check

  // find Point/Surface2 distance
  if(lPointDim == 3 && pSurface2)
    {
      // get point/surf1 dist without checking trim boundaries
      sSolutions.ReSet() ; 
      SmSurfaceCache *pSurfaceCache = smsurf_GetSurfaceCache(pSurface2);
      
      // Turn off point testing so GlobalPointSolve() will keep all point solutions
      //   without classifying the solution point against the trim boundaries.
      SmTemporaryChangeValue<SmBoolean> sChange(pSurfaceCache->m_bPointTestEnabled,FALSE);
      pSurface2->GlobalPointSolve(pSurface2->GetNaturalUVDomain(),
                                  SM_SO_MINIMIZE,
                                  *pPoint,
                                  dMaxDropDist,
                                  &dMaxDropDist,
                                  SM_SR_ALL,
                                  sSolutions) ;
      if(sSolutions.GetSize() == 0)
           { // only output error messages for intersection curves
             //   curves being merged from one brep to another due to
             //   coincident surfaces or fillets are often longer than
             //   the surface's domain in the other brep.  So not
             //   dropping under those circumstances is ok.
             if(pSurface1 && pSurface2)
               { SM_DBG_WARN(_T("XSectCurvePoint failed to drop to Surface2")) ; }
           }
      else { dDist2 = sSolutions[0].m_vStart.m_dSolutionValue ;
           }
    } // end pSurface1 existence check

  // switch on PointClass Type to output data
  switch (m_ePointClass) 
    {
      case SM_PC_UNKNOWN:
          smos_sprintf(sBuff,       _T("     Unknown: (InitVal=Not yet mapped or doesn't map to a target), SrcZoneTol3d:[%16.16lg]"), (double)m_sSrcZoneTol3d);
          smos_WriteBuffer(sBuff);
          break;

      case SM_PC_NOTHING:
          smos_sprintf(sBuff,       _T("     On Nothing: (known not to map to any target at all), SrcZoneTol3d:[%16.16lg]"), (double)m_sSrcZoneTol3d);
          smos_WriteBuffer(sBuff);
          break;

      case SM_PC_POINT:
          smos_sprintf(sBuff,       _T("     On Point:[0x%p], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], PtZoneTol3d:[%16.16lf]"),
                     m_pObject, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d) ;

          smos_sprintf(sBuffForFile,_T("     On Point:[%s], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], PtZoneTol3d:[%16.16lf]"),
                     m_pObject ? _T("notNULL") : _T("NULL"), dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d) ;

          break;

      case SM_PC_CURVE:
          if ( m_bParamSet )
          {
              smos_sprintf(sBuff,       _T("     On Curve:[0x%p], T:[%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], CrvZoneTol3d:[%16.16lg]"),
                         m_pObject,m_dTParam, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Curve:[%s], T:[%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], CrvZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"),m_dTParam, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          else
          {
              smos_sprintf(sBuff,       _T("     On Curve:[0x%p], T:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], CrvZoneTol3d:[%16.16lg]"),
                         m_pObject, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Curve:[%s], T:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], CrvZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"), dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          break;

      case SM_PC_SURFACE:
          if ( m_bParamSet )
          {
              smos_sprintf(sBuff,       _T("     On Surface:[0x%p], UV:[%16.16lf,%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], SrfZoneTol3d:[%16.16lg]"),
                         m_pObject,m_vUVParam.x,m_vUVParam.y, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Surface:[%s], UV:[%16.16lf,%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], SrfZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"),m_vUVParam.x,m_vUVParam.y, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          else
          {
              smos_sprintf(sBuff,       _T("     On Surface:[0x%p], UV:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], SrfZoneTol3d:[%16.16lg]"),
                         m_pObject, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Surface:[%s], UV:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], SrfZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"), dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          break;

      case SM_PC_VOLUME:
          if ( m_bParamSet )
          {
              smos_sprintf(sBuff,       _T("     On Volume:[0x%p], UVW:[%16.16lf,%16.16lf,%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], VolZoneTol3d:[%16.16lg]"),
                         m_pObject,m_vUVWParam.x,m_vUVWParam.y,m_vUVWParam.z, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Volume:[%s], UV:[%16.16lf,%16.16lf,%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], VolZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"),m_vUVWParam.x,m_vUVWParam.y,m_vUVWParam.z, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          else
          {
              smos_sprintf(sBuff,       _T("     On Volume:[0x%p], UVW:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], VolZoneTol3d:[%16.16lg]"),
                         m_pObject, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Volume:[%s], UV:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], VolZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"), dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          break;

      case SM_PC_VERTEX:
          smos_sprintf(sBuff,       _T("     On Vertex:[0x%p], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], VtxZoneTol3d:[%16.16lg]"),
                     m_pObject, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          smos_sprintf(sBuffForFile,_T("     On Vertex:[%s], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], VtxZoneTol3d:[%16.16lg]"),
                     m_pObject ? _T("notNULL") : _T("NULL"), dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          break;

      case SM_PC_EDGE:
          if ( m_bParamSet )
          {
              smos_sprintf(sBuff,       _T("     On Edge:[0x%p], T:[%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], EdgeZoneTol3d:[%16.16lg]"),
                         m_pObject,m_dTParam, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Edge:[%s], T:[%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], EdgeZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"),m_dTParam, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          else
          {
              smos_sprintf(sBuff,       _T("     On Edge:[0x%p], T:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], EdgeZoneTol3d:[%16.16lg]"),
                         m_pObject, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Edge:[%s], T:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], EdgeZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"), dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          break;

      case SM_PC_FACE:
          if ( m_bParamSet )
          {
              smos_sprintf(sBuff,       _T("     On Face:[0x%p], UV:[%16.16lf,%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], FaceZoneTol3d:[%16.16lg]"),
                         m_pObject,m_vUVParam.x,m_vUVParam.y, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Face:[%s], UV:[%16.16lf,%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], FaceZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"),m_vUVParam.x,m_vUVParam.y, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          else
          {
              smos_sprintf(sBuff,       _T("     On Face:[0x%p], UV:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], FaceZoneTol3d:[%16.16lg]"),
                         m_pObject, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Face:[%s], UV:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], FaceZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"), dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          break;

      case SM_PC_REGION:
          smos_sprintf(sBuff,       _T("     In Region:[0x%p]"),m_pObject);
          smos_sprintf(sBuffForFile,_T("     In Region:[%s]"),m_pObject ? _T("notNULL") : _T("NULL"));
          break;

      case SM_PC_EDGEUSE:
          if ( m_bParamSet )
          {
              smos_sprintf(sBuff,       _T("     On Edgeuse:[0x%p], T:[%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], EdgeuseZoneTol3d:[%16.16lg]"),
                         m_pObject,m_dTParam, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Edgeuse:[%s], T:[%16.16lf], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], EdgeuseZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"),m_dTParam, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          else
          {
              smos_sprintf(sBuff,       _T("     On Edgeuse:[0x%p], T:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], EdgeuseZoneTol3d:[%16.16lg]"),
                         m_pObject, dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
              smos_sprintf(sBuffForFile,_T("     On Edgeuse:[%s], T:[ Unset ], Dev:[%16.16lf], SrcZoneTol3d:[%16.16lg], EdgeuseZoneTol3d:[%16.16lg]"),
                         m_pObject ? _T("notNULL") : _T("NULL"), dGap3d, (double)m_sSrcZoneTol3d, (double)sObjZoneTol3d);
          }
          break;

      default:
          SE(SM_ERR);

    } // end Switch

    smos_WriteBuffer(sBuff, sBuffForFile);

  // dump curve parameter when given
  if(dClassifyParam != SM_BIG_DOUBLE)
    { smos_sprintf(sBuff,_T(", for CrvIvl Param:[%16.16lf]"), dClassifyParam) ;
      smos_WriteBuffer(sBuff);
    }
      
  // dump ClassificationPoint/IntersectionSurface distances
  if(pSurface1 || pSurface2) { smos_WriteBuffer(_T(", Point/Surf ")) ; }
  if(pSurface1) { smos_sprintf(sBuff,_T("Gap1:[%16.16lf]"), dDist1) ;
                  smos_WriteBuffer(sBuff);
                }
  if(pSurface1 && pSurface2) { smos_WriteBuffer(_T(", ")) ; }
  if(pSurface2) { smos_sprintf(sBuff,_T("Gap2:[%16.16lf] "), dDist2) ;
                  smos_WriteBuffer(sBuff);
                }

  // dump snap state if requested
  if(   dClassifyParam  != SM_BIG_DOUBLE
     && m_dPreSnapParam != SM_BIG_DOUBLE)
    {
      smos_sprintf(sBuff,_T(", Snap:[%16.16lf]"),
                 smos_Fabs(dClassifyParam - m_dPreSnapParam)) ;
      smos_WriteBuffer(sBuff);
    }

  // dump SSSPoint data
  if(m_bSSSPointFlag)
    {
      smos_sprintf(sBuff,_T(", SSSPOINT:[%16.16lf,%16.16lf,%16.16lf]\n"),
                 m_vSSSPoint.x, 
                 m_vSSSPoint.y, 
                 m_vSSSPoint.z) ;
      smos_WriteBuffer(sBuff);
   }
  else
    {
      smos_WriteBuffer(_T(", NO SSSPoint\n")) ;
    }

} // end SmPointClassification::Dump

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmDisplayList * SmPointClassification::Draw(void) const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start or continue using currently open display list
  smgfx_Open(smgfx_GetRuleColor());
  
  // draw point and edge, vertex, or face to which point is classified
  switch (m_ePointClass) 
    {

      case SM_PC_CURVE:   { SmCurve *pCurve = (SmCurve*)m_pObject;
                            pCurve->Draw();
                            if(m_bSSSPointFlag == FALSE) { pCurve->DrawAt(m_dTParam,0); }
                          }
                          break;
      case SM_PC_SURFACE: { SmSurface *pSurface = (SmSurface*)m_pObject;
                            pSurface->DrawUV();
                            if(m_bSSSPointFlag == FALSE) { pSurface->DrawAt(m_vUVParam); }
                          }
                          break;
      case SM_PC_VOLUME:  { SmVolume *pVolume = (SmVolume*)m_pObject;
                            pVolume->DrawUVW();
                            if(m_bSSSPointFlag == FALSE) { pVolume->DrawAtParamPoint(m_vUVWParam); }
                          }
                          break;
      case SM_PC_VERTEX:  { SmVertex *pVertex = (SmVertex*)m_pObject;
                            if(m_bSSSPointFlag == FALSE) { pVertex->Draw(); }
                          }
                          break;
                          
      case SM_PC_EDGE:    { SmEdge *pEdge = (SmEdge*)m_pObject;
                            pEdge->Draw();
                            if(m_bSSSPointFlag == FALSE) { pEdge->GetCurve()->DrawAt(m_dTParam); }
                          }
                          break;
      case SM_PC_FACE:    { SmFace *pFace = (SmFace*)m_pObject;
                            pFace->Draw(SM_DM_CROSSHATCH,8,8);
                            if(m_bSSSPointFlag == FALSE) { pFace->GetSurface()->DrawAt(m_vUVParam) ; }
                          }
                          break;

      case SM_PC_UNKNOWN:
      case SM_PC_REGION:
      case SM_PC_POINT:
      case SM_PC_EDGEUSE: 
      default:            break;

    } // end switch on m_ePointClass Classification object type

  // draw SSSPoint when appropriate
  if(m_bSSSPointFlag) 
    {
      m_vSSSPoint.Draw() ; 
    }

  // all done
  pRtn = smgfx_Close() ;

#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmPointClassification::Draw
