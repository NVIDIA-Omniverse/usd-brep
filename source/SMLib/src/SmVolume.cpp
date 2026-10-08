// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmVolume.cpp 
* PURPOSE: Implementation of SmVolume methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmVolume.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#include <SmAttribute.h>
#include <SmBSplineVolume.h>
#include <SmTransform.h>  
#include <SmTwistVolume.h>
#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>
#include <SmTArray.h>
#include <SmLocalSolve1d.h>
#include <SmLocalSolveNd.h>
#include <SmGeomUtility.h>
#include <SmGlobalSolver.h>
#include <SmSolutionArray.h>
#include <SmDatabaseIO.h>
#include <SmCrvInVolume.h>
#include <SmSrfInVolume.h>
#include <SmGraphicsVertexArray.h>
#include <SmPseudoBox.h>
#include <SmLine.h>
#include <SmPlane.h>
#include <SmNurbsCrv.h>
#include <SmAssertArray.h>
#include <SmBendVolume.h>
#include <SmUnbendVolume.h>


/*******************************************************************//**
File Local Declarations
***********************************************************************/

/*******************************************************************//**
PURPOSE: drop a 3d Vector into its base components given a set
 of 3 vectors that span the 3d space.  The base vectors need not be
 otrhogonal nor unit-length.

NOTES:  
***********************************************************************/
SmStatus smvol_DropVector 
  (const SmVector3d & rDU,               // in : Volume U_dir tangent 
   const SmVector3d & rDV,               // in : Volume V_dir tangent
   const SmVector3d & rDW,               // in : Volume V_dir tangent
   const SmVector3d & rDropVec3d,        // in : vector to drop
   SmVector3d       & rUVW) ;            // out: resulting UVW vector  [u v w]

/*******************************************************************//**
PURPOSE: constructor

NOTES:  
***********************************************************************/
SmVolume::SmVolume
 (const SmContext * cpContext,           // in : context for this new object
  SmObject        * pOwner,              // in : Owner value                                                                                       
  SmVolume        * pNextMap,            // in : Opt NextMap of CompoundMaps, NULL to ignnore. default:[NULL]                                                                         
  ULONG             lNextOwnerFlag,      // in : 0 = don't copy m_pNextMap when constructed, don't delete m_pNextMap when destructed               
                                         //      1 = copy m_pNextMap when constructed, delete m_pNextMap when destructed                           
                                         //      2 = don't copy m_pNextMap when constructed, delete m_pNextMap when destructed                     
                                         //      default:[0]                                                                                                     
  SmTransform     * pOrientMap,          // in : Opt Orient Map. NULL=IdentityMatrix. default:[NULL]
  ULONG             lOrientOwnerFlag)    // in : 0 = don't copy m_pOrientMap when constructed, don't delete m_pOrientMap when destructed
                                         //      1 = copy m_pOrientMap when constructed, delete m_pOrientMap when destructed
                                         //      2 = don't copy m_pOrientMap when constructed, delete m_pOrientMap when destructed
                                         //      default:[0]
: m_pOwner(pOwner),
  m_pNextMap(pNextMap),
  m_lNextOwnerFlag(lNextOwnerFlag),
  m_pOrientMap(pOrientMap),
  m_pInvOrientMap(NULL),
  m_lOrientOwnerFlag(lOrientOwnerFlag)            
{
  // propagate the contexts - OK for now because Volumes have no cache
  if(cpContext) { SetContext(cpContext) ;
                  if(pNextMap)   { pNextMap->SetContext(cpContext) ; }
                  if(pOrientMap) { pOrientMap->SetContext(cpContext) ; } 
                }

  // when asked - copy pNextMap
  if(pNextMap && lNextOwnerFlag == 1) 
    {
      m_pNextMap = NULL ;
      SmContext * pContext =   GetContext() ? (SmContext *)GetContext()
                             : pNextMap->GetContext() ? (SmContext *)pNextMap->GetContext() 
                             : NULL ;
      if(pContext == NULL)
        {
          WARN(_T("SmVolume constructor could not copy pNextMap because a cpContext could not be found")) ;
        }
      else
        { 
          pNextMap->Copy(*pContext, m_pNextMap) ;
        } 
    }
 
  // OrientMap
  if(pOrientMap)
    {
      // when asked - copy pOrientMap
      if(lOrientOwnerFlag == 1) 
        {
          m_pOrientMap = NULL ;
          SmContext * pContext =   GetContext() ? (SmContext *)GetContext()
                                 : pOrientMap->GetContext() ? (SmContext *)pOrientMap->GetContext() 
                                 : NULL ;

          if(pContext == NULL)
            {
              WARN(_T("SmVolume constructor could not copy pOrientMap because a cpContext could not be found")) ;
            }
          else
            { 
              SmVolume *pVolume = NULL ;
              pOrientMap->Copy(*pContext, pVolume) ;
              m_pOrientMap = (SmTransform *) pVolume ;
            }
        } // end asked to copy pOrientMap

      // set the inverse orient map
      RefreshInvOrientMap() ;

    } // end pOrientMap existence check

  SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL) ;
 
} // end SmVolume::SmVolume constructor

/*******************************************************************//**
PURPOSE: copy constructor

NOTES:  
***********************************************************************/
SmVolume::SmVolume
 (const SmVolume & crSourceVolume,  // in : SourceVolume to copy
  SmBoolean        bSimpleMapOnly)  // in : TRUE = Copy this Volume omitting any compounding volumes
                                    //      FALSE= Copy this Volumes with any compounding volumes    
: SmAObject(crSourceVolume), 
  m_pOwner(crSourceVolume.m_pOwner),
  m_pNextMap        (crSourceVolume.m_pNextMap),
  m_lNextOwnerFlag  (crSourceVolume.m_lNextOwnerFlag),
  m_pOrientMap      (crSourceVolume.m_pOrientMap),
  m_pInvOrientMap   (crSourceVolume.m_pInvOrientMap),
  m_lOrientOwnerFlag(crSourceVolume.m_lOrientOwnerFlag)             
{ 
  // when asked 
  if(bSimpleMapOnly)
    {
      // omit any NextMap making the copy a 'simple' map
      m_pNextMap = NULL ;
    }
  // else when there is a NextMap and it's independently owned
  else if(m_pNextMap && m_lNextOwnerFlag >= 1) 
    { 
      // copy NextMap keeping the copy a 'compound' map
      m_pNextMap           = NULL ; 
      SmContext * pContext =   GetContext() ? (SmContext *)GetContext()
                             : crSourceVolume.m_pNextMap->GetContext() ? (SmContext *)crSourceVolume.m_pNextMap->GetContext() 
                             : NULL ;

      if(pContext == NULL)
        {
          WARN(_T("SmVolume copy constructor could not copy pNextMap because a cpContext could not be found")) ;
        }
      else
        { 
          crSourceVolume.m_pNextMap->Copy(*pContext, m_pNextMap) ;
        } 
    }

  // copy OrientMap when it's independently owned
  if(m_pOrientMap && m_lOrientOwnerFlag >= 1) 
    { 
      m_pOrientMap        = NULL ; 
      SmContext * pContext =   GetContext() ? (SmContext *)GetContext()
                             : crSourceVolume.m_pOrientMap->GetContext() ? (SmContext *) crSourceVolume.m_pOrientMap->GetContext() 
                             : NULL ;

      if(pContext == NULL)
        {
          WARN(_T("SmVolume copy constructor could not copy pOrientMap because a cpContext could not be found")) ;
        }
      else
        { 
          SmVolume *pVolume = NULL ;
          crSourceVolume.m_pOrientMap->Copy(*pContext, pVolume) ;
          m_pOrientMap = (SmTransform *)pVolume ; 
        }
    }

  // always copy InvOrientMap
  if(m_pInvOrientMap) 
    { 
      m_pInvOrientMap   = NULL ; 
      SmContext *pContext =   GetContext() ? (SmContext *)GetContext()
                            : crSourceVolume.m_pInvOrientMap->GetContext() ? (SmContext *) crSourceVolume.m_pInvOrientMap->GetContext() 
                            : NULL ;

      if(pContext == NULL)
        {
          WARN(_T("SmVolume copy constructor could not copy pInvOrientMap because a cpContext could not be found")) ;
        }
      else
        { 
          SmVolume *pVolume = NULL ;
          crSourceVolume.m_pInvOrientMap->Copy(*pContext, pVolume) ; 
          m_pInvOrientMap   = (SmTransform *)pVolume ;
        }
    }

  // inform the attributes
  Notify(SM_NO_CONSTRUCTION, this, (SmVolume*)&crSourceVolume, NULL);      
  
} // end SmVolume::SmVolume copy constructor

/*******************************************************************//**
PURPOSE: assignment operator

NOTES:  
***********************************************************************/
SmVolume &SmVolume::operator= (const SmVolume & crVolume)
{
  if(&crVolume == this)
    { return *this ; }
  
  // make initial shallow copies
  m_pOwner        = crVolume.m_pOwner ;
  m_pNextMap      = crVolume.m_pNextMap    ;
  m_pOrientMap    = crVolume.m_pOrientMap  ;
  m_pInvOrientMap = crVolume.m_pInvOrientMap ;

  // convert shallow to deep copies when appropriate

  // copy NextMap when it's independently owned
  if(m_pNextMap && m_lNextOwnerFlag >= 1) 
    { 
      m_pNextMap           = NULL ; 
      SmContext * pContext =   GetContext() ? (SmContext *)GetContext()
                             : crVolume.m_pNextMap->GetContext() ? (SmContext *)crVolume.m_pNextMap->GetContext() 
                             : NULL ;

      if(pContext == NULL)
        {
          WARN(_T("SmVolume copy constructor could not copy pNextMap because a cpContext could not be found")) ;
        }
      else
        { 
          crVolume.m_pNextMap->Copy(*pContext, m_pNextMap) ;
        } 
    }

  // copy OrientMap when it's independently owned
  if(m_pOrientMap && m_lOrientOwnerFlag >= 1) 
    { 
      m_pOrientMap        = NULL ; 
      SmContext * pContext =   GetContext() ? (SmContext *)GetContext()
                             : crVolume.m_pOrientMap->GetContext() ? (SmContext *) crVolume.m_pOrientMap->GetContext() 
                             : NULL ;

      if(pContext == NULL)
        {
          WARN(_T("SmVolume copy constructor could not copy pOrientMap because a cpContext could not be found")) ;
        }
      else
        { 
          SmVolume *pVolume = NULL ;
          crVolume.m_pOrientMap->Copy(*pContext, pVolume) ;
          m_pOrientMap = (SmTransform *)pVolume ; 
        }
    }

  // always copy InvOrientMap
  if(m_pInvOrientMap) 
    { 
      m_pInvOrientMap   = NULL ; 
      SmContext *pContext =   GetContext() ? (SmContext *)GetContext()
                            : crVolume.m_pInvOrientMap->GetContext() ? (SmContext *) crVolume.m_pInvOrientMap->GetContext() 
                            : NULL ;

      if(pContext == NULL)
        {
          WARN(_T("SmVolume copy constructor could not copy pInvOrientMap because a cpContext could not be found")) ;
        }
      else
        { 
          SmVolume *pVolume = NULL ;
          crVolume.m_pInvOrientMap->Copy(*pContext, pVolume) ; 
          m_pInvOrientMap   = (SmTransform *)pVolume ;
        }
    }

  // inform the attributes
  ((SmVolume)crVolume).Notify(SM_NO_COPY, this, SM_NO_GET_OWNER(this), SM_NO_GET_OWNER(&crVolume)) ;

  // all done
  return(*this) ;
  
} // end SmVolume::operator= 

/*******************************************************************//**
PURPOSE: destructor

NOTES:  
***********************************************************************/
SmVolume::~SmVolume()                            
{ 
  Notify(SM_NO_DESTRUCTION, this, NULL, NULL) ; 
   
  if(m_pNextMap      && m_lNextOwnerFlag   >= 1) { delete m_pNextMap ;      } m_pNextMap      = NULL ;
  if(m_pOrientMap    && m_lOrientOwnerFlag >= 1) { delete m_pOrientMap ;    } m_pOrientMap    = NULL ;
  if(m_pInvOrientMap)                            { delete m_pInvOrientMap ; } m_pInvOrientMap = NULL ;

} // SmVolume::~SmVolume destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmVolume

NOTES: At this level, volumes of the same type and set of NextMaps are equivalent. 
       Virtual functions will do the additional needed volume specific testing.
***********************************************************************/
SmBoolean SmVolume::operator==
  (const SmVolume& crOther) 
 const
{
  // low work
  if( this == &crOther) 
    { return TRUE ; }

  // when objects are equivalent
  SmBoolean bRtn = (// same derived types
                       GetType() == crOther.GetType()        

                    // and same next maps
                    && (   ( m_pNextMap == crOther.m_pNextMap)
                        || ( m_pNextMap == NULL && crOther.m_pNextMap == NULL)
                        || (   m_pNextMap != NULL && crOther.m_pNextMap != NULL
                            && *m_pNextMap == *crOther.m_pNextMap))

                    // and same orient maps
                    && (   ( m_pOrientMap == crOther.m_pOrientMap)
                        || ( m_pOrientMap == NULL && crOther.m_pOrientMap == NULL)
                        || (   m_pOrientMap != NULL && crOther.m_pOrientMap != NULL
                            && *m_pOrientMap == *crOther.m_pOrientMap))        
                   ) ;

  // all done
  return bRtn ;

} // end SmVolume::operator==

/*******************************************************************//**
PURPOSE: convenience access to m_pOrient Origin

NOTES: 
***********************************************************************/
SmVector3d SmVolume::GetOrientOrigin() const               
{ 
  if(m_pOrientMap) 
    { return( m_pOrientMap->GetToDisp() ) ; }
  else             
    { SmVector3d sVec(0,0,0) ; 
      return(sVec) ; 
    }

} // end SmVolume::GetOrientOrigin

/*******************************************************************//**
PURPOSE: convenience access to m_pOrient X Axis

NOTES: 
***********************************************************************/
SmVector3d SmVolume::GetOrientXAxis()  const               
{ 
  if(m_pOrientMap)  
    { return( m_pOrientMap->GetToXAxis() ) ; }
            
  SmVector3d sVec(1,0,0) ; 
  return(sVec) ; 

} // end SmVolume::GetOrientXAxis

/*******************************************************************//**
PURPOSE: convenience access to m_pOrient Y Axis

NOTES: 
***********************************************************************/
SmVector3d SmVolume::GetOrientYAxis()  const               
{ 
  if(m_pOrientMap)  
    { return( m_pOrientMap->GetToYAxis() ) ; }
            
  SmVector3d sVec(0,1,0) ; 
  return(sVec) ; 

} // end SmVolume::GetOrientYAxis

/*******************************************************************//**
PURPOSE: convenience access to m_pOrient Z Axis

NOTES: 
***********************************************************************/
SmVector3d SmVolume::GetOrientZAxis()  const               
{ 
  if(m_pOrientMap)  
    { return( m_pOrientMap->GetToZAxis() ) ; }
         
  SmVector3d sVec(0,0,1) ; 
  return(sVec) ; 

} // end SmVolume::GetOrientZAxis

/*******************************************************************//**
PURPOSE: Set NextMap of compound evaluation Volume(uvw) = NextMap(ThisVolume(uvw))
         replacing current NextMap if there is one.

NOTES: This method is not for building up a sequence of transformations.
       Do that indirectly using the Mirror, Scale, and Transform methods and
       directly with the AddNextMap() method. 
***********************************************************************/
SmStatus SmVolume::SetNextMap
 (SmVolume *pNextMap,        // in : new compounding Map
  ULONG     lOwnerFlag)      // in : 0 = don't copy m_pNextMap when constructed, don't delete m_pNextMap when destructed
                             //      1 = copy m_pNextMap when constructed, delete m_pNextMap when destructed
                             //      2 = don't copy m_pNextMap when constructed, delete m_pNextMap when destructed
                             //      default:[0]                                                                            
{
  // consistent contexts
  if(   pNextMap
     && pNextMap->GetContext() != NULL
     && m_cpContext != NULL
     && m_cpContext != pNextMap->GetContext())
    {
      SER_MSG(SM_ERR, _T("Bad SetNextMap: This and new pNextMap contexts do not match")) ;
    }

  // clean up NULL contexts
  if(pNextMap)
    {
      if(pNextMap->GetContext() == NULL)
        {
          pNextMap->SetContext(m_cpContext) ;
        }
      else if(GetContext() == NULL)
        {
          if(   m_pOrientMap == NULL
             || m_pOrientMap->GetContext() == m_pOrientMap->GetContext())
            { SetContext(pNextMap->GetContext()) ; }
        }
    }

  // clean out the old NextMap
  if(m_pNextMap && m_lNextOwnerFlag >= 1) { delete m_pNextMap ; m_pNextMap = NULL ; } 

  // bring in the new
  m_pNextMap       = pNextMap ;
  m_lNextOwnerFlag = lOwnerFlag ;

  // when asked - copy the input
  if(m_pNextMap && m_lNextOwnerFlag == 1) 
    { 
      m_pNextMap          = NULL ; 
      SmContext *pContext = (SmContext *)pNextMap->GetContext() ;

      if(pContext == NULL)
        {
          SER_MSG(SM_ERR, _T("SmVolume SetNextMap could not copy pNextMap because a cpContext could not be found")) ;
        }
      else
        { 
          SER(pNextMap->Copy(*pContext, m_pNextMap)) ;
        } 
    }

  // all done
  return(SM_SUCCESS) ; 

} // end SmVolume::SetNextMap

/*******************************************************************//**
PURPOSE: Add next level of compounding so that
     NextMap of compound evaluation Volume(uvw) = NextMap(ThisVolume(uvw))

NOTES:  
***********************************************************************/
SmStatus SmVolume::AddNextMap
 (SmVolume *pNextMap,        // in : new compounding Map
  ULONG     lOwnerFlag)      // in : 0 = don't copy m_pNextMap when constructed, don't delete m_pNextMap when destructed
                             //      1 = copy m_pNextMap when constructed, delete m_pNextMap when destructed
                             //      2 = don't copy m_pNextMap when constructed, delete m_pNextMap when destructed
                             //      default:[0]                                                                            
{
  // consistent contexts
  if(   pNextMap->GetContext() != NULL
     && m_cpContext != NULL
     && m_cpContext != pNextMap->GetContext())
    {
      SER_MSG(SM_ERR, _T("Bad AddNextMap: This and new pNextMap contexts do not match - NextMap not added")) ;
    }

  // clean up NULL contexts
  if(pNextMap->GetContext() == NULL)
    {
      pNextMap->SetContext(m_cpContext) ;
    }

  // branch on existence of m_pNextMap
  if(m_pNextMap == NULL)
    {
      // add transformation here!
      m_pNextMap       = pNextMap ;
      m_lNextOwnerFlag = lOwnerFlag ;

      // when asked - copy the input
      if(m_pNextMap && m_lNextOwnerFlag == 1) 
        { 
          m_pNextMap          = NULL ; 
          SmContext *pContext = (SmContext *)pNextMap->GetContext() ;

          if(pContext == NULL)
            {
              SER_MSG(SM_ERR, _T("SmVolume AddNextMap could not copy pNextMap because a cpContext could not be found - NextMap not added")) ;
            }
          else
            { 
              SER(pNextMap->Copy(*pContext, m_pNextMap)) ;
            } 
        }
    }
  else // This volume is already compounded
    {
      // Compound the m_pNextMap Volume
      m_pNextMap->AddNextMap(pNextMap, lOwnerFlag) ;
    }

  // all done
  return(SM_SUCCESS) ; 

} // end SmVolume::AddNextMap

/*******************************************************************//**
PURPOSE: Set OrientMap so that InPoint = OrientMap(ParamPoint)
         replacing current OrientMap if there is one.

NOTES: You may set OrientMap = NULL, A NULL Orient map is
  treated as the identity mapping setting InPoint = ParamPoint. 
***********************************************************************/
SmStatus SmVolume::SetOrientMap
 (SmTransform *pOrientMap,   // in : new orientation Map
  ULONG        lOwnerFlag)   // in : 0 = don't copy m_pOrientMap when constructed, don't delete m_pOrientMap when destructed
                             //      1 = copy m_pOrientMap when constructed, delete m_pOrientMap when destructed
                             //      2 = don't copy m_pOrientMap when constructed, delete m_pOrientMap when destructed
                             //      default:[0]                                                                            
{
  // consistent contexts
  if(   pOrientMap->GetContext() != NULL
     && m_cpContext != NULL
     && m_cpContext != pOrientMap->GetContext())
    {
      SER_MSG(SM_ERR, _T("Bad SetOrientMap: This and new pOrientMap contexts do not match")) ;
    }

  // clean up NULL contexts
  if(pOrientMap->GetContext() == NULL)
    {
      pOrientMap->SetContext(m_cpContext) ;
    }
  else if(GetContext() == NULL)
    {
      if(   m_pNextMap == NULL
         || m_pNextMap->GetContext() == pOrientMap->GetContext())
        { SetContext(pOrientMap->GetContext()) ; }
    }

  // clean out the old OrientMap
  if(m_pOrientMap    && m_lOrientOwnerFlag >= 1) { delete m_pOrientMap ; }    m_pOrientMap = NULL ; 
  if(m_pInvOrientMap)                            { delete m_pInvOrientMap ; } m_pInvOrientMap = NULL ; 
  
  // bring in the new
  m_pOrientMap       = pOrientMap ;
  m_lOrientOwnerFlag = lOwnerFlag ;

  // when asked - copy the input
  if(m_pOrientMap && m_lOrientOwnerFlag == 1) 
    { 
      m_pOrientMap      = NULL ; 
      SmContext *pContext = (SmContext *)pOrientMap->GetContext() ;

      if(pContext == NULL)
        {
          SER_MSG(SM_ERR, _T("SmVolume SetOrientMap could not copy pOrientMap because a cpContext could not be found")) ;
        }
      else
        { 
          SmVolume *pVolume = NULL ;
          SER(pOrientMap->Copy(*pContext, pVolume)) ; 
          m_pOrientMap      = (SmTransform *)pVolume ;
        }
    }

  // build the inverse map
  RefreshInvOrientMap() ;

  // all done
  return(SM_SUCCESS) ; 

} // end SmVolume::SetOrientMap

/*******************************************************************//**
PURPOSE: Build m_pInvOrientMap from current m_pOrientMap

NOTES:  
  m_pOrientMap and m_pInvOrientMap are only used for their EvaluateSimple behaviors,
  as such, their m_pNextMap, m_pOrientMap, and m_pInvOrient pointers are always set to NULL
***********************************************************************/
SmStatus SmVolume::RefreshInvOrientMap()
{
  // clear existing InvOrientMap
  if(m_pInvOrientMap) { delete m_pInvOrientMap ; m_pInvOrientMap = NULL ; }

  // when there is an OrientMap
  if(m_pOrientMap)
    {
      // Copy m_pOrientMap into m_pInvOrientMap
      SmContext * pContext =   GetContext() ? (SmContext *)GetContext()
                             : m_pOrientMap->GetContext() ? (SmContext *)m_pOrientMap->GetContext()
                             : NULL ;
      SmVolume  * pVolume = NULL ; 
      if(pContext == NULL)
        { SER_MSG(SM_ERR, _T("Can't RefreshInvOrientMap because m_cpContext is NULL")) ; }
      m_pOrientMap->Copy(*pContext, pVolume) ;
      m_pInvOrientMap = (SmTransform *)pVolume ;

      // and invert it
      m_pInvOrientMap->InvertSimple() ;
    }

  // ensure m_pOrientMap and m_pInvOrientMap are simple, unoriented maps
  m_pOrientMap->MakeMapSimple() ;
  m_pInvOrientMap->MakeMapSimple() ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::RefreshInvOrientMap

/*******************************************************************//**    
PURPOSE: Make a SmVolume a simple map by ensuring its m_pNextMap, 
         m_pOrientMap, and m_pInvOrientMaps are all NULL. 
                                                                            
NOTES:                                              
***********************************************************************/    
SmStatus SmVolume::MakeMapSimple()  
{
  // ensure m_pNextMap is NULL
  if(m_pNextMap) 
    { 
      if(m_lNextOwnerFlag >= 1) { delete m_pNextMap ; } 
      m_pNextMap = NULL ;  
    }

  // ensure m_pOrientMap is NULL
  if(m_pOrientMap) 
    { 
      if(m_lOrientOwnerFlag >= 1) { delete m_pOrientMap ; } 
      m_pOrientMap = NULL ;   
    }

  // ensure m_pInvOrientMap is NULL
  if(m_pInvOrientMap) 
    { 
      delete m_pInvOrientMap ; 
      m_pInvOrientMap = NULL ;  
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::MakeMapSimple

/*******************************************************************//**    
PURPOSE: Get defined domain size of the Volume mapping - can be unbounded
                                                                            
NOTES: Currently set to the size of the Volume's NaturalParamDomain                                             
***********************************************************************/    
SmStatus SmVolume::ApproximateParamSize
 (SmVector3d & rUVWSize)
 const 
{ 
  // Param Domain Bounding Box - can be unbounded
  SmExtent3d sParamDomain = GetNaturalParamDomain() ;

  // get size of ParamDomain
  if(sParamDomain.IsBounded()) 
       { rUVWSize = sParamDomain.GetSize() ; }
  else { rUVWSize.Set(SM_INFINITE_PARAMETER, 
                      SM_INFINITE_PARAMETER, 
                      SM_INFINITE_PARAMETER) ; 
       }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::ApproximateParamSize

/*******************************************************************//**    
PURPOSE: Get InSpace size of the Volume mapping - can be unbounded
                                                                            
NOTES: Currently set to the size of the Volume's InSpaceBoundingBox
***********************************************************************/    
SmStatus SmVolume::ApproximateInSpaceSize
 (SmVector3d & rInSize)
 const 
{ 
  // Param Domain Bounding Box - can be unbounded
  SmExtent3d sParamDomain = GetNaturalParamDomain() ;

  // get size of InSpace bounding box - can be unbounded
  if(sParamDomain.IsBounded()) 
       { SmExtent3d sInBox ;
         EvaluateBoundingBox(sParamDomain, NULL, NULL, NULL, &sInBox) ;
         rInSize = sInBox.GetSize() ; 
       }
  else { rInSize.Set(SM_INFINITE_PARAMETER, 
                     SM_INFINITE_PARAMETER, 
                     SM_INFINITE_PARAMETER) ; 
       }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::ApproximateInSpaceSize

/*******************************************************************//**    
PURPOSE: Get OutSpace size of the Volume mapping - can be unbounded
                                                                            
NOTES: Currently set to the size of the Volume's InSpaceBoundingBox
***********************************************************************/    
SmStatus SmVolume::ApproximateOutSpaceSize
 (SmVector3d & rOutSize)
 const 
{ 
  // Param Domain Bounding Box - can be unbounded
  SmExtent3d sParamDomain = GetNaturalParamDomain() ;
  
  // get size of OutSpace bounding box - can be unbounded
  if(sParamDomain.IsBounded()) 
       { SmExtent3d sOutBox ;
         EvaluateBoundingBox(sParamDomain, NULL, &sOutBox) ;
         rOutSize = sOutBox.GetSize() ; 
       }
  else { rOutSize.Set(SM_INFINITE_PARAMETER, 
                      SM_INFINITE_PARAMETER, 
                      SM_INFINITE_PARAMETER) ; 
       }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::ApproximateOutSpaceSize

/*******************************************************************//**    
PURPOSE: Get Natural Inspace Domain - can be unbounded
                                                                            
NOTES: The volume InSpaceDomains are just axis aligned ParamSpace SmExtent3d BBoxes 
        oriented by the volume's OrientMap. Squeezing a rotated SmExtent3d BBox into
        an axis oriented SmExtent3d Box just makes it a very large BBox or poorly defined
        when the original BBox is unbounded.  So InSpaceDomains are
        represented only as PseudoBoxes which are generally just a rotated 
        axis aligned SmExtent3d BBox. 

***********************************************************************/    
SmPseudoBox SmVolume::GetNaturalInSpaceDomain()
 const 
{ 
  SmVector3d sO = GetOrientOrigin() ;
  SmVector3d sX = GetOrientXAxis() ;  double sXLength = sX.Length() ;
  SmVector3d sY = GetOrientYAxis() ;  double sYLength = sY.Length() ;
  SmVector3d sZ = GetOrientZAxis() ;  double sZLength = sZ.Length() ;
  SmExtent3d sNaturalParamDomain = GetNaturalParamDomain() ;
  SmExtent1d sXIvl = sNaturalParamDomain.GetUInterval().Scale(1./sXLength).Translate(sO.Dot(sX)) ; 
  SmExtent1d sYIvl = sNaturalParamDomain.GetVInterval().Scale(1./sYLength).Translate(sO.Dot(sY)) ; 
  SmExtent1d sZIvl = sNaturalParamDomain.GetWInterval().Scale(1./sZLength).Translate(sO.Dot(sZ)) ; 

  SmPseudoBox sInSpaceDomain(sX, sY, sZ, sXIvl, sYIvl, sZIvl) ;

  // all done
  return sInSpaceDomain ;
} // end SmVolume::GetNaturalInSpaceDomain

// gwc: alternative idea for ApproximateParamSize
//      /*******************************************************************//**
//      PURPOSE: Compute the approximate InSpace Size by looking at the ISO curves.
//      
//      NOTES:  This tells us how wide the volume is in U, V, W by looking at the 
//          ISO curves through the center point, start and end parameters and
//          taking an average.
//      ***********************************************************************/
//      SmStatus SmVolume::ApproximateParamSize
//        (SmVector3d & rUVWSize)      // out: approximate size of volume's InSpace image
//       const
//      {
//        // init output
//        rUVWSize.Set(0.0,0.0,0.0);
//      
//        // locals
//        SmExtent3d  sParamDomain  = GetNaturalParamDomain();
//        SmCurve    *pIsoCurveVW = NULL ;
//        SmCurve    *pIsoCurveUW = NULL ;
//        SmCurve    *pIsoCurveUV = NULL ;
//      
//        // evaluate volume center point
//        double dParam     = 0.5;
//        SmPoint3d sUVWMid = sParamDomain.Evaluate(dParam,dParam,dParam);
//      
//        // get u, v, w dir isoParamCurves running through center point
//        SER(EvaluateIsoParametricCurve(*GetContext(), SM_VPS_VW, sUVWMid.y, sUVWMid.z, 0.0, &pIsoCurveVW));
//        SER(EvaluateIsoParametricCurve(*GetContext(), SM_VPS_UW, sUVWMid.x, sUVWMid.z, 0.0, &pIsoCurveUW));
//        SER(EvaluateIsoParametricCurve(*GetContext(), SM_VPS_UV, sUVWMid.x, sUVWMid.y, 0.0, &pIsoCurveUV));
//      
//        // make isocurves temporary
//        SmObjDelete sClean1(pIsoCurveVW);
//        SmObjDelete sClean2(pIsoCurveUW);
//        SmObjDelete sClean3(pIsoCurveUV);
//        
//        // get isoParamCurve lengths
//        double dUSize = pIsoCurveVW->ApproximateLength(pIsoCurveVW->GetNaturalInterval(),8);
//        double dVSize = pIsoCurveUW->ApproximateLength(pIsoCurveUW->GetNaturalInterval(),8);
//        double dWSize = pIsoCurveUV->ApproximateLength(pIsoCurveUV->GetNaturalInterval(),8);
//      
//        // set output
//        rUVWSize.x = dUSize ;
//        rUVWSize.y = dVSize ;
//        rUVWSize.z = dWSize ;
//      
//        // all done
//        return SM_SUCCESS;
//      
//      } // end SmVolume::ApproximateParamSize
//      
//      /*******************************************************************//**
//      PURPOSE: Calculate a rough average size of volume first derivatives.
//          This can be used to relate 3d tolerances to 3d distances.
//      
//      NOTES: 
//          This of course just gives a rough average over the whole volume.
//          When working at a particular location on a volume, it is much better
//          to work with actual derivative sizes.  Use this only if you
//          don't have a particular uvw value.
//      
//      METHOD --- simple, just evaluate on a grid and average.
//      ***********************************************************************/
//      SmVector3d SmVolume::ApproxDerivativeLengths
//        (const SmExtent3d & crParamDomain )       // in : desired domain
//      const
//      {
//        // locals
//        ULONG      i, j, k ;
//        const ULONG lNumSamples = 3, LTotalSamples = 27 ;
//        double      dMagU = 0, dMagV = 0, dMagW = 0 ;
//        double      dSamples[lNumSamples] = { 0.15, 0.50, 0.85 };
//        SmPoint3d   sUVW ;
//        SmVector3d  aDerivs[8] ;
//      
//        // for every sample point
//        for(i=0;i<lNumSamples;i++)
//          {
//            for(j=0;j<lNumSamples;j++)
//              {
//                for(k=0;k<lNumSamples;k++)
//                  {
//                    // accumulate size of all derivative values
//                    sUVW = crParamDomain.Evaluate( dSamples[i], dSamples[j], dSamples[k] );
//                    Evaluate( sUVW, 1, TRUE, TRUE, TRUE, aDerivs );
//                    dMagU += aDerivs[4].Length();
//                    dMagV += aDerivs[2].Length();
//                    dMagW += aDerivs[1].Length();
//                  }
//              }
//          } // end iter every sample point
//      
//        // get 1st derivative averages
//        dMagU /= LTotalSamples ;
//        dMagV /= LTotalSamples ;
//        dMagW /= LTotalSamples ;
//      
//        // all done
//        return SmVector3d( dMagU, dMagV, dMagW );
//      
//      } // end SmVolume::ApproxDerivativeLengths

/*******************************************************************//**
PURPOSE: Return TRUE when given ParamPoint lies within the ParamSpace
         NaturalDomain of the Volume mapping.
    
NOTES: when bWithCompounding is TRUE, crParamPoint is projected
       to every NextMap ParamSpace in the linked list of NextMaps
       and FALSE is returned if the crParamPoint maps to any ParamPoint that
       is outside of its NaturalParamDomain.
***********************************************************************/
SmBoolean SmVolume::IsPointInParamDomain
 (const SmPoint3d & crParamPoint,      // in : tgt ParamPoint
  SmBoolean         bWithCompounding)  // in : TRUE = OutSpacePoint is in LastOutSpace
 const                                 //      FALSE= OutSpacePoint is in 1stOutSpace
{
  // pass the call along
  SmBoolean bIsIn = IsPointInParamDomainSimple(crParamPoint) ;

  // when asked - check compounded volumes as well
  if(bIsIn && bWithCompounding && m_pNextMap)
    {
      SmPoint3d sNextInSpacePoint ;

      // map crParamPoint to 1stOutSpace (which equals NextInSpace)
      EvaluatePoint(crParamPoint, sNextInSpacePoint) ;

      // Check the NextInSpacePoint
      bIsIn &= m_pNextMap->IsPointInInSpaceDomain(sNextInSpacePoint, NULL, TRUE) ; 

    } // end need to compound check

  // all done 
  return(bIsIn) ;

} // end SmVolume::IsPointInParamDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given InSpacePoint can be mapped through the 
         InvOrientMap to a ParamPoint which is within the ParamSpace 
         NaturalDomain of the Volume mapping.
    
NOTES: When asked, the inverted param point can be computed and returned
***********************************************************************/
SmBoolean SmVolume::IsPointInInSpaceDomain
 (const SmPoint3d & crInSpacePoint,    // in : target InSpace Point
  SmPoint3d       * pOptParamPoint,    // out: InvOriented ParamPoint when return is TRUE, NULL to ignore, default:[NULL]
  SmBoolean         bWithCompounding)  // NotUsed: in : TRUE = OutSpacePoint is in LastOutSpace
 const                                 //      FALSE= OutSpacePoint is in 1stOutSpace
{
  SM_REF1(bWithCompounding) ;
  // locals
  SmPoint3d sParamPoint ;

  // everything can map through the OrientMap
  if(m_pInvOrientMap)
    {
      m_pInvOrientMap->EvaluatePointSimple(crInSpacePoint, sParamPoint) ;
    }
  else
    {
      sParamPoint = crInSpacePoint ;
    }

  // pass the check along
  SmBoolean bIsIn = IsPointInParamDomainSimple(sParamPoint) ;

  // set output
  if(pOptParamPoint) { *pOptParamPoint = sParamPoint ; }

  // all done 
  return(bIsIn) ;

} // end SmVolume::IsPointInInSpaceDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given OutSpacePoint can be inverted back to 
         a ParamPoint which is within the NaturalDomain of the Volume mapping.
    
NOTES: When asked, the inverted param point can be computed and returned
***********************************************************************/
SmBoolean SmVolume::IsPointInOutSpaceDomain
 (const SmPoint3d & crOutSpacePoint,   // in : OutSpace Point to test
  SmPoint3d       * pOptParamPoint,    // out: inverted ParamPoint when return is TRUE, NULL to ignore, default:[NULL]
  SmBoolean         bWithCompounding)  // in : TRUE = OutSpacePoint is in LastOutSpace
 const                                 //      FALSE= OutSpacePoint is in 1stOutSpace
{
  // init output
  if(pOptParamPoint) { pOptParamPoint->SetUninitialized() ; }
  
  // locals  
  SmPoint3d sMidSpacePoint, sProjSpacePoint ;

  // with compounding - recurse from last to next Volume mappings
  if(bWithCompounding && m_pNextMap)
    {
      SmPoint3d sNextParamPoint ;

      // Classify the point for NextMap 
      SmBoolean bThisPointIn = m_pNextMap->IsPointInOutSpaceDomain(crOutSpacePoint,
                                                                  &sNextParamPoint,
                                                                   bWithCompounding) ;

      // exit - when OutSpacePoint maps to a point not within the NextMap's ParamDomain space
      if(bThisPointIn == FALSE)
        {
          return(FALSE) ;
        }

      // map sNextParamPoint to NextInSpace through m_pOrientMap
      if(m_pOrientMap) { m_pOrientMap->EvaluatePointSimple(sNextParamPoint, sMidSpacePoint) ; }
      else             { sMidSpacePoint = sNextParamPoint ; }
    }
  else
    {
      sMidSpacePoint = crOutSpacePoint ;
    }

  // the given crOutSpacePoint has been mapped to this volume's 1stOutSpace in sMidSpacePoint

  // Map sMidSpacePoint to ProjSpace
  if(m_pInvOrientMap) { m_pInvOrientMap->EvaluatePointSimple(sMidSpacePoint, sProjSpacePoint) ; }
  else                { sProjSpacePoint = sMidSpacePoint ; }

  // pass the call along
  SmBoolean bIsIn = IsPointInProjDomainSimple(sProjSpacePoint, pOptParamPoint) ;

  // all done 
  return(bIsIn) ;

} // end SmVolume::IsPointInOutSpaceDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given ParamLineSeg given by start and end
         ParamPoints lies within the ParamSpace NaturalDomain of 
         the Volume mapping without crossing any internal discontinuities.
    
NOTES: when bWithCompounding is TRUE, crParamLineSegment is projected
       to every NextMap ParamSpace in the linked list of NextMaps
       and FALSE is returned if the crParamLineSegment maps to any 
       ParamLineSegment that is outside of its NaturalParamDomain or
       crosses any internal discontinuities.
***********************************************************************/
SmBoolean SmVolume::IsLineInParamDomain
 (const SmPoint3d & crStartParamPoint, // in : tgt ParamLine Start Point
  const SmPoint3d & crEndParamPoint,   // in : tgt ParamLine Start Point
  SmBoolean         bWithCompounding)  // in : TRUE = OutSpacePoint is in LastOutSpace
 const                                 //      FALSE= OutSpacePoint is in 1stOutSpace
{
  // pass the call along
  SmBoolean bIsIn = IsLineInParamDomainSimple(crStartParamPoint, crEndParamPoint) ;

  // when asked - check compounded volumes as well
  if(bIsIn && bWithCompounding && m_pNextMap)
    {
      SmPoint3d sStartInSpacePoint ;
      SmPoint3d sEndInSpacePoint ;

      // map Param points to InSpace
      if(m_pOrientMap) { OrientPoint(crStartParamPoint, sStartInSpacePoint) ;
                         OrientPoint(crEndParamPoint, sEndInSpacePoint) ; 
                       }
      else             { sStartInSpacePoint = crStartParamPoint ;
                         sEndInSpacePoint   = crEndParamPoint ;
                       }

      // build curve in OutSpace = NextInSpace
      SmVolume      sSimpleVolume(*this, TRUE) ; // TRUE = copy simple map only - no compounded volumes
      SmLine        sInSpaceLine(sStartInSpacePoint, sEndInSpacePoint, 3, GetContext()) ; 
      SmCrvInVolume sNextInSpaceCurve(sInSpaceLine, FALSE, sSimpleVolume, 0, 0, GetContext()) ; // 0 = don't copy Curve and Volume, 0 = don't delete Curve and Volume

      // Check the NextInSpaceCurve
      bIsIn &= m_pNextMap->IsCurveInInSpaceDomain(sNextInSpaceCurve, TRUE) ; 

    } // end need to compound check

  // all done 
  return(bIsIn) ;

} // end SmVolume::IsLineInParamDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given InSpaceLineSegment can be mapped through the 
         InvOrientMap to a ParamLineSegment which is within the ParamSpace 
         NaturalDomain of the Volume mapping without crossing any
         internal discontinuities.
    
NOTES: When asked, the inverted param lineSegment can be computed and returned
***********************************************************************/
SmBoolean SmVolume::IsLineInInSpaceDomain
 (const SmPoint3d & crStartInSpacePoint, // in : target InSpace Line Start Point
  const SmPoint3d & crEndInSpacePoint,   // in : target InSpace Line End Point
  SmPoint3d       * pOptStartParamPoint, // out: InvOriented StartParamPoint when return is TRUE, NULL to ignore, default:[NULL]
  SmPoint3d       * pOptEndParamPoint,   // out: InvOriented EndParamPoint when return is TRUE, NULL to ignore, default:[NULL]
  SmBoolean         bWithCompounding)    // in : TRUE = OutSpaceLine is in LastOutSpace
 const                                   //      FALSE= OutSpaceLine is in 1stOutSpace
{
  // locals
  SmPoint3d sStartParamPoint ;
  SmPoint3d sEndParamPoint ;

  // Lines map to lines through the Orient and InvOrient Maps 
  if(m_pInvOrientMap)
    {
      m_pInvOrientMap->EvaluatePointSimple(crStartInSpacePoint, sStartParamPoint) ;
      m_pInvOrientMap->EvaluatePointSimple(crEndInSpacePoint, sEndParamPoint) ;
    }
  else
    {
      sStartParamPoint = crStartInSpacePoint ;
      sEndParamPoint   = crEndInSpacePoint ;
    } 
  
  // set optional output  
  if(pOptStartParamPoint) { *pOptStartParamPoint = crStartInSpacePoint ; ; }
  if(pOptEndParamPoint  ) { *pOptEndParamPoint   = crEndInSpacePoint ; ; }              

  // pass the check along
  SmBoolean bIsIn = IsLineInParamDomain(sStartParamPoint, sEndParamPoint, bWithCompounding) ;

  // set output
  if(pOptStartParamPoint) { *pOptStartParamPoint = sStartParamPoint ; }
  if(pOptEndParamPoint)   { *pOptEndParamPoint   = sEndParamPoint ; }

  // all done 
  return(bIsIn) ;

} // end SmVolume::IsLineInInSpaceDomain

//  /*******************************************************************//**
//  ToBe implemented: base implementations on CrvInVolumes tested with IsCurveInInSpaceDomain
//  
//  PURPOSE: Return TRUE when given OutSpaceLineSegment can be inverted back to 
//           a ParamLineSegment which is within the NaturalDomain of 
//           the Volume mapping without crossing any internal discontinutities.
//      
//  NOTES: When asked, the inverted param LineSegment can be computed and returned
//  ***********************************************************************/
//  SmBoolean SmVolume::IsLineInOutSpaceDomain
//   (const SmPoint3d & crStartOutSpacePoint, // in : OutSpace Line Start Point to test
//    const SmPoint3d & crEndOutSpacePoint,   // in : OutSpace Line End Point to test
//    SmPoint3d       * pOptStartParamPoint,  // out: inverted Start ParamPoint when return is TRUE, NULL to ignore, default:[NULL]
//    SmPoint3d       * pOptEndParamPoint,    // out: inverted End ParamPoint when return is TRUE, NULL to ignore, default:[NULL]
//    SmBoolean         bWithCompounding)     // in : TRUE = OutSpaceLine is in LastOutSpace
//   const                                    //      FALSE= OutSpaceLine is in 1stOutSpace
//  {
//    // init output
//    if(pOptStartParamPoint) { pOptStartParamPoint->SetUninitialized() ; }
//    if(pOptEndParamPoint)   { pOptEndParamPoint->SetUninitialized() ; }
//  
//    // locals
//    SmBoolean bIsIn = TRUE ;
//  
//    // make input line
//  
//    // build ParamSpace curve - bWithCompounding TRUE : need inverse of Volume mapping including all compound objects
//    //                                           FALSE: need inverse of Volume without any compounded objects
//    // idea 1: make an approximate curves and test with IsCurveInParamDomain
//    // idea 2: sample line, project each point back to param space, test each pair of points with isLineInParamDomain
//  
//  
//    // all done 
//    return(bIsIn) ;
//  
//  } // end SmVolume::IsLineInOutSpaceDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given SmExtent3d 
         is within the NaturalDomain of the Volume mapping.
    
NOTES: 1. when bWithCompounding is TRUE, crParamBox is projected
          to every NextMap ParamSpace in the linked list of NextMaps
          and FALSE is returned if the crParamPoint maps to any ParamPoint that
          is outside of its NaturalParamDomain.
***********************************************************************/
SmBoolean SmVolume::IsBoxInParamDomain
 (const SmExtent3d & crParamBox,       // in : ParamBox to test
  SmBoolean          bWithCompounding) // in : TRUE = OutSpacePoint is in LastOutSpace
 const                                 //      FALSE= OutSpacePoint is in 1stOutSpace
{
  // locals
  ULONG ii, i1 ;
  SmPoint3d sData[24] ;
  SmTArray<SmPoint3d> sEdgeEndPoints(24,sData,24) ;

  // some volumes have internal discontinuities - don't check vertices check edges
  
  // get Box edges // every pair of points [iEven,iEven+1] marks one edge
  crParamBox.GetEdges(sEdgeEndPoints) ;

  // for every Edge
  for(ii=0,i1=1;i1<sEdgeEndPoints.GetSize();ii+=2,i1+=2)
    {
      // test that ParamLine is completely within ParamSpace domain
      SmBoolean bThisEdgeInDomain = IsLineInParamDomain(sEdgeEndPoints[ii], sEdgeEndPoints[i1], bWithCompounding) ;

      // when any ParamBoxEdge line is not completely within ParamSpace domain - return FALSE
      if(bThisEdgeInDomain == FALSE)
        { return(FALSE) ; }
    }

  // all done
  return(TRUE) ;

} // end SmVolume::IsBoxInParamDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given SmExtent3d can be inverted back to 
         a ParamBox which is within the NaturalDomain of the Volume mapping.
    
NOTES: 
***********************************************************************/
SmBoolean SmVolume::IsBoxInInSpaceDomain
 (const SmExtent3d & crInSpaceBox,     // in : InSpace Box to check 
  SmBoolean          bWithCompounding) // in : TRUE = OutSpacePoint is in LastOutSpace
 const                                 //      FALSE= OutSpacePoint is in 1stOutSpace
{
  // locals
  ULONG ii, i1 ;
  SmPoint3d sData[24] ;
  SmTArray<SmPoint3d> sEdgeEndPoints(24,sData,24) ;

  // some volumes have internal discontinuities - don't check vertices check edges
  
  // get Box edges // every pair of points [iEven,iEven+1] marks one edge
  crInSpaceBox.GetEdges(sEdgeEndPoints) ;

  // for every Edge
  for(ii=0,i1=1;i1<sEdgeEndPoints.GetSize();ii+=2,i1+=2)
    {
      // test that InSpaceLine maps to a ParamSpaceLine that is completely within the ParamSpace domain
      SmBoolean bThisEdgeInDomain = IsLineInInSpaceDomain(sEdgeEndPoints[ii], sEdgeEndPoints[i1], NULL,  NULL, bWithCompounding) ;

      // when any InSpaceBoxEdge Line does not map completely within the ParamSpace domain - return FALSE
      if(bThisEdgeInDomain == FALSE)
        { return(FALSE) ; }
    }

  // all done
  return(TRUE) ;

  // old idea = check box corners since the box is convex and many of the spaces are as well
  //  SmTArray<SmPoint3d> sCornerPoints(8,sData,8) ;
  //  
  //  // get Box corners
  //  crInSpaceBox.GetCorners(sCornerPoints) ;
  //  
  //  // for every corner
  //  for(ii=0;ii<sCornerPoints.GetSize();ii++)
  //    {
  //      // test the point
  //      SmBoolean bThisPointInDomain = IsPointInInSpaceDomain(sCornerPoints[ii], NULL, bWithCompounding) ;
  //  
  //      // when any corner point is bad - return FALSE
  //      if(bThisPointInDomain == FALSE)
  //        { return(FALSE) ; }
  //    }
  //  
  //  // all done
  //  return(TRUE) ;

} // end SmVolume::IsBoxInInSpaceDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given SmExtent3d can be inverted back to 
         a ParamBox which is within the NaturalDomain of the Volume mapping.
    
NOTES: GWC TODO: rewrite this method
                 The map from Outspace to InSpace is not affine so
                 checking the original convex box's corners in its
                 domain space is not a complete check.

                 idea 1: GetEdges(), For Every Edge() - sample, for every sample - test point
                         but this is only approximate for the edges and doesn't
                         deal with the surfaces
                         so, Sample every face, for every sample - test point
                           still approximate - but better.
                 idea 2: For every surface - test surface with IsSurfaceInOutSpaceDomain
                          (IsSurfaceInOutSpaceDomain has to be written) 
***********************************************************************/
SmBoolean SmVolume::IsBoxInOutSpaceDomain
 (const SmExtent3d & crOutSpaceBox,    // in : OutSpace Box to test 
  SmBoolean          bWithCompounding) // in : TRUE = OutSpacePoint is in LastOutSpace
 const                                 //      FALSE= OutSpacePoint is in 1stOutSpace
{
  // locals
  ULONG ii ;
  SmPoint3d sData[8] ;
  SmTArray<SmPoint3d> sCornerPoints(8,sData,8) ;

  // get Box corners
  crOutSpaceBox.GetCorners(sCornerPoints) ;

  // for every corner
  for(ii=0;ii<sCornerPoints.GetSize();ii++)
    {
      // test the point
      SmBoolean bThisPointInDomain = IsPointInOutSpaceDomain(sCornerPoints[ii],NULL,bWithCompounding) ;

      // when any corner point is bad - return FALSE
      if(bThisPointInDomain == FALSE)
        { return(FALSE) ; }
    }

  // all done
  return(TRUE) ;

} // end SmVolume::IsBoxInOutSpaceDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given SmExtent3d 
         is within the NaturalDomain of the Volume mapping.
    
NOTES: 1. when bWithCompounding is TRUE, crParamBox is projected
          to every NextMap ParamSpace in the linked list of NextMaps
          and FALSE is returned if the crParamPoint maps to any ParamPoint that
          is outside of its NaturalParamDomain.
***********************************************************************/
SmBoolean SmVolume::IsPseudoBoxInParamDomain
 (const SmPseudoBox & crParamPseudoBox,  // in : ParamBox to test
  SmBoolean           bWithCompounding)  // in : TRUE = OutSpacePoint is in LastOutSpace
 const                                   //      FALSE= OutSpacePoint is in 1stOutSpace
{
  // locals
  ULONG ii, i1 ;
  SmPoint3d sData[24] ;
  SmTArray<SmPoint3d> sEdgeEndPoints(24,sData,24) ;

  // some volumes have internal discontinuities - don't check vertices check edges
  
  // get Box edges // every pair of points [iEven,iEven+1] marks one edge
  crParamPseudoBox.GetEdges(sEdgeEndPoints) ;

  // for every Edge
  for(ii=0,i1=1;i1<sEdgeEndPoints.GetSize();ii+=2,i1+=2)
    {
      // test that ParamLine is completely within ParamSpace domain
      SmBoolean bThisEdgeInDomain = IsLineInParamDomain(sEdgeEndPoints[ii], sEdgeEndPoints[i1], bWithCompounding) ;

      // when any InSpaceBoxEdge Line does not map completely within the ParamSpace domain - return FALSE
      if(bThisEdgeInDomain == FALSE)
        { return(FALSE) ; }
    }

  // all done
  return(TRUE) ;



  //  // old idea = check box corners since the box is convex and many of the spaces are as well
  //  // locals
  //  ULONG ii ;
  //  SmPoint3d sData[8] ;
  //  SmTArray<SmPoint3d> sCornerPoints(8,sData,8) ;
  //  
  //  // get Box corners
  //  crParamPseudoBox.GetCorners(sCornerPoints) ;
  //  
  //  // for every corner
  //  for(ii=0;ii<sCornerPoints.GetSize();ii++)
  //    {
  //      // test the point
  //      SmBoolean bThisPointInDomain = IsPointInParamDomain(sCornerPoints[ii], bWithCompounding) ;
  //  
  //      // when any corner point is bad - return FALSE
  //      if(bThisPointInDomain == FALSE)
  //        { return(FALSE) ; }
  //    }
  //  
  //  // all done
  //  return(TRUE) ;

} // end SmVolume::IsPseudoBoxInParamDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given SmExtent3d can be inverted back to 
         a ParamBox which is within the NaturalDomain of the Volume mapping.
    
NOTES: 
***********************************************************************/
SmBoolean SmVolume::IsPseudoBoxInInSpaceDomain
 (const SmPseudoBox & crInSpacePseudoBox,  // in : InSpace Box to check 
  SmBoolean           bWithCompounding)    // in : TRUE = OutSpacePoint is in LastOutSpace
 const                                     //      FALSE= OutSpacePoint is in 1stOutSpace
{
  // locals
  ULONG ii, i1 ;
  SmPoint3d sData[24] ;
  SmTArray<SmPoint3d> sEdgeEndPoints(24,sData,24) ;

  // some volumes have internal discontinuities - don't check vertices check edges
  
  // get Box edges // every pair of points [iEven,iEven+1] marks one edge
  crInSpacePseudoBox.GetEdges(sEdgeEndPoints) ;

  // for every Edge
  for(ii=0,i1=1;i1<sEdgeEndPoints.GetSize();ii+=2,i1+=2)
    {
      // test that InSpaceLine maps to a ParamSpaceLine that is completely within the ParamSpace domain
      SmBoolean bThisEdgeInDomain = IsLineInInSpaceDomain(sEdgeEndPoints[ii], sEdgeEndPoints[i1], NULL,  NULL, bWithCompounding) ;

      // when any InSpaceBoxEdge Line does not map completely within the ParamSpace domain - return FALSE
      if(bThisEdgeInDomain == FALSE)
        { return(FALSE) ; }
    }

  // all done
  return(TRUE) ;

  // old idea = check box corners since the box is convex and many of the spaces are as well
  //  // locals
  //  ULONG ii ;
  //  SmPoint3d sData[8] ;
  //  SmTArray<SmPoint3d> sCornerPoints(8,sData,8) ;
  //  
  //  // get Box corners
  //  crInSpacePseudoBox.GetCorners(sCornerPoints) ;
  //  
  //  // for every corner
  //  for(ii=0;ii<sCornerPoints.GetSize();ii++)
  //    {
  //      // test the point
  //      SmBoolean bThisPointInDomain = IsPointInInSpaceDomain(sCornerPoints[ii], NULL, bWithCompounding) ;
  //  
  //      // when any corner point is bad - return FALSE
  //      if(bThisPointInDomain == FALSE)
  //        { return(FALSE) ; }
  //    }
  //  
  //  // all done
  //  return(TRUE) ;

} // end SmVolume::IsPseudoBoxInInSpaceDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given SmExtent3d can be inverted back to 
         a ParamBox which is within the NaturalDomain of the Volume mapping.
    
NOTES: GWC TODO: rewrite this method
                 The map from Outspace to InSpace is not affine so
                 checking the original convex box's corners in its
                 domain space is not a complete check.

                 idea 1: GetEdges(), For Every Edge() - sample, for every sample - test point
                         but this is only approximate for the edges and doesn't
                         deal with the surfaces
                         so, Sample every face, for every sample - test point
                           still approximate - but better.
                 idea 2: For every surface - test surface with IsSurfaceInOutSpaceDomain
                          (IsSurfaceInOutSpaceDomain has to be written) 
***********************************************************************/
SmBoolean SmVolume::IsPseudoBoxInOutSpaceDomain
 (const SmPseudoBox & crOutSpacePseudoBox,  // in : OutSpace Box to test 
  SmBoolean           bWithCompounding)     // in : TRUE = OutSpacePoint is in LastOutSpace
 const                                      //      FALSE= OutSpacePoint is in 1stOutSpace
{
  // locals
  ULONG ii ;
  SmPoint3d sData[8] ;
  SmTArray<SmPoint3d> sCornerPoints(8,sData,8) ;

  // get Box corners
  crOutSpacePseudoBox.GetCorners(sCornerPoints) ;

  // for every corner
  for(ii=0;ii<sCornerPoints.GetSize();ii++)
    {
      // test the point
      SmBoolean bThisPointInDomain = IsPointInOutSpaceDomain(sCornerPoints[ii],NULL,bWithCompounding) ;

      // when any corner point is bad - return FALSE
      if(bThisPointInDomain == FALSE)
        { return(FALSE) ; }
    }

  // all done
  return(TRUE) ;

} // end SmVolume::IsPseudoBoxInOutSpaceDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given ParamCurve lies within the ParamSpace
         NaturalDomain of the Volume mapping.
    
NOTES: when bWithCompounding is TRUE, crParamCurve is projected
       to every NextMap ParamSpace in the linked list of NextMaps
       and FALSE is returned if the crParamCurve fails to lie within
       any of the NaturalDomains of any of those ParamSpaces.
***********************************************************************/
SmBoolean SmVolume::IsCurveInParamDomain
 (const SmCurve & crParamCurve,      // in : tgt ParamCurve
  SmBoolean       bWithCompounding)  // in : TRUE = Check this and all compounded ParamSpaceDomains
 const                               //      FALSE= Check only this ParamSpaceDomain
{
  // pass the call along
  SmBoolean bIsIn = IsCurveInParamDomainSimple(crParamCurve) ;

  // when asked - check compounded volumes as well
  if(bIsIn && bWithCompounding && m_pNextMap)
    {
      SmContext sContext ;
      SmVolume *pThisSimpleMap = NULL ;

      // Copy this Volume omitting any compounding volumes
      Copy(sContext, pThisSimpleMap, TRUE) ;
      SmObjDelete sClean(pThisSimpleMap) ; 

      // project Curve to 1st OutSpace = Next InSpace 
      SmCrvInVolume sThisOutSpaceCurve((SmCurve &)crParamCurve, TRUE, *pThisSimpleMap, 0, 0, &sContext) ; 

      // Check the NextInSpaceCurve
      bIsIn &= m_pNextMap->IsCurveInInSpaceDomain(sThisOutSpaceCurve, TRUE) ; 

    } // end need to compound check

  // all done 
  return(bIsIn) ;

} // end SmVolume::IsCurveInParamDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given InSpaceCurve can be mapped through the 
         InvOrientMap to a ParamCurve which is within the ParamSpace 
         NaturalDomain of the Volume mapping.
    
NOTES:   when bWithCompounding is TRUE, crParamCurve is projected
       to every NextMap ParamSpace in the linked list of NextMaps
       and FALSE is returned if the crParamCurve fails to lie within
       any of the NaturalDomains of any of those ParamSpaces.
***********************************************************************/
SmBoolean SmVolume::IsCurveInInSpaceDomain
 (const SmCurve & crInSpaceCurve,    // in : target InSpace Curve
  SmBoolean       bWithCompounding)  // in : TRUE = Check this and all compounded ParamSpaceDomains
 const                               //      FALSE= Check only this ParamSpaceDomain
{
  // locals
  SmContext sContext ;
  SmBoolean bIsIn = FALSE ; 

  // when there is an InvOrientMap
  if(m_pInvOrientMap)
    {
      // project InSpaceCurve to Param space
      SmCrvInVolume sThisParamSpaceCurve((SmCurve &)crInSpaceCurve, TRUE, *GetInvOrientMap(), 0, 0, &sContext) ;
      bIsIn = IsCurveInParamDomain(sThisParamSpaceCurve, bWithCompounding) ; 
    }
  else // no InvOrientMap
    {
      // InSpaceCurve == ParamSpaceCurve
      bIsIn = IsCurveInParamDomain(crInSpaceCurve, bWithCompounding) ; 
    }

  // all done 
  return(bIsIn) ;

} // end SmVolume::IsCurveInInSpaceDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given ParamSurface lies within the ParamSpace
         NaturalDomain of the Volume mapping.
    
NOTES: when bWithCompounding is TRUE, crParamSurface is projected
       to every NextMap ParamSpace in the linked list of NextMaps
       and FALSE is returned if the crParamSurface fails to lie within
       any of the NaturalDomains of any of those ParamSpaces.
***********************************************************************/
SmBoolean SmVolume::IsSurfaceInParamDomain
 (const SmSurface & crParamSurface,   // in : tgt ParamSurface
  SmBoolean         bWithCompounding) // in : TRUE = Check this and all compounded ParamSpaceDomains
 const                                //      FALSE= Check only this ParamSpaceDomain
{
  // pass the call along
  SmBoolean bIsIn = IsSurfaceInParamDomainSimple(crParamSurface) ;

  // when asked - check compounded volumes as well
  if(bIsIn && bWithCompounding && m_pNextMap)
    {
      SmContext sContext ;
      SmVolume *pThisSimpleMap = NULL ;

      // Copy this Volume omitting any compounding volumes
      Copy(sContext, pThisSimpleMap, TRUE) ;
      SmObjDelete sClean(pThisSimpleMap) ; 

      // project Surface to 1st OutSpace = Next InSpace 
      SmSrfInVolume sThisOutSpaceSurface((SmSurface &)crParamSurface, TRUE, *pThisSimpleMap, 0, 0, &sContext) ; 

      // Check the NextInSpaceSurface
      bIsIn &= m_pNextMap->IsSurfaceInInSpaceDomain(sThisOutSpaceSurface, TRUE) ; 

    } // end need to compound check

  // all done 
  return(bIsIn) ;

} // end SmVolume::IsSurfaceInParamDomain

/*******************************************************************//**
PURPOSE: Return TRUE when given InSpaceSurface can be mapped through the 
         InvOrientMap to a ParamSurface which is within the ParamSpace 
         NaturalDomain of the Volume mapping.
    
NOTES:   when bWithCompounding is TRUE, crParamSurface is projected
       to every NextMap ParamSpace in the linked list of NextMaps
       and FALSE is returned if the crParamSurface fails to lie within
       any of the NaturalDomains of any of those ParamSpaces.
***********************************************************************/
SmBoolean SmVolume::IsSurfaceInInSpaceDomain
 (const SmSurface & crInSpaceSurface, // in : target InSpace Surface
  SmBoolean       bWithCompounding)   // in : TRUE = Check this and all compounded ParamSpaceDomains
 const                                //      FALSE= Check only this ParamSpaceDomain
{
  // locals
  SmContext sContext ;
  SmBoolean bIsIn = FALSE ; 

  // when there is an InvOrientMap
  if(m_pInvOrientMap)
    {
      // project InSpaceSurface to Param space
      SmSrfInVolume sThisParamSpaceSurface((SmSurface &)crInSpaceSurface, FALSE, *GetInvOrientMap(), 0, 0, &sContext) ;
      bIsIn = IsSurfaceInParamDomain(sThisParamSpaceSurface, bWithCompounding) ; 
    }
  else // no InvOrientMap
    {
      // InSpaceSurface == ParamSpaceSurface
      bIsIn = IsSurfaceInParamDomain(crInSpaceSurface, bWithCompounding) ; 
    }

  // all done 
  return(bIsIn) ;

} // end SmVolume::IsSurfaceInInSpaceDomain

//      / *******************************************************************//**
//      PURPOSE: Given a bounded InSpace Plane description, report
//               what rectilinear portion of that plane lies completely within
//               the NaturalParamSpace domain.
//          
//      NOTES: Currently, this function only looks at the NaturalParamDomain
//        limitiations of the current Volume and does not address the question
//        compounding.  In the future, compounding should be adressed and
//        the way I'd do it is to represent the bounds of every ParamDomain
//        as a shape (most will be represented by oriented planes for half-spaces,
//        some might be represented by oriented cylinders.  The compounded
//        NaturalParamDomain can be computed as the boolean of a set of
//        half-space constraints.  The half-spaces can be represented by
//        simple shapes projected through the various compounded volumes and
//        that combination of shape married to projection can be represnted
//        by the class SmSrfInVolume.  So the representation of the NatrualParamDomain
//        for a compounded volume will be a set of SmSrfInVolume objects that
//        each represent one half space constraint on the param domain.  The 
//        actual ParamDomain would be defined as the intersection of all the half-spaces.
//      
//        That's a lot of work - and is certainly not justified at this time.
//      
//      RETURNS: SM_ERR when no part of the given plane is found within the NaturalParamDomain.
//      
//      *********************************************************************** /
//      SmStatus SmVolume::FindParamExtentForInSpacePlane // rtn: SM_ERR for NULL line, else SM_SUCCESS
//       (const SmPoint3d      & crInSpacePoint,          // in : LinePoint          of Line(s) = LinePoint + u * LineVecU + v * LineVecV              
//        const SmVector3d     & crInSpaceVecU,           // in : scaled LineVectorU of Line(s) = LinePoint + u * LineVecU + v * LineVecV              
//        const SmVector3d     & crInSpaceVecV,           // in : scaled LineVectorV of Line(s) = LinePoint + u * LineVecU + v * LineVecV              
//        const SmExtent2d     & crCurrentUV,             // in : current limits on UV extent
//        SmTArray<SmExtent2d> & rTrimUVs)                // out: UVExtent of plane that maps legally within the NaturalParamDomains
//       const                                            
//      { 
//        return( FindParamIntervalForInSpacePlaneSimple
//                 (crInSpacePoint, crInSpaceVectorU, crInSpaceVectorV,
//                  crCurrentIvl, rTrimIvls) ) ;
//      
//      } // end SmVolume::FindParamExtentForInSpacePlane

/*******************************************************************//**
PURPOSE: Given a ParamSpace domain Box, compute the axis alligned 
    (normalBox), and/or non-axis alligned (pseudoBox), 
    in InSpace and/or OutSpace.
    
NOTES: 

RETURNS: SM_ERR when any point within crParamBox does not
   map to the volume's Natural ParamDomain as tested by
   calls to IsPointInParamDomain().
***********************************************************************/
SmStatus SmVolume::EvaluateBoundingBox
  (const SmExtent3d & crParamBox,          // in : ParamSpace BBox to project to In and Out Spaces 
   SmPseudoBox      * pOptParamPseudoBox,  // in : optional ParamSpace PseudoBox used to get output PseudoBox orientations
                                           //      NULL = align axis to projected corners
                                           //      default:[NULL]
   SmExtent3d       * pOutSpaceBox,        // out: OutSpace Axis aligned box,     NULL to ignore, default:[NULL]
   SmPseudoBox      * pOutSpacePseudoBox,  // out: OutSpace Non-axis aligned box, NULL to ignore, default:[NULL]
   SmExtent3d       * pInSpaceBox,         // out: InSpace Axis aligned box,      NULL to ignore, default:[NULL]
   SmPseudoBox      * pInSpacePseudoBox,   // out: InSpace Non-axis aligned box,  NULL to ignore, default:[NULL]
   SmBoolean          bWithCompounding)    // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
  const
{
  // no work - no output requests
  if(   pInSpaceBox        == NULL
     && pInSpacePseudoBox  == NULL
     && pOutSpaceBox       == NULL
     && pOutSpacePseudoBox == NULL) 
    { return SM_SUCCESS ; }

  // check input - ParamBox must be maintained within Natural ParamDomain
  if(FALSE == IsBoxInParamDomain(crParamBox) ) 
    { SER_MSG(SM_ERR, _T("SmVolume::EvaluateBoundingBox() given bad ParamBox not within NaturalParamDomain")) ; }

  // when input ParamBox is the same as either InSpace or OutSpace output boxes - copy ParamBox
  SmExtent3d         sBox ;
  const SmExtent3d * pBox ;
  if(   (pOutSpaceBox && (&crParamBox == pOutSpaceBox)) 
     || (pInSpaceBox  && (&crParamBox == pInSpaceBox))) { sBox = crParamBox ;
                                                          pBox = &sBox ;
                                                        }
  else                                                  { pBox = &crParamBox ; 
                                                        }

  // when input pOptParamPseudoBox is the same as either InSpace or OutSpace output PseudoBoxes - copy OptParamPseudoBox
  SmBoolean bCopyParamPseudoBox =    pOptParamPseudoBox 
                                 && (   pOptParamPseudoBox == pOutSpacePseudoBox 
                                     || pOptParamPseudoBox == pInSpacePseudoBox) ;
  SmPseudoBox  sPseudoBox ;
  SmPseudoBox *pPseudoBox ;
  if(bCopyParamPseudoBox) { sPseudoBox = *pOptParamPseudoBox ;
                            pPseudoBox = &sPseudoBox ;
                          }
  else                    { pPseudoBox = pOptParamPseudoBox ; 
                          }
                         
  // indirection: from here on - pBox       == ParamSpaceBox
  //                             pPseudoBox == ParamSpacePseudoBox

  // when asked for InSpace Boxes
  if(   pInSpaceBox       != NULL
     || pInSpacePseudoBox != NULL)
    {
      // when given an orient map
      if(m_pOrientMap)
        {
          // project ParamDomain boxes to InSpace boxes through OrientMap()
          // that's the same as using the m_pOrientMap to project ParamDomain boxes to m_pOrient->ProjSpace
          SER(m_pOrientMap->EvaluateBoundingBoxSimple(*pBox, pPseudoBox, pInSpaceBox, pInSpacePseudoBox)) ;
        }
      else // without an OrientMap, ParamSpace == InSpace
        {
          // return InSpaceBox = DomainSpaceBox.            
          if(pInSpaceBox) { *pInSpaceBox = *pBox ; }

          // when building an InSpace PseudoBox
          if(pInSpacePseudoBox) 
            {  
              // with a pOptParamPseudoBox
              if(pOptParamPseudoBox) 
                {
                  // let pInSpacePseudoBox = pOptParamPseudoBox (for orientation)
                  *pInSpacePseudoBox = *pPseudoBox ;

                  // Unbounded paramDomains - set PseudoBox intervals to unbounded
                  if(pBox->IsBounded() == FALSE)
                    {
                      long alAxisMap[3] ;
                      if(!pInSpacePseudoBox->IsAxisAligned(alAxisMap))  // out: alAxisMap[0] = index+1 of basis parallel to X, neg = in negative direction
                                                                        //      alAxisMap[1] = index+1 of basis parallel to Y, neg = in negative direction
                                                                        //      alAxisMap[2] = index+1 of basis parallel to Z, neg = in negative direction
                        {
                          // return unbounded BBoxes
                          pInSpacePseudoBox->SetUnbounded() ;

                        } // end unaligned transformation check
                      else // target orientation is axis aligned
                        {
                          // Set m_vMin and m_vMax directly from the BBox intervals.
                          //  the reason this extra branch exists is to allow partially
                          //   unbound BBoxes to be built for partially unbound PseudoBoxes.
                          //  That's helpful to the SmVolume class
                          ULONG ii ;
                          for(ii=0;ii<3;ii++)
                            {
                              ULONG      lx      = smos_Labs(alAxisMap[ii]) - 1 ;
                              double     dLength = pInSpacePseudoBox->GetBasis(lx).Length() ;
                              SmExtent1d sIvl(pBox->GetMin(lx), pBox->GetMax(lx)) ; // don't scale here because of infinite values
                              sIvl.Scale(dLength) ;                                 // scale must come before Translate

                              pInSpacePseudoBox->SetInterval(lx, sIvl) ; 

                            } // end iter every basis 
                        }  // end target orientation  is axis aligned check

                    } // end ParamDomain is not bounded branch
                  else // bounded ParamDomains - map corners to set PseudoBox interval ranges
                    {
                      // add ParamDomain Corners to PseudoBox
                      pInSpacePseudoBox->AddPoint3d(pBox->Evaluate(0,0,0)) ;
                      pInSpacePseudoBox->AddPoint3d(pBox->Evaluate(0,0,1)) ;
                      pInSpacePseudoBox->AddPoint3d(pBox->Evaluate(0,1,0)) ;
                      pInSpacePseudoBox->AddPoint3d(pBox->Evaluate(0,1,1)) ;

                      pInSpacePseudoBox->AddPoint3d(pBox->Evaluate(1,0,0)) ;
                      pInSpacePseudoBox->AddPoint3d(pBox->Evaluate(1,0,1)) ;
                      pInSpacePseudoBox->AddPoint3d(pBox->Evaluate(1,1,0)) ;
                      pInSpacePseudoBox->AddPoint3d(pBox->Evaluate(1,1,1)) ;
                    }
                } // end with a pOptParamPseudoBox branch
              else // without a pOptParamPseudoBox
                {
                  // let pInSpacePseudoBox = crParamBox
                  pInSpacePseudoBox->InitBasis() ;
                  pInSpacePseudoBox->SetIntervals(pBox->GetUInterval(),
                                                  pBox->GetVInterval(),
                                                  pBox->GetWInterval()) ;
                }
            } // end build pInSpacePseudoBox check
        } // end without an OrientMap check
    } // end build InSpace boxes check

  // when asked for OutSpace boxes with compounding
  if(   pOutSpaceBox       != NULL
     || pOutSpacePseudoBox != NULL)
    { 
      SmPseudoBox sThisOutSpacePseudoBox ;

      // project ParamDomain Boxes to ProjectSpace Boxes
      if(m_pOrientMap)
        {
          SmExtent3d   sProjBox ;
          SmPseudoBox  sProjPseudoBox ;
          SmPseudoBox *pProjPseudoBox = &sProjPseudoBox ;

          // map from Param to Proj space
          SER(EvaluateBoundingBoxSimple(*pBox, pPseudoBox, &sProjBox, pProjPseudoBox)) ;

          // map from Proj to Out space
          SER(m_pOrientMap->EvaluateBoundingBoxSimple(sProjBox, pProjPseudoBox, pOutSpaceBox, &sThisOutSpacePseudoBox)) ;
        }
      else // no OrientMap branch
        {
          // projSpace == OutSpace, map from Param to Out space
          SER(EvaluateBoundingBoxSimple(*pBox, pPseudoBox, pOutSpaceBox, &sThisOutSpacePseudoBox)) ;
        }

      // arrive here when pOutSpaceBox, pOutSpacePseudoBox are computed for the 1st OutSpace

      // when asked - Map 1st OutSpace boxes to last OutSpace
      if(   (m_pNextMap && bWithCompounding)
         && (   pOutSpaceBox
             || pOutSpacePseudoBox))
        {
          // project the OutSpace boxes through the compounding maps
          SER(m_pNextMap->MapPseudoBox(sThisOutSpacePseudoBox, pOutSpaceBox, &sThisOutSpacePseudoBox, bWithCompounding)) ;
        }    

      // set output
      if(pOutSpacePseudoBox) { *pOutSpacePseudoBox = sThisOutSpacePseudoBox ; }

    } // need for OutSpace boxes check

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::EvaluateBoundingBox

/*******************************************************************//**
PURPOSE: Given a ParamSpace PseudoBox, compute the axis alligned 
    (normalBox), and/or non-axis alligned (pseudoBox), 
    in InSpace and/or OutSpace.
    
NOTES: 

RETURNS: SM_ERR when any point within crParamBox does not
   map to the volume's Natural ParamDomain as tested by
   calls to IsPointInParamDomain().
***********************************************************************/
SmStatus SmVolume::EvaluatePseudoBox
  (const SmPseudoBox & crParamPseudoBox,    // in : ParamSpace Pseudo box to proj to OutSpace and/or InSpace
   SmExtent3d        * pOutSpaceBox,        // out: OutSpace Axis aligned box,     NULL to ignore, default:[NULL]
   SmPseudoBox       * pOutSpacePseudoBox,  // out: OutSpace Non-axis aligned box, NULL to ignore, default:[NULL]
   SmExtent3d        * pInSpaceBox,         // out: InSpace Axis aligned box,      NULL to ignore, default:[NULL]
   SmPseudoBox       * pInSpacePseudoBox,   // out: InSpace Non-axis aligned box,  NULL to ignore, default:[NULL]
   SmBoolean           bWithCompounding)    // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
  const
{
  // no work - no output requests
  if(   pInSpaceBox        == NULL
     && pInSpacePseudoBox  == NULL
     && pOutSpaceBox       == NULL
     && pOutSpacePseudoBox == NULL) 
    { return SM_SUCCESS ; }

  // check input - ParamBox must be maintained within Natural ParamDomain
  if(FALSE == IsPseudoBoxInParamDomain(crParamPseudoBox) ) 
    { SER_MSG(SM_ERR, _T("SmVolume::EvaluatePseudoBox() given bad Param PseudoBox not within NaturalParamDomain")) ; }

  // when input ParamBox is the same as either InSpace or OutSpace output boxes - copy ParamBox
  SmPseudoBox         sPseudoBox ;
  const SmPseudoBox * pPseudoBox ;
  if(   (pOutSpacePseudoBox && (&crParamPseudoBox == pOutSpacePseudoBox)) 
     || (pInSpacePseudoBox  && (&crParamPseudoBox == pInSpacePseudoBox ))) { sPseudoBox = crParamPseudoBox ;
                                                                             pPseudoBox = &sPseudoBox ;
                                                                           }
  else                                                                     { pPseudoBox = &crParamPseudoBox ; 
                                                                           }

  // indirection: from here on - pPseudoBox == ParamSpacePseudoBox

  // when asked build InSpace Boxes - they are either output or used to build OutSpace boxes
  if(   pInSpaceBox       != NULL 
     || pInSpacePseudoBox != NULL)
    {
      // locals
      SmPseudoBox    sInSpacePseudoBox ;  // initializes with basis:[x y z]
      double         dMin, dMax ; 
      SmVector3d     sParamSpaceX, sParamSpaceY, sParamSpaceZ ;
      SmVector3d     sInSpaceX, sInSpaceY, sInSpaceZ, sInSpaceVec ;
      SmExtent1d     sXIvl, sYIvl, sZIvl ;  // start as ParamIvls - get changed into InSpace Ivls
      SmBoundaryType eBndryType ;  // oneof SM_BT_BOUNDED,     
                                   //       SM_BT_UNBOUNDED_MIN
                                   //       SM_BT_UNBOUNDED_MAX
                                   //       SM_BT_UNBOUNDED
                                     
      pPseudoBox->GetBasis(sParamSpaceX, sParamSpaceY, sParamSpaceZ) ;
      pPseudoBox->GetIntervals(sXIvl, sYIvl, sZIvl) ;

      // when given an orient map - Transform Param bases to InSpace bases
      if(m_pOrientMap)
        {
          // orient sInSpacePseudoBox axes
          m_pOrientMap->TransformVector(m_pOrientMap->GetToXAxis(), sInSpaceX) ;
          m_pOrientMap->TransformVector(m_pOrientMap->GetToYAxis(), sInSpaceY) ;
          m_pOrientMap->TransformVector(m_pOrientMap->GetToZAxis(), sInSpaceZ) ;
          sInSpacePseudoBox.SetBasis(sInSpaceX, sInSpaceY, sInSpaceZ) ;

        }

      // set sInSpacePseudoBox basis vectors - orient ParamSpace basis vectors to OutSpace
      sInSpacePseudoBox.GetBasis(sInSpaceX, sInSpaceY, sInSpaceZ) ;

      // build OutSpace X Ivl
      { sXIvl.IsBounded(&eBndryType) ;

        if(   eBndryType == SM_BT_UNBOUNDED 
           || eBndryType == SM_BT_UNBOUNDED_MIN) { dMin = -SM_INFINITE_PARAMETER ; }
        else                                     { m_pOrientMap->TransformVector(sXIvl.GetMin() * sParamSpaceX, sInSpaceVec) ;
                                                   dMin = sInSpaceX.Dot(sInSpaceVec) ; 
                                                 }
        if(   eBndryType == SM_BT_UNBOUNDED 
           || eBndryType == SM_BT_UNBOUNDED_MAX) { dMax = SM_INFINITE_PARAMETER ; }
        else                                     { m_pOrientMap->TransformVector(sXIvl.GetMax() * sParamSpaceX, sInSpaceVec) ;
                                                   dMax = sInSpaceX.Dot(sInSpaceVec) ; 
                                                 }
        sXIvl.SetMinMax(dMin, dMax) ; 
      }

      // build OutSpace Y Ivl
      { sYIvl.IsBounded(&eBndryType) ;

        if(   eBndryType == SM_BT_UNBOUNDED 
           || eBndryType == SM_BT_UNBOUNDED_MIN) { dMin = -SM_INFINITE_PARAMETER ; }
        else                                     { m_pOrientMap->TransformVector(sYIvl.GetMin() * sParamSpaceY, sInSpaceVec) ;
                                                   dMin = sInSpaceY.Dot(sInSpaceVec) ; 
                                                 }
        if(   eBndryType == SM_BT_UNBOUNDED 
           || eBndryType == SM_BT_UNBOUNDED_MAX) { dMax = SM_INFINITE_PARAMETER ; }
        else                                     { m_pOrientMap->TransformVector(sYIvl.GetMax() * sParamSpaceY, sInSpaceVec) ;
                                                   dMax = sInSpaceY.Dot(sInSpaceVec) ; 
                                                 }
        sYIvl.SetMinMax(dMin, dMax) ; 
      }

      // build OutSpace Z Ivl
      { sZIvl.IsBounded(&eBndryType) ;

        if(   eBndryType == SM_BT_UNBOUNDED 
           || eBndryType == SM_BT_UNBOUNDED_MIN) { dMin = -SM_INFINITE_PARAMETER ; }
        else                                     { m_pOrientMap->TransformVector(sZIvl.GetMin() * sParamSpaceZ, sInSpaceVec) ;
                                                   dMin = sInSpaceZ.Dot(sInSpaceVec) ; 
                                                 }
        if(   eBndryType == SM_BT_UNBOUNDED 
           || eBndryType == SM_BT_UNBOUNDED_MAX) { dMax = SM_INFINITE_PARAMETER ; }
        else                                     { m_pOrientMap->TransformVector(sZIvl.GetMax() * sParamSpaceZ, sInSpaceVec) ;
                                                   dMax = sInSpaceZ.Dot(sInSpaceVec) ; 
                                                 }
        sZIvl.SetMinMax(dMin, dMax) ; 
      }

      // set sInSpacePseudoBox intervals
      sInSpacePseudoBox.SetIntervals(sXIvl, sYIvl, sZIvl) ;

      // set outputs
      if(pInSpacePseudoBox) { *pInSpacePseudoBox = sInSpacePseudoBox ; }
      if(pInSpaceBox)       { pInSpaceBox->Circumscribe(sInSpacePseudoBox) ; } 

    } // end need InSpace boxes check

  // when asked for OutSpace boxes with/without compounding
  if(   pOutSpaceBox       != NULL
     || pOutSpacePseudoBox != NULL)
    { 
      SmPseudoBox sOutSpacePseudoBox ; // initializes to basis:[x y z]

      // project ParamDomain Boxes to ProjectSpace Boxes
      if(m_pOrientMap)
        {
          SmExtent3d   sProjBox ;
          SmPseudoBox  sProjPseudoBox ;
          SmPseudoBox *pProjPseudoBox = &sProjPseudoBox ;

          // map from Param to Proj space
          SER(EvaluatePseudoBoxSimple(*pPseudoBox, &sProjBox, pProjPseudoBox)) ;

          // map from Proj to Out space
          SER(m_pOrientMap->EvaluateBoundingBoxSimple(sProjBox, NULL, pOutSpaceBox)) ;
          SER(m_pOrientMap->EvaluatePseudoBoxSimple(*pProjPseudoBox, NULL, &sOutSpacePseudoBox)) ;
        }
      else // no OrientMap branch
        {
          // projSpace == OutSpace, map from Param to Out space
          SER(EvaluatePseudoBoxSimple(*pPseudoBox, pOutSpaceBox, &sOutSpacePseudoBox)) ;
        }

      // arrive here when pOutSpaceBox, pOutSpacePseudoBox are computed for the 1st OutSpace

      // when asked - Map 1st OutSpace boxes to last OutSpace
      if(   (m_pNextMap && bWithCompounding)
         && (   pOutSpaceBox
             || pOutSpacePseudoBox))
        {
          // project the OutSpace boxes through the compounding maps
          SER(m_pNextMap->MapPseudoBox(sOutSpacePseudoBox, pOutSpaceBox, &sOutSpacePseudoBox, bWithCompounding)) ;
        }    

      // set output
      if(pOutSpacePseudoBox) { *pOutSpacePseudoBox = sOutSpacePseudoBox ; }

    } // need for OutSpace boxes check

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::EvaluatePseudoBox

/*******************************************************************//**
PURPOSE: Given an InSpace axis aligned domain, compute the OutSpace axis alligned 
    (normalBox), and/or non-axis alligned (pseudoBox), and/or 
    polar (PolarBox) bounding boxes.  
    
NOTES: One or more of the outputs must be non-NULL.
    Bounding boxes are built for whole volumes not subdomains.
    The cache mechanism subdivides a large volume into a set of
    bezier patches and then calls this function to bound each sub-region.
***********************************************************************/
SmStatus SmVolume::MapPseudoBox
  (const SmPseudoBox & crInSpacePseudoBox,      // in : Bounding box in InSpace
   SmExtent3d        * pOutSpaceBox,            // out: Axis aligned OutSpace box
   SmPseudoBox       * pOutSpacePseudoBox,      // out: Non-axis aligned OutSpace box
   SmBoolean           bWithCompounding,        // in : TRUE = project to last OutSpace, FALSE = project to first OutSpace
                                                //      default:[TRUE]    
   SmBoolean           bTrimToLegalParamSpace)  // in : TRUE  = Trim ParamSpaceBox to LegalParamDomain  
  const                                         //      FALSE = Report boxes that need trimming as errors. 
                                                //      default:[FALSE]
{
  // when asked check input - InSpacePseudoBox must map within Natural ParamDomain
  if(bTrimToLegalParamSpace == FALSE)
    {
      // see if the projection of crInSpacePseudoBox will be in the legal ParamSpace
      SmBoolean IsInDomain = IsPseudoBoxInInSpaceDomain(crInSpacePseudoBox) ;

      // treat boxes that don't map inside of NaturalParamDomain as errors
      if(IsInDomain == FALSE ) 
        { 
          SER_MSG(SM_ERR, _T("SmVolume::EvaluateBoundingBox() given bad InSpacePseudoBox that doesn't map inside the NaturalParamDomain")) ; 
        }

    } // end asked to run input check

  // when PseudoBox input is the same as output - copy crInSpacePseudoBox input
  SmPseudoBox sPseudoBox, *pPseudoBox ;
  if(    pOutSpacePseudoBox
     && &crInSpacePseudoBox == pOutSpacePseudoBox) { sPseudoBox = crInSpacePseudoBox ;
                                                     pPseudoBox = &sPseudoBox ;
                                                   }
  else                                             { pPseudoBox = &(SmPseudoBox &)crInSpacePseudoBox ;
                                                   }

  // when asked for compound maps - force this function to compute a OutSpacePseudoBox to pass along
  SmPseudoBox sThisOutSpacePseudoBox ;
  SmPseudoBox *pThisOutSpacePseudoBox = (m_pNextMap != NULL && bWithCompounding == TRUE && pOutSpacePseudoBox == NULL)
                                        ? &sThisOutSpacePseudoBox
                                        : pOutSpacePseudoBox ;

  // indirection: from here on out - pPseudoBox == crInSpacePseudoBox
  //                                 pThisOutSpacePseudoBox == pOutSpacePseudoBox

  // when given an orient map
  if(m_pInvOrientMap)
    {
      // intermediate ParamSpace Boxes - make sParamPBox = NULL when pThisOutSpacePseudoBox == NULL
      SmExtent3d   sParamBox ;
      SmPseudoBox  sParamPseudoBox ;
      SmPseudoBox *pParamPseudoBox = pThisOutSpacePseudoBox ? &sParamPseudoBox : NULL ;

      // intermediate ProjSpace Boxes  - make sProjPseudoBox = NULL when pThisOutSpacePseudoBox == NULL
      SmExtent3d   sProjBox ;
      SmPseudoBox  sProjPseudoBox ;
      //SmPseudoBox *pProjPseudoBox = pThisOutSpacePseudoBox ? &sProjPseudoBox : NULL ;

      // map inSpace box to ParamBoxes (same as mapping from pInvOrientMap->DomainSpace to pInvOrientMap->InSpace)
      m_pInvOrientMap->EvaluatePseudoBoxSimple(*pPseudoBox, &sParamBox, pParamPseudoBox) ;

      // when asked - trim ParamBoxes to legal space
      if(bTrimToLegalParamSpace)
        {
          TrimParamBoundingBoxSimple(sParamBox, sParamBox) ;
        } 

      // map ParamBoxes to ProjSpace
      EvaluateBoundingBoxSimple(sParamBox, pParamPseudoBox, &sProjBox, &sProjPseudoBox) ;

      // map ProjBox to OutSpace
      m_pOrientMap->EvaluateBoundingBoxSimple(sProjBox, &sProjPseudoBox, pOutSpaceBox, pThisOutSpacePseudoBox) ;
    }
  else // no orient map, DomainSpace == ParamSpace and ProjSpace == OutSpace
    {
      
      // when asked - trim ParamBoxes to legal space
      if(bTrimToLegalParamSpace)
        {
          SmExtent3d   sParamBox ;
          TrimParamBoundingBoxSimple(crInSpacePseudoBox, sParamBox) ;

          // map trimmed InBoxes to OutBoxes directly
          EvaluateBoundingBoxSimple(sParamBox, NULL, pOutSpaceBox, pThisOutSpacePseudoBox) ;
        }
      else
        { 
          // map InBoxes to OutBoxes directly
          EvaluatePseudoBoxSimple(crInSpacePseudoBox, pOutSpaceBox, pThisOutSpacePseudoBox) ;
        }
    }

  // arrive here when pOutSpaceBox and pThisOutSpacePseudoBox are computed for the 1st OutSpace
      
  // when asked for compound maps - concatenate mapping sequence from first to last OutSpace
  if(m_pNextMap != NULL && bWithCompounding == TRUE)
    {
      // pass the call along to next mapping
      m_pNextMap->MapPseudoBox(*pThisOutSpacePseudoBox, pOutSpaceBox, pThisOutSpacePseudoBox, bWithCompounding, bTrimToLegalParamSpace) ;

    } // end compound map existence check

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::MapPseudoBox

/*******************************************************************//**
PURPOSE: Create InSpace and/or OutSpace IsoCurves from a ParamSpace IsoParamLine

NOTES --- InIsoCurve  = InvOrient(IsoParamLine)        
          OutIsoCurve = Orient(EvaluateSimple(IsoParamLine))
***********************************************************************/
SmStatus SmVolume::EvaluateIsoParametricCurve
  (const SmContext   & crContext,               // in : context for created objects
   SmVolumeParamsType  eConstantParams,         // in : Defines constant Volume parameter directions 
                                                //      SM_VPS_UV = create varying w isoParameter curve
                                                //      SM_VPS_UW = create varying v isoParameter curve
                                                //      SM_VPS_VW = create varying u isoParameter curve
   double              dIsoParameter1,          // in : Defines parametric value at which to extract the curve.  
                                                //      When eConstantParams==SM_VPS_UV, Param = constant u
                                                //                            SM_VPS_UW, Param = constant u
                                                //                            SM_VPS_VW, Param = constant v
   double              dIsoParameter2,          // in : Defines parametric value at which to extract the curve.  
                                                //      When eConstantParams==SM_VPS_UV, Param = constant v
                                                //                            SM_VPS_UW, Param = constant w
                                                //                            SM_VPS_VW, Param = constant w
   double              d3DTolerance,            // in : d3DTolerance 
   SmCurve          ** ppNewOutSpaceIsoCurve,   // out: OutSpace 3D IsoParameterCurve, NULL to ignore, default:[NULL]
   SmCurve          ** ppNewInSpaceIsoCurve,    // out: InSpace 3D IsoParameterCurve, NULL to ignore, default:[NULL]
   const SmExtent3d  * pOptParamTrimBox,        // in : opt ParamSpace trim box for the IsoCurve, NULL=natural BoundingBox
                                                //      NULL=natural BoundingBox or default subVolume for infinite ParamDomains
                                                //      default:[NULL]
   SmBoolean          bWithCompounding)         // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // no work - no outputs
  if(   ppNewInSpaceIsoCurve  == NULL 
     && ppNewOutSpaceIsoCurve == NULL)
    { return SM_SUCCESS ; }

  // locals
  SmExtent1d        sFreeIvl ;
  SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
  const SmExtent3d *pTgtDomain   = pOptParamTrimBox ? pOptParamTrimBox : &sParamDomain ;
  SmExtent1d        sFixedIvl1, sFixedIvl2 ; 

  // InSpace IsoCurve
  if(ppNewInSpaceIsoCurve)
    {
      // init output
      if( *ppNewInSpaceIsoCurve )  { delete *ppNewInSpaceIsoCurve ; *ppNewInSpaceIsoCurve = NULL ; }

      // with orient map
      if(m_pOrientMap)
        {
          // Build InSpace IsoCurve
          SER(m_pOrientMap->EvaluateIsoParametricCurveSimple( crContext, eConstantParams, 
                                                              dIsoParameter1, dIsoParameter2, 
                                                              d3DTolerance, 
                                                             *ppNewInSpaceIsoCurve, 
                                                              pOptParamTrimBox)) ;

        }
      else // without orient map: Orient == identity build isoLine
        {
          // locals 
          SmPoint3d  sPuvw ;
          SmVector3d sVuvw ;

           // watch out for confused user I/O
           eConstantParams = SM_INSPACE_TO_PARAMS_TYPE(eConstantParams) ;

          // switch on varying parameter
          switch(eConstantParams)
            {
              // varying u
              case SM_VPS_VW : sPuvw.Set(0.0, dIsoParameter1, dIsoParameter2) ;
                               sVuvw.Set(1.0, 0.0, 0.0) ;
                               sFreeIvl   = pTgtDomain->GetUInterval() ;
                               sFixedIvl1 = pTgtDomain->GetVInterval() ;                    
                               sFixedIvl2 = pTgtDomain->GetWInterval() ;                    
                               break ;

              // varying v
              case SM_VPS_UW : sPuvw.Set(dIsoParameter1, 0.0, dIsoParameter2) ;
                               sVuvw.Set(0.0, 1.0, 0.0) ;                      
                               sFreeIvl   = pTgtDomain->GetVInterval() ;
                               sFixedIvl1 = pTgtDomain->GetUInterval() ;                    
                               sFixedIvl2 = pTgtDomain->GetWInterval() ; 
                               break ;

              // varying w
              case SM_VPS_UV : sPuvw.Set(dIsoParameter1, dIsoParameter2, 0.0) ;
                               sVuvw.Set(0.0, 0.0, 1.0) ;                      
                               sFreeIvl   = pTgtDomain->GetWInterval() ;
                               sFixedIvl1 = pTgtDomain->GetUInterval() ;                    
                               sFixedIvl2 = pTgtDomain->GetVInterval() ;                      
                               break ;

              default : SER(SM_ERR) ;
            }

          // when asked - skip culling, else cull lines outside of TrimBox
          if(   pOptParamTrimBox == NULL
             || (   sFixedIvl1.ContainsValue(dIsoParameter1, SM_EFF_ZERO)
                 && sFixedIvl2.ContainsValue(dIsoParameter2, SM_EFF_ZERO)))
            {
              // create InSpace line
              *ppNewInSpaceIsoCurve = new (crContext) SmLine(sPuvw, sVuvw) ;

              // trim to given bounded box
              (*ppNewInSpaceIsoCurve)->AdjustSTEPInterval(sFreeIvl) ;

            } // end not trimmed or within trim box check
        } // end without orient map branch
    } // end need InSpaceIsoCurve check

  // OutSpace IsoCurve
  if(ppNewOutSpaceIsoCurve)
    {
      // init output
      if( *ppNewOutSpaceIsoCurve )  { delete *ppNewOutSpaceIsoCurve ; *ppNewOutSpaceIsoCurve = NULL ; }

      // Create ProjSpace IsoCurve by mapping ParamSpace IsoLine to ProjSpace IsoCurve
      SER(EvaluateIsoParametricCurveSimple( crContext, eConstantParams, 
                                            dIsoParameter1, dIsoParameter2, 
                                            d3DTolerance, 
                                           *ppNewOutSpaceIsoCurve, 
                                            pOptParamTrimBox)) ;

      // Map ProjSpace IsoCurve to OutSpace when needed 
      if(m_pOrientMap && *ppNewOutSpaceIsoCurve != NULL)
        {
          // when ProjSpace curve is a Line
          SmPoint3d sProjSpacePV[2] ;
          if( (*ppNewOutSpaceIsoCurve)->IsLine(4, SM_EFF_ZERO, sProjSpacePV[0], sProjSpacePV[1]) )
            {
              // locals
              SmExtent1d sIvl = (*ppNewOutSpaceIsoCurve)->GetNaturalInterval() ;
              SmPoint3d sProjSpaceStart, sProjSpaceEnd ;
              SmPoint3d sOutSpaceStart, sOutSpaceEnd ;

              // Get ProjSpace start and end points
              (*ppNewOutSpaceIsoCurve)->EvaluatePoint(sIvl.GetMin(), sProjSpaceStart) ;
              (*ppNewOutSpaceIsoCurve)->EvaluatePoint(sIvl.GetMax(), sProjSpaceEnd) ;

              // map ProjectPoints to 1st OutSpace
              m_pOrientMap->TransformPoint(sProjSpaceStart, sOutSpaceStart) ;
              m_pOrientMap->TransformPoint(sProjSpaceEnd,   sOutSpaceEnd) ;

              // free the ProjSpace line
              delete *ppNewOutSpaceIsoCurve ; *ppNewOutSpaceIsoCurve = NULL ;

              // build the 1stOutSpace line
              SmLine::CreateLineSegment(crContext, 3, sOutSpaceStart, sOutSpaceEnd, (SmLine *&)*ppNewOutSpaceIsoCurve, &sIvl) ; 
            }
          else // ProjSpace IsoCurve is not a Line
            {
              // build 1stOutSpace IsoCurve by mapping ProjSpace IsoCurve through m_pOrientMap
              *ppNewOutSpaceIsoCurve = new (crContext) SmCrvInVolume 
                                         (**ppNewOutSpaceIsoCurve,    // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
                                          FALSE,                      // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
                                          (SmVolume &)*m_pOrientMap,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                           2,                         // in : 2 = copy volume and save curve orig
                                           3,                         // in : 3 = delete both curve and volume when destructed
                                          &crContext) ;               // in : req for stack objs, opt for heap objs
           }
        } // end need to orient ProjSpace IsoCurve check

      // when needed - map 1stOutSpace IsoCurve to LastOutSpace IsoCurve through m_pNextMap
      if(m_pNextMap && bWithCompounding && *ppNewOutSpaceIsoCurve != NULL)
        {
          // compound the IsoCurve by mapping 1stOutSpace IsoCurve through m_pNextMap
          *ppNewOutSpaceIsoCurve = new (crContext) SmCrvInVolume
                                     (**ppNewOutSpaceIsoCurve,  // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
                                      FALSE,                    // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
                                      (SmVolume &)*m_pNextMap,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                       2,                       // in : 2 = copy volume and save curve orig
                                       3,                       // in : 3 = delete both curve and volume when destructed
                                      &crContext) ;             // in : req for stack objs, opt for heap objs
        }
    } // end need OutSpaceIsoCurve check

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::EvaluateIsoParametricCurve

/*******************************************************************//**
PURPOSE: Map InSpace IsoParamLine to outSpace IsoCurve

NOTES --- OutIsoCurve = Orient(EvaluateSimple(InvOrient(IsoInSpaceLine))
***********************************************************************/
SmStatus SmVolume::MapIsoParametricCurve
  (const SmContext   & crContext,               // in : context for created objects
   SmVolumeParamsType  eConstantParams,         // in : Defines constant Volume parameter directions 
                                                //      SM_VPS_XY_IN = create varying Zin isoParameter curve
                                                //      SM_VPS_XZ_IN = create varying Yin isoParameter curve
                                                //      SM_VPS_YZ_IN = create varying Xin isoParameter curve
   double              dIsoParameter1,          // in : Defines parametric value at which to extract the curve.  
                                                //      When eConstantParams==SM_VPS_XY_IN, Param = constant Xin
                                                //                            SM_VPS_XZ_IN, Param = constant Xin
                                                //                            SM_VPS_YZ_IN, Param = constant Yin
   double              dIsoParameter2,          // in : Defines parametric value at which to extract the curve.  
                                                //      When eConstantParams==SM_VPS_XY_IN, Param = constant Yin
                                                //                            SM_VPS_XZ_IN, Param = constant Zin
                                                //                            SM_VPS_YZ_IN, Param = constant Zin
   double              d3DTolerance,            // in : d3DTolerance 
   SmCurve          *& rpNewOutSpaceIsoCurve,   // out: OutSpace 3D IsoParameterCurve
   const SmPseudoBox * pOptInSpaceTrimBox,      // in : opt InSpace trim box for the IsoCurve
                                                //      NULL=natural BoundingBox or default subVolume for infinite InSpaceDomains
                                                //      default:[NULL]
   const SmExtent3d  * pOptParamTrimBox,        //      opt ParamSpace trim box for the IsoCurve - used when m_pOrient == NULL
                                                //      NULL=natural BoundingBox or default subVolume for infinite ParamDomains
                                                //      default:[NULL]
   SmBoolean           bWithCompounding)        // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // init output
  if( rpNewOutSpaceIsoCurve )  { delete rpNewOutSpaceIsoCurve ; rpNewOutSpaceIsoCurve = NULL ; }

  // locals
  SmExtent1d sFreeIvl ;

  // pick InSpace bounding box - making sure that it maps to a legal ParamDomain.
  SmPseudoBox sTgtDomain = pOptInSpaceTrimBox ? *pOptInSpaceTrimBox : GetNaturalInSpaceDomain() ;

  // GWC - this approach seems to be too conservative - try trimming InSpaceLines to legal ParamDomains later
  //     SmStatus   eStat      = TrimInSpaceBoundingBox(sTgtDomain, sTgtDomain) ;
  //      
  //      // if sTgtDomain has been reduced to the NULL set - quit - no curve to render
  //      if(eStat != SM_SUCCESS)
  //        { // no curve to return
  //          rpNewOutSpaceIsoCurve = NULL ;
  //          return(SM_SUCCESS) ; 
  //        }

  // with orient map
  if(m_pOrientMap)
    {
      // locals 
      SmPoint3d  sPin ;
      SmVector3d sVin ; 

      // watch out for confused user I/O
      eConstantParams = SM_PARAMS_TO_INSPACE_TYPE(eConstantParams) ;

      // switch on varying parameter - build InSpace IsoLine
      switch(eConstantParams)
        {
          // varying u
          case SM_VPS_YZ_IN : sPin.Set(0.0, dIsoParameter1, dIsoParameter2) ;
                              sVin.Set(1.0, 0.0, 0.0) ;
                              sFreeIvl   = sTgtDomain.GetUInterval(sPin) ;
                              break ;

          // varying v
          case SM_VPS_XZ_IN : sPin.Set(dIsoParameter1, 0.0, dIsoParameter2) ;
                              sVin.Set(0.0, 1.0, 0.0) ;                      
                              sFreeIvl   = sTgtDomain.GetVInterval(sPin) ;
                              break ;

          // varying w
          case SM_VPS_XY_IN : sPin.Set(dIsoParameter1, dIsoParameter2, 0.0) ;
                              sVin.Set(0.0, 0.0, 1.0) ;                      
                              sFreeIvl   = sTgtDomain.GetWInterval(sPin) ;
                              break ;

          default : SER(SM_ERR) ;
        }

      // when Some portion of IsoParamLine is inside the InSpaceDomain BBox
      if(sFreeIvl.IsDegenerate(SM_XSECT_TOL_3D))
        {
          SER(SmCurve::CreateDegenerateCurve(*m_cpContext, 3,
                                             sPin,
                                             rpNewOutSpaceIsoCurve));
        }
      else if(sFreeIvl.IsInit() == FALSE)
        {
          // create InSpace infinite line
          rpNewOutSpaceIsoCurve = new (crContext) SmLine(sPin, sVin) ;

          // trim the line
          rpNewOutSpaceIsoCurve->AdjustSTEPInterval(sFreeIvl) ; 

          // Find the portion of the InSpace Line interval that maps to the NaturalParamDomain 
          SmTArray<SmExtent1d> sTrimIvls ;
          SmExtent1d sCurrentIvl = rpNewOutSpaceIsoCurve->GetNaturalInterval() ;
          FindParamIntervalForInSpaceLine(sPin, sVin, sCurrentIvl, sTrimIvls) ;
          if(sTrimIvls.GetSize() == 0 || sTrimIvls[0].IsDegenerate())
            {
              // no curve to return
              if(rpNewOutSpaceIsoCurve) { delete rpNewOutSpaceIsoCurve ; rpNewOutSpaceIsoCurve = NULL ; }
              return(SM_SUCCESS) ;
            }
          else
            {
              rpNewOutSpaceIsoCurve->AdjustSTEPInterval(sTrimIvls[0]) ;
            }

          // Map InSpace IsoCurve to LastOutSpace 
          rpNewOutSpaceIsoCurve = new (crContext) SmCrvInVolume
                                    (*rpNewOutSpaceIsoCurve,  // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
                                     FALSE,                   // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
                                     (SmVolume &)*this,       // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                      2,                      // in : 2 = copy volume and save curve orig
                                      3,                      // in : 3 = delete both curve and volume when destructed
                                     &crContext) ;            // in : req for stack objs, opt for heap objs
      
          // when asked to skip compounding
          if(bWithCompounding == FALSE)
            {
              // remove compounding from rpNewOutSpaceIsoCurve's volume
              ((SmVolume *)((SmCrvInVolume *)rpNewOutSpaceIsoCurve)->GetVolume())->SetNextMap(NULL) ;
            }
        } // end no trimming or in trimming box check

    } // end with OrientMap branch
  else // without OrientMap branch
    {
      // pick ParamSpace bounding box - making sure that it maps to a legal ParamDomain.
      SmExtent3d sDomain = pOptParamTrimBox ? *pOptParamTrimBox : GetDiscontinuityFreeParamDomain().ApproximateUnbounded() ;


      // Build SimpleEvaluate IsoCurve
      SER(EvaluateIsoParametricCurveSimple(crContext, SM_INSPACE_TO_PARAMS_TYPE(eConstantParams), 
                                           dIsoParameter1, dIsoParameter2, 
                                           d3DTolerance, 
                                           rpNewOutSpaceIsoCurve, 
                                           &sDomain )) ;
      
      // compound map
      if(m_pNextMap && bWithCompounding && rpNewOutSpaceIsoCurve != NULL)
        {
          // Compound OutSpace IsoCurve to final OutSpace 
          rpNewOutSpaceIsoCurve = new (crContext) SmCrvInVolume
                                    (*rpNewOutSpaceIsoCurve,  // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
                                     FALSE,                   // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
                                     (SmVolume &)*m_pNextMap, // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                      2,                      // in : 2 = copy volume and save curve orig
                                      3,                      // in : 3 = delete both curve and volume when destructed
                                     &crContext) ;            // in : req for stack objs, opt for heap objs

        }

    } // end without OrientMap branch

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::MapIsoParametricCurve

/*******************************************************************//**
PURPOSE: Create InSpace and/or OutSpace IsoSurfaces from a ParamSpace IsoParamPlane

NOTES --- InIsoSurface  = InvOrient(IsoParamPlane)    
          OutIsoSurface = Orient(EvaluateSimple(IsoParamPlane) 
***********************************************************************/
SmStatus SmVolume::EvaluateIsoParametricSurface
  (const SmContext   & crContext,               // in : context for created objects
   SmVolumeParamType   eConstantParam,          // in : specify the IsoPlane constant parameter 
                                                //      SM_VP_U = create constant u isoParameter surface
                                                //      SM_VP_V = create constant v isoParameter surface
                                                //      SM_VP_W = create constant w isoParameter surface
   double              dIsoParameter,           // in : Defines parametric value at which to extract the surface.  
                                                //      When eConstantParam==SM_VP_U, Param = constant u
                                                //                           SM_VP_V, Param = constant v
                                                //                           SM_VP_W, Param = constant w
   double              d3DTolerance,            // in : d3DTolerance
   SmSurface        ** ppNewOutSpaceIsoSurface, // out: OutSpace IsoSurface, NULL to ignore, default:[NULL]
   SmSurface        ** ppNewInSpaceIsoSurface,  // out: InSpace IsoSurface, NULL to ignore, default:[NULL] 
   const SmExtent3d  * pOptParamTrimBox,        // in : opt ParamSpace trim box for the IsoCurve
   SmBoolean           bWithCompounding)        // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
const
{
  // no work - no outputs
  if(   ppNewInSpaceIsoSurface  == NULL 
     && ppNewOutSpaceIsoSurface == NULL)
    { return SM_SUCCESS ; }

  // locals
  SmExtent2d        sFreeUV ;
  SmExtent1d        sFixedIvl ;
  SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
  const SmExtent3d *pTgtDomain   = pOptParamTrimBox ? pOptParamTrimBox : &sParamDomain ;

  // InSpace IsoSurface
  if(ppNewInSpaceIsoSurface)
    {
      // init output
      if( *ppNewInSpaceIsoSurface )  { delete *ppNewInSpaceIsoSurface ; *ppNewInSpaceIsoSurface = NULL ; }

      // with orient map
      if(m_pOrientMap)
        {
          // Build InSpace IsoSurface
          SER(m_pOrientMap->EvaluateIsoParametricSurfaceSimple( crContext, eConstantParam, 
                                                                dIsoParameter, 
                                                                d3DTolerance, 
                                                               *ppNewInSpaceIsoSurface, 
                                                                pOptParamTrimBox)) ;

        }
      else // without orient map: Orient == identity build isoPlane
        {
          // locals 
          SmPoint3d  sPuvw ;

          // need all 3 direction projections
          SmVector3d sU(1,0,0), sV(0,1,0), sW(0,0,1) ;
          SmVector3d sNormal, sDir1, sDir2 ;
          SmAxis2Placement sPlanePosition ;

          // watch out for confused user I/O
          eConstantParam = SM_INSPACE_TO_PARAM_TYPE(eConstantParam) ;

          // switch on varying parameter
          switch(eConstantParam)
            {
              // constant u
              case SM_VP_U : sPuvw.Set(dIsoParameter, 0.0, 0.0) ;
                             sNormal = sU ;
                             sDir1   = sV ;
                             sDir2   = sW ; 
                             sFreeUV.SetMinMax(pTgtDomain->GetVMin(), 
                                               pTgtDomain->GetWMin(),  
                                               pTgtDomain->GetVMax(),  
                                               pTgtDomain->GetWMax()) ;
                             sFixedIvl = pTgtDomain->GetUInterval() ;
                             break ;

              // constant v
              case SM_VP_V : sPuvw.Set(0.0, dIsoParameter, 0.0) ;
                             sNormal = sV ;
                             sDir1   = sW ;
                             sDir2   = sU ;                      
                             sFreeUV.SetMinMax(pTgtDomain->GetWMin(), 
                                               pTgtDomain->GetUMin(),  
                                               pTgtDomain->GetWMax(),  
                                               pTgtDomain->GetUMax()) ;
                             sFixedIvl = pTgtDomain->GetVInterval() ;
                             break ;

              // constant w
              case SM_VP_W : sPuvw.Set(0.0, 0.0, dIsoParameter) ;
                             sNormal = sW ;
                             sDir1   = sU ;
                             sDir2   = sV ;                      
                             sFreeUV.SetMinMax(pTgtDomain->GetUMin(), 
                                               pTgtDomain->GetVMin(),  
                                               pTgtDomain->GetUMax(),  
                                               pTgtDomain->GetVMax()) ;
                             sFixedIvl = pTgtDomain->GetWInterval() ;
                             break ;

              default : SER(SM_ERR) ;
            }

          // watch for containment
          if(   pOptParamTrimBox == NULL
             || sFixedIvl.ContainsValue(dIsoParameter, SM_EFF_ZERO))
            {
              // infinite plane
              *ppNewInSpaceIsoSurface = new (crContext) SmPlane(sPuvw, sNormal) ;

              // control the basis vector directions
              sPlanePosition.SetCanonical(sPuvw, sDir1, sDir2) ; 
              ((SmPlane *)(*ppNewInSpaceIsoSurface))->SetPosition(sPlanePosition) ;

              // and trimming
              (*ppNewInSpaceIsoSurface)->AdjustSTEPUVDomain(sFreeUV) ;

            } // end watch for trimming check 
        } // end without Orient map branch
    } // end need InSpaceIsoSurface check

  // OutSpace IsoSurface from InSpace
  if(ppNewOutSpaceIsoSurface)
    {
      // init output
      if( *ppNewOutSpaceIsoSurface )  { delete *ppNewOutSpaceIsoSurface ; *ppNewOutSpaceIsoSurface = NULL ; }

      // Create ProjSpace IsoSurface
      SER(EvaluateIsoParametricSurfaceSimple( crContext, eConstantParam, 
                                              dIsoParameter, 
                                              d3DTolerance, 
                                             *ppNewOutSpaceIsoSurface, 
                                              pOptParamTrimBox)) ;

      // Map ProjSpace IsoSurface to 1stOutSpace when needed
      if(m_pOrientMap && *ppNewOutSpaceIsoSurface != NULL)
        {
          *ppNewOutSpaceIsoSurface = new (crContext) SmSrfInVolume
                                       (**ppNewOutSpaceIsoSurface, // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this
                                        FALSE,                     // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace
                                        (SmVolume&)*m_pOrientMap,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                         2,                        // in : 2 = copy volume and save surface orig
                                         3,                        // in : 3 = delete both surface and volume when destructed
                                        &crContext) ;              // in : req for new stack objs, opt for new heap objs

        } // end need to orient ProjSpace IsoSurface check

      // map 1stOutSpace IsoSurface to lastOutSpace when needed
      if(m_pNextMap && bWithCompounding && *ppNewOutSpaceIsoSurface != NULL)
        {
          *ppNewOutSpaceIsoSurface = new (crContext) SmSrfInVolume
                                       (**ppNewOutSpaceIsoSurface, // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this
                                        FALSE,                     // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace
                                        (SmVolume &)*m_pNextMap,   // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                         2,                        // in : 2 = copy volume and save surface orig
                                         3,                        // in : 3 = delete both surface and volume when destructed
                                        &crContext) ;              // in : req for new stack objs, opt for new heap objs
        }
    } // end need OutSpaceIsoSurface check

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::EvaluateIsoParametricSurface

/*******************************************************************//**
PURPOSE: When possible build the exact Curve produced by projecting
    rInSpaceCurve from InSpace to last OutSpace

NOTES: This SmVolume level method manages the recursion required to support compounded maps
  and depends on the derived virtual implementation of 
  MakeExactBSpline1stOutCurve(eInputSpace==SM_VS_IN_SPACE).

  When any compounding of the InSpaceCurve cannot produce an exact BSplineGeometry,
  rpNewCurve is set to NULL and SM_SUCCESS is returned.
***********************************************************************/
SmStatus SmVolume::MakeExactBSplineOutCurveFromInSpace
 (const SmCurve   & rInSpaceCurve, // in : Tgt Curve to project to Last OutSpace as an exact curve if possible             
  SmBSplineCurve *& rpNewCurve)    // in : LastOutSpace exact projection of rInSpaceCurve, context:[rInSpaceCurve.GetContext()]
 const
{
  // init output and locals
  rpNewCurve                         = NULL ;
  const SmVolume * pThisMap          = this ;
  const SmCurve  * pThisInSpaceCurve = &rInSpaceCurve ;
  SmBoolean        bIsTemp           = FALSE ;

  // for every compounding map
  for(;pThisMap!=NULL;pThisMap=pThisMap->m_pNextMap)
    { 
      // see if the current InSpaceCurve can project exactly to the Next OutSpace 
      SER(pThisMap->MakeExactBSpline1stOutCurve(SM_VS_IN_SPACE, *pThisInSpaceCurve, rpNewCurve)) ;

      // free intermediate curves
      if(bIsTemp && pThisInSpaceCurve != NULL) 
        { delete pThisInSpaceCurve ; pThisInSpaceCurve = NULL ; } 

      // return NULL when an exact projection was not possible
      if(rpNewCurve == NULL) 
        { return(SM_SUCCESS) ; }

      // set up for the next iteration
      bIsTemp           = TRUE ;
      pThisInSpaceCurve = rpNewCurve ;

    } // end iter every compounding map

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      const SmEdge *pEdge = (SmEdge *)(rInSpaceCurve.GetEdge()) ;
      const SmBrep *pBrep = pEdge ? pEdge->GetBrep() : NULL ;

      SM_DUMP_AND_ASSERT_VALID(pBrep) ;
      SM_DUMP_AND_ASSERT_VALID(rpNewCurve) ;
      SM_DUMP_AND_ASSERT_VALID(pEdge) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInSpaceCurve.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInSpaceCurve.DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; rpNewCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; rpNewCurve->DrawParams() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ; 

} // end MakeExactBSplineOutCurveFromInSpace

/*******************************************************************//**
PURPOSE: When possible build the exact Curve produced by projecting
    rParamSpaceCurve from ParamSpace to last OutSpace

NOTES: This SmVolume level method manages the recursion required to support compounded maps
  and depends on the derived virtual implementations of 
  MakeExactBSpline1stOutCurve(SM_VS_PARAM_SPACE), and
  MakeExactBSpline1stOutCurve(SM_VS_IN_SPACE).

  When any compounding of the ParamSpaceCurve cannot produce an exact BSplineGeometry,
  rpNewCurve is set to NULL and SM_SUCCESS is returned.
***********************************************************************/
SmStatus SmVolume::MakeExactBSplineOutCurveFromParamSpace
 (const SmCurve   & rParamSpaceCurve, // in : Tgt Curve to project to Last OutSpace as an exact curve if possible             
  SmBSplineCurve *& rpNewCurve)       // in : LastOutSpace exact projection of rParamSpaceCurve, context:[rParamSpaceCurve.GetContext()]
 const
{
  // init output and locals
  rpNewCurve                         = NULL ;
  const SmVolume * pThisMap          = this ;

  // for 1st volume mapping - Get 1stOutSpace project if possible
  SER(pThisMap->MakeExactBSpline1stOutCurve(SM_VS_PARAM_SPACE, rParamSpaceCurve, rpNewCurve)) ;

  // return NULL when an exact projection was not possible
  if(rpNewCurve == NULL) 
    { return(SM_SUCCESS) ; }

  // set up for compounding
  pThisMap                           = pThisMap->m_pNextMap ;
  const SmCurve  * pThisInSpaceCurve = rpNewCurve ;

  // for every subsequent compounding map
  for(;pThisMap!=NULL;pThisMap=pThisMap->m_pNextMap)
    { 
      // see if the current InSpaceCurve can project exactly to the Next OutSpace 
      SER(pThisMap->MakeExactBSpline1stOutCurve(SM_VS_IN_SPACE, *pThisInSpaceCurve, rpNewCurve)) ;

      // Free intermediate curves
      if(pThisInSpaceCurve)
        { delete pThisInSpaceCurve ; pThisInSpaceCurve = NULL ; }

      // return NULL when an exact projection was not possible
      if(rpNewCurve == NULL) 
        { return(SM_SUCCESS) ; }

      // set up for the next iteration
      pThisInSpaceCurve = rpNewCurve ;
    }

  // all done
  return(SM_SUCCESS) ; 

} // end MakeExactBSplineOutCurveFromParamSpace

/*******************************************************************//**
PURPOSE: When possible build the exact Surface produced by projecting
    rInSpaceSurface from InSpace to last OutSpace

NOTES: This SmVolume level method manages the recursion required to support compounded maps
  and depends on the derived virtual implementation of 
  MakeExactBSpline1stOutSurfaceFromInSpace().

  When any compounding of the InSpaceSurface cannot produce an exact BSplineGeometry,
  rpNewSurface is set to NULL and SM_SUCCESS is returned.
***********************************************************************/
SmStatus SmVolume::MakeExactBSplineOutSurfaceFromInSpace
 (const SmSurface   & rInSpaceSurface, // in : Tgt Surface to project to Last OutSpace as an exact curve if possible             
  SmBSplineSurface *& rpNewSurface)    // in : LastOutSpace exact projection of rInSpaceSurface, context:[rInSpaceSurface.GetContext()]
 const
{
  // init output and locals
  rpNewSurface                           = NULL ;
  const SmVolume   * pThisMap            = this ;
  const SmSurface  * pThisInSpaceSurface = &rInSpaceSurface ;

  // for every compounding map
  for(;pThisMap!=NULL;pThisMap=pThisMap->m_pNextMap)
    { 
      // see if the current InSpaceSurface can project exactly to the Next OutSpace 
      rpNewSurface = NULL ;
      SER(pThisMap->MakeExactBSpline1stOutSurface(SM_VS_IN_SPACE, *pThisInSpaceSurface, rpNewSurface)) ;

      // Free intermediate surfaces
      if(pThisInSpaceSurface && (pThisInSpaceSurface != &rInSpaceSurface))
        { delete pThisInSpaceSurface ; pThisInSpaceSurface = NULL ; }

      // return NULL when an exact projection was not possible
      if(rpNewSurface == NULL) 
        { return(SM_SUCCESS) ; }

      // set up for the next iteration
      pThisInSpaceSurface = rpNewSurface ;
    
    } // end iter all compounded maps

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      const SmFace *pFace = (SmFace *)(rInSpaceSurface.GetFace()) ;
      const SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
      SM_ASSERT_VALID(pFace) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInSpaceSurface.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,0) ; rInSpaceSurface.DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; rInSpaceSurface.DrawVectorField(rInSpaceSurface.GetNaturalUVDomain(),5,5,SM_DM_UNIT_NORMAL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; rpNewSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; rpNewSurface->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; rpNewSurface->DrawVectorField(rpNewSurface->GetNaturalUVDomain(),5,5,SM_DM_UNIT_NORMAL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->DrawUV() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ; 

} // end MakeExactBSplineOutSurfaceFromInSpace

/*******************************************************************//**
PURPOSE: When possible build the exact Surface produced by projecting
    rParamSpaceSurface from ParamSpace to last OutSpace

NOTES: This SmVolume level method manages the recursion required to support compounded maps
  and depends on the derived virtual implementations of 
  MakeExactBSpline1stOutSurface(SM_VS_IN_SPACE), and
  MakeExactBSpline1stOutSurface(SM_VS_PARAM_SPACE).

  When any compounding of the ParamSpaceSurface cannot produce an exact BSplineGeometry,
  rpNewSurface is set to NULL and SM_SUCCESS is returned.
***********************************************************************/
SmStatus SmVolume::MakeExactBSplineOutSurfaceFromParamSpace
 (const SmSurface   & rParamSpaceSurface, // in : Tgt Surface to project to Last OutSpace as an exact curve if possible             
  SmBSplineSurface *& rpNewSurface)       // in : LastOutSpace exact projection of rParamSpaceSurface, context:[rParamSpaceSurface.GetContext()]
 const
{
  // init output and locals
  rpNewSurface              = NULL ;
  const SmVolume * pThisMap = this ;

  // for 1st volume mapping - Get 1stOutSpace project if possible
  SER(pThisMap->MakeExactBSpline1stOutSurface(SM_VS_PARAM_SPACE, rParamSpaceSurface, rpNewSurface)) ;

  // return NULL when an exact projection was not possible
  if(rpNewSurface == NULL) 
    { return(SM_SUCCESS) ; }

  // set up for compounding
  pThisMap                              = pThisMap->m_pNextMap ;
  const SmSurface * pThisInSpaceSurface = rpNewSurface ;

  // for every subsequent compounding map
  for(;pThisMap!=NULL;pThisMap=pThisMap->m_pNextMap)
    { 
      // see if the current InSpaceSurface can project exactly to the Next OutSpace 
      SER(pThisMap->MakeExactBSpline1stOutSurface(SM_VS_IN_SPACE, *pThisInSpaceSurface, rpNewSurface)) ;

      // return NULL when an exact projection was not possible
      if(rpNewSurface == NULL) 
        { return(SM_SUCCESS) ; }

      // set up for the next iteration
      if(pThisInSpaceSurface) { delete pThisInSpaceSurface ; pThisInSpaceSurface = NULL ; } 
      pThisInSpaceSurface = rpNewSurface ;
    }

  // all done
  return(SM_SUCCESS) ; 

} // end MakeExactBSplineOutSurfaceFromParamSpace

// for first release - exclude SmVolume::MapIsoParametricSurface method.  All the code is built but there 
//  are bugs I just don't see in the complicated trimming function, SmBendVolume::FindParamExtentForInSpacePlaneSimple()
//  that will be looked at later. 
//      
//       *******************************************************************//**
//      PURPOSE: Create OutSpace IsoSurface from a InSpace IsoParamLine
//      
//      NOTES --- OutIsoSurface = Compound(Orient(EvaluateSimple(InvOrient(IsoInSpaceLine))))
//      *********************************************************************** 
//      SmStatus SmVolume::MapIsoParametricSurface
//       (const SmContext     & crContext,                     // in : context for new created objects
//        SmVolumeParamType     eConstantParam,                // in : specify the IsoPlane constant parameter
//                                                             //      When eConstantParam==SM_VP_X_IN = constant Xin isosurface
//                                                             //                           SM_VP_Y_IN = constant Yin isosurface
//                                                             //                           SM_VP_Z_IN = constant Zin isosurface
//        double                dIsoParameter,                 // in : specify the constant InSpace parameter value
//                                                             //      When eConstantParam==SM_VP_X_IN, Param = constant Xin 
//                                                             //                           SM_VP_Y_IN, Param = constant Yin 
//                                                             //                           SM_VP_Z_IN, Param = constant Zin 
//        double                d3DTolerance,                  // in : max allowed 3d approximation distance
//        SmSurface          *& rpNewOutSpaceIsoSurface,       // out: OutSpace IsoSurface
//        const SmExtent3d    * pOptInSpaceTrimBox,            // in : opt InSpace TrimBox, NULL=Use NaturalBBox, default:[NULL]
//        SmBoolean             bWithCompounding)              // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
//       const  
//      {
//        // pick InSpace bounding box - making sure that it maps to a legal ParamDomain
//        SmExtent3d sTgtDomain = pOptInSpaceTrimBox ? *pOptInSpaceTrimBox : GetNaturalInSpaceDomain() ;
//      
//        // gwc - this seems to be too conservative - Instead trim FreeUV domain to legal ParamDomain later 
//        // SmStatus   eStat      = TrimInSpaceBoundingBox(sTgtDomain, sTgtDomain) ;
//        
//        // init output
//        if( rpNewOutSpaceIsoSurface )  { delete rpNewOutSpaceIsoSurface ; rpNewOutSpaceIsoSurface = NULL ; }
//      
//        // with orient map
//        if(m_pOrientMap)
//          {
//            // locals 
//            SmPoint3d  sPin ;
//      
//            // need all 3 direction projections
//            SmVector3d sXin(1,0,0), sYin(0,1,0), sZin(0,0,1) ;
//            SmVector3d sNormal, sDir1, sDir2 ;
//            SmVector2d sScale(1.0,1.0) ;
//            SmExtent2d sFreeUV ;
//            SmExtent1d sFixedIvl ;
//            SmAxis2Placement sPlanePosition ;
//            SmExtent3d sNaturalParamDomain = GetNaturalParamDomain() ;
//      
//            // watch out for confused user I/O
//            eConstantParam = SM_PARAM_TO_INSPACE_TYPE(eConstantParam) ;
//      
//            // switch on varying parameter
//            switch(eConstantParam)
//              {
//                // constant Xin
//                case SM_VP_X_IN : sPin.Set(dIsoParameter, 0.0, 0.0) ;
//                                  sNormal = sXin ;
//                                  sDir1   = sYin ;
//                                  sDir2   = sZin ;
//                                  sFreeUV.SetMinMax(sTgtDomain.GetVMin(), 
//                                                    sTgtDomain.GetWMin(),  
//                                                    sTgtDomain.GetVMax(),  
//                                                    sTgtDomain.GetWMax()) ;
//                                  sFixedIvl = sTgtDomain.GetUInterval() ;                     
//                                  break ;
//      
//                // constant Yin
//                case SM_VP_Y_IN : sPin.Set(0.0, dIsoParameter, 0.0) ;
//                                  sNormal = sYin ;
//                                  sDir1   = sZin ;
//                                  sDir2   = sXin ;                      
//                                  sFreeUV.SetMinMax(sTgtDomain.GetWMin(), 
//                                                    sTgtDomain.GetUMin(),  
//                                                    sTgtDomain.GetWMax(),  
//                                                    sTgtDomain.GetUMax()) ;
//                                  sFixedIvl = sTgtDomain.GetVInterval() ;                     
//                                  break ;
//      
//                // constant Zin
//                case SM_VP_Z_IN : sPin.Set(0.0, 0.0, dIsoParameter) ;
//                                  sNormal = sZin ;
//                                  sDir1   = sXin ;
//                                  sDir2   = sYin ;                      
//                                  sFreeUV.SetMinMax(sTgtDomain.GetUMin(), 
//                                                    sTgtDomain.GetVMin(),  
//                                                    sTgtDomain.GetUMax(),  
//                                                    sTgtDomain.GetVMax()) ;
//                                  sFixedIvl = sTgtDomain.GetWInterval() ;                     
//                                  break ;
//      
//                default : SER(SM_ERR) ;
//              }
//      
//            // when the NaturalParamDomain is bounded, see if the InSpace domain needs to be trimmed
//            if(sNaturalParamDomain.IsBounded())
//              {
//                // check state - see if the plane intersects the NaturalParamDomain
//                double dDistToPlane = sNaturalParamDomain.DistanceToPlane(sPin, sNormal) ;
//      
//                // no IsoSurface to return when plane does not intersect NaturalParamDomain
//                if(dDistToPlane > SM_EFF_ZERO)
//                  {
//                    return(SM_SUCCESS) ;
//                  }
//      
//                // test UIvls at top and bottom of sFreeUV for containment in the ParamDomain
//                SmTArray<SmExtent1d> sU1TrimIvls, sU2TrimIvls ;
//                SmExtent1d  sCurrentIvlU = sFreeUV.GetUInterval() ;
//                SmPoint3d   sInLinePointU1 = sPin + sFreeUV.GetVMin() * sDir2 ;
//                SmPoint3d   sInLinePointU2 = sPin + sFreeUV.GetVMax() * sDir2 ;
//                FindParamIntervalForInSpaceLine(sInLinePointU1, sDir1, sCurrentIvlU, sU1TrimIvls) ;
//                FindParamIntervalForInSpaceLine(sInLinePointU2, sDir1, sCurrentIvlU, sU2TrimIvls) ;
//      
//                // when no part of the FreeUV domain is legal - no IsoSurface to build
//                if(   sU1TrimIvls.GetSize() == 0
//                   || sU2TrimIvls.GetSize() == 0
//                   || sU1TrimIvls[0].IsDegenerate()
//                   || sU2TrimIvls[0].IsDegenerate())
//                 { return SM_SUCCESS ; }
//      
//                // test VIvls at left and right of sFreeUV for containment in the ParamDomain
//                SmTArray<SmExtent1d> sV1TrimIvls, sV2TrimIvls ;
//                SmExtent1d  sCurrentIvlV = sFreeUV.GetVInterval() ;
//                SmPoint3d   sInLinePointV1 = sPin + sFreeUV.GetUMin() * sDir1 ;
//                SmPoint3d   sInLinePointV2 = sPin + sFreeUV.GetUMax() * sDir1 ;
//                FindParamIntervalForInSpaceLine(sInLinePointV1, sDir2, sCurrentIvlV, sV1TrimIvls) ;
//                FindParamIntervalForInSpaceLine(sInLinePointV2, sDir2, sCurrentIvlV, sV2TrimIvls) ;
//            
//                // when no part of the FreeUV domain is legal - no IsoSurface to build
//                if(   sV1TrimIvls.GetSize() == 0
//                   || sV2TrimIvls.GetSize() == 0
//                   || sV1TrimIvls[0].IsDegenerate()
//                   || sV2TrimIvls[0].IsDegenerate())
//                 { return SM_SUCCESS ; }
//      
//                // Trim sFreeUV to that portion known to be within the legal ParamDomain
//                sFreeUV.SetMinMax( smos_3Max(sU1TrimIvls[0].GetMin(), sU2TrimIvls[0].GetMin(), sFreeUV.GetUMin()),
//                                   smos_3Max(sV1TrimIvls[0].GetMin(), sV2TrimIvls[0].GetMin(), sFreeUV.GetVMin()),
//                                   smos_3Min(sU1TrimIvls[0].GetMax(), sU2TrimIvls[0].GetMax(), sFreeUV.GetUMax()),
//                                   smos_3Min(sV1TrimIvls[0].GetMax(), sV2TrimIvls[0].GetMax(), sFreeUV.GetVMax()) ) ;
//      
//      
//              } // end need to trim to NaturalParamDomain check
//      
//            // watch for containment 
//            if(   (   pOptInSpaceTrimBox == NULL
//                   || sFixedIvl.ContainsValue(dIsoParameter, SM_EFF_ZERO))
//               && !sFreeUV.IsDegenerate()) 
//              {
//                // infinite plane
//                rpNewOutSpaceIsoSurface = new (crContext) SmPlane(sPin, sNormal) ;
//      
//                // control the basis vector directions
//                sPlanePosition.SetCanonical(sPin, sDir1, sDir2) ; 
//                ((SmPlane *)rpNewOutSpaceIsoSurface)->SetPosition(sPlanePosition) ;
//      
//                // and trimming
//                rpNewOutSpaceIsoSurface->AdjustSTEPUVDomain(sFreeUV) ;
//      
//                // see if the InSpace domain needs to be trimmed to limits in the NaturalParamDomain
//                if(sNaturalParamDomain.AnyBounds())
//                  {
//                    // find a domain of the Plane that is within the bounded NaturalParamDomain
//                    SmTArray<SmExtent2d> sTrimUVs ;
//                    FindParamExtentForInSpacePlane(sPin, sDir1, sDir2, sFreeUV, sTrimUVs) ;
//      
//                    // no IsoSurfact to return when remaining plain domain is degenerate
//                    if(sTrimUVs.GetSize() == 0 || sTrimUVs[0].IsDegenerate())
//                      {  return(SM_SUCCESS) ; }
//      
//                    // and trimming
//                    rpNewOutSpaceIsoSurface->AdjustSTEPUVDomain(sTrimUVs[0]) ;
//      
//                  } // end need to trim to Bounded NaturalParamDomain check
//              } // end watch for trimming branch
//      
//            else // trimming has culled isoCurve
//              {
//                // all done  
//                return(SM_SUCCESS) ;
//              }
//      
//            // Map InSpace IsoPlane to LastOutSpace 
//            rpNewOutSpaceIsoSurface = new (crContext) SmSrfInVolume(*rpNewOutSpaceIsoSurface, 
//                                                                    FALSE,
//                                                                    (SmVolume&)*this, 
//                                                                     2,           // 2 = copy volume and save surf orig
//                                                                     3,           // 3 = delete both surf and volume when destructed
//                                                                    &crContext) ;
//             
//            // when asked to skip compounding - truncate mapping to 1stOutSpace
//            if(bWithCompounding == FALSE)
//              {
//                // remove compounding from rpNewOutSpaceIsoCurve's volume
//                SmSrfInVolume * pSrfInVolume = (SmSrfInVolume *)rpNewOutSpaceIsoSurface ;
//                SmVolume      * pVolume      = (SmVolume *)pSrfInVolume->GetVolume() ;
//                pVolume->SetNextMap(NULL) ;
//              }
//      
//          } // end with OrientMap branch
//        else // without OrientMap branch
//          {
//            // Build 1stOutSpace IsoSurface = SimpleEvaluate IsoSurface
//            SER(EvaluateIsoParametricSurfaceSimple(crContext, SM_INSPACE_TO_PARAM_TYPE(eConstantParam), 
//                                                   dIsoParameter, 
//                                                   d3DTolerance, 
//                                                   rpNewOutSpaceIsoSurface, 
//                                                   pOptInSpaceTrimBox)) ;
//      
//            // map 1stOutSpace IsoSurface to LastOutSpace IsoSurface when needed
//            if(m_pNextMap && bWithCompounding && rpNewOutSpaceIsoSurface != NULL)
//              {
//                // Compound OutSpace IsoCurve to final OutSpace 
//                rpNewOutSpaceIsoSurface = new (crContext) SmSrfInVolume(*rpNewOutSpaceIsoSurface, 
//                                                                        FALSE,
//                                                                        (SmVolume &)*m_pNextMap, 
//                                                                         2,           // 2 = copy volume and save surf orig
//                                                                         3,           // 3 = delete both surf and volume when destructed
//                                                                        &crContext) ;
//      
//              }
//      
//          } // end without OrientMap branch
//      
//        // all done
//        return(SM_SUCCESS) ;
//      
//      } // end SmVolume::MapIsoParametricSurface

/*******************************************************************//**
PURPOSE: Compute OutPoint = Evaluate(ParamPoint) with compounding

NOTES:  Evaluate(ParamPoint) = Orient(EvaluateSimple(ParamPoint))
***********************************************************************/
SmStatus SmVolume::Evaluate
  (const SmPoint3d & crParamPoint,   // in : ParamSpace point to map to OutSpace point
   ULONG             lHighestDeriv,  // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3
   SmBoolean         bUFromLeft,     // in : if P is on U, V, or W interval boundary
   SmBoolean         bVFromLeft,     //      TRUE  = evaluate P in upper interval where P is on the left of the interval
   SmBoolean         bWFromLeft,     //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmVector3d      * aDerivatives,   // out: matrix of OutSpace evaluations values
                                     //      3d organized: D[u][v][w]
                                     //      1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3
                                     //      sized       : [n+1][n+1][n+1], where n=lHighesDeriv
                                     //      lHghDrv = 0,   sized: [1],      
                                     //        i=0          order: [D]
                                     //      lHghDrv = 1,   sized: [8]    
                                     //        i=u*4+v*2+w  order: [D  Dw  Dv  ---
                                     //                             Du --- --- ---]
                                     //      lHghDrv = 2,   sized: [27]   
                                     //        i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---
                                     //                             Du  Duw --- Duv --- --- --- --- ---
                                     //                             Duu --- --- --- --- --- --- --- ---]
                                     //      lHghDrv = 3,   sized: [81]   
                                     //        i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw
                                     //                             --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---
                                     //                             Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---
                                     //                             --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  --- 
                                     //                             --- ---  ---   ---   ---  ---  ---  ---  Duuu --- 
                                     //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                     //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                     //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                     //                             --- --- ]
   SmBoolean bNonZeroTangents,       // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                     //      FALSE= return exact tangent values, default:[TRUE]
                                     //      note: Surprisingly TRUE is the common choice because most tangent uses
                                     //            are for their direction (Binorm, SurfNorm comps), but when the 
                                     //            tangent is being used for its magnitude (like an arc-length comp)
                                     //            then set this to FALSE.
                                     //      default:[TRUE]               
   SmBoolean bDoZeroSampling,        // in : for internal use only, always set to TRUE, default:[TRUE]
   SmBoolean bWithCompounding)       // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const  
{
  // check input - number of supported derivatives is exceeded
  SER_MSG(lHighestDeriv <= 3 ? SM_SUCCESS : SM_ERR,
          _T("Volume Evaluate request exceeds maximum derivative count")) ;

  // locals
  SmBoolean  bNextMap = m_pNextMap && bWithCompounding ;
  SmVector3d aEvalSimpleDerivs[4*4*4] ;
  SmVector3d aOrientDerivs[4*4*4] ;   
  SmVector3d aNextDerivs[4*4*4] ; 
  SmVector3d aMidDerivs[4*4*4] ;    

  // Map InSpace point to 1st OutSpace evaluations

  if(m_pOrientMap)
    { 
      SM_ASSERT(m_pInvOrientMap && m_pOrientMap) ;

      // Map ParamPoint to ProjSpace
      SER(EvaluateSimple(crParamPoint, lHighestDeriv, 
                         bUFromLeft, bVFromLeft, bWFromLeft, 
                         aEvalSimpleDerivs, 
                         bNonZeroTangents, bDoZeroSampling)) ;

      // Map ProjPoint to 1st OutSpace
      SER(m_pOrientMap->EvaluateSimple(aEvalSimpleDerivs[0], lHighestDeriv, 
                                       bUFromLeft, bVFromLeft, bWFromLeft, 
                                       aOrientDerivs, 
                                       bNonZeroTangents, bDoZeroSampling)) ;

      // concatenate with the chain rule the individual evaluates of the compound mapping
      smvol_ConcatEvals(lHighestDeriv, aEvalSimpleDerivs, aOrientDerivs, bNextMap ? aMidDerivs : aDerivatives) ;

    }
  else // no Orient map, InSpace == ParamSpace and ProjSpace == 1stOutSpace
    {
      // Map ParamPoint to ProjSpace
      SER(EvaluateSimple(crParamPoint, lHighestDeriv, 
                         bUFromLeft, bVFromLeft, bWFromLeft, 
                         bNextMap ? aMidDerivs : aDerivatives, 
                         bNonZeroTangents, bDoZeroSampling)) ;
    }
  
  // arrive here when crInSpacePoint has been mapped to 1st OutSpace with concatenation
  // when bNextMap == TRUE,  evaluation is in aMid2Derivs
  // when bNextMap == FALSE, evaluation is in outout, aDerivatives
  
  // Map OutSpace Point through concatenated NextMaps
  if(bNextMap)
    {
      SER(m_pNextMap->Map(aMidDerivs[0], lHighestDeriv, 
                          bUFromLeft, bVFromLeft, bWFromLeft, 
                          aNextDerivs, 
                          bNonZeroTangents, bDoZeroSampling, bWithCompounding)) ;

      // compound the evaluations with the chain rule
      smvol_ConcatEvals(lHighestDeriv, aMidDerivs, aNextDerivs, aDerivatives) ;
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::Evaluate

/*******************************************************************//**
PURPOSE: Compute OutPoint = Evaluate(ParamPoint) with compounding
         for position.

NOTES: Evaluate(ParamPoint) = Orient(EvaluateSimple(ParamPoint))
***********************************************************************/
SmStatus SmVolume::EvaluatePoint
  (const SmPoint3d & crParamPoint,      // in : ParamSpace point to map to OutSpace point
   SmPoint3d       & rOutSpacePoint,    // out: mapped point in outSpace
   SmBoolean         bWithCompounding)  // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // pass the call along
  return( Evaluate( crParamPoint, 0, TRUE, TRUE, TRUE, &rOutSpacePoint,
                    TRUE,                  // in : bNonZeroTangents
                    TRUE,                  // in : bDoZeroSampling
                    bWithCompounding) ) ;  // in : bWithCompounding

} // end SmVolume::EvaluatePoint

/*******************************************************************//**
PURPOSE: Compute OutPoint = Evaluate(ParamPoint) with compounding
         for position and first partial derivatives.

NOTES: Evaluate(ParamPoint) = Orient(EvaluateSimple(ParamPoint))
***********************************************************************/
SmStatus SmVolume::Evaluate1stDerivatives
 (const SmPoint3d & crParamPoint,     // in : ParamSpace point to map to OutSpace point
  SmBoolean         bUFromLeft,       // in : if P is on U, V, or W interval boundary
  SmBoolean         bVFromLeft,       //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bWFromLeft,       //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmPoint3d       & rOutSpacePoint,   // out: OutSpace Point
  SmVector3d      & rDUout,           // out: OutSpace U direction tangent
  SmVector3d      & rDVout,           // out: OutSpace V direction tangent
  SmVector3d      & rDWout,           // out: OutSpace W direction tangent
  SmBoolean         bNonZeroTangents, // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                      //      FALSE= return exact tangent values, default:[TRUE]    
  SmBoolean         bWithCompounding) // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // pass the call along
  SmVector3d sMat[2][2][2];
  SER(Evaluate(crParamPoint, 1, bUFromLeft, bVFromLeft, bWFromLeft, sMat[0][0], 
               bNonZeroTangents,    // in : bNonZeroTangents
               TRUE,                // in : bDoZeroSampling 
               bWithCompounding)) ; // in : bWithCompounding

  // set output
  rOutSpacePoint = sMat[0][0][0];
  rDUout         = sMat[1][0][0];
  rDVout         = sMat[0][1][0];
  rDWout         = sMat[0][0][1];

  // all done
  return SM_SUCCESS;

} // end SmVolume::Evaluate1stDerivatives

/*******************************************************************//**
PURPOSE: Compute OutPoint = Evaluate(ParamPoint) with compounding
         for position, first partial, and second partial derivatives.

NOTES: Evaluate(ParamPoint) = Orient(EvaluateSimple(ParamPoint))
***********************************************************************/
SmStatus SmVolume::Evaluate2ndDerivatives
 (const SmPoint3d & crParamPoint,     // in : ParamSpace point to map to OutSpace point
  SmBoolean         bUFromLeft,       // in : if P is on U, V, or W interval boundary
  SmBoolean         bVFromLeft,       //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bWFromLeft,       //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmPoint3d       & rOutSpacePoint,   // out: OutSpace position           
  SmVector3d      & rDUout,           // out: OutSpace U direction tangent           
  SmVector3d      & rDVout,           // out: OutSpace V direction tangent           
  SmVector3d      & rDWout,           // out: OutSpace W direction tangent           
  SmVector3d      & rDUUout,          // out: OutSpace UU direction 2nd derivative   
  SmVector3d      & rDVVout,          // out: OutSpace VV direction 2nd derivative   
  SmVector3d      & rDWWout,          // out: OutSpace WW direction 2nd derivative   
  SmVector3d      & rDUVout,          // out: OutSpace UV 2nd order cross derivative 
  SmVector3d      & rDUWout,          // out: OutSpace UW 2nd order cross derivative 
  SmVector3d      & rDVWout,          // out: OutSpace VW 2nd order cross derivative 
  SmBoolean         bNonZeroTangents, // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                      //      FALSE= return exact tangent values, default:[TRUE]
  SmBoolean         bWithCompounding) // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{                            
  SmVector3d sMat[3][3][3];  
  SER(Evaluate(crParamPoint, 2, bUFromLeft, bVFromLeft, bWFromLeft, sMat[0][0], 
               bNonZeroTangents,     // in : bNonZeroTangents
               TRUE,                 // in : bDoZeroSampling 
               bWithCompounding)) ;  // in : bWithCompounding

  rOutSpacePoint = sMat[0][0][0];    
  rDUout         = sMat[1][0][0];    
  rDVout         = sMat[0][1][0];    
  rDWout         = sMat[0][0][1];    
  rDUUout        = sMat[2][0][0];    
  rDVVout        = sMat[0][2][0];    
  rDWWout        = sMat[0][0][2];    
  rDUVout        = sMat[1][1][0];    
  rDUWout        = sMat[1][0][1];    
  rDVWout        = sMat[0][1][1];    
                             
  return SM_SUCCESS;         

} // end SmVolume::Evaluate2ndDerivatives

/*******************************************************************//**
PURPOSE: Compute higher order directional derivatives using the chain
  rule.

NOTES: Limited to computing 3rd derivatives
***********************************************************************/
SmStatus SmVolume::EvaluateDirectionalDerivs
 (const SmPoint3d  & crParamPoint,   // in : ParamSpace Point to evaluate
  SmVector3d       & rParamDir,      // in : ParamSpace direction for derivatives, gets unitized
  ULONG              lHighestDeriv,  // in : 1=1st deriv, 2=1st and 2nd derivs, 3=1st, 2nd, and 3rd derivs
  SmVector3d         aDirDerivs[],   // out: aDirDerivs[0] = position
                                     //      aDirDerivs[1] = 1st directional derivative
                                     //      aDirDerivs[2] = 2nd directional derivative
                                     //      aDirDerivs[3] = 3rd directional derivative
                                     //      sized:[lHighestDeriv+1]
  SmBoolean          bUFromLeft,     // in : if P is on U, V, or W interval boundary
  SmBoolean          bVFromLeft,     //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean          bWFromLeft)     //      FALSE = evaluate P in lower interval where P is on the right of the interval
 const
{
  // check state
  SER_MSG(lHighestDeriv <= 3 ? SM_SUCCESS : SM_ERR, _T("Directional Derivatives only supported to 3rd order")) ;

  // make the surface evaluation
  SmVector3d aDerivs[SM_VSIZE(3)] ;
  SER(Evaluate(crParamPoint, lHighestDeriv, bUFromLeft, bVFromLeft, bWFromLeft, aDerivs, 
               FALSE,    // in : bNonZeroTangents: FALSE = use exact tangent values
               TRUE,     // in : bDoZeroSampling 
               TRUE)) ;  // in : bWithCompounding

  // pass the call along
  SER(smgu_VolDirectionalDerivs(rParamDir, lHighestDeriv, aDerivs, aDirDerivs)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::EvaluateDirectionalDerivs

/*******************************************************************//**
PURPOSE: invert map point from OutSpace back to ParamSpace on this 3d volume.

NOTES:
 1. Depending on the value of bWithCompounding, the input crOutSpacePoint is
    expected to be in different spaces.
      When bWithCompounding == TRUE : crOutSpacePoint is in the final OutSpace of the last compounded volume
           bWithCompounding == FALSE: crOutSpacePoint is in the first OutSpace of this SmVolume
 
 2. rParamPoint is always found in the first ParamSpace of this Volume
***********************************************************************/
SmStatus SmVolume::InvEvaluatePoint
 (const SmPoint3d     & crOutSpacePoint,      // in : OutSpace Point to map back to ParamSpace Point
  const SmPoint3d     * pOptParamGuess,       // in : last intermediate ParamSpace guess location at which to start the search
  SmBoolean           & rbSuccess,            // out: TRUE = inverse was found, else FALSE
  SmTArray<SmPoint3d> & rParamPoints,         // out: the ParamSpace inverse mapping
  SmTArray<double>    & rdGaps,               // out: max distance between found result (in case of bounding) and target point
  const SmExtent3d    * pOptParamDomain,      // in : ParamSpace domain over which to search for the inverse point
                                              //      NULL = use Map's NaturalDomain, default:[NULL]
  SmBoolean             bWithCompounding)     // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // init output
  ULONG ii ;
  rbSuccess = FALSE ;
  rParamPoints.ReSet() ;
  rdGaps.ReSet() ;

  // locals
  SmTArray<SmPoint3d> s1stOutSpacePoints ;

  // invert all compound maps from last to first with recursion
  if(m_pNextMap && bWithCompounding)
    {
      // map pOptParamDomain to this map outSpace to get Next InSpaceDomain.
      SmExtent3d   sNextInBox ;  
      SmPseudoBox  sNextInPseudoBox, *pNextInPseudoBox = NULL ;
      if(pOptParamDomain)
        {
          EvaluateBoundingBox(*pOptParamDomain, NULL, &sNextInBox, &sNextInPseudoBox, NULL, NULL, FALSE) ;
          pNextInPseudoBox = &sNextInPseudoBox ;
        }

      // map pOptParamGuess to this map 1stOutSpace 
      SmPoint3d sNextInGuess, *pNextInGuess = NULL ;
      if(pOptParamGuess)
        {
          EvaluatePoint(*pOptParamGuess, sNextInGuess, FALSE) ;
          pNextInGuess = & sNextInGuess ;
        }
          
      // invert OutSpacePoint from lastOutSpace to m_pNextMap->InSpace (with pNextInGuess) 
      SER(m_pNextMap->InvMapPoint(crOutSpacePoint, pNextInGuess, rbSuccess, s1stOutSpacePoints, rdGaps, pNextInPseudoBox, TRUE)) ;
    }
  else // without compounding, 1stOutSpace == LastOutSpace
    {
      s1stOutSpacePoints.Add(crOutSpacePoint) ;
    }

  // arrive here when pOutSpace is this map's outspace point
  SmTArray<SmPoint3d> sThisParamPoints ;

  // map pOutSpace back to ParamSpace
  if(m_pOrientMap) 
     { 
       SmPoint3d sProjPoint ;

       // for every 1stOutSpacePoint
       for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
         {
           // map 1stOutSpacePoint back to ProjSpace
           SER(m_pInvOrientMap->EvaluatePointSimple(s1stOutSpacePoints[ii], sProjPoint)) ;

           // map ProjPoint back to ParamSpace
           SER(InvEvaluatePointSimple(sProjPoint, pOptParamGuess, rbSuccess, sThisParamPoints, rdGaps, pOptParamDomain)) ;

           // accumulate output
           rParamPoints.Append(sThisParamPoints) ;
         } // end iter every 1stOutSpacePoint
     }
   else // without an orient branch, 1stOutSpace == ProjSpace
     {
       // for every 1stOutSpacePoint
       for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
         {
           // map ProjPoint back to ParamSpace
           SER(InvEvaluatePointSimple(s1stOutSpacePoints[ii], pOptParamGuess, rbSuccess, sThisParamPoints, rdGaps, pOptParamDomain)) ;

           // accumulate output
           rParamPoints.Append(sThisParamPoints) ;
         } // end iter every 1stOutSpacePoint
     }

   // Get map found rParamPoint back to OutSpace
   SmPoint3d sOutSpace ;

   rdGaps.SetSize(rParamPoints.GetSize()) ;
   for(ii=0;ii<rParamPoints.GetSize();ii++)
     {
       EvaluatePoint(rParamPoints[ii], sOutSpace, bWithCompounding) ;
   
       // Set outputs
       rdGaps[ii] = crOutSpacePoint.DistanceBetween(sOutSpace) ;
     }

   // all done
   rbSuccess = TRUE ;
   return(SM_SUCCESS) ;

} // end SmVolume::InvEvaluatePoint

/*******************************************************************//**
PURPOSE: Drop a OutSpace 3d vector whose origin corresponds to the 
    mapping of the given Param crParam point to OutSpace
    back to Param vectors so that  
    rParamVec = xVecIn*rDX + yVecIn*rDY + zVecIn*rDZ

NOTES:
 1. Depending on the value of bWithCompounding, the input crOutSpaceVec is
    expected to be in different spaces.
      When bWithCompounding == TRUE : crOutSpaceVec is in the final OutSpace of the last compounded volume
           bWithCompounding == FALSE: crOutSpaceVec is in the first OutSpace of this SmVolume

 2. crParamPoint is always in the first ParamSpace of this this volume.
***********************************************************************/
SmStatus SmVolume::InvEvaluateVector
 (const SmVector3d     & crOutSpaceVec,          // in : OutSpace Vector to map back to InSpace Vector
  const SmPoint3d      & crParamPoint,           // in : ParamSpace Point specifying where the Drop will take place (see InvEvaluatePoint())
  SmTArray<SmVector3d> & rParamVecs,             // out: Dropped ParamSpace vecs (2 if crParamPoint maps through seam)
                                                 //      note: with rParamVecs[ii] = [ParamVecU, ParamVecV, ParamVecW] 
                                                 //                 rOutSpaceVec   = ParamVecU*rDX + ParamVecV*rDY + ParamVecW*rDZ
                                                 //            where rDX, rDY, rDZ are the outpspace 1stDeriv vectors at crParamPoint
  SmBoolean              bWithCompounding)       // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // check for seams
  SmBoolean bOnU, bOnV, bOnW ;
  SmBoolean bOnBoundary = IsOnBoundary(crParamPoint, bOnU, bOnV, bOnW) ;

  // init output
  rParamVecs.SetSize(bOnBoundary ? 2 : 1) ;

  // locals
  SmPoint3d sPnt;
  SmVector3d sOutSpace1stDerivs[3] ;
  
  // get first derivatives for OutPoint = Volume(InPoint) with compounding
  SER(Evaluate1stDerivatives(crParamPoint, TRUE, TRUE, TRUE,
                             sPnt, sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2],
                             TRUE,                 // in : bNonZeroTangents
                             bWithCompounding)) ;  // in : bWithCompounding

  // project rDropVec3D into non-orthogonal 1st derivative axis set
  SER(smvol_DropVector(sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2], crOutSpaceVec, rParamVecs[0]));

  // when on seam - evaluate other side
  if(bOnBoundary)
    {
      SER(Evaluate1stDerivatives(crParamPoint, FALSE, FALSE, FALSE,
                                 sPnt, sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2],
                                 TRUE,                 // in : bNonZeroTangents
                                 bWithCompounding)) ;  // in : bWithCompounding

      // project rDropVec3D into non-orthogonal 1st derivative axis set
      SER(smvol_DropVector(sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2], crOutSpaceVec, rParamVecs[1]));

    } // end on seam check

  // all done
  return SM_SUCCESS;

} // end SmVolume::InvEvaluateVector

/*******************************************************************//**
PURPOSE: get approximate ParamSpace location for given OutSpace point
         for upcoming newton raphson search

NOTES: For compounded Volumes gets the guess point in the very 1st
       ParamSpace of the compounding series.
***********************************************************************/
SmStatus SmVolume::InvEvaluateGuessPoint
 (const SmPoint3d     & crOutSpacePoint,        // in : OutSpace Point to map back to InSpace Point
  SmTArray<SmPoint3d> & rGuessParamPoints,      // out: ParamSpace Point near Target Point actual map back to ParamSpace
  SmBoolean             bWithCompounding)       // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // init output
  rGuessParamPoints.ReSet() ;

  // locals
  ULONG ii ;
  SmBoolean bNextMap = m_pNextMap && bWithCompounding ;
  SmTArray<SmPoint3d> s1stOutSpacePoints ;

  // map crOutSpacePoint from lastOutSpace to 1stOutSpace
  if(bNextMap)
    {
      SER(m_pNextMap->InvMapGuessPoint(crOutSpacePoint, s1stOutSpacePoints, bWithCompounding)) ;
    }
  else // 1stOutSpace == LastOutSpace
    {
      s1stOutSpacePoints.Add(crOutSpacePoint) ;
    }

  // arrive here when s1stOutSpacePoints is in this map's OutSpace
  SmPoint3d sGuessProjPoint ;
  SmTArray<SmPoint3d> sThisGuessParamPoints ;

  // map s1stOutSpacePoint back to ParamSpace
  if(m_pOrientMap) 
     { 
       // for every 1stOutSpace point
       for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
         {
           // map s1stOutSpacePoint back to ProjSpace
           SER(m_pInvOrientMap->EvaluatePointSimple(s1stOutSpacePoints[ii], sGuessProjPoint)) ;

           // map ProjPoint back to paramSpace
           SER(InvEvaluateGuessPointSimple(sGuessProjPoint, sThisGuessParamPoints)) ;

           // build output
           rGuessParamPoints.Append(sThisGuessParamPoints) ;

         } // end iter every 1stOutSpacePoint
     }
   else // 1stOutSpace == ProjSpace
     {
       // for every 1stOutSpace point
       for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
         {
           // map s1stOutSpacePoint back to ParamSpace
           SER(InvEvaluateGuessPointSimple(s1stOutSpacePoints[ii], sThisGuessParamPoints)) ;

           // build output
           rGuessParamPoints.Append(sThisGuessParamPoints) ;

         } // end iter every 1stOutSpacePoint
     } // end without orient map branch

   // all done
   return(SM_SUCCESS) ;

} // end SmVolume::InvEvaluateGuessPoint

/*******************************************************************//**
PURPOSE: Compute OutPoint = Map(InPoint) with compounding

NOTES:  Map(InPoint) = m_pNextMap(Orient(EvaluateSimple(InvOrient(InPoint))))
***********************************************************************/
SmStatus SmVolume::Map
  (const SmPoint3d & crInSpacePoint,       // in : InSpace point to map to OutSpace point
   ULONG             lHighestDeriv,        // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3
   SmBoolean         bXinFromLeft,         // in : if P is on Xin, Yin, or Zin interval boundary
   SmBoolean         bYinFromLeft,         //      TRUE  = evaluate P in upper interval where P is on the left of the interval
   SmBoolean         bZinFromLeft,         //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmVector3d      * aDerivatives,         // out: matrix of OutSpace evaluations values
                                           //      sized:[n+1][n+1][n+1], where n=lHighesDeriv
                                           //      indexing:[1dIndex = xIn*(n+1)*(n+1)+yIn*(n+1)+zIn], where xIn,yIn,Zin=number of partial derivs 
                                           //      3d organized: D[XdirCnt][YDirCnt][ZDirCnt]
                                           //      1d organized: for lHighestDeriv from 0 to 3,
                                           //            0,  sized:[1]    order:[D]
                                           //            1,  sized:[8]    order:[D    DzIn DyIn ----
                                           //                                    DxIn ---- ---- ----]
                                           //            2,  sized:[27]   order:[D     DzIn  DzzIn DyIn  DyzIn ----- DyyIn ----- -----
                                           //                                    DxIn  DxzIn ----- DxyIn ----- ----- ----- ----- -----
                                           //                                    DxxIn ----- ----- ----- ----- ----- ----- ----- -----]
                                           //            3,  sized:[64]   order: 1dIndex = u*16+v*4+w
                                           //                 [D     DzIn   DzzIn   DzzzIn  DyIn   DyzIn  DyzzIn ...    DyyIn  DyzIn
                                           //                  ...   ...    DyyyIn  ...     ...    ...    DxIn   DxzIn  DxzzIn ...
                                           //                  DxyIn DxyzIn ...     ...     DxyyIn ...    ...    ...    ...    ...  
                                           //                  ...   ...    DxxIn   DxxzIn  ...    ...    DxxyIn ...    ...    ... 
                                           //                  ...   ...    ...     ...     ...    ...    ...    ...    DxxxIn ... 
                                           //                  ...   ...    ...     ...     ...    ...    ...    ...    ....   ... 
                                           //                  ...   ...    ...     ...     ...    ...    ...    ...    ....   ... 
                                           //                  ...   ...    ...     ...     ...    ...    ...    ...    ....   ... 
                                           //                  ...   ... ]
   SmBoolean bNonZeroTangents,             // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                           //      FALSE= return exact tangent values, default:[TRUE]
                                           //      note: Surprisingly TRUE is the common choice because most tangent uses
                                           //            are for their direction (Binorm, SurfNorm comps), but when the 
                                           //            tangent is being used for its magnitude (like an arc-length comp)
                                           //            then set this to FALSE.
                                           //      default:[TRUE]               
   SmBoolean bDoZeroSampling,              // in : for internal use only, always set to TRUE, default:[TRUE]
   SmBoolean bWithCompounding)             // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
  const
{
  // check input - number of supported derivatives is exceeded
  SER_MSG(lHighestDeriv <= 3 ? SM_SUCCESS : SM_ERR,
          _T("Volume Evaluate request exceeds maximum derivative count")) ;

  // locals
  SmBoolean  bNextMap = m_pNextMap && bWithCompounding ;
  SmVector3d aInvOrientDerivs[4*4*4] ;
  SmVector3d aEvalSimpleDerivs[4*4*4] ;
  SmVector3d aOrientDerivs[4*4*4] ;   
  SmVector3d aNextDerivs[4*4*4] ; 
  SmVector3d aMid1Derivs[4*4*4] ;    
  SmVector3d aMid2Derivs[4*4*4] ; 
  
  // Map InSpace point to 1st OutSpace evaluations

  if(m_pOrientMap)
    { 
      SM_ASSERT(m_pInvOrientMap && m_pOrientMap) ;

      // map crInSpacePoint to ParamSpace   
      SER(m_pInvOrientMap->EvaluateSimple(crInSpacePoint, lHighestDeriv, 
                                          bXinFromLeft, bYinFromLeft, bZinFromLeft, 
                                          aInvOrientDerivs, 
                                          bNonZeroTangents, bDoZeroSampling)) ;

      // Map ParamPoint to ProjSpace
      SER(EvaluateSimple(aInvOrientDerivs[0], lHighestDeriv, 
                         bXinFromLeft, bYinFromLeft, bZinFromLeft, 
                         aEvalSimpleDerivs, 
                         bNonZeroTangents, bDoZeroSampling)) ;

      // Map ProjPoint to 1st OutSpace
      SER(m_pOrientMap->EvaluateSimple(aEvalSimpleDerivs[0], lHighestDeriv, 
                                       bXinFromLeft, bYinFromLeft, bZinFromLeft, 
                                       aOrientDerivs, 
                                       bNonZeroTangents, bDoZeroSampling)) ;

      // concatenate with the chain rule the individual evaluates of the compound mapping
      smvol_ConcatEvals(lHighestDeriv, aInvOrientDerivs, aEvalSimpleDerivs, aMid1Derivs) ;
      smvol_ConcatEvals(lHighestDeriv, aMid1Derivs, aOrientDerivs, bNextMap ? aMid2Derivs : aDerivatives) ;

    }
  else // no Orient map, InSpace == ParamSpace and ProjSpace == 1stOutSpace
    {
      // Map ParamPoint to ProjSpace
      SER(EvaluateSimple(crInSpacePoint, lHighestDeriv, 
                         bXinFromLeft, bYinFromLeft, bZinFromLeft, 
                         bNextMap ? aMid2Derivs : aDerivatives, 
                         bNonZeroTangents, bDoZeroSampling)) ;
    }
  
  // arrive here when crinSpacePoint has been mapped to 1st OutSpace with concatenation
  // when bNextMap == TRUE,  evaluation is in aMid2Derivs
  // when bNextMap == FALSE, evaluation is in outout, aDerivatives
  
  // Map OutSpace Point through concatenated NextMaps
  if(bNextMap)
    {
      SER(m_pNextMap->Map(aMid2Derivs[0], lHighestDeriv, 
                          bXinFromLeft, bYinFromLeft, bZinFromLeft, 
                          aNextDerivs, 
                          bNonZeroTangents, bDoZeroSampling, bWithCompounding)) ;

      // compound the evaluations with the chain rule
      smvol_ConcatEvals(lHighestDeriv, aMid2Derivs, aNextDerivs, aDerivatives) ;
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::Map

/*******************************************************************//**
PURPOSE: Compute OutPoint = Map(InPoint) with compounding for position.

NOTES:  Map(InPoint) = m_pNextMap(Orient(EvaluateSimple(InvOrient(InPoint))))
***********************************************************************/
SmStatus SmVolume::MapPoint
  (const SmPoint3d & crInSpacePoint,    // in : point to map from inSpace
   SmPoint3d       & rOutSpacePoint,    // out: mapped point in outSpace
   SmBoolean         bWithCompounding)  // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // pass the call along
  return( Map( crInSpacePoint, 0, TRUE, TRUE, TRUE, &rOutSpacePoint, 
               TRUE,                  // in : bNonZeroTangents
               TRUE,                  // in : bDoZeroSampling
               bWithCompounding) ) ;  // in : bWithCompounding

} // end SmVolume::MapPoint

/*******************************************************************//**
PURPOSE: Compute OutPoint = Map(InPoint) with compounding for 
         position and first partial derivatives.

NOTES:  Map(InPoint) = m_pNextMap(Orient(EvaluateSimple(InvOrient(InPoint))))
***********************************************************************/
SmStatus SmVolume::Map1stDerivatives
 (const SmPoint3d & crInSpacePoint,   // in : InSpace point to map to OutSpace point
  SmBoolean         bXinFromLeft,     // in : if P is on Xin, Yin, or Zin interval boundary
  SmBoolean         bYinFromLeft,     //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bZinFromLeft,     //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmPoint3d       & rOutSpacePoint,   // out: OutSpace Point
  SmVector3d      & rDXin,            // out: OutSpace Xin direction tangent
  SmVector3d      & rDYin,            // out: OutSpace Yin direction tangent
  SmVector3d      & rDZin,            // out: OutSpace Zin direction tangent
  SmBoolean bNonZeroTangents,         // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                      //      FALSE= return exact tangent values, default:[TRUE]
  SmBoolean         bWithCompounding) // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // pass the call along
  SmVector3d sMat[2][2][2];
  SER(Map(crInSpacePoint,1,bXinFromLeft, bYinFromLeft, bZinFromLeft, sMat[0][0], 
          bNonZeroTangents,     // in : bNonZeroTangents 
          TRUE,                 // in : bDoZeroSampling  
          bWithCompounding)) ;  // in : bWithCompounding 

  // set output
  rOutSpacePoint = sMat[0][0][0];
  rDXin          = sMat[1][0][0];
  rDYin          = sMat[0][1][0];
  rDZin          = sMat[0][0][1];

  // all done
  return SM_SUCCESS;

} // end SmVolume::Map1stDerivatives

/*******************************************************************//**
PURPOSE: Compute OutPoint = Map(InPoint) with compounding for 
         position, first partial, and second partial derivatives.

NOTES:  Map(InPoint) = m_pNextMap(Orient(EvaluateSimple(InvOrient(InPoint))))
***********************************************************************/
SmStatus SmVolume::Map2ndDerivatives
 (const SmPoint3d & crInSpacePoint,   // in : InSpace point to map to OutSpace Point
  SmBoolean         bXinFromLeft,     // in : if P is on Xin, Yin, or Zin interval boundary
  SmBoolean         bYinFromLeft,     //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bZinFromLeft,     //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmPoint3d       & rOutSpacePoint,   // out: OutSpace position           
  SmVector3d      & rDXin,            // out: OutSpace Xin direction tangent
  SmVector3d      & rDYin,            // out: OutSpace Yin direction tangent
  SmVector3d      & rDZin,            // out: OutSpace Zin direction tangent
  SmVector3d      & rDXXin,           // out: OutSpace Xin direction 2nd derivative
  SmVector3d      & rDYYin,           // out: OutSpace Yin direction 2nd derivative
  SmVector3d      & rDZZin,           // out: OutSpace Zin direction 2nd derivative
  SmVector3d      & rDXYin,           // out: OutSpace XYin 2nd order cross derivative
  SmVector3d      & rDXZin,           // out: OutSpace XZin 2nd order cross derivative
  SmVector3d      & rDYZin,           // out: OutSpace YZin 2nd order cross derivative
  SmBoolean         bNonZeroTangents, // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                      //      FALSE= return exact tangent values, default:[TRUE]
  SmBoolean         bWithCompounding) // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{                            
  SmVector3d sMat[3][3][3];  
  SER(Map(crInSpacePoint,2,bXinFromLeft, bYinFromLeft, bZinFromLeft, sMat[0][0], 
          bNonZeroTangents,    // in : bNonZeroTangents 
          TRUE,                // in : bDoZeroSampling  
          bWithCompounding)) ; // in : bWithCompounding 

  rOutSpacePoint = sMat[0][0][0];    
  rDXin          = sMat[1][0][0];    
  rDYin          = sMat[0][1][0];    
  rDZin          = sMat[0][0][1];    
  rDXXin         = sMat[2][0][0];    
  rDYYin         = sMat[0][2][0];    
  rDZZin         = sMat[0][0][2];    
  rDXYin         = sMat[1][1][0];    
  rDXZin         = sMat[1][0][1];    
  rDYZin         = sMat[0][1][1];    
                             
  return SM_SUCCESS;         

} // end SmVolume::Map2ndDerivatives

/*******************************************************************//**
PURPOSE: Compute higher order directional derivatives using the chain
  rule.

NOTES: Limited to computing 3rd derivatives
***********************************************************************/
SmStatus SmVolume::MapDirectionalDerivs
 (const SmPoint3d  & crInSpacePoint,     // in : InSpace Point to evaluate
  SmVector3d       & rInSpaceDir,        // in : InSpace direction for derivatives, gets unitized
  ULONG              lHighestDeriv,      // in : 1=1st deriv, 2=1st and 2nd derivs, 3=1st, 2nd, and 3rd derivs
  SmVector3d         aDirDerivs[],       // out: aDirDerivs[0] = position
                                         //      aDirDerivs[1] = 1st directional derivative
                                         //      aDirDerivs[2] = 2nd directional derivative
                                         //      aDirDerivs[3] = 3rd directional derivative
                                         //      sized:[lHighestDeriv+1]
  SmBoolean          bUFromLeft,         // in : if P is on U, V, or W interval boundary
  SmBoolean          bVFromLeft,         //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean          bWFromLeft,         //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmBoolean          bWithCompounding)   // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // check state
  SER_MSG(lHighestDeriv <= 3 ? SM_SUCCESS : SM_ERR, _T("Directional Derivatives only supported to 3rd order")) ;

  // make the surface evaluation
  SmVector3d aDerivs[SM_VSIZE(3)] ;
  SER(Map(crInSpacePoint, lHighestDeriv, bUFromLeft, bVFromLeft, bWFromLeft, aDerivs, 
          FALSE,               // exact tangent values
          TRUE,                // Do zero sampling
          bWithCompounding)) ; // TRUE=Do compounding, FALSE=Don't

  // pass the call along
  SER(smgu_VolDirectionalDerivs(rInSpaceDir, lHighestDeriv, aDerivs, aDirDerivs)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::MapDirectionalDerivs

/*******************************************************************//**
PURPOSE: invert map point from OutSpace back to InSpace on this 3d volume.

NOTES: 
***********************************************************************/
SmStatus SmVolume::InvMapPoint
 (const SmPoint3d     & crOutSpacePoint,        // in : OutSpace Point to map back to InSpace Point
  const SmPoint3d     * pOptInPointGuess,       // in : last intermediate InSpace guess location at which to start the search
  SmBoolean           & rbSuccess,              // out: TRUE = inverse was found, else FALSE
  SmTArray<SmPoint3d> & rInSpacePoints,         // out: the InSpace inverse mapping
  SmTArray<double>    & rdGaps,                 // out: max distance between found result (in case of bounding) and target point
  const SmPseudoBox   * pOptInSpaceDomain,      // in : InSpace domain over which to search for the inverse point
                                                //      NULL = use Map's NaturalDomain, default:[NULL]
  SmBoolean             bWithCompounding)       // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // init output
  rbSuccess = FALSE ;
  rInSpacePoints.ReSet() ;
  rdGaps.ReSet() ;

  // locals
  SmBoolean bNextMap = m_pNextMap && bWithCompounding ;
  SmTArray<SmPoint3d> s1stOutSpacePoints ;

  // map crOutSpacePoint from LastOutSpace back to 1stOutSpace
  if(bNextMap)
    {
      // map pOptInSpaceDomain to this map's outSpace to get Next InSpaceDomain.
      SmPseudoBox sNextInBox, *pNextInBox = NULL ;
      if(pOptInSpaceDomain)
        {
          MapPseudoBox(*pOptInSpaceDomain, NULL, &sNextInBox, 
                         FALSE,      // bWithCompounding
                         TRUE) ;     // bTrimToLegalParamSpace
          pNextInBox = & sNextInBox ;
        }

      // map pOptInPointGuess to this map outSpace to get Next InSpace point
      SmPoint3d sNextInGuess, *pNextInGuess = NULL ;
      if(pOptInPointGuess)
        {
          MapPoint(*pOptInPointGuess, sNextInGuess, FALSE) ;
          pNextInGuess = (SmVector3d *)&sNextInGuess ;
        }
          
      // Map OutSpacePoint to 1stOutSpace
      SER(m_pNextMap->InvMapPoint(crOutSpacePoint, pNextInGuess, rbSuccess, s1stOutSpacePoints, rdGaps, pNextInBox, TRUE)) ;
    }
  else // without compounding, 1stOutSpace == LastOutSpace
    {
      s1stOutSpacePoints.Add(crOutSpacePoint) ;
    }

  // arrive here when s1stOutSpacePoints are in this map's outspace point
  ULONG ii, jj ; 
  SmTArray<SmPoint3d> sThisParamPoints ;
  SmPoint3d sInSpacePoint ;

  // map s1stOutSpacePoint back to ParamSpace
  if(m_pOrientMap) 
     { 
       SmPoint3d sProjPoint ;

       // get ready - Map pOptInSpaceDomain to ParamSpace
      SmExtent3d sParamBox, *pParamBox = NULL ;
      if(pOptInSpaceDomain)
        {
          m_pInvOrientMap->EvaluatePseudoBoxSimple(*pOptInSpaceDomain, &sParamBox, NULL) ;
          TrimParamBoundingBoxSimple(sParamBox, sParamBox) ;
          pParamBox = &sParamBox ;
        }

       // get ready - when given an InSpace GuessPoint, map pOptInPointGuess to ParamSpace
       SmPoint3d sParamGuessPoint, *pParamGuessPoint = NULL ;
       if(pOptInPointGuess) { m_pInvOrientMap->EvaluatePointSimple(*pOptInPointGuess, sParamGuessPoint) ;
                              pParamGuessPoint = &sParamGuessPoint ;
                            }

       // for every 1stOutSpacePoint
       for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
         {
           // Map s1stOutSpacePoint back to ProjSpace 
           SER(m_pInvOrientMap->EvaluatePointSimple(s1stOutSpacePoints[ii], sProjPoint)) ;

           // Map ProjPoint back to ParamSpace
           SER(InvEvaluatePointSimple(sProjPoint, pParamGuessPoint, rbSuccess, sThisParamPoints, rdGaps, pParamBox)) ;

           // For EveryParamSpace Point
           for(jj=0;jj<sThisParamPoints.GetSize();jj++)
             {
               // Map ParamPoint back to InSpace
               SER(m_pOrientMap->EvaluatePointSimple(sThisParamPoints[jj], sInSpacePoint)) ;

               // accumulate output
               rInSpacePoints.Add(sInSpacePoint) ;
             } // end iter every ParamPoint
         }
     }
   else // without an orient branch, 1stOutSpace == ProjSpace and ParamSpace == InSpace
     {
       SmExtent3d sParamBox, *pParamBox = NULL ;
       if(pOptInSpaceDomain)
         {
           sParamBox = *pOptInSpaceDomain ;
           TrimParamBoundingBoxSimple(sParamBox, sParamBox) ;
           pParamBox = &sParamBox ;
         }

       // for every 1stOutSpacePoint
       for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
         {
           // map s1stOutSpacePoint back to InSpace
           SER(InvEvaluatePointSimple(s1stOutSpacePoints[ii], pOptInPointGuess, rbSuccess, sThisParamPoints, rdGaps, pParamBox)) ;

           // accumulate output
           rInSpacePoints.Append(sThisParamPoints) ;
         } // end iter every 1st OutSpace Point
     } // end without an orient map branch

   // map found rInSpacePoints back to OutSpace to calc gaps
   SmPoint3d sOutPoint ;
   rdGaps.SetSize( rInSpacePoints.GetSize() ) ;

   for(ii=0;ii<rInSpacePoints.GetSize();ii++)
     {
       MapPoint(rInSpacePoints[ii], sOutPoint, bWithCompounding) ;  

       // Set outputs
       rdGaps[ii] = crOutSpacePoint.DistanceBetween(sOutPoint) ;
     }

   // all done
   rbSuccess = TRUE ;
   return(SM_SUCCESS) ;

} // end SmVolume::InvMapPoint

/*******************************************************************//**
PURPOSE: Drop a OutSpace 3d vector whose origin corresponds to the 
    mapping of the given InSpace crInSpace point to OutSpace
    back to InSpace vectors so that  
    rInSpaceVec = xVecIn*rDX + yVecIn*rDY + zVecIn*rDZ

NOTES: 
***********************************************************************/
SmStatus SmVolume::InvMapVector
 (const SmVector3d     & crOutSpaceVec,        // in : OutSpace Vector to map back to InSpace Vector
  const SmPoint3d      & crInSpacePoint,       // in : InSpace Point specifying where the Drop will take place (see InvMapPoint())
  SmTArray<SmVector3d> & rInSpaceVecs,         // out: Dropped InSpace vecs (2 if crParamPoint maps through seam)
                                               //      note: with rInSpaceVecs[ii] = [InSpaceVecU, InSpaceVecV, InSpaceVecW] 
                                               //                 rOutSpaceVec   = InSpaceVecU*rDX + InSpaceVecV*rDY + InSpaceVecW*rDZ
                                               //            where rDX, rDY, rDZ are the outpspace 1stDeriv vectors at crInSpacePoint
  SmBoolean              bWithCompounding)     // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const   
{
  // check for seams
  SmPoint3d sParamPoint ;
  SmBoolean bOnU, bOnV, bOnW ;
  if(m_pInvOrientMap) { m_pInvOrientMap->EvaluatePointSimple(crInSpacePoint, sParamPoint) ; }
  else                { sParamPoint = crOutSpaceVec ; }
  SmBoolean bOnBoundary = IsOnBoundary(sParamPoint, bOnU, bOnV, bOnW) ;

  // init output
  rInSpaceVecs.SetSize(bOnBoundary ? 2 : 1) ;

  // locals
  SmPoint3d sPnt;
  SmVector3d sOutSpace1stDerivs[3] ;
  
  // get first derivatives for OutPoint = Volume(InPoint)
  SER(Map1stDerivatives(crInSpacePoint, TRUE, TRUE, TRUE,
                        sPnt, sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2],
                        TRUE,               // in : bNonZeroTangents 
                        bWithCompounding)); // in : bWithCompounding 

  // project rDropVec3D into non-orthogonal 1st derivative axis set
  SER(smvol_DropVector(sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2], crOutSpaceVec, rInSpaceVecs[0]));

  // when on seam - evaluate other side
  if(bOnBoundary)
    {
      SER(Map1stDerivatives(crInSpacePoint, FALSE, FALSE, FALSE,
                            sPnt, sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2],
                            TRUE,                 // in : bNonZeroTangents
                            bWithCompounding)) ;  // in : bWithCompounding

      // project rDropVec3D into non-orthogonal 1st derivative axis set
      SER(smvol_DropVector(sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2], crOutSpaceVec, rInSpaceVecs[1]));

    } // end on seam check

  // all done
  return SM_SUCCESS;

} // end SmVolume::InvMapVector

/*******************************************************************//**
PURPOSE: get approximate InSpace location for given OutSpace point
         for upcoming newton raphson search

NOTES: For compounded Volumes gets the guess point in the very 1st
 InSpace of the compounding series.
***********************************************************************/
SmStatus SmVolume::InvMapGuessPoint
 (const SmPoint3d     & crOutSpacePoint,        // in : OutSpace Point to map back to InSpace Point
  SmTArray<SmPoint3d> & rGuessInSpacePoints,    // out: InSpace Point near Target Point actual map back to InSpace
  SmBoolean             bWithCompounding)       // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // init output
  rGuessInSpacePoints.ReSet() ;

  // locals
  ULONG ii, jj ;
  SmBoolean bNextMap = m_pNextMap && bWithCompounding ;
  SmTArray<SmPoint3d> s1stOutSpacePoints ; 

  // invert all compound maps from last to first with recursion
  if(bNextMap)
    {
      SER(m_pNextMap->InvMapGuessPoint(crOutSpacePoint, s1stOutSpacePoints, bWithCompounding)) ;
    }
  else // without compounding, 1stOutSpace == LastOutSpace
    {
      s1stOutSpacePoints.Add(crOutSpacePoint) ;
    }

  // arrive here when s1stOutSpacePoints is in this map's OutSpace
  SmPoint3d sThisGuessProjPoint, sThisGuessInSpacePoint ;
  SmTArray<SmPoint3d> sThisGuessParamPoints ;
 
  // map s1stOutSpacePoint back to InSpace
  if(m_pOrientMap) 
     { 
       // for every 1stOutSpace point
       for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
         {
           // map s1stOutSpacePoint back to ProjSpace 
           SER(m_pInvOrientMap->EvaluatePointSimple(s1stOutSpacePoints[ii], sThisGuessProjPoint)) ;

           // map ProjPoint back to ParamSpace
           SER(InvEvaluateGuessPointSimple(sThisGuessProjPoint, sThisGuessParamPoints)) ;

           // for every ThisGuessParamPoint
           for(jj=0;jj<sThisGuessParamPoints.GetSize();jj++)
             {
               // map ParamPoint back to InSpace
               SER(m_pOrientMap->EvaluatePointSimple(sThisGuessParamPoints[jj], sThisGuessInSpacePoint)) ;

               // accumulate output
               rGuessInSpacePoints.Add(sThisGuessInSpacePoint) ;
             } // end iter every ThisGuessParamPoint
         } // end iter every 1stOutSpace point 
     }
   else // without OrientMap, 1stOutSpace == ProjSpace and ParamSpace == InSpace
     {
       // for every 1stOutSpace point
       for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
         {
           // map s1stOutSpacePoint back to InSpace
           SER(InvEvaluateGuessPointSimple(s1stOutSpacePoints[ii], sThisGuessParamPoints)) ;

           // build output
           rGuessInSpacePoints.Append(sThisGuessParamPoints) ; 
         } // endi ter every 1stOutSpace point
     } // end without orient map branch

   // all done
   return(SM_SUCCESS) ;

} // end SmVolume::InvMapGuessPoint

/*******************************************************************//**
PURPOSE: Compute InPoint = Orient(ParamPoint) with compounding.

NOTES:  Orient(InPoint) = m_pOrientMap(ParamPoint)
***********************************************************************/
SmStatus SmVolume::Orient
 (const SmPoint3d & crParamPoint,         // in : ParamSpace point to map to InSpace point
  ULONG             lHighestDeriv,        // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3
  SmBoolean         bUFromLeft,           // in : if P is on U, V, or W interval boundary
  SmBoolean         bVFromLeft,           //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bWFromLeft,           //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d      * aDerivatives,         // out: matrix of InSpace evaluations values
                                          //      3d organized: D[u][v][w]
                                          //      1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3
                                          //      sized       : [n+1][n+1][n+1], where n=lHighesDeriv
                                          //      lHghDrv = 0,   sized: [1],      
                                          //        i=0          order: [D]
                                          //      lHghDrv = 1,   sized: [8]    
                                          //        i=u*4+v*2+w  order: [D  Dw  Dv  ---
                                          //                             Du --- --- ---]
                                          //      lHghDrv = 2,   sized: [27]   
                                          //        i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---
                                          //                             Du  Duw --- Duv --- --- --- --- ---
                                          //                             Duu --- --- --- --- --- --- --- ---]
                                          //      lHghDrv = 3,   sized: [81]   
                                          //        i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw
                                          //                             --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---
                                          //                             Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---
                                          //                             --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  Duuu --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                          //                             --- --- ]
  SmBoolean bNonZeroTangents,             // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                          //      FALSE= return exact tangent values, default:[TRUE]
                                          //      note: Surprisingly TRUE is the common choice because most tangent uses
                                          //            are for their direction (Binorm, SurfNorm comps), but when the 
                                          //            tangent is being used for its magnitude (like an arc-length comp)
                                          //            then set this to FALSE.
                                          //      default:[TRUE]               
  SmBoolean bDoZeroSampling)              // in : for internal use only, always set to TRUE, default:[TRUE]
 const 
{
  // when there is an orient map
  if(m_pOrientMap)
    {
      // return OrientMap's simple evaluation
      m_pOrientMap->EvaluateSimple(crParamPoint, 
                                   lHighestDeriv,
                                   bUFromLeft,        // Orient mapping is the Volume map of the m_pOrientMap object
                                   bVFromLeft,   
                                   bWFromLeft,   
                                   aDerivatives, 
                                   bNonZeroTangents,
                                   bDoZeroSampling) ;

    }
  else // without OrientMap, ParamSpace == InSpace, evaluate unity mapping
    {
      // check input
      if (lHighestDeriv > GW_MAX_DERIV) SER(SM_ERR_INVALID_INPUT);

      // init output array
      ULONG ii, dASize = (lHighestDeriv+1) * (lHighestDeriv+1) * (lHighestDeriv+1) ;
      for(ii=1;ii<dASize;ii++)
        {
          aDerivatives[ii].Set(0,0,0) ;
        }

// For indexing into the volume pt/deriv array:
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

      // position
      aDerivatives[0] = crParamPoint ; 

      // 1st derivs are unit axis vectors
      if(lHighestDeriv >= 1)
        {
          aDerivatives[sv(1,0,0)].Set(1,0,0) ;     // sDUin
          aDerivatives[sv(0,1,0)].Set(0,1,0) ;     // sDVin
          aDerivatives[sv(0,0,1)].Set(0,0,1) ;     // sDWin
        }

      // all higher derivs are zero

  // done indexing into arrays
#undef sv
    }
      
  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::Orient

/*******************************************************************//**
PURPOSE: Compute InPoint = Orient(ParamPoint) with compounding for 
         position.

NOTES:  Orient(InPoint) = m_pOrientMap(ParamPoint)
***********************************************************************/
SmStatus SmVolume::OrientPoint
 (const SmPoint3d & crParamPoint,         // in : ParamSpace point to map to InSpace point
  SmPoint3d       & rInSpacePoint)        // out: InSpace point mapped from ParamSpace point
  const
{
  // when there is an orient map
  if(m_pOrientMap)
    {
      // return OrientMap's simple evaluation
      return(m_pOrientMap->EvaluatePointSimple(crParamPoint, rInSpacePoint)) ;

    }
  else // return Unity mapping
    {
      rInSpacePoint = crParamPoint ;
    }

  // all done
  return( SM_SUCCESS ) ;

} // end SmVolume::OrientPoint

/*******************************************************************//**
PURPOSE: Compute InPoint = Orient(ParamPoint) with compounding for 
         position and first partial derivatives.

NOTES:  Orient(InPoint) = m_pOrientMap(ParamPoint)
***********************************************************************/
SmStatus SmVolume::Orient1stDerivatives
 (const SmPoint3d & crParamPoint,     // in : ParamSpace point to map to InSpace point
  SmBoolean         bUFromLeft,       // in : if P is on U, V,or W interval boundary
  SmBoolean         bVFromLeft,       //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bWFromLeft,       //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmPoint3d       & rInSpacePoint,    // out: InSpace Point
  SmVector3d      & rDUin,            // out: InSpace U direction tangent
  SmVector3d      & rDVin,            // out: InSpace V direction tangent
  SmVector3d      & rDWin,            // out: InSpace W direction tangent
  SmBoolean       bNonZeroTangents)   // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
 const                                //      FALSE= return exact tangent values, default:[TRUE]
{
  // when there is an orient map
  if(m_pOrientMap)
    {
      // return OrientMap's simple evaluation
      return(m_pOrientMap->Evaluate1stDerivativesSimple(crParamPoint, 
                                                        bUFromLeft, bVFromLeft, bWFromLeft,
                                                        rInSpacePoint,
                                                        rDUin, rDVin, rDWin,   
                                                        bNonZeroTangents)) ;

    }
  else // return Unity mapping
    {
      rInSpacePoint = crParamPoint ;
      rDUin.Set(1,0,0) ;
      rDVin.Set(0,1,0) ;
      rDWin.Set(0,0,1) ;
    }

  // all done
  return( SM_SUCCESS ) ;

} // end SmVolume::Orient1stDerivatives

/*******************************************************************//**
PURPOSE: Compute InPoint = Orient(ParamPoint) with compounding for 
         position, first partial, and second partial derivatives.

NOTES:  Orient(InPoint) = m_pOrientMap(ParamPoint)
***********************************************************************/
SmStatus SmVolume::Orient2ndDerivatives
 (const SmPoint3d & crParamPoint,     // in : ParamSpace point to map to InSpace Point
  SmBoolean         bUFromLeft,       // in : if P is on U, V, or W interval boundary
  SmBoolean         bVFromLeft,       //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bWFromLeft,       //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmPoint3d       & rInSpacePoint,    // out: InSpace position           
  SmVector3d      & rDUin,            // out: InSpace U direction tangent            
  SmVector3d      & rDVin,            // out: InSpace V direction tangent            
  SmVector3d      & rDWin,            // out: InSpace W direction tangent            
  SmVector3d      & rDUUin,           // out: InSpace Uin direction 2nd derivative   
  SmVector3d      & rDVVin,           // out: InSpace Vin direction 2nd derivative   
  SmVector3d      & rDWWin,           // out: InSpace Win direction 2nd derivative   
  SmVector3d      & rDUVin,           // out: InSpace UVin 2nd order cross derivative
  SmVector3d      & rDUWin,           // out: InSpace UWin 2nd order cross derivative
  SmVector3d      & rDVWin,           // out: InSpace VWin 2nd order cross derivative
  SmBoolean       bNonZeroTangents)   // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
 const                                //      FALSE= return exact tangent values, default:[TRUE]
{                            
  // when there is an orient map
  if(m_pOrientMap)
    {
      // return OrientMap's simple evaluation
      return(m_pOrientMap->Evaluate2ndDerivativesSimple(crParamPoint, 
                                                        bUFromLeft, bVFromLeft, bWFromLeft,
                                                        rInSpacePoint,
                                                        rDUin,  rDVin,  rDWin,
                                                        rDUUin, rDVVin, rDWWin,
                                                        rDUVin, rDUWin, rDVWin,   
                                                        bNonZeroTangents)) ;

    }
  else // return Unity mapping
    {
      rInSpacePoint = crParamPoint ;
      rDUin.Set(1,0,0) ;
      rDVin.Set(0,1,0) ;
      rDWin.Set(0,0,1) ;
      rDUUin.Set(0,0,0) ;
      rDVVin.Set(0,0,0) ;
      rDWWin.Set(0,0,0) ;
      rDUVin.Set(0,0,0) ;
      rDUWin.Set(0,0,0) ;
      rDVWin.Set(0,0,0) ;
    }

  // all done
  return( SM_SUCCESS ) ;

} // end SmVolume::Orient2ndDerivatives

/*******************************************************************//**
PURPOSE: Compute higher order directional derivatives using the chain
  rule.

NOTES: Limited to computing 3rd derivatives
***********************************************************************/
SmStatus SmVolume::OrientDirectionalDerivs
 (const SmPoint3d  & crParamPoint,   // in : ParamSpace Point to evaluate
  SmVector3d       & rParamDir,      // in : ParamSpace direction for derivatives, gets unitized
  ULONG              lHighestDeriv,  // in : 1=1st deriv, 2=1st and 2nd derivs, 3=1st, 2nd, and 3rd derivs
  SmVector3d         aDirDerivs[],   // out: InSpace aDirDerivs[0] = position
                                     //              aDirDerivs[1] = 1st directional derivative
                                     //              aDirDerivs[2] = 2nd directional derivative
                                     //              aDirDerivs[3] = 3rd directional derivative
                                     //      sized:[lHighestDeriv+1]
  SmBoolean          bUFromLeft,     // in : if P is on U, V, or W interval boundary
  SmBoolean          bVFromLeft,     //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean          bWFromLeft)     //      FALSE = evaluate P in lower interval where P is on the right of the interval
 const
{
  // check state
  SER_MSG(lHighestDeriv <= 3 ? SM_SUCCESS : SM_ERR, _T("Directional Derivatives only supported to 3rd order")) ;

  // make the surface evaluation
  SmVector3d aDerivs[SM_VSIZE(3)] ;
  SER(Orient(crParamPoint, lHighestDeriv, bUFromLeft, bVFromLeft, bWFromLeft, aDerivs, 
               FALSE,     // exact tangent values
               TRUE));    // Do zero sampling

  // pass the call along
  SER(smgu_VolDirectionalDerivs(rParamDir, lHighestDeriv, aDerivs, aDirDerivs)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::OrientDirectionalDerivs

/*******************************************************************//**
PURPOSE: invert map point from InSpace back to ParamSpace on this 3d volume.

NOTES: 
***********************************************************************/
SmStatus SmVolume::InvOrientPoint
 (const SmPoint3d  & crInSpacePoint,  // in : InSpace Point to map back to ParamSpace Point
  SmBoolean        & rbSuccess,       // out: TRUE = inverse was found, else FALSE
  SmPoint3d        & rParamPoint,     // out: the parameter space inverse mapping
  double           & rdGap)           // out: max distance between found result (in case of bounding) and target point
 const  
{
  // init output
  rbSuccess = TRUE ;
  rdGap     = 0.0 ;

  // when there is an orient map
  if(m_pOrientMap)
    {
      // return OrientMap's simple evaluation
      return(m_pInvOrientMap->EvaluatePointSimple(crInSpacePoint, rParamPoint)) ;
    }
  else // return Unity mapping
    {
      rParamPoint = crInSpacePoint ;
    }

  // all done
  return( SM_SUCCESS ) ;

} // end SmVolume::InvOrientPoint

/*******************************************************************//**
PURPOSE: Drop an InSpace 3d vector whose origin corresponds to the 
    mapping of the given ParamSpace crParamPoint point to InSpace
    back to ParamSpace vectors so that  
    rInSpaceVec   = ParamVecU*rDU + ParamVecV*rDV + ParamVecW*rDW

NOTES: 
***********************************************************************/
SmStatus SmVolume::InvOrientVector
 (const SmVector3d & crInSpaceVec,    // in : InSpace Vector to map back to ParamSpace Vector
  const SmPoint3d  & crParamPoint,    // in : ParamSpace Point specifying where the Drop will take place (see InvOrientPoint())
  SmVector3d       & rParamVec)       // out: Dropped ParamSpace vecs                                                    
 const                                //      note: with rParamVec = [ParamVecU, ParamVecV, ParamVecW]                  
                                      //                 rOutSpaceVec   = ParamVecU*rDX + ParamVecV*rDY + ParamVecW*rDZ  
                                      //            where rDX, rDY, rDZ are the InSpace 1stDeriv vectors at crParamPoint 
{
  // locals
  SmVector3d sOrient1stDerivs[3] ;

  // when there is an orient map
  if(m_pOrientMap)
    {
      // get Param unitVecs mapped to InSpace first derivatives through OrientMap 
      SmPoint3d sPnt;
      SER(m_pOrientMap->Evaluate1stDerivativesSimple(crParamPoint, TRUE, TRUE, TRUE,
                                                     sPnt, sOrient1stDerivs[0], sOrient1stDerivs[1], sOrient1stDerivs[2]));

      // project rDropVec3D into non-orthogonal 1st derivative axis set
      SER(smvol_DropVector(sOrient1stDerivs[0], sOrient1stDerivs[1], sOrient1stDerivs[2], crInSpaceVec, rParamVec));

    }
  else // return Unity mapping
    {
      rParamVec = crInSpaceVec ;
    }

  // all done
  return( SM_SUCCESS ) ;

} // end SmVolume::InvOrientVector

/*******************************************************************//**
PURPOSE: Convenience function to make SmVolume similar to SmSurface
         for mapping points from OutSpace back to Param Space without a guess point.

NOTES: same mapping as InvEvaluatePoint (without the guess point) 
***********************************************************************/
SmStatus SmVolume::GlobalPointSolve
 (const SmPoint3d  & crOutSpacePoint,       // in : OutSpace Point to map back to ParamSpace
  SmBoolean        & rbFoundAnswer,         // out: TRUE = successfully mapped OutSpace Point to a ParamSpace Point
  SmSolutionArray  & rSolutions,            // out: Contains ParamSpace Found Points
                                            //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                                               
                                            //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crOutSpacePoint);                      
                                            //      rSolution[i].m_vStart[0] =  u of [u,v,w] the found ParamSpace point location                    
                                            //      rSolution[i].m_vStart[1] =  v of [u,v,w] the found ParamSpace point location                    
                                            //      rSolution[i].m_vStart[2] =  w of [u,v,w] the found ParamSpace point location                    
  const SmExtent3d * pOptParamDomain,       // in : ParamSpace domain over which to search for the inverse point
                                            //      NULL = use Map's NaturalDomain, default:[NULL]
  SmBoolean          bWithCompounding)      // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const
{
  // init output
  rbFoundAnswer = FALSE ;
  rSolutions.ReSet() ;

  // locals   
  ULONG ii ; 
  SmBoolean bNextMap = m_pNextMap && bWithCompounding ;
  SmTArray<SmVector3d> s1stOutSpacePoints ;
  SmSolutionArray  sThisSolutions ;

  // when compounding
  if(bNextMap)
    {
      SmBoolean        bSuccess = TRUE ;
      SmTArray<double> dGaps ;

      // map pOptParamDomain from ParamSpace to 1stOutSpace
      SmExtent3d sNextInBox ;
      SmPseudoBox sNextInPseudoBox, *pNextInPseudoBox = NULL ;
      if(pOptParamDomain)
        {
          EvaluateBoundingBox(*pOptParamDomain, NULL, &sNextInBox, &sNextInPseudoBox, NULL, NULL, FALSE) ;
          pNextInPseudoBox = &sNextInPseudoBox ;
        }

      // map crOutSpacePoint from last OutSpace to 1st OutSpace
      SER(m_pNextMap->InvMapPoint(crOutSpacePoint, 
                                  NULL, 
                                  bSuccess, 
                                  s1stOutSpacePoints, 
                                  dGaps, 
                                  pNextInPseudoBox, // NULL = use entire Natural Domain
                                  TRUE) ) ;         // TRUE = WithCompounding

      // quit when mapping back to 1st OutSpace fails
      if(bSuccess == FALSE)
        { SER(SM_ERR) ; }

    }
  else // not compounding
    {
      // use the input OutSpacePoint
      s1stOutSpacePoints.Add(crOutSpacePoint) ;
    }

  // map OutSpacePoint back to ProjSpace
  if(m_pOrientMap)
    {
      SmVector3d sProjPoint ;

      // for every 1stOutSpace point
      for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
        {
          // map s1stOutSpacePont back to ProjSpace
          SER(m_pInvOrientMap->EvaluatePointSimple(s1stOutSpacePoints[ii], sProjPoint)) ;

          // map ProjPoint back to ParamSpace
          SER(GlobalPointSolveSimple(sProjPoint, rbFoundAnswer, sThisSolutions, pOptParamDomain)) ;

          // accumulate output
          rSolutions.Append(sThisSolutions) ;
        } // end iter every 1stOutSpace point

    } // end with orient map branch
  else // no orient map, 1stOutSpace == ProjSpace
    {
      // for every 1stOutSpace point
      for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
        {
          // Map from ProjSpace back to ParamSpace
          SER( GlobalPointSolveSimple(s1stOutSpacePoints[ii], rbFoundAnswer, sThisSolutions, pOptParamDomain)) ;

          // accumulate output
          rSolutions.Append(sThisSolutions) ;
        } // end iter every 1stOutSpace point
    } // end no orient map branch

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::GlobalPointSolve  

/*******************************************************************//**
PURPOSE: Convenience function to make SmVolume similar to SmSurface
         for mapping points from OutSpace back to Param Space with a guess point.

NOTES: same mapping as InvEvaluatePoint (with the guess point) 
***********************************************************************/
SmStatus SmVolume::LocalPointSolve
 (const SmPoint3d  & crOutSpacePoint,        // in : OutSpace point to map back to ParamSpace
  const SmPoint3d  & crParamPointGuess,      // in : ParamSpace point guess, the closer to the actual ParamSpace point the better
  SmBoolean        & rbFoundAnswer,          // out: TRUE = successfully mapped OutSpace Point to an ParamSpace Point
  SmSolutionArray  & rSolutions,             // out: Found ParamSpace Point coordinates and accuracy of inverse mapping
                                             //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                                               
                                             //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crOutSpacePoint);                      
                                             //      rSolution[i].m_vStart[0] =  u of [u,v,w] the found ParamSpace point location                    
                                             //      rSolution[i].m_vStart[1] =  v of [u,v,w] the found ParamSpace point location                    
                                             //      rSolution[i].m_vStart[2] =  w of [u,v,w] the found ParamSpace point location                    
  const SmExtent3d * pOptParamDomain,        // in : ParamSpace domain over which to search for the inverse point
                                             //      NULL = use Map's NaturalDomain, default:[NULL]
  SmBoolean          bWithCompounding)       // in : TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
 const  
{
  // init output
  rbFoundAnswer = FALSE ;
  rSolutions.ReSet() ;

  // locals
  ULONG ii ;
  SmBoolean bNextMap = m_pNextMap && bWithCompounding ;
  SmTArray <SmVector3d> s1stOutSpacePoints ;
  SmSolution sThisSolution ;

  // when compounding
  if(bNextMap)
    {
      SmBoolean        bSuccess ;
      SmTArray<double> dGaps ;

      // map pOptParamDomain from ParamSpace to 1stOutSpace
      SmExtent3d sNextInBox ;
      SmPseudoBox sNextInPseudoBox, *pNextInPseudoBox = NULL ;
      if(pOptParamDomain)
        {
          EvaluateBoundingBox(*pOptParamDomain, NULL, &sNextInBox, &sNextInPseudoBox, NULL, NULL, FALSE) ;
          pNextInPseudoBox = &sNextInPseudoBox ;
        }

      // map crParamPointGuess from ParamSpace to 1stOutSpace
      SmPoint3d sNextInGuess ;
      EvaluatePoint(crParamPointGuess, sNextInGuess, FALSE) ;

      // map from last OutSpace to 1st OutSpace (with the 1stOutSpace guess point)
      SER(m_pNextMap->InvMapPoint(crOutSpacePoint, 
                                  &sNextInGuess, 
                                  bSuccess, 
                                  s1stOutSpacePoints, 
                                  dGaps,
                                  pNextInPseudoBox,       // NULL = use entire natural domain
                                  bWithCompounding)) ;    // TRUE = with compounding

      // quit when mapping back to 1st OutSpace fails
      if(bSuccess == FALSE)
        { SER(SM_ERR) ; }

    }
  else // no m_pNextMap, 1stOutSpace == LastOutSpace
    {
      // use the input OutSpacePoint
      s1stOutSpacePoints.Add(crOutSpacePoint) ;
    }

  // map 1stOutSpacePoint back to ParamSpace
  if(m_pOrientMap)
    {
      SmVector3d sProjPoint ;

      // for every 1stOutSpace point
      for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
        {
          // map s1stOutSpacePoint from 1stOutSpace back to ProjSpace
          SER(m_pInvOrientMap->EvaluatePointSimple(s1stOutSpacePoints[ii], sProjPoint)) ;

          // map ProjPoint back to ParamSpace (using ParamPointGuess)
          SER( LocalPointSolveSimple(sProjPoint, crParamPointGuess, rbFoundAnswer, sThisSolution, pOptParamDomain)) ;
          
          // accumulate output
          rSolutions.Add(sThisSolution) ;

        } // end iter every 1stOutSpace point
    } // end with orient map branch
  else // no orient map, 1stOutSpace == ProjSpace
    {
      // for every 1stOutSpace point
      for(ii=0;ii<s1stOutSpacePoints.GetSize();ii++)
        {
          // Map from ProjSpace back to ParamSpace (using ParamPointGuess)
          SER( LocalPointSolveSimple(s1stOutSpacePoints[ii], crParamPointGuess, rbFoundAnswer, sThisSolution, pOptParamDomain)) ;
          
          // accumulate output
          rSolutions.Add(sThisSolution) ;

        } // end iter every 1stOutSpace point
    } // end no orient map branch

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::LocalPointSolve  

/*******************************************************************//*** 
To be called by SolveIt() in a Newton-Raphson iteration to
find a ParamSpace point in a volume (m_crVolume) that maps to a given
ProjSpace point (m_crProjSpacePoint).  The Newton-Raphson zero functions are

 fun[0] = EvaluateSimple(u, v, w).x-ProjSpacePoint.x,
 fun[1] = EvaluateSimple(u, v, w).y-ProjSpacePoint.y, and
 fun[2] = EvaluateSimple(u, v, w).z-ProjSpacePoint.z.

 The Jacobi, [ d(fun[0])/du d(fun[0])/dv d(fun[0])/dw ], for these functions is, 
             [ d(fun[1])/du d(fun[1])/dv d(fun[1])/dw ] 
             [ d(fun[2])/du d(fun[2])/dv d(fun[2])/dw ] 
                                   
 [ Vu.x  Vv.x  Vw.x ]
 [ Vu.y  Vv.y  Vw.y ]
 [ Vu.z  Vv.z  Vw.z ].

where V = Volume(u, v, w) a evaluated InPoint (yields an OutPoint)
      P = fixed point in OutSpace
      Vu = d(Volume(u, v, w))/du  - ProjSpace u direction tangent
      Vv = d(Volume(u, v, w))/dv  - ProjSpace v direction tangent
      Vw = d(Volume(u, v, w))/dw  - ProjSpace w direction tangent

************************************************************************/
class SmFindPVIntersectENFO : public SmEvalNFunctionsObject
{
protected:
  const SmPoint3d & m_crProjSpacePoint;       // Target ProjSpace Point     
  const SmVolume  & m_crVolume;               // Volume being searched     
  double m_dSquaredTolerance;                 // convergence criterial tolerance
                                              // converges when GuessPoint/m_crProjSpacePoint < Tolerance
                                              //           or   Gap/Tangent angle ~= cos(Gap/Tangent angle) * gapSize < tolerance
public:
  // constructor, destructor
  SmFindPVIntersectENFO(const SmPoint3d     & crProjSpacePoint, 
                        const SmVolume      & crVolume,
                        double                dSquaredTolerance) 
                      : m_crProjSpacePoint(crProjSpacePoint), 
                        m_crVolume(crVolume), 
                        m_dSquaredTolerance(dSquaredTolerance)
                     { }
  virtual ~SmFindPVIntersectENFO() { }

  // NewtonRaphson Iteration callback evaluate function
  virtual SmStatus Evaluate(const SmTArray<double> & crX,             // in : x of Ax=F     
                            SmTArray<double>       & rF,              // out: F of Ax=F function values,                     
                            SmMatrix               * pOptJacobian,    // out: Partial derivatives of the functions.
                            SmBoolean              & rbFoundAnswer);  // out: Not always used. When used
                                                                      //      TRUE = converged (F members are within tolerance of 0.0
                                                                      //      FALSE= Not Used or Not Converged
} ; // end class SmFindPVIntersectENFO

/*******************************************************************//**
PURPOSE:  Evaluate fun[0] = EvaluateSimple(u, v, w).x-ProjSpacePoint.x,
                   fun[1] = EvaluateSimple(u, v, w).y-ProjSpacePoint.y, and
                   fun[2] = EvaluateSimple(u, v, w).z-ProjSpacePoint.z.
NOTES: Returns SM_ERR whenever any Volume Tangent value goes to zero.
***********************************************************************/
SmStatus SmFindPVIntersectENFO::Evaluate
  (const SmTArray<double> & crX,           // in : x of Ax=F, [u v w] a ParamSpace Point
   SmTArray<double>       & rF,            // out: F of Ax=F function values, 
   SmMatrix               * pOptJacobian,  // out: Partial derivatives of the functions.              
   SmBoolean              & rbFoundAnswer) // out: TRUE = Current GuessPoint/m_crOutSpacePoint Gap less than tolerance
                                           //             or cos(gap/Tangent angle) * gapSize less than tolerance
                                           //      FALSE= Current gap larger than tolerance
{
  // check input
  SM_ASSERT(crX.GetSize() == 3);
  SM_ASSERT( rF.GetSize() == 3);
  if (pOptJacobian) 
    {
      SM_ASSERT(pOptJacobian->GetNumRows()    == 3);
      SM_ASSERT(pOptJacobian->GetNumColumns() == 3);
    }

  // init output
  rbFoundAnswer = FALSE;

  // locals
  SmPoint3d  sParamSpacePoint(crX[0],crX[1],crX[2]);
  SmPoint3d  sProjSpacePoint;
  SmVector3d sDU, sDV, sDW ;

  // get Volume evaluations
  SER(m_crVolume.Evaluate1stDerivativesSimple(sParamSpacePoint,TRUE,TRUE,TRUE,sProjSpacePoint,sDU, sDV, sDW));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  // draw Current GuessPoint on Volume(blue)
  if (bDebugMe) 
    {
      smgfx_SetLook(2,6, 0,0,1); m_crVolume.DrawAtParamPoint(sParamSpacePoint, 1); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // failure - zero length volume tangents
  double sDULenSq = sDU.LengthSquared();
  double sDVLenSq = sDV.LengthSquared();
  double sDWLenSq = sDW.LengthSquared();
  if(   sDULenSq < SM_EFF_ZERO_SQ 
     || sDVLenSq < SM_EFF_ZERO_SQ
     || sDWLenSq < SM_EFF_ZERO_SQ) 
    { return SM_ERR; }

  // Global Point Solve finds the location where the gap
  // between the target point and the Evaluation of the found point goes to zero.
  //
  // fun[0] = (V-P).x
  // fun[1] = (V-P).y
  // fun[2] = (V-P).z
  // 
  // Jacobial = [ VxIn.x  VyIn.x  VzIn.x  ]
  //            [ VxIn.y  VyIn.y  VzIn.y  ]
  //            [ VxIn.z  VyIn.z  VzIn.z  ]
  //
  // where V = Volume(xIn,yIn,zIn) ProjSpace Point for given ParamSpace Point
  //       P = fixed point in ProjSpace
  //       VxIn = d(V(xIn,yIn,zIn))/dxIn
  //       VyIn = d(V(xIn,yIn,zIn))/dyIn
  //       VzIn = d(V(xIn,yIn,zIn))/dzIn
  //
  
  // get gap vector
  SmVector3d sGap              = sProjSpacePoint - m_crProjSpacePoint;
  double     dGapLengthSquared = sGap.LengthSquared() ;

#ifdef SM_DEBUG_CODE
  // draw gap vector(red)
  if (bDebugMe) 
    {
      smgfx_SetLook(1,2, 0,1,1); sGap.Draw(&m_crProjSpacePoint); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  if (pOptJacobian) 
    {
      (*pOptJacobian)[0][0] = sDU.x ;     
      (*pOptJacobian)[0][1] = sDV.x ;     
      (*pOptJacobian)[0][2] = sDW.x ;     

      (*pOptJacobian)[1][0] = sDU.y ;
      (*pOptJacobian)[1][1] = sDV.y ;
      (*pOptJacobian)[1][2] = sDW.y ;

      (*pOptJacobian)[2][0] = sDU.z ;
      (*pOptJacobian)[2][1] = sDV.z ;
      (*pOptJacobian)[2][2] = sDW.z ;
    }
  
  // Compute function values
  rF[0] = sGap.x ;
  rF[1] = sGap.y ;
  rF[2] = sGap.z ;
  
  // convergence criteria
  double dsPointMax  = sProjSpacePoint.GetMaxDimension() ;
  double dcrPointMax = m_crProjSpacePoint.GetMaxDimension() ;
  double dScale      = 1.0 + smos_Max(dsPointMax, dcrPointMax) ;
  if (dGapLengthSquared < m_dSquaredTolerance * dScale * dScale) 
    {
      rbFoundAnswer = TRUE;
      return SM_SUCCESS;
    }

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      if (pOptJacobian) pOptJacobian->Dump();
      smos_WriteBuffer(_T(" X --- \n")); crX.Dump();
      smos_WriteBuffer(_T(" F --- \n")); rF.Dump();
      //      m_crVolume.Dump() ;

      // compute and display likely next point
      if(pOptJacobian)
        {
          // copy outputs
          SmTArray<double> sF(rF) ;
          SmMatrix sJacobian(*pOptJacobian) ;

          // Solve for next Newton/Raphson parameter Steps 
          SmTArray<double> sDeltas(3,NULL,3) ;
          for(ULONG ii=0; ii<3; ii++) { sF[ii] = -sF[ii] ; }
          if(SM_SUCCESS == sJacobian.SolveLinearSystem(sF,sDeltas))
            {
              // increment parameters and add NextPoint Display to Graphic
              SmPoint3d sParamSpaceNext(sParamSpacePoint.x + sDeltas[0],
                                        sParamSpacePoint.y + sDeltas[1],
                                        sParamSpacePoint.z + sDeltas[2]) ;
              smgfx_SetLook(2,6, 0,0,1); m_crVolume.DrawAtParamPoint(sParamSpaceNext,1); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
        } // end pOptJacobian existence check
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmFindPVIntersectENFO::Evaluate

/*******************************************************************//**
PURPOSE: Allocate a mirror image of this volume

NOTES: Default Base Class implementation behavior 
       uses expensive compounding to mirror any volume as
       MirrorVolume(uvw) = SmTransform_SetToMirror(OrigVolumeCopy(uvw))

       Any derived SmVolume classes that can mirror directly should
       do so in a virtual method for efficiencey so that
       MirrorVolume(uvw) = OrigVolumeCopy->Mirror() ;
***********************************************************************/
SmStatus SmVolume::CreateMirrorVolume
 (const SmContext        & crContext,         // in : context for new object construction
  const SmAxis2Placement & crMirrorPlane,     // in : volume is mirrored about the MirrorPlane's XY plane 
  SmVolume              *& rpMirrorVolume)    // out: newly allocated volume
{
  // init output
  rpMirrorVolume = NULL ;

  // Copy current Volume into output
  SER(this->Copy(crContext, rpMirrorVolume)) ;

  // Create a SmTransform Mirror mapping
  SmVector3d sMirrorNormal = crMirrorPlane.GetZAxis() ;
  SmTransform sTransform ;
  SER(sTransform.MirrorSimple(crMirrorPlane.GetOriginRef(), sMirrorNormal)) ;

  // Concatenate the Mirror map onto the copy map
  SER(rpMirrorVolume->AddNextMap(&sTransform, 1)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::CreateMirrorVolume

/*******************************************************************//**
PURPOSE: Translate Volume's Outspace Image by given Outspace Vector.

NOTES: Calls TranslateSimple() on the last map of the compound map list.

       TranslateSimple() Default Base Class implementation behavior: 
       uses expensive compounding to Translate any volume as
       Translate SmVolume = SmTransform_SetToTranslate(Orient(SimpleMap(uvw))) or
       ParamSpace -SimpleMap-> ProjSpace -Orient-> OutSpace -Translate-> FinalOutSpace

       For improved run-time evaluate performance Derived SmVolume classes that 
       can Translate by directly modifying the parameters of the SimpleMap 
       should do so in the virtual method TranslateSimple() so that:

       Orient(ModifiedSimpleMap(uvw)) = Translate(Orient(SimpleMap(uvw))) which is
       ModifiedSimpleMap(uvw)         = InvOrient(Translate(Orient(SimpleMap(uvw)))) 
***********************************************************************/
SmStatus SmVolume::Translate            
 (const SmVector3d & crOutTranslate)  // in : Outspace translate vector  
{
  SmVolume *pLastMap = GetLastMap() ;

  // save on m_pNextMap linked list length - try to build the Translate into the last NextMap
  if(pLastMap)
    {
      SER(pLastMap->TranslateSimple(crOutTranslate)) ; 
    }
  else // build the Translate into this map
    { 
      SER(this->TranslateSimple(crOutTranslate)) ; 
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::Translate

/*******************************************************************//**
PURPOSE: Mirror Volume's Outspace Image about given Outspace MirrorPlane.

NOTES: Calls MirrorSimple() on the last map of the compound map list.

       MirrorSimple() Default Base Class implementation behavior: 
       uses expensive compounding to Mirror any volume as
       Mirror SmVolume = SmTransform_SetToMirror(Orient(SimpleMap(uvw))) or
       ParamSpace -SimpleMap-> ProjSpace -Orient-> OutSpace -Mirror-> FinalOutSpace

       For improved run-time evaluate performance derived SmVolume classes that 
       can Mirror by directly modifying the parameters of the SimpleMap 
       should do so in the virtual method MirrorSimple() so that:

       Orient(ModifiedSimpleMap(uvw)) = Mirror(Orient(SimpleMap(uvw))) which is
       ModifiedSimpleMap(uvw)         = InvOrient(Mirror(Orient(SimpleMap(uvw)))) 
***********************************************************************/
SmStatus SmVolume::Mirror            
 (const SmAxis2Placement & crOutMirrorPlane)  // in : Outspace is mirrored about the MirrorPlane's Outspace XY plane  

{
  SmVolume *pLastMap = GetLastMap() ;

  // save on m_pNextMap linked list length - try to build the Mirror into the last NextMap
  if(pLastMap)
    {
      SER(pLastMap->MirrorSimple(crOutMirrorPlane.GetOriginRef(), crOutMirrorPlane.GetZAxis())) ; 
    }
  else // build the Mirror into this map
    { 
      SER(this->MirrorSimple(crOutMirrorPlane.GetOriginRef(), crOutMirrorPlane.GetZAxis())) ; 
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::Mirror

/*******************************************************************//**
PURPOSE: Scale Volume's Outspace Image about an optional Outspace ScalePoint.

NOTES: Calls ScaleSimple() on the last map of the compound map list.

       ScaleSimple() Default Base Class implementation behavior: 
       uses expensive compounding to Scale any volume as
       Scale SmVolume = SmTransform_SetToScale(Orient(SimpleMap(uvw))) or
       ParamSpace -SimpleMap-> ProjSpace -Orient-> OutSpace -Scale-> FinalOutSpace

       For improved run-time evaluate performance derived SmVolume classes that 
       can Scale by directly modifying the parameters of the SimpleMap 
       should do so in the virtual method ScaleSimple() so that:

       Orient(ModifiedSimpleMap(uvw)) = Scale(Orient(SimpleMap(uvw))) which is
       ModifiedSimpleMap(uvw)         = InvOrient(Scale(Orient(SimpleMap(uvw)))) 
***********************************************************************/
SmStatus SmVolume::Scale             
 (SmVector3d & crOutScaleVec,     // in : ScaleVec coordinates define xyz scaling factors.
  SmPoint3d  * cpOptOutCenter)    // in : Sole point at which the scaled location equals the input location
                                  //      NULL=(0,0,0), default:[NULL]
{
  SmVolume *pLastMap = GetLastMap() ;

  // save on m_pNextMap linked list length - try to build the Scale into the last NextMap
  if(pLastMap)
    {
      SER(pLastMap->ScaleSimple(crOutScaleVec, cpOptOutCenter)) ; 
    }
  else // build the Scale into this map
    { 
      SER(this->ScaleSimple(crOutScaleVec, cpOptOutCenter)) ; 
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::Scale

/*******************************************************************//**
PURPOSE: Rotate Volume's Outspace Image about an Axis through the OutSpace origin

NOTES: Calls RotateAboutAxisSimple() on the last map of the compound map list.

       RotateAboutAxisSimple() Default Base Class implementation behavior: 
       uses expensive compounding to RotateAboutAxis any volume as
       RotateAboutAxis SmVolume = SmTransform_SetToRotateAboutAxis(Orient(SimpleMap(uvw))) or
       ParamSpace -SimpleMap-> ProjSpace -Orient-> OutSpace -RotateAboutAxis-> FinalOutSpace

       For improved run-time evaluate performance derived SmVolume classes that 
       can RotateAboutAxis by directly modifying the parameters of the SimpleMap 
       should do so in the virtual method RotateAboutAxisSimple() so that:

       Orient(ModifiedSimpleMap(uvw)) = RotateAboutAxis(Orient(SimpleMap(uvw))) which is
       ModifiedSimpleMap(uvw)         = InvOrient(RotateAboutAxis(Orient(SimpleMap(uvw)))) 
***********************************************************************/
SmStatus SmVolume::RotateAboutAxis             
 (double             dAngRad,     // in : Rotation AngRad about an axis that runs through the OutSpace origin
  const SmVector3d & crOutAxis)   // in : Outspace Axis
{
  SmVolume *pLastMap = GetLastMap() ;

  // save on m_pNextMap linked list length - try to build the RotateAboutAxis into the last NextMap
  if(pLastMap)
    {
      SER(pLastMap->RotateAboutAxisSimple(dAngRad, crOutAxis)) ; 
    }
  else // build the RotateAboutAxis into this map
    { 
      SER(this->RotateAboutAxisSimple(dAngRad, crOutAxis)) ; 
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::RotateAboutAxis

/*******************************************************************//**
PURPOSE: Rotate Volume's Outspace Image about an Axis through an OutSpace point

NOTES: Calls RotateAboutAxisAtPointSimple() on the last map of the compound map list.

       RotateAboutAxisAtPointSimple() Default Base Class implementation behavior: 
       uses expensive compounding to RotateAboutAxisAtPoint any volume as
       RotateAboutAxisAtPoint SmVolume = SmTransform_SetToRotateAboutAxisAtPoint(Orient(SimpleMap(uvw))) or
       ParamSpace -SimpleMap-> ProjSpace -Orient-> OutSpace -RotateAboutAxisAtPoint-> FinalOutSpace

       For improved run-time evaluate performance derived SmVolume classes that 
       can RotateAboutAxisAtPoint by directly modifying the parameters of the SimpleMap 
       should do so in the virtual method RotateAboutAxisAtPointSimple() so that:

       Orient(ModifiedSimpleMap(uvw)) = RotateAboutAxisAtPoint(Orient(SimpleMap(uvw))) which is
       ModifiedSimpleMap(uvw)         = InvOrient(RotateAboutAxisAtPoint(Orient(SimpleMap(uvw)))) 
***********************************************************************/
SmStatus SmVolume::RotateAboutAxisAtPoint             
 (double             dAngRad,     // in : Rotation AngRad about an axis that runs through the OutSpace origin
  const SmPoint3d  & crOutOrigin, // in : Outspace point on rotation axis
  const SmVector3d & crOutAxis)   // in : Outspace rotation Axis direction
{
  SmVolume *pLastMap = GetLastMap() ;

  // save on m_pNextMap linked list length - try to build the RotateAboutAxisAtPoint into the last NextMap
  if(pLastMap)
    {
      SER(pLastMap->RotateAboutAxisAtPointSimple(dAngRad, crOutOrigin, crOutAxis)) ; 
    }
  else // build the RotateAboutAxisAtPoint into this map
    { 
      SER(this->RotateAboutAxisAtPointSimple(dAngRad, crOutOrigin, crOutAxis)) ; 
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::RotateAboutAxisAtPoint

/*******************************************************************//**
PURPOSE: Rotate, Move, and Transform Volume's Outspace Image given a 
         general OutSpace to FinalOutSpace Transformation.

NOTES: current Outspace pts will get rotated, moved, and scaled 
       to Next OutSpace pts.
       1. A pure Rotate and Move can be applied by setting 
            cpOptScale = NULL or cpOptScale = (1,1,1) ;
       2. A pure Scaling can be applied by setting crRotateNMove to 
            the identity transformation with a call to crRotateNMove.Init() ; 

       Calls TransformSimple() on the last map of the compound map list.

       TransformSimple() Default Base Class implementation behavior: 
       uses expensive compounding to Transform any volume as
       Transform SmVolume = SmTransform_SetToTransform(Orient(SimpleMap(uvw))) or
       ParamSpace -SimpleMap-> ProjSpace -Orient-> OutSpace -Transform-> FinalOutSpace

       For improved run-time evaluate performance derived SmVolume classes that 
       can Transform by directly modifying the parameters of the SimpleMap 
       should do so in the virtual method TransformSimple() so that:

       Orient(ModifiedSimpleMap(uvw)) = Transform(Orient(SimpleMap(uvw))) which is
       ModifiedSimpleMap(uvw)         = InvOrient(Transform(Orient(SimpleMap(uvw)))) 
***********************************************************************/
SmStatus SmVolume::Transform             
 (const SmAxis2Placement & crOutRotateNMove,  // in : Map Pts from current OutSpace to the next OutSpace. i.e. a (0,0,0) 
                                              //      OutSpacePoint projects to the RotatNMove NextOutSpace origin.
  const SmVector3d       * cpOptOutScale)     // in : Opt OutSpace Origin Scaling done after RotateNMove mapping.
                                              //      NULL to ignore. default:[NULL]
{
  SmVector3d sIdentityScale(1, 1, 1);

  // no work - identity transform
  if (crOutRotateNMove.IsIdentity() && (cpOptOutScale == NULL || *cpOptOutScale == sIdentityScale))
  {
      return SM_SUCCESS;
  }

  SmVolume *pLastMap = GetLastMap() ;

  // save on m_pNextMap linked list length - try to build the Transform into the last NextMap
  if(pLastMap)
    {
      SER(pLastMap->TransformSimple(crOutRotateNMove, cpOptOutScale)) ; 
    }
  else // build the Transform into this map
    { 
      SER(this->TransformSimple(crOutRotateNMove, cpOptOutScale)) ; 
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::Transform

/*******************************************************************//**
PURPOSE: return TRUE when any of the compounded mappings are bounded

NOTES: 
***********************************************************************/
SmBoolean SmVolume::IsBounded() const           
{
  // all done
  return(   (IsBoundedSimple()) 
         || (m_pNextMap ? m_pNextMap->IsBounded() : FALSE)) ;

} // end SmVolume::IsBounded    

/*******************************************************************//**
PURPOSE: return TRUE when any of the compounded mappings 
         have an internal C1 discontinuity

NOTES: Call HasDiscontinutiesSimple if you want to know about this
       mapping without compounding.
***********************************************************************/
SmBoolean SmVolume::HasDiscontinuities   // rtn: TRUE if Simple map has internal discontinuities 
 (SmDiscontinuities3d * pOptDisconts,    // out: optional ParamSpace discontinuties summary, NULL to ignore, default:[NULL]
  SmBoolean             bCalcGeometric)  // in : TRUE = expensive - use geometric testing to compute actal geometric discontinuities
                                         //      FALSE= cheap - report representational discontinuites
                                         //      default:[TRUE]
 const
{
  // Pass the call along to the simple map
  SmBoolean bThisHasDisconts = HasDiscontinuitiesSimple(pOptDisconts, bCalcGeometric) ;

  // when compounding 
  if(m_pNextMap) 
    { 
      SmBoolean  bNextHasDisconts ;

      // Get the NextMap sequence of discontinuties
      if(pOptDisconts)
        {
          SmDiscontinuities3d  sNextDisconts ;
          bNextHasDisconts = m_pNextMap->HasDiscontinuities(&sNextDisconts, bCalcGeometric) ;

          // when NextMap has no discontinuities - output equals ThisMap discontinuities
          // when NextMap has discontinuities - alignment is problematic.  Generally the UVW param planes 
          //                                    will not be aligned with basis vectors of a sequence of maps.
          //                                    So, just set the output to assume unaligned discontinuties.

          if(bNextHasDisconts)
            {
              SmContinuityType eMinCont = smos_Min(pOptDisconts->m_eMinContinuity, sNextDisconts.m_eMinContinuity) ;
              pOptDisconts->SetUnaligned() ;
              pOptDisconts->m_bHasDiscontinuities |= bNextHasDisconts ;
              pOptDisconts->m_eMinContinuity = eMinCont ;
            }

          // combine results
          bThisHasDisconts |= bNextHasDisconts ;
          pOptDisconts->m_bHasDiscontinuities = bThisHasDisconts ; 

        } // end pOptDisconts != NULL branch
      else // not asked to output details, pOptDisconts == NULL
        {
          bNextHasDisconts = m_pNextMap->HasDiscontinuities(NULL, FALSE) ;

          // combine results
          bThisHasDisconts |= bNextHasDisconts ;

        } // end pOptDisconts == NULL branch

    } // end compounding check

  // all done
  return(bThisHasDisconts) ;

} // end SmVolume::HasDiscontinuities    

/*******************************************************************//**
PURPOSE: Determine if a volume is closed in OutSpace along the U, V, and W directions
         and optionally report the max level of continuity found in each direction.

NOTES: 
  1. This algorithm is only worked out for Box domains. Derived SmVolume classes
     with non axis aligned box domains should do something different with a virtual method.
  
  2. A volume is closed in a direction when the associated direction's Min and Max
  isoSurfaces are coincident.  This is tested only by point sampling.

  eVolParam specifies which boundaries of the volume to test as
  SM_VP_U = check Pos(Umin,v,w) == Pos(Umax,v,w) at several v,w sample points
  SM_VP_V = check Pos(u,Vmin,w) == Pos(u,Vmax,w) at several u,w sample points
  SM_VP_W = check Pos(u,v,Wmin) == Pos(u,w,Vmax) at several u,v sample points

  peOptContinuity is set with the highlest level of continuity found at the sample points.
  SM_VP_U = check U direction derivatives at the ends of variable U isoparameter lines
  IS_VP_V = check V direction derivatives at the ends of variable V isoparameter lines
  IS_VP_W = check W direction derivatives at the ends of variable W isoparameter lines
  Set peOptContinuity == NULL to ignore.

  Currently continuity is only checked to C2/G2
***********************************************************************/
SmBoolean SmVolume::IsClosed
  (SmBoolean        & rbClosedU,          // out: TRUE = closed in U direction, [check Pos[Umin,v,w] == Pos[Umax,v,w] for v,w samples
   SmBoolean        & rbClosedV,          // out: TRUE = closed in V direction, [check Pos[u,Vmin,w] == Pos[u,Vmax,w] for w,u samples
   SmBoolean        & rbClosedW,          // out: TRUE = closed in W direction, [check Pos[u,v,Wmin] == Pos[u,v,Wmax] for u,v samples
   double           * pdOptTolerance,     // out: pdOptTolerance = Tolerance to allow for check
   const SmExtent3d * pOptParamDomain,    // in : ParamSpace domain over which to search for the inverse point
                                          //      NULL = use Map's NaturalDomain, default:[NULL]
   SmContinuityType * peOptContinuityU,   // out: U dir Continuity when closed 
   SmContinuityType * peOptContinuityV,   // out: V dir Continuity when closed      
   SmContinuityType * peOptContinuityW)   // out: W dir Continuity when closed
                                          //      oneof SM_CT_DISCONTINUOUS      
                                          //            SM_CT_C0                 
                                          //            SM_CT_G1                 
                                          //            SM_CT_G1_G2              
                                          //            SM_CT_G1_G2_G3              
                                          //            SM_CT_C1                 
                                          //            SM_CT_C1_G2        
                                          //            SM_CT_C1_G2_G3        
                                          //            SM_CT_C1_C2        
                                          //            SM_CT_C1_G3        
                                          //            SM_CT_C1_C3        
 const
{
#define NUM_TEST_POINTS 5
  // init output
  rbClosedU = TRUE ;
  rbClosedV = TRUE ;
  rbClosedW = TRUE ;
  if(peOptContinuityU) { *peOptContinuityU = SM_CT_CINFINITY ; }
  if(peOptContinuityV) { *peOptContinuityV = SM_CT_CINFINITY ; }
  if(peOptContinuityW) { *peOptContinuityW = SM_CT_CINFINITY ; }

  // locals
  ULONG dd, ii, jj ;
  ULONG lCount = 3 ;
  SmBoolean bDone = FALSE;
  //SmContinuityType eMinContinuity = SM_CT_CINFINITY ;
  SmPoint3d  sPMin, sPMax, sPMid[3], sGMid[4] ;
  SmPoint3d  sUVWEval ;
  SmPoint3d  sUVWMin, sUVWMax, sUVWMid, sParamDir ;
  SmVector3d sDUMin (0,0,0), sDUMax (0,0,0) ;
  SmVector3d sDVMin (0,0,0), sDVMax (0,0,0) ;
  SmVector3d sDWMin (0,0,0), sDWMax (0,0,0) ;
  SmVector3d sDUUMin(0,0,0), sDUUMax(0,0,0) ;
  SmVector3d sDVVMin(0,0,0), sDVVMax(0,0,0) ;
  SmVector3d sDWWMin(0,0,0), sDWWMax(0,0,0) ;
  SmVector3d sDUVMin(0,0,0), sDUVMax(0,0,0) ;
  SmVector3d sDUWMin(0,0,0), sDUWMax(0,0,0) ;
  SmVector3d sDVWMin(0,0,0), sDVWMax(0,0,0) ;
  SmExtent3d sNaturalParamDomain ;
  if(pOptParamDomain) { sNaturalParamDomain = *pOptParamDomain ; }
  else                { sNaturalParamDomain = GetNaturalParamDomain() ; }

  // Approximate unbounded domains - clip unbounded dimensions
  sNaturalParamDomain = sNaturalParamDomain.ApproximateUnbounded() ;

  // for every param direction to be checked
  for(dd=0; dd<lCount; dd++, bDone=FALSE)
    {
      // param direction
      SmVolumeParamType eThisParam =   (dd == 0) ? SM_VP_U
                                     : (dd == 1) ? SM_VP_V
                                     :             SM_VP_W ;
      if     (eThisParam == SM_VP_U) { sParamDir.Set(1.0, 0.0, 0.0) ; }
      else if(eThisParam == SM_VP_V) { sParamDir.Set(0.0, 1.0, 0.0) ; }
      else                           { sParamDir.Set(0.0, 0.0, 1.0) ; }
      SmContinuityType *pContinuity =  (dd == 0) ? peOptContinuityU
                                     : (dd == 1) ? peOptContinuityV
                                     :             peOptContinuityW ;

      // for every sample point
      for (ii=0; ii<=NUM_TEST_POINTS && !bDone; ii++)
        {
          double dParamI = (ii+1.0)/(2.0+NUM_TEST_POINTS) ;

          // for every sample point
          for (jj=0; jj<=NUM_TEST_POINTS && !bDone; jj++)
            {
              double dParamJ = (jj+1.0)/(2.0+NUM_TEST_POINTS) ;

              // get next UVWPoint on UVWDomain - avoid endPoints
              sUVWEval.Set(dParamI, ii<2 ? dParamI : dParamJ, dParamJ) ;
              SmPoint3d sUVW = sNaturalParamDomain.Evaluate( sUVWEval.x, sUVWEval.y, sUVWEval.y );

              // set sUVWMin/sUVWMax to be requested isoParameters lines endPoints
              if     (eThisParam == SM_VP_U) { // testing U parameter
                                               sUVWMin.x = sNaturalParamDomain.GetMin().x ;
                                               sUVWMax.x = sNaturalParamDomain.GetMax().x ;
                                               sUVWMid.x = (sUVWMin.x + sUVWMax.x) / 2.0 ;
                                               sUVWMin.y = sUVWMax.y = sUVWMid.y = sUVW.y ;
                                               sUVWMin.z = sUVWMax.z = sUVWMid.z = sUVW.z ;
                                             }
              else if(eThisParam == SM_VP_V) { // testing V parameter
                                               sUVWMin.y = sNaturalParamDomain.GetMin().y ;
                                               sUVWMax.y = sNaturalParamDomain.GetMax().y ;
                                               sUVWMid.y = (sUVWMin.y + sUVWMax.y) / 2.0 ;
                                               sUVWMin.x = sUVWMax.x = sUVWMid.x = sUVW.x ;
                                               sUVWMin.z = sUVWMax.z = sUVWMid.z = sUVW.z ;
                                             }
              else                           { // testing W parameter
                                               sUVWMin.z = sNaturalParamDomain.GetMin().z ;
                                               sUVWMax.z = sNaturalParamDomain.GetMax().z ;
                                               sUVWMid.z = (sUVWMin.z + sUVWMax.z) / 2.0 ;
                                               sUVWMin.x = sUVWMax.x = sUVWMid.x = sUVW.x ;
                                               sUVWMin.y = sUVWMax.y = sUVWMid.y = sUVW.y ;
                                             }

              // evaluate isoParameter endPoints
              if(   (dd == 0 && peOptContinuityU != NULL)
                 || (dd == 1 && peOptContinuityV != NULL)  
                 || (dd == 1 && peOptContinuityW != NULL) )
                {
                  Evaluate2ndDerivatives( sUVWMin, TRUE, TRUE, TRUE,
                                          sPMin, sDUMin, sDVMin, sDWMin,
                                          sDUUMin, sDVVMin, sDWWMin, sDUVMin, sDUWMin, sDVWMin );
                  Evaluate2ndDerivatives( sUVWMax, TRUE, TRUE, TRUE,
                                          sPMax, sDUMax, sDVMax, sDWMax,
                                          sDUUMax, sDVVMax, sDWWMax, sDUVMax, sDUWMax, sDVWMax );
                }
              else
                {
                  EvaluatePoint( sUVWMin, sPMin );
                  EvaluatePoint( sUVWMax, sPMax );
                }

#ifdef SM_DEBUG_CODE
#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE ;
              // draw EndPoints(red)
              if(bDebugMe)
                {
                  if(dd == 0 && ii == 0 && jj == 0)
                    { smgfx_Erase() ; 
                      smgfx_SetLook(1,2, 0,0,1) ; Draw(TRUE, TRUE) ; sm_GraphicsLoop() ;
                    }
                  smgfx_SetLook(3,4, 1,0,0) ; sPMin.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(3,6, 0,1,0) ; sPMax.Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_GFX_CODE
#endif // SM_DEBUG_CODE

              // pick a tolerance
              double dScaledZero = SM_EFF_ZERO * 1000.0 * (1.0 + sPMax.GetMaxDimension() + sPMin.GetMaxDimension());
              double dTol        =   (pdOptTolerance)
                                   ? *pdOptTolerance
                                   : dScaledZero;

              // get the startPoint/endPoint gap
              SmVector3d sVecDiff = sPMin - sPMax ;

              // when the gap is too big - not closed
              if ( sVecDiff.LengthSquared() > dTol*dTol )
                {
                  if     (dd == 0) { rbClosedU = FALSE ; 
                                     if(peOptContinuityU) { *peOptContinuityU = SM_CT_DISCONTINUOUS ; }
                                   }
                  else if(dd == 1) { rbClosedV = FALSE ;  
                                     if(peOptContinuityV) { *peOptContinuityV = SM_CT_DISCONTINUOUS ; }
                                   }
                  else             { rbClosedW = FALSE ;  
                                     if(peOptContinuityW) { *peOptContinuityW = SM_CT_DISCONTINUOUS ; }
                                   }
                  bDone = TRUE ;
                  break ;
                }
              else if(pContinuity != NULL)
                {
                  SmContinuityType eCurveContinuity ;
                  if     (dd == 0) 
                    { smgu_EvaluateCurveContinuity(sPMin, sDUMin, sDUUMin,
                                                   sPMax, sDUMax, sDUUMax,
                                                   eCurveContinuity) ;
                    }
                  else if(dd == 1)
                    { smgu_EvaluateCurveContinuity(sPMin, sDVMin, sDVVMin,
                                                   sPMax, sDVMax, sDVVMax,
                                                   eCurveContinuity) ;
                    }
                  else
                    { smgu_EvaluateCurveContinuity(sPMin, sDWMin, sDWWMin,
                                                   sPMax, sDWMax, sDWWMax,
                                                   eCurveContinuity) ;
                    }

                  // save lowest found continuity
                  if(eCurveContinuity < *pContinuity) { *pContinuity = eCurveContinuity ; }
                }

              // gwc: extra test
              if (*pContinuity > SM_CT_DISCONTINUOUS)
                {

                  // filter out very short curve passing as closed curves
                  EvaluateDirectionalDerivs( sUVWMid, sParamDir, 2, sPMid );

                  SmVector3d sGapDiff1 = sPMid[0] - sPMin ;
                  SmVector3d sGapDiff2 = sPMid[0] - sPMax ;
                  double     dGap1Sq   = sGapDiff1.LengthSquared() ;
                  double     dGap2Sq   = sGapDiff2.LengthSquared() ;
                  double     dMidGapSq = smos_Max(dGap1Sq, dGap2Sq) ;

                  // cheesy one point test for a short curve
                  if (dMidGapSq < 4.0*dTol*dTol)
                    {
                      // but some short curves can be very small circles - watch for those

                      // get geometric properties from directional derivatives
                      smgu_FrenetFromDerivatives(sPMid, 3, sGMid) ;

                      // get curvature and radius of curvature
                      double dMidCurvSq = sGMid[2].LengthSquared() ;
                      double dMidRadius = (dMidCurvSq > SM_EFF_ZERO_SQ) ? 1.0 / smos_Sqrt(dMidCurvSq) : SM_BIG_DOUBLE ;

                      // When the radius is larger (small circles can be closed - but short large radius arcs can't be
                      if (dMidRadius > dTol)
                        {
                          if ( pContinuity != NULL )
                            { *pContinuity = SM_CT_DISCONTINUOUS ; }

                          return FALSE;
                        } // end curve not a small circle check
                    } // end curve estimated to be a short arc check
                } // end still checking for C0 continuity check
            } // end iter jj, every sample point
        } // end iter ii, every sample point
    } // end iter dd, every param direction - checking for closure

  // when all test endPoint gaps pass tolerance test - surface is closed
  return (   rbClosedU
          || rbClosedV
          || rbClosedW) ;

#undef NUM_TEST_POINTS

} // end SmVolume::IsClosed

/*******************************************************************//**
PURPOSE: Determine if a simple volume is periodic with G1 continuity or higher

NOTES: 
***********************************************************************/
SmBoolean SmVolume::IsPeriodic
 (SmBoolean        & rbPeriodicU,      // out: TRUE = G1 or better in U Dir 
  SmBoolean        & rbPeriodicV,      // out: TRUE = G1 or better in V Dir
  SmBoolean        & rbPeriodicW,      // out: TRUE = G1 or better in W Dir
  const SmExtent3d * pOptParamDomain)  // in : ParamSpace domain over which to search for the inverse point
                                       //      NULL = use Map's NaturalDomain, default:[NULL]
 const
{
  // locals
  SmBoolean        bClosedU,     bClosedV,     bClosedW ;          
  SmContinuityType eContinuityU, eContinuityV, eContinuityW ;

  // pass the call along
  IsClosed(bClosedU, bClosedV, bClosedW,
           NULL, pOptParamDomain,
           &eContinuityU, &eContinuityV, &eContinuityW) ;

  // set output
  rbPeriodicU = (eContinuityU >= SM_CT_G1) ;
  rbPeriodicV = (eContinuityV >= SM_CT_G1) ;
  rbPeriodicW = (eContinuityW >= SM_CT_G1) ;

  // all done
  return (   rbPeriodicU
          || rbPeriodicV
          || rbPeriodicW) ;

} // end SmVolume::IsPeriodic

/*******************************************************************//**
PURPOSE: Determine if a ParamPoint is a singluar point in OutSpace
         and get the singular direction(s).

NOTES: 
    Return: TRUE    = dW/du, dW/dv, or dW/dw (1st partial derivative in U, V, or W directions)
                      is zero.
            FALSE   = All 1st partial derivatives are non-zero.
    Output: rbSingularU: TRUE = U direction derivative is  zero 
                         FALSE= U direction derivative NOT zero.
            rbSingularV: TRUE = V direction derivative is  zero 
                         FALSE= V direction derivative NOT zero.
            rbSingularW: TRUE = W direction derivative is  zero 
                         FALSE= W direction derivative NOT zero.

    note: The cases where a point is degenerate in two or more directions 
          are not distinguished; TRUE is returned and
          reSingularDirection is set to one of the degenerate directions.
***********************************************************************/
SmBoolean SmVolume::IsSingularity
  (const SmPoint3d & crParamPoint,    // in : Volume ParamSpace Point to test
   SmBoolean       & rbSingularU,     // out: TRUE = Singular [Wu=0] in U parameter direction
   SmBoolean       & rbSingularV,     // out: TRUE = Singular [Wv=0] in V parameter direction
   SmBoolean       & rbSingularW,     // out: TRUE = Singular [Ww=0] in W parameter direction
   double            d3dTol)          // NotUsed: in : min dist between distinct 3d points
 const
{
  SM_REF1(d3dTol) ;
  // init output
  rbSingularU = FALSE ;
  rbSingularV = FALSE ;
  rbSingularW = FALSE ;

  // get first derivatives from ParamSpace to OutSpace (with compounding)
  SmVector3d sMat[2][2][2];
  SER(Evaluate(crParamPoint,1,TRUE,TRUE,TRUE,sMat[0][0]));

  double dLengSqDU = sMat[1][0][0].LengthSquared();
  double dLengSqDV = sMat[0][1][0].LengthSquared();
  double dLengSqDW = sMat[0][0][1].LengthSquared();

  // when any 1st derivative is small - do further classification
  if(   dLengSqDU < SM_EFF_ZERO_SQRT
     || dLengSqDV < SM_EFF_ZERO_SQRT
     || dLengSqDW < SM_EFF_ZERO_SQRT) 
    {
      // Do further tests to qualify
      double dLengDU = smos_Sqrt(dLengSqDU);
      double dLengDV = smos_Sqrt(dLengSqDV);
      double dLengDW = smos_Sqrt(dLengSqDW);

      // check for UDir Singularity
      if(    dLengDU < dLengDV * SM_EFF_ZERO
          || dLengDU < dLengDW * SM_EFF_ZERO
          || dLengDU < SM_EFF_ZERO * 100.0) 
        {
          rbSingularU = TRUE ;
        }

      // check for VDir Singularity
      if(    dLengDV < dLengDU * SM_EFF_ZERO
          || dLengDV < dLengDW * SM_EFF_ZERO
          || dLengDV < SM_EFF_ZERO * 100.0) 
        {
          rbSingularV = TRUE ;
        }

      // check for WDir Singularity
      if(    dLengDW < dLengDU * SM_EFF_ZERO
          || dLengDW < dLengDV * SM_EFF_ZERO
          || dLengDW < SM_EFF_ZERO * 100.0) 
        {
          rbSingularW = TRUE ;
        }
    }

  // all done
  return (rbSingularU || rbSingularV || rbSingularW) ;

} // end SmVolume::IsSingularity

/*******************************************************************//**
PURPOSE: return TRUE when ParamPoint is on ParamDomain boundary or
         maps to a point on any subsequent compounded object's natural boundary

NOTES: 
***********************************************************************/
SmBoolean SmVolume::IsOnBoundary       
 (const SmPoint3d   & crParamPoint,     // in : 
  SmBoolean         & rbOnU,            // out: 
  SmBoolean         & rbOnV,            // out: 
  SmBoolean         & rbOnW,            // out: 
  double            * pdOptTolerance,   // in : default:[NULL]
  const SmExtent3d * pOptParamDomain)   // in : ParamSpace domain to limit search
                                        //      NULL = use Map's NaturalDomain, default:[NULL]
 const  
{
  // Simple Maps
  if(m_pNextMap == NULL)
    {
      // pass call along
      return( IsOnBoundarySimple( crParamPoint, rbOnU, rbOnV, rbOnW, pdOptTolerance, pOptParamDomain)) ;
    }
  else // Compound Maps
    {
      // locals
      SmBoolean bThisOnU, bNextOnU ;
      SmBoolean bThisOnV, bNextOnV ;
      SmBoolean bThisOnW, bNextOnW ;
      SmPoint3d sMidPoint ;

      // Get Mid Point in ParamSpace of NextMap
      EvaluatePointSimple(crParamPoint, sMidPoint) ;
      if(m_pNextMap->GetInvOrientMap())
        {
          m_pNextMap->GetInvOrientMap()->EvaluatePointSimple(sMidPoint, sMidPoint) ;
        } 

      // map classifications
      IsOnBoundarySimple( crParamPoint, bThisOnU, bThisOnV, bThisOnW, pdOptTolerance) ;
      m_pNextMap->IsOnBoundarySimple( sMidPoint,   
                                      bNextOnU, bNextOnV, bNextOnW, 
                                      pdOptTolerance ) ;

      // set output
      rbOnU = bThisOnU || bNextOnU ;
      rbOnV = bThisOnV || bNextOnV ;
      rbOnW = bThisOnW || bNextOnW ;

      // all done
      return(rbOnU && rbOnV && rbOnW) ;
    } // end compound branch

} // end SmVolume::IsOnBoundary    

/*******************************************************************//**
PURPOSE: Receive notification of things happening to the object and
    take appropriate actions.  

NOTES: For example cleaning up the cache of an
    object which is being deleted or edited.  Also clean up any attributes
    and relations specific to this class which are not handled automatically
    by construtors.
***********************************************************************/
void SmVolume::Notify                  // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
 (SmNotifyOperation  eNotifyOperation, //       event                | caller      |  pData1  | pData2                | pData3                   
  SmObject         * pData1,           //----------------------------+-------------+----------+-----------------------+--------------------------
  SmObject         * pData2,           // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL             
  SmObject         * pData3)           // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2
                                       // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep
                                       // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                       // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
                                       // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL
                                       // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL
                                       // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL
                                       // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL
                                       // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL
                                       // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL
                                       // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL
                                       // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL
                                       // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL
                                       // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   | NULL                  | NULL 
{
  switch (eNotifyOperation)
    {
      case SM_NO_ADD_TO_BREP           : break ;
      case SM_NO_SPLIT_IN_BREP         : break ;
      case SM_NO_MERGE_IN_BREP         : break ;
      case SM_NO_TRIM_NO_SPLIT_IN_BREP : break ;
      case SM_NO_RM_FROM_BREP          : break ;
      case SM_NO_CHANGE_GEOMETRY       : break ;
      case SM_NO_CHANGE_OWNER          : break ;
      case SM_NO_CONSTRUCTION          : break ;
      case SM_NO_COPY                  : break ;
      case SM_NO_PRE_EDIT              : break ;
      case SM_NO_POST_EDIT             : break ;
      case SM_NO_SPLIT                 : break ;
      case SM_NO_MERGE                 : break ;
      case SM_NO_REG_PROPAGATION       : break ;
      case SM_NO_DESTRUCTION           : break ;
      case SM_NO_COINCIDENT            : break;
      case SM_NO_UNKNOWN:                { SE_MSG(SM_ERR, _T("SmVolume::Notify - SM_NO_UNKNOWN event signalled")) ; } 
                                         break ;
    }

  // Propagate notification up hierarchy
  SmAObject::Notify(eNotifyOperation,pData1,pData2,pData3);

} // end SmVolume::Notify

/*******************************************************************//**
PURPOSE: If the given ParamSpace Point is near one
   or more discontinuity values on the volume, snap it to the nearest dicontinuity.

NOTES:
   This snaps the U, V, and W knot values independently 
   so the possible outcomes include:
   1. not snapped at all
   2. snapped in U direction only
   3. snapped in V direction only
   4. snapped in W direction only
   5. snapped in U and V directions
   6. snapped in U and W directions
   7. snapped in V and W directions
   8. snapped in U, V and W directions
***********************************************************************/
SmStatus SmVolume::SnapToDiscontinuity
  (const SmPoint3d & crParamPoint,        // in : 
   double            dParamSnapTol,       // in : 
   SmPoint3d       & rSnappedParamPoint)  // out: 
  const
{
  // init output
  rSnappedParamPoint = crParamPoint;

  // locals
  ULONG ii, jj, kk ;
  SmContinuityType eMinContinuity ;
  double adParamData[256] ;
  SmContinuityType  aeContsData[256] ;
  SmTArray<double> sParams(256,adParamData);
  SmTArray<SmContinuityType>  sConts(256,aeContsData);

  // Snap U direction to closest discontinuity within tolerance
  SER(CalculateContinuitiesSimple(SM_VP_U, eMinContinuity, sParams, sConts, FALSE));

  double dMinSnapDist = SM_BIG_DOUBLE;
  for (ii=0; ii<sParams.GetSize(); ii++) 
    {
      double dSnapDist = smos_Fabs(sParams[ii]-crParamPoint.x);
      if(   dSnapDist < dMinSnapDist 
         && dSnapDist < dParamSnapTol) 
        {
          rSnappedParamPoint.x = sParams[ii];
          dMinSnapDist         = dSnapDist;
        }
    }

  // Snap V direction to closest discontinuity within tolerance
  SER(CalculateContinuitiesSimple(SM_VP_V, eMinContinuity, sParams, sConts, FALSE));

  dMinSnapDist = SM_BIG_DOUBLE;
  for (jj=0; jj<sParams.GetSize(); jj++) 
    {
      double dSnapDist = smos_Fabs(sParams[jj]-crParamPoint.y);
      if(   dSnapDist < dMinSnapDist 
         && dSnapDist < dParamSnapTol) 
        {
          rSnappedParamPoint.y = sParams[jj];
          dMinSnapDist           = dSnapDist;
        }
    }

  // Snap W direction to closest discontinuity within tolerance
  SER(CalculateContinuitiesSimple(SM_VP_W, eMinContinuity, sParams, sConts, FALSE));

  dMinSnapDist = SM_BIG_DOUBLE;
  for (kk=0; kk<sParams.GetSize(); kk++) 
    {
      double dSnapDist = smos_Fabs(sParams[kk]-crParamPoint.z);
      if(   dSnapDist < dMinSnapDist 
         && dSnapDist < dParamSnapTol) 
        {
          rSnappedParamPoint.z = sParams[kk];
          dMinSnapDist           = dSnapDist;
        }
    }

  // all done
  return SM_SUCCESS ;

} // end SmVolume::SnapToDiscontinuity

/*******************************************************************//**
PURPOSE: Compute the ProjSpace Bounding and/or Pseudo Boxes that 
   contain the projection of a given BBox from ParamSpace to ProjSpace.

NOTES: This method is the default base behavior for simple EvaluateBoundingBox
  1. If Given, maps pOptParamPseudoBox basis vectors to ProjectSpace and uses
       those to set the orientation of pProjSpacePseudoBox.
  2. When given crParamBox
       is bounded  : Adds the ProjSpace mapped CornerPoints, Edge SamplePoints,
                     and Face SamplePoints of the ParamSpace box 
                     to the output boxes, pProjBox and pProjPseudoBox.

                     The output ProjSpace boxes might be undersized since 
                     the size of the ouput ProjSpace boxes are
                     based on sampling of the input ParamSpace boxes. 
                     If this is ever a problem, either try increasing the
                     sampling density, or replacing the algorithm by 
                     a newton-raphson based approach that finds Edge and Face
                     maxima in ProjectSpace.
       is unbounded:
***********************************************************************/
SmStatus SmVolume::EvaluateBoundingBoxSimple
  (const SmExtent3d & crParamBox,               // in : ParamSpace BBox to project to Project Space 
   SmPseudoBox      * pOptParamPseudoBox,       // in : optional ParamSpace PseudoBox used to set output PseudoBox orientations,
                                                //      NULL   : ProjPseudoBox Basis = Project X Y Z ParamVecs to ProjSpace at crParamBox Center
                                                //      NotNULL: ProjPseudoBox Basis = Project pOptParamPseudoBox BasisVecs to ProjSpace at crParamBox Center
                                                //      default:[NULL]
   SmExtent3d       * pProjBox,                 // out: ProjectSpace Axis aligned box
   SmPseudoBox      * pProjPseudoBox)           // out: ProjectSpace Non-axis aligned box
  const
{
  // local
  ULONG lSmpCnt = 5 ;  // Each of 6 BBox faces are sampled dSmpCnt x dSmpCnt times: dSmpCnt5 => 150 samples
                       // Plus 12 BBox edges sampled dSmpCnt times: dSmpCnt 5 => 60 samples
                       // Plus 8  BBox corners sampled once       : 8 samples
                       // dSmpCnt Total = 150 + 60 + 8 = 218 samples

  // no work - no output
  if(   pProjBox == NULL
     && pProjPseudoBox == NULL)
    { return SM_SUCCESS ; }

  // when input ParamBox is same as output ProjBox - copy input ParamBox
  SmExtent3d        sParamBox ;
  const SmExtent3d *pParamBox ;
  if(pProjBox && (&crParamBox == pProjBox)) {  sParamBox = crParamBox ;
                                               pParamBox = &sParamBox ;
                                            }
  else                                      {  pParamBox = &crParamBox ;
                                            }

  // when input ParamPseudoBox is same as output ProjPseudoBox - copy input ParamPseudoBox
  SmPseudoBox sParamPseudoBox, *pParamPseudoBox ;
  if(   pProjPseudoBox
     && pOptParamPseudoBox == pProjPseudoBox) { sParamPseudoBox = *pOptParamPseudoBox ;
                                                pParamPseudoBox = &sParamPseudoBox ;
                                              }
  else                                        { pParamPseudoBox = pOptParamPseudoBox ;
                                              }

  // indirection: from here on out - pParamBox       == ParamSpace Box
  //                               - pParamPseudoBox == ParamSpace PseudoBox

  // locals
  SmPoint3d  sProjPoint ;

  // init ProjSpaceBox
  if(pProjBox) { pProjBox->Init() ; }

  // When given - Init pProjPseudoBox intervals and set orientation basis vectors
  if(pProjPseudoBox)
    {
      SmVector3d sBX(1,0,0) ;
      SmVector3d sBY(0,1,0) ;
      SmVector3d sBZ(0,0,1) ;
      SmVector3d sDX, sDY, sDZ ;
      SmVector3d sO = crParamBox.Evaluate(.5,.5,.5) ;

      // map ParamSpace basis vectors to ProjectSpace                    
      Evaluate1stDerivatives(sO, TRUE, TRUE, TRUE, sProjPoint, sDX, sDY, sDZ, TRUE) ;

      // When asked, switch vecs to be project to OptParamPseudoBox bases
      if(pOptParamPseudoBox)
        {
          // Extract the projected PseudoBox basis directions
          sBX = pParamPseudoBox->GetBasis(0) ;
          sBY = pParamPseudoBox->GetBasis(1) ;
          sBZ = pParamPseudoBox->GetBasis(2) ;
        }

      SmVector3d sProjDX = sBX.x * sDX + sBX.y * sDY + sBX.z * sDZ ;
      SmVector3d sProjDY = sBY.x * sDX + sBY.y * sDY + sBY.z * sDZ ;
      SmVector3d sProjDZ = sBZ.x * sDX + sBZ.y * sDY + sBZ.z * sDZ ;

      // unitize the projected directions
      sProjDX.Unitize() ;                                                
      sProjDY.Unitize() ;                                                
      sProjDZ.Unitize() ;  

      // orient the ProjectSpace PseudoBox to the projection of the ParamSpace PseudoBox orientation                                         
      pProjPseudoBox->SetBasis(sProjDX, sProjDY, sProjDZ) ; // basis set to    :[sProjDX, sProjDY, sProjDZ]
      pProjPseudoBox->Init() ;                              // intervals set to:[SM_BIG_DOUBLE, -SM_BIG_DOUBLE]

    } // end setting pProjPseudoBox orientation branch

  // When given ParamSpace BoundingBox is bounded
  if(pParamBox->IsBounded())
    {
      // get ordered corners
      ULONG ii,jj,kk ;
      ULONG i0 = 0, i1 = 0, j0 = 0, j1 = 0;
      double     dParami, dParamj, dParamInc = 1.0 / (lSmpCnt + 1.0) ;
      SmPoint3d  sParamPoint, sParamPtI, sParamPtJ ; 
      SmTArray<SmPoint3d> sParamCorners(8,NULL,8) ;
      pParamBox->GetCorners(sParamCorners) ; // ordered:{ 000 010 100 110 001 011 101 111 }
      SM_ASSERT(sParamCorners.GetSize() == 8) ; 

      // corners
      for(ii=0;ii<8;ii++)
        {
          // map the corners to ProjectSpace
          EvaluatePoint(sParamCorners[ii], sProjPoint) ; 

          // Add point to bounding boxes
          if(pProjBox)       { pProjBox->AddPoint3d(sProjPoint) ; }
          if(pProjPseudoBox) { pProjPseudoBox->AddPoint3d(sProjPoint) ; }
        } // end iter kk

      // Edges
      for(ii=0;ii<12;ii++)
        { 
          // ordered:{ 000 010 100 110 001 011 101 111 }
          switch(ii) { case  0 : i0 = 0 ; i1 = 1 ; break ; // 000 010  z=0 plane edges
                       case  1 : i0 = 1 ; i1 = 2 ; break ; // 010 100
                       case  2 : i0 = 2 ; i1 = 3 ; break ; // 100 110
                       case  3 : i0 = 3 ; i1 = 0 ; break ; // 110 000 
                       case  4 : i0 = 4 ; i1 = 5 ; break ; // 001 011  z=1 plane edges
                       case  5 : i0 = 5 ; i1 = 6 ; break ; // 011 101
                       case  6 : i0 = 6 ; i1 = 7 ; break ; // 101 111
                       case  7 : i0 = 7 ; i1 = 4 ; break ; // 111 001 
                       case  8 : i0 = 0 ; i1 = 4 ; break ; // 000 001  edges from z=0 to z=1
                       case  9 : i0 = 1 ; i1 = 5 ; break ; // 010 011
                       case 10 : i0 = 2 ; i1 = 6 ; break ; // 100 101
                       case 11 : i0 = 3 ; i1 = 7 ; break ; // 110 111
                     }
          for(jj=0,dParami=dParamInc;jj<lSmpCnt;jj++, dParami+=dParamInc)
            {
              // map edge samples to ProjectSpace
              sParamPoint = dParami * sParamCorners[i0] + (1.0-dParami) * sParamCorners[i1] ;
              EvaluatePoint(sParamPoint, sProjPoint) ; 

              // Add point to bounding boxes
              if(pProjBox)       { pProjBox->AddPoint3d(sProjPoint) ; }
              if(pProjPseudoBox) { pProjPseudoBox->AddPoint3d(sProjPoint) ; }
            } // end iter jj, samples on the edge
        } // end iter ii, edges

      // faces
      for(ii=0;ii<6;ii++)
        {
          // ordered:{ 000 010 100 110 001 011 101 111 }
          //            0   1   2   3   4   5   6   7
          switch(ii) { case  0 : i0 = 0 ; i1 = 1 ; j0 = 2 ; j1 = 3 ; break ; // 000 010 100 110 z=0 plane
                       case  1 : i0 = 4 ; i1 = 5 ; j0 = 6 ; j1 = 7 ; break ; // 001 011 101 111 z=1 plane
                       case  2 : i0 = 0 ; i1 = 4 ; j0 = 1 ; j1 = 5 ; break ; // 000 001 010 011 x=0 plane
                       case  3 : i0 = 2 ; i1 = 6 ; j0 = 3 ; j1 = 7 ; break ; // 100 101 110 111 x=1 plane
                       case  4 : i0 = 0 ; i1 = 4 ; j0 = 2 ; j1 = 6 ; break ; // 000 001 100 101 y=0 plane
                       case  5 : i0 = 1 ; i1 = 5 ; j0 = 3 ; j1 = 7 ; break ; // 010 011 110 111 y=1 plane
                    }
          // for every isoParamLine on Face
          for(jj=0,dParami=dParamInc;jj<lSmpCnt;jj++,dParami+=dParamInc)
            {
              sParamPtI = dParami * sParamCorners[i0] + (1.0-dParami) * sParamCorners[i1] ;
              sParamPtJ = dParami * sParamCorners[j0] + (1.0-dParami) * sParamCorners[j1] ;

              // for every isoParamLine sample point
              for(kk=0,dParamj=dParamInc;kk<lSmpCnt;kk++,dParamj+=dParamInc)
                {
                  // map edge samples to ProjectSpace      sParamPtI
                  sParamPoint = dParamj * sParamPtI + (1.0-dParamj) * sParamPtJ ;
                  EvaluatePoint(sParamPoint, sProjPoint) ; 

                  // Add point to bounding boxes
                  if(pProjBox)       { pProjBox->AddPoint3d(sProjPoint) ; }
                  if(pProjPseudoBox) { pProjPseudoBox->AddPoint3d(sProjPoint) ; }
                } // end iter kk, every sample on iso param curve
            } // end iter jj, every face isoParam curve
        } // end iter ii, every face
    } // end input BoundingBox is bounded branch
  else // just return an infinite box - we could do something here about half spaces
    {
      // set output to infinite boxes
      if(pProjBox)       { pProjBox->SetUnbounded() ; }
      if(pProjPseudoBox) { pProjPseudoBox->SetUnbounded() ; }

    } // end given Box is unbounded branch

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::EvaluateBoundingBoxSimple

/*******************************************************************//**
PURPOSE: Map the given ParamSpace PseudoBox to ProjectSpace

NOTES: This method is the default base behavior for simple EvaluateBoundingBox
  1. If Given, maps pOptParamPseudoBox basis vectors to ProjectSpace and uses
       those to set the orientation of pProjSpacePseudoBox.
  2. When given crParamBox
       is bounded  : Adds the ProjSpace mapped CornerPoints of the ParamSpace box 
                     to the output boxes, pProjBox and pProjPseudoBox. 
       is unbounded:
***********************************************************************/
SmStatus SmVolume::EvaluatePseudoBoxSimple
  (const SmPseudoBox & crParamPseudoBox, // in : ParamSpace BBox to project to Project Space 
   SmExtent3d        * pProjBox,         // out: ProjectSpace Axis aligned box
   SmPseudoBox       * pProjPseudoBox)   // out: ProjectSpace Non-axis aligned box
  const
{
  // local
  ULONG lSmpCnt = 5 ;  // Each of 6 BBox faces are sampled dSmpCnt x dSmpCnt times: dSmpCnt5 => 150 samples
                       // Plus 12 BBox edges sampled dSmpCnt times: dSmpCnt 5 => 60 samples
                       // Plus 8  BBox corners sampled once       : 8 samples
                       // dSmpCnt Total = 150 + 60 + 8 = 218 samples

  // no work - no output
  if(   pProjBox == NULL
     && pProjPseudoBox == NULL)
    { return SM_SUCCESS ; }

  // when input ParamPseudoBox is same as output ProjPseudoBox - copy input ParamPseudoBox
  SmPseudoBox         sParamPseudoBox ;
  const SmPseudoBox * pParamPseudoBox ;
  if(pProjPseudoBox && (&crParamPseudoBox == pProjPseudoBox)) { sParamPseudoBox = crParamPseudoBox ;
                                                                pParamPseudoBox = &sParamPseudoBox ;
                                                              }
  else                                                        { pParamPseudoBox = &crParamPseudoBox ;
                                                              }

  // indirection: from here on out - pParamPseudoBox == ParamSpace PseudoBox

  // locals
  SmPoint3d  sProjPoint ;
  SmVector3d sDX, sDY, sDZ ;
  SmVector3d sO = crParamPseudoBox.Evaluate(.5,.5,.5) ;

  // init ProjSpaceBox
  if(pProjBox) { pProjBox->Init() ; }

  // Init pProjPseudoBox
  if(pProjPseudoBox)
    {
      // orient pProjPseudoBox to projection of pOptParamPseudoBox orientation to ProjectSpace 
      
      // map ParamSpace basis vectors to ProjectSpace                    
      Evaluate1stDerivatives(sO, TRUE, TRUE, TRUE, sProjPoint, sDX, sDY, sDZ, TRUE) ;

      // Extract the projected PseudoBox basis directions
      const SmVector3d &rBX = pParamPseudoBox->GetBasis(0) ;
      const SmVector3d &rBY = pParamPseudoBox->GetBasis(1) ;
      const SmVector3d &rBZ = pParamPseudoBox->GetBasis(2) ;
      SmVector3d sProjDX = rBX.x * sDX + rBX.y * sDY + rBX.z * sDZ ;
      SmVector3d sProjDY = rBY.x * sDX + rBY.y * sDY + rBY.z * sDZ ;
      SmVector3d sProjDZ = rBZ.x * sDX + rBZ.y * sDY + rBZ.z * sDZ ;

      // unitize the projected directions
      sProjDX.Unitize() ;                                                
      sProjDY.Unitize() ;                                                
      sProjDZ.Unitize() ;  

      // orient the ProjectSpace PseudoBox to the projection of the ParamSpace PseudoBox orientation                                         
      pProjPseudoBox->SetBasis(sProjDX, sProjDY, sProjDZ) ;
      pProjPseudoBox->Init() ; 

    } // end setting pProjPseudoBox orientation branch

  // When given ParamSpace BoundingBox is bounded
  if(pParamPseudoBox->IsBounded())
    {
      // get ordered corners
      ULONG ii,jj,kk ;
      ULONG i0 = 0, i1 = 0, j0 = 0, j1 = 0 ;  
      double     dParami, dParamj, dParamInc = 1.0 / (lSmpCnt + 1.0) ;
      SmPoint3d  sParamPoint, sParamPtI, sParamPtJ ; 
      SmTArray<SmPoint3d> sParamCorners(8,NULL,8) ;
      pParamPseudoBox->GetCorners(sParamCorners) ; // ordered:{ 000 010 100 110 001 011 101 111 }
      SM_ASSERT(sParamCorners.GetSize() == 8) ; 

      // corners
      for(ii=0;ii<8;ii++)
        {
          // map the corners to ProjectSpace
          EvaluatePoint(sParamCorners[ii], sProjPoint) ; 

          // Add point to bounding boxes
          if(pProjBox)       { pProjBox->AddPoint3d(sProjPoint) ; }
          if(pProjPseudoBox) { pProjPseudoBox->AddPoint3d(sProjPoint) ; }
        } // end iter kk

      // Edges
      for(ii=0;ii<12;ii++)
        { 
          // ordered:{ 000 010 100 110 001 011 101 111 }
          switch(ii) { case  0 : i0 = 0 ; i1 = 1 ; break ; // 000 010  z=0 plane edges
                       case  1 : i0 = 1 ; i1 = 2 ; break ; // 010 100
                       case  2 : i0 = 2 ; i1 = 3 ; break ; // 100 110
                       case  3 : i0 = 3 ; i1 = 0 ; break ; // 110 000 
                       case  4 : i0 = 4 ; i1 = 5 ; break ; // 001 011  z=1 plane edges
                       case  5 : i0 = 5 ; i1 = 6 ; break ; // 011 101
                       case  6 : i0 = 6 ; i1 = 7 ; break ; // 101 111
                       case  7 : i0 = 7 ; i1 = 4 ; break ; // 111 001 
                       case  8 : i0 = 0 ; i1 = 4 ; break ; // 000 001  edges from z=0 to z=1
                       case  9 : i0 = 1 ; i1 = 5 ; break ; // 010 011
                       case 10 : i0 = 2 ; i1 = 6 ; break ; // 100 101
                       case 11 : i0 = 3 ; i1 = 7 ; break ; // 110 111
                     }
          for(jj=0,dParami=dParamInc;jj<lSmpCnt;jj++, dParami+=dParamInc)
            {
              // map edge samples to ProjectSpace
              sParamPoint = dParami * sParamCorners[i0] + (1.0-dParami) * sParamCorners[i1] ;
              EvaluatePoint(sParamPoint, sProjPoint) ; 

              // Add point to bounding boxes
              if(pProjBox)       { pProjBox->AddPoint3d(sProjPoint) ; }
              if(pProjPseudoBox) { pProjPseudoBox->AddPoint3d(sProjPoint) ; }
            } // end iter jj, samples on the edge
        } // end iter ii, edges

      // faces
      for(ii=0;ii<6;ii++)
        {
          // ordered:{ 000 010 100 110 001 011 101 111 }
          //            0   1   2   3   4   5   6   7
          switch(ii) { case  0 : i0 = 0 ; i1 = 1 ; j0 = 2 ; j1 = 3 ; break ; // 000 010 100 110 z=0 plane
                       case  1 : i0 = 4 ; i1 = 5 ; j0 = 6 ; j1 = 7 ; break ; // 001 011 101 111 z=1 plane
                       case  2 : i0 = 0 ; i1 = 4 ; j0 = 1 ; j1 = 5 ; break ; // 000 001 010 011 x=0 plane
                       case  3 : i0 = 2 ; i1 = 6 ; j0 = 3 ; j1 = 7 ; break ; // 100 101 110 111 x=1 plane
                       case  4 : i0 = 0 ; i1 = 4 ; j0 = 2 ; j1 = 6 ; break ; // 000 001 100 101 y=0 plane
                       case  5 : i0 = 1 ; i1 = 5 ; j0 = 3 ; j1 = 7 ; break ; // 010 011 110 111 y=1 plane
                    }
          // for every isoParamLine on Face
          for(jj=0,dParami=dParamInc;jj<lSmpCnt;jj++,dParami+=dParamInc)
            {
              sParamPtI = dParami * sParamCorners[i0] + (1.0-dParami) * sParamCorners[i1] ;
              sParamPtJ = dParami * sParamCorners[j0] + (1.0-dParami) * sParamCorners[j1] ;

              // for every isoParamLine sample point
              for(kk=0,dParamj=dParamInc;kk<lSmpCnt;kk++,dParamj+=dParamInc)
                {
                  // map edge samples to ProjectSpace      sParamPtI
                  sParamPoint = dParamj * sParamPtI + (1.0-dParamj) * sParamPtJ ;
                  EvaluatePoint(sParamPoint, sProjPoint) ; 

                  // Add point to bounding boxes
                  if(pProjBox)       { pProjBox->AddPoint3d(sProjPoint) ; }
                  if(pProjPseudoBox) { pProjPseudoBox->AddPoint3d(sProjPoint) ; }
                } // end iter kk, every sample on iso param curve
            } // end iter jj, every face isoParam curve
        } // end iter ii, every face
    } // end input BoundingBox is bounded branch
  else // just return an infinite box - we could do something here about half spaces
    {
      // set output to infinite boxes
      if(pProjBox)       { pProjBox->SetUnbounded() ; }
      if(pProjPseudoBox) { pProjPseudoBox->SetUnbounded() ; }

    } // end given Box is unbounded branch

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::EvaluatePseudoBoxSimple

/*******************************************************************//**
PURPOSE: Create ProjSpace IsoCurve from a ParamSpace IsoParamLine

NOTES: Creates and returns appropriated SmCrvInVolume objects whose geometry
       are defined by a line and this Volume.  This is the default behavior
       for all classes derived from SmVolume.

       Derived classes whose isoparameter lines projected to ProjSpace are
       explicitly defined shapes (e.g. SmLine, SmCircle, ...) should
       implement their own derived versions of this method.
   +-----------------+----------------+----------------+-------------------+
   | eConstantParams | dIsoParameter1 | dIsoParameter2 | Varying Parameter |
   +-----------------+----------------+----------------+-------------------+
   |    SM_VPS_UV    |   constant u   |   constant v   |     varying w     |
   |    SM_VPS_UW    |   constant u   |   constant w   |     varying v     |
   |    SM_VPS_VW    |   constant v   |   constant w   |     varying u     |
   +-----------------+----------------+----------------+-------------------+

***********************************************************************/
SmStatus SmVolume::EvaluateIsoParametricCurveSimple
 (const SmContext     & crContext,        // in : context for created objects
  SmVolumeParamsType    eConstantParams,  // in : oneof: SM_VPS_UV_IN,
                                          //             SM_VPS_UW_IN,
                                          //             SM_VPS_VW_IN.
  double                dIsoParam1,       // in : 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value
  double                dIsoParam2,       // in : 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value
  double                d3DTolerance,     // NotUsed: in : Max ApproxCurve to IdealCurve deviation
  SmCurve            *& rpNewIsoCurve,    // out: the ProjSpace IsoCurve
  const SmExtent3d    * pOptParamDomain)  // in : limiting domain, NULL to ignore. default:[NULL]  
 const
{
  SM_REF1(d3DTolerance) ;
  // init output
  rpNewIsoCurve = NULL ; 

  // locals        
  SmVector3d sParamStart, sParamMid, sParamEnd ;
  SmVector3d sProjStart, sProjMid, sProjEnd ;
  SmExtent1d sParamIvl ;

  // TargetDomain - use input value when given, else use NaturalParamDomain
  SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
  const SmExtent3d *pTgtDomain   = pOptParamDomain ? pOptParamDomain : &sParamDomain ;

  // varying u - build line params parallel to the Param U Axis
  if(eConstantParams == SM_VPS_VW)
    {
      // locals
      double dUMin = pTgtDomain->GetUMin() ; 
      double dUMax = pTgtDomain->GetUMax() ;
      double dV    = dIsoParam1 ;  
      double dW    = dIsoParam2 ;  

      // ParamLine EndPoints and Ivl
      sParamStart.Set(dUMin, dV, dW) ;
      sParamEnd.  Set(dUMax, dV, dW) ;
      sParamIvl.SetMinMax(dUMin, dUMax) ;

    } // end varying u building Paramline params parallel to the Param U Axis

  // varying v - build line params parallel to the Param V Axis
  else if(eConstantParams == SM_VPS_UW)
    {
      double dU    = dIsoParam1 ; 
      double dVMin = pTgtDomain->GetVMin() ; 
      double dVMax = pTgtDomain->GetVMax() ;
      double dW    = dIsoParam2 ;

      // ParamLine EndPoints and Ivl
      sParamStart.Set(dU, dVMin, dW) ;
      sParamEnd.  Set(dU, dVMax, dW) ;
      sParamIvl.SetMinMax(dVMin, dVMax) ;

    } // end varying v building Paramline params parallel to the Param V Axis

  // varying w - - build line params parallel to the Param W Axis
  else if(eConstantParams == SM_VPS_UV)
    {
      double dU        =  dIsoParam1 ;
      double dV        =  dIsoParam2 ;
      double dWMin     =  pTgtDomain->GetWMin() ;
      double dWMax     =  pTgtDomain->GetWMax() ;
      
      // ParamLine EndPoints and Ivl
      sParamStart.Set(dU, dV, dWMin) ;
      sParamEnd.  Set(dU, dV, dWMax) ;
      sParamIvl.SetMinMax(dWMin, dWMax) ;

    } // end varying w building Paramline params parallel to the Param W Axis

  else
    {
      SER_MSG(SM_ERR, _T("Bad Input eConstantParams value")) ; 
    }
      
  // when sParamStart or sParamEnd is not within pOptParamDomain
  if(   !pTgtDomain->ContainsPoint3d(sParamStart, SM_EFF_ZERO) 
     || !pTgtDomain->ContainsPoint3d(sParamEnd, SM_EFF_ZERO)) 
    {
      // no curve to return - all done
      return(SM_SUCCESS) ;
    }

  // project ParamLine end and mid Pts to ProjSpace
  sParamMid = (sParamStart + sParamEnd) / 2.0 ; 

  EvaluatePointSimple(sParamStart, sProjStart) ;
  EvaluatePointSimple(sParamMid,   sProjMid) ;
  EvaluatePointSimple(sParamEnd,   sProjEnd) ;

  // cull degenerate ProjSpace curves
  if(   sProjStart.DistanceBetweenSquared(sProjEnd) <= SM_EFF_ZERO_SQ 
     && sParamMid. DistanceBetweenSquared(sProjEnd) <= SM_EFF_ZERO_SQ)
    {
      // return degenerate curve
      SmCurve::CreateDegenerateCurve(crContext, 3, sProjStart, rpNewIsoCurve) ; 
      return(SM_SUCCESS) ;
    }

  // Param IsoLine properties for desired parameterization 
  //  - set LinePoint and LineVec so that
  //      sParamStart = sParamLinePoint + sParamIvl.GetMin() * sParamLineVector ;
  //      sParamEnd   = sParamLinePoint + sParamIvl.GetMax() * sParamLineVector ;
  double     dScale = 1.0 ;
  SmVector3d sParamLinePoint, sParamLineVector = sParamEnd - sParamStart ;
  sParamLineVector.Unitize() ;
  sParamLinePoint = sParamStart - sParamIvl.GetMin() * sParamLineVector ;

  // create properly parameterized ParamLine
  SmLine *pGenLine = new (crContext) SmLine(sParamLinePoint, sParamLineVector, sParamIvl, dScale, 3, &crContext) ; 

  // set output = SmCrvInVolume (no input copies, delete GenLine and don't delete this SmVolume when Object is deleted)
  rpNewIsoCurve = new (crContext) SmCrvInVolume(*pGenLine,         // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
                                                TRUE,              // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
                                                (SmVolume&)*this,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                                2,                 // in : 2 = copy volume and save curve orig
                                                3,                 // in : 3 = delete both curve and volume when destructed
                                                &crContext) ;      // in : req for stack objs, opt for heap objs

  // Make sure Copied Volume Orient is NULL so that rpNewIsoCurve sets in ProjSpace and not OutSpace
  ((SmCrvInVolume*)rpNewIsoCurve)->GetVolume()->MakeMapSimple() ;

  // all done
  return SM_SUCCESS ;

} // end SmVolume::EvaluateIsoParametricCurveSimple

/*******************************************************************//**
PURPOSE: Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane

NOTES: Creates and returns appropriated SmSrfInVolume objects whose geometry
       are defined by a plane and this Volume.  This is the default behavior
       for SmVolume derived classes.

       Derived classes whose isoparameter planes projected to ProjSpace are
       explicitly defined shapes (e.g. SmPlane, SmCylinder, ...) should
       implement their own derived versions of this method.
     +----------------+---------------+--------------------+
     | eConstantParam | dIsoParam     | varying parameters |
     +----------------+---------------+--------------------+
     |   SM_VP_U      | constant u    | varying v, w       |
     |   SM_VP_V      | constant v    | varying u, w       |
     |   SM_VP_W      | constant w    | varying u, v       |
     +----------------+---------------+--------------------+

***********************************************************************/
SmStatus SmVolume::EvaluateIsoParametricSurfaceSimple
 (const SmContext   & crContext,        // in : context for created objects
  SmVolumeParamType   eConstantParam,   // in : oneof: SM_VP_U_IN,
                                        //             SM_VP_V_IN,
                                        //             SM_VP_W_IN
  double              dIsoParam,        // in : constant param value
  double              d3DTolerance,     // NotUsed: in : Max ApproxSurface to IdealSurface deviation        
  SmSurface        *& rpNewIsoSurface,  // out: the ProjSpace IsoSurface                        
  const SmExtent3d  * pOptParamDomain)  // in : limiting domain, NULL to ignore. default:[NULL]
const
{
  SM_REF1(d3DTolerance) ;
  // init output
  rpNewIsoSurface = NULL ;

  // locals
  SmVector3d        sOrigin(0,0,0), sXAxis(0,0,0), sYAxis(0,0,0) ;
  SmVector3d        sParamMin, sParamMid, sParamMax ;
  SmVector3d        sProjMin, sProjMid, sProjMax ;
  SmVector2d        sUVScale(1,1) ;
  SmExtent2d        sUVDomain ;
  SmExtent3d        sParamDomain = GetNaturalParamDomain() ;
  const SmExtent3d *pTgtDomain   = pOptParamDomain ? pOptParamDomain : &sParamDomain ;

  // constant u - build plane params perpendicular to the Param U Axis
  if(eConstantParam == SM_VP_U)
    {
      double dU    = dIsoParam ; 
      double dMinV = pTgtDomain->GetVMin() ;
      double dMaxV = pTgtDomain->GetVMax() ;    
      double dMinW = pTgtDomain->GetWMin() ;     
      double dMaxW = pTgtDomain->GetWMax() ;  
      
      // set origin and axes so that 
      //  ParamPoint[dU,dMinV,dMinW] = Origin + sV * dMinV + sW * dMinW, and
      //  ParamPoint[dU,dMaxV,dMaxW] = Origin + sV * dMaxV + sW * dMaxW

      sOrigin.Set(dU, 0, 0) ;
      sXAxis. Set(0,1,0) ;
      sYAxis. Set(0,0,1) ; 
      sUVDomain.SetMinMax(dMinV, dMinW, dMaxV, dMaxW) ; 
      sParamMin.Set(dU, dMinV, dMinW) ;
      sParamMax.Set(dU, dMaxV, dMaxW) ; 

    } // end constant u building ParamPlane params perpendicular to the Param U axis

  // constant v - build plane params perpendicular to the Param V Axis
  else if(eConstantParam == SM_VP_V)
    {
      double dMinU = pTgtDomain->GetUMin() ;
      double dMaxU = pTgtDomain->GetUMax() ;
      double dV    = dIsoParam ;     
      double dMinW = pTgtDomain->GetWMin() ;     
      double dMaxW = pTgtDomain->GetWMax() ;  
      
      // set origin and axes so that 
      //  ParamPoint[dMinU,dV,dMinW] = Origin + sU * dMinU + sW * dMinW, and
      //  ParamPoint[dMaxU,dV,dMaxW] = Origin + sU * dMaxU + sW * dMaxW

      sOrigin.Set(0, dV, 0) ;
      sXAxis. Set(1,0,0) ;
      sYAxis. Set(0,0,1) ; 
      sUVDomain.SetMinMax(dMinU, dMinW, dMaxU, dMaxW) ;  
      sParamMin.Set(dMinU, dV, dMinW) ;
      sParamMax.Set(dMaxU, dV, dMaxW) ; 

    } // end constant v building ParamPlane params perpendicular to the Param V axis

  // constant w - build plane params perpendicular to the Param W Axis
  else if(eConstantParam == SM_VP_W)
    {
      double dMinU = pTgtDomain->GetUMin() ;
      double dMaxU = pTgtDomain->GetUMax() ;
      double dMinV = pTgtDomain->GetVMin() ;        
      double dMaxV = pTgtDomain->GetVMax() ;     
      double dW    = dIsoParam ;  
      
      // set origin and axes so that 
      //  ParamPoint[dMinU,dMinV,dW] = Origin + sU * dMinU + sV * dMinV, and
      //  ParamPoint[dMaxU,dMaxV,dW] = Origin + sU * dMaxU + sV * dMaxV

      sOrigin.Set(0, 0, dW) ;
      sXAxis. Set(1,0,0) ;
      sYAxis. Set(0,1,0) ; 
      sUVDomain.SetMinMax(dMinU, dMinV, dMaxU, dMaxV) ;  
      sParamMin.Set(dMinU, dMinV, dW) ;
      sParamMax.Set(dMaxU, dMaxV, dW) ; 

    } // end constant w building ParamPlane params perpendicular to the Param W axis

  // default - bad eConstantParam value - inform the public
  else
    {
      SER_MSG(SM_ERR, _T("Bad Input eConstantParam value")) ; 
    }

  // check input - ParamExt corners must be in ParamSpace
  if(   !pTgtDomain->ContainsPoint3d(sParamMin, SM_EFF_ZERO) 
     || !pTgtDomain->ContainsPoint3d(sParamMax, SM_EFF_ZERO)) 
    {
      // no surface to return - all done
      return(SM_SUCCESS) ;
    }

  // project Param Min, Mid, and Max points to ProjSpace
  sParamMid = (sParamMin + sParamMax) / 2.0 ;

  EvaluatePointSimple(sParamMin, sProjMin) ;
  EvaluatePointSimple(sParamMid, sProjMid) ;
  EvaluatePointSimple(sParamMax, sProjMax) ;

  // Check input - degenerate ProjSpace shapes
  if(   sProjMin.DistanceBetweenSquared(sProjMax) <= SM_EFF_ZERO_SQ
     && sProjMid.DistanceBetweenSquared(sProjMax) <= SM_EFF_ZERO_SQ)
    {     
      // no surface to return - all done
      return(SM_SUCCESS) ; 
    }

  // GenSurface with desired parameterization and orientation
  SmPlane * pGenPlane = new (crContext) SmPlane(sOrigin, sXAxis, sYAxis, sUVScale, sUVDomain, &crContext) ;

  // set output = SmSrfInVolume (no input copies, delete GenPlane and don't delete this SmVolume when Object is deleted)
  rpNewIsoSurface = new (crContext) SmSrfInVolume(*pGenPlane,        // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this
                                                  TRUE,              // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace
                                                  (SmVolume&)*this,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                                  2,                 // in : 2 = copy volume and save surface orig
                                                  3,                 // in : 3 = delete both surface and volume when destructed
                                                  &crContext) ;      // in : req for new stack objs, opt for new heap objs

  // Make sure Copied Volume Orient is NULL so that rpNewIsoCurve sets in ProjSpace and not OutSpace
  ((SmSrfInVolume*)rpNewIsoSurface)->GetVolume()->MakeMapSimple() ;

  // all done
  return SM_SUCCESS;

} // end SmVolume::EvaluateIsoParametricSurfaceSimple

/*******************************************************************//**
PURPOSE: Evaluate just the point without compounding from ParamSpace
         to ProjSpace

NOTES: 
***********************************************************************/
SmStatus SmVolume::EvaluatePointSimple
  (const SmPoint3d & crParamPoint,  // in : point to map from ParamSpace
   SmPoint3d       & rProjPoint)    // out: mapped point in ProjSpace
 const
{
  // pass the call along
  return( EvaluateSimple( crParamPoint, 0, TRUE, TRUE, TRUE, &rProjPoint) ) ; 

} // end SmVolume::EvaluatePointSimple

/*******************************************************************//**
PURPOSE: Evaluate a point and partial first derivatives of a volume without compounding

NOTES: 
***********************************************************************/
SmStatus SmVolume::Evaluate1stDerivativesSimple
  (const SmPoint3d & crInSpacePoint, // in : InSpace point to map to OutSpace point
   SmBoolean         bXinFromLeft,   // in : if P is on Xin, Yin, or Zin interval boundary
   SmBoolean         bYinFromLeft,   //      TRUE  = evaluate P in upper interval where P is on the left of the interval
   SmBoolean         bZinFromLeft,   //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmPoint3d       & rOutSpacePoint, // out: OutSpace Point
   SmVector3d      & rDX,            // out: Xin direction tangent
   SmVector3d      & rDY,            // out: Yin direction tangent
   SmVector3d      & rDZ,            // out: Zin direction tangent
   SmBoolean       bNonZeroTangents) // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
 const                               //      FALSE= return exact tangent values, default:[TRUE]
{
  // pass the call along
  SmVector3d sMat[2][2][2];
  SER(EvaluateSimple(crInSpacePoint,1,bXinFromLeft,bYinFromLeft,bZinFromLeft,sMat[0][0],bNonZeroTangents));

  // set output
  rOutSpacePoint = sMat[0][0][0];
  rDX            = sMat[1][0][0];
  rDY            = sMat[0][1][0];
  rDZ            = sMat[0][0][1];

  // all done
  return SM_SUCCESS;

} // end SmVolume::Evaluate1stDerivativesSimple

/*******************************************************************//**
PURPOSE: Evaluate a point, partial first, and partial second derivatives
   of a volume without compounding.

NOTES: Note this routine attempts to step off when singularities
   are found.
***********************************************************************/
SmStatus SmVolume::Evaluate2ndDerivativesSimple
  (const SmPoint3d & crInSpacePoint, // in : InSpace point to map to OutSpace Point
   SmBoolean         bXinFromLeft,   // in : if P is on Xin, Yin, or Zin interval boundary
   SmBoolean         bYinFromLeft,   //      TRUE  = evaluate P in upper interval where P is on the left of the interval
   SmBoolean         bZinFromLeft,   //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmPoint3d       & rOutSpacePoint, // out: OutSpace position           
   SmVector3d      & rDX,            // out: OutSpace Xin direction tangent
   SmVector3d      & rDY,            // out: OutSpace Yin direction tangent
   SmVector3d      & rDZ,            // out: OutSpace Zin direction tangent
   SmVector3d      & rDXX,           // out: OutSpace Xin direction 2nd derivative
   SmVector3d      & rDYY,           // out: OutSpace Yin direction 2nd derivative
   SmVector3d      & rDZZ,           // out: OutSpace Zin direction 2nd derivative
   SmVector3d      & rDXY,           // out: OutSpace XYin 2nd order cross derivative
   SmVector3d      & rDXZ,           // out: OutSpace XZin 2nd order cross derivative
   SmVector3d      & rDYZ,           // out: OutSpace YZin 2nd order cross derivative
   SmBoolean       bNonZeroTangents) // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
 const                               //      FALSE= return exact tangent values, default:[TRUE]
{                            
  SmVector3d sMat[3][3][3];  
  SER(Evaluate(crInSpacePoint,2,bXinFromLeft,bYinFromLeft,bZinFromLeft,sMat[0][0],bNonZeroTangents));
  rOutSpacePoint = sMat[0][0][0];    
  rDX            = sMat[1][0][0];    
  rDY            = sMat[0][1][0];    
  rDZ            = sMat[0][0][1];    
  rDXX           = sMat[2][0][0];    
  rDYY           = sMat[0][2][0];    
  rDZZ           = sMat[0][0][2];    
  rDXY           = sMat[1][1][0];    
  rDXZ           = sMat[1][0][1];    
  rDYZ           = sMat[0][1][1];    
                             
  return SM_SUCCESS;         

} // end SmVolume::Evaluate2ndDerivativesSimple

/*******************************************************************//**
PURPOSE: Compute higher order directional derivatives in ProjSpace
         using the chain rule.

NOTES: Limited to computing 3rd derivatives
***********************************************************************/
SmStatus SmVolume::EvaluateDirectionalDerivsSimple
 (const SmPoint3d  & crParamPoint,   // in : UVW Param Point to evaluate
  SmVector3d       & rParamDir,      // in : UVW direction for derivatives, gets unitized
  ULONG              lHighestDeriv,  // in : 1=1st deriv, 2=1st and 2nd derivs, 3=1st, 2nd, and 3rd derivs
  SmVector3d         aDirDerivs[],   // out: aDirDerivs[0] = position
                                     //      aDirDerivs[1] = 1st directional derivative
                                     //      aDirDerivs[2] = 2nd directional derivative
                                     //      aDirDerivs[3] = 3rd directional derivative
                                     //      sized:[lHighestDeriv+1]
  SmBoolean          bUFromLeft,     // in : if P is on U, V, or W interval boundary
  SmBoolean          bVFromLeft,     //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean          bWFromLeft)     //      FALSE = evaluate P in lower interval where P is on the right of the interval
 const
{
  // check state
  SER_MSG(lHighestDeriv <= 3 ? SM_SUCCESS : SM_ERR, _T("Directional Derivatives only supported to 3rd order")) ;

  // make the surface evaluation
  SmVector3d aDerivs[SM_VSIZE(3)] ;
  SER(EvaluateSimple(crParamPoint, lHighestDeriv, bUFromLeft, bVFromLeft, bWFromLeft, aDerivs, 
                     FALSE,     // exact tangent values
                     TRUE)) ;   // Do zero sampling

  // pass the call along
  SER(smgu_VolDirectionalDerivs(rParamDir, lHighestDeriv, aDerivs, aDirDerivs)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::EvaluateDirectionalDerivsSimple

/*******************************************************************//**
PURPOSE: InvertMap point from ProjSpace back to ParamSpace

NOTES: 
   Invert the given ProjSpace ProjPoint to find the ParamSpace ParamPoint
   whose projection back to ProjSpace is closest to the given ProjPoint.

   A single closest point will be returned. 

   For volumes with bounded ParamDomains when given ProjPoints who do not
   map from within the image of the volume, the found ParamPoint will be 
   on the domain boundary in which case the distance between the given 
   ProjPoint and the projection of the found ParamPoint may be arbitrarily large.

METHOD:      calls virtual method LocalPointSolve when given a pOptParamPointGuess
        else calls virtual method GlobalPointSolve()
***********************************************************************/
SmStatus SmVolume::InvEvaluatePointSimple
 (const SmPoint3d     & crProjPoint,          // in : ProjSpace Point to map back to InSpace Point
  const SmPoint3d     * pOptParamPointGuess,  // in : last intermediate ParamSpace guess location at which to start the search
  SmBoolean           & rbSuccess,            // out: TRUE = inverse was found, else FALSE
  SmTArray<SmPoint3d> & rParamPoints,         // out: the ParamSpace inverse mapping
  SmTArray<double>    & rdGaps,               // out: max distance between found result (in case of bounding) and target point
  const SmExtent3d    * pOptParamDomain)      // in : ParamSpace domain over which to search for the inverse point
 const                                        //      NULL = use Map's NaturalDomain, default:[NULL]
{
  // init output and return value
  SmStatus eStat = SM_SUCCESS ;
  rbSuccess      = FALSE;
  rParamPoints.ReSet() ;
  rdGaps.ReSet() ;

  // locals
  ULONG ii ;
  SmSolution sData[4];
  SmSolutionArray sSolutions(4,sData);

  // Try LocalPointSolve if a guess was supplied.
  if(pOptParamPointGuess)
    {
      sSolutions.SetSize(1) ;

      eStat = LocalPointSolveSimple( crProjPoint, *pOptParamPointGuess, rbSuccess, sSolutions[0], pOptParamDomain );
      if(eStat != SM_SUCCESS )
        { rbSuccess = FALSE ; }

    } // end pOptParamPointGuess existence check for LocalPointSolve() call.

  // When LocalPointSolve() can't be used or fails, try GlobalPointSolve.
  if(rbSuccess == FALSE)
    {
      SER( GlobalPointSolveSimple(crProjPoint, rbSuccess, sSolutions, pOptParamDomain));

      if(eStat != SM_SUCCESS )
        { rbSuccess = FALSE ; }
    } // need to run GlobalPointSolve() check

  // set output from LocalPointSolve() solution when appropriate
  if(rbSuccess)
    {
      rParamPoints.SetSize(sSolutions.GetSize()) ;
      rdGaps.SetSize(sSolutions.GetSize()) ;
      for(ii=0;ii<sSolutions.GetSize();ii++)
        {
          SmSolution &rSol = sSolutions[ii] ;

          rParamPoints[ii].Set( rSol.m_vStart[0], rSol.m_vStart[1], rSol.m_vStart[2] ) ;
          rdGaps[ii] =          rSol.m_vStart.m_dSolutionValue ;
        }
    
    } // end set output from LocalPointSolve() solution when appropriate

  // all done
  return SM_SUCCESS;

} // end SmVolume::InvEvaluatePointSimple

/*******************************************************************//**
PURPOSE: Drop a ProjSpace 3d vector whose origin corresponds to the 
    mapping of the given Param crParamPoint to ProjSpace
    back to Param vectors so that  
    rParamVec = xVecProj*rDX + yVecProj*rDY + zVecProj*rDZ

NOTES:
***********************************************************************/
SmStatus SmVolume::InvEvaluateVectorSimple
 (const SmVector3d     & crProjSpaceVec,     // in : ProjSpace Vector to map back to InSpace Vector
  const SmPoint3d      & crParamPoint,       // in : ParamSpace Point specifying where the Drop will take place (see InvEvaluatePoint())
  SmTArray<SmVector3d> & rParamVecs)         // out: Dropped ParamSpace vecs (2 if crParamPoint maps through seam)
 const                                       //      note: with rParamVecs[ii] = [ParamVecU, ParamVecV, ParamVecW] 
                                             //                 rOutSpaceVec   = ParamVecU*rDX + ParamVecV*rDY + ParamVecW*rDZ
                                             //            where rDX, rDY, rDZ are the outpspace 1stDeriv vectors at crParamPoint
{
  // check for seams
  SmBoolean bOnU, bOnV, bOnW ;
  SmBoolean bOnBoundary = IsOnBoundary(crParamPoint, bOnU, bOnV, bOnW) ;

  // init output
  rParamVecs.SetSize(bOnBoundary ? 2 : 1) ;

  // locals
  SmPoint3d sPnt;
  SmVector3d sOutSpace1stDerivs[3] ;
  
  // get first derivatives for OutPoint = Volume(InPoint) with compounding
  SER(Evaluate1stDerivativesSimple(crParamPoint, TRUE, TRUE, TRUE,
                                   sPnt, sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2],
                                   TRUE)) ;              // in : bNonZeroTangents

  // project rDropVec3D into non-orthogonal 1st derivative axis set
  SER(smvol_DropVector(sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2], crProjSpaceVec, rParamVecs[0]));

  // when on seam - evaluate other side
  if(bOnBoundary)
    {
      SER(Evaluate1stDerivativesSimple(crParamPoint, FALSE, FALSE, FALSE,
                                       sPnt, sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2],
                                       TRUE)) ;            // in : bNonZeroTangents

      // project rDropVec3D into non-orthogonal 1st derivative axis set
      SER(smvol_DropVector(sOutSpace1stDerivs[0], sOutSpace1stDerivs[1], sOutSpace1stDerivs[2], crProjSpaceVec, rParamVecs[1]));

    } // end on seam check

  // all done
  return SM_SUCCESS;

} // end SmVolume::InvEvaluateVectorSimple

/*******************************************************************//**
PURPOSE: inverse map a point from ProjSpace back to ParamSpace when 
         no ParamPoint guess is available

NOTES: Generates a GuessPoint from the UVW values associated with
     the controlPoint closest to the target point and then
     passes the call along to LocalPointSolve().

     Assumes the volume mapping is one-to-one and returns the first
     answer found.
***********************************************************************/
SmStatus SmVolume::GlobalPointSolveSimple  // eff: Find volume UVW Point that maps to TargetPoint
 (const SmPoint3d  & crProjPoint,          // in : ProjSpace Point to map back to ParamSpace
  SmBoolean        & rbFoundAnswer,        // out: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point
  SmSolutionArray  & rSolutions,           // out: Contains ParamSpace Found Point
                                           //      rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                                               
                                           //      rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);                      
                                           //      rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location                    
  const SmExtent3d * pOptParamDomain)      // in : ParamSpace domain over which to search for the inverse point
 const                                     //      NULL = use Map's NaturalDomain, default:[NULL]
{
  // init output
  rbFoundAnswer = FALSE ;
  rSolutions.ReSet() ;

  // locals
  ULONG ii ;
  SmSolution sThisSolution ;
  SmBoolean  bThisFoundAnswer ;
  SmTArray<SmPoint3d> sGuessPoints ;

  // Get sGuessPoint
  InvEvaluateGuessPointSimple(crProjPoint, sGuessPoints) ;

  // for every possible guess point
  for(ii=0;ii<sGuessPoints.GetSize();ii++)
    {
      SER(LocalPointSolveSimple(crProjPoint,
                                sGuessPoints[ii],  
                                bThisFoundAnswer,
                                sThisSolution,
                                pOptParamDomain) ) ; 

      // accumulate the output
      rbFoundAnswer |= bThisFoundAnswer ;
      if(bThisFoundAnswer)
        { rSolutions.Add(sThisSolution) ; }

    } // end iter every guess point
  
  // all done
  return(SM_SUCCESS) ;  
  
} // end SmVolume::GlobalPointSolveSimple

/*******************************************************************//**
PURPOSE: Find the Volume ParamSpace point which maps to a given 
         target ProjSpace Point given a ParamSpace GuessPoint.
    
   When a solution is found, sets
     rbFoundAnswer                       = TRUE 
     rSolution.m_eSolutionType           = SM_ST_SINGLE_VALUE
     rSolution.m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjSpacePoint);
     rSolution.m_vStart[0]               =  u of [u,v,w] the found ParamSpace point location
     rSolution.m_vStart[1]               =  v of [u,v,w] the found ParamSpace point location
     rSolution.m_vStart[2]               =  w of [u,v,w] the found ParamSpace point location
     rSolution.m_lNumVariables           = 3 ;
     rSolution.m_lNumObjects             = 1 ;
     rSolution.m_apObjects[0]            = (SmVolume *)this ;
   else sets
    rbFoundAnswer = FALSE.

NOTES: Valid solver operations include 
                    SM_SO_MINIMIZE,  -\           
                    SM_SO_MAXIMIZE,   -} - returns Same Answer and Same Failures
                    SM_SO_NORMALIZE, -/            
                    SM_SO_INTERSECT.  
***********************************************************************/
SmStatus SmVolume::LocalPointSolveSimple
 (const SmPoint3d  & crProjPoint,          // in : ProjSpace Point to map back to ParamSpace
  const SmPoint3d  & crParamPointGuess,    // in : ParamSpace point guess, the closer to the actual ParamSpace point the better
  SmBoolean        & rbFoundAnswer,        // out: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point            
  SmSolution       & rSolution,            // out: Contains ParamSpace Found Point
                                           //      rSolution.m_eSolutionType           = SM_ST_SINGLE_VALUE                                               
                                           //      rSolution.m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint);                      
                                           //      rSolution.m_vStart[0]  =  U of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution.m_vStart[1]  =  V of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution.m_vStart[2]  =  W of [U,V,W] the found ParamSpace point location                    
                                           //      rSolution.m_lNumVariables = 3 ;               
                                           //      rSolution.m_lNumObjects   = 1 ;               
                                           //      rSolution.m_apObjects[0]  = (SmVolume *)this ;
  const SmExtent3d * pOptParamDomain)      // in : ParamSpace domain over which to search for the inverse point
 const                                     //      NULL = use Map's NaturalDomain, default:[NULL]
{                                        
  // init output
  rbFoundAnswer             = FALSE ;
  rSolution.m_eSolutionType = SM_ST_UNKNOWN ;

  // locals
  double adSolData[3] ;
  SmTArray<double> sSolutionVector (3,adSolData) ;
  SmExtent3d sParamDomain ;
  const SmExtent3d *pParamDomain=NULL ;
  if(pOptParamDomain) { pParamDomain = pOptParamDomain ; }
  else                { sParamDomain = GetNaturalParamDomain() ; 
                        pParamDomain = &sParamDomain ; 
                      }

  // SolveIt Interval locals
  SmExtentNd sIntervals(3);
  sIntervals[0].SetMinMax(pParamDomain->GetUMin(), pParamDomain->GetUMax()) ;    
  sIntervals[1].SetMinMax(pParamDomain->GetVMin(), pParamDomain->GetVMax()) ;    
  sIntervals[2].SetMinMax(pParamDomain->GetWMin(), pParamDomain->GetWMax()) ;    

  // Do not utilize periodicities in local solver -- local solvers
  // should not jump across seams.
  //sPeriodicities[0] = IsPeriodic(sIntervals[0]) ;
  //sPeriodicities[1] = IsPeriodic(sIntervals[1]) ;
  //sPeriodicities[2] = IsPeriodic(sIntervals[2]) ;

  // SolveIt Periodicity locals
  // for now - don't check for periodicity in Volumes
  SmBoolean           alPerData[3] ;
  SmTArray<SmBoolean> sPeriodicities(3,alPerData,3) ;
  sPeriodicities[0] = FALSE ; 
  sPeriodicities[1] = FALSE ; 
  sPeriodicities[2] = FALSE ; 

  // set the guess value
  double              adGuessData[3] ;
  SmTArray<double>    sGuessT(3,adGuessData,3) ;
  sGuessT[0] = crParamPointGuess.x ;
  sGuessT[1] = crParamPointGuess.y ;
  sGuessT[2] = crParamPointGuess.z ;
  SmBoolean bFoundSolution ;

  // 1. construct NR Solver - the equation set callback function
  //                        - the Newton-Raphson solver using the equation set
  //                        - specify boundary behavior
  //                        - specify convergence distance as SM_EFF_ZERO
  SmFindPVIntersectENFO sEvalFun(crProjPoint, *this, SM_EFF_ZERO*SM_EFF_ZERO) ;
  SmLocalSolveNd        sLS     (sEvalFun,          // in : Define Eqns to set to Zero (defines the DOF COUNT)
                                 &sIntervals,       // in : NULL or sized:[N], N = this rEvalFun DOF count
                                                    //      bounds on problem parameters.
                                                    //      Set m_cpIntervals[i].SetUnbounded() for ivl whose prob params don't get clamped
                                 &sPeriodicities) ; // in : NULL or sized:[N], N = this rEvalFun DOF count
                                                    //      ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                                    //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped

  sLS.SetBoundaryHandler(SM_BH_TOTAL_BOUNDARY_HITS,3) ;

  // 2. find anser with NR solver
  SER(sLS.SolveIt(sGuessT, SM_EFF_ZERO_SQRT*100.0, bFoundSolution, sSolutionVector));

  // 3. when SolveIt failed to converge
  if (!bFoundSolution) 
    { // gwc:note sLS.SolveIt() does not return SM_TR_OUT_OF_BOUNDS
      // I suspect it can't distinguish between a valid and invalid
      // out of bounds solution.  So for now any failure to converge
      // is considered a failure.

      rbFoundAnswer = FALSE ;
      return SM_SUCCESS ;
    }

  // 3. else when SolveIt converged or found a boundary value solution
  //    let solutionValue = dist(TestPoint,SolutionPoint)
  SmPoint3d sPnt1 ;
  SmPoint3d sUVW(sSolutionVector[0], sSolutionVector[1], sSolutionVector[2]) ;
  SER(EvaluatePoint(sUVW, sPnt1)) ;

  // let rbFoundAnswer = TRUE and set output
  rbFoundAnswer                       = TRUE ;
  rSolution.m_eSolutionType           = SM_ST_SINGLE_VALUE ;
  rSolution.m_vStart.m_dSolutionValue = sPnt1.DistanceBetween(crProjPoint) ;
  rSolution.m_vStart[0]               = sSolutionVector[0] ;
  rSolution.m_vStart[1]               = sSolutionVector[1] ;
  rSolution.m_vStart[2]               = sSolutionVector[2] ;
  rSolution.m_lNumVariables           = 3 ;
  rSolution.m_lNumObjects             = 1 ;
  rSolution.m_apObjects[0]            = (SmVolume *)this ;

  // all done
  return SM_SUCCESS;

} // end SmVolume::LocalPointSolveSimple

/*******************************************************************//**
PURPOSE: Returns the Param space interval for the given InSpace Line

NOTES: returns success
***********************************************************************/
SmStatus SmVolume::FindParamIntervalForInSpaceLineSimple // rtn: SM_ERR for NULL line, else SM_SUCCESS
 (const SmPoint3d      & crInSpacePoint,                 // in : LinePoint         of Line(s) = LinePoint + s * LineVector               
  const SmVector3d     & crInSpaceVector,                // in : scaled LineVector of Line(s) = LinePoint + s * LineVector               
  const SmExtent1d     & crCurrentIvl,                   // in : current limits on s interval
  SmTArray<SmExtent1d> & rTrimIvls)                      // out: ParamSpace Intervals of line that map legally within the NaturalParamDomains
 const                                     
{ 
  // init output
  rTrimIvls.ReSet() ;

  // low work - unbounded transform
  if(FALSE == IsBounded())
    { rTrimIvls.Add(crCurrentIvl) ; }

  // locals
  ULONG      lNumFound ;
  double     dGap, dEnterParam, dExitParam ;
  SmBoolean  bSuccess ;
  SmPoint3d  sParamPoint, sParamVector ;
  SmExtent3d sParamDomain = GetNaturalParamDomain() ;
  double     dLen         = crInSpaceVector.Length() ;

  // check input - nonZero InSpaceVector
  if(SM_IS_ZERO(dLen))
    {
      SER_MSG(SM_ERR, _T("SmVolume::FindParamIntervalForInSpaceLineSimple bad input - zero length tangent vector")) ;
    }

  // check input - valid CurrentIvl
  if(crCurrentIvl.HasNegativeLength())
    {
      SER_MSG(SM_ERR, _T("SmVolume::FindParamIntervalForInSpaceLineSimple bad input - neg length input interval")) ;
    }

  // map InSpace props to ParamSpace
  InvOrientPoint (crInSpacePoint, bSuccess, sParamPoint, dGap) ;
  InvOrientVector(crInSpaceVector, crInSpacePoint, sParamVector) ;

  // intersect input line with ParamDomain - supports any mixture of Unbounded sParamDomain extents
  sParamDomain.IntersectLine(sParamPoint,  // in : point on line
                             sParamVector, // in : vector defining line's direction
                             lNumFound,    // out: number of intersections 
                                           //      0 = there is no intersection.  
                                           //      1 = grazes a corner.
                                           //      2 = portion of ray is inside box.
                             dEnterParam,  // out: entering ray parameter of ray/box xsect
                             dExitParam) ; // out: exiting  ray parameter of ray/box xsect

  // when ray intersects the ParamDomain - Add result to output
  if(   lNumFound == 1
     || lNumFound == 2)
    {
      // Build ivl for ParamLine portion in the ParamDomain
      SmExtent1d sIvl ;
      if(dEnterParam < dExitParam) { sIvl.SetMinMax(dEnterParam, dExitParam) ; }
      else                         { sIvl.SetMinMax(dExitParam, dEnterParam) ; }

      // get the portion of the input Ivl in the ParamDomain
      sIvl.Intersect(crCurrentIvl, sIvl) ;

      // add the portion to the output
      rTrimIvls.Add(sIvl) ;
    }

  // all done
  return(SM_SUCCESS) ; 

} // end SmVolume::FindParamIntervalForInSpaceLineSimple

/*******************************************************************//**
PURPOSE: Trim ParamSpace BBox to Natural Parameter Domain

NOTES: implement if:[DerivedMap ParamDomain not rectilinear)
***********************************************************************/
SmStatus SmVolume::TrimParamBoundingBoxSimple // rtn: SM_ERR when ParamBox trims to empty set, else SM_SUCCESS
 (const SmExtent3d & crParamBox,              // in : Tgt Param Box to trim
  SmExtent3d       & rParamTrimBox)           // out: Box trimmed to Natural Param Domain
 const                                        
{ 
  // for bounded Natrual Param Domains
  if(IsBounded())
    {
      // locals
      SmExtent3d         sBox ;
      const SmExtent3d * pBox ;
      SmExtent3d         sParamDomain = GetNaturalParamDomain() ;

      // when crParamBox and rParamTrimBox are the same - copy crParamBox
      if(&crParamBox == &rParamTrimBox) { sBox = crParamBox ; pBox = &sBox ;}
      else                              { pBox = &crParamBox ; }

      // set output TrimBox = XSect(ParamBox, NaturalDomainBox)
      pBox->Intersect(sParamDomain, rParamTrimBox) ; 
    }

  // else Unbounced Natural Param Domain - all BBoxes are in param space - let TrimBox = ParamBox

  // when ParamBox and ParamTrimBox are different - copy ParamBox into TrimBox 
  else if(&crParamBox != &rParamTrimBox) 
    { 
      rParamTrimBox = crParamBox ; 
    }

  // all done
  return(SM_SUCCESS) ; 

} // end SmVolume::TrimParamBoundingBoxSimple

/*******************************************************************//**
PURPOSE: Default virtual implementation of SmVolume Outspace Translate

NOTES: 1. This method is only called by SmVolume::Translate()
       2. This method is only called on the last compounding
          map of the original target map.
       3. This method builds and concatenates to the end of the
          of the compounding map list an SmTransform::Translate map.

       4. Derived classes can improve evaluation performance by
          implementing this virtual method to edit their SimpleMap's 
          parameters so that:

            Translate(Orient(SimpleMap(ParamPt))) = Orient(ModifiedSimpleMap(ParamPt)) and
            InvOrient(Translate(Orient(SimpleMap(ParamPt)))) = ModifiedSimpleMap(ParamPt)

          rather than using this default behavior of
          appending another SmTransform to the compound Map list.
***********************************************************************/
SmStatus SmVolume::TranslateSimple             
 (const SmVector3d  & crOutTranslate)   // in : OutSpace translation vector
{
  // arrive here when this volume needs to be Translated and
  // its virtual method TranslateSimple() doesn't handle this case.

  SM_ASSERT_MSG(m_pNextMap == NULL, _T("SmVolume::TranslateSimple not called on the last mapping in the compound map list (m_pNextMap != NULL)")) ; 

  // Create a Translate SmTransform to compound with current mapping
  SmTransform sTransform ;  // identity map with no Orient map
  SER(sTransform.TranslateSimple(crOutTranslate)) ;

  // Add mirror map to end of compound map list
  SER(this->AddNextMap(&sTransform, 1)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::TranslateSimple

/*******************************************************************//**
PURPOSE: Default virtual implementation of SmVolume OutSpace Mirror

NOTES: 1. This method is only called by SmVolume::Mirror()
       2. This method is only called on the last compounding
          map of the original target map.
       3. This method builds and concatenates to the end of the
          of the compounding map list an SmTransform::Mirror map.

       4. Derived classes can improve evaluation performance by
          implementing this virtual method to edit their SimpleMap's 
          parameters so that:

            Mirror(Orient(SimpleMap(ParamPt))) = Orient(ModifiedSimpleMap(ParamPt)) and
            InvOrient(Mirror(Orient(SimpleMap(ParamPt)))) = ModifiedSimpleMap(ParamPt)

          rather than using this default behavior of
          appending another SmTransform to the compound Map list.
***********************************************************************/
SmStatus SmVolume::MirrorSimple            
 (const SmPoint3d  & crPlaneOutPt,      // in : OutSpace Pt on Mirror Plane
  const SmVector3d & crPlaneOutNormal)  // in : OutSpace Normal to Mirror Plane
{
  // arrive here when this volume needs to be Mirrored and
  // its virtual method MirrorSimple() doesn't handle this case.

  SM_ASSERT_MSG(m_pNextMap == NULL, _T("SmVolume::MirrorSimple not called on the last mapping in the compound map list (m_pNextMap != NULL)")) ; 

  // Create a Mirror SmTransform to compound with current mapping
  SmTransform sTransform ;  // identity map with no Orient map
  SER(sTransform.MirrorSimple(crPlaneOutPt, crPlaneOutNormal)) ;

  // Add mirror map to end of compound map list
  SER(this->AddNextMap(&sTransform, 1)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::MirrorSimple

/*******************************************************************//**
PURPOSE: Default virtual implementation of SmVolume OutSpace Scale

NOTES: 1. This method is only called by SmVolume::Scale()
       2. This method is only called on the last compounding
          map of the original target map.
       3. This method builds and concatenates to the end of the
          of the compounding map list an SmTransform::Scale map.

       4. Derived classes can improve evaluation performance by
          implementing this virtual method to edit their SimpleMap's 
          parameters so that:

            Scale(Orient(SimpleMap(ParamPt))) = Orient(ModifiedSimpleMap(ParamPt)) and
            InvOrient(Scale(Orient(SimpleMap(ParamPt)))) = ModifiedSimpleMap(ParamPt)

          rather than using this default behavior of
          appending another SmTransform to the compound Map list.
***********************************************************************/
SmStatus SmVolume::ScaleSimple            
 (const SmVector3d & crScaleOutVec,   // in : OutSpace scale factors
  const SmPoint3d  * cpOptOutCenter)  // in : Optional OutSpace scaling center point
{
  // arrive here when this volume needs to be Scaleed and
  // its virtual method ScaleSimple() doesn't handle this case.

  SM_ASSERT_MSG(m_pNextMap == NULL, _T("SmVolume::ScaleSimple not called on the last mapping in the compound map list (m_pNextMap != NULL)")) ; 

  // Create a Scale SmTransform to compound with current mapping
  SmTransform sTransform ;  // identity map with no Orient map
  SER(sTransform.ScaleSimple(crScaleOutVec, cpOptOutCenter)) ;

  // Add mirror map to end of compound map list
  SER(this->AddNextMap(&sTransform, 1)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::ScaleSimple

/*******************************************************************//**
PURPOSE: Default virtual implementation of SmVolume OutSpace RotateAboutAxis
         which rotates OutSpace about an axis through the OutSpace origin.

NOTES: 1. This method is only called by SmVolume::RotateAboutAxis()
       2. This method is only called on the last compounding
          map of the original target map.
       3. This method builds and concatenates to the end of the
          of the compounding map list an SmTransform::RotateAboutAxis map.

       4. Derived classes can improve evaluation performance by
          implementing this virtual method to edit their SimpleMap's 
          parameters so that:

            RotateAboutAxis(Orient(SimpleMap(ParamPt))) = Orient(ModifiedSimpleMap(ParamPt)) and
            InvOrient(RotateAboutAxis(Orient(SimpleMap(ParamPt)))) = ModifiedSimpleMap(ParamPt)

          rather than using this default behavior of
          appending another SmTransform to the compound Map list.
***********************************************************************/
SmStatus SmVolume::RotateAboutAxisSimple            
 (double             dAngRad,    // in : OutSpace rotation AngRad 
  const SmVector3d & crOutAxis)  // in : OutSpace rotation axis through OutSpace origin
{
  // arrive here when this volume needs to be RotateAboutAxised and
  // its virtual method RotateAboutAxisSimple() doesn't handle this case.

  SM_ASSERT_MSG(m_pNextMap == NULL, _T("SmVolume::RotateAboutAxisSimple not called on the last mapping in the compound map list (m_pNextMap != NULL)")) ; 

  // Create a RotateAboutAxis SmTransform to compound with current mapping
  SmTransform sTransform ;  // identity map with no Orient map
  SER(sTransform.RotateAboutAxisSimple(dAngRad, crOutAxis)) ;

  // Add mirror map to end of compound map list
  SER(this->AddNextMap(&sTransform, 1)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::RotateAboutAxisSimple

/*******************************************************************//**
PURPOSE: Default virtual implementation of SmVolume OutSpace RotateAboutAxisAtPoint
         which rotates OutSpace about an axis through a given OutSpace point.

NOTES: 1. This method is only called by SmVolume::RotateAboutAxisAtPoint()
       2. This method is only called on the last compounding
          map of the original target map.
       3. This method builds and concatenates to the end of the
          of the compounding map list an SmTransform::RotateAboutAxisAtPoint map.

       4. Derived classes can improve evaluation performance by
          implementing this virtual method to edit their SimpleMap's 
          parameters so that:

            RotateAboutAxisAtPoint(Orient(SimpleMap(ParamPt))) = Orient(ModifiedSimpleMap(ParamPt)) and
            InvOrient(RotateAboutAxisAtPoint(Orient(SimpleMap(ParamPt)))) = ModifiedSimpleMap(ParamPt)

          rather than using this default behavior of
          appending another SmTransform to the compound Map list.
***********************************************************************/
SmStatus SmVolume::RotateAboutAxisAtPointSimple            
 (double             dAngRad,     // in : OutSpace rotation AngRad 
  const SmPoint3d  & crOutOrigin, // in : OutSpace point on rotation axis
  const SmVector3d & crOutAxis)   // in : OutSpace rotation axis direction
{
  // arrive here when this volume needs to be RotateAboutAxisAtPointed and
  // its virtual method RotateAboutAxisAtPointSimple() doesn't handle this case.

  SM_ASSERT_MSG(m_pNextMap == NULL, _T("SmVolume::RotateAboutAxisAtPointSimple not called on the last mapping in the compound map list (m_pNextMap != NULL)")) ; 

  // Create a RotateAboutAxisAtPoint SmTransform to compound with current mapping
  SmTransform sTransform ;  // identity map with no Orient map
  SER(sTransform.RotateAboutAxisAtPointSimple(dAngRad, crOutOrigin, crOutAxis)) ;

  // Add mirror map to end of compound map list
  SER(this->AddNextMap(&sTransform, 1)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::RotateAboutAxisAtPointSimple

/*******************************************************************//**
PURPOSE: Default virtual implementation of SmVolume OutSpace Transform

NOTES: 1. This method is only called by SmVolume::Transform()
       2. This method is only called on the last compounding
          map of the original target map.
       3. This method builds and concatenates to the end of the
          of the compounding map list an SmTransform::Transform map.

       4. Derived classes can improve evaluation performance by
          implementing this virtual method to edit their SimpleMap's 
          parameters so that:

            Transform(Orient(SimpleMap(ParamPt))) = Orient(ModifiedSimpleMap(ParamPt)) and
            InvOrient(Transform(Orient(SimpleMap(ParamPt)))) = ModifiedSimpleMap(ParamPt)

          rather than using this default behavior of
          appending another SmTransform to the compound Map list.
***********************************************************************/
SmStatus SmVolume::TransformSimple            
 (const SmAxis2Placement & crOutRotateNMove,  // in : OutSpace Rotate and Move Transform 
  const SmVector3d       * cpOptOutScale)     // in : Optional OutSpace scale factors applied after Rotate and Move
{
  // arrive here when this volume needs to be Transformed and
  // its virtual method TransformSimple() doesn't handle this case.

  SM_ASSERT_MSG(m_pNextMap == NULL, _T("SmVolume::TransformSimple not called on the last mapping in the compound map list (m_pNextMap != NULL)")) ; 

  // Create a Transform SmTransform to compound with current mapping
  SmTransform sTransform ;  // identity map with no Orient map
  SER(sTransform.TransformSimple(crOutRotateNMove, cpOptOutScale)) ;

  // Add mirror map to end of compound map list
  SER(this->AddNextMap(&sTransform, 1)) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::TransformSimple

/*******************************************************************//**
PURPOSE: return TRUE if SimpleMap has any discontinuties, 
    else return FALSE for a totall continuous map

NOTES: 1. This Base class default implemenations always returns FALSE
          for an unbounded and continuous Simple Map.  Implement a derived
          verison for any SmVolume derived class that has discontinuities.

          Most maps have unbounded domains and are continuous throughout
          but SmBSplineVolume maps have bounded domains with internal discontinuities.

       2. The list of discontinuties is reported in the optional pOptDisconts argument.

       3. When bCalcGeometric == TRUE, a set of evaluations are made across every representational
               discontinuity to compute the actual geometric dicsontinuities.
          When bCalcGeometric == FALSE, the representational discontinuities are reported.
               For BSplines, careful placement of the control points can turn a C0 representational
               continuity into something more continuous such as G1, C1, G2, or more.  
                
          Representational discontinuities may be more conservative than the geometric discontinuties 
          but are very cheap to compute.  If all you need to know is that a discontinuity exists
          always choos bCalcGeometric=FALSE.  If you actually need to know the current discontinuity
          at a boundary then pay the price and use bCalcGeometric = TRUE.
***********************************************************************/
SmBoolean SmVolume::HasDiscontinuitiesSimple   // rtn: TRUE if Simple map has internal C1 discontinuities
 (SmDiscontinuities3d *pOptDisconts,           // out: Opt list of ParamSpace discontinuities and summary data, NULL to ignore
                                               //      default:[NULL]
  SmBoolean bCalcGeometric)                    // NotUsed: in : TRUE = expensive - use geometric testing to compute actal geometric discontinuities
 const                                         //      FALSE= cheap - report representational discontinuites
                                               //      default:[TRUE]
{ 
  SM_REF1(bCalcGeometric) ;
  // set output to be unbounded and continuous
  if(pOptDisconts) 
    { pOptDisconts->Init() ; }

  // all done - no discontinuities
  return(FALSE) ;

} // end SmVolume::HasDiscontinuitiesSimple

/*******************************************************************//**
PURPOSE: Determine if a volume is closed in ProjSpace along the U, V, and W directions
         and optionally report the max level of continuity found in each direction.

NOTES:   
  A volume is closed in a direction when the associated direction's Min and Max
  isoSurfaces are coincident.  This is tested only by point sampling.

  eVolParam specifies which boundaries of the volume to test as
  SM_VP_U = check Pos(Umin,v,w) == Pos(Umax,v,w) at several v,w sample points
  SM_VP_V = check Pos(u,Vmin,w) == Pos(u,Vmax,w) at several u,w sample points
  SM_VP_W = check Pos(u,v,Wmin) == Pos(u,w,Vmax) at several u,v sample points

  peOptContinuity is set with the highlest level of continuity found at the sample points.
  SM_VP_U = check U direction derivatives at the ends of variable U isoparameter lines
  IS_VP_V = check V direction derivatives at the ends of variable V isoparameter lines
  IS_VP_W = check W direction derivatives at the ends of variable W isoparameter lines
  Set peOptContinuity == NULL to ignore.

  Currently continuity is only checked to C2/G2
***********************************************************************/
SmBoolean SmVolume::IsClosedSimple
 (SmBoolean        & rbClosedU,          // out: TRUE = closed in U direction, [check Pos[Umin,v,w] == Pos[Umax,v,w] for v,w samples
  SmBoolean        & rbClosedV,          // out: TRUE = closed in V direction, [check Pos[u,Vmin,w] == Pos[u,Vmax,w] for w,u samples
  SmBoolean        & rbClosedW,          // out: TRUE = closed in W direction, [check Pos[u,v,Wmin] == Pos[u,v,Wmax] for u,v samples
  double           * pdOptTolerance,     // out: pdOptTolerance = Tolerance to allow for check, NULL=dScaledZero(Volume), default:[NULL]
  const SmExtent3d * pOptParamDomain,    // in : ParamSpace domain over which to search for the inverse point
                                         //      NULL = use Map's NaturalDomain, default:[NULL]
  SmContinuityType * peOptContinuityU,   // out: U dir Continuity when closed, NULL to igmore, default:[NULL] 
  SmContinuityType * peOptContinuityV,   // out: V dir Continuity when closed, NULL to igmore, default:[NULL]      
  SmContinuityType * peOptContinuityW)   // out: W dir Continuity when closed, NULL to igmore, default:[NULL]
                                         //      oneof SM_CT_DISCONTINUOUS      
                                         //            SM_CT_C0                 
                                         //            SM_CT_G1                 
                                         //            SM_CT_G1_G2              
                                         //            SM_CT_G1_G2_G3              
                                         //            SM_CT_C1                 
                                         //            SM_CT_C1_G2        
                                         //            SM_CT_C1_G2_G3        
                                         //            SM_CT_C1_C2        
                                         //            SM_CT_C1_G3        
                                         //            SM_CT_C1_C3        
 const
{
  // Sample density
#define NUM_TEST_POINTS 5

  // init output
  rbClosedU = TRUE ;
  rbClosedV = TRUE ;
  rbClosedW = TRUE ;
  if(peOptContinuityU) { *peOptContinuityU = SM_CT_CINFINITY ; }
  if(peOptContinuityV) { *peOptContinuityV = SM_CT_CINFINITY ; }
  if(peOptContinuityW) { *peOptContinuityW = SM_CT_CINFINITY ; }

  // locals
  ULONG dd, ii, jj ;
  ULONG              lDim  = 3 ;
  SmBoolean          bDone = FALSE;
  SmBoolean        * pbClosed ;
  SmContinuityType * pContinuity ;
  SmPoint3d          sPMin, sPMax, sPMid[3], sGMid[4] ;
  SmPoint3d          sUVWEval ;
  SmPoint3d          sUVWMin, sUVWMax, sUVWMid, sParamDir ;
  SmVector3d         sDUMin (0,0,0), sDUMax (0,0,0) ;
  SmVector3d         sDVMin (0,0,0), sDVMax (0,0,0) ;
  SmVector3d         sDWMin (0,0,0), sDWMax (0,0,0) ;
  SmVector3d         sDUUMin(0,0,0), sDUUMax(0,0,0) ;
  SmVector3d         sDVVMin(0,0,0), sDVVMax(0,0,0) ;
  SmVector3d         sDWWMin(0,0,0), sDWWMax(0,0,0) ;
  SmVector3d         sDUVMin(0,0,0), sDUVMax(0,0,0) ;
  SmVector3d         sDUWMin(0,0,0), sDUWMax(0,0,0) ;
  SmVector3d         sDVWMin(0,0,0), sDVWMax(0,0,0) ;
  SmExtent3d         sNaturalDomain = GetNaturalParamDomain() ;
  SmExtent3d       * pParamDomain   = pOptParamDomain ? (SmExtent3d *)pOptParamDomain : &sNaturalDomain ;

  // for every param direction to be checked (lDim == 3 for UVW)
  for(dd=0; dd<lDim; dd++, bDone=FALSE)
    {
      // param direction
      SmVolumeParamType eThisParam =   (dd == 0) ? SM_VP_U
                                     : (dd == 1) ? SM_VP_V
                                     :             SM_VP_W ;
      if     (eThisParam == SM_VP_U) { sParamDir.Set(1.0, 0.0, 0.0) ;
                                       pContinuity = peOptContinuityU ;
                                       pbClosed    = &rbClosedU ; 
                                       // low work: assume unbounded intevals are open
                                       if(FALSE == pParamDomain->GetUInterval().IsBounded())
                                         { rbClosedU = FALSE ; 
                                           if(pContinuity) { *pContinuity = SM_CT_DISCONTINUOUS ; }
                                           continue ; 
                                         }  
                                     }
      else if(eThisParam == SM_VP_V) { sParamDir.Set(0.0, 1.0, 0.0) ;
                                       pContinuity = peOptContinuityV ;
                                       pbClosed    = &rbClosedV ; 
                                       // low work: assume unbounded intevals are open
                                       if(FALSE == pParamDomain->GetVInterval().IsBounded())
                                         { rbClosedV = FALSE ; 
                                           if(pContinuity) { *pContinuity = SM_CT_DISCONTINUOUS ; }
                                           continue ; 
                                         }  
                                     }
      else                           { sParamDir.Set(0.0, 0.0, 1.0) ;
                                       pContinuity = peOptContinuityW ;
                                       pbClosed    = &rbClosedW ; 
                                       // low work: assume unbounded intevals are open
                                       if(FALSE == pParamDomain->GetWInterval().IsBounded())
                                         { rbClosedW = FALSE ; 
                                           if(pContinuity) { *pContinuity = SM_CT_DISCONTINUOUS ; }
                                           continue ; 
                                         }  
                                     }

      // for every sample point
      for (ii=0; ii<=NUM_TEST_POINTS && !bDone; ii++)
        {
          double dParamI = (ii+1.0)/(2.0+NUM_TEST_POINTS) ;       // dParamI = [1/7 2/7 3/7 4/7 5/7 6/7]

          // for every sample point
          for (jj=0; jj<=NUM_TEST_POINTS && !bDone; jj++)
            {
              double dParamJ = (jj+1.0)/(2.0+NUM_TEST_POINTS) ;   // dParamJ = [1/7 2/7 3/7 4/7 5/7 6/7]

              // get next UVWPoint on UVWDomain - avoid endPoints
              sUVWEval.Set(dParamI,                       // dd == 0: u=NotUsed, v=dParamI, w=dParamJ 
                           dd<2 ? dParamI : dParamJ,      // dd == 1: u=dParamI, v=NotUsed, w=dParamJ
                           dParamJ) ;                     // dd == 2: u=dParamI, v=dParamJ, w=NotUsed

              SmPoint3d sUVW = pOptParamDomain->Evaluate( sUVWEval.x, sUVWEval.y, sUVWEval.y );

              // set sUVWMin/sUVWMax to be requested isoParameters lines endPoints
              if     (eThisParam == SM_VP_U) { // testing U parameter
                                               sUVWMin.x = pOptParamDomain->GetMin().x ;
                                               sUVWMax.x = pOptParamDomain->GetMax().x ;
                                               sUVWMid.x = (sUVWMin.x + sUVWMax.x) / 2.0 ;
                                               sUVWMin.y = sUVWMax.y = sUVWMid.y = sUVW.y ;  // 1/7 1-6/7
                                               sUVWMin.z = sUVWMax.z = sUVWMid.z = sUVW.z ;  // 2/7 1-6/7
                                             }
              else if(eThisParam == SM_VP_V) { // testing V parameter
                                               sUVWMin.y = pOptParamDomain->GetMin().y ;
                                               sUVWMax.y = pOptParamDomain->GetMax().y ;
                                               sUVWMid.y = (sUVWMin.y + sUVWMax.y) / 2.0 ;
                                               sUVWMin.x = sUVWMax.x = sUVWMid.x = sUVW.x ;
                                               sUVWMin.z = sUVWMax.z = sUVWMid.z = sUVW.z ;
                                             }
              else                           { // testing W parameter
                                               sUVWMin.z = pOptParamDomain->GetMin().z ;
                                               sUVWMax.z = pOptParamDomain->GetMax().z ;
                                               sUVWMid.z = (sUVWMin.z + sUVWMax.z) / 2.0 ;
                                               sUVWMin.x = sUVWMax.x = sUVWMid.x = sUVW.x ;
                                               sUVWMin.y = sUVWMax.y = sUVWMid.y = sUVW.y ;
                                             }

              // evaluate isoParameter endPoints
              if(pContinuity != NULL)
                {
                  // check position and higher order derivatives
                  Evaluate2ndDerivativesSimple( sUVWMin, TRUE, TRUE, TRUE,
                                                sPMin, sDUMin, sDVMin, sDWMin,
                                                sDUUMin, sDVVMin, sDWWMin, sDUVMin, sDUWMin, sDVWMin );
                  Evaluate2ndDerivativesSimple( sUVWMax, TRUE, TRUE, TRUE,
                                                sPMax, sDUMax, sDVMax, sDWMax,
                                                sDUUMax, sDVVMax, sDWWMax, sDUVMax, sDUWMax, sDVWMax );
                }
              else // just check position
                {
                  EvaluatePointSimple( sUVWMin, sPMin );
                  EvaluatePointSimple( sUVWMax, sPMax );
                }

#ifdef SM_DEBUG_CODE
#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE ;
              // draw EndPoints(red)
              if(bDebugMe)
                {
                  if(dd == 0 && ii == 0 && jj == 0)
                    { smgfx_Erase() ; 
                      smgfx_SetLook(1,2, 0,0,1) ; Draw(TRUE, TRUE) ; sm_GraphicsLoop() ;
                    }
                  smgfx_SetLook(3,4, 1,0,0) ; sPMin.Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(3,6, 0,1,0) ; sPMax.Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_GFX_CODE
#endif // SM_DEBUG_CODE

              // pick a tolerance
              double dScaledZero = SM_EFF_ZERO * 1000.0 * (1.0 + sPMax.GetMaxDimension() + sPMin.GetMaxDimension());
              double dTol        =   (pdOptTolerance)
                                   ? *pdOptTolerance
                                   : dScaledZero;

              // get the startPoint/endPoint gap
              SmVector3d sVecDiff = sPMin - sPMax ;

              // when the gap is too big - not closed
              if ( sVecDiff.LengthSquared() > dTol*dTol )
                {
                  *pbClosed = FALSE ;
                  if(pContinuity) { *pContinuity = SM_CT_DISCONTINUOUS ; } 
                  bDone = TRUE ;
                  break ;
                }
              else if(pContinuity != NULL)
                {
                  SmContinuityType eCurveContinuity ;
                  if     (dd == 0) 
                    { smgu_EvaluateCurveContinuity(sPMin, sDUMin, sDUUMin,
                                                   sPMax, sDUMax, sDUUMax,
                                                   eCurveContinuity) ;
                    }
                  else if(dd == 1)
                    { smgu_EvaluateCurveContinuity(sPMin, sDVMin, sDVVMin,
                                                   sPMax, sDVMax, sDVVMax,
                                                   eCurveContinuity) ;
                    }
                  else
                    { smgu_EvaluateCurveContinuity(sPMin, sDWMin, sDWWMin,
                                                   sPMax, sDWMax, sDWWMax,
                                                   eCurveContinuity) ;
                    }

                  // save lowest found continuity
                  if(eCurveContinuity < *pContinuity) { *pContinuity = eCurveContinuity ; }
                }

              // gwc: extra test
              if (*pContinuity > SM_CT_DISCONTINUOUS)
                {

                  // filter out very short curve passing as closed curves
                  EvaluateDirectionalDerivsSimple( sUVWMid, sParamDir, 2, sPMid );

                  SmVector3d sGapDiff1 = sPMid[0] - sPMin ;
                  SmVector3d sGapDiff2 = sPMid[0] - sPMax ;
                  double     dGap1Sq   = sGapDiff1.LengthSquared() ;
                  double     dGap2Sq   = sGapDiff2.LengthSquared() ;
                  double     dMidGapSq = smos_Max(dGap1Sq, dGap2Sq) ;

                  // cheesy one point test for a short curve
                  if (dMidGapSq < 4.0*dTol*dTol)
                    {
                      // but some short curves can be very small circles - watch for those

                      // get geometric properties from directional derivatives
                      smgu_FrenetFromDerivatives(sPMid, 3, sGMid) ;

                      // get curvature and radius of curvature
                      double dMidCurvSq = sGMid[2].LengthSquared() ;
                      double dMidRadius = (dMidCurvSq > SM_EFF_ZERO_SQ) ? 1.0 / smos_Sqrt(dMidCurvSq) : SM_BIG_DOUBLE ;

                      // When the radius is larger (small circles can be closed - but short large radius arcs can't be
                      if (dMidRadius > dTol)
                        {
                          if ( pContinuity != NULL )
                            { *pContinuity = SM_CT_DISCONTINUOUS ; }

                          return FALSE;
                        } // end curve not a small circle check
                    } // end curve estimated to be a short arc check
                } // end still checking for C0 continuity check
            } // end iter jj, every sample point
        } // end iter ii, every sample point
    } // end iter dd, every param direction - checking for closure

  // when all test endPoint gaps pass tolerance test - surface is closed
  return (   rbClosedU
          || rbClosedV
          || rbClosedW) ;

#undef NUM_TEST_POINTS

} // end SmVolume::IsClosedSimple

/*******************************************************************//**
PURPOSE: Determine if a simple volume is periodic with G1 continuity or higher

NOTES: calls IsClosedSimple()
***********************************************************************/
SmBoolean SmVolume::IsPeriodicSimple
 (SmBoolean        & rbPeriodicU,      // out: TRUE = G1 or better in U Dir 
  SmBoolean        & rbPeriodicV,      // out: TRUE = G1 or better in V Dir
  SmBoolean        & rbPeriodicW,      // out: TRUE = G1 or better in W Dir
  const SmExtent3d * pOptParamDomain)  // in : ParamSpace domain over which to search for the inverse point
 const                                 //      NULL = use Map's NaturalDomain, default:[NULL]
{
  // locals
  SmBoolean        bClosedU,     bClosedV,     bClosedW ;          
  SmContinuityType eContinuityU, eContinuityV, eContinuityW ;

  // pass the call along
  IsClosedSimple(bClosedU, bClosedV, bClosedW,
                 NULL, 
                 pOptParamDomain,
                 &eContinuityU, &eContinuityV, &eContinuityW) ;

  // set output
  rbPeriodicU = (eContinuityU >= SM_CT_G1) ;
  rbPeriodicV = (eContinuityV >= SM_CT_G1) ;
  rbPeriodicW = (eContinuityW >= SM_CT_G1) ;

  // all done
  return FALSE ;

} // end SmVolume::IsPeriodicSimple

/*******************************************************************//**
PURPOSE: Determine if a point is a singluar point in ProjSpace
         and get the singular direction(s).

NOTES: 
    Return: TRUE    = dW/du, dW/dv, or dW/dw (1st partial derivative in U, V, or W directions)
                      is zero.
            FALSE   = All 1st partial derivatives are non-zero.
    Output: rbSingularU: TRUE = U direction derivative is  zero 
                         FALSE= U direction derivative NOT zero.
            rbSingularV: TRUE = V direction derivative is  zero 
                         FALSE= V direction derivative NOT zero.
            rbSingularW: TRUE = W direction derivative is  zero 
                         FALSE= W direction derivative NOT zero.
***********************************************************************/
SmBoolean SmVolume::IsSingularitySimple
  (const SmPoint3d & crParamPoint,    // in : Volume ParamSpace Point to test
   SmBoolean       & rbSingularU,     // out: TRUE = Singular [Wu=0] in U parameter direction
   SmBoolean       & rbSingularV,     // out: TRUE = Singular [Wv=0] in V parameter direction
   SmBoolean       & rbSingularW,     // out: TRUE = Singular [Ww=0] in W parameter direction
   double            d3dTol)          // NotUsed: in : min dist between distinct 3d points
 const
{
  SM_REF1(d3dTol) ;
  // init output
  rbSingularU = FALSE ;
  rbSingularV = FALSE ;
  rbSingularW = FALSE ;

  // get first derivatives in ProjSpace
  SmVector3d sMat[2][2][2];
  SER(EvaluateSimple(crParamPoint,1,TRUE,TRUE,TRUE,sMat[0][0]));

  double dLengSqDU = sMat[1][0][0].LengthSquared();
  double dLengSqDV = sMat[0][1][0].LengthSquared();
  double dLengSqDW = sMat[0][0][1].LengthSquared();

  // when any 1st derivative is small - do further classification
  if(   dLengSqDU < SM_EFF_ZERO_SQRT
     || dLengSqDV < SM_EFF_ZERO_SQRT
     || dLengSqDW < SM_EFF_ZERO_SQRT) 
    {
      // Do further tests to qualify
      double dLengDU = smos_Sqrt(dLengSqDU);
      double dLengDV = smos_Sqrt(dLengSqDV);
      double dLengDW = smos_Sqrt(dLengSqDW);

      SmPoint3d sDomSize = GetNaturalParamDomain().GetSize();
      dLengDU = dLengDU * sDomSize.x;
      dLengDV = dLengDV * sDomSize.y;
      dLengDW = dLengDW * sDomSize.z;

      // check for UDir Singularity
      if(    dLengDU < dLengDV * SM_EFF_ZERO_SQRT
          || dLengDU < dLengDW * SM_EFF_ZERO_SQRT
          || dLengDU < SM_EFF_ZERO * 100.0) 
        {
          rbSingularU = TRUE ;
        }

      // check for VDir Singularity
      if(    dLengDV < dLengDU * SM_EFF_ZERO_SQRT
          || dLengDV < dLengDW * SM_EFF_ZERO_SQRT
          || dLengDV < SM_EFF_ZERO * 100.0) 
        {
          rbSingularV = TRUE ;
        }

      // check for WDir Singularity
      if(    dLengDW < dLengDU * SM_EFF_ZERO_SQRT
          || dLengDW < dLengDV * SM_EFF_ZERO_SQRT
          || dLengDW < SM_EFF_ZERO * 100.0) 
        {
          rbSingularW = TRUE ;
        }
    } // end small 1st deriv check

  // all done
  return (rbSingularU || rbSingularV || rbSingularW) ;

} // end SmVolume::IsSingularitySimple

/*******************************************************************//**
PURPOSE: Test Point to see if it is on given ParamDomainBoundary
            within the volume natural ParamDomain.

NOTES:
***********************************************************************/
SmBoolean SmVolume::IsOnBoundarySimple
  (const SmPoint3d    & crParamPoint,    // in : Volume UVWPoint to test
   SmBoolean          & rbOnU,           // out: TRUE = TargetPoint on UMin or UMax
   SmBoolean          & rbOnV,           // out: TRUE = TargetPoint on VMin or VMax
   SmBoolean          & rbOnW,           // out: TRUE = TargetPoint on WMin or WMax
   double             * pdOptTolerance,  // in : max deviation allowed for point on seam
                                         //      NULL = use SM_EFF_ZERO * 1000 * (1 + maxDimension())
  const SmExtent3d    * pOptParamDomain) // in : ParamSpace domain over which to search for the inverse point
 const                                   //      NULL = use Map's NaturalDomain, default:[NULL]
{
  // pick a tolerance
  SmExtent3d sParamDomain = pOptParamDomain ? *pOptParamDomain : GetNaturalParamDomain() ; 
  double     dTol         =   (pdOptTolerance)
                            ? *pdOptTolerance
                            : SM_EFF_ZERO * 1000.0 * (1.0 + sParamDomain.GetMaxDimension()) ;
  SmVector3d sBinorm ;

  // pass the call along
  SmBoolean bRtn = sParamDomain.IsPoint3dOnBoundary( crParamPoint, dTol, &sBinorm) ;

  // set output
  rbOnU = !(sBinorm.x == 0.0) ;
  rbOnV = !(sBinorm.y == 0.0) ;
  rbOnW = !(sBinorm.z == 0.0) ;

  // return boundary value while making sure point is in natural domain
  return(bRtn) ;

} // end SmVolume::IsOnBoundarySimple

/*******************************************************************//**
PURPOSE: Write SmVolume to output stream

NOTES: 
***********************************************************************/
SmStatus SmVolume::WriteToFile
  (const TCHAR * cOutputFileName,      // in : target file name                           
   SmBoolean     bSkipHeaderWrite,     // NotUsed: in : TRUE = Omit "Volume Type: TYPE" header label
   SmBoolean     bNewFile,             // in : TRUE = open file and rewrite contents      
                                       //      FALSE= open file and append to end 
                                       //      default:[FALSE]
   SmBoolean     bWriteAttributes)     // in : TRUE=Write attributes, FALSE=don't
                                       //      default:[FALSE]
  const        
{ 
  SM_REF1(bSkipHeaderWrite) ;
  SM_ASSERT_VALID( this );
  TCHAR        sBuff[SM_TBLOCK_SIZE] ; 
  SmDatabaseIOFile sDB ;
  long         lDBVersionNumber = SM_CURRENT_DATABASE_VERSION ;

  // open file for output
  if(SM_SUCCESS != sDB.OpenFileForWrite(cOutputFileName, // in : target file name
                                        SM_ASCII,        // in : oneof: SM_ASCII    = write ascii file with comment lines
                                                         //             SM_BINARY   = write binary file from system format to LittleEndian
                                                         //             SM_BYTESWAP = write binary file - forcing byte swap
                                                         //      default:[SM_ASCII]
                                        bNewFile))       // in : bNewFile = TRUE  = open file and rewrite contents
                                                         //                 FALSE = open file and append to end
                                                         //                 default:[FALSE]
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cOutputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }
  //ofstream & rFileOut = *sDB.GetOutStreamPtr();

  // Output type 
  SM_TYPE lType = GetType() ;
  sDB.WriteType(lType) ;
  
  // pass the call along to appropriate derived type
  WriteToDB(sDB, lDBVersionNumber) ;

  // handle attributes
  if(bWriteAttributes && lDBVersionNumber > 30)
    {
       const SmContext *cpContext = GetContext() ;

       if(cpContext != NULL)
         {
           SmTArray<ULONG>                     sThisAttributes ;
           SmTArray<SmAttribute*>              sAllAttributes ;
           SmMapTypeToType<SmAttribute*,ULONG> sAttrMap ; 
           SER(sm_ExtractAttributes(*cpContext,this,sThisAttributes,sAllAttributes,sAttrMap));
           SmAttributeData::WriteIndexedAttributesToDB(sThisAttributes,sDB);
         } // end volume has context check
    } // end write attribute check

  // all done
  return SM_SUCCESS;

} // end SmVolume::WriteToFile

/*******************************************************************//**
PURPOSE: Write Array Of Volumes to File

NOTES: 
***********************************************************************/
SmStatus SmVolume::WriteArrayToFile
  (const TCHAR                 * cOutputFileName,  // in : target file name                           
   const SmTArray<SmVolume *>  & rVolArr,          // in : array of volumes to write
   SmBoolean                     bWriteAttributes) // in : TRUE=Write attributes, FALSE=don't
                                                   //      default:[FALSE]
{
  SmDatabaseIOFile sDB ;
  TCHAR            sBuff[SM_TBLOCK_SIZE] ; 
  long             lDBVersionNumber = SM_CURRENT_DATABASE_VERSION ;

  // check input
  ULONG lVolCnt = rVolArr.GetSize();
  if (lVolCnt < 1) 
    { return  SM_ERR ; }

  // open file for output
  if(SM_SUCCESS != sDB.OpenFileForWrite(cOutputFileName, // in : target file name
                                        SM_ASCII,        // in : oneof: SM_ASCII    = write ascii file with comment lines
                                                         //             SM_BINARY   = write binary file from system format to LittleEndian
                                                         //             SM_BYTESWAP = write binary file - forcing byte swap
                                                         //      default:[SM_ASCII]
                                        TRUE))           // in : bNewFile = TRUE  = open file and rewrite contents
                                                         //                 FALSE = open file and append to end
                                                         //                 default:[FALSE]
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cOutputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }
  std::ostream & rFileOut = *sDB.GetOutStreamPtr();

  // output max index number
  lVolCnt--;
  rFileOut << lVolCnt << "\n" ;

  // output the volumes
  ULONG ii ;
  for(ii=0;ii<=lVolCnt;ii++) 
   { 
      SmVolume *pVol  = rVolArr[ii];

      // write volume type and dim
      SM_TYPE  lType = pVol->GetType() ;
      SER(sDB.WriteType(lType)) ;

      // write volume
      SER(pVol->WriteToDB(sDB, lDBVersionNumber)) ;

      // handle attributes
      if(bWriteAttributes && lDBVersionNumber > 30)
        {
           const SmContext *cpContext = pVol->GetContext() ;

           if(cpContext != NULL)
             {
               SmTArray<ULONG>                     sThisAttributes ;
               SmTArray<SmAttribute*>              sAllAttributes ;
               SmMapTypeToType<SmAttribute*,ULONG> sAttrMap ; 
               SER(sm_ExtractAttributes(*cpContext,pVol,sThisAttributes,sAllAttributes,sAttrMap));
               SmAttributeData::WriteIndexedAttributesToDB(sThisAttributes,sDB);
             } // end volume has context check
        }
   } // end iter writing every volume

  // all done
  return SM_SUCCESS;

} // end SmVolume::WriteArrayToFile

/*******************************************************************//**
PURPOSE: Read a Volume in from file.  

NOTES: This method is primarily for debugging.
***********************************************************************/
SmStatus SmVolume::ReadFromFile
  (const SmContext & crContext,          // in : context for new object construction
   const TCHAR     * cInputFileName,     // in : target file
   SmVolume       *& rpNewVolume,        // out:    NULL on input = new object allocated in this routine built from stream data
                                         //      NotNULL on input = pointer to an empty object to be filled by this routine
   const ULONG       lFileOffsetInBytes) // NotUsed: in : Number of characters in file to skip before reading
                                         //      default:[0]
{
  SM_REF1(lFileOffsetInBytes) ;
  SmDatabaseIOFile sDB ;
  TCHAR            sBuff[SM_TBLOCK_SIZE] ;
  SM_TYPE          lType ;

  // build and init the stream structure
  if(SM_SUCCESS != sDB.OpenFileForRead(cInputFileName, SM_ASCII))
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cInputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }
  //ifstream & rFileIn = *sDB.GetInStreamPtr();

  ULONG lDBVersionNumber = SM_CURRENT_DATABASE_VERSION  ; 

  // get Volume type
  sDB.ReadType(lType) ;

  // pass the call along to appropriate derived type
  SmVolume::ReadFromDB(lType, sDB, crContext, rpNewVolume, lDBVersionNumber) ;
  SM_ASSERT_VALID(rpNewVolume) ;

  // all done
  return SM_SUCCESS;

} // end SmVolume::ReadFromFile - for single volume

/*******************************************************************//**
PURPOSE: Read an Array of Volumes in from a file.  

NOTES: This method is primarily for use when debugging.
***********************************************************************/
SmStatus SmVolume::ReadArrayFromFile
  (const SmContext        & crContext,        // in : context for new object construction
   const TCHAR            * cInputFileName,   // in : target file
   SmTArray <SmVolume *>  & rNewVolumes,      // out: the read volume with newly allocated memory
   const ULONG            lFileOffsetInBytes, // NotUsed: in : Number of characters in file to skip before reading)
                                              //      default:[0]       
   SmTArray <SmVolume *> * pTestVolumes)      // in : for debug only - the array of curves expected to be read 
                                              //      NULL to ignore. Default:[NULL]
{
  SM_REF2(lFileOffsetInBytes, pTestVolumes);

  SmDatabaseIOFile sDB;
  TCHAR            sBuff[SM_TBLOCK_SIZE] ;
  SM_TYPE          lType ;
  ULONG            lVolCnt ;
  ULONG            lDBVersionNumber = SM_CURRENT_DATABASE_VERSION ;
  SmVolume       * pNewVolume = NULL ;

  // init output
  rNewVolumes.ReSet() ;

  // build and init the stream structure
  if(SM_SUCCESS != sDB.OpenFileForRead(cInputFileName, SM_ASCII))
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cInputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }
  std::istream & rFileIn = *sDB.GetInStreamPtr();

  // read max index number
  rFileIn >> lVolCnt ;

  // input the volumes
  ULONG ii ;
  for(ii=0;ii<=lVolCnt;ii++)
    {
      // read type
      sDB.ReadType(lType) ;

      // pass the call along to appropriate derived type
      SmVolume::ReadFromDB(lType, sDB, crContext, pNewVolume, lDBVersionNumber) ;
      SM_ASSERT_VALID(pNewVolume) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe)
        {
          SM_DUMP_AND_ASSERT_VALID(pNewVolume) ;

          if(pTestVolumes && ii < pTestVolumes->GetSize())
            {
              // compare the just read curve with the test curve
              SM_ASSERT_MSG( *pTestVolumes->GetAt(ii) == *pNewVolume,
                            _T("SmVolume::ReadFromFile - read volume is not the same as given comparison test volume.") ) ;
            }
        }
#endif // SM_DEBUG_CODE

      // add volume to output
      rNewVolumes.Add(pNewVolume) ; 
      pNewVolume = NULL ;

    } // end iter to read every Volume

  // all done
  return SM_SUCCESS ;

} // end SmVolume::ReadArrayFromFile

/*******************************************************************//**
PURPOSE: 

NOTES:
  1. make sure all derived type WriteToDB methods call the base class method
     after writing their own data to file.
***********************************************************************/
SmStatus SmVolume::WriteToDB
 (SmDatabaseIO & rDB,              // in : target output stream
  ULONG          lDBVersionNumber) // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType eType = rDB.GetFileType();

  // NextMap Header
  SmBoolean bHasNextMap = m_pNextMap != NULL ;
  if(eType == SM_ASCII) { std::ostream  & rFileOut = *rDB.GetOutStreamPtr() ;
                          rFileOut << bHasNextMap << " : 1 = with NextMap, 0 = without \n" ; 
                        }
  else                  { SER(rDB.WriteBoolean(bHasNextMap) ) ; }

  // when needed - output m_pNextMap
  if(bHasNextMap)
    {
      // recursion gets all NextMaps in the NextMap linked list
      SM_TYPE lNextType = m_pNextMap->GetType() ;
      rDB.WriteType(lNextType) ;
      m_pNextMap->WriteToDB(rDB, lDBVersionNumber) ;
    }

  // OrientMap Header
  SmBoolean bHasOrientMap = m_pOrientMap != NULL ;
  if(eType == SM_ASCII) { std::ostream  & rFileOut = *rDB.GetOutStreamPtr() ;
                          rFileOut << bHasOrientMap << " : 1 = with OrientMap, 0 = without \n" ; 
                        }
  else                  { SER(rDB.WriteBoolean(bHasOrientMap) ) ; }

  // when needed - output m_pOrientMap
  if(bHasOrientMap)
    {
      SM_TYPE lOrientType = m_pOrientMap->GetType() ;
      rDB.WriteType(lOrientType) ;
      m_pOrientMap->WriteToDB(rDB, lDBVersionNumber) ;
    }

  // don't write InvOrientMap - it's derived from OrientMap

  // all done
  return SM_SUCCESS;

} // end SmVolume::WriteToDB

/*******************************************************************//**
PURPOSE: To act like a virtual static method for reading any volume type.

NOTES: branches on lType and passes the read call along to the appropriate
       derived type volume and then reads the SmVolume class data.

1. make sure all derived type WriteToDB methods call the base class method
   after writing their own data to file.
***********************************************************************/
SmStatus SmVolume::ReadFromDB
 (SM_TYPE           lType,              // in : volume type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  const SmContext & crContext,          // in : context for new object construction
  SmVolume       *& rpNewVolume,        // out: new object allocated in this routine built from stream data
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{                                       
  // file type, ASCII or BINARY
  SmFileType eType = rDB.GetFileType();

  // First read the derived type data - note derived class ReadFromDB methods do NOT call SmVolume::ReadFromDB
  switch(lType)
    {
      case SmBSplineVolume_TYPE : { SER( SmBSplineVolume::ReadFromDB(lType, rDB, crContext, rpNewVolume, lDBVersionNumber) ) ; } 
                                  break ; 
      case SmBendVolume_TYPE    : { SER( SmBendVolume::   ReadFromDB(lType, rDB, crContext, rpNewVolume, lDBVersionNumber) ) ; }  
                                  break ; 
      case SmTransform_TYPE     : { SER( SmTransform::    ReadFromDB(lType, rDB, crContext, rpNewVolume, lDBVersionNumber) ) ; }  
                                  break ; 
      case SmUnbendVolume_TYPE  : { SER( SmUnbendVolume:: ReadFromDB(lType, rDB, crContext, rpNewVolume, lDBVersionNumber) ) ; }  
                                  break ; 
      case SmTwistVolume_TYPE   : { SER( SmTwistVolume::  ReadFromDB(lType, rDB, crContext, rpNewVolume, lDBVersionNumber) ) ; }  
                                  break ;
      case SmVolume_TYPE        :
        {
          // NextMap local
          SmBoolean bHasNextMap = FALSE ;

          // check for NextMap
          if(eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                                  rFileIn >> bHasNextMap ; rDB.GoToNextLine() ;
                                }
          else                  { SER(rDB.ReadBoolean(bHasNextMap)) ;
                                }

          rpNewVolume->m_pNextMap = NULL ;

          // when there is a NextMap
          if(bHasNextMap)
            {
              // read NextMap type
              SM_TYPE lNextType ;
              rDB.ReadType(lNextType) ;

              // read NextMap - recursion gets all NextMaps in the NextMap linked list
              SmVolume::ReadFromDB(lNextType, rDB, crContext, rpNewVolume->m_pNextMap, lDBVersionNumber) ;
              rpNewVolume->m_lNextOwnerFlag = 1 ; 

            } // end Has next map check

          // Orient Map local
          SmBoolean bHasOrientMap = FALSE ;

          // check for OrientMap
          if(eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                                  rFileIn >> bHasOrientMap ; rDB.GoToNextLine() ;
                                }
          else                  { SER(rDB.ReadBoolean(bHasOrientMap)) ;
                                }
          rpNewVolume->m_pOrientMap = NULL ;

          // when there is a OrientMap
          if(bHasOrientMap)
            {
              // read OrientMap type
              SM_TYPE lOrientType ;
              rDB.ReadType(lOrientType) ;

              // read OrientMap
              SmVolume *pVolume = NULL ;
              SmVolume::ReadFromDB(lOrientType, rDB, crContext, pVolume, lDBVersionNumber) ;
              rpNewVolume->m_pOrientMap = (SmTransform *)pVolume ;
              rpNewVolume->m_lOrientOwnerFlag = 1 ; 

              // set the inverse orient map
              rpNewVolume->RefreshInvOrientMap() ;

            } // end Has Orient map check

        } // end read SmVolume_TYPE
        break ;

      default : SER(SM_ERR) ; break ; 
    } // end switch on type

  // all done
  return(SM_SUCCESS) ;

} // end SmVolume::ReadFromDB

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmEllipse.

NOTES: Does include the volumes's attribute memory
***********************************************************************/
ULONG SmVolume::GetMemoryUsed       // rtn: smaller size of actually used memory in bytes
 (ULONG    & rlMemoryAllocated,     // out: bigger size of all allocated memory in bytes
  SmMarkType eMarkType)             // in : uses without increment eMarkType value
 const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + NextMap + OrientMap + InvOrientMap memory
  ULONG lNextMapAllocated      = (m_pNextMap   ? m_pNextMap->GetMemoryUsed(lNextMapAllocated)   : 0) ;           
  ULONG lOrientMapAllocated    = (m_pOrientMap ? m_pOrientMap->GetMemoryUsed(lOrientMapAllocated) : 0) ;         
  ULONG lInvOrientMapAllocated = (m_pInvOrientMap ? m_pInvOrientMap->GetMemoryUsed(lInvOrientMapAllocated) : 0) ;

  rlMemoryAllocated  =   sizeof(*this) + lNextMapAllocated + lOrientMapAllocated + lInvOrientMapAllocated;

  // + attribute memory
  ULONG lThisAllocated ;
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                                       eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // all done
  return(lUsed) ;

} // end SmVolume::GetMemoryUsed

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmVolume::IsKindOf( SM_TYPE t ) const
{
  return ((SmVolume_TYPE == t) ? TRUE : SmAObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmVolume::Dump() const 
{ 
  Dump(FALSE, 0) ; 
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmVolume::Dump
 (SmBoolean bAbbrev,  // in : TRUE = skip nested object dumps, FALSE=include nested object dumps
  ULONG lIndentCnt)   // in : Indent print statements by lIndentCnt number of spaces
 const
{
  // locals
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE], sIndent[32] = {};

  // build prefix
  for(ii=0;ii<lIndentCnt;ii++) 
    { SM_STRCAT(sIndent, _T(" ")) ; }

  // header
  smos_sprintf(sBuff,        _T("\n%sBegin BaseClass SmVolume[0x%p]::Dump()"), sIndent, this) ;
  smos_sprintf(sBuffForFile, _T("\n%sBegin BaseClass SmVolume::Dump()"), sIndent) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // header
  smos_sprintf(sBuff,       _T("\n%s  SmVolume = 0x%p, m_pOwner = 0x%p, m_pNextMap = 0x%p, m_pOrientMap = 0x%p, m_pInvOrientMap = 0x%p"), sIndent, 
             this,
             m_pOwner,
             m_pNextMap,
             m_pOrientMap,
             m_pInvOrientMap);
  smos_sprintf(sBuffForFile,_T("\n%s  SmVolume = %s, m_pOwner = %s, m_pNextMap = %s, m_pOrientMap = %s, m_pInvOrientMap = %s\n"), sIndent,
             _T("notNULL"),
             (m_pOwner)        ? _T("notNULL") : _T("NULL"),
             (m_pNextMap)      ? _T("notNULL") : _T("NULL"),
             (m_pOrientMap)    ? _T("notNULL") : _T("NULL"),
             (m_pInvOrientMap) ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // nested object summary: m_pNextMap
  smos_sprintf(sBuff,        _T("\n%s    SmVolume has %s compounding map: SmVolume[0x%p]->m_pNextMap[0x%p] %s"), sIndent,
                           m_pNextMap == NULL ? _T("no")           : _T("a "), this, m_pNextMap,
                           m_pNextMap == NULL ? _T("     == NULL") : (bAbbrev ? _T("") : _T("     Dump coming up")) ) ;
  smos_sprintf(sBuffForFile, _T("\n%s    SmVolume has %s compounding map: SmVolume->m_pNextMap %s"), sIndent,
                           m_pNextMap == NULL ? _T("no")           : _T("a "), 
                           m_pNextMap == NULL ? _T("     == NULL") : (bAbbrev ? _T("     == NotNULL") : _T("     Dump coming up")) ) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  
  // nested object summary: m_pOrientMap
  smos_sprintf(sBuff,        _T("\n%s    SmVolume has %s orienting map  : SmVolume[0x%p]->m_pOrientMap[0x%p] %s"), sIndent,
                           m_pOrientMap == NULL ? _T("no")         : _T("an"), this, m_pOrientMap,
                           m_pOrientMap == NULL ? _T("   == NULL") : (bAbbrev ? _T("") : _T("   Dump coming up")) ) ;
  smos_sprintf(sBuffForFile, _T("\n%s    SmVolume has %s orienting map  : SmVolume->m_pOrientMap %s"), sIndent,
                           m_pOrientMap == NULL ? _T("no")         : _T("an"), 
                           m_pOrientMap == NULL ? _T("   == NULL") : (bAbbrev ? _T("   == NotNULL") : _T("   Dump coming up")) ) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  
  // nested object summary: m_pInvOrientMap
  smos_sprintf(sBuff,        _T("\n%s    SmVolume has %s InvOrient map  : SmVolume[0x%p]->m_pInvOrientMap[0x%p] %s"), sIndent,
                           m_pInvOrientMap == NULL ? _T("no")      : _T("an"), this, m_pInvOrientMap,
                           m_pInvOrientMap == NULL ? _T("== NULL") : (bAbbrev ? _T("") : _T("Dump coming up")) ) ;
  smos_sprintf(sBuffForFile, _T("\n%s    SmVolume has %s InvOrient map  : SmVolume->m_pInvOrientMap %s"), sIndent,
                           m_pInvOrientMap == NULL ? _T("no")      : _T("an"), 
                           m_pInvOrientMap == NULL ? _T("== NULL") : (bAbbrev ? _T("== NotNULL") : _T("Dump coming up")) ) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  
  // nested object dumps: m_pNextMap, m_pOrientMap, and m_pInvOrientMap

  // when asked
  if(bAbbrev == FALSE)
    {
      // nested object Dump: NextMap
      if(m_pNextMap)
        {
          // Dump NextMap
          smos_sprintf(sBuff,        _T("\n%s  Begin nested SmVolume[0x%p]->m_pNextMap[0x%p]::Dump()"), sIndent, this, m_pNextMap) ;  
          smos_sprintf(sBuffForFile, _T("\n%s  Begin nested SmVolume->m_pNextMap::Dump()"), sIndent) ;  
          smos_WriteBuffer(sBuff, sBuffForFile);  
            m_pNextMap->Dump(bAbbrev, lIndentCnt+2) ;
          smos_sprintf(sBuff,        _T("\n%s  End nested SmVolume[0x%p]->m_pNextMap[0x%p]::Dump()\n"), sIndent, this, m_pNextMap) ;  
          smos_sprintf(sBuffForFile, _T("\n%s  End nested SmVolume->m_pNextMap::Dump()\n"), sIndent) ;  
          smos_WriteBuffer(sBuff, sBuffForFile);  
        }

      // nested object Dump: OrientMap
      if(m_pOrientMap)
        {
          // Dump OrientMap
          smos_sprintf(sBuff,        _T("\n%s  Begin nested SmVolume[0x%p]->m_pOrientMap[0x%p]::Dump()"), sIndent, this, m_pOrientMap) ;
          smos_sprintf(sBuffForFile, _T("\n%s  Begin nested SmVolume->m_pOrientMap::Dump()"), sIndent) ; 
          smos_WriteBuffer(sBuff, sBuffForFile);  
            m_pOrientMap->Dump(bAbbrev, lIndentCnt+2) ;
          smos_sprintf(sBuff,        _T("\n%s  End nested SmVolume[0x%p]->m_pOrientMap[0x%p]::Dump()\n"), sIndent, this, m_pOrientMap) ;  
          smos_sprintf(sBuffForFile, _T("\n%s  End nested SmVolume->m_pOrientMap::Dump()\n"), sIndent) ;  
          smos_WriteBuffer(sBuff, sBuffForFile);  
        }

       // nested object Dump: InvOrientMap
      if(m_pInvOrientMap)
        {
         // Dump InvOrientMap
          smos_sprintf(sBuff,        _T("\n%s  Begin nested SmVolume[0x%p]->m_pInvOrientMap[0x%p]::Dump()"), sIndent, this, m_pInvOrientMap) ;  
          smos_sprintf(sBuffForFile, _T("\n%s  Begin nested SmVolume->m_pInvOrientMap::Dump()"), sIndent) ;  
          smos_WriteBuffer(sBuff, sBuffForFile);  
            m_pInvOrientMap->Dump(bAbbrev, lIndentCnt+2) ;
          smos_sprintf(sBuff,        _T("\n%s  End nested SmVolume[0x%p]->m_pInvOrientMap[0x%p]::Dump()\n"), sIndent, this, m_pInvOrientMap) ;  
          smos_sprintf(sBuffForFile, _T("\n%s  End nested SmVolume->m_pInvOrientMap::Dump()\n"), sIndent) ;  
          smos_WriteBuffer(sBuff, sBuffForFile);  
        }
    }

  // all done
  smos_sprintf(sBuff,        _T("\n%sEnd BaseClass SmVolume[0x%p]::Dump()%s"), sIndent, this, lIndentCnt==0?_T("\n"):_T("")) ;  
  smos_sprintf(sBuffForFile, _T("\n%sEnd BaseClass SmVolume::Dump()%s"), sIndent, lIndentCnt==0?_T("\n"):_T("")) ;  
  smos_WriteBuffer(sBuff, sBuffForFile) ;

} // end SmVolume::Dump

/*******************************************************************//**
PURPOSE: Add Volume graphics to the draw state

NOTES: Accumulates smgfx_Xxxx() nested OpenGL commands in one of two ways.
  1.) if(pOptGfxSet == NULL) : In a new or open OpenGL drawList whose ID is tracked in the        
      using old OpenGL      : file global SmGraphicsExtern.cpp:s_View.m_pActiveLists array.
                             
  2.) if(pOptGfxSet != NULL) : In a set of SmGraphicsVertexArray objects added to the
      using OpenGL ES       : input pOptGfxSet SmGfxArraySet object.            
***********************************************************************/
SmDisplayList * SmVolume::Draw
(
  SmBoolean       bShowOutSpace,   // in : TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]
  SmBoolean       bShowInSpace,    // in : TRUE = Draw InSpace Volume graphics, FALSE=don't ,default:[FALSE]
  SmExtent3d    * pOptParamDomain, // in : optional limiting param domain, NULL to ignore, default:[NULL]
  SmGfxArraySet * pOptGfxSet       // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
) const                             
{                                  
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE
  // Get SmGraphicsExtern.cpp:s_Disp global display parameters
  const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;

  // start drawlist (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this, rDisp.GetShadedColorRule()), NULL, NULL, FALSE, pOptGfxSet);

  // Draw unbounded volumes for a standard sized box centered on the origin
  SmPoint3d sUnboundedCenter(0,0,0) ;
  double    dUnboundedHalfSize = 33 ; 

  // Make the graphics calls
  SE(OutputGraphics(rDisp, pOptParamDomain, bShowOutSpace, bShowInSpace, &sUnboundedCenter, dUnboundedHalfSize, pOptGfxSet));

  // end this DisplayList 
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF4(bShowOutSpace, bShowInSpace, pOptParamDomain, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVolume::Draw

/*******************************************************************//**
PURPOSE: Add InSpace and/or OutSpace Point Position and Derivative Graphics 
         for a given ParamSpace Point.

NOTES:  Allowed values for lNumDeriv are
   0 = only draw point
   1 = draw point, 1st derivatives (dW/du and dW/dv), and volume normal

 Accumulates smgfx_Xxxx() nested OpenGL commands in one of two ways.
  1.) if(pOptGfxSet == NULL) : In a new or open OpenGL drawList whose ID is tracked in the        
      using old OpenGL      : file global SmGraphicsExtern.cpp:s_View.m_pActiveLists array.
                             
  2.) if(pOptGfxSet != NULL) : In a set of SmGraphicsVertexArray objects added to the
      using OpenGL ES       : input pOptGfxSet SmGfxArraySet object.            
***********************************************************************/
SmDisplayList * SmVolume::DrawAtParamPoint
  (const SmPoint3d & sParamPoint,   // in : target ParamSpace Point
   ULONG             lNumDeriv,     // in : optional derivative count
                                    //      0 = only draw point                                                   
                                    //      1 = draw point, 1st derivatives (dW/du, dW/dv, dW/dw)
   SmBoolean         bShowOutSpace, // in : TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]
   SmBoolean         bShowInSpace,  // in : TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]
   SmGfxArraySet   * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                    //      NULL to ignore. default:[NULL]                            
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new DisplayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);
  SmVector3d sColor = smgfx_OutputColor(1,0,0,pOptGfxSet); 

  SmPoint3d p0, p1; // evaluated pt, and evaluated pt plus derivative

  // output just the volume point
  if(lNumDeriv == 0) 
    {
      SmPoint3d sInPoint, sOutPoint;      

      if(bShowInSpace)  { OrientPoint(sParamPoint, sInPoint) ;
                          smgfx_OutputPoint(sInPoint.x,  sInPoint.y,  sInPoint.z,  pOptGfxSet) ; 
                        }
      if(bShowOutSpace) { EvaluatePoint(sParamPoint, sOutPoint) ;
                          smgfx_OutputPoint(sOutPoint.x, sOutPoint.y, sOutPoint.z, pOptGfxSet) ; 
                        }
    }
  
  // output volume point plus 1st derivatives
  else if (lNumDeriv >= 1) 
    {
      // evaluate point and derivatives
      SmPoint3d sInPoints[2][2][2], sOutPoints[2][2][2] ;

      // InSpace display
      if(bShowInSpace)
        {
          // orient
          Orient(sParamPoint,1,TRUE,TRUE,TRUE,sInPoints[0][0]);

          // position
          p0 = sInPoints[0][0][0];
          smgfx_OutputPoint( p0.x, p0.y, p0.z, pOptGfxSet ) ;

          // Wu
          smgfx_OutputColor(.4,.4,.8, pOptGfxSet) ; 
          p1 = p0 + sInPoints[1][0][0];
          smgfx_OutputLine ( p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, pOptGfxSet ) ;

          // Wv
          smgfx_OutputColor(.8,.4,.8, pOptGfxSet) ; 
          p1 = p0 + sInPoints[0][1][0];
          smgfx_OutputLine ( p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, pOptGfxSet ) ;

          // Ww
          smgfx_OutputColor(.4,.8,.8, pOptGfxSet) ; 
          p1 = p0 + sInPoints[0][0][1];
          smgfx_OutputLine ( p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, pOptGfxSet ) ;

        } // end if InSpace display

      // OutSpace display
      if(bShowOutSpace)
        {
          // evaluate
          Evaluate(sParamPoint,1,TRUE,TRUE,TRUE,sOutPoints[0][0]);

          // position
          p0 = sOutPoints[0][0][0];
          smgfx_OutputPoint( p0.x, p0.y, p0.z, pOptGfxSet );

          // Wu
          smgfx_OutputColor(0,0,1, pOptGfxSet) ; 
          p1 = p0 + sOutPoints[1][0][0] ;
          smgfx_OutputLine ( p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, pOptGfxSet ) ;

          // Wv
          smgfx_OutputColor(1,0,1, pOptGfxSet) ; 
          p1 = p0 + sOutPoints[0][1][0] ;
          smgfx_OutputLine ( p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, pOptGfxSet ) ;

          // Ww
          smgfx_OutputColor(0,1,1, pOptGfxSet) ; 
          p1 = p0 + sOutPoints[0][0][1] ;
          smgfx_OutputLine ( p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, pOptGfxSet ) ;

        } // end if OutSpace display
    } // end if lNumDeriv > 0

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF5(sParamPoint, lNumDeriv, bShowOutSpace, bShowInSpace, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVolume::DrawAtParamPoint

/*******************************************************************//**
PURPOSE: Add OutSpace Point position and optionally 1st derivative 
         vectors to draw state for given InSpace Point.

NOTES:  Allowed values for lNumDeriv are
   0 = only draw point
   1 = draw point, 1st derivatives (dW/du and dW/dv), and volume normal

 Accumulates smgfx_Xxxx() nested OpenGL commands in one of two ways.
  1.) if(pOptGfxSet == NULL) : In a new or open OpenGL drawList whose ID is tracked in the        
      using old OpenGL      : file global SmGraphicsExtern.cpp:s_View.m_pActiveLists array.
                             
  2.) if(pOptGfxSet != NULL) : In a set of SmGraphicsVertexArray objects added to the
      using OpenGL ES       : input pOptGfxSet SmGfxArraySet object.            
***********************************************************************/
SmDisplayList * SmVolume::DrawAtInSpacePoint
  (const SmPoint3d & sInSpacePoint, // in : target InSpace Point
   ULONG             lNumDeriv,     // in : optional derivative count
                                    //      0 = only draw point                                                   
                                    //      1 = draw point, 1st derivatives (dW/du, dW/dv, dW/dw)
   SmGfxArraySet   * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                    //      NULL to ignore. default:[NULL]                            
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new DisplayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);
  SmVector3d sColor = smgfx_OutputColor(1,0,0,pOptGfxSet); 

  SmPoint3d p0, p1 ; // evaluated pt, and evaluated pt plus derivative

  // output just the volume point
  if(lNumDeriv == 0) 
    {
      SmPoint3d sOutPoint ;
      MapPoint(sInSpacePoint, sOutPoint, TRUE) ;
      smgfx_OutputPoint(sOutPoint.x, sOutPoint.y, sOutPoint.z, pOptGfxSet) ;
    }
  
  // output volume point plus 1st derivatives
  else if (lNumDeriv >= 1) 
    {
      // evaluate point and derivatives
      SmPoint3d sOutPoints[2][2][2];
      Map(sInSpacePoint,1,TRUE,TRUE,TRUE,sOutPoints[0][0]);

      // position
      p0 = sOutPoints[0][0][0];
      smgfx_OutputPoint( p0.x, p0.y, p0.z, pOptGfxSet );

      // WxIn
      smgfx_OutputColor(0,0,1, pOptGfxSet); 
      p1 = p0 + sOutPoints[1][0][0];
      smgfx_OutputLine ( p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, pOptGfxSet );

      // WyIn
      smgfx_OutputColor(1,0,1, pOptGfxSet); 
      p1 = p0 + sOutPoints[0][1][0];
      smgfx_OutputLine ( p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, pOptGfxSet );

      // WzIn
      smgfx_OutputColor(0,1,1, pOptGfxSet); 
      p1 = p0 + sOutPoints[0][0][1];
      smgfx_OutputLine ( p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, pOptGfxSet );

    }

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(sInSpacePoint, lNumDeriv, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVolume::DrawAtInSpacePoint

/*******************************************************************//**
PURPOSE:  Add OutSpace ControlMesh and/or InSpace IsoLine Grid to draw state.

NOTES: 
 For non SmBSplineVolume types, instead of a ControlMesh draws an IsoCurve Grid mesh.

 Accumulates smgfx_Xxxx() nested OpenGL commands in one of two ways.
  1.) if(pOptGfxSet == NULL) : In a new or open OpenGL drawList whose ID is tracked in the        
      using old OpenGL      : file global SmGraphicsExtern.cpp:s_View.m_pActiveLists array.
                             
  2.) if(pOptGfxSet != NULL) : In a set of SmGraphicsVertexArray objects added to the
      using OpenGL ES       : input pOptGfxSet SmGfxArraySet object.            
   
***********************************************************************/
SmDisplayList * SmVolume::DrawMesh
 (SmBoolean       bShowOutSpace, // in : TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]
  SmBoolean       bShowInSpace,  // in : TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]
  SmGfxArraySet * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                 //      NULL to ignore. default:[NULL]
 const                          
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // override display values
  sDisp.m_bDrawPolygon = TRUE ;
    
  // open display list (unless one is already open
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // Draw unbounded volumes for a standard sized box centered on the origin
  SmPoint3d sUnboundedCenter(0,0,0) ;
  double    dUnboundedHalfSize = 33 ; 

  OutputGraphics(sDisp, NULL, bShowOutSpace, bShowInSpace, &sUnboundedCenter, dUnboundedHalfSize, pOptGfxSet) ; 

  // end displayList
  pRtn = smgfx_Close(pOptGfxSet) ;
  smgfx_OutputColor(sColor, pOptGfxSet) ;

#else
  SM_REF3(bShowOutSpace, bShowInSpace, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVolume::DrawMesh

/*******************************************************************//**
PURPOSE:  Add OutSpace ControlPoint and/or InSpace IsoLine Grid to draw state.

NOTES:
 For non SmBSplineVolume types, instead of a ControlMesh draws an IsoCurve Grid mesh.
 
 Accumulates smgfx_Xxxx() nested OpenGL commands in one of two ways.
  1.) if(pOptGfxSet == NULL) : In a new or open OpenGL drawList whose ID is tracked in the        
      using old OpenGL      : file global SmGraphicsExtern.cpp:s_View.m_pActiveLists array.
                             
  2.) if(pOptGfxSet != NULL) : In a set of SmGraphicsVertexArray objects added to the
      using OpenGL ES       : input pOptGfxSet SmGfxArraySet object.            
***********************************************************************/
SmDisplayList * SmVolume::DrawControlPoints
  (SmBoolean      bShowOutSpace,  // in : TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]
   SmBoolean      bShowInSpace,   // in : TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]
   SmGfxArraySet *pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                  //      NULL to ignore. default:[NULL] 
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // override display values
  sDisp.m_bDrawControlPoints = TRUE ;
    
  // open display list (unless one is already open
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // Draw unbounded volumes for a standard sized box centered on the origin
  SmPoint3d sUnboundedCenter(0,0,0) ;
  double    dUnboundedHalfSize = 33 ; 

  OutputGraphics(sDisp, NULL, bShowOutSpace, bShowInSpace, &sUnboundedCenter, dUnboundedHalfSize, pOptGfxSet) ; 

  // end displayList
  pRtn = smgfx_Close(pOptGfxSet) ;
  smgfx_OutputColor(sColor, pOptGfxSet) ;

#else
  SM_REF3(bShowOutSpace, bShowInSpace, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVolume::DrawControlPoints

/*******************************************************************//**
PURPOSE: Add Volume IsoCurve InSpace and/or OutSpace graphics to draw state.

NOTES: 
 Accumulates smgfx_Xxxx() nested OpenGL commands in one of two ways.
  1.) if(pOptGfxSet == NULL) : In a new or open OpenGL drawList whose ID is tracked in the        
      using old OpenGL      : file global SmGraphicsExtern.cpp:s_View.m_pActiveLists array.
                             
  2.) if(pOptGfxSet != NULL) : In a set of SmGraphicsVertexArray objects added to the
      using OpenGL ES       : input pOptGfxSet SmGfxArraySet object.            
***********************************************************************/
SmDisplayList * SmVolume::DrawUVW      
  (ULONG     lNumBetweenU,            // in : number of U IsoParameter lines between knots
   ULONG     lNumBetweenV,            // in : number of V IsoParameter lines between knots
   ULONG     lNumBetweenW,            // in : number of W IsoParameter lines between knots
   SmBoolean bVaryCrossHatchColor,    // in : TRUE = Draw U Lines in ObjectColor 
                                      //             Draw V lines in m_VaryCrossHatchColor  
                                      //      FALSE= Draw both U and V Lines in ObjectColor
                                      //      default:[FALSE]
   const SmExtent3d *pOptParamDomain, // in : ParamDomain to crossHatch, NULL=NaturalParamDomain, default:[NULL]
   SmBoolean         bShowOutSpace,   // in : TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]
   SmBoolean         bShowInSpace,    // in : TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]
   SmGfxArraySet    *pOptGfxSet)       // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                      //      NULL to ignore. default:[NULL]
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // override display values
  sDisp.m_bDrawCrossHatch      = TRUE ;
  sDisp.m_lCrossHatchUCount    = lNumBetweenU ;
  sDisp.m_lCrossHatchVCount    = lNumBetweenV ;
  sDisp.m_lCrossHatchWCount    = lNumBetweenW ;
  sDisp.m_bVaryCrossHatchColor = bVaryCrossHatchColor ;
    
  // start DisplayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // Draw unbounded volumes for a standard sized box centered on the origin
  SmPoint3d sUnboundedCenter(0,0,0) ;
  double    dUnboundedHalfSize = 33 ; 

  // output graphics commands
  SE(OutputGraphics(sDisp, pOptParamDomain, bShowOutSpace, bShowInSpace, &sUnboundedCenter, dUnboundedHalfSize, pOptGfxSet));

  // close displayList
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF8(lNumBetweenU, lNumBetweenV, lNumBetweenW, bVaryCrossHatchColor, pOptParamDomain, bShowOutSpace, bShowInSpace, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVolume::DrawUVW

/*******************************************************************//**
PURPOSE:  Add array of Volume sample points and optional
             1st derivative vectors to new
             or open displayList added to global displayList array.

NOTES: 
***********************************************************************/
SmDisplayList * SmVolume::DrawVectorField
  (const SmExtent3d & crParamDomain,    // NotUsed: in : volume domain
   ULONG              lNumUPoints,      // in : number of U points
   ULONG              lNumVPoints,      // in : number of V point
   ULONG              lNumWPoints,      // in : number of W points
   SmPinCushionType   eType,            // in : oneof: SM_DM_POINTS       
                                        //             SM_DM_U_NATURAL    
                                        //             SM_DM_V_NATURAL   
                                        //             SM_DM_W_NATURAL 
                                        //             SM_DM_UVW_NATURAL   
                                        //             SM_DM_U_SCALED     
                                        //             SM_DM_V_SCALED     
                                        //             SM_DM_W_SCALED     
                                        //             SM_DM_UVW_SCALED    
   SmBoolean           bShowOutSpace,   // in : TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]
   SmBoolean           bShowInSpace,    // in : TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]
   SmGfxArraySet     * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                        //      NULL to ignore. default:[NULL]
 const                                    
{     
  SM_REF1(crParamDomain) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  SmDisplayParameters sDisp ;
  smgfx_GetGlobalDisplayParameters(sDisp) ;

  // override display values
  sDisp.m_bDrawPinCushion = TRUE ;
  sDisp.m_lUPointCount    = lNumUPoints ;
  sDisp.m_lVPointCount    = lNumVPoints ;
  sDisp.m_lWPointCount    = lNumWPoints ;
  sDisp.m_ePinCushionType = eType ;
    
  // open display list (unless one is already open
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this, sDisp.GetShadedColorRule()), NULL, NULL, FALSE, pOptGfxSet);

  // Draw unbounded volumes for a standard sized box centered on the origin
  SmPoint3d sUnboundedCenter(0,0,0) ;
  double    dUnboundedHalfSize = 33 ; 

  OutputGraphics(sDisp, NULL, bShowOutSpace, bShowInSpace, &sUnboundedCenter, dUnboundedHalfSize, pOptGfxSet) ; 

  // end displayList]
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF7(lNumUPoints, lNumVPoints, lNumWPoints, eType, bShowOutSpace, bShowInSpace, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmVolume::DrawVectorField

#ifdef SM_GFX_OUTPUT_CODE
/*******************************************************************//**
PURPOSE:  Create and display a Volume isoParameter Curve

NOTES: 
***********************************************************************/
static SmStatus sm_DrawParamCurve
  (const SmDisplayParameters & crDisp,           // in : Current display parameters            
   const SmVolume            * pVolume,          // in : target volume
   const SmPoint3d           & crUVWStart,       // in : Curve start point
   const SmPoint3d           & crUVWEnd,         // in : Curve end point
   const SmExtent3d          * pOptParamDomain,  // in : domain limit for ParamUVTrimCurves
                                                 //      NULL = use Volume->NaturalParamDomain 
   SmBoolean                   bShowOutSpace,    // in : TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]
   SmBoolean                   bShowInSpace,     // in : TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]
   SmGfxArraySet             * pOptGfxSet)       // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                                 //      NULL to ignore. default:[NULL]
{ 
  // local Context pointer
  const SmContext *cpContext = pVolume->GetContext() ; SM_ASSERT(cpContext != NULL) ;

  // make temporary iso curve
  SmCurve *pOutSpaceIsoCurve = NULL ;
  SmCurve *pInSpaceIsoCurve  = NULL ;

  if(   SM_ARE_SAME(crUVWStart.y, crUVWEnd.y)
     && SM_ARE_SAME(crUVWStart.z, crUVWEnd.z)) 
    {        
      SER(pVolume->EvaluateIsoParametricCurve(*cpContext,SM_VPS_VW, crUVWStart.y, crUVWStart.z, 0.0, 
                                              bShowOutSpace ? &pOutSpaceIsoCurve : NULL,
                                              bShowInSpace  ? &pInSpaceIsoCurve  : NULL,
                                              pOptParamDomain, TRUE));
    }
  else if(   SM_ARE_SAME(crUVWStart.x, crUVWEnd.x)
          && SM_ARE_SAME(crUVWStart.z, crUVWEnd.z))
    {
      SER(pVolume->EvaluateIsoParametricCurve(*cpContext,SM_VPS_UW, crUVWStart.x, crUVWStart.z, 0.0, 
                                              bShowOutSpace ? &pOutSpaceIsoCurve : NULL,
                                              bShowInSpace  ? &pInSpaceIsoCurve  : NULL,
                                              pOptParamDomain, TRUE));
    }
  else 
    {
      SER(pVolume->EvaluateIsoParametricCurve(*cpContext,SM_VPS_UV, crUVWStart.x, crUVWStart.y, 0.0, 
                                              bShowOutSpace ? &pOutSpaceIsoCurve : NULL,
                                              bShowInSpace  ? &pInSpaceIsoCurve  : NULL,
                                              pOptParamDomain, TRUE));
    }
  SmObjDelete sCleanup(pOutSpaceIsoCurve);

  // output iso curves
  if(bShowOutSpace)
    SER(pOutSpaceIsoCurve->OutputGraphics(pOutSpaceIsoCurve->GetNaturalInterval(), crDisp, NULL, NULL, pOptGfxSet)) ;

  if(bShowInSpace)
    SER(pInSpaceIsoCurve->OutputGraphics(pInSpaceIsoCurve->GetNaturalInterval(), crDisp, NULL, NULL, pOptGfxSet)) ;

  // all done
  return SM_SUCCESS;

} // end static SmStatus sm_DrawParamCurve

/*******************************************************************//**
PURPOSE:  Create and display a Volume isoParameter Surface

NOTES: 
***********************************************************************/
static SmStatus sm_DrawParamSurface
  (const SmDisplayParameters & crDisp,           // in : Current display parameters            
   const SmVolume            * pVolume,          // in : target volume
   SmVolumeParamType           eVolumeParam,     // in : Specify Volume constant parameter direction
   double                      dIsoParameter,    // in : Specify Volume constant parameter value
   const SmExtent3d          * pOptParamDomain,  // in : domain limit for ParamUVTrimCurves
                                                 //      NULL = use Volume->NaturalParamDomain 
   SmBoolean                   bShowOutSpace,    // in : TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]
   SmBoolean                   bShowInSpace,     // in : TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]
   SmGfxArraySet             * pOptGfxSet)       // NotUsed: i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                                 //      NULL to ignore. default:[NULL]
                                                
{                                               
  SM_REF1(pOptGfxSet) ;
  // local Context pointer
#ifdef SM_GFX_CODE
  const SmContext *cpContext = pVolume->GetContext() ; SM_ASSERT(cpContext != NULL) ;

  // make temporary iso surface
  SmSurface *pOutSpaceIsoSurface = NULL ;
  SmSurface *pInSpaceIsoSurface = NULL ;

  SER(pVolume->EvaluateIsoParametricSurface(*cpContext,eVolumeParam, dIsoParameter, 0.0, 
                                            bShowOutSpace ? &pOutSpaceIsoSurface : NULL,
                                            bShowInSpace  ? &pInSpaceIsoSurface  : NULL,
                                            pOptParamDomain, TRUE));
  SmObjDelete sCleanup(pOutSpaceIsoSurface);

  // draw OutSpace iso surface
  if(pOutSpaceIsoSurface)
    {
      // Make the IsoSurface graphics calls
      SE(pOutSpaceIsoSurface->OutputGraphics(crDisp, NULL));
    } // end draw pOutSpaceIsoSurface

  // draw InSpace iso surface
  if(pInSpaceIsoSurface)
    {
      // Make the IsoSurface graphics calls
      SE(pInSpaceIsoSurface->OutputGraphics(crDisp, NULL));
    } // end draw pInSpaceIsoSurface

#endif

  // all done
  return SM_SUCCESS;

} // end static SmStatus sm_DrawParamSurface
#endif // SM_GFX_OUTPUT_CODE

/*******************************************************************//**
PURPOSE: Add volume Graphics switching on DisplayParameters to
            open displayList

NOTES: This method is just a sample way of drawing a Brep Volume.
   Different applications will want to construct graphics differently.

   Switch on bits in crDisp to draw different images as follows:

   if(crDisp.m_lCrossHatchUCount == 999) // draw volume as 8 corners connected by lines
                                         //   used for drawing larger volumes

   if(m_bDrawWireFrame)       // Draw all Edges and Vertices
   if(m_bUseEdgeTypeColors)   // Color edges by Type (manifold, lamina, wire)
                              
   if(IsNLibFaceting())       // Generate/Output NLib facets   on displayed isoSurfaces
   if(IsSMLibFaceting())      // Generate/Output SMLib facets on displayed isoSurfaces
   if(m_bDoShading)           // TRUE = Draw tessellation facets shaded on displayed isoSurfaces
                              // FALSE= Draw tessellation facets polygon outlines on displayed isoSurfaces
   if(m_bDraw3DField)         // TRUE = Project Brep through 3dField - shace shaping
                              
   if(m_bDrawCrossHatch)      // Draw sequence of constant w IsopParameterSurfaces
   if(m_bVaryCrossHatchColor) // Draw U Lines in ObjectColor - V Lines in m_VaryCrossHatchColor   
   if(m_bDashedLines)         // Draw crossHatched lines dashed
                              
   if(m_bDrawKnots)           // Draw edge knotPoints (place volume xhatch lines on knot boundaries)
   if(m_bDrawPolygon)         // Draw volume->ControlMesh  = ControlPoints + Mesh
   if(m_bDrawControlPoints)   // Draw volume->ControlPoints  = just control points
   if(m_bDrawPinCushion)      // Draw Volume Vector fields

***********************************************************************/
SmStatus SmVolume::OutputGraphics
  (const SmDisplayParameters & crDisp,              // in : display control parameters
   const SmExtent3d          * pOptParamDomain,     // NotUsed: in : ParamDomain to crossHatch
                                                    //      NULL=NaturalParamDomain
   SmBoolean                   bShowOutSpace,       // in : TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]
   SmBoolean                   bShowInSpace,        // in : TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]
   SmPoint3d                 * pOptUnboundedCenter, // in : Center of Interest for Unbounded Domains, NUll to ignore, 
                                                    //      default:[NULL]
   double                      dUnboundedHalfSize,  // in : Unbounded half space display size. Unbounded volumes displayed 
                                                    //      over a domain cube twice this size, default:[33]
   SmGfxArraySet             * pOptGfxSet)          // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                    //      NULL to ignore. default:[NULL]

 const
{
  SM_REF1(pOptParamDomain) ;
#ifdef SM_GFX_OUTPUT_CODE
  // locals
  ULONG i, j, k, off ;
  SmVector3d sColor;
  double     dLineWidth;
  const SmVolume  *pVolume      = this ;
  SmBSplineVolume *pBSV         = SM_CAST_NONNULL_PTR(SmBSplineVolume, this) ;
  SmExtent3d       sParamDomain = pVolume->GetDiscontinuityFreeParamDomain() ; 
  // note: Special case SmUnbendVolumes - until something is done for the idea of an infinite domain without internal discontinuties
  //  limits the SmUnbendVolumes domain to any box in the x>0 1/2 space.  

  SmBoolean        bUnbounded   = !sParamDomain.IsBounded() ; 

  // when asked - trim unbounded volumes domains
  if(pOptUnboundedCenter && bUnbounded)
    {
      // Approximate infinite domain with a finite one centered on pOptUnboundedCenter and sized as dUnboundedHalfSize
      sParamDomain = sParamDomain.ApproximateUnbounded(pOptUnboundedCenter, dUnboundedHalfSize) ;

      bUnbounded = FALSE ;
    } // end watch for unbounded trim volumes check

  // Crosshatch of 999 means draw outline as a trilinear polyhedron
  if(   crDisp.m_lCrossHatchUCount == 999 
     || crDisp.m_lCrossHatchVCount == 999
     || crDisp.m_lCrossHatchWCount == 999
     || bUnbounded) 
    { 
      SmTArray<SmPoint3d> sPnts;
      if(!IsBounded()) { sParamDomain.SetMinMax(SmPoint3d(0.0,0.0,0.0), SmPoint3d(1.0,1.0,1.0)) ;
                       }

      // eval corner points
      SmPoint3d sP000, sP010, sP100, sP110, sP001, sP011, sP101, sP111 ;      
      EvaluatePoint(sParamDomain.Evaluate(0.0, 0.0, 0.0), sP000);
      EvaluatePoint(sParamDomain.Evaluate(1.0, 1.0, 0.0), sP110);
      EvaluatePoint(sParamDomain.Evaluate(1.0, 0.0, 0.0), sP100);
      EvaluatePoint(sParamDomain.Evaluate(0.0, 1.0, 0.0), sP010);
      EvaluatePoint(sParamDomain.Evaluate(0.0, 0.0, 0.0), sP001);
      EvaluatePoint(sParamDomain.Evaluate(1.0, 1.0, 0.0), sP111);
      EvaluatePoint(sParamDomain.Evaluate(1.0, 0.0, 0.0), sP101);
      EvaluatePoint(sParamDomain.Evaluate(0.0, 1.0, 0.0), sP011);
      
      // traverse all corners - some lines are drawn twice
      sPnts.Add(sP000);
      sPnts.Add(sP100);
      sPnts.Add(sP110);
      sPnts.Add(sP010);
      sPnts.Add(sP000);
      sPnts.Add(sP001);
      sPnts.Add(sP101); sPnts.Add(sP100); sPnts.Add(sP101);
      sPnts.Add(sP111); sPnts.Add(sP110); sPnts.Add(sP111);
      sPnts.Add(sP011); sPnts.Add(sP010); sPnts.Add(sP011);
      sPnts.Add(sP001);

      // output Polyline
      smgfx_OutputPolyline((double*)sPnts.GetDataArray(),sPnts.GetSize(),3,pOptGfxSet);
      return SM_SUCCESS;

    } // end SpecialCase lCrossHatchUCount == 999 check
  
  // Draw interval boundary corners and isoParameter Curves
  if (crDisp.m_bDrawWireFrame) 
    { 
      // domain is in sParamDomain
      SmTArray<double> sUParams(2, NULL, 2), sVParams(2, NULL, 2), sWParams(2, NULL, 2);

      if(bUnbounded)
        {
          SmExtent3d sBoundedBox = sParamDomain.ApproximateUnbounded() ;
          sUParams[0] = sBoundedBox.GetMin().x;
          sUParams[1] = sBoundedBox.GetMax().x;
          sVParams[0] = sBoundedBox.GetMin().y;
          sVParams[1] = sBoundedBox.GetMax().y; 
          sWParams[0] = sBoundedBox.GetMin().z;
          sWParams[1] = sBoundedBox.GetMax().z; 
        }
      else
        {
          sUParams[0] = sParamDomain.GetMin().x;
          sUParams[1] = sParamDomain.GetMax().x;
          sVParams[0] = sParamDomain.GetMin().y;
          sVParams[1] = sParamDomain.GetMax().y; 
          sWParams[0] = sParamDomain.GetMin().z;
          sWParams[1] = sParamDomain.GetMax().z; 
        }
           
      dLineWidth = smgfx_OutputLineWidth(crDisp.m_dCrossHatchLineWidth, pOptGfxSet);

      // Draw 4 varying U edge boundaries for constant V and W values
      for (j=0; j<sVParams.GetSize(); j++) 
        {
         // constant V
         double dV = sVParams[j];
         for(k=0; k<sWParams.GetSize(); k++)
           {
             // constant W
             double dW = sWParams[k];

             // varying U IsoCurve
             SmPoint3d sStart(sUParams[0],dV,dW);
             SmPoint3d sEnd  (sUParams[1],dV,dW);
             sm_DrawParamCurve(crDisp, pVolume, sStart, sEnd, &sParamDomain, bShowOutSpace, bShowInSpace, pOptGfxSet);
           }                                            
        } // end iter every V knot

      // Draw 4 varying V edge boundaries for constant U and W values
      for (i=0; i<sUParams.GetSize(); i++) 
        {
          // constant U
          double dU = sUParams[i];
          for(k=0; k<sWParams.GetSize(); k++)
            {
              // constant W
              double dW = sWParams[k];
              
              // varying V IsoCurve
              SmPoint3d sStart(dU,sVParams[0],dW);
              SmPoint3d sEnd  (dU,sVParams[1],dW);
              sm_DrawParamCurve(crDisp, pVolume, sStart, sEnd, &sParamDomain, bShowOutSpace, bShowInSpace, pOptGfxSet);
            }                                            
        } // end iter every V knot

      // Draw 4 varying W edge boundaries for constant V and W values 
      for (i=0; i<sUParams.GetSize(); i++) 
        {
          // constant U
          double dU = sUParams[i];
          for(j=0; j<sVParams.GetSize(); j++)
            {
              // constant V
              double dV = sVParams[j];
              
              // Varying W IsoCurve
              SmPoint3d sStart(dU,dV,sWParams[0]);
              SmPoint3d sEnd  (dU,dV,sWParams[1]);
              sm_DrawParamCurve(crDisp, pVolume, sStart, sEnd, &sParamDomain, bShowOutSpace, bShowInSpace, pOptGfxSet);
            }                                            
        } // end iter every U knot

      smgfx_OutputLineWidth(dLineWidth, pOptGfxSet) ;

    } // end draw boundary isoParameter curves


  // Draw CrossHatched set of W isoSurfaces and varying W boundary iso edges
  if (crDisp.m_bDrawCrossHatch)   
    {
      dLineWidth = smgfx_OutputLineWidth(crDisp.m_dCrossHatchLineWidth, pOptGfxSet);

      // Draw 4 Varying W edge boundaries with constant U and V values
      for (i=0; i<2; i++) 
        {
          // constant U
          double dU = i == 0 ? sParamDomain.GetMin().x
                             : sParamDomain.GetMax().x ;
          for(j=0; j<2; j++)
            {
              // constant V
              double dV = j == 0 ? sParamDomain.GetMin().y
                                 : sParamDomain.GetMax().y ;

              // varying W isoCurve
              SmPoint3d sStart(dU,dV,sParamDomain.GetMin().z);
              SmPoint3d sEnd  (dU,dV,sParamDomain.GetMax().z);
              sm_DrawParamCurve(crDisp, pVolume, sStart, sEnd, &sParamDomain, bShowOutSpace, bShowInSpace, pOptGfxSet);
            }                                            
        } // end iter every U knot

      // draw m_lCrossHatchWCount constant W isoSurfaces
      SmExtent1d sWInterval = sParamDomain.GetWInterval() ;
      ULONG lWCount =   crDisp.m_lCrossHatchWCount > 3 
                      ? crDisp.m_lCrossHatchWCount
                      : 3 ;

      // draw every isoSurface
      for(k=0;k<lWCount;k++)
        {
          // constant W
          double dParam =   k == 0         ? 0.0
                          : k == lWCount-1 ? 1.0
                          :                  (double)k/(double)(lWCount-1) ;
          double dW = sWInterval.Evaluate(dParam) ;

          // constant W IsoPlane
          sm_DrawParamSurface(crDisp, pVolume, SM_VP_W, dW, &sParamDomain, bShowOutSpace, bShowInSpace, pOptGfxSet) ;
           
        } // end iter every isoSurface

    } // end Draw Volume crossHatch isoSurfaces and boundary lines

  // Draw volume polyhedron - only for SmBSplineVolumes
  if (   pBSV
      && (   crDisp.m_bDrawPolygon
          || crDisp.m_bDrawControlPoints)) 
    { 
     // make this blue
     sColor     = smgfx_OutputObjectColor(this, SM_CR_VOLUMECONTROLPOLYGON, pOptGfxSet) ;
     dLineWidth = smgfx_OutputLineWidth(1.0, pOptGfxSet);

     // add controlPoint and ControlNet graphics to displayList
     SmTArray<SmPoint3d> sCPoint ;
     SmPoint3d           sPnts[2] ;
     SmTArray<double>    Weight ;   
     ULONG               lUCount, lVCount, lWCount ;
     pBSV->GetControlPointMesh(lUCount, lVCount, lWCount, sCPoint, Weight) ;  // in ProjSpace
     ULONG lUStep = lVCount * lWCount ;

     // for every control Point - draw in OutSpace mesh lines
     for(i=0,off=0;i<lUCount;i++)
       {
         for(j=0;j<lVCount;j++)
           {
             for(k=0;k<lWCount;k++,off++)
               {
                 // sPnts[0] = sCPoint[off] ;
                 OrientPoint(sCPoint[off], sPnts[0]) ;


                 // draw all control points
                 smgfx_OutputPoint(sPnts[0].x,
                                   sPnts[0].y,
                                   sPnts[0].z,
                                   pOptGfxSet) ;

                 // draw u dir mesh line
                 if(crDisp.m_bDrawPolygon)
                   {
                     if(i > 0) { // sPnts[1] = sCPoint[off-lUStep] ;
                                 OrientPoint(sCPoint[off-lUStep], sPnts[1]) ; 
                                 smgfx_OutputLine(sPnts[0].x, sPnts[0].y, sPnts[0].z,
                                                  sPnts[1].x, sPnts[1].y, sPnts[1].z, pOptGfxSet) ;
                               }
                     if(j > 0) { // sPnts[1] = sCPoint[off-lWCount] ;
                                 OrientPoint(sCPoint[off-lWCount], sPnts[1]) ;
                                 smgfx_OutputLine(sPnts[0].x, sPnts[0].y, sPnts[0].z,
                                                  sPnts[1].x, sPnts[1].y, sPnts[1].z, pOptGfxSet) ;
                               }
                     if(k > 0) { // sPnts[1] = sCPoint[off-1] ;
                                 OrientPoint(sCPoint[off-1], sPnts[1]) ;
                                 smgfx_OutputLine(sPnts[0].x, sPnts[0].y, sPnts[0].z,
                                                  sPnts[1].x, sPnts[1].y, sPnts[1].z, pOptGfxSet) ;
                               }
                   }
               } // end iter W mesh steps
           } // end iter V mesh steps
       } // end iter U mesh steps  
    } // if bDrawPolygon

  // draw PinCushions
  if (crDisp.m_bDrawPinCushion) 
    { 
      // make this blue
      sColor     = smgfx_OutputObjectColor(this, SM_CR_VOLUMECONTROLPOLYGON, pOptGfxSet) ;
      dLineWidth = smgfx_OutputLineWidth(1.0, pOptGfxSet);

      double lNumUPoints = crDisp.m_lUPointCount ;
      double lNumVPoints = crDisp.m_lVPointCount ;
      double lNumWPoints = crDisp.m_lWPointCount ;
      double dScale      = crDisp.m_dPinCushionScale ;

      // locals
      SmPoint3d sPnt, sPnt2, sPnt3, sPnt4 ;
      SmPoint3d sPar, sPar2, sPar3, sPar4 ;
      SmVector3d sParU(1,0,0), sParV(0,1,0), sParW(0,0,1) ;
      SmBoolean bPnt4 = FALSE ;
      SmVector3d sDU, sDV, sDW;

      // for every point
      for (i=0; i<=lNumUPoints; i++) 
        {
          for (j=0; j<=lNumVPoints; j++) 
            {
               for (k=0; k<=lNumWPoints; k++) 
                { 
                  // evaluate VolumePoint
                  double du = (double)(i)/(double)(lNumUPoints);
                  double dv = (double)(j)/(double)(lNumVPoints);
                  double dw = (double)(k)/(double)(lNumWPoints);
                  SmPoint3d sUVW = sParamDomain.Evaluate(du,dv,dw);
                  sPar = sUVW ;
                  Evaluate1stDerivatives(sUVW,TRUE,TRUE,TRUE,sPnt,sDU,sDV,sDW);

                  if(crDisp.m_ePinCushionType == SM_DM_POINTS) 
                    { continue; }

                  switch(crDisp.m_ePinCushionType)
                    {
                      case SM_DM_U_NATURAL     : sPnt2 = sPnt + sDU ;  sPar2 = sPar + sParU ;    break ;       
                      case SM_DM_V_NATURAL     : sPnt2 = sPnt + sDV ;  sPar2 = sPar + sParV ;    break ;
                      case SM_DM_W_NATURAL     : sPnt2 = sPnt + sDW ;  sPar2 = sPar + sParW ;    break ;
                      case SM_DM_UV_NATURAL    :
                      case SM_DM_UVW_NATURAL   : sPnt2 = sPnt + sDU ;  sPar2 = sPar + sParU ;         
                                                 sPnt3 = sPnt + sDV ;  sPar3 = sPar + sParV ;  
                                                 sPnt4 = sPnt + sDW ;  sPar4 = sPar + sParW ;  
                                                 bPnt4 = TRUE ;                                  break ;
                      case SM_DM_U_SCALED      : SE(sDU.Unitize()) ;   sPnt2 = sPnt + sDU * dScale ;   sPar2 = sPar + sParU * dScale; break ;
                      case SM_DM_V_SCALED      : SE(sDV.Unitize()) ;   sPnt2 = sPnt + sDV * dScale ;   sPar2 = sPar + sParV * dScale; break ;
                      case SM_DM_W_SCALED      : SE(sDW.Unitize()) ;   sPnt2 = sPnt + sDW * dScale ;   sPar2 = sPar + sParW * dScale; break ;
                      case SM_DM_UV_SCALED     :
                      case SM_DM_UVW_SCALED    : SE(sDU.Unitize()) ;   sPnt2 = sPnt + sDU * dScale ;   sPar2 = sPar + sParU * dScale; 
                                                 SE(sDV.Unitize()) ;   sPnt3 = sPnt + sDV * dScale ;   sPar3 = sPar + sParV * dScale;    
                                                 SE(sDW.Unitize()) ;   sPnt4 = sPnt + sDW * dScale ;   sPar4 = sPar + sParW * dScale;    
                                                 bPnt4 = TRUE ;                                  break ;
                      default: SER(SM_ERR) ; 
                    } // end switch on lMode

                  if(bShowOutSpace)
                    {
                      // DrawPoint
                      smgfx_OutputPoint(sPnt.x, sPnt.y, sPnt.z, pOptGfxSet) ;

                      if(bPnt4 == FALSE)
                        { 
                          // draw one vector in current color
                          smgfx_OutputLine(sPnt.x, sPnt.y, sPnt.z,
                                           sPnt2.x,sPnt2.y,sPnt2.z, pOptGfxSet);
                        }
                      else // draw three vecotrs in U(Red), V(Green), W(Blue) colors
                        { 
                          smgfx_OutputColor(1,0,0, pOptGfxSet) ;
                          smgfx_OutputLine(sPnt.x, sPnt.y, sPnt.z,
                                           sPnt2.x,sPnt2.y,sPnt2.z, pOptGfxSet);

                          smgfx_OutputColor(0,1,0, pOptGfxSet) ;
                          smgfx_OutputLine(sPnt.x, sPnt.y, sPnt.z,
                                           sPnt3.x,sPnt3.y,sPnt3.z, pOptGfxSet);

                          smgfx_OutputColor(0,0,1, pOptGfxSet) ;
                          smgfx_OutputLine(sPnt.x, sPnt.y, sPnt.z,
                                           sPnt4.x,sPnt4.y,sPnt4.z, pOptGfxSet);
                          smgfx_OutputColor(sColor, pOptGfxSet) ;
                        }
                    } // end bShowOutSpace check

                  if(bShowInSpace)
                    {
                      smgfx_OutputColor(.8,.4,.4, pOptGfxSet) ;
                      
                      // DrawPoint
                      smgfx_OutputPoint(sPar.x, sPar.y, sPar.z, pOptGfxSet) ;

                      if(bPnt4 == FALSE)
                        { 
                          // draw one vector in current color
                          smgfx_OutputLine(sPar.x, sPar.y, sPar.z,
                                           sPar2.x,sPar2.y,sPar2.z, pOptGfxSet);
                        }
                      else // draw three vecotrs in U(Red), V(Green), W(Blue) colors
                        { 
                          smgfx_OutputLine(sPar.x, sPar.y, sPar.z,
                                           sPar2.x,sPar2.y,sPar2.z, pOptGfxSet);

                          smgfx_OutputColor(.4,.8,.4, pOptGfxSet) ;
                          smgfx_OutputLine(sPar.x, sPar.y, sPar.z,
                                           sPar3.x,sPar3.y,sPar3.z, pOptGfxSet);

                          smgfx_OutputColor(.4,.4,.8, pOptGfxSet) ;
                          smgfx_OutputLine(sPar.x, sPar.y, sPar.z,
                                           sPar4.x,sPar4.y,sPar4.z, pOptGfxSet);
                          smgfx_OutputColor(sColor, pOptGfxSet) ;
                        }
                    } // end bShowOutSpace check
                } // end iter k, for every ijk point
            } // end iter j
        } // end iter i

      smgfx_OutputLineWidth(dLineWidth, pOptGfxSet);
      smgfx_OutputColor(sColor, pOptGfxSet) ;

    } // end if bPinCushions check

#else
  SM_REF6(crDisp, bShowOutSpace, bShowInSpace, pOptUnboundedCenter, dUnboundedHalfSize, pOptGfxSet);
#endif

  // all done
  return SM_SUCCESS;

} // end SmVolume::OutputGraphics

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertVolume_list[] =
{
  /*  0 */ {SM_AT_NESTED_TEST,     _T("Contained NextMap"),      _T("Contained NextMap does not pass AssertValid") },
  /*  1 */ {SM_AT_NESTED_TEST,     _T("Contained OrientMap"),    _T("Contained OrientMap does not pass AssertValid") },
  /*  2 */ {SM_AT_NESTED_TEST,     _T("Contained InvOrientMap"), _T("Contained InvOrientMap does not pass AssertValid") },
  /*  3 */ {SM_AT_NESTED_PROPERTY, _T("Simple OrientMap"),       _T("Contained OrientMap->m_pNextMap is not NULL") },
  /*  4 */ {SM_AT_NESTED_PROPERTY, _T("Simple OrientMap"),       _T("Contained OrientMap->m_pOrientMap is not NULL") },
  /*  5 */ {SM_AT_NESTED_PROPERTY, _T("Simple OrientMap"),       _T("Contained OrientMap->m_pInvOrientMap is not NULL") },
  /*  6 */ {SM_AT_NESTED_PROPERTY, _T("Simple InvOrientMap"),    _T("Contained InvOrientMap->m_pNextMap is not NULL") },
  /*  7 */ {SM_AT_NESTED_PROPERTY, _T("Simple InvOrientMap"),    _T("Contained InvOrientMap->m_pOrientMap is not NULL") },
  /*  8 */ {SM_AT_NESTED_PROPERTY, _T("Simple InvOrientMap"),    _T("Contained InvOrientMap->m_pInvOrientMap is not NULL") }
} ;

/*******************************************************************//**
PURPOSE:

NOTES: returns TRUE  = OK
               FALSE = Problem
***********************************************************************/
SmBoolean SmVolume::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  // check base class validity
  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmAObject::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // check NextMap
  SmBoolean bOK = (   m_pNextMap == NULL
                   || m_pNextMap->AssertValid(pAList, eTestLevel, eWalkTree, pTestRequests)) ;
  bRtn &= bOK ;
  SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (bRtn), _T("")) ;

  // check OrientMap
  bOK = (   m_pOrientMap == NULL 
         || m_pOrientMap->AssertValid(pAList, eTestLevel, eWalkTree, pTestRequests)) ;
  bRtn &= bOK ;
  SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (bRtn), _T("")) ;

  // check InvOrientMap
  bOK = (   m_pInvOrientMap == NULL 
         || m_pInvOrientMap->AssertValid(pAList, eTestLevel, eWalkTree, pTestRequests)) ;
  bRtn &= bOK ;
  SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (bRtn), _T("")) ;

  // simple OrientMap check - Orient maps are simple, i.e. they have no Next, Orient, or InvOrient maps
  SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (m_pOrientMap == NULL || m_pOrientMap->m_pNextMap == NULL), _T("")) ;
  SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (m_pOrientMap == NULL || m_pOrientMap->m_pOrientMap == NULL), _T("")) ;
  SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, (m_pOrientMap == NULL || m_pOrientMap->m_pInvOrientMap == NULL), _T("")) ;

  // simple InvOrientMap check - InvOrient maps are simple, i.e. they have no Next, Orient, or InvOrient maps
  SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, (m_pInvOrientMap == NULL || m_pInvOrientMap->m_pNextMap == NULL), _T("")) ;
  SM_ASSERT_BOOLEAN_REPORT(7, SM_LEVEL_0, (m_pInvOrientMap == NULL || m_pInvOrientMap->m_pOrientMap == NULL), _T("")) ;
  SM_ASSERT_BOOLEAN_REPORT(8, SM_LEVEL_0, (m_pInvOrientMap == NULL || m_pInvOrientMap->m_pInvOrientMap == NULL), _T("")) ;

  // all done
  return(bRtn) ;

} // end SmVolume::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmVolume::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport 
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return ( SmAObject::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmVolume::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmVolume::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: SmDiscontinuities3d Copy constructor

NOTES:
***********************************************************************/
SmDiscontinuities3d::SmDiscontinuities3d(const SmDiscontinuities3d &crOther)
{ 
  m_bHasDiscontinuities = crOther. m_bHasDiscontinuities ;
  m_eMinContinuity      = crOther. m_eMinContinuity ;    

  m_bUnalignedU = crOther.m_bUnalignedU ;
  m_eMinContU   = crOther.m_eMinContU ;
  m_sParamsU    = crOther.m_sParamsU ;   
  m_sContsU     = crOther.m_sContsU ;   

  m_bUnalignedV = crOther.m_bUnalignedV ;
  m_eMinContV   = crOther.m_eMinContV ;
  m_sParamsV    = crOther.m_sParamsV ;   
  m_sContsV     = crOther.m_sContsV ;   

  m_bUnalignedW = crOther.m_bUnalignedW ;
  m_eMinContW   = crOther.m_eMinContW ;
  m_sParamsW    = crOther.m_sParamsW ;   
  m_sContsW     = crOther.m_sContsW ;   

} // end SmDiscontinuities3d copy constructor

/*******************************************************************//**
PURPOSE: SmDiscontinuities3d Assignment operator

NOTES: 
***********************************************************************/
SmDiscontinuities3d & SmDiscontinuities3d::operator=(const SmDiscontinuities3d &crOther) 
{ 
  if(&crOther == this) 
    { return *this ; }

  m_bHasDiscontinuities = crOther. m_bHasDiscontinuities ;
  m_eMinContinuity      = crOther. m_eMinContinuity ;    

  m_bUnalignedU = crOther.m_bUnalignedU ;
  m_eMinContU   = crOther.m_eMinContU ;
  m_sParamsU    = crOther.m_sParamsU ;   
  m_sContsU     = crOther.m_sContsU ;   

  m_bUnalignedV = crOther.m_bUnalignedV ;
  m_eMinContV   = crOther.m_eMinContV ;
  m_sParamsV    = crOther.m_sParamsV ;   
  m_sContsV     = crOther.m_sContsV ;   

  m_bUnalignedW = crOther.m_bUnalignedW ;
  m_eMinContW   = crOther.m_eMinContW ;
  m_sParamsW    = crOther.m_sParamsW ;   
  m_sContsW     = crOther.m_sContsW ;   

  // all done
  return(*this) ; 

} // end SmDiscontinuities3d::operator=

/*******************************************************************//**
PURPOSE: Drop 3d vectors down into the parameter space
    defined by three volume tangent vectors

NOTES: When any of the tangent vectors are degenerate
                rUVW is set to [0 0 0] and SM_ERR is returned
***********************************************************************/
SmStatus smvol_DropVector 
  (const SmVector3d & rDU,               // in : Volume U_dir tangent 
   const SmVector3d & rDV,               // in : Volume V_dir tangent
   const SmVector3d & rDW,               // in : Volume V_dir tangent
   const SmVector3d & rDropVec3d,        // in : vector to drop
   SmVector3d       & rUVW)              // out: resulting UVW vector  [u v w]
                                         //      where rDropVec3d = u*rDU + v*rDV + w*rDW
{
  // locals
  double dDULengthSq   = rDU.LengthSquared() ;
  double dDVLengthSq   = rDV.LengthSquared() ;
  double dDWLengthSq   = rDW.LengthSquared() ;
  double dScaledZeroSq = SM_EFF_ZERO_SQ * (1.0 + smos_3Max(dDULengthSq, dDVLengthSq, dDWLengthSq)) ;

  // Solve a 3 by 3 linear equation for U, V, and W.
  // rDropVec3d = u*rDU + v*rDV + w*rDW
  // [ rDU.x  rDV.x  rDW.x] [ u ]   [ rDropVec3d.x ]
  // [ rDU.y  rDV.y  rDW.y] [ v ] = [ rDropVec3d.y ]
  // [ rDU.z  rDV.z  rDW.z] [ w ]   [ rDropVec3d.z ]

  // Cramer's rule
  // u = Det [ rDropVec3d.x  rDV.x  rDW.x]  v = Det [ rDU.x  rDropVec3d.x  rDW.x]  w = Det [ rDU.x  rDV.x  rDropVec3d.x] 
  //         [ rDropVec3d.y  rDV.y  rDW.y]          [ rDU.y  rDropVec3d.y  rDW.y]          [ rDU.y  rDV.y  rDropVec3d.y] 
  //         [ rDropVec3d.z  rDV.z  rDW.z]          [ rDU.z  rDropVec3d.z  rDW.z]          [ rDU.z  rDV.z  rDropVec3d.z] 
  //     ----------------------------------     ----------------------------------     ---------------------------------- 
  //     Det    [ rDU.x  rDV.x  rDW.x]          Det     [ rDU.x  rDV.x  rDW.x]         Det     [ rDU.x  rDV.x  rDW.x] 
  //            [ rDU.y  rDV.y  rDW.y]                  [ rDU.y  rDV.y  rDW.y]                 [ rDU.y  rDV.y  rDW.y] 
  //            [ rDU.z  rDV.z  rDW.z]                  [ rDU.z  rDV.z  rDW.z]                 [ rDU.z  rDV.z  rDW.z] 

  double dDen  = rDU.TripleProduct(rDV, rDW) ;
  double dNumU = rDropVec3d.TripleProduct(rDV,rDW) ;
  double dNumV = rDU.TripleProduct(rDropVec3d,rDW) ;
  double dNumW = rDU.TripleProduct(rDV,rDropVec3d) ;

  // no solution possible when rDU, rDV, rDW are degenerate
  //   1. zero length derivatives
  //   2. derivatives don't span all 3 space 
  //      (parallel, equal, or coplanar directions)
  if(   dDULengthSq < dScaledZeroSq
     || dDVLengthSq < dScaledZeroSq
     || dDWLengthSq < dScaledZeroSq
     || dDen      < SM_EFF_ZERO
     || dDen * dDen / (1.0 + (  dDULengthSq      // gwc: this check for degenerate denominator may not be right.
                              * dDVLengthSq
                              * dDWLengthSq)) < SM_EFF_ZERO_SQ) 
    {
      rUVW.x = 0.0;
      rUVW.y = 0.0;
      rUVW.z = 0.0;

      return SM_ERR;
    }

  // set solution
  rUVW.x = dNumU / dDen ;
  rUVW.y = dNumV / dDen ;
  rUVW.z = dNumW / dDen ;

  // all done
  return SM_SUCCESS;

} // end smvol_DropVector

/*******************************************************************//**
PURPOSE: Compute 1st derivative of CrvInVol(s) = D(C(s)) 
         using the chain rule given C'(s) 
         and all the D(u,v,w) 1st derivatives .

NOTES:  D = Volume(u,v,w) - called D (not V) to not be confused with parameter 'v'
***********************************************************************/
SmVector3d smvol_LiftFirstDerivative
  (const SmVector3d & crCs,     // in : Curve 1st deriv

   const SmVector3d & crDu,     // in : Volume 1st U deriv
   const SmVector3d & crDv,     // in : Volume 1st V deriv
   const SmVector3d & crDw)     // in : Volume 1st W deriv
{
  SmVector3d sRet =    crCs.x * crDu
                     + crCs.y * crDv
                     + crCs.z * crDw;
  return sRet;

} // end smvol_LiftFirstDerivative

/*******************************************************************//**
PURPOSE: Compute 2nd derivative of CrvInVol(s) = D(C(s)) 
         using the chain rule given C'(s) and C''(s) 
         and all the D(u,v,w) 1st and 2nd derivatives .

NOTES:  D = Volume(u,v,w) - called D (not V) to not be confused with parameter 'v'
***********************************************************************/
SmVector3d smvol_LiftSecondDerivative
  (const SmVector3d & crCs,     // in : Curve 1st deriv
   const SmVector3d & crCss,    // in : Curve 2nd deriv

   const SmVector3d & crDu,     // in : Volume 1st U deriv
   const SmVector3d & crDv,     // in : Volume 1st V deriv
   const SmVector3d & crDw,     // in : Volume 1st W deriv

   const SmVector3d & crDuu,    // in : Volume 2nd UU deriv
   const SmVector3d & crDuv,    // in : Volume 2nd UV deriv
   const SmVector3d & crDuw,    // in : Volume 2nd UW deriv
   const SmVector3d & crDvv,    // in : Volume 2nd VV deriv
   const SmVector3d & crDvw,    // in : Volume 2nd VW deriv
   const SmVector3d & crDww)    // in : Volume 2nd WW deriv
{
  SmVector3d sRet =          crCs.x * crCs.x * crDuu
                     +       crCs.y * crCs.y * crDvv
                     +       crCs.z * crCs.z * crDww
                     + 2.0 * crCs.x * crCs.y * crDuv
                     + 2.0 * crCs.x * crCs.z * crDuw
                     + 2.0 * crCs.y * crCs.z * crDvw
                     +       crCss.x * crDu
                     +       crCss.y * crDv
                     +       crCss.z * crDw;
  return sRet;

} // end smvol_LiftSecondDerivative

/*******************************************************************//**
PURPOSE: Compute 3rd derivative of CrvInVol(s) = D(C(s)) 
         using the chain rule given C'(s), C''(s), and C'''(s) 
         and all the D(u,v,w) 1st, 2nd, and 3rd derivatives .

NOTES:  D = Volume(u,v,w) - called D (not V) to not be confused with parameter 'v'
***********************************************************************/
SmVector3d smvol_LiftThirdDerivative
  (const SmVector3d & crCs,       // in : Curve 1st deriv 
   const SmVector3d & crCss,      // in : Curve 2nd deriv 
   const SmVector3d & crCsss,     // in : Curve 3rd deriv

   const SmVector3d & crDu,       // in : Volume 1st U deriv 
   const SmVector3d & crDv,       // in : Volume 1st V deriv 
   const SmVector3d & crDw,       // in : Volume 1st W deriv 

   const SmVector3d & crDuu,      // in : Volume 2nd UU deriv
   const SmVector3d & crDuv,      // in : Volume 2nd UV deriv
   const SmVector3d & crDuw,      // in : Volume 2nd UW deriv
   const SmVector3d & crDvv,      // in : Volume 2nd VV deriv
   const SmVector3d & crDvw,      // in : Volume 2nd VW deriv
   const SmVector3d & crDww,      // in : Volume 2nd WW deriv
   
   const SmVector3d & crDuuu,     // in : Volume 3rd UUU deriv
   const SmVector3d & crDuuv,     // in : Volume 3rd UUV deriv
   const SmVector3d & crDuuw,     // in : Volume 3rd UUW deriv
   const SmVector3d & crDuvv,     // in : Volume 3rd UVV deriv
   const SmVector3d & crDuvw,     // in : Volume 3rd UVW deriv
   const SmVector3d & crDuww,     // in : Volume 3rd UWW deriv
   const SmVector3d & crDvvv,     // in : Volume 3rd VVV deriv
   const SmVector3d & crDvvw,     // in : Volume 3rd VVW deriv
   const SmVector3d & crDvww,     // in : Volume 3rd VWW deriv
   const SmVector3d & crDwww)     // in : Volume 3rd WWW deriv
{
  SmVector3d sRet =          crCsss.x * crDu
                     +       crCsss.y * crDv
                     +       crCsss.z * crDw ;

  sRet           +=    3.0 * (crCss.x * crCs.x)                    * crDuu
                     + 3.0 * (crCss.x * crCs.y + crCss.y * crCs.x) * crDuv
                     + 3.0 * (crCss.x * crCs.z + crCss.z * crCs.x) * crDuw
                     + 3.0 * (crCss.y * crCs.y)                    * crDvv
                     + 3.0 * (crCss.y * crCs.z + crCss.z * crCs.y) * crDvw
                     + 3.0 * (crCss.z * crCs.z)                    * crDww ;

  sRet           +=          crCs.x * crCs.x * crCs.x * crDuuu
                     + 3.0 * crCs.x * crCs.x * crCs.y * crDuuv
                     + 3.0 * crCs.x * crCs.x * crCs.z * crDuuw
                     + 3.0 * crCs.x * crCs.y * crCs.y * crDuvv
                     + 6.0 * crCs.x * crCs.y * crCs.z * crDuvw
                     + 3.0 * crCs.x * crCs.z * crCs.z * crDuww
                     +       crCs.y * crCs.y * crCs.y * crDvvv
                     + 3.0 * crCs.y * crCs.y * crCs.z * crDvvw
                     + 3.0 * crCs.y * crCs.z * crCs.z * crDvww
                     +       crCs.z * crCs.z * crCs.z * crDwww ;

  return sRet;

} // end smvol_LiftThirdDerivative

/*******************************************************************//**
PURPOSE: Compute 1st derivative of SrfInVol(u,v) = D(S(u,v)) 
         using the chain rule given Su(u,v) and Sv(u,v)
         and all the D(x,y,z) 1st derivatives .

NOTES:  D = Volume(x,y,z) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v'
***********************************************************************/
void smvol_LiftFirstDerivative
  (const SmVector3d & crSu,     // in : surface 1st u deriv 
   const SmVector3d & crSv,     // in : surface 1st v deriv 

   const SmVector3d & crDx,     // in : Volume 1st X deriv
   const SmVector3d & crDy,     // in : Volume 1st Y deriv
   const SmVector3d & crDz,     // in : Volume 1st Z deriv

   SmVector3d       & rDu,      // out: Volume 1st u deriv
   SmVector3d       & rDv)      // out: Volume 1st v deriv
{
  rDu =   crSu.x * crDx
        + crSu.y * crDy
        + crSu.z * crDz ;

  rDv =   crSv.x * crDx
        + crSv.y * crDy
        + crSv.z * crDz ;

} // end smvol_LiftFirstDerivative

/*******************************************************************//**
PURPOSE: Compute 2nd derivative of SrfInVol(u,v) = D(S(u,v)) 
         using the chain rule given Su, Sv, Suu, Suv, and Svv
         and all the D(x,y,z) 1st and 2nd derivatives .

NOTES:  D = Volume(x,y,z) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v'
***********************************************************************/
void smvol_LiftSecondDerivative
  (const SmVector3d & crSu,     // in : surface 1st u deriv 
   const SmVector3d & crSv,     // in : surface 1st v deriv 

   const SmVector3d & crSuu,    // in : surface 2nd UU deriv 
   const SmVector3d & crSuv,    // in : surface 2nd UV deriv 
   const SmVector3d & crSvv,    // in : surface 2nd VV deriv 

   const SmVector3d & crDx,     // in : Volume 1st X deriv
   const SmVector3d & crDy,     // in : Volume 1st Y deriv
   const SmVector3d & crDz,     // in : Volume 1st Z deriv

   const SmVector3d & crDxx,    // in : Volume 2nd XX deriv
   const SmVector3d & crDxy,    // in : Volume 2nd XY deriv
   const SmVector3d & crDxz,    // in : Volume 2nd XZ deriv
   const SmVector3d & crDyy,    // in : Volume 2nd YY deriv
   const SmVector3d & crDyz,    // in : Volume 2nd YZ deriv
   const SmVector3d & crDzz,    // in : Volume 2nd ZZ deriv

   SmVector3d       & rDuu,     // out: volume 2nd UU deriv 
   SmVector3d       & rDuv,     // out: volume 2nd UV deriv 
   SmVector3d       & rDvv)     // out: volume 2nd VV deriv 
{
  rDuu =         crSu.x * crSu.x * crDxx
         +       crSu.y * crSu.y * crDyy
         +       crSu.z * crSu.z * crDzz
         + 2.0 * crSu.x * crSu.y * crDxy
         + 2.0 * crSu.x * crSu.z * crDxz
         + 2.0 * crSu.y * crSu.z * crDyz
         +       crSuu.x * crDx
         +       crSuu.y * crDy
         +       crSuu.z * crDz;

  rDuv =   crSv.x * crSu.x * crDxx
         + crSv.y * crSu.y * crDyy
         + crSv.z * crSu.z * crDzz
         + (crSv.x * crSu.y + crSv.y * crSu.x) * crDxy
         + (crSv.x * crSu.z + crSv.z * crSu.x) * crDxz
         + (crSv.y * crSu.z + crSv.z * crSu.y) * crDyz
         + crSuv.x * crDx
         + crSuv.y * crDy
         + crSuv.z * crDz;

  rDvv =         crSv.x * crSv.x * crDxx
         +       crSv.y * crSv.y * crDyy
         +       crSv.z * crSv.z * crDzz
         + 2.0 * crSv.x * crSv.y * crDxy
         + 2.0 * crSv.x * crSv.z * crDxz
         + 2.0 * crSv.y * crSv.z * crDyz
         +       crSvv.x * crDx
         +       crSvv.y * crDy
         +       crSvv.z * crDz;

} // end smvol_LiftSecondDerivative

/*******************************************************************//**
PURPOSE: Compute 3rd derivative of SrfInVol(u,v) = D(S(u,v)) 
         using the chain rule given Su, Sv, Suu, Suv, Svv,
         Suuu, Suuv, Suvv, Svvv
         and all the D(x,y,z) 1st, 2nd, and 3rd derivatives .

NOTES:  D = Volume(x,y,z) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v'
***********************************************************************/
void smvol_LiftThirdDerivative
  (const SmVector3d & crSu,     // in : surface 1st u deriv 
   const SmVector3d & crSv,     // in : surface 1st v deriv 

   const SmVector3d & crSuu,    // in : surface 2nd XX deriv  
   const SmVector3d & crSuv,    // in : surface 2nd XY deriv  
   const SmVector3d & crSvv,    // in : surface 2nd YY deriv  
                                                              
   const SmVector3d & crSuuu,   // in : surface 3rd XXX deriv 
   const SmVector3d & crSuuv,   // in : surface 3rd XXY deriv 
   const SmVector3d & crSuvv,   // in : surface 3rd XYY deriv 
   const SmVector3d & crSvvv,   // in : surface 3rd YYY deriv 
                                                              
   const SmVector3d & crDx,     // in : Volume 1st X deriv    
   const SmVector3d & crDy,     // in : Volume 1st Y deriv    
   const SmVector3d & crDz,     // in : Volume 1st Z deriv    
                                                              
   const SmVector3d & crDxx,    // in : Volume 2nd XX deriv   
   const SmVector3d & crDxy,    // in : Volume 2nd XY deriv   
   const SmVector3d & crDxz,    // in : Volume 2nd XZ deriv   
   const SmVector3d & crDyy,    // in : Volume 2nd YY deriv   
   const SmVector3d & crDyz,    // in : Volume 2nd YZ deriv   
   const SmVector3d & crDzz,    // in : Volume 2nd ZZ deriv   
                                                              
   const SmVector3d & crDxxx,   // in : Volume 3rd XXX deriv  
   const SmVector3d & crDxxy,   // in : Volume 3rd XXY deriv  
   const SmVector3d & crDxxz,   // in : Volume 3rd XXZ deriv  
   const SmVector3d & crDxyy,   // in : Volume 3rd XYY deriv  
   const SmVector3d & crDxyz,   // in : Volume 3rd XYZ deriv  
   const SmVector3d & crDxzz,   // in : Volume 3rd XZZ deriv  
   const SmVector3d & crDyyy,   // in : Volume 3rd YYY deriv  
   const SmVector3d & crDyyz,   // in : Volume 3rd YYZ deriv  
   const SmVector3d & crDyzz,   // in : Volume 3rd YZZ deriv  
   const SmVector3d & crDzzz,   // in : Volume 3rd ZZZ deriv  

   SmVector3d       & rDuuu,    // out: Volume 3rd UUU deriv
   SmVector3d       & rDuuv,    // out: Volume 3rd UUV deriv
   SmVector3d       & rDuvv,    // out: Volume 3rd UVV deriv
   SmVector3d       & rDvvv)    // out: Volume 3rd VVV deriv
{
  rDuuu  =         crSuuu.x * crDx
           +       crSuuu.y * crDy
           +       crSuuu.z * crDz  
           + 3.0 * (crSuu.x * crSu.x)                    * crDxx
           + 3.0 * (crSuu.x * crSu.y + crSuu.y * crSu.x) * crDxy
           + 3.0 * (crSuu.x * crSu.z + crSuu.z * crSu.x) * crDxz
           + 3.0 * (crSuu.y * crSu.y)                    * crDyy
           + 3.0 * (crSuu.y * crSu.z + crSuu.z * crSu.y) * crDyz
           + 3.0 * (crSuu.z * crSu.z)                    * crDzz  
           +       crSu.x * crSu.x * crSu.x * crDxxx
           + 3.0 * crSu.x * crSu.x * crSu.y * crDxxy
           + 3.0 * crSu.x * crSu.x * crSu.z * crDxxz
           + 3.0 * crSu.x * crSu.y * crSu.y * crDxyy
           + 6.0 * crSu.x * crSu.y * crSu.z * crDxyz
           + 3.0 * crSu.x * crSu.z * crSu.z * crDxzz
           +       crSu.y * crSu.y * crSu.y * crDyyy
           + 3.0 * crSu.y * crSu.y * crSu.z * crDyyz
           + 3.0 * crSu.y * crSu.z * crSu.z * crDyzz
           +       crSu.z * crSu.z * crSu.z * crDzzz ;

  rDuuv =   crSuuv.x * crDx
          + crSuuv.y * crDy
          + crSuuv.z * crDz

          + (2.0 * crSuv.x * crSu.x  + crSv.x * crSuu.x) * crDxx
          + (2.0 * crSuv.y * crSu.y  + crSv.y * crSuu.y) * crDyy
          + (2.0 * crSuv.z * crSu.z  + crSv.z * crSuu.z) * crDzz  
          + (2.0 * crSuv.x * crSu.y  + crSv.y * crSuu.x + 2.0 * crSu.x  * crSuv.y + crSv.x * crSuu.y) * crDxy
          + (2.0 * crSuv.x * crSu.z  + crSv.z * crSuu.x + 2.0 * crSu.x  * crSuv.z + crSv.x * crSuu.z) * crDxz
          + (2.0 * crSuv.y * crSu.z  + crSv.z * crSuu.y + 2.0 * crSu.y  * crSuv.z + crSv.y * crSuu.z) * crDyz

          + (      crSv.y * crSu.x * crSu.x + 2.0 * crSv.x * crSu.x * crSu.y) * crDxxy  
          + (      crSv.z * crSu.x * crSu.x + 2.0 * crSv.x * crSu.x * crSu.z) * crDxxz  
          + (      crSv.x * crSu.y * crSu.y + 2.0 * crSv.y * crSu.x * crSu.y) * crDxyy  
          + (2.0 * crSv.y * crSu.x * crSu.z + 2.0 * crSv.z * crSu.x * crSu.y + 2.0 * crSv.x * crSu.y * crSu.z) * crDxyz  
          + (      crSv.x * crSu.z * crSu.z + 2.0 * crSv.z * crSu.x * crSu.z) * crDxxz
          + (      crSv.z * crSu.y * crSu.y + 2.0 * crSv.y * crSu.y * crSu.z) * crDyyz  
          + (      crSv.y * crSu.z * crSu.z + 2.0 * crSv.z * crSu.y * crSu.z) * crDyzz  
          + (      crSv.x * crSu.x * crSu.x) * crDxxx 
          + (      crSv.y * crSu.y * crSu.y) * crDyyy
          + (      crSv.z * crSu.z * crSu.z) * crDzzz ; 
                 
  rDuvv =   crSuvv.x * crDx
          + crSuvv.y * crDy
          + crSuvv.z * crDz

          + (2.0 * crSuv.x * crSv.x  + crSu.x * crSvv.x) * crDxx
          + (2.0 * crSuv.y * crSv.y  + crSu.y * crSvv.y) * crDyy
          + (2.0 * crSuv.z * crSv.z  + crSu.z * crSvv.z) * crDzz  
          + (2.0 * crSuv.x * crSv.y  + crSu.y * crSvv.x + 2.0 * crSv.x  * crSuv.y + crSu.x * crSvv.y) * crDxy
          + (2.0 * crSuv.x * crSv.z  + crSu.z * crSvv.x + 2.0 * crSv.x  * crSuv.z + crSu.x * crSvv.z) * crDxz
          + (2.0 * crSuv.y * crSv.z  + crSu.z * crSvv.y + 2.0 * crSv.y  * crSuv.z + crSu.y * crSvv.z) * crDyz

          + (      crSu.y * crSv.x * crSv.x + 2.0 * crSu.x * crSv.x * crSv.y) * crDxxy  
          + (      crSu.z * crSv.x * crSv.x + 2.0 * crSu.x * crSv.x * crSv.z) * crDxxz  
          + (      crSu.x * crSv.y * crSv.y + 2.0 * crSu.y * crSv.x * crSv.y) * crDxyy  
          + (2.0 * crSu.y * crSv.x * crSv.z + 2.0 * crSu.z * crSv.x * crSv.y + 2.0 * crSu.x * crSv.y * crSv.z) * crDxyz  
          + (      crSu.x * crSv.z * crSv.z + 2.0 * crSu.z * crSv.x * crSv.z) * crDxxz
          + (      crSu.z * crSv.y * crSv.y + 2.0 * crSu.y * crSv.y * crSv.z) * crDyyz  
          + (      crSu.y * crSv.z * crSv.z + 2.0 * crSu.z * crSv.y * crSv.z) * crDyzz  
          + (      crSu.x * crSv.x * crSv.x) * crDxxx 
          + (      crSu.y * crSv.y * crSv.y) * crDyyy
          + (      crSu.z * crSv.z * crSv.z) * crDzzz ; 
                 
  rDvvv  =         crSvvv.x * crDx
           +       crSvvv.y * crDy
           +       crSvvv.z * crDz  
           + 3.0 * (crSvv.x * crSv.x)                    * crDxx
           + 3.0 * (crSvv.x * crSv.y + crSvv.y * crSv.x) * crDxy
           + 3.0 * (crSvv.x * crSv.z + crSvv.z * crSv.x) * crDxz
           + 3.0 * (crSvv.y * crSv.y)                    * crDyy
           + 3.0 * (crSvv.y * crSv.z + crSvv.z * crSv.y) * crDyz
           + 3.0 * (crSvv.z * crSv.z)                    * crDzz  
           +       crSv.x * crSv.x * crSv.x * crDxxx
           + 3.0 * crSv.x * crSv.x * crSv.y * crDxxy
           + 3.0 * crSv.x * crSv.x * crSv.z * crDxxz
           + 3.0 * crSv.x * crSv.y * crSv.y * crDxyy
           + 6.0 * crSv.x * crSv.y * crSv.z * crDxyz
           + 3.0 * crSv.x * crSv.z * crSv.z * crDxzz
           +       crSv.y * crSv.y * crSv.y * crDyyy
           + 3.0 * crSv.y * crSv.y * crSv.z * crDyyz
           + 3.0 * crSv.y * crSv.z * crSv.z * crDyzz
           +       crSv.z * crSv.z * crSv.z * crDzzz ;

} // end smvol_LiftThirdDerivative

/*******************************************************************//**
PURPOSE: Compute 1st derivative of VolInVol(u,v,w) = D(F(u,v,w)) 
         using the chain rule given Fu(u,v,w), Fv(u,v,w), and Fw(u,v,w)
         and all the D(x,y,z) 1st derivatives.

NOTES:  D = Volume(xyz) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v' and
        F = Volume(uvw) 
***********************************************************************/
void smvol_LiftFirstDerivative
  (const SmVector3d & crFu,     // in : BaseVolume 1st u deriv 
   const SmVector3d & crFv,     // in : BaseVolume 1st v deriv 
   const SmVector3d & crFw,     // in : BaseVolume 1st w deriv 

   const SmVector3d & crDx,     // in : CompoundingVolume 1st X deriv
   const SmVector3d & crDy,     // in : CompoundingVolume 1st Y deriv
   const SmVector3d & crDz,     // in : CompoundingVolume 1st Z deriv

   SmVector3d       & rDu,      // out: CompoundingVolume 1st u deriv
   SmVector3d       & rDv,      // out: CompoundingVolume 1st v deriv
   SmVector3d       & rDw)      // out: CompoundingVolume 1st v deriv
{
  rDu =   crFu.x * crDx
        + crFu.y * crDy
        + crFu.z * crDz ;

  rDv =   crFv.x * crDx
        + crFv.y * crDy
        + crFv.z * crDz ;

  rDw =   crFw.x * crDx
        + crFw.y * crDy
        + crFw.z * crDz ;

} // end smvol_LiftFirstDerivative

/*******************************************************************//**
PURPOSE: Compute 2nd derivative of VolInVol(u,v,w) = D(F(u,v,w)) 
         using the chain rule given Fu(u,v,w), Fv(u,v,w), Fw(u,v,w), and
         F 2nd derivs and all the D(x,y,z) 1st and 2nd derivatives.

NOTES:  D = Volume(xyz) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v' and
        F = Volume(uvw)
***********************************************************************/
void smvol_LiftSecondDerivative
  (const SmVector3d & crFu,     // in : BaseVolume 1st u deriv 
   const SmVector3d & crFv,     // in : BaseVolume 1st v deriv 
   const SmVector3d & crFw,     // in : BaseVolume 1st w deriv 

   const SmVector3d & crFuu,    // in : BaseVolume 2nd XX deriv
   const SmVector3d & crFuv,    // in : BaseVolume 2nd XY deriv
   const SmVector3d & crFuw,    // in : BaseVolume 2nd XZ deriv
   const SmVector3d & crFvv,    // in : BaseVolume 2nd YY deriv
   const SmVector3d & crFvw,    // in : BaseVolume 2nd YZ deriv
   const SmVector3d & crFww,    // in : BaseVolume 2nd ZZ deriv

   const SmVector3d & crDx,     // in : CompoundingVolume 1st X deriv
   const SmVector3d & crDy,     // in : CompoundingVolume 1st Y deriv
   const SmVector3d & crDz,     // in : CompoundingVolume 1st Z deriv

   const SmVector3d & crDxx,    // in : CompoundingVolume 2nd XX deriv
   const SmVector3d & crDxy,    // in : CompoundingVolume 2nd XY deriv
   const SmVector3d & crDxz,    // in : CompoundingVolume 2nd XZ deriv
   const SmVector3d & crDyy,    // in : CompoundingVolume 2nd YY deriv
   const SmVector3d & crDyz,    // in : CompoundingVolume 2nd YZ deriv
   const SmVector3d & crDzz,    // in : CompoundingVolume 2nd ZZ deriv

   SmVector3d       & rDuu,     // out: CompoundingVolume 2nd UU deriv
   SmVector3d       & rDuv,     // out: CompoundingVolume 2nd UV deriv
   SmVector3d       & rDuw,     // out: CompoundingVolume 2nd UW deriv
   SmVector3d       & rDvv,     // out: CompoundingVolume 2nd VV deriv
   SmVector3d       & rDvw,     // out: CompoundingVolume 2nd VW deriv
   SmVector3d       & rDww)     // out: CompoundingVolume 2nd WW deriv
{
  rDuu =         crFu.x * crFu.x * crDxx
         +       crFu.y * crFu.y * crDyy
         +       crFu.z * crFu.z * crDzz
         + 2.0 * crFu.x * crFu.y * crDxy
         + 2.0 * crFu.x * crFu.z * crDxz
         + 2.0 * crFu.y * crFu.z * crDyz
         +       crFuu.x * crDx
         +       crFuu.y * crDy
         +       crFuu.z * crDz;

  rDuv =   crFv.x * crFu.x * crDxx
         + crFv.y * crFu.y * crDyy
         + crFv.z * crFu.z * crDzz
         + (crFv.x * crFu.y + crFv.y * crFu.x) * crDxy
         + (crFv.x * crFu.z + crFv.z * crFu.x) * crDxz
         + (crFv.y * crFu.z + crFv.z * crFu.y) * crDyz
         + crFuv.x * crDx
         + crFuv.y * crDy
         + crFuv.z * crDz;

  rDuw =   crFw.x * crFu.x * crDxx
         + crFw.y * crFu.y * crDyy
         + crFw.z * crFu.z * crDzz
         + (crFw.x * crFu.y + crFw.y * crFu.x) * crDxy
         + (crFw.x * crFu.z + crFw.z * crFu.x) * crDxz
         + (crFw.y * crFu.z + crFw.z * crFu.y) * crDyz
         + crFuw.x * crDx
         + crFuw.y * crDy
         + crFuw.z * crDz;

  rDvv =         crFv.x * crFv.x * crDxx
         +       crFv.y * crFv.y * crDyy
         +       crFv.z * crFv.z * crDzz
         + 2.0 * crFv.x * crFv.y * crDxy
         + 2.0 * crFv.x * crFv.z * crDxz
         + 2.0 * crFv.y * crFv.z * crDyz
         +       crFvv.x * crDx
         +       crFvv.y * crDy
         +       crFvv.z * crDz;

  rDvw =   crFw.x * crFv.x * crDxx
         + crFw.y * crFv.y * crDyy
         + crFw.z * crFv.z * crDzz
         + (crFw.x * crFv.y + crFw.y * crFv.x) * crDxy
         + (crFw.x * crFv.z + crFw.z * crFv.x) * crDxz
         + (crFw.y * crFv.z + crFw.z * crFv.y) * crDyz
         + crFvw.x * crDx
         + crFvw.y * crDy
         + crFvw.z * crDz;

  rDww =         crFw.x * crFw.x * crDxx
         +       crFw.y * crFw.y * crDyy
         +       crFw.z * crFw.z * crDzz
         + 2.0 * crFw.x * crFw.y * crDxy
         + 2.0 * crFw.x * crFw.z * crDxz
         + 2.0 * crFw.y * crFw.z * crDyz
         +       crFww.x * crDx
         +       crFww.y * crDy
         +       crFww.z * crDz;

} // end smvol_LiftSecondDerivative

/*******************************************************************//**
PURPOSE: Compute 3rd derivative of VolInVol(u,v,w) = D(F(u,v,w)) 
         using the chain rule given Fu(u,v,w), Fv(u,v,w), Fw(u,v,w), and
         F 2nd and 3rd derivs and all the D(x,y,z) 1st, 2nd, and 3rd derivatives.

NOTES:  D = Volume(xyz) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v' and
        F = Volume(uvw)
***********************************************************************/
void smvol_LiftThirdDerivative
  (const SmVector3d & crFu,     // in : BaseVolume 1st U deriv    
   const SmVector3d & crFv,     // in : BaseVolume 1st V deriv    
   const SmVector3d & crFw,     // in : BaseVolume 1st W deriv    
                                                              
   const SmVector3d & crFuu,    // in : BaseVolume 2nd UU deriv   
   const SmVector3d & crFuv,    // in : BaseVolume 2nd UV deriv   
   const SmVector3d & crFuw,    // in : BaseVolume 2nd UW deriv   
   const SmVector3d & crFvv,    // in : BaseVolume 2nd VV deriv   
   const SmVector3d & crFvw,    // in : BaseVolume 2nd VW deriv   
   const SmVector3d & crFww,    // in : BaseVolume 2nd WW deriv   
                                                              
   const SmVector3d & crFuuu,   // in : BaseVolume 3rd UUU deriv  
   const SmVector3d & crFuuv,   // in : BaseVolume 3rd UUV deriv  
   const SmVector3d & crFuuw,   // in : BaseVolume 3rd UUW deriv  
   const SmVector3d & crFuvv,   // in : BaseVolume 3rd UVV deriv  
   const SmVector3d & crFuvw,   // in : BaseVolume 3rd UVW deriv  
   const SmVector3d & crFuww,   // in : BaseVolume 3rd UWW deriv  
   const SmVector3d & crFvvv,   // in : BaseVolume 3rd VVV deriv  
   const SmVector3d & crFvvw,   // in : BaseVolume 3rd VVW deriv  
   const SmVector3d & crFvww,   // in : BaseVolume 3rd VWW deriv  
   const SmVector3d & crFwww,   // in : BaseVolume 3rd WWW deriv  
                                                              
   const SmVector3d & crDx,     // in : CompoundingVolume 1st X deriv    
   const SmVector3d & crDy,     // in : CompoundingVolume 1st Y deriv    
   const SmVector3d & crDz,     // in : CompoundingVolume 1st Z deriv    
                                                              
   const SmVector3d & crDxx,    // in : CompoundingVolume 2nd XX deriv   
   const SmVector3d & crDxy,    // in : CompoundingVolume 2nd XY deriv   
   const SmVector3d & crDxz,    // in : CompoundingVolume 2nd XZ deriv   
   const SmVector3d & crDyy,    // in : CompoundingVolume 2nd YY deriv   
   const SmVector3d & crDyz,    // in : CompoundingVolume 2nd YZ deriv   
   const SmVector3d & crDzz,    // in : CompoundingVolume 2nd ZZ deriv   
                                                              
   const SmVector3d & crDxxx,   // in : CompoundingVolume 3rd XXX deriv  
   const SmVector3d & crDxxy,   // in : CompoundingVolume 3rd XXY deriv  
   const SmVector3d & crDxxz,   // in : CompoundingVolume 3rd XXZ deriv  
   const SmVector3d & crDxyy,   // in : CompoundingVolume 3rd XYY deriv  
   const SmVector3d & crDxyz,   // in : CompoundingVolume 3rd XYZ deriv  
   const SmVector3d & crDxzz,   // in : CompoundingVolume 3rd XZZ deriv  
   const SmVector3d & crDyyy,   // in : CompoundingVolume 3rd YYY deriv  
   const SmVector3d & crDyyz,   // in : CompoundingVolume 3rd YYZ deriv  
   const SmVector3d & crDyzz,   // in : CompoundingVolume 3rd YZZ deriv  
   const SmVector3d & crDzzz,   // in : CompoundingVolume 3rd ZZZ deriv  

   SmVector3d       & rDuuu,    // out: Volume 3rd UUU deriv  
   SmVector3d       & rDuuv,    // out: Volume 3rd UUV deriv  
   SmVector3d       & rDuuw,    // out: Volume 3rd UUW deriv  
   SmVector3d       & rDuvv,    // out: Volume 3rd UVV deriv  
   SmVector3d       & rDuvw,    // out: Volume 3rd UVW deriv  
   SmVector3d       & rDuww,    // out: Volume 3rd UWW deriv  
   SmVector3d       & rDvvv,    // out: Volume 3rd VVV deriv  
   SmVector3d       & rDvvw,    // out: Volume 3rd VVW deriv  
   SmVector3d       & rDvww,    // out: Volume 3rd VWW deriv  
   SmVector3d       & rDwww)    // out: Volume 3rd WWW deriv  
{
  rDuuu  =         crFuuu.x * crDx
           +       crFuuu.y * crDy
           +       crFuuu.z * crDz  
           + 3.0 * (crFuu.x * crFu.x)                    * crDxx
           + 3.0 * (crFuu.x * crFu.y + crFuu.y * crFu.x) * crDxy
           + 3.0 * (crFuu.x * crFu.z + crFuu.z * crFu.x) * crDxz
           + 3.0 * (crFuu.y * crFu.y)                    * crDyy
           + 3.0 * (crFuu.y * crFu.z + crFuu.z * crFu.y) * crDyz
           + 3.0 * (crFuu.z * crFu.z)                    * crDzz  
           +       crFu.x * crFu.x * crFu.x * crDxxx
           + 3.0 * crFu.x * crFu.x * crFu.y * crDxxy
           + 3.0 * crFu.x * crFu.x * crFu.z * crDxxz
           + 3.0 * crFu.x * crFu.y * crFu.y * crDxyy
           + 6.0 * crFu.x * crFu.y * crFu.z * crDxyz
           + 3.0 * crFu.x * crFu.z * crFu.z * crDxzz
           +       crFu.y * crFu.y * crFu.y * crDyyy
           + 3.0 * crFu.y * crFu.y * crFu.z * crDyyz
           + 3.0 * crFu.y * crFu.z * crFu.z * crDyzz
           +       crFu.z * crFu.z * crFu.z * crDzzz ;

  rDuuv =   crFuuv.x * crDx
          + crFuuv.y * crDy
          + crFuuv.z * crDz

          + (2.0 * crFuv.x * crFu.x  + crFv.x * crFuu.x) * crDxx
          + (2.0 * crFuv.y * crFu.y  + crFv.y * crFuu.y) * crDyy
          + (2.0 * crFuv.z * crFu.z  + crFv.z * crFuu.z) * crDzz  
          + (2.0 * crFuv.x * crFu.y  + crFv.y * crFuu.x + 2.0 * crFu.x  * crFuv.y + crFv.x * crFuu.y) * crDxy
          + (2.0 * crFuv.x * crFu.z  + crFv.z * crFuu.x + 2.0 * crFu.x  * crFuv.z + crFv.x * crFuu.z) * crDxz
          + (2.0 * crFuv.y * crFu.z  + crFv.z * crFuu.y + 2.0 * crFu.y  * crFuv.z + crFv.y * crFuu.z) * crDyz

          + (      crFv.y * crFu.x * crFu.x + 2.0 * crFv.x * crFu.x * crFu.y) * crDxxy  
          + (      crFv.z * crFu.x * crFu.x + 2.0 * crFv.x * crFu.x * crFu.z) * crDxxz  
          + (      crFv.x * crFu.y * crFu.y + 2.0 * crFv.y * crFu.x * crFu.y) * crDxyy  
          + (2.0 * crFv.y * crFu.x * crFu.z + 2.0 * crFv.z * crFu.x * crFu.y + 2.0 * crFv.x * crFu.y * crFu.z) * crDxyz  
          + (      crFv.x * crFu.z * crFu.z + 2.0 * crFv.z * crFu.x * crFu.z) * crDxxz
          + (      crFv.z * crFu.y * crFu.y + 2.0 * crFv.y * crFu.y * crFu.z) * crDyyz  
          + (      crFv.y * crFu.z * crFu.z + 2.0 * crFv.z * crFu.y * crFu.z) * crDyzz  
          + (      crFv.x * crFu.x * crFu.x) * crDxxx 
          + (      crFv.y * crFu.y * crFu.y) * crDyyy
          + (      crFv.z * crFu.z * crFu.z) * crDzzz ; 
                 
  rDuuw =   crFuuw.x * crDx
          + crFuuw.y * crDy
          + crFuuw.z * crDz

          + (2.0 * crFuw.x * crFu.x  + crFw.x * crFuu.x) * crDxx
          + (2.0 * crFuw.y * crFu.y  + crFw.y * crFuu.y) * crDyy
          + (2.0 * crFuw.z * crFu.z  + crFw.z * crFuu.z) * crDzz  
          + (2.0 * crFuw.x * crFu.y  + crFw.y * crFuu.x + 2.0 * crFu.x  * crFuw.y + crFw.x * crFuu.y) * crDxy
          + (2.0 * crFuw.x * crFu.z  + crFw.z * crFuu.x + 2.0 * crFu.x  * crFuw.z + crFw.x * crFuu.z) * crDxz
          + (2.0 * crFuw.y * crFu.z  + crFw.z * crFuu.y + 2.0 * crFu.y  * crFuw.z + crFw.y * crFuu.z) * crDyz

          + (      crFw.y * crFu.x * crFu.x + 2.0 * crFw.x * crFu.x * crFu.y) * crDxxy  
          + (      crFw.z * crFu.x * crFu.x + 2.0 * crFw.x * crFu.x * crFu.z) * crDxxz  
          + (      crFw.x * crFu.y * crFu.y + 2.0 * crFw.y * crFu.x * crFu.y) * crDxyy  
          + (2.0 * crFw.y * crFu.x * crFu.z + 2.0 * crFw.z * crFu.x * crFu.y + 2.0 * crFw.x * crFu.y * crFu.z) * crDxyz  
          + (      crFw.x * crFu.z * crFu.z + 2.0 * crFw.z * crFu.x * crFu.z) * crDxxz
          + (      crFw.z * crFu.y * crFu.y + 2.0 * crFw.y * crFu.y * crFu.z) * crDyyz  
          + (      crFw.y * crFu.z * crFu.z + 2.0 * crFw.z * crFu.y * crFu.z) * crDyzz  
          + (      crFw.x * crFu.x * crFu.x) * crDxxx 
          + (      crFw.y * crFu.y * crFu.y) * crDyyy
          + (      crFw.z * crFu.z * crFu.z) * crDzzz ; 
                 
  rDuvv =   crFuvv.x * crDx
          + crFuvv.y * crDy
          + crFuvv.z * crDz

          + (2.0 * crFuv.x * crFv.x  + crFu.x * crFvv.x) * crDxx
          + (2.0 * crFuv.y * crFv.y  + crFu.y * crFvv.y) * crDyy
          + (2.0 * crFuv.z * crFv.z  + crFu.z * crFvv.z) * crDzz  
          + (2.0 * crFuv.x * crFv.y  + crFu.y * crFvv.x + 2.0 * crFv.x  * crFuv.y + crFu.x * crFvv.y) * crDxy
          + (2.0 * crFuv.x * crFv.z  + crFu.z * crFvv.x + 2.0 * crFv.x  * crFuv.z + crFu.x * crFvv.z) * crDxz
          + (2.0 * crFuv.y * crFv.z  + crFu.z * crFvv.y + 2.0 * crFv.y  * crFuv.z + crFu.y * crFvv.z) * crDyz

          + (      crFu.y * crFv.x * crFv.x + 2.0 * crFu.x * crFv.x * crFv.y) * crDxxy  
          + (      crFu.z * crFv.x * crFv.x + 2.0 * crFu.x * crFv.x * crFv.z) * crDxxz  
          + (      crFu.x * crFv.y * crFv.y + 2.0 * crFu.y * crFv.x * crFv.y) * crDxyy  
          + (2.0 * crFu.y * crFv.x * crFv.z + 2.0 * crFu.z * crFv.x * crFv.y + 2.0 * crFu.x * crFv.y * crFv.z) * crDxyz  
          + (      crFu.x * crFv.z * crFv.z + 2.0 * crFu.z * crFv.x * crFv.z) * crDxxz
          + (      crFu.z * crFv.y * crFv.y + 2.0 * crFu.y * crFv.y * crFv.z) * crDyyz  
          + (      crFu.y * crFv.z * crFv.z + 2.0 * crFu.z * crFv.y * crFv.z) * crDyzz  
          + (      crFu.x * crFv.x * crFv.x) * crDxxx 
          + (      crFu.y * crFv.y * crFv.y) * crDyyy
          + (      crFu.z * crFv.z * crFv.z) * crDzzz ; 
  
  rDuvw = + crFuvw.x * crDx
          + crFuvw.y * crDy
          + crFuvw.z * crDz

          + (  crFvw.x * crFu.x  + crFw.x  * crFuv.x + crFv.x * crFuw.x) * crDxx
          + (  crFvw.x * crFu.y  + crFvw.y * crFu.x  + crFw.y * crFuv.x                      
             + crFv.x  * crFuw.y + crFv.y  * crFuw.x + crFw.x * crFuv.y) * crDxy
          + (  crFvw.x * crFu.z  + crFvw.z * crFu.x  + crFw.z * crFuv.x
             + crFv.x  * crFuw.z + crFv.z  * crFuw.x + crFw.x * crFuv.z) * crDxz
          + (  crFvw.y * crFu.y  + crFw.y  * crFuv.y + crFv.y * crFuw.y) * crDyy
          + (  crFvw.y * crFu.z  + crFvw.z * crFu.y  + crFw.z * crFuv.y
             + crFv.y  * crFuw.z + crFv.z  * crFuw.y + crFw.y * crFuv.z) * crDyz
          + (  crFvw.z * crFu.z  + crFw.z  * crFuv.z + crFv.z * crFuw.z) * crDzz

          + (  crFw.x * crFv.x * crFu.x) * crDxxx  
          + (  crFw.y * crFv.x * crFu.x + crFw.x * (crFv.x * crFu.y + crFv.y * crFu.x)) * crDxxy 
          + (  crFw.z * crFv.x * crFu.x + crFw.x * (crFv.x * crFu.z + crFv.z * crFu.x)) * crDxxz
          + (  crFw.x * crFv.y * crFu.y + crFw.y * (crFv.x * crFu.y + crFv.y * crFu.x)) * crDxyy
          + (  crFw.z * (crFv.x * crFu.y + crFv.y * crFu.x)
             + crFw.y * (crFv.x * crFu.z + crFv.z * crFu.x)
             + crFw.x * (crFv.y * crFu.z + crFv.z * crFu.y)) * crDxyz
          + (  crFw.y * crFv.y * crFu.y) * crDyyy 
          + (  crFw.z * crFv.y * crFu.y + crFw.y * (crFv.y * crFu.z + crFv.z * crFu.y)) * crDyyz
          + (  crFw.x * crFv.z * crFu.z + crFw.z * (crFv.x * crFu.z + crFv.z * crFu.x)) * crDxzz
          + (  crFw.y * crFv.z * crFu.z + crFw.z * (crFv.y * crFu.z + crFv.z * crFu.y)) * crDyzz
          + (  crFw.z * crFv.z * crFu.z) * crDzzz ;

  rDuww =   crFuww.x * crDx
          + crFuww.y * crDy
          + crFuww.z * crDz

          + (2.0 * crFuv.x * crFv.x  + crFu.x * crFww.x) * crDxx
          + (2.0 * crFuv.y * crFv.y  + crFu.y * crFww.y) * crDyy
          + (2.0 * crFuv.z * crFv.z  + crFu.z * crFww.z) * crDzz  
          + (2.0 * crFuv.x * crFv.y  + crFu.y * crFww.x + 2.0 * crFv.x  * crFuv.y + crFu.x * crFww.y) * crDxy
          + (2.0 * crFuv.x * crFv.z  + crFu.z * crFww.x + 2.0 * crFv.x  * crFuv.z + crFu.x * crFww.z) * crDxz
          + (2.0 * crFuv.y * crFv.z  + crFu.z * crFww.y + 2.0 * crFv.y  * crFuv.z + crFu.y * crFww.z) * crDyz

          + (      crFu.y * crFv.x * crFv.x + 2.0 * crFu.x * crFv.x * crFv.y) * crDxxy  
          + (      crFu.z * crFv.x * crFv.x + 2.0 * crFu.x * crFv.x * crFv.z) * crDxxz  
          + (      crFu.x * crFv.y * crFv.y + 2.0 * crFu.y * crFv.x * crFv.y) * crDxyy  
          + (2.0 * crFu.y * crFv.x * crFv.z + 2.0 * crFu.z * crFv.x * crFv.y + 2.0 * crFu.x * crFv.y * crFv.z) * crDxyz  
          + (      crFu.x * crFv.z * crFv.z + 2.0 * crFu.z * crFv.x * crFv.z) * crDxxz
          + (      crFu.z * crFv.y * crFv.y + 2.0 * crFu.y * crFv.y * crFv.z) * crDyyz  
          + (      crFu.y * crFv.z * crFv.z + 2.0 * crFu.z * crFv.y * crFv.z) * crDyzz  
          + (      crFu.x * crFv.x * crFv.x) * crDxxx 
          + (      crFu.y * crFv.y * crFv.y) * crDyyy
          + (      crFu.z * crFv.z * crFv.z) * crDzzz ; 
                 
  rDvvv  =         crFvvv.x * crDx
           +       crFvvv.y * crDy
           +       crFvvv.z * crDz  
           + 3.0 * (crFvv.x * crFv.x)                    * crDxx
           + 3.0 * (crFvv.x * crFv.y + crFvv.y * crFv.x) * crDxy
           + 3.0 * (crFvv.x * crFv.z + crFvv.z * crFv.x) * crDxz
           + 3.0 * (crFvv.y * crFv.y)                    * crDyy
           + 3.0 * (crFvv.y * crFv.z + crFvv.z * crFv.y) * crDyz
           + 3.0 * (crFvv.z * crFv.z)                    * crDzz  
           +       crFv.x * crFv.x * crFv.x * crDxxx
           + 3.0 * crFv.x * crFv.x * crFv.y * crDxxy
           + 3.0 * crFv.x * crFv.x * crFv.z * crDxxz
           + 3.0 * crFv.x * crFv.y * crFv.y * crDxyy
           + 6.0 * crFv.x * crFv.y * crFv.z * crDxyz
           + 3.0 * crFv.x * crFv.z * crFv.z * crDxzz
           +       crFv.y * crFv.y * crFv.y * crDyyy
           + 3.0 * crFv.y * crFv.y * crFv.z * crDyyz
           + 3.0 * crFv.y * crFv.z * crFv.z * crDyzz
           +       crFv.z * crFv.z * crFv.z * crDzzz ;

  rDvvw =   crFvvw.x * crDx
          + crFvvw.y * crDy
          + crFvvw.z * crDz

          + (2.0 * crFvw.x * crFv.x  + crFw.x * crFvv.x) * crDxx
          + (2.0 * crFvw.y * crFv.y  + crFw.y * crFvv.y) * crDyy
          + (2.0 * crFvw.z * crFv.z  + crFw.z * crFvv.z) * crDzz  
          + (2.0 * crFvw.x * crFv.y  + crFw.y * crFvv.x + 2.0 * crFv.x  * crFvw.y + crFw.x * crFvv.y) * crDxy
          + (2.0 * crFvw.x * crFv.z  + crFw.z * crFvv.x + 2.0 * crFv.x  * crFvw.z + crFw.x * crFvv.z) * crDxz
          + (2.0 * crFvw.y * crFv.z  + crFw.z * crFvv.y + 2.0 * crFv.y  * crFvw.z + crFw.y * crFvv.z) * crDyz

          + (      crFw.y * crFv.x * crFv.x + 2.0 * crFw.x * crFv.x * crFv.y) * crDxxy  
          + (      crFw.z * crFv.x * crFv.x + 2.0 * crFw.x * crFv.x * crFv.z) * crDxxz  
          + (      crFw.x * crFv.y * crFv.y + 2.0 * crFw.y * crFv.x * crFv.y) * crDxyy  
          + (2.0 * crFw.y * crFv.x * crFv.z + 2.0 * crFw.z * crFv.x * crFv.y + 2.0 * crFw.x * crFv.y * crFv.z) * crDxyz  
          + (      crFw.x * crFv.z * crFv.z + 2.0 * crFw.z * crFv.x * crFv.z) * crDxxz
          + (      crFw.z * crFv.y * crFv.y + 2.0 * crFw.y * crFv.y * crFv.z) * crDyyz  
          + (      crFw.y * crFv.z * crFv.z + 2.0 * crFw.z * crFv.y * crFv.z) * crDyzz  
          + (      crFw.x * crFv.x * crFv.x) * crDxxx 
          + (      crFw.y * crFv.y * crFv.y) * crDyyy
          + (      crFw.z * crFv.z * crFv.z) * crDzzz ; 

  rDvww =   crFvww.x * crDx
          + crFvww.y * crDy
          + crFvww.z * crDz

          + (2.0 * crFvv.x * crFv.x  + crFv.x * crFww.x) * crDxx
          + (2.0 * crFvv.y * crFv.y  + crFv.y * crFww.y) * crDyy
          + (2.0 * crFvv.z * crFv.z  + crFv.z * crFww.z) * crDzz  
          + (2.0 * crFvv.x * crFv.y  + crFv.y * crFww.x + 2.0 * crFv.x  * crFvv.y + crFv.x * crFww.y) * crDxy
          + (2.0 * crFvv.x * crFv.z  + crFv.z * crFww.x + 2.0 * crFv.x  * crFvv.z + crFv.x * crFww.z) * crDxz
          + (2.0 * crFvv.y * crFv.z  + crFv.z * crFww.y + 2.0 * crFv.y  * crFvv.z + crFv.y * crFww.z) * crDyz

          + (      crFv.y * crFv.x * crFv.x + 2.0 * crFv.x * crFv.x * crFv.y) * crDxxy  
          + (      crFv.z * crFv.x * crFv.x + 2.0 * crFv.x * crFv.x * crFv.z) * crDxxz  
          + (      crFv.x * crFv.y * crFv.y + 2.0 * crFv.y * crFv.x * crFv.y) * crDxyy  
          + (2.0 * crFv.y * crFv.x * crFv.z + 2.0 * crFv.z * crFv.x * crFv.y + 2.0 * crFv.x * crFv.y * crFv.z) * crDxyz  
          + (      crFv.x * crFv.z * crFv.z + 2.0 * crFv.z * crFv.x * crFv.z) * crDxxz
          + (      crFv.z * crFv.y * crFv.y + 2.0 * crFv.y * crFv.y * crFv.z) * crDyyz  
          + (      crFv.y * crFv.z * crFv.z + 2.0 * crFv.z * crFv.y * crFv.z) * crDyzz  
          + (      crFv.x * crFv.x * crFv.x) * crDxxx 
          + (      crFv.y * crFv.y * crFv.y) * crDyyy
          + (      crFv.z * crFv.z * crFv.z) * crDzzz ; 
                 
  rDwww  =         crFwww.x * crDx
           +       crFwww.y * crDy
           +       crFwww.z * crDz  
           + 3.0 * (crFww.x * crFw.x)                    * crDxx
           + 3.0 * (crFww.x * crFw.y + crFww.y * crFw.x) * crDxy
           + 3.0 * (crFww.x * crFw.z + crFww.z * crFw.x) * crDxz
           + 3.0 * (crFww.y * crFw.y)                    * crDyy
           + 3.0 * (crFww.y * crFw.z + crFww.z * crFw.y) * crDyz
           + 3.0 * (crFww.z * crFw.z)                    * crDzz  
           +       crFw.x * crFw.x * crFw.x * crDxxx
           + 3.0 * crFw.x * crFw.x * crFw.y * crDxxy
           + 3.0 * crFw.x * crFw.x * crFw.z * crDxxz
           + 3.0 * crFw.x * crFw.y * crFw.y * crDxyy
           + 6.0 * crFw.x * crFw.y * crFw.z * crDxyz
           + 3.0 * crFw.x * crFw.z * crFw.z * crDxzz
           +       crFw.y * crFw.y * crFw.y * crDyyy
           + 3.0 * crFw.y * crFw.y * crFw.z * crDyyz
           + 3.0 * crFw.y * crFw.z * crFw.z * crDyzz
           +       crFw.z * crFw.z * crFw.z * crDzzz ;

} // end smvol_LiftThirdDerivative

/*******************************************************************//**
PURPOSE: Project an evaluation set of position and derivatives values through
         a 2nd concatenated mapping as
             pConcatMap(uvw) = pSecondMap(pFirstMap(uvw))

NOTES:  
  1. The 2nd mapping evaluations are assumed to have been taken
     at the 1st mapping's output location.

  2. pConcatMap may equal either pFirstMap, pSecondMap
***********************************************************************/
SmStatus smvol_ConcatEvals
 ( ULONG        lHighestDeriv, // in : highest desired derivative, 
                               //       0 = position
                               //       1 = position & 1st Derivs
                               //       2 = position, 1st Derivs, 2nd Derivs
                               //       3 = position, 1st Derivs, 2nd Derivs, 3rd Derivs
   SmVector3d * pFirstMap,     // in : FirstMap(UVW) evals,           of pConcatMap(uvw) = pSecondMap(pFirstMap(uvw))
   SmVector3d * pSecondMap,    // in : SecondMap(pFirstMap[0]) evals, of pConcatMap(uvw) = pSecondMap(pFirstMap(uvw)) 
   SmVector3d * pConcatMap)    // in : ConcatMap(UVW) evals,          of pConcatMap(uvw) = pSecondMap(pFirstMap(uvw))
{
#define sv(u,v,w) ((u*(lHighestDeriv+1)+v)*(lHighestDeriv+1)+w)

  // locals - allow pConcatMap to equal either pFirstMap or pSecondMap
  SmVector3d  sTempMap[4*4*4] ;
  SmVector3d *pCalcMap = (pConcatMap == pFirstMap || pConcatMap == pSecondMap) ? sTempMap : pConcatMap ;

  // compounded position
  pCalcMap[0] = pSecondMap[0] ;

  // compounded 1st derivs
  if( lHighestDeriv >= 1)
    {
      smvol_LiftFirstDerivative( pFirstMap [sv(1,0,0)], pFirstMap [sv(0,1,0)], pFirstMap [sv(0,0,1)],
                                 pSecondMap [sv(1,0,0)], pSecondMap [sv(0,1,0)], pSecondMap [sv(0,0,1)],
                                 pCalcMap[sv(1,0,0)], pCalcMap[sv(0,1,0)], pCalcMap[sv(0,0,1)]) ;

      // 2nd derivs
      if( lHighestDeriv >= 2)
        {
          smvol_LiftSecondDerivative( pFirstMap [sv(1,0,0)], pFirstMap [sv(0,1,0)], pFirstMap [sv(0,0,1)],
                                      pFirstMap [sv(2,0,0)], pFirstMap [sv(1,1,0)], pFirstMap [sv(1,0,1)],
                                      pFirstMap [sv(0,2,0)], pFirstMap [sv(0,1,1)], pFirstMap [sv(0,0,2)],

                                      pSecondMap [sv(1,0,0)], pSecondMap [sv(0,1,0)], pSecondMap [sv(0,0,1)],
                                      pSecondMap [sv(2,0,0)], pSecondMap [sv(1,1,0)], pSecondMap [sv(1,0,1)],
                                      pSecondMap [sv(0,2,0)], pSecondMap [sv(0,1,1)], pSecondMap [sv(0,0,2)],

                                      pCalcMap[sv(2,0,0)], pCalcMap[sv(1,1,0)], pCalcMap[sv(1,0,1)],
                                      pCalcMap[sv(0,2,0)], pCalcMap[sv(0,1,1)], pCalcMap[sv(0,0,2)]) ;
      
          // 3rd derivs
          if( lHighestDeriv >= 3)
            {
              smvol_LiftThirdDerivative( pFirstMap [sv(1,0,0)], pFirstMap [sv(0,1,0)], pFirstMap [sv(0,0,1)],

                                         pFirstMap [sv(2,0,0)], pFirstMap [sv(1,1,0)], pFirstMap [sv(1,0,1)],
                                         pFirstMap [sv(0,2,0)], pFirstMap [sv(0,1,1)], pFirstMap [sv(0,0,2)],

                                         pFirstMap [sv(3,0,0)], pFirstMap [sv(2,1,0)], pFirstMap [sv(2,0,1)],
                                         pFirstMap [sv(1,2,0)], pFirstMap [sv(1,1,1)], pFirstMap [sv(1,0,2)],
                                         pFirstMap [sv(0,3,0)], pFirstMap [sv(0,2,1)], pFirstMap [sv(0,1,2)],
                                         pFirstMap [sv(0,0,3)], 

                                         pSecondMap [sv(1,0,0)], pSecondMap [sv(0,1,0)], pSecondMap [sv(0,0,1)],

                                         pSecondMap [sv(2,0,0)], pSecondMap [sv(1,1,0)], pSecondMap [sv(1,0,1)],
                                         pSecondMap [sv(0,2,0)], pSecondMap [sv(0,1,1)], pSecondMap [sv(0,0,2)],

                                         pSecondMap [sv(3,0,0)], pSecondMap [sv(2,1,0)], pSecondMap [sv(2,0,1)],
                                         pSecondMap [sv(1,2,0)], pSecondMap [sv(1,1,1)], pSecondMap [sv(1,0,2)],
                                         pSecondMap [sv(0,3,0)], pSecondMap [sv(0,2,1)], pSecondMap [sv(0,1,2)],
                                         pSecondMap [sv(0,0,3)],

                                         pCalcMap[sv(3,0,0)], pCalcMap[sv(2,1,0)], pCalcMap[sv(2,0,1)],
                                         pCalcMap[sv(1,2,0)], pCalcMap[sv(1,1,1)], pCalcMap[sv(1,0,2)],
                                         pCalcMap[sv(0,3,0)], pCalcMap[sv(0,2,1)], pCalcMap[sv(0,1,2)],
                                         pCalcMap[sv(0,0,3)]) ;
            } // end need 3rd derivs check
        } // end need 2nd derivs check
    } // end need 1st derivs check

  // when needed - copy pCalcMap into output
  if(pCalcMap != pConcatMap)
    {
      // smallest size of pConcatMap that will hold defined values
      ULONG lCnt = (lHighestDeriv + 1) * (lHighestDeriv + 1) * (lHighestDeriv + 1) ;

      // copy array block 
      SER(smos_MemCpy(pConcatMap, pCalcMap, lCnt * sizeof(SmVector3d), lCnt * sizeof(SmVector3d) )) ;

    } // end need to set output check
#undef sv

  // all done
  return(SM_SUCCESS) ;

} // end smvol_ConcatEvals
