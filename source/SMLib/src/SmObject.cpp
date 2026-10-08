// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmObject.cpp
* PURPOSE: Implementation of methods for SmObject
**********************************************************************/

#include "StdAfx.h"

#include <SmObject.h>
#include <SmContext.h>
#include <stdio.h>           // fopen(), fclose(), struct FILE
#include <SmAssertArray.h>

#include <SmFace.h>          // used only by SmObject::GetObjectColor()
#include <SmEdge.h>          // used only by SmObject::GetObjectColor()
#include <SmPoly.h>

#include <SmSurface.h>
#include <SmCurve.h>
#include <SmAttribute.h>     // used only by SmObject::GetObjectColor()
       // used only by SmObject::GetObjectColor()

#include <SmThreadLocalStorage.h>  // SmThreadLocalStorage::GetTLS()->SetBeenThroughNew() ;
//      #include <SmMemBlockMgr.h>

#ifdef SM_GFX_CODE
 #include <SmGraphicsExtern.h>
#else
 static SmVector3d s_NonGFXDefaultColor(0,0,0) ;
 static SmVector3d s_NonGFXDefaultShadingColor(.7,.7,.7) ;
 static SmVector3d s_NonGFXWireEdgeColor(0,1,0) ;                // color for wire   edges when drawn from Breps
 static SmVector3d s_NonGFXLaminaEdgeColor(0,0,1) ;              // color for lamina edges when drawn from Breps
 static SmVector3d s_NonGFXCurveControlPolygonColor(1,0,0) ;     // color for edge and surface control polygons
 static SmVector3d s_NonGFXSurfaceControlPolygonColor(0,0,1) ;   // color for edge and surface control polygons
 static SmVector3d s_NonGFXVolumeControlPolygonColor(0,0,1) ;  
   
 static SmVector3d s_NonGFXSurfaceNormalColor(1,0,0) ;           // color for surface normals pointing out of the screen
 static SmVector3d s_NonGFXNegSurfaceNormalColor(0,0,1) ;        // color for surface normals pointing into the screen
 static SmVector3d s_NonGFXSurfaceNormalInfiniteColor(0,1,0) ;   // color for surface normals pointing into infinite region
 static SmVector3d s_NonGFXSurfaceNormalSolidColor(0,0,1) ;      // color for surface normals pointing into a solid region
 static SmVector3d s_NonGFXSurfaceNormalVoidColor(1,0,0) ;       // color for surface normals pointing into a void region

 static SmVector3d s_NonGFXCurvatureColor(1,1,0) ;               // color for curvature combs
 static SmVector3d s_NonGFXCurvature2Color(1,.65,0) ;            // color for curvature2 combs
 static SmVector3d s_NonGFXSpeedColor(1,.5,0) ;                  // color for speed combs
 static SmVector3d s_NonGFXKnotColor(0,0,1) ;                    // color for curve knots
 static SmVector3d s_NonGFXVaryCrossHatchColor(1,1,0) ;          // color for varied cross hatch lines
 static SmVector3d s_NonGFXHighCountCurveColor(1,.5,.1) ;        // color for high ControlPoint count curves
 static SmVector3d s_NonGFXBaseSurfaceColor(.2,.2,.5) ;          // color for base surface of SmOffsetSurface Objects
 static SmVector3d s_NonGFXExtendedSurfaceColor(.2,.5,.2) ;      // color for extended surface of SmOffsetSurface Objects
 static SmVector3d s_NonGFXNeighborColor(1,0,0) ;                // color for neighbors in DrawNeighbor functions
 static SmVector3d s_NonGFXNeighborMateColor(0,1,0) ;            // color for neighbor Mates in DrawNeighbor functions
 static SmVector3d s_NonGFXHotPointColor(1,0,0) ;                // color for pickable hot points
 static SmVector3d s_NonGFXHotPointSelectColor(1,0,0) ;          // color for non-picable portions of hot point displays
 static SmVector3d s_NonGFXHotPointFillColor(1,1,0) ;            // color for selected hot points
 static SmVector3d s_NonGFXHotPointHighlightColor(1,0,0) ;       // color for highlighted hot points
 static SmVector3d s_NonGFXTrackTargetColor(.05,.05,.05) ;       // color for tracking mode display
 static SmVector3d s_NonGFXTrackInfoColor(.7,.7,.7) ;            // color for tracking orientation display
 static SmVector3d s_NonGFXTestGeometryColor(0,.8,.1) ;          // color for test geometry under construction
 static SmVector3d s_NonGFXTestGapPointColor(1,0,0) ;            //  color for gap points          
 static SmVector3d s_NonGFXTestVertexEdgeGapColor(1,0,1) ;       //  color for vertexEdge gap lines
 static SmVector3d s_NonGFXTestVertexFaceGapColor(0,1,0) ;       //  color for VertexFace gap lines
 static SmVector3d s_NonGFXTestEdgeFaceGapColor(0,1,1) ;         //  color for EdgeFace gap lines
 static SmVector3d s_NonGFXTestEdgeEdgeGapColor(0,.5,1) ;        //  color for EdgeEdge gap lines
 static SmVector3d s_NonGFXTestVertexFaceTrimCurveGapColor(.7,.2,.1) ; // color for VertexUVTrimCurve gap lines
 static SmVector3d s_NonGFXTestEdgeFaceTrimCurveGapColor(.2,.7,.1) ;   // color fo EdgeUVTrimCurve gap line
 static SmVector3d s_NonGFXTestInTolGapColor(1,.5,0) ;           //  color for In Tolerance GapFunction gap segments
 static SmVector3d s_NonGFXTestOutTolGapColor(.3,.3,.3) ;        //  color for Out of Tolerance GapFunction gap segments  

#endif // not defined SM_GFX_CODE

/*******************************************************************//**
PURPOSE: SmObject class constructor

NOTES:
***********************************************************************/
SmObject::SmObject
 (SmBoolean bCheckBeenThroughNew)  // in : bCheckBeenThrough new for internal use only - always set to TRUE                     
                                   //      default:[TRUE]
{ 
  // don't init m_cpContext here if we've come through 
  // overloaded new method - its already been set

  if(bCheckBeenThroughNew && SmThreadLocalStorage::GetBeenThroughNew(this))
    { SmThreadLocalStorage::SetBeenThroughNew(FALSE, this) ; } // FALSE = forget this, TRUE = remember this
  else
    { m_cpContext = NULL ; }

} // end SmObject::SmObject constructor

//---------------------------------------------------------------------------
SmObject::~SmObject() {
	m_cpContext = NULL;
}
/*******************************************************************//**
PURPOSE: SmObject class copy constructor

NOTES:
***********************************************************************/
SmObject::SmObject(const SmObject &rObj)                              
{ 
  if(SmThreadLocalStorage::GetBeenThroughNew(this))
    { SmThreadLocalStorage::SetBeenThroughNew(FALSE, this) ; } // FALSE = forget this, TRUE = remember this
  else // have not been through new and need to set a Context value - try copying the context
    { m_cpContext = rObj.m_cpContext ; }

} // end SmObject::SmObject copy constructor

/*******************************************************************//**
PURPOSE: This method is called to notify the object of impending 
    changes.  

NOTES: It may have been edited or it may about to be deleted.
    Notification will take care of cleaning up attributes.
***********************************************************************/
void SmObject::Notify                  // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
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
                                       // SM_NO_SPLIT                | SplitObj    | Child1   | Child2                | SplitObj's Owner or NULL   
                                       // SM_NO_MERGE                | MergeObj    | OrigObj1 | OrigObj2              | MergeObj's Owner or NULL   
                                       // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     // 
{
#ifdef SM_DEBUG_CODE
  #ifdef SM_ECHO_NOTIFY    
    SmBoolean bDebugMe = TRUE ;
  #else
    SmBoolean bDebugMe = FALSE ;
  #endif // no SM_ECHO_NOTIFY

  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  TCHAR sOpLabel[64], sData1Label[64], sData2Label[64], sData3Label[64] ;

  if(bDebugMe)
    {
      // eNotifyOperation label
      SM_SPRINTF( sOpLabel, _T("%s"),  eNotifyOperation == SM_NO_ADD_TO_BREP           ? _T("SM_NO_ADD_TO_BREP          ")
                                     : eNotifyOperation == SM_NO_SPLIT_IN_BREP         ? _T("SM_NO_SPLIT_IN_BREP        ")
                                     : eNotifyOperation == SM_NO_MERGE_IN_BREP         ? _T("SM_NO_MERGE_IN_BREP        ")
                                     : eNotifyOperation == SM_NO_TRIM_NO_SPLIT_IN_BREP ? _T("SM_NO_TRIM_NO_SPLIT_IN_BREP")
                                     : eNotifyOperation == SM_NO_COINCIDENT            ? _T("SM_NO_COINCIDENT           ")
                                     : eNotifyOperation == SM_NO_RM_FROM_BREP          ? _T("SM_NO_RM_FROM_BREP         ")
                                     : eNotifyOperation == SM_NO_CHANGE_GEOMETRY       ? _T("SM_NO_CHANGE_GEOMETRY      ")
                                     : eNotifyOperation == SM_NO_CHANGE_OWNER          ? _T("SM_NO_CHANGE_OWNER         ")
                                     : eNotifyOperation == SM_NO_CONSTRUCTION          ? _T("SM_NO_CONSTRUCTION         ")
                                     : eNotifyOperation == SM_NO_COPY                  ? _T("SM_NO_COPY                 ")
                                     : eNotifyOperation == SM_NO_PRE_EDIT              ? _T("SM_NO_PRE_EDIT             ")
                                     : eNotifyOperation == SM_NO_POST_EDIT             ? _T("SM_NO_POST_EDIT            ")
                                     : eNotifyOperation == SM_NO_SPLIT                 ? _T("SM_NO_SPLIT                ")
                                     : eNotifyOperation == SM_NO_MERGE                 ? _T("SM_NO_MERGE                ")
                                     : eNotifyOperation == SM_NO_REG_PROPAGATION       ? _T("SM_NO_REG_PROPAGATION      ")
                                     : eNotifyOperation == SM_NO_DESTRUCTION           ? _T("SM_NO_DESTRUCTION          ")
                                     : eNotifyOperation == SM_NO_UNKNOWN               ? _T("SM_NO_UNKNOWN")
                                     : _T("Bad value")) ;

      // pData1 label
      SM_SPRINTF( sData1Label, _T("%s"),  eNotifyOperation == SM_NO_ADD_TO_BREP           ? _T("AddObj  ")
                                        : eNotifyOperation == SM_NO_SPLIT_IN_BREP         ? _T("OrigObj ")
                                        : eNotifyOperation == SM_NO_MERGE_IN_BREP         ? _T("SurvObj ")
                                        : eNotifyOperation == SM_NO_TRIM_NO_SPLIT_IN_BREP ? _T("TgtObj  ")
                                        : eNotifyOperation == SM_NO_COINCIDENT            ? _T("BrepAObj")
                                        : eNotifyOperation == SM_NO_RM_FROM_BREP          ? _T("RmObj   ")
                                        : eNotifyOperation == SM_NO_CHANGE_GEOMETRY       ? _T("NewGeom ")
                                        : eNotifyOperation == SM_NO_CHANGE_OWNER          ? _T("NewOwner")
                                        : eNotifyOperation == SM_NO_CONSTRUCTION          ? _T("NewObj  ")
                                        : eNotifyOperation == SM_NO_COPY                  ? _T("ToObj   ")
                                        : eNotifyOperation == SM_NO_PRE_EDIT              ? _T("EditObj ")
                                        : eNotifyOperation == SM_NO_POST_EDIT             ? _T("EditObj ")
                                        : eNotifyOperation == SM_NO_SPLIT                 ? _T("Child1  ")
                                        : eNotifyOperation == SM_NO_MERGE                 ? _T("OrigObj1")
                                        : eNotifyOperation == SM_NO_REG_PROPAGATION       ? _T("ThisRegs")
                                        : eNotifyOperation == SM_NO_DESTRUCTION           ? _T("DelObj  ")
                                        : eNotifyOperation == SM_NO_UNKNOWN               ? _T("SM_NO_UNKNOWN")
                                        : _T("Bad value")) ;

      // pData2 label
      SM_SPRINTF( sData2Label, _T("%s"),  eNotifyOperation == SM_NO_ADD_TO_BREP           ? _T("Brep                 ")
                                        : eNotifyOperation == SM_NO_SPLIT_IN_BREP         ? _T("Child1               ")
                                        : eNotifyOperation == SM_NO_MERGE_IN_BREP         ? _T("DelObj               ")
                                        : eNotifyOperation == SM_NO_TRIM_NO_SPLIT_IN_BREP ? _T("AddedBndryObj        ")
                                        : eNotifyOperation == SM_NO_COINCIDENT            ? _T("BrepBObj             ")
                                        : eNotifyOperation == SM_NO_RM_FROM_BREP          ? _T("Brep                 ")
                                        : eNotifyOperation == SM_NO_CHANGE_GEOMETRY       ? _T("Brep or NULL         ")
                                        : eNotifyOperation == SM_NO_CHANGE_OWNER          ? _T("NewOwner Brep or NULL")
                                        : eNotifyOperation == SM_NO_CONSTRUCTION          ? _T("CopyFromObj or NULL  ")
                                        : eNotifyOperation == SM_NO_COPY                  ? _T("ToObj's Owner or NULL")
                                        : eNotifyOperation == SM_NO_PRE_EDIT              ? _T("EditObj Owner or NULL")
                                        : eNotifyOperation == SM_NO_POST_EDIT             ? _T("EditObj Owner or NULL")
                                        : eNotifyOperation == SM_NO_SPLIT                 ? _T("Child2               ")
                                        : eNotifyOperation == SM_NO_MERGE                 ? _T("OrigObj2             ")
                                        : eNotifyOperation == SM_NO_REG_PROPAGATION       ? _T("OtherBrep->SrcRegions")
                                        : eNotifyOperation == SM_NO_DESTRUCTION           ? _T(" NULL                ")
                                        : eNotifyOperation == SM_NO_UNKNOWN               ? _T("SM_NO_UNKNOWN")
                                        : _T("Bad value")) ;

      // pData3 label
      SM_SPRINTF( sData3Label, _T("%s"),  eNotifyOperation == SM_NO_ADD_TO_BREP           ? _T("AddObj's GeomPtr or NULL")
                                        : eNotifyOperation == SM_NO_SPLIT_IN_BREP         ? _T("Child2                  ")
                                        : eNotifyOperation == SM_NO_MERGE_IN_BREP         ? _T("Brep                    ")
                                        : eNotifyOperation == SM_NO_TRIM_NO_SPLIT_IN_BREP ? _T("NULL                    ")
                                        : eNotifyOperation == SM_NO_COINCIDENT            ? _T("BrepB                   ")
                                        : eNotifyOperation == SM_NO_RM_FROM_BREP          ? _T("RmObj's GeomPtr or NULL ")
                                        : eNotifyOperation == SM_NO_CHANGE_GEOMETRY       ? _T("OldGeom or NULL         ")
                                        : eNotifyOperation == SM_NO_CHANGE_OWNER          ? _T("OldOwner or NULL        ")
                                        : eNotifyOperation == SM_NO_CONSTRUCTION          ? _T("NULL                    ")
                                        : eNotifyOperation == SM_NO_COPY                  ? _T("FromObj's Owner or NULL ")
                                        : eNotifyOperation == SM_NO_PRE_EDIT              ? _T("NULL                    ")
                                        : eNotifyOperation == SM_NO_POST_EDIT             ? _T("NULL                    ")
                                        : eNotifyOperation == SM_NO_SPLIT                 ? _T("SplitObj's Owner or NULL")
                                        : eNotifyOperation == SM_NO_MERGE                 ? _T("MergeObj's Owner or NULL")
                                        : eNotifyOperation == SM_NO_REG_PROPAGATION       ? _T("ThisBrep->MergedRegion  ")
                                        : eNotifyOperation == SM_NO_DESTRUCTION           ? _T(" NULL                   ")
                                        : eNotifyOperation == SM_NO_UNKNOWN               ? _T("SM_NO_UNKNOWN")
                                        : _T("Bad value")) ;

      // Notify report
      SM_SPRINTF( sBuff, _T("\n  Notify: Op[%s], %16s(0x%p), %s[%16s(0x%p)], %s[%16s(0x%p)], %s[%16s(0x%p)]"), 
                  sOpLabel,
                  this->GetTypeString(), this,
                  sData1Label, pData1 ? pData1->GetTypeString() : _T(""), pData1,
                  sData2Label, pData2 ? pData2->GetTypeString() : _T(""), pData2,
                  sData3Label, pData3 ? pData3->GetTypeString() : _T(""), pData3) ;

      SM_SPRINTF( sBuffForFile, _T("\n  Notify: Op[%s], %16s(0x%p), %s[%16s(%s)], %s[%16s(%s)], %s[%16s(%s)]"), 
                  sOpLabel,
                  this->GetTypeString(), _T("notNULL"),
                  sData1Label, pData1 ? pData1->GetTypeString() : _T(""), pData1 ? _T("notNULL") : _T("NULL"),
                  sData2Label, pData2 ? pData2->GetTypeString() : _T(""), pData2 ? _T("notNULL") : _T("NULL"),
                  sData3Label, pData3 ? pData3->GetTypeString() : _T(""), pData3 ? _T("notNULL") : _T("NULL")) ;

      smos_WriteBuffer(sBuff, sBuffForFile);
    
    } // end if bDebugMe

#endif // SM_DEBUG_CODE

  // Call user callback if present.
  const SmContext *pContext = GetContext() ;
  if ( pContext != NULL )
    {
      SmNotifyCallback *pUCB = pContext->GetUserNotifyCallback() ;
      if ( pUCB != NULL ) 
        {
          pUCB->Execute( eNotifyOperation, this, pData1, pData2, pData3 ) ;
        }
    }

  // Call system callback if present.
  if ( pContext != NULL )
    {
      SmNotifyCallback *pSCB = ((SmContext*)pContext)->GetSysNotifyCallback() ;
      if ( pSCB != NULL ) 
        {
          pSCB->Execute( eNotifyOperation, this, pData1, pData2, pData3 ) ;
        }
    }

} // end SmObject::Notify

/*******************************************************************//**
PURPOSE: Overrides the new operator for SmThreadLocalStorage.

NOTES: This resolves a mess of conflicting implementation
  hacks to allow BeenThroughNew to work around the original 
  overloaded new error of making an assignment within a new operator.
***********************************************************************/
#ifndef SM_BORLAND
void *SmObject::operator new
 (size_t                       size,                 // in : 
  const SmThreadLocalStorage * pThreadLocalStorage)  // NotUsed: in : 
{ 
  SM_REF1(pThreadLocalStorage) ; 
  // okay to use smos_Calloc in overloaded new operators for base classes
  SmObject *pRet    = (SmObject*)smos_Calloc(1,size); 
  pRet->m_cpContext = NULL;
  return (void*)pRet;

} // SmObject::operator new with size
#endif

/*******************************************************************//**
PURPOSE: Overrides the new operator for SmObject subclasses.

NOTES: 
- SmObject *a = new SmObject();
   Try to use the context based news instead of this one if you wish
   to maintain the pool based memory capabilities.
***********************************************************************/
#ifndef SM_BORLAND
void *SmObject::operator new(size_t size)
{ 
  // okay to use smos_Calloc in overloaded new operators for base classes
  SmObject *pRet    = (SmObject*)smos_Calloc(1,size); 
  pRet->m_cpContext = NULL;
  return (void*)pRet;

} // SmObject::operator new with size
#endif // no SM_BORLAND

/*******************************************************************//**
PURPOSE: Overrides the new operator for SmObject subclasses which
    take a reference to a context as the argument.  

NOTES:
  Example use: 
    SmObject *a = new (crContext) SmObject();

  The pointer to the SmContext object is saved and made available
  throughout all the SMLibrary.  'Global' values are stored in this
  context.  
  
  Current SmContext uses include, 
    access to a common memPool (when using SmartHeap or its equivalent)
    access to the global cache queues,
    access to application-supplied attribute read/write callback functions,
    access to application-supplied creation/deletion/modification callback functions,
    access to state data including the boolean m_bDoingBoolean, and
           two global mark values used by functions to mark when a 
           entity has been visited.
    access to 3DField, a 3d to 3d tranformation that modifies all the
           shapes created with this context

***********************************************************************/
void *SmObject::operator new
 (size_t            size,       // in : size in bytes for new memory
  const SmContext & crContext)  // in : context object for new Object construction
{ 
  // okay to use smos_Calloc in overloaded new operators for base classes
  SmObject *pRet    = (SmObject*)smos_Calloc(1,size);
  pRet->m_cpContext = &crContext;

  // set SmThreadLocalStorage::m_bBeenThroughNew to tell SmObject::Constructor
  // that m_cpContext has been initialized.
  SM_ASSERT(SmThreadLocalStorage::GetBeenThroughNew(pRet) == FALSE) ;
  SmThreadLocalStorage::SetBeenThroughNew(TRUE, pRet) ; // TRUE = remember pRet, FALSE = forget pRet

  // all done
  return (void*)pRet;

} // end SmObject::operator new with context reference

/*******************************************************************//**
PURPOSE: Overrides the new operator for SmObject subclasses which
    take a pointer to a context as the argument.  

NOTES: 
- SmObject *a = new (&crContext) SmObject();

  The pointer to the SmContext object is saved and made available
  throughout all the SMLibrary.  'Global' values are stored in this
  context.  
  
  Current SmContext uses include, 
    access to a common memPool (when using SmartHeap or its equivalent)
    access to the global cache queues,
    access to application-supplied attribute read/write callback functions,
    access to application-supplied creation/deletion/modification callback functions,
    access to state data including the boolean m_bDoingBoolean, and
           two global mark values used by functions to mark when a 
           entity has been visited.
    access to 3DField, a 3d to 3d tranformation that modifies all the
           shapes created with this context

***********************************************************************/
#ifndef SM_BORLAND

void *SmObject::operator new
  (size_t size, 
   const SmContext * cpContext)
{ 
  // check state - inform the public
  if(cpContext == NULL) 
    { SE(SM_ERR) ; }  // On Brepdata Context is NULL

  // Alloc SmObject memory - okay to use smos_Calloc in overloaded new operators for base classes
  SmObject * pRet     = (SmObject*)smos_Calloc(1,size) ; 
  pRet->m_cpContext   = cpContext ;

  // Hack workaround for SMLib design flaw - overloaded new should never have set a m_cpContext value.
  // That is a violation of the ANSI C++ standard. Overloaded new operators should only allocate
  // memory.  They should never set values.
  //
  // We don't want to fix this design mistake now because that would change the syntax of 
  // every SMLib new call requiring every customer program to be rewritten.
  // The problem is that by the time the SmObject Constructor is run, the value of 
  // SmObject::m_cpContext is either 1.) Set properly or 2.) full of uninitialized bits and
  // that is determined only by the history of the object construction.  Objects declared
  // in the heap will have properly set m_cpContext values and objects declared on the stack
  // will have m_cpContext values full of uninitialized bits as:
  //  call: SmDerivedClass * pObj = new (cpContext) SmDerivedClass(args) ; 
  //            // runs this overloaded new operator, allocs mem on the heap, 
  //            // and sets the value of SmObject::m_cpContext.
  //  call: SmDerivedClass sObj(args) ; 
  //            // does not run this overloaded new operator, allocs mem on the stack, 
  //            // and leaves the value of SmObject::m_cpContext full of uninitialized bits.
  // The constructor doesn't get to know the history of calls made before it runs 
  // and can't figure out the right thing to do for the SmObject::m_cpContext value.  
  // It should do the following:
  //     if(m_cpContext is full of uninit bits) { m_cpContext = NULL ; }
  //     else                                   { leave m_cpContext alone ; }
  // As a hack to fix this problem for the SmObject::Constructor, this overloaded new
  // operator has been extended to set a bit in a global class that can be checked by
  // the SmObject::Constructor so that it can know when to set the SmObject::m_cpContext value
  // to NULL and when to leave it alone.  That idea was extended to support multithreaded
  // runs by moving the bit to a thread global area.  The bit is now named,
  // SmThreadLocalStorage::m_bBeenThroughNew.  Each thread gets its own SmThreadLocalStorage object
  // so that there is one m_bBeenThroughNew bit per thread. This thread local bit value is set 
  // in SmObject::operator new and checked and cleared in SmObject::Constructor. The SmObject::Constructor
  // uses the bit to decide if the value of m_cpContext is set or uninitialized as:
  //  In SmObject::Constructor:  If(FALSE == SmThreadLocalStorage::GetBeenThroughNew()) 
  //                               { m_cpContext = NULL ; }

  // set m_bBeenThroughNew bit to tell SmObject::Constructor that this overloaded operator new method 
  // has been run for the pRet object so that it can know that the pRet::m_cpContext value has been set 
  // and is not uninitialized.
  SmThreadLocalStorage::SetBeenThroughNew(TRUE, pRet) ; // TRUE = remember this, FALSE = forget this

  // all done
  return (void*)pRet ;

} // end SmObject::operator new with context pointer

#endif // no SM_BORLAND

/*******************************************************************//**
PURPOSE: Virtual method used to determine validity of objects.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmObject::AssertValid
 (SmAssertArray    * /* pAList       */ , // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  /* eTestLevel   */ , // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                          //      default:[SM_LEVEL_0] 
  SmAssertWalking    /* eWalkTree    */ , // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * /* pTestRequests*/ ) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  return TRUE ;

} // end SmObject::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmObject::AssertHeal
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
//   // check state - only for SmObject 
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       ERR_MSG(_T("AssertHeal() error: virtual SmClass::AssertHeal() was not the SmClass that generated this report - Logic Bug.")) ;
//        
//       return( FALSE ) ;
//     }
// 
//   //  // check state - not the class that generated this report - pass call to parent class
//   //  if(rAReport.m_lReportingType != GetClassType())
//   //    {
//   //      // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//   //      return ( Parent::AssertHeal(rAReport, pAList) ) ;
//   //    }
//    
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 999 : { // run fix code here
//                    // if fix works set rAReport.m_bOK = TRUE ; 
//                  }
//                  break ;
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//                rAReport.m_pHealMessage = _T("SmObject::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmObject::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Get the display color for this object using the specified rule.

NOTES: Color Rules are listed in the SmColorRuleType enum as
  SM_CR_STANDARD,              // if stackColor - return stack color
                               // else if specialRule - process special rule
                               // else if object->Geometry and geometry->ColorAttribute - return colorAttribute
                               // else if object->ColorAttribute - return colorAttribute
                               // else if isGeometry && object->Owner and owner->ColorAttribute - return colorAttribute
                               // else if object->Brep and Brep->ColorAttribute - return ColorAttribute
                               // else return defaultColor
  SM_CR_SHADING,               // when returning a default color
                               //      return sDefaultShadingColor
  SM_CR_OBJPROPERTY,           // if   Object->AttributeColor
                               // else object->HasProperty - return PropertyColro
                               // else return defaultColor
  SM_CR_CURVECONTROLPOLYGON,   // use the object attribute color or the curve control polygon color
  SM_CR_SURFACECONTROLPOLYGON, // use the object attribute color or the surface control polygon color
  SM_CR_VOLUMECONTROLPOLYGON,  // use the object attribute color or the volume control polygon color
  SM_CR_SURFACENORMAL,         // use the SurfaceNormal color
  SM_CR_NEG_SURFACENORMAL,     // 2nd surfaceNormal color for normals pointing into screen
  SM_CR_SURFACENORMAL_INFINITE,// when drawing TypedShells - use this color for a shell that bounds an infinite region
  SM_CR_SURFACENORMAL_SOLID,   // when drawing TypedShells - use this color for a shell that bounds a solid region
  SM_CR_SURFACENORMAL_VOID,    // when drawing TypedShells - use this color for a shell that bounds a void region
  SM_CR_CURVATURE,             // use the curvature comb color
  SM_CR_CURVATURE2,            // use curvature comb color 2 (when drawing U and V isoparam Curves with curvature on surfaces)  SM_CR_SPEED,                 // use the speed comb color
  SM_CR_KNOT,                  // use the knot color
  SM_CR_VARYCROSSHATCH,        // use the varied crosshatch line color
  SM_CR_HIGHCOUNTCURVE,        // use the high ControlPoint count curve color
  SM_CR_BASESURFACE,           // use the BaseSurface color for SmOffsetSurface base surfaces
  SM_CR_EXTENDEDSURFACE,       // use the ExtendedSurface color for SmOffsetSurface extended surfaces
  SM_CR_NEIGHBOR,              // use the Neighbor Color for drawing neighbors in DrawNeighbors functions     
  SM_CR_NEIGHBORMATE,          // use the Neighbor Color for drawing neighbor mates in DrawNeighbors functions
  SM_CR_HOTPOINT,              // use the hot point color for hot points
  SM_CR_HOTPOINTFILL,          // use the hot point fill Color to draw non-pickable potions of the hot point display
  SM_CR_HOTPOINTSELECT,        // use the hot point select color for selected hot points
  SM_CR_HOTPOINTHIGHLIGHT,     // use the hot point highlight color for highlighted hot points
  SM_CR_TRACKTARGET,           // use the track target color to display track mode orientation graphics
  SM_CR_TRACKINFO,             // use the track info color to display none track mode tracking orientation graphics
  SM_CR_TESTGEOMETRY,          // use the Test Geometry color to display geometry under interactive construction
  SM_CR_GAPPOINT,              // use the gap color to display gap points          
  SM_CR_VERTEX_EDGEGAP,        // use the gap color to display vertexEdge gap lines
  SM_CR_VERTEX_FACEGAP,        // use the gap color to display VertexFace gap lines
  SM_CR_EDGE_FACEGAP           // use the gap color to display EdgeFace gap lines  
  SM_CR_IN_TOL_GAP,            // use the color for In Tolerance GapFunction gap segments    
  SM_CR_OUT_TOL_GAP,           // use the color for Out of Tolerance GapFunction gap segments
***********************************************************************/
const SmVector3d &
SmObject::GetObjectColor
  (SmColorRuleType eColorRule)   // in : oneof SM_CR_STANDARD,      
                                 //            SM_CR_SHADING,        
                                 //            SM_CR_OBJPROPERTY,
                                 //            SM_CR_CURVECONTROLPOLYGON,  
                                 //            SM_CR_SURFACECONTROLPOLYGON,
                                 //            SM_CR_VOLUMECONTROLPOLYGON,  
                                 //            SM_CR_SURFACENORMAL,   
                                 //            SM_CR_NEG_SURFACENORMAL,
                                 //            SM_CR_SURFACENORMAL_INFINITE,
                                 //            SM_CR_SURFACENORMAL_SOLID,  
                                 //            SM_CR_SURFACENORMAL_VOID,   
                                 //            SM_CR_KNOT,
                                 //            SM_CR_VARYCROSSHATCH,
                                 //            SM_CR_HIGHCOUNTCURVE,
                                 //            SM_CR_BASESURFACE,
                                 //            SM_CR_EXTENDEDSURFACE,
                                 //            SM_CR_NEIGHBOR,   
                                 //            SM_CR_NEIGHBORMATE,
                                 //            SM_CR_HOTPOINT,     
                                 //            SM_CR_HOTPOINTFILL,
                                 //            SM_CR_HOTPOINTSELECT,
                                 //            SM_CR_HOTPOINTHIGHLIGHT,
                                 //            SM_CR_TRACKTARGET,
                                 //            SM_CR_TRACKINFO,  
                                 //            SM_CR_TESTGEOMETRY,
                                 //            SM_CR_GAPPOINT,      
                                 //            SM_CR_VERTEX_EDGEGAP,
                                 //            SM_CR_VERTEX_FACEGAP,
                                 //            SM_CR_EDGE_FACEGAP,
                                 //            SM_CR_IN_TOL_GAP, 
                                 //            SM_CR_OUT_TOL_GAP,
  const
{
  // local object->ColorAttribute Pointer
  SmBoolean             bDone         = FALSE ;
  SmAObject           * pAObject      =   IsKindOf(SmAObject_TYPE) 
                                        ? (SmAObject *)this 
                                        : NULL ;
  SmVector3dAttribute * pColorAttr    = NULL ;
  const SmVector3d    * pDefaultColor = NULL ;
  const SmVector3d    * pRtn = NULL;

  // when there is a color on the colorStack
#ifdef SM_GFX_CODE
  if(smgfx_AnyInterruptColor())
    {
      // return stackColor for different rules
      switch(eColorRule)
        {
          // return interrupt color
          case SM_CR_SHADING               : 
          case SM_CR_STANDARD              : 
          case SM_CR_OBJPROPERTY           : 
                                             bDone = TRUE ; 
                                             pRtn  = &smgfx_GetColor() ;
                                             break ;

          // don't return stack color
          case SM_CR_CURVECONTROLPOLYGON   : 
          case SM_CR_SURFACECONTROLPOLYGON :
          case SM_CR_VOLUMECONTROLPOLYGON  : 
          case SM_CR_SURFACENORMAL         :
          case SM_CR_NEG_SURFACENORMAL     :
          case SM_CR_SURFACENORMAL_INFINITE:
          case SM_CR_SURFACENORMAL_SOLID   :
          case SM_CR_SURFACENORMAL_VOID    :
          case SM_CR_CURVATURE             :
          case SM_CR_CURVATURE2            :
          case SM_CR_SPEED                 :
          case SM_CR_KNOT                  : 
          case SM_CR_VARYCROSSHATCH        :
          case SM_CR_HIGHCOUNTCURVE        :
          case SM_CR_BASESURFACE           :
          case SM_CR_EXTENDEDSURFACE       :
          case SM_CR_NEIGHBOR              :  
          case SM_CR_NEIGHBORMATE          :
          case SM_CR_HOTPOINT              :
          case SM_CR_HOTPOINTFILL          :
          case SM_CR_HOTPOINTSELECT        :
          case SM_CR_HOTPOINTHIGHLIGHT     :
          case SM_CR_TRACKTARGET           :
          case SM_CR_TRACKINFO             :
          case SM_CR_TESTGEOMETRY          :
          case SM_CR_GAPPOINT              :
          case SM_CR_VERTEX_EDGEGAP        :
          case SM_CR_VERTEX_FACEGAP        :
          case SM_CR_EDGE_EDGEGAP          :
          case SM_CR_EDGE_FACEGAP          :
          case SM_CR_IN_TOL_GAP            :
          case SM_CR_OUT_TOL_GAP           :
          case SM_CR_VERTEX_UVTRIMCURVEGAP :
          case SM_CR_EDGE_UVTRIMCURVEGAP   :
          case SM_CR_UNDEFINED             :
               break ;
        }
    } // end smgfx_AnyInterruptColor check
#endif

  // seek ColorAttribute for different rules
  if(!bDone)
    {
      switch(eColorRule)
        {
          // seek in sequence: Object->Geometry->ColorAttribute
          //                   Object->ColorAttribute           
          //                   Object->Brep->ColorAttribute 
          case SM_CR_SHADING :                               
          case SM_CR_STANDARD:                            
              if(pAObject) 
              {
                  // seek object->Geometry and object->Brep
                  SmAObject *pGeometry  =   (IsKindOf(SmFace_TYPE))    ? (SmAObject *)(((SmFace *)this)->GetSurface())
                                        : (IsKindOf(SmEdge_TYPE))    ? (SmAObject *)(((SmEdge *)this)->GetCurve())
                                        : (IsKindOf(SmEdgeuse_TYPE)) ? (SmAObject *)(((SmEdgeuse *)this)->GetEdge()->GetCurve())
                                        : NULL ;
                  SmAObject *pOwner     =   (IsKindOf(SmSurface_TYPE)) ? (SmAObject *)(((SmSurface *)this)->GetOwner())
                                        : (IsKindOf(SmCurve_TYPE))   ? (SmAObject *)(((SmCurve *)this)->GetOwner())
                                        : (IsKindOf(SmEdgeuse_TYPE)) ? (SmAObject *)(((SmEdgeuse *)this)->GetEdge())
                                        : NULL ;
                  SmAObject *pBrep      =   (IsKindOf(SmPolyBrep_TYPE))   ? (SmAObject *)this
                                        : (IsKindOf(SmPolyRegion_TYPE)) ? ((SmPolyRegion *)this)->GetPolyBrep()
                                        : (IsKindOf(SmPolyShell_TYPE))  ? ((SmPolyShell  *)this)->GetPolyBrep()
                                        : (IsKindOf(SmPolyFace_TYPE))   ? ((SmPolyFace   *)this)->GetPolyBrep()
                                        : (IsKindOf(SmPolyLoop_TYPE))   ? ((SmPolyLoop   *)this)->GetPolyBrep()
                                        : (IsKindOf(SmPolyEdge_TYPE))   ? ((SmPolyEdge   *)this)->GetPolyBrep()
                                        : (IsKindOf(SmPolyVertex_TYPE)) ? ((SmPolyVertex *)this)->GetPolyBrep()
                                        : (IsKindOf(SmTopology_TYPE))   ? (SmAObject *)(((SmTopology *)this)->GetBrep())
                                        : NULL ;


                  // seek ColorAttribute through sequence of Geometry, Object, Owner, then Brep objects
                  if(pGeometry) 
                     pColorAttr = (SmVector3dAttribute*)pGeometry->FindAttribute(SM_AI_COLOR) ;
       
                  if(!pColorAttr) 
                      pColorAttr = (SmVector3dAttribute*)pAObject->FindAttribute(SM_AI_COLOR) ;
       
                  if(!pColorAttr && pOwner) 
                      pColorAttr = (SmVector3dAttribute*)pOwner->FindAttribute(SM_AI_COLOR) ;
       
                  if(!pColorAttr && pBrep) 
                      pColorAttr = (SmVector3dAttribute*)pBrep->FindAttribute(SM_AI_COLOR) ;
       
              } // end search for color attribute check
                                       
              break ;
                                       
          // seek Object->ColorAttribute
          case SM_CR_OBJPROPERTY           : 
          case SM_CR_CURVECONTROLPOLYGON   : 
          case SM_CR_SURFACECONTROLPOLYGON : 
          case SM_CR_VOLUMECONTROLPOLYGON  :
                 if(pAObject)
                  {
                    pColorAttr = (SmVector3dAttribute*)pAObject->FindAttribute(SM_AI_COLOR) ;
                  } 
                 break ;

          // don't seek a ColorAttribute
          case SM_CR_SURFACENORMAL         :
          case SM_CR_NEG_SURFACENORMAL     :
          case SM_CR_SURFACENORMAL_INFINITE:
          case SM_CR_SURFACENORMAL_SOLID   :
          case SM_CR_SURFACENORMAL_VOID    :
          case SM_CR_CURVATURE             :
          case SM_CR_CURVATURE2            :
          case SM_CR_SPEED                 :
          case SM_CR_KNOT                  :
          case SM_CR_VARYCROSSHATCH        :
          case SM_CR_HIGHCOUNTCURVE        :
          case SM_CR_BASESURFACE           :
          case SM_CR_EXTENDEDSURFACE       :
          case SM_CR_NEIGHBOR              :
          case SM_CR_NEIGHBORMATE          :
          case SM_CR_HOTPOINT              :
          case SM_CR_HOTPOINTFILL          :
          case SM_CR_HOTPOINTSELECT        :
          case SM_CR_HOTPOINTHIGHLIGHT     :
          case SM_CR_TRACKTARGET           :
          case SM_CR_TRACKINFO             :
          case SM_CR_TESTGEOMETRY          :
          case SM_CR_GAPPOINT              :
          case SM_CR_VERTEX_EDGEGAP        :
          case SM_CR_VERTEX_FACEGAP        :
          case SM_CR_EDGE_EDGEGAP          :
          case SM_CR_EDGE_FACEGAP          :
          case SM_CR_IN_TOL_GAP            :
          case SM_CR_OUT_TOL_GAP           :
          case SM_CR_VERTEX_UVTRIMCURVEGAP :
          case SM_CR_EDGE_UVTRIMCURVEGAP   :
          case SM_CR_UNDEFINED             :
               break ;
        }

      // if there is a colorAttribute - return its color
      if(pColorAttr) { bDone = TRUE ;
                       pRtn  = &pColorAttr->GetValue() ; 
                     }
    } // end need to search for color attirbute check

  // else Get default color for different rules
  if(!bDone)
    {
      switch(eColorRule)
        {
#ifdef SM_GFX_CODE  
          case SM_CR_STANDARD              : pDefaultColor = &smgfx_GetDefaultColor() ;
                                             break ;
          case SM_CR_SHADING               : pDefaultColor = &smgfx_GetDefaultShadingColor() ;
                                             break ;
         case SM_CR_OBJPROPERTY           : pDefaultColor =     IsKindOf(SmEdge_TYPE)
                                                              ? (  ((SmEdge *)pAObject)->IsSpine() ? &smgfx_GetSpineEdgeColor() 
                                                                 : ((SmEdge *)pAObject)->IsLamina() ? &smgfx_GetLaminaEdgeColor() 
                                                                 : ((SmEdge *)pAObject)->IsWire()   ? &smgfx_GetWireEdgeColor()
                                                                 : &smgfx_GetDefaultColor() )
                                                              : &smgfx_GetDefaultColor() ;
                                             break ;

          case SM_CR_CURVECONTROLPOLYGON   : pDefaultColor = &smgfx_GetCurveControlPolygonColor() ;
                                             break ;
          case SM_CR_SURFACECONTROLPOLYGON : pDefaultColor = &smgfx_GetSurfaceControlPolygonColor() ;
                                             break ;
          case SM_CR_VOLUMECONTROLPOLYGON  : pDefaultColor = &smgfx_GetVolumeControlPolygonColor() ;
                                             break ;
          case SM_CR_SURFACENORMAL         : pDefaultColor = &smgfx_GetSurfaceNormalColor() ;
                                             break ;
          case SM_CR_NEG_SURFACENORMAL     : pDefaultColor = &smgfx_GetNegSurfaceNormalColor() ;
                                             break ;
          case SM_CR_SURFACENORMAL_INFINITE: pDefaultColor = &smgfx_GetSurfaceNormalInfiniteColor() ;
                                             break ;
          case SM_CR_SURFACENORMAL_SOLID   : pDefaultColor = &smgfx_GetSurfaceNormalSolidColor() ;
                                             break ;
          case SM_CR_SURFACENORMAL_VOID    : pDefaultColor = &smgfx_GetSurfaceNormalVoidColor() ;
                                             break ;
          case SM_CR_CURVATURE             : pDefaultColor = &smgfx_GetCurvatureColor() ;
                                             break ;
          case SM_CR_CURVATURE2            : pDefaultColor = &smgfx_GetCurvature2Color() ;
                                             break ;
          case SM_CR_SPEED                 : pDefaultColor = &smgfx_GetSpeedColor() ;
                                             break ;
          case SM_CR_KNOT                  : pDefaultColor = &smgfx_GetKnotColor() ;
                                             break ;
          case SM_CR_VARYCROSSHATCH        : pDefaultColor = &smgfx_GetVaryCrossHatchColor() ;
                                             break ;
          case SM_CR_HIGHCOUNTCURVE        : pDefaultColor = &smgfx_GetHighCountCurveColor() ;
                                             break ;
          case SM_CR_BASESURFACE           : pDefaultColor = &smgfx_GetBaseSurfaceColor() ;
                                             break ;
          case SM_CR_EXTENDEDSURFACE       : pDefaultColor = &smgfx_GetExtendedSurfaceColor() ;
                                             break ;
          case SM_CR_NEIGHBOR              : pDefaultColor = &smgfx_GetNeighborColor() ;
                                             break ;
          case SM_CR_NEIGHBORMATE          : pDefaultColor = &smgfx_GetNeighborMateColor() ;
                                             break ;
          case SM_CR_HOTPOINT              : pDefaultColor = &smgfx_GetHotPointColor() ;
                                             break ;
          case SM_CR_HOTPOINTFILL          : pDefaultColor = &smgfx_GetHotPointFillColor() ; 
                                             break ;
          case SM_CR_HOTPOINTSELECT        : pDefaultColor = &smgfx_GetHotPointSelectColor() ;
                                             break ;
          case SM_CR_HOTPOINTHIGHLIGHT     : pDefaultColor = &smgfx_GetHotPointHighlightColor() ;
                                             break ;
          case SM_CR_TRACKTARGET           : pDefaultColor = &smgfx_GetTrackTargetColor() ;
                                             break ;
          case SM_CR_TRACKINFO             : pDefaultColor = &smgfx_GetTrackInfoColor() ;
                                             break ;
          case SM_CR_TESTGEOMETRY          : pDefaultColor = &smgfx_GetTestGeometryColor() ;
                                             break ;
          case SM_CR_GAPPOINT              : pDefaultColor = &smgfx_GetGapPointColor() ;
                                             break ;
          case SM_CR_VERTEX_EDGEGAP        : pDefaultColor = &smgfx_GetVertexEdgeGapColor() ;
                                             break ;
          case SM_CR_VERTEX_FACEGAP        : pDefaultColor = &smgfx_GetVertexFaceGapColor() ;
                                             break ;
          case SM_CR_EDGE_FACEGAP          : pDefaultColor = &smgfx_GetEdgeFaceGapColor() ;
                                             break ;
          case SM_CR_EDGE_EDGEGAP          : pDefaultColor = &smgfx_GetEdgeEdgeGapColor() ;
                                             break ;
          case SM_CR_VERTEX_UVTRIMCURVEGAP : pDefaultColor = &smgfx_GetVertexFaceTrimCurveGapColor() ;
                                             break ;
          case SM_CR_EDGE_UVTRIMCURVEGAP   : pDefaultColor = &smgfx_GetEdgeFaceTrimCurveGapColor() ;
                                             break ;
          case SM_CR_IN_TOL_GAP            : pDefaultColor = &smgfx_GetInTolGapColor() ;
                                             break ;
          case SM_CR_OUT_TOL_GAP           : pDefaultColor = &smgfx_GetOutTolGapColor() ;
                                             break ;
          case SM_CR_UNDEFINED             : break ;
#else // not defined SM_GFX_CODE       
          case SM_CR_STANDARD              : pDefaultColor = &s_NonGFXDefaultColor ;
                                             break ;
          case SM_CR_SHADING               : pDefaultColor = &s_NonGFXDefaultShadingColor ;
                                             break ;
          case SM_CR_OBJPROPERTY           : pDefaultColor =     IsKindOf(SmEdge_TYPE)
                                                              ? (  ((SmEdge *)pAObject)->IsLamina() ? &s_NonGFXLaminaEdgeColor 
                                                                 : ((SmEdge *)pAObject)->IsWire()   ? &s_NonGFXWireEdgeColor
                                                                 : &s_NonGFXDefaultColor )
                                                              : &s_NonGFXDefaultColor ;
                                             break ;
          case SM_CR_CURVECONTROLPOLYGON   : pDefaultColor = &s_NonGFXCurveControlPolygonColor ;
                                             break ;
          case SM_CR_SURFACECONTROLPOLYGON : pDefaultColor = &s_NonGFXSurfaceControlPolygonColor ;
                                             break ;
          case SM_CR_VOLUMECONTROLPOLYGON : pDefaultColor = &s_NonGFXVolumeControlPolygonColor ;
                                             break ;
          case SM_CR_SURFACENORMAL         : pDefaultColor = &s_NonGFXSurfaceNormalColor ;
                                             break ;
          case SM_CR_NEG_SURFACENORMAL     : pDefaultColor = &s_NonGFXNegSurfaceNormalColor ;
                                             break ;
          case SM_CR_SURFACENORMAL_INFINITE: pDefaultColor = &s_NonGFXSurfaceNormalInfiniteColor ;
                                             break ;
          case SM_CR_SURFACENORMAL_SOLID   : pDefaultColor = &s_NonGFXSurfaceNormalSolidColor ;
                                             break ;
          case SM_CR_SURFACENORMAL_VOID    : pDefaultColor = &s_NonGFXSurfaceNormalVoidColor ;
                                             break ;
          case SM_CR_CURVATURE             : pDefaultColor = &s_NonGFXCurvatureColor ;
                                             break ;
          case SM_CR_CURVATURE2            : pDefaultColor = &s_NonGFXCurvature2Color ;
                                             break ;
          case SM_CR_SPEED                 : pDefaultColor = &s_NonGFXSpeedColor ;
                                             break ;
          case SM_CR_KNOT                  : pDefaultColor = &s_NonGFXKnotColor ;
                                             break ;
          case SM_CR_VARYCROSSHATCH        : pDefaultColor = &s_NonGFXVaryCrossHatchColor ;
                                             break ;
          case SM_CR_HIGHCOUNTCURVE        : pDefaultColor = &s_NonGFXHighCountCurveColor ;
                                             break ;
          case SM_CR_BASESURFACE           : pDefaultColor = &s_NonGFXBaseSurfaceColor ;
                                             break ;
          case SM_CR_EXTENDEDSURFACE       : pDefaultColor = &s_NonGFXExtendedSurfaceColor ; 
                                             break ;
          case SM_CR_NEIGHBOR              : pDefaultColor = &s_NonGFXNeighborColor ; 
                                             break ;
          case SM_CR_NEIGHBORMATE          : pDefaultColor = &s_NonGFXNeighborMateColor ; 
                                             break ;
          case SM_CR_HOTPOINT              : pDefaultColor = &s_NonGFXHotPointColor ;
                                             break ;
          case SM_CR_HOTPOINTFILL          : pDefaultColor = &s_NonGFXHotPointFillColor ;
                                             break ;
          case SM_CR_HOTPOINTSELECT        : pDefaultColor = &s_NonGFXHotPointSelectColor ;
                                             break ;
          case SM_CR_HOTPOINTHIGHLIGHT     : pDefaultColor = &s_NonGFXHotPointHighlightColor ;
                                             break ;
          case SM_CR_TRACKTARGET           : pDefaultColor = &s_NonGFXTrackTargetColor ;
                                             break ;
          case SM_CR_TRACKINFO             : pDefaultColor = &s_NonGFXTrackInfoColor ;
                                             break ;
          case SM_CR_TESTGEOMETRY          : pDefaultColor = &s_NonGFXTestGeometryColor ;
                                             break ;
          case SM_CR_GAPPOINT              : pDefaultColor = &s_NonGFXTestGapPointColor ;
                                             break ;
          case SM_CR_VERTEX_EDGEGAP        : pDefaultColor = &s_NonGFXTestVertexEdgeGapColor ;
                                             break ;
          case SM_CR_VERTEX_FACEGAP        : pDefaultColor = &s_NonGFXTestVertexFaceGapColor ;
                                             break ;
          case SM_CR_EDGE_FACEGAP          : pDefaultColor = &s_NonGFXTestEdgeFaceGapColor ;
                                             break ;
          case SM_CR_EDGE_EDGEGAP          : pDefaultColor = &s_NonGFXTestEdgeEdgeGapColor ;
                                             break ;
          case SM_CR_VERTEX_UVTRIMCURVEGAP : pDefaultColor = &s_NonGFXTestVertexFaceTrimCurveGapColor ;
                                             break ;
          case SM_CR_EDGE_UVTRIMCURVEGAP   : pDefaultColor = &s_NonGFXTestEdgeFaceTrimCurveGapColor ;
                                             break ;
          case SM_CR_IN_TOL_GAP            : pDefaultColor = &s_NonGFXTestInTolGapColor ;
                                             break ;
          case SM_CR_OUT_TOL_GAP           : pDefaultColor = &s_NonGFXTestOutTolGapColor ;
                                             break ;
          case SM_CR_UNDEFINED             : break ;
#endif // not defined SM_GFX_CODE branch
        }
     }

  // Return selected color if possible, else return default color
  return( bDone ? *pRtn : *pDefaultColor) ;
 
} // end SmObject::GetObjectColor

/*******************************************************************//**
PURPOSE: Dump preceded by a one line message

NOTES:
***********************************************************************/
void SmObject::Dump(const TCHAR * message) const
{
     TCHAR sBuff[SM_TBLOCK_SIZE];
     SM_SPRINTF(sBuff,_T("\n%s "), message);
     smos_WriteBuffer(sBuff);
     this->Dump();

} // end SmObject::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmObject::Dump(ULONG i) const
{
     TCHAR sBuff[SM_TBLOCK_SIZE];
     SM_SPRINTF(sBuff,_T("\n%ld "), i);
     smos_WriteBuffer(sBuff);
     this->Dump();

} // end SmObject::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmObject::Dump
 (SmBoolean bAbbrev)  // NotUsed: in :
 const
{
  SM_REF1(bAbbrev) ;
  this->Dump();
} // end SmObject::Dump

/*******************************************************************//**
PURPOSE: Debug dump data describing the SmObject.

NOTES: 
***********************************************************************/
void SmObject::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  SM_SPRINTF(sBuff,_T("  Type = %ld, Context = 0x%p"),GetType(),m_cpContext);
  SM_SPRINTF(sBuffForFile,_T("  Type = %ld, Context = %s"),GetType(),m_cpContext ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  SM_SPRINTF(sBuff,_T("%s"), _T(" \n"));
  smos_WriteBuffer(sBuff);

} // end ::Dump()

/*******************************************************************
PURPOSE: Open a FILE stream pointer in this DLL for i/o

NOTES:
********************************************************************/
FILE* SmObject::OpenFile(const TCHAR *filename, const TCHAR *mode)
{
  // pass the call along
  return( SM_FOPEN(filename, mode) ) ;

} // end SmObject::OpenFile

/*******************************************************************
PURPOSE: Close a FILE stream pointer in this DLL for i/o

NOTES:
********************************************************************/
void SmObject::CloseFile(FILE *pFile)
{
  // pass the call along
  fclose(pFile) ;

} // end SmObject::CloseFile
