// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSAGObject.cpp
* PURPOSE: Source file for SmSAGObject.
**********************************************************************/

#include "StdAfx.h"

#include <SmSAGObject.h>
#include <SmGraphicsOutput.h>
#include <SmAssertArray.h>

/*******************************************************************//**
PURPOSE: Destructor for SmSAGObject - cleans up display list if 
    present.

NOTES: 
***********************************************************************/
SmSAGObject::~SmSAGObject()
{ 
  if (m_sDisplayList.GetDisplayListId() != SM_BIG_ULONG) 
    {
      m_sDisplayList.SetDisplayListId(SM_BIG_ULONG) ;
    }

} // end SmSAGObject::~SmSAGObject destructor

/*******************************************************************//**
PURPOSE: Notify the SAG object that it is being edited.

NOTES: 
***********************************************************************/
void SmSAGObject::Notify               // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
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
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     // 
{
  switch (eNotifyOperation) 
    {
      case SM_NO_PRE_EDIT:
      case SM_NO_RM_FROM_BREP:
      case SM_NO_MERGE_IN_BREP: 
      case SM_NO_TRIM_NO_SPLIT_IN_BREP: 
      case SM_NO_DESTRUCTION:
          if (m_sDisplayList.GetDisplayListId() != SM_BIG_ULONG) 
            {
              m_sDisplayList.SetDisplayListId(SM_BIG_ULONG) ;
            }
          break;

      case SM_NO_COINCIDENT      : break ;
      case SM_NO_ADD_TO_BREP     : break ;
      case SM_NO_SPLIT_IN_BREP   : break ;
      case SM_NO_CHANGE_GEOMETRY : break ;
      case SM_NO_CHANGE_OWNER    : break ;
      case SM_NO_CONSTRUCTION    : break ;
      case SM_NO_COPY            : break ;
      case SM_NO_POST_EDIT       : break ;
      case SM_NO_SPLIT           : break ;
      case SM_NO_MERGE           : break ;
      case SM_NO_REG_PROPAGATION : break ;
      case SM_NO_UNKNOWN:          { SE_MSG(SM_ERR, _T("SmOwningTopology::Notify - SM_NO_UNKNOWN event signalled")) ; } 
                                   break ;
    }

  SmOwningTopology::Notify(eNotifyOperation,pData1,pData2,pData3);

} // end SmSAGObject::Notify

/*******************************************************************//**
PURPOSE: This is a pure virtual method that is not implemented
     at this level.

NOTES: 
***********************************************************************/
SmStatus SmSAGObject::BuildGraphicsStructure
  (const SmDisplayParameters & crDisp,                // NotUsed: in : graphics controlling parameters
   SmDisplayList             & rDisplayList,          // NotUsed: out: displayList name for this Brep Graphics
   SmDisplayList             * pActiveDisplayListRef, // out: Ref to DisplayList Copy placed on the s_View.m_pActiveLists that actually gets drawn
   SmGfxArraySet             * pOptGfxSet)            // NotUsed: i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                      //      NULL to ignore, default:[NULL]
{
  SM_REF3(crDisp, rDisplayList, pOptGfxSet) ;
  // init output
  pActiveDisplayListRef = NULL ;

  SE(SM_ERR);
  return SM_ERR;

} // end SmSAGObject::BuildGraphicsStructure

/*******************************************************************//**
PURPOSE: Get saved DisplayList if the saved DisplayList has 
         the same parameters as the Tgt DisplayParameters  

NOTES:  If it does not match the display
    parameters than delete the saved graphics structures.
***********************************************************************/
SmStatus SmSAGObject::RetrieveGraphicsStructure
  (const SmDisplayParameters & crDisplayParameters,  // in : Tgt DisplayParameters
   SmDisplayList             * pSavedDisplayList)    // out: Ptr to saved DisplayList or NULL
{
  // init output
  pSavedDisplayList = NULL ;

  // When SavedDisplayExists and has matching parameters - update output
  if (   m_sDisplayList.GetDisplayListId() != SM_BIG_ULONG
      && crDisplayParameters == m_vDisplayParameters) 
    {
      pSavedDisplayList = & m_sDisplayList ;
    }


  return SM_SUCCESS;

} // end SmSAGObject::RetrieveGraphicsStructure

/*******************************************************************//**
PURPOSE: Save a graphics structure along with the parameters
   used to generate it.

NOTES: 
***********************************************************************/
SmStatus SmSAGObject::SaveGraphicsStructure
  (const SmDisplayParameters & crDisplayParameters,
   SmDisplayList             * pDisplayListToCopy) 
{
    // Now save graphics segment
    m_vDisplayParameters = crDisplayParameters;

    // copy the DisplayList
    m_sDisplayList = *pDisplayListToCopy ;

    // all done
    return SM_SUCCESS;

} // end SmSAGObject::SaveGraphicsStructure

/*******************************************************************//**
PURPOSE: This method validates an existing cache and/or creates
            a new cache for a Brep or PolyBrep Object. 

NOTES:
 
The returned ObjectCache stores a pointer to the target cpObject.
A pointer to the returned ObjectCache is stored in cpObject

The value of eObjectCacheType must match the type of cpObject:
     -----------------------------------------------------
     | cpObject Type     |     SmObjectCacheType Value   |
     |-------------------|-------------------------------|
     | SmCurve           |     SM_OC_CURVE               |
     |-------------------|-------------------------------|
     | SmSurface         |     SM_OC_SURFACE             |
     |                   |     SM_OC_TRIMSRF             |
     |-------------------|-------------------------------|
     | SmBrep            |     SM_OC_BREP                |
     | SmPolyBrep        |                               |
     -----------------------------------------------------
***********************************************************************/
SmStatus SmSAGObject::CacheMakeOrValidate
  (SmObjectCacheType eObjectCacheType,       // NotUsed: in : oneof SM_OC_CURVE
                                             //            SM_OC_SURFACE        
                                             //            SM_OC_TRIMSRF
                                             //            SM_OC_BREP
   const SmAObject * cpObject,               // in : target object
   SmCacheObj      * cpOldCache,             // in : existing target Object's ObjectCache or NULL
   SmCacheObj     *& rpNewCache)             // out: ptr to target object's ObjectCache
    const
{
  SM_REF1(eObjectCacheType) ;
  // init output
  rpNewCache = NULL;

  // cast Object to SmSAGObject 
  //  (SAG = Standalone with Graphics 
  //   derived Types include: SmBrep, SmPolyBrep, SmAssembly, and SmAssemblyInstance)
  SmSAGObject * pSAGObject = SM_CAST_PTR(SmSAGObject,cpObject);

  // check that object is an SmBrep or SmPolyBrep object
  SmBrep     * cpBrep     = SM_CAST_PTR(SmBrep,cpObject);
  SmPolyBrep * cpPolyBrep = SM_CAST_PTR(SmPolyBrep,cpObject);
  if ( !cpBrep && !cpPolyBrep )
    {
      SM_ASSERT(cpBrep || cpPolyBrep) ;
      return SM_ERR;
    }

  // low work - use OldCache when its given
  SmBrepCache *pBrepCache = (SmBrepCache*)cpOldCache;
  if (pBrepCache) 
    {
      rpNewCache = (SmCacheObj*)pBrepCache;
      return SM_SUCCESS;
    }
  
  // construct new SmBrepCache Object - mark it for delete if a failure exits scope
  pBrepCache = new (*pSAGObject->GetContext()) SmBrepCache(pSAGObject);
  NER(pBrepCache);
  SmObjDelete sCleanup(pBrepCache);

  // create the vertex, curve, and surface bounding-box spatial trees
  SER(pBrepCache->BuildTrees());
  
  // subdivision worked - clear the delete mark and set output
  sCleanup.Clear();
  rpNewCache = pBrepCache;

  SetCacheObj( pBrepCache );

  // all done
  return SM_SUCCESS;

} // end SmSAGObject::CacheMakeOrValidate

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertSAGObject_list[] =
{
  {SM_AT_UNKNOWN, _T("UNKNOWN"), _T("Not Implemented Yet") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmSAGObject::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmOwningTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // todo - add SmSAGObject checks here

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmSAGObject::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmSAGObject::AssertHeal
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
//       return ( SmOwningTopology::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmSAGObject::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmSAGObject::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSAGObject::IsKindOf( SM_TYPE t ) const
{
  return ((SmSAGObject_TYPE == t) ? TRUE : SmAObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump a Standalone Geometry Object

NOTES: 
***********************************************************************/
void SmSAGObject::Dump() const
{
    smos_WriteBuffer(_T("Stand Alone Geometry Object\n"));

} // end SmSAGObject::Dump

