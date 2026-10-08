// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTrackTopologyChanges.cpp
* PURPOSE: Methods for SmTrackTopologyChanges.
**********************************************************************/

 
#include "StdAfx.h"
#include <SmTrackTopologyChanges.h>

/*******************************************************************//**
PURPOSE: SmNotifyEvent pretty print

NOTES:        event                | caller      |  pData1  | pData2                | pData3                    
       ----------------------------+-------------+----------+-----------------------+-------------------------- 
        SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL  
        SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                    
        SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      
        SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                      
        SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB                     
        SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   
        SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           
        SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          
        SM_NO_CONSTRUCTION         | NewObj      | NewObj   | CopyFromObj or NULL   | NULL                     
        SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   
        SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
        SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
        SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL  
        SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL  
        SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg        
        SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     
***********************************************************************/
void SmNotifyEvent::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  TCHAR sCallerStr[64], sCallerStrForFile[64] ;
  TCHAR sData1Str[64],  sData1StrForFile[64] ;
  TCHAR sData2Str[64],  sData2StrForFile[64] ;
  TCHAR sData3Str[64],  sData3StrForFile[64] ;

  #define SM_EVENT_PRINT(sCallerLabel, sData1Label, sData2Label, sData3Label) \
  if(m_lCallerIndx != SM_UNDEF_ULONG)      { smos_snprintf(sCallerStr, 64, _T("indx:%d]"), m_lCallerIndx) ; smos_snprintf(sCallerStrForFile, 64, _T("indx:%d]"), m_lCallerIndx) ; }                             \
                                      else { smos_snprintf(sCallerStr, 64, _T("0x%p"),     m_pCaller) ;     smos_snprintf(sCallerStrForFile, 64, _T("%.10s"),       m_pCaller ? _T("notNULL") : _T("NULL")) ; } \
  if(m_lData1Indx  != SM_UNDEF_ULONG)      { smos_snprintf(sData1Str , 64, _T("indx:%d]"), m_lData1Indx) ;  smos_snprintf(sData1StrForFile , 64, _T("indx:%d]"), m_lData1Indx) ; }                              \
                                      else { smos_snprintf(sData1Str , 64, _T("0x%p"),     m_pData1) ;      smos_snprintf(sData1StrForFile , 64, _T("%.10s"),       m_pData1 ? _T("notNULL") : _T("NULL")) ; }  \
  if(m_lData2Indx  != SM_UNDEF_ULONG)      { smos_snprintf(sData2Str , 64, _T("indx:%d]"), m_lData2Indx) ;  smos_snprintf(sData2StrForFile , 64, _T("indx:%d]"), m_lData2Indx) ; }                              \
                                      else { smos_snprintf(sData2Str , 64, _T("0x%p"),     m_pData2) ;      smos_snprintf(sData2StrForFile , 64, _T("%.10s"),       m_pData2 ? _T("notNULL") : _T("NULL")) ; }  \
  if(m_lData3Indx  != SM_UNDEF_ULONG)      { smos_snprintf(sData3Str , 64, _T("indx:%d]"), m_lData3Indx) ;  smos_snprintf(sData3StrForFile , 64, _T("indx:%d]"), m_lData3Indx) ; }                              \
                                      else { smos_snprintf(sData3Str , 64, _T("0x%p"),     m_pData3) ;      smos_snprintf(sData3StrForFile , 64, _T("%.10s"),       m_pData3 ? _T("notNULL") : _T("NULL")) ; }  \
  smos_sprintf(sBuff,        _T("\n   %.63s: caller_%.63s:[%.63s], pData1_%.63s:[%.63s], pData2_%.63s:[%.63s], pData3_%.63s:[%.63s] "), SM_NOTIFYOPERATIONTYPE_NAME(m_eAction), \
                                                                                                             sCallerLabel, sCallerStr,               \
                                                                                                             sData1Label,  sData1Str,                \
                                                                                                                                        sData2Label,  sData2Str,                \
                                                                                                                                        sData3Label,  sData3Str);               \
  smos_sprintf(sBuffForFile, _T("\n   %.63s: caller_%.63s:[%.63s], pData1_%.63s:[%.63s], pData2_%.63s:[%.63s], pData3_%.63s:[%.63s] "), SM_NOTIFYOPERATIONTYPE_NAME(m_eAction), \
                                                                                                             sCallerLabel, sCallerStrForFile,        \
                                                                                                             sData1Label,  sData1StrForFile,         \
                                                                                                                                        sData2Label,  sData2StrForFile,         \
                                                                                                                                        sData3Label,  sData3StrForFile);        \
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // switch on eAction
  switch(m_eAction)
    {
      case SM_NO_ADD_TO_BREP          : { SM_EVENT_PRINT(_T("Brep        "), _T("AddObj  "), _T("Brep                 "), _T("AddObj's GeomPtr or NULL")) ; } break ;                                       
      case SM_NO_SPLIT_IN_BREP        : { SM_EVENT_PRINT(_T("Brep/TopoObj"), _T("OrigObj "), _T("Child1               "), _T("Child2                  ")) ; } break ;                                       
      case SM_NO_MERGE_IN_BREP        : { SM_EVENT_PRINT(_T("Brep/TopoObj"), _T("SurvObj "), _T("DelObj               "), _T("Brep                    ")) ; } break ;                                       
      case SM_NO_TRIM_NO_SPLIT_IN_BREP: { SM_EVENT_PRINT(_T("Brep        "), _T("TgtObj  "), _T("AddedBndryObj        "), _T("NULL                    ")) ; } break ;                                       
       case SM_NO_RM_FROM_BREP        : { SM_EVENT_PRINT(_T("BrepA       "), _T("BrepAObj"), _T("BrepBObj             "), _T("BrepB                   ")) ; } break ;                                       
      case SM_NO_COINCIDENT           : { SM_EVENT_PRINT(_T("Brep        "), _T("RmObj   "), _T("Brep                 "), _T("RmObj's GeomPtr or NULL ")) ; } break ;                                     
      case SM_NO_CHANGE_GEOMETRY      : { SM_EVENT_PRINT(_T("TopoObj     "), _T("NewGeom "), _T("Brep or NULL         "), _T("OldGeom or NULL         ")) ; } break ;                                       
      case SM_NO_CHANGE_OWNER         : { SM_EVENT_PRINT(_T("GeomObj     "), _T("NewOwner"), _T("NewOwner Brep or NULL"), _T("OldOwner or NULL        ")) ; } break ;                                     
      case SM_NO_CONSTRUCTION         : { SM_EVENT_PRINT(_T("NewObj      "), _T("NewObj  "), _T("CopyFromObj or NULL  "), _T("NULL                    ")) ; } break ;                                     
      case SM_NO_COPY                 : { SM_EVENT_PRINT(_T("FromObj     "), _T("ToObj   "), _T("ToObj's Owner or NULL"), _T("FromObj's Owner or NULL ")) ; } break ;                                     
      case SM_NO_PRE_EDIT             : { SM_EVENT_PRINT(_T("EditObj     "), _T("EditObj "), _T("EditObj Owner or NULL"), _T("NULL                    ")) ; } break ;                                     
      case SM_NO_POST_EDIT            : { SM_EVENT_PRINT(_T("EditObj     "), _T("EditObj "), _T("EditObj Owner or NULL"), _T("NULL                    ")) ; } break ;                                     
      case SM_NO_SPLIT                : { SM_EVENT_PRINT(_T("SplitGeomObj"), _T("Child1  "), _T("Child2               "), _T("SplitObj's Owner or NULL")) ; } break ;                                       
      case SM_NO_MERGE                : { SM_EVENT_PRINT(_T("MergeGeomObj"), _T("OrigObj1"), _T("OrigObj2             "), _T("MergeObj's Owner or NULL")) ; } break ; 
      case SM_NO_REG_PROPAGATION      : { SM_EVENT_PRINT(_T("MergeReg    "), _T("ThisRegs"), _T("OtherBrep->SrcRegs   "), _T("ThisBrep->MergeReg      ")) ; } break ; 
      case SM_NO_DESTRUCTION          : { SM_EVENT_PRINT(_T("DelObj      "), _T("DelObj  "), _T("NULL                 "), _T(" NULL                   ")) ; } break ; 
      default                         : { smos_WriteBuffer(_T("\n default event - error event type not yet supported - needs another case")) ; } break ;
   } // end switch on eAction

#undef SM_EVENT_PRINT
} // end SmNotifyEvent::Dump

/*******************************************************************//**
PURPOSE: Add a Tgt to array m_pRmList when its not already in the list

NOTES: Return TRUE = Tgt was added
              FALSE= Tgt was not added
***********************************************************************/
SmBoolean SmTopologyChangeCallback::AddUnique
 (SmTArray<SmTopology*> * m_pTgtList, // in : Tgt list, oneof m_pRmList, m_pAddList, m_pChgList
  SmTopology            * pTgt,       // in : Tgt to add uniquely to m_pTgtList
  SmBrep                * pBrep)      // in : In Debug mode used for pretty printing - NULL to ignore
{ 
  SM_REF1(pBrep) ;

  // Never track a null topology object. Some notify events (e.g. a split/trim that yields only one
  // child) pass a null pData2/pData3, and those children are AddUnique'd without a null check by the
  // caller. Storing a null here would later crash consumers such as SmHealData::UpdateTargetLists,
  // which dereferences every AddList entry via GetAt(ii)->IsKindOf().
  if(pTgt == NULL)
    { return(FALSE) ; }

  // check state
  SM_ASSERT_MSG(   m_pTgtList == m_pRmList
                || m_pTgtList == m_pAddList
                || m_pTgtList == m_pChgList, _T("SmTopologyChangeCallback::AddUnique, bad input Tgt Array value")) ; 

  // Add Tgt to m_pRmList when its not already in the list
  if(m_pTgtList->AddUnique(pTgt))
    {
#ifdef SM_DEBUG_CODE
      // in debug mode and when tracking Tgt indices - also add an indx value to the associated index array
      if     (m_pTgtList == m_pRmList  && m_pRmListIndx ) { m_pRmListIndx ->Add(pBrep ? pBrep->GetTopologyIndex(pTgt) : 99999) ; }
      else if(m_pTgtList == m_pAddList && m_pAddListIndx) { m_pAddListIndx->Add(pBrep ? pBrep->GetTopologyIndex(pTgt) : 99999) ; }
      else if(m_pTgtList == m_pChgList && m_pChgListIndx) { m_pChgListIndx->Add(pBrep ? pBrep->GetTopologyIndex(pTgt) : 99999) ; }
#endif // SM_DEBUG_CODE
      return TRUE ; 
    }
  return(FALSE) ;
} // end SmTopologyChangeCallback::AddUnique

/*******************************************************************//**
PURPOSE: Remove a Tgt from array m_pRmList when its in the list, otherwise do nothing

NOTES: Return TRUE = Tgt was removed
              FALSE= Tgt was not removed
***********************************************************************/
SmBoolean SmTopologyChangeCallback::Remove
 (SmTArray<SmTopology*> * m_pTgtList, // in : Tgt list, oneof m_pRmList, m_pAddList, m_pChgList
  SmTopology            * pTgt)       // in : Tgt to remove from m_pTgtList
{ 
  ULONG lFound ; 

  // check state
  SM_ASSERT_MSG(   m_pTgtList == m_pRmList
                || m_pTgtList == m_pAddList
                || m_pTgtList == m_pChgList, _T("SmTopologyChangeCallback::AddUnique, bad input Tgt Array value")) ; 

  // Find Tgt in m_pTgtList 
  if(m_pTgtList->FindElement(pTgt, lFound))
    { m_pTgtList->RemoveAt(lFound,1) ; 
#ifdef SM_DEBUG_CODE
      // in debug mode and when tracking Tgt indices - also add an indx value to the associated index array
      if     (m_pTgtList == m_pRmList  && m_pRmListIndx ) { m_pRmListIndx ->RemoveAt(lFound,1) ; }
      else if(m_pTgtList == m_pAddList && m_pAddListIndx) { m_pAddListIndx->RemoveAt(lFound,1) ; }
      else if(m_pTgtList == m_pChgList && m_pChgListIndx) { m_pChgListIndx->RemoveAt(lFound,1) ; }
#endif // SM_DEBUG_CODE
      return(TRUE) ;
    }
  return(FALSE) ;
} // end SmTopologyChangeCallback::RemoveFromRmList 

/*******************************************************************//**
PURPOSE: SmTopologyChangeCallback virtual method to build
         Add, Rm, and Chg Lists of the changed topology objects 
         within a single atomic topology editing function

NOTES: Use the management class SmTrackTopologyChanges which uses this SmTopologyChangeCallback class
       to build lists of changed topology objects from the Notify sequence.
       Example use: {
                       // init TopoChange tracking - allocates list memory
                       // and loads a Notify() callback function into the pBrep->Context
                       SmTrackTopologyChanges sTopoChanges(pBrep, FALSE) ;  // FALSE = don't start tracking changes just yet
                                                                            // TRUE  = start tracking now - saves one sTopoChanges.StartTracking() call

                       // Start tracking changes
                       sTopoChanges.StartTracking() ;

                       // execute any series of Brep changes
                       . . . Brep changing code, ex:pBrep->RemoveFace() . . .

                       // Stop tracking
                       sTopoChanges.StopTracking() ;

                       // sTopoChanges now contains all objects removed and added since the last ReSetArrays() call
                       //  sTopoChanges has list of all topology objects
                       //     - added to the pBrep Topology graph
                       //     - removed from the pBrep Topology graph
                       //     - whose geometry definitions have been changed

                     }                 
***********************************************************************/
SmStatus SmTopologyChangeCallback::Execute           // run once for every SmObject::Notify() call caused by any caller->Notify() listed below //             
   (SmNotifyOperation  eAction,   /* in : event  */  // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                      //
    SmObject         * pObj,      /* in : caller */  //       event                | caller      |  pData1  | pData2                | pData3                    //
    SmObject         * pData1,    /* in : pData1 */  //----------------------------+-------------+----------+-----------------------+-------------------------- //
    SmObject         * pData2,    /* in : pData2 */  // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL  //
    SmObject         * pData3)    /* in : pData3 */  // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                    //
                                                     // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      //
                                                     // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                      //
                                                     // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB                     //
                                                     // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   //
                                                     // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           //
                                                     // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          //
                                                     // SM_NO_CONSTRUCTION         | NewObj      | NewObj   | CopyFromObj or NULL   | NULL                      //
                                                     // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   //
                                                     // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      //
                                                     // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      //
                                                     // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL  //
                                                     // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL  //
                                                     // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg        //
                                                     // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     //
{
  SmBrep * pBrep    = NULL ; 
#ifdef SM_DEBUG_CODE
#ifdef SM_TOPOCHANGE_NOTIFY    
  SmBoolean bDebugLog = TRUE ;
#else // no SM_TOPOCHANGE_NOTIFY
  SmBoolean bDebugLog = FALSE ;
#endif // no SM_TOPOCHANGE_NOTIFY
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  #define SM_DBG_LOG(ActionLabel, Tgt, Brep) \
          if(bDebugLog == TRUE) \
            { smos_sprintf(sBuff,        _T("\n  TopologyChange - %s %s:[0x%p] indx:[%lu]"), ActionLabel, SM_TOPO_TYPENAME(Tgt->GetType()), Tgt, Brep ? Brep->GetTopologyIndex(Tgt) : 99999) ; \
              smos_sprintf(sBuffForFile, _T("\n  TopologyChange - %s %s:[0x%p] indx:[%lu]"), ActionLabel, SM_TOPO_TYPENAME(Tgt->GetType()), Tgt ? _T("NotNULL") : _T("NULL"), Brep ? Brep->GetTopologyIndex(Tgt) : 99999) ; \
              smos_WriteBuffer(sBuff, sBuffForFile) ; \
            }
#else
  #define SM_DBG_LOG(ActionLabel, Tgt, TgtIndx)
#endif // SM_DEBUG_CODE

  // encapsulate event
  SmNotifyEvent sNotifyEvent(eAction, pObj, pData1, pData2, pData3) ; 

  // SM_NO_REG_PROPAGATION passes region arrays through the SmObject slots.
  // The tracker has no handling for this event, so do not inspect those opaque pointers.
  if(eAction == SM_NO_REG_PROPAGATION)
    { return SM_SUCCESS ; }

  SmObject * pTrackedObject = pData1 ;
  if(eAction == SM_NO_CHANGE_GEOMETRY || eAction == SM_NO_POST_EDIT)
    { pTrackedObject = pObj ; }
  else if(eAction == SM_NO_SPLIT || eAction == SM_NO_MERGE)
    { pTrackedObject = pData3 ; }

  if(   (eAction == SM_NO_POST_EDIT && pObj
         && (pObj->IsKindOf(SmSurface_TYPE) || pObj->IsKindOf(SmCurve_TYPE)))
     || eAction == SM_NO_SPLIT || eAction == SM_NO_MERGE)
    {
      // Generator geometry can nest curves and surfaces in either order.
      // UV trim curves reach their edge through an edgeuse.
      while(pTrackedObject)
        {
          if(pTrackedObject->IsKindOf(SmSurface_TYPE))
            { pTrackedObject = ((SmSurface *)pTrackedObject)->GetOwner() ; }
          else if(pTrackedObject->IsKindOf(SmCurve_TYPE))
            { pTrackedObject = ((SmCurve *)pTrackedObject)->GetOwner() ; }
          else if(pTrackedObject->IsKindOf(SmEdgeuse_TYPE))
            { pTrackedObject = ((SmEdgeuse *)pTrackedObject)->GetEdge() ; }
          else
            { break ; }
        }
    }

  // currently - only track Faces, Edges, and Vertices
  if(   pTrackedObject != NULL
     && (   pTrackedObject->IsKindOf(SmFace_TYPE)
         || pTrackedObject->IsKindOf(SmEdge_TYPE)
         || pTrackedObject->IsKindOf(SmVertex_TYPE)))
    {
      // switch on eAction
      switch(eAction)
        {
          case SM_NO_ADD_TO_BREP          : { pBrep      = SM_CAST_PTR(SmBrep, pObj) ;
                                              m_pNotifyEvents->Add(sNotifyEvent) ;
                                              if(AddUnique(m_pAddList, (SmTopology *)pData1, pBrep)) { SM_DBG_LOG( _T("From AddToBrep  : AddLIST Added NewObj"), pData1, pBrep) ; }
                                                                                                    //    #ifdef SM_DEBUG_CODE
                                                                                                    //      if(m_pAddListData) m_pAddListData->Add(((SmTopology *)pData1)->GetType()) ; 
                                                                                                    //      if(m_pAddListData) m_pAddListData->Add(eAction) ; 
                                                                                                    //      if(m_pAddListData) m_pAddListData->Add(SM_UNDEF_ULONG) ; 
                                                                                                    //    #endif // SM_DEBUG_CODE
                                            }        
                                              break ;
                                         
          case SM_NO_SPLIT_IN_BREP        : { pBrep      = SM_CAST_PTR(SmTopology, pData2) ? SM_CAST_PTR(SmTopology, pData2)->GetBrep() : NULL ; 
                                              m_pNotifyEvents->Add(sNotifyEvent) ;
                                              if(AddUnique(m_pRmList, (SmTopology*)pData1, pBrep))   { SM_DBG_LOG(_T("From SplitInBrep: RmList Added Parent"), pData1, pBrep) ;
                                                                                                       // #ifdef SM_DEBUG_CODE                                                                                 
                                                                                                       //   if(m_pRmListData) m_pRmListData->Add(((SmTopology *)pData1)->GetType()) ; 
                                                                                                       //   if(m_pRmListData) m_pRmListData->Add(eAction) ; 
                                                                                                       // #endif // SM_DEBUG_CODE    
                                                                                                     }                                    
                                              if(Remove   (m_pAddList, (SmTopology *)pData1))        { SM_DBG_LOG( _T("From SplitInBrep: AddList Rm    Parent"), pData1, pBrep) ; }
                                              if(Remove   (m_pChgList, (SmTopology *)pData1))        { SM_DBG_LOG( _T("From SplitInBrep: ChgList Rm    Parent"), pData1, pBrep) ; }
                                              if(AddUnique(m_pAddList, (SmTopology *)pData2, pBrep)) { SM_DBG_LOG( _T("From SplitInBrep: AddList Added Child1"), pData2, pBrep) ; }
                                              if(AddUnique(m_pAddList, (SmTopology *)pData3, pBrep)) { SM_DBG_LOG( _T("From SplitInBrep: AddList Added Child2"), pData3, pBrep) ; }
                                            } break ;
                                         
          case SM_NO_MERGE_IN_BREP        : { pBrep      = SM_CAST_PTR(SmBrep, pData3) ;
                                              m_pNotifyEvents->Add(sNotifyEvent) ;
                                              if(AddUnique (m_pRmList, (SmTopology *)pData1, pBrep)) { SM_DBG_LOG( _T("From MergeInBrep: RmList Added Input1"), pData1, pBrep) ;  
                                                                                                       // #ifdef SM_DEBUG_CODE                     
                                                                                                       //  if(m_pRmListData) m_pRmListData->Add(((SmTopology *)pData1)->GetType()) ; 
                                                                                                       //  if(m_pRmListData) m_pRmListData->Add(eAction) ; 
                                                                                                       // #endif // SM_DEBUG_CODE                  
                                                                                                     } 
                                              if(AddUnique (m_pRmList, (SmTopology *)pData2, pBrep)) { SM_DBG_LOG( _T("From MergeInBrep: RmList Added Input2"), pData2, pBrep) ;  
                                                                                                       // #ifdef SM_DEBUG_CODE                     
                                                                                                       //   if(m_pRmListData) m_pRmListData->Add(((SmTopology *)pData2)->GetType()) ; 
                                                                                                       //   if(m_pRmListData) m_pRmListData->Add(eAction) ; 
                                                                                                       // #endif // SM_DEBUG_CODE                  
                                                                                                     } 
                                              if(Remove   (m_pAddList, (SmTopology *)pData1))        { SM_DBG_LOG( _T("From MergeInBrep: AddList Rm    Input1"), pData1, pBrep) ; }
                                              if(Remove   (m_pAddList, (SmTopology *)pData2))        { SM_DBG_LOG( _T("From MergeInBrep: AddList Rm    Input2"), pData2, pBrep) ; }
                                              if(Remove   (m_pChgList, (SmTopology *)pData1))        { SM_DBG_LOG( _T("From MergeInBrep: ChgList Rm    Input1"), pData1, pBrep) ; }
                                              if(Remove   (m_pChgList, (SmTopology *)pData2))        { SM_DBG_LOG( _T("From MergeInBrep: ChgList Rm    Input2"), pData2, pBrep) ; }
                                              if(AddUnique(m_pAddList, (SmTopology *)pData1, pBrep)) { SM_DBG_LOG( _T("From MergeInBrep: AddList Added Result"), pData1, pBrep) ; }
                                            } break ;
                                         
          case SM_NO_TRIM_NO_SPLIT_IN_BREP: { pBrep      = SM_CAST_PTR(SmBrep, pObj) ;
                                              m_pNotifyEvents->Add(sNotifyEvent) ;
                                              if(AddUnique (m_pRmList, (SmTopology *)pData1, pBrep)) { SM_DBG_LOG( _T("From InsSeamInBrep: RmList Added TgtObj"), pData1, pBrep) ;  
                                                                                                       // #ifdef SM_DEBUG_CODE                     
                                                                                                       //    if(m_pRmListData) m_pRmListData->Add(((SmTopology *)pData1)->GetType()) ; 
                                                                                                       //    if(m_pRmListData) m_pRmListData->Add(eAction) ; 
                                                                                                       // #endif // SM_DEBUG_CODE                  
                                                                                                     } 
                                              if(Remove   (m_pChgList, (SmTopology *)pData1))        { SM_DBG_LOG( _T("From InsSeamInBrep: ChgList Rm    TgtObj"), pData1, pBrep) ; }
                                              if(AddUnique(m_pAddList, (SmTopology *)pData1, pBrep)) { SM_DBG_LOG( _T("From InsSeamInBrep: AddList Added TgtObj"), pData1, pBrep) ; }
                                            } break ;
                                         
          case SM_NO_RM_FROM_BREP         : { 
                                              pBrep      = SM_CAST_PTR(SmBrep, pObj) ;
                                              m_pNotifyEvents->Add(sNotifyEvent) ;
                                              if(AddUnique (m_pRmList, (SmTopology *)pData1, pBrep)) { SM_DBG_LOG( _T("From RmFromBrep : RmList Added RmObj"), pData1, pBrep) ; 
                                                                                                       // #ifdef SM_DEBUG_CODE                     
                                                                                                       //    if(m_pRmListData) m_pRmListData->Add(((SmTopology *)pData1)->GetType()) ; 
                                                                                                       //    if(m_pRmListData) m_pRmListData->Add(eAction) ; 
                                                                                                       // #endif // SM_DEBUG_CODE                  
                                                                                                     } 
                                              if(Remove   (m_pAddList, (SmTopology *)pData1))        { SM_DBG_LOG( _T("From RmFromBrep : RmObj Rm  from AddList"), pData1, pBrep) ; }
                                              if(Remove   (m_pChgList, (SmTopology *)pData1))        { SM_DBG_LOG( _T("From RmFromBrep : RmObj Rm  from ChgList"), pData1, pBrep) ; }
                                            } break ;
                                         
          case SM_NO_COINCIDENT           : { } break ;
          case SM_NO_CHANGE_GEOMETRY      : { 
                                              pBrep      = SM_CAST_PTR(SmBrep, pData2) ;
                                              m_pNotifyEvents->Add(sNotifyEvent) ;
                                              if(AddUnique(m_pChgList, (SmTopology *)pTrackedObject, pBrep)) { SM_DBG_LOG( _T("From ChgGeometry: ChgList Added Owner"), pTrackedObject, pBrep) ; }
                                            } break ;
          case SM_NO_CHANGE_OWNER         : { } break ;
          case SM_NO_CONSTRUCTION         : { } break ;
          case SM_NO_COPY                 : { } break ;
          case SM_NO_PRE_EDIT             : { } break ;
          case SM_NO_POST_EDIT            : {
                                              pBrep = SM_CAST_PTR(SmBrep, pData2) ;
                                              m_pNotifyEvents->Add(sNotifyEvent) ;
                                              if(pTrackedObject == pObj)
                                                {
                                                  // Completed topology edits must rebuild HealData properties,
                                                  // which are refreshed through the remove/add lists.
                                                  if(AddUnique(m_pRmList, (SmTopology *)pTrackedObject, pBrep))  { SM_DBG_LOG( _T("From PostEdit   : RmList Added TgtObj"), pTrackedObject, pBrep) ; }
                                                  if(AddUnique(m_pAddList, (SmTopology *)pTrackedObject, pBrep)) { SM_DBG_LOG( _T("From PostEdit   : AddList Added TgtObj"), pTrackedObject, pBrep) ; }
                                                }
                                              else if(AddUnique(m_pChgList, (SmTopology *)pTrackedObject, pBrep)) { SM_DBG_LOG( _T("From PostEdit   : ChgList Added Owner"), pTrackedObject, pBrep) ; }
                                            } break ;
          case SM_NO_SPLIT                : { 
                                              pBrep      = SM_CAST_PTR(SmTopology, pData3) ? SM_CAST_PTR(SmTopology, pData3)->GetBrep() : NULL ;
                                              m_pNotifyEvents->Add(sNotifyEvent) ;
                                              if(AddUnique(m_pChgList, (SmTopology *)pTrackedObject, pBrep)) { SM_DBG_LOG( _T("From SplitGeom  : ChgList Added Owner"), pTrackedObject, pBrep) ; }
                                            } break ;
          case SM_NO_MERGE                : { 
                                              pBrep      = SM_CAST_PTR(SmTopology, pData3) ? SM_CAST_PTR(SmTopology, pData3)->GetBrep() : NULL ;
                                              m_pNotifyEvents->Add(sNotifyEvent) ;
                                              if(AddUnique(m_pChgList, (SmTopology *)pTrackedObject, pBrep)) { SM_DBG_LOG( _T("From MergeGeom  : ChgList Added Owner"), pTrackedObject, pBrep) ; }
                                            } break ;
          case SM_NO_REG_PROPAGATION      : { } break ;
          case SM_NO_DESTRUCTION          : {
                                              // An object is being freed. If it was tracked as added or changed, drop it
                                              // now so a dangling (freed) pointer never survives into consumers such as
                                              // SmHealData::UpdateTargetLists, which dereferences every AddList entry via
                                              // GetAt(ii)->IsKindOf(). This happens when a heal op (e.g. CombineCoincident-
                                              // Vertices) creates then frees temporary topology, or frees an object without
                                              // a preceding SM_NO_RM_FROM_BREP. RmList entries are looked up by identity only
                                              // (stale-tolerant), so leaving a freed object there for props cleanup is safe.
                                              if(Remove(m_pAddList, (SmTopology *)pData1)) { SM_DBG_LOG( _T("From Destruct   : AddList Rm    DelObj"), pData1, pBrep) ; }
                                              if(Remove(m_pChgList, (SmTopology *)pData1)) { SM_DBG_LOG( _T("From Destruct   : ChgList Rm    DelObj"), pData1, pBrep) ; }
                                            } break ;
          default                         : break ;
       } // end switch on eAction
    } // end is Face, Edge, or Vertex check

#undef SM_DBG_LOG

  // all done 
  return SM_SUCCESS ;

} // end SmTopologyChangeCallback::Execute

/*******************************************************************//**
PURPOSE: SmTrackTopologyChanges constructor

NOTES: Init internal members and start SmContext::m_SysNotifyCallback
       SmObject::Notify() based topoGraph change tracking
***********************************************************************/
SmTrackTopologyChanges::SmTrackTopologyChanges
 (SmBrep  * pBrep,     // in : Tgt Brep whose topology graph changes are being tracked
  SmBoolean bStartNow) // in : TRUE = start topo change tracking now, FALSE = don't
                       //      default:[TRUE}
: m_pBrep(pBrep),
  m_pLastNotifyCallback(NULL),
  m_sTopologyChangeCallback(&m_sRmList, 
                            &m_sAddList, 
                            &m_sChgList,
                            &m_sNotifyList
                                              
#ifdef SM_DEBUG_CODE
                        //    ,&m_sRmListData
                        //    ,&m_sAddListData
                           ,&m_sRmListIndx 
                           ,&m_sAddListIndx
                           ,&m_sChgListIndx
#endif // no SM_DEBUG_CODE
                           )
{ 
  // when asked start tracking now - else just init the Rm and Add arrays.
  if(bStartNow) { StartTracking() ; }
  else          { ReSetArrays() ; }

} // end SmTrackTopologyChanges constructor

/*******************************************************************//**
PURPOSE: ReSet Rm and Add Arrays and Load SmContext::SysNotifyCallback
         with m_sTopologyChangeCallback to turn on the SmObject::Notify() 
         mechanism for tracking Brep TopologyGraph topology Object changes.
NOTES: 
***********************************************************************/
void SmTrackTopologyChanges::StartTracking()
{ 
#ifdef SM_DEBUG_CODE
#ifdef SM_TOPOCHANGE_NOTIFY    
  SmBoolean bDebugLog = TRUE ;
#else // no SM_TOPOCHANGE_NOTIFY
  SmBoolean bDebugLog = FALSE ;
#endif // no SM_TOPOCHANGE_NOTIFY

  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  if(bDebugLog)
    {
      // eNotifyOperation label
      smos_WriteBuffer(_T("\n\nEntering StartTracking()")) ;
      smos_sprintf(sBuff,        _T("\n  In StartTracking(): m_pLastNotifyCallback:InVal :[0x%p (should be NULL)], "), m_pLastNotifyCallback) ;
      smos_sprintf(sBuffForFile, _T("\n  In StartTracking(): m_pLastNotifyCallback:InVal :[%s (should be NULL)], "),   m_pLastNotifyCallback ? _T("NotNULL") : _T("NULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;
    }
#endif // SM_DEBUG_CODE

  // init internal arrays
  ReSetArrays() ;

  // when m_pBrep and its Context exist
  if(m_pBrep != NULL && m_pBrep->GetContext() != NULL)
    { 
      // start TopoGraph Change Tracking
      SmContext * pContext = (SmContext *)m_pBrep->GetContext() ;
      m_pLastNotifyCallback = pContext->GetSysNotifyCallback() ;

      // when SmContext::m_pSysNotifyCallBack called in SmObject::Notify() is not in use - load it with m_sTopologyChangeCallback (future: allow nested uses of SysNotifyCallback())
      if(m_pLastNotifyCallback == NULL)
        { pContext->SetSysNotifyCallback(&m_sTopologyChangeCallback) ; }
      else // inform the public: trying to nest use of SysNotifyCallback 
        { SM_ASSERT_MSG(m_pLastNotifyCallback != NULL,
                       _T("SmTrackTopologyChanges::Constructor error: SmContext::m_SysNotifyCallback TopoGraph change tracking already turned on - no action taken")) ;
        }
    } // end valid Brep existence check
  else // uninit m_pBrep->m_cpContext branch
    { 
      // inform the public
      SM_ASSERT_MSG(m_pBrep != NULL && m_pBrep->GetContext() != NULL,
                   _T("SmTrackTopologyChanges::Constructor error: invalid constructor argument - was not given valid pBrep with a Context")) ;
    }

#ifdef SM_DEBUG_CODE
  if(bDebugLog)
    {
      // eNotifyOperation label
      smos_sprintf(sBuff,        _T("\n  In StartTracking(): m_pLastNotifyCallback:OutVal:[0x%p] (temp holder of SmContext::m_pSysNotifyCallback - expected:[NULL=NoNesting] - else a bug until nested tracking calls are allowed"), m_pLastNotifyCallback) ;
      smos_sprintf(sBuffForFile, _T("\n  In StartTracking(): m_pLastNotifyCallback:OutVal:[%s] (temp holder of SmContext::m_pSysNotifyCallback - expected:[NULL=NoNesting] - else a bug until nested tracking calls are allowed"),   m_pLastNotifyCallback ? _T("NotNULL") : _T("NULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;
      smos_WriteBuffer(_T("\nExit StartTracking()")) ;
    } 
#endif // SM_DEBUG_CODE

} // end SmTrackTopologyChanges::StartTracking()

/*******************************************************************//**
PURPOSE: Return TRUE if this SmTrackTopologyChanges NOtifyCallback is
         currently loaded into the context SysNotifyCallback slot.
NOTES:
***********************************************************************/
SmBoolean SmTrackTopologyChanges::IsTracking() const 
{ 
  // set return TRUE when m_sTopologyChangeCallback is loaded into context->SysNotifyCallback slot
  SmContext * pContext = m_pBrep ? (SmContext *)m_pBrep->GetContext() : NULL ;
  SmBoolean   bRtn     = pContext && (pContext->GetSysNotifyCallback() == &m_sTopologyChangeCallback) ;

  return( bRtn ) ; 

} // end SmTrackTopologyChanges::IsTracking()

/*******************************************************************//**
PURPOSE: Clear the Context->SysNotifyCallback pointer to turn off
         tracking of Brep Topology Graph object changes.
NOTES:
***********************************************************************/
void SmTrackTopologyChanges::StopTracking()
{
#ifdef SM_DEBUG_CODE
#ifdef SM_TOPOCHANGE_NOTIFY    
  SmBoolean bDebugLog = TRUE ;
#else // no SM_TOPOCHANGE_NOTIFY
  SmBoolean bDebugLog = FALSE ;
#endif // no SM_TOPOCHANGE_NOTIFY

  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  if(bDebugLog)
    {
      smos_WriteBuffer(_T("\n\nEntering StopTracking() - Ask for SmTrackTopologyChanges Dump")) ;
      Dump() ;

      // eNotifyOperation label
      smos_sprintf(sBuff,        _T("\n  In  StopTracking(): in m_pLastNotifyCallback:[0x%p = val placed back in SmContext::m_pSysNotifyCallback - should be NULL for NoNesting], "), m_pLastNotifyCallback) ;
      smos_sprintf(sBuffForFile, _T("\n  In  StopTracking(): in m_pLastNotifyCallback:[%s = val placed back in SmContext::m_pSysNotifyCallback - should be NULL for NoNesting], "), m_pLastNotifyCallback ? _T("NotNULL") : _T("NULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;
    }
#endif // SM_DEBUG_CODE

  // when m_pBrep and its Context exist
  if(m_pBrep != NULL && m_pBrep->GetContext() != NULL)
    { 
      // when LastNotifyCallback was NULL (it was turned on by this object)
      if(m_pLastNotifyCallback == NULL) 
        { 
          // turn off SmObject::Notify() tracking by setting SmContext::m_pSysNotifyCallback ptr to NULL
          ((SmContext *)m_pBrep->GetContext())->SetSysNotifyCallback(m_pLastNotifyCallback) ;
          
          // clear this objects - m_pLastNotifyCallback value
          m_pLastNotifyCallback = NULL ;

        } // end m_pLastNotifyCallback existence check
      else // were in an illegal nested tracking call
        {
          // inform the public
          SM_ASSERT_MSG(m_pLastNotifyCallback == NULL, _T("\n   SmTrackTopologyChanges::StopTracking error - TopoTracking does not support nested tracking calls")) ;
        }
    } // end m_pBrep->m_cpContext existence check

#ifdef SM_DEBUG_CODE
  if(bDebugLog)
    {
      // eNotifyOperation label
      smos_sprintf(sBuff,        _T("\n  In StopTracking(): out m_pLastNotifyCallback:[0x%p] (should be NULL)"), m_pLastNotifyCallback) ;
      smos_sprintf(sBuffForFile, _T("\n  In StopTracking(): out m_pLastNotifyCallback:[%s] (should be NULL)"), m_pLastNotifyCallback ? _T("NotNULL") : _T("NULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;
      smos_WriteBuffer(_T("\nExit StopTracking()")) ;

    }
#endif // SM_DEBUG_CODE

} // end SmTrackTopologyChanges::StopTracking()

/*******************************************************************//**
PURPOSE: SmTrackTopologyChanges Pretty Print

NOTES:
***********************************************************************/
void SmTrackTopologyChanges::Dump()
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // locals
  SmContext * pContext = m_pBrep ? (SmContext *)m_pBrep->GetContext() : NULL ;
  SmBoolean   bLogging = (pContext && pContext->GetSysNotifyCallback()) ;

  // Begin
  smos_sprintf(sBuff, _T("\nBegin SmTrackTopologyChanges Dump  - Logging is %s"), bLogging ? _T("ON") : _T("OFF")) ;
  smos_WriteBuffer(sBuff);
                                                                                                              
  // Header 
  smos_sprintf(sBuff,        _T("\n SmTrackTopologyChanges     :[0x%p]"), this) ;
  smos_sprintf(sBuffForFile, _T("\n SmTrackTopologyChanges     :[%s]"),   _T("notNULL")) ;  // replaced: this     ? _T("notNULL") : _T("NULL")) ; comparing this to NULL generates a Linux warning.  
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  smos_sprintf(sBuff,        _T("\n for m_pBrep                :[0x%p]"), m_pBrep) ;
  smos_sprintf(sBuffForFile, _T("\n for m_pBrep                :[%s]"),   m_pBrep  ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  smos_sprintf(sBuff,        _T("\n in  Context                :[0x%p]"), pContext) ;
  smos_sprintf(sBuffForFile, _T("\n in  Context                :[%s]"),   pContext ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  smos_sprintf(sBuff,        _T("\n Context->LastNotifyCallback:[0x%p]"), m_pLastNotifyCallback) ;
  smos_sprintf(sBuffForFile, _T("\n Context->LastNotifyCallback:[%s]"),   m_pLastNotifyCallback ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  smos_sprintf(sBuff,        _T("\n Context->SysNotifyCallback :[0x%p] %s "), (pContext && pContext->GetSysNotifyCallback()) ? pContext->GetSysNotifyCallback() : NULL, 
                                                                              bLogging ? _T("- logging ON:should be Same") : _T("- Logging OFF:should be NULL")) ;
  smos_sprintf(sBuffForFile, _T("\n Context->SysNotifyCallback :[%s] %s "),   (pContext && pContext->GetSysNotifyCallback()) ? _T("notNULL") : _T("NULL"),
                                                                              bLogging ? _T("- logging ON:should be Same") : _T("- Logging OFF:should be NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  smos_sprintf(sBuff,        _T("\n this->NotifyCallback       :[0x%p] %s "), &m_sTopologyChangeCallback, 
                                                                              bLogging ? _T("- logging ON:should be Same") : _T("")) ;
  smos_sprintf(sBuffForFile, _T("\n this->NotifyCallback       :[%s] %s "), _T("notNULL"), 
                                                                            bLogging ? _T("- logging ON:should be Same") : _T("")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // the "Remove List" header
    {
      smos_sprintf(sBuff,        _T("\n  RmList Size:[%ld], Ptr:[0x%p], alloc size:[%ld]"), m_sRmList.GetSize(), this,          m_sRmList.GetDataSize()) ;
      smos_sprintf(sBuffForFile, _T("\n  RmList Size:[%ld], Ptr:[%s], alloc size:[%ld]"),   m_sRmList.GetSize(), _T("NotNULL"), m_sRmList.GetDataSize()) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;
      
      // for every m_sRmList entry
      for(ii=0;ii<m_sRmList.GetSize();ii++)
        {
#ifdef SM_DEBUG_CODE
        //   smos_sprintf(sBuff,        _T("\n    [%3ld] = 0x%p  Type:[%ld %s] Indx:[%ld] from:%s"), ii,  
        //                                                                                           m_sRmList[ii],  
        //                                                                                           m_sRmListData[3*ii], 
        //                                                                                           SM_TOPO_TYPENAME( m_sRmListData[3*ii] ), 
        //                                                                                           m_sRmListData[3*ii+2], 
        //                                                                                           SM_NOTIFYOPERATIONTYPE_NAME( m_sRmListData[3*ii+1] )) ;
        //   smos_sprintf(sBuffForFile, _T("\n    [%3ld] = %s  Type:[%ld %s] Indx:[%ld] from:%s"), ii,  
        //                                                                                         (m_sRmList[ii]) ? _T("NotNULL") : _T("NULL"),  
        //                                                                                         m_sRmListData[3*ii], 
        //                                                                                         SM_TOPO_TYPENAME( m_sRmListData[3*ii] ), 
        //                                                                                         m_sRmListData[3*ii+2], 
        //                                                                                         SM_NOTIFYOPERATIONTYPE_NAME( m_sRmListData[3*ii+1] )) ;
#else // no SM_DEBUG_CODE
          smos_sprintf(sBuff,        _T("\n    [%3ld] = 0x%p"), ii, m_sRmList[ii]) ;
          smos_sprintf(sBuffForFile, _T("\n    [%3ld] = %s"),   ii, (m_sRmList[ii]) ? _T("NotNULL") : _T("NULL")) ;
#endif // no SM_DEBUG_CODE
          smos_WriteBuffer(sBuff, sBuffForFile) ;
        } // end iter sRmList members

   } // end RmList

  // the "Add List" header
    {
      smos_sprintf(sBuff,        _T("\n  AddList Size:[%ld], Ptr:[0x%p], alloc size:[%ld]  "), m_sAddList.GetSize(), this, m_sAddList.GetDataSize()) ;
      smos_sprintf(sBuffForFile, _T("\n  AddList Size:[%ld], Ptr:[%s], alloc size:[%ld]  "), m_sAddList.GetSize(), _T("NotNULL"), m_sAddList.GetDataSize()) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;

      // for every m_sAddList entry
      for(ii=0;ii<m_sAddList.GetSize();ii++)
        {
// #ifdef SM_DEBUG_CODE
          // smos_sprintf(sBuff,        _T("\n    [%3ld] = 0x%p  Type:[%ld %s] Indx:[%ld] from:%s"), ii,  
          //                                                                                         m_sAddList[ii],  
          //                                                                                         m_sAddListData[3*ii], 
          //                                                                                         SM_TOPO_TYPENAME( m_sAddListData[3*ii] ), 
          //                                                                                         m_sAddListData[3*ii+2], 
          //                                                                                         SM_NOTIFYOPERATIONTYPE_NAME( m_sAddListData[3*ii+1] )) ;
          // smos_sprintf(sBuffForFile, _T("\n    [%3ld] = %s  Type:[%ld %s] Indx:[%ld] from:%s"), ii,  
          //                                                                                       (m_sAddList[ii]) ? _T("NotNULL") : _T("NULL"),  
          //                                                                                       m_sAddListData[3*ii], 
          //                                                                                       SM_TOPO_TYPENAME( m_sAddListData[3*ii] ), 
          //                                                                                       m_sAddListData[3*ii+2], 
          //                                                                                       SM_NOTIFYOPERATIONTYPE_NAME( m_sAddListData[3*ii+1] )) ;
// #else // no SM_DEBUG_CODE
          // smos_sprintf(sBuff,        _T("\n    [%3ld] = 0x%p  Type:[%ld %s]"), ii, m_sAddList[ii], m_sAddList[ii]->GetType(), m_sAddList[ii]->GetClassString()) ;
          // smos_sprintf(sBuffForFile, _T("\n    [%3ld] = %s  Type:[%ld %s]"), ii, (m_sAddList[ii]) ? _T("NotNULL") : _T("NULL"), m_sAddList[ii]->GetType(), m_sAddList[ii]->GetClassString()) ;
          // smos_WriteBuffer(sBuff, sBuffForFile) ;
// #endif // no SM_DEBUG_CODE
          // gwc - what we want is to print objects rather than addresses of array elements
          // for now just print the m_pData entries as pointer values
          smos_sprintf(sBuff,        _T("\n    [%3ld] = 0x%p"), ii, m_sAddList[ii]) ;
          smos_sprintf(sBuffForFile, _T("\n    [%3ld] = %s"),   ii, (m_sAddList[ii]) ? _T("NotNULL") : _T("NULL")) ;
          smos_WriteBuffer(sBuff, sBuffForFile) ;

#ifdef SM_DEBUG_CODE
          smos_sprintf(sBuff,        _T(", Indx:[%lu]"), m_sAddListIndx[ii]) ;
          smos_WriteBuffer(sBuff) ;
#endif // SM_DEBUG_CODE

          smos_sprintf(sBuff,        _T(" Type:[%ld %s]"), m_sAddList[ii]->GetType(), m_sAddList[ii]->GetClassString()) ;
          smos_WriteBuffer(sBuff) ;
        } // iter sAddList members

    } // end AddList

  // the "Chg List" header
    {
      smos_sprintf(sBuff,        _T("\n  ChgList Size:[%ld], Ptr:[0x%p], alloc size:[%ld]  "), m_sChgList.GetSize(), this, m_sChgList.GetDataSize()) ;
      smos_sprintf(sBuffForFile, _T("\n  ChgList Size:[%ld], Ptr:[%s], alloc size:[%ld]  "), m_sChgList.GetSize(), _T("NotNULL"), m_sChgList.GetDataSize()) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;

      // for every m_sChgList entry
      for(ii=0;ii<m_sChgList.GetSize();ii++)
        {
          // gwc - what we want is to print objects rather than addresses of array elements
          // for now just print the m_pData entries as pointer values
          smos_sprintf(sBuff,        _T("\n    [%3ld] = 0x%p"), ii, m_sChgList[ii]) ;
          smos_sprintf(sBuffForFile, _T("\n    [%3ld] = %s"),   ii, (m_sChgList[ii]) ? _T("NotNULL") : _T("NULL")) ;
          smos_WriteBuffer(sBuff, sBuffForFile) ;

#ifdef SM_DEBUG_CODE
          smos_sprintf(sBuff,        _T(", Indx:[%lu]"), m_sChgListIndx[ii]) ;
          smos_WriteBuffer(sBuff) ;
#endif // SM_DEBUG_CODE

          smos_sprintf(sBuff,        _T("  Type:[%ld %s]"), m_sChgList[ii]->GetType(), m_sChgList[ii]->GetClassString()) ;
          smos_WriteBuffer(sBuff) ;
        } // iter sChgList members

    } // end ChgList

  // the "NotifyEvent List" header
    {
      smos_sprintf(sBuff,        _T("\n  NotifyEventList Size:[%ld], Ptr:[0x%p], alloc size:[%ld]  "), m_sNotifyList.GetSize(), this, m_sNotifyList.GetDataSize()) ;
      smos_sprintf(sBuffForFile, _T("\n  NotifyEventList Size:[%ld], Ptr:[%s], alloc size:[%ld]  "), m_sNotifyList.GetSize(), _T("NotNULL"), m_sNotifyList.GetDataSize()) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;

      // for every m_sChgList entry
      for(ii=0;ii<m_sNotifyList.GetSize();ii++)
        {
          // gwc - what we want is to print objects rather than addresses of array elements
          // for now just print the m_pData entries as pointer values
          m_sNotifyList[ii].Dump() ;
        } // iter sChgList members

    } // end NotifyEventList


  // End
  smos_sprintf(sBuff, _T("%s"), _T("\nEnd SmTrackTopologyChanges Dump")) ;
  smos_WriteBuffer(sBuff);

} // end SmTrackTopologyChanges::Dump
