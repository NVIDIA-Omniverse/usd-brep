// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmTrackTopologyChanges.h
* PURPOSE: Header file for SmTrackTopologyChanges.
**********************************************************************/

#ifndef __SM_TRACKTOPOLOGYCHANGES_H__
#define __SM_TRACKTOPOLOGYCHANGES_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMOBJECT_H__
#include <SmObject.h>
#endif

#ifndef __SMOS_MATH_H_
#include <SmMath.h> 
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif // no __SMTARRAY_H__

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

class SmNotifyEvent ;

/*******************************************************************//**
PURPOSE:  Derived SmNotifyCallback class 

NOTES: 1. Meant for system use and to be an example of what can be 
       loaded into the SmContext::m_pUserNotifyCallback
       member to allow an application to have a chance to take an action
       on every SmObject::Notify() call.

       2. This example maintains two lists of ptrs to Topology objects
          a. all objects removed from the topology graph
          b. all objects added to the topology graph

          It's possible to remove a ptr from a topology graph, delete it
          and then reuse it in a subsequent new SmTopology() call and
          then add the new topology object using the same old topology
          ptr back into the topology graph.  That's one ptr that
          was used to point to two different objects, the one that
          was removed and the one that was added.  It's even possible
          that after the second obj using the ptr value was added
          it gets removed and a third obj might be placed on the list.

          The lists in this implementation are accumulative and unique
            1. all objects ptrs that are removed are uniquely represented
               on the rm list - even those ptrs that get reused
               and added back into the topology graph.
            2. all objects ptrs that are added are uniquely represented
               on the add list.  When an obj ptr is removed the
               add list is checked and if it contains the ptr that
               ptr value is removed from the add list.

          After a sequence of calls the rm list will contain all ptr values
          that once pointed to any topology obj removed from the topology graph.
          Unless these pointer values get reused those pointers are expected to
          be stale - don't dereference values through these pointers.  Only use
          them as handles to know which objects have been removed so that
          any array of obj pointers the app is maintaining can be updated.

          The add list will contain only those objs which have been added
          and were not subsequently removed from the topology graph.  For
          a single pointer which gets frequently reused the rm list will 
          contain it once and the add list will contain it only if the
          last time it was added happened after the last time it was removed.

        3. the expeced use of this callback execute scheme is to create
           a list of all the obj ptr values that were removed and all
           the object ptrs that were added after an atomic modification
           operation has run.  It's possible that an obj ptr is on
           both the add and remove lists if the ptr was once used for
           an object that was removed and then resued for a second object
           that was added.  These lists ignore the case of an
           ptr that gets reused multiple times for multiple objects
           that get added and removed only giving the state from the
           beginning of the track sequence to the end.
***********************************************************************/
class SM_EXPORT SmTopologyChangeCallback : public SmNotifyCallback
{
 public:
  SmTArray<SmTopology*>   * m_pRmList       = NULL ; // list of all topology objects notified of a removal - treat as stale pointers
  SmTArray<SmTopology*>   * m_pAddList      = NULL ; // list of all topology objects notified of a addition
  SmTArray<SmTopology*>   * m_pChgList      = NULL ; // list of all topology objects notified of a geometry change
  SmTArray<SmNotifyEvent> * m_pNotifyEvents = NULL ; // list of all events that modified the Rm, Add, or Chg lists

#ifdef SM_DEBUG_CODE
  SmTArray<ULONG>       * m_pRmListIndx  = NULL ;  // Brep TopologyObj Array Indx value of associated RmList entry at time of removal
  SmTArray<ULONG>       * m_pAddListIndx = NULL ;  // Brep TopologyObj Array Indx value of associated AddList entry at time of add
  SmTArray<ULONG>       * m_pChgListIndx = NULL ;  // Brep TopologyObj Array Indx value of associated chgList entry at time of chg
#endif // SM_DEBUG_CODE

 public:
  // constructor
  SmTopologyChangeCallback(SmTArray<SmTopology*>   * pRmList       =NULL, // in : list of all topology objects notified of a removal
                           SmTArray<SmTopology*>   * pAddList      =NULL, // in : list of all topology objects notified of a addition
                           SmTArray<SmTopology*>   * pChgList      =NULL, // in : list of all topology objects notified of a geometry change
                           SmTArray<SmNotifyEvent> * pNotifyEvents =NULL  // in : list of all events that modified the Rm, Add, or Chg lists
   #ifdef SM_DEBUG_CODE                                            
                          , SmTArray<ULONG>        * pRmListIndx   =NULL  // Brep TopologyObj Array Indx value of associated RmList entry at time of removal
                          , SmTArray<ULONG>        * pAddListIndx  =NULL  // Brep TopologyObj Array Indx value of associated AddList entry at time of add
                          , SmTArray<ULONG>        * pChgListIndx  =NULL  // Brep TopologyObj Array Indx value of associated chgList entry at time of chg
   #endif // no SM_DEBUG_CODE
                          ) : m_pRmList(pRmList),
                              m_pAddList(pAddList),
                              m_pChgList(pChgList),
                              m_pNotifyEvents(pNotifyEvents)
   #ifdef SM_DEBUG_CODE
                             ,m_pRmListIndx (pRmListIndx )
                             ,m_pAddListIndx(pAddListIndx)
                             ,m_pChgListIndx(pChgListIndx)
   #endif // no SM_DEBUG_CODE
                            { }

  // copy constructor
  SmTopologyChangeCallback(const SmTopologyChangeCallback &crOther) { *this = crOther ; } // avoid duplicate code - call assignment operator

  // assignment operator
  SmTopologyChangeCallback & operator=(const SmTopologyChangeCallback &crOther) { if(&crOther == this) { return *this; }
                                                                                  if(m_pRmList )      { m_pRmList ->ReSet() ;      m_pRmList ->Append(*crOther.m_pRmList) ; }
                                                                                  if(m_pAddList)      { m_pAddList->ReSet() ;      m_pAddList->Append(*crOther.m_pAddList) ; }
                                                                                  if(m_pChgList)      { m_pChgList->ReSet() ;      m_pChgList->Append(*crOther.m_pChgList) ; }
                                                                                  if(m_pNotifyEvents) { m_pNotifyEvents->ReSet() ; m_pNotifyEvents->Append(*crOther.m_pNotifyEvents) ; }
                                                                                  #ifdef SM_DEBUG_CODE
                                                                                    if (m_pRmListIndx)  { m_pRmListIndx->ReSet() ;  m_pRmListIndx->Append(*crOther.m_pRmListIndx) ; }
                                                                                    if (m_pAddListIndx) { m_pAddListIndx->ReSet() ; m_pAddListIndx->Append(*crOther.m_pAddListIndx) ; }
                                                                                    if (m_pChgListIndx) { m_pChgListIndx->ReSet() ; m_pChgListIndx->Append(*crOther.m_pChgListIndx) ; }
                                                                                  #endif // SM_DEBUG_CODE
                                                                                  return *this ;
                                                                                 }
  // equality operator
  SmBoolean operator==(const SmTopologyChangeCallback &crOther)                  { SmBoolean bRtn = TRUE ;
                                                                                   bRtn &= m_pRmList       == crOther.m_pRmList ;
                                                                                   bRtn &= m_pAddList      == crOther.m_pAddList ;
                                                                                   bRtn &= m_pChgList      == crOther.m_pChgList ;
                                                                                   bRtn &= m_pNotifyEvents == crOther.m_pNotifyEvents ; 
                                                                                   return(bRtn) ;
                                                                                 }
  // destructor
  ~SmTopologyChangeCallback() { if(m_pRmList)       m_pRmList->ReSet() ;
                                if(m_pAddList)      m_pAddList->ReSet() ; 
                                if(m_pChgList)      m_pChgList->ReSet() ;
                                if(m_pNotifyEvents) m_pNotifyEvents->ReSet() ;
                                #ifdef SM_DEBUG_CODE
                                  if(m_pRmListIndx ) m_pRmListIndx->ReSet() ; 
                                  if(m_pAddListIndx) m_pAddListIndx->ReSet() ; 
                                  if(m_pChgListIndx) m_pChgListIndx->ReSet() ; 
                                #endif // SM_DEBUG_CODE

                              }
   // associated array management - in debug mode manages associted ListIndx arrays
   SmBoolean AddUnique(SmTArray<SmTopology*> * m_pTgtList, SmTopology * pTgt, SmBrep * pBrep) ;
   SmBoolean Remove   (SmTArray<SmTopology*> * m_pTgtList, SmTopology * pTgt) ; 
   
   #ifdef SM_DEBUG_CODE
     void SetArrays(SmTArray<SmTopology*>   * pRmList,       // in : list of all topology objects notified of a removal
                    SmTArray<SmTopology*>   * pAddList,      // in : list of all topology objects notified of a addition
                    SmTArray<SmTopology*>   * pChgList,      // in : list of all topology objects notified of a geometry change
                    SmTArray<SmNotifyEvent> * pNotifyEvents, // in : list of all events that modified Rm, Add, or Chg lists
                    SmTArray<ULONG>         * pRmListIndx,   // in : Brep TopologyObj Array Indx value of associated RmList entry at time of removal
                    SmTArray<ULONG>         * pAddListIndx,  // in : Brep TopologyObj Array Indx value of associated AddList entry at time of add
                    SmTArray<ULONG>         * pChgListIndx)  // in : Brep TopologyObj Array Indx value of associated chgList entry at time of chg
                   { m_pRmList       = pRmList ;
                     m_pAddList      = pAddList ;
                     m_pChgList      = pChgList ;
                     m_pNotifyEvents = pNotifyEvents ; 
                     m_pRmListIndx  = pRmListIndx ; 
                     m_pAddListIndx = pAddListIndx ; 
                     m_pChgListIndx = pChgListIndx ; 
                   }
   #else // no SM_DEBUG_CODE
     void SetArrays(SmTArray<SmTopology*>   * pRmList,       // in : list of all topology objects notified of a removal
                    SmTArray<SmTopology*>   * pAddList,      // in : list of all topology objects notified of a addition
                    SmTArray<SmTopology*>   * pChgList,      // in : list of all topology objects notified of a geometry change
                    SmTArray<SmNotifyEvent> * pNotifyEvents) // in : list of all events that modified Rm, Add, or Chg lists
                   { m_pRmList       = pRmList ;
                     m_pAddList      = pAddList ;
                     m_pChgList      = pChgList ;
                     m_pNotifyEvents = pNotifyEvents ;
                   }
   #endif // no SM_DEBUG_CODE

  // ReSetArray
  void ReSetArrays()   { if(m_pRmList)  m_pRmList->ReSet() ;
                         if(m_pAddList) m_pAddList->ReSet() ;
                         if(m_pChgList) m_pChgList->ReSet() ;
                         if(m_pNotifyEvents) m_pNotifyEvents->ReSet() ; 
                         #ifdef SM_DEBUG_CODE
                        //   if(m_pRmListData)  m_pRmListData->ReSet() ; 
                        //   if(m_pAddListData) m_pAddListData->ReSet() ; 
                           if(m_pRmListIndx ) m_pRmListIndx ->ReSet() ; 
                           if(m_pAddListIndx) m_pAddListIndx->ReSet() ; 
                           if(m_pChgListIndx) m_pChgListIndx->ReSet() ; 
                         #endif // SM_DEBUG_CODE
                       }
  void ReSetRmArray()  { if(m_pRmList)  m_pRmList->ReSet()  ;
                         #ifdef SM_DEBUG_CODE
                        // if(m_pRmListData)  m_pRmListData->ReSet() ; 
                           if(m_pRmListIndx ) m_pRmListIndx ->ReSet() ;
                         #endif // SM_DEBUG_CODE
                       }
  void ReSetAddArray() { if(m_pAddList) m_pAddList->ReSet() ;
                         #ifdef SM_DEBUG_CODE
                        // if(m_pAddListData) m_pAddListData->ReSet() ; 
                           if(m_pAddListIndx ) m_pAddListIndx ->ReSet() ;
                         #endif // SM_DEBUG_CODE
                       }
  void ReSetChgArray() { if(m_pChgList) m_pChgList->ReSet() ; 
                         #ifdef SM_DEBUG_CODE
                           if(m_pChgListIndx ) m_pChgListIndx ->ReSet() ;
                         #endif // SM_DEBUG_CODE
                       }
  void ReSetNotifyEvents() { if(m_pNotifyEvents) m_pNotifyEvents->ReSet() ; }

  // the method that gets called once every time SmObject::Notify() is called
  virtual SmStatus Execute                        
   (SmNotifyOperation  eAction,   /* in : event  */  // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                      //
    SmObject         * pObj,      /* in : caller */  //       event                | caller      |  pData1  | pData2                | pData3                    //
    SmObject         * pData1,    /* in : pData1 */  //----------------------------+-------------+----------+-----------------------+-------------------------- //
    SmObject         * pData2,    /* in : pData2 */  // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL  //
    SmObject         * pData3)  ; /* in : pData3 */  // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                    //
                                                     // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      //
                                                     // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                      //
                                                     // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB                     //
                                                     // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   //
                                                     // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           //
                                                     // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          //
                                                     // SM_NO_CONSTRUCTION         | NewObj      | NewObj   | CopyFromObj or NULL    | NULL                     //
                                                     // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   //
                                                     // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      //
                                                     // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      //
                                                     // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL  //
                                                     // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL  //
                                                     // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg        //
                                                     // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     //

} ; // end SmTopologyChangeCallback

/*******************************************************************//**
PURPOSE: Track history of changes to the Change Map by tracking 
         the parent to child relationships of the changes accumulated
         in a SmTrackTopologyChanges.

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
class SM_EXPORT SmNotifyEvent
{
 public:
  SmNotifyOperation   m_eAction = SM_NO_UNKNOWN ;                             /* in : event  */
  SmObject          * m_pCaller = NULL ; ULONG m_lCallerIndx = SM_UNDEF_ULONG ; /* in : caller */
  SmObject          * m_pData1  = NULL ; ULONG m_lData1Indx  = SM_UNDEF_ULONG ; /* in : pData1 */
  SmObject          * m_pData2  = NULL ; ULONG m_lData2Indx  = SM_UNDEF_ULONG ; /* in : pData2 */ 
  SmObject          * m_pData3  = NULL ; ULONG m_lData3Indx  = SM_UNDEF_ULONG ; /* in : pData3 */
 public:
  // constructor 
  SmNotifyEvent(SmNotifyOperation  eAction = SM_NO_UNKNOWN,  // in : event 
                SmObject         * pCaller = NULL,           // in : caller
                SmObject         * pData1  = NULL,           // in : pData1
                SmObject         * pData2  = NULL,           // in : pData2
                SmObject         * pData3  = NULL)           // in : pData3 '
               : m_eAction(eAction),
                 m_pCaller(pCaller), m_lCallerIndx(SM_UNDEF_ULONG),
                 m_pData1 (pData1 ), m_lData1Indx (SM_UNDEF_ULONG),
                 m_pData2 (pData2 ), m_lData2Indx (SM_UNDEF_ULONG),
                 m_pData3 (pData3 ), m_lData3Indx (SM_UNDEF_ULONG)
               { }
 
  // equality operator
  SmBoolean operator==(const SmNotifyEvent &crOther) { SmBoolean bRtn = TRUE ;
                                                       bRtn &= m_eAction == crOther.m_eAction ;
                                                       bRtn &= m_pCaller == crOther.m_pCaller ;  
                                                       bRtn &= m_pData1  == crOther.m_pData1 ;
                                                       bRtn &= m_pData2  == crOther.m_pData2 ;
                                                       bRtn &= m_pData3  == crOther.m_pData3 ;
                                                       return(bRtn) ;
                                                     }
  // destructor
  ~SmNotifyEvent() { ReSet() ; } 

  // operators
  void ReSet() { m_eAction = SM_NO_UNKNOWN ;
                 m_pCaller = NULL ;
                 m_pData1  = NULL ;
                 m_pData2  = NULL ;
                 m_pData3  = NULL ;
               }

  void Dump() const ;

} ; // end SmNotifyEvent   

/*******************************************************************//**
PURPOSE: example class using SmContext::SysNotifyCallback to accumulate
         changes to the Brep topology graph

NOTES: 1. manages the SmContext::m_SysNotifyCallback based TopoGraph Change 
       Tracking by turning it on when an instance of the SmTrackTopologyChanges
       class is constructed and turning it off when destructed.

       2. This class is used by the SmHealData class - don't change this
       when creating a new Callback based application.  Just copy and edit.
       
       3. Arguably the classes SmTopologyChangeCallback and SmTrackTopologiesChanges
       could have been one class, as implemented the memory to store
       the Add and Rm lists is in SmTrackTopologyChanges.  The methods
       that load and edit that memory is in class SmTopologyChangeCallback.
***********************************************************************/
class SM_EXPORT SmTrackTopologyChanges
{
 protected:
   // for use by Fix functions to track additions and removals to the Brep Topology Graph   
  SmBrep                  * m_pBrep = NULL ;         // Brep whose topology graph changes are being tracked
  SmTArray<SmTopology *>    m_sRmList ;              // list of all topology removed from the m_pBrep topology graph after a StartChangeTracking()
                                                     // and since the last ReSet_RmList() call.  All ptrs that were removed from the topology
                                                     // graph are on this list once - even those ptrs that get removed, deleted, reused, added, and removed again.
  #ifdef SM_DEBUG_CODE      
//  SmTArray<SM_TYPE>      m_sRmListData ;          // for debug only - currently [type of objs, NotifyType] for each RmList item. for Dump() pretty prints.
//                                                  // could be anything, perhaps a copy of the SmTopology obj that got Rm and deleted
    SmTArray<ULONG>         m_sRmListIndx ;          // associated list of RmList member topology index values - for debug pretty printing only                                        
    SmTArray<ULONG>         m_sAddListIndx ;         // associated list of RmList member topology index values - for debug pretty printing only                                         
    SmTArray<ULONG>         m_sChgListIndx ;         // associated list of RmList member topology index values - for debug pretty printing only                                         
  #endif // SM_DEBUG_CODE   
    SmTArray<SmTopology *>  m_sAddList ;             // list of all topology currently in the m_pBrep topology graph that have been added
                                                     // after a StartChangeTracking() call and since the last ReSet_RmList() call.
                                                     // An object ptr added and removed from the topology graph won't be in this list.
                                                     // An object ptr added, removed, deleted, reused, added again will be in this list.
                            
    SmTArray<SmTopology *>  m_sChgList ;             // list of all topology objects whose geometries have been redefined
                            
    SmTArray<SmNotifyEvent> m_sNotifyList ;          // capture the event history that added and removed elements from Add and Rm lists                                                      
                                                      
//  #ifdef SM_DEBUG_CODE     
//    SmTArray<SM_TYPE>      m_sRmListData ;          // for debug and Dump() pretty prints only 
//                                                    //   - for each RmList item [ Object type,                                  ]
//                                                    //                          [ Notify type,                                  ]
//                                                    //                          [ Associated HealObjProps Indx or SM_UNDEF_ULONG]
//                                                    // could be anything, perhaps a copy of the SmTopology obj that got Rm and deleted
//
//    SmTArray<SM_TYPE>      m_sAddListData ;         // for debug and Dump() pretty prints only 
//                                                    //   - for each AddList item [ Object type,                                  ]
//                                                    //                           [ Notify type,                                  ]
//                                                    //                           [ Associated HealObjProps Indx or SM_UNDEF_ULONG]
//  #endif // SM_DEBUG_CODE                           // could be anything, perhaps a copy of the SmTopology obj that got Rm and deleted
                            
  SmNotifyCallback        * m_pLastNotifyCallback = NULL ; // Temp holder to restore the SmContext::m_pSysNotifyCallback when ChangeTracking is stopped.
  SmTopologyChangeCallback  m_sTopologyChangeCallback ;    // object containing the SmNotifyCallback ptr to drive Topology graph changes
                                                           // note: setup with m_sTopologyChangeCallback.SetArrrays(&m_sRmList, &m_AddList) - currently in ReSet()
 public:                   
   // constructor - when asked, also Start topology change tracking in SmObject::Notify() by setting pContext->m_pSysNotifyCallBack
   SmTrackTopologyChanges(SmBrep  * pBrep,        // in : Tgt Brep whose topology graph changes are being tracked
                          SmBoolean bStartNow) ;  // in : TRUE = start topo change tracking now in SmObject::Notify() by
                                                  //             setting pContext->m_pSysNotifyCallBack = &m_sTopologyChangeCallback
                                                  //      FALSE= don't

   // destructor - if topology change tracking is on, stop it
  ~SmTrackTopologyChanges() { if(IsTracking()) { StopTracking() ; } }

   // state management
   void      StartTracking() ;    // set SmContext::m_pSysNotifyCallback with m_sTopologyChangeCallback to be called in SmObject::Notify()
   void      StopTracking()  ;    // set SmContext::m_pSysNotifyCallback to NULL
   SmBoolean IsTracking() const ; // rtn TRUE = (SmContext::m_pSysNotifyCallback == m_sTopologyChangeCallback)

   // simple data access
   SmTArray<SmTopology *>  * GetRmList()       { return &m_sRmList ; }
   SmTArray<SmTopology *>  * GetAddList()      { return &m_sAddList ; }
   SmTArray<SmTopology *>  * GetChgList()      { return &m_sChgList ; }
   SmTArray<SmNotifyEvent> * GetNotifyEvents() { return &m_sNotifyList; }

 //  #ifdef SM_DEBUG_CODE                        
 //    SmTArray<SM_TYPE>    * GetRmListData()    { return &m_sRmListData ; }
 //    SmTArray<SM_TYPE>    * GetAddListData()   { return &m_sAddListData ; }
//   #endif // SM_DEBUG_CODE                     

   // predicates
   SmBoolean IsInRmList (SmTopology * pTgt) const { return m_sRmList. IsIn(pTgt) ; }
   SmBoolean IsInAddList(SmTopology * pTgt) const { return m_sAddList.IsIn(pTgt) ; }
   SmBoolean IsInChgList(SmTopology * pTgt) const { return m_sChgList.IsIn(pTgt) ; }

   // list management
   void ReSetArrays()     { m_sTopologyChangeCallback.ReSetArrays() ; }
   void ReSetAddList()    { m_sTopologyChangeCallback.ReSetAddArray() ; }
   void ReSetRmList()     { m_sTopologyChangeCallback.ReSetRmArray() ; }
   void ReSetChgList()    { m_sTopologyChangeCallback.ReSetChgArray() ; }
   void ReSetNotifyList() { m_sTopologyChangeCallback.ReSetNotifyEvents() ; }

   void Dump() ;

} ; // end class SmTrackTopologyChanges

#endif // !__SM_TRACKTOPOLOGYCHANGES_H__
