// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmContext.h
* PURPOSE: Header file for SmContext.
**********************************************************************/

#ifndef __SMCONTEXT_H_
#define __SMCONTEXT_H_

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMOBJECT_H__
#include <SmObject.h>
#endif

#ifndef __SMOS_MATH_H_
#include <SmMath.h> 
#endif

// forward declarations
class SmGlobalCache ;
class SmVolume ;
class SmGlobalCacheSize ;

// for backward compatibility
#ifndef SmUserCallback
#define SmUserCallback SmNotifyCallback
#endif // SmUserCallback

/*******************************************************************//**
PURPOSE:  NotifyCallback class.

NOTES:
   0. See SmTopologyChangeCallback.[h,cpp] for an example of using
      the SysNotifyCallback mechanism to track topology object
      additions and removals from a Brep TopologyGraph.
   1. This is an abstract class meant to give user applications
      a handle on tracking the creation, addition, modification, 
      removal and deletion of topology objects within a context.
      This handle can be used by applications that mirror the 
      current set of topology objects to connect information
      within the brep to information in their own appliations.
   2. An instance to SmNotifyCallback can be stored and accessed
      within the SmContext::m_pUserNotifyCallback member with calls:
        SmContext::SetUserNotifyCallback() ;
        SmContext::GetUserNotifyCallback() ;
   3. When SmContext::m_pUserNotifyCallback is nonNULL its 
      SmNotifyCallback::Execute() method is called once
      for every call made to SmObject::Notify()

  To use:
  - Derive your own class from SmNotifyCallback;
  - Implement its Execute() method to do what you want;
  - Define an object of that class;
  - Assign that object to the relevant SmContext object
    using SmContext::SetUserCallback().

  The Execute() method would presumably branch on the type of operation,
  and on the type of the object:
    SM_TYPE eObjType = pObj->GetType();
    switch ( eObjType ) ...
***********************************************************************/
class SM_EXPORT SmNotifyCallback
{
 public:
  // provide default constructor and virtual destructor
  SmNotifyCallback() { }
  virtual ~SmNotifyCallback() { }

  // sole virtual method - called once for every SmObject::Notify() call caused by one of the caller->Notify() calls listed below.
  virtual SmStatus Execute                        
   (SmNotifyOperation  eAction,   /* in : event  */  // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                      //
    SmObject         * pObj,      /* in : caller */  //       event                | caller      |  pData1  | pData2                | pData3                    //
    SmObject         * pData1,    /* in : pData1 */  //----------------------------+-------------+----------+-----------------------+-------------------------- //
    SmObject         * pData2,    /* in : pData2 */  // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL  //
    SmObject         * pData3)=0; /* in : pData3 */  // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                    //
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
} ; // end class SmNotifyCallback

/*******************************************************************//**
PURPOSE:  The SmContext object is used to share contextual information
    throughout the SMLib functions accessable through almost all
    SMLib objects.  This Object is used to contain any 'global' data
    used by the NMT Library.
    
NOTES: 

    SMLib overloads the New constructor requiring
    an SmContext object as an argument.  Every Object derived
    from SmObject stores a pointer to the SmContext with which
    it was built.
    
    Currently the contextual information contains, 
      1. a memory pool handle (when using SmartHeap),
      2. a pointer to the global Brep, Surface, TrimmedSurface, 
           and Curve cache queues.
           2a. Only true if SM_USE_GLOBAL_CACHE is defined
           2b. Global cache is to be deprecated
      3. function state information used by several modeling methods 
         including a bDoingBoolean bit and several general mark values.  
           The marks are used by SmTopology and SmAttribute type objects to 'mark'
           individual objects as already done so that it can be skipped on
           subsequent passes of iterative or recursive functions.
           use: SmNewMarkAndLock sMarkLock ;    // increment and lock any unlocked mark.
                                                // unlocks the mark when sMarkLock goes out of scope.
                SmMarkType eMarkType = sMarkLock.GetMarkType() ;
                SmTopology::Mark(eMarkType)     sets the individual SmTopology::m_lMark value 
                                                to the current SmContext::m_Mark value.
                SmTopology::UnMark(eMarkType)   clears the current value
                SmTopology::IsMarked(eMarkType) compares the individual to the global mark value.
      4. a list of Attribute Callbacks to allow application specified
         attribute propagation and persistence behavior.
      5. a general void * pointer, available to applications to store
         'global' data.
      6. Dedicated memory for use by distinct SMLib functions. 
      
SmContext must stay a static class without virtual methods because
new was overloaded with a method that allocates memory with smos_Calloc()       
***********************************************************************/
class SM_EXPORT SmContext
{
  friend class SmTol ;
  friend class SmBrep ; // allow only SmBrep access to SmContext::GetThisModelSizeEstimate()
  friend class SmHealData ;

protected: 
  ULONG     m_lCurrentMark       = 1 ;    // Current topology mark to compare with object SmTopology::m_lMark  values
  ULONG     m_lCurrentMark2      = 1 ;    // Current topology mark to compare with object SmTopology::m_lMark2 values
  ULONG     m_lCurrentMark3      = 1 ;    // Current topology mark to compare with object SmTopology::m_lMark3 values
  ULONG     m_lCurrentMarkIO     = 1 ;    // Current topology mark used by info routines to prevent conflicts with Mark and Mark2 users
  ULONG     m_lCurrentMarkAssert = 1 ;    // Current topology mark used by AssertValid routines
public:                                   
  SmBoolean m_bMarkLock          = 0 ;    // TRUE  = Locked - A debug only warning is signalled when calling NewMark
  SmBoolean m_bMark2Lock         = 0 ;    // FALSE = Not Locked
  SmBoolean m_bMark3Lock         = 0 ;    // FALSE = Not Locked
  SmBoolean m_bMarkIOLock        = 0 ;    
  SmBoolean m_bMarkAssertLock    = 0 ;    // note: NewStyle: use class SmNewMarkAndLock() to get and lock available marks 
                                          //       Old Style:
                                          //    The Lock members are public so that a function can call one of the following
                                          //    SmTemporaryChangeValue<SmBoolean> sLock(brep->GetContext()->m_bMarkLock, TRUE);
                                          //    SmTemporaryChangeValue<SmBoolean> sLock2(brep->GetContext()->m_bMark2Lock, TRUE);
                                          //    SmTemporaryChangeValue<SmBoolean> sLock3(brep->GetContext()->m_bMark3Lock, TRUE);
                                          //    SmTemporaryChangeValue<SmBoolean> sLockIO(brep->GetContext()->m_bMarkIOLock, TRUE);
                                          //    SmTemporaryChangeValue<SmBoolean> sLockAssert(brep->GetContext()->m_bMarkAssertLock, TRUE);
                                          // to lock a mark and then to unlock the mark automatically when the
                                          // current scope closes
protected:
  // The flag will be used to optimize cache creation during boolean operations
  SmBoolean m_bDoingBoolean = FALSE ;     // TRUE = During a boolean operation - delay surface cache tree construction
                                          // FALSE= Normal operation for all functions

    // { begin long temporary note on NewTolerance/OldTolerance model problems  (same comment in SmBrep.h)
       // New/Old Tolerance Problem:
       //          This code includes a confused mix of old an new tolerance data fields and needs to be cleaned
       //          up to just use the new tolerance model.
       // WorkAround: For now the two values m_sBrepZoneTol3d and m_sThisZoneTol3d are tied together so that
       //                  SmBrep::m_sBrepZoneTol3d == SmBrep::m_sThisZoneTol3d.
       // Real Solution: Implement the new tolerance model design for Brep tolerances
       //                The working tolerance used as the value to define the local neighborhoods for all Brep::Vertex, Edge, and Face objs
       //                is not stored in SmBrep but rather in the SmBrep->cpContext.  A pointer to that cpContext is in every Vertex, Edge, and Face definition.
       //                Changing the SmContext::m_sThisZoneTol3d will change the tolerance for all entities sharing this SmContext object.
       // 
       //                    Water tightness:  1. When a Vertex, Edge, or Face is connected to another topology object through a gap that is larger than
       //                                         the Context's working tolerance, the tolerances of the topology objects are increased so that
       //                                         the local neighborhoods of the two topology objects intersect.  In the NewTolerance model
       //                                         a topology's object is a function of it's Context->ThisZoneTol3d value and it's max Gap size.
       //                                      2. To avoid 'sticky' large tolerance when topology connections are made through large gaps, the
       //                                         local neighborhoods of the vertices, edges, and faces will not be simply increased, but rather
       //                                         a local neighborhood will be defined for the Vertexuses and Edgeuses associated with the large gaps.
       //                                         This idea increases the topology object's local neighborhood by a minimal amount to make sure that
       //                                         it's local neighborhood intersects the local neighborhood of the large-gap connected-neighbor wihtout
       //                                         inadvertantly getting such a large local neighborhood that it starts to have tolerant-intersections with
       //                                         geometry to which it is not supposed to be connected.
       //                    Picking the Context->m_sThisZoneTol3d value:
       //                                         Working SmContext::m_sThisZoneTol3d values should be selected based on the size of the Brep being modeled,
       //                                         ie. pinholes should use different tolerances than battleships. When a new Brep is constructed it
       //                                         won't have any geometry to size so the SmContext::ThisZoneTol3d value is computed from
       //                                         a ModelSizeEstimate value.  Large ranges of ModelSize measures get rounded into a small set
       //                                         of ModelSizeEstimates so that parts built on the same order of magnitude all share a common tolerance value.
       //                                         So, the initial value of SmContext::m_sThisZoneTol3d comes from a ModelSize measure or guess.  For parts being imported
       //                                         that already have geometry defined, the ModelSize can come from any measure of the models size, eg. The model's MaxEdgeLength,
       //                                         the Model's bounding box, or even the bounding box of just the model's vertices. For models being created
       //                                         interactively the ModelSize comes from a guess about how big the part will be when completed.  That guess
       //                                         can be supplied by the user or come from a default system value called SM_USE_DEFAULT.  A very large range
       //                                         of model sizes will all use the same working ZoneTol3d value and for parts that fit in a bounding box between
       //                                         0.5 and 500 units the default SM_USE_DEFAULT will be just fine.
       //                                          code:
       //                                           // no need to call SmTol::SizeToModelSizeEstimate() before calling SmTol::SizeEstimateToZoneTol3d()
       //                                           // both functions encode the same ModelSize_to_ZoneTol3d tables
       //                                           // SmBrep::m_dThisModelSizeEstimate = SmTol::SizeToModelSizeEstimate(AnyModelSizeMeasure,
       //                                           //                                                                   a User's model size guess, or the
       //                                           //                                                                   SM_USE_DEFAULT def value)   (<== recommended in most cases) 
       //                                           SmBrep::m_dThisModelSizeEstimate = (AnyModelSizeMeasure,
       //                                                                               a User's model size guess, or the
       //                                                                               SM_USE_DEFAULT def value)   (<== recommended in most cases) 
       //                                           SmBrep::m_sThisZoneTol3d         = SmTol::SizeEstimateToZoneTol3d(m_dThisModelSizeEstimate) ;
       //   Real Solution TODOs list:
       //                  1. Remove ZoneTol3d, ThisZoneTol3d, ThisModelSizeEstimate from Brep - update all the SmTol methods to fetch the right values.
       //                  2. Work on Context constructor to set m_dThisModelSizeEstimate and m_dThisZoneTol3d values
       //                       2a.  large ranges of ModelSize measures get rounded into a small set of ModelSizeEstimates with call
       //                            SmBrep::m_dThisModelSizeEstimate = (AnyModelSizeMeasure, a User's model size guess, or sys def SM_MODEL_SIZE_ESTIMATE)    
       //                       2b.  The mapping from ModelSizeEstimate to Brep ZoneTol3d values is roughly:
       //                                ModelSizeEstimate:[1.0e4    - 1.0e5    ] => ZoneTol3d:[1.0e-3]: rare
       //                                ModelSizeEstimate:[100      - 10,000   ] => ZoneTol3d:[1.0e-4]: occasionally
       //                                ModelSizeEstimate:[  1      -    100   ] => ZoneTol3d:[1.0e-5]: default
       //                                ModelSizeEstimate:[  0.01   -      1   ] => ZoneTol3d:[1.0e-6]: occasionally
       //                                ModelSizeEstimate:[  0.0001 -      0.01] => ZoneTol3d:[1.0e-7]: rare
       //                  3. Set Brep::GetThisModelSizeEstimate() = Brep->cpContext->GetThisModelSizeEstimate()
       //                  4. Set Brep::GetThisZoneTol3d()         = Brep->cpContext->GetThisZoneTol3d()
       //                  5. Cache MaxGap sizes on the vertexuses and edgeuses.
       //                  6. modify the classifications and intersectors to look for intersections with large-gap vertex uses and large-gap edge uses.
       //                  7. WorkAround: Until the New model/Old model implementation is cleaned up and Brep::m_sZoneTol3d is removed,
       //                            SmBrep::m_sBrepZoneTol3d = SmBrep::m_sThisZoneTol3d
       
       // The new and old tolerance models have been released simultaneously and need to be filtered down to a single NewTolModel value.
       //  Old TolModel: m_sBrepZoneTol3d is an independent value set by default, through the constructor, or explicitly by a call to SetTolerance()
       //  New TolModel: A SizeEstimate is stored by the Brep->cpContext, from which the following values are cached for convenience
       //                  SmContext::m_dThisModelSizeEstimate  ==>  SmContext::m_dThisZoneTol3d      = SmTol::SizeEstimateToZoneTol3d(SmContext::m_dThisModelSizeEstimate) ;
       //                                                       ==>  SmBrep::GetThisModelSizeEstimate() = pBrep->SmContext->GetThisModelSizeEstimate()
       //                                                       ==>  SmBrep::GetThisZoneTol3d()         = pBrep->SmContext->GetThisZoneTol3d()
       //                Seting SmContext::m_dThisModelSizeEstimate from Default           : SM_USE_DEFAULT ==> SM_MODEL_SIZE_ESTIMATE
       //                                                           From ModelSize measure : BrepSize Measure
       //                                                           From User estimate     : BrepSize UserEstimate
  // } end long temporary note on NewTolerance model
  double m_dThisModelSizeEstimate = SM_USE_DEFAULT ;   // Tol UserInterface: Specify Expected Model Size => Get System calculated Tol Size
                                                       // note: When m_dThisModelSizeEstimate == SM_USE_DEFAULT
                                                       //        uses system SmTol::GetModelSizeEstimate() value 
                                                       // Note:  size:[1.0e4    - 1.0e5    ] => m_sZoneTol3d:[1.0e-3]: rare
                                                       //        size:[100      - 10,000   ] => m_sZoneTol3d:[1.0e-4]: occasionally
                                                       //        size:[  1      -    100   ] => m_sZoneTol3d:[1.0e-5]: default
                                                       //        size:[  0.01   -      1   ] => m_sZoneTol3d:[1.0e-6]: occasionally
                                                       //        size:[  0.0001 -      0.01] => m_sZoneTol3d:[1.0e-7]: rare
                                                       // default:[SM_USE_DEFAULT=Rtn SystemVal, default:[SM_USE_DEFAULT=50] - only for experts
                                          
  double m_dThisLargeSmallSizeRatio = SM_USE_DEFAULT ; // Model Pinholes in Battleships (very rare) - give small topology tighter tolerances
                                                       // note: When m_dThisLargeSmallSizeRatio == SM_USE_DEFAULT
                                                       //        uses system SmTol::GetLargeSmallSizeRatio() value 
                                                       // Note: ZoneTol3d =   Obj->IsSmallTopology() 
                                                       //                   ? m_sZoneTol3d/m_dLargeSmallSizeRatio 
                                                       //                   : m_sZoneTol3d ;
                                                       // default:[SM_USE_DEFAULT=Rtn SystemVal, default:[SM_LARGE_SMALL_SIZE_RATIO = 1000.0] - only for experts
                                          
  SmZoneTol3d m_sThisZoneTol3d = SM_ZONE_TOL_3D ;      // cached: can't change val directly. Gets sets by  
                                                       //     SetModelSizeEstimate(pContext, dModelSize)
                                                       // NOTE: Used to calc: Degenerate Geometry and Topology sizes
                                                       //                     Approximation Tolerances
                                                       //                     Intersection distances
                                                       //                     Tolerant Neighborhood Sizes for Surfaces, Curves, and Points
               
  SmObject         * m_pAttributeCallbacks = NULL ;  // This will be an array of void* {SmTArray<void*> * pCallBacks}
#ifdef SM_USE_GLOBAL_CACHE                           
  SmGlobalCache    * m_pGlobalCache        = NULL ;  // Global cache data goes here
#endif                                               
  void             * m_pGlobalUserData     = NULL ;  // Global user data
                                                     
  SmNotifyCallback * m_pUserNotifyCallback = NULL ;  // Optional User SmObject::Notify callback object. 
                                                     // Signature: Callback(SmNotifyOperation eAction,    ///< [in] : oneof: SM_NO_COPY, COPY_REPLACE, SPLIT, MERGE, PRE_EDIT, POST_EDIT, CONSTRUCTION, DESTRUCTION 
                                                     //                     SmObject *pNotigyTgt,         ///< [in] : Target of Object call
                                                     //                     SmObject *pData1              ///< [in] : depending on eAction value: an Object or NULL pointer, see SmTypes.h
                                                     //                     SmObject *pData2,             ///< [in] : depending on eAction value: an Object or NULL pointer, see SmTypes.h 
                                                     //                     SmObject *pData3)             ///< [in] : An optional extra action notification.
                                                     // Once this pointer is set, 
                                                     // it gets called once for every SmObject::Notify() call made within SMLib.
                                                     // This mechanism allows an application to track all the topology and geometry object, 
                                                     // construction, destruction, and modifications made as a consequence of high level API calls.
                                                     // Note, the function pointed to is not owned by SmObject:
                                                     // the function is not deleted when reset, or when this is deleted.
  SmNotifyCallback * m_pSysNotifyCallBack = NULL ;   // system reserved callback obj used by methods that want to track changes reported through Notify()
                                                     // For Now - this callback mechanism is just one level deep so methods that use
                                                     // smObject::Notify() change tracking can't call methods that also use the feature.
                                                     // In the future if the system grows so that nested NotifyCallBacks are needed
                                                     // change this member into an Array and change related implementations
                 
public:
#ifdef SM_USE_GLOBAL_CACHE
  // constructor - with ThisModelSizeEstimate value
  SmContext
  (
    double dThisModelSizeEstimate,     ///< [in] : ModelSizeEstimate to Set Tolerances, use SM_USE_DEFAULT unless you're an expert
    double dThisLargeSmallSizeRatio=SM_USE_DEFAULT  ///< [in] : for Pinholes in Battleships, use SM_USE_DEFAULT unless you're an expert
  ) ; 

  // constructor - with CacheSizes and ModelSizeEstimate value
  SmContext
  (
    ULONG lCurveCacheCount    = SM_DEFAULT_MAXCOUNT_CURVECACHE  ,    ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
    ULONG lSurfaceCacheCount  = SM_DEFAULT_MAXCOUNT_SURFACECACHE,    ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
    ULONG lTrimSrfCacheCount  = SM_DEFAULT_MAXCOUNT_TRIMSRFCACHE,    ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
    ULONG lBrepCacheCount     = SM_DEFAULT_MAXCOUNT_BREPCACHE   ,    ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                       
    ULONG lCurveCacheByteSize   = SM_DEFAULT_BYTESIZE_CURVECACHE  ,  ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
    ULONG lSurfaceCacheByteSize = SM_DEFAULT_BYTESIZE_SURFACECACHE,  ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
    ULONG lTrimSrfCacheByteSize = SM_DEFAULT_BYTESIZE_TRIMSRFCACHE,  ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
    ULONG lBrepCacheByteSize    = SM_DEFAULT_BYTESIZE_BREPCACHE,     ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
    double dThisModelSizeEstimate  =SM_USE_DEFAULT,                  ///< [in] : ModelSizeEstimate for Tols, don't change unless you're an expert
    double dThisLargeSmallSizeRatio=SM_USE_DEFAULT                   ///< [in] : for Pinholes in Battleships Tols, don't change unless you're an expert
  ) ;                

  // destructor and overloaded new/delete 
  ~SmContext();  
#else
  // constructor - with ThisModelSizeEstimate value
  SmContext
  (
    double dThisModelSizeEstimate  =SM_USE_DEFAULT,                  ///< [in] : ModelSizeEstimate for Tols, don't change unless you're an expert
    double dThisLargeSmallSizeRatio=SM_USE_DEFAULT                   ///< [in] : for Pinholes in Battleships Tols, don't change unless you're an expert
  ) ;                

#endif // SM_USE_GLOBAL_CACHE

  // simple data access
  SmObject         * GetAttributeCallbacks()   const { return  m_pAttributeCallbacks; }
  void             * GetGlobalUserData()       const { return  m_pGlobalUserData; }
  SmBoolean          GetDoingBoolean()         const { return  m_bDoingBoolean ; }
  SmBoolean        & GetDoingBooleanRef()            { return  m_bDoingBoolean ; }
  SmNotifyCallback * GetUserNotifyCallback()   const { return  m_pUserNotifyCallback ; }
  SmNotifyCallback * GetUserCallback()         const { return  GetUserNotifyCallback() ; }
  SmNotifyCallback * GetSysNotifyCallback()          { return  m_pSysNotifyCallBack ; }
#ifdef SM_USE_GLOBAL_CACHE
  SmGlobalCache  * GetGlobalCache()            const { return m_pGlobalCache; }
  void             GetGlobalCacheSize (SmGlobalCacheSize &rGlobalCacheSize) const ;
#endif // SM_USE_GLOBAL_CACHE

  protected: // Tolerance Management
  // public access to Tol values: SmTol::GetZoneTol3d(pBrep)            (don't change vals unless you're an expert)
  //                              SmTol::GetModelSizeEstimate(pBrep)    SmTol::SetModelSizeEstimate(pBrep, NewVal)
  //                              SmTol::GetLargeSmallSizeRatio(pBrep)  SmTol::SetLargeSmallSizeRatio(pBrep, NewVal)
// #ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE SmZoneTol3d GetThisZoneTol3d()           const { return m_sThisZoneTol3d ; }
  SM_NEWTOL_LINE double      GetThisModelSizeEstimate()   const { return   m_dThisModelSizeEstimate == SM_USE_DEFAULT
  SM_NEWTOL_LINE                                                         ? SmTol::GetModelSizeEstimate()
  SM_NEWTOL_LINE                                                         : m_dThisModelSizeEstimate ; 
  SM_NEWTOL_LINE                                                }
  SM_NEWTOL_LINE double      GetThisLargeSmallSizeRatio() const { return   m_dThisLargeSmallSizeRatio == SM_USE_DEFAULT
  SM_NEWTOL_LINE                                                         ? SmTol::GetLargeSmallSizeRatio()
  SM_NEWTOL_LINE                                                         : m_dThisLargeSmallSizeRatio ;
  SM_NEWTOL_LINE                                                }
  SM_NEWTOL_LINE SmZoneTol3d SetThisModelSizeEstimate(double dThisModelSizeEstimate=SM_USE_DEFAULT)  
  SM_NEWTOL_LINE                                                { m_dThisModelSizeEstimate = dThisModelSizeEstimate ;
  SM_NEWTOL_LINE                                                  m_sThisZoneTol3d         = SmTol::SizeEstimateToZoneTol3d(m_dThisModelSizeEstimate) ; 
  SM_NEWTOL_LINE                                                  return(m_sThisZoneTol3d) ;
  SM_NEWTOL_LINE                                                }
  SM_NEWTOL_LINE void        SetThisLargeSmallSizeRatio(double dThisLargeSmallSizeRatio=SM_USE_DEFAULT)   
  SM_NEWTOL_LINE                                                { m_dThisLargeSmallSizeRatio = dThisLargeSmallSizeRatio ; }
// #endif // SM_USE_NEWTOL

  public:
  
  // simple data assignments                                                                                                                            
  void SetAttributeCallbacks(SmObject * pCallback)   { m_pAttributeCallbacks = pCallback; }
  void SetGlobalUserData(void * pGlobalUserData)     { m_pGlobalUserData     = pGlobalUserData; }
  void SetDoingBoolean(SmBoolean bDoingBoolean)      { m_bDoingBoolean       = bDoingBoolean; }
  void SetUserNotifyCallback(SmNotifyCallback *pUCB) { m_pUserNotifyCallback = pUCB; }
  void SetUserCallback(SmNotifyCallback *pUCB)       { SetUserNotifyCallback(pUCB) ; }
  void SetSysNotifyCallback(SmNotifyCallback *pSCB)  { SM_ASSERT_MSG(m_pSysNotifyCallBack == NULL || pSCB == NULL, 
                                                                     _T("SmContext::SetSysNotifyCallback error: not set up for nested loadings of the callback pointer")) ; 
                                                       m_pSysNotifyCallBack  = pSCB; 
                                                     }
#ifdef SM_USE_GLOBAL_CACHE
  void SetGlobalCache(SmGlobalCache *pGlobalCache) { m_pGlobalCache = pGlobalCache; }
#endif // SM_USE_GLOBAL_CACHE

  // mark management:                 
  //  rule 1  : Mark->Context == SmTopology::GetContext(), marks don't work when MarkContext differs from SmTopologyContext
  //  feature : SmNewMark can coordinate using a mark in two different contexts (good for booleans with two pBrep->Contexts)
  //  Mark use: note: Users don't call SmContext::MarkMethods(). 
  //                  Users build SmNewMarkAndLock objs and call SmTopology::MarkMethods() as:
  //   SmNewMarkAndLock sMark(pObjContext) ;                // eff: Fetch, Lock, and Increment an Unlocked mark
  //   SmNewMarkAndLock sMark(pObjContext1, pObjContext2) ; // eff: Fetch, Lock, and Increment an Unlocked mark in two contexts
  //   SmMarkType       eMarkType = sMark.GetMarkType() ;   // eff: Fetch the mark that was locked
  //
  //   pTopology->Mark(eMarkType) ;                         // eff: set   TopologyObj eMarkType mark. Requires: pObjContext == pTopology->GetContext()
  //   pTopology->UnMark(eMarkType) ;                       // eff: clear TopologyObj eMarkType mark. Requires: pObjContext == pTopology->GetContext()
  //   sMark.NewMark(eMarkType) ;                           // eff: clear all TopologyObj eMarkType marks. Incs eMarkType value.
  //
  //   SmBoolean bMarked = pTopology->IsMarked(eMarkType) ; // rtn: TRUE=pTopologyObj is marked, FALSE=isn't
  //
  //   delete sMark ;                                       // eff: Clear and Unlock eMarkType Mark

  // Context Mark management needed to make SmNewMarkAndLock work.
  SmBoolean        IsMarkLocked  (SmMarkType eMarkType) const ; 
  ULONG            GetCurrentMark(SmMarkType eMarkType) const ;

#ifdef SM_USE_GLOBAL_CACHE
#ifdef SM_DEBUG_CODE
  // retrieve max global cache sizes 
  //   (updated each time an object cache is added or removed from cache queue)
  SmGlobalCacheSize &GetMaxGlobalCacheSizes() const ;
#endif // SM_DEBUG_CODE
#endif

  void *operator new(size_t size);
  void  operator delete(void *ptr) { smos_Free(ptr); ptr = NULL ; }

  void           Dump()                  const;

  const TCHAR * GetTypeString() const { return _T("SmContext") ; }

private:
  friend class     SmNewMarkAndLock ;                                                                                                               
  friend class     SmTopology ;          // only needed for backward compatibility for obsolete    

  // mark management without locking - use SmNewMarkAndLock to access
  ULONG NewMark(SmMarkType eMarkType, SmBoolean bIgnoreLock=FALSE) ; 

} ; // end class SmContext

/*******************************************************************//**
PURPOSE:  Encapsulate Incrementing, Locking, and Unlocking of 
             SmContext marks
    
NOTES: 
  //    To IncrementAndLock and Unlock marks, declare SmNewMarkAndLock object.
  //    The SmNewMarkAndLock constructor increments and locks a mark, 
  //    The SmNewMarkAndLock destructor unlocks the mark when it goes out of scope.

  Typical code that uses marks and mark locking looks like the following
  {
    SmNewMarkAndLock sMarkLock(pContext) ;             // increment and lock any unlocked mark
    SmMarkType eMarkType = sMarkLock.GetMarkType() ;   // fetch the mark that was locked

    . . . code that sets and checks marks of type eMarkType
    pTopology->Mark(), UnMark(), IsMarked(), IsUnMarked()

    // increment the associated mark
    sMarkLock.NewMark() ;                              // increment the mark

    . . . code that sets and checks marks of type eMarkType

  } // close scope containing sMarkLock - its destructor unlocks the mark

************************************************************************/
class SM_EXPORT SmNewMarkAndLock
{
 protected:
  SmContext * m_pContext1 = NULL ;           // pointer to context whose mark is locked
  SmContext * m_pContext2 = NULL ;           // for binary operators like the Booleans that
                                             //  use a mark across two differnt Breps possibly in
                                             //  two different contexts

  SmMarkType  m_eMarkType1 = SM_MT_NOMARK ;  // one of: SM_MT_MARK  
  SmMarkType  m_eMarkType2 = SM_MT_NOMARK ;  //         SM_MT_MARK2 
                                             //         SM_MT_MARK3 
                                             //         SM_MT_MARKIO
                                             //         SM_MT_MARKASSERT
                                             //         SM_MT_NOMARK - uninitialized
 public:

  // // unary constructor - increment and lock any unlocked mark
  // SmNewMarkAndLock(const SmContext *cpContext,        ///< [in] : target context containing desired Mark
  //                  SmBitArray baMask=SM_MT_ALLMARKS)  ///< [in] : orof list of marks to check: 
  //                                                     //         SM_MT_MARK  
  //                                                     //         SM_MT_MARK2 
  //                                                     //         SM_MT_MARK3 
  //                                                     //         SM_MT_MARKIO
  //                                                     //         SM_MT_MARKASSERT
  //                                                     //       SM_MT_ALLMARKS  = use any unlocked mark
  //                                                     //       default:[SM_MT_CODEMARKS]
  //                                                     : m_pContext1((SmContext *)cpContext),
  //                                                       m_pContext2(NULL),
  //                                                       m_eMarkType2(SM_MT_NOMARK)
  //                                                     { m_eMarkType1 = GetUnlockedMark(*cpContext, baMask) ; 
  //                                                       m_pContext1->NewMark(m_eMarkType1) ;
  //                                                       LockMark(m_pContext1, m_eMarkType1) ;
  //                                                     }
                                             
  // unary constructor - increment and lock specified mark - signal error in dbg mode if already locked         
   SmNewMarkAndLock
   (
     const SmContext *cpContext,                        ///< [in] : target context containing desired Mark
     SmBitArray       baMarkBitMask = SM_MT_ALLMARKS    ///< [in] : list of mark bits to check, orof
   )
     //         SM_MT_MARK2
     //         SM_MT_MARK3
     //         SM_MT_MARKIO
     //         SM_MT_MARKASSERT
     //       SM_MT_CODEMARKS = use SM_MT_MARK or SM_MT_MARK2 or SM_MT_MARK3
     //       SM_MT_ALLMARKS  = use any unlocked mark
     //       default:[SM_MT_ALLMARKS]
     : m_pContext1( (SmContext *)cpContext ),
     m_pContext2( NULL ),
     m_eMarkType1( SM_MT_NOMARK ),
     m_eMarkType2( SM_MT_NOMARK )
   {
     m_eMarkType1 = GetUnlockedMark( *cpContext, baMarkBitMask );
     m_pContext1->NewMark( m_eMarkType1 );
     LockMark( m_pContext1, m_eMarkType1 );
   }

  // binary constructor - increment and lock any Mark for binary operators 
  //               using two contexts (which may be same or different)
   SmNewMarkAndLock
   (
     SmContext *pContext1,
     SmContext *pContext2
   )
     : m_pContext1( pContext1 ),
     m_pContext2( pContext2 )
   {
     m_eMarkType1 = GetUnlockedMark( *pContext1, SM_MT_ALLMARKS );
     m_pContext1->NewMark( m_eMarkType1 );
     LockMark( m_pContext1, m_eMarkType1 );

     if(m_pContext1 == m_pContext2)
     {
       m_eMarkType2 = m_eMarkType1;
     }
     else
     {
       m_eMarkType2 = GetUnlockedMark( *pContext2, SM_MT_ALLMARKS );
       m_pContext1->NewMark( m_eMarkType1 );
       LockMark( m_pContext2, m_eMarkType1 );
     }
   }
                                               
  // empty constructor - don't lock any mark yet
   SmNewMarkAndLock()
     : m_pContext1( NULL ),
     m_pContext2( NULL ),
     m_eMarkType1( SM_MT_NOMARK ),
     m_eMarkType2( SM_MT_NOMARK )
   {}

  // destructor                                
   ~SmNewMarkAndLock()
   {
     if(m_pContext1 && m_pContext1->IsMarkLocked( m_eMarkType1 ))
     {
       UnlockMark( m_pContext1, m_eMarkType1 );
     }
     if(m_pContext2 && m_pContext2->IsMarkLocked( m_eMarkType2 ))
     {
       UnlockMark( m_pContext2, m_eMarkType2 );
     }
     m_pContext1 = NULL;
     m_pContext2 = NULL;
     m_eMarkType1 = SM_MT_NOMARK;
     m_eMarkType2 = SM_MT_NOMARK;
   }

  // simple access
  SmMarkType        GetMarkType()              { return(m_eMarkType1) ; }
  SmMarkType        GetMarkType1()             { return(m_eMarkType1) ; }
  SmMarkType        GetMarkType2()             { return(m_eMarkType2) ; }

  SmContext       * GetContext()               { return(m_pContext1) ; }
  SmContext       * GetContext1()              { return(m_pContext1) ; }
  SmContext       * GetContext2()              { return(m_pContext2) ; }

  // increment a (unary or binary) locked mark without an error message
  void              NewMark()
  {
    m_pContext1->NewMark( m_eMarkType1, TRUE );
    if(m_pContext2 && m_pContext1 != m_pContext2)
    {
      m_pContext2->NewMark( m_eMarkType2, TRUE );
    }
  }

  // Set m_pContext1 and lock an unlocked mark but don't increment it
  SmMarkType        SetContext
  (
    SmContext *pContext,                    ///< [in] : context for this mark                            <br>
    SmMarkType eMarkType = SM_MT_ALLMARKS   ///< [in] : list of marks to check for availability, orof    <br>
  )                                         //        :   SM_MT_MARK  
                                            //        :   SM_MT_MARK2 
                                            //        :   SM_MT_MARK3 
                                            //        :   SM_MT_MARKIO
                                            //        :   SM_MT_MARKASSERT
                                            //        : SM_MT_ALLMARKS  = use any unlocked mark
  {
    if(m_pContext1 && m_eMarkType1 != SM_MT_NOMARK)
    {
      SM_DBG_WARN( _T( "SmNewMarkAndLock::SetContext - Mark being unlocked through SetContext() - review code" ) );
      UnlockMark( m_pContext1, m_eMarkType1 );
    }
    m_pContext1 = pContext;
    m_eMarkType1 = GetUnlockedMark( *pContext, eMarkType );
    LockMark( pContext, m_eMarkType1 );
    return(m_eMarkType1);
  }

  SmMarkType        SetContext( SmContext *pContext1, SmContext *pContext2 )
  {
    if(m_pContext1 && m_eMarkType1 != SM_MT_NOMARK)
    {
      UnlockMark( m_pContext1, m_eMarkType1 );
    }
    if(m_pContext1 != m_pContext2
       && m_pContext2 && m_eMarkType2 != SM_MT_NOMARK)
    {
      UnlockMark( m_pContext2, m_eMarkType2 );
    }
    m_pContext1 = pContext1;
    m_pContext2 = pContext2;
    m_eMarkType1 = GetUnlockedMark( *pContext1, SM_MT_ALLMARKS );
    LockMark( pContext1, m_eMarkType1 );
    if(pContext1 != pContext2)
    {
      m_eMarkType2 = GetUnlockedMark( *pContext2, SM_MT_ALLMARKS );
      LockMark( pContext2, m_eMarkType2 );
    }
    else { m_eMarkType2 = m_eMarkType1; }
    return(m_eMarkType1);
  }

  void              Dump() const ;

 private:
  void              LockMark    (SmContext *pContext, SmMarkType eMarkType) ;
  void              UnlockMark  (SmContext *pContext, SmMarkType eMarkType) ;
  SmBoolean         IsLockedMark(SmContext *pContext, SmMarkType eMarkType) ;
                    
  static SmMarkType GetUnlockedMark
  (
    const SmContext & crContext,                ///< [in] : target context containing desired Mark          <br>
    SmBitArray        baMask=SM_MT_ALLMARKS     ///< [in] : list of marks to check, orof                    <br>
                                                ///<         SM_MT_MARK                                     <br>
                                                ///<         SM_MT_MARK2                                    <br>
                                                ///<         SM_MT_MARK3                                    <br>
                                                ///<         SM_MT_MARKIO                                   <br>
                                                ///<         SM_MT_MARKASSERT                               <br>
                                                ///<         SM_MT_ALLMARKS  = use any unlocked mark        <br>
  ) ;  
                                                                   
  

} ; // end class SmNewMarkAndLock

#endif // !__SMCONTEXT_H_


