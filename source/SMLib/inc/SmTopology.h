// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTopology.h
* PURPOSE: Header file for SmTopology class.
**********************************************************************/

#ifndef __SMTOPOLOGY_H__
#define __SMTOPOLOGY_H__

//#pragma warning(disable : 4291)   // no matching operator delete found; // restored for debugging Linux builds

#ifndef __SMAOBJECT_H__
#include <SmAObject.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMGAP_H__
#include <SmGap.h>  // for SmGapArray
#endif

/*******************************************************************//**
PURPOSE: This object is the abstract superclass of all topology
   objects. It contains generic parent and sibling topology graph pointers including
      1. an 'owner' pointer and 
      2. a pair of Next/Last Pointers to support a doubly linked list of topology objects.
   The SmOwningTopology derived class adds to this 
      3. a child pointer.

   It also contains some per SmTopology Object state values including
   marks, indexes, and flags.

NOTES: 

  SMTOPOLOGY OBJECT MARKS:
    Individual SmTopology derived objects are marked by several
    iterative or recursive functions to keep track of processed
    and unprocessed objects.  Several 'global' mark values are stored
    in The SmContext object and the matching 'individual' mark values are stored
    in each SmTopology object.  A function marking and testing objects
    use the following calls:
      SmNewMarkAndLock sMarkLock(pContext) ; // increment and lock an unlocked mark.
                                             // unlocks the mark when sMarkLock goes out of scope.
      SmMarkType eMarkType = sMarkLock.GetMarkType() ;
      SmNewMarkAndLock::NewMark()
      SmTopology::Mark(eMarkType)     sets the individual SmTopology::m_lMark value 
                                      to the current SmContext::m_lMark value.
      SmTopology::UnMark(eMarkType)   clears the current individual SmTopology::m_lMark value
      SmTopology::IsMarked(eMarkType) compares the individual to the global mark value.

  SMTOPOLOGY OBJECT INDEXING
    Individual SmTopology derived objects contain two user index values
    that can be used in a similar fashion to the marks.  They are
    available when compiled with the compile constant, SM_INDEXING.

  SMTOPOLOGY OBJECT FLAGS
    Each SmTopology derived object contains a ULONG value treated
    as a bit array.
  
  NOTE: MARKS AND ATTRIBUTES
    The mark mechanism is intended to be used to keep track of state
    during a single modeling operation.  Use the SmAttribute mechanism
    to store data on a per/object basis that is intended to last for one
    or more editing sessions, needs to be stored on objects derived
    from SmAObject but not SmTopology, or has data values more complicated
    than just a bit.
***********************************************************************/
class SM_EXPORT SmTopology : public SmAObject
{
  friend class SmOwningTopology;
  friend class SmBrep;
  friend class SmRegion;

  protected:
  SmOwningTopology * m_pListOwner;    // Owner of this member and its doubly linked topology list
  SmTopology       * m_pNext;         // Next     pointer in doubly linked topology list
  SmTopology       * m_pLast;         // Previous pointer in doubly linked topology list
 // future change: SmTArray<ULONG>    m_sMarks;        // A single list of Marks to replace the old set marks

  ULONG              m_lMark;         // Used to mark topology during various traversal operations.
                                      //   [not persistent - not written to and read from file]
  ULONG              m_lMark2;        // Second level Mark used when m_lMark is taken
                                      //   [not persistent - not written to and read from file]
  ULONG              m_lMark3;        // Third level Mark used when m_lMark and m_lMark2 are taken
                                      //   [not persistent - not written to and read from file]
  ULONG              m_lMarkIO;       // A Mark for use in Dump, Draw, AssertValid functions
                                      //   [not persistent - not written to and read from file]
  ULONG              m_lMarkAssert;   // A Mark for use in AssertValid functions
                                      //   [not persistent - not written to and read from file]
  ULONG              m_lFlags;        // Flags used in selection and other application specific operations.
                                      //   [persistent - written to and read from file]
#ifdef SM_INDEXING
  ULONG              m_lUserIndex1;   // First User Definable Index, 
                                      //   [persistent - written to and read from file]
  ULONG              m_lUserIndex2;   // Second User Definable Index,  
                                      //   [persistent - written to and read from file]
  void*              m_pUserPtr1;     // Third User Definable Index,  
                                      //   [not persistent - not written to and read from file]
#endif // SM_INDEXING

 SM_NEWTOL_LINE SmBoolean m_bIsSmallTopology ; // used by Face, Edge, Vertex (rarely needed - leave defaulted 99.99% of the time)
                                                // TRUE = Intended small geometry size (Pinhole in Battleship) - gets tighter tolerances
                                                // FALSE= Typical size - gets typical tolerances
                                                // default:[FALSE] 
public:
  // constructor
    SmTopology() : m_pListOwner(NULL), m_pNext(NULL), m_pLast(NULL),
                   m_lMark(0), m_lMark2(0), m_lMark3(0),        m_lMarkIO(0),  m_lMarkAssert(0), m_lFlags(0)
#ifdef SM_INDEXING
               , m_lUserIndex1(0),   m_lUserIndex2(0), m_pUserPtr1(0)
#endif // SM_INDEXING:
SM_NEWTOL_LINE , m_bIsSmallTopology(FALSE)
               { /* future change: m_sMarks.SetAll(0) ; */ }
  // destructor
  virtual ~SmTopology() ;

  // simple data access
  virtual SmBrep    * GetBrep  ()                          const { SE_MSG(SM_ERR,_T("Called pure virtual method")); return NULL ; }     
  virtual SmShell   * GetShell ()                          const { SE_MSG(SM_ERR,_T("Called pure virtual method")); return NULL ; }    
  SmOwningTopology  * GetOwner ()                          const { return m_pListOwner ; }
  virtual SmAObject * GetAOwner()                          const { return((SmAObject *)GetOwner()) ; }
  virtual SmBoolean   IsConnectedTo(const SmTopology *cpConnectTgt) const { SM_REF1(cpConnectTgt) ; SE_MSG(SM_ERR,_T("Called pure virtual method")); return FALSE ; }
          SmBoolean   IsConnectedTo(const SmTArray<SmTopology *> &cpConnectTgts){ SmBoolean bRtn = FALSE ;
                                                                                  for(ULONG ii=0;ii<cpConnectTgts.GetSize() && bRtn==FALSE;ii++) 
                                                                                    { bRtn |= IsConnectedTo(cpConnectTgts[ii]) ; }
                                                                                  return(bRtn) ;
                                                                                }
                                                                            
                                                          
  SmTopology        * GetNext  ()                          const { return m_pNext ; }
  SmTopology        * GetLast  ()                          const { return m_pLast ; }
  void                SetListOwner(SmOwningTopology * pOwner)    { m_pListOwner = pOwner ; }

  int                 TestFlag     (ULONG mask)                  { return(m_lFlags & mask ? 1 : 0) ; }
  ULONG               GetAllFlags  ()                            { return m_lFlags ; }
  void                SetAllFlags  (ULONG lFlags)                { m_lFlags = lFlags ; }
  void                SetFlag      (ULONG mask)                  { m_lFlags|=mask; }
  void                SetFlag      (ULONG mask, SmBoolean bBool) { if (bBool) { SetFlag(mask) ; } else { ClearFlag(mask) ; } }
  void                ClearAllFlags()                            { m_lFlags = 0 ; }
  void                ClearFlag    (ULONG mask)                  { m_lFlags &= ~mask ; }

#ifdef SM_INDEXING
  ULONG  GetUserIndex1()               { return m_lUserIndex1; }
  ULONG  GetUserIndex2()               { return m_lUserIndex2; }
  void * GetUserPtr1  ()               { return m_pUserPtr1; }
  void   SetUserIndex1 (ULONG lIndex1) { m_lUserIndex1 = lIndex1; }
  void   SetUserIndex2 (ULONG lIndex2) { m_lUserIndex2 = lIndex2; }
  void   SetUserPtr1   (void* lPtr1)   { m_pUserPtr1 = lPtr1; }

#else // no SM_INDEXING
  ULONG GetUserIndex1()              { return 0; }
  ULONG GetUserIndex2()              { return 0; }
  void* GetUserPtr1  ()              { return 0; }               
  void SetUserIndex1 (ULONG )        { }                
  void SetUserIndex2 (ULONG )        { }                
  void SetUserPtr1   (void* )        { }
                   
#endif // no SM_INDEXING                                             

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

  // To manage marks on individual SmTopology objects 
  void       Mark        (SmMarkType eMarkType) ;        // eff: set  Obj->Mark value = Obj->Context->Mark value
  void       UnMark      (SmMarkType eMarkType) ;        // eff: set  Obj->Mark value = Obj->Context->Mark value - 1
  ULONG      GetMark     (SmMarkType eMarkType) const ;  // rtn: current mark value for given mark type
  SmBoolean  IsMarked    (SmMarkType eMarkType) const ;  // rtn: TRUE=Obj->Mark value == Obj->Context->Mark value, FALSE=not
  SmBoolean  IsUnMarked  (SmMarkType eMarkType) const ;  // rtn: this->GetMark() == (GetContext()->GetCurrentMark() - 1)
  SmBoolean  IsMarkLocked(SmMarkType eMarkType) const ;  // rtn Obj->Context->IsMarkLocked()

  // :--------------------------------------------------------------------------------------------:
  // :                          UpDim, DownDim, and SameDim Gaps                                  :
  // :----------+--------------------------+---------------------------+--------------------------+            
  // : Topology : UpDim Gap types          : DownDim Gap types         : SameDim Gap types        :      
  // :----------+--------------------------+---------------------------+--------------------------+       
  // :  Vertex  : Vertex/Edge, Vertex/Face : none                      : none                     :       
  // :  Edge    : Edge/Face                : Vertex/Edge               : none                     :       
  // :  Face    : none                     : Vertex/Face, Edge/Face    : none                     :       
  // :  Loop    : none                     : none                      : EdgeEnd/EdgeEnd LoopGaps :       
  // :----------+--------------------------+---------------------------+--------------------------+   
  //        MaxGap = Max( Max(UpDimGap), Max(DownDimGap) )  - does not include LoopGaps
  virtual const SmGap * GetMaxGap3d       (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const { SM_REF1(pOptTol3d) ; if(pOptGapArray) { pOptGapArray->ReSet() ; } return( NULL ) ; }
  virtual const SmGap * GetMaxUpDimGap3d  (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const { SM_REF1(pOptTol3d) ; if(pOptGapArray) { pOptGapArray->ReSet() ; } return( NULL ) ; }
  virtual const SmGap * GetMaxDownDimGap3d(SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const { SM_REF1(pOptTol3d) ; if(pOptGapArray) { pOptGapArray->ReSet() ; } return( NULL ) ; }
  virtual const SmGap * GetMaxSameDimGap3d(SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const { SM_REF1(pOptTol3d) ; if(pOptGapArray) { pOptGapArray->ReSet() ; } return( NULL ) ; }

#ifdef SM_USE_NEWTOL
  // tolerance model: for pinholes in battleships (rarely used) - default:[FALSE] (99.9% of the time - leave it that way)
  SM_NEWTOL_LINE SmBoolean   IsSmallTopology() const                { return m_bIsSmallTopology ; }
  SM_NEWTOL_LINE void        SetIsSmallTopology(SmBoolean bIsSmall) { m_bIsSmallTopology = bIsSmall ; }

  // tolerance model: obsolete old-style compatible - instead use SmTol::GetZoneTol3d(this) ; 
  SM_NEWTOL_LINE SmZoneTol3d GetTolerance() const                   { return SmTol::GetZoneTol3d(this, m_bIsSmallTopology) ; } // GWC: obsolete
  SM_NEWTOL_LINE void        SetTolerance(SmZoneTol3d, SmBoolean)   { /* no action - only for backward compatibility */ ; }

#else // SM_USE_OLDTOL
  // tolerance model: old style
  SM_OLDTOL_LINE virtual SmZoneTol3d GetTolerance() const { return(SM_ZONE_TOL_3D) ; }  // old Tol ZoneTol3d stored on the object or default
  SM_OLDTOL_LINE void                SetTolerance(SmZoneTol3d sNewZoneTol3d, 
                                                  SmBoolean bUpdateOnlyIfLarger=TRUE,
                                                  SmBoolean bCascadeToVertices=TRUE) { SM_REF3(sNewZoneTol3d, bUpdateOnlyIfLarger, bCascadeToVertices) ; }
#endif // SM_USE_OLDTOL

  // utilities
  virtual void Notify(SmNotifyOperation eNotifyOperation,
                      SmObject * pData1,
                      SmObject * pData2,
                      SmObject * pData3) ;

  void *operator new(size_t size, SmObject * pObjectToGetContext);
  void  operator delete(void *ptr) { smos_Free(ptr); ptr = NULL ; }

#ifndef SM_BORLAND
  // void *operator new(size_t size);
  void  operator delete(void *ptr, SmObject *) { smos_Free(ptr); ptr = NULL ; }
#endif // no SM_BORLAND

  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=/fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                          //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  SmBoolean CheckPointerChain();
    
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmTopology,SmAObject,SmTopology_TYPE);

  // Begin Obsolete - replaced by SmNewMarkAndLock objects - kept for backwards compatibility
    ULONG     NewMark(SmMarkType eMarkType = SM_MT_MARK, SmBoolean bIgnoreLock=FALSE) const;
    ULONG     NewMark2()  const { return(NewMark(SM_MT_MARK2)) ; }
    void      Mark2()           { Mark(SM_MT_MARK2) ; }
    SmBoolean IsMarked2() const { return(IsMarked(SM_MT_MARK2)) ; }
  // End Obsolete

} ; // end class SmTopology

#endif // !__SMTOPOLOGY_H__

