// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmObject.h
* PURPOSE: Header file for SmObject.
**********************************************************************/

#ifndef __SMOBJECT_H__
#define __SMOBJECT_H__

//#pragma warning(disable : 4291)   // no matching operator delete found; // restored to debug Linux builds

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMOS_MEMORY_H__
#include <SmMemory.h>
#endif

#ifndef  __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMOS_MATH_H_
#include <SmMath.h>  // includes SmTol.h
#endif 

#ifdef __SMASSERT_VALID_H__
#include <SmAssertArray.h>
#endif

// forward declarations
// Note: Borland compiler dose not like this statement
//typedef struct _iobuf FILE;

// backward compatibility
#define OnStack SetContext
#define SM_BEEN_THROUGH_NEW 123454321 

class SmThreadLocalStorage ;

// Use the following define to echo all Notify calls to the stderr and the stdout streams
// #define SM_ECHO_NOTIFY 1

// Use the following define to echo all SmTopologyChangeCallback calls to the stderr and the stdout streams
extern ULONG GWC_COMMENT_OUT_THE_NEXT_LINE_THAT_DEFINES_SM_TOPOCHANGE_NOTIFY_BEFORE_RELEASE ;
#define SM_TOPOCHANGE_NOTIFY 1

/*******************************************************************//**
PURPOSE: This is the base level object for many classes.  It provides
   memory management, attribute management, and type management. 

NOTES: Objects which inherit from SmObject must use the
   SM_COMMON macro to declare predefined type methods.

   SmObject overloads new and should remain a base class
***********************************************************************/
class SM_EXPORT SmObject 
{
protected:
  const SmContext * m_cpContext ;     // Gives derived objects access to 'global values.'
                                      //   m_cpContext is not 'owned' by this object.
                                      //   m_cpContext is not deleted when this object is deleted.
                                      //
                                      // m_cpContext is set by overloaded new when allocated on the heap as
                                      //   SmObject *pObject = new (cpContext) SmAnyDerivedFromObjectClass()
                                      //
                                      // m_cpContext is set to NULL when allocated on the stack as
                                      //   ExFunc() 
                                      //     { SmAnyDerivedFromObjectClass sObject ; // m_cpContext is set to NULL 
                                      //     }
                                      //
                                      //   note: some derived classes take a cpContext ptr argument in their constructors.
                                      //         Those constructors will set m_cpContext from the input argument value.
                                        
public:
  // constructor
  SmObject(SmBoolean bCheckBeenThroughNew=TRUE) ;   // bCheckBeenThrough new for internal use only - always set to TRUE

  // copy constructor
  SmObject(const SmObject &rObj) ;

  // destructor
  virtual ~SmObject();

  // simple data access
  const SmContext     * GetContext() const                      { return m_cpContext ; }      
  virtual void          SetContext(const SmContext * cpContext) { m_cpContext = cpContext ; }

  // allow objects to respond to changes, some uses include:
  // 1. managing SmBrep entity lists, 
  // 2. managing geometry cache construction/destruction/refreshing,
  // 3. managing attribute propagation,
  // 4. reporting Notify calls with the user-callback mechanism to application code.
  virtual void Notify                    // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
   (SmNotifyOperation  eNotifyOperation, //       event                | caller      |  pData1  | pData2                | pData3                   
    SmObject         * pData1,           //----------------------------+-------------+----------+-----------------------+--------------------------
    SmObject         * pData2,           // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL             
    SmObject         * pData3) ;         // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                     
                                         // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      
                                         // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                         // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
                                         // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   
                                         // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           
                                         // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          
                                         // SM_NO_CONSTRUCTION         | NewObj      | NewObj   | CopyFromObj or NULL   | NULL
                                         // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   
                                         // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                         // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                         // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL   
                                         // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL   
                                         // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                         // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     

  // overload operator new to take an SmContext object argument
  void *operator new   (size_t size, const SmContext & crContext); 
  void  operator delete(void *ptr)                                 { smos_Free(ptr); ptr = NULL ; }

#ifndef SM_BORLAND
private:
  void *operator new   (size_t size);
public:                
  void *operator new   (size_t size, const SmContext * cpContext); 
  void  operator delete(void *ptr,   const SmContext &)            { smos_Free(ptr); ptr = NULL ; }
  void  operator delete(void *ptr,   const SmContext *)            { smos_Free(ptr); ptr = NULL ; }
#endif // no SM_BORLAND

private: // for class SmThreadLocalStorage only
  // overloaded operator new for class SmThreadLocalStorage - to resolve infinite loop problems at startup 
  friend class SmThreadLocalStorage ;
  void *operator new   (size_t size, const SmThreadLocalStorage *pThreadLocalStorage) ;
  void  operator delete(void *ptr, const SmThreadLocalStorage *pThreadLocalStorage) { SM_REF1(pThreadLocalStorage) ; smos_Free(ptr); ptr = NULL ; }
public:

  // common macros
  SM_COMMON_BASE(SmObject, SmObject_TYPE) ;

  // debug functions
  virtual SmBoolean   AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                  SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                            //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                            //      default:[SM_LEVEL_0] 
                                  SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                  SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                                 const ;
  // obsolete
  //  virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
                      
  virtual void        Dump(const TCHAR *) const;
  virtual void        Dump(SmBoolean bAbbrev) const; // NotUsed: in : bAbbrev 
  virtual void        Dump(ULONG)   const;
  const SmVector3d  & GetObjectColor(SmColorRuleType eColorRule=SM_CR_STANDARD) const;

  static FILE       * OpenFile(const TCHAR *filename, const TCHAR *mode) ;
  static void         CloseFile(FILE *pFile) ;

} ; // end class SmObject

/*******************************************************************//**
PURPOSE: These macros do a pointer casting.

NOTES: SM_CAST_NONNULL_PTR was created to eliminate g++ compile warnings.
The g++ compiler warns whenever any code of the forms
  this       != NULL,
  &rVariable != NULL (where rVariable is a reference to a variable), or
  &sVariable != NULL (where sVariable is a local variable)

because the compiler knows at compiler time that those 3 cases are always
TRUE.

USE:  use SM_CAST_NONNULL_PTR only when you know the ptr being checked can
never be NULL.  That's the this pointer and any dereferencing through local
variables and references to variables.

use SM_CAST_PTR only when you know that the ptr can have a NULL value such
as a pointer argument being passed into a method.

A quick compile under g++ will show you if you selected the correct macro.
***********************************************************************/
#define SM_CAST_NONNULL_PTR(T, p) ((        (p)->IsKindOf(T ## _TYPE) ) ? (T*)(p) : NULL)
#define SM_CAST_PTR(T, p)         (( (p) && (p)->IsKindOf(T ## _TYPE) ) ? (T*)(p) : NULL)

// #define SM_CAST_PTR(T, p)         dynamic_cast<T*> (reinterpret_castp)
// #define SM_CAST_NONNULL_PTR(T, p) dynamic_cast<T*> (p)
// #define SM_CAST_CONST_PTR(T, p)   dynamic_cast<const T*> (p)


/*******************************************************************//**
PURPOSE: This object is a stack based deletion object for objects 
           derived from SmObject and allocated with a call to new().  

NOTES: This class only works on objects derived from SmObject allocated
       with calls to new(). 
       It calls delete() on its contained object when it goes out of scope.

       Use class SmMemDelete(void* pMem) for objects allocated 
         with smos_Calloc() calls rather than new() calls.

EXAMPLE: Automatic Deletion of stack object a when exiting scope
          { SmObject * a = new SmObject();
            SmObjDelete sCleanup(a);
          } // automatically deletes a on exit of scope
       
EXAMPLE: Survival of stack object a when exiting scope
          { SmObject * a = new SmObject();
            SmObjDelete sCleanup(a);       // a is specified as a temporary object
            . . .                          // code changes a's status to a saved object
            sCleanup.Clear();              // allows a to exist outside of current scope
          } // a will now exist outside of this scope
***********************************************************************/
class SmObjDelete 
{
 protected:
  SmObject * m_pObj;              // the object to delete when this object is deleted

 public:
  // constructors, destructor
  SmObjDelete()                    { m_pObj = NULL ; }
  SmObjDelete(SmObject * pObj)     { m_pObj = pObj ; }
 ~SmObjDelete()                    { DeleteObj() ; }
                         
  // methods             
  void Clear     ()                { m_pObj = NULL ;}
  void SetObj    (SmObject * pObj) { m_pObj = pObj ;}
  void DeleteObj ()                { if(m_pObj) { delete m_pObj ; m_pObj = NULL ; } }
  void ReplaceObj(SmObject * pObj) { if(m_pObj) { delete m_pObj ; } m_pObj = pObj ; }

} ; // end class SmObjDelete

/*******************************************************************//**
PURPOSE: This is a templated stack based deletion object.  It will automatically
   delete an object which goes out of scope.  

NOTES: TYPE must be a pointer type

Example:
 {
     MyClassType *a = new MyClassType();
     SmTypedDelete<MyClassType *> sCleanup(a);
 }  // automatically deletes a on exit of scope
***********************************************************************/
template<class TYPE> class SmTypedDelete 
{
 protected:
  TYPE m_pType ;

 public:
  SmTypedDelete()              { m_pType = NULL ; }
  SmTypedDelete(TYPE pType)    { m_pType = pType ; }
 ~SmTypedDelete()              { if (m_pType) { delete m_pType ; m_pType = NULL ; } }

  // methods
  void Clear()                 { m_pType = NULL ; }
  void SetObj(TYPE pType)      { m_pType = pType ; }
  void ReplaceObj(TYPE pType)  { if(m_pType) { delete m_pType ; } m_pType = pType ; }

} ; // end class SmTypedDelete<TYPE>

/*******************************************************************//**
PURPOSE: This is a templated stack based deletion object for arrays.  
   It will automatically delete an object array which goes out of scope.  

NOTES: does not delete the elements of the object array

Example:
 {
     MyClassType *a = new MyClassType[lCount] ;
     SmTypedArrayDelete<MyClassType> sCleanup(a);
 }  // automatically deletes a on exit of scope

***********************************************************************/
template<class TYPE> class SmTypedArrayDelete 
{
 protected:
  TYPE *m_pType ;

 public:
  SmTypedArrayDelete()              { m_pType = NULL ; }
  SmTypedArrayDelete(TYPE *pType)   { m_pType = pType ; }
 ~SmTypedArrayDelete()              { if (m_pType) { delete [] m_pType ; m_pType = NULL ; } }

  // methods
  void Clear()                 { m_pType = NULL ; }
  void SetObj(TYPE *pType)     { m_pType = pType ; }
  void ReplaceObj(TYPE *pType) { if(m_pType) { delete [] m_pType ; } m_pType = pType ; }

} ; // end class SmTypedArrayDelete<TYPE>

/*******************************************************************//**
PURPOSE: A little stack based object to temporarily change a value
   and set it back to the original value on exiting scope.

NOTES: 

Example:
 double b = 5.0;
 {
 SmTemporaryChangeValue<double> sTCV(b,25.0);  // Sets b to 25.0
 ....
 } // Exiting scope automatically changes b back to 5.0
***********************************************************************/
template<class TYPE>
class SmTemporaryChangeValue 
{
  private:
    TYPE    * m_pOriginalPtr = nullptr; // place New and Old values are written to on construction and destruction
    TYPE      m_vOldValue;              // value restored when this object goes out of scope
    TYPE      m_vNewValue;              // value used while this object is in scope
    SmBoolean m_bEnabled = FALSE;

  public:
    SmTemporaryChangeValue()                       : m_vOldValue(0), m_vNewValue(0)
                                                   { } //FS: empty to use set method
    SmTemporaryChangeValue(TYPE & rOriginalValue,  
                           TYPE   sNewValue)       { Set(rOriginalValue, sNewValue) ; }
    void Clear()                                   { m_bEnabled     = FALSE; }
   ~SmTemporaryChangeValue()                       { if ( m_bEnabled && *m_pOriginalPtr != m_vOldValue)
                                                     { *m_pOriginalPtr = m_vOldValue; }
                                                   }
    void Set(TYPE& rOriginalValue, TYPE sNewValue) { m_pOriginalPtr  = &rOriginalValue ;
                                                     m_vOldValue     = rOriginalValue;
                                                     m_vNewValue     = sNewValue; 
                                                     m_bEnabled      = TRUE;
                                                     if (rOriginalValue != sNewValue) { rOriginalValue  = sNewValue ; }
                                                   }

    // commonly used by Debug functions when running trial functions that require
    //  the permanent value instead of the temporary values
		  void UseOrigValue()                            { *m_pOriginalPtr = m_vOldValue; }
		  void UseTempValue()                            { *m_pOriginalPtr = m_vNewValue; }
    void SetNewValue(TYPE sNewValue)               { m_vNewValue     = sNewValue ; }

} ; // end class SmTemporaryChangeValue

#endif // !__SMOBJECT_H__


