// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME: SmTypes.h
* PURPOSE: Type definitions
**********************************************************************/

#ifndef __SMOS_TYPES_H__
#define __SMOS_TYPES_H__

#ifndef _INC_STDIO
#include <stdio.h>
#endif

#ifdef _UNICODE
#include <tchar.h>
#endif

#ifndef __SMOS_CONFIG_H__
#include <SmConfig.h>
#endif

#ifndef __SMOS_STRING_H__
#include <SmString.h>
#endif

#include <iostream>

#ifndef NULL
#define NULL 0
#endif

#define SmStatus    long

#define ALIGN_SIZE(s) (((s) % 8) ? ((s)/8+1)*8 : (s))

#include <string.h>
#include <stdlib.h>

#if defined(_WIN32)
#define SM_EXPORT   __declspec( dllexport )
#define SM_EXPORT_TEMPLATE

typedef int SmBoolean;
typedef int SmBitArray;
// if SmBoolean is changed to unsigned long then SmLinkBoolean can be replaced with SmBoolean
// For some reason this typedef does not work in linux

#elif defined(__GNUC__) && __GNUC__ >= 4
#define SM_EXPORT __attribute__((visibility("default")))
#define SM_EXPORT_TEMPLATE
#define SmBoolean int
#define SmBitArray int
#else
#define SM_EXPORT
#define SM_EXPORT_TEMPLATE
#define SmBoolean int
#define SmBitArray int
#endif

#if !defined(SM_EXPORT)
#define SM_EXPORT
#define SM_EXPORT_TEMPLATE
#endif

//BASETYPES is defined when windef.h is called
#if !defined (BASETYPES)
typedef unsigned long ULONG;
// #define SmBooleanUL unsigned long
#endif //BASETYPES
typedef unsigned long SmBooleanUL;

#define LINE_NUMBER __LINE__
#define SM_TBLOCK_SIZE     1024
#define SM_LARGE_TBLOCK_SIZE 4096
#define SM_TBLOCK_128_SIZE  128
#define SM_TBLOCK_64_SIZE    64 

#ifdef _UNICODE
#define FILE_NAME _T(__FILE__)
#define FUNC_NAME _T(__FUNCTION__)
#define SM_PRINTF(a) _tprintf(a);
#define SM_SPRINTF(a, fmt, ...) _sntprintf(a, SM_TBLOCK_SIZE, fmt, __VA_ARGS__)
#define SM_SNPRINTF(a, size, fmt, ...) _sntprintf(a, size, fmt, __VA_ARGS__)
#define SM_FOPEN   _wfopen
#define SM_FROPEN  _tfreopen
#define SM_FSCANF  _ftscanf
#define SM_SSCANF  _stscanf
#define SM_FPRINTF _ftprintf
#define SM_REMOVE  _wremove
#define SM_STRCPY  _tcscpy
#define SM_STRNCPY _tcsncpy
#define SM_STRCAT  _tcscat
#define SM_STRCHR  _tcschr
#define SM_STRLEN  _tcslen
#define SM_STRCMP  _tcscmp

// notes: smos_sprintf() and smos_snprintf()
// 1. when to use smos_sprintf() vs. smos_snprintf()
//     smos_sprintf() is used when we use the hardcoded buffer size SM_TBLOCK_SIZE
//     smos_snprintf() is used when we use a variable buffer size, buffer size is passed as an argument
// 2. always need a format specifier and an argument list in both smos_sprintf() and smos_snprintf()
//    - smos_sprintf(sBuff, _T("%s"), _T("some comment")) ; // correct
//    - smos_sprintf(sBuff, _T("some comment")) ;           // wrong
// 
//    - smos_snprintf(sBuff, lBuffSize, _T("%s"), _T("some comment")) ; // correct
//    - smos_snprintf(sBuff, lBuffSize, _T("some comment")) ;           // wrong
// 3. concatenation issue: if we attempt to write one buffer into another buffer, we need to either make sure that the
//    buffer is large enough to hold the data or we need to truncate the strings using a precision specifier.
//    Example:
//    TCHAR sBuff[SM_TBLOCK_SIZE];
//    TCHAR sBuff2[SM_TBLOCK_SIZE];
//    TCHAR sBuff3[2 * SM_TBLOCK_SIZE];
//
//    smos_sprintf(sBuff, _T("%s"), _T("some comment"));
//
//    smos_sprintf(sBuff2, _T("%s something else"), sBuff);    // wrong, sBuff2 is not large enough to hold the data (sBuff + " something else")
//    smos_sprintf(sBuff2, _T("%512s something else"), sBuff); // correct, truncate the string sBuff to 512 characters
//    smos_sprintf(sBuff3, _T("%s something else"), sBuff);    // correct, sBuff3 is large enough to hold the data (sBuff + " something else")

template <typename... Args>
void inline smos_sprintf(wchar_t* buff, const wchar_t * format, Args... args)
{
    _sntprintf(buff, (size_t)(SM_TBLOCK_SIZE), format, args...);
}

template <typename... Args>
void inline smos_snprintf(wchar_t* buff, size_t buffsize, const wchar_t * format, Args... args)
{
    _sntprintf(buff, buffsize, format, args...);
}

 // For Borland
 #ifndef TCHAR
  typedef _TCHAR TCHAR;
 #endif

#else  // no _UNICODE
#define FILE_NAME __FILE__
#define FUNC_NAME __FUNCTION__
#define SM_PRINTF(a) { int cnt = printf("%s", a); SM_ASSERT(cnt < SM_TBLOCK_SIZE) ; }
#define SM_SPRINTF(a, fmt, ...) snprintf(a, SM_TBLOCK_SIZE, fmt, __VA_ARGS__)
#define SM_SNPRINTF(a, size, fmt, ...) snprintf(a, size, fmt, __VA_ARGS__)
#define SM_SSCANF    sscanf      // <=== need to check rtn count and make sure it's smaller than SM_TBLOCK_SIZE
#define SM_FOPEN     fopen       //   SM_TBLOCK_SIZE and VS_TBLOCK_SIZE need to be larger, maybe 2048
#define SM_FROPEN    freopen
#define SM_FPRINTF   fprintf
#define SM_FSCANF    fscanf
#define SM_REMOVE    remove
#define SM_STRCPY    strcpy
#define SM_STRNCPY   strncpy
#define SM_STRCAT    strcat
#define SM_STRCHR    strchr
#define SM_STRLEN    strlen
#define SM_STRCMP    strcmp

 typedef char TCHAR;
 #undef _T
 #define _T(a) a

// notes: smos_sprintf() and smos_snprintf()
// 1. when to use smos_sprintf() vs. smos_snprintf()
//     smos_sprintf() is used when we use the hardcoded buffer size SM_TBLOCK_SIZE
//     smos_snprintf() is used when we use a variable buffer size, buffer size is passed as an argument
// 2. always need a format specifier and an argument list in both smos_sprintf() and smos_snprintf()
//    - smos_sprintf(sBuff, _T("%s"), _T("some comment")) ; // correct
//    - smos_sprintf(sBuff, _T("some comment")) ;           // wrong
// 
//    - smos_snprintf(sBuff, lBuffSize, _T("%s"), _T("some comment")) ; // correct
//    - smos_snprintf(sBuff, lBuffSize, _T("some comment")) ;           // wrong
// 3. concatenation issue: if we attempt to write one buffer into another buffer, we need to either make sure that the
//    buffer is large enough to hold the data or we need to truncate the strings using a precision specifier.
//    Example:
//    TCHAR sBuff[SM_TBLOCK_SIZE];
//    TCHAR sBuff2[SM_TBLOCK_SIZE];
//    TCHAR sBuff3[2 * SM_TBLOCK_SIZE];
//
//    smos_sprintf(sBuff, _T("%s"), _T("some comment"));
//
//    smos_sprintf(sBuff2, _T("%s something else"), sBuff);    // wrong, sBuff2 is not large enough to hold the data (sBuff + " something else")
//    smos_sprintf(sBuff2, _T("%512s something else"), sBuff); // correct, truncate the string sBuff to 512 characters
//    smos_sprintf(sBuff3, _T("%s something else"), sBuff);    // correct, sBuff3 is large enough to hold the data (sBuff + " something else")

 template <typename... Args>
 void inline smos_sprintf(char* buff, const char * format, Args... args)
 {
     snprintf(buff, (size_t)(SM_TBLOCK_SIZE), format, args...);
 }

 template <typename... Args>
 void inline smos_snprintf(char* buff, size_t buffsize, const char * format, Args... args)
 {
     snprintf(buff, buffsize, format, args...);
 }
#endif // no _UNICODE

// Narrow (UTF-8) C string -> build-width TCHAR string: UTF-8 -> UTF-16 on
// _UNICODE, narrow copy otherwise.  Replaces the SM_TCONVERT macro.
SM_EXPORT std::basic_string<TCHAR> smos_ToTChar(const char* pNarrowUtf8);

// Build-width TCHAR C string -> narrow (UTF-8) string: the inverse of
// smos_ToTChar.  NULL yields an empty string.
SM_EXPORT std::string smos_FromTChar(const TCHAR* pTChar);

template <typename... Args>
void inline smos_sprintf_char(char* buff, const char * format, Args... args)
{
    snprintf(buff, (size_t)(SM_TBLOCK_SIZE), format, args...);
}

template <typename... Args>
void inline smos_snprintf_char(char* buff, size_t buffsize, const char * format,  Args... args)
{
    snprintf(buff, buffsize, format, args...);
}

// MACRO to echo output to both printf and smos_WriteBuffer for prog_test.txt files
#ifdef _UNICODE
#define MYPRINTF(a) { printf("%ls", a) ; smos_WriteBuffer(a,NULL) ; }
#else
#define MYPRINTF(a) { printf("%s", a) ; smos_WriteBuffer(a,NULL) ; }
#endif

#ifdef _MSC_VER
#define SSCANF sscanf_s
#else
#define SSCANF sscanf
#endif

#ifndef TRUE 
#define TRUE 1
#endif

#ifndef FALSE 
#define FALSE 0
#endif

#ifndef UNSURE   // an uninitialized SmBoolean value
#define UNSURE 2
#endif

/*******************************************************************//**
PURPOSE: a three state 'boolean' value

NOTES: SmTriState class objects should interchange with SmBooleans
       the only difference being the SmTriState can be
       TRUE, FALSE, UNSURE, and UNINIT where UNSURE and UNINIT are the same value
***********************************************************************/
// three state 'binary' enum values
enum SmTriStateType
{
  SM_TS_FALSE  = 0,   // FALSE  = 0 so that SmTriStateType:SM_TS_FALSE  == FALSE  is consistent
  SM_TS_TRUE   = 1,   // TRUE   = 1 so that SmTriStateType:SM_TS_TRUE   == TRUE   is consistent
  SM_TS_UNSURE = 2,   // UNSURE = 2 so that SmTriStateType:SM_TS_UNSURE == UNSURE is consistent
  SM_TS_UNINIT = 3    // UNINIT = 3 
} ; // end enum SmTriStateType

/*******************************************************************//**
PURPOSE: class SmTriState    

NOTES: declare variables as SmTriState and use the SM_TS_FALSE, SM_TS_TRUE,
       SM_TS_UNINIT, and SM_TS_UNSURE enum values to set and check them.
       One may also set and check SmTriState objects with SmBooleans and
       ULONGs.
***********************************************************************/
class SM_EXPORT SmTriState    
{
 public:
  SmTriStateType m_Val ;

 public:
  SmTriState (SmTriStateType eArg) : m_Val(eArg) { }
  SmTriState (SmBoolean bArg)      : m_Val((SmTriStateType)bArg) { }
  SmTriState (ULONG lArg)          : m_Val((SmTriStateType)lArg) { }
  SmTriState & operator=(SmTriStateType eArg) { m_Val = eArg ; return *this ; }
  SmTriState & operator=(SmBoolean bArg)      { m_Val = (SmTriStateType)bArg ; return *this ; }
  SmTriState & operator=(ULONG lArg)          { m_Val = (SmTriStateType)lArg ; return *this ; }
  SmBoolean operator==(SmTriStateType eArg) { return(m_Val == eArg) ; }
  SmBoolean operator==(SmBoolean bArg)      { return(m_Val == (SmTriStateType)bArg) ; }
  SmBoolean operator==(ULONG lArg)          { return(m_Val == (SmTriStateType)lArg) ; }
  operator SmBoolean() const { return m_Val ; }

} ; // end class SmTriState    

/*******************************************************************//**
PURPOSE: SM_TYPE declaration, and 
         SM_TYPE Base class values for SMLib's core modules, and 
         SM_TYPE base class value predicates. 

NOTES: SM_TYPE Base class values increment by 1000.
       All Class objects are given a unique SM_TYPE value of the form:
         CLASS_TYPE = BASE_TYPE + Number ;

       where Number is < 1000, so  all objects grouped together
       into the same module share unique Class SM_TYPE values all
       within 1000 of one another.
***********************************************************************/
#define SM_TYPE long

// Base SM_TYPE Values
#define   OS_BASE_TYPE   11000  /* for SmObject_TYPE, SmObjDelete_TYPE, SmContext_TYPE                 */
#define CONT_BASE_TYPE   12000  /* for SmTArray_TYPE, SmAttribute_TYPE, SmMapPtrToPtr_TYPE, . . .      */
#define CURV_BASE_TYPE   14000  /* for SmCurve_TYPE, SmBSplineCurve_TYPE, SmLine_TYPE, . . .           */
#define SURF_BASE_TYPE   15000  /* for SmSurface_TYPE, SmBSplineSurface_TYPE, SmTorus_TYPE, . . .      */
#define TOPO_BASE_TYPE   16000  /* for SmTopology_TYPE, SmBrep_TYPE, SmFace_TYPE, . . .                */
#define POLY_BASE_TYPE   18000  /* for SmPolyBrep_TYPE, SmPolyEdge_TYPE, . . .                         */
#define SOLV_BASE_TYPE   19000  /* for SmGlobalSolver_TYPE, SmFilletIntersector_TYPE, . . .            */
#define  VOL_BASE_TYPE   22000  /* for SmVolume_TYPE, SmBSplineVolume_TYPE, . . .                      */
#define   UNKNOWN_TYPE   20000  /* unknown type value                                                  */

// Base SM_TYPE predicates
SM_EXPORT inline SmBoolean sm_Is_OS_BASE_TYPE  (SM_TYPE lType) { return( (lType %   OS_BASE_TYPE) < 1000) ; }
SM_EXPORT inline SmBoolean sm_Is_CONT_BASE_TYPE(SM_TYPE lType) { return( (lType % CONT_BASE_TYPE) < 1000) ; }
SM_EXPORT inline SmBoolean sm_Is_CURV_BASE_TYPE(SM_TYPE lType) { return( (lType % CURV_BASE_TYPE) < 1000) ; }
SM_EXPORT inline SmBoolean sm_Is_SURF_BASE_TYPE(SM_TYPE lType) { return( (lType % SURF_BASE_TYPE) < 1000) ; }
SM_EXPORT inline SmBoolean sm_Is_TOPO_BASE_TYPE(SM_TYPE lType) { return( (lType % TOPO_BASE_TYPE) < 1000) ; }
SM_EXPORT inline SmBoolean sm_Is_POLY_BASE_TYPE(SM_TYPE lType) { return( (lType % POLY_BASE_TYPE) < 1000) ; }
SM_EXPORT inline SmBoolean sm_Is_SOLV_BASE_TYPE(SM_TYPE lType) { return( (lType % SOLV_BASE_TYPE) < 1000) ; }
SM_EXPORT inline SmBoolean sm_Is_VOL_BASE_TYPE (SM_TYPE lType) { return( (lType %  VOL_BASE_TYPE) < 1000) ; }

// SM_TYPE Topo/Geometry predicate
SM_EXPORT inline SmBoolean sm_Is_Object_TYPE(SM_TYPE lType) { return (   sm_Is_TOPO_BASE_TYPE(lType)
                                                                      || sm_Is_POLY_BASE_TYPE(lType)
                                                                      || sm_Is_CURV_BASE_TYPE(lType)
                                                                      || sm_Is_SURF_BASE_TYPE(lType)
                                                                      || sm_Is_VOL_BASE_TYPE (lType) ) ;
}

// // GWC - the following are not being used
// #define   ROOT_BASE_TYPE    10000
// #define   SESS_BASE_TYPE    13000
// #define   PVXB_BASE_TYPE    17000
// #define   POPL_BASE_TYPE    19000
// #define    SSL_BASE_TYPE    20000
// #define    PGA_BASE_TYPE    21000
// #define METRIC_BASE_TYPE      150

/*******************************************************************//**
PURPOSE: If you are developing an application and subclassing
   SmObject or one of its subclasses, you should start your type
   definitions using APPLICATION_BASE_TYPE.  You will be guaranteed
   not to conflict with any type definitions inside of our software.
   If you use the SM_COMMON macro in your class definition, it will
   automatically create a GetType and IsKindOf methods that utilize
   the <your object name>_TYPE that is defined using APPLICATION_BASE_TYPE.

NOTES:
***********************************************************************/
#define APPLICATION_BASE_TYPE 1000000


/*******************************************************************//**
PURPOSE: This macro defines standard methods for subClasses of SmObject
            including: GetType(), IsKindOf(), and Dump().

   GetType()      - return lowest derived class type_ID (a virtual function)
   GetClassType() - return type_ID for class            (a static  function)
   IsKindOf()     - return TRUE if type_ID is in object's inheritance tree
   Dump()         - pretty print Object values

NOTES:  The macro takes
   (cl) = the class name (not in quotes),
   (sc) = super class name (not in quotes), and
   (ty) = the type_ID <class name>_TYPE (not in quotes).
   You will need to implement a Dump method in the corresponding source file.

   Usually this macro goes into the 'public:' portion of a class definition.
***********************************************************************/
#define SM_COMMON(cl,sc,ty)                                            \
  virtual SM_TYPE       GetType()           const { return ty; }       \
  virtual const TCHAR * GetTypeString()     const { return _T(#ty) ; } \
  virtual const TCHAR * GetClassString()    const { return _T(#cl) ; } \
  static  SM_TYPE       GetClassType()            { return ty; }       \
  static  const TCHAR * GetClassTypeString()      { return _T(#ty) ; } \
  virtual SmBoolean     IsKindOf(SM_TYPE t) const;                     \
  virtual void          Dump(void)          const;                        

#define SM_COMMON_FOR_EXPORT(cl,sc,ty,export)                                            \
  export virtual SM_TYPE       GetType()           const { return ty; }       \
  export virtual const TCHAR * GetTypeString()     const { return _T(#ty) ; } \
  export virtual const TCHAR * GetClassString()    const { return _T(#cl) ; } \
  export static  SM_TYPE       GetClassType()            { return ty; }       \
  export static  const TCHAR * GetClassTypeString()      { return _T(#ty) ; } \
  export virtual SmBoolean     IsKindOf(SM_TYPE t) const;                     \
  export virtual void          Dump(void)          const;                        

// same macro as above modified so it can be used in a base class with derived classes
#define SM_COMMON_BASE(cl,ty)                                          \
  virtual SM_TYPE       GetType()           const { return ty; }       \
  virtual const TCHAR * GetTypeString()     const { return _T(#ty) ; } \
  virtual const TCHAR * GetClassString()    const { return _T(#cl) ; } \
  static  SM_TYPE       GetClassType()            { return ty; }       \
  static  const TCHAR * GetClassTypeString()      { return _T(#ty) ; } \
  virtual SmBoolean     IsKindOf(SM_TYPE t) const                      \
                          { return ((ty == t) ? TRUE : FALSE); }       \
  virtual void          Dump()              const;                        

// same macro as above modified so it can be used in a base class with no derived classes
#define SM_COMMON_ONLY_BASE(cl,ty)                                    \
  SM_TYPE              GetType()           const { return ty; }       \
  const  TCHAR *       GetTypeString()     const { return _T(#ty) ; } \
  const  TCHAR *       GetClassString()    const { return _T(#cl) ; } \
  static SM_TYPE       GetClassType()            { return ty; }       \
  static const TCHAR * GetClassTypeString()      { return _T(#ty) ; } \
  SmBoolean            IsKindOf(SM_TYPE t) const                      \
                          { return ((ty == t) ? TRUE : FALSE); }      \
  void                 Dump()              const;                        

class SmObject;
class SmObjectDelete;
class SmContext;
class SmCubicBezierSurface;
class SmAssertArray ;
class SmAssertReport ;
enum SmAssertObjDump
{
  SM_NO_OBJDUMP,     // in SM_ASSERT_VALID(), don't call pTarget->Dump(), only call pTarget->AssertValid()
  SM_OBJDUMP_BEFORE, // in SM_ASSERT_VALID(), call pTarget->Dump() before calling pTarget->AssertValid()
  SM_OBJDUMP_AFTER,  // in SM_ASSERT_VALID(), call pTarget->Dump() after  calling pTarget->AssertValid()
  SM_OBJDUMP_ONLY    // in SM_ASSERT_VALID(), only call pTarget->Dump(), skip calling pTarget->AssertValid()
} ;
enum SmAssertStream
{
  SM_STREAM,         // in SM_ASSERT_VALID(), always dump AssertArray report
  SM_NO_STREAM,      // in SM_ASSERT_VALID(), only dump AssertArray report for failed AssertValid checks
  SM_GLOBAL_STREAM   // in SM_ASSERT_VALID(), Let global SM_bAssertValidStream value pick streaming
} ;
enum SmAssertTestLevel // These entries are order dependent - don't change them
{
  SM_LEVEL_0     = 0, // AssertValid makes basic tests but skips expensive geometry checks
  SM_LEVEL_1     = 1, // AssertValid makes all level 0 checks plus geometry checks - potentially expensive computations
  SM_LEVEL_2     = 2, // AssertValid makes all level 0 and 1 checks plus checks known to always be slow
  SM_LEVEL_HEAL  = 3, // AssertValid makes all tests that have an implemented Heal method
  SM_LEVEL_GIVEN = 4, // AssertValid makes only ListEntryIndex tests in the order requested in rTestRequest
} ;                  
enum SmAssertWalking
{
  SM_WALK,           // in SM_ASSERT_VALID(), run AssertValid on target and any topology graph descendants 
  SM_NO_WALK         // in SM_ASSERT_VALID(), run AssertValid only on target
} ;
enum SmAssertReportLevel
{
  SM_RL_HARD_ERRS,  // AssertValid only reports hard errors (soft errors not likely to interrupt operations are ignored).
  SM_RL_ALL_ERRS    // AssertValid reports all errors
} ;

// most classes have one AssertValid function and one array of AssertReport Labels. 
// classes with multiple AssertReport generating functions have multiple arrays of AssertReport Labels. 
// Use these enums in the SM_ASSERT_VALUE_REPORT and SM_ASSERT_BOOLEAN_REPORT macros to select between
//   the multiple arrays.  Use SM_LI_0 for all those classes with just one label list.
enum SmAssertListIndex
{ SM_LIST_0,
  SM_LIST_1,
  SM_LIST_2,
  SM_LIST_3,
  SM_LIST_4,
  SM_LIST_5
} ;

// shorten some names to comply with Visual Studio's 255 character name length limit
// #define SmTArray SmTA

#define SmObject_TYPE       (OS_BASE_TYPE+1)
#define SmObjDelete_TYPE    (OS_BASE_TYPE+3)
#define SmContext_TYPE      (OS_BASE_TYPE+4)

/*******************************************************************//**
PURPOSE: This enum indicates the currently supported notification
operations used with the SmObject::Notify method.

NOTES: The Notify method is used to coordinate the propagation
  of associated information within the model data structrues. 
  
  SmObject::Notify() is a virtual function.  By convention, all derived
  Notify() implementations, after taking the appropriate notification actions, 
  always call the Notify() method on its immediate base class. So a single
  call to an object's Notify() function results in a sequence of 
  Notify() calls to all the object's inherited types.  

  In the Notify() methods each class type has the opportunity to
  update any lists or associated data as needed depending on the
  SmNofiticationOperation value as needed.
  
  Some of the coordinated data movements managed by Notify() include:
    1. managing SmBrep entity lists, 
    2. managing geometry cache construction/destruction/refreshing,
    3. managing attribute propagation.

  Callbacks: Use SmObject::SetCall

   expected calls: caller->Notify(Event, pData1, pData2, pData2)                                    
 
 |       event                | caller      | pData1   | pData2                 | pData3                  |
 +----------------------------+-------------+----------+------------------------+-------------------------+
 | SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                   | AddObj->GeomPtr or NULL |
 | SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| SplitObj | Child1                 | Child2                  |
 | SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SrvObj   | DelObj                 | Brep                    |
 | SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj          | NULL                    | 
 | SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj               | BrepB                   |
 | SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                   | RmObj->GeomPtr or NULL  |
 | SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL           | OldGeom or NULL         |
 | SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner->Brep or NULL | OldOwner or NULL        |
 | SM_NO_CONSTRUCTION         | NewObj      | NewObj   | CopyFromObj or NULL    | NULL                    |
 | SM_NO_COPY                 | FromObj     | ToObj    | ToObj->Owner or NULL   | FromObj->Owner or NULL  |
 | SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj->Owner or NULL | NULL                    |
 | SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj->Owner or NULL | NULL                    |
 | SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                 | SplitObj->Owner or NULL |
 | SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2               | MergeObj->Owner or NULL |
 | SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs     | ThisBrep->MergeReg      | 
 | SM_NO_DESTRUCTION          | DelObj      | DelObj   | NULL                   |  NULL                   |
***********************************************************************/
enum SmNotifyOperation {
    SM_NO_ADD_TO_BREP,           // ex: called when a topology object is added to a brep
                                 //       caller: brep
                                 //         pData1: AddObj
                                 //         pData2: brep
                                 //         pData3: AddObj->GeomPtr or NULL
                                 //     use: SmBrep::Notify() adds obj to cached spatial-decomposition object trees. 
                                 
    SM_NO_SPLIT_IN_BREP,         // ex: called when a topology object is split into 2 children objects in a Brep 
                                 //       caller: brep
                                 //         pData1: SplitObj
                                 //         pData2: Child1  - usually SplitObj reused as Child1
                                 //         pData3: Child2  - usually new Child2
                                 //     use: SmAObject::Notify() ForEvery SplitObj attrib, call pAttrib->Split(pData1, pData2, pData3)  
                                 //     use: SmTopology::Notify() Copy SplitObj mark state to Child1 and Child2
                                 
    SM_NO_MERGE_IN_BREP,         // ex: called when 2 topology objects are merged into 1 within a Brep
                                 //       caller: brep
                                 //         pData1: SurvivingObj
                                 //         pData2: DeleteObj
                                 //         pData3: Brep
                                 //     use: SmBrep::Notify() removes obj from cached spatial-decomposition object trees.
                                 //     use: SmAObject::Notify() ForEvery DeleteObj attrib, call pAttrib->Merge(pData1, pData2, pData1)  
                                 //     use: SmTopology::Notify() Merge AllFlags state to surviving obj
                                 
    SM_NO_TRIM_NO_SPLIT_IN_BREP, // ex: called when a Bndry Edge or Vertex is added to Tgt Face or Edge Obj without splitting it within a Brep
                                 //       caller: brep
                                 //         pData1: TgtObj        - to which the bndry obj is being added, either a Face or an Edge on closed geometry
                                 //         pData2: AddedBndryObj - the bndry obj being added,     either an Edge or a Vertex
                                 //         pData3: Brep
                                 //     note: SM_NO_ADD_TO_BREP should have already been called on the pData2 AddedBndryObj item
                                 
    SM_NO_COINCIDENT,            // ex: called when 2 topology objects in different Breps are found coincident during a Boolean operation
                                 //       caller: brepA of Boolean(brepA, brepB)
                                 //         pData1: BrepAObj
                                 //         pData2: BrepBObj
                                 //         pData3: BrepB
                                 //     use: SmBrep::Notify()
                                 //     use: SmAObject::Notify()  - ForEvery BrepAObj attrib, call pAttrib->Merge(pData1, pData3, caller)
                                 //     use: SmTopology::Notify()
                                 
    SM_NO_RM_FROM_BREP,          // ex: called when a topology object is removed from a brep
                                 //       caller: brep
                                 //         pData1: RmObj
                                 //         pData2: Brep
                                 //         pData3: RmObj->GeomPtr or NULL
                                 
    SM_NO_CHANGE_GEOMETRY,       // ex: called when topology object's geometry object is changed
                                 //       caller: TopoObj whose geomPtr is changing
                                 //         pData1: NewGeom or NULL (SmVertex)
                                 //         pData2: Brep or NULL
                                 //         pData3: OldGeom or NULL
                                 
    SM_NO_CHANGE_OWNER,          // ex: called when geometry object's owner object is changed
                                 //       caller: GeomObj whose OwnerPtr is changing
                                 //         pData1: NewOwner
                                 //         pData2: NewOwner's Brep or NULL
                                 //         pData3: OldOwner or NULL
                                 
    SM_NO_CONSTRUCTION,          // ex: called by constructors (note: avoid virtual method call from constructor, call SmObject::Notify())
                                 //       caller: NewObj
                                 //         pData1: NewObj
                                 //         pData2: CopyFromObj or NULL  (when called from Copy Constructor)
                                 //         pData3: NULL
                                 
    SM_NO_COPY,                  // ex: called when Topology and Geometry objects are copied. 
                                 //       caller: FromObj
                                 //         pData1: ToObj
                                 //         pData2: ToObj's Owner or Brep, or NULL
                                 //         pData3: FromObj's Owner or Brep, or NULL
                                 //     use: SmAObject::Notify() ForEvery FromObj attrib, call pAttrib->Copy(this, pData1)  
                                 //     use: SmTopology::Notify() Copy FromObj mark state to ToObj
                                 
    SM_NO_PRE_EDIT,              // ex: called prior to a geometry or Brep change, commonly used to remove any object caches. 
                                 //       caller: EditObj
                                 //         pData1: EditObj
                                 //         pData2: EditObj's Owner or Brep, or NULL
                                 //         pData3: NULL
                                 //     use: SmAObject::Notify() ForEvery EditObj attrib, call pAttrib->Edit(this)
                                 //     use: SmBrep, SmCurve, SmSurfac, and SmPolyBrep::Notify(), deletes EditObj's ObjectsCache
                                 //     use: SmFace::Notify() sets pFace->pSurface->ObjectCache->m_bFaceWasModified bit
                                 //     use: SmSAGObject::Notify() deletes associated DisplayList
                                 
    SM_NO_POST_EDIT,             // ex: called after changing a geometry or Brep definition, no SmAttribute method call.
                                 //       caller: EditObj
                                 //         pData1: EditObj
                                 //         pData2: EditObj's Owner or Brep, or NULL
                                 //         pData3: NULL
                                 //     use: SmEllipse::Notify() calls m_vPolarConverter.SetUpPolarConversion
                                 //     use: SmPolyBrep::Notify() deletes PolyBrep's ObjectsCache - (SmBrep::Notify() does not do this) 
                                 //    note: since it's rarely used and does nothing to Breps, it's not always paired with SM_NO_PRE_EDIT
                                 
    SM_NO_SPLIT,                 // ex: called when 1 geometry object is split into 2 
                                 //       caller: OrigObj as SplitObj
                                 //         pData1: OrigObj reused as child1
                                 //         pData2: new child2
                                 //         pData3: SplitObj's Owner, or NULL
                                 //     use: SmAObject::Notify() ForEvery SplitObj attrib, call pAttrib->Split(this, pData1, pData2) 
                                 //     use: SmTopology::Notify() copy mark state from SplitObj to child1 and child2
                                 //     note: Child1 == SplitObj when SplitObj is reused.
                                 
    SM_NO_MERGE,                 // ex: called when 2 geometry objects are merged into 1
                                 //       caller: SurvivingObj as MergeObj
                                 //         pData1: SurvivingObj as OrigObj1
                                 //         pData2: DeleteObj    as OrigObj2
                                 //         pData3: MergeObj's Owner, or NULL
                                 //     use: SmAObject::Notify() ForEvery DeleteObj attrib, call pAttrib->Merge(pData1, pData2, this) 
                                 //     use: SmTopology::Notify() copy DeleteObj mark state to SurvivingObj and Merge AllFlag values
                                 
    SM_NO_REG_PROPAGATION,       // ex: only called when a Boolean operation combines an OtherBrep Region with a ThisBrep Region
                                 //       caller: ThisBrep->SurvivingRegion
                                 //         pData1: SmTArray<SmRegion*> list of the ThisBrep->InputRegions
                                 //         pData2: SmTArray<SmRegion*> list of the OtherBrep->InputRegions
                                 //         pData3: ThisBrep->SurvivingRegion
                                 
    SM_NO_DESTRUCTION,           // ex: DeleteObj->Notify(SM_NO_DESTRUCTION, DeleteObj, NULL, NULL ) ;
                                 //     called by destructors, typically used to delete object caches,
                                 //       caller: DeleteObj
                                 //         pData1: DeleteObj
                                 //         pData2: NULL
                                 //         pData3: NULL
                                 //     use: SmBrep, SmCurve, SmSurfac, and SmPolyBrep::Notify(), deletes EditObj's ObjectsCache
                                 //     use: SmAObject::Notify() ForEvery DeleteObj attrib, call pAttrib->Destruction(this) 
                                 //     use: SmSAGObject::Notify() deletes associated DisplayList
                                 
    SM_NO_UNKNOWN                // do-nothing value, for completeness
} ;
#define SM_NOTIFYOPERATIONTYPE_NAME(a) (  ((a) == SM_NO_ADD_TO_BREP          ) ? _T("SM_NO_ADD_TO_BREP          ") \
                                        : ((a) == SM_NO_SPLIT_IN_BREP        ) ? _T("SM_NO_SPLIT_IN_BREP        ") \
                                        : ((a) == SM_NO_MERGE_IN_BREP        ) ? _T("SM_NO_MERGE_IN_BREP        ") \
                                        : ((a) == SM_NO_TRIM_NO_SPLIT_IN_BREP) ? _T("SM_NO_TRIM_NO_SPLIT_IN_BREP") \
                                        : ((a) == SM_NO_COINCIDENT           ) ? _T("SM_NO_COINCIDENT           ") \
                                        : ((a) == SM_NO_RM_FROM_BREP         ) ? _T("SM_NO_RM_FROM_BREP         ") \
                                        : ((a) == SM_NO_CHANGE_GEOMETRY      ) ? _T("SM_NO_CHANGE_GEOMETRY      ") \
                                        : ((a) == SM_NO_CHANGE_OWNER         ) ? _T("SM_NO_CHANGE_OWNER         ") \
                                        : ((a) == SM_NO_CONSTRUCTION         ) ? _T("SM_NO_CONSTRUCTION         ") \
                                        : ((a) == SM_NO_COPY                 ) ? _T("SM_NO_COPY                 ") \
                                        : ((a) == SM_NO_PRE_EDIT             ) ? _T("SM_NO_PRE_EDIT             ") \
                                        : ((a) == SM_NO_POST_EDIT            ) ? _T("SM_NO_POST_EDIT            ") \
                                        : ((a) == SM_NO_SPLIT                ) ? _T("SM_NO_SPLIT                ") \
                                        : ((a) == SM_NO_MERGE                ) ? _T("SM_NO_MERGE                ") \
                                        : ((a) == SM_NO_REG_PROPAGATION      ) ? _T("SM_NO_REG_PROPAGATION      ") \
                                        : ((a) == SM_NO_DESTRUCTION          ) ? _T("SM_NO_DESTRUCTION          ") \
                                        : _T("SM_NO_UNKNOWN        ") )

// macros for Notify common argument values - added to simplify debugging - allows one spot to modify all Notify argument behaviors
#define SM_NO_GET_BREP(ptr)    ((SmObject*)(0))
#define SM_NO_GET_POLYBREP(ptr)((SmObject*)(0))
#define SM_NO_GET_OWNER(ptr)   ((SmObject*)(0))
#define SM_NO_GET_SURFACE(ptr) ((SmObject*)(0))
#define SM_NO_GET_CURVE(ptr)   ((SmObject*)(0))

//      #define SM_NO_GET_BREP(ptr)    ((ptr)->GetBrep())
//      #define SM_NO_GET_POLYBREP(ptr)((ptr)->GetPolyBrep())
//      #define SM_NO_GET_OWNER(ptr)   ((ptr)->GetOwner())
//      #define SM_NO_GET_SURFACE(ptr) ((ptr)->GetSurface())
//      #define SM_NO_GET_CURVE(ptr)   ((ptr)->GetCurve())

// shorten some names to comply with Visual Studio's 255 character name length limit
#define SmOrientType SmOrTy

/*******************************************************************//**
PURPOSE: This enum is used whenever an orientation is asked for.

NOTES: 
***********************************************************************/
enum SmOrientType 
{
  SM_OT_UNKNOWN=0,
  SM_OT_SAME=1,        // Edgeuse:[Ivl=(EdgeIvl.min,EdgeIvl.max)], Loopuse:[CCW OuterLoop], Faceuse:[Norm= FaceSrfNorm]
  SM_OT_OPPOSITE=2,    // Edgeuse:[Ivl=(EdgeIvl.max,EdgeIvl.min)], Loopuse:[CW InnerLoop],  Faceuse:[Norm=-FaceSrfNorm]
                       // note: Possible only on Face Missing Seam: a NoArea_Loop
  SM_OT_UPPERDOMAIN=1, //       SM_OT_SAME,     NoArea_Loop:[LeftHandRuleInside=UpperDomainHalf]
  SM_OT_LOWERDOMAIN=2  //       SM_OT_OPPOSITE, NoArea_lopp:[LeftHandRuleInside=LowerDomainHalf]
};

/*******************************************************************//**
PURPOSE: This enum is used whenever a containment relationship is asked for.

NOTES: Initial use: Loop classification in SmFace::ClassifyLoops()
***********************************************************************/
enum SmContainmentType 
{
  SM_CMT_UNKNOWN,         // uninit value
  SM_CMT_OUTERLOOP,       // Area_Loop not in another, contains all UVPoints 'inside' loop  
  SM_CMT_INNERLOOP,       // Area_Loop in another, excludes all UVPoints 'inside' loop
  SM_CMT_NESTEDLOOP_EVEN, // Area_Loop (invalid) nested in InnerLoop with an even nesting depth (0,2,4) - healed to outer loop
  SM_CMT_NESTEDLOOP_ODD,  // Area_Loop (invalid) nested in InnerLoop with an odd nesting depth (1,3,5) - healed to inner loop
  SM_CMT_BOTLOOP,         // NoArea_Loop (invalid) contains upperdomain UVPoints - AddMisingSeam heals to OuterLoop
  SM_CMT_TOPLOOP,         // NoArea_Loop (invalid) contains lowerdomain UVPoints - AddMisingSeam heals to OuterLoop
  SM_CMT_WIRELOOP,        // A set of connected Edgeuses without a closed portion.
  SM_CMT_ERROR,           // Error value assigned for unexpected cases
} ;

/*******************************************************************//**
PURPOSE: Healer planned actions to fix face seam problems

NOTES: 
***********************************************************************/
enum SmFixSeamPlanType
{
  SM_FSP_UNKNOWN,           // uninit value
  SM_FSP_NO_CHANGES,        // no changes needed - Face has no Seam problems                           
  SM_FSP_MOVE_OUT_OF_FACE,  // Seam moved out of face - fixes CrossedSeam, MissingSeam, and NearMissSeam probs
  SM_FSP_MOVE_IN_FACE,      // Seam moved in face then Face split at seam
                            //                        - fixes NearMissSeam and avoid SliverFace Split problems
  SM_FSP_SPLIT_FACE         // Face split at seam     - fixes CrossedSeam and MissingSeam probs               
} ;  

// #define SM_MS_NO_MOVE        0   // Seam_U and Seam_V not moved - no changes made
// #define SM_MS_OUT_OF_FACE_U  1   // Seam_U moved out of face - NO MORE   LoopEdge/Seam_U XSects
// #define SM_MS_IN_FACE_U      2   // Seam_U moved in Face     - REMAINING LoopEdge/Seam_U XSects - avoid SliverFaces in SplitAtSeam()
// #define SM_MS_OUT_OF_FACE_V  4   // Seam_V moved out of face - NO MORE   LoopEdge/Seam_V XSects
// #define SM_MS_IN_FACE_V      8   // Seam_V moved in Face     - REMAINING LoopEdge/Seam_V XSects - avoid SliverFaces in SplitAtSeam() 

/*******************************************************************//**
PURPOSE: This enum is used to pass the Domain type of a geometric object
         as a domain dimension value.

NOTES: 
***********************************************************************/
enum SmDomainDimType 
{
 SM_DD_POINT     = 0,
 SM_DD_CURVE     = 1,
 SM_DD_SURFACE   = 2,
 SM_DD_VOLUME    = 3,
 SM_DD_UNDEFINED = 4
} ;

/*******************************************************************//**
PURPOSE: This enum is used to specify what sort of projection to use
    for some projection related methods.

NOTES: 
***********************************************************************/
enum SmProjectionType 
{
    SM_PT_PARALLEL,
    SM_PT_PERSPECTIVE,
    SM_PT_ROTATION,
    SM_PT_UNKNOWN
} ;

/*******************************************************************//**
PURPOSE: This enum defines the classification of a point - what does
    the point lie on.

NOTES: 
***********************************************************************/
#define SmPointClassType SmPointClassificationType
enum SmPointClassificationType 
{
    // SmCurveClassification depends on this numerical ordering - don't change
    SM_PC_UNKNOWN = 100,    // Point classification is currently unknown
    SM_PC_REGION  = 101,    // Point lies in a topological region (SmRegion)
    SM_PC_FACE    = 102,    // Point lies on or near a topological face (SmFace)
    SM_PC_EDGE    = 103,    // Point lies on or near a topological edge (SmEdge)
    SM_PC_VERTEX  = 104,    // Point lies on or near a topological vertex (SmVertex)
    SM_PC_POINT   = 105,    // Point lies on or near a geometric point (SmPoint3d)
    SM_PC_CURVE   = 106,    // Point lies on or near a geometric curve (SmCurve) 
    SM_PC_SURFACE = 107,    // Point lies on or near a geometric surface (SmSurface)
    SM_PC_VOLUME  = 108,    // Point lies on, in, or near a geometric volume (SmVolume)
    SM_PC_EDGEUSE = 109,    // Point lies on or near a topological edgeuse (SmEdgeuse)
    SM_PC_NOTHING = 110     // Point is known not to map to a topological or geometric object
                            //  GWC: SM_PC_NOTHING is new and only used temporarily within methods
                            //       unless changed, SM_PC_UNKNOWN is used for both
                            //       intervals not yet classified and intervals known to not
                            //       classify to topology or geometry objects
} ; 

/*******************************************************************//**
PURPOSE: This type defines the containment of a point in any other entity.

NOTES: 
  Subsumes SmNodeClassType and SmPolyContainmentType.
***********************************************************************/
enum SmPointObjectContainmentType 
{
    SM_POC_UNKNOWN,
    SM_POC_INSIDE,
    SM_POC_OUTSIDE,
    SM_POC_ON_BOUNDARY
};

/*******************************************************************//**
PURPOSE: This type defines the set of classifications currently
            detectable by the SmPointSet3d class and its derived
            classes SmPointSequence and SmPointGrid

NOTES: 
***********************************************************************/
enum SmPointSetType 
{
  SM_PST_UNKNOWN,       
  SM_PST_VOID,          // empty point set
  SM_PST_POINTSIZED,    // all points within tol of centroid
  SM_PST_LINEAR,        // all points within tol of a centroid intersecting line
  SM_PST_PLANAR,        // all points within tol of a centroid intersecting plane
  SM_PST_SCATTERED      // points are not within tolerance of a point, line or plane
};


/*******************************************************************//**
PURPOSE: This type defines what happens during tessellation.

NOTES: 
***********************************************************************/
enum SmTessStepTestResultType 
{
  SM_ST_NOT_TESTED,    // uninit value
    SM_ST_ACCEPT_STEP,   // Accept the current tessellation
    SM_ST_CUT_STEP,      // Cut the step of a curve tessellation interval
    SM_ST_INCREASE_STEP  // Increase the step of a curve tessellation interval
};

/*******************************************************************//**
PURPOSE: Allow some functions that use SmTopology::m_lMarks to communicate
 and track topology traversals which mark to use.

NOTES: For debug purposes these enums are used as bit values
  in the bitarray SmContext::m_bMarkLock - make sure these values map to
  distinct bit values, i.e, (1,2,4,8,16, etc.)  
***********************************************************************/
enum SmMarkType
{
  SM_MT_NOMARK    =  0,   // don't use a mark - also uninit value in SmNewMarkAndLock
  SM_MT_MARK      =  1,   // use SmTopology::m_lMark       with SmContext::m_lCurrentMark
  SM_MT_MARK2     =  2,   // use SmTopology::m_lMark2      with SmContext::m_lCurrentMark2
  SM_MT_MARK3     =  4,   // use SmTopology::m_lMark3      with SmContext::m_lCurrentMark3
  SM_MT_MARKIO    =  8,   // use SmTopology::m_lMarkIO     with SmContext::m_lCurrentMarkIO     // for Draw methods
  SM_MT_MARKASSERT= 16,   // use SmTopology::m_lMarkAssert with SmContext::M_lCurrentMarkAssert // for AssertValid methods
  SM_MT_ALLMARKS  = 31    // a combination of all marks
} ;

/*******************************************************************//**
PURPOSE: Reverse the orientation of an SmOrientType object.

NOTES: It is assumed that the type is currently either 
   SM_OT_SAME or SM_OT_OPPOSITE.  If it is SM_OT_UNKNOWN it will change
   it to SM_OT_SAME.
***********************************************************************/
#define SM_REVERSE_ORIENTATION(a) (((a)==SM_OT_SAME) ? SM_OT_OPPOSITE : SM_OT_SAME)

/*******************************************************************//**
PURPOSE: This enum defines the type of an intersection curve type.

NOTES: 
***********************************************************************/
enum SmTsectCurveType 
{
    SM_TC_UNKNOWN,        // Point not yet classified
    SM_TC_TOUCHING,       // Curve is a degenerate point which represents a 
                          // single point where surfaces touch.
                          // or intersection trimmed by boundaries to a single point
    SM_TC_CROSSING,       // Curve represents a crossing intersection where 
                          // surface normals are not parallel.
    SM_TC_TANGENT,        // Curve represents a tangent curve where surfaces touch
                          // along a curve
    SM_TC_COINCIDENT,     // Curve represent the point where the two surfaces
                          // butt up against one another but do not overlap
                          // like patches in a quilt.  The surfaces are coincident, have
                          // a parallel surface normals, and are G1 at this
                          // curve but do not share any common interior surface points.
    SM_TC_NEAR_TANGENT,   // Curve has a relatively small angle of intersection
    SM_TC_REGION_BOUNDARY // Curve bounds a region, within which the surfaces
                          // are coincident.  
};

/*******************************************************************//**
PURPOSE: This enum defines where a domain point on a curve
   falls relative to an interval on that curve

NOTES:  Use one of these to classify a curve domain point,
 two for a surface domain point, and three for a volume domain.
***********************************************************************/
#define SmIntervalPosition SmIntervalPositionType
enum SmIntervalPositionType 
{
    SM_IP_UNINIT,         // not yet classified (SM_IP_UNKNOWN was already used)
    SM_IP_START,          // Falls on start point of interval
    SM_IP_INSIDE,         // Falls between start pont and end point of interval
    SM_IP_END,            // Falls on end point of interval
    SM_IP_OUTSIDE         // Falls outside of this interval
};
        
/*******************************************************************//**
PURPOSE: This enum is used when asking for the continuity of a
   curve or a surface.  It is numbered so that checks can be made
   to see if the continuity is greater than, less than, etc.

NOTES: enum members have to be in order of increasing continuity
***********************************************************************/
enum SmContinuityType 
{
    SM_CT_UNDEFINED     = 0,   // initialization and error values
    SM_CT_DISCONTINUOUS = 1,   // Discontinuous position
    SM_CT_C0            = 2,   // Continuous in position
    SM_CT_G1            = 3,   // Continuous in direction but not magnitude of first deriv.
    SM_CT_G1R           = 4,   // G1-Ruled.
    SM_CT_G1_G2         = 5,   // G1 plus continuous in curvature vector
    SM_CT_G1_G2_G3      = 6,   // G1 plus continuous in curvature vector, plus continuous in G3 vector 
    SM_CT_C1            = 7,   // Continuous in both direction and magnitude of first deriv.
    SM_CT_C1_G2         = 8,   // C1 plus continuous in curvature vector
    SM_CT_C1_G2_G3      = 9,   // C1 plus continuous in curvature vector and G3
    SM_CT_C1_C2         = 10,  // C1 plus continuous in second derivative 
    SM_CT_C1_C2_G3      = 11,  // C1 plus continuous in second derivative and G3
    SM_CT_C1_C2_C3      = 12,  // C1 plus continuous in second derivative and third derivative
    SM_CT_CINFINITY     = 13   // Unlimited continuity
};

// return type string for each Continuity type - (used for dump reporting)
SM_EXPORT inline TCHAR* SmGetContinuityTypeString(SmContinuityType eContinuityType)
{

  TCHAR* pRtString = new TCHAR[32];

  switch(eContinuityType)
   {
     case SM_CT_UNDEFINED     : smos_WStrCpy( pRtString, 32, _T("SM_CT_UNDEFINED")); break;
     case SM_CT_DISCONTINUOUS : smos_WStrCpy( pRtString, 32, _T("SM_CT_DISCONTINUOUS") ); break;
     case SM_CT_C0            : smos_WStrCpy( pRtString, 32, _T("SM_CT_C0") ); break;
     case SM_CT_G1            : smos_WStrCpy( pRtString, 32, _T("SM_CT_G1") ); break;
     case SM_CT_G1R           : smos_WStrCpy( pRtString, 32, _T("SM_CT_G1R") ); break;
     case SM_CT_G1_G2         : smos_WStrCpy( pRtString, 32, _T("SM_CT_G1_G2") ); break;
     case SM_CT_G1_G2_G3      : smos_WStrCpy( pRtString, 32, _T("SM_CT_G1_G2_G3") ); break;
     case SM_CT_C1            : smos_WStrCpy( pRtString, 32, _T("SM_CT_C1") ); break;
     case SM_CT_C1_G2         : smos_WStrCpy( pRtString, 32, _T("SM_CT_C1_G2") ); break;
     case SM_CT_C1_G2_G3      : smos_WStrCpy( pRtString, 32, _T("SM_CT_C1_G2_G3") ); break;
     case SM_CT_C1_C2         : smos_WStrCpy( pRtString, 32, _T("SM_CT_C1_C2") ); break;
     case SM_CT_C1_C2_G3      : smos_WStrCpy( pRtString, 32, _T("SM_CT_C1_C2_G3") ); break;
     case SM_CT_C1_C2_C3      : smos_WStrCpy( pRtString, 32, _T("SM_CT_C1_C2_C3") ); break;
     case SM_CT_CINFINITY     : smos_WStrCpy( pRtString, 32, _T("SM_CT_CINFINITY") );break;
     default :                  smos_WStrCpy( pRtString, 32, _T("UnKnown Case") ); break;
  } 
  return( pRtString );
}

#define SM_IS_G1(a) ((a) >= SM_CT_G1)
#define SM_IS_C1(a) ((a) >= SM_CT_C1)
#define SM_IS_G2(a) ((a) == SM_CT_G1_G2 || (a) == SM_CT_G1_G2_G3 || (a) >= SM_CT_C1_G2)
#define SM_IS_C2(a) ((a) >= SM_CT_C1_C2)
#define SM_IS_G3(a) ((a) == SM_CT_G1_G2_G3 || (a) == SM_CT_C1_G2_G3 || (a) >= SM_CT_C1_C2_G3)
#define SM_IS_C3(a) ((a) >= SM_CT_C1_C2_C3)

/*******************************************************************//**
PURPOSE: This enum defines what happens when trimming a surface.

NOTES: 
***********************************************************************/
enum SmTrimType 
{
  SM_TT_KEEP_POINT,      // Keep the 'side' of UVTrimCurves where the reference point is on
  SM_TT_DELETE_POINT,    // Remove the 'side' of UVTrimCurves where the reference point is on
  SM_TT_SPLIT            // Split the brep along the UVTrimCurves
};

/*******************************************************************//**
PURPOSE: This enum is to determine what sort of database IO is 
   to be done. ( ASCII, Binary, ByteSwap Binary )

NOTES: 
***********************************************************************/
enum SmFileType 
{
    SM_ASCII,    // Database is an ASCII file 
    SM_BINARY,   // Database is a Binary format of some type - the default is
                 // a binary file....or:
    SM_BYTESWAP, // Binary with bytes swapped ( Big/Little Endian )

    SM_UNKNOWN   // uninitialized value
};

/*******************************************************************//**
PURPOSE: This enum is used to control the amount of information
  put out by the SmBrep::Dump() function.

NOTES: 
***********************************************************************/
enum SmBrepDumpType 
{
    SM_BD_BASE_ONLY,  // output Brep summary data only
    SM_BD_MAX_GAPS,   // output Brep Max Edge/UVTrimCurve, Vertex/Edge, and Vertex/Face gaps
    SM_BD_GEOM_TYPES, // output Surface and Curve Type counts
    SM_BD_POINTS,     // output vertex point location and nearest neighbor information
    SM_BD_CURVES,     // output vertex and edge curve descriptions
    SM_BD_SURFACES    // output vertex, edge, and face surface descriptions
} ;

/*******************************************************************//**
PURPOSE: This enum is used to specify which parameter of a surface
    is of interest in various query operations.

NOTES: don't change item order - some methods use the magic numbers
***********************************************************************/
enum SmSurfParamType 
{
  SM_SP_U = 0,   // specifies a constant u isoParameter line, or the u direction as needed // note: NL_UDIR = 1
  SM_SP_V = 1,   // specifies a constant v isoParameter line, or the v direction as needed // note: NL_VDIR = 2
  SM_SP_UNKNOWN,
  SM_SP_BOTH,
  SM_SP_NEITHER,
  SM_SP_UMIN,
  SM_SP_VMIN,
  SM_SP_UMAX,
  SM_SP_VMAX,
  SM_SP_OUT
} ;
#define SM_SURFPARAM_TO_NLDIR(eSrfP)  (  eSrfP == SM_SP_U ? NL_UDIR \
                                       : eSrfP == SM_SP_V ? NL_VDIR \
                                       :                    NL_NO)

/*******************************************************************//**
PURPOSE: This enum specifies the classification values for a specified
  direction from a given SrfPt relative to the regions of a Face.

NOTES: Given a Surface, a Face, a SrfPoint, and a SrfVector classify the
SrfPts in the local neighborhood of SrfPoint in the SrfVector
direction relative to the regions and boundaries of the Face
and the Surface.
***********************************************************************/
enum SmCrvDirOnFaceType   
{ 
  SM_CD_UNINIT,             
  SM_CD_NONE,           
  SM_CD_IN,               // SrfDir at a SrfPt points towards 'in' FacePts as defined by the 'lefthand' rule for Face Loopuses
  SM_CD_ON,               // SrfDir on a FaceBndry points Tangent to that Bndry
  SM_CD_OUT,              // SrfDir at a SrfPt points towards 'Out' FacePts as defined by the 'lefthand' rule for Face Loopuses
  SM_CD_LAMINA,           // SrfDir on a Lamina SrfBndry points out of the Surface
  SM_CD_SEAM,             // SrfDir on a Seam SrfBndry points out of the Surface
  SM_CD_POLE,             // SrfDir on a Pole SrfBndry marked by a Vertex points out of the Surface
  SM_CD_POLE_NO_VERTEX    // SrfDir on a Pole SrfBndry NOT marked by a Vertex points out of the Surface
                            //   gwc note: The SM_CD_POLE_NO_VERTEX value may seem like mixing a topology concept with 
                            //         geometry concepts.  But in this enum all these classifications are
                            //         a mix of geometry and topology concepts.  The SM_CD_POLE_NO_VERTEX
                            //         was created to simplify communications within the healer.  In the end
                            //         SMLib may want to add SM_CD_LAMINA_NO_EDGE and SM_CD_SEAM_NO_EDGE if the
                            //         the healer wants to start tracking that kind of information.
} ;

void DumpCrvDirOnFaceType(const TCHAR      * pOptLabel,       // in : optional Character string label, NULL to ignore
                          SmCrvDirOnFaceType eCrvDirOnFace) ; // in : Target eCrvDirOnFace to pretty print

/*******************************************************************//**
PURPOSE: Interval boundary states.

NOTES: 
***********************************************************************/
enum SmBoundaryType 
{
    SM_BT_UNKNOWN,        // interval not yet classified - interval is probably uninit     
    SM_BT_BOUNDED,        // interval bounded from below and above    
    SM_BT_UNBOUNDED_MIN,  // interval min is unbounded (Interval.Min = -SM_INFINITE_PARAMETER)  
    SM_BT_UNBOUNDED_MAX,  // interval max is unbounded (Interval.Max =  SM_INFINITE_PARAMETER)     
    SM_BT_UNBOUNDED,      // interval unbounded in both directions (Interval = [ -SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER])    
};

/*******************************************************************//**
PURPOSE: The output from CheckCurveSweep indicating when a proposed sweep
  will produce a valid swept surface or why it won't

NOTES: 
***********************************************************************/
enum SmSweepCheckType 
{
  SM_SC_OKAY,               // sweeping this curve will produce a valid swept surface
                              
  SM_SC_ROT_ON_CURVE,       // rotating a curve about a point on the curve creates a cone apex or a 
                            // bow-tie surface with a degenerate surface point on one of its boundaries
                            // where TangentU == +/-TangentV.
                            //    Split the curve at the point and try the same sweep on the pieces.

  SM_SC_SWEEP_ALONG_LENGTH, // The curve has point(s) tangent to the sweep direction, sweeping this curve
                            // will produce a line of degenerate surface points for each such tangency point.
                            //    This sweep can't work, either change the sweep (ex: pick a new sweep direction)
                            //    or the sweep geometry (ex: trim the curve to remove the tangent point) to try again.

  SM_SC_SELF_INTERSECT,     // Sweeping the curve will create a self-intersecting swept surface.  
                            //    It's possible that a shorter sweep amount will work, or 
                            //    Split the curve into segments at every point the curve is tangent to the sweep surfaces
                            //    and sweep the segements to produce a set of valid swept surfaces that intersect one another. 

  SM_SC_UNKNOWN             // The check method failed to run properly so the sweep validity is unknown 
};


/*******************************************************************//**
PURPOSE: This define set is used to specify which sides of a surface
               are singular.
NOTES: 
  define values are set so that they can be or-ed together
  to represent all possible combinations of allowed surface
  edge singularities, For example a Complete sphere would
  have a value = SM_SS_UMIN || SM_SS_UMIN

  In NMTLIb surfaces must be nonSingular (i.e. their 1st derivatives are NonZero)
  everywhere except along the edges of the natural domain of the surface.
  Surfaces are allowed to be pinched at their boundary edges down to a single point.
  These singular edges allow spheres to be represented. A sphere is represented
  by a square domain which is pinched with singular UMin and UMax edges
  in combination with a seam running between the VMin and VMax edges making a 
  north and south pole with one periodic longitudinal boundary.
***********************************************************************/
// SS for SurfaceSingularity - these are not an enum so values can be used as a bit array
#define SM_SS_NONE     0    // Surface has no singularities
#define SM_SS_UMIN     1    // singular on u = UMin bdry [Surf(UMin,v) = const, SurfDV(UMin,v) = 0.0 for all allowed v vals]
#define SM_SS_VMIN     2    // singular on v = VMin bdry [Surf(u,VMin) = const, SurfDU(u,VMin) = 0.0 for all allowed u vals]
#define SM_SS_UMAX     4    // singular on u = UMax bdry [Surf(UMax,v) = const, SurfDV(UMax,v) = 0.0 for all allowed v vals]
#define SM_SS_VMAX     8    // singular on v = VMax bdry [Surf(u,VMax) = const, SurfDU(u,VMax) = 0.0 for all allowed u vals]
#define SM_SS_UNKNOWN 16    // Surface singularity not yet tested

#define SM_SS_UMIN_INDEX  0 // associated index value for SM_SS_UMIN
#define SM_SS_VMIN_INDEX  1 // associated index value for SM_SS_VMIN
#define SM_SS_UMAX_INDEX  2 // associated index value for SM_SS_UMAX
#define SM_SS_VMAX_INDEX  3 // associated index value for SM_SS_VMAX

// for FlatCorners are identified as bitors of the SM_SS values as
// where FlatCorner is a corner of a BSplineSurface whose control points have been positioned so that
//        the Surface TanU3d and TanV3d vectors at the corner point are parallel.  Users try this
//        to force a square surface domain to fill a circular hole.  In SMLib this is an illegal shape
//        because the surface has no local neighborhood at the point, e.g UVTrimCurves ending at the corner 
//        can only have tangents that are parallel to the corner tangent direction.  Since Faces have
//        to support being split by any shaped SurfaceTrimCurve, including those that might end at the 
//        corner with any tangent direction other than the one this corner can support, SMLib has to make
//        the flat corner shape illegal. Also there is no defined surface normal at the corner but that's
//        not a good enough reason by itself to make flatcorners illegal since the normal could be found
//        in the limit as one approaches the corner.
#define SM_FC_NONE        0
#define SM_FC_UMIN_VMIN   1  // SM_SS_UMIN | SM_SS_VMIN
#define SM_FC_UMIN_VMAX   2  // SM_SS_UMIN | SM_SS_VMAX
#define SM_FC_UMAX_VMIN   4  // SM_SS_UMAX | SM_SS_VMIN
#define SM_FC_UMAX_VMAX   8  // SM_SS_UMAX | SM_SS_VMAX
#define SM_FC_UNKNOWN    16  // surface rounded corner not yet tested

#define SM_FC_UMIN_VMIN_INDEX  0 // associated index value for SM_FC_UMIN_VMIN
#define SM_FC_UMIN_VMAX_INDEX  1 // associated index value for SM_FC_UMIN_VMAX
#define SM_FC_UMAX_VMIN_INDEX  2 // associated index value for SM_FC_UMAX_VMIN
#define SM_FC_UMAX_VMAX_INDEX  3 // associated index value for SM_FC_UMAX_VMAX

/*******************************************************************//**
PURPOSE: This enum is used to specify which parameter of a volume
    is of interest in various query operations.

NOTES: Volumes have three spaces: InSpace, OutSpace, and ParamSpace.
       The InSpace and ParamSpace parameters are named.
***********************************************************************/
enum SmVolumeParamType 
{
    SM_VP_X_IN,      // specifies a constant Xin isoParameter surface, or the Xin direction as needed
    SM_VP_Y_IN,      // specifies a constant Yin isoParameter surface, or the Yin direction as needed
    SM_VP_Z_IN,      // specifies a constant Zin isoParameter surface, or the Zin direction as needed
    SM_VP_U,         // specifies a constant u isoParameter surface, or the u direction as needed
    SM_VP_V,         // specifies a constant v isoParameter surface, or the v direction as needed
    SM_VP_W,         // specifies a constant w isoParameter surface, or the w direction as needed
    SM_VP_UNKNOWN,
    SM_VP_ALL,
    SM_VP_NONE
};

// Map InSpace ParamType to ParamSpace ParamType
#define SM_INSPACE_TO_PARAM_TYPE(InSpaceType) ((SmVolumeParamType)(  ((InSpaceType) == SM_VP_X_IN) ? SM_VP_U  \
                                                                   : ((InSpaceType) == SM_VP_Y_IN) ? SM_VP_V  \
                                                                   : ((InSpaceType) == SM_VP_Z_IN) ? SM_VP_W  \
                                                                   :  InSpaceType ))

// Map ParamSpace ParamType to InSpace ParamType 
#define SM_PARAM_TO_INSPACE_TYPE(ParamSpaceType) ((SmVolumeParamType)(  ((ParamSpaceType) == SM_VP_U) ? SM_VP_X_IN  \
                                                                      : ((ParamSpaceType) == SM_VP_V) ? SM_VP_Y_IN  \
                                                                      : ((ParamSpaceType) == SM_VP_W) ? SM_VP_Z_IN  \
                                                                      :  ParamSpaceType ))

/*******************************************************************//**
PURPOSE: This enum is used to specify which pairs of parameters in a volume
    are of interest in various query operations.

NOTES: Volumes have three spaces, InSpace, OutSpace, and ParamSpace.
       The InSpace and ParamSpace parameters are named.
***********************************************************************/
enum SmVolumeParamsType 
{
    SM_VPS_XY_IN,     // specifies a constant Xin/Yin isoParameter curve, Zin varies
    SM_VPS_XZ_IN,     // specifies a constant Xin/Zin isoParameter curve, Yin varies
    SM_VPS_YZ_IN,     // specifies a constant Yin/Zin isoParameter curve, Xin varies
    SM_VPS_UV,        // specifies a constant u/v isoParameter curve, w varies
    SM_VPS_UW,        // specifies a constant u/w isoParameter curve, v varies
    SM_VPS_VW,        // specifies a constant v/w isoParameter curve, u varies
    SM_VPS_UNKNOWN,
    SM_VPS_ALL,
    SM_VPS_NONE
};

// Map InSpace ParamsType to ParamSpace ParamsType
#define SM_INSPACE_TO_PARAMS_TYPE(InSpaceType) ((SmVolumeParamsType)(  ((InSpaceType) == SM_VPS_XY_IN) ? SM_VPS_UV  \
                                                                     : ((InSpaceType) == SM_VPS_XZ_IN) ? SM_VPS_UW  \
                                                                     : ((InSpaceType) == SM_VPS_YZ_IN) ? SM_VPS_VW  \
                                                                     : InSpaceType ))

// Map ParamSpace ParamsType to InSpace ParamsType 
#define SM_PARAMS_TO_INSPACE_TYPE(ParamSpaceType) ((SmVolumeParamsType)(  ((ParamSpaceType) == SM_VPS_UV) ? SM_VPS_XY_IN  \
                                                                        : ((ParamSpaceType) == SM_VPS_UW) ? SM_VPS_XZ_IN  \
                                                                        : ((ParamSpaceType) == SM_VPS_VW) ? SM_VPS_YZ_IN  \
                                                                        :  ParamSpaceType ))

/*******************************************************************//**
PURPOSE: Macro swapping 2 anythings by resorting to operator= 
    (and, depending on compiler, the copy constructor)
    Does not work for type* type arguments, see next macro

NOTES: 
***********************************************************************/
#ifndef SM_SWAP
#define SM_SWAP(S_TYPE,a,b){S_TYPE tmp=(a); (a)=(b); (b)=tmp;}
#endif

/*******************************************************************//**
PURPOSE: Macro swapping 2 anything_stars

NOTES: 
***********************************************************************/
#ifndef SM_SWAP_PTR
#define SM_SWAP_PTR(S_TYPE,a,b) {S_TYPE* tmp =(a); (a)=(b); (b)=tmp;}
#endif

/*******************************************************************//**
PURPOSE: Typedef for pointers to functions of the ErrCallbackFunction type

NOTES: 
***********************************************************************/
typedef void (*SmErrorCallbackFunctionPtr)
  (SmStatus            sError,     // to give users a chance to respond to errors given
   const TCHAR * const file_name,  // the number and location of the error.
   ULONG               line_num, 
   const TCHAR * const message) ;

#endif // !__SMOS_TYPES_H__
