// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmAssertArray.h
* PURPOSE: Header file for SmAssertReport and SmAssertArray classes. 
**********************************************************************/

// contains: The AssertValid mechanism including
// 1. SmAssertType        enum of Type labels for each AssertReport
// 2. SmAssertReportLabel struct used to hold static strings that load into AssertReports
// 3. extern declarations of all SmAssertReportLabel arrays defined with each Object::AssertValid() method
// 4. SM_GetAssertLabelList() function that maps Object Types to appropriate SmAssertReportLable Arrays
// 5. Macros used in AssertValid() methods
//      SM_ASSERT_BOOLEAN_REPORT                // boolean predicate without tolerance macro used in AssertValid methods
//        (helper) SM_ASSERT_OBJ_BOOLEAN_REPORT // boolean predicate without tolerance macro used in non-AssertValid methods
//      SM_ASSERT_VALUE_REPORT                  // boolean predicate with tolerance macros used in AssertValid methods
//        (helper) SM_ASSERT_OBJ_VALUE_REPORT   // boolean predicate with tolerance macros used in non-AssertValid methods
//      SM_ASSERT_PAIR_REPORT                   // boolean predicate between two objects with tolerance macro used in AssertValid methods
// 6. SmAssertOptions    Class - RunTime Assert Option List with default values
// 7. SmAssertReport     Class - One AssertValid Test failure report    
// 8. SmAssertArray      Class - An array of SmAssertReports   

#ifndef __SMASSERT_VALID_H__
#define __SMASSERT_VALID_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMMAPTYPETOTYPE_H__
#include <SmMapTypeToType.h>
#endif

// forward declares
class SmAssertReport ;
class SmAssertArray ;

/*******************************************************************//**
PURPOSE: MACRO interface to the AssertValid mechanisms

         SM_ASSERT_VALID macros to standardize access to
           sm_AssertValid() that runs pObj->AssertValid(). 
         In SM_DEBUG_CODE mode return pObj->AssertValid() else return TRUE

NOTES: 
 1. TO DEBUG SM_ASSERT_VALID() calls: put break point in 
      SmAssertReport() constructor and/or sm_AssertValid().

 2. The SM_ASSERT_VALID_ARGS() argument features include:
     eDumpObjFlag       - SM_NO_OBJDUMP     = don't call a->Dump()
                          SM_OBJDUMP_BEFORE = call obj->Dump() before obj->AssertValid()  
                          SM_OBJDUMP_AFTER  = call obj->Dump() after  obj->AssertValid()  
                          SM_OBJDUMP_ONLY   = only obj->Dump() skip   obj->AssertValid()
     eStreamFlag        - SM_STREAM        = always dump AssertArray report
                          SM_NO_STREAM     = only dump AssertArray report for failed AssertValid checks
                          SM_GLOBAL_STREAM = use SmGetAssertValidStream() Stream/NoStream value ;
     eLevelFlag         - SM_LEVEL_0    = execute AssertValid Checks labeled as SM_LEVEL_0
                          SM_LEVEL_1    = execute AssertValid Checks labeled as SM_LEVEL_0 or SM_LEVEL_1
                          SM_LEVEL_2    = execute AssertValid Checks labeled as SM_LEVEL_0, SM_LEVEL_1, or SM_LEVEL_2
                          SM_LEVEL_GIVEN= only execute AssertValid test number checks listed in pTestRequests input array  
     lExeClassType      - ClassType of ObjClass of ObjClass::AssertValid() method to execute - used to navigate virtual stacks - only used when Level == SM_LEVEL_GIVEN
     pTestRequests      - SmTArray<ULONG> of test numbers to execute   - only used when Level == SM_LEVEL_GIVEN
     eWalkFlag          - SM_WALK    = if target has descendant topology objects, walk the topo tree calling AssertValid on every descendant
                          SM_NO_WALK = call AssertValid on target only
     bDraw              - TRUE  = Add Graphics for failed AssertReports
                          FALSE = no Graphics

 3. The SM_ASSERT_VALID MACROS drive sm_AssertValid() with various combinations of the following input arguments:

  sm_AssertValid(pObj,          // in : the object to inspect
                 eDumpObjFlag,  // in : SM_OBJDUMP_BEFORE SM_OBJDUMP_AFTER SM_OBJDUMP_ONLY SM_NO_OBJDUMP, default:[SM_NO_OBJDUMP]
                 eStreamFlag,   // in : SM_STREAM         SM_NO_STREAM     SM_GLOBAL_STREAM,              default:[SM_NO_STREAM]
                 eLevelFlag,    // in : SM_LEVEL_0        SM_LEVEL_1       SM_LEVEL_2,  SM_LEVEL_GIVEN    default:[SM_LEVEL_2=all tests]
                 lExeClassType, // in : ClassType of ObjClass of ObjClass::AssertValid() method to execute - used to navigate virtual stacks - only used when Level == SM_LEVEL_GIVEN
                 pTestRequests, // in : SmTArray<ULONG> of test numbers to execute   - only used when Level == SM_LEVEL_GIVEN
                 eWalkFlag,     // in : SM_WALK           SM_NO_WALK, default:[SM_WALK]   
                 bDraw)         // in : TRUE              FALSE,      default:[FALSE]

HINT - TO DEBUG SM_ASSERT_VALID() calls: put break point in SmAssertReport() constructor and/or sm_AssertValid().
***********************************************************************/
#ifdef SM_DEBUG_CODE

  #define SM_ASSERT_VALID_ARGS(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, lExeClassType, pTReqs, eWalkFlag, bDraw) \
          sm_AssertValid      (pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, lExeClassType, pTReqs, eWalkFlag, bDraw, FILE_NAME, LINE_NUMBER, FUNC_NAME)
  //                                                 SM_ASSERT_VALID_ARGS(pObj, eDumpObjFlag,      eStreamFlag,      eLevelFlag, lExeClassType, pTReqs, eWalkFlag, bDraw)
  #define SM_ASSERT_VALID(a)                         SM_ASSERT_VALID_ARGS((a),  SM_NO_OBJDUMP,     SM_GLOBAL_STREAM, SM_LEVEL_2, 0,             NULL,   SM_WALK,   FALSE)
  #define SM_ASSERT_VALID_NO_STREAM(a)               SM_ASSERT_VALID_ARGS((a),  SM_NO_OBJDUMP,     SM_NO_STREAM,     SM_LEVEL_2, 0,             NULL,   SM_WALK,   FALSE)
  #define SM_ASSERT_VALID_AND_DUMP(a)                SM_ASSERT_VALID_ARGS((a),  SM_OBJDUMP_AFTER,  SM_GLOBAL_STREAM, SM_LEVEL_2, 0,             NULL,   SM_WALK,   TRUE )
  #define SM_ASSERT_VALID_AND_DEBUGME_DUMP(a,DbgMe)  do { if(DbgMe) {SM_ASSERT_VALID_ARGS((a),  SM_OBJDUMP_AFTER,  SM_GLOBAL_STREAM, SM_LEVEL_2, 0, NULL, SM_WALK, TRUE ) ; } \
                                                          else      {SM_ASSERT_VALID_ARGS((a),  SM_NO_OBJDUMP,     SM_GLOBAL_STREAM, SM_LEVEL_2, 0, NULL, SM_WALK, FALSE); } } while(0)
  #define SM_ASSERT_VALID_AND_DRAW(a)                SM_ASSERT_VALID_ARGS((a),  SM_OBJDUMP_BEFORE, SM_GLOBAL_STREAM, SM_LEVEL_2, 0,             NULL,   SM_WALK,   TRUE )
  #define SM_DUMP_AND_ASSERT_VALID(a)                SM_ASSERT_VALID_ARGS((a),  SM_OBJDUMP_BEFORE, SM_GLOBAL_STREAM, SM_LEVEL_2, 0,             NULL,   SM_WALK,   FALSE)
  #define SM_DUMP(a)                                 SM_ASSERT_VALID_ARGS((a),  SM_OBJDUMP_ONLY,   SM_GLOBAL_STREAM, SM_LEVEL_2, 0,             NULL,   SM_WALK,   FALSE)

  // obsolete
  // #define SM_ASSERT_HEAL_TESTS(a, type, bDoHeal) SM_ASSERT_VALID_ARGS((a), SM_NO_OBJDUMP, SM_NO_STREAM, SM_LEVEL_HEAL, type, NULL, SM_NO_WALK, FALSE)
  // #define SM_ASSERT_1_TEST(a,  type, t1)                  sm_AssertTests(a, type, FILE_NAME, LINE_NUMBER, FUNC_NAME, 1, t1)              
  // #define SM_ASSERT_1_TESTS(a, type, t1)                  sm_AssertTests(a, type, FILE_NAME, LINE_NUMBER, FUNC_NAME, 1, t1)              
  // #define SM_ASSERT_2_TESTS(a, type, t1, t2)              sm_AssertTests(a, type, FILE_NAME, LINE_NUMBER, FUNC_NAME, 2, t1, t2)        
  // #define SM_ASSERT_3_TESTS(a, type, t1, t2, t3)          sm_AssertTests(a, type, FILE_NAME, LINE_NUMBER, FUNC_NAME, 3, t1, t2, t3)    
  // #define SM_ASSERT_4_TESTS(a, type, t1, t2, t3, t4)      sm_AssertTests(a, type, FILE_NAME, LINE_NUMBER, FUNC_NAME, 4, t1, t2, t3, t4)
  // #define SM_ASSERT_5_TESTS(a, type, t1, t2, t3, t4, t5)  sm_AssertTests(a, type, FILE_NAME, LINE_NUMBER, FUNC_NAME, 5, t1, t2, t3, t4, t5)

#ifdef SM_DBG2_CODE
  #define SM_ASSERT2_VALID(a)                        SM_ASSERT_VALID(a)           
  #define SM_ASSERT2_VALID_NO_STREAM(a)              SM_ASSERT_VALID_NO_STREAM(a) 
  #define SM_ASSERT2_VALID_AND_DUMP(a)               SM_ASSERT_VALID_AND_DUMP(a)  
  #define SM_ASSERT2_VALID_AND_DEBUGME_DUMP(a,DbgMe) SM_ASSERT_VALID_AND_DEBUGME_DUMP(a,DbgMe)
  #define SM_ASSERT2_VALID_AND_DRAW(a)               SM_ASSERT_VALID_AND_DRAW(a)  
  #define SM_DUMP_AND_ASSERT2_VALID(a)               SM_DUMP_AND_ASSERT_VALID(a)  
  #define SM_DUMP2(a)                                SM_DUMP(a)                   
#else // no SM_DBG2_CODE
  #define SM_ASSERT2_VALID(a)            
  #define SM_ASSERT2_VALID_NO_STREAM(a)  
  #define SM_ASSERT2_VALID_AND_DUMP(a)   
  #define SM_ASSERT2_VALID_AND_DRAW(a)   
  #define SM_ASSERT2_VALID_AND_DEBUGME_DUMP(a,DbgMe) 
  #define SM_DUMP_AND_ASSERT2_VALID(a)   
  #define SM_DUMP2(a)                    
#endif // no SM_DBG2_CODE

#else // no SM_DEBUG_CODE


  #define SM_ASSERT_VALID_ARGS(pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, lExeClassType, pTReqs, eWalkFlag, bDraw, bHeal) \
          sm_AssertValid      (pObj, eDumpObjFlag, eStreamFlag, eLevelFlag, lExeClassType, pTReqs, eWalkFlag, bDraw, bHeal)

  #define SM_ASSERT_VALID(a) 
  #define SM_ASSERT_VALID_NO_STREAM(a)
  #define SM_ASSERT_VALID_AND_DUMP(a) 
  #define SM_ASSERT_VALID_AND_DRAW(a) 
  #define SM_ASSERT_VALID_AND_DEBUGME_DUMP(a,DbgMe)
  #define SM_DUMP_AND_ASSERT_VALID(a) 
  #define SM_DUMP(a)
  // obsolete
  // #define SM_ASSERT_HEAL_TESTS(a, type, bDoHeal)  { a = a; }
  // #define SM_ASSERT_1_TEST(a, t1)                   sm_AssertTests(a, FILE_NAME, LINE_NUMBER, FUNC_NAME, 1, t1)              
  // #define SM_ASSERT_1_TESTS(a, t1)                  sm_AssertTests(a, FILE_NAME, LINE_NUMBER, FUNC_NAME, 1, t1)              
  // #define SM_ASSERT_2_TESTS(a, t1, t2)              sm_AssertTests(a, FILE_NAME, LINE_NUMBER, FUNC_NAME, 2, t1, t2)        
  // #define SM_ASSERT_3_TESTS(a, t1, t2, t3)          sm_AssertTests(a, FILE_NAME, LINE_NUMBER, FUNC_NAME, 3, t1, t2, t3)    
  // #define SM_ASSERT_4_TESTS(a, t1, t2, t3, t4)      sm_AssertTests(a, FILE_NAME, LINE_NUMBER, FUNC_NAME, 4, t1, t2, t3, t4)
  // #define SM_ASSERT_5_TESTS(a, t1, t2, t3, t4, t5)  sm_AssertTests(a, FILE_NAME, LINE_NUMBER, FUNC_NAME, 5, t1, t2, t3, t4, t5)

  #define SM_ASSERT2_VALID(a)            
  #define SM_ASSERT2_VALID_NO_STREAM(a)  
  #define SM_ASSERT2_VALID_AND_DUMP(a)   
  #define SM_ASSERT2_VALID_AND_DRAW(a)   
  // #define SM_ASSERT2_VALID_AND_HEAL(a)   
  #define SM_DUMP_AND_ASSERT2_VALID(a)   
  #define SM_DUMP2(a)                    

#endif // no SM_DEBUG_CODE

/*******************************************************************//*******
// AssertReport Category Types - used to group errors by function 
****************************************************************************/
enum SmAssertType
{
  SM_AT_UNKNOWN,          // an uninitialized value
 // SM_AT_HEALER,           // a report from a Healer function
 // SM_AT_NO_HEAL_YET,      // a report from a Healer function not yet implemented

  SM_AT_ANGLE,            // check that an angle is less than a tolerance
  SM_AT_BOX,              // check box
  SM_AT_CACHE,            // check data consistency
  SM_AT_CLOSED,           // check if closed

  SM_AT_COINCIDENCE,      // check coincidence
  SM_AT_DEGENERATE,       // check degenerate
  SM_AT_DIRECTION,        // check direction
  SM_AT_DISTANCE,         // check distance

  SM_AT_DOMAIN,           // check domain
  SM_AT_FLAG,             // check that flags are set correctly
  SM_AT_GEOMETRIC,        // check geometric property
  SM_AT_TOPOLOGICAL,      // check a topology graph property
  SM_AT_GAP,              // check that a gap is less than a tolerance

  SM_AT_INSIDE,           // check if inside
  SM_AT_KNOTS,            // check number of knots
  SM_AT_LIST,             // check if in list
  SM_AT_MARK,             // check marks

  SM_AT_MINMAX,           // check that min is less than max
  SM_AT_PARAMETERIZATION, // check issues dealing with parameterization
  SM_AT_POINTER,          // check pointer
  SM_AT_POINTS,           // check that points are within tolerance

  SM_AT_RADIUS,           // check that xRadius and yRadius is the same
  SM_AT_SCALE,            // check scale
  SM_AT_SIZE,             // check size
  SM_AT_TYPE,             // check that data types are correct

  SM_AT_VALUES,           // check values
  SM_AT_VECTOR,           // check that a vector has length
  SM_AT_UNIT_VECTOR,      // check that a vector is unit length
  SM_AT_POLE,             // Check issues dealing with poles

  SM_AT_NESTED_TEST,      // Object must pass a nested test
  SM_AT_NESTED_PROPERTY,  // Object must have a nested property

} ; // end enum SmAssertType

// assert report static labeling structure
typedef struct 
{ SmAssertType    m_eAssertType ;      // a category for the message
  TCHAR const   * m_pName ;            // A short identifier string for the message
  TCHAR const   * m_pMessage ;         // A more detailed message string
  TCHAR const   * m_pMessageForFile ;  // A more detailed message string for file to simplify comparing log files
  //  TCHAR const   * m_pHealName ;    // A short identifier string for a heal message
  //  TCHAR const   * m_pHealMessage ; // a Heal message - describing Heal failures and Successes
} SmAssertReportLabel ;

/*******************************************************************//**
PURPOSE: A class to hold the data for one AssertValid test suitable for:
      pretty printing,            // current requirement
      drawing,                    // current requirement
      rerunning the test,         // future requirement
      suggesting fix functions,   // future requirement
      executing fix functions     // future requirement

NOTES:
***********************************************************************/
class SM_EXPORT SmAssertReport
{
public:
  // Report Status
  SmBoolean     m_bOK ;             // TRUE  = test passed
                                    // FALSE = test failed  
  // Report Header  
  const void *  m_pReporter ;       // The this object pointer of the method that generated this report
                                    //  usually equals m_pOwner, but not for all SM_ASSERT_PAIR_REPORTs()                
  long          m_lReportingType ;  // GetType() value of the method that generated this report
  ULONG         m_lListIndex ;      // index of array when type has more than one array of labels
  ULONG         m_lTestIndex ;      // ListIndex into the SmAssertLabel array for this AssertValid method
                                    //  the canonical number for each test in the AssertValid method.
  SmAssertType  m_eAssertType ;     // oneof: SM_AT_ANGLE,      
                                    //        SM_AT_UNIT_VECTOR,
                                    //        SM_AT_BACKPOINTER,
                                    //        SM_AT_GAP, . . .       
                                    
  const TCHAR * m_pName ;           // Name of test used in Dump comments     
  const TCHAR * m_pMessage ;        // Description of test used in Dump comments 
  const TCHAR * m_pMessageForFile ; // Description of test used in Dump comments without values - to simplify log files for compare 
  // const TCHAR * m_pHealName ;       // Name(desc) of Healer Method
  // const TCHAR * m_pHealMessage ;    // Heal Message - only used for failed Heal attempts
#ifdef SM_DEBUG_CODE                
  const TCHAR * m_file_name ;       // filename label for Assert report
  ULONG         m_line_num ;        // linenumber label for Assert report
#endif // SM_DEBUG_CODE             
                                    
  // Test Inputs
  const void  * m_pOwner ;          // Object identified by the AssertValid Test as the 'Owner' of this test
  SM_TYPE       m_lOwnerType ;      //  gwc: a void* because AssertValid runs on all types, some derived from SmObject, others not.
  const TCHAR * m_sOwnerTypeString ;
              
  const void  * m_pOther ;          // Optional 2nd Object for binary tests, e.g. gap tests, ptr tests, etc.
  SM_TYPE       m_lOtherType ;
  const TCHAR * m_sOtherTypeString ;
              
  SmTol3d       m_sTol3d ;          // Tolerance value used for any distance or angle tests
                                    // ToIgnore:[SM_UNDEF_DOUBLE]  

  // Test Results: to make better Dump and Draw methods
  //               for now - results are a union of all AssertReport reporting needs
  //               in the future - if this gets unmanageable the AssertReport class may be broken up into a class hierarchy.
  double        m_dValue ;          // When appropriate the value checked against a tolerance in SM_ASSERT_VALUE_REPORT
  SmPoint2d     m_sOwnerParam ;     // When appropriate curve or Surface params for Point problems. SM_UNDEF_DOUBLE to ignore
  SmPoint2d     m_sOtherParam ;     // When appropriate curve or Surface params for Point problems. SM_UNDEF_DOUBLE to ignore
  double        m_dDist3d ;         // When appropriate a distance value

public:
  // constructor
  SmAssertReport(const void  * pReporter,          // in : this pointer of the AssertValid() method generating this report.             
                 ULONG         lTestIndex,         // in : canonical number of the AssertValid method test                              
                 SM_TYPE       lTestClass,         // in : class type making the call - used to fetch AssertLabelList                   
                 const void  * pOwner,             // in : Owner ptr of the AssertTest generating this report.                          
                 SM_TYPE       lOwnerType,         // in : this->GetType()                                                              
                 const TCHAR * sOwnerTypeString,   // in : this->GetTypeString()                                                        
                 const void  * pOther,             // in : Optional Other object ptr, (good for bad gap reports)                        
                 SM_TYPE       lOtherType,         // in : other->GetType()                                                             
                 const TCHAR * sOtherTypeString,   // in : other->GetTypeString()                                                       
                 SmTol3d       sTol,               // in : Optional sTol value used in this test if appropriate, SM_UNDEF_DOUBLE=ignore.
                 double        dValue,             // in : Optional value checked against sTol, SM_UNDEF_DOUBLE=ignore                  
                 ULONG         lListIndex,         // in : When a type has more than one array of labels, use this                      
                                                   //      index to specify which array is of interest, 0=ignore 
                 const TCHAR * pOptMessage         // in : notNULL = use this as the SmAssertReport dump string
                                                   //      NULL    = use the TextIndex val to fetch dump string from Array                                                   
#ifdef SM_DEBUG_CODE
                 ,const TCHAR * const file_name    // in : filename label for Assert report                                             
                 ,ULONG               line_num     // in : linenumber label for Assert report                                           
#endif // SM_DEBUG_CODE
                ) ;

  // copy constructor
  SmAssertReport(const SmAssertReport & crOriginal) ;

  // static destructor - called by SmAssertArray destructor so that AssertReports can always be deleted in this dll to avoid crashes
  static void      Delete( SmAssertReport *pReport );

  // operators 
  SmAssertReport & operator= (const SmAssertReport &crOther ) ;
  SmBoolean        operator==(const SmAssertReport &crOther ) ;      

  // destructor
 ~SmAssertReport()  { m_pOwner = m_pOther = NULL; }

  // obsolete
  // // Heal AssertFailure Try (when Owner is derived from SmObject)
  // SmBoolean    AssertHeal(SmAssertArray * pAList) 
  // { 
  //   return(   sm_Is_Object_TYPE(m_lReportingType) // when Reporter is derived from SmObject
  //    ? ((SmObject *)m_pReporter)->AssertHeal(*this, pAList) // pass call along
  //    : sm_Is_Object_TYPE(m_lOwnerType) // else if Owner is derived from SmObject
  //      ? ((SmObject *)m_pOwner)->AssertHeal(*this, pAList) // pass call along
  //      : FALSE ) ;                                 // else fail
  // }

  // simple access Report Header
  ULONG        GetListIndex()   const { return m_lListIndex ; }
  ULONG        GetTestIndex()   const { return m_lTestIndex ; }

  // simple access Test Inputs
  const void * GetOwner()       const { return m_pOwner ; }
  SM_TYPE      GetOwnerType()   const { return m_lOwnerType ; }
  SmObject   * GetOwnerObject() const { return( sm_Is_Object_TYPE(m_lOwnerType) ? (SmObject *)m_pOwner : NULL) ;  }
                                 
  const void * GetOther()       const { return m_pOther ; }
  SM_TYPE      GetOtherType()   const { return m_lOtherType ; } 
  SmObject   * GetOtherObject() const { return( sm_Is_Object_TYPE(m_lOtherType) ? (SmObject *)m_pOther : NULL) ;  } 

  // simple access Test Results
  SmTol3d      GetTol3d()       const { return m_sTol3d ; }
  double       GetValue()       const { return m_dValue ; }
  SmPoint2d    GetOwnerParam()  const { return m_sOwnerParam ; }
  SmPoint2d    GetOtherParam()  const { return m_sOtherParam ; }
  double       GetDist3d()      const { return m_dDist3d ; }

  void         SetTol3d     (SmTol3d sTol3d)          { m_sTol3d = sTol3d ; }
  void         SetValue     (double  dValue)          { m_dValue = dValue ; }
  void         SetOwnerParam(SmPoint2d const & rOwnerParam) { m_sOwnerParam = rOwnerParam ; }
  void         SetOtherParam(SmPoint2d const & rOtherParam) { m_sOtherParam = rOtherParam ; }
  void         SetDist3d    (double      dDist3d)     { m_dDist3d     = dDist3d     ; }

  // Predicate
  SmBoolean    IsThisReport
  (
    const void  * pReporter,   // in : this pointer of the AssertValid() method generating this report.  
    ULONG         lTestIndex,  // in : canonical number of the AssertValid method test                   
    SM_TYPE       lTestClass,  // in : class type making the call - used to fetch AssertLabelList        
    const void  * pOwner,      // in : Owner ptr of the AssertTest generating this report.               
    const void  * pOther       // in : Optional Other object ptr, (good for bad gap reports)             
  )
  { 
    return(   pReporter  == m_pReporter
           && lTestIndex == m_lTestIndex
           && lTestClass == m_lReportingType
           && pOwner     == m_pOwner
           && pOther     == m_pOther) ;
  }

  // obsolete
  // // Update changed Obj ptrs - typically changed by an AssertHeal method run on another AssertReport in a common AssertArray
  // void         ReplaceObjPtr(void * pFrom, void * pTo) { if(m_pOwner == pFrom) { m_pOwner = pTo ; }
  //                                                        if(m_pOther == pFrom) { m_pOther = pTo ; }
  //                                                     }
  
  // Pretty print the AssertReport
  void Dump() const ;
  void Dump (size_t lReportTypeMax, size_t lFileNameMax) const ; // align reports in common column

  // Draw a Report center point
  SmDisplayList * Draw(SmBoolean bAddToUIPickList=FALSE, SmGfxArraySet   * pOptGfxSet=NULL) const; 

  // helper methods

  // Default Graphics Center Point based on Owner type and location.
  // rtn: TRUE = found a graphics point, FALSE = didn't
  SmBoolean GetGraphicsPoint                
  (
    SmPoint3d & rGraphicsPoint,   // out: when rtn is TRUE, set with display graphics point, else set to init   
    SM_TYPE   * pOptType=NULL     // in : optional base type for m_pOwner, used for recursion,                  
  ) const ;                       
                                                  
  // return a character string version of the stored m_eAssertType value
  const TCHAR * GetAssertTypeString() const ;   

  // format m_pMessage plus any stored tolerance/value for log output
  void FormatLogMessage(TCHAR* sBuff, ULONG lBuffSize) const ;

} ; // end class SmAssertReport

/*******************************************************************//**
PURPOSE: This class represents an array of problems found with
  a call to any hierarchy of SmClass::AssertValid() methods.

NOTES: AssertValid() methods build these arrays.
   All AssertValid() methods return TRUE  = OK
                                    FALSE = Problem
***********************************************************************/
class SM_EXPORT SmAssertArray : public SmTArray<SmAssertReport *>
{
public:
  SmBoolean m_bLastAdd;  // TRUE = last call to AddUniqueDeleteDuplicate() added a new SmAssertReport.
                          // FALSE= last call found a duplicate and did not add a new SmAssertReport.
                          // State kept so the SM_ASSERT_SET_VALUE() macros can work immediately after
                          // calling the SM_ASSERT_VALUE_REPORT() or SM_ASSERT_BOOLEAN_REPORT() macros.
                          // A kludgy implementation - but it simplifies writing AssertValid() methods.

  SmMapTypeToType<SmObject*, SmObject*> sObjMap; // <OrigObject, NewObject> list used during a heal sequence.
                                                  //
                                                  // An AssertArray is expected to hold a set of AssertReports
                                                  // all generated at one time from a constant set of
                                                  // objects (usually contained within a single topology graph).
                                                  //
  // obsolete
  //                                                // When healing an AssertArray, the set of AssertReports 
  //                                                // are sent one at a time to their object's heal action
  //                                                // functions.  A heal action function might change an
  //                                                // object pointer that might be stored in another 
  //                                                // AssertReport within the same AssertArray creating 
  //                                                // stale pointers. sObjMap is meant to carry the current
  //                                                // list of <OrigObject, NewObject> pairs without keeping all
  //                                                // the change history that comes from running a sequence
  //                                                // of heal actions. Maybe this poor man's object pointer
  //                                                // change history won't be adequate to run the healer.
  //                                                // If that turns out to be the case, we'll redesign the
  //                                                // AssertValid/AssertHeal sequence as needed.
  //                                                //
  //                                                // Before the Healer calls a heal action, it updates 
  //                                                // the AssertReport's object by mapping it through
  //                                                // sObjMap.  The AssertReport's object pointer is
  //                                                // updated as needed and passed to every object
  //                                                // that mapped from the OrigObject pointer. 
  //                                                //  
  //                                                // The sObjMap mapping is only used by the Heal mechanism
  //                                                // but must be logged by every AssertHeal Action 
  //                                                // that removes, splits, merges, or replaces a 
  //                                                // stored object pointer by calling the appropriate
  //                                                //   LogReplaceObject() = Log one Object was replaced by Another.
  //                                                //   LogSplitObject()   = Log one Object was Split into a set of Other objects.
  //                                                //   LogMergeObject()   = Log two Objects were Merged into one.
  //                                                //   LogDeleteObject()  = Log one Object was deleted and is now a stale pointer.
  // end obsolete
  
public:

  // default constructor
  SmAssertArray
  (
    ULONG             nMemorySize = 0,              // in : initial m_pData array size or size of given pOptPtrArray  
    SmAssertReport ** pOptPtrArray = NULL,          // in : optional preallocated array to use for storage            
    ULONG             nArraySize = 0                // in : initial array size, less than or equal to nMemorySize.    
  )
    : SmTArray<SmAssertReport *>( nMemorySize, pOptPtrArray, nArraySize ),
    m_bLastAdd( FALSE )
  {}

  // Copy Constructor
  SmAssertArray( const SmAssertArray &crOther )    // in : object to copy
    : SmTArray<SmAssertReport *>( crOther )
  {
    m_bLastAdd = crOther.m_bLastAdd;
  }

  // Assignment operator
  SmAssertArray & operator=( const SmAssertArray &crOther )
  {
    if(&crOther == this)
      return *this;
    SmTArray<SmAssertReport *>::operator =( crOther );
    m_bLastAdd = crOther.m_bLastAdd;
    return *this;
  }

  // Equality operator
  SmBoolean operator==( const SmAssertArray &crOther )
  {
    ULONG ii;
    for(ii = 0; ii < GetSize(); ii++)
      if(!(m_pData[ii] == crOther.m_pData[ii])) { return FALSE; }
    return(TRUE);
  }

  // Destructor - deletes all contained SmAssertReports
  virtual ~SmAssertArray();

  // inherited methods
  // SmTArray<SmAssertReport*>::Add()
  // SmTArray<SmAssertReport*>::GetAt()
  // SmTArray<SmAssertReport*>::operator[]
  // SmTArray<SmAssertReport*>::RemoveAll()
  // SmTArray<SmAssertReport*>::FindElement()
  // SmTArray<SmAssertReport*>::GetSize()
  // etc . . .

  // access as Base Class SmTArray<SmAssertReport *>
  SmTArray<SmAssertReport *> & GetReports() { return *this; }

  // Spcialized methods to support SM_ASSERT_VALID() MACRO interface
  SmBoolean       AddUniqueDeleteDuplicate( SmAssertReport *pNewElement );         // side effect: set m_lLastIndx
  ULONG           GetLastAdd() { return(m_bLastAdd); }
  void            SetLastAdd( SmBoolean bLastAdd ) { m_bLastAdd = bLastAdd; }
  SmBoolean       Contains( const SmAssertReport & crAssertReport );
  SmBoolean       FindReport
  (
    const void  * pReporter,              // in : this pointer of the AssertValid() method generating this report.                
    ULONG         lTestIndex,             // in : canonical number of the AssertValid method test                                 
    SM_TYPE       lTestClass,             // in : class type making the call - used to fetch AssertLabelList                      
    const void  * pOwner,                 // in : Owner ptr of the AssertTest generating this report.                             
    const void  * pOther = NULL,          // in : Optional Other object ptr, (good for bad gap reports)                           
    ULONG       * pOptFoundIndex = NULL   // out: Optional FoundIndx, When Rtn==TRUE index of found report in this SmAssertArray  
  ) const;

  // obsolete
  // // pass all SmAssertReports to their heal methods - rtn TRUE when all are healed, else return FALSE
  // SmBoolean       Heal();

  // Map one OrigObject to current set of NewObjects. Used by Heal(): Heal passes AssertReports orig generated for pKey onto the rValues[ii] objects
  void MapObject( SmObject * pKey, SmTArray<SmObject*> & rValues ) { sObjMap.GetValuesFor( pKey, rValues ); }

  // obsolete
  // 
  //  // Heal Action Object Mapping Logging - Called by Heal Action Functions.
  //  // Log one Object was replaced by Another. Called by Heal Action Functions.
  //  void LogReplaceObject
  //  (
  //    SmObject * pDelObj,             // eff: add <pDelObj, pNewObj> to sObjMap List             
  //    SmObject * pNewObj              //       changes all <pKey, pDelObj> pairs to <pKey, pNewObj>
  //  );
  //  
  //  // Log one Object was Split into a set of Other objects. Called by Heal Action Functions.
  //  void LogSplitObject
  //  ( 
  //    SmObject * pParent,              // eff: add <pParent, rChildren[ii]> pairs to sObjMap list             
  //    SmTArray<SmObject*> & rChildren  //        replace all <pKey, pParent> pairs with <pKey, rChildren[ii]> pairs
  //  );
  //  
  //  // Log two Objects were Merged into one. Called by Heal Action Functions.
  //  void LogMergeObject
  //  ( 
  //    SmObject * pParent1,             // eff: adds <pParent1, pCheck> and <pParent2, pChild> to sObjMap list     
  //    SmObject * pParent2,             //        changes all <pKey, pParent1> pairs to <pKey, pChild>             
  //    SmObject * pChild                //                all <pKey, pParent2> pairs to <pKey, pChild>
  //  );
  //  
  //  // Log one Object was deleted and is now a stale pointer. Called by Heal Action Functions.
  //  void LogDeleteObject( SmObject * pDelObj );              // eff: adds <pDelObj, NULL> to sObjMap List              
  //                                                           //        changes all <pKey, pDelObj> pairs to <pKey, NULL>
  // end obsolete
  
// Add AssertReport Graphics to new or open Display List
  SmDisplayList * Draw( SmBoolean bAddToUIPickList = FALSE, SmGfxArraySet * pOptGfxSet = NULL ) const;

  // pretty print AssertArray entries
  void Dump
  (
    SmBoolean           bWriteGoodReport,    // in : TRUE = write report for empty AssertArrays, FALSE=don't  
    const TCHAR * const file_name = NULL,    // in : filename label for AssertArray report                    
    ULONG               line_num = 0,        // in : linenumber label for AssertArray report                  
    const TCHAR * const func_name = NULL,    // in : function label for AssertArray report                    
    const TCHAR * const class_string = NULL  // in : tested obj's class string                                
  ) const;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON( SmAssertArray, SmObject, SmAssertArray_TYPE );

}; // end class SmAssertArray

/*******************************************************************//**
PURPOSE: external declaration of all the AssertReportLabels

NOTES: When writing an AssertValid method
  1. Make sure the class has a type definition: #define ClassName_TYPE unique_number
  2. Define an array of SmAssertReportLabel objects for every test
       in the AssertValid function.
     2a. Declare that array in the source file containing the AssertValid() method.
     2b. Declare that array extern in this header file.
     2c. Update function SM_GetAssertLabelList().
  3. Write every AssertValid test as a predicate and place that
      predicate in an SM_LEVEL_N_ASSERT_VALUE_REPORT() macro along with
      necessary identifying documentation.
      
  note: follow the example in SmAxis2Placement::AssertValid() 
***********************************************************************/
// external declaration of all the AssertReportLabels found
//  next to the header of each AssertValid method
extern SmAssertReportLabel sAssertAssembly_list[] ;            
extern SmAssertReportLabel sAssertAssemblyInstance_list[] ;    
extern SmAssertReportLabel sAssertSAGObject_list[] ;
extern SmAssertReportLabel sAssertAObject_list[] ;           
extern SmAssertReportLabel sAssertAttribute_list[] ;           
extern SmAssertReportLabel sAssertAxis2Placement_list[] ; 

extern SmAssertReportLabel sAssertBrep_list[] ;                
extern SmAssertReportLabel sAssertBSplineCurve_list[] ;        
extern SmAssertReportLabel sAssertCurve_list[] ;               
extern SmAssertReportLabel sAssertBSplineSurface_list[] ;      
extern SmAssertReportLabel sAssertSurface_list[] ;             

extern SmAssertReportLabel sAssertCEdge_list[] ;               
extern SmAssertReportLabel sAssertCFace_list[] ;               
extern SmAssertReportLabel sAssertEdge_list[] ;                
extern SmAssertReportLabel sAssertFace_list[] ;                

extern SmAssertReportLabel sAssertPolarConversion_list[] ;  
extern SmAssertReportLabel sAssertPolarIsCurrent_list[] ;   
extern SmAssertReportLabel sAssertCircle_list[] ;              
extern SmAssertReportLabel sAssertEllipse_list[] ;             
extern SmAssertReportLabel sAssertPlane_list[] ;               
extern SmAssertReportLabel sAssertCone_list[] ; 
extern SmAssertReportLabel sAssertHermiteCurve_list[] ;               

extern SmAssertReportLabel sAssertSphere_list[] ;
extern SmAssertReportLabel sAssertCylinder_list[] ;              
extern SmAssertReportLabel sAssertTorus_list[] ;               
extern SmAssertReportLabel sAssertSurfOfRevolution_list[] ;    
extern SmAssertReportLabel sAssertCurveClassification_list[] ;
extern SmAssertReportLabel sAssertCurveInterval_list[] ;
extern SmAssertReportLabel sAssertPointClassification_list[] ;
extern SmAssertReportLabel sAssertCurveBoundedSurface_list[] ;

extern SmAssertReportLabel sAssertTopology_list[] ;            
extern SmAssertReportLabel sAssertOwningTopology_list[] ;      
extern SmAssertReportLabel sAssertEdgeuse_list[] ;             
extern SmAssertReportLabel sAssertExtent1d_list[] ;            

extern SmAssertReportLabel sAssertExtent2d_list[] ;            
extern SmAssertReportLabel sAssertExtent3d_list[] ;            
extern SmAssertReportLabel sAssertFaceuse_list[] ;             
extern SmAssertReportLabel sAssertFilletVertex_list[] ;        
extern SmAssertReportLabel sAssertFilletVertexuse_list[] ;     

extern SmAssertReportLabel sAssertFilletEdgeuse_list[] ;       
extern SmAssertReportLabel sAssertVertexuse_list[] ;           
extern SmAssertReportLabel sAssertVertex_list[] ;              
extern SmAssertReportLabel sAssertFilletEdge_list[] ;          

extern SmAssertReportLabel sAssertFilletBrep_list[] ;          
extern SmAssertReportLabel sAssertLine_list[] ;                
extern SmAssertReportLabel sAssertLoop_list[] ;                
extern SmAssertReportLabel sAssertLoopuse_list[] ;             
extern SmAssertReportLabel sAssertPeriodicExtent1d_list[] ;    

extern SmAssertReportLabel sAssertPolarBox_list[] ;            
extern SmAssertReportLabel sAssertPseudoBox_list[] ;           
extern SmAssertReportLabel sAssertRegion_list[] ;              
extern SmAssertReportLabel sAssertShell_list[] ;               
extern SmAssertReportLabel sAssertPolyFace_list[] ;            

extern SmAssertReportLabel sAssertCPolyFace_list[] ;           
extern SmAssertReportLabel sAssertPolyLoop_list[] ;            
extern SmAssertReportLabel sAssertPolyVertex_list[] ;          
extern SmAssertReportLabel sAssertPolyEdge_list[] ;            
extern SmAssertReportLabel sAssertPolyRegion_list[] ;          

extern SmAssertReportLabel sAssertPolyShell_list[] ;           
extern SmAssertReportLabel sAssertPolyBrep_list[] ;            
extern SmAssertReportLabel sAssertTree_list[] ;                
extern SmAssertReportLabel sAssertHashTable_list[] ;                
extern SmAssertReportLabel sAssertObjsInVoxels_list[] ;                
extern SmAssertReportLabel sAssertSolutionArray_list[] ;       
extern SmAssertReportLabel sAssertDerivSurfDefinition_list[] ; 

extern SmAssertReportLabel sAssertSurfaceCache_list[] ;        
extern SmAssertReportLabel sAssertSurfOfExtrusion_list[] ;     
extern SmAssertReportLabel sAssertDefault_list[] ;          // only used when there is an error

extern SmAssertReportLabel sAssertValidatePointers_list[] ;   // these are the exceptions: nonClass named list of reports 
extern SmAssertReportLabel sAssertValidateTolerances_list[] ;
extern SmAssertReportLabel sAssertCoincidentTopology_list[] ;
extern SmAssertReportLabel sAssertSubTopology_list[] ;

extern SmAssertReportLabel sTrimmingCheckLoopForMiniHourglass_list[] ;  
extern SmAssertReportLabel sTrimmingCheckTessellation_list[] ;          
extern SmAssertReportLabel sTrimmingCheckFace_list[] ;                  
extern SmAssertReportLabel sTrimmingCheckLoop_list[] ;

extern SmAssertReportLabel sAssertCrvOnSurf_list[] ;
extern SmAssertReportLabel sAssertProjectedCurve_list[] ;
extern SmAssertReportLabel sAssertCrvInVolume_list[] ;
extern SmAssertReportLabel sAssertSrfInVolume_list[] ;
extern SmAssertReportLabel sAssertBend_list[] ;
extern SmAssertReportLabel sAssertUnbend_list[] ;

extern SmAssertReportLabel sAssertVolume_list[] ;
extern SmAssertReportLabel sAssertBSplineVolume_list[] ;
extern SmAssertReportLabel sAssertTransform_list[] ;     
extern SmAssertReportLabel sAssertBendVolume_list[] ; 
extern SmAssertReportLabel sAssertUnbendVolume_list[] ;
extern SmAssertReportLabel sAssertTwistVolume_list[] ;
      
extern SmAssertReportLabel sAssertOffsetSurface_list[] ;
extern SmAssertReportLabel sAssertOffsetCurve_list[] ; 

/*******************************************************************//**
PURPOSE: Fetch the AssertReportLabel array from object type and ListIndex

NOTES: everytime an SmAssertReportLabel is declared external above, update this function
***********************************************************************/
SM_EXPORT SmAssertReportLabel * SM_GetAssertLabelList
(
  SM_TYPE lExeClassType,   // in : ClassType of ObjClass of ObjClass::AssertValid() method to execute -
                           // in : used to navigate virtual stacks - only used when Level == SM_LEVEL_GIVEN 
  ULONG   lListIndex = 0   // in : When a type has more than one array of labels, use this    
                           //      index to specify which array is of interest.                
);

// obsolete
// /*******************************************************************//**
// PURPOSE: helper function for SM_ASSERT_NUM_TEST(S) MACRO set to support 
//          sending a variable number of specified tests to the method 
//          sm_AssertValid() while allowing the supported macros to continue
//          evaluating to a boolean TRUE/FALSE value.
// 
// NOTES: Access this method only through the macros
//          SM_ASSERT_1_TEST(a,  type, t1)
//          SM_ASSERT_2_TESTS(a, type, t1, t2)
//          SM_ASSERT_3_TESTS(a, type, t1, t2, t3)
//          SM_ASSERT_4_TESTS(a, type, t1, t2, t3, t4)  
//          . . .
// ***********************************************************************/
// SM_EXPORT SmBoolean sm_AssertTests
//  (SmObject          * pObj,          // in : pObject to test
//   SM_TYPE             lExeClassType, // in : ClassType of ObjClass of ObjClass::AssertValid() method to execute
//   const TCHAR * const file_name,     // in : filename label for AssertArray report
//   ULONG               line_num,      // in : linenumber label for AssertArray report
//   const TCHAR * const func_name,     // in : function label for AssertArray report
//   ULONG               lNum,          // in : Number of test listed next
//  ... ) ;                             // in : list AssertValid test numbers to run, length:[lNum]
// end obsolete

/*******************************************************************//**
PURPOSE: sm_AssertValid() = Standardized access to pObj->AssertValid(), 
         in SM_DEBUG_CODE mode return pObj->AssertValid() else return TRUE

NOTES: 1. SM_ASSERT_VALID(a) was originally a macro that appeared only in SM_DEBUG_CODE mode.
       The MACRO was becoming awkward as more and more arguments were being added to control its behavior.
       The MACRO was rebuilt as a function retaining  old UPPER_CASE name for backward compatibility.

 2. The SM_ASSERT_VALID_ARGS() argument features include:
     eDumpObjFlag       - SM_NO_OBJDUMP     = don't call a->Dump()
                          SM_OBJDUMP_BEFORE = call obj->Dump() before obj->AssertValid()  
                          SM_OBJDUMP_AFTER  = call obj->Dump() after  obj->AssertValid()  
                          SM_OBJDUMP_ONLY   = only obj->Dump() skip   obj->AssertValid()
     eStreamFlag        - SM_STREAM        = always dump AssertArray report
                          SM_NO_STREAM     = only dump AssertArray report for failed AssertValid checks
                          SM_GLOBAL_STREAM = use SmGetAssertValidStream() Stream/NoStream value ;
     eLevelFlag         - SM_LEVEL_0    = execute AssertValid Checks labeled as SM_LEVEL_0
                          SM_LEVEL_1    = execute AssertValid Checks labeled as SM_LEVEL_0 or SM_LEVEL_1
                          SM_LEVEL_2    = execute AssertValid Checks labeled as SM_LEVEL_0, SM_LEVEL_1, or SM_LEVEL_2
                          SM_LEVEL_GIVEN= only execute AssertValid test number checks listed in pTestRequests input array  
     lExeClassType      - ClassType of ObjClass of ObjClass::AssertValid() method to execute - used to navigate virtual stacks - only used when Level == SM_LEVEL_GIVEN
     pTestRequests      - SmTArray<ULONG> of test numbers to execute   - only used when Level == SM_LEVEL_GIVEN
     eWalkFlag          - SM_WALK    = if target has descendant topology objects, walk the topo tree calling AssertValid on every descendant
                          SM_NO_WALK = call AssertValid on target only
     bDraw              - TRUE  = Add Graphics for failed AssertReports
                          FALSE = no Graphics

 3. TO DEBUG SM_ASSERT_VALID() calls: put break point in SmAssertReport() constructor

 RETRUN: TRUE = OK, FALSE = Problem
***********************************************************************/
SM_EXPORT SmBoolean sm_AssertValid
(
  const SmObject  * pObj,                           // in : Target Object to check                                                                    
  SmAssertObjDump   eDumpObjFlag =SM_NO_OBJDUMP,    // in : SM_NO_OBJDUMP     = never Dump Target Object                                              
                                                    //      SM_OBJDUMP_BEFORE = Dump Target Object before AssertArray report                          
                                                    //      SM_OBJDUMP_AFTER  = Dump Target Object after  AssertArray report                          
                                                    //      SM_OBJDUMP_ONLY   = Dump Target Object only, skip AssertArray report                      
  SmAssertStream    eStreamFlag=SM_GLOBAL_STREAM,   // in : SM_STREAM       = always dump AssertArray report                                          
                                                    //      SM_NO_STREAM    = only dump AssertArray report for failed AssertValid checks              
                                                    //      SM_GLOBAL_STREAM= Let global SM_bAssertValidStream value pick streaming                   
  SmAssertTestLevel eLevelFlag =SM_LEVEL_2,         // in : SM_LEVEL_0,    = do SM_LEVEL_O labeled Assert Checks                      (fastest)       
                                                    //      SM_LEVEL_1,    = do SM_LEVEL_0 and SM_LEVEL_1 labeled Assert Checks       (inbetween)     
                                                    //      SM_LEVEL_2     = do SM_LEVEL_0, SM_LEVEL_1, and SM_LEVEL_2 labeled Checks (most complete) 
                                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                            
  SM_TYPE           lExeClassType = SmObject_TYPE,  // in : ClassType of ObjClass of ObjClass::AssertValid() method to execute                        
                                                    // in : used to navigate virtual stacks - only used when Level == SM_LEVEL_GIVEN              
  SmTArray<ULONG> * pTRequests=NULL,                // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]          
  SmAssertWalking   eWalkFlag =SM_WALK,             // in : SM_WALK    = run AssertValid on target and any topology graph descendants                 
                                                    //     SM_NO_WALK = run AssertValid only on target                                                
  SmBoolean         bDraw  = FALSE,                 // in : TRUE = Add Graphics to open Stream for failing AssertReports                              
                                                    //      FALSE= No Graphics                                                                        
  // SmBoolean         bHeal  = FALSE,              // in : TRUE = Call AssertHeal on all failed AssertReports                                        
  //                                                //      FALSE= No Healing                                                                         
  const TCHAR * const file_name = NULL,             // in : filename label for AssertArray report                                                     
  ULONG               line_num  = 0,                // in : linenumber label for AssertArray report                                                   
  const TCHAR * const func_name = NULL              // in : function label for AssertArray report                                                     
);

// global management for eStreamFlag values
SM_EXPORT SmBoolean   SmSetAssertValidStream(SmBoolean bAssertValidStream) ;
SM_EXPORT SmBoolean & SmGetAssertValidStream() ;

  // TO DEBUG SM_ASSERT_VALID() calls: put break point in SmAssertReport() constructor
                                                 
// declarations for classes not derived from SmObject  (could of been a template function - who knew?/)                                                                                        
//                  sm_AssertValid(      pObj,                 eDumpObjFlag,    eStreamFlag,    eLevelFlag,     lExeClassType, pTRequests,        eWalkFlag,       bDraw,     file_name,         line_num, func_name       
SM_EXPORT SmBoolean sm_AssertValid(const SmAxis2Placement *,   SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG,  const TCHAR * const) ; 
SM_EXPORT SmBoolean sm_AssertValid(const SmPseudoBox *,        SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG,  const TCHAR * const) ; 
SM_EXPORT SmBoolean sm_AssertValid(const SmPeriodicExtent1d *, SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG,  const TCHAR * const) ; 
SM_EXPORT SmBoolean sm_AssertValid(const SmExtent1d *,         SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG,  const TCHAR * const) ;
                                                                                                                                                                                                           
SM_EXPORT SmBoolean sm_AssertValid(const SmPolarConversion *,  SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG,  const TCHAR * const) ; 
SM_EXPORT SmBoolean sm_AssertValid(const SmPolarBox *,         SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG,  const TCHAR * const) ; 
SM_EXPORT SmBoolean sm_AssertValid(const SmExtent2d *,         SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG,  const TCHAR * const) ;
SM_EXPORT SmBoolean sm_AssertValid(const SmExtent3d *,         SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG,  const TCHAR * const) ;

SM_EXPORT SmBoolean sm_AssertValid(const SmFilletGeom *,       SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG,  const TCHAR * const) ;
SM_EXPORT SmBoolean sm_AssertValid(const SmCurveInterval *,    SmAssertObjDump, SmAssertStream, SmAssertTestLevel, SM_TYPE,    SmTArray<ULONG> *, SmAssertWalking, SmBoolean, const TCHAR * const, ULONG, const TCHAR * const) ;
//                  sm_AssertValid(      pObj,                 eDumpObjFlag,    eStreamFlag,    eLevelFlag,      lExeClassType, pTRequests,        eWalkFlag,       bDraw,     file_name,          line_num, func_name       
  
/*******************************************************************//**
// MACROS standardizing the checks written into the AssertValid() methods
//   SM_ASSERT_BOOLEAN_REPORT(ListEntryIndex, TestLevel, Predicate, OptMessage) 
//   SM_ASSERT_VALUE_REPORT  (ListEntryIndex, TestLevel, Predicate, Tol, Val, OptMessage)  
//   SM_ASSERT_PAIR_REPORT   (ListEntryIndex, TestLevel, Predicate, This, Other, Tol, OptMessage)
//
// MACROS standardizing the checks written into functions that might be called by an AssertValid() method
//        like the legacy SmBrep::ValidatePointers() and the SmTrimmingTools::Check functions.   
//   SM_ASSERT_OBJ_VALUE_REPORT  (ListIndex, ListEntryIndex, TestLevel, Predicate, this, Tol, Val, OptMessage)
//   SM_ASSERT_OBJ_BOOLEAN_REPORT(ListIndex, ListEntryIndex, TestLevel, Predicate, this, OptMessage)
//     
//   where: 
//    enum SmAssertListIndex  ListIndex      = index selects desired SmAssertReportLabel array for current class,
//                                              note: this value is used in non AssertValid() methods to allow
//                                                    those to have their own SmAssertReportLabel array to enable
//                                                    specific reporting for every check made.  These methods are
//                                                    typically legacy methods that are called by the newer set
//                                                    of AssertValid() methods.
//    ULONG                   ListEntryIndex = index specifies desired item in target SmAssertReportLabel array
//    enum SmAssertTestLevel  TestLevel      = AssertTest label used to skip very slow checks when desired.
//                                             SM_LEVEL_0 = this test run for all AssertValid calls
//                                             SM_LEVEL_1 = this test run only for level 1 and 2 AssertValid calls
//                                             SM_LEVEL_2 = this test run only for level 2 AssertValid calls.
//    SmBoolean expression    Predicate      = Any expression based on object data that evaluates to TRUE or FALSE
//                                             TRUE  = object data passes sanity check
//                                             FALSE = object data fails sanity check
//    double                  Tol            = when predicate depends on a tolerance value, that tolerance value.
//                                             It's saved in any crated SmAssertReports and displayed later when the
//                                             report is pretty printed.
//    TCHAR                   OptMessage     = When not empty - specifies the text for the SmAssertReportLabel Dump report
//                                             ignored when empty == _T("")
**********************************************************************/  

// macro for Boolean predicates in AssertValid() methods                           
#define SM_ASSERT_BOOLEAN_REPORT(ListEntryIndex, TestLevel, Predicate, OptMessage) \
        SM_ASSERT_OBJ_BOOLEAN_REPORT(this, 0, ListEntryIndex, TestLevel, Predicate, this, OptMessage)

// macro for tolerance using predicates in AssertValid() methods                           
#define SM_ASSERT_VALUE_REPORT(ListEntryIndex, TestLevel, Predicate, Tol, Val, OptMessage) \
        SM_ASSERT_OBJ_VALUE_REPORT(this, 0, ListEntryIndex, TestLevel, Predicate, this, Tol, Val, OptMessage)

// macro removes all cascading SmAssertReports:[ThisObj, CascadeClass CascadeListIndex] from an SmAssertArray
//   when ThisObj fails the ThisListIndex AssertValid test.
#define SM_ASSERT_REMOVE_CASCADING_REPORT(ThisListIndex, CascadeClass, CascadeListIndex, CascadeObj) \
  { ULONG nThisIndx, nTgtIndx ; \
    if(pAList && pAList->FindReport(this, ThisListIndex, GetClassType(), this, NULL, &nThisIndx)) \
      { for(;TRUE;) { if(pAList->FindReport(CascadeObj,  CascadeListIndex, CascadeClass##_TYPE, CascadeObj, NULL, &nTgtIndx)) \
                           { pAList->RemoveAt(nTgtIndx) ; } \
                      else { break ; } \
  }   }             }  

// macro boolean TRUE = AssertReport(TgtClass, TgtListIndex) for TgtObj exists in pAList[lIndx]
#define SM_ASSERT_REPORT_NOT_IN_ARRAY(TgtClass, TgtListIndex, TgtObj) \
   ( ((pAList) ? (FALSE == pAList->FindReport(TgtObj,  TgtListIndex, TgtClass##_TYPE, TgtObj)) : TRUE ) )

// Set last pAList->AssertReport member values only if previous SM_ASSERT_BOOLEAN_REPORT(),  
//   SM_ASSERT_VALUE_REPORT(), or SM_ASSERT_PAIR_REPORT() macro call added an AssertReport to pAList. 
// Intent: called immediately after an SM_ASSERT_XXX_REPORT() macro call.
#define SM_ASSERT_SET_VALUE(a)       if(pAList && pAList->GetLastAdd()) { pAList->GetLast()->SetValue(a) ; }
#define SM_ASSERT_SET_TOL3D(a)       if(pAList && pAList->GetLastAdd()) { pAList->GetLast()->SetTol3d(a) ; }
#define SM_ASSERT_SET_OWNER_PARAM(a) if(pAList && pAList->GetLastAdd()) { pAList->GetLast()->SetOwnerParam(a) ; }
#define SM_ASSERT_SET_OTHER_PARAM(a) if(pAList && pAList->GetLastAdd()) { pAList->GetLast()->SetOtherParam(a) ; }
#define SM_ASSERT_SET_DIST3D(a)      if(pAList && pAList->GetLastAdd()) { pAList->GetLast()->SetDist3d(a) ; }

// MACRO to standardize when an AssertValid predicate is run or skipped - returns TRUE when predicate is skipped or when predicate is TRUE
#define SM_ASSERT_TEST_NEED(LstEntryIdx, TLev)  SM_ASSERT_TEST_NEED_LISTIDX(0, LstEntryIdx, TLev)
#define SM_ASSERT_TEST_NEED_LISTIDX(ListIdx, ListEntryIdx, TLvl) \
/* need when chcking level and level is being tested */ (    ((eTestLevel <= SM_LEVEL_2) && (eTestLevel >= (TLvl))) \
/* need when chcking TestRequests and test requested */  ||  ((eTestLevel == SM_LEVEL_GIVEN) && pTestRequests && pTestRequests->IsIn(ListEntryIdx)) )

#define SM_ASSERT_TEST_CHECK(ListIndx, ListEntryIndx, TestLvl, Test) \
/* when test is NOT needed or */ (    !SM_ASSERT_TEST_NEED_LISTIDX(ListIndx, ListEntryIndx, TestLvl) \
/* when test passes           */  ||  (Test) )           


// in SM_DEBUG_CODE mode the following macros store a file_name and line number.
// not in SM_DEBUG_CODE mode file_name and line number are not stored.
#ifdef SM_DEBUG_CODE
#define SM_ASSERT_OBJ_BOOLEAN_REPORT(CodeOwner, ListIndex, ListEntryIndex, TestLevel, Predicate, ObjPtr, OptMessage)  \
        (  /* when asked - run test */             SM_ASSERT_TEST_CHECK(ListIndex, ListEntryIndex, TestLevel, Predicate) \
         ? /* passed or skipped test - rtn TRUE */ ( pAList ? pAList->SetLastAdd(FALSE), TRUE : TRUE) \
         : /* failed test - when given pAList, construct and store AssertReport, set m_lLastIndx - rtn FALSE */ \
           (   pAList \
            ? (pAList->AddUniqueDeleteDuplicate( new SmAssertReport(CodeOwner, ListEntryIndex, GetClassType(), \
                                                                    (ObjPtr), (ObjPtr)->GetClassType(), \
                                                                    (ObjPtr)->GetClassString(),         \
                                                                    NULL, 0, _T("NULL"),                \
                                                                    SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE,   \
                                                                    (ListIndex), OptMessage, FILE_NAME,LINE_NUMBER)), FALSE) \
            : FALSE))
// macro for tolerance using predicates in other methods (like SmBrep::VerifyPointers() and SmTrimmingTools::CheckFunctions())
#define SM_ASSERT_OBJ_VALUE_REPORT(CodeOwner, ListIndex, ListEntryIndex, TestLevel, Predicate, ObjPtr, Tol, Val, OptMessage) \
        ([&]() -> SmBoolean {                                                                            \
            SmBoolean bAssertRtn = TRUE;                                                                 \
            if( SM_ASSERT_TEST_CHECK(ListIndex, ListEntryIndex, TestLevel, Predicate) ) /* passed/skipped - TRUE */ \
                { if(pAList) { pAList->SetLastAdd(FALSE); } }                                             \
            else /* failed test - when given pAList, construct and store AssertReport, set m_lLastIndx - FALSE */ \
                { bAssertRtn = FALSE;                                                                    \
                  if(pAList) { pAList->AddUniqueDeleteDuplicate( new SmAssertReport(CodeOwner, ListEntryIndex, GetClassType(), \
                                                                    (ObjPtr), (ObjPtr)->GetClassType(), \
                                                                    (ObjPtr)->GetClassString(),         \
                                                                    NULL, 0, _T("NULL"),                \
                                                                    Tol, Val, (ListIndex), OptMessage, FILE_NAME,LINE_NUMBER) ); } } \
            return bAssertRtn; }())

// macro for comparing two objects with a tolerance 
#define SM_ASSERT_PAIR_REPORT(ListIndex, ListEntryIndex, TestLevel, Predicate, ThisPtr, OtherPtr, Tol, Val, OptMessage) \
        (  /* when asked - run test */             SM_ASSERT_TEST_CHECK(ListIndex, ListEntryIndex, TestLevel, Predicate) \
         ? /* passed or skipped test - rtn TRUE */ ( pAList ? pAList->SetLastAdd(FALSE), TRUE : TRUE)             \
         : /* failed test - when given pAList, construct and store AssertReport, set m_lLastIndx - rtn FALSE */ \
           (   pAList \
            ? (pAList->AddUniqueDeleteDuplicate( new SmAssertReport((this), ListEntryIndex, GetClassType(), \
                                                                    (ThisPtr), (ThisPtr)->GetClassType(),   \
                                                                    (ThisPtr)->GetClassString(),            \
                                                                    (OtherPtr), (OtherPtr)->GetClassType(), \
                                                                    (OtherPtr)->GetClassString(),           \
                                                                    Tol, Val, (ListIndex), OptMessage, FILE_NAME,LINE_NUMBER)), FALSE) \
            : FALSE))

#else  // no SM_DEBUG_CODE
// macro for Boolean predicates in other methods (like SmBrep::VerifyPointers() and SmTrimmingTools::CheckFunctions()
#define SM_ASSERT_OBJ_BOOLEAN_REPORT(CodeOwner, ListIndex, ListEntryIndex, TestLevel, Predicate, ObjPtr, OptMessage) \
        (  /* when asked  - run test */ ((eTestLevel < (TestLevel)) || (Predicate) )       \
         ? /* passed test - rtn TRUE */ ( pAList ? pAList->SetLastAdd(FALSE), TRUE : TRUE) \
         : /* failed test - when given pAList, construct and store AssertReport, set m_lLastIndx - rtn FALSE */ \
           (   pAList \
            ? (pAList->AddUniqueDeleteDuplicate( new SmAssertReport(CodeOwner, ListEntryIndex, GetClassType(), \
                                                                    (ObjPtr), (ObjPtr)->GetClassType(), \
                                                                    (ObjPtr)->GetClassString(),         \
                                                                    NULL, 0, _T("NULL"),                \
                                                                    SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE,   \
                                                                    (ListIndex), OptMessage)), FALSE)   \
            : FALSE))      
                                 
// macro for tolerance using predicates in other methods (like SmBrep::VerifyPointers() and SmTrimmingTools::CheckFunctions())
#define SM_ASSERT_OBJ_VALUE_REPORT(CodeOwner, ListIndex, ListEntryIndex, TestLevel, Predicate, ObjPtr, Tol, Val, OptMessage) \
        ([&]() -> SmBoolean {                                                                                   \
            SmBoolean bAssertRtn = TRUE;                                                                        \
            if( ((eTestLevel < (TestLevel)) || (Predicate)) ) /* when asked - run test; passed/skipped - TRUE */\
                { if(pAList) { pAList->SetLastAdd(FALSE); } }                                                   \
            else /* failed test - when given pAList, construct and store AssertReport, set m_lLastIndx - FALSE */\
                { bAssertRtn = FALSE;                                                                           \
                  if(pAList) { pAList->AddUniqueDeleteDuplicate( new SmAssertReport(CodeOwner, ListEntryIndex, GetClassType(),  \
                                                                    (ObjPtr), (ObjPtr)->GetClassType(),         \
                                                                    (ObjPtr)->GetClassString(),                 \
                                                                    NULL, 0, _T("NULL"),                        \
                                                                    Tol, Val, (ListIndex), OptMessage) ); } }   \
            return bAssertRtn; }())

// macro for comparing two objects with a tolerance 
#define SM_ASSERT_PAIR_REPORT(ListIndex, ListEntryIndex, TestLevel, Predicate, ThisPtr, OtherPtr, Tol, Val, OptMessage) \
        (  /* when asked  - run test */ ((eTestLevel < (TestLevel)) || (Predicate) )       \
         ? /* passed test - rtn TRUE */ ( pAList ? pAList->SetLastAdd(FALSE), TRUE : TRUE) \
         : /* failed test - when given pAList, construct and store AssertReport, set m_lLastIndx - rtn FALSE */ \
           (   pAList \
            ? (pAList->AddUniqueDeleteDuplicate( new SmAssertReport(this, ListEntryIndex, GetClassType(),       \
                                                                    (ThisPtr), (ThisPtr)->GetClassType(),       \
                                                                    (ThisPtr)->GetClassString(),                \
                                                                    (OtherPtr), (OtherPtr)->GetClassType(),     \
                                                                    (OtherPtr)->GetClassString(),               \
                                                                    Tol, Val, (ListIndex), OptMessage)), FALSE) \
            : FALSE))

#endif // no SM_DEBUG_CODE               

#endif // __SMASSERT_VALID_H__

  
