// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmHealProbArray.h
* PURPOSE: Header file for template class hierarchy 
             SmProbArray<TYPE>
               SmPairProbArray <TYPE>
               SmMixedProbArray<TYPE>
               SmTriedProbArray<TYPE>
**********************************************************************/

#ifndef __SMHEALPROBARRAY_H__
#define __SMHEALPROBARRAY_H__

//#pragma warning(disable : 4291)   // no matching operator delete found; // restored these warnings to clean up errors on linux

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif // no __SMOS_TYPES_H__ 

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif // no __SMTARRAY_H__

/*******************************************************************//**
PURPOSE: Section - HealProbArray/LinkedObjPropMember enums

NOTES: declare before including the ObjProps headers
***********************************************************************/

/*******************************************************************//**
PURPOSE: list of SmMixedProbArray<TYPE> object property enum values

NOTES: an enum of this type is declared for every obj property
       that gets stored in a SmHealData Mixed ProbArray where a obj
       property may be either okay or a problem that needs to be
       fixed. 
EXAMPLE: SmHealData::m_sFaceProps_Sheets is paired with SmFaceProps::m_eSheet
***********************************************************************/
enum SmMixedLinkType
{
  SM_MP_UNDEF,    // propetry value not yet known
  SM_MP_NOPROP,   // obj does not have the property
  SM_MP_HASPROP,  // obj has property - not yet known if that's okay or a problem
  SM_MP_OKAY,     // obj has property and is known to be okay
  SM_MP_PROB,     // obj has property and is known to be problem needing fixing
  SM_MP_FIXED,    // obj has property that was a problem and is now fixed
} ; // end enum SmMixedLinkType

/*******************************************************************//**
PURPOSE: list of SmTriedProbArray<TYPE> object property enum values

NOTES: an enum of this type is declared for every obj property
       that gets stored in a SmHealData Tried ProbArray where an obj
       was run through a particular Fix function.  
EXAMPLE: SmHealData::m_sFixFaceProps_MoveSeam is paired with SmFaceProps::m_eBeenThroughMoveSeam
***********************************************************************/
enum SmTriedLinkType
{
  SM_TRY_NONE,     // obj not yet run through targeted Fix function
  SM_TRY_RAN,      // obj run through targeted fix function - not yet checked for fixed or broken
  SM_TRY_FIXED,    // obj run through targeted fix function - checked property is fixed
  SM_TRY_NOFIX,    // obj run through targeted fix function - checked property is not fixed
} ; // end enum SmMixedLinkType

/*******************************************************************//**
PURPOSE: Control Healer Operations

NOTES: In the future this can be extended to target specific checks and
       fixes within the healer.  Currently the only options are
       to try and fix all known problems and look for as many problems
       as possible.  Problem checking terminates early if Degenerate Edges
       or Edges Crossing Seams are found.
***********************************************************************/
enum SmHealerOpType
{
  // these enum values must be in the order in which the healer runs these operations
  SM_HO_NONE = 0,                           //  No Healer Steps run
                                 
  SM_HO_FIX_BACKPOINTERS,                   // Ran All Healer Steps thru SmHealData::Fix_BackPointers
                                     
  SM_HO_CACHE_EDGEPROPS,                    // Ran All Healer Steps thru SmHealData::Cache_EdgeProps()  
  SM_HO_CACHE_VERTEXPROPS,                  // Ran All Healer Steps thru SmHealData::Cache_VertexProps()          
  SM_HO_CACHE_FACEPROPS_GAPS,               // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Gaps()
                                            
  SM_HO_FIX_TOLSIZES,                       // Ran All Healer Steps thru SmHealData::Fix_TolSizes()
  SM_HO_CACHE_COIN_VERTICES,                // Ran All Healer Steps thru SmHealData::Cache_CoinVertices()
  SM_HO_FIX_COIN_VERTICES,                  // Ran All Healer Steps thru SmHealData::Fix_CoinVertices()
                                               
  SM_HO_CACHE_DEGEN_FACES,                  // Ran All Healer Steps thru SmHealData::Cache_DegenFaces()
  SM_HO_FIX_DEGEN_FACES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenFaces()
                                               
  SM_HO_FIX_DEGEN_EDGES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenEdges()
                                             
  SM_HO_CACHE_COIN_EDGES,        /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_CoinEdges()
  SM_HO_FIX_COIN_EDGES,          /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_CoinEdges()
                                               
  SM_HO_CACHE_MISSED_EDGEXSECTS, /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_MissedEdgeXSects()
  SM_HO_FIX_MISSED_EDGEXSECTS,   /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_MissedEdgeXSects()
                                               
  SM_HO_FIX_UNCONTAINED_EDGES,   /* STUB */ // Ran All Healer Steps thru SmHealData::Fix_UncontainedEdges()
  SM_HO_FIX_BADGAPS,             /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_BadGaps()
                                               
  SM_HO_CACHE_FACEPROPS_2,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage2()
                                               
  SM_HO_FIX_MOVESEAM,                       // Ran All Healer Steps thru SmHealData::Fix_MoveSeam()
  SM_HO_FIX_SPLITEDGE_ATSEAM,               // Ran All Healer Steps thru SmHealData::Fix_SplitEdgesAtSeam()
  SM_HO_FIX_BADSHEETS,                      // Ran All Healer Steps thru SmHealData::Fix_BadSheets()
                                               
  SM_HO_CACHE_FACEPROPS_3,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage3()
                                               
  SM_HO_FIX_SPLITFACE_ATSEAMS,              // Ran All Healer Steps thru SmHealData::Fix_SplitFaceAtSeams()
  SM_HO_FIX_BADLOOPS,                       // Ran All Healer Steps thru SmHealData::Fix_BadLoops()
  SM_HO_MAKE_UVTRIMCURVES,                  // Ran All Healer Steps thru SmHealData::Make_UVTrimCurves()
  SM_HO_FIX_INFINITE_REGIONS,               // Ran All Healer Steps thru SmHealData::Fix_InfiniteRegion()
                                               
  SM_HO_ALL                                 // Ran All Healer Steps 
                                       
} ; // end enum SmHealerOpType

#define SM_HEALOP_STRING(HealerOp) \
 (  (HealerOp == SM_HO_NONE                   ) ? _T("No Healer Step")                     \
  : (HealerOp == SM_HO_FIX_BACKPOINTERS       ) ? _T("SmHealData::Fix_BackPointers")       \
  : (HealerOp == SM_HO_CACHE_EDGEPROPS        ) ? _T("SmHealData::Cache_EdgeProps() (includes Cache_DegenEdges())") \
  : (HealerOp == SM_HO_CACHE_VERTEXPROPS      ) ? _T("SmHealData::Cache_VertexProps()")    \
  : (HealerOp == SM_HO_CACHE_FACEPROPS_GAPS   ) ? _T("SmHealData::Cache_FaceProps_Gaps()") \
                                                                                           \
  : (HealerOp == SM_HO_FIX_TOLSIZES           ) ? _T("SmHealData::Fix/*  */ _TolSizes()")  \
  : (HealerOp == SM_HO_CACHE_COIN_VERTICES    ) ? _T("SmHealData::Cache_CoinVertices()")   \
  : (HealerOp == SM_HO_FIX_COIN_VERTICES      ) ? _T("SmHealData::Fix_CoinVertices()") \
                                                                                                  \
  : (HealerOp == SM_HO_CACHE_DEGEN_FACES      ) ? _T("SmHealData::Cache_DegenFaces()")     \
  : (HealerOp == SM_HO_FIX_DEGEN_FACES        ) ? _T("SmHealData::Fix_DegenFaces()")       \
                                                                                           \
  : (HealerOp == SM_HO_FIX_DEGEN_EDGES        ) ? _T("SmHealData::Fix_DegenEdges()")       \
                                                                                           \
  : (HealerOp == SM_HO_CACHE_COIN_EDGES       ) ?  _T("/* TODO */ SmHealData::Cache_CoinEdges()")  \
  : (HealerOp == SM_HO_FIX_COIN_EDGES         ) ?  _T("/* TODO */ SmHealData::Fix_CoinEdges()")    \
                                                                                                   \
  : (HealerOp == SM_HO_CACHE_MISSED_EDGEXSECTS) ?  _T("/* TODO */ SmHealData::Cache_MissedEdgeXSects()") \
  : (HealerOp == SM_HO_FIX_MISSED_EDGEXSECTS  ) ?  _T("/* TODO */ SmHealData::Fix_MissedEdgeXSects()")   \
                                                                                                         \
  : (HealerOp == SM_HO_FIX_UNCONTAINED_EDGES  ) ?  _T("/* STUB */ SmHealData::Fix_UncontainedEdges()")   \
  : (HealerOp == SM_HO_FIX_BADGAPS            ) ?  _T("/* TODO */ SmHealData::Fix_BadGaps()") \
                                                                                              \
  : (HealerOp == SM_HO_CACHE_FACEPROPS_2      ) ?  _T("SmHealData::Cache_FaceProps_Stage2()") \
                                                                                              \
  : (HealerOp == SM_HO_FIX_MOVESEAM           ) ?  _T("SmHealData::Fix_MoveSeam()")           \
  : (HealerOp == SM_HO_FIX_SPLITEDGE_ATSEAM   ) ?  _T("SmHealData::Fix_SplitEdgesAtSeam()")   \
  : (HealerOp == SM_HO_FIX_BADSHEETS          ) ?  _T("SmHealData::Fix_BadSheets()")          \
                                                                                              \
  : (HealerOp == SM_HO_CACHE_FACEPROPS_3      ) ?  _T("SmHealData::Cache_FaceProps_Stage3()") \
                                                                                              \
  : (HealerOp == SM_HO_FIX_SPLITFACE_ATSEAMS  ) ?  _T("SmHealData::Fix_SplitFaceAtSeams()")   \
  : (HealerOp == SM_HO_FIX_BADLOOPS           ) ?  _T("SmHealData::Fix_BadLoops()")           \
  : (HealerOp == SM_HO_MAKE_UVTRIMCURVES      ) ?  _T("SmHealData::Make_UVTrimCurves()")      \
  : (HealerOp == SM_HO_FIX_INFINITE_REGIONS   ) ?  _T("SmHealData::Fix_InfiniteRegion()")     \
                                                                                              \
  : (HealerOp == SM_HO_ALL                    ) ?  _T("All Healer Steps ")                    \
  :                                                _T("Unknown Option")                       \
 ) /* end Macro SM_HEALOP_STRING */

/*******************************************************************//**
PURPOSE: define the classes to be loaded into SmHealProbArray objs

NOTES: 
***********************************************************************/
#ifndef __SMVERTEXPROPS_H__
#include <SmVertexProps.h>
#endif

#ifndef __SMEDGEPROPS_H__
#include <SmEdgeProps.h>
#endif

#ifndef __SMLOOPPROPS_H__
#include <SmLoopProps.h>
#endif

#ifndef __SMFACEPROPS_H__
#include <SmFaceProps.h>
#endif

/*******************************************************************//**
PURPOSE: SmHealData ProbArrays

NOTES: HealData ProbArrays managed by one of the derived classes as oneof:
        ProbArray          Loaded with all bad ObjProps - prob ObjProps  
                           Each ObjProp has a boolean flag marking itself as fixed or broken for this problem. 
                           Objs are left in the list or removed from the list as they are fixed as managed by the user. 
                           The original prob cnt is saved in the ULONG value m_lOrigProbCount. 
                           The current Prob is the number of members whose boolean flag is set to true.
        COIN ProbArray     Class SmPairProbArray<TYPE>: Loaded with coincident pairs of ObjProps.  
                           Deleted obj ObjProps are removed from the list as the coincidence is repaired 
                           and surviving members are moved to the back of the list.  
                           The original coincident prob cnt is kept as a ULONG value.
        MiXED ProbArray    Class SmMixedProbArray<TYPE>: Loaded with all obj ObjProps that have a 
                           property that may or may not be a problem. Each ObjProp has an enum flag  
                           marking the Obj as oneof SM_MP_HASPROP, SM_MP_OKAY, SM_MP_PROB, SM_MP_FIXED.
                           The original prob count is sum of members marked as Prob or Fixed. 
                           The length of the array is the count of objects with the property.
        TRIED ProbArray    Class SmTriedProbArray<TYPE>: Loaded with all obj ObjProbs that were run 
                           through a particular FIX method whether they were fixed or not.
                           Each ObjProp has an enum flag marking itself as fixed or not due to being run through
                           this particular FIX method. 

EXAMPLE USE: ProbArray      : SmProbArray<SmEdgeProp> SmHealData ::m_sBadEdgeProps_SmallZoneTol3d
             linked ProbFlag: SmBoolean               SmEdgeProps::&m_bBadSmallZoneTol3d ;

      // Setup ProbArray
      m_sBadEdgeProps_SmallZoneTol3d.Setup(SM_HO_CACHE_EDGEPROPS, SmEdgeProps::&m_bBadSmallZoneTol3d) ;
      m_sBadEdgeProps_SmallZoneTol3d.SetLabels(_T("Edge->ZoneTol3d Too Small"),
                                               _T("Small Tols Cnt"),
                                               SM_PROBTYPE_CAN_BE_OKAY) ; // oneof: SM_PROBTYPE_BAD     
                                                                          //        SM_PROBTYPE_WARN       
                                                                          //        SM_PROBTYPE_CAN_BE_OKAY

      . . . 

      // run SM_HO_CACHE_EDGEPROPS heal step
      Cache_EdgeProps(pBrep,SM_HO_CACHE_EDGEPROPS) 

      . . . 

      // output its report
      sPPEdge_SmallZoneTol3d.Dump() ;

      // which outputs one of the following two lines
      //   if(m_sBadEdgeProps_SmallZoneTol3d == 0)
      //        "step:[HealerOp] Edge->ZoneTol3d Too Small: Small Tols Cnt:[OrigCnt, 0] - Okay"
      //   else "step:[HealerOp] Edge->ZoneTol3d Too Small: Small Tols Cnt:[OrigCnt, CurrCnt] - Can Be Okay"

LAST NOTE: Valid values for TYPE include 
   SmProbArray<SmFaceProp>      using SmProbArray<SmFaceProp *>   will cause a compile error
   SmProbArray<SmLoopProp>      using SmProbArray<SmLoopProp *>   will cause a compile error
   SmProbArray<SmEdgeProp>      using SmProbArray<SmEdgeProp *>   will cause a compile error
   SmProbArray<SmVertexProp>    using SmProbArray<SmVertexProp *> will cause a compile error
***********************************************************************/
template<class TYPE> class SmProbArray
{
 public:
  SmTArray<TYPE *> m_sProbArray ;                   // problem array managed by one of the derived classes
                   
  ULONG            TYPE::*m_plObjProbFlag = NULL ;  // ptr to member ObjProbFlag. works with ULONGs and SmBooleans
                                                    //    ex: &SmEdgeProps::m_bBadSmallZoneTol3d
                                                    //    FALSE or 0         = orig prob m_pDataArray fixed
                                                    //    TRUE  or 1 or MORE = not yet fixed
                   
  ULONG            m_lOrigProbCount = 0 ;           // Max number of problems found. Note: one fix is to delete an object
                                                    //  when that happens the object is deleted from all the prob lists.
                                                    //  OrigProbCount records the total number of prob objs before
                                                    //  any objects were deleted.
  SmHealerOpType   m_eGetHealerOp = SM_HO_NONE ;    // HealerOp where ProbArray is built
                   
  TCHAR            m_sOutputLabel[SM_TBLOCK_SIZE] = {0} ; // Dump output label,                    ex: _T("Edge->ZoneTol3d Too Small")
  TCHAR            m_sDataLabel[SM_TBLOCK_SIZE]   = {0} ; // Dump data label,                      ex: _T("Small Tols Cnt")
  TCHAR            m_sProbLabel[SM_TBLOCK_SIZE]   = {0} ; // for problem arrays EndOfStr Prob Text - use std macro strings
                                                          //  ex: SM_PROBTYPE_BAD          = _T(" - Bad ")
                                                          //      SM_PROBTYPE_WARN         = _T(" - Warn ")
                                                          //      SM_PROBTYPE_CAN_BE_OKAY  = _T(" - Can Be Okay ")
 public:
  // constructor - 
  SmProbArray(SmHealerOpType eGetHealerOp=SM_HO_NONE)      // in : Healer op that constructs this probArray
    : m_plObjProbFlag(NULL),
      m_lOrigProbCount(0),
      m_eGetHealerOp(eGetHealerOp)
    { }

  // copy constructor, assignment operator, virtual MakeCopy, equality operator
  SmProbArray(const SmProbArray & crOriginal) { m_sProbArray     = crOriginal.m_sProbArray ;
                                                m_plObjProbFlag  = crOriginal.m_plObjProbFlag ;
                                                m_lOrigProbCount = crOriginal.m_lOrigProbCount ;
                                                m_eGetHealerOp   = crOriginal.m_eGetHealerOp ; 
                                                smos_WStrCpy(m_sOutputLabel, SM_TBLOCK_SIZE -1, crOriginal.m_sOutputLabel) ;
                                                smos_WStrCpy(m_sDataLabel,   SM_TBLOCK_SIZE -1, crOriginal.m_sDataLabel) ;
                                                smos_WStrCpy(m_sProbLabel,   SM_TBLOCK_SIZE -1, crOriginal.m_sProbLabel) ;
                                              }
  SmProbArray & operator= (const SmProbArray &crOther ) { if(this == &crOther) return *this ;
                                                          m_sProbArray     = crOther.m_sProbArray ;
                                                          m_plObjProbFlag  = crOther.m_plObjProbFlag ;
                                                          m_lOrigProbCount = crOther.m_lOrigProbCount ;
                                                          m_eGetHealerOp   = crOther.m_eGetHealerOp ; 
                                                          smos_WStrCpy(m_sOutputLabel, SM_TBLOCK_SIZE -1,crOther.m_sOutputLabel);
                                                          smos_WStrCpy(m_sDataLabel,   SM_TBLOCK_SIZE -1,crOther.m_sDataLabel);
                                                          smos_WStrCpy(m_sProbLabel,   SM_TBLOCK_SIZE -1,crOther.m_sProbLabel);
                                                          return(*this) ;
                                                        }
  SmBoolean operator== (const SmProbArray &crOther ) { if(this == &crOther) return TRUE ;
                                                          SmBoolean bRtn = m_sProbArray == crOther.m_sProbArray ;
                                                          bRtn &= m_plObjProbFlag  == crOther.m_plObjProbFlag ;
                                                          bRtn &= m_lOrigProbCount == crOther.m_lOrigProbCount ;
                                                          bRtn &= m_eGetHealerOp   == crOther.m_eGetHealerOp ; 
                                                          bRtn &= (0 == SM_STRCMP(m_sOutputLabel, crOther.m_sOutputLabel)) ;
                                                          bRtn &= (0 == SM_STRCMP(m_sDataLabel,   crOther.m_sDataLabel)) ;
                                                          bRtn &= (0 == SM_STRCMP(m_sProbLabel,   crOther.m_sProbLabel)) ;
                                                          return(bRtn) ;
                                                         }
  virtual SmProbArray * MakeCopy() const { return( new SmProbArray(*this) ) ; }

  // destructor
  ~SmProbArray<TYPE>() { m_sProbArray.ReSet() ; m_plObjProbFlag = NULL ; m_lOrigProbCount = 0 ; m_eGetHealerOp = SM_HO_NONE ; } 

  // ReSet
  virtual void ReSet() { m_sProbArray.ReSet() ; }  // note: resetting the ProbArray does not reset the OrigProbCount

  // init boolean ProbFlag pointer
  void Setup
    (SmHealerOpType eGetHealerOp,         // in : Healer op that constructs this probArray
     ULONG          TYPE::*plObjProbFlag) // in : ptr to member ObjProbFlag. ex: &SmEdgeProps::m_bBadSmallZoneTol3d
                                          //        value of ptr: FALSE = orig prob listed in m_pDataArray fixed
                                          //                      TRUE  = not yet fixed
    { m_eGetHealerOp   = eGetHealerOp ;
      m_plObjProbFlag  = plObjProbFlag ; 
    } 

  // simple data access
  ULONG             GetOrigProbCount()                               { return m_lOrigProbCount ; }
  ULONG             IncOrigProbCount()                               { return ( m_lOrigProbCount++ ) ; }
  virtual SmBoolean Remove          (TYPE * pTgtElement)             { return m_sProbArray.Remove(pTgtElement) ; }
  void              RemoveAll       ()                               { m_sProbArray.RemoveAll() ; }
  ULONG             GetSize         ()                               { return m_sProbArray.GetSize() ; }
  TYPE           *& operator[]      (ULONG lIndex)                   { return m_sProbArray.GetAt(lIndex) ; }
  TYPE           *& GetAt           (ULONG lIndex)                   { return m_sProbArray.GetAt(lIndex) ; }
  void              SetOrigProbCnt  ( ULONG lOrigProbCount)          { m_lOrigProbCount = lOrigProbCount ; }
  ULONG             Add             (TYPE * pNewElement,
                                     SmBoolean bSetObjProbFlag=TRUE) { if(pNewElement == NULL) 
                                                                         { SM_ASSERT_ERR_MSG(_T("SmProbArray::Add: Error - added a NULL prob to array - see if this is needed")) ; 
                                                                           return SM_UNDEF_ULONG; }
                                                                       m_lOrigProbCount = m_sProbArray.GetSize() + 1 ;
                                                                       if(bSetObjProbFlag && m_plObjProbFlag != NULL) 
                                                                         { pNewElement->*m_plObjProbFlag = TRUE ; }
                                                                       return(m_sProbArray.Add(pNewElement)) ;
                                                                     }
  SmBoolean         AddUnique       (TYPE * pNewElement,
                                     SmBoolean bSetObjProbFlag=TRUE) { if(pNewElement == NULL) 
                                                                         { SM_ASSERT_ERR_MSG(_T("SmProbArray::AddUnique: Error - added a NULL prob to array - see if this is needed")) ; 
                                                                           return FALSE; }
                                                                       if(m_sProbArray.AddUnique(pNewElement))
                                                                         { m_lOrigProbCount = m_sProbArray.GetSize() ; 
                                                                           if(bSetObjProbFlag && m_plObjProbFlag != NULL) 
                                                                             { pNewElement->*m_plObjProbFlag = TRUE ; }
                                                                           return TRUE ; 
                                                                         }
                                                                       return FALSE ;
                                                                     }
                                    
  // init text labels
  void SetLabels(const TCHAR * pOutputLabel, // in : Dump output label,                    ex: _T("Edge->ZoneTol3d Too Small")
                 const TCHAR * pDataLabel,   // in : Dump data label,                      ex: _T("Small Tols Cnt")
                 const TCHAR * pProbLabel)   // in : for problem arrays EndOfStr Prob Text - use std macro strings
                                       //       ex: SM_PROBTYPE_BAD          = _T(" - Bad ")
                                       //           SM_PROBTYPE_WARN         = _T(" - Warn ")
                                       //           SM_PROBTYPE_CAN_BE_OKAY  = _T(" - Can Be Okay ")
    { // copy the strings
      smos_sprintf(m_sOutputLabel,_T("%s"),pOutputLabel) ; 
      smos_sprintf(m_sDataLabel,  _T("%s"),pDataLabel  ) ; 
      smos_sprintf(m_sProbLabel,  _T("%s"),pProbLabel  ) ; 
    }

  // pretty print
  //  "step:[m_eGetHealerOp] m_sOutputLabel: m_sDataLabel:[m_sProbArray.GetSize()] - Okay (only informational)"
  //   example: step:[m_eGetHealerOp] MoveSeam run count:[%4lu] - Okay (only informational)
  virtual ULONG Dump(SmHealerOpType eDoneHealerOp) const 
    {
      // locals
      ULONG ii, lCurrCnt = 0 ;

      // When heal sequence is far enough along                               
      if(eDoneHealerOp >= m_eGetHealerOp)                                  
        {            
          TCHAR sBuff[8*SM_TBLOCK_SIZE];    

          // count the unfixed problems in this array                          
          for(ii=0; ii<m_sProbArray.GetSize();ii++)                            
            { if(FALSE != m_sProbArray.GetAt(ii)->*m_plObjProbFlag)                    
                { lCurrCnt++ ; }                                                   
            }                                                                      
                                                                                
          smos_sprintf(sBuff, _T("\n  step:[%2d] %.256s: %.256s:[%4lu, %4lu]%.256s"),        
                     m_eGetHealerOp, m_sOutputLabel, m_sDataLabel,                    
                     smos_Max(m_lOrigProbCount, m_sProbArray.GetSize()), lCurrCnt,                     
                     (lCurrCnt == 0) ? _T(" - Okay") : m_sProbLabel ) ;
          smos_WriteBuffer(sBuff) ;

          // checkmarx - rewrite the orig SM_SPRINTF call so SM_SPRINTF isn't used to copy a buffer to a buffer - see if that removes the warning on the next scan
          SM_SPRINTF(sBuff, _T("\n  step:[%2d] "), m_eGetHealerOp) ; smos_WriteBuffer(sBuff) ;
          smos_WriteBuffer(              m_sOutputLabel) ; smos_WriteBuffer(_T(": ")) ;
          smos_WriteBuffer(              m_sDataLabel)   ; smos_WriteBuffer(_T(":")) ;
          SM_SPRINTF(sBuff,              _T("[%4lu, %4lu]"), smos_Max(m_lOrigProbCount, m_sProbArray.GetSize()), lCurrCnt) ; smos_WriteBuffer(sBuff) ;
          if(lCurrCnt == 0) { smos_WriteBuffer(_T(" - Okay")) ; }
          else              { smos_WriteBuffer( m_sProbLabel ) ; }
          //  SM_SPRINTF(sBuff, _T("\n  step:[%2d] %s: %s:[%4lu, %4lu]%s"),        
          //             m_eGetHealerOp, m_sOutputLabel, m_sDataLabel,                    
          //             smos_Max(m_lOrigProbCount, m_sProbArray.GetSize()), lCurrCnt,                     
          //             (lCurrCnt == 0) ? _T(" - Okay") : m_sProbLabel ) ;
          //  smos_WriteBuffer(sBuff) ;
        }                                                                       

      // all done
      return(lCurrCnt) ;
    } // end SmProbArray<TYPE>::Dump

  // pretty print the array and its contained tgts if it contains any of the input tgt objs, Rtn number of Tgts in Array
  ULONG DumpTgts(SmTArray<TYPE *> &rTgts, SmBoolean bRptNoTgtArrays) const 
    {
      TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
      SmTArray<TYPE *> sResult ;

      // find tgts in array
      m_sProbArray.FindCommonElements(rTgts, sResult) ;
          
      // count the hits
      ULONG lTgtCnt = sResult.GetSize() ;

      // When ProbArray does not contain any tgted members
      if(lTgtCnt == 0)  
        { 
          if(bRptNoTgtArrays == TRUE)
            {
              smos_sprintf(sBuff, _T("\n sized:[%3lu] %.512s contains:[no ] Tgted members: [none]"), m_sProbArray.GetSize(), m_sOutputLabel) ; 
              smos_WriteBuffer(sBuff) ; 
            }
        } // end empty array branch 
      else // ProbArray does contain tgted members branch
        {
          // no Tgts in array
          smos_sprintf(sBuff, _T("\n sized:[%3lu] %.512s contains:[%3lu] Tgts: ["), m_sProbArray.GetSize(), m_sOutputLabel, sResult.GetSize()) ; 
          smos_WriteBuffer(sBuff) ; 
          for(ULONG mi=0;mi<sResult.GetSize();mi++)
            { smos_sprintf(sBuff,        _T("0x%p"), sResult[mi]) ; 
              smos_sprintf(sBuffForFile, _T("%s"), sResult[mi] ? _T("notNULL") : _T("NULL")) ; 
              smos_WriteBuffer(sBuff, sBuffForFile) ; 
              if(mi+1 == sResult.GetSize()) { smos_WriteBuffer(_T("]"), _T("]")) ; } 
              else                          { smos_WriteBuffer(_T(", "), _T(", ")) ; } 
            } // end iter all common members
         } // end array has members branch

       // all done
       return(lTgtCnt) ;
     } // end wHealData::DumpTgts

} ; // end template<class TYPE> class SmProbArray

#ifdef _WIN32
#define SM_PROB_ARRAY_TEMPLATE_PREDECLARATION(x) template class SM_EXPORT SmProbArray<x>
  SM_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmVertexProps) ;
  SM_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmEdgeProps) ;
  SM_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmLoopProps) ;
  SM_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmFaceProps) ;
  SM_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmObjProps) ;
#undef SM_KEEP_PROB_ARRAY_TEMPLATE_PREDECLARATION
#endif // _WIN32

/*******************************************************************//**
PURPOSE: SmHealData PairProbArrays (initially for coincidence)

NOTES: A pairProbArray is:
         Loaded with coincident pairs of ObjProps.  deleted obj ObjProps are removed from
         the list as the coincidence is repaired and surviving members are moved to the back of the list.  
         The original coincident prob cnt is kept as a ULONG value.
           Orig Prob Count = m_lOrigCoinCnt ;
           Curr Prob Count = Count of all m_sProbArray list member pairs where
                                m_sProbArray[ii]->ProbFlag == TRUE for both pair members.
EXAMPLE USE: with SmHealData::m_sBadVertexProps_CoinVertices and OrigCnt m_plOrigCoinCnt

      // Setup ProbArray
      m_sBadEdgeProps_DegenEdges.Setup(SM_HO_CACHE_VERTEXPROPS) ;
      m_sBadEdgeProps_DegenEdges.SetLabels(_T("Vertex->Vertex spacing too short"),
                                           _T("Coincident Vertex Pair Cnt"),
                                           SM_PROBTYPE_BAD) ;   // oneof: SM_PROBTYPE_BAD     
                                                                //        SM_PROBTYPE_WARN       
                                                                //        SM_PROBTYPE_CAN_BE_OKAY

      . . . 

      // run SM_HO_CACHE_EDGEPROPS heal step
      Cache_VertexProps(pBrep,SM_HO_CACHE_VERTEXPROPS) 

      . . . 

      // output its report
      m_sBadVertexProps_CoinVertices.Dump() ;

      // which outputs one of the following two lines
      //   if(m_lBadEdgeCnt_DegenEdges == 0)
      //        "step:[HealerOp] Vertex->Vertex spacing too short: Coincident Vertex Pair Cnt:[OrigCnt, 0] - Okay"
      //   else "step:[HealerOp] Vertex->Vertex spacing too short: Coincident Vertex Pair Cnt:[OrigCnt, CurrCnt] - Bad"
***********************************************************************/
template<class TYPE> class SmPairProbArray : public SmProbArray<TYPE>
{
 public:
  // inherited from SmProbArray<TYPE>
  //  SmTArray<TYPE *>   m_sProbArray ;                   // problem array managed by one of the derived classes               
  //  ULONG          TYPE::*m_plObjProbFlag ;             // ptr to member ObjProbFlag. works with ULONGs and SmBooleans
  //                                                      //    ex: &SmEdgeProps::m_bBadSmallZoneTol3d
  //                                                      //    FALSE or 0         = orig prob m_pDataArray fixed
  //                                                      //    TRUE  or 1 or MORE = not yet fixed
  //  ULONG              m_lOrigProbCount ;               // Max number of problems found before any objs are deleted
  //  SmHealerOpType     m_eGetHealerOp ;                 // HealerOp where ProbArray is built
  //  
  //  TCHAR              m_sOutputLabel[SM_TBLOCK_SIZE] ; // Dump output label,                    ex: _T("Vertex->Vertex spacing too short"),
  //  TCHAR              m_sDataLabel[SM_TBLOCK_SIZE] ;   // Dump data label,                      ex: _T("Coincident Vertex Cnt"),
  //  TCHAR              m_sProbLabel[SM_TBLOCK_SIZE] ;   // for SmPairProbArray<TYPE> use std macro SM_PROBTYPE_BAD
                                                          //  ex: SM_PROBTYPE_BAD          = _T(" - Bad ")
                                                          //      SM_PROBTYPE_WARN         = _T(" - Warn ")
                                                          //      SM_PROBTYPE_CAN_BE_OKAY  = _T(" - Can Be Okay ")

  ULONG      m_lOrigCoinPairCnt ;             // orig number of coincident object pairs
             
 public:     
  // constructor - 
  SmPairProbArray(SmHealerOpType eGetHealerOp=SM_HO_NONE)  // in : Healer op that constructs this probArray
    : SmProbArray<TYPE>    (eGetHealerOp),
      m_lOrigCoinPairCnt(0)
    { }

  // copy constructor, assignment operator, virtual MakeCopy 
  SmPairProbArray(const SmPairProbArray & crOriginal) : SmProbArray<TYPE>(crOriginal) { m_lOrigCoinPairCnt = crOriginal.m_lOrigCoinPairCnt ; }
  SmPairProbArray & operator= (const SmPairProbArray &crOther ) { if(this == &crOther) return *this ;
                                                                  SmProbArray<TYPE>::operator=(crOther) ;
                                                                  m_lOrigCoinPairCnt   = crOther.m_lOrigCoinPairCnt ;
                                                                  return(*this) ;
                                                                }
  SmBoolean operator== (const SmPairProbArray &crOther ) { if(this == &crOther) return TRUE ;
                                                           SmBoolean bRtn = SmProbArray<TYPE>::operator==(crOther) ;
                                                           bRtn &= m_lOrigCoinPairCnt == crOther.m_lOrigCoinPairCnt ;
                                                           return(bRtn) ;
                                                         }
  virtual SmProbArray<TYPE> * MakeCopy() const { return( new SmPairProbArray(*this) ) ; }

  // ReSet
  virtual void ReSet() { SmProbArray<TYPE>::ReSet() ; 
                         m_lOrigCoinPairCnt = 0 ; 
                       }

  // simple data management
  // inherited from SmProbArray<TYPE>
  //  void Setup(SmHealerOpType eGetHealerOp,          // in : HealerOp where m_sProbArray is first built    ex: SM_HO_CACHE_EDGEPROPS
  //             SmBoolean      TYPE::*pbObjCoinFlag)  // in : ptr to member ObjCoinFlag. ex: &SmCVertexProps::m_bBadCoincidentVertex
  //                                                   //        value of ptr: FALSE = orig prob listed in m_pDataArray fixed
  //                                                   //                      TRUE  = not yet fixed

  // simple data access
  void SetOrigCoinPairCnt( ULONG lOrigCoinPairCnt)  { m_lOrigCoinPairCnt = lOrigCoinPairCnt ; }
  virtual SmBoolean Remove(TYPE * pTgtElement)      { // manage coin pairs
                                                      ULONG ii, lCurrCnt=0 ; SmBoolean bRemovedPair = FALSE, bRtn = FALSE ;
                                                      for(ii=0;ii<SmProbArray<TYPE>::m_sProbArray.GetSize()+(bRemovedPair?2:0);ii+=2)
                                                        {
                                                          if(bRemovedPair) {ii-=2 ; bRemovedPair=FALSE ; }
                                                          // when past the list of current problems - 
                                                          if(  (ii+1) > SmProbArray<TYPE>::m_sProbArray.GetSize()
                                                             || TRUE != SmProbArray<TYPE>::m_sProbArray.GetAt(ii  )->*SmProbArray<TYPE>::m_plObjProbFlag
                                                             || TRUE != SmProbArray<TYPE>::m_sProbArray.GetAt(ii+1)->*SmProbArray<TYPE>::m_plObjProbFlag)
                                                            { break ; }
                                                          lCurrCnt++ ;

                                                          // when this coincident pair contains the TgtElement
                                                          if(   SmProbArray<TYPE>::m_sProbArray.GetAt(ii  ) == pTgtElement
                                                             || SmProbArray<TYPE>::m_sProbArray.GetAt(ii+1) == pTgtElement)
                                                            { SmProbArray<TYPE>::m_sProbArray.RemoveAt(ii,2) ;
                                                              lCurrCnt--;
                                                              bRemovedPair = TRUE ; bRtn = TRUE ;
                                                            }
                                                        } // end iter every coin pair problem
                                                      // manage fixed single entries
                                                      for(ii=2*lCurrCnt;ii<SmProbArray<TYPE>::m_sProbArray.GetSize();ii++)
                                                        {
                                                          // when this is the TgtElement
                                                          if(SmProbArray<TYPE>::m_sProbArray.GetAt(ii) == pTgtElement)
                                                            { SmProbArray<TYPE>::m_sProbArray.RemoveAt(ii) ; 
                                                              bRtn = TRUE ;
                                                            }
                                                        } // end iter every coin pair problem
                                                      return(bRtn) ;
                                                    }

  // inherited from SmProbArray<TYPE>
  // void SetLabels(TCHAR * pOutputLabel, // in : Dump output label,                    ex: _T("Vertex->Vertex spacing too short"),
  //                TCHAR * pDataLabel,   // in : Dump data label,                      ex: _T("Coincident Vertex Pair Cnt"),
  //                TCHAR * pProbLabel) ; // in : for problem arrays EndOfStr Prob Text ex: SM_PROBTYPE_BAD) ;   // oneof: SM_PROBTYPE_BAD     
  //                                      //                                                                     //        SM_PROBTYPE_WARN       
  //                                      //                                                                     //        SM_PROBTYPE_CAN_BE_OKAY

  // pretty print 
  //  "step:[m_eGetHealerOp] m_sOutputLabel: m_sDataLabel:[m_sProbArray.GetSize(), lCurrCnt] - m_sProbLabel"
  //   example: if(m_sBadEdgeProps_SmallZoneTol3d == 0)
  //                 step:[HealerOp] Vertex->Vertex spacing too short: Coincident Vertex Pair Cnt:[OrigCnt, 0] - Okay
  //            else step:[HealerOp] Vertex->Vertex spacing too short: Coincident Vertex Pair Cnt:[OrigCnt, CurrCnt] - BAD
   ULONG Dump(SmHealerOpType eDoneHealerOp) const
    {
      // locals
      ULONG ii, lCurrCnt = 0 ;                                                 

      // when the heal sequence is far enough along
      if(eDoneHealerOp >= SmProbArray<TYPE>::m_eGetHealerOp)                                  
        {    
          TCHAR sBuff[SM_TBLOCK_SIZE] ;

          // count the coincident pairs                        
          for(ii=0; ii<SmProbArray<TYPE>::m_sProbArray.GetSize();ii+=2)                            
            { if(   (ii+1) < SmProbArray<TYPE>::m_sProbArray.GetSize()
                 && TRUE == SmProbArray<TYPE>::m_sProbArray.GetAt(ii  )->*SmProbArray<TYPE>::m_plObjProbFlag
                 && TRUE == SmProbArray<TYPE>::m_sProbArray.GetAt(ii+1)->*SmProbArray<TYPE>::m_plObjProbFlag)                    
                { lCurrCnt++ ; }                                                   
            }                                                                      
                                                                                
          // Output the orig and curr prob Cnts                               
          smos_sprintf(sBuff, _T("\n  step:[%2d] %.256s: %.256s:[%4lu, %4lu]%.256s"),        
                     SmProbArray<TYPE>::m_eGetHealerOp, SmProbArray<TYPE>::m_sOutputLabel, SmProbArray<TYPE>::m_sDataLabel,                    
                     m_lOrigCoinPairCnt, lCurrCnt,                     
                     (lCurrCnt == 0) ? _T(" - Okay") : SmProbArray<TYPE>::m_sProbLabel ) ;  
        }                                                                       
      // all done
      return(lCurrCnt) ;
    } // end SmPairProbArray<TYPE>::Dump

} ; // end template<class TYPE> class SmPairProbArray

#ifdef _WIN32
#define SM_COIN_PROB_ARRAY_TEMPLATE_PREDECLARATION(x) template class SM_EXPORT SmPairProbArray<x>
  SM_COIN_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmVertexProps) ;
  SM_COIN_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmEdgeProps) ;
  SM_COIN_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmLoopProps) ;
  SM_COIN_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmFaceProps) ;
#undef SM_COIN_PROB_ARRAY_TEMPLATE_PREDECLARATION
#endif // _WIN32

/*******************************************************************//**
PURPOSE: SmHealData MixedProbArrays

NOTES: A MixedProbArray is: 
         Loaded with all obj ObjProps that have a property that may or may not be a problem.
         Each ObjProp has a SmMixedLinkType enum flag marking the Obj as oneof 
                  SM_MP_UNDEF,  // obj propetry value not yet known
                  SM_MP_HASPROP,  // obj has property - not yet known if that's okay or a problem
                  SM_MP_OKAY,     // obj has property and is known to be okay
                  SM_MP_PROB,     // obj has property and is known to be problem needing fixing
                  SM_MP_FIXED,    // obj has property that was a problem and is now fixed
                  
         The length of the array is the count of objects with the property.
           Orig Prob Count = Count of all m_sProbArray list members where m_peObjProp == SM_MP_PROB or SM_MP_FIXED
           Curr Prob Count = Count of all m_sProbArray list members where m_peObjProp == SM_MP_PROB 
EXAMPLE USE: SmHealData::m_sFaceProps_Sheets with member Property enum SmFaceProps::&m_eSheetFace ;
             A bad sheet face has the same region on both sides while an analysis of the region shows that the region
             is really separated into two regions to be split by the bad sheet face which will turn the sheet face back
             into a manifold face and split the single region into a pair of child regions.

      // Setup ProbArray
      m_sFaceProps_Sheets.Setup(SM_HO_CACHE_FACEPROPS, SmFaceProps::&m_eSheet) ;
      m_sFaceProps_Sheets.SetLabels(_T("Sheet Face Counts"), // in : line label   
                                    _T(""),                  // in : not used
                                    _T("")) ;                // in : not used

      . . . 

      // run SM_HO_CACHE_EDGEPROPS heal step
      Cache_FaceProps(pBrep,SM_HO_CACHE_EDGEPROPS) 

      . . . 

      // output its report
      m_sFaceProps_Sheets.Dump() ;

      // which outputs one of the lines
      // step[x]:Sheet Face Counts: OrigCnt:[%4lu], UnTested[%4lu], AlwaysOkay:[%4lu] Bad:[%4lu] Fixed:[%4lu] - Bad                                                   
      // step[x]:Sheet Face Counts: OrigCnt:[%4lu], UnTested[%4lu], AlwaysOkay:[%4lu] Bad:[%4lu] Fixed:[%4lu] - Okay
      // step[x]:Sheet Face Counts: OrigCnt:[%4lu], UnTested[%4lu], AlwaysOkay:[%4lu] Bad:[%4lu] Fixed:[%4lu] - May be okay
***********************************************************************/
template<class TYPE> class SmMixedProbArray : public SmProbArray<TYPE>
{
 public:
  // inherited from SmProbArray<TYPE>
  //  SmTArray<TYPE *>   m_sProbArray ;                   // problem array managed by one of the derived classes               
  //  ULONG              TYPE::*m_plObjProbFlag ;         // not used - ptr to member ObjProbFlag.  
  //                                                      //   instead: use the enum m_peObjProp.
  //  ULONG              m_lOrigProbCount ;               // Max number of problems found before any objs are deleted
  //  SmHealerOpType     m_eGetHealerOp ;                 // HealerOp where ProbArray is built
  //  
  //  TCHAR              m_sOutputLabel[SM_TBLOCK_SIZE] ; // Used to label the property. ex: _T("Sheet Face Counts")
  //  TCHAR              m_sDataLabel[SM_TBLOCK_SIZE] ;   // not used
  //  TCHAR              m_sProbLabel[SM_TBLOCK_SIZE] ;   // not used

  SmMixedLinkType        TYPE::*m_peObjProp ;         // ptr to member ObjPropertyEnum. ex: &SmFaceProps::m_eSheet
                                                      //    oneof: SM_MP_UNDEF, // obj propetry value not yet known
                                                      //           SM_MP_HASPROP, // obj has property - not yet known if that's okay or a problem
                                                      //           SM_MP_OKAY,    // obj has property and is known to be okay
                                                      //           SM_MP_PROB,    // obj has property and is known to be problem needing fixing
                                                      //           SM_MP_FIXED,   // obj has property that was a problem and is now fixed
 public:
  // constructor - 
  SmMixedProbArray(SmHealerOpType eGetHealerOp=SM_HO_NONE)  // in : Healer op that constructs this probArray
    : SmProbArray<TYPE>(eGetHealerOp),
      m_peObjProp(NULL)
    { }

  // copy constructor, assignment operator, virtual MakeCopy 
  SmMixedProbArray(const SmMixedProbArray & crOriginal) : SmProbArray<TYPE>(crOriginal) { m_peObjProp = crOriginal.m_peObjProp ; }
  SmMixedProbArray & operator= (const SmMixedProbArray &crOther ) { if(this == &crOther) return *this ;
                                                                    SmProbArray<TYPE>::operator=(crOther) ;
                                                                    m_peObjProp = crOther.m_peObjProp ;
                                                                    return(*this) ;
                                                                  }
  SmBoolean operator== (const SmMixedProbArray &crOther ) { if(this == &crOther) return TRUE ;
                                                              SmBoolean bRtn = SmProbArray<TYPE>::operator==(crOther) ;
                                                              bRtn &= m_peObjProp == crOther.m_peObjProp ;
                                                              return(bRtn) ;
                                                            }
  virtual SmProbArray<TYPE> * MakeCopy() const { return( new SmMixedProbArray(*this) ) ; }

  // ReSet
  virtual void ReSet() { SmProbArray<TYPE>::ReSet() ; 
                         SmProbArray<TYPE>::m_lOrigProbCount = 0 ;
                       }

  // init boolean ProbFlag pointer
  void Setup
    (SmHealerOpType  eGetHealerOp,      // in : Healer op that constructs this probArray
     SmMixedLinkType TYPE::*peObjProp)  // in : ptr to member ObjMixedPropType, ex: &SmFaceProp::m_eSheet
                                        //        value of ptr: FALSE = orig prob listed in m_pDataArray fixed
                                        //                      TRUE  = not yet fixed
    { SmProbArray<TYPE>::m_eGetHealerOp = eGetHealerOp ;
      m_peObjProp    = peObjProp ; 
    } 

  // Add list members - set member link values
  ULONG Add(TYPE          * pNewElement,    // in : elem to add to list
            SmMixedLinkType eMixedLinkType) // oneof: SM_MP_UNDEF,   // propetry value not yet known
                                            //        SM_MP_NOPROP,  // obj does not have the property
                                            //        SM_MP_HASPROP, // obj has property - not yet known if that's okay or a problem
                                            //        SM_MP_OKAY,    // obj has property and is known to be okay
                                            //        SM_MP_PROB,    // obj has property and is known to be problem needing fixing
                                            //        SM_MP_FIXED,   // obj has property that was a problem and is now fixed
                                            { if(pNewElement == NULL) 
                                                { SM_ASSERT_ERR_MSG(_T("SmMixedProbArray::Add: Error - added a NULL prob to array - see if this is needed")) ; 
                                                  return SM_UNDEF_ULONG; }
                                              SmProbArray<TYPE>::m_lOrigProbCount = SmProbArray<TYPE>::m_sProbArray.GetSize() + 1 ;
                                              if(m_peObjProp != NULL) 
                                                { pNewElement->*m_peObjProp = eMixedLinkType ; }
                                              return(SmProbArray<TYPE>::m_sProbArray.Add(pNewElement)) ;
                                            }
                 
  // pretty print 
  //  "step:[m_eGetHealerOp] m_sOutputLabel: OrigCnt:[%4lu], CurrCnt:[%4lu], UnTested:[%4lu], AlwaysOkay:[%4lu] Bad:[%4lu, %4lu] Fixed:[%4lu]%s"
  ULONG Dump(SmHealerOpType eDoneHealerOp) const
    {
      ULONG ii, lHasPropCnt = 0, lOkayCnt = 0, lProbCnt = 0, lFixedCnt = 0, lUnknownCnt = 0 ;   
          
      // When heal sequence is far enough along                               
      if(eDoneHealerOp >= SmProbArray<TYPE>::m_eGetHealerOp)                                  
        {            
          // locals
          TCHAR sBuff[SM_TBLOCK_SIZE];

          // count the unfixed and fixed problems in this array                          
          for(ii=0; ii<SmProbArray<TYPE>::m_sProbArray.GetSize();ii++)                            
            { if(SM_MP_HASPROP == SmProbArray<TYPE>::m_sProbArray.GetAt(ii)->*m_peObjProp) { lHasPropCnt++ ; }
              if(SM_MP_OKAY    == SmProbArray<TYPE>::m_sProbArray.GetAt(ii)->*m_peObjProp) { lOkayCnt++ ; }
              if(SM_MP_PROB    == SmProbArray<TYPE>::m_sProbArray.GetAt(ii)->*m_peObjProp) { lProbCnt++ ; }
              if(SM_MP_FIXED   == SmProbArray<TYPE>::m_sProbArray.GetAt(ii)->*m_peObjProp) { lFixedCnt++ ; }                                                   
              if(SM_MP_UNDEF   == SmProbArray<TYPE>::m_sProbArray.GetAt(ii)->*m_peObjProp) { lUnknownCnt++ ; }  // marks objs that no longer have the prop due to a fix                                                 
            }                                                                      
                            
          smos_sprintf(sBuff, _T("\n  step:[%2d] %.512s: OrigCnt:[%4lu]"),        
                     SmProbArray<TYPE>::m_eGetHealerOp, SmProbArray<TYPE>::m_sOutputLabel,                   
                     SmProbArray<TYPE>::m_sProbArray.GetSize()) ;
          if(SmProbArray<TYPE>::m_sProbArray.GetSize() > 0)
            {
              smos_sprintf(sBuff, _T(", CurrCnt:[%4lu], UnTested[%4lu], AlwaysOkay:[%4lu] Bad:[%4lu, %4lu] Fixed:[%4lu]"),        
                           smos_Max(SmProbArray<TYPE>::m_lOrigProbCount, lHasPropCnt+lOkayCnt+lProbCnt), 
                           lHasPropCnt,
                           lOkayCnt, 
                           lProbCnt+lFixedCnt, 
                           lProbCnt, 
                           lFixedCnt) ;  
              smos_WriteBuffer(sBuff) ;
            }
          smos_sprintf(sBuff, _T("%s"),    
                      ((SmProbArray<TYPE>::m_sProbArray.GetSize() == 0) || (lProbCnt == 0 && lHasPropCnt == 0)) ? _T(" - Okay") 
                    : (lProbCnt > 0)                                                         ? _T(" - Bad") 
                    : (lHasPropCnt > 0)                                                      ? _T(" - May Be Okay")
                    :                                                                          _T(" - error in SmMixedProbArray<TYPE>::Dump Code")) ; 
          smos_WriteBuffer(sBuff) ;
        }                                                                       
      // all done
      return(lProbCnt) ;
    } // end SmMixedProbArray<TYPE>::Dump

} ; // end template<class TYPE> class SmMixedProbArray

#ifdef _WIN32
#define SM_MIXED_PROB_ARRAY_TEMPLATE_PREDECLARATION(x) template class SM_EXPORT SmMixedProbArray<x>
  SM_MIXED_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmVertexProps) ;
  SM_MIXED_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmEdgeProps) ;
  SM_MIXED_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmLoopProps) ;
  SM_MIXED_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmFaceProps) ;
#undef SM_MIXED_PROB_ARRAY_TEMPLATE_PREDECLARATION
#endif // _WIN32

/*******************************************************************//**
PURPOSE: SmHealData TriedProbArrays

NOTES: A TriedProbArray is: 
         Loaded with all obj ObjProbs that were run through a particular FIX method to fix a specified obj property.
         Each ObjProp has an enum flag marking itself as 
            SM_TRY_NONE,     // obj not yet run through targeted Fix function
            SM_TRY_FIXED,    // obj run through targeted fix function - checked property is fixed
            SM_TRY_NOFIX,    // obj run through targeted fix function - checked property is not fixed
          Total run Count = m_sProbArray.GetSize() ; // may have duplicate entries if a Fix is ever run twice on one object
          Fixed Count     = number of entries whose flag marks them as fixed by this fix call.
          NotFixed Count  = number of entries whose flag marks  them as unfixed by this fix call.
EXAMPLE USE: SmHealData::m_sFixFaceProps_MoveSeam
             with member ProbFlag SmFaceProps::&m_eBeenThroughMoveSeam ;

      // Setup ProbArray
      m_sFixFaceProps_MoveSeam.Setup(SM_HO_CACHE_FACEPROPS, 
                                     SmFaceProps::&m_eBeenThroughMoveSeam) ;
      m_sFixFaceProps_MoveSeam.SetLabels(_T("Faces sent through MoveSeam")) ;

      . . . 

      // run SM_HO_CACHE_EDGEPROPS heal step
      Cache_FaceProps(pBrep,SM_HO_CACHE_EDGEPROPS) 

      . . . 

      // output its report
      m_sFixFaceProps_MoveSeam.Dump() ;

      // which outputs the following line
      //   "step:[HealerOp] Faces sent through MoveSeam: Run [total, Fix, NoFix] Cnts:[totCnt, FixCnt, NoFixCnt] - Okay (informational only)"
***********************************************************************/
template<class TYPE> class SmTriedProbArray : public SmProbArray<TYPE>
{
 public:
  // inherited from SmProbArray<TYPE>
  //  SmTArray<TYPE *> m_sProbArray ;                   // problem array managed by one of the derived classes               
  //  ULONG            TYPE::*m_plObjProbFlag ;         // not used - ptr to member ObjProbFlag.  
  //                                                    //   instead: use the enum m_peBeenThroughFix.
  //  ULONG            m_lOrigProbCount ;               // Max number of problems found before any objs are deleted
  //  SmHealerOpType   m_eGetHealerOp ;                 // HealerOp where ProbArray is built
  //                   
  //  TCHAR            m_sOutputLabel[SM_TBLOCK_SIZE] ; // Dump output label.  ex: _T("Faces sent through Fix_MoveSeam")
  //  TCHAR            m_sDataLabel[SM_TBLOCK_SIZE] ;   // not used
  //  TCHAR            m_sProbLabel[SM_TBLOCK_SIZE] ;   // not used
                       
  SmTriedLinkType TYPE::*m_peBeenThroughFix ;  // Ptr to Obj member tracking BeenThroughFixFunction state
                                               // oneof: SM_TRY_NONE,     // obj not yet run through targeted Fix function
                                               //        SM_TRY_RAN,      // obj run through targeted fix function - not yet checked for fixed or broken
                                               //        SM_TRY_FIXED,    // obj run through targeted fix function - checked property is fixed
                                               //        SM_TRY_NOFIX,    // obj run through targeted fix function - checked property is not fixed

 public:
  // constructor - 
  SmTriedProbArray(SmHealerOpType eGetHealerOp=SM_HO_NONE)  // in : Healer op that constructs this probArray
    : SmProbArray<TYPE>    (eGetHealerOp),
      m_peBeenThroughFix(NULL)
    { }

  // copy constructor, assignment operator, virtual MakeCopy 
  SmTriedProbArray(const SmTriedProbArray & crOriginal) : SmProbArray<TYPE>(crOriginal) { m_peBeenThroughFix = crOriginal.m_peBeenThroughFix ; }
  SmTriedProbArray & operator= (const SmTriedProbArray &crOther ) { if(this == &crOther) return *this ;
                                                                    SmProbArray<TYPE>::operator=(crOther) ;
                                                                    m_peBeenThroughFix = crOther.m_peBeenThroughFix ;
                                                                    return(*this) ;
                                                                  }
  SmBoolean operator== (const SmTriedProbArray &crOther ) { if(this == &crOther) return TRUE ;
                                                              SmBoolean bRtn = SmProbArray<TYPE>::operator==(crOther) ;
                                                              bRtn &= m_peBeenThroughFix == crOther.m_peBeenThroughFix ;
                                                              return(bRtn) ;
                                                            }
  virtual SmProbArray<TYPE> * MakeCopy() const { return( new SmTriedProbArray(*this) ) ; }

  // virtual void ReSet() ; - inherited from SmProbArray<TYPE>

  // init boolan ProbFlag pointer
  void Setup
    (SmHealerOpType  eGetHealerOp,               // in : Healer op that constructs this probArray
     SmTriedLinkType TYPE::*peBeenThroughFix)    // in : ptr to member ObjBeenThroughFix. ex: &SmFaceProps::m_eBeenThroughMoveSeam
    { SmProbArray<TYPE>::m_eGetHealerOp     = eGetHealerOp ;
      m_peBeenThroughFix = peBeenThroughFix ;
    } 

  // inherited from SmProbArray<TYPE>
  // void SetLabels(TCHAR * pOutputLabel, // in : Dump output label,                     ex: _T("Faces sent through Fix_MoveSeam")
  //                TCHAR * pDataLabel,   // in : NOT USED
  //                TCHAR * pProbLabel) ; // in : NOT USED
                                          
  // Add list members - set member link values
  ULONG Add(TYPE          * pNewElement,    // in : elem to add to list
            SmTriedLinkType eTriedLinkType) // oneof: SM_TRY_NONE,  // obj not yet run through targeted Fix function
                                            //        SM_TRY_FIXED, // obj run through targeted fix function - checked property is fixed
                                            //        SM_TRY_NOFIX, // obj run through targeted fix function - checked property is not fixed
                                            { if(pNewElement == NULL) 
                                                { SM_ASSERT_ERR_MSG(_T("SmTriedProbArray::Add: Error - added a NULL prob to array - see if this is needed")) ; 
                                                  return SM_UNDEF_ULONG; }
                                              SmProbArray<TYPE>::m_lOrigProbCount = SmProbArray<TYPE>::m_sProbArray.GetSize() + 1 ;
                                              if(m_peBeenThroughFix != NULL) 
                                                { pNewElement->*m_peBeenThroughFix = eTriedLinkType ; }
                                              return(SmProbArray<TYPE>::m_sProbArray.Add(pNewElement)) ;
                                            }
  // pretty print 
  //   example: if(lNoFixCnt == 0) 
  //          step:[HealerOp] Faces sent through Fix_MoveSeam: TotalRuns:[%4lu], Fixed:[%4lu], NoFix:[%4lu] - Okay - all runs fixed objs           
  //     else step:[HealerOp] Faces sent through Fix_MoveSeam: TotalRuns:[%4lu], Fixed:[%4lu], NoFix:[%4lu] - Bad  - some runs failed to fix objs
  ULONG Dump(SmHealerOpType eDoneHealerOp) const
    {
      // locals
      ULONG ii, lFixedCnt = 0, lNoFixCnt = 0 ;    
      
      // When heal sequence is far enough along                               
      if(eDoneHealerOp >= SmProbArray<TYPE>::m_eGetHealerOp)                                  
        {            
          TCHAR sBuff[SM_TBLOCK_SIZE];

          // count the unfixed and fixed problems in this array                          
          for(ii=0; ii<SmProbArray<TYPE>::m_sProbArray.GetSize();ii++)                            
            { if(SM_TRY_NONE  == SmProbArray<TYPE>::m_sProbArray.GetAt(ii)->*m_peBeenThroughFix) { }
              if(SM_TRY_FIXED == SmProbArray<TYPE>::m_sProbArray.GetAt(ii)->*m_peBeenThroughFix) { lFixedCnt++ ; }
              if(SM_TRY_NOFIX == SmProbArray<TYPE>::m_sProbArray.GetAt(ii)->*m_peBeenThroughFix) { lNoFixCnt++ ; }
            }                                                                      
          
          // when any objs have been through fix
          if(lFixedCnt + lNoFixCnt > 0)
            {
              // step:[HealerOp] Faces sent through Fix_MoveSeam: TotalRuns:[%4lu], Fixed:[%4lu], NoFix:[%4lu] - Okay - all runs fixed objs
              // step:[HealerOp] Faces sent through Fix_MoveSeam: TotalRuns:[%4lu], Fixed:[%4lu], NoFix:[%4lu] - Bad  - some runs failed to fix objs
              smos_sprintf(sBuff, _T("\n  step:[%2d] %.256s: TotalRuns:[%4lu], Fixed:[%4lu], NoFix:[%4lu]%.256s"),        
                         SmProbArray<TYPE>::m_eGetHealerOp, SmProbArray<TYPE>::m_sOutputLabel, 
                         lFixedCnt + lNoFixCnt,
                         lFixedCnt,
                         lNoFixCnt,
                         (lNoFixCnt == 0) ? _T(" - Okay - all runs fixed objs") : _T(" - Bad  - some runs failed to fix objs")  ) ; 
              smos_WriteBuffer(sBuff) ;
              // SM_SPRINTF(sBuff, _T("\n  step:[%2d] %s: TotalRuns:[%4lu], Fixed:[%4lu], NoFix:[%4lu]%s"),        
              //            SmProbArray<TYPE>::m_eGetHealerOp, SmProbArray<TYPE>::m_sOutputLabel, 
              //            lFixedCnt + lNoFixCnt,
              //            lFixedCnt,
              //            lNoFixCnt,
              //            (lNoFixCnt == 0) ? _T(" - Okay - all runs fixed objs") : _T(" - Bad  - some runs failed to fix objs")  ) ; 
              // smos_WriteBuffer(sBuff) ;
            } // end some objs been through fix check
        } // end healer far enough along check                                                                     
      // all done
      return(lNoFixCnt) ;
    } // end SmTriedProbArray<TYPE>::Dump

} ; // end template<class TYPE> class SmTriedProbArray

#ifdef _WIN32
#define SM_TRIED_PROB_ARRAY_TEMPLATE_PREDECLARATION(x) template class SM_EXPORT SmTriedProbArray<x>
  SM_TRIED_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmVertexProps) ;
  SM_TRIED_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmEdgeProps) ;
  SM_TRIED_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmLoopProps) ;
  SM_TRIED_PROB_ARRAY_TEMPLATE_PREDECLARATION(SmFaceProps) ;
#undef SM_TRIED_PROB_ARRAY_TEMPLATE_PREDECLARATION
#endif // _WIN32

#endif // !__SMHEALPROBARRAY_H__
