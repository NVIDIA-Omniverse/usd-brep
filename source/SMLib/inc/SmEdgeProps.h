// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmEdgeProps.h
* PURPOSE: Header file for SmEdgeProps class.
**********************************************************************/

#ifndef __SMEDGEPROPS_H__
#define __SMEDGEPROPS_H__

//#pragma warning(disable : 4291)   // no matching operator delete found; // restored these warnings to clean up errors on linux

#include <SmTypes.h>
#include <SmTArray.h>
#include <SmHealData.h>
#include <SmObjProps.h>

class SmEdgeEdgeGap ;
class SmEdgeFaceGap ;

/*******************************************************************//**
PURPOSE: Container class for common Edge and Edge AssertValid property values.

NOTES: 1. Initial use: To pass informtion around the Heal Methods
          called by the HealBrep() method.
***********************************************************************/
class SM_EXPORT SmEdgeProps : public SmObjProps
{
 public:
  // context pointer (not owned)
  const SmContext * m_cpContext           = NULL ;            // set : [constructor()]
                                                              
  // Edge and Tolerance ('Size Members')                      
  SmEdge          * m_pEdge               = NULL ;            // set : [SetProps_Metadata()]
  ULONG             m_lEdgeIndx           = SM_UNDEF_ULONG ;  // set : [SetProps_Metadata] associated EdgeIndx within managing SmHeadData::m_sTgtEdges list - (constant between debug runs)

  SmExtent1d        m_sEdgeInterval ;                         // set : [SetProps_Metadata()]
  SmZoneTol3d       m_sZoneTol3d          = SM_UNDEF_DOUBLE ; // set : [SetProps_Metadata() && SmHeal::Fix_Tolerances()]
  SmZoneTol3d       m_sOrigZoneTol3d      = SM_UNDEF_DOUBLE ; // set : [SetProps_Metadata()]
  double            m_dApproxEdgeLength3d = SM_UNDEF_DOUBLE ; // set : [SetProps_Metadata()]

  // Edge->Curve    ('Size Members')
  const SmCurve   * m_cpCurve             = NULL ;            // set : [SetProps_Metadata()]
  SmExtent1d        m_sNaturalInterval ;                      // set : [SetProps_Metadata()]

  // Gap3ds         ('Gap Members')
  SmEdgeEdgeGap   * m_pMaxGap3d_EdgeEdge  = NULL ;            // set : [SetProps_Gaps()]  Max EdgeEnd/CCWEdgeEnd Gap3d (think loops) - NULL for ShellEdges
  SmEdgeFaceGap   * m_pMaxGap3d_EdgeFace  = NULL ;            // set : [SetProps_Gaps()]  max Edge/Face Gap3d - NULL for ShellEdges

  // Problems
  SmBooleanUL       m_bBadSmallZoneTol3d  = UNSURE ;  // link: SmHealData::m_sBadEdgeProps_SmallZoneTol3d (note: SmBooleanUL = booleans declared as ULONG)
                                                      // set : [SetProps_Gaps()]
                                                      // use : TRUE = stored ZoneTol3d < Calc ZoneTol3d(MaxGap3ds)
  SmBooleanUL       m_bBadLargeZoneTol3d  = UNSURE ;  // link: SmHealData::m_sBadEdgeProps_LargeZoneTol3d (note: SmBooleanUL = booleans declared as ULONG)
                                                      // set : [SetProps_Gaps()]
                                                      // use : TRUE = stored ZoneTol3d > Calc ZoneTol3d(MaxGap3ds)
  SmBooleanUL       m_bWarnLargeGap3d     = UNSURE ;  // link: SmHealData::m_sWarnEdgeProps_LargeGap3d (note: SmBooleanUL = booleans declared as ULONG)
                                                      // set : [SetProps_Gaps()]
                                                      // use : TRUE = any Edge/Edge or Edge/Face Gap3d > DefZoneTol3d
                 
  // Major Edge->Curve properties that force the Edge to be checked for problems and healing 
  SmBooleanUL       m_bBadDegenEdge       = UNSURE ;  // link: SmHealData::m_sBadEdgeProps_DegenEdges (note: SmBooleanUL = booleans declared as ULONG)
                                                      // set : [SetProps_Metadata()]
                                                      // use : TRUE = Edge Length < Edge Tolerance

  // Closed pEdge/Curve classifications
  SmBooleanUL       m_bBadUncontainedEdge = UNSURE ;  // link: SmHealData::m_sBadEdgeProps_Uncontained (note: SmBooleanUL = booleans declared as ULONG)
                                                      // set : [SetProps_Metadata()]
                                                      // use : TRUE = all or some Edge->Interval is uncontained by Curve->NaturalInterval

  SmBooleanUL       m_bBadCoincidentEdge  = UNSURE ;  // link: SmHealData::m_sBadEdgeProps_CoinEdges (note: SmBooleanUL = booleans declared as ULONG)
                                                      // link: SmHealData::m_sFixedEdgeProps_CoinEdges (note: SmBooleanUL = booleans declared as ULONG)
                                                      // set: [SmHealData::Cache_CoinVertices()] 

  SmBooleanUL       m_bBadMissedEdgeXSect = UNSURE ;  // link: SmHealData::m_sBadEdgeProps_MissedEdgeXSects (note: SmBooleanUL = booleans declared as ULONG)
                                                      // link: SmHealData::m_sFixedEdgeProps_MissedEdgeXSects (note: SmBooleanUL = booleans declared as ULONG)
                                                      // set: [SmHealData::Cache_MissedEdgeXSects()]

 public:                                                     

  // constructors and destructor
  SmEdgeProps(const SmContext * cpContext) ;                              
  SmEdgeProps(SmEdge * pEdge, ULONG lEdgeIndx, const SmContext * cpContext) ;
  SmEdgeProps(const SmEdgeProps & crOther) ;
  virtual ~SmEdgeProps() { ReSet(); m_cpContext = NULL; }

  // deep copy operator
  SmStatus Copy(SmEdgeProps *& rpNewEdgeProps) { rpNewEdgeProps = new SmEdgeProps( *this ); NER( rpNewEdgeProps );
                                                 return SM_SUCCESS;
                                               }

  // assignment and equality operators
  SmEdgeProps & operator= (const SmEdgeProps &crOther) ;
  SmBoolean     operator==(const SmEdgeProps &crOther) const ;
  void          SetContext(const SmContext * cpContext) { m_cpContext = cpContext ; }

 public:
  
  // set all members to init values
  void ReSet(SmBoolean bResetEdge_Metadata=TRUE) ; // in : default:[TRUE] = reset both SetProps_Gaps() and SetProps_Metadata() properties
                                                   //      FALSE          = reset only SetProps_Gaps() properties

  // Evaluate and Set all internal member properties for this given Edge Before Edges are split at MissingSeams
  //                  m_pMaxGap3d_EdgeEdge m_bBadSmallZoneTol3d  m_bWarnLargeGap3d
  //                  m_pMaxGap3d_EdgeFace m_bBadLargeZoneTol3d  m_bBadUncontainedEdge
  //            calls helper function SetProps_MetaData()
  SmStatus SetProps
  (
    SmEdge          * cpEdge,                           ///< [in] : tgt Edge                                                                                    <br>
    ULONG             lEdgeIndx,                        ///< NotUsed: [in] : associated indx of cpEdge in managing SmHealData::m_sTgtEdges list <br>
    SmTArray<ULONG> * pOptLoopGap3d_Histogram=NULL,     ///< [in,out]: accumulating Loop EdgeuseEnd/EdgeuseEnd Gap3d histogram                                  <br>
    SmTArray<ULONG> * pOptEdgeFaceGap3d_Histogram=NULL, ///< [in,out]: accumulating Edge/Face Gap3d histogram                                                   <br>
    ULONG             lOptLabel=SM_UNDEF_ULONG          ///< [in] : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]  <br>
  );

  // SetProps helper function - Evaluate and Set only the m_pBadDegenEdge member
  //                  m_pEdge         m_lEdgeIndx,
  //                  m_sEdgeInterval m_dApproxEdgeLength3d
  //                  m_sZoneTol3d    m_sOrigZoneTol3d            
  //                  m_cpCurve       m_sNaturalInterval      
  //                  m_bBadDegenEdge m_bBadUncontainedEdge
  SmStatus SetProps_Metadata(SmEdge * cpEdge,                     // in :
                             ULONG    lEdgeIndx,                  // in :
                             ULONG    lOptLabel=SM_UNDEF_ULONG) ; // NotUsed: in :

  SmBoolean IsSet_Metadata() const ; // rtn: TRUE=Obj has no uninit size member values
  SmBoolean IsSet_Gaps()  const ; // rtn: TRUE=Obj has no uninit gap member values
  SmBoolean IsSet()       const { return( IsSet_Metadata() && IsSet_Gaps() ) ; }
  SmBoolean HasProblems() const ; // rtn: TRUE=Obj has any problems

  // draw function
  SmDisplayList * Draw
  (
    SmBoolean bDrawAllCases            = FALSE,         ///< [in] : TRUE = output graphics for all Edges (with or without probs)                    <br>
                                                        ///<      : default:[FALSE] = output graphics only for Edges with probs                     <br>
    SmVector3d sHighlightColor         ={ 1, 0, 0},     ///< [in] : def:[ 1, 0, 0]: If EdgeHasProblems Draw EdgeUV for highlight                    <br>
    SmVector3d sBadSmallZoneTol3dColor ={ 0, 1, 1},     ///< [in] : def:[ 0, 1, 1]: Vertices with too small ZoneTol3d Highlight Color               <br>
    SmVector3d sBadLargeZoneTol3dColor ={ 1, 0, 1},     ///< [in] : def:[ 1, 0, 1]: Vertices with too large ZoneTol3d Highlight Color               <br>
    SmVector3d sWarnLargeGap3dColor    ={ 1,.5,.1},     ///< [in] : def:[ 1,.5,.1]: Vertices with Gaps larger than DefZoneTol3d Highlight Color     <br>
    SmVector3d sBadDegenColor          ={ 0, 1, 1},     ///< [in] : def:[ 0, 1, 1]: Edges mislabled as Sheets splitting their Shell Highlight Color <br>  
    SmVector3d sBadUncontainedColor    ={ 1, 0, 1},     ///< [in] : def:[ 1, 0, 1]: Loops Crossing Seams Highlight Color                            <br>
    SmVector3d sBadCoincidentColor     ={.5, 1,.5},     ///< [in] : def:[.5, 1,.5]: Edges coincident with other edges
    SmVector3d sBadMissedXSectColor    ={.2,.5,.8},     ///< [in] : def:[.2,.5,.8]: Edges with EdgeXSects missing vertices
    SmGfxArraySet * pOptGfxSet=NULL                     ///< [in] : When given, output GfxVertexArrays not GL calls.                                <br>
  ) const ;
                      
  // pretty print
  void Dump(ULONG iLabel) const ;       // in : iLabel = SM_UNDEF_ULONG to ignore

  // Common label methods and void Dump() const ;
  SM_COMMON(SmEdgeProps, SmObjProps, SmEdgeProps_TYPE) ;

} ; // end class SmEdgeProps

// add a SmTArray<SmLoopProps> template to the dll interface
// SM_TARRAY_TEMPLATE_PREDECLARATION(SmEdgeProps) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmEdgeProps*) ;

#endif // !__SMEDGEPROPS_H__
