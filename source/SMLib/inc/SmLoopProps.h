// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmLoopProps.h
* PURPOSE: Header file for SmLoopProps and SmLooptreeItem classes.
**********************************************************************/

#ifndef __SMLOOPPROPS_H__
#define __SMLOOPPROPS_H__

//#pragma warning(disable : 4291)   // no matching operator delete found; // restored these warnings to clean up errors on linux

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif // no __SMOS_TYPES_H__ 

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif // no __SMTARRAY_H__

#ifndef __SMHEALDATA_H__
#include <SmHealData.h>
#endif // no __SMHEALDATA_H__

#ifndef __SMOBJPROPS_H__
#include <SmObjProps.h>
#endif // no __SMOBJPROPS_H__

class SmFace ; 
class SmSurface ; 
class SmCurveClassification ;
class SmLoopProps ;
class SmLooptreeItem ;
class SmClassifyLoopIO ;
class SmFaceProps ;
class SmLoop ; 

/*******************************************************************//**
PURPOSE: holder class for ClassifyLoops I/O

NOTES: Initial use: Loop classification in SmFace::ClassifyLoops
***********************************************************************/
class SM_EXPORT SmClassifyLoopIO
{
 public:
  SmLoop           * m_pLoop=NULL ;                      // Classified Loop                           
  SM_TYPE            m_tLoopuseType=16999 ;              // type of loop: SmVertexuse_TYPE (16006)    
                                                         //               SmEdgeuse_TYPE   (16005)    
                                                         //      default:[SmUnknown_TYPE   (16999)]   
  SmOrientType       m_eOrient=SM_OT_UNKNOWN ;           // EdgeuseLoops only
                                                         //  SM_OT_SAME     = CCW (expected Area_OuterLoop dir (for NoArea_Loops SM_OT_UPPERDOMAIN))
                                                         //  SM_OT_OPPOSITE = CW  (expected Area_InnerLoop dir (for NoArea_Loops SM_OT_LOWERDOMAIN))
  SmBoolean          m_bDegen_Loop=UNSURE ;              // NotYetDone - TRUE = Loop has degenerate size in 3d space
  SmBoolean          m_bArea_Loop=UNSURE ;               // EdgeuseLoops and VertexLoops
                                                         //  TRUE = Loop is Area_Loop (normal case)                              
                                                         //  FALSE= Loop is NoArea_Loop - a problem only seen on MissingSeamFaces
  SmContainmentType  m_eContainmentType=SM_CMT_UNKNOWN ; // EdgeuseLoops and VertexLoops
                                                         //  Area_Loop   : SM_CMT_OUTERLOOP,       // Area_Loop not contained by any other loops, contains all UVPoints 'inside' loop  
                                                         //                SM_CMT_INNERLOOP,       // Area_Loop contained by another, excludes all UVPoints 'inside' loop
                                                         //                SM_CMT_NESTEDLOOP_EVEN, // Area_Loop (invalid) nested in an InnerLoop with an even nesting depth (0,2,4) - healed to outer loops
                                                         //                SM_CMT_NESTEDLOOP_ODD,  // Area_Loop (invalid) nested in an InnerLoop with an odd nesting depth (1,3,5) - healed to inner loops
                                                         //  NoArea_Loop : SM_CMT_BOTLOOP,         // NoArea_Loop (invalid) contains all UVPoints in its upperdomain
                                                         //                SM_CMT_TOPLOOP,         // NoArea_Loop (invalid) contains all UVPoints in its lowerdomain
                                                         //  Wire_Loop   : SM_CMT_WIRELOOP,        // A set of conneted Edgeuses without a closed portion.

   SmBoolean operator==(const SmClassifyLoopIO &crOther) const { return(   m_pLoop            == crOther.m_pLoop
                                                                        && m_tLoopuseType     == crOther.m_tLoopuseType
                                                                        && m_eOrient          == crOther.m_eOrient         
                                                                        && m_bDegen_Loop      == crOther.m_bDegen_Loop
                                                                        && m_bArea_Loop       == crOther.m_bArea_Loop      
                                                                        && m_eContainmentType == crOther.m_eContainmentType) ;
                                                               }
  // pretty print
  void Dump(ULONG iLabel=SM_UNDEF_ULONG) const ;

} ; // end class SmClassifyLoopIO

// add a SmTArray<SmLoopProps> template to the dll interface
SM_TARRAY_TEMPLATE_PREDECLARATION(SmClassifyLoopIO) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmClassifyLoopIO*) ;

/*******************************************************************//**
PURPOSE: Container class for Loop properties that get used repeatedly
 by the Heal methods called by the HealBrep() method

NOTES: currently SmLoopProps::SetProps() is a helper function 
       only called once per Face->Loop from SmFaceProps::SetProps_Stage3()
***********************************************************************/
class SM_EXPORT SmLoopProps : public SmObjProps
{ 
 public:
  // context pointer (not owned)
  const SmContext     * m_cpContext ;                           // set : [constructor()]
  
  // Properties set in SmLoopProps::SetProps
  const SmLoop        * m_cpLoop             = NULL ;           // set : [SetProps()]
  SM_TYPE               m_tLoopuseType       = 16999 ;          // set : [SetProps()] type of loop: 
                                                                //               SmVertexuse_TYPE (16006)    
                                                                //               SmEdgeuse_TYPE   (16005)    
                                                                //      default:[SmUnknown_TYPE   (16999)]
  SmBoolean             m_bVertexLoopOnPole  = UNSURE ;         // set : SmVertexuse_TYPE only - [SetProps()->SmLoop->                                                              
  ULONG                 m_lEdgeCnt           = SM_UNDEF_ULONG ; // set : [SetProps()->SmLoop::GetEdges()]                                          
  ULONG                 m_lEdgeuseCnt        = SM_UNDEF_ULONG ; // set : [SetProps()->SmLoop::GetEdgeuses()]                                          
  SmBoolean             m_bClosed3d          = UNSURE ;         // set : [SetProps()->SmLoop::IsClosed()]
  SmBoolean             m_bClosedPtrs        = UNSURE ;         // set : [SetProps()->SmLoop::IsClosed()]
  SmBoolean             m_bGoodClosed        = UNSURE ;         // set : [SetProps()]->bGoodClosed = bClosed3d == bClosedPtrs

  ULONG                 m_lSeamCrossingCntU  = SM_UNDEF_ULONG ; // set : [SetProps() from SmFaceProps::m_pCrvClassU]
  ULONG                 m_lSeamWindingCntU   = SM_UNDEF_ULONG ; // set : [SetProps() from SmFaceProps::m_pCrvClassU]
  SmOrientType          m_eWindingOrientU    = SM_OT_UNKNOWN ;  // set : [SetProps() from SmFaceProps::m_pCrvClassU]
                                                                //        SM_OT_UNKNOWN = For Loops that don't cross seams

  ULONG                 m_lSeamCrossingCntV  = SM_UNDEF_ULONG ; // set : [SetProps() from SmFaceProps::m_pCrvClassV]
  ULONG                 m_lSeamWindingCntV   = SM_UNDEF_ULONG ; // set : [SetProps() from SmFaceProps::m_pCrvClassV]
  SmOrientType          m_eWindingOrientV    = SM_OT_UNKNOWN ;  // set : [SetProps() from SmFaceProps::m_pCrvClassV]
                                                                //        SM_OT_UNKNOWN = For Loops that don't cross seams

  SmBoolean             m_bDegen_Loop        = UNSURE ;         // set : NOT YET [SetProps() from SmFaceProps::m_pCrvClassU and m_pCrvClassV]
  SmBoolean             m_bArea_Loop         = UNSURE ;         // set : [SetProps() from SmFaceProps::m_pCrvClassU and m_pCrvClassV]
                                                                //       [checked against SmFace::ClassifyLoops() value - for lamina and singly periodic surfaces]
  
  // Properties set later in SmFaceProps::SetProps_Stage3()
  //   Because - calc Containment uses ClassifyCrvs that transect the Face Domain XSecting many Loops at once,
  //             calc orient comes from those ClassifyCrv/Loop XSections
  SmContainmentType     m_eContainmentType   = SM_CMT_UNKNOWN ; // set : [SmFaceProps::SetProps_Stage3() -> SmFace::ClassifyLoops()]
                                                                // use : oneof: SM_CMT_OUTERLOOP,       // Area_Loop not contained by any other loops, contains all UVPoints 'inside' loop  
                                                                //     :        SM_CMT_INNERLOOP,       // Area_Loop contained by another, excludes all UVPoints 'inside' loop
                                                                //     :        SM_CMT_NESTEDLOOP_EVEN, // Area_Loop (invalid) nested in an InnerLoop with an even nesting depth (0,2,4) - healed to outer loops
                                                                //     :        SM_CMT_NESTEDLOOP_ODD,  // Area_Loop (invalid) nested in an InnerLoop with an odd nesting depth (1,3,5) - healed to inner loops
                                                                //     :        SM_CMT_BOTLOOP,         // NoArea_Loop (invalid) contains all UVPoints in its upperdomain
                                                                //     :        SM_CMT_TOPLOOP,         // NoArea_Loop (invalid) contains all UVPoints in its lowerdomain
                                                                //     :        SM_CMT_WIRELOOP,        // A set of conneted Edgeuses without a closed portion.
  SmOrientType          m_eLoopOrient        = SM_OT_UNKNOWN ;  // set : [SmFaceProps::SetProps_Stage3() -> SmFace::ClassifyLoops()]
  SmOrientType          m_eDesiredLoopOrient = SM_OT_UNKNOWN ;  // set : [SmFaceProps::SetProps_Stage3() -> SmFace::ClassifyLoops()]
  SmBoolean             m_bGoodOrient        = UNSURE ;         // set : [SmFaceProps::SetProps_Stage3() -> SmFace::ClassifyLoops()]

 public:                
  // constructor and destructor
  SmLoopProps(const SmContext * cpContext) ;
  SmLoopProps(const SmLoop *cpLoop, const SmContext * cpContext) ;
  SmLoopProps(const SmLoopProps & crOther) ;
  virtual ~SmLoopProps() { ReSet() ; m_cpContext = NULL ; }

  // to use SmTArray<SmLoopProps> must support operator=, operator==
  // assignment and equality operators
  SmLoopProps & operator= (const SmLoopProps &crOther) ;
  SmBoolean     operator==(const SmLoopProps &crOther) const;

  // clear all member values (both those set in SmLoopProps::SetProps() and SmFaceProps::SetProps_Stage3()]
  void ReSet() ;

  // member values
  void SetContext(const SmContext * cpContext) { m_cpContext = cpContext; }

  // set SmLoopProps::m_cpLoop *              m_tLoopuseType *
  //                ::m_lEdgeCnt *            m_lEdgeuseCnt *
  //                ::m_bClosed3d *           m_bClosedPtrs *
  //                ::m_bGoodClosed *         
  //                ::m_lSeamCrossingCntU *   m_lSeamCrossingCntV *
  //                ::m_lSeamWindingCntU *    m_lSeamWindingCntV *
  //                ::m_eWindingOrientU *     m_eWindingOrientV *  - set to SM_OT_UNKNOWN for Loops that don't cross seams
  //                ::m_bDegen_Loop * - NotYetDone       
  //                ::m_bArea_Loop *          
  //                ::m_eContainmentType ##   m_eLoopOrient ##        
  //                ::m_eDesiredLoopOrient ## m_bGoodOrient ##   
  // currently only called from SmFaceProps::SetProps_Stage3()
  //   where *  = properties set in SmLoopProps::SetProps()
  //         ## = properties set later in SmFaceProps::SetProps_Stage3()
  SmStatus SetProps(const SmLoop * cpLoop,              ///< [in] : tgt Loop                                                           <br>
                    SmFaceProps  & rFaceProps) ;        ///< [in] : Owner Face's FaceProps processed with SetProps_BeforeEdgeSplits()  <br>

  SmBoolean HasProblems() const ; // rtn: TRUE=Obj has any problems
  
  // note: The method SmFaceProps::SetProps_Stage3() sets the remaining
  //       members SmLoopProps::m_eContainmentType
  //                          ::m_eLoopOrient
  //                          ::m_eDesiredLoopOrient
  //                          ::m_bGoodOrient
  //       whose values are computed by a call to SmFace::ClassifyLoops()

  // draw function
  SmDisplayList * Draw
  (
    SmVector3d sBadOrientColor      ={.3,.3,.3},  ///< [in] : def:[.3,.3,.3]: Loop actual and desired orientations differ                <br>
    SmVector3d sBadDegen_LoopColor  ={1.,.5, 0},  ///< [in] : def:[1.,.5, 0]: Loop has at least one degenerate dimension                 <br>
    SmVector3d sBadNoArea_LoopColor ={ 0, 0, 0},  ///< [in] : def:[ 0, 0, 0]: Loop is a NoArea_Loop                                      <br>
    SmVector3d sBadContainmentColor ={.8,.2,.7},  ///< [in] : def:[.8,.2,.7]: Loop is a Nested Loop (i.e. inside an InnerLoop)           <br>
    SmVector3d sBadCrossingSeamColor={.2,.7,.5},  ///< [in] : def:[.2,.7,.5]: Loop crosses a Seam                                        <br>
    SmVector3d sBadClosed_LoopColor ={.4,.8,.2},  ///< [in] : def:[.4,.8,.2]: ClosedLoop3d != ClosedLoopPtrs (assumed intent is closed)  <br>
    SmGfxArraySet * pOptGfxSet=NULL               ///< [in] : When given, output GfxVertexArrays not GL calls.                           <br>
  ) const ; 

  // pretty print
  void Dump(ULONG lLabel=SM_UNDEF_ULONG) const ;

  // Common label methods and void Dump() const ;
  SM_COMMON(SmLoopProps, SmObjProps, SmLoopProps_TYPE) ;

} ; // end class SmLoopProps

// add a SmTArray<SmLoopProps> template to the dll interface
SM_TARRAY_TEMPLATE_PREDECLARATION(SmLoopProps) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmLoopProps*) ;

/*******************************************************************//**
PURPOSE: Looptree item objects

NOTES: Scratch class Used by healer to represent all the 
       containment relationships between loops
***********************************************************************/
class SM_EXPORT SmLooptreeItem 
{
 public:
  // all loops
  SmBoolean          m_bPoleVertex            = FALSE ;  // TRUE=Loop is a PoleVertex, FALSE=isn't
  SmLoop           * m_pLoop                  = NULL ;   // This Item's Loop, NULL= uninit when (bPoleVertex==FALSE) or PoleVertex When(m_bPoleVertex == TRUE) 
  SmLoopProps      * m_pLoopProps             = NULL ;   // Loop's LoopProps, NULL= uninit when (bPoleVertex==FALSE) or PoleVertex When(m_bPoleVertex == TRUE) 
  SmClassifyLoopIO * m_pClassifyLoopIO        = NULL ;   // scratch work area for building LoopProps,
                                                         // initial use in SmFace::ClassifyLoops() where Looptrees are built

  // The paired NoArea_Loop when NoArea_Loops get paired
  SmBoolean          m_bPartnerPoleVertex     = FALSE ;  // TRUE=PartnerLoop is a PoleVertex, FALSE=isn't
  SmLoop           * m_pPartnerLoop           = NULL ;   // ThisLoop's Paired NoArea_Loop, NULL= NotUsed when (bPoleVertex==FALSE) or PoleVertex When(m_bPoleVertex == TRUE)
  SmLoopProps      * m_pPartnerLoopProps      = NULL ;   // Paired Loop's LoopProps,       NULL= NotUsed when (bPoleVertex==FALSE) or PoleVertex When(m_bPoleVertex == TRUE)
  SmClassifyLoopIO * m_pPartnerClassifyLoopIO = NULL ;   // initial use in SmFace::ClassifyLoops() where Looptrees are built

  SmLooptreeItem   * m_pSibling = NULL ;    
  SmLooptreeItem   * m_pChild   = NULL ;

 public:
  // empty constructor, Area_Loop, 1st of NoArea_Loop pair (See AddPartner()), and NoArea_Loop pair constructors
  SmLooptreeItem() { ReSetNoRecursion() ; }

  SmLooptreeItem
  (
    SmLoop           * pLoop,             
    SmLoopProps      * pLoopProps,
    SmClassifyLoopIO * pClassifyLoopIO
  ) ;

  SmLooptreeItem
  (
    SmLoop           * pLoop,                    ///< [in] : LowerParam NoArea_Loop or NULL=VertexPole     <br>
    SmLoopProps      * pLoopProps,               ///< [in] : associated LoopProps   or NULL=VertexPole     <br>
    SmClassifyLoopIO * pClassifyLoopIO,          ///< [in] : associated ClassifyLoopIO or NULL=VertexPole  <br>
    SmLoop           * pPartnerLoop,             ///< [in] : UpperParam NoArea_Loop or NULL=VertexPole     <br>
    SmLoopProps      * pPartnerLoopProps,        ///< [in] : associated LoopProps   or NULL=VertexPole     <br>
    SmClassifyLoopIO * pPartnerClassifyLoopIO    ///< [in] : associated ClassifyLoopIO or NULL=VertexPole  <br>
  );

  // deep copy construtor - this Node and descendants
  SmLooptreeItem(const SmLooptreeItem & crOther) ;

  // operator= and operator== this Node only. Needed to support SmTArray<SmLooptreeItem>.
  SmLooptreeItem & operator= (const SmLooptreeItem &crOther) ;
  SmBoolean        operator==(const SmLooptreeItem &crOther) const ;

  // init all member values to uninit values - this Node only
  void ReSetNoRecursion() ;

  // destructor - delete this Node and all its siblings and descendants
 ~SmLooptreeItem() ;

public:

  // predicates
  SmBoolean        IsSet()                                { return( m_pLoop != NULL || m_bPoleVertex == TRUE) ; }
  SmBoolean        IsNoArea_Pair()                        { return( m_pPartnerLoop != NULL || m_bPartnerPoleVertex == TRUE) ; }

  // data access
  SmContainmentType GetContainmentType()        const
  {
    return(m_pLoopProps ? m_pLoopProps->m_eContainmentType
                        : m_pClassifyLoopIO ? m_pClassifyLoopIO->m_eContainmentType
                        : SM_CMT_UNKNOWN);
  }

  SmContainmentType GetPartnerContainmentType() const
  {
    return(m_pPartnerLoopProps ? m_pPartnerLoopProps->m_eContainmentType
                               : m_pPartnerClassifyLoopIO ? m_pPartnerClassifyLoopIO->m_eContainmentType
                               : SM_CMT_UNKNOWN);
  }

  // build a copied flat list of this and all its descendants
  void GetDescendants
  (
    SmTArray<SmLooptreeItem> & rDescendants, 
    SmTArray<SmLoop *>       & rLoopDescendants, 
    SmBoolean                  bReSet=TRUE
  ) const ;

  void Set
  (
    SmLoop           * pLoop,
    SmLoopProps      * pLoopProps,
    SmClassifyLoopIO * pClassifyLoopIO
  )
  {
    ReSetNoRecursion();
    m_bPoleVertex = (pLoop == NULL);
    m_pLoop = pLoop;
    m_pLoopProps = pLoopProps;
    m_pClassifyLoopIO = pClassifyLoopIO;
  }
                            
  SmLooptreeItem * FindItem  
  (
    const SmLoop    * pLoop,     ///< [in] : TgtLoop or NULL=Look for VertexPole Loop 
    SmLooptreeItem *& rpParent   ///< [out]: If FoundItem, Parent of FoundItem or NULL=Item is TreeRoot 
  );

  // Tree Building
  // Add NewChild to existing 'this' parent's ChildList
  void AddChild( SmLooptreeItem  * pNewChild )
  { 
    pNewChild->m_pSibling = this->m_pChild;
    this->m_pChild = pNewChild;
  }

  // Add NewSibling to existing 'this' sibling's SiblingList
  void AddSibling( SmLooptreeItem  * pNewSibling )
  { 
    pNewSibling->m_pSibling = this->m_pSibling;
    this->m_pSibling = pNewSibling;
  }

  // Add NewSibling to existing 'this' sibling's SiblingList 
  void AddPartner
  (
    SmLoop           * pPartnerLoop,    ///< [in] : UpperParam NoArea_Loop    or NULL=VertexPole
    SmLoopProps      * pPartnerProps,   ///< [in] : associated LoopProps      or NULL=VertexPole
    SmClassifyLoopIO * pPartnerIO       ///< [in] : associated ClassifyLoopIO or NULL=VertexPole
  )
  { // Add Partner data to Existing 'this' object
    m_bPartnerPoleVertex = (pPartnerLoop == NULL);
    m_pPartnerLoop = pPartnerLoop;
    m_pPartnerLoopProps = pPartnerProps;
    m_pPartnerClassifyLoopIO = pPartnerIO;
  }

  void Dump(ULONG lGen=0, SmLooptreeItem * pParent=NULL) ; // in : 0 = Root, 1 = Child, 2 = GrandChildren, . . .

} ; // end class SmLooptreeItem

// add a SmTArray<SmLoopProps> template to the dll interface
SM_TARRAY_TEMPLATE_PREDECLARATION(SmLooptreeItem) ;


#endif // !__SMLOOPPROPS_H__
