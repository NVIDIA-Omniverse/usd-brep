// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmFaceProps.h
* PURPOSE: Header file for SmFaceProps class.
**********************************************************************/

#ifndef __SMFACEPROPS_H__
#define __SMFACEPROPS_H__

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

#ifndef __SMLOOPPROPS_H__
#include <SmLoopProps.h>
#endif // no __SMLOOPPROPS_H__

class SmFace;
class SmSurface;
class SmCurveClassification;
enum SmMixedLinkType;
enum SmHealerOpType;

/*******************************************************************//**
PURPOSE: list of SmFace::ClassifyLoops() failure codes

NOTES: SmFace::ClassifyLoops() is used by
***********************************************************************/
enum SmHealNotYetFaceType
{
    SM_HNF_NO_PROBS,                   // Face has no NotYet Supported Face problems
    SM_HNF_NO_CLASSIFY_ZERO_CROSSCNT,  // SmFace::ClassifyLoops() couldn't classify FaceLoops.
                                       //   ClassifyCrv built to cross a TgtLoop 
                                       //   found No Loop Bndrys to Cross.
                                       //    - saw this on a DegenFace that had Edge bndrys
                                       //   that were crossed, but those Edges had no
                                       //   Edgeuse connecting Edge to Face inside.
                                       SM_HNF_NO_CLASSIFY_ODD_CROSSCNT,   // SmFace::ClassifyLoops() couldn't classify FaceLoops.
                                                                          //   ClassifyCrv built to cross a TgtLoop 
                                                                          //   found Odd number of Loop Bndrys to Cross on
                                                                          //   a Face with a NonClosed Surface.  On a Closed Surface
                                                                          //   an odd number of crossings is a NoArea_Loop (okay).
                                                                          //   On an open Surface its an NotYet Supported problem.
                                                                          //    - saw this on a DegenFace that had Coinc Edge bndrys
                                                                          //      that were crossed an even number of times but then
                                                                          //      merged into an odd count because the ClassifyCrv param
                                                                          //      values of the two XSects were the same.
                                                                          SM_HNF_NO_LOOP_TREE_ROOT           // SmFace::ClassifyLoops() couldn't build an SmLooptreeItem
                                                                                                             //   for the Face.
}; // end enum SmHealNotYetFaceType

/*******************************************************************//**
PURPOSE: list of SmFaceProps SetProps States

NOTES: Face props have to be built in steps after various
       Heal steps have run because some of the FaceProp evaluations
       depend on data that can be healed.
***********************************************************************/
enum SmPropStageType
{
    SM_PROPSTAGE_0,     // 0 = Props  immediately after construction
    SM_PROPSTAGE_GAPSa, // 1 = during SmFaceProps::SetProps_Gaps()   call      
    SM_PROPSTAGE_GAPS,  // 2 = after  SmFaceProps::SetProps_Gaps()   call  (was SetProps_BeforeTolFix())    
    SM_PROPSTAGE_2a,    // 3 = during SmFaceProps::SetProps_Stage2() call   
    SM_PROPSTAGE_2,     // 4 = after  SmFaceProps::SetProps_Stage2() call  (was SetProps_BeforeEdgeSplit())
    SM_PROPSTAGE_3a,    // 5 = during SmFaceProps::SetProps_Stage3() call  
    SM_PROPSTAGE_3      // 6 = after  SmFaceProps::SetProps_Stage3() call  (Was SetProps_AfterEdgeSplit()) 
}; // end SmPropStageType

/*******************************************************************//**
PURPOSE: Container class for common Face and Face AssertValid property values.

NOTES: 1. Initial use: To pass information around the Heal Methods
          called by the HealTopologyFromData() method.

       2. Not all member values are set at the same time.  In the long
          run values should be marked with ready bits.  But for
          now the value assignments are simple and done in just 3 functions:
            1. SmFaceProps::SetProps_Stage2
            2. SmFaceProps::SetProps_Stage3
          The member values document where they get set.
***********************************************************************/
class SM_EXPORT SmFaceProps : public SmObjProps
{
public:
    // Face Prop State
    SmPropStageType m_ePropStage = SM_PROPSTAGE_0; // oneof: 0 = SM_PROPSTAGE_0     = SmFaceProps immediately after construction
                                                    //        1 = SM_PROPSTAGE_GAPSa = during SmFaceProps::SetProps_Gaps() call      
                                                    //        2 = SM_PROPSTAGE_GAPS  = after  SmFaceProps::SetProps_Gaps() call      
                                                    //        3 = SM_PROPSTAGE_2a    = during SmFaceProps::SetProps_Stage2() call    
                                                    //        4 = SM_PROPSTAGE_2     = after  SmFaceProps::SetProps_Stage2() call    
                                                    //        5 = SM_PROPSTAGE_3a    = during SmFaceProps::SetProps_Stage3() call
                                                    //        6 = SM_PROPSTAGE_3     = after  SmFaceProps::SetProps_Stage3() call

    // SetGroup:[constructor()] 
    const SmContext* m_cpContext = NULL;      // context for new object construction - not owned

    // SetGroup:[Wherever currently unfixable problems are detected]
    SmBooleanUL               m_bBadHasNotYetProb = FALSE;                                 // link: SmHealData::m_sNotYetFaceProps (note: SmBooleanUL = booleans declared as ULONG)
    enum SmHealNotYetFaceType m_eNotYetFaceType   = SmHealNotYetFaceType::SM_HNF_NO_PROBS; // Heal_NotYet_Resolved_Problems: a flag value set to the first unsupported problem found - if any
                                                                   //   SM_HNF_NO_PROBS                  = ok : Face has no unsupported Problems
                                                                   //   SM_HNF_NO_CLASSIFY_ZERO_CROSSCNT = bad: SmFace::ClassifyLoops() NotYet Supported case
                                                                   //   SM_HNF_NO_CLASSIFY_ODD_CROSSCNT  = bad: SmFace::ClassifyLoops() NotYet Supported case

    // no Face connected to CoinVertices, CoinEdges, DegenEdges, or DegenFaces stored here.  This info is in SmHealData

    // Set in SmHealData::Cache_DegenFaces(): Face degeneracy bit
    SmBooleanUL   m_bBadDegenFace = UNSURE;    // link: SmHealData::m_sBadFaceProps_DegenFaces (note: SmBooleanUL = booleans declared as ULONG)
                                               // set : [Get_DegenFaces()]
                                               // use : TRUE = Face->Edges all within tol of another Face->Edge. Face will be squeezed

    // SetProps(SM_PROPSTAGE_GAPS)
    SmFace      * m_pFace = NULL;                          // Target Face being studied - (changes every debug run)
    ULONG         m_lFaceIndx = SM_UNDEF_ULONG;            // associated FaceIndx within managing SmHealData::m_TgtFaces list - (constant between debug runs)
                                                           //   note: can be any value when using FaceProps outside of the healer
    SmZoneTol3d   m_sZoneTol3d = SM_UNDEF_DOUBLE;          // TolVal PostFix: When bad - Tol as changed by healer
    SmZoneTol3d   m_sOrigZoneTol3d = SM_UNDEF_DOUBLE;      // TolVal PreFix : Tol as given to healer
    SmBooleanUL   m_bBadSmallZoneTol3d = UNSURE;           // link: SmHealData::m_sBadFaceProps_SmallZoneTol3d (note: SmBooleanUL = booleans declared as ULONG)
                                                           // BadTolFlag: TRUE = FaceOrigZoneTol3d less    than SmTol::CalcFaceZoneTol3d() (ObjTol with Gaps)
                                                           //             List:[m_sBadFaceProps_SmallZoneTol3d], Fix:[SmHealData::Fix_TolSizes()]
    SmBooleanUL   m_bBadLargeZoneTol3d = UNSURE;           // link: SmHealData::m_sBadFaceProps_LargeZoneTol3d (note: SmBooleanUL = booleans declared as ULONG)
                                                           // BadTolFlag: TRUE = FaceOrigZoneTol3d greater than SmTol::CalcFaceZoneTol3d() (ObjTol with Gaps)
                                                           //             List:[m_sBadFaceProps_LargeZoneTol3d], Fix:[SmHealData::Fix_TolSizes()]
    // end SetProps(SM_PROPSTAGE_GAPS)

    // SetProps(SM_PROPSTAGE_2)
    const SmSurface* m_cpSurface = NULL;                       // TgtFace->Surface
    SmExtent2d          m_sNaturalUVDomain;                    // TgtFace->Surface->NaturalDomain
    SmMixedLinkType     m_eSheet = SM_MP_UNDEF;                // link: SmHealData::m_sFaceProps_Sheets
                                                               // SheetFlag      : SM_MP_NOPROP   // obj does not have property
                                                               //                  SM_MP_HASPROP, // obj has property - not yet known if that's okay or a problem
                                                               //                  SM_MP_OKAY,    // obj has property and is known to be okay
                                                               //                  SM_MP_PROB,    // obj has property and is known to be problem needing fixing
                                                               //                  SM_MP_FIXED,   // obj has property that was a problem and is now fixed
                                                               //                  SM_MP_UNDEF    // propetry value not yet known
    SmBoolean           m_bClosedU = UNSURE;                   // ClosedUFlag    : TRUE = Face is on ClosedU Surface - check for seam problems
                                                               //                  Surface(UMin,v) == Surface(UMax,v)
    SmBoolean           m_bClosedV = UNSURE;                   // ClosedVFlag    : TRUE = Face is on ClosedV Surface - check for seam problems
                                                               //                  Surface(u,VMIN) == Surface(u,VMAX)
    SmBoolean           m_bClosedSurf = UNSURE;                // ClosedSurfFlag : TRUE= Torus or Sphere, ie. (ClosedU && ClosedV) or (ClosedUorV and Seam bounded by Poles)
    SmContinuityType    m_eClosedValU = SM_CT_UNDEFINED;       // ClosedUVal     : SM_CT_UNDEFINED        SM_CT_G1(3) SM_CT_G1_G2(5)                             
                                                               //     oneof        SM_CT_DISCONTINUOUS(1) SM_CT_C1(7) SM_CT_C1_G2(8) 
                                                               //                  SM_CT_C0(2)                        SM_CT_C1_C2(10)
    SmContinuityType    m_eClosedValV = SM_CT_UNDEFINED;       // ClosedVVal     : SM_CT_UNDEFINED        SM_CT_G1(3) SM_CT_G1_G2(5) 
                                                               //     oneof        SM_CT_DISCONTINUOUS(1) SM_CT_C1(7) SM_CT_C1_G2(8)           
                                                               //                  SM_CT_C0(2)                        SM_CT_C1_C2(10)
    SmMixedLinkType     m_ePoles = SM_MP_UNDEF;                // link: SmHealData::m_sFaceProps_NoVertexPole
                                                               // PoleFlag       : SM_MP_NOPROP   // obj does not have property
                                                               //                  SM_MP_HASPROP,  // obj has property - not yet known if that's okay or a problem
                                                               //                  SM_MP_OKAY,     // obj has property and is known to be okay
                                                               //                  SM_MP_PROB,     // obj has property and is known to be problem needing fixing
                                                               //                  SM_MP_FIXED,    // obj has property that was a problem and is now fixed
                                                               //                  SM_MP_UNDEF     // propetry value not yet known
    ULONG                m_lPoles = SM_UNDEF_ULONG;            // SrfPoleFlag    : SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX  when PoleTol < Scaled(IW_EFF_ZERO)
    ULONG                m_lApproxPoles = SM_UNDEF_ULONG;      // ApproxPoleFlag : SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX  wgen PoleTol < ZoneTol3d
    ULONG                m_lNoVertexPoles = SM_UNDEF_ULONG;    // NonFacePoleFlag: SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX
                                                               //                  This can be okay if the pole is not 'in' or 'on the boundary' of the Face
    SmTArray<SmPoint3d>  m_sPolePoints;                        // PolePtVals     : PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                                               //                  NonSingular side values set to SmPoint3d::SetUninitialized()
                                                               //                  m_lPoles is a BitMask for set m_sPolePoint values.
    SmTArray<SmVector3d> m_sPoleNormals;                       // PoleNormalVals : PoleNormal[4] array, ordered :[UMinNrml, VMinNrml, UMaxNrml, VMaxNrml]
                                                               //                  NonSingular side values set to SmVector3d::SetUninitialized()
                                                               //                  m_lPoles is a BitMask for set m_sPoleNormals values.

    ULONG       m_lBadFlatCorners = SM_UNDEF_ULONG;          // link: SmHealData::m_sBadFaceProps_FlatCorner
                                                             // BadFlatCornerFlag: SM_FC_NONE or an orof: SM_FC_UMIN_VMIN, SM_FC_UMIN_VMAX, SM_FC_UMAX_VMIN, SM_FC_UMAX_VMAX
                                                             //                    List:[m_sBadFaceProps_FlatCorner()], Fix:[NoneYet]
    SmTArray<SmPoint3d> m_sFlatCornerPoints;                 // FlatCornerPtVals : array[4] of FlatCorner locs[UMinVMin, UMinVMax, UMaxVMin, UMaxVmax] 
                                                             //                    NonFlatCorner points set to SmPoint3d::SetUninitialized()
                                                             //                    m_lBadFlatCorners is a BitMask for set m_sFlatCornerPoints

    SmCurveClassification* m_pCrvClassU = NULL;              // set:[SmFace::HasSeamProblems()] SeamU ClassifyCrv: FaceBndry/SeamU XSects, 
                                                             //                    if(m_bClosedU == TRUE) SetIn:[SetProps_Stage2()->SmFace::HasSeamProplem()]
                                                             //                    if(m_bClosedU == FALSE), Left UnInit
                                                             //                    m_pCrvClassU->m_cpCurve is the SeamU_Crv
    SmTArray<SmTouchData>   m_sRawTouchListU;                // set:[SmFace::HasSeamProblems()] SeamU RawTouches : m_pCrvClassU mapped to Loop, Pole, and NoVertex_Pole touches
    SmTArray<SmTouchData>   m_sDoneTouchListU;               // set:[SmFace::HasSeamProblems()] SeamU DoneTouches: m_sRawTouchListU purged of cracks, tangent, strut and periodice touches

    SmCurveClassification * m_pCrvClassV = NULL;             // set:[SmFace::HasSeamProblems()] SeamV ClassifyCrv: FaceBndry/SeamV XSects, 
                                                             //                    if(m_bClosedV == TRUE) SetIn:[SetProps_Stage2()->SmFace::HasSeamProplem()]
                                                             //                    if(m_bClosedV == FALSE), Left UnInit
                                                             //                    m_pCrvClassV->m_cpCurve is the SeamV_Crv
    SmTArray<SmTouchData>   m_sRawTouchListV;                // set:[SmFace::HasSeamProblems()] SeamV RawTouches : m_pCrvClassV mapped to Loop, Pole, and NoVertex_Pole touches
    SmTArray<SmTouchData>   m_sDoneTouchListV;               // set:[SmFace::HasSeamProblems()] SeamV DoneTouches: m_sRawTouchListV purged of cracks, tangent, strut and periodice touches

    SmBooleanUL      m_bBadCrossedSeam = FALSE;              // link: SmHealData::m_sBadFaceProps_CrossedSeam
                                                             // set:[SmFaceProps::SetProps_Stage2()] 
    SmSurfParamType  m_eBadCrossedSeam = SM_SP_UNKNOWN;      // set:[SmFace::HasSeamProblems()] BadCrossedSeamFlag: 
                                                             //    SM_SP_U       = 1 or more FaceEdges cross periodic U BndrySeam (Edges should be split at the seam)  
                                                             //    SM_SP_V       = 1 or more FaceEdges cross periodic V BndrySeam (Edges should be split at the seam) 
                                                             //    SM_SP_BOTH    = FaceEdges cross both periodic U and V BndrySeams
                                                             //    SM_SP_NEITHER = no FaceEdges cross a periodic U or V BndrySeam 
                                                             //                     List:[m_sBadFaceProps_SeamProbs], Fix:[Fix_MoveSeam(), Fix_SplitEdgesAtSeam()]
    SmBooleanUL      m_bBadMissingSeam = FALSE;              // link: SmHealData::m_sBadFaceProps_MissingSeam
                                                             // set:[SmFaceProps::SetProps_Stage2()] 
    SmSurfParamType  m_eBadMissingSeam = SM_SP_UNKNOWN;      // set:[SmFace::HasSeamProblems()] BadMissingSeamFlag: 
                                                             //    SM_SP_U       = MissingEdge for 1 or more periodic U BndrySeam segments on the Face
                                                             //    SM_SP_V       = MissingEdge for 1 or more periodic V BndrySeam segments on the Face
                                                             //    SM_SP_BOTH    = MissingEdges for both U and V periodic BndrySeam segments on the Face
                                                             //    SM_SP_NEITHER = No MissingEdges for BndrySeam segments on the Face
                                                             //                     (SM_SP_NEITHER is also used when no BndrySeam segments are on the Face)
                                                             //                     List:[m_sBadFaceProps_SeamProbs], Fix:[Fix_MoveSeam(), Fix_SplitFaceAtSeams()]
    SmBooleanUL      m_bBadNearMissSeam = FALSE;             // link: SmHealData::m_sBadFaceProps_NearMissSeam
                                                             // set:[SmFaceProps::SetProps_Stage2()] 
    SmSurfParamType  m_eBadNearMissSeam = SM_SP_UNKNOWN;     // set:[SmFace::HasSeamProblems()] For future use once Fix_BadGaps() gets built
                                                             //    Exact gaps are of numerical tolerance size ~1.0e-12 * 100
                                                             //    NearMiss gaps are valid gap sized less than XSectTol3d but
                                                             //    NearMiss Topo connections are valid - future Fix_BadGaps() will refine geom to tighten gaps from NearMiss to Exact
                                                             // BadNearMissSeamFlag: 
                                                             //    SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam 
                                                             //    SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam
                                                             //    SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams 
                                                             //    SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams
                                                             //                     List:[m_sBadFaceProps_SeamProbs], Fix:[Fix_MoveSeam(), Need to add to Fix_SplitEdgesAtSeams()]
    // end SetProps(SM_PROPSTAGE_2)

    SmTriedLinkType m_eBeenThroughMoveSeam = SM_TRY_NONE;    // link: SmHealData::m_sRanFaceProps_MoveSeam
                                                             // one of: SM_TRY_NONE,     // obj not yet run through Fix_MoveSeam Fix function
                                                             //         SM_TRY_FIXED,    // obj run through Fix_MoveSeam fix function - all Seam probs fixed
                                                             //         SM_TRY_NOFIX,    // obj run through Fix_MoveSeam fix function - some Seam probs not fixed
                                                             //
    SmTriedLinkType m_eBeenThroughSplitEdgesAtSeam = SM_TRY_NONE; // link: SmHealData::m_sRanFaceProps_SplitEdgesAtSeam
                                                             // one of: SM_TRY_NONE,     // obj not yet run through Fix_SplitEdgesAtSeam Fix function
                                                             //         SM_TRY_FIXED,    // obj run through Fix_SplitEdgesAtSeam fix function - all Seam probs fixed
                                                             //         SM_TRY_NOFIX,    // obj run through Fix_SplitEdgesAtSeam fix function - some Seam probs not fixed
                                                             //

    SmTriedLinkType m_eBeenThroughSplitFacesAtSeam = SM_TRY_NONE; // link: SmHealData::m_sRanFaceProps_SplitFaceAtSeam
                                                             // one of: SM_TRY_NONE,     // obj not yet run through Fix_SplitFaceAtSeams Fix function
                                                             //         SM_TRY_FIXED,    // obj run through Fix_SplitFaceAtSeams fix function - all Seam probs fixed
                                                             //         SM_TRY_NOFIX,    // obj run through Fix_SplitFaceAtSeams fix function - some Seam probs not fixed
                                                             //

    // SetProps(SM_PROPSTAGE_3) from m_sLoopProps
    SmLooptreeItem* m_pLooptreeRoot = NULL;           // set:[SmFace::ClassifyLoops()] Containment LoopTree Root (One LoopTreeItem per FaceLoop)
    SmTArray<SmLoopProps>  m_sLoopProps;              // set:[SmFaceProps::SetProps_Stage2()]
                                                      //     [SmLoopProps::SetProps()       ] 
                                                      //     [SmFace::ClassifyLoops()       ] LoopProps for all LoopTree Nodes.  one LoopProps per LoopTree node.
                                                      //    FaceLoopCnt == m_sLoopProps.GetSize()

    ULONG     m_lClosedLoopCnt = SM_UNDEF_ULONG;      // total number of Closed FaceLoops
    ULONG     m_lOuterLoopCnt = SM_UNDEF_ULONG;       // link: SmHealData::m_sBadFaceProps_MultiOuterLoop
                                                      // Number of outer loops: One is required, Greater than 1 is bad
                                                      //    Loops with ContainmentType == SM_CMT_OUTERLOOP count as 1
                                                      //    RootLevel Looptree Loops that are paired NoArea_Loops count as 1

    SmBooleanUL   m_bBadLoopProblems = UNSURE;        // link: SmHealDataL::m_sBadFaceProps_LoopProblems - union of all FaceLoopProblems
                                                      // TRUE  = Face has at least one kind of a loop problem
                                                      // FALSE = Face has no loop problems

    SmBooleanUL   m_bBadOuterLoopOrder = UNSURE;         // link: SmHealData::m_sBadFaceProps_OuterLoopOrder     // TRUE = 1st Loop on FaceLoopList is not an OuterLoop, FALSE = okay
    ULONG         m_lBadOrient_LoopCnt = SM_UNDEF_ULONG; // link: SmHealData::m_sBadFaceProps_OrientLoop         // total number of AreaLoops and PoleCross_NoArea_Loops with bad orientations
    ULONG         m_lBadNoArea_LoopCnt = SM_UNDEF_ULONG; // link: SmHealData::m_sBadFaceProps_NoArea_Loop        // total number of NoArea_FaceLoops (not counting WireEdges)
    ULONG         m_lBadNested_LoopCnt = SM_UNDEF_ULONG; // link: SmHealData::m_sBadFaceProps_NestedOuterLoops   // total number of badly nested loops
    SmBooleanUL   m_bBadUnpairedNoArea_Loop = UNSURE;    // link: SmHealData::m_sBadFaceProps_UnpairedNoAreaLoop // TRUE = NoArea_Loops and PoleVertices do not come in matched pairs. FALSE = okay

    ULONG         m_lBadClosed3d_LoopCnt = SM_UNDEF_ULONG;  // link: SmHealData::m_sBadFaceProps_Closed3dLoop   // total number of loops open in 3d space
    ULONG         m_lBadClosedPtr_LoopCnt = SM_UNDEF_ULONG; // link: SmHealData::m_sBadFaceProps_ClosedPtrLoop  // total number of Loops where bClosedLoop3d != bClosedLoopPtrs (assumed supposed to be closed)
    ULONG         m_lBadMissingPoles = SM_UNDEF_ULONG;      // link: SmHealData::m_sBadFaceProps_MissingPole    // BadPoleFlag: SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX
                                                                                                                //    Currently only for cones missing seams and not marking
                                                                                                                //    the pole at the tip of the cone with a vertex
    // end SetProps(SM_PROPSTAGE_3) from m_sLoopProps

public:

    // constructors and destructor
    SmFaceProps(const SmContext* cpContext);
    SmFaceProps(SmFace* pFace, ULONG lFaceIndx, const SmContext* cpContext);
    SmFaceProps(const SmFaceProps& crOther);
    virtual ~SmFaceProps();

    // deep copy operator
    SmStatus Copy(SmFaceProps*& rpNewFaceProps)
    {
        rpNewFaceProps = new SmFaceProps(*this); NER(rpNewFaceProps);
        return SM_SUCCESS;
    }

    // assignment and equality operators
    SmFaceProps& operator= (const SmFaceProps& crOther);
    SmBoolean     operator==(const SmFaceProps& crOther) const;
    void          SetContext(const SmContext* cpContext);

public:

    // clear all member values                                                                                  
    void ReSet(SmPropStageType eClearPropStage = SM_PROPSTAGE_0); // in : oneof: SM_PROPSTAGE_0    = clear constructor values     and
                                                                  //             SM_PROPSTAGE_GAPS = clear SetProps_Gaps() vals and
                                                                  //             SM_PROPSTAGE_2    = clear SetProps_Stage2() vals and
                                                                  //             SM_PROPSTAGE_3    = clear SetProps_Stage3() vals
                                                                  //             default:[SM_PROPSTAGE_0]

    // Set Props thru given eSetPropStage by calling SetProps_Gaps(), SetProps_Stage2(), SetProps_Stage3() as needed 
    SmStatus SetProps(SmPropStageType  eSetPropStage,
        const SmFace* cpFace,
        ULONG            lFaceIndx,
        SmHealerOpType   eExeHealOp,
        ULONG            lOptLabel = SM_UNDEF_ULONG);

    // SetProps_Gaps to be run aftre Fix_CoinEdges() - (was SetProps_BeforeTolFix())   
    SmStatus SetProps_Gaps(const SmFace * cpFace,                       // in : 
                           ULONG          lFaceIndx,                    // in : 
                           SmHealerOpType eExeHealOp,                   // NotUsed: in : 
                           ULONG          lOptLabel = SM_UNDEF_ULONG) ; // in : 
    //        (sets: m_pFace               m_sZoneTol3d m_sOrigZoneTol3d          
    //               m_bBadSmallZoneTol3d  m_bBadLargeZoneTol3d)  

    // SetProps_Stage2 to be run after Fix_TolSizes() - (was SetProps_BeforeEdgeSplit())
    SmStatus SetProps_Stage2(const SmFace * cpFace,
                             ULONG          lFaceIndx,
                             SmHealerOpType eExeHealOp,
                             ULONG          lOptLabel = SM_UNDEF_ULONG) ;
    //        (sets: m_cpSurface           m_sNaturalUVDomain
    //               m_bSheetFace          
    //               m_bClosedU            m_bClosedV          m_bClosedSurf
    //               m_eClosedValU         m_eClosedValV              
    //               m_lPoles              m_lApproxPoles      m_lNoVertexPoles
    //               m_sPolePoints         m_sPoleNormals      
    //               m_lBadFlatCorners     m_sFlatCornerPoints 
    //               m_pCrvClassU          m_pCrvClassV        
    //               m_sRawTouchListU      m_sRawTouchListV
    //               m_sDoneTouchListU     m_sDoneTouchListV
    //               m_eBadMissingSeam     m_eBadCrossedSeam   m_eBadNearMissSeam)

    // SetProps_Stage3 to be run after Fix_SplitEdgesAtSeam() - (was SetProps_AfterEdgeSplit()) 
    SmStatus SetProps_Stage3(const SmFace * cpFace,                       // in : 
                             ULONG          lFaceIndx,                    // in : 
                             SmHealerOpType eExeHealOp,                   // NotUsed: in : 
                             ULONG          lOptLabel = SM_UNDEF_ULONG) ; // in : 
    //        (sets: m_sLoopProps //  with call m_sLoopProps[ii].SetProps(pLoop, *this) ; 
    //                            //  that sets: SmLoopProps::m_cpLoop             m_bDegen_Loop  
    //                            //                        ::m_lEdgeCnt           m_bArea_Loop        
    //                            //                        ::m_lEdgeuseCnt        m_eContainmentType  
    //                            //                        ::m_bClosed3d          m_eLoopOrient       
    //                            //                        ::m_bClosedPtrs        m_eDesiredLoopOrient
    //                            //                        ::m_lSeamCrossingCntU  m_lSeamCrossingCntV
    //                            //                        ::m_lSeamWindingCntU   m_lSeamWindingCntV
    //                            //                        ::m_eWindingOrientU    m_eWindingOrientV
    //                            //                        ::m_sRawTouchListU     m_sRawTouchListV 
    //                            //                        ::m_sDoneTouchListU    m_sDoneTouchListV
    //                            //                        ::m_bGoodOrient        m_bGoodClosed       
    //               m_lClosedLoopCnt       m_lOuterLoopCnt          
    //               m_bBadOuterLoopOrder   m_lBadOrient_LoopCnt     
    //               m_lBadNoArea_LoopCnt   m_lBadNested_LoopCnt     
    //               m_lBadMissingPoles     m_bBadUnpairedNoArea_Loop
    //               m_lBadClosed3d_LoopCnt m_lBadClosedPtr_LoopCnt)

    // predicates
    SmBoolean IsPeriodicU()                         const { return(m_bClosedU && m_eClosedValU >= SM_CT_C0); }
    SmBoolean IsPeriodicV()                         const { return(m_bClosedV && m_eClosedValV >= SM_CT_C0); }
    SmBoolean IsPoint3dOnPole(SmPoint3d &rPoint3d)  const ;
    SmBoolean HasProps(SmPropStageType  ePropStage,
                       const SmFace* cpFace)     const { if (ePropStage == SM_PROPSTAGE_GAPS) { return HasProps_Stage1(cpFace); }
                                                         if (ePropStage == SM_PROPSTAGE_2) { return HasProps_Stage2(cpFace); }
                                                         if (ePropStage == SM_PROPSTAGE_3) { return HasProps_Stage3(cpFace); }
                                                         return m_pFace == cpFace;
                                                       }
    SmBoolean HasProps_Stage1(const SmFace* cpFace) const { SM_REF1(cpFace) ;
                                                            SM_ASSERT_MSG(   (m_ePropStage < SM_PROPSTAGE_GAPS&& m_pFace == cpFace)
                                                                          || (   (m_pFace != NULL) && (m_pFace == cpFace)
                                                                              && (m_sOrigZoneTol3d != SM_UNDEF_DOUBLE)),
                                                                _T("SmFaceProps: data vals not up to date with stored DataState1 - a bug"));
                                                            return(m_ePropStage >= SM_PROPSTAGE_GAPS);
                                                          }
    SmBoolean HasProps_Stage2(const SmFace* cpFace) const { SM_REF1(cpFace) ;
                                                            SM_ASSERT_MSG(   (m_ePropStage < SM_PROPSTAGE_2)
                                                                          || (   (m_pFace != NULL) && (m_pFace == cpFace)
                                                                              && (m_sOrigZoneTol3d != SM_UNDEF_DOUBLE)
                                                                              && (m_cpSurface != NULL)),
                                                                _T("SmFaceProps: data vals not up to date with stored DataState2 - a bug"));
                                                            return(m_ePropStage >= SM_PROPSTAGE_2 && m_pFace == cpFace);
                                                          }
    SmBoolean HasProps_Stage3(const SmFace* cpFace) const { SM_REF1(cpFace) ;
                                                            SM_ASSERT_MSG(   (m_ePropStage < SM_PROPSTAGE_3)
                                                                          || (   (m_pFace != NULL) && (m_pFace == cpFace)
                                                                              && (m_sOrigZoneTol3d != SM_UNDEF_DOUBLE)
                                                                              && (m_cpSurface != NULL)
                                                                              && (m_lBadOrient_LoopCnt != SM_UNDEF_ULONG)),
                                                                _T("SmFaceProps: data vals not up to date with stored DataState3 - a bug"));
                                                            return(m_ePropStage >= SM_PROPSTAGE_3 && m_pFace == cpFace);
                                                          }

    SmBoolean HasNotYetProblems()                   const { return(m_eNotYetFaceType != SM_HNF_NO_PROBS); }

    SmBoolean HasProblems(ULONG lBitArray = 7)      const; ///< [in] : OrOf 1 = Post SetProps_Gaps() problems   <br>
                                                           ///<      :      2 = Post SetProps_Stage2() problems <br>
                                                           ///<      :      4 = Post SetProps_Stage3() problems <br>
                                                    
    // draw function
    SmDisplayList* Draw(SmBoolean bDrawAllCases = FALSE,                 ///< [in] : TRUE = output graphics for all Faces (with or without probs)                          <br>
                                                                         ///<      : default:[FALSE] = output graphics only for Faces with probs                           <br>
                        SmVector3d sHighlightColor = { 1, 0, 0 },        ///< [in] : def:[ 1, 0, 0]: If FaceHasProblems Draw FaceUV for highlight                          <br>
                        SmVector3d sBadZoneTol3dColor = { .6,.3,.8 },    ///< [in] : def:[.6,.3,.8]: Faces with bad (too large or small) ZoneTol3d vals Highlight Color    <br>
                        SmVector3d sBadSheetColor = { 0, 1, 1 },         ///< [in] : def:[ 0, 1, 1]: Faces mislabled as Sheets splitting their Shell Highlight Color       <br>
                        SmVector3d sCrossedSeamColor = { 1, 0, 1 },      ///< [in] : def:[ 1, 0, 1]: Loops Crossing Seams Highlight Color                                  <br>
                        SmVector3d sMissingSeamColor = { 1, 0, 0 },      ///< [in] : def:[ 1, 0, 0]: MissingSeam Highlight Color                                           <br>
                        SmVector3d sBadNearMissSeamColor = { 0, 1, 1 },  ///< [in] : def:[ 0, 1, 1]: NearMissSeam Hightlight Color                                         <br>
                        SmVector3d sMissingPoleColor = { 0, 0, 1 },      ///< [in] : def:[ 0, 0, 1]: MissingPoles Point Highlight Color                                    <br>
                        SmVector3d sFlatCornerColor = { .2,.2, 1 },      ///< [in] : def:[.2,.2, 1]: FlatCorner (SurfDU parallel to SurfDV) HIghlight Color                <br>
                        SmVector3d sBadOrientLoopColor = { .3,.3,.3 },   ///< [in] : def:[.3,.3,.3]: Badly oriented Loops Highlight Color                                  <br>
                        SmVector3d sBadDegen_LoopColor = { 1.,.5, 0 },   ///< [in] : def:[1.,.5, 0]: Loop has at least one degenerate dimension                            <br>
                        SmVector3d sNoArea_LoopColor = { 0, 0, 0 },      ///< [in] : def:[ 0, 0, 0]: NoArea_Loops Highlight Color                                          <br>
                        SmVector3d sOrient_ClassCrvColor = { .8,.2,.7 }, ///< [in] : def:[.8,.2,.7]: Orient CurveClassifcation Highlight Color                             <br>
                        SmVector3d sBadClosed_LoopColor = { .4,.8,.2 },  ///< [in] : def:[.4,.8,.2]: ClosedLoop3d != ClosedLoopPtrs (assumed intent is closed)             <br>
                        SmGfxArraySet* pOptGfxSet = NULL)                ///< [in] : When given, output GfxVertexArrays not GL calls.                                      <br>
                       const;

    // pretty print
    void Dump(ULONG     iLabel,                  // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]
              SmBoolean bLong  = TRUE) const;    // in : TRUE = Dump SeamU and SeamV CrvClassifications and Per Loop LoopProp data
                                                 //      FALSE= Don't

    void DumpProps_Gaps() const;
    void DumpProblems_Gaps() const;

    void DumpProps_Stage2() const;
    void DumpProblems_Stage2() const;

    void DumpProps_Stage3() const;
    void DumpProblems_Stage3() const;

    // Common label methods and void Dump() const ;
    SM_COMMON(SmFaceProps, SmObjProps, SmFaceProps_TYPE) ;

}; // end class SmFaceProps

// add a SmTArray<SmLoopProps> template to the dll interface
// SM_TARRAY_TEMPLATE_PREDECLARATION(SmFaceProps) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFaceProps*);

#endif // !__SMFACEPROPS_H__
