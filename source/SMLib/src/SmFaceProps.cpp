// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmFaceProps.cpp
* PURPOSE: Source file for implementation of SmFaceProps methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmFaceProps.h>
#include <SmEdgeProps.h>
#include <SmVertexProps.h>
#include <SmFace.h>
#include <SmTol.h>
#include <SmLoop.h>
#include <SmVertex.h>
#include <SmGraphicsOutput.h>
#include <SmCurveClass.h>
#include <SmContext.h>
#ifdef SM_DEBUG_CODE
  #include <SmBrep.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: SmFaceProp Method implementations
***********************************************************************/

/*******************************************************************//**
 PURPOSE:
 NOTES:
***********************************************************************/
SmBoolean SmFaceProps::IsKindOf( SM_TYPE t ) const
{
  return ((SmFaceProps_TYPE == t) ? TRUE : SmObjProps::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Default Constructor

NOTES: 
***********************************************************************/
SmFaceProps::SmFaceProps(const SmContext * cpContext) 
{ 
  m_cpContext = cpContext ;

  // init members checked in ReSet()
  m_pCrvClassU    = NULL ;
  m_pCrvClassV    = NULL ;
  m_pLooptreeRoot = NULL ; 

  // init all member values to uninit values
  ReSet(SM_PROPSTAGE_0) ; // SM_PROPSTAGE_0 = reset all values

} // end SmFaceProps::SmFaceProps default constructor

/*******************************************************************//**
PURPOSE: SmFaceProps Constructor

NOTES: 
***********************************************************************/
SmFaceProps::SmFaceProps(SmFace * pFace, ULONG lFaceIndx, const SmContext * cpContext) 
{ 
  m_cpContext = cpContext ;
  
  // init members checked in ReSet()
  m_pCrvClassU    = NULL ;
  m_pCrvClassV    = NULL ;
  m_pLooptreeRoot = NULL ; 
  
  // init all member values to uninit values
  ReSet(SM_PROPSTAGE_0) ; // SM_PROPSTAGE_0 = reset all values

  m_pFace     = (SmFace *)pFace ; 
  m_lFaceIndx = lFaceIndx ; 

} // end SmFaceProps::SmFaceProps default constructor

/*******************************************************************//**
PURPOSE: Copy Constructor

NOTES: 
***********************************************************************/
#define SM_CRVCLASS_DEEP_COPY(To,From,cpCText) \
  { if(From) { if((To) == NULL)                                     \
                 { (To) = new (cpCText) SmCurveClassification() ; } \
               *(To) = *(From) ;                                    \
             }                                                      \
    else     { if(To) { delete (To) ; }                             \
               (To) = NULL ;                                        \
             }                                                      \
  }

/*******************************************************************//**
PURPOSE: SmFaceProps Copy Constructor

NOTES: 
***********************************************************************/
SmFaceProps::SmFaceProps(const SmFaceProps & crOther) 
{
  // copy crOther properties
  *this = crOther ;

} // end SmFaceProps::SmFaceProps copy constructor

/*******************************************************************//**
PURPOSE: assignment operator

NOTES: 
***********************************************************************/
SmFaceProps & SmFaceProps::operator= (const SmFaceProps &crOther)
{
  // locals
  ULONG ii ;

  // no work - same object
  if(this == &crOther)
    { return *this ; }

  m_ePropStage        = crOther.m_ePropStage ; 
  m_cpContext         = crOther.m_cpContext ;
  m_bBadHasNotYetProb = crOther.m_bBadHasNotYetProb ; 
  m_eNotYetFaceType   = crOther.m_eNotYetFaceType ;
  m_bBadDegenFace     = crOther.m_bBadDegenFace ;

  // SetProps(SM_PROPSTAGE_GAPS) "Stage1"
    {
      // Face
      m_pFace     = crOther.m_pFace ;
      m_lFaceIndx = crOther.m_lFaceIndx ; 
  
      // tolerances
      m_sZoneTol3d     = crOther.m_sZoneTol3d ;
      m_sOrigZoneTol3d = crOther.m_sOrigZoneTol3d ;
                
      // Gap3d
      m_bBadSmallZoneTol3d = crOther.m_bBadSmallZoneTol3d ;
      m_bBadLargeZoneTol3d = crOther.m_bBadLargeZoneTol3d ;
    } // end SetProps(SM_PROPSTAGE_GAPS)

  // SetProps(SM_PROPSTAGE_2)
    {
      // Face->Surface 
      m_cpSurface        = crOther.m_cpSurface ;
      m_sNaturalUVDomain = crOther.m_sNaturalUVDomain ;

      // Major Face->Surface properties that force the face to be checked for problems and healing 
      m_eSheet           = crOther.m_eSheet ;         // oneof: : SM_MP_NOPROP   // obj does not have property
                                                      //          SM_MP_HASPROP,  // obj has property - not yet known if that's okay or a problem
                                                      //          SM_MP_OKAY,     // obj has property and is known to be okay
                                                      //          SM_MP_PROB,     // obj has property and is known to be problem needing fixing
                                                      //          SM_MP_FIXED,    // obj has property that was a problem and is now fixed
                                                      //          SM_MP_UNDEF   // propetry value not yet known
                                                      
      // Closed pFace->Surface classification         
      m_bClosedU         = crOther.m_bClosedU ;       // TRUE = Face is on ClosedU Surface - check for seam problems
      m_bClosedV         = crOther.m_bClosedV ;       // TRUE = Face is on ClosedV Surface - check for seam problems
      m_bClosedSurf      = crOther.m_bClosedSurf ;    // TRUE = Closed Torus or Sphere surface - check for unintersected seam containment
      m_eClosedValU      = crOther.m_eClosedValU ;    // oneof SM_CT_UNDEFINED        SM_CT_G1(3) SM_CT_G1_G2(5) 
                                                      //       SM_CT_DISCONTINUOUS(1) SM_CT_C1(7) SM_CT_C1_G2(8) 
                                                      //       SM_CT_C0(2)                        SM_CT_C1_C2(10)
      m_eClosedValV      = crOther.m_eClosedValV ;    // oneof SM_CT_UNDEFINED        SM_CT_G1(3) SM_CT_G1_G2(5) 
                                                      //       SM_CT_DISCONTINUOUS(1) SM_CT_C1(7) SM_CT_C1_G2(8) 
                                                      //       SM_CT_C0(2)                        SM_CT_C1_C2(10)
      // Singular pFace->Surface classification          
      m_ePoles           = crOther.m_ePoles ;         // oneof: : SM_MP_NOPROP   // obj does not have property
                                                      //          SM_MP_HASPROP,  // obj has property - not yet known if that's okay or a problem
                                                      //          SM_MP_OKAY,     // obj has property and is known to be okay
                                                      //          SM_MP_PROB,     // obj has property and is known to be problem needing fixing
                                                      //          SM_MP_FIXED,    // obj has property that was a problem and is now fixed
                                                      //          SM_MP_UNDEF   // propetry value not yet known
      m_lPoles           = crOther.m_lPoles ;         // SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX
      m_lApproxPoles     = crOther.m_lApproxPoles ;   // SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX
      m_lNoVertexPoles   = crOther.m_lNoVertexPoles ; // SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX

      { // m_sPolePoints array deep copy
        m_sPolePoints.SetSize(crOther.m_sPolePoints.GetSize()) ;  // PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                                                  //      NonSingular side values set to SmPoint3d::SetUninitialized()
        for(ii=0;ii<m_sPolePoints.GetSize();ii++)
          { if(crOther.m_sPolePoints[ii].IsInitialized()) { m_sPolePoints[ii] = crOther.m_sPolePoints[ii] ; }
            else                                          { m_sPolePoints[ii].SetUninitialized() ; }
          }
      } // end m_sPolePoints array deep copy

      { // m_sPoleNormals array deep copy
        m_sPoleNormals.SetSize(crOther.m_sPoleNormals.GetSize()) ; // PoleNormal[4] array, ordered :[UMinNrml, VMinNrml, UMaxNrml, VMaxNrml]
                                                                   //      NonSingular side values set to SmVector3d::SetUninitialized()
        for(ii=0;ii<m_sPoleNormals.GetSize();ii++)
          { if(crOther.m_sPoleNormals[ii].IsInitialized()) { m_sPoleNormals[ii] = crOther.m_sPoleNormals[ii] ; }
            else                                           { m_sPoleNormals[ii].SetUninitialized() ; }
          }
      } // end m_sPoleNormals array deep copy

      m_lBadFlatCorners = crOther.m_lBadFlatCorners ;        // SM_FC_NONE or an orof: SM_FC_UMIN_VMIN, SM_FC_UMIN_VMAX, SM_FC_UMAX_VMIN, SM_FC_UMAX_VMAX

      { // m_sFlatCornerPoints array deep copy
        m_sFlatCornerPoints.SetSize(crOther.m_sFlatCornerPoints.GetSize()) ; // array[4] of FlatCorner locs[UMinVMin, UMinVax, UMaxVMin, UMaxVmax]
                                                                             //         NonFlatCorner points set to Uninit
        for(ii=0;ii<m_sFlatCornerPoints.GetSize();ii++)
          { if(crOther.m_sFlatCornerPoints[ii].IsInitialized()) { m_sFlatCornerPoints[ii] = crOther.m_sFlatCornerPoints[ii] ; }
            else                                                { m_sFlatCornerPoints[ii].SetUninitialized() ; }
          }
      } // end m_sPoleNormals array deep copy

      // when pFace->Surface is Closed: The following are set in SmFaceProps::SetProps_Stage2() by SmFace::HasSeamProblem()

      //     m_pCrvClassU, m_sRawTouchListU, m_sDoneTouchListU 
      // and m_pCrvClassV, m_sRawTouchListV, m_sDoneTouchListV 
      if(m_pCrvClassU != NULL) { delete m_pCrvClassU ; m_pCrvClassU = NULL ; m_sRawTouchListU.ReSet() ; m_sDoneTouchListU.ReSet() ; }
      if(m_pCrvClassV != NULL) { delete m_pCrvClassV ; m_pCrvClassV = NULL ; m_sRawTouchListV.ReSet() ; m_sDoneTouchListV.ReSet() ; }

      SM_CRVCLASS_DEEP_COPY(m_pCrvClassU, crOther.m_pCrvClassU, m_cpContext) ;
      SM_CRVCLASS_DEEP_COPY(m_pCrvClassV, crOther.m_pCrvClassV, m_cpContext) ;

      m_sRawTouchListU.SetSize(crOther.m_sRawTouchListU.GetSize()) ;
      m_sDoneTouchListU.SetSize(crOther.m_sDoneTouchListU.GetSize()) ;

      m_sRawTouchListV.SetSize(crOther.m_sRawTouchListV.GetSize()) ;
      m_sDoneTouchListV.SetSize(crOther.m_sDoneTouchListV.GetSize()) ;

      for(ii=0;m_sRawTouchListU.GetSize() ;ii++) { m_sRawTouchListU[ii]  = crOther.m_sRawTouchListU[ii] ; }
      for(ii=0;m_sDoneTouchListU.GetSize();ii++) { m_sDoneTouchListU[ii] = crOther.m_sDoneTouchListU[ii] ; }
      for(ii=0;m_sRawTouchListV.GetSize() ;ii++) { m_sRawTouchListV[ii]  = crOther.m_sRawTouchListV[ii] ; }
      for(ii=0;m_sDoneTouchListV.GetSize();ii++) { m_sDoneTouchListV[ii] = crOther.m_sDoneTouchListV[ii] ; }

      m_eBadCrossedSeam  = crOther.m_eBadCrossedSeam ;   // SM_SP_U       = 1 or more FaceEdges cross periodic U BndrySeam (Edges should be split at the seam)
                                                         // SM_SP_V       = 1 or more FaceEdges cross periodic V BndrySeam (Edges should be split at the seam)
                                                         // SM_SP_BOTH    = FaceEdges cross both periodic U and V BndrySeams 
                                                         // SM_SP_NEITHER = no FaceEdges cross a periodic U or V BndrySeam 
                                                         // SM_SP_UNKNOWN = value not yet set

      m_eBadMissingSeam  = crOther.m_eBadMissingSeam ;   // SM_SP_U       = MissingEdge for 1 or more periodic U BndrySeam segments on the Face
                                                         // SM_SP_V       = MissingEdge for 1 or more periodic V BndrySeam segments on the Face
                                                         // SM_SP_BOTH    = MissingEdges for both U and V periodic BndrySeam segments on the Face
                                                         // SM_SP_NEITHER = No MissingEdges for BndrySeam segments on the Face 
                                                         // SM_SP_UNKNOWN = value not yet set

      m_eBadNearMissSeam = crOther.m_eBadNearMissSeam ;  //  For future use once Fix_BadGaps gets built
                                                         // SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam
                                                         // SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam
                                                         // SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams 
                                                         // SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams
                                                         // SM_SP_UNKNOWN = value not yet set
                                                         // NearMiss Topo connections are valid - future Fix_BadGaps will refine geom to tighten gaps from NearMiss to Exact
    } // end SetProps(SM_PROPSTAGE_2)

  m_eBeenThroughMoveSeam          = crOther.m_eBeenThroughMoveSeam ;
  m_eBeenThroughSplitEdgesAtSeam  = crOther.m_eBeenThroughSplitEdgesAtSeam ;
  m_eBeenThroughSplitFacesAtSeam  = crOther.m_eBeenThroughSplitFacesAtSeam ;

  // SetProps(SM_PROPSTAGE_3) from m_sLoopProps
    {
      // deep copy m_sLoopProps
      m_pLooptreeRoot         = crOther.m_pLooptreeRoot ? new SmLooptreeItem(*crOther.m_pLooptreeRoot) : NULL ; 

      m_sLoopProps.SetSize(crOther.m_sLoopProps.GetSize()) ;
      for(ii=0;ii<m_sLoopProps.GetSize();ii++)
        { m_sLoopProps[ii]    = crOther.m_sLoopProps[ii] ; }

      m_lClosedLoopCnt          = crOther.m_lClosedLoopCnt ; 
      m_lOuterLoopCnt           = crOther.m_lOuterLoopCnt ;

      m_bBadOuterLoopOrder      = crOther.m_bBadOuterLoopOrder ;
      m_lBadOrient_LoopCnt      = crOther.m_lBadOrient_LoopCnt ;               
      m_lBadNoArea_LoopCnt      = crOther.m_lBadNoArea_LoopCnt ;              
      m_lBadNested_LoopCnt      = crOther.m_lBadNested_LoopCnt ;     
      m_bBadUnpairedNoArea_Loop = crOther.m_bBadUnpairedNoArea_Loop ;
      m_lBadClosed3d_LoopCnt    = crOther.m_lBadClosed3d_LoopCnt  ; 
      m_lBadClosedPtr_LoopCnt   = crOther.m_lBadClosedPtr_LoopCnt ; 
      m_lBadMissingPoles        = crOther.m_lBadMissingPoles ;  
    } // end SetProps(SM_PROPSTAGE_3) from m_sLoopProps

  // all done
  return(*this) ;
  
} // end SmFaceProps::operator= assignment operator

/*******************************************************************//**
PURPOSE: SmFaceProps equality operator

NOTES: 
***********************************************************************/
SmBoolean SmFaceProps::operator== (const SmFaceProps &crOther) const
{
  // locals
  SmBoolean bRtn = TRUE ; 
  ULONG ii ;

  // no work - same object
  if(this == &crOther)
    { return bRtn ; }

  // bRtn &= m_cpContext    == crOther.m_cpContext ;  // not doing a context compare, Skip for now
  bRtn &= m_ePropStage        == crOther.m_ePropStage ; 
  bRtn &= m_cpContext         == crOther.m_cpContext ;
  bRtn &= m_bBadHasNotYetProb == crOther.m_bBadHasNotYetProb ; 
  bRtn &= m_eNotYetFaceType   == crOther.m_eNotYetFaceType ;
  bRtn &= m_bBadDegenFace     == crOther.m_bBadDegenFace ;

  // SetProps(SM_PROPSTAGE_GAPS) "Stage1"
    {
      // Face
      bRtn &= m_pFace     == crOther.m_pFace ;
      bRtn &= m_lFaceIndx == crOther.m_lFaceIndx ; 
  
      // tolerances
      bRtn &= m_sZoneTol3d     == crOther.m_sZoneTol3d ;
      bRtn &= m_sOrigZoneTol3d == crOther.m_sOrigZoneTol3d ;
                
      // Gap3d
      bRtn &= m_bBadSmallZoneTol3d == crOther.m_bBadSmallZoneTol3d ;
      bRtn &= m_bBadLargeZoneTol3d == crOther.m_bBadLargeZoneTol3d ;
    } // end SetProps(SM_PROPSTAGE_GAPS)

  // SetProps(SM_PROPSTAGE_2)
    {
      // Face->Surface 
      bRtn &= m_cpSurface        == crOther.m_cpSurface ;
      bRtn &= m_sNaturalUVDomain == crOther.m_sNaturalUVDomain ;

      // Major Face->Surface properties that force the face to be checked for problems and healing 
      bRtn &= m_eSheet           == crOther.m_eSheet ;         // oneof: : SM_MP_NOPROP   // obj does not have property
                                                                //          SM_MP_HASPROP,  // obj has property - not yet known if that's okay or a problem
                                                                //          SM_MP_OKAY,     // obj has property and is known to be okay
                                                                //          SM_MP_PROB,     // obj has property and is known to be problem needing fixing
                                                                //          SM_MP_FIXED,    // obj has property that was a problem and is now fixed
                                                                //          SM_MP_UNDEF   // propetry value not yet known
                                                      
      // Closed pFace->Surface classification         
      bRtn &= m_bClosedU         == crOther.m_bClosedU ;       // TRUE == Face is on ClosedU Surface - check for seam problems
      bRtn &= m_bClosedV         == crOther.m_bClosedV ;       // TRUE == Face is on ClosedV Surface - check for seam problems
      bRtn &= m_bClosedSurf      == crOther.m_bClosedSurf ;    // TRUE == Closed Torus or Sphere surface - check for unintersected seam containment
      bRtn &= m_eClosedValU      == crOther.m_eClosedValU ;    // oneof SM_CT_UNDEFINED        SM_CT_G1(3) SM_CT_G1_G2(5) 
                                                               //       SM_CT_DISCONTINUOUS(1) SM_CT_C1(7) SM_CT_C1_G2(8) 
                                                               //       SM_CT_C0(2)                        SM_CT_C1_C2(10)
      bRtn &= m_eClosedValV      == crOther.m_eClosedValV ;    // oneof SM_CT_UNDEFINED        SM_CT_G1(3) SM_CT_G1_G2(5) 
                                                               //       SM_CT_DISCONTINUOUS(1) SM_CT_C1(7) SM_CT_C1_G2(8) 
                                                               //       SM_CT_C0(2)                        SM_CT_C1_C2(10)
      // Singular pFace->Surface classification          
      bRtn &= m_ePoles           == crOther.m_ePoles ;         // oneof: : SM_MP_NOPROP   // obj does not have property
                                                               //          SM_MP_HASPROP,  // obj has property - not yet known if that's okay or a problem
                                                               //          SM_MP_OKAY,     // obj has property and is known to be okay
                                                               //          SM_MP_PROB,     // obj has property and is known to be problem needing fixing
                                                               //          SM_MP_FIXED,    // obj has property that was a problem and is now fixed
                                                               //          SM_MP_UNDEF   // propetry value not yet known
      bRtn &= m_lPoles           == crOther.m_lPoles ;         // SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX
      bRtn &= m_lApproxPoles     == crOther.m_lApproxPoles ;   // SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX
      bRtn &= m_lNoVertexPoles   == crOther.m_lNoVertexPoles ; // SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX

      { // m_sPolePoints array deep compare
        bRtn &= m_sPolePoints.GetSize() == crOther.m_sPolePoints.GetSize() ;  // PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                                                              //      NonSingular side values set to SmPoint3d::SetUninitialized()
        for(ii=0;bRtn && ii<m_sPolePoints.GetSize();ii++)
          { if(crOther.m_sPolePoints[ii].IsInitialized()) { bRtn &= m_sPolePoints[ii] == crOther.m_sPolePoints[ii] ; }
            else                                          { bRtn &= !m_sPolePoints[ii].IsInitialized() ; }
          }
      } // end m_sPolePoints array deep compare

      { // m_sPoleNormals array deep compare
        bRtn &= m_sPoleNormals.GetSize() == crOther.m_sPoleNormals.GetSize() ; // PoleNormal[4] array, ordered :[UMinNrml, VMinNrml, UMaxNrml, VMaxNrml]
                                                                               //      NonSingular side values set to SmVector3d::SetUninitialized()
        for(ii=0;bRtn && ii<m_sPoleNormals.GetSize();ii++)
          { if(crOther.m_sPoleNormals[ii].IsInitialized()) { bRtn &= m_sPoleNormals[ii] == crOther.m_sPoleNormals[ii] ; }
            else                                           { bRtn &= !m_sPoleNormals[ii].IsInitialized() ; }
          }
      } // end m_sPoleNormals array deep compare

      bRtn &= m_lBadFlatCorners == crOther.m_lBadFlatCorners ;        // SM_FC_NONE or an orof: SM_FC_UMIN_VMIN, SM_FC_UMIN_VMAX, SM_FC_UMAX_VMIN, SM_FC_UMAX_VMAX

      { // m_sFlatCornerPoints array deep compare
        bRtn &= m_sFlatCornerPoints.GetSize() == crOther.m_sFlatCornerPoints.GetSize() ; // array[4] of FlatCorner locs[UMinVMin, UMinVax, UMaxVMin, UMaxVmax]
                                                                                          //         NonFlatCorner points set to Uninit
        for(ii=0;bRtn && ii<m_sFlatCornerPoints.GetSize();ii++)
          { if(crOther.m_sFlatCornerPoints[ii].IsInitialized()) { bRtn &= m_sFlatCornerPoints[ii] == crOther.m_sFlatCornerPoints[ii] ; }
            else                                                { bRtn &= !m_sFlatCornerPoints[ii].IsInitialized() ; }
          }
      } // end m_sPoleNormals array deep compare

      // when pFace->Surface is Closed: The following are set in SmFaceProps::SetProps_Stage2() by SmFace::HasSeamProblem()

      //     m_pCrvClassU, m_sRawTouchListU, m_sDoneTouchListU 
      // and m_pCrvClassV, m_sRawTouchListV, m_sDoneTouchListV 
      // no SmCurveClassification operator== // bRtn &= *m_pCrvClassU == *crOther.m_pCrvClassU ;
      { // m_sRawTouchListU array deep compare
        bRtn &= m_sRawTouchListU.GetSize() == crOther.m_sRawTouchListU.GetSize() ;  
                                                                                    
        for(ii=0;bRtn && ii<m_sRawTouchListU.GetSize();ii++)
          { bRtn &= m_sRawTouchListU[ii] == crOther.m_sRawTouchListU[ii] ; }
      } // end m_sPolePoints array deep compare

      { // m_sDoneTouchListU array deep compare
        bRtn &= m_sDoneTouchListU.GetSize() == crOther.m_sDoneTouchListU.GetSize() ;
                                                                                    
        for(ii=0;bRtn && ii<m_sDoneTouchListU.GetSize();ii++)
          { bRtn &= m_sDoneTouchListU[ii] == crOther.m_sDoneTouchListU[ii] ; }
      } // end m_sPolePoints array deep compare

      { // m_sRawTouchListV array deep compare
        bRtn &= m_sRawTouchListV.GetSize() == crOther.m_sRawTouchListV.GetSize() ;  
                                                                                    
        for(ii=0;bRtn && ii<m_sRawTouchListV.GetSize();ii++)
          { bRtn &= m_sRawTouchListV[ii] == crOther.m_sRawTouchListV[ii] ; }
      } // end m_sPolePoints array deep compare

      { // m_sDoneTouchListV array deep compare
        bRtn &= m_sDoneTouchListV.GetSize() == crOther.m_sDoneTouchListV.GetSize() ;
                                                                                    
        for(ii=0;bRtn && ii<m_sDoneTouchListV.GetSize();ii++)
          { bRtn &= m_sDoneTouchListV[ii] == crOther.m_sDoneTouchListV[ii] ; }
      } // end m_sPolePoints array deep compare

      bRtn &= m_bBadCrossedSeam  == crOther.m_bBadCrossedSeam ;  // SM_SP_U       == 1 or more FaceEdges cross periodic U BndrySeam (Edges should be split at the seam)
      bRtn &= m_eBadCrossedSeam  == crOther.m_eBadCrossedSeam ;  // SM_SP_V       == 1 or more FaceEdges cross periodic V BndrySeam (Edges should be split at the seam)
                                                                 // SM_SP_BOTH    == FaceEdges cross both periodic U and V BndrySeams 
                                                                 // SM_SP_NEITHER == no FaceEdges cross a periodic U or V BndrySeam 
                                                                 // SM_SP_UNKNOWN == value not yet set

      bRtn &= m_bBadMissingSeam  == crOther.m_bBadMissingSeam ;  // SM_SP_U       == MissingEdge for 1 or more periodic U BndrySeam segments on the Face
      bRtn &= m_eBadMissingSeam  == crOther.m_eBadMissingSeam ;  // SM_SP_V       == MissingEdge for 1 or more periodic V BndrySeam segments on the Face
                                                                 // SM_SP_BOTH    == MissingEdges for both U and V periodic BndrySeam segments on the Face
                                                                 // SM_SP_NEITHER == No MissingEdges for BndrySeam segments on the Face 
                                                                 // SM_SP_UNKNOWN == value not yet set

      bRtn &= m_bBadNearMissSeam == crOther.m_bBadNearMissSeam ; //  For future use once Fix_BadGaps gets built
      bRtn &= m_eBadNearMissSeam == crOther.m_eBadNearMissSeam ; // SM_SP_U       == 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam
                                                                 // SM_SP_V       == 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam
                                                                 // SM_SP_BOTH    == NearMiss for both U and V periodic BndrySeams 
                                                                 // SM_SP_NEITHER == No NearMiss CoincidentEdges or Vertices for BndrySeams
                                                                 // SM_SP_UNKNOWN == value not yet set
                                                                 // NearMiss Topo connections are valid - future Fix_BadGaps will refine geom to tighten gaps from NearMiss to Exact
    } // end SetProps(SM_PROPSTAGE_2)

  bRtn &= m_eBeenThroughMoveSeam          == crOther.m_eBeenThroughMoveSeam ;
  bRtn &= m_eBeenThroughSplitEdgesAtSeam  == crOther.m_eBeenThroughSplitEdgesAtSeam ;
  bRtn &= m_eBeenThroughSplitFacesAtSeam  == crOther.m_eBeenThroughSplitFacesAtSeam ;

  // SetProps(SM_PROPSTAGE_3) from m_sLoopProps
    {
      // deep compare m_sLoopProps
      bRtn &= m_sLoopProps.GetSize() == crOther.m_sLoopProps.GetSize() ; 

      for(ii=0;ii<m_sLoopProps.GetSize();ii++)
        { bRtn &= m_sLoopProps[ii] == crOther.m_sLoopProps[ii] ; }

      bRtn &= m_lClosedLoopCnt          == crOther.m_lClosedLoopCnt ; 
      bRtn &= m_lOuterLoopCnt           == crOther.m_lOuterLoopCnt ;

      bRtn &= m_bBadOuterLoopOrder      == crOther.m_bBadOuterLoopOrder ;
      bRtn &= m_lBadOrient_LoopCnt      == crOther.m_lBadOrient_LoopCnt ;               
      bRtn &= m_lBadNoArea_LoopCnt      == crOther.m_lBadNoArea_LoopCnt ;              
      bRtn &= m_lBadNested_LoopCnt      == crOther.m_lBadNested_LoopCnt ;     
      bRtn &= m_bBadUnpairedNoArea_Loop == crOther.m_bBadUnpairedNoArea_Loop ;
      bRtn &= m_lBadClosed3d_LoopCnt    == crOther.m_lBadClosed3d_LoopCnt  ; 
      bRtn &= m_lBadClosedPtr_LoopCnt   == crOther.m_lBadClosedPtr_LoopCnt ; 
      bRtn &= m_lBadMissingPoles        == crOther.m_lBadMissingPoles ;  

    } // end SetProps(SM_PROPSTAGE_3) from m_sLoopProps

  // all done
  return(bRtn) ;
  
} // end SmFaceProps::operator== equality operator

/*******************************************************************//**
PURPOSE: SmFaceProps Destructor

NOTES: 
***********************************************************************/
 SmFaceProps::~SmFaceProps()                                
 { 
   if(m_pCrvClassU)    { delete m_pCrvClassU ; m_pCrvClassU = NULL ; m_sRawTouchListU.ReSet() ; m_sDoneTouchListU.ReSet() ; }
   if(m_pCrvClassV)    { delete m_pCrvClassV ; m_pCrvClassV = NULL ; m_sRawTouchListV.ReSet() ; m_sDoneTouchListV.ReSet() ; }
   if(m_pLooptreeRoot) { delete m_pLooptreeRoot ; m_pLooptreeRoot = NULL ; }
   ReSet() ;
   m_cpContext = NULL ; 

 } // end SmFaceProps destructor

/*******************************************************************//**
PURPOSE: Set SmFaceProps Context

NOTES: Set Context value in contained SmCurveClassifications 
       (can't be defined in the SmFaceProps.h file)
***********************************************************************/
void SmFaceProps::SetContext
 (const SmContext * cpContext) 
{ 
  m_cpContext = cpContext ;
  if(m_pCrvClassU) { m_pCrvClassU->SetContext(cpContext) ; }
  if(m_pCrvClassV) { m_pCrvClassV->SetContext(cpContext) ; }

} // end SmFaceProps::SetContext

/*******************************************************************//**
PURPOSE: SmFaceProps initialize all member values

NOTES: Resets all values including and following the eClearPropStage input value. 
***********************************************************************/
void SmFaceProps::ReSet                                                                          
 (SmPropStageType eClearPropStage) // in : SM_PROPSTAGE_0 = clear constructor values     and
                                   //      SM_PROPSTAGE_GAPS = clear SetProps_Gaps() vals and 
                                   //      SM_PROPSTAGE_2 = clear SetProps_Stage2() vals and
                                   //      SM_PROPSTAGE_3 = clear SetProps_Stage3() vals
                                   //      default:[SM_PROPSTAGE_0]
{
  // locals
  ULONG ii ;

  // Clear General Properties

  m_ePropStage      =   (eClearPropStage <= SM_PROPSTAGE_0) ? SM_PROPSTAGE_0
                      : (eClearPropStage <= SM_PROPSTAGE_GAPS) ? SM_PROPSTAGE_0
                      : (eClearPropStage <= SM_PROPSTAGE_2) ? SM_PROPSTAGE_GAPS
                      : (eClearPropStage <= SM_PROPSTAGE_3) ? SM_PROPSTAGE_2 
                      : m_ePropStage ;
  // m_cpContext = NULL ; // Leave Context alone

  m_bBadHasNotYetProb = FALSE ;
  m_eNotYetFaceType   = SM_HNF_NO_PROBS;

  // Set in SmHealData::Get_DegenFaces() // gwc note: review this assignment. Maybe it should be skipped since its not a SetProps value.
  m_bBadDegenFace = UNSURE ; 

  // when asked - clear SetState1 _Props values
  if(eClearPropStage <= SM_PROPSTAGE_GAPS)
    {
      // Face and Tolerance
      m_pFace     = NULL ; 
      m_lFaceIndx = SM_UNDEF_ULONG ;

      // tolerances
      m_sZoneTol3d     = SM_UNDEF_DOUBLE ;
      m_sOrigZoneTol3d = SM_UNDEF_DOUBLE ;
               
      // Gap3d
      m_bBadSmallZoneTol3d = UNSURE ;
      m_bBadLargeZoneTol3d = UNSURE ;
    } // end eClearPropStage <= SM_PROPSTAGE_0 check

  // when asked - clear SetProps_BeforeSplitEdge values
  if(eClearPropStage <= SM_PROPSTAGE_2)
    {
      // Face->Surface 
      m_cpSurface  = NULL ;
      m_sNaturalUVDomain.Init() ;
  
      // Major Face->Surface properties that force the face to be checked for problems and healing 
      m_eSheet = SM_MP_UNDEF ;

      // Closed pFace->Surface classification
      m_bClosedU = UNSURE ;
      m_bClosedV = UNSURE ;
      m_bClosedSurf = UNSURE ;
      m_eClosedValU = SM_CT_UNDEFINED ; 
      m_eClosedValV = SM_CT_UNDEFINED ; 

      // Singular pFace->Surface classification                  
      m_ePoles         = SM_MP_UNDEF ;
      m_lPoles         = SM_UNDEF_ULONG ;   // SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX
      m_lApproxPoles   = SM_UNDEF_ULONG ;   // SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX
      m_lNoVertexPoles = SM_UNDEF_ULONG ;   // SM_SS_NONE or an orof SM_SS_UMIN SM_SS_VMIN SM_SS_UMAX SM_SS_VMAX
  
      { // m_sPolePoints array
        m_sPolePoints.SetSize(4) ;  // PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                    //      NonSingular side values set to SmPoint3d::SetUninitialized()
        for(ii=0;ii<m_sPolePoints.GetSize();ii++)
          { m_sPolePoints[ii].SetUninitialized() ; }

      } // end m_sPoleNormals array

      { // m_sPoleNormals array
        m_sPoleNormals.SetSize(4) ; // PoleNormal[4] array, ordered :[UMinNrml, VMinNrml, UMaxNrml, VMaxNrml]
                                    //      NonSingular side values set to SmVector3d::SetUninitialized()
        for(ii=0;ii<m_sPoleNormals.GetSize();ii++)
          { m_sPoleNormals[ii].SetUninitialized() ; }

      } // end m_sPoleNormals array

      m_lBadFlatCorners = SM_UNDEF_ULONG ;         // SM_FC_NONE or an orof: SM_FC_UMIN_VMIN, SM_FC_UMIN_VMAX, SM_FC_UMAX_VMIN, SM_FC_UMAX_VMAX

      { // m_sFlatCornerPoints array
        m_sFlatCornerPoints.SetSize(4) ; // array[4] of FlatCorner locs[UMinVMin, UMinVax, UMaxVMin, UMaxVmax]
                                         //         NonFlatCorner points set to Uninit
        for(ii=0;ii<m_sFlatCornerPoints.GetSize();ii++)
          { m_sFlatCornerPoints[ii].SetUninitialized() ; }

      } // end m_sPoleNormals array

      // when pFace->Surface is Closed: The following are set in SmFaceProps::SetProps_Stage2() by SmFace::HasSeamProblem()
      if(m_pCrvClassU) { delete m_pCrvClassU ; m_pCrvClassU = NULL ; m_sRawTouchListU.ReSet() ; m_sDoneTouchListU.ReSet() ; }
      if(m_pCrvClassV) { delete m_pCrvClassV ; m_pCrvClassV = NULL ; m_sRawTouchListV.ReSet() ; m_sDoneTouchListV.ReSet() ; }

      m_bBadCrossedSeam  = FALSE ;
      m_eBadCrossedSeam  = SM_SP_UNKNOWN ;

      m_bBadMissingSeam  = FALSE ;
      m_eBadMissingSeam  = SM_SP_UNKNOWN ;

      m_bBadNearMissSeam = FALSE ;
      m_eBadNearMissSeam = SM_SP_UNKNOWN ;

    } // end eClearPropStage <= SM_PROPSTAGE_GAPS check

  m_eBeenThroughMoveSeam         = SM_TRY_NONE ;
  m_eBeenThroughSplitEdgesAtSeam = SM_TRY_NONE ;
  m_eBeenThroughSplitFacesAtSeam = SM_TRY_NONE ;

  // when asked - clear SetProps_AfterSplitEdge values
  if(eClearPropStage <= SM_PROPSTAGE_3)
    {
      // values that are computed only after FaceEdges have been split at MissingSeams
      if(m_pLooptreeRoot) { delete m_pLooptreeRoot ; m_pLooptreeRoot = NULL ; }
      m_sLoopProps.ReSet() ; 
      
      m_lClosedLoopCnt          = SM_UNDEF_ULONG ;
      m_lOuterLoopCnt           = SM_UNDEF_ULONG ;

      m_bBadOuterLoopOrder      = UNSURE ;
      m_lBadOrient_LoopCnt      = SM_UNDEF_ULONG ;
      m_lBadNoArea_LoopCnt      = SM_UNDEF_ULONG ;
      m_lBadNested_LoopCnt      = SM_UNDEF_ULONG ;
      m_bBadUnpairedNoArea_Loop = UNSURE ;

      m_lBadClosed3d_LoopCnt    = SM_UNDEF_ULONG ; 
      m_lBadClosedPtr_LoopCnt   = SM_UNDEF_ULONG ; 
      m_lBadMissingPoles        = SM_UNDEF_ULONG ;  
    
    } // end eClearPropStage <= SM_PROPSTAGE_2 check

  // when asked - clear
    if(eClearPropStage > SM_PROPSTAGE_3)
    {
      // no values here yet
    } // end eClearPropStage <= SM_PROPSTAGE_3 check

} // end SmFaceProps::ReSet

/*******************************************************************//**
PURPOSE: Standardize calls to SetProps_Stage_i where a call
         to SetProps_StageI makes sure all the SetProps_StageJ calls
         have been made

NOTES:  
***********************************************************************/
SmStatus SmFaceProps::SetProps
 (SmPropStageType  eSetPropStage,  // in : oneof 
                                   //      SM_PROPSTAGE_GAPS = SetProps_Gaps()
                                   //        (sets: m_pFace               m_lFaceIndx
                                   //               m_sZoneTol3d          m_sOrigZoneTol3d          
                                   //               m_bBadSmallZoneTol3d  m_bBadLargeZoneTol3d)  
                                   //      SM_PROPSTAGE_2 = SetProps_Stage2()
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
                                   //      SM_PROPSTAGE_3 = SetProps_Stage3()
                                   //        (sets: m_sLoopProps //  with call m_sLoopProps[ii].SetProps(pLoop, *this) ; 
                                   //                            //  that sets: SmLoopProps::m_cpLoop             m_bDegen_Loop  
                                   //                            //                        ::m_lEdgeCnt           m_bArea_Loop        
                                   //                            //                        ::m_lEdgeuseCnt        m_eContainmentType  
                                   //                            //                        ::m_bClosed3d          m_eLoopOrient       
                                   //                            //                        ::m_bClosedPtrs        m_eDesiredLoopOrient
                                   //                            //                        ::m_lSeamCrossingCntU  m_lSeamCrossingCntV
                                   //                            //                        ::m_lSeamWindingCntU   m_lSeamWindingCntV
                                   //                            //                        ::m_eWindingOrientU    m_eWindingOrientV
                                   //                            //                        ::m_bGoodOrient        m_bGoodClosed       
                                   //               m_lClosedLoopCnt       m_lOuterLoopCnt          
                                   //               m_bBadOuterLoopOrder   m_lBadOrient_LoopCnt     
                                   //               m_lBadNoArea_LoopCnt   m_lBadNested_LoopCnt     
                                   //               m_lBadMissingPoles     m_bBadUnpairedNoArea_Loop
                                   //               m_lBadClosed3d_LoopCnt m_lBadClosedPtr_LoopCnt)
  const SmFace   * cpFace,         // in : TgtFace
  ULONG            lFaceIndx,      // in : Associated FaceIndx within managing SmHeadData::m_sTgtFaces listSmHealerOpType   eExeHealOp,     // in : Max Healer Operation executed on this Face and Face->Brep
  SmHealerOpType   eExeHealOp,     // in : Max Healer Operation executed on this Face and Face->Brep
  ULONG            lOptLabel)      // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
{ 
  // init rtn
  SmStatus sRtn = SM_SUCCESS ;

#ifdef SM_DEBUG_CODE
// ULONG GWC_CHANGE_NEXT_LINE_TO_FALSE_BEFORE_RELEASE ; 
SmBoolean bDebugMe = FALSE ;
SmPropStageType ePropStageTgt = SM_PROPSTAGE_2a ; 
  if(bDebugMe && (eSetPropStage >= ePropStageTgt) && (lOptLabel == 18))
    {
      this->Dump(lOptLabel) ; 
      SmBrep * pBrep = m_pFace ? m_pFace->GetBrep() : NULL ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      this->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                             // sCrossedSeamColor   - def:[ 1, 0, 1]
                                             // sMissingSeamColor   - def:[ 1, 0, 0]
                                             // sMissingPoleColor   - def:[ 0, 0, 1]
                                             // sBadOrientLoopColor - def:[.3,.3,.3]
                                             // sNoArea_LoopColor   - def:[ 0, 0, 0]
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // SetProps_Gaps
  if(   (eSetPropStage == SM_PROPSTAGE_GAPS)
     || (eSetPropStage > SM_PROPSTAGE_GAPS && HasProps(SM_PROPSTAGE_GAPS, cpFace) == FALSE)) 
    { sRtn = SetProps_Gaps(cpFace,lFaceIndx,eExeHealOp,lOptLabel) ; }

  // SetProps_Stage2
  if(   (eSetPropStage == SM_PROPSTAGE_2)
     || (eSetPropStage > SM_PROPSTAGE_2 && HasProps(SM_PROPSTAGE_2, cpFace) == FALSE)) 
    { sRtn = SetProps_Stage2(cpFace,lFaceIndx,eExeHealOp,lOptLabel) ; }

  // SetProps_Stage3
  if(   (eSetPropStage == SM_PROPSTAGE_3)
     || (eSetPropStage > SM_PROPSTAGE_3 && HasProps(SM_PROPSTAGE_3, cpFace) == FALSE))
    { sRtn = SetProps_Stage3(cpFace,lFaceIndx,eExeHealOp,lOptLabel) ; }

  // all done
  return sRtn ;

} // end SmFaceProps::SetProps

/*******************************************************************//**
PURPOSE: Set all member values to be cached after running
         Fix_CoinEdges()

NOTES: 1. requires the existence of cpFace and cpFace->Surface
       2. FaceProps values are calculated in three groups:

         a. SetProps_Gaps()   : calc GapSize dependent prop vals 
         b. SetProps_Stage2(): calc prop vals independent of Edgeuse data
         c. SetProps_Stage3() : calc prop vals dependent upon Edgeuse data

          Edgeuse UVTrimCurve creation is notoriously sensitive to Edge/Face database 
          problems.  UVTrimCurve creation is robust once the defining Edge
          and Face has been processed enough so that the Edgeuses' Edge->Curve drops 
          cleaning to the Edgeuses' Face->Surface without crossing any Surface 
          boundaries. Intersections detection needed to identify and fix
          BoundaryCrvs crossing Seams are sensitive to bad topological object 
          tolerance values.
          
          At the beginning of the Heal process, SetProps_Gaps() is called
          followed by 
          SetProps_Stage2() is called 
          to identify database problems that don't depend on UVTrimCurve data.  
          After the Heal process competes the "SplitEdgesAtSeams" step, 
          it becomes possible to depend on UVTrimCurves and the method 
          SetProps_Stage3() is called to finish checking for the rest of 
          the problems that the HealBrep() heals.
***********************************************************************/
SmStatus SmFaceProps::SetProps_Gaps
  (const SmFace * cpFace,     // in : target Face
   ULONG          lFaceIndx,  // in : Associated FaceIndx within managing SmHeadData::m_sTgtFaces list
   SmHealerOpType eExeHealOp, // NotUsed: in : Max Healer Operation executed on this Face and Face->Brep
   ULONG          lOptLabel)  // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
                              // sets: FaceProps: 
                              //        m_pFace               m_lFaceIndx
                              //        m_sZoneTol3d          m_sOrigZoneTol3d          
                              //        m_bBadSmallZoneTol3d  m_bBadLargeZoneTol3d  
{
  SM_REF1(eExeHealOp) ;
  // no work - already done or being done
  if(m_ePropStage >= SM_PROPSTAGE_GAPSa)
    { return(SM_SUCCESS) ; }

  // Init Props about to be computed - also resets SM_PROPSTAGE_2 and SM_PROPSTAGE_3 Values
  ReSet(SM_PROPSTAGE_GAPS) ; 

  // increment PropState here - prevent possible infinite loops
  m_ePropStage = SM_PROPSTAGE_GAPSa ; // SM_PROPSTAGE_GAPSa = during SmFaceProps::SetProps_Gaps() call

  // locals and queries
  m_pFace     = (SmFace *)cpFace ;  // forward ptr: FaceProps->Face
  m_lFaceIndx = lFaceIndx ; 

  m_sZoneTol3d         = SmTol::GetZoneTol3d(m_pFace) ; 
  if(m_sOrigZoneTol3d == SM_UNDEF_DOUBLE)
    { m_sOrigZoneTol3d = m_sZoneTol3d ; }

  //SmZoneTol3d  sDefZoneTol3d  = SmTol::GetZoneTol3d(m_cpContext) ;      // DefTol is Tol with no Gaps
  SmZoneTol3d  sCalcZoneTol3d = SmTol::CalcFaceZoneTol3d(m_cpContext) ; // CalcZone is DesiredTol for obj with current gaps
  //SmXSectTol3d sXSectTol3d    = SmTol::GetXSectTol3d(m_sZoneTol3d, m_sZoneTol3d) ;

  // ZoneTol3d for current Gap3d sizes
  m_bBadSmallZoneTol3d = (m_sZoneTol3d <= sCalcZoneTol3d - SM_EFF_ZERO) ;
  m_bBadLargeZoneTol3d = (m_sZoneTol3d >= sCalcZoneTol3d + SM_EFF_ZERO) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE ; 
  if(bDebugMe2)
    {
      this->Dump(lOptLabel) ; 
      SmBrep * pBrep = m_pFace ? m_pFace->GetBrep() : NULL ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      this->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                             // sCrossedSeamColor   - def:[ 1, 0, 1]
                                             // sMissingSeamColor   - def:[ 1, 0, 0]
                                             // sMissingPoleColor   - def:[ 0, 0, 1]
                                             // sBadOrientLoopColor - def:[.3,.3,.3]
                                             // sNoArea_LoopColor   - def:[ 0, 0, 0]
      sm_GraphicsLoop() ;
    }
#else
  SM_REF1(lOptLabel);
#endif // SM_DEBUG_CODE

  // all done
  m_ePropStage = SM_PROPSTAGE_GAPS ; // SM_PROPSTAGE_GAPS = after SmFaceProps::SetProps_Gaps() call
  return(SM_SUCCESS) ;

} // end SmFaceProps::SetProps_Gaps

/*******************************************************************//**
PURPOSE: Set all member values to be cached after running
         Fix_TolSizes()

NOTES: 1. requires the existence of cpFace and cpFace->Surface
       2. FaceProps values are calculated in three groups:

          a. SetProps_Gaps(): calc GapSize dependent prop vals
          b. SetProps_Stage2(): calc prop vals independent of Edgeuse data
          c. SetProps_Stage3(): calc prop vals dependent upon Edgeuse data

          Edgeuse UVTrimCurve creation is notoriously sensitive to Edge/Face database 
          problems.  UVTrimCurve creation is robust once the defining Edge
          and Face has been processed enough so that the Edgeuses' Edge->Curve drops 
          cleaning to the Edgeuses' Face->Surface without crossing any Surface 
          boundaries. 
          
          At the beginning of the Heal process, SetProps_Stage2() is called 
          to identify database problems that don't depend on UVTrimCurve data.  
          After the Heal process competes the "SplitEdgesAtSeams" step, 
          it becomes possible to depend on UVTrimCurves and the method 
          SetProps_Stage3() is called to finish checking for the rest of 
          the problems that the HealBrep() heals.
***********************************************************************/
SmStatus SmFaceProps::SetProps_Stage2
  (const SmFace * cpFace,     // in : target Face
   ULONG          lFaceIndx,  // in : Associated FaceIndx within managing SmHeadData::m_sTgtFaces list
   SmHealerOpType eExeHealOp, // in : Max Healer Operation executed on this Face and Face->Brep
   ULONG          lOptLabel)  // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
                              // sets: FaceProps:
                              //        if(m_ePropStage < SM_PROPSTAGE_GAPS) call SetProps_Gaps() to set
                              //          { m_pFace               m_lFaceIndx
                              //            m_sZoneTol3d          m_sOrigZoneTol3d          
                              //            m_bBadSmallZoneTol3d  m_bBadLargeZoneTol3d } 
                              //        m_cpSurface           m_sNaturalUVDomain
                              //        m_bSheetFace          
                              //        m_bClosedU            m_bClosedV          m_bClosedSurf
                              //        m_eClosedValU  m_eClosedValV              
                              //        m_lPoles              m_lApproxPoles      m_lNoVertexPoles
                              //        m_sPolePoints         m_sPoleNormals      
                              //        m_lBadFlatCorners     m_sFlatCornerPoints 
                              //        m_pCrvClassU          m_pCrvClassV        
                              //        m_sRawTouchListU      m_sRawTouchListV
                              //        m_sDoneTouchListU     m_sDoneTouchListV  
                              //        m_eBadMissingSeam     m_eBadCrossedSeam   m_eBadNearMissSeam
{    
  // no work - already done or being done
  if(m_ePropStage >= SM_PROPSTAGE_2a)
    { return(SM_SUCCESS) ; }

  // no work - called before SmHealData::Fix_TolSizes() has run on this Brep
  if(eExeHealOp < SM_HO_FIX_TOLSIZES)
    { return(SM_ERR) ; }

  // extra work - Get props that need to be built before these props
  if(m_ePropStage < SM_PROPSTAGE_GAPS || m_pFace != cpFace)
    {
      SER(SetProps_Gaps(cpFace, lFaceIndx, eExeHealOp, lOptLabel)) ;
    }

  // Init props about to be computed - also resets SM_PROPSTAGE_3 Values
  ReSet(SM_PROPSTAGE_2) ; 

  // increment PropState here - prevent infinite loops
  m_ePropStage = SM_PROPSTAGE_2a ; // SM_PROPSTAGE_2a = during SmFaceProps::SetProps_Stage2() call

  // manage contained memory - CrvClass U and V
  if(m_pCrvClassU == NULL) { m_pCrvClassU = new (m_cpContext) SmCurveClassification() ; }
  else                     { m_pCrvClassU->ReSet() ; }
  m_pCrvClassU->SetOKToSetTolerance(FALSE) ;
  if(m_pCrvClassV == NULL) { m_pCrvClassV = new (m_cpContext) SmCurveClassification() ; }
  else                     { m_pCrvClassV->ReSet() ; }
  m_pCrvClassV->SetOKToSetTolerance(FALSE) ;
      
  // locals and queries
  ULONG ii, jj ;

  SmXSectTol3d sXSectTol3d    = SmTol::GetXSectTol3d(m_sZoneTol3d, m_sZoneTol3d) ;
  m_cpSurface                 = m_pFace->GetSurface() ;
  m_sNaturalUVDomain          = m_cpSurface->GetNaturalUVDomain() ; 
                              
  SmFaceuse * pUpFaceuse = NULL ;
  SmFaceuse * pDownFaceuse = NULL ;
  m_pFace->GetFaceuses(pUpFaceuse, pDownFaceuse) ;

  // Major Face->Surface properties that force the face to be checked for problems and healing
  m_eSheet        = (pUpFaceuse->GetShell() == pDownFaceuse->GetShell()) ? SM_MP_HASPROP : SM_MP_NOPROP ;

  // Closed pFace->Surface classification: m_bClosedU, m_bClosedV, m_eClosedValU, m_eClosedValV, 
  m_bClosedU = m_cpSurface->IsClosed(m_sNaturalUVDomain, SM_SP_U, &sXSectTol3d.val, &m_eClosedValU) ;
  m_bClosedV = m_cpSurface->IsClosed(m_sNaturalUVDomain, SM_SP_V, &sXSectTol3d.val, &m_eClosedValV) ;

  // Singularity classification: m_lPoles, m_lApproxPoles, m_sPolePoints ,m_sPoleNormals                     
  m_lPoles = m_cpSurface->GetSingularities(&m_sPolePoints, 
                                           &m_sPoleNormals, 
                                           &m_lApproxPoles) ;

   // Closed pFace->Surface = Tori or Spheres
   m_bClosedSurf =   (m_bClosedU && m_bClosedV)  // torus
                  || (m_bClosedV && (m_lPoles & SM_SS_UMIN) && (m_lPoles & SM_SS_UMAX))   // Sphere
                  || (m_bClosedU && (m_lPoles & SM_SS_VMIN) && (m_lPoles & SM_SS_VMAX)) ; // Sphere

  // find any poles not marked by a vertex (this can be okay if the pole is not 'in' or 'on the boundary' of the Face
  // m_lNoVertexPoles
  if(m_lPoles == SM_SS_NONE) { m_ePoles = SM_MP_NOPROP ; }
  else // (m_lPoles != SM_SS_NONE)
    {
      SmTArray<SmVertex*> sVertices ;
      m_pFace->GetVertices(sVertices) ;
      ULONG lFoundVertices = 0 ; 
      double dZoneTol3dSq = m_sZoneTol3d * m_sZoneTol3d ; 

      // for every Pole
      for(ii=0;ii<4;ii++)
        {
          // no work - no pole
          if(    ((ii == 0) && ((m_lPoles & SM_SS_UMIN) == 0))
             ||  ((ii == 1) && ((m_lPoles & SM_SS_VMIN) == 0))
             ||  ((ii == 2) && ((m_lPoles & SM_SS_UMAX) == 0))
             ||  ((ii == 3) && ((m_lPoles & SM_SS_VMAX) == 0))
            ) { continue ; }

          // for every vertex
          for(jj=0;jj<sVertices.GetSize();jj++)
            {
              double dDistSq = sVertices[jj]->GetPoint().DistanceBetweenSquared(m_sPolePoints[ii]) ;

              // note any poles that we find marked with a vertex
              if(dDistSq < dZoneTol3dSq)
                {
                  lFoundVertices |= (  (ii == 0) ? SM_SS_UMIN 
                                     : (ii == 1) ? SM_SS_VMIN 
                                     : (ii == 2) ? SM_SS_UMAX 
                                     : (ii == 3) ? SM_SS_VMAX : 0) ;
                  break ;
                } // end found a vertex marking the pole
            } // end iter every vertex
        } // end iter every pole

      // set the missing pole value
      m_lNoVertexPoles = (~lFoundVertices & m_lPoles) ;
      m_ePoles         = (m_lNoVertexPoles == SM_SS_NONE) ? SM_MP_OKAY : SM_MP_PROB ; 

    } // end Pole existence check

  // Flat Corner Surface Classification: m_lBadFlatCorners 
  m_lBadFlatCorners  = m_cpSurface->GetFlatCorners(&m_sFlatCornerPoints) ;

  // when pFace->Surface is Closed: The following are set here with this next SmFace::HasSeamProblem() call.
  // m_pCrvClassU, m_sRawTouchListU, m_sDoneTouchListU
  // m_pCrvClassV, m_sRawTouchListV, m_sDoneTouchListV
  // m_eBadMissingSeam, m_eBadCrossedSeam, m_eBadNearMissSeam
  if(m_bClosedU == TRUE || m_bClosedV == TRUE)
    {
      m_pFace->HasSeamProblem
        (
         *m_pCrvClassU,       // i/o: ConstU_Seam/Face Classification <br>
         *m_pCrvClassV,       // i/o: ConstV_Seam/Face Classification <br>
          m_eBadCrossedSeam,  // out: SM_SP_U = periodic U bndry seam crossed by Face->Edges <br> 
                              //      SM_SP_V = periodic V bndry seam crossed by Face->Edges <br>
                              //      SM_SP_BOTH = both periodic U and V bndry seams crossed by Face->Edges <br>
                              //      SM_SP_NEITHER = no seams crossed by Face->Edges <br>
          m_eBadMissingSeam,  // out: SM_SP_U = periodic U bndry seam not represented by Face->Edges <br>
                              //      SM_SP_V = periodic V bndry seam not represented by Face->Edges <br>
                              //      SM_SP_BOTH = both periodic U and V bndry seams not represented by Face->Edges <br>
                              //      SM_SP_NEITHER = both periodic U and V bndry seams are represented by Face->Edges <br>
          m_eBadNearMissSeam, // out: For future use once Fix_BadGaps() gets built
                              //      SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam  
                              //      SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam 
                              //      SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams  
                              //      SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams  
                              //      NearMiss Topo connections are valid - future Fix_BadGaps() will refine geom to tighten gaps from NearMiss to Exact
          FALSE,              // in : TRUE = Only classify against the Face outer loop, used internally on partially built faces <br>
                              //      FALSE= classify against all Face->Loops, default:[FALSE] <br>
                              //      NOTE: Always use FALSE when CrvClassif will be used to insert Ivls into a Face <br>
          &m_bClosedU,        // in : TRUE=known to be U periodic, FALSE=not, NULL to ignore
          &m_bClosedV,        // in : TRUE=known to be V periodic, FALSE=not, NULL to ignore
          this                // in : optional pre-cached set of this-Face properites, NULL to ignore
                              //      gets access to m_sRawTouchListU   m_sRawTouchListV
                              //                     m_sDoneTouchListU  m_sDoneTouchListV
                              //  TODO: see if input args m_bClosedU, m_bClosedV, and the m_eBad... flags can be passed through the SmFaceProps argument.
        ) ;

       // set the boolean error flags
       m_bBadMissingSeam  = (m_eBadMissingSeam  != SM_SP_NEITHER) ; // TRUE = Face has a problem
       m_bBadCrossedSeam  = (m_eBadCrossedSeam  != SM_SP_NEITHER) ; // TRUE = Face has a problem
       m_bBadNearMissSeam = (m_eBadNearMissSeam != SM_SP_NEITHER) ; // TRUE = Face has a problem

    } // end Surface is closed check

  // not used - instead started tracking m_lBadMissingPoles
  // // remember which (if any) singularities are on the (any) MissingSeam: m_lPolesOnMissingSeam
  // SmBoolean bMissSeamU = (m_eBadMissingSeam == SM_SP_U || m_eBadMissingSeam == SM_SP_BOTH) ;
  // SmBoolean bMissSeamV = (m_eBadMissingSeam == SM_SP_V || m_eBadMissingSeam == SM_SP_BOTH) ;
  // m_lPolesOnMissingSeam =   (bMissSeamV ? m_lPoles & (SM_SS_UMIN | SM_SS_UMAX) : 0)
  //                         | (bMissSeamU ? m_lPoles & (SM_SS_VMIN | SM_SS_VMAX) : 0) ;
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE ; 
  if(bDebugMe2)
    {
      this->Dump(lOptLabel) ; 
      SmBrep * pBrep = m_pFace ? m_pFace->GetBrep() : NULL ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      this->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                             // sCrossedSeamColor   - def:[ 1, 0, 1]
                                             // sMissingSeamColor   - def:[ 1, 0, 0]
                                             // sMissingPoleColor   - def:[ 0, 0, 1]
                                             // sBadOrientLoopColor - def:[.3,.3,.3]
                                             // sNoArea_LoopColor   - def:[ 0, 0, 0]
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  m_ePropStage = SM_PROPSTAGE_2 ; // SM_PROPSTAGE_2 = after SmFaceProps::SetProps_Stage2() call
  return(SM_SUCCESS) ;

} // end SmFaceProps::SetProps_Stage2

/*******************************************************************//**
PURPOSE: Set all member values to be cached after running
         Fix_SplitEdgesAtSeams()

RETURNS: SM_SUCCESS              = normal operation - properties set and problem lists updated
         SM_ERR_NOTYET_HEAL_FACE = Found a Face with a problem not yet supported - output in unknown state

NOTES: 1. requires all the vals set by the SetProps_Stage2() call.    
       2. Requires the Face and Edges have been healed at least through
          the "SplitEdgesAtSeams" step.

       3. SetProps_Stage3() primarily calcs property values needed 
          for finding the current and desired orientations and containment of Loops.  
          When working with MissingSeam Faces - these properties 
          characterize Area and NoArea_Loops and their orientations.  
          See long Loop orientation note included below. 
          (The note is also contained in SmBrep::HealBrep() header.)

       4. FaceProps values are calculated in three groups by three methods:

          a. SetProps_Gaps()   : calc GapSize dependent prop vals
          b. SetProps_Stage2(): calc prop vals independent of Edgeuse data
          c. SetProps_Stage3() : calc prop vals dependent upon Edgeuse data

          Edgeuse UVTrimCurve creation is notoriously sensitive to Edge/Face database 
          problems.  UVTrimCurve creation is robust once the defining Edge
          and Face have been processed enough so that the Edgeuses' Edge->Curve drops 
          cleaning to the Edgeuses' Face->Surface without crossing any Face or Surface 
          boundaries. 
          
          At the beginning of the Heal process, SetProps_Stage2() is called 
          to identify database problems that don't depend on Edgeuse and UVTrimCurve data.  
          After the Heal process completes the "SplitEdgesAtSeams" step, 
          it becomes possible to depend on UVTrimCurves and the method 
          SetProps_Stage3() is called to finish checking the Face  
          for the rest of the problems that the HealBrep() heals.

ORIENT: 5. A NOTE ON LOOP ORIENTATIONS FOR LOOPS ON FACES WITH MISSING SEAMS
NOTE  :    When missing a seam: 
            A loop can be Closed or Open.
            A Closed loop can either be With or Without Area (called Area_Loops and NoArea_Loops). 
            A Closed Area_Loop is a valid loop that partitions the UVDomain into 2 areas where
              one of those areas will be isolated from the surface's natural UVBoundaries in the sense 
              that a path starting at a point of the 'isolated' area will have to cross the loop boundary 
              before exiting the UVDomain.
            A Closed NoArea_Loop is an invalid loop only possible on a MissingSeamFace. This Loop will also 
              partition the UVDomain into two areas but neither of those will be 'isolated' areas in that a path 
              from any point in either area can always be constructed that exits the UVDomain without 
              crossing the loop boundary.
          
          Observation: 1. An Area_Loop will cross the MissingSeam an even number of times  e.g.:[0,2,4,..]
                          A NoArea_Loop will cross the MissingSeam an odd number of times. e.g.:[1,3,5,..]
                          An inexpensive predicate for classifying a ClosedLoop on a Face with a MissingSeam
                          as an Area_Loop or a NoArea_Loop can be built by counting the number of times the
                          loop crosses the MissingSeam.
                       2. Example of a pair of NoArea_Loops being connected into proper CCW Area_Loops
                          by Splitting the Face at the Seam.  In this case a NoArea_Loop with 3 SeamCrossings
                          is connected to a NoArea_Loop with 1 SeamCrossing by merging two segments
                          of the SeamCurve into the Face resulting in 2 New SeamEdges, 2 CCW Area_Loops, 
                          and 2 Faces.
          
                                 Period 0         Period 1      Period 0         Period 1   
                                Seam             Seam          Seam             Seam        
                                +----------------+----...      +----------------+----...    
                                |                |             |// UVRegion_C //|           
          Seam Croosings:[3]+---#<_            _<#<_           #<_////////////_<#<_         
                             \  |  `o      _.<'  |  `o         |\\`o//////_.<'\\|\\`o       
                              +-#>'    _.<'    _>#>'           #>'////_.<'\\\\_>#>'         
                               \|  _.<'      o'  |             |//_.<' \\\\\<'  |           
                  NoArea_Loop2--#<'            `<#             #<'\\\\\\\\\\\\`<#--NoArea_Loop2            
             [SM_OT_LOWERDOMAIN]|                |             |\\ UVRegion_B \\|    orient:[SM_OT_LOWERDOMAIN]        
                                |                |             |\\\\\\\\\\\\\\\\|           
                  NoArea_Loop1--#-->--->--->--->-#->--...      #-->--->--->--->-#--NoArea_Loop1           
             [SM_OT_UPPERDOMAIN]|                |             |////////////////|    orient:[SM_OT_UPPERDOMAIN]       
                                |                |             |// UVRegion_A //|           
                                +----------------+----...      +----------------+----...
                   NoArea_Loop2 Orientation:[SM_OT_LOWERDOMAIN]: RightHandDomain = UVRegion_C
                   NoArea_Loop2                                  LeftHandDomain = UVRegion_B \_ Future interior of 
                   NoArea_Loop1 Orientation:[SM_OT_UPPERDOMAIN]: LeftHandDomain = UVRegion_B /  Face_1 and Face_2
                   NoArea_Loop1                                  RightHandDomain = UVRegion_A    After merging MissingSeam
          
              Add MissingSeam: Merge 2 SeamSegments as NewEdges, Split Face into 2 Faces. 
                                
                                 Period 0         Period 1      Just the
                                Seam             Seam             Seam
                                +----------------+----...          +
                                |     Outside    |                 |
                                #<_            _<#<_               #--]
                        Face_2--$v `o      _.<' ^$v `o             $   }- New Seam Edge_2
                                #>'    _.<'    _>#>' Face_2        #--]
                                |  _.<'      o'  |                 |
                                #<'            `<#                 #--]
                                $v    Face_1    ^$                 $   }- New Seam Edge_1
                                $v              ^$                 $  ]
                                #-->--->--->--->-#->--...          #--]
                                |                |                 |
                                |     Outside    |                 |
                                +----------------+----...          +
                            
                    note: 1. The MissingSeam Curve is split into 5 segments 
                             by the NoArea_Loop SeamCrossings.  Merging the 
                             first inside Seam_Segment creates New Seam Edge_1 
                             connecting NoArea_Loop_1 and NoArea_Loop_2 into 
                             one CCW OuterLoop for Face_1. 
                              (this Face_1 Loop still crosses the seam)
                    note: 2. Merging the second inside Seam_Segment creates new
                             Seam Edge_2 which Splits the one CCW Loop into
                             2 CCW Loops while also Spliting Face_1 into  
                             Face_1 and Face_2. 
                              (Face_1 and Face_2 Loops do not cross the seam)
          
          Computing the desired orientation of a loop on a face without a MissingSeam is easy.
          The OuterLoop must be oriented CCW, and all InnerLoops must be oriented CW.
          
          The desired orientation of a loop on a face with a MissingSeam is more complicated.
          If the Loop is a Closed Area_Loop, then it follows the same orientation rules as for
          any loop on a valid Face; the OuterLoop is CCW and the InnerLoops are CW.
          
          A pair of invalid Closed NoArea_Loops can be turned into one valid Area_Loop CCW  outerloop
          by adding the MisingSeamEdge to the Face.  For another example, consider a cylinder bounded 
          by upper and lower circles but missing the seam.  Both circles will be 
          Single-SeamCrossing NoArea-Loops .  When their orientations are consistent with one another 
          together they can define a UVDomain SubRegion that is 'isolated' in that a path starting at 
          any point in the 'isolated' region will have to cross one of the two NoArea-Loops before 
          exiting the UVDomain (crossing the seam does not count as exiting the domain).  
          Adding the MissingSeam will connect this matched pair of NoArea_Loops into a single valid 
          CCW OuterLoop. (Observation: any Loop that includes a SeamEdgw must be an OuterLoop.)
          
          We can set the orientations of closed NoArea_Loops so that when they are connected 
          by adding a missing seam later they will create a properly oriented OuterLoop. Each 
          Single SeamCrossing NoArea-Loop divides the UVDomain into an 'upper' and a 'lower' half.  
          Using the Edgeuse's LeftHand rule for 'inside' and the UV parameterization of the domain to 
          name the two sides we define the following for Single SeamCrossing NoArea-Loop orientations:
            Rule: a SM_OT_SAME oriented NoArea-Loop's 'Lefthand-Inside' = 'upper' UV Partition   
                  a SM_OT_OPPOSITE oriented NoArea-Loop's 'LeftHand-Inside' = 'lower' UV Partition.
          
          If we order theNoArea_Loops of a Face with a Missing seam from lowest partition
          param value to highest, we can see the requirements for NoArea-Loop orientations
          so that an addition of the MissingSeam can connect those into valid OuterLoops.
            rules:  1. NoArea_Loops must come in pairs. (a VertexLoop pole on a seam is a NoArea_Loop IsoLine in UV space)
                    2. NoArea_Loops must cross the MissingSeam.
                    3. When Single SeamCrossing NoArea_Loops come in pairs (the common case)
                      4a. The Lowest  param NoArea_Loop member of each pair must be oriented SM_OT_SAME
                      4b. The Highest param NoArea_Loop member of each pair must be oriented SM_OT_OPPOSITE
                      4c. The number of outer loops created by adding the MissingSeam to a Face with a pair
                          of properly oriented NoArea_Loops will equal the number of SeamCrossings/2. 
                          CCW OuterLoopCount After Adding SeamEdges to a pair of NoArea_Loops = SeamCrossings/2
          
          A common read problem is a Face on a disk modeled as a periodic surface with a singularity
          where both the Seam and the Pole are missing.  In these Face models its common to have
          just one NoArea-Loop.  In this case we add a Vertex at the pole which will become the
          second NoArea-Loop.  After which, adding the MissingSeam connects the two NoArea-Loops
          into a valid single OuterLoop.  The desired orientation of the one NoAreaLoop depends on
          the parameter value for the MissingPole.  If the Pole is at UVMin, then the orientation 
          needs to be SM_OT_OPPOSITE, and conversely if the Pole is at UVMax the orientation needs
          to be SM_OT_SAME.  
          
          Another common read problem is a Face on a periodic Surface with no singularities but just
          one NoArea-Loop.  There is no way to turn that loop into a valid OuterLoop by just adding
          a MissingSeam.  There are two options available for fixing the problem.  The Face might
          degenerate to a single Edge consisting of the NoArea-Loop.  Or the Face may need the addition
          of another NoArea-Loop.  There is no 'right' way to add that missing NoArea-Loop without
          changing the geometry and the topology of model.  If the part being read is intended to be
          manifold, adding the missing NoArea-Loop might require the creation and addition of another
          Face.  We will leave this problem for a later time. (perhaps that time is now for you!)
  End note 5. A NOTE ON LOOP ORIENTATIONS FOR LOOPS ON FACES WITH MISSING SEAMS
***********************************************************************/
SmStatus SmFaceProps::SetProps_Stage3
  (const SmFace * cpFace,     // in : target Face
   ULONG          lFaceIndx,  // in : Associated FaceIndx within managing SmHeadData::m_sTgtFaces list
   SmHealerOpType eExeHealOp, // in : Max Healer Operation executed on this Face and Face->Brep
   ULONG          lOptLabel)  // in : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]
                    // sets: FaceProps:
                    //        if(m_ePropStage < SM_PROPSTAGE_2) call SetProps_Stage2() to set
                    //          { m_pFace               m_lFaceIndx
                    //            m_sZoneTol3d          m_sOrigZoneTol3d          
                    //            m_bBadSmallZoneTol3d  m_bBadLargeZoneTol3d
                    //          } 
                    //        if(!IsSet_BeforeBeforeEdgeSplit()) call SetProps_Stage2() to set
                    //          { m_cpSurface           m_sNaturalUVDomain
                    //            m_bSheetFace          
                    //            m_bClosedU            m_bClosedV          m_bClosedSurf
                    //            m_eClosedValU  m_eClosedValV
                    //            m_lPoles              m_lApproxPoles      m_lNoVertexPoles
                    //            m_sPolePoints         m_sPoleNormals      
                    //            m_lBadFlatCorners     m_sFlatCornerPoints 
                    //            m_pCrvClassU          m_pCrvClassV        
                    //            m_sRawTouchListU      m_sRawTouchListV
                    //            m_sDoneTouchListU     m_sDoneTouchListV
                    //            m_eBadMissingSeam     m_eBadCrossedSeam   m_eBadNearMissSeam 
                    //          }
                    //      FaceProps set by SetProps_Stage3():
                    //        m_sLoopProps //  with call m_sLoopProps[ii].SetProps(pLoop, *this) ; 
                                           //  that sets: SmLoopProps::m_cpLoop             m_bDegen_Loop  
                                           //                        ::m_lEdgeCnt           m_bArea_Loop        
                                           //                        ::m_lEdgeuseCnt        m_eContainmentType  
                                           //                        ::m_bClosed3d          m_eLoopOrient       
                                           //                        ::m_bClosedPtrs        m_eDesiredLoopOrient
                                           //                        ::m_lSeamCrossingCntU  m_lSeamCrossingCntV
                                           //                        ::m_lSeamWindingCntU   m_lSeamWindingCntV
                                           //                        ::m_eWindingOrientU    m_eWindingOrientV
                                           //                        ::m_bGoodOrient        m_bGoodClosed       
                    //        m_lClosedLoopCnt       m_lOuterLoopCnt          
                    //        m_bBadOuterLoopOrder   m_lBadOrient_LoopCnt     
                    //        m_lBadNoArea_LoopCnt   m_lBadNested_LoopCnt     
                    //        m_lBadMissingPoles     m_bBadUnpairedNoArea_Loop
                    //        m_lBadClosed3d_LoopCnt m_lBadClosedPtr_LoopCnt
{                             
  // no work - already done or being done
  if(m_ePropStage >= SM_PROPSTAGE_3a)
    { return(SM_SUCCESS) ; }

  // no work - called before SmHealData::Fix_SplitEdgesAtSeam() has run on this Brep
  if(eExeHealOp < SM_HO_FIX_SPLITEDGE_ATSEAM)
    { return(SM_ERR) ; }

  // extra work - Get props that need to be built before these props
  if(m_ePropStage < SM_PROPSTAGE_2 || m_pFace != cpFace)
    {
      SER(SetProps_Stage2(cpFace, lFaceIndx, eExeHealOp, lOptLabel)) ;
    }

  // Init props about to be computed - also resets any future higher stage values (currently none)
  ReSet(SM_PROPSTAGE_3) ;

  // increment PropState here - prevent infinite loops
  m_ePropStage = SM_PROPSTAGE_3a ; // SM_PROPSTAGE_3a = during SmFaceProps::SetProps_Stage3() call

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump(lOptLabel) ;
    }
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii, jj, cnt ; 
  SmTArray<SmVertex*>        sVerticesU ;
  SmTArray<SmVertex*>        sVerticesV ;  // Lists of Vertices where Loop crosses a Missing Seam
  SmCurveClassification      sOptCurveClassification ; 
  SmTArray<SmLoop*>          sLoops, sClosedLoops ;
  SmTArray<SmClassifyLoopIO> sClassifyLoopIOs ;
  SmTArray<ULONG>            sOrderedNoAreaIndices ;
  SmCurveClassification      sCrvClassification ; sCrvClassification.SetContext(m_cpContext) ;
  SmCrvDirOnFaceType         eClassifyCrv_StartType = SM_CD_UNINIT ;
  SmCrvDirOnFaceType         eClassifyCrv_StopType  = SM_CD_UNINIT ;
  SmHealNotYetFaceType       eClassifiable      = SM_HNF_NO_PROBS ;

  // Get Face->Loops
  m_pFace->GetLoops(sLoops) ; 
  m_sLoopProps.SetSize(sLoops.GetSize()) ; 

  // for every Loop - Set following Loop Props marked with '*': 
  //                  (Loop Props marked with '##' are set later in this method with SmFace::ClassifyLoops() results)
  //   SmLoopProps::m_cpLoop *              m_tLoopuseType *
  //              ::m_lEdgeCnt *            m_lEdgeuseCnt *
  //              ::m_bClosed3d *           m_bClosedPtrs *
  //              ::m_bGoodClosed *         
  //              ::m_lSeamCrossingCntU *   m_lSeamCrossingCntV *
  //              ::m_lSeamWindingCntU *    m_lSeamWindingCntV *
  //              ::m_eWindingOrientU *     m_eWindingOrientV * - set to SM_OT_UNKNOWN for Loops that don't cross seams
  //              ::m_bDegen_Loop * - NotYetDone       
  //              ::m_bArea_Loop *          
  //              ::m_eContainmentType ##   m_eLoopOrient ##          
  //              ::m_eDesiredLoopOrient ## m_bGoodOrient ##       
  //    where *  = properties set in SmLoopProps::SetProps()
  //          ## = properties set later in SmFaceProps::SetProps_Stage3()
  for(ii=0;ii<sLoops.GetSize();ii++)
    {
      SmLoop * pLoop = sLoops[ii] ;
      m_sLoopProps[ii].SetContext(m_cpContext) ; 

      // Set Loop props (when m_bAreaLoop==FALSE, m_eDesiredLoopOrient && m_bGoodOrient are set in this method)
      m_sLoopProps[ii].SetProps(pLoop, *this) ; // sets: following Loop Props marked with '*'
                                                //       (Loop Props marked with '##' are set later in this method with SmFace::ClassifyLoops() results)
                                                //       SmLoopProps::m_cpLoop *              m_tLoopuseType *
                                                //                  ::m_lEdgeCnt *            m_lEdgeuseCnt *
                                                //                  ::m_bClosed3d *           m_bClosedPtrs *
                                                //                  ::m_bGoodClosed *         
                                                //                  ::m_lSeamCrossingCntU *   m_lSeamCrossingCntV *
                                                //                  ::m_lSeamWindingCntU *    m_lSeamWindingCntV *
                                                //                  ::m_eWindingOrientU *     m_eWindingOrientV * - set to SM_OT_UNKNOWN for Loops that don't cross seams
                                                //                  ::m_bDegen_Loop * - NotYetDone       
                                                //                  ::m_bArea_Loop *          
                                                //                  ::m_eContainmentType ##   m_eLoopOrient ##          
                                                //                  ::m_eDesiredLoopOrient ## m_bGoodOrient ##       
                                                //   where *  = properties set in SmLoopProps::SetProps()
                                                //         ## = properties set later in SmFaceProps::SetProps_Stage3()
      // accumulate closed loops and mark open loops as wires
      if     ( m_sLoopProps[ii].m_tLoopuseType == SmVertexuse_TYPE )
        { // include vertex loops when the vertex is on a pole and the pole is on a missing seam edge
          if(   (   m_sLoopProps[ii].m_lSeamCrossingCntU > 0
                 || m_sLoopProps[ii].m_lSeamCrossingCntV > 0)
             && ( m_sLoopProps[ii].m_bVertexLoopOnPole )) // vertex is on pole
            { sClosedLoops.Add(pLoop) ; }
        }
      else if(   m_sLoopProps[ii].m_bClosed3d
              || m_sLoopProps[ii].m_bClosedPtrs) { sClosedLoops.Add(pLoop) ; }
      else                                       { m_sLoopProps[ii].m_eContainmentType = SM_CMT_WIRELOOP ; }

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          m_sLoopProps[ii].Dump(ii) ;

          SmBrep    * pBrep    = m_pFace ? m_pFace->GetBrep() : NULL ;
          SmSurface * pSurface = m_pFace ? m_pFace->GetSurface() : NULL ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ;    if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ;    if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, .3,.3,.3) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,1,0) ;    if(pSurface) pSurface->DrawParams() ; sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ;    if(sLoops[ii]) sLoops[ii]->Draw() ; sm_GraphicsLoop() ; 
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

    } // end iter ii, every Loop getting FirstLoop props 
    
  // closed Loop count
  m_lClosedLoopCnt = sClosedLoops.GetSize() ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      Dump(lOptLabel == SM_UNDEF_ULONG ? 100 : 100 + lOptLabel) ;
    }
#endif // SM_DEBUG_CODE

  // Call SmFace::ClasifyLoops to get Loop orientations and containment for all NonWireLoop Loops
  SmStatus sClassifyStatus = m_pFace->ClassifyLoops
    (sClosedLoops,              // i/o: target loop(s), empty = GetAll Loops 
     eClassifiable,             // out: SM_HNF_NO_PROBS = SUCCESS - use the results
                                //      SM_HNF_NO_CLASSIFY_ZERO_CROSSCNT: No ClassifyCrv found to cross a TgtLoop = Problem Face - don't use the results
                                //      SM_HNF_NO_CLASSIFY_ODD_CROSSCNT : Open Surf with odd Loop Crossing Cnts   = Problem Face - don't use the results
                                //      SM_HNF_NO_LOOP_TREE_ROOT        : couldn't build SmLooptreeItem           = Problem Face - don't use the results
     sClassifyLoopIOs,          // out: associated Loop Classification Data containing:
                                //      m_pLoop ;            // Classified Loop
                                //      m_tLoopuseType ;     // oneof:   SmVertexuse_TYPE (16006)
                                //                           //          SmEdgeuse_TYPE   (16005) 
                                //                           // default:[SmUnknown_TYPE   (16999)]
                                //      m_eOrient ;          // SM_OT_SAME     = CCW (expected Area_OuterLoop dir (for NoArea_Loops SM_OT_UPPERDOMAIN)) 
                                //                           // SM_OT_OPPOSITE = CW  (expected Area_InnerLoop dir (for NoArea_Loops SM_OT_LOWERDOMAIN)) 
                                //      m_bArea_Loop ;       // TRUE = Loop is Area_Loop (normal case)                                 
                                //                           // FALSE= Loop is NoArea_Loop - a problem only seen on MissingSeamFaces   
                                //      m_eContainmentType ; // oneof: SM_CMT_OUTERLOOP,       // Area_Loop not in another, contains all UVPoints 'inside' loop  
                                //                           //        SM_CMT_INNERLOOP,       // Area_Loop in another, excludes all UVPoints 'inside' loop
                                //                           //        SM_CMT_NESTEDLOOP_EVEN, // Area_Loop (invalid) nested in InnerLoop with an even nesting depth (0,2,4) - healed to outer loop
                                //                           //        SM_CMT_NESTEDLOOP_ODD,  // Area_Loop (invalid) nested in InnerLoop with an odd nesting depth (1,3,5) - healed to inner loop
                                //                           //        SM_CMT_BOTLOOP,         // NoArea_Loop (invalid) contains upperdomain UVPoints - AddMisingSeam heals to OuterLoop
                                //                           //        SM_CMT_TOPLOOP,         // NoArea_Loop (invalid) contains lowerdomain UVPoints - AddMisingSeam heals to OuterLoop
                                //                           //        SM_CMT_WIRELOOP,        // A set of connected Edgeuses without a closed portion.
     eClassifyCrv_StartType,    // out: oneof: SM_CD_LAMINA SM_CD_SEAM SM_CD_POLE SM_CD_POLE_NO_VERTEX SM_CD_NONE
     eClassifyCrv_StopType,     // out: oneof: SM_CD_LAMINA SM_CD_SEAM SM_CD_POLE SM_CD_POLE_NO_VERTEX SM_CD_NONE
     sOrderedNoAreaIndices,     // out: NoArea_loop indices in rLoops/rClassifyLoopIOs ordered from lowest domain to highest domain.
     m_pLooptreeRoot,           // out: LoopContainmentTree based solely on geometry relations, NULL on Input
     sCrvClassification,        // in : scratch CurveClassification memory to save time
     *this) ;                   // in : FaceProps after running FaceProps_BeforeEdgeSplit() to save time

 #ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      Dump(lOptLabel == SM_UNDEF_ULONG ? 200 : 200 + lOptLabel) ;
    }
#endif // SM_DEBUG_CODE

// watch out for problem faces that defy ClassifyLoops
 //  - for now, treat these as unknown problems that need further study so that the healer and ClassifyLoops
 //    can be extended to handle whatever problems this problem Face is causing.
 //  - return an error code - let the caller handle the problem
 if(eClassifiable != SM_HNF_NO_PROBS)
   {
     // remember this problem - return an error
     m_eNotYetFaceType = eClassifiable ;
     return(SM_ERR_NOTYET_HEAL_FACE) ; 

   } // end found a ProbFace to isolate from HealSequence check

  // for every Loop - Set SmLoopProps::m_eContainmentType  - from SmFace::ClassifyLoops results
  //                                 ::m_eLoopOrient       - from SmFace::ClassifyLoops results
  //                                 ::m_eDesiredLoopOrient
  //                                 ::m_bGoodOrient
  //     Accumulate counts for 1. m_lOuterLoopCnt           Vals to be set after this Loop
  //                           2. m_lBadOrient_LoopCnt              5. m_bBadOuterLoopOrder
  //                           3. m_lBadNoArea_LoopCnt              6. m_lBadMissingPoles
  //                           4. m_lBadNested_LoopCnt              7. m_bBadUnpairedNoArea_Loop
  //                           8. m_lBadClosed3d_LoopCnt            9. m_lBadClosedPtr_LoopCnt                      
  //                                                                   
  for(ii=0,jj=0;ii<sLoops.GetSize();ii++,jj++)
    {
      // WireLoops were skipped when building the sClassifyLoopIO array
      while(ii<sLoops.GetSize() && (   m_sLoopProps[ii].m_eContainmentType == SM_CMT_WIRELOOP
                                    || m_sLoopProps[ii].m_tLoopuseType     == SmVertexuse_TYPE ) )
        { ii++ ; }
      if(ii>=sLoops.GetSize()) { break ; }

      // match SmClassifyLoopIO and m_sLoopProps indices
      SmLoop           * pLoop           = sLoops[ii] ; 
      SmClassifyLoopIO * pClassifyLoopIO = &sClassifyLoopIOs[jj] ;
      SmLooptreeItem   * pParent         = NULL ;

      // Guard against NULL m_pLooptreeRoot - can happen when ClassifyLoops cannot process loops
      if (!m_pLooptreeRoot)
        {
          // verify that ClassifyLoops failed
          SE_MSG(sClassifyStatus != SM_SUCCESS, _T("SetProps__Stage3::Possible error, ClassifyLoops returned success but left m_pLooptreeRoot NULL - needs investigation - could be a bug or a missed heal case.")); 
          
          // GWC: another block of code that might have been added by cursor and without the test case hard to determine if its correct.
          // Cannot process loop properties without looptree root
          // This indicates a problem face that couldn't be classified
          m_eNotYetFaceType = SM_HNF_NO_LOOP_TREE_ROOT ;
          return(SM_ERR_NOTYET_HEAL_FACE) ;
        }
      SmLooptreeItem   * pTreeItem       = m_pLooptreeRoot->FindItem(pLoop, pParent) ;

#ifdef SM_DEBUG_CODE
      if(pLoop != pClassifyLoopIO->m_pLoop)
        {
          SM_ASSERT_MSG(pLoop == pClassifyLoopIO->m_pLoop, 
                        _T("SmFaceProps::SetPropos_AfterEdgeSplit - Loop Array data out of sync - needs bug fix")) ;
          if(bDebugMe)
            {
              m_sLoopProps[ii].Dump(ii) ;

              SmBrep    * pBrep    = m_pFace ? m_pFace->GetBrep() : NULL ;
              SmSurface * pSurface = m_pFace ? m_pFace->GetSurface() : NULL ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ;    if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ;    if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, .3,.3,.3) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ;    if(pSurface) pSurface->DrawParams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 1,0,0) ;    if(sLoops[ii]) sLoops[ii]->Draw() ; sm_GraphicsLoop() ; 
              sm_GraphicsLoop() ;
            }
        }
#endif // SM_DEBUG_CODE

      // update the TreeLoop objects - store LoopProps, remove old pClassifyLoopIO pts (owned by sClassifyLoopIOs)
      if(pTreeItem)
        {
          if(pTreeItem->m_pLoop == pLoop) { pTreeItem->m_pLoopProps = &m_sLoopProps[ii] ; 
                                            pTreeItem->m_pClassifyLoopIO = NULL ;
                                          }
          else if(pTreeItem->m_pPartnerLoop == pLoop) { pTreeItem->m_pPartnerLoopProps = &m_sLoopProps[ii] ; 
                                                        pTreeItem->m_pPartnerClassifyLoopIO = NULL ;
                                                      }
        } // end Found pTreeItem check

      // move ClassifyLoops results to m_sLoopProps (yuk!)
      m_sLoopProps[ii].m_eContainmentType = pClassifyLoopIO->m_eContainmentType ;
      m_sLoopProps[ii].m_eLoopOrient      = pClassifyLoopIO->m_eOrient ;

#ifdef SM_DEBUG_CODE
      if(m_sLoopProps[ii].m_bArea_Loop != pClassifyLoopIO->m_bArea_Loop)
        {
          // m_sLoopProps[ii].m_bArea_Loop from SmLoopProps::SetProps() ; (called earlier in this method)
          // pClassifyLoopIO->m_bArea_Loop from SmFace::ClassifyLoops() ; (called earlier in this method)
          SM_ASSERT_MSG(m_sLoopProps[ii].m_bArea_Loop == pClassifyLoopIO->m_bArea_Loop,
                        _T("SmFaceProps::SetPropos_AfterEdgeSplit - SmLoopProps::SetProps() and SmFace::ClassifyLoops computed different m_bAreaLoop values for a Loop.")) ; 
          if(bDebugMe)
            {
              this->Dump() ;
              m_sLoopProps[ii].Dump(ii) ;
              pClassifyLoopIO->Dump() ;
              ULONG di ;
              SmBrep    * pBrep    = m_pFace ? m_pFace->GetBrep() : NULL ;
              SmSurface * pSurface = m_pFace ? m_pFace->GetSurface() : NULL ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ;    if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ;    if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, .3,.3,.3) ; if(pSurface) pSurface->DrawSeams() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ;    if(pSurface) pSurface->DrawParams() ; sm_GraphicsLoop() ;
              for(di=0;di<sLoops.GetSize();di++) // draw loops, red=iith Loop, green= otherLoops
                { smgfx_SetLook(4,5, (di==ii)?1:0,(di!=ii)?1:0,0) ; if(sLoops[di]) sLoops[di]->Draw() ; sm_GraphicsLoop() ; } 
              sm_GraphicsLoop() ;
            }
        }
#endif // SM_DEBUG_CODE
      m_sLoopProps[ii].m_bArea_Loop = pClassifyLoopIO->m_bArea_Loop;

      // Set DesiredLoop Orientations
      m_sLoopProps[ii].m_eDesiredLoopOrient =   (   m_sLoopProps[ii].m_tLoopuseType == SmUnknown_TYPE) ? SM_OT_UNKNOWN
                                              : // SmEdgeuseType or SmVertexuseType
                                                (  m_sLoopProps[ii].m_eContainmentType == SM_CMT_OUTERLOOP ? SM_OT_SAME
                                                 : m_sLoopProps[ii].m_eContainmentType == SM_CMT_INNERLOOP ? SM_OT_OPPOSITE
                                                 : m_sLoopProps[ii].m_eContainmentType == SM_CMT_NESTEDLOOP_EVEN ? SM_OT_SAME
                                                 : m_sLoopProps[ii].m_eContainmentType == SM_CMT_NESTEDLOOP_ODD  ? SM_OT_OPPOSITE
                                                 : m_sLoopProps[ii].m_eContainmentType == SM_CMT_BOTLOOP  ? SM_OT_UPPERDOMAIN
                                                 : m_sLoopProps[ii].m_eContainmentType == SM_CMT_TOPLOOP  ? SM_OT_LOWERDOMAIN  
                                                 : SM_OT_UNKNOWN) ;

      // check assumption: SmVertexuse_TYPE loops will be oneof: SM_CMT_INNERLOOP, (VertexLoop in a Face case)
      //                                                         SM_CMT_TOPLOOP, (paired NoArea_Loop case)
      //                                                         SM_CMT_BOTLOOP, (paired NoArea_Loop case) 
      //     we don't expect to see SM_CMT_OUTERLOOP (vertexLoop in a degenerate Face case?)
      //                            SM_CMT_NESTEDLOOP_EVEN (VertexLoop nested in InnerLoop with an even nesting
      //                            SM_CMT_NESTEDLOOP_ODD  (VertexLoop nested in InnerLoop with an odd nesting)
      //                            SM_CMT_WIRELOOP (VertexLoops cannot be wire loops)
      SM_ASSERT_MSG(   (   m_sLoopProps[ii].m_tLoopuseType == SmUnknown_TYPE
                        || m_sLoopProps[ii].m_tLoopuseType == SmEdgeuse_TYPE)
                    || (   m_sLoopProps[ii].m_eContainmentType != SM_CMT_OUTERLOOP
                        && m_sLoopProps[ii].m_eContainmentType != SM_CMT_NESTEDLOOP_EVEN
                        && m_sLoopProps[ii].m_eContainmentType != SM_CMT_NESTEDLOOP_ODD 
                        && m_sLoopProps[ii].m_eContainmentType != SM_CMT_WIRELOOP       
                        && m_sLoopProps[ii].m_eContainmentType != SM_CMT_ERROR),
                    _T("SmFaceProps::SetPRops_Stage3: Found a VertexLoop with an unexpected ContainmentType - This may or may not be a bug - needs review")) ;

      // remember okay/problem orientations
      m_sLoopProps[ii].m_bGoodOrient = m_sLoopProps[ii].m_eLoopOrient == m_sLoopProps[ii].m_eDesiredLoopOrient ;
    } // end iter every Loop getting desired and actual LoopOrientations and ContainmentTypes

  // iter all the loops again (TreeItems are initialized) counting loop properties and problems
  for(ii=0,cnt=0;ii<sLoops.GetSize();ii++,cnt++)
    {
      // Skip WireLoops - WireLoops don't have an associated sClassifyLoopIO array entry
      while(ii<sLoops.GetSize() && m_sLoopProps[ii].m_eContainmentType == SM_CMT_WIRELOOP)
        { ii++ ; }
      if(ii >= sLoops.GetSize())
        {
          if(cnt == 0) { // All ProblemLoop counts are zero - no NonWire loops
                         m_lOuterLoopCnt         = 0 ;
                         m_lBadOrient_LoopCnt    = 0 ;
                         m_lBadNoArea_LoopCnt    = 0 ;
                         m_lBadNested_LoopCnt    = 0 ;
                         m_lBadClosed3d_LoopCnt  = 0 ;
                         m_lBadClosedPtr_LoopCnt = 0 ;
                       }
          break ; 
        } // end all loops are wire loops check

      // find the Loop's LooptreeItem
      const SmLoop   * pLoop         = m_sLoopProps[ii].m_cpLoop ;
      SmLooptreeItem * pParentItem   = NULL ; 
      SmLooptreeItem * pLoopTreeItem = m_pLooptreeRoot->FindItem(pLoop, pParentItem) ; 

      // 1. count the OuterLoops - NoArea_Loop pairs count as well
      if(cnt == 0) { m_lOuterLoopCnt = 0 ; }
      if(       m_sLoopProps[ii].m_eContainmentType == SM_CMT_OUTERLOOP   // OuterLoops count
         || (   pLoopTreeItem                                             // NoArea_Loop pairs count
             && pLoopTreeItem->m_pLoop == pLoop 
             && pLoopTreeItem->IsNoArea_Pair()) ) 
        {
          m_lOuterLoopCnt++ ; 
        }

      // 2. count the problem orientations
      if(cnt == 0) { m_lBadOrient_LoopCnt = 0 ; }
      if(m_sLoopProps[ii].m_bGoodOrient == FALSE)
        { 
          m_lBadOrient_LoopCnt++ ; 
        }

      // 3. count the NoArea_Loops
      if(cnt == 0) { m_lBadNoArea_LoopCnt = 0 ; }
      if(   m_sLoopProps[ii].m_eContainmentType == SM_CMT_BOTLOOP
         || m_sLoopProps[ii].m_eContainmentType == SM_CMT_TOPLOOP)               
        {                                                                        
          m_lBadNoArea_LoopCnt++ ;                                               
        }                                                                        

      // 4. count the problem nested Loops
      if(cnt == 0) { m_lBadNested_LoopCnt = 0 ; }
      if(   m_sLoopProps[ii].m_eContainmentType == SM_CMT_NESTEDLOOP_EVEN
         || m_sLoopProps[ii].m_eContainmentType == SM_CMT_NESTEDLOOP_ODD)
        { 
          m_lBadNested_LoopCnt++ ;                                                     
        }                                                                              
                                                                                       
      // 5. count the problem nested Loops                                             
      if(cnt == 0) { m_lBadClosed3d_LoopCnt  = 0 ; 
                     m_lBadClosedPtr_LoopCnt = 0 ;
                   }
      if(   m_sLoopProps[ii].m_bGoodClosed == FALSE)                                   
        { if(m_sLoopProps[ii].m_bClosed3d == FALSE) { m_lBadClosed3d_LoopCnt++ ; }     
          else                                      { m_lBadClosedPtr_LoopCnt++ ; }
        }
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          m_sLoopProps[ii].Dump(ii) ; 
          SmBrep * pBrep = m_pFace ? m_pFace->GetBrep() : NULL ; 

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          this->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                                 // sCrossedSeamColor   - def:[ 1, 0, 1]
                                                 // sMissingSeamColor   - def:[ 1, 0, 0]
                                                 // sMissingPoleColor   - def:[ 0, 0, 1]
                                                 // sBadOrientLoopColor - def:[.3,.3,.3]
                                                 // sNoArea_LoopColor   - def:[ 0, 0, 0]
          smgfx_SetLook(3,4, 1,0,0) ; if(sLoops[ii]) sLoops[ii]->Draw() ; sm_GraphicsLoop() ; 
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

    } // end iter every Loop getting desired and actual LoopOrientations and ContainmentTypes

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      Dump(lOptLabel == SM_UNDEF_ULONG ? 300 : 300 + lOptLabel) ;
    }
#endif // SM_DEBUG_CODE

  // 5. m_bBadOuterLoopOrder from m_lOuterLoopCnt and Loop orientations
  m_bBadOuterLoopOrder = (   m_lOuterLoopCnt == 1
                          && (   m_sLoopProps[0].m_eContainmentType != SM_CMT_OUTERLOOP
                              && m_sLoopProps[0].m_eContainmentType != SM_CMT_BOTLOOP)) ;

  // 6. m_lBadMissingPoles -
  m_lBadMissingPoles = SM_SS_NONE ;            
  // UMIN missing pole
  m_lBadMissingPoles |=  (   (eClassifyCrv_StartType == SM_CD_POLE_NO_VERTEX)
                          && (m_lNoVertexPoles & SM_SS_UMIN)            // Pole on the MinUSrfDomain
                          && (m_bClosedU)                               // Surf closed in U Dir
                          && (   (sOrderedNoAreaIndices.GetSize() > 0)  // pole contained in a NoArea_Loop
                              && (sClassifyLoopIOs[sOrderedNoAreaIndices[0]].m_eContainmentType == SM_CMT_TOPLOOP))
                         )
                        ? SM_SS_UMIN : SM_SS_NONE ; 
  // VMIN missing pole
  m_lBadMissingPoles |=  (   (eClassifyCrv_StopType == SM_CD_POLE_NO_VERTEX)
                          && (m_lNoVertexPoles & SM_SS_VMIN)            // Pole on the MinVSrfDomain
                          && (m_bClosedV)                               // Surf closed in V Dir
                          && (   (sOrderedNoAreaIndices.GetSize() > 0)  // pole contained in a NoArea_Loop
                              && (sClassifyLoopIOs[sOrderedNoAreaIndices[0]].m_eContainmentType == SM_CMT_BOTLOOP))
                         )
                        ? SM_SS_VMIN : SM_SS_NONE ; 
  // UMAX missing pole
  m_lBadMissingPoles |=  (   (eClassifyCrv_StopType == SM_CD_POLE_NO_VERTEX)
                          && (m_lNoVertexPoles & SM_SS_UMAX)            // Pole on the MaxUSrfDomain
                          && (m_bClosedU)                               // Surf closed in U Dir
                          && (   (sOrderedNoAreaIndices.GetSize() > 0)  // pole contained in a NoArea_Loop
                              && (sClassifyLoopIOs[sOrderedNoAreaIndices[0]].m_eContainmentType == SM_CMT_TOPLOOP))
                         )
                        ? SM_SS_UMAX : SM_SS_NONE ; 
  // VMAX missing pole
  m_lBadMissingPoles |=  (   (eClassifyCrv_StopType == SM_CD_POLE_NO_VERTEX)
                          && (m_lNoVertexPoles & SM_SS_VMAX)            // Pole on the MaxVSrfDomain
                          && (m_bClosedV)                               // Surf closed in V Dir
                          && (   (sOrderedNoAreaIndices.GetSize() > 0)  // pole contained in a NoArea_Loop
                              && (sClassifyLoopIOs[sOrderedNoAreaIndices[0]].m_eContainmentType == SM_CMT_BOTLOOP))
                         )
                        ? SM_SS_VMAX : SM_SS_NONE ; 
                                                                    
  // 7. m_bBadUnpairedNoArea_Loop from m_lBadMissingPoles bit array
  m_bBadUnpairedNoArea_Loop = 
    smos_IsOdd(  (   (m_lBadMissingPoles & (SM_SS_UMIN | SM_SS_VMIN))                  // add 1 for    Starting Pole is part of Face Boundary
                     && (eClassifyCrv_StartType == SM_CD_POLE_NO_VERTEX) ? 1 : 0)      //          and surface has no starting pole vertex
                  + sOrderedNoAreaIndices.GetSize()                                    // plus all the NoArea_Loops
                  + (   (m_lBadMissingPoles & (SM_SS_UMAX | SM_SS_VMAX))               // add 1 for    Ending Pole is part of Face Boundary
                     && (eClassifyCrv_StopType  == SM_CD_POLE_NO_VERTEX) ? 1 : 0) ) ;  //              and surface has no ending pole vertex

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE ; 
  if(bDebugMe2)
    {
      this->Dump(lOptLabel == SM_UNDEF_ULONG ? 400 : 400 + lOptLabel) ; 
      SmBrep * pBrep = m_pFace ? m_pFace->GetBrep() : NULL ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      this->Draw(TRUE) ; sm_GraphicsLoop() ; // sSheetColor         - def:[ 0, 1, 1]
                                             // sCrossedSeamColor   - def:[ 1, 0, 1]
                                             // sMissingSeamColor   - def:[ 1, 0, 0]
                                             // sMissingPoleColor   - def:[ 0, 0, 1]
                                             // sBadOrientLoopColor - def:[.3,.3,.3]
                                             // sNoArea_LoopColor   - def:[ 0, 0, 0]
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // set m_bBadLoopProblems = TRUE: has at least one Loop problem
  m_bBadLoopProblems = HasProblems(4) ; // 4 has Post SetProps_Stage3() problems 

  // all done
  // increment PropState here - prevent infinite loops
  m_ePropStage = SM_PROPSTAGE_3 ; // SM_PROPSTAGE_3 = after SmFaceProps::SetProps_Stage3() call
  return(SM_SUCCESS) ;

} // end SmFaceProps::SetProps_Stage3

/*******************************************************************//**
PURPOSE: Return TRUE if Point3d is within Face->ZoneTol3d of a surface pole
                                        
NOTES: Dependent member values
         m_sZoneTol3    set in SetProps(SM_PROPSTAGE_GAPS)
         m_lPoles       set in SetProps(SM_PROPSTAGE_2)
         m_sPolePoints  set in SetProps(SM_PROPSTAGE_2)
***********************************************************************/
SmBoolean SmFaceProps::IsPoint3dOnPole
 (SmPoint3d & rPoint3d)   // in : TgtPoint to test
 const
{
  // init output
  SmBoolean bRtn = FALSE ;

  // Points within ZoneTol3d of a pole location are on the pole
  if(m_lPoles)
    {
      double dZoneTol3dSq = m_sZoneTol3d * m_sZoneTol3d ;
      bRtn |= (m_lPoles & SM_SS_UMIN) && ((m_sPolePoints[0]-rPoint3d).LengthSquared() < dZoneTol3dSq) ;  
      bRtn |= (m_lPoles & SM_SS_VMIN) && ((m_sPolePoints[1]-rPoint3d).LengthSquared() < dZoneTol3dSq) ;  
      bRtn |= (m_lPoles & SM_SS_UMAX) && ((m_sPolePoints[2]-rPoint3d).LengthSquared() < dZoneTol3dSq) ;  
      bRtn |= (m_lPoles & SM_SS_VMAX) && ((m_sPolePoints[3]-rPoint3d).LengthSquared() < dZoneTol3dSq) ;  
    } // end Surface has poles check

  // all done
  return(bRtn) ;

} // end SmFaceProps::IsPoint3dOnPole, m_lPoles, m_sPolePoints)                                                            

/*******************************************************************//**
PURPOSE: Return TRUE if obj has any problems
                                        
NOTES: Member values are set with a calls to 
         SetProps_Stage2()
         SetProps_Stage3()
***********************************************************************/
SmBoolean SmFaceProps::HasProblems
 (ULONG lBitArray) const   // in : OrOf : OrOf 1 = Post SetProps_Gaps() problems       
                           //           :      2 = Post SetProps_Stage2() problems     
                           //           :      4 = Post SetProps_Stage3() problems
{ 
  // return val
  SmBoolean bProb = FALSE ;
  
  // NotYet supported Face problem
  bProb |= m_eNotYetFaceType != SM_HNF_NO_PROBS ;

  // SetProps(SM_PROPSTAGE_GAPS) problem list
  if(m_ePropStage >= SM_PROPSTAGE_GAPS && (lBitArray & 1))
    { 
      bProb |= (m_bBadSmallZoneTol3d == TRUE) ;
      bProb |= (m_bBadLargeZoneTol3d == TRUE) ;

    }  // end SetProps(SM_PROPSTAGE_GAPS) problem list

  // SetProps(SM_PROPSTAGE_2) problem list
  if(m_ePropStage >= SM_PROPSTAGE_2 && (lBitArray & 2))
    { bProb |= (m_eSheet == SM_MP_PROB) ;
      bProb |= (m_ePoles == SM_MP_PROB) ;
      bProb |= (   m_lBadFlatCorners & SM_FC_UMIN_VMIN
                || m_lBadFlatCorners & SM_FC_UMIN_VMAX
                || m_lBadFlatCorners & SM_FC_UMAX_VMIN
                || m_lBadFlatCorners & SM_FC_UMAX_VMAX) ;
      bProb |= (m_bBadCrossedSeam == TRUE) ;
      bProb |= (m_bBadMissingSeam == TRUE) ;
      // NOT Yet - needed later when Fix_BadGaps() gets built
      //  bProb |= (m_bBadNearMissSeam == TRUE) ; 

    } // end // SetProps(SM_PROPSTAGE_2) problem list

  // SetProps(SM_PROPSTAGE_3) problem list
  if(m_ePropStage >= SM_PROPSTAGE_3 && (lBitArray & 4))
    { 
      bProb |= (m_lOuterLoopCnt           != 1) ;
      
      // don't check m_bBadLoopProblems - it's just the union of these other problems
      bProb |= (m_bBadOuterLoopOrder      == TRUE) ;
      bProb |= (m_lBadOrient_LoopCnt       > 0) ;
      bProb |= (m_lBadNoArea_LoopCnt       > 0) ;
      bProb |= (m_lBadNested_LoopCnt       > 0) ;
      bProb |= (m_bBadUnpairedNoArea_Loop == TRUE) ;
      bProb |= (m_lBadClosed3d_LoopCnt     > 0) ;
      bProb |= (m_lBadClosedPtr_LoopCnt    > 0) ;
      bProb |= (m_lBadMissingPoles         > 0) ;

    } // end SetProps(SM_PROPSTAGE_3) problem list

  // all done
  return(bProb) ;  
  
} // end SmFaceProps::HasProblems

/****************************************************************
PURPOSE: add graphics for problem Faces

NOTES: i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
       NULL to ignore. default:[NULL]
****************************************************************/
SmDisplayList * SmFaceProps::Draw
 ( SmBoolean bDrawAllCases,             // in : TRUE = output graphics for all Faces (with or without probs)
                                        //      default:[FALSE] = output graphics only for Faces with probs 
   SmVector3d sHighlightColor,          // in : def:[ 1, 0, 0]: If FaceHasProblems Draw FaceUV for highlight
   SmVector3d sBadZoneTol3dColor,       // in : def:[.6,.3,.8]: Faces with bad (too large or small) ZoneTol3d Highlight Color
   SmVector3d sBadSheetColor,           // in : def:[ 0, 1, 1]: Faces mislabled as Sheets splitting their Shell Highlight Color    
   SmVector3d sBadCrossedSeamColor,     // in : def:[ 1, 0, 1]: Loops Crossing Seams Highlight Color
   SmVector3d sBadMissingSeamColor,     // in : def:[ 1, 0, 0]: MissingSeams Highlight Color
   SmVector3d sBadNearMissSeamColor,    // in : def:[ 0, 1, 1]: NearMissSeams Hightlight Color
   SmVector3d sBadMissingPoleColor,     // in : def:[ 0, 0, 1]: MissingPoles Highlight Color
   SmVector3d sBadFlatCornerColor,      // in : def:[.2,.2, 1]: FlatCorner (SurfDU parallel to SurfDV) Highlight Color
   SmVector3d sBadOrientLoopColor,      // in : def:[.3,.3,.3]: Badly oriented Loops Highlight Color   
   SmVector3d sBadDegen_LoopColor,      // in : def:[1.,.5, 0]: Loop has at least one degenerate dimension
   SmVector3d sBadNoArea_LoopColor,     // in : def:[ 0, 0, 0]: NoArea_Loops Highlight Color
   SmVector3d sBadOrient_ClassCrvColor, // in : def:[.8,.2,.7]: Orient CurveClassifcation Highlight Color
   SmVector3d sBadClosed_LoopColor,     // in : def:[.4,.8,.2]: ClosedLoop3d != ClosedLoopPtrs (assumed intent is closed)  
   SmGfxArraySet * pOptGfxSet           // in : When given, output GfxVertexArrays not GL calls. 
 ) const
{
 SM_REF1(sBadNearMissSeamColor);
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE

  // locals 
  SmBoolean bHasProblems = HasProblems() ; 

  // no work - face has no problems and only drawing faces with problems or during SetProp calls
  if(   bDrawAllCases == FALSE
     && bHasProblems  == FALSE 
     && m_ePropStage != SM_PROPSTAGE_GAPSa
     && m_ePropStage != SM_PROPSTAGE_2a
     && m_ePropStage != SM_PROPSTAGE_3a)
    {
      // all done
      return(NULL) ; 
    }

  // locals: global display parameters
  ULONG ii, lCnt ; 
  // const SmDisplayParameters & rDisp = smgfx_RefGlobalDisplayParameters() ;
  SmPoint3d sPoint ;
  SmPoint2d sUVPoint ; 

  // start new displayList (unless one is already open)
  SmVector3d sFacePropsColor(0,0,1) ; 
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(sFacePropsColor, NULL, NULL, FALSE, pOptGfxSet);

  // Face has a NotYet supported Face problem - Add Loop graphics
  if(m_eNotYetFaceType != SM_HNF_NO_PROBS)
    {
      // locals
      SmTArray<SmLoop*> sdLoops ;
      if(m_pFace) { m_pFace->GetLoops(sdLoops) ; }

      // add Loopgraphics (and seam)
      smgfx_SetLook(3,4, .3,.3,.3) ; if(m_cpSurface) m_cpSurface->DrawSeams() ;  
      smgfx_SetLook(2,3, 0,1,1)    ; for(ULONG di=0;di<sdLoops.GetSize();di++)
                                       { if(sdLoops[di]) sdLoops[di]->Draw() ; }

    } // end Face has a NotYet supported Face problem check

  // Highlight faces - CrossHatching does not work on BadOuterLoopOrder problem faces
  smgfx_SetLook(2,7, sHighlightColor) ;  if(m_pFace) m_pFace->Draw(  (m_bBadOuterLoopOrder == TRUE)
                                                                     ? SM_DM_WIREFRAME : SM_DM_CROSSHATCH,
                                                                     5,5, pOptGfxSet) ;

  // Bad ZoneTol3d vals
  if(   m_bBadSmallZoneTol3d
     || m_bBadLargeZoneTol3d) 
    { smgfx_SetLook(3,9, sBadZoneTol3dColor) ; if(m_pFace) m_pFace->Draw(SM_DM_WIREFRAME,5,5, pOptGfxSet) ; }

  // output Problem Sheet Graphics - only Draw those sheets that are seperating two unconnected shells  
  if(m_eSheet == SM_MP_PROB) 
    { smgfx_SetLook(3,9, sBadSheetColor) ; m_pFace->Draw(SM_DM_CROSSHATCH,5,5, pOptGfxSet) ; }

  // Problem Poles
  if(m_ePoles == SM_MP_PROB)
    {
      if(m_lNoVertexPoles != SM_SS_NONE)
        {
          // NoVertex Poles - okay when pole is not 'in' Face or 'on' FaceBoundary
          smgfx_SetLook(4,5, 0,0,0) ; 
          if(m_lNoVertexPoles & SM_SS_UMIN) { if(m_sPolePoints[0].IsInitialized()) m_sPolePoints[0].Draw() ; }
          if(m_lNoVertexPoles & SM_SS_VMIN) { if(m_sPolePoints[1].IsInitialized()) m_sPolePoints[1].Draw() ; }
          if(m_lNoVertexPoles & SM_SS_UMAX) { if(m_sPolePoints[2].IsInitialized()) m_sPolePoints[2].Draw() ; }
          if(m_lNoVertexPoles & SM_SS_VMAX) { if(m_sPolePoints[3].IsInitialized()) m_sPolePoints[3].Draw() ; }
        } // end has missing poles check

      // Bad Missing Poles - Drawn bigger than NoVertexPoles
      if(m_lBadMissingPoles != SM_SS_NONE)
        {
          smgfx_SetLook(9,10, sBadMissingPoleColor) ; 
          if(m_lBadMissingPoles & SM_SS_UMIN) { if(m_sPolePoints[0].IsInitialized()) m_sPolePoints[0].Draw() ; }
          if(m_lBadMissingPoles & SM_SS_VMIN) { if(m_sPolePoints[1].IsInitialized()) m_sPolePoints[1].Draw() ; }
          if(m_lBadMissingPoles & SM_SS_UMAX) { if(m_sPolePoints[2].IsInitialized()) m_sPolePoints[2].Draw() ; }
          if(m_lBadMissingPoles & SM_SS_VMAX) { if(m_sPolePoints[3].IsInitialized()) m_sPolePoints[3].Draw() ; }
        } // end has missing poles check
    } // end problem poles check

  // Bad FlatCorners
  if(m_lBadFlatCorners != SM_FC_NONE)
    {
      smgfx_SetLook(11,12, sBadFlatCornerColor) ; 
      if(m_lBadFlatCorners & SM_FC_UMIN_VMIN) { if(m_sFlatCornerPoints[0].IsInitialized()) m_sFlatCornerPoints[0].Draw() ; }
      if(m_lBadFlatCorners & SM_FC_UMIN_VMAX) { if(m_sFlatCornerPoints[1].IsInitialized()) m_sFlatCornerPoints[1].Draw() ; }
      if(m_lBadFlatCorners & SM_FC_UMAX_VMIN) { if(m_sFlatCornerPoints[2].IsInitialized()) m_sFlatCornerPoints[2].Draw() ; }
      if(m_lBadFlatCorners & SM_FC_UMAX_VMAX) { if(m_sFlatCornerPoints[3].IsInitialized()) m_sFlatCornerPoints[3].Draw() ; }

    } // end has FlatCorners check

  // draw Surf UV and params on okay surfaces
  if(bHasProblems == TRUE)
    {
      smgfx_SetLook(1,2, 0,1,1) ; if(m_cpSurface) m_cpSurface->DrawUV() ;
      smgfx_SetLook(14,15, 0,1,0) ; if(m_cpSurface) m_cpSurface->DrawParams() ;
      smgfx_SetLook(3,4, .3,.3,.3) ; if(m_cpSurface) m_cpSurface->DrawSeams() ;
    }

  // UDir Seam Crossings
  smgfx_SetLook(3,6, sBadCrossedSeamColor) ;
  if(m_eBadCrossedSeam == SM_SP_U || m_eBadCrossedSeam == SM_SP_BOTH) 
    { 
      for(ii=0,lCnt=0;ii<m_pCrvClassU->GetSize();ii++)
        { SmCurveInterval & rCrvIvl = m_pCrvClassU->GetAt(ii) ;
          if(rCrvIvl.m_vStart.GetPointClass() == SM_PC_EDGE)
            { m_pCrvClassU->GetCurve()->EvaluatePoint(rCrvIvl.GetMin(), sPoint) ;
             sPoint.Draw() ; lCnt++ ;  
            }
          if(rCrvIvl.m_vEnd.GetPointClass() == SM_PC_EDGE)
            { m_pCrvClassU->GetCurve()->EvaluatePoint(rCrvIvl.GetMax(), sPoint) ;
             sPoint.Draw() ; lCnt++ ;  
            }
        } // end iter CrvClassU intervals looking for SeamCrossings

      if(lCnt > 0) 
        { // smgfx_SetLook(4,5, .2,.2,.2) ; m_pCrvClassU->GetCurve()->Draw() ; 
          smgfx_SetLook(1,2, 0,1,1) ;    if(m_cpSurface) m_cpSurface->DrawUV() ; 
          smgfx_SetLook(14,15, 0,1,0) ;  if(m_cpSurface) m_cpSurface->DrawParams() ; 
          smgfx_SetLook(2,3, .3,.3,.3) ; if(m_cpSurface) m_cpSurface->DrawSeams() ; 
        }
    } // end Has UDir SeamCrossings check

  // VDir Seam Crossings
  smgfx_SetLook(3,6, sBadCrossedSeamColor) ;
  if(m_eBadCrossedSeam == SM_SP_V || m_eBadCrossedSeam == SM_SP_BOTH) 
    { 
      for(ii=0,lCnt=0;ii<m_pCrvClassV->GetSize();ii++)
        { SmCurveInterval & rCrvIvl = m_pCrvClassV->GetAt(ii) ;
          if(rCrvIvl.m_vStart.GetPointClass() == SM_PC_EDGE)
            { m_pCrvClassV->GetCurve()->EvaluatePoint(rCrvIvl.GetMin(), sPoint) ;
              sPoint.Draw() ; lCnt++ ;
            }
          if(rCrvIvl.m_vEnd.GetPointClass() == SM_PC_EDGE)
            { m_pCrvClassV->GetCurve()->EvaluatePoint(rCrvIvl.GetMax(), sPoint) ;
              sPoint.Draw() ; lCnt++ ;
            }
        } // end iter CrvClassV intervals looking for SeamCrossings
    
      if(lCnt > 0) 
        { // smgfx_SetLook(4,5, .2,.2,.2) ; m_pCrvClassV->GetCurve()->Draw() ; 
          smgfx_SetLook(1,2, 0,1,1) ;    if(m_cpSurface) m_cpSurface->DrawUV() ; 
          smgfx_SetLook(14,15, 0,1,0) ;  if(m_cpSurface) m_cpSurface->DrawParams() ; 
          smgfx_SetLook(2,3, .3,.3,.3) ; if(m_cpSurface) m_cpSurface->DrawSeams() ; 
        }
    } // end Has VDir SeamCrossings check

  // UDir MissingSeam Segments
  smgfx_SetLook(3,5, sBadMissingSeamColor) ;
  if(m_eBadMissingSeam == SM_SP_U || m_eBadMissingSeam == SM_SP_BOTH) 
    { 
      for(ii=0,lCnt=0;ii<m_pCrvClassU->GetSize();ii++)
        { SmCurveInterval & rCrvIvl = m_pCrvClassU->GetAt(ii) ;
          if(rCrvIvl.m_vMid.GetPointClass() == SM_PC_FACE)
            { m_pCrvClassU->GetCurve()->Draw(&rCrvIvl.GetInterval()) ; lCnt++ ; }
        } // end iter CrvClassU intervals looking for MissingSeams

      if(lCnt > 0) 
        { // smgfx_SetLook(4,5, .2,.2,.2) ; m_pCrvClassU->GetCurve()->Draw() ; 
          smgfx_SetLook(1,2, 0,1,1) ;    if(m_cpSurface) m_cpSurface->DrawUV() ; 
          smgfx_SetLook(14,15, 0,1,0) ;  if(m_cpSurface) m_cpSurface->DrawParams() ; 
          smgfx_SetLook(2,3, .3,.3,.3) ; if(m_cpSurface) m_cpSurface->DrawSeams() ; 
        }
    } // end Has UDir MissingSeams check

  // VDir MissingSeam Segments
  smgfx_SetLook(3,5, sBadMissingSeamColor) ;
  if(m_eBadMissingSeam == SM_SP_V || m_eBadMissingSeam == SM_SP_BOTH) 
    { 
      for(ii=0,lCnt=0;ii<m_pCrvClassV->GetSize();ii++)
        { SmCurveInterval & rCrvIvl = m_pCrvClassV->GetAt(ii) ;
          if(rCrvIvl.m_vMid.GetPointClass() == SM_PC_FACE)
            { m_pCrvClassV->GetCurve()->Draw(&rCrvIvl.GetInterval()) ; lCnt++ ; }
        } // end iter CrvClassV intervals looking for MissingSeams

      if(lCnt > 0) 
        { // smgfx_SetLook(4,5, .2,.2,.2) ; m_pCrvClassV->GetCurve()->Draw() ; 
          smgfx_SetLook(1,2, 0,1,1) ;    if(m_cpSurface) m_cpSurface->DrawUV() ; 
          smgfx_SetLook(14,15, 0,1,0) ;  if(m_cpSurface) m_cpSurface->DrawParams() ; 
          smgfx_SetLook(2,3, .3,.3,.3) ; if(m_cpSurface) m_cpSurface->DrawSeams() ; 
        }
    } // end Has VDir MissingSeams check

   // UDir NearMissSeam Segments
   // Not Yet: Add when future Fix_BadGaps() is built and fixing NearMiss TopoConnections
   //  smgfx_SetLook(3,5, sBadNearMissSeamColor) ;
   //  if(m_eBadNearMissSeam == SM_SP_U || m_eBadNearMissSeam == SM_SP_BOTH) 
   //    { 
   //      for(ii=0,lCnt=0;ii<m_pCrvClassU->GetSize();ii++)
   //        { SmCurveInterval & rCrvIvl = m_pCrvClassU->GetAt(ii) ;
   //          if(rCrvIvl.m_vMid.GetPointClass() == SM_PC_EDGE)
   //            { m_pCrvClassU->GetCurve()->Draw(&rCrvIvl.GetInterval()) ; lCnt++ ; }
   //          if(rCrvIvl.m_vMid.GetPointClass() == SM_PC_FACE)
   //            { lCnt++ ; }
   //        } // end iter CrvClassU intervals looking for NearMissSeams
   //  
   //      if(lCnt > 0) 
   //        { // smgfx_SetLook(4,5, .2,.2,.2) ; m_pCrvClassU->GetCurve()->Draw() ; 
   //          smgfx_SetLook(1,2, 0,1,1) ;    if(m_cpSurface) m_cpSurface->DrawUV() ; 
   //          smgfx_SetLook(14,15, 0,1,0) ;  if(m_cpSurface) m_cpSurface->DrawParams() ; 
   //          smgfx_SetLook(2,3, .3,.3,.3) ; if(m_cpSurface) m_cpSurface->DrawSeams() ; 
   //        }
   //    } // end Has UDir NearMissSeams check
   // end Not Yet: section removal - to be added back in after Fix_BadGaps() is built

   // VDir NearMissSeam Segments
   // Not Yet: Add when future Fix_BadGaps() is built and fixing NearMiss TopoConnections
   //  smgfx_SetLook(3,5, sBadNearMissSeamColor) ;
   //  if(m_eBadNearMissSeam == SM_SP_V || m_eBadNearMissSeam == SM_SP_BOTH) 
   //    { 
   //      for(ii=0,lCnt=0;ii<m_pCrvClassV->GetSize();ii++)
   //        { SmCurveInterval & rCrvIvl = m_pCrvClassV->GetAt(ii) ;
   //          if(rCrvIvl.m_vMid.GetPointClass() == SM_PC_EDGE)
   //            { m_pCrvClassV->GetCurve()->Draw(&rCrvIvl.GetInterval()) ; lCnt++ ; }
   //          if(rCrvIvl.m_vMid.GetPointClass() == SM_PC_FACE)
   //            { lCnt++ ; }
   //        } // end iter CrvClassV intervals looking for NearMissSeams
   //  
   //      if(lCnt > 0) 
   //        { // smgfx_SetLook(4,5, .2,.2,.2) ; m_pCrvClassV->GetCurve()->Draw() ; 
   //          smgfx_SetLook(1,2, 0,1,1) ;    if(m_cpSurface) m_cpSurface->DrawUV() ; 
   //          smgfx_SetLook(14,15, 0,1,0) ;  if(m_cpSurface) m_cpSurface->DrawParams() ; 
   //          smgfx_SetLook(2,3, .3,.3,.3) ; if(m_cpSurface) m_cpSurface->DrawSeams() ; 
   //        }
   //    } // end Has VDir NearMissSeams check
   // end Not Yet: section removal - to be added back in after Fix_BadGaps() is built

  // For Faces with problem Loops (only available after SmFaceProps::SetProps_Stage3() has run)
  if(   (m_lOuterLoopCnt           != 1)
     || (m_bBadOuterLoopOrder      == TRUE)
     || (m_lBadOrient_LoopCnt      != 0)
     || (m_lBadNoArea_LoopCnt      != 0)
     || (m_lBadNested_LoopCnt      != 0)
     || (m_bBadUnpairedNoArea_Loop == TRUE)
     || (m_lBadClosed3d_LoopCnt    != 0) 
     || (m_lBadClosedPtr_LoopCnt   != 0)
    )
    {
      // for every loop prop
      for(ii=0;ii<m_sLoopProps.GetSize();ii++)
        {
          // Draw Bad Orient, Bad NoArea, Bad SeamCrossing, and Bad Nested Loops
          if(   m_sLoopProps[ii].m_bGoodOrient == FALSE
             || m_sLoopProps[ii].m_bArea_Loop  == FALSE
             || m_sLoopProps[ii].m_bDegen_Loop == TRUE
             || m_sLoopProps[ii].m_lSeamCrossingCntU > 0
             || m_sLoopProps[ii].m_lSeamCrossingCntV > 0
             || (   m_sLoopProps[ii].m_eContainmentType == SM_CMT_NESTEDLOOP_EVEN 
                 || m_sLoopProps[ii].m_eContainmentType == SM_CMT_NESTEDLOOP_ODD)
             || m_sLoopProps[ii].m_bGoodClosed == FALSE)
            {  
              m_sLoopProps[ii].Draw(sBadOrientLoopColor,      
                                    sBadDegen_LoopColor, 
                                    sBadNoArea_LoopColor, 
                                    sBadOrient_ClassCrvColor, 
                                    sBadCrossedSeamColor, 
                                    sBadClosed_LoopColor,     
                                    pOptGfxSet) ;
            } // End LoopProp has problems check
        } // end iter every m_sLoopProps
    } // end Face has Bad Loops check

  // end new displayList
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF12(
      bDrawAllCases,
      sHighlightColor,
      sBadZoneTol3dColor,
      sBadSheetColor,
      sBadCrossedSeamColor,
      sBadMissingSeamColor,
      sBadMissingPoleColor,
      sBadFlatCornerColor,
      sBadOrientLoopColor,
      sBadDegen_LoopColor,
      sBadNoArea_LoopColor,
      sBadOrient_ClassCrvColor
  );
  SM_REF2(sBadClosed_LoopColor, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmFaceProps::Draw

// obsolete
//  /*******************************************************************//**
//  PURPOSE: Helper PrettyPrint function for SmFaceProps::Dump()
//  
//  NOTES:
//  ***********************************************************************/
//  void smos_DumpBeforeTolFixProblems
//   (const SmFaceProps * pFaceProps, 
//    TCHAR               sBuff[SM_TBLOCK_SIZE],
//    SmBoolean         & rbDumpSeamU,
//    SmBoolean         & rbDumpSeamV,
//    SmBoolean           bLabel)
//  {
//    // locals
//    TCHAR sTab[SM_TBLOCK_SIZE] ; 
//  
//    if(pFaceProps->m_ePropStage >= SM_PROPSTAGE_GAPS) 
//      {
//        if(bLabel) { smos_WriteBuffer( _T("\n  BeforeTolFix ")) ;
//                     smos_sprintf(sTab,_T("                           ")) ;
//                   }
//        else       { smos_WriteBuffer( _T("\n    ")) ;
//                     smos_sprintf(sTab,_T("             ")) ; 
//                   }
//  
//        if(pFaceProps->HasProblems() == FALSE)
//          {
//            smos_WriteBuffer(_T(" Problems : [None]")) ;
//            rbDumpSeamU = FALSE ;
//            rbDumpSeamV = FALSE ; 
//          }
//        else // Has Problems branch
//          {
//            SmZoneTol3d sDefZoneTol3d = SmTol::GetZoneTol3d(pFaceProps->m_cpContext) ; // Def Tol is Tol with no gaps
//  
//            // m_bBadSmallZoneTol3d
//            smos_sprintf(sBuff, _T("Problems : BadZoneTol3d (too small) :[%s]"),
//                         (pFaceProps->m_bBadSmallZoneTol3d == TRUE)  ? _T("TRUE")
//                       : (pFaceProps->m_bBadSmallZoneTol3d == FALSE) ? _T("FALSE") 
//                       :                                               _T("UNSURE")) ;
//            smos_WriteBuffer(sBuff) ; 
//            if(pFaceProps->m_bBadSmallZoneTol3d == TRUE)
//              { smos_sprintf(sBuff, _T(" Bad : Increase OrigZoneTol3d:[%lf] to DefZoneTol3d:[%lf]"),
//                           pFaceProps->m_sOrigZoneTol3d.val,
//                           sDefZoneTol3d.val) ; 
//                smos_WriteBuffer(sBuff) ;
//              }
//            else if(pFaceProps->m_bBadSmallZoneTol3d == FALSE && pFaceProps->m_sOrigZoneTol3d < pFaceProps->m_sZoneTol3d)
//              {
//                smos_sprintf(sBuff,_T(" Okay: OrigZoneTol3d:[%lf] increased to ZoneTol3d:[%lf]"),
//                                 pFaceProps->m_sOrigZoneTol3d.val,
//                                 pFaceProps->m_sZoneTol3d.val) ; 
//                smos_WriteBuffer(sBuff) ;
//              }
//            else
//              { smos_WriteBuffer(_T(" Okay")) ; } 
//  
//            // m_bBadLargeZoneTol3d
//            smos_sprintf(sBuff, _T("\n%s: BadZoneTol3d (too large) :[%s]"),
//                       sTab,
//                         (pFaceProps->m_bBadLargeZoneTol3d == TRUE)  ? _T("TRUE")
//                       : (pFaceProps->m_bBadLargeZoneTol3d == FALSE) ? _T("FALSE") : _T("UNSURE")) ;
//            smos_WriteBuffer(sBuff) ; 
//            if(pFaceProps->m_bBadLargeZoneTol3d == TRUE)
//              { smos_sprintf(sBuff, _T(" Bad : Decrease OrigZoneTol3d:[%lf] to DefZoneTol3d:[%lf]"),
//                           pFaceProps->m_sOrigZoneTol3d.val,
//                           sDefZoneTol3d.val) ; 
//                smos_WriteBuffer(sBuff) ;
//              }
//            else if(pFaceProps->m_bBadSmallZoneTol3d == FALSE && pFaceProps->m_sOrigZoneTol3d > pFaceProps->m_sZoneTol3d)
//              {
//                smos_sprintf(sBuff,_T(" Okay: OrigZoneTol3d:[%lf] decreased to ZoneTol3d:[%lf]"),
//                                 pFaceProps->m_sOrigZoneTol3d.val,
//                                 pFaceProps->m_sZoneTol3d.val) ; 
//                smos_WriteBuffer(sBuff) ;
//              }
//            else
//              { smos_WriteBuffer(_T(" Okay")) ; } 
//          } // end has problems branch
//  
//      } // end pFaceProps->m_ePropStage >= SM_PROPSTAGE_GAPS check
//  } // end smos_DumpBeforeTolFixProblems
// end obsolete

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmFaceProps::Dump() const
{
  // pass the call along
  Dump(SM_UNDEF_ULONG, TRUE) ;

}// end SmFaceProps::Dump()

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
void SmFaceProps::Dump
  (ULONG     lLabel,  // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]
   SmBoolean bLong)   // in : TRUE = Dump SeamU and SeamV CrvClassifications and Per Loop LoopProp data
                      //      FALSE= Don't, default:[TRUE]
 const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\nBegin SmFaceProps Dump ") ) ; }
  else                         { smos_sprintf(sBuff, _T("\nBegin SmFaceProps Dump [%3lu]"), lLabel) ; }
  smos_WriteBuffer(sBuff);

  // base class
  SmObjProps::Dump() ;

  // Begin Header = PropsPtr, FacePtr, SurfacePtr, and ZoneTol3d val.
  smos_sprintf(sBuff,        _T("\n SmFaceProps:[0x%p] For Face:[0x%p] Indx:[%4lu] Surface:[0x%p] Context:[0x%p]"),
             this,
             m_pFace,
             m_lFaceIndx,
             m_cpSurface,
             m_cpContext) ; 
  smos_sprintf(sBuffForFile, _T("\n SmFaceProps:[%s] For Face:[%s] Indx[%4lu] Surface:[%s] Context:[%s]"),
             _T("notNULL"),
             m_pFace     ? _T("notNULL") : _T("NULL"),
             m_lFaceIndx,
             m_cpSurface ? _T("notNULL") : _T("NULL"),
             m_cpContext ? _T("notNULL") : _T("NULL")) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // End Header = ZoneTol3d, OrigZoneTol3d val.
  if(m_ePropStage >= SM_PROPSTAGE_GAPS) 
    {
      smos_sprintf(sBuff,        _T(" ZoneTol3d:[%5.7lf] OrigZoneTol3d:[%5.7lf]"),
                 m_sZoneTol3d.val, 
                 m_sOrigZoneTol3d.val) ; 
      smos_WriteBuffer(sBuff) ;
    }

  // DegenFace - Set in SmHealData::Get_DegenFaces()
  smos_sprintf(sBuff,        _T(" DegenFace:[ %1lu = %s"),
             m_bBadDegenFace,
               m_bBadDegenFace == TRUE   ? _T("TRUE] Bad")
             : m_bBadDegenFace == FALSE  ? _T("FALSE] Okay")
             : m_bBadDegenFace == UNSURE ? _T("UNSURE] Not Yet Set")
             :                             _T("UnKnown Val] Error - fix bug")) ; 
  smos_WriteBuffer(sBuff) ;

  // Prop State: 
  smos_sprintf(sBuff,        _T("\n   PropState :[%s]"), 
                     m_ePropStage == SM_PROPSTAGE_0  ? _T("SM_PROPSTAGE_0  = SmFaceProps immediately after construction")
                   : m_ePropStage == SM_PROPSTAGE_GAPSa ? _T("SM_PROPSTAGE_GAPSa = during SmFaceProps::SetProps_Gaps() call")      
                   : m_ePropStage == SM_PROPSTAGE_GAPS  ? _T("SM_PROPSTAGE_GAPS  = after SmFaceProps::SetProps_Gaps() call")      
                   : m_ePropStage == SM_PROPSTAGE_2a ? _T("SM_PROPSTAGE_2a = during SmFaceProps::SetProps_Stage2() call")    
                   : m_ePropStage == SM_PROPSTAGE_2  ? _T("SM_PROPSTAGE_2  = after SmFaceProps::SetProps_Stage2() call")    
                   : m_ePropStage == SM_PROPSTAGE_3a ? _T("SM_PROPSTAGE_3a = during SmFaceProps::SetProps_Stage3() call")
                   : m_ePropStage == SM_PROPSTAGE_3  ? _T("SM_PROPSTAGE_3  = after SmFaceProps::SetProps_Stage3() call")
                   : _T("Unknown Value")) ; 
  smos_WriteBuffer(sBuff) ;         

  smos_sprintf(sBuff, _T("%s"), _T("\n     Constructed      :[DONE] - (save m_cpContext val)") ) ; 
  smos_WriteBuffer(sBuff) ;

  smos_sprintf(sBuff, _T("\n     SetProps_Gaps()  :[%s] - (save DegenGeoms, CoincidentGeoms, and Orig/Fixed Tol Vals)"), 
                      m_ePropStage == SM_PROPSTAGE_GAPSa ? _T("AMID") 
                    : m_ePropStage >= SM_PROPSTAGE_GAPS  ? _T("DONE") : _T("ToDo")) ; 
  smos_WriteBuffer(sBuff) ;

  smos_sprintf(sBuff, _T("\n     SetProps_Stage2():[%s] - (save SurfaceProps, FlatCorners, and Missing/Crossed Seam Vals)"), 
                      m_ePropStage == SM_PROPSTAGE_2a ? _T("AMID") 
                    : m_ePropStage >= SM_PROPSTAGE_2  ? _T("DONE") : _T("ToDo")) ; 
  smos_WriteBuffer(sBuff) ;

  smos_sprintf(sBuff, _T("\n     SetProps_Stage3():[%s] - (save LoopProp and Missing Pole Vals)"),                        
                      m_ePropStage == SM_PROPSTAGE_3a ? _T("AMID") 
                    : m_ePropStage >= SM_PROPSTAGE_3  ? _T("DONE") : _T("ToDo")) ; 
  smos_WriteBuffer(sBuff) ;
  
  // m_eBeenThroughMoveSeam
  smos_sprintf(sBuff,        _T("\n      BeenThroughMoveSeam        :[%s]"),
                   m_eBeenThroughMoveSeam == TRUE ? _T("TRUE") : m_eBeenThroughMoveSeam == FALSE ? _T("FALSE") : _T("UNSURE")) ;
  smos_WriteBuffer(sBuff) ;

  // m_eBeenThroughSplitEdgesAtSeam
  smos_sprintf(sBuff,        _T("\n      BeenThroughEdgesSplitAtSeam:[%s]"),
                   m_eBeenThroughSplitEdgesAtSeam == TRUE ? _T("TRUE") : m_eBeenThroughSplitEdgesAtSeam == FALSE ? _T("FALSE") : _T("UNSURE")) ;
  smos_WriteBuffer(sBuff) ;

  // m_eBeenThroughSplitFacesAtSeam 
  smos_sprintf(sBuff,        _T("\n      BeenThroughSplitFacesAtSeam:[%s]"),
                   m_eBeenThroughSplitFacesAtSeam == TRUE ? _T("TRUE") : m_eBeenThroughSplitFacesAtSeam == FALSE ? _T("FALSE") : _T("UNSURE")) ;
  smos_WriteBuffer(sBuff) ;

  // NotYet Supported Face Problems
  if(m_eNotYetFaceType != SM_HNF_NO_PROBS)
    {
      smos_sprintf(sBuff,        _T("\n             : Not Yet Supported Prob:[%s]:%s"), 
                   m_bBadHasNotYetProb == TRUE ? _T("TRUE") : m_bBadHasNotYetProb == FALSE ? _T("FALSE") : _T("UNSURE"), 
                   m_eNotYetFaceType == SM_HNF_NO_PROBS ? _T("[NONE] - Okay") 
                 : m_eNotYetFaceType == SM_HNF_NO_CLASSIFY_ZERO_CROSSCNT ? _T("[TgtLoop NotCrvClassifiable Zero CrossCnt - Maybe DegenFace or CoincidentEdges] - Bad")
                 : m_eNotYetFaceType == SM_HNF_NO_CLASSIFY_ODD_CROSSCNT  ? _T("[TgtLoop NotCrvClassifiable Odd CrossCnt on OpenSurface Face - Maybe DegenFace or CoincidentEdges] - Bad")
                 : _T("[Undescribed Problem] - Bad")) ;
      smos_WriteBuffer(sBuff) ;

    } // end NotYetSupported Face Problems check

  // Section 1: Props and Probs Post SetProps_Gaps
  if(m_ePropStage >= SM_PROPSTAGE_GAPS) 
     { 
       smos_WriteBuffer(_T("\n  Section 1: Post SetProps_Gaps Props and Probs")) ;
       DumpProps_Gaps() ;

       // 1 = Post SetProps_Gaps() problems
       if(HasProblems(1) == TRUE) { DumpProblems_Gaps() ; }
       else                       { smos_WriteBuffer(_T("\n   Problems  : [None] Okay")) ; }
     }
  else { smos_WriteBuffer(_T("\n  Section 1: Data Not Yet Set by SetProps_Gaps()")) ; }

  // Section 2: Props and Probs Post SetProps_Stage2
  if(m_ePropStage >= SM_PROPSTAGE_2) 
     { 
       smos_WriteBuffer(_T("\n  Section 2: Post SetProps_Stage2 Props and Probs")) ;
       DumpProps_Stage2() ; // SeamU and SeamV dumps done below

       // 2 = Post SetProps_Stage2() problems
       if(HasProblems(2) == TRUE) { DumpProblems_Stage2() ; }
       else                       { smos_WriteBuffer(_T("\n   Problems  : [None] Okay")) ; }
     }
  else { smos_WriteBuffer(_T("\n  Section 2: Data Not Yet Set by SetProps_Stage2()")) ; }

  // Section 3: Props and Probs Post SetProps_Stage3
  if(m_ePropStage >= SM_PROPSTAGE_3) 
     { 
       smos_WriteBuffer(_T("\n  Section 3: Post SetProps_Stage3 Props and Probs")) ;
       DumpProps_Stage3() ; // Per Loop data dumps done below

       // 4 = Post SetProps_Stage3() problems
       if(HasProblems(4) == TRUE) { DumpProblems_Stage3() ; }
       else                       { smos_WriteBuffer(_T("\n   Problems  : [None] Okay")) ; }
     }
  else { smos_WriteBuffer(_T("\n  Section 3: Data Not Yet Set by SetProps_Stage3()")) ; }

  // Section 4: Dump Long Data when asked and present
  if(bLong == TRUE)
    {
      // when there is section 4 data
      if(   (m_ePropStage >= SM_PROPSTAGE_2)
         || (   m_ePropStage >= SM_PROPSTAGE_3
             && m_sLoopProps.GetSize() > 0))
        {
          smos_WriteBuffer(_T("\n  Section 4: Dump Long Data when asked (SeamCrvClassifies, LoopContainmentTree, and LoopProps)")) ;
          
          // SeamU ClassifyCrv - when present
          if(m_ePropStage >= SM_PROPSTAGE_2)
            {
              if(   m_bClosedU   == TRUE
                 && m_pCrvClassU != NULL)
                { 
                  smos_WriteBuffer( _T("\n   Begin Seam_U CrvClassification   :")) ;
                  m_pCrvClassU->Dump() ;
                  smos_WriteBuffer( _T("\n   End Seam_U CrvClassification     :")) ;
          
                  SmTouchData::DumpTouchList(_T("Raw_U :"), m_sRawTouchListU,  FALSE) ;
                  SmTouchData::DumpTouchList(_T("Done_U:"), m_sDoneTouchListU, FALSE) ;
                } // end SeamU data to dump check
              else
                {
                  smos_WriteBuffer(_T("\n   No Seam_U CrvClassification to report: NONE")) ; 
                }
          
              // SeamV ClassifyCrv - when present
              if(   m_bClosedV   == TRUE
                 && m_pCrvClassV != NULL)
                { 
                  smos_WriteBuffer( _T("\n   Begin Seam_V CrvClassification   :")) ;
                  m_pCrvClassV->Dump() ;
                  smos_WriteBuffer( _T("\n   End Seam_V CrvClassification     :")) ;
          
                  SmTouchData::DumpTouchList(_T("Raw_V :"), m_sRawTouchListV,  FALSE) ;
                  SmTouchData::DumpTouchList(_T("Done_V:"), m_sDoneTouchListV, FALSE) ;
          
                } // end SeamV data to dump check
              else
                {
                  smos_WriteBuffer(_T("\n   No Seam_V CrvClassification to report: NONE")) ; 
                }
            } // end m_ePropStage >= SM_PROPSTAGE_2 check - to output Seams
          
          // Per Loop Props - when present
          if(   m_ePropStage >= SM_PROPSTAGE_3
             && m_sLoopProps.GetSize() > 0) 
            { 
              // Looptree
              if(m_pLooptreeRoot)
                { m_pLooptreeRoot->Dump() ; }
          
              // m_sLoopProps Size
              smos_sprintf(sBuff, _T("\n  Contained LoopProps Cnt      :[%2lu]"), m_sLoopProps.GetSize()) ; 
          
              // PrettyPrint every LoopProps  
              for(ii=0;ii<m_sLoopProps.GetSize();ii++)
                {
                  m_sLoopProps[ii].Dump(ii) ; 
                } // end iter dumping every LoopProps
            } // end PerLoop_Data_to_dump existence check

          smos_WriteBuffer(_T("\n  End Section 4: Dump Long Data when asked (SeamCrvClassifies, LoopContainmentTree, and LoopProps)")) ;

        } // end section 4 data to dump branch
      else  // else no Section 4 data to dump branch
        {
          smos_WriteBuffer(_T("\n  Section 4: Long Data not yet set by SetProps_Stage2() or SetProps_Stage3()")) ;
        }

    } // end bLone == TRUE check

  // for ease of debugging - reproduce the Problem statements at bottom of PrettyPrint report
#ifdef SM_DEBUG_CODE
  if(bLong == TRUE)
    {

      smos_WriteBuffer(_T("\n  Section 5: Repeat FaceProblems Dump - (* LongDump DebugMode Only)")) ;
      DumpProblems_Gaps() ;
      smos_WriteBuffer(_T("\n")) ; DumpProblems_Stage2() ; 
      smos_WriteBuffer(_T("\n")) ; DumpProblems_Stage3() ; 
      smos_WriteBuffer(_T("\n  End Section 5: FaceProblems Dump")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  if(lLabel == SM_UNDEF_ULONG) { smos_sprintf(sBuff, _T("%s"), _T("\nEnd   SmFaceProps Dump ")) ; }
  else                         { smos_sprintf(sBuff, _T("\nEnd   SmFaceProps Dump [%3ld]"), lLabel) ; }
  smos_WriteBuffer(sBuff);
  
} // end SmFaceProps::Dump

/*******************************************************************//**
PURPOSE: Formatted Write for SmFaceProps 
         SetProps_Gaps()] properties
NOTES:
***********************************************************************/
void SmFaceProps::DumpProps_Gaps() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // properties

  // m_pFace m_lFaceIndx
  smos_sprintf(sBuff,        _T("\n   Properties: Face         [0x%p] FaceIndx[%4lu]"), m_pFace, m_lFaceIndx) ;
  smos_sprintf(sBuffForFile, _T("\n   Properties: Face         [%s] FaceIndx[%4lu]"),   m_pFace ? _T("notNULL") : _T("NULL"), m_lFaceIndx) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // m_pFace->Surface
  if(m_pFace)
    { smos_sprintf(sBuff,        _T("\n             : Surface      [0x%p]"), m_pFace->GetSurface()) ;
      smos_sprintf(sBuffForFile, _T("\n             : Surface      [%s]"),   m_pFace->GetSurface() ? _T("notNULL") : _T("NULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile) ;
    }

  // m_sZoneTol3d
  smos_sprintf(sBuff,        _T("\n             : ZoneTol3d    [%5.7lf]"), m_sZoneTol3d.val) ;
  smos_WriteBuffer(sBuff) ;

  // m_sOrigZoneTol3d
  smos_sprintf(sBuff,        _T("\n             : OrigZoneTol3d[%5.7lf]"), m_sOrigZoneTol3d.val) ;
  smos_WriteBuffer(sBuff) ;

} // end SmFaceProps::DumpProps_Gaps

/*******************************************************************//**
PURPOSE: Formatted Write for SmFaceProps 
         SetProps_Gaps()] problems
NOTES:
***********************************************************************/
void SmFaceProps::DumpProblems_Gaps() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // problems

  // m_sBadSmallZoneTol3d
  smos_sprintf(sBuff, _T("\n   Problems  : Bad_SmallZoneTol3d          :[%s" ), 
               m_bBadSmallZoneTol3d == TRUE  ? _T("TRUE]  Bad - Calc new ZoneTol3d vals for Objects with too small ZoneTol3d vals")
             : m_bBadSmallZoneTol3d == FALSE ? _T("FALSE] Okay")
             :                                 _T("UNSURE] Okay - hasn't been set yet")) ;
  smos_WriteBuffer(sBuff) ;

  // m_sBadLargeZoneTol3d
  smos_sprintf(sBuff, _T("\n             : Bad_LargeZoneTol3d          :[%s" ), 
               m_bBadLargeZoneTol3d == TRUE  ? _T("TRUE]  Bad - Calc new ZoneTol3d vals for Objects with too large ZoneTol3d vals")
             : m_bBadLargeZoneTol3d == FALSE ? _T("FALSE] Okay")
             :                                 _T("UNSURE] Okay - hasn't been set yet")) ;
  smos_WriteBuffer(sBuff) ;

} // end SmFaceProps::DumpState1_Probs

/*******************************************************************//**
PURPOSE: Formatted Write for SmFaceProps 
         SetProps_Stage2()] properties
NOTES:
***********************************************************************/
void SmFaceProps::DumpProps_Stage2() const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // Properties: m_sNaturalUVDomain
  smos_sprintf(sBuff, _T("\n   Properties: NatUVDomain       :Min:[%lf, %lf] Max:[%lf, %lf]"),
             m_sNaturalUVDomain.GetUMin(), m_sNaturalUVDomain.GetVMin(),
             m_sNaturalUVDomain.GetUMax(), m_sNaturalUVDomain.GetVMax()) ; 
  smos_WriteBuffer(sBuff) ;

  // SheetFace
  smos_sprintf(sBuff, _T("\n             : SheetFace         :[%s]"), 
               m_eSheet == SM_MP_UNDEF   ? _T("Sheet propetry value not yet known") 
             : m_eSheet == SM_MP_NOPROP  ? _T("Face is not a sheet") 
             : m_eSheet == SM_MP_HASPROP ? _T("Face is sheet - not yet known if that's okay or a problem") 
             : m_eSheet == SM_MP_OKAY    ? _T("Face is sheet and is known to be okay") 
             : m_eSheet == SM_MP_PROB    ? _T("Face is sheet and is known to be problem needing fixing") 
             : m_eSheet == SM_MP_FIXED   ? _T("Face is sheet that was a problem and is now fixed") 
             :                     _T("")) ; 
  smos_WriteBuffer(sBuff) ;

  // m_bClosedU, m_eClosedValU
  smos_sprintf(sBuff, _T("\n             : ClosedU           :[%s]%s"),
             m_bClosedU ? (  m_eClosedValU == SM_CT_C0        ? _T("C0")
                           : m_eClosedValU == SM_CT_G1        ? _T("G1")
                           : m_eClosedValU == SM_CT_G1R       ? _T("Ruled G1")
                           : m_eClosedValU == SM_CT_G1_G2     ? _T("G2 = G1+ContCurvatureVec")
                           : m_eClosedValU == SM_CT_G1_G2_G3  ? _T("G3 = G2+ContG3Vec")
                           : m_eClosedValU == SM_CT_C1        ? _T("C1")
                           : m_eClosedValU == SM_CT_C1_G2     ? _T("C1-G2=C1+ContCurvatureVec")
                           : m_eClosedValU == SM_CT_C1_G2_G3  ? _T("C1-G3=C1-G2+ContG3Vec")
                           : m_eClosedValU == SM_CT_C1_C2     ? _T("C2")
                           : m_eClosedValU == SM_CT_C1_C2_G3  ? _T("C2-G3=C2+ContG3Vec")
                           : m_eClosedValU == SM_CT_C1_C2_C3  ? _T("C3")
                           : m_eClosedValU == SM_CT_CINFINITY ? _T("C-Infinite") : _T("undefined value"))
                        : _T("FALSE"),
             m_bClosedU ? _T(" (* SeamU ClassifyCrv listed below)")
                        : _T("")) ; 
  smos_WriteBuffer(sBuff) ;

  // m_bClosedV, m_eClosedValV
  smos_sprintf(sBuff, _T("\n             : ClosedV           :[%s]%s"),
             m_bClosedV ? (  m_eClosedValV == SM_CT_C0        ? _T("C0")
                           : m_eClosedValV == SM_CT_G1        ? _T("G1")
                           : m_eClosedValV == SM_CT_G1R       ? _T("Ruled G1")
                           : m_eClosedValV == SM_CT_G1_G2     ? _T("G2 = G1+ContCurvatureVec")
                           : m_eClosedValV == SM_CT_G1_G2_G3  ? _T("G3 = G2+ContG3Vec")
                           : m_eClosedValV == SM_CT_C1        ? _T("C1")
                           : m_eClosedValV == SM_CT_C1_G2     ? _T("C1-G2=C1+ContCurvatureVec")
                           : m_eClosedValV == SM_CT_C1_G2_G3  ? _T("C1-G3=C1-G2+ContG3Vec")
                           : m_eClosedValV == SM_CT_C1_C2     ? _T("C2")
                           : m_eClosedValV == SM_CT_C1_C2_G3  ? _T("C2-G3=C2+ContG3Vec")
                           : m_eClosedValV == SM_CT_C1_C2_C3  ? _T("C3")
                           : m_eClosedValV == SM_CT_CINFINITY ? _T("C-Infinite") : _T("undefined value"))
                        : _T("FALSE"),
             m_bClosedV ? _T(" (* SeamV ClassifyCrv listed below)")
                        : _T("")) ; 
  smos_WriteBuffer(sBuff) ;

  // m_bClosedSurf
  smos_sprintf(sBuff, _T("\n             : Closed Surface    :[%s] TRUE = Sphere or Torus homeomorph"), 
                      m_bClosedSurf == TRUE  ? _T("TRUE ") 
                    : m_bClosedSurf == FALSE ? _T("FALSE") 
                    :                          _T("UNSURE")) ; 
  smos_WriteBuffer(sBuff) ;

  // m_ePoles
  smos_sprintf(sBuff, _T("\n             : Poles             :[%s]"), 
               m_ePoles == SM_MP_UNDEF   ? _T("Pole propetry value not yet known") 
             : m_ePoles == SM_MP_NOPROP  ? _T("Face has no poles") 
             : m_ePoles == SM_MP_HASPROP ? _T("Face has poles - not yet known if that's okay or a problem") 
             : m_ePoles == SM_MP_OKAY    ? _T("Face has poles and is known to be okay") 
             : m_ePoles == SM_MP_PROB    ? _T("Face has poles and is known to be problem needing fixing") 
             : m_ePoles == SM_MP_FIXED   ? _T("Face has poles that was a problem and is now fixed") 
             :                     _T("")) ; 
  smos_WriteBuffer(sBuff) ;

  // m_lPoles
  smos_sprintf(sBuff, _T("\n             : Poles             :[%s%s%s%s]  (TRUE = PoleTol < IW_EFF_ZERO)"),
               (m_lPoles == SM_SS_NONE) ? _T("None") : (m_lPoles & SM_SS_UMIN)  ? _T("UMin, ") : _T(""),
               (m_lPoles == SM_SS_NONE) ? _T("")     : (m_lPoles & SM_SS_VMIN)  ? _T("VMin, ") : _T(""),
               (m_lPoles == SM_SS_NONE) ? _T("")     : (m_lPoles & SM_SS_UMAX)  ? _T("UMax, ") : _T(""),
               (m_lPoles == SM_SS_NONE) ? _T("")     : (m_lPoles & SM_SS_VMAX)  ? _T("VMax")   : _T("")) ;
  smos_WriteBuffer(sBuff) ;

  // m_lApproxPoles
  smos_sprintf(sBuff, _T("\n             : ApproxPoles       :[%s%s%s%s]  (TRUE = IW_EFF_ZERO < PoleTol < ZoneTol3d)"),
               (m_lApproxPoles == SM_SS_NONE) ? _T("None") : ((m_lApproxPoles & ~m_lPoles) & SM_SS_UMIN)  ? _T("UMin, ") : _T(""),
               (m_lApproxPoles == SM_SS_NONE) ? _T("")     : ((m_lApproxPoles & ~m_lPoles) & SM_SS_VMIN)  ? _T("VMin, ") : _T(""),
               (m_lApproxPoles == SM_SS_NONE) ? _T("")     : ((m_lApproxPoles & ~m_lPoles) & SM_SS_UMAX)  ? _T("UMax, ") : _T(""),
               (m_lApproxPoles == SM_SS_NONE) ? _T("")     : ((m_lApproxPoles & ~m_lPoles) & SM_SS_VMAX)  ? _T("VMax")   : _T("")) ;
  smos_WriteBuffer(sBuff) ;

  // m_lNoVertexPoles
  smos_sprintf(sBuff, _T("\n             : NoVertex Poles    :[%s%s%s%s]%s (TRUE = Pole not marked by a Vertex)"),
               (m_lNoVertexPoles == SM_SS_NONE) ? _T("None") : (m_lNoVertexPoles & SM_SS_UMIN)  ? _T("UMin, ") : _T(""),
               (m_lNoVertexPoles == SM_SS_NONE) ? _T("")     : (m_lNoVertexPoles & SM_SS_VMIN)  ? _T("VMin, ") : _T(""),
               (m_lNoVertexPoles == SM_SS_NONE) ? _T("")     : (m_lNoVertexPoles & SM_SS_UMAX)  ? _T("UMax, ") : _T(""),
               (m_lNoVertexPoles == SM_SS_NONE) ? _T("")     : (m_lNoVertexPoles & SM_SS_VMAX)  ? _T("VMax")   : _T(""),
               (m_lNoVertexPoles == SM_SS_NONE) ? _T("")     : _T(" - Might be Okay: only a prob when UnmarkedPole is part of Face boundary") ) ;
  smos_WriteBuffer(sBuff) ;

  // UMin, VMin, UMax, VMax Pole Points and Normals
  if(m_lPoles != SM_SS_NONE)
    {
      for(ii=0;ii<4;ii++)
        {
          ULONG lTest =   (ii == 0) ? SM_SS_UMIN
                        : (ii == 1) ? SM_SS_VMIN
                        : (ii == 2) ? SM_SS_UMAX
                                    : SM_SS_VMAX ;

          if((m_lPoles & lTest) || (m_lApproxPoles & lTest))
            {                                
              smos_sprintf(sBuff, _T("\n              : %s_Pole Pt     :"), 
                           (ii == 0) ? _T("UMin")
                         : (ii == 1) ? _T("VMin")
                         : (ii == 2) ? _T("UMax")
                                     : _T("VMax")) ; 
              smos_WriteBuffer(sBuff) ;          m_sPolePoints[ii]. Dump(FALSE) ; 
              smos_WriteBuffer(_T(" Norm : ")) ; m_sPoleNormals[ii].Dump(FALSE) ; 
            }
        } // end iter PolePts and PoleNormals
    } // end Poles exist check

} // end SmFaceProps::DumpProps_Stage2

/*******************************************************************//**
PURPOSE: Formatted Write for SmFaceProps 
         SetProps_Stage2()] problems
NOTES:
***********************************************************************/
void SmFaceProps::DumpProblems_Stage2() const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // bad m_eSheet
  smos_sprintf(sBuff, _T("\n   Problems  : Bad_SheetFace               :[%s]%s"),
               (m_eSheet == SM_MP_PROB)  ? _T("TRUE") : _T("FALSE"),
               (m_eSheet == SM_MP_UNDEF) ? _T(" Okay - hasn't been set yet")
             : (m_eSheet == SM_MP_PROB)  ? _T(" Bad - OrigFace Labeled as Sheet actually bounds two Closed Shells - Split Region")
             :                             _T(" Okay")) ; 
  smos_WriteBuffer(sBuff) ; 

  // m_lBadFlatCorners
  smos_sprintf(sBuff, _T("\n             : Bad_FlatCorners             :[%s%s%s%s]%s"),
               (m_lBadFlatCorners == SM_FC_NONE)      ? _T("None")
             : (m_lBadFlatCorners  & SM_FC_UMIN_VMIN) ? _T("UMin-VMin, ") : _T(""),
               (m_lBadFlatCorners == SM_FC_NONE)      ? _T("")
             : (m_lBadFlatCorners  & SM_FC_UMIN_VMAX) ? _T("UMin-VMax, ") : _T(""),
               (m_lBadFlatCorners == SM_FC_NONE)      ? _T("")
             : (m_lBadFlatCorners  & SM_FC_UMAX_VMIN) ? _T("UMax-VMin, ") : _T(""),
               (m_lBadFlatCorners == SM_FC_NONE)      ? _T("")
             : (m_lBadFlatCorners  & SM_FC_UMAX_VMAX) ? _T("UMax-VMax")   : _T(""),
               m_lBadFlatCorners == SM_UNDEF_ULONG ? _T(" Okay - hasn't been set yet")
             : m_lBadFlatCorners != SM_FC_NONE     ? _T(" Bad - Replace and Trim NewSrf to current 3d Bndrys - Fix not yet Implemented")
                                                   : _T(" Okay")) ;
  smos_WriteBuffer(sBuff) ;

  // if(m_lBadFlatCorners) Write m_sFlatCornerPoints
  if(m_lBadFlatCorners != SM_FC_NONE && m_lBadFlatCorners != SM_UNDEF_ULONG)
    {
      for(ii=0;ii<4;ii++)
        {
          ULONG lTest =   (ii == 0) ? SM_FC_UMIN_VMIN
                        : (ii == 1) ? SM_FC_UMIN_VMAX
                        : (ii == 2) ? SM_FC_UMAX_VMIN
                                    : SM_FC_UMAX_VMAX ;

          if(m_lBadFlatCorners & lTest)
            {                       
              smos_sprintf(sBuff, _T("\n             : %s BadFlatCrn Pt     :"),
                         (  (ii == 0) ? _T("UMin-VMin")
                          : (ii == 1) ? _T("UMin-VMax")
                          : (ii == 2) ? _T("UMax-VMin")
                                      : _T("UMax-VMax"))) ; 
              smos_WriteBuffer(sBuff) ; m_sFlatCornerPoints[ii].Dump(FALSE) ; 
            }
        } // end iter FlatCorner Points
    } // end m_lBadFlatCorners existence check

  // m_eBadCrossedSeam
  smos_sprintf(sBuff, _T("\n             : Bad_EdgeCrossingSeam[%s] :[%s]%s"),
             (m_bBadCrossedSeam == TRUE) ? _T("TRUE ") : _T("FALSE"),
               (m_eBadCrossedSeam == SM_SP_U)       ? _T("UDir")
             : (m_eBadCrossedSeam == SM_SP_V)       ? _T("VDir")
             : (m_eBadCrossedSeam == SM_SP_BOTH)    ? _T("UDir and VDir")
             : (m_eBadCrossedSeam == SM_SP_NEITHER) ? _T("None") 
             : (m_eBadCrossedSeam == SM_SP_UNKNOWN) ? _T("UnInit") 
             : _T("Unexpected Value"),
             (   m_eBadCrossedSeam == SM_SP_U 
              || m_eBadCrossedSeam == SM_SP_V 
              || m_eBadCrossedSeam == SM_SP_BOTH)   ? _T(" Bad - If possible MoveSeam out of Face Else SplitEdges at Seam Crossings")
               : m_eBadCrossedSeam == SM_SP_UNKNOWN ? _T(" Okay - hasn't been set yet")                                     
                                                    : _T(" Okay")) ;
  smos_WriteBuffer(sBuff) ; 

  // m_eBadMissingSeam
  smos_sprintf(sBuff, _T("\n             : Bad_MissingSeamEdge [%s] :[%s]%s"),
             (m_bBadMissingSeam == TRUE) ? _T("TRUE ") : _T("FALSE"),
               (m_eBadMissingSeam == SM_SP_U)       ? _T("UDir")
             : (m_eBadMissingSeam == SM_SP_V)       ? _T("VDir")
             : (m_eBadMissingSeam == SM_SP_BOTH)    ? _T("UDir and VDir")
             : (m_eBadMissingSeam == SM_SP_NEITHER) ? _T("None") 
             : (m_eBadMissingSeam == SM_SP_UNKNOWN) ? _T("UnInit") 
             : _T("Unexpected Value"),
            (   m_eBadMissingSeam == SM_SP_U 
             || m_eBadMissingSeam == SM_SP_V 
             || m_eBadMissingSeam == SM_SP_BOTH)   ? _T(" Bad - If possible MoveSeam out of Face else AddSeamEdge for every MissingSeamSegment")
              : m_eBadMissingSeam == SM_SP_UNKNOWN ? _T(" Okay - hasn't been set yet")                                     
                                                   : _T(" Okay")) ;
  smos_WriteBuffer(sBuff) ;

  // m_eBadNearMissSeam
  smos_sprintf(sBuff, _T("\n             : Bad_NearMissSeamEdge[%s] :[%s]%s"),
             (m_bBadNearMissSeam == TRUE) ? _T("TRUE ") : _T("FALSE"),
               (m_eBadNearMissSeam == SM_SP_U)       ? _T("UDir")
             : (m_eBadNearMissSeam == SM_SP_V)       ? _T("VDir")
             : (m_eBadNearMissSeam == SM_SP_BOTH)    ? _T("UDir and VDir")
             : (m_eBadNearMissSeam == SM_SP_NEITHER) ? _T("None") 
             : (m_eBadNearMissSeam == SM_SP_UNKNOWN) ? _T("UnInit") 
             : _T("Unexpected Value"),
             (   m_eBadNearMissSeam == SM_SP_U 
              || m_eBadNearMissSeam == SM_SP_V 
              || m_eBadNearMissSeam == SM_SP_BOTH)   ? _T(" Bad - NotYet - future Fix_BadGaps() will tighten NearMiss TopoConnections to Exact connects - NearMiss connects are still valid")
               : m_eBadNearMissSeam == SM_SP_UNKNOWN ? _T(" Okay - hasn't been set yet")                                     
                                                     : _T(" Okay")) ; 
  smos_WriteBuffer(sBuff) ; 

} // end SmFaceProps::DumpState2_Probs

/*******************************************************************//**
PURPOSE: Formatted Write for SmFaceProps 
         SetProps_Stage3()] properties
NOTES:
***********************************************************************/
void SmFaceProps::DumpProps_Stage3() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  SmTArray<SmLoop *> sLoops ;
  m_pFace->GetLoops(sLoops) ;

  // m_lClosedLoopCnt
  smos_sprintf(sBuff, _T("\n   Properties: LoopCnt          :[%2lu]%s"), 
             sLoops.GetSize(),
             sLoops.GetSize() > 1 ? _T(" (* PerLoop Data and LoopContainment Tree listed below)")
                                  : _T("")) ;    
  smos_WriteBuffer(sBuff) ;

  // m_lClosedLoopCnt
  smos_sprintf(sBuff, _T("\n             : ClosedLoopCnt    :[%2lu]"), m_lClosedLoopCnt) ;    
  smos_WriteBuffer(sBuff) ;

  // m_lOuterLoopCnt
  smos_sprintf(sBuff, _T("\n             : OuterLoopCnt     :[%2lu]%s"), 
             m_lOuterLoopCnt,
             m_lOuterLoopCnt != 1 ? _T(" Bad - Every Face must have exactly one OuterLoop")
                                  : _T(" Okay")) ;
  smos_WriteBuffer(sBuff) ;

} // end SmFaceProps::DumpProps_Stage3

/*******************************************************************//**
PURPOSE: Formatted Write for SmFaceProps 
         SetProps_Stage3()] problems
NOTES:
***********************************************************************/
void SmFaceProps::DumpProblems_Stage3() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // m_bBadLoopProblems    
  smos_sprintf(sBuff, _T("\n   Problems  : Has Any Loop Problems       :[%s"), 
                    m_bBadLoopProblems == FALSE ? _T("FALSE] - Okay") 
                  : m_bBadLoopProblems == TRUE  ? _T("TRUE ] - Bad") 
                                                : _T("UNSURE]") ) ;
  smos_WriteBuffer(sBuff) ;

  // m_lOuterLoopCnt    
  smos_sprintf(sBuff, _T("\n   Problems  : BadOuterLoop_Cnt            :[%2lu]%s"), 
             m_lOuterLoopCnt,
             (m_lOuterLoopCnt != 1
              ? (   (m_lOuterLoopCnt == SM_UNDEF_ULONG) ? _T(" Okay - hasn't been set yet")
                  : (m_lOuterLoopCnt == 0 && m_lBadNoArea_LoopCnt > 0 && m_bBadUnpairedNoArea_Loop == FALSE) 
                       ? _T(" Bad - AddMissingSeam to connect NoArea_Loop pairs into OuterLoops")
                  : (m_lOuterLoopCnt == 0 && (m_lBadNoArea_LoopCnt == 0 || m_bBadUnpairedNoArea_Loop != FALSE))
                       ? _T(" Bad - Face is missing OuterLoop - No Fix available")
                  : (m_lOuterLoopCnt > 1)
                       ? _T(" Bad - Split Face until One_Face_Per_OuterLoop")
                  :      _T(" Bad - Case not yet supported - needs review")
                )
               : (_T(" Okay")))) ;
  smos_WriteBuffer(sBuff) ;

  // m_bBadOuterLoopOrder
  smos_sprintf(sBuff, _T("\n             : Bad_OuterLoop_Order         :%s%s"), 
                      m_bBadOuterLoopOrder == TRUE  ? _T("[TRUE] ") 
                    : m_bBadOuterLoopOrder == FALSE ? _T("[FALSE]") 
                    :                                 _T("[UNSURE]   "),
                      m_bBadOuterLoopOrder == TRUE  ? _T(" Bad - Reorder LoopList to place OuterLoop first")
                    : m_bBadOuterLoopOrder == FALSE ? _T(" Okay")
                    :                                 _T(" Okay - hasn't been set yet")  ) ; 
  smos_WriteBuffer(sBuff) ;

  // m_lBadOrient_LoopCnt
  smos_sprintf(sBuff, _T("\n             : Bad_OrientLoop_Cnt          :[%2lu]%s"),
             m_lBadOrient_LoopCnt,
             m_lBadOrient_LoopCnt == SM_UNDEF_ULONG ? _T(" Okay - hasn't been set yet")
           : m_lBadOrient_LoopCnt != 0              ? _T(" Bad - Flip BadLoop Orientations") 
                                                    : _T(" Okay")) ;
  smos_WriteBuffer(sBuff) ;

  // m_lBadNoArea_LoopCnt
  smos_sprintf(sBuff, _T("\n             : Bad_NoAreaLoop_Cnt          :[%2lu]%s"),
             m_lBadNoArea_LoopCnt,
             m_lBadNoArea_LoopCnt == SM_UNDEF_ULONG ? _T(" Okay - hasn't been set yet")
           : m_lBadNoArea_LoopCnt != 0              ? _T(" Bad - AddMissingSeam to connect NoArea_Loop pairs into OuterLoops")
                                                    : _T(" Okay")) ;
  smos_WriteBuffer(sBuff) ;

  // m_lBadNested_LoopCnt
  smos_sprintf(sBuff, _T("\n             : Bad_NestedLoop_Cnt          :[%2lu]%s"),
             m_lBadNested_LoopCnt,
             m_lBadNested_LoopCnt == SM_UNDEF_ULONG ? _T(" Okay - hasn't been set yet")
           : m_lBadNested_LoopCnt != 0              ? _T(" Bad - Split Loops inside InnerLoops from Face turning them into OuterLoops")
                                                    : _T(" Okay")) ; 
  smos_WriteBuffer(sBuff) ;

  // m_bBadUnpairedNoArea_Loop
  smos_sprintf(sBuff, _T("\n             : Bad_Unpaired_NoAreaLoop_Cnt :%s%s"), 
                     m_bBadUnpairedNoArea_Loop == TRUE  ? _T("[TRUE] ") 
                   : m_bBadUnpairedNoArea_Loop == FALSE ? _T("[FALSE]") 
                   :                                      _T("[UNSURE]   "),
                     m_bBadUnpairedNoArea_Loop == TRUE ? _T(" Bad - AddMissingSeam will not fix unpaired NoArea_Loops - Face is missing OuterLoop - No Fix available")
                   : m_bBadUnpairedNoArea_Loop == TRUE ? _T(" Okay")
                                                       : _T(" Okay - hasn't been set yet")) ;  
  smos_WriteBuffer(sBuff) ;
                                                                          
  // Total Bad_ClosedLoop Count
  smos_sprintf(sBuff, _T("\n             : Bad_ClosedLoop_TotCnt       :[%2lu]%s"),
                     m_lBadClosed3d_LoopCnt + m_lBadClosedPtr_LoopCnt,
                     m_lBadClosed3d_LoopCnt == SM_UNDEF_ULONG               ? _T(" Okay - hasn't been set yet")
                  : (m_lBadClosed3d_LoopCnt + m_lBadClosedPtr_LoopCnt) != 0 ? _T(" Bad - Assumed Loop is meant to be closed")
                                                                            : _T(" Okay"));
  smos_WriteBuffer(sBuff) ;

  // m_lBadClosed3d_LoopCnt
  smos_sprintf(sBuff, _T("\n             :   Bad_ClosedLoop_Cnt BadGaps:[%2lu]%s"), 
             m_lBadClosed3d_LoopCnt,
             m_lBadClosed3d_LoopCnt == SM_UNDEF_ULONG ? _T(" Okay - hasn't been set yet")
           : m_lBadClosed3d_LoopCnt != 0              ? _T(" Bad - Assumed Loop is meant to be closed")
                                                      : _T(" Okay")) ; 
  smos_WriteBuffer(sBuff) ;

  // m_lBadClosedPtr_LoopCnt
  smos_sprintf(sBuff, _T("\n             :   Bad_ClosedLoop_Cnt BadPtrs:[%2lu]%s"),
             m_lBadClosedPtr_LoopCnt,
             m_lBadClosedPtr_LoopCnt == SM_UNDEF_ULONG ? _T(" Okay - hasn't been set yet")
           : m_lBadClosedPtr_LoopCnt != 0              ? _T(" Bad - Assumed Loop is meant to be closed")
                                                       : _T(" Okay")) ; 
  smos_WriteBuffer(sBuff) ;

  // m_lBadMissingPoles
  smos_sprintf(sBuff, _T("\n             : Bad_MissingVertex_Pole      :%s%s%s%s%s%s"),
             (m_lBadMissingPoles == SM_UNDEF_ULONG) ? _T("[UNSURE") 
           : (m_lBadMissingPoles == SM_SS_NONE)     ? _T("[None")   
                                                    : _T("["),
           (m_lBadMissingPoles == SM_SS_NONE || m_lBadMissingPoles == SM_UNDEF_ULONG) ? _T("") : (m_lBadMissingPoles & SM_SS_UMIN)  ? _T("UMin, ") : _T(""),
           (m_lBadMissingPoles == SM_SS_NONE || m_lBadMissingPoles == SM_UNDEF_ULONG) ? _T("") : (m_lBadMissingPoles & SM_SS_VMIN)  ? _T("VMin, ") : _T(""),
           (m_lBadMissingPoles == SM_SS_NONE || m_lBadMissingPoles == SM_UNDEF_ULONG) ? _T("") : (m_lBadMissingPoles & SM_SS_UMAX)  ? _T("UMax, ") : _T(""),
           (m_lBadMissingPoles == SM_SS_NONE || m_lBadMissingPoles == SM_UNDEF_ULONG) ? _T("") : (m_lBadMissingPoles & SM_SS_VMAX)  ? _T("VMax")   : _T(""),
             m_lBadMissingPoles == SM_UNDEF_ULONG ? _T("]    Okay - hasn't been set yet")
           : m_lBadMissingPoles != SM_SS_NONE     ? _T("] Bad - Add Vertices at PolePoints that are part of Face Boundary")
                                                  : _T("] Okay")) ; 
  smos_WriteBuffer(sBuff) ;

} // end SmFaceProps::DumpState3_Probs
