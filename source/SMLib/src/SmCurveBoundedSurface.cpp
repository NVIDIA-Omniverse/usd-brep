// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCurveBoundedSurface.cpp
* PURPOSE: Source file for SmCurveBoundedSurface methods.
**********************************************************************/

#include "StdAfx.h"
#include <SmCurveBoundedSurface.h>
#include <SmGraphicsOutput.h>


#ifdef SM_DEBUG_CODE
#include <SmCrvOnSurf.h>
#endif

/*******************************************************************//**
PURPOSE: Constructor for the composite surface.  This constructor assumes
   all boundary curves are valid.  It consumes the curves and surface 
   which are input.

NOTES: 
***********************************************************************/
SmCurveBoundedSurface::SmCurveBoundedSurface
  (SmSurface & rBasisSurface,
   const SmTArray<SmCurve*> * cp3DBoundaries,
   const SmTArray<SmCurve*> * cpUVBoundaries,
   SmBoolean bImplicitOuter)
 : m_pBasisSurface(&rBasisSurface),
   m_v3DBoundaries(*(new (*GetContext()) SmTArray<SmCurve*>(*GetContext()))),
   m_vUVBoundaries(*(new (*GetContext()) SmTArray<SmCurve*>(*GetContext()))),
   m_bImplicitOuter(bImplicitOuter)
{
  if (cp3DBoundaries) 
    {
      for (ULONG i=0; i<cp3DBoundaries->GetSize(); i++) 
        {
          m_v3DBoundaries.Add((*cp3DBoundaries)[i]);
        }  
    }
  if (cpUVBoundaries) 
    {
      for (ULONG i=0; i<cpUVBoundaries->GetSize(); i++) 
        {
          m_vUVBoundaries.Add((*cpUVBoundaries)[i]);
        }
    }

} // end SmCurveBoundedSurface::SmCurveBoundedSurface constructor

/*******************************************************************//**
PURPOSE: Destructor for the surface curve.  Parent curves are destroyed.

NOTES: 
***********************************************************************/
SmCurveBoundedSurface::~SmCurveBoundedSurface()
{
  for (ULONG i=0; i<m_v3DBoundaries.GetSize(); i++) 
    {
      SmCompositeCurve *pCC = SM_REINTERPRET_CAST(SmCompositeCurve*,m_v3DBoundaries[i]);
      SM_ASSERT(pCC != NULL) ; delete pCC ; pCC = NULL ;
    }

  for (ULONG j=0; j<m_vUVBoundaries.GetSize(); j++) 
    {
      SmCompositeCurve *pCC = SM_REINTERPRET_CAST(SmCompositeCurve*,m_vUVBoundaries[j]);
      SM_ASSERT(pCC != NULL) ; delete pCC ; pCC = NULL ;
    }

  if(m_pBasisSurface) { delete m_pBasisSurface ;  m_pBasisSurface = NULL ; }
  delete &m_vUVBoundaries; 
  delete &m_v3DBoundaries; 

} // end SmCurveBoundedSurface::~SmCurveBoundedSurface destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmCurveBoundedSurface

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmCurveBoundedSurface::operator==
  (const SmSurface& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmSurface::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmCurveBoundedSurface &rOther = (SmCurveBoundedSurface &)crOther ;

      // check equivalence of these objects
      bRtn =  (   (   ( m_pBasisSurface == rOther.m_pBasisSurface)
                   || ( m_pBasisSurface == NULL && rOther.m_pBasisSurface == NULL)
                   || (   m_pBasisSurface != NULL && rOther.m_pBasisSurface != NULL
                       && *m_pBasisSurface == *rOther.m_pBasisSurface))
               && m_v3DBoundaries.GetSize() == rOther.m_v3DBoundaries.GetSize()
               && m_vUVBoundaries.GetSize() == rOther.m_vUVBoundaries.GetSize()
               && m_bImplicitOuter          == rOther.m_bImplicitOuter) ;

      // check the array elements
      if(bRtn)
        {
          ULONG ii ;

          for(ii=0;ii<m_v3DBoundaries.GetSize()&&bRtn;ii++)
            {
              bRtn &= (   ( m_v3DBoundaries[ii] == rOther.m_v3DBoundaries[ii])
                       || ( m_v3DBoundaries[ii] == NULL && rOther.m_v3DBoundaries[ii] == NULL)
                       || (    m_v3DBoundaries[ii] != NULL && rOther.m_v3DBoundaries[ii] != NULL
                           && *m_v3DBoundaries[ii] == *rOther.m_v3DBoundaries[ii])) ;
            }

          for(ii=0;ii<m_v3DBoundaries.GetSize()&&bRtn;ii++)
            {
              bRtn &= (   ( m_vUVBoundaries[ii] == rOther.m_vUVBoundaries[ii])
                       || ( m_vUVBoundaries[ii] == NULL && rOther.m_vUVBoundaries[ii] == NULL)
                       || (    m_vUVBoundaries[ii] != NULL && rOther.m_vUVBoundaries[ii] != NULL
                           && *m_vUVBoundaries[ii] == *rOther.m_vUVBoundaries[ii])) ;
            }
        }
    }

  // all done
  return bRtn ;

} // end SmCurveBoundedSurface::operator==

// gwc removed unused function.
//      //   note: the function looks incomplete - it generates UVCurves for each
//      //         3D Curve but never stores them so when this function returns
//      //         all that work is lost.
//      /*******************************************************************//**
//      PURPOSE: This method will create the corresponding parameteric space
//         representation for a trimmed surface.  
//      
//      NOTES: Incomplete method - now all it does is test curve projection.
//      ***********************************************************************/
//      SmStatus SmCurveBoundedSurface::CreateParameterSpaceRep
//        ()
//      {
//        SmTArray<SmCurve*> sCurves;
//        SmTArray<SmCurve*> sCurves2;
//        SmTArray<SmCurve*> sCurves3;
//        SmTArray<SmBSplineCurve*> sUVCurves;
//      
//        for (ULONG i=0; i<m_v3DBoundaries.GetSize(); i++) 
//          {
//            sCurves3.ReSet();
//            SmCompositeCurve *pCC = SM_REINTERPRET_CAST(SmCompositeCurve*,m_v3DBoundaries[i]);
//            ULONG lDim;
//            SmBoolean bClosed;
//      
//            // First collect all lowest denominator curves
//            SER(pCC->GetCanonical(lDim,sCurves,bClosed,NULL));
//            for (ULONG j=0; j<sCurves.GetSize(); j++) 
//              {
//                SmCurve *pCurve = SM_CAST_PTR(SmBSplineCurve,sCurves[j]);
//                if (pCurve->IsKindOf(SmCompositeCurve_TYPE)) 
//                  {
//                    SmCompositeCurve *pCC = SM_REINTERPRET_CAST(SmCompositeCurve*,pCurve);
//                    SER(pCC->GetCanonical(lDim,sCurves2,bClosed,NULL));
//                    sCurves3.Append(sCurves2);
//                  }
//                else 
//                  {
//                    SM_ASSERT(pCurve->IsKindOf(SmBSplineCurve_TYPE));
//                    sCurves3.Add(pCurve);
//                  }
//              }
//      
//            // Now drop all curves collected from above
//            for (ULONG jj=0; jj<sCurves3.GetSize(); jj++) 
//              {
//                SmBSplineCurve *pCurve = SM_CAST_PTR(SmBSplineCurve,sCurves3[jj]);
//                SM_ASSERT(pCurve->IsKindOf(SmBSplineCurve_TYPE));
//      
//                double dLeng = pCurve->ApproximateLength(pCurve->GetNaturalInterval(),5);
//                double dTol = 1.0e-3 * (dLeng + 1.0);
//                double dGap, dDeviation;
//                SER(m_pBasisSurface->DropCurve(*GetContext(),
//                                              m_pBasisSurface->GetNaturalUVDomain(),
//                                              *pCurve,    
//                                              pCurve->GetNaturalInterval(),
//                                              dTol,
//                                              dGap,        // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
//                                              dDeviation,  // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
//                                              sUVCurves));
//                if (sUVCurves.GetSize() == 0) 
//                  {
//                    SM_ASSERT_ERR ;
//      #ifdef SM_DEBUG_CODE
//                    sm_GraphicsLoop();
//                    pCurve->Draw();
//                    ((SmBSplineSurface&)m_pBasisSurface)->DrawUV(3,3);
//                    sm_GraphicsLoop();
//      #endif
//                  }
//      #ifdef SM_DEBUG_CODE
//                for (ULONG kk=0; kk<sUVCurves.GetSize(); kk++) 
//                  {
//      
//                    SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve,sUVCurves[kk]);
//      SmBoolean bDebugMe = FALSE;
//                    if (bDebugMe) 
//                      {
//                        pBSC->Dump();
//                      }
//                    pBSC->Draw();
//                  }
//      #endif
//              }
//          }
//      
//        // all done
//        return SM_SUCCESS;
//      
//      } // end SmCurveBoundedSurface::CreateParameterSpaceRep

/*******************************************************************//**
PURPOSE: Create a trimmed surface Brep corresponding to the 
   curve bounded surface object.  

NOTES: Right now 3D curves never are kept - always regenerated.
   See comment area below to change this.
***********************************************************************/
SmStatus SmCurveBoundedSurface::CreateTrimmedSurfaceInBrep
  (SmBrep * pExistingBrep,
   SmFace *& rpNewFace)
{
    // Get the infinite region of the Brep.  For now assume that we have
    // no solids.
    SmRegion *pRegion = pExistingBrep->GetInfiniteRegion();

    // Get the context of the existing brep to be used in creation
    // of new subelements which go into the brep.
    const SmContext *pContext = pExistingBrep->GetContext();

    // Get the tolerance associated with the existing brep.  This
    // value will be used in any approximation which occurs.
    //double dTol = pExistingBrep->GetTolerance();

    // Create a surface which is a copy of the surface in the curve bounded surface object.
    SmSurface *pOldSurface = m_pBasisSurface ;
    SmSurface *pNewSurface =  NULL ;

    // Copy pOldSurface for new face, when possible as an analytic surface
    SER(pOldSurface->CopyAndAddAnalytics(*pContext,pNewSurface))
#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if (bDebugMe) 
      {
        pOldSurface->Dump() ;
        pNewSurface->Dump() ;

        smgfx_Erase() ;
        smgfx_SetLook(1,2, 0,0,1) ; pExistingBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 0,1,1) ; pOldSurface->DrawUV(); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 0,1,0) ; pNewSurface->DrawUV(); sm_GraphicsLoop();
        sm_GraphicsLoop() ;
      }
#endif


    // We need a bunch of arrays to do things with curves.
    SmTArray<ULONG>           sCurveLoops;
    SmTArray<SmOrientType>    sCurveOrients;

    SmTArray<SmCurve*>        sUVCurves;
    SmTArray<SmBSplineCurve*> sUVCurves3;
    SmTArray<SmCurve*>        sCurves;
    SmTArray<SmCurve*>        sCurves2;
    SmTArray<SmCurve*>        sCurves3;
    SmTArray<SmBoolean>       sBoolOrients;

    // Newly created curves will always end up in 
    // sCurves3 or sUVCurves3.  Make sure that if we
    // error out that these new curves are deleted by
    // putting the arrays into cleanup objects.
    SmObjsDelete<SmCurve*> sClean3d(&sCurves3);
    SmObjsDelete<SmBSplineCurve*> sCleanUV(&sUVCurves3);

    // In the definition of the curve bounded surface m_v3DBoundaries
    // corresponds to an array of loops which are composite curves.
    // This is how things are in IGES and the curve bounded surface is
    // more or less an image of an IGES trimmed surface (type 144 or 
    // type 143).
    for (ULONG i=0; i<m_v3DBoundaries.GetSize(); i++) {
        // Get the 2D and 3D composite curves for each loop
        SmCompositeCurve *pCC = SM_REINTERPRET_CAST(SmCompositeCurve*,m_v3DBoundaries[i]);
        SmCompositeCurve *pUVCC = SM_REINTERPRET_CAST(SmCompositeCurve*,m_vUVBoundaries[i]);
        ULONG lDim;
        SmBoolean bClosed;
        // Get the individual trimming curves out of the composite curve.
        SER(pCC->GetCanonical(lDim,sCurves,bClosed,&sBoolOrients));

        ULONG lCurvesInLoop = 0;

        // For each 3D trimming curve - if the curve is a composite create
        // the corresponding NURB from the composite.  If it is not just
        // make a copy of it and put it into the sCurves3 array.
        for (ULONG j=0; j<sCurves.GetSize(); j++) {
            if (sBoolOrients[j] == TRUE) {sCurveOrients.Add(SM_OT_SAME);}
            else {sCurveOrients.Add(SM_OT_OPPOSITE);}
            SmCurve *pCurve = sCurves[j];
            SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve,pCurve);
            SmBSplineCurve *p3DBSC = NULL ;
            if (pBSC) {
                p3DBSC = new (*pContext) SmBSplineCurve(*pBSC);
            }
            else {
                SmCompositeCurve *pCompCrv = SM_CAST_PTR(SmCompositeCurve,pCurve);
                if (pCompCrv) {
                  SER( pCompCrv->MakeCompositeNurb(*pContext,p3DBSC));
                }
                else { SER(SM_ERR); }
            }

            sCurves3.Add(p3DBSC);
            lCurvesInLoop ++;
#ifdef SM_DEBUG_CODE
            if (bDebugMe) 
              {
                p3DBSC->Dump();

                smgfx_Erase() ;
                smgfx_SetLook(1,2, 0,0,1) ; pExistingBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,1,1) ; pNewSurface->DrawUV(); sm_GraphicsLoop();
                smgfx_SetLook(3,4, 1,0,0) ; p3DBSC->Draw(); sm_GraphicsLoop() ;
                sm_GraphicsLoop() ;
              }
#endif
        }

        {  // Now do the same thing for the UV curves as was done for the 3D
            // curves in terms of fixing the nested composites.
            SER(pUVCC->GetCanonical(lDim,sUVCurves,bClosed,NULL));
            for (ULONG j=0; j<sUVCurves.GetSize(); j++) {
                SmCurve *pCurve = sUVCurves[j];
                SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve,pCurve);
                SmBSplineCurve *pUVBSC = NULL ;
                if (pBSC) {
                    pUVBSC = new (*pContext) SmBSplineCurve(*pBSC);
                }
                else {
                    SmCompositeCurve *pCompCrv = SM_CAST_PTR(SmCompositeCurve,pCurve);
                    if (pCompCrv) {
                        SER( pCompCrv->MakeCompositeNurb(*pContext,pUVBSC));
                    }
                    else { SER(SM_ERR); }
                }
                sUVCurves3.Add(pUVBSC);
#ifdef SM_DEBUG_CODE
                if (bDebugMe) 
                  {
                    pUVBSC->Dump();
                    SmCrvOnSurf sSurfCurve(*pUVBSC, *pNewSurface) ;
                    
                    smgfx_Erase() ;
                    smgfx_SetLook(1,2, 0,0,1) ; pExistingBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                    smgfx_SetLook(1,2, 0,1,1) ; pNewSurface->DrawUV(); sm_GraphicsLoop();
                    smgfx_SetLook(3,4, 1,0,0) ; sSurfCurve.Draw(); sm_GraphicsLoop() ;
                    sm_GraphicsLoop() ;
                  }
#endif
            }
        }

        SM_ASSERT(sUVCurves3.GetSize() == sCurves3.GetSize());

        // sCurveLoops contains the number of curves in each loop.
        // Add the number of curves in the loop that was just processed.
        sCurveLoops.Add(lCurvesInLoop);
    }

    // Now that we have made all edges and vertices of the trimmed surface -- 
    // make the face.
    SmRegion *pNewRegion;
    SmShell *pNewShell;
    SmTArray<SmPoint3d> sLoopPoints;
  
    // the following code does not do automatic face splitting.
    SmFace *pNewFace;
    SER(pExistingBrep->MakeFaceWithCurves(pRegion,                             // in : region to contain new topology objects
                                          sCurveLoops,                         // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
                                          NULL,                                // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
                                          &sUVCurves3,                         // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
                                          sCurveOrients,                       // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
                                          sLoopPoints,                         // in : Point positions to build SmVertex VertexLoops
                                          pNewSurface,                         // in : new face->Surface
                                          pNewSurface->GetNaturalUVDomain(),   // in : domain of Surface used by face
                                          SM_OT_SAME,                          // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
                                          pNewRegion,                          // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
                                          pNewShell,                           // out: New shell if any.  Trimmed surfaces always create a new shell.
                                          pNewFace));                          // out: the new face
    rpNewFace = pNewFace;

// The following code will automatically split C0 surfaces
//    SmTArray<SmFace*> sFaces;
//    SER(pExistingBrep->MakeFacesWithCurves(pRegion,
//        sCurveLoops,NULL,&sUVCurves3,sCurveOrients,
//        sLoopPoints,pNewSurface,pNewSurface->GetNaturalUVDomain(),
//        SM_OT_SAME,pNewRegion,pNewShell,sFaces));
//    rpNewFace = sFaces[0];


// The following code will use both 2D and 3D curves which come in from other systems.
// Note however that this is dangerous and the curves have lots of constraints if they
// are to work properly within TSNLib.
//    sClean3d.Clear();
//    SER(pExistingBrep->MakeFaceWithCurves(pRegion,
//        sCurveLoops,&sCurves3,&sUVCurves3,sCurveOrients,
//        sLoopPoints,pNewSurface,pNewSurface->GetNaturalUVDomain(),
//        SM_OT_SAME,pNewRegion,pNewShell,pNewFace));

    sCleanUV.Clear();
    return SM_SUCCESS;

} // end SmCurveBoundedSurface::CreateTrimmedSurfaceInBrep

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmCurveBoundedSurface.

NOTES:  Does not add in attribute memory
***********************************************************************/
ULONG SmCurveBoundedSurface::GetMemoryUsed   // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,             // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)                     // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + surface memory
  ULONG ii, lThisAllocated = 0 ;
  ULONG lUsed        = sizeof(*this) + m_pBasisSurface->GetMemoryUsed(lThisAllocated, eMarkType) ;
  rlMemoryAllocated  = sizeof(*this) + lThisAllocated ;

  // m_v3DBoundaries
  lUsed             += m_v3DBoundaries.GetMemoryUsed(lThisAllocated) ;
  rlMemoryAllocated += lThisAllocated ;

  // for every m_v3DBoundaries curve
  for(ii=0;ii<m_v3DBoundaries.GetSize();ii++)
    {
      SmCurve *pCurve = m_v3DBoundaries[ii] ;

      lUsed             += pCurve->GetMemoryUsed(lThisAllocated, eMarkType) ;
      rlMemoryAllocated += lThisAllocated ;
    }

  // m_vUVBoundaries
  lUsed             += m_vUVBoundaries.GetMemoryUsed(lThisAllocated) ;
  rlMemoryAllocated += lThisAllocated ;

  // for every m_vUVBoundaries curve
  for(ii=0;ii<m_vUVBoundaries.GetSize();ii++)
    {
      SmCurve *pCurve = m_vUVBoundaries[ii] ;

      lUsed             += pCurve->GetMemoryUsed(lThisAllocated, eMarkType) ;
      rlMemoryAllocated += lThisAllocated ;
    }

  // + attribute memory
  lUsed  += this->GetAttributeMemoryUsed(lThisAllocated, 
                                         eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }

  // all done
  return(lUsed) ;

} // end SmCurveBoundedSurface::GetMemoryUsed

/*******************************************************************//**
PURPOSE:

NOTES: Draw surface and crossHatch lines, or NumBetweenU == NumBetweenV == 999: Draw as 4 corners connected by lines
***********************************************************************/
SmDisplayList * SmCurveBoundedSurface::DrawUV
(
  ULONG              lNumBetweenU,                 ///< [in] : number of U IsoParameter lines between knots       
  ULONG              lNumBetweenV,                 ///< [in] : number of V IsoParameter lines between knots                            
  SmBoolean          bVaryCrossHatchColor,         ///< [in] : TRUE = Draw U Lines in ObjectColor                 
                                                   ///<      :        Draw V lines in m_VaryCrossHatchColor       
                                                   ///<      : FALSE= Draw both U and V Lines in ObjectColor      
  const SmExtent2d * pOptUVDomain,                 ///< [in] : UVDomain to crossHatch, NULL=use NaturalUVDomain   
  SmBoolean          bAddToUIPickList,             ///< [in] : TRUE=Add to UI pick list, FALSE=don't         
  SmGfxArraySet    * pOptGfxSet                    ///< [in,out]: When given output GfxVertexArrays not GL calls. 
) const
{  
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
    smgfx_Open(smgfx_GetRuleColor(this));
    smgfx_OutputLineWidth(smgfx_GetLineWidth()) ;

    for (ULONG i=0; i<m_v3DBoundaries.GetSize(); i++) 
      {
        SmCompositeCurve *pCC = SM_REINTERPRET_CAST(SmCompositeCurve*,m_v3DBoundaries[i]);
        pCC->Draw();
      }
    ((SmBSplineSurface*)m_pBasisSurface)->DrawUV( lNumBetweenU, lNumBetweenV,bVaryCrossHatchColor,pOptUVDomain, bAddToUIPickList, pOptGfxSet ) ;

  pRtn = smgfx_Close() ;

#else
  SM_REF6(lNumBetweenU, lNumBetweenV, bVaryCrossHatchColor, pOptUVDomain, bAddToUIPickList, pOptGfxSet);
#endif
  return(pRtn) ;

} // end SmCurveBoundedSurface::DrawUV

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertCurveBoundedSurface_list[] =
{
  /*  0 */ {SM_AT_NESTED_TEST, _T("UNKNOWN"), _T("Contained BasisSurface must pass its AssertValid tests") },
  /*  1 */ {SM_AT_NESTED_TEST, _T("UNKNOWN"), _T("Contained 3DBoundaries curve must pass its AssertValid tests") },
  /*  2 */ {SM_AT_NESTED_TEST, _T("UNKNOWN"), _T("Contained UVBoundaries curve must pass its AssertValid tests") },
  /*  3 */ {SM_AT_POINTER,     _T("Base Surface Context"), _T("Basis surface context must be this surface context") },
  /*  4 */ {SM_AT_POINTER,     _T("3D Curves Context"),    _T("3D Curves context must be this surface context") },
  /*  5 */ {SM_AT_POINTER,     _T("UV Curves Context"),    _T("UV Curves context must be this surface context") }

} ;

/*******************************************************************//**
PURPOSE: Make sure STEP and Nurb parameterizations are equivalent.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmCurveBoundedSurface::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  // run parent AssertValid
  SmBoolean bRtn = TRUE ;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmSurface::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // assert valid on the contained surface 
  SmBoolean bBasisOK = m_pBasisSurface->AssertValid(pAList, eTestLevel, eWalkTree) ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, bBasisOK, _T("") ) ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, m_pBasisSurface->GetContext() == GetContext(), _T("") ) ;

  // assert valid on the contained 3d curves
  ULONG ii ;
  for(ii=0;ii<m_v3DBoundaries.GetSize();ii++)
    { 
      SmBoolean b3dCurveOK = m_v3DBoundaries[ii]->AssertValid(pAList, eTestLevel, eWalkTree) ;
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, b3dCurveOK, _T("") ) ;
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, m_v3DBoundaries[ii]->GetContext() == GetContext(), _T(""));
    }

  // assert valid on the contained uv curves 
  for(ii=0;ii<m_vUVBoundaries.GetSize();ii++)
    { 
      SmBoolean bUVCurveOK = m_vUVBoundaries[ii]->AssertValid(pAList, eTestLevel, eWalkTree) ;
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, bUVCurveOK, _T("") ) ;
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, m_vUVBoundaries[ii]->GetContext() == GetContext(), _T(""));
    }

  // test that might be built if needed
  // 1. check that all UV-Curves lie within the surface domain
  // 2. check gaps from uvcurves to 3d curves
  // 3. check that surface is OK

  // all done
#ifdef SM_DEBUG_CODE
  if(!bRtn)
    { 
      // SM_ASSERT(bRtn);
    }
#endif // SM_DEBUG_CODE

  return(bRtn) ;

} // end SmCurveBoundedSurface::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmCurveBoundedSurface::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport 
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return ( SmSurface::AssertHeal(rAReport, pAList) ) ;
//     }
//    
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ; 
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//                rAReport.m_pHealMessage = _T("SmCurveBoundedSurface::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmCurveBoundedSurface::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmCurveBoundedSurface to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmCurveBoundedSurface::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type
  SmFileType      eType    = rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();
      
  if (eType == SM_ASCII) 
    {
      rFileOut << m_v3DBoundaries.GetSize() << " SmCurveBoundedSurface Number of 3D Boundaries \n";
      rFileOut << m_vUVBoundaries.GetSize() << " SmCurveBoundedSurface Number of UV Boundaries \n";
      rFileOut << m_bImplicitOuter          << " SmCurveBoundedSurface Implicit Outer Flag \n";
    }
  else 
    {
      SER(rDB.WriteLong(m_v3DBoundaries.GetSize()));
      SER(rDB.WriteLong(m_vUVBoundaries.GetSize()));
      SER(rDB.WriteBoolean(m_bImplicitOuter));
    }

  // basis surface
  if (eType == SM_ASCII) { rFileOut << " SmCurveBoundedSurface->BasisSurface \n"; }
  SER(rDB.WriteType(m_pBasisSurface->GetType())) ;
  SER(m_pBasisSurface->WriteToDB(rDB, lDBVersionNumber)) ;

  // every 3DBoundary curve
  ULONG ii ; 
  for(ii=0;ii<m_v3DBoundaries.GetSize();ii++)
    {
      ULONG l3DDim = m_v3DBoundaries[ii]->GetDim() ;
      SER(rDB.WriteType(m_v3DBoundaries[ii]->GetType(), &l3DDim)) ;
      SER(m_v3DBoundaries[ii]->WriteToDB(rDB, lDBVersionNumber)) ;
    }

  // every UVBoundary curve
  for(ii=0;ii<m_vUVBoundaries.GetSize();ii++)
    {
      ULONG lUVDim = m_vUVBoundaries[ii]->GetDim() ;
      SER(rDB.WriteType(m_vUVBoundaries[ii]->GetType(), &lUVDim)) ;
      SER(m_vUVBoundaries[ii]->WriteToDB(rDB, lDBVersionNumber)) ;
    }

  // all done
  return SM_SUCCESS ;

} // end SmCurveBoundedSurface::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmCurveBoundedSurface from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmCurveBoundedSurface::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  const SmContext & crContext,          // in : context for new object construction
  SmSurface      *& rpNewSurface,       // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF1(lType) ;
  rpNewSurface = NULL ; 

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  ULONG     l3DBoundariesSize ; 
  ULONG     lUVBoundariesSize ; 
  SmBoolean bImplicitOuter;   

  if (eType == SM_ASCII) 
    {
      rFileIn >> l3DBoundariesSize ;  rDB.GoToNextLine() ;
      rFileIn >> lUVBoundariesSize ;  rDB.GoToNextLine() ;
      rFileIn >> bImplicitOuter ;     rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadLong(l3DBoundariesSize)) ;
      SER(rDB.ReadLong(lUVBoundariesSize)) ;
      SER(rDB.ReadBoolean(bImplicitOuter)) ;
    }

  // surface locals
  SmSurface *pBasisSurface = NULL ;
  SmTArray<SmCurve*> s3DCurves, sUVCurves ;
  SmCurve   *p3DCurve = NULL, *pUVCurve = NULL ;
  SM_TYPE    lBasisType, l3DType = 0, lUVType ;  
  ULONG      ii, l3DDim, lUVDim ;

  // basis surface
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lBasisType)) ; 
  SER(SmSurface::ReadFromDB(lBasisType, rDB, crContext, pBasisSurface, lDBVersionNumber)) ; 

  // for every 3d curve
  for(ii=0;ii<l3DBoundariesSize;ii++)
    {
      SER(rDB.ReadType(l3DType, &l3DDim)) ;
      SER(SmCurve::ReadFromDB(l3DType, rDB, l3DDim, crContext, p3DCurve, lDBVersionNumber)) ; 

      s3DCurves.Add(p3DCurve) ; 
      p3DCurve = NULL ;
    }
 
  // for every UV curve
  for(ii=0;ii<lUVBoundariesSize;ii++)
    {
      SER(rDB.ReadType(lUVType, &lUVDim)) ;
      SER(SmCurve::ReadFromDB(l3DType, rDB, lUVDim, crContext, pUVCurve, lDBVersionNumber)) ; 

      sUVCurves.Add(pUVCurve) ;
      pUVCurve = NULL ; 
    }
 
  // all done
  rpNewSurface =  new (crContext) SmCurveBoundedSurface(*pBasisSurface, &s3DCurves, &sUVCurves) ;
  return SM_SUCCESS;

} // end SmCurveBoundedSurface::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCurveBoundedSurface::IsKindOf( SM_TYPE t ) const
{
  return ((SmCurveBoundedSurface_TYPE == t) ? TRUE : SmSurface::IsKindOf( (t) ));
}


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCurveBoundedSurface::Dump
  (void) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmCurveBoundedSurface::Dump()")) ;

  // report Cache data
  SmSurface::Dump(FALSE) ;

  SM_SPRINTF(sBuff,       
             _T("SmCurveBoundedSurface 0x%p,  Surface=0x%p,  Implicit Outer=%d\n"),
             this,
             this->m_pBasisSurface,
             this->m_bImplicitOuter);
  SM_SPRINTF(sBuffForFile,
             _T("SmCurveBoundedSurface %s,  Surface=%s,  Implicit Outer=%d\n"),
             _T("notNULL"),
             this->m_pBasisSurface ? _T("notNULL") : _T("NULL"),
             this->m_bImplicitOuter);
  smos_WriteBuffer(sBuff, sBuffForFile);

  for (ULONG i=0; i<m_v3DBoundaries.GetSize(); i++) 
    {
      SM_SPRINTF(sBuff,       
                 _T("     Boundary[%ld]=0x%p\n"),
                 i,
                 m_v3DBoundaries[i]);
      SM_SPRINTF(sBuffForFile,
                 _T("     Boundary[%ld]=%s\n"),
                 i,
                 m_v3DBoundaries[i] ? _T("notNULL") : _T("NULL"));
      smos_WriteBuffer(sBuff, sBuffForFile);

      SmCompositeCurve *pBoundaryCurve = SM_REINTERPRET_CAST(SmCompositeCurve*,m_v3DBoundaries[i]);
      pBoundaryCurve->Dump();
    }

  smos_WriteBuffer(_T(" End SmCurveBoundedSurface::Dump()\n")) ;

} // end SmCurveBoundedSurface::Dump
