// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmBSplineSurface.cpp
* PURPOSE: Implementation of SmBSplineSurface methods.
**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Spiral.h>      /* Spiral functions */

#ifndef __Sm_Nurbs_H__
#include <SmNurbs.h>
#endif

#include <SmBSplineSurface.h>
#include <SmPolarBox.h>
#include <SmNurbsSrf.h>
#include <SmPlane.h>
#include <SmCone.h>
#include <SmCylinder.h>
#include <SmSphere.h>
#include <SmSurfOfExtrusion.h>
#include <SmGeomUtility.h>
#include <SmIsoCurve.h>
#include <SmCubicBezierSurface.h>
#include <SmCircle.h>
#include <SmLine.h>
#include <SmEdge.h>
#include <SmGraphicsOutput.h>
#include <SmAssertArray.h>
#include <SmDatabaseIO.h>
#include <SmPseudoBox.h>
#include <SmPointSet.h>


#ifdef SM_DEBUG_CODE
#include <SmFace.h>  // for debug code draw blocks
#include <SmBrep.h>  // for debug code draw blocks
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Private constructor which should only be used by experts - it needs to be
            followed by a Set method of some flavor before being useful

NOTES: 
***********************************************************************/
SmBSplineSurface::SmBSplineSurface()
  : m_pNurb(NULL),
    m_bNurbIsBorrowed(FALSE),
    m_bOutOfBoundsEnabled(FALSE),
    m_eKnotType(SM_KT_UNSPECIFIED),
    m_eBSplineSurfaceForm(SM_SF_UNSPECIFIED) 
{ }

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmBSplineSurface::SmBSplineSurface
  (ULONG        n,           // in : high index of Pw (U)
   ULONG        m,           // in : high index of Pw (V)
   short        p,           // in : degree in u
   short        q,           // in : degree in v
   ULONG        r,           // in : high index of U
   ULONG        s,           // in : high index of V
   gw_CPOINT ** Pw,          // in : Control polygon
   double     * U,           // in : knots in u
   double     * V)           // in : knots in v
   // SmBoolean    bAssertHeal) // NotUsed: in : TRUE = run AssertHeal sequence on new BSplineSurfaces
   //                           //      default:[TRUE]
:  m_pNurb(NULL),
   m_bNurbIsBorrowed(FALSE),
   m_bOutOfBoundsEnabled(FALSE),
   m_eKnotType(SM_KT_UNSPECIFIED),
   m_eBSplineSurfaceForm(SM_SF_UNSPECIFIED) 
{
  // allocate single block Surface with internal pointers and size params set
  gw_SURFACE *pNewSur = sm_AllocateNurbSurface(n, m, p, q, r, s);

  // copy knot values from target surface to new surface.
  // okay to use smos_MemCpy on base (double) objects.
  SE(smos_MemCpy(pNewSur->knu->U, U, sizeof(gw_REAL) * ((size_t)r + (size_t)1), sizeof(gw_REAL) * ((size_t)r + (size_t)1)));
  SE(smos_MemCpy(pNewSur->knv->U, V, sizeof(gw_REAL) * ((size_t)s + (size_t)1), sizeof(gw_REAL) * ((size_t)r + (size_t)1)));

  // copy control point values from target surface to new surface one row at a time
  for (ULONG i=0; i<=n; i++) 
    {
      // okay to use smos_MemCpy on static class (gw_CPOINT) objects.
      SE(smos_MemCpy(pNewSur->net->Pw[i], Pw[i],  sizeof(gw_CPOINT) * ((size_t)m + (size_t)1),sizeof(gw_CPOINT) * ((size_t)m + (size_t)1) ));
    }

  m_pNurb = pNewSur;

  // when built call sm_FixupKnotVector() and when asked call Healer.
  if (m_pNurb) 
    {
// obsolete
//       // AssertValid() and AssertHeal() calls are always compiled to debug by hand.           
//       //   Making the calls are controlled by conditional compiles:
//       // with    SM_USE_AUTOHEAL with    SM_DEBUG_CODE and - run AssertValid and AssertHeal always 
//       // with    SM_USE_AUTOHEAL without SM_DUBUG_CODE and - run AssertValid and AssertHeal when bAssertHeal == TRUE
//       // without SM_USE_AUTOHEAL                           - run AssertValid and AssertHeal never
//
//       // gwc: for now - don't run AssertHeal on infinite domains - that needs work.
//       //      The answer may be to change SMLib to only build BSplines for Finite geometry.
//       // TRUE = ranges of both knot vectors are finite
//       
// #if   defined(SM_USE_AUTOHEAL) &&  defined(SM_DEBUG_CODE)
//       SmBoolean bIsBounded = GetNaturalUVDomain().IsBounded();
//       SmBoolean bUseAutoHeal    = bIsBounded && TRUE ;
// #elif defined(SM_USE_AUTOHEAL) && !defined(SM_DEBUG_CODE)
//       SmBoolean bIsBounded = GetNaturalUVDomain().IsBounded();
//       SmBoolean bUseAutoHeal    = bIsBounded && bAssertHeal ;
// #else // with SM_USE_AUTOHEAL
//       SmBoolean bUseAutoHeal    = FALSE ;
// #endif // no SM_USE_AUTOHEAL
//       // check this SmBSplineSurface for problems and fix the ones we can
//       if(m_pNurb && bUseAutoHeal)
//         {
//           SmAssertArray sAList ;
//           this->AssertValid(&sAList, SM_LEVEL_0, SM_NO_WALK) ; 
//           sAList.Heal() ; 
//          } // end bAssertHeal check
// end obsolete

      // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
      //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
      gw_KNOTVECTOR *pKnu = m_pNurb->knu;
      gw_KNOTVECTOR *pKnv = m_pNurb->knv;

      sm_FixupKnotVector(pKnu,SM_EFF_ZERO_PARAM);
      sm_FixupKnotVector(pKnv,SM_EFF_ZERO_PARAM); 
    }       

} // end SmBSplineSurface::SmBSplineSurface constructor from NLib curve data

/*******************************************************************//**
PURPOSE: Private constructor for fast performance when already have allocated nurb

NOTES: 
***********************************************************************/
SmBSplineSurface::SmBSplineSurface
  (gw_SURFACE      * pAlreadyAllocatedNurb,   // in : Pointer to allocated NURB
   SmBoolean         bNurbIsBorrowed,         // in : TRUE = don't delete pNurb when destructed
                                              //      good for temporary Surfaces built for persistent gw_SURFACE objects
   const SmContext * pContext)                // in : Use this context when given
  // SmBoolean         bAssertHeal)             // NotUsed: in : TRUE = run AssertHeal sequence on new BSplineSurfaces
                                              //      default:[TRUE]
 : m_pNurb              (pAlreadyAllocatedNurb),
   m_bNurbIsBorrowed    (bNurbIsBorrowed),
   m_bOutOfBoundsEnabled(FALSE),
   m_eKnotType          (SM_KT_UNSPECIFIED),
   m_eBSplineSurfaceForm(SM_SF_UNSPECIFIED) 
{ 
  SM_ASSERT(pAlreadyAllocatedNurb != NULL);

  if(pContext) 
    { m_cpContext = pContext ; }

  // when owned call sm_FixupKnotVector() and when asked call Healer.
  if(!m_bNurbIsBorrowed) 
    {
// obsolete
//       // AssertValid() and AssertHeal() calls are always compiled to debug by hand.           
//       //   Making the calls are controlled by conditional compiles:
//       // with    SM_USE_AUTOHEAL with    SM_DEBUG_CODE and - run AssertValid and AssertHeal always 
//       // with    SM_USE_AUTOHEAL without SM_DUBUG_CODE and - run AssertValid and AssertHeal when bAssertHeal == TRUE
//       // without SM_USE_AUTOHEAL                           - run AssertValid and AssertHeal never
// 
//       // gwc: for now - don't run AssertHeal on infinite domains - that needs work.
//       //      The answer may be to change SMLib to only build BSplines for Finite geometry.
//       // TRUE = ranges of both knot vectors are finite
//        
// #if   defined(SM_USE_AUTOHEAL) &&  defined(SM_DEBUG_CODE)
//       SmBoolean bIsBounded = GetNaturalUVDomain().IsBounded();
//       SmBoolean bUseAutoHeal    = bIsBounded && TRUE ;
// #elif defined(SM_USE_AUTOHEAL) && !defined(SM_DEBUG_CODE)
//       SmBoolean bIsBounded = GetNaturalUVDomain().IsBounded();
//       SmBoolean bUseAutoHeal    = bIsBounded && bAssertHeal ;
// #else // with SM_USE_AUTOHEAL
//       SmBoolean bUseAutoHeal    = FALSE ;
// #endif // no SM_USE_AUTOHEAL
//       // check this SmBSplineSurface for problems and fix the ones we can
//       if(m_pNurb && bUseAutoHeal)
//         {
//           SmAssertArray sAList ;
//           this->AssertValid(&sAList, SM_LEVEL_0, SM_NO_WALK) ; 
//           sAList.Heal() ; 
//          } // end bAssertHeal check
// end obsolete

      if (m_pNurb) 
        {
          // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
          //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
          gw_KNOTVECTOR *pKnu = m_pNurb->knu;
          gw_KNOTVECTOR *pKnv = m_pNurb->knv;

          sm_FixupKnotVector(pKnu,SM_EFF_ZERO_PARAM);
          sm_FixupKnotVector(pKnv,SM_EFF_ZERO_PARAM); 
        }
    }       
 
} // end SmBSplineSurface::SmBSplineSurface constructor

/*******************************************************************//**
PURPOSE:  Construct a B-Spline Surface given a pointer to an NLib
   NURB Surface and the corresponding dimension.  The NLib Surface will
   not be consumed but will be copied.

NOTES: The resulting Surface will be projected to the XY plane or 
   the X line if the dimension of the input Surface is not the same as the 
   dimension asked for.
***********************************************************************/
SmBSplineSurface::SmBSplineSurface
  (const gw_SURFACE * cpGwNurb)    // in : NLib BSplineSurface representation - copied into m_pNurb
   // SmBoolean          bAssertHeal) // NotUsed: in : TRUE = run AssertHeal sequence on new BSplineSurfaces
   //                                 //      default:[TRUE]
  : m_pNurb(NULL),
    m_bNurbIsBorrowed(FALSE),
    m_bOutOfBoundsEnabled(FALSE),
    m_eKnotType(SM_KT_UNSPECIFIED),
    m_eBSplineSurfaceForm(SM_SF_UNSPECIFIED) 
{ 
  SM_ASSERT(cpGwNurb != NULL);

  // This routine allocates a single piece of memory to
  // contain the nurb Surface.  The following order is used
  // to map the memory to the Surface structure:
  //    gw_SURFACE
  //    gw_CNET
  //    KNOTVECTORU
  //    KNOTVECTORV
  //    <array of double for knots U>
  //    <array of double for knots V>
  //    <array of pointers to gw_CPOINT*>
  //    <array of double*4 for CPOINTS>
  const gw_SURFACE *pSrcSur = (gw_SURFACE*)cpGwNurb;
  SM_ASSERT(m_pNurb == NULL) ;
  m_pNurb = sm_AllocateAndCopyNurbSurface(pSrcSur);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(10,10,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // when built call sm_FixupKnotVector() and when asked call Healer.
  if (m_pNurb) 
    {
// obsolete
//       // AssertValid() and AssertHeal() calls are always compiled to debug by hand.           
//       //   Making the calls are controlled by conditional compiles:
//       // with    SM_USE_AUTOHEAL with    SM_DEBUG_CODE and - run AssertValid and AssertHeal always 
//       // with    SM_USE_AUTOHEAL without SM_DUBUG_CODE and - run AssertValid and AssertHeal when bAssertHeal == TRUE
//       // without SM_USE_AUTOHEAL                           - run AssertValid and AssertHeal never
// 
//       // gwc: for now - don't run AssertHeal on infinite domains - that needs work.
//       //      The answer may be to change SMLib to only build BSplines for Finite geometry.
//       // TRUE = ranges of both knot vectors are finite
// 
// #if   defined(SM_USE_AUTOHEAL) &&  defined(SM_DEBUG_CODE)
//       SmBoolean bIsBounded = GetNaturalUVDomain().IsBounded();
//       SmBoolean bUseAutoHeal    = bIsBounded && TRUE ;
// #elif defined(SM_USE_AUTOHEAL) && !defined(SM_DEBUG_CODE)
//       SmBoolean bIsBounded = GetNaturalUVDomain().IsBounded();
//       SmBoolean bUseAutoHeal    = bIsBounded && bAssertHeal ;
// #else // with SM_USE_AUTOHEAL
//       SmBoolean bUseAutoHeal    = FALSE ;
// #endif // no SM_USE_AUTOHEAL
//       // check this SmBSplineSurface for problems and fix the ones we can
//       if(m_pNurb && bUseAutoHeal)
//         {
//           SmAssertArray sAList ;
//           this->AssertValid(&sAList, SM_LEVEL_0, SM_NO_WALK) ; 
//           sAList.Heal() ; 
//         } // end bAssertHeal check 
// end obsolete

      // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
      //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
      gw_KNOTVECTOR *pKnu = m_pNurb->knu;
      gw_KNOTVECTOR *pKnv = m_pNurb->knv;

      sm_FixupKnotVector(pKnu,SM_EFF_ZERO_PARAM);
      sm_FixupKnotVector(pKnv,SM_EFF_ZERO_PARAM); 
    }       
} // end SmBSplineSurface::SmBSplineSurface constructor

/*******************************************************************//**
PURPOSE: Copy constructor for a B-Spline Surface.

NOTES:      
    This constructor should only be used in the case of a new allocated
    surface or one which is being declared on the stack.  It should not
    be used to assign to a preexisting surface.

***********************************************************************/
SmBSplineSurface::SmBSplineSurface
  (const SmBSplineSurface & crSourceSurface) 
 : SmSurface(crSourceSurface),
   m_pNurb(NULL),
   m_bNurbIsBorrowed(FALSE), 
   m_bOutOfBoundsEnabled(crSourceSurface.m_bOutOfBoundsEnabled), 
   m_eKnotType(crSourceSurface.m_eKnotType),
   m_eBSplineSurfaceForm(crSourceSurface.m_eBSplineSurfaceForm) 
{
  // free any existing NurbSurface structure
  if (m_pNurb && !m_bNurbIsBorrowed) 
    {
      smos_Free(m_pNurb);
      m_pNurb = NULL;
    }

  // fetch the source NurbSurface 
  const gw_SURFACE *pSrcSur = ((SmBSplineSurface &)crSourceSurface).GetOrCreateGwNurbPointer() ;

  // error - can't fetch m_pNurb
  if (!pSrcSur) 
    { SE(SM_ERR); }

  // copy and store m_pNurb
  SM_ASSERT(m_pNurb == NULL) ;
  m_pNurb = sm_AllocateAndCopyNurbSurface(pSrcSur);

  if (m_pNurb) 
    {

      // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
      //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
      gw_KNOTVECTOR *pKnu = m_pNurb->knu;
      gw_KNOTVECTOR *pKnv = m_pNurb->knv;

      sm_FixupKnotVector(pKnu,SM_EFF_ZERO_PARAM);
      sm_FixupKnotVector(pKnv,SM_EFF_ZERO_PARAM); 
    }       

} // end SmBSplineSurface::SmBSplineSurface constructor

/*******************************************************************//**
PURPOSE: Default destructor for B-Spline Surfaces.

NOTES: 
***********************************************************************/
SmBSplineSurface::~SmBSplineSurface()
{
  if(!m_bNurbIsBorrowed)
    {
      if (m_pNurb) { smos_Free(m_pNurb) ; m_pNurb = NULL ; }
    }

} // end SmBSplineSurface::~SmBSplineSurface destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmBSplineSurface

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmBSplineSurface::operator==
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
      SmBSplineSurface &rOther = (SmBSplineSurface &)crOther ;

      // check equivalence of these objects
      bRtn =    ( m_pNurb == rOther.m_pNurb)
             || ( m_pNurb == NULL && rOther.m_pNurb == NULL)
             || (   m_pNurb != NULL && rOther.m_pNurb != NULL
                 && N_SrfsAreEqual(m_pNurb, rOther.GetGwNurbPointer(), SM_EFF_ZERO, SM_EFF_ZERO )) ;
    }

  // all done
  return bRtn ;

} // end SmBSplineSurface::operator==

/*******************************************************************//**
PURPOSE: Given a domain on the Surface, compute the axis alligned
    rectilinear (normalBox), and/or non-axis alligned parallelpiped (pseudoBox),
    and/or polar (PolarBox of Surface normal vectors) bounding boxes.

NOTES:
    Most Bounding boxes are built directly from the surface control-net.

    Bounding boxes built from the control-net are always built for the entire surface
      ignoring the input crUVDomain value.  This may not be optimal but it is conservative
      always returning a valid bounding box which may be bigger than we want.
      If you need the bounding box of a subdomain - copy and trim the original
      surface and get that copied surface's bounding boxes.

    PolarBoxes of analytic surfaces are built from sample values.
      These sample values are limited to the input crUVDomain so
      the polarBox of an Analytic Surface is the only bounding
      box which takes advantage of the given crUVDomain to reduce
      its size.

    PolarBox Problem: The PolarBox computation is based on a heuristic
      and may be slightly smaller than desired.  When building sub-division
      trees - take the time to union the children polar boxes together to
      build the parent boxes to remove tolerance build up.  Otherwise use
      the PolarBoxes as given.
***********************************************************************/
SmStatus SmBSplineSurface::CalculateBoundingBox
  (const SmExtent2d  & crUVDomain,           // in : only whole surfaces are bounded (except for polarBoxes of analytic surfaces)
   SmExtent3d        * pNormalBox,           // out: Axis alligned box
   SmPseudoBox       * pPseudoBox,           // out: Non-axis aligned box
   SmPolarBox        * pPolarBox,            // out: Surface normal vector field bounding box
   const SmPseudoBox * pOptPseudoBasisGuess, // in : guess for PseudoBox basis vectors - used unless another better orientation is found
                                             //      NULL to ignore, default:[NULL]
   const SmPolarBox  * pOptPolarBasisGuess,          // in : guess for PolarBox basis vectors - used unless another better orientation is found
                                             //      NULL to ignore, default:[NULL]
   SmBoolean           bExpandPosBoxesByZoneTol3d)   // in : TRUE = returned Normal & Pseudo BBoxes = BBox->ExpandAbsoluate(ZoneTol3d)
                                                     //    : FALSE= returned Normal & Pseudo BBoxes = BBox with no expansion 
                                                     //      default:[TRUE] = previous behavior
  const
{
  SM_ASSERT(   pNormalBox != NULL
            || pPseudoBox != NULL
            || pPolarBox  != NULL);

  // init output
  if(pNormalBox) { pNormalBox->Init() ; }
  if(pPseudoBox) { pPseudoBox->Init() ; } // init intervals - leave basis vectors alone
  if(pPolarBox)  { pPolarBox->ReSet() ; }

  if (   pOptPolarBasisGuess != NULL
      && pOptPolarBasisGuess->GetPolarBoxState() == SM_PS_UNINITIALIZED )
    { pOptPolarBasisGuess = NULL; }

  // locals
  SmPseudoBox       sTrialPseudoBox ;
  SmPolarBox        sTrialPolarBox ;
  const SmVector3d *pGuessBasis = NULL ;
  SmVector3d sCross ;

  // If the interval is equal to the natural interval then we don't need
  // to chop it up and use a sub segment.
  SmExtent2d sNaturalUVDomain = GetNaturalUVDomain();

  ULONG lUDegree = GetDegree(SM_SP_U);
  ULONG lVDegree = GetDegree(SM_SP_V);
  ULONG lUKnots  = GetNumberNaturalKnots(SM_SP_U);
  ULONG lVKnots  = GetNumberNaturalKnots(SM_SP_V);

  // check for analytic/non-analytic splines
  // note: this test may be over-ridden ...see below
  SmBoolean bPlanar   =   lUDegree == 1 && lUKnots <= 4
                       && lVDegree == 1 && lVKnots <= 4;
  SmBoolean bAnalytic =   (   (lUDegree == 1 && lUKnots > 4)
                           || (lUDegree == 2 && lUKnots > 6)
                           || (lUDegree > 2)

                           || (lVDegree == 1 && lVKnots > 4)
                           || (lVDegree == 2 && lVKnots > 6)
                           || (lVDegree > 2)

                           || (lUDegree == 2 && lVDegree == 2) )
                        ? FALSE
                        : TRUE ;

   // also check the derived type of the surface
   if(   IsKindOf(SmPlane_TYPE)
      || IsKindOf(SmCone_TYPE)
      || IsKindOf(SmSphere_TYPE)
      || IsKindOf(SmTorus_TYPE))
    { bAnalytic = TRUE ; }

  // locals
  const gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
  const gw_CNET    *pNET = pSur->net;

  // Correction for poor approximation to 'plane'
  if ( bPlanar )
  {
      gw_FLAG flat = 0;
      gw_FLAG dir = 0;
      N_SrfIsFlatCheap( (gw_SURFACE *)pSur, 0.0001, &flat, &dir );
      if ( !flat )
        { bPlanar = FALSE; }
   }

  // First make sure interval sent in is valid
  // Handle special case of degenerate Surface differently - skip the interval test.
  if (!crUVDomain.IsContainedBy(sNaturalUVDomain, SM_EFF_ZERO))
    {
      SE(SM_ERR_INVALID_INPUT); // SER: a bit severe? [B306]  // changed SER to SE [B660]
    }

  // Compute the basis vectors of the PseudoBox using the control polygon
  if (pPseudoBox)
    {
      // Note that Basis 1 will be in U direction and
      // Basis 2 will be in V direction.
      gw_CPOINT *pCP00 = &pNET->Pw[0][0];
      gw_CPOINT *pCP10 = &pNET->Pw[pNET->n][0];
      gw_CPOINT *pCP01 = &pNET->Pw[0][pNET->m];
      gw_CPOINT *pCP11 = &pNET->Pw[pNET->n][pNET->m];

      // Use these three points to try to compute a basis vector set
      // If we are unable to then we need to just use the default vectors
      SmPoint3d sP00, sP10, sP01, sP11;
      TO_EUCLID(*pCP00,sP00);
      TO_EUCLID(*pCP10,sP10);
      TO_EUCLID(*pCP01,sP01);
      TO_EUCLID(*pCP11,sP11);

      SmVector3d sV1 = (sP00 - sP10) + (sP01 - sP11);
      SmVector3d sV2 = (sP00 - sP01) + (sP10 - sP11);
      double sV1LS = sV1.LengthSquared();
      // Note that we will just use the standard basis vectors if the Surface is
      // closed.  If the mid point lies on same line as the start and end point
      // then we will get two arbitrary vectors for Basis2 and Basis3 otherwise V2 will
      // determine the direction for Basis2 and orthogonal direction for Basis3
      if (sV1LS > SM_EFF_ZERO_SQ)
        {
          SmVector3d sBasis1, sBasis2, sBasis3;
          sV1.MakeUnitOrthoVectors(&sV2,sBasis1,sBasis2,sBasis3);
          *pPseudoBox = SmPseudoBox(); // Initialize it
          pPseudoBox->SetBasis(sBasis1,sBasis2,sBasis3);
        }
      else
        {
          *pPseudoBox = SmPseudoBox(); // Initialize it
        }

      // when given a guess for the basis vectors
      if(pOptPseudoBasisGuess)
        {
          // init trial PseudoBox basis vectors
          sTrialPseudoBox.Init() ; // init intervals - leave basis vectors alone
          sTrialPseudoBox.SetBasis(pOptPseudoBasisGuess->GetBasis(0),
                                   pOptPseudoBasisGuess->GetBasis(1),
                                   pOptPseudoBasisGuess->GetBasis(2)) ;
        }
    } // end need to compute PseudoBox check

  // Compute polar bounding box really fast for analytics
  if (pPolarBox && bAnalytic)
    {
      const ULONG lVCnt = 10 ;
      SmVector3d sNorm[lVCnt] ;

      // get center normal
      SER(EvaluateNormal(crUVDomain.Evaluate(0.5, 0.5),TRUE,TRUE,sNorm[9]));

      // for planes
      if(bPlanar)
        {
          pPolarBox->AddVector3d(sNorm[9]);
          if(pOptPolarBasisGuess)
            {
              sTrialPolarBox.AddVector3d(sNorm[9]);
            }
        }
      else // not a plane
        {

          // get corner normals
          SER(EvaluateNormal(crUVDomain.Evaluate(0.0,0.0),TRUE,TRUE,sNorm[5]));
          SER(EvaluateNormal(crUVDomain.Evaluate(1.0,0.0),TRUE,TRUE,sNorm[6]));
          SER(EvaluateNormal(crUVDomain.Evaluate(1.0,1.0),TRUE,TRUE,sNorm[7]));
          SER(EvaluateNormal(crUVDomain.Evaluate(0.0,1.0),TRUE,TRUE,sNorm[8]));

          // Build a central vector that can be used as a good PolarBox base vector
          if (pNET->n <= 2 && pNET->m <= 2)
            {
              sNorm[0] = (sNorm[5] + sNorm[6] + sNorm[7] + sNorm[8]) / 4.0;
            }
          else
            {
              sNorm[0] = sNorm[9] ;
            }

          // when we have a midPoint vector for the polar box
          if (sNorm[0].LengthSquared() > SM_EFF_ZERO_SQ)
            {
              // get mid side normals
              SER(EvaluateNormal(crUVDomain.Evaluate(0.0,0.5),TRUE,TRUE,sNorm[1]));
              SER(EvaluateNormal(crUVDomain.Evaluate(1.0,0.5),TRUE,TRUE,sNorm[2]));
              SER(EvaluateNormal(crUVDomain.Evaluate(0.5,0.0),TRUE,TRUE,sNorm[3]));
              SER(EvaluateNormal(crUVDomain.Evaluate(0.5,1.0),TRUE,TRUE,sNorm[4]));

              // build the polar box by adding all the vectors
              //  The 1st added vector sets the PolarBox principle basis direction - a good selection centers the box
              //  The 2nd added vector sets the 2nd direction - a good selection orients the box with the shape and minimizes box size
              pPolarBox->ReSet() ;
              for(ULONG ii=0;ii<lVCnt;ii++)
                {
                  pPolarBox->AddVector3d(sNorm[ii]);
                }

              if(pOptPolarBasisGuess)
                {
                  pGuessBasis = (  pOptPolarBasisGuess->GetDomain().XLength()
                                 > pOptPolarBasisGuess->GetDomain().YLength())
                                ? pOptPolarBasisGuess->GetBasis(2)
                                : pOptPolarBasisGuess->GetBasis(1) ;
                  sTrialPolarBox.ReSet() ;
                  for(ULONG ii=0;ii<lVCnt;ii++)
                    {
                      sTrialPolarBox.AddVector3d(sNorm[ii], pGuessBasis);
                    }
                }

            }
        } // not a plane branch
    } // end build polarBox for analytic surface check

  // Do a little work to try to get the polar box to be centered - use the
  // derivatives of the midpoints to establish the base line vector.
  if (pPolarBox && ! bAnalytic)
    {
      SmVector3d sNorm[3] ;
      SmPoint3d sPnt;

      SER(EvaluateNormal(crUVDomain.Evaluate(0.5,0.5),TRUE,TRUE,sNorm[0]));
      pPolarBox->AddVector3d(sNorm[0]);

      SER(EvaluateNormal(crUVDomain.Evaluate(1.0,0.5),TRUE,TRUE,sNorm[1]));
      pPolarBox->AddVector3d(sNorm[1]);

      SER(EvaluateNormal(crUVDomain.Evaluate(0.5,1.0),TRUE,TRUE,sNorm[2]));
      pPolarBox->AddVector3d(sNorm[2]);

      if(pOptPolarBasisGuess)
        {
          pGuessBasis = (  pOptPolarBasisGuess->GetDomain().XLength()
                         > pOptPolarBasisGuess->GetDomain().YLength())
                        ? pOptPolarBasisGuess->GetBasis(2)
                        : pOptPolarBasisGuess->GetBasis(1) ;
          sTrialPolarBox.ReSet() ;
          sTrialPolarBox.AddVector3d(sNorm[0], pGuessBasis);
          sTrialPolarBox.AddVector3d(sNorm[1], pGuessBasis);
          sTrialPolarBox.AddVector3d(sNorm[2], pGuessBasis);
        }
    }

  SmPoint3d sPoint, sNext ;
  SmPoint3d sVNext, sVLast, sDV0, sDV1 ;
  SmPoint3d sUNext, sULast, sDU0, sDU1 ;
  SmVector3d sVec ;

  // iter over the BSpline SurfaceNet ControlPoints
  if(   pNormalBox
     || pPseudoBox
     || (pPolarBox && !bAnalytic))
    {
      for (long i=0; i<=pNET->n; i++)
        {
          for (long j=0; j<=pNET->m; j++)
            {
              // get controlPoint[i][j]
              TO_EUCLID(pNET->Pw[i][j],sPoint);

              // add control points (or their differences) to requested bounding boxes
              if (pNormalBox)
                {
                  if (i==0 && j==0) *pNormalBox = SmExtent3d(sPoint);
                  else pNormalBox->AddPoint3d(sPoint);
                }
              if (pPseudoBox)
                {
                  pPseudoBox->AddPoint3d(sPoint);
                  if(pOptPseudoBasisGuess)
                    {
                      sTrialPseudoBox.AddPoint3d(sPoint) ;
                    }
                }
              if (pPolarBox && !bAnalytic)
                {
                  // get control net tangent vectors
                  if(j > 1)       { TO_EUCLID(pNET->Pw[i][j-1],sVLast);
                                    sDV0 = sPoint - sVLast ;
                                  }
                  if(j < pNET->m) { TO_EUCLID(pNET->Pw[i][j+1],sVNext);
                                    sDV1 = sVNext - sPoint ;
                                  }
                  if(i > 1)       { TO_EUCLID(pNET->Pw[i-1][j],sULast);
                                    sDU0 = sPoint - sULast ;
                                    if(j > 1)
                                      { sCross = sDU0 * sDV0 ;
                                        pPolarBox->AddVector3d(sCross) ;
                                        if(pOptPolarBasisGuess) { sTrialPolarBox.AddVector3d(sCross, pGuessBasis); }
                                      }
                                     if(j < pNET->m)
                                       { sCross = sDU0 * sDV1 ;
                                         pPolarBox->AddVector3d(sCross) ;
                                         if(pOptPolarBasisGuess) { sTrialPolarBox.AddVector3d(sCross, pGuessBasis); }
                                       }
                                  }
                  if(i < pNET->n) { TO_EUCLID(pNET->Pw[i+1][j],sUNext);
                                    sDU1 = sUNext - sPoint ;
                                    if(j > 1)
                                      { sCross = sDU1 * sDV0 ;
                                        pPolarBox->AddVector3d(sCross) ;
                                        if(pOptPolarBasisGuess) { sTrialPolarBox.AddVector3d(sCross, pGuessBasis); }
                                      }
                                     if(j < pNET->m)
                                       { sCross = sDU1 * sDV1 ;
                                         pPolarBox->AddVector3d(sCross) ;
                                         if(pOptPolarBasisGuess) { sTrialPolarBox.AddVector3d(sCross, pGuessBasis); }
                                       }
                                  }

                  //      if(j==0) { TO_EUCLID(pNET->Pw[i][j+1],sNext);
                  //                 sDV    = sNext - sPoint ;
                  //                 sVLast = sPoint;
                  //               }
                  //      else     { sDV    = sPoint-sVLast ;
                  //                 sVLast = sPoint;
                  //               }
                  //      if(i==0) { TO_EUCLID(pNET->Pw[i+1][j],sNext);
                  //                 sDU    = sNext - sPoint ;
                  //               }
                  //      else     { TO_EUCLID(pNET->Pw[i-1][j],sULast);
                  //                 sDU    = sPoint-sULast ;
                  //               }
                  //
                  //      // add normal (tangent cross products) to polar box
                  //      sCross = sDU * sDV ;
                  //      pPolarBox->AddVector3d(sCross) ;
                  //
                  //      if(pOptPolarBasisGuess)
                  //        {
                  //          sTrialPolarBox.AddVector3d(sCross, pGuessBasis);
                  //        }
                }
            } // end iter every v dir controlPoint index
        } // end iter every u dir controlPoint index

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
         if(bDebugMe)
           {
             SmPoint3d sCenterPoint ;
             EvaluatePoint(crUVDomain.Evaluate(.5,.5), sCenterPoint) ;
             smgfx_SetRotationCenter(sCenterPoint) ;

             if(pNormalBox)           pNormalBox->Dump() ;
             if(pPseudoBox)           pPseudoBox->Dump() ;
             if(pPolarBox)            pPolarBox->Dump() ;
             if(pOptPseudoBasisGuess) sTrialPseudoBox.Dump() ;
             if(pOptPolarBasisGuess)  sTrialPolarBox.Dump() ;

             smgfx_Erase() ;
             smgfx_SetLook(5,7, 1,1,0) ;  SmVector3d(0,0,0).Draw() ; sm_GraphicsLoop() ;
             smgfx_SetLook(1,2, 0,0,1) ;  DrawUV(3, 3) ; sm_GraphicsLoop() ;
             smgfx_SetLook(4,6, 0,1,1) ;  DrawControlPoints() ; sm_GraphicsLoop() ;
             smgfx_SetLook(1,2, 1,0,0) ;  if(pNormalBox)           pNormalBox->Draw() ; sm_GraphicsLoop() ;
             smgfx_SetLook(3,4, 0,1,0) ;  if(pPseudoBox)           pPseudoBox->Draw() ; sm_GraphicsLoop() ;
             smgfx_SetLook(1,2, 1,0,1) ;  if(pPolarBox)            pPolarBox->Draw(sCenterPoint) ; sm_GraphicsLoop() ;
             smgfx_SetLook(1,2, 0,.9,.4) ; if(pOptPseudoBasisGuess) sTrialPseudoBox.Draw() ; sm_GraphicsLoop() ;
             smgfx_SetLook(1,2, 1,.4,.8) ; if(pOptPolarBasisGuess)  sTrialPolarBox.Draw(sCenterPoint) ; sm_GraphicsLoop() ;
             sm_GraphicsLoop() ;

           }
#endif // SM_DEBUG_CODE

     // save the better Psuedo box
     if(   pPseudoBox
        && pOptPseudoBasisGuess
        && sTrialPseudoBox.GetVolume() < pPseudoBox->GetVolume() - SM_EFF_ZERO)
       {
         *pPseudoBox = sTrialPseudoBox ;
       }

     // save the better Polar box
     if(   pPolarBox && !bAnalytic
        && pOptPolarBasisGuess
        && sTrialPolarBox.GetPolarArea() < pPolarBox->GetPolarArea() - SM_EFF_ZERO)
       {
         *pPolarBox = sTrialPolarBox ;
       }

    } // end need to check control net check

    // When asked, expand the normal box by the zone tolerance
    if(bExpandPosBoxesByZoneTol3d && pNormalBox)
      { pNormalBox->ExpandAbsolute(  GetFace() ? (double)GetFace()->GetTolerance()
                                   : GetBrep() ? (double)GetBrep()->GetTolerance()
                                   : SM_ZONE_TOL_3D); }

    // When asked, expand the pPseudoBox box by the zone tolerance
    if(bExpandPosBoxesByZoneTol3d && pPseudoBox)
      { pPseudoBox->ExpandAbsolute(  GetFace() ? (double)GetFace()->GetTolerance()
                                   : GetBrep() ? (double)GetBrep()->GetTolerance()
                                   : SM_ZONE_TOL_3D); }

  return SM_SUCCESS;

} // end SmBSplineSurface::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Compute various bounding values for a partial control net
   of this surface.  

NOTES: not called in SMLib
***********************************************************************/
SmStatus SmBSplineSurface::CalculatePartialNetValues
  (const SmExtent2d & crUVDomain,           // in : target domain
   SmPoint3d          aCorners[2][2],       // out: aCorners[1][0] = U=1, V=0
   SmExtent3d       * pOptNormalBox,        // out: 
   SmPseudoBox      * pOptPseudoBox,        // out: 
   SmPoint2d        * pOptUVChordHeight,    // out: 
   SmPoint2d        * pOptUVAngleTolDegree) // out: 
  const
{
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
  ULONG lU0Span, lV0Span, lU1Span, lV1Span;
  SER(FindSpans(crUVDomain.GetMin(), FALSE, FALSE, lU0Span, lV0Span));
  SER(FindSpans(crUVDomain.GetMax(), TRUE, TRUE, lU1Span,lV1Span));

  double *pUChord = NULL;
  double *pVChord = NULL;
  if (pOptUVChordHeight) 
    {
      pUChord = &pOptUVChordHeight->x;
      pVChord = &pOptUVChordHeight->y;
    }
  double *pUAng = NULL;
  double *pVAng = NULL;
  if (pOptUVAngleTolDegree) 
    {
      pUAng = &pOptUVAngleTolDegree->x;
      pVAng = &pOptUVAngleTolDegree->y;
    }

  // pass the call along
  SER(sm_ComputePartialNetConstants(pSur,
                                    lU0Span,
                                    lV0Span,
                                    lU1Span,
                                    lV1Span,
                                    pOptNormalBox,
                                    pOptPseudoBox,
                                    aCorners[0][0],
                                    aCorners[1][0],
                                    aCorners[0][1],
                                    aCorners[1][1],
                                    pUChord,
                                    pVChord,
                                    pUAng,
                                    pVAng));
  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CalculatePartialNetValues

/*******************************************************************//**
PURPOSE: Copy a SmBSplineSurface.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::Copy
  (const SmContext & crContext,
   SmSurface *& rpNewSurface) 
  const
{
  rpNewSurface = new (crContext) SmBSplineSurface(*this) ;
  NER(rpNewSurface) ;
  return SM_SUCCESS ;

} // end SmBSplineSurface::Copy

/*******************************************************************//**
PURPOSE: Copy this surface as a SmBSplineSurface if possible - else
            set output to NULL.

NOTES: The default behavior is to just copy the surface.
***********************************************************************/
SmStatus SmBSplineSurface::CopyAnalyticAsNurb
  (const SmContext & crContext,     // in : context for new object construction
   SmSurface      *& rpNewSurface)  // out: newly copied SmBSplineSurface of appropriate
                                    //      analytic derived type when analytic
 const
{
  if(IsKindOf(SmBSplineSurface_TYPE))
    { 
      return(SmBSplineSurface::Copy(crContext, rpNewSurface)) ; 
    }
  else 
    { 
      rpNewSurface = NULL ;
      return(SM_SUCCESS) ; 
    }

} // end SmBSplineSurface::CopyAnalyticAsNurb

/*******************************************************************//**
PURPOSE: Copy a BSpline surface. When it is geometrically possible, copy 
  the BSplineSurface as an analytic else as an exact copy of its current type.

NOTES: when BSplineSurface's shape is appropriate, builds a return surface
       of one one of the following types:
         SmPlane,  
         SmCone,   (note: cylinders are returned as type SmCone)         
         SmSphere,
         SmTorus, - currently disabled
         SmSurfOfRevolution,
         SmSurfOfExtrusion.

       Otherwise returns a new Surface of the same derived type as this BSplineSurface.   
***********************************************************************/
SmStatus SmBSplineSurface::CopyAndAddAnalytics
  (const SmContext & crContext,     // in : context for new object construction
   SmSurface      *& rpNewSurface)  // out: newly copied SmBSplineSurface of appropriate
                                    //      analytic derived type when analytic 
 const
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(6,6,FALSE,NULL,TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // init output
  rpNewSurface = NULL;

#ifdef USE_ANALYTICS

  // When surface is not SmBSplineSurface_TYPE, it's already an analytic or an offset surface - copy it.
  if (GetType() != SmBSplineSurface_TYPE) 
    {
      SER(Copy(crContext,rpNewSurface));
    }

  // SmPlane
  else if (SmPlane::IsNurbSurfacePlane(crContext,this,(SmPlane*&)rpNewSurface)) 
    { }

  //else if (SmCylinder::IsNurbSurfaceCylinder(crContext,this,(SmCylinder*&)rpNewSurface)) 
  //  { }
  
  // SmCone
  else if (SmCone::IsNurbSurfaceCone(crContext,this,(SmCone*&)rpNewSurface)) 
    { }
  
  // SmSphere
  else if (SmSphere::IsNurbSurfaceSphere(crContext,this,(SmSphere*&)rpNewSurface)) 
    { }
//  
//  // SmTorus
//  else if (SmTorus::IsNurbSurfaceTorus(crContext,this,(SmTorus*&)rpNewSurface)) 
//  { }
  
  // SmSurfOfRevolution
  else if (SmSurfOfRevolution::IsNurbSurfaceSurfOfRevolution(crContext,this,(SmSurfOfRevolution*&)rpNewSurface)) 
    { }
  
  // SmSurfOfExtrusion
  else if (SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion(crContext,this,(SmSurfOfExtrusion*&)rpNewSurface)) 
    { }
  
  // arrive here - make sure it's a copy
  else 
    { 
      rpNewSurface = new (crContext) SmBSplineSurface(*this);
    }

#else // no USE_ANALYTICS

  // when not using ANALYTICS, just copy the input surface
  rpNewSurface = new (crContext) SmBSplineSurface(*this);

#endif // no USE_ANALYTICS

#ifdef SM_DEBUG_CODE
SmBoolean bDebugCheckSurface = FALSE ;
  if (bDebugCheckSurface)
    {
      // verify output
      SM_DUMP_AND_ASSERT_VALID(rpNewSurface) ;
    }
  if (bDebugMe) 
    {
      Dump() ;
      rpNewSurface->Dump() ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(6,6,FALSE,NULL,TRUE); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0) ; if(rpNewSurface) rpNewSurface->DrawUV(7,7,FALSE,NULL,TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return(rpNewSurface != NULL ? SM_SUCCESS : SM_ERR) ; 

} // end SmBSplineSurface::CopyAndAddAnalytics

/*******************************************************************//**
PURPOSE: Create a bilinear surface.  If all four points are planar
    then a plane is created.
                                  
NOTES: input Control Point order:  
             3rd                        4th
  ControlPoint(U0,V1)----------ControlPoint(U1,V1)
         |                           |
         |                           |
         |                           |
         |   1st                     |  2nd
  ControlPoint(U0,V0)----------ControlPoint(U1,V0)
***********************************************************************/
SmStatus SmBSplineSurface::CreateBilinearSurface
  (const SmContext   & crContext,            // in : context for new object construction
   const SmPoint3d   & crU0V0Corner,         // in : ControlPoint[0][0] of 4 corner points to interpolate
   const SmPoint3d   & crU1V0Corner,         // in : ControlPoint[1][0] of 4 corner points to interpolate
   const SmPoint3d   & crU0V1Corner,         // in : ControlPoint[0][1] of 4 corner points to interpolate
   const SmPoint3d   & crU1V1Corner,         // in : ControlPoint[1][1] of 4 corner points to interpolate
   SmBSplineSurface *& rpNewBSplineSurface)  // out: NonRational Bilinear BSplineSurface interpolating given corners
{
  // NLib call init and locals
  NL_STACKS      SC;
  gw_SURFACE     Sur;
  SmStackHandler sStackKp(&SC);
  N_SrfInitArrays(&Sur);

  NL_POINT P00, P10, P01, P11;

  COPY_XYZ(crU0V0Corner,P00);
  COPY_XYZ(crU1V0Corner,P10);
  COPY_XYZ(crU0V1Corner,P01);
  COPY_XYZ(crU1V1Corner,P11);

  // create an NLib NonRational bilinear BSpline Surface interpolating the 4 corner points 
  if(NL_YES == N_CreateSrfCornerPts(P00,P10,P01,P11,&Sur,&SC))
    { SER(SM_ERR); }
  // GW_SER(N_CreateSrfCornerPts(P00,P10,P01,P11,&Sur,&SC));

  // set output = SMLib BSpline surface
  rpNewBSplineSurface = new (crContext) SmBSplineSurface(&Sur) ;

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateBilinearSurface

/*******************************************************************//**
PURPOSE: This is the STEP Based canonical creation.  See STEP Part 42
    for details about this form.  Note that the control points are always
    in Euclidian space even for rational curves.  If you have homogeneous 
    coordinates you need to perform homogeneous division prior to calling
    this method (x = x/w, y=y/w, z=z/w w=w).

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::CreateCanonical
  (const SmContext           & crContext,              // in : context for new object construction
   ULONG                       lUDegree,               // in : U dir polynomial degree. supported range: [1 through 32]
   ULONG                       lVDegree,               // in : V dir polynomial degree. supported range: [1 through 32]
   const SmTArray<SmPoint3d> & crControlPointsList,    // in : control points,  (Euclidian coordinates)
   SmBSplineSurfaceForm        eBSplineSurfaceForm,    // in : oneof SM_SF_PLANE_SURF,   SM_SF_CYLINDRICAL_SURF,    
                                                       //            SM_SF_CONICAL_SURF, SM_SF_SPHERICAL_SURF,    
                                                       //            SM_SF_RULED_SURF,   SM_SF_SURF_OF_REVOLUTION,     
                                                       //            SM_SF_QUADRIC_SURF, SM_SF_GENERALIZED_CONE,       
                                                       //            SM_SF_UNSPECIFIED,  SM_SF_SURF_OF_LINEAR_EXTRUSION,           
                                                       //            SM_SF_POLYNOMIAL,   SM_SF_HELICAL_SWEEP               
   const SmTArray<ULONG>     & crUMultiplicities,      // in : U dir multiplicities for each associated knot
   const SmTArray<ULONG>     & crVMultiplicities,      // in : v dir multiplicities for each associated knot
   const SmTArray<double>    & crUKnots,               // in : U dir unique knot values
   const SmTArray<double>    & crVKnots,               // in : v dir unique knot values
   SmKnotType                  eKnotType,              // in : oneof SM_KT_UNIFORM_KNOTS, SM_KT_QUASI_UNIFORM_KNOTS,       
                                                       //            SM_KT_UNSPECIFIED,   SM_KT_PIECEWISE_BEZIER_KNOTS       
   const SmTArray<double>    * cpOptWeights,           // in : optional associated ControlPoint weights, NULL for nonRational 
   const SmExtent2d          * cpOptUVDomain,          // in : optional domain whose min/max values are used to overwrite
                                                       //      the input crUKnots and crVKnots first and last values.
                                                       //      NUll to ignore.
   SmBSplineSurface         *& rpNewBSplineSurface)    // out: The created BSplineSurface 
{
  // checkinput
  RANGE_ER(1,lUDegree,32);
  RANGE_ER(1,lVDegree,32);

  // If have weights make sure have right number of them
  if (cpOptWeights && cpOptWeights->GetSize() != 0) 
    { 
      if (cpOptWeights->GetSize() != crControlPointsList.GetSize()) SER(SM_ERR_INVALID_INPUT);
    }

  // Make sNewUKnots array with one entry for each multiplicity
  SmTArray<double> sNewUKnots;
  for (ULONG i=0; i<crUKnots.GetSize(); i++) 
    {
      double dKnot = crUKnots[i];
      long   lMult = crUMultiplicities[i];
      for (long k=0; k<lMult; k++) 
        {
          sNewUKnots.Add(dKnot);
        }
    }

  // Make sNewVKnots array with one entry for each multiplicity
  SmTArray<double> sNewVKnots;
  for (ULONG j=0; j<crVKnots.GetSize(); j++) 
    {
      double dKnot = crVKnots[j];
      long lMult = crVMultiplicities[j];
      for (long k=0; k<lMult; k++) 
        {
          sNewVKnots.Add(dKnot);
        }
    }

  // Get number of ControlPoints
  ULONG lNumUCPts = sNewUKnots.GetSize() - lUDegree - 1;
  ULONG lNumVCPts = sNewVKnots.GetSize() - lVDegree - 1;

  // check input - number of ControlPoints is consistent with Knot Count and degree
  if (lNumUCPts * lNumVCPts != crControlPointsList.GetSize()) SER(SM_ERR_INVALID_INPUT);

  // for the weights
  SmTArray<double*> sWeightPtrs(lNumVCPts);
  SmTArray<double>  sWeights;
  SmBoolean bNon1Weight = FALSE;

  // when weights are all 1.0 within tolerance, make BSpline non-rational. 
  if (cpOptWeights) 
    {
      for (ULONG sm=0; sm<cpOptWeights->GetSize(); sm++) 
        {
          if (smos_Fabs((*cpOptWeights)[sm] - 1.0) > SM_EFF_ZERO) 
            {
              bNon1Weight = TRUE;
            }
        }
    }

  // when BSpline is rational with given weights
  if (bNon1Weight && cpOptWeights && cpOptWeights->GetSize() != 0) 
    {
      // set up weight pointer array to look into OptWeight array
      double * daW = cpOptWeights->GetDataArray();
      for (ULONG k=0; k<lNumUCPts; k++) 
        {
          sWeightPtrs.Add(&daW[k*lNumVCPts]);
        }
    }
  else // BSpline is nonRational
    {
      // build an array of 1s
      for (ULONG k=0; k<crControlPointsList.GetSize(); k++) 
        {
          sWeights.Add(1.0);
        }

      // set up wieght pointer array to look into OptWeight array
      double *daW = sWeights.GetDataArray();
      for (ULONG kk=0; kk<lNumUCPts; kk++) 
        {
          sWeightPtrs.Add(&daW[kk*lNumVCPts]);
        }
    } // end setup of sWeightPtrs array
  
  // set up ControlPoint array pointers
  SmTArray<NL_POINT*> sCtrlPtsPtrs(lNumVCPts);
  NL_POINT *paP = SM_REINTERPRET_CAST(NL_POINT*,crControlPointsList.GetDataArray());
  for (ULONG jj=0; jj<lNumUCPts; jj++) 
    {
      sCtrlPtsPtrs.Add(&paP[jj*lNumVCPts]);
    }

  // prepare for Nlib call
  NL_STACKS SC;
  gw_SURFACE Sur;
  SmStackHandler sStackKp(&SC);
  N_SrfInitArrays(&Sur);
  
  gw_INDEX  k1 = lNumUCPts - 1;
  gw_INDEX  k2 = lNumVCPts - 1;
  gw_DEGREE m1 = (gw_DEGREE)lUDegree;
  gw_DEGREE m2 = (gw_DEGREE)lVDegree;
  gw_REAL   *s = (gw_REAL*)sNewUKnots.GetDataArray();
  gw_REAL   *t = (gw_REAL*)sNewVKnots.GetDataArray();
  gw_REAL  **w = sWeightPtrs.GetDataArray();
  NL_POINT **p = sCtrlPtsPtrs.GetDataArray();
  gw_PARAMETER u[2], v[2];

  // init u and v to domain min/max
  if (cpOptUVDomain) 
    {
      u[0] = cpOptUVDomain->GetMin().x;
      u[1] = cpOptUVDomain->GetMax().x;
      v[0] = cpOptUVDomain->GetMin().y;
      v[1] = cpOptUVDomain->GetMax().y;
      SM_ASSERT(u[0] <= sNewUKnots[m1]) ;
      SM_ASSERT(u[1] >= sNewUKnots[lNumUCPts]) ;
      SM_ASSERT(v[0] <= sNewVKnots[m2]) ;
      SM_ASSERT(v[1] >= sNewVKnots[lNumVCPts]) ;
    }
  else // get u and v boundary values from the knot arrays
    {
      u[0] = sNewUKnots[m1];
      u[1] = sNewUKnots[lNumUCPts];
      v[0] = sNewVKnots[m2];
      v[1] = sNewVKnots[lNumVCPts];
    }

  // build the Nlib BSpline object
  if(NL_YES == N_Iges128Srf(k1,k2,m1,m2,s,t,w,p,u,v,&Sur,&SC))
    { SER(SM_ERR); }
  // GW_SER(N_Iges128Srf(k1,k2,m1,m2,s,t,w,p,u,v,&Sur,&SC));

  // build NMTlib SmBSpline from Nlib Nurb
  rpNewBSplineSurface = new (crContext) SmBSplineSurface(&Sur) ;
  rpNewBSplineSurface->m_eBSplineSurfaceForm = eBSplineSurfaceForm;
  rpNewBSplineSurface->m_eKnotType = eKnotType;

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateCanonical

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
static SmStatus sm_CreateConePatch
  (const SmContext & crContext,
   const SmAxis2Placement & crReferenceFrame,
   double dBottomRadius,
   double dTopRadius,                       // If equal to dBottomRadius a cylinder is created
   double dStartAngleDeg,                   
   double dEndAngleDeg,                     
   double dHeight,                          // Perpendicular height measured along the axis of cylinder/cone
   SmNurbCircleParam eParameterization,     // SM_CO_QUADRATIC - produces a surface using a 
                                            // quadratic (degree 2) parameterization.
                                            // SM_CO_QUINTIC - produces a surface using a quintic
                                            // (degree 5) parametrization.
   SmSurfParamType eSrfParam,               // Parameter of base circle is in
                                            // either U direction or V direction.  If in V direction
                                            // the normal is inward.  U direction normal is outward.
   SmBSplineSurface *& rpNewBSplineSurface)
{
    LT_ZERO_ER(dBottomRadius);
    LT_ZERO_ER(dTopRadius);
    RANGE_ER(0.0,dStartAngleDeg,360.0);
    RANGE_ER(0.0,dEndAngleDeg,360.0);

    NL_POINT C;
    NL_POINT X, Y;
    sm_ExtractFrame(crReferenceFrame,C,X,Y);
    NL_STACKS SC;
    gw_SURFACE Sur;
    SmStackHandler sStackKp(&SC);
    N_SrfInitArrays(&Sur);
    
    gw_FLAG ctp=0;
    if (eParameterization == SM_CO_QUADRATIC) ctp = NL_QUADRATIC;  // degree 2
    else if (eParameterization == SM_CO_QUINTIC) ctp = NL_QUINTIC;  // degree 5
    else SER(SM_ERR_INVALID_INPUT);

    gw_FLAG dir = NL_VDIR;
    if (eSrfParam == SM_SP_U) dir = NL_UDIR;
    if(NL_YES == N_CreateCylCone(C,X,Y,dBottomRadius,dTopRadius,
                              dStartAngleDeg,dEndAngleDeg,
                              dHeight,ctp,dir,&Sur,&SC))
      { SER(SM_ERR); }
    // GW_SER(N_CreateCylCone(C,X,Y,dBottomRadius,dTopRadius,
    //                        dStartAngleDeg,dEndAngleDeg,
    //                        dHeight,ctp,dir,&Sur,&SC));
    
    rpNewBSplineSurface = new (crContext) SmBSplineSurface(&Sur) ;
    rpNewBSplineSurface->SetBSplineSurfaceForm(SM_SF_CONICAL_SURF);
    if (SM_ARE_SAME(dBottomRadius,dTopRadius)) {
        rpNewBSplineSurface->SetBSplineSurfaceForm(SM_SF_CYLINDRICAL_SURF);
    }

    return SM_SUCCESS;

} // end sm_CreateConePatch

/*******************************************************************//**
PURPOSE: Given the definition of an infinite cylinder and a 3D bounding
    box -- create a NURB which represents a cylinder that goes completely
    through the bounding box.  Note that the normal of the cylinder will
    point outwards to conform to STEP.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::CreateCylinderThroughBox
(
  const SmContext & crContext,                // in :
  const SmAxis2Placement & crReferenceFrame,  // in :
  double dRadius,                             // in :
  double dStartAngle,                         // in :
  double dEndAngle,                           // in :
  SmNurbCircleParam eParameterization,        // in : SM_CO_QUADRATIC - produces a surface using a quadratic (degree 2) parameterization.
                                              // in : SM_CO_QUINTIC - produces a surface using a quintic (degree 5) parametrization.
  const SmExtent3d & crBoundingBox,           // in :
  SmBSplineSurface *& rpNewBSplineSurface     // out:
)
{
    const SmPoint3d &rLineStart = crReferenceFrame.GetOriginRef();
    SmVector3d sLineVec = crReferenceFrame.GetZAxis();

    SmPoint3d sSphCenter;
    double dSphRadius = 0.0;
    crBoundingBox.ComputeSphereBound(sSphCenter,dSphRadius);

    double dParam;
    SER(smgu_LineClosestPoint(rLineStart,sLineVec,sSphCenter,dParam));
    SmPoint3d sCenterOnLine = rLineStart + dParam * sLineVec;
    SmPoint3d sNewOrigin = sCenterOnLine - dSphRadius * sLineVec;

    double dHeight = dSphRadius * 2.0; 

    SmAxis2Placement sNewRef;
    sNewRef.SetCanonical(sNewOrigin,crReferenceFrame.GetXAxisRef(),crReferenceFrame.GetYAxisRef());

    SER(sm_CreateConePatch(crContext,sNewRef,dRadius,dRadius,
        dStartAngle,dEndAngle,dHeight,eParameterization,SM_SP_U,rpNewBSplineSurface));

    return SM_SUCCESS;

} // end SmBSplineSurface::CreateCylinderThroughBox

/*******************************************************************//**
PURPOSE: Given the definition of an infinite cone and a 3D bounding
    box -- create a NURB which represents a cone that goes completely
    through the bounding box.  Note that the normal will point outward
    to conform to step.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::CreateConeThroughBox
(
  const SmContext & crContext,                // in :
  const SmAxis2Placement & crReferenceFrame,  // in :
  double dRadius,                             // in : Radius of cone corresponding to a plane through the origin of the reference frame.
  double dSemiAngleDeg,                       // in : Angle between cone axis and surface of cone.
  double dStartAngleDeg,                      // in :
  double dEndAngleDeg,                        // in :
  SmNurbCircleParam eParameterization,        // in : SM_CO_QUADRATIC - produces a surface using a quadratic (degree 2) parameterization.
                                              //      SM_CO_QUINTIC - produces a surface using a quintic (degree 5) parametrization.
  const SmExtent3d & crBoundingBox,           // in :
  SmBSplineSurface *& rpNewBSplineSurface     // out:
)
{
    const SmPoint3d &rLineStart = crReferenceFrame.GetOriginRef();
    SmVector3d sLineVec = crReferenceFrame.GetZAxis();

    SmPoint3d sSphCenter;
    double dSphRadius = 0.0;
    crBoundingBox.ComputeSphereBound(sSphCenter,dSphRadius);

    // Let's compute the actual origin point of the cone
    double dTanAng = smos_Tangent(dSemiAngleDeg*SM_PI/180.0);
    if (dTanAng < SM_EFF_ZERO) SER(SM_ERR);
    double dParamAtApex = - dRadius / dTanAng;
    SmPoint3d sActualOrigin = rLineStart + dParamAtApex * sLineVec;

    double dParam;
    SER(smgu_LineClosestPoint(sActualOrigin,sLineVec,sSphCenter,dParam));
    double dMinT = dParam - dSphRadius;
    double dMaxT = dParam + dSphRadius;

    // Sorry but the box is on the wrong side of the cone apex.
    if (dMaxT - SM_EFF_ZERO < 0.0) SER(SM_ERR);

    // Let's determine which is the real minimum value.
    double dRadiusStart = 0.0;
    if (dMinT <= 0.0) {
        dRadiusStart = 0.0;
        dMinT = 0.0;
    }
    else {
        dRadiusStart = dMinT * dTanAng;
    }

    double dRadiusEnd = dMaxT * dTanAng;

    double dHeight = dMaxT - dMinT;

    SmPoint3d sNewOrigin = sActualOrigin + dMinT * sLineVec;

    SmAxis2Placement sNewRef;
    sNewRef.SetCanonical(sNewOrigin,crReferenceFrame.GetXAxisRef(),crReferenceFrame.GetYAxisRef());

    SER(sm_CreateConePatch(crContext,sNewRef,dRadiusStart,dRadiusEnd,
        dStartAngleDeg,dEndAngleDeg,dHeight,eParameterization,SM_SP_U,rpNewBSplineSurface));

    return SM_SUCCESS;

} // end SmBSplineSurface::CreateConeThroughBox

/*******************************************************************//**
PURPOSE: Create nurb which exactly represents a patch of a cylinder 
    or cone.  By default the normals of the cone points to the outside
    of the cone.  You can use 'SwapUV' to change the orientation.  

NOTES: Note that the side of the cylinder starts at the start 
    angle and goes  counter clockwise until it hits the end angle.  
    An angle of 0 corresponds to the X axis of the Axis2Placement.  
    The start angle can be greater than the end angle.  To make a cylinder
    instead of a cone make the two radii equal.
***********************************************************************/
SmStatus SmBSplineSurface::CreateConePatch
  (const SmContext & crContext,                 // in :
   const SmAxis2Placement & crReferenceFrame,   // in :
   double dBottomRadius,                        // in :
   double dTopRadius,                           // in : If equal to dBottomRadius a cylinder is created
   double dStartAngleDeg,                       // in :
   double dEndAngleDeg,                         // in :
   double dHeight,                              // in : Perpendicular height measured along the axis of cylinder/cone
   SmNurbCircleParam eParameterization,         // in : SM_CO_QUADRATIC - produces a surface using a quadratic (degree 2) parameterization.
                                                //      SM_CO_QUINTIC - produces a surface using a quintic (degree 5) parametrization.
   SmBSplineSurface *& rpNewBSplineSurface      // out: Resulting new cone surface
  ) 
{
    SER(sm_CreateConePatch(crContext,crReferenceFrame,dBottomRadius,dTopRadius,
        dStartAngleDeg,dEndAngleDeg,dHeight,eParameterization, SM_SP_U,rpNewBSplineSurface));
    return SM_SUCCESS;

} // end SmBSplineSurface::CreateConePatch

/*******************************************************************//**
PURPOSE: Create an 3D iso-parametric curve of a SmBSplineSurface given the 
    Nurb parameter direction (U or V) and the constant parameter 
    in that direction.

VIRTUAL FUNCTION ---
    for SmBSplineSurface - Make a BSpline Curve          (exact)
        SmSphere         - Make a SmCircle Curve         (exact)
        SmPlane          - Make a Line                   (exact)
        SmCone           - Make a Line or SmCircle Curve (exact)
        SmTorus          - Make a SmCircle curve         (exact)
        All Others       - Make a piecewise Hermite Curve approximation
                              good to optional tolerance or
                              dLength * SM_EFF_ZERO_SQRT * 100.0.
***********************************************************************/
SmStatus SmBSplineSurface::CreateIsoParametricCurve
 (const SmContext  & crContext,        // in : context for created objects
  SmSurfParamType    eSurfParam,       // in : Defines which Nurb parameter direction on surface to extract curve from
                                       //      SM_SP_U = create constant u isoParameter curve
                                       //      SM_SP_V = create constant v isoParameter curve
  double             dIsoParameter,    // in : Defines Nurb parametric value at which to extract the curve.
                                       //      If eSurfParam==SM_SP_U this is a U parameter, if eSurfParam==SM_SP_V
                                       //      then this is the V parameter
  SmApproxTol3d      sApproxTol3d,     // NotUsed: in : passed to ApproximateCurve() when approximation is required.
                                       //      If set to 0.0, tolerance is set by system: old[curve length * 1.0e-4] new[GetApproxTol3d()]
  SmBSplineCurve  *& rpNewIsoCurve,    // out: 3d IsoParameterCurve
  const SmExtent2d * pOptDomain,       // in : optional trim bound for IsoParameterCurve, NULL to ignore, default:[NULL]
  double           * pOptMaxGap3d,     // out: opt achieved max gap, NULL to ignore, default:[NULL]
  SmCurve         ** pOptUVIsoCurve)   // out: opt 2d UVTrimCurve Line (diff parameterization), NULL to ignore, default:[NULL]
 const
{
  SM_REF1(sApproxTol3d) ;
  // check state - gwc: skip for now because method is called during transitions
  //  SM_ASSERT_VALID_CONSTRUCTION(this) ;

  // init output
  rpNewIsoCurve = NULL;
  if(pOptMaxGap3d) { *pOptMaxGap3d = 0.0 ; }

  // get natural or optional domain min/max points
  SmExtent2d sUVDomain =   (pOptDomain)
                         ? *pOptDomain
                         : GetNaturalUVDomain();
  double dMinParam, dMaxParam;
  if (eSurfParam == SM_SP_U ) { dMinParam = sUVDomain.GetMin().x;
                                dMaxParam = sUVDomain.GetMax().x;
                              }
  else                        { dMinParam = sUVDomain.GetMin().y;
                                dMaxParam = sUVDomain.GetMax().y;
                              }

  // Clamp ParamValues within Tolerance to interval boundaries
  if (   dIsoParameter < dMinParam 
      || dIsoParameter > dMaxParam) 
    {
      SmExtent1d sIvl(dMinParam,dMaxParam);

      // error - ParamValue out of boundary tolerance
      if (!sIvl.IsValueOnBoundary(dIsoParameter,SM_EFF_ZERO_SQRT)) 
        {
          SER(SM_ERR);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              smgfx_Erase();
              DrawUV(6,6);
              sm_GraphicsLoop();
            }
#endif
        }
      dIsoParameter = sIvl.ClampValue(dIsoParameter);
    
    } // end need to clamp boundary

  // when asked - build associated 2d UVTrimCurve line (different parameterization)
  if(pOptUVIsoCurve) 
    { *pOptUVIsoCurve = NULL ; 

      // define the isoParameter Curve as an object
      SmIsoCurve sIso(*this,eSurfParam,dIsoParameter,TRUE);
      sIso.SetContext(NULL);

      // build OptUVIsoCurve
      sIso.CreateUVIsoLine(*pOptUVIsoCurve, &crContext) ; 
    } 

  // locals
  gw_FLAG dir;
  gw_INDEX lCpointHighest;
  gw_DEGREE lDegree;
  gw_INDEX lKnotsHighest;
  gw_SURFACE *pSurface = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
  NER( pSurface );
  if (eSurfParam == SM_SP_U) 
    {
      dir            = NL_VDIR;  // constant U curves go in V direction
      lCpointHighest = pSurface->net->m;
      lDegree        = pSurface->q;
      lKnotsHighest  = pSurface->knv->m;
    }
  else 
    {
      dir            = NL_UDIR;  // constant V curves go in U direction
      lCpointHighest = pSurface->net->n;
      lDegree        = pSurface->p;
      lKnotsHighest  = pSurface->knu->m;
    }

  // allocate the ObjectMemory
  gw_CURVE *cur = sm_AllocateNurbCurve(lCpointHighest,lDegree,lKnotsHighest); // memory leak here
  NER(cur);
  SmMemDelete sCleanup(cur);

  // build the isoParameter gw_CURVE
  if(NL_YES == N_SrfExtractIsoCrv(pSurface, dIsoParameter, dir, cur, NULL))  // memory leak here
    { ERR_MSG(_T("ERROR returned from NLib\n")) ; }

  // Use it as m_pNurb in a BSplineCurve object
  rpNewIsoCurve = new (crContext) SmBSplineCurve(cur,3);
  NER(rpNewIsoCurve);
  sCleanup.Clear();

  // when asked - trim curve to optional domain limits
  if(pOptDomain)
    {
      SmExtent1d sTrimIvl =   eSurfParam == SM_SP_U
                            ? sUVDomain.GetVInterval()
                            : sUVDomain.GetUInterval() ;
      rpNewIsoCurve->Trim(sTrimIvl) ;  // may snap sIvl by tol to existing knots
    }
  
  // all done
  SM_ASSERT_MSG(rpNewIsoCurve != NULL, _T("SmBSplineSurface::CreateIsoParametricCurve: rpNewIsoCurve == NULL. This is a bug in method CreateIsoParametricCurve()")) ;
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateIsoParametricCurve

/*******************************************************************//**
PURPOSE: Create a swept surface by sweeping a curve by a vector.  Note
    that the sweep vector does not have to be perpendicular to the curve
    if the curve is planar.  The U direction of the new surface corresponds
    to the sweep direction.  The V direction of the new surface corresponds
    to the sweep curve.  Therefore, constant V iso-curves are always lines.

NOTES: No check is done here for sweeps which self-intersect.
***********************************************************************/
SmStatus SmBSplineSurface::CreateLinearSweep
  (const SmContext      & crContext,           // in : context for new object construction
   const SmBSplineCurve & crCurveToSweep,      // in : target Curve
   const SmVector3d     & crSweepVector,       // in : Defines magnitude and direction of the sweep
   SmBSplineSurface    *& rpNewBSplineSurface) // out: The swept surface with new domain
                                               //      UVDomainMin = { (0.0, CurveToSweep_Ivl->Min),
                                               //                      (1.0, CurveToSweep_Ivl->Max) }
{
  // locals
  SmVector3d sVec  = crSweepVector;
  double     dDist = crSweepVector.Length();

  // check state - need nonZero SweepVector
  if (dDist < SM_EFF_ZERO) SER(SM_ERR_INVALID_INPUT);
  SER(sVec.Unitize());

  // convert to NLib structs
  NL_POINT W;
  COPY_XYZ(sVec,W);

  NL_STACKS SC;
  gw_SURFACE Sur;
  SmStackHandler sStackKp(&SC);

  // init output
  N_SrfInitArrays(&Sur);

  // pass the call along
  if(NL_YES == N_CreateSrfExtrudeCrv(SM_REINTERPRET_CAST(gw_CURVE*,
                               ((SmBSplineCurve &)crCurveToSweep).GetOrCreateGwNurbPointer()), 
                               W, 
                               dDist,
                               NL_UDIR,   // extruded surface is linear in u direction
                               &Sur, 
                               &SC))
    { SER(SM_ERR); }

  // set output
  rpNewBSplineSurface = new (crContext) SmBSplineSurface(&Sur) ;

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateLinearSweep

/*******************************************************************//**
PURPOSE: Create an approximate SmBSplineSurface that is within
            tolerance of the exact offset surface.  

NOTES:

  Approximate When necessary - Out Surf->Domain(s) may be trimmed but not scaled 

METHOD ---

  1. Create starter Offset Surface with SmSurface::CreateOffsetSurface() - 
  2. For each requested span subdivision level (starting with no subdivisions)
     2a. Copy this surface
     2b. Insert knots for span subdivision and to make each knot fully multiple
     2c. build an approximate surface
         o - For each surface patch 
             - build approx bicubic surface
             - copy approx surface control points into newly copied surface
     2d. test the approximate surface
         o for each approx surface patch
             - test dist between SmOffsetSurface and approx surface points
             - If any dist exceeds tolerance mark the approx surface as bad
     2e. save and return good approximations
  3. if no subdivision level surface is good enough - 
     return without saving an offset surface
***********************************************************************/
SmStatus SmBSplineSurface::CreateOffsetWithPolygon
  (const SmContext & crContext,                 // in : context for new object construction
   double            dSignedOffsetDistance,     // in : offset distance (negative for insets)
   double            dThisApproxTol3d,          // in : max allowed approximation distance
   ULONG             lSubdivisionLevel,         // in : How many times to split spans before giving up 
                                                //      0-no subdivisions, 
                                                //      1-split spans a max of 1 time, (1 patch becomes 4)
                                                //      2-split spans a max of 2 times (1 patch becomes 8) . . 
   SmTArray<SmSurface*> & rOffsetSurfaces)      // out: Contains 1 approximation surface with the smallest number
                                                //        of subdivisions possible that is within tolerance
                                                //        of exact offset surface.  
                                                //      Contains no surfaces when finest allowed subdivision surface 
                                                //        is still out of tolerance
 const
{
  // init output
  rOffsetSurfaces.ReSet();

  // locals
  SmSurface* pIOff = NULL;

  // Build a temporary SmOffsetSurface containing a copy of this surface and the offset distance
  SER(SmSurface::CreateOffsetSurface(crContext,
                                     dSignedOffsetDistance,
                                     dThisApproxTol3d, 
                                     pIOff ));
  SmObjDelete sCleanIOff(pIOff);

  // specify the control point form
  SmControlPointFormType eForm = (IsRational())
                                 ? SM_CP_EUCLIDIAN_RATIONAL
                                 : SM_CP_NON_RATIONAL;
  // Nurb locals
  SmTArray<double>    sUKnots;
  SmTArray<double>    sVKnots;
  SmTArray<ULONG>     sUMult;
  SmTArray<ULONG>     sVMult;
  SmTArray<SmPoint3d> sCPnts;
  SmTArray<double>    sWeights;

  // for every subdivision level
  for (ULONG i=0; i<=lSubdivisionLevel; i++) 
    {
      // make temporary copy of this surface
      SmBSplineSurface *pOff = new (crContext) SmBSplineSurface(*this);
      SmObjDelete sCleanOff(pOff);

      // locals
      ULONG lUDeg = pOff->GetDegree(SM_SP_U);
      ULONG lVDeg = pOff->GetDegree(SM_SP_V);

      // no work - high degree surfaces
      if (lUDeg > 3 || lVDeg > 3)
        {
          return SM_SUCCESS; // Let's pick this one up some other day.
        }

      // elevate surface degrees to 3
      if (lUDeg < 3) { pOff->DegreeElevate(SM_SP_U,3); }
      if (lVDeg < 3) { pOff->DegreeElevate(SM_SP_V,3); }

      // Subdivide each span i times
      // make each knot fully multiple so the
      //  curve is in a 'bezier' form
      for (ULONG inum=0; inum<=i; inum++) 
        {
          // surface locals
          lUDeg = pOff->GetDegree(SM_SP_U);
          lVDeg = pOff->GetDegree(SM_SP_V);
          pOff->GetKnots(SM_SP_U,sUKnots,&sUMult);
          pOff->GetKnots(SM_SP_V,sVKnots,&sVMult);
          SmTArray<double> sMoreUKnots, sMoreVKnots;

          // figure out where to add more knots for subdivision and to make each knot full
          // for every UKnot - 
          for (ULONG iu=0; iu<sUKnots.GetSize(); iu++) 
            {
              // needed knots to make existing knots full
              for (ULONG im=sUMult[iu]; im<lUDeg; im++) 
                {
                  sMoreUKnots.Add(sUKnots[iu]);
                }

              // needed knots for subdivision
              if (i > 0 && iu > 0) 
                { // add knots between existing knots
                  for (ULONG j=0; j<lUDeg; j++) 
                    {
                      sMoreUKnots.Add((sUKnots[iu-1]+sUKnots[iu])/2.0);
                    }
                }
            } // end iter every UKnot

          // for every v knot
          for (ULONG iv=0; iv<sVKnots.GetSize(); iv++) 
            {
              // needed knots to make existing knots full
              for (ULONG im=sVMult[iv]; im<lVDeg; im++) 
                {
                  sMoreVKnots.Add(sVKnots[iv]);
                }
              
              // needed knots for subdivision
              if (i > 0 && iv > 0) 
                { // add knots between existing knots
                  for (ULONG j=0; j<lVDeg; j++) 
                    {
                      sMoreVKnots.Add((sVKnots[iv]+sVKnots[iv-1])/2.0);
                    }
                }
            } // end iter every VKnot

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
          if (bDebugMe2) {
              sMoreUKnots.Dump();
              sMoreVKnots.Dump();
              pOff->WriteToFile(_T("../OutputFiles/BeforeKnots.sms"));
          }
#endif
          // insert needed knots
          if (sMoreUKnots.GetSize() > 0) SER(pOff->InsertKnots(SM_SP_U,sMoreUKnots));
          if (sMoreVKnots.GetSize() > 0) SER(pOff->InsertKnots(SM_SP_V,sMoreVKnots));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) {
              smgfx_Erase();
              pOff->DrawUV(2,2);
              sm_GraphicsLoop();
              pOff->DrawPolygon();
              sm_GraphicsLoop();
              pOff->WriteToFile(_T("../OutputFiles/AfterKnots.sms"));
          }
#endif
        } // end iter i times subdividing spans

      // get knots of the subdivided surface
      pOff->GetKnots(SM_SP_V,sVKnots,&sVMult);
      pOff->GetKnots(SM_SP_U,sUKnots,&sUMult);

      // local single span approximation surface
      SmCubicBezierSurface sCB;

      // for every surface patch - copy single patch approximation control Points into new offset surface
      for (ULONG iu=1; iu<sUKnots.GetSize(); iu++) 
        {
          // for every VKnot but the first
          for (ULONG iv=1; iv<sVKnots.GetSize(); iv++) 
            {
              SmPoint2d sMin(sUKnots[iu-1],sVKnots[iv-1]);
              SmPoint2d sMax(sUKnots[iu],sVKnots[iv]);
              SmExtent3d sDom(sMin,sMax);

              // build cubic bezier approximation to this curve
              SER(pIOff->ComputeCubicBezierApprox(sDom,sCB));
              ULONG lUMin = (iu-1) * 3;
              ULONG lVMin = (iv-1) * 3;
              ULONG lNumCtrlU = pOff->GetNumberControlPoints(SM_SP_U);
              ULONG lNumCtrlV = pOff->GetNumberControlPoints(SM_SP_V);

              // copy span approximation control points into new offset curve, pOff
              for (ULONG iub=0; iub<=3; iub++) 
                {
                  for (ULONG ivb=0; ivb<=3; ivb++) 
                    {
                      double dWeight=1.0;
                      SmPoint3d sPnt;
                      if (   lUMin+iub >= lNumCtrlU
                          || lVMin+ivb >= lNumCtrlV) 
                        {
                          SE(SM_ERR); // Some problem here with knots
                          return SM_SUCCESS;
                        }
                      pOff->GetControlPoint(eForm,lUMin+iub,lVMin+ivb,sPnt,dWeight);
                      dWeight = 1.0;
                      pOff->SetControlPoint(eForm,lUMin+iub,lVMin+ivb,sCB.m_vP[ivb][iub],dWeight);
                    }
                }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3 = FALSE;
              if (bDebugMe3) {
                  smgfx_Erase();
                  smgfx_SetColor(0,0,0);
                  DrawPolygon();
                  sm_GraphicsLoop();
                  smgfx_SetColor(0,0,1);
                  pOff->DrawPolygon();
                  sm_GraphicsLoop();            
              }
#endif
            } // end iter every VKnot but the first
        } // end iter every UKnot but the first


      // Now test the accuracy of this control polygon offset surface.  If good enough
      // then we can stop here.
      {
        // locals 
        SmBoolean bFailed            = FALSE;
        double    dAbsOffsetDistance = smos_Fabs(dSignedOffsetDistance);
        pOff->GetKnots(SM_SP_U,sUKnots);
        pOff->GetKnots(SM_SP_V,sVKnots);
        ULONG lNumBetween = 3;

        // for every surface patch
        for (ULONG iu=1; iu<sUKnots.GetSize(); iu++) 
          {
            SmExtent1d sUIvl(sUKnots[iu-1],sUKnots[iu]);
            
            // for every U dir sample point
            for (ULONG k=0; k<=lNumBetween; k++) 
              {
                double dUParam = sUIvl.Evaluate( (k*1.0)/lNumBetween );

                // for every VKnot but the first
                for (ULONG iv=1; iv<sVKnots.GetSize(); iv++) 
                  {
                    SmExtent1d sVIvl(sVKnots[iv-1],sVKnots[iv]);

                    // for every V dir sample point
                    for (ULONG kk=0; kk<=lNumBetween; kk++) 
                      {
                        double dVParam = sVIvl.Evaluate( (kk*1.0)/lNumBetween );
                        SmPoint2d sUV(dUParam,dVParam);
                        SmPoint3d sPnt, sOffPnt;

                        // eval points on the SmOffsetSurface and the new approx surface
                        SER(EvaluatePoint(sUV,sPnt));
                        SER(pOff->EvaluatePoint(sUV,sOffPnt));

                        // when dist between points is not within tolerance of offset distance
                        double dDist = sPnt.DistanceBetween(sOffPnt);
                        if (smos_Fabs(dDist-dAbsOffsetDistance) > 10.0*dThisApproxTol3d) 
                          {
                            // mark the approx surface as no good and quit
                            bFailed = TRUE;
                            break;
                          }
                      } // end iter every V dir sample point
                    if (bFailed) break;
                  } // end iter every VKnot but the first
                if (bFailed) break;
              } // end iter every U patch sample point
            if (bFailed) break;
          } // end iter every UKnot but the first

        // when the approx surface is good enough
        if (!bFailed) 
          {
            // make it permanent - put it into the output - and return
            sCleanOff.Clear();
            rOffsetSurfaces.Add(pOff);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) {
          smgfx_Erase();
          smgfx_SetColor(0,0,0);
          DrawPolygon();
          sm_GraphicsLoop();
          smgfx_SetColor(0,0,1);
          pOff->DrawPolygon();
          sm_GraphicsLoop();            
      }
#endif
              return SM_SUCCESS;
          }
      } // end test this subDivision level approximation
    } // end iter every subdivision level

  return SM_SUCCESS;
    
} // end SmBSplineSurface::CreateOffsetWithPolygon

/*******************************************************************//**
PURPOSE: Create an offset surface in a generic way by using the
   implicit offset surface.

NOTES: Returns an SmOffsetSurface. Approximate When necessary.
    Out Surf->Domain(s) may be trimmed but not scaled.

    There remain cases when the resulting offset surface is self intersecting
       This is difficult to remedy after the offset surface is created
       An effort needs to be made to predict the self intersection by looking at the curvature

***********************************************************************/
SmStatus SmBSplineSurface::CreateOffsetSurface
(
  const SmContext      & crContext,             // in : context for new obj construction
  double                 dSignedOffsetDistance, // in : offset dist, (neg val = Offset dir opposite surface normal)
  SmApproxTol3d          dThisApproxTol3d,      // in : Max Dist between ApproxOffsetSurface and ideal offset shape
  SmSurface* &           rOffsetSurface         // out: Offset Surf Approx. Result may have self intersection
) const
{   

  // Create implicit offset surface
  SmSurface* sOffsetSurf = NULL;
  SER(SmSurface::CreateOffsetSurface(crContext,
                                     dSignedOffsetDistance,
                                     dThisApproxTol3d,
                                     sOffsetSurf));

  rOffsetSurface = sOffsetSurf;

  return SM_SUCCESS;

} // end SmBSplineSurface::CreateOffsetSurface

/*******************************************************************//**
PURPOSE: Create a new surface which is a mirror of the surface about the
    X, Y plane of the SmAxis2Placement.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::CreateMirrorSurface
(
  const SmContext & crContext,            // in :
  const SmAxis2Placement & crMirrorPlane, // in :
  SmSurface *& rpMirrorSurface            // out:
) const
{
    NL_STACKS SC;
    NL_RMATRIX rma;
    SmStackHandler sStackKp(&SC);
    N_InitRealMatrix(&rma);
    N_SetRealMatrix(&rma,3,3,full,3,&SC);
    gw_REAL  **RM = rma.RM;

    crMirrorPlane.GetMirrorMatrix(RM);

    SmBSplineSurface *pBSS = new (crContext) SmBSplineSurface(*this);
    NER(pBSS);

    N_SrfTransform(pBSS->GetOrCreateGwNurbPointer(),&rma);

    rpMirrorSurface = pBSS;
    return SM_SUCCESS;

} // end SmBSplineSurface::CreateMirrorSurface

/*******************************************************************//**
PURPOSE: Create a spherical surface from 3 arcs.

NOTES: It will generate a 'quiet' error if it fails because 
     curves are not arcs or for some other reason.
***********************************************************************/
SmStatus SmBSplineSurface::CreateSphereFromArcs
  (const SmContext                 & crContext,   // in : context for new object construction
   const SmTArray<SmBSplineCurve*> & crArcs,      // in : boundary curves to fill with a Spherical Surface - if possible
   double                            d3DTol,      // in : Max allowed gap between NewSphere and input Arcs
   SmTArray<SmCurve*>              & rCCWArcs,    // out: These are the same curves as passed in
                                                  //      by crArcs ordered CCW about the sphere
   SmTArray<SmOrientType>          & rOrients,    // out: Orientation of individual curves relative
                                                  //      to the CCW orientation about the surface.
   SmBSplineSurface               *& rpNewSphere) // out: SurfaceOfRevolution with small gaps to crArcs, else NULL
{
  // init outputs
  rCCWArcs.ReSet();
  rOrients.ReSet();
  rpNewSphere = NULL;

  // locals
  ULONG            lTotalCurves   = crArcs.GetSize();
  SmBSplineCurve * p3DCurve       = crArcs[0];
  SmBSplineCurve * pGenCurve      = p3DCurve;
  ULONG            lGenCurveIndex = 0;
  SmAxis2Placement sRefFrame;
  double           dRad, dStartAng, dEndAng;
  SmVector3d       sRefZAxis;

  // no work - not given exactly 3 curves
  if (lTotalCurves != 3) return SM_ERR; // Only works for 3 now

  // no work - first curve is not an arc
  if (!p3DCurve->IsArc(5,            // in : number of points to sample and test
                       d3DTol,       // in : max deviation from exact Arc allowed each samplePoint
                       sRefFrame,    // out: orientation for found arc,
                                     //      XAxis = centerPoint to StartPoint of curve's interval
                                     //      YAxis = perp to XAxis in direction of Pt on Curve at .15 of interval
                       dRad,         // out: Found Arc radius
                       dStartAng,    // out: Start angle in degrees
                                     //      relative to a counter clockwise angle about Z from 
                                     //      the X axis of reference frame.
                       dEndAng))     // out: End angle in degrees
                                     //      relative to a counter clockwise angle about Z from 
                                     //      the X axis of the reference frame.
    { return SM_ERR; }

  // extract Sphere Params from Arc shapes
  double            dAngleSpan = dEndAng - dStartAng;
  const SmPoint3d & rOrigin = sRefFrame.GetOriginRef();

  // for all other input arcs - check for Compatible centers and radii
  for (ULONG i=1; i<crArcs.GetSize(); i++) 
    {
      p3DCurve = crArcs[i];
      double dRad1;

      // no work - other Curve is not an arc of the same radius as the 1st Curve
      if(   !p3DCurve->IsArc(5,
                             d3DTol,
                             sRefFrame,
                             dRad1,
                             dStartAng,
                             dEndAng)
         || smos_Fabs(dRad1 - dRad) > d3DTol)
         { return SM_ERR; }

      // no work - OtherCurve center is not the same as 1st Curve cent
      SmVector3d sDiff = sRefFrame.GetOriginRef() - rOrigin;
      if(sDiff.Length() > d3DTol)
        { return SM_ERR; }

      // remember the largest ArcAngle seen
      double dAngleSpan1 = dEndAng - dStartAng;
      if (dAngleSpan1 > dAngleSpan) 
        {
          lGenCurveIndex = i;// Will choose the largest arc as generator curve
          dAngleSpan     = dAngleSpan1;
          pGenCurve      = p3DCurve;
        }
    } // end iter i, comparing 2nd and 3rd input arcs to the 1st for compatibility

  // Get 1st Neighbor Curve to largest Span
  p3DCurve = crArcs[(lGenCurveIndex+1)%lTotalCurves];
  SmBSplineCurve *pSecondLargest = p3DCurve;
  p3DCurve->IsArc(5,                // gwc: no need to recompute - if we save these values in above loop
                  d3DTol,
                  sRefFrame,
                  dRad,
                  dStartAng,
                  dEndAng);
  sRefZAxis  = sRefFrame.GetZAxis();
  dAngleSpan = dEndAng-dStartAng;

  // get other Neighbor Curve to largest Span
  p3DCurve = crArcs[((lGenCurveIndex+lTotalCurves)-1)%lTotalCurves];
  SmBSplineCurve *pSmallest = p3DCurve;
  double dStartAng1,dEndAng1;
  p3DCurve->IsArc(5,
                  d3DTol,
                  sRefFrame,
                  dRad,
                  dStartAng1,
                  dEndAng1);
  double dAngleSpan1 = dEndAng1-dStartAng1;

  // Order the Neighbor Curves based on length
  if (dAngleSpan1 > dAngleSpan) 
    {
      dAngleSpan     = dAngleSpan1;
      pSmallest      = pSecondLargest;
      pSecondLargest = p3DCurve;
      sRefZAxis      = sRefFrame.GetZAxis();
    }

  // Increase angle span by 10% to make sure it's big enough
  dAngleSpan *= 1.1;
  if (dAngleSpan > 360.0) dAngleSpan = 360.0;

  // Now find the non common point between the second largest and the gen curve
  SmPoint3d sSt1, sEnd1 ;
  SmPoint3d sSt2, sEnd2 ;
  SmPoint3d sSt3, sEnd3 ;
  SmPoint3d sNonCommonPnt;

  // GenCurve endPoints
  SER(pGenCurve->EvaluatePoint(pGenCurve->GetNaturalInterval().GetMin(),sSt1));
  SER(pGenCurve->EvaluatePoint(pGenCurve->GetNaturalInterval().GetMax(),sEnd1));

  // SecondLargest neighbor EndPoints
  SER(pSecondLargest->EvaluatePoint(pSecondLargest->GetNaturalInterval().GetMin(),sSt2));
  SER(pSecondLargest->EvaluatePoint(pSecondLargest->GetNaturalInterval().GetMax(),sEnd2));

  // Smallest neighbor EndPoints
  SER(pSmallest->EvaluatePoint(pSmallest->GetNaturalInterval().GetMin(),sSt3));
  SER(pSmallest->EvaluatePoint(pSmallest->GetNaturalInterval().GetMax(),sEnd3));

  // Set 1st Curve of output = GenCurve
  rCCWArcs.Add(pGenCurve);
  rOrients.Add(SM_OT_OPPOSITE);

  // Set 2nd and 3rd Curves of output based on EndPoint coincidences
  if     (sSt1.DistanceBetween(sSt2) < d3DTol)  { sNonCommonPnt = sEnd1;
                                                  rCCWArcs.Add(pSecondLargest);
                                                  rOrients.Add(SM_OT_SAME);

                                                  rCCWArcs.Add(pSmallest);
                                                  if (sEnd1.DistanceBetween(sSt3) < d3DTol) { rOrients.Add(SM_OT_OPPOSITE); }
                                                  else                                      { rOrients.Add(SM_OT_SAME); }
                                                }
  else if(sSt1.DistanceBetween(sEnd2) < d3DTol) { sRefZAxis     = - sRefZAxis;
                                                  sNonCommonPnt = sEnd1;
                                                  rCCWArcs.Add(pSecondLargest);
                                                  rOrients.Add(SM_OT_OPPOSITE);

                                                  rCCWArcs.Add(pSmallest);
                                                  if (sEnd1.DistanceBetween(sSt3) < d3DTol) { rOrients.Add(SM_OT_OPPOSITE); }
                                                  else                                      { rOrients.Add(SM_OT_SAME); }
                                                }
  else if(sEnd1.DistanceBetween(sSt2) < d3DTol) { sNonCommonPnt = sSt1;
                                                  rCCWArcs.Add(pSmallest);
                                                  if (sSt1.DistanceBetween(sSt3) < d3DTol) { rOrients.Add(SM_OT_SAME); }
                                                  else                                     { rOrients.Add(SM_OT_OPPOSITE); }

                                                  rCCWArcs.Add(pSecondLargest);
                                                  rOrients.Add(SM_OT_OPPOSITE);
                                                }
  else if(sEnd1.DistanceBetween(sEnd2) < d3DTol){ sRefZAxis     = - sRefZAxis;
                                                  sNonCommonPnt = sSt1;
                                                  rCCWArcs.Add(pSmallest);
                                                  if (sSt1.DistanceBetween(sSt3) < d3DTol) { rOrients.Add(SM_OT_SAME); }
                                                  else                                     { rOrients.Add(SM_OT_OPPOSITE); }

                                                  rCCWArcs.Add(pSecondLargest);
                                                  rOrients.Add(SM_OT_SAME);
                                                }
  else                                          { SER(SM_ERR); }

  // Define ZAxis in same general direction as sRefZAxis
  SmVector3d sZAxis = rOrigin - sNonCommonPnt;
  if (sZAxis.Dot(sRefZAxis) < 0.0) 
    { sZAxis = - sZAxis; }
  sZAxis.Unitize() ;

  // Make the Sphere as a SurfOfRevolution
  SmBSplineSurface * pSurface = NULL;
  SER(SmBSplineSurface::CreateSurfOfRevolution(crContext,
                                               pGenCurve,
                                               rOrigin,
                                               sZAxis,
                                               dAngleSpan,
                                               pSurface));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_SetLook(1,4, 1,0,0); rCCWArcs[0]->DrawWDeriv(rCCWArcs[0]->GetNaturalInterval(),0); sm_GraphicsLoop();
      smgfx_SetLook(1,4, 1,1,0); rCCWArcs[1]->DrawWDeriv(rCCWArcs[1]->GetNaturalInterval(),0); sm_GraphicsLoop();
      smgfx_SetLook(1,4, 1,0,1); rCCWArcs[2]->DrawWDeriv(rCCWArcs[2]->GetNaturalInterval(),0); sm_GraphicsLoop();
      smgfx_SetLook(1,4, 0,0,0); pSurface->DrawUV(20,4); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  NER(pSurface) ;

  // set output
  rpNewSphere = pSurface;

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateSphereFromArcs

/*******************************************************************//**
PURPOSE: Create a surface of revolution given a generator curve, an
    axis of revolution and angle of revolution (measured  clockwise
    looking in the direction of axis)

NOTES: 
    This can create surfaces which are parameterised 0-1 in U and 0-360 
    in V which can cause angle problems in uv space.
***********************************************************************/
SmStatus SmBSplineSurface::CreateSurfOfRevolution
  (const SmContext      & crContext,           // in : new object context
   const SmBSplineCurve * pGenCurve,           // in : Generating curve
   const SmPoint3d      & crOrigin,            // in : Origin of the axis of revoluation
   const SmVector3d     & crAxisDir,           // in : Direction vector of axis of revolution 
   double                 dAngleSpanDeg,       // in : Angle of revolution (> 0.0 && <= 360.0) in degrees
   SmBSplineSurface    *& rpNewBSplineSurface, // out: The new surface 
   SmBoolean              bSwapUV,             // in : TRUE = Degree 2 circular U direction (constant V iso curves) 
                                               //      FALSE= Degree 2 circular V direction (constant U iso curves)
                                               //      default:[FALSE]
   const SmCircle       * pOptSweepCurve)      // in : Opt Surface SweepDirection Parameterization specification.
                                               //      Circle must be centered upon and normal to the Revolution axis.
                                               //      NULL to ignore. default:[NULL]
{
  // unitized rotation axis
  SmVector3d sAxisDir = crAxisDir;
  SER(sAxisDir.Unitize());

  // GenCurve locals
  SmExtent1d sCrvIvl = pGenCurve->GetNaturalInterval();
  ULONG lDim, lCrvDeg=0;
  SmBSplineCurveForm  eCurveForm;
  SmTArray<SmPoint3d> sCrvCtrlPts;
  SmTArray<double>    sCrvKnots;
  SmTArray<ULONG>     sCrvKnotMult;
  SmTArray<double>    sCrvWeights;
  SmKnotType          eKnotType;
  SER(pGenCurve->GetCanonical(lDim, lCrvDeg, sCrvCtrlPts, eCurveForm,
                             sCrvKnotMult, sCrvKnots,
                             eKnotType, sCrvWeights));
  SmBoolean bIsGenCrvRational =  (sCrvWeights.GetSize() != 0)
                                ? TRUE
                                : FALSE ;

  // Sweep direction locals
  ULONG            lCirDim, lCirDeg = 0;
  SmTArray<double> sCirKnots;
  SmTArray<ULONG>  sCirKnotMult;
  SmTArray<double> sCirWeights;
  SmExtent1d       sCirIvl;
  ULONG            lNumCirCtrlPts = 0;

  // when given an option SweepCurve
  if(pOptSweepCurve)
    {
      // use it to specify the sweep curve parameters

      // get the sweep circle parameters
      SmTArray<SmPoint3d> sCirCtrlPts;
      SER(pOptSweepCurve->SmBSplineCurve::GetCanonical(lCirDim, lCirDeg, 
                                                       sCirCtrlPts, eCurveForm,
                                                       sCirKnotMult, sCirKnots, 
                                                       eKnotType, sCirWeights));
      lNumCirCtrlPts = sCirCtrlPts.GetSize();
      sCirIvl        = pOptSweepCurve->GetNaturalInterval();
    }
  else // get sweep curve parameters from new sweep curve built by sweeping 1st nonZero radius controlPoint
    {
      // Rotate each control point about the axis to get a circle
      // In order to determine the size of control-points array & knots
      // array of the circles, we need to find the first non-degenerate one

      // for every GenCurve ControlPoint
      for (ULONG i=0; i<sCrvCtrlPts.GetSize(); i++) 
        {
          SmPoint3d sPnt        = sCrvCtrlPts[i];
          double    dScaledZero = SM_EFF_ZERO * (1.0 + sPnt.GetMaxDimension());
      
          // Get sPnt/RotationAxis Distance
          SmVector3d sXVec   = sPnt - crOrigin;
          double     dT      = sXVec.Dot(sAxisDir);
          SmPoint3d  sCenter = crOrigin + dT*sAxisDir;
          sXVec              = sPnt - sCenter;
          double     dRadius = sXVec.Length();

          // skip points on the rotation axis
          if (dRadius < dScaledZero) 
            { continue; }

          // create circle segment for this point's rotational sweep
          SmBSplineCurve  *pCir  = NULL;
          sXVec                  = sXVec / dRadius;;
          SmVector3d       sYVec = sAxisDir * sXVec;
          SmAxis2Placement sRefFrame(sCenter,sXVec,sYVec);
          SER(SmBSplineCurve::CreateCircleSegment(crContext, 3, sRefFrame,
                                                  dRadius, 0.0, dAngleSpanDeg, 
                                                  SM_CO_QUADRATIC, pCir));
          SmObjDelete sClean(pCir);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
          // draw 
          if(bDebugMe)
            {
              pCir->Dump() ;
            }
#endif // SM_DEBUG_CODE

          // If we have a cylinder, reparametrize it with arc length
          if (   pGenCurve->GetType() == SmLine_TYPE
              && ((SmLine *)pGenCurve)->GetLineVector().IsParallelTo(sAxisDir,0.001)) 
            {
              SER(pCir->ReparametrizeWithArcLength());
            }

          // get the sweep circle parameters
          SmTArray<SmPoint3d> sCirCtrlPts;
          SER(pCir->GetCanonical(lCirDim, lCirDeg, sCirCtrlPts, eCurveForm,
                                 sCirKnotMult, sCirKnots, eKnotType, sCirWeights));
          lNumCirCtrlPts = sCirCtrlPts.GetSize();
          sCirIvl        = pCir->GetNaturalInterval();

          // quit after 1st controlPoint not on rotation axis
          break;

        } // end iter every GenCurve ControlPoint looking for 1st one off the rotation axis
    } // end no pOptSweepCurve given - so build one from 1st nonDegenerate ControlPoint branch

  // arrive here after building sweep and getting its controlPoints and knot vector

  // quit when all GenCurve ControlPoints are on rotation vector
  if (lNumCirCtrlPts == 0) 
    {
      SER(SM_ERR);
    }

  // For every ControlPoint in the output Surface, build a sweep circle.
  SmTArray<SmPoint3d> sCtrlPts;
  SmTArray<double> sWeights;
  for (ULONG j=0; j<sCrvCtrlPts.GetSize(); j++) 
    {
      SmPoint3d sPnt        = sCrvCtrlPts[j];
      double    dScaledZero = SM_EFF_ZERO * (1.0 + sPnt.GetMaxDimension());
      
      // Calculate this sweep circles new controlPoint weights 
      for (ULONG ii=0; ii<lNumCirCtrlPts; ii++) 
        {
          double dWeight = sCirWeights[ii];
          if (bIsGenCrvRational) 
            {
              dWeight *= sCrvWeights[j];
            }
          sWeights.Add(dWeight);
        }

      // get sweep radius for this ControlPoint
      SmVector3d sXVec   = sPnt - crOrigin;
      double     dT      = sXVec.Dot(sAxisDir);
      SmPoint3d  sCenter = crOrigin + dT*sAxisDir;
      sXVec              = sPnt - sCenter;
      double     dRadius = sXVec.Length();

      // when radius is zero - make all controlPoints for this sweep circle the same
      if (dRadius < dScaledZero) 
        {
          // Add this control point lNumCirCtrlPts times
          for (ULONG jj=0; jj<lNumCirCtrlPts; jj++) 
            {
              sCtrlPts.Add(sPnt);
            }
          continue;
        }

      // build sweep circle for this control point
      SmBSplineCurve * pCir  = NULL;
      if(pOptSweepCurve)
        {
          // copy, scale, and move sweep circle to desired location
          // this lets pSweepCurve specify the sweep parameterization of Surface
          SmCurve *pSweepCopy = NULL ;
          pOptSweepCurve->Copy(crContext, pSweepCopy) ;
          pCir = (SmBSplineCurve *)pSweepCopy ;

          // set up transformation from GivenSweepCurve to needed SweepCurve
          double                  dSweepRadius = pOptSweepCurve->GetRadius() ;
          const SmAxis2Placement &rSweepFrame  = pOptSweepCurve->GetPosition() ;
          const SmPoint3d        &rOrigin      = rSweepFrame.GetOriginRef() ;
          double                  dScale       = dRadius/dSweepRadius ;

          // Check for inverted circle.  [B505]
          // Notes: (1) This likely means that the generator curve crosses the rotation axis,
          //  which would mean a self-intersecting result.  The caller should check that.
          //  (2) This creates a transform with a negative scale, which creates an SmCircle
          //  with a negative radius.  That works though, and the circle is transient.
          //  (We just use its control points.)
          if ( sXVec.Dot( rSweepFrame.GetXAxisRef() ) < 0.0 )
            { dScale *= -1.0; }

          SmVector3d              sMoveVec     = - dScale  * rOrigin 
                                                 + sCenter ;
                                            // = (1.0 - dScale) * rOrigin // translate to current origin
                                            //   + sCenter - rOrigin ;    // move to new circle center
          SmAxis2Placement sMove ;
          sMove.Translate(sMoveVec) ;
          SmVector3d sScale(dScale, dScale, dScale) ;
          pCir->Transform(sMove, &sScale) ; 
        } // end copy given sweep circle to get its parameterization branch
      else // generate a sweep circle from scratch
        {
          sXVec /= dRadius ; // Unitize.
          SmVector3d sYVec = sAxisDir * sXVec;
          SER(sYVec.Unitize());
          SmAxis2Placement sRefFrame(sCenter,sXVec,sYVec) ;
          SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sRefFrame,
                                                  dRadius,0.0,dAngleSpanDeg,
                                                  SM_CO_QUADRATIC,pCir));
        }
      SmObjDelete sClean(pCir);

      // Add sweep circle controlPoint to newSurface ControlPoint array
      SmTArray<SmPoint3d> sCirCtrlPts;
      SER(pCir->GetPolygon(pCir->GetNaturalInterval(),sCirCtrlPts));
      for (ULONG k=0; k<lNumCirCtrlPts; k++) 
        {
          sCtrlPts.Add(sCirCtrlPts[k]);
        }
    } // end iter every ControlPoint in the output surface

  // create the rotational BSplineSurface
  SmBSplineSurfaceForm eSurfForm = SM_SF_SURF_OF_REVOLUTION;
  SmExtent2d sUVDomain(sCrvIvl.GetMin(),sCirIvl.GetMin(),
                       sCrvIvl.GetMax(),sCirIvl.GetMax());

  // UV params are already set for SwapUV - just build the BSpline 
  if(bSwapUV) { SER(SmBSplineSurface::CreateCanonical(crContext,
                                                      lCrvDeg, lCirDeg,
                                                      sCtrlPts, eSurfForm,
                                                      sCrvKnotMult, sCirKnotMult,
                                                      sCrvKnots, sCirKnots,
                                                      eKnotType, &sWeights, &sUVDomain,
                                                      rpNewBSplineSurface));
              }
  else        { // SwapUV is not set - Transpose Weights and control points arrays
                // ULONG lNumVCpts = sCirKnots.GetSize() - lCirDeg - 1;
                sCtrlPts.Transpose(lNumCirCtrlPts);
                sWeights.Transpose(lNumCirCtrlPts);
                SmExtent2d sTransDomain(sUVDomain.GetMin().y,
                                        sUVDomain.GetMin().x,
                                        sUVDomain.GetMax().y,
                                        sUVDomain.GetMax().x) ;
                // then build the BSpline
                SER(SmBSplineSurface::CreateCanonical(crContext,
                                                      lCirDeg, lCrvDeg, 
                                                      sCtrlPts, eSurfForm,
                                                      sCirKnotMult, sCrvKnotMult, 
                                                      sCirKnots, sCrvKnots, 
                                                      eKnotType, &sWeights, &sTransDomain,
                                                      rpNewBSplineSurface));
              }
//        // The UV params of the surface has already been reversed, just swap back 
//        if (!bSwapUV) 
//          {
//            SER(rpNewBSplineSurface->SwapUV());
//          }

  // set form factor
  rpNewBSplineSurface->SetBSplineSurfaceForm(SM_SF_SURF_OF_REVOLUTION);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1); rpNewBSplineSurface->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,0); pGenCurve->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::CreateSurfOfRevolution

/********************************************************************//**
PURPOSE: Sweep a curve along a spiral around a given RefFrame's ZAxis.

NOTES: Input BSplineCurve expected to be 
         1. contained in the RefFrame's Z=0 plane
         2. At least G1 over its entire length.
       Passes call along to N_CreateSpiralSrf()
 ***********************************************************************/
SmStatus SmBSplineSurface::CreateHelicalSweep
 (const SmContext        & crContext,           // in : context for new object construction
  const SmAxis2Placement & crReferenceFrame,    // in : Helix centered on Z Axis starting at Z=0 on X Axis, running to Z=Height
  const SmBSplineCurve   & crPlanarCurve,       // in : Planar BSplineCurve built in the RefFrame's Z=0 plane
  double                   dHeight,             // in : Helix Length along RefFrame Z Axis, Height:[0=build spiral,>0=build helix]
  double                   dRadiusStart,        // in : Helix Radius at RefFrame Z = 0      (linearly interpolated from 0 to Height)
  double                   dRadiusEnd,          // in : Helix Radius at RefFrame Z = Height (linearly interpolated from 0 to Height)
  double                   dNumTurns,           // in : Num of 360 deg Helix Turns in H=[0,Height] (Fractions okay), PeriodLength:[Height/NumTurns]
  SmBoolean                bRightHanded,        // in : TRUE = spiral is right handed, FALSE=left handed
  SmApproxTol3d            sApproxTol3d,        // in : Max allowed deviation between NewBSplineCurve and theoretical Helix
  SmBSplineSurface      *& rpNewBSplineSurface) // out: NewSurface = GenCurve swept along spiral
{
  gw_CURVE * pInputCurve = crPlanarCurve.GetGwNurbPointer() ;

  // check input
  SER(dHeight       >= -SM_EFF_ZERO ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ;
  SER(dRadiusStart  >= 0.0          ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ;
  SER(dRadiusEnd    >= 0.0          ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ;
  SER(dNumTurns     >= SM_EFF_ZERO  ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ;
  SER(pInputCurve   != NULL         ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ; 
  
  // copy Curve to global Z=0 plane from RefFrame Z=0 plane
  SmCurve * pGenCurve = NULL ;
  crPlanarCurve.Copy(crContext, pGenCurve) ;
  SmObjDelete sCleanCurve(pGenCurve) ; 

  SmAxis2Placement sInvRefFrame ;
  crReferenceFrame.Invert(sInvRefFrame) ;
  pGenCurve->Transform(sInvRefFrame) ;

  // check that input is planar on the Z=0 plane
  SmExtent3d sBBox ;
  pGenCurve->CalculateBoundingBox(pGenCurve->GetNaturalInterval(), &sBBox) ;
  SER(sBBox.ZLength() < SM_EFF_ZERO ? SM_SUCCESS : SM_ERR_INVALID_INPUT ) ;

  // locals  
  NL_STACKS      SC;
  NL_SURFACE     Sur ;
  SmStackHandler sStackKp(&SC) ;
  N_SrfInitArrays(&Sur) ;
  NL_INDEX lNumPoints = 0 ;   // i/o: 0 = fit helix sweep to tolerance                         
                              //      else number of control points in sweep direction.         
                              //      Set to number of sweep direction control points on exit.  
  NL_REAL  dApproxTol3d = sApproxTol3d ;

  // double dPeriodLength = dHeight / dNumTurns ;  // Not needed
            
  // Build the helical approximation
  if(NL_YES == N_CreateSpiralSrf(pInputCurve,
                                 dHeight, 
                                 dRadiusStart, 
                                 dRadiusEnd, 
                                 dNumTurns, 
                                 bRightHanded, 
                                 &lNumPoints, 
                                 &dApproxTol3d, 
                                 &Sur, 
                                 &SC))
    { SER(SM_ERR); }

  // Build SMLib Surface from NLib Surface
  rpNewBSplineSurface = new (crContext) SmBSplineSurface(&Sur, FALSE, &crContext) ;

  // place the Surface back at its intended position
  rpNewBSplineSurface->Transform(crReferenceFrame) ;

  rpNewBSplineSurface->m_eBSplineSurfaceForm = SM_SF_HELICAL_SWEEP ;

  // all done
  return SM_SUCCESS ;

} // end SmBSplineSurface::CreateHelicalSweep

/*******************************************************************//**
PURPOSE: Create a trimmed plane corresponding to this definition of
    a ruled surface between two curves.  The two curves must lie on the
    same plane and define a valid ruled surface.  Please note that one of the
    curves can be a degenerate curve.

NOTES: If we can create an untrimmed nurb (ruled surface is a 
    rectangle), it will not return trimmed curves.
***********************************************************************/
SmStatus SmBSplineSurface::CreateTrimmedPlaneFromRuled
(
  const SmContext         & crContext,           // in :
  const SmBSplineCurve    & crCurve1,            // in :
  const SmBSplineCurve    & crCurve2,            // in :
  SmTArray<SmCurve*>      & r3DTrimmingCurves,   // in :
  SmTArray<SmOrientType>  & rTrimOrientations,   // in :
  SmBSplineSurface       *& rpPlanarNurb         // out:
)  
{
    SmPoint3d sStart, sEnd;
    crCurve1.GetEnds(sStart,sEnd);
    double dTol = SM_EFF_ZERO * 100.0 * (1.0 + sStart.GetMaxDimension() + 
        sEnd.GetMaxDimension());

    SmPoint3d sLinePoint1, sLinePoint2;
    SmVector3d sLineVec1, sLineVec2;
    if (crCurve1.IsLine(5,dTol,sLinePoint1,sLineVec1) &&
        crCurve2.IsLine(5,dTol,sLinePoint2,sLineVec2)) {
        double dParam1, dParam2;
        SER(smgu_LineClosestPoint(sLinePoint2,sLineVec2,sStart,dParam1));
        SER(smgu_LineClosestPoint(sLinePoint2,sLineVec2,sEnd,dParam2));
        if (smos_Fabs(dParam1+dParam2-1.0) < dTol &&
            (smos_Fabs(dParam1-1.0) < dTol || smos_Fabs(dParam2-1.0) < dTol)) {
            // Square ruled surface - this is easy just create the nurb.
            SER(SmBSplineSurface::CreateBilinearSurface(crContext,sStart,sEnd,
                sLinePoint2+dParam1*sLineVec2, sLinePoint2+dParam2*sLineVec2,
                rpPlanarNurb));
            NER(rpPlanarNurb);
            return SM_SUCCESS;
        }
    }
  
    SmBSplineSurface *pRuledSurface = NULL ;
    SER(SmBSplineSurface::CreateRuledSurface(crContext,
        crCurve1,crCurve2,SM_SP_V,pRuledSurface));
    NER(pRuledSurface);
    SmObjDelete sCleanRuled(pRuledSurface);
    
    SmExtent2d sDomain = pRuledSurface->GetNaturalUVDomain();
    SmPoint3d sPnt;
    SmVector3d sDU, sDV;
    SER(pRuledSurface->Evaluate1stDerivatives(sDomain.Evaluate(0.5,0.5),
        TRUE,TRUE,sPnt,sDU,sDV));
    SER(sDU.Unitize());
    sDV = sDU * sDV * sDU;
    SER(sDV.Unitize());
    
    SmExtent3d sBBox;
    SER(pRuledSurface->CalculateBoundingBox(sDomain,&sBBox));
    SmVector3d sSize = sBBox.GetSize();
    double dLeng= sSize.Length();
    SmExtent2d sTmpDomain(SmPoint2d(-dLeng,-dLeng),SmPoint2d(dLeng,dLeng));
    SmVector2d sScale(1,1);
    SmPlane *pPlane = new (crContext) SmPlane(sPnt,sDV,sDU,sScale,sTmpDomain);
    NER(pPlane);
    SmObjDelete sCleanPlane(pPlane);
    SmVector3d sPlaneNormal;
    SER(pPlane->EvaluateNormal(sTmpDomain.Evaluate(0.5,0.5),TRUE,TRUE,sPlaneNormal));

    // Make sure both curves are on the plane before continuing.
    SmBoolean bIsOnPlane;
    double dMaxDist;
    SER(crCurve1.IsOnPlane(sPnt,sPlaneNormal,dTol,bIsOnPlane,dMaxDist));
    if (!bIsOnPlane) {
        return SM_ERR;
    }
    SER(crCurve2.IsOnPlane(sPnt,sPlaneNormal,dTol,bIsOnPlane,dMaxDist));
    if (!bIsOnPlane) {
        return SM_ERR;
    }

    SmTArray<SmBSplineCurve*> sTmpUVTrimCurves;
    SmObjsDelete<SmBSplineCurve*> sCleanUVTrim(&sTmpUVTrimCurves);
    SER(pRuledSurface->CreateNaturalUVTrimCurves(crContext,
        pRuledSurface->GetNaturalUVDomain(),SM_SP_U,FALSE,
        r3DTrimmingCurves,sTmpUVTrimCurves,rTrimOrientations));
    SmPoint3d sCorners[4];
    SmExtent2d sPlaneDomain = pPlane->GetNaturalUVDomain();
    SER(pPlane->EvaluatePoint(sPlaneDomain.GetMin(),sCorners[0]));
    SER(pPlane->EvaluatePoint(sPlaneDomain.Evaluate(0.0,1.0),sCorners[1]));
    SER(pPlane->EvaluatePoint(sPlaneDomain.Evaluate(1.0,0.0),sCorners[2]));
    SER(pPlane->EvaluatePoint(sPlaneDomain.GetMax(),sCorners[3]));
    SER(SmBSplineSurface::CreateBilinearSurface(crContext,sCorners[0],sCorners[1],
        sCorners[2],sCorners[3],rpPlanarNurb));

    return SM_SUCCESS;

} // end SmBSplineSurface::CreateTrimmedPlaneFromRuled   

/*******************************************************************//**
PURPOSE: Find the span a UV point is in.  It can find the lower 
    left span.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::FindSpans
  (const SmPoint2d & crUV,
   SmBoolean bUFromLeft,      // in : if P is on U interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bVFromBottom,    // in : if P is on V interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   ULONG & rUIndex,           // out: 
   ULONG & rVIndex)           // out: 
  const
{
    gw_SURFACE *pSurface = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
    NL_FLAG bLeftFlag = (NL_FLAG)(bUFromLeft + 1);
    gw_INDEX idx_u, idx_v;
    N_BasisFindSpan(pSurface->knu,pSurface->p,crUV.x,bLeftFlag,&idx_u);
    NL_FLAG bBotFlag = (NL_FLAG)(bVFromBottom + 1);
    N_BasisFindSpan(pSurface->knv,pSurface->q,crUV.y,bBotFlag,&idx_v);
    rUIndex = idx_u;
    rVIndex = idx_v;
    return SM_SUCCESS;

} // end SmBSplineSurface::FindSpans

/*******************************************************************//**
PURPOSE: Get the STEP canonical data out of a B-Spline.

NOTES: Please note that the control points are in Euclidian
    space (not homogeneous) even if the surface is rational.  That is the 
    homogeneous division has been performed on x,y,z prior to returning 
    the data in rControlPointsList.
***********************************************************************/
SmStatus SmBSplineSurface::GetCanonical
  (ULONG                & rlUDegree,            // out: Surface polynomial U degree                        
   ULONG                & rlVDegree,            // out: Surface polynomial V degree
   SmTArray<SmPoint3d>  & rControlPointsList,   // out: Euclidian form of the control points.          
   SmBSplineSurfaceForm & reBSplineSurfaceForm, // oneof: SM_SF_PLANE_SURF,         SM_SF_CYLINDRICAL_SURF,        
                                                //        SM_SF_CONICAL_SURF,       SM_SF_SURF_OF_REVOLUTION,       
                                                //        SM_SF_SPHERICAL_SURF,     SM_SF_RULED_SURF,              
                                                //        SM_SF_GENERALIZED_CONE,        
                                                //        SM_SF_QUADRIC_SURF,       SM_SF_SURF_OF_LINEAR_EXTRUSION,
                                                //        SM_SF_UNSPECIFIED,
                                                //        SM_SF_POLYNOMIAL,         SM_SF_HELICAL_SWEEP     
   SmTArray<ULONG>      & rUKnotMultiplicities, // out: multiplicity value for each U knot                 
   SmTArray<ULONG>      & rVKnotMultiplicities, // out: multiplicity value for each V knot 
   SmTArray<double>     & rUKnots,              // out: Unique U knot vector
   SmTArray<double>     & rVKnots,              // out: Unique V knot vector
   SmKnotType           & reKnotType,           // oneof: SM_KT_UNIFORM_KNOTS,                         
                                                //        SM_KT_UNSPECIFIED,                           
                                                //        SM_KT_QUASI_UNIFORM_KNOTS,                   
                                                //        SM_KT_PIECEWISE_BEZIER_KNOTS  
   SmTArray<double>     & rWeights)             // out: associated weight for every ControlPoint,      
      const                                     //      rWeigths.GetSize() == 0 for non-rational curves
{
    SM_DUMP_AND_ASSERT2_VALID(this) ;
    gw_SURFACE* pSurface = ((SmBSplineSurface*)this)->GetOrCreateGwNurbPointer(); NER(pSurface);
    rlUDegree = pSurface->p;
    rlVDegree = pSurface->q;
    SER(sm_GetKnots(pSurface->knu,rUKnots,&rUKnotMultiplicities));
    SER(sm_GetKnots(pSurface->knv,rVKnots,&rVKnotMultiplicities));
    reKnotType = m_eKnotType;
    reBSplineSurfaceForm = m_eBSplineSurfaceForm;

    gw_CNET *pNet = pSurface->net;
    rControlPointsList.ReSet();
    rControlPointsList.SetSize((pNet->n+1)*(pNet->m+1));
    rWeights.ReSet();
    rWeights.SetSize((pNet->n+1)*(pNet->m+1));
    SmPoint3d sPnt;
    ULONG lCount=0;
    SmBoolean bIsRational = FALSE;
    for (gw_INDEX ii=0; ii<=pNet->n; ii++) {
        for (gw_INDEX j=0; j<=pNet->m; j++) {
            sPnt.x = pNet->Pw[ii][j].x;
            sPnt.y = pNet->Pw[ii][j].y;
            sPnt.z = pNet->Pw[ii][j].z;
            if (pNet->Pw[ii][j].w != NL_NOW) {
                bIsRational = TRUE;
                rWeights[lCount] = pNet->Pw[ii][j].w;
                sPnt.x = sPnt.x / rWeights[lCount];
                sPnt.y = sPnt.y / rWeights[lCount];
                sPnt.z = sPnt.z / rWeights[lCount];
            }
            rControlPointsList[lCount++] = sPnt;
        }
    }

    if (!bIsRational) rWeights.ReSet();

    return SM_SUCCESS;

} // end SmBSplineSurface::GetCanonical

/*******************************************************************//**
PURPOSE: Get Euclidean space control points and associated weights
            as an ordered list.

NOTES: Please note that the control points are in Euclidian
    space (not homogeneous) even if the surface is rational.  That is the 
    homogeneous division has been performed on x,y,z prior to returning 
    the data in rControlPointsList.

    ARRAY TO LIST INDEXING:
    pNet->Pw[i][j] = rControlPointsList[i*rlVCount+j]
***********************************************************************/
SmStatus SmBSplineSurface::GetControlPointNet
  (ULONG &rlUCount,                            // out: U controlPoint count
   ULONG &rlVCount,                            // out: V controlPoint count
   SmTArray<SmPoint3d> & rControlPointsList,   // out: Always produced in Euclidian coordinates
   SmTArray<double> & rWeights)                // out: The Weights array will have size zero if
                                               //      the surface is non-rational.
  const
{
  // locals
  gw_SURFACE *pSurface = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
  gw_CNET    *pNet     = pSurface->net;

  // set output counts
  rlUCount             = pNet->n + 1 ;
  rlVCount             = pNet->m + 1 ;

  // init output arrays
  rControlPointsList.ReSet();
  rControlPointsList.SetSize(rlUCount*rlVCount);

  rWeights.ReSet();
  rWeights.SetSize(rlUCount*rlVCount);

  // for every control point
  SmPoint3d sPnt;
  ULONG lCount=0;
  SmBoolean bIsRational = FALSE;
  for (ULONG ii=0; ii<rlUCount; ii++) 
    {
      for (ULONG jj=0; jj<rlVCount; jj++) 
        {
          // convert to cartesian coordinates as needed
          sPnt.x = pNet->Pw[ii][jj].x;
          sPnt.y = pNet->Pw[ii][jj].y;
          sPnt.z = pNet->Pw[ii][jj].z;

          // set outputs
          if (pNet->Pw[ii][jj].w != NL_NOW) 
            {
              bIsRational = TRUE;
              rWeights[lCount] = pNet->Pw[ii][jj].w;
              sPnt.x = sPnt.x / rWeights[lCount];
              sPnt.y = sPnt.y / rWeights[lCount];
              sPnt.z = sPnt.z / rWeights[lCount];
            }
          rControlPointsList[lCount++] = sPnt;

        } // end iter every V control point
    } // end iter every U control point

  // clear nonRational weight arrays
  if (!bIsRational) rWeights.ReSet();

  return SM_SUCCESS;

} // end SmBSplineSurface::GetControlPointNet

/*******************************************************************//**
PURPOSE: Get pointer to controlPoint array. 

NOTES: Return pointer to double array of control point
                values stored as [x, y, z, w]

WARNING ---  When a controlPoint value is modified
  internal caches become out of date.  Remember to call 
  Notify(SM_NO_PRE_EDIT,  this, SM_NO_GET_OWNER(this), NULL) and 
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL).
***********************************************************************/
SmStatus SmBSplineSurface::GetControlPointsPointer
  (ULONG   &lControlPointCountU,   // out: number of control points in each U row
   ULONG   &lControlPointCountV,   // out: number of control points in each V row
   double *&pControlPoints)        // out: array of controlPoints stored as doubles
                                   //      [ x00, y00, z00, w00, 
                                   //        x01, y01, z01, w01, .. 
                                   //        . . .          w0VCount-1,
                                   //        x10, y10, z10, w10, x11... ]
                                   //      For 2d control Point   z == NL_NOZ or 0.0
                                   //      for nonRational points w == NL_NOW
                                   //      for Rational points x,y,z are stored in homogeneous space
                                   //        i.e. CartesianX = x/w
                                   //             CartesianY = y/w
                                   //             CartesianZ = (z != NL_NOZ) ? z/w : NL_NOZ ;
const
{
  // locals
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();

  // set output
  lControlPointCountU = pSur->net->n + 1 ;
  lControlPointCountV = pSur->net->m + 1 ;
  pControlPoints      = (double *) *(pSur->net->Pw);

  // all done
  return(SM_SUCCESS) ;

} // end SmBSplineSurface::GetControlPointsPointer 

/*******************************************************************//**
PURPOSE: Get pointers to both knots arrays.

NOTES: Return pointers to double arrays of knot values.

WARNING ---  When a knot value is modified
  internal caches become out of date.  Remember to call 
  Notify(SM_NO_PRE_EDIT,  this, SM_NO_GET_OWNER(this), NULL) and 
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL)
  if you modify anything in the returned array.
***********************************************************************/
SmStatus SmBSplineSurface::GetKnotsPointers
  (ULONG   &lKnotCountU,  // out: number of knots in U direction
   ULONG   &lKnotCountV,  // out: number of knots in V direction
   double *&pKnotsU,      // out: ptr to array of U-knots
   double *&pKnotsV)      // out: ptr to array of V-knots
const
{
  // locals
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();

  // set output
  lKnotCountU = pSur->knu->m + 1;
  lKnotCountV = pSur->knv->m + 1;
  pKnotsU     = pSur->knu->U;
  pKnotsV     = pSur->knv->U;

  // all done
  return(SM_SUCCESS) ;

} // end SmBSplineSurface::GetKnotsPointer 

  /*******************************************************************//**
PURPOSE: Get the number of natural knots in the given parametric 
    direction on the surface.  The number of natural knots
    is the number of knots if the knots are represented as a single array
    with duplicated knot values.

NOTES: 
***********************************************************************/
ULONG SmBSplineSurface::GetNumberNaturalKnots
  (SmSurfParamType eSurfParam) 
 const
{
    gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
		if (!pSur) {
			return 0; // FS
		}
    if (eSurfParam == SM_SP_U) {
        return pSur->knu->m + 1;
    }
    return pSur->knv->m + 1;

} // end SmBSplineSurface::GetNumberNaturalKnots

/*******************************************************************//**
PURPOSE: Get the number of control points in the given parametric 
    direction from control polygon for this surface.

NOTES: When eSurfParam == SM_SP_BOTH returns
  total number of control points in surface
***********************************************************************/
ULONG SmBSplineSurface::GetNumberControlPoints
  (SmSurfParamType eSurfParam) // in : oneof 
                               //      SM_SP_U    = return U_Dir ControlPoint count      
                               //      SM_SP_V,   = return V_Dir ControlPoint count
                               //      SM_SP_BOTH = return Total ControlPoint count
 const
{
    gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
		if (!pSur) {
			return 0; // FS
		}
    return   (eSurfParam == SM_SP_U) ? (pSur->net->n + 1)
           : (eSurfParam == SM_SP_V) ? (pSur->net->m + 1)
           : (pSur->net->n + 1) * (pSur->net->m + 1) ;

} // end SmBSplineSurface::GetNumberControlPoints

/*******************************************************************//**
PURPOSE: return true when surface bounding box's largest side is less than tol.

NOTES: tolerance set to Max(dScaledZero,d3DTol)
***********************************************************************/   
SmBoolean SmBSplineSurface::IsDegeneratePoint
  (double      d3DTol,             // in : min distance between distinct points,
                                   //      default:[SM_ZONE_TOL_3D/10.0=1.0e-6]
   const SmExtent2d *pUVDomain)    // in : target domain to check, NULL = use NaturalUVInterval
                                   //      default:[NULL]
 const
{
  // low work - for nonNaturalUVInterval intervals pass the call 
  //  along to the general SmSurface::IsDegenerate
  if(   (    pUVDomain != NULL
         && !pUVDomain->AreEqual(GetNaturalUVDomain(), SM_EFF_ZERO))
     || IsKindOf(SmCrvOnSurf_TYPE))
    {
      return( SmSurface::IsDegeneratePoint(d3DTol, pUVDomain) ) ;
    }

  // ensure existence of m_pNurb
  if(m_pNurb == NULL) 
    { ((SmBSplineSurface *)this)->MakeNurb(); } 
  SM_ASSERT(m_pNurb != NULL) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
    }
#endif // SM_DEBUG_CODE

  // pass the call along
  return( sm_IsNSrfDegenerate(m_pNurb, d3DTol) ) ;

} // end SmBSplineSurface::IsDegeneratePoint

/*******************************************************************//**
GWC: this algorithm catches some but not all of the ways a surface
     can degenerate to a curve.  So use the spottier but more 
     sophisticated SmSurface::IsDegenerateCurve() method

PURPOSE: return true when surface degenerates to a curve due to coincident
  control point positions.

NOTES: tolerance set to Max(dScaledZero,d3DTol)
***********************************************************************/   
SmBoolean SmBSplineSurface::IsDegenerateCurve
 (double             dAngleTolDeg,   // in : min angle between nonParallel vectors, default:[SM_EFF_ZERO_DEG]
  const SmExtent2d * pOptUVDomain,   // in : OptUVDomain to check. NULL = use Natural Domain, default:[NULL]
  double             dOpt3DTol)      // in : min distance between distinct points,
                                     //      default:[SM_ZONE_TOL_3D/10.0=1.0e-6]
 const
{
  // gwc: for now - pass the call along
  return(SmSurface::IsDegenerateCurve(dAngleTolDeg, pOptUVDomain, dOpt3DTol)) ;

  // gwc: what's missing from the following code is the case where
  //      all the control points fall along a common line (or curve?) without 
  //      each row (or column) of CtrlPts collapsing down to a distinct point.
  //      If we add that check below then this algorithm would be
  //      preferable to the SmBSplineSurface::IsDegenerateCurve() method.

//        // get Nurb representation
//        gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
//      
//        // no work - no BSplineSurface representation
//        if(pSur == NULL)
//          { return FALSE ; }
//      
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE ;
//        // draw 
//        if(bDebugMe)
//          {
//            Dump_NSrf(pSur, false) ;
//          }
//      #endif // SM_DEBUG_CODE
//      
//        // pick tolerance
//        double dTolerance = dOpt3DTol > 0.0 ? dOpt3DTol : SM_ZONE_TOL_3D ;
//      
//        // locals
//        NL_INDEX ii, jj ;
//        NL_INDEX   iN       = pSur->net->n ; // max 1st index in PW
//        NL_INDEX   iM       = pSur->net->m ; // max 2nd index in PW
//        SmPoint3d  sPt, sPtNext ; 
//        SmExtent3d sRowBBox ;
//        SmBoolean  bDegen ;
//      
//        // for every row - check for degeneracy
//        for(ii=0;ii<=iN && bDegen == TRUE;ii++)
//          {
//            // get row's first control point
//            TO_EUCLID(pSur->net->Pw[ii][0], sPt) ;
//                
//            // reset the RowBBox with the first control point in the current row
//            bDegen = TRUE ;
//            sRowBBox.Init() ;
//            sRowBBox.AddPoint3d(sPt) ;
//      
//            // for every other Control Point in the row
//            for(jj=1;jj<=iM && bDegen == TRUE;jj++)
//              {
//                TO_EUCLID(pSur->net->Pw[ii][jj], sPtNext) ;
//      
//                // accumulate row control points BBox
//                sRowBBox.AddPoint3d(sPtNext) ;
//      
//                // quit when BBox is too big
//                if(sRowBBox.GetMaxDimension() > dTolerance)
//                  { bDegen = FALSE ; }
//      
//              } // end iter jj, every row control point
//      
//          } // end iter ii every internal row
//      
//        // done when every row degenerates to some point
//        if(bDegen == TRUE)
//          { return(TRUE) ; }
//      
//        // arrive here when all rows were not degenerate - check columns
//        bDegen = TRUE ;
//      
//        // for every column - check for degeneracy
//        for(jj=0;jj<=iM && bDegen == TRUE;jj++)
//          {
//            // get column's first control point
//            TO_EUCLID(pSur->net->Pw[0][jj], sPt) ;
//                
//            // reset the RowBBox with the first control point in the current column
//            bDegen = TRUE ;
//            sRowBBox.Init() ;
//            sRowBBox.AddPoint3d(sPt) ;
//      
//            // for every other Control Point in the column
//            for(ii=1;ii<=iN && bDegen == TRUE;ii++)
//              {
//                TO_EUCLID(pSur->net->Pw[ii][jj], sPtNext) ;
//      
//                // accumulate column control points BBox
//                sRowBBox.AddPoint3d(sPtNext) ;
//      
//                // quit when BBox is too big
//                if(sRowBBox.GetMaxDimension() > dTolerance)
//                  { bDegen = FALSE ; }
//      
//              } // end iter jj, every column control point
//      
//          } // end iter ii every internal column
//      
//        // arrive here after checking colums for degeneracies
//      
//        // all done
//        return(bDegen) ;

} // end SmBSplineSurface::IsDegenerateCurve

/*******************************************************************//**
PURPOSE: Determine if a surface is single-span bilinear.

NOTES: 
   Bilinear will mean degree 1 in both directions, AND a single span:
   the control net is 2x2.  Note it is not necessarily planar.
***********************************************************************/
SmBoolean SmBSplineSurface::IsBilinear() const
{
  if ( GetNumberControlPoints( SM_SP_U ) != 2 ) { return FALSE; }
  if ( GetNumberControlPoints( SM_SP_V ) != 2 ) { return FALSE; }
  return TRUE;
}

/*******************************************************************//**
PURPOSE: Determine if a surface is parallel to a given vector.  If it
    is, all of the control points in one direction must lie in a line that
    is parallel to the vector.

NOTES: 
***********************************************************************/
SmBoolean SmBSplineSurface::IsParallelToVector
  (const SmVector3d & crVector) 
 const
{
    SmVector3d sTestVec = crVector;
    SER(sTestVec.Unitize());

    const SmPlane *pPlane = SM_CAST_NONNULL_PTR(SmPlane,this);
    if (pPlane) {
         SmAxis2Placement sPos   = pPlane->GetPosition();
         SmVector3d       sZAxis = sPos.GetZAxis();
         double dTol1 = SM_EFF_ZERO * (1.0 + sPos.GetOriginRef().GetMaxDimension());
         if (smos_Fabs(sZAxis.Dot(sTestVec)) < dTol1) {
             return TRUE;
         }
         return FALSE;
    }

    gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
		if (!pSur) {
			return 0; // FS
		}
    gw_CNET *net = pSur->net;

    // First check U constant control points.
    SmBoolean bFailed = FALSE;
    for (long iU=0; iU<=net->n; iU++) {
        SmPoint3d sLinePnt, sLineEnd;
        TO_EUCLID(net->Pw[iU][0],sLinePnt);
        TO_EUCLID(net->Pw[iU][1],sLineEnd);
        double dTol = SM_EFF_ZERO * (1.0 + sLinePnt.GetMaxDimension() +
            sLineEnd.GetMaxDimension());
        SmVector3d sLineVec = sLineEnd - sLinePnt;
        if (sLineVec.LengthSquared() < SM_EFF_ZERO) {
            bFailed = TRUE;
            break;
        }
        SER(sLineVec.Unitize());
        if (smos_Fabs(sLineVec.Dot(sTestVec)) < 1.0 - dTol) {
            bFailed = TRUE;
            break;
        }
        for (long iV=2; iV<=net->m; iV++) {
             SmPoint3d sPnt;
             TO_EUCLID(net->Pw[iU][iV],sPnt);
             double dDistance;
             SER(smgu_LinePointDistance(sLinePnt,sLineEnd,sPnt,dDistance));
             if (dDistance > dTol) {
                 bFailed = TRUE; 
                 break; 
             }
        }
        if (bFailed) break;
    }

    if (!bFailed) {
        return TRUE;
    }

    // First check V constant control points.
    bFailed = FALSE;
    for (long iV=0; iV<=net->m; iV++) {
        SmPoint3d sLinePnt, sLineEnd;
        TO_EUCLID(net->Pw[0][iV],sLinePnt);
        TO_EUCLID(net->Pw[1][iV],sLineEnd);
        double dTol = SM_EFF_ZERO * (1.0 + sLinePnt.GetMaxDimension() +
            sLineEnd.GetMaxDimension());
        SmVector3d sLineVec = sLineEnd - sLinePnt;
        if (sLineVec.LengthSquared() < SM_EFF_ZERO) {
            bFailed = TRUE;
            break;
        }
        SER(sLineVec.Unitize());
        if (smos_Fabs(sLineVec.Dot(sTestVec)) < 1.0 - dTol) {
            bFailed = TRUE;
            break;
        }
        for (long iU=2; iU<=net->n; iU++) {
             SmPoint3d sPnt;
             TO_EUCLID(net->Pw[iU][iV],sPnt);
             double dDistance;
             SER(smgu_LinePointDistance(sLinePnt,sLineEnd,sPnt,dDistance));
             if (dDistance > dTol) {
                 bFailed = TRUE; 
                 break; 
             }
        }
        if (bFailed) break;
    }

    if (!bFailed) return TRUE;

    return FALSE;

} // end SmBSplineSurface::IsParallelToVector

/*******************************************************************//**
PURPOSE: Determines if the surface is rational.

NOTES: 
***********************************************************************/
SmBoolean SmBSplineSurface::IsRational
  () 
 const
{
    gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
		if (!pSur) {
			return 0; // FS
		}
    if (pSur->net->Pw[0][0].w == NL_NOW) {
        return FALSE;
    }
    return TRUE;

} // end SmBSplineSurface::IsRational

/*******************************************************************//**
PURPOSE: Get a range of knots from the knot vector and put them into
    the double array.  Note it is up to the user to make sure that there
    are at least (lEndIndex - lStartIndex + 1) doubles in the array.

NOTES: Be careful for this method, if you do not input sufficient
    array size it may over write some memory.
***********************************************************************/
SmStatus SmBSplineSurface::GetKnotsExpert
(
  SmSurfParamType eSurfParam,   // in :
  ULONG lStartIndex,            // in :
  ULONG lEndIndex,              // in :
  double *adKnots               // out:
) const
{
    gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
		if (!pSur) {
			return SM_ERR; // FS
		}
    gw_KNOTVECTOR *knt;
    if (eSurfParam == SM_SP_U) {
        knt = pSur->knu;
    }
    else {
        knt = pSur->knv;
    }
    SM_ASSERT(lEndIndex <= (ULONG)knt->m);
    SM_ASSERT(lStartIndex <= lEndIndex);
    for (ULONG i=lStartIndex; i<=lEndIndex; i++) {
        adKnots[i-lStartIndex] = knt->U[i];
    }
    return SM_SUCCESS;

} // end SmBSplineSurface::GetKnotsExpert

/*******************************************************************//**
PURPOSE: Get the Greville abscissa corresponding to a control point index.

NOTES:
    If the given index is out of range,
    the extreme value of the domain will be returned.
    Note however since the argument is ULONG and not int,
    it could never be on the low side of the range.
***********************************************************************/
double SmBSplineSurface::GetGrevilleAbscissa
(
   SmSurfParamType eSurfParam,    // in :
   ULONG lCtrlPointIndex          // in :
) const
{
  // locals
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
  SM_ASSERT(m_pNurb != NULL);

  ULONG lDeg, lNumPts, lNumKts;
  double *pdKnots;

  if ( eSurfParam == SM_SP_U )
  {
      lNumPts = pSur->net->n + 1;
      lNumKts = pSur->knu->m + 1;
      pdKnots = pSur->knu->U;
      lDeg    = pSur->p;
  }
  else
  {
      lNumPts = pSur->net->m + 1;
      lNumKts = pSur->knv->m + 1;
      pdKnots = pSur->knv->U;
      lDeg    = pSur->q;
  }
  SM_ASSERT( lNumKts == lNumPts + lDeg + 1 );

  if ( lCtrlPointIndex < 1 )
      { return pdKnots[0]; }         // at or below the beginning

  if ( lCtrlPointIndex >= lNumPts-1 )
      { return pdKnots[lNumKts-1]; } // at or beyond the end

  ULONG i;
  double dRetVal = 0;

  for ( i = 1; i <= lDeg; i++ )
  {
      dRetVal += pdKnots[ lCtrlPointIndex + i ];
  }
  dRetVal /= lDeg;

  return dRetVal;
}  // end SmBSplineSurface::GetGrevilleAbscissa

/*******************************************************************//**
PURPOSE: Get a control point in various forms.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::GetControlPoint
  (SmControlPointFormType eCtrlPointForm, // in : oneof SM_CP_NON_RATIONAL         = do perspective projection and
                                          //                                         set W=1.0 if it is rational.
                                          //            SM_CP_HOMOGENEOUS_RATIONAL = don't do division and return W
                                          //            SM_CP_EUCLIDIAN_RATIONAL   = do division and return W
   ULONG                  lUIndex,        // in : TgtCPoint U index value
   ULONG                  lVIndex,        // in : TgtCPoint V index value
   SmPoint3d            & rControlPoint,  // out: rControlPoint = (  SM_CP_NON_RATIONAL         ? [X   Y   Z  ]
                                          //                       : SM_CP_HOMOGENEOUS_RATIONAL ? [X*W Y*W Z*W]
                                          //                       : SM_CP_EUCLIDIAN_RATIONAL   ? [X   Y   Z  ] ) ;
   double               & rdWeight)       // out: rdWeight = (  SM_CP_NON_RATIONAL         ? 1.0  
                                          //                  : SM_CP_HOMOGENEOUS_RATIONAL ?  W 
                                          //                  : SM_CP_EUCLIDIAN_RATIONAL   ?  W ) ;
  const
{
  // locals
  double dTmp[4] ;
  dTmp[3] = 1.0 ; // Default for non-rational.

  // locals for GetControlPointsExpert() output array control arguments
  ULONG lPtSize           = (eCtrlPointForm == SM_CP_NON_RATIONAL) ? 3 : 4 ;
  ULONG lM                = 1 ;                      // M > 1 means leave (M-1) uninitialized columes between fetched columns
  ULONG lN                = 1 ;                      // N > 1 means leave (N-1) uninitialized rows between fetched rows
  ULONG lCtrlPointUStride = lM * lPtSize ;           // lCtrlPointUStride = M * PtSize ;
  ULONG lCtrlPointVStride = lN * lCtrlPointUStride ; // lCtrlPointVStride = N * ((lEndUIndex - lStartUIndex + 1) * lCtrlPointUStride) ;

  // pass the call along 
  // This is a special case call of GetControlPointsExpert() which is built to fetch arrays of ControlPoints.
  // In this case only one control point is being fetched.
  SER(GetControlPointsExpert(eCtrlPointForm,    // in : Request output Point coordinate format
                             lUIndex, lUIndex,  // in : U index range to fetch:[UMinIndex, UMaxIndex]      
                             lVIndex, lVIndex,  // in : V index range to fetch:[VMinIndex, VMaxIndex]      
                             lCtrlPointUStride, // in : number of doubles between fetched U ControlPoints in output array            
                             lCtrlPointVStride, // in : number of doubles between fetched V ControlPoints in output array             
                             dTmp)) ;           // out: array to load with fetched Control Point Values
                                                                                                                                             
  // set output
  rControlPoint.Set(dTmp[0], dTmp[1], dTmp[2]) ;
  rdWeight = dTmp[3] ;

  // all done
  return SM_SUCCESS ;

} // end SmBSplineSurface::GetControlPoint

/*******************************************************************//**
PURPOSE: Specialty function to move a subset of the input control point array
         into an expanded output array with optional spaces between existing
         control points to be assigned values later by the calling function. 

NOTES: 
   1. This method outputs an SMLib 1d array of control points with values preloaded 
      from an NLib 2d Array of control points with optional uninitialized columns and 
      rows between the fetched columns and rows of control points.
   
   2. map 2d NLib ControlPoint Array indices to SMLib 1d array indices as
        ControlPtNLib[i][j] = CntrlPtSMLib[  i * (VRowPointCount * PtSize) 
                                            + j * (PtSize) ] ;
        i.e. the v-rows are contiguous in memory: last index varies fastest.

   3. NLib   Control Point data is packed as: [X Y Z W]
      SMLib Output Control Point data packing depends on eCtrlPointForm as:
         SM_CP_NON_RATIONAL         pt = [X Y Z]           PtSize is 3                         
         SM_CP_HOMOGENEOUS_RATIONAL pt = [X Y Z W]         PtSize is 4                         
         SM_CP_EUCLIDIAN_RATIONAL   pt = [X/W Y/W Z/W W]   PtSize is 4  
      PtSize = ((SM_CP_NON_RATIONAL) ? 3 : 4)
                                
   4. lCtrlPointUStride = OutputArray stride (number of doubles) between neighoring U CPoints.
      lCtrlPointVStride = OutputArray stride (number of doubles) between neighoring V CPoints.
         where: 
          lCtrlPointUStride = N * (lCtrlPointVStride * (lEndUIndex - lStartUIndes + 1)), N range:[any positive integer]
          lCtrlPointVStride = M * PtSize,                                                M range:[any positive integer]
            M > 1 means the output array will contain M-1 uninitialized columns between every column of fetched CPoints.
            N > 1 means the output array will contain N-1 uninitialized rows between every row of fetched CPoints.

   
   5. Warning: this is an old school method.  Be careful to compute the input
               stride parameters and to preallocate enough memory in the output array
               to satisfy all the relationships mentioned above.

   Be careful for this method, if you do not input sufficient
   array size it may over write some memory.
     ArraySize:[lCtrlPointVStride * (lEndVIndex - lStartVIndex + 1)]

   lCtrlPointUStride: The number of doubles between consecutive U control points in the output array.
                      It's a multiple of the number of doubles between consecutive U pts.
   lCtrlPointVStride: The number of doubles between consecutive V control points in the output array.
                      It's a multiple of the number of doubles between consecutive V pts.

   Set lCtrlPointUStride = M * PtSize
   Set lCtrlPointVStride = N * ((lEndUIndex - lStartUIndex + 1) * lCtrlPointUStride)

   where M = number of U points in output pre original U Point step (1 = add no spaces, M = add M-1 pointSpaces between each Pair of U CtrlPts) 
         N = number of V Points in output per original V Point step (1 = add no spaces, N = add N-1 whole row of spaces between each Pair of V CtrlPt rows)

   Use M and N values larger than 1 when setting the Stride values
   if you wish to space the points out in the array.
***********************************************************************/
SmStatus SmBSplineSurface::GetControlPointsExpert
  (SmControlPointFormType eCtrlPointForm,    // in : Output array layout = specify data written to output array for each fetched control point.
                                             //      oneof SM_CP_NON_RATIONAL          = output euclidean coords only
                                             //            SM_CP_HOMOGENEOUS_RATIONAL  = output homogeneous coords and weights
                                             //            SM_CP_EUCLIDIAN_RATIONAL    = output euclidean coords and weights
                                             //      controls the number of doubles per point in the output array.
                                             //      PtSize = ((SM_CP_NON_RATIONAL) ? 3 : 4)
   ULONG                  lStartUIndex,      // in : First NlibArray U control point index to be copied
   ULONG                  lEndUIndex,        // in : Last  NlibArray U control point index to be copied
   ULONG                  lStartVIndex,      // in : First NlibArray V control point index to be copied
   ULONG                  lEndVIndex,        // in : Last  NlibArray V control point index to be copied
   ULONG                  lCtrlPointUStride, // in : Output array layout = number of doubles to leave between fetched U Control Points.
                                             //      lCtrlPointUStride = M * PtSize, M range:[any positive integer]
                                             //        M > 1 means M-1 uninitialized columns are built into the output array 
                                             //                        between every fetched column.
   ULONG                  lCtrlPointVStride, // in : Output array layout = number of doubles to leave between fetched V Control Points.
                                             //      lCtrlPointVStride = N * ((lEndUIndex - lStartUIndex + 1) * lCtrlPointUStride), N range:[any positive integer]
                                             //        N > 1 means N-1 uninitialized rows are built into the output array 
                                             //                        between every fetched row.
   double               * adControlPoints)   // out: SM_CP_NON_RATIONAL         && 2d ? ordered:[X Y          X Y          ...] 
                                             //      SM_CP_HOMOGENEOUS_RATIONAL && 2d ? ordered:[X Y W        X Y W        ...] 
                                             //      SM_CP_EUCLIDIAN_RATIONAL   && 2d ? ordered:[X/W Y/W W    X/W Y/W W    ...] 
                                             //      SM_CP_NON_RATIONAL         && 3d ? ordered:[X Y Z        X Y Z        ...] 
                                             //      SM_CP_HOMOGENEOUS_RATIONAL && 3d ? ordered:[X Y Z W      X Y Z W      ...] 
                                             //      SM_CP_EUCLIDIAN_RATIONAL   && 3d ? ordered:[X/W Y/W Z/W  X/W Y/W Z/W  ...] 
                                             //
                                             //      assumed sized on input - watch for boundary errors!
                                             //      sized:[ lCtrlPointVStride * (lEndVIndex - lStartVIndex + 1)]
  const
{
  // locals
  gw_SURFACE * pSur        = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
  gw_CNET    * net         = pSur->net;
  SmBoolean    bIsRational = IsRational();

  // when fetching just one point - don't need the stride info
  // (Both input stride values are multiplied by 0 for the first point.)
  SmBoolean    bNeedStride = (lEndUIndex > lStartUIndex) || (lEndVIndex > lStartVIndex);

  if(bNeedStride)
    {
      // Criteria for not overwriting ourselves in the output array:
      // not requiring them to be multiples, just so they're big enough.
      ULONG     lPtSize      = eCtrlPointForm == SM_CP_NON_RATIONAL ? 3: 4 ;
      SmBoolean bOKVStride   = lCtrlPointVStride >= lPtSize;

      ULONG     lRowSize     = lCtrlPointVStride * (lEndVIndex - lStartVIndex + 1) ;
      SmBoolean bOKUStride   = lCtrlPointUStride >= lRowSize;

      // check inputs - consistent indexing and strides
      if(   lStartUIndex > lEndUIndex
         || lEndUIndex   > (ULONG)net->n
         || lStartVIndex > lEndVIndex
         || lEndVIndex   > (ULONG)net->m
         || bOKUStride   != TRUE
         || bOKVStride   != TRUE)
        {
          SER_MSG(SM_ERR_INVALID_INPUT,_T("SmBSplineSurface::GetControlPointsExpert inconsistent input args"));
        }
    } // end need Stride check
    
  // copy the requested array of control points into the requested output array shape
  ULONG iU, iV, lUCnt = 0;
  for (iU=lStartUIndex; iU<=lEndUIndex; iU++) 
    {
      ULONG lVCnt = 0;
      for (iV=lStartVIndex; iV<=lEndVIndex; iV++) 
        {
           gw_CPOINT * pCpt = &net->Pw[iU][iV];
           double      dX   = pCpt->x;
           double      dY   = pCpt->y;
           double      dZ   = pCpt->z;
           double      dW   = pCpt->w != NL_NOW ? pCpt->w : 1.0 ;
           double    * dAdd = &adControlPoints[lUCnt*lCtrlPointUStride + lVCnt*lCtrlPointVStride];
           
           // case SM_CP_NON_RATIONAL
           if (eCtrlPointForm == SM_CP_NON_RATIONAL) 
             {
               if (bIsRational) 
                 {
                   dAdd[0] = dX / dW;
                   dAdd[1] = dY / dW;
                   dAdd[2] = dZ / dW;
                 }
               else 
                 {
                   dAdd[0] = dX;
                   dAdd[1] = dY;
                   dAdd[2] = dZ;
                 }
             } // end SM_CP_NON_RATIONAL

           // case SM_CP_HOMOGENEOUS_RATIONAL
           else if (eCtrlPointForm == SM_CP_HOMOGENEOUS_RATIONAL) 
             { 
               dAdd[0] = dX;
               dAdd[1] = dY;
               dAdd[2] = dZ;
               dAdd[3] = dW;
             } // end SM_CP_HOMOGENEOUS_RATIONAL branch

           // case SM_CP_EUCLIDIAN_RATIONAL
           else if (eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL) 
             {
               dAdd[0] = dX / dW;
               dAdd[1] = dY / dW;
               dAdd[2] = dZ / dW;
               dAdd[3] = dW;
             } // end SM_CP_EUCLIDIAN_RATIONAL

           lVCnt ++;
        } // end iter iV

      lUCnt ++;
    } // end iter iU

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::GetControlPointsExpert

  /*******************************************************************//**
PURPOSE: Get the degree of a surface in either the U or V parametric
    direction.

NOTES: 
***********************************************************************/
ULONG SmBSplineSurface::GetDegree
  (SmSurfParamType eSurfParam) 
 const
{
    gw_SURFACE *pSurface = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
		if (!pSurface) {
			return 0; // FS
		}
    if (eSurfParam == SM_SP_U) return pSurface->p;
    else return pSurface->q;

} // end SmBSplineSurface::GetDegree

/*******************************************************************//**
PURPOSE: Get a list of the unique knots and optionally knot multiplicities
    of one of the parameters of a BSpline Surface.

NOTES: This is the STEP compatible form of the knots not the
    typical knot vector associated with NURBS.
***********************************************************************/
SmStatus SmBSplineSurface::GetKnots
  (SmSurfParamType    eSurfParam,           // in : SM_SP_U = get U Knot Vector
                                            //      SM_SP_V = get V knot Vector
   SmTArray<double> & rUniqueKnots,         // out: unique knot values in requested dimension 
   SmTArray<ULONG>  * pKnotMultiplicities,  // out: associated multiplicity for every knot
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL]
  const
{    
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();
	if (!pSur) {
		return SM_ERR; // FS
	}
  if (eSurfParam == SM_SP_U) 
       {
         SER(sm_GetKnots(pSur->knu,rUniqueKnots,pKnotMultiplicities,pOptIvl));
       }
  else {
         SER(sm_GetKnots(pSur->knv,rUniqueKnots,pKnotMultiplicities,pOptIvl));
       }
  return SM_SUCCESS;

} // end SmBSplineSurface::GetKnots

/*******************************************************************//**
PURPOSE: Return true when any of the polyNet col or row
  vertices has a repeated pair of end ControlPoints.  

NOTES: Repeated control points are allowed on singular 
  boundary curves and are not reported as a duplicate pair.

***********************************************************************/
SmBoolean SmBSplineSurface::HasKnotMultiplicityGreaterThanDegree() const
{
  // get Nurb representation
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();

  // pass the call along
  return ( sm_HasKnotMultiplicityGreaterThanDegree(pSur, (SmBSplineSurface *)this) ) ;

} // end SmBSplineSurface::HasKnotMultiplicityGreaterThanDegree

/*******************************************************************//**
PURPOSE: Return true when any of the polyNet col or row
  vertices has a repeated pair of end ControlPoints.  

NOTES: Repeated control points are allowed on singular 
  boundary curves and are not reported as a duplicate pair.

***********************************************************************/
SmBoolean SmBSplineSurface::HasRepeatedEndControlPoints
  (double dTol)     // in : min distance between distinct points, default:[SM_ZONE_TOL_3D=1.0e-5]
                    //      when less than or equal to 0.0 reset to SM_ZONE_TOL_3D
 const
{
  // get Nurb representation
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();

  // pass the call along
  return ( sm_HasRepeatedEndControlPoints(dTol, pSur, (SmBSplineSurface *)this) ) ;

} // end SmBSplineSurface::HasRepeatedEndControlPoints

/*******************************************************************//**
PURPOSE: Return true when enough internal neighboring control
         points are coincident to create an internal pole 
    (i.e. has a subdomain of neighboring internal DomainPoints that 
          map to a single ImageSpacePoint, 
     i.e. a singularity where all domain points within some internal subdomain 
          of the surface maps to a single 3d location.)

NOTES: Internal poles are not allowed in SMLib.  
       Some of the many problems caused by internal poles include
       1. 1st derivatives values run to zero causing (among other problems)
          NewtonRaphson based sample and walk algorithms to fail.
       2. surface normals, tangents, and curvatures within the domain region
          of the pole are not defined.
       3. for surfaces - creates a location within the surface where it's
          possible for the surface parameterization orienation to flip
          creating a surface with positive surface normals on both
          of its 3d image space sides.
          
METHOD NOTES: An internal pole happens whenever any internal subset of the uv domain
  has a degenerate mapping, i.e. all points of some sub-domain of the surface's
  natural domain map to a single image space point.

  An internal pole certainly happens when all the control points for any single patch 
  within the surface are coincident making an entire sub-domain patch degenerate.  
  However, a pole also happens when just enough of the control points 
  within a single patch are coincident to make some isoCurve within the patch degenerate.  
  For BSplines the only case where a patch can have a degenerate isocurve
  when the patch itself is not degenerate is when the degenerate isocurve
  runs along the patch's boundaries. Those patch boundaries are formed by 
  the spans taken from the surface's constant knot-u and knot-v IsoParameterCurves. 

  In this implementation, the set of control points (a sub-array of the 
  total array of control points) for each patch boundary span is checked 
  for coincidence in sequence.  The method returns TRUE after finding the 
  first such span whose set of control points are all coincident with
  one another.
***********************************************************************/
SmBoolean SmBSplineSurface::HasInternalPole
  (double dTol)     // in : min distance between distinct points, default:[SM_ZONE_TOL_3D=1.0e-5]
                    //      when less than or equal to 0.0 reset to SM_ZONE_TOL_3D
 const
{
  // init return
  SmBoolean bRtn = FALSE ;

  // get Nurb representation
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();

  // no work - no BSplineSurface representation
  if(pSur == NULL)
    { return FALSE ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      Dump_NSrf(pSur, false) ;
    }
#endif // SM_DEBUG_CODE

  // locals
  ULONG    ii, jj ;
  NL_INDEX kk, ll ;
  NL_DEGREE iDegreeU = pSur->p ;
  NL_DEGREE iDegreeV = pSur->q ;
  SmPoint3d sPt, sPtNext ; 
  SmTArray<ULONG>   sMultsU,       sMultsV ;
  SmTArray<double>  sUniqueKnotsU, sUniqueKnotsV ;
  GetKnots(SM_SP_U, sUniqueKnotsU, &sMultsU) ;
  GetKnots(SM_SP_V, sUniqueKnotsV, &sMultsV) ;
  ULONG lKntCntU, lKntCntV ;

  // pick tolerance
  double dTolerance = dTol > 0.0 ? dTol : SM_ZONE_TOL_3D ;

  // first walk U Knot vector looking for poles in the V direction along the u = UMin patch boundary
  lKntCntU = sMultsU[0] ;
  for(ii=1;ii+1<sUniqueKnotsU.GetSize() && bRtn == FALSE;ii++) // note: can't say sUniqueKnotsU.GetSize()-1
    {
      lKntCntU += sMultsU[ii] ;

      // get the controlPoint index for the 1st control point whose basis function
      // at U = sUniqueKnots[ii]
      // one additional basis function becomes zero for each multiplicity at sMults[ii]
      NL_INDEX lRowCntU  = iDegreeU - (sMultsU[ii] - 1) ;
      NL_INDEX lCPIndexU = lKntCntU - lRowCntU - 1 ;

      // to get a pole - all the control points with non-zero basis funtions 
      //  for the U = PatchUMin boundary within one patch must be coincident
      //  That will include the lRowCntU constant_ii rows starting at UIndex = lCPIndexU
      //  and the iDegreeV constant_jj rows starting VIndex = lCPIndexV

      // for every V direction knot span - seek patch arrays of coincident control points
      for(jj=0,lKntCntV=0;jj+1<sUniqueKnotsV.GetSize() && bRtn == FALSE;jj++)// note: can't say sUniqueKnotsV.GetSize()-1
        {
          lKntCntV += sMultsV[jj] ;

          // the the controlpoint jj index for the 1st control point whose basis function
          // is nonZero over interval V = [sUniqueKnotsV[jj], sUniqueKnotsV[jj+1]
          NL_INDEX lCPIndexV = (lKntCntV - 1) - iDegreeV ;

          TO_EUCLID(pSur->net->Pw[lCPIndexU][lCPIndexV], sPt) ;

          SmBoolean bTest = TRUE ;

          // for every UDir neighbor ControlPoint - check for coincidence
          for(kk=lRowCntU-1;kk>=0 && bTest==TRUE;kk--)
            {
              for(ll=iDegreeV;ll>0 && bTest==TRUE;ll--)
                {
                  // check neighbor ControlPoint for Coincidence
                  TO_EUCLID(pSur->net->Pw[lCPIndexU+kk][lCPIndexV+ll], sPtNext) ;
                  bTest &= sPt.CloserThan(dTolerance, sPtNext) ;

                } // end iter ll, every V direction ControlPoint
            } // end iter kk, every U direction ControlPoint

#ifdef SM_DEBUG_CODE
          if(bTest)
            { if(bDebugMe)
                { Dump_NSrf(pSur, false) ; }
            }
#endif // SM_DEBUG_CODE

          // These tests don't guarantee a singularity: check the point.  [B597]
          if ( bTest )
          {
              SmPoint2d sUV;  // Get uv for the first control point tested.
              sUV.x = this->GetGrevilleAbscissa( SM_SP_U, lCPIndexU+lRowCntU-1 );
              sUV.y = this->GetGrevilleAbscissa( SM_SP_V, lCPIndexV+iDegreeV   );
              SmSurfParamType eSingDir;
              if ( ! this->IsSingularity( sUV, eSingDir, dTolerance ) )
                { bTest = FALSE; }
          }

          // remember when patch is degenerate
          bRtn |= bTest ;
        
        } // end iter jj, all V direction knot spans
    } // end iter ii, all U direction knot spans

  // second walk V Knot vector looking for poles in the U direction along the v = VMin patch boundary
  lKntCntV = sMultsV[0] ;
  for(ii=1;ii+1<sUniqueKnotsV.GetSize() && bRtn == FALSE;ii++) // note: can't say sUniqueKnotsV.GetSize()-1
    {
      lKntCntV += sMultsV[ii] ;

      // get the controlPoint index for the 1st control point whose basis function
      // at V = sUniqueKnotsV[ii]
      // one additional basis function becomes zero for each multiplicity at sMultsV[ii]
      NL_INDEX lRowCntV  = iDegreeV - (sMultsV[ii] - 1) ;
      NL_INDEX lCPIndexV = lKntCntV - lRowCntV - 1 ;

      // to get a pole - all the control points with non-zero basis funtions 
      //  for the V = PatchVMin boundary within one patch must be coincident
      //  That will include the lRowCntV constant_jj rows starting at VIndex = lCPIndexV
      //  and the iDegreeU constant_ii rows starting UIndex = lCPIndexU

      // for every U direction knot span - seek patch arrays of coincident control points
      for(jj=0,lKntCntU=0;jj+1<sUniqueKnotsU.GetSize() && bRtn == FALSE;jj++) // note: can't say sUniqueKnotsU.GetSize()-1
        {
          lKntCntU += sMultsU[jj] ;

          // the the controlpoint jj index for the 1st control point whose basis function
          // is nonZero over interval U = [sUniqueKnotsU[jj], sUniqueKnotsU[jj+1]
          NL_INDEX lCPIndexU = (lKntCntU - 1) - iDegreeU ;

          TO_EUCLID(pSur->net->Pw[lCPIndexU][lCPIndexV], sPt) ;

          SmBoolean bTest = TRUE ;

          // for every VDir neighbor ControlPoint - check for coincidence
          for(kk=lRowCntV-1;kk>=0 && bTest==TRUE;kk--)
            {
              for(ll=iDegreeU;ll>0 && bTest==TRUE;ll--)
                {
                  // check neighbor ControlPoint for Coincidence
                  TO_EUCLID(pSur->net->Pw[lCPIndexU+ll][lCPIndexV+kk], sPtNext) ;
                  bTest &= sPt.CloserThan(dTolerance, sPtNext) ;

                } // end iter ll, every U direction ControlPoint
            } // end iter kk, every V direction ControlPoint

          // remember when patch is degenerate
#ifdef SM_DEBUG_CODE
          if(bTest)
            { if(bDebugMe)
                { Dump_NSrf(pSur, false) ; }
            }
#endif // SM_DEBUG_CODE

          bRtn |= bTest ;
        
        } // end iter jj, all U direction knot spans
    } // end iter ii, all V direction knot spans

//        // first walk U Knot vector looking for poles in the V direction along the u = UMin patch boundary
//        lKntCntU = sMultsU[0] ;
//        for(ii=1;ii+1<sUniqueKnotsU.GetSize() && bRtn == FALSE;ii++) // note: can't say sUniqueKnotsU.GetSize()-1
//          {
//            lKntCntU += sMultsU[ii] ;
//      
//            // get the controlPoint index for the 1st control point whose basis function
//            // at U = sUniqueKnots[ii]
//            // one additional basis function becomes zero for each multiplicity at sMults[ii]
//            NL_INDEX lRowCntU  = iDegreeU - (sMultsU[ii] - 1) ;
//            NL_INDEX lCPIndexU = lKntCntU - lRowCntU ;
//      
//            // to get a pole - all the control points with non-zero basis funtions 
//            //  for the U = PatchUMin boundary within one patch must be coincident
//            //  That will include the lRowCntU constant_ii rows starting at UIndex = lCPIndexU
//            //  and the iDegreeV constant_jj rows starting VIndex = lCPIndexV
//      
//            // for every V direction knot span - seek patch arrays of coincident control points
//            for(jj=0,lKntCntV=0;jj<sUniqueKnotsV.GetSize()-1 && bRtn == FALSE;jj++)
//              {
//                lKntCntV += sMultsV[jj] ;
//      
//                // the the controlpoint jj index for the 1st control point whose basis function
//                // is nonZero over interval V = [sUniqueKnotsV[jj], sUniqueKnotsV[jj+1]
//                NL_INDEX lCPIndexV = (lKntCntV - 1) - iDegreeV ;
//      
//                TO_EUCLID(pSur->net->Pw[lCPIndexU][lCPIndexV], sPt) ;
//      
//                SmBoolean bTest = TRUE ;
//      
//                // for every UDir neighbor ControlPoint - check for coincidence
//                for(kk=lRowCntU-1;kk>=0 && bTest==TRUE;kk--)
//                  {
//                    for(ll=iDegreeV-1;ll>=0 && bTest==TRUE;ll--)
//                      {
//                        // check neighbor ControlPoint for Coincidence
//                        TO_EUCLID(pSur->net->Pw[lCPIndexU+kk][lCPIndexV+ll], sPtNext) ;
//                        bTest &= sPt.CloserThan(dTolerance, sPtNext) ;
//      
//                      } // end iter ll, every V direction ControlPoint
//                  } // end iter kk, every U direction ControlPoint
//      
//      #ifdef SM_DEBUG_CODE
//                if(bTest)
//                  { if(bDebugMe)
//                      { Dump_NSrf(pSur, false) ; }
//                  }
//      #endif // SM_DEBUG_CODE
//      
//                // remember when patch is degenerate
//                bRtn |= bTest ;
//              
//              } // end iter jj, all V direction knot spans
//          } // end iter ii, all U direction knot spans
//      
//        // second walk V Knot vector looking for poles in the U direction along the v = VMin patch boundary
//        lKntCntV = sMultsV[0] ;
//        for(ii=1;ii<sUniqueKnotsV.GetSize()-1 && bRtn == FALSE;ii++)
//          {
//            lKntCntV += sMultsV[ii] ;
//      
//            // get the controlPoint index for the 1st control point whose basis function
//            // at V = sUniqueKnotsV[ii]
//            // one additional basis function becomes zero for each multiplicity at sMultsV[ii]
//            NL_INDEX lRowCntV  = iDegreeV - (sMultsV[ii] - 1) ;
//            NL_INDEX lCPIndexV = (lKntCntV - 1) - lRowCntV ;
//      
//            // to get a pole - all the control points with non-zero basis funtions 
//            //  for the V = PatchVMin boundary within one patch must be coincident
//            //  That will include the lRowCntV constant_jj rows starting at VIndex = lCPIndexV
//            //  and the iDegreeU constant_ii rows starting UIndex = lCPIndexU
//      
//            // for every U direction knot span - seek patch arrays of coincident control points
//            for(jj=0,lKntCntU=0;jj<sUniqueKnotsU.GetSize()-1 && bRtn == FALSE;jj++)
//              {
//                lKntCntU += sMultsU[jj] ;
//      
//                // the the controlpoint jj index for the 1st control point whose basis function
//                // is nonZero over interval U = [sUniqueKnotsU[jj], sUniqueKnotsU[jj+1]
//                NL_INDEX lCPIndexU = lKntCntU - iDegreeU ;
//      
//                TO_EUCLID(pSur->net->Pw[lCPIndexU][lCPIndexV], sPt) ;
//      
//                SmBoolean bTest = TRUE ;
//      
//                // for every VDir neighbor ControlPoint - check for coincidence
//                for(kk=lRowCntV-1;kk>=0 && bTest==TRUE;kk--)
//                  {
//                    for(ll=iDegreeU-1;ll>=0 && bTest==TRUE;ll--)
//                      {
//                        // check neighbor ControlPoint for Coincidence
//                        TO_EUCLID(pSur->net->Pw[lCPIndexU+ll][lCPIndexV+kk], sPtNext) ;
//                        bTest &= sPt.CloserThan(dTolerance, sPtNext) ;
//      
//                      } // end iter ll, every U direction ControlPoint
//                  } // end iter kk, every V direction ControlPoint
//      
//                // remember when patch is degenerate
//      #ifdef SM_DEBUG_CODE
//                if(bTest)
//                  { if(bDebugMe)
//                      { Dump_NSrf(pSur, false) ; }
//                  }
//      #endif // SM_DEBUG_CODE
//      
//                bRtn |= bTest ;
//              
//              } // end iter jj, all U direction knot spans
//          } // end iter ii, all V direction knot spans

  // all done
  return bRtn ;

} // end SmBSplineSurface::HasInternalPole

/*******************************************************************//**
PURPOSE: Return true when any single internal row or column of control points
         are all coincident.

NOTES: Sometimes an entire internal row or column of coincident control points
       forces the surface to have an actual internal pole (when the control points are
       associated with a fully multiple knot).  However, in general, that
       is not a sufficient condition to guarantee an internal pole,
       (when the associated knot isn't fully multiple, neighboring rows or columns
        also have to be coincident to ensure an actual internal pole.)
       However, in practice we have found that surfaces with degenerate 
       rows or columns (multiple control points are coincident to a single space point)
       tend to behave very much like they have an internal pole causing 
       subsequent SMLib failures.  This predicate checks for the condition even 
       though such surfaces are not illegal unless they have an actual internal 
       pole.  To check for that condition, see: SmBSplineSurface::HasInternalPole().
       
NOTES: Internal poles are not allowed in SMLib.  
       Some of the many problems caused by internal poles include
       1. 1st derivatives values run to zero causing (among other problems)
          NewtonRaphson based sample and walk algorithms to fail.
       2. surface normals, tangents, and curvatures within the domain region
          of the pole are not defined.
       3. for surfaces - creates a location within the surface where it's
          possible for the surface parameterization orienation to flip
          creating a surface with positive surface normals on both
          of its 3d image space sides.
          
***********************************************************************/
SmBoolean SmBSplineSurface::HasDegenerateControlPointRow
  (double dTol)     // in : min distance between distinct points, default:[SM_ZONE_TOL_3D=1.0e-5]
                    //      when less than or equal to 0.0 reset to SM_ZONE_TOL_3D
 const
{

  // get Nurb representation
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer();

  // no work - no BSplineSurface representation
  if(pSur == NULL)
    { return FALSE ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      Dump_NSrf(pSur, false) ;
    }
#endif // SM_DEBUG_CODE

  // pick tolerance
  double dTolerance = dTol > 0.0 ? dTol : SM_ZONE_TOL_3D ;

  // locals
  NL_INDEX ii, jj ;
  NL_INDEX   iN       = pSur->net->n ; // max 1st index in PW
  NL_INDEX   iM       = pSur->net->m ; // max 2nd index in PW
  SmPoint3d  sPt, sPtNext ; 
  SmExtent3d sRowBBox ;
  SmBoolean  bDegen = TRUE ;

  // for every internal row - check for degeneracy
  for(ii=1;ii<iN && bDegen == TRUE;ii++)
    {
      // get row's first control point
      TO_EUCLID(pSur->net->Pw[ii][0], sPt) ;
          
      // reset the RowBBox with the first control point in the current row
      bDegen = TRUE ;
      sRowBBox.Init() ;
      sRowBBox.AddPoint3d(sPt) ;

      // for every other Control Point in the row
      for(jj=1;jj<=iM && bDegen == TRUE;jj++)
        {
          TO_EUCLID(pSur->net->Pw[ii][jj], sPtNext) ;

          // accumulate row control points BBox
          sRowBBox.AddPoint3d(sPtNext) ;

          // quit when BBox is too big
          if(sRowBBox.GetMaxDimension() > dTolerance)
            { bDegen = FALSE ; }

        } // end iter jj, every row control point

      // exit case: found a degeneratee row
      if(bDegen == TRUE)
        {
          return(bDegen) ; 
        }
    } // end iter ii every internal row

  // for every internal column - check for degeneracy
  for(jj=1;jj<iM && bDegen == TRUE;jj++)
    {
      // get row's first control point
      TO_EUCLID(pSur->net->Pw[0][jj], sPt) ;
          
      // reset the RowBBox with the first control point in the current column
      bDegen = TRUE ;
      sRowBBox.Init() ;
      sRowBBox.AddPoint3d(sPt) ;

      // for every other Control Point in the column
      for(ii=1;ii<=iN && bDegen == TRUE;ii++)
        {
          TO_EUCLID(pSur->net->Pw[ii][jj], sPtNext) ;

          // accumulate row control points BBox
          sRowBBox.AddPoint3d(sPtNext) ;

          // quit when BBox is too big
          if(sRowBBox.GetMaxDimension() > dTolerance)
            { bDegen = FALSE ; }

        } // end iter jj, every column control point

      // exit case: found a degeneratee column
      if(bDegen == TRUE)
        {
          return(bDegen) ; 
        }
    } // end iter ii every internal column

  // arrive here without degenerate rows or columns
  return(FALSE) ;
 
} // end SmBSplineSurface::HasDegenerateControlPointRow

/*******************************************************************//**
PURPOSE: Separate duplicate end control points on nonDegenerate row or col
   curves of the control polyhedron.
   returns TRUE when a surface has been modified.  

NOTES: Used to heal one kind of bad BSplineSurface data.
  If a nonDegenerate curve within a surface polyhedron has a 
  duplicate pair of control points at either of its ends,
  it moves the 2nd control point between the 1st and 3rd control points.
  The amount of the move is set by the parameter, SM_FR_NORMALIZED_PARAM
    which is used by both the curve and surface version of this function
    so that their behaviors will always match.
  This modifies the parameterization of the some some, but (mostly) preserves
  the shape of the control polygon.
***********************************************************************/
SmBoolean SmBSplineSurface::FixRepeatedEndControlPoints()
{
  // init return value
  SmBoolean bMadeChanges = FALSE ;

  // get Nurb representation
  gw_SURFACE *pSur = GetGwNurbPointer();

  // no work - no BSplineSurface representation
  if(pSur == NULL)
    { return bMadeChanges ; }

  // locals
  ULONG ii, jj, jMax = 0, kk, jRowLength = 0;
  NL_INDEX i0 = 0, i1 = 0, i2 = 0, id = 0;
  NL_INDEX j0 = 0, j1 = 0, j2 = 0, jd = 0;
  NL_INDEX iN = pSur->net->n ; // max 1st index in PW
  NL_INDEX iM = pSur->net->m ; // max 2nd indes in PW
  double dScaledZero = SM_EFF_ZERO ;

  // rules: 1. singularity: a boundary edge of the conrol             
  //           polyhedron may be degenerate.                          
  //        2. Any nonDegenerate boundary edge or any internal        
  //           row or col SubCurve of the control polyhedral that     
  //           has its end control points repeated will be          
  //           modified.                                              
                                                                      

  // for every edge of the surface polyhedron
  for(ii=0;ii<4;ii++)
    { // +--------+-----------------+------+------------------+--------------------+                         
      // |   ii   |  walk edge      | jMax | i0   i1   i2  id |  j1   j1   j2  jd  |        
      // +--------+-----------------+------+------------------+--------------------+                         
      // | ii = 0 | walk i, j = 0   |  iN  |  0    0    0   1 |   0    1    2   0  | 
      // | ii = 1 | walk i, j = iM  |  iN  |  0    0    0   1 |  iM  iM-1 iM-2  0  | 
      // | ii = 2 | walk j, i = 0   |  iM  |  0    1    2   0 |   0    0    0   1  | 
      // | ii = 3 | walk j, i = iN  |  iM  |  iN iN-1 iN-2  0 |   0    0    0   1  | 
      // +--------+-----------------+------+------------------+--------------------+  
      switch(ii)
        {
          // set iterators and samplers to walk the various 4 edges of the polyhedron
          case 0 : { jMax = iN ;   jRowLength = iM ;
                     id = 1 ;      jd = 0 ;
                     i0 = 0 ;      j0 = 0 ;             
                     i1 = 0 ;      j1 = 1 ;             
                     i2 = 0 ;      j2 = 2 ;             
                   } break ;                            
          case 1 : { jMax = iN ;   jRowLength = iM ;
                     id = 1 ;      jd = 0 ;                
                     i0 = 0 ;      j0 = iM ;               
                     i1 = 0 ;      j1 = iM-1 ;
                     i2 = 0 ;      j2 = iM-2 ;
                   } break ;                               
          case 2 : { jMax = iM ;   jRowLength = iN ;       
                     id = 0 ;      jd = 1 ;
                     i0 = 0 ;      j0 = 0 ;
                     i1 = 1 ;      j1 = 0 ;
                     i2 = 2 ;      j2 = 0 ;
                   } break ;                               
          case 3 : { jMax = iM ;   jRowLength = iN ;       
                     id = 0 ;      jd = 1 ;
                     i0 = iN ;     j0 = 0 ;
                     i1 = iN-1 ;   j1 = 0 ;
                     i2 = iN-2 ;   j2 = 0 ;
                   } break ;
        } // end switch to set up iterators and samplers
      
      // skip walking sequence of net curves when curves have less than 3 control points
      if(jRowLength < 2)
        { continue ; }

      // walk every net curve that start/ends on this boundary - increment all the indices
      for(jj=0; jj<=jMax; jj++, 
                          i0+=id, i1+=id, i2+=id,
                          j0+=jd, j1+=jd, j2+=jd)
        {
          // ignore repeated control points on singular boundaries - they are supposed to be repeated
          SmBoolean bDegenerate = FALSE ;
          if(jj == 0 || jj == jMax)
            {
              // check net curve for degeneracy
              SmExtent3d sBBox ;

              // to walk the boundary curve in question - use the same indices trick as before
              //      +---------------+--------------------+
              //      |ii == (0 || 1) |  ki = jj   kid = 0 |
              //      |               |  kj = 0    kjd = 1 |
              //      +---------------+--------------------+
              //      |ii == (2 || 3) |  ki = 0    kid = 1 |
              //      |               |  kj = jj   kjd = 0 |
              //      +---------------+--------------------+

              // set up indices to walk the boundary curve in question
              NL_INDEX ki, kid, kj, kjd ;
              if(ii == 0 || ii == 1 ) { ki = jj ; kid = 0 ;
                                        kj = 0 ;  kjd = 1 ;
                                      }
              else                    { ki = 0  ; kid = 1 ;
                                        kj = jj ; kjd = 0 ;
                                      }
              // for every control point in this surface net curve
              for(kk=0;kk<=jRowLength;kk++,
                                      ki+=kid,
                                      kj+=kjd)
                {
                  // add points to the bouding box
                  sBBox.AddPoint3d(*((SmVector3d *)(&pSur->net->Pw[ki][kj]))) ;

                  // most bounding boxes will be nonDegenerate - make one stab at being quick
                  if( kk==2 )
                    {
                      // check size of box
                      dScaledZero = SM_EFF_ZERO * (1.0 + sBBox.GetMaxDimension()) ;
                      bDegenerate = sBBox.IsPointSized(dScaledZero) ;

                      if(bDegenerate == FALSE)
                        {
                          // don't need to look any further
                          break ; 
                        } 
                    } // end quicky check to skip obviously nonDegenerate curves
                } // end bounding box size

              // make sure the bbox hasn't grown since the 1st check on its size
              if(bDegenerate)
                {
                  dScaledZero = SM_EFF_ZERO * (1.0 + sBBox.GetMaxDimension()) ;
                  bDegenerate = sBBox.IsPointSized(dScaledZero) ;
                }
             
            } // end is this a boundary netCurve check and setting bDegenerate

          // skip singular boundary edges
          if(bDegenerate)
            { continue ; }

          // check ControlPoint pair coincidence
          double dCPtDist ;

          // scale tolerance
          dScaledZero = SM_EFF_ZERO * (1.0 + ((SmVector3d *)(&pSur->net->Pw[i0][j0]))->GetMaxDimension()) ; 

          // get end CPt pair distance 
          N_DistCptCpt( pSur->net->Pw[i0][j0], pSur->net->Pw[i1][j1],  &dCPtDist) ;

          // remember if it's a repeat
          SmBoolean bRepeat = SM_IS_ZERO_TO_TOL(dCPtDist, dScaledZero) ;

          // when a repeated control point pair was found
          if(bRepeat)
            {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
              // draw before
              if(bDebugMe)
                {
                  this->Dump() ;

                  SmFace *pFace = (SmFace *)this->GetFace() ;
                  SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

                  smgfx_Erase() ;
                  smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
                  smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
                  smgfx_SetLook( 2, 3, 0, 0, 0 ); if(pFace) { pFace->Draw(); sm_GraphicsLoop(); }
                  sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE

              // let the system know the surface is about to be edited
              if(bMadeChanges == FALSE)
                {
                  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;
                }

              // remember the fix
              bMadeChanges = TRUE ;

              // move control point 1  between control points 0 and 2
              N_Combine2CPts( SM_FR_NORMALIZED_PARAM,    pSur->net->Pw[i0][j0], 
                             (1-SM_FR_NORMALIZED_PARAM), pSur->net->Pw[i2][j2], 
                            &(pSur->net->Pw[i1][j1]) );

            } // end found a repeated pair of control points check

        } // end iter every ControlPoint triple on this edge of the ControlNet
    } // end iter every edge of the ControlNet

  // when changes were made
  if(bMadeChanges)
    {
      // inform the public - all done with changes
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ;
    }

  // all done
  return(bMadeChanges) ;

} // end SmBSplineSurface::FixRepeatedEndControlPoints

/*******************************************************************//**
PURPOSE: Get the natural UV domain of the surface.  This is the domain
   which defines the minimum and maximum U and V parameters at which
   the surface can be evaluated.  

NOTES: Note that x corresponds to U and that y corresponds to 
   V in the 2D point form.

   If this surface has no underlying NURB surface, one will be created.
***********************************************************************/
SmExtent2d SmBSplineSurface::GetNaturalUVDomain
  () 
 const
{
  gw_SURFACE *pSur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer() ;
	if (!pSur) {
		return SmExtent2d(0, 0, 0, 0); // FS
	}

  if ( pSur == NULL )
    {
      WARN( _T("Error: SmBSplineSurface with no m_pNurb") )
      return SmExtent2d();
    }

  // return bounded surface min/max knot values
  //SmPoint2d sMinPt, sMaxPt;
  //sMinPt.x = pSur->knu->U[0];
  //sMaxPt.x = pSur->knu->U[pSur->knu->m];
  //sMinPt.y = pSur->knv->U[0];
  //sMaxPt.y = pSur->knv->U[pSur->knv->m];
  //return SmExtent2d(sMinPt,sMaxPt);
    return SmExtent2d(pSur->knu->U[0], pSur->knv->U[0], pSur->knu->U[pSur->knu->m], pSur->knv->U[pSur->knv->m]);

} // end SmBSplineSurface::GetNaturalUVDomain

/*******************************************************************//**
PURPOSE: Get the maximum allowable domain for an analytic representaion
   of this surface.

NOTES: 
   At the SmBSplineSurface level, this is just the Nurbs parameterization.

   For surfaces of degree 3 or higher, or even 2, this expansion
   can result in wildly pathological surface behavior even for the
   expansion factors used here.  Use this method with caution.
***********************************************************************/
SmExtent2d SmBSplineSurface::GetMaxAnalyticDomain() 
 const
{
  SmExtent2d sDom = GetNaturalUVDomain();

  if ( this->IsPlanar() )
    {
      sDom.ExpandRelative( 100.0 );
      return sDom;
   }


  // Expand u and v separately.  Depends on degree.

  SmExtent1d sUDom = sDom.GetUInterval();

  // Do not extend closed surfaces.
  if ( ! IsClosed( sDom, SM_SP_U ) )
    {
      ULONG lDeg = this->GetDegree( SM_SP_U );
      if      ( lDeg == 1 ) { sUDom.ExpandRelative( 100.0 ); }
      else if ( lDeg == 2 ) { sUDom.ExpandRelative(   1.0 ); }
      else                  { sUDom.ExpandRelative(   0.5 ); }
    }

  SmExtent1d sVDom = sDom.GetVInterval();
  if ( ! IsClosed( sDom, SM_SP_V ) )
    {
      ULONG lDeg = this->GetDegree( SM_SP_V );
      if      ( lDeg == 1 ) { sVDom.ExpandRelative( 100.0 ); }
      else if ( lDeg == 2 ) { sVDom.ExpandRelative(   1.0 ); }
      else                  { sVDom.ExpandRelative(   0.5 ); }
    }

  sDom.SetMinMax( sUDom.GetMin(), sVDom.GetMin(), sUDom.GetMax(), sVDom.GetMax() );

  return sDom;

} // end SmBSplineSurface::GetMaxAnalyticDomain

/*******************************************************************//*** 
PURPOSE: Insert a given number of knots.

NOTES: 
   - Never creates any multiple knots.
   - Inserts between existing knots, as evenly as possible.
   - If the optional interval is given:
     - Inserts all knots within that interval;
     - Inserts the interval's end points, if they are not
       already at knot values.
     - Assumes that that interval is contained in the natural domain,
       but does not check.
***********************************************************************/
SmStatus SmBSplineSurface::InsertKnots
(
  SmSurfParamType eSurfParam,     // in :
  int             iNumToInsert,   // in :
  SmExtent1d    * pOptInterval    // in :
)
{
  SmTArray< double > sAllKnots;
  this->GetKnots( eSurfParam, sAllKnots );

  SmExtent1d sSubInterval;
  if ( pOptInterval != NULL )
  {
      sSubInterval = *pOptInterval;
  }
  else
  {
      SmExtent2d sDomain = this->GetNaturalUVDomain();
      sSubInterval = ( eSurfParam == SM_SP_U )
          ? sDomain.GetUInterval()
          : sDomain.GetVInterval();
  }

  SmTArray< double > sKnotsToInsert;

  SmStatus eStat = smgu_InsertKnotsIntoKnotVector( iNumToInsert,
         sAllKnots, sSubInterval, sKnotsToInsert );

  if ( eStat != SM_SUCCESS ) {
      return eStat;
  }

  // Got the knots, now do the insertion.
  return this->InsertKnots( eSurfParam, sKnotsToInsert );

} // end SmBSplineSurface::InsertKnots( which, count, where )

/*******************************************************************//*** 
PURPOSE: Make the NURB surface representation - abstract method.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::MakeNurb
()
{
  if (m_pNurb == NULL) { SER(SM_ERR); }
  return SM_SUCCESS;

} // end SmBSplineSurface::MakeNurb

/*******************************************************************//**
PURPOSE: Given a type of validity check to perform, determine if the
    surface passes.

NOTES: 
***********************************************************************/
SmBoolean SmBSplineSurface::PassesValidityCheck
(
  SmValidityCheckType eChecks,          // in :
  SmValidityCheckType & reCheckFailed   // in :
)  const
{
    SM_ASSERT_VALID_NO_STREAM( this );

    reCheckFailed = SM_VC_NONE;
    
    // Check for validity of surface knots
    double dKData[256];
    SmTArray<double> sKnots(256,dKData);
    ULONG lMData[256];
    SmTArray<ULONG> sKnotMult(256,lMData);
    {
        SER(GetKnots(SM_SP_U,sKnots,&sKnotMult));
        ULONG lDeg = GetDegree(SM_SP_U);
        if (sKnotMult[0] > lDeg + 1) {
            reCheckFailed = SM_VC_DEFINITION;
            return FALSE;
        }
        if (sKnotMult.GetLast() > lDeg + 1) {
            reCheckFailed = SM_VC_DEFINITION;
            return FALSE;
        }
        for (ULONG mm=1; mm+1<sKnotMult.GetSize(); mm++) { // note: can't say sKnotMult.GetSize()-1
            if (sKnotMult[mm] > lDeg) {
                reCheckFailed = SM_VC_DEFINITION;
                return FALSE;
            }
        }
    }
    {
        SER(GetKnots(SM_SP_V,sKnots,&sKnotMult));
        ULONG lDeg = GetDegree(SM_SP_V);
        if (sKnotMult[0] > lDeg + 1) {
            reCheckFailed = SM_VC_DEFINITION;
            return FALSE;
        }
        if (sKnotMult.GetLast() > lDeg + 1) {
            reCheckFailed = SM_VC_DEFINITION;
            return FALSE;
        }
        for (ULONG mm=1; mm+1<sKnotMult.GetSize(); mm++) { // note: can't say sKnotMult.GetSize()-1
            if (sKnotMult[mm] > lDeg) {
                reCheckFailed = SM_VC_DEFINITION;
                return FALSE;
            }
        }
    }
    
    if (eChecks == SM_VC_ALL || eChecks == SM_VC_DEFINITION) {
        gw_FLAG err = N_SrfIsValid(((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer(),_T("SmBSplineSurface::PassesValidityCheck"));
        if (err) {
            reCheckFailed = SM_VC_DEFINITION;
            return FALSE;
        }
    }

    // It's OK for a surface to have a singularity
    // This should check for the surface being of zero length in one direction
    //SmSurfParamType eSingularDirection;
    //SmPoint2d sUVMid = GetNaturalUVDomain().Evaluate(0.5,0.5);
    //if (IsSingularity(sUVMid,eSingularDirection)) {
    //    reCheckFailed = SM_VC_SINGULARITY;
    //    return FALSE;
    //}

    return SmSurface::PassesValidityCheck(eChecks,reCheckFailed);

} // end SmBSplineSurface::PassesValidityCheck

/*******************************************************************//**
PURPOSE: Edit the surface by setting the canonical form of the 
   NURBS data.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::SetCanonical
(
  ULONG lUDegree,                                   // in :
  ULONG lVDegree,                                   // in :
  const SmTArray<SmPoint3d> & crControlPointsList,  // in :
  SmBSplineSurfaceForm eBSplineSurfaceForm,         // in :
  const SmTArray<ULONG> & crUKnotMultiplicities,    // in :
  const SmTArray<ULONG> & crVKnotMultiplicities,    // in :
  const SmTArray<double> & crUKnots,                // in :
  const SmTArray<double> & crVKnots,                // in :
  SmKnotType eKnotType,                             // in :
  const SmTArray<double> * cpOptWeights             // in :
) 
{
  // Create a temporary one
  SmBSplineSurface *pTmp = NULL ;
  const SmContext *pContext = GetContext();
  NER(pContext);
  SER(SmBSplineSurface::CreateCanonical(*pContext,
                                        lUDegree, lVDegree, crControlPointsList,eBSplineSurfaceForm,
                                        crUKnotMultiplicities, crVKnotMultiplicities, crUKnots, crVKnots,
                                        eKnotType, cpOptWeights, NULL, pTmp));

  SmObjDelete sCleanup(pTmp);

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Swap Nurbs with this
  gw_SURFACE *pTmpNurb = pTmp->m_pNurb;
  pTmp->m_pNurb        = m_pNurb;
  m_pNurb              = pTmpNurb;

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // give analytics a chance to update their nested STEP data
  RebuildSTEPFromNURBParameters() ;

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return SM_SUCCESS;

} // end SmBSplineSurface::SetCanonical

/*******************************************************************//**
PURPOSE: Edit the surface by setting it using the NLib NURBS
       to copy the data from.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::SetFromGwNurb
(
  ULONG,                        // in : Not Used
  gw_SURFACE * pGwNurbSurface
)  
{
  // tell the hierarchy we are about to edit the parameterization
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // free existing m_pNurb 
  if (m_pNurb && !m_bNurbIsBorrowed) { smos_Free(m_pNurb); m_pNurb = NULL ; }

  // let m_pNurb = copy(cpGwNurbSurface)
  m_pNurb = sm_AllocateAndCopyNurbSurface(pGwNurbSurface) ;
  m_bNurbIsBorrowed = FALSE;

  // tell hierarchy parameterization has been edtied
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // give analytics a chance to update their nested STEP data
  RebuildSTEPFromNURBParameters() ;

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return SM_SUCCESS;

} // end SmBSplineSurface::SetFromGwNurb

/*******************************************************************//**
PURPOSE: Make BsplineSurface exactly singular along the given Boundary
 
NOTES: 1. When pOptSingularPt != NULL: Move all control points along the target boundary
                                         to the specified point when given,
          else                         Move all control points along the target boundary
                                         to the centroid of their current locations.

       2. Rational Points are moved leaving their current weight values constant.

       3. Does not move control points on analytics
***********************************************************************/
SmStatus SmBSplineSurface::MakeSingular
 (ULONG       lSingularSide,   // in : target side: one of SM_SS_UMIN SM_SS_UMAX
                               //                          SM_SS_VMIN SM_SS_VMAX
  SmPoint3d * pOptSingularPt)  // in : Euclidean Point to become the singularity location
                               //      NULL = Set SingularPt = Centroid of Boundary ControlPts
                               //      default:[NULL]
{
  // prohibit control point moves on analytics
  if(GetType() != SmBSplineSurface_TYPE)
    {
      // You can not edit an analytical subclass of
      // a B-Spline using this method.  You must either make a copy
      // of only the B-Spline portion or edit using methods on the
      // subclass.
      SM_ASSERT(IsKindOf(SmBSplineSurface_TYPE)) ;
      ERR_MSG(_T("SmBSplineSurface::MakeSingular - Attempt to Set Control Point on an analytic surface")) ;
      return(SM_ERR) ;
    }

  // inform the public - shape will change
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
 
  // locals
  ULONG ii ;
  double       dWt ;  
  SmPoint3d    sCentroid(0,0,0), sPt ; 
  gw_SURFACE * pSur = GetOrCreateGwNurbPointer();
  gw_CNET    * net  = pSur->net;  
  ULONG        lN   = net->n ; // max 1st index in Pw 
  ULONG        lM   = net->m ; // max 2nd index in Pw
                               //  net->Pw = Pw sized:[n+1][m+1]
  // branch on lSingularSide
  switch(lSingularSide)
    {
      case SM_SS_UMAX :
      case SM_SS_UMIN : { ULONG idx = lSingularSide == SM_SS_UMIN ? 0 : lN ;
                          // get singularity location in sCentroid
                          if(pOptSingularPt) { sCentroid = *pOptSingularPt ; }
                          else { for(ii=0;ii<=lM;ii++) { TO_EUCLID(net->Pw[idx][ii], sPt) ; 
                                                         sCentroid += sPt ;
                                                       }
                                 sCentroid /= (lM+1) ;
                               }

                          // set all boundary CPts to singularity location
                          for(ii=0;ii<=lM;ii++) 
                            { dWt = net->Pw[idx][ii].w ;
                              sPt = sCentroid ; 
                              if(dWt != NL_NOW) 
                                { sPt *= dWt ; }
                              net->Pw[idx][ii].x = sPt.x ;
                              net->Pw[idx][ii].y = sPt.y ;
                              if(net->Pw[idx][ii].z != NL_NOZ) { net->Pw[idx][ii].z = sPt.z ; } 
                            }
                        } // end cases SM_SS_UMIN and SM_SS_UMAX
                        break ; 

      case SM_SS_VMAX :
      case SM_SS_VMIN : { ULONG idx = lSingularSide == SM_SS_VMIN ? 0 : lM ;
                          if(pOptSingularPt) { sCentroid = *pOptSingularPt ; }
                          else { for(ii=0;ii<=lN;ii++) { TO_EUCLID(net->Pw[ii][idx], sPt) ; 
                                                         sCentroid += sPt ;
                                                       }
                                 sCentroid /= (lN+1.0) ;
                               }
                          for(ii=0;ii<=lN;ii++) 
                            { dWt = net->Pw[ii][idx].w ;
                              sPt = sCentroid ; 
                              if(dWt != NL_NOW) 
                                { sPt *= dWt ; }
                              net->Pw[ii][idx].x = sPt.x ;
                              net->Pw[ii][idx].y = sPt.y ;
                              if(net->Pw[ii][idx].z != NL_NOZ) { net->Pw[ii][idx].z = sPt.z ; } 
                            }
                        } // end cases SM_SS_UMIN and SM_SS_UMAX
                        break ; 

      default: ERR_MSG(_T("SmBSplineSurface::MakeSingular - given bad lSingularSide input")) ;
               return(SM_ERR) ;
    } // end switch on lSingularSide

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::MakeSingular

/*******************************************************************//**
PURPOSE: Edit the surface by setting one control point.
 
NOTES:
***********************************************************************/
SmStatus SmBSplineSurface::SetControlPoint
(
  SmControlPointFormType eCtrlPointForm,  // in : Either SM_CP_EUCLIDIAN_RATIONAL with a weight
                                          //      or SM_CP_NON_RATIONAL which ignores the weight
   ULONG                  lUIndex,        // in :
   ULONG                  lVIndex,        // in :
   const SmPoint3d      & crControlPoint, // in :
   double                 dWeight)        // in :
{
  // GWC prohibit control point moves on analytics
  if(GetType() != SmBSplineSurface_TYPE)
    {
      // You can not edit an analytical subclass of
      // a B-Spline using this method.  You must either make a copy
      // of only the B-Spline portion or edit using methods on the
      // subclass.
      SM_ASSERT(IsKindOf(SmBSplineSurface_TYPE)) ;
      ERR_MSG(_T("Attempt to Set Control Point on an analytic surface")) ;
      return(SM_ERR) ;
    }

  // locals
  SmPoint3d    sControlPoint = crControlPoint;
  gw_SURFACE * pSur = GetOrCreateGwNurbPointer();
  gw_CNET    * net  = pSur->net;

  // error check - out of bounds indices
  if(   lUIndex > (ULONG)net->n 
     || lVIndex > (ULONG)net->m) 
    { SER(SM_ERR) ; }

  // the target control point
  gw_CPOINT  * pCpt = &net->Pw[lUIndex][lVIndex]; 
 
  // convert ControlPts to homogeneous rational form
  if (eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL)
    {
      sControlPoint.x *= dWeight;
      sControlPoint.y *= dWeight;
      sControlPoint.z *= dWeight;

      // make sure all the other control points are also rational 
      for (ULONG i = 0 ; i <= (ULONG)net->n ; i++) 
        { 
          for (ULONG j = 0 ; j <= (ULONG)net->m ; j++)
            {
              gw_CPOINT *pCpti = &net->Pw[i][j];
              if (pCpti->w == NL_NOW) { net->Pw[i][j].w = 1.0; }
            }
        }
    } // end CtrlPtForm = Euclidean Rational check

  else if (eCtrlPointForm == SM_CP_NON_RATIONAL) 
    { dWeight = 1.0; }
 
  // inform the public - starting to change shape - gwc should this be a SM_NO_CHANGE_GEOMETRY
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Set the Control Point value
  pCpt->x = sControlPoint.x;
  pCpt->y = sControlPoint.y;
  if (pCpt->z != NL_NOZ) { pCpt->z = sControlPoint.z; }
  if ( IsRational() )    { pCpt->w = dWeight; }

  // inform the public - all done
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
 
  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::SetControlPoint

/*******************************************************************//**
PURPOSE: Set the BSplineSurface using data arrays.  This method
    should be flexible enough to handle most forms of NURBS and 
    Bezier surfaces which exist in various software products.  It is
    somewhat like the form used by OPENGL.  Please see the OPENGL
    description of gluNurbsSurface to better understand this method.

NOTES: This is an expert method which should only be used by
    those who are willing to take the risk.  
***********************************************************************/
SmStatus SmBSplineSurface::SetExpert
(
  ULONG lUDegree,                             // in :
  ULONG lVDegree,                             // in :
  SmBSplineSurfaceForm eBSplineSurfaceForm,   // in :
  SmEndKnotFormType eEndKnotForm,             // in :
  ULONG lUNumKnots,                           // in :
  const double *cadUKnots,                    // in :
  ULONG lVNumKnots,                           // in :
  const double *cadVKnots,                    // in :
  SmControlPointFormType eCtrlPointForm,      // in :
  ULONG lCtrlPointUStride,                    // in :
  ULONG lCtrlPointVStride,                    // in :
  const double *cadCtrlPoints                 // in :
)
{
  // GWC prohibit control point moves on analytics
  if(GetType() != SmBSplineSurface_TYPE)
    {
      // You can not edit an analytical subclass of
      // a B-Spline using this method.  You must either make a copy
      // of only the B-Spline portion or edit using methods on the
      // subclass.
      SM_ASSERT(IsKindOf(SmBSplineSurface_TYPE)) ;
      ERR_MSG(_T("Attempt to Set Control Point on an analytic surface")) ;
      return(SM_ERR) ;
    }

    if (eEndKnotForm == SM_EK_UNCLAMPPED) {
        lUNumKnots = lUNumKnots + 2;
        lVNumKnots = lVNumKnots + 2;
    }
    ULONG lUCtrlPointCount = lUNumKnots - lUDegree - 1;
    ULONG lVCtrlPointCount = lVNumKnots - lVDegree - 1;

    gw_SURFACE *pSur = sm_AllocateNurbSurface(lUCtrlPointCount-1, lVCtrlPointCount-1,
                                              (gw_DEGREE)lUDegree,(gw_DEGREE)lVDegree,
                                              lUNumKnots - 1, lVNumKnots - 1);

    gw_KNOTVECTOR *pKnu = pSur->knu;
    gw_KNOTVECTOR *pKnv = pSur->knv;
    gw_CNET *pNet = pSur->net;
    gw_CPOINT **Pw = pNet->Pw;

    ULONG lUIdx = 0;
    ULONG lVIdx = 0;
    if (eEndKnotForm == SM_EK_UNCLAMPPED) {
        pKnu->U[lUIdx++] = cadUKnots[0];
        pKnv->U[lVIdx++] = cadVKnots[0];
        lUNumKnots = lUNumKnots - 2;
        lVNumKnots = lVNumKnots - 2;
    }
    for (ULONG i=0; i<lUNumKnots; i++) {
        pKnu->U[lUIdx++] = cadUKnots[i];
    }
    for (ULONG ii=0; ii<lVNumKnots; ii++) {
        pKnv->U[lVIdx++] = cadVKnots[ii];
    }

    if (eEndKnotForm == SM_EK_UNCLAMPPED) {
        pKnu->U[pKnu->m] = cadUKnots[lUNumKnots-1];
        pKnv->U[pKnv->m] = cadVKnots[lVNumKnots-1];
    }

    for (ULONG jU=0; jU<lUCtrlPointCount; jU++) {
        for (ULONG jV=0; jV<lVCtrlPointCount; jV++) {
            const double *dAdd = &cadCtrlPoints[jU*lCtrlPointUStride+jV*lCtrlPointVStride];
            double dX = dAdd[0];
            double dY = dAdd[1];
            double dZ = dAdd[2];
            double dW = 1.0;
            if (eCtrlPointForm != SM_CP_NON_RATIONAL) {
                dW = dAdd[3];
            }
            else { // not rational
                dW = NL_NOW;
            }
            if (eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL) {
                // Put back into Euclidian form - suitable for
                // Create Canonical.
                dX = dX * dW;
                dY = dY * dW;
                dZ = dZ * dW;
            }
            Pw[jU][jV].x = dX;
            Pw[jU][jV].y = dY;
            Pw[jU][jV].z = dZ;
            Pw[jU][jV].w = dW;
        }
    }
    
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    m_eBSplineSurfaceForm = eBSplineSurfaceForm;
    m_bNurbIsBorrowed = FALSE;
    m_eKnotType = SM_KT_UNSPECIFIED;

    // Swap Nurbs with this
    if (m_pNurb) { smos_Free(m_pNurb); m_pNurb = NULL ; }
    m_pNurb = pSur;

    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    return SM_SUCCESS;

} // end SmBSplineSurface::SetExpert

/*******************************************************************//**
PURPOSE: Copy BSplineSurface subpatch to a new BSplineSurface

NOTES: 
  1. The boundaries of the requested UVDomain may be moved
     by tol (1.0e-8) to prevent the creation of very short
     knot spans.
***********************************************************************/
SmStatus SmBSplineSurface::CopySubPatch
 (const SmContext   & crContext,    // in : Context for new object construction
  const SmExtent2d  & crUVDomain,   // in : Domain of NewSurface (snapped to existing knots within 1.0e-10)
  SmBSplineSurface *& rpNewSurface) // out: NewSurface = ThisSurface over crUVDomain
 const
{
  // Init output
  rpNewSurface = NULL;

  // locals
  gw_SURFACE   sNewSur ;
  gw_SURFACE * pSur  = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer() ;
  NL_STACKS    SC;

  SmStackHandler sStackKp(&SC);

  // Trim crUVDomain to NaturalUVDomain
  SmExtent2d sUVDomain, sNatUVDomain = GetNaturalUVDomain() ;
  sNatUVDomain.Intersect(crUVDomain, sUVDomain) ;

  // init pSur data structure
  N_SrfInitArrays(&sNewSur);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump_NSrf(pSur, FALSE) ; 
    }
#endif // SM_DEBUG_CODE

  // pass the call along
  NL_FLAG iFlg = N_SrfExtractPatch(pSur, 
                                   sUVDomain.GetUMin(),
                                   sUVDomain.GetUMax(),
                                   sUVDomain.GetVMin(),
                                   sUVDomain.GetVMax(),
                                   & sNewSur,
                                   & SC,
                                   & SC) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      Dump_NSrf(pSur, FALSE) ; 
      Dump_NSrf(&sNewSur, FALSE) ; 
    }
#endif // SM_DEBUG_CODE

  // If that fails, return error: never want an SmBSpline with no Nurb.  [B429]
  if ( iFlg == NL_YES )
    { return SM_ERR; }

  // copy sNewSur back into a SmBSplineSurface object
  rpNewSurface = new (crContext) SmBSplineSurface(&sNewSur) ;

  // note: leftover sNewSur memory is freed when NL_STACKS SC goes out of scope

  // all done
  return(SM_SUCCESS) ;

} // end SmBSplineSurface::CopySubPatch

/*******************************************************************//**
PURPOSE: Build two new child BSpline Surfaces
            by spliting a B-Spline surface at a given parameter.

NOTES: 
  1. The original 'this' BSplineSurface is not modified

  2. dParam gets refined; it snaps up to dTol distance to existing knots where
        dTol = SM_EFF_ZERO*100.0*(1.0+smos_Fabs(dParam));

  3. Output surface domains depend on eSurfParam as
     
     eSurfParam == SM_SP_U, LeftSurface_uDomain = [u_min, t], RightSurface_uDomain = [t, u_max]
     eSurfParam == SM_SP_V, LeftSurface_vDomain = [v_min, t], RightSurface_vDomain = [t, v_max]

***********************************************************************/
SmStatus SmBSplineSurface::SplitAt
  (const SmContext & crContext,         // in : context for new object construction
   double            dParam,            // in : split parameter
   SmSurfParamType   eSurfParam,        // in : oneof SM_SP_U = split u domain at dParam
                                        //            SM_SP_V = split v domain at dParam
   SmSurface      *& rpLeftSurface,     // out: SmBSplineSurface (left  or bot)
   SmSurface      *& rpRightSurface)    // out: SmBSplineSurface (right or top)
{
  // init output
  rpLeftSurface = rpRightSurface = NULL;

  double dTol = SM_EFF_ZERO*100.0*(1.0+smos_Fabs(dParam));

  // If we are close to a knot (i.e. 1.0e-10) then split at the
  // knot.  Otherwise we could get a very bad surface.

  // snap knot to nearby knot values
  double dData[32];
  SmTArray<double> sKnots(32,dData);
  GetKnots(eSurfParam,sKnots);
  for (ULONG i=0; i<sKnots.GetSize(); i++) 
    {
      double dKnot = sKnots[i];
      if (smos_Fabs(dKnot - dParam) < dTol) 
        {
          dParam = dKnot;
        }
    } // end iter every knot

  // locals
  gw_SURFACE * pSurL = NULL;
  gw_SURFACE * pSurR = NULL;
  gw_SURFACE * pSur  = GetOrCreateGwNurbPointer();
  gw_FLAG      gwDir = SM_SURFPARAM_TO_NLDIR(eSurfParam) ;

  // split the gw_SURFACE object - allocate and set memory for both pSurL and pSurR
  if(NL_YES == sm_SplitSrf(pSur,    // in : tgt surface
                           dParam,  // in : split param
                           gwDir,   // in : SplitDir: NL_UDIR=Split KnotVectorU, use - SM_SURFPARAM_TO_NLDIR(eSurfParam)
                                    //                NL_VDir=Split KnotVectorV        to convert SmSurfParamType to NL_DIR types
                           pSurL,   // out: Split surface result, Ivl=[MinParam, TgtParam]
                           pSurR))  // out: Split surface result, Ivl=[TgtParam, MaxParam]
    { ERR_MSG(_T("ERROR returned from NLib\n")) ; }

  // GW_ERR(sm_SplitSrf(pSur,dParam,gwDir,pSurL,pSurR));

  if ( pSurL == NULL || pSurR == NULL )
    {
      if(pSurL != NULL) { delete pSurL; pSurL = NULL; }
      if(pSurR != NULL) { delete pSurR; pSurR = NULL; }
      SER( SM_ERR );
    }

  // build BSplineSurfaces for each new gw_Surface object
  rpLeftSurface  = new (crContext) SmBSplineSurface(pSurL,FALSE, &crContext) ;
  rpRightSurface = new (crContext) SmBSplineSurface(pSurR,FALSE, &crContext) ;

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::SplitAt

/*******************************************************************//**
PURPOSE: Given a point on the SmBSplineSurface, find its
    corresponding analytic UV-parameter.

NOTES: This method requires a point exactly on the surface.
    Therefore, users should be cautious when calling this method.

   This method should be instantiated for all derived analytic classes.
   When called at this level, the surface is not analytic, and the
   "analytic parameters" are the same as the Nurbs parameters.
   So this is basically just GlobalPointSolve().

   If the result is on the seam of a closed surface, and a guess was
   given, the output will be set to whichever side of the seam is closer
   to the guess.  If no guess was given, it will be set to the low end
   of the closed domain.

   If no solution is found, the function returns SM_ERR.
***********************************************************************/
SmStatus SmBSplineSurface::STEPInversion
  (const SmExtent2d & crAnalDomain,       // in : domain limit for successful inversions
   const SmPoint3d  & crPointOnSurf,      // in : Target Point - must be on Surface within Tolerance
   double             d3DTolerance,       // in : max allowed dist between point and surface
   SmPoint2d        & rdAnalUVParameter,  // out: Nurb Surface Point[U,V] for input point
   SmLocationType   & reLocation,         // out: oneof SM_LT_POLE,   
                                          //            SM_LT_U_SEAM, 
                                          //            SM_LT_V_SEAM, 
                                          //            SM_LT_UV_SEAM,
                                          //            SM_LT_INTERIOR
   SmPoint2d        * pUVGuess)           // in, opt: guess parameter.
  const
{
  // init output
  reLocation = SM_LT_INTERIOR;
  
  // intersect point with Surface
  SmSolution sSData[16];
  SmSolutionArray sSolutions(16,sSData);

  // If a guess is given, try a local solve first.
  SmBoolean bLocalSucceeded = FALSE;
  if ( pUVGuess != NULL )
    {
      SmSolution sSol;
      SER( LocalPointSolve( crAnalDomain, SM_SO_INTERSECT, crPointOnSurf, *pUVGuess,
                            bLocalSucceeded, sSol ));
      if ( bLocalSucceeded )
        { sSolutions.Add( sSol ); }
    }

  if ( ! bLocalSucceeded )
    {
      SER(GlobalPointSolve(crAnalDomain, SM_SO_INTERSECT, crPointOnSurf,
                           d3DTolerance, NULL,
                           SM_SR_ALL, sSolutions));
    }

  // Point not close enough to surface - error
  if (sSolutions.GetSize() == 0) 
    { return SM_ERR; }

  // set output UVParameters = Nurb parameters
  rdAnalUVParameter.x = sSolutions[0].m_vStart[0];
  rdAnalUVParameter.y = sSolutions[0].m_vStart[1];

  // when point is on a singular boundary 
  SmSurfParamType eSingDir;
  if (IsSingularity(rdAnalUVParameter,eSingDir)) 
    {
      // update the location value
      reLocation = SM_LT_POLE;
    }

  // done for interior and pole points
  if (   reLocation == SM_LT_POLE
      || sSolutions.GetSize() == 1) 
    { return SM_SUCCESS; }
  
  // when surface is closed in U direction and solution on Seam
  if (IsClosed(crAnalDomain,SM_SP_U)) 
    {
      SmExtent1d sIvl(crAnalDomain.GetMin().x,crAnalDomain.GetMax().x);
      if (sIvl.IsValueOnBoundary(rdAnalUVParameter.x)) 
        {
          // update the location value
          reLocation = SM_LT_U_SEAM;
        }
    }

  // when surface is closed in V direction and solution on Seam
  if (IsClosed(crAnalDomain,SM_SP_V)) 
    {
      SmExtent1d sIvl(crAnalDomain.GetMin().y,crAnalDomain.GetMax().y);
      if (sIvl.IsValueOnBoundary(rdAnalUVParameter.y)) 
        {
          // update the location value
          reLocation =   (reLocation == SM_LT_INTERIOR) 
                       ? SM_LT_V_SEAM
                       : SM_LT_UV_SEAM ;
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::STEPInversion

/*******************************************************************//**
PURPOSE: Swap the U and V parameterizations of a surface.  This 
    effectively reverses the orientation of the surface.  

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::SwapUV()
{
  // locals
  ULONG                lUDegree, lVDegree;
  SmTArray<SmPoint3d>  sCPts;
  SmTArray<double>     sUKnots, sVKnots, sWeights;
  SmTArray<ULONG>      sUMult, sVMult;
  SmKnotType           eKnotType;
  SmBSplineSurfaceForm eBSForm;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
      this->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // get current surface description
  SER(GetCanonical(lUDegree,lVDegree,sCPts,eBSForm,sUMult,sVMult,
                   sUKnots,sVKnots,eKnotType,sWeights));

  // let analytic surfaces remember that they have been swapped
  ToggleSwapUVBit() ;

  // Transpose Weights and control points arrays
  ULONG lNumVCpts = GetNumberControlPoints(SM_SP_V);
  sCPts.Transpose(lNumVCpts);
  sWeights.Transpose(lNumVCpts);
  
  // Transpose Surface definition
  SER(SetCanonical(lVDegree,lUDegree,sCPts,eBSForm,sVMult,sUMult,
                   sVKnots,sUKnots,eKnotType,&sWeights));

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
      this->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::SwapUV

/*******************************************************************//**
PURPOSE: Scale and transform a B-Spline surface.
   If scale is passed in apply it first then the rotation and
   move defined by the placement.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::Transform
  (const SmAxis2Placement & crRotateNMove, // in : affine rotate and move transformation      
   const SmVector3d       * cpOptScale)    // in : optional scaling about current origin point before RotateNMove
                                           //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                           //      other geom types only support isoptropic scaling
{
    SmVector3d sIdentityScale(1, 1, 1);
    
    // no work - identity transform
    if (crRotateNMove.IsIdentity() && (cpOptScale == NULL || *cpOptScale == sIdentityScale))
    {
        return SM_SUCCESS;
    }

    // SM_DUMP_AND_ASSERT2_VALID(this) ;  this can be called by analytics between atomic states
    const SmPoint3d  &rP = crRotateNMove.GetOriginRef();
    const SmVector3d &rX = crRotateNMove.GetXAxisRef();
    const SmVector3d &rY = crRotateNMove.GetYAxisRef();
    SmVector3d        sZ = crRotateNMove.GetZAxis();

    NL_STACKS SC;
    NL_RMATRIX rma;
    SmStackHandler sStackKp(&SC);
    N_InitRealMatrix(&rma);
    N_SetRealMatrix(&rma,3,3,full,3,&SC);
    gw_REAL  **RM = rma.RM;

    SmVector3d sScale(1,1,1);
    if (cpOptScale) sScale = *cpOptScale;

    RM[0][0] = rX.x * sScale.x;
    RM[0][1] = rY.x * sScale.y;
    RM[0][2] = sZ.x * sScale.z;

    RM[1][0] = rX.y * sScale.x;
    RM[1][1] = rY.y * sScale.y;
    RM[1][2] = sZ.y * sScale.z;

    RM[2][0] = rX.z * sScale.x;
    RM[2][1] = rY.z * sScale.y;
    RM[2][2] = sZ.z * sScale.z;

    RM[0][3] = rP.x;
    RM[1][3] = rP.y;
    RM[2][3] = rP.z;
    
    RM[3][0] = 0.0;
    RM[3][1] = 0.0;
    RM[3][2] = 0.0;
    RM[3][3] = 1.0;

    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    N_SrfTransform(GetOrCreateGwNurbPointer(),&rma);

    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    return SM_SUCCESS;

} // end SmBSplineSurface::Transform

/*******************************************************************//**
PURPOSE: Modify a B-Spline surface to corresponds to a new UV boundary

NOTES: 
   The original intent was that the new domain be a subset of the existing
   domain, i.e., do not try to extend the surface.  That has been relaxed. [B208]
   However, when used for extensions, either the extension should be very
   small or the surface should be planar.  Otherwise the resulting surface
   can get very bad where it's extended.

   The output geometry over the original domain will be coincident with the input geometry
   but the parameterization may vary slightly but by more than reasonably sized tolerances.

   Callers of this function should delete all UVTrimCurves associated with this surface.

   Note: Extending surfaces using AdjustSTEPUVDomain by large amounts for
         the derived analytic surfaces is just fine.  Those surfaces rebuild
         a new larger Nurb surface from the analytic definition
         which is very stable rather than through extension which
         is not stable.
***********************************************************************/
SmStatus SmBSplineSurface::AdjustSTEPUVDomain
  (const SmExtent2d & crNewSTEPUVDomain)
{
  // new domain has to be smaller
  SmExtent2d sUVDomain = GetNaturalUVDomain();

  // Grow the surface if indicated, then trim.
  // Note, both routines (expand and trim) will do no work if not needed.

  SmPoint2d sMinMax = crNewSTEPUVDomain.GetMin();
  SER( ExpandBoundarySpan( sMinMax.x, SM_SP_U ));
  SER( ExpandBoundarySpan( sMinMax.y, SM_SP_V ));

  sMinMax = crNewSTEPUVDomain.GetMax();
  SER( ExpandBoundarySpan( sMinMax.x, SM_SP_U ));
  SER( ExpandBoundarySpan( sMinMax.y, SM_SP_V ));

  // These would be redundant:
  // sUVDomain = GetNaturalUVDomain();
  // if (!crNewSTEPUVDomain.IsContainedBy(sUVDomain, SM_EFF_ZERO))
  //     SER(SM_ERR);

  sUVDomain = crNewSTEPUVDomain;

  // change the domain - note: makes coincident geometry but the parameterization may change
  SER( TrimWithDomain( sUVDomain ));

  return SM_SUCCESS;

} // end SmBSplineSurface::AdjustSTEPUVDomain

/*******************************************************************//**
PURPOSE: Test for a fillet surface in one direction
    and extract some useful information if we find one.

NOTES:
  input --
  eWhichDir             if SM_SP_U, then test for constant-U isocurves being
                        cross-sections (ribs) of a fillet.
                        Note, if a constant-U curve is a rib, then the rails
                        run in the U-direction (constant V curves).
  output --
  rbIsFilletSurface     is set to TRUE if this surface might be a fillet Surface based solely on its shape
  rlFilletCrossSection  is set to the crossSection shape type
  rlFilletSolverType    is set for constant Radius, constant Distance, or variable
  rbNormalIsOutward     is set TRUE If surface normal points toward outward part of surface

METHOD ---
  Test the shape of cross-section (rib) curves to see whether they all
  have the same fillet crossSection shape type.  Test the isoParameter
  curves at the natural Domain boundaries and at all the interior knot
  values.  If all isoParameter curves could be the same type of Fillet
  cross section, return that.
***********************************************************************/
SmStatus SmBSplineSurface::TestForFilletSurfaceUOrV
  (const SmContext & crContext,             // in : context for new object construction
   SmSurfParamType   eWhichDir,             // in : U- or V-direction, which cross sections to test.
   double            d3DTolerance,          // in : max allowed distance between this curve and a filletCrossSection Shape
   SmBoolean       & rbIsFilletSurface,     // out: TRUE = surface's naturalDomain isoParameter curves have Fillet CrossSection shapes
                                            //      FALSE= curves natural domain boundaries do not have fillet crossSection shapes
   ULONG           & rlFilletCrossSection,  // out: fillet crossSection type    0 - circular,
                                            //                                  1 - Approx Circular,
                                            //                                  2 - linear
                                            //                                  3 - G1,
                                            //                                  4 - G2,
                                            //                                  5 - G3
   ULONG           & rlFilletSolverType,    // out: 0 - Constant Radius
                                            //      1 - Constant Distance,
                                            //      2 - Variable Radius
   SmBoolean       & rbNormalIsOutward,     // out: TRUE = If surface normal points toward outward part of surface
   double          * pdRadius,              // out [optional; default NULL]: Radius, if constant radius
   double          * pdDist,                // out [optional; default NULL]: Cross-section distance, if constant distance
   SmExtent1d      * pMinMaxRadii )         // out [optional; default NULL]: Min and max radii, if variable radius
 const
{
  // init outputs
  rbIsFilletSurface    = FALSE;
  rlFilletCrossSection = 9999;
  rlFilletSolverType   = 9999;
  rbNormalIsOutward    = FALSE;
  if ( pdRadius != NULL ) { *pdRadius = -1; }
  if ( pdDist   != NULL ) { *pdDist   = -1; }
  if ( pMinMaxRadii != NULL ) { pMinMaxRadii->Init(); }

  // locals
  SmBSplineCurve *pRibMin=NULL, *pRibMax=NULL;
  double    dDist             = 0.0;
  double    dRadius           = 0.0;
  double    dBlendScale       = 1.0;
  SmBoolean bConstantDistance = TRUE;
  SmBoolean bConstantRadius   = TRUE;

  // 1. Check that min/max naturalDomain isoParmeter curves have fillet crossSection shapes

  // Construct temporary naturalDomain Iso boundaries for this direction: rib curves.
  SER( CreateIsoBoundaries( crContext, eWhichDir, SM_EFF_ZERO, pRibMin, pRibMax ));
  SmObjDelete sCleanMin( pRibMin );
  SmObjDelete sCleanMax( pRibMax );

  // skip min isoParameter curves that don't have a known filletCrossSection shape
  ULONG     lFilletCrossSection;
  SmBoolean bIsFilletCrossSection;
  SER( pRibMin->TestForFilletCrossSection( crContext, d3DTolerance,
       bIsFilletCrossSection, lFilletCrossSection, dDist, dRadius, dBlendScale ));
  if ( !bIsFilletCrossSection )
    { return SM_SUCCESS; }

  // skip max isoParameter curves that don't have a known filletCrossSection shape
  ULONG lFilletCrossSection2;
  double dDist2, dRadius2;
  SER( pRibMax->TestForFilletCrossSection(crContext, d3DTolerance,
       bIsFilletCrossSection, lFilletCrossSection2, dDist2, dRadius2, dBlendScale ));
  if ( !bIsFilletCrossSection )
    { return SM_SUCCESS; }

  // skip cases where min and max curves don't have same FilletCrossSection shapes
  if ( lFilletCrossSection != lFilletCrossSection2 )
    { return SM_SUCCESS; }

  // arrive here when min/max naturalDomain isoParameter curves have the same crossSection shapes

  // surface is not a ConstantDistance fillet when min/max curve beg/end Point chord distances vary
  if (smos_Fabs(dDist-dDist2) > d3DTolerance)
    { bConstantDistance = FALSE; }

  // surface is not a ConstantRadius fillet when min/max curve beg/end Fillet radii vary
  if (   dRadius == 0.0
      || smos_Fabs(dRadius-dRadius2) > d3DTolerance)
    {
      bConstantRadius = FALSE;
    }

  // Test at every knot in rib direction.
  SmTArray<double> sKnots;
  GetKnots( eWhichDir, sKnots );

  // for every knot value
  SmBoolean bGoodCrossSections = TRUE;
  for ( ULONG k=1; k+1<sKnots.GetSize(); k++ ) // note: can't say sKnots.GetSize()-1
    {
      // create fillet crossSection direction isoParameter curve
      SmBSplineCurve *pRib = NULL ;
      SER( CreateIsoParametricCurve( crContext, 
                                     eWhichDir, 
                                     sKnots[k], 
                                     SM_EFF_ZERO, 
                                     pRib ));
      SmObjDelete sCleanIso(pRib);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) {
          pRib->Dump();
      }
#endif
      // test the internal isoParameter for a fillet CrossSection shape
      SER( pRib->TestForFilletCrossSection( crContext, d3DTolerance,
           bIsFilletCrossSection, lFilletCrossSection2, dDist2, dRadius2, dBlendScale ));

      // quit when any crossSection fails to be the same kind of fillet CrossSection shape as the minBoundary
      if (!bIsFilletCrossSection)                        { bGoodCrossSections = FALSE; break; }
      if (lFilletCrossSection2 != lFilletCrossSection)   { bGoodCrossSections = FALSE; break; }

      // watch for varying distance and radius blends
      if (smos_Fabs(dDist-dDist2) > d3DTolerance)        { bConstantDistance = FALSE; }
      if (   dRadius == 0.0
          || smos_Fabs(dRadius-dRadius2) > d3DTolerance) { bConstantRadius = FALSE; }

    } // end iter every internal knot isoParameter curve

  if ( !bGoodCrossSections )
    { return SM_SUCCESS; }

  // Here we have a possible fillet - set outputs and return.

  // Set rbNormalIsOutward.
  // For that we need surface normal and filletCrossSection 2nd derivative.
  // The cross-section 2nd derivative points 'inward',
  // so normal is outward if it's in the opposite direction.
  SmVector3d sNormal;
  SER( EvaluateNormal( GetNaturalUVDomain().GetMin(), TRUE, TRUE, sNormal ));
  SmPoint3d sPVV[3];
  SER( pRibMin->Evaluate( pRibMin->GetNaturalInterval().GetMin(), 2, TRUE, sPVV ));

  rbNormalIsOutward =  ( sNormal.Dot(sPVV[2]) < 0.0 )
                       ? TRUE
                       : FALSE ;

  // set other outputs and return
  rlFilletSolverType =   (bConstantRadius)   ? 0    // Constant Radius
                       : (bConstantDistance) ? 1    // Constant Distance
                       :                       2 ;  // variable radius or unknown
  rbIsFilletSurface     = TRUE;
  rlFilletCrossSection  = lFilletCrossSection;

  if ( pdRadius != NULL && bConstantRadius   ) { *pdRadius = dRadius; }
  if ( pdDist   != NULL && bConstantDistance ) { *pdDist   = dDist;   }

  // Check for variable radius.
  if ( pMinMaxRadii != NULL && ! bConstantRadius )
    {
      this->FindFilletMinMaxRadii( eWhichDir, *pMinMaxRadii );
    }

  return SM_SUCCESS;

} // end SmBSplineSurface::TestForFilletSurfaceUOrV

/*******************************************************************//**
PURPOSE: Test for a fillet surface in either direction
    and extract some useful information if we find one.

NOTES:
  output --
  rbIsFilletSurface     is set to TRUE if this surface might be a fillet Surface based solely on its shape
  rlFilletCrossSection  is set to the crossSection shape type
  rlFilletSolverType    is set for constant Radius, constant Distance, or variable
  reFilletRailDirection is set to direction in which the rail curves run.
                        e.g. SM_SP_U - ribs are constant U curves (run in V-direction).
                        So if a constant-U curve is a rib, then the rails
                        run in the U-direction (constant V curves).
  rbNormalIsOutward     is set TRUE If surface normal points toward outward part of surface

LIMITATIONS ---
  With no topology information, it's impossible to be definitive.
  (Even with topology info, it's still ambiguous and very difficult.)
  For example, if a surface is linear in one direction and circular
  in the other, it's probably an ordinary circular fillet on a straight
  edge.  But it could also be a chamfer around a cylinder.
  So we are forced to use heuristics.  (We can't even tell whether
  surfaces meet tangentially.)
  In particular, if both directions could be fillets, we will just say
  that in whichever direction the surface is bigger, is the rail direction.

METHOD ---
  Call the overloaded method TestForFilletSurface in both U- and V-directions
  and see whether either or both could be a fillet surface.
  If both, then decide which, as above.
***********************************************************************/
SmStatus SmBSplineSurface::TestForFilletSurface
  (const SmContext & crContext,             // in : context for new object construction
   double            d3DTolerance,          // in : max allowed distance between this curve and a filletCrossSection Shape
   SmBoolean       & rbIsFilletSurface,     // out: TRUE = surface's naturalDomain isoParameter curves have Fillet CrossSection shapes
                                            //      FALSE= curves natural domain boundaries do not have fillet crossSection shapes
   ULONG           & rlFilletCrossSection,  // out: fillet crossSection type    0 - circular,
                                            //                                  1 - Approx Circular,
                                            //                                  2 - linear
                                            //                                  3 - G1, 
                                            //                                  4 - G2, 
                                            //                                  5 - G3
   ULONG           & rlFilletSolverType,    // out: 0 - Constant Radius
                                            //      1 - Constant Distance,  
                                            //      2 - Variable Radius
   SmSurfParamType & reFilletRibDirection,  // out: Direction in which ribs are isoparms
                                            //      SM_SP_U - ribs are constant U curves
   SmBoolean       & rbNormalIsOutward,     // out: TRUE = If surface normal points toward outward part of surface
   double          * pdRadius,              // out: Radius, if constant radius
   double          * pdDist,                // out: Cross-section distance, if constant distance
   SmExtent1d      * pMinMaxRadii )         // out [optional; default NULL]: Min and max radii, if variable radius
 const
{
  // init output
  rbIsFilletSurface    = FALSE;
  rlFilletCrossSection = 9999;
  rlFilletSolverType   = 9999;
  rbNormalIsOutward    = FALSE;

  SmBoolean  bIsFilletU,     bIsFilletV;
  ULONG      lCrossSectionU, lCrossSectionV;
  ULONG      lSolverTypeU,   lSolverTypeV;
  SmBoolean  bNormOutwardU,  bNormOutwardV;
  double     dRadiusU,       dRadiusV;
  double     dDistU,         dDistV;
  SmExtent1d sRadiiU,       sRadiiV;
  SmExtent1d *pRadiiU=NULL;
  SmExtent1d *pRadiiV=NULL;
  if ( pMinMaxRadii != NULL )
    {
      pRadiiU = &sRadiiU;
      pRadiiV = &sRadiiV;
    }

  // Test in both U- and V- directions.
  TestForFilletSurfaceUOrV( crContext, SM_SP_U, d3DTolerance, bIsFilletU,
      lCrossSectionU, lSolverTypeU, bNormOutwardU, &dRadiusU, &dDistU, pRadiiU );

  TestForFilletSurfaceUOrV( crContext, SM_SP_V, d3DTolerance, bIsFilletV,
      lCrossSectionV, lSolverTypeV, bNormOutwardV, &dRadiusV, &dDistV, pRadiiV );

  // Now sort out which direction.
  // If neither, it's easy.
  if ( !bIsFilletU && !bIsFilletV )
    { return SM_SUCCESS; }

  // Ok, at least one direction is a Fillet cross section.
  rbIsFilletSurface = TRUE;

  // If only one, then it's still easy.
  if ( !bIsFilletU )
    {
      reFilletRibDirection = SM_SP_V;
    }
  else if ( !bIsFilletV )
    {
      reFilletRibDirection = SM_SP_U;
    }
  else
    {
      // Both directions: we have to decide which is better.
      // See 'Limitations' in the header comments.
      SmPoint2d sUVSize;
      SER( ApproximateSize( sUVSize ));
      // Cross section will be the shorter one.
      reFilletRibDirection = ( sUVSize.x < sUVSize.y ) ? SM_SP_V : SM_SP_U;
    }

  // All done. Set outputs and return.
  if ( reFilletRibDirection == SM_SP_U )
    {
      // U-direction is cross section.
      rlFilletCrossSection = lCrossSectionU;
      rlFilletSolverType   = lSolverTypeU;
      rbNormalIsOutward    = bNormOutwardU;
      if ( pdRadius     != NULL ) { *pdRadius     = dRadiusU; }
      if ( pdDist       != NULL ) { *pdDist       = dDistU;   }
      if ( pMinMaxRadii != NULL ) { *pMinMaxRadii = sRadiiU;   }
    }
  else
    {
      // V-direction is cross section.
      rlFilletCrossSection = lCrossSectionV;
      rlFilletSolverType   = lSolverTypeV;
      rbNormalIsOutward    = bNormOutwardV;
      if ( pdRadius     != NULL ) { *pdRadius     = dRadiusV; }
      if ( pdDist       != NULL ) { *pdDist       = dDistV;   }
      if ( pMinMaxRadii != NULL ) { *pMinMaxRadii = sRadiiV;   }
    }

  return SM_SUCCESS;

} // end SmBSplineSurface::TestForFilletSurface

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmBSplineSurface::IsKindOf( SM_TYPE t ) const
{
  return ((SmBSplineSurface_TYPE == t) ? TRUE : SmSurface::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump preceded by a one line message

NOTES: 
***********************************************************************/
void SmBSplineSurface::Dump
  (const TCHAR * message) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%s "), message);
  smos_WriteBuffer(sBuff);
  this->SmBSplineSurface::Dump(FALSE);

} // end SmBSplineSurface::Dump

/*******************************************************************//**
PURPOSE: Dump preceded by an integer (ULONG)

NOTES: 
***********************************************************************/
void SmBSplineSurface::Dump
  (ULONG i) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%ld "),i);
  smos_WriteBuffer(sBuff);
  this->SmBSplineSurface::Dump(FALSE);

} // end SmBSplineSurface::Dump

/*******************************************************************//**
PURPOSE: Dump bspline surface data out for debugging.
            Dump is in similar sequence to Write to file

NOTES: 
***********************************************************************/
void SmBSplineSurface::Dump
  (void) 
const
{
  this->SmBSplineSurface::Dump(FALSE);
} // end SmBSplineSurface::Dump

/*******************************************************************//**
PURPOSE: Dump with abbreviated option

NOTES: if bAbbrev = TRUE just dump 1st and last control point
         and knots
Note: Control points are converted to Euclidean for printout, if Rational
***********************************************************************/
void SmBSplineSurface::Dump
  (SmBoolean bAbbrev) 
 const
{
  // side effect: make an n_pNurb when there is none.
  // SM_ASSERT_VALID_NO_STREAM( this , SM_NO_OBJDUMP, SM_NO_STREAM, SM_LEVEL_1, SM_WALK) ;

  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  smos_WriteBuffer(_T("\nBegin SmBSplineSurface::Dump()")) ;

  // report Cache data
  SmSurface::Dump(FALSE) ;

  // BSplineSurface Pointer
  smos_sprintf(sBuff,_T("\nSmBSplineSurface = 0x%p "),this);
  smos_sprintf(sBuffForFile,_T("\nSmBSplineSurface = %s "),_T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  gw_SURFACE *pSur = GetGwNurbPointer();    
  Dump_NSrf(pSur, bAbbrev) ;
  smos_WriteBuffer(_T(" End SmBSplineSurface::Dump()\n")) ;

} // end SmBSplineSurface::Dump

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmBSplineSurface.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmBSplineSurface::GetMemoryUsed   // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,        // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)                // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + m_pNurb memory
  rlMemoryAllocated = sizeof(*this) + sm_ComputeNurbSurfaceSize(m_pNurb) ;

  // This + attribute memory + m_pNurb memory
  ULONG lThisAllocated ;
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                                       eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated = rlMemoryAllocated + lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }

  // all done
  return(lUsed) ;

} // end SmBSplineSurface::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertBSplineSurface_list[] =
{
  /*  0 */ {SM_AT_POINTER,          _T("Bad NULL Ptr"),            _T("SmBSplineSurface m_pNurb ptr is NULL") },
  /*  1 */ {SM_AT_PARAMETERIZATION, _T("Bad Knot Multiplicity"),   _T("SmBSplineSurface Knot multiplicity greater than surface degree") },
  /*  2 */ {SM_AT_DEGENERATE,       _T("Bad Internal Pole"),       _T("SmBSplineSurface has an Internal pole caused by combinations of coincident CPts") },
  /*  3 */ {SM_AT_DEGENERATE,       _T("Bad Degnerate CPt Row"),   _T("SmBSplineSurface has an internal row or column of all coincident control points") },
  /*  4 */ {SM_AT_DEGENERATE,       _T("Warn: Repeated End Cpts"), _T("SmBSplineSurface has a repeated end CPt row. Causes bdry cross-tangents to degenerate to zero - warning") },
  /*  5 */ {SM_AT_GEOMETRIC,        _T("Bad NonMonotonic CPts"),   _T("SmBSplineSurface Control Net doubles back on itself probably making a self intersecting surface") },
  /*  6 */ // already in SmSurface::AssertValid
           // {SM_AT_GEOMETRIC,        _T("Bad Colinear Tangent Pt"), _T("SmBSplineSurface has a illegal and degenerate UVCorner whose U and V 1st Derivatives are Colinear") } },
} ;

/*******************************************************************//**
PURPOSE: Check the validity of an SmBSplineSurface.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmBSplineSurface::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // check base class validity
  SmBoolean bRtn = TRUE ;
  
  // call the base class AssertValid
  //  currently checks for : FlatCorners
  //                         Surface Degenerates to a Point
  //                         Surface Degenerates to a Curve
  //                         Surface has an interior discontinuity
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmSurface::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // build m_pNurb when not available
  if (m_pNurb == NULL) 
    {
      SmBSplineSurface *pBSS = SM_CONST_CAST(SmBSplineSurface*,this);
      pBSS->MakeNurb();
    }

  /*  0 */ // m_pNurb can't be non-NULL"
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pNurb != NULL ), _T("") ) ;

  /*  1 */ // Knot multiplicity too high
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (!HasKnotMultiplicityGreaterThanDegree()), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T(""));

  // pick tolerance for coincident CPt checks
  // GWC - should use the pSurface->Face->Tolerance value but that's unavailable
  //       for now - just hard code in a reasonable value
  double  dTol = SM_ZONE_TOL_3D/10.0 ; // gwc: need a real tolerance here - this one is just made up.

  // when surface is not degenerate - check for other problem coincident control point conditions
  SmBoolean bDegenPoint = IsDegeneratePoint() ;
  SmBoolean bDegenCurve = FALSE;
  SmBoolean bInternalPole = FALSE;
  SmBoolean bDegenRow = FALSE;

  if(bDegenPoint == FALSE)
    {
      bDegenCurve = IsDegenerateCurve() ;
      if(bDegenCurve == FALSE)
        {
          /*  2 */ // Surfaces may not have internal poles caused by sets of coincident CPts
          bInternalPole = HasInternalPole(dTol) ;
          bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, bInternalPole == FALSE, dTol, SM_UNDEF_DOUBLE, _T("")) ;

          // when the surface has no internal poles - check for a row or column of coincident points
          if(bInternalPole == FALSE)
            {
              /*  3 */ // Surface has an internal degenerate row or column of control points
              bDegenRow = HasDegenerateControlPointRow(dTol) ; 

              // GWC - Internal poles are illegal and when the knot associated with a degenerate row
              //       of control points is fully multiple, a degenerate internal row yields an internal pole.
              //       However, when the associated knot is not fully multiple it is possible
              //       to have a surface with a degenreate interal cpt row without an internal pole.
              //       As such, this check could be issued as a warning.  However, I'm assuming
              //       any surface with a degenerate row was intended to have an internal pole
              //       and as such is going to be flagged as degenerate.
              bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, bDegenRow == FALSE, dTol, SM_UNDEF_DOUBLE, _T("")) ;
            
            } // end surface has no internal poles check

          // check on surface's with repeated endControlPoints
          /*  4 */ //Surfaces with repeated end CPt rows have bdry cross-tangents that degenerate to zero - warning
                   // not yet an error - just a warning
          SmBoolean bHasRepeatedCPts = HasRepeatedEndControlPoints() ;
          if(bHasRepeatedCPts)
            {
#ifdef SM_DEBUG_CODE // cull repeated notifications

static constexpr ULONG                       sListLength = 6;
static       ULONG                       lFoundItem ;
static SmTArray<const SmSurface *> sBadSurfaceList ;

              if(!sBadSurfaceList.FindElement(this, lFoundItem))
                {
                  // WARN(_T("BSplineSurface ControlNet has a row or col with repeated end ControlPoints - not an error, but not good practice.\n")) ;
                  if(sBadSurfaceList.GetSize() == sListLength) { sBadSurfaceList.RemoveLast() ; }
                  sBadSurfaceList.InsertAt(0, this) ; 
                }

#else  // no SM_DEBUG_CODE
              WARN(_T("BSplineSurface ControlNet has a row or col with repeated end ControlPoints - not an error, but not good practice.\n")) ;
#endif // no SM_DEBUG_CODE
      
            } // end surface has repeated cpts check
        } // end surface has not degenerated to a curve check
    } // surface has not degenerated to a point check

  /*  5 */ // when surface is not degenerate - check for Planar Control Net that doubles back on itself
  if(   !bDegenPoint   && !bDegenCurve 
     && !bInternalPole && !bDegenRow)
    {
      ULONG               lUCnt, lVCnt ;
      SmTArray<SmPoint3d> sCPointArray ;
      SmTArray<double>    sWeightArray ;
      SmExtent2d          sUVDomain  = GetNaturalUVDomain() ;
      SmBoolean           bClosedRow = IsClosed(sUVDomain,  SM_SP_U, &dTol) ;
      SmBoolean           bClosedCol = IsClosed(sUVDomain,  SM_SP_V, &dTol) ;
      GetControlPointNet(lUCnt, lVCnt, sCPointArray, sWeightArray) ;

      // place Control Point Grid into SmPointGrid object
      SmPointGrid         sCptGrid(sCPointArray,  // in : array copied or converted into PointGrid                                    
                                   lVCnt,         // in : number of points in each row of the grid
                                   dTol*1000,     // in : min dist between distinct points                                    
                                   bClosedRow,    // in : TRUE = 1st and last U point in each Column are connected        
                                   bClosedCol,    // in : TRUE = 1st and last V point in each Row are connected        
                                   UNSURE,        // in : TRUE = PointSequence is known to lie on a plane                          
                                   NULL,          // in : NULL=system computes m_sNormal, NotNULL=user given vector as Normal
                                   FALSE) ;       // in : TRUE=Copy sPoints into a new array - sPoints and this m_pData arrays are different
                                                  //      FALSE=Share sPoints with new array - sPoints and this m_pData arrays are same
                                                  //      default:[TRUE] 
      // for planar ControlPointMeshes
      // gwc: let's try this on all surfaces - not just planar ones and see if we get any false positives.
      // gwc note 2: What we really want is to determine when a surface has some kind of internal singularity
      //              as caused by folding a surface back on itself or by twisting a surface into a bow tie.
      //              Those kinds of bad surfaces will have internal points where the Surface tangents
      //              fail to span a plane.  Either because the 1stDeriv speed drops to zero or because
      //              the TangentU and TangentV vectors both point in the same direction.
      //              What we really want is a test that checks for
      //              min(Cross(TangentU,TangentV)) goes to zero, or
      //              min(Cross(1stDerivU, 1stDerivV)) goes to zero.
      //              That'll catch bizarre control point positions that cause the surface to collapse upon itself
      //              like ridges where coincident control points create zero derivatives in the surface,
      //                   smoothed corners when TangentU=TangentV at BSpline Surface corners where users
      //                     try to map the natural boundaries of a single BSPline Surface to a circle
      //                   folds where control points have been pulled back into the net to cause the surface to
      //                     fold back upon itself
      //                   bow-ties where the surface twists through itself at a point (a kind of fold)
      //               Near Poles the 1st derivatives drop to zero but that's not a problem.
      //                   The test will have to allow that.  Far from poles zero 1st derivates are illegal.
      //
      //  if(sCptGrid.IsPlanar())
      //    {
          /*  5 */ // check for Planar Control Net that doubles back on itself
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, FALSE == (sCptGrid.HasNonMonotonicPoints()), _T("") ) ;
      //    }
    } // end not degenerate surface check
    
#ifdef SM_DEBUG_CODE // cull repeated notifications 
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      SmFace * pFace = (SmFace *)GetFace() ; 
      SmBrep * pBrep = pFace ? pFace->GetBrep() : NULL ; 

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; 
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

    // already in SmSurface::AssertValid
    //   {
    //     /*  6 */ // see if SmBSplineSurface has an illegal and degenerate UVCorner whose U and V 1st Derivatives are Colinear 
    //     // check for Surface corners whose TangentU and TangentV have been made to be colinear
    //     //    this is illegal since there is no surface normal at the point but done to remove corners.
    //     //    The surface should be made with corners and then the face made with trim curves to represent
    //     //    a face without corners.
    //     SmPoint3d sPt00, sPt01, sPt10, sPt11 ;
    //     SmPoint3d sDU00, sDU01, sDU10, sDU11 ;
    //     SmPoint3d sDV00, sDV01, sDV10, sDV11 ;
    //     SmExtent2d sUVDomain = GetNaturalUVDomain() ;
    //   
    //     // evaluate tangents at surface corners
    //     Evaluate1stDerivatives(sUVDomain.Evaluate(0.0, 0.0), TRUE,  TRUE,  sPt00, sDU00, sDV00) ;
    //     Evaluate1stDerivatives(sUVDomain.Evaluate(0.0, 1.0), TRUE,  FALSE, sPt01, sDU01, sDV01) ;
    //     Evaluate1stDerivatives(sUVDomain.Evaluate(1.0, 0.0), FALSE, TRUE,  sPt10, sDU10, sDV10) ;
    //     Evaluate1stDerivatives(sUVDomain.Evaluate(1.0, 1.0), FALSE, FALSE, sPt11, sDU11, sDV11) ;
    //   
    //     // complain when corner tangents are within tol of being colinear 
    //     bRtn &= SM_ASSERT_VALUE_REPORT(6, SM_LEVEL_0, (   (FALSE == sDU00.IsParallelTo(sDV00, SM_ANG_TOL_DEG))
    //                                                    && (FALSE == sDU01.IsParallelTo(sDV01, SM_ANG_TOL_DEG))
    //                                                    && (FALSE == sDU10.IsParallelTo(sDV10, SM_ANG_TOL_DEG))
    //                                                    && (FALSE == sDU11.IsParallelTo(sDV11, SM_ANG_TOL_DEG))), 
    //                                                    SM_ANG_TOL_DEG, SM_UNDEF_DOUBLE, _T("")) ;
    //   } // end check 6 - dependent corners

  // all done 
  return(bRtn) ;

} // end SmBSplineSurface::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmBSplineSurface::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmBSplineSurface::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmBSplineSurface::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: For Analytic Surfaces that use GenCurves,
            Test GenCurve's Anal and NURB Domain compatibility with
            this surface's appropriate AnalUVDomain Intervals.

RETURNS --- TRUE when GenCurve parameterization matches up appropriately 
            with Surface parameterization, else return FALSE.

NOTES:
  returns TRUE when:
    Curve's analytic domain matches the surface's analytic domain as
       OtherCurve->GetSTEPInterval()    ==   eAnalyticDirection == SM_SP_U 
                                           ? GetSTEPUVDomain().GetUInternval
                                           : GetSTEPUVDomain().GetVInternval
    and Curve's NURB domain and knot vector matches the surface's NURB domain as
       OtherCurve->GetNaturalInterval() ==   eNurbDirection == SM_SP_U 
                                           ? GetNaturalUVDomain().GetUInternval
                                           : GetNaturalUVDomain().GetVInternval
    && OtherCurve->KnotVector ==   eNurbDirection == SM_SP_U 
                                 ? Surface->KnotVectorU
                                 : Surface->KnotVectorV
***********************************************************************/
SmBoolean SmBSplineSurface::HasSameParameterization
  (SmSurfParamType        eAnalyticDirection, // in : Check this direction's surf AnalyDomain against the curve's 
   SmSurfParamType        eNurbDirection,     // in : Check this direction's surf Nurb Knot Vector against the curve's
                                              //      The two directions are usually the same except for
                                              //      derived surface types with an m_bSwapUV flag value which is TRUE.
   const SmBSplineCurve * cpOtherCurve)       // in : target curve
  const                   
{
  const SmBSplineSurface *cpThisSurface = this ;

  // surface and Curve analytic parameterization locals
  SmExtent1d sSurfaceAnalDomain =   eAnalyticDirection == SM_SP_U 
                                  ? GetSTEPUVDomain().GetUInterval()  
                                  : GetSTEPUVDomain().GetVInterval() ;
  SmExtent1d sCurveAnalDomain   = cpOtherCurve->GetSTEPInterval() ;

  // surface and curve Nurb parameterization locals
  SmTArray<double> sSurfaceKnots, sCurveKnots ;           
  SmTArray<ULONG>  sSurfaceMults, sCurveMults ;
  cpThisSurface->GetKnots(eNurbDirection, sSurfaceKnots, &sSurfaceMults) ;  
  cpOtherCurve->GetKnots(sCurveKnots, &sCurveMults) ;

  // The surface and genCurve analytic domains must be compatible
  double dTol = SM_EFF_ZERO * ( 1.0 + smos_Max(sSurfaceAnalDomain.GetMaxDimension(), 
                                               sCurveAnalDomain.GetMaxDimension()  ));
  SmBoolean bRtn = (   sSurfaceAnalDomain.IsContainedBy(sCurveAnalDomain, dTol)
                    && sCurveAnalDomain.IsContainedBy(sSurfaceAnalDomain, dTol)) ;
  if ( bRtn==FALSE )
    { SM_ASSERT_MSG(FALSE, _T("Compatible Surf and Curve analytic domain ranges differ") ) ;  }

  // The surface and genCurve knot vectors must be the same
  ULONG lSurfaceCount = sSurfaceKnots.GetSize() ;
  ULONG lCurveCount   = sCurveKnots.GetSize() ;
  if(lSurfaceCount != lCurveCount) { bRtn = FALSE ; }
  
  // for every knot and multiplicity
  for(ULONG ii=0;ii<lSurfaceCount && bRtn;ii++) 
    {
      // check compatibility - 
      // Cone Generator curves and cone surface knot vectors don't have to be the same
        //cbi So shouldn't we skip this test if 'this' is an SmCone?  [B375]
      SmBoolean bTest ;
      bTest = SM_ARE_SAME(sSurfaceKnots[ii], sCurveKnots[ii]) ;
      SM_ASSERT_MSG(bTest == TRUE, _T("Compatible Surf and Curve knot value differs") ) ;
      bRtn &= bTest ;
        
      bTest &= sSurfaceMults[ii] == sCurveMults[ii] ;
      if(bTest == FALSE)
        {
          SM_ASSERT_MSG(bTest == TRUE, _T("Compatible Surf and Curve knot multiplicity differs") ) ;
        }
      bRtn &= bTest ;

    } // end iter every genCurve knot checking for compatibility

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // Note: AssertValid() calls this method.
      // SM_DUMP_AND_ASSERT_VALID(this) ;
      this->Dump();

      SM_DUMP_AND_ASSERT_VALID(cpOtherCurve) ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmEdge *pEdge = (SmEdge *)cpOtherCurve->GetEdge() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() 
                      : pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      smgfx_SetLook( 3, 4, 1, 0, 0 ); if(cpOtherCurve) { cpOtherCurve->Draw( NULL, TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook( 5, 6, 1, 0, 1 ); if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ;

} // end SmBSplineSurface::HasSameParameterization
  
/*******************************************************************//**
PURPOSE: Return TRUE when surface has the same analytic domain
            and knot vectors as the otherSurface.

NOTES:
***********************************************************************/
SmBoolean SmBSplineSurface::HasSameParameterization
  (const SmBSplineSurface *cpOtherSurface) // in : target surface
 const 
{
  const SmBSplineSurface *cpThisSurface = this ;

  // analytic parameterization locals
  SmExtent2d sAnalDomain1 = cpThisSurface->GetSTEPUVDomain();
  SmExtent2d sAnalDomain2 = cpOtherSurface->GetSTEPUVDomain() ;

  // Nurb parameterization locals
  SmTArray<double> sKnotsU1, sKnotsV1, sKnotsU2, sKnotsV2 ;           
  SmTArray<ULONG>  sMultsU1, sMultsV1, sMultsU2, sMultsV2 ;
  cpThisSurface ->GetKnots(SM_SP_U, sKnotsU1, &sMultsU1) ;  
  cpThisSurface ->GetKnots(SM_SP_V, sKnotsV1, &sMultsV1) ;  
  cpOtherSurface->GetKnots(SM_SP_U, sKnotsU2, &sMultsU2) ;  
  cpOtherSurface->GetKnots(SM_SP_V, sKnotsV2, &sMultsV2) ;  

  // analytic domains must be compatible
  SmBoolean bRtn = (   sAnalDomain1.IsContainedBy(sAnalDomain2, SM_EFF_ZERO)
                    && sAnalDomain2.IsContainedBy(sAnalDomain1, SM_EFF_ZERO)) ;

  // knot vectors must be the same
  ULONG ii ;
  ULONG lU1Count = sKnotsU1.GetSize() ;
  ULONG lV1Count = sKnotsV1.GetSize() ;
  ULONG lU2Count = sKnotsU2.GetSize() ;
  ULONG lV2Count = sKnotsV2.GetSize() ;
  if(   lU1Count != lU2Count
     && lV1Count != lV2Count) { bRtn = FALSE ; }
  
  // for every U knot and multiplicity
  for(ii=0;ii<lU1Count && bRtn;ii++) 
    {
      // check compatibility
      bRtn &= SM_ARE_SAME(sKnotsU1[ii], sKnotsU2[ii]) ;
      bRtn &= sMultsU1[ii] == sMultsU2[ii] ;

    } // end iter every U knot checking for compatibility

  // for every V knot and multiplicity
  for(ii=0;ii<lV1Count && bRtn;ii++) 
    {
      // check compatibility
      bRtn &= SM_ARE_SAME(sKnotsV1[ii], sKnotsV2[ii]) ;
      bRtn &= sMultsV1[ii] == sMultsV2[ii] ;

    } // end iter every V knot checking for compatibility

  // all done
  return(bRtn) ;

} // end SmBSplineSurface::HasSameParameterization

/*******************************************************************//**
PURPOSE: Write SmBSplineSurface to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::WriteToDB
 (SmDatabaseIO & rDB,               // in : target output stream
  ULONG          lDBVersionNumber)  // in : database version to get proper sequence of writes                                                                       
 const
{
  gw_REAL  wx, wy, wz, w;

  // file type, ASCII or BINARY
  SmFileType eType = rDB.GetFileType();

  // Surface locals 
  gw_SURFACE *sur = ((SmBSplineSurface *)this)->GetOrCreateGwNurbPointer() ; 
  gw_INDEX   i, j, n, m, r, s;
  gw_DEGREE  p, q;
  gw_CPOINT  **Pw;
  gw_REAL    *U, *V ;
  N_SrfGetCPtsDegreesAndKnots(sur,&n,&m,&Pw,&p,&q,&r,&s,&U,&V);
  ULONG      lKnotType           = m_eKnotType ;
  ULONG      lBSplineSurfaceForm = m_eBSplineSurfaceForm ;

  // rational flag 
  gw_FLAG  rat =  N_IsSrfRat(sur) ? NL_YES : NL_NO ; 

  // Create the output file 
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << n << " " << m << "\n";
      rFileOut << p << " " << q << "\n";
      rFileOut << rat << "\n";
      if( lDBVersionNumber > 32) 
        {
          rFileOut << lKnotType << " BSplineSurface Knot Type Flag \n" ;
          rFileOut << lBSplineSurfaceForm << " BSplineSurfaceForm Flag \n" ;
           }
    }
  else /* Binary */      
    { 
      SER(rDB.WriteLong(n));
      SER(rDB.WriteLong(m));
      SER(rDB.WriteShort(p));
      SER(rDB.WriteShort(q));
      SER(rDB.WriteShort(rat));
      if( lDBVersionNumber > 32)
        {
          SER(rDB.WriteLong(lKnotType));
          SER(rDB.WriteLong(lBSplineSurfaceForm));
        }
    } 

  // switch on rational/nonRational to output control points
  switch( rat )
    {
      // Non-rational
      case NL_NO : { // for every control point
                  for( i=0; i<=n; i++ )
                    {   
                      for( j=0; j<=m; j++ )
                        {
                          N_CPtToWxWyWz(Pw[i][j],&wx,&wy,&wz,&w);
                          if (eType == SM_ASCII) 
                            {
                              std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                              rFileOut << wx << " " << wy << " " << wz << "\n";    
                            }
                          else /* Binary */  
                            {
                              SER(rDB.WriteDouble(wx));
                              SER(rDB.WriteDouble(wy));
                              SER(rDB.WriteDouble(wz));
                  //            rFileOut.write((char*)&wx,sizeof(gw_REAL));
                  //            rFileOut.write((char*)&wy,sizeof(gw_REAL));
                  //            rFileOut.write((char*)&wz,sizeof(gw_REAL));
                            }
                        } // end iter every controlPoint j
                    } // end iter every controlPoint i
                } // end non-rational branch
              break ;

      // Rational
      case NL_YES : { // for every control point
                  for( i=0; i<=n; i++ )
                    {   
                      for( j=0; j<=m; j++ )
                        {
                          N_CPtToWxWyWz(Pw[i][j],&wx,&wy,&wz,&w);
                          if (eType == SM_ASCII) 
                            {
                              std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                              rFileOut << wx << " " << wy << " " << wz << " " << w << "\n";    
                            }
                          else /* Binary */      
                            { 
                              SER(rDB.WriteDouble(wx));
                              SER(rDB.WriteDouble(wy));
                              SER(rDB.WriteDouble(wz));
                              SER(rDB.WriteDouble(w));
                              //    rFileOut.write((char*)&wx,sizeof(gw_REAL));
                              //    rFileOut.write((char*)&wy,sizeof(gw_REAL));
                              //    rFileOut.write((char*)&wz,sizeof(gw_REAL));
                              //    rFileOut.write((char*)&w,sizeof(gw_REAL));
                            }
                        } // end iter every controlPoint j
                    } // end iter every controlPoint i
                } // end non-rational branch
              break ;

      // Wrong type
      default : { SER(SM_ERR) ; }

    } // end switch on rational/nonRational to output control points

  // output u knots
  for( i=0; i<=r; i++ )
    {
      if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                               rFileOut << U[i] << "\n";    
                             }
      else /* Binary */      { SER(rDB.WriteDouble(U[i]));
                              //  rFileOut.write((char*)&U[i],sizeof(gw_REAL));
                             }
    }    

  // output v knots
  for( j=0; j<=s; j++ )
   {
     if (eType == SM_ASCII) { std::ostream & rFileOut = *rDB.GetOutStreamPtr();
                              rFileOut << V[j] << "\n";    
                            }
     else /* Binary */      { SER(rDB.WriteDouble(V[j]));
                              //  rFileOut.write((char*)&V[j],sizeof(gw_REAL));
                            }
   }    

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmBSplineSurface from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmBSplineSurface::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  const SmContext & crContext,          // in : context for new object construction
  SmSurface      *& rpNewSurface,       // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF1(lType) ;
  // check input
  SER(  (   rpNewSurface == NULL
         || rpNewSurface->IsKindOf(SmBSplineSurface_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmBSplineSurface *pBSplineSurface =   (rpNewSurface == NULL)
                                      ? new (crContext) SmBSplineSurface()
                                      : (SmBSplineSurface *)rpNewSurface ;

  // file type
  SmFileType eType = rDB.GetFileType();

  // locals
  gw_REAL    wx, wy, wz, w;

  // Get parameters from the top 
  gw_INDEX   i, j, n, m ;
  gw_DEGREE  p, q;
  gw_FLAG    rat ; 
  ULONG lKnotType           = (ULONG) SM_KT_UNSPECIFIED ;
  ULONG lBSplineSurfaceForm = (ULONG) SM_SF_UNSPECIFIED ;

  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> n >> m ; rDB.GoToNextLine();
      rFileIn >> p >> q ; rDB.GoToNextLine();
      rFileIn >> rat;     rDB.GoToNextLine();
      if( lDBVersionNumber > 32) 
        { 
                rFileIn >> lKnotType ;           
                rDB.GoToNextLine();
          rFileIn >> lBSplineSurfaceForm ;
                rDB.GoToNextLine();
         }
    }
  else /* Binary */      
    { 
      SER(rDB.ReadLong(n));
      SER(rDB.ReadLong(m));
      SER(rDB.ReadShort(p));
      SER(rDB.ReadShort(q));
      SER(rDB.ReadShort(rat));
      if( lDBVersionNumber > 32)
        {
          SER(rDB.ReadLong(lKnotType));
          SER(rDB.ReadLong(lBSplineSurfaceForm));
        }
    }

  // surface knot counts
  gw_INDEX r = n+p+1;
  gw_INDEX s = m+q+1;

  // allocate Sur memory 
  gw_SURFACE *sur = sm_AllocateNurbSurface(n,m,p,q,r,s);
  if(sur == NULL) 
    { SER(SM_ERR) ; }

  // surface locals
  gw_CPOINT  **Pw;
  gw_REAL    *U, *V ; 
  N_SrfGetCPtsAndKnots(sur,&Pw,&U,&V);

  // Read in data 
  switch( rat )
    {
      // Non-rational
      case NL_NO  : { // for every control point   
                   for( i=0; i<=n; i++ )
                     {   
                       for( j=0; j<=m; j++ )
                         {
                           if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                                                    rFileIn >> wx >> wy >> wz; rDB.GoToNextLine();
                                                  }
                           else /* Binary */      { SER(rDB.ReadDouble(wx));
                                                    SER(rDB.ReadDouble(wy));
                                                    SER(rDB.ReadDouble(wz));
                                                    //   rFileIn.read((char*)&wx,sizeof(gw_REAL));
                                                    //   rFileIn.read((char*)&wy,sizeof(gw_REAL));
                                                    //   rFileIn.read((char*)&wz,sizeof(gw_REAL));
                                                  }
                           N_CPtFromWxWyWz(wx,wy,wz,NL_NOW,&Pw[i][j]);
                         } // end iter every conrol point i
                     } // end iter every control point j
                 } break ;

      // Rational
      case NL_YES : { // for every control point  
                   for( i=0; i<=n; i++ )
                     {   
                       for( j=0; j<=m; j++ )
                         {
                           if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                                                    rFileIn >> wx >> wy >> wz >> w; rDB.GoToNextLine();
                                                  }
                           else /* Binary */      { SER(rDB.ReadDouble(wx));
                                                    SER(rDB.ReadDouble(wy));
                                                    SER(rDB.ReadDouble(wz));
                                                    SER(rDB.ReadDouble(w));
                                                    // rFileIn.read((char*)&wx,sizeof(gw_REAL));
                                                    // rFileIn.read((char*)&wy,sizeof(gw_REAL));
                                                    // rFileIn.read((char*)&wz,sizeof(gw_REAL));
                                                    // rFileIn.read((char*)&w,sizeof(gw_REAL));
                                                  }
                           N_CPtFromWxWyWz(wx,wy,wz,w,&Pw[i][j]);
                         } // end iter every conrol point i
                     } // end iter every control point j
                 } break ;

      // Wrong type
      default : { SER(SM_ERR) ; }
    }

  // for every U knot
  for( i=0; i<=r; i++ )
    {
      if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                               rFileIn >> U[i]; rDB.GoToNextLine();
                             }
      else /* Binary */      { SER(rDB.ReadDouble(U[i]));
                               //  rFileIn.read((char*)&U[i],sizeof(gw_REAL));
                             }
    }    

  // for every V knot
  for( j=0; j<=s; j++ )
    {
      if (eType == SM_ASCII) { std::istream & rFileIn = *rDB.GetInStreamPtr();
                               rFileIn >> V[j]; rDB.GoToNextLine();
                             }
      else /* Binary */      { SER(rDB.ReadDouble(V[j]));
                               // rFileIn.read((char*)&V[j],sizeof(gw_REAL));
                             }
    }
  
  // output locals  
  SmKnotType            eKnotType           = (SmKnotType) lKnotType ;             
  SmBSplineSurfaceForm  eBSplineSurfaceForm = (SmBSplineSurfaceForm) lBSplineSurfaceForm ;

  // set output
  pBSplineSurface->m_pNurb               = sur ;
  pBSplineSurface->m_bNurbIsBorrowed     = FALSE ; 
  pBSplineSurface->m_bOutOfBoundsEnabled = FALSE ;
  pBSplineSurface->m_eKnotType           = eKnotType ;         
  pBSplineSurface->m_eBSplineSurfaceForm = eBSplineSurfaceForm ;

  rpNewSurface = pBSplineSurface ;

  // all done
  return SM_SUCCESS;

} // end SmBSplineSurface::ReadFromDB

