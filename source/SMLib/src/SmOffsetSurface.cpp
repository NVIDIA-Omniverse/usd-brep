// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmOffsetSurface.cpp 
* PURPOSE: Implementation of SmOffsetSurface methods.
**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <SmExtent2d.h>
#include <SmBSplineSurface.h>
#include <SmOffsetSurface.h>
#include <SmGraphicsExtern.h>
#include <SmExtent3d.h>
#include <SmPseudoBox.h>
#include <SmPolarBox.h>
#include <SmDatabaseIO.h>
#include <SmAssertArray.h>

#include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */


#ifdef SM_DEBUG_CODE
 #include <SmBrep.h>
 #include <SmFace.h>
 #include <SmGap.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Constructor for offset surface objects

NOTES:
  When constructing an offset from an offset surface, the
  given offset distance is measured from the original surface not the
    the position of the offset surface.
     
***********************************************************************/
SmOffsetSurface::SmOffsetSurface
  (double      dOffsetDistance,    // in : offset distance (negative for insets)
   SmSurface & rSurface,           // in : surface being offset
   SmBoolean   bOwnsSurface,       // in : TRUE = owns surface - deletes m_pSurface when its deleted
                                   //      FALSE= shares surface - no m_pSurface delete when its deleted
                                   //      default:[FALSE]
   ULONG       lSingularities)     // in : SM_SS_NONE or one of: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX, or SM_SS_UNKNOWN
                                   //    
                                        
 : m_dOffsetDistance(dOffsetDistance), 
   m_bAllowExtension(FALSE),
   m_bOwnsSurface(bOwnsSurface), 
   m_pSurface(&rSurface), 
   m_pExtendedSurface(NULL),
   m_bIsPeriodicOnU(FALSE),
   m_bIsPeriodicOnV(FALSE),
   m_lSingularities(SM_SS_NONE)
{
  if ( m_cpContext == NULL )
    { m_cpContext = rSurface.GetContext(); }
  
  const SmOffsetSurface *pOff = SM_CAST_NONNULL_PTR(SmOffsetSurface,&rSurface);
  if (pOff) 
    {
      m_bIsPeriodicOnU = pOff->m_bIsPeriodicOnU;
      m_bIsPeriodicOnV = pOff->m_bIsPeriodicOnV;
      m_lSingularities  = pOff->m_lSingularities;
    }
  else 
    {
      SmExtent2d sDomain = rSurface.GetNaturalUVDomain();
      if (rSurface.IsPeriodic(sDomain,SM_SP_U)) { m_bIsPeriodicOnU = TRUE; }
      if (rSurface.IsPeriodic(sDomain,SM_SP_V)) { m_bIsPeriodicOnV = TRUE; }
      m_lSingularities = lSingularities != SM_SS_UNKNOWN ? lSingularities : rSurface.GetSingularities();
    }

  if( m_bOwnsSurface == TRUE) { m_pSurface->SetOwner(this) ; }
} // end SmOffsetSurface::SmOffsetSurface constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmOffsetSurface object.

NOTES: Currently you will only be able to copy SmOffsetSurface if
   either you reference the geometry or the geometry is owned and is
   a SmBSplineCurve and/or SmBSplineSurface.  You can not copy a 
   SmOffsetSurface if it owns a non-SmBSplineCurve or non SmBSplineSurface
***********************************************************************/
SmOffsetSurface::SmOffsetSurface
  (const SmOffsetSurface & crSurfaceToCopy)
 : SmSurface(crSurfaceToCopy),
   m_dOffsetDistance(crSurfaceToCopy.m_dOffsetDistance),
   m_bAllowExtension(crSurfaceToCopy.m_bAllowExtension),
   m_bOwnsSurface(TRUE),
   m_pSurface(NULL),
   m_pExtendedSurface(NULL),
   m_bIsPeriodicOnU(crSurfaceToCopy.m_bIsPeriodicOnU),  
   m_bIsPeriodicOnV(crSurfaceToCopy.m_bIsPeriodicOnV),
   m_lSingularities(crSurfaceToCopy.m_lSingularities)
 { 
   const SmContext *cpContext =  GetContext()                             ? GetContext()
                               : crSurfaceToCopy.GetContext()             ? crSurfaceToCopy.GetContext()
                               : crSurfaceToCopy.m_pSurface->GetContext() ? crSurfaceToCopy.m_pSurface->GetContext()
                               : NULL ;
   SE_MSG(cpContext != NULL ? SM_SUCCESS : SM_ERR, 
          _T("SmOffsetSurface copy constructor can't find a SmContext for new object construction")) ;
   crSurfaceToCopy.m_pSurface->Copy(*cpContext, m_pSurface) ;

   m_pSurface->SetOwner(this) ;
 } // end SmOffsetSurface::SmOffsetSurface copy constructor  

/*******************************************************************//**
PURPOSE: Destructor for offset surface objects

NOTES: 
***********************************************************************/
SmOffsetSurface::~SmOffsetSurface()
{
  if (m_pSurface && m_bOwnsSurface) { delete m_pSurface; m_pSurface = NULL ; }
  if (m_pExtendedSurface)           { delete m_pExtendedSurface; m_pExtendedSurface = NULL ; }

} // end SmOffsetSurface::~SmOffsetSurface destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmOffsetSurface

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmOffsetSurface::operator==
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
      SmOffsetSurface &rOther = (SmOffsetSurface &)crOther ;

      // check equivalence of these objects
      bRtn =  (   SM_IS_ZERO(m_dOffsetDistance - rOther.m_dOffsetDistance)
               && (   ( m_pSurface == rOther.m_pSurface)
                   || ( m_pSurface == NULL && rOther.m_pSurface == NULL)
                   || (   m_pSurface != NULL && rOther.m_pSurface != NULL
                       && *m_pSurface == *rOther.m_pSurface))
               && m_bIsPeriodicOnU == rOther.m_bIsPeriodicOnU
               && m_bIsPeriodicOnV == rOther.m_bIsPeriodicOnV
               && m_lSingularities  == rOther.m_lSingularities) ;
    }

  // all done
  return bRtn ;

} // end SmOffsetSurface::operator==

/*******************************************************************//**
PURPOSE: Copy an offset surface.

NOTES: 
***********************************************************************/
SmStatus SmOffsetSurface::Copy
  (const SmContext & crContext,     // in : object for new objects
   SmSurface      *& rpNewSurface)  // out: deep copy output
  const
{
  SmOffsetSurface *pSur;

  // when BaseSurface is owned
  if (m_bOwnsSurface) 
    {
      SmSurface *pNewSur;
      SER(m_pSurface->Copy(crContext,pNewSur));
      pSur = new (crContext) SmOffsetSurface(m_dOffsetDistance, *pNewSur);
      NER(pSur);
      pSur->m_bOwnsSurface = TRUE;
      
      pNewSur->SetOwner(pSur);
    }
  else // when BaseSurface is not owned
    {
      pSur = new (crContext) SmOffsetSurface(m_dOffsetDistance, *m_pSurface);
      NER(pSur);
      pSur->m_bOwnsSurface = FALSE;
    }

  pSur->m_bAllowExtension = m_bAllowExtension;

  // Copy ExtendedSurface
  if (m_pExtendedSurface) 
    {
      SmSurface *pNewExtended;
      SER(m_pExtendedSurface->Copy(crContext,pNewExtended));
      pSur->m_pExtendedSurface = pNewExtended;
    }

  // set output
  rpNewSurface = pSur;

  // inform the public
  ((SmOffsetSurface*)this)->Notify(SM_NO_COPY, rpNewSurface, SM_NO_GET_OWNER(rpNewSurface), SM_NO_GET_OWNER(this));

  // all done
  return SM_SUCCESS;

} // end SmOffsetSurface::Copy

/*******************************************************************//**
PURPOSE: Create a mirror surface of an offset surface.

NOTES: 
***********************************************************************/
SmStatus SmOffsetSurface::CreateMirrorSurface
  (const SmContext        & crContext,        // in : context for new object 
   const SmAxis2Placement & crMirrorPlane,    // in : mirror plane
   SmSurface             *& rpMirrorSurface)  // out: mirrored surface
  const
{
  SmSurface *pMirror;
  SmOffsetSurface *pMirrorOff;
  SER(m_pSurface->CreateMirrorSurface(crContext,crMirrorPlane,pMirror));
  pMirrorOff = new (crContext) SmOffsetSurface(m_dOffsetDistance,*pMirror,TRUE);
  NER(pMirrorOff);

  // GWC: Surface ownership is not mandatory and used for topology
  //   pMirror->SetOwner(pMirrorOff);
  pMirrorOff->m_bAllowExtension = m_bAllowExtension;

  if (m_pExtendedSurface) 
    {
      SmSurface *pMirrorExtended;
      SER(m_pExtendedSurface->CreateMirrorSurface(crContext,crMirrorPlane,pMirrorExtended));
      NER(pMirrorExtended);
      pMirrorOff->m_pExtendedSurface = pMirrorExtended;
      // GWC: Surface ownership is not mandatory and used for topology
      //   pMirrorExtended->SetOwner(pMirrorOff);
    }
  pMirrorOff->m_dOffsetDistance = - m_dOffsetDistance; // This needs to be done
  // because mirror reverses orientation of the surface.
  rpMirrorSurface = pMirrorOff;
  return SM_SUCCESS;

} // end SmOffsetSurface::CreateMirrorSurface

/*******************************************************************//**
PURPOSE: Create an extended surface by extending a given distance from
    each side of the original surface

NOTES: This method is only available for users with NLib

***********************************************************************/
SmStatus SmOffsetSurface::CreateExtendedSurface
( 
  const SmContext   & crContext,            // in : context for new object construction
  double              dDist,                // in : Distance of extension from each side
  SmContinuityType    eExtensionContinuity, // in : OneOf: SM_CT_G1 - linear extension
                                            //             SM_CT_G1R - 
                                            //             SM_CT_G1_G2 - extension with second derivative
                                            //             SM_CT_CINFINITY - infinite continuity
  SmSurface        *& rpExtended            // out: the newly constructed surface (NULL on input)
)
{
    // pass the call along
    return ( CreateExtendedSurface( crContext, SM_SP_BOTH, dDist, eExtensionContinuity, rpExtended ) );
}

/*******************************************************************//**
PURPOSE: Create an extended surface by extending a given distance from
    one or all side(s) of the original surface. This is an overloaded function.

NOTES: This method is only available for users with NLib

***********************************************************************/
SmStatus SmOffsetSurface::CreateExtendedSurface
( 
  const SmContext   & crContext,            // in : context for new object construction
  SmSurfParamType     eExtDirection,        // in : Either extend in SM_SP_U or SM_SP_V direction
                                            //      or SM_SP_UMIN/VMIN/UMAX/VMAX/BOTH
  double              dDist,                // in : Distance of extension from each side
  SmContinuityType    eExtensionContinuity, // in : oneof: SM_CT_G1 - linear extension
                                            //             SM_CT_G1R - 
                                            //             SM_CT_G1_G2 - extension with second derivative
                                            //             SM_CT_CINFINITY - infinite continuity
  SmSurface        *& rpExtended            // out: the newly constructed surface (NULL on input)
)
{
    SmSurface *pSurface = NULL;
    SER(m_pSurface->CreateExtendedSurface( crContext, eExtDirection, dDist, eExtensionContinuity, pSurface ));

    rpExtended = new ( crContext ) SmOffsetSurface( m_dOffsetDistance, *pSurface, TRUE );

    return SM_SUCCESS;

} // end SmOffsetSurface::CreateExtendedSurface

/*******************************************************************//**
PURPOSE: Given a domain on the Surface, compute the axis alligned 
    (normalBox), and/or non-axis alligned (pseudoBox), and/or 
    polar (PolarBox) bounding boxes.  
    
    The domain argument is not used to reduce normalBox and pseudoBox
    sizes.  Those boxes are the size of the natural NURBS surface.
    When the surface is analytic, its PolarBox is limited to the 
    given crUVDomain value, non-Analytic PolarBoxes are sized to the
    natural NURBs surface. The center of the given subDomain is used
    to orient polarBoxes.

NOTES: One or more of the outputs must be non-NULL.
    Bounding boxes are built for whole surfaces not subdomains.
    The cache mechanism subdivides a large surface into a set of
    bezier patches and then calls this function to bound each sub-region.
***********************************************************************/
SmStatus SmOffsetSurface::CalculateBoundingBox
  (const SmExtent2d & crUVDomain,            // in : only whole surfaces are bounded (except for polarBoxes of analytic surfaces)
   SmExtent3d  * pNormalBox,                 // out: Axis alligned box
   SmPseudoBox * pPseudoBox,                 // out: Non-axis aligned box
   SmPolarBox  * pPolarBox,                  // out: Surface normal vector field bounding box
   const SmPseudoBox * pOptPseudoBasisGuess,         // in : guess for PseudoBox basis vectors - used unless another better orientation is found
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

  // for request using the extended surface - use the SmSurface::CalculateBoundingBox algorithm
  if(!crUVDomain.IsContainedBy(m_pSurface->GetNaturalUVDomain(), SM_EFF_ZERO))
    {
      // pass the call along to SmSurface
      return( SmSurface::CalculateBoundingBox(crUVDomain,
                                              pNormalBox,
                                              pPseudoBox,
                                              pPolarBox, 
                                              pOptPseudoBasisGuess,
                                              pOptPolarBasisGuess,
                                              bExpandPosBoxesByZoneTol3d) ) ;
    }

  // arrive here when the requested domain crUVDomain fits within the 
  // domain of the original base surface.

  // init output
  if(pNormalBox) { pNormalBox->Init() ; }
  if(pPseudoBox) { pPseudoBox->Init() ; }  // init intervals - leave basis vectors alone
  if(pPolarBox)  { pPolarBox->ReSet() ; }

  // locals
  SmExtent3d  sBaseNormalBox ;
  SmPseudoBox sBasePseudoBox ;
  SmPolarBox  sBasePolarBox ;
  ULONG       crn, vv, lNumVectors ;
  SmPoint3d   sPointBase, sPointTip ;
  SmVector3d  sVectors[4], sPseudoCorners[8] ;

  // pass the call along to the base surface
  SmStatus sRtn = m_pSurface->CalculateBoundingBox(crUVDomain, &sBaseNormalBox, &sBasePseudoBox, &sBasePolarBox, pOptPseudoBasisGuess, pOptPolarBasisGuess, bExpandPosBoxesByZoneTol3d) ;
  if(sRtn != SM_SUCCESS)
    {
      return(sRtn) ;
    }

  // copy the polar box as needed
  if(pPolarBox)
    { 
      *pPolarBox = sBasePolarBox ; 
    }

  // copy the PseudoBox base vectors
  if(pPseudoBox)
    {
      pPseudoBox->SetBasis(sBasePseudoBox.GetBasis(0),
                           sBasePseudoBox.GetBasis(1),
                           sBasePseudoBox.GetBasis(2)) ;
    } 
  
  // init the      

  // grow the Normal and Pseudo bounding boxes as needed in the direction of the surface normals
  switch(sBasePolarBox.GetPolarBoxState())
    {
      case SM_PS_POINT_WITH_BASIS :
      case SM_PS_SINGLE_POINT  : 
      case SM_PS_ARC           : 
      case SM_PS_REGION        : 
        { // offset the bounding boxes

          // get Polar Box offset vectors
          sBasePolarBox.GetBoundaryVectors(lNumVectors, sVectors) ;
          sBasePseudoBox.CalcCorners(sPseudoCorners) ;

          // for every bounding box corner
          for(crn=0;crn<8;crn++)
            {
              if(pNormalBox)
                {
                  sPointBase = sBaseNormalBox.Evaluate((crn/4)%2,(crn/2)%2,(crn/1)%2) ;

                  // for every PolarBox boundary vector
                  for(vv=0;vv<lNumVectors;vv++)
                    {
                       sPointTip = sPointBase + m_dOffsetDistance * sVectors[vv] ; 
                       pNormalBox->AddPoint3d(sPointTip) ;  
                    }
                }

              if(pPseudoBox)
                {
                  // for every PolarBox boundary vector
                  for(vv=0;vv<lNumVectors;vv++)
                    {
                       sPointTip = sPseudoCorners[crn] + m_dOffsetDistance * sVectors[vv] ; 
                       pPseudoBox->AddPoint3d(sPointTip) ;  
                    }
                }
            } // end iter every bounding box corner
        }
        break ;

      case SM_PS_UNBOUNDED     : 
      case SM_PS_UNINITIALIZED : 
        { // offset the bounding boxes in all directions

          // expand Normal Box by Offset Distance as needed
          if(pNormalBox)
            { *pNormalBox = sBaseNormalBox ;
              pNormalBox->ExpandAbsolute(fabs(m_dOffsetDistance)) ; 
            }

          // expand Pseudo Box by Offset Distance as needed
          if(pPseudoBox)
            { *pPseudoBox = sBasePseudoBox ;
              pPseudoBox->ExpandAbsolute(fabs(m_dOffsetDistance)) ; 
            }
        }
        break ;

    } // end switch on pBasePolarBox.m_ePolarBoxState

  // no need to expand Polar Box - offsetting does not change the polar box

  // all done
  return(sRtn) ;

} // end SmOffsetSurface::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Make sure given DomainPoint is contained in
            domain of given surface by 
   1. Wrapping UV coordinates in domain periodic directions and
   2. Extending the nonPeriodic domain directions

NOTES: 
  When input Point is contained by wrapping coordinates in periodic directions 
    - no Extended surface is made and output surface is set to NULL.
***********************************************************************/
static SmStatus sm_ExtendSurface
  (const SmContext        & crContext,       // in : context for new object
   SmBSplineSurface       * pSurface,        // in : BaseSurface to extend
   SmPoint2d              & rUV,             // i/o: DomainPoint to include in extended domain
                                             //      coordinates of periodic domains are
                                             //      wrapped to the principle period
   SmBoolean                bIsPeriodicOnU,  // in : TRUE = Periodic in U direction
   SmBoolean                bIsPeriodicOnV,  // in : TRUE = Periodic in V direction
   ULONG                    lSingularities,   // in : SM_SS_NONE or an orof: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX
   SmBSplineSurface      *& rpExtended)      // out: New extended Surface or NULL
                                             //      when periodicWrapping of rUV puts
                                             //      it into pSurface->Domain limits
{
  // init output
  rpExtended = NULL;

  // locals
  SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
  SmExtent1d sIvlU   = sDomain.GetUInterval() ; 
  SmExtent1d sIvlV   = sDomain.GetVInterval() ; 
  SmPoint2d  sSize   = sDomain.GetSize();

  // For Periodic Domain directions - wrap rUV
  if (bIsPeriodicOnU) { rUV.x = sIvlU.PeriodicWrap(rUV.x) ; }
  if (bIsPeriodicOnV) { rUV.y = sIvlV.PeriodicWrap(rUV.y) ; }

  // for surface singularities - bound rUV to stop extending past a singularity
  if((lSingularities & SM_SS_UMIN) && (rUV.x < sIvlU.GetMin())) { rUV.x = sIvlU.GetMin() ; }
  if((lSingularities & SM_SS_VMIN) && (rUV.y < sIvlV.GetMin())) { rUV.y = sIvlV.GetMin() ; }
  if((lSingularities & SM_SS_UMAX) && (rUV.x > sIvlU.GetMax())) { rUV.x = sIvlU.GetMax() ; }
  if((lSingularities & SM_SS_VMAX) && (rUV.y > sIvlV.GetMax())) { rUV.y = sIvlV.GetMax() ; }
  
  // if PeriodicWrap or Singularity clamping inserted point into domain - all done
  if (sDomain.ContainsPoint2d(rUV)) 
    { return SM_SUCCESS; }

  // arrive here when BaseSurface has to be extended to include target DomainPoint

  // only Bsplines can be extended
  if (!pSurface->IsKindOf(SmBSplineSurface_TYPE)) 
    { SER(SM_ERR); } 
  
  // increase the size of legally requested extensions by a big amount 
  // to prevent a series of evaluations from causing a series of small extensions.
  SmPoint2d sUV = rUV;
  if      (sUV.x < sDomain.GetMin().x) { sUV.x = sUV.x - sSize.x ; }
  else if (sUV.x > sDomain.GetMax().x) { sUV.x = sUV.x + sSize.x ; }
  if      (sUV.y < sDomain.GetMin().y) { sUV.y = sUV.y - sSize.y ; }
  else if (sUV.y > sDomain.GetMax().y) { sUV.y = sUV.y + sSize.y ; }

  // set up for NLIb Call
  NL_SURFACE    esur, *sur = pSurface->GetOrCreateGwNurbPointer();
  NL_PARAMETER  u = sUV.x; 
  NL_PARAMETER  v = sUV.y;
  NL_FLAG    gflg, cflg = NL_G1R;
  NL_INDEX      ndr = 0 ;
  NL_POINT   P, **D = NULL;
  NL_STACKS     SG;

  N_InitNurbs(&SG);
  N_SrfInitArrays(&esur);

  // Have NLib evaluate the surface position
  // side effect: build extended surface in esur
  NL_FLAG err = N_SrfEvalPtDerivsUnbounded(sur,u,v,ndr,cflg,&esur,&gflg,&P,D,&SG);           
  if (err) 
    { 
      // see if a modest extension will generate an evaluation
      u = rUV.x ;
      v = rUV.y ;
      err = N_SrfEvalPtDerivsUnbounded(sur,u,v,ndr,cflg,&esur,&gflg,&P,D,&SG);
      if (err) 
        {
          SER(SM_ERR);
        }
    } // end err check

  // copy extended surface into SMLib format
  rpExtended = new (crContext) SmBSplineSurface((gw_SURFACE *)&esur) ; 

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,0) ; pSurface->DrawUV(5,5) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; rpExtended->DrawUV(4,4,FALSE,NULL,TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
      if ( FALSE )
        { pSurface->DrawInspectTwoSurfaces( rpExtended ); }
    }
#endif  // SM_DEBUG_CODE
  
  N_EndNurbs(&SG);

  // Do a little check to make sure we don't go too far outside of original surface.
  SmExtent2d sNewDomain = rpExtended->GetNaturalUVDomain();
  SmVector2d sNewSize   = sNewDomain.GetSize();
  if (sNewSize.Length() > 20.0*sSize.Length()) 
    {
      MSG(_T("Extending Surface Too Much"));
    }
  
  // all done    
  return SM_SUCCESS;

} // end sm_ExtendSurface

/*******************************************************************//**
PURPOSE: Get a list of the unique knots and optionally knot multiplicities
    of one of the parameters of the underlying surface of the offset.
    
NOTES: This is the STEP compatible form of the knots not the
    typical knot vector associated with NURBS.
***********************************************************************/
SmStatus SmOffsetSurface::GetKnots
  (SmSurfParamType    eSurfParam,            // in :    
   SmTArray<double> & rKnots,                // out: 
   SmTArray<ULONG>  * pKnotMultiplicities,   // out:
   const SmExtent1d * pOptIvl)               // in : interval of interest, NULL=Natural Interval, default:[NULL] 
  const
{    
  if (m_pExtendedSurface) 
    {
      SER(m_pExtendedSurface->GetKnots(eSurfParam,rKnots,pKnotMultiplicities,pOptIvl));
    }
  else 
    {
      SER(m_pSurface->GetKnots(eSurfParam,rKnots,pKnotMultiplicities,pOptIvl));
    }
  return SM_SUCCESS;

} // end SmOffsetSurface::GetKnots

/*******************************************************************//**
PURPOSE: Evaluation of the normal of an offset surface at a point.  This routine
   returns a unitized normal vector
NOTES:  
***********************************************************************/ 
SmStatus SmOffsetSurface::EvaluateNormal
  (const SmPoint2d & crUV,       // in : target domain point
   SmBoolean bUFromLeft,         // in : if P is on U interval boundary
                                 //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                 //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bVFromLeft,         // in : if P is on V interval boundary
                                 //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                 //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmVector3d & rSurfaceNormal)  // out: unit-normal
  const
{
  // see if domainPoint is within BaseSurface domain
  SmExtent2d sDomain = m_pSurface->GetNaturalUVDomain();
  if (!sDomain.ContainsPoint2d(crUV))
    {
      // pass the call along to base virtual function
      // which calls Evaluate which will cause an extended surface
      //   to be built if needed and possible
      return SmSurface::EvaluateNormal(crUV, bUFromLeft, bVFromLeft,
                                       rSurfaceNormal);
    }
  else
    {
      return m_pSurface->EvaluateNormal(crUV, bUFromLeft, bVFromLeft,
                                        rSurfaceNormal);
    }

} // end SmOffsetSurface::EvaluateNormal

/*******************************************************************//**
PURPOSE: Evaluate a point on the offset surface.

NOTES:
  Offset(uv) = ActingBase(uv) + dOffsetDistance * sNorm
  
    ActingBase = (crUV is in  pBaseSurface Domain)
                 ? m_pSurface
                 : m_pExtendedSurface
  when m_bAllowExtension == FALSE, input UV points beyond
       the BaseSurface domain are clamped to the domain and
       the extendedSurface is never used.
***********************************************************************/
SmStatus SmOffsetSurface::EvaluatePoint
  (const SmPoint2d & crUV,        // in : Target Domain Point
   SmPoint3d       & rPoint)      // out: Image Point
 const
{
  // locals
  double            dOffsetDistance = m_dOffsetDistance;
  SmPoint2d         sUV             = crUV;
  const SmSurface * pSurface        = m_pSurface;

  // when base surface is also an offset surface
  const SmOffsetSurface * pOff = SM_CAST_PTR(SmOffsetSurface,pSurface);
  if (pOff) 
    {
      // use the nested surface and combine offsets
      pSurface = pOff->m_pSurface;
      if (SM_CAST_PTR(SmOffsetSurface,pSurface) != NULL) 
        {
          // Should not be nesting offset surfaces
          SER(SM_ERR); 
        }
      dOffsetDistance = dOffsetDistance + pOff->m_dOffsetDistance;
    } // end nested offset surface check

  // when evalPoint is outside of BaseSurface domain - use ExtendedSurface if possible
  SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
  if (   !sDomain.ContainsPoint2d(sUV) 
      && m_pExtendedSurface) 
    {
      // replace base surface with extended surface
      pSurface = m_pExtendedSurface;
      sDomain = pSurface->GetNaturalUVDomain();
    }

  // when evalPoint is still outside of BaseSurface domain - - extend or clamp as possible
  if (!sDomain.ContainsPoint2d(sUV)) 
    {
      const SmContext *pContext = GetContext();

      // when point is within tolerance or extensions are not allowed
      if (   // old line: sDomain.ContainsPoint2d(sUV,dScaledTol)
             pContext == NULL 
          || !m_bAllowExtension) 
        {
          // clamp UVPoint to domain
          sUV = sDomain.ClampPoint2d(sUV);
        }
      else // extensions are allowed
        {
          // extend original base surface so that it contains given sUV
          SmBSplineSurface *pOriginal = SM_CAST_PTR(SmBSplineSurface,pSurface);
          NER(pOriginal);
          SmBSplineSurface *pExtended = NULL ;
          SER(sm_ExtendSurface(*GetContext(),
                                pOriginal,sUV,
                                m_bIsPeriodicOnU,
                                m_bIsPeriodicOnV,
                                m_lSingularities,
                                pExtended));

          // when BaseSurface was extended - store and use the extension in m_pExtendedSurface
          if (pExtended) 
            {
              pOriginal->Notify(SM_NO_PRE_EDIT, pOriginal, SM_NO_GET_OWNER(pOriginal), NULL);

              // const method modifies members here - use casting to sort things out
              if (m_pExtendedSurface) { delete m_pExtendedSurface; 
                                        ((SmOffsetSurface*)this)->m_pExtendedSurface = NULL ;
                                      }
              ((SmOffsetSurface*)this)->m_pExtendedSurface = pExtended;
              pSurface = pExtended;
            }
        } // end extensions are allowed branch
    } // end domainPoint not in Domain check
  
  // Optimization for zero offset distance
  if (dOffsetDistance == 0.0) 
    {
      SER(pSurface->EvaluatePoint(sUV,rPoint));
      return SM_SUCCESS;
    }

//cbi: don't do cross product, just call EvaluatePoint() (above, always)
//cbi  and EvaluateNormal(), on SER.

  // Evaluate the BaseSurface (either m_pBaseSurface or m_pExtendedSurface as needed)
  SmVector3d sDU, sDV;
  SmPoint3d sPnt;
  SER(pSurface->Evaluate1stDerivatives(sUV,TRUE,TRUE,sPnt,sDU,sDV));
  SmVector3d sNorm = sDU * sDV;
  if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) 
    {
      SER(pSurface->EvaluateNormal(sUV,TRUE,TRUE,sNorm));
    }
  SER(sNorm.Unitize());

  // compute and return the point
  rPoint = sPnt + dOffsetDistance * sNorm;

  // all done
  return SM_SUCCESS;

} // end SmOffsetSurface::EvaluatePoint

/*******************************************************************//**
PURPOSE: Evaluate a point and derivatives on the offset surface.

NOTES: 
   Maximum of three derivatives.
***********************************************************************/
SmStatus SmOffsetSurface::Evaluate
  (const SmPoint2d & crUV,   // in : param value to evaluate
   ULONG lHighestUDeriv,     // in : number of U derivatives
   ULONG lHighestVDeriv,     // in : number of V derivatives to compute
   SmBoolean bUFromLeft,     // in : if P is on U interval boundary
                             //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                             //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bVFromLeft,     // in : if P is on V interval boundary
                             //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                             //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bOnlyUpperHalf, // in : TRUE=compute upper half of matrix only
                             //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value 
                             //                [Dv --]       [Dv   Duv   ---]           (the memory has to be allocated)
                             //                              [Dvv  ---   ---]
   SmVector3d *aDerivatives, // out: matrix of evaluations values
                             //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]
                             //      2d organized: [D    Du    Duu    Duuu    ]  (the same no matter the value of)
                             //                    [Dv   Duv   Duuv   Duuuv   ]  (  bOnlyUpperHalf               )
                             //                    [Dvv  Duvv  Duuvv  Duuuvv  ]
                             //                    [Dvvv Duvvv Duuvvv Duuuvvv ]
                             //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]
  SmBoolean bNonZeroTangents,// in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                             //      FALSE= return exact tangent values
                             //      note: Surprisingly TRUE is the common choice because most tangent uses
                             //            are for their direction (Binorm, SurfNorm comps), but when the 
                             //            tangent is being used for its magnitude (like an arc-length comp)
                             //            then set this to FALSE.
                             //      default:[TRUE]
  SmBoolean )                // in : bDoZeroSampling = for internal use only, always set to TRUE, default:[TRUE]
 const 
{
  // pass the call along
  return( EvaluatePlainOrSimple( !bNonZeroTangents, // Not EvaluateSimple
                                  crUV, 
                                  lHighestUDeriv, 
                                  lHighestVDeriv, 
                                  bUFromLeft, 
                                  bVFromLeft,
                                  bOnlyUpperHalf, 
                                  aDerivatives )) ;

} // end SmOffsetSurface::Evaluate

/*******************************************************************//**
PURPOSE: Evaluate a point and derivatives on the offset surface,
            without trying to correct zero-length first derivatives.

NOTES: 
   Maximum of three derivatives.
***********************************************************************/
SmStatus SmOffsetSurface::EvaluateSimple
  (const SmPoint2d & crUV,    // in : target surface point
   ULONG lHighestUDeriv,      // in : Requested highest U derivative
   ULONG lHighestVDeriv,      // in : Requested highest V derivative
   SmBoolean bUFromLeft,      // in : if P is on U interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bVFromLeft,      // in : if P is on V interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bOnlyUpperHalf,  // in : TRUE=compute upper half of matrix only
                              //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value 
                              //                [Dv --]       [Dv   Duv   ---] 
                              //                              [Dvv  ---   ---]
   SmVector3d *aDerivatives)  // out: matrix of evaluations values                   
  const                       //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]     
                              //      2d organized: [D    Du    Duu    Duuu    Duuuu   ] 
                              //                    [Dv   Duv   Duuv   Duuuv   Duuuuv  ]
                              //                    [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]
                              //                    [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]
                              //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]

{
  return( EvaluatePlainOrSimple(TRUE, // Do EvaluateSimple
                                crUV, 
                                lHighestUDeriv, 
                                lHighestVDeriv, 
                                bUFromLeft, 
                                bVFromLeft,
                                bOnlyUpperHalf, 
                                aDerivatives ));

} // end SmOffsetSurface::EvaluateSimple

/*******************************************************************//**
PURPOSE: return surface domain

NOTES: 
***********************************************************************/
SmExtent2d SmOffsetSurface::GetNaturalUVDomain() const
{   
  SmExtent2d sRet = m_pSurface->GetNaturalUVDomain();

  if (m_pExtendedSurface) 
    { sRet = m_pExtendedSurface->GetNaturalUVDomain(); }
  return sRet;

} // end SmOffsetSurface::GetNaturalUVDomain

/*******************************************************************//**
PURPOSE: Local helper for EvaluatePlainOrSimple().

NOTES: 
***********************************************************************/
static SmStatus sm_FindNonsingularStepoff(
    const SmSurface  *pSurface,
    const SmPoint2d  &crUV,
    const SmExtent2d &crDomain,
          double      dBuLen, // in: just because they're already calculated.
          double      dBvLen,
          double      dTol,

          SmVector2d &rParamStep )  // out.
{
  // We're at a singularity in the base surface.
  // Formerly we just evaluated the base surface requesting zero-adjusted
  // first derivatives.  Analysis shows however that we also need limiting
  // directions for all zero derivatives, not just the first: if Bu is zero,
  // then so will Buu, Buuu, and Buuuu; but we need those directions as well.
  // In practice, it works better just to use the brute-force method of
  // stepping off from the singularity.  In this case, we know exactly
  // which direction to step.  This method also has the property that it
  // will mask instabilities in the data right at the singularity, for
  // example when the base surface is a general B-Spline, not an SmSphere
  // or SmCone.  [B247, B259]
  // Method: Get two equally-spaced parameter steps in a line from the
  // given parameter towards the center, then extrapolate linearly.

  // But: it's not always obvious which way to step to escape the singularity.
  // Normally a singularity is a pole, and to escape it you step in the
  // direction of the non-degenerate derivative.  But there can exist
  // singularities where you have to step in the direction of the degenerate
  // derivative to get out.  So we have to check both directions and see
  // which gives us a non-zero derivative.  [B323]

  SmBoolean bUIsDegen = ( dBuLen < dBvLen );
  SmPoint2d sStepUV;
  SmVector3d sMat[ 4 ];
  SmPoint2d sMidParam = crDomain.GetMid();

  // Loop three times, increasing stepoff size: .001, .01, .1.
  double dStepSize = 0.001;
  while ( dStepSize < 0.2 )
    {
      if ( bUIsDegen )  // The usual: a pole, step in other direction.
        { rParamStep.Set( 0, (sMidParam.y-crUV.y) * dStepSize ); }
      else
        { rParamStep.Set( (sMidParam.x-crUV.x) * dStepSize, 0 ); }

      sStepUV = crUV + rParamStep;

      // Evaluate the base surface to see where we lose the zero derivatives.
      // Check the expected direction first, which will handle almost every caes.
      SER_MSG(pSurface->EvaluateSimple(sStepUV, 1, 1, TRUE, TRUE, TRUE, sMat ),
                          _T("SmOffsetSurface::Evaluate(): BaseSurface::Evaluate failure"));

      // Return as soon as we find a good result.
      if (   (  bUIsDegen && sMat[2].Length() > dTol )    // sMat[2] is Su,
          || ( !bUIsDegen && sMat[1].Length() > dTol ) )  // sMat[1] is Sv.
        { return SM_SUCCESS; }


      // The usual direction didn't work.  Try the other.

      if ( bUIsDegen )
        { rParamStep.Set( (sMidParam.x-crUV.x) * dStepSize, 0 ); }
      else
        { rParamStep.Set( 0, (sMidParam.y-crUV.y) * dStepSize ); }

      sStepUV = crUV + rParamStep;

      // Evaluate the base surface to see where we lose the zero derivatives.
      // Check the expected direction first, which will handle almost every caes.
      SER_MSG(pSurface->EvaluateSimple(sStepUV, 1, 1, TRUE, TRUE, TRUE, sMat ),
                              _T("SmOffsetSurface::Evaluate(): BaseSurface::Evaluate failure"));

      // Return as soon as we find a good result.
      if (   (  bUIsDegen && sMat[2].Length() > dTol )
          || ( !bUIsDegen && sMat[1].Length() > dTol ) )
        { return SM_SUCCESS; }

      // Try a bigger stepoff size.
      dStepSize *= 10;

    }  // end iterations increasing stepoff size.

  // Neither direction worked.  We can't get an offset direction.
  return SM_ERR;

}  // end static sm_FindNonsingularStepoff

/*******************************************************************//**
PURPOSE: Private sub for the common functionality of
    Evaluate and EvaluateSimple.

NOTES: This evaluator was extended to handle the following cases.  However,
       the job is not complete, special case handling of negative offsets
       at C0 poles (on cones) needs to be handled.
  Handled: Evaluations at nonSingular suface locations (far from poles) - all cases
           Pole evaluations; bDoEvalSimple = TRUE ; Pole is G1 (a sphere); positive and negative offsets
           Pole evaluations; bDoEvalSimple = FALSE; Pole is G1 (a sphere); positive and negative offsets
           Pole evaluations; bDoEvalSimple = TRUE ; Pole is C0 (a cone); positive offsets
           Pole evaluations; bDoEvalSimple = FALSE; Pole is C0 (a cone); positive offsets
  Needed:  Pole evaluations; bDoEvalSimple = TRUE ; Pole is C0 (a cone); negative offsets
           Pole evaluations; bDoEvalSimple = FALSE; Pole is C0 (a cone); negative offsets
***********************************************************************/
SmStatus SmOffsetSurface::EvaluatePlainOrSimple
  (SmBoolean bDoEvalSimple,   // in : TRUE=EvaluateSimple, FALSE=Evaluate, adjust zero tangents.
   const SmPoint2d & crUV,    // in : target surface point
   ULONG lHighestUDeriv,      // in : Requested highest U derivative
   ULONG lHighestVDeriv,      // in : Requested highest V derivative
   SmBoolean bUFromLeft,      // in : if P is on U interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bVFromLeft,      // in : if P is on V interval boundary
                              //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                              //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bOnlyUpperHalf,  // in : TRUE=compute upper half of matrix only
                              //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value 
                              //                [Dv --]       [Dv   Duv   ---] 
                              //                              [Dvv  ---   ---]
   SmVector3d *aDerivatives)  // out: matrix of evaluations values                   
                              //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]     
                              //      2d organized: [D     Du     Duu     Duuu     ] 
                              //                    [Dv    Duv    Duuv    ....     ]
                              //                    [Dvv   Duvv   ....    ....     ]
                              //                    [Dvvv  ....   ....    ....     ]
                              //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv,,.. Duu, Duuv,...]
      const
{
  // locals
  double           dOffsetDistance = m_dOffsetDistance;
  SmPoint2d        sUV             = crUV;  // needed so that &crUV can be same memory as aDerivatives
  const SmSurface *pSurface        = m_pSurface;

  // get number of BaseSurface derivatives needed to compute the requested OffsetSurface values
  ULONG lHighestOffsetDeriv =    bOnlyUpperHalf
                              ?  smos_Max(lHighestUDeriv,lHighestVDeriv)
                              : (lHighestUDeriv + lHighestVDeriv) ;
  ULONG lHighestBaseDeriv   = lHighestOffsetDeriv + 1 ;

  // check input - Offsets only support 3 derivatives
  const ULONG MAX_DERIVS = 3 ;
  if(lHighestOffsetDeriv > MAX_DERIVS)
    { SER(SM_ERR) ; }

  // when offsetting an OffsetSurface - combine OffsetDistances into a single step
  const SmOffsetSurface *pOff = SM_CAST_PTR( SmOffsetSurface, pSurface );
  if (pOff)
    {
      // use the nested surface and combine offsets
      pSurface = pOff->m_pSurface;
      if (SM_CAST_PTR(SmOffsetSurface,pSurface) != NULL) 
        {
          // Should not be nesting offset surfaces
          SER(SM_ERR); 
        }
      dOffsetDistance = dOffsetDistance + pOff->m_dOffsetDistance;
    }

  // Check for evaluating outside of BaseSurface domain.
  // If so, check whether we have an extended surface.
  // If not, then we either clamp the point or try to
  // create an extended surface, depending on some
  // other criteria, below.

  // When EvalPoint is outside of baseDomain - use ExtendedSurface if possible
  SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
  if (   !sDomain.ContainsPoint2d(sUV) 
      && m_pExtendedSurface) 
    {
      // use the ExtendedSurface
      pSurface = m_pExtendedSurface;
      sDomain  = pSurface->GetNaturalUVDomain();
    }

  // When EvalPoint is still outside of baseDomain - extend BaseSurface or clamp EvalPoint as possible
  if (!sDomain.ContainsPoint2d(sUV)) 
    {
      const SmContext *pContext   = GetContext();

      // when extensions are not allowed
      if(   // old line: sDomain.ContainsPoint2d(sUV,dScaledTol)
            pContext == NULL
         || !m_bAllowExtension) 
        {
          // just clamp the UVPoint
          sUV = sDomain.ClampPoint2d(sUV);
        }
      else // extensions are allowed
        {
          // extend original BaseSurface nonPeriodic Directions 
          // and wrap sUV coordinates in BaseSurface Periodice Directions
          // to get a surface containing target sUV
          SmBSplineSurface *pOriginal = SM_CAST_PTR(SmBSplineSurface,pSurface);
          NER(pOriginal);
          SmBSplineSurface *pExtended = NULL ;
          SER(sm_ExtendSurface(*GetContext(), 
                               pOriginal, 
                               sUV,
                               m_bIsPeriodicOnU, 
                               m_bIsPeriodicOnV, 
                               m_lSingularities,
                               pExtended));
          
          // when the surface was extended - store and use the extension
          if (pExtended) 
            {
              pOriginal->Notify(SM_NO_PRE_EDIT, pOriginal, SM_NO_GET_OWNER(pOriginal), NULL);

              // const method modifies member values - use casting to sort things out. 
              if (m_pExtendedSurface) { delete m_pExtendedSurface ; 
                                        ((SmOffsetSurface*)this)->m_pExtendedSurface = NULL ;
                                      }
              ((SmOffsetSurface*)this)->m_pExtendedSurface = pExtended;
              pSurface = pExtended;
            }
        } // end extensions are allowed branch
    } // end UVPoint not in Surf->Domain check

  // Done with out-of-domain checking: sUV is now within pSurface domain, sDomain.


  // zero offset distance optimization.
  if ( dOffsetDistance == 0.0 )
    {
      if ( bDoEvalSimple )
        {
          SER(pSurface->EvaluateSimple(sUV,
                                       lHighestUDeriv, lHighestVDeriv, 
                                       bUFromLeft,     bVFromLeft,
                                       bOnlyUpperHalf, aDerivatives ));
        }
      else
        {
          SER(pSurface->Evaluate(sUV,
                                 lHighestUDeriv, lHighestVDeriv, 
                                 bUFromLeft, bVFromLeft,
                                 bOnlyUpperHalf, aDerivatives,
                                 TRUE )); // True: adjust to non-zero first derivatives
        }
      return SM_SUCCESS;
    } // end offset distance=0.0 optimizaton

  // arrive here for evaluations on offset surfaces


  // Evaluate the baseSurface - with enough memory for a max number of evaluations
  SmStatus eStat = SM_SUCCESS ;
  SmVector3d sMat[ (2*MAX_DERIVS+1) * (2*MAX_DERIVS+1) ];

  // Get the actual, un-corrected derivatives.  If we're at a singularity
  // in the base surface, then we deal with that ourselves (see below).  [B247, B259]
  //if ( bDoEvalSimple )

  SER_MSG(pSurface->EvaluateSimple(sUV,
                                   lHighestBaseDeriv, 
                                   lHighestBaseDeriv, 
                                   TRUE, 
                                   TRUE, 
                                   TRUE, 
                                   sMat ),
                          _T("SmOffsetSurface::Evaluate(): BaseSurface::Evaluate failure")); 


  // get BaseSurface evaluation vectors
  SmVector3d *B, *Bv = NULL, *Bvv = NULL, *Bvvv = NULL, *Bvvvv = NULL, *Bu = NULL, *Buv = NULL, *Buvv = NULL, *Buvvv = NULL, *Buu = NULL, *Buuv = NULL, *Buuvv = NULL, *Buuu = NULL, *Buuuv = NULL, *Buuuu = NULL;
  B  = &sMat[0];
  if ( lHighestBaseDeriv >= 1 ) { Bv = &sMat[1];
                                  Bu = &sMat[lHighestBaseDeriv+1];
                                }
  if ( lHighestBaseDeriv >= 2 ) { Bvv = &sMat[2];
                                  Buv = &sMat[1*(lHighestBaseDeriv+1)+1];
                                  Buu = &sMat[2*(lHighestBaseDeriv+1)  ];
                                }
  if ( lHighestBaseDeriv >= 3 ) { Bvvv = &sMat[3];
                                  Buvv = &sMat[1*(lHighestBaseDeriv+1)+2];
                                  Buuv = &sMat[2*(lHighestBaseDeriv+1)+1];
                                  Buuu = &sMat[3*(lHighestBaseDeriv+1)  ];
                                }
  if ( lHighestBaseDeriv >= 4 ) { Bvvvv = &sMat[4];
                                  Buvvv = &sMat[1*(lHighestBaseDeriv+1)+3];
                                  Buuvv = &sMat[2*(lHighestBaseDeriv+1)+2];
                                  Buuuv = &sMat[3*(lHighestBaseDeriv+1)+1];
                                  Buuuu = &sMat[4*(lHighestBaseDeriv+1)  ];
                                }

  // For offset, we absolutely need a good surface normal.
  // Tolerance: Use what is in SmBSplineSurface::Evaluate(). [B247]
  // (Actually, if anything, this could be bigger here.)
  double dTol = 10.0 * SM_EFF_ZERO * (1.0 + (*B).GetMaxDimension());

  double dBuLen = (*Bu).Length();
  double dBvLen = (*Bv).Length();

  if(   (bDoEvalSimple == FALSE)  // only use approx values at poles when not doing a Simple evaluation
     && (dBuLen < dTol || dBvLen < dTol ))
    {
      // At a singularity.  Look for a stepoff uv.
      SmVector2d sParamStep;
      eStat = sm_FindNonsingularStepoff(pSurface, crUV, sDomain,
                                                 dBuLen, dBvLen, dTol,
                                                 sParamStep );

      // If that didn't work, we can't work.
      if ( eStat != SM_SUCCESS )
        { SER( eStat ); }

      // At this point, sParamStep is set to a workable stepoff.
      SmPoint2d sStepUV = crUV + sParamStep;
      SmVector3d sMat1[ (2*MAX_DERIVS+1) * (2*MAX_DERIVS+1) ];
      SmVector3d sMat2[ (2*MAX_DERIVS+1) * (2*MAX_DERIVS+1) ];

      SER( this->EvaluatePlainOrSimple(bDoEvalSimple, sStepUV,
         lHighestUDeriv, lHighestVDeriv, bUFromLeft, bVFromLeft, bOnlyUpperHalf, sMat1 ));

      sStepUV = crUV + 2 * sParamStep;

      SER( this->EvaluatePlainOrSimple(bDoEvalSimple, sStepUV, 
         lHighestUDeriv, lHighestVDeriv, bUFromLeft, bVFromLeft, bOnlyUpperHalf, sMat2 ));

      SmVector3d v2, v1, v0;  // for extrapolation

      // Position
      ULONG idx = 0;
      v1 = sMat1[ idx ];
      v2 = sMat2[ idx ];
      v0 = 2 * v1 - v2;  // Linear extraplation

      aDerivatives[ idx ] = v0;

      // Su
      if ( lHighestUDeriv >= 1 )
      {
          idx = lHighestVDeriv + 1;
          v1 = sMat1[ idx ];
          v2 = sMat2[ idx ];

          // For the first derivatives, if non-zero-corrected derivatives
          // are requested, calculate the limiting values of the directions,
          // and give them very small magnitudes.
          v0 = 2 * v1 - v2;
          if ( ! bDoEvalSimple && v0.Length() < dTol )
          {
              SER( v1.Unitize() );
              SER( v2.Unitize() );
              v0 = 2 * v1 - v2;
              v0.Unitize();
              v0 *= dTol * 1.1;
          }
          aDerivatives[ idx ] = v0;
      }
      // Sv
      if ( lHighestVDeriv >= 1 )
      {
          idx = 1;
          v1 = sMat1[ idx ];
          v2 = sMat2[ idx ];

          v0 = 2 * v1 - v2;
          if ( ! bDoEvalSimple && v0.Length() < dTol )
          {
              SER( v1.Unitize() );
              SER( v2.Unitize() );
              v0 = 2 * v1 - v2;
              v0.Unitize();
              v0 *= dTol * 1.1;
          }
          aDerivatives[ idx ] = v0;
      }

      // Suu
      if ( lHighestUDeriv >= 2 )
      {
          idx = 2*(lHighestVDeriv+1);
          v1 = sMat1[ idx ];
          v2 = sMat2[ idx ];
          v0 = 2 * v1 - v2;
          aDerivatives[ idx ] = v0;
      }

      // Suv
      if (   ( bOnlyUpperHalf && lHighestOffsetDeriv >= 2 )
          || (!bOnlyUpperHalf && (   lHighestUDeriv >= 1
                                  && lHighestVDeriv >= 1) )
         )
      {
          idx = lHighestVDeriv+2;
          v1 = sMat1[ idx ];
          v2 = sMat2[ idx ];
          v0 = 2 * v1 - v2;
          aDerivatives[ idx ] = v0;
      }

      // Svv
      if ( lHighestVDeriv >= 2 )
      {
          idx = 2;
          v1 = sMat1[ idx ];
          v2 = sMat2[ idx ];
          v0 = 2 * v1 - v2;
          aDerivatives[ idx ] = v0;
      }

      // Suuu
      if (lHighestUDeriv >= 3)
      {
          idx = 3*(lHighestVDeriv+1);
          v1 = sMat1[ idx ];
          v2 = sMat2[ idx ];
          v0 = 2 * v1 - v2;
          aDerivatives[ idx ] = v0;
      }

      // Suuv
      if (   ( bOnlyUpperHalf && lHighestOffsetDeriv >= 3)
          || (!bOnlyUpperHalf && (   lHighestUDeriv >= 2
                                  && lHighestVDeriv >= 1)))
      {
          idx = 2*(lHighestVDeriv+1)+1;
          v1 = sMat1[ idx ];
          v2 = sMat2[ idx ];
          v0 = 2 * v1 - v2;
          aDerivatives[ idx ] = v0;
      }

      // Suvv
      if (   ( bOnlyUpperHalf && lHighestOffsetDeriv >= 3)
          || (!bOnlyUpperHalf && (   lHighestUDeriv >= 1
                                  && lHighestVDeriv >= 2)))
      {
          idx = 1*(lHighestVDeriv+1)+2;
          v1 = sMat1[ idx ];
          v2 = sMat2[ idx ];
          v0 = 2 * v1 - v2;
          aDerivatives[ idx ] = v0;
      }

      // Svvv
      if (lHighestVDeriv >= 3)
      {
          idx = 3;
          v1 = sMat1[ idx ];
          v2 = sMat2[ idx ];
          v0 = 2 * v1 - v2;
          aDerivatives[ idx ] = v0;
      }

      return SM_SUCCESS;

    }  // end singularity handling.

  // arrive here when evaluation is far from a pole, or on a pole and doing SimpleEvals

  // The following two checks for degeneracy will presumably be hit
  // only if Bu and Bv are non-zero but parallel
  //      or doing a SImpleEval at a pole
  SmVector3d sDU, sDV;

  // Compute the normal (N) and its length (L)
  SmVector3d N = *Bu * *Bv;
  double     L = N.Length();

  // L will be small at poles or when Bu and Bv are nearly parallel
  if ( L < dTol )
    {
      SmPoint3d sPnt;
      // This will do the zero-derivative correction.
      eStat = pSurface->Evaluate1stDerivatives(sUV, TRUE, TRUE, sPnt, sDU, sDV);
      if ( eStat == SM_SUCCESS )
        {
          N = sDU * sDV;
          L = N.Length();

          // when doing general evals - Also reset our 1st derivs. [B247]
          if(bDoEvalSimple == FALSE)
            {
              Bu = &sDU;
              Bv = &sDV;
            } // end not doing SimpleEval check
        } // end Evaluate1stDerivatives call success check
    }  // end surface Normal size is less than tol check

  // when not doing SimpleEvals and that didn't work, try a different way
  if(bDoEvalSimple == FALSE)
    {
      // Check for NonZero Normal and 1st deriv vectors
      double BuLength = (*Bu).Length() ;
      double BvLength = (*Bv).Length() ;
      double Lt       = smos_3Min(L, BuLength, BvLength) ;

      // when any normal or 1st deriv vec is zero
      if ( Lt < dTol )
        {
          SmVector3d sNorm;
          eStat = pSurface->EvaluateNormal( sUV, bUFromLeft, bVFromLeft, sNorm );
          if ( eStat == SM_SUCCESS )
            {
              N = sNorm ;
              L = N.Length() ;  // L will be 1.0, EvaluateNormal returns a normalized sNorm vector

              // Also reset our 1st derivs. [B247]
              if ( BuLength < BvLength )
                {
                  sDU = *Bv * N ;
                  sDU.Unitize();
                  sDU *= dTol * 1.1; // Make it big enough: no other info on magnitude.
                  Bu = &sDU ;
                }
              else
                {
                  sDV = N * *Bu;
                  sDV.Unitize();
                  sDV *= dTol * 1.1;
                  Bv = &sDV ;
                } // end BvLength is shorter than BuLength branch
            } // end EvaluateNormal success check
          SER(eStat);
        } // end any Normal or a 1st deriv vec is zero check
    } // end not doing SimpleEval check

  // only proceed when N is nonZero
  if(L < SM_EFF_ZERO) // Leave this tol at SM_EFF_ZERO: that's where output becomes invalid.
    { SER( SM_ERR ) ; }

  // Ok, now we've got a good normal, and good BaseSurface 1st derivative values.
  // Next: evaluate the offset position and derivatives.

  // For an explanation of the algorithm for taking derivatives of a quotient
  // (i.e., a unitized vector), see SmVector3d::UnitizedDerivative().

  // Cross product and magnitude, and all their derivs.  Terminology:
  // C is the cross product Bu*Bv
  // m is its magnitude
  // N is the unit normal, C/m.

  SmVector3d C, Cu, Cv, Cuu, Cuv, Cvv, Cuuu, Cuuv, Cuvv, Cvvv;
  double     m, mu = 0.0, mv = 0.0, muu = 0.0, muv = 0.0, mvv = 0.0, muuu, muuv, muvv, mvvv;
  SmVector3d    Nu, Nv, Nuu, Nuv, Nvv, Nuuu, Nuuv, Nuvv, Nvvv ;

  // Might use UnitizedDerivative() except that it doesn't work for
  // mixed partials, plus it takes a vector length on each call
  // which we only have to do once.

  C = N ;
  m = C.Length();

  N = C / m;

  // Position
  aDerivatives[0] = *B + dOffsetDistance * N;

  // Su
  if ( lHighestUDeriv >= 1 )
  {
      Cu = *Buu * *Bv + *Bu * *Buv;
      mu = N.Dot( Cu );
      Nu = ( Cu  -  mu * N ) / m;
      aDerivatives[ lHighestVDeriv+1 ] = *Bu + dOffsetDistance * Nu;
  }

  // Sv
  if ( lHighestVDeriv >= 1 )
  {
      Cv = *Buv * *Bv + *Bu * *Bvv;
      mv = N.Dot( Cv );
      Nv = ( Cv  -  mv * N ) / m;
      aDerivatives[                1 ] = *Bv + dOffsetDistance * Nv;
  }

  // Suu
  if ( lHighestUDeriv >= 2 )
  {
      Cuu = *Buuu * *Bv  +  2 * *Buu * *Buv  +  *Bu * *Buuv;
      muu = ( Cu.Dot( Cu )  +  C.Dot( Cuu )  -  mu*mu ) / m;
      Nuu = ( Cuu  -      2 * mu * Nu      -  muu * N  ) / m;
      aDerivatives[ 2*(lHighestVDeriv+1) ] = *Buu + dOffsetDistance * Nuu;
  }

  // Suv
  if (   ( bOnlyUpperHalf && lHighestOffsetDeriv >= 2 )
      || (!bOnlyUpperHalf && (   lHighestUDeriv >= 1
                              && lHighestVDeriv >= 1) )
     )
  {
      Cuv = *Buuv * *Bv  +  *Buu * *Bvv  /* +  *Buv * *Buv */  +  *Bu * *Buvv;
      muv = ( Cu.Dot( Cv )  +  C.Dot( Cuv )  -  mu*mv ) / m;
      Nuv = ( Cuv  -  mu * Nv  -  mv * Nu  -  muv * N  ) / m;
      aDerivatives[    lHighestVDeriv+2  ] = *Buv + dOffsetDistance * Nuv;
  }

  // Svv
  if ( lHighestVDeriv >= 2 )
  {
      Cvv = *Buvv * *Bv  +  2 * *Buv * *Bvv  +  *Bu * *Bvvv;
      mvv = ( Cv.Dot( Cv )  +  C.Dot( Cvv )  -  mv*mv ) / m;
      Nvv = ( Cvv  -      2 * mv * Nv      -  mvv * N  ) / m;
      aDerivatives[                   2  ] = *Bvv + dOffsetDistance * Nvv;
  }

  // Suuu
  if (lHighestUDeriv >= 3)
  {
      Cuuu = *Buuuu * *Bv  +  3.0 * *Buuu * *Buv  +  3.0 * *Buu * *Buuv  +  *Bu * *Buuuv;
      muuu = ( 3 * Cu.Dot( Cuu )  +  C.Dot( Cuuu )  -  3 * mu * muu ) / m;
      Nuuu = ( Cuuu - 3 * mu * Nuu  -  3 * muu * Nu  - muuu * N ) / m;
      aDerivatives[ 3*(lHighestVDeriv+1)   ] = *Buuu + dOffsetDistance * Nuuu;
  }

  // Suuv
  if (   ( bOnlyUpperHalf && lHighestOffsetDeriv >= 3)
      || (!bOnlyUpperHalf && (   lHighestUDeriv >= 2
                              && lHighestVDeriv >= 1)))
  {
      Cuuv = *Buuuv * *Bv  +  *Buuu * *Bvv  +  3.0 * *Buuv * *Buv  +  2.0 * *Buu * *Buvv  +  *Bu * *Buuvv;
      muuv = ( Cuu.Dot( Cv ) + 2 * Cu.Dot( Cuv ) + C.Dot( Cuuv ) - muu*mv - 2*mu*muv ) / m;
      Nuuv = ( Cuuv - mv * Nuu - 2 * muv * Nu - 2 * mu * Nuv - muu * Nv - muuv * N ) / m;
      aDerivatives[ 2*(lHighestVDeriv+1)+1 ] = *Buuv + dOffsetDistance * Nuuv;
  }

  // Suvv
  if (   ( bOnlyUpperHalf && lHighestOffsetDeriv >= 3)
      || (!bOnlyUpperHalf && (   lHighestUDeriv >= 1
                              && lHighestVDeriv >= 2)))
  {
      Cuvv = *Buuvv * *Bv  +  2.0 * *Buuv * *Bvv  +  3.0 * *Buvv * *Buv  +  *Buu * *Bvvv  +  *Bu * *Buvvv;
      muvv = ( Cvv.Dot( Cu ) + 2 * Cv.Dot( Cuv ) + C.Dot( Cuvv ) - mvv*mu - 2*mv*muv ) / m;
      Nuvv = ( Cuvv - mu * Nvv - 2 * muv * Nv - 2 * mv * Nuv - mvv * Nu - muvv * N ) / m;
      aDerivatives[ 1*(lHighestVDeriv+1)+2 ] = *Buuv + dOffsetDistance * Nuvv;
  }

  // Svvv
  if (lHighestVDeriv >= 3)
  {
      Cvvv = *Buvvv * *Bv  +  3.0 * *Buvv * *Bvv  +  3.0 * *Buv * *Bvvv  +  *Bu * *Bvvvv;
      mvvv = ( 3 * Cv.Dot( Cvv )  +  C.Dot( Cvvv )  -  3 * mv * mvv ) / m;
      Nvvv = ( Cvvv - 3 * mv * Nvv  -  3 * mvv * Nv  - mvvv * N ) / m;
      aDerivatives[                      3 ] = *Buuv + dOffsetDistance * Nvvv;
  }

//        // - old computations: quotient rule.
//        // Compute du
//        if (lHighestUDeriv > 0) 
//          {
//            Su = Buu * Bv + Bu * Buv;
//            Lu = N.Dot(Su) / L;
//            Nu = (L*Su - Lu*N) / (L*L);
//            aDerivatives[lHighestVDeriv+1] = Bu + dOffsetDistance * Nu;
//          }
//      
//        // Compute dv
//        if (lHighestVDeriv > 0) 
//          {
//            Sv = Buv * Bv + Bu * Bvv;
//            Lv = N.Dot(Sv) / L;
//            Nv = (L*Sv - Lv*N) / (L*L);
//            aDerivatives[1] = Bv + dOffsetDistance * Nv;
//          }
//      
//        // Compute duu
//        if (lHighestUDeriv > 1) 
//          {
//            Suu = Buuu*Bv + 2.0 * Buu * Buv + Bu * Buuv;
//            Luu = (N.Dot(Suu) + Su.Dot(Su) - Lu*Lu) / L;
//            Nuu = (L*L*Suu - 2.0*L*Lu*Su + (2.0*Lu*Lu - L*Luu) * N) / (L*L*L);
//            aDerivatives[2*(lHighestVDeriv+1)] = Buu + dOffsetDistance * Nuu;
//          }
//      
//        // Compute dvv
//        if (lHighestVDeriv > 1) 
//          {
//            Svv = Buvv * Bv + 2.0 * Buv * Bvv + Bu * Bvvv;
//            Lvv = (N.Dot(Svv) + Sv.Dot(Sv) - Lv*Lv) / L;
//            Nvv = (L*L*Svv - 2.0*L*Lv*Sv + (2.0*Lv*Lv - L*Lvv) * N) / (L*L*L);
//            aDerivatives[2] = Bvv + dOffsetDistance * Nvv;
//          }
//      
//        // Compute duv
//        if ((lHighestUDeriv > 1 && lHighestVDeriv > 1) ||
//            (lHighestUDeriv == 1 && lHighestVDeriv == 1 && !bOnlyUpperHalf) ) 
//          { 
//            Suv = Buuv*Bv + Buu*Bvv + Bu*Buvv;   // remembering Buv*Bvu = 0
//            Luv = (N.Dot(Suv) + Su.Dot(Sv) - Lu*Lv) / L;
//            Nuv = (L*L*Suv - L*Lv*Su - L*Lu*Sv + (2.0*Lu*Lv - L*Luv) * N) / (L*L*L);
//            aDerivatives[lHighestVDeriv+2] = Buv + dOffsetDistance * Nuv;
//          }
//      
//        if (lHighestUDeriv > 1 && lHighestVDeriv > 1 && !bOnlyUpperHalf) 
//          {
//            SER(SM_ERR);  // Sorry we don't compute higher order partials for
//            // the mixed derivatives.
//          }
//
// Suuu = Buuu + dOffsetDistance * (  (  (  (Nuuu*L + Nuu*Lu - Nu*Luu - N*Luuu) * L**2
//                                        + (Nuu*L - N*Luu) * 2.0 * L * Lu
//                                        - (Nuu*L + Nu*Lu - Nu*Lu - N*Luu) * 2.0 * L * Lu
//                                        - (Nu*L - N*Lu) * 2.0 * (Lu * Lu + L * Luu)) 
//                                     * (L**4)
//                                     - (  (Nuu*L - N*Luu) * L**2
//                                        - (Nu*L  - N*Lu) * 2.0 * L * Lu) 
//                                     * (4.0 * L**3 * Lu))
//                                  / (L**8))
//
// Suuv = Buuv + dOffsetDistance * (  (  (  (Nuuv*L - Nuu*Lv - Nuv*Lu - Nu*Luv) * L**2
//                                        + (Nuv*Lu - Nu*Luv - Nv*Luu - N*Luuv) * L**2
//                                        + (Nuv*L - Nv*Lu) * 2.0 * L * Lu
//                                        + 2.0 * N  * Luu * L  * Lv 
//                                        + 2.0 * N  * Lu  * Lu * Lv) 
//                                     * (L**4)
//                                     - (  (Nuv*L - Nu*Lv - Nv*Lu - N*Luv) * L**2
//                                        + 2.0 * N*Lu * L * Lv) 
//                                     * (4.0 * L**3 * Lu)) 
//                                  / (L**8))
//
// Suvv = Buvv + dOffsetDistance * (  (  (  (Nuvv*L - Nvv*Lu - Nuv*Lv - Nv*Luv) * L**2
//                                        + (Nuv*Lv - Nv*Luv - Nu*Lvv - N*Luvv) * L**2
//                                        + (Nuv*L - Nu*Lv) * 2.0 * L * Lv
//                                        + 2.0 * N  * Lvv * L  * Lu 
//                                        + 2.0 * N  * Lv  * Lv * Lu)
//                                     * (L**4)
//                                     - (  (Nuv*L - Nv*Lu - Nu*Lv - N*Luv) * L**2
//                                        + 2.0 * N*Lv * L * Lu) 
//                                     * (4.0 * L**3 * Lv)) 
//                                  / (L**8))
//
// Svvv = Bvvv + dOffsetDistance * (  (  (  (Nvvv*L + Nvv*Lv - Nv*Lvv - N*Lvvv) * L**2
//                                        + (Nvv*L - N*Lvv) * 2.0 * L * Lv
//                                        - (Nvv*L + Nv*Lv - Nv*Lv - N*Lvv) * 2.0 * L * Lv
//                                        - (Nv*L - N*Lv) * 2.0 * (Lv * Lv + L * Lvv)) 
//                                     * (L**4)
//                                     - (  (Nvv*L - N*Lvv) * L**2
//                                        - (Nv*L  - N*Lv) * 2.0 * L * Lv) 
//                                     * (4.0 * L**3 * Lv))
//                                  / (L**8))
//      // end old computations

  // Check for zero first derivatives if requested (not "simple").
  if ( !bDoEvalSimple )
    {
      // Use the same tolerance that's used in SmBSplineSurface::Evaluate(),
      // because many calls go through there.

      double dTolerance = SM_EFF_ZERO * (1.0 + aDerivatives[0].GetMaxDimension());

      // Check d/du
      if (    lHighestUDeriv > 0
           && aDerivatives[lHighestVDeriv+1].LengthSquared() < dTolerance * dTolerance)
        {
          SmPoint3d sPnt;
          // This will calculate non-zero derivatives:
          eStat = pSurface->Evaluate1stDerivatives(sUV, TRUE, TRUE, sPnt, sDU, sDV);
          if ( eStat == SM_SUCCESS && sDU.LengthSquared() > dTolerance * dTolerance)
            {
              // Make it as short as possible, long enough not to get flagged.
              sDU.Unitize();
              aDerivatives[lHighestVDeriv+1] = sDU * dTolerance * 1.1;
            }
        }

      // Check d/dv
      if (    lHighestVDeriv > 0
           && aDerivatives[1].LengthSquared() < dTolerance * dTolerance)
        {
          SmPoint3d sPnt;
          // This will calculate non-zero derivatives:
          eStat = pSurface->Evaluate1stDerivatives(sUV, TRUE, TRUE, sPnt, sDU, sDV);
          if ( eStat == SM_SUCCESS && sDV.LengthSquared() > dTolerance * dTolerance)
            {
              // Make it as short as possible, long enough not to get flagged.
              sDV.Unitize();
              aDerivatives[1] = sDV * dTolerance * 1.1;
            }
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmOffsetSurface::EvaluatePlainOrSimple

/*******************************************************************//**
PURPOSE: Reverse one of the parameterizations of the surface.

NOTES: 
***********************************************************************/
SmStatus SmOffsetSurface::Reverse(SmSurfParamType eSurfParam)
{
    SER(m_pSurface->Reverse(eSurfParam));
    if (m_pExtendedSurface) SER(m_pExtendedSurface->Reverse(eSurfParam));
    return SM_SUCCESS;

} // end SmOffsetSurface::Reverse

/*******************************************************************//**
PURPOSE: Split an offset surface.

NOTES: 
***********************************************************************/
SmStatus SmOffsetSurface::SplitAt
  (const SmContext & crContext,        // in : context for new object construction     
   double            dParam,           // in : split parameter                         
   SmSurfParamType   eSurfParam,       // in : oneof SM_SP_U = split u domain at dParam
                                       //            SM_SP_V = split v domain at dParam
   SmSurface      *& rpLeftSurface,    // out: SmBSplineSurface (left  or bot)         
   SmSurface      *& rpRightSurface)   // out: SmBSplineSurface (right or top)         
{
  // split Base Surface
  SmSurface *pSurLeft, *pSurRight;
  SER(m_pSurface->SplitAt(crContext,dParam,eSurfParam,pSurLeft,pSurRight));

  // build left offset surface from left baseSurface split
  SmOffsetSurface *pLeftOff;
  pLeftOff                    = new (crContext) SmOffsetSurface(m_dOffsetDistance,*pSurLeft,TRUE);
  pLeftOff->m_bAllowExtension = m_bAllowExtension;
  
  // GWC: Surface ownership is not mandatory and used for topology
  //   pSurLeft->SetOwner(pLeftOff);

  // build right offset surface from right baseSurface split
  SmOffsetSurface *pRightOff;
  pRightOff                    = new (crContext) SmOffsetSurface(m_dOffsetDistance,*pSurRight,TRUE);
  pRightOff->m_bAllowExtension = m_bAllowExtension;
  // GWC: Surface ownership is not mandatory and used for topology
  //   pSurRight->SetOwner(pRightOff);

  // split extended surfaces as well 
  if (m_pExtendedSurface) 
    {
      SER(m_pSurface->SplitAt(crContext,dParam,eSurfParam,pSurLeft,pSurRight));
      pLeftOff->m_pExtendedSurface  = pSurLeft;
      pRightOff->m_pExtendedSurface = pSurRight;
      // GWC: Surface ownership is not mandatory and used for topology
      //   pSurLeft->SetOwner(pLeftOff);
      // GWC: Surface ownership is not mandatory and used for topology
      //   pSurRight->SetOwner(pRightOff);
    }

  // set output
  rpLeftSurface = pLeftOff;
  rpRightSurface = pRightOff;

  Notify( SM_NO_SPLIT, rpLeftSurface, rpRightSurface, GetOwner() );

  // all done
  return SM_SUCCESS;

} // end SmOffsetSurface::SplitAt

/*******************************************************************//**
PURPOSE: Invert a 3d point on the surface, using the STEP representation if appropriate.

NOTES: 
   If the result is on the seam of a closed surface, and a guess was
   given, the output will be set to whichever side of the seam is closer
   to the guess.  If no guess was given, it will be set to the low end
   of the closed domain.

   Returns SM_ERR if the point is not on the surface within tolerance.
***********************************************************************/
SmStatus SmOffsetSurface::STEPInversion
 (const SmExtent2d & crAnalUVDomain,
  const SmPoint3d  & crPointOnSurf,
  double             dDistanceTolerance,
  SmPoint2d        & rdAnalUVParameter,
  SmLocationType   & reLocation,
  SmPoint2d        * pUVGuess) 
 const
{
  double dOffsetTol = dDistanceTolerance + smos_Fabs( m_dOffsetDistance );

  SmStatus eStat = m_pSurface->STEPInversion( crAnalUVDomain,
                                              crPointOnSurf,
                                              dOffsetTol,
                                              rdAnalUVParameter,
                                              reLocation,
                                              pUVGuess );

  if ( eStat != SM_SUCCESS || reLocation == SM_LT_EXTERIOR )
  {
      if ( m_pExtendedSurface != NULL )
      {
          eStat = m_pExtendedSurface->STEPInversion( crAnalUVDomain,
                                 crPointOnSurf,
                                 dDistanceTolerance,
                                 rdAnalUVParameter,
                                 reLocation,
                                 pUVGuess);
      }
  }

  // Check on-surface.
  if ( eStat == SM_SUCCESS )
    {
      SmPoint3d sPt;
      SER( this->EvaluatePoint( rdAnalUVParameter, sPt ) );
      if ( sPt.DistanceBetween( crPointOnSurf ) > dDistanceTolerance )
        { eStat = SM_ERR; }
    }

  return eStat;

} // end SmOffsetSurface::STEPInversion

/*******************************************************************//**
PURPOSE: Swap the UV parameterization of the surface.

NOTES: 
***********************************************************************/
SmStatus SmOffsetSurface::SwapUV()
{
  if (m_pExtendedSurface) 
    {
      SER(m_pExtendedSurface->SwapUV());
      m_dOffsetDistance = -m_dOffsetDistance;
    }
  if (m_bOwnsSurface) 
    {
      SER(m_pSurface->SwapUV());
      m_dOffsetDistance = -m_dOffsetDistance;
    }
  else 
    {
      SER(SM_ERR);  // Can not swap UV if I do not own the surface
    }
  return SM_SUCCESS;

} // end SmOffsetSurface::SwapUV

/*******************************************************************//**
PURPOSE: Scale and transform an offset surface.
   If scale is passed in apply it first then the rotation and
   move defined by the placement.

NOTES: 
***********************************************************************/
SmStatus SmOffsetSurface::Transform
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

  if (cpOptScale) 
    {
      if (   SM_ARE_SAME(cpOptScale->x,cpOptScale->y)
          && SM_ARE_SAME(cpOptScale->x,cpOptScale->z) ) 
        {
          m_dOffsetDistance = cpOptScale->x * m_dOffsetDistance;
        }
      else 
        {
          SER_MSG(SM_ERR,
                  _T("OffsetSurface NonUniform scaling not supported, see SmOffsetExecutive::ReplaceImplicitOffsets to convert to NURBS")) ;
        }
    }

  if (m_pExtendedSurface) 
    {
      SER(m_pExtendedSurface->Transform(crRotateNMove,cpOptScale));
    }
  if (m_bOwnsSurface) 
    {
      SER(m_pSurface->Transform(crRotateNMove,cpOptScale));
    }
  else 
    {
      SER(SM_ERR);  // Can not transform if I do not own the surface
    }
    
  return SM_SUCCESS;

} // end SmOffsetSurface::Transform

/*******************************************************************//**
PURPOSE: Trim the offset surface to a domain when possible.

NOTES: 
  This function preserves the input surface geometry and its
  parameterization at the domain corners exactly, however the surface's 
  parameterization between domain corners may vary slightly but by
  amounts easily larger than reasonable tolerance sizes.  

  As such, existing Edgeuse->UVTrimCurves that reference the surface
  being trimmed should be deleted and rebuilt after this call.
***********************************************************************/
SmStatus SmOffsetSurface::TrimWithDomain
  (SmExtent2d & crTrimInterval)
{
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;

  // copy surface in case we want to compare before/after deformations
  SmSurface *pCopySurface ;
  this->Copy(*GetContext(), pCopySurface) ; 
  SmObjDelete sCopyClean(pCopySurface) ;
   
  // draw 
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  SmExtent2d sBaseDomain = m_pSurface->GetNaturalUVDomain();
  SmPoint3d  sPnt;
  SmBoolean  bGetsBigger = FALSE;

  if (!sBaseDomain.ContainsPoint2d(crTrimInterval.GetMin())) 
    {
      SER(EvaluatePoint(crTrimInterval.GetMin(),sPnt));
      bGetsBigger = TRUE;
    }

  if (!sBaseDomain.ContainsPoint2d(crTrimInterval.GetMax())) 
    {
      SER(EvaluatePoint(crTrimInterval.GetMax(),sPnt));
      bGetsBigger = TRUE;
    }

  // trim ExtendedSurface
  if (m_pExtendedSurface) 
    {
      SER(m_pExtendedSurface->TrimWithDomain(crTrimInterval));
    }

  // trim Surface when owned and not getting bigger
  if(    m_bOwnsSurface  
     && !bGetsBigger) 
    {
      SER(m_pSurface->TrimWithDomain(crTrimInterval)); //sBaseDomain));
    }

  // Can not trim if I do not own the surface
  if(!m_bOwnsSurface) 
    {
      SER(SM_ERR);  
    }

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      // copy surface in case we want to compare before/after deformations
      SmSolutionArray sSolutions ;
      pCopySurface->GlobalSurfaceSolve(this->GetNaturalUVDomain(), 
                                       *this, 
                                       this->GetNaturalUVDomain(), 
                                       SM_SO_MAXIMIZE, SM_EFF_ZERO, NULL, NULL, 
                                       SM_SR_ALL, sSolutions) ;

      SM_ASSERT_VALID(this) ;
      sSolutions.Dump() ;

      SmFace *pFace = (SmFace *)this->GetFace() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep() : NULL ;
      SmExtent2d sDom( this->GetNaturalUVDomain() );
      SmZoneTol3d sZoneTol3d = pFace ? (double)pFace->GetTolerance() : pBrep ? (double)pBrep->GetTolerance() : 0.00001 ;

      SmSrfSrfGapFunction sSrfSrfGap( (SmXSectTol3d)sZoneTol3d,
                                      this, sDom,
                                      pCopySurface, 40, 40) ;
      sSrfSrfGap.Dump() ;


      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; sSrfSrfGap.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
        
  return SM_SUCCESS;

} // end SmOffsetSurface::TrimWithDomain

/*******************************************************************//**
PURPOSE: Create an offset surface in a generic way by using this
            object's base offset surface.

NOTES:  When the compound offset happens to be zero, a copy
        of the base surface is put in the output,
        Otherwise a new SmOffsetSurface object is placed in the
        output.

        Out Surf->Domain(s) may be trimmed but not scaled
***********************************************************************/
SmStatus SmOffsetSurface::CreateOffsetSurface
(
  const SmContext      & crContext,             // in : context for new obj construction
  double                 dSignedOffsetDistance, // in : offset dist, (neg val = Offset dir opposite surface normal)
  SmApproxTol3d          dThisApproxTol3d,      // NotUsed: in : Max Dist between ApproxOffsetSurface and ideal offset shape
  SmSurface* &           rOffsetSurface         // out: Offset Surf Approx, may be more than 1 when offsets have self-intersections
) const
{ 
  SM_REF1(dThisApproxTol3d) ; 
  // Make a temporary copy of the base surface
  SmSurface * pSurfCopy;
  SER(m_pSurface->Copy(crContext, pSurfCopy)); NER(pSurfCopy);
  SmObjDelete sClean1(pSurfCopy);

  // compute the compounded offset distance
  double dCompoundDist = dSignedOffsetDistance + m_dOffsetDistance;

  // when comound offset is zero
  if (smos_Fabs(dCompoundDist) < SM_EFF_ZERO) 
    {
      // output the base surface copy
      sClean1.Clear();
      rOffsetSurface = pSurfCopy;
      return SM_SUCCESS;
    }

  // else output a new OffsetSurface
  SmOffsetSurface *pOff = new(crContext) SmOffsetSurface(dCompoundDist,
                                                         *pSurfCopy,
                                                         TRUE);
  NER(pOff);
  sClean1.Clear();
  rOffsetSurface = pOff;

  // all done
  return SM_SUCCESS;

} // end SmOffsetSurface::CreateOffsetSurface 

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmOffsetSurface.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmOffsetSurface::GetMemoryUsed   // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,       // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)               // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + surface memory
  ULONG lThisAllocated = 0 ;
  ULONG lUsed        =   sizeof(*this) + m_pSurface->GetMemoryUsed(lThisAllocated, eMarkType) ;
  rlMemoryAllocated  =   sizeof(*this) + lThisAllocated ;

  // extended surface
  lThisAllocated     = 0 ;
  lUsed             += m_pExtendedSurface ? m_pExtendedSurface->GetMemoryUsed(lThisAllocated, eMarkType) : 0 ;
  rlMemoryAllocated += lThisAllocated ;

  // + attribute memory
  lUsed += rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
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

} // end SmOffsetSurface::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Add OffsetSurface graphics to new or open drawList added to 
            global drawList array.

NOTES:
***********************************************************************/
SmDisplayList * SmOffsetSurface::Draw
  (SmBoolean       bAddToUIPickList, // in : TRUE = Add this surface to UI pick interface for debugging
   SmGfxArraySet * pOptGfxSet)       // in : When given output GfxVertexArrays not GL calls.
 const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE

  // Draw the Surface
  pRtn = SmSurface::Draw(bAddToUIPickList, pOptGfxSet) ;

  // Draw the BaseSurface in BaseSurfaceColor
  SmVector3d sColor = smgfx_SetColor(smgfx_GetRuleColor(this, SM_CR_BASESURFACE), pOptGfxSet) ;
  m_pSurface->SmSurface::Draw(bAddToUIPickList, pOptGfxSet) ;
  smgfx_SetColor(sColor, pOptGfxSet);

  // Draw the BaseSurface in BaseSurfaceColor
  if(m_pExtendedSurface)
    {
      SmVector3d sColor2 = smgfx_SetColor(smgfx_GetRuleColor(this, SM_CR_EXTENDEDSURFACE), pOptGfxSet) ;
      m_pExtendedSurface->SmSurface::Draw(bAddToUIPickList, pOptGfxSet);
      smgfx_SetColor(sColor2, pOptGfxSet);
    } 

#else
  SM_REF2(bAddToUIPickList, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmOffsetSurface::Draw

/*******************************************************************//**
PURPOSE: Add OffsetSurface graphics to new DisplayList added to
            global displayList array using

              sDisp.m_bDrawCrossHatch   = TRUE ;        
              sDisp.m_lCrossHatchUCount = lNumBetweenU ;
              sDisp.m_lCrossHatchVCount = lNumBetweenV ;
              sDisp.m_bVaryCrossHatchColor = Draw U Lines in ObjectColor - Draw V lines in m_VaryCrossHatchColor  

NOTES: 
   special case: when sDisp.m_lCrossHatchUCount == 999
                  and sDisp.m_lCrossHatchVCount == 999,
                 then draw surface as 4 corners connected by line segments.

                 This is used to draw the surface cache bezier surface fragments.
***********************************************************************/
SmDisplayList * SmOffsetSurface::DrawUV      
  (ULONG     lNumBetweenU,           // in : number of U IsoParameter lines between knots
   ULONG     lNumBetweenV,           // in : number of V IsoParameter lines between knots
   SmBoolean bVaryCrossHatchColor,   // in : TRUE = Draw U Lines in ObjectColor 
                                     //             Draw V lines in m_VaryCrossHatchColor  
                                     //      FALSE= Draw both U and V Lines in ObjectColor
   const SmExtent2d *pOptUVDomain,   // in : UVDomain to crossHatch
                                     //      NULL=NaturalUVDomain
   SmBoolean       bAddToUIPickList, // in : TRUE=Add to UI pick list, FALSE=don't
                                     //      default:[FALSE]
   SmGfxArraySet * pOptGfxSet)       // in : When given output GfxVertexArrays not GL calls.
  const
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE

  // Draw the Surface
  pRtn = SmSurface::DrawUV(lNumBetweenU, lNumBetweenV, bVaryCrossHatchColor, pOptUVDomain, bAddToUIPickList, pOptGfxSet);

  // Draw the BaseSurface in BaseSurfaceColor
  if(pOptUVDomain == NULL)
    {
      SmVector3d sColor = smgfx_SetColor(smgfx_GetRuleColor(this, SM_CR_BASESURFACE), pOptGfxSet);
      m_pSurface->SmSurface::DrawUV(lNumBetweenU, lNumBetweenV, bVaryCrossHatchColor, pOptUVDomain, bAddToUIPickList, pOptGfxSet);
      smgfx_SetColor(sColor, pOptGfxSet);

      // Draw the ExtendedSurface in ExtendedSurfaceColor
      if(m_pExtendedSurface)
        {
          SmVector3d sColor2 = smgfx_SetColor(smgfx_GetRuleColor(this, SM_CR_EXTENDEDSURFACE), pOptGfxSet) ;
          m_pExtendedSurface->SmSurface::DrawUV(lNumBetweenU, lNumBetweenV, bVaryCrossHatchColor, pOptUVDomain, bAddToUIPickList,  pOptGfxSet) ;
          smgfx_SetColor(sColor2, pOptGfxSet);
        }
    } 

#else
  SM_REF6(lNumBetweenU, lNumBetweenV, bVaryCrossHatchColor, pOptUVDomain, bAddToUIPickList, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmOffsetSurface::DrawUV

/*******************************************************************//**
PURPOSE: Write SmOffsetSurface to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmOffsetSurface::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();
      
  // locals
  SmBoolean bHasExtendedSurface = m_pExtendedSurface != NULL ; 

  if (eType == SM_ASCII) 
    {
      rFileOut << m_dOffsetDistance    << " SmOffsetSurface Offset Distance \n";
      rFileOut << m_bAllowExtension    << " SmOffsetSurface Allow Extension \n";
      rFileOut << m_bIsPeriodicOnU     << " SmOffsetSurface PeriodicOnU \n";
      rFileOut << m_bIsPeriodicOnV     << " SmOffsetSurface PeriodicOnV \n";
      rFileOut << bHasExtendedSurface  << " SmOffsetSurface HasExtendedSurface \n";
    }
  else 
    {
      SER(rDB.WriteDouble(m_dOffsetDistance));
      SER(rDB.WriteBoolean(m_bAllowExtension));
      SER(rDB.WriteBoolean(m_bIsPeriodicOnU));
      SER(rDB.WriteBoolean(m_bIsPeriodicOnV));
      SER(rDB.WriteBoolean(bHasExtendedSurface));
    }

  // base surface
  if (eType == SM_ASCII) { rFileOut << " OffsetSurface->BaseSurface \n"; }
  SER(rDB.WriteType(m_pSurface->GetType())) ; 
  SER(m_pSurface->WriteToDB(rDB, lDBVersionNumber)) ;

  // extended surface
  if(bHasExtendedSurface) { if (eType == SM_ASCII)  { rFileOut << " OffsetSurface->ExtendedSurface \n"; }
                            SER(rDB.WriteType(m_pExtendedSurface->GetType())) ; 
                            SER(m_pExtendedSurface->WriteToDB(rDB, lDBVersionNumber)) ;
                          }
  else                    { if (eType == SM_ASCII)  { rFileOut << "Has No OffsetSurface->ExtendedSurface \n"; }
                          }

  // all done
  return SM_SUCCESS;

} // end SmOffsetSurface::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmOffsetSurface from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmOffsetSurface::ReadFromDB
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
         || rpNewSurface->IsKindOf(SmOffsetSurface_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmOffsetSurface *pOffsetSurface =   (rpNewSurface == NULL)
                                    ? new (crContext) SmOffsetSurface()
                                    : (SmOffsetSurface *)rpNewSurface ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  double    dOffsetDistance = 0.0;  
  SmBoolean bAllowExtension;  
  SmBoolean bIsPeriodicOnU;   
  SmBoolean bIsPeriodicOnV;   
  SmBoolean bHasExtendedSurface ;

  if (eType == SM_ASCII) 
    {
      rFileIn >> dOffsetDistance ;      rDB.GoToNextLine() ;
      rFileIn >> bAllowExtension ;      rDB.GoToNextLine() ;
      rFileIn >> bIsPeriodicOnU ;       rDB.GoToNextLine() ;
      rFileIn >> bIsPeriodicOnV ;       rDB.GoToNextLine() ;
      rFileIn >> bHasExtendedSurface ;  rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble(dOffsetDistance)) ;
      SER(rDB.ReadBoolean(bAllowExtension)) ;
      SER(rDB.ReadBoolean(bIsPeriodicOnU)) ;
      SER(rDB.ReadBoolean(bIsPeriodicOnV)) ;
      SER(rDB.ReadBoolean(bHasExtendedSurface)) ;
    }

  // surface locals
  SmSurface *pBaseSurface=NULL, *pExtendedSurface=NULL ;
  SM_TYPE    lBaseType, lExtendedType ;  

  // base surface
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lBaseType)) ; 
  SER(SmSurface::ReadFromDB(lBaseType, rDB, crContext, pBaseSurface, lDBVersionNumber)) ; 

  // extended surface
  if (eType == SM_ASCII)  { rDB.GoToNextLine() ; }
  if(bHasExtendedSurface) { SER(rDB.ReadType(lExtendedType)) ;
                            SER(SmSurface::ReadFromDB(lExtendedType, rDB, crContext, pExtendedSurface, lDBVersionNumber)) ; 
                          }

  // load the object
  pOffsetSurface->m_dOffsetDistance  = dOffsetDistance ;
  pOffsetSurface->m_bAllowExtension  = bAllowExtension ; 
  pOffsetSurface->m_bOwnsSurface     = TRUE ;
  pOffsetSurface->m_pSurface         = pBaseSurface ;
  pOffsetSurface->m_pSurface->SetOwner(pOffsetSurface) ;
  pOffsetSurface->m_pExtendedSurface = pExtendedSurface ; 
  pOffsetSurface->m_bIsPeriodicOnU   = bIsPeriodicOnU ; 
  pOffsetSurface->m_bIsPeriodicOnV   = bIsPeriodicOnV ; 
  pOffsetSurface->m_lSingularities   = pBaseSurface->GetSingularities() ; // regen here - this data is not persistent

  // all done
  rpNewSurface = pOffsetSurface ;
  return SM_SUCCESS;

} // end SmOffsetSurface::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmOffsetSurface::IsKindOf( SM_TYPE t ) const
{
  return ((SmOffsetSurface_TYPE == t) ? TRUE : SmSurface::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
void SmOffsetSurface::Dump
  (void) 
const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmOffsetSurface::Dump()")) ;

  // report Cache data
  SmSurface::Dump(FALSE) ;

  smos_sprintf(sBuff,       
             _T("\nSmOffsetSurface = 0x%p, BaseSurface = 0x%p, OffsetDistance - %16.16lf\n"),
             this, 
             m_pSurface, 
             m_dOffsetDistance);
  smos_sprintf(sBuffForFile,
             _T("\nSmOffsetSurface = %s, BaseSurface = %s, OffsetDistance - %16.16lf\n"),
             _T("notNULL"),
             m_pSurface ? _T("notNULL") : _T("NULL"), 
             m_dOffsetDistance);
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_WriteBuffer(_T("\n  Begin OffsetSurface BaseSurface Dump")) ;
  m_pSurface->Dump();
  smos_WriteBuffer(_T("  End OffsetSurface BaseSurface Dump\n")) ;

  smos_WriteBuffer(_T(" End SmOffsetSurface::Dump()\n")) ;

} // end SmOffsetSurface::Dump

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertOffsetSurface_list[] =
{
  /*  0 */ {SM_AT_POINTER,          _T("Context"),             _T("m_pSurface shares the same context") },
  /*  1 */ {SM_AT_POINTER,          _T("Bad Surface Owner"),   _T("Base Surface owner must be this SmOffsetSurface object") },
  /*  2 */ {SM_AT_DEGENERATE,       _T("Intersecting Surface"),_T("Offset Surface has self-intersecting interior") }
} ;

/*******************************************************************//**
PURPOSE: Make sure GenCurve and Surface parameterizations are
            compatible.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmOffsetSurface::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ; 
  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
          ? SmSurface::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
          : TRUE ) ;

  // all contained pointers to crContext should be the same
  /*  0 */ // m_pSurface shares the same context
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pSurface == NULL || m_cpContext == m_pSurface->GetContext()), _T("") ) ;

  /*  1 */ // Bad m_pSurface Owner
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, m_pSurface->GetOwner() == this, _T("")) ;

  /*  2 */ // Self-intersecting surface interior
  SmExtent2d sUVDomain = GetNaturalUVDomain() ;
  SmBoolean sbUseSurfaceEdges[] = { FALSE, FALSE } ;
  SmTArray<SmCurve*> pOpt3DCurves(*m_cpContext), pOptSurface1UVCurves(*m_cpContext);
  SmTArray<SmTsectCurveType> pOptCurveTypes(*m_cpContext);
  GlobalSurfaceIntersect(*m_cpContext, sUVDomain, *this, sUVDomain, sbUseSurfaceEdges, NULL, NULL, \
      &pOpt3DCurves, &pOptSurface1UVCurves, NULL, &pOptCurveTypes ,NULL ) ;
  // If the surface does not intersect, then pOptCurveTypes is an empty array
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, pOptCurveTypes.GetSize() < 1, _T("") ) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
    }
#endif // SM_DEBUG_CODE

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmOffsetSurface::AssertValid
