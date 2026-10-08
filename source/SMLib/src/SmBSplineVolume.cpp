// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmBSplineVolume.cpp
* PURPOSE: Implementation of SmBSplineVolume methods.
* Oct-2006 - gwc - author
**********************************************************************/

#include "StdAfx.h"

#include <SmBSplineVolume.h>
#include <nurbs.h>
#include <SmNurbsVol.h>
#include <SmNurbsSrf.h>
#include <SmAssertArray.h>
#include <SmDatabaseIO.h>
#include <SmTransform.h>
#include <SmContext.h>

/*******************************************************************//**
  Local File Function Declarations
***********************************************************************/

/*******************************************************************//**
PURPOSE: get or create m_pNurb ponter data 

NOTES: file local function
***********************************************************************/
gw_VOLUME * smGet_gw_VOLUME
  (const SmBSplineVolume *pBSV)
{
  return ((SmBSplineVolume *)pBSV)->GetOrCreateGwNurbPointer() ;

} // end smGet_gw_VOLUME

/*******************************************************************//*
PURPOSE: Equivalence check for gw_VOLUME

NOTES:
***********************************************************************/
SmBoolean AreEqual_gw_VOLUME
 (gw_VOLUME &rVol1,    // in : Volume1 of Volume1 == Volume2
  gw_VOLUME &rVol2,    // in : Volume2 of Volume1 == Volume2
  double    dTol3d,    // in : max distance between coincident control points
  double    dTol1d)    // in : max distance between coincident knots
  
{
#ifdef USE_VOLUMES
    SmBoolean bRtn = N_VolumesAreEqual(&rVol1, &rVol2, dTol3d, dTol1d);
    return(bRtn);
#else
    SM_REF4(rVol1, rVol2, dTol3d, dTol1d);
    return(false);
#endif

} // end AreEqual_gw_VOLUME

/*******************************************************************//**
PURPOSE: Apply transform to a BSplineVolume's control points

NOTES: calls Notify SM_NO_PRE_EDIT before the control point change
       calls N_VolumeTransform to move the control points
       calls Notify SM_NO_POST_EDIT after the control point change
***********************************************************************/
SmStatus sm_LoadNLibTransform
 (SmTransform     & rT,             // in : Transform to apply to BSplineVolume control points                                                                     
  SmBSplineVolume & rBSplineVolume) // in : the volume to be transformed
{
#ifdef USE_VOLUMES
    // start NLib stack
    NL_STACKS      SC;
    SmStackHandler sStackKp(&SC);

    // build Nlib transformation matrix
    NL_RMATRIX        rma;
    N_InitRealMatrix(&rma);
    N_SetRealMatrix(&rma, 3, 3, full, 3, &SC);
    gw_REAL** RM = rma.RM;

    // load NLib matrix
    rT.LoadNLibTransform(RM);

    // warn the public
    rBSplineVolume.Notify(SM_NO_PRE_EDIT, &rBSplineVolume, SM_NO_GET_OWNER(&rBSplineVolume), NULL);

    // apply transformation to BSplineVolume 
    N_VolumeTransform(smGet_gw_VOLUME(&rBSplineVolume), &rma);

    // inform the public
    rBSplineVolume.Notify(SM_NO_POST_EDIT, &rBSplineVolume, SM_NO_GET_OWNER(&rBSplineVolume), NULL);

    // all done
    return SM_SUCCESS;
#else
    SM_REF2(rT, rBSplineVolume);
    return SM_ERR;
#endif // USE_VOLUMES

} // end sm_LoadNLibTransform

/*******************************************************************//**
PURPOSE: Empty constructor required for persistence methods

NOTES: 
***********************************************************************/
SmBSplineVolume::SmBSplineVolume
 (const SmContext * cpContext)      // in : context for this new object, may be NULL for temp objects
 :m_pNurb(NULL),
   m_bNurbIsBorrowed(FALSE),
   m_bOutOfBoundsEnabled(FALSE) 
{ 
  if(cpContext) 
    {
       m_cpContext = cpContext ;
    }

} // end SmBSplineVolume::SmBSplineVolume empty constructor

/*******************************************************************//**
PURPOSE: NLib Descriptors Constructor 

NOTES: 
***********************************************************************/
SmBSplineVolume::SmBSplineVolume
  (const SmContext * cpContext, // in : context for this new object, may be NULL for temp objects
   ULONG             n,         // in : high index of Pw (U)
   ULONG             m,         // in : high index of Pw (V)
   ULONG             o,         // in : high index of Pw (W)
   short             p,         // in : degree in u
   short             q,         // in : degree in v
   short             r,         // in : degree in w
   ULONG             ir,        // in : high index of U
   ULONG             is,        // in : high index of V
   ULONG             it,        // in : high index of W
   gw_CPOINT     *** Pw,        // in : Control polygon, P[u,v,w]
                                //      ordered:[w index varies fastest, then v, then u]
                                //      sized:  [  lNumUCPts = high index of U - lUDegree 
                                //               * lNumVCPts = high index of V - lVDegree 
                                //               * lNumWCPts = high index of W - lWDegree] 
   double            *U,        // in : knots in u, sized:[high index of U+1]
   double            *V,        // in : knots in v, sized:[high index of V+1]
   double            *W)        // in : knots in w, sized:[high index of W+1]
:  m_pNurb(NULL),
   m_bNurbIsBorrowed(FALSE),
   m_bOutOfBoundsEnabled(FALSE) 
{
  if(cpContext) { SetContext(cpContext) ; }

  ULONG ii, jj ;
  
  // allocate single block Volume with internal pointers and size params set
  gw_VOLUME *pNewVol = sm_AllocateNurbVolume(n, m, o, p, q, r, ir, is, it);

  // copy knot values from target volume to new volume
  SE(smos_MemCpy(pNewVol->knu->U, U, sizeof(gw_REAL) * (ir + 1), sizeof(gw_REAL) * (ir + 1)));
  SE(smos_MemCpy(pNewVol->knv->U, V, sizeof(gw_REAL) * (is + 1), sizeof(gw_REAL) * (is + 1)));
  SE(smos_MemCpy(pNewVol->knw->U, W, sizeof(gw_REAL) * (it + 1), sizeof(gw_REAL) * (it + 1)));

  // copy control point values from target volume to new volume one row at a time
  for(ii=0; ii<=n; ii++) 
  {
    for(jj=0; jj<=m; jj++)
    {
      SE(smos_MemCpy(pNewVol->mesh->Pw[ii][jj], Pw[ii][jj],  sizeof(gw_CPOINT) * (o + 1), sizeof(gw_CPOINT) * (o + 1) ));
    }
  }

  m_pNurb = pNewVol;

  if(m_pNurb)
    {
      // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
      //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
      gw_KNOTVECTOR *pKnu = m_pNurb->knu;
      gw_KNOTVECTOR *pKnv = m_pNurb->knv;
      gw_KNOTVECTOR *pKnw = m_pNurb->knw;
      sm_FixupKnotVector(pKnu,SM_EFF_ZERO_PARAM);
      sm_FixupKnotVector(pKnv,SM_EFF_ZERO_PARAM);        
      sm_FixupKnotVector(pKnw,SM_EFF_ZERO_PARAM);
    }        

  // not needed - called in SmVolume constructor
  //      // report construction at SmObject::Notify level - skip other levels
  //      SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL);

  // GWC:old notify
  //      Notify(SM_NO_CONSTRUCTION, this, NULL);

} // end SmBSplineVolume::SmBSplineVolume constructor from NLib data

/*******************************************************************//**
PURPOSE: Constructor when given an already structured gw_VOLUME m_pNurb object

NOTES: 
***********************************************************************/
SmBSplineVolume::SmBSplineVolume
  (gw_VOLUME       * pAlreadyAllocatedNurb,  // in : Pointer to allocated NURB
   SmBoolean         bNurbIsBorrowed,        // in : TRUE = don't delete pNurb when destructed
                                             //      good for temporary Volumes built for persistent gw_VOLUME objects
   const SmContext * cpContext)              // in : Use this context when given
 : m_pNurb(pAlreadyAllocatedNurb),
   m_bNurbIsBorrowed(bNurbIsBorrowed),
   m_bOutOfBoundsEnabled(FALSE) 
{ 
  SM_ASSERT(pAlreadyAllocatedNurb != NULL);

  // not needed - called in base class constructor
  // if(!m_bNurbIsBorrowed) Notify(SM_NO_CONSTRUCTION, this, NULL);

  if(cpContext) 
    { m_cpContext = cpContext ; }

  if(!m_bNurbIsBorrowed) 
    {
      if(m_pNurb)
        {
          // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
          //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
          gw_KNOTVECTOR *pKnu = m_pNurb->knu;
          gw_KNOTVECTOR *pKnv = m_pNurb->knv;
          gw_KNOTVECTOR *pKnw = m_pNurb->knw;
          sm_FixupKnotVector(pKnu,SM_EFF_ZERO_PARAM);
          sm_FixupKnotVector(pKnv,SM_EFF_ZERO_PARAM);        
          sm_FixupKnotVector(pKnw,SM_EFF_ZERO_PARAM);
        }        
    }
  
} // end SmBSplineVolume::SmBSplineVolume constructor

/*******************************************************************//**
PURPOSE:  Construct a B-Spline Volume given a pointer to an NLib
   NURB Volume and the corresponding dimension.  The NLib Volume will
   not be consumed but will be copied.

NOTES: The resulting Volume will be projected to the XY plane or 
   the X line if the dimension of the input Volume is not the same as the 
   dimension asked for.
***********************************************************************/
SmBSplineVolume::SmBSplineVolume
  (const SmContext * cpContext,   // in : context for this new object, may be NULL for temp objects
   const gw_VOLUME* cpGwNurb)
  : m_pNurb(NULL),
    m_bNurbIsBorrowed(FALSE),
    m_bOutOfBoundsEnabled(FALSE) 
{ 
  if(cpContext) 
    { m_cpContext = cpContext ; }

  // check input and state
  SM_ASSERT(cpGwNurb != NULL) ;
  SM_ASSERT(m_pNurb  == NULL) ;

  // This routine allocates a single piece of memory to
  // contain the nurb Volume.  The following order is used
  // to map the memory to the Volume structure:
  //    gw_VOLUME
  //    NL_CMESH
  //    KNOTVECTORU
  //    KNOTVECTORV
  //    <array of double for knots U>
  //    <array of double for knots V>
  //    <array of double for knots W>
  //    <array of pointers to gw_CPOINT**>
  //    <array of pointers to gw_CPOINT*>
  //    <array of double*4 for CPOINTS>
  const gw_VOLUME *pSrcVol = (gw_VOLUME*)cpGwNurb;
  m_pNurb = sm_AllocateAndCopyNurbVolume(pSrcVol);

  if(m_pNurb)
    {
      // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
      //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
      gw_KNOTVECTOR *pKnu = m_pNurb->knu;
      gw_KNOTVECTOR *pKnv = m_pNurb->knv;
      gw_KNOTVECTOR *pKnw = m_pNurb->knw;
      sm_FixupKnotVector(pKnu,SM_EFF_ZERO_PARAM);
      sm_FixupKnotVector(pKnv,SM_EFF_ZERO_PARAM);        
      sm_FixupKnotVector(pKnw,SM_EFF_ZERO_PARAM);
    }        

  // not needed - called in SmVolume constructor
  //      // report construction at SmObject::Notify level - skip other levels
  //      SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL);
  // GWC:old notify
  //      Notify(SM_NO_CONSTRUCTION, this, NULL);

} // end SmBSplineVolume::SmBSplineVolume constructor

/*******************************************************************//**
PURPOSE: Copy constructor for a B-Spline Volume.

NOTES:
***********************************************************************/
SmBSplineVolume::SmBSplineVolume
 (const SmBSplineVolume & crSourceVolume,   // in : SourceVolume to copy
  SmBoolean               bSimpleMapOnly)   // in : TRUE = Copy this Volume omitting any compounding volumes
                                            //      FALSE= Copy this Volumes with any compounding volumes 
: SmVolume(crSourceVolume, bSimpleMapOnly),
  m_pNurb(NULL),
  m_bNurbIsBorrowed(FALSE), 
  m_bOutOfBoundsEnabled(crSourceVolume.m_bOutOfBoundsEnabled)
{
  // free any existing NurbVolume structure
  if (m_pNurb && !m_bNurbIsBorrowed) 
    {
      smos_Free(m_pNurb);
      m_pNurb = NULL;
    }

  // fetch the source NurbVolume 
  const gw_VOLUME *pSrcVol = smGet_gw_VOLUME(&crSourceVolume);
  if (!pSrcVol) 
    { 
      // for the construction of m_pNurb
      SM_ASSERT_VALID(&crSourceVolume) ;
      pSrcVol = smGet_gw_VOLUME(&crSourceVolume);
    }

  // error - can't fetch m_pNurb
  if (!pSrcVol) 
    { SE(SM_ERR); }

  // copy and store m_pNurb
  SM_ASSERT(m_pNurb == NULL) ;
  m_pNurb = sm_AllocateAndCopyNurbVolume(pSrcVol);

  if(m_pNurb)
    {
      // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
      //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
      gw_KNOTVECTOR *pKnu = m_pNurb->knu;
      gw_KNOTVECTOR *pKnv = m_pNurb->knv;
      gw_KNOTVECTOR *pKnw = m_pNurb->knw;
      sm_FixupKnotVector(pKnu,SM_EFF_ZERO_PARAM);
      sm_FixupKnotVector(pKnv,SM_EFF_ZERO_PARAM);        
      sm_FixupKnotVector(pKnw,SM_EFF_ZERO_PARAM);
    }        

  // not needed - called in SmVolume constructor
  //      // report construction at SmObject::Notify level - skip other levels
  //      SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL);

  // GWC:old notify
  //      // inform the attributes
  //      Notify(SM_NO_CONSTRUCTION, this, NULL);

} // end SmBSplineVolume::SmBSplineVolume constructor

/*******************************************************************//**
PURPOSE: Default destructor for B-Spline Volumes.

NOTES: 
***********************************************************************/
SmBSplineVolume::~SmBSplineVolume()
{
  if (m_pNurb && !m_bNurbIsBorrowed) 
    {
      smos_Free(m_pNurb);
      m_pNurb = NULL ;
    }

  // GWC:old notify
  //      if (!m_bNurbIsBorrowed) Notify(SM_NO_DESTRUCTION, this, NULL);

} // end SmBSplineVolume::~SmBSplineVolume destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmBSplineVolume

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmBSplineVolume::operator==
  (const SmVolume& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmVolume::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmBSplineVolume &rOther = (SmBSplineVolume &)crOther ;

      // check equivalence of these objects
      bRtn =    ( m_pNurb == rOther.m_pNurb)
             || ( m_pNurb == NULL && rOther.m_pNurb == NULL)
             || (   m_pNurb != NULL && rOther.m_pNurb != NULL
                 && AreEqual_gw_VOLUME(*m_pNurb, *rOther.GetGwNurbPointer(), SM_EFF_ZERO, SM_EFF_ZERO )) ;
    }

  // all done
  return bRtn ;

} // end SmBSplineVolume::operator==

/*******************************************************************//**
PURPOSE: assignment operator

NOTES:
***********************************************************************/
SmBSplineVolume & SmBSplineVolume::operator=
  (const SmBSplineVolume &crBSplineVolume)       // in : object to copy
{ 
  if(this == &crBSplineVolume) return(*this) ;
  
  // assign base values
  SmVolume::operator=(crBSplineVolume) ;

  // free any nested memory
  if (m_pNurb && !m_bNurbIsBorrowed) 
    {
      smos_Free(m_pNurb);
      m_pNurb = NULL ;
    }

  // make member assignments
  const gw_VOLUME *pSrcVol = crBSplineVolume.GetGwNurbPointer() ;
  m_pNurb               = sm_AllocateAndCopyNurbVolume(pSrcVol);
  m_bNurbIsBorrowed     = FALSE ;
  m_bOutOfBoundsEnabled = crBSplineVolume.m_bOutOfBoundsEnabled ;

  // all done
  return(*this) ;

} // end SmBSplineVolume::operator=

/*******************************************************************//**
PURPOSE: Copy a SmBSplineVolume.

NOTES: 
***********************************************************************/
SmStatus SmBSplineVolume::Copy
 (const SmContext & crContext,      // in : context for new object construction
  SmVolume       *& rpNewVolume,    // out: The copied Volume
  SmBoolean         bSimpleMapOnly) // in : TRUE = Copy this Volume omitting any compounding volumes
 const                              //      FALSE= Copy this Volumes with any compounding volumes
                                    //      default:[FALSE]
{
    rpNewVolume = new (crContext) SmBSplineVolume(*this, bSimpleMapOnly) ;
    NER(rpNewVolume);
    return SM_SUCCESS;

} // end SmBSplineVolume::Copy

// GWC:old notify
//      /*******************************************************************//**
//      PURPOSE: Receive notification of things happening to the object and
//          take appropriate actions.  Here we will fix up the knot vector during
//          construction.  
//      
//      NOTES: 
//      ***********************************************************************/
//      void SmBSplineVolume::Notify
//        (SmNotifyOperation eNotifyOperation,
//         void *pData1,
//         void *pData2,
//         SmNotifyOperation eAuxOperation )
//      {
//        switch (eNotifyOperation) 
//          {
//            case SM_NO_CONSTRUCTION:
//              {
//                gw_VOLUME *pVol = smGet_gw_VOLUME(this);
//                if (!pVol) return;
//      
//                // Edit knot values: close knots (knotSpacing<ScaledZero) become same and 
//                //                   near knots (ScaledZero<KnotSpacing<sTol2d) are seperated by more than sTol2d.
//                gw_KNOTVECTOR *pKnu = pVol->knu;
//                gw_KNOTVECTOR *pKnv = pVol->knv;
//                gw_KNOTVECTOR *pKnw = pVol->knw;
//                sm_FixupKnotVector(pKnu,SM_EFF_ZERO_PARAM);
//                sm_FixupKnotVector(pKnv,SM_EFF_ZERO_PARAM);        
//                sm_FixupKnotVector(pKnw,SM_EFF_ZERO_PARAM);        
//              }
//              break;
//            case SM_NO_COPY:
//            case SM_NO_SPLIT:
//            case SM_NO_MERGE: 
//            case SM_NO_REG_PROPAGATION:
//            case SM_NO_PRE_EDIT:
//            case SM_NO_POST_EDIT:
//            case SM_NO_DESTRUCTION:
//            case SM_NO_UNKNOWN:
//              break;
//          }
//      
//        // not needed - called in SmVolume constructor
//        //      // Propagate notification up hierarchy
//        //      SmVolume::Notify(eNotifyOperation,pData1,pData2,eAuxOperation);
//      
//      } // end SmBSplineVolume::Notify

/*******************************************************************//**
PURPOSE: Create a SmBSplineVolume from component data.  

NOTES: Note that the control points are always given
    in Euclidian space even for rational curves.  If you have homogeneous 
    coordinates you need to perform homogeneous division prior to calling
    this method (x = x/w, y=y/w, z=z/w w=w).

    Input arrays can describe a spline using unique or complete knot arrays as
    unique knot  : mult[4 1 1 4]                      // degree 3, 6 control point spline
    example      : knot[0 .333 .666 1]                // unique knots
                  
    Complete knot: mult[1 1 1 1   1    1  1 1 1 1]    // degree 3, 6 control point spline
    example      : knot[0 0 0 0 .333 .666 1 1 1 1]    // complete knots

    lNumUCPts = sTotalUKnots.GetSize() - lUDegree - 1;  n = lNumUCPts - 1
    lNumVCPts = sTotalVKnots.GetSize() - lVDegree - 1;  m = lNumVCPts - 1
    lNumWCPts = sTotalWKnots.GetSize() - lWDegree - 1;  o = lNumWCPts - 1

    crControlPointsList: sized:[lNumUCPts * lNumVCPts * lNumWCPts]
                         ordered:[P000, P001, ... P00o,           
                                  P010, P011, ... P01o,           
                                  . . .                           
                                  P0m0, P0m1, ... P0mo,           
                                  P100, P101, ... P10o,           
                                  . . .                           
                                  Pnm0, Pnm1, ... Pnmo]           

    The given degrees must be no bigger than the smallest of
    GW_MAX_DEGREE and NL_DMAX which is current the rang 1 to 17.
***********************************************************************/
SmStatus SmBSplineVolume::CreateCanonical
  (const SmContext & crContext,                      // in : context for new object construction
   ULONG lUDegree,                                   // in : U dir Degree
   ULONG lVDegree,                                   // in : V dir Degree
   ULONG lWDegree,                                   // in : W dir Degree                
   const SmTArray<SmPoint3d> & crControlPointsList,  // in : coords : Euclidian 
                                                     //      sized  : [  lNumUCPts = UKnotTotalCnt - lUDegree - 1 
                                                     //                * lNumVCPts = VKnotTotalCnt - lVDegree - 1 
                                                     //                * lNumWCPts = WKnotTotalCnt - lWDegree - 1]
                                                     //      ordered: for Puvw - w varies fastest, then v, then u as:
                                                     //              [P000, P001, ... P00o, with: n = lNumUCPts - 1
                                                     //               P010, P011, ... P01o,       m = lNumVCPts - 1
                                                     //               . . .                       o = lNumWCPts - 1
                                                     //               P0m0, P0m1, ... P0mo,
                                                     //               P100, P101, ... P10o,
                                                     //               . . .                
                                                     //               Pnm0, Pnm1, ... Pnmo]
   const SmTArray<ULONG> & crUMultiplicities,        // in : each crUKnot[i] multiplicity, sized:[crUKnots.GetSize()]
   const SmTArray<ULONG> & crVMultiplicities,        // in : each crVKnot[j] multiplicity, sized:[crVKnots.GetSize()]
   const SmTArray<ULONG> & crWMultiplicities,        // in : each crWKnot[k] multiplicity, sized:[crWKnots.GetSize()]
   const SmTArray<double> & crUKnots,                // in : unique or complete U Knot vals, as coordinated with Mults arrays  
   const SmTArray<double> & crVKnots,                // in : unique or complete V Knot vals, as coordinated with Mults arrays  
   const SmTArray<double> & crWKnots,                // in : unique or complete W Knot vals, as coordinated with Mults arrays  
   const SmTArray<double> * cpOptWeights,            // in : optional weight for each ControlPoint, NULL to ignore
   const SmExtent3d * cpOptUVWDomain,                // in : Will trim volume to smaller domain
   SmBSplineVolume *& rpNewBSplineVolume)            // out: New SmBSplineVolume, NULL on input 
{
#ifdef USE_VOLUMES
    // check input
    RANGE_ER(1, lUDegree, 32);
    RANGE_ER(1, lVDegree, 32);
    RANGE_ER(1, lWDegree, 32);
    SM_ASSERT_MSG(rpNewBSplineVolume == NULL,
        _T("SmBSplineVolume::CreateCanonical called with nonNULL rpNewBSplineVolume ptr value"));

    // If have weights make sure have right number of them
    if (cpOptWeights && cpOptWeights->GetSize() != 0)
    {
        if (cpOptWeights->GetSize() != crControlPointsList.GetSize())
        {
            SER(SM_ERR_INVALID_INPUT);
        }
    }

    // locals
    ULONG i, j, k, sm, jw;

    // Make sNewUKnots array with one entry for each multiplicity
    SmTArray<double> sNewUKnots;
    for (i = 0; i < crUKnots.GetSize(); i++)
    {
        double dKnot = crUKnots[i];
        long   lMult = crUMultiplicities[i];
        for (long t = 0; t < lMult; t++)
        {
            sNewUKnots.Add(dKnot);
        }
    }

    // Make sNewVKnots array with one entry for each multiplicity
    SmTArray<double> sNewVKnots;
    for (j = 0; j < crVKnots.GetSize(); j++)
    {
        double dKnot = crVKnots[j];
        long lMult = crVMultiplicities[j];
        for (long t = 0; t < lMult; t++)
        {
            sNewVKnots.Add(dKnot);
        }
    }

    // Make sNewWKnots array with one entry for each multiplicity
    SmTArray<double> sNewWKnots;
    for (k = 0; k < crWKnots.GetSize(); k++)
    {
        double dKnot = crWKnots[k];
        long lMult = crWMultiplicities[k];
        for (long t = 0; t < lMult; t++)
        {
            sNewWKnots.Add(dKnot);
        }
    }

    // Get number of ControlPoints
    ULONG lNumUCPts = sNewUKnots.GetSize() - lUDegree - 1;
    ULONG lNumVCPts = sNewVKnots.GetSize() - lVDegree - 1;
    ULONG lNumWCPts = sNewWKnots.GetSize() - lWDegree - 1;

    // check input - number of ControlPoints is consistent with Knot Count and degree
    if (lNumUCPts * lNumVCPts * lNumWCPts != crControlPointsList.GetSize()) SER(SM_ERR_INVALID_INPUT);

    // when given optional weight array
    SmBoolean bNon1Weight = FALSE;
    if (cpOptWeights)
    {
        for (sm = 0; sm < cpOptWeights->GetSize(); sm++)
        {
            // when weights are all 1.0 to within tolerance, make BSpline non-rational. 
            if (smos_Fabs((*cpOptWeights)[sm] - 1.0) > SM_EFF_ZERO)
            {
                bNon1Weight = TRUE;
                break;
            }
        }
    } // end OptWeights existence check - to see if nonRational or not

  // for the weights
    SmTArray<double**> sWeightPtrs(lNumUCPts);
    SmTArray<double*> sWeightRows(lNumUCPts * lNumVCPts);
    SmTArray<double>   sWeights(lNumUCPts * lNumVCPts * lNumWCPts);

    // when BSpline is rational with given weights - use cpOptWeights, else use an array of 1's
    double* daW = NULL;
    if (bNon1Weight && cpOptWeights && cpOptWeights->GetSize() != 0)
    {
        daW = cpOptWeights->GetDataArray();
    }
    else // build and use an array of 1's
    {
        // build an array of 1s
        for (sm = 0; sm < crControlPointsList.GetSize(); sm++)
        {
            sWeights.Add(1.0);
        }
        daW = sWeights.GetDataArray();
    }

    // set up NLib style matrix indirection pointers to look into OptWeight array
    double** sWeightRowData = sWeightRows.GetDataArray();
    for (i = 0, sm = 0, jw = 0; i < lNumUCPts; i++, sm += lNumVCPts)
    {
        sWeightPtrs.Add(&sWeightRowData[sm]);
        for (j = 0; j < lNumVCPts; j++, jw += lNumWCPts)
        {
            sWeightRows.Add(&daW[jw]);
        }
    }

    // set up NLib style ControlPoint array indirection pointers
    SmTArray<NL_POINT**> sCtrlPtsPtrs(lNumUCPts);
    SmTArray<NL_POINT*> sCtrpPtsRows(lNumUCPts * lNumVCPts);
    NL_POINT** sCtrpPtsRowData = sCtrpPtsRows.GetDataArray();
    NL_POINT* paP = SM_REINTERPRET_CAST(NL_POINT*, crControlPointsList.GetDataArray());
    for (i = 0, sm = 0, jw = 0; i < lNumUCPts; i++, sm += lNumVCPts)
    {
        sCtrlPtsPtrs.Add(&sCtrpPtsRowData[sm]);
        for (j = 0; j < lNumVCPts; j++, jw += lNumWCPts)
        {
            sCtrpPtsRows.Add(&paP[jw]);
        }
    }

    // prepare for Nlib call
    NL_STACKS SC;
    gw_VOLUME Vol;
    SmStackHandler sStackKp(&SC);
    N_VolumeInitArrays(&Vol);

    gw_INDEX  k1 = lNumUCPts - 1;
    gw_INDEX  k2 = lNumVCPts - 1;
    gw_INDEX  k3 = lNumWCPts - 1;
    gw_DEGREE m1 = (gw_DEGREE)lUDegree;
    gw_DEGREE m2 = (gw_DEGREE)lVDegree;
    gw_DEGREE m3 = (gw_DEGREE)lWDegree;
    gw_REAL* U = (gw_REAL*)sNewUKnots.GetDataArray();
    gw_REAL* V = (gw_REAL*)sNewVKnots.GetDataArray();
    gw_REAL* W = (gw_REAL*)sNewWKnots.GetDataArray();
    gw_REAL*** w = sWeightPtrs.GetDataArray();
    NL_POINT*** p = sCtrlPtsPtrs.GetDataArray();
    gw_PARAMETER ru[2], rv[2], rw[2];

    // init ru and rv to domain min/max
    if (cpOptUVWDomain)
    {
        ru[0] = cpOptUVWDomain->GetMin().x;
        ru[1] = cpOptUVWDomain->GetMax().x;
        rv[0] = cpOptUVWDomain->GetMin().y;
        rv[1] = cpOptUVWDomain->GetMax().y;
        rw[0] = cpOptUVWDomain->GetMin().z;
        rw[1] = cpOptUVWDomain->GetMax().z;
        SM_ASSERT(ru[0] <= sNewUKnots[m1]);
        SM_ASSERT(ru[1] >= sNewUKnots[lNumUCPts]);
        SM_ASSERT(rv[0] <= sNewVKnots[m2]);
        SM_ASSERT(rv[1] >= sNewVKnots[lNumVCPts]);
        SM_ASSERT(rw[0] <= sNewWKnots[m3]);
        SM_ASSERT(rw[1] >= sNewWKnots[lNumWCPts]);
    }
    else // get ru and rv boundary values from the knot arrays
    {
        ru[0] = sNewUKnots[m1];
        ru[1] = sNewUKnots[lNumUCPts];
        rv[0] = sNewVKnots[m2];
        rv[1] = sNewVKnots[lNumVCPts];
        rw[0] = sNewWKnots[m3];
        rw[1] = sNewWKnots[lNumWCPts];
    }


    // build the Nlib BSpline object
    GW_SER(N_VolumeConstruct(k1, k2, k3, m1, m2, m3, U, V, W, w, p, ru, rv, rw, &Vol, &SC));

    // build NMTlib SmBSpline from Nlib Nurb
    rpNewBSplineVolume = new (crContext) SmBSplineVolume(&crContext, &Vol);

    // all done
    return SM_SUCCESS;

#else

    SM_REF7(crContext, lUDegree, lVDegree, lWDegree, crControlPointsList, crUMultiplicities, crVMultiplicities);
    SM_REF7(crWMultiplicities, crUKnots, crVKnots, crWKnots, cpOptWeights, cpOptUVWDomain, rpNewBSplineVolume);
    return SM_ERR;
#endif // USE_VOLUMES

} // end SmBSplineVolume::CreateCanonical

// gwc: a function to consider adding later on
//      /*******************************************************************//**
//      PURPOSE: Split a B-Spline volume at a given parameter into two 
//                  children volumes.
//      
//      NOTES: 
//        The left volume will have parameter range from U_, V_, W_Min to dParam. 
//        The right volume will have parameter range from dParam to U_, V_, or W_Max.
//      ***********************************************************************/
//      SmStatus SmBSplineVolume::SplitAt
//        (const SmContext    & crContext,      // in : context for new object construction
//         double               dParam,         // in : target split parameter
//         SmVolumeParamType    eVolumeParam,   // in : split direction SM_VP_U, SM_VP_V, SM_VP_W
//         SmVolume          *& rpLeftVolume,   // out: low  range child volume
//         SmVolume          *& rpRightVolume)  // out: high range child volume
//      {
//        // local parametric tolerance
//        double dScaledZero = SM_EFF_ZERO*100.0*(1.0+smos_Fabs(dParam));
//      
//        // If we are close to a knot then snap split to knot value.
//        // Otherwise we could get a very bad volume.
//      
//        // get the knot array to be split
//        double dData[32];
//        SmTArray<double> sKnots(32,dData);
//        GetKnots(eVolumeParam,sKnots);
//      
//        // for every knot - check split-to-knot distance
//        for (ULONG i=0; i<sKnots.GetSize(); i++) 
//          {
//            double dKnot = sKnots[i];
//            if (smos_Fabs(dKnot - dParam) < dScaledZero) 
//              {
//                dParam = dKnot;
//                break ;
//              }
//          } // end iter every knot looking for a snap opportunity
//      
//        // prepare the 
//        gw_VOLUME * pVol = smGet_gw_VOLUME(this);
//        gw_FLAG gwDir =   (eVolumeParam == SM_VP_V) ? NL_UDIR
//                        ? (eVolumeParam == SM_VP_V) ? NL_VDIR
//                        :                             NL_WDIR ;
//        gw_VOLUME * pSurL = NULL;
//        gw_VOLUME * pSurR = NULL;
//      
//        // gwc: a function to consider extending to 3d later on
//        GW_ERR(sm_SplitSrf(pVol,dParam,gwDir,pSurL,pSurR));
//      
//        // set output
//        rpLeftVolume  = new (crContext) SmBSplineVolume(pSurL,FALSE);
//        rpRightVolume = new (crContext) SmBSplineVolume(pSurR,FALSE);
//      
//        // all done
//        return SM_SUCCESS;
//      
//      } // end SmBSplineVolume::SplitAt

/*******************************************************************//**
PURPOSE: Get the degree of a volume in either the U, V, or W parametric
    direction.

NOTES:  When eVolumeParam == SM_VP_ALL, the max
                 degree is returned.
***********************************************************************/
ULONG SmBSplineVolume::GetDegree
  (SmVolumeParamType eVolumeParam)  // in : required direction, oneof:
                                    //      SM_VP_U, SM_VP_V, SM_SP_W
 const
{
  gw_VOLUME *pVolume = smGet_gw_VOLUME(this);
  switch(eVolumeParam)
    {
      case SM_VP_U   : return pVolume->p;
      case SM_VP_V   : return pVolume->q;
      case SM_VP_W   : return pVolume->r;
      case SM_VP_ALL : return smos_3Max(pVolume->p,
                                        pVolume->q,
                                        pVolume->r) ;
      default : MSG(_T("SmBSplineVolume::GetDegree() - bad eVolumeparam argument value\n")) ; 
                       return smos_3Max(pVolume->p,
                                        pVolume->q,
                                        pVolume->r) ;
    } // end switch on eVolumeParam

} // end SmBSplineVolume::GetDegree

/*******************************************************************//**
PURPOSE: Get the number of natural knots in the given parametric 
    direction on the volume.  The number of natural knots
    is the number of knots if the knots are represented as a single array
    with every multiple knot represented explicitly.

NOTES: 
***********************************************************************/
ULONG SmBSplineVolume::GetNumberNaturalKnots
 (SmVolumeParamType eVolumeParam) 
 const
{
  gw_VOLUME *pVol = smGet_gw_VOLUME(this);
  if      (eVolumeParam == SM_VP_U) return pVol->knu->m + 1 ;
  else if (eVolumeParam == SM_VP_V) return pVol->knv->m + 1 ;
  else                              return pVol->knw->m + 1 ;

} // end SmBSplineVolume::GetNumberNaturalKnots

/*******************************************************************//**
PURPOSE: Get the natural UVW domain of the volume.  This is the domain
   which defines the minimum and maximum U, V, and W parameters at which
   the volume can be evaluated at.  

NOTES: Note that x corresponds to U, y corresponds to 
   V, and z corresponds to W in the 3D point form.
***********************************************************************/
SmExtent3d SmBSplineVolume::GetNaturalParamDomain
  () 
 const
{
  gw_VOLUME *pVol = smGet_gw_VOLUME(this);

  // an unbounded volume has no BSplineRepresentation
  if (!pVol) return SmExtent3d();

  // return bounded volume min/max knot values
  SmPoint3d sMinPt, sMaxPt;
  sMinPt.x = pVol->knu->U[0];
  sMaxPt.x = pVol->knu->U[pVol->knu->m];
  sMinPt.y = pVol->knv->U[0];
  sMaxPt.y = pVol->knv->U[pVol->knv->m];
  sMinPt.z = pVol->knw->U[0];
  sMaxPt.z = pVol->knw->U[pVol->knw->m];

  // all done
  return SmExtent3d(sMinPt,sMaxPt);

} // end SmBSplineVolume::GetNaturalParamDomain

/*******************************************************************//**
PURPOSE: Get the number of control points in the given parametric 
    direction from control polygon for this volume.

NOTES: When eVolumeParam == SM_VP_ALL returns
  total number of control points in volume
***********************************************************************/
ULONG SmBSplineVolume::GetNumberControlPoints
  (SmVolumeParamType eVolumeParam) // in : oneof 
                                   //      SM_VP_U    = return U_Dir ControlPoint count      
                                   //      SM_VP_V,   = return V_Dir ControlPoint count
                                   //      SM_VP_W,   = return W_Dir ControlPoint count
                                   //      SM_VP_ALL  = return Total ControlPoint count
 const
{
  gw_VOLUME *pVol = smGet_gw_VOLUME(this);
  return   (eVolumeParam == SM_VP_U) ? (pVol->mesh->m + 1)
         : (eVolumeParam == SM_VP_V) ? (pVol->mesh->n + 1)
         : (eVolumeParam == SM_VP_W) ? (pVol->mesh->o + 1)
         :   (pVol->mesh->m + 1) 
           * (pVol->mesh->n + 1)  
           * (pVol->mesh->o + 1) ;

} // end SmBSplineVolume::GetNumberControlPoints

/*******************************************************************//**
PURPOSE: Get a list of the unique knots and optionally knot multiplicities
    of one of the parameters of a BSpline Volume.

NOTES: This is the STEP compatible form of the knots not the
    typical knot vector associated with NURBS.
***********************************************************************/
SmStatus SmBSplineVolume::GetKnots
  (SmVolumeParamType  eVolumeParam,         // in : SM_VP_U = get U Knot Vector
                                            //      SM_VP_V = get V knot Vector
                                            //      SM_VP_W = get W knot Vector
   SmTArray<double> & rKnots,               // out: unique knot values in requested dimension 
   SmTArray<ULONG>  * pKnotMultiplicities,  // out: associated multiplicity for every knot
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL]
  const
{ 
  // locals   
  gw_VOLUME *pVol = smGet_gw_VOLUME(this);

  // pass the call along
  if      (eVolumeParam == SM_VP_U) { SER(sm_GetKnots(pVol->knu,rKnots,pKnotMultiplicities,pOptIvl));
                                    }
  else if (eVolumeParam == SM_VP_V) { SER(sm_GetKnots(pVol->knv,rKnots,pKnotMultiplicities,pOptIvl));
                                    }
  else                              { SER(sm_GetKnots(pVol->knw,rKnots,pKnotMultiplicities,pOptIvl));
                                    }
  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::GetKnots

/*******************************************************************//**
PURPOSE: Get Euclidean space control points and associated weights
            as an ordered list.

NOTES: Please note that the control points are in Euclidian
    space (not homogeneous) even if the volume is rational.  That is the 
    homogeneous division has been performed on x,y,z prior to returning 
    the data in rControlPointsList.

    ARRAY TO LIST INDEXING:
    pMesh->Pw[i][j] = rControlPointsList[i*rlVCount+j]
***********************************************************************/
SmStatus SmBSplineVolume::GetControlPointMesh
  (ULONG               &rlUCount,              // out: U controlPoint count
   ULONG               &rlVCount,              // out: V controlPoint count
   ULONG               &rlWCount,              // out: W controlPoint count
   SmTArray<SmPoint3d> & rControlPointsList,   // out: Always produced in Euclidian coordinates
   SmTArray<double>    & rWeights)             // out: The Weights array will have size zero if
                                               //      the volume is non-rational.
  const
{
  // locals
  ULONG ii, jj, kk ;
  gw_VOLUME *pVolume = smGet_gw_VOLUME(this);
  NL_CMESH  *pMesh   = pVolume->mesh;

  // set output counts
  rlUCount           = pMesh->m + 1 ;
  rlVCount           = pMesh->n + 1 ;
  rlWCount           = pMesh->o + 1 ;

  // init output arrays
  rControlPointsList.ReSet();
  rControlPointsList.SetSize(rlUCount*rlVCount*rlWCount);

  rWeights.ReSet();
  rWeights.SetSize(rlUCount*rlVCount*rlWCount);

  // for every control point
  SmPoint3d sPnt;
  ULONG lCount=0;
  SmBoolean bIsRational = FALSE;
  for (ii=0; ii<rlUCount; ii++) 
    {
      for (jj=0; jj<rlVCount; jj++)
        { 
          for (kk=0; kk<rlWCount; kk++) 
            {
              // convert to cartesian coordinates as needed
              sPnt.x = pMesh->Pw[ii][jj][kk].x;
              sPnt.y = pMesh->Pw[ii][jj][kk].y;
              sPnt.z = pMesh->Pw[ii][jj][kk].z;

              // set outputs
              if (pMesh->Pw[ii][jj][kk].w != NL_NOW) 
                {
                  bIsRational = TRUE;
                  rWeights[lCount] = pMesh->Pw[ii][jj][kk].w;
                  sPnt.x = sPnt.x / rWeights[lCount];
                  sPnt.y = sPnt.y / rWeights[lCount];
                  sPnt.z = sPnt.z / rWeights[lCount];
                }
              rControlPointsList[lCount++] = sPnt;

            } // end iter every W control point
        } // end iter every V control point
    } // end iter every U control point

  // clear nonRational weight arrays
  if (!bIsRational) rWeights.ReSet();

  return SM_SUCCESS;

} // end SmBSplineVolume::GetControlPointMesh

/*******************************************************************//**
PURPOSE: Get a control point in various forms.

NOTES: 
***********************************************************************/
SmStatus SmBSplineVolume::GetControlPoint
  (SmControlPointFormType eCtrlPointForm, // in : SM_CP_NON_RATIONAL         - do perspective projection and
                                          //                                   set W=1.0 if it is rational.
                                          //      SM_CP_HOMOGENEOUS_RATIONAL - don't do division and return W
                                          //      SM_CP_EUCLIDIAN_RATIONAL   - do division and return W
   ULONG       lUIndex,                   // in : target ControlPoint 1st index
   ULONG       lVIndex,                   // in : target ControlPoint 2nd index
   ULONG       lWIndex,                   // in : target ControlPoint 3rd index
   SmPoint3d & rControlPoint,             // out: ControlPoint position - in requested format
   double    & rdWeight)                  // out: assocaited weight - in requested format
  const
{
  // locals
  double dTmp[4];
  dTmp[3] = 1.0;   // Default for non-rational.
  ULONG lCPtStride = eCtrlPointForm == SM_CP_NON_RATIONAL ? 3: 4 ;

  // get control point components in requested form
  SER(GetControlPointsExpert(eCtrlPointForm,
                             lUIndex,lUIndex,
                             lVIndex,lVIndex,
                             lWIndex,lWIndex,
                             lCPtStride,lCPtStride,lCPtStride,
                             dTmp));

  // set output
  rControlPoint.Set(dTmp[0],dTmp[1],dTmp[2]);
  rdWeight = dTmp[3];

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::GetControlPoint

/*******************************************************************//**
PURPOSE: Get pointer to controlPoint array. 

NOTES: Return pointer to double array of control point
                values stored as [x, y, z, w]

WARNING ---  When a controlPoint value is modified
  internal caches become out of date.  Remember to call 
  Notify(SM_NO_PRE_EDIT, this, NULL) and Notify(SM_NO_POST_EDIT, this, NULL).

  Currently SmVolumes do not have a cahce - but eventually they will.
***********************************************************************/
SmStatus SmBSplineVolume::GetControlPointsPointer
  (ULONG   &lControlPointCountU,   // out: number of control points in each U row
   ULONG   &lControlPointCountV,   // out: number of control points in each V row
   ULONG   &lControlPointCountW,   // out: number of control points in each W row
   double *&pControlPoints)        // out: array of controlPoints stroed as doubles
                                   //      with Pijk = [x y z w]
                                   //      ordered:[P000, P001, .... P00W,
                                   //               P010, P011, .... P01W,
                                   //               . . .
                                   //               P0V0, P0V1, .... P0VW,
                                   //               P10W, P101, .... P10W,
                                   //               . . .
                                   //               PUV0, PUV1, .... PUVW]
                                   //      For 2d control Point   z == NL_NOZ or 0.0
                                   //      for nonRational points w == NL_NOW
                                   //      for Rational points x,y,z are stored in homogeneous space
                                   //        i.e. CartesianX = x/w
                                   //             CartesianY = y/w
                                   //             CartesianZ = (z != NL_NOZ) ? z/w : NL_NOZ ;
const
{
  // locals
  if(m_pNurb == NULL) { ((SmBSplineCurve *)this)->MakeNurb(); } SM_ASSERT(m_pNurb != NULL) ;
  gw_VOLUME *pVol = smGet_gw_VOLUME(this);

  // set output
  lControlPointCountU = pVol->mesh->m + 1 ;
  lControlPointCountV = pVol->mesh->n + 1 ;
  lControlPointCountW = pVol->mesh->o + 1 ;
  pControlPoints      = (double *) **(pVol->mesh->Pw);

  // all done
  return(SM_SUCCESS) ;

} // end SmBSplineVolume::GetControlPointsPointer 

/*******************************************************************//**
PURPOSE: Get a range of knots from the knot vector and put them into
    the double array.  Note it is up to the user to make sure that there
    are at least (lEndIndex - lStartIndex + 1) doubles in the array.

NOTES: KNots are written out by unique index, a multiple
    knot value will be written to the array multiple times.

    Be careful with this method, if you do not input sufficient
    array size it may over write some memory.
***********************************************************************/
SmStatus SmBSplineVolume::GetKnotsExpert
  (SmVolumeParamType eVolumeParam,  // in : oneof: SM_VP_U, SM_VP_V, SM_VP_W
   ULONG             lStartIndex,   // in : 1st knot index to extract
   ULONG             lEndIndex,     // in : last knot index to extract
   double           *adKnots)       // out: specified knot values, 
                                    //      sized:[lEndIndex-lStartIndex+1]
  const
{
  // locals
  gw_VOLUME *pVol = smGet_gw_VOLUME(this);
  gw_KNOTVECTOR *knt;

  // fetch the knot vector
  if      (eVolumeParam == SM_VP_U) { knt = pVol->knu; }
  else if (eVolumeParam == SM_VP_V) { knt = pVol->knv; }
  else                              { knt = pVol->knw; }

  // check input
  SM_ASSERT(lEndIndex   <= (ULONG)knt->m);
  SM_ASSERT(lStartIndex <= lEndIndex);

  // fetch the knot values
  ULONG i, j ;
  for (i=lStartIndex,j=0; i<=lEndIndex; i++,j++) 
    {
      adKnots[j] = knt->U[i];
    }

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::GetKnotsExpert

/*******************************************************************//**
PURPOSE: Get the control points of a given range
         from the control mesh of the volume.

NOTES: Control Point data is packed as:
         SM_CP_NON_RATIONAL         pt = [X Y Z]         PtSize is 3                         
         SM_CP_HOMOGENEOUS_RATIONAL pt = [X Y Z W]       PtSize is 4                         
         SM_CP_EUCLIDIAN_RATIONAL   pt = [X/W Y/W Z/W W] PtSize is 4  
   
   PtSize = ((SM_CP_NON_RATIONAL) ? 3 : 4)                         

   Be memory allocation careful, if you do not input sufficient
   array size the method will overwrite memory.

   ArraySize:[lCtrlPointWStride * (lEndWIndex - lStartWIndex + 1)]

   lCtrlPointUStride: The number of doubles between consectuive U control points in the output array.
     It's a multiple of the number of doubles between consecutive U pts.
   lCtrlPointVStride: The number of doubles between consectuvie V control points in the output array.
     It's a multiple of the number of doubles between consecutive V pts.
   lCtrlPointWStride: The number of doubles between consectuvie W control points in the output array.
     It's a multiple of the number of doubles between consecutive W pts.

   Set lCtrlPointUStride = M * PtSize                                                (Number of doubles between consecutive UCtrlPts)
   Set lCtrlPointVStride = N * ((lEndUIndex - lStartUIndex + 1) * lCtrlPointUStride) (size of one row of UCtrlPts)
   Set lCtrlPointWStride = O * ((lEndVIndex - lStartVIndex + 1) * lCtrlPointVStride) (size of one sheet of U and V CtrlPts)

   where M = number of U points in output pre original U Point step (1 = add no spaces, M = add M-1 pointSpaces between each Pair of U CtrlPts) 
         N = number of V Points in output per original V Point step (1 = add no spaces, N = add N-1 whole row of spaces between each Pair of V CtrlPt rows)
         O = number of W Points in output per original W Point setp (1 = add no spaces, O = add O-1 whoele sheet of spaces between each pari of W CtrlPt sheets)

   Use M, N and O values larger than 1 when setting the Stride values
   if you wish to space the points out in the array.

***********************************************************************/
SmStatus SmBSplineVolume::GetControlPointsExpert
  (SmControlPointFormType eCtrlPointForm, // oneof: SM_CP_NON_RATIONAL           - output euclidean coords only
                                          //        SM_CP_HOMOGENEOUS_RATIONAL   - output homogeneous coords and weights
                                          //        SM_CP_EUCLIDIAN_RATIONAL     - output euclidean coords and weights
   ULONG   lStartUIndex,                  // in: First U control point index to be copied
   ULONG   lEndUIndex,                    // in: Last  U control point index to be copied
   ULONG   lStartVIndex,                  // in: First V control point index to be copied
   ULONG   lEndVIndex,                    // in: Last  V control point index to be copied
   ULONG   lStartWIndex,                  // in: First W control point index to be copied
   ULONG   lEndWIndex,                    // in: Last  W control point index to be copied
   ULONG   lCtrlPointUStride,             // in: spacing between U control point data in output adControlPoints
                                          //     lCtrlPointUStride = M * PtSize
                                          //     where M = number of output U CtrlPts per original U CtrlPt. 
                                          //     M=1 means no spaces between output U CtrlPts points.
                                          //     The 1st output Pt is copied, extra pts left undefined.
   ULONG   lCtrlPointVStride,             // in: spacing between V control point data in output adControlPoints
                                          //     lCtrlPointVStride = N * ((lEndUIndex - lStartUIndex + 1) * lCtrlPointUStride)
                                          //     where N = number of output V CtrlPts per original V CtrlPt.
                                          //     N=1 means no blank V rows between output V CtrlPt rows.
                                          //     The 1st output row is copied (with possible U spacing), extra row values left undefined.
   ULONG   lCtrlPointWStride,             // in: spacing between W control point data in output adControlPoints
                                          //     lCtrlPointWStride = O * ((lEndVIndex - lStartVIndex + 1) * lCtrlPointVStride)
                                          //     where N = number of output V rows per original V row.
                                          //     O=1 means no blank W sheets between output W CtrlPt wheets.
                                          //     The 1st output sheet is copied (with possible U and V spacing), extra sheet values left undefined.
   double *adControlPoints)               // out: SM_CP_NON_RATIONAL         && 2d ? ordered:[X Y          X Y          ...] 
                                          //      SM_CP_HOMOGENEOUS_RATIONAL && 2d ? ordered:[X Y W        X Y W        ...] 
                                          //      SM_CP_EUCLIDIAN_RATIONAL   && 2d ? ordered:[X/W Y/W W    X/W Y/W W    ...] 
                                          //      SM_CP_NON_RATIONAL         && 3d ? ordered:[X Y Z        X Y Z        ...] 
                                          //      SM_CP_HOMOGENEOUS_RATIONAL && 3d ? ordered:[X Y Z W      X Y Z W      ...] 
                                          //      SM_CP_EUCLIDIAN_RATIONAL   && 3d ? ordered:[X/W Y/W Z/W  X/W Y/W Z/W  ...] 
                                          //
                                          //      assumed sized on input - watch for boundary errors!
                                          //      sized:[ lCtrlPointWStride * (lEndWIndex - lStartWIndex + 1)]
  const
{
  // locals
  ULONG iU, iV, iW, lUCnt, lVCnt, lWCnt ;
  gw_VOLUME *pVol        = smGet_gw_VOLUME(this);
  NL_CMESH  *mesh        = pVol->mesh;
  SmBoolean  bIsRational = IsRational();

  ULONG        lPtSize      = eCtrlPointForm == SM_CP_NON_RATIONAL ? 3: 4 ;
  ULONG        lUStrideMult = lCtrlPointUStride / lPtSize ;
  SmBoolean    bOKUStride   = lCtrlPointUStride == (lUStrideMult * lPtSize) ; // must be a multiple of lPtSize
  ULONG        lRowSize     = lCtrlPointUStride * (lEndUIndex - lStartUIndex + 1) ;
  ULONG        lVStrideMult = lCtrlPointVStride / lRowSize ;
  SmBoolean    bOKVStride   = lCtrlPointVStride == (lVStrideMult * lRowSize) ;

  ULONG        lSheetSize   = lCtrlPointVStride * (lEndVIndex - lStartVIndex + 1) ;
  ULONG        lWStrideMult = lCtrlPointWStride / lSheetSize ;
  SmBoolean    bOKWStride   = lCtrlPointWStride == (lWStrideMult * lSheetSize) ;

  // check inputs - consistent indexing and strides
  if(   lStartUIndex > lEndUIndex
     || lEndUIndex   > (ULONG)mesh->m
     || lStartVIndex > lEndVIndex
     || lEndVIndex   > (ULONG)mesh->n
     || lStartWIndex > lEndWIndex
     || lEndWIndex   > (ULONG)mesh->o
     || bOKUStride   != TRUE
     || bOKVStride   != TRUE
     || bOKWStride   != TRUE)
    {
      SER_MSG(SM_ERR_INVALID_INPUT,_T("SmBSplineVolume::GetControlPointsExpert inconsistent input args")); // Can not get a rational from a non-rational curve
    }

  // for every control point
  for (iU=lStartUIndex,lUCnt=0; iU<=lEndUIndex; iU++,lUCnt++) 
    {
      for (iV=lStartVIndex,lVCnt=0; iV<=lEndVIndex; iV++,lVCnt++) 
        {
          for (iW=lStartWIndex,lWCnt=0; iW<=lEndWIndex; iW++,lWCnt++) 
            {
              gw_CPOINT *pCpt = &mesh->Pw[iU][iV][iW];
              double     dX   = pCpt->x;
              double     dY   = pCpt->y;
              double     dZ   = pCpt->z;
              double     dW   = pCpt->w != NL_NOW ? pCpt->w : 1.0 ;
              double    *dAdd = &adControlPoints[  lUCnt*lCtrlPointUStride 
                                                 + lVCnt*lCtrlPointVStride 
                                                 + lWCnt*lCtrlPointWStride];
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
                }
              if (eCtrlPointForm == SM_CP_HOMOGENEOUS_RATIONAL) 
                { 
                  dAdd[0] = dX;
                  dAdd[1] = dY;
                  dAdd[2] = dZ;
                  dAdd[3] = dW;
                }
              if (eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL) 
                {
                  dAdd[0] = dX / dW;
                  dAdd[1] = dY / dW;
                  dAdd[2] = dZ / dW;
                  dAdd[3] = dW;
                }
            } // end iter iW
        }  // end iter iV
    } // end iter iU

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::GetControlPointsExpert

/*******************************************************************//**
PURPOSE: Get the STEP canonical data out of a B-Spline.

NOTES: Please note that the control points are in Euclidian
    space (not homogeneous) even if the volume is rational.  That is the 
    homogeneous division has been performed on x,y,z prior to returning 
    the data in rControlPointsList.
***********************************************************************/
SmStatus SmBSplineVolume::GetCanonical
  (ULONG & rlUDegree,                          // out: degree in U direction
   ULONG & rlVDegree,                          // out: degree in V direction
   ULONG & rlWDegree,                          // out: degree in W direction
   SmTArray<SmPoint3d> & rControlPointsList,   // out: Always produced in Euclidian coordinates
   SmTArray<ULONG> & rUKnotMultiplicities,     // out: U Knot multiplicities 
   SmTArray<ULONG> & rVKnotMultiplicities,     // out: V Knot multiplicities
   SmTArray<ULONG> & rWKnotMultiplicities,     // out: W Knot multiplicities
   SmTArray<double> & rUKnots,                 // out: U unique knot array
   SmTArray<double> & rVKnots,                 // out: V unique knot array
   SmTArray<double> & rWKnots,                 // out: W unique knot array
   SmTArray<double> & rWeights)                // out: The Weights array will have size zero if
                                               //      the volume is non-rational.
    const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;

  // locals
  gw_VOLUME *pVolume = smGet_gw_VOLUME(this);

  // set degree and knot output
  rlUDegree = pVolume->p;
  rlVDegree = pVolume->q;
  rlWDegree = pVolume->r;
  SER(sm_GetKnots(pVolume->knu,rUKnots,&rUKnotMultiplicities));  // get unique knot vectors and their mults
  SER(sm_GetKnots(pVolume->knv,rVKnots,&rVKnotMultiplicities));  // get unique knot vectors and their mults
  SER(sm_GetKnots(pVolume->knw,rWKnots,&rWKnotMultiplicities));  // get unique knot vectors and their mults

  // get control points and weights

  // control point locals
  gw_INDEX ii, jj, kk ;
  NL_CMESH *pMesh = pVolume->mesh;
  rControlPointsList.ReSet();
  rControlPointsList.SetSize((pMesh->m+1)*(pMesh->n+1)*(pMesh->o+1));
  rWeights.ReSet();
  rWeights.SetSize((pMesh->m+1)*(pMesh->n+1)*(pMesh->o+1));
  SmPoint3d sPnt;
  ULONG lCount=0;
  SmBoolean bIsRational = FALSE;

  // for every control point
  for (ii=0; ii<=pMesh->m; ii++) 
    {
      for (jj=0; jj<=pMesh->n; jj++)
        {
          for (kk=0; kk<=pMesh->n; kk++) 
            {
              sPnt.x = pMesh->Pw[ii][jj][kk].x;
              sPnt.y = pMesh->Pw[ii][jj][kk].y;
              sPnt.z = pMesh->Pw[ii][jj][kk].z;
              if (pMesh->Pw[ii][jj][kk].w != NL_NOW) 
                {
                  bIsRational = TRUE;
                  rWeights[lCount] = pMesh->Pw[ii][jj][kk].w;
                  sPnt.x = sPnt.x / rWeights[lCount];
                  sPnt.y = sPnt.y / rWeights[lCount];
                  sPnt.z = sPnt.z / rWeights[lCount];
                }
              rControlPointsList[lCount++] = sPnt;
            }
        }
    }

  if (!bIsRational) rWeights.ReSet();

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::GetCanonical

/*******************************************************************//**
PURPOSE: Edit the volume by setting the canonical form of the 
            NURBS data.

NOTES: Note that the control points are always given
    in Euclidian space even for rational curves.  If you have homogeneous 
    coordinates you need to perform homogeneous division prior to calling
    this method (x = x/w, y=y/w, z=z/w w=w).

    lNumUCPts = sNewUKnots.GetSize() - lUDegree - 1;  n = lNumUCPts - 1
    lNumVCPts = sNewVKnots.GetSize() - lVDegree - 1;  m = lNumVCPts - 1
    lNumWCPts = sNewWKnots.GetSize() - lWDegree - 1;  o = lNumWCPts - 1

    crControlPointsList: sized:[lNumUCPts * lNumVCPts * lNumWCPts]
                         ordered:[P000, P001, ... P00o,           
                                  P010, P011, ... P01o,           
                                  . . .                           
                                  P0m0, P0m1, ... P0mo,           
                                  P100, P101, ... P10o,           
                                  . . .                           
                                  Pnm0, Pnm1, ... Pnmo]           

    The given degrees must be no bigger than the smallest of
    GW_MAX_DEGREE and NL_DMAX which is current the rang 1 to 17.
***********************************************************************/
SmStatus SmBSplineVolume::SetCanonical
  (ULONG                       lUDegree,              // in : U dir Degree                                             
   ULONG                       lVDegree,              // in : V dir Degree                                             
   ULONG                       lWDegree,              // in : W dir Degree                                             
   const SmTArray<SmPoint3d> & crControlPointsList,   // in : Euclidian coordinates                                   
   const SmTArray<ULONG>     & crUKnotMultiplicities, // in : multiplicity of each knot in crUKnots                    
   const SmTArray<ULONG>     & crVKnotMultiplicities, // in : multiplicity of each knot in crVKnots                    
   const SmTArray<ULONG>     & crWKnotMultiplicities, // in : multiplicity of each knot in crWKnots                    
   const SmTArray<double>    & crUKnots,              // in : Knots in U                                               
   const SmTArray<double>    & crVKnots,              // in : Knots in V                                               
   const SmTArray<double>    & crWKnots,              // in : Knots in W                                               
   const SmTArray<double>    * cpOptWeights)          // in : optional weight for each ControlPoint, same size & order 
{
  // Create a temporary one
  SmBSplineVolume *pTmp = NULL ;
  const SmContext *pContext = GetContext();
  NER(pContext);
  SER(SmBSplineVolume::CreateCanonical(*pContext,
                                        lUDegree, lVDegree, lWDegree,
                                        crControlPointsList,
                                        crUKnotMultiplicities, crVKnotMultiplicities, crWKnotMultiplicities, 
                                        crUKnots, crVKnots, crWKnots,
                                        cpOptWeights, NULL, pTmp));

  SmObjDelete sCleanup(pTmp);

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Swap Nurbs with this
  gw_VOLUME *pTmpNurb  = pTmp->m_pNurb;
  pTmp->m_pNurb        = m_pNurb;
  m_pNurb              = pTmpNurb;

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return SM_SUCCESS;

} // end SmBSplineVolume::SetCanonical

/*******************************************************************//**
PURPOSE: Edit the volume by setting one control point.
 
NOTES:
***********************************************************************/
SmStatus SmBSplineVolume::SetControlPoint
  (SmControlPointFormType eCtrlPointForm, // in : SM_CP_EUCLIDIAN_RATIONAL = convert point to homogeneous form
                                          //      SM_CP_NON_RATIONAL       = store point in euclidean space
   ULONG                  lUIndex,        // in : 1st index of Target Mesh CPoint to modify
   ULONG                  lVIndex,        // in : 2nd index of Target Mesh CPoint to modify
   ULONG                  lWIndex,        // in : 3rd index of Target Mesh CPoint to modify
   const SmPoint3d      & crControlPoint, // in : euclidian space point
   double                 dWeight)        // in : associated weight only used when
                                          //      eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL
{
  // locals
  ULONG i, j, k ;
  gw_VOLUME *pVol         = smGet_gw_VOLUME(this);
  NL_CMESH  *mesh         = pVol->mesh;
  SmPoint3d sControlPoint = crControlPoint;
  gw_CPOINT *pCpt         = &mesh->Pw[lUIndex][lVIndex][lWIndex];
  if (eCtrlPointForm == SM_CP_NON_RATIONAL) { dWeight = 1.0; }

  // check input
  if(   lUIndex > (ULONG)mesh->m 
     || lVIndex > (ULONG)mesh->n
     || lWIndex > (ULONG)mesh->o) 
    { SER(SM_ERR); }

  // when asked - convert from euclidean to homogeneous rational form:
  if (eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL)
  {
    sControlPoint.x *= dWeight;
    sControlPoint.y *= dWeight;
    sControlPoint.z *= dWeight;

    // make sure all the other control points are also rational 
    for (i = 0 ; i <= (ULONG)mesh->m ; i++) 
      { 
        for (j = 0 ; j <= (ULONG)mesh->n ; j++)
          {
            for (k = 0 ; k <= (ULONG)mesh->o ; k++)
              {
                gw_CPOINT *pCpti = &mesh->Pw[i][j][k];
                if (pCpti->w == NL_NOW) mesh->Pw[i][j][k].w = 1.0;
              }
          }
      }
  } // end eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL check

  // set the controlPoint components
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  pCpt->x = sControlPoint.x;
  pCpt->y = sControlPoint.y;
  if (pCpt->z != NL_NOZ) { pCpt->z = sControlPoint.z; }
  if ( IsRational() )  { pCpt->w = dWeight; }
                 
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::SetControlPoint

/*******************************************************************//**
PURPOSE: Set the BSplineVolume using data arrays.  This method
    should be flexible enough to handle most forms of NURBS and 
    Bezier volumes which exist in various software products.  It is
    somewhat like the form used by OPENGL.  

NOTES: This is an expert method which should only be used by
    those who are willing to take the risk. 
    
    if (eEndKnotForm == SM_EK_UNCLAMPPED) { lUNumKnots += 2;
                                            lVNumKnots += 2;
                                            lWNumKnots += 2;
                                          }

    lUPointCount = lUNumKnots - lUDegree - 1; m = lUPointCount - 1;
    lVPointCount = lVNumKnots - lVDegree - 1; n = lVPointCount - 1;
    lWPointCount = lWNumKnots - lWDegree - 1; o = lWPointCount - 1;

    Control Points are always stored in sequence as [x y z optional_w],
                   and may be given in 3 different styles
      eCtrlPointForm == SM_CP_NON_RATIONAL          : euclidean point,   sized:[3]
      eCtrlPointForm == SM_CP_HOMOGENEOUS_RATIONAL  : homogeneous point, sized:[4]
      eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL    : euclidean point with assoc weight, sized:[4]

    cadCtrlPoints arrays, sized:[PointSize * (lUPointCount * lVPointCount * lWPointCount)]
      ordered:[anyway you want but lCtrlPointUStride, lCtrlPointVStride, lCtrlPointWStride
               must be set so that
      P[i][j][k].x = cadCtrlPoints[  i * lCtrlPointUStride
                                   + j * lCtrlPointVStride
                                   + k * lCtrlPointWStride]
***********************************************************************/
SmStatus SmBSplineVolume::SetExpert
  (ULONG                  lUDegree,            // in : U dir degree
   ULONG                  lVDegree,            // in : V dir degree
   ULONG                  lWDegree,            // in : W dir degree
   SmEndKnotFormType      eEndKnotForm,        // in : SM_EK_CLAMPPED   = knot arrays contain unused extra endKnot
                                               //      SM_EK_UNCLAMPPED = knot arrays missing unused extra endKnot
   ULONG                  lUNumKnots,          // in : U knot count
   const double          *cadUKnots,           // in : U Knot array
   ULONG                  lVNumKnots,          // in : V knot count
   const double          *cadVKnots,           // in : V Knot array
   ULONG                  lWNumKnots,          // in : W knot count
   const double          *cadWKnots,           // in : W Knot array
   SmControlPointFormType eCtrlPointForm,      // in : SM_CP_NON_RATIONAL         = PointSize = 3
                                               //      SM_CP_HOMOGENEOUS_RATIONAL = PointSize = 4
                                               //      SM_CP_EUCLIDIAN_RATIONAL   = PointSize = 4
   ULONG                  lCtrlPointUStride,   // in : Stride in cadCtrlPoints for each UIndex increment
   ULONG                  lCtrlPointVStride,   // in : Stride in cadCtrlPoints for each VIndex increment
   ULONG                  lCtrlPointWStride,   // in : Stride in cadCtrlPoints for each WIndex increment
   const double          *cadCtrlPoints)       // in : Control Points Array, sized:[PointSize * (lUPointCount * lVPointCount * lWPointCount)]
                                               //      ordered so that
                                               //      P[i][j][k].x = cadCtrlPoints[  i * lCtrlPointUStride
                                               //                                   + j * lCtrlPointVStride
                                               //                                   + k * lCtrlPointWStride]
{
  // locals
  ULONG i, j, k ;

  if (eEndKnotForm == SM_EK_UNCLAMPPED) { lUNumKnots += 2;
                                          lVNumKnots += 2;
                                          lWNumKnots += 2;
                                        }
  ULONG lUCtrlPointCount = lUNumKnots - lUDegree - 1;
  ULONG lVCtrlPointCount = lVNumKnots - lVDegree - 1;
  ULONG lWCtrlPointCount = lWNumKnots - lWDegree - 1;

  // allocate the nurb volume object
  gw_VOLUME *pVol = sm_AllocateNurbVolume(lUCtrlPointCount-1, lVCtrlPointCount-1, lWCtrlPointCount-1,
                                            (gw_DEGREE)lUDegree, (gw_DEGREE)lVDegree, (gw_DEGREE)lWDegree,
                                            lUNumKnots - 1, lVNumKnots - 1, lWNumKnots - 1);

  gw_KNOTVECTOR *pKnu  = pVol->knu;
  gw_KNOTVECTOR *pKnv  = pVol->knv;
  gw_KNOTVECTOR *pKnw  = pVol->knw;
  NL_CMESH      *pMesh = pVol->mesh;
  gw_CPOINT   ***Pw    = pMesh->Pw;

  // copy knot arrays into Volume object
  ULONG lUIdx = 0;
  ULONG lVIdx = 0;
  ULONG lWIdx = 0;
  if (eEndKnotForm == SM_EK_UNCLAMPPED) { pKnu->U[lUIdx++] = cadUKnots[0];
                                          pKnv->U[lVIdx++] = cadVKnots[0];
                                          pKnw->U[lWIdx++] = cadWKnots[0];
                                          lUNumKnots = lUNumKnots - 2;
                                          lVNumKnots = lVNumKnots - 2;
                                          lWNumKnots = lWNumKnots - 2;
                                        }
  for (i=0; i<lUNumKnots; i++)          { pKnu->U[lUIdx++] = cadUKnots[i]; }
  for (j=0; j<lVNumKnots; j++)          { pKnv->U[lVIdx++] = cadVKnots[j]; }
  for (k=0; k<lWNumKnots; k++)          { pKnw->U[lVIdx++] = cadWKnots[k]; }
  if (eEndKnotForm == SM_EK_UNCLAMPPED) { pKnu->U[pKnu->m] = cadUKnots[lUNumKnots-1];
                                          pKnv->U[pKnv->m] = cadVKnots[lVNumKnots-1];
                                          pKnw->U[pKnw->m] = cadWKnots[lWNumKnots-1];
                                        }

  // copy the weight arrays
  for (i=0; i<lUCtrlPointCount; i++) 
    {
      for (j=0; j<lVCtrlPointCount; j++) 
        {
          for (k=0; k<lWCtrlPointCount; k++) 
            {
              const double *dAdd = &cadCtrlPoints[  i*lCtrlPointUStride
                                                  + j*lCtrlPointVStride
                                                  + k*lCtrlPointWStride] ;
              double dX = dAdd[0];                    
              double dY = dAdd[1];
              double dZ = dAdd[2];
              double dW = 1.0;
              if (eCtrlPointForm != SM_CP_NON_RATIONAL) 
                {
                  dW = dAdd[3];
                }
              else 
                { // not rational
                  dW = NL_NOW;
                }
              if (eCtrlPointForm == SM_CP_EUCLIDIAN_RATIONAL) 
                {
                  // Put back into Euclidian form - suitable for
                  // Create Canonical.
                  dX = dX * dW;
                  dY = dY * dW;
                  dZ = dZ * dW;
                }
              Pw[i][j][k].x = dX;
              Pw[i][j][k].y = dY;
              Pw[i][j][k].z = dZ;
              Pw[i][j][k].w = dW;
            }  // end iter every W index
        }  // end iter every V index
    } // end iter every U index
  
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // initialize rest of SmBSplineVolume member values
  m_bNurbIsBorrowed     = FALSE ;
  m_bOutOfBoundsEnabled = FALSE ;

  // Swap Nurbs with this
  if (m_pNurb) { smos_Free(m_pNurb); m_pNurb = NULL ; }
  m_pNurb = pVol;

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::SetExpert

/*******************************************************************//**
PURPOSE: Set Volume->m_pNurb equal to copy of input pGwNurbVolume.

NOTES: 
***********************************************************************/
SmStatus SmBSplineVolume::SetFromGwNurb
  (ULONG,                        // in : Not Used
   gw_VOLUME * pGwNurbVolume)  
{
  // tell the hierarchy we are about to edit the parameterization
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // copy pGwNurbVolume 
  gw_VOLUME * pCopyNurb = sm_AllocateAndCopyNurbVolume(pGwNurbVolume) ;
  // free existing m_pNurb 
  if (m_pNurb && !m_bNurbIsBorrowed) { smos_Free(m_pNurb); m_pNurb = NULL ; }

  // let m_pNurb = copy(cpGwNurbVolume)
  m_pNurb = pCopyNurb ;
  m_bNurbIsBorrowed = FALSE;

  // tell hierarchy parameterization has been edtied
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
  SM_DUMP_AND_ASSERT2_VALID(this) ;
  return SM_SUCCESS;

} // end SmBSplineVolume::SetFromGwNurb

/*******************************************************************//**
PURPOSE: Determines if the volume is rational.

NOTES: 
***********************************************************************/
SmBoolean SmBSplineVolume::IsRational() 
 const
{
  gw_VOLUME *pVol = smGet_gw_VOLUME(this);
  return ( (pVol->mesh->Pw[0][0][0].w == NL_NOW) ? FALSE : TRUE ) ;

} // end SmBSplineVolume::IsRational

/*******************************************************************//**
PURPOSE: Find the span a UV point is in.  It can find the lower 
    left span.

NOTES: 
***********************************************************************/
SmStatus SmBSplineVolume::FindSpans
  (const SmPoint3d & crParamPoint,
   SmBoolean         bUFromLeft,        // in : if P is on U, V, or w interval boundary
   SmBoolean         bVFromLeft,        //      TRUE  = evaluate P in upper interval where P is on the left of the interval
   SmBoolean         bWFromLeft,        //      FALSE = evaluate P in lower interval where P is on the right of the interval
   ULONG           & rUIndex,           // out: 
   ULONG           & rVIndex,           // out: 
   ULONG           & rWIndex)           // out: 
  const
{
  // locals
  gw_VOLUME *pVolume = smGet_gw_VOLUME(this);
  gw_FLAG bULeftFlag = bUFromLeft ? NL_LEFT : NL_RIGHT ;
  gw_FLAG bVLeftFlag = bVFromLeft ? NL_LEFT : NL_RIGHT ;
  gw_FLAG bWLeftFlag = bWFromLeft ? NL_LEFT : NL_RIGHT ;
  gw_INDEX idx_u, idx_v, idx_w;

  // pass the call along
  N_BasisFindSpan(pVolume->knu,pVolume->p,crParamPoint.x,bULeftFlag,&idx_u);
  N_BasisFindSpan(pVolume->knv,pVolume->q,crParamPoint.y,bVLeftFlag,&idx_v);
  N_BasisFindSpan(pVolume->knw,pVolume->r,crParamPoint.z,bWLeftFlag,&idx_w);

  // set output
  rUIndex = idx_u;
  rVIndex = idx_v;
  rWIndex = idx_w;

  // all done
  return SM_SUCCESS;

} // end SmBSplineVolume::FindSpans

/*******************************************************************//**
PURPOSE: Evaluate the NURBS directly without compounding and
    without any stepping off derivative correction techniques.

NOTES: 
***********************************************************************/
SmStatus SmBSplineVolume::EvaluateSimple
 (const SmPoint3d & crParamPoint,         // in : ParamSpace point to map to ProjSpace point
  ULONG             lHighestDeriv,        // in : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3
  SmBoolean         bUFromLeft,           // in : if P is on U, V, or w interval boundary
  SmBoolean         bVFromLeft,           //      TRUE  = evaluate P in upper interval where P is on the left of the interval
  SmBoolean         bWFromLeft,           //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d      * aDerivatives,         // out: matrix of ParamSpace evaluations values
                                          //      3d organized: D[u][v][w]
                                          //      1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3
                                          //      sized       : [n+1][n+1][n+1], where n=lHighesDeriv
                                          //      lHghDrv = 0,   sized: [1],      
                                          //        i=0          order: [D]
                                          //      lHghDrv = 1,   sized: [8]    
                                          //        i=u*4+v*2+w  order: [D  Dw  Dv  ---
                                          //                             Du --- --- ---]
                                          //      lHghDrv = 2,   sized: [27]   
                                          //        i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---
                                          //                             Du  Duw --- Duv --- --- --- --- ---
                                          //                             Duu --- --- --- --- --- --- --- ---]
                                          //      lHghDrv = 3,   sized: [81]   
                                          //        i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw
                                          //                             --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---
                                          //                             Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---
                                          //                             --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  Duuu --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                          //                             --- ---  ---   ---   ---  ---  ---  ---  ---- --- 
                                          //                             --- --- ]
  SmBoolean bNonZeroTangents,             // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                                          //      FALSE= return exact tangent values, default:[TRUE]
                                          //      note: Surprisingly TRUE is the common choice because most tangent uses
                                          //            are for their direction (Binorm, SurfNorm comps), but when the 
                                          //            tangent is being used for its magnitude (like an arc-length comp)
                                          //            then set this to FALSE.
                                          //      default:[TRUE]               
  SmBoolean bDoZeroSampling)              // in : for internal use only, always set to TRUE, default:[TRUE]
const 
{
#ifdef USE_VOLUMES
    // give user a chance to escape out
  /*
static ULONG lCounter = 0;
lCounter ++;
if (lCounter % 10000 == 0)
  {
    if (smos_ExcapeCallback())
      { return SM_ERR; }
  }
  */
  // check input
    if (lHighestDeriv > GW_MAX_DERIV) SER(SM_ERR_INVALID_INPUT);

    // note: in SmBSplineVolume::Evaluate the sUVW point is clamped
    //       to the volume's NaturalUVWDomain right here.
    SmPoint3d sUVW = crParamPoint;  // needed so that &crParamPoint can be same memory as aDerivatives

    // Set up NLib style matrix indirection pointers 
    //   Assign ptrs into aDerivatives to SD.
    //     We are a little tricky casting here because we know
    //     that a NL_POINT and a SmVector3d are both three doubles
    //     in the same order (x,y,z).
    ULONG i, j, sm, jw, jc;
    NL_POINT** SD[GW_MAX_DERIV + 1];
    NL_POINT* SDD[(GW_MAX_DERIV + 1) * (GW_MAX_DERIV + 1)];
    for (i = 0, sm = 0, jw = 0, jc = 0; i <= lHighestDeriv; i++, sm += lHighestDeriv + 1)
    {
        SD[i] = &SDD[sm];
        for (j = 0; j <= lHighestDeriv; j++, jc++, jw += lHighestDeriv + 1)
        {
            SDD[jc] = SM_REINTERPRET_CAST(NL_POINT*, &aDerivatives[jw]);
        }
    }

    // set N_VolumeDerivs flags
    NL_FLAG uflg = (bUFromLeft) ? NL_LEFT : NL_RIGHT;
    NL_FLAG vflg = (bVFromLeft) ? NL_LEFT : NL_RIGHT;
    NL_FLAG wflg = (bWFromLeft) ? NL_LEFT : NL_RIGHT;

    // pass the call along
    NL_FLAG err = 0;
    err = N_VolumeDerivs((gw_VOLUME*)m_pNurb, sUVW.x, sUVW.y, sUVW.z,
        uflg, vflg, wflg, NL_TRUE,
        lHighestDeriv, lHighestDeriv, lHighestDeriv,
        SD);
    if (err == NL_YES)
    {
        SER(SM_ERR);
    }

    // when asked to protect against zero length tangent values
    if (bNonZeroTangents && (lHighestDeriv > 0))
    {
        // position and tangents
        //    watch out for aDerivatives boundary violations,
        //    for now: when a derivative is missing - just assign the first derivative that is available to both values.
        SmPoint3d  sOrigPnt = aDerivatives[0];
        SmVector3d sOrigDU = aDerivatives[(lHighestDeriv + 1) * (lHighestDeriv + 1)];
        SmVector3d sOrigDV = aDerivatives[(lHighestDeriv + 1)];
        SmVector3d sOrigDW = aDerivatives[1];

        // tangent sizes
        double sOrigDULenSq = sOrigDU.LengthSquared();
        double sOrigDVLenSq = sOrigDV.LengthSquared();
        double sOrigDWLenSq = sOrigDW.LengthSquared();

        // tolerance
        double     dTol = SM_EFF_ZERO * (1.0 + sOrigPnt.GetMaxDimension());
        double     dTolSq = dTol * dTol;

        // test
        SmBoolean bZeroDU = sOrigDULenSq < dTolSq;
        SmBoolean bZeroDV = sOrigDVLenSq < dTolSq;
        SmBoolean bZeroDW = sOrigDWLenSq < dTolSq;
        SmBoolean bFixedDU = !bZeroDU;
        SmBoolean bFixedDV = !bZeroDV;
        SmBoolean bFixedDW = !bZeroDW;

        // no work -  no zero tangents to fix
        //    when both original tangents are larger than tolerance.
        //        This used to quit when tangents were short but the
        //             ratio of their lengths was in tolerance.
        //        I've removed that to let the trick attempt to find a nonZeroTangent value
        //             for all short enough tangents.  GWC 
        if (bZeroDU == FALSE && bZeroDV == FALSE && bZeroDW == FALSE)
        {
            return SM_SUCCESS;
        }

        // arrive here to kluge a short properly-directed vector for a zero 1st derivative.
        //   Note: the magnitude of a kluged derivative doesn't matter
        //         because it's not correct anyway.  (It's supposed to be zero.)
        //         So just try to find its direction, and return something short.

        // When working with just one zero tangent vector,
        //    Suv works best for both Su and Sv, else use their respective 2nd derivs.

        // If we're at the 'top' of the domain, i.e., the 'good' parameter is
        // entering a singularity instead of leaving it, then the degenerate
        // derivative is shrinking, so its change (Suv) is in the opposite
        // direction of the derivative.
        SmExtent3d sNaturalUVWDomain = this->GetNaturalParamDomain();
        double dFlipUDeriv = sUVW.x > sNaturalUVWDomain.GetUInterval().GetMid() ? -1.0 : 1.0;
        double dFlipVDeriv = sUVW.y > sNaturalUVWDomain.GetVInterval().GetMid() ? -1.0 : 1.0;
        double dFlipWDeriv = sUVW.z > sNaturalUVWDomain.GetWInterval().GetMid() ? -1.0 : 1.0;

        // check for a point with two zero tangents
        //  I've tried to anticipate this but we may need to rethink the behavior
        //   in response to this rare case.  I suspect we may not get a good solution
        //   without inspecting nearby points.
        //   Example 1 :  a singular side next to a repeated endPoint row
        //     solution:  get nonZeroTangent values in the repeated endPoint direction
        //                at two points - use those to estimate sDUV (the 2nd derivative)
        //                and then use sDUV as the estimate for the tangent in the
        //                the degenerate direction.
        //  Example 2 : a pair of singular sides meeting at a corner
        //    solution:  get estimates from nearby points.  All the derivatives at
        //               the corner will be zero and this trick won't work here.
        //     I've not coded up anything as elaboate as mentioned above - in practice
        //     we may need to do that. For now, I'll output a message so that if
        //     a failed evaluate causes a bigger failure we'll have a hint of where to
        //     look first.
        SmBoolean bTwoZeroTangents = (bZeroDU && bZeroDV)
            || (bZeroDV && bZeroDW)
            || (bZeroDW && bZeroDU);
        // SmBoolean bThreeZeroTangents = (bZeroDU && bZeroDV && bZeroDW) ;

        // when not enough derivatives were asked for - recurse to this function
        //   note: we may have to use the recursion trick when bTwoZeroTangents is TRUE
        //         but we'll ignore that for now.
        if (lHighestDeriv < 2)
        {
            // recurse values
            SmVector3d aReDer[27];

            // pass the call along
            SER(Evaluate(crParamPoint, 2, bUFromLeft, bVFromLeft, bWFromLeft, aReDer, TRUE));

            // save the kluged tangent values
            aDerivatives[(lHighestDeriv + 1) * (lHighestDeriv + 1)] = aReDer[9];  // sDU     
            aDerivatives[(lHighestDeriv + 1)] = aReDer[3];  // sDV     
            aDerivatives[1] = aReDer[1];  // sDW     

            // all done
            return(SM_SUCCESS);

        }  // end recursion branch when asked for less than 2 derivatives

      // try getting 1st Tangent from Suv value
        SmVector3d sDUV = aDerivatives[(lHighestDeriv + 1) * (lHighestDeriv + 1) + (lHighestDeriv + 1)]; // Suv
        SmVector3d sDVW = aDerivatives[lHighestDeriv + 2];                                      // Svw
        SmVector3d sDUW = aDerivatives[(lHighestDeriv + 1) * (lHighestDeriv + 1) + 1];                // Suw

        // try getting 1st tangents from 2nd derivative values that are associated with nonZero 1st derivatives

        // sDU from sDUV
        if (bZeroDU && !bFixedDU
            && !bZeroDV
            && sDUV.LengthSquared() > dTolSq)
        {
            sDUV.Unitize();
            sDUV *= (1.1 * dTol);
            bFixedDU = TRUE;
            aDerivatives[(lHighestDeriv + 1) * (lHighestDeriv + 1)] = dFlipUDeriv * sDUV; // sDU  
        }

        // sDU from sDUW
        if (bZeroDU && !bFixedDU
            && !bZeroDW
            && sDUW.LengthSquared() > dTolSq)
        {
            sDUW.Unitize();
            sDUW *= (1.1 * dTol);
            bFixedDU = TRUE;
            aDerivatives[(lHighestDeriv + 1) * (lHighestDeriv + 1)] = dFlipUDeriv * sDUW; // sDU  
        }

        // sDV from sDUV
        if (bZeroDV && !bFixedDV
            && !bZeroDU
            && sDUV.LengthSquared() > dTolSq)
        {
            sDUV.Unitize();
            sDUV *= (1.1 * dTol);
            bFixedDV = TRUE;
            aDerivatives[(lHighestDeriv + 1)] = dFlipVDeriv * sDUV; // sDV  
        }

        // sDV from sDVW
        if (bZeroDV && !bFixedDV
            && !bZeroDW
            && sDVW.LengthSquared() > dTolSq)
        {
            sDVW.Unitize();
            sDVW *= (1.1 * dTol);
            bFixedDV = TRUE;
            aDerivatives[(lHighestDeriv + 1)] = dFlipVDeriv * sDVW; // sDV  
        }

        // sDW from sDUW
        if (bZeroDW && !bFixedDW
            && !bZeroDU
            && sDUW.LengthSquared() > dTolSq)
        {
            sDUW.Unitize();
            sDUW *= (1.1 * dTol);
            bFixedDW = TRUE;
            aDerivatives[1] = dFlipWDeriv * sDUW; // sDW  
        }

        // sDW from sDVW
        if (bZeroDW && !bFixedDW
            && !bZeroDV
            && sDVW.LengthSquared() > dTolSq)
        {
            sDVW.Unitize();
            sDVW *= (1.1 * dTol);
            bFixedDW = TRUE;
            aDerivatives[1] = dFlipWDeriv * sDVW; // sDW  
        }

        // all done if we fixed all problems
        if (bFixedDU && bFixedDV && bFixedDW)
        {
            return(SM_SUCCESS);
        }

        // 2nd cross derivatives didn't work.
        // arrive here for cases where 
        //    all 1st derivatives are zero or
        //    the nonzero 1st derivatives are associated with zero 2nd cross derivatives.

        // Suv didn't work, try Suu, Svv, Sww for the short tangent

        // Su and Suu
        if (bZeroDU && !bFixedDU)
        {
            SmVector3d sDUU = aDerivatives[2 * (lHighestDeriv + 1) * (lHighestDeriv + 1)];
            if (sDUU.LengthSquared() > dTolSq)
            {
                sDUU.Unitize();
                sDUU *= 1.1 * dTol * dFlipUDeriv;
                aDerivatives[(lHighestDeriv + 1) * (lHighestDeriv + 1)] = sDUU;  // Su
                bFixedDU = TRUE;
            }
        }

        // Sv and Svv
        if (bZeroDV && !bFixedDV)
        {
            SmVector3d sDVV = aDerivatives[2 * (lHighestDeriv + 1)];
            if (sDVV.LengthSquared() > dTolSq)
            {
                sDVV.Unitize();
                sDVV *= 1.1 * dTol * dFlipVDeriv;
                aDerivatives[(lHighestDeriv + 1)] = sDVV;  // Sv
                bFixedDV = TRUE;
            }
        }

        // Sw and Sww
        if (bZeroDW && !bFixedDW)
        {
            SmVector3d sDWW = aDerivatives[2];
            if (sDWW.LengthSquared() > dTolSq)
            {
                sDWW.Unitize();
                sDWW *= 1.1 * dTol * dFlipWDeriv;
                aDerivatives[1] = sDWW;  // Sw
                bFixedDW = TRUE;
            }
        }

        // all done if we fixed all problems
        if (bFixedDU && bFixedDV && bFixedDW)
        {
            return(SM_SUCCESS);
        }

        // arrive here when: need to kluge a direction for a zero 1st derivative, 
        //                   and getting the kluged 1st der direction from 2nd derivative failed.

        //    (bZeroDU && !bFixedDU) 
        // or (bZeroDV && !bFixedDV)
        // or (bZeroDW && !bFixedDW)  - work until they are all Fixed

        // recurse values
        SmVector3d aReDer[27];
        double dStep = 1.0e-5 / 10.0;  // works well in practice.

        // Try finding a nonZeroTangent value with a more brute-force method.
        // Step around and try to find a good one.
        // for larger and larger step off distances
        while (bDoZeroSampling && dStep < 0.1)
        {
            // increase the step off distance
            dStep = dStep * 10.0;

            // for this step size: try 5 directions; last 4 are diagonals.
            SmPoint3d sTestUVW[9];

            // Make the first step go in the most likely direction:
            // direction of good deriv, towards center of volume.
            // This should have the smallest effect on things.
            for (i = 0; i < 9; i++)
            {
                sTestUVW[i] = sUVW;
            }

            // step in U direction 
            if (sOrigDULenSq > sOrigDVLenSq
                && sOrigDULenSq > sOrigDWLenSq)
            {
                sTestUVW[0].x += dFlipUDeriv * dStep;
            }

            // step in V direction
            else if (sOrigDVLenSq > sOrigDULenSq
                && sOrigDVLenSq > sOrigDWLenSq)
            {
                sTestUVW[0].y += dFlipVDeriv * dStep;
            }

            // step in W direction
            else
            {
                sTestUVW[0].z += dFlipWDeriv * dStep;
            }

            // If that doesn't work, just try all eight different octant directions
            sTestUVW[1].x = sUVW.x + dStep;
            sTestUVW[1].y = sUVW.y + dStep;
            sTestUVW[1].z = sUVW.z + dStep;

            sTestUVW[2].x = sUVW.x + dStep;
            sTestUVW[2].y = sUVW.y + dStep;
            sTestUVW[2].z = sUVW.z - dStep;

            sTestUVW[3].x = sUVW.x + dStep;
            sTestUVW[3].y = sUVW.y - dStep;
            sTestUVW[3].z = sUVW.z + dStep;

            sTestUVW[4].x = sUVW.x + dStep;
            sTestUVW[4].y = sUVW.y - dStep;
            sTestUVW[4].z = sUVW.z - dStep;

            sTestUVW[5].x = sUVW.x - dStep;
            sTestUVW[5].y = sUVW.y + dStep;
            sTestUVW[5].z = sUVW.z + dStep;

            sTestUVW[6].x = sUVW.x - dStep;
            sTestUVW[6].y = sUVW.y + dStep;
            sTestUVW[6].z = sUVW.z - dStep;

            sTestUVW[7].x = sUVW.x - dStep;
            sTestUVW[7].y = sUVW.y - dStep;
            sTestUVW[7].z = sUVW.z + dStep;

            sTestUVW[8].x = sUVW.x - dStep;
            sTestUVW[8].y = sUVW.y - dStep;
            sTestUVW[8].z = sUVW.z - dStep;

            // for all 9 sample directions at this step size
            for (i = 0; i < 9; i++)
            {
                // pass the call along
                sTestUVW[i] = sNaturalUVWDomain.ClampPoint3d(sTestUVW[i]);
                SmStatus sRtn = Evaluate(sTestUVW[i], 2, bUFromLeft, bVFromLeft, bWFromLeft, aReDer, TRUE, FALSE);

                // save the kluged tangent values
                if (sRtn == SM_SUCCESS)
                {
                    aDerivatives[(lHighestDeriv + 1) * (lHighestDeriv + 1)] = aReDer[9]; // sDU
                    aDerivatives[(lHighestDeriv + 1)] = aReDer[3]; // sDV
                    aDerivatives[1] = aReDer[1]; // sDW

                    // all done
                    return(SM_SUCCESS);
                }

            } // end iter all test points - recursion branch
        } // end iter all larger step sizes

        if (bTwoZeroTangents)
        {
            SM_DBG_WARN(_T("SM_ERR in SmBSplineSurface::Evaluate - evaluating a surface point with two zero tangents"));
        }

        // If we make it to here we are unable to get both of the first
        // derivatives to a non-zero size.  Return an error.
        return SM_ERR;

    } // end asked to fix nonZeroTangents check

  // all done
    return SM_SUCCESS;

#else
   SM_REF5(crParamPoint, lHighestDeriv, bUFromLeft, bVFromLeft, bWFromLeft);
   SM_REF3(aDerivatives, bNonZeroTangents, bDoZeroSampling);
   return SM_ERR;
#endif // USE_VOLUMES

} // SmBSplineVolume::EvaluateSimple

/*******************************************************************//**
PURPOSE: Compute the ProjSpace Bounding and/or Pseudo Boxes that 
   contain the projection of a given BBox from ParamSpace to ProjSpace.  
    
NOTES:
    For SmBSplineVolumes, the input crParamBox is not used to reduce
    the output BoundingBox sizes.  Those are always sized to circumscribe
    the entire projection of the BSplineVolume. The center of the crParamBox
    is used to compute the orientation of the pProjPseudoBox when pOptParamPseudoBox
    is given.

 pProjPseudoBox orientation
    Default pProjPseudoBox orientation: ProjX = EvaluateSimple(Corner100) - EvaluateSimple(Corner000)
                                        ProjY = EvaluateSimple(Corner010) - EvaluateSimple(Corner000)
                                        ProjZ = EvaluateSimple(Corner001) - EvaluateSimple(Corner000)
    When given pOptParamPseudoBox     : ProjX = EvaluateSimple(pOptParamPseudoBox.GetToXAxis, CenterOf crParamBox)
                                        ProjY = EvaluateSimple(pOptParamPseudoBox.GetToYAxis, CenterOf crParamBox)
                                        ProjZ = EvaluateSimple(pOptParamPseudoBox.GetToZAxis, CenterOf crParamBox)
    In words: Without pOptParamPseudoBox, the output pProjPseudoBox orientation
    is found by projecting a set of axisAligned vectors through this volume's map
    at the center of the given CrParamBox.  When pOptParamPseudoBox is given
    the output pProjPseudoBox orientation is found by projecting the pProjPseudoBox
    basis vectors at the center of the given crParamBox.  

NOTES: One or more of the outputs must be non-NULL.
    Bounding boxes are built for whole volumes not subdomains.
    The cache mechanism subdivides a large volume into a set of
    bezier patches and then calls this function to bound each sub-region.
***********************************************************************/
SmStatus SmBSplineVolume::EvaluateBoundingBoxSimple
  (const SmExtent3d & crParamBox,               // in : ParamSpace BBox to project to Project Space 
   SmPseudoBox      * pOptParamPseudoBox,       // in : optional ParamSpace PseudoBox used to set output PseudoBox orientations,
                                                //      NULL   : ProjPseudoBox Basis = vecs connecting crParamBox corner projections into ProjSpace
                                                //      NotNULL: ProjPseudoBox Basis = Project pOptParamPseudoBox BasisVecs to ProjSpace at crParamBox Center
                                                //      default:[NULL]
   SmExtent3d       * pProjBox,                 // out: ProjectSpace Axis aligned box
   SmPseudoBox      * pProjPseudoBox)           // out: ProjectSpace Non-axis aligned box
  const
{ 
  // no work - no output
  if(   pProjBox == NULL
     && pProjPseudoBox == NULL)
    { return SM_SUCCESS ; }

  // when input ParamBox is same as output ProjBox - copy input ParamBox
  SmExtent3d        sBox ;
  const SmExtent3d *pBox ;
  if(&crParamBox == pProjBox) {  sBox = crParamBox ;
                                 pBox = &sBox ;
                              }
  else                        {  pBox = &crParamBox ;
                              }

  // when input ParamPseudoBox is same as output ProjPseudoBox - copy input ParamPseudoBox
  SmPseudoBox sPBox ;
  // SmPseudoBox *pPBox; // unused
  if(   pProjPseudoBox
     && pOptParamPseudoBox == pProjPseudoBox) { sPBox = *pOptParamPseudoBox ;
                                                // pPBox = &sPBox ;
                                              }
  else                                        { // pPBox = pOptParamPseudoBox ;
                                              }

  // indirection: from here on out - pBox  == ParamSpace Box
  //                               - pPBox == ParamSpace PseudoBox

  // If the interval is equal to the natural interval then we don't need
  // to chop it up and use a sub segment.
  SmExtent3d sNaturalUVWDomain = GetNaturalParamDomain();

  // locals
  const gw_VOLUME *pVol  = smGet_gw_VOLUME(this);
  const NL_CMESH  *pMESH = pVol->mesh;

  // First make sure interval sent in is valid
  // Handle special case of degenerate Volume differently - skip the interval 
  // test.
  if (!pBox->IsContainedBy(sNaturalUVWDomain, SM_EFF_ZERO)) 
    {
      SER(SM_ERR_INVALID_INPUT);
    }
  
  // initialize pProjBox
  if(pProjBox)
    { pProjBox->Init() ; }

  // Compute the basis vectors of the PseudoBox using the control polygon
  if (pProjPseudoBox) 
    {
      // Note that Basis 1 will be in U direction and
      // Basis 2 will be in V direction.
      gw_CPOINT *pCP000 = &pMESH->Pw[0][0][0];
      gw_CPOINT *pCP100 = &pMESH->Pw[pMESH->m][0][0];
      gw_CPOINT *pCP010 = &pMESH->Pw[0][pMESH->n][0];
      gw_CPOINT *pCP110 = &pMESH->Pw[pMESH->m][pMESH->n][0];

      // Use these three points to try to compute a basis vector set
      // If we are unable to then we need to just use the default vectors
      SmPoint3d sP000, sP100, sP010, sP110; 
      TO_EUCLID(*pCP000,sP000);
      TO_EUCLID(*pCP100,sP100);
      TO_EUCLID(*pCP010,sP010);
      TO_EUCLID(*pCP110,sP110);

      SmVector3d sV1 = (sP000 - sP100) + (sP010 - sP110);
      SmVector3d sV2 = (sP000 - sP010) + (sP100 - sP110);
      double sV1LS = sV1.LengthSquared();
      // Note that we will just use the standard basis vectors if the Volume is
      // closed.  If the mid point lies on same line as the start and end point
      // then we will get two arbitrary vectors for Basis2 and Basis3 otherwise V2 will
      // determine the direction for Basis2 and orthogonal direction for Basis3
      if (sV1LS > SM_EFF_ZERO_SQ) 
        {  
          SmVector3d sBasis1, sBasis2, sBasis3;
          sV1.MakeUnitOrthoVectors(&sV2,sBasis1,sBasis2,sBasis3);
          *pProjPseudoBox = SmPseudoBox(); // Initialize it
          pProjPseudoBox->SetBasis(sBasis1,sBasis2,sBasis3);
        } 
      else 
        {
          // initialize it
          pProjPseudoBox->InitBasis() ; // set basis vectors to unit orthogaonal
          pProjPseudoBox->Init() ;      // init intervals to [SM_BIG_DOUBLE -SM_BIG_DOUBLE]
        }
    }

  long i, j, k ;
  SmPoint3d sPoint, sNext ;
  SmPoint3d sVLast, sDV ;
  SmPoint3d sULast, sDU ;
  SmPoint3d sWLast, sDW ;
  SmVector3d sVec ;

  // iter over the BSpline VolumeMesh ControlPoints
  if(   pProjBox 
     || pProjPseudoBox)
    { 
      for (i=0; i<=pMESH->m; i++) 
      { 
        for (j=0; j<=pMESH->n; j++) 
        {
          for (k=0; k<=pMESH->o; k++) 
          {
            // get controlPoint[i][j][k] 
            TO_EUCLID(pMESH->Pw[i][j][k],sPoint);

            // add control points to requested bounding boxes
            if (pProjBox)       { pProjBox->AddPoint3d(sPoint) ; }
            if (pProjPseudoBox) { pProjPseudoBox->AddPoint3d(sPoint) ; }
          } // end iter every w dir controlPoint index
        } // end iter every v dir controlPoint index
      } // end iter every u dir controlPoint index
    } // end need to check control mesh check

  return SM_SUCCESS;

} // end SmBSplineVolume::EvaluateBoundingBoxSimple

/*******************************************************************//**
PURPOSE: Create ProjSpace IsoCurve from a ParamSpace IsoParamLine

NOTES: Specify isoline as pair of constant param values as:
   +-----------------+----------------+----------------+-------------------+
   | eConstantParams | dIsoParam1 | dIsoParam2 | Varying Parameter |
   +-----------------+----------------+----------------+-------------------+
   |    SM_VPS_UV    |   constant u   |   constant v   |     varying w     |
   |    SM_VPS_UW    |   constant u   |   constant w   |     varying v     |
   |    SM_VPS_VW    |   constant v   |   constant w   |     varying u     |
   +-----------------+----------------+----------------+-------------------+

    for SmBSplineVolume - Makes an exact BSpline Curve 
***********************************************************************/
SmStatus SmBSplineVolume::EvaluateIsoParametricCurveSimple
 (const SmContext     & crContext,        // in : context for created objects
  SmVolumeParamsType    eConstantParams,  // in : oneof: SM_VPS_UV_IN,
                                          //             SM_VPS_UW_IN,
                                          //             SM_VPS_VW_IN.
  double                dIsoParam1,       // in : 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value
  double                dIsoParam2,       // in : 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value
  double                d3DTolerance,     // NotUsed: in : Max ApproxCurve to IdealCurve deviation
  SmCurve            *& rpNewIsoCurve,    // out: the ProjSpace IsoCurve
  const SmExtent3d    * pOptParamDomain)  // in : limiting domain, NULL to ignore. default:[NULL]  
 const
{
#ifdef USE_VOLUMES
    SM_REF1(d3DTolerance);
    // check state - skip for now because method is called during transitions
  //  SM_ASSERT_VALID(this) ;

    // get natural or optional domain min/max points
    SmExtent3d sUVWDomain = (pOptParamDomain)
        ? *pOptParamDomain
        : GetNaturalParamDomain();
    double dMinParam1, dMaxParam1;
    double dMinParam2, dMaxParam2;
    if (eConstantParams == SM_VPS_VW) {
        dMinParam1 = sUVWDomain.GetMin().y;
        dMaxParam1 = sUVWDomain.GetMax().y;
        dMinParam2 = sUVWDomain.GetMin().z;
        dMaxParam2 = sUVWDomain.GetMax().z;
    }
    else if (eConstantParams == SM_VPS_UW) {
        dMinParam1 = sUVWDomain.GetMin().x;
        dMaxParam1 = sUVWDomain.GetMax().x;
        dMinParam2 = sUVWDomain.GetMin().z;
        dMaxParam2 = sUVWDomain.GetMax().z;
    }
    else {
        dMinParam1 = sUVWDomain.GetMin().x;
        dMaxParam1 = sUVWDomain.GetMax().x;
        dMinParam2 = sUVWDomain.GetMin().y;
        dMaxParam2 = sUVWDomain.GetMax().y;
    }

    // Clamp Param1 Values within Tolerance to interval boundaries
    if (dIsoParam1 < dMinParam1
        || dIsoParam1 > dMaxParam1)
    {
        SmExtent1d sIvl(dMinParam1, dMaxParam1);

        // error - ParamValue out of boundary tolerance
        if (!sIvl.IsValueOnBoundary(dIsoParam1, SM_EFF_ZERO_SQRT))
        {
            SER(SM_ERR);
        }

        dIsoParam1 = sIvl.ClampValue(dIsoParam1);

    } // end need to clamp boundary

  // Clamp Param2 Values within Tolerance to interval boundaries
    if (dIsoParam2 < dMinParam2
        || dIsoParam2 > dMaxParam2)
    {
        SmExtent1d sIvl(dMinParam2, dMaxParam2);

        // error - ParamValue out of boundary tolerance
        if (!sIvl.IsValueOnBoundary(dIsoParam2, SM_EFF_ZERO_SQRT))
        {
            SER(SM_ERR);
        }

        dIsoParam2 = sIvl.ClampValue(dIsoParam2);

    } // end need to clamp boundary

  // locals
    gw_FLAG dir;
    gw_INDEX  lCpointHighest;
    gw_DEGREE lDegree;
    gw_INDEX  lKnotsHighest;
    gw_VOLUME* pVolume = smGet_gw_VOLUME(this);
    if (eConstantParams == SM_VPS_VW)
    {
        dir = NL_UDIR;  // variable U curves go in U direction
        lCpointHighest = pVolume->mesh->m;
        lDegree = pVolume->p;
        lKnotsHighest = pVolume->knu->m;
    }
    else if (eConstantParams == SM_VPS_UW)
    {
        dir = NL_VDIR;  // variable V curves go in V direction
        lCpointHighest = pVolume->mesh->n;
        lDegree = pVolume->q;
        lKnotsHighest = pVolume->knv->m;
    }
    else
    {
        dir = NL_WDIR;  // variable W curves go in W direction
        lCpointHighest = pVolume->mesh->o;
        lDegree = pVolume->r;
        lKnotsHighest = pVolume->knw->m;
    }

    // allocate the ObjectMemory
    gw_CURVE* cur = sm_AllocateNurbCurve(lCpointHighest, lDegree, lKnotsHighest);
    NER(cur);
    SmMemDelete sCleanup(cur);

    // build the isoParameter gw_CURVE
    NL_STACKS SL;  // note: SL won't be used because cur is already allocated but must be provided
    //       to make a valid C language call
    GW_ERR(N_VolumeMakeIsoCurve(pVolume, dIsoParam1, dIsoParam2, dir, cur, &SL));

    // Use it as m_pNurb in a BSplineCurve object
    rpNewIsoCurve = new (crContext) SmBSplineCurve(cur, 3);
    NER(rpNewIsoCurve);
    sCleanup.Clear();

    // when asked - trim curve to optional domain limits
    if (pOptParamDomain)
    {
        SmExtent1d sTrimIvl = eConstantParams == SM_VPS_VW ? sUVWDomain.GetUInterval()
            : eConstantParams == SM_VPS_UW ? sUVWDomain.GetVInterval()
            : sUVWDomain.GetWInterval();
        rpNewIsoCurve->Trim(sTrimIvl); // may snap sIvl by tol to existing knots
    }

    // all done
    return SM_SUCCESS;
#else
    SM_REF7(crContext, eConstantParams, dIsoParam1, dIsoParam2, d3DTolerance, rpNewIsoCurve, pOptParamDomain);
    return SM_ERR;
#endif // USE_VOLUMES

} // end SmBSplineVolume::EvaluateIsoParametricCurveSimple

/*******************************************************************//**
PURPOSE: Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane

NOTES: Specify IsoPlane as a constant param value as:
     +----------------+---------------+--------------------+
     | eConstantParam | dIsoParam | varying parameters |
     +----------------+---------------+--------------------+
     |   SM_VP_U      | constant u    | varying v, w       |
     |   SM_VP_V      | constant v    | varying u, w       |
     |   SM_VP_W      | constant w    | varying u, v       |
     +----------------+---------------+--------------------+

    for SmBSplineVolume - Make a BSpline Surface           (exact)
***********************************************************************/
SmStatus SmBSplineVolume::EvaluateIsoParametricSurfaceSimple
 (const SmContext   & crContext,        // in : context for created objects
  SmVolumeParamType   eConstantParam,   // in : oneof: SM_VP_U_IN,
                                        //             SM_VP_V_IN,
                                        //             SM_VP_W_IN
  double              dIsoParam,        // in : constant param value
  double              d3DTolerance,     // NotUsed: in : Max ApproxSurface to IdealSurface deviation        
  SmSurface        *& rpNewIsoSurface,  // out: the ProjSpace IsoSurface                        
  const SmExtent3d  * pOptParamDomain)  // in : limiting domain, NULL to ignore. default:[NULL]
const
{
#ifdef USE_VOLUMES
    SM_REF1(d3DTolerance);
    // check state - skip for now because method is called during transitions
  //  SM_ASSERT_VALID(this) ;

    // get natural or optional domain min/max points
    SmExtent3d sUVWDomain = (pOptParamDomain)
        ? *pOptParamDomain
        : GetNaturalParamDomain();
    double dMinParam, dMaxParam;
    if (eConstantParam == SM_VP_U) {
        dMinParam = sUVWDomain.GetMin().x;
        dMaxParam = sUVWDomain.GetMax().x;
    }
    else if (eConstantParam == SM_VP_V) {
        dMinParam = sUVWDomain.GetMin().y;
        dMaxParam = sUVWDomain.GetMax().y;
    }
    else {
        dMinParam = sUVWDomain.GetMin().z;
        dMaxParam = sUVWDomain.GetMax().z;
    }

    // Clamp Param Values within Tolerance to interval boundaries
    if (dIsoParam < dMinParam
        || dIsoParam > dMaxParam)
    {
        SmExtent1d sIvl(dMinParam, dMaxParam);

        // error - ParamValue out of boundary tolerance
        if (!sIvl.IsValueOnBoundary(dIsoParam, SM_EFF_ZERO_SQRT))
        {
            SER(SM_ERR);
        }

        dIsoParam = sIvl.ClampValue(dIsoParam);

    } // end need to clamp boundary

  // locals
    gw_FLAG dir;
    gw_INDEX  lCpointHighest1, lCpointHighest2;
    gw_DEGREE lDegree1, lDegree2;
    gw_INDEX  lKnotsHighest1, lKnotsHighest2;
    gw_VOLUME* pVolume = smGet_gw_VOLUME(this);
    if (eConstantParam == SM_VP_U)
    {
        dir = NL_UDIR;  // constant U surfaces go in VW direction
        lCpointHighest1 = pVolume->mesh->n;
        lDegree1 = pVolume->q;
        lKnotsHighest1 = pVolume->knv->m;
        lCpointHighest2 = pVolume->mesh->o;
        lDegree2 = pVolume->r;
        lKnotsHighest2 = pVolume->knw->m;
    }
    else if (eConstantParam == SM_VP_V)
    {
        dir = NL_VDIR;  // constant V surfaces go in VW direction
        lCpointHighest1 = pVolume->mesh->m;
        lDegree1 = pVolume->p;
        lKnotsHighest1 = pVolume->knu->m;
        lCpointHighest2 = pVolume->mesh->o;
        lDegree2 = pVolume->r;
        lKnotsHighest2 = pVolume->knw->m;
    }
    else
    {
        dir = NL_WDIR;  // constant W surfaces go in UV direction
        lCpointHighest1 = pVolume->mesh->m;
        lDegree1 = pVolume->p;
        lKnotsHighest1 = pVolume->knu->m;
        lCpointHighest2 = pVolume->mesh->n;
        lDegree2 = pVolume->q;
        lKnotsHighest2 = pVolume->knv->m;
    }

    // allocate the ObjectMemory
    gw_SURFACE* sur = sm_AllocateNurbSurface(lCpointHighest1, lCpointHighest2,
                                             lDegree1, lDegree2,
                                             lKnotsHighest1, lKnotsHighest2);
    NER(sur);
    SmMemDelete sCleanup(sur);

    // build the isoParameter gw_CURVE
    NL_STACKS SL;  // note: SL won't be used because sur is already allocated but must be provided
    //       to make a valid C language call
    GW_ERR(N_VolumeMakeIsoSurface(pVolume, dIsoParam, dir, sur, &SL));

    // Use it as m_pNurb in a BSplineSurface object
    rpNewIsoSurface = new (crContext) SmBSplineSurface(sur, 3);
    NER(rpNewIsoSurface);
    sCleanup.Clear();

    // when asked - trim surface to optional domain limits
    if (pOptParamDomain)
    {
        SmExtent2d sTrimDomain = eConstantParam == SM_VP_U ? sUVWDomain.GetVWDomain()
            : eConstantParam == SM_VP_V ? sUVWDomain.GetUWDomain()
            : sUVWDomain.GetUVDomain();

        // Trim the IsoSurface
        //  - should be parameterized the same as the original surface since it's a BSplineSurface and not an Analytic
        rpNewIsoSurface->TrimWithDomain(sTrimDomain);
    }

    // all done
    return SM_SUCCESS;
#else
    SM_REF6(crContext, eConstantParam, dIsoParam, d3DTolerance, rpNewIsoSurface, pOptParamDomain);
    return SM_ERR;
#endif // USE_VOLUMES

} // end SmBSplineVolume::EvaluateIsoParametricSurfaceSimple

/*******************************************************************//**
PURPOSE: Compute a Parameter Point that is near to 
            a give XYZ space Target Point to be used
            as the start of a Newton Raphson iteration.

NOTES:
***********************************************************************/
SmStatus SmBSplineVolume::InvEvaluateGuessPointSimple
 (const SmPoint3d     & crProjPoint,        // in : ProjSpace Point to map back to ParamSpace Point
  SmTArray<SmPoint3d> & rGuessParamPoints)  // out: ParamSpace Point near Target Point actual map back to ParamSpace
 const
{
  // init output
  rGuessParamPoints.SetSize(1) ;

  // locals
  NL_INDEX i,  j,  k ;
  NL_INDEX i_min = 0, j_min = 0, k_min = 0 ;
  double   dThisDist2, dMinDist2 = SM_BIG_DOUBLE ;
  NL_VOLUME *pVolume = (NL_VOLUME *)m_pNurb ;
  NL_CMESH  *pCMesh  = pVolume->mesh ; 
  NL_POINT   sPoint ;

  // find closest ControlPoint to crProjPoint
  for(i=0;i<=pCMesh->m;i++)
    {
      for(j=0;j<=pCMesh->n;j++)
        {
          for(k=0;k<=pCMesh->o;k++)
            {
              NL_CPOINT *pCPoint = &(pCMesh->Pw[i][j][k]) ;
              N_CPtToPtEuclid(*pCPoint,&sPoint);

              // get CPoint/TargetPoint distance
              //
              // Temp workaround to appease Linux gcc compiler
              // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
              // dThisDist2 = crProjPoint.DistanceBetweenSquared((const SmVector3d &)sPoint) ;

              SmVector3d sVector;
              sVector.Set(sPoint.x, sPoint.y, sPoint.z);
              
              dThisDist2 = crProjPoint.DistanceBetweenSquared(sVector);
              
              // save closest point
              if(dThisDist2 < dMinDist2)
                {
                  dMinDist2 = dThisDist2 ;
                  i_min     = i ; 
                  j_min     = j ; 
                  k_min     = k ; 

                } // end found a keeper check
            } // end iter every W index
        } // end iter every V index
    } // end iter every U index

  // build a UVW GuessPoint from the controlPoint
  SM_ASSERT(i_min <= pVolume->knu->m - pVolume->p) ;
  SM_ASSERT(j_min <= pVolume->knv->m - pVolume->q) ;
  SM_ASSERT(k_min <= pVolume->knw->m - pVolume->r) ;

  // get u, v, w guesses
  double dU=0.0, dV=0.0, dW=0.0 ;
  for(i=0;i<=pVolume->p;i++) dU += pVolume->knu->U[i_min+i] ;
  for(j=0;j<=pVolume->q;j++) dV += pVolume->knv->U[j_min+j] ;
  for(k=0;k<=pVolume->r;k++) dW += pVolume->knw->U[k_min+k] ;
  dU /= (double)(pVolume->p + 1) ; 
  dV /= (double)(pVolume->q + 1) ; 
  dW /= (double)(pVolume->r + 1) ; 
  rGuessParamPoints[0].Set(dU, dV, dW) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmBSplineVolume::InvEvaluateGuessPointSimple 

/*******************************************************************//**
PURPOSE: Modify SimpleMap to incorporate an OutSpace Translate modification so that

           Orient(ModifiedSimpleMap(ParamPt)) = Translate(Orient(SimpleMap(ParamPt)))  and
           ModifiedSimpleMap(ParamPt)         = InvOrient(Translate(Orient(SimpleMap(ParamPt))))  

NOTES: The compounded mapping InvOrient(Translate(Orient())) is a sequence
       of affine mappings which can built into a single affine transformation.

       This method builds that transform and then applies it to the
       control points of the BSPlineVolume.

       The purpose of this method is performance.  Without this call
       BSplineVolume ParamPoints are projected to OutSpace through
       three mappings as

          ParamSpace -SimpleMap-> ProjSpace -OrientMap-> OutSpace -Translate-> FinalOutSpace

       After this call the same mapping will be done in two mappings
          
          ParamSpace -ModifiedSimpleMap-> ProjSpace -OrientMap-> FinalOutSpace

       Any derived SmVolume classes that can build the mirror mapping directly 
       into the SimpleMap should do so in the virtual method TranslateSimple() 
       for efficiency.
***********************************************************************/
SmStatus SmBSplineVolume::TranslateSimple            
 (const SmVector3d & crOutTranslate)  // in : Outspace translate vector  
{
  // build InvOrient(Translate(Orient())) transform SMLib style
  SmTransform sT ; // identity mapping with no OrientMap

  // Orient
  if(GetOrientMap())
    { sT.ConcatTransform(*GetOrientMap()) ; }

  // Translate
  sT.TranslateSimple(crOutTranslate) ;

  // InvOrient
  if(GetInvOrientMap())
    { sT.ConcatTransform(*GetInvOrientMap()) ; }

  // apply the transformation to the volume
  SmStatus sStatus = sm_LoadNLibTransform(sT, *this) ;

  // all done
  return sStatus ; 

} // end SmBSplineVolume::TranslateSimple

/*******************************************************************//**
PURPOSE: Modify SimpleMap to incorporate an OutSpace Mirror modification so that

           Orient(ModifiedSimpleMap(ParamPt)) = Mirror(Orient(SimpleMap(ParamPt)))  and
           ModifiedSimpleMap(ParamPt)         = InvOrient(Mirror(Orient(SimpleMap(ParamPt))))  

NOTES: The compounded mapping InvOrient(Mirror(Orient())) is a sequence
       of affine mappings which can built into a single affine transformation.

       This method builds that transform and then applies it to the
       control points of the BSPlineVolume.

       The purpose of this method is performance.  Without this call
       BSplineVolume ParamPoints are projected to OutSpace through
       three mappings as

          ParamSpace -SimpleMap-> ProjSpace -OrientMap-> OutSpace -Mirror-> FinalOutSpace

       After this call the same mapping will be done in two mappings
          
          ParamSpace -ModifiedSimpleMap-> ProjSpace -OrientMap-> FinalOutSpace

       Any derived SmVolume classes that can build the mirror mapping directly 
       into the SimpleMap should do so in the virtual method MirrorSimple() 
       for efficiency.
***********************************************************************/
SmStatus SmBSplineVolume::MirrorSimple            
 (const SmPoint3d  & crPlaneProjPt,      // in : Pt on Mirror Plane given in ProjSpace
  const SmVector3d & crPlaneProjNormal)  // in : Normal to Mirror Plane given in ProjSpace
{
  // build InvOrient(Mirror(Orient())) transform SMLib style
  SmTransform sT ; // identity mapping with no OrientMap

  // Orient
  if(GetOrientMap())
    { sT.ConcatTransform(*GetOrientMap()) ; }

  // Mirror
  sT.MirrorSimple(crPlaneProjPt, crPlaneProjNormal) ;

  // InvOrient
  if(GetInvOrientMap())
    { sT.ConcatTransform(*GetInvOrientMap()) ; }

  // apply the transformation to the volume
  SmStatus sStatus = sm_LoadNLibTransform(sT, *this) ;

  // all done
  return sStatus ; 

} // end SmBSplineVolume::MirrorSimple

/*******************************************************************//**
PURPOSE: Modify SimpleMap to incorporate an OutSpace Scale modification so that

           Orient(ModifiedSimpleMap(ParamPt)) = Scale(Orient(SimpleMap(ParamPt)))  and
           ModifiedSimpleMap(ParamPt)         = InvOrient(Scale(Orient(SimpleMap(ParamPt))))  

NOTES: The compounded mapping InvOrient(Scale(Orient())) is a sequence
       of affine mappings which can built into a single affine transformation.

       This method builds that transform and then applies it to the
       control points of the BSPlineVolume.

       The purpose of this method is performance.  Without this call
       BSplineVolume ParamPoints are projected to OutSpace through
       three mappings as

          ParamSpace -SimpleMap-> ProjSpace -OrientMap-> OutSpace -Scale-> FinalOutSpace

       After this call the same mapping will be done in two mappings
          
          ParamSpace -ModifiedSimpleMap-> ProjSpace -OrientMap-> FinalOutSpace

       Any derived SmVolume classes that can build the mirror mapping directly 
       into the SimpleMap should do so in the virtual method ScaleSimple() 
       for efficiency.
***********************************************************************/
SmStatus SmBSplineVolume::ScaleSimple            
 (const SmVector3d & crScaleOutVec,   // in : OutSpace scale factors
  const SmPoint3d  * cpOptOutCenter)  // in : Optional OutSpace scaling center point
{
  // build InvOrient(Scale(Orient())) transform SMLib style
  SmTransform sT ; // identity mapping with no OrientMap

  // Orient
  if(GetOrientMap())
    { sT.ConcatTransform(*GetOrientMap()) ; }

  // Scale
  sT.ScaleSimple(crScaleOutVec, cpOptOutCenter) ;

  // InvOrient
  if(GetInvOrientMap())
    { sT.ConcatTransform(*GetInvOrientMap()) ; }

  // apply the transformation to the volume
  SmStatus sStatus = sm_LoadNLibTransform(sT, *this) ;

  // all done
  return sStatus ; 

} // end SmBSplineVolume::ScaleSimple

/*******************************************************************//**
PURPOSE: Modify SimpleMap to incorporate an OutSpace RotateAboutAxis modification so that

           Orient(ModifiedSimpleMap(ParamPt)) = RotateAboutAxis(Orient(SimpleMap(ParamPt)))  and
           ModifiedSimpleMap(ParamPt)         = InvOrient(RotateAboutAxis(Orient(SimpleMap(ParamPt))))  

NOTES: The compounded mapping InvOrient(RotateAboutAxis(Orient())) is a sequence
       of affine mappings which can built into a single affine transformation.

       This method builds that transform and then applies it to the
       control points of the BSPlineVolume.

       The purpose of this method is performance.  Without this call
       BSplineVolume ParamPoints are projected to OutSpace through
       three mappings as

          ParamSpace -SimpleMap-> ProjSpace -OrientMap-> OutSpace -RotateAboutAxis-> FinalOutSpace

       After this call the same mapping will be done in two mappings
          
          ParamSpace -ModifiedSimpleMap-> ProjSpace -OrientMap-> FinalOutSpace

       Any derived SmVolume classes that can build the mirror mapping directly 
       into the SimpleMap should do so in the virtual method RotateAboutAxisSimple() 
       for efficiency.
***********************************************************************/
SmStatus SmBSplineVolume::RotateAboutAxisSimple            
 (double             dAngRad,    // in : OutSpace rotation AngRad 
  const SmVector3d & crOutAxis)  // in : OutSpace rotation axis 
{
  // build InvOrient(RotateAboutAxis(Orient())) transform SMLib style
  SmTransform sT ; // identity mapping with no OrientMap

  // Orient
  if(GetOrientMap())
    { sT.ConcatTransform(*GetOrientMap()) ; }

  // RotateAboutAxis
  sT.RotateAboutAxisSimple(dAngRad, crOutAxis) ;

  // InvOrient
  if(GetInvOrientMap())
    { sT.ConcatTransform(*GetInvOrientMap()) ; }

  // apply the transformation to the volume
  SmStatus sStatus = sm_LoadNLibTransform(sT, *this) ;

  // all done
  return sStatus ; 

} // end SmBSplineVolume::RotateAboutAxisSimple

/*******************************************************************//**
PURPOSE: Modify SimpleMap to incorporate an OutSpace RotateAboutAxisAtPoint modification so that

           Orient(ModifiedSimpleMap(ParamPt)) = RotateAboutAxisAtPoint(Orient(SimpleMap(ParamPt)))  and
           ModifiedSimpleMap(ParamPt)         = InvOrient(RotateAboutAxisAtPoint(Orient(SimpleMap(ParamPt))))  

NOTES: The compounded mapping InvOrient(RotateAboutAxisAtPoint(Orient())) is a sequence
       of affine mappings which can built into a single affine transformation.

       This method builds that transform and then applies it to the
       control points of the BSPlineVolume.

       The purpose of this method is performance.  Without this call
       BSplineVolume ParamPoints are projected to OutSpace through
       three mappings as

          ParamSpace -SimpleMap-> ProjSpace -OrientMap-> OutSpace -RotateAboutAxisAtPoint-> FinalOutSpace

       After this call the same mapping will be done in two mappings
          
          ParamSpace -ModifiedSimpleMap-> ProjSpace -OrientMap-> FinalOutSpace

       Any derived SmVolume classes that can build the mirror mapping directly 
       into the SimpleMap should do so in the virtual method RotateAboutAxisAtPointSimple() 
       for efficiency.
***********************************************************************/
SmStatus SmBSplineVolume::RotateAboutAxisAtPointSimple            
 (double             dAngRad,     // in : OutSpace rotation AngRad 
  const SmPoint3d  & crOutOrigin, // in : OutSpace point on rotation axis
  const SmVector3d & crOutAxis)   // in : OutSpace rotation axis direction
{
  // build InvOrient(RotateAboutAxisAtPoint(Orient())) transform SMLib style
  SmTransform sT ; // identity mapping with no OrientMap

  // Orient
  if(GetOrientMap())
    { sT.ConcatTransform(*GetOrientMap()) ; }

  // RotateAboutAxisAtPoint
  sT.RotateAboutAxisAtPointSimple(dAngRad, crOutOrigin, crOutAxis) ;

  // InvOrient
  if(GetInvOrientMap())
    { sT.ConcatTransform(*GetInvOrientMap()) ; }

  // apply the transformation to the volume
  SmStatus sStatus = sm_LoadNLibTransform(sT, *this) ;

  // all done
  return sStatus ; 

} // end SmBSplineVolume::RotateAboutAxisAtPointSimple

/*******************************************************************//**
PURPOSE: Modify SimpleMap to incorporate an OutSpace Transform modification so that

           Orient(ModifiedSimpleMap(ParamPt)) = Transform(Orient(SimpleMap(ParamPt)))  and
           ModifiedSimpleMap(ParamPt)         = InvOrient(Transform(Orient(SimpleMap(ParamPt))))  

NOTES: The compounded mapping InvOrient(Transform(Orient())) is a sequence
       of affine mappings which can built into a single affine transformation.

       This method builds that transform and then applies it to the
       control points of the BSPlineVolume.

       The purpose of this method is performance.  Without this call
       BSplineVolume ParamPoints are projected to OutSpace through
       three mappings as

          ParamSpace -SimpleMap-> ProjSpace -OrientMap-> OutSpace -Transform-> FinalOutSpace

       After this call the same mapping will be done in two mappings
          
          ParamSpace -ModifiedSimpleMap-> ProjSpace -OrientMap-> FinalOutSpace

       Any derived SmVolume classes that can build the mirror mapping directly 
       into the SimpleMap should do so in the virtual method TransformSimple() 
       for efficiency.
***********************************************************************/
SmStatus SmBSplineVolume::TransformSimple            
 (const SmAxis2Placement & crOutRotateNMove,  // in : OutSpace Rotate and Move Transform 
  const SmVector3d       * cpOptOutScale)     // in : Optional OutSpace scale factors applied after Rotate and Move
{
  // build InvOrient(Transform(Orient())) transform SMLib style
  SmTransform sT ; // identity mapping with no OrientMap

  // Orient
  if(GetOrientMap())
    { sT.ConcatTransform(*GetOrientMap()) ; }

  // Transform
  sT.TransformSimple(crOutRotateNMove, cpOptOutScale) ;

  // InvOrient
  if(GetInvOrientMap())
    { sT.ConcatTransform(*GetInvOrientMap()) ; }

  // apply the transformation to the volume
  SmStatus sStatus = sm_LoadNLibTransform(sT, *this) ;

  // all done
  return sStatus ; 

} // end SmBSplineVolume::TransformSimple

/*******************************************************************//**
PURPOSE: Write SmBSplineVolume to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmBSplineVolume::WriteToDB
 (SmDatabaseIO & rDB,              // in : target output stream
  ULONG          lDBVersionNumber) // in : database version to get proper sequence of writes                                                                       
 const
{
#ifdef USE_VOLUMES
    gw_REAL  wx, wy, wz, w;

    // file type, ASCII or BINARY
    SmFileType eType = rDB.GetFileType();

    // Volume locals 
    gw_VOLUME* vol = ((SmBSplineVolume*)this)->GetOrCreateGwNurbPointer();
    gw_INDEX   i, j, k, m, n, o, ir, is, it;
    gw_DEGREE  p, q, r;
    gw_CPOINT*** Pw;
    gw_REAL* U, * V, * W;
    N_VolumeGetCPtsDegreesAndKnots(vol, &n, &m, &o, &Pw, &p, &q, &r, &ir, &is, &it, &U, &V, &W);

    // rational flag 
    gw_FLAG  rat = N_VolumeIsRat(vol) ? NL_YES : NL_NO;

    // Create the output file 
    if (eType == SM_ASCII) {
        std::ostream& rFileOut = *rDB.GetOutStreamPtr();
        rFileOut << n << " " << m << " " << o << "\n";
        rFileOut << p << " " << q << " " << r << "\n";
        rFileOut << rat << "\n";
    }
    else /* Binary */ {
        SER(rDB.WriteLong(m));
        SER(rDB.WriteLong(n));
        SER(rDB.WriteLong(o));
        SER(rDB.WriteShort(p));
        SER(rDB.WriteShort(q));
        SER(rDB.WriteShort(r));
        SER(rDB.WriteShort(rat));
    }

    // switch on rational/nonRational to output control points
    switch (rat)
    {
        // Non-rational
    case NL_NO: { // for every control point
        for (i = 0; i <= m; i++)
        {
            for (j = 0; j <= n; j++)
            {
                for (k = 0; k <= o; k++)
                {
                    N_CPtToWxWyWz(Pw[i][j][k], &wx, &wy, &wz, &w);

                    if (eType == SM_ASCII)
                    {
                        std::ostream& rFileOut = *rDB.GetOutStreamPtr();
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
                } // end iter every controlPoint k
            } // end iter every controlPoint j
        } // end iter every controlPoint i
    } // end non-rational branch
              break;

              // Rational
    case NL_YES: { // for every control point
        for (i = 0; i <= m; i++)
        {
            for (j = 0; j <= n; j++)
            {
                for (k = 0; k <= o; k++)
                {
                    N_CPtToWxWyWz(Pw[i][j][k], &wx, &wy, &wz, &w);

                    if (eType == SM_ASCII) {
                        std::ostream& rFileOut = *rDB.GetOutStreamPtr();
                        rFileOut << wx << " " << wy << " " << wz << " " << w << "\n";
                    }
                    else /* Binary */ {
                        SER(rDB.WriteDouble(wx));
                        SER(rDB.WriteDouble(wy));
                        SER(rDB.WriteDouble(wz));
                        SER(rDB.WriteDouble(w));
                        //    rFileOut.write((char*)&wx,sizeof(gw_REAL));
                        //    rFileOut.write((char*)&wy,sizeof(gw_REAL));
                        //    rFileOut.write((char*)&wz,sizeof(gw_REAL));
                        //    rFileOut.write((char*)&w,sizeof(gw_REAL));
                    }
                } // end iter every controlPoint k
            } // end iter every controlPoint j
        } // end iter every controlPoint i
    } // end non-rational branch
               break;

               // Wrong type
    default: { SER(SM_ERR); }

    } // end switch on rational/nonRational to output control points

  // output u knots
    for (i = 0; i <= ir; i++)
    {
        if (eType == SM_ASCII) {
            std::ostream& rFileOut = *rDB.GetOutStreamPtr();
            rFileOut << U[i] << "\n";
        }
        else /* Binary */ {
            SER(rDB.WriteDouble(U[i]));
            //  rFileOut.write((char*)&U[i],sizeof(gw_REAL));
        }
    }

    // output v knots
    for (j = 0; j <= is; j++)
    {
        if (eType == SM_ASCII) {
            std::ostream& rFileOut = *rDB.GetOutStreamPtr();
            rFileOut << V[j] << "\n";
        }
        else /* Binary */ {
            SER(rDB.WriteDouble(V[j]));
            //  rFileOut.write((char*)&V[j],sizeof(gw_REAL));
        }
    }

    // output w knots
    for (k = 0; k <= it; k++)
    {
        if (eType == SM_ASCII) {
            std::ostream& rFileOut = *rDB.GetOutStreamPtr();
            rFileOut << W[k] << "\n";
        }
        else /* Binary */ {
            SER(rDB.WriteDouble(W[k]));
            //  rFileOut.write((char*)&W[k],sizeof(gw_REAL));
        }
    }

    // Write Base class type
    SmVolume::WriteToDB(rDB, lDBVersionNumber);

    // all done
    return SM_SUCCESS;
#else
    SM_REF2(rDB, lDBVersionNumber);
    return SM_ERR;
#endif // USE_VOLUMES

} // end SmBSplineVolume::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmBSplineVolume from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmBSplineVolume::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  const SmContext & crContext,          // in : context for new object construction
  SmVolume       *& rpNewVolume,        // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
#ifdef USE_VOLUMES
    SM_REF1(lType);

    // check input
    SER((rpNewVolume == NULL
        || rpNewVolume->IsKindOf(SmBSplineVolume_TYPE)) ? SM_SUCCESS : SM_ERR);

    // init output object
    SmBSplineVolume* pBSplineVolume = (rpNewVolume == NULL)
        ? new (crContext) SmBSplineVolume()
        : (SmBSplineVolume*)rpNewVolume;

    // file type
    SmFileType eType = rDB.GetFileType();

    // locals
    gw_REAL    wx, wy, wz, w;

    // Get parameters from the top 
    gw_INDEX   i, j, k, m, n, o;
    gw_DEGREE  p, q, r;
    gw_FLAG    rat;
    //ULONG lKnotType          = (ULONG) SM_KT_UNSPECIFIED ;
    //ULONG lBSplineVolumeForm = (ULONG) SM_SF_UNSPECIFIED ;

    if (eType == SM_ASCII) {
        std::istream& rFileIn = *rDB.GetInStreamPtr();
        rFileIn >> m >> n >> o; rDB.GoToNextLine();
        rFileIn >> p >> q >> r; rDB.GoToNextLine();
        rFileIn >> rat;          rDB.GoToNextLine();
    }
    else /* Binary */ {
        SER(rDB.ReadLong(m));
        SER(rDB.ReadLong(n));
        SER(rDB.ReadLong(o));
        SER(rDB.ReadShort(p));
        SER(rDB.ReadShort(q));
        SER(rDB.ReadShort(r));
        SER(rDB.ReadShort(rat));
    }

    // volume knot counts
    gw_INDEX ir = m + p + 1;
    gw_INDEX is = n + q + 1;
    gw_INDEX it = o + r + 1;

    // allocate Sur memory 
    gw_VOLUME* vol = sm_AllocateNurbVolume(m, n, o, p, q, r, ir, is, it);
    if (vol == NULL)
    {
        SER(SM_ERR);
    }

    // volume locals

    gw_CPOINT*** Pw;
    gw_REAL* U, * V, * W;
    N_VolumeGetCPtsAndKnots(vol, &Pw, &U, &V, &W);


    // Read in data 
    switch (rat)
    {
        // Non-rational
    case NL_NO: { // for every control point   
        for (i = 0; i <= m; i++)
        {
            for (j = 0; j <= n; j++)
            {
                for (k = 0; k <= o; k++)
                {

                    if (eType == SM_ASCII) {
                        std::istream& rFileIn = *rDB.GetInStreamPtr();
                        rFileIn >> wx >> wy >> wz; rDB.GoToNextLine();
                    }
                    else /* Binary */ {
                        SER(rDB.ReadDouble(wx));
                        SER(rDB.ReadDouble(wy));
                        SER(rDB.ReadDouble(wz));
                        //   rFileIn.read((char*)&wx,sizeof(gw_REAL));
                        //   rFileIn.read((char*)&wy,sizeof(gw_REAL));
                        //   rFileIn.read((char*)&wz,sizeof(gw_REAL));
                    }
                    N_CPtFromWxWyWz(wx, wy, wz, NL_NOW, &Pw[i][j][k]);
                } // end iter every ControlPoint k
            } // end iter every ControlPoint j
        } // end iter every ControlPoint i
    } break;

        // Rational
    case NL_YES: { // for every control point  
        for (i = 0; i <= m; i++)
        {
            for (j = 0; j <= n; j++)
            {
                for (k = 0; k <= o; k++)
                {

                    if (eType == SM_ASCII) {
                        std::istream& rFileIn = *rDB.GetInStreamPtr();
                        rFileIn >> wx >> wy >> wz >> w;  rDB.GoToNextLine();
                    }
                    else /* Binary */ {
                        SER(rDB.ReadDouble(wx));
                        SER(rDB.ReadDouble(wy));
                        SER(rDB.ReadDouble(wz));
                        SER(rDB.ReadDouble(w));
                        // rFileIn.read((char*)&wx,sizeof(gw_REAL));
                        // rFileIn.read((char*)&wy,sizeof(gw_REAL));
                        // rFileIn.read((char*)&wz,sizeof(gw_REAL));
                        // rFileIn.read((char*)&w,sizeof(gw_REAL));
                    }
                    N_CPtFromWxWyWz(wx, wy, wz, w, &Pw[i][j][k]);
                } // end iter every ControlPoint k
            } // end iter every ControlPoint j
        } // end iter every ControlPoint i
    } break;

        // Wrong type
    default: { SER(SM_ERR); }
    }

    // for every U knot
    for (i = 0; i <= ir; i++)
    {
        if (eType == SM_ASCII) {
            std::istream& rFileIn = *rDB.GetInStreamPtr();
            rFileIn >> U[i]; rDB.GoToNextLine();
        }
        else /* Binary */ {
            SER(rDB.ReadDouble(U[i]));
            //  rFileIn.read((char*)&U[i],sizeof(gw_REAL));
        }
    }

    // for every V knot
    for (j = 0; j <= is; j++)
    {
        if (eType == SM_ASCII) {
            std::istream& rFileIn = *rDB.GetInStreamPtr();
            rFileIn >> V[j]; rDB.GoToNextLine();
        }
        else /* Binary */ {
            SER(rDB.ReadDouble(V[j]));
            // rFileIn.read((char*)&V[j],sizeof(gw_REAL));
        }
    }

    // for every W knot
    for (k = 0; k <= it; k++)
    {
        if (eType == SM_ASCII) {
            std::istream& rFileIn = *rDB.GetInStreamPtr();
            rFileIn >> W[k]; rDB.GoToNextLine();
        }
        else /* Binary */ {
            SER(rDB.ReadDouble(W[k]));
            // rFileIn.read((char*)&W[k],sizeof(gw_REAL));
        }
    }

    // set output
    pBSplineVolume->m_pNurb = vol;
    pBSplineVolume->m_bNurbIsBorrowed = FALSE;
    pBSplineVolume->m_bOutOfBoundsEnabled = FALSE;
    rpNewVolume = pBSplineVolume;

    // read base class data
    SmVolume::ReadFromDB(SmVolume_TYPE, rDB, crContext, rpNewVolume, lDBVersionNumber);

    // all done
    return SM_SUCCESS;

#else
  SM_REF1(lType) ;
  SM_REF4(rDB, crContext, lDBVersionNumber, rpNewVolume);
  return SM_ERR;
#endif // USE_VOLUMES

} // end SmBSplineVolume::ReadFromDB

/*******************************************************************//**
PURPOSE: Compute various bounding values for a partial control mesh
   of this volume.  

NOTES: 
***********************************************************************/
SmStatus SmBSplineVolume::CalculatePartialMeshValues
  (const SmExtent3d & crParamDomain,           // in : ParamSpace domain to query
   SmPoint3d          aCorners[2][2][2],       // out: ParamDomain corners mapped to ProjSpace, ordered:[aCorners[u][v][w]]
   SmExtent3d       * pOptNormalBox,           // out: ProjSpace bounding box, NULL to ignore, default:[NULL]
   SmPseudoBox      * pOptPseudoBox,           // out: ProjSpace pseudo box, NULL to ignore, default:[NULL]
   SmPoint3d        * pOptUVWChordHeight,      // out: Max projX,Y,Z Chord height along any single row of varying uvw CPoints
                                               //      NULL to ignore, default:[NULL]
   SmPoint3d        * pOptUVWAngleTolDegree)   // out: Max ProjSpace tangAngle change along any single row of varying uvw CPoints
                                               //      NULL to ignore, default:[NULL]
  const             
{
  // locals
  gw_VOLUME *pVol = smGet_gw_VOLUME(this);
  ULONG lU0Span, lV0Span, lW0Span, lU1Span, lV1Span, lW1Span;

  SER(FindSpans(crParamDomain.GetMin(),TRUE, TRUE, TRUE, lU0Span,lV0Span,lW0Span));
  SER(FindSpans(crParamDomain.GetMax(),FALSE,FALSE,FALSE,lU1Span,lV1Span,lW1Span));
  double *pUChord = NULL;
  double *pVChord = NULL;
  double *pWChord = NULL;
  if (pOptUVWChordHeight) 
    {
      pUChord = &pOptUVWChordHeight->x;
      pVChord = &pOptUVWChordHeight->y;
      pWChord = &pOptUVWChordHeight->z;
    }
  double *pUAng = NULL;
  double *pVAng = NULL;
  double *pWAng = NULL;
  if (pOptUVWAngleTolDegree) 
    {
      pUAng = &pOptUVWAngleTolDegree->x;
      pVAng = &pOptUVWAngleTolDegree->y;
      pWAng = &pOptUVWAngleTolDegree->z;
    }
  SER(sm_ComputePartialMeshConstants
            (pVol,
             lU0Span,lV0Span,lW0Span,
             lU1Span,lV1Span,lW1Span,
             pOptNormalBox,pOptPseudoBox,
             aCorners[0][0][0],aCorners[1][0][0],aCorners[0][1][0],aCorners[1][1][0],
             aCorners[0][0][1],aCorners[1][0][1],aCorners[0][1][1],aCorners[1][1][1],
             pUChord,pVChord,pWChord,
             pUAng,  pVAng,  pWAng  ));

  return SM_SUCCESS;

} // end SmBSplineVolume::CalculatePartialMeshValues

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmBSplineVolume.

NOTES: includes attribute memory
***********************************************************************/
ULONG SmBSplineVolume::GetMemoryUsed        // rtn: smaller size of actually used memory in bytes
 (ULONG    & rlMemoryAllocated,     // out: bigger size of all allocated memory in bytes
  SmMarkType eMarkType)             // in : uses without increment eMarkType value
 const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + m_pNurb memory
  rlMemoryAllocated = sizeof(*this) + sm_ComputeNurbVolumeSize(m_pNurb) ;

  // + attribute memory
  ULONG lThisAllocated ;
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                                       eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // all done
  return(lUsed) ;

} // end SmBSplineVolume::GetMemoryUsed

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmBSplineVolume::IsKindOf( SM_TYPE t ) const
{
  return ((SmBSplineVolume_TYPE == t) ? TRUE : SmVolume::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump bspline volume data out for debugging.
            Dump is in similar sequence to Write to file

NOTES: 
***********************************************************************/
void SmBSplineVolume::Dump() 
  const
{
  // pass the call along
  this->SmBSplineVolume::Dump(FALSE, 0);
} // end SmBSplineVolume::Dump


/*******************************************************************//**
PURPOSE: Dump preceded by a one line message

NOTES: 
***********************************************************************/
void SmBSplineVolume::Dump
  (TCHAR * message,      // in : Text line label written prior to dump
   ULONG   lIndentCnt)   // in : Indent print statements by lIndentCnt number of spaces
 const
{
  // output message string
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%s "), message);
  smos_WriteBuffer(sBuff);

  // pass the call along
  this->SmBSplineVolume::Dump(FALSE, lIndentCnt);

} // end SmBSplineVolume::Dump

/*******************************************************************//**
PURPOSE: Dump preceded by an integer (ULONG)

NOTES: 
***********************************************************************/
void SmBSplineVolume::Dump
  (ULONG i,             // in : number label written prior to dump
   ULONG lIndentCnt)    // in : Indent print statements by lIndentCnt number of spaces
 const
{
  
  // output integer
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%lu "),i);
  smos_WriteBuffer(sBuff);
  
  // pass the call along
  this->SmBSplineVolume::Dump(FALSE, lIndentCnt);

} // end SmBSplineVolume::Dump

/*******************************************************************//**
PURPOSE: Dump with abbreviated option

NOTES: if bAbbrev = TRUE just dump 1st and last control point
         and knots
Note: Control points are converted to Euclidean for printout, if Rational
***********************************************************************/

void SmBSplineVolume::Dump
 (SmBoolean bAbbrev,      // in : TRUE = skip nested object dumps, FALSE=include nested object dumps
  ULONG     lIndentCnt)   // in : Indent print statements by lIndentCnt number of spaces
 const
{
#ifdef USE_VOLUMES
  // locals
  ULONG i ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE], sIndent[32] = {};

  // build prefix
  for(i=0;i<lIndentCnt;i++) 
    { SM_STRCAT(sIndent, _T(" ")) ; }
  
  // header
  smos_sprintf(sBuff,        _T("\n%sBegin SmBSplineVolume[0x%p]::Dump()"), sIndent, this) ;
  smos_sprintf(sBuffForFile, _T("\n%sBegin SmBSplineVolume::Dump()"), sIndent) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // locals
  ULONG j, k ;
  gw_INDEX ii, jj, kk ;
  gw_VOLUME *pVol  = smGet_gw_VOLUME(this);    
  NL_CMESH  *pMesh = pVol->mesh;
  gw_FLAG    lrat  = ( N_VolumeIsRat(pVol) ) ? 1: 0;

  // knot locals
  SmTArray<double> sUKnots, sVKnots, sWKnots;
  SmTArray<ULONG>  sUMult,  sVMult, sWMult;
  GetKnots(SM_VP_U,sUKnots,&sUMult);
  GetKnots(SM_VP_V,sVKnots,&sVMult);
  GetKnots(SM_VP_W,sWKnots,&sWMult);
  ULONG lUKnotCount = 0 ; for(i=0;i<sUMult.GetSize();i++) { lUKnotCount += sUMult[i] ; }
  ULONG lVKnotCount = 0 ; for(j=0;j<sVMult.GetSize();j++) { lVKnotCount += sVMult[j] ; }
  ULONG lWKnotCount = 0 ; for(k=0;k<sWMult.GetSize();k++) { lWKnotCount += sWMult[k] ; }

  // BSplineVolume Pointer
  smos_sprintf(sBuff,_T("\n%s%s SmBSplineVolume = 0x%p "), sIndent, 
             lrat ? _T("Rational") : _T("NonRational"),
             this);
  smos_sprintf(sBuffForFile,_T("\n%s%s SmBSplineVolume = %s "), sIndent, 
             lrat ? _T("Rational") : _T("NonRational"),
             _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // ControlPoint Counts, Degrees, RationalFlag
  smos_sprintf(sBuff,_T("\n%s   # CP_u = %ld, # CP_v = %ld, # CP_w = %ld\n    Deg_u = %d,  Deg_v = %d,  Deg_w = %d"), sIndent, 
             pMesh->m+1, pMesh->n+1, pMesh->o+1, 
             pVol->p, pVol->q, pVol->r);
  smos_WriteBuffer(sBuff);

  // Unique Knot Counts
  smos_sprintf(sBuff,_T("\n%s   # Unique U Knots = %lu, # Unique V Knots = %lu, # Unique W Knots = %lu "), sIndent, 
             sUKnots.GetSize(), sVKnots.GetSize(), sWKnots.GetSize());
  smos_WriteBuffer(sBuff);

  // Total Knot Counts
  smos_sprintf(sBuff,_T("\n%s   # Total  U Knots = %lu, # Total  V Knots = %lu, # Total  W Knots = %lu "), sIndent, 
             lUKnotCount, lVKnotCount, lWKnotCount);
  smos_WriteBuffer(sBuff);

  // output every or just the corner controlPoints
  ULONG inc_m = bAbbrev ? pMesh->m : 1;
  ULONG inc_n = bAbbrev ? pMesh->n : 1;
  ULONG inc_o = bAbbrev ? pMesh->o : 1;

  smos_sprintf(sBuff,_T("\n%s   ControlPoint Locations"), sIndent) ; 
  smos_WriteBuffer(sBuff);
  for (ii=0; ii<=pMesh->m; ii+= inc_m) 
    {
      for (jj=0; jj<=pMesh->n; jj+= inc_n) 
        {
          for (kk=0; kk<=pMesh->o; kk+= inc_o) 
            {
              //      SmPoint3d sPnt;
              //      TO_EUCLID(pMesh->Pw[ii][jj][kk],sPnt);
              //      smos_sprintf(sBuff,_T("\n     [%lu][%lu][%lu] = "),ii,jj,kk);
              //      smos_WriteBuffer(sBuff);
              //      sPnt.Dump(bAbbrev);

              // output PointIndex, x, y, z, optional w
              smos_sprintf(sBuff,_T("\n%s  [%lu][%lu][%lu] = %16.16lf %16.16lf %16.16lf"), sIndent,
                                  ii,jj,kk, 
                                  pMesh->Pw[ii][jj][kk].x,
                                  pMesh->Pw[ii][jj][kk].y,
                                  pMesh->Pw[ii][jj][kk].z);
              smos_WriteBuffer(sBuff);
              if (pMesh->Pw[ii][jj][kk].w != NL_NOW) { smos_sprintf(sBuff,_T(" %16.16lf"),pMesh->Pw[ii][jj][kk].w);
                                                       smos_WriteBuffer(sBuff);
                                                     }
            }
          smos_WriteBuffer(_T("\n"));
        }
      smos_WriteBuffer(_T("\n"));
    } // end iter every controlPoint

  // output every or just the end knots
  ULONG inc_ir = bAbbrev ? sUKnots.GetSize()-1 : 1 ;
  ULONG inc_is = bAbbrev ? sVKnots.GetSize()-1 : 1 ;
  ULONG inc_it = bAbbrev ? sWKnots.GetSize()-1 : 1 ;

  // output u knots
  smos_WriteBuffer(_T("\n%s   KnotU [value, multiplicity]"), sIndent);
  inc_m = bAbbrev ? sUKnots.GetSize()-1 : 1;
  for (i=0; i<sUKnots.GetSize(); i+= inc_ir) 
    {
      if(i==0) smos_sprintf(sBuff,_T("\n%s     UniqueKnot[%lu] = %16.16lf, %lu"), sIndent, i, sUKnots[i],sUMult[i]);
      else     smos_sprintf(sBuff,_T("\n%s               [%lu] = %16.16lf, %lu"), sIndent, i, sUKnots[i],sUMult[i]);
      smos_WriteBuffer(sBuff);
    }

  // output v knots
  smos_WriteBuffer(_T("\n%s   KnotV [value, multiplicity]"), sIndent);
  inc_n = bAbbrev ? sVKnots.GetSize()-1 : 1;
  for (j=0; j<sVKnots.GetSize(); j+= inc_is) 
    {
      if(j==0) smos_sprintf(sBuff,_T("\n%s     UniqueKnot[%lu] = %16.16lf, %lu"), sIndent, j, sVKnots[j],sVMult[j]);
      else     smos_sprintf(sBuff,_T("\n%s               [%lu] = %16.16lf, %lu"), sIndent, j, sVKnots[j],sVMult[j]);
      smos_WriteBuffer(sBuff);
    }

  // output w knots
  smos_WriteBuffer(_T("\n   KnotW [value, multiplicity]"));
  inc_n = bAbbrev ? sWKnots.GetSize()-1 : 1;
  for (k=0; k<sWKnots.GetSize(); k+= inc_it) 
    {
      if(k==0) smos_sprintf(sBuff,_T("\n%s     UniqueKnot[%lu] = %16.16lf, %lu"), sIndent, k, sWKnots[k],sWMult[k]);
      else     smos_sprintf(sBuff,_T("\n%s               [%lu] = %16.16lf, %lu"), sIndent, k, sWKnots[k],sWMult[k]);
      smos_WriteBuffer(sBuff);
    }

  // dump the base class 
  SmVolume::Dump(bAbbrev, lIndentCnt+2) ;

  smos_sprintf(sBuff,        _T("\n%sEnd SmBSplineVolume[0x%p]::Dump()%s"), sIndent, this, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_sprintf(sBuffForFile, _T("\n%sEnd SmBSplineVolume::Dump()%s"), sIndent, lIndentCnt==0?_T("\n"):_T("")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
#else
    SM_REF2(bAbbrev, lIndentCnt);  
#endif // USE_VOLUMES

} // end SmBSplineVolume::Dump

/*******************************************************************//**
PURPOSE: Given a type of validity check to perform, determine if the
    volume passes.

NOTES: 
***********************************************************************/
SmBoolean SmBSplineVolume::PassesValidityCheck
  (SmValidityCheckType   eChecks,        // in : SM_VC_NONE,                                
                                         //      SM_VC_ALL,                                 
                                         //      SM_VC_DEFINITION,                          
                                         //      SM_VC_SELF_INTERSECTION,   // Not supported
                                         //      SM_VC_CUSPS,               // Not supported
                                         //      SM_VC_SMOOTH               // Not supported
   SmValidityCheckType & reCheckFailed)  // out: The first run check that fails
  const
{
#ifdef USE_VOLUMES
    // init output
    reCheckFailed = SM_VC_NONE;

    // gwc rewrote init: don't call AssertValid here - AssertValid calls PassesValidityCheck()
    //   reCheckFailed = AssertValid() ? SM_VC_NONE 
    //                                 : SM_VC_DEFINITION ;

    // Check for validity of volume knots
    ULONG ii, mm;
    double           dKData[256];
    ULONG            lMData[256];
    SmTArray<double> sKnots(256, dKData);
    SmTArray<ULONG>  sKnotMult(256, lMData);

    for (ii = 0; ii < 3; ii++)
    {
        ULONG lDeg = 0;
        if (ii == 0) {
            SER(GetKnots(SM_VP_U, sKnots, &sKnotMult));
            lDeg = GetDegree(SM_VP_U);
        }
        else if (ii == 1) {
            SER(GetKnots(SM_VP_V, sKnots, &sKnotMult));
            lDeg = GetDegree(SM_VP_V);
        }
        else if (ii == 2) {
            SER(GetKnots(SM_VP_W, sKnots, &sKnotMult));
            lDeg = GetDegree(SM_VP_W);
        }
        if (sKnotMult[0] > lDeg + 1) {
            reCheckFailed = SM_VC_DEFINITION;
            return FALSE;
        }
        if (sKnotMult.GetLast() > lDeg + 1) {
            reCheckFailed = SM_VC_DEFINITION;
            return FALSE;
        }
        for (mm = 1; mm < sKnotMult.GetSize() - 1; mm++)
        {
            if (sKnotMult[mm] > lDeg) {
                reCheckFailed = SM_VC_DEFINITION;
                return FALSE;
            }
        }
    }

    // pass the call along to NLib if asked
    if (eChecks == SM_VC_ALL
        || eChecks == SM_VC_DEFINITION)
    {
        gw_FLAG err = N_VolumeIsValid(smGet_gw_VOLUME(this), _T("SmBSplineCurve::PassesValidityCheck"));
        if (err) {
            reCheckFailed = SM_VC_DEFINITION;
            return FALSE;
        }
    }

    // This should check for the volume being of zero length in one direction
    SmBoolean bSingularU, bSingularV, bSingularW;
    SmPoint3d sUVWMid = GetNaturalParamDomain().Evaluate(0.5, 0.5, 0.5);
    if (IsSingularity(sUVWMid, bSingularU, bSingularV, bSingularW))
    {
        return FALSE;
    }

    // pass the call along to the base class
    return SmVolume::AssertValid();
#else
    SM_REF2(eChecks, reCheckFailed);
    return SM_ERR;
#endif // USE_VOLUMES

} // end SmBSplineVolume::PassesValidityCheck

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertBSplineVolume_list[] =
{
 /*  0 */ {SM_AT_POINTER,     _T("Null"),           _T("m_pNurb is non-NULL") },
 /*  1 */ {SM_AT_NESTED_TEST, _T("Validity Tests"), _T("Volume failed PassesValidityCheck() call") }
} ;

/*******************************************************************//**
PURPOSE: Make sure m_pNurb is not NULL by calling MakeNurb() when needed.

NOTES: returns TRUE  = OK
               FALSE = Problem
***********************************************************************/
SmBoolean SmBSplineVolume::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  // check base class 
  SmBoolean bRtn = SmVolume::AssertValid(pAList, eTestLevel, eWalkTree, pTestRequests) ;

  // run SmBSplineVolume PassesValidityCheck checks
  SmValidityCheckType eCheckFailed ;
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (PassesValidityCheck(SM_VC_ALL, eCheckFailed) ), _T("") ) ;

  //      // build m_pNurb when not available
  //      if (m_pNurb == NULL) 
  //        {
  //          SmBSplineVolume *pBSV = SM_CONST_CAST(SmBSplineVolume*,this);
  //          pBSV->MakeNurb();
  //        }

  // check for nonNULL m_pNurb pointer
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pNurb != NULL ), _T("") ) ;

  // all done 
  return(bRtn) ;

} // end SmBSplineVolume::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmBSplineVolume::AssertHeal
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
//       return ( SmVolume::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmBSplineVolume::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmBSplineVolume::AssertHeal
// end obsolete
