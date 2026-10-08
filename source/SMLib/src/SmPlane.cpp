// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPlane.cpp
* PURPOSE: Implementation of SmPlane methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmPlane.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#include <SmCone.h>
#include <SmSphere.h>
#include <SmCircle.h>
#include <SmSurfOfExtrusion.h>
#include <SmMatrix.h>
#include <SmLine.h>
#include <SmSurfaceCache.h>
#include <SmNurbsSrf.h>
#include <SmGeomUtility.h>
#include <SmGlobalSolver.h>
#include <SmAssertArray.h>
#include <SmDatabaseIO.h>
#include <SmIsoCurve.h>
#include <nurbs.h>

#ifdef SM_DEBUG_CODE
 #include <SmGap.h>
#endif // SM_DEBUG_CODE


/*******************************************************************//**
PURPOSE: Create a bounded plane given (1) origin, (2) local X,Y axes,
    (3) scale of the 'unit' vectors in U & V direction (4) analytic domain.

NOTES: This plane should be used when surface operations (like
    SS-Intersection) may take advantage of its analytic properties.

    Plane = Origin + UScale * XAxis * u 
                   + VScale * YAxis * v
***********************************************************************/
SmPlane::SmPlane
  (const SmPoint3d  & crOrigin,         // in : 3Space loc of parametric origin = Plane.Evaluate(0,0)
   const SmVector3d & crXAxis,          // in : 3D X Axis - made unit. Does not affect UVScale values
   const SmVector3d & crYAxis,          // in : 3D Y Axis - made unit - should be perp to X. Does not affect UVScale values
   const SmVector2d & crUVScale,        // in : UScale and VScale
   const SmExtent2d & crAnalUVDomain,   // in : UV Domain limiting allowed evaluations
   const SmContext  * cpContext)        // in : Set context if given, default:[NULL]
 : m_vAnalUVDomain(crAnalUVDomain),
   m_vUVScale(crUVScale)
{
   SE(m_vPosition.SetCanonical(crOrigin, crXAxis, crYAxis));

  // set context if given
  if(cpContext) 
    { m_cpContext = cpContext ; }

} // end SmPlane::SmPlane constructor

/*******************************************************************//**
PURPOSE: Create an infinite plane given an origin and a normal vector.

NOTES: Plane = Origin + UScale * XAxis * u, 
                      + VScale * YAxis * v, 
       UScale = 1.0, XAxis = any vector perp to PlaneNormal 
       VScale = 1.0, YAxis = any vector perp to PlaneNormal and XAxis
       
       such that: PlaneNormal = Cross(XAxis, YAxis) ;
***********************************************************************/
SmPlane::SmPlane
  (const SmPoint3d  & crOrigin,       // in : 3d loc of parametric origin = Plane.Evaluate(0,0)
   const SmVector3d & crPlaneNormal,  // in : normal vetor to plane (does not have to be unit)
   const SmContext  * cpContext)      // in : Set context if given, default:[NULL]
 : m_vUVScale(1.0,1.0)
{
  // set m_vPosition
  SM_ASSERT(crPlaneNormal.LengthSquared() > SM_EFF_ZERO_SQ);
  SmVector3d sX, sY, sZ;
  SE(crPlaneNormal.MakeUnitOrthoVectors(NULL,sX,sY,sZ));
  SE(m_vPosition.SetCanonical(crOrigin,sY,sZ));

  // Make the plane m_vAnalUVDomain essentially infinite
  m_vAnalUVDomain.SetMinMax(SmPoint2d(-SM_INFINITE_PARAMETER,-SM_INFINITE_PARAMETER),
                            SmPoint2d(SM_INFINITE_PARAMETER,SM_INFINITE_PARAMETER));

  // set context if given
  if(cpContext) 
    { m_cpContext = cpContext ; }

} // end SmPlane::SmPlane constructor


/*******************************************************************//**
PURPOSE: Copy constructor for a SmPlane.

NOTES: 
***********************************************************************/
SmPlane::SmPlane
  (const SmPlane & crPlane)
 : SmBSplineSurface(crPlane), 
   m_vPosition(crPlane.m_vPosition),
   m_vAnalUVDomain(crPlane.m_vAnalUVDomain),
   m_vUVScale(crPlane.m_vUVScale)
{

} // end SmPlane::SmPlane constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmPlane

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmPlane::operator==
  (const SmSurface& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmBSplineSurface::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmPlane &rOther = (SmPlane &)crOther ;

      // check equivalence of these objects
      bRtn =  (   m_vPosition     == rOther.m_vPosition
               && m_vAnalUVDomain == rOther.m_vAnalUVDomain
               && m_vUVScale      == rOther.m_vUVScale     ) ;
    }

  // all done
  return bRtn ;

} // end SmPlane::operator==

/*******************************************************************//**
PURPOSE: Adjust the Domain of the plane to correspond to a new 
   UV box.  This method moves the control points and adjusts the knot
   vector such that the basic parameterization of the plane remains
   the same but the extents on it change.

NOTES: 
***********************************************************************/
SmStatus SmPlane::AdjustSTEPUVDomain
  (const SmExtent2d & crNewSTEPUVDomain)  // in : new domain to use
{
  // no work - new domain is equal to current domain
  if (   crNewSTEPUVDomain.IsContainedBy(m_vAnalUVDomain, SM_EFF_ZERO)
      && m_vAnalUVDomain.IsContainedBy(crNewSTEPUVDomain, SM_EFF_ZERO))
    { return SM_SUCCESS; }

  // save the new UVDomain
  m_vAnalUVDomain = crNewSTEPUVDomain;

  // replace the m_pNurb pointer with a new Planar Nurb with the new Extent
  SER(MakeNurb());

  // all done
  return SM_SUCCESS;

} // end SmPlane::AdjustSTEPUVDomain

/*******************************************************************//**
PURPOSE: Copy a plane.

NOTES:
***********************************************************************/
SmStatus SmPlane::Copy
(const SmContext& crContext,
    SmPlane*& rpNewPlane)
    const
{
    SmPlane* pCopy = new(crContext) SmPlane(*this);
    NER(pCopy);
    rpNewPlane = pCopy;
    return SM_SUCCESS;

} // end SmPlane::Copy

/*******************************************************************//**
PURPOSE: Copy a plane.

NOTES: 
***********************************************************************/
SmStatus SmPlane::Copy
  (const SmContext & crContext,
   SmSurface *& rpNewSurface) 
  const
{
    SmPlane *pCopy = new(crContext) SmPlane(*this);
    NER(pCopy);
    rpNewSurface = pCopy;
    return SM_SUCCESS;

} // end SmPlane::Copy
      
/*******************************************************************//**
PURPOSE: Method to create a canonical plane object with an infinite domain

NOTES: crOrigin is copied not saved into the new SmPlane object
***********************************************************************/
SmStatus SmPlane::CreateCanonical
  (const SmContext        & crContext,  // in : context for new object construction
   const SmAxis2Placement & crOrigin,   // in : position and orientation of new plane
   SmPlane               *& rpNewPlane) // out: the new plane 
{
  SmVector2d sUVScale(1.0,1.0);
  SmExtent2d sUVDomain(SmPoint2d(-SM_INFINITE_PARAMETER,-SM_INFINITE_PARAMETER),
                       SmPoint2d( SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER));
  rpNewPlane = new (crContext) SmPlane(crOrigin.GetOriginRef(),
                                       crOrigin.GetXAxisRef(),
                                       crOrigin.GetYAxisRef(),
                                       sUVScale,
                                       sUVDomain);

  NER(rpNewPlane);

  SER(rpNewPlane->MakeNurb());
  return SM_SUCCESS;

} // end SmPlane::CreateCanonical

/*******************************************************************//**
PURPOSE: Method to get canonical data from plane object.

NOTES: 
***********************************************************************/
SmStatus SmPlane::GetCanonical
  (SmAxis2Placement & rOrigin)
 const
{
    rOrigin = m_vPosition;

    return SM_SUCCESS;

} // end SmPlane::GetCanonical


/*******************************************************************//**
PURPOSE: Create an offset surface for the plane.

NOTES: Offset Surface domains == Input Surface domains
***********************************************************************/
SmStatus SmPlane::CreateOffsetSurface
(
  const SmContext      & crContext,             // in : context for new obj construction
  double                 dSignedOffsetDistance, // in : offset dist, (neg val = Offset dir opposite surface normal)
  SmApproxTol3d          dThisApproxTol3d,      // NotUsed: in : Max Dist between ApproxOffsetSurface and ideal offset shape
  SmSurface* &           rOffsetSurface         // out: Offset Surf Approx, may be more than 1 when offsets have self-intersections
) const
{
  SM_REF1(dThisApproxTol3d) ; 
  // construct plane copy
  SmPlane *pOffPlane = new(crContext) SmPlane(*this);

  // translate the plane copy
  SmAxis2Placement sTrans;
  SmVector3d sMove = m_vPosition.GetZAxis() * dSignedOffsetDistance;
  sTrans.Translate(sMove);
  SER(pOffPlane->Transform(sTrans,NULL));

  // set output
  rOffsetSurface = pOffPlane;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pOffPlane) pOffPlane->DrawUV() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmPlane::CreateOffsetSurface

/*******************************************************************//**
PURPOSE: Check to see whether two surfaces are coincident, either over
   their entire domains or over any subset, depending on the input flag.

NOTES: Returns an error when either input surface is degenerate.

   The output argument rpContainingSurface indicates which surface could be
   expanded to cover the other.  SmPlanes can always be expanded easily,
   so we always return 'this'.

***********************************************************************/
SmStatus SmPlane::CoincidenceCheck
  (const SmSurface & crOther,               // in : target surface to compare
   double            d3DTolerance,          // in : max allowed distance between coincident surfaces
   SmBoolean       & rbAreCoincident,       // out: TRUE = surfaces are the same to within tolerance
   double          & rdMaxDistanceBetween,  // out: max distance between surfaces
   SmBoolean         bWholeSurfaceCoinc,    // in : TRUE = Test whether surfaces are totally coincident
                                            //      FALSE= Test whether there is any coincident overlap
   SmSurface      *& rpContainingSurface)   // out: See Notes above.
  const
{
  // init output
  rbAreCoincident = FALSE;
  rdMaxDistanceBetween = -1.0;
  rpContainingSurface  = SM_CAST_NONNULL_PTR( SmSurface, SM_CONST_CAST( SmPlane*, this ));  // See Notes above.

  // low work - surfaces are the same
  if ( &crOther == this )
    {
      rbAreCoincident     = TRUE;
      rdMaxDistanceBetween= 0;
      rpContainingSurface = (SmSurface *)this ;
      return SM_SUCCESS;
    }

  // Quick check: other surface would have to be planar.
  if ( ! crOther.IsPlanar( d3DTolerance ) )
    { return SM_SUCCESS; }

  // locals
  ULONG ii, jj, kk;
  double dDist, dDistSq = 0.0, dMaxDist = -1.0;
  SmBoolean bIsMulti;
  SmPoint2d sUV0, sUV1;
  SmPoint3d sPt0, sPt1;

  SmExtent2d sDomain0 = this->GetNaturalUVDomain();
  SmExtent2d sDomain1 = crOther.GetNaturalUVDomain();

  const SmPoint3d & rPlanePnt0  = m_vPosition.GetOriginRef();
  SmVector3d        sPlaneNorm0 = m_vPosition.GetZAxis();

  // We check angles here: find an angle tolerance that makes sense.
  // That would be the angle that would make a plane this size deviate
  // by the given tol.
  SmVector2d sSize;
  sSize.x = m_vAnalUVDomain.GetUInterval().GetLength() * m_vUVScale.x;
  sSize.y = m_vAnalUVDomain.GetVInterval().GetLength() * m_vUVScale.y;
  // Actually, atan(), but they're pretty much the same for small angles:
  double dAngTolDeg = SM_RAD2DEG( d3DTolerance / sSize.Length() );

  SmBoolean bSuccess;
  SmStatus eStat;


  // Special for planes: rule out cases where infinite planes don't coincide.
  SmPlane *pOtherPlane = SM_CAST_NONNULL_PTR( SmPlane, &crOther );

  if ( pOtherPlane != NULL )
    {
      const SmAxis2Placement & rOtherPos = pOtherPlane->GetPosition();

      // Check normals.
      SmVector3d sOtherNorm = rOtherPos.GetZAxis();
      if ( ! sOtherNorm.IsParallelTo( sPlaneNorm0, dAngTolDeg ) )
        { return SM_SUCCESS; }

      // Check positions.
      eStat = smgu_PlanePointDistance( rPlanePnt0, sPlaneNorm0, rOtherPos.GetOriginRef(), dDist );
      if ( dDist > d3DTolerance )
        { return SM_SUCCESS; }

    } // end ruling out non-coincident plane/plane case.


  if ( bWholeSurfaceCoinc )
    {
      rbAreCoincident = FALSE;

      // First, all four corner points must match.
      SmTArray< SmPoint3d > sCorners0;
      SmTArray< SmPoint3d > sCorners1;
      for ( ii=0; ii<2; ii++ )
        {
          for ( jj=0; jj<2; jj++ )
            {
              this->EvaluatePoint( sDomain0.Evaluate( ii, jj ), sPt0 );
              sCorners0.Add( sPt0 );
              crOther.EvaluatePoint( sDomain1.Evaluate( ii, jj ), sPt1 );
              sCorners1.Add( sPt1 );
            }
        } // end collecting corner points

      // All four points of other must match one of ours.
      double dTolSq = d3DTolerance*d3DTolerance;
      for ( ii=0; ii<4; ii++ )  // for each this-plane corner point
        {
          for ( jj=0; jj<4; jj++ )  // for each crOther corner point
            {
              dDistSq = sCorners0[ii].DistanceBetweenSquared( sCorners1[jj] );
              if ( dDistSq <= dTolSq )
                { break; }  // Found a match.
            }

          if ( dDistSq > dTolSq )
            { break; } // this-plane point did not find a match.
        }

      if ( dDistSq > dTolSq )
        { return SM_SUCCESS; } // Some corner did not match.

      // Arrive here when all four corners matched.

      // If the other surface is a plane as well, we're done.
      // Actually, bilinear is sufficient, after matching four distinct points.
      if ( crOther.IsBilinear() )
        {
          rdMaxDistanceBetween = smos_Sqrt( dDistSq );
          rbAreCoincident = TRUE;
          return SM_SUCCESS;
        }

      // Match some points.  Midpoint and four around, not exact.
      SmTArray< SmPoint2d > sUVs;
      sUV1.Set( .51, .48 ); sUVs.Add( sUV1 );
      sUV1.Set( .27, .23 ); sUVs.Add( sUV1 );
      sUV1.Set( .27, .77 ); sUVs.Add( sUV1 );
      sUV1.Set( .72, .77 ); sUVs.Add( sUV1 );
      sUV1.Set( .72, .23 ); sUVs.Add( sUV1 );

      ULONG lNumSamples = sUVs.GetSize();

      SmVector3d sNorm1;

      for ( ii=0; ii<lNumSamples; ii++ )
        {
          sUV1 = sUVs[ii];
          SER( crOther.EvaluatePoint(sUV1, sPt1 ));
          SER( smgu_PlanePointDistance( rPlanePnt0, sPlaneNorm0, sPt1, dDist ));

          if ( dDist > dMaxDist )
            { rdMaxDistanceBetween = dMaxDist = dDist; }

          if ( dDist > d3DTolerance )
            { return SM_SUCCESS; }

          // Also check normals.
          SER( crOther.EvaluateNormal(sUV1, TRUE, TRUE, sNorm1 ));
          if ( ! sNorm1.IsParallelTo( sPlaneNorm0, dAngTolDeg ) )
            { return SM_SUCCESS; }
        }

      // Still here -- must be coincident.
      rbAreCoincident = TRUE;
      return SM_SUCCESS;

  } // end if checking whole-surface coincidence.


  // Now, check for any coincident overlap.

  // Plane/Plane case.
  if ( pOtherPlane != NULL )
    {
      // At this point we already know we have two coplanar rectangles,
      // not necessarily axis-aligned.
      // Just have to see whether they overlap.

      // Drop other corners to this plane.
      // If any corner lands inside this plane's domain, we're done.
      // Otherwise, intersect each side of the other (projected into
      // our domain) with our domain.  If there are any intersections,
      // then there is overlap -- except for the case of two rectangles
      // that just touch but don't overlap.  Rule out that case first,
      // by checkng bounding boxes.

      SmTArray< SmPoint2d > sOtherUVs;
      SmExtent2d sOtherExtent;

      for(ii=0; ii<2; ii++) 
        {
          for(jj=0; jj<2; jj++) 
            {
              // plane1 corners to plane2 distances
              if ( ii==0 )
                { sUV1 = sDomain1.Evaluate(ii, jj); }
              else
                { sUV1 = sDomain1.Evaluate(ii, 1-jj); } // To go around in order.
              SER( pOtherPlane->EvaluatePointFast( sUV1, sPt1 ));
              SER( this->ProjectPointToUVDomain  ( sPt1, sUV0 ));

              // First check max distance, for the output value.
              SER( this->EvaluatePointFast( sUV0, sPt0 ));
              dDist = sPt1.DistanceBetween( sPt0 );

              if ( dDist > rdMaxDistanceBetween )
                { rdMaxDistanceBetween = dDist; }

              if ( dDist > d3DTolerance )  // (Shouldn't happen.)
                { rbAreCoincident = FALSE; return SM_SUCCESS; }

              // Now check whether it's internal; if so, we're done.
              // But not on boundary: pass negative tol.
              if ( sDomain0.ContainsPoint2d( sUV0, -SM_EFF_ZERO_SQRT ) )
                { rbAreCoincident = TRUE; return SM_SUCCESS; }

              sOtherUVs.Add( sUV0 );
              sOtherExtent.AddPoint2d( sUV0 );
            }
        } // end iter other Plane corners

      // Now do the bounding box check.
      // Again, negative tol: don't catch just-touching cases.
      if ( sOtherExtent.AreDisjoint( sDomain0, -SM_EFF_ZERO_SQRT ) )
        { rbAreCoincident = FALSE; return SM_SUCCESS; }

      // No corners landed inside our domain, but the bounding boxes overlap.
      // Intersect the four sides of the other with our domain.
      sOtherUVs.Add( sOtherUVs[0] ); // to make the logic easier...
      SmExtent1d sIntIvl;
      SmExtent1d sUnitIvl( 0, 1 );
      for ( ii=1; ii<sOtherUVs.GetSize(); ii++ )
        {
          SmVector2d sVec( sOtherUVs[ii] - sOtherUVs[ii-1] );
          sDomain0.IntersectWithInfiniteLine( sOtherUVs[ii-1], sVec, bSuccess, sIntIvl );

          if ( bSuccess )
            {
              // Intersect it with the unit interval: that's what corresponds to
              // the actual segment between the corner points.
              // (I.e., we don't really want infinite.)
              eStat = sUnitIvl.Intersect( sIntIvl, sIntIvl );
              if ( eStat == SM_SUCCESS && sIntIvl.GetLength() >= d3DTolerance )
                { rbAreCoincident = TRUE; return SM_SUCCESS; }
            }
        }

      // None of other's edges enters our domain.
      // Two possibilities left: either they're disjoint,
      // or this plane is totally contained in the other.
      // Test our midpoint.
      sUV0 = sDomain0.Evaluate( 0.5, 0.5 );
      this->EvaluatePoint( sUV0, sPt0 );
      SER( pOtherPlane->ProjectPointToUVDomain( sPt0, sUV1 ));

      // Also check max distance, for the output value.
      SER( pOtherPlane->EvaluatePointFast( sUV1, sPt1 ));
      dDist = sPt1.DistanceBetween( sPt0 );

      if ( dDist > rdMaxDistanceBetween )
        { rdMaxDistanceBetween = dDist; }

      if ( dDist > d3DTolerance )  // (Shouldn't happen.)
        { rbAreCoincident = FALSE; return SM_SUCCESS; }

      // Now check internal; if so, we're done.  But not on boundary: negative tol.
      if ( sDomain1.ContainsPoint2d( sUV1, -SM_EFF_ZERO_SQRT ) )
        { rbAreCoincident = TRUE; return SM_SUCCESS; }


      // Didn't find any overlap.
      rbAreCoincident = FALSE;
      return SM_SUCCESS;

    } // end if other is also an SmPlane.



  // Checking partial overlap, and the other surface is not an SmPlane.
  // Method: if there is coincident overlap, then some edge of one
  // has to lie in the other.

  // Create two sets of boundary curves.
  SmTArray<SmCurve*>        sBdrys3D0, sBdrys3D1;
  SmTArray<SmBSplineCurve*> sBdrysUV0, sBdrysUV1;
  SmTArray<SmOrientType>    sOrients0, sOrients1;

  SmObjsDelete<SmCurve*>        sClean3D0( &sBdrys3D0 );
  SmObjsDelete<SmCurve*>        sClean3D1( &sBdrys3D1 );
  SmObjsDelete<SmBSplineCurve*> sCleanUV0( &sBdrysUV0 );
  SmObjsDelete<SmBSplineCurve*> sCleanUV1( &sBdrysUV1 );

  this->CreateNaturalUVTrimCurves( *(this->GetContext()), sDomain0, SM_SP_V, FALSE,
      sBdrys3D0, sBdrysUV0, sOrients0 );  
  ULONG lNumBdrys0 = sBdrys3D0.GetSize(); // Presumably 4.

  crOther.CreateNaturalUVTrimCurves( *(crOther.GetContext()), sDomain1, SM_SP_V, FALSE,
      sBdrys3D1, sBdrysUV1, sOrients1 );  
  ULONG lNumBdrys1 = sBdrys3D1.GetSize();

  double dT0, dT1;

  // For evaluation:
  SmVector3d sSu0, sSv0, sSu1, sSv1;
  SmVector3d sSuu1, sSuv1, sSvv1;
  SmVector3d s3dBinorm0, s3dBinorm1;
  SmVector2d sUVBinorm0, sUVBinorm1;

  // We'll be comparing surface normals.
  SmVector3d sNorm0, sNorm1;

  SmCurve *pCrv0, *pCrv1;
  SmExtent1d sCrvDom0, sCrvDom1;

  SmSolutionArray sSols;

  // We'll be checking normals, and this plane's normal never changes.
  sNorm0 = this->m_vPosition.GetZAxis();

  // Also, our first derivatives never change.
  sSu0 = m_vPosition.GetXAxisRef() * m_vUVScale.x;
  sSv0 = m_vPosition.GetYAxisRef() * m_vUVScale.y;

  // We check for degenerate curves in the loops.
  // For the inner loop, that means four times for each curve.
  // That is a moderately expensive call, so let's do it
  // once for each curve in the inner loop and save the results.
  SmBoolean bData[4];
  SmTArray< SmBoolean > sCrv1Degen(4,bData,4);
  for ( jj=0; jj<lNumBdrys1; jj++ )
    {
      pCrv1 = sBdrys3D1[jj];
      sCrv1Degen[jj] = pCrv1->IsDegenerate();
    }


  // Set this True as default, so that we can just return if found.
  rbAreCoincident = TRUE;

  for ( ii=0; ii<lNumBdrys0; ii++ )
    {
      pCrv0 = sBdrys3D0[ii];
      if ( pCrv0 == NULL || pCrv0->IsDegenerate() ) { continue; }

      sCrvDom0 = ( ii==0 || ii==2 ) ? sDomain0.GetUInterval() : sDomain0.GetVInterval();

      for ( jj=0; jj<lNumBdrys1; jj++ )
        {
          pCrv1 = sBdrys3D1[jj];
          if ( pCrv1 == NULL || sCrv1Degen[jj] ) { continue; }

          sCrvDom1 = ( jj==0 || jj==2 ) ? sDomain1.GetUInterval() : sDomain1.GetVInterval();

          // Check for an intersection.
          eStat = pCrv0->GlobalCurveIntersect( sCrvDom0, *pCrv1, sCrvDom1, d3DTolerance, sSols );

          ULONG lNumSols = sSols.GetSize();
          if ( eStat != SM_SUCCESS || lNumSols == 0 )
            { continue; }

          for ( kk=0; kk<lNumSols; kk++ )
            {
              SmSolution & rSol = sSols[kk];

              // Note, for range sols we might think to use the midpoints.
              // However, the parameterizations of the curves might well be
              // different enough to make that invalid.  We could take the
              // midpoint of one and drop to the other ... but without loss
              // of generality, it's fine to just use the start or end.  So
              // just use an end point.
              dT0 = rSol.m_vStart[0];
              dT1 = rSol.m_vStart[1];

              // Get the surface uv's from the edge curve params.
              // (We can assume that the isocurves are parameterized the same as the surface.)
              // Also, while we're doing these switches, set sUVBinorm0/1
              // for possible later use.
              switch ( ii ) {
                case 0: { sUV0.Set( dT0, sDomain0.GetVMin() );  sUVBinorm0.Set(  0,  1 ); break; }
                case 1: { sUV0.Set( sDomain0.GetUMax(), dT0 );  sUVBinorm0.Set( -1,  0 ); break; }
                case 2: { sUV0.Set( dT0, sDomain0.GetVMax() );  sUVBinorm0.Set(  0, -1 ); break; }
                case 3: { sUV0.Set( sDomain0.GetUMin(), dT0 );  sUVBinorm0.Set(  1,  0 ); break; }
              }
              switch ( jj ) {
                case 0: { sUV1.Set( dT1, sDomain1.GetVMin() );  sUVBinorm1.Set(  0,  1 ); break; }
                case 1: { sUV1.Set( sDomain1.GetUMax(), dT1 );  sUVBinorm1.Set( -1,  0 ); break; }
                case 2: { sUV1.Set( dT1, sDomain1.GetVMax() );  sUVBinorm1.Set(  0, -1 ); break; }
                case 3: { sUV1.Set( sDomain1.GetUMin(), dT1 );  sUVBinorm1.Set(  1,  0 ); break; }
              }
              crOther.EvaluateNormal( sUV1, TRUE, TRUE, sNorm1 );

              if ( ! sNorm1.IsParallelTo( sNorm0, dAngTolDeg ) )
                { continue; }

              // Have to check 2nd order terms: cyl with seam in plane.
              crOther.Evaluate2ndDerivatives( sUV1, TRUE, TRUE,
                          sPt1, sSu1, sSv1, sSuv1 , sSuu1, sSvv1);

              // Basically, if any of the other surface's derivatives
              // have components out of the plane, it can't be coincident.

              if ( smos_Fabs( sNorm0.Dot( sSu1  ) ) > dAngTolDeg )
                { continue; }
              if ( smos_Fabs( sNorm0.Dot( sSv1  ) ) > dAngTolDeg )
                { continue; }
              if ( smos_Fabs( sNorm0.Dot( sSuu1 ) ) > dAngTolDeg )
                { continue; }
              if ( smos_Fabs( sNorm0.Dot( sSuv1 ) ) > dAngTolDeg )
                { continue; }
              if ( smos_Fabs( sNorm0.Dot( sSvv1 ) ) > dAngTolDeg )
                { continue; }

              if ( rSol.m_eSolutionType == SM_ST_SINGLE_VALUE )
                {
                  // If this hits the interior of both boundaries
                  if (   ! sCrvDom0.IsValueOnBoundary( dT0 )
                      && ! sCrvDom1.IsValueOnBoundary( dT1 ) )
                    {  return SM_SUCCESS;  }

                } // end if single-value solution
              else
                {
                  // Range solution: edge-to-edge.
                  // Need to check binormals.
                  s3dBinorm0 = sUVBinorm0.x * sSu0  +  sUVBinorm0.y * sSv0;

                  crOther.Evaluate1stDerivatives( sUV1, TRUE, TRUE,
                          sPt1, sSu1, sSv1 );
                  s3dBinorm1 = sUVBinorm1.x * sSu1  +  sUVBinorm1.y * sSv1;

                  if ( s3dBinorm0.Dot( s3dBinorm1 ) > 0.0 )
                    {
                      return SM_SUCCESS;
                    }
                }
            } // end loop (kk) on all curve/curve intersection solutions
        } // end inner loop (jj) on other surface's boundary curves
    } // end outer loop (ii) on our boundary curves

  // Ok, no edge/edge intersections.
  // If there is coincidence, then one surface completely contains the other.
  // Drop any point from each to the other.
  // Drop to this first because planes are easy.
  sUV1 = sDomain1.GetMid();
  crOther.EvaluatePoint( sUV1, sPt1 );
  eStat = this->DropPoint(sPt1, 
                          sDomain0, 
                          NULL,                                                     
                          bSuccess, 
                          sUV0, 
                          dDist,
                          bIsMulti,
                          SM_SO_NORMALIZE) ;

  // Also set output argument:
  if ( dDist > rdMaxDistanceBetween )
    { rdMaxDistanceBetween = dDist; }

  if ( eStat == SM_SUCCESS && bSuccess && dDist <= d3DTolerance )
    {
      // Point coincidence, check normals.

      // Have to check 2nd order terms, not just normals: cyl with seam in plane.
      crOther.Evaluate2ndDerivatives( sUV1, TRUE, TRUE,
                          sPt1, sSu1, sSv1, sSuv1 , sSuu1, sSvv1);

      // Basically, if any of the other surface's derivatives
      // have components out of the plane, it can't be coincident.

      if ( smos_Fabs( sNorm0.Dot( sSu1  ) ) < dAngTolDeg
        && smos_Fabs( sNorm0.Dot( sSv1  ) ) < dAngTolDeg
        && smos_Fabs( sNorm0.Dot( sSuu1 ) ) < dAngTolDeg
        && smos_Fabs( sNorm0.Dot( sSuv1 ) ) < dAngTolDeg
        && smos_Fabs( sNorm0.Dot( sSvv1 ) ) < dAngTolDeg )
        { return SM_SUCCESS; }
    }

  // Repeat the other way around.
  sUV0 = sDomain0.GetMid();
  this->EvaluatePoint( sUV0, sPt0 );
  eStat = crOther.DropPoint(sPt0, 
                            sDomain1, 
                            NULL, 
                            bSuccess, 
                            sUV1, 
                            dDist,
                            bIsMulti,
                            SM_SO_NORMALIZE) ;

  // Also set output argument:
  if ( dDist > rdMaxDistanceBetween )
    { rdMaxDistanceBetween = dDist; }

  if ( eStat == SM_SUCCESS && bSuccess && dDist <= d3DTolerance )
    {
      // Point coincidence, check normals.

      // Have to check 2nd order terms, not just normals: cyl with seam in plane.
      crOther.Evaluate2ndDerivatives( sUV1, TRUE, TRUE,
                          sPt1, sSu1, sSv1, sSuv1 , sSuu1, sSvv1);

      if ( smos_Fabs( sNorm0.Dot( sSu1  ) ) < dAngTolDeg
        && smos_Fabs( sNorm0.Dot( sSv1  ) ) < dAngTolDeg
        && smos_Fabs( sNorm0.Dot( sSuu1 ) ) < dAngTolDeg
        && smos_Fabs( sNorm0.Dot( sSuv1 ) ) < dAngTolDeg
        && smos_Fabs( sNorm0.Dot( sSvv1 ) ) < dAngTolDeg )
        { return SM_SUCCESS; }
    }


  // If we arrive here, we haven't found coincidence.
  rbAreCoincident = FALSE;

  return SM_SUCCESS;

} // end SmPlane::CoincidenceCheck

/*******************************************************************//**
PURPOSE: Create an 3D iso-parametric curve of an analytic surface given 
    the Nurb parameter direction (U or V) and the constant parameter 
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
SmStatus SmPlane::CreateIsoParametricCurve
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
  // init output
  rpNewIsoCurve = NULL ;
  if(pOptMaxGap3d) { *pOptMaxGap3d = 0.0 ; }

  // get natural or optional domain min/max points
  SmExtent2d sUVDomain = GetNaturalUVDomain();
  SmPoint2d  sPMin     = sUVDomain.GetMin();
  SmPoint2d  sPMax     = sUVDomain.GetMax();
  if (pOptDomain) 
    {
      sPMin = pOptDomain->GetMin();
      sPMax = pOptDomain->GetMax();
    }

  // when asked - build associated 2d UVTrimCurve line (different parameterization)
  if(pOptUVIsoCurve) 
    { *pOptUVIsoCurve = NULL ; 

      // define the isoParameter Curve as an object
      SmIsoCurve sIso(*this,eSurfParam,dIsoParameter,TRUE);
      sIso.SetContext(NULL);

      // build OptUVIsoCurve
      sIso.CreateUVIsoLine(*pOptUVIsoCurve, &crContext) ; 
    } 

  // move min/max points to isoParameter Line endPoints
  SmExtent1d sLineIvl;
  if (eSurfParam == SM_SP_U ) { sPMin.x = dIsoParameter;
                                sPMax.x = dIsoParameter;
                                sLineIvl.SetMinMax(sPMin.y,sPMax.y);
                              }
  else                        { sPMin.y = dIsoParameter;
                                sPMax.y = dIsoParameter;
                                sLineIvl.SetMinMax(sPMin.x,sPMax.x);
                              }

  // evaluate Line 3D endPoints
  SmPoint3d sStPt, sEndPt;
  SER(EvaluatePointFast(sPMin,sStPt));
  SER(EvaluatePointFast(sPMax,sEndPt));

  // create the line
  SmLine *pLine;
  SER(SmLine::CreateLineSegment(crContext,3,sStPt,sEndPt,pLine));
  NER(pLine);
  SER(pLine->EditParameterization(sLineIvl));

  // set output
  rpNewIsoCurve = pLine;
  return SM_SUCCESS;

} // end SmPlane::CreateIsoParametricCurve

/*******************************************************************//**
PURPOSE: Create a ray from the start point to the given domain boundary
    along a constant parametric direction.

NOTES: 
***********************************************************************/
SmStatus SmPlane::CreateSurfaceRay
  (const SmContext  & crContext,                    // in : Context for new objects
   const SmExtent2d & crPlaneUVDomain,              // in : Surface domain to consider
   const SmPoint2d  & crStartPoint,                 // in : Ray Start Point on Surface
   SmBoolean          bCurveInParameterSpace,       // in : TRUE  = build 2d Curve, 
                                                    //      FALSE = build 3d curve
   SmBoolean          bTowardUpperBoundaryOfDomain, // in : TRUE  = direct ray up
                                                    //      FALSE = direct ray down
   SmSurfParamType    eSurfParam,                   // in : parameter held constant
   SmApproxTol3d     /*d3DTolerance*/,              // in : Not used for planes
   SmBSplineCurve   *& rpRayCurve)                  // out: Ray
   const
{
  // set ray start/end points for SM_SP_U and SM_SP_V directions
  SmPoint2d sUVStart(crStartPoint);
  SmPoint2d sUVEnd(eSurfParam == SM_SP_U ? crStartPoint.x
                                         : (  bTowardUpperBoundaryOfDomain
                                            ? crPlaneUVDomain.GetMax().x
                                            : crPlaneUVDomain.GetMin().x),
                   eSurfParam == SM_SP_V ? crStartPoint.y
                                         : (  bTowardUpperBoundaryOfDomain
                                            ? crPlaneUVDomain.GetMax().y
                                            : crPlaneUVDomain.GetMin().y)) ;

  // locals
  SmPoint3d sStartPt, sEndPt;
  ULONG lDim;

  // set curve startPoint, stopPoint, and dimension (2d or 3d)
  if (bCurveInParameterSpace) 
    {
      lDim = 2;
      sStartPt.x = sUVStart.x;
      sStartPt.y = sUVStart.y;
      sStartPt.z = 0.0;
      sEndPt.x = sUVEnd.x;
      sEndPt.y = sUVEnd.y;
      sEndPt.z = 0.0;
    }
  else // 3d space - project UVPoints to XYZPoints
    {
      lDim = 3;
      SER(EvaluatePointFast(sUVStart,sStartPt));
      SER(EvaluatePointFast(sUVEnd,  sEndPt));
    }

  // convert start/stop points into unitized direction vector
  SmVector3d sPlaneVec   = sEndPt - sStartPt;
  double     dScale = sPlaneVec.Length();
  SER(sPlaneVec.Unitize());

  // build an SmLine to represent the ray
  SmExtent1d sIvl(0.0,1.0);
  SmLine *pLine = new (crContext) SmLine(sStartPt,sPlaneVec,sIvl,dScale,lDim);
  NER(pLine);

  // all done - set output and return
  rpRayCurve = pLine;
  return SM_SUCCESS;

} // end SmPlane::CreateSurfaceRay

/*******************************************************************//**
PURPOSE: Virtual method to drop a curve to a plane.

NOTES: dropCurve is trimmed to the portion of the curve
  that drops within the given crPlaneUVDomain of interest.
***********************************************************************/
SmStatus SmPlane::DropCurve
 (const SmContext           & crContext,            // in : context for new object construction
  const SmExtent2d          & crUVDomain,           // in : domain of interest for this surface
  const SmCurve             & cr3dCurve,            // in : Curve to project onto the surface
  const SmExtent1d          & crInterval,           // in : interval of interest for target curve
  SmApproxTol3d               dApproxTol,           // in : dApproxTol = max allowed distance between drop point and m_SrfNormal line at drop point
  double                    & rdMaxDropToSurf,      // out: MaxDist(DropPt, 3dCurvePt), found by sampling, may be slightly less than actual max.
  double                    & rdMaxApproxDev,       // out: MaxDist(DropPt->SrfNormalLine, 3dCurvePt), Quality Measure: expected to be close to 0.0
  SmTArray<SmBSplineCurve*> & rUVCurves,            // out: 1 (or 2) curves constructed by projection.
                                                    //      (2 curves for closed surfaces when cr3dCurve is coincident with seam)
  SmBoolean                   bKeepAllDropCurves,   // in : TRUE = return all drop curves that stay over the surface (those that wander off do not drop), 
                                                    //      FALSE= only return drop curves with drop distances less than sApproxTol3d
                                                    //      default:[TRUE]
  SmDropCurveFail           * pOptDropCurveFail,    // NotUsed: out: Optional data container of a DropCurve fail or success
                                                    //      NUll to ignore, default:[NULL]
  SmBoolean                   bCreatingUVTrimCurve) // in : TRUE when creating a UVTrimCurve. This forces the UVTrimcurve parameterization to match crInterval
 const
{
  SM_REF1(pOptDropCurveFail) ; 
  // init output
  rdMaxDropToSurf = 0.0;
  rdMaxApproxDev  = 0.0; 
  rUVCurves.ReSet();

  // locals
  SmPoint3d   sLinePoint, sLineVec ;
  SmBoolean   bIsLine =   (SM_CAST_NONNULL_PTR(SmLine, &cr3dCurve) != NULL)
                        ||(cr3dCurve.IsLine(5, SM_EFF_ZERO, sLinePoint, sLineVec)) ;
  SmExtent1d  sIvl(crInterval) ;


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;
      SM_DUMP_AND_ASSERT_VALID(&cr3dCurve) ;

      SmFace *pFace = (SmFace *)GetFace() ;
      SmEdge *pEdge = (SmEdge *)cr3dCurve.GetEdge() ;
      SmBrep *pBrep =   pFace ? pFace->GetBrep()  
                      : pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      smgfx_SetLook(3,4, 1,0,0) ; cr3dCurve.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook( 5, 6, 1, 0, 1 ); if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Check the fairly trivial case of projecting a line to a plane.
  if (bIsLine) 
    {
      // get 3d line endPts
      SmPoint3d sStPnt3d, sEndPnt3d;
      SER(cr3dCurve.EvaluatePoint(sIvl.GetMin(),sStPnt3d));
      SER(cr3dCurve.EvaluatePoint(sIvl.GetMax(),sEndPnt3d));

      // Get Line 2d endPt projections to infinite plane
      SmPoint2d sStPnt2d,sEndPnt2d ;
      SER(ProjectPointToUVDomain(sStPnt3d,sStPnt2d));
      SER(ProjectPointToUVDomain(sEndPnt3d,sEndPnt2d));
      SmPoint2d sVec2d = sEndPnt2d - sStPnt2d ;

      // trim drop line to plane domain
      SmBoolean  bFoundIvl = FALSE ;
      SmExtent1d sTrimIvl ;
      double dTolU = dApproxTol / m_vUVScale.x; // 3d tol to 2d
      double dTolV = dApproxTol / m_vUVScale.y;
      crUVDomain.IntersectWithInfiniteLine(sStPnt2d,
                                           sVec2d,
                                           bFoundIvl,
                                           sTrimIvl,
                                           &dTolU, &dTolV);

      // union the trimIvl with the line ivl
      SmExtent1d sLineIvl(0.0,1.0), sFinalIvl(sIvl) ;
      sTrimIvl.Intersect(sLineIvl, sTrimIvl) ;

      // no DropCurve without UVDomain intersection
      if(   bFoundIvl == FALSE
         || sTrimIvl.IsInit())
        { return(SM_SUCCESS) ; }


      // adjust for partial drops
      if(!sTrimIvl.AreEqual(0.0, 1.0, SM_EFF_ZERO))
        {
          // reevaluate srf[UVPts]
          sEndPnt2d = sStPnt2d + sTrimIvl.GetMax() * sVec2d ; // order dependent code
          sStPnt2d  = sStPnt2d + sTrimIvl.GetMin() * sVec2d ;

          if ( !bCreatingUVTrimCurve )
          {
              // reevaluate crv[ivl]
              sFinalIvl.SetMinMax( sFinalIvl.Evaluate( sTrimIvl.GetMin() ),
                                   sFinalIvl.Evaluate( sTrimIvl.GetMax() ) );

              // reevaluate crv[endpts]
              SER( cr3dCurve.EvaluatePoint( sFinalIvl.GetMin(), sStPnt3d ) );
              SER( cr3dCurve.EvaluatePoint( sFinalIvl.GetMax(), sEndPnt3d ) );
          }
        }

      //      // clamp to domain  GWC:replaced this with the above trim curve logic
      //      sStPnt2d  = crPlaneUVDomain.ClampPoint2d(sStPnt2d);
      //      sEndPnt2d = crPlaneUVDomain.ClampPoint2d(sEndPnt2d);

      // get srf[endPts]
      SmPoint3d sNewSt3d, sNewEnd3d;
      SER(EvaluatePointFast(sStPnt2d,sNewSt3d));
      SER(EvaluatePointFast(sEndPnt2d,sNewEnd3d));

      // Max drop distance to surface
      double dDist1   = sNewSt3d.DistanceBetween(sStPnt3d);
      double dDist2   = sNewEnd3d.DistanceBetween(sEndPnt3d);
      rdMaxDropToSurf = smos_Max(dDist1,dDist2);

      // when the drop curve is good enough - save it to the output
      if(   bKeepAllDropCurves || rdMaxDropToSurf <= dApproxTol)
        {

          // build dropCurve as a UVline
          SmVector2d sVector2d = sEndPnt2d-sStPnt2d;
          double     dScale = sVector2d.Length();

          // skip degenerate solutions
          if (dScale > SM_EFF_ZERO) 
            {
              // compose the UVTrimCurve - same cr3dCurve parameterization
              SER( sVector2d.Unitize());
              SmExtent1d sLineIntervl(0.0,1.0);
              SmLine *pLine2d = new(crContext) SmLine(sStPnt2d, sVector2d, sLineIntervl,dScale,2);
              NER(pLine2d);
              SER(pLine2d->EditParameterization(sFinalIvl));

#ifdef SM_DEBUG_CODE
              // draw 
              if(bDebugMe)
                {
                  SM_DUMP_AND_ASSERT_VALID(pLine2d) ;

                  SmFace *pFace = (SmFace *)GetFace() ;
                  SmEdge *pEdge = (SmEdge *)cr3dCurve.GetEdge() ;
                  SmBrep *pBrep =   pFace ? pFace->GetBrep() 
                                  : pEdge ? pEdge->GetBrep() : NULL ;

                  smgfx_Erase() ;
                  smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
                  smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV() ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 1,0,0) ; this->DrawInspectUVTrimCurve(*pLine2d, cr3dCurve) ; sm_GraphicsLoop() ;
                  smgfx_SetLook(3,4, 1,.5,0) ; cr3dCurve.DrawSpeed() ; sm_GraphicsLoop() ;
                  smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
                  smgfx_SetLook(3,4, 1,0,0) ; cr3dCurve.Draw() ; sm_GraphicsLoop() ;
                  smgfx_SetLook( 5, 6, 1, 0, 1 ); if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
                  sm_GraphicsLoop() ;
                
                  // and check some equivalent evaluations
                  SmPoint3d sUVTestSt, sUVTestEnd;
                  pLine2d->EvaluatePoint(sFinalIvl.GetMin(),sUVTestSt);
                  pLine2d->EvaluatePoint(sFinalIvl.GetMax(),sUVTestEnd);
                  SmPoint2d sUVTSt2d(sUVTestSt.x,sUVTestSt.y);
                  SmPoint2d sUVTEnd2d(sUVTestEnd.x,sUVTestEnd.y);

                  if(   bKeepAllDropCurves == FALSE 
                     && sUVTSt2d.DistanceBetween(sStPnt2d) > dApproxTol) 
                    { SM_ASSERT( FALSE ); }
                  if(   bKeepAllDropCurves == FALSE
                     && sUVTEnd2d.DistanceBetween(sEndPnt2d) > dApproxTol) 
                    { SM_ASSERT( FALSE ); }
                }
#endif // SM_DEBUG_CODE
              
              // all done - set output and return
              rUVCurves.Add(pLine2d);
              rdMaxApproxDev = 0.0;  // the UVLine is an exact rep of the UVDropCurve
            } // end not a degenerate curve check

          return SM_SUCCESS;

        } // end dropCurve is good enough check
    } // end if curve is a line

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smgfx_Erase();
      cr3dCurve.Draw(); sm_GraphicsLoop();
      cr3dCurve.DrawAt(sIvl.GetMin(),1); sm_GraphicsLoop();
      this->DrawUV(3,3); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // arrive here for all nonLine 3d curves

  // when the curve is a BSpline Curve
  if(cr3dCurve.IsKindOf(SmBSplineCurve_TYPE))
    {
      const SmBSplineCurve &crBSplineCurve = (const SmBSplineCurve &)cr3dCurve ;
      // try dropping all the control points to the plane
      //   if they fit within the PlaneUVDomain then those dropped control
      //   points define exactly the dropped curve.  Otherwise call 
      //   the general SmSurface::DropCurve function.
      // GWC Comment: This algorithm will not detect when a valid curve 
      //              is dropped into a self-intersecting one or to a curve
      //              with cusps (e.g. when a circle perp to the plane is dropped
      //                               to the plane - it forms a double line with 2 cusps.)

      // copy and trim the the original crv to the given interval
      //   (might reduce the number of control points to drop and
      //    increase the number of curves handled by the cheaper dropPlane option)
      SmBSplineCurve *pCrvCopy3d = new(crContext) SmBSplineCurve(crBSplineCurve);
      SmObjDelete     sClean(pCrvCopy3d);
      pCrvCopy3d->Trim(sIvl);   // may snap sIvl by tol to existing knots

      // locals for GetCanonical call
      ULONG               lDim, lDegree;
      SmBSplineCurveForm  eCurveForm;
      SmPoint3d           sPData[256];
      SmTArray<SmPoint3d> sCtrlPts(256,sPData);
      double              dDData[256];
      SmTArray<double>    sKnots(256,dDData);
      ULONG               dLData[256];
      SmTArray<ULONG>     sKnotMult(256,dLData);
      double              dWData[256];
      SmTArray<double>    sWeights(256,dWData);
      SmKnotType          eKnotType;

      // get the BSpline representation of the curve
      SER(pCrvCopy3d->GetCanonical(lDim,lDegree,
                                   sCtrlPts,eCurveForm,
                                   sKnotMult,sKnots,
                                   eKnotType,sWeights));

      // loop locals
      double dMaxDropSq = 0.0;
      SmPoint3d *p3DPnt ;
      SmPoint3d s3DOnSrf;
      SmPoint2d sUVPnt;

      // for every control point
      for (ULONG i=0; i<sCtrlPts.GetSize(); i++) 
        {
          p3DPnt = &sCtrlPts[i];

          // project to infinite plane
          SER(ProjectPointToUVDomain(*p3DPnt,sUVPnt));

          // clamp control points - (when clamped - this changes the shape of the drop curve)
          sUVPnt = crUVDomain.ClampPoint2d(sUVPnt);

          // save largest control point drop
          SER(EvaluatePointFast(sUVPnt,s3DOnSrf));
          double dDropToSurfSq = s3DOnSrf.DistanceBetweenSquared(*p3DPnt);
          if (dDropToSurfSq > dMaxDropSq) 
            {
              dMaxDropSq = dDropToSurfSq;
            }

          // find out how far clamping moved the control point from the drop point
          double     dDevFromSrfNrmLine, dDropDist;
          SmVector3d sPlaneNormal;
          SER(EvaluateNormal(sUVPnt,TRUE,TRUE,sPlaneNormal)); // unit normal
             //      SER(smgu_LineClosestPoint(s3DOnSrf, sPlaneNormal, s3DPnt, dLineT));
             //      SmVector3d sLinePnt = s3DOnSrf + dLineT * sPlaneNormal;
             //      SmVector3d sDiff = sLinePnt - s3DPnt;
          SER(smgu_LinePointDistance(s3DOnSrf, sPlaneNormal, *p3DPnt, dDevFromSrfNrmLine, &dDropDist)) ;

          // if clamping moves the control point - give up - pass the call along to the general SmSurface::DropCurve
          if(dDevFromSrfNrmLine > dApproxTol) 
            {
              return(SmSurface::DropCurve(crContext,                // in : context for new object construction                                                    
                                          crUVDomain,               // in : domain of interest for this surface                                                    
                                          crBSplineCurve,           // in : Curve to project onto the surface                                                      
                                          sIvl,                     // in : interval of interest for target curve                                                  
                                          dApproxTol,               // in : max allowed distance between drop point and surfNormal line at drop point              
                                                                    //      max allowed distance between 3dCurve and Surface for successful drops.                 
                                          rdMaxDropToSurf,          // out: Max dist between 3dCurve(t) and Surface(UVCurve(t)).                                   
                                                                    //      Found by checking 20 sample pts - it might be slightly less than the actual max.       
                                          rdMaxApproxDev,           // out: Max deviation from ideal dropCurve and surfaceDropPoints 
                                          rUVCurves,                // out: 1 (or 2) curves constructed by projection.                                             
                                                                    //      (2 curves for closed surfaces when crBSplineCurve is coincident with seam)                  
                                          bKeepAllDropCurves,       // in : TRUE= return all curves dropped
                                                                    //      FALSE= only return drop curves with drop distances less than dApproxTol                
                                          NULL,
                                          bCreatingUVTrimCurve));    
            }
      
          // save the dropped control point                                            
          sCtrlPts[i] = SmPoint3d(sUVPnt.x,sUVPnt.y,0.0);

        } // end iter every control point

      // arrive here when all control points drop nicely within the plane trim boundaries
      SmBSplineCurve *pUVCurve = NULL ;

      if(   bKeepAllDropCurves
         || dMaxDropSq < dApproxTol*dApproxTol)
        {
          // build 2d UVDropCurve
          SER(SmBSplineCurve::CreateCanonical(crContext,
                                              2, lDegree,
                                              sCtrlPts, eCurveForm,
                                              sKnotMult, sKnots, eKnotType,
                                              (crBSplineCurve.IsRational() ? &sWeights : NULL),
                                              NULL,
                                              pUVCurve));

          // set output
          rUVCurves.Add(pUVCurve);
          rdMaxDropToSurf  = smos_Sqrt(dMaxDropSq);
          rdMaxApproxDev   = 0.0; // ControlPt projection produces an exact UVDropCurve
        } // end have a good UVTrimCurve check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2=FALSE;
      if (bDebugMe2)  // compare this output to general DropCurve output
        {
          double dThisDropToSurf, dThisApproxDev ;
          SmTArray<SmBSplineCurve*> sTempCurves;
          SmStatus eStat = SmSurface::DropCurve(crContext,
                                                crUVDomain,
                                                crBSplineCurve,
                                                crInterval,
                                                dApproxTol,
                                                dThisDropToSurf,
                                                dThisApproxDev,
                                                sTempCurves,
                                                bKeepAllDropCurves,
                                                NULL,
                                                bCreatingUVTrimCurve);
          if ( eStat == SM_SUCCESS && sTempCurves.GetSize() > 0 ) 
            {
              smgfx_SetLook(2,3, 0,0,0); sTempCurves[0]->DrawWDeriv(crInterval,0); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 1,0,0); pUVCurve->DrawWDeriv(crInterval,0); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
        }
#endif // SM_DEBUG_CODE

      // all done 
      return SM_SUCCESS;

    } // end 3dCurve is a BSplineCurve check

  // else pass the call along to the general DropCurve method
  return(SmSurface::DropCurve(crContext,                // in : context for new object construction                                                    
                              crUVDomain,               // in : domain of interest for this surface                                                    
                              cr3dCurve,                // in : Curve to project onto the surface                                                      
                              sIvl,                     // in : interval of interest for target curve                                                  
                              dApproxTol,               // in : max allowed distance between drop point and surfNormal line at drop point              
                                                        //      max allowed distance between 3dCurve and Surface for successful drops.                 
                              rdMaxDropToSurf,          // out: Max dist between 3dCurve(t) and Surface(UVCurve(t)).                                   
                                                        //      Found by checking 20 sample pts - it might be slightly less than the actual max.       
                              rdMaxApproxDev,           // out: Max deviation from ideal dropCurve and surfaceDropPoints 
                              rUVCurves,                // out: 1 (or 2) curves constructed by projection.                                             
                                                        //      (2 curves for closed surfaces when crBSplineCurve is coincident with seam)                  
                              bKeepAllDropCurves,       // in : TRUE= return all curves dropped
                                                        //      FALSE= only return drop curves with drop distances less than dApproxTol                
                              NULL,
                              bCreatingUVTrimCurve));

} // end SmPlane::DropCurve

/*******************************************************************//**
PURPOSE: Drop a point to a plane very fast.  Note that this is an
    optimization routine and should not be used for general dropping.
    For general surfaces this method returns an error.

NOTES: Only Minimize, Normalize and Intersect allowed.
  Only returns points that drop within the analytic domain to within tolerance.  
  Else finds no drop point and returns SM_ERR forcing the calling function
  to try again with a more global solver.
***********************************************************************/
SmStatus SmPlane::DropPointFast
  (const SmExtent2d        & crPlaneUVDomain,         // in : this plane's domain
   SmSolverOperationType     eSolverOperation,        // in : oneof SM_SO_MINIMIZE, SM_SO_NORMALIZE, or SM_SO_INTERSECT
   const SmPoint3d         & crTestPoint,             // in : target point
   const SmVector3d        * cpOptInPointingVector,   // Notused: in : cpOptInPointingVector not used
   double                    dDistanceTolerance,      // in : when eSolverOperation == SM_SO_INTERSECT
                                                      //        skip solutions larger than DistanceTolerance
   const double            * cpdOptTargetDistance,    // in : when eSolverOperation == SM_SO_MINIMIZE
                                                      //        skip solutions larger than TargetDistance
                                                      //        NULL to ignore.
   SmSolutionRequestedType   eSolutionRequested,      // NotUsed: in : 
   SmSolutionArray         & rSolutions)              // out: Solution array with 0 or 1 entries
  const
{
  SM_REF2(cpOptInPointingVector, eSolutionRequested) ; 
  // check input
  if (   eSolverOperation != SM_SO_MINIMIZE
      && eSolverOperation != SM_SO_NORMALIZE
      && eSolverOperation != SM_SO_INTERSECT) 
    {
      SER_MSG(SM_ERR,_T("SmPlane::DropPointFast() bad input eSolverOperation value"));
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,0) ; crTestPoint.Draw() ; sm_GraphicsLoop() ; 
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // init output
  rSolutions.ReSet();

  // Project targetPoint to plane
  SmPoint2d sUVFound;
  SER(ProjectPointToUVDomain(crTestPoint,sUVFound));

  // when NORMALIZING - only use project solutions within the plane domain
  if (   eSolverOperation == SM_SO_NORMALIZE )
  {
      SmTol2d sTolUV = SmTol::MapTo2d( SmTol3d(dDistanceTolerance), sUVFound, *this );
      if ( !crPlaneUVDomain.ContainsPoint2d( sUVFound, sTolUV ))
        {
          return SM_SUCCESS; // No answer found
        }
  }

  // clamp projectPoint to planeDomain
  sUVFound = crPlaneUVDomain.ClampPoint2d(sUVFound);

  // get 3dProjectPoint/TargetPoint gap
  SmPoint3d sPnt;
  SER(EvaluatePointFast(sUVFound,sPnt));
  double dDist = sPnt.DistanceBetween(crTestPoint);

  // when INTERSECTING skip solutions > DistanceTolerance
  if(   eSolverOperation == SM_SO_INTERSECT 
     && dDist > dDistanceTolerance) 
    { return SM_SUCCESS; // No answer found
    }

  // when MINIMIZING or NORMALIZING skip solutions > TargetDistance
  if(   cpdOptTargetDistance
     && (   eSolverOperation == SM_SO_MINIMIZE
         || eSolverOperation == SM_SO_NORMALIZE)
     && dDist > *cpdOptTargetDistance) 
    { return SM_SUCCESS; // No answer found
    }

  SmSolution sSol;
  sSol.m_eSolutionType           = SM_ST_SINGLE_VALUE;
  sSol.m_lNumObjects             = 1;
  sSol.m_apObjects[0]            = SM_CONST_CAST(SmPlane*,this);
  sSol.m_apNodes[0]              = NULL;
  sSol.m_lNumVariables           = 2;
  sSol.m_vStart.m_dSolutionValue = dDist;
  sSol.m_vStart[0]               = sUVFound.x;
  sSol.m_vStart[1]               = sUVFound.y;
  rSolutions.Add(sSol);
  return SM_SUCCESS;

} // end SmPlane::DropPointFast

/*******************************************************************//**
PURPOSE: Evaluate a point on a plane.

NOTES: 
***********************************************************************/
SmStatus SmPlane::EvaluatePoint
  (const SmPoint2d & crUV,      // in : UV Point to evaluate - gets clamped
   SmPoint3d & rPoint)          // out: evaluated 3d point
  const
{
  SmPoint2d sUV = crUV;

  // clamp UVPoint to UVDomain
  if ( ! IsOutOfBoundsEnabled() && !m_vAnalUVDomain.ContainsPoint2d(sUV)) 
    {
      sUV = m_vAnalUVDomain.ClampPoint2d(sUV);
    }

  // evaluate
  rPoint =   m_vPosition.GetOriginRef() 
           + sUV.x * m_vUVScale.x * m_vPosition.GetXAxisRef()
           + sUV.y * m_vUVScale.y * m_vPosition.GetYAxisRef();

  // all done
  return SM_SUCCESS;

} // end SmPlane::EvaluatePoint

/*******************************************************************//**
PURPOSE: Evaluate a point and derivatives on a plane.

NOTES: Higher order evaluation request generate an error
***********************************************************************/
SmStatus SmPlane::Evaluate
  (const SmPoint2d & crUV,     // in : param value to evaluate
   ULONG lHighestUDeriv,       // in : number of U derivatives
   ULONG lHighestVDeriv,       // in : number of V derivatives to compute
   SmBoolean ,                 // in : bUFromLeft: if P is on U interval boundary
                               //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                               //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean ,                 // in : bVFromLeft: if P is on V interval boundary
                               //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                               //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bOnlyUpperHalf,   // in : TRUE=compute upper half of matrix only
                               //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value 
                               //                [Dv --]       [Dv   Duv   ---]           (the memory has to be allocated)
                               //                              [Dvv  ---   ---]
   SmVector3d *aDerivatives,   // out: matrix of evaluations values
                               //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]
                               //      2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)
                               //                    [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )
                               //                    [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]
                               //                    [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]
                               //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]
                               //                     Du, Duv, Duvv, Duvvv,.. 
                               //                     Duu, Duuv, Duuvv, Duuvvv,...]
  SmBoolean bNonZeroTangents,  // NotUsed: in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                               //      FALSE= return exact tangent values
                               //      note: Surprisingly TRUE is the common choice because most tangent uses
                               //            are for their direction (Binorm, SurfNorm comps), but when the 
                               //            tangent is being used for its magnitude (like an arc-length comp)
                               //            then set this to FALSE.
                               //      default:[TRUE]
  SmBoolean )                  // in : bDoZeroSampling = for internal use only, always set to TRUE, default:[TRUE]
 const  
{
  SM_REF1(bNonZeroTangents) ; 
  // local
  SmPoint2d sUV = crUV;

  // when appropriate - clamp uncontained points
  if(   !IsOutOfBoundsEnabled() 
     && !m_vAnalUVDomain.ContainsPoint2d(sUV)) 
    {
      sUV = m_vAnalUVDomain.ClampPoint2d(sUV);
    }

  // init output - only clear those values actually computed
  //   so that the SmVector3d calls can signal unitialized values when appropriate
  
  // smos_MemSet(aDerivatives, 0, sizeof(SmVector3d) * (lHighestUDeriv+1) * (lHighestVDeriv+1) );
  for(ULONG k=0; k<=lHighestUDeriv; k++ )
    {
      ULONG lHighestVIndex = bOnlyUpperHalf ? lHighestVDeriv-k : lHighestVDeriv ; 
      for(ULONG l=0; l<=lHighestVIndex; l++ )
        {
          // okay to use smos_MemSet on static (SmVector3d) objects.
          smos_MemSet(&(aDerivatives[k*(lHighestVDeriv+1)+l]), 0, sizeof(SmVector3d));
          //      aDerivatives[k*(lHighestVDeriv+1)+l].x = 0.0;
          //      aDerivatives[k*(lHighestVDeriv+1)+l].y = 0.0;
          //      aDerivatives[k*(lHighestVDeriv+1)+l].z = 0.0;
        }
    }
          
  // position
  aDerivatives[0] =  m_vPosition.GetOriginRef() 
                   + sUV.x * m_vPosition.GetXAxisRef() * m_vUVScale.x 
                   + sUV.y * m_vPosition.GetYAxisRef() * m_vUVScale.y;
          
  // 1st deriv in U
  if (lHighestVDeriv > 0) { aDerivatives[1] = m_vPosition.GetYAxisRef() * m_vUVScale.y ; }                                         
  if (lHighestUDeriv > 0) { aDerivatives[lHighestVDeriv+1] = m_vPosition.GetXAxisRef() * m_vUVScale.x ; }
          
  // higher order derivatives already set to zero - no more work to do.
  
  // all done
  return SM_SUCCESS;

} // end SmPlane::Evaluate

/*******************************************************************//**
PURPOSE: Evaluation of the normal of a surface at a point.  This routine
   returns a unitized normal vector or an error if unable to compute a
   normal.

NOTES:  Note that this routine will do its very best to find an
   answer in cases where where there is a pole or singularity.
***********************************************************************/
SmStatus SmPlane::EvaluateNormal
  (const SmPoint2d & /*crUV*/,
   SmBoolean /*bUFromLeft*/,     // in : if P is on U interval boundary
                                 //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                 //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean /*bVFromLeft*/,     // in : if P is on V interval boundary
                                 //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                 //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmVector3d & rSurfaceNormal)  // out: unit-normal
  const
{   
  rSurfaceNormal = m_vPosition.GetZAxis();
  return SM_SUCCESS;

} // end SmPlane::EvaluateNormal

/*******************************************************************//**
PURPOSE: This method intersects a line with a plane.  The line can
   be a ray (bRayFire==TRUE), an infinite line (cpOptLineInterval==NULL),
   or a line segment (cpOptLineInterval!=NULL).  When firing a ray only
   the closest intersection along the ray will be found.  
   When intersecting a line segment only intersections within the 
   bounding interval will be found.  Also, solutions outside of
   the plane domain will not be returned.

NOTES: Please note that the solutions returned by this method
   will have three parameters.  The first parameter belongs to the line
   and is parameterized naturally.  The second two parameters in a solution
   belong to the surface.
***********************************************************************/
SmStatus SmPlane::GlobalLineIntersect
 (const SmExtent2d & crPlaneUVDomain,    // in : Plane's Domain
  const SmPoint3d  & crLinePoint,        // in : Point on the infinite line
  const SmVector3d & crLineVector,       // in : Direction vector of the infinite line
  const SmExtent1d * cpOptLineInterval,  // in : If specified bounds the line to a specific segment
  SmBoolean          bFireRay,           // in : TRUE = bounded at StartPt and proceeds along Vec to infinity.
                                         //      FALSE= bounded by cpOptLineInterval if given, else infinite.
  double             dDistanceTolerance, // in : Max Dist at which two points still intersect
  SmSolutionArray & rSolutions)          // out: solutions: sSol.m_vStart[0] = Line param
 const                                   //                 sSol.m_vStart[1] = Plane param U
                                         //                 sSol.m_vStart[2] = Plane param V
{
    if ((crLineVector).LengthSquared() < SM_EFF_ZERO_SQ) { SER(SM_ERR_INVALID_INPUT) }

    // init output
    rSolutions.ReSet();

    // locals
    SmVector3d sPlaneNormal;
    SER(EvaluateNormal(crPlaneUVDomain.GetMin(),TRUE,TRUE,sPlaneNormal));
    double dAngleTol = 0.0001; // 1/10000 of a degree

    // check for plane/curve coincidence
    if (sPlaneNormal.IsPerpendicularTo(crLineVector,dAngleTol)) 
      {
        // Line and plane are nearly parallel - have either coincidence or miss.
        // Trim the line to the plane boundaries.

        // get interval to trim - given, infinite, or semi-infinite (for ray firing)
        SmExtent1d sIvl;
        if ( cpOptLineInterval != NULL )
          {
            sIvl = *cpOptLineInterval;
          }
        else
          {
            // Drop plane corners to line, to get param range of possible intersection.
            SmPoint3d sPt;
            SmPoint2d sLo = crPlaneUVDomain.GetMin();
            SmPoint2d sHi = crPlaneUVDomain.GetMax();
            SmPoint2d sUV( sLo );
            double dLineParam;
            SER( this->EvaluatePointFast( sUV, sPt ));
            smgu_LineClosestPoint( crLinePoint, crLineVector, sPt, dLineParam );
            sIvl.AddValue( dLineParam );
            sUV.Set( sLo.x, sHi.y );
            SER( this->EvaluatePointFast( sUV, sPt ));
            smgu_LineClosestPoint( crLinePoint, crLineVector, sPt, dLineParam );
            sIvl.AddValue( dLineParam );
            sUV.Set( sHi.x, sLo.y );
            SER( this->EvaluatePointFast( sUV, sPt ));
            smgu_LineClosestPoint( crLinePoint, crLineVector, sPt, dLineParam );
            sIvl.AddValue( dLineParam );
            sUV.Set( sHi.x, sHi.y );
            SER( this->EvaluatePointFast( sUV, sPt ));
            smgu_LineClosestPoint( crLinePoint, crLineVector, sPt, dLineParam );
            sIvl.AddValue( dLineParam );

            sIvl.ExpandRelative( 1.2 );
          }
        if ( bFireRay )
          { sIvl.SetMinMax( 0.0, smos_Max( sIvl.GetMax(), 0.0 ) ); }

        // get untrimmed interval endPoints
        SmPoint3d sPlaneStartPt = crLinePoint + sIvl.GetMin() * crLineVector;
        SmPoint3d sPlaneEndPt   = crLinePoint + sIvl.GetMax() * crLineVector;

        // project untrimmed interval endPoints to plane UV params
        SmPoint2d sUVStart, sUVEnd;
        SER(ProjectPointToUVDomain(sPlaneStartPt,sUVStart));
        SER(ProjectPointToUVDomain(sPlaneEndPt,  sUVEnd));

        // build UVLine vector
        SmVector2d sUVVec = sUVEnd - sUVStart;

        // find interval for UVLine that maps to plane domain
        SmExtent1d sTrimIvl;
        SmBoolean bFound;

        SER(crPlaneUVDomain.IntersectWithInfiniteLine(sUVStart,sUVVec,bFound,sTrimIvl));

        if (!bFound) 
          { return SM_SUCCESS; }

        // handle round off - max UVLine interval is 0.0 to 1.0
        double dMin = smos_Max(sTrimIvl.GetMin(),0.0);
        double dMax = smos_Min(sTrimIvl.GetMax(),1.0);

        if (dMin > dMax+SM_EFF_ZERO) // was dMax-SM_EFF_ZERO: missed single-point ints.
          { return SM_SUCCESS; } // no trimmed portion in area of interest

        // get UVLine 2D trimmed endPoints
        SmPoint2d sTrimStart = sUVStart + dMin * sUVVec;
        SmPoint2d sTrimEnd   = sUVStart + dMax * sUVVec;

        // get associated 3D trimmed endPoints on Plane
        SER(EvaluatePointFast(sTrimStart,sPlaneStartPt));
        SER(EvaluatePointFast(sTrimEnd,  sPlaneEndPt));

        // get closest point on Line to trimmed plane start point
        double dTStart;
        SER(smgu_LineClosestPoint(crLinePoint,crLineVector,sPlaneStartPt,dTStart));
        SmVector3d sTestStart = crLinePoint + dTStart * crLineVector;
        double     dStartDist = sTestStart.DistanceBetween(sPlaneStartPt);

        // quit when plane points are not close to line points
        if (dStartDist > dDistanceTolerance) 
          { return SM_SUCCESS; }

        // get closest point on Line to trimmed plane end point
        double dTEnd;
        SER(smgu_LineClosestPoint(crLinePoint,crLineVector,sPlaneEndPt,dTEnd));
        SmVector3d sTestEnd = crLinePoint + dTEnd * crLineVector;
        double     dEndDist = sTestEnd.DistanceBetween(sPlaneEndPt);
        
        // quit when plane points are not close to line points
        if (dEndDist > dDistanceTolerance) 
          { return SM_SUCCESS; }

        // when line parameters are the same
        if (SM_ARE_SAME(dTStart,dTEnd)) 
          {
            dTStart = dTStart - 1.0;
            dTEnd   = dTEnd + 1.0;
            if (cpOptLineInterval) 
              {
                dTStart = cpOptLineInterval->GetMin();
                dTEnd   = cpOptLineInterval->GetMax();
              }

            // try global surface/curve solver
            SmExtent1d sIntvl(dTStart,dTEnd);
            SmSurfaceCache *pSC = smsurf_GetSurfaceCache(this); NER(pSC);
            SmCacheCheckOutIn sCheckIO(pSC);
              {
                // Make the surface think that it is a standalone surface.
                SmTemporaryChangeValue<SmBoolean> sChange(pSC->m_bHaveTSurfaceCache,FALSE);
                return SmSurface::GlobalLineIntersect(crPlaneUVDomain,crLinePoint,crLineVector,
                    &sIntvl,bFireRay,dDistanceTolerance,rSolutions);
              }
          }

        // test that 3D Points project to UVDomain
        SER(ProjectPointToUVDomain(sPlaneStartPt,sUVStart));
        SER(ProjectPointToUVDomain(sPlaneEndPt,  sUVEnd));

        // build a coincident solution 
        SmSolution sSol;
        sSol.m_lNumVariables           = 3;
        sSol.m_lNumObjects             = 0;
        sSol.m_eSolutionType           = SM_ST_RANGE_OF_VALUES;
        sSol.m_vStart.m_dSolutionValue = dStartDist;
        sSol.m_vStart[0]               = sIvl.ClampValue(dTStart);
        sSol.m_vStart[1]               = sUVStart.x;
        sSol.m_vStart[2]               = sUVStart.y;

        sSol.m_vEnd.m_dSolutionValue   = dEndDist;
        sSol.m_vEnd[0]                 = sIvl.ClampValue(dTEnd);
        sSol.m_vEnd[1]                 = sUVEnd.x;
        sSol.m_vEnd[2]                 = sUVEnd.y;

        // set the output
        rSolutions.Add(sSol);

      } // end coincident branch

    else  // Single intersection
      {
        // Find curve t and plane uv, and check domains.
        // If the solution is outside either domain, return no solution.
        double dTParam;
        SmPoint2d sUV;

        // Set the curve domain.
        SmExtent1d sCurveDomain( -1.0e10, 1.0e10 );
        if ( cpOptLineInterval )
          { sCurveDomain = *cpOptLineInterval; }
        if ( bFireRay )
          { sCurveDomain.SetMinMax( 0.0, SM_BIG_DOUBLE ); }

        // Method: Plane P(u,v) = P0 + u*Pu + v*Pv, Line L(t) = L0 + t*Lt,
        //  Pu, Pv and Lt are all constants, so set line and plane points equal,
        //  get a 3x3 linear system in the three unknown parameters u, v and t:
        //
        //  u*Pu + v*Pv - t*Lt  =  ( L0 - P0 );
        //
        // The matrix equation is:
        //
        //  (            )     ( u )     (    )
        //  ( Pu  Pv  Lt )  *  ( v )  =  ( dV )
        //  (            )     ( t )     (    )
        //
        // Solve 3x3 by Cramer's rule.  Since the matrices are each three
        // column vectors, their determinants are just their triple products.

        SmVector3d Pu = m_vPosition.GetXAxis() * m_vUVScale.x;
        SmVector3d Pv = m_vPosition.GetYAxis() * m_vUVScale.y;
        SmVector3d Lt = -crLineVector;
        SmVector3d Diff ( crLinePoint - m_vPosition.GetOriginRef() );

        double dDet = ( Pu * Pv ).Dot( Lt );
        if ( fabs(dDet) < SM_EFF_ZERO )
          { SER( SM_ERR ); }  // should never happen, we're in the non-parallel case here.

        // Cramer's rule:
        double dNumerT = ( Pu * Pv ).Dot( Diff ); // triple product
        dTParam = dNumerT / dDet;

        // Check curve param vs. domain.
        if ( !sCurveDomain.ContainsValue( dTParam ))
          { return SM_SUCCESS;  // but no solution.
          }

        double dNumerU = ( Diff * Pv ).Dot( Lt );
        double dNumerV = ( Pu * Diff ).Dot( Lt );
        sUV.Set( dNumerU / dDet, dNumerV / dDet );

        // Now we have t, u, and v.
        // Eval curve and plane, and check whether they're within tol.
        // Clamp uv first, that will be how we tell whether the solution
        // is outside of our domain.

        SmPoint3d sLinePoint = crLinePoint + dTParam * crLineVector;

        sUV = crPlaneUVDomain.ClampPoint2d( sUV );
        SmPoint3d sSrfPnt;
        SER( EvaluatePointFast( sUV, sSrfPnt ));
        double dDist = sSrfPnt.DistanceBetween( sLinePoint );
        if ( dDist > dDistanceTolerance )
          { return SM_SUCCESS;  // but no solution.
          }

        // Good to go, set up solution and get out.
        SmSolution sSol;
        sSol.m_lNumVariables           = 3;
        sSol.m_lNumObjects             = 0;
        sSol.m_eSolutionType           = SM_ST_SINGLE_VALUE;
        sSol.m_vStart.m_dSolutionValue = dDist;
        sSol.m_vStart[0]               = dTParam;
        sSol.m_vStart[1]               = sUV.x;
        sSol.m_vStart[2]               = sUV.y;
        rSolutions.Add( sSol );
      }

    return SM_SUCCESS;

} // end SmPlane::GlobalLineIntersect


/*******************************************************************//**
PURPOSE: Given a point in Euclidian space determine the corresponding
     extrema points on the surface based on STEP-parametrization.
     Valid solver operations for this method include:
     SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT.

NOTES:
***********************************************************************/
SmStatus SmPlane::GlobalPointSolveSTEP
  (const SmExtent2d & crAnalUVDomain,
   // STEP-based Domain of surface
   SmSolverOperationType eSolverOperation,
   const SmPoint3d & crTestPoint,
   double dDistanceTolerance,
   const double * cpdOptTargetDistance,
   SmSolutionRequestedType eSolutionRequested,
   SmSolutionArray & rSolutions)
{
    SM_ASSERT(eSolverOperation == SM_SO_MINIMIZE ||
              eSolverOperation == SM_SO_MAXIMIZE ||
              eSolverOperation == SM_SO_NORMALIZE ||
              eSolverOperation == SM_SO_INTERSECT);

    SER(AdjustSTEPUVDomain(crAnalUVDomain));

    if (eSolverOperation == SM_SO_MINIMIZE ||
        eSolverOperation == SM_SO_NORMALIZE ||
        eSolverOperation == SM_SO_INTERSECT) {
        SER(DropPointFast(crAnalUVDomain,eSolverOperation,crTestPoint,
            NULL,dDistanceTolerance,cpdOptTargetDistance,
            eSolutionRequested,rSolutions));
    }
    else {//eSolverOperation == SM_SO_MAXIMIZE
        SmSolution sSol;
        sSol.m_eSolutionType = SM_ST_SINGLE_VALUE;
        sSol.m_lNumObjects = 1;
        sSol.m_apObjects[0] = SM_CONST_CAST(SmPlane*,this);
        sSol.m_apNodes[0] = NULL;
        sSol.m_lNumVariables = 1;

        SmPoint2d sUVMin = crAnalUVDomain.GetMin();
        SmPoint2d sUVMax = crAnalUVDomain.GetMax();
        SmPoint2d sUV[4];
        double dDist[4];
        long lStack[4];
        sUV[0] = SmPoint2d(sUVMin.x,sUVMin.y);
        sUV[1] = SmPoint2d(sUVMin.x,sUVMax.y);
        sUV[2] = SmPoint2d(sUVMax.x,sUVMin.y);
        sUV[3] = SmPoint2d(sUVMax.x,sUVMax.y);
        for (ULONG i=0; i<4; i++) {
            lStack[i] = -1;//initialize
            SmPoint3d sPnt;
            SER(EvaluateSTEPPoint(sUV[i],sPnt));
            dDist[i] = crTestPoint.DistanceBetween(sPnt);
        }
        double dMaxDist = dDist[0];
        ULONG lMaxIndex = 0;
        lStack[lMaxIndex] = 0;
        for (ULONG j=1; j<4; j++) {
            if (dDist[j] < dMaxDist-dDistanceTolerance)
                continue;
            if (dDist[j] > dMaxDist+dDistanceTolerance) {
                // clear the whole stack and push in the current one
                lMaxIndex = 0;
                lStack[lMaxIndex] = j;
                dMaxDist = dDist[j];
                continue;
            }
            // push current one on top of stack
            if (dDist[j] > dMaxDist)
                dMaxDist = dDist[j];
            lMaxIndex++;   
            lStack[lMaxIndex] = j;
        }
        for (ULONG k=0; k<=lMaxIndex; k++) {
            sSol.m_vStart[0] = sUV[lStack[k]].x;
            sSol.m_vStart[1] = sUV[lStack[k]].y;
            sSol.m_vStart.m_dSolutionValue = dDist[lStack[k]];
            rSolutions.Add(sSol);
        }
    }

    return SM_SUCCESS;

} // end SmPlane::GlobalPointSolveSTEP

/*******************************************************************//**
PURPOSE: This virtual method may invoke special cases of intersection
    of analytics with planes.

NOTES: 
***********************************************************************/
SmStatus SmPlane::GlobalSurfaceIntersect
  (const SmContext     & crContext,               // in : Context for the creation of curves
   const SmExtent2d    & crPlaneUVDomain,         // 
   const SmSurface     & crOtherSurface,          // in : target 2nd intersecting surface
   const SmExtent2d    & crOtherUVDomain,         //
   const SmBoolean       bUseSurfaceEdges[2],     // in : Normally both are TRUE unless you know
                                                  //      that the edges of one surface do not intersect
                                                  //      the other surface.  It is a slight optimization
                                                  //      to set the flag to FALSE
   const SmApproxTol3d * pdOptApproxTol3d,        // in : If not given it uses 1/1000 of surface size
                                                  //      approximation tolerance
   const double        * pdOptAngTolRad,          // in : If not given it uses 30 degrees
   SmTArray<SmCurve*>  * pOpt3DCurves,            // out: 3D curves produced by intersection
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,    // out: UV curves on this surface produced by intersection
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,    // out: UV curves on crOtherSurface produced by intersection
   SmTArray<SmTsectCurveType> * pOptCurveTypes,   // out: What type of curve is produced - 
                                                  //      oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)
                                                  //             SM_TC_CROSSING   - curve intersection (surf norms not parallel)
                                                  //             SM_TC_TANGENT    - curve intersection (surf norms parallel)
                                                  //             SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal)
                                                  //             SM_TC_NEAR_TANGENT    - curve has small angle of intersection
                                                  //             SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
   SmTArray<double>    * pOptDeviations)          // out: 
  const
{
  // assume special case intersections will be complete
  SmBoolean bNeedNurbIntersection = TRUE;

  // switch on OtherSurface Type looking for cheap special intersection case
  switch (crOtherSurface.GetType()) 
    {
      // plane/plane special case intersection
      case SmPlane_TYPE:
          SER(IntersectWithPlane(crContext,crPlaneUVDomain,(SmPlane&)crOtherSurface,
                                 crOtherUVDomain, bUseSurfaceEdges, pdOptApproxTol3d,
                                 pdOptAngTolRad, bNeedNurbIntersection, 
                                 pOpt3DCurves, pOptSurface1UVCurves,
                                 pOptSurface2UVCurves, pOptCurveTypes, pOptDeviations));
          break;

      // cone/plane special case intersection
      case SmCone_TYPE:
      case SmCylinder_TYPE:
          {
              // Note that we swap ordering and swap UV curves.
              SmCone & rCone = (SmCone&)crOtherSurface;
              SmBoolean bUseSurfE[2];
              bUseSurfE[0] = bUseSurfaceEdges[1];
              bUseSurfE[1] = bUseSurfaceEdges[0];
              SER(rCone.IntersectWithPlane(crContext,crOtherUVDomain,*this,
                                           crPlaneUVDomain, bUseSurfE, pdOptApproxTol3d,
                                           pdOptAngTolRad, bNeedNurbIntersection, 
                                           pOpt3DCurves, pOptSurface2UVCurves,
                                           pOptSurface1UVCurves, pOptCurveTypes, pOptDeviations));
          }
          break;

      // sphere/plane special case intersection
      case SmSphere_TYPE:
          {
              // Note that we swap ordering and swap UV curves.
              SmSphere & rSphere = (SmSphere&)crOtherSurface;
              SmBoolean bUseSurfE[2];
              bUseSurfE[0] = bUseSurfaceEdges[1];
              bUseSurfE[1] = bUseSurfaceEdges[0];
              SER(rSphere.IntersectWithPlane(crContext,crOtherUVDomain,*this,
                                             crPlaneUVDomain, bUseSurfE, pdOptApproxTol3d,
                                             pdOptAngTolRad, bNeedNurbIntersection, 
                                             pOpt3DCurves, pOptSurface2UVCurves,
                                             pOptSurface1UVCurves, pOptCurveTypes, pOptDeviations));
          }
          break;

      // SurfOfExtrusion/plane special case intersection
      case SmSurfOfExtrusion_TYPE:
          {
              // Note that we swap ordering and swap UV curves.
              SmSurfOfExtrusion & rExt = (SmSurfOfExtrusion&)crOtherSurface;
              SmBoolean bUseSurfE[2];
              bUseSurfE[0] = bUseSurfaceEdges[1];
              bUseSurfE[1] = bUseSurfaceEdges[0];
              SER(rExt.IntersectWithPlane(crContext,crOtherUVDomain,*this,
                                          crPlaneUVDomain, bUseSurfE, pdOptApproxTol3d,
                                          pdOptAngTolRad, bNeedNurbIntersection, 
                                          pOpt3DCurves, pOptSurface2UVCurves,
                                          pOptSurface1UVCurves, pOptCurveTypes, pOptDeviations));
          }
          break;

      // SurfOfRevolution/plane special case intersection
      case SmSurfOfRevolution_TYPE:
          {
              // Note that we swap ordering and swap UV curves.
              SmSurfOfRevolution & rRev = (SmSurfOfRevolution&)crOtherSurface;
              SmBoolean bUseSurfE[2];
              bUseSurfE[0] = bUseSurfaceEdges[1];
              bUseSurfE[1] = bUseSurfaceEdges[0];
              SER(rRev.IntersectWithPlane(crContext,crOtherUVDomain,*this,
                                          crPlaneUVDomain, bUseSurfE, pdOptApproxTol3d,
                                          pdOptAngTolRad, bNeedNurbIntersection, 
                                          pOpt3DCurves, pOptSurface2UVCurves,
                                          pOptSurface1UVCurves, pOptCurveTypes, pOptDeviations));
          }
          break;

      default:
          break;
    } // end switch on other surface type

  // If we're here, then analytic intersection were not found - do general intersection
  if (bNeedNurbIntersection) 
    {
      SER(SmSurface::GlobalSurfaceIntersect
                       (crContext,
                        crPlaneUVDomain,
                        crOtherSurface,
                        crOtherUVDomain,
                        bUseSurfaceEdges,
                        pdOptApproxTol3d,
                        pdOptAngTolRad,
                        pOpt3DCurves,
                        pOptSurface1UVCurves,
                        pOptSurface2UVCurves,
                        pOptCurveTypes,
                        pOptDeviations));
    }

  // all done
  return SM_SUCCESS;

} // end SmPlane::GlobalSurfaceIntersect

/*******************************************************************//**
PURPOSE: Find intersection curve (line) of this with other plane.

NOTES:
  1. when planes are not parallel
     - sets rbNeedsMoreIntersections = FALSE 
       - when planes are bounded          - outputs 1 trimmed 3DLine 
                                              parameters from 0 to 1 
                                              are in the intersection 
         else when planes are not bounded - outputs 1 infinite 3dLine
       - sets UVTrimCurves = NULL,
         sets CurveType    = SM_TC_CROSSING,
         sets Deviation    = 0.0
  2. when planes are parallel
     - sets rbNeedsMoreIntersections = FALSE
       generates no output curves
  3. when planes are coincident
     - sets rbNeedsMoreIntersections = TRUE
       generates no output curves

***********************************************************************/
SmStatus SmPlane::IntersectWithPlane
  (const SmContext     & crContext,                  // in : context for new object construction
   const SmExtent2d    & crPlaneUVDomain,            // in : intersection limit for this surface
   const SmPlane       & crOtherPlane,               // in : target intersection plane
   const SmExtent2d    & crOtherUVDomain,            // in : intersection limit for target plane
   const SmBoolean       /* bUseSurfaceEdges */[2],  // in : TRUE = find xsect curve start points from boundaryCurve/surface xsects
                                                     //      typically these values are TRUE - its a small savings if you know
                                                     //      the boundaries of one surface don't intersect the other surface                               
   const SmApproxTol3d * pdOptApproxTol3d,           // in :
   const double        * /* pdOptAngTolRad */,       // in :                                 
   SmBoolean           & rbNeedsMoreIntersections,   // out: TRUE = special case intersection failed-use general intersection 
   SmTArray<SmCurve*>  * pOpt3DCurves,               // out: Intersection 3DCurves, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface1UVCurves,       // out: associated UVTrimCurves on this surface, NULL to ignore
   SmTArray<SmCurve*>  * pOptSurface2UVCurves,       // out: associated UVTrimCurves on plane, NULL to ignore
   SmTArray<SmTsectCurveType> * pOptCurveTypes,      // out: oneof for each 3DCurve, NULL to ignore
                                                     //      SM_TC_TOUCHING        - single point intersection (surf norms parallel)
                                                     //      SM_TC_CROSSING        - curve intersection (surf norms not parallel)
                                                     //      SM_TC_TANGENT         - curve intersection (surf norms parallel)
                                                     //      SM_TC_COINCIDENT      - curve intersection (surf norms parallel & cross-tangents equal)
                                                     //      SM_TC_NEAR_TANGENT    - curve has small angle of intersection
                                                     //      SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
   SmTArray<double>    * pOptDeviations)             // out: associated max 3DCurve to surface distance, NULL to ignore
 const
{
  SM_DUMP_AND_ASSERT2_VALID(this) ;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  static ULONG lCount      = 1 ; lCount++ ;
  static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount) 
    {
      SmFace *pFace1 = (SmFace *)GetFace() ;
      SmFace *pFace2 = (SmFace *)crOtherPlane.GetFace() ;
      SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
      SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; crOtherPlane.DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace1) { pFace1->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace2) { pFace2->Draw(SM_DM_CROSSHATCH,10,10) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // init output
  rbNeedsMoreIntersections = FALSE;
  if (pOpt3DCurves)         pOpt3DCurves->ReSet();
  if (pOptSurface1UVCurves) pOptSurface1UVCurves->ReSet();
  if (pOptSurface2UVCurves) pOptSurface2UVCurves->ReSet();
  if (pOptCurveTypes)       pOptCurveTypes->ReSet();
  if (pOptDeviations)       pOptDeviations->ReSet();

  // no work - looking for self-intersections; planes have no self-intersections
  if (this == &crOtherPlane) 
    { return SM_SUCCESS; }

  // locals
  SmBoolean bThisBounded  = IsBounded() ;
  SmBoolean bOtherBounded = crOtherPlane.IsBounded() ;

  // get thisPlane midPoint and normal
  SmPoint3d sOriginThis;
  SER(EvaluatePointFast(crPlaneUVDomain.Evaluate(0.5,0.5),sOriginThis));
  SmVector3d sNormalThis = m_vPosition.GetZAxis();
  
  // get otherPlane midPoint and normal
  SmPoint3d sOriginOther;
  SER(crOtherPlane.EvaluatePointFast(crOtherUVDomain.Evaluate(0.5,0.5),sOriginOther));
  SmVector3d sNormalOther = crOtherPlane.m_vPosition.GetZAxis();

  // set tol proportional to midPoint size or as given
  double dTol = 0.0;
  if( pdOptApproxTol3d ) dTol = *pdOptApproxTol3d;
  else dTol = SM_EFF_ZERO_SQRT * (1.0 + sOriginThis.GetMaxDimension()+ sOriginOther.GetMaxDimension());

  // intersection line direction = CrossProduct(PlaneNormal1, PlaneNormal2)
  SmVector3d sLineDir = sNormalThis * sNormalOther;

  // when planes are parallel or coincident
  // Tol: This algorithm should work fine with vectors on the order of 1e-8.  [B601]
  double dAngTolSq = 1.0e-16;  // Cross product length is sine, which is same as angle for small angles.
  double dLenSq = sLineDir.LengthSquared();
  if ( dLenSq < dAngTolSq )
    {
      // Planes are parallel.
      if (this->m_pNurb == NULL || crOtherPlane.m_pNurb == NULL)
          return (SM_SUCCESS);

      // cull as possible and pass surviving calls to general intersector

      // get plane1 and plane2 bounding and pseudo boxes
      SmPseudoBox sPBox1, sPBox2;
      SmExtent3d  sBBox1, sBBox2;
      SER(CalculateBoundingBox(crPlaneUVDomain,&sBBox1,&sPBox1));
      SER(crOtherPlane.CalculateBoundingBox(crOtherUVDomain,&sBBox2,&sPBox2));

      // no work - when bounding or pseudo boxes are disjoint
      if (   sBBox1.AreDisjoint(sBBox2)
          || sPBox1.AreDisjoint(sPBox2)) return SM_SUCCESS;

      // let sPnt2 = plane2 midPoint projected to plane1 
      SmPoint2d sUVFound;
      SER(ProjectPointToUVDomain(sOriginOther,sUVFound));
      SmPoint3d sPnt2;
      SER(EvaluatePointFast(sUVFound,sPnt2));

      // no work - planes are parallel not coincident
      double dDist = sOriginOther.DistanceBetween(sPnt2);
      if (dDist > dTol) return SM_SUCCESS; 

      // more work needed - planes are coincident
      rbNeedsMoreIntersections = TRUE; 
      return SM_SUCCESS;  

   } // end planes are parallel or coincident check

  // Arrive here when planes are not parallel.

  // unitize lineDir and set OriginCross = avg(midPoint1, midPoint2) 
  SER(sLineDir.Unitize());
  SmPoint3d sOriginCross =   ( bThisBounded && !bOtherBounded) ? sOriginThis
                           : (!bThisBounded &&  bOtherBounded) ? sOriginOther
                           : (sOriginThis + sOriginOther) / 2.0 ;

  // Solve for point which is on both planes and as close
  // as possible to OriginCross.  This can be found as the 
  // solution to a 3x3 linear equation with x = the solution point
  //  0 = (x-P1)*N1  where P1 = plane1 point, N1 = plane1 normal
  //  0 = (x-P2)*N2        P2 = plane2 point, N2 = plane2 normal
  //  0 = (x-OC)*LD        OC = sOriginCross, LD = sLineDir

  // build A of Ax=b matrix problem
  SmMatrix sMat(3, 3);
  for (ULONG i=0; i<3; i++) 
    {
      sMat.SetAt(0, i, sNormalThis[i]);
      sMat.SetAt(1, i, sNormalOther[i]);
      sMat.SetAt(2, i, sLineDir[i]);
    }

  // build b of Ax=b matrix problem
  SmTArray<double> sRHS(3,NULL,3);
  sRHS.SetAt(0, sNormalThis.Dot(sOriginThis));
  sRHS.SetAt(1, sNormalOther.Dot(sOriginOther));
  sRHS.SetAt(2, sLineDir.Dot(sOriginCross));

  // let sStartPt = solution x in Ax=b
  SmTArray<double> sSolution(3, NULL);
  SER(sMat.SolveLinearSystem(sRHS, sSolution));
  SmPoint3d sStartPt(sSolution[0], sSolution[1], sSolution[2]);

  // when both planes aren't bounded - output infinite line
  if (   !bThisBounded
      && !bOtherBounded) 
    {
      // Create an 'infinite' line
      SmCurve * pLine = new (crContext) SmLine(sStartPt, sLineDir, 3);
      if (pOpt3DCurves) pOpt3DCurves->Add(pLine);
      if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
      if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
      if (pOptCurveTypes) pOptCurveTypes->Add(SM_TC_CROSSING);
      if (pOptDeviations) pOptDeviations->Add(0.0);
    }
  else // set output when planes are bounded
    {
      // Create a bounded line

      // get bounding and pseudo boxes for both planes
      double      dRadius1, dRadius2, dDist = 0.0 ; 
      SmPseudoBox sPBox1,   sPBox2; 
      SmExtent3d  sBBox1,   sBBox2; 
      SmPoint3d   sCent1,   sCent2;
      
      // get size of bounded ThisPlane
      if(bThisBounded)
        {
          SER(CalculateBoundingBox(crPlaneUVDomain,&sBBox1,&sPBox1));
          sBBox1.ComputeSphereBound(sCent1,dRadius1);
          sBBox1.ExpandAbsolute(dTol/2.0);
          sPBox1.ExpandAbsolute(dTol/2.0);
          dDist += 2.0 * dRadius1 ;
        }

      // get size of bounded OtherPlane
      if(bOtherBounded)
        {
          SER(crOtherPlane.CalculateBoundingBox(crOtherUVDomain,&sBBox2,&sPBox2));
          sBBox2.ComputeSphereBound(sCent2,dRadius2);
          sBBox2.ExpandAbsolute(dTol/2.0);
          sPBox2.ExpandAbsolute(dTol/2.0);
          dDist += 2.0 * dRadius2 ;
        }

      // no work - bounding boxes are disjoint
      if (   (bThisBounded && bOtherBounded)
          && (   sBBox1.AreDisjoint(sBBox2)
              || sPBox1.AreDisjoint(sPBox2)))
        { return SM_SUCCESS; }

      // Init the line length to be longer than both bounding boxes
      SmPoint3d sSegStart = sStartPt - dDist * sLineDir;
      SmPoint3d sSegEnd   = sStartPt + dDist * sLineDir;

      // Trim line to Plane boundaries (no trimming infinite planes)
      SmBoolean bThisFoundInterval=TRUE, bOtherFoundInterval=TRUE ;

      if(bThisBounded)  { TrimPlaneLineToPlaneDomain(crPlaneUVDomain, 
                                                     sSegStart, sSegEnd, dTol, 
                                                     sSegStart, sSegEnd, 
                                                     bThisFoundInterval) ;
                        }
      if(bOtherBounded) { crOtherPlane.TrimPlaneLineToPlaneDomain(crOtherUVDomain, 
                                                                  sSegStart, sSegEnd, dTol, 
                                                                  sSegStart, sSegEnd, 
                                                                  bOtherFoundInterval) ;
                        }

      // when line segment doesn't intersect both plane domains
      if(   !bThisFoundInterval 
         || !bOtherFoundInterval)
        {
          // arrive here when intervals don't intersect - no intersection
          return SM_SUCCESS;
        }

      // when segment is short - return degenerate curve xsect
      double dLengthOfLine = sSegStart.DistanceBetween(sSegEnd) ;
      if (dLengthOfLine < dTol) 
        {
          SmBSplineCurve *pBSC;
          SER(SmBSplineCurve::CreateDegenerateCurve(crContext,3,sSegStart,pBSC));
          NER(pBSC);
          if (pOpt3DCurves)         pOpt3DCurves->Add(pBSC);
          if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
          if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
          if (pOptCurveTypes)       pOptCurveTypes->Add(SM_TC_TOUCHING);
          if (pOptDeviations)       pOptDeviations->Add(0.0) ;
          return SM_SUCCESS;
        }

      // create output line and set outputs
      SmCurve *pLine = new (crContext) SmLine(sSegStart, 
                                              sLineDir, 
                                              SmExtent1d(0.0,1.0),
                                              dLengthOfLine, 3);

      SmObjDelete sCleanLine( pLine );
      if ( pOpt3DCurves )     { pOpt3DCurves->Add( pLine );
                                sCleanLine.Clear(); }
      if (pOptSurface1UVCurves) pOptSurface1UVCurves->Add(NULL);
      if (pOptSurface2UVCurves) pOptSurface2UVCurves->Add(NULL);
      if (pOptCurveTypes)       pOptCurveTypes->Add(SM_TC_CROSSING);
      if (pOptDeviations)       pOptDeviations->Add(0.0);

    } // end set output when planes are bounded branch

#ifdef SM_DEBUG_CODE
      if (bDebugMe || lCount == lDebugCount) 
        {
          SmFace *pFace1 = (SmFace *)GetFace() ;
          SmFace *pFace2 = (SmFace *)crOtherPlane.GetFace() ;
          SmBrep *pBrep1 = pFace1 ? pFace1->GetBrep() : NULL ;
          SmBrep *pBrep2 = pFace2 ? pFace2->GetBrep() : NULL ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; crOtherPlane.DrawUV() ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,0,0) ; for(ULONG ii=0;ii<pOpt3DCurves->GetSize();ii++)
                                        { SmCurve *pCurve = (*pOpt3DCurves)[ii] ;
                                          pCurve->Draw() ; sm_GraphicsLoop() ;
                                        }
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS; 

} // end SmPlane::IntersectWithPlane

/*******************************************************************//**
PURPOSE: Trim line endPoints that lie outside of given plane analytic domain
            to analytic boundary.

NOTES: Sets bFoundInterval = FALSE when line is completely outside of domain 
***********************************************************************/
SmStatus SmPlane::TrimPlaneLineToPlaneDomain
  (const SmExtent2d & crPlaneUVDomain,   // in : Extent of this plane
   const SmPoint3d  & rStartPt,          // in : start point of line segment
   const SmPoint3d  & rEndPt,            // in : end point of line segment
   double             d3dTolerance,      // in : max allowed point/plane boundary
   SmPoint3d        & rStartTrim,        // out: start point of trimmed line segment
   SmPoint3d        & rEndTrim,          // out: end point of trimmed line segment
   SmBoolean        & bFoundInterval)    // out: TRUE = some part of segment is in plane crPlaneUVDomain
 const                                   //      FALSE= segment is outside of plane crPlaneUVDomain
{
  bFoundInterval = FALSE ;
  SmExtent1d sThisIvl(0.0,1.0), sPlaneIvl, sIvl ;
  SmPoint2d sUVStart, sUVEnd, sUVVec ;
  SER(ProjectPointToUVDomain(rStartPt,sUVStart));
  SER(ProjectPointToUVDomain(rEndPt,sUVEnd));
  sUVVec = sUVEnd - sUVStart;

  // extend UVDomain to allow for tolerances
//  SmVector2d sDuv(d3dTolerance/m_vUVScale.x, d3dTolerance/m_vUVScale.y) ;
//  SmExtent2d sTolDomain(crPlaneUVDomain.GetMin()-sDuv,
//                        crPlaneUVDomain.GetMax()+sDuv) ;

  // Find thisPlane and otherPlane line/domain intersection interval
  double dTolUV = d3dTolerance / m_vUVScale.GetMaxDimension() ;
  crPlaneUVDomain.IntersectWithInfiniteLine(sUVStart,sUVVec,bFoundInterval,sPlaneIvl,&dTolUV);
  if (!bFoundInterval) 
    { return(SM_SUCCESS) ;
    }
  
  // find the intersection between intersection result and input segment
  // 1. degenerate overlap on ThisIvl max end
  if     (smos_Fabs(sThisIvl.GetMax()-sPlaneIvl.GetMin()) < SM_EFF_ZERO)
    { sIvl.SetMinMax(sThisIvl.GetMax(),sThisIvl.GetMax()) ;
    }
  // 2. degenerate overlap on ThisIvl min end
  else if(smos_Fabs(sThisIvl.GetMin()-sPlaneIvl.GetMax()) < SM_EFF_ZERO)
    { sIvl.SetMinMax(sThisIvl.GetMin(),sThisIvl.GetMin()) ;
    }
  // 3. intersection of the two intervals      
  else if(SM_ERR == sThisIvl.Intersect(sPlaneIvl,sIvl))
    {
      // 4. when intervals don't intersect - no intersection
      bFoundInterval = FALSE ;
      return(SM_SUCCESS) ;
    }

  // set outputs
  SmPoint3d rStart, rEnd ;
  rStart = (1-sIvl.GetMin()) * rStartPt + sIvl.GetMin() * rEndPt ;
  rEnd   = (1-sIvl.GetMax()) * rStartPt + sIvl.GetMax() * rEndPt ;

  rStartTrim = rStart ;
  rEndTrim   = rEnd ;

  // all done
  return(SM_SUCCESS) ;

} // end SmPlane::TrimPlaneLineToPlaneDomain

/*******************************************************************//**
PURPOSE: Static helper for TrimCurveToPlaneDonain(): Test whether
   a line-curve intersection is a grazing (tangent) intersection.

NOTES:  
***********************************************************************/
static SmBoolean sm_IsGrazingIntersection( const SmCurve* cpCurve,
                                                 double dCurveParam,
                                           const SmLine* cpLine,
                                                 double dLineParam )
{
  // Method: check tangents.  If parallel, then as long as the curve
  // has any curvature, it's a graze.
  SmVector3d sCurveEval[3];
  SmVector3d sLineEval[2];
  cpCurve->Evaluate( dCurveParam, 2, FALSE, sCurveEval );
  cpLine ->Evaluate( dLineParam , 1, FALSE, sLineEval  );
  if ( ! sCurveEval[1].IsParallelTo( sLineEval[1] ) )
    { return FALSE; }

  // Check curvature.  If the 2nd deriv has a component that's not parallel
  // to the 1st deriv, there is curvature.
  // (Note, IsParallelTo() returns False for zero vectors; we would want True.)
  SmVector3d sCross( sCurveEval[1] * sCurveEval[2] );
  if ( sCross.LengthSquared() > SM_EFF_ZERO_SQ )
    { return TRUE; }

  return FALSE;

} // end sm_IsGrazingIntersection

/*******************************************************************//**
PURPOSE: Find and return all portions of the input curve
            that lie inside given plane domain.

NOTES:  
   We no longer break the curve at grazing intersections.
***********************************************************************/
SmStatus SmPlane::TrimCurveToPlaneDomain
  (const SmExtent2d &crPlaneUVDomain,    // in : Extent of this plane
   const SmCurve &crCurve,               // in : Curve to be trimmed
   double d3dTolerance,                  // in : max distance between curve and plane domain boundaries
                                         //      to count as an intersection.
   SmTArray<SmCurve *> &r3dCurves)       // out: Trimmed copies of all crCurve intervals
                                         //      that are within the trim boundaries.
                                         //      When crCurve lies completely within the
                                         //      square - a copy is made and placed in rCurves.
 const                                  
{
  // init output
  r3dCurves.ReSet() ;

  // locals
  ULONG ii, jj ;
  SmExtent1d sIvl = crCurve.GetNaturalInterval() ;
  SmSolutionArray sSolutions, sTSolutions ;

  // get Plane domain corners
  const SmContext *pContext = GetContext();
  SmPoint2d sUV ;
  SmPoint3d sP00, sP01, sP10, sP11 ;
  double minX = crPlaneUVDomain.GetMin().x ;
  double minY = crPlaneUVDomain.GetMin().y ;
  double maxX = crPlaneUVDomain.GetMax().x ;
  double maxY = crPlaneUVDomain.GetMax().y ;
  sUV.Set(minX, minY) ; EvaluatePoint(sUV, sP00) ;
  sUV.Set(minX, maxY) ; EvaluatePoint(sUV, sP01) ;
  sUV.Set(maxX, minY) ; EvaluatePoint(sUV, sP10) ;
  sUV.Set(maxX, maxY) ; EvaluatePoint(sUV, sP11) ;

  // boundary line segments - a PlaneSurface domain boundary is a line
  SmExtent1d sLineIvl(0.0,1.0) ;
  SmLine *sLines[4] ;
  SmLine sLine0(sP00, sP10-sP00, sLineIvl, (sP10-sP00).Length(), 3, pContext) ;
  SmLine sLine1(sP10, sP11-sP10, sLineIvl, (sP11-sP10).Length(), 3, pContext) ;
  SmLine sLine2(sP11, sP01-sP11, sLineIvl, (sP01-sP11).Length(), 3, pContext) ;
  SmLine sLine3(sP01, sP00-sP01, sLineIvl, (sP00-sP01).Length(), 3, pContext) ;
  sLines[0] = &sLine0 ;
  sLines[1] = &sLine1 ;
  sLines[2] = &sLine2 ;
  sLines[3] = &sLine3 ;

  // for every boundary line
  for(ii=0;ii<4;ii++)
    {
      // Intersect Curve with boundary lines
      crCurve.GlobalCurveIntersect(sIvl, *sLines[ii], sLines[ii]->GetNaturalInterval(),
                                   d3dTolerance, sTSolutions) ; 

      // accumulate the solutions - sorted by crCurve parameter
      SmGlobalSolver sGS ;
      sGS.SetSolutions(&sSolutions) ;
      sGS.SetNumVariables(1) ;
      for(jj=0;jj<sTSolutions.GetSize();jj++)
        {
          SmSolution &rSolution = sTSolutions[jj] ;

          // Skip grazing solutions.  [Boolean Restructure]
          if ( rSolution.m_eSolutionType == SM_ST_SINGLE_VALUE )
            { if ( sm_IsGrazingIntersection( &crCurve, rSolution.m_vStart[0],
                                           sLines[ii], rSolution.m_vStart[1] ) )
                { continue; }
            }

          sGS.AddSortedSolution(rSolution, SM_SK_BY_FIRST_PARAMETER, TRUE) ; 

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
          // draw 
          if(bDebugMe)
            {
              sTSolutions.Dump() ;
              sSolutions.Dump() ;

              SmFace *pFace  = (SmFace *)GetFace() ;
              SmEdge *pEdge  = (SmEdge *)crCurve.GetEdge() ;
              SmBrep *pBrep1 = pFace ? pFace->GetBrep() : NULL ; sm_GraphicsLoop() ;
              SmBrep *pBrep2 = pEdge ? pEdge->GetBrep() : NULL ; sm_GraphicsLoop() ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,1) ; crCurve.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 0,1,.5); sLines[ii]->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 1,0,0) ; sTSolutions.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(6,7, 0,0,1) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(8,9, 0,1,0) ; rSolution.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

        } // end sorting every solution into sSolutions
    } // end iter every boundary line

  // get all intervals to be turned into curves
  ULONG lIvlCount = 0 ;
  SmTArray<double> sStarts, sEnds ;
  SmTArray<ULONG>  sKeeps ;          // 0 = no, 1=yes, 2=unknown ;
  double dSParam, dEParam ;

  // for every solution
  for(ii=0;ii<sSolutions.GetSize();ii++)
    {
      // get curve interval from last boundary to start of this solution
      SmSolution &rSolution = sSolutions[ii] ;
      dSParam = lIvlCount == 0 ? sIvl.GetMin() : sEnds[lIvlCount-1] ;
      dEParam = rSolution.m_vStart.m_adParameters[0] ;

      // skip zero length intervals at the curve start, curve end, and PlaneCorner/curve intersections
      SmPoint3d sStartPoint[2], sEndPoint ;
      crCurve.Evaluate(dSParam, 1, TRUE, sStartPoint) ;
      crCurve.EvaluatePoint(dEParam, sEndPoint) ;
      double dDist  = sStartPoint[0].DistanceBetween(sEndPoint) ;
      double dSpeed = sStartPoint[1].Length() ;
      if(   SM_IS_ZERO_TO_TOL(dSParam - dEParam, 10.0 * d3dTolerance/smos_Max(1.0, dSpeed))
         && dDist < d3dTolerance)
        { continue ; }

      // add an interval with an unknown classification
      sStarts.Add(dSParam) ;
      sEnds.Add(dEParam) ;
      sKeeps.Add(2) ;       // unclassified interval
      lIvlCount++ ;

      // when the solution is a coincident length
      if(rSolution.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
        {
          // add another interval marked for keeping
          dSParam = dEParam ;
          dEParam = rSolution.m_vEnd.m_adParameters[0] ;
          sStarts.Add(dSParam) ;
          sEnds.Add(dEParam) ;
          sKeeps.Add(1) ;   // classify interval for keeping
          lIvlCount++ ;
        }
    } // end iter every solution extracting curve intervals

  // get interval from last solution end to curve end
  dSParam = lIvlCount > 0 ? sEnds[lIvlCount-1] : sIvl.GetMin() ;
  dEParam = sIvl.GetMax() ;

  // add non-zero length interval at the end of the curve 
  if(   lIvlCount == 0
     || !SM_IS_ZERO(dSParam - dEParam))
    {
      sStarts.Add(dSParam) ;
      sEnds.Add(dEParam) ;
      sKeeps.Add(2) ;       // unclassified interval
      lIvlCount++ ;
    }      

  // classify every interval unclassified interval
  SmPoint3d sMidPoint ;
  for(ii=0;ii<lIvlCount;ii++)
    {
      // get interval bounds
      dSParam = sStarts[ii] ;
      dEParam = sEnds[ii] ;

      // skip classified intervals
      if(sKeeps[ii] != 2) 
        { continue ; }

      // should be no zero length intervals
      SM_ASSERT(!SM_IS_ZERO(dSParam - dEParam)) ; 

      // drop curve interval midPoint to inside of trimmed plane
      double dT = ( dSParam + dEParam ) / 2.0;
      crCurve.EvaluatePoint( dT, sMidPoint );
      DropPointFast(crPlaneUVDomain, SM_SO_NORMALIZE, sMidPoint, NULL, d3dTolerance, NULL,
                    SM_SR_SINGLE, sTSolutions) ;
      if ( sTSolutions.GetSize() > 0 )
        {
          // If the point is on the boundary of the domain, use a different curve point.  [B627]
          SmPoint2d sPtUV( sTSolutions[0].m_vStart[0], sTSolutions[0].m_vStart[1] );
          if ( crPlaneUVDomain.IsPoint2dOnBoundary( sPtUV, 2*d3dTolerance ) ) // Tol: anywhere close.
            {
              double dFrac = 0.3579;  // Make it not quite the midpoint.
              double dParam = dSParam * dFrac  +  dEParam * (1.0-dFrac);
              crCurve.EvaluatePoint( dParam, sMidPoint );
              DropPointFast(crPlaneUVDomain, SM_SO_NORMALIZE, sMidPoint, NULL, d3dTolerance, NULL,
                            SM_SR_SINGLE, sTSolutions) ;
            }
        }
      
      // classify the interval
      SmBoolean bKeep = ( sTSolutions.GetSize() > 0 );
      sKeeps[ii]      = bKeep ;

    } // end iter every interval to classify it

  // build an output curve for every keeper interval
  SmCurve        *pNewCurve = NULL ;
  SmBSplineCurve *pNewBSP   = NULL ;
  for(ii=0;ii<lIvlCount;ii++)
    {
      // degenerate curves for tangency intersections
      if(   ii > 0
         && sKeeps[ii]   == 0
         && sKeeps[ii-1] == 0)
        {
          // Create a degenerate curve - at the curve point
          SmPoint3d sPoint ;
          crCurve.EvaluatePoint(sStarts[ii], sPoint) ;
          SE(SmBSplineCurve::CreateDegenerateCurve(*m_cpContext, 3,
                                                   sPoint,
                                                   pNewBSP));
          
          r3dCurves.Add(pNewBSP);

        } // end need degenerate curve check

      // skip intervals outside the square
      if(sKeeps[ii] == 0)
        { continue ; }

      // copy, trim, and add curve to output
      SM_ASSERT(sKeeps[ii] == 1) ;
      SmExtent1d sTrimIvl(sStarts[ii], sEnds[ii]) ;
      SE(crCurve.Copy(*m_cpContext, pNewCurve)) ;
      SE(pNewCurve->Trim(sTrimIvl)) ;  // may snap sIvl by tol to existing knots
      r3dCurves.Add(pNewCurve);
    } // end iter every interval keeping the inside square ones

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SmFace *pFace  = (SmFace *)GetFace() ;
      SmEdge *pEdge  = (SmEdge *)crCurve.GetEdge() ;
      SmBrep *pBrep1 = pFace ? pFace->GetBrep() : NULL ; sm_GraphicsLoop() ;
      SmBrep *pBrep2 = pEdge ? pEdge->GetBrep() : NULL ; sm_GraphicsLoop() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; crCurve.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 1,0,0) ; for(ii=0;ii<r3dCurves.GetSize();ii++)
                                   { r3dCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;
 
} // end SmPlane::TrimCurveToPlaneDomain

/*******************************************************************//**
PURPOSE: Determine if the plane is bounded.

NOTES: 
***********************************************************************/
SmBoolean SmPlane::IsBounded() const
{
  return( m_vAnalUVDomain.IsBounded() ) ;

} // end SmPlane::IsBounded

/*******************************************************************//**
PURPOSE: Static function to determine if a given nurbs is a plane.
   If it is a plane then one is created with the same parameterization
   as the input surface.  

NOTES:
   A plane is tested by its representation and its geometry.

   A plane is a degree 1 (when HIGHER_ORDER_USE_ANALYTICS is not defined) surface.
   and for each point at a set of test points
     1. its U and V surface tangents are perpendicular to one another.
     2. lies within tolerance of the plane
     3. the surface normal is parallel to the plane surface normal. 

***********************************************************************/
SmBoolean SmPlane::IsNurbSurfacePlane
  (const SmContext        & crContext,       // in : context for new object construction
   const SmBSplineSurface * pTestSurface,    // in : surface to examine
   SmPlane               *& rpPlane,         // out: new output surface when pTestSurface is a plane, otherwise NULL.
   double                   dToleranceScale) // in : Max 3d variation allowed for plane
{
#ifdef HIGHER_ORDER_USE_ANALYTICS
//if ( pTestSurface->GetDegree(SM_SP_U) > 3     // skip this [bd 5/5/09]
//    || pTestSurface->GetDegree(SM_SP_V) > 3 
//    || pTestSurface->GetNumberControlPoints(SM_SP_U) > 4 
//    || pTestSurface->GetNumberControlPoints(SM_SP_V) > 4)
//  { return FALSE; }
#else 
  if (   pTestSurface->GetDegree(SM_SP_U) > 1 
      || pTestSurface->GetDegree(SM_SP_V) > 1 
      || pTestSurface->GetNumberControlPoints(SM_SP_U) > 2 
      || pTestSurface->GetNumberControlPoints(SM_SP_V) > 2) 
    { return FALSE; }
  
#endif

  // init output
  rpPlane = NULL ;

  // surface locals
  SmExtent2d sUVDomain = pTestSurface->GetNaturalUVDomain();
  SmPoint3d sOrigin;
  SmVector3d sXAxis, sYAxis;
  SmPlane *pPlane = NULL ;

  // get surface throughPoint, tangents, and normal at natural domain upper corner
  SE(pTestSurface->Evaluate1stDerivatives(sUVDomain.Evaluate(1.0,1.0),TRUE,TRUE,sOrigin,sXAxis,sYAxis));
  SmVector3d sNormalAtMax = sXAxis * sYAxis;
  
  // pick angle tolerance
  double dAngleTol = dToleranceScale * 0.0001; // 1/10000 of a degree

  // not a plane - UTangent not perpendicular to VTangent
  if (!sXAxis.IsPerpendicularTo(sYAxis,dAngleTol)) 
    { return FALSE; }

  // pick spatial tolerances
  double dTol   = SM_EFF_ZERO_SQRT * (1.0 + sOrigin.GetMaxDimension());
  double dTolSq = dTol*dTol;

  // get surface throughPoint, tangents, and normal at natural domain lower corner
  SE(pTestSurface->Evaluate1stDerivatives(sUVDomain.Evaluate(0.0,0.0),TRUE,TRUE,sOrigin,sXAxis,sYAxis));
  SmVector3d sNormalAtOrigin = sXAxis * sYAxis;

  // not a plane - UTangent not perpendicular to VTangent
  if (!sXAxis.IsPerpendicularTo(sYAxis,dAngleTol)) 
    { return FALSE; }

  // not a plane - origin and max normal vectors not parallel
  if (!sNormalAtOrigin.IsParallelTo(sNormalAtMax,dAngleTol)) 
    { return FALSE; }

  // get normal near domain center
  SE(pTestSurface->EvaluateNormal(sUVDomain.Evaluate(0.45,0.56),TRUE,TRUE,sNormalAtMax));

  // not a plane - center and origin normal vectors not parallel
  if (!sNormalAtOrigin.IsParallelTo(sNormalAtMax,dAngleTol)) 
    { return FALSE; }

  // looks like a plane - one more test to go

  // set up plane origin and X/Y axis
  SmVector2d sUVScale(sXAxis.Length(),sYAxis.Length());
  SE(sXAxis.Unitize());
  SE(sYAxis.Unitize());

  // Move the origin to correspond to the zero [u=0, v=0] parameter.
  SmPoint3d sNewOrigin =   sOrigin
                         - sUVDomain.GetMin().x * sXAxis * sUVScale.x
                         - sUVDomain.GetMin().y * sYAxis * sUVScale.y;

  // Force XAxis and YAxis to be nicely perpendicular
  //   without this you can get failures in SmAxis2Placement, and still return
  //   a non-null rpPlane value
  if ( smos_Fabs( sXAxis.Dot( sYAxis ) ) > SM_EFF_ZERO )
    { sYAxis = sXAxis * sYAxis * sXAxis; // Force them to be nicely perpendicular
      SER(sYAxis.Unitize());  
    }

  // construct temporary new plane (m_pNurb == NULL)
  pPlane = new (crContext) SmPlane(sNewOrigin,sXAxis,sYAxis,sUVScale,sUVDomain);
  if (!pPlane) { return FALSE; }
  
  // remember to delete the plane on exit
  SmObjDelete sClean(pPlane) ; 

  // Now test about 10 points along surface just to make sure.
  SmPoint2d sUVs[10];
  sUVs[0] = sUVDomain.Evaluate(0.3,0.4);
  sUVs[1] = sUVDomain.Evaluate(0.2,0.1);
  sUVs[2] = sUVDomain.Evaluate(0.6,0.4);
  sUVs[3] = sUVDomain.Evaluate(0.7,0.7);
  sUVs[4] = sUVDomain.Evaluate(0.8,0.9);
  sUVs[5] = sUVDomain.Evaluate(0.4,0.6);
  sUVs[6] = sUVDomain.Evaluate(0.9,0.2);
  sUVs[7] = sUVDomain.Evaluate(0.6,0.1);
  sUVs[8] = sUVDomain.Evaluate(0.1,0.8);
  sUVs[9] = sUVDomain.Evaluate(1.0,0.0);
  SmPoint3d sOrigPnt, sPlanePnt;

  // for every test point
  for (ULONG i=0; i<10; i++) 
    {
      // when any point is not a plane - return not a plane
      SER(pTestSurface->EvaluatePoint(sUVs[i],sOrigPnt));
      SER(pPlane->EvaluatePoint(sUVs[i],sPlanePnt));
      if (sOrigPnt.DistanceBetweenSquared(sPlanePnt) > dTolSq) 
        {
          return FALSE;
        }
    }

  // arrive here with valid plane

  // reparameterize the NURB so that AnalyticSurface->NurbDomain == InputSurface->NurbDomain
  pPlane->Reparameterize(pTestSurface->GetNaturalUVDomain()) ;

  // Copy pTestSurface attributes onto newPlane
  ((SmBSplineSurface *)pTestSurface)->Notify(SM_NO_COPY, pPlane, SM_NO_GET_OWNER(pPlane), SM_NO_GET_OWNER(pTestSurface));

  // all done - don't update the Nurb representation
  // set output
  sClean.Clear() ;
  rpPlane = pPlane ;

  return TRUE;

//        // obsolete - used to update the Nurb representation from the pTestSurface
//        //            don't do that, it's a mistake.
//        // place a copy of pTestSurface->m_pNurb into pPlane
//        SM_ASSERT(pPlane->m_pNurb == NULL) ;
//        pPlane->m_pNurb = sm_AllocateAndCopyNurbSurface(((SmBSplineSurface *)pTestSurface)->GetGwNurbPointer());
//      
//      #ifdef REDUCE_DEGREE_FOR_ANALYTICS
//        if (pPlane->GetDegree(SM_SP_U) > 1) 
//          {
//            pPlane->DegreeReduction(dTol,TRUE,SM_SP_U);
//            if (pPlane->GetDegree(SM_SP_U) > 1) 
//              { SE(SM_ERR); }
//          }
//      
//        if (pPlane->GetDegree(SM_SP_V) > 1) 
//          {
//            pPlane->DegreeReduction(dTol,TRUE,SM_SP_V);
//            if (pPlane->GetDegree(SM_SP_V) > 1) 
//              { SE(SM_ERR); }
//          }
//      #endif
//      
//        ((SmBSplineSurface*)pTestSurface)->Notify(SM_NO_COPY, pPlane, SM_NO_GET_OWNER(pPlane), SM_NO_GET_OWNER(pTestSurface));
//      
//      #ifdef SM_DEBUG_CODE
//      SmBoolean bDebugMe = FALSE;
//        if (bDebugMe) {
//            pTestSurface->Dump();
//            pPlane->Dump();
//            smgfx_Erase();
//            pTestSurface->Draw();
//            sm_GraphicsLoop();
//            smgfx_Erase();
//            pPlane->Draw();
//            sm_GraphicsLoop();
//        }
//      #endif
//      
//        // set output
//        sClean.Clear() ;
//        rpPlane = pPlane ;
//      
//        return TRUE;
// end obsolete section

} // end SmPlane::IsNurbSurfacePlane

/*******************************************************************//**
PURPOSE: Determine if a point is a singluar point and which direction
    does the singularity occur.

NOTES: Planes have no singularities 
   - set SingularDirection to NEITHER and return FALSE
***********************************************************************/
SmBoolean SmPlane::IsSingularity
(const SmPoint2d& crUVToTest,             // NotUsed: in : Point to test
    SmSurfParamType& reSingularDirection, // out: always set to SM_SP_NEITHER
    double            d3dTol,             // NotUsed: in : min dist between distinct 3d points
    SmBoolean         bPtTestOnly)        // NotUsed: in : TRUE = Pt tests only
    //    : FALSE= Pt and Surf tests
    //    : default:[FALSE] typical behavior except for some debug cases
    const
{
    SM_REF3(crUVToTest, d3dTol, bPtTestOnly);
    reSingularDirection = SM_SP_NEITHER;
    return FALSE;

} // end SmPlane::IsSingularity

SmBoolean SmPlane::IsSingularity
 (const SmPoint2d           & crUVToTest,          // NotUsed: in : Point to test
  ULONG                       lSingularities,      // NotUsed: in : SM_SS_NONE or one of: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX, SM_SS_UNKNOWN
  const SmTArray<SmPoint3d> * sSrfPolePoints,      // NotUsed: in : PolePoint[4] array, ordered :[UMinPole, VMinPole, UMaxPole, VMaxPole]
                                                   //        NonSingular side values set to SmPoint3d::SetUninitialized(),
  SmSurfParamType           & reSingularDirection, // out: always set to SM_SP_NEITHER
  double                      d3dTol,              // NotUsed: in : min dist between distinct 3d points
  SmBoolean                   bPtTestOnly)         // NotUsed: in : TRUE = Pt tests only
                                                   //    : FALSE= Pt and Surf tests
                                                   //    : default:[FALSE] typical behavior except for some debug cases
  const
{
  SM_REF5(crUVToTest, lSingularities, sSrfPolePoints, d3dTol, bPtTestOnly) ;
  reSingularDirection = SM_SP_NEITHER ;
  return FALSE;

} // end SmPlane::IsSingularity

/*******************************************************************//**
PURPOSE: Make a nurb corresponding to the current configuration
     of the plane.

NOTES:
  constructs a nurb with
    DegreeU = 1
    DegreeV = 1
    4 Control Points (one in each corner)
    2 mult=2 UKnots: [m_vAnalUVDomain.m_vMin.x, m_vAnalUVDomain.m_vMax.x]
    2 mult=2 VKnots: [m_vAnalUVDomain.m_vMin.y, m_vAnalUVDomain.m_vMax.y] 
***********************************************************************/
SmStatus SmPlane::MakeNurb
  ()
{
  // new NURB locals
  ULONG lUDeg = 1 ;
  ULONG lVDeg = 1 ;
  SmPoint3d sData[4];    SmTArray<SmPoint3d> sCtrlPts(4,sData,4);
  ULONG     alUMData[2]; SmTArray<ULONG>     sUKnotMult(2,alUMData,2);
  ULONG     alVMData[2]; SmTArray<ULONG>     sVKnotMult(2,alVMData,2);
  double    adUData[2];  SmTArray<double>    sUKnots(2,adUData,2);
  double    adVData[2];  SmTArray<double>    sVKnots(2,adVData,2);

  // evaluate and save the 4 3d corner positions of the m_vAnalUVDomain
  SmPoint3d sPnt;
  SER(EvaluatePointFast(m_vAnalUVDomain.Evaluate(0,0),sPnt)); sCtrlPts[0] = sPnt;
  SER(EvaluatePointFast(m_vAnalUVDomain.Evaluate(0,1),sPnt)); sCtrlPts[1] = sPnt;
  SER(EvaluatePointFast(m_vAnalUVDomain.Evaluate(1,0),sPnt)); sCtrlPts[2] = sPnt;
  SER(EvaluatePointFast(m_vAnalUVDomain.Evaluate(1,1),sPnt)); sCtrlPts[3] = sPnt;

  // set knot vectors to the corners of the m_vAnalUVDomain extent
  sUKnots[0] = m_vAnalUVDomain.GetMin().x;
  sUKnots[1] = m_vAnalUVDomain.GetMax().x;
  sVKnots[0] = m_vAnalUVDomain.GetMin().y;
  sVKnots[1] = m_vAnalUVDomain.GetMax().y;

  // set endPoint knot multiplicities
  sUKnotMult[0] = 2;    sUKnotMult[1] = 2;
  sVKnotMult[0] = 2;    sVKnotMult[1] = 2;

  // create temporary planar Bspline surface
  SmBSplineSurface *pTmp = NULL ;
  SER(SmBSplineSurface::CreateCanonical(*GetContext(),lUDeg,lVDeg,
                                        sCtrlPts,SM_SF_PLANE_SURF,sUKnotMult,sVKnotMult,
                                        sUKnots,sVKnots,SM_KT_UNSPECIFIED,NULL,NULL,pTmp));
  SmPlane *pTmpPlane = (SmPlane*)pTmp;
  SmObjDelete sClean(pTmp);

  // Swap nurbs - old nurb is deleted when temporary pTmp goes out of scope
  gw_SURFACE *pTmpNurb = m_pNurb;
  m_pNurb              = pTmp->GetGwNurbPointer();
  pTmpPlane->m_pNurb   = pTmpNurb;
  return SM_SUCCESS;

} // end SmPlane::MakeNurb

/*******************************************************************//**
PURPOSE: Find the UV coordinate of a point projected into a plane.
    This function assumes that the plane is infinite.

NOTES: 
***********************************************************************/
SmStatus SmPlane::ProjectPointToUVDomain
  (const SmPoint3d & cr3DPoint,           // in : target point to project
   SmPoint2d & rUVPoint)                  // out: UV param of projected point
  const
{
  const SmVector3d &rVec  = cr3DPoint - m_vPosition.GetOriginRef();
  double     dUProjection = m_vPosition.GetXAxisRef().Dot(rVec);
  double     dVProjection = m_vPosition.GetYAxisRef().Dot(rVec);
  rUVPoint.x = dUProjection / m_vUVScale.x;
  rUVPoint.y = dVProjection / m_vUVScale.y;

  // all done
  return SM_SUCCESS;

} // end SmPlane::ProjectPointToUVDomain

/*******************************************************************//**
PURPOSE: Reparameterize the NURB domain of a B-Spline Surface

NOTES: This method is only available for users with NLib
***********************************************************************/
SmStatus SmPlane::Reparameterize(const SmExtent2d & crTrimDomain )                                        
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Get new Analytic parameters
  double     dScaleX   = (m_vUVScale.x * m_vAnalUVDomain.XLength()) / crTrimDomain.XLength() ;
  double     dScaleY   = (m_vUVScale.y * m_vAnalUVDomain.YLength()) / crTrimDomain.YLength() ;
  SmVector3d sPointMax =   m_vPosition.GetOriginRef() 
                         + m_vUVScale.x * m_vPosition.GetXAxisRef() * m_vAnalUVDomain.GetUMax()
                         + m_vUVScale.y * m_vPosition.GetYAxisRef() * m_vAnalUVDomain.GetVMax() ;
  SmVector3d sPointMin =   m_vPosition.GetOriginRef() 
                         + m_vUVScale.x * m_vPosition.GetXAxisRef() * m_vAnalUVDomain.GetUMin()
                         + m_vUVScale.y * m_vPosition.GetYAxisRef() * m_vAnalUVDomain.GetVMin() ;
  SmVector3d sOriginMax =   sPointMax 
                          - dScaleX * m_vPosition.GetXAxisRef() * crTrimDomain.GetUMax()
                          - dScaleY * m_vPosition.GetYAxisRef() * crTrimDomain.GetVMax() ;

#ifdef SM_DEBUG_CODE // check new analytic domain accuracy
  SmVector3d sOriginMin =   sPointMin 
                          - dScaleX * m_vPosition.GetXAxisRef() * crTrimDomain.GetUMin()
                          - dScaleY * m_vPosition.GetYAxisRef() * crTrimDomain.GetVMin() ;
  double dDistErr = sOriginMax.DistanceBetweenSquared(sOriginMin) ; 
  if(dDistErr > SM_EFF_ZERO_SQ * (1 + sOriginMax.GetMaxDimension()) * (1 + sOriginMax.GetMaxDimension()))
    {
      SM_ASSERT_MSG(SM_ERR, _T("SmPlane::Reparameterize error in setting new Origin")) ; 
    }
#endif // SM_DEBUG_CODE

  // new NURB locals
  NL_RECTANGLE R;

  R.ul = crTrimDomain.GetMin().x;
  R.ur = crTrimDomain.GetMax().x;
  R.vb = crTrimDomain.GetMin().y;
  R.vt = crTrimDomain.GetMax().y;

  NL_SURFACE * surP = GetOrCreateGwNurbPointer();
  NER(surP);

  // set NURB domain parameterization - no change in shape
  N_SrfReparamToInterval(surP,R, NL_UVDIR); 

  // Set Analytic domain parameterization - no change in shape
  m_vAnalUVDomain = crTrimDomain ;
  m_vUVScale.Set(dScaleX, dScaleY) ;

  // done with modifications
  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmPlane::Reparameterize

/*******************************************************************//**
PURPOSE: Transpose STEP definition.

NOTES: 
***********************************************************************/
void SmPlane::ToggleSwapUVBit() 
{
  // switch defintion so that
  // Plane(u,v) = SwapPlane(v,u) 
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_ASSERT_VALID(this) ;
      this->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // m_vPosition swaps XAxis with YAxis
  SmVector3d sOrigin = m_vPosition.GetOrigin() ;
  SmVector3d sXAxis  = m_vPosition.GetXAxis() ;
  SmVector3d sYAxis  = m_vPosition.GetYAxis() ;
  m_vPosition.SetCanonical(sOrigin, sYAxis, sXAxis) ;
  
  // transpose the m_vAnalUVDomain
  m_vAnalUVDomain.Transpose() ;
  
  // Swap the scale factors
  SM_SWAP(double, m_vUVScale.x, m_vUVScale.y) ; 

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      // can't assert valid here because NaturalUVDomain is not yet swapped
      // SM_ASSERT_VALID(this) ;
      this->Dump() ;
    }
#endif // SM_DEBUG_CODE

} // end SmPlane::ToggleSwapUVBit
   

/*******************************************************************//**
PURPOSE: Scale and transform a Plane surface.

NOTES: The NURBS gets transformed in any case
       If there is a problem with the analytic, we return an error
       The caller should keep in mind that the NURBS was already transformed
NOTES: Scaling is not allowed on analytical surfaces.
***********************************************************************/
SmStatus SmPlane::Transform
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

  // apply m_pNurb
  SER(SmBSplineSurface::Transform(crRotateNMove,cpOptScale));
 
  // when scaling
  if(cpOptScale && ! (   cpOptScale->x == 1.0 
                      && cpOptScale->y == 1.0 
                      && cpOptScale->z == 1.0)) 
    {
      // scaling must be isotropic
      if(   !SM_ARE_SAME(cpOptScale->x,cpOptScale->y)  
         || !SM_ARE_SAME(cpOptScale->x,cpOptScale->z) ) 
        {
          ERR_MSG(_T("Unable to scale analytical surfaces\n"));
          SER(SM_ERR);
        }
    } // end scaling check
 
  // locals
  SmPlane         * pPlane   = NULL;
  const SmContext * pContext = GetContext();
  SmBSplineSurface  sBSS(m_pNurb,TRUE,pContext) ;
  // sBSS.SetContext(pContext);
 
  // check transformed m_pNurb for planarity and gen transformed Plane parameters
  if (!SmPlane::IsNurbSurfacePlane(*pContext,&sBSS,pPlane)) 
    {
      if (!SmPlane::IsNurbSurfacePlane(*pContext,&sBSS,pPlane,10.0)) 
        {
          if (!SmPlane::IsNurbSurfacePlane(*pContext,&sBSS,pPlane,100.0)) 
            {
              if (!SmPlane::IsNurbSurfacePlane(*pContext,&sBSS,pPlane,1000.0)) 
                {
                  SER(SM_ERR); // Something wrong here if scaling a plane does
                  // not produce a plane.
                }
            }
        }
    } // end transformed m_pNurb planartiy check
  SmObjDelete sClean(pPlane);
 
  // save the transformed plane parameters
  m_vPosition     = pPlane->m_vPosition;
  m_vAnalUVDomain = pPlane->m_vAnalUVDomain;
  m_vUVScale      = pPlane->m_vUVScale;
 
  // all done
  return SM_SUCCESS;

} // end SmPlane::Transform

/*******************************************************************//**
PURPOSE: Trim the plane with the given domain.

NOTES: 
  This function preserves the input surface geometry and its
  parameterization at the domain corners exactly, however the surface's 
  parameterization between domain corners may vary slightly but by
  amounts easily larger than reasonable tolerance sizes.  

  As such, existing Edgeuse->UVTrimCurves that reference the surface
  being trimmed should be deleted and rebuilt after this call.
***********************************************************************/
SmStatus SmPlane::TrimWithDomain
  (SmExtent2d & rTrimDomain)
{
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    SmExtent2d sNaturalDomain = GetNaturalUVDomain();

    // Make sure trim domain in inside of natural domain
    if (!rTrimDomain.IsContainedBy(sNaturalDomain, SM_EFF_ZERO_PARAM))
    {
        SER(SM_ERR);
    }

    // See if no trimming necessary - just return
    if (sNaturalDomain.IsContainedBy(rTrimDomain, SM_EFF_ZERO_PARAM))
    {
        return SM_SUCCESS;
    }

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
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

    SER(AdjustSTEPUVDomain(rTrimDomain));

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
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) { pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 1, 2, 0, 0, 0 ); if(pFace) { pFace->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 1,0,0) ; sSrfSrfGap.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
        
    return SM_SUCCESS;

} // end SmPlane::TrimWithDomain

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmPlane.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmPlane::GetMemoryUsed      // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,  // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)          // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + m_pNurb memory
  rlMemoryAllocated =   sizeof(*this) 
                      + sm_ComputeNurbSurfaceSize(m_pNurb) ;
  // + attribute memory
  ULONG lThisAllocated ;
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
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

} // end SmPlane::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertPlane_list[] =
{
 /*  0 */ {SM_AT_PARAMETERIZATION, _T("Domain X"),     _T("m_vAnalUVDomain should have length in X") },
 /* 01 */ {SM_AT_PARAMETERIZATION, _T("Domain Y"),     _T("m_vAnalUVDomain should have length in Y") },
 /* 02 */ {SM_AT_PARAMETERIZATION, _T("Equal Domain"), _T("sNaturalUVDomain should be equal to m_vAnalUVDomain") },
 /* 03 */ {SM_AT_SCALE,            _T("Scale"),        _T("m_vUVScale should be positive in X") },
 /* 04 */ {SM_AT_SCALE,            _T("Scale"),        _T("m_vUVScale should be positive in Y") },
 /* 05 */ {SM_AT_GEOMETRIC,        _T("Equal Shapes"), _T("Analytic and Nurb shapes should be equal") }
} ;   


/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmPlane::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ; 
  // init rtn value
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmBSplineSurface::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // m_vPosition is valid
  bRtn &= m_vPosition.AssertValid(pAList) ;

  // 0-1. AnalUVDomain Check - its has a nonZero area
  bRtn &= SM_ASSERT_VALUE_REPORT(0, SM_LEVEL_0, (m_vAnalUVDomain.XLength() > SM_EFF_ZERO), SM_EFF_ZERO,m_vAnalUVDomain.XLength(), _T("")) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(1, SM_LEVEL_0, (m_vAnalUVDomain.YLength() > SM_EFF_ZERO), SM_EFF_ZERO,m_vAnalUVDomain.YLength(), _T("")) ;

  if(m_pNurb)
    {
      SmExtent2d sNaturalUVDomain = GetNaturalUVDomain() ;

      // 2. Check AnalUVDomain equal NurbUVDomain
      bRtn &= SM_ASSERT_VALUE_REPORT(2, SM_LEVEL_0, (sNaturalUVDomain.AreEqual(m_vAnalUVDomain, SM_EFF_ZERO)), SM_EFF_ZERO, SM_UNDEF_DOUBLE, _T("")) ;
    
      // 4. Check for Anal/Nurb equivalent shapes
      SmPoint2d sUV ;
      SmPoint3d sAnalPt, sNurbPt ;
          
      // Domain min corner
      sUV.Set(sNaturalUVDomain.GetUMin(), sNaturalUVDomain.GetVMin()) ;           
      EvaluatePoint( sUV, sAnalPt );
      SmBSplineSurface::EvaluatePoint( sUV, sNurbPt );
      double dScaledZero = SM_EFF_ZERO * (1.0 + sAnalPt.GetMaxDimension()) ;
      bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, (sAnalPt.CloserThan(dScaledZero, sNurbPt)), dScaledZero,sAnalPt.DistanceBetween(sNurbPt), _T("")) ;

      // Domain max corner
      sUV.Set(sNaturalUVDomain.GetUMax(), sNaturalUVDomain.GetVMax()) ; 
      EvaluatePoint( sUV, sAnalPt );
      SmBSplineSurface::EvaluatePoint( sUV, sNurbPt );
      dScaledZero = SM_EFF_ZERO * (1.0 + sAnalPt.GetMaxDimension()) ;
      bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, (sAnalPt.CloserThan(dScaledZero, sNurbPt)), dScaledZero,sAnalPt.DistanceBetween(sNurbPt), _T("")) ;
      if(bRtn == FALSE)
        { // place for a debug break
          // SM_ASSERT(bRtn) ;
        }

    } // end has a Nurb description check

  // 3-4. positive definite m_vUVScale
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, (m_vUVScale.x > SM_EFF_ZERO), SM_EFF_ZERO, m_vUVScale.x, _T("")) ;
  bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0, (m_vUVScale.y > SM_EFF_ZERO), SM_EFF_ZERO, m_vUVScale.y, _T("")) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
    }
#endif // SM_DEBUG_CODE
  // all done
#ifdef GWC
  if(bRtn == FALSE)
    {
      WARN(_T("Bad SmPlane")) ;
    }
#endif // GWC

  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmPlane::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmPlane::AssertHeal
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
//       return ( SmBSplineSurface::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmPlane::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmPlane::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Write SmPlane to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmPlane::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();
  
  // locals  
  const SmPoint3d  &rOrigin = m_vPosition.GetOriginRef() ;
  const SmVector3d &rX      = m_vPosition.GetXAxisRef() ;
  const SmVector3d &rY      = m_vPosition.GetYAxisRef() ;
  if (eType == SM_ASCII) 
    {
      rFileOut << rOrigin.x << " " << rOrigin.y << " " << rOrigin.z  << " SmPlane Placement Origin \n";
      rFileOut << rX.x      << " " << rX.y      << " " << rX.z       << " SmPlane Placement X Axis \n";
      rFileOut << rY.x      << " " << rY.y      << " " << rY.z       << " SmPlane Placement Y Axis \n";
      rFileOut <<        m_vAnalUVDomain.GetMin().x 
               << " " << m_vAnalUVDomain.GetMin().y 
               << " " << m_vAnalUVDomain.GetMax().x 
               << " " << m_vAnalUVDomain.GetMax().y                  << " SmPlane AnalUVDomain \n";
      rFileOut << m_vUVScale.x << " " << m_vUVScale.y                << " SmPlane UV Scale \n";
    }
  else 
    {
      SER(rDB.WriteDouble(rOrigin.x));
      SER(rDB.WriteDouble(rOrigin.y));
      SER(rDB.WriteDouble(rOrigin.z));

      SER(rDB.WriteDouble(rX.x));
      SER(rDB.WriteDouble(rX.y));
      SER(rDB.WriteDouble(rX.z));

      SER(rDB.WriteDouble(rY.x));
      SER(rDB.WriteDouble(rY.y));
      SER(rDB.WriteDouble(rY.z));

      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMin().x));
      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMin().y));
      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMax().x));
      SER(rDB.WriteDouble(m_vAnalUVDomain.GetMax().y));

      SER(rDB.WriteDouble(m_vUVScale.x));
      SER(rDB.WriteDouble(m_vUVScale.y));
    }

  // output the parent
  SmBSplineSurface::WriteToDB(rDB, lDBVersionNumber) ; 

  // all done
  return SM_SUCCESS;

} // end SmPlane::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmPlane from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmPlane::ReadFromDB
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
         || rpNewSurface->IsKindOf(SmPlane_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmPlane *pPlane =   (rpNewSurface == NULL)
                    ? new (crContext) SmPlane()
                    : (SmPlane *)rpNewSurface ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  SmPoint3d sOrig ;
  SmVector3d sX, sY ; 
  SmPoint2d sAnalMin, sAnalMax ;
  SmAxis2Placement sPosition;    
  SmExtent2d       sAnalUVDomain;
  SmVector2d       sUVScale;     

  if (eType == SM_ASCII) 
    {
      rFileIn >> sOrig.x >> sOrig.y >> sOrig.z ;                         rDB.GoToNextLine() ;
      rFileIn >> sX.x >> sX.y >> sX.z ;                                  rDB.GoToNextLine() ;
      rFileIn >> sY.x >> sY.y >> sY.z ;                                  rDB.GoToNextLine() ;
                                                                         
      rFileIn >> sAnalMin.x >> sAnalMin.y >> sAnalMax.x >> sAnalMax.y  ; rDB.GoToNextLine() ;

      rFileIn >> sUVScale.x >> sUVScale.y  ;                             rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble(sOrig.x)) ;
      SER(rDB.ReadDouble(sOrig.y)) ;
      SER(rDB.ReadDouble(sOrig.z)) ;

      SER(rDB.ReadDouble(sX.x)) ;
      SER(rDB.ReadDouble(sX.y)) ;
      SER(rDB.ReadDouble(sX.z)) ;

      SER(rDB.ReadDouble(sY.x)) ;
      SER(rDB.ReadDouble(sY.y)) ;
      SER(rDB.ReadDouble(sY.z)) ;

      SER(rDB.ReadDouble(sAnalMin.x)) ;
      SER(rDB.ReadDouble(sAnalMin.y)) ;

      SER(rDB.ReadDouble(sAnalMax.x)) ;
      SER(rDB.ReadDouble(sAnalMax.y)) ;

      SER(rDB.ReadDouble(sUVScale.x)) ;
      SER(rDB.ReadDouble(sUVScale.y)) ;
    }

  // load the plane
  pPlane->m_vPosition.SetCanonical(sOrig, sX, sY) ;
  pPlane->m_vAnalUVDomain.SetMinMax(sAnalMin, sAnalMax) ; 
  pPlane->m_vUVScale = sUVScale ;

  // read the parent object
  SmSurface *pSurface = pPlane ; 
  SER(SmBSplineSurface::ReadFromDB(SmBSplineSurface_TYPE, rDB, crContext, pSurface, lDBVersionNumber)) ;

  // all done
  rpNewSurface = pPlane ; 
  return SM_SUCCESS;

} // end SmPlane::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmPlane::IsKindOf( SM_TYPE t ) const
{
  return ((SmPlane_TYPE == t) ? TRUE : SmBSplineSurface::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmPlane::Dump( ULONG i ) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];

  smos_sprintf(sBuff,_T("\n%ld "), i);
  smos_WriteBuffer(sBuff);

  this->SmPlane::Dump();

} // end SmPlane::Dump


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmPlane::Dump( const TCHAR * message ) const
{
     TCHAR sBuff[SM_TBLOCK_SIZE];
     smos_sprintf(sBuff,_T("\n%s "), message);
     smos_WriteBuffer(sBuff);
     this->SmPlane::Dump();

} // end SmPlane::Dump

/*******************************************************************//**
PURPOSE: Dump SmPlane surface data out for debugging.

NOTES: 
***********************************************************************/
void SmPlane::Dump( void ) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmPlane::Dump()")) ;

  // report Cache data
  SmSurface::Dump(FALSE) ;

  smos_sprintf(sBuff,       _T("\nSmPlane = 0x%p"),this);
  smos_sprintf(sBuffForFile,_T("\nSmPlane = %s"), _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_WriteBuffer(_T("\n  AnalUVDomain  = ")); m_vAnalUVDomain.Dump();
  smos_WriteBuffer(_T("  NurbDomain    = "));   GetNaturalUVDomain().Dump() ;

  smos_sprintf(sBuff,       _T("  UScale = %16.16lf, VScale = %16.16lf"), m_vUVScale.x, m_vUVScale.y) ;
  smos_WriteBuffer(sBuff, sBuff);

  smos_WriteBuffer(_T("\n  Transform       = ")) ;  m_vPosition.Dump();
  smos_WriteBuffer(_T("\n  Nurb Surface    = ")) ;  SmBSplineSurface::Dump();

  smos_WriteBuffer(_T(" End SmPlane::Dump()\n")) ;

} // end SmPlane::Dump

/*******************************************************************//**
PURPOSE: Make a line, known to be in the target plane, trimmed
            to the target Plane's UVDomain.

NOTES:  Sets bFoundInterval = TRUE when given line has a segment
                 within the plane's give UVDomain.  Set to FALSE for
                 NULL and degenerate curves.
***********************************************************************/
SmBSplineCurve *SmPlane::MakeNewPlaneLine
  (const SmContext  &crContext,         // in : context for new object construction
   const SmExtent2d &crPlaneUVDomain,   // in : plane's UVDomain
   const SmPoint3d  &crStartPt,         // in : Line Start Point
   const SmPoint3d  &crEndPt,           // in : Line End Point
   double            dTol,              // in : min 3d distance between distinct points
   SmBoolean &rbFoundInterval)          // out: TRUE = Returned a Line curve
 const                                  //      FALSE= Returned no Curve or a Degenerate Curve
{
  // init output
  SmBSplineCurve *pCrv = NULL ;
  rbFoundInterval = FALSE ;

  // trim line endPoints to plane UVDomain
  SmPoint3d sT0, sT1 ;
  TrimPlaneLineToPlaneDomain(crPlaneUVDomain, crStartPt, crEndPt, dTol, sT0, sT1, rbFoundInterval) ;

  // when no part of the line is on the plane
  if(!rbFoundInterval) { return pCrv ; }

  // get line length
  double dLineDist = sT0.DistanceBetween(sT1) ;

  // when line is degenerate
  if(dLineDist < dTol) { SmStatus sErr = SmBSplineCurve::CreateDegenerateCurve(crContext,3,sT0,pCrv);
                         SM_ASSERT(pCrv != NULL) ;
                         if(sErr == SM_ERR) { return(NULL) ; }
                         return(pCrv) ;
                       }

  // when line has a finite length
  SmVector3d sLineVec = sT1 - sT0 ;
  sLineVec.Unitize() ;    
  rbFoundInterval = TRUE ;
  pCrv            = new (crContext) SmLine(sT0, sLineVec, SmExtent1d(0.0,1.0), dLineDist, 3);

  // all done
  return(pCrv) ;

} // end SmPlane::MakeNewPlaneLine   

/*******************************************************************//**
PURPOSE: Make an ellipse or circle, known to be in the target plane, trimmed
            to the target Plane's and Arc's Domains. The function can return 
            from 0 to 5 different arcs or degenerate points.

NOTES:  Sets bFoundInterval = TRUE when given line has a segment
                 within the plane's give UVDomain.  Set to FALSE for
                 NULL and degenerate curves.
***********************************************************************/
SmStatus SmPlane::MakeNewPlaneEllipse
  (const SmContext  &crContext,         // in : context for new object construction
   const SmExtent2d &crPlaneUVDomain,   // in : plane's UVDomain
   const SmPoint3d  &crCircCenter,      // in : Center of circle
   double            dCircRadiusX,      // in : radius of circle at X Axis
   double            dCircRadiusY,      // in : radius of circle at Y Axis
   const SmVector3d &crCircXAxis,       // in : vector marking start/end of circle
   const SmVector3d &crCircYAxis,       // in : vector marking circle 90 degree point
   const SmExtent1d &crCircAnalIvl,     // in : domain of circle [0 to 360]
   double            dTol,              // in : min 3d distance between distinct points
   SmTArray<SmBSplineCurve *> &rCrvs)   // out: 0 to 5 new Circle, Ellipse, or DegenerateCurve curves
 const
{
  // init output
  rCrvs.ReSet() ;

  // locals
  ULONG ii, jj ;
  SmBoolean bCircle = SM_ARE_SAME(dCircRadiusX, dCircRadiusY) ;
  SmTArray <SmPoint3d> sTrimPoints ;    // array of circ/plane boundary xsectPoints
  SmTArray <SmPoint2d> sTrimUVs ;       // plane UVPoint,                    for each xsectPoint
  SmTArray <SmBoolean> sTrimEntering ;  // TRUE = entering, FALSE = exiting, for each xsectPoint
  SmTArray <ULONG>     sTrimSide ;      // isoCurve index,                   for each xsect point
  SmBSplineCurve       *pCrv ;

  // get circle center and XAxis to plane UVPoints
  SmPoint2d  sCenterUV ;
  SmVector3d sCircZAxis  = crCircXAxis * crCircYAxis ;
  SmVector3d sCircXPoint = crCircCenter + crCircXAxis ;
  ProjectPointToUVDomain(crCircCenter, sCenterUV) ;
  sCircZAxis.Unitize() ;

  // get circle's yAxis using plane normal as the positive direction
  SmVector3d sPlaneNormal, sCircYAxisForPlaneNormal ;
  EvaluateNormal(sCenterUV, TRUE, TRUE, sPlaneNormal) ;
  sCircYAxisForPlaneNormal = sPlaneNormal * crCircXAxis ;

  // note when circle axis and plane Normal are in opposite directions
  SmBoolean bOpposite = sCircZAxis.Dot(sPlaneNormal) < 0.0 ;

  // zero radius ellipse
  if(   dCircRadiusX < dTol
     || dCircRadiusY < dTol)
    {
      // when center is on plane
      if(   dCircRadiusX < dTol
         && dCircRadiusY < dTol
         && crPlaneUVDomain.ContainsPoint2d(sCenterUV))
        {
          // create a degenerate curve to mark the point
          SmStatus sErr = SmBSplineCurve::CreateDegenerateCurve(crContext,3,crCircCenter,pCrv);
          SM_ASSERT(pCrv != NULL) ;
          if(sErr == SM_ERR) { return SM_ERR ; }
          rCrvs.Add(pCrv) ;
          return SM_SUCCESS ;
       }

      // else make a degenerate line and quit
      SmPoint3d sLineStartPt, sLineEndPt ;
      if(dCircRadiusX < dTol) { sLineStartPt = crCircCenter - dCircRadiusY * crCircYAxis ;
                                sLineEndPt   = crCircCenter + dCircRadiusY * crCircYAxis ;
                              }
      else                    { sLineStartPt = crCircCenter - dCircRadiusX * crCircXAxis ;
                                sLineEndPt   = crCircCenter + dCircRadiusX * crCircXAxis ;
                              }
      SmBoolean bFoundIvl ;
      pCrv = MakeNewPlaneLine(crContext, crPlaneUVDomain, sLineStartPt, sLineEndPt, dTol, bFoundIvl) ;
      if(bFoundIvl) { rCrvs.Add(pCrv) ; } 
      return SM_SUCCESS ;
    } // end zero radius check

  // for all 4 plane boundary lines
  double dMinX = crPlaneUVDomain.GetMin().x ;
  double dMaxX = crPlaneUVDomain.GetMax().x ;
  double dMinY = crPlaneUVDomain.GetMin().y ;
  double dMaxY = crPlaneUVDomain.GetMax().y ;
  SmBoolean bTangent       = FALSE ;
  SmBoolean bTangentInside = FALSE ;
  SmPoint3d sTangentPt, sTangentVec ;
  SmPoint2d sTangentUV ;
  for(ii=0;ii<4;ii++)
    {
      SmPoint2d sPlaneStartUV, sPlaneEndUV ;
      SmPoint3d sPlaneStartPt, sPlaneEndPt, sPlaneVec, sBiNormIn ;
      double dVecY = 0.0, dVecX = 0.0;

      // get Plane isoParamLine running in counter clockwise direction
      switch(ii)
        {
          // set   IsoCurveTangentUV    = [ dVecX dVecY]
          // note: IsoCurveBiNormInside = [-dVecY dVecX]
          case 0: sPlaneStartUV.Set(dMinX, dMinY) ; sPlaneEndUV.Set(dMaxX, dMinY) ; 
                  dVecX =  1.0 ; dVecY =  0.0 ;
                  break ;
          case 1: sPlaneStartUV.Set(dMaxX, dMinY) ; sPlaneEndUV.Set(dMaxX, dMaxY) ; 
                  dVecX =  0.0 ; dVecY =  1.0 ;
                  break ;
          case 2: sPlaneStartUV.Set(dMaxX, dMaxY) ; sPlaneEndUV.Set(dMinX, dMaxY) ; 
                  dVecX = -1.0 ; dVecY =  0.0 ;
                  break ;
          case 3: sPlaneStartUV.Set(dMinX, dMaxY) ; sPlaneEndUV.Set(dMinX, dMinY) ; 
                  dVecX =  0.0 ; dVecY = -1.0 ;
                  break ;

        } // end switch on ii

      // set isoLineVec
      EvaluatePointFast(sPlaneStartUV, sPlaneStartPt) ;
      EvaluatePointFast(sPlaneEndUV,   sPlaneEndPt) ;
      sPlaneVec = sPlaneEndPt - sPlaneStartPt ;
      sBiNormIn = sPlaneNormal * sPlaneVec ;

      // intersect the line with the ellipse
      ULONG lXSectCount = 0 ;
      double aParams[2] ;
      SmPoint3d sPt ;
      SmPoint2d sPtUV ;
      smgu_LineCoPlanarEllipseIntersect
            (sPlaneStartPt,                // in : plane isocurve start 3d point
             sPlaneVec,                    // in : plane isocurve StartToEnd 3d vector
             crCircCenter,                 // in : circle center point 3d
             crCircXAxis,
             dCircRadiusX,                 // in : circle 3d radius at XAXIS
             sCircYAxisForPlaneNormal,     // in : Y axis for CCW rotations within the plane
             dCircRadiusY,                 // in : circle 3d radius at XAXIS
             dTol, lXSectCount, aParams) ; // out: line params

      // when there are solutions
      if(lXSectCount > 0)
        {
          // Check for solutions within tol. (The intersection routine doesn't do that.)
          // If so, collapse them to the midpoint.  [B10]
          if( lXSectCount == 2 )
            {
              double dDist = smos_Fabs( aParams[1] - aParams[0] ) * sPlaneVec.Length();

              // Actually, check for the entire span between the intersections
              // being within tol of the line.
              double dRad  = smos_Min( dCircRadiusX, dCircRadiusY );
              if ( dDist < dRad * 0.1745 )  // About 10 deg: just to avoid a sqrt when not close.
                {
                  // Max dist from ellipse to line between the two solutions:
                  double dH = dRad - smos_Sqrt( dRad*dRad - dDist*dDist*0.25 );
                  if ( dH < dTol / 2.0 )
                    {
                      lXSectCount = 1;
                      aParams[0] = ( aParams[0] + aParams[1] ) / 2.0;
                    }
                }
            }

          // order aParams
          if(   lXSectCount == 2
             && aParams[0] > aParams[1])
            { 
              double dTmp = aParams[0] ;
              aParams[0]  = aParams[1] ;
              aParams[1]  = dTmp ;
            }

          // get the first solution point
          SmBoolean bInsideLine = (   0.0-SM_EFF_ZERO <= aParams[0] 
                                   && 1.0+SM_EFF_ZERO >= aParams[0]) ;
          sPt =   (1.0-aParams[0]) * sPlaneStartPt
                + (  aParams[0]  ) * sPlaneEndPt ;

          // when inside the line extent
          //   - save 1 tangent inside/outside state (only used if no other intersections)
          //     and all crossing intersections inside/outside states
          if(lXSectCount == 1) 
            { if(bInsideLine && bTangent == FALSE) 
              { bTangent   = TRUE ; 
                sTangentPt = sPt ; 
                // curve inside when DotProduct(SurfaceBoundaryBinorm,CircleRadiusVec) > 0
                // Binorm = [-dVecY, dVecX], CircleRadius = sCenterUV - sTangentUV.
                ProjectPointToUVDomain(sTangentPt, sTangentUV) ;
                bTangentInside =  (( -dVecY * (sCenterUV.x - sTangentUV.x)
                                     +dVecX * (sCenterUV.y - sTangentUV.y)) > 0.0)
                                 ? TRUE
                                 : FALSE ;
              }                                       
            }                                                     
          else // save two points
            {
              // save 1st point when inside the line extent
              if(bInsideLine) 
                { 
                  sTrimPoints.Add(sPt) ;
                  ProjectPointToUVDomain(sPt, sPtUV) ;
                  sTrimUVs.Add(sPtUV) ;
                  // entering when dot(BiNormInside,EllipseTangent) > 0.0
                  //   (P-C) = Rx*cos(a)*X + Ry*sin(a)*Y
                  //   Dot(P-C,X) = Rx*cos(a)
                  //   Dot(P-C,Y) = Ry*sin(a)
                  //    Tan  = -Rx*Sin(a)*X + Ry*cos(a)*Y
                  //    Tan  = -Rx/Ry *Ry*sin(a)*X + Ry/Rx*Rx *cos(a)*Y
                  //    Tan  = -Rx/Ry * Dot(P-C,Y) * x + Ry/Rx * Dot(P-C,X) * Y
                  sTangentVec =  -dCircRadiusX/dCircRadiusY * (sPt-crCircCenter).Dot(sCircYAxisForPlaneNormal) * crCircXAxis 
                                 +dCircRadiusY/dCircRadiusX * (sPt-crCircCenter).Dot(crCircXAxis) * sCircYAxisForPlaneNormal ;
                  SmBoolean bEntering = (sBiNormIn.Dot(sTangentVec) > 0.0)
                                        ? TRUE : FALSE ;
                  sTrimEntering.Add(bEntering) ;
                  sTrimSide.Add(ii) ; 
                }

              // save 2nd point when when inside the line extent
              bInsideLine = (   0.0-SM_EFF_ZERO <= aParams[1] 
                             && 1.0+SM_EFF_ZERO >= aParams[1]) ;
              sPt =   (1.0-aParams[1]) * sPlaneStartPt
                    + (  aParams[1]  ) * sPlaneEndPt ;
              if(bInsideLine) 
                { 
                  sTrimPoints.Add(sPt) ;
                  ProjectPointToUVDomain(sPt, sPtUV) ;
                  sTrimUVs.Add(sPtUV) ;
                  // entering when dot(BiNormInside,EllipseTangent) > 0.0
                  //   (P-C) = Rx*cos(a)*X + Ry*sin(a)*Y
                  //   Dot(P-C,X) = Rx*cos(a)
                  //   Dot(P-C,Y) = Ry*sin(a)
                  //    Tan  = -Rx*Sin(a)*X + Ry*cos(a)*Y
                  //    Tan  = -Rx/Ry *Ry*sin(a)*X + Ry/Rx*Rx *cos(a)*Y
                  //    Tan  = -Rx/Ry * Dot(P-C,Y) * x + Ry/Rx * Dot(P-C,X) * Y
                  sTangentVec =  -dCircRadiusX/dCircRadiusY * (sPt-crCircCenter).Dot(sCircYAxisForPlaneNormal) * crCircXAxis 
                                 +dCircRadiusY/dCircRadiusX * (sPt-crCircCenter).Dot(crCircXAxis) * sCircYAxisForPlaneNormal ;
                  SmBoolean bEntering = (sBiNormIn.Dot(sTangentVec) > 0.0)
                                        ? TRUE : FALSE ;
                  sTrimEntering.Add(bEntering) ;
                  sTrimSide.Add(ii) ; 
                }
            } // end 2 xSect branch
        } // end any xSect check
    } // end iter all 4 boundary lines

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
      sCircYAxisForPlaneNormal.Unitize() ;
      SmEllipse sEllipse(crCircCenter, crCircXAxis, crCircYAxis, crCircAnalIvl, dCircRadiusX, dCircRadiusY, 3, &crContext) ;
      SmPoint3d sPlaneCenter ;
      EvaluatePointFast(sCenterUV, sPlaneCenter) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(6,6) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 0,1,0) ; crCircCenter.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,8, 0,1,1) ; sPlaneCenter.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,9, 0,0,1) ; (dCircRadiusX * crCircXAxis).Draw(&crCircCenter) ; sm_GraphicsLoop() ;   
      smgfx_SetLook(4,9, 0,0,1) ; (dCircRadiusX * sCircZAxis).Draw(&crCircCenter) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5, 0,1,0) ; (dCircRadiusX * GetPosition().GetZAxis()).Draw(&crCircCenter) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; sEllipse.Draw() ;  sm_GraphicsLoop() ;
      for(ii=0;ii<sTrimPoints.GetSize();ii++)
        { SmPoint3d sPlanePoint ;
          EvaluatePointFast(sTrimUVs[ii],sPlanePoint) ;
          // blue = entering, red = exiting
          if(   (!bOpposite && sTrimEntering[ii] == TRUE)
             || ( bOpposite && sTrimEntering[ii] == FALSE)) { smgfx_SetLook(4,6, 0,0,1) ; } // blue = entering
          else                                                    { smgfx_SetLook(6,7, 1,0,0) ; } // red  = exiting
          sTrimPoints[ii].Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,9, 0,1,1) ; sPlanePoint.Draw() ; sm_GraphicsLoop() ;
        }
      sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // If there were no trim points, with the whole (untrimmed) ellipse,
  // then it's all either inside or outside the domain.
  //
  // Also do this in the one-intersection case: example, a half circle hitting
  // a corner of the plane at the end of the circle, all axis-aligned.
  // The circle is tangent to one of the two sides at the corner.  [B286]
  if(sTrimPoints.GetSize() < 2)
    {
      SmVector2d sCircleUV ; 
      SmVector3d sCirclePt ;

      // need to classify a circumference point when there are no tangent points
      // (hack - reuse bTangentInside to hold value.
      //  If bTangent is false, then so is bTangentInside at this point.)
      if ( ! bTangent )
        {
          sCirclePt = crCircCenter + dCircRadiusX * crCircXAxis ;
          ProjectPointToUVDomain(sCirclePt, sCircleUV) ; 
          bTangentInside = crPlaneUVDomain.ContainsPoint2d(sCircleUV) ;
        }

       // and the circle is in the plane
       if ( bTangentInside )
         {
           // build 1 full circle or ellipse
           if(bCircle) { pCrv = new (crContext) SmCircle(crCircCenter, 
                                                         crCircXAxis, crCircYAxis, crCircAnalIvl, 
                                                         dCircRadiusX, 3) ;
                       }
           else        { pCrv = new (crContext) SmEllipse(crCircCenter, 
                                                          crCircXAxis, crCircYAxis, crCircAnalIvl, 
                                                          dCircRadiusX, dCircRadiusY, 3) ;
                       }
           rCrvs.Add(pCrv) ;
           return SM_SUCCESS ;
         } // end contained circle case

       // or the circle is outside the plane and there is a tangent point
       if( bTangent )
         {
           // mark the tangency with a degenerate curve
           SmStatus sErr = SmBSplineCurve::CreateDegenerateCurve(crContext,3,sTangentPt,pCrv);
           SM_ASSERT(pCrv != NULL) ;
           if(sErr == SM_ERR) { return SM_ERR ; }
           rCrvs.Add(pCrv) ;
           return SM_SUCCESS ;
         } // end outside tangent case

       // there is no part of the circle in the square
       return SM_SUCCESS ;
    } // end no trim points check

  // arrive here when there are trim points
  // sTrimPoints is ordered in a CCW traversal of the plane ZAxis.
  // Make a trim curve from every entering xSectPoint to every exiting xSectPoint
  // Any trim curve that happens to contain the circles start/end point needs
  // to be broken up into two intervals.

  // for every trim point
  ULONG     lCount    = sTrimPoints.GetSize() ;
  SM_ASSERT(lCount % 2 == 0) ;
  for(ii=0;ii<lCount;ii++)
    {
      SM_ASSERT(   (   sTrimEntering[ii]           == TRUE  
                    && sTrimEntering[(ii+1)%lCount] == FALSE)
                || (   sTrimEntering[ii]           == FALSE 
                    && sTrimEntering[(ii+1)%lCount] == TRUE )) ;

      // skip exiting points 
      // - when plane and circ normals are opposite, entering and exiting swap
      if(   (!bOpposite && sTrimEntering[ii] == FALSE)
         || ( bOpposite && sTrimEntering[ii] == TRUE)) 
        { continue ; }

      // circle arc 3d start/end points
      SmPoint3d &rPoint1 = sTrimPoints[ii] ;
      SmPoint3d &rPoint2 = sTrimPoints[  bOpposite
                                       ? ((ii+lCount)-1)%lCount
                                       : (ii+1)%lCount] ;
      double dDist = rPoint2.DistanceBetween(rPoint1) ;

      // ellipse start angle Rad
      SmVector3d sVec    = rPoint1-crCircCenter;
      double     dX      = sVec.Dot(crCircXAxis);
      double     dY      = sVec.Dot(crCircYAxis);
      double dEnterAngRad = smos_ArcTangent2(dCircRadiusX/dCircRadiusY * dY, dX) ;

      // ellipse end angle Rad
      sVec    = rPoint2-crCircCenter;    
      dX      = sVec.Dot(crCircXAxis);
      dY      = sVec.Dot(crCircYAxis);
      double dExitAngRad  = smos_ArcTangent2(dCircRadiusX/dCircRadiusY * dY, dX) ;
      
      // ellipse start/stop angles deg with periodicity    
      double     dEnterAngDeg, dExitAngDeg ; 
      crCircAnalIvl.ContainsPeriodicValue(dEnterAngRad * 180.0 / SM_PI, 360.0, &dEnterAngDeg) ;  
      crCircAnalIvl.ContainsPeriodicValue(dExitAngRad  * 180.0 / SM_PI, 360.0, &dExitAngDeg) ;  

      // when circNorm is opposite to plane Norm Start/End will be reversed
      double dTolEnterDeg =   (dEnterAngDeg > dExitAngDeg + SM_EFF_ZERO) ? dEnterAngDeg - 360.0 
                            : (dEnterAngDeg > dExitAngDeg)               ? dExitAngDeg
                            : dEnterAngDeg ;
      SmExtent1d sArcIvl(dTolEnterDeg, dExitAngDeg) ;
      
      // trim arc to input circle domain
      SmTArray<SmExtent1d> sTrimIvls ;
      crCircAnalIvl.IntersectPeriodic(sArcIvl,     // in : Other interval          
                                      360.0,       // in : period                  
                                      sTrimIvls,   // out: list of intersections   
                                      TRUE) ;      // in : TRUE = split output intervals at
                                                   //      sCricleExtent boundaries
      // for every trimmed arc interval
      for(jj=0;jj<sTrimIvls.GetSize();jj++)
        {
          SmExtent1d &rTrimIvl = sTrimIvls[jj] ;

          // build the output curve
          // Check for degenerate.
          double dArcLen = SM_DEG2RAD( rTrimIvl.GetLength() ) * dCircRadiusX;
          if ( dArcLen < dTol )
            {
              // output a degenerate curve

              // Note that trim Points are crossing points - they are not tangent points.
              //   When two trimPoints have the same angle they should be
              //   either a finite interval trimmed down to a point
              //   or from different edges and occur when the circle
              //   intersects a corner point exactly - Because the square and the
              //   circle are both convex this can happen only when the square/circle
              //   intersection is a single point - build a degenerate curve
              SM_ASSERT(   sArcIvl.GetLength() > SM_EFF_ZERO             // trimmed to point case
                        || sTrimSide[ii] != sTrimSide.GetAt(  bOpposite  // intersect a corner case
                                                                  ? ((ii+lCount)-1)%lCount
                                                                  : (ii+1)%lCount)) ;
          
              // build a degenerate line at the degenerate point
              double dCosU = smos_Cosine(rTrimIvl.GetMin() * SM_PI / 180.0) ;
              double dSinU = smos_Sine  (rTrimIvl.GetMin() * SM_PI / 180.0) ;
              SmPoint3d sPoint =   crCircCenter 
                                 + dCosU * dCircRadiusX * crCircXAxis
                                 + dSinU * dCircRadiusY * crCircYAxis ;
              SmStatus sErr = SmBSplineCurve::CreateDegenerateCurve(crContext,3,sPoint,pCrv);
              SM_ASSERT(pCrv != NULL) ;
              if(sErr == SM_ERR) { return SM_SUCCESS ; }
              rCrvs.Add(pCrv) ;

              if(dDist < dTol) { break ; }
              else             { continue ; }
            }
          else // output a circle or ellipse arc
            {
              // build 1 full circle or ellipse
              if(bCircle) { pCrv = new (crContext) SmCircle(crCircCenter, 
                                                            crCircXAxis, crCircYAxis, rTrimIvl, 
                                                            dCircRadiusX, 3) ;
                          }
              else        { pCrv = new (crContext) SmEllipse(crCircCenter, 
                                                             crCircXAxis, crCircYAxis, rTrimIvl, 
                                                             dCircRadiusX, dCircRadiusY, 3) ;
                          }
              rCrvs.Add(pCrv) ;
            }
        } // end iter every interval
    } // end iter every trim point

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace() ;
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
      sCircYAxisForPlaneNormal.Unitize() ;
      SmEllipse sEllipse(crCircCenter, crCircXAxis, crCircYAxis, crCircAnalIvl, dCircRadiusX, dCircRadiusY, 3, &crContext) ;
      SmPoint3d sPlaneCenter, sPlaneX ;
      EvaluatePointFast(sCenterUV, sPlaneCenter) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; DrawUV(6,6) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,6, 0,1,0) ; crCircCenter.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,8, 0,1,1) ; sPlaneCenter.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,9, 0,0,1) ; (dCircRadiusX * crCircXAxis).Draw(&crCircCenter) ; sm_GraphicsLoop() ;   
      smgfx_SetLook(4,9, 0,0,1) ; (dCircRadiusX * sCircZAxis).Draw(&crCircCenter) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5, 0,1,0) ; (dCircRadiusX * GetPosition().GetZAxis()).Draw(&crCircCenter) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; sEllipse.Draw() ;  sm_GraphicsLoop() ;
      for(ii=0;ii<sTrimPoints.GetSize();ii++)
        { SmPoint3d sPlanePoint ;
          EvaluatePointFast(sTrimUVs[ii],sPlanePoint) ; 
          // blue = entering, red = exiting
          if(   (!bOpposite && sTrimEntering[ii] == TRUE)
             || ( bOpposite && sTrimEntering[ii] == FALSE)) { smgfx_SetLook(4,6, 0,0,1) ; } // blue = entering
          else                                                    { smgfx_SetLook(6,7, 1,0,0) ; } // red  = exiting
          sTrimPoints[ii].Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,9, 0,1,1) ; sPlanePoint.Draw() ; sm_GraphicsLoop() ;
        }
      ULONG lCnt = rCrvs.GetSize() ;
      for(ii=0;ii<lCnt;ii++)
        { smgfx_SetLook(5,6, 1,0,lCnt == 1 ? 1.0 : (double)ii/(double)(lCnt-1)) ;
          rCrvs[ii]->Draw() ; sm_GraphicsLoop() ;
        }
      sm_GraphicsLoop() ;
  }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ;

} // end MakeNewPlaneEllipse 

/*******************************************************************//**
PURPOSE: Rebuild the analytic definition of the plane to match
   the Nurbs definition, which might have been modified.
   This method does not modify the Nurbs definition at all.

NOTES: 
   Update:
   - m_vAnalUVDomain
   - m_vUVScale
   - m_vPosition.Origin
***********************************************************************/
SmStatus SmPlane::RebuildSTEPFromNURBParameters()
{
  // Make our analytic data match the Nurbs shape.
  SmExtent2d sNurbDomain = GetNaturalUVDomain();

  // Get three corner 3d points, to get position and length scales.
  SmPoint3d sPt00, sPt01, sPt10;
  SmPoint2d sUV = sNurbDomain.Evaluate( 0, 0 );
  SmBSplineSurface::EvaluatePoint( sUV, sPt00 );
  sUV = sNurbDomain.Evaluate( 0, 1 );
  SmBSplineSurface::EvaluatePoint( sUV, sPt01 );
  sUV = sNurbDomain.Evaluate( 1, 0 );
  SmBSplineSurface::EvaluatePoint( sUV, sPt10 );

  // Set our scale first: 3d length divided by domain length.
  // U-scale:
  SmVector3d sUVec( sPt10 - sPt00 );
  double d3dLen = sUVec.Length();
  m_vUVScale.x = d3dLen / sNurbDomain.XLength();

  sUVec /= d3dLen; // Unitize, for later.

  // V-scale:
  SmVector3d sVVec( sPt01 - sPt00 );
  d3dLen = sVVec.Length();
  m_vUVScale.y = d3dLen / sNurbDomain.YLength();

  sVVec /= d3dLen; // Unitize, for later.

  // The origin is where (0,0) would evaluate to.
  // The start point sPt00 is where the min of the domain would evaluate to.
  // This is the vector from the origin to sPt00:
  SmVector3d sStartVec =
      sNurbDomain.GetMin().x * m_vUVScale.x * m_vPosition.GetXAxis()
    + sNurbDomain.GetMin().y * m_vUVScale.y * m_vPosition.GetYAxis();

  // The new origin would be the start point minus that vector.
  m_vPosition.SetCanonical( sPt00 - sStartVec, sUVec, sVVec );

  // Finally update the domain.
  m_vAnalUVDomain = sNurbDomain;

  return SM_SUCCESS;

} // end RebuildSTEPFromNURBParameters

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmExtent2d SmPlane::GetSTEPUVDomain() const
{
  return m_vAnalUVDomain;

} // end SmPlane::GetSTEPUVDomain    

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmVector2d SmPlane::GetUVScale() const
{
  return m_vUVScale;

} // end SmPlane::GetUVScale    

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
const SmAxis2Placement & SmPlane::GetPosition() const
{
  return m_vPosition;

} // end SmPlane::GetPosition

/*******************************************************************//**
 PURPOSE: This is a fast inline method to evaluate a plane.  It assumes
 that the UV point is clamped or that you don't care if it is outside
 of the domain.

 NOTES:
***********************************************************************/
SmStatus SmPlane::EvaluatePointFast
( 
  const SmPoint2d & crStepUV,
  SmPoint3d & rPoint 
) const
{
  rPoint = m_vPosition.GetOriginRef()
    + crStepUV.x * m_vPosition.GetXAxisRef() * m_vUVScale.x
    + crStepUV.y * m_vPosition.GetYAxisRef() * m_vUVScale.y;
  return SM_SUCCESS;
} // end SmPlane::EvaluatePointFast
 
