// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfaceSilhouette.cpp
* PURPOSE: This file contains surface silhouette methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmBSplineSurface.h>
#include <SmBSplineCurve.h>
#include <SmSurfaceSilhouette.h>
#include <SmSolutionArray.h>
#include <SmGraphicsExtern.h>
#include <SmHermiteCurve.h>
#include <SmSurfaceCache.h>
#include <SmCurveCache.h>
#include <SmGeomUtility.h>
#include <SmLine.h>
#include <SmPlane.h>
#include <SmIsoCurve.h>
#include <SmLocalSolve1d.h>
/*******************************************************************//**
PURPOSE: Construct a surface silhouette object.

NOTES: 
***********************************************************************/
SmSurfaceSilhouette::SmSurfaceSilhouette
 (const SmSurface  * cpSurface, 
  const SmExtent2d & crUVDomain,
  SmBoolean          bPerspective,
  const SmVector3d & crEye)
 :
  SmSurfaceTracer(cpSurface,crUVDomain), 
  m_bPerspective(bPerspective), m_vEye(crEye) 
{
    if (!bPerspective) {
        SM_ASSERT(m_vEye.LengthSquared() > SM_EFF_ZERO_SQ);
        SE(m_vEye.Unitize());
    }
}

/*******************************************************************//**
PURPOSE: Determine if the given point is a silhouette point on the
   surface.  Note that not all curves silhouettes are also surface
   silhouettes.

NOTES: 
***********************************************************************/
SmBoolean SmSurfaceSilhouette::DoesPointLieOnCurve(SmPoint2d & rUV, SmBoolean bRefinePoint) const
{
    SmVector3d sNormal;
    SER(m_cpSurface->EvaluateNormal(rUV,TRUE,TRUE,sNormal));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        SmPoint3d sPnt;
        SER(m_cpSurface->EvaluatePoint(rUV,sPnt));
        smgfx_SetLook(1,2, 0,1,1); sPnt.Draw();         sm_GraphicsLoop();
        smgfx_SetLook(1,2, 1,0,0); sNormal.Draw(&sPnt); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif
    SmVector3d sVec;
    if (m_bPerspective) {
        SmPoint3d sPnt;
        SER(m_cpSurface->EvaluatePoint(rUV,sPnt));
        sVec = sPnt - m_vEye;
        sVec.Unitize();
    }
    else {
        sVec = m_vEye;
    }
    if (!bRefinePoint) {
        double dDot = sVec.Dot(sNormal);
        if (smos_Fabs(dDot) < SM_EFF_ZERO_SQRT) {
            return TRUE;
        }
        return FALSE;
    }

    // Refine the point some
    SmTracePnt sPnt1, sPnt2;
    SmBoolean bFound = FALSE;
    SmPoint2d sFoundUV;
    SE(LocalPointSolve(sPnt1,rUV,sPnt2,bFound,sFoundUV));
    if (bFound) {
        SmVector3d sNormal2;
        SER(m_cpSurface->EvaluateNormal(sFoundUV,TRUE,TRUE,sNormal2));
        if (m_bPerspective) {
            SmPoint3d sPnt;
            SER(m_cpSurface->EvaluatePoint(sFoundUV,sPnt));
            SmVector3d sVec2 = sPnt - m_vEye;
            if (smos_Fabs(sVec2.Dot(sNormal2)) < smos_Fabs(sVec.Dot(sNormal))) {
                rUV = sFoundUV;
            }
        }
        else {
            if (smos_Fabs(sVec.Dot(sNormal2)) < smos_Fabs(sVec.Dot(sNormal))) {
                rUV = sFoundUV;
            }   
        }
        return TRUE;
    }
    return FALSE;

} // end DoesPointLieOnCurve


/*******************************************************************//**
PURPOSE: Find potential start points along a specified boundary curve.

NOTES: Note that the solutions will be refined by the calling
   routine.  This method does not need to find exact solutions but it
   would be good if it found solutions near the actual solutions on the
   surface.
***********************************************************************/
SmStatus SmSurfaceSilhouette::FindSolutionsOnBoundaryCurve
 (const SmBSplineCurve & crBoundaryCurve,  // in : 
  const SmExtent1d     & crInterval,       // in : 
  ULONG                  lSide,            // NotUsed: in : 0 - min, 1 - max
  SmSurfParamType        eSurfParam,       // NotUsed: in : 
  SmSolutionArray      & rSolutions)       // out: 
 const
{
  SM_REF2(lSide, eSurfParam) ;
    SmCurvePropertyType eCurveProp = SM_CP_SILHOUETTE_VECTOR;
    if (m_bPerspective) {
        eCurveProp = SM_CP_SILHOUETTE_POINT;
    }
    
    SER(crBoundaryCurve.GlobalPropertyAnalysis(crInterval,
                                               eCurveProp,
                                               NULL,
                                               &m_vEye,
                                               m_dThisApproxTol3d,
                                               rSolutions));
    return SM_SUCCESS;

} // end FindSolutionsOnBoundaryCurve

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmFindSilSingEFO : public SmEvalFunctionObject
{
protected:
  const SmSurface & m_crSurface;
  const SmPoint2d & m_crUVStart;
  const SmPoint2d & m_crUVEnd;
  SmBoolean              m_bZeroGauss;
  SmBoolean              m_bPerspective;
  const SmPoint3d & m_crEye;
public:

  // constructor
  SmFindSilSingEFO(const SmSurface   & crSurface, 
                     const SmPoint2d & crUVStart, 
                     const SmPoint2d & crUVEnd,
                     SmBoolean         bZeroGauss,
                     SmBoolean         bPerspective,
                     const SmPoint3d & crEye)            : m_crSurface(crSurface), 
                                                           m_crUVStart(crUVStart), 
                                                           m_crUVEnd(crUVEnd),  
                                                           m_bZeroGauss(bZeroGauss), 
                                                           m_bPerspective(bPerspective), 
                                                           m_crEye(crEye) 
                                                         { }
  // destructor
  virtual ~SmFindSilSingEFO() { }

  // evaluate
  virtual SmStatus Evaluate(double      dT,                   // in : target T value to query
                            double    & rdFOfT,               // out: F(T)     = Function value for given dT value
                            double    & rdFPrimeOfT,          // out: dF(T)/dT = Function derivative value for given dT value
                            SmBoolean & rbFoundAnswer,        // out: TRUE = Function value is within tolerance of zero, FALSE=Not
                            SmBoolean   bSignalErrors=TRUE) ; // NotUsed: in : TRUE = signal errors, FALSE=errors anticipated, don't signal, default:[TRUE]

} ; // end class SmFindSilSingEFO

/****************************************************************
PURPOSE: 

NOTES:
****************************************************************/
SmStatus SmFindSilSingEFO::Evaluate
 (double      dT,             // in : target T value to query                                
  double    & rdFOfT,         // out: F(T)     = Function value for given dT value           
  double    & rdFPrimeOfT,    // out: dF(T)/dT = Function derivative value for given dT value
  SmBoolean & rbFoundAnswer,  // out: TRUE = Function value is within tolerance of zero      
                              //      FALSE= Not
  SmBoolean   bSignalErrors)  // NotUsed: in : TRUE = signal errors, FALSE=errors anticipated, don't signal, default:[TRUE]                                                                            
{
  SM_REF1(bSignalErrors) ;
  // init output
  rdFOfT = 0.0;
  rdFPrimeOfT = 0.0;
  rbFoundAnswer = FALSE;

  SmPoint2d sUV = m_crUVStart + dT * (m_crUVEnd - m_crUVStart);
  double dGauss1 = 0.0, dNorm = 0.0, dPrinK1 = 0.0, dPrinK2 = 0.0;
  SmVector3d sEFG, sLMN, sPrinKV1, sPrinKV2;
  SER(m_crSurface.EvaluateGeometric(sUV,TRUE,TRUE,
      dGauss1,dNorm,dPrinK1,dPrinK2,sEFG,sLMN,sPrinKV1,sPrinKV2));

  if (m_bZeroGauss) {
      rdFOfT = dGauss1;
      SmPoint2d sUV2 = m_crUVStart + (dT+1.0e-8) * (m_crUVEnd - m_crUVStart);
      double dGauss2 = 0.0;
      SER(m_crSurface.EvaluateGeometric(sUV2,TRUE,TRUE,
          dGauss2,dNorm,dPrinK1,dPrinK2,sEFG,sLMN,sPrinKV1,sPrinKV2));
      rdFPrimeOfT = (dGauss2 - dGauss1) / 1.0e-8;  //may need to refeverse 1 and 2
      if (smos_Fabs(rdFOfT) < SM_EFF_ZERO_SQ) {
          rbFoundAnswer = TRUE;
      }
  }
  else {
      SmVector3d R = m_crEye;
      if (m_bPerspective) {
          SmPoint3d sPnt;
          SER(m_crSurface.EvaluatePoint(sUV,sPnt));
          R = sPnt - m_crEye;
      }
      if (R.LengthSquared() < SM_EFF_ZERO) {
          return SM_ERR;  // point on surface must be at the eye point
      }
      double dDot1 = R.Dot(sPrinKV1);
      if (smos_Fabs(R.Dot(sPrinKV2)) < smos_Fabs(dDot1)) dDot1 = R.Dot(sPrinKV2);
      rdFOfT = dDot1;
      SmPoint2d sUV2 = m_crUVStart + (dT+1.0e-8) * (m_crUVEnd - m_crUVStart);
      SER(m_crSurface.EvaluateGeometric(sUV2,TRUE,TRUE,
          dGauss1,dNorm,dPrinK1,dPrinK2,sEFG,sLMN,sPrinKV1,sPrinKV2));
      if (m_bPerspective) {
          SmPoint3d sPnt;
          SER(m_crSurface.EvaluatePoint(sUV2,sPnt));
          R = sPnt - m_crEye;
      }
      if (R.LengthSquared() < SM_EFF_ZERO) {
          return SM_ERR;  // point on surface must be at the eye point
      }
      double dDot2 = R.Dot(sPrinKV1);
      if (smos_Fabs(R.Dot(sPrinKV2)) < smos_Fabs(dDot2)) dDot2 = R.Dot(sPrinKV2);
      rdFPrimeOfT = (dDot2 - dDot1) / 1.0e-8;
      if (smos_Fabs(rdFOfT) < SM_EFF_ZERO_SQ) {
          rbFoundAnswer = TRUE;
      }
  }

  return SM_SUCCESS;

} // end SmFindSilSingEFO::Evaluate

/*******************************************************************//**
PURPOSE: Compute the point and derivative values of the point.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceSilhouette::ComputePointValues(const SmPoint2d & crUV,
                                                 SmTracePnt & rCurrPnt,
                                                 SmTracePnt * pPrevPnt,
                                                 double *)
{
    // Evaluate the surface, 2 derivs, into rCurrPnt.m_vSurfacePV.
    SER(m_cpSurface->Evaluate(crUV,2,2,TRUE,TRUE,TRUE,
        &rCurrPnt.m_vSurfacePV[0][0]));

    // Store a unitized surface normal in [2][2].
    SmVector3d sNorm = rCurrPnt.m_vSurfacePV[1][0] * 
        rCurrPnt.m_vSurfacePV[0][1];
    if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) {
        SER(m_cpSurface->EvaluateNormal(crUV,TRUE,TRUE,
            sNorm));
        if (sNorm.LengthSquared() < SM_EFF_ZERO_SQ) {
            SER(SM_ERR);
        }
        if (rCurrPnt.m_vSurfacePV[1][0].LengthSquared() < SM_EFF_ZERO_SQ) {
            rCurrPnt.m_vSurfacePV[1][0] = rCurrPnt.m_vSurfacePV[0][1] * sNorm;
        }
        if (rCurrPnt.m_vSurfacePV[0][1].LengthSquared() < SM_EFF_ZERO_SQ) {
            rCurrPnt.m_vSurfacePV[0][1] = sNorm * rCurrPnt.m_vSurfacePV[1][0];
        }
    }
    sNorm.Unitize();
    rCurrPnt.m_vSurfacePV[2][2] = sNorm;

    rCurrPnt.m_v3DCurvePV[0] = rCurrPnt.m_vSurfacePV[0][0];

    // Compute the direction of the silhouette curve at this point.
    // The silhouette curve is the locus of points on the surface
    // where F(u,v) = 0, where F(u,v) is ( N(u,v) dot R ).
    // (Note, we could normalize N(u,v), but we don't have to,
    // and it's much easier not to, we can just use (Su cross Sv). )
    // It's an implicit function; we don't have a parametric
    // representation of the silhouette curve.
    // For the direction of the curve, we want an iso-value of the
    // function, where the value stays the same (zero).
    // That's described by the gradient of the function being zero.
    // The gradient is:
    // (R . dN/du) du + (R . dN/dv) dv = 0
    // This is satisfied when:
    // (du,dv) = ( R . dN/dv, - R . dN/du )
    //            
    // Since N = du x dv,
    // dN/du = duu x dv + du x duv
    // dN/dv = duv x dv + du x dvv
    //
    // (Un-normalized N: the direction is the same.)
    // Note that this is not a parametric derivative of the silhouette curve,
    // just a direction.

    SmVector3d R = m_vEye;
    if ( m_bPerspective ) {
        R = rCurrPnt.m_vSurfacePV[0][0] - m_vEye;
    }

    if ( R.LengthSquared() < SM_EFF_ZERO ) {
        return SM_ERR;  // point on surface must be at the eye point
    }


    SmVector3d du  = rCurrPnt.m_vSurfacePV[1][0];
    SmVector3d duu = rCurrPnt.m_vSurfacePV[2][0];
    SmVector3d dv  = rCurrPnt.m_vSurfacePV[0][1];
    SmVector3d dvv = rCurrPnt.m_vSurfacePV[0][2];
    SmVector3d duv = rCurrPnt.m_vSurfacePV[1][1];

    // Check for being near a singularity in tangency.
    // A singularity occurs if the position on the surface cannot
    // be adjusted to zero the function (dot product of surface
    // normal with the view vector).  That happens (only) when the
    // surface normal has no component of change in the direction of
    // the view vector.  This corresponds to having one of the
    // principle directions line up with the view vector, with a
    // zero principle curvature value.
    // This is also characterized by the silhouette curve lining up
    // with the view vector.

    double dGaussK, dNorm, dPrinK1, dPrinK2;
    SmVector3d sEFG, sLMN, sPrinDir1, sPrinDir2;
    SER( smsurf_EvaluateGeometric( rCurrPnt.m_vSurfacePV, 
        dGaussK, dNorm, dPrinK1, dPrinK2, sEFG, sLMN, sPrinDir1, sPrinDir2 ));

    double dAng1, dAng2;
    SER( R.AngleBetween( sPrinDir1, dAng1 ));
    SER( R.AngleBetween( sPrinDir2, dAng2 ));

    // We don't care about parallel vs. anti-parallel:
    if ( dAng1 > SM_PI/2.0 ) { dAng1 = SM_PI - dAng1; }
    if ( dAng2 > SM_PI/2.0 ) { dAng2 = SM_PI - dAng2; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        if ( FALSE ) {
            smgfx_Erase();
        }
        smgfx_SetLook( 1,1, 0,0,1 ); m_cpSurface->DrawUV(1,1); sm_GraphicsLoop();
        SmPoint3d sPnt;
        m_cpSurface->EvaluatePoint( crUV, sPnt );
        smgfx_SetLook(2,3, 0,1,1); sPnt.Draw();       sm_GraphicsLoop();
        smgfx_SetLook(2,3, 1,0,0); sNorm.Draw(&sPnt); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif


    // Check for singularity.
    // If we have a prevPoint, check it.

    if ( pPrevPnt != NULL )
    {
        double dPrevGaussK;
        SmVector3d sPrevPrinDir1, sPrevPrinDir2;
        SER( smsurf_EvaluateGeometric( pPrevPnt->m_vSurfacePV, 
                dPrevGaussK, dNorm,
                dPrinK1, dPrinK2,
                sEFG, sLMN,
                sPrevPrinDir1, sPrevPrinDir2
        ));

        //cbi Note: we're checking dAng1,dAng2 of curr pt and dPrinK1,dPrinK2 of prev.
        //cbi This whole thing should be simplified.
        int iSingularity = 0;  // 1: singularity in 1st principle dir, 2 in 2nd.
        if (   smos_Fabs( dPrinK1 ) < SM_EFF_ZERO_SQRT
            && smos_Fabs( dAng1 )   < SM_EFF_ZERO_SQRT )
        {
            iSingularity = 1;
        }
        else if (   smos_Fabs( dPrinK2 ) < SM_EFF_ZERO_SQRT
                 && smos_Fabs( dAng2 )   < SM_EFF_ZERO_SQRT )
        {
            iSingularity = 2;
        }

        if ( iSingularity > 0 )
        {
            // [Note: I haven't verified this first test yet,
            //  it doesn't get hit in prog_test.  bd 06 Nov 07 ]
            // if the gaussian curvature changes sign and the
            // smaller angle is the one which has the vanishing curvature.
            if (smos_Fabs(dGaussK) > SM_EFF_ZERO_SQRT &&
                smos_Fabs(dPrevGaussK) > SM_EFF_ZERO_SQRT &&
                dGaussK * dPrevGaussK < 0.0 &&
                ((dAng1 < dAng2 && smos_Fabs(dPrinK1) < smos_Fabs(dPrinK2)) ||
                 (dAng1 > dAng2 && smos_Fabs(dPrinK1) > smos_Fabs(dPrinK2) )) )
            {

                // Here we need to minimize GAUSS between surface points.
                SmFindSilSingEFO sEFO(*m_cpSurface,pPrevPnt->m_vUVCurvePV[0],
                    crUV,TRUE,m_bPerspective,m_vEye);
                SmExtent1d sIvl(0.0,1.0);
                SmLocalSolve1d sLS(sEFO, sIvl, FALSE);
                SmBoolean bFoundSolution;
                double dFoundT;
                if (sLS.SolveIt(0.5,1.0e-8,bFoundSolution,dFoundT) == SM_SUCCESS)
                {
                    // Found an answer in the solver -- use that new parameter and
                    // recompute the point values.
                    SmPoint2d sUV = pPrevPnt->m_vUVCurvePV[0] + dFoundT * 
                        (crUV - pPrevPnt->m_vUVCurvePV[0]);
                    SER(m_cpSurface->EvaluateGeometric(sUV,TRUE,TRUE,
                        dGaussK,dNorm,dPrinK1,dPrinK2,sEFG,sLMN,sPrinDir1,sPrinDir2));
                   
                    SmVector3d R2 = R;
                    if (m_bPerspective) {
                        SmPoint3d sSrfPnt;
                        SER(m_cpSurface->EvaluatePoint(sUV,sSrfPnt));
                        R2 = sSrfPnt - m_vEye;
                    }

                    double dAngle1, dAngle2;
                    SER(R2.AngleBetween(sPrinDir1, dAngle1 ));
                    SER(R2.AngleBetween(sPrinDir2, dAngle2));
                    // If the angle between the two vectors is very small or very large
                    // then they are parallel.
                    if (dAngle1 > SM_PI/2.0 ) { dAngle1 = SM_PI - dAngle1; }
                    if (dAngle2 > SM_PI/2.0 ) { dAngle2 = SM_PI - dAngle2; }
                    // This case only really happens in cases where one of the curvature
                    // vectors is exactly parallel to the view vector and the other is
                    // exactly perpendicular
                    if (dAngle1 < m_dThisAngTolRad/100.0 || dAngle2 < m_dThisAngTolRad/100.0)
                    {
                        rCurrPnt.m_v3DCurvePV[1] = pPrevPnt->m_v3DCurvePV[1];
                        rCurrPnt.m_ePointType = SM_TP_SINGULARITY;
                        SER(ComputePointValues(sUV,rCurrPnt));
                        return SM_SUCCESS;
                    }
                }
            } // end if Gaussian curvature non-zero and changing sign.

            // [Note: the following angle test is not verified.  It checks
            //  whether either of the principle directions is within tol/4
            //  of perpendicular.
            //  It is always the case throughout prog_test, and I think
            //  would always have to be true, after the earlier tests.
            //  Note that tol is usually 10 or 20 degrees.  bd 06 Nov 07]

            else if ( smos_Fabs( dGaussK ) < SM_EFF_ZERO_SQRT && 
                ( smos_Fabs( SM_PI/2.0 - dAng1 ) < m_dThisAngTolRad*4.0 ||
                  smos_Fabs( SM_PI/2.0 - dAng2 ) < m_dThisAngTolRad*4.0) )
            {
                // Here we are on parabolic point -- we need to find place
                // where one of the principle directions lines up with R.
                // Note, it's much easier to find the zero of two vectors being
                // perpendicular than parallel.  And since we know that the two
                // principle directions are perpendicular to each other, we solve
                // for the other principle direction being perpendicular to R.

                double dCurrDot = (iSingularity==1) ? R.Dot(sPrinDir2) : R.Dot(sPrinDir1);

                // Eye vec for previous point could be different, if m_bPerspective.
                SmVector3d RPrev = m_vEye;
                if (m_bPerspective) {
                    RPrev = pPrevPnt->m_vSurfacePV[0][0] - m_vEye;
                }
                double dPrevDot = (iSingularity==1) ? RPrev.Dot( sPrevPrinDir2 )
                                                    : RPrev.Dot( sPrevPrinDir1 );

                if ( smos_Fabs( dCurrDot ) < SM_EFF_ZERO )
                {
                    // Hit the singularity directly - no need to use solver
                    SmPoint2d sUV = crUV;
                    rCurrPnt.m_ePointType = SM_TP_SINGULARITY;
                    rCurrPnt.m_v3DCurvePV[1] = pPrevPnt->m_v3DCurvePV[1];
                    // Call ourselves recursively, without pPrevPnt, to do simple
                    // evaluation without singularity checking.
                    SER( ComputePointValues( sUV, rCurrPnt ));
                    return SM_SUCCESS;
                }
                else // No: do this with or without a sign change:
                     // if (dCurrDot * dPrevDot < 0.0 && smos_Fabs(dPrevDot) > SM_EFF_ZERO)
                {
                    // Here we need to minimize dot between R and one of the
                    // principle directions.
                    SmFindSilSingEFO sEFO( *m_cpSurface, pPrevPnt->m_vUVCurvePV[0],
                        crUV, FALSE, m_bPerspective, m_vEye );

                    // Solve for a value of T such that (T==0) is prev uv and
                    // (T==1) is crUV.  Allow convergence outside of
                    // [ prev uv, crUV ], which would be [0,1]:
                    SmExtent1d sIvl( -5.0, 5.0 );
                    SmLocalSolve1d sLS( sEFO, sIvl, FALSE );
                    SmBoolean bFoundSolution;
                    double dFoundT;
                    // Guess param: linearly interpolate between PrevDot (at t==0)
                    // and CurrDot (at t==1) to where Dot would be zero.
                    // dGuessT = PrevDot / (PrevDot - CurrDot)
                    double dGuessT = 0.5; // in case we can't get a guess
                    double denom = dPrevDot - dCurrDot;
                    if ( smos_Fabs(denom) > smos_Fabs( dPrevDot ) * SM_EFF_ZERO_SQRT )
                    {
                        dGuessT = dPrevDot / denom;
                        // Make sure it's within sIvl so SolveIt doesn't complain.
                        if ( smos_Fabs( dGuessT ) < 10.0 ) {
                            sIvl.AddValue( dGuessT );
                        }
                        dGuessT = sIvl.ClampValue( dGuessT );
                    }

                    if ( sLS.SolveIt( dGuessT, 1.0e-8, bFoundSolution, dFoundT ) == SM_SUCCESS )
                    {
                        // Call ourselves recursively, without pPrevPnt, to do simple
                        // evaluation without singularity checking.
                        if (smos_Fabs(sLS.m_dFoundAccuracy) > SM_EFF_ZERO_SQRT)
                        {
                            // Didn't find a good solution,
                            SmPoint2d sUV = crUV;
                            // Temporarily indicate a singularity, which will
                            // disable the setting of the curve tangent,
                            // just below (in this routine, recursively).
                            rCurrPnt.m_ePointType = SM_TP_SINGULARITY;
                            rCurrPnt.m_v3DCurvePV[1] = pPrevPnt->m_v3DCurvePV[1];
                            SER(ComputePointValues(sUV,rCurrPnt));
                            rCurrPnt.m_ePointType = SM_TP_NORMAL;
                        }
                        else
                        {
                            // Found the singular point.
                            // The found parameter is from prevPnt uv to crUV: interpolate.
                            SmPoint2d sUV = pPrevPnt->m_vUVCurvePV[0]
                                          + dFoundT * (crUV - pPrevPnt->m_vUVCurvePV[0]);
                            rCurrPnt.m_ePointType = SM_TP_SINGULARITY;

                            // For setting the curve tangent direction, it probably
                            // makes sense to set it to the view direction.  (That's
                            // really what caused the singularity in the first place,
                            // the curve tangent lines up with the view direction.)
                            // Do that if it's pretty close already.
                            // Preserve the magnitude of the previous tangent.
                            SmVector3d sPrevTan = pPrevPnt->m_v3DCurvePV[1];
                            double dTanMag = sPrevTan.Length();

                            rCurrPnt.m_v3DCurvePV[1] = sPrevTan; // default

                            // Update the view vector, in case rCurrPnt has moved.
                            R = m_vEye;
                            if ( m_bPerspective ) {
                                R = rCurrPnt.m_vSurfacePV[0][0] - m_vEye;
                            }
                            double dDot = sPrevTan.Dot( R );
                            if ( dDot > 0.995 )
                            {
                                rCurrPnt.m_v3DCurvePV[1] =  R * dTanMag;
                            }
                            else if ( dDot < -0.995 )
                            {
                                rCurrPnt.m_v3DCurvePV[1] = -R * dTanMag;
                            }
                            SER( ComputePointValues( sUV, rCurrPnt ));
                        }
                        return SM_SUCCESS;

                    } // end if SolveIt succeeded.
                } // end if current point not the exact solution, need to iterate.
            } // end else: zero Gaussian curvature.
        } // end if iSingularity is 1 or 2
    } // end if we have a prev point

    else
    {
        // No previous point.  Check for singularity.
        if (   ( dAng1 < m_dThisAngTolRad / 100 && smos_Fabs( dPrinK1 ) < SM_EFF_ZERO_SQRT )
            || ( dAng2 < m_dThisAngTolRad / 100 && smos_Fabs( dPrinK2 ) < SM_EFF_ZERO_SQRT ) )
        {
            // Singularity: there would be two solutions,
            // one parallel to the view direction and one perpendicular.
            // Use the one perpendicular. [B100]
            rCurrPnt.m_ePointType = SM_TP_SINGULARITY; // so we don't overwrite this:
            rCurrPnt.m_v3DCurvePV[1] = ( dAng1 > dAng2 ) ? sPrinDir1 : sPrinDir2;
        }
    }

    if ( rCurrPnt.m_ePointType != SM_TP_SINGULARITY )
    {
        // The normal (nonsingular) case:
        // Compute sTanVec, the 3d direction, according to the gradient formula.

        // Derivatives of the Normal:
        SmVector3d dNdu = duu * dv + du * duv;
        SmVector3d dNdv = duv * dv + du * dvv;

        // The zero-gradient direction:
        SmPoint2d sUVVec;
        sUVVec.x =   R.Dot( dNdv );
        sUVVec.y = - R.Dot( dNdu );

        // Compute the 3D tangent vector from the 2D tangent and 
        // the cross derivatives.
        SmVector3d sTanVec = du * sUVVec.x + dv * sUVVec.y;
        sTanVec = sTanVec * m_dCurveTraceDirection;

        // If there was a previous point, and it points in the opposite
        // direction from sTanVec, flip sTanVec to align (roughly) with previous.
        if ( pPrevPnt != NULL )
        {
            if ( sTanVec.Dot( pPrevPnt->m_v3DCurvePV[1] ) < 0.0) {
                sTanVec = - sTanVec;
            }
        }

        // Got sTanVec.
        // Set curve tangent to sTanVec, if it's not zero.
        if ( sTanVec.LengthSquared() > SM_EFF_ZERO_SQ )
        {
            rCurrPnt.m_v3DCurvePV[1] = sTanVec;
            rCurrPnt.m_v3DCurvePV[1].Unitize();
        }
        else
        {
            // If we have a previous point use it otherwise we are in trouble
            if ( pPrevPnt != NULL )
            {
                rCurrPnt.m_v3DCurvePV[1] = pPrevPnt->m_v3DCurvePV[1];
            }
            else
            {
                double dTanLen = sTanVec.Length();
                if ( dTanLen > SM_EFF_ZERO_SQ )
                {
                    sTanVec = sTanVec / dTanLen;
                    sTanVec.Unitize();
                    rCurrPnt.m_v3DCurvePV[1] = sTanVec;
                }
                else
                {
                    SER(SM_ERR);
                }
            } // end else: no prevPnt.
        } // end else: zero-length TanVec branch.
    } // end if currPnt is not a singularity

    // Now compute parameter space point and vector
    rCurrPnt.m_vUVCurvePV[0] = crUV;
    SER( smsurf_DropVectors( rCurrPnt.m_vSurfacePV[1][0],
                rCurrPnt.m_vSurfacePV[0][1],
            1, &rCurrPnt.m_v3DCurvePV[1],
               &rCurrPnt.m_vUVCurvePV[1]
    ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2=FALSE;
    if (bDebugMe2) {
        smos_WriteBuffer(_T("\n------------------------\n"));
        rCurrPnt.m_v3DCurvePV[0].Dump();
        rCurrPnt.m_v3DCurvePV[1].Dump();
        rCurrPnt.m_v3DCurvePV[0].Draw();
        rCurrPnt.m_v3DCurvePV[1].Draw(&rCurrPnt.m_v3DCurvePV[0]);
    }
#endif

    return SM_SUCCESS;

} // end ComputePointValues


/*******************************************************************//**
PURPOSE: Compute the actual UV point given a guess UV point on the
    curve being traced.

NOTES: 
***********************************************************************/
SmStatus SmSurfaceSilhouette::LocalPointSolve(const SmTracePnt & crCurrPnt,
                                              const SmPoint2d & crGuessUV,
                                              SmTracePnt & ,
                                              SmBoolean & rbFoundAnswer,
                                              SmPoint2d & rUVFound) const
{
    rbFoundAnswer = FALSE;

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe2 = FALSE;
    if (bDebugMe2) {
        if ( FALSE ) {
            smgfx_Erase();
            smgfx_SetLook( 1,2, 0,0,1 ); m_cpSurface->DrawUV( 6,6 ); sm_GraphicsLoop();
        }
        SmPoint2d sCurrUV = crCurrPnt.m_vUVCurvePV[0];
        smgfx_SetLook( 1,2, 0,1,0 ); m_cpSurface->DrawAt( sCurrUV, 1 ); sm_GraphicsLoop();
        smgfx_SetLook( 1,2, 1,0,0 ); m_cpSurface->DrawAt( crGuessUV, 1 ); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#else
    SM_REF1(crCurrPnt);
#endif

    // If the guess point is outside of our domain, clamp it.
    SmPoint2d sUV = m_vUVDomain.ClampPoint2d( crGuessUV );


    // First check serendipity: evaluate the function at the guess (clamped).
    // We're going to need two derivs for the main algorithm,
    // so go ahead and evaluate everything now.
    SmVector3d sEval[3][3];
    SER(m_cpSurface->Evaluate(sUV,2,2,TRUE,TRUE,TRUE,&sEval[0][0]));
    SmVector3d sNormal = sEval[1][0] * sEval[0][1];

    // If cross product is zero, try to get the normal another way,
    // and then use that to make sure that the derivatives are not zero.
    if ( sNormal.LengthSquared() < SM_EFF_ZERO_SQ )
    {
        SER(m_cpSurface->EvaluateNormal(sUV,TRUE,TRUE,sNormal));
        if (sNormal.LengthSquared() < SM_EFF_ZERO_SQ) {
            SER(SM_ERR);
        }
        if (sEval[1][0].LengthSquared() < SM_EFF_ZERO_SQ)
          { sEval[1][0] = sEval[0][1] * sNormal; }
        if (sEval[0][1].LengthSquared() < SM_EFF_ZERO_SQ)
          { sEval[0][1] = sNormal * sEval[1][0]; }
    }
    else {
        SER(sNormal.Unitize());
    }
    SmVector3d R = m_vEye;
    if (m_bPerspective) {
        R = sEval[0][0] - m_vEye;
    }
    double dFcnVal = R.Dot( sNormal );
    // Don't compare to m_dThisApproxTol3d: that can be big,
    // and we would like to do better if possible.
    if ( smos_Fabs( dFcnVal ) < SM_EFF_ZERO * 100.0 )
    {
        rbFoundAnswer = TRUE;
        rUVFound = sUV;
        return SM_SUCCESS;
    }
    // end serendipity check.


    SmBoolean bClampU = FALSE;
    SmBoolean bClampV = FALSE;
    // First check for case where Guess is on boundary
    // For this case intersect boundary curve with plane.
    if (SM_ARE_SAME(sUV.x,m_vUVDomain.GetMin().x) ||
        SM_ARE_SAME(sUV.x,m_vUVDomain.GetMax().x) ) {
        bClampU = TRUE;
    }

    if (SM_ARE_SAME(sUV.y,m_vUVDomain.GetMin().y) ||
        SM_ARE_SAME(sUV.y,m_vUVDomain.GetMax().y) ) {
        bClampV = TRUE;
    }

    // Create a plane perpendicular to the last tangent and use that
    // to find a point on the surface where the surface silhouette exists.

    SmVector3d du  = sEval[1][0];
    SmVector3d duu = sEval[2][0];
    SmVector3d dv  = sEval[0][1];
    SmVector3d dvv = sEval[0][2];
    SmVector3d duv = sEval[1][1];
    
    // Derivatives of du*dv w.r.t. u and v:
    SmVector3d dNdu = duu * dv + du * duv;
    SmVector3d dNdv = duv * dv + du * dvv;
    
    // uv direction perpendicular to gradient of function
    // (so the function value should stay zero).
    SmPoint2d sUVVec;
    sUVVec.x =   R.Dot(dNdv);
    sUVVec.y = - R.Dot(dNdu);
    
    // Check to see if we hit the point directly
    if (smos_Fabs(sNormal.Dot(m_vEye)) < SM_EFF_ZERO &&
        sUVVec.LengthSquared() > SM_EFF_ZERO_SQ) {
        rbFoundAnswer = TRUE;
        rUVFound = sUV;
        return SM_SUCCESS;
    }

    // Compute the 3D tangent vector from the 2D tangent and 
    // the cross derivatives.
    SmVector3d sTanVec = du * sUVVec.x + dv * sUVVec.y;
    if (sTanVec.LengthSquared() < SM_EFF_ZERO_SQ) {
        rbFoundAnswer = FALSE;
        return SM_SUCCESS;
    }
    
    // Make a plane perpendicular to the current curve
    // Use the plane to compute the normal section to the
    // surface at the given point.  Perform Newton iteration
    // on the theoretical curve generated by the normal section
    // to approach the real silhouette point.
    
    // First find the intersection of the plane with the 
    // tangent plane and express the results as a vector
    // in the tangent plane which has the same magnitude as
    // the linear combination of DU and DV.  
    SmPoint3d sPnt;
    // Note that the following three evaluations can be turned
    // into a single evaluation.  Save for later optimization.
    sPnt = sEval[0][0];
    
    SmVector3d sNormalPlaneVec = sTanVec * sNormal;
    SmVector3d sTangentPlaneDir;
    double dCurvature=0.0;
    SER(m_cpSurface->EvaluateNormalSection(sUV,TRUE,TRUE,sNormalPlaneVec,
        sTangentPlaneDir, dCurvature, sNormal));
    
    // Find center and normal of circle 
    if (smos_Fabs(dCurvature) < SM_EFF_ZERO) {
//        return SM_SUCCESS;
        // Note, this was here originally, commented out in favor of the return.
        // Changed it back, with the addition of the sign.
        // [B22 and 091023; cf. B100, B155]
        dCurvature = (dCurvature > 0 ) ? SM_EFF_ZERO : -SM_EFF_ZERO;
    }
    SmPoint3d sCenter = sPnt + sNormal / dCurvature;
    double dRadius = smos_Fabs(1.0 / dCurvature);
    SmVector3d sCircleNormal = sTangentPlaneDir * sNormal;
#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smos_WriteBuffer(_T("\n***************\n"));
        sCenter.Dump();
        sTanVec.Dump();
    }
#endif
    
    // Find the closest silhouette point on the normal circle
    SmPoint3d sSilPoint1, sSilPoint2;
    SmBoolean bFoundSil;
    SER(smgu_CircleFindSilhouettePoints(sCenter,sCircleNormal,dRadius,
        m_bPerspective,m_vEye,
        bFoundSil, sSilPoint1, sSilPoint2));
    if (!bFoundSil) {
        rbFoundAnswer = FALSE;
        return SM_SUCCESS;
    }
    SmPoint3d sFoundPoint = sSilPoint1;
    if (sPnt.DistanceBetweenSquared(sSilPoint1) > 
        sPnt.DistanceBetweenSquared(sSilPoint2) ) {
        sFoundPoint = sSilPoint2;
    }
    
    // Now find the parametric vector corresponding to sFoundPoint.
    SmVector3d s3DStep = sFoundPoint - sPnt;
    SER( sNormalPlaneVec.Unitize() );
    s3DStep = sNormalPlaneVec * s3DStep.Length();
    
    SmVector2d sUVStep;
    double dStepLeng = s3DStep.LengthSquared();
    if ( dStepLeng < (m_dThisApproxTol3d / 10000.0) *
        (m_dThisApproxTol3d / 10000.0) ) {
        rbFoundAnswer = TRUE;
        rUVFound = sUV;
        return SM_SUCCESS;
    }
    // This little bit of logic gives us a small first step which
    // will better control Brent.
    if (dStepLeng > (m_dThisApproxTol3d) * (m_dThisApproxTol3d)) {
        SER(s3DStep.Unitize());
        s3DStep = s3DStep * m_dThisApproxTol3d;
    }
    SER(m_cpSurface->DropVectors(sUV,TRUE,TRUE,1,&s3DStep,&sUVStep));
    if (bClampU) { sUVStep.x = 0.0; }
    if (bClampV) { sUVStep.y = 0.0; }
    
    // Make sure we don't step too far on that first step or we might
    // miss the curve.
    SmVector2d sSize = m_vUVDomain.GetSize();
    if (smos_Fabs(sUVStep.x) > sSize.x/100.0) {
        sUVStep = sUVStep * (sSize.x/100.0) / sUVStep.x;
    }
    if (smos_Fabs(sUVStep.y) > sSize.y/100.0) {
        sUVStep = sUVStep * (sSize.y/100.0) / sUVStep.y;
    }

    // Now that we have established a direction to look for silhouette
    // use Brent's method to try to converge upon the root where the
    // normal section is a silhouette point.
    double dA = 0.0;
    double dB = 1.0;
    double dC = 2.0;
    double dFA = R.Dot(sNormal);
    SmPoint2d sUVNew1 = sUV + sUVStep;
    SmPoint2d sUVNew2 = sUV + sUVStep * 2.0;
    SmVector3d sNormal1, sNormal2;
    if (!m_vUVDomain.ContainsPoint2d(sUVNew1,SM_EFF_ZERO))
    {
        dC = m_vUVDomain.ClipLine2d(sUV,sUVStep,1.0);
        // Returns SM_BIG_DOUBLE in case of error.
        if ( dC == SM_BIG_DOUBLE || smos_Fabs( dC ) < SM_EFF_ZERO )
          { return SM_SUCCESS; }

        sUVNew2 = sUV + sUVStep * dC;
        dB = dC / 2.0;
        sUVNew1 = sUV + sUVStep * dB;
    }
    else if (!m_vUVDomain.ContainsPoint2d(sUVNew2,SM_EFF_ZERO))
    {
        dC = m_vUVDomain.ClipLine2d(sUV,sUVStep,2.0);
        if ( dC == SM_BIG_DOUBLE || smos_Fabs( dC ) < SM_EFF_ZERO )
          { return SM_SUCCESS; }

        sUVNew2 = sUV + sUVStep * dC;
    }
    sUVNew1 = m_vUVDomain.ClampPoint2d(sUVNew1);
    sUVNew2 = m_vUVDomain.ClampPoint2d(sUVNew2);
    SER(m_cpSurface->EvaluateNormal(sUVNew1,TRUE,TRUE,sNormal1));
    double dFB = R.Dot(sNormal1);
    SER(m_cpSurface->EvaluateNormal(sUVNew2,TRUE,TRUE,sNormal2));
    double dFC = R.Dot(sNormal2);

    // First check to see if we found answer without any iteration.
    if (smos_Fabs(dFB) < SM_EFF_ZERO) {
        rbFoundAnswer = TRUE;
        rUVFound = sUVNew1;
        return SM_SUCCESS;
    }
    if (smos_Fabs(dFA) < SM_EFF_ZERO) {
        rbFoundAnswer = TRUE;
        rUVFound = sUV;
        return SM_SUCCESS;
    }
    if (smos_Fabs(dFC) < SM_EFF_ZERO) {
        rbFoundAnswer = TRUE;
        rUVFound = sUVNew2;
        return SM_SUCCESS;
    }

    ULONG lCount = 0;
    while (TRUE) {
        lCount ++;
        // Use Brent's method of stepping - quadratic root
        double dR = dFB / dFC;
        double S = dFB / dFA;
        double T = dFA / dFC;
        double P = S * ( T  * (dR-T)* (dC - dB) - (1.0-dR)*(dB - dA));
        double Q = (T - 1.0)*(dR - 1.0)*(S - 1.0);
        // The following will occur if we hit the boundary during the 
        // iteration.
        if (Q == 0.0 || smos_Fabs(dFB-dFC) < SM_EFF_ZERO_SQ) {
            if (smos_Fabs(dFC) < SM_EFF_ZERO_SQRT) {
                rbFoundAnswer = TRUE;
                rUVFound = sUV + sUVStep * dC;
                return SM_SUCCESS;
            }
            break;
        }
        double x = dB + P / Q;
        SmVector2d sDomSize = m_vUVDomain.GetSize();
        // Keep silhouette settled down by damping it by domain size
        if (sUVStep.x > sDomSize.x/30.0) {
            sUVStep = sUVStep * ((sDomSize.x/30.0)/sUVStep.x);
        }
        if (sUVStep.y > sDomSize.y/30.0) {
            sUVStep = sUVStep * ((sDomSize.y/30.0)/sUVStep.y);
        }
        SmPoint2d sUVNew = sUV + sUVStep * x;
        // Make sure our step sizes are not too large
        if (!m_vUVDomain.ContainsPoint2d(sUVNew,SM_EFF_ZERO)) {
            x = m_vUVDomain.ClipLine2d(sUV,sUVStep,1.0);
            sUVNew = sUV + sUVStep * x;
        }
        sUVNew = m_vUVDomain.ClampPoint2d(sUVNew);
        SmVector3d sNorm;
        SER(m_cpSurface->EvaluateNormal(sUVNew,TRUE,TRUE,sNorm));
        if (m_bPerspective) {
            SmPoint3d sPt;
            SER(m_cpSurface->EvaluatePoint(sUVNew,sPt));
            R = sPt - m_vEye;
        }
        double dFx = R.Dot(sNorm);
        // If we have a very good hit then stop
        if (smos_Fabs(dFx) < SM_EFF_ZERO) {
            rbFoundAnswer = TRUE;
            rUVFound = sUVNew;
            return SM_SUCCESS;
        }
        // If we hit a limit of some sort compute 3D step and
        // see if we are near to a solution.
        if (lCount > 100 || smos_Fabs(dB-dC) < SM_EFF_ZERO) {                
            if (smos_Fabs(dFx) < SM_EFF_ZERO_SQRT) {
                rbFoundAnswer = TRUE;
                rUVFound = sUVNew;
                return SM_SUCCESS;
            }
            break;
        }
        // Move to next iteration in loop
        dA = dB;
        dB = dC;
        dC = x;
        dFA = dFB;
        dFB = dFC;
        dFC = dFx;

    } // end Brent iteration

    // If we didn't return within the loop,
    // then we didn't find a solution.

    rbFoundAnswer = FALSE;
    return SM_SUCCESS;

} // end LocalPointSolve



/*******************************************************************//**
PURPOSE: Determine this branch of the tree intersects with the 
   plane we are using to section with.

NOTES: 
***********************************************************************/
SmBoolean SmSurfaceSilhouette::BranchMayContainAnswers(SmTreeNode * apBranch[SM_GS_MAX_TREES])
{
    // Use the Gauss map of the normal of the surface to do an
    // angular test to see if we can ignore this node.
    SmTreeNode *pSurfaceNode = apBranch[0];
    if (pSurfaceNode->m_eAuxDataType == SM_AD_BEZIER_SURFACE) {
        SmBezierPatch *pBezPatch = (SmBezierPatch*)pSurfaceNode->m_pData; 
        NER(pBezPatch);
        if (!m_bPerspective) {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
            if (bDebugMe) {
                const SmSurface &sBSS1 = *pBezPatch->mBA_pSurface ;
                SmExtent2d sUVDomain1 = pBezPatch->GetUVDomain();
                SmPoint3d sPnt;
                SER(sBSS1.EvaluatePoint(sUVDomain1.Evaluate(0.5,0.5),sPnt));
                sm_GraphicsLoop();
                smgfx_ChangeColor(TRUE);
                sBSS1.DrawUV(3,3,FALSE,&sUVDomain1);
                smgfx_SetColor(1,0,0);
                m_vEye.Draw(&sPnt);
                smgfx_SetColor(0,0,1);
                pBezPatch->GetPolarBox().Draw(sPnt);
                sm_GraphicsLoop();
            }
#endif

            if (pBezPatch->GetPolarBox().HasPerpendicularToVector(m_vEye)) {
                return TRUE;
            }
            else {
                return FALSE;
            }
        }
        else {
            // Create a polar box from the point to the
            // surface patch.  Then test to see if that
            // polar box is disjoint with the normal polar
            // box of the surface.  We can do this using
            // the Cross method.  
            SmPoint3d sCorners[8];
            pBezPatch->GetPseudoBox().CalcCorners(sCorners);
            SmPolarBox sTmpPolarBox;
            for (ULONG jj=0; jj<8; jj++) {
                SmVector3d sVec = m_vEye - sCorners[jj];
                if (sVec.LengthSquared() > SM_EFF_ZERO) {
                    SER(sTmpPolarBox.AddVector3d(sVec));
                }
            }
            SmPolarBox sCross1, sCross2;
            SE(pBezPatch->GetPolarBox().Cross(sTmpPolarBox,sCross1));
            SE(sCross1.Cross(pBezPatch->GetPolarBox(),sCross2));
            if (sCross2.AreDisjoint(sTmpPolarBox)) {
                return FALSE;
            }
            else {
                return TRUE;
            }
        }
    }

    return TRUE;


} // end SmSurfaceSilhouette::BranchMayContainAnswers

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSurfaceSilhouette::IsKindOf( SM_TYPE t ) const
{
  return ((SmSurfaceSilhouette_TYPE == t) ? TRUE : SmSurfaceTracer::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print   

NOTES: 
***********************************************************************/
void SmSurfaceSilhouette::Dump() const
{
  // TCHAR sBuff[SM_TBLOCK_SIZE];

  // start
  smos_WriteBuffer(_T("\nBegin SmSurfaceSilhouette::Dump()")) ;

  // dump base
  SmSurfaceTracer::Dump() ;

  // pretty print values

  // end
  smos_WriteBuffer(_T("\nEnd SmSurfaceSilhouette::Dump()\n")) ;

} // end SmSurfaceSilhouette::Dump



