// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmBlendSurface.cpp 
* PURPOSE: Implementation of SmBlendSurface methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmBlendSurface.h>
#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmSurface.h>
#include <SmCrvOnSurf.h>
#include <SmGeomUtility.h>

/*******************************************************************//**
PURPOSE: Set The Cross Derivatives at start and end of a blend section
    curve.

NOTES: 
***********************************************************************/
void SmBlendSection::SetCrossDerivs(ULONG lNumDerivs,
                                    SmVector3d *pStartDerivs,
                                    SmVector3d *pEndDerivs)
{
    if (lNumDerivs > 3) lNumDerivs = 3;
    for (ULONG i=0; i<lNumDerivs; i++) {
        m_vStartCrossDerivs[i] = pStartDerivs[i];
        m_vEndCrossDerivs[i] = pEndDerivs[i];
    }
}






/*******************************************************************//**
PURPOSE: Compute the local reference frame at the given parameter.

NOTES: 
***********************************************************************/
SmStatus SmBlendSection::ComputeFrameAt
  (double dNormalizedParameter,
   SmPoint3d & rOrigin,
   SmVector3d & rCurveTangent,
   SmVector3d & rXAxis,
   SmVector3d & rYAxis,
   SmBlendSection *)                      // pOptMatingSection = If given it will
                                          // be used to align the Y of the local axis.  It will 
                                          // override the tangent of curve being the local frame.
  const 
{
    if (m_eOrientation == SM_OT_OPPOSITE) {
        dNormalizedParameter = 1.0-dNormalizedParameter;
    }
    SmPoint3d sPV[2];

    if (m_pUVCurve) {
        SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval();
        SER(m_pUVCurve->Evaluate(sIvl.Evaluate(dNormalizedParameter),1,TRUE,sPV));
        SmPoint2d sUV(sPV[0].x,sPV[0].y);
        SmVector3d sDU, sDV;
        SER(m_pSurface->Evaluate1stDerivatives(sUV,TRUE,TRUE,rOrigin,sDU,sDV));
        SmVector3d sXDir = sDU*sPV[1].x + sDV*sPV[1].y;
        if (m_eOrientation == SM_OT_OPPOSITE) {
            sXDir = - sXDir;
        }
        if (sXDir.LengthSquared() < SM_EFF_ZERO_SQ) {
            sXDir = sDU;
            if (sXDir.LengthSquared() < SM_EFF_ZERO_SQ) {
                sXDir = sDV;
            }
        }
        rCurveTangent = sXDir; // output curve tangent

        SER(sXDir.Unitize());
        SmVector3d sNorm;
        SER(m_pSurface->EvaluateNormal(sUV,TRUE,TRUE,sNorm));
        SmVector3d sYDir = sNorm * sXDir;
        SER(sYDir.Unitize());

//        if (FALSE && pOptMatingSection) {
//            SmTArray<SmVector3d> sDerivs;
//            SmPoint3d sPnt;
//            SER(Evaluate(dNormalizedParameter,1,sPnt,sDerivs));
//            SmVector3d sTanMe = sDerivs[0];
//            SER(pOptMatingSection->Evaluate(dNormalizedParameter,1,sPnt,sDerivs));
//            SmVector3d sTanMate = sDerivs[0];
//            SER(sTanMe.Unitize());
//            SER(sTanMate.Unitize());
//            SmVector3d sPlaneOfCurveNorm = sTanMe + sTanMate;
//            SER(sPlaneOfCurveNorm.Unitize());
//            
//            SmVector3d sVec = sPnt - rOrigin;
//            SmVector3d sYAltDir = sPlaneOfCurveNorm * sNorm;
//            if (sYAltDir.LengthSquared() > SM_EFF_ZERO_SQ) {
//                if (sYAltDir.Dot(sYDir) < 0.0) {
//                    sYAltDir = - sYAltDir;
//                }
//                sYDir = sYAltDir;
//                SER(sYDir.Unitize());
//                sXDir = sYDir * sNorm;
//                SER(sXDir.Unitize())
//            }
//        }


        rXAxis = sXDir;
        rYAxis = sYDir;
        return SM_SUCCESS;
    }

    // If only have a 3D curve we'll assume that the Z direction is in 
    // m_vStartCrossDerivs[1] and m_vEndCrossDerivs[1] which is the second
    // derivative vector.
    SmExtent1d sIvl = m_p3DCurve->GetNaturalInterval();
    SER(m_p3DCurve->Evaluate(sIvl.Evaluate(dNormalizedParameter),1,TRUE,sPV));
    SmVector3d sXDir = sPV[1];
    if (m_eOrientation == SM_OT_OPPOSITE) {
        sXDir = - sXDir;
    }
    // Test for NULL curve - if there then do some interpolation of derivatives
    if (sXDir.LengthSquared() < SM_EFF_ZERO_SQ) {
        SmVector3d sXDir1 = m_vStartCrossDerivs[0] * m_vStartCrossDerivs[1];
        SmVector3d sXDir2 = m_vEndCrossDerivs[0] * m_vEndCrossDerivs[1];
        SER(sXDir1.Unitize());
        SER(sXDir2.Unitize());
        sXDir = sXDir1 + sXDir2;
    }
    rCurveTangent = sXDir;

    SER(sXDir.Unitize());
    SmVector3d sYDir1 = m_vStartCrossDerivs[1] * sXDir;
    SmVector3d sYDir2 = m_vEndCrossDerivs[1] * sXDir;
    SER(sYDir1.Unitize());
    SER(sYDir2.Unitize());
    SmVector3d sYDir = sYDir1 + sYDir2;
    SER(sYDir.Unitize());
    rOrigin = sPV[0];
    rXAxis = sXDir;
    rYAxis = sYDir;
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Compute vectors in the local frame at the given point.

NOTES: 
***********************************************************************/
SmStatus SmBlendSection::ComputeLocalFrameVectors(double dNormalizedParameter,
                                                  ULONG lNumVectors,
                                                  SmVector3d * pEuclidVectors,
                                                  SmVector3d * pLocalFrameVectors)
{
    SmPoint3d sOrig;
    SmVector3d sTan, sX, sY;
    SER(ComputeFrameAt(dNormalizedParameter,sOrig,sTan,sX,sY));
    SmVector3d sZ = sX * sY;

    for (ULONG i=0; i<lNumVectors; i++) {
        pLocalFrameVectors[i].x = sX.Dot(pEuclidVectors[i]);
        pLocalFrameVectors[i].y = sY.Dot(pEuclidVectors[i]);
        pLocalFrameVectors[i].z = sZ.Dot(pEuclidVectors[i]);
    }

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Compute the accuracy of this span for distance and derivative
    approximation using Hermite.

NOTES: 
***********************************************************************/
SmBoolean SmBlendSection::IsSpanAccurate
  (double ,                                // dStart
   const SmTArray<SmVector3d> & ,          // crStartVectors = Contains Vector and Derivative for 
                                           // Point, Cross Deriv 1 - 3 possibly
   double ,                                // dEnd
   SmTArray<SmVector3d> & ,                // rEndVectors = Computed Vector and Derivative for end
   const SmTArray<double> & )              // crTolerances
{   
    return TRUE;
}

/*******************************************************************//**
PURPOSE: Compute Cross Constraints given a section plane normal.

NOTES: 
***********************************************************************/
SmStatus SmBlendSection::ComputeSectionCrossConstraints(double dNormalizedParam, 
                                                        const SmVector3d & crPlaneNormal,
                                                        SmTArray<SmPoint3d> & rPoints, // This and other arrays are additive here not reset
                                                        SmTArray<SmVector3d> & rTangents,
                                                        SmTArray<SmVector3d> & rHigherOrderDerivs) const
{
    SmPoint3d sPnt;
    SmVector3d sX, sY, sTan;
    SER(ComputeFrameAt(dNormalizedParam,sPnt,sTan,sX,sY,NULL));
    SmVector3d sZ = sX * sY;

    SmTArray<SmVector3d> sDerivs;

    SER(ComputeLocalFrameDerivatives(dNormalizedParam,sDerivs));

    SmVector3d sTanInFrame = sDerivs[0];
    SmVector3d sTanIn3D = sX*sTanInFrame.x + sY*sTanInFrame.y + sZ*sTanInFrame.z;
    
    SmPoint3d sPV[2];
    SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval();
    SER(m_pUVCurve->Evaluate(sIvl.Evaluate(dNormalizedParam),1,TRUE,sPV));
    SmPoint2d sUV(sPV[0].x,sPV[0].y);

    SER(m_pSurface->EvaluatePoint(sUV,sPnt));
    rPoints.Add(sPnt);

    SmVector3d sNormal;
    SER(m_pSurface->EvaluateNormal(sUV,TRUE,TRUE,sNormal));

    SmVector3d sLinePnt, sLineVec;
    SER(smgu_IntersectTwoPlanes(sPnt,sNormal,sPnt,crPlaneNormal,sLinePnt,sLineVec));
    if (sLineVec.Dot(sTanIn3D) < 0.0) {
        sLineVec = - sLineVec;
    }
    double dLeng = sTanIn3D.Length();
    SER(sLineVec.Unitize());
    sTanIn3D = sLineVec * dLeng;

    rTangents.Add(sTanIn3D);
    SmVector2d sUVTan;
    SER(m_pSurface->DropVectors(sUV,TRUE,TRUE,1,&sTanIn3D,&sUVTan));
    SmVector3d sG2Vec;
    if (m_eCrossBoundaryType == SM_CB_G2) {
        SER(m_pSurface->ComputeHigherOrderDerivs(sUV,sUVTan,sG2Vec));
        rHigherOrderDerivs.Add(sG2Vec);
    }
    SmVector3d sG3Vec;
    if (m_eCrossBoundaryType == SM_CB_G3) {
        SER(m_pSurface->ComputeHigherOrderDerivs(sUV,sUVTan,sG2Vec,&sG3Vec));
        rHigherOrderDerivs.Add(sG2Vec);
        rHigherOrderDerivs.Add(sG3Vec);
    }
    if (m_eCrossBoundaryType == SM_CB_G4) {
        SmVector3d sG4Vec;
        SER(m_pSurface->ComputeHigherOrderDerivs(sUV,sUVTan,sG2Vec,&sG3Vec,&sG4Vec));
        rHigherOrderDerivs.Add(sG2Vec);
        rHigherOrderDerivs.Add(sG3Vec);
        rHigherOrderDerivs.Add(sG4Vec);
    }
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Compute cross sectional constraints needed to create the
    blend curve.

NOTES: The rPoints and rTangents, and rHigherOrderDerivs arrays
    are not reinitiaized by this method.  They are added to.
***********************************************************************/
SmStatus SmBlendSection::ComputeIsoCrossConstraints(double dNormalizedParameter,
                                                    ULONG lCrossSection,
                                                    const SmPoint2d & crUV,
                                                    SmTArray<SmPoint3d> & rPoints, // This and other arrays are additive here not reset
                                                    SmTArray<SmVector3d> & rTangents,
                                                    SmTArray<SmVector3d> & rHigherOrderDerivs)
{
    SmPoint3d sPnt;
    SmVector3d sDU, sDV;
    SER(m_pSurface->Evaluate1stDerivatives(crUV,TRUE,TRUE,sPnt,sDU,sDV));
    SmVector3d sTanIn3D;
    rPoints.Add(sPnt);
    if (m_eSurfParam == SM_SP_U) {
        sTanIn3D = sDU;
    }
    else {
        sTanIn3D = sDV;
    }

    if (sTanIn3D.LengthSquared() < SM_EFF_ZERO_SQ) {
        rTangents.Add(SmVector3d(0,0,0));
        if (lCrossSection == 4) {
            rHigherOrderDerivs.Add(SmVector3d(0,0,0));
        }
        if (lCrossSection == 5) {
            rHigherOrderDerivs.Add(SmVector3d(0,0,0));
            rHigherOrderDerivs.Add(SmVector3d(0,0,0));
        }
        if (lCrossSection == 6) {
            rHigherOrderDerivs.Add(SmVector3d(0,0,0));
            rHigherOrderDerivs.Add(SmVector3d(0,0,0));
            rHigherOrderDerivs.Add(SmVector3d(0,0,0));
        }
        return SM_SUCCESS;
    }
    SER(sTanIn3D.Unitize());

    SmPoint3d sPnt2;
    SmVector3d sTan, sX, sY;
    SER(ComputeFrameAt(dNormalizedParameter,sPnt2,sTan,sX,sY,NULL));
    SmVector3d sZ = sX * sY;
    SmTArray<SmVector3d> sDerivs;
    SER(ComputeLocalFrameDerivatives(dNormalizedParameter,sDerivs));

    SmVector3d sTanInFrame = sDerivs[0];
    SmVector3d sTanCrv = sX*sTanInFrame.x + sY*sTanInFrame.y + sZ*sTanInFrame.z;
    SmVector3d sG2VecCrv, sG3VecCrv, sG4VecCrv;
    sG2VecCrv = sX*sDerivs[1].x + sY*sDerivs[1].y + sZ*sDerivs[1].z;
    sG3VecCrv = sX*sDerivs[2].x + sY*sDerivs[2].y + sZ*sDerivs[2].z;
    sG4VecCrv = sX*sDerivs[3].x + sY*sDerivs[3].y + sZ*sDerivs[3].z;
    double dLeng = sTanCrv.Length();
    sTanIn3D = sTanIn3D * dLeng; // Makes both tan vectors same length

    // Make sure we have correct choice of ISO direction
    if (sTanIn3D.Dot(sTanCrv) < 0.0) {
        sTanIn3D = - sTanIn3D;
    }
    // Blend between the Cross Tangent and the Iso Tangent over first 1/3 of the
    // curve.
    double dBlend = dNormalizedParameter * 3.0;
    if (dNormalizedParameter < 1.0/3.0) {
        sTanIn3D = (1.0-dBlend) * sTanCrv + dBlend * sTanIn3D;
    }


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        smgfx_SetColor(1,0,0);
        sPnt.Draw();
        sm_GraphicsLoop();
        sTanIn3D.Draw(&sPnt);
        smgfx_SetColor(0,1,1);
        sm_GraphicsLoop();
    }
#endif

    rTangents.Add(sTanIn3D);
    SmVector2d sUVTan;
    SER(m_pSurface->DropVectors(crUV,TRUE,TRUE,1,&sTanIn3D,&sUVTan));
    SmVector3d sG2Vec;
    if (lCrossSection == 4) {
        SER(m_pSurface->ComputeHigherOrderDerivs(crUV,sUVTan,sG2Vec));
        if (dNormalizedParameter < 1.0/3.0) {
            sG2Vec = (1.0-dBlend) * sG2VecCrv + dBlend * sG2Vec;
        }
        rHigherOrderDerivs.Add(sG2Vec);
    }
    SmVector3d sG3Vec;
    if (lCrossSection == 5) {
        SER(m_pSurface->ComputeHigherOrderDerivs(crUV,sUVTan,sG2Vec,&sG3Vec));
        if (dNormalizedParameter < 1.0/3.0) {
            sG2Vec = (1.0-dBlend) * sG2VecCrv + dBlend * sG2Vec;
            sG3Vec = (1.0-dBlend) * sG3VecCrv + dBlend * sG3Vec;
        }
        rHigherOrderDerivs.Add(sG2Vec);
        rHigherOrderDerivs.Add(sG3Vec);
    }
    if (lCrossSection == 6) {
        SmVector3d sG4Vec;
        SER(m_pSurface->ComputeHigherOrderDerivs(crUV,sUVTan,sG2Vec,&sG3Vec,&sG4Vec));
        if (dNormalizedParameter < 1.0/3.0) {
            sG2Vec = (1.0-dBlend) * sG2VecCrv + dBlend * sG2Vec;
            sG3Vec = (1.0-dBlend) * sG3VecCrv + dBlend * sG3Vec;
            sG4Vec = (1.0-dBlend) * sG4VecCrv + dBlend * sG4Vec;
        }
        rHigherOrderDerivs.Add(sG2Vec);
        rHigherOrderDerivs.Add(sG3Vec);
        rHigherOrderDerivs.Add(sG4Vec);
    }
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Compute cross sectional constraints needed to create the
    blend curve.

NOTES: The rPoints and rTangents, and rHigherOrderDerivs arrays
    are not reinitiaized by this method.  They are added to.
***********************************************************************/
SmStatus SmBlendSection::ComputeCrossConstraints(double dNormalizedParam, 
                                                 SmBoolean bUseCrossCurveDerivs,
                                                 SmTArray<SmPoint3d> & rPoints, // This and other arrays are additive here not reset
                                                 SmTArray<SmVector3d> & rTangents,
                                                 SmTArray<SmVector3d> & rHigherOrderDerivs,
                                                 SmBlendSection *pOptMateSection) const
{
    SmPoint3d sPnt;
    SmVector3d sX, sY, sTan;
    SER(ComputeFrameAt(dNormalizedParam,sPnt,sTan,sX,sY,pOptMateSection));
    SmVector3d sZ = sX * sY;

    SmTArray<SmVector3d> sDerivs;

    SER(ComputeLocalFrameDerivatives(dNormalizedParam,sDerivs));

    SmVector3d sTanInFrame = sDerivs[0];
    SmVector3d sTanIn3D = sX*sTanInFrame.x + sY*sTanInFrame.y + sZ*sTanInFrame.z;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        smgfx_SetColor(1,0,0);
        sPnt.Draw();
        sm_GraphicsLoop();
        sX.Draw(&sPnt);
        smgfx_SetColor(0,1,1);
        sm_GraphicsLoop();
        sY.Draw(&sPnt);
        smgfx_SetColor(0,0,1);
        sm_GraphicsLoop();
        sTanIn3D.Draw(&sPnt);
        smgfx_SetColor(0,0,0);
        sm_GraphicsLoop();
        Draw();
        sm_GraphicsLoop();
        if (pOptMateSection) pOptMateSection->Draw();
        sm_GraphicsLoop();
    }
#endif



    if (bUseCrossCurveDerivs) {
        if (m_pUVCurve) {
            SmPoint3d sPV[2];
            SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval();
            SER(m_pUVCurve->Evaluate(sIvl.Evaluate(dNormalizedParam),1,TRUE,sPV));
            SmPoint2d sUV(sPV[0].x,sPV[0].y);
            SER(m_pSurface->EvaluatePoint(sUV,sPnt));
            rPoints.Add(sPnt);
        }
        else {
            SER(SM_ERR);
        }
        rTangents.Add(sTanIn3D);
        SmVector3d s2ndIn3D = sX*sDerivs[1].x + sY*sDerivs[1].y + sZ*sDerivs[1].z;
        SmVector3d s3rdIn3D = sX*sDerivs[2].x + sY*sDerivs[2].y + sZ*sDerivs[2].z;
        SmVector3d s4thIn3D = sX*sDerivs[3].x + sY*sDerivs[3].y + sZ*sDerivs[3].z;
        if (m_eCrossBoundaryType == SM_CB_G2) {
            rHigherOrderDerivs.Add(s2ndIn3D);
        }
        if (m_eCrossBoundaryType == SM_CB_G3) {
            rHigherOrderDerivs.Add(s2ndIn3D);
            rHigherOrderDerivs.Add(s3rdIn3D);
        }
        if (m_eCrossBoundaryType == SM_CB_G4) {
            rHigherOrderDerivs.Add(s2ndIn3D);
            rHigherOrderDerivs.Add(s3rdIn3D);
            rHigherOrderDerivs.Add(s4thIn3D);
        }
        return SM_SUCCESS;
    }

    if (m_pUVCurve) {
    }
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Evaluate a blend section at the given normalized parameters.

NOTES: 
***********************************************************************/
SmStatus SmBlendSection::Evaluate(double dNormalizedParameter,
                                  ULONG lNumberDerivatives,
                                  SmPoint3d & rPoint,
                                  SmTArray<SmVector3d> & rDerivatives,
                                  double * pdArcLength) const
{
    rDerivatives.ReSet();
    if (m_pUVCurve) {
        SmPoint3d sPV[2];
        SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval();
        SER(m_pUVCurve->Evaluate(sIvl.Evaluate(dNormalizedParameter),1,TRUE,sPV));
        SmPoint2d sUV(sPV[0].x,sPV[0].y);
        SmPoint3d sPnt;
        SmVector3d sDU, sDV;
        SER(m_pSurface->Evaluate1stDerivatives(sUV,TRUE,TRUE,rPoint,sDU,sDV));
        SmVector3d sXDir = sDU*sPV[1].x + sDV*sPV[1].y;
        if (m_eOrientation == SM_OT_OPPOSITE) {
            sXDir = - sXDir;
        }
        if (sXDir.LengthSquared() < SM_EFF_ZERO_SQ) {
            // This is probably a NULL curve - let's just send it back with zero
            // derivatives.
            for (ULONG i=0; i<lNumberDerivatives; i++) {
                rDerivatives.Add(SmVector3d(0,0,0));
            }
        }
        if (lNumberDerivatives < 1) return SM_SUCCESS;

        double dScale = 1.0;
        if (pdArcLength) {
            double dCurrLength = sXDir.Length();
            dScale = *pdArcLength / dCurrLength;
        }

        sXDir = sXDir * dScale;
        if (m_p3DCurve) {
            SmPoint3d sPV3D[6];
            sIvl = m_pUVCurve->GetNaturalInterval();
            SER(m_p3DCurve->Evaluate(sIvl.Evaluate(dNormalizedParameter),4,TRUE,sPV3D));
            rPoint = sPV3D[0];
            rDerivatives.Add(sPV3D[1]);
            if (lNumberDerivatives == 1) return SM_SUCCESS;
            rDerivatives.Add(sPV3D[2]);
            if (lNumberDerivatives == 2) return SM_SUCCESS;
            rDerivatives.Add(sPV3D[3]);
            if (lNumberDerivatives == 3) return SM_SUCCESS;
            rDerivatives.Add(sPV3D[4]);
            if (lNumberDerivatives == 4) return SM_SUCCESS;
            rDerivatives.Add(sPV3D[5]);
            if (lNumberDerivatives == 5) return SM_SUCCESS;
        }

        rDerivatives.Add(sXDir);

        SmVector3d sTanIn3D = sXDir; // output curve tangent
        SmVector2d sUVTan(sPV[1].x,sPV[1].y);

        SmVector3d sG2Vec;
        if (lNumberDerivatives == 2) {
            SER(m_pSurface->ComputeHigherOrderDerivs(sUV,sUVTan,sG2Vec));
            rDerivatives.Add(sG2Vec);
        }
        SmVector3d sG3Vec;
        if (lNumberDerivatives == 3) {
            SER(m_pSurface->ComputeHigherOrderDerivs(sUV,sUVTan,sG2Vec,&sG3Vec));
            rDerivatives.Add(sG2Vec);
            rDerivatives.Add(sG3Vec);
        }
        if (lNumberDerivatives == 4) {
            SmVector3d sG4Vec;
            SER(m_pSurface->ComputeHigherOrderDerivs(sUV,sUVTan,sG2Vec,&sG3Vec,&sG4Vec));
            rDerivatives.Add(sG2Vec);
            rDerivatives.Add(sG3Vec);
            rDerivatives.Add(sG4Vec);
        }
    }

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Evaluate the blend section in UV

NOTES: 
***********************************************************************/
SmStatus SmBlendSection::EvaluateUV(double dNormalizedParameter,
                                    ULONG lNumberDerivatives,
                                    SmPoint3d & rPoint,
                                    SmTArray<SmVector3d> & rDerivatives) const
{
    rDerivatives.ReSet();
    if (lNumberDerivatives > 4) SER(SM_ERR);
    if (m_pUVCurve) {
        SmPoint3d sPVVVV[5];
        SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval();
        SER(m_pUVCurve->Evaluate(sIvl.Evaluate(dNormalizedParameter),lNumberDerivatives,TRUE,sPVVVV));
        rPoint = sPVVVV[0];
        for (ULONG i=0; i<lNumberDerivatives; i++) {
            SmVector3d sDer = sPVVVV[i+1];
            if (m_eOrientation == SM_OT_OPPOSITE) {
                sDer = - sDer;
            }
            rDerivatives.Add(sDer);
        }
    }
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Compute local start and end tangents.  This is an initialization
    function to set up cross derivatives in local frame space of the curve.

NOTES: It assumes we have set up start and end vectors.
***********************************************************************/
SmStatus SmBlendSection::ComputeLocalStartEndInFrame()
{
    ULONG lNum = m_vStartCrossDerivs.GetSize();
    m_vStartCrossDerivsInFrame.SetSize(lNum);
    m_vEndCrossDerivsInFrame.SetSize(lNum);

    SER(ComputeLocalFrameVectors(0.0,lNum,m_vStartCrossDerivs.GetDataArray(),m_vStartCrossDerivsInFrame.GetDataArray()));
    SER(ComputeLocalFrameVectors(1.0,lNum,m_vEndCrossDerivs.GetDataArray(),m_vEndCrossDerivsInFrame.GetDataArray()));
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Compute the local frame derivatives at this given parameter.

NOTES: 
***********************************************************************/
SmStatus SmBlendSection::ComputeLocalFrameDerivatives(double dParameter,
                                                      SmTArray<SmVector3d> & rDerivs) const
{
    ULONG lNumDerivs = m_vStartCrossDerivsInFrame.GetSize();
    rDerivs.SetSize(lNumDerivs);
    for (ULONG i=0; i<lNumDerivs; i++) {
        rDerivs[i] = m_vStartCrossDerivsInFrame[i] + dParameter * (m_vEndCrossDerivsInFrame[i] - m_vStartCrossDerivsInFrame[i]);
    }
    return SM_SUCCESS;
}



/*******************************************************************//**
PURPOSE: SmBlendSurface Constructor

NOTES: 
***********************************************************************/
SmBlendSurface::SmBlendSurface
  (SmBlendSection * pLeftProfile,          // in :
   SmBlendSection * pRightProfile,         // in :
   SmBlendSection * pBottomRail,           // in :
   SmBlendSection * pTopRail,              // in :
   double           d3DTolerance,          // in :
   double           dG1AngleToleranceDeg,  // NotUsed: in :
   double           dG2RadialTolerance,    // NotUsed: in :
   double           dG3VectorTolerance)    // NotUsed: in :
  : m_lNumberRailSteps(0), 
    m_lNumberProfileSteps(0), 
    m_bUseCrossCurveDerivs(FALSE),
    m_bUCurvesAreRails(FALSE),
    m_d3DTolerance(d3DTolerance)

    // Unused
    // m_dG1AngleToleranceDeg(dG1AngleToleranceDeg),
    // m_dG2RadialTolerance(dG2RadialTolerance), 
    // m_dG3VectorTolerance(dG3VectorTolerance) 
{ 
    SM_REF3(dG1AngleToleranceDeg, dG2RadialTolerance, dG3VectorTolerance) ;
    m_pLeftProfile  = pLeftProfile; 
    m_pRightProfile = pRightProfile; 
    m_pBottomRail   = pBottomRail; 
    m_pTopRail      = pTopRail; 

    // Now let's populate start and ending cross derivatives.
    SmPoint3d sPnt, sPnt2;

    // Now let's make sure things are scaled correctly
//    SmContext sContext;
//    SmCrvOnSurf *sCrv = new(sContext) SmCrvOnSurf((SmCurve&)*pLeftProfile->m_pUVCurve,(SmSurface&)*pLeftProfile->m_pSurface);
//    double dLengthLeftProfile;
//    sCrv->Length(sCrv->GetNaturalInterval(),d3DTolerance,dLengthLeftProfile);
//    SmCrvOnSurf *sCrv2 = new(sContext) SmCrvOnSurf((SmCurve&)*pRightProfile->m_pUVCurve,(SmSurface&)*pRightProfile->m_pSurface);
//    double dLengthRightProfile;
//    sCrv2->Length(sCrv2->GetNaturalInterval(),d3DTolerance,dLengthRightProfile);
//    delete sCrv;
//    delete sCrv2;

    SE(pLeftProfile->Evaluate(0.0,4,sPnt,pBottomRail->m_vStartCrossDerivs));
    SE(pBottomRail->Evaluate(0.0,4,sPnt2,pLeftProfile->m_vStartCrossDerivs));

    SE(pRightProfile->Evaluate(0.0,4,sPnt,pBottomRail->m_vEndCrossDerivs));
    SE(pBottomRail->Evaluate(1.0,4,sPnt,pRightProfile->m_vStartCrossDerivs));

    SE(pLeftProfile->Evaluate(1.0,4,sPnt,pTopRail->m_vStartCrossDerivs));
    SE(pTopRail->Evaluate(0.0,4,sPnt,pLeftProfile->m_vEndCrossDerivs));

    SE(pRightProfile->Evaluate(1.0,4,sPnt,pTopRail->m_vEndCrossDerivs));
    SE(pTopRail->Evaluate(1.0,4,sPnt,pRightProfile->m_vEndCrossDerivs));


    pTopRail->ComputeLocalStartEndInFrame();
    pBottomRail->ComputeLocalStartEndInFrame();
    pLeftProfile->ComputeLocalStartEndInFrame();
    pRightProfile->ComputeLocalStartEndInFrame();

    m_vProfiles.ReSet();
    m_vCrossCurves.ReSet();
}


/*******************************************************************//**
PURPOSE: Destructor for blend surface.

NOTES: 
***********************************************************************/
SmBlendSurface::~SmBlendSurface()
{
    for (ULONG j=0; j<m_vProfiles.GetSize(); j++) {
        SM_ASSERT(m_vProfiles[j] != NULL) ; delete m_vProfiles[j] ; m_vProfiles[j] = NULL ;
    }
    for (ULONG i=0; i<m_vCrossCurves.GetSize(); i++) {
        SM_ASSERT(m_vCrossCurves[i] != NULL) ; delete m_vCrossCurves[i] ; m_vCrossCurves[i] = NULL ;
    }
}


/*******************************************************************//**
PURPOSE: Approximate a single boundary curve

NOTES: 
***********************************************************************/
SmStatus SmBlendSurface::ApproximateBoundaryCurves(const SmContext & crContext,
                                                   SmBlendSection *pSec1,
                                                   ULONG lNumberSteps,
                                                   SmBSplineCurve *& rpSec1Curve,
                                                   double & rdAccuracy)
{
    rpSec1Curve = NULL;

    SmTArray<SmPoint3d> sPoints;
    SmTArray<SmVector3d> sTangents;
    SmTArray<SmVector3d> sHigherOrderDerivs;
    SmTArray<SmVector3d> sDerivs;

    SmPoint3d sPnt;
    SER(pSec1->Evaluate(0.0,2,sPnt,sDerivs));
    sPoints.Add(sPnt);
    sTangents.Add(sDerivs[0]);
//    sHigherOrderDerivs.Add(sDerivs[1]);
    SER(pSec1->Evaluate(1.0,2,sPnt,sDerivs));
    sPoints.Add(sPnt);
    sTangents.Add(sDerivs[0]);
//    sHigherOrderDerivs.Add(sDerivs[1]);  
    ULONG lDegree = 3;
    SmTArray<SmVector3d> *pHigherOrderDerivs = NULL;
    if (sHigherOrderDerivs.GetSize() == 2) { lDegree = 5; pHigherOrderDerivs = &sHigherOrderDerivs; }
    if (sHigherOrderDerivs.GetSize() == 4) { lDegree = 7; pHigherOrderDerivs = &sHigherOrderDerivs; }
    sPoints.ReSet();
    // Grab points from the profile curves.
    for (ULONG k=0; k<=lNumberSteps; k++) {
        double dParam = (k*1.0) / lNumberSteps;
        pSec1->Evaluate(dParam,0,sPnt,sDerivs);
        sPoints.Add(sPnt);
    }
    if (lNumberSteps > 4) {
        pHigherOrderDerivs = NULL;
    }
    SmBSplineCurve *pCurve = NULL ;
    SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,lDegree,sPoints,sTangents,
        pHigherOrderDerivs,FALSE,pCurve));

    double dMaxDist = 0.0;
    for (ULONG j=1; j<=lNumberSteps; j++) {
        double dParamSt = ((j-1)*1.0) / lNumberSteps;
        double dParamEnd = ((j)*1.0) / lNumberSteps;
        for (ULONG i=1; i<4; i++) {
            double dParamTest = dParamSt + ((i*1.0)/4.0) * (dParamEnd-dParamSt);
            SER(pSec1->Evaluate(dParamTest,0,sPnt,sDerivs));
            double dDist,dParam;
            SmBoolean bSuccess;
            SER(pCurve->DropPoint(pCurve->GetNaturalInterval(), // in : target curve allowed domain
                                  sPnt,                         // in : Point to drop to curve
                                  NULL,                         // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                  SM_BIG_DOUBLE,                // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                  NULL,                         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                  bSuccess,                     // out: TRUE = found a drop point
                                  dParam,                       // out: found drop curve param
                                  dDist)) ;                     // out: found drop distance
                                                                // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                //      default:[SM_SO_MINIMIZE] to preserve original behavior

            if (dDist > dMaxDist) dMaxDist = dDist;
        }
    }
    rpSec1Curve = pCurve;
    rdAccuracy = dMaxDist;
    return SM_SUCCESS;
}



/*******************************************************************//**
PURPOSE: Approximate the boundary curves to within the desired accuracy.
     The first curve uses the standard algorithm.  The second one uses the
     algorithm where we intersect it with a plane and use the point/tan from
     first to compute it.

NOTES: 
***********************************************************************/
SmStatus SmBlendSurface::ApproximateSectionBoundaryCurves(const SmContext & crContext,
                                                          const SmVector3d & crSectionPlane,
                                                          SmBoolean bAdjustPlane,
                                                          SmBlendSection *pSec1,
                                                          SmBlendSection *pSec2,
                                                          ULONG lNumberSteps,
                                                          SmBSplineCurve *& rpSec1Curve,
                                                          SmBSplineCurve *& rpSec2Curve)
{
    for (ULONG i=0; i<10; i++) {
        double dDist1;
        SER(ApproximateBoundaryCurves(crContext,pSec1,lNumberSteps,rpSec1Curve,dDist1));
        if (dDist1 > m_d3DTolerance) {
            if (i == 9) {
                SM_ASSERT(rpSec1Curve != NULL) ; delete rpSec1Curve ; rpSec1Curve = NULL ;
            }
            lNumberSteps = lNumberSteps * 2;
            if (lNumberSteps == 0) {
                lNumberSteps = 1;
            }
        }
        else {
            break;
        }
    }

    SmTArray<SmPoint3d> sPoints;
    SmTArray<SmVector3d> sTangents;
    SmVector3d sNorm = crSectionPlane;

    SmCrvOnSurf sCrv((SmCurve&)*(pSec2->m_pUVCurve),(SmSurface&)*(pSec2->m_pSurface),NULL,0,&crContext);
    SmExtent1d sIvl = sCrv.GetNaturalInterval();
    SmTArray<double> sKnots;
    rpSec1Curve->GetKnots(sKnots);
    for (ULONG ii=0; ii<sKnots.GetSize(); ii++) {
        double dParam = sKnots[ii];
        SmPoint3d sPnt;
        SER(rpSec1Curve->EvaluatePoint(dParam,sPnt));
        double dD = - (sPnt.Dot(crSectionPlane));
        SmSolution sSolution;
        SmBoolean bFoundAnswer; 
        if (ii < sKnots.GetSize()-1) {
            SER(sCrv.LocalPropertyAnalysis(sIvl,SM_CP_PLANE_INTERSECTION,dParam,&dD,
                &sNorm,bFoundAnswer,sSolution));
            if (bFoundAnswer) {
                dParam = sSolution.m_vStart[0];
            }
        }
        SmPoint3d sPV[2], sPV2[2];
        SER(sCrv.Evaluate(dParam,1,TRUE,sPV));
        if (bAdjustPlane) {
            SER(rpSec1Curve->Evaluate(dParam,1,TRUE,sPV2));
            SmVector3d sPP = sPV[0] - sPV2[0];
            sPV[1].Unitize();
            sPV2[1].Unitize();
            SmVector3d sAv = (sPV[1] + sPV2[1]) / 2.0;
            sNorm = sPP * sAv * sPP;
        }
        sPoints.Add(sPV[0]);
        if (ii==0 || ii==sKnots.GetSize()-1) {
            sTangents.Add(sPV[1]);
        }
    }

    ULONG lDegree = 3;
    SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,lDegree,sPoints,sTangents,
        NULL,FALSE,rpSec2Curve));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        pSec1->Draw();
        pSec2->Draw();
        rpSec1Curve->Dump();
        rpSec1Curve->DrawWithKnots();
        sm_GraphicsLoop();
        rpSec2Curve->Dump();
        rpSec2Curve->DrawWithKnots();
        sm_GraphicsLoop();
    }
#endif

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Approximate a single boundary curve using the Iso Stepping
    algorithm that takes uniform steps along an iso curve direction.

NOTES: 
***********************************************************************/
SmStatus SmBlendSurface::ApproximateIsoBoundaryCurve(const SmContext & crContext,
                                                      SmBlendSection *pSec1,
                                                      ULONG lNumberDivisions,
                                                      SmBSplineCurve *& rpSec1Curve,
                                                      SmBSplineCurve *& rpSec1UVCurve,
                                                      double & rdAccuracy)
{
    rpSec1Curve = NULL;

    SmTArray<SmPoint3d> sPoints;
    SmTArray<SmPoint3d> sUVPoints;
    SmTArray<SmVector3d> sUVTangents;
    SmTArray<SmVector3d> sTangents;
    SmTArray<SmVector3d> sHigherOrderDerivs;
    SmTArray<SmVector3d> sDerivs;

    SmPoint3d sPnt;
    SER(pSec1->Evaluate(0.0,2,sPnt,sDerivs));
    sTangents.Add(sDerivs[0]);
//    sHigherOrderDerivs.Add(sDerivs[1]);
    SER(pSec1->Evaluate(1.0,2,sPnt,sDerivs));
    sTangents.Add(sDerivs[0]);
//    sHigherOrderDerivs.Add(sDerivs[1]);
    ULONG lDegree = 3;

    SER(pSec1->EvaluateUV(0.0,1,sPnt,sDerivs));
    sUVTangents.Add(sDerivs[0]);

    SER(pSec1->EvaluateUV(1.0,1,sPnt,sDerivs));
    sUVTangents.Add(sDerivs[0]);

    const SmBSplineCurve *pUVCrv = pSec1->m_pUVCurve;
    SmPoint3d sPntSt, sPntEnd;
    pUVCrv->GetEnds(sPntSt,sPntEnd);

    SmExtent3d sDom(sPntSt);
    sDom.AddPoint3d(sPntEnd);
    SmVector3d sPlaneVec(1,0,0);
    if (pSec1->m_eSurfParam == SM_SP_U) {
        sPlaneVec.Set(0,1,0);
    }
    SmTArray<double> sCutT;
    SmSolution sData[16];
    SmSolutionArray sSolutions(16,sData);
    SmExtent1d sIvl = pUVCrv->GetNaturalInterval();

    for (ULONG i=0; i<=lNumberDivisions; i++) {
        double dParam = (i*1.0) / lNumberDivisions;
        SmPoint3d sPlanePnt = sDom.Evaluate(dParam,dParam,dParam);
        double dD = - sPlanePnt.Dot(sPlaneVec);
        SmSolution sSol;
        SmBoolean bFoundAnswer;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe) {
            smgfx_Erase();
            smgfx_SetColor(1,0,0);
            sPlanePnt.Draw();
            sPlaneVec.Draw(&sPlanePnt);
            smgfx_SetColor(0,0,0);
            pUVCrv->Draw();
            sm_GraphicsLoop();
        }
#endif

        SER(pUVCrv->LocalPropertyAnalysis(sIvl,SM_CP_PLANE_INTERSECTION,
            sIvl.Evaluate(dParam),&dD,&sPlaneVec,bFoundAnswer,sSol));
        double dT;
        if (bFoundAnswer) {
            dT = sSol.m_vStart[0];
        }
        else { // Use global algorithm
            SER(pUVCrv->GlobalPropertyAnalysis(sIvl,SM_CP_PLANE_INTERSECTION,&dD,
                &sPlaneVec,m_d3DTolerance,sSolutions));
            if (sSolutions.GetSize() != 1) {
                SER(SM_ERR);
            }
            SmSolution &rSol = sSolutions[0];
            dT = rSol.m_vStart[0];
        }
        sCutT.Add(dT);
    }

    // See which orientation the curve goes - if reversed flip it
    if (sCutT[0] > sCutT.GetLast()) {
        sCutT.ReverseArray(0,sCutT.GetSize());
    }

    sPoints.ReSet();
    sUVPoints.ReSet();
    // Grab points from the profile curves.
    for (ULONG k=0; k<sCutT.GetSize(); k++) {
        double dParam = sCutT[k];
        pSec1->Evaluate(dParam,0,sPnt,sDerivs);
        sPoints.Add(sPnt);
        pSec1->EvaluateUV(dParam,0,sPnt,sDerivs);
        sUVPoints.Add(sPnt);
    }
//    if (lNumberSteps > 4) {
//        pHigherOrderDerivs = NULL;
//    }
    SmBSplineCurve *pCurve = NULL ;
    SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,lDegree,sPoints,sTangents,
        NULL,FALSE,pCurve));

    SmBSplineCurve *pUVCurve = NULL ;
    SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,2,lDegree,sUVPoints,sUVTangents,
        NULL,FALSE,pUVCurve));

    double dMaxDist = 0.0;
    for (ULONG j=1; j<sCutT.GetSize(); j++) {
        double dParamSt = sCutT[j-1];
        double dParamEnd = sCutT[j];
        for (ULONG i=1; i<4; i++) {
            double dParamTest = dParamSt + ((i*1.0)/4.0) * (dParamEnd-dParamSt);
            SER(pSec1->Evaluate(dParamTest,0,sPnt,sDerivs));
            double dDist,dParam;
            SmBoolean bSuccess;
            SER(pCurve->DropPoint(pCurve->GetNaturalInterval(), // in : target curve allowed domain
                                  sPnt,                         // in : Point to drop to curve
                                  NULL,                         // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                  SM_BIG_DOUBLE,                // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                                //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                  NULL,                         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                  bSuccess,                     // out: TRUE = found a drop point
                                  dParam,                       // out: found drop curve param
                                  dDist)) ;                     // out: found drop distance
                                                                // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                //      default:[SM_SO_MINIMIZE] to preserve original behavior

            if (dDist > dMaxDist) dMaxDist = dDist;
        }
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
    if (bDebugMe2) {
        smgfx_Erase();
        smgfx_SetColor(0,0,0);
        pSec1->Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        pCurve->Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        pUVCrv->Draw();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        pUVCurve->Draw();
        sm_GraphicsLoop();
    }
#endif

    rpSec1UVCurve = pUVCurve;
    rpSec1Curve = pCurve;
    rdAccuracy = dMaxDist;
    return SM_SUCCESS;
}



/*******************************************************************//**
PURPOSE: Approximate the boundary curves to within the desired accuracy.
     The first curve uses the standard algorithm.  The second one uses the
     algorithm where we intersect it with a plane and use the point/tan from
     first to compute it.

NOTES: 
***********************************************************************/
SmStatus SmBlendSurface::ApproximateIsoSteppingBoundaryCurves(const SmContext & crContext,
                                                              SmBlendSection *pSec1,
                                                              SmBlendSection *pSec2,
                                                              ULONG lNumberSteps,
                                                              ULONG lNumberDivisions,
                                                              SmBSplineCurve *& rpSec1Curve,
                                                              SmBSplineCurve *& rpSec2Curve,
                                                              SmBSplineCurve *& rpSec1UVCurve,
                                                              SmBSplineCurve *& rpSec2UVCurve)
{
    for (ULONG i=0; i<lNumberSteps; i++) {
        double dDist1, dDist2;
        SER(ApproximateIsoBoundaryCurve(crContext,pSec1,lNumberDivisions,rpSec1Curve,rpSec1UVCurve,dDist1));
        SER(ApproximateIsoBoundaryCurve(crContext,pSec2,lNumberDivisions,rpSec2Curve,rpSec2UVCurve,dDist2));
        if (dDist1 > m_d3DTolerance || dDist2 > m_d3DTolerance) {
            if (i <= 9) {
                SM_ASSERT(rpSec1Curve != NULL) ; delete rpSec1Curve ; rpSec1Curve = NULL ;
                SM_ASSERT(rpSec2Curve != NULL) ; delete rpSec2Curve ; rpSec2Curve = NULL ;
                SM_ASSERT(rpSec1UVCurve != NULL) ; delete rpSec1UVCurve ; rpSec1UVCurve = NULL ;
                SM_ASSERT(rpSec2UVCurve != NULL) ; delete rpSec2UVCurve ; rpSec2UVCurve = NULL ;
            }
            lNumberDivisions = lNumberDivisions * 2;
            if (lNumberDivisions == 0) {
                lNumberDivisions = 1;
            }
        }
        else {
            break;
        }
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        pSec1->Draw();
        pSec2->Draw();
        rpSec1Curve->Dump();
        rpSec1Curve->DrawWithKnots();
        sm_GraphicsLoop();
        rpSec2Curve->Dump();
        rpSec2Curve->DrawWithKnots();
        sm_GraphicsLoop();
    }
#endif

    return SM_SUCCESS;
}




/*******************************************************************//**
PURPOSE: Approximate the boundary curves to within the desired accuracy.

NOTES: 
***********************************************************************/
SmStatus SmBlendSurface::ApproximateBoundaryCurves(const SmContext & crContext,
                                                   SmBlendSection *pSec1,
                                                   SmBlendSection *pSec2,
                                                   ULONG lNumberSteps,
                                                   SmBSplineCurve *& rpSec1Curve,
                                                   SmBSplineCurve *& rpSec2Curve)
{
    for (ULONG i=0; i<10; i++) {
        double dDist1, dDist2;
        SER(ApproximateBoundaryCurves(crContext,pSec1,lNumberSteps,rpSec1Curve,dDist1));
        SER(ApproximateBoundaryCurves(crContext,pSec2,lNumberSteps,rpSec2Curve,dDist2));
        if (dDist1 > m_d3DTolerance || dDist2 > m_d3DTolerance) {
            if (i < 9) {
                SM_ASSERT(rpSec1Curve != NULL) ; delete rpSec1Curve ; rpSec1Curve = NULL ;
                SM_ASSERT(rpSec2Curve != NULL) ; delete rpSec2Curve ; rpSec2Curve = NULL ;
            }
            lNumberSteps = lNumberSteps * 2;
            if (lNumberSteps == 0) {
                lNumberSteps = 1;
            }
        }
        else {
            break;
        }
    }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        pSec1->Draw();
        pSec2->Draw();
        rpSec1Curve->Dump();
        rpSec1Curve->DrawWithKnots();
        sm_GraphicsLoop();
        rpSec2Curve->Dump();
        rpSec2Curve->DrawWithKnots();
        sm_GraphicsLoop();
    }
#endif

    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Create a set of profiles along the rails including the first
 and last profile curves if given.

NOTES: 
***********************************************************************/
SmStatus SmBlendSurface::CreateProfileCurves(const SmContext & crContext)
{
    SmTArray<SmPoint3d> sPoints;
    SmTArray<SmVector3d> sTangents;
    SmTArray<SmVector3d> sHigherOrderDerivs;

    SmBSplineCurve *pLeftCurve=NULL, *pRightCurve=NULL;
    SER(ApproximateBoundaryCurves(crContext,m_pLeftProfile,m_pRightProfile,m_lNumberProfileSteps,
        pLeftCurve,pRightCurve));
    m_vProfiles.Add(pLeftCurve);

    for (ULONG j=1; j<m_lNumberRailSteps; j++) {
        double dParam = (j*1.0) / m_lNumberRailSteps;
        sPoints.ReSet();
        sTangents.ReSet();
        sHigherOrderDerivs.ReSet();
        SER(m_pBottomRail->ComputeCrossConstraints(dParam,m_bUseCrossCurveDerivs,
            sPoints,sTangents,sHigherOrderDerivs,m_pTopRail));
        SER(m_pTopRail->ComputeCrossConstraints(dParam,m_bUseCrossCurveDerivs,
            sPoints,sTangents,sHigherOrderDerivs,m_pBottomRail));
        ULONG lDegree = 3;
        SmTArray<SmVector3d> *pHigherOrderDerivs = NULL;
        if (sHigherOrderDerivs.GetSize() == 2) { lDegree = 5; pHigherOrderDerivs = &sHigherOrderDerivs; }
        if (sHigherOrderDerivs.GetSize() == 4) { 
            lDegree = 7; pHigherOrderDerivs = &sHigherOrderDerivs; 
            SmVector3d sTmp = sHigherOrderDerivs[1];
            sHigherOrderDerivs[1] = sHigherOrderDerivs[2];
            sHigherOrderDerivs[2] = sTmp;
        }

        SmBSplineCurve *pProfile = NULL ;
        SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,sTangents,pHigherOrderDerivs));
        SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,lDegree,sPoints,sTangents,
            pHigherOrderDerivs,FALSE,pProfile));
        m_vProfiles.Add(pProfile);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        smgfx_Erase();
        smgfx_SetColor(0,0,0);
        pProfile->Draw();
        sm_GraphicsLoop();
        pProfile->DrawPolygon();
        sm_GraphicsLoop();
        pProfile->DrawWithKnots();
        sm_GraphicsLoop();
        pProfile->DrawCurvature(0.25,30);
        SmBSplineCurve *pProfile2 = NULL ;

        SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,lDegree,sPoints,sTangents,
            &sHigherOrderDerivs,FALSE,pProfile2));
        smgfx_SetColor(1,0,0);
        pProfile2->Draw();
        sm_GraphicsLoop();
        pProfile2->DrawPolygon();
        sm_GraphicsLoop();
        pProfile2->DrawWithKnots();
        sm_GraphicsLoop();
        pProfile2->DrawCurvature(0.25,32);
        sm_GraphicsLoop();
    }
#endif

    }
    m_vProfiles.Add(pRightCurve);

    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Create a set of profiles along the rails including the first
 and last profile curves if given.  It assumes that the rails are symmetric
 for now.  

NOTES: The Left Profile must not be the degenerate profile in
  the case of a 3 sided patch situation.
***********************************************************************/
SmStatus SmBlendSurface::CreateOrientedSweepProfileCurves(const SmContext & crContext,
                                                          SmBoolean bAdjustNormal)
{

    const SmBSplineCurve *pProf = m_pLeftProfile->m_p3DCurve;
//  const SmBSplineCurve *pEnd = m_pRightProfile->m_p3DCurve;
    SmBoolean bIsFilletCrossSection;
    ULONG lCrossSection;
    double dDistance = 0.0, dRadius = 0.0;
    double dBlendScale = 1.0;
    SER(pProf->TestForFilletCrossSection(crContext,m_d3DTolerance,
        bIsFilletCrossSection,lCrossSection,dDistance,dRadius,dBlendScale));

    if (!bIsFilletCrossSection) {
        // For now just punt if we don't have a good curve type
        return SM_ERR;
    }


    SmPoint3d sPV[2], sPV2[2];
    SER(pProf->Evaluate(pProf->GetNaturalInterval().GetMin(),1,TRUE,sPV));
    SER(pProf->Evaluate(pProf->GetNaturalInterval().GetMax(),1,TRUE,sPV2));
    SmVector3d sNormal = sPV[1] * sPV2[1];
    SER(sNormal.Unitize());

    SmBSplineCurve *pBot=NULL, *pTop=NULL;
    SER(ApproximateSectionBoundaryCurves(crContext,sNormal,bAdjustNormal,m_pBottomRail,m_pTopRail,4,pBot,pTop));
    m_vCrossCurves.Add(pBot);
    m_vCrossCurves.Add(pTop);

    SmTArray<double> sKnots;
    SmBSplineCurve *pBotRail = m_vCrossCurves[0];
    SmBSplineCurve *pTopRail = m_vCrossCurves[1];
    pBotRail->GetKnots(sKnots);

    SmTArray<SmPoint3d> sPoints;
    SmTArray<SmVector3d> sTangents;
    SmTArray<SmVector3d> sHigher;

    for (ULONG j=0; j<sKnots.GetSize(); j++) {
        double dParam = sKnots[j];
        SmBSplineCurve *pProfile = NULL;
        SmPoint3d sP1, sP2;
        SER(pBotRail->EvaluatePoint(dParam,sP1));
        SER(pTopRail->EvaluatePoint(dParam,sP2));
        if (sP1.DistanceBetween(sP2) < m_d3DTolerance) {
            if (m_vProfiles.GetSize() == 0) SER(SM_ERR);
            SmBSplineCurve *pPrev = m_vProfiles.GetLast();
            SmBSplineCurve *pCopy = new (crContext) SmBSplineCurve(*pPrev);
            SmPoint3d sPnt;
            SmControlPointFormType eForm = SM_CP_EUCLIDIAN_RATIONAL;
            if (!pPrev->IsRational()) eForm = SM_CP_NON_RATIONAL;
            // Let's build a NULL curve using an existing non NULL curve
            // this should keep everyone syncronized and happy in Skin
            ULONG lNumCP = pCopy->GetNumberControlPoints();
            for (ULONG kk=0; kk<lNumCP; kk++) {
                double dWeight=1.0;
                SmPoint3d sCPnt;
                pCopy->GetControlPoint(eForm,kk,sCPnt,dWeight);
                dWeight = 1.0;
                pCopy->SetControlPoint(eForm,kk,sP1,dWeight);
            }
            m_vProfiles.Add(pCopy);
            continue;
        }

        sPoints.ReSet();
        sTangents.ReSet();
        sHigher.ReSet();
        if (bAdjustNormal) {
            SmPoint3d sPVB[2], sPVT[2];
            pBotRail->Evaluate(dParam,1,TRUE,sPVB);
            SmPoint3d sTanBot = sPVB[1];
            pTopRail->Evaluate(dParam,1,TRUE,sPVT);
            SmPoint3d sTanTop = sPVT[1];
            SER(sTanBot.Unitize());
            SER(sTanTop.Unitize());
            SmVector3d sAverage = (sTanBot + sTanTop) / 2.0;
            SmVector3d sPtoP = sP2 - sP1;
            sNormal = sPtoP * sAverage * sPtoP;
            SER(sNormal.Unitize());
        }
        SER(m_pBottomRail->ComputeSectionCrossConstraints(dParam,sNormal,sPoints,sTangents,sHigher));
        SER(m_pTopRail->ComputeSectionCrossConstraints(dParam,sNormal,sPoints,sTangents,sHigher));

        if (lCrossSection == 0 || lCrossSection == 1) { // Circular or approx circular case
            SmBoolean bIsCircle;
            SmAxis2Placement sPlace;
            double dStartAngDeg, dEndAngDeg;
            if (smgu_CircleFromPointsTangents(sP1,sTangents[0],sP2,sTangents[1],2.0,
                bIsCircle,sPlace,dRadius,dStartAngDeg,dEndAngDeg) != SM_SUCCESS) {
                SER(SM_ERR);
            }
            if (bIsCircle) {
                SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sPlace,dRadius,
                    dStartAngDeg,dEndAngDeg,SM_CO_QUADRATIC,pProfile));
            }
        }
        if (lCrossSection == 2) { // Chamfer
            SER(SmBSplineCurve::CreateLineSegment(crContext,3,sP1,sP2,pProfile));
        }

        if (lCrossSection == 3) { // G1
            SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,sTangents,NULL));
            sTangents[0] = sTangents[0]*dBlendScale;
            sTangents[1] = sTangents[1]*dBlendScale;
            SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,3,sPoints,sTangents,NULL,FALSE,pProfile));
        }

        if (lCrossSection == 4) { // G2
            sHigher.SetSize(2);
            SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,sTangents,&sHigher));
            sTangents[0] = sTangents[0]*dBlendScale;
            sTangents[1] = sTangents[1]*dBlendScale;
            sHigher[0] = sHigher[0]*dBlendScale*dBlendScale;
            sHigher[1] = sHigher[1]*dBlendScale*dBlendScale;
            SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,5,sPoints,sTangents,&sHigher,FALSE,pProfile));
        }

        if (lCrossSection == 5) { // G3
            sHigher.SetSize(4);
            SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,sTangents,&sHigher));
            sTangents[0] = sTangents[0]*dBlendScale;
            sTangents[1] = sTangents[1]*dBlendScale;
            sHigher[0] = sHigher[0]*dBlendScale*dBlendScale;
            sHigher[1] = sHigher[1]*dBlendScale*dBlendScale;
            sHigher[2] = sHigher[2]*dBlendScale*dBlendScale*dBlendScale;
            sHigher[3] = sHigher[3]*dBlendScale*dBlendScale*dBlendScale;
            SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,7,sPoints,sTangents,&sHigher,FALSE,pProfile));
        }


        m_vProfiles.Add(pProfile);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        if (j==0) smgfx_Erase();
        pTopRail->DrawWithKnots();
        pBotRail->DrawWithKnots();
        smgfx_SetColor(0,0,0);
        pProfile->Draw();
        sm_GraphicsLoop();
//        pProfile->DrawPolygon();
//        sm_GraphicsLoop();
//        pProfile->DrawWithKnots();
//        sm_GraphicsLoop();
//        pProfile->DrawCurvature(0.25,30);
    }
#endif

    }

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Create a set of profiles along the rails including the first
 and last profile curves if given.  It will make equal steps along the
 Rail curves relative to the parameter space of the underlying surface.

NOTES: This can probably be extended to work for 4 sided things.
  We would need to blend between the start and end blend scale.
***********************************************************************/
SmStatus SmBlendSurface::CreateIsoSteppingProfileCurves(const SmContext & crContext)
{
    const SmBSplineCurve *pProf = m_pLeftProfile->m_p3DCurve;
//  const SmBSplineCurve *pEnd = m_pRightProfile->m_p3DCurve;
    SmBoolean bIsFilletCrossSection;
    ULONG lCrossSection;
    double dDistance = 0.0, dRadius = 0.0;
    double dBlendScale = 1.0;

    SER(pProf->TestForFilletCrossSection(crContext,m_d3DTolerance,
        bIsFilletCrossSection,lCrossSection,dDistance,dRadius,dBlendScale));

    if (!bIsFilletCrossSection) {
        // For now just punt if we don't have a good curve type
        return SM_ERR;
    }

// Grab ending information
//    SER(pEnd->TestForFilletCrossSection(crContext,m_d3DTolerance,
//        bIsFilletCrossSection,lCrossSection,dDistance,dRadius,dBlendScale));

    SmPoint3d sPV[2], sPV2[2];
    SER(pProf->Evaluate(pProf->GetNaturalInterval().GetMin(),1,TRUE,sPV));
    SER(pProf->Evaluate(pProf->GetNaturalInterval().GetMax(),1,TRUE,sPV2));

    SmBSplineCurve *pBot=NULL, *pTop=NULL;
    SmBSplineCurve *pBotUV=NULL, *pTopUV=NULL;
    SER(ApproximateIsoSteppingBoundaryCurves(crContext,m_pBottomRail,m_pTopRail,
        4,m_lNumberRailSteps,pBot,pTop,pBotUV,pTopUV));
    m_vCrossCurves.Add(pBot);
    m_vCrossCurves.Add(pTop);

    SmTArray<double> sKnots;
    SmBSplineCurve *pBotRail = m_vCrossCurves[0];
    SmBSplineCurve *pTopRail = m_vCrossCurves[1];
    pBotRail->GetKnots(sKnots);

    SmTArray<SmPoint3d> sPoints;
    SmTArray<SmVector3d> sTangents;
    SmTArray<SmVector3d> sHigher;
    SmExtent1d sIvl = pBotRail->GetNaturalInterval();

    for (ULONG j=0; j<sKnots.GetSize(); j++) {
        double dParam = sKnots[j];
        double dNormalizedParameter;
        sIvl.Inversion(dParam,dNormalizedParameter);
        SmBSplineCurve *pProfile = NULL;
        SmPoint3d sP1, sP2;
        SER(pBotRail->EvaluatePoint(dParam,sP1));
        SER(pTopRail->EvaluatePoint(dParam,sP2));
        if (sP1.DistanceBetween(sP2) < m_d3DTolerance) {
            if (m_vProfiles.GetSize() == 0) SER(SM_ERR);
            SmBSplineCurve *pPrev = m_vProfiles.GetLast();
            if (pPrev == NULL)SER(SM_ERR);
            SmBSplineCurve *pCopy = new (crContext) SmBSplineCurve(*pPrev);
            SmPoint3d sPnt;
            SmControlPointFormType eForm = SM_CP_EUCLIDIAN_RATIONAL;
            if (!pPrev->IsRational()) eForm = SM_CP_NON_RATIONAL;
            // Let's build a NULL curve using an existing non NULL curve
            // this should keep everyone syncronized and happy in Skin
            ULONG lNumCP = pCopy->GetNumberControlPoints();
            for (ULONG kk=0; kk<lNumCP; kk++) {
                double dWeight=1.0;
                SmPoint3d sCPnt;
                pCopy->GetControlPoint(eForm,kk,sCPnt,dWeight);
                dWeight = 1.0;
                pCopy->SetControlPoint(eForm,kk,sP1,dWeight);
            }
            m_vProfiles.Add(pCopy);
            continue;
        }

        sPoints.ReSet();
        sTangents.ReSet();
        sHigher.ReSet();

        SmPoint3d sPnt;
        SER(pBotUV->EvaluatePoint(dParam,sPnt));
        SmPoint2d sUVBot(sPnt.x,sPnt.y);
        SER(m_pBottomRail->ComputeIsoCrossConstraints(dNormalizedParameter,lCrossSection,
            sUVBot,sPoints,sTangents,sHigher));

        SER(pTopUV->EvaluatePoint(dParam,sPnt));
        SmPoint2d sUVTop(sPnt.x,sPnt.y);
        SER(m_pTopRail->ComputeIsoCrossConstraints(dNormalizedParameter,lCrossSection,
            sUVTop,sPoints,sTangents,sHigher));

        if (lCrossSection == 0 || lCrossSection == 1) { // Circular or approx circular case
            SmBoolean bIsCircle;
            SmAxis2Placement sPlace;
            double dStartAngDeg, dEndAngDeg;
            if (smgu_CircleFromPointsTangents(sP1,sTangents[0],sP2,sTangents[1],2.0,
                bIsCircle,sPlace,dRadius,dStartAngDeg,dEndAngDeg) != SM_SUCCESS) {
                SER(SM_ERR);
            }
            if (bIsCircle) {
                SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sPlace,dRadius,
                    dStartAngDeg,dEndAngDeg,SM_CO_QUADRATIC,pProfile));
            }
        }
        if (lCrossSection == 2) { // Chamfer
            SER(SmBSplineCurve::CreateLineSegment(crContext,3,sP1,sP2,pProfile));
        }

        if (lCrossSection == 3) { // G1
            SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,sTangents,NULL));
            sTangents[0] = sTangents[0]*dBlendScale;
            sTangents[1] = sTangents[1]*dBlendScale;
            SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,3,sPoints,sTangents,NULL,FALSE,pProfile));
        }

        if (lCrossSection == 4) { // G2
            sHigher.SetSize(2);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
            if (bDebugMe) {
                sPoints.Dump();
                sTangents.Dump();
                sHigher.Dump();
            }
#endif
            SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,sTangents,&sHigher));
            sTangents[0] = sTangents[0]*dBlendScale;
            sTangents[1] = sTangents[1]*dBlendScale;
            sHigher[0] = sHigher[0]*dBlendScale*dBlendScale;
            sHigher[1] = sHigher[1]*dBlendScale*dBlendScale;
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4 = FALSE;
            if (bDebugMe4) {
                sPoints.Dump();
                sTangents.Dump();
                sHigher.Dump();
            }
#endif

            SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,5,sPoints,sTangents,&sHigher,FALSE,pProfile));
        }

        if (lCrossSection == 5) { // G3
            sHigher.SetSize(4);
            SER(SmBSplineCurve::FairBlendCurveDerivatives(sPoints,sTangents,&sHigher));
            sTangents[0] = sTangents[0]*dBlendScale;
            sTangents[1] = sTangents[1]*dBlendScale;
            sHigher[0] = sHigher[0]*dBlendScale*dBlendScale;
            sHigher[1] = sHigher[1]*dBlendScale*dBlendScale;
            sHigher[2] = sHigher[2]*dBlendScale*dBlendScale*dBlendScale;
            sHigher[3] = sHigher[3]*dBlendScale*dBlendScale*dBlendScale;
            SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,7,sPoints,sTangents,&sHigher,FALSE,pProfile));
        }


        m_vProfiles.Add(pProfile);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        if (j==0) smgfx_Erase();
        pTopRail->DrawWithKnots();
        pBotRail->DrawWithKnots();
        smgfx_SetColor(0,0,0);
        pProfile->Draw();
        sm_GraphicsLoop();
        pProfile->DrawPolygon();
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        pProf->Draw();
        pProf->DrawPolygon();
        sm_GraphicsLoop();
        pProf->Dump();

//        pProfile->DrawWithKnots();
//        sm_GraphicsLoop();
//        pProfile->DrawCurvature(0.25,30);
    }
#endif

    }

    return SM_SUCCESS;
}



/*******************************************************************//**
PURPOSE: Create a set of profiles along the rails including the first
 and last profile curves if given.

NOTES: 
***********************************************************************/
SmStatus SmBlendSurface::CreateCrossCurves(const SmContext & crContext)
{
    SmTArray<SmPoint3d> sPoints;
    SmTArray<SmVector3d> sTangents;
    SmTArray<SmVector3d> sHigherOrderDerivs;

    SmBSplineCurve *pBottomCurve=NULL, *pTopCurve=NULL;
    SER(ApproximateBoundaryCurves(crContext,m_pBottomRail,m_pTopRail,m_lNumberRailSteps,
        pBottomCurve,pTopCurve));
    m_vCrossCurves.Add(pBottomCurve);


    for (ULONG j=1; j<m_lNumberProfileSteps; j++) {
        double dParam = (j*1.0) / m_lNumberProfileSteps;
        sPoints.ReSet();
        sTangents.ReSet();
        sHigherOrderDerivs.ReSet();
        SER(m_pLeftProfile->ComputeCrossConstraints(dParam,m_bUseCrossCurveDerivs,
            sPoints,sTangents,sHigherOrderDerivs));
        SER(m_pRightProfile->ComputeCrossConstraints(dParam,m_bUseCrossCurveDerivs,
            sPoints,sTangents,sHigherOrderDerivs));
        ULONG lDegree = 3;
        SmTArray<SmVector3d> *pHigherOrderDerivs = NULL;
        if (sHigherOrderDerivs.GetSize() == 2) { lDegree = 5; pHigherOrderDerivs = &sHigherOrderDerivs; }
        if (sHigherOrderDerivs.GetSize() == 4) { lDegree = 7; pHigherOrderDerivs = &sHigherOrderDerivs; }

        sPoints.ReSet();
        // Grab points from the profile curves.
        for (ULONG k=0; k<m_vProfiles.GetSize(); k++) {
            SmPoint3d sPnt;
            m_vProfiles[k]->EvaluatePoint(dParam,sPnt);
            sPoints.Add(sPnt);
        }

        SmBSplineCurve *pCross = NULL ;
        SER(SmBSplineCurve::CreateInterpolatingCurve(crContext,SM_CP_UNIFORM,3,lDegree,sPoints,sTangents,
            pHigherOrderDerivs,FALSE,pCross));
        m_vCrossCurves.Add(pCross);
    }

    m_vCrossCurves.Add(pTopCurve);

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Create a Blend by stepping in equal Iso parametric steps along
    two rail surfaces.  This basically is a End-To-End blending of two 
    surfaces.

NOTES: 
***********************************************************************/
SmStatus SmBlendSurface::CreateIsoSteppingBlend(const SmContext & crContext,
                                                SmBSplineSurface *& rpBlendingSurface)
{
    if (CreateIsoSteppingProfileCurves(crContext) != SM_SUCCESS) {
        return SM_ERR;
    }

    SmTArray<double> sKnots;
    m_vCrossCurves[0]->GetKnots(sKnots);
    SmBSplineSurface * pDerivSurfs[2] = { NULL, NULL };
    SER(SmBSplineSurface::CreateSkinnedSurface(crContext,m_vProfiles,TRUE,SM_SP_U,
        m_d3DTolerance,m_vCrossCurves[0],m_vCrossCurves[1],FALSE,&sKnots,
        pDerivSurfs,rpBlendingSurface));

    return SM_SUCCESS;
}




/*******************************************************************//**
PURPOSE: Create By Sweeping with Tangency.  We sweep the profile 
  curve along the rails while maintaining its orientation and tangency
  with the rail surfaces.  This may be useful for creating corner patches
  with 2 different radii for 3 fillets.

NOTES: This works best for a symetric case where the two side
  curves are very close to being symetric and the angle traversed is 90
  degrees or less.
***********************************************************************/
SmStatus SmBlendSurface::CreateTangentOrientedSweep(const SmContext & crContext,
                                                    SmBoolean bAdjustNormal,
                                                    SmBSplineSurface *& rpBlendingSurface)
{
    m_lNumberProfileSteps = 0; // This will give us just 2 rails for sweep
    if (CreateOrientedSweepProfileCurves(crContext,bAdjustNormal) != SM_SUCCESS) {
        return SM_ERR;
    }

    SmTArray<double> sKnots;
    m_vCrossCurves[0]->GetKnots(sKnots);
    SmBSplineSurface * pDerivSurfs[2] = { NULL, NULL };
    SER(SmBSplineSurface::CreateSkinnedSurface(crContext,m_vProfiles,TRUE,SM_SP_U,
        m_d3DTolerance,m_vCrossCurves[0],m_vCrossCurves[1],FALSE,&sKnots,pDerivSurfs,rpBlendingSurface));

    return SM_SUCCESS;
}



/*******************************************************************//**
PURPOSE: Create the blending surface

NOTES: 
***********************************************************************/
SmStatus SmBlendSurface::CreateDoubleBlendSurface(const SmContext & crContext,
                                                  SmBSplineSurface *& rpBlendingSurface)
{
    SER(CreateProfileCurves(crContext));
    SER(CreateCrossCurves(crContext));
    if (m_bUCurvesAreRails) {
        SER(SmBSplineSurface::CreateGordonSurface(crContext,m_vProfiles,m_vCrossCurves,rpBlendingSurface));
    }
    else {
        SER(SmBSplineSurface::CreateGordonSurface(crContext,m_vCrossCurves,m_vProfiles,rpBlendingSurface));
    }
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmBlendSection::Dump(void) const
{
    m_pUVCurve->Dump();
    m_pSurface->Dump();
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmDisplayList *SmBlendSection::Draw(void) const
{
    // return value
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  
  // locals
  SmCrvOnSurf sCrv((SmCurve&)*m_pUVCurve,(SmSurface&)*m_pSurface);
  SmPoint3d sPnt;
  SmTArray<SmVector3d> sDerivs;
  ULONG j ;


  // start displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor());

  sCrv.Draw();
 
  // start point
  Evaluate(0.0,0,sPnt,sDerivs);
  sPnt.Draw();

  // start cross-derivs
  for (j=0; j<m_vStartCrossDerivs.GetSize(); j++) 
    {
      m_vStartCrossDerivs[j].Draw(&sPnt);
    }

  // end point
  Evaluate(1.0,0,sPnt,sDerivs);
  sPnt.Draw();

  // end cross-derivs
  for (j=0; j<m_vStartCrossDerivs.GetSize(); j++) 
    {
      m_vEndCrossDerivs[j].Draw(&sPnt);
    }

  // all done
  pRtn = smgfx_Close() ;
#endif

  return(pRtn) ;

} // end SmBlendSection::Draw

