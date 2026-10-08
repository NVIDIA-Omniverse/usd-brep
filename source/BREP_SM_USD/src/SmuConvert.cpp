// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuConvert.cpp
* PURPOSE: Implementation for USD / SMLib object conversions
**********************************************************************/

#include "UsdBrepConfig.h"

// pixar includes
#include "UsdBrepSuppressPixarWarningsPush.h"        // turn off compile warnings for problematic pixar include files
#         include <pxr/usd/usdShade/shader.h>
#include "UsdBrepSuppressPixarWarningsPop.h"         // done loading problematic pixar headers - restore compile warnings

#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

#include "SmuConvert.h"
#include "SmuUtilities.h"
#include "SmuAttribute.h"

#include "UsdBrepUtilities.h"
#include "UsdBrepTokens.h"
#include "UsdBrepArrayData.h"
#include "UsdBrepWrite.h"
#include "UsdBrepRead.h"

#include "UsdBrepDebugTools.h"

#include <cmath>
#include <string>
#include <unordered_map>

// SMLib includes
#include "SmCurve.h"
#include "SmLine.h"
#include "SmSurface.h"
#include "SmBSplineCurve.h"
#include "SmBSplineSurface.h"
#include "SmCircle.h"
#include "SmCone.h"
#include "SmCylinder.h"
#include "SmEllipse.h"
#include "SmPlane.h"
#include "SmSphere.h"
#include "SmTorus.h"
#include "SmBrep.h"
#include "SmBrepData.h"
#include "SmEdge.h"
#include "SmFace.h"
#include "SmVertex.h"
#include "SmLoop.h"
#include "SmFaceGrid.h"
#include "SmPoly.h"
#include "SmTess.h"
#include "SmPrimitiveCreation.h"
#include "nurbs.h"
#include "SmNurbsSrf.h"
#include "SmAssertArray.h"
#include "SmMessages.h"
#include "SmAttribute.h"

using namespace pxr;
using namespace UsdBrepData;
using namespace SMU_BrepConvert;

namespace
{

template <typename T>
void AccumulateMaximumTopologyTolerance(const SmTArray<T*>& rTopologies, double& dMaximumTolerance)
{
    for (ULONG ii = 0; ii < rTopologies.GetSize(); ++ii)
    {
        const double dTolerance = static_cast<double>(SmTol::GetZoneTol3d(rTopologies[ii]));
        if (std::isfinite(dTolerance) && dTolerance >= 0.0 && dTolerance > dMaximumTolerance)
        {
            dMaximumTolerance = dTolerance;
        }
    }
}

double GetMaxZoneTol3d(const SmBrep* pBrep)
{
    if (pBrep == nullptr)
    {
        return 0.0;
    }

    double dMaximumTolerance = SmTol::GetZoneTol3d(pBrep);

    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces(sFaces);
    AccumulateMaximumTopologyTolerance(sFaces, dMaximumTolerance);

    SmTArray<SmEdge*> sEdges;
    pBrep->GetEdges(sEdges);
    AccumulateMaximumTopologyTolerance(sEdges, dMaximumTolerance);

    SmTArray<SmVertex*> sVertices;
    pBrep->GetVertices(sVertices);
    AccumulateMaximumTopologyTolerance(sVertices, dMaximumTolerance);

    return dMaximumTolerance;
}

double SnapSMLibAngularBoundaryDeg(double dDegrees)
{
    static constexpr double dSnapTolDeg = 1.0e-9;

    for (int iQuarterTurn = -4; iQuarterTurn <= 4; ++iQuarterTurn)
    {
        const double dBoundary = 90.0 * static_cast<double>(iQuarterTurn);
        if (std::fabs(dDegrees - dBoundary) <= dSnapTolDeg)
        {
            return dBoundary;
        }
    }

    return dDegrees;
}

double UsdRadiansToSMLibDegrees(double dRadians)
{
    return SnapSMLibAngularBoundaryDeg(SM_RAD2DEG(dRadians));
}

void DeleteSmBreps(std::vector<SmBrep*>& rBreps)
{
    for (SmBrep* pBrep : rBreps)
    {
        delete pBrep;
    }
    rBreps.clear();
}

void DeleteConversionAttributes(SmTArray<SmAttribute*>& rAttributes)
{
    for (ULONG ii = 0; ii < rAttributes.GetSize(); ++ii)
    {
        SmAttribute* pAttribute = rAttributes[ii];
        if (pAttribute == NULL)
        {
            continue;
        }

        // These attributes can already be attached to topology when
        // MakeTopologyFromData reports an error. Detach without triggering
        // last-user deletion, then delete the converter-owned attribute once.
        SmTArray<SmAObject*> sUsers;
        pAttribute->GetUsers(sUsers);
        for (ULONG jj = 0; jj < sUsers.GetSize(); ++jj)
        {
            if (sUsers[jj] != NULL)
            {
                sUsers[jj]->RemoveAttribute(pAttribute, TRUE);
            }
        }

        delete pAttribute;
        rAttributes.SetAt(ii, NULL);
    }
    rAttributes.ReSet();
}

SmExtent2d GetBoundedFaceRangeForUsd(const SmFace* pFace, const SmExtent2d& crFaceDomain, bool bBoundUnboundedFaceRanges)
{
    if (!bBoundUnboundedFaceRanges || crFaceDomain.IsBounded() || pFace == NULL || pFace->GetSurface() == NULL)
    {
        return crFaceDomain;
    }

    SmExtent2d sTrimDomain;
    if (pFace->CalculateUVDomainFromUVTrimCurves(sTrimDomain) != SM_SUCCESS || !sTrimDomain.IsBounded())
    {
        return crFaceDomain;
    }

    // Match the face-domain calculation from SmBrep::ShrinkGeometry(), but
    // keep the result local to BrepArray face:range authoring.
    SmExtent2d sSurfDomain = pFace->GetSurface()->GetNaturalUVDomain();
    double dTol = sTrimDomain.GetSize().Length() / 1000.0;

    SmPoint2d sMin = sTrimDomain.GetMin();
    SmPoint2d sMax = sTrimDomain.GetMax();
    if (sMin.x - sSurfDomain.GetMin().x < dTol)
        sMin.x = sSurfDomain.GetMin().x;
    if (sMin.y - sSurfDomain.GetMin().y < dTol)
        sMin.y = sSurfDomain.GetMin().y;
    if (sSurfDomain.GetMax().x - sMax.x < dTol)
        sMax.x = sSurfDomain.GetMax().x;
    if (sSurfDomain.GetMax().y - sMax.y < dTol)
        sMax.y = sSurfDomain.GetMax().y;

    SmExtent2d sBoundedDomain(sMin, sMax);
    return sBoundedDomain.IsBounded() ? sBoundedDomain : crFaceDomain;
}

} // namespace

/****************************************************************************************************************/ /**
 PURPOSE: for SmEUData::m_lNextEU and m_lMateNextEU long indices
 ********************************************************************************************************************/
int32_t MapLocalInt_ToGlobal(int32_t iLocalIndex, uint32_t uiGlobalStartIndex)
{
    if (iLocalIndex == SM_UNDEF_ULONG || iLocalIndex == USDBREP_NO_OBJECT_INDEX)
    {
        TF_RUNTIME_ERROR("MapLocalInt_ToGlobal error: input GlobalIndex set to SM_UNDEF_ULONG or USDBREP_NO_OBJECT_INDEX value");
    }

    return (iLocalIndex >= 0) ? (iLocalIndex + static_cast<int32_t>(uiGlobalStartIndex)) :
                                (iLocalIndex - static_cast<int32_t>(uiGlobalStartIndex));

} // end MapLocalInt_ToGlobal

/****************************************************************************************************************/ /**
 PURPOSE: convert std::String to TCHAR for windows and linux

 NOTES:  TCHAR = wchar_t for _UNICODE on Windows and char otherwise

 PARAMETERS:
 ********************************************************************************************************************/
void StdStringToTCHAR(const std::string& input, TCHAR* output, const size_t MaxOutputSize)
{
    if (output == NULL || MaxOutputSize == 0) { return; }

    // smos_ToTChar picks the TCHAR width; copy into the caller buffer, truncate,
    // null-terminate.
    std::basic_string<TCHAR> sConverted = smos_ToTChar(input.c_str());
    const size_t nCopied = sConverted.copy(output, MaxOutputSize - 1);
    output[nCopied] = static_cast<TCHAR>(0);

} // end StdStringToTCHAR

/*****************************************************************************************************************/ /**
 PURPOSE: send char buffers to targets

 NOTES: 1. Targets
            on windows   in debug - tgt OutputDebugString()
            on Linux/MAC in debug - tgt stderr
            if(b_OutputLong)      - tgt LongFile (if pBuff use pBuff else pBuffForFile)
            if(b_OutputThin)      - tgt ThinFile (if pBuffForFile use pBuffForFile else pBuff)
        2. Side Effects

 PARAMETERS:
    pBuff        : in : output string sent to stderr also echoed to stdout when buffForFile == NULL

pBuffForFile : in : When not NULL string sent to stdout instead of pBuff, default NULL
 ********************************************************************************************************************/
void smu_WriteBuffer(const TCHAR* pBuff, const TCHAR* pBuffForFile)
{
    // check input
    bool bInput = (pBuff || pBuffForFile);

    // no work - no input
    if (bInput == false)
    {
        return;
    }

// Write to stdout when on Windows and in debug
#if defined(_WIN32) && defined(_DEBUG)
    if (pBuff)
    {
        OutputDebugString((LPCTSTR)pBuff);
    }
    else if (pBuffForFile)
    {
        OutputDebugString((LPCTSTR)pBuffForFile);
    }
#endif // _WIN32 && _DEBUG

// Write to stderr when not on Windows (Linux, macOS) and in debug
#if !defined(_WIN32) && defined(_DEBUG)
    if (pBuff)
    {
        SM_FPRINTF(stderr, _T( "%s" ), pBuff);
    }
    else if (pBuffForFile)
    {
        SM_FPRINTF(stderr, _T( "%s" ), pBuffForFile);
    }
    fflush(stderr);
#endif // no WIN3d && _DEBUG

#if 0
    // when writing to OutputLong file - Echo pBuff messages to OutputLong file and if pBuff is NULL try pBuffForFile
    if (b_OutputLong && bInput)
    {
        // Direct stdout to sOutputLongFile
        usdBrep_ReDirectStdOutToFile(sOutputLongFile);

        // if pBuff exists write it to file buffer  - else try pBuffForFile
        if (pBuff)
        {
            USDBREP_PRINTF(pBuff);
        }
        else if (pBuffForFile)
        {
            USDBREP_PRINTF(pBuffForFile);
        }

        // flush file buffer
        fflush(stdout);
    } // end Output LongFile check

    // when writing to OutputThin file - Echo pBuffForFile messages to OutputThin file and if pBuffForFile is NULL try
    // pBuff
    if (b_OutputThin && bInput)
    {
        // Direct stdout to sOutputThinFile
        usdBrep_ReDirectStdOutToFile(sOutputThinFile);

        // if pBuffForFile exists write it to file buffer  - else try pBuff
        if (pBuffForFile)
        {
            USDBREP_PRINTF(pBuffForFile);
        }
        else if (pBuff)
        {
            USDBREP_PRINTF(pBuff);
        }

        // flush file buffer
        fflush(stdout);
    } // end Output thin file check
#endif

} // end smu_WriteBuffer

/*******************************************************************//**
PURPOSE: return SMLib total vertex count for given BrepArrayData

NOTES: total SMLib_VertexCount = Usd_ShelVertexCount + Usd_VertexCount
***********************************************************************/
uint32_t smu_SMlibTotalVertexCount(const UsdBrepArrayData &rArrays) 
{
  return rArrays.TotalShellVertexCount() + rArrays.TotalVertexCount();
}

/*******************************************************************//**
PURPOSE: return SMLib total edge count for given BrepArrayData

NOTES: total SMLib_EdgeCount    = Usd_EdgeCount + Usd_WireEdgeCount
***********************************************************************/
uint32_t smu_SMLibTotalEdgeCount(const UsdBrepArrayData &rArrays) 
{
  return rArrays.TotalEdgeCount() + rArrays.TotalWireEdgeCount();
} 

/*******************************************************************//**
PURPOSE: return SMLib total edgeuse count for given BrepArrayData

NOTES:  total SMLib_EdgeuseCount = 2 * Usd_EdgeuseCount + Usd_WireEdgeCount + 2 * Usd_LoopVertexCount
***********************************************************************/
uint32_t smu_SMLibTotalEdgeuseCount(const UsdBrepArrayData &rArrays) 
{
  return 2 * rArrays.TotalEdgeuseCount() + rArrays.TotalWireEdgeCount() + 2 * rArrays.TotalLoopVertexCount();
} 

/*******************************************************************//**
PURPOSE: Pretty print SMLib vertex, edge, and edgeuse counts for given BrepArrayData

NOTES: 
***********************************************************************/
void Dump_SMLibCounts(const UsdBrepArrayData& rArrays)
{
  std::string sstring;
  
  sstring = TfStringPrintf("\n    SMLib TotalEdgeuseCount :[%4u] (SMLib TotalEdgeuseCount = ",
      smu_SMLibTotalEdgeuseCount(rArrays));
  usdBrep_WriteString(sstring);
  
  sstring = TfStringPrintf("2 * TotalEdgeuseCount + TotalWireEdgeCount + TotalLoopVertexCount");
  usdBrep_WriteString(sstring);
   
  sstring = TfStringPrintf(
      "\n    SMLib TotalEdgeCount    :[%4u] (SMLib TotalEdgeCount  = TotalEdgeCount  + TotalWireEdgeCount",
      smu_SMLibTotalEdgeCount(rArrays) );
  usdBrep_WriteString(sstring);
  
  sstring = TfStringPrintf(
      "\n    SMLib TotalVertexCount  :[%4u] (SMLib TotalVertexCount  = TotalVertexCount  + TotalShellVertexCount",
      smu_SMlibTotalVertexCount(rArrays));
  usdBrep_WriteString(sstring);
} // end Dump_SMLibCounts

namespace SMU_BrepConvert
{
/*******************************************************************//**
PURPOSE: rtn TRUE when a flat (one entry per multiplicity) knot vector is
         clamped, i.e. the first and last knot values each repeat at least
         (degree + 1) times. Some authoring kernels emit periodic
         curves/surfaces with an unclamped knot vector; SMLib's Bezier
         decomposition assumes clamped knots and reads out of bounds on
         unclamped input.

NOTES: helper for smu_CreateSmBSpline{Curve,Surface}_FromArrays()
***********************************************************************/
static bool smu_IsFlatKnotVectorClamped
 (const SmTArray<double> & crFlatKnots, // in : flat knot vector (one value per multiplicity)
  uint32_t                 iDegree)     // in : polynomial degree in this parametric direction
{
  uint32_t iNum = crFlatKnots.GetSize();

  // Need at least (degree+1) knots at each end to be clamped, i.e.
  // iNum >= 2 * (iDegree + 1). Written as a subtraction/division so a malformed,
  // oversized iDegree cannot overflow uint32 in (iDegree + 1) * 2 and slip past
  // the guard; this also guarantees iDegree and (iNum - 1 - iDegree) index in bounds.
  if (iNum < 2 || iDegree > (iNum - 2) / 2)
    { return false; }

  // clamped => first value repeats (degree+1) times and last value repeats (degree+1) times.
  // Check every entry in each end run rather than just the endpoint pair: comparing only
  // crFlatKnots[0]==crFlatKnots[iDegree] would misclassify a malformed (non-monotonic) flat knot
  // vector as clamped, bypassing CreateCanonical() on exactly the bad-data path we are guarding.
  for (uint32_t ii = 1; ii <= iDegree; ++ii)
    {
      if (crFlatKnots[ii] != crFlatKnots[0] ||
          crFlatKnots[iNum - 1 - ii] != crFlatKnots[iNum - 1])
        { return false; }
    }

  return true;

} // end smu_IsFlatKnotVectorClamped

/*******************************************************************//**
PURPOSE: collapse a flat knot vector (one entry per multiplicity) into the
         unique knot values plus their multiplicities, as expected by
         SmBSpline{Curve,Surface}::CreateCanonical().

NOTES: helper for smu_CreateSmBSpline{Curve,Surface}_FromArrays()
***********************************************************************/
static void smu_CompressFlatKnots
 (const SmTArray<double> & crFlatKnots,    // in : flat knot vector (one value per multiplicity)
  SmTArray<double>       & rUniqueKnots,   // out: unique knot values
  SmTArray<ULONG>        & rMultiplicities)// out: multiplicity for each unique knot value
{
  rUniqueKnots.SetSize(0);
  rMultiplicities.SetSize(0);

  for (uint32_t ii = 0; ii < crFlatKnots.GetSize(); ii++)
    {
      double dKnot = crFlatKnots[ii];
      if (rUniqueKnots.GetSize() != 0 && rUniqueKnots[rUniqueKnots.GetSize() - 1] == dKnot)
        {
          rMultiplicities[rMultiplicities.GetSize() - 1] += 1;
        }
      else
        {
          rUniqueKnots.Add(dKnot);
          rMultiplicities.Add(1);
        }
    }

} // end smu_CompressFlatKnots

/*******************************************************************//**
PURPOSE: rtn SmBSplineCurve Obj built from BrepCurve array data

NOTES: helper for smu_Create...() functions
***********************************************************************/
static SmBSplineCurve * smu_CreateSmBSplineCurve_FromArrays
 (const SmContext           & crContext,
  const SmTArray<SmPoint3d> & crControlPoints,
  const SmTArray<double>    & crWeights,
  const SmTArray<double>    & crKnots,
        uint32_t              iDegree,
        uint32_t              iDim)
{
    // SmLib locals
    uint32_t lDim = iDim;

    // Allocate memory for curve given degree,
    // number of knots and number of control points

    uint32_t lNumKnots = crKnots.GetSize();
    uint32_t lNumControlPoints = crControlPoints.GetSize();

    // Periodic curves (e.g. UV trim curves on periodic surfaces) arrive with an
    // unclamped knot vector. SMLib's Bezier decomposition assumes clamped knots. When
    // unclamped, route through CreateCanonical(), which clamps the curve to its valid
    // parametric domain before constructing it.
    if (!smu_IsFlatKnotVectorClamped(crKnots, iDegree))
    {
        SmTArray<double> sUniqueKnots;
        SmTArray<ULONG>  sMults;
        smu_CompressFlatKnots(crKnots, sUniqueKnots, sMults);

        SmBSplineCurve* pClampedCurve = NULL;
        SmStatus eStatus = SmBSplineCurve::CreateCanonical
          (crContext, iDim, iDegree, crControlPoints, SM_CF_UNSPECIFIED,
           sMults, sUniqueKnots, SM_KT_UNSPECIFIED, &crWeights, NULL, pClampedCurve);

        if (eStatus != SM_SUCCESS)
        { SERN(SM_ERR); }

        return pClampedCurve;
    }

    gw_CURVE* cur = sm_AllocateNurbCurve(lNumControlPoints - 1, (gw_DEGREE)iDegree, lNumKnots - 1);

    if (cur == NULL)
    {
        SERN(SM_ERR);
    }

    // curve locals
    gw_CPOINT* Pw;
    gw_REAL* U;
    N_CrvGetCPtsAndKnots(cur, &Pw, &U);

    if (iDim == 3)
    {
      for (uint32_t ii = 0; ii < lNumControlPoints; ii++)
      {
          SmPoint3d sCPt = crControlPoints[ii];
          if (crWeights[ii] == 1.0)
          {
              N_CPtFromWxWyWz(sCPt.x, sCPt.y, sCPt.z, crWeights[ii], &Pw[ii]);
          }
          else
          {
              N_CPtFromWxWyWz(
                  crWeights[ii] * sCPt.x, crWeights[ii] * sCPt.y, crWeights[ii] * sCPt.z, crWeights[ii], &Pw[ii]);
          }
      }
    }
    else
    {
      for (uint32_t ii = 0; ii < lNumControlPoints; ii++)
      {
          SmPoint3d sCPt = crControlPoints[ii];
          if (crWeights[ii] == 1.0)
          {
              N_CPtFromWxWyWz(sCPt.x, sCPt.y, NL_NOZ, 1.0, &Pw[ii]);
          }
          else
          {
              N_CPtFromWxWyWz(crWeights[ii] * sCPt.x, crWeights[ii] * sCPt.y, NL_NOZ, crWeights[ii], &Pw[ii]);
          }
      }
    }

    for (uint32_t ii = 0; ii < lNumKnots; ii++)
    {
      U[ii] = crKnots[ii];
    }

    // SetFromGwNurb copies the temporary NURB, so release that temporary on
    // both success and failure instead of leaking one allocation per imported
    // NURBS curve.
    SmBSplineCurve* pSmCurve = new (crContext) SmBSplineCurve(lDim);
    const SmStatus sSetStatus = pSmCurve->SetFromGwNurb(0, cur);
    smos_Free(cur);
    if (sSetStatus != SM_SUCCESS)
      {
        delete pSmCurve;
        return NULL;
      }

    return pSmCurve;

} // end smu_CreateSmBSplineCurve_FromArrays

/*******************************************************************//**
PURPOSE: rtn SmBSplineSurface Obj built from BrepSurface array data

NOTES: helper for smu_Create...() functions
***********************************************************************/
static SmBSplineSurface * smu_CreateSmBSplineSurface_FromArrays
 (const SmContext           & crSmContext,          // in : Context in which the surface is created                       
  const SmTArray<SmPoint3d> & crCPts,
  const SmTArray<double>    & crWeights,
  const SmTArray<double>    & crUKnots,
  const SmTArray<double>    & crVKnots,
        uint32_t              iUDegree,
        uint32_t              iVDegree)
{
  // Gathering parameters for surface construction
  // 
  // USD stores 'Deg+1' knots at each end, for clamped knots.
  // Knots and Mults: Get Knots and calculate Mults from them.
  uint32_t iNumUKnots = crUKnots.GetSize();
  uint32_t iNumVKnots = crVKnots.GetSize();

  // Control Point Counts:
  uint32_t iNumCPtsU = iNumUKnots - iUDegree - 1;
  uint32_t iNumCPtsV = iNumVKnots - iVDegree - 1;

  // Periodic surfaces arrive with an unclamped knot vector. SMLib's Bezier
  // decomposition (sm_DecomposeSrf) assumes clamped knots and reads out of bounds
  // on unclamped input. When unclamped, route through CreateCanonical(), which
  // clamps the surface to its valid parametric domain before constructing it.
  if (!smu_IsFlatKnotVectorClamped(crUKnots, iUDegree) ||
      !smu_IsFlatKnotVectorClamped(crVKnots, iVDegree))
    {
      SmTArray<double> sUUniqueKnots, sVUniqueKnots;
      SmTArray<ULONG>  sUMults,       sVMults;
      smu_CompressFlatKnots(crUKnots, sUUniqueKnots, sUMults);
      smu_CompressFlatKnots(crVKnots, sVUniqueKnots, sVMults);

      SmBSplineSurface* pClampedSurf = NULL;
      SmStatus eStatus = SmBSplineSurface::CreateCanonical
        (crSmContext, iUDegree, iVDegree, crCPts, SM_SF_UNSPECIFIED,
         sUMults, sVMults, sUUniqueKnots, sVUniqueKnots, SM_KT_UNSPECIFIED,
         &crWeights, NULL, pClampedSurf);

      if (eStatus != SM_SUCCESS)
        { return NULL; }

      return pClampedSurf;
    }

  // allocate Sur memory
  gw_SURFACE* sur = sm_AllocateNurbSurface(iNumCPtsU - 1, iNumCPtsV - 1, iUDegree, iVDegree, iNumUKnots - 1, iNumVKnots - 1);

  // surface locals
  gw_CPOINT** Pw;
  gw_REAL *U, *V;
  N_SrfGetCPtsAndKnots(sur, &Pw, &U, &V);

  // Gather all control points move from vector to array
  //  Vec crCPts = [Puov0, Puov1, ..., PuovM, ..., PuMv0, PuMv1, ..., PuMvN]
  // maps to Array Pw = [ P[u0][v0], P[u0][v1], ..., P[u0][vN], ]
  //                    [ . . .,                                ]
  //                    [ P[uM][v0], P[uM][V1], ..., P[uM][vN]  ]
  for (uint32_t ii = 0, usdIdx=0; ii < iNumCPtsU; ii++)
  {
      for (uint32_t jj = 0; jj < iNumCPtsV; jj++, usdIdx++)
        {
            NL_REAL x(crCPts[usdIdx][0]);
            NL_REAL y(crCPts[usdIdx][1]);
            NL_REAL z(crCPts[usdIdx][2]);
            NL_REAL w(crWeights[usdIdx]);
      
            N_CPtFromWxWyWz(w * x, w * y, w * z, w, &Pw[ii][jj]);
        } // end iter every conrol point i
  } // end iter every control point j

  // Gather all U knots
  for (uint32_t ii = 0; ii < iNumUKnots; ii++)
  {
        NL_REAL dUKnt = crUKnots[ii];
        U[ii] = dUKnt;
  }

  // Gather all V knots
  for (uint32_t ii = 0; ii < iNumVKnots; ii++)
  {
        NL_REAL dVKnt = crVKnots[ii];
        V[ii] = dVKnt;
  }

  SmBSplineSurface* pNewBSSurf = new (crSmContext) SmBSplineSurface(sur, FALSE, &crSmContext);

  return pNewBSSurf;

} // end smu_CreateSmBSplineSurface_FromArrays

/*******************************************************************//**
PURPOSE: find SmEdgeData obj for tgt SmEdge

NOTES: 1. helper for BrepAppend_SMLibToUsd() function.
       2. works for Edges and WireEdges - just pass in appropriate arrays
       3. returns NULL ptr when pEdge is not found in vEdges
***********************************************************************/
static SmEdgeData * smu_FindEdgeDataObj_ForSmEdge
 (SmEdge               * pEdge,  // in : Tgt SmEdge to find within desired SmEdgeData 
  SmTArray<SmEdgeData> & vEdges) // in : Array of SmEdgeData objs to search
{
  // linear search - yuck
  for(uint32_t ii=0;ii<vEdges.GetSize();ii++)
    {
      if(vEdges[ii].m_pEdge == pEdge)
        { return &(vEdges[ii]) ; }
    }

  // tgt not in array
  return(NULL) ;

} // end smu_FindEdgeDataObj_ForSmEdge 

/*******************************************************************//**
PURPOSE: rtn 3d SmBSplineCurve built from UsdBrepArrayData for Tgt EdgeIndex
         or NULL for errors

NOTES: required: m_lEdgeBSplineCurve3d StartIndex vals set for iEdgeIndex.
***********************************************************************/
static SmBSplineCurve * smu_CreateSmBSplineCurve3d_ForEdgeIndex
 ( const SmContext           & crContext,      // in : SmContext for new object construction
   const UsdBrepArrayData    & rArrays,        // in : UsdBrepArrayData
   UsdBrepArraySpans         & rSpans,         // in : rSpans.StartIndices set for rBrep_ii, and EdgeBSplineSurface[iCurveIndex] arrays
   uint32_t                    iEdgeIndex)     // in : Global EdgeIndex=EdgeCurveIndex to convert to SmBSplineCurve3d
{                                
  (void)iEdgeIndex;

    const VtArray<GfVec3d>  & sCVsArray         = rArrays.m_sEdge_CurveNurb_ControlVerticesArray; 
    const VtArray<uint32_t> & sOrdersArray      = rArrays.m_sEdge_CurveNurb_OrderArray;    
    const VtArray<uint32_t> & sVertexCountArray = rArrays.m_sEdge_CurveNurb_VertexCountArray;            
    const VtArray<double >  & sKnotsArray       = rArrays.m_sEdge_CurveNurb_KnotsArray;           
    const VtArray<double >  & sWeightsArray     = rArrays.m_sEdge_CurveNurb_WeightsArray;         

    size_t lVertexIndex = rSpans.m_lEdgeBSplineCurve3d_ControlVerticesStartIndex ;             
    size_t lWeightIndex = rSpans.m_lEdgeBSplineCurve3d_ControlVerticesStartIndex ;
    size_t lKnotIndex   = rSpans.m_lEdgeBSplineCurve3d_KnotStartIndex ;         

    // Use the NURB-specific start index for Order/VertexCount arrays, which only contain
    // entries for NURB edges (not circle/line/ellipse edges)
    uint32_t iNurbIndex = rSpans.m_lEdgeBSplineCurve3d_StartIndex;

    if (iNurbIndex >= sOrdersArray.size() || iNurbIndex >= sVertexCountArray.size())
      {
        SM_ASSERT_MSG(FALSE,
                      _T("smu_CreateSmBSplineCurve3d_ForEdgeIndex error: input iEdgeIndex is out of bounds - this is a bug")) ;
        return NULL;
      }

    uint32_t iDimension   = 3;
    uint32_t iDeg         = sOrdersArray[iNurbIndex] - 1;
    uint32_t iVertexCount = sVertexCountArray[iNurbIndex];
    uint32_t iKnotCount   = iVertexCount + iDeg + 1;

    SmTArray<SmPoint3d> sCPts   (iVertexCount);
    SmTArray<double>    sWeights(iVertexCount);
    SmTArray<double>    sKnots(iKnotCount);

    // Set CPts
    for (uint32_t jj = 0; jj < iVertexCount; ++jj, ++lVertexIndex)
    {
        const double* cdpCV = sCVsArray[lVertexIndex].data();
        sCPts.Add(SmPoint3d(cdpCV[0], cdpCV[1], cdpCV[2]));
    }

    // Set Knots
    for (uint32_t jj = 0; jj < iKnotCount; ++jj, ++lKnotIndex)
    {
        sKnots.Add(sKnotsArray[lKnotIndex]);
    }

    // Set weights, default to 1.0
    if ( sWeightsArray.size() != sCVsArray.size() )
    {
        sWeights.InsertAt(0, 1.0, iVertexCount);
    }
    else
    {
        for (uint32_t jj = 0; jj < iVertexCount; ++jj, ++lWeightIndex)
        {
            sWeights.Add(sWeightsArray[lWeightIndex]);
        }
    }

    // build and return SmBSplineCurve obj
    SmBSplineCurve * pCurve = smu_CreateSmBSplineCurve_FromArrays(crContext, sCPts, sWeights, sKnots, iDeg, iDimension); 

    // increment start indices
    rSpans.m_lEdgeBSplineCurve3d_StartIndex++ ;
    rSpans.m_lEdgeBSplineCurve3d_ControlVerticesStartIndex += iVertexCount ;             
    rSpans.m_lEdgeBSplineCurve3d_KnotStartIndex += iKnotCount ;         

    return pCurve;

} // end smu_CreateSmBSplineCurve3d_ForEdgeIndex

/*******************************************************************//**
PURPOSE: rtn 3d SmBSplineCurve built from UsdBrepArrayData for Tgt WireEdgeIndex
         or NULL for errors.

NOTES: required: m_lWireEdgeBSplineCurve3d StartIndex vals set for iWireEdgeIndex.
***********************************************************************/
static SmBSplineCurve * smu_CreateSmBSplineCurve3d_ForWireEdgeIndex
 ( const SmContext           & crContext,      // in : SmContext for new object construction
   const UsdBrepArrayData    & rArrays,        // in : UsdBrepArrayData
   UsdBrepArraySpans         & rSpans,         // in : rSpans.StartIndices set for rBrep_ii, and WireEdgeBSplineSurface[iWireEdgeIndex] arrays
   uint32_t                    iWireEdgeIndex) // in : Global WireEdgeIndex = WireEdgeCurve to convert to SmBSplineCurve
{                                
  (void)iWireEdgeIndex;

    const VtArray<GfVec3d>  & sCVsArray         = rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray; 
    const VtArray<uint32_t> & sOrdersArray      = rArrays.m_sWireEdge_CurveNurb_OrderArray;    
    const VtArray<uint32_t> & sVertexCountArray = rArrays.m_sWireEdge_CurveNurb_VertexCountArray;            
    const VtArray<double >  & sKnotsArray       = rArrays.m_sWireEdge_CurveNurb_KnotsArray;           
    const VtArray<double >  & sWeightsArray     = rArrays.m_sWireEdge_CurveNurb_WeightsArray;         

    size_t lVertexIndex = rSpans.m_lWireEdgeBSplineCurve3d_ControlVerticesStartIndex ;             
    size_t lWeightIndex = rSpans.m_lWireEdgeBSplineCurve3d_ControlVerticesStartIndex ;
    size_t lKnotIndex   = rSpans.m_lWireEdgeBSplineCurve3d_KnotStartIndex ;         

    // Use the NURB-specific start index for Order/VertexCount arrays, which only contain
    // entries for NURB wire edges (not circle/line/ellipse wire edges)
    uint32_t iNurbIndex = rSpans.m_lWireEdgeBSplineCurve3d_StartIndex;

    if (iNurbIndex >= sOrdersArray.size() || iNurbIndex >= sVertexCountArray.size())
      {
        SM_ASSERT_MSG(FALSE,
                      _T("smu_CreateSmBSplineCurve3d_ForWireEdgeIndex error: input iWireEdgeIndex is out of bounds - this is a bug")) ;
        return NULL;
      }

    uint32_t iDimension   = 3;
    uint32_t iDeg         = sOrdersArray[iNurbIndex] - 1;
    uint32_t iVertexCount = sVertexCountArray[iNurbIndex];
    uint32_t iKnotCount   = iVertexCount + iDeg + 1;

    SmTArray<SmPoint3d> sCPts   (iVertexCount);
    SmTArray<double>    sWeights(iVertexCount);
    SmTArray<double>    sKnots(iKnotCount);

    // Set CPts
    for (uint32_t jj = 0; jj < iVertexCount; ++jj, ++lVertexIndex)
    {
        const double* cdpCV = sCVsArray[lVertexIndex].data();
        sCPts.Add(SmPoint3d(cdpCV[0], cdpCV[1], cdpCV[2]));
    }

    // Set Knots
    for (uint32_t jj = 0; jj < iKnotCount; ++jj, ++lKnotIndex)
    {
        sKnots.Add(sKnotsArray[lKnotIndex]);
    }

    // Set weights, default to 1.0
    if ( sWeightsArray.size() != sCVsArray.size() )
    {
        sWeights.InsertAt(0, 1.0, iVertexCount);
    }
    else
    {
        for (uint32_t jj = 0; jj < iVertexCount; ++jj, ++lWeightIndex)
        {
            sWeights.Add(sWeightsArray[lWeightIndex]);
        }
    }

    SmBSplineCurve * pBSplineCurve = smu_CreateSmBSplineCurve_FromArrays(crContext, sCPts, sWeights, sKnots, iDeg, iDimension); 

    // increment start indices
    rSpans.m_lWireEdgeBSplineCurve3d_StartIndex++ ;
    rSpans.m_lWireEdgeBSplineCurve3d_ControlVerticesStartIndex += iVertexCount ;
    rSpans.m_lWireEdgeBSplineCurve3d_KnotStartIndex += iKnotCount;         

    return pBSplineCurve;

} // end smu_CreateSmBSplineCurve3d_ForWireEdgeIndex

/*******************************************************************//**
PURPOSE: rtn 2d SmBSplineCurve built from UsdBrepArrayData for Tgt EdgeuseIndex
         or NULL for errors.

NOTES: required: m_lEdgeBSplineCurve2d StartIndex vals set for iEdgeuseIndex.
***********************************************************************/
static SmBSplineCurve *  smu_CreateSmBSplineCurve2d_ForEdgeuseIndex
 ( const SmContext           & crContext,      // in : SmContext for new object construction
   const UsdBrepArrayData    & rArrays,        // in : UsdBrepArrayData
   UsdBrepArraySpans         & rSpans,         // in : rSpans.StartIndices set for rBrep_ii, and EdgeuseBSplineSurface[iEdgeuseIndex] arrays
   uint32_t                    iEdgeuseIndex,   // in : Global edgeuseIndex = edgeuseCurveIndex to convert to SmBSplineCurve2d
   const TfToken             & rFaceSurfaceType,// in : owner face surface type; only NURBS-face UV curves are imported
   SmBoolean                   bEdgeuseSameOrientation) // in : TRUE when the USD UV curve follows the edge's curve direction
{                                
    // locals                    
    const VtArray<GfVec2d>  & sCVsArray         = rArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray; 
    const VtArray<uint32_t> & sOrdersArray      = rArrays.m_sEdgeuse_CurveNurb_OrderArray;    
    const VtArray<uint32_t> & sVertexCountArray = rArrays.m_sEdgeuse_CurveNurb_VertexCountArray;            
    const VtArray<double >  & sKnotsArray       = rArrays.m_sEdgeuse_CurveNurb_KnotsArray;           
    const VtArray<double >  & sWeightsArray     = rArrays.m_sEdgeuse_CurveNurb_WeightsArray;         

    size_t lVertexIndex = rSpans.m_lEdgeuseBSplineCurve2d_ControlVerticesStartIndex ;             
    size_t lWeightIndex = rSpans.m_lEdgeuseBSplineCurve2d_ControlVerticesStartIndex ;
    size_t lKnotIndex   = rSpans.m_lEdgeuseBSplineCurve2d_KnotStartIndex ;         

    // check input - valid iEdgeIndex for all arrays - a bad EdgeIndex here is a bug not an option
    if (iEdgeuseIndex >= sOrdersArray.size() || iEdgeuseIndex >= sVertexCountArray.size())
      {
        SM_ASSERT_MSG(FALSE,
                      _T("smu_CreateSmBSplineCurve2d_ForEdgeuseIndex error: input iEdgeIndex is out of bounds - this is a bug")) ;
        return NULL;
      }

    // no work - no UVTrimCurve
    if (sOrdersArray[iEdgeuseIndex] == 0 || sVertexCountArray[iEdgeuseIndex] == 0)
    { return NULL; }

    uint32_t iDimension   = 2;
    uint32_t iDeg         = sOrdersArray[iEdgeuseIndex] - 1;
    uint32_t iVertexCount = sVertexCountArray[iEdgeuseIndex];
    uint32_t iKnotCount   = iVertexCount + iDeg + 1;

    const auto fAdvancePackedCurve = [&]()
      {
        rSpans.m_lEdgeuseBSplineCurve2d_ControlVerticesStartIndex += iVertexCount ;
        rSpans.m_lEdgeuseBSplineCurve2d_KnotStartIndex += iKnotCount ;
      };

    if (lVertexIndex + iVertexCount > sCVsArray.size() || lKnotIndex + iKnotCount > sKnotsArray.size())
      {
        SM_ASSERT_MSG(FALSE, _T("smu_CreateSmBSplineCurve2d_ForEdgeuseIndex: packed UV curve arrays are out of bounds"));
        fAdvancePackedCurve();
        return NULL;
      }

    // Analytic surfaces use a different parameterization from their internal SMLib NURBS form.
    // Discard authored curves and let SMLib construct trims from the 3D edges when they are needed.
    if (rFaceSurfaceType != UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
      {
        fAdvancePackedCurve();
        return NULL;
      }

    SmTArray<SmPoint3d> sCPts   (iVertexCount);
    SmTArray<double>    sWeights(iVertexCount);
    SmTArray<double>    sKnots(iKnotCount);

    for (uint32_t jj = 0; jj < iVertexCount; ++jj)
    {
        const size_t lSourceVertexIndex = lVertexIndex + (bEdgeuseSameOrientation ? jj : iVertexCount - 1 - jj);
        const double* cdpCV = sCVsArray[lSourceVertexIndex].data();
        sCPts.Add(SmPoint3d(cdpCV[0], cdpCV[1], 0));
    }

    // SMLib requires every UV trim curve to use its edge's parameter interval. USD permits an
    // independent NURBS knot interval, and analytic edge ranges are radians in USD but degrees in
    // SMLib, so affinely map the UV knot domain to the converted edge range while preserving shape.
    if (iEdgeuseIndex >= rArrays.m_sEdgeuseEdgeIndexArray.size())
      {
        SM_ASSERT_MSG(FALSE, _T("smu_CreateSmBSplineCurve2d_ForEdgeuseIndex: edgeuse edge index is out of bounds"));
        fAdvancePackedCurve();
        return NULL;
      }
    const uint32_t iGlobalEdgeIndex = rArrays.m_sEdgeuseEdgeIndexArray[iEdgeuseIndex];
    if (static_cast<size_t>(iGlobalEdgeIndex) >= rArrays.m_sEdgeCurveTypeArray.size() ||
        2 * static_cast<size_t>(iGlobalEdgeIndex) + 1 >= rArrays.m_sEdgeRangeArray.size())
      {
        SM_ASSERT_MSG(FALSE, _T("smu_CreateSmBSplineCurve2d_ForEdgeuseIndex: referenced edge range is out of bounds"));
        fAdvancePackedCurve();
        return NULL;
      }
    double dTargetMin = rArrays.m_sEdgeRangeArray[2 * static_cast<size_t>(iGlobalEdgeIndex)];
    double dTargetMax = rArrays.m_sEdgeRangeArray[2 * static_cast<size_t>(iGlobalEdgeIndex) + 1];
    if (rArrays.m_sEdgeCurveTypeArray[static_cast<size_t>(iGlobalEdgeIndex)] == UsdBrepCurveTokens->brepCurve3dCircleAPI
     || rArrays.m_sEdgeCurveTypeArray[static_cast<size_t>(iGlobalEdgeIndex)] == UsdBrepCurveTokens->brepCurve3dEllipseAPI)
      {
        dTargetMin = UsdRadiansToSMLibDegrees(dTargetMin);
        dTargetMax = UsdRadiansToSMLibDegrees(dTargetMax);
      }
    const double dSourceMin = sKnotsArray[lKnotIndex + iDeg];
    const double dSourceMax = sKnotsArray[lKnotIndex + iVertexCount];
    if (!std::isfinite(dSourceMin) || !std::isfinite(dSourceMax) || dSourceMax <= dSourceMin ||
        !std::isfinite(dTargetMin) || !std::isfinite(dTargetMax) || dTargetMax <= dTargetMin)
      {
        SM_ASSERT_MSG(FALSE, _T("smu_CreateSmBSplineCurve2d_ForEdgeuseIndex: UV or edge parameter interval is invalid"));
        fAdvancePackedCurve();
        return NULL;
      }
    const double dParameterScale = (dTargetMax - dTargetMin) / (dSourceMax - dSourceMin);
    for (uint32_t jj = 0; jj < iKnotCount; ++jj)
    {
        const double dSourceKnot = bEdgeuseSameOrientation ?
                                       sKnotsArray[lKnotIndex + jj] :
                                       dSourceMin + dSourceMax - sKnotsArray[lKnotIndex + iKnotCount - 1 - jj];
        sKnots.Add(dTargetMin + (dSourceKnot - dSourceMin) * dParameterScale);
    }

    // Set weights, default to 1.0;
    if (sWeightsArray.size() != sCVsArray.size())
    {
        sWeights.InsertAt(0, 1.0, iVertexCount);
    }
    else
    {
        for (uint32_t jj = 0; jj < iVertexCount; ++jj)
        {
            const size_t lSourceWeightIndex = lWeightIndex + (bEdgeuseSameOrientation ? jj : iVertexCount - 1 - jj);
            sWeights.Add(sWeightsArray[lSourceWeightIndex]);
        }
    }

    SmBSplineCurve * pBSplineCurve = smu_CreateSmBSplineCurve_FromArrays(crContext, sCPts, sWeights, sKnots, iDeg, iDimension); 

    // increment start indices
    fAdvancePackedCurve();

    return pBSplineCurve;

} // end smu_CreateSmBSplineCurve2d_ForEdgeuseIndex

/*******************************************************************//**
PURPOSE: rtn 3d SmBSplineSurface built from UsdBrepArrayData for Tgt FaceIndex
         or NULL for errors.

NOTES: required: m_lFaceBSplineSurface StartIndex vals set for lSurfaceIndex.
***********************************************************************/
static SmBSplineSurface * smu_CreateSmBSplineSurface3d_ForFaceIndex
( const SmContext        & crContext,      // in : SmContext for new object construction
  const UsdBrepArrayData & rArrays,        // in : all brep attribute values from UsdBrepData::BrepReadFromUsdStage()
  UsdBrepArraySpans      & rSpans,         // in : Incremental rArrays first
  uint32_t                 lSurfaceIndex   // in : Index into NURB surface arrays (m_sFace_SurfaceNurb_*)
)                         
{                         
    // Get packed surface attributes
    const VtArray<GfVec3d>  & sCVsArray          = rArrays.m_sFace_SurfaceNurb_ControlVerticesArray;
    const VtArray<uint32_t> & sUVertexCountArray = rArrays.m_sFace_SurfaceNurb_UVertexCountArray;   
    const VtArray<uint32_t> & sVVertexCountArray = rArrays.m_sFace_SurfaceNurb_VVertexCountArray;   
    const VtArray<uint32_t> & sUOrdersArray      = rArrays.m_sFace_SurfaceNurb_UOrderArray;         
    const VtArray<uint32_t> & sVOrdersArray      = rArrays.m_sFace_SurfaceNurb_VOrderArray;         
    const VtArray<double >  & sUKnotsArray       = rArrays.m_sFace_SurfaceNurb_UKnotsArray;         
    const VtArray<double >  & sVKnotsArray       = rArrays.m_sFace_SurfaceNurb_VKnotsArray;         
    const VtArray<double >  & sWeightsArray      = rArrays.m_sFace_SurfaceNurb_WeightsArray;        

    auto validateArrayIndex = [lSurfaceIndex](const auto& rArray, const TCHAR* pName) -> bool
    {
        if (lSurfaceIndex >= rArray.size())
        {
            TCHAR sMessage[SM_TBLOCK_SIZE];
            SM_SPRINTF(
                sMessage,
                _T("Brep surface index %u is missing %s data (array size %u)."),
                lSurfaceIndex,
                pName,
                static_cast<uint32_t>(rArray.size()));
            ERR_MSG(sMessage);
            return false;
        }
        return true;
    };

    if (!validateArrayIndex(sUOrdersArray,      _T("U order"))      ||
      !validateArrayIndex(sVOrdersArray,      _T("V order"))      ||
      !validateArrayIndex(sUVertexCountArray, _T("U vertex count")) ||
      !validateArrayIndex(sVVertexCountArray, _T("V vertex count")))
    {
        return NULL;
    }

    size_t lVertexIndex = rSpans.m_lFaceBSplineSurface_ControlVerticesStartIndex ;
    size_t lWeightIndex = rSpans.m_lFaceBSplineSurface_ControlVerticesStartIndex ;
    size_t lUKnotIndex  = rSpans.m_lFaceBSplineSurface_UKnotStartIndex ;
    size_t lVKnotIndex  = rSpans.m_lFaceBSplineSurface_VKnotStartIndex ;

    uint32_t iUDeg         = sUOrdersArray[lSurfaceIndex] - 1;
    uint32_t iVDeg         = sVOrdersArray[lSurfaceIndex] - 1;
    uint32_t iUVertexCount = sUVertexCountArray[lSurfaceIndex];
    uint32_t iVVertexCount = sVVertexCountArray[lSurfaceIndex];
    uint32_t iUKnotCount   = iUVertexCount + iUDeg + 1;
    uint32_t iVKnotCount   = iVVertexCount + iVDeg + 1;

    uint32_t iNumCPts = iUVertexCount * iVVertexCount;
    SmTArray<SmPoint3d> sCPts   (iNumCPts);
    SmTArray<double>    sWeights(iNumCPts);
    SmTArray<double>    sUKnots(iUKnotCount);
    SmTArray<double>    sVKnots(iVKnotCount);

    // Set CPts
    for (uint32_t jj = 0; jj < iNumCPts; ++jj, ++lVertexIndex)
    {
        const double* cdpCV = sCVsArray[lVertexIndex].data();
        sCPts.Add(SmPoint3d(cdpCV[0], cdpCV[1], cdpCV[2]));
    }

    // Set U knots
    for (uint32_t jj = 0; jj < iUKnotCount; ++jj, ++lUKnotIndex)
    {
        sUKnots.Add(sUKnotsArray[lUKnotIndex]);
    }

    // Set V knots
    for (uint32_t jj = 0; jj < iVKnotCount; ++jj, ++lVKnotIndex)
    {
        sVKnots.Add(sVKnotsArray[lVKnotIndex]);
    }

    // Set weights, default to 1.0
    if (sWeightsArray.size() != sCVsArray.size())
    {
        sWeights.InsertAt(0, 1.0, iNumCPts );
    }
    else
    {
        for (uint32_t jj = 0; jj < iNumCPts; ++jj, ++lWeightIndex)
        {
            sWeights.Add(sWeightsArray[lWeightIndex]);
        }
    }

    SmBSplineSurface * pBSplineSurface = smu_CreateSmBSplineSurface_FromArrays(crContext, 
                                                                              sCPts, 
                                                                              sWeights, 
                                                                              sUKnots, 
                                                                              sVKnots, 
                                                                              iUDeg, 
                                                                              iVDeg);

    // increment start indices
    rSpans.m_lFaceBSplineSurface_ControlVerticesStartIndex += iNumCPts ;
    rSpans.m_lFaceBSplineSurface_UKnotStartIndex += iUKnotCount;
    rSpans.m_lFaceBSplineSurface_VKnotStartIndex += iVKnotCount;

    return pBSplineSurface;

} // end smu_CreateSmBSplineSurface3d_ForFaceIndex

/*******************************************************************//**
PURPOSE: Add smu_AddSmBSplineCurve2d_ToBrepArrayData data to appended arrays

NOTES: works for Edgeuse->UVTrimCurves
***********************************************************************/
static SmStatus smu_AddSmBSplineCurve2d_ToBrepArrayData
 (SmBSplineCurve   * pBSplineCurve2d,    // in : SmEdge being converted to an OmniSolidNurbsCurve, NULL = add empty Curve to Appended arrays
  SmBoolean          bSameOrientation,   // in : TRUE when the edgeuse runs in the same direction as its edge
  UsdBrepArrayData & rArrays)            // i/o: Tgt UsdBrepArrayData to be appended with surface data
{
    // For NULL UVTrimCurves         
    if(pBSplineCurve2d == NULL)      
      {
        // add VertexCount = 0 curve to the m_sEdgeuse_CurveNurb_VertexCountArray to preserve the edgeuse/UVTrimCurve one-to-one relationship
        rArrays.m_sEdgeuse_CurveNurb_VertexCountArray.emplace_back(0) ;
        rArrays.m_sEdgeuse_CurveNurb_OrderArray.emplace_back(0) ;

        // all done
        return(SM_SUCCESS) ;

      } // end pBSplineCurve2d == NULL branch

    // arrive here for nonNULL pBSplineCurve2d cases
    
    // Get the NURBS
    gw_CURVE   * nurb      = pBSplineCurve2d->GetOrCreateGwNurbPointer();
    const bool   bReverse  = bSameOrientation != TRUE; // reverse the curve if the edgeuse runs in the opposite direction as its edge
    NL_CURVE sReversedNurb;
    NL_STACKS SC;
    if (bReverse)
      {
        N_InitNurbs(&SC);
        N_CrvInitArrays(&sReversedNurb);
        if (NL_YES == N_CrvReverse(nurb, &sReversedNurb, &SC))
          {
            N_EndNurbs(&SC);
            return SM_ERR;
          }
        nurb = &sReversedNurb;
      }

    NL_REAL   * knts      = nurb->knt->U;
    uint32_t    lNumKnots = nurb->knt->m + 1;
    NL_DEGREE   lDegree   = nurb->p ; 
    NL_CPOINT * cpts      = nurb->pol->Pw;
    uint32_t    lNumCpts  = nurb->pol->n + 1;
              
    // add VertexCount, Order to appended Arrays
    rArrays.m_sEdgeuse_CurveNurb_VertexCountArray.emplace_back(lNumCpts) ;
    rArrays.m_sEdgeuse_CurveNurb_OrderArray      .emplace_back(lDegree + 1) ;

    // add Knots to appended Arrays
    for (uint32_t i = 0; i < lNumKnots; i++)
      {
        rArrays.m_sEdgeuse_CurveNurb_KnotsArray  .emplace_back(static_cast<double>(knts[i])) ;
      }

    // add Cpts, Weights to appended Arrays
    for (uint32_t i = 0; i < lNumCpts; ++i)
      {
        GfVec2d sCpt(cpts[i].x, cpts[i].y);
    
        double sWeight = 1.0;
        if (cpts[i].w != NL_NOW)
          {
            sCpt /= cpts[i].w;
            sWeight = cpts[i].w;
          }
    
        rArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray.emplace_back(sCpt);
        rArrays.m_sEdgeuse_CurveNurb_WeightsArray        .emplace_back(sWeight);
      }

    if (bReverse)
      { N_EndNurbs(&SC); }

    return SM_SUCCESS;

} // end smu_AddSmBSplineCurve2d_ToBrepArrayData

/*******************************************************************//**
PURPOSE: Add SmBSplineCurve3d data to appended arrays

NOTES: works for Edge->Curves and WireEdge->Curves
       set the tgt input token to oneof:[edge3dNurb, wireEdge3dNurb]
***********************************************************************/
static SmStatus smu_AddSmBSplineCurve3d_ToBrepArrayData
 (TfToken             tEdgeType,         // in : oneof:[edge3dNurb, wireEdge3dNurb]
  SmBSplineCurve    & rBSplineCurve3d,   // in : SmEdge being converted to an OmniSolidNurbsCurve 
  UsdBrepArrayData  & rArrays)           // i/o: Tgt UsdBrepArrayData to be appended with surface data
{
    // Get the NURBS
    gw_CURVE  * nurb      = rBSplineCurve3d.GetOrCreateGwNurbPointer();
    NL_REAL   * knts      = nurb->knt->U;
    uint32_t    lNumKnots = nurb->knt->m + 1;
    NL_DEGREE   lDegree   = nurb->p ; 
    NL_CPOINT * cpts      = nurb->pol->Pw;
    uint32_t    lNumCpts  = nurb->pol->n + 1;
              
    // for Edge->Curve
    if(tEdgeType == UsdBrepCurveTokens->edge3dNurb)
      {
        // add VertexCount, Order to appended Arrays
        rArrays.m_sEdge_CurveNurb_VertexCountArray.emplace_back(lNumCpts) ;
        rArrays.m_sEdge_CurveNurb_OrderArray      .emplace_back(lDegree + 1) ;

        // add Knots to appended Arrays
        for (uint32_t i = 0; i < lNumKnots; i++)
          { rArrays.m_sEdge_CurveNurb_KnotsArray  .emplace_back(static_cast<double>(knts[i])) ; }

        // add Cpts, Weights to appended Arrays
        for (uint32_t i = 0; i < lNumCpts; ++i)
          {
            GfVec3d sCpt(cpts[i].x, cpts[i].y, cpts[i].z);
        
            double sWeight = 1.0;
            if (cpts[i].w != NL_NOW)
              {
                sCpt /= cpts[i].w;
                sWeight = cpts[i].w;
              }
        
            rArrays.m_sEdge_CurveNurb_ControlVerticesArray.emplace_back(sCpt);
            rArrays.m_sEdge_CurveNurb_WeightsArray        .emplace_back(sWeight);
          }
      } // end edge->curve branch

    // else for WireEdge->Curve
    else if(tEdgeType == UsdBrepCurveTokens->wireEdge3dNurb)
      {
        // add VertexCount, Order to appended Arrays
        rArrays.m_sWireEdge_CurveNurb_VertexCountArray.emplace_back(lNumCpts) ;
        rArrays.m_sWireEdge_CurveNurb_OrderArray      .emplace_back(lDegree + 1) ;

        // add Knots to appended Arrays
        for (uint32_t i = 0; i < lNumKnots; i++)
          { rArrays.m_sWireEdge_CurveNurb_KnotsArray  .emplace_back(static_cast<double>(knts[i])) ; }

        // add Cpts, Weights to appended Arrays
        for (uint32_t i = 0; i < lNumCpts; ++i)
          {
            GfVec3d sCpt(cpts[i].x, cpts[i].y, cpts[i].z);
        
            double sWeight = 1.0;
            if (cpts[i].w != NL_NOW)
              {
                sCpt /= cpts[i].w;
                sWeight = cpts[i].w;
              }
        
            rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray.emplace_back(sCpt);
            rArrays.m_sWireEdge_CurveNurb_WeightsArray        .emplace_back(sWeight);
          }

      } // end WireEdge->Curve branch

    return SM_SUCCESS;

} // end smu_AddSmBSplineCurve3d_ToBrepArrayData

/*******************************************************************//**
PURPOSE: Internal function to add an SMLib surface to a UsdBrepArrayData

NOTES:
***********************************************************************/
static SmStatus smu_AddSmBSplineSurface_ToBrepArrayData
 (const SmBSplineSurface  & crBSplineSurface,  // in : SmSurface being converted to an OmniSolidNurbsSurface          
  UsdBrepArrayData        & rArrays)           // i/o: Tgt UsdBrepArrayData to be appended with surface data
{
    // create the opacity texture as an alpha map for the untrimmed surface
    // SmFace * pFace = dynamic_cast<SmFace*>(crSmSurface.GetOwner());

    // Get the surface NURBS
    NL_SURFACE * surf = ((SmBSplineSurface &)crBSplineSurface).GetOrCreateGwNurbPointer();

    // locals
    NL_CNET * cnet       = surf->net;
    NL_REAL * uKnts      = surf->knu->U;
    NL_REAL * vKnts      = surf->knv->U;
    uint32_t  lNumUKnots = surf->knu->m + 1;
    uint32_t  lNumVKnots = surf->knv->m + 1;
    uint32_t  degreeU    = surf->p;
    uint32_t  degreeV    = surf->q;
    uint32_t  numControlPointsInU = cnet->n + 1;
    uint32_t  numControlPointsInV = cnet->m + 1;

    // check state - degenerate surface UV domain
      { 
        double startU = uKnts[0];
        double endU   = uKnts[lNumUKnots - 1];
        
        double startV = vKnts[0];
        double endV   = vKnts[lNumVKnots - 1];
        
        // Check state: surfaces have nonDegenerate knot vectors - else abort
        AERN_MSG((startU < endU) && (startV < endV), SM_ERR, 
                _T("NURBS surface has degenerate knots and can't be converted - aborting conversion."));
      } // end check state - degenerate surface UV domain

    // add VertexCounts to appended arrays
    rArrays.m_sFace_SurfaceNurb_UVertexCountArray.emplace_back(numControlPointsInU);
    rArrays.m_sFace_SurfaceNurb_VVertexCountArray.emplace_back(numControlPointsInV);

    // add Orders to appended arrays. order = degree + 1 values
    rArrays.m_sFace_SurfaceNurb_UOrderArray.emplace_back(degreeU + 1);
    rArrays.m_sFace_SurfaceNurb_VOrderArray.emplace_back(degreeV + 1);

    // add U knots to appended arrays
    for (uint32_t i = 0; i < lNumUKnots; i++)
        rArrays.m_sFace_SurfaceNurb_UKnotsArray.emplace_back(static_cast<double>(uKnts[i]));

    // add V knots to appended arrays
    for (uint32_t i = 0; i < lNumVKnots; i++)
        rArrays.m_sFace_SurfaceNurb_VKnotsArray.emplace_back(static_cast<double>(vKnts[i]));

    // Add control points and weights to appended arrays
    // Gather all control points 
    // move from vector to array: Vec crCPts = [Puov0, Puov1, ..., PuovM, ..., PuMv0, PuMv1, ..., PuMvN]
    //                            maps to Array Pw = [ P[u0][v0], P[u0][v1], ..., P[u0][vN], ]
    //                                               [ . . .,                                ]
    //                                               [ P[uM][v0], P[uM][V1], ..., P[uM][vN]  ]
    for (uint32_t u = 0; u < numControlPointsInU; ++u)
    {
    for (uint32_t v = 0; v < numControlPointsInV; ++v)
    {
            NL_CPOINT control_point = cnet->Pw[u][v];
            GfVec3d sCpt(control_point.x, control_point.y, control_point.z);
            double dWeight = 1.0;

            if (control_point.w != NL_NOW)
            {
                dWeight = control_point.w;
                sCpt /= dWeight;
            }

            rArrays.m_sFace_SurfaceNurb_ControlVerticesArray.emplace_back(sCpt);
            rArrays.m_sFace_SurfaceNurb_WeightsArray.emplace_back(dWeight);
        }
    }

    return SM_SUCCESS;

} // end smu_AddSmBSplineSurface_ToBrepArrayData

/*******************************************************************/ /**
 PURPOSE: Return UsdBrepArrayData compacted local index given
          a SmBrepData local index and the array of 
          indices that have been removed from the list

 NOTES: 
 ***********************************************************************/
int32_t smu_CompactLocalIndex               // rtn: compacted local index to be used when referencing UsdBreArrays that were built by compacting a BrepData array
(int32_t           lBrepData_LocalIndex,    // in : SMlib BrepData local index pointing into an BrepData array that gets compacted into a UsdBrepArray array
 SmTArray<ULONG> & rSmBrep_RemovedIndices)  // in : array of local indices that are removed from the BrepData array when getting compacted into the UsdBrepArray array
{
  // locals
  uint32_t ii ;
  int32_t lBrepArrayData_LocalIndex = lBrepData_LocalIndex;

  // watch for undefined Indices - don't change the magic number
  if(lBrepData_LocalIndex == SM_UNDEF_ULONG)
    { return SM_UNDEF_ULONG ; }

  // for every RemovedIndex
  for(ii=0;ii<rSmBrep_RemovedIndices.GetSize();ii++)
    {
      // when BrepData_LocalIndex is larger than a RemovedIndex
      if (lBrepData_LocalIndex > (int32_t)rSmBrep_RemovedIndices[ii])
        {
          // decrement UsdBrepArrayData_LocalIndex
          lBrepArrayData_LocalIndex--;
        } // end if lBrepData_LocalIndex > RemovedIndex check
    } // end iter RemovedIndex

  // all done
  return lBrepArrayData_LocalIndex ;

} // end smu_CompactLocalIndex

/*******************************************************************//**
PURPOSE: Add a set of SmBreps to an OmniSolidBrepArray prim

NOTES: 
 1. pipeline:     std:vector<SmBrep*>  
              =>  UsdBrepArrayData 
              =>  UsdBrepArray
***********************************************************************/
SmStatus BrepAppend_SMLibToUsd
 (std::vector<SmBrep*>   & rSmBreps,    // i/o: 'from' SmBreps being converted to an OmniSolidBrep
                                        //      (an SmSdfPathAttribute(UsdBrepArraypath) is added to each SmBrep_ii.)
  pxr::UsdPrim& rUsdBrepArray,          // in : read from prims
  bool           bExportUVCurves,       // in : when true, export Edgeuse UV trim curves (UVCurves) to USD
  bool           bBoundUnboundedFaceRanges // in : when true, replace unbounded face ranges with trim-curve bounds in USD only
 )
{
  // Get the usd stage
  pxr::UsdStageRefPtr stage = rUsdBrepArray.GetStage();

  // get the path for the UsdBrepArray
  pxr::SdfPath sBrepArrayPath = rUsdBrepArray.GetPath();

  // Get the UsdBrepArraySpecHandle
  pxr::SdfPrimSpecHandle sUsdBrepArraySpecHandle = stage->GetRootLayer()->GetPrimAtPath(sBrepArrayPath);

  // no work - missing or dormant sUsdBrepArraySpecHandle input
  if (!sUsdBrepArraySpecHandle || sUsdBrepArraySpecHandle->IsDormant())
    {
      SER_MSG(SM_ERR_INVALID_INPUT, _T("no or dormant sUsdBrepArraySpecHandle input."));
    }

  // locals
  SdfPathVector    sPaths         = SdfPathVector{sBrepArrayPath};
  uint32_t         lOldBrepCount  = GetUsdBrepArray_BrepCount(rUsdBrepArray) ; 
  UsdBrepArrayData sArrays ;       // target appendedArray built from BrepData and then copied into the UsdBrepArray
                          
  // kludge - make sure CustomData source is loaded with string "SMLib" used later to skip heal steps
  sUsdBrepArraySpecHandle->SetCustomData(UsdBrepSolidTokens->source, VtValue("SMLib"));
  
  // when OldBrepCount > 0, load existing UsdBrepArray Brep data into sArrays
  if(lOldBrepCount > 0)
    {
      if(!BrepReadFromUsdStage(rUsdBrepArray,  // in : source UsdBrepArray
                              sArrays))         // out: UsdBrepArrayData
        {
          SER_MSG(SM_ERR_INVALID_INPUT, _T("cannot read existing BrepArray data."));
        }
    }
  else // when OldBrepCount == 0 - only load UsdBrepArray metadata
    {
      // UsdBrepArray.GetPrim() path - to be stored in SmBrep->Attributes built from this UsdBrepArrayData
      sArrays.m_sPrimPath = rUsdBrepArray.GetPath() ;
  
      // no work - metadata fields not built here.  They're built in the Brep_ii loop in BrepAppend_SMLibToUsdBrep()
      //   built below: sArrays.m_sBrepArray_BBox
      //    sArrays.m_sCADSource ???
      //   built below: sArrays.m_sBrepArray_MaterialPath
      //   built below: sArrays.m_sBrepMaterial_BrepPathArray       
      //   built below: sArrays.m_sBrepMaterial_BrepIndexArray 
      //   built below: sArrays.m_sFaceMaterial_FaceIndexArray
      //   built below: sArrays.m_sFaceMaterial_FacePathArray      
    } // end no preexisting UsdBrepArray data branch
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe) // dump input: Brep to Output window, BrepData to file:["MoveSmBreps_ToUsdBrepArray_InputBrep0.smb"], UsdBrepArray to Output window
    {
      std::string brepPathString(sBrepArrayPath.GetAsString());
    
      // dump 1st SMLib SmBrep
      if(rSmBreps.size() > 0) 
        { SM_DUMP_AND_ASSERT_VALID(rSmBreps[0]); }
    
      // move 1st SmBrep to BrepData
      SmContext    sContext ; 
      SmBrep     * pDbgBrep_0     = rSmBreps[0] ;
      SmBrepData * pDbgBrepData_0 = new (sContext) SmBrepData(FALSE, SM_DS_USD);
      SmObjDelete sClean(pDbgBrepData_0) ; 
      SmTArray<SmAttribute*> sAttributes ;
      pDbgBrepData_0->FromBrep(*pDbgBrep_0, sAttributes) ; 
    
      // write BrepData to file:["MoveSmBreps_ToUsdBrepArray_InputBrep0.smb"]
      if (usdBrep_CreateOutputFiles())
        {
            std::string sPathString = "OutputFiles/MoveSmBreps_ToUsdBrepArray_InputBrep0.smb";

            // TRUE = open file and rewrite contents, FALSE= open file and append to end
            pDbgBrepData_0->WriteToFile(sPathString, SM_ASCII, TRUE);

            usdBrep_WriteString("\n\n  FILE_WRITE: " + sPathString + "\n");
         
            // Dump input UsdBrepArray
            UsdBrepData::Dump_PrimProperties(rUsdBrepArray); 
        }
     }
#endif // SM_DEBUG_CODE

  // Add SmBreps to UsdBrepArrayData
  SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(rSmBreps, // i/o: in : 'from' SmBreps to convert to a UsdBrepArray
                                                              //      out: an SmSdfPathAttribute(UsdBrepArraypath) is added to each SmBrep_ii.
                                                    sArrays,  // i/o: in : UsdBrepArrayData with preexisting Brep data
                                                              //      out: Augmented with rSmBreps data
                                                    &sPaths,  // in : SdfPathVector of UsdBrepArray paths to be added to each SmBrep_ii as an SMLib attribute
                                                    bExportUVCurves, // in : whether to export Edgeuse UV trim curves (UVCurves)
                                                    bBoundUnboundedFaceRanges); // in : whether to bound unbounded face ranges in USD only
                                   
#ifdef SM_DEBUG_CODE
    if(bDebugMe) // dump UsdBrepArrayData
      {
        // Dump sArrays
        Dump_BrepArrayData(sArrays, TRUE); // TRUE  = also dump array data values, FALSE = dump only Topology obj counts
        Dump_SMLibCounts(sArrays);
      }
#endif // SM_DEBUG_CODE

    { // open scope for ChangeBlock
      // Open the changeblock
      SdfChangeBlock sChangeBlock;
      
      // Move Appended UsdBrepArrayData into rUsdBrepArray
      BrepWriteToUsdStage(sArrays, rUsdBrepArray) ;

#ifdef SM_DEBUG_CODE
      if(bDebugMe) // dump UsdBrepArray - before closing SdfChangeBlock
        {
          // Dump UsdBrepArray
          UsdBrepData::Dump_PrimProperties(rUsdBrepArray); 
        }
#endif // SM_DEBUG_CODE

    } // close scope for ChangeBlock

#ifdef SM_DEBUG_CODE
  if(bDebugMe) // dump UsdBrepArray - after closing SdfChangeBlock
    {
      // Dump UsdBrepArray
      UsdBrepData::Dump_PrimProperties(rUsdBrepArray); 
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end BrepAppend_SMLibToUsd

/*******************************************************************//**
PURPOSE: Add a set of SmBreps to a UsdBrepArrayData struct

NOTES: 
 1. pipeline: for(ii = every input rSmBreps member) 
                {    rSmBreps[ii] 
                  => SmBrepData_ii 
                  => emplace_back(UsdBrepArrayData) 
                }
 2. SmBrepData vs. UsdBrepArrayData organization:
     - SmBrepData       = lists of topology object data for one Brep
     - UsdBrepArrayData = lists of associated arrays of topology object data for an array of Breps
        +------------------++---------------------------------------------------------+
        |   BrepData       ||            UsdBrepArrayData                             |
        |   1 list of      ||                                                         |
        +------------------++---------------------------------------------------------+
        |  Region Objects  || 1 set of associated Region arrays                       |
        |  Shell Objects   || 1 set of associated Shell arrays                        |
        |  Faceuse Objects || 1 set of associated Faceuse arrays                      |
        |  Face Objects    || 1 set of associated Face arrays                         |
        |  Loop Objects    || 1 set of associated Loop arrays                         |
        |  Edgeuse Objects || 1 culled set of associated Edgeuse arrays               |
        |                  ||   wireEdge-Edgeuses removed.                            |
        |    m_vEdgeuses   || == [Edgeuses, 1 Edgeuse per WireEdge]                   |
        |  Edge Objects    || Divided sets of associated Edge and WireEdge arrays     |
        |    m_vEdge       || == [FaceEdges, WireEdges]                               |
        |  Vertex Objects  || Divided sets of associatedShellVertex and Vertex arrays |
        |    m_vVertex     || == [ShellVertices, Vertices]                            |
        +------------------++---------------------------------------------------------+
 3. When moving data from SMBrepData to usdBrepArrayData, the single SmBrepData
    vertex and edge lists are split into the UsdBrepArrayData two pairs of lists,
    the list of Edgeuses is culled of its WireEdge-Edgeuses, and
    all stored indices to vertices, edges, and edgeuses must be adjusted to account for the split.
   a. packing of usdBrepArrayData arrays:
        USD FaceEdges     = FaceEdges of m_vEdges in the order that they appear in m_vEdges
        USD WireEdges     = WireEdges of m_vEdges in the order that they appear in m_vEdges
        USD ShellVertices = ShellVertices of m_vShells in the order that they appear in m_vShells
        USD Vertices      = Edge-Vertices, WireEdge-Vertices, and Loop-Vertices of m_vVertex in the order that they appear in m_vVertex
        USD Edgeuses      = Edge-Edgeuses of m_vEdgeuses in the order that they appear in m_vEdgeuses
   b. Edgeuses:
        SMLib : contains an Edgeuse for every FaceEdge, WireEdge, LoopVertex, and ShellVertex.
        USD   : only has Edgeuses for FaceEdges.
                - BrepAppend_SMLibToUsd()        : Edgeuse data for WireEdges, LoopVertices, and ShellVertices is ignored.
                                                          Edgeuse data for FaceEdges in m_vEdgeuses is copied into USD Edgeuse arrays.
                                                            m_sEdgeuse associated arrays = Edgeuse data packed in the order that Edge_Edgeuses appear in m_vEdges.
                - BrepMove_OneUsdBrepToSMLib(): Edgeuse data for FaceEdges is copied from USD edgeuse arrays and
                                                          Edgeuse data for WireEdge, LoopVertices, and ShellVertices are generated from USD Shell and Vertex lists.
                                                            pBrepData_ii->m_vEdgeuses = [FaceEdge-Edgeuses, new WireEdge-Edgeuses, new ShellVertex-Edgeuses, new LoopVertex-Edgeuses].
   c. Vertices:
        SMLib : stores a single list of {ShellVertices, LoopVertices, EdgeVertices}.
        USD   : stores a list of {ShellVertices} and a list of {LoopVertices, EdgeVertices}
                - BrepAppend_SMLibToUsd()        : the single m_sVertices array data is divided between the USD Vertices and VertexShell data arrays.
                                                           vertex indices from pBrepData_ii->m_vShells[jj]->WireEdges->m_lStartVertex, m_lEndVertex
                                                           and                 pBrepData_ii->m_vLoops[jj]->m_lVertex
                                                           and                 pBrepData_ii->m_vEdges[jj]->m_lStartVertex, m_lEndVertex
                                                           are compacted and stored in: m_sLoopVertexIndexArray, m_sEdgeVertexIndicesArray and m_sWireEdgeVertexIndicesArray
                                                             m_sShell_PointPositionArray  = ShellVertex positions packed in the order that ShellVertices appear in m_vShells.
                                                             m_sVertex_PointPositionArray = LoopVertex, EdgeVertex, and WireEdgeVertex positions packed in the order that LoopVertices, EdgeVertices, and WireEdgeVertices appear in m_vLoops and m_vEdges.
                - BrepMove_OneUsdBrepToSMLib(): The USD ShellVertex and Vertices arrays are combined into one SMLib m_sVertices array.
                                                             one VertexData obj is made for every m_sShell_PointPositionArray entry and added to pBrepData_ii->m_sVertices
                                                             one VertexData obj is made for every m_sVertex_PointPositionArray entry and added to pBrepData_ii->m_sVertices
   d. Edges                                                  pBrepData_ii->m_sVertices = [mixed{USD LoopVertices, USD EdgeVertices, USD WireEdgeVertices}, USD ShellVertices]
        SMLib : stores a single list of {FaceEdges, WireEdges}
        USD   : stores a list of {FaceEdges} and a list of {WireEdges}
                - BrepAppend_SMLibToUsd()        : m_sWireEdge associated arrays = built in Shell block in the order WireEdges appear in the shells of m_vShells
                                                         m_sEdge associated arrays = built from m_vEdges skipping the wire edges
                                                         edge indices from pBrepData_ii->m_vEdgeuses[jj]->m_lEdgeIndex are 
                                                         compacted and stored in: m_sEdgeuseEdgeIndexArray.
                - BrepMove_OneUsdBrepToSMLib(): The USD WireEdge and Edge arrays are combined into one SMLib m_sEdges array.
                                                          one EdgeData obj is made for every m_sEdge associated array object and added to pBrepData_ii->m_sEdges
                                                          one EdgeData obj is made for every m_sWireEdge associated array object and added to pBrepData_ii->m_sEdges
                                                          pBrepData_ii->m_vEdges = [FaceEdges, WireEdges]
***********************************************************************/
SmStatus BrepAppend_SMLibToUsdBrep
 (std::vector<SmBrep*>           & rSmBreps,   // i/o: in : 'from' SmBreps to convert to a UsdBrepArrayData
                                               //      out: a SmSdfPathAttribute( UsdBrepArrayPath ) is added to each SmBrep_ii.
  UsdBrepData::UsdBrepArrayData & rArrays,    // i/o: in : UsdBrepArrayData with preexisting Brep data
                                               //      out: Augmented with rSmBreps data
  SdfPathVector                  * pOptPaths,  // in : optional SdfPathVector of UsdBrepArrayPaths to be added to each SmBrep_ii as an SMLib attribute
  bool                             bExportUVCurves, // in : when true, export Edgeuse UV trim curves (UVCurves)
  bool                             bBoundUnboundedFaceRanges) // in : when true, replace unbounded face ranges with trim-curve bounds in USD only
{                                              //      0x0 to ignore. default:[0x0]
    // locals
    UsdBrepArraySpans sSpans ;                                  // associated index counts and offsets for rArrays
    uint32_t          lNewBrepCount = rSmBreps.size() ;         // number of Breps to add to Arrays
    uint32_t          lOldBrepCount = rArrays.TotalBrepCount() ;

#define MAP_SM_TO_USD_UNUSED_VALUE(a) (((a) == SM_UNDEF_ULONG) ? -1 : (a))

#ifdef SM_DEBUG_CODE
    TCHAR sBuff[SMU_TBLOCK_SIZE];
    SmBoolean bDebugMe = FALSE;
    if(bDebugMe)
      {
        smu_sprintf(sBuff, _T("entering MoveSmBreps_ToBrepArray: newBrepCount:[%d]"), lNewBrepCount);
        smu_WriteBuffer(sBuff);
      }
#endif // SM_DEBUG_CODE

    // for every Brep_ii - add Brep_ii data to rArrays
    for (uint32_t ii = 0; ii < lNewBrepCount; ++ii)
      {
        // locals
        const SmContext      * cpContext    = rSmBreps[ii]->GetContext();
        SmBrep               * pBrep_ii  = rSmBreps[ii] ;
        uint32_t               lBrepIndex   = lOldBrepCount + ii ;
        SmTArray<SmAttribute*> sAttributes; // working array for attributes - // TODO write these to USD
        SmTArray<ULONG>        sSmBrep_VertexShell_VertexIndices ;      // list of all BrepData VertexShells_VertyexIndices - used to map SmBrepData to compacted usd BrepArrayData VertexIndices 
        SmTArray<ULONG>        sSmBrep_WireEdge_CurveIndicies ;         // list of all BrepData WireEdge indices - used to map SmBrepData to compacted usd BrepArrayData WireEdgeIndices 
        SmTArray<ULONG>        sSmBrep_LoopAndWireEdgeType_EUIndicies ; // list of all BrepData Loop_TYPE and WireEdge_TYPE Edgeuses indices - used to map SmBrepData to compacted usd BrepArrayData EdgeuseIndices 

        // Face-range calculation may attach missing UV trims. Prepare it before
        // FromBrep snapshots the trim pointers/indices, so newly generated trims
        // are available to this export as well as later users of the source Brep.
        // Store only unbounded faces; bounded faces retain their authored domain.
        std::unordered_map<const SmFace*, SmExtent2d> sBoundedFaceRanges;
        if (bBoundUnboundedFaceRanges)
          {
            SmTArray<SmFace*> sFaces;
            pBrep_ii->GetFaces(sFaces);
            for (ULONG jj = 0; jj < sFaces.GetSize(); ++jj)
              {
                SmFace* pFace = sFaces[jj];
                const SmExtent2d sDomain = pFace->GetUVDomain();
                if (!sDomain.IsBounded())
                  {
                    sBoundedFaceRanges.emplace(pFace, GetBoundedFaceRangeForUsd(pFace, sDomain, true));
                  }
              }
          }

        // temp BrepData_ii to be built from Brep_ii
        //      note: each SmBrepData topology object has a pointer back to its source SmBrep topology object.
        //            example: SmVertexData::m_pVertex = SmVertex object within pBrep_ii
        //            that gets used frequently below to build SchemaUSD values not explicit in the pBrepData_ii model
        SmBrepData * pBrepData_ii = new (*cpContext) SmBrepData(FALSE, SM_DS_SMLIB);
        SmObjDelete  sClean(pBrepData_ii);

        // build pBrepData_ii from pBrep_ii - with attributes queued into sAttributes
        pBrepData_ii->FromBrep(*pBrep_ii,     // in : target Brep to move to BrepData format                
                               sAttributes);  // i/o: attribute accumulation array

#ifdef SM_DEBUG_CODE
        if (bDebugMe)
          {
            SM_DUMP_AND_ASSERT_VALID(pBrep_ii) ;

             if (usdBrep_CreateOutputFiles())              
               {
                 std::string sPathString = TfStringPrintf("OutputFiles/AddSmBreps_ToUsdBrepArray_InputBrep%d.smb", ii );

                 // TRUE = open file and rewrite contents, FALSE= open file and append to end
                 pBrepData_ii->WriteToFile(sPathString, SM_ASCII, TRUE);

                 usdBrep_WriteString("\n\n  FILE_WRITE: " + sPathString + "\n");
               }

            smgfx_Erase();
            smgfx_SetLook(1,2, 0,0,1); if(pBrep_ii) pBrep_ii->Draw(TRUE) ; sm_GraphicsLoop() ; 
            sm_GraphicsLoop();
          }
#endif // SM_DEBUG_CODE

        // set sSpans.StartIndex values for Brep_ii 
        sSpans.SetStartsForNextBrepAdd( rArrays ) ; 

        // Get number of each object in this Brep
        uint32_t lNumRegions  = pBrepData_ii->m_lNumRegions;             
        uint32_t lNumShells   = pBrepData_ii->m_vShells.GetSize();       
        uint32_t lNumFaceuses = pBrepData_ii->m_vFaceuses.GetSize();     
        uint32_t lNumFaces    = pBrepData_ii->m_vFaces.GetSize();        
        uint32_t lNumLoops    = pBrepData_ii->m_vLoops.GetSize();        
        uint32_t lNumEdgeuses = pBrepData_ii->m_vEdgeuses.GetSize();     
        uint32_t lNumEdges    = pBrepData_ii->m_vEdges.GetSize();        
        uint32_t lNumVertices = pBrepData_ii->m_vVertices.GetSize();
        std::vector<SmBoolean> sLoopExportsUVCurves(lNumLoops, FALSE);
         
        // BrepArray stores the tolerance needed to preserve the source topology.
        // Use the largest effective face, edge, or vertex tolerance
        double sZoneTol3d = GetMaxZoneTol3d(pBrep_ii);
        SmXSectTol3d  sXSectTol3d  = SM_ZONE_TO_XSECTTOL3D(sZoneTol3d);
        SmApproxTol3d sApproxTol3d = SmTol::GetApproxTol3d(sXSectTol3d) ;

        // check state - one-to-one {edges to curves} and {faces to surfaces}
        SM_ASSERT(lNumEdges == pBrepData_ii->m_v3DCurves.GetSize());
        SM_ASSERT(lNumFaces == pBrepData_ii->m_vSurfaces.GetSize());

        // perBrep material properties 
          {
            // SmBrep_ii attributes 
            SmTArray<SmAttribute*> sBrep_ii_Attrs;
            pBrep_ii->GetAttributes(sBrep_ii_Attrs);

            // for every Brep_ii material_binding attribute - insert Brep_ii_Index values into the sBrepMaterials index lists
            for (uint32_t jj = 0; jj < sBrep_ii_Attrs.GetSize(); ++jj)
              {
                SmAttribute        * pAttr     = sBrep_ii_Attrs[jj];
                SmSdfPathAttribute * pPathAttr = SM_CAST_PTR(SmSdfPathAttribute, pAttr);
                // SmUsdXformAttribute* pXformAttr = SM_CAST_PTR(SmUsdXformAttribute, pAttr);

                // when attribute ID == SM_AI_MATERIAL_BINDING and materialTypeCount > 0
                if (   pPathAttr
                    && pAttr->GetAttributeID() == SM_AI_MATERIAL_BINDING
                    && pPathAttr->GetMaterialPaths()->size() > 0)
                  {
                    bool bFoundMatPath = false;
                    // look for an already used material path
                    for (uint32_t kk = 0; kk < rArrays.m_sBrepMaterial_BrepPathArray.size() && !bFoundMatPath; ++kk)
                      {
                        if (rArrays.m_sBrepMaterial_BrepPathArray[kk] == pPathAttr->GetMaterialPaths()->front())
                          {
                            // add this Brep_ii index to the sBrepMaterials[kk] BrepIndex list
                            rArrays.m_sBrepMaterial_BrepIndexArray[kk].emplace_back(lBrepIndex);
                            bFoundMatPath = true;
                          }
                      }

                    // when Brep_ii is using a materialPath not used before
                    if (!bFoundMatPath)
                      {
                        // add Brep_ii materialPath to the sBrepMaterialPaths list
                        rArrays.m_sBrepMaterial_BrepPathArray.emplace_back(pPathAttr->GetMaterialPaths()->front());

                        // place this Brep_ii index into a uint32_t array and add to back of sBrepMaterials array
                        rArrays.m_sBrepMaterial_BrepIndexArray.emplace_back(VtIntArray{(int)lBrepIndex});
                      }
                  } // end attribute is a material_binding attribute check
              } // end iter every Brep_ii_Attribute
          } // end adding perBrep material properties

        // Calculate Brep_ii bounding box (== Usd Extent)
        SmExtent3d         sLocalBBox;
        pBrep_ii->CalculateBoundingBox(sLocalBBox, FALSE,  // FALSE = output BBox = union(vertex, edge, and face BBoxes)...very slow on huge breps 
                                                   FALSE,  // FALSE = compute any box (not minimal box) larger than each contained face and edge (cheaper)
                                                   FALSE); // FALSE = return BBox as computed without an expansion,

        // m_sBrepArray_BBox = union(LocalBBox, GlobalBBox), for all Breps being added to UsdBrepArrayData  
        GfRange3f localRange(GfVec3f(sLocalBBox.GetUMin(), sLocalBBox.GetVMin(), sLocalBBox.GetWMin()),
                             GfVec3f(sLocalBBox.GetUMax(), sLocalBBox.GetVMax(), sLocalBBox.GetWMax()));
        rArrays.m_sBrepArray_BBox = GfRange3f::Union(rArrays.m_sBrepArray_BBox, localRange);

        // Add UsdBrepArray sPath as an SmAttribute to Brep_ii
        if(pOptPaths)
          {
            SmSdfPathAttribute * pAttr  = new (cpContext) SmSdfPathAttribute(SM_AI_BREP_ARRAY, SM_AB_REFERENCE, *pOptPaths );
            pBrep_ii->AddAttribute(pAttr);
          }

        // begin scope: move BrepData_ii data into the rArrays arrays
          {
            // Brep data
              {
               // add Brep object data                     
               rArrays.m_sBrepXSectTol3dArray.emplace_back(sXSectTol3d) ;                    // {Brep XSectTol3d},   one per Brep, sized:[BrepCnt]
               rArrays.m_sBrepExtentArray    .emplace_back(GfVec3d(sLocalBBox.GetUMin(),     // {Brep BBoxMin:{XYZ}, 1st of 2 per Brep, sized:[2*BrepCnt]                         
                                                                   sLocalBBox.GetVMin(),                                                 
                                                                   sLocalBBox.GetWMin()));                            
               rArrays.m_sBrepExtentArray    .emplace_back(GfVec3d(sLocalBBox.GetUMax(),     // {Brep BBoxMax:{XYZ}, 2nd of 2 per Brep                         
                                                                   sLocalBBox.GetVMax(),                                                
                                                                   sLocalBBox.GetWMax()));                            
               rArrays.m_sBrepRegionCountArray.emplace_back(lNumRegions) ;                   // {Brep Region Cnt} one per Brep, sized:[BrepCnt]
              }

            // Regions
            for (uint32_t jj = 0; jj < lNumRegions; ++jj)
              {
                SmRegionData & rRegionData = pBrepData_ii->m_vRegions[jj] ;
                TfToken        sRegionType = (rRegionData.m_bIsVoidFlag) ? UsdBrepSolidTokens->voidRegion   // gwc: rRegionData.m_bIsVoidFlag uses UNSURE for notSet.
                                                                         : UsdBrepSolidTokens->solidRegion; //      usd region:type has no notSet value. defaults to solidRegion
                // add region data
                rArrays.m_sRegionShellCountArray.emplace_back(rRegionData.m_lNumShells);  // {Loop cnt},                            one entry per BrepArray Region, sized:[TotalRegionCnt] 
                rArrays.m_sRegionTypeArray      .emplace_back(sRegionType);               // {oneof:["solidRegion", "voidRegion"]}, one entry per BrepArray Region, sized:[TotalRegionCnt]
              }

            // Shells, WireEdges, WireEdge->Curves, and ShellVertex Points
            for (uint32_t jj = 0; jj < lNumShells; ++jj)
              {
                SmShellData & rShellData = pBrepData_ii->m_vShells[jj] ;
                SmShell     * pShell     = rShellData.m_pShell1 ;

                // locals
                SmTArray<SmFaceuse*> sShellFaceuses ;
                SmTArray<SmEdge*>    sShellWireEdges ;
                pShell->GetFaceuses (sShellFaceuses) ;
                pShell->GetWireEdges(sShellWireEdges) ;

                // shell->faceuse and shell->wireEdge counts
                uint32_t lFaceuseCnt  = sShellFaceuses.GetSize() ;
                uint32_t lWireEdgeCnt = sShellWireEdges.GetSize() ;

                // shell point type (in the future may add option for MultiPoints:["BrepMultiPointAPI"])
                TfToken tShellPointType = (lFaceuseCnt == 0 && lWireEdgeCnt == 0) ? UsdBrepSolidTokens->brepPointAPI 
                                                                                  : UsdBrepSolidTokens->none ;

                // add shell data
                rArrays.m_sShellFaceuseCountArray .emplace_back(lFaceuseCnt) ;             // {Faceuse cnt},                     one entry per BrepArray Shell, sized:[TotalShellCnt]
                rArrays.m_sShellWireEdgeCountArray.emplace_back(lWireEdgeCnt) ;            // {WireEdge cnt},                    one entry per BrepArray Shell, sized:[TotalShellCnt]
                rArrays.m_sShellPointTypeArray    .emplace_back(tShellPointType) ;         // {oneof:["BrepPointAPI", "none"],}  one entry per BrepArray Shell, sized:[TotalShellCnt]
          
                // for every shell WireEdge - append wireEdge and wireEdge->Curve3d data
                for(uint32_t kk = 0; kk < lWireEdgeCnt; kk++)
                  {
                    SmEdge     * pWireEdge     = sShellWireEdges[kk] ;
                    SmEdgeData * pWireEdgeData = smu_FindEdgeDataObj_ForSmEdge(pWireEdge, pBrepData_ii->m_vEdges) ;

                    // remember BrepData wireEdge indices - used later to map BrepData EdgeIndices into compacted USD BrepArray EdgeIndices
                    sSmBrep_WireEdge_CurveIndicies.Add(pWireEdgeData->m_lCurve) ;

                    // it's bad data when pWireEdgeData == NULL.  gwc??? need to decide on failure cases.

                    // wireEdge curveType - (in the future add analytics)
                    TfToken tWireEdgeCurveType = (pWireEdge != NULL && pWireEdge->GetCurve() != NULL) ? UsdBrepCurveTokens->brepCurve3dNurbAPI  // all curve types are currently converted to BSplines for USD. In the future USD may support more curve types.
                                                                                                      : UsdBrepSolidTokens->none ;
                    double dWireEdgeParamMin = pWireEdgeData->m_vInterval.GetMin() ;  
                    double dWireEdgeParamMax = pWireEdgeData->m_vInterval.GetMax() ;  

                    // note: WireEdge VertexIndices are not compacted here - that happens in the next block after sSmBrep_VertexShell_VertexIndices is built.
                    GfVec2i sWireEdgeVertices(rArrays.MapLocalToGlobal(pWireEdgeData->m_lStartVertex, sSpans.m_lVertexStartIndex),
                                              rArrays.MapLocalToGlobal(pWireEdgeData->m_lEndVertex,   sSpans.m_lVertexStartIndex) );
                    // add wireEdge data                              
                    rArrays.m_sWireEdgeCurveTypeArray    .emplace_back(tWireEdgeCurveType);         // {oneof:["BrepCurve3dNurbAPI"]},    one entry per BrepArray WireEdge, sized:[TotalWireEdgeCnt]
                                                                                                    //   in the future add options for analytics:["wireEdge_BrepCurveCircleAPI", ...]
                    rArrays.m_sWireEdgeRangeArray        .emplace_back(dWireEdgeParamMin);          // {CrvStartParam},   1st of 2 per BrepArray WireEdge, sized:[2*TotalWireEdgeCnt]
                    rArrays.m_sWireEdgeRangeArray        .emplace_back(dWireEdgeParamMax);          // {CrvEndParam},     2nd of 2 per BrepArray WireEdge, sized:[2*TotalWireEdgeCnt]
                    rArrays.m_sWireEdgeVertexIndicesArray.emplace_back(sWireEdgeVertices);          // {Global StartVtxIndex, Global EndVtxIndex}, one entry per BrepArray WireEdge, sized:[TotalWireEdgeCnt]
                                                    
                    // wireEdge curve
                    SmCurve        * pWireEdgeCurve      = pWireEdge->GetCurve() ;
                    SmBSplineCurve * pBSplineCurve       = NULL ;
                    SmObjDelete      sCleanBSplineCurve; // when pBSplineCurve is a approximation or copy of pWireEdgeCurve, clean up memory when done

                    // for now - force every curve to be a BSplineCurve for USD - later may add support for other curve types
                    if(pWireEdgeCurve->IsKindOf(SmBSplineCurve_TYPE))
                      {
                         pBSplineCurve = SM_REINTERPRET_CAST( SmBSplineCurve*, pWireEdgeCurve) ;
                      } // end WireEdgeCurve is a BSplineCurve branch
                    else // not a BSpline Curve
                      {
                        SmTArray<double> vBreaks;
                        vBreaks.Add(pWireEdgeData->m_vInterval.GetMin());
                        vBreaks.Add(pWireEdgeData->m_vInterval.GetMax());
                        double dAchievedTol = 0.0;
                        pWireEdgeCurve->ApproximateCurve( *(pWireEdgeCurve->GetContext()), // in : memory context for new object
                                                          vBreaks,                         // in : input curve params exactly interpolated by approx curve.
                                                          sApproxTol3d,                    // in : Max dist allowed between approx and original curves.
                                                          dAchievedTol,                    // out: Max dist between approx and original curves seen at sample points.
                                                          pBSplineCurve);                  // out: the new BSpline (always Dim==3)

                        // remember to clean pBSplineCurve memory
                        sCleanBSplineCurve.SetObj(pBSplineCurve); // clean up memory when done
                      } // end WireEdgeCurve is a SmCrvOnSurf branch
               
                    SM_ASSERT_MSG(pBSplineCurve != NULL, _T("SMU_BrepConvert::BrepAppend_SMLibToUsd: WireEdge Curve is NULL - needs debugging"));

                    // copy BSplineCurve data into  UsdBrepArrayData
                    smu_AddSmBSplineCurve3d_ToBrepArrayData(UsdBrepCurveTokens->wireEdge3dNurb, 
                                                            *pBSplineCurve, 
                                                            rArrays) ; 
                  } // end iter every wireEdge 

                // when Shell is a Vertex shell - append ShellVertex position
                if(tShellPointType == UsdBrepSolidTokens->brepPointAPI)
                  {

                    // remember BrepData ShellVertex indices - used later to map BrepData VertexIndices into compacted USD BrepArray VertexIndices
                    sSmBrep_VertexShell_VertexIndices.Add(rShellData.m_lVertex) ; 

                    // shell vertex point
                    SmPoint3d sVertexShellPoint = pShell->GetVertex()->GetPoint() ;
                    GfVec3d   sVertexShell_GfVec3d(sVertexShellPoint.x, sVertexShellPoint.y, sVertexShellPoint.z) ;
                    rArrays.m_sShell_PointPositionArray.emplace_back(sVertexShell_GfVec3d) ;
                  } // end ShellVertex Point
              } // end Shells, WireEdges, WireEdge->Curves, and ShellVertex Points

            // compact Brep_ii's WireEdge->VertexIndices - do this after shells where sSmBrep_VertexShell_VertexIndices is built
            if(   !rArrays.m_sWireEdgeVertexIndicesArray.empty()
               && sSmBrep_VertexShell_VertexIndices.GetSize() > 0)
              {
                for(uint32_t jj=sSpans.m_lWireEdgeStartIndex;jj<rArrays.m_sWireEdgeVertexIndicesArray.size();jj++)
                  {
                    // map global to local indices
                    int32_t lLocalVertex0 = rArrays.MapGlobalToLocal(static_cast<uint32_t>(rArrays.m_sWireEdgeVertexIndicesArray[jj][0]), sSpans.m_lVertexStartIndex) ;
                    int32_t lLocalVertex1 = rArrays.MapGlobalToLocal(static_cast<uint32_t>(rArrays.m_sWireEdgeVertexIndicesArray[jj][1]), sSpans.m_lVertexStartIndex) ;

                    // compact local indices
                    int32_t lCompactVertex0 = smu_CompactLocalIndex(lLocalVertex0, sSmBrep_VertexShell_VertexIndices) ;
                    int32_t lCompactVertex1 = smu_CompactLocalIndex(lLocalVertex1, sSmBrep_VertexShell_VertexIndices) ;
              
                    // store as compacted global indices
                    rArrays.m_sWireEdgeVertexIndicesArray[jj][0] = rArrays.MapLocalToGlobal(lCompactVertex0, sSpans.m_lVertexStartIndex) ;
                    rArrays.m_sWireEdgeVertexIndicesArray[jj][1] = rArrays.MapLocalToGlobal(lCompactVertex1, sSpans.m_lVertexStartIndex) ;
                  } // end iter every wireEdge comapcting the wireEdge->Vertex list
              } // end WireEdge existence check

            // Faceuses
            for (uint32_t jj = 0; jj < lNumFaceuses; ++jj)
              {
                SmFaceuseData & rFaceuseData = pBrepData_ii->m_vFaceuses[jj] ;

                // add faceuse data
                rArrays.m_sFaceuseFaceIndexArray      .emplace_back(rArrays.MapLocalToGlobal(rFaceuseData.m_lFace, sSpans.m_lFaceStartIndex));
                rArrays.m_sFaceuseOrientationTypeArray.emplace_back( (rFaceuseData.m_bOrientation  == TRUE) ? UsdBrepSolidTokens->same        // gwc: rFaceuseData.m_bOrientation uses UNSURE for notSet.
                                                                                                            : UsdBrepSolidTokens->opposite);  //      usd faceuse:orientationType has no notSet value. defaults to opposite
              }  // end Faceuses

            // Faces and Face->Surfaces
            for (uint32_t jj = 0; jj < lNumFaces; ++jj)
              {
                SmFaceData & rFaceData = pBrepData_ii->m_vFaces[jj] ;

                // Attributes
                SmSdfPathAttribute* pAttr = SM_CAST_PTR(SmSdfPathAttribute, rFaceData.m_pFace->FindAttribute(SM_AI_MATERIAL_BINDING));

                // If this is a material binding attribute, add it to list or start a new one
                if (pAttr )
                {
                    bool bFoundMatPath = false;

                    // for every existing FaceMaterial - look for an preexisting entry for this material
                    for (uint32_t kk = 0; kk < rArrays.m_sFaceMaterial_FacePathArray.size(); ++kk)
                    {
                        // look for a match
                        if (rArrays.m_sFaceMaterial_FacePathArray[kk] == pAttr->GetMaterialPaths()->front())
                        {
                            // found match - add FaceIndex to end of existing faceIndices array FaceIndex[kk] = { .., FaceIndex }
                            rArrays.m_sFaceMaterial_FaceIndexArray[kk].emplace_back((int)rArrays.MapLocalToGlobal(jj, sSpans.m_lFaceStartIndex));
                            bFoundMatPath = true;
                        }
                    }
                    // when no existing match was found - make new path and faceIndices entries in the Face materialPaths and MaterialFaceIndices arrays
                    if (!bFoundMatPath)
                    {
                        // add new path to m_sFaceMaterial_FacePathArray
                        rArrays.m_sFaceMaterial_FacePathArray.emplace_back(pAttr->GetMaterialPaths()->front());
                  
                        // add new FaceIndices list {FaceIndex} to m_sFaceMaterial_FaceIndexArray 
                        rArrays.m_sFaceMaterial_FaceIndexArray.emplace_back(VtIntArray{(int)rArrays.MapLocalToGlobal(jj, sSpans.m_lFaceStartIndex)}); 
                    }
                } // end set lMatRelIdx

                // face locals
                SmFace* pFace = rFaceData.m_pFace;
                TfToken tFaceTrimType    = (rFaceData.m_bRectangularTrim == TRUE) ? UsdBrepSolidTokens->rectangular // gwc: rFaceData.m_bRectangularTrim uses UNSURE for notSet.
                                                                                  : UsdBrepSolidTokens->general ;   //      usd face:trimType has no notSet value. defaults to general
                // face surfaceType - detect analytic types first, then fall back to NURB
                TfToken tFaceSurfaceType = UsdBrepSolidTokens->none ;
                if (pFace != NULL && pFace->GetSurface()->IsKindOf(SmSphere_TYPE))
                  {
                    tFaceSurfaceType = UsdBrepSurfaceTokens->brepSurfaceSphereAPI ;
                  }
                else if (pFace != NULL && pFace->GetSurface()->IsKindOf(SmPlane_TYPE))
                  {
                    tFaceSurfaceType = UsdBrepSurfaceTokens->brepSurfacePlaneAPI ;
                  }
                else if (pFace != NULL && pFace->GetSurface()->IsKindOf(SmCylinder_TYPE))
                  {
                    tFaceSurfaceType = UsdBrepSurfaceTokens->brepSurfaceCylinderAPI ;
                  }
                else if (pFace != NULL && pFace->GetSurface()->IsKindOf(SmCone_TYPE))
                  {
                    tFaceSurfaceType = UsdBrepSurfaceTokens->brepSurfaceConeAPI ;
                  }
                else if (pFace != NULL && pFace->GetSurface()->IsKindOf(SmTorus_TYPE))
                  {
                    tFaceSurfaceType = UsdBrepSurfaceTokens->brepSurfaceTorusAPI ;
                  }
                else if (pFace != NULL && pFace->GetSurface()->IsKindOf(SmBSplineSurface_TYPE))
                  {
                    tFaceSurfaceType = UsdBrepSurfaceTokens->brepSurfaceNurbAPI ;
                  }
                else if (pFace != NULL && pFace->GetSurface() != NULL)
                  {
                    tFaceSurfaceType = UsdBrepSurfaceTokens->brepSurfaceNurbAPI ;
                  }

                // SMLib UV trim curves are parameterized on the analytic surface's internal NURBS
                // representation, not on the analytic parameters authored to USD. Retain UV curves
                // only when this face is itself exported as a NURBS surface.
                for (uint32_t kk = 0; kk < rFaceData.m_lNumLoops; ++kk)
                  {
                    const uint32_t lLoopIndex = rFaceData.m_lStartLoop + kk;
                    AERN_MSG(lLoopIndex < sLoopExportsUVCurves.size(),
                             SM_ERR,
                             _T("BrepAppend_SMLibToUsdBrep: face loop index is out of bounds"));
                    sLoopExportsUVCurves[lLoopIndex] =
                        tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfaceNurbAPI;
                  }

                // face range - must match the UV conventions declared by each surface API's schema:
                //   Plane    : (u, v) linear
                //   Sphere   : (u longitude, v latitude)    in radians
                //   Cylinder : (u angle,     v linear)      u in radians
                //   Cone     : (u angle,     v linear)      u in radians
                //   Torus    : (u major ang, v minor ang)   in radians
                //   Nurb     : native Nurb (u, v)
                // For analytic surfaces we source bounds from the surface's STEP UV domain (SmLib degrees for angles)
                // and convert angular parameters to radians. The Face range need not be tight to the trimmed region.
                // For NURB surfaces we keep the Face's native (Nurb) UV domain.
                const auto sBoundedRange = sBoundedFaceRanges.find(pFace);
                SmExtent2d sFaceRangeDomain = sBoundedRange != sBoundedFaceRanges.end()
                                                 ? sBoundedRange->second : rFaceData.m_vUVDomain;
                SmBoolean bUseBoundedFaceRange = bBoundUnboundedFaceRanges && !rFaceData.m_vUVDomain.IsBounded() && sFaceRangeDomain.IsBounded();
                GfVec2d sFaceUVMin(sFaceRangeDomain.GetUMin(), sFaceRangeDomain.GetVMin()) ;
                GfVec2d sFaceUVMax(sFaceRangeDomain.GetUMax(), sFaceRangeDomain.GetVMax()) ;
                if (pFace != NULL && pFace->GetSurface() != NULL && tFaceSurfaceType != UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
                  {
                    SmExtent2d sStepDom = pFace->GetSurface()->GetSTEPUVDomain() ;
                    if (bUseBoundedFaceRange)
                      {
                        // Trim-derived bounds are in native/NURB UV; analytic USD face ranges use STEP UV.
                        SmExtent2d sStepFaceRangeDomain ;
                        if (   pFace->GetSurface()->ConvertDomainFromNURBSToSTEP(sFaceRangeDomain, sStepFaceRangeDomain) == SM_SUCCESS
                            && sStepFaceRangeDomain.IsBounded())
                          {
                            sStepDom = sStepFaceRangeDomain ;
                          }
                      }
                    double dUMin = sStepDom.GetUMin() ;
                    double dVMin = sStepDom.GetVMin() ;
                    double dUMax = sStepDom.GetUMax() ;
                    double dVMax = sStepDom.GetVMax() ;

                    if (   tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfaceSphereAPI
                        || tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
                      {
                        sFaceUVMin = GfVec2d(SM_DEG2RAD(dUMin), SM_DEG2RAD(dVMin)) ;
                        sFaceUVMax = GfVec2d(SM_DEG2RAD(dUMax), SM_DEG2RAD(dVMax)) ;
                      }
                    else if (   tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI
                             || tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
                      {
                        sFaceUVMin = GfVec2d(SM_DEG2RAD(dUMin), dVMin) ;
                        sFaceUVMax = GfVec2d(SM_DEG2RAD(dUMax), dVMax) ;
                      }
                    else // brepSurfacePlaneAPI
                      {
                        // USD plane parameters are distances along unit placement axes.
                        SmVector2d sScale = static_cast<const SmPlane*>(pFace->GetSurface())->GetUVScale();
                        sStepDom.Scale(sScale.x, &sScale.y);
                        sFaceUVMin = GfVec2d(sStepDom.GetUMin(), sStepDom.GetVMin()) ;
                        sFaceUVMax = GfVec2d(sStepDom.GetUMax(), sStepDom.GetVMax()) ;
                      }
                  }

                // add face data                          
                rArrays.m_sFaceLoopCountArray  .emplace_back(rFaceData.m_lNumLoops);   // {LoopCount},                        one entry per BrepArray Face, sized:[TotalFaceCnt]
                rArrays.m_sFaceSurfaceTypeArray.emplace_back(tFaceSurfaceType);        // {oneof:["BrepSurfaceNurbAPI","BrepSurfaceSphereAPI"]}, one per Face, sized:[TotalFaceCnt]
                rArrays.m_sFaceTrimTypeArray   .emplace_back(tFaceTrimType);           // {oneof:["rectangular", "general"]}, one entry per BrepArray Face, sized:[TotalFaceCnt]    
                rArrays.m_sFaceRangeArray      .emplace_back(sFaceUVMin);              // {UVDomain:[UVMin] },              two entries per BrepArray Face, sized:[TotalFaceCnt]
                rArrays.m_sFaceRangeArray      .emplace_back(sFaceUVMax);              // {UVDomain:[UVMax] },              two entries per BrepArray Face, sized:[TotalFaceCnt]

                // face surface
                SmSurface        * pSurface        = pBrepData_ii->m_vSurfaces[rFaceData.m_lSurface] ;

                if (tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
                  {
                    // Extract SmSphere analytic data and pack into sphere arrays
                    SmSphere * pSphere = SM_REINTERPRET_CAST(SmSphere*, pSurface) ;
                    SmAxis2Placement sOrigin ;
                    double dRadius = 0.0 ;
                    pSphere->GetCanonical(sOrigin, dRadius) ;

                    SmPoint3d  sCenter = sOrigin.GetOrigin() ;
                    SmVector3d sZAxis  = sOrigin.GetZAxis() ;
                    SmVector3d sXAxis  = sOrigin.GetXAxis() ;

                    rArrays.m_sFace_SurfaceSphere_CenterArray.emplace_back(GfVec3d(sCenter.x, sCenter.y, sCenter.z)) ;
                    rArrays.m_sFace_SurfaceSphere_AxisArray.emplace_back(GfVec3d(sZAxis.x, sZAxis.y, sZAxis.z)) ;
                    rArrays.m_sFace_SurfaceSphere_RefDirectionArray.emplace_back(GfVec3d(sXAxis.x, sXAxis.y, sXAxis.z)) ;
                    rArrays.m_sFace_SurfaceSphere_RadiusArray.emplace_back(dRadius) ;
                  }
                else if (tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfacePlaneAPI)
                  {
                    SmPlane * pPlane = SM_REINTERPRET_CAST(SmPlane*, pSurface) ;
                    SmAxis2Placement sPlacement ;
                    pPlane->GetCanonical(sPlacement) ;

                    SmPoint3d  sOriginPt = sPlacement.GetOrigin() ;
                    SmVector3d sZAxis    = sPlacement.GetZAxis() ;
                    SmVector3d sXAxis    = sPlacement.GetXAxis() ;

                    rArrays.m_sFace_SurfacePlane_OriginArray.emplace_back(GfVec3d(sOriginPt.x, sOriginPt.y, sOriginPt.z)) ;
                    rArrays.m_sFace_SurfacePlane_AxisArray.emplace_back(GfVec3d(sZAxis.x, sZAxis.y, sZAxis.z)) ;
                    rArrays.m_sFace_SurfacePlane_RefDirectionArray.emplace_back(GfVec3d(sXAxis.x, sXAxis.y, sXAxis.z)) ;
                  }
                else if (tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI)
                  {
                    SmCylinder * pCylinder = SM_REINTERPRET_CAST(SmCylinder*, pSurface) ;
                    SmAxis2Placement sPlacement ;
                    double dRadius = 0.0 ;
                    pCylinder->GetCanonical(sPlacement, dRadius) ;

                    SmPoint3d  sOriginPt = sPlacement.GetOrigin() ;
                    SmVector3d sZAxis    = sPlacement.GetZAxis() ;
                    SmVector3d sXAxis    = sPlacement.GetXAxis() ;

                    rArrays.m_sFace_SurfaceCylinder_OriginArray.emplace_back(GfVec3d(sOriginPt.x, sOriginPt.y, sOriginPt.z)) ;
                    rArrays.m_sFace_SurfaceCylinder_AxisArray.emplace_back(GfVec3d(sZAxis.x, sZAxis.y, sZAxis.z)) ;
                    rArrays.m_sFace_SurfaceCylinder_RefDirectionArray.emplace_back(GfVec3d(sXAxis.x, sXAxis.y, sXAxis.z)) ;
                    rArrays.m_sFace_SurfaceCylinder_RadiusArray.emplace_back(dRadius) ;
                  }
                else if (tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
                  {
                    SmCone * pCone = SM_REINTERPRET_CAST(SmCone*, pSurface) ;
                    SmAxis2Placement sPlacement ;
                    double dRadius = 0.0 ;
                    double dSemiAngleDeg = 0.0 ;
                    pCone->GetCanonical(sPlacement, dRadius, dSemiAngleDeg) ;

                    SmPoint3d  sOriginPt = sPlacement.GetOrigin() ;
                    SmVector3d sZAxis    = sPlacement.GetZAxis() ;
                    SmVector3d sXAxis    = sPlacement.GetXAxis() ;

                    rArrays.m_sFace_SurfaceCone_OriginArray.emplace_back(GfVec3d(sOriginPt.x, sOriginPt.y, sOriginPt.z)) ;
                    rArrays.m_sFace_SurfaceCone_AxisArray.emplace_back(GfVec3d(sZAxis.x, sZAxis.y, sZAxis.z)) ;
                    rArrays.m_sFace_SurfaceCone_RefDirectionArray.emplace_back(GfVec3d(sXAxis.x, sXAxis.y, sXAxis.z)) ;
                    rArrays.m_sFace_SurfaceCone_RadiusArray.emplace_back(dRadius) ;
                    rArrays.m_sFace_SurfaceCone_SemiAngleArray.emplace_back(SM_DEG2RAD(dSemiAngleDeg)) ;
                  }
                else if (tFaceSurfaceType == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
                  {
                    SmTorus * pTorus = SM_REINTERPRET_CAST(SmTorus*, pSurface) ;
                    SmAxis2Placement sPlacement ;
                    double dMajorRadius = 0.0 ;
                    double dMinorRadius = 0.0 ;
                    pTorus->GetCanonical(sPlacement, dMajorRadius, dMinorRadius) ;

                    SmPoint3d  sOriginPt = sPlacement.GetOrigin() ;
                    SmVector3d sZAxis    = sPlacement.GetZAxis() ;
                    SmVector3d sXAxis    = sPlacement.GetXAxis() ;

                    rArrays.m_sFace_SurfaceTorus_OriginArray.emplace_back(GfVec3d(sOriginPt.x, sOriginPt.y, sOriginPt.z)) ;
                    rArrays.m_sFace_SurfaceTorus_AxisArray.emplace_back(GfVec3d(sZAxis.x, sZAxis.y, sZAxis.z)) ;
                    rArrays.m_sFace_SurfaceTorus_RefDirectionArray.emplace_back(GfVec3d(sXAxis.x, sXAxis.y, sXAxis.z)) ;
                    rArrays.m_sFace_SurfaceTorus_MajorRadiusArray.emplace_back(dMajorRadius) ;
                    rArrays.m_sFace_SurfaceTorus_MinorRadiusArray.emplace_back(dMinorRadius) ;
                  }
                else
                  {
                    // NURB or approximation path
                    SmBSplineSurface * pBSplineSurface = NULL;
                    SmObjDelete        sCleanBSplineSurface ; 

                    if(pFace != NULL && pFace->GetSurface()->IsKindOf(SmBSplineSurface_TYPE))
                      {
                         pBSplineSurface = SM_REINTERPRET_CAST( SmBSplineSurface*, pSurface) ;
                      } // end BSplineSurfaceTYPE branch
                    else if(pFace != NULL)
                      {
                        double dAchievedTol = 0.0 ;

                        pSurface->ApproximateSurface( *pSurface->GetContext(), // in : context for new object construction
                                                      sApproxTol3d,            // in : max allowed approximation distance                                        
                                                      dAchievedTol,            // out: Max tolerance achieved.                                                   
                                                      5,                       // in : How many times to split spans before giving up                            
                                                      pBSplineSurface,         // out: approx surf with the smallest number of                                   
                                                                               //    :  subdivisions possible within tol of exact surf.                          
                                                      TRUE);                   // in : TRUE = if this Surface is a BSpline, copy it FALSE= approximate the surface 
                        sCleanBSplineSurface.SetObj(pBSplineSurface) ;

                      } // end pSurface is not a BSplineSurface type branch

                    if (pBSplineSurface == NULL)
                      {
                        TCHAR sMessage[SM_TBLOCK_SIZE];
                        SM_SPRINTF(sMessage, _T("BrepAppend_SMLibToUsd: unable to convert face surface to BSpline for face %u"), ii);
                        ERR_MSG(sMessage);
                        continue;
                      }

                    // add face->surface data to sArray
                    smu_AddSmBSplineSurface_ToBrepArrayData(*pBSplineSurface, rArrays) ;
                  }

              } // end Faces

            // Loops
            for (uint32_t jj = 0; jj < lNumLoops; ++jj)
              {
                SmLoopData & rLoopData = pBrepData_ii->m_vLoops[jj] ;

                // locals
                uint32_t     lLoopType              = rLoopData.m_lLoopType ; 
                uint32_t     lEdgeuseCount          = lLoopType == 0 ? rLoopData.m_lNumEU : 0 ; 
                uint32_t     lGlobalLoopVertexIndex = (lLoopType == 0) ? USDBREP_NO_OBJECT_INDEX    // since sSmBrep_VertexShell_VertexIndices is built, store a global, compacted vertex index
                                                                       : rArrays.MapLocalToGlobal(smu_CompactLocalIndex(rLoopData.m_lVertex, sSmBrep_VertexShell_VertexIndices), 
                                                                                          sSpans.m_lVertexStartIndex);
          
                // check state: Loop must be either edgeuse_TYPE (lLoopType==0) or vertex_TYPE (lLoopType==1) - else abort
                AERN_MSG((lEdgeuseCount != 0) || (lGlobalLoopVertexIndex != USDBREP_NO_OBJECT_INDEX), SM_ERR, 
                       _T("SMU_BrepConvert::BrepAppend_SMLibToUsd: Incorrect LoopType"));
  
                // add Loop data                             
                rArrays.m_sLoopEdgeuseCountArray.emplace_back(lEdgeuseCount) ;           // {EdgeuseCount},       one entry per BrepArray Loop, sized:[TotalLoopCnt]
                rArrays.m_sLoopVertexIndexArray .emplace_back(lGlobalLoopVertexIndex) ;  // {Global VertexIndex}, one entry per BrepArray Loop, sized:[TotalLoopCnt]
                                                                                         //    where VertexIndex = Loop_ii's vertex index when loop:edgeuseCount[ii] == 0, else ignored.
              }

            // Edgeuses
            for (int32_t jj = 0; jj < (int32_t)lNumEdgeuses; ++jj)
              {
                SmEUData & rEUData = pBrepData_ii->m_vEdgeuses[jj] ;

                // ignore edgeuses used for wireEdges, ShellVertices, and VertexLoops - only add SmBrepData Edgeuses connected edges 
                if(rEUData.m_lEUType != 0) // 0 == Edgeuse connects an Edge to a Face
                  { 
                    // remember which LoopVertex_TYPE and wireEdge_TYPE edgeuse indices - to compact SMLib EUIndex values into compacted USD EU_indices
                    sSmBrep_LoopAndWireEdgeType_EUIndicies.Add(jj) ; 
                    continue ; 
                  } // end EUType == loopVertex_TYPE or wireEdge_TYPE check

                // locals - since SmBrep_WireEdgeIndices is built, store a global, compacted Edge index
                uint32_t lEU_Edge_GlobalIndex = rArrays.MapLocalToGlobal(smu_CompactLocalIndex(rEUData.m_lEdge, sSmBrep_WireEdge_CurveIndicies), 
                                                                 sSpans.m_lEdgeStartIndex) ;
                TfToken  tEU_OrientType       = (rEUData.m_bOrientation == TRUE) ? UsdBrepSolidTokens->same       // gwc: rEUData.m_bOrientation uses UNSURE for notSet.
                                                                                 : UsdBrepSolidTokens->opposite ; //      usd edgeuse:orientationType has no notSet value. defaults to opposite

                // radial edgeuse list: 
                //  SMLib: each SmEUData obj represents a mated top/bot pair of SmEdgeuse objects.
                //         The radial edgeuse order is stored in the SmEUData object as a pair of Next Edgeuse indices
                //           SmEUData::m_lNextEU     = the SmEUData->topEdgeuse's next edgeuse index in the radial edgeuse list.  
                //           SmEUData::m_lMateNextEU = the SmEUData->botEdgeuse's next edgeuse index in the radial edgeuse list.
                //          one of the top/bot SmEdgeuse pair will point to its mate and the other will point to a bottom or top edgeuse member
                //          of another SmEUData object.
                //            a nextIndex value:[positive ii] = the next radialEdgeuseList entry is the topEdgeuse member of the (ii)th    SmEUData object
                //            a nextIndex value:[negative ii] = the next radialEdgeuseList entry is the botEdgeuse member of the (-ii-1)th SmEUData object
                //  USD: each UsdEdgeuse data set also represents a mated top/bot pair of SmEdgeuse objects.
                //         edgeuse:nextRadialEUIndex   = the next UsdEdgeuse data object on the radialEdgeuselist (could be either the next UsdEdgeuse's top or bot edgeuse) 
                //         edgeuse:thisRadialEntryType = stores the radial list order for the top/bot edgeuse pair of this UsdEdgeuse data set on the radialEdgeList
                //              value:[topEntry   ] = this UsdEdgeuse->edgeuse order:{ topEdgeuse, botEdgeuse }
                //              value:[bottomEntry] = this UsdEdgeuse->edgeuse order:{ botEdgeuse, topEdgeuse }
                int32_t lEU_NextEU_GlobalIndex     = MapLocalInt_ToGlobal(rEUData.m_lNextEU,     sSpans.m_lEdgeuseStartIndex) ;  // note: m_lNextEU can be neg, don't use rArrays.MapLocalToGlobal
                int32_t lEU_MateNextEU_GlobalIndex = MapLocalInt_ToGlobal(rEUData.m_lMateNextEU, sSpans.m_lEdgeuseStartIndex) ;  // note: m_lMateNextEU can be neg, don't use rArrays.MapLocalToGlobal
                int32_t lEU_TopEUNext_GlobalIndex = (rEUData.m_lNextEU     >= 0) ? lEU_NextEU_GlobalIndex     : (-lEU_NextEU_GlobalIndex    -1) ;
                int32_t lEU_BotEUNext_GlobalIndex = (rEUData.m_lMateNextEU >= 0) ? lEU_MateNextEU_GlobalIndex : (-lEU_MateNextEU_GlobalIndex-1) ; ;

                // when SMLib's thisEdgeuse->topEdgeuseNext points to its own mated botEdgeuse, the RadialEntryType = topEntry.
                //      SMLib's thisEdgeuse->botEdgeuseNext points to its own mated topEdgeuse, the RadialEntryType = bottomEntry.
                TfToken  tEU_RadialEntryType =   (lEU_TopEUNext_GlobalIndex == (int32_t)rArrays.MapLocalToGlobal(jj, sSpans.m_lEdgeuseStartIndex)) 
                                               ? UsdBrepSolidTokens->topEntry 
                                               : UsdBrepSolidTokens->bottomEntry ;

                // when USD entryType == topEntry,    the NextRadialEUIndex = SMLib's lSMLibBotEUNextIndex
                //      USD entryType == bottomEntry, the NextRadialEUIndex = SMLib's lSMLibTopEUNextIndex
                int32_t lEU_NextRadialEU_GlobalIndex =  (lEU_TopEUNext_GlobalIndex == (int32_t)rArrays.MapLocalToGlobal(jj, sSpans.m_lEdgeuseStartIndex)) 
                                                       ? lEU_BotEUNext_GlobalIndex 
                                                       : lEU_TopEUNext_GlobalIndex ;
                                                                                                    
                // add Edgeuse data
                rArrays.m_sEdgeuseEdgeIndexArray      .emplace_back(lEU_Edge_GlobalIndex); // {compacted global EdgeIndex}, one entry per BrepArray Edge_Edgeuse, sized:[TotalEdge_EdgeuseCnt]
                rArrays.m_sEdgeuseOrientationTypeArray.emplace_back(tEU_OrientType);       // {oneof:["same", "opposite"],  one entry per BrepArray Edge_Edgeuse, sized:[TotalEdge_EdgeuseCnt] 
                                                                                           //    where same   = edgeuse's UVTrimCurve runs in the same direction as the edge's curve and
                                                                                           //                   represents the owning edge's binormal side connecting to a face.
                                                                                           //    and opposite = edgeuse's UVTrimCurve runs in the opposite direction and
                                                                                           //                   represents the edge's other side connecting to a face.
                rArrays.m_sEdgeuseNextRadialEUIndexArray.emplace_back(lEU_NextRadialEU_GlobalIndex); // {global next radial EdgeuseIndex},   one entry per BrepArray Edgeuse, sized:[TotalEdgeuseCnt]
                                                                                                     //    where  nextRadialEdgeuse = index of the nextRadialEdgeuse in a right-hand-rule traversal around the edgeuse's edge.
                rArrays.m_sEdgeuseThisRadialEntryTypeArray.emplace_back(tEU_RadialEntryType);        // {oneof:["topEntry", "bottomEntry"]}, one entry per BrepArray Edgeuse, sized:[TotalEdgeuseCnt]
                                                                                                     //    where topEntry    = a {topFaceSideEntry botFaceSideExit}, traversal of this edgeuse's face during a right hand traversal around the edgeuse's edge.
                                                                                                     //    where bottomEntry = a {botFaceSideEntry topFaceSideExit}, traversal of this edgeuse's face during a right hand traversal around the edgeuse's edge.
                                                                      
                // Optional UV Trimcurve. Analytic-face curves are represented as missing because
                // their coordinates use SMLib's internal NURBS parameterization, not USD analytic UV.
                if (bExportUVCurves)
                {
                    AERN_MSG(rEUData.m_lLoop < sLoopExportsUVCurves.size(),
                             SM_ERR,
                             _T("BrepAppend_SMLibToUsdBrep: edgeuse loop index is out of bounds"));
                    const SmBoolean bExportThisUVCurve = sLoopExportsUVCurves[rEUData.m_lLoop];
                    SmCurve * pUVCurve = bExportThisUVCurve && rEUData.m_lUVCurve != SM_NO_OBJECT
                                               ? pBrepData_ii->m_vUVCurves[rEUData.m_lUVCurve]
                                               : NULL ;
                    SmBSplineCurve * pBSplineCurve2d =  SM_CAST_PTR(SmBSplineCurve, pUVCurve) ;
                    // SmBSplineCurve * pBSplineCurve2d =   rEUData.m_pEdgeuse1 ? rEUData.m_pEdgeuse1->GetUVTrimCurve()
                    //                                    : rEUData.m_pEdgeuse2 ? rEUData.m_pEdgeuse2->GetUVTrimCurve()
                    //                                    : NULL ; 
                    const SmStatus sUvCurveStatus =
                        smu_AddSmBSplineCurve2d_ToBrepArrayData(pBSplineCurve2d,         // in : BSplineCurve to Append, NULL = add empty BrepCurve with VertexCount == 0
                                                               rEUData.m_bOrientation, // in : edgeuse orientation relative to the edge
                                                               rArrays) ;               // i/o: UsdBrepArrayData being appended.
                    if (sUvCurveStatus != SM_SUCCESS)
                      {
                        // UV trims are optional. Preserve the edgeuse/UV-curve one-to-one record
                        // alignment and let the reader reconstruct this trim from 3D topology.
                        rArrays.m_sEdgeuse_CurveNurb_VertexCountArray.emplace_back(0) ;
                        rArrays.m_sEdgeuse_CurveNurb_OrderArray.emplace_back(0) ;
                      }
                 }
              } // end Edgeuses

            // compact Brep_ii's Edgeuse->NextRadialEUIndices - do this after edgeuses where sSmBrep_LoopAndWireEdgeType_EUIndicies is built
            if(   !rArrays.m_sEdgeuseNextRadialEUIndexArray.empty()
               && sSmBrep_LoopAndWireEdgeType_EUIndicies.GetSize() > 0)
              {
                for(uint32_t jj=sSpans.m_lEdgeuseStartIndex;jj<rArrays.m_sEdgeuseNextRadialEUIndexArray.size();jj++)
                  {
                    // map global to local indices
                    uint32_t lLocalEU = rArrays.MapGlobalToLocal(rArrays.m_sEdgeuseNextRadialEUIndexArray[jj], sSpans.m_lEdgeuseStartIndex) ;

                    // compact local indices
                    uint32_t lCompactEU = smu_CompactLocalIndex(lLocalEU, sSmBrep_LoopAndWireEdgeType_EUIndicies) ;
              
                    // store as compacted global indices
                    rArrays.m_sEdgeuseNextRadialEUIndexArray[jj] = rArrays.MapLocalToGlobal(lCompactEU, sSpans.m_lEdgeuseStartIndex) ;
                  } // end iter every wireEdge comapcting the wireEdge->Vertex list
              } // end WireEdge existence check

            // Edges - get divided between USd FaceEdges and WireEdges - only do FaceEdges here, WireEdges are done in the Shell block
            for (uint32_t jj = 0; jj < lNumEdges; ++jj)
              {
                SmEdgeData & rEdgeData = pBrepData_ii->m_vEdges[jj] ;
                SmEdge     * pEdge     = rEdgeData.m_pEdge ;
          
                // skip WireEdges
                if(sSmBrep_WireEdge_CurveIndicies.IsIn(jj))
                  {  continue ; }

                // edge curveType - (in the future add analytics)
                TfToken tEdgeCurveType = (pEdge != NULL && pEdge->GetCurve() != NULL) ? UsdBrepCurveTokens->brepCurve3dNurbAPI  // all curve types are currently converted to BSplines for USD. In the future USD may support more curve types.
                                                                                      : UsdBrepSolidTokens->none ;
                double dEdgeParamMin = rEdgeData.m_vInterval.GetMin() ;  
                double dEdgeParamMax = rEdgeData.m_vInterval.GetMax() ;  
                // since sSmBrep_VertexShell_VertexIndices is built, store global, compacted Edge->Vertex indices
                GfVec2i sEdgeVertices(rArrays.MapLocalToGlobal(smu_CompactLocalIndex(rEdgeData.m_lStartVertex, sSmBrep_VertexShell_VertexIndices), 
                                                       sSpans.m_lVertexStartIndex), 
                                      rArrays.MapLocalToGlobal(smu_CompactLocalIndex(rEdgeData.m_lEndVertex, sSmBrep_VertexShell_VertexIndices), 
                                                       sSpans.m_lVertexStartIndex) );
           
                // add edge data                              
                rArrays.m_sEdgeCurveTypeArray    .emplace_back(tEdgeCurveType);            // {oneof:["BrepCurve3dNurbAPI"]},    one entry per BrepArray Edge, sized:[TotalEdgeCnt]
                                                                                           //         in the future add options for analytics:["edge_BrepCurveCircleAPI", ...]
                rArrays.m_sEdgeRangeArray        .emplace_back(dEdgeParamMin);             // {CrvStartParam},   1st of 2 per BrepArray Edge, sized:[2*TotalEdgeCnt]
                rArrays.m_sEdgeRangeArray        .emplace_back(dEdgeParamMax);             // {CrvEndParam},     2nd of 2 per BrepArray Edge, sized:[2*TotalEdgeCnt]
                rArrays.m_sEdgeVertexIndicesArray.emplace_back(sEdgeVertices);             // {Global StartVtxIndex, Global EndVtxIndex}, one per BrepArray Edge, sized:[TotalEdgeCnt]
          
                // edge curve
                SmCurve        * pEdgeCurve    = pEdge->GetCurve() ;
                SmBSplineCurve * pBSplineCurve = NULL ;
                SmObjDelete      sCleanBSplineCurve; // when pBSplineCurve is a approximation or copy of pEdgeCurve, clean up memory when done
          
                // for now - force every curve to be a BSplineCurve for USD - later may add support for other curve types
                if(pEdgeCurve->IsKindOf(SmBSplineCurve_TYPE))
                  {
                     pBSplineCurve = SM_REINTERPRET_CAST( SmBSplineCurve*, pEdgeCurve) ;
                  } // end EdgeCurve is a BSplineCurve branch
          
                else
                  {
                    SmTArray<double> vBreaks;
                    vBreaks.Add(rEdgeData.m_vInterval.GetMin());
                    vBreaks.Add(rEdgeData.m_vInterval.GetMax());
                    double dAchievedTol = 0.0;
                    pEdgeCurve->ApproximateCurve( *(pEdgeCurve->GetContext()), // in : memory context for new object
                                                 vBreaks,                 // in : input curve params exactly interpolated by approx curve.
                                                 sApproxTol3d,            // in : Max dist allowed between approx and original curves.
                                                 dAchievedTol,            // out: Max dist between approx and original curves seen at sample points.
                                                 pBSplineCurve);          // out: the new BSpline (always Dim==3)
          
                    // remember to clean pBSplineCurve memory
                    sCleanBSplineCurve.SetObj(pBSplineCurve); // clean up memory when done
                  } // end EdgeCurve is a SmCrvOnSurf branch

                SM_ASSERT_MSG(pBSplineCurve != NULL, _T("SMU_BrepConvert::BrepAppend_SMLibToUsd: Edge Curve is NULL - needs debugging"));

                // copy BSplineCurve data into UsdBrepArrayData
                smu_AddSmBSplineCurve3d_ToBrepArrayData(UsdBrepCurveTokens->edge3dNurb, 
                                                        *pBSplineCurve, 
                                                        rArrays) ; 
              } // end iter every Edge 

            // Vertices - get divided between USD Vertex and ShellVertex - only do Vertices here - ShellVertices done in the Shell block
            for (uint32_t jj = 0; jj < lNumVertices; ++jj)
              {
                SmVertexData & rVertexData = pBrepData_ii->m_vVertices[jj] ;

                // skip Shell vertices
                if(sSmBrep_VertexShell_VertexIndices.IsIn(jj))
                  {  continue ; }

                // locals
                const SmPoint3d & crPt             = rVertexData.m_vPoint;
                TfToken           tVertexPointType = UsdBrepSolidTokens->brepPointAPI ;  // (in the future may add option for MultiPoints:["BrepMultiPointAPI"])

                // add vertex data                          
                rArrays.m_sVertexPointTypeArray.emplace_back(UsdBrepSolidTokens->brepPointAPI); // {oneof:["BrepPointAPI", "none"]}, one entry per BrepArray Vertex, sized:[TotalVertexCnt]
                                                                                            //   in the future may add option for MultiPoints:["vertex_BrepMultiPointAPI"]

                // add vertex->Point data
                if(tVertexPointType == UsdBrepSolidTokens->brepPointAPI)
                  {
                    GfVec3d sVertexPoint(crPt.x, crPt.y, crPt.z) ;
                    rArrays.m_sVertex_PointPositionArray.emplace_back(sVertexPoint) ;
                  } // end Vertex->Point
              } // end Vertices
  
          } // end scope: move BrepData_ii data into the rArrays arrays
      } // end iter every SmBrep_ii being added to UsdBrepArrayGprim and UsdBrepArray

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
       {
         SmContext    sContext ; 
         SmBrep     * pDbgBrep_0     = rSmBreps[0] ;  // input
         SmBrepData * pDbgBrepData_0 = new (sContext) SmBrepData(FALSE, SM_DS_USD);
         SmObjDelete sClean(pDbgBrepData_0) ; 
         SmTArray<SmAttribute*> sAttributes ;

         SM_DUMP_AND_ASSERT_VALID(pDbgBrep_0) ;
    
         pDbgBrepData_0->FromBrep(*pDbgBrep_0, sAttributes) ; // input-BrepData

         if (usdBrep_CreateOutputFiles())              
          {
             std::string sPathString = "OutputFiles/AddSmBreps_ToUsdBrepArray() InputBrep0_To_BrepData.smb";

             // TRUE = open file and rewrite contents, FALSE= open file and append to end
             pDbgBrepData_0->WriteToFile(sPathString, SM_ASCII, TRUE);

             usdBrep_WriteString("\n\n  FILE_WRITE: " + sPathString + "\n");
          }

         // rArrays = data built from input Breps
         smu_sprintf(sBuff, _T("\n\n  AddSmBreps_ToUsdBrepArray(): Begin dump of Output_BrepArrayData with %d Breps"), rSmBreps.size()) ;
         smu_WriteBuffer(sBuff) ;
         Dump_BrepArrayData(rArrays, TRUE); // TRUE  = also dump array data values, FALSE = dump only Topology obj counts
         Dump_SMLibCounts(rArrays);
         smu_sprintf(sBuff, _T("\n\n  AddSmBreps_ToUsdBrepArray(): End dump of Output_BrepArrayData with %d Breps"), rSmBreps.size()) ;
         smu_WriteBuffer(sBuff) ;
       }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SMU_BrepConvert::AppendSmBreps_ToBrepArrayData

static bool IsFatalBrepImportStatus(SmStatus status)
{
    return status == SM_ERR_OUT_OF_MEMORY ||
           status == SM_ERR_FATAL ||
           status == SM_ERR_ASSERT_FAILURE ||
           status == SM_ERR_LICENSE_EXPIRED;
}

static void DeleteBrepImportResults(std::vector<BrepImportResult>& rResults)
{
    for (BrepImportResult& rResult : rResults)
      {
        delete rResult.pBrep;
        rResult.pBrep = NULL;
      }
    rResults.clear();
}

/*******************************************************************/ /**
PURPOSE: Attempt every locatable Brep member in an OmniSolidBrep prim.

NOTES: The caller owns every nonNULL pBrep until it transfers or deletes the
       pointer. Member conversion failures are results, not call failures.
       A malformed container that prevents member enumeration is a call
       failure. A malformed member boundary stops the rest of that prim because
       subsequent packed offsets cannot be trusted.
***********************************************************************/
SmStatus BrepMove_UsdToSMLibWithResults
 (const SmContext                    & crSmContext,
  const pxr::UsdGeomGprim           & crUsdBrepArray,
  std::vector<BrepImportResult>     & rResults,
  SmBoolean                           bHealerIsEnabled,
  SmBoolean                           bStopAfterFailure)
{
    TCHAR sPathBuff[SMU_TBLOCK_SIZE];
    StdStringToTCHAR(crUsdBrepArray.GetPrim().GetPath().GetAsString(), sPathBuff, SMU_TBLOCK_SIZE);

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    TCHAR     sBuff[SMU_TBLOCK_SIZE];
#endif // SM_DEBUG_CODE

    UsdBrepArrayData sArrays;
    UsdBrepArraySpans sSpans;
    std::vector<BrepImportResult> sNewResults;

    const uint32_t lBrepCount = GetUsdBrepArray_BrepCount(crUsdBrepArray.GetPrim());
    if (lBrepCount == 0)
      {
        TCHAR sMessage[SM_TBLOCK_SIZE];
        SM_SPRINTF(sMessage,
                   _T("BrepMove_UsdToSMLibWithResults: prim [%.800s] contains no packed Breps"),
                   sPathBuff);
        ERR_MSG(sMessage);
        return SM_ERR_INVALID_INPUT;
      }

    if (!BrepReadFromUsdStage(crUsdBrepArray.GetPrim(), sArrays))
      {
        TCHAR sMessage[SM_TBLOCK_SIZE];
        SM_SPRINTF(sMessage,
                   _T("BrepMove_UsdToSMLibWithResults: cannot read BrepArray data for prim [%.800s]"),
                   sPathBuff);
        ERR_MSG(sMessage);
        return SM_ERR_INVALID_INPUT;
      }

#ifdef SM_DEBUG_CODE
    if (bDebugMe)
      {
        smu_sprintf(sBuff,
                    _T("\n  Begin BrepMove_UsdToSMLibWithResults() for SdfPath:[%s], TotalBrepCnt:[%4lu]"),
                    sPathBuff,
                    lBrepCount);
        smu_WriteBuffer(sBuff);
        Dump_BrepArrayData(sArrays, TRUE);
        Dump_SMLibCounts(sArrays);
      }
#endif // SM_DEBUG_CODE

    sNewResults.reserve(sArrays.TotalBrepCount());
    for (uint32_t iBrep = 0; iBrep < sArrays.TotalBrepCount(); ++iBrep)
      {
        // In continuing mode, validate packed-member boundaries before
        // construction. Conversion advances some geometry start indices while
        // consuming the member, so preserve the historical behavior of
        // recomputing spans for the next packed index rather than incrementing
        // this mutated span set. Strict mode lets the one-member converter do
        // the same validation once, then stops on its first failure.
        if (!bStopAfterFailure &&
            !sSpans.SetStartsAndCountsForBrepIndex(sArrays, iBrep, FALSE, TRUE))
          {
            BrepImportResult sResult;
            sResult.iPackedBrepIndex = iBrep;
            sResult.status = SM_ERR_INVALID_INPUT;
            sResult.bRemainingMembersSkipped = iBrep + 1 < sArrays.TotalBrepCount();
            sNewResults.push_back(sResult);

            TCHAR sMessage[SM_TBLOCK_SIZE];
            if (sResult.bRemainingMembersSkipped)
              {
                SM_SPRINTF(
                    sMessage,
                    _T("BrepMove_UsdToSMLibWithResults: malformed packed-member boundary for prim [%.800s], packed Brep index %u; remaining members skipped"),
                    sPathBuff,
                    static_cast<unsigned int>(iBrep));
              }
            else
              {
                SM_SPRINTF(
                    sMessage,
                    _T("BrepMove_UsdToSMLibWithResults: malformed packed-member boundary for prim [%.800s], packed Brep index %u"),
                    sPathBuff,
                    static_cast<unsigned int>(iBrep));
              }
            ERR_MSG(sMessage);
            break;
          }

        SmBrep* pBrep = NULL;
        SmStatus stat = BrepMove_OneUsdBrepToSMLib(
            crSmContext, sArrays, sSpans, iBrep, pBrep, bHealerIsEnabled);
        if (stat == SM_SUCCESS && pBrep == NULL)
          {
            stat = SM_ERR;
          }

        if (IsFatalBrepImportStatus(stat))
          {
            delete pBrep;
            DeleteBrepImportResults(sNewResults);
            return stat;
          }

        BrepImportResult sResult;
        sResult.iPackedBrepIndex = iBrep;
        sResult.status = stat;

        if (stat == SM_SUCCESS)
          {
            sResult.pBrep = pBrep;
          }
        else
          {
            if (pBrep != NULL)
              {
                ERR_MSG(_T("BrepMove_UsdToSMLibWithResults: failed member conversion returned a nonNULL Brep"));
              }
            delete pBrep;

            TCHAR sMessage[SMU_TBLOCK_SIZE];
            SM_SPRINTF(
                sMessage,
                _T("BrepMove_UsdToSMLibWithResults: conversion failed for prim [%.800s], packed Brep index %u"),
                sPathBuff,
                static_cast<unsigned int>(iBrep));
            ERR_MSG(sMessage);
          }

        sNewResults.push_back(sResult);
        if (stat != SM_SUCCESS && bStopAfterFailure)
          {
            break;
          }
      }

#ifdef SM_DEBUG_CODE
    if (bDebugMe)
      {
        smu_sprintf(sBuff,
                    _T("\n  End BrepMove_UsdToSMLibWithResults() for SdfPath:[%s], TotalBrepCnt:[%4lu]"),
                    sPathBuff,
                    lBrepCount);
        smu_WriteBuffer(sBuff);
      }
#endif // SM_DEBUG_CODE

    rResults.insert(rResults.end(), sNewResults.begin(), sNewResults.end());
    return SM_SUCCESS;
}

/*******************************************************************/ /**
PURPOSE: Convert an OmniSolidBrep prim into a list of SmBreps

NOTES: Creates all Breps in same SmContext. Conversion is all-or-nothing:
       when any packed Brep fails, all Breps created by this call are deleted
       and pre-existing entries in rSmBreps are preserved.

RETURNS: SM_SUCCESS only when every packed Brep converts successfully;
         otherwise, the first conversion error.
***********************************************************************/
SmStatus BrepMove_UsdToSMLib
 (const SmContext         & crSmContext,
  const pxr::UsdGeomGprim & crUsdBrepArray,
  std::vector<SmBrep*>    & rSmBreps,
  SmBoolean                 bHealerIsEnabled)
{
    std::vector<BrepImportResult> sResults;
    SmStatus stat = BrepMove_UsdToSMLibWithResults(
        crSmContext, crUsdBrepArray, sResults, bHealerIsEnabled, TRUE);
    if (stat != SM_SUCCESS)
      {
        DeleteBrepImportResults(sResults);
        return stat;
      }

    SmStatus firstFailure = SM_SUCCESS;
    std::vector<SmBrep*> sConvertedBreps;
    sConvertedBreps.reserve(sResults.size());
    for (BrepImportResult& rResult : sResults)
      {
        if (rResult.status != SM_SUCCESS || rResult.pBrep == NULL)
          {
            if (firstFailure == SM_SUCCESS)
              {
                firstFailure = (rResult.status != SM_SUCCESS) ? rResult.status : SM_ERR;
              }
            delete rResult.pBrep;
            rResult.pBrep = NULL;
          }
        else
          {
            sConvertedBreps.push_back(rResult.pBrep);
            rResult.pBrep = NULL;
          }
      }

    if (firstFailure != SM_SUCCESS)
      {
        DeleteSmBreps(sConvertedBreps);
        return firstFailure;
      }

    if (sConvertedBreps.empty())
      {
        return SM_ERR;
      }

    rSmBreps.insert(rSmBreps.end(), sConvertedBreps.begin(), sConvertedBreps.end());
    return SM_SUCCESS;
}

/*******************************************************************/ /**
PURPOSE: Move one AppendedBrepArrayData item into an SmBrep object

NOTES:
 1. pipeline: =     UsdBrepArrayData[iBrepIndex]  
                =>  SmBrepData_ii  
                =>  prSmBrep_ii

 2. SmBrepData vs. UsdBrepArrayData organization:
     - SmBrepData       = lists of topology object data for one Brep
     - UsdBrepArrayData = lists of associated arrays of topology object data for an array of Breps
        +------------------++---------------------------------------------------------+
        |   BrepData       ||            UsdBrepArrayData                             |
        |   1 list of      ||                                                         |
        +------------------++---------------------------------------------------------+
        |  Region Objects  || 1 set of associated Region arrays                       |
        |  Shell Objects   || 1 set of associated Shell arrays                        |
        |  Faceuse Objects || 1 set of associated Faceuse arrays                      |
        |  Face Objects    || 1 set of associated Face arrays                         |
        |  Loop Objects    || 1 set of associated Loop arrays                         |
        |  Edgeuse Objects || 1 culled set of associated Edgeuse arrays               |
        |                  ||   wireEdge-Edgeuses removed.                            |
        |    m_vEdgeuses   || == [Edgeuses, 1 Edgeuse per WireEdge]                   |
        |  Edge Objects    || Divided sets of associated Edge and WireEdge arrays     |
        |    m_vEdge       || == [FaceEdges, WireEdges]                               |
        |  Vertex Objects  || Divided sets of associatedShellVertex and Vertex arrays |
        |    m_vVertex     || == [ShellVertices, Vertices]                            |
        +------------------++---------------------------------------------------------+
 3. When moving data from usdBrepArrayData to SMBrepData, the UsdBrepArrayData
    pair of Edge and WireEdge lists are combined into a single SmBrepData Edge list,
    pair of ShellVertex and Vertex lists are combined into a single SmBrepData Vertex list,
    and the Edgeuse list is the combination of the FaceEdge-Edgeuses and new WireEdge-Edgeuses.

   a. packing of SMLib SmBrepData arrays:
      m_vEdges    = [USD FaceEdges, USD WireEdges]                      from StartIndex to Count for Edges and WireEdges for Brep_ii
      m_vVertices = [USD Edge_and_Loop_Vertices, USD ShellVertice]      from StartIndex to Count for Vertices and ShellVertices for Brep_ii
      m_vEdgeuses = [USD FaceEdge-Edgeuses, new USD WireEdge-Edgeuses]  from StartIndex to Count for Edgeuses and WireEdges for Brep_ii
                      with - new WireEdge_ii-Edgeuses_Edgeindex = FaceEdge-EdgeuseCount + WireEdge_ii

   b. Edgeuses:
        SMLib : contains an Edgeuse for every FaceEdge, WireEdge, LoopVertex, and ShellVertex.
        USD   : only has Edgeuses for FaceEdges.
                - BrepAppend_SMLibToUsd()        : Edgeuse data for WireEdges, LoopVertices, and ShellVertices is ignored.
                                                          Edgeuse data for FaceEdges in m_vEdgeuses is copied into USD Edgeuse arrays.
                                                           m_sEdgeuse associated arrays = Edgeuse data packed in the order that Edge_Edgeuses appear in m_vEdges.
                - BrepMove_OneUsdBrepToSMLib(): Edgeuse data for FaceEdges is copied from USD edgeuse arrays and
                                                          Edgeuse data for WireEdge, LoopVertices, and ShellVertices are generated from USD Shell and Vertex lists.
                                                           pBrepData_ii->m_vEdgeuses = [FaceEdge-Edgeuses, new WireEdge-Edgeuses, new ShellVertex-Edgeuses, new LoopVertex-Edgeuses].
   c. Vertices:
        SMLib : stores a single list of {ShellVertices, LoopVertices, EdgeVertices}.
        USD   : stores a list of {ShellVertices} and a list of {LoopVertices, EdgeVertices}
                - BrepAppend_SMLibToUsd()        : the single m_sVertices array data is divided between the USD Vertices and VertexShell data arrays.
                                                           vertex indices from pBrepData_ii->m_vShells[jj]->WireEdges->m_lStartVertex, m_lEndVertex
                                                           and                 pBrepData_ii->m_vLoops[jj]->m_lVertex
                                                           and                 pBrepData_ii->m_vEdges[jj]->m_lStartVertex, m_lEndVertex
                                                           are compacted and stored in: m_sLoopVertexIndexArray, m_sEdgeVertexIndicesArray and m_sWireEdgeVertexIndicesArray
                                                             m_sShell_PointPositionArray  = ShellVertex positions packed in the order that ShellVertices appear in m_vShells.
                                                             m_sVertex_PointPositionArray = LoopVertex, EdgeVertex, and WireEdgeVertex positions packed in the order that LoopVertices, EdgeVertices, and WireEdgeVertices appear in m_vLoops and m_vEdges.
                - BrepMove_OneUsdBrepToSMLib(): The USD ShellVertex and Vertices arrays are combined into one SMLib m_sVertices array.
                                                           one VertexData obj is made for every m_sShell_PointPositionArray entry and added to pBrepData_ii->m_sVertices
                                                           one VertexData obj is made for every m_sVertex_PointPositionArray entry and added to pBrepData_ii->m_sVertices
   d. Edges                                                pBrepData_ii->m_sVertices = [mixed{USD LoopVertices, USD EdgeVertices, USD WireEdgeVertices}, USD ShellVertices]
        SMLib : stores a single list of {FaceEdges, WireEdges}
        USD   : stores a list of {FaceEdges} and a list of {WireEdges}
                - BrepAppend_SMLibToUsd()        : m_sWireEdge associated arrays = built in Shell block in the order WireEdges appear in the shells of m_vShells
                                                          m_sEdge associated arrays = built from m_vEdges skipping the wire edges
                                                          edge indices from pBrepData_ii->m_vEdgeuses[jj]->m_lEdgeIndex are 
                                                          compacted and stored in: m_sEdgeuseEdgeIndexArray.
                - BrepMove_OneUsdBrepToSMLib(): The USD WireEdge and Edge arrays are combined into one SMLib m_sEdges array.
                                                          one EdgeData obj is made for every m_sEdge associated array object and added to pBrepData_ii->m_sEdges
                                                          one EdgeData obj is made for every m_sWireEdge associated array object and added to pBrepData_ii->m_sEdges
                                                          pBrepData_ii->m_vEdges = [FaceEdges, WireEdges]
***********************************************************************/
SmStatus BrepMove_OneUsdBrepToSMLib
 (const SmContext         & crSmContext,    // in : Context in which prSmBrep is created 
  const UsdBrepArrayData  & rArrays,        // in : all brep attribute values from BrepReadFromUsdStage()
  UsdBrepArraySpans       & rSpans,         // i/o: Container for Brep_ii object count and 1stIndex values computed from the rArrays member arrays.
  uint32_t                  iBrepIndex,     // in : Index of Tgt Brep to convert to a SmBrep    
  SmBrep                 *& prSmBrep_ii,    // out: created new SmBrep for iBrepIndex     
  SmBoolean                 bHealerIsEnabled) // in : default TRUE: healer is enabled  FALSE: disable healer
  
{           
    // Initialize output to NULL in case of error
    prSmBrep_ii = NULL;
    
    // locals
    SmBrepData           * pBrepData_ii = NULL; // new SmBrepData for iBrepIndex
    SmTArray<SmAttribute*> sAttributes; // list of all attributes added to Breps and Faces for materials

    // build BrepData for BrepIndex
    SmStatus buildStatus = BrepMove_OneUsdBrepToSMLibData(crSmContext, rArrays, rSpans, iBrepIndex, pBrepData_ii, sAttributes);
    
    // Check if BrepData construction failed
    if (buildStatus != SM_SUCCESS || pBrepData_ii == NULL)
      {
        if (buildStatus != SM_SUCCESS && pBrepData_ii != NULL)
          {
            ERR_MSG(_T("BrepMove_OneUsdBrepToSMLib: failed data conversion returned a nonNULL BrepData"));
          }

        delete pBrepData_ii;
        DeleteConversionAttributes(sAttributes);

        // inform the public - aborting this Brep to process subsequent Breps
        TCHAR sMessage[SM_TBLOCK_SIZE];
        SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLib: BrepMove_OneUsdBrepToSMLibData failed for brep index = %u"), iBrepIndex);
        ERR_MSG(sMessage);
        return (buildStatus != SM_SUCCESS) ? buildStatus : SM_ERR;
      }

    SmObjDelete sCleanBrepData(pBrepData_ii);

    // BrepMove_OneUsdBrepToSMLibData validated and restored the packed
    // intersection tolerance as this topology zone tolerance.
    SmZoneTol3d sZoneTol3d = pBrepData_ii->m_sZoneTol3d;

#define MAP_USD_TO_SM_UNUSED_VALUE(a) (((a) == -1) ? SM_UNDEF_ULONG : (a))

#ifdef SM_DEBUG_CODE   
SmBoolean bDebugMe = FALSE;
    if (bDebugMe)
      {
        if (usdBrep_CreateOutputFiles())              
          {
            std::string sPathString = TfStringPrintf("OutputFiles/BrepMove_OneUsdBrepToSMLib() InputBrepArray[%d]_To_OutputBrepData.smb", iBrepIndex);

            // TRUE = open file and rewrite contents, FALSE= open file and append to end
            pBrepData_ii->WriteToFile(sPathString, SM_ASCII, TRUE);

            usdBrep_WriteString("\n\n  FILE_WRITE: " + sPathString + "\n");
          }
      }
#endif // SM_DEBUG_CODE

    // move pBrepData_ii to SmBrep_ii
    SmBrep * pBrep_ii = new (crSmContext) SmBrep();
    SmObjDelete sCleanBrep(pBrep_ii);

    // Kludge. No heal for OmniSolidBreps written by SMLib.
    uint32_t lHealerVersion = (rArrays.m_sCADSource.compare("SMLib") == 0) ? SM_HEALER_VERSION : SM_HEALER_VERSION - 2;
    pBrepData_ii->SetHealerVersion(lHealerVersion);

    // Hard coding SmZoneTol3d here to avoid inflation of tolerance for slender breps,
    // which lead to issues in tessellation.

    pBrepData_ii->m_sZoneTol3d = sZoneTol3d;

    // For performance on tessellation we set the following options FALSE
    // UVTrimCurves will be generated ad hoc
    // Conversion to analytics can be completed as a separate heal step
    SmBoolean bCreateUVTrimCurves = FALSE;
    SmBoolean bConvertToAnalytics = FALSE;
    SmStatus topologyStatus = pBrep_ii->MakeTopologyFromData(pBrepData_ii,        // in : target BrepData object
                                                             sAttributes,         // in : list of all attributes to be assigned to Brep entities
                                                                                  //      currently contains one placeholder attribute
                                                                                  //      for every attribute referenced within the SmBrep topology graph.
                                                             bCreateUVTrimCurves, // in : default TRUE = call SmFace::CreateUVTrimCurves for each face
                                                             bConvertToAnalytics, // in : default TRUE = replace every Surface and Curve with
                                                                                  //        with a derived analytic type when appropriate.
                                                             TRUE,                // in : default FALSE: when reading IGES, allow sensible reset of tolerance
                                                                                  //      when reading breps set TRUE (use original tol);
                                                             FALSE,               // in : bCheckUVTrimCurves
                                                             FALSE,               // in : bAlwaysHeal
                                                             bHealerIsEnabled);   // in : bHealerIsEnabled

    if (topologyStatus != SM_SUCCESS)
      {
        // MakeTopologyFromData has not reached its healer on an error return,
        // so every entry is still live. Delete attributes before the partial
        // topology so their destructors can detach from any users safely.
        DeleteConversionAttributes(sAttributes);

        TCHAR sMessage[SM_TBLOCK_SIZE];
        SM_SPRINTF(sMessage,
                   _T("BrepMove_OneUsdBrepToSMLib: MakeTopologyFromData failed for brep index = %u"),
                   iBrepIndex);
        ERR_MSG(sMessage);
        return topologyStatus;
      }

    // Successful topology construction transfers every converter-created
    // attribute to the Brep. The healer may already have deleted attributes
    // whose last topology user was removed, so discard these non-owning raw
    // pointers without dereferencing them.
    sAttributes.ReSet();

    prSmBrep_ii = pBrep_ii;
    sCleanBrep.Clear();

#ifdef SM_DEBUG_CODE
    if (bDebugMe)
      {
        SM_DUMP_AND_ASSERT_VALID(pBrep_ii) ;

        // move pBrep_ii to pBrepData_ii
        SmBrepData           * pOutputBrepData_ii = new (crSmContext) SmBrepData(FALSE, SM_DS_USD);
        SmObjDelete            sClean(pOutputBrepData_ii);
        SmTArray<SmAttribute*> sOutputAttributes; // list of all attributes added to Breps and Faces for materials

        
        // write pBrepData_ii to file
        pOutputBrepData_ii->FromBrep(*pBrep_ii, sOutputAttributes); // output-BrepData
        if (usdBrep_CreateOutputFiles())              
          {
            std::string sPathString = TfStringPrintf("OutputFiles/BrepMove_OneUsdBrepToSMLib() OutputBrepData[%d]_To_Brep_To_BrepData.smb", iBrepIndex);

            // TRUE = open file and rewrite contents, FALSE= open file and append to end
            pBrepData_ii->WriteToFile(sPathString, SM_ASCII, TRUE);

            usdBrep_WriteString("\n\n  FILE_WRITE: " + sPathString + "\n");
          }
        smgfx_Erase();
        smgfx_SetLook(1,2, 0,0,1); if (pBrep_ii) pBrep_ii->Draw(TRUE); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

#undef MAP_USD_TO_SM_UNUSED_VALUE

    return SM_SUCCESS;
} // end SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib

/*******************************************************************/ /**
PURPOSE: move one AppendedBrepArrayData object into an SmBrepData object

NOTES:
 1. pipeline: = UsdBrepArrayData[iBrepIndex] =>  SmBrepData_ii
***********************************************************************/
SmStatus BrepMove_OneUsdBrepToSMLibData
 (const SmContext         & crSmContext,     // in : Context in which prSmBrep is created 
  const UsdBrepArrayData  & rArrays,         // in : all brep attribute values from BrepReadFromUsdStage()
  UsdBrepArraySpans       & rSpans,          // i/o: Container for Brep_ii object count and 1stIndex values computed from the rArrays member arrays.
  uint32_t                  iBrepIndex,      // in : Index of Tgt Brep to convert to a SmBrep    
  SmBrepData             *& prSmBrepData_ii, // out: created new SmBrep for iBrepIndex            
  SmTArray<SmAttribute*>  & rAttributes)     // out: list of all attributes referenced by index by members of prSmBrepData_ii
{     
    // check state: prSmBrepData_ii null on input - else abort
    AERN_MSG(prSmBrepData_ii == NULL, SM_ERR, 
            _T("SMU_BrepConvert::BrepMove_OneUsdBrepToSMLibData: prSmBrepData_ii is not NULL on input"));

    // init outputs
    rAttributes.ReSet() ;
    SmObjsDelete<SmAttribute*> sCleanAttributes(&rAttributes);
    SmTemporaryChangeValue<SmBrepData*> sResetOutputOnFailure(prSmBrepData_ii, NULL);

    if (iBrepIndex >= rArrays.TotalBrepCount() || iBrepIndex >= rArrays.m_sBrepXSectTol3dArray.size())
      {
        SER_MSG(SM_ERR_INVALID_INPUT,
                _T("BrepMove_OneUsdBrepToSMLibData: Brep index or intersection-tolerance index is out of bounds"));
      }

    // rArrays.StartIndex values for iBrepIndex
    //   note: - no work when iBrepIndex == UsdBrepArraySpans.m_lBrepIndex
    //           (which is the case when called from the loop in BrepMove_UsdToSMLib)
    if(!rSpans.SetStartsAndCountsForBrepIndex(rArrays,     // eff: set Brep_ii start index and count values by walking
                                                            //      array data for every Brep up to Brep_ii
                                               iBrepIndex,  // in : Tgt Brep_ii index
                                               FALSE,       // in : TRUE = set Brep_ii start Indices by walking rArrays data for every Brep prior to Brep_ii
                                                            //      FALSE= use StartIndices as is assuming they are set for Brep_ii.
                                                            //      default:[TRUE]
                                               TRUE))       // in : TRUE = set Brep_ii counts,
                                                            //      FALSE= set Brep_ii icounts=0, ready for next move Brep call.
                                                            //      default:[FALSE]
      {
        SER_MSG(SM_ERR_INVALID_INPUT, _T("BrepMove_OneUsdBrepToSMLibData: malformed packed BrepArray data"));
      }

    prSmBrepData_ii = new (crSmContext) SmBrepData(FALSE, SM_DS_USD);
    SmObjDelete sClean(prSmBrepData_ii);

#define MAP_USD_TO_SM_UNUSED_VALUE(a) (((a) == -1) ? SM_UNDEF_ULONG : (a))

    // locals
    uint32_t iGlobal, iLocal ; 
    bool     bHasUvCurves = rArrays.m_sEdgeuse_CurveNurb_OrderArray.size() == rArrays.m_sEdgeuseEdgeIndexArray.size();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe)
      {
        Dump_BrepArrayData(rArrays, FALSE) ; // FALSE = dump only Topology obj counts, TRUE = also dump array data values
        Dump_SMLibCounts(rArrays);
        Dump_BrepArraySpans(rArrays, rSpans, FALSE) ; // FALSE = dump only Topology obj counts, TRUE = also dump array data values
      }
#endif // SM_DEBUG_CODE
                                                          
    // locals
    // Restore the packed maximum as the topology zone tolerance. Reconstruct
    // the corresponding SMLib intersection tolerance only for approximation.
    SmXSectTol3d           sXSectTol3d   = rArrays.m_sBrepXSectTol3dArray[iBrepIndex] ;
    SmZoneTol3d            sZoneTol3d    = SmTol::GetZoneTol3d(sXSectTol3d) ;
    SmApproxTol3d          sApproxTol3d  = SmTol::GetApproxTol3d(sXSectTol3d) ;

    // populate prSmBrepData_ii with topology and shape objs
      {
        // TODO color, bounding box
        prSmBrepData_ii->m_bManifold                = UNSURE ; // brep type no longer stored in USD; will be determined during brep construction
        prSmBrepData_ii->m_sZoneTol3d               = sZoneTol3d ;
        prSmBrepData_ii->m_dThisModelSizeEstimate   = SM_MODEL_SIZE_ESTIMATE ;     // Not part of the USD model - set to default values - needs update based on BrepBoundingBoxSize.
        prSmBrepData_ii->m_dThisLargeSmallSizeRatio = SM_LARGE_SMALL_SIZE_RATIO ;  // Not part of the USD model - set to default values - needs update based on BrepBoundingBoxSize.
        prSmBrepData_ii->m_sApproxTol3d             = sApproxTol3d ;
        prSmBrepData_ii->m_lStartRegion             = 0; // First region is always the infinite region
        prSmBrepData_ii->m_lNumRegions              = rArrays.m_sBrepRegionCountArray[iBrepIndex] ;
        prSmBrepData_ii->m_vColor.Set(0.0,0.0,0.0) ;  // gwc??? needs a value from USD
        prSmBrepData_ii->m_bIsGeomBorrowed          = FALSE ; // FALSE = curves & surfaces in arrays are internally allocated. 

        // Add the Brep Material and PrimPath Attributes
          {
            // Get the Brep material binding
            SdfPathVector sMaterialPaths ;

            // when different Breps use different Materials - look for a perBrep material
            if(rArrays.m_sBrepMaterial_BrepPathArray.size() > 0)
              {
                if (rArrays.m_sBrepMaterial_BrepPathArray.size() != rArrays.m_sBrepMaterial_BrepIndexArray.size())
                  {
                    SER_MSG(SM_ERR_INVALID_INPUT,
                            _T("BrepMove_OneUsdBrepToSMLibData: Brep material paths and index lists have different sizes"));
                  }

                // for every BrepMaterial look for BrepIndex in the associated indices list
                for (size_t ii = 0; ii < rArrays.m_sBrepMaterial_BrepPathArray.size() && sMaterialPaths.empty(); ++ii)
                  {
                    // Search all indices
                    for (size_t jj = 0; jj < rArrays.m_sBrepMaterial_BrepIndexArray[ii].size() && sMaterialPaths.empty(); ++jj)
                      {
                        // If this brep-type GeomSubset includes this Brep, then assign this material
                        if (rArrays.m_sBrepMaterial_BrepIndexArray[ii][jj] == (int32_t)iBrepIndex)
                          {
                            sMaterialPaths = SdfPathVector{rArrays.m_sBrepMaterial_BrepPathArray[ii]};
                          } // end if this GeomSubset includes this Brep
                      } // end iter every BrepMaterialBrepIndex_jj 
                  } // end iter BrepMaterialPaths_ii 
              } // end if perBrep materials exist branch
            
            // no perBrep material covers this Brep - inherit the BrepArray material, if it has one
            if(sMaterialPaths.empty() && !rArrays.m_sBrepArray_MaterialPath.IsEmpty())
              {
                sMaterialPaths = SdfPathVector{rArrays.m_sBrepArray_MaterialPath};
              }

            // when BrepMaterial was found - add attribute to prSmBrepData_ii
            if(!sMaterialPaths.empty())
              {
                SmSdfPathAttribute * pAttr  = new (crSmContext) SmSdfPathAttribute(SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, sMaterialPaths);

                // index to material attribute for prSmBrepData_ii
                prSmBrepData_ii->m_vAttributes.Add(rAttributes.GetSize());

                // add material attribute to the attributes list
                rAttributes.Add(pAttr);
              }  

            //// Add the Xform Attribute, if it was created
            //if (pXformAttr)
            //{
            //    prSmBrepData_ii->m_vAttributes.Add(sAttributes.GetSize());
            //    sAttributes.Add(pXformAttr);
            //}

            // store crUsdBrep PrimPath as the last SmAttribute in Attributes array
            SdfPathVector        sPrimPaths = SdfPathVector{rArrays.m_sPrimPath} ;  
            SmSdfPathAttribute * pAttr  = new (crSmContext) SmSdfPathAttribute(SM_AI_BREP_ARRAY, SM_AB_REFERENCE, sPrimPaths);
            prSmBrepData_ii->m_vAttributes.Add(rAttributes.GetSize());
            rAttributes.Add(pAttr);
          } // end Brep Attributes

        // Regions
        prSmBrepData_ii->m_vRegions.SetSize(rSpans.m_lRegionCount) ;
        uint32_t lShellStartIndex = 0 ;
        for(iGlobal=rSpans.m_lRegionStartIndex, iLocal=0; iLocal<rSpans.m_lRegionCount; iGlobal++, iLocal++)
          {
            SmRegionData & rRegionData = prSmBrepData_ii->m_vRegions[iLocal];
            rRegionData.ReSet(); // set all member values back to default values.

            // locals
            
            // load region[iLocal] values
            rRegionData.m_lStartShell = lShellStartIndex ;
            rRegionData.m_lNumShells  = rArrays.m_sRegionShellCountArray[iGlobal];
            rRegionData.m_bIsVoidFlag = (rArrays.m_sRegionTypeArray[iGlobal] == UsdBrepSolidTokens->voidRegion) ? TRUE : FALSE;

            // make unused values look like SMLib unused values (simplifies debugging)
            if(rRegionData.m_lFlags      == SM_UNDEF_ULONG) { rRegionData.m_lFlags      = 0 ; } 
            if(rRegionData.m_lUserIndex1 == SM_UNDEF_ULONG) { rRegionData.m_lUserIndex1 = 0 ; }
            if(rRegionData.m_lUserIndex2 == SM_UNDEF_ULONG) { rRegionData.m_lUserIndex2 = 0 ; } 

            // leaving these region[iLocal] obj members unset
            //    prSmBrepData_ii->m_vRegions[iLocal].m_sAttributes - done in ReSet() call
            //    prSmBrepData_ii->m_vRegions[iLocal].m_pUserPtr1
            //    prSmBrepData_ii->m_vRegions[iLocal].m_bIsSmallTopology

            // accumulate topology counts
            lShellStartIndex += rArrays.m_sRegionShellCountArray[iGlobal];

          } // end Regions

        // Shells - Set m_vShells and adds entries into m_vEdgeuses
        prSmBrepData_ii->m_vShells.  SetSize(rSpans.m_lShellCount) ;
        prSmBrepData_ii->m_vEdgeuses.SetSize(rSpans.m_lWireEdgeCount + rSpans.m_lEdgeuseCount) ;   // m_vEdgeuses = [ 1 SmEUData perWireEdges, 1 SmEUData per FaceEdge,no SmEUdata for LoopVertices or ShellVertices] mimic SMLib - WireEdge-Edgeuses 1st, FaceEdge-Edgeuses 2nd
        prSmBrepData_ii->m_vVertices.SetSize(rSpans.m_lShellVertexCount + rSpans.m_lVertexCount) ; // m_vVertices = [ ShellVertices, mixed:{EdgeVertices, WireEdgeVetices, LoopVertices}] mimic SMLib - ShellVertices 1st other vertices 2nd

        uint32_t lFaceuseStartIndex       = 0 ;
        uint32_t lWireEdgeIndex           = 0 ;
        uint32_t lShellVertexPointIndex   = 0 ;
        SmTArray<SmEdge*> sWireEdges ;
        for(iGlobal=rSpans.m_lShellStartIndex, iLocal=0; iLocal<rSpans.m_lShellCount; iGlobal++, iLocal++)
          {
            SmShellData & rShellData = prSmBrepData_ii->m_vShells[iLocal];
            rShellData.ReSet();

            // locals
            uint32_t ii ; 
            const bool bIsPointShell = rArrays.IsBrepPointShell(iGlobal);
            uint32_t sShellTypeFlag = (rArrays.m_sShellFaceuseCountArray[iGlobal]  > 0) ?                    0 :  // faceuse shell
                                      (rArrays.m_sShellWireEdgeCountArray[iGlobal] > 0) ?                    1 :  // wireEdge shell
                                      bIsPointShell ?                                                     2 :  // Vertex shell
                                                                                                             3 ;  // error: unknown case
            // load shell[iLocal] values
            rShellData.m_lShellType   = sShellTypeFlag ;
            rShellData.m_lNumFaceuses = 0 ;

            // make unused values look like SMLib unused values (simplifies debugging)
            if(rShellData.m_lFlags      == SM_UNDEF_ULONG) { rShellData.m_lFlags      = 0 ; } 
            if(rShellData.m_lUserIndex1 == SM_UNDEF_ULONG) { rShellData.m_lUserIndex1 = 0 ; }
            if(rShellData.m_lUserIndex2 == SM_UNDEF_ULONG) { rShellData.m_lUserIndex2 = 0 ; } 

            // Shells with Faceuses
            if(rArrays.m_sShellFaceuseCountArray[iGlobal] > 0)
              { 
                rShellData.m_lFaceuseStart = lFaceuseStartIndex ;
                rShellData.m_lNumFaceuses  = rArrays.m_sShellFaceuseCountArray[iGlobal];
                
                lFaceuseStartIndex += rArrays.m_sShellFaceuseCountArray[iGlobal];
              } 

            // Shells with WireEdges  // build m_vEdges = [FaceEdges, WireEdges], m_vEdgeuses = [WireEdge-Edgeuses, FaceEdge-Edgeuses]
            if(rArrays.m_sShellWireEdgeCountArray[iGlobal] > 0)
              {
                rShellData.m_lEdge = lWireEdgeIndex + rSpans.m_lEdgeCount ;

                // for every WireEdge                                           // build sEdges    = [FaceEdges, WireEdges]
                for(ii=0;ii<rArrays.m_sShellWireEdgeCountArray[iGlobal];ii++)   // build sEdgeuses = [WireEdge_Edgeuses, FaceEdge_Edgeuses]
                  { 
                    // 1 new WireEdge-Edgeuse per WireEdge with  WireEdge-EdgeuseIndex      = lWireEdgeIndex, 
                    //                                           WireEdge-Edgeuse_EdgeIndex = EdgeCount    + lWireEdgeIndex
                    // place WireEdge-Edgeuses 1st on the m_vEdgeuses array to match SMLib use
                    SmEUData & rEdgeuseData = prSmBrepData_ii->m_vEdgeuses[lWireEdgeIndex];
                    rEdgeuseData.ReSet() ; 
                    rEdgeuseData.m_lEUType  = 2 ; // 2 = WireEdge
                    rEdgeuseData.m_lEdge    = rSpans.m_lEdgeCount + lWireEdgeIndex ;
                    rEdgeuseData.m_lShell   = iLocal ;
                    
                    // make unused values look like SMLib unused values (simplifies debugging)
                    if(rEdgeuseData.m_bOrientation    == UNSURE)         { rEdgeuseData.m_bOrientation    = TRUE ; }
                    if(rEdgeuseData.m_lFlags          == SM_UNDEF_ULONG) { rEdgeuseData.m_lFlags          = 0 ; }
                    if(rEdgeuseData.m_lUserIndex1     == SM_UNDEF_ULONG) { rEdgeuseData.m_lUserIndex1     = 0 ; }
                    if(rEdgeuseData.m_lUserIndex2     == SM_UNDEF_ULONG) { rEdgeuseData.m_lUserIndex2     = 0 ; }
                    if(rEdgeuseData.m_lMateFlags      == SM_UNDEF_ULONG) { rEdgeuseData.m_lMateFlags      = 0 ; }
                    if(rEdgeuseData.m_lMateUserIndex1 == SM_UNDEF_ULONG) { rEdgeuseData.m_lMateUserIndex1 = 0 ; }
                    if(rEdgeuseData.m_lMateUserIndex2 == SM_UNDEF_ULONG) { rEdgeuseData.m_lMateUserIndex2 = 0 ; }
                    if(rEdgeuseData.m_lUVCurve        == SM_UNDEF_ULONG) { rEdgeuseData.m_lUVCurve        = SM_NO_OBJECT ; }  // not like SMLib but consistent with all other edgeuse values
                    
                    lWireEdgeIndex ++ ;
                  } // end iter every Shell-WireEdge
              } // end Shell has WireEdges check

            // VertexShells   
            if(sShellTypeFlag == 2)
              {
                SM_ASSERT_MSG(rShellData.m_lNumFaceuses == 0 && rShellData.m_lEdge == SM_UNDEF_ULONG, _T("SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib: Found a VertexShell that thinks it has Faceuses and/or WireEdges- needs debugging")) ;
                rShellData.m_lVertex        = lShellVertexPointIndex ;  // place Shell vertices before other vertices
                uint32_t iVertexShellPointGlobal = rArrays.MapLocalToGlobal(lShellVertexPointIndex, rSpans.m_lShellPointPosition_StartIndex) ;
                    
                // 1 new ShellVertex->Vertex per VertexShell with ShellVertex->Vertex_VertexIndex = ShellVertex_ii + VertexCount
                SmVertexData & rShell_VertexData = prSmBrepData_ii->m_vVertices[rShellData.m_lVertex];
                rShell_VertexData.ReSet() ;
                rShell_VertexData.m_sZoneTol3d  = sZoneTol3d;
                
                // make unused values look like SMLib unused values (simplifies debugging)
                if(rShell_VertexData.m_lFlags      == SM_UNDEF_ULONG) { rShell_VertexData.m_lFlags      = 0 ; } 
                if(rShell_VertexData.m_lUserIndex1 == SM_UNDEF_ULONG) { rShell_VertexData.m_lUserIndex1 = 0 ; }
                if(rShell_VertexData.m_lUserIndex2 == SM_UNDEF_ULONG) { rShell_VertexData.m_lUserIndex2 = 0 ; } 
                rShell_VertexData.m_vPoint.Set(rArrays.m_sShell_PointPositionArray[iVertexShellPointGlobal][0],
                                               rArrays.m_sShell_PointPositionArray[iVertexShellPointGlobal][1],
                                               rArrays.m_sShell_PointPositionArray[iVertexShellPointGlobal][2]) ;
                
                lShellVertexPointIndex ++ ;
              } // end VertexShell check                                                                                                                   

            // accumulate Faceuse & WireEdges topology counts - ShellVertexPointIndex already updated
                               
          } // end Shells

        // Faceuses
        prSmBrepData_ii->m_vFaceuses.SetSize(rSpans.m_lFaceuseCount) ;
        for(iGlobal=rSpans.m_lFaceuseStartIndex, iLocal=0; iLocal<rSpans.m_lFaceuseCount; iGlobal++, iLocal++)
          {
            SmFaceuseData & rFaceuseData = prSmBrepData_ii->m_vFaceuses[iLocal];
            rFaceuseData.ReSet() ; 

            // locals
            SmBoolean       bOrientation = (rArrays.m_sFaceuseOrientationTypeArray[iGlobal] == UsdBrepSolidTokens->same) ? TRUE : FALSE;

            // load faceuse[iLocal] values
            rFaceuseData.m_lFace        = rArrays.MapGlobalToLocal(rArrays.m_sFaceuseFaceIndexArray[iGlobal], rSpans.m_lFaceStartIndex) ;
            rFaceuseData.m_bOrientation = bOrientation ;

            // make unused values look like SMLib unused values (simplifies debugging)
            if(rFaceuseData.m_lFlags      == SM_UNDEF_ULONG) { rFaceuseData.m_lFlags      = 0 ; } 
            if(rFaceuseData.m_lUserIndex1 == SM_UNDEF_ULONG) { rFaceuseData.m_lUserIndex1 = 0 ; } 
            if(rFaceuseData.m_lUserIndex2 == SM_UNDEF_ULONG) { rFaceuseData.m_lUserIndex2 = 0 ; } 

          } // end Faceuses

        // FaceCount = FaceuseCount / 2
        // gwc_moved_count_accumulation_to_top_of_method // // rSpans.m_lFaceCount = rSpans.m_lFaceuseCount / 2 ; 

        // Faces
        prSmBrepData_ii->m_vFaces.SetSize(rSpans.m_lFaceCount) ;
        uint32_t lLoopIndex = 0 ;
        for(iGlobal=rSpans.m_lFaceStartIndex, iLocal=0; iLocal<rSpans.m_lFaceCount; iGlobal++, iLocal++)
          {
            AERN_MSG(static_cast<size_t>(iGlobal) < rArrays.m_sFaceSurfaceTypeArray.size() &&
                     static_cast<size_t>(iGlobal) < rArrays.m_sFaceLoopCountArray.size(),
                     SM_ERR,
                    _T("BrepMove_OneUsdBrepToSMLibData: iGlobal face index out of bounds - this is a bug"));

            SmFaceData & rFaceData = prSmBrepData_ii->m_vFaces[iLocal];
            rFaceData.ReSet();

            // locals
            // Mapping USD face:trimType -> SMLib m_bRectangularTrim (tri-state):
            //   "rectangular" -> TRUE   (definitively naturally/rectangularly trimmed)
            //   "general"     -> UNSURE (USD has no way to distinguish FALSE from UNSURE;
            //                            treat as "not determined" so round-trips of
            //                            freshly-built SMLib geometry (all UNSURE) are
            //                            lossless, and the tessellator re-computes as needed)
            //   anything else -> UNSURE
            SmBoolean bRectangularTrim = (rArrays.m_sFaceTrimTypeArray[iGlobal] == UsdBrepSolidTokens->rectangular) ? TRUE
                                                                                                                : UNSURE ;
            SmExtent2d sDomain(rArrays.m_sFaceRangeArray[2*iGlobal  ][0],   // Umin
                               rArrays.m_sFaceRangeArray[2*iGlobal  ][1],   // Vmin
                               rArrays.m_sFaceRangeArray[2*iGlobal+1][0],   // Umax
                               rArrays.m_sFaceRangeArray[2*iGlobal+1][1]);  // Vmax

            // USD stores analytic surface face:range angular parameters in radians,
            // but SMLib analytic surfaces (SmSphere, SmCylinder, SmCone, SmTorus)
            // use degrees for their STEP parameterization.
            // Convert angular parameters from radians to degrees.
            if (rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceSphereAPI
             || rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
              {
                // Sphere/Torus: both U (rotation) and V (latitude/tube angle) are angular
                sDomain = SmExtent2d(UsdRadiansToSMLibDegrees(sDomain.GetMin().x),
                                     UsdRadiansToSMLibDegrees(sDomain.GetMin().y),
                                     UsdRadiansToSMLibDegrees(sDomain.GetMax().x),
                                     UsdRadiansToSMLibDegrees(sDomain.GetMax().y));
              }
            else if (rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI
                  || rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
              {
                // Cylinder/Cone: only U (rotation) is angular, V is distance along axis
                sDomain = SmExtent2d(UsdRadiansToSMLibDegrees(sDomain.GetMin().x), sDomain.GetMin().y,
                                     UsdRadiansToSMLibDegrees(sDomain.GetMax().x), sDomain.GetMax().y);
              }
                       
            // load face[iLocal] values
#ifdef SM_USE_OLDTOL
            rFaceData.m_sZoneTol3d       = sZoneTol3d ;         // OldTol: face's tolerant neighborhood offset size
#endif // SM_USE_OLDTOL
            rFaceData.m_lSurface         = prSmBrepData_ii->m_vSurfaces.GetSize();
            rFaceData.m_lStartLoop       = lLoopIndex ;
            rFaceData.m_lNumLoops        = rArrays.m_sFaceLoopCountArray[iGlobal];
            rFaceData.m_vUVDomain.Set(sDomain);
            rFaceData.m_bRectangularTrim = bRectangularTrim;

            // make unused values look like SMLib unused values (simplifies debugging)
            if(rFaceData.m_lFlags      == SM_UNDEF_ULONG) { rFaceData.m_lFlags      = 0 ; } 
            if(rFaceData.m_lUserIndex1 == SM_UNDEF_ULONG) { rFaceData.m_lUserIndex1 = 0 ; }
            if(rFaceData.m_lUserIndex2 == SM_UNDEF_ULONG) { rFaceData.m_lUserIndex2 = 0 ; } 

            // Create Face->SmSurface and add it to prSmBrepData_ii->m_vSurfaces
            if (rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceSphereAPI)
              {
                // Create SmSphere from sphere arrays
                uint32_t iSphereIndex = rSpans.m_lFaceSphereSurface_StartIndex ;
                GfVec3d sCenter       = rArrays.m_sFace_SurfaceSphere_CenterArray[iSphereIndex] ;
                GfVec3d sAxis         = rArrays.m_sFace_SurfaceSphere_AxisArray[iSphereIndex] ;
                GfVec3d sRefDir       = rArrays.m_sFace_SurfaceSphere_RefDirectionArray[iSphereIndex] ;
                double  dRadius       = rArrays.m_sFace_SurfaceSphere_RadiusArray[iSphereIndex] ;

                // Build coordinate system: X = refDirection, Y = axis cross refDirection
                SmVector3d sXAxis(sRefDir[0], sRefDir[1], sRefDir[2]) ;
                SmVector3d sZAxis(sAxis[0], sAxis[1], sAxis[2]) ;
                SmVector3d sYAxis = sZAxis * sXAxis ; // cross product

                SmAxis2Placement sOrigin ;
                sOrigin.SetCanonical(
                    SmPoint3d(sCenter[0], sCenter[1], sCenter[2]),  // origin
                    sXAxis,                                          // x axis (refDirection)
                    sYAxis                                           // y axis (axis cross refDirection)
                );

                SmSphere * pSphere = NULL ;
                SmStatus stat = SmSphere::CreateCanonical(*prSmBrepData_ii->GetContext(), sOrigin, dRadius, pSphere) ;
                SmObjDelete sSphereCleanup(pSphere);
                if (stat != SM_SUCCESS || pSphere == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmSphere::CreateCanonical failed for face index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }
                if (sDomain.IsBounded())
                  {
                    SmStatus adjStat = pSphere->AdjustSTEPUVDomain(sDomain) ;
                    if (adjStat != SM_SUCCESS)
                      {
                        TCHAR sMessage[SM_TBLOCK_SIZE];
                        SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: AdjustSTEPUVDomain failed for face index %u"), iGlobal);
                        ERR_MSG(sMessage);
                        prSmBrepData_ii = NULL;
                        return adjStat;
                      }
                  }
                prSmBrepData_ii->m_vSurfaces.Add(pSphere);
                sSphereCleanup.Clear();
                rSpans.m_lFaceSphereSurface_StartIndex++ ;
              }
            else if (rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfacePlaneAPI)
              {
                uint32_t iPlaneIndex = rSpans.m_lFacePlaneSurface_StartIndex ;
                GfVec3d sOriginPt = rArrays.m_sFace_SurfacePlane_OriginArray[iPlaneIndex] ;
                GfVec3d sAxis     = rArrays.m_sFace_SurfacePlane_AxisArray[iPlaneIndex] ;
                GfVec3d sRefDir   = rArrays.m_sFace_SurfacePlane_RefDirectionArray[iPlaneIndex] ;

                SmVector3d sXAxis(sRefDir[0], sRefDir[1], sRefDir[2]) ;
                SmVector3d sZAxis(sAxis[0], sAxis[1], sAxis[2]) ;
                SmVector3d sYAxis = sZAxis * sXAxis ;

                SmAxis2Placement sPlacement ;
                sPlacement.SetCanonical(
                    SmPoint3d(sOriginPt[0], sOriginPt[1], sOriginPt[2]),
                    sXAxis,
                    sYAxis
                );

                SmPlane * pPlane = NULL ;
                SmStatus stat = SmPlane::CreateCanonical(*prSmBrepData_ii->GetContext(), sPlacement, pPlane) ;
                SmObjDelete sPlaneCleanup(pPlane);
                if (stat != SM_SUCCESS || pPlane == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmPlane::CreateCanonical failed for face index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }
                // CreateCanonical sets the plane's UV domain to [-SM_INFINITE_PARAMETER, SM_INFINITE_PARAMETER].
                // The face's trim curves use sDomain coordinates (e.g. [0,1]), so the plane's analytical
                // domain must match. AdjustSTEPUVDomain restricts the domain without changing the
                // parameterization (UVScale stays 1,1), unlike Reparameterize which would rescale the
                // plane so that sDomain covers the entire infinite 3D extent.
                if (sDomain.IsBounded())
                  {
                    SmStatus adjStat = pPlane->AdjustSTEPUVDomain(sDomain) ;
                    if (adjStat != SM_SUCCESS)
                      {
                        TCHAR sMessage[SM_TBLOCK_SIZE];
                        SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: AdjustSTEPUVDomain failed for face index %u"), iGlobal);
                        ERR_MSG(sMessage);
                        prSmBrepData_ii = NULL;
                        return adjStat;
                      }
                  }
                prSmBrepData_ii->m_vSurfaces.Add(pPlane);
                sPlaneCleanup.Clear();
                rSpans.m_lFacePlaneSurface_StartIndex++ ;
              }
            else if (rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI)
              {
                uint32_t iCylIndex = rSpans.m_lFaceCylinderSurface_StartIndex ;
                GfVec3d sOriginPt = rArrays.m_sFace_SurfaceCylinder_OriginArray[iCylIndex] ;
                GfVec3d sAxis     = rArrays.m_sFace_SurfaceCylinder_AxisArray[iCylIndex] ;
                GfVec3d sRefDir   = rArrays.m_sFace_SurfaceCylinder_RefDirectionArray[iCylIndex] ;
                double  dRadius   = rArrays.m_sFace_SurfaceCylinder_RadiusArray[iCylIndex] ;

                SmVector3d sXAxis(sRefDir[0], sRefDir[1], sRefDir[2]) ;
                SmVector3d sZAxis(sAxis[0], sAxis[1], sAxis[2]) ;
                SmVector3d sYAxis = sZAxis * sXAxis ;

                SmAxis2Placement sPlacement ;
                sPlacement.SetCanonical(
                    SmPoint3d(sOriginPt[0], sOriginPt[1], sOriginPt[2]),
                    sXAxis,
                    sYAxis
                );

                SmCylinder * pCylinder = NULL ;
                SmStatus stat = SmCylinder::CreateCanonical(*prSmBrepData_ii->GetContext(), sPlacement, dRadius, pCylinder) ;
                SmObjDelete sCylinderCleanup(pCylinder);
                if (stat != SM_SUCCESS || pCylinder == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmCylinder::CreateCanonical failed for face index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }
                if (sDomain.IsBounded())
                  {
                    SmStatus adjStat = pCylinder->AdjustSTEPUVDomain(sDomain) ;
                    if (adjStat != SM_SUCCESS)
                      {
                        TCHAR sMessage[SM_TBLOCK_SIZE];
                        SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: AdjustSTEPUVDomain failed for face index %u"), iGlobal);
                        ERR_MSG(sMessage);
                        prSmBrepData_ii = NULL;
                        return adjStat;
                      }
                  }
                prSmBrepData_ii->m_vSurfaces.Add(pCylinder);
                sCylinderCleanup.Clear();
                rSpans.m_lFaceCylinderSurface_StartIndex++ ;
              }
            else if (rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceConeAPI)
              {
                uint32_t iConeIndex = rSpans.m_lFaceConeSurface_StartIndex ;
                GfVec3d sOriginPt   = rArrays.m_sFace_SurfaceCone_OriginArray[iConeIndex] ;
                GfVec3d sAxis       = rArrays.m_sFace_SurfaceCone_AxisArray[iConeIndex] ;
                GfVec3d sRefDir     = rArrays.m_sFace_SurfaceCone_RefDirectionArray[iConeIndex] ;
                double  dRadius     = rArrays.m_sFace_SurfaceCone_RadiusArray[iConeIndex] ;
                double  dSemiAngle  = rArrays.m_sFace_SurfaceCone_SemiAngleArray[iConeIndex] ;

                SmVector3d sXAxis(sRefDir[0], sRefDir[1], sRefDir[2]) ;
                SmVector3d sZAxis(sAxis[0], sAxis[1], sAxis[2]) ;
                double dSemiAngleDeg = SM_RAD2DEG(dSemiAngle) ;

                // SmCone::CreateCanonical requires a non-negative semi-angle. A cone with a
                // negative USD semi-angle has an exactly equivalent positive-angle form when
                // its axis and both STEP parameters are reversed. Reversing both parameters
                // preserves the surface normal as well as the evaluated points.
                if (dSemiAngleDeg < 0.0)
                  {
                    dSemiAngleDeg = -dSemiAngleDeg ;
                    sZAxis = -sZAxis ;
                    sDomain.SetMinMax(-sDomain.GetUMax(), -sDomain.GetVMax(),
                                      -sDomain.GetUMin(), -sDomain.GetVMin()) ;
                  }

                SmCone * pCone = NULL ;
                SmStatus stat = SM_ERR ;
                SmVector3d sYAxis = sZAxis * sXAxis ;
                SmPoint3d sConeOrigin(sOriginPt[0], sOriginPt[1], sOriginPt[2]) ;

                if (sDomain.IsBounded())
                  {
                    // AdjustSTEPUVDomain on an initially unbounded SmCone does not preserve the
                    // surface when the requested axial interval starts away from zero. Construct
                    // the equivalent finite segment directly at Vmin and normalize its SMLib axial
                    // parameter to [0, height]. Analytic-face UV curves are regenerated from the 3D
                    // edges, so no authored trim coordinates depend on the original V offset here.
                    const double dVMin = sDomain.GetVMin() ;
                    const double dVMax = sDomain.GetVMax() ;
                    const double dHeight = dVMax - dVMin ;
                    const double dTanSemiAngle = std::tan(SM_DEG2RAD(dSemiAngleDeg)) ;
                    const double dBottomRadius = dRadius + dVMin * dTanSemiAngle ;
                    const double dTopRadius = dRadius + dVMax * dTanSemiAngle ;
                    if (dHeight > SM_EFF_ZERO && dBottomRadius >= -SM_EFF_ZERO && dTopRadius >= -SM_EFF_ZERO)
                      {
                        sConeOrigin += dVMin * sZAxis ;
                        SmAxis2Placement sPlacement ;
                        sPlacement.SetCanonical(sConeOrigin, sXAxis, sYAxis) ;
                        stat = SmCone::CreateCanonical(
                            *prSmBrepData_ii->GetContext(),
                            sPlacement,
                            smos_Max(0.0, dBottomRadius),
                            smos_Max(0.0, dTopRadius),
                            dHeight,
                            pCone) ;
                        sDomain.SetMinMax(sDomain.GetUMin(), 0.0, sDomain.GetUMax(), dHeight) ;
                      }
                  }
                else
                  {
                    SmAxis2Placement sPlacement ;
                    sPlacement.SetCanonical(sConeOrigin, sXAxis, sYAxis) ;
                    stat = SmCone::CreateCanonical(*prSmBrepData_ii->GetContext(), sPlacement, dRadius, dSemiAngleDeg, pCone) ;
                  }
                SmObjDelete sConeCleanup(pCone);
                if (stat != SM_SUCCESS || pCone == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmCone::CreateCanonical failed for face index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }

                if (sDomain.IsBounded())
                  {
                    SmStatus adjStat = pCone->AdjustSTEPUVDomain(sDomain) ;
                    if (adjStat != SM_SUCCESS)
                      {
                        TCHAR sMessage[SM_TBLOCK_SIZE];
                        SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: AdjustSTEPUVDomain failed for face index %u"), iGlobal);
                        ERR_MSG(sMessage);
                        prSmBrepData_ii = NULL;
                        return adjStat;
                      }
                  }
                prSmBrepData_ii->m_vSurfaces.Add(pCone);
                sConeCleanup.Clear();
                rSpans.m_lFaceConeSurface_StartIndex++ ;
              }
            else if (rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceTorusAPI)
              {
                uint32_t iTorusIndex = rSpans.m_lFaceTorusSurface_StartIndex ;
                GfVec3d sOriginPt    = rArrays.m_sFace_SurfaceTorus_OriginArray[iTorusIndex] ;
                GfVec3d sAxis        = rArrays.m_sFace_SurfaceTorus_AxisArray[iTorusIndex] ;
                GfVec3d sRefDir      = rArrays.m_sFace_SurfaceTorus_RefDirectionArray[iTorusIndex] ;
                double  dMajorRadius = rArrays.m_sFace_SurfaceTorus_MajorRadiusArray[iTorusIndex] ;
                double  dMinorRadius = rArrays.m_sFace_SurfaceTorus_MinorRadiusArray[iTorusIndex] ;

                SmVector3d sXAxis(sRefDir[0], sRefDir[1], sRefDir[2]) ;
                SmVector3d sZAxis(sAxis[0], sAxis[1], sAxis[2]) ;
                SmVector3d sYAxis = sZAxis * sXAxis ;

                SmAxis2Placement sPlacement ;
                sPlacement.SetCanonical(
                    SmPoint3d(sOriginPt[0], sOriginPt[1], sOriginPt[2]),
                    sXAxis,
                    sYAxis
                );

                SmTorus * pTorus = NULL ;
                SmStatus stat = SmTorus::CreateCanonical(*prSmBrepData_ii->GetContext(), sPlacement, dMajorRadius, dMinorRadius, pTorus) ;
                SmObjDelete sTorusCleanup(pTorus);
                if (stat != SM_SUCCESS || pTorus == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmTorus::CreateCanonical failed for face index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }

                if (sDomain.IsBounded())
                  {
                    SmStatus adjStat = pTorus->AdjustSTEPUVDomain(sDomain) ;
                    if (adjStat != SM_SUCCESS)
                      {
                        TCHAR sMessage[SM_TBLOCK_SIZE];
                        SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: AdjustSTEPUVDomain failed for face index %u"), iGlobal);
                        ERR_MSG(sMessage);
                        prSmBrepData_ii = NULL;
                        return adjStat;
                      }
                  }
                prSmBrepData_ii->m_vSurfaces.Add(pTorus);
                sTorusCleanup.Clear();
                rSpans.m_lFaceTorusSurface_StartIndex++ ;
              }
            else if (rArrays.m_sFaceSurfaceTypeArray[iGlobal] == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
              {
                SmBSplineSurface * pBSplineSurface = smu_CreateSmBSplineSurface3d_ForFaceIndex(*prSmBrepData_ii->GetContext(), 
                                                                                               rArrays, 
                                                                                               rSpans, 
                                                                                               rSpans.m_lFaceBSplineSurface_StartIndex) ;
                if (pBSplineSurface == NULL)
                  {
                    // inform the public - aborting this Brep to process subsequent Breps
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: smu_CreateSmBSplineSurface3d_ForFaceIndex %u - returned NULL"), iGlobal);
                    ERR_MSG(sMessage);
                    return(SM_ERR);

                  }
                prSmBrepData_ii->m_vSurfaces.Add(pBSplineSurface);
                rSpans.m_lFaceBSplineSurface_StartIndex++ ;
              }

            // SmFaceData stores the NURBS parameter domain consumed by MakeTopologyFromData, while
            // face:range for analytic USD surfaces is a STEP domain (angles are converted to degrees
            // above). AdjustSTEPUVDomain rebuilds the analytic surface over that STEP range, but its
            // natural domain is the resulting NURBS interval. Convert the face domain as well so a
            // shifted angular range (for example 270..360 degrees) is not intersected with a NURBS
            // interval beginning at zero and cleared as empty.
            const TfToken & rSurfaceType = rArrays.m_sFaceSurfaceTypeArray[iGlobal];
            const bool bAnalyticSurface = rSurfaceType == UsdBrepSurfaceTokens->brepSurfaceSphereAPI
                                       || rSurfaceType == UsdBrepSurfaceTokens->brepSurfacePlaneAPI
                                       || rSurfaceType == UsdBrepSurfaceTokens->brepSurfaceCylinderAPI
                                       || rSurfaceType == UsdBrepSurfaceTokens->brepSurfaceConeAPI
                                       || rSurfaceType == UsdBrepSurfaceTokens->brepSurfaceTorusAPI;
            if (bAnalyticSurface && sDomain.IsBounded())
              {
                SmSurface * pFaceSurface = prSmBrepData_ii->m_vSurfaces[prSmBrepData_ii->m_vSurfaces.GetSize() - 1];
                SmExtent2d sNurbDomain;
                SmStatus domainStat = pFaceSurface->ConvertDomainFromSTEPToNURBS(sDomain, sNurbDomain);
                if (domainStat != SM_SUCCESS)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF(
                      sMessage,
                      _T("BrepMove_OneUsdBrepToSMLibData: STEP-to-NURBS face domain conversion failed for face index %u"),
                      iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return domainStat;
                  }
                rFaceData.m_vUVDomain.Set(sNurbDomain);
              }

            // accumulate topology and shape counts
            lLoopIndex += rArrays.m_sFaceLoopCountArray[iGlobal];

            // gwc_moved_count_accumulation_to_top_of_method                                         

          } // end Faces

        // FaceData material bindings
        if(rArrays.m_sFaceMaterial_FacePathArray.size() > 0)
          {
            if (rArrays.m_sFaceMaterial_FacePathArray.size() != rArrays.m_sFaceMaterial_FaceIndexArray.size())
              {
                SER_MSG(SM_ERR_INVALID_INPUT,
                        _T("BrepMove_OneUsdBrepToSMLibData: Face material paths and index lists have different sizes"));
              }

            // when different Faces use different Materials - set up a manyFaceUsers-to-EachFaceMaterial relationship
            //   - add attributes to sAttributes for each AppendedArray FaceMaterial_ii 
            //   - for every FaceData_jj using FaceMaterial_ii, add FaceMaterial_ii sAttributeIndex to FaceData_jj attributeIndex Array
            // locals
            size_t ii, jj ;
            SmSdfPathAttribute * pAttr                 = NULL ;
            int32_t              lFaceData_GlobalIndex = 0 ;
            uint32_t             lFaceData_localIndex  = 0 ;
            SmFaceData         * pFaceData_jj          = NULL ;

            // for every FaceMaterial
            for(ii=0;ii<rArrays.m_sFaceMaterial_FacePathArray.size();++ii)
              {
                pAttr = NULL;

                // for every FaceData using this FaceMaterial
                for(jj=0;jj<rArrays.m_sFaceMaterial_FaceIndexArray[ii].size();++jj)
                  {
                    lFaceData_GlobalIndex = rArrays.m_sFaceMaterial_FaceIndexArray[ii][jj] ;

                    if (lFaceData_GlobalIndex < 0 ||
                        static_cast<size_t>(lFaceData_GlobalIndex) >= rArrays.m_sFaceLoopCountArray.size())
                      {
                        SER_MSG(SM_ERR_INVALID_INPUT,
                                _T("BrepMove_OneUsdBrepToSMLibData: Face material index is out of bounds"));
                      }

                    // Face material indices are global to the packed BrepArray.
                    // Only attach users owned by the member currently being converted.
                    if (static_cast<uint32_t>(lFaceData_GlobalIndex) < rSpans.m_lFaceStartIndex ||
                        static_cast<uint32_t>(lFaceData_GlobalIndex) - rSpans.m_lFaceStartIndex >= rSpans.m_lFaceCount)
                      {
                        continue;
                      }

                    // Create the attribute lazily so this member does not own
                    // an attribute whose users all belong to another member.
                    if (pAttr == NULL)
                      {
                        SdfPathVector sPaths = SdfPathVector{rArrays.m_sFaceMaterial_FacePathArray[ii]};
                        pAttr = new (crSmContext) SmSdfPathAttribute(SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, sPaths);
                        rAttributes.Add(pAttr);
                      }

                    // add attributeIndex to each used FaceData->m_sAttributes array
                    lFaceData_localIndex = static_cast<uint32_t>(lFaceData_GlobalIndex) - rSpans.m_lFaceStartIndex;
                    pFaceData_jj          = &prSmBrepData_ii->m_vFaces[lFaceData_localIndex] ;
                    pFaceData_jj->m_sAttributes.Add(rAttributes.GetSize() - 1) ;
                  } // end iter every FaceData_jj using FaceMaterial_ii
              } // end iter every FaceMaterial_jj
          } // end FaceData material bindings

        // Loops
        prSmBrepData_ii->m_vLoops.SetSize(rSpans.m_lLoopCount) ;
        uint32_t lEdguseIndex = rSpans.m_lWireEdgeCount; // build m_vEdgeuses = [ WireEdge-Edgeuses, FaceEdge-Edgeuses, no LoopVertex or ShellVertex Edgeuses ]
        for(iGlobal=rSpans.m_lLoopStartIndex, iLocal=0; iLocal<rSpans.m_lLoopCount; iGlobal++, iLocal++)
          {
            SmLoopData & rLoopData = prSmBrepData_ii->m_vLoops[iLocal];
            rLoopData.ReSet();

            // locals
            uint32_t sLoopType = (rArrays.m_sLoopEdgeuseCountArray[iGlobal] > 0) ? 0   // edgeuse loop
                                                                                 : 1 ; // vertex loop
                     
            // load Loop[iLocal] values
            rLoopData.m_lLoopType   = sLoopType;
            rLoopData.m_lNumEU      = rArrays.m_sLoopEdgeuseCountArray[iGlobal];

            // make unused values look like SMLib unused values (simplifies debugging)
            if(rLoopData.m_lFlags      == SM_UNDEF_ULONG) { rLoopData.m_lFlags      = 0 ; } 
            if(rLoopData.m_lUserIndex1 == SM_UNDEF_ULONG) { rLoopData.m_lUserIndex1 = 0 ; }
            if(rLoopData.m_lUserIndex2 == SM_UNDEF_ULONG) { rLoopData.m_lUserIndex2 = 0 ; } 
            switch (rLoopData.m_lLoopType)
            {
            case 0: // edgeuse loop
            { 
                rLoopData.m_lStartEU = lEdguseIndex;
                lEdguseIndex += rArrays.m_sLoopEdgeuseCountArray[iGlobal];
            }
            break;

            case 1: // vertex loop
            { 
                // UsdBrepArrayData should never store a no object value
                if (rArrays.m_sLoopVertexIndexArray[iGlobal] == USDBREP_NO_OBJECT_INDEX)
                {
                    TF_RUNTIME_ERROR(
                        "SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib: Found a LoopType=Vertex loop with no LoopVertex- needs debugging");
                }

                rLoopData.m_lVertex =
                    rArrays.MapGlobalToLocal(rSpans.m_lShellVertexCount + rArrays.m_sLoopVertexIndexArray[iGlobal],
                                             rSpans.m_lVertexStartIndex);

                // gwc: SmBrepData->m_vEdgeuses - does not contain a SmEUData for each LoopVertex
                //  // create one new LoopVertex-Edgeuse with LoopVertex-Edgeuse_VertexIndex = FaceEdge-EdgeuseCount +
                //  WireEdge_Count + LoopVertex_ii SmEUData & rEdgeuseData =
                //  prSmBrepData_ii->m_vEdgeuses[rSpans.m_lWireEdgeCount + rSpans.m_lEdgeuseCount];
                //  rEdgeuseData.ReSet() ;
                //  rEdgeuseData.m_lEUType = 1 ; // 1 = LoopVertex
                //  rEdgeuseData.m_lVertex = rLoopData.m_lVertex ;
            }
            break;

            default:
                SE_MSG(SM_ERR, _T("Incorrect LoopType"));
            }

            // accumulate topology and shape counts - done in LoopType branches

          } // end Loops

        // Edgeuses - build m_vEdgeuses = [ WireEdge-Edgeuses, FaceEdge-Edgeuses, no LoopVertex or ShellVertex Edgeuses ]
        //            only add Edgeuses for FaceEdges here.  Edgeuses for WireEdges are created in the Shell block
        //  notes: on iteration Edgeuse_ii, set Edgeuse_ii->NextRadialEU->m_lNextEU     not Edgeuse_ii->m_lNextEU
        //                                      Edgeuse_ii->NextRadialEU->m_lMateNextEU not Edgeuse_ii->m_lMateNextEU
        //       : Edgeuse_ii->m_lLoop is a back pointer.  Assume Loop order matches loop->edgeuse order.
        //                                                 Count Edgeuses against the rArrays.m_sLoopEdgeuseCountArray to get LoopIndex per EU
        uint32_t lLoopIndexForEdgeuse = 0 ;
        uint32_t lFaceIndexForEdgeuse = 0 ;
        // Use the first loop of THIS brep (at m_lLoopStartIndex), not global index 0
        SM_ASSERT_MSG(rArrays.m_sLoopEdgeuseCountArray.empty() || rSpans.m_lLoopStartIndex < rArrays.m_sLoopEdgeuseCountArray.size(), 
                      _T("BrepMove_OneUsdBrepToSMLibData: found a bad rSpans.m_lLoopStartIndex value - needs debugging"));
        uint32_t lEdgeuseStopForLoop  = (!rArrays.m_sLoopEdgeuseCountArray.empty() && rSpans.m_lLoopStartIndex < rArrays.m_sLoopEdgeuseCountArray.size() 
                                         ? rArrays.m_sLoopEdgeuseCountArray[rSpans.m_lLoopStartIndex] : 0) + rSpans.m_lWireEdgeCount ;
        // prSmBrepData_ii->m_vEdgeuses already sized in Shell block

        // Set Edge ReferenceArray size to max number of possible Edges (i.e., the number of Edgeuses)
        rSpans.m_iProcessedEdge_1stEdgeuses.assign(rSpans.m_lEdgeuseCount, SM_NO_OBJECT);
        for(iGlobal=rSpans.m_lEdgeuseStartIndex, iLocal=rSpans.m_lWireEdgeCount; iLocal<rSpans.m_lWireEdgeCount+rSpans.m_lEdgeuseCount; iGlobal++, iLocal++)
          {
            // update LoopIndex for this Edgeuse - LoopVertices use a LoopIndex number but don't add any Edgeuses to the Stop value
            while(iLocal >= lEdgeuseStopForLoop && lLoopIndexForEdgeuse + 1 < rSpans.m_lLoopCount)
              {
                lLoopIndexForEdgeuse++ ;
                uint32_t iGlobalLoopIndex = rArrays.MapLocalToGlobal(lLoopIndexForEdgeuse, rSpans.m_lLoopStartIndex);
                if (iGlobalLoopIndex < rArrays.m_sLoopEdgeuseCountArray.size())
                {
                    lEdgeuseStopForLoop += rArrays.m_sLoopEdgeuseCountArray[iGlobalLoopIndex] ;
                }
              }

            while(lFaceIndexForEdgeuse + 1 < rSpans.m_lFaceCount)
              {
                const SmFaceData & rFaceForEdgeuse = prSmBrepData_ii->m_vFaces[lFaceIndexForEdgeuse];
                if(lLoopIndexForEdgeuse < rFaceForEdgeuse.m_lStartLoop + rFaceForEdgeuse.m_lNumLoops)
                  { break ; }
                lFaceIndexForEdgeuse++ ;
              }

            // locals
            int32_t iGlobalNextRadialEUIndex = rArrays.m_sEdgeuseNextRadialEUIndexArray[iGlobal] ;
            
            // check state in debug mode: iGlobalNextRadialEUIndex is in bounds - else abort
            AERN_MSG((iGlobalNextRadialEUIndex >= 0) && 
                     (static_cast<size_t>(iGlobalNextRadialEUIndex) < rArrays.m_sEdgeuseThisRadialEntryTypeArray.size()),
                     SM_ERR,
                    _T("BrepMove_OneUsdBrepToSMLibData: iGlobalNextRadialEUIndex out of bounds - this is a bug"));
            
            int32_t    iLocalNextRadialEUIndex  = rArrays.MapGlobalToLocal(iGlobalNextRadialEUIndex, rSpans.m_lEdgeuseStartIndex ) + rSpans.m_lWireEdgeCount ;
            SmBoolean  bThisTopEntry            = (rArrays.m_sEdgeuseThisRadialEntryTypeArray[iGlobal] == UsdBrepSolidTokens->topEntry) ? TRUE : FALSE;
            SmBoolean  bNextTopEntry            = (rArrays.m_sEdgeuseThisRadialEntryTypeArray[iGlobalNextRadialEUIndex] == UsdBrepSolidTokens->topEntry) ? TRUE : FALSE;
            int32_t    iLocalNextRadialEUFlag   = (bNextTopEntry) ? iLocalNextRadialEUIndex : -iLocalNextRadialEUIndex - 1;
            SmBoolean  bOrientation             = (rArrays.m_sEdgeuseOrientationTypeArray[iGlobal] == UsdBrepSolidTokens->same) ? TRUE : FALSE;

            // EdgeuseData
            SmEUData & rEdgeuseData = prSmBrepData_ii->m_vEdgeuses[iLocal];

            // load Edgeuse[iLocal] values
            rEdgeuseData.m_lEUType      = 0;
            rEdgeuseData.m_lEdge        = rArrays.MapGlobalToLocal(rArrays.m_sEdgeuseEdgeIndexArray[iGlobal], rSpans.m_lEdgeStartIndex);
            rEdgeuseData.m_lLoop        = lLoopIndexForEdgeuse ;
            rEdgeuseData.m_bOrientation = bOrientation ;
            rEdgeuseData.m_bDownwardLU  = false ;

            // make unused values look like SMLib unused values (simplifies debugging)
            if(rEdgeuseData.m_lFlags      == SM_UNDEF_ULONG) { rEdgeuseData.m_lFlags      = 0 ; } 
            if(rEdgeuseData.m_lUserIndex1 == SM_UNDEF_ULONG) { rEdgeuseData.m_lUserIndex1 = 0 ; } 
            if(rEdgeuseData.m_lUserIndex2 == SM_UNDEF_ULONG) { rEdgeuseData.m_lUserIndex2 = 0 ; } 

            // map from {NextRadialIndex, RadialEntryType} to {NextRadialEUFlag, MateNextRadialEUFlag} for the edge's radial edgeuse list
            // if(RadialEntryType == UsdBrepSolidTokens->topEntry)    { From Top - to Bot - to NextRadialEUFlag }
            // if(RadialEntryType == UsdBrepSolidTokens->bottomEntry) { From Bot - to Top - to NextRadialEUFlag }
            //  where NextRadialEUFlag = Enter NextEdgeuse TopEdgeuse when Flag >= 0
            //                           Enter NextEdgeuse BotEdgeuse when Flag <  0
            //        NextRadialEUFlag = bNextTopEntry ? NextRadialEUIndex : -NextRadialEUIndex - 1 ; 
            if(bThisTopEntry) { // From Top - to Bot - to NextRadialEUFlag
                                /* TopEdgeuse */ rEdgeuseData.m_lNextEU     = -((int32_t)iLocal)-1 ; 
                                /* BotEdgeuse */ rEdgeuseData.m_lMateNextEU =  iLocalNextRadialEUFlag ;
                              }
            else              { // From Bot - to Top - to NextRadialEUFlag
                                /* TopEdgeuse */ rEdgeuseData.m_lNextEU     = iLocalNextRadialEUFlag ; 
                                /* BotEdgeuse */ rEdgeuseData.m_lMateNextEU = iLocal ;
                              }

            // Attach UV TrimCurves to edgeuses
            if (bHasUvCurves)
              {
                // Append this Edgeuse's Curve to prSmBrepData_ii->m_vUVCurves
                const uint32_t iGlobalFaceIndex = rArrays.MapLocalToGlobal(lFaceIndexForEdgeuse, rSpans.m_lFaceStartIndex);
                AERN_MSG(iGlobalFaceIndex < rArrays.m_sFaceSurfaceTypeArray.size(),
                         SM_ERR,
                         _T("BrepMove_OneUsdBrepToSMLibData: edgeuse owner face index is out of bounds"));
                SmBSplineCurve * pBSplineCurve = smu_CreateSmBSplineCurve2d_ForEdgeuseIndex(*prSmBrepData_ii->GetContext(), 
                                                                                            rArrays, 
                                                                                            rSpans, 
                                                                                            iGlobal,
                                                                                            rArrays.m_sFaceSurfaceTypeArray[iGlobalFaceIndex],
                                                                                            bOrientation);

                if (pBSplineCurve != NULL)
                  {
                    rEdgeuseData.m_lUVCurve = prSmBrepData_ii->m_vUVCurves.GetSize();
                    prSmBrepData_ii->m_vUVCurves.Add(pBSplineCurve);
                  }
                else
                  {
                    rEdgeuseData.m_lUVCurve = SM_NO_OBJECT;
                  }
              }
            else
              {
                rEdgeuseData.m_lUVCurve = SM_NO_OBJECT;
              }

            // accumulate topology data counts
            if( rSpans.m_iProcessedEdge_1stEdgeuses[ rEdgeuseData.m_lEdge ] == SM_NO_OBJECT)
              {
                // these will be future edge->PrimEU values.  The PrimaryEdgeuse is the first edgeuse in the radial list and the 2nd edgeuse must be that edgeuse's mate.
                //    for TopEntry EUs use the top Edgeuse index:[iLocal] 
                //    for BotEntry EUs use the bot Edgeuse index:[-ilocal-1]
                rSpans.m_iProcessedEdge_1stEdgeuses[ rEdgeuseData.m_lEdge ] = bThisTopEntry ? iLocal : -(int)iLocal - 1;
                // gwc_moved_count_accumulation_to_top_of_method // // rSpans.m_lEdgeCount++;
              }
          } // end iter every Edgeuse

        // Edges - // build m_vEdges = [FaceEdges, WireEdges]
        // ??? gwc::todo - what about attributes and materials for Edges and WireEdges
        
        // Set ReferenceArray sizes to max number of possible Edges and Vertices (i.e., twice the number of Edges and WireEdges)
        prSmBrepData_ii->m_vEdges.SetSize      (     (rSpans.m_lEdgeCount + rSpans.m_lWireEdgeCount)) ;
        rSpans.m_bProcessedVertices.assign( 2 * (rSpans.m_lEdgeCount + rSpans.m_lWireEdgeCount), false);
        for(iGlobal=rSpans.m_lEdgeStartIndex, iLocal=0; iLocal<rSpans.m_lEdgeCount; iGlobal++, iLocal++)
          {
            // locals
            SmEdgeData  & rEdgeData  = prSmBrepData_ii->m_vEdges[iLocal];

            // clear edge[iLocal] values
            rEdgeData.ReSet();

            // load edge[iLocal] values
            rEdgeData.m_sZoneTol3d    = sZoneTol3d;
            double dEdgeMin = rArrays.m_sEdgeRangeArray[2*iGlobal];
            double dEdgeMax = rArrays.m_sEdgeRangeArray[2*iGlobal+1];
            // USD stores circle/ellipse edge range in radians; SMLib SmCircle/SmEllipse use degrees
            if (rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dCircleAPI
             || rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dEllipseAPI)
              {
                dEdgeMin = UsdRadiansToSMLibDegrees(dEdgeMin);
                dEdgeMax = UsdRadiansToSMLibDegrees(dEdgeMax);
              }
            rEdgeData.m_vInterval.SetMinMax(dEdgeMin, dEdgeMax);
            rEdgeData.m_lCurve        = prSmBrepData_ii->m_v3DCurves.GetSize();
            rEdgeData.m_lStartVertex  = rArrays.MapGlobalToLocal(rSpans.m_lShellVertexCount + rArrays.m_sEdgeVertexIndicesArray[iGlobal][0], rSpans.m_lVertexStartIndex)  ; 
            rEdgeData.m_lEndVertex    = rArrays.MapGlobalToLocal(rSpans.m_lShellVertexCount + rArrays.m_sEdgeVertexIndicesArray[iGlobal][1], rSpans.m_lVertexStartIndex)  ;
            rEdgeData.m_lPrimEU       = rSpans.m_iProcessedEdge_1stEdgeuses[iLocal] ;

            // make unused values look like SMLib unused values (simplifies debugging)
            if(rEdgeData.m_lFlags      == SM_UNDEF_ULONG) { rEdgeData.m_lFlags      = 0 ; } 
            if(rEdgeData.m_lUserIndex1 == SM_UNDEF_ULONG) { rEdgeData.m_lUserIndex1 = 0 ; }
            if(rEdgeData.m_lUserIndex2 == SM_UNDEF_ULONG) { rEdgeData.m_lUserIndex2 = 0 ; } 

            // Append this Edge's Curve to prSmBrepData_ii->m_v3DCurves
            if (rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dNurbAPI)
              { 
                SmBSplineCurve * pBSplineCurve = smu_CreateSmBSplineCurve3d_ForEdgeIndex(*prSmBrepData_ii->GetContext(), 
                                                                                         rArrays, 
                                                                                         rSpans, 
                                                                                         iGlobal);
                if (pBSplineCurve == NULL)
                  {
                    // inform the public - aborting this Brep to process subsequent Breps
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: smu_CreateSmBSplineCurve3d_ForEdgeIndex %u - returned NULL"), iGlobal);
                    ERR_MSG(sMessage);
                    return(SM_ERR);
                  }

                prSmBrepData_ii->m_v3DCurves.Add(pBSplineCurve);
              }
            else if (rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dCircleAPI)
              {
                uint32_t idx = rSpans.m_lEdgeCircleCurve3d_StartIndex ;
                GfVec3d sCenter = rArrays.m_sEdge_CurveCircle_CenterArray[idx] ;
                GfVec3d sAxis   = rArrays.m_sEdge_CurveCircle_AxisArray[idx] ;
                GfVec3d sRefDir = rArrays.m_sEdge_CurveCircle_RefDirectionArray[idx] ;
                double  dRadius = rArrays.m_sEdge_CurveCircle_RadiusArray[idx] ;

                SmVector3d sXAxis(sRefDir[0], sRefDir[1], sRefDir[2]) ;
                SmVector3d sZAxis(sAxis[0], sAxis[1], sAxis[2]) ;
                SmVector3d sYAxis = sZAxis * sXAxis ;
                SmAxis2Placement sPlacement ;
                sPlacement.SetCanonical(SmPoint3d(sCenter[0], sCenter[1], sCenter[2]), sXAxis, sYAxis) ;

                SmCircle * pCircle = NULL ;
                SmExtent1d sEdgeDegreeInterval(dEdgeMin, dEdgeMax);
                SmStatus stat = SmCircle::CreateCanonical(*prSmBrepData_ii->GetContext(), sPlacement, dRadius, pCircle, &sEdgeDegreeInterval) ;
                SmObjDelete sCircleCleanup(pCircle);
                if (stat != SM_SUCCESS || pCircle == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmCircle::CreateCanonical failed for edge index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }
                pCircle->EditParameterization(sEdgeDegreeInterval, FALSE);
                prSmBrepData_ii->m_v3DCurves.Add(pCircle);
                sCircleCleanup.Clear();
                rSpans.m_lEdgeCircleCurve3d_StartIndex++ ;
              }
            else if (rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dLineAPI)
              {
                uint32_t idx = rSpans.m_lEdgeLineCurve3d_StartIndex ;
                GfVec3d sOriginPt  = rArrays.m_sEdge_CurveLine_OriginArray[idx] ;
                GfVec3d sDirection = rArrays.m_sEdge_CurveLine_DirectionArray[idx] ;

                SmLine * pLine = NULL ;
                SmStatus stat = SmLine::CreateCanonical(*prSmBrepData_ii->GetContext(),
                    SmPoint3d(sOriginPt[0], sOriginPt[1], sOriginPt[2]),
                    SmVector3d(sDirection[0], sDirection[1], sDirection[2]),
                    pLine) ;
                SmObjDelete sLineCleanup(pLine);
                if (stat != SM_SUCCESS || pLine == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmLine::CreateCanonical failed for edge index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }
                prSmBrepData_ii->m_v3DCurves.Add(pLine);
                sLineCleanup.Clear();
                rSpans.m_lEdgeLineCurve3d_StartIndex++ ;
              }
            else if (rArrays.m_sEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dEllipseAPI)
              {
                uint32_t idx = rSpans.m_lEdgeEllipseCurve3d_StartIndex ;
                GfVec3d sCenter = rArrays.m_sEdge_CurveEllipse_CenterArray[idx] ;
                GfVec3d sAxis   = rArrays.m_sEdge_CurveEllipse_AxisArray[idx] ;
                GfVec3d sRefDir = rArrays.m_sEdge_CurveEllipse_RefDirectionArray[idx] ;
                double  dXRadius = rArrays.m_sEdge_CurveEllipse_XRadiusArray[idx] ;
                double  dYRadius = rArrays.m_sEdge_CurveEllipse_YRadiusArray[idx] ;

                SmVector3d sXAxis(sRefDir[0], sRefDir[1], sRefDir[2]) ;
                SmVector3d sZAxis(sAxis[0], sAxis[1], sAxis[2]) ;
                SmVector3d sYAxis = sZAxis * sXAxis ;
                SmAxis2Placement sPlacement ;
                sPlacement.SetCanonical(SmPoint3d(sCenter[0], sCenter[1], sCenter[2]), sXAxis, sYAxis) ;

                SmEllipse * pEllipse = NULL ;
                SmExtent1d sEllipseDegreeInterval(dEdgeMin, dEdgeMax);
                SmStatus stat = SmEllipse::CreateCanonical(*prSmBrepData_ii->GetContext(), sPlacement, dXRadius, dYRadius, pEllipse, &sEllipseDegreeInterval) ;
                SmObjDelete sEllipseCleanup(pEllipse);
                if (stat != SM_SUCCESS || pEllipse == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmEllipse::CreateCanonical failed for edge index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }
                pEllipse->EditParameterization(sEllipseDegreeInterval, FALSE);
                prSmBrepData_ii->m_v3DCurves.Add(pEllipse);
                sEllipseCleanup.Clear();
                rSpans.m_lEdgeEllipseCurve3d_StartIndex++ ;
              }

            // accumulate topology data counts
            if(rSpans.m_bProcessedVertices[ rEdgeData.m_lStartVertex ] == false)
              {
                rSpans.m_bProcessedVertices[rEdgeData.m_lStartVertex ] = true ;
                // gwc_moved_count_accumulation_to_top_of_method // rSpans.m_lVertexCount++;
              }
            if(rSpans.m_bProcessedVertices[rEdgeData.m_lEndVertex ] == false)
              {
                rSpans.m_bProcessedVertices[rEdgeData.m_lEndVertex ] = true ;
                // gwc_moved_count_accumulation_to_top_of_method // rSpans.m_lVertexCount++;
              }
          } // end iter every edge

        // WireEdges - set m_vEdges with WireEdge data.                                 // build m_vEdges    = [FaceEdges, WireEdges]
        // ??? gwc::todo - what about attributes and materials for Edges and WireEdges  // build m_vEdgeuses = WireEdge_Edgeuses, [FaceEdge_Edgeuses]
        for(iGlobal=rSpans.m_lWireEdgeStartIndex, iLocal=rSpans.m_lEdgeCount; iLocal<rSpans.m_lEdgeCount+rSpans.m_lWireEdgeCount; iGlobal++, iLocal++)
          {
            SmEdgeData & rWireEdgeData = prSmBrepData_ii->m_vEdges[iLocal];

            // clear wireEdge[iLocal] values
            rWireEdgeData.ReSet();

            // load wireEdge[iLocal] values
            rWireEdgeData.m_sZoneTol3d    = sZoneTol3d;
            double dWireMin = rArrays.m_sWireEdgeRangeArray[2*iGlobal];
            double dWireMax = rArrays.m_sWireEdgeRangeArray[2*iGlobal+1];
            // USD stores circle/ellipse wire edge range in radians; SMLib SmCircle/SmEllipse use degrees
            if (rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dCircleAPI
             || rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dEllipseAPI)
              {
                dWireMin = UsdRadiansToSMLibDegrees(dWireMin);
                dWireMax = UsdRadiansToSMLibDegrees(dWireMax);
              }
            rWireEdgeData.m_vInterval.SetMinMax(dWireMin, dWireMax);
            rWireEdgeData.m_lCurve        = prSmBrepData_ii->m_v3DCurves.GetSize();
            rWireEdgeData.m_lStartVertex  = rArrays.MapGlobalToLocal(rSpans.m_lShellVertexCount + rArrays.m_sWireEdgeVertexIndicesArray[iGlobal][0], rSpans.m_lVertexStartIndex)  ; 
            rWireEdgeData.m_lEndVertex    = rArrays.MapGlobalToLocal(rSpans.m_lShellVertexCount + rArrays.m_sWireEdgeVertexIndicesArray[iGlobal][1], rSpans.m_lVertexStartIndex)  ; 
            rWireEdgeData.m_lPrimEU       = iLocal - rSpans.m_lEdgeCount;     // iLocal = WireEdge index, need = WireEdge_Edgeuse index = WireEdge_index - EdgeCount.

            // make unused values look like SMLib unused values (simplifies debugging)
            if(rWireEdgeData.m_lFlags      == SM_UNDEF_ULONG) { rWireEdgeData.m_lFlags      = 0 ; } 
            if(rWireEdgeData.m_lUserIndex1 == SM_UNDEF_ULONG) { rWireEdgeData.m_lUserIndex1 = 0 ; }
            if(rWireEdgeData.m_lUserIndex2 == SM_UNDEF_ULONG) { rWireEdgeData.m_lUserIndex2 = 0 ; } 

            // Append this WireEdge's Curve to prSmBrepData_ii->m_v3DCurves
            if (rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dNurbAPI)
              { 
                SmBSplineCurve *pBSplineCurve = smu_CreateSmBSplineCurve3d_ForWireEdgeIndex(*prSmBrepData_ii->GetContext(), 
                                                                                           rArrays, 
                                                                                           rSpans, 
                                                                                           iGlobal);
                if (pBSplineCurve == NULL)
                  {
                    // inform the public - aborting this Brep to process subsequent Breps
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: smu_CreateSmBSplineCurve3d_ForWireEdgeIndex %u - returned NULL"), iGlobal);
                    ERR_MSG(sMessage);
                    return(SM_ERR);
                  }

                prSmBrepData_ii->m_v3DCurves.Add(pBSplineCurve);
              }
            else if (rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dCircleAPI)
              {
                uint32_t idx = rSpans.m_lWireEdgeCircleCurve3d_StartIndex ;
                GfVec3d sCenter = rArrays.m_sWireEdge_CurveCircle_CenterArray[idx] ;
                GfVec3d sAxis   = rArrays.m_sWireEdge_CurveCircle_AxisArray[idx] ;
                GfVec3d sRefDir = rArrays.m_sWireEdge_CurveCircle_RefDirectionArray[idx] ;
                double  dRadius = rArrays.m_sWireEdge_CurveCircle_RadiusArray[idx] ;

                SmVector3d sXAxis(sRefDir[0], sRefDir[1], sRefDir[2]) ;
                SmVector3d sZAxis(sAxis[0], sAxis[1], sAxis[2]) ;
                SmVector3d sYAxis = sZAxis * sXAxis ;
                SmAxis2Placement sPlacement ;
                sPlacement.SetCanonical(SmPoint3d(sCenter[0], sCenter[1], sCenter[2]), sXAxis, sYAxis) ;

                SmCircle * pCircle = NULL ;
                SmExtent1d sWireDegreeInterval(dWireMin, dWireMax);
                SmStatus stat = SmCircle::CreateCanonical(*prSmBrepData_ii->GetContext(), sPlacement, dRadius, pCircle, &sWireDegreeInterval) ;
                SmObjDelete sCircleCleanup(pCircle);
                if (stat != SM_SUCCESS || pCircle == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmCircle::CreateCanonical failed for wire edge index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }
                pCircle->EditParameterization(sWireDegreeInterval, FALSE);
                prSmBrepData_ii->m_v3DCurves.Add(pCircle);
                sCircleCleanup.Clear();
                rSpans.m_lWireEdgeCircleCurve3d_StartIndex++ ;
              }
            else if (rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dLineAPI)
              {
                uint32_t idx = rSpans.m_lWireEdgeLineCurve3d_StartIndex ;
                GfVec3d sOriginPt  = rArrays.m_sWireEdge_CurveLine_OriginArray[idx] ;
                GfVec3d sDirection = rArrays.m_sWireEdge_CurveLine_DirectionArray[idx] ;

                SmLine * pLine = NULL ;
                SmStatus stat = SmLine::CreateCanonical(*prSmBrepData_ii->GetContext(),
                    SmPoint3d(sOriginPt[0], sOriginPt[1], sOriginPt[2]),
                    SmVector3d(sDirection[0], sDirection[1], sDirection[2]),
                    pLine) ;
                SmObjDelete sLineCleanup(pLine);
                if (stat != SM_SUCCESS || pLine == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmLine::CreateCanonical failed for wire edge index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }
                prSmBrepData_ii->m_v3DCurves.Add(pLine);
                sLineCleanup.Clear();
                rSpans.m_lWireEdgeLineCurve3d_StartIndex++ ;
              }
            else if (rArrays.m_sWireEdgeCurveTypeArray[iGlobal] == UsdBrepCurveTokens->brepCurve3dEllipseAPI)
              {
                uint32_t idx = rSpans.m_lWireEdgeEllipseCurve3d_StartIndex ;
                GfVec3d sCenter = rArrays.m_sWireEdge_CurveEllipse_CenterArray[idx] ;
                GfVec3d sAxis   = rArrays.m_sWireEdge_CurveEllipse_AxisArray[idx] ;
                GfVec3d sRefDir = rArrays.m_sWireEdge_CurveEllipse_RefDirectionArray[idx] ;
                double  dXRadius = rArrays.m_sWireEdge_CurveEllipse_XRadiusArray[idx] ;
                double  dYRadius = rArrays.m_sWireEdge_CurveEllipse_YRadiusArray[idx] ;

                SmVector3d sXAxis(sRefDir[0], sRefDir[1], sRefDir[2]) ;
                SmVector3d sZAxis(sAxis[0], sAxis[1], sAxis[2]) ;
                SmVector3d sYAxis = sZAxis * sXAxis ;
                SmAxis2Placement sPlacement ;
                sPlacement.SetCanonical(SmPoint3d(sCenter[0], sCenter[1], sCenter[2]), sXAxis, sYAxis) ;

                SmEllipse * pEllipse = NULL ;
                SmExtent1d sWireEllipseDegreeInterval(dWireMin, dWireMax);
                SmStatus stat = SmEllipse::CreateCanonical(*prSmBrepData_ii->GetContext(), sPlacement, dXRadius, dYRadius, pEllipse, &sWireEllipseDegreeInterval) ;
                SmObjDelete sEllipseCleanup(pEllipse);
                if (stat != SM_SUCCESS || pEllipse == NULL)
                  {
                    TCHAR sMessage[SM_TBLOCK_SIZE];
                    SM_SPRINTF( sMessage, _T("BrepMove_OneUsdBrepToSMLibData: SmEllipse::CreateCanonical failed for wire edge index %u"), iGlobal);
                    ERR_MSG(sMessage);
                    prSmBrepData_ii = NULL;
                    return (stat != SM_SUCCESS) ? stat : SM_ERR;
                  }
                pEllipse->EditParameterization(sWireEllipseDegreeInterval, FALSE);
                prSmBrepData_ii->m_v3DCurves.Add(pEllipse);
                sEllipseCleanup.Clear();
                rSpans.m_lWireEdgeEllipseCurve3d_StartIndex++ ;
              }

            // accumulate topology data counts
            if(rSpans.m_bProcessedVertices[ rWireEdgeData.m_lStartVertex ] == false)
              {
                rSpans.m_bProcessedVertices[rWireEdgeData.m_lStartVertex ] = true ;
                // gwc_moved_count_accumulation_to_top_of_method // rSpans.m_lVertexCount++;
              }
            if(rSpans.m_bProcessedVertices[rWireEdgeData.m_lEndVertex ] == false)
              {
                rSpans.m_bProcessedVertices[rWireEdgeData.m_lEndVertex ] = true ;
                // gwc_moved_count_accumulation_to_top_of_method // rSpans.m_lVertexCount++;
              }

          } // end iter every wireEdge

        // EdgeVertices - set m_vVertices with [ShellVertices, mixed{EdgeVertex, WireEdge, LoopVertex}] data.
        // ??? gwc::todo - what about attributes and materials for Vertices
        // m_vVertices sized in Shell block
        SM_ASSERT_MSG(prSmBrepData_ii->m_vVertices.GetSize() == rSpans.m_lShellVertexCount + rSpans.m_lVertexCount, _T("BrepMove_OneUsdBrepToSMLib: error, m_vVertices Size != ShellVertexCount + VertexCount")) ;
        for(iGlobal=rSpans.m_lVertexStartIndex, iLocal=rSpans.m_lShellVertexCount; iLocal<rSpans.m_lShellVertexCount+rSpans.m_lVertexCount; iGlobal++, iLocal++)
          {
            SmVertexData & rVertexData = prSmBrepData_ii->m_vVertices[iLocal];
            rVertexData.ReSet();

            // load vertex[iLocal] values
            rVertexData.m_sZoneTol3d  = sZoneTol3d;

            // make unused values look like SMLib unused values (simplifies debugging)
            if(rVertexData.m_lFlags      == SM_UNDEF_ULONG) { rVertexData.m_lFlags      = 0 ; } 
            if(rVertexData.m_lUserIndex1 == SM_UNDEF_ULONG) { rVertexData.m_lUserIndex1 = 0 ; }
            if(rVertexData.m_lUserIndex2 == SM_UNDEF_ULONG) { rVertexData.m_lUserIndex2 = 0 ; } 

            if(rArrays.m_sVertexPointTypeArray[iGlobal] == UsdBrepSolidTokens->brepPointAPI)
              {
                  rVertexData.m_vPoint.Set(rArrays.m_sVertex_PointPositionArray[iGlobal][0],
                                           rArrays.m_sVertex_PointPositionArray[iGlobal][1],
                                           rArrays.m_sVertex_PointPositionArray[iGlobal][2]) ;
              }

            // gwc_moved_count_accumulation_to_top_of_method                                    

          } // end iter every EdgeVertex

        // ??? gwc::todo - what about attributes and materials for ShellVertices

      } // end scope: populating prSmBrepData_ii with topologyData and SmShape objs

#ifdef SM_DEBUG_CODE
    if (bDebugMe)
      {         
          // write pBrepData_ii to file
          if (usdBrep_CreateOutputFiles())              
          {
            std::string sPathString = TfStringPrintf("OutputFiles/MoveOneBrepArrayDataItem_ToSmBrepData__OutputBrepData%d.smb", iBrepIndex);

            // TRUE = open file and rewrite contents, FALSE= open file and append to end
            prSmBrepData_ii->WriteToFile(sPathString, SM_ASCII, TRUE);

            usdBrep_WriteString("\n\n  FILE_WRITE: " + sPathString + "\n");
          }
      }
#endif // SM_DEBUG_CODE

#undef MAP_USD_TO_SM_UNUSED_VALUE

    // all done - save the output Brep and return
    sClean.Clear();
    sCleanAttributes.Clear();
    sResetOutputOnFailure.Clear();
    return SM_SUCCESS;

} // end SMU_BrepConvert::BrepMove_OneUsdBrepToSMLibData

/*******************************************************************//**
PURPOSE: Create an SMLib SmBrep of a capsule from a USD UsdGeomCapsule

NOTES:  This method treats the UsdGeomCapsule as a solid, creating the Brep
    equivalent. There is no means to adjust the parameterization.

    NB that a capsule is a capped cylinder, where the caps are hemispheres
***********************************************************************/
SMU_EXPORT SmStatus CreateSmBrep_FromUsdCapsule
(
    const SmContext            & crSmContext,  // in : Context in which prSmBrep is created       
    const pxr::UsdGeomCapsule  & crUsdCapsule, // in : Usd prim being converted to an SmBrep      
          SmBrep              *& prSmBrep      // out: The newly created SmBrep                   
)
{

    // Locals - initialize with USD default values
    double dHeight = 1.0;  // USD default height for capsule
    double dRadius = 0.5;  // USD default radius for capsule
    SmVector3d sOrigin, sXAxis, sYAxis;
    SmAxis2Placement sPlacement; 
    TfToken tAxis = UsdGeomTokens->y; // USD default axis is Y

    // UsdGeomCapsule Attributes
    crUsdCapsule.GetHeightAttr().Get(&dHeight);
    crUsdCapsule.GetRadiusAttr().Get(&dRadius);
    crUsdCapsule.GetAxisAttr().Get(&tAxis);

    // Set sXAxis & sYAxis vectors from tAxis (spine) token
    if (tAxis == UsdGeomTokens->x)
    {
        sOrigin.Set(-dHeight / 2., 0., 0.);
        sXAxis.Set(0., 1., 0.);
        sYAxis.Set(0., 0., 1.);
    }
    else if (tAxis == UsdGeomTokens->y)
    {
        // Default USD axis - Y points up
        sOrigin.Set(0., -dHeight / 2., 0.);
        sXAxis.Set(0., 0., 1.);
        sYAxis.Set(1., 0., 0.);
    }
    else if (tAxis == UsdGeomTokens->z)
    {
        sOrigin.Set(0., 0., -dHeight / 2.);
        sXAxis.Set(1., 0., 0.);
        sYAxis.Set(0., 1., 0.);
    }
    else
    {
        // Unknown axis - default to Y axis behavior
        sOrigin.Set(0., -dHeight / 2., 0.);
        sXAxis.Set(0., 0., 1.);
        sYAxis.Set(1., 0., 0.);
    }

    sPlacement.SetCanonical(sOrigin, sXAxis, sYAxis);

    // Create the SmBrep
    prSmBrep = new (crSmContext) SmBrep();

    // Create the cylinder
    SmCylinder* pCylinder;
    SmCylinder::CreateCanonical(crSmContext, sPlacement, dRadius, pCylinder);

    // Create the first cap
    SmExtent2d sSphereDomain0(0., -90., 360., 0.);
    SmSphere* pSphere0 = new (crSmContext) SmSphere(sOrigin, sXAxis, sYAxis, sSphereDomain0, dRadius);

    // Create the second cap
    SmExtent2d sSphereDomain1(0., 0., 360., 90.);
    SmSphere* pSphere1 = new (crSmContext) SmSphere(-sOrigin, sXAxis, sYAxis, sSphereDomain0, dRadius);

    // Make faces from the surfaces
    SmFace* pNewFace;
    prSmBrep->CreateFaceFromSurface(pCylinder, pCylinder->GetNaturalUVDomain(), pNewFace);
    prSmBrep->CreateFaceFromSurface(pSphere0, pSphere0->GetNaturalUVDomain(), pNewFace);
    prSmBrep->CreateFaceFromSurface(pSphere1, pSphere1->GetNaturalUVDomain(), pNewFace);

    // Make manifold
    prSmBrep->StitchAndOrient();

    // Apply local XFormOps
    bool resetXformStack;
    std::vector<UsdGeomXformOp> xOrderedXFormOpsAttr = crUsdCapsule.GetOrderedXformOps(&resetXformStack);
    for (std::vector<UsdGeomXformOp>::iterator iter = xOrderedXFormOpsAttr.begin();
         iter != xOrderedXFormOpsAttr.end(); ++iter)
    {
        SMU_BrepConvert::ApplyUsdXformOp(*prSmBrep, *iter);
    }

    return SM_SUCCESS;
} // end SMU_BrepConvert::CreateSmBrep_FromUsdCapsule

/*******************************************************************//**
PURPOSE: Create an SMLib SmBrep of a capped cone from a USD UsdGeomCone

NOTES:  This method treats the UsdGeomCone as a solid, creating the Brep
    equivalent. There is no means to adjust the parameterization.
***********************************************************************/
SMU_EXPORT SmStatus CreateSmBrep_FromUsdCone
(
    const SmContext         & crSmContext, // in : Context in which prSmBrep is created       
    const pxr::UsdGeomCone  & crUsdCone,   // in : Usd prim being converted to an SmBrep      
          SmBrep           *& prSmBrep     // out: The newly created SmBrep                   
)
{

    // Locals - initialize with USD default values
    double dHeight = 2.0;  // USD default height
    double dRadius = 1.0;  // USD default radius
    SmVector3d sXAxis, sYAxis;
    SmAxis2Placement sOrigin; 
    TfToken tAxis = UsdGeomTokens->y; // USD default axis is Y

    // UsdGeomCone Attributes
    crUsdCone.GetHeightAttr().Get(&dHeight);
    crUsdCone.GetRadiusAttr().Get(&dRadius);
    crUsdCone.GetAxisAttr().Get(&tAxis);

    // Set sXAxis & sYAxis vectors from tAxis (spine) token
    // Note: The cone's axis runs from base to apex. The placement origin is at the base.
    // USD Cone: centered at origin, base at -height/2, apex at +height/2 along axis
    if (tAxis == UsdGeomTokens->x)
    {
        sXAxis.Set(0., 1., 0.);
        sYAxis.Set(0., 0., 1.);
        sOrigin.SetCanonical(SmVector3d(-dHeight/2., 0, 0), sXAxis, sYAxis);
    }
    else if (tAxis == UsdGeomTokens->y)
    {
        // Default USD axis - Y points up
        sXAxis.Set(0., 0., 1.);
        sYAxis.Set(1., 0., 0.);
        sOrigin.SetCanonical(SmVector3d(0, -dHeight / 2., 0), sXAxis, sYAxis);
    }
    else if (tAxis == UsdGeomTokens->z)
    {
        sXAxis.Set(1., 0., 0.);
        sYAxis.Set(0., 1., 0.);
        sOrigin.SetCanonical(SmVector3d(0, 0, -dHeight / 2.), sXAxis, sYAxis);
    }
    else
    {
        // Unknown axis - default to Y axis behavior
        sXAxis.Set(0., 0., 1.);
        sYAxis.Set(1., 0., 0.);
        sOrigin.SetCanonical(SmVector3d(0, -dHeight / 2., 0), sXAxis, sYAxis);
    }

    // Create the SmBrep
    prSmBrep = new (crSmContext) SmBrep();
    SmPrimitiveCreation sPC(prSmBrep->GetInfiniteRegion());
    sPC.CreateCone(dHeight, dRadius, 0., 0., 360., sOrigin);

    // Apply local XFormOps
    bool resetXformStack;
    std::vector<UsdGeomXformOp> xOrderedXFormOpsAttr = crUsdCone.GetOrderedXformOps(&resetXformStack);
    for (std::vector<UsdGeomXformOp>::iterator iter = xOrderedXFormOpsAttr.begin();
         iter != xOrderedXFormOpsAttr.end(); ++iter)
    {
        SMU_BrepConvert::ApplyUsdXformOp(*prSmBrep, *iter);
    }

    return SM_SUCCESS;
} // end SMU_BrepConvert::CreateSmBrep_FromUsdCone

/*******************************************************************//**
PURPOSE: Create an SMLib SmBrep of a cube  from a USD UsdGeomCube

NOTES:  This method treats the UsdGeomCube as a solid, creating the Brep
    equivalent. There is no means to adjust the parameterization.
***********************************************************************/
SMU_EXPORT SmStatus CreateSmBrep_FromUsdCube
(
    const SmContext         & crSmContext, // in : Context in which prSmBrep is created       
    const pxr::UsdGeomCube  & crUsdCube,   // in : Usd prim being converted to an SmBrep      
          SmBrep           *& prSmBrep     // out: The newly created SmBrep                   
)
{

    // Locals - initialize with USD default value
    double dSideLength = 2.0;  // USD default size

    // UsdGeomCube Attributes
    crUsdCube.GetSizeAttr().Get(&dSideLength);

    // Create the SmBrep
    prSmBrep = new (crSmContext) SmBrep();
    SmPrimitiveCreation sPC(prSmBrep->GetInfiniteRegion());
    double dPosition = - dSideLength / 2.;
    SmAxis2Placement sOrigin(dPosition, dPosition, dPosition, // origin
                              1., 0., 0.,                     // xAxis
                              0., 1., 0.);                    // yAxis

    sPC.CreateBox(dSideLength, dSideLength, dSideLength, sOrigin);

    // Apply local XFormOps
    bool resetXformStack;
    std::vector<UsdGeomXformOp> xOrderedXFormOpsAttr = crUsdCube.GetOrderedXformOps(&resetXformStack);
    for (std::vector<UsdGeomXformOp>::iterator iter = xOrderedXFormOpsAttr.begin();
         iter != xOrderedXFormOpsAttr.end(); ++iter)
    {
        SMU_BrepConvert::ApplyUsdXformOp(*prSmBrep, *iter);
    }

    return SM_SUCCESS;
} // end SMU_BrepConvert::CreateSmBrep_FromUsdCube
        
/*******************************************************************//**
PURPOSE: Create an SMLib SmBrep of a capped cylinder from a USD UsdGeomCylinder

NOTES:  This method treats the UsdGeomCylinder as a solid, creating the Brep
    equivalent. There is no means to adjust the parameterization.
***********************************************************************/
SMU_EXPORT SmStatus CreateSmBrep_FromUsdCylinder
(
    const SmContext             & crSmContext,   // in : Context in which prSmBrep is created       
    const pxr::UsdGeomCylinder  & crUsdCylinder, // in : Usd prim being converted to an SmBrep      
          SmBrep               *& prSmBrep       // out: The newly created SmBrep                   
)
{

    // Locals - initialize with USD default values
    double dHeight = 2.0;  // USD default height
    double dRadius = 1.0;  // USD default radius
    SmVector3d sXAxis, sYAxis;
    SmAxis2Placement sOrigin;
    TfToken tAxis = UsdGeomTokens->y; // USD default axis is Y

    // UsdGeomCylinder Attributes
    crUsdCylinder.GetHeightAttr().Get(&dHeight);
    crUsdCylinder.GetRadiusAttr().Get(&dRadius);
    crUsdCylinder.GetAxisAttr().Get(&tAxis);

    // Set sXAxis & sYAxis vectors from tAxis (spine) token
    if (tAxis == UsdGeomTokens->x)
    {
        sXAxis.Set(0., 1., 0.);
        sYAxis.Set(0., 0., 1.);
        sOrigin.SetCanonical(SmVector3d(-dHeight/2., 0, 0), sXAxis, sYAxis);
    }
    else if (tAxis == UsdGeomTokens->y)
    {
        // Default USD axis - Y points up
        sXAxis.Set(0., 0., 1.);
        sYAxis.Set(1., 0., 0.);
        sOrigin.SetCanonical(SmVector3d(0, -dHeight / 2., 0), sXAxis, sYAxis);
    }
    else if (tAxis == UsdGeomTokens->z)
    {
        sXAxis.Set(1., 0., 0.);
        sYAxis.Set(0., 1., 0.);
        sOrigin.SetCanonical(SmVector3d(0, 0, -dHeight / 2.), sXAxis, sYAxis);
    }
    else
    {
        // Unknown axis - default to Y axis behavior
        sXAxis.Set(0., 0., 1.);
        sYAxis.Set(1., 0., 0.);
        sOrigin.SetCanonical(SmVector3d(0, -dHeight / 2., 0), sXAxis, sYAxis);
    }

    // Create the SmBrep
    prSmBrep = new (crSmContext) SmBrep();
    SmPrimitiveCreation sPC(prSmBrep->GetInfiniteRegion());
    sPC.CreateCone(dHeight, dRadius, dRadius, 0., 360., sOrigin);

    // Apply local XFormOps
    bool resetXformStack;
    std::vector<UsdGeomXformOp> xOrderedXFormOpsAttr = crUsdCylinder.GetOrderedXformOps(&resetXformStack);
    for (std::vector<UsdGeomXformOp>::iterator iter = xOrderedXFormOpsAttr.begin();
         iter != xOrderedXFormOpsAttr.end(); ++iter)
    {
        SMU_BrepConvert::ApplyUsdXformOp(*prSmBrep, *iter);
    }

    return SM_SUCCESS;
} // end SMU_BrepConvert::CreateSmBrep_FromUsdCylinder

/*******************************************************************//**
PURPOSE: Create an SMLib SmBrep of a sphere from a USD UsdGeomSphere

NOTES:  This method treats the UsdGeomSphere as a solid, creating the Brep
    equivalent. There is no means to adjust the parameterization.
***********************************************************************/
SMU_EXPORT SmStatus CreateSmBrep_FromUsdSphere
(
    const SmContext           & crSmContext, // in : Context in which prSmBrep is created       
    const pxr::UsdGeomSphere  & crUsdSphere, // in : Usd prim being converted to an SmBrep      
          SmBrep             *& prSmBrep     // out: The newly created SmBrep                   
)
{

    // Locals - initialize with USD default value
    double dRadius = 1.0;  // USD default radius

    // UsdGeomSphere attributes
    crUsdSphere.GetRadiusAttr().Get(&dRadius);

    // Create the SmBrep
    prSmBrep = new (crSmContext) SmBrep();
    SmPrimitiveCreation sPC(prSmBrep->GetInfiniteRegion());
    sPC.CreateSphere(dRadius, 0., 360., SmAxis2Placement());

    // Apply local XFormOps
    bool resetXformStack;
    std::vector<UsdGeomXformOp> xOrderedXFormOpsAttr = crUsdSphere.GetOrderedXformOps(&resetXformStack);
    for (std::vector<UsdGeomXformOp>::iterator iter = xOrderedXFormOpsAttr.begin();
         iter != xOrderedXFormOpsAttr.end(); ++iter)
    {
        SMU_BrepConvert::ApplyUsdXformOp(*prSmBrep, *iter);
    }

    return SM_SUCCESS;
} // end SMU_BrepConvert::CreateSmBrep_FromUsdSphere

/*******************************************************************//**
PURPOSE: Convert a UsdGeomMesh into an SmPolyBrep

NOTES:
***********************************************************************/
SMU_EXPORT SmStatus CreateSmPolyBrep_FromUsdMesh
(
    const SmContext         & crSmContext,   // in : Context in which prSmPolyBrep is created       
    const pxr::UsdGeomMesh  & crUsdGeomMesh, // in : Usd prim being converted to an SmPolyBrep  
          SmPolyBrep       *& prSmPolyBrep   // out: The newly created SmPolyBrep               

)
{
    // Locals
    SmZoneTol3d sZoneTol = SmTol::GetZoneTol3d(&crSmContext);
    SmPolyBrep* pPolyBrep = new (crSmContext) SmPolyBrep(sZoneTol);

    // Locals
    SmPoint3d sStartVertex, sEndVertex;
    VtArray<GfVec3f> s3dPoints;
    VtIntArray sFaceVertexCounts, sFaceVertexIndices;
    SmTArray<SmPolyVertex*> sPolyVertices;

    // Get UsdGeomMesh properties
    crUsdGeomMesh.GetPointsAttr().Get(&s3dPoints);
    crUsdGeomMesh.GetFaceVertexCountsAttr().Get(&sFaceVertexCounts);
    crUsdGeomMesh.GetFaceVertexIndicesAttr().Get(&sFaceVertexIndices);

    // Convert 3dPoints to SmPolyVertex
    size_t iNumPoints = s3dPoints.size(); 
    sPolyVertices.SetSize((uint32_t) iNumPoints);
    uint32_t ii = 0;
    for (VtArray<GfVec3f>::iterator p3dPoint = s3dPoints.begin(); p3dPoint < s3dPoints.end(); ++p3dPoint, ++ii)
    {
        SmPoint3d sPoint((*p3dPoint)[0], (*p3dPoint)[1], (*p3dPoint)[2]);
        SmPolyVertex* pPolyVertex = new (pPolyBrep) SmPolyVertex(sPoint, sZoneTol);
        sPolyVertices[ii] = pPolyVertex;
    }

    ii = 0;
    for (VtIntArray::iterator pFaceVertexCount = sFaceVertexCounts.begin(); pFaceVertexCount < sFaceVertexCounts.end(); ++pFaceVertexCount)
    {
        SmPolyFace* pPolyFace = new (pPolyBrep) SmPolyFace(sZoneTol,  // in : tol to store with new PolyFace
                                                           pPolyBrep, // in : if(pOptPolyShell==NULL &&
                                                                      // pOptPolyShell==NULL)
                                                                      //        NotNULL = Make NewPolyRegion in
                                                                      //        pOptPolyBrep,
                                                                      //                  Make NewPolyShell in
                                                                      //                  NewPolyRegion, Make this
                                                                      //                  PolyFace in NewPolyShell
                                                                      //        NULL    = error
                                                           NULL,      // in : if(pOptPolyShell==NULL)
                                                                      //        NotNull = make NewPolyShell in
                                                                      //        pOptPolyRegion,
                                                                      //                  make this PolyFace in
                                                                      //                  NewPolyShell
                                                           NULL,      // in : NotNULL = make this PolyFace in
                                                                      // pOptPolyShell
                                                           NULL);     // in : Normal for Face when known, NULL=ignore,
                                                                      // default:[NULL]
        SmPolyLoop* pPolyLoop = NULL;
        SER(pPolyFace->StartPolyEdgeLoop(pPolyLoop));
        SmPolyEdge* pPolyEdge = NULL;

        for ( int32_t jj = 0; jj < *pFaceVertexCount; ++jj, ++ii)
        {
            uint32_t sStartIndex = (jj == 0) ? ii + *pFaceVertexCount - 1 : ii - 1;
            uint32_t sEndIndex = ii;

            SmPolyVertex* pStartPolyVertex = sPolyVertices[sFaceVertexIndices[sStartIndex]];
            SmPolyVertex* pEndPolyVertex = sPolyVertices[sFaceVertexIndices[sEndIndex]];

            SER(pPolyLoop->AddPolyEdge(sZoneTol,             // in : min dist between distinct points
                                       pStartPolyVertex->GetPoint(), // in : Line start position   
                                       pEndPolyVertex->GetPoint(), // in : Line end position   
                                       pStartPolyVertex, // in : when m_pLastEndPolyVertex NotNULL (set on last call
                                                        // through pOptEndPolyVertex),
                                                             //           m_pLastEndPolyVertex is Start PolyVertex for PolyLoop->PolyEdge 
                                                             //      else: pOptStartPolyVertex NotNULL = Start PolyVertex for PolyLoop->PolyEdge, 
                                                             //                                NULL    = create New PolyVertex for 1stPolyEdge
                                       pEndPolyVertex, // in : NotNULL = stored in m_pLastEndPolyVertex to be 
                                                             //                Start PolyVertex for next AddPolyEdge() call.
                                       pPolyBrep,            // in : provides context for new obj construction and
                                                             //      accumulates new PolyVertices on its m_pVertexListHead list
                                       pPolyEdge));          // out: new edge, stitched to radial partners when pOptStartPolyVertex and pOptEndPolyVertex are NotNULL

        }
        SER(pPolyLoop->FinishPolyEdgeLoop(pPolyBrep));
    }

#ifdef SMU_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if (bDebugMe)
    {
        pPolyBrep->Dump();
        pPolyBrep->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SMU_DEBUG_CODE

    // Set output
    prSmPolyBrep = pPolyBrep;

    return SM_SUCCESS;
} // end SMU_BrepConvert::CreateSmPolyBrep_FromUsdMesh

} // end namespace SMU_BrepConvert
