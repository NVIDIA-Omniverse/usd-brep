// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmApiBrep.cpp

PURPOSE:
    Contains popular high level "C" type functions that operate on SmBreps.
    Additional functions exist in SMLib

**********************************************************************/

#include "StdAfx.h"

#include "SmApiBrep.h"
#include "SmApiGeneral.h"

#include <SmBrepCutting.h>
#include <SmCopyBrepMap.h>
#include "SmTess.h"
#include "SmHCR.h"
#include <SmMapPtrToPtr.h>
#include <SmTopologySweep.h>
#include <SmTransSweepGeometry.h>
#include <SmRotationalSweepGeometry.h>
#include <SmFilletExecutive.h>
#include <SmFilletStandardSolver.h>
#include <SmOffsetExecutive.h>
#include <SmStitch.h>
#include <SmTol.h>

#if SM_DEBUG_CODE
static const SmVector3d s_kBlack(0, 0, 0);
static const SmVector3d s_kBlue (0, 0, 1);
static const SmVector3d s_kGreen(0, 1, 0);
static const SmVector3d s_kRed  (1, 0, 0);
#endif

/*******************************************************************//**
PURPOSE --- Add zero-initialized slots for caller-owned curve outputs.

NOTES --- SmTArray::SetSize clears existing pointers if allocation fails.
          Allocate a replacement first, then transfer its buffer without
          another allocation. On failure the caller's array is unchanged.
***********************************************************************/
static SmApiStatus SmApiAddCurveOutputSlots
(
    SmTArray<SmCurve*>& rCurves,          ///< [in/out]: Existing outputs followed by new NULL slots
    ULONG               lAdditional      ///< [in ]: Number of slots to add
)
{
    const ULONG lOldSize = rCurves.GetSize();
    if( lAdditional == 0 )
        return SM_SUCCESS;
    if( lAdditional >= SM_BIG_ULONG - lOldSize )
        return SM_ERR_OUT_OF_MEMORY;
    const ULONG lNewSize = lOldSize + lAdditional;
    if( lNewSize <= rCurves.GetDataSize() )
    {
        rCurves.SetSize(lNewSize); // Already allocated; cannot fail to grow.
        return SM_SUCCESS;
    }

    SmTArray<SmCurve*> sGrown;
    if( sGrown.SetSize(lNewSize) >= SM_BIG_ULONG )
        return SM_ERR_OUT_OF_MEMORY;
    for( ULONG ii = 0; ii < lOldSize; ++ii )
        sGrown[ii] = rCurves[ii];

    // SetArray releases only owned storage, never the curves or borrowed
    // storage. The destination takes ownership of the replacement buffer.
    rCurves.SetArray(sGrown.GetDataSize(), sGrown.GetDataArray(), lNewSize);
    rCurves.SetIsBorrowed(FALSE);
    sGrown.SetIsBorrowed(TRUE);
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE --- Deep-copy caller-supplied profile curves into context-owned
            copies for the sweep / revolution wrappers below.

NOTES --- The sweep / revolution kernel paths
          (SmPrimitiveCreation::CreateLinearSweep, CreateRotationalSweep,
          and CreateTaperExtrude) consume - and sometimes delete - the
          SmCurve objects handed to them, stripping the internal NURB out
          of analytic inputs.  The public API and the Python bindings
          layered on top of it promise the caller's curves survive the
          call unchanged, so every consuming wrapper sweeps copies rather
          than the originals.  The copies are owned by the modeling
          context and are consumed / freed by the kernel.
***********************************************************************/
static SmApiStatus SmApiCopyCurvesForConsumingSweep
(
    const SmContext&          crContext,     ///< [in ]: Context that owns the copies
    const SmTArray<SmCurve*>& rInputCurves,  ///< [in ]: Caller-owned curves (left unchanged)
    SmTArray<SmCurve*>&       rCurveCopies   ///< [out]: Context-owned copies for the kernel to consume
)
{
    rCurveCopies.RemoveAll();
    for( ULONG ii = 0; ii < rInputCurves.GetSize(); ii++ )
    {
        SmCurve* pInput = rInputCurves[ii];
        SmCurve* pCopy = NULL;
        SmStatus stat = SM_ERR_INVALID_INPUT;
        if( pInput != NULL )
        {
            stat = pInput->Copy( crContext, pCopy );
            if( stat == SM_SUCCESS && pCopy == NULL )
                stat = SM_ERR;
        }
        if( stat != SM_SUCCESS )
        {
            for( ULONG jj = 0; jj < rCurveCopies.GetSize(); jj++ )
                delete rCurveCopies[jj];
            rCurveCopies.RemoveAll();
            return stat;
        }

        rCurveCopies.Add( pCopy );
    }
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE --- Validate a public N-ary Boolean input list without consuming it.
***********************************************************************/
static SmBoolean SmApiMergeBrepsInputsAreValid
(
    const SmTArray<SmBrep*>& rBreps
)
{
    if( rBreps.GetSize() < 2 )
        return FALSE;

    for( ULONG ii = 0; ii < rBreps.GetSize(); ii++ )
    {
        if( rBreps[ii] == NULL )
            return FALSE;

        for( ULONG jj = 0; jj < ii; jj++ )
            if( rBreps[ii] == rBreps[jj] )
                return FALSE;
    }

    return TRUE;
}

/*******************************************************************//**
PURPOSE --- Map the public Boolean enum to SmMerge::merge_breps' legacy
            zero-based operation selector.
***********************************************************************/
static SmBoolean SmApiMergeBrepsKernelOperation
(
    SmBooleanOperationType eOperation,
    ULONG&                  rlKernelOperation
)
{
    switch( eOperation )
    {
        case SM_BO_UNION:
            rlKernelOperation = 0;
            return TRUE;
        case SM_BO_INTERSECTION:
            rlKernelOperation = 1;
            return TRUE;
        case SM_BO_DIFFERENCE:
            rlKernelOperation = 2;
            return TRUE;
        case SM_BO_MERGE:
            rlKernelOperation = 3;
            return TRUE;
        default:
            return FALSE;
    }
}

/*******************************************************************//**
PURPOSE --- Validate the operands of a two-Brep Boolean before the kernel
            modifies or deletes either of them.

NOTES ---   The kernel modifies pBrep1 in place and deletes pBrep2, so the
            same Brep cannot be both operands. SM_BO_EXTRACT_SEPARATE has no
            single result: the kernel deletes both inputs and leaves the
            pieces in SmMerge members that nothing frees.
***********************************************************************/
static SmApiStatus SmApiValidateBooleanOperands
(
    SmBrep*                pBrep1,     ///< [in ]: Primary operand
    SmBrep*                pBrep2,     ///< [in ]: Second operand
    SmBooleanOperationType eOperation, ///< [in ]: Requested operation
    SmBrep*&               rpResult    ///< [out]: Cleared
)
{
    rpResult = NULL;
    if( pBrep1 == NULL || pBrep2 == NULL || pBrep1 == pBrep2 )
        return SM_ERR_INVALID_INPUT;
    if( eOperation == SM_BO_EXTRACT_SEPARATE )
        return SM_ERR_INVALID_INPUT;
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE --- Boolean two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiBoolean
(
    SmBrep*  pBrep1,                  ///< [in ]: Pointer to primary brep
    SmBrep*  pBrep2,                  ///< [in ]: Pointer to second brep
    SmBooleanOperationType operation, ///< [in ]: specify boolean operation
    SmBrep*& rpResult                ///< [out]: Resulting SmBrep
)
{
    SER( SmApiValidateBooleanOperands( pBrep1, pBrep2, operation, rpResult ) );

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
      SmApiDraw(pBrep1, &s_kBlue, 0, 1);
      SmApiDraw(pBrep2, &s_kGreen, 0, 1);
  }
#endif // SM_DEBUG_CODE

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmMerge sMerge( *pContext, pBrep1, pBrep2 );

  if(pBrep1->IsManifoldSolid() && pBrep2->IsManifoldSolid())
  {
    SER( sMerge.ManifoldBoolean( operation, rpResult ) );
  }
  else
  {
    SER( sMerge.NonManifoldBoolean( operation, rpResult ) );
  }


#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiBoolean

/*******************************************************************//**
PURPOSE --- Boolean Union two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiBooleanUnion
(
    SmBrep*  pBrep1,                  ///< [in ]: Pointer to primary brep
    SmBrep*  pBrep2,                  ///< [in ]: Pointer to second brep
    SmBrep*& rpResult                ///< [out]: Resulting SmBrep
)
{
    SER( SmApiValidateBooleanOperands( pBrep1, pBrep2, SM_BO_UNION, rpResult ) );

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pBrep2, &s_kGreen, 0, 1 );
  }
#endif // SM_DEBUG_CODE

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmMerge sMerge( *pContext, pBrep1, pBrep2 );

  if(pBrep1->IsManifoldSolid() && pBrep2->IsManifoldSolid())
  {
    SER( sMerge.ManifoldBoolean( SM_BO_UNION, rpResult ) );
  }
  else
  {
    SER( sMerge.NonManifoldBoolean( SM_BO_UNION, rpResult ) );
  }


#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif // SM_DEBUG_CODE 


  return(SM_SUCCESS);

} // End SmApiBooleanUnion

/*******************************************************************//**
PURPOSE --- Boolean Union two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiBooleanDifference
(
    SmBrep*  pBrep1,                  ///< [in ]: Pointer to primary brep
    SmBrep*  pBrep2,                  ///< [in ]: Pointer to second brep
    SmBrep*& rpResult                ///< [out]: Resulting SmBrep
)
{
    SER( SmApiValidateBooleanOperands( pBrep1, pBrep2, SM_BO_DIFFERENCE, rpResult ) );

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pBrep2, &s_kGreen, 0, 1 );
  }
#endif // SM_DEBUG_CODE 

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmMerge sMerge( *pContext, pBrep1, pBrep2 );

  if(pBrep1->IsManifoldSolid() && pBrep2->IsManifoldSolid())
  {
    SER( sMerge.ManifoldBoolean( SM_BO_DIFFERENCE, rpResult ) );
  }
  else
  {
    SER( sMerge.NonManifoldBoolean( SM_BO_DIFFERENCE, rpResult ) );
  }


#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiBooleanDifference

/*******************************************************************//**
PURPOSE --- Boolean Intersection two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiBooleanIntersection
(
    SmBrep*  pBrep1,                  ///< [in ]: Pointer to primary brep
    SmBrep*  pBrep2,                  ///< [in ]: Pointer to second brep
    SmBrep*& rpResult                ///< [out]: Resulting SmBrep
)
{
    SER( SmApiValidateBooleanOperands( pBrep1, pBrep2, SM_BO_INTERSECTION, rpResult ) );

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pBrep2, &s_kGreen, 0, 1 );
  }
#endif // SM_DEBUG_CODE

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmMerge sMerge( *pContext, pBrep1, pBrep2 );

  if(pBrep1->IsManifoldSolid() && pBrep2->IsManifoldSolid())
  {
    SER( sMerge.ManifoldBoolean( SM_BO_INTERSECTION, rpResult ) );
  }
  else
  {
    SER( sMerge.NonManifoldBoolean( SM_BO_INTERSECTION, rpResult ) );
  }


#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiBooleanIntersection

/*******************************************************************//**
PURPOSE --- Boolean two objects and return intersection curves.

USAGE NOTES --- Same as SmApiBoolean but also returns the intersection
    curves computed during the merge.  Curves reference geometry owned
    by rpResult; they are valid as long as rpResult is alive.

***********************************************************************/
SmApiStatus SmApiBooleanWithCurves
(
    SmBrep*  pBrep1,                         ///< [in ]: Pointer to primary brep
    SmBrep*  pBrep2,                         ///< [in ]: Pointer to second brep
    SmBooleanOperationType operation,        ///< [in ]: specify boolean operation
    SmBrep*& rpResult,                       ///< [out]: Resulting SmBrep
    SmTArray<SmEdge*>& rIntersectionEdges   ///< [out]: Intersection edges from the boolean (on first brep)
)
{
    SER( SmApiValidateBooleanOperands( pBrep1, pBrep2, operation, rpResult ) );

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
      SmApiDraw(pBrep1, &s_kBlue, 0, 1);
      SmApiDraw(pBrep2, &s_kGreen, 0, 1);
  }
#endif // SM_DEBUG_CODE

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmMerge sMerge( *pContext, pBrep1, pBrep2 );

  if(pBrep1->IsManifoldSolid() && pBrep2->IsManifoldSolid())
  {
    SER( sMerge.ManifoldBoolean( operation, rpResult ) );
  }
  else
  {
    SER( sMerge.NonManifoldBoolean( operation, rpResult ) );
  }

  // Collect intersection edge pointers into the caller's array.
  // The SmEdge objects themselves live on rpResult's topology and
  // remain valid as long as rpResult is alive.  We return edges
  // (not bare curves) so callers can use SmEdge::GetInterval()
  // to obtain the trimmed parametric domain for tessellation.
  SmTArray<SmEdge*>& xsectEdges = sMerge.m_vEdges;
  for (ULONG i = 0; i < xsectEdges.GetSize(); ++i) {
    if (xsectEdges[i])
      rIntersectionEdges.Add(xsectEdges[i]);
  }

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiBooleanWithCurves


/*******************************************************************//**
PURPOSE --- Boolean Merge two objects.  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiBooleanMerge
(
    SmBrep*  pBrep1,                  ///< [in ]: Pointer to primary brep
    SmBrep*  pBrep2,                  ///< [in ]: Pointer to second brep
    SmBrep*& rpResult                ///< [out]: Resulting SmBrep
)
{
    SER( SmApiValidateBooleanOperands( pBrep1, pBrep2, SM_BO_MERGE, rpResult ) );

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pBrep2, &s_kGreen, 0, 1 );
  }
#endif // SM_DEBUG_CODE

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmMerge sMerge( *pContext, pBrep1, pBrep2 );

  if(pBrep1->IsManifoldSolid() && pBrep2->IsManifoldSolid())
  {
    SER( sMerge.ManifoldBoolean( SM_BO_MERGE, rpResult ) );
  }
  else
  {
    SER( sMerge.NonManifoldBoolean( SM_BO_MERGE, rpResult ) );
  }


#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiBooleanMerge

/*******************************************************************//**
PURPOSE --- Apply one Boolean operation across a list of at least two Breps.

USAGE NOTES --- Inputs are consumed only after validation succeeds.

***********************************************************************/
SmApiStatus SmApiMergeBreps
(
    SmTArray<SmBrep*>& rBreps,         ///< [in/out]: Input Breps; consumed after validation
    SmBooleanOperationType lOperation, ///< [in ]: UNION, INTERSECTION, DIFFERENCE, or MERGE
    SmBrep*& rpResult                 ///< [out]: Resulting SmBrep; NULL on failure
)
{
  // The output is valid only on success.  Reject every unsafe list shape
  // before the debug draw or kernel has a chance to dereference/consume it.
  rpResult = NULL;
  ULONG lKernelOperation = 0;
  if (   !SmApiMergeBrepsInputsAreValid(rBreps)
      || !SmApiMergeBrepsKernelOperation(lOperation, lKernelOperation))
  {
    return SM_ERR_INVALID_INPUT;
  }

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    for(ULONG ii = 0; ii < rBreps.GetSize(); ii++)
    {
      SmApiDraw( rBreps[ii], &s_kBlue, 0, 1 );
    }
  }
#endif // SM_DEBUG_CODE

  SER( SmMerge::merge_breps( rBreps, lKernelOperation, rpResult ) );
  if (rpResult == NULL)
  {
    SER(SM_ERR);
  }

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiMergeBreps

/*******************************************************************//**
PURPOSE --- Perform a 2D Boolean operation on two Breps 

USAGE NOTES --- 
  Each Brep must have only one surface and it must be a plane
  The planar surfaces from each Brep must lie on the same infinite plane.
  Kernel failures are returned to the caller without assigning rpResult.

***********************************************************************/
SmApiStatus SmApiBoolean2d
(
  SmBrep*  pBrep1,                        ///< [in ]: Primary brep                                                  <br>
  SmBrep*  pBrep2,                        ///< [in ]: Second brep                                                   <br>
  Sm2DBooleanOperationType eOpType,       ///< [in ]: Operation Type                                                <br>
                                          ///<      : SM_2D_UNION,        = Union of 2D regions A and B             <br>
                                          ///<      : SM_2D_INTERSECTION, = Intersection of 2D regions A and B      <br>
                                          ///<      : SM_2D_DIFFERENCE,   = Difference - A minus B                  <br>
                                          ///<      : SM_2D_EXCLUSIVE_OR, = XOR = (A union B) - (A intersect B)     <br> 
                                          ///<      : SM_2D_MERGE         = Merge Operation  <br>                  <br>
  SmBrep*& rpResult                      ///< [out]: Resulting SmBrep                                              <br>
)
{
#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep1, &s_kBlue, 0, 1 );
    SmApiDraw( pBrep2, &s_kGreen, 0, 1 );
  }
#endif // SM_DEBUG_CODE

  SER( SmPrimitiveCreation::Boolean2D( pBrep1, pBrep2, eOpType, rpResult ) );

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kRed, 1, 1 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiBoolean2d


/*******************************************************************//**
PURPOSE --- Tesselate given brep.  

USAGE NOTES --- We could offer different ways to tessellate

***********************************************************************/
SmApiStatus SmApiTessellateBoundaries(
    SmBrep* pBrep,
    double dAngleTolDeg,
    SmTArray<SmPoint3d>& rPoints,
    SmTArray<ULONG>& rVertexCounts,
    SmTArray<SmEdge*>& rEdges)
{
    if (!pBrep)
    {
        rPoints.RemoveAll();
        rVertexCounts.RemoveAll();
        rEdges.RemoveAll();
        return SM_ERR_INVALID_INPUT;
    }
    return pBrep->TessellateBoundaries(dAngleTolDeg, rPoints, rVertexCounts, rEdges);
}

/****************************************************************************************
PURPOSE: Tessellate with the quality controls supplied as a struct.

NOTES: Rejects failed faces unless bAllowPartial is TRUE and pFailures or pReport is
       supplied. Output errors always fail. The caller owns successful output; errors
       leave it null. pFailures is cleared on entry and retained on error, using input
       GetFaces() indices.
****************************************************************************************/
SmApiStatus SmApiTessellate
(
    SmBrep* pBrep,                              ///< [in ]: Pointer to brep
    SmPolyBrep*& rpResult,                      ///< [out]: Resulting SmPolyBrep
    const SmTessellationParams& rParams,        ///< [in ]: Quality controls
    SmBoolean bAllowPartial,                    ///< [in ]: Retain partial meshes when some faces fail
    SmTArray<SmTessellationFailure>* pFailures, ///< [out]: First failure per original face
    SmTessellationReport* pReport               ///< [out]: Nonfatal output topology diagnostics
)
{
  const double dChordHeightTol     = rParams.dChordHeightTol;
  const double dCurveAngleTolDeg   = rParams.dCurveAngleTolDeg;
  const double dSurfaceAngleTolDeg = rParams.dSurfaceAngleTolDeg;
  double       dMaxEdgeLength      = rParams.dMaxEdgeLength;  // mutable: negative selects an auto cap below
  const double dMaxAspectRatio     = rParams.dMaxAspectRatio;
  rpResult = nullptr;
  SmBoolean rbFailedFaces = FALSE;
  SmTArray<SmTessellationFailure> failures;
  if (pReport) *pReport = SmTessellationReport();
  if (pFailures) pFailures->SetSize(0);
  if (bAllowPartial && !pFailures && !pReport)
  {
    SER_MSG(SM_ERR_INVALID_INPUT, _T("Partial tessellation requires per-face diagnostics"));
  }
#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
  }
#endif // SM_DEBUG_CODE
  
  // Validate non-null input
  NER_MSG(pBrep, _T("Null Brep input"));

  // when asked - pick a max 3DEdge size from model size
  if(dMaxEdgeLength < 0.0)
  {
    SmExtent3d brepBBox;
    pBrep->CalculateBoundingBox( brepBBox );
    double size = brepBBox.GetSize().Length();
    if(size >= 1.0e15 || size <= 0.0) size = 1.0;
    dMaxEdgeLength = -0.025 * size * dMaxEdgeLength;
  }
  
  ULONG rlNumLamina = 0;
  SmStatus eStatus = pBrep->ConvertToPolyBrep( rpResult, 
                                              rbFailedFaces, 
                                              rlNumLamina, 
                                              dChordHeightTol, 
                                              dCurveAngleTolDeg,
                                              dSurfaceAngleTolDeg,
                                              dMaxEdgeLength, 
                                              dMaxAspectRatio,
                                              FALSE,
                                              FALSE,
                                              TRUE, FALSE, &failures, pReport);

  if (pFailures) *pFailures = failures;

  // Discard unusable or unrequested partial output.
  if (eStatus != SM_SUCCESS || (!failures.IsEmpty() && !bAllowPartial) || rpResult == nullptr)
  {
    if (rpResult)
    {
      rpResult->SetOKBackPtrs(FALSE);
      delete rpResult;
      rpResult = nullptr;
    }
    if (eStatus != SM_SUCCESS)
      return eStatus;
    SER_MSG(SM_ERR, _T("Tessellation failed to produce a complete mesh"));
  }

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( pBrep, &s_kBlue, 1, 0 );
  }
#endif // SM_DEBUG_CODE

  return(eStatus);

} // End SmApiTessellate


/*******************************************************************//**
PURPOSE --- Project a brep onto a plane  

USAGE NOTES --- The result is an array of curves

***********************************************************************/
SmApiStatus SmApiProjectBrepOntoPlane
(
    SmBrep*  pBrep,                         ///< [in ]: Pointer to brep
    const SmVector3d& rPlanePt,             ///< [in ]: Point to define position of plane
    const SmVector3d& rPlaneNormal,         ///< [in ]: Normal to define direction of projection
    SmTArray<SmCurve*>& rProjectionCurves  ///< [out]: Resulting projected curves
)
{
#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
  }
#endif // SM_DEBUG_CODE

  if( pBrep == NULL )
    return( SM_ERR_INVALID_INPUT );

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmVector3d sPlaneNormal = rPlaneNormal;
  if( sPlaneNormal.Unitize() != SM_SUCCESS )
    return( SM_ERR_INVALID_INPUT );

  SmExtent3d BBox;
  pBrep->CalculateBoundingBox( BBox );

  SmVector3d diagonal = BBox.GetSize();
  double length = diagonal.Length();

  SmVector3d x, y, z;
  sPlaneNormal.MakeUnitOrthoVectors( 0, x, y, z );

  SmPoint3d planeOrigin = BBox.GetMid() - sPlaneNormal * length;

  SmAxis2Placement plane;
  plane.SetCanonical( planeOrigin, y, z );

  SmAxis2Placement sInvertPlane;
  plane.Invert( sInvertPlane );

  double approxTol = length * 1.0e-6;
  approxTol = smos_Min( approxTol, pBrep->GetTolerance()*0.1 );
  double angleToleranceRadians = 10.0*SM_PI / 180.0;

  SmHCR sHCR( *pContext, pBrep, sInvertPlane, approxTol, angleToleranceRadians, TRUE, FALSE );
  SmApiStatus status = sHCR.ComputeGlobalVisibility();
  if(status != SM_SUCCESS)
  {
    return(status);
  }

  SmTArray<SmCurve*> sVisibleCrvs;
  sHCR.GetVisibleCurves( sVisibleCrvs );

  SmVector3d sDirection( -sPlaneNormal.x, -sPlaneNormal.y, -sPlaneNormal.z );
  const ULONG lSizeOnEntry = rProjectionCurves.GetSize();

  for(ULONG i = 0; i < sVisibleCrvs.GetSize(); i++)
  {
    SmCurve* pProjectedCurve = NULL;
    SmStatus projStat = sVisibleCrvs.GetAt( i )->CreatePlaneProjection( *pContext, SM_PT_PARALLEL, rPlanePt, sPlaneNormal, sDirection, pProjectedCurve );
    if( projStat != SM_SUCCESS )
    {
      // Drop this call's curves rather than return a partial projection.
      delete pProjectedCurve;
      for( ULONG jj = lSizeOnEntry; jj < rProjectionCurves.GetSize(); jj++ )
        delete rProjectionCurves[jj];
      rProjectionCurves.SetSize( lSizeOnEntry );
      return( projStat );
    }
    if(pProjectedCurve)
      rProjectionCurves.Add( pProjectedCurve );
  }


#ifdef SM_ASSERT_VALID
  for(ULONG ii = 0; ii < rProjectionCurves.GetSize(); ii++)
    SM_ASSERT_VALID( rProjectionCurves[ii] );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rProjectionCurves, &s_kBlue, 0, 0 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiProjectBrepOntoPlane

/*******************************************************************//**
PURPOSE --- Cut a brep with a plane  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiCut
(
    SmBrep*  pBrep,                         ///< [in/out]: Pointer to brep
    const SmVector3d& rPlanePt,             ///< [in ]: Point to define position of plane
    const SmVector3d& rPlaneNormal         ///< [in ]: Normal to define direction of projection
)
{
#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
  }
#endif // SM_DEBUG_CODE

  SmVector3d sPlaneNormal = rPlaneNormal;
  if( sPlaneNormal.Unitize() != SM_SUCCESS )
    return( SM_ERR_INVALID_INPUT );

  SmBrepCutting sCutting( pBrep );

  SmPlaneCutter sCutter( rPlanePt, sPlaneNormal );

  double dTol = pBrep->GetTolerance();
  SmApiStatus stat = sCutting.DoCut( &sCutter, dTol, TRUE );
  if(stat != SM_SUCCESS)
    return(stat);

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( pBrep );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( pBrep, &s_kBlue, 1, 1 );
  }
#endif // SM_DEBUG_CODE



  return(SM_SUCCESS);

} // End SmApiCut

/*******************************************************************//**
PURPOSE --- Cut a brep with a projected curve 

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiProjectAndTrim
(
    SmBrep*  pBrep,                         ///< [in/out]: Pointer to brep
    SmBSplineCurve* rCurveToProject,        ///< [in ]: Curve to project onto brep
    const SmVector3d& rProjectionDir,       ///< [in ]: Normal to define direction of projection
    const SmPoint3d& rRefPt                ///< [in ]: Point to project onto brep that will be on the side to keep
)
{
  if(pBrep == NULL || rCurveToProject == NULL)
    return(SM_ERR_INVALID_INPUT);

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
    SmApiDraw( rCurveToProject, NULL, 0, 0 );

    SmPoint3d sStart, sEnd;
    rCurveToProject->GetEnds( sStart, sEnd );
    SmVector3d sDrawDir = rProjectionDir;
    SmApiDraw( sStart, sDrawDir, &s_kBlue, 0, 0 );
  }
#endif // SM_DEBUG_CODE

  SmVector3d sProjectionDir = rProjectionDir;
  if( sProjectionDir.Unitize() != SM_SUCCESS )
    return( SM_ERR_INVALID_INPUT );

  // Keep the side of the projected loop that contains the caller's reference
  // point. SM_TT_KEEP_POINT only needs a point on the side to keep, which is
  // exactly what rRefPt is documented to be, so pass it through directly.
  SmApiStatus stat = pBrep->ProjectAndTrim( *rCurveToProject, sProjectionDir, rRefPt, SM_TT_KEEP_POINT, NULL, NULL, NULL );
  if( stat != SM_SUCCESS )
    return( stat );

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( pBrep );
#endif // SM_ASSERT_VALID 

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( pBrep, &s_kBlue, 1, 1 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiProjectAndTrim


/*******************************************************************//**
PURPOSE --- Determine the silhouette curves  

USAGE NOTES --- The result is an array of curves

***********************************************************************/
SmApiStatus SmApiCreateSilhouetteCurves
(
    SmBrep*  pBrep,                         ///< [in/out]: Pointer to brep
    const SmVector3d& rPlanePt,             ///< [in ]: Point to define position of plane
    const SmVector3d& rPlaneNormal,         ///< [in ]: Normal to define direction of projection
    SmTArray<SmCurve*>& rProjectionCurves  ///< [out]: Resulting projected curves
)
{
  (void)rPlanePt; // only referenced inside SM_DEBUG_CODE draw below
#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
    SmPoint3d sDrawPt = rPlanePt;
    SmVector3d sDrawNormal = rPlaneNormal;
    SmApiDraw( sDrawPt, sDrawNormal, &s_kBlue, 0, 0 );
  }
#endif // SM_DEBUG_CODE

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmVector3d sPlaneNormal = rPlaneNormal;
  if( sPlaneNormal.Unitize() != SM_SUCCESS )
    return( SM_ERR_INVALID_INPUT );

  SmExtent3d BBox;
  pBrep->CalculateBoundingBox( BBox );

  SmVector3d diagonal = BBox.GetSize();
  double length = diagonal.Length();

  double approxTol = length * 1.0e-4;
  double angleToleranceRadians = 10.0*SM_PI / 180.0;

  SmHCR sHCR( *pContext, pBrep, sPlaneNormal, approxTol, angleToleranceRadians, TRUE, FALSE );
  SmApiStatus stat = sHCR.ComputeGlobalVisibility();
  if(stat != SM_SUCCESS)
  {
    return(stat);
  }

  SmTArray<SmCurve*> sHcrCurves;
  sHCR.GetVisibleCurves( sHcrCurves );

  ULONG lStartSize = rProjectionCurves.GetSize();
  SER( SmApiAddCurveOutputSlots(rProjectionCurves, sHcrCurves.GetSize()) );
  for( ULONG ii = 0; ii < sHcrCurves.GetSize(); ii++ )
  {
    SmCurve*& pCopy = rProjectionCurves[lStartSize + ii];
    SmStatus copyStat = sHcrCurves[ii]->Copy( *pContext, pCopy );
    if( copyStat != SM_SUCCESS || pCopy == NULL )
    {
      for( ULONG jj = lStartSize; jj < rProjectionCurves.GetSize(); jj++ )
        delete rProjectionCurves[jj];
      rProjectionCurves.SetSize( lStartSize );
      return ( copyStat != SM_SUCCESS ) ? copyStat : SM_ERR;
    }
  }

#ifdef SM_ASSERT_VALID
  for(ULONG ii = 0; ii < rProjectionCurves.GetSize(); ii++)
    SM_ASSERT_VALID( rProjectionCurves[ii] );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rProjectionCurves, &s_kBlue, 0, 0 );
  }
#endif // SM_DEBUG_CODE


  return(SM_SUCCESS);

} // End SmApiCreateSilhouetteCurves

/*******************************************************************//**
PURPOSE --- Project a curve onto the faces of a brep, returning the 3d
            projection curves.

USAGE NOTES --- Append newly allocated curves on success; the caller owns
                them. Leave rCurves3d unchanged on failure, including when
                the projection produces no curves.
***********************************************************************/
SmApiStatus SmApiProjectCurve
(
    SmBrep* pBrep,                       ///< [in ]: Pointer to brep
    SmBSplineCurve* pCurveToProject,     ///< [in ]: Pointer to curve to project
    const SmVector3d& projectionVector,        ///< [in ]: Projection Vector
    SmTArray<SmCurve*> & rCurves3d      ///< [in/out]: Projection curves appended on success; unchanged on failure
)
{
  if(pBrep == NULL || pCurveToProject == NULL)
    return(SM_ERR_INVALID_INPUT);

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
    SmApiDraw( pCurveToProject, NULL, 0, 0 );

    SmPoint3d sStart, sEnd;
    pCurveToProject->GetEnds( sStart, sEnd );
    SmApiDraw( sStart, projectionVector, NULL, 0, 0 );
  }
#endif // SM_DEBUG_CODE

  const SmContext* pContext = SmApiGetOrCreateContext();

  // The kernel resets its output array. Keep caller-owned curves out of it.
  SmTArray<SmCurve*> sProjectedCurves;
  SmObjsDelete<SmCurve*> cleanupCurves(&sProjectedCurves);
  SmApiStatus stat = pBrep->CreateParallelProjectionCurves( *pContext, *pCurveToProject,
          projectionVector, NULL, NULL, NULL, &sProjectedCurves, NULL, NULL );

  // check result
  if(stat != SM_SUCCESS || sProjectedCurves.GetSize() == 0)
    return ( stat != SM_SUCCESS ) ? stat : SM_ERR;

  const ULONG lStartSize = rCurves3d.GetSize();
  SER( SmApiAddCurveOutputSlots(rCurves3d, sProjectedCurves.GetSize()) );
  for( ULONG ii = 0; ii < sProjectedCurves.GetSize(); ++ii )
    rCurves3d[lStartSize + ii] = sProjectedCurves[ii];
  cleanupCurves.Clear();

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rCurves3d, &s_kBlue, 0, 0 );
  }
#endif // SM_DEBUG_CODE

  return(SM_SUCCESS);

} // End SmApiProjectCurve

/*******************************************************************//**
PURPOSE --- Stitch all edges of a brep

USAGE NOTES --- 
***********************************************************************/
SmApiStatus SmApiStitch
(
    SmBrep* pBrep                          ///< [in ]: Pointer to brep                                             <br>
)
{
  if(pBrep == NULL)
    return(SM_ERR_INVALID_INPUT);

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
  }
#endif // SM_DEBUG_CODE

  pBrep->StitchAndOrient();

  // Check manifold?

  return SM_SUCCESS;

} // End SmStitch

/*******************************************************************//**
PURPOSE ---

USAGE NOTES --- 
***********************************************************************/
SmApiStatus SmApiStitch
(
    SmBrep* pBrep,                     ///< [in ]: Pointer to brep                                             <br>
    SmFace* pFaceToStitch             ///< [in ]: Pointer to curve to project                                 <br>
)
{
  if(pBrep == NULL || pFaceToStitch == NULL )
    return(SM_ERR_INVALID_INPUT);

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( pBrep, NULL, 1, 1 );
    SmApiDraw( pFaceToStitch, NULL, 1, 1 );
  }
#endif // SM_DEBUG_CODE 


  ULONG  lNumEdgesStitched;
  double dMaxVertGap;
  double dMaxEdgeGap;
  double dMinUnstitchedVertGap;
  double dMinUnstitchedEdgeGap;

  const SmContext* pContext = SmApiGetOrCreateContext();

  SmMerge sMerge( *pContext, pBrep, NULL );

  return sMerge.MergeAddedFace( pFaceToStitch, lNumEdgesStitched, dMaxVertGap, dMaxEdgeGap, dMinUnstitchedVertGap, dMinUnstitchedEdgeGap  );

} // End SmApiStitch

/*******************************************************************//**
PURPOSE --- Sweep a set of curves along a vector creating a brep  

NOTES --- 

***********************************************************************/   
SmApiStatus SmApiCreateLinearSweep
( 
    const SmTArray<SmCurve*>& rCurvesToSweep,  ///< [in ]: Curves to sweep
    const SmVector3d& rSweepDir,               ///< [in ]: Sweep direction
    double dSweepDist,                   ///< [in ]: Sweep distance
    SmBoolean bCapEnds,                  ///< [in ]: Cap the ends of the result
    SmBrep*& rpResult                   ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;

 #if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( rCurvesToSweep, NULL, 1, 1 );
       
        SmPoint3d sStart, sEnd;
        rCurvesToSweep[0]->GetEnds( sStart, sEnd );
        SmApiDraw( sStart, rSweepDir, NULL, 0, 0 );
    }
#endif // SM_DEBUG_CODE
    
    SmContext* pContext = SmApiGetOrCreateContext();

    // Sweep copies of the inputs: CreateLinearSweep consumes the curves.
    SmTArray<SmCurve*> sCurveCopies;
    SER( SmApiCopyCurvesForConsumingSweep( *pContext, rCurvesToSweep, sCurveCopies ) );

    SmBrep* pBrep = new (*pContext) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation pPrimitive( pBrep->GetInfiniteRegion(), 0, true );
    SER( pPrimitive.CreateLinearSweep( sCurveCopies, rSweepDir, dSweepDist, 1, bCapEnds ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif // SM_ASSERT_VALID

    sBrepCleanup.Clear();
    rpResult = pBrep;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, NULL, 0, 0 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateLinearSweep


/*******************************************************************//**
PURPOSE --- Revolve a set of curves creating a brep  

NOTES --- 

***********************************************************************/   
SmApiStatus SmApiCreateRotationalSweep
( 
    const SmTArray<SmCurve*>& rCurvesToSweep,  ///< [in ]: Curves to sweep
    const SmPoint3d& rBasePt,                  ///< [in ]: Base point of axis
    const SmVector3d& rAxis,                   ///< [in ]: Direction of axis
    double dAngleDeg,                    ///< [in ]: Angle of rotation
    SmBoolean bCapEnds,                  ///< [in ]: Cap the ends of the result
    SmBrep*& rpResult                   ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( rCurvesToSweep, NULL, 1, 1 );
        SmApiDraw( rBasePt, rAxis, NULL, 0, 0 );
    }
#endif // SM_DEBUG_CODE
    
    SmContext* pContext = SmApiGetOrCreateContext();

    // Sweep copies of the inputs: CreateRotationalSweep consumes the curves.
    SmTArray<SmCurve*> sCurveCopies;
    SER( SmApiCopyCurvesForConsumingSweep( *pContext, rCurvesToSweep, sCurveCopies ) );

    SmBrep* pBrep = new (*pContext) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation pPrimitive( pBrep->GetInfiniteRegion(), 0, true );
    SER( pPrimitive.CreateRotationalSweep( sCurveCopies, rBasePt, rAxis, dAngleDeg, 1, bCapEnds ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif // SM_ASSERT_VALID

    sBrepCleanup.Clear();
    rpResult = pBrep;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 0 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateRotationalSweep


/*******************************************************************//**
PURPOSE --- Sweep (Extrude) a set of curves using a draft angle (taper)  

NOTES --- 

***********************************************************************/   
SmApiStatus SmApiCreateDraftSweep
(
    const SmTArray<SmCurve*>& rCurvesToSweep, ///< [in ]: Curves to sweep
    double dHeight,                     ///< [in ]: Height of sweep or extrusion
    double dAngleDeg,                   ///< [in ]: Draft Angle
    SmBoolean bCornerType,              ///< [in ]: 0 = SM_OC_LINEAR_EXTENSION, 1 = SM_OC_FILLET_CORNER
    int iCapEnds,                       ///< [in ]: 0 = none, 1 = at curves, 2 = at offset, 3 = both ends
    SmBrep*& rpResult                  ///< [out]: Resulting SmBrep
)
{
    rpResult = NULL;

#if SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    SmApiDraw( rCurvesToSweep, NULL, 1, 1 );
  }
#endif // SM_DEBUG_CODE

  SmContext* pContext = SmApiGetOrCreateContext();

  // Sweep copies of the inputs: CreateTaperExtrude consumes the curves.
  SmTArray<SmCurve*> sCurveCopies;
  SER( SmApiCopyCurvesForConsumingSweep( *pContext, rCurvesToSweep, sCurveCopies ) );

  SmBrep* pBrep = new (*pContext) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

  // Let CreateTaperExtrude compute normal
  SmVector3d sCrvNormal( 0, 0, 0 );

  SmPrimitiveCreation pPrimitive( pBrep->GetInfiniteRegion(), 0, true );

  if(bCornerType == 0)
  {
    SER( pPrimitive.CreateTaperExtrude( sCurveCopies, dHeight, dAngleDeg, iCapEnds, sCrvNormal, SM_OC_LINEAR_EXTENSION ) );
  }
  else
  {
    SER( pPrimitive.CreateTaperExtrude( sCurveCopies, dHeight, dAngleDeg, iCapEnds, sCrvNormal, SM_OC_FILLET_CORNER ) );
  }

#ifdef SM_ASSERT_VALID
  SM_ASSERT_VALID( pBrep );
#endif // SM_ASSERT_VALID

  sBrepCleanup.Clear();
  rpResult = pBrep;

#if SM_DEBUG_CODE
  if(bDebugMe)
  {
    SmApiDraw( rpResult, &s_kBlue, 0, 0 );
  }
#endif // SM_DEBUG_CODE

  return(SM_SUCCESS);
} // End SmApiCreateDraftSweep

/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreatePipeSweep
(
    double              dPipeRadius,    ///< [in ]: Radius                                             <br>
    SmBSplineCurve    * pPathCurve,     ///< [in ]: Path Curve                                         <br>
    SmBoolean           bCapEnds,       ///< [in ]: 
    SmBrep*&            rpResult       ///< [out]: Resulting SmBrep                                          <br>
)
{

    if( pPathCurve == NULL || dPipeRadius <= 0.0 )
        return SM_ERR_INVALID_INPUT;

    SmBrep* pBrep = new (*SmApiGetOrCreateContext()) SmBrep();

    SmTArray<SmFace*> sStartFaces, sSideFaces, sEndFaces;
    SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion(), 0, true );
    SmStatus stat = sPC.CreatePipeSweep(
        dPipeRadius, pPathCurve, 0.0, bCapEnds, FALSE,
        sStartFaces, sSideFaces, sEndFaces );

    if( stat != SM_SUCCESS )
    {
        delete pBrep;
        rpResult = NULL;
        return stat;
    }

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    rpResult = pBrep;
    return SM_SUCCESS;
} // End SmApiCreatePipeSweep

/*******************************************************************//**
PURPOSE --- Sweep profile boundary curves along a planar path.

NOTES --- Closed profile loops use even/odd nesting.  An open path with
  both endpoint caps, or a closed path, classifies a completed manifold
  boundary as a nested solid.  Other open-path cap selections leave sheet
  or open-shell geometry.  Tangent-discontinuous joins use sharp, mitered
  transitions rather than inserted rounded corners.
***********************************************************************/
SmApiStatus SmApiCreateSweepAlongPlanarPath
( 
    SmTArray<SmCurve*>& rProfileCurves, ///< [in ]: Profile boundary curves                                 <br>
    SmTArray<SmCurve*>& rPathCurves,    ///< [in ]: Ordered, endpoint-connected open or closed planar path    <br>
    SmBoolean           bMoveProfile,   ///< [in ]: Anchor profile center (TRUE) or first point (FALSE)       <br>
                                        ///<        to the path start                                         <br>
    int iCapEnds,                       ///< [in ]: 0 = none, 1 = profile/start, 2 = far/path, 3 = both ends; <br>
                                        ///<        ignored for closed paths; other values return             <br>
                                        ///<        SM_ERR_INVALID_INPUT                                      <br>
    SmBrep*& rpResult                  ///< [out]: New SmBrep; inputs unchanged; NULL on failure             <br>
)
{
    rpResult = NULL;

    if( rProfileCurves.GetSize() == 0 || rPathCurves.GetSize() == 0 )
        return SM_ERR_INVALID_INPUT;
    if( iCapEnds < 0 || iCapEnds > 3 )
        return SM_ERR_INVALID_INPUT;

    SmBrep* pBrep = NULL;
    double dTol = SM_ZONE_TOL_3D;
    SmBoolean bCapStart = ( iCapEnds & 1 ) ? TRUE : FALSE;
    SmBoolean bCapEnd = ( iCapEnds & 2 ) ? TRUE : FALSE;

    SmStatus stat = SmPrimitiveCreation::CreateSweepAlongPlanarPath(
        *SmApiGetOrCreateContext(),
        rProfileCurves,
        rPathCurves,
        dTol,
        bCapStart,
        bCapEnd,
        bMoveProfile,
        pBrep );

    if( stat != SM_SUCCESS || pBrep == NULL )
    {
        delete pBrep;
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;
    }

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    rpResult = pBrep;

    return SM_SUCCESS;
} // End SmApiCreateSweepAlongPlanarPath


/*******************************************************************//**
PURPOSE --- Full curve sweep with scale reference, scale curve, and options

NOTES ---   Wraps SmPrimitiveCreation::CreateCurveSweep
***********************************************************************/
SmApiStatus SmApiCreateCurveSweep
(
    SmTArray<SmCurve*>& rProfileCurves,
    SmBSplineCurve* pPathCurve,
    SmBSplineCurve* pScaleReference,
    SmBSplineCurve* pScaleCurve,
    SmSweepOptions* pOptions,
    SmBrep*& rpResult,
    SmTArray<SmFace*>* pOptStartFaces,
    SmTArray<SmFace*>* pOptSideFaces,
    SmTArray<SmFace*>* pOptEndFaces
)
{
    rpResult = NULL;

    if( rProfileCurves.GetSize() < 1 || !pPathCurve )
        return SM_ERR_INVALID_INPUT;

    SmSweepOptions sDefaultOptions;
    if( !pOptions )
        pOptions = &sDefaultOptions;

    rpResult = new (*SmApiGetOrCreateContext()) SmBrep();

    SmTArray<SmFace*> sStartFaces, sSideFaces, sEndFaces;

    SmPrimitiveCreation sPC( rpResult->GetInfiniteRegion() );
    const SmStatus eSweepStat = sPC.CreateCurveSweep( rProfileCurves, pPathCurve, pScaleReference, pScaleCurve,
                               SM_ZONE_TOL_3D, pOptions, sStartFaces, sSideFaces, sEndFaces );
    if( eSweepStat != SM_SUCCESS )
    {
        // Do not hand back a half-built Brep. The kernel works on copies of
        // the input curves, so the Brep owns nothing of the caller's.
        delete rpResult;
        rpResult = NULL;
        SER( eSweepStat );
    }

    if( pOptStartFaces ) *pOptStartFaces = sStartFaces;
    if( pOptSideFaces  ) *pOptSideFaces  = sSideFaces;
    if( pOptEndFaces   ) *pOptEndFaces   = sEndFaces;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

    return SM_SUCCESS;

} // SmApiCreateCurveSweep

/*******************************************************************//**
PURPOSE --- Sweep from existing face topology along a path

NOTES ---   Wraps SmPrimitiveCreation::CreateCurveSweepFromFaces
***********************************************************************/
SmApiStatus SmApiCreateCurveSweepFromFaces
(
    SmTArray<SmFace*>& rFaces,
    SmBSplineCurve* pPathCurve,
    SmBSplineCurve* pScaleReference,
    SmBSplineCurve* pScaleCurve,
    SmSweepOptions* pOptions,
    SmBrep*& rpResult,
    SmTArray<SmFace*>* pOptStartFaces,
    SmTArray<SmFace*>* pOptSideFaces,
    SmTArray<SmFace*>* pOptEndFaces
)
{
    rpResult = NULL;

    if( rFaces.GetSize() < 1 || !pPathCurve )
        return SM_ERR_INVALID_INPUT;

    SmSweepOptions sDefaultOptions;
    if( !pOptions )
        pOptions = &sDefaultOptions;

    rpResult = new (*SmApiGetOrCreateContext()) SmBrep();

    SmTArray<SmFace*> sStartFaces, sSideFaces, sEndFaces;

    SmPrimitiveCreation sPC( rpResult->GetInfiniteRegion() );
    const SmStatus eSweepStat = sPC.CreateCurveSweepFromFaces( rFaces, pPathCurve, pScaleReference, pScaleCurve,
                                        SM_ZONE_TOL_3D, pOptions, sStartFaces, sSideFaces, sEndFaces );
    if( eSweepStat != SM_SUCCESS )
    {
        // Do not hand back a half-built Brep. The kernel works on copies of
        // the input curves, so the Brep owns nothing of the caller's.
        delete rpResult;
        rpResult = NULL;
        SER( eSweepStat );
    }

    if( pOptStartFaces ) *pOptStartFaces = sStartFaces;
    if( pOptSideFaces  ) *pOptSideFaces  = sSideFaces;
    if( pOptEndFaces   ) *pOptEndFaces   = sEndFaces;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

    return SM_SUCCESS;

} // SmApiCreateCurveSweepFromFaces

/*******************************************************************//**
PURPOSE --- Sweep from existing edge topology along a path

NOTES ---   Wraps SmPrimitiveCreation::CreateCurveSweepFromEdges
***********************************************************************/
SmApiStatus SmApiCreateCurveSweepFromEdges
(
    SmTArray<SmEdge*>& rEdges,
    SmBSplineCurve* pPathCurve,
    SmBSplineCurve* pScaleReference,
    SmBSplineCurve* pScaleCurve,
    SmSweepOptions* pOptions,
    SmBrep*& rpResult,
    SmTArray<SmFace*>* pOptStartFaces,
    SmTArray<SmFace*>* pOptSideFaces,
    SmTArray<SmFace*>* pOptEndFaces
)
{
    rpResult = NULL;

    if( rEdges.GetSize() < 1 || !pPathCurve )
        return SM_ERR_INVALID_INPUT;

    SmSweepOptions sDefaultOptions;
    if( !pOptions )
        pOptions = &sDefaultOptions;

    rpResult = new (*SmApiGetOrCreateContext()) SmBrep();

    SmTArray<SmFace*> sStartFaces, sSideFaces, sEndFaces;

    SmPrimitiveCreation sPC( rpResult->GetInfiniteRegion() );
    const SmStatus eSweepStat = sPC.CreateCurveSweepFromEdges( rEdges, pPathCurve, pScaleReference, pScaleCurve,
                                        SM_ZONE_TOL_3D, pOptions, sStartFaces, sSideFaces, sEndFaces );
    if( eSweepStat != SM_SUCCESS )
    {
        // Do not hand back a half-built Brep. The kernel works on copies of
        // the input curves, so the Brep owns nothing of the caller's.
        delete rpResult;
        rpResult = NULL;
        SER( eSweepStat );
    }

    if( pOptStartFaces ) *pOptStartFaces = sStartFaces;
    if( pOptSideFaces  ) *pOptSideFaces  = sSideFaces;
    if( pOptEndFaces   ) *pOptEndFaces   = sEndFaces;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

    return SM_SUCCESS;

} // SmApiCreateCurveSweepFromEdges

/*******************************************************************//**
PURPOSE --- Taper extrude with offset corner type control

NOTES ---   Wraps SmPrimitiveCreation::CreateTaperExtrude
***********************************************************************/
SmApiStatus SmApiCreateTaperExtrude
(
    const SmTArray<SmCurve*>& rCurves,
    double dHeight,
    double dDraftAngleDeg,
    int iEndCaps,
    SmBoolean bFilletCorner,
    SmBrep*& rpResult
)
{
    rpResult = NULL;
    if( rCurves.GetSize() < 1 )
        return SM_ERR_INVALID_INPUT;

    SmContext* pContext = SmApiGetOrCreateContext();

    // Sweep copies of the inputs: CreateTaperExtrude consumes the curves.
    SmTArray<SmCurve*> sCurveCopies;
    SER( SmApiCopyCurvesForConsumingSweep( *pContext, rCurves, sCurveCopies ) );

    SmBrep* pBrep = new (*pContext) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmVector3d sNormal(0,0,0);
    SmOffsetCornerType eCorner = bFilletCorner ? SM_OC_FILLET_CORNER : SM_OC_LINEAR_EXTENSION;

    SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
    SER( sPC.CreateTaperExtrude( sCurveCopies, dHeight, dDraftAngleDeg, iEndCaps, sNormal, eCorner ) );

    sBrepCleanup.Clear();
    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    return SM_SUCCESS;

} // SmApiCreateTaperExtrude

/*******************************************************************//**
PURPOSE --- Linear sweep with repetitions (repeat end-to-end)

NOTES ---   Exposes nRepetitions parameter from SmPrimitiveCreation::CreateLinearSweep
***********************************************************************/
SmApiStatus SmApiCreateLinearSweepWithRepetitions
(
    const SmTArray<SmCurve*>& rCurvesToSweep,
    const SmVector3d& rSweepDir,
    double dSweepDist,
    ULONG nRepetitions,
    SmBoolean bCapEnds,
    SmBrep*& rpResult
)
{
    rpResult = NULL;
    if( rCurvesToSweep.GetSize() < 1 )
        return SM_ERR_INVALID_INPUT;

    SmContext* pContext = SmApiGetOrCreateContext();

    // Sweep copies of the inputs: CreateLinearSweep consumes the curves.
    SmTArray<SmCurve*> sCurveCopies;
    SER( SmApiCopyCurvesForConsumingSweep( *pContext, rCurvesToSweep, sCurveCopies ) );

    SmBrep* pBrep = new (*pContext) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
    SER( sPC.CreateLinearSweep( sCurveCopies, rSweepDir, dSweepDist, nRepetitions, bCapEnds ) );

    sBrepCleanup.Clear();
    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    return SM_SUCCESS;

} // SmApiCreateLinearSweepWithRepetitions

/*******************************************************************//**
PURPOSE --- Rotational sweep with repetitions (repeat end-to-end)

NOTES ---   Exposes nRepetitions parameter from SmPrimitiveCreation::CreateRotationalSweep
***********************************************************************/
SmApiStatus SmApiCreateRotationalSweepWithRepetitions
(
    const SmTArray<SmCurve*>& rCurvesToSweep,
    const SmPoint3d& rBasePt,
    const SmVector3d& rAxis,
    double dAngleDeg,
    ULONG nRepetitions,
    SmBoolean bCapEnds,
    SmBrep*& rpResult
)
{
    rpResult = NULL;
    if( rCurvesToSweep.GetSize() < 1 )
        return SM_ERR_INVALID_INPUT;

    SmContext* pContext = SmApiGetOrCreateContext();

    // Sweep copies of the inputs: CreateRotationalSweep consumes the curves.
    SmTArray<SmCurve*> sCurveCopies;
    SER( SmApiCopyCurvesForConsumingSweep( *pContext, rCurvesToSweep, sCurveCopies ) );

    SmBrep* pBrep = new (*pContext) SmBrep();
    SmObjDelete sBrepCleanup( pBrep );

    SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
    SER( sPC.CreateRotationalSweep( sCurveCopies, rBasePt, rAxis, dAngleDeg, nRepetitions, bCapEnds ) );

    sBrepCleanup.Clear();
    rpResult = pBrep;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    return SM_SUCCESS;

} // SmApiCreateRotationalSweepWithRepetitions

/*******************************************************************//**
PURPOSE --- Verify a non-manifold sweep selection references only live topology
            owned by the Brep being swept

NOTES ---   A null entry is dereferenced unconditionally by the sweep helpers, an
            entry owned by another Brep is swept into pBrepToSweep and silently
            corrupts it, and a handle left over from an earlier mutation is freed
            memory. Reject all three before anything is touched.

            IsLiveTopologyMember, not GetBrep(): it compares pointer identity
            against the Brep's live topology lists and never dereferences the
            candidate, so it is safe on a stale or already-freed handle. Testing
            pItem->GetBrep() would take the use-after-free path this guard exists
            to prevent. It also returns FALSE for null, covering that case too.
***********************************************************************/
template<class TOPOLOGY>
static SmBoolean smApiSweepSelectionIsOwnedBy( const SmTArray<TOPOLOGY*>* pSelection, const SmBrep* pBrepToSweep )
{
    if( !pSelection )
        return TRUE;

    for( ULONG ii = 0; ii < pSelection->GetSize(); ++ii )
    {
        if( !pBrepToSweep->IsLiveTopologyMember( (*pSelection)[ii] ) )
            return FALSE;
    }

    return TRUE;

} // smApiSweepSelectionIsOwnedBy

/*******************************************************************//**
PURPOSE --- Non-manifold translational sweep of faces/edges/vertices

NOTES ---   Wraps SmTopologySweep::DoSweep with SmTranslationalSweepGeometry
***********************************************************************/
SmApiStatus SmApiNonManifoldSweep
(
    SmBrep* pBrepToSweep,
    SmBrep* pRegionBrep,
    const SmVector3d& rSweepDir,
    double dSweepDist,
    SmBoolean bDoStitching,
    SmBoolean bDoMerge,
    SmTArray<SmFace*>* pOptFaces,
    SmTArray<SmEdge*>* pOptEdges,
    SmTArray<SmVertex*>* pOptVertices
)
{
    if( !pBrepToSweep )
        return SM_ERR_INVALID_INPUT;

    // A distinct target Brep splits the operation: the sweep helpers build the new
    // topology in pRegionBrep while the stitch/orient pass, the validation below and
    // the Python return value all use pBrepToSweep, so the call reports success with
    // the result in neither Brep coherently. Reject it here rather than expose that.
    // SmTopologySweep::DoSweep still offers the capability to kernel callers.
    if( pRegionBrep && pRegionBrep != pBrepToSweep )
        return SM_ERR_INVALID_INPUT;

    // A null, foreign or stale handle crashes or silently corrupts pBrepToSweep
    // below. These are rejected caller arguments, like a distinct pRegionBrep.
    if(    !smApiSweepSelectionIsOwnedBy( pOptFaces,    pBrepToSweep )
        || !smApiSweepSelectionIsOwnedBy( pOptEdges,    pBrepToSweep )
        || !smApiSweepSelectionIsOwnedBy( pOptVertices, pBrepToSweep ) )
        return SM_ERR_INVALID_INPUT;

    SmVector3d sSweepVec = rSweepDir;
    if( sSweepVec.Unitize() != SM_SUCCESS )
        return SM_ERR_INVALID_INPUT;
    sSweepVec = sSweepVec * dSweepDist;

    SmTranslationalSweepGeometry sSweepGeom( sSweepVec );
    SmTopologySweep sSweep( sSweepGeom, bDoStitching, FALSE, FALSE, bDoMerge );

    SmRegion* pRegion = pBrepToSweep->GetInfiniteRegion();

    SER( sSweep.DoSweep( pBrepToSweep, pRegion,
                         pOptFaces, pOptEdges, pOptVertices,
                         NULL, NULL, NULL ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrepToSweep );
#endif

    return SM_SUCCESS;

} // SmApiNonManifoldSweep

/*******************************************************************//**
PURPOSE --- Non-manifold rotational sweep of faces/edges/vertices

NOTES ---   Wraps SmTopologySweep::DoSweep with SmRotationalSweepGeometry
***********************************************************************/
SmApiStatus SmApiNonManifoldRotationalSweep
(
    SmBrep* pBrepToSweep,
    SmBrep* pRegionBrep,
    const SmPoint3d& rBasePt,
    const SmVector3d& rAxis,
    double dAngleDeg,
    SmBoolean bDoStitching,
    SmBoolean bDoMerge,
    SmTArray<SmFace*>* pOptFaces,
    SmTArray<SmEdge*>* pOptEdges,
    SmTArray<SmVertex*>* pOptVertices
)
{
    if( !pBrepToSweep )
        return SM_ERR_INVALID_INPUT;

    // A distinct target Brep splits the operation: the sweep helpers build the new
    // topology in pRegionBrep while the stitch/orient pass, the validation below and
    // the Python return value all use pBrepToSweep, so the call reports success with
    // the result in neither Brep coherently. Reject it here rather than expose that.
    // SmTopologySweep::DoSweep still offers the capability to kernel callers.
    if( pRegionBrep && pRegionBrep != pBrepToSweep )
        return SM_ERR_INVALID_INPUT;

    // A null, foreign or stale handle crashes or silently corrupts pBrepToSweep
    // below. These are rejected caller arguments, like a distinct pRegionBrep.
    if(    !smApiSweepSelectionIsOwnedBy( pOptFaces,    pBrepToSweep )
        || !smApiSweepSelectionIsOwnedBy( pOptEdges,    pBrepToSweep )
        || !smApiSweepSelectionIsOwnedBy( pOptVertices, pBrepToSweep ) )
        return SM_ERR_INVALID_INPUT;

    SmRotationalSweepGeometry sSweepGeom( rBasePt, rAxis, dAngleDeg, SM_ZONE_TOL_3D );
    SmTopologySweep sSweep( sSweepGeom, bDoStitching, FALSE, FALSE, bDoMerge );

    SmRegion* pRegion = pBrepToSweep->GetInfiniteRegion();

    SER( sSweep.DoSweep( pBrepToSweep, pRegion,
                         pOptFaces, pOptEdges, pOptVertices,
                         NULL, NULL, NULL ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrepToSweep );
#endif

    return SM_SUCCESS;

} // SmApiNonManifoldRotationalSweep

/*******************************************************************//**
PURPOSE --- Boolean with full SmMergeOptions control

NOTES ---   Wraps SmMerge with SmMergeOptions for cookie cutter, imprinting, etc.
***********************************************************************/
SmApiStatus SmApiBooleanWithOptions
(
    SmBrep* pBrep1,
    SmBrep* pBrep2,
    SmBooleanOperationType eOperation,
    SmBoolean bCookieCutter,
    SmBoolean bImprinting,
    SmBoolean bBooleanPostProcess,
    SmBoolean bImprintAndClassifyFaces,
    SmBoolean bKeepOtherBrep,
    SmBrep*& rpResult
)
{
    SER( SmApiValidateBooleanOperands( pBrep1, pBrep2, eOperation, rpResult ) );
    const SmContext* pContext = SmApiGetOrCreateContext();
    SmMergeOptions sOptions( bCookieCutter, bImprinting, bBooleanPostProcess,
                             bImprintAndClassifyFaces, bKeepOtherBrep );

    SmMerge sMerge( *pContext, pBrep1, pBrep2, 0, 0, &sOptions );

    if( pBrep1->IsManifoldSolid() && pBrep2->IsManifoldSolid() )
    {
        SER( sMerge.ManifoldBoolean( eOperation, rpResult ) );
    }
    else
    {
        SER( sMerge.NonManifoldBoolean( eOperation, rpResult ) );
    }

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

    return SM_SUCCESS;

} // SmApiBooleanWithOptions

/*******************************************************************//**
PURPOSE --- Non-manifold boolean operation

NOTES ---   Directly calls SmMerge::NonManifoldBoolean
***********************************************************************/
SmApiStatus SmApiNonManifoldBoolean
(
    SmBrep* pBrep1,
    SmBrep* pBrep2,
    SmBooleanOperationType eOperation,
    SmBrep*& rpResult
)
{
    SER( SmApiValidateBooleanOperands( pBrep1, pBrep2, eOperation, rpResult ) );
    const SmContext* pContext = SmApiGetOrCreateContext();
    SmMerge sMerge( *pContext, pBrep1, pBrep2 );
    SER( sMerge.NonManifoldBoolean( eOperation, rpResult ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

    return SM_SUCCESS;

} // SmApiNonManifoldBoolean

/*******************************************************************//**
PURPOSE --- Piecewise merge without deleting the other Brep

NOTES ---   Wraps SmMerge::PiecewiseMerge
***********************************************************************/
SmApiStatus SmApiPiecewiseMerge
(
    SmBrep* pBrep1,
    SmBrep* pBrep2,
    SmBoolean bCreateNewBrepsForFaces,
    SmBrep*& rpResult
)
{
    SER( SmApiValidateBooleanOperands( pBrep1, pBrep2, SM_BO_PARTIAL_MERGE, rpResult ) );
    const SmContext* pContext = SmApiGetOrCreateContext();
    SmMerge sMerge( *pContext, pBrep1, pBrep2 );
    SER( sMerge.PiecewiseMerge( NULL, FALSE, bCreateNewBrepsForFaces, rpResult ) );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif

    return SM_SUCCESS;

} // SmApiPiecewiseMerge

/*******************************************************************//**
PURPOSE --- Boolean between two lists of breps/surfaces

NOTES ---   Wraps SmMerge::BooleanLists
***********************************************************************/
SmApiStatus SmApiBooleanLists
(
    const SmTArray<SmBrep*>& rBreps1,
    const SmTArray<SmSurface*>& rSurfaces1,
    const SmTArray<SmBrep*>& rBreps2,
    const SmTArray<SmSurface*>& rSurfaces2,
    int operation,
    SmTArray<SmBrep*>& rResultBreps,
    SmTArray<SmSurface*>& rResultSurfaces
)
{
    SmContext* pContext = SmApiGetOrCreateContext();

    SER( SmMerge::BooleanLists( pContext, rBreps1, rSurfaces1, rBreps2, rSurfaces2,
                                operation, rResultBreps, rResultSurfaces ) );

    return SM_SUCCESS;

} // SmApiBooleanLists

/*******************************************************************//**
PURPOSE --- CSG tree boolean operations via postfix notation

NOTES ---   Wraps SmMerge::BooleanTrees
***********************************************************************/
SmApiStatus SmApiBooleanTrees
(
    SmTArray<SmBrep*>& rBreps,
    SmTArray<long>& rPostFixTrees
)
{
    SER( SmMerge::BooleanTrees( rBreps, rPostFixTrees ) );

    return SM_SUCCESS;

} // SmApiBooleanTrees

/*******************************************************************//**
PURPOSE --- Fillet specified edges with same radius and cross-section type

NOTES ---   Wraps SmFilletExecutive::SetFilletParameters + DoFillet
***********************************************************************/
SmApiStatus SmApiFilletEdges
(
    SmBrep* pBrep,
    SmTArray<SmEdge*>& rEdges,
    double dRadius,
    ULONG lXSectType,
    ULONG lContinuity,
    double dThumbweight
)
{
    if( !pBrep || rEdges.GetSize() < 1 )
        return SM_ERR_INVALID_INPUT;

    const SmContext* pContext = SmApiGetOrCreateContext();

    SmFilletSurfaceGeneratorType eFSGType = SM_FSG_CIRCULAR;
    if( lXSectType == 0 ) eFSGType = SM_FSG_LINEAR;
    else if( lXSectType == 2 ) eFSGType = SM_FSG_BLEND_CURVE;

    SmFilletExecutive sFilExec( *pContext, pBrep );
    sFilExec.SetDoGlobalMerge( TRUE );

    SER( sFilExec.SetFilletParameters( pBrep, rEdges,
                                       eFSGType, SM_FS_CONST_RADIUS,
                                       dRadius, NULL,
                                       -1.0, -1.0, -1.0, -1.0,
                                       lContinuity, dThumbweight ) );

    SER( sFilExec.DoFillet() );

    pBrep->StitchAndOrient();

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    return SM_SUCCESS;

} // SmApiFilletEdges

/*******************************************************************//**
PURPOSE --- Fillet edges with different radius/type per edge

NOTES ---   Wraps per-edge SetFilletParameters overload
***********************************************************************/
SmApiStatus SmApiFilletEdgesPerEdge
(
    SmBrep* pBrep,
    SmTArray<SmEdge*>& rEdges,
    SmTArray<double>& rRadii,
    SmTArray<SmFilletSurfaceGeneratorType>& rXSectTypes,
    ULONG lContinuity,
    double dThumbweight
)
{
    if( !pBrep || rEdges.GetSize() < 1 )
        return SM_ERR_INVALID_INPUT;

    if( rRadii.GetSize() != rEdges.GetSize() || rXSectTypes.GetSize() != rEdges.GetSize() )
        return SM_ERR_INVALID_INPUT;

    const SmContext* pContext = SmApiGetOrCreateContext();

    SmTArray<SmFilletSolverType> sRadiusTypes;
    for( ULONG i = 0; i < rEdges.GetSize(); i++ )
        sRadiusTypes.Add( SM_FS_CONST_RADIUS );

    SmFilletExecutive sFilExec( *pContext, pBrep );
    sFilExec.SetDoGlobalMerge( TRUE );

    SER( sFilExec.SetFilletParameters( pBrep, rEdges,
                                       rXSectTypes, sRadiusTypes,
                                       rRadii, NULL,
                                       -1.0, -1.0, -1.0, -1.0,
                                       lContinuity, dThumbweight ) );

    SER( sFilExec.DoFillet() );

    pBrep->StitchAndOrient();

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    return SM_SUCCESS;

} // SmApiFilletEdgesPerEdge

/*******************************************************************//**
PURPOSE --- Variable-radius fillet (linear law from start to end radius)

NOTES ---   Creates SmLinearFilletLaw and uses SetFilletParameters
***********************************************************************/
SmApiStatus SmApiVariableRadiusFillet
(
    SmBrep* pBrep,
    SmTArray<SmEdge*>& rEdges,
    double dStartRadius,
    double dEndRadius,
    ULONG lXSectType,
    ULONG lContinuity
)
{
    if( !pBrep || rEdges.GetSize() < 1 )
        return SM_ERR_INVALID_INPUT;

    const SmContext* pContext = SmApiGetOrCreateContext();

    SmFilletSurfaceGeneratorType eFSGType = SM_FSG_CIRCULAR;
    if( lXSectType == 0 ) eFSGType = SM_FSG_LINEAR;
    else if( lXSectType == 2 ) eFSGType = SM_FSG_BLEND_CURVE;

    SmLinearFilletLaw sLaw( dStartRadius, dEndRadius );

    SmFilletExecutive sFilExec( *pContext, pBrep );
    sFilExec.SetDoGlobalMerge( TRUE );

    SER( sFilExec.SetFilletParameters( pBrep, rEdges,
                                       eFSGType, SM_FS_VARIABLE_RADIUS,
                                       dStartRadius, &sLaw,
                                       -1.0, -1.0, -1.0, -1.0,
                                       lContinuity ) );

    SER( sFilExec.DoFillet() );

    pBrep->StitchAndOrient();

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    return SM_SUCCESS;

} // SmApiVariableRadiusFillet

/*******************************************************************//**
PURPOSE --- Fillet between two arbitrary surfaces

NOTES ---   Wraps SmFilletExecutive::SurfaceSurfaceFillet
***********************************************************************/
SmApiStatus SmApiSurfaceSurfaceFillet
(
    SmSurface* pSurface1,
    SmSurface* pSurface2,
    double dRadius1,
    double dRadius2,
    double dTolerance,
    SmBrep*& rpResult,
    ULONG lXSectType,
    SmBoolean bMirror,
    SmBoolean bComplement
)
{
    if( !pSurface1 || !pSurface2 )
        return SM_ERR_INVALID_INPUT;

    const SmContext* pContext = SmApiGetOrCreateContext();

    SmBrep* pTempBrep1 = NULL;
    SmBrep* pTempBrep2 = NULL;
    if( !pSurface1->GetFace() )
    {
        pTempBrep1 = new(*pContext) SmBrep();
        SmFace* pFace1 = NULL;
        SmStatus faceStat = pTempBrep1->CreateFaceFromSurface( pSurface1,
            pSurface1->GetNaturalUVDomain(), pFace1 );
        if( faceStat != SM_SUCCESS || pFace1 == NULL )
        {
            pSurface1->SetOwner( NULL );
            delete pTempBrep1;
            return faceStat != SM_SUCCESS ? faceStat : SM_ERR;
        }
    }
    if( !pSurface2->GetFace() )
    {
        pTempBrep2 = new(*pContext) SmBrep();
        SmFace* pFace2 = NULL;
        SmStatus faceStat = pTempBrep2->CreateFaceFromSurface( pSurface2,
            pSurface2->GetNaturalUVDomain(), pFace2 );
        if( faceStat != SM_SUCCESS || pFace2 == NULL )
        {
            if( pTempBrep1 ) { pSurface1->SetOwner( NULL ); delete pTempBrep1; }
            pSurface2->SetOwner( NULL );
            delete pTempBrep2;
            return faceStat != SM_SUCCESS ? faceStat : SM_ERR;
        }
    }

    SmFilletExecutive sFilExec( *pContext );
    SmStatus stat = sFilExec.SurfaceSurfaceFillet( *pContext, pSurface1, pSurface2,
                                        dRadius1, dRadius2, dTolerance,
                                        rpResult, lXSectType, 0.1,
                                        SM_BT_MAXIMAL, 0,
                                        bMirror, bComplement );

    if( pTempBrep1 )
    {
        pSurface1->SetOwner( NULL );
        delete pTempBrep1;
        pTempBrep1 = NULL;
    }
    if( pTempBrep2 )
    {
        pSurface2->SetOwner( NULL );
        delete pTempBrep2;
        pTempBrep2 = NULL;
    }

    if( stat != SM_SUCCESS )
        return stat;

#ifdef SM_ASSERT_VALID
    if( rpResult )
        SM_ASSERT_VALID( rpResult );
#endif

    return SM_SUCCESS;

} // SmApiSurfaceSurfaceFillet

/*******************************************************************//**
PURPOSE --- Get preview fillet surfaces without executing the fillet

NOTES ---   Sets up fillet then calls GetPreviewFilletSurfaces
***********************************************************************/
SmApiStatus SmApiFilletPreview
(
    SmBrep* pBrep,
    SmTArray<SmEdge*>& rEdges,
    double dRadius,
    SmTArray<SmSurface*>& rPreviewSurfaces
)
{
    if( !pBrep || rEdges.GetSize() < 1 )
        return SM_ERR_INVALID_INPUT;

    const SmContext* pContext = SmApiGetOrCreateContext();

    SmFilletExecutive sFilExec( *pContext, pBrep );

    SER( sFilExec.SetFilletParameters( pBrep, rEdges,
                                       SM_FSG_CIRCULAR, SM_FS_CONST_RADIUS,
                                       dRadius, NULL,
                                       -1.0, -1.0, -1.0, -1.0 ) );

    SER( sFilExec.GetPreviewFilletSurfaces( rPreviewSurfaces ) );

    return SM_SUCCESS;

} // SmApiFilletPreview

/*******************************************************************//**
PURPOSE --- Fillet edges with bevel corners at specified vertices

NOTES ---   Sets bevel on SmFilletCorner objects at requested vertices
***********************************************************************/
SmApiStatus SmApiSetBevelCorners
(
    SmBrep* pBrep,
    SmTArray<SmEdge*>& rEdges,
    double dRadius,
    SmTArray<SmVertex*>& rBevelVertices
)
{
    if( !pBrep || rEdges.GetSize() < 1 )
        return SM_ERR_INVALID_INPUT;

    const SmContext* pContext = SmApiGetOrCreateContext();

    SmFilletExecutive sFilExec( *pContext, pBrep );
    sFilExec.SetDoGlobalMerge( TRUE );

    SER( sFilExec.SetFilletParameters( pBrep, rEdges,
                                       SM_FSG_CIRCULAR, SM_FS_CONST_RADIUS,
                                       dRadius, NULL,
                                       -1.0, -1.0, -1.0, -1.0 ) );

    SER( sFilExec.CreateFilletCorners() );

    for( ULONG i = 0; i < rBevelVertices.GetSize(); i++ )
    {
        SER( sFilExec.SetBevelCorner( rBevelVertices[i] ) );
    }

    SER( sFilExec.DoFilleting() );

    pBrep->StitchAndOrient();

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pBrep );
#endif

    return SM_SUCCESS;

} // SmApiSetBevelCorners

/*******************************************************************//**
PURPOSE --- Full ShellBrep with face selection and options

NOTES ---   Wraps the ownership-reporting SmOffsetExecutive::ShellBrep overload
***********************************************************************/
SmApiStatus SmApiShellBrepFull
(
    const SmBrep* pBrep,
    double dOffsetDistance,
    SmBoolean bDoExtendedOffset,
    SmBoolean bDoSelfInt,
    SmBoolean bCreateOffsetSolid,
    const SmTArray<const SmFace*>& rFacesToShell,
    SmBrep*& rpResult
)
{
    rpResult = NULL;

    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    const SmContext* pContext = SmApiGetOrCreateContext();

    // The kernel shell operation consumes, returns, or retains its input on
    // different paths.  The public API is pure, so validate the selected
    // faces before copying and run the kernel only on private working data.
    SmTArray<SmFace*> sSourceFaces;
    pBrep->GetFaces( sSourceFaces );
    for( ULONG ii = 0; ii < rFacesToShell.GetSize(); ii++ )
    {
        const SmFace* pFace = rFacesToShell[ii];
        SmBoolean bFaceIsFromSource = FALSE;
        for( ULONG jj = 0; jj < sSourceFaces.GetSize(); jj++ )
        {
            if( sSourceFaces[jj] == pFace )
            {
                bFaceIsFromSource = TRUE;
                break;
            }
        }
        if( pFace == NULL || !bFaceIsFromSource )
            return SM_ERR_INVALID_INPUT;
    }

    SmCopyBrepMap sOriginalToCopyMap;
    SmBrep* pWorkingBrep = new (*pContext) SmBrep( *pBrep, NULL, &sOriginalToCopyMap );
    if( pWorkingBrep == NULL )
        return SM_ERR;
    SmObjDelete sWorkingBrepCleanup( pWorkingBrep );

    SmTArray<SmFace*> sWorkingFaces;
    for( ULONG ii = 0; ii < rFacesToShell.GetSize(); ii++ )
    {
        // SmCopyBrepMap's read-only lookup retains a legacy mutable key type.
        SmTopology* pMappedTopology = sOriginalToCopyMap.GetAt(
            SM_CONST_CAST(SmFace*, rFacesToShell[ii]) );
        if( pMappedTopology == NULL || !pMappedTopology->IsKindOf( SmFace_TYPE ) )
            return SM_ERR;
        sWorkingFaces.Add( (SmFace*)pMappedTopology );
    }

    SmBrep* pWorkingResult = NULL;
    SmBoolean bInputOwnershipTransferred = FALSE;
    SmStatus eStat = SmOffsetExecutive::ShellBrep(
        *pContext, pWorkingBrep, dOffsetDistance,
        bDoExtendedOffset, bDoSelfInt, bCreateOffsetSolid,
        sWorkingFaces, pWorkingResult, bInputOwnershipTransferred );

    if( bInputOwnershipTransferred || pWorkingResult == pWorkingBrep )
        sWorkingBrepCleanup.Clear();

    if( eStat != SM_SUCCESS )
        { SER( eStat ); }

    if( pWorkingResult == NULL )
        return SM_ERR;

    rpResult = pWorkingResult;

#ifdef SM_ASSERT_VALID
    if( rpResult )
        SM_ASSERT_VALID( rpResult );
#endif

    return SM_SUCCESS;

} // SmApiShellBrepFull

/*******************************************************************//**
PURPOSE --- Full OffsetBrep with extended offset and self-intersection options

NOTES ---   Wraps SmOffsetExecutive::OffsetBrep
***********************************************************************/
SmApiStatus SmApiOffsetBrepFull
(
    SmBrep* pBrep,
    double dOffsetDistance,
    SmBoolean bDoExtendedOffset,
    SmBoolean bDoSelfInt,
    SmBrep*& rpResult
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    const SmContext* pContext = SmApiGetOrCreateContext();

    SER( SmOffsetExecutive::OffsetBrep( *pContext, pBrep, dOffsetDistance,
                                        bDoExtendedOffset, bDoSelfInt,
                                        rpResult ) );

#ifdef SM_ASSERT_VALID
    if( rpResult )
        SM_ASSERT_VALID( rpResult );
#endif

    return SM_SUCCESS;

} // SmApiOffsetBrepFull

/*******************************************************************//**
PURPOSE --- Goal-directed stitching into a solid

NOTES ---   Wraps SmStitch::StitchIntoSolid
***********************************************************************/
SmApiStatus SmApiStitchIntoSolid
(
    SmBrep* pBrep,
    SmBoolean& rbProducesASolid,
    ULONG& rlStitchedEdges,
    double& rdMaxVertexGap,
    double& rdMaxEdgeGap
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    SER( SmStitch::StitchIntoSolid( pBrep, rbProducesASolid,
                                    rlStitchedEdges, rdMaxVertexGap, rdMaxEdgeGap ) );

    return SM_SUCCESS;

} // SmApiStitchIntoSolid

/*******************************************************************//**
PURPOSE --- Goal-directed stitching into a shell

NOTES ---   Wraps SmStitch::StitchIntoShell
***********************************************************************/
SmApiStatus SmApiStitchIntoShell
(
    SmBrep* pBrep,
    SmBoolean& rbShellIsWellFormed,
    double dMaxStitchingRatio,
    ULONG& rlStitchedEdges,
    double& rdMaxVertexGap,
    double& rdMaxEdgeGap
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    SER( SmStitch::StitchIntoShell( pBrep, rbShellIsWellFormed, dMaxStitchingRatio,
                                    rlStitchedEdges, rdMaxVertexGap, rdMaxEdgeGap ) );

    return SM_SUCCESS;

} // SmApiStitchIntoShell

/*******************************************************************//**
PURPOSE --- Orient face normals consistently

NOTES ---   Wraps SmStitch::UnifyNormals
***********************************************************************/
SmApiStatus SmApiUnifyNormals
(
    SmBrep* pBrep,
    SmFace* pStartFace,
    ULONG lNumSamples,
    SmTArray<SmFace*>& rFlippedFaces
)
{
    if( !pBrep || !pStartFace )
        return SM_ERR_INVALID_INPUT;

    double dTol = pBrep->GetTolerance();
    SmStitchCallback sCallback;
    SmStitch sStitch( sCallback, dTol );

    SER( sStitch.UnifyNormals( pBrep, pStartFace, lNumSamples, rFlippedFaces ) );

    return SM_SUCCESS;

} // SmApiUnifyNormals

/*******************************************************************//**
PURPOSE --- Face-pair stitching

NOTES ---   Wraps SmStitch::DoSimpleFaceStitch
***********************************************************************/
SmApiStatus SmApiSimpleFaceStitch
(
    SmBrep* pBrep,
    const SmTArray<SmFace*>& rFacesToKeep,
    const SmTArray<SmFace*>& rFacesToDelete,
    ULONG& rlStitchedEdges,
    double& rdMaxVertexGap,
    double& rdMaxEdgeGap
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    double dTol = pBrep->GetTolerance();
    SmStitchCallback sCallback;
    SmStitch sStitch( sCallback, dTol );

    SER( sStitch.DoSimpleFaceStitch( pBrep, rFacesToKeep, rFacesToDelete,
                                     rdMaxVertexGap, rdMaxEdgeGap, rlStitchedEdges ) );

    return SM_SUCCESS;

} // SmApiSimpleFaceStitch

/*******************************************************************//**
PURPOSE --- Advanced stitch with all SmStitch options exposed

NOTES ---   squeeze small edges, split edges at vertices, laminar sliver
            removal, manifold solid mode control
***********************************************************************/
SmApiStatus SmApiStitchAdvanced
(
    SmBrep* pBrep,
    double dStitchTol3d,
    SmBoolean bSqueezeSmallEdges,
    SmBoolean bSplitEdgesWithVertices,
    SmBoolean bMakingManifoldSolid,
    SmBoolean bRemoveLaminarSlivers,
    ULONG& rlStitchedEdges,
    ULONG& rlLaminaEdges,
    double& rdMaxVertexGap,
    double& rdMaxEdgeGap
)
{
    if( !pBrep )
        return SM_ERR_INVALID_INPUT;

    SmStitchCallback sCallback;
    SmStitch sStitch( sCallback, dStitchTol3d );

    sStitch.m_bSqueezeSmallEdges      = bSqueezeSmallEdges;
    sStitch.m_bSplitEdgesWithVertices = bSplitEdgesWithVertices;
    sStitch.m_bMakingManifoldSolid    = bMakingManifoldSolid;
    sStitch.m_bRemoveLaminarSlivers   = bRemoveLaminarSlivers;

    SER( sStitch.DoStitching( pBrep, NULL, NULL,
                              rlStitchedEdges, rlLaminaEdges,
                              rdMaxVertexGap, rdMaxEdgeGap ) );

    return SM_SUCCESS;

} // SmApiStitchAdvanced


/*******************************************************************//**
PURPOSE --- Replace analytic / periodic surfaces with NURBS in place.

NOTES   --- No-op on already-NURBS surfaces. Analytic surfaces, lines and
            circles are copied exactly; offset surfaces and other edge
            curves are approximated. Face UV trim curves are removed.
***********************************************************************/
SmApiStatus SmApiTurnToNurbs
(
    SmBrep*  pBrep                         ///< [in/out]: Pointer to brep                                              <br>
)
{
    if( pBrep == NULL )
        return SM_ERR_INVALID_INPUT;

    SmStatus stat = pBrep->TurnToNURBS();

#ifdef SM_ASSERT_VALID
    if( stat == SM_SUCCESS )
        SM_ASSERT_VALID( pBrep );
#endif // SM_ASSERT_VALID

    return stat;

} // SmApiTurnToNurbs
