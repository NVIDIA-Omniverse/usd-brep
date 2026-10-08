// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmRegion.cpp
* PURPOSE: Source file for SmRegion class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmRegion.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMSHELL_H__
#include <SmShell.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#include <SmGraphicsOutput.h>
#include <SmAssertArray.h>

//    // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//    #ifdef SM_DEBUG_CODE                  
//    
//    SmRegion * dbgRegion1    = NULL ;
//    SmRegion * dbgRegion2    = NULL ;
//    
//    #endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Calculate the bounding box for this region.

NOTES: 
  The result is the union of the boxes of all the shells.
  So the bounding box of the infinite region is not infinite.
***********************************************************************/
SmStatus SmRegion::CalculateBoundingBox
 ( SmExtent3d & rBBox,       // out: Bounding Box containing region
   SmBoolean    bTight)      // in : TRUE = compute minimal box for each contained face and edge(expensive)      
                             //      FALSE= compute any box larger than each contained face and edge (cheaper)
                             //      default:[FALSE]
 const
{
  rBBox.Init();

  SmTArray<SmShell*> sShells;
  GetShells( sShells );

  SmExtent3d sThisBox;
  ULONG i;
  ULONG lNumShells = sShells.GetSize();
  if (lNumShells < 1) 
      return (SM_ERR);
  for ( i = 0; i < lNumShells; i++ )
    {
      SER( sShells[i]->CalculateBoundingBox( sThisBox, bTight ) );
      rBBox.Union( sThisBox, rBBox );
    }

  return SM_SUCCESS;

} // end SmRegion::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Compute the area, volume, center of gravity and moments
    of a solid volume using analytical techniques.

NOTES: 
***********************************************************************/
SmStatus SmRegion::ComputePreciseProperties
  (double dRelativeAccuracy,               // Relative accuracy between
                                           // 1.0e-1 and 1.0e-4 appears to be the best.  You can go
                                           // tighter but it gets really slow if you have complex 
                                           // trim curves.  
   const SmPoint3d & crOriginOfComputation,
   double dEstimatedArea,                  // If not given then the area
                                           // will be estimated by a bounding box of the SmBrep
   double & rdArea,                        
   double & rdVolume,                      
   SmTArray<SmVector3d> & rMoments)        // [0] Ix, Iy, Iz    - Area static (first) moments:       integral of (x, y, z) dA
                                           // [1] Ixx, Iyy, Izz - Area second moments about planes:   integral of (x^2, y^2, z^2) dA
                                           // [2] Iyz, Izx, Ixy - Area products of inertia:           integral of (yz, zx, xy) dA
                                           // [3] Ixx, Iyy, Izz - Area moments of inertia about axes: integral of (y^2+z^2, x^2+z^2, x^2+y^2) dA
                                          // [4] Ix, Iy, Iz    - Volume static (first) moments:      integral of (x, y, z) dV
                                          // [5] Ixx, Iyy, Izz - Volume second moments about planes: integral of (x^2, y^2, z^2) dV
                                          // [6] Iyz, Izx, Ixy - Volume products of inertia:         integral of (yz, zx, xy) dV
                                          // [7] Ixx, Iyy, Izz - Volume moments of inertia about axes: integral of (y^2+z^2, x^2+z^2, x^2+y^2) dV
                                          // ([4..7] are geometric dV integrals; density weighting (dm) is applied only by the SmBrep layer.)
 const
{   
    ULONG i, j, k, kk;
    ULONG lNumProps = rMoments.GetSize();

    // Init outputs.
    rdArea   = 0.0;
    rdVolume = 0.0;
    for ( i=0; i<lNumProps; i++ ) {
        rMoments[i].Set( 0,0,0 );
    }

    double dDeltaArea, dDeltaVolume;
    SmTArray<SmVector3d> sDeltaMoments;
    sDeltaMoments.SetSize( lNumProps );

    if (dEstimatedArea < SM_EFF_ZERO) {
        SmBrep *pBrep = GetBrep();
        SmExtent3d sBBox;
        SER(pBrep->CalculateBoundingBox(sBBox));
        // Compute an estimated volume and area from the bounding box
        SmVector3d sBoxSize = sBBox.GetSize();
        dEstimatedArea = smos_Max(sBoxSize.z,smos_Max(sBoxSize.x,sBoxSize.y));
        dEstimatedArea = dEstimatedArea * dEstimatedArea / 10.0;
    }

    SmTArray<SmShell*> sShells;
    SmTArray<SmFaceuse*> sFaceuses;
    GetShells(sShells);

    for ( j=0; j<sShells.GetSize(); j++) {
        SmShell *pS = sShells[j];
        pS->GetFaceuses(sFaceuses);
        for ( k=0; k<sFaceuses.GetSize(); k++) { // For debugging
            SmFaceuse *pFU = sFaceuses[k];
            SmFace *pF = pFU->GetFace();
            if (pFU->GetMate()->GetShell()->GetRegion() == this) {
                if (pFU != pF->GetUpwardFaceuse()) {
                    continue;
                }
            }
            SER( pF->ComputePreciseProperties( pFU->GetOrientation(), dRelativeAccuracy,
                 crOriginOfComputation, dEstimatedArea, 0.0, dDeltaArea, dDeltaVolume,
                 sDeltaMoments ));

            rdArea   += dDeltaArea;
            rdVolume += dDeltaVolume;
            for ( kk=0; kk<lNumProps; kk++) {
                rMoments[kk] += sDeltaMoments[kk];
            }
        }
    }

    return SM_SUCCESS;

} // end SmRegion::ComputePreciseProperties

/*******************************************************************//**
PURPOSE: Compute the area, volume, center of gravity and moments
    of a solid volume using a tessellated approximation to the region.

NOTES: 
  See SmFace::ComputeProperties() for more details.

  Input: eWhichProps: which properties to evaluate.  Possible values are:
    SM_MPT_AREA
    SM_MPT_VOLUME
    SM_MPT_CENTROID
    SM_MPT_MOMENTS
    SM_MPT_ALL.
    These are sequenctial, in this order: specifying any one causes
    that one and all earlier ones to be calculated.
    Note SM_MPT_MOMENTS is (currently) the same as SM_MPT_ALL.
    default:[SM_MPT_ALL].

   Output:
     aMoments[2]  The following moments are currently supported:
                  aMoments[0] Contains Ixx, Iyy, Izz
                  aMoments[1] Contains Ixy, Iyz, Izx

***********************************************************************/
SmStatus SmRegion::ComputeProperties
  (double dEdgeTessTol,
   double dFaceTessTol,
   const SmPoint3d & crOriginOfComputation,
   double & rdArea,
   double & rdVolume,
   SmPoint3d & rBarycenter,
   SmVector3d aMoments[2],           // out: see Usage Notes, above.
   SmMassPropertiesType eWhichProps) // in:  see Usage Notes, above.
  const
{
  ULONG i, j;

  // Init outputs.
  rdArea = rdVolume = 0.0;
  rBarycenter.Set( 0,0,0 );
  aMoments[0].Set( 0,0,0 );
  aMoments[1].Set( 0,0,0 );

  SmTArray<SmShell*> sShells;
  SmTArray<SmFaceuse*> sFaceuses;
  GetShells(sShells);

  double dDeltaArea, dDeltaVolume;
  SmPoint3d sDeltaBarycenter;
  SmVector3d aDeltaMoments[2];

  for ( i=0; i<sShells.GetSize(); i++ )
  {
      SmShell *pS = sShells[i];
      pS->GetFaceuses( sFaceuses );

      for ( j=0; j<sFaceuses.GetSize(); j++ )
      {
          SmFaceuse *pFU = sFaceuses[j];
          SmFace *pF = pFU->GetFace();

          SER( pF->ComputeProperties( pFU->GetOrientation(),
                      dEdgeTessTol, dFaceTessTol,
                      crOriginOfComputation, dDeltaArea, dDeltaVolume,
                      sDeltaBarycenter, aDeltaMoments, eWhichProps ));

          rdArea      += dDeltaArea;
          rdVolume    += dDeltaVolume;
          rBarycenter += sDeltaBarycenter;
          aMoments[0] += aDeltaMoments[0];
          aMoments[1] += aDeltaMoments[1];
      }
  }

  rBarycenter = rBarycenter / rdVolume;

  return SM_SUCCESS;

} // end SmRegion::ComputeProperties

/*******************************************************************//**
PURPOSE: Collect all regions connected to this region through a face.

NOTES: 
***********************************************************************/
SmStatus SmRegion::GetAdjacentRegions( SmTArray< SmRegion* > & rRegions )
 const
{
  rRegions.ReSet();

  SmTArray< SmShell* > sOurShells, sShellMates;
  GetShells( sOurShells );
  ULONG ii, lNumShells = sOurShells.GetSize();

  for ( ii = 0; ii < lNumShells; ii++ )
    {
      SmShell * pShell = sOurShells[ii];
      if ( ! pShell->IsFaceuseShell() ) { continue; }

      SER( pShell->GetMatedShells( sShellMates )); // increments an unused Mark value

      ULONG jj, lNumMates = sShellMates.GetSize();
      for ( jj = 0; jj < lNumMates; jj++ )
        {
          SmRegion *pRegionMate = sShellMates[jj]->GetRegion();
          if ( pRegionMate != this )
            {
              rRegions.AddUnique( pRegionMate );
            }
        } // end for each of this Shell's mated Shells
    } // end for each of our Shells

    return SM_SUCCESS;

} // end GetAdjacentRegions

/*******************************************************************//**
PURPOSE: Return TRUE when all Region geometries fit within BBox less than given Tol

NOTES: 
***********************************************************************/
SmBoolean SmRegion::IsDegenerate // rtn: TRUE = Shell is degenerate        
 (double d3DTol)                 // in : min distance between distinct points,
                                 //      default:[SM_EFF_ZERO]
const 
{
  // locals
  SmExtent3d          sBBox ;

  // Do expensive check
  CalculateBoundingBox(sBBox) ;
  SmBoolean bRtn = sBBox.IsPointSized(d3DTol) ;

  // all done
  return(bRtn) ;

} // end SmRegion::IsDegenerate

/*******************************************************************//**
PURPOSE: Return TRUE when Region belongs to a Brep and is that Brep's
         Infinite Region

NOTES: 
***********************************************************************/
SmBoolean SmRegion::IsInfiniteRegion() // rtn: TRUE = this Region is its Brep's InfiniteRegion       
const 
{
  return(GetBrep() ? (this == GetBrep()->GetInfiniteRegion()) : FALSE) ;

} // end SmRegion::IsInfiniteRegion

/*******************************************************************//**
PURPOSE: Set the IsVoidFlag of a region.

NOTES: 
***********************************************************************/
void SmRegion::SetIsVoid
  (SmBoolean bIsVoidFlag)
{
    m_bIsVoidFlag = bIsVoidFlag;

} // end SmRegion::SetIsVoid

/*******************************************************************//**
PURPOSE:  Make sure our shells follow SMLib topology rules.

NOTES: 
  The rules are that, for any Region other than the infinite region,
  the first Shell in that Region must be the Shell that separates it
  from the Region that contains it.  This means that it must be a
  Faceuse-type shell.
***********************************************************************/
SmStatus SmRegion::OrderShells()
{
  SmTArray< SmShell* > sShells;
  this->GetShells( sShells );
  ULONG jj, lNumShells = sShells.GetSize();

  if ( lNumShells < 2 ) // If only one shell, no reordering.
    { return SM_SUCCESS; }

  if ( sShells[0]->IsFaceuseShell() ) // That's what it's supposed to be.
    { return SM_SUCCESS; }

  // Find all the Faceuse shells.
  // If more than one, check for a closed, inner one.
  SmTArray< SmShell* > sFaceuseShells;
  for (jj=0; jj<lNumShells; jj++)
    {
      SmShell *pS = sShells[jj];
      if ( pS->IsFaceuseShell()  )
        {
          sFaceuseShells.Add( pS );
        }
    } // end for each shell looking for Faceuse shells

  SmShell *pShellToUse = NULL;
  ULONG lNumFUShells = sFaceuseShells.GetSize();

  if ( lNumFUShells == 1 )
    {
      pShellToUse = sFaceuseShells[0];
    }
  else if ( lNumFUShells > 1 )
    {
      for ( jj=0; jj<lNumFUShells; jj++ )
        {
          SmShell *pS = sFaceuseShells[jj];
          if ( pS->IsClosed() )
            {
              SmBoolean bIsInner = FALSE;
              SmStatus eStat = pS->IsInnerShell( bIsInner );
              if ( eStat == SM_SUCCESS && bIsInner == TRUE )
                {
                  pShellToUse = pS;
                  break;
                }
            }
        }  // end for each Faceuse-type shell

      // If we didn't find a closed inner one, just use the first.
      if ( pShellToUse == NULL )
        { pShellToUse = sFaceuseShells[0]; }

    } // end if more than one Faceuse-type shell

  if ( pShellToUse != NULL )
    {
      // Doubly-linked list, so we can just do this:
      this->m_pList = pShellToUse;
    }

  return SM_SUCCESS;

} // end OrderShells

/*******************************************************************//**
PURPOSE: Add graphics for every region->Shell to new or open
            DisplayList

NOTES:
***********************************************************************/
SmDisplayList * SmRegion::Draw
 (double          dNormalGain,   // in : Amount to scale the Normal vector, 0.0 = Guess from Shell BBox Size, default:[0.0]
  SmGfxArraySet * pOptGfxSet)    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
 const                           //      NULL to ignore, default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_OUTPUT_CODE
  // Get global SmGraphicsExtern.cpp:s_Disp display parameters
  const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;

  // start new displayList (unless one is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(this, rDisp.GetShadedColorRule()), NULL, NULL, FALSE, pOptGfxSet);

  SmTArray<SmShell*> sShells;
  GetShells(sShells);
  for (ULONG i=0; i<sShells.GetSize(); i++) 
    {
      sShells[i]->Draw(dNormalGain, pOptGfxSet);
    }

  // end displayList
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(dNormalGain, pOptGfxSet);
#endif // end SM_GFX_OUTPUT_CODE
  return(pRtn) ;

} // end SmRegion::Draw

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertRegion_list[] =
{
  /*  0 */ {SM_AT_NESTED_TEST, _T("Bad SubTopology"), _T("A Region SubTopology Shell, Face, Loop, Edge, or Vertex object failed its AssertValid checks") },
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmRegion::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  // init rtn value
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmOwningTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // todo - add SmRegion checks here

  // when asked - AssertValid for a topology graph traversal
  if(eWalkTree == SM_WALK)
    {
      bRtn &= SmBrep::AssertSubTopology( (SmTopology &)*this, pAList, eTestLevel) ;
      // bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (SmBrep::AssertSubTopology( (SmTopology &)*this, pAList, eTestLevel)), _T("")) ;

    } // end asked to walk the tree check

  // all done
  return(bRtn) ;

} // end SmRegion::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmRegion::AssertHeal
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
//       return ( SmOwningTopology::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmRegion::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmRegion::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmRegion::IsKindOf( SM_TYPE t ) const
{
  return ((SmRegion_TYPE == t) ? TRUE : SmOwningTopology::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmRegion::Dump
  (void) 
 const
{
  // Get the lists of things contained in this region
  SmTArray<SmShell*> sShells ;  GetShells(sShells) ;
  SmBrep *pBrep = GetBrep() ;

  // write region PointerValue and header data
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  SM_SPRINTF(sBuff, _T("Region[0x%p]: %s %s[0x%p] %s- m_pListLength:%ld (HeadShell)[0x%p]\n"), 
             this,
             m_bIsVoidFlag ? _T("IsVoid[TRUE ]") : _T("IsVoid[FALSE]"),
             pBrep ? _T("Brep's") : _T("Brep"),
             pBrep,
             (pBrep) ? ((pBrep->GetInfiniteRegion() == this) ? _T("InfiniteRegion    ") : _T("NotInfiniteRegion ")) : _T(""),
             m_lListSize, 
             m_pList);

  SM_SPRINTF(sBuffForFile, _T("Region[%s]: %s %s[%s] %s- m_pListLength:%ld (HeadShell)[%s]\n"), 
             _T("NotNULL"),
             m_bIsVoidFlag ? _T("IsVoid[TRUE ]") : _T("IsVoid[FALSE]"),
             pBrep ? _T("Brep's") : _T("Brep"),
             pBrep ? _T("VOID   ") : _T("NotVOID"),
             (pBrep) ? ((pBrep->GetInfiniteRegion() == this) ? _T("InfiniteRegion    ") : _T("NotInfiniteRegion ")) : _T(""),
             m_lListSize, 
             m_pList ? _T("VOID   ") : _T("NotVOID"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // list the shells (when there is more than one)
  if(sShells.GetSize() > 1)
    {
      smos_WriteBuffer(_T("    shells: ")) ;
      ULONG ii ;
      for(ii=0;ii<sShells.GetSize();ii++) 
        { 
          SM_SPRINTF(sBuff, _T("0x%p  "), sShells[ii]) ;
          smos_WriteBuffer(sBuff) ;
        }
      smos_WriteBuffer(_T("\n")) ;
    }

} // end SmRegion::Dump

/*******************************************************************//**
PURPOSE:  Pretty print pointer values for this Region showing how
             it connects to its neighbor Brep and Shells in the topology graph.

NOTES: Only good for debugging because pointer values don't
                stay constant from run to run.
***********************************************************************/
void SmRegion::DumpTopology
  (ULONG lWalkDepth) // in : 0 = no walking,  ... 99 = walk to bottom
 const
{
  // locals
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] = {};
  SmShell *pShell = (SmShell *)m_pList ;
  SmTArray<SmShell*> sShells ; GetShells(sShells) ; SM_ASSERT(sShells.GetSize() == m_lListSize) ;

  // output Brep connection
  SM_SPRINTF(sBuff,_T("  Region  [0x%p] -> Brep[0x%p] and has %ld shells\n"), this, m_pListOwner, m_lListSize) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // locals
  SmTArray<SmFaceuse *> sFaceuses, sNextFaceuses ;
  SmTArray<SmEdge *>    sWireEdges, sNextWireEdges ;
  // output every owned Shell
  for(ii=0;ii<m_lListSize;ii++,pShell=(SmShell *)pShell->m_pNext)
    {
      pShell->GetFaceuses(sFaceuses) ;
      pShell->GetWireEdges(sWireEdges) ;
      SmVertex *pVertex = pShell->GetVertex() ;

      ((SmShell *)pShell->m_pNext)->GetFaceuses(sNextFaceuses) ;  
      ((SmShell *)pShell->m_pNext)->GetWireEdges(sNextWireEdges) ;
      SmVertex *pNextVertex = ((SmShell *)pShell->m_pNext)->GetVertex() ;
 
      // check for pointer consistency as possible
      SM_ASSERT(pShell->m_pNext->m_pLast == pShell) ;
      SM_ASSERT((SmRegion *)(pShell->m_pListOwner) == this) ;
      SM_SPRINTF(sBuff,_T("    Region[0x%p] <- Shell[0x%p %ld %s] -> NextShell[0x%p %ld %s]\n"),
                 (SmRegion *)(pShell->m_pListOwner), 
                 pShell, 
                   sFaceuses.GetSize()  > 0 ? sFaceuses.GetSize()
                 : sWireEdges.GetSize() > 0 ? sWireEdges.GetSize()
                 : pVertex != NULL          ? 1
                 : 0,
                   sFaceuses.GetSize()  > 0 ? _T("Faceuses")
                 : sWireEdges.GetSize() > 0 ? _T("WireEdges")
                 : pVertex != NULL          ? _T("Vertex")
                 : _T("Objects"),
                 pShell->m_pNext,
                   sNextFaceuses.GetSize()  > 0 ? sNextFaceuses.GetSize()
                 : sNextWireEdges.GetSize() > 0 ? sNextWireEdges.GetSize()
                 : pNextVertex != NULL          ? 1
                 : 0,
                   sNextFaceuses.GetSize()  > 0 ? _T("Faceuses")
                 : sNextWireEdges.GetSize() > 0 ? _T("WireEdges")
                 : pNextVertex != NULL          ? _T("Vertex")
                 : _T("Objects")) ;

      smos_WriteBuffer(sBuff, sBuffForFile);
    }
    
  if(lWalkDepth > 0)
    {
      for(ii=0;ii<sShells.GetSize();ii++)
        {
          pShell = sShells[ii] ;
          if(pShell) pShell->DumpTopology(lWalkDepth == 99 ? lWalkDepth : lWalkDepth - 1) ;
        }
    } 

} // end SmRegion::DumpTopology
