// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCEdge.cpp
* PURPOSE: Source file for SmCEdge class methods.
**********************************************************************/

#include "StdAfx.h"
#include <SmCEdge.h>

// Remove Composites
//  
//  /*******************************************************************//**
//  PURPOSE: Constructor for the composite edge.  It creates a composite
//     from two regular edges.  Please note that a composite must have a
//     minimum of two edges to exist.  
//  
//  NOTES: 
//  ***********************************************************************/
//  SmCEdge::SmCEdge
//    (SmEdge *pEdge1, 
//     SmEdge *pEdge2)
//  {
//    SM_ASSERT(pEdge1 != NULL && pEdge2 != NULL);
//    SM_ASSERT(pEdge1->GetCurve() == pEdge2->GetCurve());
//    m_pEdges = NULL;
//    m_pBrep  = NULL;
//    m_pCurve = NULL;
//  
//  #ifdef SM_USE_NEWTOL
//    SM_NEWTOL_LINE m_sLocalZoneTol3d = (SM_UNDEF_DOUBLE) ;
//  #else // SM_USE_OLDTOL
//    SM_OLDTOL_LINE m_sZoneTol3d = (0.0) ;  
//  #endif // SM_USE_OLDTOL
//  
//    SmBrep * pBrep = pEdge1->GetBrep();
//    if (pBrep)
//    {
//        m_pEdges = new (*pBrep->GetContext()) SmTArray<SmEdge*> (*pBrep->GetContext());
//        m_pEdges->Add(pEdge1);
//        m_pEdges->Add(pEdge2);
//        m_pCurve = pEdge1->GetCurve();
//        m_pCurve->SetOwner(this);  // Composite now owns the surface.
//        pEdge1->m_vInterval.Union(pEdge2->m_vInterval,m_vInterval);
//  #ifdef SM_USE_NEWTOL
//        SM_NEWTOL_LINE m_sLocalZoneTol3d =   pEdge1->GetLocalZoneTol3d() == SM_UNDEF_DOUBLE ? pEdge2->GetLocalZoneTol3d()
//        SM_NEWTOL_LINE                        : pEdge2->GetLocalZoneTol3d() == SM_UNDEF_DOUBLE ? pEdge1->GetLocalZoneTol3d()
//        SM_NEWTOL_LINE                        : smos_Max(pEdge1->GetLocalZoneTol3d(), pEdge2->GetLocalZoneTol3d()) ;
//  #else // SM_USE_OLDTOL
//        SM_OLDTOL_LINE m_sZoneTol3d = smos_Max(pEdge1->m_sZoneTol3d, pEdge2->m_sZoneTol3d) ; 
//  #endif // SM_USE_OLDTOL
//        m_pBrep = pEdge1->GetBrep();
//    }
//  
//    SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL) ;
//  
//  } // end SmCEdge::SmCEdge constructor
//  
//  /*******************************************************************//**
//  PURPOSE: Destructor for the edge - it cleans up the curve
//  
//  NOTES: Commonly 1 edge (still part of a Brep) remains in m_pEdges,
//         don't delete it, just the m_pEdges array
//  ***********************************************************************/
//  SmCEdge::~SmCEdge() 
//  { 
//    Notify(SM_NO_DESTRUCTION, this, NULL, NULL) ;
//  
//    if (m_pEdges) { delete m_pEdges; m_pEdges = NULL ; }
//  
//  } // end SmCEdge::~SmCEdge destructor
//  
//  /*******************************************************************//**
//  PURPOSE: Add another face to the composite face.
//  
//  NOTES: 
//  ***********************************************************************/
//  SmStatus SmCEdge::AddEdge(SmEdge *pEdgeToAdd)
//  {
//      if(pEdgeToAdd->GetCurve() != m_pCurve) SER(SM_ERR);
//  
//      // Make sure edge is not already in there
//      for (ULONG i=0; i<m_pEdges->GetSize(); i++) {
//          if (pEdgeToAdd == (*m_pEdges)[i]) {
//              SER(SM_ERR);
//          }
//      }
//  
//      m_pEdges->Add(pEdgeToAdd);
//      m_vInterval.Union(pEdgeToAdd->m_vInterval, m_vInterval);
//  
//  #ifdef SM_USE_NEWTOL
//      SM_NEWTOL_LINE m_sLocalZoneTol3d =  pEdgeToAdd->GetLocalZoneTol3d() == SM_UNDEF_DOUBLE ? GetLocalZoneTol3d()
//      SM_NEWTOL_LINE                       \: GetLocalZoneTol3d()             == SM_UNDEF_DOUBLE ? pEdgeToAdd->GetLocalZoneTol3d()
//      SM_NEWTOL_LINE                       : smos_Max(pEdgeToAdd->GetLocalZoneTol3d(), GetLocalZoneTol3d()) ;
//  #else // SM_USE_OLDTOL
//      SM_OLDTOL_LINE m_sZoneTol3d = smos_Max(pEdgeToAdd->m_sZoneTol3d, m_sZoneTol3d) ; 
//  #endif // SM_USE_OLDTOL
//  
//      return SM_SUCCESS;
//  
//  } // end SmCEdge::AddEdge
//  
//  /*******************************************************************//**
//  PURPOSE: Get the brep which owns this cedge.
//  
//  NOTES: 
//  ***********************************************************************/
//  SmBrep* SmCEdge::GetBrep() const
//  {
//      if (m_pEdges->GetSize() == 0) 
//        {
//          SE(SM_ERR);
//          return NULL;
//        }
//      SmEdge *pE = m_pEdges->GetLast();
//      return pE->GetBrep();
//  
//  } // end SmCEdge::GetBrep
//  
//  
//  /*******************************************************************//**
//  PURPOSE: Get the underlying child edges of a composite edge.
//  
//  NOTES: does not add this CEdge to the output - just the children
//  ***********************************************************************/
//  void SmCEdge::GetEdges
//    (SmTArray<SmEdge*> & rEdges) 
//   const
//  {
//      rEdges.ReSet();
//      rEdges.Append(*m_pEdges);
//  
//  } // end SmCEdge::GetEdges
//      
//  /*******************************************************************//**
//  PURPOSE: Return TRUE when given edge is in composite Edge list
//  
//  NOTES: 
//  ***********************************************************************/
//  SmBoolean SmCEdge::IsEdgeInCEdge
//    (const SmEdge *pEdge)              // in : target edge to find
//   const
//  {
//    ULONG lFoundIndex ;
//    return( m_pEdges->FindElement((SmEdge *)pEdge, lFoundIndex)) ;
//  
//  } // end SmCEdge::IsEdgeInCEdge
//  
//  
//  /*******************************************************************//**
//  PURPOSE: Removes an Edge without deleting it from the composite.
//           Sets Edge->Curve ptr NULL so Curve is not used by two owners.
//           Deletes any Edge->AllEdgeuse->UVTrimCurves and sets those ptrs to NULL.
//  
//  NOTES: 1. After pEdgeToRemove is removed, if CEdge only contains one Edge,
//            that OtherEdge is also removed from the CEdge, and the CEdge object is deleted,
//            making the otherEdge another standalone Edge.
//  
//            This change does not have to be reported back to the caller
//            because the topology graph only points to Edges and not CEdges.
//            Deleting a CEdge when it becomes a single edge won't leave any stale
//            pointers behind.
//  ***********************************************************************/
//  SmStatus SmCEdge::RemoveEdge
//    (SmEdge *pEdgeToRemove)   // in : target Edge to Remove from CEdge list
//  {
//    ULONG ii, jj ; 
//  
//    // for every contained - edge
//    for(ii=0;ii<m_pEdges->GetSize();ii++) 
//      {
//        SmEdge * pEdge = (*m_pEdges)[ii] ;
//  
//        // when EdgeToRemove is found - remove it from the list
//        if(pEdge == pEdgeToRemove) 
//          {
//            m_pEdges->RemoveAt(ii,1);
//  
//            // when only 1 edge remains in CEdge - make that edge standalone and delete CEdge object
//            if(m_pEdges->GetSize() == 1) 
//              {
//                // change remaining edge so that it's a free standing Edge
//                SmEdge * pLastEdge = m_pEdges->GetLast();
//                m_pCurve->SetOwner(pLastEdge);
//  
//                // Remove this CEdge from Brep and delete it 
//                this->m_pBrep->DeleteCEdge(this);
//  
//              } // end down to 1 contained edge branch
//            else // else more than 1 contained edge branch
//              {
//                // Here we update the internal CEdge values
//                SmEdge * pFirstE   = (*m_pEdges)[0];
//                m_vInterval        = pFirstE->m_vInterval;
//  #ifdef SM_USE_NEWTOL
//                SM_NEWTOL_LINE m_sLocalZoneTol3d  =  pFirstE->GetLocalZoneTol3d() ;
//  #else // SM_USE_OLDTOL
//                SM_OLDTOL_LINE m_sZoneTol3d = pFirstE->GetTolerance() ; 
//  #endif // SM_USE_OLDTOL
//                
//                for(jj=1;jj<m_pEdges->GetSize();jj++) 
//                  {
//                    SmEdge *pE = (*m_pEdges)[jj];
//                    m_vInterval.Union(pE->m_vInterval,m_vInterval);
//  #ifdef SM_USE_NEWTOL
//                    SM_NEWTOL_LINE m_sLocalZoneTol3d =  pE->GetLocalZoneTol3d() == SM_UNDEF_DOUBLE ? GetLocalZoneTol3d()
//                    SM_NEWTOL_LINE                       : GetLocalZoneTol3d()     == SM_UNDEF_DOUBLE ? pE->GetLocalZoneTol3d()
//                    SM_NEWTOL_LINE                       : smos_Max(pE->GetLocalZoneTol3d(), GetLocalZoneTol3d()) ;
//  #else // SM_USE_OLDTOL
//                    SM_OLDTOL_LINE m_sZoneTol3d = smos_Max(pE->m_sZoneTol3d, m_sZoneTol3d) ; 
//  #endif // SM_USE_OLDTOL
//                  }
//              }  // end more than 1 contained edge branch
//  
//            // next: set pEdgeToRemove->Curve ptr NULL, Delete pEdgeToRemove->AllEdgeuses->UVTrimCurves,
//            //       so that pEdgeToRemove->Curve won't have two owners
//            pEdge->SetCurve(NULL, FALSE, FALSE) ; // FALSE = don't delete preExisting Curve, don't LeakWarn - it's being used by the surviving CEdge
//                                                  // side effect: delete all UVTrimCurves
//  
//            // set pEdgeToRemove->Curve ptr NULL
//            // all done
//            return SM_SUCCESS;
//  
//          } // end current edge is edgeToRemove check
//      } // end iter every contained edge
//  
//    SE(SM_ERR);
//    return SM_ERR;
//  
//  } // end SmCEdge::RemoveEdge
//  
//  /*********************************************************
//  PURPOSE: Set m_vInteval and those of each Edge according to dropping vertices to our curve.
//     To be used if the curve parameterization changes.
//  **********************************************************/
//  SmStatus SmCEdge::UpdateDomain()
//  {
//    SmExtent1d sNewDomain;
//  
//    ULONG ii, lNumEdges = m_pEdges->GetSize();
//    SmStatus eStat, eRetStat = SM_SUCCESS;
//  
//    for ( ii=0; ii<lNumEdges; ii++ )
//    {
//        SmEdge *pE = (*m_pEdges)[ii];
//        eStat = pE->UpdateDomain();
//        if ( eStat != SM_SUCCESS )
//          { eRetStat = eStat; }
//  
//        sNewDomain.Union( pE->GetInterval(), sNewDomain );
//    }
//  
//    m_vInterval = sNewDomain;
//  
//    return eRetStat;
//  
//  } // end SmCEdge::UpdateDomain
//      
//  /*******************************************************************//**
//  PURPOSE: Compute the total size of the memory used by the SmCEdge.
//  
//  NOTES:  Does not include any of the CEdge children Edge memory.
//  
//    uses without increment MarkType value
//  ***********************************************************************/
//  ULONG SmCEdge::GetMemoryUsed         // rtn: Total Memory being used for all objects
//    (ULONG     & rlMemoryAllocated,    // out: Total Memory allocated for all objects
//     SmMarkType  eMarkType)            // in : uses without increment eMarkType value
//   const
//  {
//    // in case this method is called directly - get a mark for attribute memory usage
//    SmNewMarkAndLock sMarkLock ;
//    if(eMarkType == SM_MT_NOMARK)
//      {
//        eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
//      }
//  
//    // locals
//    ULONG lThisAllocated ;
//  
//    // this + attribute memory
//    ULONG lUsed        =   sizeof(*this) + this->GetAttributeMemoryUsed(lThisAllocated, 
//                                                                        eMarkType) ;  // note: uses without increment eMarkType value
//  
//    rlMemoryAllocated  =   sizeof(*this) + lThisAllocated ; 
//  
//    // m_pEdges memory - but not any memory of the contained item Edges
//    if(m_pEdges)
//      {
//        lUsed             += m_pEdges->GetMemoryUsed(lThisAllocated) ;
//  
//        rlMemoryAllocated += lThisAllocated ; 
//      }
//  
//    // all done
//    return(lUsed) ;
//  
//  } // end SmCEdge::GetMemoryUsed
//  
//  /*******************************************************************//**
//  PURPOSE: AssertValid() Test reporting static labels
//  ***********************************************************************/
//  SmAssertReportLabel sAssertCEdge_list[] =
//  {
//    /*  0 */ {SM_AT_COINCIDENCE, _T("Bad member edge Intervals"), _T("SmCEdge member edge intervals overlap") }
//  } ;
//  
//  /*******************************************************************//**
//  PURPOSE:
//  
//  RETURNS ---  TRUE  = OK
//               FALSE = Problem
//  ***********************************************************************/
//  SmBoolean SmCEdge::AssertValid
//   (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
//    SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
//                                      //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
//                                      //      default:[SM_LEVEL_0] 
//    SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
//    SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
//   const
//  {
//    SM_REF1(eWalkTree) ;
//    // init rtn value
//    SmBoolean bRtn = TRUE;
//    
//    // call the base class AssertValid
//    bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
//             ? SmEdge::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
//             : TRUE ) ; 
//  
//    // locals
//    ULONG ii, jj ;
//  
//    // when m_pEdges exists - for every pair of member edges
//    if(m_pEdges)
//      {
//        for(ii=0;ii<m_pEdges->GetSize() && bRtn;ii++)
//          {
//            SmExtent1d sIvl1 = m_pEdges->GetAt(ii)->GetInterval() ;
//  
//            for(jj=ii+1;jj<m_pEdges->GetSize() && bRtn;jj++)
//              {
//                SmExtent1d sIvl2 = m_pEdges->GetAt(jj)->GetInterval() ;
//  
//                // see if Edge pairs overlap
//                SmBoolean bAreOverlapping = sIvl1.AreOverlapping(sIvl2, SM_EFF_ZERO) ;
//  
//                // Test 0: Edge pair intervals must not overlap
//                 bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, !bAreOverlapping, _T("")) ;
//  
//              } // end iter jj, every m_pEdges[ii,jj] pair
//          } // end iter ii, for every m_pEdges[ii,jj] pair
//      } // end m_pEdges existence check
//  
//    // todo - add SmCEdge checks here
//  
//    // all done
//    // SM_ASSERT(bRtn) ;
//    return(bRtn) ;
//  
//  } // end SmCEdge::AssertValid
//  
//  // obsolete
//  // /*******************************************************************//**
//  // PURPOSE: Fix Assert Report failures reported by an AssertValid call
//  // 
//  // RETURNS ---  TRUE  = OK
//  //              FALSE = Problem
//  // ***********************************************************************/
//  // SmBoolean SmCEdge::AssertHeal
//  //  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//  //   SmAssertArray  * pAList)    // in : AssertArray holding rAReport 
//  // {
//  //   SmBoolean bRtn = FALSE ;
//  // 
//  //   // check state - no work
//  //   if(rAReport.m_bOK == TRUE)
//  //     { return( TRUE ) ; }
//  // 
//  //   // check state - not the class that generated this report - pass call to parent class
//  //   if(rAReport.m_lReportingType != GetClassType())
//  //     {
//  //       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//  //       return ( SmEdge::AssertHeal(rAReport, pAList) ) ;
//  //     }
//  //    
//  //   // branch on the report type
//  //   switch(rAReport.m_lTestIndex)
//  //     {
//  //       case 99 : { // set case number appropriately - run fix code here
//  //                   // if fix works set rAReport.m_bOK = TRUE ; 
//  //                 }
//  //                 break ;
//  // 
//  //       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//  //                rAReport.m_pHealMessage = _T("SmCEdge::AssertHeal fix not yet supported") ;  
//  //                 
//  //     }
//  // 
//  //   // all done
//  //   return(bRtn) ;
//  // 
//  // } // end SmCEdge::AssertHeal
//  // end obsolete
//  
//  /*******************************************************************//**
//  PURPOSE:
//  NOTES:
//  ***********************************************************************/
//  SmBoolean SmCEdge::IsKindOf( SM_TYPE t ) const
//  {
//    return ((SmCEdge_TYPE == t) ? TRUE : SmEdge::IsKindOf( (t) ));
//  }
//  
//  /*******************************************************************//**
//  PURPOSE:
//  
//  NOTES:
//  ***********************************************************************/
//  void SmCEdge::Dump(void) const
//  {
//    smos_WriteBuffer(_T("SmCEdge object\n"));
//  
//  } // end SmCEdge::Dump
//  
//  /*******************************************************************//**
//  PURPOSE:  Pretty print pointer values for this CompositeEdge showing its
//               list of child Edges in the topology graph.
//  
//  NOTES: Only good for debugging because pointer values don't
//                  stay constant from run to run.
//  ***********************************************************************/
//  void SmCEdge::DumpTopology
//    (ULONG lWalkDepth) // in : 0 = no walking, 1 = and member edges, 2 =their vertices, ... 99 = walk to bottom
//   const
//  {
//    // locals
//    TCHAR        sBuff[SM_TBLOCK_SIZE] ;
//  
//    smos_sprintf(sBuff,_T("  CEdge[0x%p] has %ld children \n"), this, m_pEdges->GetSize()) ;
//    smos_WriteBuffer(sBuff);
//  
//    ULONG ii;
//    for(ii=0;ii<m_pEdges->GetSize();ii++)
//      {
//        SmEdge *pEdge = (*m_pEdges)[ii] ;
//  
//        smos_sprintf(sBuff,_T("    CEdge[0x%p] -> Edge [0x%p] \n"), this, pEdge) ;
//        smos_WriteBuffer(sBuff);
//      }
//  
//    if(lWalkDepth > 0)
//      {
//        for(ii=0;ii<m_pEdges->GetSize();ii++)
//          {
//            SmEdge *pEdge = (*m_pEdges)[ii] ;
//  
//            pEdge->DumpTopology(lWalkDepth == 99 ? lWalkDepth : lWalkDepth - 1) ; 
//          }
//      }
//  
//  } // end SmCEdge::DumpTopology
