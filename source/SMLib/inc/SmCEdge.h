// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCEdge.h
* PURPOSE: Header file for SmCEdge class.
**********************************************************************/

#ifndef __SMCEDGE_H__
#define __SMCEDGE_H__

// Remove Composites - removed entire class definition
// #ifndef __SMEDGE_H__
// #include <SmEdge.h>
// #endif
// 
// /*******************************************************************//**
// PURPOSE: This class represents a collection of topological edges
//      which have the same underlying curve in a Brep.  It is a Composite
//      Edge representation.
// 
// NOTES: 
// ***********************************************************************/
// class SM_EXPORT SmCEdge : public SmEdge
// {
//     friend class SmBrep;
//     friend class SmEdge;
// 
//     // inherited:
//     // SmTopology::m_pListOwner - when in a Brep - SmBrep::m_pCEdgeListHead
//     // SmTopology::m_pNext      - Next member of CEdge list owned by SmBrep::m_pCEdgeListHead
//     // SmTopology::m_pLast      - Last member of CEdge list owned by SmBrep::m_pCEdgeListHead 
//     //
//     // SmOwningTopology::m_pList      - 
//     // SmOwningTopology::m_lListSize  - 
// 
// protected:
//     SmBrep * m_pBrep;
//     SmTArray<SmEdge*> * m_pEdges;    // All of the edges which use the underlying curve in
//                                      // the Brep
// protected:
//     SmCEdge(SmEdge *pEdge1, SmEdge *pEdge2);
//     virtual ~SmCEdge();
// 
// public:
//     SmStatus AddEdge(SmEdge *pEdgeToAdd);
// 
//     SmStatus RemoveEdge(SmEdge *pEdgeToRemove);  // note: pEdgeToRemove->Curve set to NULL not deleted, UVTrimCurves are deleted.
// 
//     void GetEdges(SmTArray<SmEdge*> & rEdges) const;
// 
//     SmBoolean IsEdgeInCEdge(const SmEdge *pE) const;
//     
//     virtual SmBrep* GetBrep() const;
// 
//   // For when curve parameterization changes:
//     virtual SmStatus UpdateDomain();
// 
//     // get memory used by CEdge and its attributes - not any of its child Edges
//     virtual ULONG GetMemoryUsed
//     (
//       ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes <br>
//       SmMarkType eMarkType=SM_MT_NOMARK  ///< [in] : uses without increment eMarkType value       <br>
//     ) const ;
// 
//     virtual SmBoolean AssertValid
//     (
//       SmAssertArray    * pAList=NULL,           ///< [in,out] : Accumulating list of failed Asserts, NULL to ignore                                           <br>
//       SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        <br>
//                                                 ///<        SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    <br>
//       SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  <br>
//       SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  <br>
//     ) const ;
// 
//     // obsolete
//     // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
// 
//     // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
//     SM_COMMON(SmCEdge,SmEdge,SmCEdge_TYPE);
//     void DumpTopology(ULONG lWalkDepth=0) const ;  // lWalkDepth[0] = no walk, [99] = walk to bottom
// 
// } ; // end class SmCEdge

#endif // !__SMCEDGE_H__

