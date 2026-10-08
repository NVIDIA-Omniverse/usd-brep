// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmAssembly.cpp
* PURPOSE: Source file for Assembly object class.
**********************************************************************/

#include "StdAfx.h"

#include <SmAssembly.h>
#include <SmAssemblyInstance.h>
#include <SmGraphicsOutput.h>
#include <SmAssertArray.h>
#include <SmPoly.h>

/*******************************************************************//**
PURPOSE: Constructor for SmAssembly object.

NOTES: 
***********************************************************************/
SmAssembly::SmAssembly(const TCHAR* pName)
 : m_pName(NULL), 
   m_pUserPointer(NULL)
{
  // okay to use smos_Calloc on base (char) objects.
  TCHAR * pNewName = (TCHAR*)smos_Calloc(SM_TBLOCK_SIZE, sizeof(TCHAR));
  if (pName) {
      smos_WStrCpy(pNewName,SM_TBLOCK_SIZE,pName);
      m_pName = pNewName;
  }

} // end SmAssembly::SmAssembly constructor

/*******************************************************************//**
PURPOSE: Destructor for the SmAssembly object.

NOTES: 
***********************************************************************/
SmAssembly::~SmAssembly()
{
    if (m_pName) { smos_Free(m_pName); m_pName = NULL ; }

    SmTArray<SmAssemblyInstance*> sInstances;
    GetAssemblyInstances(sInstances);
    for (ULONG i=0; i<sInstances.GetSize(); i++)
    { Remove(sInstances[i]); }
}



/*******************************************************************//**
PURPOSE: Build the graphics structure for an assembly.

NOTES: 
***********************************************************************/
SmStatus SmAssembly::BuildGraphicsStructure
  (const SmDisplayParameters & crDisp,                // in : graphics controlling parameters
   SmDisplayList             & rDisplayList,          // out: displayList name for this Brep Graphics
   SmDisplayList             * pActiveDisplayListRef, // out: Ref to DisplayList Copy placed on the s_View.m_pActiveLists that actually gets drawn
   SmGfxArraySet             * pOptGfxSet)            // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                      //      NULL to ignore, default:[NULL]
{
  SM_REF2(rDisplayList, pActiveDisplayListRef);
    // How to tranforms each Brep according to the AssemblyInstance?
    // How to not tessellate each instance of the Brep? i.e. reuse tessellations?
  SmDisplayList         * pCompActiveDisplayListRef = NULL ;
  SmTArray<SmAssemblyInstance*> sInstances;
  GetAssemblyInstances(sInstances);
  SmTArray<SmDisplayList> sCompGraphics;
  for (ULONG i=0; i<sInstances.GetSize(); i++) 
    {
      SmSAGObject* pComponent = sInstances[i]->GetComponent();
      SmDisplayList lCompDispName;
      SER(pComponent->BuildGraphicsStructure(crDisp, lCompDispName, pCompActiveDisplayListRef, pOptGfxSet));
      if (lCompDispName.GetDisplayListId() != SM_BIG_ULONG) 
        {
          sCompGraphics.Add(lCompDispName);
        }
    }

  return SM_SUCCESS;

} // end SmAssembly::BuildGraphicsStructure

/*******************************************************************//**
PURPOSE: Draw routine for Assemblies

NOTES:
  For debugging - let bAddToUIPickList = TRUE,
  to place this Assembly onto the pick list (It only needs to be added
  once) and then it can be picked by the host application from
  a sm_GraphicsLoop() call.

  See Also: sm_GraphicsAddToBrepList(SmObject *pObject) ;
            sm_GraphicsBrepListClear() ;
***********************************************************************/
SmDisplayList * SmAssembly::Draw
  (SmBoolean bAddToUIPickList,     // in : TRUE = add this Brep to display list so it can be picked
                                   //      FALSE= don't
   SmGfxArraySet * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore, default:[NULL]
 const                   
{
  SM_REF1(bAddToUIPickList) ;
  SmDisplayList *pRtnDisplayList = NULL ;

#ifdef SM_GFX_OUTPUT_CODE
  SmMapTypeToType<SmBrep*, SmPolyBrep*> sBrepMap;
  SmTArray<const SmAxis2Placement*> sTransformStack;
  SmTArray<SmVector3d> sScaleStack;

SM_REF1(bAddToUIPickList);

  Draw(bAddToUIPickList, pOptGfxSet, sBrepMap, sTransformStack, sScaleStack);
  //// Brute for the display for now.  Want to reuse the assembly structure
  //// Convert all breps to polybreps and draw those?
  //for (ULONG ii = 0; ii < sInstances.GetSize(); ++ii)
  //{
  //    // Loop locals
  //    SmAssemblyInstance* pInstance = sInstances[ii];
  //    SmSAGObject* pComponent = pInstance->GetComponent();
  //    SmBrep* pBrep = SM_CAST_PTR(SmBrep, pComponent);
  //    SmAssembly* pAssembly = SM_CAST_PTR(SmAssembly, pComponent);
  //    const SmAxis2Placement & crTransform = pInstance->GetPlacement();
  //    const SmVector3d& rScale = pInstance->GetScale();

  //    if ( pBrep )
  //    {
  //        // Transform the brep, then draw it
  //        pBrep->Transform(crTransform, &rScale);
  //        pRtnDisplayList = pBrep->Draw(bAddToUIPickList, pOptGfxSet);

  //        // Invert the transforms.
  //        SmAxis2Placement sTransformInverse;
  //        SmVector3d sScaleInverse(1., 1., 1.);
  //        sScaleInverse.Divide(rScale, sScaleInverse);
  //        crTransform.Invert(sTransformInverse);

  //        // Transform applys scale, then transform. So for the inverse we have to operate in the other order
  //        // (these operations do not commute)
  //        pBrep->Transform(sTransformInverse);
  //        pBrep->Transform(SmAxis2Placement(), &sScaleInverse);
  //    }
  //    else if (pAssembly)
  //    {
  //        SmTArray<const SmAxis2Placement*> sTransformStack;
  //        SmTArray<SmVector3d> sScaleStack;
  //        sTransformStack.Add(&crTransform);
  //        sScaleStack.Add(rScale);
  //        pAssembly->Draw(bAddToUIPickList, pOptGfxSet, sBrepMap, sTransformStack, sScaleStack);
  //    }
  //    else
  //    {
  //        SM_DBG_WARN(_T("Unexpected component in an SmAssemblyInstance."));
  //    }
  //}
  // Attempt below at lower level functions. 
  //// locals: Global display parameters
  //const SmDisplayParameters &rDisp = smgfx_RefGlobalDisplayParameters() ;
  ////SmBrep                    *pBrep = SM_CONST_CAST(SmBrep*, this);

  ////// when asked - add this Brep to UI pick list
  ////if(bAddToUIPickList)
  ////  { sm_GraphicsAddToBrepList(pBrep) ; }

  //// set DrawState for color, lineWidth, and PointSize.
  //SmVector3d sColor = smgfx_OutputObjectColor(this, rDisp.GetShadedColorRule(), pOptGfxSet) ;
  //double dLineWidth = smgfx_OutputLineWidth(rDisp.m_dLineWidth, pOptGfxSet);
  //double dPointSize = smgfx_OutputPointSize(rDisp.m_dPointSize, pOptGfxSet);

  //// make the Brep display list for these display parameters
  //SmDisplayList sTmpList;
  ////SE();
  //SM_CONST_CAST(SmAssembly*, this)->BuildGraphicsStructure(rDisp, sTmpList, pRtnDisplayList, pOptGfxSet);

  //// restore Display state value
  //smgfx_OutputColor(sColor, pOptGfxSet) ;
  //smgfx_OutputLineWidth(dLineWidth, pOptGfxSet) ;
  //smgfx_OutputPointSize(dPointSize, pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // defined SM_GFX_OUTPUT_CODE

  // all done
  return(pRtnDisplayList) ;

} // end SmAssembly::Draw

/*******************************************************************//**
PURPOSE: Draw routine for Assemblies

NOTES:
  For debugging - let bAddToUIPickList = TRUE,
  to place this Assembly onto the pick list (It only needs to be added
  once) and then it can be picked by the host application from
  a sm_GraphicsLoop() call.

  See Also: sm_GraphicsAddToBrepList(SmObject *pObject) ;
            sm_GraphicsBrepListClear() ;
***********************************************************************/
SmDisplayList * SmAssembly::Draw  
(
  SmBoolean bAddToUIPickList,                       ///< [in] :
  SmGfxArraySet * pOptGfxSet,                       ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters()
  SmMapTypeToType<SmBrep*, SmPolyBrep*> & rBrepMap, ///< [in] :
  SmTArray<const SmAxis2Placement*> &rTransformStack,     ///< [in] :
  SmTArray<SmVector3d> &rScaleStack                 ///< [in] :
) const
{
  SmDisplayList *pRtnDisplayList = NULL ;

#ifdef SM_GFX_OUTPUT_CODE
  SmTArray<SmAssemblyInstance*> sInstances;
  GetAssemblyInstances(sInstances);

  // Brute for the display for now.  Want to reuse the assembly structure
  // Convert all breps to polybreps and draw those?
  for (ULONG ii = 0; ii < sInstances.GetSize(); ++ii)
  {
      // Loop locals
      SmAssemblyInstance* pInstance = sInstances[ii];
      SmSAGObject* pComponent = pInstance->GetComponent();
      SmBrep* pBrep = SM_CAST_PTR(SmBrep, pComponent);
      SmAssembly* pAssembly = SM_CAST_PTR(SmAssembly, pComponent);
      const SmAxis2Placement & crTransform = pInstance->GetPlacement();
      const SmVector3d& rScale = pInstance->GetScale();
      rTransformStack.Add(&crTransform);
      rScaleStack.Add(rScale);

      if ( pBrep )
      {
          SmContext sContext;
          SmBrep* pBrepCopy = new (sContext) SmBrep(*pBrep);
          //SmObjDelete sClean(pBrepCopy);
          
          // Apply all transforms to the brep, starting with the most local
          for ( ULONG jj = rTransformStack.GetSize(); jj > 0; --jj)
          { pBrepCopy->Transform(*rTransformStack[jj - 1], &rScaleStack[jj-1]); }

          // Draw the brep
          pRtnDisplayList = pBrepCopy->Draw(bAddToUIPickList, pOptGfxSet);

          pBrepCopy->Dump();
      }
      else if (pAssembly)
      {
          pAssembly->Draw(bAddToUIPickList, pOptGfxSet, rBrepMap, rTransformStack, rScaleStack);
      }
      else
      {
          SM_DBG_WARN(_T("Unexpected component in an SmAssemblyInstance."));
      }
      rTransformStack.RemoveLast();
      rScaleStack.RemoveLast();
  }
#else
  SM_REF2(bAddToUIPickList, pOptGfxSet);
  SM_REF3(rBrepMap, rTransformStack, rScaleStack);
#endif // defined SM_GFX_OUTPUT_CODE

  // all done
  return(pRtnDisplayList) ;
} // end SmAssembly::Draw

/*******************************************************************//**
PURPOSE: Get the Breps and Assemblies used by this assembly using a
    breadth first traversal of the assembly tree.  

NOTES: increments an unlocked mark
***********************************************************************/
SmStatus SmAssembly::GetComponents(SmTArray<SmBrep*> & rBreps, SmTArray<SmPolyBrep*> & rPolyBreps, SmTArray<SmAssembly*> & rSubAssemblies)
{
  SmNewMarkAndLock sMarkLock(GetContext(), SM_MT_ALLMARKS);
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;

  rBreps.ReSet();
  SmTArray<SmAssembly*> sAssemblies;
  sAssemblies.Add(this);

  // Traverse the stack of Assemblies
  while ( sAssemblies.GetSize() > 0)
  {
      // Get the last Assembly and remove it from the stack
      SmAssembly* pThisAssembly = sAssemblies.GetLast();
      sAssemblies.RemoveLast();

      // Traverse the AssemblyInstances in this Assembly
      SmAssemblyInstance* pHeadInstance = (SmAssemblyInstance*) pThisAssembly->GetList();

      if (pHeadInstance)
      {
          SmAssemblyInstance* pNextInstance = pHeadInstance;
          do
          {
              // Get a pointer to the component in this AssemblyInstance
              SmSAGObject* pComponent = pNextInstance->GetComponent();

              // Add assembly components to the stack. Add breps to the output.
              if (!pComponent->IsMarked(eMarkType))
              {
                  switch (pComponent->GetType())
                  {
                  case SmAssembly_TYPE:
                  {
                      rSubAssemblies.Add((SmAssembly*)pComponent);
                      sAssemblies.Add((SmAssembly*)pComponent);
                      pComponent->Mark(eMarkType);
                      break;
                  }
                  case SmBrep_TYPE:
                  {
                      rBreps.Add((SmBrep*)pComponent);
                      pComponent->Mark(eMarkType);
                      break;
                  }
                  case SmPolyBrep_TYPE:
                  {
                      rPolyBreps.Add((SmPolyBrep*)pComponent);
                      pComponent->Mark(eMarkType);
                      break;
                  }
                  default:
                  {
                      SE_MSG(SM_ERR, _T("AssemblyInstance does not point to Assembly or Brep"));
                      break;
                  }
                  }
              }

              // Get the next instance in this Assembly
              pNextInstance = (SmAssemblyInstance*)pNextInstance->GetNext();

          } while (pHeadInstance != pNextInstance);
      }
  }

  return SM_SUCCESS;

} // end SmAssembly::GetComponents

///*******************************************************************//**
//PURPOSE: Get the components of the assembly.
//
//NOTES: 
//***********************************************************************/
//void SmAssembly::GetComponents(SmTArray<SmSAGObject*> & rComponents) const
//{ 
//    SmTArray<SmAssemblyInstance*> sAssemblyInstances;
//    GetAll(SM_REINTERPRET_CAST(SmTArray<class SmTopology*>&,sAssemblyInstances)); 
//
//    ULONG lNumInstance = sAssemblyInstances.GetSize();
//    for (ULONG ii =0; ii < lNumInstance; ++ii)
//    {
//
//    }
//}

/*******************************************************************//**
PURPOSE: Get all instances of this assembly.

NOTES: 

***********************************************************************/
void SmAssembly::GetAssemblyInstances(SmTArray<SmAssemblyInstance*> & rInstances) const
{
    GetAll(SM_REINTERPRET_CAST(SmTArray<class SmTopology*>&,rInstances)); 
}

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertAssembly_list[] =
{
  {SM_AT_POINTER, _T("Context"), _T("m_vInstances share the same context") }
} ;

/*******************************************************************//**
PURPOSE: This new operator exists to work with the overloaded new.
    To be removed. 

NOTES: Note that the optimization has not yet been implemented.
***********************************************************************/
void *SmAssembly::operator new(size_t size, const SmContext & crContext)
{
    SmTopology *pRet = (SmTopology*)SmObject::operator new (size,crContext);
    return (void*)pRet;

} // end SmAssembly::operator new
/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmAssembly::AssertValid
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
           ? SmSAGObject::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ;

  // SmAssembly and the objects it attaches to need to share common contexts
  ULONG ii ;
  SmTArray<SmAssemblyInstance*> sInstances;
  GetAssemblyInstances(sInstances);
  for(ii=0;ii<sInstances.GetSize();ii++)
    {
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (GetContext() == sInstances[ii]->GetContext() ), _T("") ) ;

    } // end iter every context 

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmAssembly::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmAssembly::AssertHeal
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
//       return ( SmSAGObject::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmAssembly::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmAssembly::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmAssembly::IsKindOf( SM_TYPE t ) const
{
  return ((SmAssembly_TYPE == t) ? TRUE : SmSAGObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmAssembly::Dump( void ) const
{
  if(m_pName && smos_WStrLen( m_pName ) > 0)
  {
    smos_WriteBuffer( _T( "SmAssembly name = " ) );
    smos_WriteBuffer( m_pName );
    smos_WriteBuffer( _T( "\n" ) );
  }

  SmTArray<SmAssemblyInstance*> sInstances;
  GetAssemblyInstances(sInstances);
  sInstances.Dump();
  for(ULONG i = 0; i < sInstances.GetSize(); i++)
  {
    SmSAGObject* pComp = sInstances[i]->GetComponent();
    SmBrep *pBrep = SM_CAST_PTR( SmBrep, pComp );
    if(pBrep)
    {
      smos_WriteBuffer( _T( "    Brep\n" ) );
    }
    SmAssembly *pInst = SM_CAST_PTR( SmAssembly, pComp );
    if(pInst)
    {
      TCHAR* pInstName = pInst->GetName();
      if(pInstName && (smos_WStrLen( pInstName ) < SM_TBLOCK_SIZE))
      {
        smos_WriteBuffer( pInstName );
        smos_WriteBuffer( _T( "\n" ) );
      }
    }
    else
    {
      smos_WriteBuffer( _T( "UNKNOWN Object in Assembly\n" ) );
    }
  }
  for(ULONG ii = 0; ii < sInstances.GetSize(); ii++)
  {
    sInstances[ii]->Dump();
  }
}

/*******************************************************************//**
PURPOSE: Recursive dump of the Assembly to see the whole tree

NOTES: If lDepth==0, then a histogram of Assembly and Brep uses is dumped
***********************************************************************/
void SmAssembly::Dump(int lDepth) const
{
  // For root assemblies, dump the usage of breps and assemblies
  if (lDepth == 0) { DumpUsageStats(); }

  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE], sIndent[SM_TBLOCK_SIZE];

  // set indent string equal to depth of recursive call
  int i, i2;
  for(i=0,i2=0;i<=lDepth;i++,i2+=2)
    {
      sIndent[i2  ] = ' ' ;
      sIndent[i2+1] = ' ' ;                                       
    }                                              
  sIndent[i2] = '\0' ;

  ULONG lNumBreps(0), lNumSubAssemblies(0);
  SmTArray<SmAssemblyInstance*> sInstances;
  GetAssemblyInstances(sInstances);

  // Get component counts
  for (ULONG ii = 0; ii < sInstances.GetSize(); ++ii)
  {
      SmSAGObject* pComponent = sInstances[ii]->GetComponent();
      if (pComponent->IsKindOf(SmAssembly_TYPE)) lNumSubAssemblies++;
      if (pComponent->IsKindOf(SmBrep_TYPE)) lNumBreps++;
  }

  // Get Name if there is one
  TCHAR sName[SM_TBLOCK_SIZE];
  (m_pName) ? smos_sprintf(sName, _T("%s"), m_pName) : smos_sprintf(sName, _T("%s"), _T("NULL"));

  smos_sprintf(sBuff, _T("\n%.256sAssembly name: %.256s. Address: [0x%p]. Number of SubAssemblies: %ld.  Number of Breps: %ld"), sIndent, sName, this, lNumSubAssemblies, lNumBreps);
  smos_sprintf(sBuffForFile, _T("\n%.256sAssembly name: %.256s. Number of SubAssemblies: %ld.  Number of Breps: %ld"), sIndent, sName, lNumSubAssemblies, lNumBreps);
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Dump the Brep names and pointers, and the sub assemblies
  for (ULONG ii = 0; ii < sInstances.GetSize(); ++ii)
  { sInstances[ii]->Dump(lDepth+1); }
}

/*******************************************************************//**
PURPOSE: Dump histograms showing uses of Assemblies and Breps in this assembly

NOTES: Breps are not double-counted in repeated Assembly use. That is,
    if an Assembly is used twice, then the Brep is only counted once.
***********************************************************************/
void SmAssembly::DumpUsageStats( ) const
{
    // Locals
    SmMapTypeToType<SmAssembly*, ULONG> sAssemblyUses; // Maps Assembly* to number of uses
    SmMapTypeToType<SmBrep*, ULONG> sBrepUses;         // Maps Brep* to number of uses
    SmTArray<SmAssemblyInstance*> sInstanceStack, sInstances;
    ULONG lCount;

    // Get all the instances in this Assembly
    GetAssemblyInstances(sInstanceStack);

    // Walk the Assembly tree to find the count the uses of each Brep and Assembly
    while (sInstanceStack.GetSize() > 0)
    {
        // Pop the stack
        SmAssemblyInstance* pInstance;
        sInstanceStack.Pop(pInstance);

        // Get the component in this AssemblyInstance
        SmSAGObject* pComponent = pInstance->GetComponent();
        SmAssembly* pAssembly = SM_CAST_PTR(SmAssembly, pComponent);
        SmBrep* pBrep = SM_CAST_PTR(SmBrep, pComponent);

        // For the component, find the current number of uses and add one.
        if ( pAssembly)
        {
            lCount = sAssemblyUses.GetValueAt(pAssembly);

            // If this is the first use of the assembly, add its instances to the stack
            if ( lCount == 0)
            {
                pAssembly->GetAssemblyInstances(sInstances);
                sInstanceStack.Append(sInstances);
            }
            sAssemblyUses.SetAt(pAssembly, ++lCount);
        }
        else if ( pBrep)
        {
            lCount = sBrepUses.GetValueAt(pBrep);
            sBrepUses.SetAt(pBrep, ++lCount);
        }
        else
        { SE_MSG(SM_ERR, _T("Unexpected component type in AssemblyInstance.")); }
    }

    // Histogram locals
    SmTArray<ULONG> sBrepCounts, sAssemblyCounts;
    SmTArray<ULONG> sBrepHistogram(11, NULL, 11), sAssemblyHistogram(11, NULL, 11);
    sBrepHistogram.SetAll(0);
    sAssemblyHistogram.SetAll(0);

    // Get the counts from each map
    sAssemblyUses.GetAllValues(sAssemblyCounts);
    sBrepUses.GetAllValues(sBrepCounts);

    // Populate the Assembly histogram
    for (ULONG ii = 0; ii < sAssemblyCounts.GetSize(); ++ii)
    {
        if (sAssemblyCounts[ii] > 9)
        { sAssemblyHistogram[10]++; }
        else
        { sAssemblyHistogram[sAssemblyCounts[ii]]++; }
    }

    // Populate the Brep histogram
    for (ULONG ii = 0; ii < sBrepCounts.GetSize(); ++ii)
    {
        if (sBrepCounts[ii] > 9)
        { sBrepHistogram[10]++; }
        else
        { sBrepHistogram[sBrepCounts[ii]]++; }
    }

    // Printing locals
    TCHAR sBuff[SM_TBLOCK_SIZE];

    // Print header
    smos_WriteBuffer(_T("\n  Assembly and Brep usage histogram for the assembly tree:"));

    // sAssemblyHistogram    
    smos_WriteBuffer( _T("\n                         +---------------------------------------------------------------------------------+")) ; 
    smos_sprintf(sBuff, _T("\n  Assembly Count      : [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu]"), 
                       sAssemblyHistogram[ 1], sAssemblyHistogram[ 2],
                       sAssemblyHistogram[ 3], sAssemblyHistogram[ 4], sAssemblyHistogram[ 5],
                       sAssemblyHistogram[ 6], sAssemblyHistogram[ 7], sAssemblyHistogram[ 8],
                       sAssemblyHistogram[ 9], sAssemblyHistogram[10]) ;                            
    smos_WriteBuffer(sBuff) ; 
    smos_WriteBuffer( _T("\n           Uses       : [    1] [    2] [    3] [    4] [    5] [    6] [    7] [    8] [    9] [  10+]")) ; 
    // end sAssemblyHistogram

    // sBrepHistogram    
    smos_WriteBuffer( _T("\n                         +---------------------------------------------------------------------------------+")) ; 
    smos_sprintf(sBuff, _T("\n  Brep     Count      : [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu] [ %4lu]"), 
                       sBrepHistogram[ 1], sBrepHistogram[ 2],
                       sBrepHistogram[ 3], sBrepHistogram[ 4], sBrepHistogram[ 5],
                       sBrepHistogram[ 6], sBrepHistogram[ 7], sBrepHistogram[ 8],
                       sBrepHistogram[ 9], sBrepHistogram[10]) ;                            
    smos_WriteBuffer(sBuff) ; 
    smos_WriteBuffer( _T("\n           Uses       : [    1] [    2] [    3] [    4] [    5] [    6] [    7] [    8] [    9] [  10+]")) ; 
    // end sBrepHistogram
}

/*******************************************************************//**
PURPOSE: Go to the root assembly, then do a recursive dump of the
         root assembly to see the whole tree

NOTES: 
***********************************************************************/
void SmAssembly::DumpWholeAssembly() const
{
    TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
    SmAssembly* pThisAssembly = SM_CONST_CAST(SmAssembly*, this);
    TCHAR sName[SM_TBLOCK_SIZE];
    (m_pName) ? smos_sprintf(sName, _T("%s"), m_pName) : smos_sprintf(sName, _T("%s"), _T("NULL"));
    smos_sprintf( sBuff, _T("\nDumping whole assembly tree for Assembly: %.512s, [0x%p]."), sName, this);
    smos_sprintf(sBuffForFile, _T("\nDumping whole assembly tree for Assembly: %.512s, [0x%p]."), sName, this);
    smos_WriteBuffer(sBuff, sBuffForFile);

    // If ThisAssembly has a parent, it's owned by an AssemblyInstance, which is owned by the parent Assembly
    while (pThisAssembly->GetParentAssembly())
    { pThisAssembly = pThisAssembly->GetParentAssembly(); }

    // When there's no owning AssemblyInstance, we are at the Assembly root. Dump it.
    if (pThisAssembly)
    { pThisAssembly->Dump(0); }
    else
    { NE(pThisAssembly); }
}

/*******************************************************************//**
PURPOSE: Write out the assembly as an .smp (SMS part file)

NOTES: Breps are duplicated and transformed. The assembly structure is lost
***********************************************************************/
SmStatus smGetBrepsForWrite(const SmAssembly& rAssembly,
                            SmTArray<SmBrep*>& rBreps,
                            SmTArray<SmPolyBrep*>& rPolyBreps,
                            SmTArray<const SmAxis2Placement*>& rTransformStack,
                            SmTArray<SmVector3d>& rScaleStack)
{
    SmTArray<SmAssemblyInstance*> sInstances;
    rAssembly.GetAssemblyInstances(sInstances);

    // Brute for the display for now.  Want to reuse the assembly structure
    // Convert all breps to polybreps and draw those?
    for (ULONG ii = 0; ii < sInstances.GetSize(); ++ii)
    {
        // Loop locals
        SmAssemblyInstance* pInstance = sInstances[ii];
        SmSAGObject* pComponent = pInstance->GetComponent();
        SmBrep* pBrep = SM_CAST_PTR(SmBrep, pComponent);
        SmPolyBrep* pPolyBrep = SM_CAST_PTR(SmPolyBrep, pComponent);
        SmAssembly* pAssembly = SM_CAST_PTR(SmAssembly, pComponent);
        const SmAxis2Placement& crTransform = pInstance->GetPlacement();
        const SmVector3d& rScale = pInstance->GetScale();
        rTransformStack.Add(&crTransform);
        rScaleStack.Add(rScale);

        if (pBrep)
        {
            SmBrep* pBrepCopy = new (*pBrep->GetContext()) SmBrep(*pBrep);
            // Apply all transforms to the brep, starting with the most local
            for (ULONG jj = rTransformStack.GetSize(); jj > 0; --jj)
            {
                pBrepCopy->Transform(*rTransformStack[jj - 1], &rScaleStack[jj - 1]);
            }

            rBreps.Add(pBrepCopy);
        }
        else if (pPolyBrep)
        {
            SmPolyBrep* pPolyBrepCopy = new (*pPolyBrep->GetContext()) SmPolyBrep(*pPolyBrep);

            // Apply all transforms to the polybrep, starting with the most local
            for (ULONG jj = rTransformStack.GetSize(); jj > 0; --jj)
            {
                pPolyBrepCopy->Transform(*rTransformStack[jj - 1], &rScaleStack[jj - 1]);
            }

            rPolyBreps.Add(pPolyBrepCopy);
        }
        else if (pAssembly)
        {
            smGetBrepsForWrite(*pAssembly, rBreps, rPolyBreps, rTransformStack, rScaleStack);
        }
        else
        {
            SM_DBG_WARN(_T("Unexpected component in an SmAssemblyInstance."));
        }
        rTransformStack.RemoveLast();
        rScaleStack.RemoveLast();
    }

    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Write out the assembly as an .smp (SMS part file)

NOTES: Breps are duplicated and transformed. The assembly structure is lost
***********************************************************************/
SmStatus SmAssembly::WriteAssemblyAsPartFile
  (const TCHAR         * cOutputFileName,    // in : target File name
    SmFileType           eType,              ///< [in] : Specify output type: oneof                                        <br>
                                             ///<        SM_ASCII  = Database is an ASCII file                             <br>
                                             ///<        SM_BINARY = Database is a Binary format                           <br>
    SmBoolean            bWriteAsBSplines,   ///< [in] : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes   <br>   
                                             ///<        FALSE= Write native formats for nonBSplines                       <br>
    SmApproxTol3d        sApproxTol3d        ///< [in] : only used when bWriteAsBSplines is TRUE                           <br>
                                             ///<        default:[SM_APPROX_TOL_3D = SM_ZONE_TOL_3D/2 = 5.0e-6]            <br>
) const
{
    //Locals to get Breps
    SmTArray<SmBrep*> sBreps;
    SmTArray<SmPolyBrep*> sPolyBreps;
    SmTArray<const SmAxis2Placement*> sTransformStack;
    SmTArray<SmVector3d> sScaleStack;
    SmObjsDelete<SmBrep*> sCleanBreps(&sBreps);

    SE(smGetBrepsForWrite(*this, sBreps, sPolyBreps, sTransformStack, sScaleStack));
    SM_ASSERT_MSG( sTransformStack.GetSize() == 0 && sScaleStack.GetSize() == 0, _T("Transform stack not cleared properly"));

    // Locals to write file
    SmTArray<SmCurve*> sCurves;
    SmTArray<SmSurface*> sSurfaces;
    SmTArray<long> sTree;

    // Write assembly as part file
    return SmBrepData::WritePartToFile(cOutputFileName, sCurves, sSurfaces, sTree, sBreps, eType, FALSE, bWriteAsBSplines, sApproxTol3d);
}


/*******************************************************************/ /**
 PURPOSE: Flatten components brep sand polybreps

 NOTES: Breps are copied and transformed. The assembly structure is lost
 ***********************************************************************/
SmStatus SmAssembly::Flatten(SmTArray<SmBrep*>& rBreps, SmTArray<SmPolyBrep*>& rPolyBreps)
{
    // Locals to get Breps

    SmTArray<const SmAxis2Placement*> sTransformStack;
    SmTArray<SmVector3d> sScaleStack;

    SE(smGetBrepsForWrite(*this, rBreps, rPolyBreps, sTransformStack, sScaleStack));
    SM_ASSERT_MSG(sTransformStack.GetSize() == 0 && sScaleStack.GetSize() == 0, _T("Transform stack not cleared properly"));

    return SM_SUCCESS;
}
