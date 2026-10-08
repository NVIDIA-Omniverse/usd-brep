// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmAssemblyInstance.cpp
* PURPOSE: Source file for AssemblyInstance object class.
**********************************************************************/

#include "StdAfx.h"

#include <SmAssemblyInstance.h>
#include <SmGraphicsOutput.h>
#include <SmAssertArray.h>
#include <SmAssembly.h>


/*******************************************************************//**
PURPOSE: Constructor for SmAssemblyInstance object.

NOTES: 
***********************************************************************/
SmAssemblyInstance::SmAssemblyInstance
 (const TCHAR            * pName,
  SmAssembly             & rOwningAssembly,
  SmSAGObject            * pComponent, // SmAssembly or SmBrep owned by this SmAssemblyInsstance
  const SmAxis2Placement & crPlacement,
  const SmVector3d       & crScaling)
 : m_pName(NULL), 
   m_vPlacement(crPlacement), 
   m_vScale(crScaling),
   m_bMirrored(FALSE)
{
  // okay to use smos_Calloc on base (char) objects.
  TCHAR* pNewName = (TCHAR*)smos_Calloc( SM_TBLOCK_SIZE, sizeof( TCHAR ));
  if (pName) 
    {
      smos_WStrCpy(pNewName,SM_TBLOCK_SIZE,pName);
      m_pName = pNewName;
    }

  rOwningAssembly.AddInstance(this);
  m_pComponent = pComponent;

} // end SmAssemblyInstance::SmAssemblyInstance constructor

SmAssemblyInstance::SmAssemblyInstance
 (const TCHAR            * pName,
  SmAssembly             & rOwningAssembly,
  SmSAGObject            * pComponent, // SmAssembly or SmBrep owned by this SmAssemblyInsstance
  const SmAxis2Placement & crPlacement,
  const SmVector3d       & crScaling,
  SmBoolean                bMirrored)
 : m_pName(NULL), 
   m_vPlacement(crPlacement), 
   m_vScale(crScaling),
   m_bMirrored(bMirrored)
{
  // okay to use smos_Calloc on base (char) objects.
  TCHAR* pNewName = (TCHAR*)smos_Calloc( SM_TBLOCK_SIZE, sizeof( TCHAR ));
  if (pNewName) 
    {
      smos_WStrCpy(pNewName,SM_TBLOCK_SIZE,pName);
      m_pName = pNewName;
    }

  rOwningAssembly.AddInstance(this);
  m_pComponent = pComponent;

} // end SmAssemblyInstance::SmAssemblyInstance constructor

/*******************************************************************//**
PURPOSE: Destructor for the SmAssemblyInstance object.

NOTES:
***********************************************************************/
SmAssemblyInstance::~SmAssemblyInstance()
{
    if (m_pName) { smos_Free(m_pName); m_pName = NULL ; }


    SmAssembly *pParent = GetParentAssembly();
    if (pParent) {
        pParent->RemoveInstance(this);
    }

}

/*******************************************************************//**
PURPOSE: Build the graphics structure for an SmAssemblyInstance.

NOTES: 
***********************************************************************/
SmStatus SmAssemblyInstance::BuildGraphicsStructure
  (const SmDisplayParameters & crDisp,                // in : graphics controlling parameters
   SmDisplayList             & rDisplayList,          // out: displayList name for this Brep Graphics
   SmDisplayList             * pActiveDisplayListRef, // out: Ref to DisplayList Copy placed on the s_View.m_pActiveLists that actually gets drawn
   SmGfxArraySet             * pOptGfxSet)            // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                      //      NULL to ignore, default:[NULL]
{
  SM_REF2(rDisplayList, pActiveDisplayListRef);
  SmAssembly* pAssembly = SM_CAST_PTR(SmAssembly, m_pComponent);
  SmBrep* pBrep = SM_CAST_PTR(SmBrep, m_pComponent);
  if (pAssembly)
  {

      SmDisplayList sAssemGraphics, *pActiveAssemGraphicsRef = NULL;
      SER(pAssembly->BuildGraphicsStructure(crDisp, sAssemGraphics, pActiveAssemGraphicsRef, pOptGfxSet));
  }
  else if (pBrep)
  {
      SM_DBG_MSG(TRUE, _T("How to display Brep in Assembly?"));
  }
  return SM_SUCCESS;

} // end SmAssemblyInstance::BuildGraphicsStructure

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertAssemblyInstance_list[] =
{
  {SM_AT_POINTER, _T("Context"), _T("m_pAssembly shares the same context") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmAssemblyInstance::AssertValid
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

  // SmAssemblyInstance and the objects it attaches to need to share common contexts
  if(m_pComponent) { 
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (GetContext() == m_pComponent->GetContext() ), _T("") ) ;
  }

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmAssemblyInstance::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmAssemblyInstance::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmAssemblyInstance::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmAssemblyInstance::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmAssemblyInstance::IsKindOf( SM_TYPE t ) const
{
  return ((SmAssemblyInstance_TYPE == t) ? TRUE : SmSAGObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump the Assembly Instance for debugging.

NOTES: 
***********************************************************************/
void SmAssemblyInstance::Dump( void ) const
{

  if(m_pName && smos_WStrLen( m_pName ) > 0)
  {
    smos_WriteBuffer( _T( "\n\nSmAssemblyInstance name = " ) );
    smos_WriteBuffer( m_pName );
    smos_WriteBuffer( _T( "\n" ) );
  }

  smos_WriteBuffer(_T("Transformation -\n"));
  m_vPlacement.Dump();

  if (m_pComponent) m_pComponent->Dump();
}

/*******************************************************************//**
PURPOSE: This new operator exists to work with the overloaded new.
    To be removed. 

NOTES: Note that the optimization has not yet been implemented.
***********************************************************************/
void *SmAssemblyInstance::operator new(size_t size, const SmContext & crContext)
{
    SmTopology *pRet = (SmTopology*)SmObject::operator new (size,crContext);
    return (void*)pRet;

} // end SmAssembly::operator new

/*******************************************************************//**
PURPOSE: Dump the Assembly Instance for debugging.

NOTES: lDepth used to indent dump (for tree display of SmAssembly)
***********************************************************************/
void SmAssemblyInstance::Dump( int lDepth ) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE], sIndent[SM_TBLOCK_SIZE];

  // set indent string equal to depth of recursive call
  int i, i2;
  for(i=0,i2=0;i<=lDepth;i++,i2+=2)
    {
      sIndent[i2  ] = ' ' ;
      sIndent[i2+1] = ' ' ;                                       
    }                                              
  sIndent[i2] = '\0' ;

  TCHAR sMirrored[SM_TBLOCK_SIZE];
  (m_bMirrored) ? smos_sprintf(sMirrored, _T("%s"), _T("TRUE")) : smos_sprintf(sMirrored, _T("%s"), _T("FALSE"));

  // Get Name if there is one
  TCHAR sName[SM_TBLOCK_SIZE];
  (m_pName) ? smos_sprintf(sName, _T("%s"), m_pName) : smos_sprintf(sName, _T("%s"), _T("NULL"));

  // Dump the name and mirrored
  smos_sprintf(sBuff, _T("\n%.256sAssemblyInstance Name: %.256s. Is mirrored: %.256s"), sIndent, sName, sMirrored);
  smos_sprintf(sBuffForFile, _T("\n%.256sAssemblyInstance Name: %.256s. Is mirrored: %.256s"), sIndent, sName, sMirrored);
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Dump the transform
  SmPoint3d sOrigin = m_vPlacement.GetOrigin();
  SmVector3d sXAxis = m_vPlacement.GetXAxis();
  SmVector3d sYAxis = m_vPlacement.GetYAxis();
  smos_sprintf(sBuff, _T("\n%.512s  Transform:"), sIndent);
  smos_sprintf(sBuffForFile, _T("\n%.512s  Transform:"), sIndent);
  smos_WriteBuffer(sBuff, sBuffForFile);
  // Dump the origin
  smos_sprintf(sBuff, _T("\n%.512s    Origin = [%16.16lf, %16.16lf, %16.16lf]"), sIndent,sOrigin.x,sOrigin.y,sOrigin.z);
  smos_sprintf(sBuffForFile, _T("\n%.512s    Origin = [%16.16lf, %16.16lf, %16.16lf]"), sIndent,sOrigin.x,sOrigin.y,sOrigin.z);
  smos_WriteBuffer(sBuff, sBuffForFile);
  // Dump the XAxis
  smos_sprintf(sBuff, _T("\n%.512s    XAxis = [%16.16lf, %16.16lf, %16.16lf]"), sIndent,sXAxis.x,sXAxis.y,sXAxis.z);
  smos_sprintf(sBuffForFile, _T("\n%.512s    XAxis = [%16.16lf, %16.16lf, %16.16lf]"), sIndent,sXAxis.x,sXAxis.y,sXAxis.z);
  smos_WriteBuffer(sBuff, sBuffForFile);
  // Dump the YAxis
  smos_sprintf(sBuff, _T("\n%.512s    YAxis = [%16.16lf, %16.16lf, %16.16lf]"), sIndent,sYAxis.x,sYAxis.y,sYAxis.z);
  smos_sprintf(sBuffForFile, _T("\n%.512s    YAxis = [%16.16lf, %16.16lf, %16.16lf]"), sIndent,sYAxis.x,sYAxis.y,sYAxis.z);
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Dump the scale
  smos_sprintf(sBuff, _T("\n%.512s    Scale = [%16.16lf, %16.16lf, %16.16lf]"), sIndent,m_vScale.x,m_vScale.y,m_vScale.z);
  smos_sprintf(sBuffForFile, _T("\n%.512s    Scale = [%16.16lf, %16.16lf, %16.16lf]"), sIndent,m_vScale.x,m_vScale.y,m_vScale.z);
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Get Name if there is one
  TCHAR sComponentName[SM_TBLOCK_SIZE];
  (m_pComponent->GetName()) ? smos_sprintf(sComponentName, _T("%s"), m_pComponent->GetName()) : smos_sprintf(sComponentName, _T("%s"), _T("NULL"));

  switch (m_pComponent->GetType())
  {
  case SmAssembly_TYPE:
  {
      //smos_sprintf(sBuff, _T("\n%s  Assembly Name: %s. Address: [0x%p]. Assembly Dump: "), sIndent, sComponentName, m_pComponent);
      //smos_sprintf(sBuffForFile, _T("\n%s  Assembly Name: %s. Address: [0x%p]. Assembly Dump: "), sComponentName, m_pComponent);
      //smos_WriteBuffer(sBuff, sBuffForFile);
      ((SmAssembly*)m_pComponent)->Dump(lDepth + 1);
      break;
  }
  case SmBrep_TYPE:
  {
      smos_sprintf(sBuff, _T("\n%.256s  Brep Name: %.256s. Address: [0x%p]. "), sIndent, sComponentName, m_pComponent);
      smos_sprintf(sBuffForFile, _T("\n%.256s  Brep Name: %.256s. Address: [0x%p].  "), sIndent, sComponentName, m_pComponent);
      smos_WriteBuffer(sBuff, sBuffForFile);
      break;
  }
  default:
      break;
  }
}
