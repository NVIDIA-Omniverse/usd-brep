// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmAssemblyInstance.h
* PURPOSE: Header file for Assembly Instance object class.
**********************************************************************/

#ifndef __SMASSEMBLYINSTANCE_H__
#define __SMASSEMBLYINSTANCE_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMSAGOBJECT_H__
#include <SmSAGObject.h>
#endif

//#ifndef __SMASSEMBLY_H__
//#include <SmAssembly.h>
//#endif

class SmAssembly;

/*******************************************************************//**
PURPOSE: efficiently represent a copy of a component in an assembly.


NOTES: 
    An assembly instance contains 
      a pointer to an assembly,
      a name,
      and position data.

    An AssemblyInstance can be included as a component of an SmAssembly
      at which point it will have a backpointer to the parent assembly and
      next and prev pointers to its sibling components.

    This is a very efficient manner to include multiple copies of a 
      subassembly without having to make a copy of every brep in the
      subassembly every time the sub-assembly is used.

   This class is used for efficiency.  Rather than pay the memory
   and computing costs to duplicate a sub-assembly once for every time it's 
   included into an assembly, just create the original sub-assembly once 
   as its own SmAssembly object and then create and add an SmAssemblyInstance
   to the assembly's SmAssembly object every time the sub-assembly appears in 
   the final assembly.

***********************************************************************/
class SM_EXPORT SmAssemblyInstance : public SmSAGObject
{
protected:
    // inherited:
    // SmTopology::m_pListOwner            - Parent Assembly containing this AssemblyInstance
    // SmTopology::m_pNext                 - next SmAssemblyInstance component owned by Parent Assembly
    // SmTopology::m_pLast                 - prev SmAssemblyInstance component owned by Parent Assembly
    //
    // SmOwningTopology::m_pList           - not used (current best guess)
    // SmOwningTopology::m_lListSize       - not used (current best guess)
    // SmSAGObject::m_lDisplayListName     -
    // SmSAGObject::m_vDisplayParameters   -

    TCHAR*                 m_pName;        // name assigned to this use of m_pAssmOrBrep
    SmAxis2Placement       m_vPlacement;   // Move of sub-assembly m_pAssembly to its location in the Parent Assembly     
    SmVector3d             m_vScale;       // Scaling of sub-assembly m_pAssembly to its use in the Parent Assembly     
    SmBoolean              m_bMirrored;    // TRUE = sub-assembly m_pAssembly is mirrored in its use in the Parent Assembly     
                                           // FALSE= not
    SmSAGObject          * m_pComponent;   // Pointer to sub-assembly or Brep

public:
    SmAssemblyInstance
    (
      const TCHAR* pName,                       ///< [in] :   <br>
      SmAssembly &rOwningAssembly,              ///< [in] :   <br>
      SmSAGObject *pComponent,                  ///< [in] :   <br>
      const SmAxis2Placement & crPlacement,     ///< [in] :   <br>
      const SmVector3d & dScaling               ///< [in] :   <br>
    );

    SmAssemblyInstance
    (
      const TCHAR*  pName,                      ///< [in] :    <br>
      SmAssembly &rOwningAssembly,              ///< [in] :    <br>
      SmSAGObject *pComponent,                  ///< [in] :   <br>
      const SmAxis2Placement & crPlacement,     ///< [in] :    <br>
      const SmVector3d &crScaling,              ///< [in] :    <br>
      SmBoolean bMirrored                       ///< [in] :    <br>
    );

    virtual ~SmAssemblyInstance();

    virtual SmStatus BuildGraphicsStructure
    (
      const SmDisplayParameters & crDisplayParameters,   ///< [in] : graphics controlling parameters                                                       <br>
      SmDisplayList             & rDisplayList,          ///< [out]: displayList name for this Brep Graphics                                               <br>
      SmDisplayList             * pActiveDisplayListRef, ///< [out]: Ref to DisplayList Copy placed on the s_View.m_pActiveLists that actually gets drawn  <br>
      SmGfxArraySet             * pOptGfxSet=NULL        ///< [in] : When given output GfxVertexArrays not GL calls.                                       <br>
    ) ;     

    TCHAR                  * GetName()            { return m_pName; }
    SmAxis2Placement const & GetPlacement() const { return m_vPlacement; }
    SmVector3d const       & GetScale()     const { return m_vScale; }
    SmBoolean                GetMirrored()  const { return m_bMirrored; }
    SmAssembly const       * GetAssembly()  const { return SM_CAST_PTR(SmAssembly,m_pComponent); }
    SmAssembly             * GetAssembly()        { return SM_CAST_PTR(SmAssembly,m_pComponent); }
    SmBrep                 * GetBrep()            { return SM_CAST_PTR(SmBrep,m_pComponent); }
    SmSAGObject const      * GetComponent() const { return SM_CAST_PTR(SmSAGObject,m_pComponent); }
    SmSAGObject            * GetComponent()       { return SM_CAST_PTR(SmSAGObject,m_pComponent); }
    SmAssembly             * GetParentAssembly()  { return (SmAssembly*)m_pListOwner; }
    
    virtual SmBoolean AssertValid
    (
      SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                              <br>
      SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                          <br>
                                                ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                      <br>                                                                             <br>
      SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]    <br>
      SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                    <br>
    ) const ;

    // obsolete
    //  virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmAssemblyInstance,SmSAGObject,SmAssemblyInstance_TYPE);

    void Dump(int lDepth) const;

    void *operator new(size_t size, const SmContext & crContext);
    void  operator delete(void *ptr) { smos_Free(ptr); ptr = NULL ; }
    void  operator delete(void* ptr, const SmContext& ) { smos_Free(ptr); ptr = NULL;}
} ; // end class SmAssemblyInstance

#endif // !__SMASSEMBLYINSTANCE_H__



