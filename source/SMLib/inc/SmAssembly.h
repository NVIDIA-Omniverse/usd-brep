// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmAssembly.h
* PURPOSE: Header file for Assembly object class.
**********************************************************************/

#ifndef __SMASSEMBLY_H__
#define __SMASSEMBLY_H__

#include <SmTypes.h>
#include <SmSAGObject.h>
#include <SmMapTypeToType.h>
#include <SmAssemblyInstance.h>

// GWC:BIND_TEMPLATES_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmAssemblyInstance*) ;

/*******************************************************************//**
PURPOSE: Represent hierarchical assembly trees whose leaf nodes are SmBrep objects.

NOTES: 

  Assembly Tree Design:

    Each assembly tree has one top level assembly with no owner or siblings.
    Each assembly has its own name.
    Each assembly has a doubly linked list of components (stored under m_pList using m_pNext and m_pPrev).
    Components are SmAssemblyInstances:
      SmAssemblyInstance - a moved, rotated, and optionally mirrored copy of a:
        SmBrep             - a shape component contained directly in this AssemblyInstance
        SmAssembly         - a sub-assembly    contained directly in this AssemblyInstance
   
   (SmAssemblyInstance is a class to efficiently include multiple copies of a component into an assembly)

    The assembly component list is implemented using the pointers
      SmOwningTopology::m_pList,
      SmTopology::m_pNext, and
      SmTopology::m_pPrev.
    
    SmSAGObjects are added and removed from the Assembly component list
    using the inherited methods: 
       SmOwningTopology::InsertAfter(), SmOwningTopology::Remove(),
       SmOwningTopology::PreInsert(), and
       SmOwningTopology::PostInsert().

***********************************************************************/
class SM_EXPORT SmAssembly : public SmSAGObject
{
protected:
    // inherited:
    // SmOwningTopology::m_pList           - head pointer to this Assembly's doubly linked component list
    // SmOwningTopology::m_lListSize       - number of components in the assembly
    //
    // SmSAGObject::m_lDisplayListName     -
    // SmSAGObject::m_vDisplayParameters   -

    // Not used:
    // SmTopology::m_pListOwner            - 
    // SmTopology::m_pNext                 - 
    // SmTopology::m_pLast                 - 

    TCHAR*                        m_pName;          // name of this assembly
    //SmTArray<SmAssemblyInstance*> m_vInstances;     // list of uses of this assembly in other parent assemblies.
    //                                                // (This list managed by the SmAssemblyInstance constructor and destructor.)
    void                        * m_pUserPointer;   //

public:
    SmAssembly(const TCHAR* pName);

    virtual ~SmAssembly();

    virtual SmStatus BuildGraphicsStructure
    (
      const SmDisplayParameters & crDisplayParameters,   ///< [in] : graphics controlling parameters                                                      <br>
      SmDisplayList             & rDisplayList,          ///< [out]: displayList name for this Brep Graphics                                              <br>
      SmDisplayList             * pActiveDisplayListRef, ///< [out]: Ref to DisplayList Copy placed on the s_View.m_pActiveLists that actually gets drawn <br>
      SmGfxArraySet             * pOptGfxSet=NULL        ///< [in] : When given output GfxVertexArrays not GL calls.                                      <br>
    ) ;     

    //// Get assembly components (places members of m_pList linked list into an SmTArray)
    //void GetComponents(SmTArray<SmSAGObject*> & rComponents) const;

    // Get list of all AssemblyInstances in this Assembly
    void GetAssemblyInstances(SmTArray<SmAssemblyInstance*> & rInstances) const;

    // Get all components used in this assembly by walking the complete assembly tree
    SmStatus GetComponents(SmTArray<SmBrep*>& rBreps,
                           SmTArray<SmPolyBrep*>& rPolyBreps,
                           SmTArray<SmAssembly*>& rAssemblies); // increments an unlocked mark

    // Get all components used in this assembly by walking the complete assembly tree
    SmStatus Flatten(SmTArray<SmBrep*>& rBreps, SmTArray<SmPolyBrep*>& rPolyBreps);

    void SetUserPointer(void *pUserPointer) { m_pUserPointer = pUserPointer; }

    void * GetUserPointer() { return m_pUserPointer; }
    TCHAR* GetName() { return m_pName; }
    SmAssemblyInstance* GetOwningAssemblyInstance() { return (SmAssemblyInstance*)GetOwner(); }
    SmAssembly *GetParentAssembly()
    {
        SmAssembly* pParent = NULL;
        if (GetOwner())
        {
            pParent = (SmAssembly*)GetOwner()->GetOwner();
            NE(pParent);
        }
        return pParent;
    }

private:
    friend class SmAssemblyInstance;
    // Remember Assembly instance in some parent assembly (used only by SmAssemblyInstance constructor)
    void AddInstance(SmAssemblyInstance* pInstance) { PostInsert(pInstance); /*m_vInstances.AddUnique(pInstance);*/ }

    // Forget Assembly instance in some parent assembly (used only by SmAssemblyInstance destructor)
    void RemoveInstance(SmAssemblyInstance *pInstance) 
    { 
      SE(Remove(pInstance));
      //ULONG lFound; 
      //if (m_vInstances.FindElement(pInstance,lFound)) { m_vInstances.RemoveAt(lFound,1); }
      //else { SE(SM_ERR); }                                       
    }

public:
    virtual SmBoolean AssertValid
    (
      SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                            <br>
      SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        <br>
                                                ///<        SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    <br>
      SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  <br>
      SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  <br>
    ) const ;

    // obsolete
    //  virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmAssembly,SmSAGObject,SmAssembly_TYPE);  

    // Dump this Assembly and it's subassemblies
    void Dump(int lDepth) const;

    // Dump histograms of Assembly and Brep use in Instances
    void DumpUsageStats( ) const;

    // Go to the root Assembly, then dump
    void DumpWholeAssembly() const;

    // build and add Assembly display list for global display parameters to global display list array
    virtual SmDisplayList * Draw  
    (
      SmBoolean bAddToUIPickList=FALSE, ///< [in] :
      SmGfxArraySet * pOptGfxSet=NULL   ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters()
    ) const;

    SmStatus WriteAssemblyAsPartFile
  (const TCHAR         * cOutputFileName,                ///< [in] : target File name                                                  <br>
    SmFileType           eType = SM_ASCII,               ///< [in] : Specify output type: oneof                                        <br>
                                                         ///<        SM_ASCII  = Database is an ASCII file                             <br>
                                                         ///<        SM_BINARY = Database is a Binary format                           <br>
    SmBoolean            bWriteAsBSplines=FALSE,         ///< [in] : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes   <br>   
                                                         ///<        FALSE= Write native formats for nonBSplines                       <br>
    SmApproxTol3d        sApproxTol3d=SM_APPROX_TOL_3D   ///< [in] : only used when bWriteAsBSplines is TRUE                           <br>
                                                         ///<        default:[SM_APPROX_TOL_3D = SM_ZONE_TOL_3D/2 = 5.0e-6]            <br>
    ) const;

private:
    // internal draw function used recursively
    SmDisplayList * Draw  
    (
      SmBoolean bAddToUIPickList,                       ///< [in] :
      SmGfxArraySet * pOptGfxSet,                       ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters()
      SmMapTypeToType<SmBrep*, SmPolyBrep*> & rBrepMap, ///< [in] :
      SmTArray<const SmAxis2Placement*> &rTransformStack,     ///< [in] :
      SmTArray<SmVector3d> &rScaleStack                 ///< [in] :
    ) const;

public:

    void *operator new(size_t size, const SmContext & crContext);
    void  operator delete(void *ptr) { smos_Free(ptr); ptr = NULL ; }
    void  operator delete(void* ptr, const SmContext& ) { smos_Free(ptr); ptr = NULL; }
} ; // end class SmAssembly

#endif // !__SMASSEMBLY_H__




