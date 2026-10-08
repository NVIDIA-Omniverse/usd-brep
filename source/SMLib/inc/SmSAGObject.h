// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSAGObject.h
* PURPOSE: Header file for Stand Alone Geometry object class.
**********************************************************************/

#ifndef __SMSAGOBJECT_H__
#define __SMSAGOBJECT_H__

#ifndef __SMOWNINGTOPOLOGY_H__
#include <SmOwningTopology.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMGFX_EXTERN_H__
#include <SmGraphicsExtern.h>
#endif

// GWC:BIND_TEMPLATES_MOVE  SM_TARRAY_TEMPLATE_PREDECLARATION(SmAssembly*) ;

/*******************************************************************//**
PURPOSE: This is the base level objects for all objects which can
   stand on their own.  It is often used for assemblies.  This object
   keeps track of its graphics list and updates upon notification
   and when the display parameters change.

NOTES: 
***********************************************************************/
class SM_EXPORT SmSAGObject : public SmOwningTopology  // SAG = 'Stand alone with Graphics'
{
    friend class SmAssembly;
protected:
    SmDisplayList         m_sDisplayList;       // OpenGL display list name
                                                //   SM_BIG_ULONG = No Display List
    SmDisplayParameters   m_vDisplayParameters; // List of display parameters 

    // inherited:
    //      SmOwningTopology::m_pList;       -
    //      SmOwningTopology::m_lListSize;   -
    //      SmTopology::m_pListOwner;        -
    //      SmTopology::m_pNext;             -
    //      SmTopology::m_pLast;             -
                                              
public:                                       
    SmSAGObject() : m_sDisplayList(SM_BIG_ULONG) { }

    virtual ~SmSAGObject();

    virtual TCHAR* GetName()
    {
        SE_MSG(SM_ERR, _T("Asking SAGObject for a name (doesn't exist)."));
        return NULL;
    }

    virtual SmStatus BuildGraphicsStructure(const SmDisplayParameters& crDisplayParameters, // in : graphics controlling
                                                                                            // parameters
                                            SmDisplayList& rDisplayList, // out: displayList name for this Brep Graphics
                                            SmDisplayList* pActiveDisplayListRef, // out: Ref to DisplayList Copy placed
                                                                                  // on the s_View.m_pActiveLists that
                                                                                  // actually gets drawn
                                            SmGfxArraySet* pOptGfxSet = NULL // in : When given output GfxVertexArrays
                                                                             // not GL calls.
    );

    SmStatus RetrieveGraphicsStructure(const SmDisplayParameters& crDisplayParameters, // in : Tgt DisplayParameters
                                       SmDisplayList* pSavedDisplayList // out: Ptr to saved DisplayList or NULL
    );

    SmStatus SaveGraphicsStructure(const SmDisplayParameters& crDisplayParameters, SmDisplayList* pDisplayListToCopy);

    // used by SmCacheMgr::GetOrCreateObjectCache()
    // create a new or fetch an existing ObjectCache - place ObjectCache in m_pCacheObj
    SmStatus CacheMakeOrValidate(SmObjectCacheType eObjectCacheType, ///< NotUsed: [in] : oneof SM_OC_CURVE <br>
                                                                     ///<              SM_OC_SURFACE <br> SM_OC_TRIMSRF
                                                                     ///<              <br> SM_OC_BREP <br>
                                 const SmAObject* pObject, ///< [in] : target object <br>
                                 SmCacheObj* pOldCache, ///< [in] : existing target Object's ObjectCache or NULL <br>
                                 SmCacheObj*& rpNewCache ///< [out]: ptr to target object's ObjectCache <br>
    ) const;

    virtual void Notify(SmNotifyOperation eNotifyOperation, SmObject* pData1, SmObject* pData2, SmObject* pData3);

    virtual SmBoolean AssertValid(SmAssertArray* pAList = NULL, // i/o: Accumulating list of failed Asserts, NULL to
                                                                // ignore
                                  SmAssertTestLevel eTestLevel = SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests,
                                                                             // SM_LEVEL_1, SM_LEVEL_2=all tests,
                                                                             //      with implemented heal methods,
                                                                             //      SM_LEVEL_GIVEN = run tests in order
                                                                             //      requested in pTestRequests
                                                                             //      default:[SM_LEVEL_0]
                                  SmAssertWalking eWalkTree = SM_WALK, // NotUsed: in : SM_WALK = run AssertValid on any
                                                                       // topology graph descendants, SM_NO_WALK=don't,
                                                                       // default:[SM_WALK]
                                  SmTArray<ULONG>* pTestRequests = NULL // in : when eTestLevel == SM_LEVEL_GIVEN, run
                                                                        // these tests in this order, default:[NULL]
    ) const;

    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport& rAReport, SmAssertArray* pAList);

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmSAGObject, SmAObject, SmSAGObject_TYPE);
};


#endif // !__SMSAGOBJECT_H__
