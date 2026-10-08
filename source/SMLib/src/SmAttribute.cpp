// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmAttribute.cpp
* PURPOSE: Header file for Attribute object
**********************************************************************/

#include "StdAfx.h"

#include <SmAttribute.h>
#include <SmAssertArray.h>
#include <SmRegion.h>

#include <fstream>

#define GOTO_NEXT_ATTLINE \
{ULONG lCount = 0; signed char sCh; while ((sCh = (signed char)rFileIn.get()) != '\n') \
{ lCount ++; if (lCount > 1000) {ERR_MSG(_T("Corrupt file - No Carriage return in 1000 characters\n")); SER(SM_ERR);} \
if (sCh == EOF) {ERR_MSG(_T("Illegal End Of File - Corrupt file\n")); SER(SM_ERR);} continue;}}


//SmTArray<void*> * SmAttribute::m_pCallBacks = NULL;       

/*******************************************************************//**
PURPOSE: Returns TRUE when attribute is in AttrMap and in Attributes array
              outputs the attribute's Index value (range:[0,AttributeCount-1])
            else returns FALSE

NOTES: 
***********************************************************************/
SmBoolean sm_FindAttribute
  (SmAttribute                         * pAttributeToFind, // in : target attribute pointer
   const SmTArray<SmAttribute*>        & rAttributes,      // in : array of attributes to search
   SmMapTypeToType<SmAttribute*,ULONG> & rAttrMap,         // in : list of [attribute,index] pairs to search
   ULONG                               & rlFoundIndex)     // out: index of pAttributeToFind in rAttributes array.
{
  // init output
  rlFoundIndex = 0;

  // local
  ULONG lIndx = 0 ; 
  // void *pdata = NULL;  // secretly the ULONG index+1 value for attribute entries

  // look for attribute in AttrMap
  rAttrMap.Lookup(pAttributeToFind,lIndx);
  // rAttrMap.Lookup(pAttributeToFind,pdata);

  // no work - not in AttrMap
  if (lIndx == 0) { return FALSE; }
 // if (pdata == NULL) { return FALSE; }

  // convert map value (index+1) to index value 
  rlFoundIndex = lIndx - 1; 
  // rlFoundIndex = ((ULONG)pdata) - 1; 

  // search Attributes array for 
  for (ULONG i=0; i<rAttributes.GetSize(); i++) 
    { // RCLxx removed //
      if (pAttributeToFind == rAttributes[i]) 
        {
          rlFoundIndex = i;
          return TRUE;
        }
    }
  return FALSE;

} // end sm_FindAttribute

/*******************************************************************//**
PURPOSE: gather and place on output arrays the list of attributes 
  associated with the target pObject.

NOTES: 
***********************************************************************/
SmStatus sm_ExtractAttributes
  (const SmContext                     & crContext,          // NotUsed: in : current context for object creation
   const SmAObject                     * pObject,            // in : target object potentially containing attributes
   SmTArray<ULONG>                     & rIndexedAttributes, // out: rAttributes index array for this object's attributes
   SmTArray<SmAttribute*>              & rAttributes,        // i/o: accumulation of all attributes on all entities
   SmMapTypeToType<SmAttribute*,ULONG> & rAttrMap)           // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs
{
  SM_REF1(crContext) ;
  // init outputs
  rIndexedAttributes.ReSet() ;
  //  
  //  if(rIndexedAttributes) { delete rIndexedAttributes ; rIndexedAttributes = NULL ; }
  //  // rIndexedAttributes = NULL; // No attributes

  // get object's attributes
  SM_PTR_ARRAY(sAttr, SmAttribute, 16) ; // SmTArray<SmAttribute *>
  pObject->GetAttributes(sAttr);

  // when object has attributes
  if (sAttr.GetSize() > 0) 
    {
      // size output array
      rIndexedAttributes.SetDataSize( sAttr.GetSize() ) ;

      //  // Create a temporary array for attribute indices range:[0,attributeCount-1]
      //  SmTArray<ULONG> * pObjectAttributes = new(crContext) SmTArray<ULONG>(crContext);
      //  pObjectAttributes->SetDataSize(sAttr.GetSize());
      //  NER(pObjectAttributes);
      //  SmObjDelete sClean(pObjectAttributes);
      ULONG lTempCount = 0 ;

      // for every attribute
      for (ULONG i=0; i<sAttr.GetSize(); i++) 
        {
          // GWC:ADDED SM_AB_TEMP behavior
          // no work - skip attributes of type SM_AB_TEMP
          if(sAttr[i]->GetBehavior() == SM_AB_TEMP) 
            { lTempCount += 1 ;
              continue ;
            }

          // when attribute is already in the attribute list
          ULONG lFoundIndex;
          if (sm_FindAttribute(sAttr[i],rAttributes,rAttrMap,lFoundIndex)) 
            {
              // add the attribute index to the object attribute index list
              rIndexedAttributes.Add(lFoundIndex);
            }
          else // when attribute is not in the attribute list
            {
              // add it to the rAttributes array
              lFoundIndex = rAttributes.GetSize();
              rAttributes.Add(sAttr[i]);

              // add [attribute,ObjectAttributeIndex+1] pair to sAttrMap 
              ULONG lIndex = i+1-lTempCount;
              rAttrMap.SetAt(sAttr[i],lIndex);

              // place rAttributesIndex value into ObjectAttribute list
              rIndexedAttributes.Add(lFoundIndex);
            }
        } // end iter every attribute

      // set output
      //  sClean.Clear();
      //  rIndexedAttributes = pObjectAttributes;
    
    } // end attribute existence check

  return SM_SUCCESS;

} // end sm_ExtractAttributes
            
/*******************************************************************//**
PURPOSE: Write the total number of attributes indices and
  each index value listed in pAttributes to a file

NOTES: This function does not output the attribute values, it
                only outputs the total number of indices listed 
                in pAttributes array and those index values.
***********************************************************************/
SmStatus SmAttributeData::WriteIndexedAttributesToDB
  (SmTArray<ULONG> & rAttributes,   // in : list of attribute indices
   SmDatabaseIO    & rDB)           // in : contains target stream
{
  SmFileType eType = rDB.GetFileType();

  // switch on file type
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      if (rAttributes.GetSize() > 0) 
        {
          // output total number of attributes
          rFileOut << "            " << rAttributes.GetSize() << " ";

          // list each attribute index
          for (ULONG i=0; i<rAttributes.GetSize(); i++) 
            {
              long lIndex = rAttributes[i];
              rFileOut << lIndex << " ";
            }
        }
      else 
        {
          rFileOut << "            0 ";
        }
      rFileOut << "\n";    
    }
  else //Binary
    {
      if (rAttributes.GetSize() > 0) 
        {
          ULONG lSize = rAttributes.GetSize();
          ULONG *pData = rAttributes.GetDataArray();
          SER(rDB.WriteLong(lSize));
          SER(rDB.WriteLongs(pData,lSize));
        }
      else 
        {
          ULONG lSize = 0;
          SER(rDB.WriteLong(lSize));
        }
      }

  return SM_SUCCESS;

} // end SmAttributeData::WriteIndexedAttributesToDB

/*******************************************************************//**
PURPOSE: Read the indexes of the attributes from a file

NOTES: 
***********************************************************************/
SmStatus SmAttributeData::ReadIndexedAttributesFromDB
  (const SmContext        & crContext,       // in : context for new object construction
   SmTArray<SmAttribute*> & rAllAttributes,  // out: accumulation array of all attribute objects
                                             //      initialized with a dummy attribute object for every
                                             //      attribute index value seen.
   SmTArray<ULONG>        & rAttributes,     // out: array of newly allocated attribute index values
   SmDatabaseIO           & rDB)             // in : contains target I/O stream
{
  // init output
  rAttributes.ReSet() ;
  //  if(rpAttributes) { delete rpAttributes ; rpAttributes = NULL ; }

  // locals
  SmFileType eType = rDB.GetFileType();
  ULONG lCnt;

  // get number of attributes
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> lCnt;
    }
  else //Binary
    {
      SER(rDB.ReadLong(lCnt));
    }

  // no work - no attributes
  if (lCnt == 0)
    {
      // rpAttributes = NULL;
      if (eType == SM_ASCII) 
        {
          std::istream & rFileIn = *rDB.GetInStreamPtr();
          GOTO_NEXT_ATTLINE;
        }
      return SM_SUCCESS;
    } // end no attribute check

  // allocate an index for each attribute
  // rpAttributes = new(crContext) SmTArray<ULONG>(crContext);
  rAttributes.SetSize(lCnt);
  ULONG *pData = rAttributes.GetDataArray();

  // read the attribute index array
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      for (ULONG i=0; i<lCnt; i++) { rFileIn >> pData[i];
                                     }
      GOTO_NEXT_ATTLINE;
    }
  else 
    {//Binary
      SER(rDB.ReadLongs(pData,lCnt));
    }

  // for every attribute 
  //   make sure the AllAttributes array is big enough to hold the index values
  //   make sure every index value is allocated a dummy attribute object
  for (ULONG i=0; i<rAttributes.GetSize(); i++) 
    {
      ULONG lIndex = rAttributes[i];

      // make room in rAllAttributes for every index value
      if (rAllAttributes.GetSize() < lIndex+1) 
        {
          rAllAttributes.SetSize(lIndex+1);
        }

      // allocate an attribute for every attribute index value
      if (rAllAttributes[lIndex] == NULL) 
        {
          rAllAttributes[lIndex] = new (crContext) SmAttribute(SM_AI_UNKNOWN,SM_AB_STANDALONE_REFERENCE);
        }
    }

  return SM_SUCCESS;

} // end SmAttributeData::ReadIndexedAttributesFromDB

/*******************************************************************//**
PURPOSE: Static method to create attributes for those attribute types
   that we know about.  Used to implement attribute persistence.  
   This function is called from SmBrepData::ReadAttributeFromDB to
   create attributes from data stored in data files.

NOTES: 
***********************************************************************/
SmAttribute * SmAttribute::CreateAttribute
 (const SmContext        & crContext,
  ULONG                    lAttributeID,
  SmAttributeBehaviorType  eBehavior,
  const SmTArray<long>   & rLongs,
  const SmTArray<double> & rDoubles,
  const SmTArray<char>   & rChars)
{
  // Try to create the attribute using registered callbacks
  SmTArray<void*> * pCallBacks = (SmTArray<void*> *)crContext.GetAttributeCallbacks();
  if (pCallBacks) 
    {
      for (ULONG i=0; i<pCallBacks->GetSize(); i++) 
        {
          SmAttribute* (*pCallBack)(const SmContext        & crContext,
                                    ULONG                    m_lAttributeID,
                                    SmAttributeBehaviorType  eBehavior,
                                    const SmTArray<long>   & rLongs,
                                    const SmTArray<double> & rDoubles,
                                    const SmTArray<char>   & rChars) = 
              (SmAttribute* (*)(const SmContext        & crContext,
                                ULONG                    m_lAttributeID,
                                SmAttributeBehaviorType  eBehavior,
                                const SmTArray<long>   & rLongs,
                                const SmTArray<double> & rDoubles,
                                const SmTArray<char>   & rChars)) (*pCallBacks)[i];
                
          SmAttribute *pAttr = pCallBack(crContext,lAttributeID,eBehavior,rLongs,rDoubles,rChars);
          if (pAttr) return pAttr;
        }
    }

  SmAttribute *pRet = NULL;

  switch (lAttributeID) 
    {
      case SM_AI_COLOR:
        {
          SmAttribute *pAttr = new (crContext) SmVector3dAttribute(lAttributeID,
                                                                   SmVector3d(rDoubles[0],
                                                                              rDoubles[1],
                                                                              rDoubles[2]),
                                                                   eBehavior);
          pRet = pAttr;
        }
        break;

      case SM_AI_TAG:
        {
          SmAttribute *pAttr = new (crContext) SmTagAttribute(lAttributeID,
                                                              rLongs[0],rLongs[1],
                                                              rLongs[2],rLongs[3],
                                                              rLongs[4],rLongs[5],
                                                              rLongs[6],rLongs[7],
                                                              eBehavior);
          pRet = pAttr;
        }
        break;

      default:
        {
          // Just create a generic attribute
          SmAttribute *pGenAttr = new (crContext) SmGenericAttribute(lAttributeID,eBehavior,
                                                                     rLongs,rDoubles,rChars);
          pRet = pGenAttr;
        }
    }
  return pRet;

}  // end SmAttribute::CreateAttribute

/*******************************************************************//**
PURPOSE: Default attribute handler for splitting an object.
   It creates a new copy of the attribute and puts it into the
   attribute list of the new object.

NOTES: 
***********************************************************************/
void SmAttribute::Split
 (SmAObject *pSplitObject,    // in : Object being split (the object who owns this attribute)
  SmAObject *pChild1,         // in : child1, gets copy of this attribute when needed
  SmAObject *pChild2)         // in : child2, gets copy of this attribute when needed
{
  SM_ASSERT(pChild1 != NULL);
  SM_ASSERT(pChild2 != NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pSplitObject) { smos_WriteBuffer(_T("\n pSplitObject attributes: ")); pSplitObject->SmAObject::Dump() ; }
      if(pChild1)      { smos_WriteBuffer(_T("\n pChild1 attributes: ")); pChild1->SmAObject::Dump() ; }
      if(pChild2)      { smos_WriteBuffer(_T("\n pChild2 attributes: ")); pChild2->SmAObject::Dump() ; }
    }
#endif // SM_DEBUG_CODE

  // when child1 is not the reused SplitObject
  if (pChild1 != pSplitObject) 
    {
      // locals
      SmAttribute *pOld = pChild1->FindAttribute(m_lAttributeID);

      // when Child1 does not already have the attribute
      if (!pOld) 
        {
          // when attribute requires a deep copy
          if (   m_eBehavior == SM_AB_COPY
              || m_eBehavior == SM_AB_STANDALONE_COPY) 
            {
              // add deep copy of attribute to child1
              const SmContext * pContext = pChild1->GetContext();
              SmAttribute     * pNew1    = MakeCopy(*pContext);
              pChild1->AddAttribute(pNew1);
            }

          // when attribute requires a referenced copy
          else if (   m_eBehavior == SM_AB_REFERENCE
                   || m_eBehavior == SM_AB_STANDALONE_REFERENCE) 
            {
              // add reference copy of attribute to child1
              pChild1->AddAttribute(this);
            }
         } // end this attrib not already on pChild1 check
       } // end child1 is not the reused SplitObject check

  // when child2 is not the reused SplitObject
  if (pChild2 != pSplitObject) 
    {
      // locals
      SmAttribute *pOld = pChild2->FindAttribute(m_lAttributeID);
          
      // when Child2 does not already have the attribute
      if (!pOld) 
        {
          // when attribute requires a deep copy
          if (   m_eBehavior == SM_AB_COPY 
              || m_eBehavior == SM_AB_STANDALONE_COPY) 
            {
              // add deep copy of attribute to child1
              const SmContext * pContext = pChild2->GetContext();
              SmAttribute     * pNew2    = MakeCopy(*pContext);
              pChild2->AddAttribute(pNew2);
            }
      
          // when attribute requires a referenced copy
          else if (   m_eBehavior == SM_AB_REFERENCE 
                   || m_eBehavior == SM_AB_STANDALONE_REFERENCE) 
            {
              // add reference copy of attribute to child1
              pChild2->AddAttribute(this);
            }
        } // end this attrib not already on pChild2 check
    } // end child2 is not the reused SplitObject check

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pSplitObject) { smos_WriteBuffer(_T("\n pSplitObject attributes: ")); pSplitObject->SmAObject::Dump() ; }
      if(pChild1)      { smos_WriteBuffer(_T("\n pChild1 attributes: ")); pChild1->SmAObject::Dump() ; }
      if(pChild2)      { smos_WriteBuffer(_T("\n pChild2 attributes: ")); pChild2->SmAObject::Dump() ; }
    }
#endif // SM_DEBUG_CODE

} // end SmAttribute::Split

/*******************************************************************//**
PURPOSE: Default attribute handler for copying an object.
   It creates a new copy of the attribute and puts it into the
   attribute list of the new object.

NOTES: 
  don't copy duplicates:  When the ToObj has an Attribute with an 
    attributeID equal to the this attributeID then skip adding a 
    copy of this attribute to the pToObj
***********************************************************************/
void SmAttribute::Copy         // called on attribute from pFromObj
  (const SmAObject *,          // in : pFromObj - Object being copied from  
   SmAObject       *pToObj)    // in : Object being copied into
{
    SM_ASSERT(pToObj != NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pToObj) { smos_WriteBuffer(_T("\n ToObj attributes: ")); pToObj->SmAObject::Dump() ; }
    }
#endif // SM_DEBUG_CODE

  // locals
  SmAttribute *pOld = pToObj->FindAttribute(m_lAttributeID);

  // when this attribute is not already on the pToObj
  if (!pOld) 
    {
      // for Copy behaviors
      if (   m_eBehavior == SM_AB_COPY
          || m_eBehavior == SM_AB_STANDALONE_COPY) 
        {
           // add a deep copy of this to the ToObj
           const SmContext * pContext = pToObj->GetContext();
           SmAttribute     * pNew     = MakeCopy(*pContext);
           pToObj->AddAttribute(pNew);
        }
      else if (   m_eBehavior == SM_AB_REFERENCE
               || m_eBehavior == SM_AB_STANDALONE_REFERENCE) 
        {
          // add a reference to this to the ToObj 
          pToObj->AddAttribute(this);
        }
    } // end attribute is not yet on pToObj check

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pToObj) { smos_WriteBuffer(_T("\n ToObj attributes: ")); pToObj->SmAObject::Dump() ; }
    }
#endif // SM_DEBUG_CODE

} // end SmAttribute::Copy

/*******************************************************************//**
PURPOSE: Default attribute handler for merging two objects.
    If the remaining object does not have the same attribute type
    as this attribute->m_lAttributeType, then copy the attribute to the new one.

NOTES: 
***********************************************************************/
void SmAttribute::Merge
  (SmAObject *, // pOrigObject1 
   SmAObject *, // pOrigObject2
   SmAObject * pMergedResult)
{
  // check state - pMergedResult must be NonNULL
  SM_ASSERT(pMergedResult != NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pMergedResult) { smos_WriteBuffer(_T("\n MergedResult attributes: ")); pMergedResult->SmAObject::Dump() ; } 
    }
#endif // SM_DEBUG_CODE

  // When Merge Result does not already have an attribute of this->m_lAttributeType
  if (pMergedResult->FindAttribute(m_lAttributeID) == NULL) 
    {
      // Copy (with appropriate behavior) this Attribute onto pMergedResult 
      if (   m_eBehavior == SM_AB_COPY
          || m_eBehavior == SM_AB_STANDALONE_COPY) 
        {
          const SmContext * pContext = pMergedResult->GetContext();
          SmAttribute     * pNew     = MakeCopy(*pContext);
          pMergedResult->AddAttribute(pNew);
        }
      else if (   m_eBehavior == SM_AB_REFERENCE
               || m_eBehavior == SM_AB_STANDALONE_REFERENCE) 
        {
          pMergedResult->AddAttribute(this);
        }

    } // end this attrib not already on pMergedResult check

  // check state - pMergedResult must be NonNULL

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pMergedResult) { smos_WriteBuffer(_T("\n MergedResult attributes: ")); pMergedResult->SmAObject::Dump() ; } 
    }
#endif // SM_DEBUG_CODE

} // end SmAttribute::Merge

/*******************************************************************//**
PURPOSE: Default attribute handler for coincident topology objects 
    found during a Boolean operation.
    If the remaining object does not have the same attribute type
    as this attribute->m_lAttributeType, then copy the attribute to the new one.

NOTES:      this = an attribute from the pFromObj 
       e.g. this = & pFromObj->GetAttributes().GetAt(ii) ;   // for some valid value of ii
***********************************************************************/
void SmAttribute::Coincident
  (SmAObject * pFromObj,    // in : FromObj of the FromObj-ToObj Coincident Topology pair 
   SmAObject * pToObj,      // NotUsed: in : ToObj   of the FromObj-ToObj Coincident Topology pair 
   SmAObject * pFromBrep,   // NotUsed: in : Brep containing pFromObj
   SmAObject * pToBrep)     // in : Brep containing pToObj
{
  SM_REF3(pFromObj, pFromBrep, pToBrep );

  // check state - pToObj must be NonNULL
  SM_ASSERT(pToObj != NULL);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pFromObj) { smos_WriteBuffer(_T("\n From Object attributes: ")); pFromObj->SmAObject::Dump() ; }
      if(pToObj)   { smos_WriteBuffer(_T("\n To Object attributes: ")); pToObj->SmAObject::Dump() ; }
    }
#endif // SM_DEBUG_CODE

  // When Coincident ToObj does not already have an attribute of this->m_lAttributeType
  if (pToObj->FindAttribute(m_lAttributeID) == NULL) 
    {
      // Copy (with appropriate behavior) this Attribute onto pToObj 
      if (   m_eBehavior == SM_AB_COPY
          || m_eBehavior == SM_AB_STANDALONE_COPY) 
        {
          const SmContext * pContext = pToObj->GetContext();
          SmAttribute     * pNew     = MakeCopy(*pContext);
          pToObj->AddAttribute(pNew);
        }
      else if (   m_eBehavior == SM_AB_REFERENCE
               || m_eBehavior == SM_AB_STANDALONE_REFERENCE) 
        {
          pToObj->AddAttribute(this);
        }

    } // end need to copy attribute check

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pFromObj) { smos_WriteBuffer(_T("\n From Object attributes: ")); pFromObj->SmAObject::Dump() ; }
      if(pToObj)   { smos_WriteBuffer(_T("\n To Object attributes: ")); pToObj->SmAObject::Dump() ; }
    }
#endif // SM_DEBUG_CODE

} // end SmAttribute::Coincident

/*******************************************************************//**
PURPOSE: Default attribute handler for Editing an object.
    It does nothing.

NOTES: 
***********************************************************************/
void SmAttribute::Edit(SmAObject * /* pEditedObject */)
{

} // end SmAttribute::Edit

/*******************************************************************//**
PURPOSE: Default attribute handler for destroying an object.
    Please note that destruction of attribute will not occur here
    but will occur when destructor for object calls delete on the
    attribute.

NOTES: 
***********************************************************************/
void SmAttribute::Destruction(SmAObject * /* pToBeDeleted */)
{

} // end SmAttribute::Destruction

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by SmAttribute.

NOTES:
***********************************************************************/
ULONG SmAttribute::GetMemoryUsed   // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated)   // out: bigger size of all allocated memory in bytes
  const
{
  // get referenced memory
  ULONG lAllocated ;
  ULONG lUsed      = m_vUsers.GetMemoryUsed(lAllocated) ;

  // return total - avoid counting contained SmTARRAY memory twice
  rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<SmAObject*>) ;
  return(             sizeof(this) + lUsed      - sizeof(SmTArray<SmAObject*>)) ;

} // end SmAttribute::GetMemoryUsed

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertAttribute_list[] =
{
  {SM_AT_POINTER, _T("Context"), _T("m_vUsers share the same context") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmAttribute::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;// init rtn value
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmObject::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // SmAttribute and the objects it attaches to need to share common contexts
  ULONG ii ;
 // mUsersMutex.lock();
  for(ii=0;ii<m_vUsers.GetSize();ii++) {
      if (m_vUsers[ii])
      {
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0,
                                           (m_eBehavior == SM_AB_REFERENCE || m_eBehavior == SM_AB_STANDALONE_REFERENCE ||
                                            GetContext() == m_vUsers[ii]->GetContext()),
                                           _T("")) ;
      }
  }
  // mUsersMutex.unlock();
  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmAttribute::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmAttribute::AssertHeal
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
//       return ( SmObject::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmAttribute::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmAttribute::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: SmMergeRegionAttribute constructor

NOTES:  hard codes 
          AttributeID:[SM_AI_OTHER_REGION_ID] and 
          Behavior   :[SM_AB_COPY]
***********************************************************************/
SmMergeRegionAttribute::SmMergeRegionAttribute
 (SmRegion * pThisSourceRegion,      // in : This Brep source Region (as input - before any splits) 
  ULONG      lThisSourceType,        // in : Associated This Source Region type
                                     //      magic numbers: 0 = Internal Solid
                                     //                     1 = Internal Void
                                     //                     2 = Infinite Region
  SmRegion * pOtherSourceRegion,     // in : OtherBrep source Region (as input)
  ULONG      lOtherSourceType)       // in : Associated Other Source Region type
                                     //      magic numbers: 0 = Internal Solid
                                     //                     1 = Internal Void
                                     //                     2 = Infinite Region
: SmAttribute( SM_AI_OTHER_REGION_ID, SM_AB_COPY ),
  m_eOperation(SM_BO_UNKNOWN)
    
{ 
  SM_ASSERT_MSG(pThisSourceRegion && pOtherSourceRegion, _T("SmMergeRegionAttribute constructor given bad NULL input pointers")) ; 
  SM_ASSERT_MSG(lThisSourceType <= 2 && lOtherSourceType <=2, _T("SmMergeRegionAttribute constructor given bad NULL input pointers")) ; 

  m_vThisSourceRegions.Add(pThisSourceRegion) ;
  m_vThisSourceType.Add(lThisSourceType) ;

  m_vOtherSourceRegions.Add(pOtherSourceRegion) ;
  m_vOtherSourceType.Add(lOtherSourceType) ;

} // end SmMergeRegionAttribute constructor

/*******************************************************************//**
PURPOSE: SmMergeRegionAttribute is a special use Merge::ManifoldBoolean() helper
 attribute having it's own nonDefault Merge() behavior.

NOTES: Uniquely Appends the Region lists of the two Regions being merged
 to make sure the output Region has a list of all the original ThisBrep
 and OtherBrep regions that share points with the Output region.
***********************************************************************/
void SmMergeRegionAttribute::Merge
  (SmAObject * pOrigObject1,  // in : ThisBrep source Region1
   SmAObject * pOrigObject2,  // in : ThisBrep source Region2
   SmAObject * pMergedResult) // in : ThisBrep output Region (expected to be the same as one of the OrigRegions)
{
  // check state - inputs must be NonNULL
  SM_ASSERT(   (pOrigObject1  != NULL) && (pOrigObject2  != NULL) && (pMergedResult != NULL)) ;
  if(   (pOrigObject1  == NULL) || (pOrigObject2  == NULL) || (pMergedResult == NULL))
    { return ; }

  // ThisRegions which have been split have SmMergeRegionAttributes
  // ThisRegions which have not been split don't. 
  // 
  // This method runs when a ThisRegion previously split is merged 
  //   with a 2nd ThisRegion which may or may not hve been previously split
  //   when faces are removed as part of the Boolean operation.  

  // locals
  ULONG ii ;
  SmRegion * pRegion1   = (SmRegion*)pOrigObject1 ;
  SmRegion * pRegion2   = (SmRegion*)pOrigObject2 ; 
  SmRegion * pResRegion = (SmRegion*)pMergedResult ;
  SmBrep   * pBrep    = pRegion1->GetBrep() ;

  // no work - pOrigObject1 and pOrigObject2 are regions from different Breps 
  if(pBrep != pRegion2->GetBrep())
    { return ; }

  // locals
  SmMergeRegionAttribute * pOutputRegAtt = (SmMergeRegionAttribute*)(pMergedResult->FindAttribute(SM_AI_OTHER_REGION_ID)) ;
  SmMergeRegionAttribute * pInputRegAtt1 = (SmMergeRegionAttribute*)(pOrigObject1->FindAttribute(SM_AI_OTHER_REGION_ID)) ;
  SmMergeRegionAttribute * pInputRegAtt2 = (SmMergeRegionAttribute*)(pOrigObject2->FindAttribute(SM_AI_OTHER_REGION_ID)) ;

  // check state assumption; this object will be the MergeAttribute on one of the input objects
  SM_ASSERT_MSG((this == pOutputRegAtt) || (this == pInputRegAtt1) || (this == pInputRegAtt2),
                _T("SmMergeRegionAttribute::Merge assumption is wrong. this != MergedResult->Attribute.  Method needs rewrite")) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { 
      if(pInputRegAtt1) { smos_WriteBuffer(_T("\n pMergeInput1 attribute: ")); pInputRegAtt1->Dump() ; }
      if(pInputRegAtt2) { smos_WriteBuffer(_T("\n pMergeInput2 attribute: ")); pInputRegAtt2->Dump() ; }
      smos_WriteBuffer(_T("\n MergeResult attributes: ")); this->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // logic:
  // 6 cases: Reg1->HasMergeAttrib, Reg2->HasMergeAttrib, Reg1 == ResReg : Append Reg2AttLists to ResRegAttLists
  //          Reg1-> NoMergeAttrib, Reg2->HasMergeAttrib, Reg1 == ResReg : Copy R2AttLists to ResRegAttLists, Add R1 to ResRegThisSource
  //          Reg1->HasMergeAttrib, Reg2-> NoMergeAttrib, Reg1 == ResReg : Add R2 to ResRegThisSource
  //          Reg1->HasMergeAttrib, Reg2->HasMergeAttrib, Reg2 == ResReg : Append Reg1AttLists to ResRegAttLists
  //          Reg1-> NoMergeAttrib, Reg2->HasMergeAttrib, Reg2 == ResReg : Add R1 to ResRegThisSource
  //          Reg1->HasMergeAttrib, Reg2-> NoMergeAttrib, Reg2 == ResReg : Copy R1AttLists to ResRegAttLists, Add R2 to ResRegThisSource   

  // ASSUMES ResReg will equal one of the input Regions
  SM_ASSERT_MSG(pRegion1 == pResRegion || pRegion2 == pResRegion, _T("SmMergeRegionAttribute::Merge assumption violated, algorithm needs extension")) ;

  // when both input regions have mergeAttributes
  if(pInputRegAtt1 && pInputRegAtt2)
    {
      // append arrays of the NonResult region into the ResRegion attribute
      SmMergeRegionAttribute * pAddAtt = (pRegion1 == pResRegion) ? pInputRegAtt2 : pInputRegAtt1 ;

      // propagate the ThisBrep Region data
      for(ii=0;ii<pAddAtt->m_vThisSourceRegions.GetSize();ii++)
        {
          SmRegion * pThisRegion = pAddAtt->m_vThisSourceRegions[ii] ;

          // When value is added uniquely
          if(pThisRegion && pOutputRegAtt->m_vThisSourceRegions.AddUnique(pThisRegion))
            {
              // keep the source lists associated
              pOutputRegAtt->m_vThisSourceType.Add(pAddAtt->m_vThisSourceType[ii]) ;
            } // end added a unique ThisBrep Region check
        } // end iter all OrigObj ThisBrep Regions

      // propagate the OtherBrep Region data
      for(ii=0;ii<pAddAtt->m_vOtherSourceRegions.GetSize();ii++)
        {
          SmRegion * pOtherRegion = pAddAtt->m_vOtherSourceRegions[ii] ;

          // When value is added uniquely
          if(pOtherRegion && pOutputRegAtt->m_vOtherSourceRegions.AddUnique(pOtherRegion))
            {
              // keep the source lists associated
              pOutputRegAtt->m_vOtherSourceType.Add(pAddAtt->m_vOtherSourceType[ii]) ;
            } // end added a unique OtherBrep Region check
        } // end iter all OrigObj OtherBrep Regions
    } // end both regions have MergeAttributes branch
  else // just one Region has a MergeAttribute
    {
      // when MergedResult does not have an SmMergeRegionAttribute give it a copy of this attribute
      if(pOutputRegAtt == NULL)
        {
          const SmContext * cpContext =   pMergedResult->GetContext() ? pMergedResult->GetContext()
                                        : pOrigObject1->GetContext()  ? pOrigObject1->GetContext()
                                        : pOrigObject1->GetContext() ;
          SM_ASSERT(cpContext != NULL) ;
          SM_ASSERT(   (pInputRegAtt1 == this && pInputRegAtt2 == NULL && pMergedResult == pOrigObject2)
                    || (pInputRegAtt2 == this && pInputRegAtt1 == NULL && pMergedResult == pOrigObject1)) ;

          pOutputRegAtt = new (*cpContext) SmMergeRegionAttribute(*this) ;
          SM_ASSERT(pOutputRegAtt != NULL) ;
          pMergedResult->AddAttribute(pOutputRegAtt) ;

        } // end need to create a new attribute for pMergedResult check

      // Add the Region without an array to the ResRegion Attribute ThisSourceRegions and types lists
      SmRegion *pAddRegion = (pInputRegAtt1 == NULL) ? pRegion1 : pRegion2 ;
      pOutputRegAtt->GetThisRegions().Add(pAddRegion) ;
      pOutputRegAtt->GetThisRegionTypes().Add(  (pAddRegion->IsInfiniteRegion()) ? 2
                                               : pAddRegion->IsVoid()            ? 1  
                                               :                                 0) ;
    } // end just one Region has a MergeAttribute branch

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      if(pInputRegAtt1) { smos_WriteBuffer(_T("\n pMergeInput1 attribute: ")); pInputRegAtt1->Dump() ; }
      if(pInputRegAtt2) { smos_WriteBuffer(_T("\n pMergeInput2 attribute: ")); pInputRegAtt2->Dump() ; }
      smos_WriteBuffer(_T("\n MergeResult attributes: ")); this->Dump() ;
    }
#endif // SM_DEBUG_CODE

} // end SmMergeRegionAttribute::Merge

/*******************************************************************//**
PURPOSE: Check Src Regions Solid/Void bits for any Solid ones

NOTES: return TRUE  = At least one Solid Source Region in the requested set,
              FALSE = No Solid regions
***********************************************************************/
SmBoolean  SmMergeRegionAttribute::HasAnySolid
 (ULONG lThisOtherFlag)  // in : 1=ThisSrcs, 2=OtherSrcs, 3=Both This & Other Sources
 const 
{ 
  // When asked - return TRUE if any ThisBrep Source Regions are Solid
  if(lThisOtherFlag & 1) 
    { for(ULONG ii=0;ii<m_vThisSourceType.GetSize();ii++)   
        { if(m_vThisSourceType.GetAt(ii) == 0) return TRUE ; }
    }

  // when asked - return TRUE if any OtherBrep Source Regions Are Solid
  if(lThisOtherFlag & 2) 
    { for(ULONG ii=0;ii<m_vOtherSourceType.GetSize();ii++)   
        { if(m_vOtherSourceType.GetAt(ii) == 0) return TRUE ; }
    }

  // arrive here when all checke3d source regions are void
  return FALSE ;

} // end SmMergeRegionAttribute::HasAnySolid 

/*******************************************************************//**
PURPOSE: Check Src Regions Solid/Void bits for any Void ones

NOTES: return TRUE  = At least one Void Source Region in the requested set,
              FALSE = No Void regions
***********************************************************************/
SmBoolean  SmMergeRegionAttribute::HasAnyVoid
 (ULONG lThisOtherFlag)  // in : 1=ThisSrcs, 2=OtherSrcs, 3=Both This & Other Sources
 const 
{ 
  // When asked - return TRUE if any ThisBrep Source Regions are Solid
  if(lThisOtherFlag & 1) 
    { for(ULONG ii=0;ii<m_vThisSourceType.GetSize();ii++)   
        { if(m_vThisSourceType.GetAt(ii) >= 1) return TRUE ; }
    }

  // when asked - return TRUE if any OtherBrep Source Regions Are Solid
  if(lThisOtherFlag & 2) 
    { for(ULONG ii=0;ii<m_vOtherSourceType.GetSize();ii++)   
        { if(m_vOtherSourceType.GetAt(ii) >= 1) return TRUE ; }
    }

  // arrive here when all checke3d source regions are void
  return FALSE ;

} // end SmMergeRegionAttribute::HasAnyVoid                                     

/*******************************************************************//**
PURPOSE: Check Src Regions Solid/Void bits for any PropagedThourghBoolean attributes

NOTES: return TRUE  = At least one PropagedThourghBoolean attribute in the requested set,
              FALSE = No Void regions

       Only check the m_vOtherSourceRegions Region list because
       the m_vThisSourceRegions list may contain stale pointers
***********************************************************************/
SmBoolean  SmMergeRegionAttribute::HasAnyOtherPropagatedAttribs()
 const 
{ 
  // locals 
  ULONG ii, jj ;
  SmTArray<SmAttribute *> sRegAttributes ;

  // for every OtherBrep Source Region
  for(ii=0;ii<m_vOtherSourceRegions.GetSize();ii++)   
    { 
      m_vOtherSourceRegions[ii]->GetAttributes(sRegAttributes) ;

      // For every Reg->Attribute
      for(jj=0;jj<sRegAttributes.GetSize();jj++)
        {
          // return TRUE when attribute is Propagated through booleans
          if(sRegAttributes[jj]->IsPropagatedThroughBooleans())
            { return TRUE ; }

        } // end iter every Reg Attribute
    } // end iter every region

  // arrive here when all checke3d other regions have no Propogated attributes
  return FALSE ;

} // end SmMergeRegionAttribute::HasAnyOtherPropagatedAttribs                                     

/*******************************************************************//**
PURPOSE: set SmMergeRegionAtribute stored values for a given existing index

NOTES: when given Index is not valid, does not work and returns error 
***********************************************************************/
SmStatus SmMergeRegionAttribute::SetRegions
  (ULONG      lIndx,                // in : SmMergeRegionAttribute data array Index target
   SmRegion * pThisSourceRegion,    // in : This  Source region to set for given index
   ULONG      lThisSourceType,      // in : This  Source type   to set for given index
   SmRegion * pOtherSourceRegion,   // in : Other Source region to set for given index
   ULONG      lOtherSourceType)     // in : Other Source type   to set for given index
{
  // check state - lIndx is valid
  if(lIndx >= m_vThisSourceRegions.GetSize())        
    { return(SM_ERR) ; }

  // could check state - after upcoming Sets, the SourceRegion lists remain unique

  // set values
  m_vThisSourceRegions .SetAt(lIndx, pThisSourceRegion) ;
  m_vThisSourceType    .SetAt(lIndx, lThisSourceType) ;
  m_vOtherSourceRegions.SetAt(lIndx, pOtherSourceRegion) ;
  m_vOtherSourceType   .SetAt(lIndx, lOtherSourceType) ;

  // all done
  return(SM_SUCCESS) ;

} // end SmMergeRegionAttribute::SetRegions

/*******************************************************************//**
PURPOSE: return first TgtAttribId attribute found in the OtherSrcRegions list

NOTES: sets output to owner OtherRegion
***********************************************************************/
SmAttribute * SmMergeRegionAttribute::HasOtherTgtAttribs
( ULONG       lTgtAttribID,   // in : Tgt AttributeID
 SmRegion *& pOtherRegion )   // out: pRegion owner of the returned Attribute
  const
{
  SmAttribute *pOtherAttrib = NULL;

  // itereate all OtherSourceRegions looking for a matching attribute
  for(ULONG ii = 0; ii < m_vOtherSourceRegions.GetSize(); ii++)
  {
    // return the first found matching attribute
    pOtherAttrib = m_vOtherSourceRegions[ii]->FindAttribute( lTgtAttribID );
    if(pOtherAttrib)
    {
      pOtherRegion = m_vOtherSourceRegions[ii];
      return pOtherAttrib;
    }
  }

  // arrive here when no attributes were found
  return NULL;

} // end SmMergeRegionAttribute::HasOtherTgtAttribs


/*******************************************************************//**
PURPOSE: SmColorAttribute Overwritten attribute handler for merging two objects.

NOTES: if both objects have SmColorAttributes - assign an average color to pMergeResult.
       if one object has an SmColorAttribute - copy it to pMergeResult
       if neither object has an SmColorAttribute (shouldn't happen) do nothing.

       Depends on Notify() only calling Merge() once per merge for this combination of Objects
***********************************************************************/
void SmColorAttribute::PropagateRegionsThroughBooleans
  (SmAObject * pOrigObject1,    // in : This Brep Source Region
   SmAObject * pOrigObject2,    // in : original Obj2 being merged
   SmAObject * pMergedResult)   // in : Result Region with a SmMergeRegionAttribute
{
  // check state - pMergedResult must be NonNULL
  SM_ASSERT(pOrigObject1  != NULL) ;
  SM_ASSERT(pOrigObject2  != NULL) ;
  SM_ASSERT(pMergedResult != NULL) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pMergedResult) { smos_WriteBuffer(_T("\n MergedResult attributes: ")); pMergedResult->SmAObject::Dump() ; } 
    }
#endif // SM_DEBUG_CODE

  // locals
  SmColorAttribute * pColorObj1  = (SmColorAttribute *)pOrigObject1->FindAttribute(SM_AI_COLOR) ;
  SmColorAttribute * pColorObj2  = (SmColorAttribute *)pOrigObject2->FindAttribute(SM_AI_COLOR) ;
  SmColorAttribute * pColorMerge = (SmColorAttribute *)pMergedResult->FindAttribute(SM_AI_COLOR) ;

  // no work - unexpected case (because this attrib is expected to be attached to the MergeObj which is expected to be one of the inputs)
  if(pColorObj1 == NULL && pColorObj2 == NULL)
    { 
      // no work - no input Object attributes
    }  // end neither InputObj has a color attribute branch

  // just one InputObj color attribute - make sure its copied onto the pMergedResult object
  else if(   pColorObj1 == NULL
          || pColorObj2 == NULL)
    {
      // local
      SmColorAttribute *pColorTgt = (pColorObj1 == NULL) ? pColorObj2 : pColorObj1 ;

      // make sure pColorMerge is set
      if(pColorTgt != pColorMerge)
        {
          // When Merge Result does not already have an attribute of this->m_lAttributeType
          SM_ASSERT_MSG(pColorMerge == NULL,_T("SmColorAttribute::Merge expects MErgedResult to be equal one of the two OrigObjects"))

          // Copy (with appropriate behavior) this Attribute onto pMergedResult 
          if (   m_eBehavior == SM_AB_COPY
              || m_eBehavior == SM_AB_STANDALONE_COPY) 
            {
              const SmContext * pContext = pMergedResult->GetContext();
              SmAttribute     * pNew     = MakeCopy(*pContext);
              pMergedResult->AddAttribute(pNew);
            }
          else if (   m_eBehavior == SM_AB_REFERENCE
                   || m_eBehavior == SM_AB_STANDALONE_REFERENCE) 
            {
              pMergedResult->AddAttribute(this);
            }
        } // end this attrib not already on pMergedResult check
    } // end just one InputObj has a color attribute branch

  else // both input objects have a color - set pColorMerge = Avg(pColorObj1, pColorObj2)
    {
      // check state - pColorMerge != NULL since we expect pMergedResult will equal pInputObj1 or pInputObj2
      SM_ASSERT_MSG(pColorMerge != NULL, _T("SmColorAttribute::Merge assumption is false - requires debugging")) ; 

      // set sColorMerge = Avg(pColorObj1, pColorObj2)
      SmVector3d sColorMerge((pColorObj1->GetValue() + pColorObj2->GetValue()) / 2.0);
      pColorMerge->SetValue(sColorMerge) ;

    } // end both InputObjs have a color attribute branch

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      smos_WriteBuffer(_T("\n This attribute: ")); this->Dump() ;
      if(pMergedResult) { smos_WriteBuffer(_T("\n MergedResult attributes: ")); pMergedResult->SmAObject::Dump() ; } 
    }
#endif // SM_DEBUG_CODE

  // all done

} // end SmColorAttribute::PropagateRegionsThroughBooleans

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmAttribute_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump contents of this attribute.

NOTES: Pretty Print starts on current line - no leading "\n"
***********************************************************************/
void SmAttribute::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T("SmAttribute[0x%p]: ID=[%ld], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              this, m_lAttributeID,
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_sprintf( sBuffForFile, _T("SmAttribute: [ID=%ld], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              m_lAttributeID,
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ?  _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ?  _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ?  _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ?  _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                              _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

} // end SmAttribute::Dump

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmLongAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmLongAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES: Pretty Print starts on current line - no leading "\n"
***********************************************************************/
void SmLongAttribute::Dump(void) const
{  
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T("SmLongAttribute[0x%p]: ID[%ld], Value[%ld], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              this, m_lAttributeID,
              m_lValue,
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_sprintf( sBuffForFile, _T("SmLongAttribute: ID[%ld], Value[%ld], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              m_lAttributeID,
              m_lValue,
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

} // end SmLongAttribute::Dump


/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmPointerAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmPointerAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES: Pretty Print starts on current line - no leading "\n"
***********************************************************************/
void SmPointerAttribute::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T("SmPointerAttribute[0x%p]: ID[%ld], Value[0x%p], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              this, m_lAttributeID,
              m_pPointer,
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_sprintf( sBuffForFile, _T("SmPointerAttribute: ID[%ld], Value[0x%p], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              m_lAttributeID,
              m_pPointer,
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

} // end SmPointerAttribute::Dump

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmMergeRegionAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmMergeRegionAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES: Pretty Print starts on current line - no leading "\n" 
***********************************************************************/
void SmMergeRegionAttribute::Dump(void) const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T("SmMergeRegionAttribute[0x%p]: ID[%ld], UserCount[%ld], 1stUser[0x%p], Mark[%ld], Behavior[%s]\n"), 
              this, m_lAttributeID,
              m_vUsers.GetSize(),
              m_vUsers.GetSize()>0 ? m_vUsers[0] : NULL,
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_sprintf( sBuffForFile, _T("SmMergeRegionAttribute: ID[%ld], UserCount[%ld], 1stUser[%s], Mark[%ld], Behavior[%s]\n"), 
              m_lAttributeID,
              m_vUsers.GetSize(),
              m_vUsers.GetSize()>0 ? _T("NotNULL") : _T("NULL"),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // This source regions
  smos_sprintf( sBuff, _T("\n      ThisSrcRegCnt :[%ld] "), m_vThisSourceRegions.GetSize()) ;
  smos_WriteBuffer(sBuff);

  for(ii=0;ii<m_vThisSourceRegions.GetSize();ii++)
    {
      if(ii>0) smos_WriteBuffer(_T("\n                         ")) ;
      smos_sprintf( sBuff, _T("Region:[0x%p %s] type:[%s]\n"), 
                  m_vThisSourceRegions[ii],
                  m_vThisSourceRegions[ii]->IsVoid() ? _T(" IsVoid") : _T("NotVoid"),
                    m_vThisSourceType[ii] == 0 ? _T("InternalSolid")
                  : m_vThisSourceType[ii] == 1 ? _T("InternalVoid")
                  : m_vThisSourceType[ii] == 2 ? _T("InfiniteRegion") : _T("ERROR")) ;
      smos_WriteBuffer(sBuff);
    }

  // Other source regions
  smos_sprintf( sBuff, _T("\n      OtherSrcRegCnt:[%ld] \n"), m_vOtherSourceRegions.GetSize()) ;
  smos_WriteBuffer(sBuff);

  for(ii=0;ii<m_vOtherSourceRegions.GetSize();ii++)
    {
      if(ii>0) smos_WriteBuffer(_T("\n                         ")) ;
      smos_sprintf( sBuff, _T("Region:[0x%p %s] type:[%s]\n"), 
                  m_vOtherSourceRegions[ii],
                  m_vOtherSourceRegions[ii]->IsVoid() ? _T(" IsVoid") : _T("NotVoid"),
                    m_vOtherSourceType[ii] == 0 ? _T("InternalSolid")
                  : m_vOtherSourceType[ii] == 1 ? _T("InternalVoid")
                  : m_vOtherSourceType[ii] == 2 ? _T("InfiniteRegion") : _T("ERROR")) ;
      smos_WriteBuffer(sBuff);
    }

} // end SmMergeRegionAttribute::Dump

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmPointerListAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmPointerListAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES: Pretty Print starts on current line - no leading "\n"
***********************************************************************/
void SmPointerListAttribute::Dump(void) const
{
  ULONG ii ;

  // Start one line dump 
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T("SmPointerListAttribute[0x%p]: ID[%ld], ValueCount[%ld"), 
              this, m_lAttributeID,
              m_vList.GetSize()) ;
  smos_sprintf( sBuffForFile, _T("SmPointerListAttribute: ID[%ld], ValueCount[%ld"), 
              m_lAttributeID,
              m_vList.GetSize()) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // add punctuation as needed
  if(m_vList.GetSize() > 0)
    { smos_WriteBuffer(_T(": ")) ; }
  
  // dump variable number of values - on same line
  for(ii=0;ii<m_vList.GetSize();ii++)
    {
      smos_sprintf(sBuff, _T("0x%p"), m_vList.GetAt(ii)) ;
      smos_WriteBuffer(sBuff);

    // add punctuation as needed
    if(ii < m_vList.GetSize()-1)
      { smos_WriteBuffer(_T(", ")) ; }

    } // end iter dumping all values

  // finish the dump - on same line
    smos_sprintf( sBuffForFile, _T("], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_WriteBuffer(sBuff);

} // end SmPointerListAttribute::Dump

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmVector3dAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmVector3dAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES: Pretty Print starts on current line - no leading "\n"
***********************************************************************/
void SmVector3dAttribute::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T("SmVector3dAttribute[0x%p]: ID[%ld], Value["), this, m_lAttributeID) ; 
  smos_sprintf( sBuffForFile, _T("SmVector3dAttribute: ID[%ld], Value["), m_lAttributeID) ; 
  smos_WriteBuffer(sBuff, sBuffForFile);

   m_vValue.Dump();

  smos_sprintf( sBuff, _T("], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_WriteBuffer(sBuff);

} // end SmVector3dAttribute::Dump

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmColorAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmColorAttribute_TYPE == t) ? TRUE : SmVector3dAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES: Pretty Print starts on current line - no leading "\n"
***********************************************************************/
void SmColorAttribute::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_sprintf( sBuff, _T("SmColorAttribute[0x%p]: ID[%ld], Value["), this, m_lAttributeID) ; 
  smos_sprintf( sBuffForFile, _T("SmColorAttribute: ID[%ld], Value["), m_lAttributeID) ; 
  smos_WriteBuffer(sBuff, sBuffForFile);

   m_vValue.Dump();

  smos_sprintf( sBuff, _T("], UserCount[%ld], Mark[%ld], Behavior[%s]\n"), 
              m_vUsers.GetSize(),
              m_lMark,
                m_eBehavior == SM_AB_COPY                 ? _T("SM_AB_COPY: 1-to-1, Owned")
              : m_eBehavior == SM_AB_REFERENCE            ? _T("SM_AB_REFERENCE: 1-to-Many, Owned")
              : m_eBehavior == SM_AB_STANDALONE_COPY      ? _T("SM_AB_STANDALONE_COPY: 1-to-1, NotOwned")
              : m_eBehavior == SM_AB_STANDALONE_REFERENCE ? _T("SM_AB_STANDALONE_REFERENCE: 1-to-Many, NotOwned")
              :                                             _T("SM_AB_TEMP: Not Copied, Not Persistent")) ;
  smos_WriteBuffer(sBuff);

} // end SmColorAttribute::Dump

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by SmGenericAttribute.

NOTES:
***********************************************************************/
ULONG SmGenericAttribute::GetMemoryUsed      // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated)             // out: bigger size of all allocated memory in bytes
  const
{
  // get referenced memory
  ULONG lThisAllocated, lAllocated, lUsed ;
  //(*mUsersMutex).lock();
  lUsed  = m_vUsers            .GetMemoryUsed(lThisAllocated) ;  lAllocated  = lThisAllocated ;
  //(*mUsersMutex).unlock();
  lUsed += m_vLongElements     .GetMemoryUsed(lThisAllocated) ;  lAllocated += lThisAllocated ;
  lUsed += m_vDoubleElements   .GetMemoryUsed(lThisAllocated) ;  lAllocated += lThisAllocated ;
  lUsed += m_vCharacterElements.GetMemoryUsed(lThisAllocated) ;  lAllocated += lThisAllocated ;

  // return total - avoid counting contained SmTARRAY memory twice
  rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<SmAObject*>) 
                                                - sizeof(SmTArray<long>)
                                                - sizeof(SmTArray<double>)
                                                - sizeof(SmTArray<char>) ;

  return(             sizeof(this) + lUsed      - sizeof(SmTArray<SmAObject*>) 
                                                - sizeof(SmTArray<long>)
                                                - sizeof(SmTArray<double>)
                                                - sizeof(SmTArray<char>)) ;
} // end SmGenericAttribute::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Constructor for the Generic Attribute

NOTES: 
***********************************************************************/
SmGenericAttribute::SmGenericAttribute
  (ULONG lAttributeID,
   SmAttributeBehaviorType eBehavior,
   const SmTArray<long> & rLongElements,
   const SmTArray<double> & rDoubleElements,
   const SmTArray<char> & rCharacterElements)
 : SmAttribute(lAttributeID, eBehavior)
{
  m_vLongElements.Append(rLongElements);
  m_vDoubleElements.Append(rDoubleElements);
  m_vCharacterElements.Append(rCharacterElements);

} // end SmGenericAttribute::SmGenericAttribute

/*******************************************************************//**
PURPOSE: Copy constructor for the Generic Attribute

NOTES: 
***********************************************************************/
SmGenericAttribute::SmGenericAttribute
  (const SmGenericAttribute & crOriginal)
 : SmAttribute(crOriginal.m_lAttributeID,crOriginal.m_eBehavior)
{
    m_vLongElements.Append(crOriginal.m_vLongElements);
    m_vDoubleElements.Append(crOriginal.m_vDoubleElements);
    m_vCharacterElements.Append(crOriginal.m_vCharacterElements);

} // end SmGenericAttribute::SmGenericAttribute
  

/*******************************************************************//**
PURPOSE: Remove the user of this attribute and destroy the attribute
    if there is no user of it.

NOTES: 
***********************************************************************/
void SmAttribute::RemoveUser
 (SmAObject * pUser,         // in : target user to remove from list
  SmBoolean   bDoNotDelete)  // in : TRUE  = don't delete
                             //      FALSE = delete this 
                             //              if  userCount goes to zero
                             //              and Behavior != STANDALONE
{
  ULONG lIndex;

  // when this user is found on the users list
  //(*mUsersMutex).lock();
  if (m_vUsers.FindElement(pUser,lIndex)) 
    {
      // remove it
      m_vUsers.RemoveAt(lIndex);

      // then - when user count drops to zero
      if (m_vUsers.GetSize() == 0) 
        {
          // Delete non-standalone attributes that aren't being saved
          if (   !bDoNotDelete
              && m_eBehavior != SM_AB_STANDALONE_COPY 
              && m_eBehavior != SM_AB_STANDALONE_REFERENCE) 
            {
              //(*mUsersMutex).unlock();
              delete this;
              return;
            }
        }
    }
  else // signal an error when attempting to remove a user not on the list
    {
      SM_DBG_WARN( _T("SmAttribute::RemoveUser: attempting to remove a user not on the list") );
    }
    //(*mUsersMutex).unlock();
} // end SmAttribute::RemoveUser
/*******************************************************************/ /**
 PURPOSE: FS attribute destructor.

 NOTES: Potentially unneccessary.
 ***********************************************************************/
/*
SmAttribute::~SmAttribute()
{
	ULONG ii;
	m_lAttributeID = SM_UNDEF_ULONG;
	m_lMark = SM_UNDEF_ULONG;
	for (ii = 0; ii < m_vUsers.GetSize(); ii++)
	{
		if (m_vUsers[ii]) m_vUsers[ii]->RemoveAttribute(this);
	}
}
*/
/*******************************************************************/ /**
 PURPOSE: FS attribute clearing for SmAObject::ClearAttributes(). 

 NOTES:
 ***********************************************************************/
void SmAttribute::fromClearAttributes(SmAObject* pThis) {
  if (m_vUsers.GetSize() != 1) {
    RemoveUser(pThis);
    return;
  }
  //(*mUsersMutex).lock();
  m_vUsers.ReSet();
  //(*mUsersMutex).unlock();
	if (m_eBehavior != SM_AB_STANDALONE_COPY && m_eBehavior != SM_AB_STANDALONE_REFERENCE){
		delete this;
	}
}


/*******************************************************************//**
PURPOSE: Add a user to this attribute.

NOTES: h
***********************************************************************/
void SmAttribute::AddUser
  (SmAObject *pUser)
{
  ULONG lIndex;
    //(*mUsersMutex).lock();
  if (m_vUsers.FindElement(pUser,lIndex)) {
      SE(SM_ERR);
  }
  m_vUsers.Add(pUser);
  //(*mUsersMutex).unlock();
} // end SmAttribute::AddUser

/*******************************************************************//**
PURPOSE: Get all of the users of this attribute

NOTES: 
***********************************************************************/
void SmAttribute::GetUsers
  (SmTArray<SmAObject*> & rUsers)  // out: Array of AObjects using this attribute
 const
{
  rUsers.ReSet();
  //(*mUsersMutex).lock();
  rUsers.Append(m_vUsers);
  //(*mUsersMutex).unlock();

} // end SmAttribute::GetUsers

/*******************************************************************//**
PURPOSE: Copy constructor for Tag Attribute.

NOTES: 
***********************************************************************/
SmTagAttribute::SmTagAttribute
  (const SmTagAttribute & crOriginal)
: SmAttribute(crOriginal.m_lAttributeID,crOriginal.m_eBehavior)
{
  m_lPrimaryID = crOriginal.m_lPrimaryID;
  m_lPrimaryOrdinal = crOriginal.m_lPrimaryOrdinal;
  m_lSecondaryIDType = crOriginal.m_lSecondaryIDType;
  m_lSecondaryID = crOriginal.m_lSecondaryID;
  m_lSecondaryOrdinal = crOriginal.m_lSecondaryOrdinal;
  m_lThirdIDType = crOriginal.m_lThirdIDType;
  m_lThirdID = crOriginal.m_lThirdID;
  m_lThirdOrdinal = crOriginal.m_lThirdOrdinal;

} // end SmTagAttribute::SmTagAttribute


/*******************************************************************//**
PURPOSE: Constructor for SmTagAttribute

NOTES: 
***********************************************************************/
SmTagAttribute::SmTagAttribute
  (ULONG lAttributeID,
   long lPrimaryID, long lPrimaryOrdinal, 
   long lSecondaryIDType, long lSecondaryID, long lSecondaryOrdinal, 
   long lThirdIDType, long lThirdID, long lThirdOrdinal,
   SmAttributeBehaviorType eBehavior)
 : SmAttribute(lAttributeID,eBehavior),
   m_lPrimaryID(lPrimaryID), m_lPrimaryOrdinal(lPrimaryOrdinal),
   m_lSecondaryIDType(lSecondaryIDType), m_lSecondaryID(lSecondaryID),
   m_lSecondaryOrdinal(lSecondaryOrdinal), m_lThirdIDType(lThirdIDType),
   m_lThirdID(lThirdID), m_lThirdOrdinal(lThirdOrdinal)
{

} // end SmTagAttribute::SmTagAttribute


/*******************************************************************//**
PURPOSE: Check to see if this tag attribute is within the range specified
   by rStart and rEnd.

NOTES: 
***********************************************************************/
SmStatus SmTagAttribute::RangeCheck
  (const SmTagAttribute & rStart, // in :
   const SmTagAttribute & rEnd,   // in :
   SmBoolean & rbIsInRange,       // out: True if Tag is
                                  //      within the range between rStart and rEnd.
   ULONG & rlSelectionStatus)     // out: What is status of selection
                                  //      right now it is 0 always.  It will be used in future
                                  //      to indicate quality of tag on ambiguous selections.
    
   const
{
    rbIsInRange = FALSE;
    rlSelectionStatus = 0;
    //First check primary ID
    if (m_lPrimaryID < rStart.m_lPrimaryID ||
        m_lPrimaryID > rEnd.m_lPrimaryID) {
        return SM_SUCCESS;
    }

    // Second check primary ordinal
    if (rStart.m_lPrimaryOrdinal != 0) { 
        if (m_lPrimaryOrdinal < rStart.m_lPrimaryOrdinal ||
            m_lPrimaryOrdinal > rEnd.m_lPrimaryOrdinal) {
            return SM_SUCCESS;
        }        
    }

    // Do we use a secondary tagging ?
    if (rStart.m_lSecondaryIDType != SM_TI_UNDEFINED) {
        // Now check secondary ID
        if (rStart.m_lSecondaryID != 0) {
            if (m_lSecondaryID < rStart.m_lSecondaryID ||
                m_lSecondaryID > rEnd.m_lSecondaryID) {
                return SM_SUCCESS;
            }
        }
        
        // Now Check secondary Ordinal
        if (rStart.m_lSecondaryOrdinal != 0) { 
            if (m_lSecondaryOrdinal < rStart.m_lSecondaryOrdinal ||
                m_lSecondaryOrdinal > rEnd.m_lSecondaryOrdinal) {
                return SM_SUCCESS;
            }        
        }
    }

    // Now check Third Tagging
    if (rStart.m_lThirdIDType != SM_TI_UNDEFINED) {
        // Now check ID
        if (rStart.m_lThirdID != 0) {
            if (m_lThirdID < rStart.m_lThirdID ||
                m_lThirdID > rEnd.m_lThirdID) {
                return SM_SUCCESS;
            }
        }
        
        // Now Check Third Ordinal
        if (rStart.m_lThirdOrdinal != 0) { 
            if (m_lThirdOrdinal < rStart.m_lThirdOrdinal ||
                m_lThirdOrdinal > rEnd.m_lThirdOrdinal) {
                return SM_SUCCESS;
            }        
        }
    }

    // If we made it this far we have a tag in the range
    rbIsInRange = TRUE;
    return SM_SUCCESS;

} // end SmTagAttribute::RangeCheck

/*******************************************************************//**
PURPOSE: Check to see if this tag attribute matches the tag.

NOTES: 
***********************************************************************/
SmStatus SmTagAttribute::MatchCheck
  (const SmTagAttribute & rTag, // in :
   SmBoolean & rbIsMatch,       // out: True if Tag is matching
   ULONG & rlSelectionStatus    // out: What is status of selection
                                //      right now it is 0 always.  It will be used in future
                                //      to indicate quality of tag on ambiguous selections.
   ) const
{
    rbIsMatch = FALSE;
    rlSelectionStatus = 0;
    //First check primary ID
    if (m_lPrimaryID != rTag.m_lPrimaryID) {
        return SM_SUCCESS;
    }

    // Second check primary ordinal
    if (rTag.m_lPrimaryOrdinal != 0) { 
        if (m_lPrimaryOrdinal != rTag.m_lPrimaryOrdinal) {
            return SM_SUCCESS;
        }        
    }

    // Do we use a secondary tagging ?
    if (rTag.m_lSecondaryIDType != SM_TI_UNDEFINED) {
        // Now check secondary ID
        if (rTag.m_lSecondaryID != 0) {
            if (m_lSecondaryID != rTag.m_lSecondaryID) {
                return SM_SUCCESS;
            }
        }
        
        // Now Check secondary Ordinal
        if (rTag.m_lSecondaryOrdinal != 0) { 
            if (m_lSecondaryOrdinal != rTag.m_lSecondaryOrdinal) {
                return SM_SUCCESS;
            }        
        }
    }

    // Now check Third Tagging
    if (rTag.m_lThirdIDType != SM_TI_UNDEFINED) {
        // Now check ID
        if (rTag.m_lThirdID != 0) {
            if (m_lThirdID != rTag.m_lThirdID) {
                return SM_SUCCESS;
            }
        }
        
        // Now Check Thrid Ordinal
        if (rTag.m_lThirdOrdinal != 0) { 
            if (m_lThirdOrdinal != rTag.m_lThirdOrdinal) {
                return SM_SUCCESS;
            }        
        }
    }

    // If we made it this far we have a match
    rbIsMatch = TRUE;
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmGenericAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmGenericAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmGenericAttribute::Dump(void) const
{
  ULONG ii ;
  TCHAR sBuffer[SM_TBLOCK_SIZE]; 

  // attrib ID
  ULONG lAttrID = GetAttributeID();
  smos_sprintf( sBuffer,_T("SmAttribute (generic) ID = %ld \n"), lAttrID);
  smos_WriteBuffer( sBuffer );

  // internal Long and double array dumps
  SM_DUMP_TARRAY(m_vLongElements) ;      // was m_vLongElements.Dump();
  SM_DUMP_TARRAY(m_vDoubleElements) ;    // was m_vDoubleElements.Dump();
  
  // obsolete: m_vCharacterElements.Dump();

  // // gwc removed two lines - this is a memory leak and does not handle pString==NULL well.
  // size_t length = strlen(pString) + 1 ;
  // wchar_t * wString = new wchar_t[length];
  if(m_vCharacterElements.GetSize() > 0)
    {
      // add string terminating character after last string character
      ((SmTArray<char> &)m_vCharacterElements).Add('\0') ;

#ifdef _UNICODE
     // Create a wide character string and initialize
     wchar_t wString[SM_TBLOCK_SIZE];

      for(ii=0;ii<m_vCharacterElements.GetSize();ii++) 
        {
          wString[ii] = m_vCharacterElements[ii];
        }

      smos_WStrCpy( sBuffer, SM_TBLOCK_SIZE, wString);
#else // no _UNICODE
      // Apparently smos_WStrCpy(sBuffer, pString) does not work on Linux
      for(ii=0;ii<m_vCharacterElements.GetSize() && ii < SM_TBLOCK_SIZE;ii++) 
        {
          sBuffer[ii] = m_vCharacterElements[ii];
        }
#endif // no _UNICODE

      smos_WriteBuffer( sBuffer );

      // remove added last string terminating character
      ((SmTArray<char> &)m_vCharacterElements).SetSize(m_vCharacterElements.GetSize()-1) ;

    } // end m_vCharacterElements.GetSize() != 0 check
    
  smos_WriteBuffer(_T("\n\n"));

} // end SmGenericAttribute::Dump

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTagAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmTagAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmTagAttribute::Dump(void) const
{
    smos_WriteBuffer(_T("SmTagAttribute - "));
    smos_WriteBuffer(_T("\n"));

} // end SmTagAttribute::Dump

