// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmAObject.cpp
* PURPOSE: Source file for SmAObject.
**********************************************************************/

#include "StdAfx.h"
#include <SmAObject.h>
#include <SmAttribute.h>

#include <SmRegion.h> // for SmAObject::Notify(SM_NO_REG_PROPAGATION) - GWC: this should be managed in some other way to avoid this inclusion

/*******************************************************************//**
PURPOSE: Removes all attributes from the m_pAttributes list,
         deletes m_pAttributes,
         and sets m_pAttributes to NULL.

NOTES: 
***********************************************************************/
void SmAObject::ClearAttributes()
{
  // no work - no attributes
  if (m_pAttributes == NULL) return;

  // when there is just one attribute, m_pAttributes is a pointer to an attribute
  if (m_pAttributes->IsKindOf(SmAttribute_TYPE)) 
    {
      // remove the attribute
      SmAttribute *pAttr = (SmAttribute*)m_pAttributes;
      pAttr->fromClearAttributes(this);

      // clear the m_pAttributes list ptr
      m_pAttributes = NULL;
    }
  else // when there is a list of attributes, m_pAttributes is a ptr to an array of attributes
    {
      SmTArray<SmAttribute*> * pAttrs = (SmTArray<SmAttribute*> *)m_pAttributes;
      
      // remove every attribute from this attribute
      for (ULONG i=0; i<pAttrs->GetSize(); i++) 
        {
          SmAttribute *pAttr = (*pAttrs)[i];
          pAttr->fromClearAttributes(this);
        }

      // delete the array
      SM_ASSERT(pAttrs != NULL); delete pAttrs;
      
      // clear the m_pAttributes list ptr
      m_pAttributes = NULL;
    }

} // end SmAObject::~SmAObject destructor

/*******************************************************************//**
PURPOSE: Copy constructor - copies attributes to new object

NOTES: 
***********************************************************************/
SmAObject::SmAObject
  (const SmAObject & crObject)  // in : object being copied
 : SmObject(crObject)
{
  // get all crObject attributes
  m_pAttributes = NULL;
  SmAttribute * sData[64];
  SmTArray<SmAttribute*> sAttributes(64,sData);
  crObject.GetAttributes(sAttributes);

  // copy every crObject attribute onto this object
  for (ULONG i=0; i<sAttributes.GetSize(); i++) 
    {
      SmAttribute *pAttr = sAttributes[i];
      pAttr->Copy(&crObject,this);
    }

} // end SmAObject::SmAObject

/*******************************************************************//**
PURPOSE: Notify attributed objects of changes occuring down at the
   lower level.  

NOTES: The notify then takes the appropriate actions on the
   attributes.
***********************************************************************/
void SmAObject::Notify                 // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
 (SmNotifyOperation  eNotifyOperation, //       event                | caller      |  pData1  | pData2                | pData3                   
  SmObject         * pData1,           //----------------------------+-------------+----------+-----------------------+--------------------------
  SmObject         * pData2,           // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL             
  SmObject         * pData3)           // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                     
                                       // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      
                                       // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                       // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
                                       // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   
                                       // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           
                                       // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          
                                       // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL                      
                                       // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   
                                       // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                       // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                       // SM_NO_SPLIT                | SplitObj    | Child1   | Child2                | SplitObj's Owner or NULL   
                                       // SM_NO_MERGE                | MergeObj    | OrigObj1 | OrigObj2              | MergeObj's Owner or NULL   
                                       // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                      
{
  // locals
  ULONG ii, jj, kk ;
  SmAObject *pSrcObject1 = NULL ;
  SmAObject *pSrcObject2 = NULL ;
  SmAttribute * sData1[64], * sData2[64] ;
  SmTArray<SmAttribute*> sAttributes1(64,sData1), sAttributes2(64,sData2), *pAttributes ;

  // get all attributes from target source object
  switch(eNotifyOperation)
    {
      case SM_NO_SPLIT_IN_BREP   : pSrcObject1 = (SmAObject *)pData1 ; break ;  // pData1 = SplitObj
      case SM_NO_MERGE_IN_BREP   : pSrcObject1 = (SmAObject *)pData1 ;          // pData1 = SurvivingObj
                                   pSrcObject2 = (SmAObject *)pData2 ; break ;  // pData2 = DeleteObj
      case SM_NO_COINCIDENT      : pSrcObject1 = (SmAObject *)pData2 ; break ;  // pData1 = BrepAObj
                                                                                // pData2 = BrepBObj
      case SM_NO_CONSTRUCTION    : pSrcObject1 = (SmAObject *)pData2 ; break ;  // pData2 = CopyFromObj or NULL
      case SM_NO_COPY            : pSrcObject1 = (SmAObject *)this ;   break ;  // this = FromObj 
      case SM_NO_PRE_EDIT        : pSrcObject1 = (SmAObject *)this ;   break ;  // this = EditObj
      case SM_NO_SPLIT           : pSrcObject1 = (SmAObject *)this ;   break ;  // this = SplitObj
      case SM_NO_MERGE           : pSrcObject1 = (SmAObject *)pData1 ;          // pData1 = OrigObj1 
                                   pSrcObject2 = (SmAObject *)pData2 ; break ;  // pData2 = OrigObj2
      case SM_NO_DESTRUCTION     : pSrcObject1 = (SmAObject *)this ;   break ;  // this = DeleteObj
      case SM_NO_REG_PROPAGATION : pSrcObject1 = (SmAObject *)this ;   break ;
      case SM_NO_CHANGE_GEOMETRY : pSrcObject1 = (pData1) ? (SmAObject *)pData3 : NULL ; break ;  // pData3 = OldGeom, used when NewGeom exists
      default                    : pSrcObject1 = NULL ;                break ;
    }

  // for both possible source objects
  for(ii=0;ii<2;ii++)
    {
      // get all src attributes
      if     (ii == 0 && pSrcObject1) { pSrcObject1->GetAttributes(sAttributes1) ; pAttributes = &sAttributes1 ; }
      else if(ii == 1 && pSrcObject2) { pSrcObject2->GetAttributes(sAttributes2) ; pAttributes = &sAttributes2 ; }
      else                            { sAttributes1.ReSet() ; pAttributes = &sAttributes1 ; }

      // handle all Non-SM_NO_REG_PROPAGATION cases with an iteration through all source objects
      if(eNotifyOperation != SM_NO_REG_PROPAGATION)
        {
          // call the operation action function on every attribute
          for (jj=0; jj<pAttributes->GetSize(); jj++) 
            {
              SmAttribute *pAttr = pAttributes->GetAt(jj) ;
              if (!pAttr) continue;

              switch (eNotifyOperation) 
                {
                  case SM_NO_SPLIT_IN_BREP   : // this = brep, pData1 = SplitObj, pData2 = Child1, pData3 = Child2
                                               pAttr->Split((SmAObject*)pData1,   // in : Object being split (the object who owns this attribute)
                                                            (SmAObject*)pData2,   // in : child1, gets copy of this attribute when needed        
                                                            (SmAObject*)pData3) ; // in : child2, gets copy of this attribute when needed        
                                               break ;

                  case SM_NO_MERGE_IN_BREP   : { // this = brep, pData1 = Surviving Obj, pData2 = Delete Obj, pData3 = Brep
                                                 // don't call Merge twice for the same set of attributes
                                                 SmBoolean bDoCall = TRUE ;
                                                 if(ii == 1) // check to see if this is a redundant call
                                                   {
                                                     // see if this call has already been made
                                                     for(kk=0;kk<sAttributes1.GetSize();kk++)
                                                       {
                                                         if(sAttributes1[kk]->GetAttributeID() == pAttr->GetAttributeID())
                                                           {
                                                             bDoCall = FALSE ;
                                                             break ;
                                                           }
                                                       }
                                                   } // end check for duplicate call check

                                                 if(bDoCall)
                                                   {
                                                     pAttr->Merge((SmAObject*)pData1,   // in : not used by default behavior - object1 being merged    
                                                                  (SmAObject*)pData2,   // in : not used by default behavior - object2 being merged    
                                                                  (SmAObject*)pData1) ; // in : merged object - gets copy of this attribute when needed
                                                   }
                                               } break ;

                  case SM_NO_COINCIDENT      : // this = pBrepA(ToBrep), pData1 = pBrepAObj(ToOBj), pData2 = pBrepBObj(FromObj), pData3 = pBrepB(FromBrep)
                                               pAttr->Coincident((SmAObject*)pData2,  // in : FromObj of the FromObj-ToObj Coincident Topology pair  
                                                                 (SmAObject*)pData1,  // in : ToObj   of the FromObj-ToObj Coincident Topology pair 
                                                                 (SmAObject*)pData3,  // in : Brep containing pFromObj
                                                                 (SmAObject*)this) ;  // in : Brep containing pToObj
                                               break ;

                  case SM_NO_CHANGE_GEOMETRY : // this = TopoObj, pData1 = NewGeom, pData2 = pBrep, pData3 = OldGeom
                                               pAttr->Copy((SmAObject*) pData3,
                                                           (SmAObject*) pData1 );
                                               break ;

                  case SM_NO_CONSTRUCTION    : // this = CopyTo, pData1 = CopyTo, pData2 = CopyFromObj or NULL
                                               pAttr->Copy((SmAObject*)pData2,  // in : not used by default behavior - object being copied from
                                                           this) ;              // in : object being copied into                               
                                               break ; 

                  case SM_NO_COPY            : // this = CopyFrom, pData1 = CopyTo, pData2 = CopyTo Owner, pData3 = CopyFrom Owner
                                               pAttr->Copy(this,                  // in : not used by default behavior - object being copied from
                                                           (SmAObject*)pData1) ;  // in : object being copied into                               
                                               break ;

                  case SM_NO_PRE_EDIT        : // this = EditObj
                                               pAttr->Edit(this);
                                               break ;

                  case SM_NO_SPLIT           : // this = SplitObj, pData1 = Child1, pData2 = Child2, pData3 = SplitObj Owner
                                               pAttr->Split(this,                 // in : Object being split (the object who owns this attribute)
                                                            (SmAObject*)pData1,   // in : child1, gets copy of this attribute when needed        
                                                            (SmAObject*)pData2) ; // in : child2, gets copy of this attribute when needed        
                                               break ;

                  case SM_NO_MERGE           : { // this = MergeObj, pData1 = OrigObj1, pData2 = OrigObj2, pData3 = MergeObj Owner
                                                 // don't call Merge twice for the same set of attributes
                                                 SmBoolean bDoCall = TRUE ;
                                                 if(ii == 1)
                                                   {
                                                     // see if this call has already been made
                                                     for(kk=0;kk<sAttributes1.GetSize();kk++)
                                                       {
                                                         if(sAttributes1[kk]->GetAttributeID() == pAttr->GetAttributeID())
                                                           {
                                                             bDoCall = FALSE ;
                                                             break ;
                                                           }
                                                       }
                                                   } // end check for duplicate call check
                                                 if(bDoCall)
                                                   {
                                                     pAttr->Merge((SmAObject*)pData1,   // in : not used by default behavior - object1 being merged    
                                                                  (SmAObject*)pData2,   // in : not used by default behavior - object2 being merged    
                                                                  this) ;               // in : merged object - gets copy of this attribute when needed
                                                   }
                                               } break ;

                  case SM_NO_DESTRUCTION     : // this = DeleteObj, pData1 = DeleteObj
                                               pAttr->Destruction(this);
                                               break ;

                  case SM_NO_ADD_TO_BREP     : break ;
                  case SM_NO_RM_FROM_BREP    : break ;
                  case SM_NO_CHANGE_OWNER    : break ;
                  case SM_NO_POST_EDIT       : break ;
                  case SM_NO_UNKNOWN         :          
                  default:                    { SE_MSG(SM_ERR, _T("SmAObject::Notify - SM_NO_UNKNOWN event signalled")) ; } 
                                              break ;
                } // end switch(eNotifyOperation) - allowing each attribute to propagate
            } // end iter jj, every attribute
        } // end eNOtifyOperation != SM_NO_REG_PROPAGATION branch

      else // eNOtifyOperation == SM_NO_REG_PROPAGATION branch
        {
          // locals 
          SmRegion *pRegion = SM_CAST_NONNULL_PTR(SmRegion, this) ;

          // error case - this obj is not a Region, skip problem cases
          SM_ASSERT_MSG(pRegion != NULL, _T("SmAObject Error - case SM_NO_REG_PROPAGATION called when this Obj type != SmRegion")) ; 
          if(pRegion == NULL) 
            { continue ; }

          // locals
          SmMergeRegionAttribute * pMergeRegionAttrib         = (SmMergeRegionAttribute*)pRegion->FindAttribute(SM_AI_OTHER_REGION_ID) ;
          SmBooleanOperationType   eOperation                 = pMergeRegionAttrib->GetOperation() ;
          SmTArray<SmRegion *>   & rOtherRegions              = pMergeRegionAttrib->GetOtherRegions() ;
          // ULONG                    lOtherSrcRegCnt            = pMergeRegionAttrib->GetOtherRegionsSize() ; // for debug - expected to be 1

          // See if any Other or This Attribs are propagated through booleans
          SmBoolean bHasOtherPropagatedAttribs = pMergeRegionAttrib->HasAnyOtherPropagatedAttribs() ;
          SmBoolean bHasThisPropagatedAttribs  = FALSE ;
          for(jj=0;(bHasThisPropagatedAttribs == FALSE) && (jj<sAttributes1.GetSize());jj++)
            {
              // remember when attribute is Propagated through booleans
              if(sAttributes1[jj]->IsPropagatedThroughBooleans())
                { bHasThisPropagatedAttribs = TRUE ; }

            } // end iter every Reg Attribute

          // no work - no propagated attributes
          if(   bHasOtherPropagatedAttribs == FALSE
             && bHasThisPropagatedAttribs  == FALSE)
            { continue ; }

          // locals 
          //   I [gwc] don't have a good propagation model to implement here. So I'll gather all the information
          //   currently available to be referenced during debug to see how the model might be improved if
          //   it should need it.
          // current behavior - for every this/OtherSrc region pair - propagate attributes
          //    problem: when multiple OtherSrc regions have attributes that need to be propagated
          //             the behavior of the last OtherSrc region wins and worse yet, when the
          //             propagation behavior is merging - the order of the OtherSrc regions 
          //             affects the final merged behavior.
          // Clearly there ought to be a better propagation model - but for now, this is one which
          //   can be improved over time.
          // SmBoolean  bHasThisSolids  = pMergeRegionAttrib->HasAnySolid(1) ; // in : 1 = check only ThisBrep SrcRegions
          // SmBoolean  bHasThisVoids   = pMergeRegionAttrib->HasAnyVoid(1) ;  // in : 1 = check only ThisBrep SrcRegions
          // SmBoolean  bHasOtherSolids = pMergeRegionAttrib->HasAnySolid(2) ; // in : 2 = check only OtherBrep SrcRegions
          // SmBoolean  bHasOtherVoids  = pMergeRegionAttrib->HasAnyVoid(2) ;  // in : 2 = check only OtherBrep SrcRegions

          // pTgtRegion = ResultRegion
          SmRegion    * pTgtRegion = pRegion ;
          SmAttribute * pTgtAttrib, * pOtherAttrib ;

          // for every OtherSrcRegion
          for(jj=0;jj<rOtherRegions.GetSize();jj++)
            {
              SmRegion * pOtherRegion =  rOtherRegions[jj] ;
              SmBoolean  bOtherSolid  = !pOtherRegion->IsVoid() ;
              ULONG      lAttrib1Size = sAttributes1.GetSize() ;
              ULONG      lAttrib2Size = sAttributes2.GetSize() ;
              pOtherRegion->GetAttributes(sAttributes2) ;

              // for every TgtRegion and OtherSrcRegion Attribute
              for(kk=0;kk<lAttrib1Size+lAttrib2Size;kk++)
                {
                  // when processing TgtRegion attributes
                  if(kk < sAttributes1.GetSize())
                    {
                      pTgtAttrib   = sAttributes1[kk] ;
                      pOtherAttrib = pOtherRegion->FindAttribute(pTgtAttrib->GetAttributeID()) ;

                      // no work - not a propagated attribute
                      if(pTgtAttrib->IsPropagatedThroughBooleans() == FALSE)
                        { continue ; }

                    } // end TgtRegion attributes branch
                  else // OtherSrcRegion attributes
                    {
                      pTgtAttrib   = NULL ;
                      pOtherAttrib = sAttributes2[kk-lAttrib1Size] ;

                      // skip cases where TgtRegion has this attribute
                      if(pTgtRegion->FindAttribute(pOtherAttrib->GetAttributeID()) != NULL)
                        { continue ; }

                      // no work - not a propagated attribute
                      if(pOtherAttrib->IsPropagatedThroughBooleans() == FALSE)
                        { continue ; }

                    } // end OtherSrcRegion attributes branch

                  SmBoolean bTgtAttrib   = pTgtAttrib != NULL ;
                  SmBoolean bOtherAttrib = pOtherAttrib != NULL ;

                  // next - pick Oneof following merge actions 
                  //        based on { eOperation, OtherRegion->IsSolid, ThisAttrib presence, OtherAttrib presence}
                  enum SmAction 
                    { SM_ACT_NONE,            // take no action - the attributes are already distributed as desired
                      SM_ACT_KEEP_THIS,       // the attribute on the This object is kept as is - coincidentally like SM_ACT_NONE - needs no action
                      SM_ACT_SET_FROM_OTHER,  // Place a copy of the Other->Attribute on the This object, replace existing This->Attribute if present 
                      SM_ACT_MERGE            // call PropagateRegionsThroughBooleans() to merge this and other attribute values
                    } ;
                  SmAction eAction = SM_ACT_NONE ; 
                  
                  /* current model of attribute propagation - for every ResRegion/OtherSrcRegion combination
                                                            - for every attribute that is propagated
                                                            - SM_BO_MERGE breaks the solid model concept, so don't branch on IsVoid [B540]
                     +--------------+-----------------------+-----------------------+-----------------------+
                     + eOperation   | This&Other Attribs    | ThisAttrib Only       | OtherAttrib Only      |
                     +--------------+-----------------------+-----------------------+-----------------------+
                     + Union        |                       |                       |                       |
                     +  OtherSolid  | SM_ACT_MERGE          | SM_ACT_KEEP_THIS      | SM_ACT_SET_FROM_OTHER |
                     +  OtherVoid   | SM_ACT_NONE           | SM_ACT_KEEP_THIS      | SM_ACT_NONE           |
                     +--------------+-----------------------+-----------------------+-----------------------+
                     + Difference   |                       |                       |                       |
                     +  OtherSolid  | SM_ACT_NONE           | SM_ACT_NONE           | SM_ACT_NONE           |
                     +  OtherVoid   | SM_ACT_NONE           | SM_ACT_NONE           | SM_ACT_NONE           |
                     +--------------+-----------------------+-----------------------+-----------------------+
                     + Exclusive-or |                       |                       |                       |
                     +  OtherSolid  | SM_ACT_SET_FROM_OTHER | SM_ACT_KEEP_THIS      | SM_ACT_SET_FROM_OTHER |
                     +  OtherVoid   | SM_ACT_NONE           | SM_ACT_NONE           | SM_ACT_NONE           |
                     +--------------+-----------------------+-----------------------+-----------------------+
                     + Intersect    |                       |                       |                       |
                     +  OtherSolid  | SM_ACT_MERGE          | SM_ACT_KEEP_THIS      | SM_ACT_SET_FROM_OTHER |
                     +  OtherVoid   | SM_ACT_NONE           | SM_ACT_NONE           | SM_ACT_NONE           |
                     +--------------+-----------------------+-----------------------+-----------------------+
                     + Merge        |                       |                       |                       |
                     +  OtherSolid  | SM_ACT_MERGE          | SM_ACT_KEEP_THIS      | SM_ACT_SET_FROM_OTHER |
                     +  OtherVoid   | SM_ACT_MERGE          | SM_ACT_KEEP_THIS      | SM_ACT_SET_FROM_OTHER |
                     +--------------+-----------------------+-----------------------+-----------------------+
                  */                      

                  // switch on operation type
                  switch( eOperation )
                    { case SM_BO_UNION         : eAction =   bOtherSolid ? (  (bTgtAttrib && bOtherAttrib) ? SM_ACT_MERGE
                                                                            : (bTgtAttrib) ? SM_ACT_KEEP_THIS
                                                                            :  SM_ACT_SET_FROM_OTHER )
                                                           :               (  (bTgtAttrib && bOtherAttrib) ? SM_ACT_NONE
                                                                            : (bTgtAttrib) ? SM_ACT_KEEP_THIS
                                                                            : SM_ACT_NONE ) ;
                                                 break ;
                      case SM_BO_MERGE         :
                      case SM_BO_PARTIAL_MERGE: eAction =    bTgtAttrib  ? ( bOtherAttrib ? SM_ACT_MERGE
                                                                            : SM_ACT_KEEP_THIS )
                                                           :               ( bOtherAttrib ? SM_ACT_SET_FROM_OTHER
                                                                            : SM_ACT_NONE );
                                                 break;
                      case SM_BO_DIFFERENCE    : 
                      case SM_BO_SLICE         : eAction = SM_ACT_NONE ;
                                                 break ;
                      case SM_BO_INTERSECTION  : eAction =   bOtherSolid ? (  (bTgtAttrib && bOtherAttrib) ? SM_ACT_MERGE
                                                                            : (bTgtAttrib) ? SM_ACT_KEEP_THIS
                                                                            :  SM_ACT_SET_FROM_OTHER )
                                                           :               (  (bTgtAttrib && bOtherAttrib) ? SM_ACT_NONE
                                                                            : (bTgtAttrib) ? SM_ACT_NONE
                                                                            :  SM_ACT_NONE ) ;
                                                 break ;
                      case SM_BO_EXCLUSIVE_OR  : eAction =   bOtherSolid ? (  (bTgtAttrib && bOtherAttrib) ? SM_ACT_SET_FROM_OTHER
                                                                            : (bTgtAttrib) ? SM_ACT_KEEP_THIS
                                                                            :  SM_ACT_SET_FROM_OTHER )
                                                           :               (  (bTgtAttrib && bOtherAttrib) ? SM_ACT_NONE
                                                                            : (bTgtAttrib) ? SM_ACT_NONE
                                                                            : SM_ACT_NONE ) ;
                                                 break ;
                      case SM_BO_EXTRACT_SEPARATE :
                      case SM_BO_UNKNOWN       :
                      default                  : eAction = SM_ACT_NONE ;
                                                 break ;
                    } // end switch on eOperation

                  // set attribute values
                  switch(eAction)
                    {
                      case SM_ACT_SET_FROM_OTHER : // Place a copy of the Other->Attribute on the This object, replace existing This->Attribute if present
                                                   pRegion->RemoveAttribute(pTgtAttrib) ;
                                                   pRegion->AddAttribute( pOtherAttrib );
                                                   break ;
                      case SM_ACT_MERGE          : // call PropagateRegionsThroughBooleans() to merge this and other attribute values
                                                   pTgtAttrib->PropagateRegionsThroughBooleans(pRegion, pOtherRegion, pRegion) ;
                                                   break ;
                      case SM_ACT_KEEP_THIS      : // the attribute on the This object is kept as is - coincidentally like SM_ACT_NONE - needs no action
                      case SM_ACT_NONE           : // take no action - the attributes are already distributed as desired
                      break ;
                    } // end switch on eAction - setting attribute values

                } // end iter every TgtRegion and OtherRegion Attribute
            } // end iter every TgtRegion/OtherSrcRegion pair
        } // end eNOtifyOperation == SM_NO_REG_PROPAGATION branch


    } // end iter ii, both possible SrcObjects

  // pass the call up to the base class
  SmObject::Notify(eNotifyOperation,pData1,pData2,pData3);

} // end SmAObject::Notify

//      /*******************************************************************//**
//      PURPOSE: Notify attributed objects of changes occuring down at the
//         lower level.  
//      
//      NOTES: The notify then takes the appropriate actions on the
//         attributes.
//      ***********************************************************************/
//      void SmAObject::Notify                 // expected calls: caller->Notify(Event, pData1, pData2, pData2)                                   
//       (SmNotifyOperation  eNotifyOperation, //       event                | caller      |  pData1  | pData2                | pData3                   
//        SmObject         * pData1,           //----------------------------+-------------+----------+-----------------------+--------------------------
//        SmObject         * pData2,           // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL             
//        SmObject         * pData3)           // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                     
//                                             // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      
//                                             // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                               // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
//                                             // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   
//                                             // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           
//                                             // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          
//                                             // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL                      
//                                             // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   
//                                             // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
//                                             // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
//                                             // SM_NO_SPLIT                | SplitObj    | Child1   | Child2                | SplitObj's Owner or NULL   
//                                             // SM_NO_MERGE                | MergeObj    | OrigObj1 | OrigObj2              | MergeObj's Owner or NULL   
//                                             // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
//                                             // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     // 
//      {
//        // when there are no attributes - pass the call to the Object
//        if (m_pAttributes == NULL) 
//          { 
//            SmObject::Notify(eNotifyOperation,pData1,pData2,pData3);
//            return;
//          }
//      
//        // get all attributes
//        SmAttribute * sData[64];
//        SmTArray<SmAttribute*> sAttributes(64,sData);
//        GetAttributes(sAttributes);
//      
//        // call the operation action function on every attribute
//        for (ULONG i=0; i<sAttributes.GetSize(); i++) 
//          {
//            SmAttribute *pAttr = sAttributes[i];
//            if (!pAttr) continue;
//      
//            switch (eNotifyOperation) 
//              {
//                case SM_NO_COPY:
//                    pAttr->Copy(this,(SmAObject*)pData1);
//                    break;
//                case SM_NO_SPLIT_IN_BREP:
//                case SM_NO_SPLIT:
//                    pAttr->Split(this,(SmAObject*)pData1,(SmAObject*)pData2);
//                    break;
//                case SM_NO_MERGE_IN_BREP:
//                case SM_NO_MERGE:
//                    pAttr->Merge(this,(SmAObject*)pData1,(SmAObject*)pData2);
//                    break;
//                case SM_NO_RM_FROM_BREP:
//                case SM_NO_DESTRUCTION:
//                    pAttr->Destruction(this);
//                    break;
//                case SM_NO_PRE_EDIT:
//                    pAttr->Edit(this);
//                    break;
//                case SM_NO_POST_EDIT:
//                case SM_NO_ADD_TO_BREP:
//                case SM_NO_TRIM_NO_SPLIT_IN_BREP:
//                case SM_NO_CONSTRUCTION: 
//                case SM_NO_UNKNOWN: 
//                   break;
//              } // end switch(eNotifyOperation)
//          } // end iter every attribute
//      
//          SmObject::Notify(eNotifyOperation,pData1,pData2,pData3);
//      
//      } // end SmAObject::Notify

/*******************************************************************//**
PURPOSE: Add an attribute to this object.   

NOTES: Pointer is placed in Object attribute list without being copied.
       this pointer is added to Attribute's user list. 
       Do not place one Attribute behavior type:[SM_AB_COPY] pointer
          onto multiple objects.
       Do not add attribute if a duplicate type (same AttributeID value) is found.
          No error is signalled (the routine always returns silently)
          and creates an opportunity for a memory leak.
             
***********************************************************************/
void SmAObject::AddAttribute
  (SmAttribute * pAttribute)  // in : attribute to add
{
  // look for an attribute of this type
  if (FindAttribute(pAttribute->m_lAttributeID) == pAttribute) 
    {
      return; // Don't add same attribute type more than one time
    }

  // add attribute 
  if (m_pAttributes == NULL) 
    {
      // lone attribute is pointed at by m_pAttributes
      m_pAttributes = pAttribute;
    }
  else 
    {
      // 2nd attribute - 
      // build attribute list, add this and last attribute, store list
      if (m_pAttributes->IsKindOf(SmAttribute_TYPE)) 
        {
          const SmContext *pContext = GetContext();
          SmTArray<SmAttribute*> * pAttrs = new (*pContext) SmTArray<SmAttribute*>(*pContext);
          pAttrs->SetDataSize(4);
          pAttrs->Add((SmAttribute*)m_pAttributes);
          pAttrs->Add(pAttribute);
          m_pAttributes = pAttrs;
        }
      else 
        {
          // subsequent attributes are added to the end of the attribute list
          SmTArray<SmAttribute*> * pAttrs = (SmTArray<SmAttribute*> *)m_pAttributes;
          pAttrs->Add(pAttribute);

        } // end branch on m_pAttribute pointer type
    } // end branch on m_pAttribute value

  // tell the attribute its attached to this object
  pAttribute->AddUser(this);

} // end SmAObject::AddAttribute

/*******************************************************************//**
PURPOSE: Add an attribute to this object.  

NOTES: Add attribute without checking for duplicate
                attribute types.
***********************************************************************/
void SmAObject::AddSlewAttribute(SmAttribute * pAttribute)
{
  // add the attribute to the object attribute list
  // branch on the value of m_pAttributes
  if (m_pAttributes == NULL) 
    {
      // the first attribute is pointed to by m_pAttributes.
      m_pAttributes = pAttribute;
    }
  else 
    {
      // when there is just one attribute on the list
      if (m_pAttributes->IsKindOf(SmAttribute_TYPE)) 
        {
          // build an SmTArray<SmAttribte *> list and 
          // and place this and the last attribute on the list.
          // Store the list under m_pAttributes
          const SmContext *pContext = GetContext();
          SmTArray<SmAttribute*> * pAttrs = new (*pContext) SmTArray<SmAttribute*>(*pContext);
          pAttrs->SetDataSize(4);
          pAttrs->Add((SmAttribute*)m_pAttributes);
          pAttrs->Add(pAttribute);
          m_pAttributes = pAttrs;
        }
      else // when there are 2 or more attributes already on the list
        {
          // place the new attribute on the end of the m_pAttributes list
          SmTArray<SmAttribute*> * pAttrs = (SmTArray<SmAttribute*> *)m_pAttributes;
          pAttrs->Add(pAttribute);
        }
    }

  // tell the attribute its attached to this user
  pAttribute->AddUser(this);

} // end SmAObject::AddSlewAttribute

/*******************************************************************//**
PURPOSE: Remove an attribute from the object.  

NOTES: 
***********************************************************************/
void SmAObject::RemoveAttribute
  (SmAttribute * pAttribute,    // in : target attribute
   SmBoolean     bDoNotDelete)  // in : TRUE  = do not delete after removal
                                //      FALSE = delete attribute
                                //              if attribute's useCount goes to zero
                                //              and attribute's behavior != STANDALONE
                                //      default:[FALSE]
{
  // no work - no attributes
  if (m_pAttributes == NULL) { SE(SM_ERR);
                               return;
                             }

  // no work - no target attribute
  if(pAttribute == NULL)
    { return ; }

  // just one attribute
  if (m_pAttributes->IsKindOf(SmAttribute_TYPE)) 
    {
      // signal an error when trying to remove an attribute not on the list
      if (pAttribute != m_pAttributes) 
        { SE(SM_ERR);
          return ;
        }

      // remove it
      m_pAttributes = NULL;
      pAttribute->RemoveUser(this,bDoNotDelete);
      return;
    }
  else // a list of attributes
    {
      SmTArray<SmAttribute*> * pAttrs = (SmTArray<SmAttribute*> *)m_pAttributes;
      for (ULONG i=0; i<pAttrs->GetSize(); i++) 
        {
          SmAttribute *pAttr = (*pAttrs)[i];
          if (pAttribute == pAttr) 
            {
              pAttrs->RemoveAt(i,1);
              pAttribute->RemoveUser(this,bDoNotDelete);
              return;
            }
        }
    }

  // signal an error for trying to remove attribute not on the list
  SE(SM_ERR);

} // end SmAObject::RemoveAttribute

/*******************************************************************//**
PURPOSE: Get all of the attributes of this object.

NOTES: 
***********************************************************************/
void SmAObject::GetAttributes
  (SmTArray<SmAttribute*> & rAttributes) // out: list of all attributes
 const
{
  // init output
  rAttributes.ReSet();

  // no work - no attributes
  if (m_pAttributes == NULL) return;

  // just one attribute
  if (m_pAttributes->IsKindOf(SmAttribute_TYPE)) 
    {
      // place attribute in output list
      rAttributes.Add((SmAttribute*)m_pAttributes);
    }
  else // list of attributes
    {
      // copy attributes list into output list
      SmTArray<SmAttribute*> * pAttrs = (SmTArray<SmAttribute*> *)m_pAttributes;
      rAttributes.Append(*pAttrs);
    }

} // end SmAObject::GetAttributes

/*******************************************************************//**
PURPOSE: Find an attribute of a given type on this object.  

NOTES: Please note that currently we allow only one attribute
    of a type on a given base object.  
    If no attribute of type is found return NULL.
***********************************************************************/
SmAttribute * SmAObject::FindAttribute
  (ULONG lAttributeID)          // in : target type
 const
{
  // no work - no attributes
  if (!m_pAttributes) return NULL;
  
  // get all attributes
  SmAttribute * sData[64];
  SmTArray<SmAttribute*> sAttrs(64,sData);
  GetAttributes(sAttrs);
  if (sAttrs.GetSize() < 1) return NULL;

  // iter every attribute
  for (ULONG i=0; i<sAttrs.GetSize(); i++) 
    {
      SmAttribute *pAttr = sAttrs[i];

      // return first attribute with matching type
      if (pAttr->m_lAttributeID == lAttributeID) 
        {
          return pAttr;
        }

    } // end iter every attribute

  // attribute of target type not found
  return NULL;  

} // end SmAObject::FindAttribute

/*******************************************************************//**
PURPOSE: Get all of the attributes of given type on this object.

NOTES: 
***********************************************************************/
void SmAObject::FindSlewOfAttributes
  (ULONG lAttributeID,                    // in : target attribute type
   SmTArray<SmAttribute*> & rAttributes)  // out: array of all attributes of this type
  const
{
  // init output
  rAttributes.ReSet();

  // no work - no attributes
  if (m_pAttributes == NULL) return;

  // one attribute
  if (m_pAttributes->IsKindOf(SmAttribute_TYPE)) 
    {
      // add attribute of matching type to output list
      if(lAttributeID == ((SmAttribute*)m_pAttributes)->GetAttributeID())
        {
          rAttributes.Add((SmAttribute*)m_pAttributes);
        }
    }
  else // list of attributes
    {
      SmTArray<SmAttribute*> * pAttrs = (SmTArray<SmAttribute*> *)m_pAttributes;

      // for every attribute
      for(ULONG ii=0;ii<pAttrs->GetSize();ii++)
        {
          SmAttribute *pAttribute = (*pAttrs)[ii] ;

          // add attributes of matching type to output list
          if(lAttributeID == pAttribute->GetAttributeID())
            {
              rAttributes.Add(pAttribute);
            } // end found keeper check
        } // end iter every attribute
    } // end list of attributes check

} // end SmAObject::FindSlewOfAttributes

/*******************************************************************//**
PURPOSE: Find an attribute of a given type on this object or
     on the nearest owner if missing.  Returns NULL if no attribute
     is found on any level of the owner tree.

NOTES: 
  1. If no attribute of type is found in inheritance hierarchy, return NULL.
  2. First attribute of lAttributeID type is returned.

***********************************************************************/
SmAttribute * SmAObject::FindInheritedAttribute
  (ULONG lAttributeID) 
 const
{
  // return attribute if found on this object
  SmAttribute *pAttr = FindAttribute(lAttributeID) ;
  if(pAttr) return(pAttr) ;

  // else find the owner of this object
  SmAObject *pOwner = GetAOwner() ;

  // quit when there is no owner - else seek the attribute from the owner
  if(pOwner == NULL) return(NULL) ;
  else               return(pOwner->FindInheritedAttribute(lAttributeID)) ;

} // end SmAObject::FindInheritedAttribute

/*******************************************************************//**
PURPOSE: Return the owner from which attributes can be inherited.

NOTES: This function is used to implement attribute inheritence.
Objects which can inherit attributes need to implement this function
to specify the inheritence hierarchy
***********************************************************************/

SmAObject * SmAObject::GetAOwner() 
  const
{
  return NULL ;

} // end SmAObject::GetAOwner

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmAObject,
            all its SmVertexuses, and their attributes.

NOTES: Skips attributes which are currently marked.
                Marks attributes after adding up their memory.

   Call SmAttribute::NewMarkIO() (which just calls SmContext::NewMarkIO()
     which is the same side effect as calling SmTopology::NewMarkIO()) 
     before calling this function to refresh the global attribute 
     mark value to make sure old marks don't prevent some attributes 
     from being counted.
***********************************************************************/
ULONG SmAObject::GetAttributeMemoryUsed   // rtn: Total Memory being used by all attribute objects
  (ULONG &rlMemoryAllocated,              // out: Total Memory allocated for all attribute objects
   SmMarkType eMarkType)                  // in : uses without increment eMarkType value
 const
{
  // locals
  ULONG ii, lThisUsed, lThisAllocated ;

  // init output
  rlMemoryAllocated = 0 ;
  ULONG lAttributeMemoryUsed = 0 ; 

  // switch on number of attributes

  // no attributes - no memory used
  if(m_pAttributes == NULL) 
    { return (0) ; }

  // one attribute - return memory used by one attribute
  if(m_pAttributes->IsKindOf(SmAttribute_TYPE))
    {
      SmAttribute *pAttribute = ((SmAttribute *)m_pAttributes) ;

      // no memory for marked attributes - else return this attributes memory
      if(pAttribute->IsMarked(eMarkType)) 
        { return(0) ; }
      else                         
        { pAttribute->Mark(eMarkType) ; 
          return( pAttribute->GetMemoryUsed(rlMemoryAllocated) ) ;
        }
    }

  // many attributes - return memory used by all attributes
  SmTArray<SmAttribute*> *pArray = (SmTArray<SmAttribute*> *)m_pAttributes ;
  for(ii=0;ii<pArray->GetSize();ii++)
    {
      SmAttribute *pAttribute = pArray->GetAt(ii) ;

      // get memory used for this attribute - only count unmarked attributes
      if(FALSE == pAttribute->IsMarked(eMarkType)) 
        { 
          pAttribute->Mark(eMarkType) ;
          lThisUsed = pAttribute->GetMemoryUsed(lThisAllocated) ;

          lAttributeMemoryUsed += lThisUsed ;
          rlMemoryAllocated    += lThisAllocated ;
        } // end skip marked attributes
    } // end iter every attribute

  // all done
  return(lAttributeMemoryUsed) ;

} // end SmAObject::GetAttributeMemoryUsed

/*******************************************************************//**
PURPOSE: If this method is reached then there is no cache of the type
    requested.  Just return NULL.  Hopefully next time it will request
    something that is available.

NOTES: Use SmCacheMgr:: to access CacheMakeOrValidate
***********************************************************************/
SmStatus SmAObject::CacheMakeOrValidate
  (SmObjectCacheType ,                       // in : eObjectCacheType = oneof SM_OC_CURVE
                                             //            SM_OC_SURFACE        
                                             //            SM_OC_TRIMSRF
                                             //            SM_OC_BREP
   const SmAObject * ,                        // in : cpObject = target object
   SmCacheObj      * ,                        // in : cpOldCache = existing target Object's ObjectCache or NULL
   SmCacheObj      *& rpNewCache)             // out: ptr to target object's ObjectCache
    const
{
    rpNewCache = NULL;
    return SM_SUCCESS;

} // end SmAObject::CacheMakeOrValidate

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertAObject_list[] =
{
  {SM_AT_TYPE,    _T("Data Type"), _T("m_pAttributes is NULL or not the correct type") },
  {SM_AT_POINTER, _T("Context"),   _T("pAttrib shares the same context") },
  {SM_AT_POINTER, _T("Context"),   _T("pAttrib shares the same context") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmAObject::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  SmBoolean bRtn = TRUE ;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmObject::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ;

  // check pointer type
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, 
                                  (   m_pAttributes == NULL
                                   || m_pAttributes->IsKindOf(SmAttribute_TYPE)
                                   || m_pAttributes->IsKindOf(SmTArray_TYPE)), 
                                  _T("") ) ;


  // all attributes should share the same context as this SmAObject
  if(m_pAttributes)
    {
      if(m_pAttributes->IsKindOf(SmAttribute_TYPE))
       {
         SmAttribute *pAttrib = (SmAttribute *)m_pAttributes ;
         bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, 
                                         (   pAttrib->GetBehavior() == SM_AB_REFERENCE
                                          ||  pAttrib->GetBehavior() == SM_AB_STANDALONE_REFERENCE  
                                          ||  pAttrib->GetContext()  == GetContext() ), 
                                         _T("") ) ;

       } // end 1 attribute branch
     else // many attributes
       {
         SM_ASSERT(m_pAttributes->IsKindOf(SmTArray_TYPE)) ;
         ULONG ii, lCnt = ((SmTArray<SmAttribute*> *)m_pAttributes)->GetSize() ;
         for(ii=0; bRtn && ii<lCnt;ii++)
           {
             SmAttribute * pAttrib = ((SmTArray<SmAttribute*> *)m_pAttributes)->GetAt(ii) ; 
             bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, 
                                             (    pAttrib->GetBehavior() == SM_AB_REFERENCE
                                              ||  pAttrib->GetBehavior() == SM_AB_STANDALONE_REFERENCE  
                                              ||  pAttrib->GetContext()  == GetContext() ), 
                                             _T("") ) ;
           }
       } // end many attributes branch
    } // end any attributes check
   
  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;  

} // end SmAObject::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmAObject::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmAObject::AssertHeal fix not yet supported") ; 
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmAObject::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmAObject::IsKindOf( SM_TYPE t ) const
{
  return ((SmAObject_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Debug dump textual description of this object to the output 
     window used by smos_WriteBuffer.

NOTES: This is primarily for debugging purposes not data input/ouput.
***********************************************************************/
void SmAObject::Dump(void) 
 const
{
  // locals
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  SmTArray<SmAttribute*> sAttributes ;
  GetAttributes(sAttributes) ;

  // header
  smos_sprintf(sBuff,_T("%s[0x%p] has %ld attribute%s"), 
             GetClassString(), 
             this, 
             sAttributes.GetSize(), 
             sAttributes.GetSize() == 1 ? _T("") : _T("s")) ; 
  smos_sprintf(sBuffForFile,_T("%s has %ld attribute%s"), 
             GetClassString(), 
             sAttributes.GetSize(), 
             sAttributes.GetSize() == 1 ? _T("") : _T("s")) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // one line per attribute dump
  for(ii=0;ii<sAttributes.GetSize();ii++)
    {
      SmAttribute *pAttribute = sAttributes[ii] ;
      smos_sprintf(sBuff,_T("\n    Attrib[%ld] : "), ii) ; smos_WriteBuffer(sBuff);
      pAttribute->Dump() ;
    }  // end iter every attribute

} // end SmAObject::Dump


