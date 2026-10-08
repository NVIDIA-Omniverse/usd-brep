// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


/*******************************************************************//**
* FILE NAME --- SmHashTable.h
* PURPOSE: Utility Hash table management.
**********************************************************************/

#ifndef __SMHASHTABLE_H__
#define __SMHASHTABLE_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMASSERT_VALID_H__
#include <SmAssertArray.h>
#endif

#if defined(SM_DEBUG_CODE) || defined(SM_GFX_CODE)
  #ifndef __SMGFX_OUTPUT_H__
    #include <SmGraphicsOutput.h>
  #endif

  #ifndef __SMGFX_EXTERN_H__
    #include <SmGraphicsExtern.h>
  #endif

  #ifndef __SMBREP_H__
    #undef class SmBrep ;
    #include <SmBrep.h>
  #endif

  #ifndef __SMPLANE_H__
    #include <SmPlane.h>
  #endif
#endif // SM_DEBUG_CODE || SM_GFX_CODE

// Eliminate linux error (maybe belongs elsewhere?)
#define __int64 long long

// contains: template<class TYPE> class SM_EXPORT SmHashTable  - bucket and item organization to represent a bucket sort.
//                                                               The Bucket/Item structure is an organized tree of SmBucketItems
//           template<class TYPE> class SM_EXPORT SmBucketItem - container class with 
//                                                                 . ptrs to other SmBucketItems representing the bucket list organization
//                                                                 . an item ptr to the thing being put into buckets
//           
//           template<class TYPE> class SM_EXPORT SmPointItem - container class holding an item and the item's
//                                                              tolerant associated point position
//           template<class TYPE> class SM_EXPORT SmObjsInVoxels - a use of the SmHashTable in which
//                                                                 buckets are voxels and the items in
//                                                                 buckets are objs associated with a 
//                                                                 tolerant position. This implementation
//                                                                 supports fast item additions to buckets
//                                                                 and fast bucket search for items
//                                                                 by location. The structure can be
//                                                                 used to reduce the time to find nearest
//                                                                 neighbors in large sets of objects.

// design: A Hash table is a general design for storing items in buckets to do fast bucket sorts.
// 
//         A HashTable is a Tree of BucketItems containing:
//             1. A fixed array of Buckets       : SmHashTable::m_sBuckets     = array of ptrs to 1st BucketItem in each bucket
//                                                                               or NULL when bucket is empty.
//             2. A variable array of BucketItems: SmHashTable::m_sBucketItems = array of all BucketItems stored in the Buckets
//                                                                               where each BucketItem contains:
//                                                   - SmBucketItem::m_pNextBucketItem = a linked list of BucketItems in one Bucket
//                                                   - SmBucketItem::m_lBucketIndex    = index of bucket in SmHashTable::m_sBuckets that contains this BucketItem
//                                                   - SmBucketItem::m_pNextUsedBucket = a Ptr to the 1st BucketItem in the next used bucket skipping empty buckets.
//                                                   - SmBucketItem::m_sItem           = Ptr to item being placed in a bucket.
//                                                   - SmBucketItem::m_sItemId         = Optional user id assigned to this BucketItem
//                                                  2a. One BucketItem is entered into the BucketTree exactly one time.
//                                                  2b. Multiple BucketItems can point to the same Item.
//                                                  2c. Placing a single Item into multiple Buckets is represented by a set of 
//                                                       BucketItems all pointing to the Item and placed one unique BucketItem per Tgt Bucket.
//             3. A Root ptr to 1st used bucket  : SmHashTable::m_pUsedBuckets = 1st ptr to linked list of 1st BucketItems in used Buckets.
//             4. A method to assign BucketItems to buckets 
//             5. A method to search for items known to be in buckets.
//             
//  When in use the SmHashTable design looks like the following:
//       SmHashTable
//         { m_sBuckets     { PtrTo Bucket1_1stItem or NULL when empty, 
//                            PtrTo Bucket2_1stItem or NULL when empty, 
//                            ... 
//                            PtrTo BucketN_1stItem or NULL when empty 
//                          }
//           
//           m_sBucketItems { BucketItem1, BucketItem2, BucketItem3, ... BucketItemM } 
// 
//           m_lUsedBucketCount
//           m_pUsedBuckets -> ["1st used Bucket 1st_BucketItem" ]  ["2nd_BucketItem"   ]   ["3rd_BucketItem"   ]
//                             [ +- NextUsedBucket               ]  [                   ]   [                   ]
//           "1st used bucket" [ |  NextBucketItem +--------------> [ NextBucketItem +----> [ NextBucketItem +-----> ...
//                             [ |  Item                         ]  [ Item              ]   [ Item              ]
//                             [ |  ItemId                       ]  [ ItemId            ]   [ ItemId            ]
//                             [ |  BucketIndex                  ]  [ BucketIndex       ]   [ BucketIndex       ]
//                               |   
//                               V  
//                             ["2nd used Bucket 1st_BucketItem" ]  ["2nd_BucketItem"   ]   
//                             [ +- NextUsedBucket               ]  [                   ]   
//           "2nd used bucket" [ |  NextBucketItem +--------------> [ NextBucketItem +----> ...
//                             [ |  Item                         ]  [ Item              ]   
//                             [ |  ItemId                       ]  [ ItemId            ]   
//                             [ |  BucketIndex                  ]  [ BucketIndex       ]   
//                               |   
//                               V  
//                             ["3rd used Bucket 1st_BucketItem" ]  
//                             [ +- NextUsedBucket         ]  
//           "3rd used bucket" [ |  NextBucketItem +---------> ...
//                             [ |  Item                   ]  
//                             [ |  ItemId                 ]  
//                             [ |  BucketIndex            ]  
//                               V   
//                              ...
//                               |
//                               V
//           "UsedBucketCount ["last used Bucket 1st_BucketItem" ]
//                bucket"      

// for debug trials
// /*******************************************************************//**
// PURPOSE: A template experiment
// 
// NOTES: keep as comment for exploring compile and link bugs of the future
// ***********************************************************************/
// 
// /*******************************************************************//**
// PURPOSE: SmBaseA
// 
// NOTES:
//   SmBaseA           = SmBucketItem
//   SmBaseB           = SmPointItem
//   SmContainerLevel1 = SmHashTable
//   SmContainerLevel2 = SmObjsInVoxels
// ***********************************************************************/
// template<class TYPE> class SM_EXPORT SmBaseA 
// {
//  public:
//   TYPE m_sItem ;
// 
//   SmBoolean operator==(const SmBaseA<TYPE> &crOther) const { return(m_sItem == crOther.m_sItem) ; }
// } ; // end SmBaseA
// 
// SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertex*);          // already in SmTArray.h
// SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertexProps*);     // already in SmVertexProps.h
// 
// SM_EXPORT_TEMPLATE template class SM_EXPORT SmBaseA<SmVertex*> ;        
// SM_EXPORT_TEMPLATE template class SM_EXPORT SmBaseA<SmVertexProps*> ;
// 
// template class SM_EXPORT SmTArray<SmBaseA<SmVertex*>> ;
// template class SM_EXPORT SmTArray<SmBaseA<SmVertex*>*> ;
// template class SM_EXPORT SmTArray<SmBaseA<SmVertexProps*>> ;
// template class SM_EXPORT SmTArray<SmBaseA<SmVertexProps*>*> ;
// 
// /*******************************************************************//**
// PURPOSE: SmContainerLevel1
// 
// NOTES:
// ***********************************************************************/
// template<class TYPE> class SM_EXPORT SmContainerLevel1 
// {
//  public:
//   SmTArray<SmBaseA<TYPE>>  m_sBaseAItems ;
// 
//   SmBoolean operator==(const SmContainerLevel1<TYPE> &crOther) const { return TRUE ; }
// } ; // end SmContainerLevel1
// 
// SM_EXPORT_TEMPLATE template class SM_EXPORT SmContainerLevel1<SmVertex*> ;        
// SM_EXPORT_TEMPLATE template class SM_EXPORT SmContainerLevel1<SmVertexProps*> ;
// 
// /*******************************************************************//**
// PURPOSE: SmContainerLevel2
// 
// NOTES:
// ***********************************************************************/
// template<class TYPE> class SM_EXPORT SmContainerLevel2 
// {
//  public:
//  SmContainerLevel1<TYPE> m_sContainerLevel1_BaseA_TYPE ;
// 
//  SmBoolean operator==(const SmContainerLevel2<TYPE> &crOther) const { return TRUE ; }
// 
//  inline void Method(TYPE) ;
// 
// } ; // end SmContainerLevel2
// 
// SM_EXPORT_TEMPLATE template class SM_EXPORT SmContainerLevel2<SmVertex*> ;        
// SM_EXPORT_TEMPLATE template class SM_EXPORT SmContainerLevel2<SmVertexProps*> ;
// 
// /*******************************************************************//**
// PURPOSE: Method w error
// 
// NOTES:
// ***********************************************************************/
// template<class TYPE>
// inline void SmContainerLevel2<TYPE>::Method 
//  (TYPE sItem)      // in : Item ptr to remove from hashtable
// {
//   // update assoc m_sHashTabel.m_sBucketItems.m_lItemId (ItemId values above lPointItemIndx have decremented)
//   SmTArray<SmBaseA<TYPE>> &rBaseAItems = m_sContainerLevel1_BaseA_TYPE.m_sBaseAItems ;
// 
// } // end SmContainerLevel2::Method
// 
// SM_EXPORT_TEMPLATE template class SM_EXPORT SmContainerLevel2<SmVertex*> ;        
// SM_EXPORT_TEMPLATE template class SM_EXPORT SmContainerLevel2<SmVertexProps*> ;
// end for debug trials

/*******************************************************************//**
PURPOSE: A simple container object to represent a "point object" as the
         combination a tolerant position with a tgt object to be 
         stored in hashed bucket tables when the buckets are used 
         to represent voxels.

NOTES: 1. SmPointItem is used when the BucketItems of a HashTable are 
          used as VoxelBuckets containing lists of SmPointItems that
          fall within that bucket as shown in the SmObjsInVoxels<ObjType> 
          implementation.

          In this case: 
           <TYPE> of m_sHashTable<TYPE> in SmObjsInVoxels  = <SmPointItem<ObjType>*> and
           <ObjType> of <SmPointItem<ObjType>*> = the type of the obj stored in the voxel buckets.
              For examples: ObjType == SmVertex * to place ptrs to vertices in voxels, or
                            ObjType == SmVertexProps * to place ptrs to SmVertexProps in voxels

       2. This scheme allows nonGeometric objects,like SmVertexProps objects,
          to be sorted based on an associated tolerant position. For SmVertexProps
          that position is the SmVertexProps contained SmVertex object's position.

       3. The SmPointItem<TYPE>::Dump() function assumes TYPE is a kind of pointer.
          This one method sets the requirement that TYPE in SmPointItem be a kind of
          pointer.  Other than this limitation, the design allows nonPointer 
          objects to be organized into SmHashTable buckets. If you want to bucket
          sort items rather than pointers to items, rewrite the Dump() method to remove
          the pointer assumption.

       4. Arrays of SmPointItem<TYPE> get copied in SmObjsInVoxels.  
          SmObjsInVoxels memory management assumes:
          a. SmPointItems has no virtual functions
          b. TYPE has no virtual functions.  It's ok when TYPE is a pointer to a Type with virtual functions.

       5. Becuase of the the SmObjsInVoxels memory management and the Dump() method
          Only use TYPEs in SmObjsInVoxels which are pointers to Types
***********************************************************************/
template<class TYPE> class SM_EXPORT SmPointItem 
{
 public:
   TYPE        m_sTgtItem      = NULL ;              // the object being hashed
   SmPoint3d   m_sTgtPoint3d ;                       // 3space position for the hashed object
   SmZoneTol3d m_sTgtZoneTol3d = SM_UNDEF_DOUBLE ;   // radius of 3Space position tolerant neighborhood

   // constructor
   SmPointItem(TYPE         sTgtItem,
               SmPoint3d  & rTgtPoint3d,
               SmZoneTol3d  sTgtZoneTol3d)
              : m_sTgtItem     (sTgtItem),
                m_sTgtPoint3d  (rTgtPoint3d),
                m_sTgtZoneTol3d(sTgtZoneTol3d)
              { }

   // assignment operator
   inline SmPointItem<TYPE> &operator= (SmPointItem<TYPE> const & crOther) { m_sTgtItem      = crOther.m_sTgtItem ; 
                                                                             m_sTgtPoint3d   = crOther.m_sTgtPoint3d ; 
                                                                             m_sTgtZoneTol3d = crOther.m_sTgtZoneTol3d ;
                                                                             return *this ;
                                                                           }
   // equality operator              
   SmBoolean operator==(const SmPointItem<TYPE> &crOther) { SmBoolean bRtn = TRUE ;
                                                            if(this == &crOther) { return TRUE ; }
                                                            bRtn &= m_sTgtItem == crOther.m_sTgtItem ;
                                                            bRtn &= SM_ARE_SAME(m_sTgtZoneTol3d, crOther.m_sTgtZoneTol3d) ;
                                                            bRtn &= SmTol::InTol(m_sTgtPoint3d.DistanceBetween(crOther.m_sTgtPoint3d), m_sTgtZoneTol3d) ;
                                                            return(bRtn) ;
                                                          } // end SmPointItem<TYPE>::operator==
   // pretty print
   void Dump(ULONG lLabel, ULONG lIndent=2) const ; 

} ; // end class SmPointItem

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmPointItem<TYPE>::Dump(ULONG lLabel, ULONG lIndent) const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin line 
  smos_WriteBuffer(_T("\n")) ;

  // indent
  for(ii=0;ii<lIndent;ii++) { smos_WriteBuffer(_T(" ")) ; }

  // data - this sprintf() call is the only place where TYPE is assumed to be a pointer.  
  //        Change this call to use SmHashTables to bucket sort items instead of ptrs to items.
  smos_sprintf(sBuff,        _T("[%5lu] PointItem:[0x%p] Item:[0x%p] ZoneTol3d:[%16.16lf] Point: "), 
                   lLabel,
                   this,
                   m_sTgtItem,
                   (double)m_sTgtZoneTol3d) ; 
  smos_sprintf(sBuffForFile, _T("[%5lu] PointItem:[%s] Item:[%s] ZoneTol3d:[%16.16lf] Point: "), 
                   lLabel,
                   _T("NotNULL"),
                   m_sTgtItem ? _T("NotNULL") : _T("NULL"),
                   (double)m_sTgtZoneTol3d) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;
  m_sTgtPoint3d.Dump() ;
  
} // end SmPointItem<TYPE>::Dump

SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertex*);                  // already in SmTArray.h
SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertexProps*);             // already in SmVertexProps.h

SM_EXPORT_TEMPLATE template class SM_EXPORT SmPointItem<SmVertex*> ;
SM_EXPORT_TEMPLATE template class SM_EXPORT SmPointItem<SmVertexProps*> ;

SM_TARRAY_TEMPLATE_PREDECLARATION(SmPointItem<SmVertex*>);
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPointItem<SmVertex*>*);
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPointItem<SmVertexProps*>);
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPointItem<SmVertexProps*>*);

/*******************************************************************//**
PURPOSE: A class to represent an item in a SmHashTable bucket

NOTES: 
 1. A Hash table is a list of indexed buckets.
 2. Each bucket is a linked list of SmBucketItems.
 3. Each SmBucketItem contains members 
      o. to structure the SmBucketItems into a directed Tree of buckets:
           SmBucketItem<TYPE> * m_pNextBucketItem = a linked list of BucketItems within one HashTable's Bucket 
           SmBucketItem<TYPE> * m_pNextUsedBucket = ptr to first BucketItem in the HashTable's next used Bucket.
           ULONG                m_lBucketIndex    = effectively a back pointer to the SmHashTable's bucket containing
                                  this BucketItem stored as an index in the SmHashTable's array of buckets
                                  rather than as a pointer.
      o. to identify the object being placed into the bucket:
           TYPE  m_sItem   = a ptr to or an object being placed into a SmHashTable bucket
           ULONG m_lItemId = Optional user id assigned to this BucketItem
 4. The SmBucketItem<TYPE>::Dump() function assumes TYPE is a kind of pointer.
    This one method sets the requirement that TYPE in SmPointItem be a kind of
    pointer.  Other than this limitation, the design allows nonPointer 
    objects to be organized into SmHashTable buckets. If you want to bucket
    sort items rather than pointers to items, rewrite the Dump() method to remove
    the pointer assumption.
 5. memory management assumes 
     a. SmBucketItem has no Virtual functions.
     b. class TYPE has no virtual functions.  (gwc: It's ok when TYPE is a pointer to a Type with virtual functions.)

***********************************************************************/
template<class TYPE> class SM_EXPORT SmBucketItem 
{
 protected:
  SmBucketItem<TYPE> * m_pNextUsedBucket = NULL ;              // linked list pointer to next used bucket's 1st BucketItem in the HashTable 
                                                               //   only the 1st bucketItem in the bucketList uses this pointer.
                                                               //   For Removed Buckets: set to NULL

  SmBucketItem<TYPE> * m_pNextBucketItem = NULL ;              // linked list pointer to next BucketItem placed in this Bucket
                                                               //   For Removed Buckets: set to NULL

  ULONG                m_lBucketIndex    = SM_UNDEF_ULONG ;    // Index into the SmHashTable::m_sBuckets array that contains this BucketItem
                                                               //   For Removed Buckets: set to SM_UNDEF_ULONG

  TYPE                 m_sItem           = NULL ;              // the object being placed into a Bucket   
  ULONG                m_lItemId         = 0 ;                 // Optional user id assigned to this BucketItem 

 public:
   // Item and bucket Index constructor
   SmBucketItem(TYPE  sItem,
                ULONG lBucketIndex,
                ULONG lItemId=SM_UNDEF_ULONG)
                  : m_pNextUsedBucket(NULL), 
                    m_pNextBucketItem(NULL), 
                    m_lBucketIndex   (lBucketIndex),     
                    m_sItem          (sItem), 
                    m_lItemId        (lItemId)         
                  { }

   // empty constructor
   SmBucketItem() { Init() ; }

   // init memory to unused values
   void Init() { m_pNextUsedBucket = NULL ;
                 m_pNextBucketItem = NULL ;
                 m_lBucketIndex    = SM_UNDEF_ULONG ;  
                 m_sItem           = NULL ; 
                 m_lItemId         = SM_UNDEF_ULONG ; 
               }

   // destructor
   ~SmBucketItem() { m_pNextUsedBucket = NULL ; 
                     m_pNextBucketItem = NULL ; 
                     m_lBucketIndex    = SM_UNDEF_ULONG ; 
                     m_sItem           = NULL ; 
                     m_lItemId         = SM_UNDEF_ULONG ;
                   } 

   // assignment operator
   inline SmBucketItem<TYPE> &operator= (SmBucketItem<TYPE> const & crOther) ; 

   // equality operator
   SmBoolean operator==(const SmBucketItem<TYPE> &crOther) const { return(   m_lBucketIndex == crOther.m_lBucketIndex 
                                                                          && m_sItem        == crOther.m_sItem 
                                                                          && m_lItemId      == crOther.m_lItemId) ; }

   // simple data access
   SmBucketItem<TYPE> *  GetNextUsedBucket()    const { return m_pNextUsedBucket ; } // ptr to the lead item in another used bucket - only the 1st item in bucket is NonNULL
   SmBucketItem<TYPE> *& GetNextUsedBucketRef()       { return m_pNextUsedBucket ; } // ptr to the lead item in another used bucket - only the 1st item in bucket is NonNULL
   SmBucketItem<TYPE> *  GetNextBucketItem   () const { return m_pNextBucketItem ; } // ptr to the next bucketObj within a single bucket linked list
   SmBucketItem<TYPE> *& GetNextBucketItemRef()       { return m_pNextBucketItem ; } // ptr to the next bucketObj within a single bucket linked list
   TYPE                  GetItem          ()    const { return m_sItem ; }           // ptr to the item held within a bucketobj
   TYPE                & GetItemRef       ()          { return m_sItem ; }           // ptr to the item held within a bucketobj
   ULONG                 GetBucketIndex   ()    const { return m_lBucketIndex ; }    // HashTable bucket index
   ULONG               & GetBucketIndexRef()          { return m_lBucketIndex ; }    // HashTable bucket index
   ULONG                 GetItemId        ()    const { return m_lItemId ; }         // optional user id assigned to this bucket item.

   SmBoolean             IsRemovedBucketItem()  const { return m_lBucketIndex == SM_UNDEF_ULONG ; }

   void          SetNextUsedBucket(SmBucketItem<TYPE> * pNextBucket) { m_pNextUsedBucket = pNextBucket ; }
   void          SetNextItem      (SmBucketItem<TYPE> * pNextItem)   { m_pNextBucketItem = pNextItem ; }
   void          SetItem          (TYPE                sItem)        { m_sItem           = sItem ; }
   void          SetBucketIndex   (ULONG               lIndex)       { m_lBucketIndex    = lIndex ; }
   void          SetItemId        (ULONG               lItemId)      { m_lItemId         = lItemId ; }
                                  
   // pretty print
   void Dump(ULONG lLabel, ULONG lIndent=2) const ; 

} ; // end class SmBucketItem

/*******************************************************************//**
PURPOSE: BucketItem assignment operator

NOTES: caution: after this call, both this and the crOther BucketItems
       point to the same m_pNextBucketItem BucketItem which is not a proper
       use of a linked list to represent the array of items in one
       SmHashTable bucket.  The caller needs to organize these pointers
       as desired to maintain the integrity of the hash table linked lists
       of items.
***********************************************************************/
template<class TYPE>
inline SmBucketItem<TYPE> & SmBucketItem<TYPE>::operator=          
 (SmBucketItem<TYPE> const & crOther)    
{ 
  // no work - same object
  if(&crOther == this) return *this ;

  m_pNextBucketItem = crOther.m_pNextBucketItem ;
  m_lBucketIndex    = crOther.m_lBucketIndex ;
  m_sItem           = crOther.m_sItem ;
  m_lItemId         = crOther.m_lItemId ;

  // all done    
  return *this ;

} // end  SmBucketItem<TYPE>::operator=

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmBucketItem<TYPE>::Dump(ULONG lLabel, ULONG lIndent) const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin line 
  smos_WriteBuffer(_T("\n")) ;

  // indent
  for(ii=0;ii<lIndent;ii++) { smos_WriteBuffer(_T(" ")) ; }

  // data - NextUsedBucket is only NonNULL for 1st entry in a bucket list
  //        this sprintf() call is the only place where TYPE is assumed to be a pointer.  Change
  //        this call to use SmHashTables to bucket sort items instead of ptrs to items.
  smos_sprintf(sBuff,        _T("[%5lu] InBucket:[%5lu] SmBucketItem[0x%p]: Item:[0x%p] ItemId:[%d] NextBucketItem:[0x%p] NextUsedBucket:[0x%p]"), 
                   lLabel,
                   m_lBucketIndex, 
                   this,
                   m_sItem,
                   m_lItemId,
                   m_pNextBucketItem, 
                   m_pNextUsedBucket) ; 
  smos_sprintf(sBuffForFile, _T("[%5lu] InBucket:[%5lu] SmBucketItem[%s]: Item:[%s] ItemId:[%d] NextBucketItem:[%s] NextUsedBucket:[%s]"), 
                   lLabel,
                   m_lBucketIndex, 
                   _T("NotNULL"),
                   m_sItem ? _T("NotNULL") : _T("NULL"),
                   m_lItemId,
                   m_pNextBucketItem ? _T("NotNULL") : _T("NULL"),
                   m_pNextUsedBucket ? _T("NotNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;
  
} // end SmBucketItem<TYPE>::Dump

SM_EXPORT_TEMPLATE template class SM_EXPORT SmBucketItem<SmVertex*> ;
SM_EXPORT_TEMPLATE template class SM_EXPORT SmBucketItem<SmVertexProps*> ;

SM_EXPORT_TEMPLATE template class SM_EXPORT SmBucketItem<SmPointItem<SmVertex*>*> ;
SM_EXPORT_TEMPLATE template class SM_EXPORT SmBucketItem<SmPointItem<SmVertexProps*>*> ;
//SM_EXPORT_TEMPLATE template class SM_EXPORT SmBucketItem<SmPointItem<SmVertex*>> ;
//SM_EXPORT_TEMPLATE template class SM_EXPORT SmBucketItem<SmPointItem<SmVertexProps*>> ;

SM_TARRAY_TEMPLATE_PREDECLARATION(SmBucketItem<SmVertex*>) ;      
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBucketItem<SmVertex*>*) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBucketItem<SmVertexProps*>) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBucketItem<SmVertexProps*>*) ;

SM_TARRAY_TEMPLATE_PREDECLARATION(SmBucketItem<SmPointItem<SmVertex*>*>) ;      
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBucketItem<SmPointItem<SmVertex*>*>*) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBucketItem<SmPointItem<SmVertexProps*>*>) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBucketItem<SmPointItem<SmVertexProps*>*>*) ;

/*******************************************************************//**
PURPOSE: A class to contain a bucket sort of indexable objects

NOTES: Expected use:
  Given a known number of items that can be hashed into a unique index number
  ranging from 0 to MaxIndexVal
   - Place the items into the Hash Table
   - Walk the hash table to look at the bucket sorted items
   - a linked list of the used buckets is maintained to reduce the cost of walking the hash table

EXAMPLE:
  example() // an order[bucketCount * bucketlength N**2] compare algorithm
    {
      // sizes
      ULONG lItemCount   = Count ;
      ULONG lBucketCount = lMaxIndexVal + 1 ;

      // make and populate some kind of object list
      SmTArray<SmObject *> sObjectList(lItemCount, NULL, lItemCount) ;

      // make the HashTable
      SmHashTable<TYPE> sHashTable(lItemCount, lBucketCount) ;

      // place objects into the hash table buckets such that items in different buckets are known not to be equivalent
      for(ii=0;ii<lItemCount;ii++)
        {
          SmObject *pObject = sObjectList[ii] ;
          ULONG lIndex = MyGetIndexForObj(pObject) ;  // user writes the hash method

          sHashTable.Add(pObject, lIndex) ;
        }

      // look for equivalence between the members of the each bucket
      // example a bucketlength N**2 compare algorithm

      // get the 1st used bucket - this is just the 1st bucketobject within a linklist of bucket objects in the used bucket
      SmBucketItem<TYPE> * pUsedBucket = sHashTable.GetUsedBucket(0) ; 

      for(;pUsedBucket!=NULL;pUsedBucket = pUsedBucket->GetNextUsedBucket())
        {
          SmBucketItem<TYPE> * pBegBucketItem = pUsedBucket ;

          // while the bucket has entries
          while(pBegBucketItem != NULL)
            {
              // get first pair BucketItem to compare
              SmHashObj * pEndBucketItem = pBegBucketItem != NULL ? pBegBucketItem->GetNext() : NULL ;

              // while there are pairs of BucketItems in this bucket to compare
              while(pEndBucketItem != NULL)
                {
                  // check items for equality
                  if(pBegBucketItem->GetItem() == pEndBucketItem->GetItem())
                    {
                      // do something
                    }

                  // increment the EndBucketItem pointer
                  pEndBucketItem = pEndBucketItem->GetNextBucketItem() ; 
                } // end iter every bucket end entry

               // increment the BegBucketItem ptr for the next set of pair compares in this bucket
               pBegBucketItem = pBegBucketItem->GetNextBucketItem() ; 

            } // end iter every bucket front entry
        } // end iter every bucket
    } // end example

EXAMPLE - HASHTABLE'S DIRECTED TREE STRUCTURE:
  When in use the SmHashTable tree structure looks like the following:
       SmHashTable
         { m_sBuckets     { PtrTo Bucket1_1stBucketItem or NULL when empty, 
                            PtrTo Bucket2_1stBucketItem or NULL when empty, 
                            ... 
                            PtrTo BucketN_1stBucketItem or NULL when empty 
                          }
           m_sBucketItems { BucketItem1, BucketItem2, BucketItem3, ... BucketItemM } 
 
           m_lUsedBucketCount
           m_pUsedBuckets -> ["1st used Bucket 1st_BucketItem" ]  ["2nd_BucketItem"   ]   ["3rd_BucketItem"   ]
                             [ +- NextUsedBucket               ]  [                   ]   [                   ]
           "1st used bucket" [ |  NextBucketItem +--------------> [ NextBucketItem +----> [ NextBucketItem +-----> ...
                             [ |  Item                         ]  [ Item              ]   [ Item              ]
                             [ |  ItemId                       ]  [ ItemId            ]   [ ItemId            ]
                             [ |  BucketIndex                  ]  [ BucketIndex       ]   [ BucketIndex       ]
                               |   
                               V  
                             ["2nd used Bucket 1st_BucketItem" ]  ["2nd_BucketItem"   ]   
                             [ +- NextUsedBucket               ]  [                   ]   
           "2nd used bucket" [ |  NextBucketItem +--------------> [ NextBucketItem +----> ...
                             [ |  Item                         ]  [ Item              ]   
                             [ |  ItemId                       ]  [ ItemId            ]   
                             [ |  BucketIndex                  ]  [ BucketIndex       ]   
                               |   
                               V  
                             ["3rd used Bucket 1st_BucketItem" ]  
                             [ +- NextUsedBucket         ]  
           "3rd used bucket" [ |  NextBucketItem +---------> ...
                             [ |  Item                   ]  
                             [ |  ItemId                 ]  
                             [ |  BucketIndex            ]  
                               V   
                              ...
                               |
                               V
           "UsedBucketCount ["last used Bucket 1st_BucketItem" ]
                bucket"      
***********************************************************************/
template<class TYPE> class SM_EXPORT SmHashTable 
{                                
 protected:                      
  SmTArray<SmBucketItem<TYPE> *> m_sBuckets ;               // Array of buckets; bucket = linked BucketItem list, sized:[lBucketCount] in the constructor.
                                                            // Stores the pointer to the 1st SmBucketItem in the bucket, else NULL
  SmTArray<SmBucketItem<TYPE>>   m_sBucketItems ;           // Array of BucketItems sorted into m_sBuckets.
                                                            //   A single m_sItem may be placed in one or more Buckets. Create and place a
                                                            //   unique BucketItem into each Bucket that contains the m_sItem.
                                                            // SmBucketItem contains: SmBucketItem * SmBucketItem::m_pNextUsedBucket   // linked list pointer to next used bucket in the HashTable 
                                                            //                                                                         //  only the 1st bucketItem in the bucketList uses this pointer.
                                                            //                        SmBucketItem * SmBucketItem::m_pNextBucketItem   // linked list pointer to next HashObj placed in this hash bucket
                                                            //                        void         * SmBucketItem::m_sItem             // pointer to this Hash object   
                                                            //                        ULONG          SmBucketItem::m_lBucketIndex      // bucket index for this entry
                                                            //                        ULONG          SmBucketItem::m_lItemIndx         // m_pItems index for this entry
  ULONG                     m_lRemovedBucketItemCount = 0 ; // number of removed bucketItems.  1. Removed BucketItems are
                                                            //                                    a. removed from the m_sBucket link-list of bucket items
                                                            //                                    b. not removed from the m_sBucketItems list so that all the 
                                                            //                                         BucketItemPtr values stored within the m_sBuckets linked list
                                                            //                                         of BucketItemPtr values don't become stale when a single
                                                            //                                         BucketItem is removed.
                                                            //                                 2. UsedBucketItemCount + RemovedBucketItemCount == m_sBucketItems.GetSize()
                                                            //                                 3. For removed PointItem, m_sPointItems[ii]->m_lBucketIndex    == SM_UNDEF_ULONG
                                                            //                                                           m_sPointItems[ii]->m_pNextUsedBucket == NULL
                                                            //                                                           m_sPointItems[ii]->m_pNextBucketItem == NULL
  SmBucketItem<TYPE> *      m_pUsedBuckets = NULL ;         // link list of 1st BucketItem in each used Bucket
  ULONG                     m_lUsedBucketCount = 0 ;        // number of used buckets (length of m_pUsedBuckets linked list)
                                                            
 public:                                                    
  // Constructor                                 
  SmHashTable(ULONG lItemEstimate,                          // in : estimated number of items to be placed in the buckets
              ULONG lBucketCount = 0)                       // in : Number of buckets to contain sorted items    
                                                            { InitSize(lItemEstimate, lBucketCount) ; }

  // empty bucket contents and set buckets and item counts as specified
  inline void InitSize(ULONG lItemEstimate,                 // in : estimated number of items to be placed in the buckets
                       ULONG lBucketCount) ;                // in : Number of buckets to contain sorted items    
  
  // increase m_lMaxSize for m_sBucketItems array without emptying the current bucket contents            
  inline void IncMaxItemCount() ;                          

  // equality operator
  SmBoolean operator==(const SmHashTable<TYPE> &crOther) const ;

  // destructor
 ~SmHashTable() { InitSize(0, 0) ; }

  // simple data acces
  SmTArray<SmBucketItem<TYPE> *> & GetBuckets    ()                      { return m_sBuckets ; } 
  SmTArray<SmBucketItem<TYPE>>   & GetBucketItems()                      { return m_sBucketItems ; }                  // array of active and removed BucketItems
  SmBucketItem<TYPE>             & GetBucketItem(ULONG lItemIndx)        { return m_sBucketItems.GetAt(lItemIndx) ; } // array of active and removed BucketItems
  SmBucketItem<TYPE>             * GetBucket    (ULONG lBucketIndx)      { return m_sBuckets.GetAt(lBucketIndx) ; }  // ptr to 1st bucketItem in bucket or NULL for empty buckets
  SmBucketItem<TYPE>             * GetUsedBucket(ULONG lUsedBucketIndx)  { SM_ASSERT_BREAK(lUsedBucketIndx < m_lUsedBucketCount) ;
                                                                           SmBucketItem<TYPE> * pUsedBucket = m_pUsedBuckets ;
                                                                           for(ULONG ii=0;ii<lUsedBucketIndx;ii++) { pUsedBucket = pUsedBucket->GetNextUsedBucket() ; }
                                                                           return(pUsedBucket) ;
                                                                         }
  ULONG  GetBucketCount()                                 const { return m_sBuckets.GetSize() ; }
  ULONG  GetUsedBucketCount()                             const { return m_lUsedBucketCount ; }
                                                          
  ULONG  GetTotalBucketItemCount()                        const { return m_sBucketItems.GetSize() ; }  // number of used and removed BucketItems in all buckets
  ULONG  GetUsedBucketItemCount()                         const { return m_sBucketItems.GetSize() - m_lRemovedBucketItemCount ; } // number of used BucketItems in all buckets
  ULONG  GetRemovedBucketItemCount()                      const { return m_lRemovedBucketItemCount ; } // number of removed BucketItems left behind in m_sBucketItems list
  ULONG  GetOneBucketItemCount   (ULONG lBucketIndx)      const ; // number of BucketItems in bucket[lBucketIndx]
  void   GetOneBucket_BucketItems(ULONG BucketIndx,               // get list of all BucketItems in one bucket
            SmTArray<SmBucketItem<TYPE>*> & rBucketItems) const ;

  // predicates
  SmBoolean IsInBucket(ULONG lBucketIndex, const SmBucketItem<TYPE> * pTgtBucketItem) const { SmBoolean bRtn = (lBucketIndex == pTgtBucketItem->GetBucketIndex()) ;
                                                                                              // linear search 
                                                                                              SmBucketItem<TYPE> * pBucketItem = m_sBuckets[lBucketIndex] ;
                                                                                              for(;pBucketItem!=NULL;pBucketItem=pBucketItem->GetNextBucketItem())
                                                                                                {
                                                                                                  if(pBucketItem == pTgtBucketItem) { return(bRtn) ; }
                                                                                                }
                                                                                              return(FALSE) ;
                                                                                            }
  SmBoolean IsUsedBucket(ULONG lBucketIndex) const { return(m_sBuckets[lBucketIndex] != NULL) ; }

  // side effects

  // Add item to HashTable - 1st time a bucket is used that item is added to the link list of used buckets - this is fast
  void Add(TYPE sItem, ULONG lBucketIndex, ULONG lOptItemId=SM_UNDEF_ULONG) ;

  // change the list of TgtBucketItem indices
  ULONG Remove(SmTArray<ULONG> &rTgtIndices) ;

  // return TRUE when object is tested as valid
  SmBoolean AssertValid(SmAssertArray    * pAList=NULL,             // i/o: Accumulating list of failed Asserts, NULL to ignore                                             
                          SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                      
                                                                    //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                  
                                                                    //    : default:[SM_LEVEL_0]                                                                            
                          SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
                          SmTArray<ULONG>  * pTestRequests=NULL     //NotUsed: in: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]       
                        )  const ;

  // Define  GetType(), IsKindOf(), and Dump() (Dump needs local implementation)
  SM_COMMON_ONLY_BASE(SmHashTable,SmHashTable_TYPE);

  // pretty print
  void Dump           (SmBoolean bFull)                            const ; // in : TRUE = display the bucket item contents. FALSE=don't
  void DumpBuckets    (SmTArray<ULONG> &rBucketIndices)            const ; // in : Array of BucketIndices to pretty print

} ; // end class SmHashTable

/*******************************************************************//**
PURPOSE: empty buckets and reinit all member values

NOTES: m_sBucketItems memory management: Set
          m_sBuckets.m_lSize     = lBucketCount and
          m_sBucketItems.m_lSize = lMaxSize = number greater than lItemEstimate
          where lMaxSize = Max(1.5 * lItemEstimate, lItemEstimate + 64)

       The m_sBucketItems array is allocated with more than lItemEstimate number
       of BucektItem slots so that the Add() method can add new Items without
       having to reallocate memory.

       When the initial memory block for m_sBucketItems is exceeded by Add(),
       Add() calls IncMaxItemCount() which allocates an even larger block of memory
       for many future Add() calls and copies all the existing BucketItem structure
       into the new memory block.  This is done to minimize the number of times
       a SmHashTable might reallocate memory for the m_sBucketItems array
       as BucketItems are added to buckets.
***********************************************************************/
template<class TYPE>
inline void SmHashTable<TYPE>::InitSize
 (ULONG lItemEstimate,  // in : estimated number of items to be placed in the buckets
  ULONG lBucketCount)   // in : Number of buckets to contain sorted items                       
{ 
  // init the used bucket link list
  m_pUsedBuckets = NULL ; 

  // empty linked lists - resize empty bucket and item lists.
  m_sBuckets.SetSize(lBucketCount) ; // m_sBuckets: m_lSize = lBucketCount, m_lMaxSize >= lBucketCount
  m_sBuckets.SetAll(NULL) ;
  m_sBucketItems.RemoveAll() ; // m_sBucketItems: m_lSize and m_lMaxSize are set to 0, memory freed

  // when ItemEstimate is greater than 0
  if(lItemEstimate > 0)
    {
      // pre allocated more than enough m_pData memory 
      ULONG lMaxSize = smos_Max( (ULONG)(1.5 * (double)lItemEstimate), lItemEstimate + 64) ;
      m_sBucketItems.SetDataSize(lMaxSize) ;  // m_sBucketItems::m_lSize    (GetSize())     == 0 
                                              // m_sBucketItems::m_lMaxSize (GetDataSize()) == lMaxSize
      // if m_sBucketItems.SetSize(0) is run here - then the memory for m_pData is freed - don't do it
    }

  // clear counters
  m_lUsedBucketCount = 0 ;
  m_lRemovedBucketItemCount = 0 ;

} // end SmHashTable<TYPE>::InitSize

/*******************************************************************//**
PURPOSE: increase m_sBucketItems array's m_lMaxSize while preserving 
         the first m_lSize m_sBucketItem values and fixing stale ptr values as needed. 

NOTES: When m_sBucketItems increases in size beyond the arrays->m_lMaxSize
       the old array is replaced by a new larger data array whose
       member values are copied from the values of the old array.
       That means all the pointers stored in m_sBuckets and in the
       m_sBucketItems[ii] members are now stale and have to be updated.

       This method copies the old data into the new array and then
       walks the used items and the BucketItems list updating the stored
       pointers.

METHOD: 0. Pick new BucketItem MaxSize
        1. allocate a new memory block for m_sBucketItems
        2. Copy the old m_sBucketItems data into the new array
        3. update pointers in m_sBuckets
        4. m_pUsedBuckets pointer needs to be updated 
        5. clear all the pointers in the old m_sBucketItems array to find memory managment errors
        6. swap the m_sBucketItems data array and delete the old m_sBucketItems data array memory

***********************************************************************/
template<class TYPE>
inline void SmHashTable<TYPE>::IncMaxItemCount() // in : desired MaxItemCount (bigger than the HashedItemCount)
{ 
  // locals
  ULONG ii ;

  // m_sBuckets stays the same (its stored pointer values will be updated later

  // locals - old m_sBucketItem memory block and size
  ULONG                lOldBucketItemMaxSize = m_sBucketItems.GetDataSize() ;
  SmBucketItem<TYPE> * pOldBucketItemArray   = m_sBucketItems.GetDataArray() ;

  // new m_sBucketItems memory size and block - pick a big new lMaxSize so we don't have to run this method very often
  ULONG                lNewBucketItemMaxSize = smos_Max( (ULONG)(1.5 * m_sBucketItems.GetDataSize()), m_sBucketItems.GetDataSize() + 64) ;
  SmBucketItem<TYPE> * pNewBucketItemArray   = (SmBucketItem<TYPE> *)smos_Calloc(1, lNewBucketItemMaxSize * sizeof(SmBucketItem<TYPE>)) ;

  // init new elements with '0' characters - WARNING: when TYPE has virtual functions, this call corrupts the virtual function table
  smos_MemSet((void*)(pNewBucketItemArray), 0, lNewBucketItemMaxSize * sizeof(SmBucketItem<TYPE>)) ;

  // when the OldHashTable has HashedItems - copy old bucket structure into New Array
  if(GetUsedBucketItemCount() > 0)
    {
      // copy all BucketItem structures from old to new - only okay to use smos_MemCpy because SmBucketItems have no virtual methods
      SE(smos_MemCpy((void*)pNewBucketItemArray,                               // out: ptr to mem array to copy to  
                     (void*)pOldBucketItemArray,                               // in : ptr to mem array to copy from 
                     GetTotalBucketItemCount() * sizeof(SmBucketItem<TYPE>),   // in : size of src,  (typedef unsigned __int64 size_t;) 
                     lNewBucketItemMaxSize     * sizeof(SmBucketItem<TYPE>))); // in : size of dest, (typedef unsigned __int64 size_t;) 
      
      // now fix the m_pNextBucketItem and m_pNextUsedBucket pointers - use ptr math: get index of nextItem, store that index address from the new array
      for(ii=0;ii<GetTotalBucketItemCount();ii++)
        {
          // when NextItem ptrs are NonNULL
          if(pOldBucketItemArray[ii].GetNextBucketItem() != NULL)
            {
              // set the pointer value for the new array
              __int64 lNextItemIndx = pOldBucketItemArray[ii].GetNextBucketItem() - pOldBucketItemArray ;
              pNewBucketItemArray[ii].SetNextItem(&pNewBucketItemArray[lNextItemIndx]) ;
            } // end NextItem NotNULL check

          // when NextUsedBucket ptrs are NonNULL
          if(pOldBucketItemArray[ii].GetNextUsedBucket() != NULL)
            {
              // set the pointer value for the new array
              __int64 lNextUsedBucketIndx = pOldBucketItemArray[ii].GetNextUsedBucket() - pOldBucketItemArray ;
              pNewBucketItemArray[ii].SetNextUsedBucket(&pNewBucketItemArray[lNextUsedBucketIndx]) ;
            } // end NextItem NotNULL check
        } // end iter every BucketItem

      // fix the m_sBuckets ptrs - skip NULL (empty) buckets to avoid undefined pointer arithmetic
      for(ii=0;ii<m_sBuckets.GetSize();ii++)
        {
          if(m_sBuckets[ii] != NULL)
            {
              __int64 lBucketHeadIndx = m_sBuckets[ii] - pOldBucketItemArray ;
              m_sBuckets.SetAt(ii, &pNewBucketItemArray[lBucketHeadIndx]) ;
            }
        }

      // fix the m_pUsedBuckets pointer
      __int64 lUsedBucketsIndx = m_pUsedBuckets - pOldBucketItemArray ;
      m_pUsedBuckets = &pNewBucketItemArray[lUsedBucketsIndx] ;

    } // end HashTable has HasedItems check

  // clear old elements with '0' characters - WARNING: when TYPE has virtual functions, this call corrupts the virtual function table
  smos_MemSet((void*)(pOldBucketItemArray), 0, lOldBucketItemMaxSize * sizeof(SmBucketItem<TYPE>)) ;

  // save new DataArray in m_sBucketItems and delete old data array
  m_sBucketItems.SetArray(lNewBucketItemMaxSize,      // new m_lMaxSize
                          pNewBucketItemArray,        // new array
                          m_sBucketItems.GetSize()) ; // new m_lSize - number of slots used in new aray: m_lSize, frees the old m_pData array
  m_sBucketItems.SetIsBorrowed(FALSE) ;         // sets the m_bIsBorrowed flag

  // m_lUsedBucketCount = no change ;
  // m_lRemovedBucketItemCount = no change ;

} // end SmHashTable<TYPE>::IncItemCount

/*******************************************************************//**
PURPOSE: SmHealData Equality operator

NOTES:
***********************************************************************/
template<class TYPE>
inline SmBoolean SmHashTable<TYPE>::operator==(const SmHashTable<TYPE> &crOther ) const
{
  // return value
  SmBoolean bRtn = TRUE ;

  // no work
  if(this == &crOther)
    { return TRUE ; }

  // simple data access check
  bRtn &= m_sBuckets.GetSize()      == crOther.m_sBuckets.GetSize() ;
  bRtn &= m_sBucketItems.GetSize()  == crOther.m_sBucketItems.GetSize() ;
  bRtn &= m_lUsedBucketCount        == crOther.m_lUsedBucketCount ;
  bRtn &= m_lRemovedBucketItemCount == crOther.m_lRemovedBucketItemCount ; 

  // when simple data matches
  if(bRtn == TRUE)
    {
      ULONG ii ;

      // check the item structure
      for(ii=0;ii<m_sBucketItems.GetSize() && bRtn;ii++)
        {
          bRtn &= (   (   ((SmHashTable<TYPE> *)this)  ->m_sBucketItems.GetAt(ii).GetNextUsedBucket() == NULL
                       && ((SmHashTable<TYPE> &)crOther).m_sBucketItems.GetAt(ii).GetNextUsedBucket() == NULL)
                   || (   ((SmHashTable<TYPE> *)this)  ->m_sBucketItems.GetAt(ii).GetNextUsedBucket()->GetBucketIndex()
                       == ((SmHashTable<TYPE> &)crOther).m_sBucketItems.GetAt(ii).GetNextUsedBucket()->GetBucketIndex()) 
                  ) ;
          bRtn &= (   (   ((SmHashTable<TYPE> *)this)  ->m_sBucketItems.GetAt(ii).GetNextBucketItem() == NULL
                       && ((SmHashTable<TYPE> &)crOther).m_sBucketItems.GetAt(ii).GetNextBucketItem() == NULL)
                   || (   (((SmHashTable<TYPE> *)this)  ->m_sBucketItems.GetAt(ii).GetNextBucketItem()->GetBucketIndex())
                       == (((SmHashTable<TYPE> &)crOther).m_sBucketItems.GetAt(ii).GetNextBucketItem()->GetBucketIndex()))  
                  ) ;
          bRtn &= ((SmHashTable<TYPE> *)this)->m_sBucketItems.GetAt(ii).GetItem()           == ((SmHashTable<TYPE> &)crOther).m_sBucketItems.GetAt(ii).GetItem() ;         
          bRtn &= ((SmHashTable<TYPE> *)this)->m_sBucketItems.GetAt(ii).GetBucketIndex()    == ((SmHashTable<TYPE> &)crOther).m_sBucketItems.GetAt(ii).GetBucketIndex() ; 
        }

      // check the Bucket structure
      for(ii=0;ii<m_sBuckets.GetSize() && bRtn;ii++)
        {
          bRtn &= (   (   this  ->m_sBuckets.GetAt(ii) == NULL
                       && crOther.m_sBuckets.GetAt(ii) == NULL)
                   || (   this  ->m_sBuckets.GetAt(ii)->GetBucketIndex()
                       == crOther.m_sBuckets.GetAt(ii)->GetBucketIndex()) 
                  ) ;
        }
    } // end simple data matches check

  // all done
  return(bRtn) ;

} // end SmHashTable<TYPE>::operator==

/*******************************************************************//**
PURPOSE: Return number of entries in one target Hash table bucket

NOTES: 
***********************************************************************/
template<class TYPE>
inline ULONG SmHashTable<TYPE>::GetOneBucketItemCount 
 (ULONG lBucketIndx)   // in : tgt bucket
 const
{ 
  // rtn value
  ULONG lCnt = 0 ;

  // locals 
  SmBucketItem<TYPE> * pBucketItem = m_sBuckets.GetAt(lBucketIndx) ;
  SM_ASSERT_MSG(   pBucketItem == NULL
                || pBucketItem->GetItem() != NULL,
                _T("SmHashTable::GetOneBucketItemCount: error found a BucketItem with a NULL sItem entry!")) ;  // gwc: does this make sense for nonPointer objects?

  // while there are BucketItems in the tgt bucket
  while(pBucketItem != NULL)
    { 
      // count them
      lCnt ++ ;
      pBucketItem = pBucketItem->GetNextBucketItem() ;
      SM_ASSERT_MSG(   pBucketItem == NULL
                    || pBucketItem->GetItem() != NULL,
                    _T("SmHashTable::GetOneBucketItemCount: error found a BucketItem with a NULL sItem entry!")) ;  // gwc: does this make sense for nonPointer objects?
    }

  // all done
  return lCnt ;

} // end SmHashTable<TYPE>::GetOneBucketItemCount

/*******************************************************************//**
PURPOSE: Return list of all BucketItems in one Bucket

NOTES: 
***********************************************************************/
template<class TYPE>
inline void SmHashTable<TYPE>::GetOneBucket_BucketItems 
 (ULONG                           lBucketIndx,   // in : tgt bucket    
  SmTArray<SmBucketItem<TYPE>*> & rBucketItems)  // out: list of BucketItem ptrs contained in Buckets[lBucketIndx]
 const
{ 
  // init rt value
  rBucketItems.ReSet() ;

  // locals 
  SmBucketItem<TYPE> * pBucketItem = m_sBuckets.GetAt(lBucketIndx) ;
  SM_ASSERT_MSG(   pBucketItem == NULL
                || pBucketItem->GetItem() != NULL,
                _T("SmHashTable::GetOneBucket_BucketItems: error found a BucketItem with a NULL sItem entry!")) ;  // gwc: does this make sense for nonPointer objects?

  // while there are BucketItems in the tgt bucket
  while(pBucketItem != NULL)
    { 
      // accumulate BucketItems
      rBucketItems.Add(pBucketItem) ;
      pBucketItem = pBucketItem->GetNextBucketItem() ;
      SM_ASSERT_MSG(   pBucketItem == NULL
                    || pBucketItem->GetItem() != NULL,
                    _T("SmHashTable::GetOneBucket_BucketItems: error found a BucketItem with a NULL sItem entry!")) ;  // gwc: does this make sense for nonPointer objects?
    }

  // all done
  return ;

} // end SmHashTable<TYPE>::GetOneBucket_BucketItems

/*******************************************************************//**
PURPOSE: Add a BucketItem to specified Bucket

NOTES:  Add one BucketItem for each time an sItem is added to a different bucket.
        This means that an sItem added to several buckets will be referenced
        by several different BucketItems.
        
        If you sort the BucketItems within a single bucket make
        sure to set the final 1st BucketItem->m_pNextUsedBucket ptr and
        to clear all the other BucketItem->m_pNextUsedBucket ptrs.

METHOD: Each time a bucketItem Obj is the first added to a bucket, that 
        BucketItem is added to the m_pNextUsedBucket linked list of used Bucket ptrs.

        Subsequent BucketItem additions are added to the 2nd position in the
        bucket's m_pNextBucketItem linked list of BucketItem objects so that the 1st
        bucketobj's pNextBucket value can stay constant.
***********************************************************************/
template<class TYPE>
inline void SmHashTable<TYPE>::Add
 (TYPE  sItem,             // in : Item to add to hash table
  ULONG lBucketIndex,      // in : Item Hash table BucketIndex. It's "hash" value.
  ULONG lOptItemId)        // in : Optional Id to store with Item (often used as a back index value to a caller managed array of Items)
                           //      SM_UNDEF_ULONG to ignore. default:[SM_UNDEF_ULONG]
{
  // check state - The target BucketIndex exists
  SM_ASSERT_BREAK(lBucketIndex < m_sBuckets.GetSize()) ;
  SM_ASSERT_BREAK(sItem != NULL) ;
                                             
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
SmBoolean bOk      = TRUE ; SM_REF1(bOk) ;
#endif // SM_DEBUG_CODE

  // when Items array gets full - inc item memory
  if(m_sBucketItems.GetSize() == m_sBucketItems.GetDataSize())
    { 
      // increase m_sBucketItems array's m_lMaxSize while preserving the first m_lSize m_sBucketItem values and fixing stale ptr values as needed.
      IncMaxItemCount() ; 
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { bOk &= AssertValid() ; 
          this->Dump() ; 
        }
#endif // SM_DEBUG_CODE
    }

  SM_ASSERT_MSG(m_sBucketItems.GetSize() < m_sBucketItems.GetDataSize(), _T("SmHashTable<TYPE>::Add(): memory management failed - adding a BucketItem to a full BucketItem array using SmTArray::Add will make stale pointers"))

  // construct new BucketItem 
  SmBucketItem<TYPE> sBucketItem(sItem, lBucketIndex, lOptItemId) ;

  // add new BucketItem to BucketItem list
  m_sBucketItems.Add(sBucketItem) ;

  // locals - new BucketItemRef
  SmBucketItem<TYPE> & rBucketItem = m_sBucketItems[GetTotalBucketItemCount()-1] ;

  // add new BucketItemPtr to HashTable buckets

  // when TgtBucket is empty - make new BucketItem 1st entry in SmHashTable::m_pUsedBuckets list and the 1st BucketItem in the Bucket
  if(m_sBuckets[lBucketIndex] == NULL)
    {
      // pre-insert this BucketItem Obj on the SmHashTable::UsedBucket list
      rBucketItem.SetNextUsedBucket(m_pUsedBuckets) ;
      m_pUsedBuckets = &rBucketItem ;

      // increment the used bucket count
      m_lUsedBucketCount++ ;

      // make HashObj 1st in its bucket
      rBucketItem.SetNextItem(NULL) ;
      m_sBuckets[lBucketIndex] = &rBucketItem ;
    }
  else // this is an addition to a used bucket - make New BucketItem the 2nd item in the Bucket
    {
      // place the new hashObj 2nd on the bucket linked-list so that the BucketItem->m_pNextUsedBucket ptrs stay valid
      rBucketItem.SetNextItem(m_sBuckets[lBucketIndex]->GetNextBucketItem()) ;
      m_sBuckets[lBucketIndex]->SetNextItem(&rBucketItem) ;
      rBucketItem.SetNextUsedBucket(NULL) ;   // rule: only the 1st BucketItem in a bucketlist stores a NextUsedBucket value.
    }

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      bOk &= AssertValid() ;
      this->Dump() ;
      SM_ASSERT_MSG(bOk == TRUE, _T("SmHashTable::Add - HashTable failed AssertValid() after Add()")) ;
    }
#endif // SM_DEBUG_CODE

} // end SmHashTable<TYPE>::Add

/*******************************************************************//**
PURPOSE: Remove a BucketItem either from specified Bucket (cheaper) or from entire table (costly)

NOTES: 1. RemovedBucketItems: Removed BucketItems are 
            o. removed from the m_sBucket link-list of bucket items
            o. not removed from the m_sBucketItems list so that all the 
               BucketItemPtr values stored within the m_sBuckets linked list
               of BucketItemPtr values don't become stale when a single
               BucketItem is removed.
       2. Removed BucketItems in the m_sBucketItems list are marked
            o. m_sBucketItems[lBucketItemIndx]->m_pNextUsedBucket = NULL ;
            o. m_sBucketItems[lBucketItemIndx]->m_pNextBucketItem = NULL ;
            o. m_sBucketItems[lBucketItemIndx]->m_lBucketIndex    = SM_UNDEF_ULONG ;
***********************************************************************/
template<class TYPE>
ULONG SmHashTable<TYPE>::Remove
 (SmTArray<ULONG> &rBucketItemIndices) // in : BucketItem indices to remove from m_sBucketTree 
                                       //      and mark as removed in the m_sBucketItems array 
{ 
  // locals
  ULONG ii ;
  ULONG lRemoveCount = 0 ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
SmBoolean bOk      = TRUE ; SM_REF1(bOk) ; 
  if(bDebugMe)
    { bOk &= AssertValid() ; 
      Dump() ; 
    }
#endif // SM_DEBUG_CODE

  // for every Tgt BucketItemIndx
  for(ii=0;ii<rBucketItemIndices.GetSize();ii++)
    {
      ULONG lBucketItemIndx = rBucketItemIndices[ii] ; 

      // locals
      SmBucketItem<TYPE> *  pBucketItem       = &m_sBucketItems[lBucketItemIndx] ;
      ULONG                 lBucketIndex      = pBucketItem->GetBucketIndex() ;

      // skip already-removed or corrupted bucket items
      if(lBucketIndex == SM_UNDEF_ULONG || lBucketIndex >= m_sBuckets.GetSize())
        { continue ; }

      SmBucketItem<TYPE> *  pBucketHead       = m_sBuckets[lBucketIndex] ;
      SmBucketItem<TYPE> ** ppPrev_UsedBucket = NULL ;
      SmBucketItem<TYPE> ** ppPrev_BucketItem = NULL ;
      
      // when BucketItem[lBucketItemIndx] is not the 1st member of its Bucket - find the Previous BucketItem pointing to this one
      if(pBucketHead != pBucketItem)
        {
          // linear search Bucket[lBucketIndx] for pBucketItem keeping track of the PrevBucketItem
          for(ppPrev_BucketItem = &pBucketHead;
              *ppPrev_BucketItem != NULL;
              ppPrev_BucketItem = &((*ppPrev_BucketItem)->GetNextBucketItemRef()) )
            {
              // when PrevBucketItem pts to This BucketItem
              if( (*ppPrev_BucketItem)->GetNextBucketItem() == pBucketItem)
                { 
                  // remove references to m_sBuckeItems[lBucketItemIndx] from Bucket Tree
                  (*ppPrev_BucketItem)->GetNextBucketItemRef() = pBucketItem->GetNextBucketItem() ;
                  break ; 
                }
            }
        } // end BucketItem is not Bucket's head item branch
      else // BucketItem is Bucket's 1st item branch
        {
          // when pBucketItem is the head of the BucketTree
          if(pBucketItem == m_pUsedBuckets)
            { 
              // when BucketItem is the only item in the bucket - remove Bucket from BucketTree
              if(pBucketItem->GetNextBucketItem() == NULL)
                {
                  m_pUsedBuckets = pBucketItem->GetNextUsedBucket() ;
                  m_sBuckets[lBucketIndex] = NULL ;
                  m_lUsedBucketCount-- ;
                }
              else // promote the 2nd BucketItem to 1st place
                {
                  pBucketItem->GetNextBucketItem()->GetNextUsedBucketRef() = pBucketItem->GetNextUsedBucket() ;
                  m_pUsedBuckets                                           = pBucketItem->GetNextBucketItem() ;
                  m_sBuckets[lBucketIndex] = pBucketItem->GetNextBucketItem() ;
                }
            } // end pBucketItem == m_pSuedBuckets branch

          else // linear search m_pUsedBuckets linked list for ppPrev_UsedBucket 
            {
              // The load-bearing test is *ppPrev_UsedBucket (the current bucket; NULL == end of list).
              // ppPrev_UsedBucket itself is the address of a link field and is never NULL on this walk,
              // but guard it too as a defensive belt-and-suspenders against a malformed list.
              for(ppPrev_UsedBucket = &m_pUsedBuckets;
                  ppPrev_UsedBucket != NULL && *ppPrev_UsedBucket != NULL;
                  ppPrev_UsedBucket = &((*ppPrev_UsedBucket)->GetNextUsedBucketRef()) )
                {
                  // when Prev_UsedBucketItem points to pBucketItem
                  if((*ppPrev_UsedBucket)->GetNextUsedBucket() == pBucketItem)
                    { 
                      // remove references to m_sBuckeItems[lBucketItemIndx] from Bucket Tree
                    
                      // when Bucket contains just one element - remove bucket from used bucket list - Used BucketTree loses one Bucket
                      if(pBucketItem->GetNextBucketItem() == NULL)
                        { 
                          (*ppPrev_UsedBucket)->GetNextUsedBucketRef() = pBucketItem->GetNextUsedBucket() ; 
                          m_sBuckets[lBucketIndex] = NULL ;
                          m_lUsedBucketCount-- ;
                        }
                      else // set Prev_UsedBucket to next item in the bucket - this promotes the 2nd bucket item to first place - Used BucketTree keeps all buckets
                        { 
                          pBucketItem->GetNextBucketItem()->GetNextUsedBucketRef() = pBucketItem->GetNextUsedBucket() ;
                          (*ppPrev_UsedBucket)->GetNextUsedBucketRef()             = pBucketItem->GetNextBucketItem() ; 
                          m_sBuckets[lBucketIndex] = pBucketItem->GetNextBucketItem() ;
                        } 
                      break ; 
                    } // end Perv UsedBucketItem points to pBucketItem check
                } // end UsedBuckets linear for pBucketItem
            } // end pBucketItem != m_pUsedBucket branch
        } // end BucketItem is Bucket's 1st item branch branch

      // arrive here: refs to m_sBucketItems[lBucketItemIndx] have been removed from the Bucket tree
      // next: update BucketItem and Hashtable state to remember removing the BucketItem
      
      // count removed items
      m_lRemovedBucketItemCount++ ;
      lRemoveCount++ ;
      
      // set m_sBucketItems[lBucketItemIndx] values to be a 'removed BucketItem'
      pBucketItem->GetNextUsedBucketRef() = NULL ;
      pBucketItem->GetNextBucketItemRef() = NULL ;
      pBucketItem->GetBucketIndexRef()    = SM_UNDEF_ULONG ;
      // leave  m_sBucketItems[lBucketItemIndx]->m_sItem    val as is
      //        m_sBucketItems[lBucketItemIndx]->m_sItemId  val as is

    } // end iter every rBucketItemIndices indx value

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      bOk &= AssertValid() ; 
      this->Dump() ;
      SM_ASSERT_MSG(bOk == TRUE, _T("SmHashTable::Add - HashTable failed AssertValid() after Add()")) ;
    }
#endif // SM_DEBUG_CODE

  // all done - return number of removed BucketItems
  return lRemoveCount ;

} // end SmHashTable<TYPE>::Remove

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
// see definition of
// SmAssertReportLabel sAssertHashTable_list[] =
// in file: SmHashTable.cpp

/*******************************************************************//**
PURPOSE: Check SmHashTable for problems

NOTES:
***********************************************************************/
template<class TYPE>
inline SmBoolean SmHashTable<TYPE>::AssertValid
 (SmAssertArray    * pAList,         // [i/o]: Accumulating list of failed Asserts, NULL to ignore                                             <br>
  SmAssertTestLevel  eTestLevel,     // [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         <br>
                                     //      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     <br>
                                     //      : default:[SM_LEVEL_0]                                                                               <br>
  SmAssertWalking    eWalkTree,      // [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   <br>
  SmTArray<ULONG>  * pTestRequests)  // NotUsed: [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   <br>
 const 
{
  SM_REF1(eWalkTree) ;
  SM_REF1(pTestRequests) ;

  // locals and return
  ULONG ii, lUsedBucketItemCount = 0 ;
  SmBoolean bRtn = TRUE ; 

  // for every Item - check Item ptr
  for(ii=0;ii<m_sBucketItems.GetSize();ii++)
    {
      // skip removed BucketItems — their bucket index is SM_UNDEF_ULONG
      if(m_sBucketItems[ii].IsRemovedBucketItem()) { continue ; }

      lUsedBucketItemCount++ ;

      /* 0 */ // check BucketItem Index points to a used Bucket
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (TRUE == IsUsedBucket(m_sBucketItems[ii].GetBucketIndex())), _T("")) ;
    }

  /* 1 */ // check Total BucketItem Count >= Removed BucketItem count
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (GetTotalBucketItemCount() >= GetRemovedBucketItemCount()), _T("")) ;

  /* 2 */ // check Total == Used + Removed BucketItem counts 
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (GetTotalBucketItemCount() == GetUsedBucketItemCount() + GetRemovedBucketItemCount()), _T("")) ;

  /* 3 */ // check make sure counted-used and the stored used(total - removed) BucketItem counts match
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (lUsedBucketItemCount == GetUsedBucketItemCount()), _T("")) ;

  // all done
  return(bRtn) ;
      
} // end SmHashTable<TYPE>::AssertValid

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmHashTable<TYPE>::Dump() const
{
  // pass the call along - don't display the bucket item contents
  Dump(FALSE) ;

} // end SmHashTable<TYPE>::Dump - no args

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmHashTable<TYPE>::Dump(SmBoolean bFull) const
{
  ULONG ii, jj ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin Header 
  smos_sprintf(sBuff,        _T("\nBegin SmHashTable:[0x%p] Dump"), this) ; 
  smos_sprintf(sBuffForFile, _T("\nBegin SmHashTable:[%s] Dump"), _T("notNULL")) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // counts buckets
  smos_sprintf(sBuff,        _T("\n  Bucket     Cnts:  Alloc:[%5lu],  Used  :[%5lu]"), 
                             m_sBuckets.GetSize(), 
                             m_lUsedBucketCount) ; 
  smos_WriteBuffer(sBuff) ;                        
                                                   
  // counts items                   
  smos_sprintf(sBuff,        _T("\n  BucketItem Cnts:  Used:[%5lu], Removed[%5lu], Total-BucketItems[%5lu], Alloc:[%5lu]"), 
                             GetUsedBucketItemCount(),
                             m_lRemovedBucketItemCount,
                             m_sBucketItems.GetSize(), 
                             m_sBucketItems.GetDataSize()) ; 
  smos_WriteBuffer(sBuff) ;                        
                                                   
  // used bucket item counts
  ULONG lColCnt = m_lUsedBucketCount <= 12 ? 1 : 4 ;
  SmBucketItem<TYPE> * pUsedBucket = m_pUsedBuckets ;

  // for every used bucket - pretty print and gather some statistics
  ULONG  lTotalItemCnt       = 0 ;
  ULONG  lOneItemBucketCnt   = 0 ;
  ULONG  lMultiItemBucketCnt = 0 ;
  ULONG  lMultiItemCnt       = 0 ;
  ULONG  lSortedPairCnt      = 0 ;
  ULONG  lNaivePairCnt       = m_sBucketItems.GetSize() > 0 ? (GetUsedBucketItemCount()*(GetUsedBucketItemCount()-1)/2) : 0 ;

  // for every used bucket
  for(ii=0;ii<m_lUsedBucketCount;)
    {
      smos_WriteBuffer(_T("\n    ")) ; 

      // in groups of lColCnt
      for(jj=0;
          jj<lColCnt && ii<m_lUsedBucketCount;
          jj++,ii++,pUsedBucket=pUsedBucket->GetNextUsedBucket())
        {
          // counts m_lUsedBucketCount 
          ULONG lBucketSize = GetOneBucketItemCount(pUsedBucket->GetBucketIndex()) ;

          // gather statistics
          if(lBucketSize == 1) { lOneItemBucketCnt++ ; }
          else                 { lMultiItemBucketCnt++ ; 
                                 lMultiItemCnt  += lBucketSize ;
                                 lSortedPairCnt += (lBucketSize * (lBucketSize-1))/2 ; // number of pairs in bucket
                                }
          lTotalItemCnt += lBucketSize ;

          // pretty print
          smos_sprintf(sBuff,        _T("BucketIndex:[%5lu] ItemCnt:[%5lu]%s"), 
                           pUsedBucket->GetBucketIndex(), 
                           lBucketSize,
                           jj<lColCnt-1 ? _T(",  ") : _T("")) ;
          smos_WriteBuffer(sBuff) ;
        } // end iter cols of this row
    } // end iter rows

  // pretty print - statistics
  smos_WriteBuffer(_T("\n BucketSort statistics")) ;
  smos_sprintf(sBuff, _T("\n  Sorted TotalItems Cnt:[%5lu],  CheckCnt:[%5lu]"), GetUsedBucketItemCount(), lTotalItemCnt) ; smos_WriteBuffer(sBuff) ; 
  smos_sprintf(sBuff, _T("\n  With-Items Bucket Cnt:[%5lu]"), m_lUsedBucketCount) ; smos_WriteBuffer(sBuff) ; 
  smos_sprintf(sBuff, _T("\n  One-Item   Bucket Cnt:[%5lu]"), lOneItemBucketCnt) ; smos_WriteBuffer(sBuff) ; 
  smos_sprintf(sBuff, _T("\n  Multi-Item Bucket Cnt:[%5lu]"), lMultiItemBucketCnt) ; smos_WriteBuffer(sBuff) ; 
  if ( lMultiItemBucketCnt > 0 )
    { smos_sprintf(sBuff, _T("\n  Multi-Item Item   Cnt:[%5lu], Avg Item per Multi-item Bucket Cnt:[%lf]"), lMultiItemCnt, (double)lMultiItemCnt/(double)lMultiItemBucketCnt ) ; smos_WriteBuffer(sBuff) ; }
  smos_sprintf(sBuff, _T("\n  Sorted-Pair       Cnt:[%5lu], Naive-Pair Cnt(N*(N-1)/2):[%5lu], SortedPair/NaivePair Ratio:[%lf]"), 
                    lSortedPairCnt,
                    lNaivePairCnt, 
                    (lNaivePairCnt > 0) ? (double)lSortedPairCnt/(double)(lNaivePairCnt) : 0.0) ; smos_WriteBuffer(sBuff) ; 
                    
  // when asked for full dump
  if(bFull && lTotalItemCnt > 0)
    {
      pUsedBucket = ((SmHashTable<TYPE>*)this)->GetUsedBucket(0) ;

      for(ii=0;ii<GetUsedBucketCount();ii++,pUsedBucket = pUsedBucket->GetNextUsedBucket())
        {
          ULONG               lBucketIndex = pUsedBucket->GetBucketIndex() ;
          ULONG               lBucketSize  = GetOneBucketItemCount(lBucketIndex) ;
          SmBucketItem<TYPE> * pBucketItem   = pUsedBucket ;

          smos_sprintf(sBuff,        _T("\n  UsedBucket[%5lu]: index:[%5lu], sized:[%5lu]"),ii,lBucketIndex,lBucketSize) ;
          smos_WriteBuffer(sBuff) ;

          // for every item
          for(jj=0;jj<lBucketSize;jj++,pBucketItem = pBucketItem->GetNextBucketItem())
            {
              pBucketItem->Dump(jj, 4) ;
            } // end iter every bucket item
        } //end iter every bucket

      // pretty print - statistics - again
      smos_WriteBuffer(_T("\n BucketSort statistics")) ;
      smos_sprintf(sBuff, _T("\n  Sorted TotalItems Cnt:[%5lu],  CheckCnt:[%5lu]"), GetUsedBucketItemCount(), lTotalItemCnt) ; smos_WriteBuffer(sBuff) ; 
      smos_sprintf(sBuff, _T("\n  Used       Bucket Cnt:[%5lu]"), m_lUsedBucketCount) ; smos_WriteBuffer(sBuff) ; 
      smos_sprintf(sBuff, _T("\n  One-Item   Bucket Cnt:[%5lu]"), lOneItemBucketCnt) ; smos_WriteBuffer(sBuff) ; 
      smos_sprintf(sBuff, _T("\n  Multi-Item Bucket Cnt:[%5lu]"), lMultiItemBucketCnt) ; smos_WriteBuffer(sBuff) ; 
      if ( lMultiItemBucketCnt > 0 )
        { smos_sprintf(sBuff, _T("\n  Multi-Item Item   Cnt:[%5lu], Avg Item per Multi-item Bucket Cnt:[%lf]"), lMultiItemCnt, (double)lMultiItemCnt/(double)lMultiItemBucketCnt ) ; smos_WriteBuffer(sBuff) ; }
      smos_sprintf(sBuff, _T("\n  Sorted Item-Pair  Cnt:[%5lu], Naive N*(N-1)/2 Cnt:[%5lu], SortedPair/NaivePair Ratio:[%lf]"), 
                        lSortedPairCnt,
                        lNaivePairCnt, 
                        (lNaivePairCnt > 0) ? (double)lSortedPairCnt/(double)(lNaivePairCnt) : 0.0) ; smos_WriteBuffer(sBuff) ; 
    } // end full dump check

  // End Footer 
  smos_sprintf(sBuff,        _T("\nEnd SmHashTable:[0x%p] Dump"), this) ; 
  smos_sprintf(sBuffForFile, _T("\nEnd SmHashTable:[%s] Dump"), _T("notNULL")) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;
  
} // end SmHashTable<TYPE>::Dump

/*******************************************************************//**
PURPOSE: Pretty print list of specified Buckets

NOTES: Prints all the items in each TgtBucket Index
***********************************************************************/
template<class TYPE>
inline void SmHashTable<TYPE>::DumpBuckets
 (SmTArray<ULONG> &rBucketIndices)  // in : Array of BucketIndices to pretty print
 const  
{
  ULONG ii, jj ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin Header 
  smos_sprintf(sBuff,        _T("\nBegin Bucket IndexList Dump for SmHashTable:[0x%p]"), this) ; 
  smos_sprintf(sBuffForFile, _T("\nBegin Bucket IndexList Dump for SmHashTable:[%s]"), _T("notNULL")) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // Tgt Bucket index list
  smos_WriteBuffer(_T("\n  Dumping Buckets: [")) ;
  for(ii=0;ii<rBucketIndices.GetSize();ii++)
    {
      if     (rBucketIndices.GetAt(ii) < 10)   { smos_sprintf(sBuff, _T("%2lu"), rBucketIndices.GetAt(ii)) ; }
      else if(rBucketIndices.GetAt(ii) < 100)  { smos_sprintf(sBuff, _T("%3lu"), rBucketIndices.GetAt(ii)) ; }
      else if(rBucketIndices.GetAt(ii) < 1000) { smos_sprintf(sBuff, _T("%4lu"), rBucketIndices.GetAt(ii)) ; }
      else                                     { smos_sprintf(sBuff, _T("%5lu"), rBucketIndices.GetAt(ii)) ; }
      smos_WriteBuffer(sBuff) ;
      smos_sprintf(sBuff, _T("%s"), (ii==rBucketIndices.GetSize()-1) ? _T("]")  :_T(",")) ;
      smos_WriteBuffer(sBuff) ;
    }

  // for every Tgt Bucket - dump its contents
  for(ii=0;ii<rBucketIndices.GetSize();ii++)
    {
      ULONG                lBucketIndex = rBucketIndices.GetAt(ii) ;
      ULONG                lBucketSize  = GetOneBucketItemCount(lBucketIndex) ;
      SmBucketItem<TYPE> * pBucketItem  = ((SmHashTable<TYPE> *)this)->GetBucket(lBucketIndex) ;
  
      smos_sprintf(sBuff,        _T("\n  TgtBucket[%5lu]: index:[%5lu], sized:[%5lu]"),ii,lBucketIndex,lBucketSize) ;
      smos_WriteBuffer(sBuff) ;
  
      // for every item
      for(jj=0;jj<lBucketSize;jj++,pBucketItem = pBucketItem->GetNextBucketItem())
        {
          pBucketItem->Dump(jj, 4) ;
        } // end iter every bucket item
    } //end iter every BucketIndx

  // End Footer 
  smos_sprintf(sBuff,        _T("\nEnd Bucket IndexList Dump for SmHashTable:[0x%p]"), this) ; 
  smos_sprintf(sBuffForFile, _T("\nEnd Bucket IndexList Dump for SmHashTable:[%s]"), _T("notNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;
  
} // end SmHashTable<TYPE>::DumpBuckets for array of BucketIndices

SM_EXPORT_TEMPLATE template class SM_EXPORT SmHashTable<SmVertex*> ;
SM_EXPORT_TEMPLATE template class SM_EXPORT SmHashTable<SmVertexProps*> ;
SM_EXPORT_TEMPLATE template class SM_EXPORT SmHashTable<SmPointItem<SmVertex*>*> ;
SM_EXPORT_TEMPLATE template class SM_EXPORT SmHashTable<SmPointItem<SmVertexProps*>*> ;

SM_TARRAY_TEMPLATE_PREDECLARATION(SmHashTable<SmVertex*>);
SM_TARRAY_TEMPLATE_PREDECLARATION(SmHashTable<SmVertex*>*);
SM_TARRAY_TEMPLATE_PREDECLARATION(SmHashTable<SmVertexProps*>);
SM_TARRAY_TEMPLATE_PREDECLARATION(SmHashTable<SmVertexProps*>*);

/*******************************************************************//**
PURPOSE: Use a Voxel model to quickly find pairs of objects that share
         coincident associated Point3d positions.

NOTES: 1. The SmObjsInVoxels obj has methods:
            1a. constructor SmObjsInVoxels() and SetSize(): define the partitioning and extent of a voxel model
            1b. Add(): - place objs into voxels based on their associated Point3d value.
                       - also place objs into neighbor voxels when objs are within Tol of a voxel boundary.
                   So, one Obj may be placed into multiple voxels depending on position and tolerance.
            1c. FindCoincidentPairs(): Search all voxels containing 2 or more objs for coincident obj pairs.
            1d. Remove(): - remove TgtItem from all voxels in the voxel model
                          - this is implemented with a slow linear search - try to avoid using Remove().
            1e. Dump(), Draw(), AssertValid(): for debug purposes

       2. SmObjsInVoxels memory management assumes the classes
            SmPointItems<TYPE>, 
            SmBucketItems<SmPointItems<TYPE>*> and
            TYPE
           do not have virtual functions.  It's ok if TYPE is a pointer to a Type that does have virtual functions.
***********************************************************************/
template<class TYPE> class SM_EXPORT SmObjsInVoxels 
{
  protected:
    SmHashTable<SmPointItem<TYPE>*> m_sHashTable ;   // HashTable: One Bucket  == One Voxel 
    //                                               //            BucketIndex == VoxelIndex = ZIndex * (XCnt*YCnt) + YIndx * (XCnt) + Zindx
    //                                               //            SmTArray<SmBucektItem<TYPE> *> m_sHashTable<TYPE>.m_sBuckets     = BucketItem ptr arrays forming the buckets
    // contains                                      //            SmTArray<SmBucketItem<TYPE>>   m_sHashTable<TYPE>.m_sBucketItems = BucketItem array of thing pointed to by the bucket ptr arrays
    // SmTArray< SmBucketItem<SmPointItem<TYPE>*> *> m_sHashTable.m_sBuckets ;     // Array of buckets; bucket = linked BucketItem list, sized:[lBucketCount] in the constructor.
    //                                                                             // Stores the pointer to the 1st SmBucketItem in the bucket, else NULL
    // SmTArray< SmBucketItem<SmPointItem<TYPE>*> >  m_sHashTable.m_sBucketItems ; // One Item may appear in many bucketItems as requested by the user
    //                                                                             // SmBucketItem contains: SmBucketItem * SmBucketItem::m_pNextUsedBucket   // linked list pointer to next used bucket in the HashTable 
    //                                                                             //                                                                         //  only the 1st bucketItem in the bucketList uses this pointer.
    //                                                                             //                        SmBucketItem * SmBucketItem::m_pNextBucketItem   // linked list pointer to next HashObj placed in this hash bucket
    //                                                                             //                        void         * SmBucketItem::m_sItem             // pointer to this Hash object   
    //                                                                             //                        ULONG          SmBucketItem::m_lBucketIndex      // bucket index for this entry
    //                                                                             //                        ULONG          SmBucketItem::m_lItemIndx         // &m_sPointItems[indx] == m_sHashTable.m_BucketItems[indx]
    // ULONG                          m_sHashTable.m_lRemovedBucketItemCount = 0 ; // number of removed bucketItems left behind in the m_sBucketItems list as unused 'deleted' objects
    //                                                                             // UsedBucketItemCount + RemovedBucketItemCount = m_sBucketItems.GetSize()
    // SmBucketItem<TYPE> *           m_sHashTable.m_pUsedBuckets = NULL ;         // link list of 1st BucketItem in each used Bucket
    // ULONG                          m_sHashTable.m_lUsedBucketCount = 0 ;        // number of used buckets (length of m_pUsedBuckets linked list)

    ULONG                            m_lXCnt = SM_UNDEF_ULONG ; // number of X buckets in the m_sHashTable.m_sBuckets bucket set
    ULONG                            m_lYCnt = SM_UNDEF_ULONG ; // number of Y buckets in the m_sHashTable.m_sBuckets bucket set
    ULONG                            m_lZCnt = SM_UNDEF_ULONG ; // number of Z buckets in the m_sHashTable.m_sBuckets bucket set
    SmExtent3d                       m_sBBox ;                  // Bounding box of all voxels - voxel's origin and range
    
    ULONG                            m_lRemovedPointItemCount=0;// number of removed PointItems left behind in the m_sPointItems list as unused 'deleted' objects
                                                                //   1. m_lRemovedPointItemCount == m_sHashTable.m_lRemovedBucketItemCount
                                                                //   2. used PointItems count = m_sPointItems - m_lRemovedPointItemCount
                                                                //   3. For removed PointItem: m_sPointItems[ii].m_lBucketIndex    == SM_UNDEF_ULONG
                                                                //                             m_sPointItems[ii].m_pNextUsedBucket == NULL
                                                                //                             m_sPointItems[ii].m_pNextBucketItem == NULL
    SmTArray<SmPointItem<TYPE>>      m_sPointItems ;            // the items placed into voxels: {sItem, pt, tol}
                                                                // The m_sHashTable.m_sBucketItems ptr array maps one-to-one with the SmObjsInVoxels.m_sPointItems array so
                                                                //   m_sHashTable.m_sBucketItems[ii].GetSize() == SmObjsInVoxels.m_sPointItems[ii].GetSize() 
                                                                //   m_sHashTable.m_sBucketItems[ii]           == &SmObjsInVoxels.m_sPointItems[ii] and
                                                                // notes: m_sPointItems[ii].m_sTgtItem                    = Item being hashed into voxels
                                                                //        m_sPointItems[ii].m_sTgtPoint3d                 = Item's Point3d position 
                                                                //        m_sPointItems[ii].m_sTgtZoneTol3d               = Item's ZoneTol3d size
                                                                //        m_sHashTable.m_sBucketItems[ii]->m_sItem        = ptrs to this m_sPointItems list members
                                                                //        m_sHashTable.m_sBucketItems[ii]->m_lItemId      = associated m_sPointItems indx value
                                                                //        m_sHashTable.m_sBucketItems[ii]->m_lBucketIndex = VoxelIndex = ZIndex * (XCnt*YCnt) + YIndx * (XCnt) + Zindx
 public:                  
  // Constructor                                 
  SmObjsInVoxels(ULONG        lXCnt = 0,                        // in : Number of Voxels in X dir
                 ULONG        lYCnt = 0,                        // in : Number of Voxels in Y dir
                 ULONG        lZCnt = 0,                        // in : Number of Voxels in Z dir
                 SmExtent3d * pBBox = NULL,                     // in : Voxel Extent
                 ULONG        lItemEstimate = 0)                // in : estimated number of items to be placed in the buckets
                                                                : m_sHashTable(lItemEstimate, lXCnt * lYCnt * lZCnt)
                                                                {
                                                                  // set up memory
                                                                  SetSize(lXCnt, lYCnt, lZCnt, pBBox, lItemEstimate) ;
                                                                }
                                                
  // empty bucket contents and set buckets and item counts as specified
  inline void SetSize(ULONG        lXCnt,              // in : Number of Voxels in X dir  
                      ULONG        lYCnt,              // in : Number of Voxels in Y dir  (for 1d voxels lYCnt & lZcnt set to 1)
                      ULONG        lZCnt,              // in : Number of Voxels in Z dir  (for 2d voxels lZcnt set to 1)
                      SmExtent3d * pBBox,              // in : Voxel Extent (for 1d voxels set YLength = ZLength = 0.0, for 2d voxels set ZLength = 0.0)
                      ULONG        lItemEstimate) ;    // in : estimated number of items to be placed in the buckets

  // increase m_sPointItems and m_sHashTable.m_sBucketItems array m_lMaxSize vals - preserve first m_lSize elem vals and fix stale ptr vals.
  inline void IncMaxItemCount() ;                      

  // assignment operator
  inline SmObjsInVoxels<TYPE> &operator= (SmObjsInVoxels<TYPE> const & crOther) ; 

  // equality operator
  SmBoolean operator==(const SmObjsInVoxels<TYPE> &crOther) const ;

  // destructor
  ~SmObjsInVoxels()  { SetSize(0,0,0,NULL,0) ; }

  // simple data access
  inline SmTArray<SmPointItem<TYPE>> & GetPointItems    ()                            { return m_sPointItems ; }
  inline SmPointItem<TYPE>           * GetPointItem     (ULONG lIndx)                 { return & m_sPointItems.GetAt(lIndx) ; }
  inline ULONG                         GetSize          ()                    const   { return m_sPointItems.GetSize() ; }
  inline void                          FindPointItems   (TYPE    sTgtItem,                       
                                                         SmTArray<ULONG> &rFoundIndices) const ; // out:the Indices of every PointItem referencing sTgtItem
  ULONG                                MapIndicesToVoxelIndex(ULONG lIndxX, ULONG lIndxY, ULONG lIndxZ) const
                                                             { return(lIndxZ * (m_lXCnt*m_lYCnt) + lIndxY * (m_lXCnt) + lIndxX ) ; } 
  void                                 MapVoxelIndexToIndices(ULONG lVoxelIndex, ULONG &rIndxX, ULONG &rIndxY, ULONG &rIndxZ) const
                                                             { rIndxZ = (lVoxelIndex) / (m_lXCnt*m_lYCnt) ;
                                                               rIndxY = (lVoxelIndex - rIndxZ * (m_lXCnt*m_lYCnt)) / (m_lXCnt) ;
                                                               rIndxX = (lVoxelIndex - rIndxZ * (m_lXCnt*m_lYCnt) - rIndxY * (m_lXCnt)) ;
                                                             }
  inline ULONG                         MapPointToVoxel(SmPoint3d  & rPoint3d,                    // rtn: Indx of voxel containing sPoint3d
                                                       SmExtent3d * pOptFoundVoxelExtent3d=NULL, // out: optional containing voxel's Extent3d
                                                       ULONG      * pOptIndxX=NULL,              // out: Optional IndxX of found voxel tupleIndx:[IndxX IndxY IndxZ]
                                                       ULONG      * pOptIndxY=NULL,              // out: Optional IndxY of found voxel tupleIndx:[IndxX IndxY IndxZ]
                                                       ULONG      * pOptIndxZ=NULL,              // out: Optional IndxZ of found voxel tupleIndx:[IndxX IndxY IndxZ]
                                                       SmBrep     * pOptBrep=NULL) const ;       // in : for debug draw only, NULL to ignore, default:[NULL]
  void                                 MapVoxelIndexToBBox(ULONG        lVoxelIndex,                  // in : Tgt Voxel Index
                                                           SmExtent3d & rFoundVoxelExtent3d) const ;  // out: TgtVoxel BoundingBox
  // Add an item with its Point and tol data to the voxel hashTable - this is fast - return found voxel->Indx
  inline ULONG Add                                                // rtn: VoxelIndex = BucketIndx = ZIndex * (XCnt*YCnt) + YIndx * (XCnt) + Zindx
                   (TYPE              sItem,                      // in : TgtItem being added to the hash table
                    SmPoint3d       & rPoint3d,                   // in : TgtItem's Space3d location
                    SmZoneTol3d       sZoneTol3d,                 // in : TgtItem's ZoneTol3d about the TgtItem's sPoint3d
                    SmTArray<ULONG> * pOptUpdatedBuckets=NULL,    // i/o: NotNULL = accumulate list of Bucket Indices to which Items have been added
                                                                  //      NULL to ignore, default:[NULL]
                                                                  //      note: Add assigns ItemIndx vals to PointItems in the order they are 
                                                                  //            added to the Voxels.  
                                                                  //      note: building the pUbdatedBuckets list uses a slow AddUnique() call,
                                                                  //            only use this when adding a few items to already existing large list.
                                                                  //            Don't use when building a large list from scratch 
                                                                  //             - the m_sHashTable.GetUsedBucketCount() and GetUsedBucket(ii) will
                                                                  //               give you the used bucket list in that case.
                    SmBrep          * pOptBrep=NULL) ;            // in : for debug draw only, NULL to ignore, default:[NULL]

  // Remove all table entries for item from bucket - this is linear search slow - minimize using this method
  inline ULONG Remove(TYPE sItem) ;

  // Find Coincident PointItem pairs (Coincident pairs near boundaries may be listed multiple times)
  void FindCoincidentPairs(SmTArray<SmPointItem<TYPE> *> & rCoin1,     // out: array of PointItem1s for each CoincidentPair:[Pair1_PointItem1, Pair2_PointItem1,.. , PairN_PointItem1]
                           SmTArray<SmPointItem<TYPE> *> & rCoin2,     // out: array of PointItem2s for each CoincidentPair:[Pair1_PointItem2, Pair2_PointItem2,.. , PairN_PointItem2]
                                                                       //        note: while a PointItem might show up many times in each list,
                                                                       //              Each combined [PointItem1, PointItem2] coin pair is unique
                           ULONG           * pOptLimitIndx=NULL,       // in : NotNULL, Chk PointItems with Indxs >= *pOptLimitIndx against all PointItems for coin,
                                                                       //      NULL,    chk all PointItems against all PointItems for coin,
                                                                       //      default:[NULL]
                                                                       //      note: Add assigns ItemIndx vals to PointItems in the order they are 
                                                                       //            added to the Voxels.  Using pOptLimitIndx limits the search for 
                                                                       //            coincident points to the last PointItems added to the voxel 
                                                                       //            starting with the PointItem whose index == *pOptLimitIndx.
                           SmTArray<ULONG> * pOptBucketIndices  =NULL, // in : NotNULL = List of buckets to be checked for coincident pairs
                                                                       //      NULL    = Check all buckets, default:[NULL]
                                                                       //      note: When adding a few Items to the voxels and then checking for coincidence
                                                                       //            set pOptStartIndx = SizeOf m_sPointItems before adding new Items
                                                                       //            let pOptBucketIndices   = Add's pOptUpdatedBuckets argument
                           SmTArray<ULONG> * pOptHistogram=NULL) ;     // out: Optional Histogram of PtObj/PtObj gap sizes

  // return TRUE when object is tested as valid
  SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                                             
                        SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                      
                                                                  //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                  
                                                                  //    : default:[SM_LEVEL_0]                                                                            
                        SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
                        SmTArray<ULONG>  * pTestRequests=NULL     //NotUsed: in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]        
                      )  const ;
  
  // Define  GetType(), IsKindOf(), and Dump() (Dump needs local implementation)
  SM_COMMON_ONLY_BASE(SmObjsInVoxels,SmObjsInVoxels_TYPE);

  // pretty print
  void Dump(ULONG lLabel, SmBoolean bFull) const ;                 // in : TRUE = display the bucket item contents. FALSE=don't
  void DumpTgtBucketItems(SmTArray<SmBucketItem<SmPointItem<TYPE>*>*> &rBucketItems) const ; // in : PrettyPrint list of BucketItems from m_sHashTable
  void DumpCoincidentPairs(SmTArray<SmPointItem<TYPE> *> & rCoin1, // in : PointItem1 of CoincidentPair:[PointItem1, PointItem2]
                           SmTArray<SmPointItem<TYPE> *> & rCoin2) // in : PointItem2 of CoincidentPair:[PointItem1, PointItem2]
                          const ;                                  
  SmDisplayList * Draw(SmGfxArraySet * pOptGfxSet=NULL) const ;    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                                   //      NULL to ignore. default:[NULL]   

} ; // end class SmObjsInVoxels

/*******************************************************************//**
PURPOSE: empty bucket contents and set buckets and item counts as specified 

NOTES: method first calls base class method to manage inherited
       memory and then does the same for the m_pPointItems array
***********************************************************************/
template<class TYPE>
inline void SmObjsInVoxels<TYPE>::SetSize
 (ULONG        lXCnt,             // in : Number of Voxels in X dir
  ULONG        lYCnt,             // in : Number of Voxels in Y dir
  ULONG        lZCnt,             // in : Number of Voxels in Z dir
  SmExtent3d * pBBox,             // in : Voxel Extent
  ULONG        lItemEstimate)     // in : estimated number of items to be placed in the buckets
{
  // init voxel description 
  m_lXCnt = lXCnt ;
  m_lYCnt = lYCnt ;
  m_lZCnt = lZCnt ;

  // init BBox
  if(pBBox) { m_sBBox = *pBBox ; }
  else      { m_sBBox.Init() ; }

  // init hash table memory
  m_sHashTable.InitSize(lItemEstimate, lXCnt * lYCnt * lZCnt) ;  // sets m_sBuckets:[Size=lItemEstimate,MaxSize>Size], m_sBucketItems:[lSize=0,MaxSize>0]

  // clear out old m_sPointItems data
  m_sPointItems.RemoveAll() ; // m_lSize and m_lMaxSize are set to 0, memory freed

  // when asked - make room for new m_sPointItems data
  if(lItemEstimate > 0)
    {
      // set m_sPointItems.MaxSize == m_sHashTable.m_sBucketItems.GetMaxSize()
      ULONG lMaxSize = m_sHashTable.GetBucketItems().GetDataSize() ;

      // set m_sPointItems data size
      m_sPointItems.SetDataSize(lMaxSize) ;
      // if m_sBucketItems.SetSize(0) is run here - then the memory for m_pData is freed - don't do it

    }  // end nonZero item count check

} // end SmObjsInVoxels::SetSize

/*******************************************************************//**
PURPOSE: increase m_sPointItems and m_sHashTable.m_sBucketItems array m_lMaxSize vals 
          - preserve first m_lSize elem vals and fix stale ptr vals.

NOTES: When m_sPointItems increases in size beyond the arrays->m_lMaxSize
       the old array is replaced by a new larger data array whose
       member values are copied from the values of the old array.
       That means all the pointers stored in m_sBuckets and in the
       m_sBucketItems[ii] members are now stale and have to be updated.

       The m_sPointItems and m_sHashTable.m_sBucketItems are associated.
       the m_sHashTable.m_sBucketItems.m_lMaxSize is alos updated to be
       the same size as the expanded m_sPointItems.m_lMaxSize.
***********************************************************************/
template<class TYPE>
inline void SmObjsInVoxels<TYPE>::IncMaxItemCount() 
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
SmBoolean bOk      = TRUE ; SM_REF1(bOk) ; 
  if(bDebugMe)
    { 
      bOk &= AssertValid() ; 
      Dump() ; 
    }
#endif // SM_DEBUG_CODE

  // increment the m_sHahTable size
  m_sHashTable.IncMaxItemCount() ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      bOk &= m_sHashTable.AssertValid() ; 
      m_sHashTable.Dump() ; 
    }
#endif // SM_DEBUG_CODE

  // New m_lMaxSize
  ULONG lNewMaxSize = m_sHashTable.GetBucketItems().GetDataSize() ;

  // next: increment m_PointItems Array size - point m_sHashTable.m_sPointItems.m_pItem ptrs to the new memory

  // m_sPointItems current and larger new memory blocks
  SmPointItem<TYPE> * pOldPointItemData = m_sPointItems.GetDataArray() ;
  SmPointItem<TYPE> * pNewPointItemData = (SmPointItem<TYPE> *)smos_Calloc(1, lNewMaxSize * sizeof(SmPointItem<TYPE>)) ;

  // initialize the new elements with '0' characters -
  // WARNING: when TYPE has virtual functions, this call corrupts the virtual function table: It's ok when TYPE is a pointer to a type with virtual functions
  smos_MemSet((void*)(pNewPointItemData), 0, lNewMaxSize * sizeof(SmPointItem<TYPE>)) ;

  // when the OldHashTable had PointItems - copy old PointItems into New Array
  if(m_sPointItems.GetSize() > 0)
    {
      // copy the bucket structure from old to new - only okay to use smos_MemCpy because SmBucketItems have no virtual methods
      SE(smos_MemCpy((void*)pNewPointItemData,                              // out: ptr to mem array to copy to 
                     (void*)pOldPointItemData,                              // in : ptr to mem array to copy from
                     m_sPointItems.GetSize() * sizeof(SmPointItem<TYPE>),   // in : size of src,  (typedef unsigned __int64 size_t;)
                     lNewMaxSize             * sizeof(SmPointItem<TYPE>))); // in : size of dest, (typedef unsigned __int64 size_t;)
      
      // Update all the m_sHashTable::m_sBucketItems::m_sItem pointers
      // now fix the m_pNextBucketItem and m_pNextUsedBucket pointers - use pointer math: get index of nextItem, store that index address from the new array
      for(ULONG ii=0;ii<m_sHashTable.GetTotalBucketItemCount();ii++)
        {
          SmBucketItem<SmPointItem<TYPE>*>* pBucketItem = &m_sHashTable.GetBucketItem(ii) ;

          // when m_sHashTable.BucketItem.m_pItem ptrs are NonNULL
          if(   pBucketItem->GetItem()   != NULL
             && pBucketItem->GetItemId() != SM_UNDEF_ULONG)
            {
              // set the pointer value for the new array
              pBucketItem->SetItem(&m_sPointItems.GetAt(pBucketItem->GetItemId())) ;
            } // end NextItem NotNULL check
        } // end iter every BucketItem

    } // end OldHashTable had PointItems check

  // save the newDataArray while freeing the old
  m_sPointItems.SetArray(lNewMaxSize,               // new m_lMaxSize
                         pNewPointItemData,         // new array
                         m_sPointItems.GetSize()) ; // new m_lSize, frees the old m_pData array

  // tell the array it owns its own data array - mem management
  m_sPointItems.SetIsBorrowed(FALSE) ;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    { 
      bOk &= AssertValid() ;
      Dump() ; 
    }
#endif // SM_DEBUG_CODE

} // end SmObjsInVoxels::IncItemCount

/*******************************************************************//**
PURPOSE: SmObjsInVoxels assignment operator

NOTES: 
***********************************************************************/
template<class TYPE>
inline SmObjsInVoxels<TYPE> & SmObjsInVoxels<TYPE>::operator=          
 (SmObjsInVoxels<TYPE> const & crOther)    
{ 
  // no work - same object
  if(&crOther == this) return *this ;

  m_sHashTable   = crOther.m_sHashTable ;
  m_lXCnt        = crOther.m_lXCnt ;
  m_lYCnt        = crOther.m_lYCnt ;
  m_lZCnt        = crOther.m_lZCnt ;
  m_sBBox        = crOther.m_sBBox ;
  m_sPointItems  = crOther.m_sPointItems ;

  // all done    
  return *this ;

} // end  SmObjsInVoxels::operator=

/*******************************************************************//**
PURPOSE: SmObjsInVoxels equality opertor

NOTES: 
***********************************************************************/
template<class TYPE>
inline SmBoolean SmObjsInVoxels<TYPE>::operator==(const SmObjsInVoxels<TYPE> &crOther) const
{
  SmBoolean bRtn = TRUE ;

  // no work - same item
  if(this  == &crOther)
    { return TRUE ; }

  // low work - when description args aren't equal - return FALSE
  bRtn &= m_lXCnt == crOther.m_lXCnt ;
  bRtn &= m_lYCnt == crOther.m_lYCnt ;
  bRtn &= m_lZCnt == crOther.m_lZCnt ;
  bRtn &= m_sBBox == crOther.m_sBBox ;
  if(bRtn == FALSE)
    { return FALSE ; }

  // low work - When hash table isn't equal - return FALSE
  bRtn &= (m_sHashTable == crOther.m_sHashTable) ;

  // low work - When hash table isn't equal - return FALSE
  if(bRtn == FALSE)
    { return FALSE ; }

  // check the PointItems
  for(ULONG ii=0;ii<m_sPointItems.GetSize() && bRtn;ii++)
    {
      bRtn &= ((SmObjsInVoxels<TYPE> *)this)->m_sPointItems.GetAt(ii) == ((SmObjsInVoxels<TYPE> &)crOther).m_sPointItems.GetAt(ii) ;
    }

  // all done
  return(bRtn) ;

} // end SmObjsInVoxels::operator==

/*******************************************************************//**
PURPOSE: Search m_sPointItems list for the [sItem, point3d, sZoneTol3d, lItemIndx] tuple
         member where sItem == sTgtItem

RETURNS: Ptr to SmPointItem containing sTgtItem
         NULL = m_sPointItems does not contain a tuple for sTgtItem

NOTES: This is slow linear search
***********************************************************************/
template<class TYPE>
void SmObjsInVoxels<TYPE>::FindPointItems
 (TYPE             sTgtItem,       // in : TgtItem to find in the m_sPointItem [sItem, point3d, sZoneTol3d, lItemIndx] tuples
  SmTArray<ULONG> &rFoundIndices)  // out: when found: Index of TgtElement, else undefined
 const
{
  // init output
  rFoundIndices.ReSet() ;

  // for every PointItem
  for (ULONG ii=0; ii<m_sPointItems.GetSize(); ii++)
    {
      SmPointItem<TYPE> * pElem = &((SmTArray<SmPointItem<TYPE>> &)m_sPointItems).GetAt(ii);
      if(pElem->m_sTgtItem == sTgtItem)
        {
          rFoundIndices.Add(ii) ;
        } // end check found a point item referencing sTgtItem
    } // end iter every PointItem

} // end SmObjsInVoxels::FindPointItem

/*******************************************************************//**
PURPOSE: Map a point to this VoxelBox's voxel index

NOTES: Points outside the VoxelBox's BoundingBox are mapped to the
       nearest boundary voxel.

       When this->m_sBBox is uninit - all return values are set to uninit values
***********************************************************************/
template<class TYPE>
inline ULONG SmObjsInVoxels<TYPE>::MapPointToVoxel // rtn: Indx of voxel containing rPoint3d
 (SmPoint3d  & rPoint3d,               // in : Point3d to map
  SmExtent3d * pOptFoundVoxelExtent3d, // out: optional containing voxel's Extent3d, NULL to ignore, default:[NULL]
  ULONG      * pOptIndxX,              // out: IndxX of foundVoxel:[IndxX IndxY IndxZ], NULL to ignore, default:[NULL]
  ULONG      * pOptIndxY,              // out: IndxY of foundVoxel:[IndxX IndxY IndxZ], NULL to ignore, default:[NULL]
  ULONG      * pOptIndxZ,              // out: IndxZ of foundVoxel:[IndxX IndxY IndxZ], NULL to ignore, default:[NULL]
  SmBrep     * pOptBrep)               // in : for debug draw only. NULL to ignore, default:[NULL]
 const
{
  // no work - Uninit() m_sBBox
  if(m_sBBox.IsInit() || (m_lXCnt*m_lYCnt*m_lZCnt) == 0)
    { 
      if(pOptFoundVoxelExtent3d) { pOptFoundVoxelExtent3d->Init() ; }
      if(pOptIndxX)         { *pOptIndxX = SM_UNDEF_ULONG ; }
      if(pOptIndxY)         { *pOptIndxY = SM_UNDEF_ULONG ; }
      if(pOptIndxZ)         { *pOptIndxZ = SM_UNDEF_ULONG ; }
      return(SM_UNDEF_ULONG) ; 
    }

  // size the voxels
  double dXLen = m_sBBox.XLength() / m_lXCnt ;
  double dYLen = m_sBBox.YLength() / m_lYCnt ;
  double dZLen = m_sBBox.ZLength() / m_lZCnt ;
  
  // convert position to bucket voxel index -
  //     note for degen dims the Indx should always be 0
  //     note verts outside of the voxel model are snapped on the low end here
  ULONG lIndxX = (ULONG)((rPoint3d.x - m_sBBox.GetUMin()) / dXLen) ;
  ULONG lIndxY = (ULONG)((rPoint3d.y - m_sBBox.GetVMin()) / dYLen) ;
  ULONG lIndxZ = (ULONG)((rPoint3d.z - m_sBBox.GetWMin()) / dZLen) ;
  
  // bound Max values (min values already bound because we are using ULONGs)
  //     note verts outside of the voxel model are snapped on the high end here
  if(lIndxX >= m_lXCnt) { lIndxX = m_lXCnt - 1 ; }
  if(lIndxY >= m_lYCnt) { lIndxY = m_lYCnt - 1 ; }
  if(lIndxZ >= m_lZCnt) { lIndxZ = m_lZCnt - 1 ; }
  
  // get bucket Index
  ULONG lVoxelIndex = MapIndicesToVoxelIndex(lIndxX, lIndxY, lIndxZ) ; 
  
  // when asked - set outputs
  if(pOptFoundVoxelExtent3d) { MapVoxelIndexToBBox(lVoxelIndex, *pOptFoundVoxelExtent3d) ; }
  if(pOptIndxX)              { *pOptIndxX = lIndxX ; }
  if(pOptIndxY)              { *pOptIndxY = lIndxY ; }
  if(pOptIndxZ)              { *pOptIndxZ = lIndxZ ; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump(1, TRUE) ;
  
      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0,0,1) ; if(pOptBrep) pOptBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook( 2, 3, 1,0,0) ; Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook( 4, 5, 1,0,0) ; if (pOptFoundVoxelExtent3d) pOptFoundVoxelExtent3d->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(16,17, 1,0,1) ; rPoint3d.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop() ;
    }
#else // no SM_DEBUG_CODE
   SM_REF1(pOptBrep) ;
#endif // no SM_DEBUG_CODE
  
  // all done
  return(lVoxelIndex) ;

} // end SmObjsInVoxels::MapPointToVoxel

/*******************************************************************//**
PURPOSE: Map a voxel index to its bounding box

NOTES: Points outside the VoxelBox's BoundingBox are mapped to the
       nearest boundary voxel.

       When this->m_sBBox is uninit - all return values are set to uninit values
***********************************************************************/
template<class TYPE>
void SmObjsInVoxels<TYPE>::MapVoxelIndexToBBox
 (ULONG        lVoxelIndex,         // in : Tgt Voxel Index
  SmExtent3d & rFoundVoxelExtent3d) // out: TgtVoxel BoundingBox
 const
{
  // no work - Uninit() m_sBBox
  if(m_sBBox.IsInit() || (m_lXCnt*m_lYCnt*m_lZCnt) == 0)
    { 
      rFoundVoxelExtent3d.Init() ;
      return ;
    }

  // locals
  ULONG lIndxX ;
  ULONG lIndxY ;
  ULONG lIndxZ ;

  // size the voxels
  double dXLen = m_sBBox.XLength() / m_lXCnt ;
  double dYLen = m_sBBox.YLength() / m_lYCnt ;
  double dZLen = m_sBBox.ZLength() / m_lZCnt ;
  
  // map voxelIndex to indices
  MapVoxelIndexToIndices(lVoxelIndex, lIndxX, lIndxY, lIndxZ) ; 
  
  // set Voxel's BBox
  rFoundVoxelExtent3d.SetMinMax(m_sBBox.GetUMin() + (lIndxX    )*dXLen, 
                                m_sBBox.GetVMin() + (lIndxY    )*dYLen, 
                                m_sBBox.GetWMin() + (lIndxZ    )*dZLen, 
                                m_sBBox.GetUMin() + (lIndxX+1.0)*dXLen, 
                                m_sBBox.GetVMin() + (lIndxY+1.0)*dYLen, 
                                m_sBBox.GetWMin() + (lIndxZ+1.0)*dZLen) ;
} // end SmObjsInVoxels<TYPE>::MapVoxelIndexToBBox

/*******************************************************************//**
PURPOSE: Add an Item to the voxel that contains its associated point3d val.

RETURNS: Indx of voxel containing rPoint3d

METHOD:  1. Find VoxelIndex for input rPoint
         2. construct SmPointItem<TYPE> {sItem, sPoint3d, sZoneTol3d}
         3. set ItemId = m_sBucketItems.GetSize()
            construct BucketItem{pPointItem, lItemId)
         4. add sBucketItem to end of HashTable.m_sBucketItems list
            add Ptr to BucketItem to HashTable.m_sBucket[VoxelIndx]
         5. for every VoxelBndry within Tol of rPoint
              { add Ptr to BucketItem to m_sBucket[NeighborVoxelIndx]
              }
            so that points within tol of one another and on opposite
            sides of a voxel bndry will be in at least one common voxel.

NOTES: 1. setting ItemId = order in which pts are added to the Voxels
          lets a caller of FindCoincidentPairs() limit the search
          to the last points added to the voxels.  A small
          performance gain for incremental point coincident checks.

       2. Adding items to voxel neighbors when within tol of a boundary
       ensures that the voxel hash table has one or more voxels that
       contain items with point3d that are within tol of being
       coincident to one another.

       3. One PointItem is created for each voxel into which the TgtItem is added

ERRORS: When m_sBBox is in its Init() state (not yet sized) no changes are
        made and SM_UNDEF_ULONG is returned
***********************************************************************/
template<class TYPE>
inline ULONG SmObjsInVoxels<TYPE>::Add
 (TYPE              sItem,              // in : TgtItem being added to the hash table
  SmPoint3d       & rPoint3d,           // in : TgtItem's Space3d location
  SmZoneTol3d       sZoneTol3d,         // in : TgtItem's ZoneTol3d about the TgtItem's rPoint3d
  SmTArray<ULONG> * pOptUpdatedBuckets, // i/o: NotNULL = accumulate list of Bucket Indices to which Items have been added
                                        //      NULL to ignore, default:[NULL]
                                        //      note: building the pUbdatedBuckets list uses a slow AddUnique() call,
                                        //            only use this when adding a few items to already existing large list.
                                        //            Don't use when building a large list from scratch 
                                        //             - the m_sHashTable.GetUsedBucketCount() and GetUsedBucket(ii) will
                                        //               give you the used bucket list in that case.
  SmBrep          * pOptBrep)           // in : for debug draw only, NULL to ignore, default:[NULL]
{
  // locals
  ULONG      lIndxX, lIndxY, lIndxZ ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
SmBoolean bOk      = TRUE ; SM_REF1(bOk) ; 
#endif // SM_DEBUG_CODE

  // no work - no m_sBBox
  if(m_sBBox.IsInit())
    { return( SM_UNDEF_ULONG ) ; }

  // no output to init - don't init pOptUpdatedBuckets. It's an accumulator.

  // locals
  SmExtent3d sFoundVoxelBox ; 

  // map rPoint3d to a Bucket index - bounds points outside the VoxelBox to its boundary
  ULONG lVoxelIndex = MapPointToVoxel(rPoint3d, &sFoundVoxelBox, &lIndxX, &lIndxY, &lIndxZ, pOptBrep) ;
                    
  // check state - The target BucketIndex exists
  SM_ASSERT_BREAK_MSG(lVoxelIndex < m_sHashTable.GetBucketCount() && (FALSE == sFoundVoxelBox.IsInit()), _T("SmObjsInVoxels::Add() : Bad VoxelIndex or Voxel BBox not yet sized") ) ;
  SM_ASSERT_BREAK_MSG(   (FALSE == m_sBBox.ContainsPoint3d(rPoint3d, (1 + rPoint3d.GetMaxDimension()) * SM_EFF_ZERO))
                      || (TRUE == sFoundVoxelBox.ContainsPoint3d(rPoint3d, (1 + rPoint3d.GetMaxDimension()) * SM_EFF_ZERO)), _T("SmObjsInVoxels::Add() : Tgt Point3d not contained within Found Voxel") ) ; // check: contains with numerical tolerance 

// Reserve for all up-to-7 appends (center + 6 neighbor voxels) before adding: a raw SmTArray::Add
// realloc would dangle the bucket pointers aliasing &m_sPointItems[i] (UAF in FindCoincidentPairs).
  static const ULONG lMaxItemsPerAdd = 7 ;
  while(m_sPointItems.GetSize() + lMaxItemsPerAdd > m_sPointItems.GetDataSize())
    { 
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { bOk &= AssertValid() ; 
          Dump() ; 
        }
#endif // SM_DEBUG_CODE

IncMaxItemCount() ;
    
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { bOk &= AssertValid() ; 
          Dump() ; 
        }
#endif // SM_DEBUG_CODE
    } // end ensure item memory for the whole Add

  // Make and add a SmPointItem for the [sItem, rPoint3d, sZoneTol3d, lItemIndx] tuple
  SmPointItem<TYPE>   sPointItem(sItem, rPoint3d, sZoneTol3d) ;
  ULONG               lPointItemIndx =  m_sPointItems.Add(sPointItem) ;
  SmPointItem<TYPE> * pPointItem     = &m_sPointItems.GetAt(lPointItemIndx) ;

  // Add item's PointItem indx to VoxelBox Bucket while tracking used buckets (if asked)
  m_sHashTable.Add(pPointItem, lVoxelIndex, lPointItemIndx) ;

  // when asked - update pOptUpdatedBuckets list
  if(pOptUpdatedBuckets) { pOptUpdatedBuckets->AddUnique(lVoxelIndex) ; }

  // classify Point against voxel boundaries
  SmVector3d sInwardNorm ;
  SmBoolean  bIsOnBoundary = sFoundVoxelBox.IsPoint3dOnBoundary(rPoint3d, sZoneTol3d, &sInwardNorm) ;

  // Enter vertices within ZoneTol of VoxelBndrys into the neighbor voxels - this adds duplicate vertices to the hashtable
  if(bIsOnBoundary)
    {
      // on upper bndry - not the last box
      if(sInwardNorm.x == -1.0 && lIndxX < m_lXCnt-1) { lPointItemIndx = m_sPointItems.Add(sPointItem) ;
                                                        pPointItem = &m_sPointItems.GetAt(m_sPointItems.GetSize()-1) ;
                                                        m_sHashTable.Add(pPointItem, lVoxelIndex+1, lPointItemIndx) ;
                                                        if(pOptUpdatedBuckets) { (*pOptUpdatedBuckets).AddUnique(lVoxelIndex+1) ; }
#ifdef SM_DEBUG_CODE
                                                        SmExtent3d sVoxelBBox ; MapVoxelIndexToBBox(lVoxelIndex+1, sVoxelBBox) ;
                                                        SM_ASSERT_BREAK_MSG(   (FALSE == m_sBBox.ContainsPoint3d(rPoint3d, sZoneTol3d))
                                                                            || (TRUE == sVoxelBBox.ContainsPoint3d(rPoint3d, sZoneTol3d)), _T("SmObjsInVoxels::Add() : Tgt Point3d not within tol of Tgt Voxel 1")) ; 
#endif // SM_DEBUG_CODE
                                                      }
      // on lower bndry - not the first box
      if(sInwardNorm.x ==  1.0 && lIndxX > 0)         { lPointItemIndx = m_sPointItems.Add(sPointItem) ;
                                                        pPointItem = &m_sPointItems.GetAt(m_sPointItems.GetSize()-1) ;
                                                        m_sHashTable.Add(pPointItem, lVoxelIndex-1, lPointItemIndx) ;
                                                        if(pOptUpdatedBuckets) { (*pOptUpdatedBuckets).AddUnique(lVoxelIndex-1) ; }
#ifdef SM_DEBUG_CODE
                                                        SmExtent3d sVoxelBBox ; MapVoxelIndexToBBox(lVoxelIndex-1, sVoxelBBox) ;
                                                        SM_ASSERT_BREAK_MSG(   (FALSE == m_sBBox.ContainsPoint3d(rPoint3d, sZoneTol3d))
                                                                            || (TRUE == sVoxelBBox.ContainsPoint3d(rPoint3d, sZoneTol3d)), _T("SmObjsInVoxels::Add() : Tgt Point3d not within tol of Tgt Voxel 2")) ; 
#endif // SM_DEBUG_CODE
                                                      }
  
      // on upper bndry - not the last box
      if(sInwardNorm.y == -1.0 && lIndxY < m_lYCnt-1) { lPointItemIndx = m_sPointItems.Add(sPointItem) ;
                                                        pPointItem = &m_sPointItems.GetAt(m_sPointItems.GetSize()-1) ;
                                                        m_sHashTable.Add(pPointItem, lVoxelIndex+m_lXCnt, lPointItemIndx) ;
                                                        if(pOptUpdatedBuckets) { (*pOptUpdatedBuckets).AddUnique(lVoxelIndex+m_lXCnt) ; }
#ifdef SM_DEBUG_CODE
                                                        SmExtent3d sVoxelBBox ; MapVoxelIndexToBBox(lVoxelIndex+m_lXCnt, sVoxelBBox) ;
                                                        SM_ASSERT_BREAK_MSG(   (FALSE == m_sBBox.ContainsPoint3d(rPoint3d, sZoneTol3d))
                                                                            || (TRUE == sVoxelBBox.ContainsPoint3d(rPoint3d, sZoneTol3d)), _T("SmObjsInVoxels::Add() : Tgt Point3d not within tol of Tgt Voxel 3")) ; 
#endif // SM_DEBUG_CODE
                                                      }
      // on lower bndry - not the first box
      if(sInwardNorm.y ==  1.0 && lIndxY > 0)         { lPointItemIndx = m_sPointItems.Add(sPointItem) ;
                                                        pPointItem = &m_sPointItems.GetAt(m_sPointItems.GetSize()-1) ;
                                                        m_sHashTable.Add(pPointItem, lVoxelIndex-m_lXCnt, lPointItemIndx) ;
                                                        if(pOptUpdatedBuckets) { (*pOptUpdatedBuckets).AddUnique(lVoxelIndex-m_lXCnt) ; }
#ifdef SM_DEBUG_CODE
                                                        SmExtent3d sVoxelBBox ; MapVoxelIndexToBBox(lVoxelIndex-m_lXCnt, sVoxelBBox) ;
                                                        SM_ASSERT_BREAK_MSG(   (FALSE == m_sBBox.ContainsPoint3d(rPoint3d, sZoneTol3d))
                                                                            || (TRUE == sVoxelBBox.ContainsPoint3d(rPoint3d, sZoneTol3d)), _T("SmObjsInVoxels::Add() : Tgt Point3d not within tol of Tgt Voxel 4")) ; 
#endif // SM_DEBUG_CODE
                                                      }
  
      // on upper bndry - not the last box
      if(sInwardNorm.z == -1.0 && lIndxZ < m_lZCnt-1) { lPointItemIndx = m_sPointItems.Add(sPointItem) ;
                                                        pPointItem = &m_sPointItems.GetAt(m_sPointItems.GetSize()-1) ;
                                                        m_sHashTable.Add(pPointItem, lVoxelIndex+(m_lXCnt*m_lYCnt), lPointItemIndx) ;
                                                        if(pOptUpdatedBuckets) { (*pOptUpdatedBuckets).AddUnique(lVoxelIndex+( m_lXCnt*m_lYCnt)) ; }
#ifdef SM_DEBUG_CODE
                                                        SmExtent3d sVoxelBBox ; MapVoxelIndexToBBox(lVoxelIndex+(m_lXCnt*m_lYCnt), sVoxelBBox) ;
                                                        SM_ASSERT_BREAK_MSG(   (FALSE == m_sBBox.ContainsPoint3d(rPoint3d, sZoneTol3d))
                                                                            || (TRUE == sVoxelBBox.ContainsPoint3d(rPoint3d, sZoneTol3d)), _T("SmObjsInVoxels::Add() : Tgt Point3d not within tol of Tgt Voxel 5")) ; 
#endif // SM_DEBUG_CODE
                                                      }
      // on lower bndry - not the first box
      if(sInwardNorm.z ==  1.0 && lIndxZ > 0)         { lPointItemIndx = m_sPointItems.Add(sPointItem) ;
                                                        pPointItem = &m_sPointItems.GetAt(m_sPointItems.GetSize()-1) ;
                                                        m_sHashTable.Add(pPointItem, lVoxelIndex-(m_lXCnt*m_lYCnt), lPointItemIndx) ;
                                                        if(pOptUpdatedBuckets) { (*pOptUpdatedBuckets).AddUnique(lVoxelIndex-( m_lXCnt*m_lYCnt)) ; }
#ifdef SM_DEBUG_CODE
                                                        SmExtent3d sVoxelBBox ; MapVoxelIndexToBBox(lVoxelIndex-(m_lXCnt*m_lYCnt), sVoxelBBox) ;
                                                        SM_ASSERT_BREAK_MSG(   (FALSE == m_sBBox.ContainsPoint3d(rPoint3d, sZoneTol3d))
                                                                            || (TRUE == sVoxelBBox.ContainsPoint3d(rPoint3d, sZoneTol3d)), _T("SmObjsInVoxels::Add() : Tgt Point3d not within tol of Tgt Voxel 6")) ; 
#endif // SM_DEBUG_CODE
                                                      }
    } // end point3d IsOnBoundary of containing voxel check
  
#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      Dump() ;

      bOk &= AssertValid() ; 
      bOk &= m_sHashTable.AssertValid() ;
      SM_ASSERT_MSG(bOk == TRUE, _T("SmObjsInVoxels::Add - HashTable failed AssertValid() after Add()")) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return lVoxelIndex ;

} // end SmObjsInVoxels::Add

/*******************************************************************//**
PURPOSE: Remove all BucketItems referencing a Target Item from their voxels

NOTES: 1. Remove items is an expensive linear search - minimize the use of Remove()

METHOD: Removed items are 
          a. removed from the m_sHashTable sort tree
          b. not removed from the m_sHashTable.m_sBucketItems pointer array
             not removed frome the m_sPointItems object array
               so that 
               - all the m_sHashTable BucketItem ptrs don't become stale saving having to update those pointers
               - and the m_sHashTable.m_sBucketItems pointers and m_sPointItems objects continue
                    to correspond to each other one-to-one, eg
                      . m_sHashTable.m_sBucketItems.GetSize() == m_sPointItems.GetSize()
                      . m_sHashTable.m_sBucketItems[ii] == &(m_sPointItems[ii])
***********************************************************************/
template<class TYPE>
inline ULONG SmObjsInVoxels<TYPE>::Remove 
 (TYPE sItem)      // in : Item ptr to remove from hashtable
{
  // locals
  ULONG ii ;
  SmTArray<ULONG> sFoundIndices ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
SmBoolean bOk      = TRUE ; SM_REF1(bOk) ; 
#endif // SM_DEBUG_CODE

#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { bOk &= AssertValid() ; 
          Dump() ; 
        }
#endif // SM_DEBUG_CODE

  // find all PointItems referencing sItem
  FindPointItems(sItem, sFoundIndices) ; 

  // pass the call along to m_HashTable to remove all m_sBucketItems<SmPointItem *> referencing sItem from the sort tree.
  ULONG lRemovedCount = m_sHashTable.Remove(sFoundIndices) ;

  // update the SmObjsInVoxels state
  m_lRemovedPointItemCount += lRemovedCount ;

  // set all found PointItem values to removed
  for(ii=0;ii<sFoundIndices.GetSize();ii++)
    {
      SmPointItem<TYPE> &rPointItem = m_sPointItems[sFoundIndices[ii]] ;

      // rPointItem.m_sTgtItem       // not always a pointer - so no good uninit value to use here
      rPointItem.m_sTgtPoint3d.SetUninitialized() ;        
      rPointItem.m_sTgtZoneTol3d = SM_UNDEF_DOUBLE ;
    }
  
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        { bOk &= AssertValid() ; 
          Dump() ; 
        }
#endif // SM_DEBUG_CODE

  // all done
  return lRemovedCount ;

} // end SmObjsInVoxels<TYPE>::Remove

// obsolete Remove code
//   // The HashTable BucketItems point back to the SmObjsInVoxels->m_sPointItems objs.
//   // The removed m_sPointItems[ii] obj values are all updated
//   // Find the SmPointItem for the [sItem, rPoint3d, sZoneTol3d, lItemIndx] tuple
//   ULONG               lPointItemIndx ;
//   SmPointItem<TYPE> * pPointItem = FindPointItem(sItem, lPointItemIndx) ;
// 
//   // no work - no sItem
//   if(pPointItem == NULL)
//     { return ; }
// 
//   // remove the PointItem ptr from m_sHashTable - pPointItem becomes a 'removed' PointItem
//   m_sHashTable.Remove(pPointItem) ;
// 
//   // remove the PointItem from m_sPointItems list - RemoveAt compresses the m_pPointItems array which makes the m_sHashTable.m_sBuketItem Item ptrs stale
//   m_sPointItems.RemoveAt(lPointItemIndx, 1) ;
// 
//   // update assoc m_sHashTable.m_sBucketItems.m_lItemId (ItemId values above lPointItemIndx have decremented)
//   SmTArray<SmBucketItem<SmPointItem<TYPE>*>> &rBucketItems = m_sHashTable.GetBucketItems() ;
// 
//   for(ULONG ii=0;ii<rBucketItems.GetSize();ii++)
//     {
//       ULONG lItemId = rBucketItems[ii].GetItemId() ;
//       if(   lItemId > lPointItemIndx
//          && lItemId != SM_UNDEF_ULONG)
//         {
//           // decrement the stored index
//           rBucketItems[ii].SetItemId(lItemId-1) ;
//           rBucketItems[ii].SetItem(&m_sPointItems.GetAt(lItemId-1)) ;
//         }
//     } // end iter rBucketItems updating Item back ptrs
// 
// #ifdef SM_DEBUG_CODE
// SmBoolean bDebugMe = FALSE ;
//   if(bDebugMe)
//     {
//       Dump() ;
// 
//       SmBoolean bRtn = m_sHashTable.AssertValid() ;
//       SM_ASSERT_MSG(bRtn == TRUE, _T("SmObjsInVoxels::Remove - HashTable failed AssertValid() after Remove()")) ;
//     }
// #endif // SM_DEBUG_CODE
// 
// } // end SmObjsInVoxels::Remove
// end obsolete

/*******************************************************************//**
PURPOSE: Find Coincident PointItem pairs 

NOTES: The output coincident pair list is a unique pair list
       even though coincident pairs whose point3d are both within
       tol of a commonly voxel boundary are listed in multiple
       voxels of the SmObjsInVoxels->m_sHashTable.
***********************************************************************/
template<class TYPE>
inline void SmObjsInVoxels<TYPE>::FindCoincidentPairs
 (SmTArray<SmPointItem<TYPE> *> & rCoin1,            // out: array of PointItem1s for each CoincidentPair:[Pair1_PointItem1, Pair2_PointItem1,.. , PairN_PointItem1]
  SmTArray<SmPointItem<TYPE> *> & rCoin2,            // out: array of PointItem2s for each CoincidentPair:[Pair1_PointItem2, Pair2_PointItem2,.. , PairN_PointItem2]
                                                     //        note: while a PointItem might show up many times in each list,
                                                     //              Each combined [PointItem1, PointItem2] coin pair is unique
  ULONG                         * pOptLimitIndx,     // in : NotNULL, Chk PointItems with Indxs >= *pOptLimitIndx against all PointItems for coin,
                                                     //      NULL,    chk all PointItems against all PointItems for coin,
                                                     //      default:[NULL]
  SmTArray<ULONG>               * pOptBucketIndices, // in : NotNULL = List of buckets to be checked for coincident pairs
                                                     //      NULL    = Check all buckets, default:[NULL]
                                                     //      note: When adding a few Items to the voxels and then checking for coincidence
                                                     //            set pOptStartIndx = SizeOf m_sPointItems before adding new Items
                                                     //            let pOptBucketIndices   = Add's pOptUpdatedBuckets argument
  SmTArray<ULONG>               * pOptHistogram)     // out: Optional Histogram of PtObj/PtObj gap sizes
{
  // locals
  ULONG ii, jj ;
  SmTArray<SmBucketItem<SmPointItem<TYPE>*> *> & rBuckets = m_sHashTable.GetBuckets() ;

  // init outputs  - don't init pOptHistogram. It's an accumulator
  rCoin1.ReSet() ;
  rCoin2.ReSet() ;

  // get the 1st used bucket - this is just the 1st bucketobject within a linklist of bucket objects in the used bucket
  SmBucketItem<SmPointItem<TYPE>*> * pUsedBucket = NULL ;

  // for every bucket - look for coincident items. when asked only in pOptBucketIndices list, else in all buckets
  for(ii=0, pUsedBucket = (pOptBucketIndices) ? rBuckets[pOptBucketIndices->GetAt(0)] //  SmTArray<SmBucketItem<TYPE> *> m_sBuckets[ii] => ptr to SmBucketItem<TYPE>
                                              :  m_sHashTable.GetUsedBucket(0) ;      // => ptr to SmBucketItem<TYPE>
                          (pOptBucketIndices) ? (ii < pOptBucketIndices->GetSize())      // when walking the LoadedBuckets list for case: add and chk new PtItems to existing PtItems in this voxel model 
                                              : (pUsedBucket != NULL) ;                  // when walking the used buckets list for all vertex case
      ii++, pUsedBucket = (pOptBucketIndices) ? (ii < pOptBucketIndices->GetSize() ? rBuckets[pOptBucketIndices->GetAt(ii)] : NULL) // when walking the LoadedBuckets list for Adding vertex case
                                              : pUsedBucket->GetNextUsedBucket())           // when walking the used buckets list for all vertex case
    {
      // locals - search beg
      SmBucketItem<SmPointItem<TYPE>*> * pBegBucketItem = pUsedBucket ;
      SmPointItem<TYPE>                * pBegPointItem  = pBegBucketItem ? pBegBucketItem->GetItem() : NULL ;
      SmPoint3d                        & rBegPoint      = pBegPointItem->m_sTgtPoint3d  ;

      // while the bucket has entries
      for(;pBegBucketItem != NULL; pBegBucketItem = pBegBucketItem->GetNextBucketItem())
        {
          // check state - pBegBucketItem is not a removed BucketItem
          SM_ASSERT_BREAK_MSG(pBegBucketItem->IsRemovedBucketItem() == FALSE, _T("SmObjsInVoxels<TYPE>::FindCoincidentPairs: error - found a begin RemovedBucketItem left behind in pUsedBucket")) ; 

          // when asked - skip pointItems below the limitIndx (presumably they've been checked for coincidence on a previous call)
          if(   pOptLimitIndx 
             && pBegBucketItem->GetItemId() < *pOptLimitIndx)
            { continue ; }

          // get first pair BucketItem to compare
          SmBucketItem<SmPointItem<TYPE>*> * pEndBucketItem = pBegBucketItem != NULL ? pBegBucketItem->GetNextBucketItem() : NULL ;
          SmPointItem<TYPE>                * pEndPointItem  = pEndBucketItem ? pEndBucketItem->GetItem() : NULL ;
          SmPoint3d                        & rEndPoint      = pEndPointItem->m_sTgtPoint3d  ;

          // while there are pairs of BucketItems in this bucket to compare
          for(;pEndBucketItem != NULL; pEndBucketItem = pEndBucketItem->GetNextBucketItem())
            {
              // check state - pEndBucketItem is not a removed BucketItem
              SM_ASSERT_BREAK_MSG(pEndBucketItem->IsRemovedBucketItem() == FALSE, _T("SmObjsInVoxels<TYPE>::FindCoincidentPairs: error - found a end RemovedBucketItem left behind in pUsedBucket")) ; 

              // update locals
              pEndPointItem  = pEndBucketItem->GetItem() ;
              rEndPoint      = pEndPointItem->m_sTgtPoint3d  ;

              // gap and XSectTol between pairs of Vertices
              SmXSectTol3d sXSectTol3d   = pBegPointItem->m_sTgtZoneTol3d + pEndPointItem->m_sTgtZoneTol3d ;
              SmXSectTol3d sXSectTol3dSq = sXSectTol3d * sXSectTol3d ;
              double       dGapSq        = (rBegPoint - rEndPoint).LengthSquared() ;

              // when asked, make a histogram of all gaps within common buckets
              if(pOptHistogram)
                {
                  double       dGap        = smos_Sqrt(dGapSq) ;
                  pOptHistogram->GetAt(SM_HISTOGRAM_GAPSIZE_INDEX(dGap))++ ;
                }

              // check items for coincidence
              if(dGapSq < sXSectTol3dSq)
                {
                  // Is this pair already registered
                  SmBoolean bIn = FALSE ;
                  for(jj=0;jj<rCoin1.GetSize();jj++)
                    {
                      // skip pairs that don't match the 1st pair member
                      if(   rCoin1[jj]->m_sTgtItem != pBegPointItem->m_sTgtItem
                         && rCoin1[jj]->m_sTgtItem != pEndPointItem->m_sTgtItem) 
                        { continue ; }

                      // when the current pair match the 1st and 2nd pair members
                      if(   (   rCoin1[jj]->m_sTgtItem == pBegPointItem->m_sTgtItem 
                             && rCoin2[jj]->m_sTgtItem == pEndPointItem->m_sTgtItem)
                         || (   rCoin2[jj]->m_sTgtItem == pBegPointItem->m_sTgtItem 
                             && rCoin1[jj]->m_sTgtItem == pEndPointItem->m_sTgtItem))
                        {
                          // note that this is a duplicate and carry on
                          bIn = TRUE ;
                          break ;
                        }
                    } // end iter CoincidentVertex list looking for duplicates

                  // When pair is unique add it to the output
                  if(bIn == FALSE)
                    {
                      rCoin1.Add(pBegPointItem) ; 
                      rCoin2.Add(pEndPointItem) ;
                    } // end coincident vertex pair not yet in output check
                } // end found a coincident vertex pair check
            } // end iter every bucket end entry
        } // end iter every bucket beg entry
    } // end iter every bucket
#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE ; 
  if(bDebugMe)
    {
      Dump() ;
      if(pOptBucketIndices) { m_sHashTable.DumpBuckets(*pOptBucketIndices) ; }
      DumpCoincidentPairs(rCoin1, rCoin2) ;
    }
#endif // SM_DEBUG_CODE

} // end SmObjsInVoxels::FindCoincidentPairs

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
// see definition of
// SmAssertReportLabel sAssertObjsInVoxels_list[] =
// in file: SmHashTable.cpp

/*******************************************************************//**
PURPOSE: Virtual method used to determine validity of objects.

NOTES: For ObjsInVoxels: A SmObjsInVoxels 
        - checks consistency of array sized between the SmObjsInVoxels and the contained SmObjsInVoxels.m_sHashTable array sizes
        - checks that SmObjsInVoxels stored PointItems BucketIds refer to HashTable buckets that contain a pointer back tothe Point Item

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
template<class TYPE>
inline SmBoolean SmObjsInVoxels<TYPE>::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * pTestRequests) // NotUsed: in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  SM_REF1(pTestRequests);

  // return value
  SmBoolean bRtn = TRUE ;

  // locals
  ULONG ii ;
  ULONG      lIndxX=999999 ;
  ULONG      lIndxY=999999 ;
  ULONG      lIndxZ=999999 ;
  SmExtent3d sVoxelExtent3d ;
  SmObjsInVoxels<TYPE> * pNonConst =  (SmObjsInVoxels<TYPE> *)this ;

  /* 0 */ // check ObjsInVoxels contained m_sHashTable object
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (TRUE == pNonConst->m_sHashTable.AssertValid(pAList,eTestLevel,eWalkTree,pTestRequests)), _T("")) ;

  /* 1 */ // check ObjsInVoxels voxel count equal to HashTable bucket count
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (m_lXCnt * m_lYCnt * m_lZCnt == pNonConst->m_sHashTable.GetBuckets().GetSize()), _T("")) ;

  /* 2 */ // check ObjsInVoxels point items == HashTable total BucketItems count 
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (m_sPointItems.GetSize() == pNonConst->m_sHashTable.GetBucketItems().GetSize()), _T("")) ;

  // check SmObjsInVoxels-to-SmHashTable member cross references
  for(ii=0;ii<m_sPointItems.GetSize();ii++)
    {
      // skip removed PointItems — their BucketItem has been removed from the hash tree
      if(pNonConst->m_sHashTable.GetBucketItem(ii).IsRemovedBucketItem()) { continue ; }

      // locals 
      SmTArray<SmBucketItem<SmPointItem<TYPE>*>*> sBucketItemsInOneBucket ;
      const SmPointItem<TYPE> *                   pPointItem    = &m_sPointItems[ii] ; // iith PointItem - PointItems are referenced by BucketItems
      ULONG                                       lRawVoxelIndx =  MapPointToVoxel( pNonConst->m_sPointItems[ii].m_sTgtPoint3d,  // Voxel index for PointItem's point location
                                                                                    &sVoxelExtent3d, 
                                                                                    &lIndxX,
                                                                                    &lIndxY,
                                                                                    &lIndxZ,
                                                                                    NULL) ;
      const SmBucketItem<SmPointItem<TYPE>*>    * pBucketItem   = &pNonConst->m_sHashTable.GetBucketItem(ii) ;
      ULONG                                       lBucketIndx   =  pBucketItem->GetBucketIndex() ; 
      SmPointItem<TYPE> *                         pItem         =  pBucketItem->GetItem() ;
      ULONG                                       lItemId       =  pBucketItem->GetItemId() ;
      m_sHashTable.GetOneBucket_BucketItems(pBucketItem->GetBucketIndex(), sBucketItemsInOneBucket) ;

      /* 3 */ // check m_sHashTable.BucketItem[ii]->m_sItem == associated m_sHashTable.m_sPointItems[ii]  ((lXIndex != 0) && (lXIndex != m_lXCnt-1))
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (pPointItem == pBucketItem->GetItem()), _T("")) ;  

      // since PointItems near voxel boundaries get placed into the voxels on both sides of the near boundary 
      //   using the voxel indexing scheme of VoxelIndx = lIndxZ * (m_lXCnt*m_lYCnt) + lIndxY * (m_lXCnt) + lIndxX ;
      /* 4 */ // check m_sHashTable->BucketItem[ii]->BucketIndex is within 1 voxel of the VoxelIndex computed from m_sPointItems[ii]->m_sTgtPoint3d
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (   (lBucketIndx == lRawVoxelIndx)                                       // raw voxel
                                                       || (lBucketIndx == lRawVoxelIndx + ( (lIndxX != m_lXCnt-1) ?  1 : 0))   // raw voxel + 1 in X
                                                       || (lBucketIndx == lRawVoxelIndx - ( (lIndxX != 0)         ?  1 : 0))   // raw voxel - 1 in X
                                                       || (lBucketIndx == lRawVoxelIndx + ( (lIndxY != m_lYCnt-1) ?  m_lXCnt : 0))                        // raw voxel + 1 in Y
                                                       || (lBucketIndx == lRawVoxelIndx - ( (lIndxY != 0)         ?  m_lXCnt : 0))                        // raw voxel - 1 in Y
                                                       || (lBucketIndx == lRawVoxelIndx + ( (lIndxZ != m_lZCnt-1) ?  m_lYCnt * m_lXCnt : 0))              // raw voxel + 1 in Z
                                                       || (lBucketIndx == lRawVoxelIndx - ( (lIndxZ != 0)         ?  m_lYCnt * m_lXCnt : 0))), _T("")) ;  // raw voxel - 1 in Z

      /* 5 */ // check ObjsInVoxels m_sPointItems[ii].m_lBucketIndex refers to a HashTable.m_sBuckets[m_lBucketIndex] that contains a BucketItem with a ref to the origin PointItem 
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, (TRUE == m_sHashTable.IsInBucket(lBucketIndx, pBucketItem)), _T("")) ;

      /*  6 */ // ObjsInVoxels &m_sPointItems[m_sHashTable.m_sBucektItems[ii].m_sItemId] == m_sHashTable.m_sBucketItems[ii].m_sItem
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, (   (lItemId == SM_UNDEF_ULONG)
                                                       || (&m_sPointItems[lItemId] == pItem) ), _T("")) ;

      /*  7 */ // ObjsInVoxels m_sHashTable.m_sBucketItems[ii].m_sItemId index value in range:[0, m_sPointItems.GetSize()]
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(7, SM_LEVEL_0, (   (lItemId >= 0)
                                                       && (lItemId < m_sPointItems.GetSize()) ), _T("")) ;
    }

  // all done
  return bRtn ; 

} // end SmObjsInVoxels<TYPE>::AssertValid

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmObjsInVoxels<TYPE>::Dump() const
{
  // pass the call along - don't display the bucket item contents
  Dump(99, FALSE) ;

} // end SmObjsInVoxels<TYPE>::Dump - no args

/*******************************************************************//**
PURPOSE: Formatted Write for debug purposes

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmObjsInVoxels<TYPE>::Dump
 (ULONG    lLabel,     // in : numeric label for this dump display, default:[0]
 SmBoolean bFull)      // in : default:[TRUE] = display the bucket item contents. FALSE=don't
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin Header 
  smos_sprintf(sBuff,        _T("\nBegin:[%3lu] SmObjsInVoxels:[0x%p] Dump, PointItemCnt:[%5lu]"), lLabel, this, m_sPointItems.GetSize()) ; 
  smos_sprintf(sBuffForFile, _T("\nBegin:[%3lu] SmObjsInVoxels:[%s] Dump, PointItemCnt:[%5lu]"), lLabel, _T("notNULL"), m_sPointItems.GetSize()) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // Voxel Model
  smos_sprintf(sBuff,        _T("%s"), _T("\n  Voxel Model")) ;                 smos_WriteBuffer(sBuff) ;
  m_sBBox.Dump() ;
  smos_sprintf(sBuff,        _T("\n  Voxel X Cnt:[%5lu]"), m_lXCnt) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,        _T("\n  Voxel Y Cnt:[%5lu]"), m_lYCnt) ; smos_WriteBuffer(sBuff) ;
  smos_sprintf(sBuff,        _T("\n  Voxel Z Cnt:[%5lu], Total Cnt:[%5lu]"), m_lZCnt, m_lXCnt*m_lYCnt*m_lZCnt) ; smos_WriteBuffer(sBuff) ;
                          
  // Hash table
  m_sHashTable.Dump(bFull) ;

  // locals
  SmObjsInVoxels<TYPE> * pNonConst = (SmObjsInVoxels<TYPE> *)this ;

  // multi-item buckjet point lists
  SmBucketItem<SmPointItem<TYPE> *> * pUsedBucket = pNonConst->m_sHashTable.GetUsedBucket(0) ; 
  for(;pUsedBucket != NULL; pUsedBucket = pUsedBucket->GetNextUsedBucket())
    {
      SmTArray<SmBucketItem<SmPointItem<TYPE> *> *> sBucketItems ;
      m_sHashTable.GetOneBucket_BucketItems(pUsedBucket->GetBucketIndex(), sBucketItems) ;

      // when used bucket has multiple BucketItems
      if(sBucketItems.GetSize() > 1)
        {
          // bucket header
          smos_sprintf(sBuff,        _T("\n  Begin Bucket[%5lu] Point Set:"), pUsedBucket->GetBucketIndex()) ;   
          smos_WriteBuffer(sBuff) ;

          for(ULONG ii=0;ii<sBucketItems.GetSize();ii++)
            {

              SmBucketItem<SmPointItem<TYPE> *> * pBucketItem = sBucketItems[ii] ;

              // BucketItem Point
              if(ii==0) { smos_sprintf(sBuff,        _T(" BucketItem:[%5lu], PointId:[%5lu], pos: "), ii, pBucketItem->GetItemId()) ; }
              else      { smos_sprintf(sBuff,        _T("\n                                 BucketItem:[%5lu], PointId:[%5lu], pos: "), ii, pBucketItem->GetItemId()) ; }
              smos_WriteBuffer(sBuff) ;
              pBucketItem->GetItem()->m_sTgtPoint3d.Dump() ;
            } // end iter item in this bucket
        } // end MultiItem bucket check
    } // end iter all UsedBuckets

  // End Footer 
  smos_sprintf(sBuff,        _T("\nEnd:[%3lu] SmObjsInVoxels:[0x%p] Dump"), lLabel, this) ;
  smos_sprintf(sBuffForFile, _T("\nEnd:[%3lu] SmObjsInVoxels:[%s] Dump"), lLabel, _T("notNULL")) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;
  
} // end SmObjsInVoxels<TYPE>::Dump

/*******************************************************************//**
PURPOSE: Pretty print list of specified Buckets

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmObjsInVoxels<TYPE>::DumpTgtBucketItems
 (SmTArray<SmBucketItem<SmPointItem<TYPE>*> *> &rBucketItems)  // in : Array of BucketItem Ptrs to Pretty Print
 const  
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin Header 
  smos_sprintf(sBuff,        _T("\nBegin TgtBucketItems Dump from HashTable:[0x%p]"), this) ; 
  smos_sprintf(sBuffForFile, _T("\nBegin TgtBucketItems Dump from HashTable:[%s]"), _T("notNULL")) ; 
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // Tgt Bucket index list
  for(ii=0;ii<rBucketItems.GetSize();ii++)
    {
      SmBucketItem<SmPointItem<TYPE> *> * pBucketItem = rBucketItems[ii] ;   
      
      if(ii==0) smos_WriteBuffer(_T("\n  Dumping TgtBucketItems: ")) ;
      else      smos_WriteBuffer(_T("\n                          ")) ; 
      
      // ItemId
      smos_sprintf(sBuff, _T("ItemId:[%2lu]"), pBucketItem->GetItemId()) ; 
      smos_WriteBuffer(sBuff) ;

      // BucketId
      smos_sprintf(sBuff, _T(", InBucketIndex:[%2lu]"), pBucketItem->GetBucketIndex() ) ;
      smos_WriteBuffer(sBuff) ;

      // Next ItemId
      if(pBucketItem->GetNextBucketItem() == NULL) { smos_WriteBuffer(_T(", Next ItemId:[None]")) ; }
      else                                         { smos_sprintf(sBuff, _T(", Next ItemId:[%2lu]"), pBucketItem->GetNextBucketItem()->GetItemId() ) ;
                                                     smos_WriteBuffer(sBuff) ;
                                                   }
    }

  // End Footer 
  smos_sprintf(sBuff,        _T("\nEnd TgtBucketItems Dump from HashTable:[0x%p]"), this) ; 
  smos_sprintf(sBuffForFile, _T("\nEnd TgtBucketItems Dump from HashTable:[%s]"), _T("notNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;
  
} // end SmObjsInVoxels<TYPE>::DumpTgtBucketItems 

/*******************************************************************//**
PURPOSE: Convenience Pretty print for FinCoincidentPairs() output

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmObjsInVoxels<TYPE>::DumpCoincidentPairs
 (SmTArray<SmPointItem<TYPE> *> & rCoin1,   // in: PointItem1 of CoincidentPair:[PointItem1, PointItem2]
  SmTArray<SmPointItem<TYPE> *> & rCoin2)   // in: PointItem2 of CoincidentPair:[PointItem1, PointItem2]
 const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  
  // header
  smos_sprintf(sBuff, _T("\nBegin Coincident Pairs Dump: CoinPairCnt:[%5lu]"), rCoin1.GetSize()) ; 
  smos_WriteBuffer(sBuff) ;

  // for every coin pair - dump a report
  for(ii=0;ii<rCoin1.GetSize();ii++)
    {
      smos_sprintf(sBuff, _T("\n  [%2lu] Coincident Pair"), ii) ;
      smos_WriteBuffer(sBuff) ;
  
      rCoin1[ii]->Dump(ii, 4) ; 
      rCoin2[ii]->Dump(ii, 4) ;
    }
  
  // footer
  smos_sprintf(sBuff, _T("\nEnd Coincident Pairs Dump: CoinPairCnt:[%5lu]"), rCoin1.GetSize()) ; 
  smos_WriteBuffer(sBuff) ;
  
} // end SmObjsInVoxels<TYPE>::DumpCoincidentPairs

/***************************************************************
PURPOSE:  Add point sequence graphics for global display Parameters
             to new DisplayList added to global DisplayList array.

NOTES:
***************************************************************/
template<class TYPE>
inline SmDisplayList * SmObjsInVoxels<TYPE>::Draw
 (SmGfxArraySet * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
 const
{
  // init return value
  SmDisplayList *pRtn = NULL ;

  // no work - SmObjsInVoxel is uninitialized - no graphics
  if(m_sBBox.IsInit() && (m_lXCnt*m_lYCnt*m_lZCnt) == 0)
    { return pRtn ; }

#ifdef SM_GFX_CODE

  // start new DisplayList (unless displayList is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  smgfx_Open(smgfx_GetRuleColor(NULL),NULL, NULL, FALSE, pOptGfxSet);

  //  // set Brep faces to draw crosshatched
  //  SmDisplayParameters & rDisp              = smgfx_RefGlobalDisplayParameters() ;
  //  SmBoolean             bOldDrawCrossHatch = rDisp.m_bDrawCrossHatch ; 
  //  rDisp.m_bDrawCrossHatch = TRUE ;

  // locals
  ULONG ii ;
  SmVector3d  sOrigin = m_sBBox.GetMin() ;        
  SmVector3d  sX(1,0,0), sY(0,1,0), sZ(0,0,1) ;   
  SmExtent2d  sUVDomain(0,0,1,1) ;
  SmVector2d  sXYScale(m_sBBox.XLength(), m_sBBox.YLength()) ;
  SmVector2d  sYZScale(m_sBBox.YLength(), m_sBBox.ZLength()) ;
  SmVector2d  sZXScale(m_sBBox.ZLength(), m_sBBox.XLength()) ;

  // draw Voxel extent
  m_sBBox.Draw(NULL, NULL, pOptGfxSet) ;

  //  when there are voxels to draw
  if((m_lXCnt*m_lYCnt*m_lZCnt) > 0)
    { 

      // draw XY Planes
      for(ii=0;ii<=m_lZCnt;ii++) { SmVector3d dDelZ(0,
                                                    0,
                                                    m_sBBox.ZLength() / m_lZCnt) ;
                                   SmPlane sPlane(sOrigin + ii*dDelZ, sX, sY, sXYScale, sUVDomain) ;
                                   sPlane.Draw(FALSE, pOptGfxSet) ;
                                 }
      // draw YZ Planes
      for(ii=0;ii<=m_lXCnt;ii++) { SmVector3d dDelX(m_sBBox.XLength() / m_lXCnt, 
                                                    0, 
                                                    0) ;
                                   SmPlane sPlane(sOrigin + ii*dDelX, sY, sZ, sYZScale, sUVDomain) ;
                                   sPlane.Draw(FALSE, pOptGfxSet) ;
                                 }
      // draw ZX Planes
      for(ii=0;ii<=m_lYCnt;ii++) { SmVector3d dDelY(0, 
                                                    m_sBBox.YLength() / m_lYCnt, 
                                                    0) ;
                                   SmPlane sPlane(sOrigin + ii*dDelY, sZ, sX, sZXScale, sUVDomain) ;
                                   sPlane.Draw(FALSE, pOptGfxSet) ;
                                 }
    } // end voxel count check
    
  //  // restore state
  //  rDisp.m_bDrawCrossHatch = bOldDrawCrossHatch ;

  // end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmObjsInVoxels<TYPE>::Draw

SM_EXPORT_TEMPLATE template class SM_EXPORT SmObjsInVoxels<SmVertex*> ;
SM_EXPORT_TEMPLATE template class SM_EXPORT SmObjsInVoxels<SmVertexProps*> ;

SM_TARRAY_TEMPLATE_PREDECLARATION(SmObjsInVoxels<SmVertex*>);
SM_TARRAY_TEMPLATE_PREDECLARATION(SmObjsInVoxels<SmVertex*>*);
SM_TARRAY_TEMPLATE_PREDECLARATION(SmObjsInVoxels<SmVertexProps*>);
SM_TARRAY_TEMPLATE_PREDECLARATION(SmObjsInVoxels<SmVertexProps*>*);

#endif  // !__SMHASHTABLE_H__

