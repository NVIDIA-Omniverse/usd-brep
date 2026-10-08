// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmMapTypeToType.h
* PURPOSE: Header file for SmMapTypeToType object.
**********************************************************************/

/* Array Object definitions */
#ifndef __SMMAPTYPETOTYPE_H__
#define __SMMAPTYPETOTYPE_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMOBJECT_H__
#include <SmObject.h>
#endif


#include <sstream>

#ifdef _WIN32
 #define SM_MAPTYPETOTYPE_TEMPLATE_PREDECLARATION(x,y)      \
         template struct SM_EXPORT SmAssocTypeToType<x,y> ; \
         template class  SM_EXPORT SmMapTypeToType<x,y>

#else // no _WIN32
 #define SM_MAPTYPETOTYPE_TEMPLATE_PREDECLARATION(x,y)
#endif // no _WIN32


#ifndef SM_DEF_HASH_SIZE
#define SM_DEF_HASH_SIZE 4096  // GWC: this is huge and requires
#endif

#define BEFORE_START_MAP_POSITION ((SM_MAP_POSITION)-1L)

/*******************************************************************//**
PURPOSE: a low level memory scheme for a linked list of constant sized mem-block objects.

NOTES: An SmPlex = [ mem for: 1 LinkedList ptr to next SmPlex,
                              1 memory block, SizedInBytes:[nMax * cbElement]
                   ]

  SmPlex must remain a static class without virtual functions because
  SmPlex allocates a memory block for itself using smos_Calloc()
***********************************************************************/
struct SM_EXPORT SmPlex     // warning variable length structure
{
    // linked list ptr
    SmPlex* m_pNext;         // m_pNext pts to a block of mem big enough for the m_pNext ptr + the data block SizedInBytes:[nMax * cbElement]
    // SmPlex::m_pNext   = pointer to next SmPlex obj in linked list
    // SmPlex::m_pNext+1 = pointer to block of memory used for any kind of data
    //                     SizedInBytes:[nMax * cbElement]
    // WARNING: The plex data block is not strongly typed!
    //          The compiler thinks its an array of ptrs to SmPlex objects
    //          but it gets used to store other types with casting.

// data ptr: points to mem block SizedInBytes:[nMax * cbElement] secretly stored after the m_pNext ptr
    void* data() { return this + 1; }  // all memory after the SmPlex memory is the
    // data block, SizeInBytes:[nMax*cbElement]

// allocate and insert at head of Plex linked list one Plex obj SizedInBytes:[sizeof(SmPlex) + nMax*cbElement] 
    static SmPlex* Create(SmPlex*& rpPlexListHead, // i/o: in = current head of SmPlex Linked List
        //      out= newly allocated SmPlex obj (NewPlex->m_pNext=old head)
        ULONG       nMax,           // in : number of elements to alloc in block
        ULONG       cbElement);    // in : size of one element in 

    // free mem for this and all subsequent SmPlex objs in this SmPlex LinkedList
    void FreeDataChain();

}; // end SmPlex

// abstract iteration position -  yet another hack to get a magic number (why not just use a magic number?)
struct __MAP_POSITION { };
typedef __MAP_POSITION* SM_MAP_POSITION;


/*******************************************************************//**
The SmMapTypeToType class as a template class

***********************************************************************/

/*******************************************************************//**
PURPOSE: SmAssocTypeToType<k,v> element declaration for objects stored 
         within the SmMapTypeToType element list

NOTES: The SmMapTypeToType class allocates SmAssocTypeToType objects
       in blocks and uses them one at a time. The SmAssocTypeToType<k,v>::m_pNext
       pointer is used in two ways by the SmMapTypeToType class. 
       
       When first allocated the m_pNext ptr is used to make a list
       of all prealloacated but NotYetUsed SmAssocTypeToType objects.

       When a preallocated SmAssocTypeToType object gets used in an
       SmMapTypeToType list, the m_pNext ptr is changed to form the
       list of all used SmAssocTypeToType objects in one hash bucket.
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
struct SmAssocTypeToType
{
  SmAssocTypeToType<KeyTYPE, ValTYPE> * m_pNext;

  KeyTYPE key;
  ValTYPE value;
  ULONG   lOrderNum;

} ; // end struct SmAssocTypeToType

/*******************************************************************//**
PURPOSE: This object is a container class which implements a mapping
   between a KeyTYPE object (typically a object pointer)
   and a ValTYPE object (typically an object pointer or a ULONG index value).
   Given a KeyTYPE object it will quickly produce the corresponding
   ValTYPE value.

NOTES:
   1. This is a very simliar implementation to the old SmMapPtrToPtr class
      with two main differences
      a.  SmMapPtrToPtr used to map void* ptrs to void* ptrs
          SmMapTypeToType maps strongly typed ptrs to ptrs

      b.  SmMapPtreToPtr only supports  OneKey-to-OneVal maps
          SmMapTypeToType supports both OneKey-to-OneVal maps and
                                        OneKey-To-ManyVals maps.
   2. to build OneKey-to-OneVal maps use:
              map[key] = val ;  // adds assoc(key,val) if it does not exist else
                                // sets assoc(key,val)->val = val
              SetAt(key,val) ;
              Append()
              Copy()
              AppendPlace() ;
   3. to build OneKey-to-ManyVal maps use:
              AddUnique(key,val) ;
              AppendAdd() ;

   4. Accessors supply result in FIFO ordering based on associations made.
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
class SM_EXPORT SmMapTypeToType : public SmObject
{
 protected:
  ULONG                                 m_nCount;         // number of all associations in all hash buckets
  ULONG                                 m_nAssocsMade;    // number of associations that have been made. Used for ordering.
  ULONG                                 m_nHashTableSize; // number of hash buckets (always nonZero)
                                                          //   default:[SM_DEF_HASH_SIZE]
  SmAssocTypeToType<KeyTYPE, ValTYPE>** m_pHashTable;     // hash buckets (1 bucket = HeadPtr to Association LinkedList )
                                                          //   NULL    = sized:[0]
                                                          //   NotNULL = sized:[m_nHashTableSize]

  SmAssocTypeToType<KeyTYPE, ValTYPE> * m_pFreeList;      // head of Preallocated-NotYetUsed Associations LinkedList
                                                          //  NULL    = no more preallocated associations available
                                                          //  NotNULL = next preAllocated association to use in Heap lists

  ULONG                                 m_nBlockSize;     // number of associations allocated in one block
  struct SmPlex                       * m_pBlocks;        // LinkedList of association memory blocks

 protected:
  SmAssocTypeToType<KeyTYPE, ValTYPE> * NewAssoc();
  void                                  FreeAssoc(SmAssocTypeToType<KeyTYPE, ValTYPE>*);
  SmAssocTypeToType<KeyTYPE, ValTYPE> * GetAssocAt(const KeyTYPE, ULONG & rHash) const ;

  // rtn assoc->val for assoc->key == key (adds assoc if it does not already exist)
  //    supports statement   // effect: without assoc for key: add assoc:[key val] to map 
  //     map[key] = val ;    //         with    assoc for key: let assoc->val = val.
  //    builds OneKey-to-OneVal maps - see AddUnique() to make OneKey-to-ManyVal maps
  ValTYPE & operator[](const KeyTYPE key);

 public:
  // Constructor
  SmMapTypeToType(ULONG            nBlockSize = 100,
                  const SmContext *cpContext  = NULL);

  // destructor
  virtual ~SmMapTypeToType();

  // add/overwrite (key, value) pairs
  void      AddUnique(KeyTYPE key, ValTYPE Value ) ;                          // can add multiple assoc sharing one key value
  void      SetAt    (KeyTYPE key, ValTYPE Value ) { (*this)[key] = Value; }  // for existing Assoc : overwrites Value in old Assoc
                                                                              // for missing  Assoc : Adds Assoc and assigns Value
  // Remove Associations from Hash list
  SmBoolean RemoveKey  (const KeyTYPE key );               // remove 1st association with matching key
  SmBoolean RemoveKeys (const KeyTYPE key );               // remove all associations with matching key
  SmBoolean RemoveAssoc(const KeyTYPE key, ValTYPE val) ;  // remove 1st association with matching <key, value>

  // single (key, value) lookups - return NULL for not found (could clash with NULL values)
  ValTYPE   GetValueAt  (const KeyTYPE       key)   const ; // fetch val given key or return NULL - FAST hash algorithm
  KeyTYPE   Get1stKeyFor(ValTYPE             value) const ; // fetch 1st key given val or return NULL - SLOW linear search algorithm
  SmBoolean GetKeysFor  (ValTYPE             value,         // fetch all keys given val               - SLOW linear search algorithm
                         SmTArray<KeyTYPE> & rKeys) const ;

  // get all unique values for given key - return FALSE for not found
  SmBoolean GetValuesFor(const KeyTYPE key, SmTArray<ValTYPE> & rValues) const ; // fetch all vals given key - FAST hash algorithm
  void      ChangeValues(ValTYPE FromValue, ValTYPE ToValue) ;                   // replace values - SLOW linear search algorithm

  // table management
  void Copy             (SmMapTypeToType<KeyTYPE, ValTYPE> & rMapCopy);  // creates OneKey-to-OneVal mappings
  void Append           (SmMapTypeToType<KeyTYPE, ValTYPE> & rMapCopy);  // same as AppendReplace
  void AppendAdd        (SmMapTypeToType<KeyTYPE, ValTYPE> & rMapCopy);  // creates OneKey-to-ManyVals mappings
  void AppendReplace    (SmMapTypeToType<KeyTYPE, ValTYPE> & rMapCopy);  // creates OneKey-to-OneVal mappings
  void RemoveAll();

  // queries
  ULONG     GetCount() const  { return m_nCount; }
  SmBoolean IsEmpty () const  { return m_nCount == 0; }
  SmBoolean IsIn    (const KeyTYPE key, ValTYPE & rValue) const { return( Lookup(key, rValue) ) ; }
  SmBoolean Lookup  (const KeyTYPE key, ValTYPE & rValue) const ;
  SmBoolean IsAssociationIn(const KeyTYPE key, ValTYPE val) const ;

  // flatten table
  void GetAllKeys  (SmTArray<KeyTYPE> & rKeys) const ;
  void GetAllValues(SmTArray<ValTYPE> & rValues) const ;
  void GetAllKeyValuePairs(SmTArray<KeyTYPE> & rKeys,
                                  SmTArray<ValTYPE> & rValues) const ;
  // iterating all (key, value) pairs
  SM_MAP_POSITION GetStartPosition() const; // returns magic number which means 1st map association in next calls

  void GetNextAssoc(SM_MAP_POSITION                     & rNextPosition,      // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association
                                                                              //      out: pNextAssoc
                    KeyTYPE                             & rKey,               // out: pThisAssoc->key
                    ValTYPE                             & rValue,             // out: pThisAssoc->value
                    ULONG                               & rlOrderNum,         // out: pThisAssoc->lOrderNum
                    ULONG                               * pOptBucket=NULL,    // out: pThisAssoc->BucketIndex
                    SmAssocTypeToType<KeyTYPE,ValTYPE> ** pOptAssocRet=NULL)  // out: pThisAssoc
                   const;

  // advanced features for derived classes
  ULONG GetHashTableSize() const { return m_nHashTableSize; }
  void  InitHashTable(ULONG hashSize, SmBoolean bAllocNow = TRUE);

  // Overridables: special non-virtual (see map implementation for details)
  // Routine used to user-provided hash keys
  size_t HashKey(const KeyTYPE key) const;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmMapTypeToType, SmObject, SmMapTypeToType_TYPE);

  // dump map knowing key and vals are derived from SmObject
  // void DumpAsObjects() const ;
  void Dump(SmBoolean bTerse) const ; // in : bTerse: TRUE = dump pair list, FALSE = don't

} ; // end class SmMapTypeToType<KeyTYPE,ValTYPE>

/*******************************************************************//**
PURPOSE: Returns magic number which means find the 1st assoc to the iterator
         methods

NOTES: GetStartPosition
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SM_MAP_POSITION SmMapTypeToType<KeyTYPE,ValTYPE>::GetStartPosition() const
{ return (m_nCount == 0) ? (SM_MAP_POSITION)NULL
                         : (SM_MAP_POSITION)BEFORE_START_MAP_POSITION;
}

/*******************************************************************//**
PURPOSE: constructor

NOTES:  SmMapTypeToType
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmMapTypeToType<KeyTYPE,ValTYPE>::SmMapTypeToType
 (ULONG             nBlockSize,   // in : number of associations for each single block allocation
  const SmContext * cpContext)    // in : context for object construction
{
  SM_ASSERT(nBlockSize > 0);

  m_nCount         = 0 ;
  m_nAssocsMade    = 0 ;
  m_nHashTableSize = SM_DEF_HASH_SIZE ;  // default size
  m_pHashTable     = NULL ;
  m_pFreeList      = NULL ;
  m_nBlockSize     = nBlockSize ;
  m_pBlocks        = NULL ;
  m_cpContext      = cpContext ;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::SmMapTypeToType constructor

/*******************************************************************//**
PURPOSE:  set rMapCopy = this

NOTES: The rMapCopy is re-initialized, then every this->Assoc is added
       to rMapCopy
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::Copy
 (SmMapTypeToType<KeyTYPE,ValTYPE> & rMapCopy)
{
    rMapCopy.RemoveAll();
    this->AppendReplace( rMapCopy );

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::Copy

/*******************************************************************//**
PURPOSE:  Append this map to another. Used in one-to-one or many-to-one 
    implementations. Any key already in use has it's value rewritten
    by the new appended pair.

NOTES: Same as Copy except that the other not re-initialized.
       This method has been deprecated. Use instead AppendReplace
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::Append
 (SmMapTypeToType<KeyTYPE,ValTYPE> & rOtherMap)
{
    AppendReplace(rOtherMap);

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::Append

/*******************************************************************//**
PURPOSE:  Append this map to another. Used in many-to-one or 
    many-to-many implementations. All unique key-values pairs are
    appended.

NOTES: Same as Copy except that the other not re-initialized.
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::AppendAdd
 (SmMapTypeToType<KeyTYPE,ValTYPE> & rOtherMap)
{
    SmTArray<KeyTYPE> sKeys;
    SmTArray<ValTYPE> sValues;
    GetAllKeyValuePairs(sKeys, sValues);
    for (ULONG ii = 0; ii < sKeys.GetSize(); ii++)
    {
        rOtherMap.AddUnique( sKeys[ii], sValues[ii]);
    }

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::AppendAdd

/*******************************************************************//**
PURPOSE:  Append this map to another. Used in one-to-one or many-to-one 
    implementations. Any key already in use has it's value rewritten
    by the new appended pair.

NOTES: Same as Copy except that the other not re-initialized.
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::AppendReplace
 (SmMapTypeToType<KeyTYPE,ValTYPE> & rOtherMap)
{
#ifdef SM_DEFINED_HASH_ORDER
  SmTArray<KeyTYPE> sKeys;
  SmTArray<ValTYPE> sValues;
  GetAllKeyValuePairs(sKeys, sValues);
  for (ULONG ii = 0; ii < sKeys.GetSize(); ii++)
    {
      rOtherMap.SetAt( sKeys[ii], sValues[ii]);
    }
#else
  KeyTYPE key;
  ValTYPE val;
  SM_MAP_POSITION pos = GetStartPosition();
  ULONG lOrderNum;
  while ( pos != NULL )
    {
      GetNextAssoc( pos,          // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association
                                  //      out: pNextAssoc
                    key,          // out: pThisAssoc->key
                    val,          // out: pThisAssoc->value
                    lOrderNum );  // out: pThisAssoc->lOrderNum
      rOtherMap.SetAt( key, val );
    }
#endif

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::AppendReplace

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline size_t SmMapTypeToType<KeyTYPE,ValTYPE>::HashKey
 (const KeyTYPE key)
 const
{
    // default identity hash - works for most primitive values
    return ((size_t)key) >> 4;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::HashKey

/*******************************************************************//**
PURPOSE: Used to force allocation of a hash table or to override the default
              hash table size of (which is fairly small)

NOTES: Side effects - m_pHashTableSize = input nHashSize argument
                      m_pHashTable     = new memory block. SizeInBytes:[nHashSize*sizeof(Assoc)]

       keyTYPE and ValType can only be base (double, ptrs,..) or static class (SmVector3d,..) objects
       because the hash table memory is managed with smos_Calloc/smos_MemSet/smos_MemCpy calls.
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::InitHashTable
 (ULONG     nHashSize,  // in : number of hash buckets
  SmBoolean bAllocNow)  // in : TRUE = call smos_Calloc for m_pHashTable, FALSE = don't
{
  SM_ASSERT(m_nCount == 0);
  SM_ASSERT(nHashSize > 0);

  // free any preexisting hash table
  if (m_pHashTable != NULL)
    {
      smos_Free(m_pHashTable);
      m_pHashTable = NULL;
    }

  if (bAllocNow)
    {
      // only okay to use smos_Calloc on base (double, ptr,.. ) static class (SmVector3d, SmExtent1d,.. ) objects.
      m_pHashTable = (SmAssocTypeToType<KeyTYPE, ValTYPE>**)smos_Calloc(1, nHashSize * sizeof(SmAssocTypeToType<KeyTYPE, ValTYPE>*));
//        memset(m_pHashTable, 0, sizeof(SmAssocTypeToType<KeyTYPE,ValTYPE>*) * nHashSize);
    }
  m_nHashTableSize = nHashSize;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::InitHashTable

/*******************************************************************//**
PURPOSE: Free all SmMapTypeToType memory

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::RemoveAll()
{
  if (m_pHashTable != NULL)
    {
      // free hash table
      smos_Free(m_pHashTable); // delete the HashTable
      m_pHashTable = NULL;
    }

  // reinit counts
  m_nCount = 0;
  m_nAssocsMade = 0;
  m_pFreeList = NULL;

  // delete all association block memory
  if( m_pBlocks != NULL ) {
      m_pBlocks->FreeDataChain();
      m_pBlocks = NULL;
  }

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::RemoveAll

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmMapTypeToType<KeyTYPE,ValTYPE>::~SmMapTypeToType()
{
    RemoveAll();
    SM_ASSERT(m_nCount == 0);

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::~SmMapTypeToULONG destructor

/////  Assoc helpers  //////////////////////////////////////////////////////

/*******************************************************************//**
PURPOSE: returns pointer to next Association to add to Hash tables

NOTES: Associations are allocated in blocks.  If m_pFreeList is NULL,
       a new block of associations is allocated.  Returns the fist preallocated
       NotYetUsed assoc after removing that from the m_pFreeList.
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmAssocTypeToType<KeyTYPE,ValTYPE> * SmMapTypeToType<KeyTYPE,ValTYPE>::NewAssoc()
{
  // when all the assoc on the map->m_pFreeList have been used - add another block of SmAssociations to the FreeList array
  if (m_pFreeList == NULL)
    {
      // add another block
      SmPlex* newBlock = SmPlex::Create(m_pBlocks,                                    // i/o: Head of SmPlex Linked List
                                        m_nBlockSize,                                 // in : Number of elems wanted in block
                                        sizeof(SmAssocTypeToType<KeyTYPE, ValTYPE>)); // in : size of 1 elem in bytes

      // newBlock->data() = mem for a block of m_nBlockSize number of SmAssocTypeToType
     
      // set pAssoc to point to last SmAssociation in the newBlock->Data mem block
      SmAssocTypeToType<KeyTYPE, ValTYPE> * pAssoc = (SmAssocTypeToType<KeyTYPE, ValTYPE> *) newBlock->data();
      pAssoc += m_nBlockSize - 1;

      // in reverse order (easier debugging)
 
      // for every SmAssociation in newBlock except the first one
      for (long i = m_nBlockSize-1; i > 0; i--, pAssoc--)
        {
          // add currAssoc to head of the Map FreeList
          pAssoc->m_pNext = m_pFreeList;
          m_pFreeList = pAssoc;
        }

      // add in the first one
      pAssoc->m_pNext = m_pFreeList;
      m_pFreeList = pAssoc;
    }
  SM_ASSERT(m_pFreeList != NULL);  // we must have something

  // arrive here when the m_pFreeList array of SmAssoications ready to be used is known to have entries

  // let return Assoc = 1st Association out of the m_pFreeList array
  SmAssocTypeToType<KeyTYPE, ValTYPE> * pAssoc = m_pFreeList;

  // remove Assoc from the free list
  m_pFreeList = m_pFreeList->m_pNext;
  pAssoc->m_pNext = NULL ;

  // increment count of Associations currently in the map
  m_nCount++;
  SM_ASSERT(m_nCount > 0);       // make sure we don't overflow
  SM_ASSERT(m_nAssocsMade >= 0); // make sure we don't overflow

  // init return association member values
  pAssoc->key       = 0;
  pAssoc->value     = 0;
  pAssoc->lOrderNum = m_nAssocsMade; // set return assoc number = the order in which assoc were added to the map

  // increment the count of all associations every added to the map 
  m_nAssocsMade++;

  // all done - return uninit mem for 1 association 
  return pAssoc;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::NewAssoc

/*******************************************************************//**
PURPOSE: Place given pAssoc back on the FreeList

NOTES: Could clear the Assoc member values - but that gets done
       when the Assoc gets picked for reuse with a call to NewAssoc
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::FreeAssoc(SmAssocTypeToType<KeyTYPE,ValTYPE> * pAssoc)
{
  // insert the Association back into the head of the FreeList
  pAssoc->m_pNext = m_pFreeList;
  m_pFreeList     = pAssoc;

  // decrement the used associations count number
  SM_ASSERT(m_nCount > 0);  // make sure we don't underflow
  m_nCount--;

  // if no more elements, cleanup completely
  if(m_nCount == 0)
    { RemoveAll() ; }

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::FreeAssoc

/*******************************************************************//**
PURPOSE: find 1st association for given key (or return NULL for not found)

NOTES: returns NULL when m_pHashTable has not yet been allocated
       and when key is not in HashBucket[nHash]
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmAssocTypeToType<KeyTYPE,ValTYPE> * SmMapTypeToType<KeyTYPE,ValTYPE>::GetAssocAt
 (const KeyTYPE key,
  ULONG       & nHash)
  const
{
  // ULONG lHash = ((ULONG)key) >> 4;  // GWC: replaced this line
  size_t lHash = HashKey(key) ;        //      with this one
  nHash = lHash % m_nHashTableSize;

  if(m_pHashTable == NULL)
    { return NULL; }

  // see if it exists in just the one hash bucket[nHash] - quick
  SmAssocTypeToType<KeyTYPE,ValTYPE>* pAssoc;
  for(pAssoc = m_pHashTable[nHash];
      pAssoc != NULL;
      pAssoc = pAssoc->m_pNext)
    {
      if(pAssoc->key == key)
        { return pAssoc; }
    }

  // arrive here when key is not in HashBucket[nHash]
  return NULL;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::GetAssocAt

/*******************************************************************//**
PURPOSE: find the Value in the 1st association for given key
          (or return 0 for not found)

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline ValTYPE SmMapTypeToType<KeyTYPE,ValTYPE>::GetValueAt // eff: retrieve values by hash keys
  (const KeyTYPE key)                                       // in : key to hash table
 const
{
  if (m_pHashTable == NULL)
    { return 0; }

  size_t nHash = HashKey(key) % m_nHashTableSize;

  // look for matching Assoc in HashBucket[nHash]
  SmAssocTypeToType<KeyTYPE, ValTYPE>* pAssoc;
  for(pAssoc  = m_pHashTable[nHash];
      pAssoc != NULL;
      pAssoc  = pAssoc->m_pNext)
    {
      // when we find the key
      if (pAssoc->key == key)
        {
          // return the associated value
          return pAssoc->value ;
        }
    } // end iter entries in this hash list

  // arrive here when key is not in table
  return 0;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::GetValueAt

/*******************************************************************//**
PURPOSE: find all unique Values for given key

NOTES: Return TRUE  = one or more associations found for given key
              FALSE = no associations for for given key

       Key is not necessarily a pointer. Key may be 0
       Originally this was written assuming that the key was always a ptr
       Therefore we checked for null ptrs
       But if the key is a ULONG, for example, a zero value should be possible
       In fact, a NULL ptr should also be valid
       So the check for a zero key value has been removed. 
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmBoolean SmMapTypeToType<KeyTYPE,ValTYPE>::GetValuesFor // eff: retrieve all values by hash key
  (const KeyTYPE key,                                           // in : key to hash table
   SmTArray<ValTYPE> & rValues)                                 // in : all unique values associated with key
 const
{
#ifdef SM_DEFINED_HASH_ORDER
  // init output
  rValues.SetSize(m_nAssocsMade) ;
  rValues.SetAll(NULL) ;

  // Locals
  ULONG ii, i1 = 0;
#else
  // init output
  rValues.ReSet( );
#endif

  // no work - no hash table
  if (m_pHashTable == NULL)
    { return 0; }

  size_t nHash = HashKey(key) % m_nHashTableSize;

// look for matching Assocs in HashBucket[nHash]
  SmAssocTypeToType<KeyTYPE, ValTYPE>* pAssoc;
  for(pAssoc  = m_pHashTable[nHash];
      pAssoc != NULL;
      pAssoc  = pAssoc->m_pNext)
    {
      // when we find the key
      if (pAssoc->key == key)
        {
          // Store the value
#ifdef SM_DEFINED_HASH_ORDER
          rValues.SetAt(pAssoc->lOrderNum, pAssoc->value) ;
#else
          rValues.AddUnique( pAssoc->value );
#endif
          // rValues.Add(pAssoc->value) ;
        }
    } // end iter entries in this hash list

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump(TRUE) ; // TRUE = Dump object pairs
      rValues.Dump() ;
    }
#endif // SM_DEBUG_CODE

#ifdef SM_DEFINED_HASH_ORDER
  if (m_nCount < m_nAssocsMade)
  {
      // look for first NULL
      for (ii = 0; ii < rValues.GetSize(); ii++)
      {
          if (rValues[ii] == NULL)
          { break ; }
      }
      // compress the array after the first NULL
      for (i1 = ii; ii < rValues.GetSize(); ii++)
      {
          if (rValues[ii] != NULL)
          {
              rValues.SetAt(i1, rValues[ii]);
              i1++;
          }
      }
      rValues.SetSize(i1) ;
  } // end NULL entry check

  // Remove duplicates
  rValues.RemoveDuplicates();
#endif

  // return TRUE when associations were found
  return( rValues.GetSize() > 0) ;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::GetValuesFor

/*******************************************************************//**
PURPOSE: Change all <key, FromValue> associations to <key, ToValue>

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::ChangeValues
 (ValTYPE FromValue,  // in : change [key FromValue] assocs 
  ValTYPE ToValue)    // in : to     [key ToValue]
{
  SmAssocTypeToType<KeyTYPE,ValTYPE> * pAssocRet = NULL ;
  KeyTYPE key;
  ValTYPE val;
  ULONG   lOrderNum;

  SM_MAP_POSITION pos = GetStartPosition();  // magic number meaning get 1st assoc
  while (pos != NULL)
    {
      GetNextAssoc(pos,          // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association
                                 //      out: pNextAssoc
                   key,          // out: pThisAssoc->key
                   val,          // out: pThisAssoc->value
                   lOrderNum,    // out: pThisAssoc->lOrderNum
                   NULL,         // out: 
                   &pAssocRet);  // out: pThisAssoc
      if(val == FromValue)
        {
          pAssocRet->value = ToValue ;
        }
    }

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::ChangeValues

/*******************************************************************//**
PURPOSE: Return TRUE when Assoc<key val> is in list, else return FALSE

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmBoolean SmMapTypeToType<KeyTYPE,ValTYPE>::IsAssociationIn // eff: retrieve all values by hash key
  (const KeyTYPE key,                                              // in : key to hash table
   ValTYPE       val)                                              // in : search for value
 const
{
  // no work - no hash table
  if (m_pHashTable == NULL)
    { return FALSE ; }

  size_t nHash = HashKey(key) % m_nHashTableSize;

  // look for matching Assocs in HashBucket[nHash]
  SmAssocTypeToType<KeyTYPE, ValTYPE>* pAssoc;
  for(pAssoc  = m_pHashTable[nHash];
      pAssoc != NULL;
      pAssoc  = pAssoc->m_pNext)
    {
      // when we find the key
      if (pAssoc->key == key)
        {
          // check for value
          if(val == pAssoc->value)
            { return TRUE ; }
        }
    } // end iter entries in this hash list

  // arrive here when association is not already in the list
  return( FALSE ) ;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::IsAssociationIn

/////////////////////////////////////////////////////////////////////////////

/*******************************************************************//**
PURPOSE: return TRUE when key matches existing Assoc, else return FALSE

NOTES: sets rValue = FoundAssoc->Value when TRUE is returned
            rValue = 0 when FALSE is returned
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmBoolean SmMapTypeToType<KeyTYPE,ValTYPE>::Lookup
 (const KeyTYPE key,    // in : target key
  ValTYPE     & rValue) // out: matched Assoc->Value or 0 if not matched
 const
{
  ULONG nHash;
  SmAssocTypeToType<KeyTYPE, ValTYPE> * pAssoc = GetAssocAt(key, nHash);

  // return FALSE when matching Assoc is not in Map
  if (pAssoc == NULL)
    {
      rValue = 0 ;
      return FALSE;  // not in map
    }

  // return TRUE when matching Assoc is in map
  rValue = pAssoc->value ;
  return TRUE;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::Lookup

/*******************************************************************//**
PURPOSE: returns ref to Assoc->Val of 1st Assoc that matches key
         or makes a new one to return.

NOTES: If no Assoc matches key - adds a new Assoc in the hash table for
 this key and returns a ref to that one's Val with an unit value of zero.
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline ValTYPE & SmMapTypeToType<KeyTYPE,ValTYPE>::operator[]
 (const KeyTYPE key)
{
  ULONG nHash;
  SmAssocTypeToType<KeyTYPE,ValTYPE>* pAssoc;
  if ((pAssoc = GetAssocAt(key, nHash)) == NULL)
    {
      if(m_pHashTable == NULL)
        { InitHashTable(m_nHashTableSize) ; }

      // it doesn't exist, add a new Association
      pAssoc = NewAssoc();

      pAssoc->key = key;
      // leave pAssoc->Val uninitialized (zero)

      // insert NewAssoc into head of hash bucket[nHash] Assoc LinkedList
      pAssoc->m_pNext = m_pHashTable[nHash];
      m_pHashTable[nHash] = pAssoc;
    }

  // all done
  return pAssoc->value;  // return new reference

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::operator[]

/*******************************************************************//**
PURPOSE: Adds association <key val> if it is not already in the list

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::AddUnique
 (const KeyTYPE key,
  ValTYPE       val)
{
  // no work - assoc<key val> already exists
  if(IsAssociationIn(key, val))
    { return ; }

  // arrive here when Assoc<key val> doesn't exist, add new Association

  // get the hash bucket index
  size_t lHash = HashKey(key) ;        //      with this one
  ULONG  nHash = lHash % m_nHashTableSize;

  // init HashTable when needed
  if(m_pHashTable == NULL)
    { InitHashTable(m_nHashTableSize) ; }

  // make and set the new association
  SmAssocTypeToType<KeyTYPE,ValTYPE> * pAssoc = NewAssoc();
  pAssoc->key   = key ;
  pAssoc->value = val ;

  // insert NewAssoc into head of hash bucket[nHash] Assoc LinkedList
  pAssoc->m_pNext     = m_pHashTable[nHash];
  m_pHashTable[nHash] = pAssoc;

  // all done

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::AddUnique

/*******************************************************************//**
PURPOSE: Remove 1st Association with matching key from Hash list

NOTES: return TRUE when association was removed, else return FALSE
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmBoolean SmMapTypeToType<KeyTYPE,ValTYPE>::RemoveKey
 (const KeyTYPE key)
{
  if (m_pHashTable == NULL)
    { return FALSE; } // nothing in the table

  // locals - ptr to hash buck[nHash] and ptr to Assoc within that hash buckt
  SmAssocTypeToType<KeyTYPE,ValTYPE>  * pAssoc;
  SmAssocTypeToType<KeyTYPE,ValTYPE> ** ppAssocPrev;

  // ptr to Previous think pointing to target Assoc - starts as ptr to HashBucket[nHash] head Association
  ppAssocPrev = &m_pHashTable[HashKey(key) % m_nHashTableSize];

  // for every Assoc within HashBucket[nHash]
  for (pAssoc = *ppAssocPrev; pAssoc != NULL; pAssoc = pAssoc->m_pNext)
    {
      // When Assoc matches given key
      if (pAssoc->key == key)
        {
          // remove it
          *ppAssocPrev = pAssoc->m_pNext;  // remove from list
          FreeAssoc(pAssoc);
          return TRUE;
        }

      // increment the ppAssocPrev pointer before pAssoc gets incremented
      ppAssocPrev = &pAssoc->m_pNext;
    }

  // all done - all Associations with matching keys have been removed from HashBucket[nHash]
  return FALSE;  // not found

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::RemoveKey

/*******************************************************************//**
PURPOSE: Remove all Associations with matching key from Hash list

NOTES: return TRUE when association was removed, else return FALSE
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmBoolean SmMapTypeToType<KeyTYPE,ValTYPE>::RemoveKeys
 (const KeyTYPE key)
{
  if (m_pHashTable == NULL)
    { return FALSE; } // nothing in the table

  // locals - ptr to hash buck[nHash] and ptr to Assoc within that hash buckt
  SmBoolean bRtn = FALSE ;
  SmAssocTypeToType<KeyTYPE,ValTYPE>  * pAssoc;
  SmAssocTypeToType<KeyTYPE,ValTYPE> ** ppAssocPrev;

  // ptr to Previous think pointing to target Assoc - starts as ptr to HashBucket[nHash] head Association
  ppAssocPrev = &m_pHashTable[HashKey(key) % m_nHashTableSize];

  // for every Assoc within HashBucket[nHash]
  for (pAssoc = *ppAssocPrev; pAssoc != NULL; pAssoc = pAssoc->m_pNext)
    {
      // When Assoc matches given key
      if (pAssoc->key == key)
        {
          // remove it
          *ppAssocPrev = pAssoc->m_pNext;  // remove from list
          FreeAssoc(pAssoc);
          bRtn = TRUE;
        }

      // increment the ppAssocPrev pointer before pAssoc gets incremented
      ppAssocPrev = &pAssoc->m_pNext;
    }

  // all done - all Associations with matching keys have been removed from HashBucket[nHash]
  return bRtn ;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::RemoveKeys

/*******************************************************************//**
PURPOSE: Remove 1st Association matching <key,value> from Hash list

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmBoolean SmMapTypeToType<KeyTYPE,ValTYPE>::RemoveAssoc
 (const KeyTYPE key, // in :
  ValTYPE       val) // in :
{
  if (m_pHashTable == NULL)
    { return FALSE; } // nothing in the table

  // locals - ptr to hash buck[nHash] and ptr to Assoc within that hash buckt
  SmAssocTypeToType<KeyTYPE,ValTYPE>  * pAssoc;
  SmAssocTypeToType<KeyTYPE,ValTYPE> ** ppAssocPrev;

  // ptr to Previous think pointing to target Assoc - starts as ptr to HashBucket[nHash] head Association
  ppAssocPrev = &m_pHashTable[HashKey(key) % m_nHashTableSize];

  // for every Assoc within HashBucket[nHash]
  for (pAssoc = *ppAssocPrev; pAssoc != NULL; pAssoc = pAssoc->m_pNext)
    {
      // When Assoc matches given <key, value>
      if (   pAssoc->key   == key
          && pAssoc->value == val)
        {
          // remove it
          *ppAssocPrev = pAssoc->m_pNext;  // remove from list
          FreeAssoc(pAssoc);
          return TRUE;
        }

      // increment the ppAssocPrev pointer before pAssoc gets incremented
      ppAssocPrev = &pAssoc->m_pNext;
    }

  // all done - all Associations with matching keys have been removed from HashBucket[nHash]
  return FALSE;  // not found

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::RemoveAssoc

//////  Iterating  //////////////////////////////////////////////////////////

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::GetNextAssoc
  (SM_MAP_POSITION                     & rNextPosition, // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association)
                                                        //      out: pNextAssoc
   KeyTYPE                             & rKey,          // out: pThisAssoc key
   ValTYPE                             & rValue,        // out: pThisAssoc value
   ULONG                               & rlOrderNum,    // out: pThisAssoc order number
   ULONG                               * pOptBucket,    // out: pThisAssoc->BucketIndex, NULL to ignore, default:[NULL]
   SmAssocTypeToType<KeyTYPE,ValTYPE> ** pOptAssocRet)  // out: pThisAssoc, NULL to ignore, default:[NULL]
  const
{
  //check state - never call on empty map
  SM_ASSERT(m_pHashTable != NULL); 

  // rNextPosition on input == ThisAssoc (or magic number)
  SmAssocTypeToType<KeyTYPE,ValTYPE> * pThisAssoc = (SmAssocTypeToType<KeyTYPE,ValTYPE>*)rNextPosition;
  SM_ASSERT(pThisAssoc != NULL);

  // if ThisAssoc == magic number - let pThisAssoc = 1st map Assoc
  if (pThisAssoc == (SmAssocTypeToType<KeyTYPE,ValTYPE>*)BEFORE_START_MAP_POSITION)
    {
      // find the first association
      for (ULONG nBucket = 0; nBucket < m_nHashTableSize; nBucket++)
        {
          if ((pThisAssoc = m_pHashTable[nBucket]) != NULL)
            {
              if(pOptBucket) { *pOptBucket = nBucket ; }
              break;
            }
        }
      SM_ASSERT(pThisAssoc != NULL);  // must find something
    }

  // arrive here when pThisAssoc == curr tgt association

  // find next association (either next assoc in this bucket
  //                        or     1st assoc of next used bucket
  //                        or     NULL for no more next associations)
  SmAssocTypeToType<KeyTYPE,ValTYPE>* pNextAssoc = pThisAssoc->m_pNext ;
  if (pNextAssoc == NULL)
    {
      // go to next bucket
      for(ULONG nBucket = (HashKey(pThisAssoc->key) % m_nHashTableSize) + 1;
          nBucket < m_nHashTableSize;
          nBucket++)
        {
          pNextAssoc = m_pHashTable[nBucket] ;
          if (pNextAssoc != NULL)
            { break; }
        }
    }

  // output pNextAssoc
  rNextPosition = SM_REINTERPRET_CAST(SM_MAP_POSITION, pNextAssoc);

  // output pThisAssoc data
  rKey         = pThisAssoc->key ;
  rValue       = pThisAssoc->value ;
  rlOrderNum   = pThisAssoc->lOrderNum ;
  if(pOptBucket)   { *pOptBucket = HashKey(rKey) % m_nHashTableSize; }
  if(pOptAssocRet) { *pOptAssocRet = pThisAssoc ; }

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::GetNextAssoc

/*******************************************************************//**
PURPOSE: Get all keys in a FIFO order

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::GetAllKeys
 (SmTArray<KeyTYPE> & rKeys)
 const
{
    KeyTYPE key;
    ValTYPE val;

#ifdef SM_DEFINED_HASH_ORDER
    ULONG   ii, i1, lOrderNum;
    rKeys.SetSize(m_nAssocsMade);
    rKeys.SetAll(NULL);

    // load rKeys with every Assoc ordered by OrderNumber
    SM_MAP_POSITION pos = GetStartPosition();
    while (pos != NULL)
    {
        GetNextAssoc(pos, key, val, lOrderNum);
        rKeys.SetAt(lOrderNum, key);
    }

    // when there are gaps in the rKeys array
    if (m_nCount < (int) m_nAssocsMade)
    {
        // look for first NULL
        for (ii = 0; ii < rKeys.GetSize(); ii++)
        {
            if (rKeys[ii] == NULL)
            { break; }
        }

        // compress the array after the first NULL
        for (i1 = ii; ii < rKeys.GetSize(); ii++)
        {
            if (rKeys[ii] != NULL)
            {
                rKeys.SetAt(i1, rKeys[ii]);
                i1++;
            }
        }
        rKeys.SetSize(m_nCount);
    } // end NULL entry check
#else
    ULONG lOrderNum;
    rKeys.ReSet();
    SM_MAP_POSITION pos = GetStartPosition();
    while ( pos != NULL )
    {
        GetNextAssoc( pos,          // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association
                                    //      out: pNextAssoc
                      key,          // out: pThisAssoc->key
                      val,          // out: pThisAssoc->value
                      lOrderNum );  // out: pThisAssoc->lOrderNum
        rKeys.Add( key );
    }
#endif

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::GetAllKeys

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline KeyTYPE SmMapTypeToType<KeyTYPE,ValTYPE>::Get1stKeyFor
 (ValTYPE value)
 const
{
  KeyTYPE key;
  ValTYPE val;
  ULONG   lOrderNum;
  
  SM_MAP_POSITION pos = GetStartPosition();
  while (pos != NULL)
    {
      GetNextAssoc(pos,           // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association
                                  //      out: pNextAssoc
                   key,           // out: pThisAssoc->key
                   val,           // out: pThisAssoc->value
                   lOrderNum);    // out: pThisAssoc->lOrderNum
      if(val == value) return(key) ;
    }
  return(NULL) ;
  
} // end SmMapTypeToType<KeyTYPE,ValTYPE>::Get1stKeyFor

/*******************************************************************//**
PURPOSE: Fetch all keys for given key in FIFO order

NOTES: return TRUE when many keys were found, else return FALSE
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmBoolean SmMapTypeToType<KeyTYPE,ValTYPE>::GetKeysFor
 (ValTYPE             value,  // in :
  SmTArray<KeyTYPE> & rKeys)  // out:
 const
{
    // locals
    KeyTYPE key;
    ValTYPE val;
    ULONG lOrderNum;
#ifdef SM_DEFINED_HASH_ORDER
    ULONG   ii, i1 = 0;

    // init output
    rKeys.SetSize(m_nAssocsMade) ;
    rKeys.SetAll(NULL) ;

    // slow iter
    SM_MAP_POSITION pos = GetStartPosition();
    while (pos != NULL)
    {
        GetNextAssoc(pos, key, val, lOrderNum);
        if(val == value)
        { rKeys.SetAt(lOrderNum, key) ; }
    }

    // when there are gaps in the rKeys array
    if (m_nCount < (int) m_nAssocsMade)
    {
        // look for first NULL
        for (ii = 0; ii < rKeys.GetSize(); ii++)
        {
            if (rKeys[ii] == NULL)
            { break; }
        }

        // compress the array after the first NULL
        for (i1 = ii; ii < rKeys.GetSize(); ii++)
        {
            if (rKeys[ii] != NULL)
            {
                rKeys.SetAt(i1, rKeys[ii]);
                i1++;
            }
        }
        rKeys.SetSize(i1);
    } // end NULL entry check
#else
    // init output
    rKeys.ReSet();
    rKeys.SetDataSize( m_nCount );

    // slow iter
    SM_MAP_POSITION pos = GetStartPosition();
    while ( pos != NULL )
    {
        GetNextAssoc( pos,         // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association
                                   //      out: pNextAssoc
                      key,         // out: pThisAssoc->key
                      val,         // out: pThisAssoc->value
                      lOrderNum);  // out: pThisAssoc->lOrderNum
        if ( val == value )
        { rKeys.Add( key ); }
    }
#endif

    // all done
    return( rKeys.GetSize() > 0 ) ;

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::GetKeysFor

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::GetAllValues
 (SmTArray<ValTYPE> & rValues)
 const
{
#ifdef SM_DEFINED_HASH_ORDER
    KeyTYPE key;
    ValTYPE val;
    ULONG   ii, i1, lOrderNum;

    rValues.SetSize(m_nAssocsMade);
    rValues.SetAll(NULL);

    // load rValues with every Assoc ordered by OrderNumber
    SM_MAP_POSITION pos = GetStartPosition();
    while (pos != NULL)
    {
        GetNextAssoc(pos, key, val, lOrderNum);
        rValues.SetAt(lOrderNum, val);
    }

    // when there are gaps in the rValues array
    if (m_nCount < (int) m_nAssocsMade)
    {
        // look for first NULL
        for (ii = 0; ii < rValues.GetSize(); ii++)
        {
            if (rValues[ii] == NULL)
            { break; }
        }

        // compress the array after the first NULL
        for (i1 = ii; ii < rValues.GetSize(); ii++)
        {
            if (rValues[ii] != NULL)
            {
                rValues.SetAt(i1, rValues[ii]);
                i1++;
            }
        }
        rValues.SetSize(m_nCount);
    } // end NULL entry check
#else
    rValues.ReSet();
    rValues.SetDataSize( m_nCount );

    KeyTYPE key;
    ValTYPE val;
    ULONG lOrderNum;

    SM_MAP_POSITION pos = GetStartPosition();
    while ( pos != NULL )
      {
        GetNextAssoc( pos,         // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association
                                   //      out: pNextAssoc
                      key,         // out: pThisAssoc->key
                      val,         // out: pThisAssoc->value
                      lOrderNum ); // out: pThisAssoc->lOrderNum
        rValues.Add( val );
      }
#endif
} // end SmMapTypeToType<KeyTYPE,ValTYPE>::GetAllValues

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::GetAllKeyValuePairs
  (SmTArray<KeyTYPE> & rKeys,
   SmTArray<ValTYPE> & rValues)
 const
{
    // Locals
    KeyTYPE key;
    ValTYPE val;
    ULONG   lOrderNum;

#ifdef SM_DEFINED_HASH_ORDER
    ULONG ii, i1;
    rValues.SetSize(m_nAssocsMade);
    rValues.SetAll(NULL);
    rKeys.SetSize(m_nAssocsMade);
    rKeys.SetAll(NULL);

    // load rValues and rKeys with every Assoc ordered by OrderNumber
    SM_MAP_POSITION pos = GetStartPosition();
    while (pos != NULL)
    {
        GetNextAssoc(pos, key, val, lOrderNum);
        rValues.SetAt(lOrderNum, val);
        rKeys.SetAt(lOrderNum, key);
    }

    // when there are gaps in the rValues array
    if (m_nCount < (int) m_nAssocsMade)
    {
        // look for first NULL
        for (ii = 0; ii < rValues.GetSize(); ii++)
        {
            if (rValues[ii] == NULL)
            { break; }
        }

        // compress the array after the first NULL
        for (i1 = ii; ii < rValues.GetSize(); ii++)
        {
            if (rValues[ii] != NULL)
            {
                rValues.SetAt(i1, rValues[ii]);
                rKeys.SetAt(i1, rKeys[ii]);
                i1++;
            }
        }
        rValues.SetSize(m_nCount);
        rKeys.SetSize(m_nCount);
    } // end NULL entry check

#else
    rKeys.ReSet();
    rValues.ReSet();
    rKeys.SetDataSize( m_nCount );
    rValues.SetDataSize( m_nCount );

    // load rValues and rKeys with every Assoc 
    SM_MAP_POSITION pos = GetStartPosition();
    while (pos != NULL)
    {
        GetNextAssoc(pos,         // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association
                                  //      out: pNextAssoc
                     key,         // out: pThisAssoc->key
                     val,         // out: pThisAssoc->value
                     lOrderNum);  // out: pThisAssoc->lOrderNum
        rValues.Add( val );
        rKeys.Add( key );
    }
#endif

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::GetAllKeyValuePairs


/////////////////////////////////////////////////////////////////////////////
// Diagnostics

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline SmBoolean SmMapTypeToType<KeyTYPE, ValTYPE>::IsKindOf( SM_TYPE t ) const
{
  return ((SmMapTypeToType_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));

} // end SmMapTypeToType<TYPE>::IsKindOf


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::Dump() const
{
  Dump(TRUE) ; // dump pairs
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class KeyTYPE, class ValTYPE>
inline void SmMapTypeToType<KeyTYPE,ValTYPE>::Dump
 (SmBoolean bTerse)    // in : TRUE = omit pair list
                       //      FALSE= list pairs
 const
{

  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // get table statistics
  ULONG lNumUsedBuckets=0 ;              // num of used buckets
  ULONG lAvgBucketLength=0 ;             // (num of elements) / (num of used buckets)
  ULONG lMinBucketLength=SM_BIG_ULONG ;  // min num of elements in a used bucket
  ULONG lMaxBucketLength=0 ;             // max num of elements in a used bucket

  // for empty maps
  if(GetCount() == 0)
    {
      // dump header
      smos_sprintf(sBuff,_T("\nSmMapTypeToType - EMPTY: NumElems[0], NumBuckets[%ld], UsedBuckets[0]"),
                 m_nHashTableSize);  // num of buckets
      smos_WriteBuffer(sBuff);

      // dump base object
      SmObject::Dump();
    }
  else // for nonEmpty Maps
    {
      // for every possible bucket
      for(size_t ii=0; ii < m_nHashTableSize; ii++)
        {
          SmAssocTypeToType<KeyTYPE,ValTYPE> *pBucket = m_pHashTable[ii] ;

          // skip empty buckets
          if(pBucket == NULL)
            { continue ; }

          // Bucket length
          ULONG lBucketLength = 1 ;
          for(pBucket=pBucket->m_pNext; pBucket!=NULL; pBucket=pBucket->m_pNext)
            { lBucketLength++ ; }

          // save statistics
          lNumUsedBuckets++ ;
          lAvgBucketLength += lBucketLength ;
          if(lMinBucketLength > lBucketLength) lMinBucketLength = lBucketLength ;
          if(lMaxBucketLength < lBucketLength) lMaxBucketLength = lBucketLength ;
        } // end iter every bucket

      SM_ASSERT(lAvgBucketLength == m_nCount) ;
      if ( lNumUsedBuckets < 1 ) { lNumUsedBuckets = 1; }  // Just for zero-divide.
      lAvgBucketLength /= lNumUsedBuckets ;

      // dump header
      smos_sprintf(sBuff,_T("\nSmMapTypeToType: NumElems[%ld], NumBuckets[%ld], UsedBuckets[%ld], AvgUsedLen[%ld], MinUsedLen[%ld], MaxUsedLen[%ld]"),
                 m_nCount,          // num of elements
                 m_nHashTableSize,  // num of buckets
                 lNumUsedBuckets,   // num of used buckets
                 lAvgBucketLength,  // (num of elements) / (num of used buckets)
                 lMinBucketLength,  // min num of elements in a used bucket
                 lMaxBucketLength); // max num of elements in a used bucket
      smos_WriteBuffer(sBuff);

      // dump base object
      SmObject::Dump();

      // when asked - dump element list
      if(bTerse)
        {
#ifdef SM_DEFINED_HASH_ORDER
          SmTArray<KeyTYPE> sKeys;
          SmTArray<ValTYPE> sVals;
          GetAllKeyValuePairs( sKeys, sVals );

          for ( ULONG ii = 0; ii < sKeys.GetSize(); ii++ )
          {
              // Dump in format "[key] -> value"
              KeyTYPE key = sKeys[ii];
              ValTYPE val = sVals[ii];

#ifdef _UNICODE
			  std::wostringstream myOut;
#else
              std::ostringstream myOut ;
#endif // _UNICODE
              myOut << _T("\t[0x")     << key
                    << _T("] = 0x")    << val
                    << _T("] \n") ;
#else
          // Dump in format "[key] -> value"
          KeyTYPE key;
          ValTYPE val;   
          ULONG   lOrderNum;
          ULONG   lBucket ;

          SM_MAP_POSITION pos = GetStartPosition();
          while (pos != NULL)
            {
              GetNextAssoc(pos,          // i/o: in : pThisAssoc (or magic number BEFORE_START_MAP_POSITION == 1st map association
                                         //      out: pNextAssoc
                           key,          // out: pThisAssoc->key
                           val,          // out: pThisAssoc->value
                           lOrderNum,    // out: pThisAssoc->lOrderNum
                           &lBucket);    // out: pThisAssoc->BucketIndex

#ifdef _UNICODE
			  std::wostringstream myOut;
#else
              std::ostringstream myOut ;
#endif // _UNICODE
              myOut << _T("\t[0x")     << key
                    << _T("] = 0x")    << val
                    << _T(", Bucket[") << lBucket
                    << _T("] \n") ;
#endif

#ifdef _UNICODE
               std::wstring tmp1 = myOut.str();
			   const wchar_t *tmpStr = tmp1.c_str();
               size_t length = std::wcslen(tmpStr) + 1;
#else
              std::string tmp1 = myOut.str();
              const char* tmpStr = tmp1.c_str();
              size_t length = sizeof(tmpStr);
#endif // _UNICODE
              

              // Create a wide character string and initialize
              wchar_t * w = new wchar_t[length];  // mem leak?
              for (size_t jj = 0; jj < length; jj++) {
                  w[jj] = tmpStr[jj];
                }


#ifdef _UNICODE
              smos_WStrCpy(sBuff, SM_TBLOCK_SIZE, w) ;
#else
              smos_WStrCpy(sBuff, SM_TBLOCK_SIZE, tmpStr) ;
#endif

#ifdef _UNICODE
              std::wostringstream myOutForFile ;
#else
              std::ostringstream myOutForFile ;
#endif // _UNICODE
              myOutForFile << _T("\t[")       << ((key) ? _T("notNULL") : _T("NULL"))
                           << _T("] = ")      << ((val) ? _T("notNULL") : _T("NULL"))
                           << _T("] \n") ;

#ifdef _UNICODE
              std::wstring tmp2 = myOutForFile.str();
			  const wchar_t *tmpStrForFile = tmp2.c_str();
			  length = std::wcslen( tmpStrForFile ) + 1;

#else
              std::string tmp2 = myOutForFile.str();
              const char* tmpStrForFile = tmp2.c_str();
              length = sizeof(tmpStrForFile);

#endif // _UNICODE

              // Create a wide character string and initialize
              wchar_t * wForFile = new wchar_t[length];  // mem leak?
              for(size_t jj = 0; jj < length; jj++) {
                  wForFile[jj] = tmpStrForFile[jj];
                }

#ifdef _UNICODE
              smos_WStrCpy(sBuffForFile, SM_TBLOCK_SIZE, wForFile) ;
#else
              smos_WStrCpy(sBuffForFile, SM_TBLOCK_SIZE, tmpStrForFile) ;
#endif

              // cOut << "\t[0x" << key << "] = 0x" << val << ", Bucket[" << lBucket << "] \n" ;

              // smos_sprintf(sBuff,_T("\t[0x%p] = 0x%ps, Bucket[%ld]\n"), key, val, lBucket);
              // smos_sprintf(sBuffForFile,_T("\t[%s] = %s, Bucket[%ld]\n"),key ? _T("notNULL") : _T("NULL"),val ? _T("notNULL") : _T("NULL"), lBucket);

              smos_WriteBuffer(sBuff, sBuffForFile);
            } // end iter every used map entry
        } // end Terse = TRUE check
    } // end nonEmpty map check

} // end SmMapTypeToType<KeyTYPE,ValTYPE>::Dump

// /*******************************************************************//**
// PURPOSE: Dump Map when values and keys are known to be derived
//             from SmObject.
//
// NOTES:
// ***********************************************************************/
// template<class KeyTYPE, class ValTYPE>
// inline void SmMapTypeToType<KeyTYPE,ValTYPE>::DumpAsObjects() const
// {
//
//   TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
//
//   smos_sprintf(sBuff,_T("SmMapTypeToULONG with %d elements  "),m_nCount);
//   smos_WriteBuffer(sBuff);
//   SmObject::Dump();
//
//   // Dump in format "[key] -> value"
//   KeyTYPE   key ;
//   ValTYPE   val ;
//   SmObject *pKey ;
//
//   SM_MAP_POSITION pos = GetStartPosition();
//   while (pos != NULL)
//     {
//       GetNextAssoc(pos, key, val);
//       pKey = (SmObject *)key ;
//       ULONG lKeyType = pKey->GetType() ;
//
//       // write out the relationship pointers
//       smos_sprintf(sBuff,       _T("\t %s-ULONG : [0x%p] = %ld\n"), SM_TOPO_TYPENAME(lKeyType), key, val);
//       smos_sprintf(sBuffForFile,_T("\t %s-ULONG : [%s] =%ld\n"),    SM_TOPO_TYPENAME(lKeyType),
//                                                             key ? _T("notNULL") : _T("NULL"),
//                                                             val ? _T("notNULL") : _T("NULL"));
//       smos_WriteBuffer(sBuff, sBuffForFile);
//
//     } // end while mappings to report
//
// } // end SmMapTypeToType<KeyTYPE,ValTYPE>::DumpAsObjects

class SmEdgeuse ;
SM_MAPTYPETOTYPE_TEMPLATE_PREDECLARATION(SmEdgeuse*,ULONG) ;

class SmAttribute ;
SM_MAPTYPETOTYPE_TEMPLATE_PREDECLARATION(SmAttribute*,ULONG) ;

class SmTopology ;
SM_MAPTYPETOTYPE_TEMPLATE_PREDECLARATION(SmTopology*,ULONG) ;

class SmObject ;
SM_MAPTYPETOTYPE_TEMPLATE_PREDECLARATION(SmObject*,ULONG) ;
SM_MAPTYPETOTYPE_TEMPLATE_PREDECLARATION(SmObject*,SmObject*) ;

#endif // no __SMMAPTYPETOTYPE_H__

