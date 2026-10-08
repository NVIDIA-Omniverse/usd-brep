// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmRelation.h
* PURPOSE: Header file for SmRelation object.  
**********************************************************************/

#ifndef __SMRELATION_H__
#define __SMRELATION_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMMAPPTRTOPTR_H__
#include <SmMapPtrToPtr.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifdef _WIN32
#define SM_RELATION_TEMPLATE_PREDECLARATION(x,y)  \
template class SM_EXPORT SmRelation<x,y>; 
#else
#define SM_RELATION_TEMPLATE_PREDECLARATION(x,y)
#endif

/*******************************************************************//**
PURPOSE: This enum describes the cardinality of the given relationship.

NOTES: 
***********************************************************************/
enum SmRelationCardinalityType
{
    SM_RC_ONE_TO_ONE,   // Relationship is one to one
    SM_RC_ONE_TO_MANY,  // Relationship is one to many
    SM_RC_MANY_TO_ONE,  // Relationship is many to one
    SM_RC_MANY_TO_MANY  // Relationship is many to many
};

/*******************************************************************//**
PURPOSE: This object is templated container class which manages a 
    dynamic relationship between SmObjects.  The relationship by default
    is many to many.  You can restrict the cardinality by setting the
    cardinality during construction.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> class SmRelation : public SmObject
{
protected:
  SmMapPtrToPtrs<TYPE1, TYPE2> m_vFirstToSecond;
  SmMapPtrToPtrs<TYPE2, TYPE1> m_vSecondToFirst;

  SmRelationCardinalityType m_eCardinality;  //  oneof: 
                                             //  SM_RC_ONE_TO_ONE,  Relationship is one to one  
                                             //  SM_RC_ONE_TO_MANY, Relationship is one to many 
                                             //  SM_RC_MANY_TO_ONE, Relationship is many to one 
                                             //  SM_RC_MANY_TO_MANY Relationship is many to many
public:
  SmRelation(const SmContext & crContext, SmRelationCardinalityType eCardinality=SM_RC_MANY_TO_MANY)
      : m_vFirstToSecond(&crContext),
        m_vSecondToFirst(&crContext),
        m_eCardinality(eCardinality) 
      { m_cpContext = &crContext; }
   virtual ~SmRelation() = default;

  // establish/remove relationships
  SmStatus RelatePair(TYPE1 * pFirst, TYPE2 * pSecond);
  SmStatus RelateToMany(TYPE1 *pFirst, const SmTArray<TYPE2*> & crSeconds);
  SmStatus RemovePair(TYPE1 * pFirst, TYPE2 * pSecond);
  void RemoveAll();

  // retrieve single partners
  TYPE2 * GetSecond(TYPE1 *pFirst)  const;
  TYPE1 * GetFirst (TYPE2 *pSecond) const;

  // retrieve many partners
  void GetSeconds(TYPE1 *pFirst, SmTArray<TYPE2*> & rSeconds) const ;
  void GetFirsts(TYPE2 *pSecond, SmTArray<TYPE1*> & rFirsts) const ;

  // retrieve all
  void GetAllFirsts (SmTArray<TYPE1*> & rFirsts) const ;
  void GetAllSeconds(SmTArray<TYPE2*> & rSeconds) const ;

  ULONG GetFirstsCount()  const { return m_vFirstToSecond.Count() ; }
  ULONG GetSecondsCount() const { return m_vSecondToFirst.Count() ; }

  // retrieve maps
  const SmMapPtrToPtrs<TYPE1, TYPE2> &GetFirstToSecond() const  { return m_vFirstToSecond ; }
  const SmMapPtrToPtrs<TYPE2, TYPE1> &GetSecondToFirst() const  { return m_vSecondToFirst ; }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmRelation,SmObject,SmRelation_TYPE);

} ; // end class SmRelation


/*******************************************************************//**
PURPOSE: Get all of seconds related to pFirst.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::GetSeconds
  (TYPE1 *pFirst,
   SmTArray<TYPE2*> & rSeconds)
 const
{
  rSeconds.ReSet();
  const auto& rSecondSet = m_vFirstToSecond.At(pFirst);
  rSeconds.SetDataSize((ULONG)rSecondSet.size());

  for (TYPE2* pSecond : rSecondSet)
  {
    rSeconds.Add(pSecond);
  }
}

/*******************************************************************//**
PURPOSE: Get all of first objects related to pSecond.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::GetFirsts
  (TYPE2 *pSecond,
   SmTArray<TYPE1*> & rFirsts)
 const
{
  rFirsts.ReSet();
  const auto& rFirstSet = m_vSecondToFirst.At(pSecond);
  rFirsts.SetDataSize((ULONG)rFirstSet.size());

  for (TYPE1* pFirst : rFirstSet)
  {
    rFirsts.Add(pFirst);
  }
}

/*******************************************************************//**
PURPOSE: Get all of seconds used in this relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::GetAllSeconds
  (SmTArray<TYPE2*> & rSeconds)
 const 
{
  m_vSecondToFirst.GetAllKeys(rSeconds);
}

/*******************************************************************//**
PURPOSE: Get all of firsts used in this relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::GetAllFirsts
  (SmTArray<TYPE1*> & rFirsts)
 const
{
  m_vFirstToSecond.GetAllKeys(rFirsts);
}

/*******************************************************************//**
PURPOSE: Remove all relationship pairs and free all internal memory
     used in the relationships.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::RemoveAll()
{
  m_vFirstToSecond.RemoveAll();
  m_vSecondToFirst.RemoveAll();
}


/*******************************************************************//**
PURPOSE: Relate a pair of objects.  Reports an error if the new
   relationship violates the given cardinality.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline SmStatus SmRelation<TYPE1,TYPE2>::RelatePair
 (TYPE1 * pFirst, 
  TYPE2 * pSecond)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
    }
#endif // SM_DEBUG_CODE
  SmStatus sRet = SM_SUCCESS;
  if (pFirst == NULL || pSecond == NULL) 
    {
      sRet = SM_ERR_INVALID_INPUT;
    }
  else 
    {    
      bool hasSecond = m_vFirstToSecond.Contains(pFirst);
      bool hasFirst = m_vSecondToFirst.Contains(pSecond);
      if (m_eCardinality == SM_RC_ONE_TO_ONE && (hasFirst || hasSecond))
        {
          SE(SM_ERR); // One of pair is already used
          sRet = SM_ERR;
        }
      else if (m_eCardinality == SM_RC_ONE_TO_MANY && hasFirst)
        {
          SE(SM_ERR);
          sRet = SM_ERR;
        }
      else if (m_eCardinality == SM_RC_MANY_TO_ONE && hasSecond)
        {
          SE(SM_ERR);
          sRet = SM_ERR;
        }
      else
        {
          m_vFirstToSecond.Insert(pFirst, pSecond);
          m_vSecondToFirst.Insert(pSecond, pFirst);
        }
    }

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      Dump() ;
    }
#endif // SM_DEBUG_CODE

  return sRet;

} // end SmRelation<TYPE1,TYPE2>::RelatePair

/*******************************************************************//**
PURPOSE: Relate an element of the first to many of the second.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline SmStatus SmRelation<TYPE1,TYPE2>::RelateToMany(TYPE1 * pFirst, 
                                                      const SmTArray<TYPE2*> & crSeconds)
{
    NER(pFirst); 
    for (ULONG i=0; i<crSeconds.GetSize(); i++) {
        TYPE2 * pSecond = crSeconds[i];
        SER(RelatePair(pFirst,pSecond));
    }
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Remove the relationship between a pair.  
   Produce an error if the relationship does not exist.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline SmStatus SmRelation<TYPE1,TYPE2>::RemovePair(TYPE1 * pFirst, TYPE2 *pSecond)
{ 
  SmStatus sRet = SM_SUCCESS;
  bool hasSecond = m_vFirstToSecond.Contains(pFirst);
  bool hasFirst = m_vSecondToFirst.Contains(pSecond);

  if (!hasFirst || !hasSecond)
    {
      SE(SM_ERR);
      sRet = SM_ERR;
    }
  else
    {
      m_vFirstToSecond.Remove(pFirst, pSecond);
      m_vSecondToFirst.Remove(pSecond, pFirst);
    }

  return sRet;
}

/*******************************************************************//**
PURPOSE: Get the second object of the pair from the first.  Return
    NULL if the object is not in the relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline TYPE2 * SmRelation<TYPE1,TYPE2>::GetSecond(TYPE1 * pFirst) const
{ 
  TYPE2* pRet = nullptr;
  std::size_t numSeconds = m_vFirstToSecond.At(pFirst).size();

  if (numSeconds == 1)
    {
      pRet = *(m_vFirstToSecond.At(pFirst).begin());
      if (pRet == nullptr) // JLMCC changed to == from !=
      {
        SE(SM_ERR); // Type mismatch
      }
    }
  else if (numSeconds > 1)
    {
      SE(SM_ERR);
    }

  return pRet;
} // end SmRelation<TYPE1,TYPE2>::GetSecond

/*******************************************************************//**
PURPOSE: Get the first object of the pair from the second.  Return
    NULL if the object is not in the relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline TYPE1 * SmRelation<TYPE1,TYPE2>::GetFirst(TYPE2 * pSecond) const
{   
  TYPE1* pRet = nullptr;
  std::size_t numFirsts = m_vSecondToFirst.At(pSecond).size();

  if (numFirsts == 1)
    {
      pRet = *(m_vSecondToFirst.At(pSecond).begin());
      if (pRet == nullptr)
      {
        SE(SM_ERR); // Type mismatch
      }
    }
  else if (numFirsts > 1)
    {
      SE(SM_ERR);
    }

  return pRet;
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE1, class TYPE2>
inline SmBoolean SmRelation<TYPE1, TYPE2>::IsKindOf( SM_TYPE t ) const
{
  return ((SmRelation_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::Dump() const
{
    smos_WriteBuffer(_T("\nSmRelation"));
    m_vFirstToSecond.Dump();
    m_vSecondToFirst.Dump();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if (bDebugMe) 
    {
      m_vFirstToSecond.DumpAsObjects() ;
      m_vSecondToFirst.DumpAsObjects();
    }
#endif // SM_DEBUG_CODE

} // end SmRelation<TYPE1,TYPE2>::Dump


#endif // !__SMRELATION_H__

