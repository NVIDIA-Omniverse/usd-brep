// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmRelationMM.h
* PURPOSE: Header file for SmRelationV object.  
**********************************************************************/

#ifndef __SMRELATIONMM_H__
#define __SMRELATIONMM_H__

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
#define SM_RELATIONMM_TEMPLATE_PREDECLARATION(x,y)  \
template class SM_EXPORT SmRelationMM<x,y>; 
#else
#define SM_RELATIONMM_TEMPLATE_PREDECLARATION(x,y)
#endif


/*******************************************************************//**
PURPOSE: This object is templated container class which manages a 
    dynamic relationship between SmObjects.  The relationship by default
    is many to many.  You can restrict the cardinality by setting the
    cardinality during construction.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> class SmRelationMM : public SmObject
{
protected:
    SmMapPtrToPtr m_vFirstToSecond;
    SmMapPtrToPtr m_vSecondToFirst;

public:
    SmRelationMM(const SmContext & crContext) : 
      m_vFirstToSecond(20,&crContext), m_vSecondToFirst(20,&crContext)
      { m_cpContext = &crContext; }
    virtual ~SmRelationMM();

    SmStatus RelatePair(TYPE1 * pFirst, TYPE2 * pSecond);
    SmStatus RelateToMany(TYPE1 *pFirst, const SmTArray<TYPE2*> & crSeconds);
    SmStatus RemovePair(TYPE1 * pFirst, TYPE2 * pSecond);
    void RemoveAll();

    TYPE2 * GetSecond(const TYPE1 *pFirst) const;
    TYPE1 * GetFirst(const TYPE2 *pSecond) const;

    void GetSeconds(const TYPE1 *pFirst, SmTArray<TYPE2*> & rSeconds);
    void GetFirsts(const TYPE2 *pSecond, SmTArray<TYPE1*> & rFirsts);

    void GetAllFirsts(SmTArray<TYPE1*> & rFirsts);
    void GetAllSeconds(SmTArray<TYPE2*> & rSeconds);

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmRelationMM,SmObject,SmRelation_TYPE);
};


/*******************************************************************//**
PURPOSE: Destructor for relationship object.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline SmRelationMM<TYPE1,TYPE2>::~SmRelationMM()
{
    RemoveAll();
}

/*******************************************************************//**
PURPOSE: Get all of seconds related to pFirst.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelationMM<TYPE1,TYPE2>::GetSeconds(const TYPE1 *pFirst, 
                                                SmTArray<TYPE2*> & rSeconds)
{
    rSeconds.ReSet();
    SmTArray<TYPE2*> * pSeconds = SM_REINTERPRET_CAST(SmTArray<TYPE2*> *,m_vFirstToSecond.GetValueAt(pFirst));
    if (pSeconds) rSeconds.Append(*pSeconds);
}

/*******************************************************************//**
PURPOSE: Get all of first objects related to pSecond.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelationMM<TYPE1,TYPE2>::GetFirsts(const TYPE2 *pSecond, 
                                               SmTArray<TYPE1*> & rFirsts)
{
    rFirsts.ReSet();
    SmTArray<TYPE1*> * pFirsts = SM_REINTERPRET_CAST(SmTArray<TYPE1*> *,m_vSecondToFirst.GetValueAt(pSecond));
    if (pFirsts) rFirsts.Append(*pFirsts);
}

/*******************************************************************//**
PURPOSE: Get all of seconds used in this relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelationMM<TYPE1,TYPE2>::GetAllSeconds(SmTArray<TYPE2*> & rSeconds)
{
    rSeconds.ReSet();
    m_vSecondToFirst.GetAllKeys(SM_REINTERPRET_CAST(SmTArray<void*>&,rSeconds));
}

/*******************************************************************//**
PURPOSE: Get all of firsts used in this relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelationMM<TYPE1,TYPE2>::GetAllFirsts(SmTArray<TYPE1*> & rFirsts)
{
    rFirsts.ReSet();
    m_vFirstToSecond.GetAllKeys(SM_REINTERPRET_CAST(SmTArray<void*>&,rFirsts));
}


/*******************************************************************//**
PURPOSE: Remove all relationship pairs and free all internal memory
     used in the relationships.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelationMM<TYPE1,TYPE2>::RemoveAll()
{
    SmTArray<void*> sValues;
    void *key;
    void *val;
    SM_MAP_POSITION pos = m_vFirstToSecond.GetStartPosition();
    //m_vFirstToSecond.GetAllValues(sValues);
    //for (ULONG i=0; i<sValues.GetSize(); i++) {
    while ( pos != NULL ) {
        m_vFirstToSecond.GetNextAssoc( pos, key, val );
        SmObject *pObj = SM_REINTERPRET_CAST( SmObject*, val );
        SM_ASSERT( pObj != NULL ); delete pObj; pObj = NULL;
    }
    void *key = NULL;
    void *val = NULL;
    SM_MAP_POSITION pos = m_vSecondToFirst.GetStartPosition();
    //m_vSecondToFirst.GetAllValues(sValues);
    //for (ULONG i=0; i<sValues.GetSize(); i++) {
    while ( pos != NULL ) {
        m_vSecondToFirst.GetNextAssoc( post, key, val );
        SmObject *pObj = SM_REINTERPRET_CAST( SmObject*, val );
        SM_ASSERT( pObj != NULL ); delete pObj; pObj = NULL;
    }
    m_vFirstToSecond.RemoveAll(); 
    m_vSecondToFirst.RemoveAll(); 
}



/*******************************************************************//**
PURPOSE: Relate a pair of objects.  Reports an error if the new
   relationship violates the given cardinality.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline SmStatus SmRelationMM<TYPE1,TYPE2>::RelatePair(TYPE1 * pFirst, 
                                                    TYPE2 * pSecond)
{ 
    SmStatus sRet = SM_SUCCESS;
    if (pFirst == NULL || pSecond == NULL) {
        sRet = SM_ERR_INVALID_INPUT;
    }
    else {    
        void *pTest2 = m_vFirstToSecond.GetValueAt(pFirst);
        void *pTest1 = m_vSecondToFirst.GetValueAt(pSecond);

                // Handle first relationship to second
        if (!pTest2) {
            SmTArray<TYPE2*> * pArray = new (*m_cpContext) SmTArray<TYPE2*>(*m_cpContext);
            m_vFirstToSecond.SetAt(pFirst,pArray);
            pTest2 = pArray;
        }
        SmTArray<TYPE2*> * pArray1 = SM_REINTERPRET_CAST(SmTArray<TYPE2*> *,pTest2);
        pArray1->Add(SM_CONST_CAST(TYPE2*,pSecond));
                
        if (!pTest1) {
            SmTArray<TYPE1*> * pArray = new (*m_cpContext) SmTArray<TYPE1*>(*m_cpContext);
            m_vSecondToFirst.SetAt(pSecond,pArray);
            pTest1 = pArray;
        }
        SmTArray<TYPE1*> * pArray2 = SM_REINTERPRET_CAST(SmTArray<TYPE1*> *,pTest1);
        pArray2->Add(SM_CONST_CAST(TYPE1 *,pFirst));
    }

    return sRet;
}

/*******************************************************************//**
PURPOSE: Relate an element of the first to many of the second.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline SmStatus SmRelationMM<TYPE1,TYPE2>::RelateToMany(TYPE1 * pFirst, 
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
inline SmStatus SmRelationMM<TYPE1,TYPE2>::RemovePair(TYPE1 * pFirst, TYPE2 *pSecond)
{ 
    void *pTest1 = m_vSecondToFirst.GetValueAt(pSecond);
    SmTArray<TYPE1*> *pArray1 = SM_REINTERPRET_CAST(SmTArray<TYPE1*> *,pTest1);

    void *pTest2 = m_vFirstToSecond.GetValueAt(pFirst);
    SmTArray<TYPE2*> *pArray2 = SM_REINTERPRET_CAST(SmTArray<TYPE2*> *,pTest2);

    SmStatus sRet = SM_SUCCESS;

    if (pTest2 == NULL || pTest1 == NULL) {
        SE(SM_ERR);
        sRet = SM_ERR;
    }

    if (sRet == SM_SUCCESS) {
        ULONG lFound;
        if (!pArray2->FindElement(SM_CONST_CAST(TYPE2*,pSecond),lFound)) {
            SE(SM_ERR);
            sRet = SM_ERR;
        }
        else {
            pArray2->RemoveAt(lFound,1);
            if (pArray2->GetSize() == 0) {
                m_vFirstToSecond.RemoveKey(pFirst);
                delete pArray2; pArray2 = NULL ;
            }
        }
    }
    if (sRet == SM_SUCCESS) {
        ULONG lFound;
        if (!pArray1->FindElement(SM_CONST_CAST(TYPE1*,pFirst),lFound)) {
            SE(SM_ERR);
            sRet = SM_ERR;
        }
        else {
            pArray1->RemoveAt(lFound,1);
            if (pArray1->GetSize() == 0) {
                delete pArray1; pArray1 = NULL ;
                m_vSecondToFirst.RemoveKey(pSecond);
            }
        }
    }
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Get the second object of the pair from the first.  Return
    NULL if the object is not in the relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline TYPE2 * SmRelationMM<TYPE1,TYPE2>::GetSecond(const TYPE1 * pFirst) const
{   
    void *pTest2 = m_vFirstToSecond.GetValueAt(pFirst);
    SmTArray<TYPE2*> *pArray2 = SM_REINTERPRET_CAST(SmTArray<TYPE2*> *,pTest2);
    TYPE2* ret = NULL;
    if (pArray2) {
        ret = (*pArray2)[0];
    }
    return ret ;
}

/*******************************************************************//**
PURPOSE: Get the first object of the pair from the second.  Return
    NULL if the object is not in the relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline TYPE1 * SmRelationMM<TYPE1,TYPE2>::GetFirst(const TYPE2 * pSecond) const
{   
    void *pTest1 = m_vSecondToFirst.GetValueAt(pSecond);
    SmTArray<TYPE1*> *pArray1 = SM_REINTERPRET_CAST(SmTArray<TYPE1*> *,pTest1);
    TYPE1* ret = NULL;
    if (pArray1) {
        ret = (*pArray1)[0];
    }
    return ret;
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE1, class TYPE2>
inline SmBoolean SmRelationMM<TYPE1, TYPE2>::IsKindOf( SM_TYPE t ) const
{
  return ((SmRelationMM_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelationMM<TYPE1,TYPE2>::Dump() const
{
    smos_WriteBuffer(_T("SmRelationMM\n"));
    m_vFirstToSecond.Dump();
    m_vSecondToFirst.Dump();
}


#endif // !__SMRELATIONMM_H__

