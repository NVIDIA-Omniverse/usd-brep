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
    SmMapPtrToPtr m_vFirstToSecond;
    SmMapPtrToPtr m_vSecondToFirst;
    SmRelationCardinalityType m_eCardinality;

public:
    SmRelation(const SmContext & crContext,
               SmRelationCardinalityType eCardinality=SM_RC_MANY_TO_MANY) 
        : m_vFirstToSecond(20,&crContext), m_vSecondToFirst(20,&crContext),
        m_eCardinality(eCardinality) { m_cpContext = &crContext; }
    virtual ~SmRelation();

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
    SM_COMMON(SmRelation,SmObject,SmRelation_TYPE);
};


/*******************************************************************//**
PURPOSE: Destructor for relationship object.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline SmRelation<TYPE1,TYPE2>::~SmRelation()
{
    RemoveAll();
}

/*******************************************************************//**
PURPOSE: Get all of seconds related to pFirst.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::GetSeconds(const TYPE1 *pFirst, 
                                                SmTArray<TYPE2*> & rSeconds)
{
    rSeconds.ReSet();
    SmObject *pObj = SM_REINTERPRET_CAST(SmObject*,m_vFirstToSecond.GetValueAt(pFirst));
    if (pObj && pObj->GetType() == SmTArray_TYPE) {
        SmTArray<TYPE2*> * pSeconds = SM_REINTERPRET_CAST(SmTArray<TYPE2*> *,pObj);
        rSeconds.Append(*pSeconds);
    }
    else if (pObj) {
        rSeconds.Add(SM_REINTERPRET_CAST(TYPE2*,pObj));
    }
}

/*******************************************************************//**
PURPOSE: Get all of first objects related to pSecond.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::GetFirsts(const TYPE2 *pSecond, 
                                               SmTArray<TYPE1*> & rFirsts)
{
    rFirsts.ReSet();
    SmObject *pObj = SM_REINTERPRET_CAST(SmObject*,m_vSecondToFirst.GetValueAt(pSecond));
    if (pObj && pObj->GetType() == SmTArray_TYPE) {
        SmTArray<TYPE1*> * pFirsts = SM_REINTERPRET_CAST(SmTArray<TYPE1*> *,pObj);
        rFirsts.Append(*pFirsts);
    }
    else if (pObj) {
        rFirsts.Add(SM_REINTERPRET_CAST(TYPE1*,pObj));
    }
}

/*******************************************************************//**
PURPOSE: Get all of seconds used in this relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::GetAllSeconds(SmTArray<TYPE2*> & rSeconds)
{
    rSeconds.ReSet();
    m_vSecondToFirst.GetAllKeys(SM_REINTERPRET_CAST(SmTArray<void*>&,rSeconds));
}

/*******************************************************************//**
PURPOSE: Get all of firsts used in this relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::GetAllFirsts(SmTArray<TYPE1*> & rFirsts)
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
inline void SmRelation<TYPE1,TYPE2>::RemoveAll()
{
    SmTArray<void*> sValues;
    if (m_eCardinality == SM_RC_ONE_TO_MANY ||
        m_eCardinality == SM_RC_MANY_TO_MANY) {
        void *key = NULL;
        void *val = NULL;
        SM_MAP_POSITION pos = m_vFirstToSecond.GetStartPosition();
        //m_vFirstToSecond.GetAllValues(sValues);
        //for (ULONG i=0; i<sValues.GetSize(); i++) {
        while ( pos != NULL ){
            m_vFirstToSecond.GetNextAssoc( pos, key, val );
            SmObject *pObj = SM_REINTERPRET_CAST( SmObject*, val );
            if (pObj->GetType() == SmTArray_TYPE) {
                delete pObj; pObj = NULL ;
            }
        }
    }
    if (m_eCardinality == SM_RC_MANY_TO_ONE ||
        m_eCardinality == SM_RC_MANY_TO_MANY) {
        void *key = NULL;
        void *val = NULL;
        SM_MAP_POSITION pos = m_vSecondToFirst.GetStartPosition();
        //m_vSecondToFirst.GetAllValues(sValues);
        //for (ULONG i=0; i<sValues.GetSize(); i++) {
        while ( pos != NULL ){
            m_vSecondToFirst.GetNextAssoc( post, key, val );
            SmObject *pObj = SM_REINTERPRET_CAST( SmObject*, val );
            if (pObj->GetType() == SmTArray_TYPE) {
                delete pObj; pObj = NULL ;
            }
        }
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
inline SmStatus SmRelation<TYPE1,TYPE2>::RelatePair(TYPE1 * pFirst, 
                                                    TYPE2 * pSecond)
{ 
    SmStatus sRet = SM_SUCCESS;
    if (pFirst == NULL || pSecond == NULL) {
        sRet = SM_ERR_INVALID_INPUT;
    }
    else {    
        void *pTest2 = m_vFirstToSecond.GetValueAt(pFirst);
        void *pTest1 = m_vSecondToFirst.GetValueAt(pSecond);
        if (m_eCardinality == SM_RC_ONE_TO_ONE) {
            if (pTest1 || pTest2) {
                SE(SM_ERR); // One of pair is already used
                sRet = SM_ERR;
            }
            else {
                m_vFirstToSecond.SetAt(pFirst,pSecond);
                m_vSecondToFirst.SetAt(pSecond,pFirst);
            }
        }
        else {
            if (m_eCardinality == SM_RC_ONE_TO_MANY) {
                if (pTest1) {
                    SE(SM_ERR);
                    sRet = SM_ERR;
                }
            }
            if (m_eCardinality == SM_RC_MANY_TO_ONE) {
                if (pTest2) {
                    SE(SM_ERR);
                    sRet = SM_ERR;
                }
            }
            if (sRet == SM_SUCCESS) {
                // Handle first relationship to second
                if (!pTest2) {
                    m_vFirstToSecond.SetAt(pFirst,pSecond);
                }
                else {
                    SmObject *pObj2 = SM_REINTERPRET_CAST(SmObject*,pTest2);
                    if (pObj2->GetType() == SmTArray_TYPE) { // Just add it to the array
                        SmTArray<TYPE2*> * pArray = SM_REINTERPRET_CAST(SmTArray<TYPE2*> *,pObj2);
                        pArray->Add(SM_CONST_CAST(TYPE2*,pSecond));
                    }
                    else { // Create an array and add both elements then relate the
                        // array to the first
                        m_vFirstToSecond.RemoveKey(pFirst);
                        SmTArray<TYPE2*> * pArray = new (*m_cpContext) SmTArray<TYPE2*>(*m_cpContext);
                        pArray->Add(SM_REINTERPRET_CAST(TYPE2 *,pTest2));
                        pArray->Add(SM_CONST_CAST(TYPE2 *,pSecond));
                        m_vFirstToSecond.SetAt(pFirst,pArray);
                    }
                }
                
                if (!pTest1) {
                    m_vSecondToFirst.SetAt(pSecond,pFirst);
                }
                else {
                    SmObject *pObj1 = SM_REINTERPRET_CAST(SmObject*,pTest1);
                    if (pObj1->GetType() == SmTArray_TYPE) { // Just add it to the array
                        SmTArray<TYPE1*> * pArray = SM_REINTERPRET_CAST(SmTArray<TYPE1*> *,pObj1);
                        pArray->Add(SM_CONST_CAST(TYPE1 *,pFirst));
                    }
                    else { // Create an array and add both elements then relate the
                        // array to the first
                        m_vSecondToFirst.RemoveKey(pSecond);
                        SmTArray<TYPE1*> * pArray = new (*m_cpContext) SmTArray<TYPE1*>(*m_cpContext);
                        pArray->Add(SM_REINTERPRET_CAST(TYPE1 *,pTest1));
                        pArray->Add(SM_CONST_CAST(TYPE1 *,pFirst));
                        m_vSecondToFirst.SetAt(pSecond,pArray);
                    }
                }
            }
        }
    }
    return sRet;
}

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
    void *pTest2 = m_vFirstToSecond.GetValueAt(pFirst);
    void *pTest1 = m_vSecondToFirst.GetValueAt(pSecond);
    SmStatus sRet = SM_SUCCESS;

    if (pTest2 == NULL || pTest1 == NULL) {
        SE(SM_ERR);
        sRet = SM_ERR;
    }

    if (sRet == SM_SUCCESS) {
        SmObject *pObj2 = SM_REINTERPRET_CAST(SmObject*,pTest2);
        SmObject *pObj1 = SM_REINTERPRET_CAST(SmObject*,pTest1);
        if (pObj2->GetType() == SmTArray_TYPE) {
            SmTArray<TYPE2*> *pArray2 = SM_REINTERPRET_CAST(SmTArray<TYPE2*> *,pObj2);
            ULONG lFound;
            if (!pArray2->FindElement(SM_CONST_CAST(TYPE2*,pSecond),lFound)) {
                SE(SM_ERR);
                sRet = SM_ERR;
            }
            else {
                pArray2->RemoveAt(lFound,1);
                if (pArray2->GetSize() == 1) {
                    delete pArray2; pArray2 = NULL ;
                    m_vFirstToSecond.RemoveKey(pFirst);
                    m_vFirstToSecond.SetAt(pFirst,pSecond);
                }
            }
        }
        else { // Have a single object in relationship
            if (pObj2 == SM_CONST_CAST(TYPE2*,pSecond)) {
                m_vFirstToSecond.RemoveKey(pFirst);
            }
            else {
                SE(SM_ERR);
                sRet = SM_ERR;
            }
        }

        if (pObj1->GetType() == SmTArray_TYPE) {
            SmTArray<TYPE1*> *pArray1 = SM_REINTERPRET_CAST(SmTArray<TYPE1*> *,pObj1);
            ULONG lFound;
            if (!pArray1->FindElement(SM_CONST_CAST(TYPE1*,pFirst),lFound)) {
                SE(SM_ERR);
                sRet = SM_ERR;
            }
            else {
                pArray1->RemoveAt(lFound,1);
                if (pArray1->GetSize() == 1) {
                    delete pArray1; pArray1 = NULL ;
                    m_vSecondToFirst.RemoveKey(pSecond);
                    m_vSecondToFirst.SetAt(pSecond,pFirst);
                }
            }
        }
        else { // Have a single object in relationship
            if (pObj1 == SM_CONST_CAST(TYPE1*,pFirst)) {
                m_vSecondToFirst.RemoveKey(pSecond);
            }
            else {
                SE(SM_ERR);
                sRet = SM_ERR;
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
inline TYPE2 * SmRelation<TYPE1,TYPE2>::GetSecond(const TYPE1 * pFirst) const
{   
    void *pRet = m_vFirstToSecond.GetValueAt(pFirst);
    SmObject *pObj2 = SM_REINTERPRET_CAST(SmObject*,pRet);
    TYPE2 *pRet2;
    if (pObj2 == NULL) {
        pRet2 = NULL;
    }
    else if (pObj2->GetType() == SmTArray_TYPE) {
        SE(SM_ERR); // Should not use this method if there are arrays
        pRet2 = NULL;
    }
    else {
        pRet2 = SM_REINTERPRET_CAST(TYPE2*,pObj2);
        if (!pRet2) SE(SM_ERR); // Type mismatch
    }
    return pRet2;
}

/*******************************************************************//**
PURPOSE: Get the first object of the pair from the second.  Return
    NULL if the object is not in the relationship.

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline TYPE1 * SmRelation<TYPE1,TYPE2>::GetFirst(const TYPE2 * pSecond) const
{   
    void *pRet = m_vSecondToFirst.GetValueAt(pSecond);
    SmObject *pObj1 = SM_REINTERPRET_CAST(SmObject*,pRet);
    TYPE1 *pRet1;
    if (pObj1 == NULL) {
        pRet1 = NULL;
    }
    else if (pObj1->GetType() == SmTArray_TYPE) {
        SE(SM_ERR); // Should not use this method if there are arrays
        pRet1 = NULL;
    }
    else {
        pRet1 = SM_REINTERPRET_CAST(TYPE1*,pObj1);
        if (!pRet1) SE(SM_ERR); // Type mismatch
    }
    return pRet1;
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
template<class TYPE1, class TYPE2> 
inline void SmRelation<TYPE1,TYPE2>::Dump() const
{
    smos_WriteBuffer(_T("SmRelation\n"));
    m_vFirstToSecond.Dump();
    m_vSecondToFirst.Dump();
}


#endif // !__SMRELATION_H__

