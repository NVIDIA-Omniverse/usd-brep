// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmMapPtrToPtr.h
* PURPOSE: Header file for SmMapPtrToPtr object.  
**********************************************************************/

/* Array Object definitions */
#ifndef __SMMAPPTRTOPTR_H__
#define __SMMAPPTRTOPTR_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMOBJECT_H__
#include <SmObject.h>
#endif

#include <unordered_map>
#include <unordered_set>

/*******************************************************************/ /**
 PURPOSE: A templated class for maps between pointers of KeyTYPE and ValTYPE.

 NOTES: 

 ***********************************************************************/
template<class KeyTYPE, class ValTYPE>
class SM_EXPORT SmMapPtrToPtr final : public SmObject
{
private:
  static_assert(std::is_pointer<KeyTYPE>::value == false, _T(""));
  static_assert(std::is_pointer<ValTYPE>::value == false, _T(""));
  using KeyType     = KeyTYPE*;
  using ValueType   = ValTYPE*;

  using CRKeyType   = const KeyType&;
  using RValueType  = ValueType&;
  using CRValueType = const ValueType&; // below uses SmAllocator to force memory allocation for maps through same allocators as the rest of the library.
  using MapType = std::unordered_map<KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>, SmAllocator<std::pair<const KeyType, ValueType>>> ;

  MapType* m_pMap; // not a value to avoid warning C4251

 public:
  SmMapPtrToPtr(const SmContext* cpContext = nullptr);
  SmMapPtrToPtr(const SmMapPtrToPtr&);
  virtual ~SmMapPtrToPtr();
  SmMapPtrToPtr& operator=(const SmMapPtrToPtr&);

  void Insert(CRKeyType rKey, CRValueType rValue) { (*m_pMap)[rKey] = rValue; }
  void Insert(const SmMapPtrToPtr& rOther)        { m_pMap->insert(rOther.m_pMap->begin(), rOther.m_pMap->end()); }
  void Remove(CRKeyType rKey)                     { m_pMap->erase(rKey); }

  ValueType At(CRKeyType rKey) const
  {
    auto it = m_pMap->find(rKey);
    if (it != m_pMap->end()) { return it->second; }
    else                     { return nullptr; }
  }
  // inserts new element if rKey is not contained
  RValueType operator[](CRKeyType rKey)       { return (*m_pMap)[rKey]; }
  ValueType  operator[](CRKeyType rKey) const { return At(rKey); }
  SmBoolean Contains(CRKeyType rKey) const { return m_pMap->find(rKey) != m_pMap->end(); }

  void RemoveAll(); // does not unset m_cpContext

  ULONG Count() const { return (ULONG)m_pMap->size(); }
  SmBoolean IsEmpty() const { return m_pMap->empty(); }

  KeyType Get1stKeyFor(CRValueType rVal) const
  {
      for (const std::pair<KeyType, ValueType>& sPair : *m_pMap)
      {
          if (sPair.second == rVal) { return sPair.first; }
      }
      return nullptr;
  };

  void GetAllKeys  (SmTArray<KeyType> & rKeys) const ;
  void GetAllValues(SmTArray<ValueType> & rValues) const ;
  void GetAllKeyValuePairs(SmTArray<KeyType> & rKeys, SmTArray<ValueType> & rValues) const;

  SM_COMMON(SmMapPtrToPtr,SmObject,SmMapPtrToPtr_TYPE);

  void Dump(SmBoolean bTerse) const override;
  void DumpAsObjects() const;

} ; // end class SmMapPtrToPtr

/*******************************************************************/ /**
 PURPOSE: A templated class for maps between pointers of KeyTYPE and
 unordered sets of  ValTYPE.

 NOTES:

 ***********************************************************************/

template<class KeyTYPE, class ValTYPE>
class SM_EXPORT SmMapPtrToPtrs final : public SmObject
{
private:
  static_assert(std::is_pointer<KeyTYPE>::value == false, _T(""));
  static_assert(std::is_pointer<ValTYPE>::value == false, _T(""));
  using KeyType = KeyTYPE*;
  using ValueType = ValTYPE*;

  using CRKeyType = const KeyType&;
  using RValueType = ValueType&;
  using CRValueType = const ValueType&;
  using ValueSetType = std::unordered_set<ValueType>;
  using RValueSetType = ValueSetType&;
  using CRValueSetType = const ValueSetType&; // below uses SmAllocator to force memory allocation for maps through same allocators as the rest of the library.
  using MapType = std::unordered_map<KeyType, ValueSetType, std::hash<KeyType>, std::equal_to<KeyType>, SmAllocator<std::pair<const KeyType, ValueSetType>>> ;

  MapType* m_pMap; // not a value to avoid warning C4251

 public:
  SmMapPtrToPtrs(const SmContext* cpContext = nullptr);
  SmMapPtrToPtrs(const SmMapPtrToPtrs&);
  virtual ~SmMapPtrToPtrs();
  SmMapPtrToPtrs& operator=(const SmMapPtrToPtrs&);

  void Insert(CRKeyType rKey, CRValueType rValue) { (*m_pMap)[rKey].insert(rValue); }
  void Insert(const SmMapPtrToPtrs& rOther)       { m_pMap->insert(rOther.m_pMap->begin(), rOther.m_pMap->end()); };
  void Remove(CRKeyType rKey)                     { m_pMap->erase(rKey); }
  void Remove(CRKeyType rKey, CRValueType rValue)
  {
    auto it = m_pMap->find(rKey);
    if (it != m_pMap->end())
    {
      it->second.erase(rValue);
      if (it->second.empty())
      {
        m_pMap->erase(rKey);
      }
    }
  }

  CRValueSetType At(CRKeyType rKey) const
  {
    static const ValueSetType emptyValueSet{};
    auto it = m_pMap->find(rKey);
    if (it != m_pMap->end()) { return it->second; }
    else                     { return emptyValueSet; }
  }
  // inserts new element if rKey is not contained
  RValueSetType operator[](CRKeyType rKey) { return (*m_pMap)[rKey]; }
  CRValueSetType  operator[](CRKeyType rKey) const { return At(rKey); }
  SmBoolean Contains(CRKeyType rKey) const { return m_pMap->find(rKey) != m_pMap->end(); }

  void RemoveAll(); // does not unset m_cpContext

  ULONG Count() const { return (ULONG)m_pMap->size(); }
  SmBoolean IsEmpty() const { return m_pMap->empty(); }

  void GetAllKeys(SmTArray<KeyType>& rKeys) const;
  void GetAllValues(SmTArray<ValueType>& rValues) const;
  void GetAllKeyValuePairs(SmTArray<KeyType>& rKeys, SmTArray<ValueType>& rValues) const;

  SM_COMMON(SmMapPtrToPtrs,SmObject,SmMapPtrToPtrs_TYPE);

  void Dump(SmBoolean bTerse) const override;
  void DumpAsObjects() const;

}; // end class SmMapPtrToPtrs

template<class KeyTYPE, class ValTYPE>
SmMapPtrToPtr<KeyTYPE, ValTYPE>::SmMapPtrToPtr(const SmContext* cpContext)
{
    m_cpContext = cpContext;
    m_pMap = new MapType;
}

template<class KeyTYPE, class ValTYPE>
SmMapPtrToPtr<KeyTYPE, ValTYPE>::SmMapPtrToPtr(const SmMapPtrToPtr& other)
{
    m_cpContext = other.m_cpContext;
    m_pMap = new MapType(*other.m_pMap);
}

template<class KeyTYPE, class ValTYPE>
SmMapPtrToPtr<KeyTYPE, ValTYPE>::~SmMapPtrToPtr()
{
    m_cpContext = nullptr;
    delete m_pMap;
    m_pMap = nullptr;
}

template<class KeyTYPE, class ValTYPE>
SmMapPtrToPtr<KeyTYPE, ValTYPE>& SmMapPtrToPtr<KeyTYPE, ValTYPE>::operator=(const SmMapPtrToPtr<KeyTYPE, ValTYPE>& rOther)
{
    m_cpContext = rOther.m_cpContext;
    delete m_pMap;
    m_pMap = new MapType(*rOther.m_pMap);
    return *this;
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtr<KeyTYPE, ValTYPE>::RemoveAll()
{
    m_pMap->clear();
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtr<KeyTYPE, ValTYPE>::GetAllKeys(SmTArray<KeyType>& rKeys) const
{
    rKeys.ReSet();
    rKeys.SetDataSize(Count());
    for (const auto& [rKey, rValue] : *m_pMap)
    {
        SM_REF1(rValue);
        rKeys.Add(rKey);
    }
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtr<KeyTYPE, ValTYPE>::GetAllValues(SmTArray<ValueType>& rValues) const
{
    rValues.ReSet();
    rValues.SetDataSize(Count());
    for (const auto& [rKey, rValue] : *m_pMap)
    {
        SM_REF1(rKey);
        rValues.Add(rValue);
    }
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtr<KeyTYPE, ValTYPE>::GetAllKeyValuePairs(SmTArray<KeyType>& rKeys, SmTArray<ValueType>& rValues) const
{
    rKeys.ReSet();
    rKeys.SetDataSize(Count());
    rValues.ReSet();
    rValues.SetDataSize(Count());
    for (const auto& [rKey, rValue] : *m_pMap)
    {
        rKeys.Add(rKey);
        rValues.Add(rValue);
    }
}

template<class KeyTYPE, class ValTYPE>
SmBoolean SmMapPtrToPtr<KeyTYPE, ValTYPE>::IsKindOf(SM_TYPE t) const
{
    return ((SmMapPtrToPtr_TYPE == t) ? TRUE : SmObject::IsKindOf((t)));
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtr<KeyTYPE, ValTYPE>::Dump() const
{
    Dump(TRUE);
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtr<KeyTYPE, ValTYPE>::Dump
(SmBoolean bTerse)   // in : TRUE = omit pair list
//      FALSE= list pairs
const
{
    TCHAR sBuff[SM_TBLOCK_SIZE];

    smos_sprintf(sBuff, _T("\nSmMapPtrToPtr: NumElems[%zu]\n"), m_pMap->size());
    smos_WriteBuffer(sBuff);
    SmObject::Dump();

    if (bTerse == TRUE)
    {
        return;
    }

    for (const auto& [rKey, rValue] : *m_pMap)
    {
        smos_sprintf(sBuff, _T("\t [%p] = %p\n"), rKey, rValue);
        smos_WriteBuffer(sBuff);
    }
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtr<KeyTYPE, ValTYPE>::DumpAsObjects() const
{
    SM_ASSERT(m_pMap != nullptr);

    TCHAR sBuff[SM_TBLOCK_SIZE];

    smos_sprintf(sBuff, _T("\nSmMapPtrToPtr: NumElems[%zu]\n"), m_pMap->size());
    smos_WriteBuffer(sBuff);
    SmObject::Dump();

    for (const auto& [rKey, rValue] : *m_pMap)
    {
        SmObject* pKey = (SmObject*)rKey;
        SmObject* pValue = (SmObject*)rValue;

        ULONG lKeyType = (ULONG)pKey->GetType();
        ULONG lValType = (ULONG)pValue->GetType();

        smos_sprintf(sBuff, _T("\t %s-%s : [%p] = %p\n"), lKeyType / 1000 == TOPO_BASE_TYPE / 1000 ? SM_TOPO_TYPENAME(lKeyType)
            : lKeyType / 1000 == SURF_BASE_TYPE / 1000 ? SM_SURF_TYPENAME(lKeyType)
            : lKeyType / 1000 == CURV_BASE_TYPE / 1000 ? SM_CURV_TYPENAME(lKeyType)
            : _T("Not A Named TYPE"),
            lValType / 1000 == TOPO_BASE_TYPE / 1000 ? SM_TOPO_TYPENAME(lValType)
            : lValType / 1000 == SURF_BASE_TYPE / 1000 ? SM_SURF_TYPENAME(lValType)
            : lValType / 1000 == CURV_BASE_TYPE / 1000 ? SM_CURV_TYPENAME(lValType)
            : _T("Not A Named TYPE"),
            pKey, pValue);
        smos_WriteBuffer(sBuff);
    }
}

template<class KeyTYPE, class ValTYPE>
SmMapPtrToPtrs<KeyTYPE, ValTYPE>::SmMapPtrToPtrs(const SmContext* cpContext)
{
    m_cpContext = cpContext;
    m_pMap = new MapType;
}

template<class KeyTYPE, class ValTYPE>
SmMapPtrToPtrs<KeyTYPE, ValTYPE>::SmMapPtrToPtrs(const SmMapPtrToPtrs& other)
{
    m_cpContext = other.m_cpContext;
    m_pMap = new MapType(*other.m_pMap);
}

template<class KeyTYPE, class ValTYPE>
SmMapPtrToPtrs<KeyTYPE, ValTYPE>::~SmMapPtrToPtrs()
{
    m_cpContext = nullptr;
    delete m_pMap;
    m_pMap = nullptr;
}

template<class KeyTYPE, class ValTYPE>
SmMapPtrToPtrs<KeyTYPE, ValTYPE>& SmMapPtrToPtrs<KeyTYPE, ValTYPE>::operator=(const SmMapPtrToPtrs<KeyTYPE, ValTYPE>& rOther)
{
    m_cpContext = rOther.m_cpContext;
    delete m_pMap;
    m_pMap = new MapType(*rOther.m_pMap);
    return *this;
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtrs<KeyTYPE, ValTYPE>::RemoveAll()
{
    m_pMap->clear();
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtrs<KeyTYPE, ValTYPE>::GetAllKeys(SmTArray<KeyType>& rKeys) const
{
    rKeys.ReSet();
    rKeys.SetDataSize(Count());
    for (const auto& [rKey, rValueSet] : *m_pMap)
    {
        SM_REF1(rValueSet);
        rKeys.Add(rKey);
    }
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtrs<KeyTYPE, ValTYPE>::GetAllValues(SmTArray<ValueType>& rValues) const
{
    rValues.ReSet();
    rValues.SetDataSize(Count());
    for (const auto& [rKey, rValueSet] : *m_pMap)
    {
        SM_REF1(rKey);
        for (const auto& rValue : rValueSet)
        {
            rValues.Add(rValue);
        }
    }
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtrs<KeyTYPE, ValTYPE>::GetAllKeyValuePairs(SmTArray<KeyType>& rKeys, SmTArray<ValueType>& rValues) const
{
    rKeys.ReSet();
    rKeys.SetDataSize(Count());
    rValues.ReSet();
    rValues.SetDataSize(Count());
    for (const auto& [rKey, rValueSet] : *m_pMap)
    {
        rKeys.Add(rKey);
        for (const auto& rValue : rValueSet)
        {
            rValues.Add(rValue);
        }
    }
}

template<class KeyTYPE, class ValTYPE>
SmBoolean SmMapPtrToPtrs<KeyTYPE, ValTYPE>::IsKindOf(SM_TYPE t) const
{
    return ((SmMapPtrToPtrs_TYPE == t) ? TRUE : SmObject::IsKindOf((t)));
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtrs<KeyTYPE, ValTYPE>::Dump() const
{
    Dump(TRUE);
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtrs<KeyTYPE, ValTYPE>::Dump
(SmBoolean bTerse)   // in : TRUE = omit pair list
//      FALSE= list pairs
const
{
    TCHAR sBuff[SM_TBLOCK_SIZE];

    smos_sprintf(sBuff, _T("\nSmMapPtrToPtrs: NumElems[%zu]\n"), m_pMap->size());
    smos_WriteBuffer(sBuff);
    SmObject::Dump();

    if (bTerse == TRUE)
    {
        return;
    }

    for (const auto& [rKey, rValueSet] : *m_pMap)
    {
        smos_sprintf(sBuff, _T("\t [%p] = {"), rKey);
        for (const auto& rValue : rValueSet)
        {
            smos_sprintf(sBuff, _T("%p, "), rValue);
            smos_WriteBuffer(sBuff);
        }
        smos_sprintf(sBuff, _T("}\n"), rKey);
        smos_WriteBuffer(sBuff);
    }
}

template<class KeyTYPE, class ValTYPE>
void SmMapPtrToPtrs<KeyTYPE, ValTYPE>::DumpAsObjects() const
{
    SM_ASSERT(m_pMap != nullptr);

    TCHAR sBuff[SM_TBLOCK_SIZE];

    smos_sprintf(sBuff, _T("\nSmMapPtrToPtrs: NumElems[%zu]\n"), m_pMap->size());
    smos_WriteBuffer(sBuff);
    SmObject::Dump();

    for (const auto& [rKey, rValueSet] : *m_pMap)
    {
        SmObject* pKey = (SmObject*)rKey;
        ULONG lKeyType = (ULONG)pKey->GetType();

        smos_sprintf(sBuff, _T("\t %s : [%p] = {"), lKeyType / 1000 == TOPO_BASE_TYPE / 1000 ? SM_TOPO_TYPENAME(lKeyType)
            : lKeyType / 1000 == SURF_BASE_TYPE / 1000 ? SM_SURF_TYPENAME(lKeyType)
            : lKeyType / 1000 == CURV_BASE_TYPE / 1000 ? SM_CURV_TYPENAME(lKeyType)
            : _T("Not A Named TYPE"),
            pKey);

        for (const auto& rValue : rValueSet)
        {
            SmObject* pValue = (SmObject*)rValue;
            ULONG lValType = (ULONG)pValue->GetType();
            smos_sprintf(sBuff, _T("%s : %p, "), lValType / 1000 == TOPO_BASE_TYPE / 1000 ? SM_TOPO_TYPENAME(lValType)
                : lValType / 1000 == SURF_BASE_TYPE / 1000 ? SM_SURF_TYPENAME(lValType)
                : lValType / 1000 == CURV_BASE_TYPE / 1000 ? SM_CURV_TYPENAME(lValType)
                : _T("Not A Named TYPE"),
                pValue);
            smos_WriteBuffer(sBuff);
        }
        smos_sprintf(sBuff, _T("}\n"), rKey);
        smos_WriteBuffer(sBuff);
    }
}



#endif // no __SMMAPPTRTOPTR_H__

