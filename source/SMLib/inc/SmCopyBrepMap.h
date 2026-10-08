// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBrep.h
* PURPOSE: Header file for SmBrep class.
**********************************************************************/

#ifndef __SMCOPYBREPMAP_H__
#define __SMCOPYBREPMAP_H__

/*******************************************************************//**
PURPOSE: A structure to map the pointers of one copied Brep topology
            to the source Brep's topology object.

NOTES: this class is optionally used by SmBrep Copy constructor
            and internally by SmBrep::Merge() to remember how the
            topology objects of a copied Brep map the the topology objects
            of the original Brep.  Once either brep is modified the
            map will be obsolete.
***********************************************************************/
class SM_EXPORT SmCopyBrepMap final : public SmObject
{
protected:
  SmBrep *m_pOrigBrep = nullptr;    // Target Brep
  SmBrep *m_pCopyBrep = nullptr;    // Copy of Target Brep

  SmMapPtrToPtr<SmTopology, SmTopology> m_sMap; // map Copy topology elements to original elements

public:
  SmCopyBrepMap() = default;
  virtual ~SmCopyBrepMap() { m_pOrigBrep = nullptr ;
                             m_pCopyBrep = nullptr ;
                           }

  // simple data access
  void        SetBreps(SmBrep *pOrigBrep,
                       SmBrep *pCopyBrep)  { m_pOrigBrep = pOrigBrep ;
                                             m_pCopyBrep = pCopyBrep ;
                                           }
  SmBrep     *GetOrigBrep() const          { return m_pOrigBrep ; }
  SmBrep     *GetCopyBrep() const          { return m_pCopyBrep ; }
  void        SetAt(SmTopology *pCopyObj,
                    SmTopology *pOrigObj) ;
  SmTopology *GetAt(SmTopology *pCopyObj) const ;

  virtual void Dump() const override;
  virtual void Dump(SmBoolean bFULL) const override;

} ; // end class SmCopyBrepMap

#endif // !__SMCOPYBREPMAP_H__
