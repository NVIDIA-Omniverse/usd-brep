// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuAttribute.h
* PURPOSE: Header file for USD / SMLib object conversions
**********************************************************************/

#ifndef _SMU_ATTRIBUTE_H_
#define _SMU_ATTRIBUTE_H_

#include "SmuConfig.h"
#include "SmuTokens.h"

// pixar includes
#include "UsdBrepSuppressPixarWarningsPush.h"                 // turn off compile warnings for problematic pixar include files
#         include <pxr/usd/usdShade/materialBindingAPI.h>
#include "UsdBrepSuppressPixarWarningsPop.h"                  // done loading problematic pixar headers - restore compile warnings

// SMLib includes
#include "SmAttribute.h"

#define SmSdfPathAttribute_TYPE  (SmFirstUserAttribute_TYPE + 1)
#define SmUsdXformAttribute_TYPE (SmFirstUserAttribute_TYPE + 2)

// Following are used by USD to store metadata
#define SM_AI_MATERIAL_BINDING   (SM_AI_USER_1 + 1)  // Material binding path used by USD 
#define SM_AI_BREP_ARRAY         (SM_AI_USER_1 + 2)  // USD path to associated UsdSolidBrepAPI instance
#define SM_AI_XFORM_OP           (SM_AI_USER_1 + 3)  // USD Xform ops

/*******************************************************************//**
PURPOSE: A SMLib attribute for UsdBrepArray UsdPrimPath name for preserving
         PrimPath names during USD<->SMLib translation roundtrips

NOTES: 
***********************************************************************/
class SmSdfPathAttribute : public SmAttribute
{
protected:
 pxr::SdfPathVector m_sMaterialPaths ;   // gwc??? this was once a pointer to a SdfPathVector - see if there was a reason for that.  It may have to be a pointer once again.

public:
 // constructor - places a copy of pMaterialPaths into m_pMaterialPaths
  SMU_EXPORT SmSdfPathAttribute( ULONG                      lAttributeId,   // in :
                                 SmAttributeBehaviorType    eBehavior,      // in :
                                 const pxr::SdfPathVector & rMaterialPaths) // in :
             : SmAttribute(lAttributeId, eBehavior)
             { m_sMaterialPaths = rMaterialPaths ; }

  // copy constructor
  SMU_EXPORT SmSdfPathAttribute( const SmSdfPathAttribute & crOriginal) 
             : SmAttribute(crOriginal.m_lAttributeID,               // in :
                           crOriginal.m_eBehavior)                  // in :
             { m_sMaterialPaths = crOriginal.m_sMaterialPaths ; }

  // virtual destructor - required implementation for derived objects
  SMU_EXPORT virtual ~SmSdfPathAttribute() { }
    
  // virtual Copy function - required implementation for derived objects
  SMU_EXPORT virtual SmAttribute * MakeCopy(const SmContext & crContext) const 
  { 
    return( new (crContext) SmSdfPathAttribute( m_lAttributeID, m_eBehavior, m_sMaterialPaths) ) ;
  }

  // Attach Path attribute data to a UsdBrepArraySpecHandle - used when moving data from SMLib -> USD
  SMU_EXPORT bool AttachAttributeToBrepInstance(const pxr::SdfPrimSpecHandle & crUsdBrepArraySpecHandle, 
                                                pxr::TfToken & tInstanceName);

  // Attach Xform attributes to a UsdBrepArraySpecHandle - used when moving data from SMLib -> USD
  SMU_EXPORT bool AttachAttributeToXformable(const pxr::SdfPrimSpecHandle &crUsdBrepArraySpecHandle);

  // simple data access
  SMU_EXPORT pxr::SdfPathVector * GetMaterialPaths()                                    { return &m_sMaterialPaths ; }
  SMU_EXPORT void                 SetMaterialPaths(pxr::SdfPathVector & rMaterialPaths) { m_sMaterialPaths = rMaterialPaths; } 

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON_FOR_EXPORT(SmSdfPathAttribute, SmAttribute, SmSdfPathAttribute_TYPE, SMU_EXPORT);

}; //end class SmSdfPathAttribute

/*******************************************************************//**
PURPOSE: A SMLib attribute for space transformations for preserving
         transformation data during USD<->SMLib translation roundtrips

NOTES: Transformation matrices that get preserved through USD<->SMLib
       roundtrip translations are the one-to-one UsdBrepArray or OmniMesh 
       transformation matrix inherited through the base class Gprim
       to the transformation class usdGeomXformable represented by
       attribute xformOp:transform.
***********************************************************************/
class SmUsdXformAttribute : public SmAttribute
{
protected:
    pxr::GfVec3d               m_sTranslate ;
    pxr::GfVec3d               m_sScale ;
    pxr::GfVec3d               m_sRotateXYZ ;
    pxr::GfVec3d               m_sRotateXZY ;
    pxr::GfVec3d               m_sRotateYXZ ;
    pxr::GfVec3d               m_sRotateYZX ;
    pxr::GfVec3d               m_sRotateZXY ;
    pxr::GfVec3d               m_sRotateZYX ;
    double                     m_dRotateX ;
    double                     m_dRotateY ;
    double                     m_dRotateZ ;
    pxr::GfVec4d               m_sOrient ;
    pxr::GfMatrix4d            m_sTransform ;
    pxr::VtArray<pxr::TfToken> m_sXformOpOrder ;

public:
  SMU_EXPORT SmUsdXformAttribute(ULONG                        lAttributeId,         // in : pass-thru argument to SmAttribute constructor
                                 SmAttributeBehaviorType      eBehavior,            // in : pass-thru argument to SmAttribute constructor
                                 const pxr::GfVec3d               * pTranslate    = NULL,
                                 const pxr::GfVec3d               * pScale        = NULL,
                                 const pxr::GfVec3d               * pRotateXYZ    = NULL,
                                 const pxr::GfVec3d               * pRotateXZY    = NULL,
                                 const pxr::GfVec3d               * pRotateYXZ    = NULL,
                                 const pxr::GfVec3d               * pRotateYZX    = NULL,
                                 const pxr::GfVec3d               * pRotateZXY    = NULL,
                                 const pxr::GfVec3d               * pRotateZYX    = NULL,
                                 const double                       dRotateX      = 0.0,
                                 const double                       dRotateY      = 0.0,
                                 const double                       dRotateZ      = 0.0,
                                 const pxr::GfVec4d               * pOrient       = NULL,
                                 const pxr::GfMatrix4d            * pTransform    = NULL,
                                 const pxr::VtArray<pxr::TfToken> * pXformOpOrder = NULL) 
    : SmAttribute    (lAttributeId, eBehavior),
      m_sTranslate   (pTranslate    ? *pTranslate    : pxr::GfVec3d(0.0)),
      m_sScale       (pScale        ? *pScale        : pxr::GfVec3d(1.0)),
      m_sRotateXYZ   (pRotateXYZ    ? *pRotateXYZ    : pxr::GfVec3d(0.0)),
      m_sRotateXZY   (pRotateXZY    ? *pRotateXZY    : pxr::GfVec3d(0.0)),
      m_sRotateYXZ   (pRotateYXZ    ? *pRotateYXZ    : pxr::GfVec3d(0.0)),
      m_sRotateYZX   (pRotateYZX    ? *pRotateYZX    : pxr::GfVec3d(0.0)),
      m_sRotateZXY   (pRotateZXY    ? *pRotateZXY    : pxr::GfVec3d(0.0)),
      m_sRotateZYX   (pRotateZYX    ? *pRotateZYX    : pxr::GfVec3d(0.0)),
      m_dRotateX     (dRotateX),
      m_dRotateY     (dRotateY),
      m_dRotateZ     (dRotateZ),
      m_sOrient      (pOrient       ? *pOrient       : pxr::GfVec4d(0.0, 0.0, 0.0, 1.0)),
      m_sTransform   (pTransform    ? *pTransform    : pxr::GfMatrix4d(1.0)),
      m_sXformOpOrder(pXformOpOrder ? *pXformOpOrder : pxr::VtArray<pxr::TfToken>())
    { } // end SmUsdXformAttribute::SmUsdXformAttribute


  SMU_EXPORT SmUsdXformAttribute(const SmUsdXformAttribute & crOriginal);

  // virtual destructor - required implementation for derived objects
  SMU_EXPORT virtual ~SmUsdXformAttribute() { }
    
  // virtual Copy function - required implementation for derived objects
  SMU_EXPORT virtual SmAttribute * MakeCopy(const SmContext & crContext) const 
  { 
    return (new (crContext) SmUsdXformAttribute( m_lAttributeID,  m_eBehavior,
                                                &m_sTranslate,   &m_sScale, 
                                                &m_sRotateXYZ,   &m_sRotateXZY,
                                                &m_sRotateYXZ,   &m_sRotateYZX,
                                                &m_sRotateZXY,   &m_sRotateZYX, 
                                                 m_dRotateX,      m_dRotateY,     m_dRotateZ,
                                                &m_sOrient,      &m_sTransform, 
                                                &m_sXformOpOrder) ) ;
  }                                                                            

  // Attach Xform attribute data to a UsdBrepArraySpecHandle - used when moving data from SMLib -> USD
  SMU_EXPORT bool AttachAttributeToBrepInstance(const pxr::SdfPrimSpecHandle & crUsdBrepArraySpecHandle, pxr::TfToken & rtInstanceName);
  
  // Attach Xform attribute data to a UsdGeomXformable  primSpec - used when moving data from SMLib -> USD
  SMU_EXPORT bool AttachAttributeToXformable(const pxr::SdfPrimSpecHandle & crPrimSpecHandle);

  // simple data access
  SMU_EXPORT const pxr::GfVec3d                & GetXformOpTranslate() const {return m_sTranslate ; }  
  SMU_EXPORT const pxr::GfVec3d                & GetXformOpScale ()    const {return m_sScale     ; }
  SMU_EXPORT const pxr::GfVec3d                & GetXformOpRotateXYZ() const {return m_sRotateXYZ ; }
  SMU_EXPORT const pxr::GfVec3d                & GetXformOpRotateXZY() const {return m_sRotateXZY ; }
  SMU_EXPORT const pxr::GfVec3d                & GetXformOpRotateYXZ() const {return m_sRotateYXZ ; }
  SMU_EXPORT const pxr::GfVec3d                & GetXformOpRotateYZX() const {return m_sRotateYZX ; }
  SMU_EXPORT const pxr::GfVec3d                & GetXformOpRotateZXY() const {return m_sRotateZXY ; }
  SMU_EXPORT const pxr::GfVec3d                & GetXformOpRotateZYX() const {return m_sRotateZYX ; }
  SMU_EXPORT const double                      & GetXformOpRotateX ()  const {return m_dRotateX   ; }
  SMU_EXPORT const double                      & GetXformOpRotateY ()  const {return m_dRotateY   ; }
  SMU_EXPORT const double                      & GetXformOpRotateZ ()  const {return m_dRotateZ   ; }
  SMU_EXPORT const pxr::GfVec4d                & GetXformOpOrient ()   const {return m_sOrient    ; }
  SMU_EXPORT const pxr::GfMatrix4d             & GetXformOpTransform() const {return m_sTransform ; }
  SMU_EXPORT const pxr::VtArray<pxr::TfToken>  & GetXformOpOrder()     const {return m_sXformOpOrder ; }


  SMU_EXPORT void SetXformOpTranslate (const pxr::GfVec3d               & rTranslate)    { m_sTranslate    = rTranslate ; }  
  SMU_EXPORT void SetXformOpScale     (const pxr::GfVec3d               & rScale)        { m_sScale        = rScale ;     }
  SMU_EXPORT void SetXformOpRotateXYZ (const pxr::GfVec3d               & rRotateXYZ)    { m_sRotateXYZ    = rRotateXYZ ; }
  SMU_EXPORT void SetXformOpRotateXZY (const pxr::GfVec3d               & rRotateXZY)    { m_sRotateXZY    = rRotateXZY ; }
  SMU_EXPORT void SetXformOpRotateYXZ (const pxr::GfVec3d               & rRotateYXZ)    { m_sRotateYXZ    = rRotateYXZ ; }
  SMU_EXPORT void SetXformOpRotateYZX (const pxr::GfVec3d               & rRotateYZX)    { m_sRotateYZX    = rRotateYZX ; }
  SMU_EXPORT void SetXformOpRotateZXY (const pxr::GfVec3d               & rRotateZXY)    { m_sRotateZXY    = rRotateZXY ; }
  SMU_EXPORT void SetXformOpRotateZYX (const pxr::GfVec3d               & rRotateZYX)    { m_sRotateZYX    = rRotateZYX ; }
  SMU_EXPORT void SetXformOpRotateX   (double                             dRotateX)      { m_dRotateX      = dRotateX ;   }
  SMU_EXPORT void SetXformOpRotateY   (double                             dRotateY)      { m_dRotateY      = dRotateY ;   }
  SMU_EXPORT void SetXformOpRotateZ   (double                             dRotateZ)      { m_dRotateZ      = dRotateZ ;   }
  SMU_EXPORT void SetXformOpOrient    (const pxr::GfVec4d               & rOrient)       { m_sOrient       = rOrient ;    }
  SMU_EXPORT void SetXformOpTransform (const pxr::GfMatrix4d            & rTransform)    { m_sTransform    = rTransform ; }
  SMU_EXPORT void SetXformOpOrder     (const pxr::VtArray<pxr::TfToken> & rXformOpOrder) { m_sXformOpOrder = rXformOpOrder ; }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON_FOR_EXPORT(SmUsdXformAttribute, SmAttribute, SmUsdXformAttribute_TYPE, SMU_EXPORT);

}; //end class SmUsdXformAttribute

#endif // no _SMU_ATTRIBUTE_H_
