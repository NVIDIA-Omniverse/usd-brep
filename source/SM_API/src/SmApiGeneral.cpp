// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


/**********************************************************************
FILE NAME: SmApiGeneral.cpp

PURPOSE: 
   Contains popular high level "C" type functions of general use.

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SMLib, etc
**********************************************************************/

#include "StdAfx.h"

#include "SmApiGeneral.h"
#include <SmMemory.h>
#include <SmString.h>
#include <SmGraphicsExtern.h>
#include <SmBrep.h>
#include <SmCurve.h>
#include <SmFace.h>
#include <SmPoly.h>
#include <SmBSplineSurface.h>
#include <SmSurface.h>
#include <SmMapPtrToPtr.h>
#include <SmVector3d.h>
#include <SmApiTypes.h>

SmContext* pGlobalContext = NULL;


/*******************************************************************//**
PURPOSE --- Create global context  

USAGE NOTES ---

***********************************************************************/
void SmApiCreateContext()
{
  if(pGlobalContext == NULL)
  {
    pGlobalContext = new SmContext();
  }
}

/*******************************************************************//**
PURPOSE --- Create global context  

USAGE NOTES ---

***********************************************************************/
SmContext* SmApiGetOrCreateContext()
{
  if(pGlobalContext == NULL)
  {
    pGlobalContext = new SmContext();
  }

  return pGlobalContext;
}

/*******************************************************************//**
PURPOSE --- Draw this brep  

USAGE NOTES --- Optionally center view on object

***********************************************************************/
SmApiStatus SmApiDraw
(
    SmBrep* pBrep,                 ///< [in ]: Draw this object
    const SmVector3d* pColor,      ///< [in ]: Color of object [Optional] default:[NULL]
    SmBoolean bClearFirst,         ///< [in ]: Clear viewport  [Optional] default:[false])
    SmBoolean bCenter              ///< [in ]: Center object   [Optional] default:[false]
)
{
  if(smGet_DoGraphics() == false)
    return(SM_SUCCESS);

  if(bClearFirst)
    smgfx_Erase();

  // Use input color if it exists, otherwise draw in black
  if(pColor == NULL)
    smgfx_SetLook( 2, 3, 0, 0, 0 );
  else
    smgfx_SetLook( 2, 3, pColor->x, pColor->y, pColor->z );

  if(bCenter)
  {
    SmExtent3d sBBox;
    pBrep->CalculateBoundingBox( sBBox );

    if(!sBBox.IsInit())
    {
      double dScale = 0.9;
      SmPoint3d sNormCenter( .5, .5, .5 );
      SmPoint3d sCenter = sBBox.Evaluate( .5, .5, .5 );
      smgfx_SetRotationCenter( sCenter );
      smgfx_ZoomWorldBox( sBBox, dScale, &sNormCenter );
    }
  }

  pBrep->Draw( SM_DM_CROSSHATCH );  sm_GraphicsLoop(0);

  return(SM_SUCCESS);

} // End SmApiDraw SmBrep

/*******************************************************************//**
PURPOSE --- Draw this brep  

USAGE NOTES --- Optionally center view on object

***********************************************************************/
SmApiStatus SmApiDraw
( 
    SmFace* pFace,                 ///< [in ]: Draw this object
    const SmVector3d* pColor,      ///< [in ]: Color of object [Optional] default:[NULL]
    SmBoolean bClearFirst,         ///< [in ]: Clear viewport  [Optional] default:[false]
    SmBoolean bCenter              ///< [in ]: Center object   [Optional] default:[false]
)   
{
	if (smGet_DoGraphics() == false)
		return(SM_SUCCESS);

    if( bClearFirst )
        smgfx_Erase();

    // Use input color if it exists, otherwise draw in black
    if( pColor == NULL )
        smgfx_SetLook(2,3, 0,0,0);
    else
        smgfx_SetLook(2,3, pColor->x, pColor->y, pColor->z );

    if( bCenter ) {
        SmExtent3d sBBox;
        pFace->CalculateBoundingBox( sBBox );

        if(!sBBox.IsInit()) {
            double dScale = 0.9;
            SmPoint3d sNormCenter(.5, .5, .5);
            SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
            smgfx_SetRotationCenter(sCenter) ;
            smgfx_ZoomWorldBox( sBBox, dScale, &sNormCenter ) ;
        }
    }
  
    pFace->Draw(SM_DM_CROSSHATCH);  sm_GraphicsLoop(0);

    return( SM_SUCCESS );

} // End SmApiDraw SmFace

/*******************************************************************//**
PURPOSE --- Draw this curve  

USAGE NOTES --- Optionally center view on object

***********************************************************************/
SmApiStatus SmApiDraw
( 
    SmCurve* pCurve,               ///< [in ]: Draw this object
    const SmVector3d* pColor,      ///< [in ]: Color of object [Optional] default:[NULL]
    SmBoolean bClearFirst,         ///< [in ]: Clear viewport  [Optional] default:[false]
    SmBoolean bCenter              ///< [in ]: Center object   [Optional] default:[false]
)   
{
	if (smGet_DoGraphics() == false)
		return(SM_SUCCESS);

    if( bClearFirst )
        smgfx_Erase();

    // Use input color if it exists, otherwise draw in black
    if( pColor == NULL )
        smgfx_SetLook(2,3, 0,0,0);
    else
        smgfx_SetLook(2,3, pColor->x, pColor->y, pColor->z );

    if( bCenter ) {
        SmExtent3d sBBox;
        pCurve->CalculateBoundingBox( pCurve->GetNaturalInterval(), &sBBox );

        if(!sBBox.IsInit()) {
            double dScale = 0.9;
            SmPoint3d sNormCenter(.5, .5, .5);
            SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
            smgfx_SetRotationCenter(sCenter) ;
            smgfx_ZoomWorldBox( sBBox, dScale, &sNormCenter ) ;
        }
    }
  
    pCurve->Draw(); sm_GraphicsLoop(0);

    return( SM_SUCCESS );

} // End SmApiDraw SmCurve


/*******************************************************************//**
PURPOSE --- Draw this surface  

USAGE NOTES --- Optionally center view on object

***********************************************************************/
SmApiStatus SmApiDraw
(
    SmSurface* pSurface,           ///< [in ]: Draw this object
    const SmVector3d* pColor,      ///< [in ]: Color of object [Optional] default:[NULL]
    SmBoolean bClearFirst,         ///< [in ]: Clear viewport  [Optional] default:[false]
    SmBoolean bCenter              ///< [in ]: Center object   [Optional] default:[false]
)
{
	if (smGet_DoGraphics() == false)
		return(SM_SUCCESS);

    if( bClearFirst )
        smgfx_Erase();

    // Use input color if it exists, otherwise draw in black
    if( pColor == NULL )
        smgfx_SetLook(2,3, 0,0,0);
    else
        smgfx_SetLook(2,3, pColor->x, pColor->y, pColor->z );

    if( bCenter ) {
        SmExtent3d sBBox;
        pSurface->CalculateBoundingBox( pSurface->GetNaturalUVDomain(), &sBBox );

        if(!sBBox.IsInit()) {
            double dScale = 0.9;
            SmPoint3d sNormCenter(.5, .5, .5);
            SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
            smgfx_SetRotationCenter(sCenter) ;
            smgfx_ZoomWorldBox( sBBox, dScale, &sNormCenter ) ;
        }
    }
  
    pSurface->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop(0);

    return( SM_SUCCESS );

} // End SmApiDraw SmSurface

/*******************************************************************//**
PURPOSE --- Draw this array of curves  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiDraw
( 
    const SmTArray<SmCurve*>& pArray,    ///< [in ]: SmObject to draw
    const SmVector3d* pColor,      ///< [in ]: Color of object [Optional] default:[NULL]
    SmBoolean bClearFirst,         ///< [in ]: Clear viewport  [Optional] default:[false]
    SmBoolean bCenter              ///< [in ]: Center object   [Optional] default:[false]
)    
{
	if (smGet_DoGraphics() == false)
		return(SM_SUCCESS);
	
	if( bClearFirst )
        smgfx_Erase();

    // Use input color if it exists, otherwise draw in black
    if( pColor == NULL )
        smgfx_SetLook(2,3, 0,0,0);
    else
        smgfx_SetLook(2,3, pColor->x, pColor->y, pColor->z );
    
    if( bCenter ) {
        SmExtent3d sBBox;
        // Must calculate bounding box of array of curves
        //pArrayCalculateBoundingBox( sBBox );

        if(!sBBox.IsInit()) {
            double dScale = 0.9;
            SmPoint3d sNormCenter(.5, .5, .5);
            SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
            smgfx_SetRotationCenter(sCenter) ;
            smgfx_ZoomWorldBox( sBBox, dScale, &sNormCenter ) ;
        }
    }

    for( ULONG ii = 0; ii < pArray.GetSize(); ii++ ) {
        pArray.GetAt(ii)->Draw();  sm_GraphicsLoop(0);
    }
           

    return( SM_SUCCESS );

} // End SmApiDraw Array of Curves

/*******************************************************************//**
PURPOSE --- Draw this array of points  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiDraw
( 
    const SmTArray<SmPoint3d>& pArray,   ///< [in ]: SmObject to draw
    const SmVector3d* pColor,      ///< [in ]: Color of object [Optional] default:[NULL]
    SmBoolean bClearFirst,         ///< [in ]: Clear viewport  [Optional] default:[false]
    SmBoolean bCenter              ///< [in ]: Center object   [Optional] default:[false]
)    
{
	if (smGet_DoGraphics() == false)
		return(SM_SUCCESS);
	
	if( bClearFirst )
        smgfx_Erase();

    // Use input color if it exists, otherwise draw in black
    if( pColor == NULL )
        smgfx_SetLook(2,4, 0,0,0);
    else
        smgfx_SetLook(2,4, pColor->x, pColor->y, pColor->z );
    
    if( bCenter ) {
        SmExtent3d sBBox;
        // Must calculate bounding box of array of curves
        //pArrayCalculateBoundingBox( sBBox );

        if(!sBBox.IsInit()) {
            double dScale = 0.9;
            SmPoint3d sNormCenter(.5, .5, .5);
            SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
            smgfx_SetRotationCenter(sCenter) ;
            smgfx_ZoomWorldBox( sBBox, dScale, &sNormCenter ) ;
        }
    }

    for( ULONG ii = 0; ii < pArray.GetSize(); ii++ ) {
        pArray.GetAt(ii).Draw();  sm_GraphicsLoop(0);
    }
           
    return( SM_SUCCESS );

} // End SmApiDraw Array of points

/*******************************************************************//**
PURPOSE --- Draw this vector 

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiDraw
(
    const SmPoint3d& rBasePt,      ///< [in ]: Base point of vector to draw
    const SmVector3d* pColor,      ///< [in ]: Color of object [Optional] default:[NULL]
    SmBoolean bClearFirst,         ///< [in ]: Clear viewport  [Optional] default:[false]
    SmBoolean bCenter              ///< [in ]: Center object   [Optional] default:[false]
)
{
	if (smGet_DoGraphics() == false)
		return(SM_SUCCESS);
	
	if( bClearFirst )
        smgfx_Erase();

    // Use input color if it exists, otherwise draw in black
    if( pColor == NULL )
        smgfx_SetLook(2,4, 0,0,0);
    else
        smgfx_SetLook(2,4, pColor->x, pColor->y, pColor->z );
    
    if( bCenter ) {
        SmExtent3d sBBox;
        // Must calculate bounding box of array of curves
        //pArrayCalculateBoundingBox( sBBox );

        if(!sBBox.IsInit()) {
            double dScale = 0.9;
            SmPoint3d sNormCenter(.5, .5, .5);
            SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
            smgfx_SetRotationCenter(sCenter) ;
            smgfx_ZoomWorldBox( sBBox, dScale, &sNormCenter ) ;
        }
    }

    rBasePt.Draw();
           
    return( SM_SUCCESS );

} // End SmApiDraw point

/*******************************************************************//**
PURPOSE --- Draw this vector 

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiDraw
(
    const SmPoint3d& rBasePt,      ///< [in ]: Base point of vector to draw
    const SmVector3d& rDirection,  ///< [in ]: Vector to draw
    const SmVector3d* pColor,      ///< [in ]: Color of object [Optional] default:[NULL]
    SmBoolean bClearFirst,         ///< [in ]: Clear viewport  [Optional] default:[false]
    SmBoolean bCenter              ///< [in ]: Center object   [Optional] default:[false]
)
{
	if (smGet_DoGraphics() == false)
		return(SM_SUCCESS);
	
	if( bClearFirst )
        smgfx_Erase();

    // Use input color if it exists, otherwise draw in black
    if( pColor == NULL )
        smgfx_SetLook(2,4, 0,0,0);
    else
        smgfx_SetLook(2,4, pColor->x, pColor->y, pColor->z );
    
    if( bCenter ) {
        SmExtent3d sBBox;
        // Must calculate bounding box of array of curves
        //pArrayCalculateBoundingBox( sBBox );

        if(!sBBox.IsInit()) {
            double dScale = 0.9;
            SmPoint3d sNormCenter(.5, .5, .5);
            SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
            smgfx_SetRotationCenter(sCenter) ;
            smgfx_ZoomWorldBox( sBBox, dScale, &sNormCenter ) ;
        }
    }

    rDirection.Draw( &rBasePt );
           
    return( SM_SUCCESS );

} // End SmApiDraw Vector

/*******************************************************************//**
PURPOSE --- Draw this polybrep  

USAGE NOTES --- 

***********************************************************************/
SmApiStatus SmApiDraw
( 
    SmPolyBrep* pPolyBrep,         ///< [in ]: SmObject to draw
    const SmVector3d* pColor,      ///< [in ]: Color of object [Optional] default:[NULL]
    SmBoolean bClearFirst,         ///< [in ]: Clear viewport  [Optional] default:[false]
    SmBoolean bCenter              ///< [in ]: Center object   [Optional] default:[false]
)    
{

	if (smGet_DoGraphics() == false)
		return(SM_SUCCESS);
	
	if( bClearFirst )
        smgfx_Erase();

    // Use input color if it exists, otherwise draw in black
    if( pColor == NULL )
        smgfx_SetLook(2,3, 0,0,0);
    else
        smgfx_SetLook(2,3, pColor->x, pColor->y, pColor->z );

    if( bCenter ) {
        SmExtent3d sBBox;
        pPolyBrep->CalculateBoundingBox( sBBox );

        if(!sBBox.IsInit()) {
            double dScale = 0.9;
            SmPoint3d sNormCenter(.5, .5, .5);
            SmPoint3d sCenter = sBBox.Evaluate(.5,.5,.5) ;
            smgfx_SetRotationCenter(sCenter) ;
            smgfx_ZoomWorldBox( sBBox, dScale, &sNormCenter ) ;
        }
    }

    pPolyBrep->Draw();
    sm_GraphicsLoop();

    return( SM_SUCCESS );

} // End SmApiDraw SmPolyBrep

//////////////////////////////////////////////////
// END DRAW FUNCTIONS
//////////////////////////////////////////////////

//////////////////////////////////////////////////
// BEGIN TRANSFORMATION FUNCTIONS
//////////////////////////////////////////////////

namespace
{

SmBoolean sm_IsUniformPositiveScale(const SmVector3d& rScale)
{
    return (   SM_ARE_SAME(rScale.x, rScale.y)
            && SM_ARE_SAME(rScale.x, rScale.z)
            && rScale.x > SM_EFF_ZERO );
}

SmApiStatus sm_ReportInvalidTransformInput(const TCHAR* pMessage)
{
    // SE_MSG is debug-only. Report directly so release callers with an error
    // callback, including _omni_solid, receive the actionable reason.
    smos_ErrorMessage( SM_ERR_INVALID_INPUT, FILE_NAME, LINE_NUMBER,
                       pMessage, NULL, FUNC_NAME );
    return SM_ERR_INVALID_INPUT;
}

SmApiStatus sm_TransformIndependentObject
(
    SmObject* pObj,
    const SmAxis2Placement& rTransform,
    const SmVector3d* pOptScale
)
{
    if( pObj == NULL )
        return SM_ERR_INVALID_INPUT;

    if(   pOptScale != NULL
       && (   pOptScale->IsUndef()
           || pOptScale->GetMinDimension() <= SM_EFF_ZERO ) )
    {
        return SM_ERR_INVALID_INPUT;
    }

    if( pObj->IsKindOf(SmBrep_TYPE) )
        return ((SmBrep*)pObj)->Transform( rTransform, pOptScale );

    if( pObj->IsKindOf(SmPolyBrep_TYPE) )
    {
        // The stable API publishes only positive, non-degenerate PolyBrep
        // scales. The lower-level kernel retains its legacy support
        // for signed and singular transforms used by existing callers.
        if(   pOptScale != NULL
           && (   pOptScale->x <= SM_EFF_ZERO
               || pOptScale->y <= SM_EFF_ZERO
               || pOptScale->z <= SM_EFF_ZERO ) )
        {
            return SM_ERR_INVALID_INPUT;
        }

        return ((SmPolyBrep*)pObj)->Transform( rTransform, pOptScale );
    }

    if( pObj->IsKindOf(SmCurve_TYPE) )
    {
        SmCurve* pCurve = (SmCurve*)pObj;
        SM_TYPE eCurveType = pCurve->GetType();
        if( pCurve->GetOwner() != NULL )
        {
            return sm_ReportInvalidTransformInput(
                _T("Cannot transform a Brep-owned Curve independently; "
                   "transform the owning Brep instead.") );
        }

        if(   eCurveType == SmCrvInVolume_TYPE
           || eCurveType == SmCrvOnSurf_TYPE
           || eCurveType == SmProjectedCurve_TYPE
           || eCurveType == SmCompositeCurve_TYPE )
            return SM_ERR_INVALID_INPUT;

        // Specialized curve representations receive only positive, uniform
        // scaling through this API. Preflight before virtual dispatch so a
        // rejected scale cannot leave the curve partially modified.
        if(   pOptScale != NULL
           && eCurveType != SmBSplineCurve_TYPE
           && eCurveType != SmLine_TYPE
           && !sm_IsUniformPositiveScale(*pOptScale) )
        {
            return sm_ReportInvalidTransformInput(
                _T("Cannot transform this standalone Curve in place with "
                   "these scale factors; this API permits only positive, "
                   "uniform scale factors for its runtime type and does not "
                   "replace standalone geometry automatically. Use a generic "
                   "B-spline Curve representation for non-uniform or negative "
                   "scaling.") );
        }

        return pCurve->Transform( rTransform, pOptScale );
    }

    if( pObj->IsKindOf(SmSurface_TYPE) )
    {
        SmSurface* pSurface = (SmSurface*)pObj;
        if( pSurface->GetOwner() != NULL )
        {
            return sm_ReportInvalidTransformInput(
                _T("Cannot transform a Brep-owned Surface independently; "
                   "transform the owning Brep instead.") );
        }

        if( pSurface->GetType() == SmSrfInVolume_TYPE )
            return SM_ERR_INVALID_INPUT;

        // Specialized surface representations receive only positive, uniform
        // scaling through this API. Some update their NURBS representation
        // before reporting an error, so reject unsupported scaling first.
        if(   pOptScale != NULL
           && pSurface->GetType() != SmBSplineSurface_TYPE
           && !sm_IsUniformPositiveScale(*pOptScale) )
        {
            return sm_ReportInvalidTransformInput(
                _T("Cannot transform this standalone Surface in place with "
                   "these scale factors; this API permits only positive, "
                   "uniform scale factors for its runtime type and does not "
                   "replace standalone geometry automatically. Use a generic "
                   "B-spline Surface representation for non-uniform or "
                   "negative scaling.") );
        }

        return pSurface->Transform( rTransform, pOptScale );
    }

    return SM_ERR_INVALID_INPUT;
}

} // namespace

/*******************************************************************//**
PURPOSE --- Transform this object.

USAGE NOTES --- When scaling in addition to transforming
                It is a good idea to call SmScale first

***********************************************************************/
SmApiStatus SmApiTransform
(
    SmObject* pObj,                 ///< [in ]: Transform this object
    const SmAxis2Placement & rRotateMove ///< [in ]: Transformation (rotate and translate)
)
{
    return sm_TransformIndependentObject( pObj, rRotateMove, NULL );

}

/*******************************************************************//**
PURPOSE --- Scale this object.

USAGE NOTES --- See also SmApiScaleByPt

***********************************************************************/
SmApiStatus SmApiScale
(
    SmObject* pObj,                 ///< [in ]: Scale this object
    const SmVector3d& rScale       ///< [in ]: Scale factor in x, y, z
)
{
    SmAxis2Placement sMatrix;
    return sm_TransformIndependentObject( pObj, sMatrix, &rScale );
}

/*******************************************************************//**
PURPOSE --- Scale this object with respect to a point.

USAGE NOTES --- This differs from SmScale because it translates the
                object to the origin, performs the scale, and translates
                the object back to its original position

***********************************************************************/
SmApiStatus SmApiScaleByPt
(
    SmObject* pObj,                 ///< [in ]: Scale this object
    const SmVector3d& rRefPt,       ///< [in ]: Reference pt to translate to origin
    const SmVector3d& rScale       ///< [in ]: Scale factor in x, y, z
)
{
    if(   rScale.IsUndef()
       || rScale.GetMinDimension() <= SM_EFF_ZERO )
    {
        return SM_ERR_INVALID_INPUT;
    }

    // Transform applies scale before placement. Compose
    // x' = scale*x + ref*(1-scale) into one operation so a rejected scale
    // cannot leave the object partially translated.
    SmVector3d sTranslate(rRefPt.x * (1.0-rScale.x),
                          rRefPt.y * (1.0-rScale.y),
                          rRefPt.z * (1.0-rScale.z));
    SmAxis2Placement sTransform;
    sTransform.Translate( sTranslate );
    return sm_TransformIndependentObject( pObj, sTransform, &rScale );
}

/*******************************************************************//**
PURPOSE --- Translate this object  

USAGE NOTES ---

***********************************************************************/
SmApiStatus SmApiTranslate
(
    SmObject* pObj,                 ///< [in ]: Translate this object
    const SmVector3d& crTranslate  ///< [in ]: Translation vector
)
{
    SmAxis2Placement sTransform;
    sTransform.Translate( crTranslate );
    return sm_TransformIndependentObject( pObj, sTransform, NULL );
}

/*******************************************************************//**
PURPOSE --- Rotate this object about an axis through a reference point.

USAGE NOTES --- The object is rotated by ``dAngleDeg`` degrees about the
                line through ``rRefPt`` in the direction ``rAxis``.

***********************************************************************/
SmApiStatus SmApiRotate
(
    SmObject* pObj,                  ///< [in ]: Rotate this object
    const SmVector3d& rRefPt,        ///< [in ]: Point on the rotation axis (fixed point)
    const SmVector3d& rAxis,         ///< [in ]: Direction of the rotation axis
    double dAngleDeg                ///< [in ]: Angle of rotation, in degrees
)
{
    if( pObj == NULL )
        return SM_ERR_INVALID_INPUT;

    // Normalize the rotation axis.  SmAxis2Placement::RotateAboutAxisAtPoint
    // asserts on a non-unit axis; callers historically passed any non-zero
    // direction, so we unitize here and fail gracefully on a zero vector.
    SmVector3d sAxis = rAxis;
    if( sAxis.Unitize() != SM_SUCCESS )
    {
        SE_MSG( SM_ERR_INVALID_INPUT, _T("SmApiRotate: rotation axis has zero length") );
        return( SM_ERR_INVALID_INPUT );
    }

    SmAxis2Placement sTransform;   // identity
    sTransform.RotateAboutAxisAtPoint(SM_DEGREES_TO_RADIANS( dAngleDeg ), rRefPt, sAxis );

    return sm_TransformIndependentObject( pObj, sTransform, NULL );
}

//////////////////////////////////////////////////
// END TRANSFORMATION FUNCTIONS
//////////////////////////////////////////////////
