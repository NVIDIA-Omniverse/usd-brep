// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCacheMgrTSrf.cpp
* PURPOSE: Source file for curve cache manager methods.
**********************************************************************/

#include "StdAfx.h"

#ifdef SM_USE_GLOBAL_CACHE

#include <SmCacheMgrTSrf.h>

/*******************************************************************//**
PURPOSE: This is the constructor for the curve cache manager.  

NOTES: 
***********************************************************************/
SmCacheMgrTSrf::SmCacheMgrTSrf
  (const SmContext & crContext,                  
   ULONG lCurveCacheCount,            // in : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
   ULONG lSurfaceCacheCount,          // in : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
   ULONG lTrimSrfCacheCount,          // in : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
   ULONG lBrepCacheCount,             // in : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                     
   ULONG lCurveCacheByteSize,         // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
   ULONG lSurfaceCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
   ULONG lTrimSrfCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
   ULONG lBrepCacheByteSize)          // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
 : SmCacheMgrSrf(crContext,
                 lCurveCacheCount,
                 lSurfaceCacheCount,
                 lTrimSrfCacheCount,
                 lBrepCacheCount,

                 lCurveCacheByteSize,  
                 lSurfaceCacheByteSize,
                 lTrimSrfCacheByteSize,
                 lBrepCacheByteSize)   
{
}

/*******************************************************************//**
PURPOSE: This method validates an existing cache and/or creates
   a new cache for a trimmed surface.

NOTES:
 
The returned ObjectCache stores a pointer to the target Object.
The returned ObjectCache is not placed in the global cache queues.

The value of eObjectCacheType determines which type of ObjectCache 
is constructed.  A call to any SmCacheMgrDerivedType::CacheMakeOrValidate()
function starts by checking this value.  If it is appropriate an ObjectCache
pointer is returned else the call is passed onto the next level until 
a match between derived type and requested type is found. When no match
is found NULL is returned. 

The order of calls and return ObjectCache types are:

     call SmCacheMgrBrep::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_BREP)      return SmBrepCache
else call SmCacheMgrTSrf::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_TRIMSRF)   return SmTrimSrfCache
else call SmCacheMgrSrf::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_SURFACE)   return SmSurfaceCache
else call SmCacheMgrCrv::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_CURVE)     return SmCurveCache
else call SmCacheMgr::CacheMakeOrValidate 
                                                     return NULL

***********************************************************************/
SmStatus SmCacheMgrTSrf::CacheMakeOrValidate
  (SmObjectCacheType eObjectCacheType,       // in : oneof SM_OC_CURVE
                                             //            SM_OC_SURFACE        
                                             //            SM_OC_TRIMSRF
                                             //            SM_OC_BREP
   const SmObject  * cpObject,               // in : target object
   SmCacheObj      * cpOldCache,             // in : existing target Object's ObjectCache or NULL
   SmCacheObj     *& rpNewCache)             // out: ptr to target object's ObjectCache
{
    // init output
    rpNewCache = NULL;

    // When requested ObjectCache Type is not TrimmedSurface try SrfCache Construction 
    if (eObjectCacheType != SM_OC_TRIMSRF) 
      { 
        return SmCacheMgrSrf::CacheMakeOrValidate(eObjectCacheType,
                                                  cpObject,
                                                  cpOldCache,
                                                  rpNewCache);
      }

// Remove Composites
// // cast Object to SmSurface and get its Owner (oneof SmFace, SmCFace, or NULL)
   // cast Object to SmSurface and get its Owner (oneof SmFace or NULL)
    const SmSurface *cpSurface  = (const SmSurface*)cpObject ;   SM_ASSERT (cpSurface != NULL);
    SmObject        *pOwner     = cpSurface->GetOwner();
    SmFace          *pFace      = SM_CAST_PTR(SmFace, pOwner) ;

    // If made it to here then we are to process surface caches

    // low work - use OldCache when its given and current
    SmTrimSrfCache *pOldTrimSrfCache = (SmTrimSrfCache*)cpOldCache;
    if (pOldTrimSrfCache) 
      {
        SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;

        // If the face was modified and Brep editing is no longer enabled
        // then delete the old cache and create a new one.
        if (   pOldTrimSrfCache->m_bFaceWasModified  
            && pBrep
            && pBrep->m_bEditingEnabled == FALSE) 
          {
            SmCacheMgr::DeleteObjectsCache(eObjectCacheType,cpObject);
            pOldTrimSrfCache = NULL;
          }
        else // the Face was not modified
             // or m_bEditingEnabled == TRUE,
             // then use the old cache
          {
            rpNewCache = (SmCacheObj*)pOldTrimSrfCache;
            return SM_SUCCESS;
          }
      } // end pOldCache existence check
    
    // next create a new SmTrimSrfCache and store it with the surface.

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
//      static ULONG lAvecSurfaceCache = 0 ;
//      static ULONG lSansSurfaceCache = 0 ;
//      
//          // Get an existing cache if one exists for this object else return NULL.
//          SmSurfaceCache *pSurfaceCache = (SmSurfaceCache *)SmCacheMgr::GetObjectCache(SM_OC_SURFACE, cpSurface) ;
//          if(pSurfaceCache)
//            { lAvecSurfaceCache++ ; }
//          else
//            { lSansSurfaceCache++ ; }
//          if(   bDebugMe
//             || (lAvecSurfaceCache + lSansSurfaceCache) % 20 == 0)
//            {
//              TCHAR sBuff[SM_TBLOCK_SIZE] ;
//              smos_sprintf(sBuff,_T("\n TrimSrfCache creates[%d] with SurfaceCache[%d], without SurfaceCache[%d]"),
//                         lSansSurfaceCache + lAvecSurfaceCache,
//                         lAvecSurfaceCache,
//                         lSansSurfaceCache) ;
//              smos_WriteBuffer(sBuff) ;    // <=== GWC: output is commented out!
//            } 
#endif // SM_DEBUG_CODE

    // Set tolerances: angle and chord height.
    double dAngleTol = 20.0*SM_PI / 180.0;
    
    // Use surface size estimate to set chordHeight tolerance.
    // Get an idea of the surface's size.
    // Get some point positions scattered over the Surface's Natural Domain.
    SmExtent2d sUVDomain = cpSurface->GetNaturalUVDomain();

    SmPoint3d sPMid, sP00, sP01, sP10, sP11;
    SER( cpSurface->EvaluatePoint( sUVDomain.Evaluate(0.5,0.5), sPMid ));
    SER( cpSurface->EvaluatePoint( sUVDomain.GetMin(),          sP00 ));
    SER( cpSurface->EvaluatePoint( sUVDomain.Evaluate(0.9,0.1), sP10 ));
    SER( cpSurface->EvaluatePoint( sUVDomain.Evaluate(0.1,0.9), sP01 ));
    SER( cpSurface->EvaluatePoint( sUVDomain.GetMax(),          sP11 ));

    double dEstimatedSize = (  sPMid.DistanceBetween( sP00 )
                             + sPMid.DistanceBetween( sP11 )
                             + sPMid.DistanceBetween( sP10 )
                             + sPMid.DistanceBetween( sP01 ) ) / 2.0;
    
    // Make tessellation CH 1/40th size of surface
    double dChordHeightTol = dEstimatedSize / 40.0;
    
    // Loosen tolerances on analytic surfaces - they don't need as much subdivision.
    if (   cpSurface->GetDegree(SM_SP_U) <= 2
        && cpSurface->GetDegree(SM_SP_V) <= 2) 
      {
        dChordHeightTol = dEstimatedSize / 15.0;  
        dAngleTol = 2.0*dAngleTol;
      }

    // construct new SurfaceCache Object - mark it for delete if a failure exits scope
    const SmContext *cpContext     = cpObject->GetContext() ;   NER(cpContext);
    SmTrimSrfCache  *pTrimSrfCache = new(*cpContext) SmTrimSrfCache(*cpSurface,
                                                                    dChordHeightTol,
                                                                    dAngleTol);
                                                                 // dAspectRatio3D        = 0.0,   
                                                                 // dMaxSideLength3D      = 0.0,   
                                                                 // dMinSideLength3D      = 0.0,   
                                                                 // dMinSideLengthRatioUV = 0.001);
    NER(pTrimSrfCache);
    SmObjDelete sCleanup(pTrimSrfCache);

    // Connect surface and cache prior to doing all of the curve dropping
    // and creation of the trimmed surface cache.
    //
    // gwc note: I believe this prevents the upcoming SmFace::CreateUVTrimCurves()
    //           call from causing nested CacheMakeOrValidate(SM_OC_TRIMSRF, ...) calls
    //           when that function starts building UVTrimCurves by dropping
    //           3d curves to surfaces which generate calls to global solvers which
    //           then make GetOrMakeObjectCache() calls which can recurse to this
    //           function when no entry is found in the SM_OC_TRIMSRF global cache queue
    //           for cpSurface.
    SmCacheMgr::AddToObjectCache(SM_OC_TRIMSRF,cpSurface,pTrimSrfCache);

    // when we are not doing a boolean operation
    //  - during a boolean operation delay surface cache tree construction
    //    until some function calls SmSurfaceCache::GetTree()
    if (cpContext->GetDoingBoolean() == FALSE) 
      {
        // build the surface subdivision tree annotated with UVTrimBoundary classifications
        if (pTrimSrfCache->BuildTree() != SM_SUCCESS) 
          {
            // when construction failed - clean up and quit
            sCleanup.Clear();
            SmCacheMgr::DeleteObjectsCache(SM_OC_TRIMSRF,cpSurface);
            SER(SM_ERR);
          }
      } // end not doing a Brep-Boolean-operation check

#ifdef SM_DEBUG_CODE
    if (bDebugMe) 
      {
        pTrimSrfCache->Dump() ;

        pTrimSrfCache->Draw(); sm_GraphicsLoop();
        smgfx_Erase();
        ((SmTrimSrfCache*)pTrimSrfCache)->DrawSubdivision2D(TRUE); sm_GraphicsLoop();  
        sm_GraphicsLoop();
      }
    
    if (FALSE) 
      {
        SmTArray<SmTreeNode*> sNodes;
        pTrimSrfCache->GetTree()->GetAllTreeNodes(sNodes);
        ULONG lMemoryUsed, lMemoryAllocated ;
        lMemoryUsed = pTrimSrfCache->GetMemoryUsed(lMemoryAllocated) ;
        TCHAR sBuff[SM_TBLOCK_SIZE];
        smos_sprintf(sBuff,_T("Surface Tessellation: node count = %ld, Memory Used/Alloc = %ld/%ld bytes\n"),
            sNodes.GetSize(),lMemoryUsed, lMemoryAllocated);
        smos_WriteBuffer(sBuff);
      }
#endif
    
    // subdivision worked - clear the delete mark and set output
    sCleanup.Clear();
    rpNewCache = (SmCacheObj*)pTrimSrfCache;

    // all done
    return SM_SUCCESS;

} // end SmCacheMgrTSrf::CacheMakeOrValidate

#endif //SM_USE_GLOBAL_CACHE

