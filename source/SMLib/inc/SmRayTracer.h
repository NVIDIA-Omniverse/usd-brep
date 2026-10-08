// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmRayTracer.h
* PURPOSE: Header file for SmRayTracer object.
**********************************************************************/

#ifndef __SMRAYTRACER_H__
#define __SMRAYTRACER_H__

#ifndef __SMGRID_H__
#include <SmGrid.h>
#endif

#ifndef __SMGLOBALSOLVER_H__
#include <SmGlobalSolver.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

#ifndef __SMSURFACE_H__
#include <SmSurface.h>

#ifndef __SMSURFACECACHE_H__
#include <SmSurfaceCache.h>
#endif

#ifndef __SMTRIMSRFCACHE_H__
#include <SmTrimSrfCache.h>
#endif
#endif

#ifndef __SMEXTENT2D_H__
#include <SmExtent2d.h>
#endif

#ifndef __SMPOLY_H__
#include <SmPoly.h>
#endif

class SmRayTracer;
class SmRayRenderer;
class SmLightSource;

/*******************************************************************//**
PURPOSE: This class represents a light source.

NOTES: Right now we just assume a white light source that
   is either at a point or in a given direction.
***********************************************************************/
class SM_EXPORT SmLightSource : public SmObject
{
  public:
    SmVector3d   m_vLightDirectionOrOrigin;   // light position     
    SmBoolean    m_bIsPointLightSource;       // TRUE = Point source of light
                                              // FALSE= diffuse light     

  public:
    // constructor/destructor
    SmLightSource(const SmVector3d & crLightDirOrOrigin,
        SmBoolean bIsPointLight) 
        : m_vLightDirectionOrOrigin(crLightDirOrOrigin), 
        m_bIsPointLightSource(bIsPointLight) {}

   virtual ~SmLightSource() {}

} ; // end class SmLightSource


// GWC:BIND TEMPLATES_MOVE  SM_TARRAY_TEMPLATE_PREDECLARATION(SmLightSource*) ;


/*******************************************************************//**
PURPOSE: This class is used to do local iteration of ray/surface
    intersection.

NOTES: This Derived class is currently only used by the
    SmRayTracer Object

   Tolerances are not completely consistent within this class.
   m_d3dTolerance and m_dDistanceTolerance are both used for line-surface intersections.
   m_deDistanceTolerance could be used as a step-off distance: do not return solutions that
   are closer to m_vLinePoint than this: callers often do that themselves.
***********************************************************************/
class SM_EXPORT SmLSI3LocalSolver 
{
  public:
    double             m_d3dTolerance;          // Near-miss tolerance, used for line intersections

    SmExtent2d         m_sUVDomain;             //      
    SmPoint3d          m_vLinePoint;            //      
    SmVector3d         m_vLineVector;           //      
    SmExtent1d         m_sLineInterval;         //      
    SmBoolean          m_bRayFire;              //      
    double             m_dDistanceTolerance;    // (See note above.)
    double             m_dRayAccuracy;          // Convergence tolerance for line-surface intersections
    SmBoolean          m_bHaveTMinTMax;         //      
    double             m_dTMinOfBBox;           //      
    double             m_dTMaxOfBBox;           // 
    
    // constructor     
    SmLSI3LocalSolver()  
  : m_d3dTolerance(0.0),
    m_sLineInterval(0.0, 1.0e10)
  { }

    // constructor
    SmLSI3LocalSolver(SmTree            * pSurfaceTree,        // NotUsed: in : 
                       const SmExtent2d & crUVDomain,          // in : 
                       const SmPoint3d  & crLinePoint,         // in : 
                       const SmVector3d & crLineVector,        // in : 
                       const SmExtent1d * cpLineInterval,      // in : 
                       SmBoolean          bRayFire,            // in : 
                       double             dDistanceTolerance,  // in : 
                       double             dRayAccuracy);       // in : 

    // destructor
    ~SmLSI3LocalSolver()      { }

    // a Ray/GridElement (a SurfDecompositionBezierPatch) intersection NewtonRaphson Solver
    SmStatus LocalSolve(SmSurfaceGridElement * pSurfaceGridElement,
                        SmBoolean  & rbNeedsMoreSubdivision,
                        SmBoolean  & rbFoundIntersection,
                        SmSolution & rSolution);

} ; // end class SmLSI3LocalSolver

/*******************************************************************//**
PURPOSE: This class is the primary class the does the grid setup and
   firing of rays into geometry contained in the grid.

NOTES: 
***********************************************************************/
class SM_EXPORT SmRayTracer : public SmObject
{
  protected:
    const SmContext         & m_crContext;         // context for all objects contained in this object     
    double                    m_dRayTolerance;     //      
    double                    m_dRayAccuracy;      //      
    SmGrid                  * m_pGrid;             // Voxel Model with ptrs back to Surface and SurfaceCacheBezierPatch geometry    
    SmLSI3LocalSolver         m_sLSISolver;        // Solver used to get Ray/SurfaceCacheBezierPatch intersections     

    SmTArray<SmSurface*>      m_sSurfaces;         // Array of Surfaces added to m_pGrid 


    SmTArray<SmPolyBrep*>     m_sPolyBreps;        //      

  public:
    SmGridElement           * m_pSkipElement;      //      
    SmBoolean                 m_bHitAnyThing = false;      // TRUE = Has hit something 
                                                           // FALSE= Hasn't

  public:
    // constructor/destructor
    SmRayTracer(const SmContext & crContext,
                SmGrid *pGrid,
                double dRayTolerance,
                double dRayAccuracy);
    virtual ~SmRayTracer();

    // access
    SmGrid *GetGrid() const                       { return m_pGrid ; }
    void    SetSolverTolerance( double dTol )     { m_sLSISolver.m_d3dTolerance = dTol; }

    // Intersect a ray with the geometry stored as elements within the Grid
    SmStatus FireRay(const SmPoint3d          & crRayPoint,           // in : ray start point
                     const SmVector3d         & crRayVector,          // in : ray direction
                     double                     dMaxRayParameter,     // in : stop param for ray traversal, use SM_BIG_DOUBLE for infinite rays
                     SmBoolean                & rbRayHitsSomething,   // out: TRUE = Hit Something, FALSE = Didn't
                     SmSolution               & rSolution,            // out:  rSolution.m_vStart[0] = T parameter of line                  
                                                                      //       rSolution.m_vStart[1] = U of UVPoint    (SM_BIG_DOUBLE for PolyFaces)      
                                                                      //       rSolution.m_vStart[2] = V of UVPoint    (SM_BIG_DOUBLE for PolyFaces)                          
                                                                      //       rSolution.m_vStart[3] = X of 3D point  
                                                                      //       rSolution.m_vStart[4] = Y of 3D point                              
                                                                      //       rSolution.m_vStart[5] = Z of 3D point                              
                                                                      //       rSolution.m_vStart[6] = n.X of 3D Normal Vector       
                                                                      //       rSolution.m_vStart[7] = n.Y of 3D Normal Vector                              
                                                                      //       rSolution.m_vStart[8] = n.Z of 3D Normal Vector                              
                                                                      //       rSolution.m_apNodes[0]   =  
                                                                      //       rSolution.m_apObjects[0] = Surface, edge, vertex or PolyFace pointer
                                                                      //       rSolution.m_apObjects[1] = associated Topology pointer
                                                                      //                                  could be SmFace, SmEdge, or SmVertex, or NULL
                     SmTArray<SmGridElement*> & rGridElements,        // out: list of ray/GridElementBBox intersections 
                     SmTArray<double>         & rMinDistances,        // out: associated ray/GridElementBBox intersection ray start param value 
                     SmTArray<double>         & rMaxDistances);       // out: associated ray/GridElementBBox intersection ray end param value 
                                                                                                                                    
      // embed surf as set of elements in RayTracer->grid  
      SmStatus AddSurfaceToGrid(SmSurface *pSurface);

      // embed PolyBrep polygon faces in RayTracer->Grid
      SmStatus AddPolyBrepToGrid(SmPolyBrep * pPolyBrep,
                                 double dExpansionTolerance);

      ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,                       // out:
                          SmBoolean  bAddSurfaceMemory        =TRUE,          // in :
                          ULONG    * pOptGlobalCacheAllocated =NULL,          // NotUsed: out:
                          ULONG    * POptGridAllocated        =NULL,          // out:
                          ULONG    * POptSurfaceArrayAllocated=NULL,          // out:
                          ULONG    * POptSurfaceCacheAllocated=NULL,          // out:
                          ULONG    * pOptGlobalCacheUsed      =NULL,          // NotUsed: out:
                          ULONG    * POptGridUsed             =NULL,          // out:
                          ULONG    * POptSurfaceArrayUsed     =NULL,          // out:
                          ULONG    * POptSurfaceCacheUsed     =NULL) const ;  // out:

      virtual void Dump() const ;


} ; // end class SmRayTracer

/*******************************************************************//**
PURPOSE: This is the high level object which creates and displays
   an image using ray tracing.

NOTES: The definition of the view space is a little different
   but can easily be changed to more traditional forms.
***********************************************************************/
class SM_EXPORT SmRayRenderer : public SmObject
{
  protected:
    SmRayTracer    * m_pRayTracer;            // Pointer to the ray tracer containing voxel grid of geometry elements
    SmAxis2Placement m_vGeometryTransform;    // Here is how to move geometry into
                                              // the view volume.
    SmAxis2Placement m_vInvTransform;         // Inverse of the view transformation.
    SmExtent3d       m_vViewVolume;           // We are always looking toward - z axis
    ULONG            m_lNumXPixels;           // 0,0 pixel is upper left corner.
    ULONG            m_lNumYPixels;          
    ULONG            m_lSampleRate;           // Number of samples in a given direction on the
                                              // pixel.  2 produces 4 samples on a pixel.
    SmTArray<SmLightSource*> m_vLightSources; // Current light sources

    SmBoolean        m_bDoShadows;            // If TRUE we cast shadows.
    SmVector3d       m_vBackgroundColor;      //      
    SmVector3d       m_vDefaultObjectColor;   //      
    SmVector3d       m_vDefaultAmbientColor;  //      
    ULONG            m_lPixelsShaded;         //      
    SmTArray<SmGridElement*> m_vGridElements; //      
    SmTArray<double>         m_vMinDistances; //      
    SmTArray<double>         m_vMaxDistances; //      

  public:
    // constructor/destructor
    SmRayRenderer(SmRayTracer * pRayTracer,
                  const SmAxis2Placement & crGeometryTransform,
                  const SmExtent3d & crViewVolume,
                  ULONG lNumXPixels,
                  ULONG lNumYPixels);

    virtual ~SmRayRenderer();

    // simple data access
    ULONG GetPixelsShaded()                  { return m_lPixelsShaded; }
    void  SetSampleRate  (ULONG lSampleRate) { m_lSampleRate = lSampleRate; }

    // Add light source to list
    void AddLightSource(SmLightSource *pLightSource) { m_vLightSources.Add(pLightSource); }

    //
    SmStatus DoRender(SmBoolean bDoShadows,
                      const SmVector3d & crBackgroundColor,
                      const SmVector3d & crDefaultObjectColor);

    //
    SmStatus RenderRay(const SmVector3d & crRayPoint,
                       const SmVector3d & crRayVector,
                       SmVector3d & rRayColor);

    //
    SmStatus RenderPixel(ULONG lPixelX,
                         ULONG lPixelY,
                         SmVector3d & rPixelColor);

} ; // end class SmRayRenderer

#endif // !__SMRAYTRACER_H__


