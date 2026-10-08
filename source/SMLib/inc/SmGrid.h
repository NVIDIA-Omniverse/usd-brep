// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmGrid.h
* PURPOSE: Header file for SmGrid object.
**********************************************************************/

#ifndef __SMGRID_H__
#define __SMGRID_H__

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMMEMBLOCKMGR_H__
#include <SmMemBlockMgr.h>
#endif

#ifndef __SMSARRAY_H__
#include <SmSArray.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

#include <SmPseudoBox.h>
#include <SmPolarBox.h>
#include <SmTree.h>

class SmHierarchyElement;
class SmGridElement;
class SmGrid;
class SmVoxel;


/*******************************************************************//**
PURPOSE: This class contains subdivision tolerances for subdividing
    things that go into grids.

NOTES: 
***********************************************************************/
class SM_EXPORT SmSubdivisionTolerances
{
  public:
    double m_dChordHeightTolerance;          //      
    double m_dAngTolDeg;                    //      
    double m_dAspectRatio3D;                 //      
    double m_dMaxSideLength3D;               //      
    double m_dMinSideLength3D;               //      
    double m_dMinSideLengthRatioUV;          //      

  public:
    SmSubdivisionTolerances(double dChordHeightTolerance  = 0.0,
                            double dAngTolDeg             = 20.0,
                            double dAspectRatio3D         = 0.0,
                            double dMaxSideLength3D       = 0.0,
                            double dMinSideLength3D       = 0.0,
                            double dMinSideLengthRatioUV  = 0.001) 
    : m_dChordHeightTolerance(dChordHeightTolerance),
      m_dAngTolDeg(dAngTolDeg),
      m_dAspectRatio3D(dAspectRatio3D), 
      m_dMaxSideLength3D(dMaxSideLength3D),
      m_dMinSideLength3D(dMinSideLength3D),
      m_dMinSideLengthRatioUV(dMinSideLengthRatioUV) { }

    ~SmSubdivisionTolerances() {}

} ; // end class SmSubdivisionTolerances

/*******************************************************************//**
PURPOSE: different kinds of GridElements stored within Grid Voxels

NOTES: 
***********************************************************************/
enum SmGridElementType {
    SM_GE_SURFACE,       // passed to SmGrid::AddElement for inserting SmTreeNodes from Surface decopmpositions
    SM_GE_POLYBREP,      // passed to SmGrid::AddElement for inserting SmTreeNodes from PolyBrep decompositions
    SM_GE_HIERARCHY,     //
    SM_GE_USERDATA       // passed to SmGrid::AddElement for inserting 

} ; // end enum SmGridElementType

/*******************************************************************//**
PURPOSE: This object represents a single element in the grid. 
    Note that one element may reside in several voxels.  

NOTES: 
  base class SmGridElement                 // root class for the following
    derived class SmSurfaceGridElement     // stores: Surface Pointer and surface decomposition leaf cache data
    derived class SmPolygonGridElement     // stores: PolyBrep decomposition SmTreeNode Pointer
    derived class SmHierarchyGridElement   // stores: pointer to owning SmHierarchyElement of 
                                           //          derived type SmSurfaceHierarchyElement
    derived class SmUserGridElement        // stores: void * pointer to whatever a user Application wants to store in a grid voxel

  Base and derived SmGridElement objects are managed by the old
  SmMemBlockMgr memory management scheme.  As such large blocks of memory
  are allocated and cast to the appropriate object type without setting
  the virtual function table.  So SmGridElement and its derived types
  cannot support virtual functions unless the memory manager is replaced
  with calls to new().

  static class object - don't add virtual methods to SmGridElement
***********************************************************************/
class SM_EXPORT SmGridElement

{     friend class SmGrid;
  protected:
    ULONG        m_lMark;               // Marking mechanism, matched with SmGrid::m_lCurrentMark
    SM_TYPE      m_lType;               // derived type identifier
                                        // one of: SmSurfaceGridElement_TYPE  
                                        //         SmPolygonGridElement_TYPE  
                                        //         SmHierarchyGridElement_TYPE
                                        //         SmUserGridElement_TYPE     
    SmPoint3d    m_sSphereCenter;       // Center of bounding sphere
    double       m_dSphereRadius = 0.0; // Radius of bounding sphere
    SmExtent3d   m_sBBox;               // Bounding box of this element
                                      
  public:                             
    void             * GetElement
     () const ; // rtn: SmSurface  *         for SmSurfaceGridElement
                                            //      SmTreeNode *         for SmPolygonGridElement
                                            //      SmHierarchyElement * for SmHierarchyElement
                                            //      void *               for SmUserGridElement
    SM_TYPE            GetType()         const { return m_lType ; }
    const SmPoint3d  & GetSphereCenter() const { return m_sSphereCenter ; }
    double             GetSphereRadius() const { return m_dSphereRadius ; }
    const SmExtent3d & GetBoundingBox()  const { return m_sBBox ; } 
    
    // just for debug code convenience - 
    //           DerivedType       | SurfacePtr | LeafNode Data
    //    -------------------------+------------+--------------
    //    SmSurfaceGridElement     |  has one   |  has some
    //    SmHierarchyGridElement   |  has one   |    none
    SmSurface        * GetSurface()           const ;  // rtn: SmSurface * when possible, else NULL
    SmExtent2d       * GetLeafNodeUVDomain()  const ;  // rtn: Subdivision Leaf Node UVDomain  when possible, else NULL
    SmExtent3d       * GetLeafNodeBBox()      const ;  // rtn: Subdivision Leaf Node BBox      when possible, else NULL
    SmPseudoBox      * GetLeafNodePseudoBox() const ;  // rtn: Subdivision Leaf Node PseudoBox when possible, else NULL
    SmPolarBox       * GetLeafNodePolarBox()  const ;  // rtn: Subdivision Leaf Node PolarBox  when possible, else NULL
    const TCHAR * GetTypeString() const { return _T("SmGridElement") ; }
} ; // end class SmGridElement

SM_TARRAY_TEMPLATE_PREDECLARATION(SmGridElement*);

/*******************************************************************//**
PURPOSE: Derived Class SmSurfaceGridElement to load 
            Surface and Surface decomposition data per Voxel

NOTES:
  static class object - don't add virtual methods to SmSurfaceGridElement
***********************************************************************/
class SM_EXPORT SmSurfaceGridElement : public SmGridElement
{
  // cached information used by SmLSI3LocalSolver::LocalSolve
 public:

  const SmSurface *m_pSurface ;     // This node's Surface pointer
  SmExtent2d       m_sUVDomain ;    // This node's parameter domain
  SmExtent3d       m_sBBox ;        // this node's bounding box
  SmPseudoBox      m_sPseudoBox ;   // This node's pseudo box
  SmPolarBox       m_sPolarBox ;    // This node's polar box

  SmVector3d       m_sMidPoint ;    // temp: see comment in SmSurfaceHierarchyElement::ExpandHierarchy
  SmVector3d       m_sMidDU ;
  SmVector3d       m_sMidDV ;
  SmVector3d       m_sMidNormal ;

//  SmTreeNode      *m_pSurfTreeNode ;

 public:
//  SmTreeNode     * GetElement()           { return m_pSurfTreeNode; }
  SmSurface     * GetElement()           { return (SmSurface *)m_pSurface; }
  void Init(SmTreeNode *pSurfTreeNode) ;

} ; // end class SmSurfaceGridElement

/*******************************************************************//**
PURPOSE: Derived Class SmPolygonGridElement to load 
            PolyBrep decomposition SmTreeNode Ptrs per Voxel

NOTES:
  static class object - don't add virtual methods to SmPolygonGridElement
***********************************************************************/
class SM_EXPORT SmPolygonGridElement : public SmGridElement
{
  public:
  // cached information used by sm_FindPolyHit()
 
  SmTreeNode     * m_pPolygonTreeNode ;     // SmBrepCache leaf TreeNodes

  SmTreeNode     * GetElement()           { return m_pPolygonTreeNode; }
  void Init(SmTreeNode *pPolygonTreeNode) { m_lMark            = 0;
                                            m_lType            = SmPolygonGridElement_TYPE ;
                                            m_pPolygonTreeNode = pPolygonTreeNode ; 
                                          }

} ; // end class SmPolygonGridElement


/*******************************************************************//**
PURPOSE: Derived Class SmHierarchyGridElement to load 
             back Ptrs to owning SmHierarchyElement per Voxel

NOTEs: ugly hack: make sure SmHierarchyGridElement and SmUserGridElement
                     objects are the same size. sorry.

  static class object - don't add virtual methods to SmHierarchyGridElement
***********************************************************************/
class SM_EXPORT SmHierarchyGridElement : public SmGridElement
{
 public:
  // inherited from SmGridElement
  //   ULONG        m_lMark;          // Marking mechanism
  //   SM_TYPE      m_lType;          // derived type identifier
  //   SmPoint3d    m_sSphereCenter;  // Center of bounding sphere
  //   double       m_dSphereRadius;  // Radius of bounding sphere
  //   SmExtent3d   m_sBBox;          // Bounding box of this element

  // cached information used by SmHierarchyElement::ExpandHierarchy()
  SmHierarchyElement * m_pHierarchyElement ; // pointer to SmHierarchyElement whose
                                             // derived class SmSurfaceHierarchyElement is
                                             //   used to store ptr to Surface

 public:
  SmHierarchyElement * GetElement()          { return m_pHierarchyElement ; }
  void Init(SmHierarchyElement *pElement)    { m_lMark             = 0;
                                               m_lType             = SmHierarchyGridElement_TYPE ;
                                               m_pHierarchyElement = pElement ; 
                                             }

} ; // end class SmHierarchyGridElement

/*******************************************************************//**
PURPOSE: Derived Class SmUserGridElement to load 
             void * Ptrs per Voxel for user applications

NOTES: ugly hack: make sure SmHierarchyGridElement and SmUserGridElement
                objects are the same size. sorry.

  static class object - don't add virtual methods to SmUserGridElement
***********************************************************************/
class SM_EXPORT SmUserGridElement : public SmGridElement
{
  public:
  // cached information used by SmHierarchyElement::ExpandHierarchy()

  void               * m_pElement ;          // Anything a User Application chooses to store in a grid

  void               * GetElement()          { return m_pElement ; }
  void Init(void *pElement)                  { m_lMark    = 0;
                                               m_lType    = SmUserGridElement_TYPE ;
                                               m_pElement = pElement ; 
                                             }

} ; // end class SmUserGridElement

/*******************************************************************//**
PURPOSE: This object's derived classes represent hierarchial elements 
  contained within the grid.  Currently the only derived class is a 
  SmSurfaceHierarchyElement which has a pointer to a SmSurface object which
  can be expanded into a spatial decomposition tree where each leaf-node
  represents one small piece of the surface as its own bezier patch.

  The virtual function ExpandHierarchy() embeds the surface decomposition
  tree into the grid voxels by placing pointers in the voxel->m_pElement
  lists to every surface leaf-node that each voxel contains.

NOTES: 
  This is an abstract class which needs to have the
  ExpandHierarchy implemented by subclasses.

  Currently, the only subclass is SmSurfaceHierarchyElement.
***********************************************************************/
class SM_EXPORT SmHierarchyElement : public SmObject
{
    friend class SmGrid;
  protected:
    // currently always of derived type SmSurfaceHierarchyElement
 // SmSurface             *m_pSurface ;           // from derived type SmSurfaceHierarchyElement
    SmHierarchyGridElement m_vGridElement;        //  contains
                                                  //   SmHierarchyElement *m_pHierarchyElement  // back pointer to this
                                                  // inherited from SmGridElement
                                                  //   ULONG        m_lMark;          // Marking mechanism
                                                  //   SM_TYPE      m_lType;          // derived type identifier
                                                  //   SmPoint3d    m_sSphereCenter;  // Center of bounding sphere
                                                  //   double       m_dSphereRadius;  // Radius of bounding sphere
                                                  //   SmExtent3d   m_sBBox;          // Bounding box of this element

    double                 m_dExpansionTolerance; // How much to expand bounding box of things

    // derived class SmSurfaceHierarcyElement also contains
    //   SmSurface *       m_pSurface             // Surface we are tracing against

  public:
    SmHierarchyElement(double dExpansionTolerance) 
    : m_dExpansionTolerance(dExpansionTolerance)
    { m_vGridElement.Init((SmHierarchyElement *)NULL); }

    virtual ~SmHierarchyElement() { }

    SmHierarchyGridElement &GetHierarchyGridElement() { return m_vGridElement ; }

    // build and embed the m_vGridElement->m_pElement object decomposition tree 
    // into given grid by adding pointers to the Grid->Voxel->m_pElement lists
    virtual SmStatus ExpandHierarchy(SmGrid * /* pGrid */) { SE_MSG(SM_ERR,_T("Called pure virtual method")); return SM_ERR; }

    // simple access
    virtual SmSurface  *GetSurface() { return NULL ; }

} ; // end class SmHierarchyElement

/*******************************************************************//**
PURPOSE: This is a surface hierarchy element.

NOTES: 
***********************************************************************/
class SM_EXPORT SmSurfaceHierarchyElement : public SmHierarchyElement
{
  protected:
    // inherited from SmHierarchyElement
    //   SmHierarchyGridElement m_vGridElement;        // Mark, Type, SphereCenter, SphereRadius,
    //                                                 //      BoundingBox and back pointer      
    //   double                 m_dExpansionTolerance; // How much to expand bounding box of things

    SmSurface      * m_pSurface;       // Surface we are tracing against

  public:
    SmSurfaceHierarchyElement(SmSurface * pSurface, 
                              double      dExpansionTolerance) 
        : SmHierarchyElement(dExpansionTolerance), 
          m_pSurface(pSurface)
        { }

    virtual ~SmSurfaceHierarchyElement() {}

    //
    virtual SmStatus ExpandHierarchy(SmGrid * pGrid);

    // simple access
    virtual SmSurface  *GetSurface() { return m_pSurface ; }

} ; // end class SmSurfaceHierarchyElement



/*******************************************************************//**
PURPOSE: The Voxel object represents a small portion of space which
   is a subset of the entire grid.  It may contain elements which are
   those things which intersect the voxel.

NOTES: SmVoxel must remain a static class without virtual methods 
       because SmGrid manages an array of SmVoxels with smos_Calloc.
***********************************************************************/
class SM_EXPORT SmVoxel
{
  friend class SmGrid;
 protected:
  SmGrid                        * m_pGrid;
  SmTArray<SmHierarchyElement*> * m_pHierarchyElements; // List of all surfaces 
                                                        //  whose bounding box intersects this Voxel's bounding box
  SmTArray<SmGridElement*>      * m_pElements;          // List of all surface subdivision leaf-nodes
                                                        //   whose bounding box intersects this Voxel's bounding box.

 public:
  void Init(SmGrid * pGrid) { m_pElements          = NULL; 
                              m_pHierarchyElements = NULL; 
                              m_pGrid              = pGrid; 
                            }

  SmGrid                        * GetGrid()              { return m_pGrid ; }
  SmTArray<SmHierarchyElement*> * GetHierarchyElements() { return m_pHierarchyElements; }
  SmTArray<SmGridElement*>      * GetGridElements()      { return m_pElements; }
     
  // draw bounding boxes of all surfaces whose BBox 
  //    intersects thisVoxel->BBox
  SmDisplayList *DrawHierarchyElements() ;         

  // draw bounding boxes of all surface subdivision leaf nodes whose BBox
  //   intersects thisVoxel->BBox
  SmDisplayList *DrawGridElements() ;         

} ; // end class SmVoxel

/*******************************************************************//**
PURPOSE: The Grid object is a mechanism used to manage spatial 
   containment of objects.  The whole purpose of this object is 
   to reduce time to do intersections.  Its original purpose was to
   enable very fast raytracing.  In the future it may be used for
   other intersection problems.

NOTES: 
   The current implementation is that of a uniform grid.  In the future
   it may be an OCTTREE or some other structure.
***********************************************************************/
class SM_EXPORT SmGrid : public SmObject
{
    friend class SmRayTracer;
  protected:
    SmExtent3d                      m_sBBox;              // Extents of the grid
    long                            m_lSize[3];           // Number of voxels in X, Y, and Z
    SmVector3d                      m_sVoxelSize;         // Size of an individual voxel
    double                          m_dVoxelTolerance;    // Tolerance used in computations
    ULONG                           m_lCurrentMark;       // Used to mark things, matched with SmGridElement::m_lMark 

    SmVoxel       ***m_aVoxels;                  // 3D array of SmVoxel objects
    SmMemBlockMgr    m_sSurfaceElementMgr;       // Pool of all SmSurfaceGridElement objects 
                                                 //   referenced by Voxel Grid
#define m_sHierarchyElementMgr m_sUserElementMgr
    SmMemBlockMgr    m_sUserElementMgr;          // Pool of all SmUserGridElement/SmHierarchyGridElement 
                                                 //   objects referenced by Voxel Grid
    SmMemBlockMgr    m_sPolygonElementMgr;       // Pool of all SmPolygonGridElement objects 
                                                 //   referenced by Voxel Grid

    
    SmTArray<SmHierarchyElement*> * m_pHierarchyElements; // List of all SmHierarchyElement objects 
                                                          // referenced by Voxel Grid  

    // 3DDDA values: init with Setup3DDDA(), incremented with Step3DDDA() 
    SmPoint3d   m_sRayPoint;    // Ray Start Point
    SmVector3d  m_sRayVector;   // Ray Unit direction
                                   
    double      m_dRayT;        // scratch: current voxel entry point ray parameter.
                                //          incremented to each voxel entry boundary as ray is traversed in Step3DDA().
    SmPoint3d   m_sCurrPoss;    // scratch: current voxel entry point associated with m_dRayT,
                                //          incremented as ray is traversed in Step3DDDA().
    SmPoint3d   m_dNextT;       // scratch: param values to next X-plane Y-plane & Z-plane xsects, 
                                //          incremented as ray is traversed in Step3DDDA().
    SmPoint3d   m_dDeltaT;      // scratch: delta param values to move from one x,y or z plane to the next

    SmVector3d  m_sDeltaXVec;   // scratch: RayVector set to length to move from one x plane to the next
    SmVector3d  m_sDeltaYVec;   // scratch: RayVector set to length to move from one y plane to the next
    SmVector3d  m_sDeltaZVec;   // scratch: RayVector set to length to move from one z plane to the next

    SmPoint3d   m_sNextX;       // scratch: Next XPlane intersection Point
    SmPoint3d   m_sNextY;       // scratch: Next YPlane intersection Point
    SmPoint3d   m_sNextZ;       // scratch: Next ZPlane intersection Point
                                //          m_sNext vals incremented as ray is traversed in Step3DDDA().

    long        m_lStepX;       // scratch: mark X voxel traversal for ray: 1 = pos dir, -1 = neg dir, 0 = no change
    long        m_lStepY;       // scratch: mark Y voxel traversal for ray: 1 = pos dir, -1 = neg dir, 0 = no change
    long        m_lStepZ;       // scratch: mark Z voxel traversal for ray: 1 = pos dir, -1 = neg dir, 0 = no change

    long        m_lOutX;        // scratch: X index of 1st voxel outside the grid hit by the ray (-1 or m_lSize[0])
    long        m_lOutY;        // scratch: Y index of 1st voxel outside the grid hit by the ray (-1 or m_lSize[1])
    long        m_lOutZ;        // scratch: Z index of 1st voxel outside the grid hit by the ray (-1 or m_lSize[2])

    long        m_lAddress[3];  // scratch: initally, indices of voxel containing m_sRayPoint 
                                //          incremented as ray is traversed in Step3DDDA().
  public:
    SmGrid(const SmExtent3d & crGridBbox,
           ULONG  lSize[3]);

    virtual ~SmGrid();

    SmStatus AddElement(void             * pElement,         // in : type = SmTreeNode * from Surface decomposition for SM_GE_SURFACE
                                                             //             SmTreeNode * from PolyBrep decomposition for SM_GE_POLYBREP
                                                             //             void *       from User Application
                        SmGridElementType  eGridElementType, // in : oneof  SM_GE_SURFACE   for SmTreeNode * from Surface decomposition
                                                             //             SM_GE_POLYBREP  for SmTreeNode * from PolyBrep decomposition
                                                             //             SM_GE_USERDATA  for void *       from User Application
                        const SmExtent3d & crElementBBox);

    SmStatus AddHierarchyElement(SmHierarchyElement * pHierarchyElement,
                                 const SmExtent3d   & crElementBBox);

    SmStatus FindVoxelsInBox(const SmExtent3d   & crBBox,
                             SmTArray<SmVoxel*> & rVoxels) const;

    SmStatus GetVoxelAddress(const SmPoint3d & crPoint,
                             long lVoxelAddress[3],
                             SmBoolean & rbOutsideGrid) const;

    SmExtent3d GetVoxelBBox(long lVoxelAddress[3]) const;

    SmVoxel * GetVoxel(long lVoxelAddress[3]) const    { return &m_aVoxels[lVoxelAddress[0]]
                                                                          [lVoxelAddress[1]]
                                                                          [lVoxelAddress[2]]; 
                                                       }

    SmStatus Setup3DDDA(const SmPoint3d  & crRayStart,
                        const SmVector3d & crRayVector,
                        SmBoolean        & rbHitsNothing);  // note: increments SmGrid::m_lCurrentMark

    SmStatus Step3DDDA(double                     dMinFoundT,
                       SmBoolean                & rbExitedGrid,
                       long                       lCurrentVoxel[3],
                       SmTArray<SmGridElement*> & rSortedElementsInVoxel,
                       SmTArray<double>         & rMinDistances,
                       SmTArray<double>         & rMaxDistances); // note: uses without incrementing SmGrid::m_lCurrentMark value

    ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const ;

    SmDisplayList * Draw(SmBoolean bDrawVoxels=TRUE,
                         SmBoolean bDrawSteppingData=TRUE,
                         SmBoolean bDrawVoxelGeometry=TRUE,
                         SmBoolean bDrawHierarchyElements=TRUE);
    SmDisplayList * DrawVoxel(long lVoxelAddress[3]);
    SmDisplayList * DrawVoxelHierarchyElements() ; // Draw Surface->BBoxes that intersect target Voxel
    SmDisplayList * DrawVoxelGridElements() ;      // Draw Surface->SubdivisionLeafNode->BBoxes that intersect target Voxel

} ; // end class SmGrid

/*******************************************************************//**
PURPOSE: Compute the bounding box of this voxel.

NOTES: 
***********************************************************************/
inline SmExtent3d SmGrid::GetVoxelBBox(long lVoxelAddress[3]) const
{
    SmPoint3d sMin, sMax;
    sMin.x = m_sBBox.GetMin().x + lVoxelAddress[0] * m_sVoxelSize.x;
    sMin.y = m_sBBox.GetMin().y + lVoxelAddress[1] * m_sVoxelSize.y;
    sMin.z = m_sBBox.GetMin().z + lVoxelAddress[2] * m_sVoxelSize.z;

    sMax = sMin + m_sVoxelSize;

    return SmExtent3d(sMin,sMax);
}


#endif // !__SMGRID_H__



