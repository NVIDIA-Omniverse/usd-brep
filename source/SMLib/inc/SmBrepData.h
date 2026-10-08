// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBrepData.h
* PURPOSE: Header file for SmBrepData and associated classes.
**********************************************************************/

#ifndef __SMBREPDATA_H__
#define __SMBREPDATA_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMATTRIBUTE_H__
#include <SmAttribute.h>
#endif

class SmFaceuse;
class SmEdgeuse;

/*******************************************************************//**
PURPOSE: Read/Write interface Magic Number Names

NOTES: Please use Names rather than magic numbers
***********************************************************************/

#define SM_NO_OBJECT 9999999   // index or Ptr value in Sm_TopologyData objects used
                               //  when associated SmTopology object contains a NULL value
                               //  a Topology object pointer with a NULL value.
                               //  Currently this happens when an edge is written with a
                               //   NULL UVTrimCurve Value or when a UVTrimCurve was created on
                               //   a face without an associated edge.

/*******************************************************************//**
PURPOSE: Enum of file sources

NOTES: 
***********************************************************************/
enum SmDataSourceType
{
  SM_DS_UNKNOWN,
  SM_DS_SMLIB,
  SM_DS_IGES,
  SM_DS_STEP,
  SM_DS_STL,
  SM_DS_SAT,
  SM_DS_PARASOLID,
  SM_DS_VDAFS,
  SM_DS_OPENNURBS,
  SM_DS_DATAKIT,
  SM_DS_SOLIDWORKS,
  SM_DS_USD,
  SM_DS_PRC,
  SM_DS_DGN,
  SM_DS_ALIAS
} ;

/*******************************************************************//**
PURPOSE: This class defines a boundary of a curve (i.e. a vertex)

NOTES: 
***********************************************************************/
class SM_EXPORT SmVertexNode
{
public:
  ULONG     m_lType;       // 0 - 3-D point, 
                           // 1 - Curve Parameter
                           // 2 - UV point
  double    m_dParameter;
  SmPoint3d m_vPoint;

  // empty constructor
  SmVertexNode()
      : m_lType(SM_UNDEF_ULONG),
        m_dParameter(SM_UNDEF_DOUBLE), m_vPoint()
  { };

  // constructor for 3d curve parameter
  SmVertexNode(double dParameter)            : m_lType(1), 
                                               m_dParameter(dParameter),  
                                               m_vPoint(0,0,0)  
                                             { }

  // constructor for curve 3d point                                             
  SmVertexNode(const SmPoint3d & cr3DPoint)  : m_lType(0),  
                                               m_dParameter(0.0),  
                                               m_vPoint(cr3DPoint)  
                                             { }

  // constructor for 2d parameter (set Z=0.0)                                             
  SmVertexNode(const SmPoint2d & crUVPoint)  : m_lType(2),  
                                               m_dParameter(0.0), 
                                               m_vPoint(crUVPoint.x,crUVPoint.y,0.0)  
                                             { }

  // Destructor
 ~SmVertexNode()                             { m_lType      = SM_UNDEF_ULONG ;
                                               m_dParameter = SM_UNDEF_DOUBLE ;
                                               m_vPoint.SetUninitialized() ;
                                             }
  // no operator=, shallow copies okay

  void Set(double dParameter)
                                             {
                                                 m_lType = 1;
                                                 m_dParameter = dParameter;
                                                 m_vPoint.Set(0, 0, 0);
                                             }

  void Set(const SmPoint3d & cr3dPoint)
                                             {
                                                 m_lType = 0;
                                                 m_dParameter = 0.0;
                                                 m_vPoint = cr3dPoint;
                                             }
  
  void Set(const SmPoint2d & crUVPoint)
                                             {
                                                 m_lType = 2;
                                                 m_dParameter = 0.0;
                                                 m_vPoint.Set(crUVPoint.x, crUVPoint.y, 0.0);
                                             }
} ; // end class SmVertexNode

/*******************************************************************//**
PURPOSE: This class provides a top down programatic interface for
   creating Breps with open shells and solids.  

NOTES: This class is primarily designed to construct typical
   manifold solids or open shells.  It does not lend itself to construction
   of non-manifold objects.
***********************************************************************/
class SM_EXPORT SmBrepConstructor 
{
private:
  SmContext  * m_pContext;
  SmBrep     * m_pBrep;
  SmRegion   * m_pRegion;
  SmRegion   * m_pOuterRegion;
  SmShell    * m_pOuterShell;
  SmShell    * m_pInnerShell;
  SmFace     * m_pFace;
  SmLoop     * m_pLoop;
  SmEdge     * m_pEdge;
  SmEdgeuse  * m_pEdgeuse;
  SmVertex   * m_pVertex;
  SmBoolean    m_bIsManifold = TRUE;
  SmOrientType m_eLoopSurfaceOrientation = SM_OT_UNKNOWN;  // Orientation of the loop relative to the surface

public:
  // constructor
  SmBrepConstructor() ;

  SmBrep   * StartBrep 
  (
    const SmContext   & crContext,                           // in : context for new object construction                                                   
    double              dModelSizeEstimate = SM_USE_DEFAULT  // NotUsed: in : OldTol: Def ZoneTol3d assigned to m_pBrep to use for all Brep Contained objects       
                                                             //      NewTol: Def ZoneTol3d assigned to Context to use for all Context contained objects    
  );


  SmRegion * StartRegion                
  (
    SmBoolean           bIsSolidRegion,       // in : bIsSolidRegion = not used in this function     
    SmRegion          * pOuterRegion          // in : new shell's outer region                       
  );

  SmStatus   StartShell                 
  (
    SmShell          *& rpOuterShell,
    SmShell          *& rpInnerShell          // in : only used if bounding a solid region     
                                              //      for open shells this will be NULL        
  );

  SmFace   * StartFace                  (SmOrientType      eOutsideFaceuse);
  SmLoop   * StartLoop                  (SmOrientType      eOrienationToSurface);
  SmLoop   * StartAndEndSingleVertexLoop(const SmPoint3d & crVertexPoint);
  SmVertex * StartVertexOfLoop          (SmPoint3d         sPnt);
  SmEdge   * StartEdge                  
  (
    SmOrientType   eOrientationInLoop,       // in :        
    SmEdge       * pOptExistingEdge,         // in :        
    SmVertex     * pOptExistingStartVertex,  // in :        
    SmVertex     * pOptExistingEndVertex     // in :        
  );

  SmVertex * StartVertex(const SmPoint3d & crVertex);

  SmContext* GetContext() { return m_pContext; }
  SmBoolean  IsManifold() { return m_bIsManifold; }

  SmStatus   EndBrep  ();
  SmStatus   EndRegion();
  SmStatus   EndShell (SmBoolean bStitchShell);
  SmStatus   EndFace  ();
  SmStatus   EndLoop  ();
  SmStatus   EndEdge  ();
  SmStatus   EndVertex();
           

  // Swap inner and outer regions
  void SwapRegions()
  {
      SmRegion* pTmp = m_pOuterRegion;
      m_pOuterRegion = m_pRegion;
      m_pRegion = pTmp;
  };

  SmStatus   SetLimits
  (
    const SmVertexNode & crStartVertexNode,   // in :     
    const SmVertexNode & crEndVertexNode      // in :     
  );
             
  SmStatus   SetFaceSurface
  (
    SmSurface *pSurface,           // in :      
    const SmExtent2d & crUVDomain  // in :      
  );
             
  SmStatus   SetEdgeCurve
  (
    SmCurve *pCurve,        // in :    
    SmBoolean bUVCurve      // in :    
  );

} ; // end class SmBrepConstructor

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmTopology

NOTES:
 static class object - don't add virtual methods to SmTopologyData.  
 Virtual methods conflict with the methods in SmTArray<SmTopologyData>
 that use memset() to clear memory - with SmTopologyData virtual methods the
 virtual pointer tables get corrupted by SmTArray<SmTopologyData>::ReSet() 
 calls and the like. 
***********************************************************************/
class SM_EXPORT SmTopologyData : public SmAttributeData
{
 public:
  // inherited 
  // SmTArray<ULONG>   SmAttributeData::m_sAttributes; // - Each SmTypeData object uses this list to store the indices of the 
  //                                                   //   attributes that belong to it in the one global array of attributes.
                                                       
  ULONG                    m_lFlags = SM_UNDEF_ULONG ;       // User definable bit array marking individual TopoObjs [persistent - written to and read from file]
#ifdef SM_INDEXING         
  ULONG                    m_lUserIndex1 = SM_UNDEF_ULONG ;  // First User Definable Index,  [persistent - written to and read from file]
  ULONG                    m_lUserIndex2 = SM_UNDEF_ULONG ;  // Second User Definable Index, [persistent - written to and read from file]
  void                   * m_pUserPtr1 = 0 ;                 // User Definable pointer,      [not persistent - not written to and read from file]
                                                             //                              [not owned - not freed by destructor - freed by caller if needed]
#endif // SM_INDEXING
  SM_NEWTOL_LINE SmBoolean m_bIsSmallTopology = FALSE ;      // used by Face, Edge, Vertex (rarely needed - leave defaulted 99.99% of the time)
                                                             // TRUE = Intended small geometry size (Pinhole in Battleship) - gets tighter tolerances
                                                             // FALSE= Typical size - gets typical tolerances
                                                             // default:[FALSE]
  // constructor
  SmTopologyData() : m_lFlags(SM_UNDEF_ULONG),
#ifdef SM_INDEXING 
                     m_lUserIndex1(SM_UNDEF_ULONG),
                     m_lUserIndex2(SM_UNDEF_ULONG),
                     m_pUserPtr1(0)
#endif // SM_INDEXING
   SM_NEWTOL_LINE  , m_bIsSmallTopology(FALSE)
                    { }

  // destructor
  ~SmTopologyData() { ReSet() ; }
  void ReSet()      { SmAttributeData::ReSet() ; // clear m_sAttributes array
                      m_lFlags           = SM_UNDEF_ULONG ;
#ifdef SM_INDEXING                       
                      m_lUserIndex1      = SM_UNDEF_ULONG ;
                      m_lUserIndex2      = SM_UNDEF_ULONG ;
                      m_pUserPtr1        = NULL ;
#endif // SM_INDEXING 
       SM_NEWTOL_LINE m_bIsSmallTopology = FALSE ;
                    }
  // no operator=, shallow copies okay

  // I/O
  SmStatus   WriteToDB(ULONG           lDBVersionNumber,       // in :
                       SmDatabaseIO  & rDB,                    // in :
                       SmBoolean       bPersistAttribs=TRUE) ; // NotUsed: in :

  SmStatus   ReadFromDB(ULONG          lDBVersionNumber,       // in :
                        SmDatabaseIO & rDB,                    // in :
                        SmBoolean      bPersistAttribs=TRUE) ; // NotUsed: in : 

} ; // end class SmTopologyData

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmRegion 

NOTES:  static class object - don't add virtual methods to SmRegionData.  
***********************************************************************/
class SM_EXPORT SmRegionData : public SmTopologyData
{
 public:
  // inherited: 
  //   SmTArray<ULONG> SmAttributeData::m_sAttributes;      // this TopoObj's list of attribute indices in the translation global attribute array
  //   ULONG           SmTopologyData::m_lFlags;            // User definable bit array,    [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex1;       // First  User Definable Index, [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex2;       // Second User Definable Index, [persistent - written to and read from file]
  //   void*           SmTopologyData::m_pUserPtr1;         // User Definable ptr,          [not persistent - not written to and read from file]
  //                                                        //                              [not owned - not freed by destructor - caller can free if needed]

  ULONG          m_lStartShell = SM_UNDEF_ULONG; // Index of First (outer) shell of this Region in SmBrepData::m_vRegions.
  ULONG          m_lNumShells  = 0 ;             // Number of shells in this region - remaining shells after
                                                 // the first shell are inner shells in the region.
  SmBoolean      m_bIsVoidFlag = UNSURE;         // If TRUE this region is a void as opposed to solid region.
  SmRegion     * m_pRegion     = NULL;           // ptr to corresponding SmRegion - once it gets built

  // constructor
  SmRegionData() { }
                 
  // destructor          
  ~SmRegionData() { ReSet() ; } 
  void ReSet()    { SmTopologyData::ReSet() ;  // eff: empty SmTArray<ULONG> m_sAttributes, default SmTopologyData values 
                    m_lStartShell = SM_UNDEF_ULONG ;
                    m_lNumShells  = SM_UNDEF_ULONG ;
                    m_bIsVoidFlag = UNSURE ;
                    m_pRegion     = NULL ;    
                  }

  // no operator=, shallow copies okay

  // I/O
  SmStatus   WriteToDB(ULONG lDBVersionNumber,SmDatabaseIO & rDB,SmBoolean bPersistAttribs=TRUE);

  SmStatus   ReadFromDB(const SmContext        & crContext,
                        ULONG                    lDBVersionNumber,
                        SmTArray<SmAttribute*> & rAllAttributes,
                        SmDatabaseIO           & rDB, 
                        SmBoolean                bPersistAttribs=TRUE);
  
} ; // end class SmRegionData

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmShell

NOTES: 
  For a manifold model there are only 2 shells, inside and outside.
    In this case 1 SmShellData is used for the 2 mated SmShell objects.
    m_pShell1 = Shell for Infinite Region,
    m_pShell2 = Shell for Inside.

  For nonManifol models, each SmShell maps to 1 SmShellData.
    m_pShell1 = Shell for Region,
    m_pShell2 = NULL.

  static class object - don't add virtual methods to SmShellData.
***********************************************************************/
class SM_EXPORT SmShellData : public SmTopologyData
{
 public:
  // inherited: 
  //   SmTArray<ULONG> SmAttributeData::m_sAttributes;      // this TopoObj's list of attribute indices in the translation global attribute array
  //   ULONG           SmTopologyData::m_lFlags;            // User definable bit array,    [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex1;       // First  User Definable Index, [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex2;       // Second User Definable Index, [persistent - written to and read from file]
  //   void*           SmTopologyData::m_pUserPtr1;         // User Definable ptr,          [not persistent - not written to and read from file]
  //                                                        //                              [not owned - not freed by destructor - caller can free if needed]

  ULONG        m_lShellType    = SM_UNDEF_ULONG;  // 0 - Faceuse, 1 - WireEdge,  2 - VertexShell
  ULONG        m_lVertex       = SM_UNDEF_ULONG;  // for vertex shells,   index of Vertex in SmBrepData::m_vVertices.
  ULONG        m_lEdge         = SM_UNDEF_ULONG;  // for wireEdge shells, index of start edge in SmBrepData::m_vEdges.
  ULONG        m_lFaceuseStart = SM_UNDEF_ULONG;  // for faceuse shells,  index of start Faceuse in SmBrepData::m_vFaceuses.
  ULONG        m_lNumFaceuses  = 0;               // for faceuse shells,  number of faceusesshell.
  SmShell    * m_pShell1       = NULL;            // ptr to corresponding SmShell - once it gets built
  SmShell    * m_pShell2       = NULL;            // ptr to corresponding Infinite shell - once it gets built for speciaL case: 
                                                  //     IGES file read by function smiges_ReadIges() of a manifold solid 
                                                  //     with one solid region and any number of void regions only
                                                  //     represents the solid region explicitly.  Implied infinite
                                                  //     and internal void regions get added in a CreateBrepFromData() special
                                                  //     branch storing added shells here.  Otherwise this ptr is not used.
             
  // constructor
  SmShellData() { }
                
  // constructor
  ~SmShellData() { ReSet() ; }
  void ReSet()   { SmTopologyData::ReSet() ; // eff: empty SmTArray<ULONG> m_sAttributes, default SmTopologyData values
                   m_lShellType       = SM_UNDEF_ULONG ;   
// gwc:removed                   m_bOrientation     = UNSURE ; 
                   m_lVertex          = SM_UNDEF_ULONG ;      
                   m_lEdge            = SM_UNDEF_ULONG ;        
                   m_lFaceuseStart    = SM_UNDEF_ULONG ;
                   m_lNumFaceuses     = SM_UNDEF_ULONG ; 
                   m_pShell1          = NULL ;
                   m_pShell2          = NULL ;      
                 }
                        
  // no operator=, shallow copies okay

  // I/O                         m_pShell2;      
  SmStatus  WriteToDB(ULONG lDBVersionNumber,SmDatabaseIO & rDB,SmBoolean bPersistAttribs=TRUE);
  SmStatus  ReadFromDB(const SmContext        & crContext,
                       ULONG                    lDBVersionNumber,
                       SmTArray<SmAttribute*> & rAllAttributes,
                       SmDatabaseIO           & rDB, 
                       SmBoolean                bPersistAttribs=TRUE);

} ; // end class SmShellData

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmFaceuse 

NOTES:  static class object - don't add virtual methods to SmFaceuseData. 
***********************************************************************/
class SM_EXPORT SmFaceuseData : public SmTopologyData
{
 public:
  // inherited: 
  //   SmTArray<ULONG> SmAttributeData::m_sAttributes;      // this TopoObj's list of attribute indices in the translation global attribute array
  //   ULONG           SmTopologyData::m_lFlags;            // User definable bit array,    [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex1;       // First  User Definable Index, [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex2;       // Second User Definable Index, [persistent - written to and read from file]
  //   void*           SmTopologyData::m_pUserPtr1;         // User Definable ptr,          [not persistent - not written to and read from file]
  //                                                        //                              [not owned - not freed by destructor - caller can free if needed]

  ULONG          m_lFace        = SM_UNDEF_ULONG ; // Index of Owner Face in SmBrepData::m_vFaces
  SmBoolean      m_bOrientation = UNSURE ;         // TRUE - same orientation as surface of face - FALSE - opposite
  SmFaceuse    * m_pFaceuse     = NULL ;           // ptr to corresponding SmFaceuse - once it gets built
               
  // constructor
  SmFaceuseData() { }

  // destructor
  ~SmFaceuseData() { ReSet() ; }
  void ReSet()     { SmTopologyData::ReSet() ;  // eff: empty SmTArray<ULONG> m_sAttributes, default SmTopologyData values
                     m_lFace        = SM_UNDEF_ULONG ;
                     m_bOrientation = UNSURE ;
                     m_pFaceuse     = NULL ;
                   }

  // no operator=, shallow copies okay

  // I/O
  SmStatus WriteToDB(ULONG lDBVersionNumber,SmDatabaseIO & rDB,SmBoolean bPersistAttribs=TRUE);
  SmStatus ReadFromDB(const SmContext        & crContext,
                      ULONG                    lDBVersionNumber,
                      SmTArray<SmAttribute*> & rAllAttributes,
                      SmDatabaseIO           & rDB, 
                      SmBoolean                bPersistAttribs=TRUE);

} ; // end class SmFaceuseData

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmFace 

NOTES: static class object - don't add virtual methods to SmFaceData. 
***********************************************************************/
class SM_EXPORT SmFaceData : public SmTopologyData
{
 public:
  // inherited: 
  //   SmTArray<ULONG> SmAttributeData::m_sAttributes;      // this TopoObj's list of attribute indices in the translation global attribute array
  //   ULONG           SmTopologyData::m_lFlags;            // User definable bit array,    [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex1;       // First  User Definable Index, [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex2;       // Second User Definable Index, [persistent - written to and read from file]
  //   void*           SmTopologyData::m_pUserPtr1;         // User Definable ptr,          [not persistent - not written to and read from file]
  //                                                        //                              [not owned - not freed by destructor - caller can free if needed]

#ifdef SM_USE_OLDTOL
  SM_OLDTOL_LINE SmZoneTol3d m_sZoneTol3d = 0.0 ;    // OldTol: face's tolerant neighborhood offset size
#endif // SM_USE_OLDTOL
  ULONG                      m_lSurface         = SM_UNDEF_ULONG ; // Index of Surface owned by face in SmBrepData::m_vSurfaces
  ULONG                      m_lStartLoop       = SM_UNDEF_ULONG ; // Index of First (outer) loop of this face in SmBrepData::m_vFaces
  ULONG                      m_lNumLoops        = 0 ;              // Additional loops after first are inner loops
  SmExtent2d                 m_vUVDomain ;                         // UV Domain of Face on Surface
  SmBoolean                  m_bRectangularTrim = UNSURE ;         // TRUE = Face has 1 UVRectangle OuterLoop of 4 isoUVTrimCurves with extents m_vUVDomain, FALSE=isn't, UNSURE = Not yet set
  SmFace                   * m_pFace            = NULL ;           // ptr to corresponding SmFace
                                                                   //   Input  - once it gets built
                                                                   //   Output - Face that generated data
  // constructor   
  SmFaceData() { }

  // destructor
  ~SmFaceData() { ReSet() ; }
  void ReSet()  { SmTopologyData::ReSet() ;  // eff: empty SmTArray<ULONG> m_sAttributes, default SmTopologyData values
#ifdef SM_USE_OLDTOL
   SM_OLDTOL_LINE m_sZoneTol3d       = 0.0 ;
#endif // SM_USE_OLDTOL              
                  m_lSurface         = SM_UNDEF_ULONG ;  
                  m_lStartLoop       = SM_UNDEF_ULONG ;
                  m_lNumLoops        = 0 ; 
                  m_vUVDomain.Init() ;
                  m_bRectangularTrim = UNSURE ;
                  m_pFace            = NULL ; 
                }

  // no operator=, shallow copies okay

  // I/O
  SmStatus WriteToDB(ULONG lDBVersionNumber,SmDatabaseIO & rDB,SmBoolean bPersistAttribs=TRUE);
  SmStatus ReadFromDB(const SmContext        & crContext,
                      ULONG                    lDBVersionNumber,
                      SmTArray<SmAttribute*> & rAllAttributes,
                      SmDatabaseIO           & rDB, 
                      SmBoolean                bPersistAttribs=TRUE);

} ; // end class SmFaceData

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmLoop

NOTES:
  Loop attributes are stored for separately for upper and lower sides. 

  static class object - don't add virtual methods to SmLoopData. 
***********************************************************************/
class SM_EXPORT SmLoopData : public SmTopologyData
{
 public:
  // inherited: 
  //   SmTArray<ULONG> SmAttributeData::m_sAttributes;      // this TopoObj's list of attribute indices in the translation global attribute array
  //   ULONG           SmTopologyData::m_lFlags;            // User definable bit array,    [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex1;       // First  User Definable Index, [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex2;       // Second User Definable Index, [persistent - written to and read from file]
  //   void*           SmTopologyData::m_pUserPtr1;         // User Definable ptr,          [not persistent - not written to and read from file]
  //                                                        //                              [not owned - not freed by destructor - caller can free if needed]

  ULONG                m_lLoopType = SM_UNDEF_ULONG ;  // 0 - Edgeuse loop - 1 - Vertex loop
  ULONG                m_lVertex   = SM_UNDEF_ULONG ;  // If m_lLoopType == 1 Index of vertex        in SmBrepData::m_vVertices
  ULONG                m_lStartEU  = SM_UNDEF_ULONG ;  // If m_lLoopType == 0 Index of First Edgeuse in SmBrepData::m_vEdgeuses 
                                                       //    note: that it may actually be a vertex.  
                                                       //          You'll have to look into the EU to find out.
  ULONG                m_lNumEU    = 0 ;               // Number of edgeuses in this loop.  If it is a vertex loop then
                                                       // there can only be one edgeuse.
  SmTArray<ULONG>     m_sLUUpperAttributes;
  SmTArray<ULONG>     m_sLULowerAttributes;
  SmLoop            * m_pLoop     = NULL ;             // ptr to corresponding SmLoop - once it gets built
                    
  // constructor
  SmLoopData() { }
                       
  // destructor
  ~SmLoopData() { ReSet() ; }
  void ReSet()  { SmTopologyData::ReSet() ;  // eff: empty SmTArray<ULONG> m_sAttributes, default SmTopologyData values
                  m_lLoopType          = SM_UNDEF_ULONG ;
                  m_lVertex            = SM_UNDEF_ULONG ;
                  m_lStartEU           = SM_UNDEF_ULONG ;
                  m_lNumEU             = 0 ;   
                  m_sLUUpperAttributes.ReSet() ;
                  m_sLULowerAttributes.ReSet() ;
                  m_pLoop              = NULL ; 
                }

  // no operator=, shallow copies okay

  // I/O
  SmStatus WriteToDB(ULONG lDBVersionNumber,SmDatabaseIO & rDB,SmBoolean bPersistAttribs=TRUE);
  SmStatus ReadFromDB(const SmContext        & crContext,
                      ULONG                    lDBVersionNumber,
                      SmTArray<SmAttribute*> & rAllAttributes,
                      SmDatabaseIO           & rDB, 
                      SmBoolean                bPersistAttribs=TRUE);

} ; // end class SmLoopData

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmEdge

NOTES:  static class object - don't add virtual methods to SmEdgeData. 
***********************************************************************/
class SM_EXPORT SmEdgeData : public SmTopologyData
{
 public:
  // inherited: 
  //   SmTArray<ULONG> SmAttributeData::m_sAttributes;      // this TopoObj's list of attribute indices in the translation global attribute array
  //   ULONG           SmTopologyData::m_lFlags;            // User definable bit array,    [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex1;       // First  User Definable Index, [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex2;       // Second User Definable Index, [persistent - written to and read from file]
  //   void*           SmTopologyData::m_pUserPtr1;         // User Definable ptr,          [not persistent - not written to and read from file]
  //                                                        //                              [not owned - not freed by destructor - caller can free if needed]

#ifdef SM_USE_OLDTOL
  SM_OLDTOL_LINE SmZoneTol3d m_sZoneTol3d = 0.0 ;
#endif // SM_USE_OLDTOL         
  SmExtent1d      m_vInterval;          
  ULONG           m_lCurve       = SM_UNDEF_ULONG ;   // Index of Edge's Curve           in SmBrepData::m_v3DCurves
  ULONG           m_lStartVertex = SM_UNDEF_ULONG ;   // Index of Edge's Start Vertex    in SmBrepData::m_vVertices
  ULONG           m_lEndVertex   = SM_UNDEF_ULONG ;   // Index of Edge's End   Vertex    in SmBrepData::m_vVertices
  long            m_lPrimEU      = SM_UNDEF_ULONG ;   // Index of Edge's Primary edgeuse in SmBrepData::m_vEdgeuses
                                                      // the primary Edgeuse is the head of mate/radial link-list.  
                                                      // The 2nd Edgeuse on the radial link-list must be the primary Edgeuse's mate.
                                                      // Depending on EdgeCurve/FaceSurface orientations the primary edgeuse can
                                                      // be either a member of the face's positive or negative sides but it must be the first edgeuse
                                                      // attached to a face connected to the edge that is encountered on a right-hand rule traversal around the edge.
                                                      //   Positive Value = pBrepData->m_vEdgeuses[lIndex].m_pEdgeuse1      (edgeuse on Face Pos side)
                                                      //   Negative Value = pBrepData->m_vEdgeuses[(-lIndex)-1].m_pEdgeuse2 (edgeuse on Face Neg side)
  SmEdge        * m_pEdge        = NULL  ;            // Ptr to corresponding SmEdge
                                                      //  Input  - once it gets built
                                                      //  Output - Edge that generated data
                                                      
  // constructor
  SmEdgeData() { }

  // destructor
  ~SmEdgeData() { ReSet() ; }
  void ReSet()  { SmTopologyData::ReSet() ;  // eff: empty SmTArray<ULONG> m_sAttributes, default SmTopologyData values
#ifdef SM_USE_OLDTOL
   SM_OLDTOL_LINE m_sZoneTol3d   = 0.0 ;
#endif // SM_USE_OLDTOL          
                  m_lCurve       = SM_UNDEF_ULONG ;      
                  m_lStartVertex = SM_UNDEF_ULONG ;
                  m_lEndVertex   = SM_UNDEF_ULONG ;  
                  m_lPrimEU      = SM_UNDEF_ULONG ;     
                  m_pEdge        = NULL ; 
                }

  // no operator=, shallow copies okay

  // I/O
  SmStatus WriteToDB(ULONG lDBVersionNumber,SmDatabaseIO & rDB,SmBoolean bPersistAttribs=TRUE);
  SmStatus ReadFromDB(const SmContext        & crContext,
                      ULONG                    lDBVersionNumber,
                      SmTArray<SmAttribute*> & rAllAttributes,
                      SmDatabaseIO           & rDB, 
                      SmBoolean                bPersistAttribs=TRUE) ;


} ; // end class SmEdgeData

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmEdgeuse 

NOTES:
  1. One SmEUData object is used for every pair of SmEdgeuse Mates.
  
  2. The SmEUData Edgeuse mates are called Edgeuse1 and Edgeuse2

  3. static class object - don't add virtual methods to SmEUData.  

  4. SmEUData objects are used to store the data for both primary and mate edgeuses.
     The primary edgeuse is the one that is encountered first on a right-hand rule traversal around the edge.
     The mate edgeuse is the one that is encountered second on a right-hand rule traversal around the edge.
     The primary edgeuse is always Edgeuse1 and the mate edgeuse is always Edgeuse2.
     The primary edgeuse's m_lNextEU points to the next radial edgeuse in the link-list of radial edgeuses
     and the mate edgeuse's m_lMateNextEU points to the next radial edgeuse in the link-list of radial edgeuses.

  5. The SmBrepData->m_vEdgeuses array only contains:
      1 SmEUData per mated-pair of FaceEdgeuses.
      1 SmEUData per WireEdge.
      Although SmEUData can represent the Edgeuse used by a LoopVertex SmEUData objs are not added to the
      SmBrepData->m_vEdgeuses array.
***********************************************************************/
class SM_EXPORT SmEUData : public SmTopologyData
{
 public:
  // inherited: 
  //   SmTArray<ULONG> SmAttributeData::m_sAttributes;      // this TopoObj's list of attribute indices in the translation global attribute array
  //   ULONG           SmTopologyData::m_lFlags;            // User definable bit array,    [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex1;       // First  User Definable Index, [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex2;       // Second User Definable Index, [persistent - written to and read from file]
  //   void*           SmTopologyData::m_pUserPtr1;         // User Definable ptr,          [not persistent - not written to and read from file]
  //                                                        //                              [not owned - not freed by destructor - caller can free if needed]

  ULONG             m_lEUType         = SM_UNDEF_ULONG ;    // Type of edgeuse - 0 - face edge,      
                                                            //                   1 - loop vertex, (no Edgeuses for ShellVertices)
                                                            //                   2 - wire edge, 
                                                            //                   3 - vertex at a pole in an edge loop
  ULONG             m_lEdge           = SM_UNDEF_ULONG ;    // if m_lEUType == 0, Index of Edge   in SmBrepData::m_vEdges
  ULONG             m_lVertex         = SM_UNDEF_ULONG ;    // if m_lEUType == 1, Index of Vertex in SmBrepData::m_vVertices
  ULONG             m_lShell          = SM_UNDEF_ULONG ;    // if m_lEUType == 2, Index of Shell  in SmBrepData::m_vShells
                                                            //    note: a backPtr needing special handling
  ULONG             m_lLoop           = SM_UNDEF_ULONG ;    // Index of Edgeuse's loop            in SmBrepData::m_vLoops
                                                            //    note: a backPtr needing special handling
  SmBoolean         m_bOrientation    = UNSURE ;            // orientation of Edgeuse1 relative to Edge->orientation, [TRUE=Same, FALSE=Opposite]
                                                            // Edgeuse2 bOrientation == !Edgeuse1.m_bOrientation
  ULONG             m_lUVCurve        = SM_UNDEF_ULONG ;    // Index of Edgeuse's UV Curve             in SmBrepData::m_vUVCurves (SM_NO_OBJECT_INDEX if no curve)
  long              m_lNextEU         = SM_UNDEF_ULONG ;    // Index of Edgeuse's NEXT radial edgeuse  in SmBrepData::m_vEdgeuses
                                                            //    Note: negative indices are incremented and indicate the next EU 
                                                            //          is the mate (pEdgeuse2) edgeuse and not the edgeuse itself (pEdgeuse1).
                                                            //    Ex: m_lNextEU =  2, the next EU is SmBrepData::m_vEdgeuses[2].m_pEdgeuse1 
                                                            //        m_lNextEU = -3, the next EU is SmBrepData::m_vEdgeuses[2].m_pEdgeuse2 
  long              m_lMateNextEU     = SM_UNDEF_ULONG ;    // Index of Edgeuse->Mate's NEXT radial edgeuse in SmBrepData::m_vEdgeuses
                                                            //    Note: negative indices are incremented and indicate the next EU 
                                                            //          is the mate (pEdgeuse2) edgeuse and not the edgeuse itself (pEdgeuse1).
  bool              m_bDownwardLU     = FALSE;              // TRUE - This Edgeuse is part of the loopuse on the downward side of the surface.
                                                            //        currently used only during import of XTBreps (JT toolkit import)
  ULONG             m_lMateFlags      = 0 ;                 //
  ULONG             m_lMateUserIndex1 = SM_UNDEF_ULONG ;    //
  ULONG             m_lMateUserIndex2 = SM_UNDEF_ULONG ;    //
                                                  
  SmTArray<ULONG>   m_sMateAttributes;                      //
                                                            
  // when edge is attached to a face                        
  SmEdgeuse       * m_pEdgeuse1 = NULL ;                    // ptr to corresponding primary SmEdgeuse - once it gets built
  SmEdgeuse       * m_pEdgeuse2 = NULL ;                    // ptr to corresponding mate    SmEdgeuse - once it gets built
                                                            
  // When primary or mate edgeuse is connected to a vertex
  SmVertexuse     * m_pVertexuse1 = NULL ;                  // ptr to corresponding primary SmEdgeuse SmVertexuse - once it gets built
  SmVertexuse     * m_pVertexuse2 = NULL ;                  // ptr to corresponding mate    SmEdgeuse SmVertexuse - once it gets built
  
  // constructor
  SmEUData() { }
                     
  // destructor
  ~SmEUData()  { ReSet() ; }
  void ReSet() { SmTopologyData::ReSet() ;  // eff: empty SmTArray<ULONG> m_sAttributes, default SmTopologyData values
                 m_lEUType         = SM_UNDEF_ULONG ;     
                 m_lEdge           = SM_UNDEF_ULONG ;       
                 m_lVertex         = SM_UNDEF_ULONG ;     
                 m_lShell          = SM_UNDEF_ULONG ;      
                 m_lLoop           = SM_UNDEF_ULONG ;       
                 m_bOrientation    = UNSURE ;
                 m_lUVCurve        = SM_UNDEF_ULONG ;    
                 m_lNextEU         = SM_UNDEF_ULONG ;     
                 m_lMateNextEU     = SM_UNDEF_ULONG ; 
                 m_lMateFlags      = 0 ;
                 m_lMateUserIndex1 = SM_UNDEF_ULONG ;
                 m_lMateUserIndex2 = SM_UNDEF_ULONG ;
                 m_sMateAttributes.ReSet() ;
                 m_pEdgeuse1       = NULL ; 
                 m_pEdgeuse2       = NULL ; 
                 m_pVertexuse1     = NULL ; 
                 m_pVertexuse2     = NULL ; 
              }

  // no operator=, shallow copies okay

  // I/O
  SmStatus WriteToDB(ULONG lDBVersionNumber,SmDatabaseIO & rDB,SmBoolean bPersistAttribs=TRUE);
  SmStatus ReadFromDB(const SmContext        & crContext,
                      ULONG                    lDBVersionNumber,
                      SmTArray<SmAttribute*> & rAllAttributes,
                      SmDatabaseIO           & rDB, 
                      SmBoolean                bPersistAttribs=TRUE);

} ; // end class SmEUData

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmVertex 

NOTES: static class object - don't add virtual methods to SmVertexData. 
***********************************************************************/
class SM_EXPORT SmVertexData : public SmTopologyData
{
 public:
  // inherited: 
  //   SmTArray<ULONG> SmAttributeData::m_sAttributes;      // this TopoObj's list of attribute indices in the translation global attribute array
  //   ULONG           SmTopologyData::m_lFlags;            // User definable bit array,    [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex1;       // First  User Definable Index, [persistent - written to and read from file]
  //   ULONG           SmTopologyData::m_lUserIndex2;       // Second User Definable Index, [persistent - written to and read from file]
  //   void*           SmTopologyData::m_pUserPtr1;         // User Definable ptr,          [not persistent - not written to and read from file]
  //                                                        //                              [not owned - not freed by destructor - caller can free if needed]
#ifdef SM_USE_OLDTOL
  SM_OLDTOL_LINE SmZoneTol3d m_sZoneTol3d = 0.0 ;   
#endif // SM_USE_OLDTOL                         
  SmPoint3d                  m_vPoint ;             // vertex position
  SmVertex                 * m_pVertex = NULL ;     // ptr to corresponding SmVertex - once it gets built

  // constructor
  SmVertexData() { }

  // destructor
  ~SmVertexData() { ReSet() ; }
  void ReSet()    { SmTopologyData::ReSet() ;  // eff: empty SmTArray<ULONG> m_sAttributes, default SmTopologyData values
#ifdef SM_USE_OLDTOL
     SM_OLDTOL_LINE m_sZoneTol3d = 0.0 ;
#endif // SM_USE_OLDTOL   
                    m_vPoint.SetUninitialized() ;
                    m_pVertex    = NULL ; 
                  }                 

  // no operator=, shallow copies okay

  // I/O
  SmStatus WriteToDB(ULONG lDBVersionNumber,SmDatabaseIO & rDB,SmBoolean bPersistAttribs=TRUE);
  SmStatus ReadFromDB(const SmContext        & crContext,
                      ULONG                    lDBVersionNumber,
                      SmTArray<SmAttribute*> & rAllAttributes,
                      SmDatabaseIO           & rDB, 
                      SmBoolean                bPersistAttribs=TRUE);

} ; // end class SmVertexData

/*******************************************************************//**
PURPOSE: Serialized equivalent to SmBrep 

NOTES: 
***********************************************************************/
class SM_EXPORT SmBrepData : public SmObject
{                                         
public:                                   
  SmDataSourceType              m_eDataSourceType = SM_DS_UNKNOWN;     // oneof SM_DS_SMLIB, IS_DS_IGES, ...
  SmBoolean                     m_bManifold       = UNSURE ;           // TRUE = a manifold solid for which shells are not duplicated.
  SM_OLDTOL_LINE SmZoneTol3d    m_sZoneTol3d      = SM_ZONE_TOL_3D ;   // OldTol: Brep ZoneTol3d init value for all brep contained object ZoneTol3d values
                                
  SM_NEWTOL_LINE double         m_dThisModelSizeEstimate               // NewTol: UserInterface to Tolerances : used only to set system session m_sZoneTol3d value as
                                          = SM_MODEL_SIZE_ESTIMATE ;           
  SM_NEWTOL_LINE double         m_dThisLargeSmallSizeRatio             // NewTol: To model Pinholes in Battleships - give small topology tighter tolerances
                                          = SM_LARGE_SMALL_SIZE_RATIO ; 
                                
  SmApproxTol3d                 m_sApproxTol3d    = SM_APPROX_TOL_3D ; // Tol used when approximating geometry to write as BSplines.
  ULONG                         m_lStartRegion    = 0 ;                // Index of Brep's first region in m_vRegions. (always 0 = the infinite region) 
  ULONG                         m_lNumRegions     = 0 ;                // Number of regions in this Brep. (must be at least 1 after FromBrep() call) 
                                
  SmVector3d                    m_vColor          = {0.0, 0.0, 0.0} ;  // Brep Color, default:[0.0 0.0 0.0]
  SmBoolean                     m_bIsGeomBorrowed = FALSE ;            // TRUE  = curves & surfaces in {m_vUVCurves, m_v3DCurves, m_vSurfaces} arrays are borrowed, FALSE = aren't  
  ULONG                         m_lHealerVersion  = 0 ;                // Healer version at the time the file was written. (when m_lHealerVersion < SM_HEALER_VERSION) Brep Healing will run when reading this file.)
                                                   
  // topology objects                                                                      
  SmTArray<SmRegionData>      & m_vRegions;       // regions array,  tgts of SmBrepData   ::{m_lStartRegion,  m_lNumRegions}  index block
  SmTArray<SmShellData>       & m_vShells;        // shells array,   tgts of SmRegionData ::{m_lStartShell,   m_lNumShells}   index blocks
  SmTArray<ULONG>             & m_vCFaces;        // scratch array - old CFaces now translate to Face sets on input
                                                      
  SmTArray<SmFaceuseData>     & m_vFaceuses;      // faceuses array, tgts of SmShellData  ::{m_lFaceuseStart, m_lNumFaceuses} index blocks
  SmTArray<SmFaceData>        & m_vFaces;         // faces array,    tgts of SmFaceuseData::{m_lFace} indices
  SmTArray<SmLoopData>        & m_vLoops;         // loops array,    tgts of SmFaceData   ::{m_lStartLoop,    m_lNumLoops}    index blocks
                                                      
  SmTArray<ULONG>             & m_vCEdges;        // scratch array - old CEdges now translate as Edge sets on input
  SmTArray<SmEUData>          & m_vEdgeuses;      // edgeuses array, tgts of SmLoopData   ::{m_lStartEU, m_lNumEUs}           index blocks
                                                  //                  contains: 1 SmEUData for each pair of mated edgeuses              
                                                  //                          : No SmEUData for each WireEdge                            
                                                  //                          : No SmEUData for LoopVertex edgeuses                     
                                                  //                          : No SmEUData for ShellVertices                           
  SmTArray<SmEdgeData>        & m_vEdges;         // edges array,    tgts of SmEUData     ::{m_lEdge}                         indices
                                                  //                         SmShellData  ::{m_lStartEdge, m_lNumEdges}       index blocks
  SmTArray<SmVertexData>      & m_vVertices;      // vertices array, tgts of SmEdgeData   ::{m_lStartVertex, m_lEndVertex}    indices
                                                  //                         SmShellData  ::{m_lVertex}                       indices
                                                  //                         SmLoopData   ::{m_lVertex}                       indices
                                                      
  SmTArray<SmCurve*>          & m_vUVCurves;      // opt UVTrimCurves array, tgts of SmEUData  ::{m_lUVCurve} indices, (owned by this when m_bIsGeomBorrowed==FALSE)
  SmTArray<SmCurve*>          & m_v3DCurves;      // 3DCurves array,         tgts of SmEdgeData::{m_lCurve}   indices, (owned by this when m_bIsGeomBorrowed==FALSE)
  SmTArray<SmSurface*>        & m_vSurfaces;      // 3DSurfaces array,       tgts of SmFaceData::{m_lSurface} indices, (owned by this when m_bIsGeomBorrowed==FALSE)
  SmTArray<ULONG>             & m_vAttributes;    // indices of Brep's attributes in associated SmTArray<SmAttribute*> sAttributes array.
                                
public:                         
  // construct empty SmBrepData
  SmBrepData(SmBoolean        bManifold = FALSE, 
             SmDataSourceType eDSType   = SM_DS_UNKNOWN);

  // destructor
  virtual ~SmBrepData() ;

  // ReSet
  void ReSet(); 

  // simple data access
  SmDataSourceType GetDataSourceType()                     { return m_eDataSourceType ; }
  ULONG            GetHealerVersion()                      { return m_lHealerVersion; }
  void SetDataSourceType(SmDataSourceType eDataSourceType) { m_eDataSourceType = eDataSourceType ; }
  void SetHealerVersion(ULONG lHealerVersion)              { m_lHealerVersion  = lHealerVersion; }

  // build SmBrepData Object lists from SmBrep Topology Graph (replace pointers with indices)
  //  - mirror function = SmBrep::MakeTopologyFromData
  SmStatus FromBrep
  (
    const SmBrep           & crBrep,           // in : target Brep                 
    SmTArray<SmAttribute*> & rAllAttributes    // i/o: attribute accumulation array
  );

  // Primary User Interface: read curves, surfaces, boolean trees, and/or breps with attributes
  static SmStatus ReadPartFromFile
  (
    const SmContext      & crContext,         // in : memory context for constructing objects from file
    const TCHAR          * cInputFileName,    // in : target file to read                              
    SmTArray<SmCurve*>   & r3DCurves,         // out: array of curves in file                          
    SmTArray<SmSurface*> & rSurfaces,         // out: array of surfaces in file                        
    SmTArray<long>       & rBooleanTreeNodes, // out: array of boolean trees in file                   
    SmTArray<SmBrep*>    & crBreps,           // out: array of Brep models                             
    SmFileType             eType = SM_ASCII   // in : oneof SM_ASCII, SM_BINARY, default:[SM_ASCII]                        
  );

  // Primary User Interface: write curves, surfaces, boolean trees, and/or breps with attributes
  static SmStatus WritePartToFile
  (
    const std::string          & cOutputFileName,        // in : target File name                                                  
    const SmTArray<SmCurve*>   & cr3DCurves,             // in : Array of curves to place in file                                  
    const SmTArray<SmSurface*> & crSurfaces,             // in : Array of surfaces to place in file                                
    const SmTArray<long>       & crBooleanTreeNodes,     // in : Boolean Trees to place in file (see SmMerge::BooleanTreeNodes)    
    const SmTArray<SmBrep*>    & crBreps,                // in : Array of SmBrep objects to place in file                          
    SmFileType                   eType = SM_ASCII,       // in : Specify output type: oneof                                        
                                                         //      SM_ASCII  = Database is an ASCII file                             
                                                         //      SM_BINARY = Database is a Binary format                           
    SmBoolean                    bNewFile = FALSE,       // in : TRUE = open file and rewrite contents                             
                                                         //      FALSE= open file and append to end                                
    SmBoolean                    bWriteAsBSplines=FALSE, // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes      
                                                         //      FALSE= Write native formats for nonBSplines                       
    SmApproxTol3d        sApproxTol3d=SM_APPROX_TOL_3D   // in : only used when bWriteAsBSplines is TRUE                           
                                                         //      default:[SM_APPROX_TOL_3D = SM_ZONE_TOL_3D/2 = 5.0e-6]            
  );


  // Primary User Interface: write curves, surfaces, boolean trees, and/or breps with attributes
  static SmStatus WritePartToFile
  (
    const TCHAR                * cOutputFileName,        // in : target File name                                                  
    const SmTArray<SmCurve*>   & cr3DCurves,             // in : Array of curves to place in file                                  
    const SmTArray<SmSurface*> & crSurfaces,             // in : Array of surfaces to place in file                                
    const SmTArray<long>       & crBooleanTreeNodes,     // in : Boolean Trees to place in file (see SmMerge::BooleanTreeNodes)    
    const SmTArray<SmBrep*>    & crBreps,                // in : Array of SmBrep objects to place in file                          
    SmFileType                   eType = SM_ASCII,       // in : Specify output type: oneof                                        
                                                         //      SM_ASCII  = Database is an ASCII file                             
                                                         //      SM_BINARY = Database is a Binary format                           
    SmBoolean                    bNewFile = FALSE,       // in : TRUE = open file and rewrite contents                             
                                                         //      FALSE= open file and append to end                                
    SmBoolean                    bWriteAsBSplines=FALSE, // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes      
                                                         //      FALSE= Write native formats for nonBSplines                       
    SmApproxTol3d        sApproxTol3d=SM_APPROX_TOL_3D   // in : only used when bWriteAsBSplines is TRUE                           
                                                         //      default:[SM_APPROX_TOL_3D = SM_ZONE_TOL_3D/2 = 5.0e-6]            
  );


  // read BrepData from file without Attribute indices but not the attributes
  static SmStatus ReadFromFile
  (
    const SmContext & crContext,          // in : context for new object construction     
    const TCHAR     * cInputFileName,     // in : target file to read                     
    SmBrepData     *& rpNewBrepData,      // out: read BrepData                           
    SmFileType        eType = SM_ASCII    // in : one of SM_ASCII or SM_BINARY            
  );

  // write BrepData to file - pass through function to TCHAR version
  SmStatus WriteToFile
  (
    const std::string& cOutputFileName,     // in : target file name                                                
    SmFileType    eType = SM_ASCII,         // in : oneof SM_ASCII or SM_BINARY                                     
    SmBoolean     bNewFile = FALSE,         // in : TRUE = open file and rewrite contents                           
                                            //      FALSE= open file and append to end                                  
    SmBoolean     bWriteAsBSplines = FALSE, // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes 
                                            //      FALSE= Write native formats for nonBSplines                         
                                            //      default:[FALSE]                                                     
    SmApproxTol3d sApproxTol3d = 0.0        // in : only used when bWriteAsBSplines==TRUE                           
                                            //      when approximating geometry as BSplines for output.                 
                                            //      0.0 = use m_sApproxTol3d, default:[0.0]                             
  ) const;

  // write BrepData to file - attribute indices are written but not the attribute list
  SmStatus WriteToFile
  (
    const TCHAR * cOutputFileName,          // in : target file name                                                
    SmFileType    eType = SM_ASCII,         // in : oneof SM_ASCII or SM_BINARY                                     
    SmBoolean     bNewFile = FALSE,         // in : TRUE = open file and rewrite contents                           
                                            //      FALSE= open file and append to end                                  
    SmBoolean     bWriteAsBSplines = FALSE, // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes 
                                            //      FALSE= Write native formats for nonBSplines                         
                                            //      default:[FALSE]                                                     
    SmApproxTol3d sApproxTol3d = 0.0        // in : only used when bWriteAsBSplines==TRUE                           
                                            //      when approximating geometry as BSplines for output.                 
                                            //      0.0 = use m_sApproxTol3d, default:[0.0]                             
  ) const;

  static SmStatus ReadAssemblyFromFile
  (
    const SmContext & crContext,          // in : context for new object construction     
    const TCHAR     & crInputFileName,    // in : target file to read                     
    SmAssembly     *& rpNewAssembly,      // out: Root of assembly tree                   
    SmFileType        eType = SM_ASCII    // in : one of SM_ASCII or SM_BINARY            
  );

  static SmStatus WriteAssemblyToFile
  (
    const TCHAR      & crOutputFileName,        // in : target file name                                                
    const SmAssembly & crAssembly,              // in : Root SmAssembly of the assemlby tree to place in file           
    SmFileType         eType = SM_ASCII,        // in : oneof SM_ASCII or SM_BINARY                                     
    SmBoolean          bNewFile = FALSE,        // in : TRUE = open file and rewrite contents                           
                                                //      FALSE= open file and append to end                                  
    SmBoolean          bWriteAsBSplines = FALSE,// in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes 
                                                //      FALSE= Write native formats for nonBSplines                         
                                                //      default:[FALSE]                                                     
    SmApproxTol3d      sApproxTol3d = 0.0       // in : only used when bWriteAsBSplines==TRUE                           
                                                //      when approximating geometry as BSplines for output.                 
                                                //      0.0 = use m_sApproxTol3d, default:[0.0]                             
  );

  // read BrepData from stream with attribute indices - make placeholder attribute in rAllAttributes for each index
  static SmStatus ReadFromDB
  (
    const SmContext        & crContext,             // in : context for constructing objects from stream                
    ULONG                    lDBVersionNumber,      // in : target stream's version number                              
    SmTArray<SmAttribute*> & rAllAttributes,        // out: Array of all read attributes when lDBVersionNumber > 30     
    SmBrepData            *& rpNewBrepData,         // out: Array of all read Brep Models                               
    SmDatabaseIO           & rDB,                   // in : contains target stream                                      
    SmBoolean                bPersistAttribs = TRUE // in : TRUE = read attributes, false = don't, default:[TRUE]       
  ) ;  

  // write BrepData to stream - append Curve and Surface attribute pointers to rAllAttributes list.  
  SmStatus WriteToDB
  (
    const SmContext                       & crContext,                // in : context for constructing objects                                
    ULONG                                   lDBVersionNumber,         // in : target stream's version number                                  
    SmTArray<SmAttribute*>                & rAllAttributes,           // out: Array of all attributes accumulated during write                
    SmMapTypeToType<SmAttribute *, ULONG> & rAttrMap,                 // out: accumulated attribute to object map                             
    SmDatabaseIO                          & rDB,                      // in : contains target stream                                          
    SmBoolean                               bWriteAsBSplines = FALSE, // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes 
                                                                      //      FALSE= Write native formats for nonBSplines                     
    SmApproxTol3d                           sApproxTol3d = 0.0,       // in : only used when bWriteAsBSplines==TRUE                           
                                                                      //      when approximating geometry as BSplines for output.             
                                                                      //      0.0 = use m_sApproxTol3d, default:[0.0]                         
    SmBoolean                               bPersistAttribs = TRUE    // in : TRUE = write attributes, false = don't, default:[TRUE]          
  ) const;

  // read one curve from stream
  static SmStatus ReadCurveFromDB
  (
    const SmContext        & crContext,              // in : context for new object construction                                
    ULONG                    lDim,                   // in : size of each control Point, i.e. 2 for 2d                          
    ULONG                    lDBVersionNumber,       // in : controls backward compatible reads                                 
    SmTArray<SmAttribute*> & rAllAttributes,         // i/o: augmented with curve's attributes when lDBVersionNumber > 30    
    SmCurve               *& rpNewCurve,             // out: SmCurve built from SmCurve Data                                    
    SmDatabaseIO           & rDB,                    // in : contains target stream                                             
    SmBoolean                bPersistAttribs = TRUE  // in : TRUE = read attributes, false = don't, default:[TRUE]              
  ) ; 

  // write one curve to stream - append attribute pointers to rAllAttributes list.
  static SmStatus WriteCurveToDB
  (
    const SmContext                      & crContext,                 // in : context for new object construction                            
    SmCurve                              * crCrv,                     // in : target curve to output                                         
    ULONG                                  lDBVersionNumber,          // in : database version to get proper sequence of writes              
    SmTArray<SmAttribute*>               & rAllAttributes,            // i/o: attribute accumulation list                                      
    SmMapTypeToType<SmAttribute *, ULONG>& rAttrMap,                  // i/o: attribute to object map for each accumulated attribute           
    SmDatabaseIO                         & rDB,                       // in : contains target stream                                         
    SmBoolean                              bWriteAsBSplines = FALSE,  // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes
                                                                      //      FALSE= Write native formats for nonBSplines                    
    SmApproxTol3d                          sApproxTol3d = 0.0,        // in : only used when bWriteAsBSplines==TRUE                          
                                                                      //      when approximating geometry as BSplines for output.            
                                                                      //      0.0 = use m_sApproxTol3d, default:[0.0]                        
    SmBoolean                              bPersistAttribs = TRUE     // in : TRUE = write attributes, false = don't, default:[TRUE]         
  );                                                               

  // read one surface from stream - replace attribute indices with attributes passed in rAllAttributes.
  static SmStatus ReadSurfaceFromDB
  (
    const SmContext        & crContext,                // in : context for new object construction                                            
    ULONG                    lDBVersionNumber,         // in : controls backward compatible reads                                             
    SmTArray<SmAttribute*> & rAllAttributes,           // in : array of attributes to access with attribute indices read from stream.         
    SmSurface             *& rpNewSurface,             //      assumes:[array is larger than larger attribute index stored for this surface]  
    SmDatabaseIO           & rDB,                      // out: read surface                                                                   
    SmBoolean                bPersistAttribs = TRUE    // in : contains target stream                                                         
                                                       // in : TRUE = read attributes, false = don't, default:[TRUE]                          
  );                                                   


  // write one surface to stream - append attribute pointers to rAllAttributes list.
  static SmStatus WriteSurfaceToDB
  (
    const SmContext                       & crContext,                 // in : context for new object construction                              
    SmSurface                             * crSrf,                     // in : target surface to output                                         
    ULONG                                   lDBVersionNumber,          // in : database version to get proper sequence of writes                
    SmTArray<SmAttribute*>                & rAllAttributes,            // i/o: attribute accumulation list                                   
    SmMapTypeToType<SmAttribute *, ULONG> & rAttrMap,                  // i/o: attribute to object map for each accumulated attribute        
    SmDatabaseIO                          & rDB,                       // in : contains target stream                                          
    SmBoolean                               bWriteAsBSplines = FALSE,  // in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes 
                                                                       //       FALSE= Write native formats for nonBSplines                     
    SmApproxTol3d                           sApproxTol3d = 0.0,        // in : only used when bWriteAsBSplines==TRUE                           
                                                                       //       when approximating geometry as BSplines for output.             
                                                                       //       0.0 = use m_sApproxTol3d, default:[0.0]                         
    SmBoolean                               bPersistAttribs = TRUE     // in : TRUE = write attributes, false = don't, default:[TRUE]          
  );
    
  // read combination of stand-alone curves, surfaces, boolean trees, and breps with attributes
  static SmStatus ReadPartFromDB
  (
    const SmContext      & crContext,            // in : context for constructing objects from stream                         
    SmTArray<SmCurve*>   & r3DCurves,            // out: Array of curves to get from stream                                   
    SmTArray<SmSurface*> & rSurfaces,            // out: Array of surfaces to get from stream                                 
    SmTArray<long>       & rBooleanTreeNodes,    // out: Boolean Trees to get from stream (see SmMerge::BooleanTreeNodes)     
    SmTArray<SmBrep*>    & crBreps,              // out: Array of SmBrep objects to get from stream                           
    SmDatabaseIO         & rDB                   // in : contains target stream                                               
  );

  // write combination of stand-alone curves, surfaces, boolean trees, and breps with attributes
  static SmStatus WritePartToDB
  (
    const SmTArray<SmCurve*>   & cr3DCurves,              // in : Array of curves to place in stream                                
    const SmTArray<SmSurface*> & crSurfaces,              // in : Array of surfaces to place in stream                              
    const SmTArray<long>       & crBooleanTreeNodes,      // in : Boolean Trees to place in file (see SmMerge::BooleanTreeNodes)    
    const SmTArray<SmBrep*>    & crBreps,                 // in : Array of SmBrep objects to place in strea                         
    SmDatabaseIO               & rDB,                     // in : contains target stream                                            
    SmBoolean                    bWriteAsBSplines = FALSE,// in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes   
                                                          //      FALSE= Write native formats for nonBSplines                       
    SmApproxTol3d                sApproxTol3d = 0.0       // in : only used when bWriteAsBSplines==TRUE                             
                                                          //      when approximating geometry as BSplines for output.               
                                                          //      0.0 = use m_sApproxTol3d, default:[0.0]                           
  );

  static SmStatus ReadAssemblyFromDB
  (
    const SmContext  & crContext,            // in : context for constructing objects from stream                         
    SmAssembly      *& crBreps,              // out: Root SmAssembly of the assembly tree                                 
    SmDatabaseIO     & rDB                   // in : contains target stream                                               
  );

  static SmStatus WriteAssemblyToDB
  (
    const SmAssembly  & crAssembly,              // in : Root SmAssembly of the assemlby tree to place in stream             
    SmDatabaseIO      & rDB,                     // in : contains target stream                                            
    SmBoolean           bWriteAsBSplines = FALSE,// in : TRUE = Approx nonBSplineGeometry and write out as BSplineShapes   
                                                 //      FALSE= Write native formats for nonBSplines                       
    SmApproxTol3d       sApproxTol3d = 0.0       // in : only used when bWriteAsBSplines==TRUE                             
                                                 //      when approximating geometry as BSplines for output.               
                                                 //      0.0 = use m_sApproxTol3d, default:[0.0]                           
  );

  // write one attribute to stream
  static SmStatus WriteAttributeToDB
  (
    const SmAttribute * pAttribute,     // in : target attribute           
    SmDatabaseIO      & rDB             // in : I/O data                   
  );

  // read one attribute from stream
  static SmStatus ReadAttributeFromDB
  (
    const SmContext & crContext,         // in : context for new object construction   
    SmAttribute    *& rpNewAttribute,    // out: read attribute                        
    SmDatabaseIO    & rDB                // in : contains target stream                
  );

} ; // end class SmBrepData 

#endif // no __SMBREPDATA_H__


