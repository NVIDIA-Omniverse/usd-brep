// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPolyBrepData.h
* PURPOSE: Header file for SmPolyBrepData and associated classes.
**********************************************************************/

#ifndef __SMPOLYBREPDATA_H__
#define __SMPOLYBREPDATA_H__

#ifndef __SMOBJECT_H__
#include <SmObject.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMSARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMPOLY_H__
#include <SmPoly.h>
#endif

#ifndef __SMDATABASEIO_H__
#include <SmDatabaseIO.h>
#endif

#ifndef __SMATTRIBUTE_H__
#include <SmAttribute.h>
#endif

#ifndef __SMBREPDATA_H__
#include <SmBrepData.h>
#endif

// forward declarations

class SmCopyPolyBrepMap ;

/*******************************************************************//**
PURPOSE: Class SmPolyRegionData

NOTES: static class object - don't add virtual methods to SmPolyRegionData. 
***********************************************************************/
class SM_EXPORT SmPolyRegionData : public SmTopologyData
{
public:
  ULONG    m_lStartPolyShell; // The start PolyShell
  ULONG    m_lNumPolyShells;  // The number of PolyShells
                              // in the PolyRegion.

  SmStatus WriteToDB(ULONG              lDBVersionNumber,         // NotUsed: in :
                     SmDatabaseIOFile & rDB) ;                    // in :

  SmStatus ReadFromDB(const SmContext        & crContext,         // in : 
                      ULONG                    lDBVersionNumber,  // NotUsed: in : 
                      SmTArray<SmAttribute*> & rAllAttributes,    // in : 
                      SmDatabaseIOFile       & rDB) ;             // in : 

  SmPolyRegionData()
  {
    m_lStartPolyShell = SM_UNDEF_ULONG;
    m_lNumPolyShells = SM_UNDEF_ULONG;
  }

  ~SmPolyRegionData()
  {
    SmAttributeData::ReSet(); // empty SmTArray<ULONG> m_sAttributes
    m_lStartPolyShell = SM_UNDEF_ULONG;
    m_lNumPolyShells = SM_UNDEF_ULONG;
  }

} ; // end class SmPolyRegionData

/*******************************************************************//**
PURPOSE: Class SmPolyShellData

NOTES: static class object - don't add virtual methods to SmPolyShellData.  
***********************************************************************/
class SM_EXPORT SmPolyShellData : public SmTopologyData
{
public:
  ULONG    m_lStartPolyFace; // The start PolyFace
  ULONG    m_lNumPolyFaces;  // The number of PolyFace
                             // in the PolyShell.

  SmStatus WriteToDB(ULONG              lDBVersionNumber,         // NotUsed: in :
                     SmDatabaseIOFile & rDB) ;                    // in :

  SmStatus ReadFromDB(const SmContext        & crContext,         // in : 
                      ULONG                    lDBVersionNumber,  // NotUsed: in : 
                      SmTArray<SmAttribute*> & rAllAttributes,    // in : 
                      SmDatabaseIOFile       & rDB) ;             // in : 

  SmPolyShellData()
  {
    m_lStartPolyFace = SM_UNDEF_ULONG;
    m_lNumPolyFaces = SM_UNDEF_ULONG;
  }

  ~SmPolyShellData()
  {
    SmAttributeData::ReSet(); // empty SmTArray<ULONG> m_sAttributes
    m_lStartPolyFace = SM_UNDEF_ULONG;
    m_lNumPolyFaces = SM_UNDEF_ULONG;
  }


} ; // end class SmPolyShellData

/*******************************************************************//**
PURPOSE: Class SmPolyFaceData

NOTES: static class object - don't add virtual methods to SmPolyFaceData.  
***********************************************************************/
class SM_EXPORT SmPolyFaceData : public SmTopologyData
{
public:
  SmZoneTol3d m_sZoneTol3d;     // Tolerance of this PolyFace
  ULONG       m_lStartPolyLoop; // First (outer) PolyLoop of this PolyFace
  ULONG       m_lNumPolyLoops;  // Additional PolyLoops after first are inner PolyLoops
  ULONG       m_lFaceIndex;     // Index of SmFace in BrepData 
  double      m_dMaxDeviation;
  SmVector3d  m_vNormal;

  SmStatus WriteToDB(ULONG              lDBVersionNumber,         // NotUsed: in :
                     SmDatabaseIOFile & rDB) ;                    // in :

  SmStatus ReadFromDB(const SmContext        & crContext,         // in : 
                      ULONG                    lDBVersionNumber,  // NotUsed: in : 
                      SmTArray<SmAttribute*> & rAllAttributes,    // in : 
                      SmDatabaseIOFile       & rDB) ;             // in : 

  SmPolyFaceData()
  {
    m_sZoneTol3d = 0.0;
    m_lStartPolyLoop = SM_UNDEF_ULONG;
    m_lNumPolyLoops = SM_UNDEF_ULONG;
    m_lFaceIndex = SM_UNDEF_ULONG;
    m_dMaxDeviation = -SM_BIG_DOUBLE;
    m_vNormal.SetUninitialized();
  }

  ~SmPolyFaceData()
  {
    SmAttributeData::ReSet(); // empty SmTArray<ULONG> m_sAttributes
    m_sZoneTol3d = -SM_UNDEF_DOUBLE;
    m_lStartPolyLoop = SM_UNDEF_ULONG;
    m_lNumPolyLoops = SM_UNDEF_ULONG;
    m_lFaceIndex = SM_UNDEF_ULONG;
    m_dMaxDeviation = -SM_BIG_DOUBLE;
    m_vNormal.SetUninitialized();
  }

} ; // end class SmPolyFaceData

/*******************************************************************//**
PURPOSE: Class SmPolyLoopData

NOTES: static class object - don't add virtual methods to SmPolyLoopData.  
***********************************************************************/
class SM_EXPORT SmPolyLoopData : public SmTopologyData
{
public:
  ULONG     m_lStartPolyEdge; // First PolyEdge of this PolyLoop
  ULONG     m_lNumPolyEdge;   // Number of PolyEdges in this PolyLoop.
  SmBoolean m_bOuterLoop;     // TRUE - outer, FALSE - inner
  SmBoolean m_bClosed;        // If TRUE the loop is closed

  SmStatus WriteToDB(ULONG              lDBVersionNumber,         // NotUsed: in :
                     SmDatabaseIOFile & rDB) ;                    // in :

  SmStatus ReadFromDB(const SmContext        & crContext,         // in : 
                      ULONG                    lDBVersionNumber,  // NotUsed: in : 
                      SmTArray<SmAttribute*> & rAllAttributes,    // in : 
                      SmDatabaseIOFile       & rDB) ;             // in : 

  SmPolyLoopData()
  {
    m_lStartPolyEdge = SM_UNDEF_ULONG;
    m_lNumPolyEdge = SM_UNDEF_ULONG;
    m_bOuterLoop = UNSURE;
    m_bClosed = UNSURE;
  }

  ~SmPolyLoopData()
  {
    SmAttributeData::ReSet(); // empty SmTArray<ULONG> m_sAttributes
    m_lStartPolyEdge = SM_UNDEF_ULONG;
    m_lNumPolyEdge = SM_UNDEF_ULONG;
    m_bOuterLoop = UNSURE;
    m_bClosed = UNSURE;
  }

} ; // end class SmPolyLoopData

/*******************************************************************//**
PURPOSE: Class SmPolyEdgeData

NOTES: static class object - don't add virtual methods to SmPolyEdgeData.  
***********************************************************************/
class SM_EXPORT SmPolyEdgeData : public SmTopologyData
{
public:
  SmZoneTol3d m_sZoneTol3d;
  ULONG       m_lVertex;      // Vertex of this edge
  ULONG       m_lNextRadialE; // Next edge in radial list - may be self
  ULONG       m_lNextEdgeAtV; // Next edge starting at this vertex - may be self
  ULONG       m_lAuxData;     // Index into auxillary data (0-means none present)

  SmStatus WriteToDB(ULONG              lDBVersionNumber,         // NotUsed: in :
                     SmDatabaseIOFile & rDB) ;                    // in :

  SmStatus ReadFromDB(const SmContext        & crContext,         // in : 
                      ULONG                    lDBVersionNumber,  // NotUsed: in : 
                      SmTArray<SmAttribute*> & rAllAttributes,    // in : 
                      SmDatabaseIOFile       & rDB) ;             // in : 

  SmPolyEdgeData()
  {
    m_sZoneTol3d = 0.0;
    m_lVertex = SM_UNDEF_ULONG;
    m_lNextRadialE = SM_UNDEF_ULONG;
    m_lNextEdgeAtV = SM_UNDEF_ULONG;
    m_lAuxData = 0;
  }

  ~SmPolyEdgeData()
  {
    SmAttributeData::ReSet(); // empty SmTArray<ULONG> m_sAttributes
    m_sZoneTol3d = -SM_BIG_DOUBLE;
    m_lVertex = SM_UNDEF_ULONG;
    m_lNextRadialE = SM_UNDEF_ULONG;
    m_lNextEdgeAtV = SM_UNDEF_ULONG;
    m_lAuxData = SM_UNDEF_ULONG;
  }

} ; // end class SmPolyEdgeData

/*******************************************************************//**
PURPOSE: Class SmPolyVertexData

NOTES: static class object - don't add virtual methods to SmPolyVertexData. 
***********************************************************************/
class SM_EXPORT SmPolyVertexData : public SmTopologyData
{
public:
  SmZoneTol3d m_sZoneTol3d;
  SmPoint3d   m_vPoint;
  ULONG       m_lEdgeList;  // Pointer to list of edges starting at this vertex

  SmStatus   WriteToDB(ULONG              lDBVersionNumber,         // NotUsed: in :
                       SmDatabaseIOFile & rDB) ;                    // in :

  SmStatus   ReadFromDB(const SmContext        & crContext,         // in : 
                        ULONG                    lDBVersionNumber,  // NotUsed: in : 
                        SmTArray<SmAttribute*> & rAllAttributes,    // in : 
                        SmDatabaseIOFile       & rDB) ;             // in : 

  SmPolyVertexData()
  {
    m_sZoneTol3d = 0.0;
    m_vPoint.SetUninitialized();
    m_lEdgeList = SM_UNDEF_ULONG;
  }

  ~SmPolyVertexData()
  {
    SmAttributeData::ReSet(); // empty SmTArray<ULONG> m_sAttributes
    m_sZoneTol3d = -SM_BIG_DOUBLE;
    m_vPoint.SetUninitialized();
    m_lEdgeList = SM_UNDEF_ULONG;
  }

} ; // end class SmPolyVertexData

/*******************************************************************//**
PURPOSE: Class SmPolyBrepData

NOTES: 
***********************************************************************/
class SM_EXPORT SmPolyBrepData : public SmObject
{
public:
  SmZoneTol3d m_sZoneTol3d;                     // Tolerance of this PolyBrep
  ULONG       m_lStartRegion;                   // First region of this PolyBrep
  ULONG       m_lNumRegions;                    // Number of regions in this PolyBrep - must be at least 1
  SmVector3d  m_vColor;                         // Color of this PolyBrep

  // ObjectToObject relationships are stored as 
  //    indices in the first object's data where the index 
  //    specifies the 2nd object in the lists below.          
  SmTArray<SmPolyRegionData>   & m_vPolyRegions;   // flat list of all regions
  SmTArray<SmPolyShellData>    & m_vPolyShells;    // flat list of all Shells
  SmTArray<SmPolyFaceData>     & m_vPolyFaces;     // flat list of all Faces
  SmTArray<SmPolyLoopData>     & m_vPolyLoops;     // flat list of all Loops
  SmTArray<SmPolyEdgeData>     & m_vPolyEdges;     // flat list of all Edges
  SmTArray<SmPolyVertexData>   & m_vPolyVertices;  // flat list of all Vertices
  SmTArray<SmPolyVertAuxData*> & m_vAuxData;       // flat list of all Vertex AuxData
  SmTArray<ULONG>              & m_vAttributes;    // flat list of all attribute indices

  ULONG    m_lBrepIndex;                           // Index of brep in part's Data 

public:
  SmPolyBrepData();
  virtual ~SmPolyBrepData();

  SmStatus FromPolyBrep
  (
    const SmPolyBrep       & crPolyBrep,                    ///< [in ]: source PolyBrep for construction                             <br>
    SmTArray<SmAttribute*> & rAttributes,                   ///< [out]: accumulation of all attributes on all entities               <br>
    SmCopyPolyBrepMap      * pOptIndexObjectMap = NULL      ///< [out]: stores sequential index for every SmPoly Topology object     <br>
                                                            ///<      : NULL to ignore, default:[NULL]                               <br>
  );
    
  static SmStatus ReadFromDB(const SmContext        & crContext,         // in : 
                             ULONG                    lDBVersionNumber,  // NotUsed: in : 
                             SmTArray<SmAttribute*> & rAllAttributes,    // in :
                             SmPolyBrepData        *& rpNewPolyBrepData, // out:    
                             SmDatabaseIOFile       & rDB) ;             // in : 
    

  static SmStatus ReadFromFile
  (
    const SmContext & crContext,
    const TCHAR     * cInputFileName,        
    SmPolyBrepData *& rpNewPolyBrepData,
    SmFileType       eType = SM_ASCII
  );

  SmStatus WriteToDB(const SmContext        & crContext,         // NotUsed: in : 
                     ULONG                    lDBVersionNumber,  // in : 
                     SmTArray<SmAttribute*> & rAllAttributes,    // NotUsed: in : 
                     SmDatabaseIOFile       & rDB)               // in : 
                    const;

  SmStatus WriteToFile
  (
    const TCHAR * cOutputFileName,
    SmFileType    eType = SM_ASCII, 
    SmBoolean     bNewFile = TRUE
  ) const;

} ; // end class SmPolyBrepData

#endif // !__SMPOLYBREPDATA_H__
