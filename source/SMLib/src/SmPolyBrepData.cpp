// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPolyBrepData.cpp
* PURPOSE:
**********************************************************************/

#include "StdAfx.h"

#include <SmPolyBrepData.h>
#include <SmMapTypeToType.h>
#include <SmAttribute.h>
#include <SmPoly.h>

#include <iostream>

#define MAXSIZE  256

#define GOTO_NEXT_LINE  rFileIn.ignore(MAXSIZE,'\n')

/*******************************************************************//**
PURPOSE: Constructor for the SmPolyBrepData object

NOTES:
***********************************************************************/
SmPolyBrepData::SmPolyBrepData()
  : m_sZoneTol3d(0.0),
    m_lStartRegion(SM_UNDEF_ULONG),
    m_lNumRegions(SM_UNDEF_ULONG),
    m_vColor(0.0,0.0,0.0),
    m_vPolyRegions (*(new (*GetContext()) SmTArray<SmPolyRegionData>  (*GetContext()))),
    m_vPolyShells  (*(new (*GetContext()) SmTArray<SmPolyShellData>   (*GetContext()))),
    m_vPolyFaces   (*(new (*GetContext()) SmTArray<SmPolyFaceData>    (*GetContext()))),
    m_vPolyLoops   (*(new (*GetContext()) SmTArray<SmPolyLoopData>    (*GetContext()))),
    m_vPolyEdges   (*(new (*GetContext()) SmTArray<SmPolyEdgeData>    (*GetContext()))),
    m_vPolyVertices(*(new (*GetContext()) SmTArray<SmPolyVertexData>  (*GetContext()))),
    m_vAuxData     (*(new (*GetContext()) SmTArray<SmPolyVertAuxData*>(*GetContext()))),
    m_vAttributes  (*(new (*GetContext()) SmTArray<ULONG>             (*GetContext())))
 { }

/*******************************************************************//**
PURPOSE: Destructor for the SmPolyBrepData object

NOTES:
***********************************************************************/
SmPolyBrepData::~SmPolyBrepData()
{
  ULONG ii ;
  m_sZoneTol3d = 0.0 ;
  m_lStartRegion  = SM_UNDEF_ULONG ;
  m_lNumRegions   = SM_UNDEF_ULONG ;

  SM_ASSERT(&m_vPolyRegions  != NULL) ; delete &m_vPolyRegions ;
  SM_ASSERT(&m_vPolyShells   != NULL) ; delete &m_vPolyShells ;
  SM_ASSERT(&m_vPolyFaces    != NULL) ; delete &m_vPolyFaces ;
  SM_ASSERT(&m_vPolyLoops    != NULL) ; delete &m_vPolyLoops ;
  SM_ASSERT(&m_vPolyEdges    != NULL) ; delete &m_vPolyEdges ;
  SM_ASSERT(&m_vPolyVertices != NULL) ; delete &m_vPolyVertices ;
  SM_ASSERT(&m_vAuxData      != NULL) ; if(m_vAuxData.GetSize()>=0)
                                          { for(ii=0;ii<m_vAuxData.GetSize();ii++)
                                              { if(m_vAuxData[ii]) m_vAuxData[ii]->RemoveUser() ; }
                                            delete &m_vAuxData ;
                                          }
  SM_ASSERT(&m_vAttributes   != NULL) ; delete &m_vAttributes ;

} // end SmPolyBrepData destructor

#define INVALID_KEY 99999999

/*******************************************************************//**
PURPOSE: Fetch indices given SmPoly Topology object pointers

NOTES:
***********************************************************************/
static long sm_GetMap
 (SmMapTypeToType<SmObject*,ULONG> & sMap,
  SmObject                         * pObj)
{
  ULONG lRet = sMap.GetValueAt(pObj);
  if (lRet == 0)
    {
      // GWC: don't signal error here - SmPolyBrepData::FromPolyBrep() calls
      //      this just to see if something is in the list - when it's not
      //      we come down this path and that's ok.
      //        SE(SM_ERR);
      return INVALID_KEY;
    }
  return lRet - 1;

} // end sm_GetMap

/*******************************************************************//**
PURPOSE: manage Object to index mapping macros

NOTES:
***********************************************************************/
#define ADD_TO_MAP(pObj,lIndx) sMap.SetAt((pObj),((lIndx)+1))
#define GET_MAP(pObj)          sm_GetMap(sMap,(pObj))

/*******************************************************************//**
PURPOSE: Create SmPolyBrepData Directly from SmPolyBrep

NOTES:
***********************************************************************/
SmStatus SmPolyBrepData::FromPolyBrep
  (const SmPolyBrep       & crPolyBrep,         // in : source PolyBrep for construction
   SmTArray<SmAttribute*> & rAttributes,        // out: accumulation of all attributes on all entities
   SmCopyPolyBrepMap      * pOptIndexObjectMap) // out: stores sequential index for every SmPoly Topology object
                                                //      NULL to ignore, default:[NULL]
{
  ULONG ii;
  SmMapTypeToType<SmAttribute *,ULONG> sAttrMap; // map AttributePointers to indices
  SmMapTypeToType<SmObject*,ULONG>     sMap;     // map ObjectPointers to indices

  // Attributes
  m_sZoneTol3d = crPolyBrep.GetTolerance();
  m_vColor.Set(1.0,0.0,0.0); // why color = red?
  m_lStartRegion = 0;
  m_lBrepIndex   = INVALID_KEY;

  // Place all PolyBrep attributes in rAttributes array and save [ptr,index] mapping in sAttrMap
  const SmContext *pContext = crPolyBrep.GetContext();
  SER(sm_ExtractAttributes(*pContext,      // in : current context for object creation
                           &crPolyBrep,    // in : target object potentially containing attributes
                           m_vAttributes,  // out: rAttributes index array for this object's attributes, or NULL for none,
                                           //      expected NULL on input, owned by user, must be deleted by caller
                           rAttributes,    // i/o: accumulation of all attributes on all entities
                           sAttrMap));     // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs

  SmTArray<SmPolyShell*> sAllPolyShells;
  ULONG lTotalPolyShells = 0;

  // Get SmPolyRegions
  SmTArray<SmPolyRegion*> sPolyRegions;
  crPolyBrep.GetPolyRegions(sPolyRegions);
  m_lNumRegions = sPolyRegions.GetSize();

  // set PolyRegion count
  m_vPolyRegions.SetSize(m_lNumRegions);
  if(pOptIndexObjectMap) pOptIndexObjectMap->SetIndexSize(m_lNumRegions+1, SmPolyRegion_TYPE);

  // for every PolyRegion
  for (ii=0; ii<m_lNumRegions; ii++)
    {
      SmPolyRegion * pPolyRegion = sPolyRegions[ii];

      // PolyRegion To Index map
      if(pOptIndexObjectMap) pOptIndexObjectMap->SetAt(ii+1, pPolyRegion) ;

      // get this PolyRegion->PolyShells
      SmTArray<SmPolyShell*> sPolyShells;
      pPolyRegion->GetPolyShells(sPolyShells);
      ULONG lNumPolyShells = sPolyShells.GetSize();

      // store region data
      m_vPolyRegions[ii].m_lStartPolyShell = lTotalPolyShells;    // indexing
      m_vPolyRegions[ii].m_lNumPolyShells  = lNumPolyShells;

#ifdef SM_INDEXING
      m_vPolyRegions[ii].m_lUserIndex1 = pPolyRegion->GetUserIndex1();
      m_vPolyRegions[ii].m_lUserIndex2 = pPolyRegion->GetUserIndex2();
      m_vPolyRegions[ii].m_pUserPtr1   = pPolyRegion->GetUserPtr1();
#endif // SM_INDEXING

      // build all polyshell list
      lTotalPolyShells += lNumPolyShells;
      sAllPolyShells.Append(sPolyShells);

      // Place all PolyRegion attributes in rAttributes array and save [ptr,index] mapping in sAttrMap
      SER(sm_ExtractAttributes(*pContext,                         // in : current context for object creation
                                pPolyRegion,                      // in : target object potentially containing attributes
                                m_vPolyRegions[ii].m_sAttributes, // out: rAttributes index array for this object's attributes, or NULL for none,
                                                                  //      expected NULL on input, owned by user, must be deleted by caller
                                rAttributes,                      // i/o: accumulation of all attributes on all entities
                                sAttrMap));                       // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs
    } // end PolyRegions

  SmTArray<SmPolyFace*> sAllPolyFaces;
  ULONG lTotalPolyFaces = 0;

  // set PolyShell count
  m_vPolyShells.SetSize(lTotalPolyShells);
  if(pOptIndexObjectMap) pOptIndexObjectMap->SetIndexSize(lTotalPolyShells+1,SmPolyShell_TYPE);

  // for every PolyShell
  for (ii=0; ii<lTotalPolyShells; ii++)
    {
      SmPolyShell * pPolyShell = sAllPolyShells[ii];

      // PolyShell To Index map
      if(pOptIndexObjectMap) pOptIndexObjectMap->SetAt(ii+1, pPolyShell) ;

      // get this PolyShell->PolyFaces
      SmTArray<SmPolyFace*> sPolyFaces;
      pPolyShell->GetPolyFaces(sPolyFaces);
      ULONG lNumPolyFaces = sPolyFaces.GetSize();

      // store PolyShell data
      m_vPolyShells[ii].m_lStartPolyFace = lTotalPolyFaces;  // indexing
      m_vPolyShells[ii].m_lNumPolyFaces  = lNumPolyFaces;

#ifdef SM_INDEXING
      m_vPolyShells[ii].m_lUserIndex1 = pPolyShell->GetUserIndex1();
      m_vPolyShells[ii].m_lUserIndex2 = pPolyShell->GetUserIndex2();
      m_vPolyShells[ii].m_pUserPtr1   = pPolyShell->GetUserPtr1();
#endif // SM_INDEXING

      // build all polyface list
      lTotalPolyFaces += lNumPolyFaces;
      sAllPolyFaces.Append(sPolyFaces);

      // Place all PolyShell attributes in rAttributes array and save [ptr,index] mapping in sAttrMap
      SER(sm_ExtractAttributes(*pContext,                       // in : current context for object creation
                                pPolyShell,                     // in : target object potentially containing attributes
                                m_vPolyShells[ii].m_sAttributes,// out: rAttributes index array for this object's attributes, or NULL for none,
                                                                //      expected NULL on input, owned by user, must be deleted by caller
                                rAttributes,                    // i/o: accumulation of all attributes on all entities
                                sAttrMap));                     // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs
    } // end PolyShells

  SmTArray<SmPolyLoop*> sAllPolyLoops;
  ULONG lTotalPolyLoops = 0;

  // set PolyFace count
  m_vPolyFaces.SetSize(lTotalPolyFaces);
  if(pOptIndexObjectMap) pOptIndexObjectMap->SetIndexSize(lTotalPolyFaces+1,SmPolyFace_TYPE);

  // for every PolyFace
  for (ii=0; ii<lTotalPolyFaces; ii++)
    {
      SmPolyFace * pPolyFace = sAllPolyFaces[ii];

      // PolyFace To Index map
      if(pOptIndexObjectMap) pOptIndexObjectMap->SetAt(ii+1, pPolyFace) ;

      // get PolyFace->PolyLoops
      SmTArray<SmPolyLoop*> sPolyLoops;
      pPolyFace->GetPolyLoops(sPolyLoops);
      ULONG lNumPolyLoops = sPolyLoops.GetSize();

      // store PolyFace data
      m_vPolyFaces[ii].m_sZoneTol3d     = pPolyFace->GetTolerance();
      m_vPolyFaces[ii].m_lStartPolyLoop = lTotalPolyLoops;           // indexing
      m_vPolyFaces[ii].m_lNumPolyLoops  = lNumPolyLoops;
      m_vPolyFaces[ii].m_vNormal        = pPolyFace->GetNormal(FALSE, FALSE);
      m_vPolyFaces[ii].m_dMaxDeviation  = pPolyFace->GetMaxDeviation();
      m_vPolyFaces[ii].m_lFaceIndex     = INVALID_KEY;

#ifdef SM_INDEXING
      m_vPolyFaces[ii].m_lUserIndex1 = pPolyFace->GetUserIndex1();
      m_vPolyFaces[ii].m_lUserIndex2 = pPolyFace->GetUserIndex2();
      m_vPolyFaces[ii].m_pUserPtr1   = pPolyFace->GetUserPtr1();
#endif // SM_INDEXING

      // build all PolyLoop list
      lTotalPolyLoops += lNumPolyLoops;
      sAllPolyLoops.Append(sPolyLoops);

      // Place all PolyFace attributes in rAttributes array and save [ptr,index] mapping in sAttrMap
      SER(sm_ExtractAttributes(*pContext,                      // in : current context for object creation
                                pPolyFace,                     // in : target object potentially containing attributes
                                m_vPolyFaces[ii].m_sAttributes,// out: rAttributes index array for this object's attributes, or NULL for none,
                                                               //      expected NULL on input, owned by user, must be deleted by caller
                                rAttributes,                   // i/o: accumulation of all attributes on all entities
                                sAttrMap));                    // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs
    } // End PolyFaces

  SmTArray<SmPolyEdge*> sAllPolyEdges;
  ULONG lTotalPolyEdges = 0;

  // set PolyLoop count
  m_vPolyLoops.SetSize(lTotalPolyLoops);
  if(pOptIndexObjectMap) pOptIndexObjectMap->SetIndexSize(lTotalPolyLoops+1,SmPolyLoop_TYPE);

  // for every PolyLoop
  for (ii=0; ii<lTotalPolyLoops; ii++)
    {
      SmPolyLoop * pPolyLoop = sAllPolyLoops[ii];

      // PolyLoop To Index map
      if(pOptIndexObjectMap) pOptIndexObjectMap->SetAt(ii+1, pPolyLoop);

      // get PolyLoop->PolyEdges
      SmTArray<SmPolyEdge*> sPolyEdges;
      pPolyLoop->GetPolyEdges(sPolyEdges);
      ULONG lNumPolyEdges = sPolyEdges.GetSize();

      // store PolyLoop data
      SmBoolean bOuterLoop             = (pPolyLoop->GetOrientation() == SM_OT_OPPOSITE) ? FALSE : TRUE ;
      m_vPolyLoops[ii].m_bOuterLoop     = bOuterLoop;
      m_vPolyLoops[ii].m_bClosed        = TRUE;
      m_vPolyLoops[ii].m_lStartPolyEdge = lTotalPolyEdges;   // indexing
      m_vPolyLoops[ii].m_lNumPolyEdge   = lNumPolyEdges;

#ifdef SM_INDEXING
      m_vPolyLoops[ii].m_lUserIndex1 = pPolyLoop->GetUserIndex1();
      m_vPolyLoops[ii].m_lUserIndex2 = pPolyLoop->GetUserIndex2();
      m_vPolyLoops[ii].m_pUserPtr1   = pPolyLoop->GetUserPtr1();
#endif // SM_INDEXING

      // build all PolyEdge list
      lTotalPolyEdges += lNumPolyEdges;
      sAllPolyEdges.Append(sPolyEdges);

      // Place all PolyLoop attributes in rAttributes array and save [ptr,index] mapping in sAttrMap
      SER(sm_ExtractAttributes(*pContext,                       // in : current context for object creation
                                pPolyLoop,                      // in : target object potentially containing attributes
                                m_vPolyLoops[ii].m_sAttributes, // out: rAttributes index array for this object's attributes, or NULL for none,
                                                                //      expected NULL on input, owned by user, must be deleted by caller
                                rAttributes,                    // i/o: accumulation of all attributes on all entities
                                sAttrMap));                     // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs
    } // end PolyLoops

  // Get SmPolyVertices
  SmTArray<SmPolyVertex*> sAllVertices;
  crPolyBrep.GetPolyVertices(sAllVertices);
  ULONG lTotalVertices = sAllVertices.GetSize();

  // set PolyVertex count
  m_vPolyVertices.SetSize(lTotalVertices);
  if(pOptIndexObjectMap) pOptIndexObjectMap->SetIndexSize(lTotalVertices+1,SmPolyVertex_TYPE);

  // for every PolyVertex
  for (ii=0; ii<lTotalVertices; ii++)
    {
      SmPolyVertex * pPolyVertex = sAllVertices[ii];

      // PolyVertex To Index map
      ADD_TO_MAP(pPolyVertex,ii);
      if(pOptIndexObjectMap) pOptIndexObjectMap->SetAt(ii+1, pPolyVertex);

      // store PolyVertexData
      m_vPolyVertices[ii].m_sZoneTol3d = pPolyVertex->GetTolerance();
      m_vPolyVertices[ii].m_vPoint     = pPolyVertex->GetPoint();

#ifdef SM_INDEXING
      m_vPolyVertices[ii].m_lUserIndex1 = pPolyVertex->GetUserIndex1();
      m_vPolyVertices[ii].m_lUserIndex2 = pPolyVertex->GetUserIndex2();
      m_vPolyVertices[ii].m_pUserPtr1   = pPolyVertex->GetUserPtr1();
#endif // SM_INDEXING

      // Place all PolyVertex attributes in rAttributes array and save [ptr,index] mapping in sAttrMap
      SER(sm_ExtractAttributes(*pContext,                         // in : current context for object creation
                                pPolyVertex,                      // in : target object potentially containing attributes
                                m_vPolyVertices[ii].m_sAttributes,// out: rAttributes index array for this object's attributes, or NULL for none,
                                                                  //      expected NULL on input, owned by user, must be deleted by caller
                                rAttributes,                      // i/o: accumulation of all attributes on all entities
                                sAttrMap));                       // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs
    } // end PolyVertexes

  // set PolyEdge count
  m_vPolyEdges.SetSize(lTotalPolyEdges);
  if(pOptIndexObjectMap) pOptIndexObjectMap->SetIndexSize(lTotalPolyEdges+1,SmPolyEdge_TYPE);

  // prepare array to hold SmPolyEdge->SmPolyVertAuxData (a normal and a texture coordinate)
  for(ii=0;ii<m_vAuxData.GetSize();ii++) { if(m_vAuxData[ii]) m_vAuxData[ii]->RemoveUser() ; }
  m_vAuxData.ReSet();
  m_vAuxData.Add(NULL); // Skip first one to give spot for NULL

  // for every PolyEdge
  for (ii=0; ii<lTotalPolyEdges; ii++)
    {
      SmPolyEdge        * pPolyEdge = sAllPolyEdges[ii];
      SmPolyVertAuxData * pAuxData  = pPolyEdge->GetAuxData();

      // GWC: we should not discard information here, I've restored the pAuxData data
      //      pAuxData = NULL; // RCLxx was getting uninit value

      // build PolyEdge to index map
      ADD_TO_MAP(pPolyEdge,ii);
      if(pOptIndexObjectMap) pOptIndexObjectMap->SetAt(ii+1, pPolyEdge);

      // Build all AuxData list (watch out for multiple uses of a single AuxData)
      if (pAuxData)
        {
          ULONG lIndex = GET_MAP(pAuxData);
          if (lIndex == INVALID_KEY)
            {
              ADD_TO_MAP(pAuxData,m_vAuxData.GetSize());
              lIndex = m_vAuxData.GetSize();
              pAuxData->AddUser() ;
              m_vAuxData.Add(pAuxData);
            }
        }

      // store PolyEdge data
      SmPolyVertex * pVert = pPolyEdge->GetStartPolyVertex();
      m_vPolyEdges[ii].m_sZoneTol3d = pPolyEdge->GetTolerance();
      m_vPolyEdges[ii].m_lVertex    = GET_MAP(pVert);        // switch from pointer to index here
      m_vPolyEdges[ii].m_lNextRadialE = ii;

#ifdef SM_INDEXING
      m_vPolyEdges[ii].m_lUserIndex1     = pPolyEdge->GetUserIndex1();
      m_vPolyEdges[ii].m_lUserIndex2     = pPolyEdge->GetUserIndex2();
      m_vPolyEdges[ii].m_pUserPtr1       = pPolyEdge->GetUserPtr1();
#endif // SM_INDEXING

      // Place all PolyEdge attributes in rAttributes array and save [ptr,index] mapping in sAttrMap
      SER(sm_ExtractAttributes(*pContext,                       // in : current context for object creation
                                pPolyEdge,                      // in : target object potentially containing attributes
                                m_vPolyEdges[ii].m_sAttributes, // out: rAttributes index array for this object's attributes, or NULL for none,
                                                                //      expected NULL on input, owned by user, must be deleted by caller
                                rAttributes,                    // i/o: accumulation of all attributes on all entities
                                sAttrMap));                     // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs
    } // end PolyEdges

  // Find radial ordering of edges

  // for every PolyEdge
  for (ii=0; ii<lTotalPolyEdges; ii++)
    {
      SmPolyEdge * pPolyEdge = sAllPolyEdges[ii];

      // PolyEdge Locals
      SmPolyEdge * pNextEAtV = pPolyEdge->GetNextPolyEdgeAtV();
      SmPolyEdge * pRadialE  = pPolyEdge->GetRadial();

      // set NextRadialEdge index
      m_vPolyEdges[ii].m_lNextRadialE = (pRadialE && pRadialE != pPolyEdge)
                                      ? GET_MAP(pRadialE)
                                      : ii;

      // set NextEdgeAtV index
      m_vPolyEdges[ii].m_lNextEdgeAtV =  (pNextEAtV != pPolyEdge)
                                       ? GET_MAP(pNextEAtV)
                                       : ii;
    } // end saving radial ordering of PolyEdges

  // set PolyVertex back Edge indices now that all PolyEdges have an index
  for (ii=0; ii<lTotalVertices; ii++)
    {
      SmPolyVertex * pPolyVertex = sAllVertices[ii];
      SmPolyEdge   * pEdgeList   = pPolyVertex->GetFirstPolyEdge();

      // set PolyVertex EdgeList head index
      m_vPolyVertices[ii].m_lEdgeList = GET_MAP(pEdgeList);

    } // end seting all PolyVertex back PolyEdge index values

  // all done
  return SM_SUCCESS;

} // end SmPolyBrepData::FromPolyBrep

/*******************************************************************//**
PURPOSE: Read a single PolyBrep from a file of the existing file.

NOTES:
***********************************************************************/
SmStatus SmPolyBrepData::ReadFromFile
 (const SmContext  & crContext,          // in : context for new object construction
  const TCHAR      * cInputFileName,     // in : target file name
  SmPolyBrepData  *& rpNewPolyBrepData,  // out: object to contain read data
  SmFileType         eType)              // in : oneof: SM_ASCII    = read ascii file with comment lines
                                         //             SM_BINARY   = read binary file from littleEndian to system format
                                         //             SM_BYTESWAP = read binary file - forcing byte swap
                                         //      default:[SM_ASCII]
{
  SmDatabaseIOFile sDB;
  TCHAR        sBuff[SM_TBLOCK_SIZE] ;

  // open file for read
  if(SM_SUCCESS != sDB.OpenFileForRead(cInputFileName, eType))
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cInputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }

  // virtual PreRead for derived SmDatabaseIOFile classes - base class does nothing
  SER(sDB.BeginReading());

  // do the read
  SmTArray<SmAttribute*> sAllAttributes;
  SER(SmPolyBrepData::ReadFromDB(crContext,
                                 30,  // Use old version without attributes
                                 sAllAttributes,
                                 rpNewPolyBrepData,
                                 sDB));

  // virtual PostRead for derived SmDatabaseIOFile classes - base class does nothing
  SER(sDB.EndReading());

  return SM_SUCCESS;

} // end SmPolyBrepData::ReadFromFile

/*******************************************************************//**
PURPOSE: Write a single SmPolyBrep out to a file and append it to
            the end of the existing file.

NOTES:
***********************************************************************/
SmStatus SmPolyBrepData::WriteToFile
  (const TCHAR * cOutputFileName,    // in : target file name
   SmFileType    eType,              // in : oneof: SM_ASCII    = write ascii file with comment lines
                                     //             SM_BINARY   = write binary file from system formant to LittleEndian
                                     //             SM_BYTESWAP = write binary file - forcing byte swap
   SmBoolean     bNewFile)           // in : TRUE = open file and rewrite contents
 const                               //      FALSE= open file and append to end
{
  SmDatabaseIOFile sDB;
  TCHAR        sBuff[SM_TBLOCK_SIZE] ;

  // open file for wrtie
  if(SM_SUCCESS != sDB.OpenFileForWrite(cOutputFileName, // in : target file name
                                        eType,           // in : oneof: SM_ASCII    = write ascii file with comment lines
                                                         //             SM_BINARY   = write binary file from system format to LittleEndian
                                                         //             SM_BYTESWAP = write binary file - forcing byte swap
                                                         //      default:[SM_ASCII]
                                        bNewFile))       // in : bNewFile = TRUE  = open file and rewrite contents
                                                         //                 FALSE = open file and append to end
                                                         //                 default:[FALSE]
    {
      smos_sprintf(sBuff,_T("  *** ERROR File not found - %s\n"),cOutputFileName);
      smos_WriteBuffer(sBuff);
      return SM_ERR;
    }

  // virtual PreRead for derived SmDatabaseIOFile classes - base class does nothing
  SER(sDB.BeginWriting());

  // do the write
  const SmContext *cpContext = GetContext() ;  SM_ASSERT(cpContext != NULL) ;
  SmTArray<SmAttribute*> sAllAttributes;
  SER(WriteToDB(*cpContext,
                 30, // Use old version without attributes
                 sAllAttributes,
                 sDB));

  // virtual PostRead for derived SmDatabaseIOFile classes - base class does nothing
  SER(sDB.EndWriting());

  // all done
  return SM_SUCCESS;

} // end SmPolyBrepData::WriteToFile

/*******************************************************************//**
PURPOSE: Read the PolyBrepData from a stream (file) in either ASCII or
    binary format.

NOTES:
***********************************************************************/
SmStatus SmPolyBrepData::ReadFromDB
 (const SmContext        & crContext,          // in :
  ULONG                    lDBVersionNumber,   // in :
  SmTArray<SmAttribute*> & rAllAttributes,     // in :
  SmPolyBrepData        *& rpNewPolyBrepData,  // out:
  SmDatabaseIOFile       & rDB)                // in :
{
  ULONG lCount;
  ULONG i;

  SmPolyBrepData * pPolyBrepData = new(crContext) SmPolyBrepData();
  NER(pPolyBrepData);

  SmFileType eType = rDB.GetFileType();

  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();

      GOTO_NEXT_LINE; //PolyBrep Starts
      if (lDBVersionNumber > 30)
        {
          SmAttributeData::ReadIndexedAttributesFromDB(crContext,
                                                       rAllAttributes,
                                                       pPolyBrepData->m_vAttributes,
                                                       rDB);
        }
      else
        {
          rFileIn >> pPolyBrepData->m_vColor.x >> pPolyBrepData->m_vColor.y >> pPolyBrepData->m_vColor.z;
          GOTO_NEXT_LINE;
        }
      rFileIn >> pPolyBrepData->m_sZoneTol3d;
      GOTO_NEXT_LINE;
      rFileIn >> pPolyBrepData->m_lBrepIndex;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Region Data
      rFileIn >> pPolyBrepData->m_lNumRegions;
      GOTO_NEXT_LINE;
      rFileIn >> pPolyBrepData->m_lStartRegion;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, # Of PolyFaces, First PolyFace
    }
  else // Binary 
    {
      if (lDBVersionNumber > 30) 
        {
          SER(SmAttributeData::ReadIndexedAttributesFromDB(crContext,
                                                           rAllAttributes,
                                                           pPolyBrepData->m_vAttributes,
                                                           rDB));
        }
      else 
        {
          SER(rDB.ReadDoubles(&pPolyBrepData->m_vColor.x,3));
        }
      SER(rDB.ReadDouble(pPolyBrepData->m_sZoneTol3d));
      SER(rDB.ReadLong(pPolyBrepData->m_lNumRegions));
      SER(rDB.ReadLong(pPolyBrepData->m_lStartRegion));
    }

  // Region Data
  lCount = pPolyBrepData->m_lNumRegions;
  pPolyBrepData->m_vPolyRegions.SetSize(lCount);
  for (i=0; i<lCount; i++) {
      SER(pPolyBrepData->m_vPolyRegions[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB));
  }

  // Shell Data
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //PolyShell Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, # Of PolyFaces, First PolyFace
    }
  else // Binary 
    {
      SER(rDB.ReadLong(lCount));
    }
  pPolyBrepData->m_vPolyShells.SetSize(lCount);
  for (i=0; i<lCount; i++) 
    {
      SER(pPolyBrepData->m_vPolyShells[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB));
    }

  // PolyFace Data
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //PolyFace Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, Tolerance, # Of PolyLoops, First PolyLoop, Face Index
      GOTO_NEXT_LINE; //Normal-X, Normal-Y, Normal-Z, MaxDeviation
    }
  else // Binary 
    {
      SER(rDB.ReadLong(lCount));
    }
  pPolyBrepData->m_vPolyFaces.SetSize(lCount);
  for (i=0; i<lCount; i++) 
    {
      SER(pPolyBrepData->m_vPolyFaces[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB));
    }

  // PolyLoop Data
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //PolyLoop Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, # Of PolyEdges, Start PolyEdge, Orientation, Is Closed Loop
    }
  else // Binary 
    {
      SER(rDB.ReadLong(lCount));
    }
  pPolyBrepData->m_vPolyLoops.SetSize(lCount);
  for (i=0; i<lCount; i++) 
    {
      SER(pPolyBrepData->m_vPolyLoops[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB));
    }

  // PolyEdge Data
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //PolyEdge Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, Tolerance, Start PolyVertex, End PolyVertex, Next PolyEdge, Next Radial PolyEdge, Edgeuse Index, Vertexuse Index, User's Long, User's Double
    }
  else // Binary 
    {
      SER(rDB.ReadLong(lCount));
    }
  pPolyBrepData->m_vPolyEdges.SetSize(lCount);
  for (i=0; i<lCount; i++) 
    {
      SER(pPolyBrepData->m_vPolyEdges[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB));
    }

  // PolyVertex Data
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //PolyVertex Data
      rFileIn >> lCount;
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE; //Index, Tolerance, Next Coincident PolyVertex, Type, User's Long
      GOTO_NEXT_LINE; //X, Y, Z, Normal-X, Normal-Y, Normal-Z
    }
  else // Binary 
    {
      SER(rDB.ReadLong(lCount));
    }
  pPolyBrepData->m_vPolyVertices.SetSize(lCount);
  for (i=0; i<lCount; i++) 
    {
      SER(pPolyBrepData->m_vPolyVertices[i].ReadFromDB(crContext,lDBVersionNumber,rAllAttributes,rDB));
    }

  // Now read in auxillary data

  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE;
      rFileIn >> lCount; //Poly Vertex Auxillary Data In PolyBrep
      GOTO_NEXT_LINE;
      GOTO_NEXT_LINE;
    }
  else // Binary 
    {
      SER(rDB.ReadLong(lCount));
    }
  pPolyBrepData->m_vAuxData.Add(NULL); // First one is always NULL
  ULONG index;
  for (i=1; i<lCount; i++)
    {
      SmPolyVertAuxData *pAux = new (crContext) SmPolyVertAuxData();
      NER(pAux);
      pAux->AddUser() ;
      pPolyBrepData->m_vAuxData.Add(pAux);
      if (eType == SM_ASCII) 
        {
          std::istream & rFileIn = *rDB.GetInStreamPtr();
          rFileIn >> index >> pAux->m_vNormal.x >> pAux->m_vNormal.y >>
              pAux->m_vNormal.z >> pAux->m_vUV.x >> pAux->m_vUV.y;
          GOTO_NEXT_LINE;
        }
      else 
        {
          SER(rDB.ReadDoubles(&pAux->m_vNormal.x,3));
          SER(rDB.ReadDoubles(&pAux->m_vUV.x,2));
        }
    }

  rpNewPolyBrepData = pPolyBrepData;

  return SM_SUCCESS;

} // end SmPolyBrepData::ReadFromDB

/*******************************************************************//**
PURPOSE: Write the PolyBrep Data out to a stream or file.

NOTES:
***********************************************************************/
SmStatus SmPolyBrepData::WriteToDB
 (const SmContext        & crContext,          // NotUsed: in :
  ULONG                    lDBVersionNumber,   // in :
  SmTArray<SmAttribute*> & rAllAttributes,     // NotUsed: in :
  SmDatabaseIOFile       & rDB)                // in :
 const
{
  SM_REF2(crContext, rAllAttributes) ;
  SmFileType eType = rDB.GetFileType();

  ULONG i;
  ULONG lCount;

  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();

      rFileOut << "//PolyBrep Starts\n";
      if (lDBVersionNumber > 30)
        {
          SmAttributeData::WriteIndexedAttributesToDB(m_vAttributes,rDB);
        }
      else
        {
          rFileOut << m_vColor.x << " " << m_vColor.y << " " << m_vColor.z << " - Color\n";
        }
      rFileOut << m_sZoneTol3d << " - PolyBrep ZoneTol3d\n";
      rFileOut << m_lBrepIndex << " - Brep Index\n";
      rFileOut << "\n//Region Data\n";
      rFileOut << m_lNumRegions << " PolyRegions In PolyBrep\n";
      rFileOut << m_lStartRegion << " - Start PolyRegion\n";
      rFileOut << "//Index, # Of PolyShells, First PolyShell\n";
    }
  else //Binary
    {
      if (lDBVersionNumber > 30)
        {
          SmAttributeData::WriteIndexedAttributesToDB(m_vAttributes,rDB);
        }
      else
        {
          SER(rDB.WriteDoubles(&m_vColor.x,3));
        }
      SER(rDB.WriteDouble(m_sZoneTol3d));
      SER(rDB.WriteLong(m_lNumRegions));
      SER(rDB.WriteLong(m_lStartRegion));
    }

  // PolyRegion Data
  for (i=0; i<m_lNumRegions; i++) 
    {
      if (eType == SM_ASCII) 
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;// Region Index
        }
      m_vPolyRegions[i].WriteToDB(lDBVersionNumber,rDB);
    }

  // PolyShell Data
  lCount = m_vPolyShells.GetSize();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//PolyShell Data\n";
      rFileOut << lCount << " PolyShells In PolyBrep\n";
      rFileOut << "//Index, # Of PolyFaces, First PolyFace\n";
    }
  else //Binary
    {
      SER(rDB.WriteLong(lCount));
    }
  for (i=0; i<lCount; i++) 
    {
      if (eType == SM_ASCII) 
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;// Shell Index
        }
      m_vPolyShells[i].WriteToDB(lDBVersionNumber,rDB);
    }

  // PolyFace Data
  lCount = m_vPolyFaces.GetSize();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//PolyFace Data\n";
      rFileOut << lCount << " PolyFaces In PolyBrep\n";
      rFileOut << "//Index, Tolerance, # Of PolyLoops, First PolyLoop, Face Index\n";
      rFileOut << "//Normal-X, Normal-Y, Normal-Z, MaxDeviation\n";
    }
  else // Binary 
    {
      SER(rDB.WriteLong(lCount));
    }
  for (i=0; i<lCount; i++) 
    {
      if (eType == SM_ASCII) 
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vPolyFaces[i].WriteToDB(lDBVersionNumber,rDB);
    }

  // PolyLoop Data
  lCount = m_vPolyLoops.GetSize();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//PolyLoop Data\n";
      rFileOut << lCount << " PolyLoops In PolyBrep\n";
      rFileOut << "//Index, # Of PolyEdges, Start PolyEdge, Orientation, Is Closed Loop\n";
    }
  else // Binary 
    {
      SER(rDB.WriteLong(lCount));
    }
  for (i=0; i<lCount; i++) 
    {
      if (eType == SM_ASCII) 
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vPolyLoops[i].WriteToDB(lDBVersionNumber,rDB);
    }

  // PolyEdge Data
  lCount = m_vPolyEdges.GetSize();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//PolyEdge Data\n";
      rFileOut << lCount << " PolyEdges In PolyBrep\n";
      rFileOut << "//Index, Tolerance, Start PolyVertex, Next Radial PolyEdge, Next Edge Of Vertex, Aux Data\n";
    }
  else // Binary 
    {
      SER(rDB.WriteLong(lCount));
    }
  for (i=0; i<lCount; i++) 
    {
      if (eType == SM_ASCII)
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vPolyEdges[i].WriteToDB(lDBVersionNumber,rDB);
    }

  // PolyVertex Data
  lCount = m_vPolyVertices.GetSize();
  if (eType == SM_ASCII) {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//PolyVertex Data\n";
      rFileOut << lCount << " PolyVertices In PolyBrep\n";
      rFileOut << "//Index, Tolerance, Edge List, Point\n";
      rFileOut << "//X, Y, Z\n";
    }
  else // Binary 
    {
      SER(rDB.WriteLong(lCount));
    }
  for (i=0; i<lCount; i++) 
    {
      if (eType == SM_ASCII) 
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i;//Index
        }
      m_vPolyVertices[i].WriteToDB(lDBVersionNumber,rDB);
    }

  lCount = m_vAuxData.GetSize();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << "\n//PolyVertex Aux Data\n";
      rFileOut << lCount << " Poly Vertex Auxillary Data In PolyBrep\n";
      rFileOut << "//Index, Normal:X, Y, Z, Surface:U, V\n";
    }
  else // Binary 
    {
      SER(rDB.WriteLong(lCount));
    }
  for (i=1; i<lCount; i++) 
    {
      SmPolyVertAuxData *pAux = m_vAuxData[i];
      if (eType == SM_ASCII) 
        {
          std::ostream & rFileOut = *rDB.GetOutStreamPtr();
          rFileOut << i << " " << pAux->m_vNormal.x << " " <<
              pAux->m_vNormal.y << " " << pAux->m_vNormal.z << " "
              << pAux->m_vUV.x << " " << pAux->m_vUV.y << "\n";
        }
      else 
        {
          SER(rDB.WriteDoubles(&pAux->m_vNormal.x,3));
          SER(rDB.WriteDoubles(&pAux->m_vUV.x,2));
        }
    }

  return SM_SUCCESS;

} // end SmPolyBrepData::WriteToDB

// SmPolyRegionData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyRegionData::ReadFromDB
 (const SmContext        & crContext,         // in : 
  ULONG                    lDBVersionNumber,  // NotUsed: in : 
  SmTArray<SmAttribute*> & rAllAttributes,    // in : 
  SmDatabaseIOFile       & rDB)               // in : 
{
  SM_REF1(lDBVersionNumber) ;
  ULONG index;
  SmFileType eType = rDB.GetFileType();

  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index >> m_lNumPolyShells >> m_lStartPolyShell;
      GOTO_NEXT_LINE;
    }
  else // Binary 
    {
      SER(rDB.ReadLong(m_lNumPolyShells));
      SER(rDB.ReadLong(m_lStartPolyShell));
    }
  SER(ReadIndexedAttributesFromDB(crContext,
                                  rAllAttributes,
                                  m_sAttributes,
                                  rDB));

  return SM_SUCCESS;

} // end SmPolyRegionData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyRegionData::WriteToDB
 (ULONG              lDBVersionNumber,  // NotUsed: in : 
  SmDatabaseIOFile & rDB)               // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << " " << m_lNumPolyShells << " " << m_lStartPolyShell << "\n";
    }
  else // Binary 
    {
      SER(rDB.WriteLong(m_lNumPolyShells));
      SER(rDB.WriteLong(m_lStartPolyShell));
    }
  SER(WriteIndexedAttributesToDB(m_sAttributes,rDB));

  return SM_SUCCESS;

} // end SmPolyRegionData::WriteToDB

// SmPolyShellData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyShellData::ReadFromDB
 (const SmContext        & crContext,         // in : 
  ULONG                    lDBVersionNumber,  // NotUsed: in : 
  SmTArray<SmAttribute*> & rAllAttributes,    // in : 
  SmDatabaseIOFile       & rDB)               // in : 
{
  SM_REF1(lDBVersionNumber) ;
  ULONG index;
  SmFileType eType = rDB.GetFileType();

  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index >> m_lNumPolyFaces >> m_lStartPolyFace;
      GOTO_NEXT_LINE;
    }
  else // Binary 
    {
      SER(rDB.ReadLong(m_lNumPolyFaces));
      SER(rDB.ReadLong(m_lStartPolyFace));
    }
  SER(ReadIndexedAttributesFromDB(crContext,
                                  rAllAttributes,
                                  m_sAttributes,
                                  rDB));

  return SM_SUCCESS;

} // end SmPolyShellData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyShellData::WriteToDB
 (ULONG              lDBVersionNumber,  // NotUsed: in :
  SmDatabaseIOFile & rDB)               // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << " " << m_lNumPolyFaces << " " << m_lStartPolyFace << "\n";
    }
  else // Binary 
    {
      SER(rDB.WriteLong(m_lNumPolyFaces));
      SER(rDB.WriteLong(m_lStartPolyFace));
    }
  SER(WriteIndexedAttributesToDB(m_sAttributes,rDB));

  return SM_SUCCESS;

} // end SmPolyShellData::WriteToDB

// SmPolyFaceData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyFaceData::ReadFromDB
 (const SmContext        & crContext,        // in : 
  ULONG                    lDBVersionNumber, // NotUsed: in :
  SmTArray<SmAttribute*> & rAllAttributes,   // in : 
  SmDatabaseIOFile       & rDB)              // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  ULONG index;
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index >> m_sZoneTol3d >> m_lNumPolyLoops >> m_lStartPolyLoop >> m_lFaceIndex;
      GOTO_NEXT_LINE;
      rFileIn >> m_vNormal.x >> m_vNormal.y >> m_vNormal.z >> m_dMaxDeviation;
      GOTO_NEXT_LINE;
    }
  else // Binary 
    {
      SER(rDB.ReadDouble(m_sZoneTol3d));
      SER(rDB.ReadLong(m_lNumPolyLoops));
      SER(rDB.ReadLong(m_lStartPolyLoop));
      SER(rDB.ReadLong(m_lFaceIndex));
      SER(rDB.ReadDouble(m_vNormal.x));
      SER(rDB.ReadDouble(m_vNormal.y));
      SER(rDB.ReadDouble(m_vNormal.z));
      SER(rDB.ReadDouble(m_dMaxDeviation));
    }
  if (m_vNormal.Length() < SM_EFF_ZERO) 
    {
      MSG(_T("Degenerate polygon found"));
      SE(SM_ERR);
    }
  SER(ReadIndexedAttributesFromDB(crContext,
                                  rAllAttributes,
                                  m_sAttributes,
                                  rDB));

  return SM_SUCCESS;

} // end SmPolyFaceData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyFaceData::WriteToDB
 (ULONG              lDBVersionNumber,   // NotUsed: in :
  SmDatabaseIOFile & rDB)                // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << " " << m_sZoneTol3d << " " << m_lNumPolyLoops
          << " " << m_lStartPolyLoop << " " << m_lFaceIndex << "\n";
      rFileOut << m_vNormal.x << " " << m_vNormal.y << " "
          << m_vNormal.z << " " << m_dMaxDeviation << "\n";
    }
  else // Binary 
    {
      SER(rDB.WriteDouble(m_sZoneTol3d));
      SER(rDB.WriteLong(m_lNumPolyLoops));
      SER(rDB.WriteLong(m_lStartPolyLoop));
      SER(rDB.WriteLong(m_lFaceIndex));
      SER(rDB.WriteDouble(m_vNormal.x));
      SER(rDB.WriteDouble(m_vNormal.y));
      SER(rDB.WriteDouble(m_vNormal.z));
      SER(rDB.WriteDouble(m_dMaxDeviation));
    }
  SER(WriteIndexedAttributesToDB(m_sAttributes,rDB));

  return SM_SUCCESS;

} // end SmPolyFaceData::WriteToDB

// SmPolyLoopData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyLoopData::ReadFromDB
 (const SmContext        & crContext,         // in : 
  ULONG                    lDBVersionNumber,  // NotUsed: in :
  SmTArray<SmAttribute*> & rAllAttributes,    // in : 
  SmDatabaseIOFile       & rDB)               // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  ULONG index;
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index >> m_lNumPolyEdge >> m_lStartPolyEdge >>
          m_bOuterLoop >> m_bClosed;
      GOTO_NEXT_LINE;
    }
  else // Binary 
    {
      SER(rDB.ReadLong(m_lNumPolyEdge));
      SER(rDB.ReadLong(m_lStartPolyEdge));
      SER(rDB.ReadBoolean(m_bOuterLoop));
      SER(rDB.ReadBoolean(m_bClosed));
    }
  SER(ReadIndexedAttributesFromDB(crContext,
                                  rAllAttributes,
                                  m_sAttributes,
                                  rDB));

  return SM_SUCCESS;

} // end SmPolyLoopData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyLoopData::WriteToDB
 (ULONG              lDBVersionNumber,   // NotUsed: in :
  SmDatabaseIOFile & rDB)                // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << " " << m_lNumPolyEdge << " " << m_lStartPolyEdge <<
          " " << m_bOuterLoop << " " << m_bClosed << "\n";
    }
  else // Binary 
    {
      SER(rDB.WriteLong(m_lNumPolyEdge));
      SER(rDB.WriteLong(m_lStartPolyEdge));
      SER(rDB.WriteBoolean(m_bOuterLoop));
      SER(rDB.WriteBoolean(m_bClosed));
    }

  SER(WriteIndexedAttributesToDB(m_sAttributes,rDB));

  return SM_SUCCESS;

} // end SmPolyLoopData::WriteToDB

// SmPolyEdgeData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyEdgeData::ReadFromDB
 (const SmContext        & crContext,         // in : 
  ULONG                    lDBVersionNumber,  // NotUsed: in :
  SmTArray<SmAttribute*> & rAllAttributes,    // in : 
  SmDatabaseIOFile       & rDB)               // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  ULONG index;
  if (eType == SM_ASCII) 
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index >> m_sZoneTol3d >> m_lVertex >> m_lNextRadialE >>
          m_lNextEdgeAtV >> m_lAuxData;
      GOTO_NEXT_LINE;
    }
  else // Binary 
    {
      SER(rDB.ReadDouble(m_sZoneTol3d));
      SER(rDB.ReadLong(m_lVertex));
      SER(rDB.ReadLong(m_lNextRadialE));
      SER(rDB.ReadLong(m_lNextEdgeAtV));
      SER(rDB.ReadLong(m_lAuxData));
    }
  SER(ReadIndexedAttributesFromDB(crContext,
                                  rAllAttributes,
                                  m_sAttributes,
                                  rDB));

  return SM_SUCCESS;

} // end SmPolyEdgeData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyEdgeData::WriteToDB
 (ULONG              lDBVersionNumber,   // NotUsed: in :
  SmDatabaseIOFile & rDB)                // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << " " << m_sZoneTol3d << " " << m_lVertex << " "
          << m_lNextRadialE << " " << m_lNextEdgeAtV <<
          " " << m_lAuxData <<
          " " << "\n";
    }
  else // Binary 
    {
      SER(rDB.WriteDouble(m_sZoneTol3d));
      SER(rDB.WriteLong(m_lVertex));
      SER(rDB.WriteLong(m_lNextRadialE));
      SER(rDB.WriteLong(m_lNextEdgeAtV));
      SER(rDB.WriteLong(m_lAuxData));
    }
  SER(WriteIndexedAttributesToDB(m_sAttributes,rDB));

  return SM_SUCCESS;

} // end SmPolyEdgeData::WriteToDB

// SmPolyVertexData Methods
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyVertexData::ReadFromDB
 (const SmContext        & crContext,         // in : 
  ULONG                    lDBVersionNumber,  // NotUsed: in :
  SmTArray<SmAttribute*> & rAllAttributes,    // in : 
  SmDatabaseIOFile       & rDB)               // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  ULONG index;
  if (eType == SM_ASCII)
    {
      std::istream & rFileIn = *rDB.GetInStreamPtr();
      rFileIn >> index >> m_sZoneTol3d >> m_lEdgeList;
      GOTO_NEXT_LINE;
      rFileIn >> m_vPoint.x >> m_vPoint.y >> m_vPoint.z;
      GOTO_NEXT_LINE;
    }
  else //Binary
    {
      SER(rDB.ReadDouble(m_sZoneTol3d));
      SER(rDB.ReadLong(m_lEdgeList));
      SER(rDB.ReadDouble(m_vPoint.x));
      SER(rDB.ReadDouble(m_vPoint.y));
      SER(rDB.ReadDouble(m_vPoint.z));
    }
  SER(ReadIndexedAttributesFromDB(crContext,
                                  rAllAttributes,
                                  m_sAttributes,
                                  rDB));

  return SM_SUCCESS;

} // end SmPolyVertexData::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmPolyVertexData::WriteToDB
 (ULONG              lDBVersionNumber,   // NotUsed: in :
  SmDatabaseIOFile & rDB)                // in : 
{
  SM_REF1(lDBVersionNumber) ;
  SmFileType eType = rDB.GetFileType();
  if (eType == SM_ASCII) 
    {
      std::ostream & rFileOut = *rDB.GetOutStreamPtr();
      rFileOut << " " << m_sZoneTol3d << " " << m_lEdgeList << "\n";
      rFileOut << m_vPoint.x << " " << m_vPoint.y << " " << m_vPoint.z << "\n";
    }
  else // Binary 
    {
      SER(rDB.WriteDouble(m_sZoneTol3d));
      SER(rDB.WriteLong(m_lEdgeList));
      SER(rDB.WriteDouble(m_vPoint.x));
      SER(rDB.WriteDouble(m_vPoint.y));
      SER(rDB.WriteDouble(m_vPoint.z));
    }
  SER(WriteIndexedAttributesToDB(m_sAttributes,rDB));

  return SM_SUCCESS;

} // end SmPolyVertexData::WriteToDB

#undef ADD_TO_MAP
#undef GET_MAP
#undef MAXSIZE
#undef INVALID_KEY
