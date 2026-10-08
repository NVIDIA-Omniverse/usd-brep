// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmGfxVertexArray.cpp
* PURPOSE: Source for the SmGfxVertexArray class methods. 
**********************************************************************/

#include "StdAfx.h"

#include <SmGraphicsVertexArray.h>

#ifdef SM_GFX_OUTPUT_CODE

#endif // SM_GFX_OUTPUT_CODE

/*******************************************************************//**
PURPOSE:  convenience method to Add SmGfxVertex

NOTES: 
***********************************************************************/
ULONG SmGfxVertexArray::Add
 (float fX,
  float fY,
  float fZ)
{
  SM_ASSERT(m_pVertices != NULL) ;
  SmGfxVertex sGfxVertex(fX, fY, fZ) ;
  return(m_pVertices->Add(sGfxVertex)) ; 

} // end SmGfxVertexArray::Add  SmGfxVertex

/*******************************************************************//**
PURPOSE:  convenience method to Add SmGfxNVertex

NOTES: 
***********************************************************************/
ULONG SmGfxVertexArray::Add
 (float fX,
  float fY,
  float fZ,
  float fNX,
  float fNY,
  float fNZ)
{
  SM_ASSERT(m_pNVertices != NULL) ;
  SmGfxNVertex sGfxNVertex(fX, fY, fZ, fNX, fNY, fNZ) ;
  return(m_pNVertices->Add(sGfxNVertex)) ; 

} // end SmGfxVertexArray::Add  SmGfxNVertex

/*******************************************************************//**
PURPOSE:  convenience method to Add SmGfxColoredVertex

NOTES: 
***********************************************************************/
ULONG SmGfxVertexArray::Add
 (float fX,
  float fY,
  float fZ,
  float fNX,
  float fNY,
  float fNZ,
  float fR,
  float fG,
  float fB)
{
  SM_ASSERT(m_pVertices != NULL) ;
  SmGfxColoredVertex sGfxColoredVertex(fX, fY, fZ, fNX, fNY, fNZ, fR, fG, fB) ;
  return(m_pColoredVertices->Add(sGfxColoredVertex)) ; 

} // end SmGfxVertexArray::Add  SmGfxColoredVertex

/*******************************************************************//**
PURPOSE:  convenience method to Add SmGfxTexturedVertex

NOTES: 
***********************************************************************/
ULONG SmGfxVertexArray::Add
 (float fX,
  float fY,
  float fZ,
  float fNX,
  float fNY,
  float fNZ,
  float fU,
  float fV)
{
  SM_ASSERT(m_pVertices != NULL) ;
  SmGfxTexturedVertex sGfxTexturedVertex(fX, fY, fZ, fNX, fNY, fNZ, fU, fV) ;
  return(m_pTexturedVertices->Add(sGfxTexturedVertex)) ; 

} // end SmGfxVertexArray::Add  SmGfxTexturedVertex

/*******************************************************************//**
PURPOSE:  Add a polygon.

NOTES: Adds points of type SM_GV_PN
   to VertexArrays of type SM_GV_SIMPLE_MESH or SM_GV_INDEX_MESH.
   Not complete yet, add cases as we go.

  gwc: currently okay for m_eGfxVertexDrawType type == SM_GV_SIMPLE_MESH,
       but clearly incomplete for type SM_GV_INDEX_MESH
***********************************************************************/
SmStatus SmGfxVertexArray::LoadPolygon
 (ULONG        lPolygonType,    // NotUsed: in :
  ULONG        lNumPoints,      // in :
  SmPoint3d  * aPolygonPoints,  // in :
  SmVector3d * aPolygonNormals, // in :
  SmSurface  * pSurface,        // NotUsed: in :
  SmFace     * pFace)           // NotUsed: in :
{
  SM_REF3(lPolygonType, pSurface, pFace) ;
  ULONG ii;
  SM_ASSERT_MSG(   m_eGfxVertexDrawType == SM_GV_SIMPLE_MESH
                || m_eGfxVertexDrawType == SM_GV_INDEX_MESH,
                _T("SmGfxVertexArray::LoadPolygon has unexpected m_eGfxVertexDrawType type")) ;
  // temp Assert until method is completed for INDEX_MESH
  SM_ASSERT_MSG( m_eGfxVertexDrawType != SM_GV_INDEX_MESH,
                _T("SmGfxVertexArray::LoadPolygon not yet complete for case SM_GV_INDEX_MESH")) ;
 
  // when drawing a SM_GV_INDEX_MESH 
  if ( m_eGfxVertexDrawType == SM_GV_INDEX_MESH )
    {
      // add entry for number of Points from this call
      ULONG lLast = ( m_pIndices->GetSize() == 0 ) ? 0 : m_pIndices->GetLast();
      m_pIndices->Add( lLast + lNumPoints );
    }

  // for every point
  for ( ii=0; ii<lNumPoints; ii++ )
    {
      // add one new SmGfxNVertex obj to m_pNVertices array
      Add( (float)aPolygonPoints[ii].x,  (float)aPolygonPoints[ii].y, (float)aPolygonPoints[ii].z,
           (float)aPolygonNormals[ii].x, (float)aPolygonNormals[ii].y, (float)aPolygonNormals[ii].z );
    }
  return SM_SUCCESS;

} // end SmGfxVertexArray::LoadPolygon

/*******************************************************************//**
PURPOSE:  Add a polygon.

NOTES: Adds points of type SM_GV_TEXTURED.
***********************************************************************/
SmStatus SmGfxVertexArray::LoadPolygon
 (ULONG        lPolygonType,     // NotUsed: in : 
  ULONG        lNumPoints,       // in : 
  SmPoint3d  * aPolygonPoints,   // in : 
  SmVector3d * aPolygonNormals,  // in : 
  SmPoint2d  * aPolygonUVPoints, // in : 
  SmSurface  * pSurface,         // NotUsed: in : 
  SmFace     * pFace)            // NotUsed: in : 
{
  SM_REF3(lPolygonType, pSurface, pFace) ;
  ULONG ii;

  // for every point
  for ( ii=0; ii<lNumPoints; ii++ )
    {
      // add one new SmGfxTexturedVertex obj to m_pTexturedVertices array
      Add( (float)aPolygonPoints[ii].x,   (float)aPolygonPoints[ii].y,   (float)aPolygonPoints[ii].z,
           (float)aPolygonNormals[ii].x,  (float)aPolygonNormals [ii].y, (float)aPolygonNormals[ii].z,
           (float)aPolygonUVPoints[ii].x, (float)aPolygonUVPoints[ii].y );
    }
  return SM_SUCCESS;

} // end SmGfxVertexArray::LoadPolygon

/*******************************************************************//**
PURPOSE:  Send the contents of this object to the display.

NOTES: 
***********************************************************************/
SmStatus SmGfxVertexArray::Display( SmPolygonOutputCallback & rPolyOutput )
{
  SmGfxVertexDrawType eDrawType = this->GetVertexDrawType();
  SmGfxVertexDataType eDataType = this->GetVertexDataType();

#ifdef SM_DEBUG_CODE
  static constexpr int bDebugMe = FALSE;
  if ( bDebugMe ) {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      if ( eDrawType == SM_GV_INDEX_MESH ) {
          ULONG lNumPolys = m_pIndices->GetSize();
          smos_sprintf(sBuff, _T("   VertexArray Displaying: %p, %ld Polys\n"), this, lNumPolys);
          smos_WriteBuffer(sBuff);
      }
  }
#endif

  // Not ready to be displayed.  Not an error though.
  if ( eDrawType == SM_GV_UNKNOWN )
    { return SM_SUCCESS; }

  SmVector3d sSaveColor = smgfx_OutputColor( this->GetColor() );
  double dSaveLW = smgfx_OutputLineWidth( this->GetLineWidth() );
  double dSavePS = smgfx_OutputPointSize( this->GetPointSize() );

  // Need to switch on Draw Type and Data Type.
  //cbi For first pass, Draw type is outer, Data type is inner.
  //cbi As we do more, see what can be consolidated, maybe switch inner/outer.

  ULONG jj, kk;
  SmStatus eStat, eRetStat = SM_SUCCESS;

  switch ( eDrawType )
  {
    case SM_GV_UNKNOWN: {
      // Can't happen but helps to keep certain compilers quiet.
      break;
    }
    case SM_GV_POINT: {

      switch ( eDataType ) {
        case SM_GV_P:
        case SM_GV_PN:
        case SM_GV_COLORED:
        case SM_GV_TEXTURED:
          { break; }
      } // end switch on eDataType

      break;

    } // end case SM_GV_POINT

    case SM_GV_LINE: {

      switch ( eDataType ) {
        case SM_GV_P:
        {
//        smgfx_OutputPoint( (*paVtxArray)[jj].m_fX, (*paVtxArray)[jj].m_fY, (*paVtxArray)[jj].m_fZ );
          SmTArray<SmGfxVertex> * pPtsArrPtr = this->GetVertexArray();  NER( pPtsArrPtr );
          ULONG lNumPts = pPtsArrPtr->GetSize();
          SmTArray< SmPoint3d > sPtArray;
          sPtArray.SetSize( lNumPts );
          for ( jj=0; jj<lNumPts; jj++ )
            {
              sPtArray[jj].Set( (*pPtsArrPtr)[jj].m_fX, (*pPtsArrPtr)[jj].m_fY, (*pPtsArrPtr)[jj].m_fZ );
            }

          SmPoint3d *pPts = sPtArray.GetDataArray();
          smgfx_OutputPolyline( &pPts[0].x, lNumPts, 3, NULL );

          break;

        } // end case SM_GV_P

        case SM_GV_PN:
        case SM_GV_COLORED:
        case SM_GV_TEXTURED:
          { break; }
      } // end switch on eDataType

      break;

    } // end case SM_GV_LINE

    case SM_GV_SIMPLE_MESH: {
        switch ( eDataType) {
          case SM_GV_P:
            { break; }

          case SM_GV_PN:
          {
            SmTArray<SmGfxNVertex> * pPtsNorms = this->GetNVertexArray();  NER( pPtsNorms );

            ULONG lNumPoints = pPtsNorms->GetSize();

            // We currently can have up to 4 sides in a polygon.
            if ( lNumPoints > 4 )
              {
                SM_ASSERT_MSG( FALSE, _T("Tessellation: PolyFace with more than six edges."));
                lNumPoints = 4;
              }

            SmPoint3d  sPts  [4];
            SmVector3d sNorms[4];

            ULONG lType = ( lNumPoints == 4 ) ? 1 : 0;

            for (kk=0; kk<lNumPoints; kk++)
              {
                sPts  [kk].Set( (*pPtsNorms)[kk].m_fX,  (*pPtsNorms)[kk].m_fY,  (*pPtsNorms)[kk].m_fZ  );
                sNorms[kk].Set( (*pPtsNorms)[kk].m_fNX, (*pPtsNorms)[kk].m_fNY, (*pPtsNorms)[kk].m_fNZ );

              } // end for each triangle

            // Output a triangle
            eStat = rPolyOutput.OutputPolygon( lType, lNumPoints, sPts, sNorms, NULL, NULL, NULL );

            if ( eStat != SM_SUCCESS )
              { eRetStat = eStat; }

            break;

          } // end case SM_GV_PN

          case SM_GV_COLORED:
          case SM_GV_TEXTURED:
          default:
            { break; }

        } // end switch on eDataType

      break;

    } // end case SM_GV_SIMPLE_MESH

    case SM_GV_INDEX_MESH: {

      SM_ASSERT_BREAK( m_pIndices != NULL );
      ULONG lNumPolys = m_pIndices->GetSize();
      ULONG lStart, lNext, lNumPoints;

      SmTArray<SmGfxVertex > * pPts      = this->GetVertexArray();
      SmTArray<SmGfxNVertex> * pPtsNorms = this->GetNVertexArray();

      // We currently can have up to 4 sides in a polygon.
      SmPoint3d  sPts  [4];
      SmVector3d sNorms[4];

      lStart = lNext = 0;
      for ( jj = 0; jj<lNumPolys; jj++ )
        {
          lStart = lNext;
          lNext  = (*m_pIndices)[jj];
          lNumPoints = lNext - lStart;
          ULONG lType = ( lNumPoints == 4 ) ? 1 : 0;

          if ( lNumPoints > 4 )
            {
              SM_ASSERT_MSG( FALSE, _T("Tessellation: PolyFace with more than four edges."));
              lNumPoints = 4;
            }

          for ( kk=0; kk<lNumPoints; kk++ )
            {
              switch ( eDataType ) {
                case SM_GV_P:
                    sPts  [kk].Set( (*pPts)[lStart+kk].m_fX,  (*pPts)[kk].m_fY,  (*pPts)[kk].m_fZ  );
                    break;
                case SM_GV_PN:
                  {
                    sPts  [kk].Set( (*pPtsNorms)[lStart+kk].m_fX,  (*pPtsNorms)[lStart+kk].m_fY,  (*pPtsNorms)[lStart+kk].m_fZ  );
                    sNorms[kk].Set( (*pPtsNorms)[lStart+kk].m_fNX, (*pPtsNorms)[lStart+kk].m_fNY, (*pPtsNorms)[lStart+kk].m_fNZ );
                    break;
                  }
                case SM_GV_COLORED:
                case SM_GV_TEXTURED:
                  { break; }
                } // end switch on eDataType

            } // end for each point in polygon

          // Output a polygon.
          switch ( eDataType ) {
            case SM_GV_P:
              {
                eStat = rPolyOutput.OutputPolygon( lType, lNumPoints, sPts, NULL, NULL, NULL, NULL );
                if ( eStat != SM_SUCCESS )
                  { eRetStat = eStat; }
                break;
              }
            case SM_GV_PN:
              {
                eStat = rPolyOutput.OutputPolygon( lType, lNumPoints, sPts, sNorms, NULL, NULL, NULL );
                if ( eStat != SM_SUCCESS )
                  { eRetStat = eStat; }
                break;
              }
            case SM_GV_COLORED:
            case SM_GV_TEXTURED:
              { break; }

            } // end switch on eDataType

        } // end for all polygons

      break;

    } // end case SM_GV_INDEX_MESH

  } // end switch on eDrawType

  smgfx_OutputColor( sSaveColor );
  smgfx_OutputLineWidth( dSaveLW );
  smgfx_OutputPointSize( dSavePS );

  return eRetStat;

} // end SmGfxVertexArray::Display

/*******************************************************************//**
PURPOSE:  Set a data element.

NOTES: Currently no checks are made on the index, other than what is in
   the SmTArray operator[].
***********************************************************************/
void SmGfxArraySet::SetAt( ULONG lIndex, SmGfxVertexArray *pVtxArray )
{
  m_sArraySet[ lIndex ] = pVtxArray;
} // end SmGfxArraySet::SetAt

/*******************************************************************//**
PURPOSE:  Create a SmGfxVertexArray with all its draw state values
          copied from this SmGfxArraySet member values and
          add this new object to this object's set of Array object.

NOTES: This should be the default manner in which low level draw functions
  create SmGfxVertexArray objects making it simple for high level draw 
  functions to pass the required draw state values.
***********************************************************************/
SmGfxVertexArray * SmGfxArraySet::NewGfxVertexArray
 (SmGfxVertexDataType eGfxVertexDataType, // in : oneof SM_GV_PN,      // Vertex data includes (x,y,z, nx,ny,nz)       
                                          //            SM_GV_COLORED, // Vertex data includes (x,y,z, nx,ny,nz, r,g,b)
                                          //            SM_GV_TEXTURED // Vertex data includes (x,y,z, nx,ny,nz, u,v)  
  SmGfxVertexDrawType eGfxVertexDrawType, // in : oneof SM_GV_POINT,     
                                          //            SM_GV_LINE,       
                                          //            SM_GV_SIMPLE_MESH,
                                          //            SM_GV_INDEX_MESH  
  SmBoolean           bAddToArray)        // in, opt: TRUE: add to end of array
                                          //          FALSE: do not add to array at all
                                          //          Default: TRUE
{
  // allocate the new VertexArray
  const SmContext *pCtxt = m_sArraySet.GetContext();
  SmGfxVertexArray *pGfxVertexArray = new SmGfxVertexArray(pCtxt,
                                                           eGfxVertexDataType,
                                                           eGfxVertexDrawType,
                                                           m_fRed,
                                                           m_fGreen,
                                                           m_fBlue,
                                                           m_fPointSize,
                                                           m_fLineWidth,
                                                           FALSE) ;
  // set the stipple
  pGfxVertexArray->SetStipple(m_iStipple) ;

  // add the VertexArray to the ArraySet
  if ( bAddToArray )
    { m_sArraySet.Add(pGfxVertexArray) ; }

  // all done
  return( pGfxVertexArray ) ;

} // end SmGfxArraySet::NewGfxVertexArray 

/*******************************************************************//**
PURPOSE:  Send the contents of this object to the display.

NOTES: 
***********************************************************************/
SmStatus SmGfxArraySet::Display( SmPolygonOutputCallback & rPolyOutput )
{
  SmStatus eRetStat = SM_SUCCESS;
  SmStatus eThisStat;

  SmVector3d sSaveColor = smgfx_OutputColor( this->GetColor(),  NULL );
  double dSaveLW = smgfx_OutputLineWidth( this->GetLineWidth(), NULL );
  double dSavePS = smgfx_OutputPointSize( this->GetPointSize(), NULL );

  ULONG ii, lNumArrays = this->GetSize();

#ifdef SM_DEBUG_CODE
  static constexpr int bDebugMe = FALSE;
  if ( bDebugMe ) {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      ULONG lNumVAs = m_sArraySet.GetSize();
      smos_sprintf(sBuff, _T(" ArraySet Displaying: %p, %ld VertexArrays\n"), this, lNumVAs);
      smos_WriteBuffer(sBuff);
  }
#endif

  for ( ii=0; ii<lNumArrays; ii++ )
    {
      eThisStat = this->GetAt(ii)->Display( rPolyOutput );
      if ( eThisStat != SM_SUCCESS )
        { eRetStat = eThisStat; }
    }

  smgfx_OutputColor(  sSaveColor );
  smgfx_OutputLineWidth( dSaveLW );
  smgfx_OutputPointSize( dSavePS );

  return eRetStat;

} // end SmGfxArraySet::Display
