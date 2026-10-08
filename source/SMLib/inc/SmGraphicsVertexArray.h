// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmGraphicsVertexArray.h
* PURPOSE: Header file for SmGfxVertexArray class.    
**********************************************************************/

#ifndef __SmGfxVertexArray_H__
#define __SmGfxVertexArray_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#include <SmGraphicsExtern.h>

#ifndef __SMGFX_OUTPUT_H__
#include <SmGraphicsOutput.h>
#endif

#ifndef __SMCONTEXT_H__
#include <SmContext.h>
#endif

/*******************************************************************//**
PURPOSE: specify the data stored for SmGfxVertexArray vertices
***********************************************************************/
enum SmGfxVertexDataType {
    SM_GV_P,             // Vertex data includes (x,y,z)
    SM_GV_PN,            // Vertex data includes (x,y,z, nx,ny,nz)
    SM_GV_COLORED,       // Vertex data includes (x,y,z, nx,ny,nz, r,g,b)
    SM_GV_TEXTURED       // Vertex data includes (x,y,z, nx,ny,nz, u,v)
} ;

/*******************************************************************//**
PURPOSE: specify the display intent for SmGfxVertexArray vertices
***********************************************************************/
enum SmGfxVertexDrawType {
    SM_GV_UNKNOWN,       // Object type not yet specified
    SM_GV_POINT,         // Vertices to be displayed as points
    SM_GV_LINE,          // Vertices to be displayed as a PolyLine
    SM_GV_SIMPLE_MESH,   // Vertices form a simple mesh tri(0,1,2), tri(3,4,5) ...
    SM_GV_INDEX_MESH     // Vertices plus an index array form an indexed mesh
} ;

/*******************************************************************//**
PURPOSE: Position Graphical Vertex Array

NOTES: This class contains the xyz vector values
       for one graphical vertex.

       Never add a virtual function to this classes.  It is
       meant to be tightly packed in arrays and a virtual function table will
       interfere with that.
***********************************************************************/
class SM_EXPORT SmGfxVertex
{
  public:
    float m_fX ;      // x of vertex xyz position
    float m_fY ;      // y of vertex xyz position
    float m_fZ ;      // z of vertex xyz position

 SmGfxVertex(float fX, float fY, float fZ)    : m_fX(fX),
                                                m_fY(fY),
                                                m_fZ(fZ)
                                              { }
} ; // end class SmGfxVertex  

/*******************************************************************//**
PURPOSE: Position and Surfance Normal Graphical Vertex Array

NOTES: This class contains the xyz and normal vector values
       for one graphical vertex.

       Never add a virtual function to this classes.  It is
       meant to be tightly packed in arrays and a virtual function table will
       interfere with that.
***********************************************************************/
class SM_EXPORT SmGfxNVertex
{
  public:
    float m_fX ;      // x of vertex xyz position
    float m_fY ;      // y of vertex xyz position
    float m_fZ ;      // z of vertex xyz position

    float m_fNX ;     // x component of vertex Surface Normal Vector
    float m_fNY ;     // y component of vertex Surface Normal Vector
    float m_fNZ ;     // z component of vertex Surface Normal Vector

 SmGfxNVertex(float fX,  float fY,  float fZ,
              float fNX, float fNY, float fNZ)  : m_fX(fX),
                                                  m_fY(fY),
                                                  m_fZ(fZ),
                                                  m_fNX(fNX),
                                                  m_fNY(fNY),
                                                  m_fNZ(fNZ)
                                                { }

} ; // end class SmGfxNVertex  

/*******************************************************************//**
PURPOSE: Position, Surfance Normal, and color Graphical Vertex Array

NOTES: This class contains the xyz, normal vector, and color values
       for one graphical vertex.

       Never add a virtual function to this classes.  It is
       meant to be tightly packed in arrays and a virtual function table will
       interfere with that.
***********************************************************************/
class SM_EXPORT SmGfxColoredVertex
{
  public:
    float m_fX ;      // x of vertex xyz position
    float m_fY ;      // y of vertex xyz position
    float m_fZ ;      // z of vertex xyz position

    float m_fNX ;     // x component of vertex Surface Normal Vector
    float m_fNY ;     // y component of vertex Surface Normal Vector
    float m_fNZ ;     // z component of vertex Surface Normal Vector

    float m_fR ;      // red   of rgb color
    float m_fG ;      // green of rgb color
    float m_fB ;      // blue  of rgb color

 SmGfxColoredVertex(float fX,  float fY,  float fZ,
                    float fNX, float fNY, float fNZ,
                    float fR,  float fG,  float fB)  : m_fX(fX),
                                                       m_fY(fY),
                                                       m_fZ(fZ),
                                                       m_fNX(fNX),
                                                       m_fNY(fNY),
                                                       m_fNZ(fNZ),
                                                       m_fR(fR),
                                                       m_fG(fG),
                                                       m_fB(fB)
                                                     { }

} ; // end class SmGfxColoredVertex  

/*******************************************************************//**
PURPOSE: Position, Surfance Normal, and Texture Graphical Vertex Array

NOTES: This class contains the xyz, normal vector, and texture values
       for one graphical vertex.

       Never add a virtual function to this classes.  It is
       meant to be tightly packed in arrays and a virtual function table will
       interfere with that.
***********************************************************************/
class SM_EXPORT SmGfxTexturedVertex
{
  public:
    float m_fX ;      // x of vertex xyz position
    float m_fY ;      // y of vertex xyz position
    float m_fZ ;      // z of vertex xyz position

    float m_fNX ;     // x component of vertex Surface Normal Vector
    float m_fNY ;     // y component of vertex Surface Normal Vector
    float m_fNZ ;     // z component of vertex Surface Normal Vector

    float m_fU ;      // u of uv texture coordinate
    float m_fV ;      // v of uv texture coordinate

 SmGfxTexturedVertex(float fX,  float fY,  float fZ,
                     float fNX, float fNY, float fNZ,
                     float fU,  float fV)             : m_fX(fX),
                                                        m_fY(fY),
                                                        m_fZ(fZ),
                                                        m_fNX(fNX),
                                                        m_fNY(fNY),
                                                        m_fNZ(fNZ),
                                                        m_fU(fU),
                                                        m_fV(fV)
                                                      { }

} ; // end class SmGfxColoredVertex  

/*******************************************************************//**
PURPOSE: This object represents an array of vertices suitable
         for rendering

NOTES: This class was designed for use with OpenGL ES 2.0
  Draw functions will generate lists of SmGfxVertexArray objects
  which can then be passed to an OpenGL interface that will
  use these arrays to create VBO (Vertex Buffer Objects) suitable for 
  rendering.
***********************************************************************/
class SM_EXPORT SmGfxVertexArray
{
protected:
  // Graphic state
  SmGfxVertexDataType    m_eGfxVertexDataType ;  // specify the data stored with each vertex
                                                 // oneof: SM_GV_P,
                                                 //        SM_GV_PN,     
                                                 //        SM_GV_COLORED,
                                                 //        SM_GV_TEXTURED
  SmGfxVertexDrawType    m_eGfxVertexDrawType ;  // specify how to display contained vertices
                                                 // oneof: SM_GV_UNKNOWN,    
                                                 //        SM_GV_POINT,     
                                                 //        SM_GV_LINE,       
                                                 //        SM_GV_SIMPLE_MESH,
                                                 //        SM_GV_INDEX_MESH  

  float                  m_fRed ;                // red   value of RGB color, range:[0, 1]
  float                  m_fGreen ;              // green value of RGB color, range:[0, 1]
  float                  m_fBlue ;               // blue  value of RGB color, range:[0, 1]
  float                  m_fPointSize ;          // pixel size of point icons, range:[Greater than 1]
  float                  m_fLineWidth ;          // pixel width of anitaliased lines, range:[Greater than 1]
  int                    m_iStipple ;            // Line Stipple pattern, 0xFFFF = solid line, 0x00FF = dashed line
  // vertex data
  SmTArray <SmGfxVertex>         *m_pVertices ;          // contains variable length array of SmGfxVertex objects
  SmTArray <SmGfxNVertex>        *m_pNVertices ;         // contains variable length array of SmGfxNVertex objects
  SmTArray <SmGfxColoredVertex>  *m_pColoredVertices ;   // contains variable length array of SmGfxColoredVertex objects
  SmTArray <SmGfxTexturedVertex> *m_pTexturedVertices ;  // contains variable length array of SmGfxTexturedVertex objects
  SmTArray <ULONG>               *m_pIndices ;           // contains variable length triangle index array,
                                                         // only used when m_eGfxVertexDrawType == SM_GV_INDEX_MESH 
                                                         // use:  Tri0 = [m_pNVertices[m_pIndices[0]], m_pNVertices[m_pIndices[1]], m_pNVertices[m_pIndices[2]] ]
                                                         //       Tri1 = [m_pNVertices[m_pIndices[3]], m_pNVertices[m_pIndices[4]], m_pNVertices[m_pIndices[5]] ] 
                                                         //       etc.
public:
  // constructor -
  SmGfxVertexArray(const SmContext   * pCtxt,              // Context for SmTArray creation
                   SmGfxVertexDataType eGfxVertexDataType, // in : oneof SM_GV_PN,      // Vertex data includes (x,y,z, nx,ny,nz)       
                                                           //            SM_GV_COLORED, // Vertex data includes (x,y,z, nx,ny,nz, r,g,b)
                                                           //            SM_GV_TEXTURED // Vertex data includes (x,y,z, nx,ny,nz, u,v)  
                   SmGfxVertexDrawType eGfxVertexDrawType, // in : oneof SM_GV_POINT,     
                                                           //            SM_GV_LINE,       
                                                           //            SM_GV_SIMPLE_MESH,
                                                           //            SM_GV_INDEX_MESH  
                    float fRed,                            // in : red   of rgb color, used for entire array  
                    float fGreen,                          // in : green of rgb color,    when m_eGfxVertexDataType != SM_GV_COLORED
                    float fBlue,                           // in : blue  of rgb color,
                    float fPointSize=2,                    // in : size of point icons in pixels, used only for SM_GV_POINT                                                                
                    float fLineWidth=1,                    // in : width of lines in pixels, used only for SM_GV_LINE
                    SmBoolean bDashedLines = FALSE)        // in : TRUE = draw lines dashed, FALSE=don't, used only for SM_GV_LINE
                  : m_eGfxVertexDataType(eGfxVertexDataType), 
                    m_eGfxVertexDrawType(eGfxVertexDrawType), 
                    m_fRed  (fRed  ),             
                    m_fGreen(fGreen),             
                    m_fBlue (fBlue ),            
                    m_fPointSize(fPointSize),         
                    m_fLineWidth(fLineWidth),         
                    m_iStipple(bDashedLines ? 0x00ff : 0xffff),
                    m_pVertices(NULL),
                    m_pNVertices(NULL),
                    m_pColoredVertices(NULL),
                    m_pTexturedVertices(NULL),
                    m_pIndices(NULL) 
{
  // allocate Vertex array
  if     ( eGfxVertexDataType == SM_GV_P       ) { m_pVertices         = new(*pCtxt) SmTArray<SmGfxVertex>(*pCtxt) ; }
  else if( eGfxVertexDataType == SM_GV_PN      ) { m_pNVertices        = new(*pCtxt) SmTArray<SmGfxNVertex>(*pCtxt) ; }
  else if( eGfxVertexDataType == SM_GV_COLORED ) { m_pColoredVertices  = new(*pCtxt) SmTArray<SmGfxColoredVertex>(*pCtxt) ; }
  else if( eGfxVertexDataType == SM_GV_TEXTURED) { m_pTexturedVertices = new(*pCtxt) SmTArray<SmGfxTexturedVertex>(*pCtxt) ; }

  // when asked allocated index array
  if(eGfxVertexDrawType == SM_GV_INDEX_MESH)
    { m_pIndices = new (*pCtxt) SmTArray<ULONG>(*pCtxt) ; }

} // end SmGfxVertexArray constructor

  // destructor
 ~SmGfxVertexArray() { if(m_pVertices        ) { m_pVertices->ReSet() ;         delete m_pVertices ;         m_pVertices = NULL ; } 
                       if(m_pNVertices       ) { m_pNVertices->ReSet() ;        delete m_pNVertices ;        m_pNVertices = NULL ; } 
                       if(m_pColoredVertices ) { m_pColoredVertices->ReSet() ;  delete m_pColoredVertices ;  m_pColoredVertices  = NULL ; } 
                       if(m_pTexturedVertices) { m_pTexturedVertices->ReSet() ; delete m_pTexturedVertices ; m_pTexturedVertices = NULL ; } 
                       if(m_pIndices         ) { m_pIndices->ReSet() ;          delete m_pIndices ;          m_pIndices  = NULL ; }
                     } // end SmGfxVertexArray destructor

  // vertex array access
  SmTArray<SmGfxVertex>         * GetVertexArray()          { return( m_pVertices ) ; }
  SmTArray<SmGfxNVertex>        * GetNVertexArray()         { return( m_pNVertices ) ; }
  SmTArray<SmGfxColoredVertex>  * GetColoredVertexArray()   { return( m_pColoredVertices) ; }
  SmTArray<SmGfxTexturedVertex> * GetTexturedVertexAarray() { return( m_pTexturedVertices) ; }
  SmTArray<ULONG>               * GetIndexArray()           { return( m_pIndices ) ; }
                                                                                             
  float                         * GetVertexArrayData()      { return(   m_eGfxVertexDataType == SM_GV_P        ? (float *)m_pVertices->GetDataArray()
                                                                      : m_eGfxVertexDataType == SM_GV_PN       ? (float *)m_pNVertices->GetDataArray()
                                                                      : m_eGfxVertexDataType == SM_GV_COLORED  ? (float *)m_pColoredVertices->GetDataArray()
                                                                      : m_eGfxVertexDataType == SM_GV_TEXTURED ? (float *)m_pTexturedVertices->GetDataArray()
                                                                      : NULL ) ;
                                                            }
  // number of vertices in SmGfxVertexArray                                                             
  ULONG                           GetVertexCount()          { return (  m_eGfxVertexDataType == SM_GV_P        ? m_pVertices->GetSize()
                                                                      : m_eGfxVertexDataType == SM_GV_PN       ? m_pNVertices->GetSize()
                                                                      : m_eGfxVertexDataType == SM_GV_COLORED  ? m_pColoredVertices->GetSize()
                                                                      : m_eGfxVertexDataType == SM_GV_TEXTURED ? m_pTexturedVertices->GetSize()
                                                                      : 0) ;
                                                            }
  // size in bytes of a SmGfxVertexArray stored vertex element 
  ULONG                           GetVertexElementSize()    { return (  m_eGfxVertexDataType == SM_GV_P        ? sizeof(SmGfxVertex)
                                                                      : m_eGfxVertexDataType == SM_GV_PN       ? sizeof(SmGfxNVertex)
                                                                      : m_eGfxVertexDataType == SM_GV_COLORED  ? sizeof(SmGfxColoredVertex)
                                                                      : m_eGfxVertexDataType == SM_GV_TEXTURED ? sizeof(SmGfxTexturedVertex)
                                                                      : 0 ) ; 
                                                            }
  ULONG                         * GetIndexArrayData()       { return( m_pIndices ? (ULONG *)m_pIndices->GetDataArray()  : NULL ) ; }
  
  // Add one vertex to vertex list
  ULONG Add(float fX, float fY, float fZ) ;                            
  ULONG Add(float fX, float fY, float fZ, float fNX, float fNY, float fNZ) ;                            
  ULONG Add(float fX, float fY, float fZ, float fNX, float fNY, float fNZ, float m_fR, float m_fG, float m_fB) ;                            
  ULONG Add(float fX, float fY, float fZ, float fNX, float fNY, float fNZ, float m_fU, float m_fV) ; 

  // Type SM_GV_PN:
  SmStatus LoadPolygon(ULONG        lPolygonType,     // NotUsed: in : 
                       ULONG        lNumPoints,       // in : 
                       SmPoint3d  * aPolygonPoints,   // in : 
                       SmVector3d * aPolygonNormals,  // in : 
                       SmSurface  * pSurface,         // NotUsed: in : 
                       SmFace     * pFace);           // NotUsed: in : 

  // Type SM_GV_TEXTURED:
  SmStatus LoadPolygon( ULONG        lPolygonType,      // NotUsed: in :
                        ULONG        lNumPoints,        // in :
                        SmPoint3d  * aPolygonPoints,    // in :
                        SmVector3d * aPolygonNormals,   // in :
                        SmPoint2d  * aPolygonUVPoints,  // in :
                        SmSurface  * pSurface,          // NotUsed: in :
                        SmFace     * pFace);            // NotUsed: in :
                             
  // simple data access
  SmGfxVertexDataType GetVertexDataType() { return(m_eGfxVertexDataType) ; } 
  SmGfxVertexDrawType GetVertexDrawType() { return(m_eGfxVertexDrawType) ; }

  SmVector3d GetColor() { return SmVector3d(m_fRed, m_fGreen, m_fBlue) ; }

  float GetRed()       { return ( m_fRed ) ; }
  float GetGreen()     { return ( m_fGreen ) ; }
  float GetBlue()      { return ( m_fBlue ) ; }

  float GetPointSize() { return ( m_fPointSize ) ; }
  float GetLineWidth() { return ( m_fLineWidth ) ; }
  int   GetStipple()   { return ( m_iStipple ) ; }  
  
  // simple data set
  void SetVertexDrawType(SmGfxVertexDrawType eGfxVertexDrawType) { m_eGfxVertexDrawType = eGfxVertexDrawType ; }

  void SetRed      (float fRed  )                                { m_fRed   = fRed ; }
  void SetGreen    (float fGreen)                                { m_fGreen = fGreen ; }
  void SetBlue     (float fBlue )                                { m_fBlue  = fBlue ; }
  SmVector3d SetColor(const SmVector3d &rColor)                  { SmVector3d rtn(m_fRed, m_fGreen, m_fBlue) ;
                                                                   m_fRed   = (float)rColor.x ;
                                                                   m_fGreen = (float)rColor.y ;
                                                                   m_fBlue  = (float)rColor.z ;
                                                                   return rtn ;
                                                                 }
                                                                 
  void SetPointSize(float fPointSize)                            { m_fPointSize = fPointSize ; }
  void SetLineWidth(float fLineWidth)                            { m_fLineWidth = fLineWidth ; }
  void SetStipple  (int   iStipple  )                            { m_iStipple   = iStipple   ; }  

  void SetDashedLines(SmBoolean bStipple)                        { if(bStipple) { m_iStipple = 0x00FF ; } 
                                                                   else         { m_iStipple = 0xFFFF ; }
                                                                 }
  // predicates
  SmBoolean IsPVertex()         { return ( m_eGfxVertexDataType == SM_GV_P        ) ; }
  SmBoolean IsPNVertex()        { return ( m_eGfxVertexDataType == SM_GV_PN       ) ; }
  SmBoolean IsColoredVertex()   { return ( m_eGfxVertexDataType == SM_GV_COLORED  ) ; }
  SmBoolean SmTexturedVertex()  { return ( m_eGfxVertexDataType == SM_GV_TEXTURED ) ; }
  SmBoolean IsDashed()          { return ( m_iStipple == 0xffff ) ; }

  // Output to display
  SmStatus Display( SmPolygonOutputCallback & rPolyOutput );

} ; // end class SmGfxVertexArray

/*******************************************************************//**
PURPOSE: This object contains a set of SmGfxVertexArray objects
  and temporary draw state data used at the time each new
  SmGfxVertexArray object is created.

NOTES: This class was designed for use with OpenGL ES 2.0
  and SMLib's set of smgfx_DrawXxx() functions.
     smgfx_DrawPoint()          smgfx_DrawTriangle()
     smgfx_DrawHotPoint()       smgfx_DrawCircularArc()
     smgfx_DrawLine()           smgfx_DrawPlaneGrid()
     smgfx_DrawPolyline()       smgfx_DrawDiscGrid()
     smgfx_DrawComb()    
     smgfx_DrawPlane()   
     
   The smgfx_DrawXxxx() functions create an SmGfxVertexArray object with 
   a copy of the SmGfxArraySet of draw state member values and an 
   approriate set of vertex values when passed a pointer to a 
   SmGfxArraySet object.  That new SmGfxVertexArray object is loaded into 
   the set of arrays being collected by the SmGfxArraySet.

  INTENDED USE: Draw methods using the smgfx_DrawXxxx() functions, will 
    set the SmGfxArraySet draw state values as desired and then make
    smgfx_DrawXxxx() calls as:
      pGfxArraySet->SetLook(1,0,0, 2, 4) ;
      igfx_DrawLine(dX1, dY1, dZ1, dX2, dY2, dZ2, pGfxArraySet) ;  
  
  Object member Draw() functions will execute a sequence of
  smgfx_DrawXxx() calls accumulating a set of SmGfxVertexArray objects
  within its input SmGfxArraySet object.  
  When the Draw function is complete its updated SmGfxArraySet object
  can then be passed to an OpenGL interface that will use its SmGfxVerteArray
  objects to create VBO (Vertex Buffer Objects) suitable for rendering.
***********************************************************************/
class SM_EXPORT SmGfxArraySet
{
protected:
  // Set of SmGfxVertexArray objects
  SmTArray<SmGfxVertexArray *> m_sArraySet ;    // set of SmGfxVertexArray Objects to be drawn as VBOs

  // Graphic state - these values are copied into each SmGfxVertexArray when those are created.
  float                  m_fRed ;                // red   value of RGB color, range:[0, 1]
  float                  m_fGreen ;              // green value of RGB color, range:[0, 1]
  float                  m_fBlue ;               // blue  value of RGB color, range:[0, 1]
  float                  m_fPointSize ;          // pixel size of point icons, range:[Greater than 1]
  float                  m_fLineWidth ;          // pixel width of anitaliased lines, range:[Greater than 1]
  int                    m_iStipple ;            // Line Stipple pattern, 0xFFFF = solid line, 0x00FF = dashed line
public:
  // constructor -
  SmGfxArraySet(float fRed,                            // in : red   of rgb color, used for entire array  
                float fGreen,                          // in : green of rgb color,    when m_eGfxVertexDataType != SM_GV_COLORED
                float fBlue,                           // in : blue  of rgb color,
                float fPointSize=2,                    // in : size of point icons in pixels, used only for SM_GV_POINT                                                                
                float fLineWidth=1,                    // in : width of lines in pixels, used only for SM_GV_LINE
                SmBoolean bDashedLines = FALSE)        // in : TRUE = draw lines dashed, FALSE=don't, used only for SM_GV_LINE
              : m_fRed  (fRed  ),             
                m_fGreen(fGreen),             
                m_fBlue (fBlue ),            
                m_fPointSize(fPointSize),         
                m_fLineWidth(fLineWidth),         
                m_iStipple(bDashedLines ? 0x00ff : 0xffff)
              {
                // We keep our own context.
                SmContext *pCtxt = new SmContext();
                m_sArraySet.SetContext( pCtxt );
              }

  // destructor
 ~SmGfxArraySet()
   {
     m_sArraySet.RemoveAll() ;
     const SmContext *pCtxt = m_sArraySet.GetContext() ;
     if(pCtxt) { delete pCtxt ; pCtxt = NULL ; }
     m_sArraySet.SetContext( pCtxt ) ;
   }

  // Array Set management
  
  // create and load a new SmGfxVertexArray object whose DrawState values are copied from this object's values.
  SmGfxVertexArray * NewGfxVertexArray                       // eff: Create and add a new SmGfxVertexArray to the ArraySet
      (SmGfxVertexDataType eGfxVertexDataType,               // in : oneof SM_GV_P,       // Vertex data includes (x,y,z)       
                                                             //            SM_GV_PN,      // Vertex data includes (x,y,z, nx,ny,nz)       
                                                             //            SM_GV_COLORED, // Vertex data includes (x,y,z, nx,ny,nz, r,g,b)
                                                             //            SM_GV_TEXTURED // Vertex data includes (x,y,z, nx,ny,nz, u,v)  
       SmGfxVertexDrawType eGfxVertexDrawType,               // in : oneof SM_GV_POINT,     
                                                             //            SM_GV_LINE,       
                                                             //            SM_GV_SIMPLE_MESH,
                                                             //            SM_GV_INDEX_MESH  
       SmBoolean           bAddToArray=TRUE);                // in, opt: TRUE: add to end of array
                                                             //          FALSE: do not add to array at all
                                                             //          Default: TRUE

  // Add/Remove elements from ArraySet list - note: for elements made with CreateGfxVertexArray() do not also use Add().
  ULONG Add       (SmGfxVertexArray * pGfxVertexArray)       { return( m_sArraySet.Add(pGfxVertexArray) ) ; }
  void  RemoveAt  (ULONG nIndex, ULONG nCount=1)             { m_sArraySet.RemoveAt(nIndex, nCount) ; }
  void  RemoveLast()                                         { m_sArraySet.RemoveLast() ; }

  // simple GfxVertexArray access
  SmTArray< SmGfxVertexArray * > & GetArraySet()                { return( m_sArraySet ); }
  ULONG                            GetSize()                    { return( m_sArraySet.GetSize() ); }
        SmGfxVertexArray         * GetAt      (ULONG idx) const { return( m_sArraySet.GetAt(idx) ); }
        SmGfxVertexArray         * operator[] (ULONG idx) const { return( m_sArraySet.GetAt(idx) ); }
  void SetAt( ULONG lIndex, SmGfxVertexArray * pVtxArray );


  ULONG SetSize( ULONG lNewSize )        { return m_sArraySet.SetSize( lNewSize ); }

  // simple graphics state access
  SmVector3d GetColor()      { return SmVector3d(m_fRed, m_fGreen, m_fBlue) ; }
  float      GetRed()        { return ( m_fRed ) ; }
  float      GetGreen()      { return ( m_fGreen ) ; }
  float      GetBlue()       { return ( m_fBlue ) ; }
                             
  float      GetPointSize()  { return ( m_fPointSize ) ; }
  float      GetLineWidth()  { return ( m_fLineWidth ) ; }
  int        GetStipple()    { return ( m_iStipple ) ; }  
  
  // simple data set
  SmVector3d SetColor(const SmVector3d &rColor)                  { SmVector3d rtn(m_fRed, m_fGreen, m_fBlue) ;
                                                                   m_fRed   = (float)rColor.x ;
                                                                   m_fGreen = (float)rColor.y ;
                                                                   m_fBlue  = (float)rColor.z ;
                                                                   return rtn ;
                                                                 }
  SmVector3d SetColor(float fRed, float fGreen, float fBlue)     { SmVector3d rtn(m_fRed, m_fGreen, m_fBlue) ;
                                                                   m_fRed   = fRed   ;
                                                                   m_fGreen = fGreen ;
                                                                   m_fBlue  = fBlue  ;
                                                                   return rtn ;
                                                                 }
  float      SetRed      (float fRed  )                          { float rtn = m_fRed ;   m_fRed   = fRed ;   return rtn ; }
  float      SetGreen    (float fGreen)                          { float rtn = m_fGreen ; m_fGreen = fGreen ; return rtn ; }
  float      SetBlue     (float fBlue )                          { float rtn = m_fBlue ;  m_fBlue  = fBlue ;  return rtn ; }
                                                                 
  float      SetPointSize(float fPointSize)                      { float rtn = m_fPointSize ; m_fPointSize = fPointSize ; return rtn ; }
  float      SetLineWidth(float fLineWidth)                      { float rtn = m_fLineWidth ; m_fLineWidth = fLineWidth ; return rtn ; }
  int        SetStipple  (int   iStipple  )                      { int   rtn = m_iStipple ;   m_iStipple   = iStipple   ; return rtn ; }  
                                                          
  int        SetDashedLines(SmBoolean bStipple)                  { if(bStipple) { int rtn = m_iStipple ; m_iStipple = 0x00FF ; return rtn ; } 
                                                                   else         { int rtn = m_iStipple ; m_iStipple = 0xFFFF ; return rtn ; }
                                                                 }

  // Output to display
  SmStatus Display( SmPolygonOutputCallback & rPolyOutput );

} ; // end class SmGfxArraySet


#endif // !__SmGfxVertexArray_H__
