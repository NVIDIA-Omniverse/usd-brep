// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmGraphicsExtern.h
* PURPOSE: Interface to display list based graphics.
**********************************************************************/

#ifndef __SMGFX_EXTERN_H__
#define __SMGFX_EXTERN_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#include <fstream>

class SmFaceuse ;
class SmFace ;
class SmSurface ;
class SmAxis2Placement ;
class SmVolume ;
class SmGfxArraySet ;

/*******************************************************************//**
PURPOSE: display options and interface convenience display mode names.

NOTES: 
***********************************************************************/
enum SmTessellatorType {
  SM_DM_NLIB              = 1,  // 1 - NLIB Tessellation output using Triangles  (works on SmFace and SmSurface)
  SM_DM_SMLIB             = 3,  // 3 - SMLib Tessellation output using Triangles (works on SmFace only)
  SM_DM_UNTRIMMED_SURFACE = 4   // 4 - Untrimmed simple surface triangulation    (works only on SmSurface)
};

enum SmPinCushionType {     // specify what data is shown at each sample point in a PinCushion plot
  SM_DM_POINTS        = 1,  // 1 - Points                  
  SM_DM_U_NATURAL     = 2,  // 2 - Points + DU natural      
  SM_DM_V_NATURAL     = 3,  // 3 - Points + DV natural      
  SM_DM_UV_NATURAL    = 4,  // 4 - Points + DU natural and DV natural
  SM_DM_UNIT_NORMAL   = 5,  // 5 - Points + Normal Unitized
  SM_DM_U_SCALED      = 6,  // 6 - Points + DU scaled       
  SM_DM_V_SCALED      = 7,  // 7 - Points + DV scaled       
  SM_DM_UV_SCALED     = 8,  // 8 - Points + DU scaled and DV scaled  
  SM_DM_SCALED_NORMAL = 9,  // 9 - Points + Normal Scaled
  SM_DM_W_NATURAL     = 10, // 10 - Points + DW natural
  SM_DM_UVW_NATURAL   = 11, // 11 - Points + DU, DV, and DW natural
  SM_DM_W_SCALED      = 12, // 12 - Points + DW scaled
  SM_DM_UVW_SCALED    = 13  // 13 - Points + DU, DV, and DW scaled
} ;                         // where: DU = 1stDeriv in U, DV = 1stDeriv in V, DW = 1stDeriv in W

enum SmDrawModeType {
  SM_DM_CURRENT           = 0,
  SM_DM_HIDDENLINE        = 1,  //  1 - Hidden Curve removal
  SM_DM_WIREFRAME         = 2,  //  2 - Wireframe draw of edges and vertices of brep
  SM_DM_CROSSHATCH        = 3,  //  3 - Crosshatch draw of faces with uniform crosshatch lines
  SM_DM_FACETS_NLIB       = 4,  //  4 - NLIB Tessellation output using Triangles 
  SM_DM_FACETS_SMLIB      = 6,  //  6 - SMLib Tessellation output using Triangles
  SM_DM_NORMALS           = 7,  //  7 - Wireframe with face midPoint Normals
  SM_DM_KNOTS             = 8,  //  8 - Wireframe with knots points
  SM_DM_CONTROL_NET       = 9,  //  9 - Wireframe with control polygons
  SM_DM_CONTROL_POINTS    = 10, // 10 - Wireframe with control points
  SM_DM_CURVATRUE         = 11, // 11 - Wireframe with curvature combs
  SM_DM_SPEED             = 12, // 12 - Wireframw with curvature comb scaled to 1stDeriv Mag lengths
  SM_DM_DERIVATIVES       = 13, // 13 - Wireframe with 1st derivative combs
  SM_DM_PINCUSHION        = 14, // 14 - Wireframe with surface pinCushion
  SM_DM_PARAMETERIZATION  = 15, // 15 - Wireframe with curve parameterization sample points
  SM_DM_LOOPUSES          = 16, // 16 - Wireframe with Loopuse graphics for every loopuse
  SM_DM_NEIGHBORS         = 17, // 17 - When drawing vertex, edge, or face draw immediate neighbors with crosshatching
  SM_DM_FACEUSENEIGHBORS  = 18, // 18 - Crosshatch Faceuse and its immediate neighbors
  SM_DM_HIGHCOUNTCURVES   = 19, // 19 - Wireframe with high control point curves shown in special color and thickness
  SM_DM_SEAMS             = 20, // 20 - Just a Seam isoParameter curve for every closed boundary
  SM_DM_SINGULARITIES     = 21, // 21 - Just a Point Icon for every surface singular edge
  SM_DM_BREP_SHELLS       = 22, // 22 - Draw Brep Shells - colored for bounding infinite, void, and solid regions
  SM_DM_FLATCORNERS       = 23  // 23 - Draw FlatCorner Graphics (neighborhood of isoParam lines)                            
} ;                          

/*******************************************************************//**
PURPOSE: Specify options for how and where polygons represented in 
         a SmTess object are output by the SmTess::OutputPolygons() method.
         
NOTES: The output target options are:
   1.  Copy SmTess polygons into a PolyBrep model.
   2.  Send SmTess polygons to current draw stream through a sequence of OpenGL calls.
   3.  Send SmTess polygons to a SmGfxArray object suitable for rapid rendering.
***********************************************************************/
enum SmPolygonOutputType 
{
 SM_PO_CREATE_POLYBREP,        // Send polygons as triangles                   to SmTess::m_p3DPolyBrep 
 SM_PO_TRIANGLES,              // Send only triangles                          to draw stream or SmGfxArray.
 SM_PO_QUADRALATERALS,         // Send quadralaterals for adjacent coplanar
                               //   polygons that share a longest edge, else 
                               //   Send triangles                             to draw stream or SmGfxArray.
 SM_PO_TRIANGLE_STRIPS,        // Send triangle strips (see OpenGL) 
                               //      when possible (see m_lMinFanTriangles)  to draw stream or SmGfxArray.
 SM_PO_TRIANGLE_FANS,          // Send TriangleFans (see OpenGL) when possible to draw stream or SmGfxArray.
 SM_PO_STRIPS_OR_FANS,         // Send Strips when possible and then Fans next to draw stream or SmGfxArray.  
 SM_PO_TRIANGLE_MESH,          // Send a triangle mesh (indexed set of arrays)  
                               //       for each face                          to draw stream or SmGfxArray.
 SM_PO_ALL_POLYGON_EDGES,      // Send only the polygon edges                  to draw stream or SmGfxArray.
 SM_PO_ALL_NON_PLANAR_EDGES,   // Send only non-coplanar edges of a polygon    to draw stream or SmGfxArray.
 SM_PO_BOUNDARY_EDGES,         // Send edges on trimmed surface boundaries     to draw stream or SmGfxArray.
 SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES // Send bndry and silhouette PolyEdges    to draw stream or SmGfxArray.
};

/*******************************************************************//**
PURPOSE: Track Source types of target points for SmCurve::EditEndPoint() calls.

NOTES: 
  1. UVTrimCurve endPoints are moved to make 0.0 sized endGaps
      in sm_FixUVTrimCurves called by SmFace::CreateUVTrimCurves().  
      Target Points are selected to be somewhere in the gap between 
      existing UVTrimCurves.  The first 5 enumerations name the various
      options for that target point selection.

  2. 3d Curve EndPoints are set as part of a construction process.
     This currently happens in fillet construction in calls
       SmCircularCrossSectionFSG::CreateSurfaceFromCurves()
  
  3. UVTrimCurve endPoints are also moded to make 0.0 sized endGaps
     in SmTrimmingTools::FixTrimLoopTinyParameterSpaceGaps().

***********************************************************************/
enum SmEditEndType 
{
  SM_EE_UNKNOWN,                                // uninitialized values
  SM_EE_PROJECT_UVTRIMCURVE_VERTEX_TO_SURFACE,  // UVTrimCurve target end = projection of edge->VertexPoint onto surface
  SM_EE_UVTRIMCURVE_TANGENT_EXTENSION,          // UVTrimCurve target end = XSect of ConnectedUVTrimCurve tangent extensions (clamped to Face->UVDomain              
  SM_EE_UVTRIMCURVE_END_POINT_AVERAGE,          // UVTrimCurve target end = Average of ConnectedUVTrimCurve end point locations
  SM_EE_PREV_UVTRIMCURVE_END_POINT,             // UVTrimCurve target end = Previous Edgeuse unedited EndPoint 
  SM_EE_CURR_UVTRIMCURVE_END_POINT,             // UVTrimCurve target end = Current Edgeuse  unedited EndPoint
  SM_EE_CONSTRUCTED_END_POINT,                  // Curve endPoint position being set as part of a construction process
};

#ifdef SM_DEBUG_CODE
#ifdef GWC
static const char *sEditEndTypeDesc[] =
 {"Unknown",
  "Proj Vertex To Surface",
  "UVTrimCurve End Tangent XSect",
  "UVTrimCurve End Point Average",
  "UVTrimCurve Prev End Point",
  "UVTrimCurve Curr End Point",
  "Constructed End Point"
 } ;
#endif // GWC
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Container for Display List and its Variable Display Parameters.

NOTES: contains: GLuint - DisplayList Identifier
                          Color
                          PointSize
                          LineWidth
    included here to be distributed for all SmObject::Draw functions()
***********************************************************************/
typedef unsigned int GLuint;

class SM_EXPORT SmDisplayList
{  
protected:
  GLuint     m_lDisplayListId ;  // unique Id of active display list

public:
  double     m_dLineWidth ;      // line size output prior to DisplayList output
  double     m_dPointSize ;      // point size output prior to DisplayList output
  SmBoolean  m_bDashedLine ;     // 
  SmVector3d m_sColor ;          // Color output prior to DisplayList output
  double     m_dHotPointSize ;   // hot point size output prior to DisplayList output

public:
  // constructor
  SmDisplayList
  (
    GLuint lDisplayListId = SM_BIG_ULONG,
    double dLineWidth = 1.0,
    double dPointSize = 2.0,
    double dRed = 0.0,
    double dGreen = 0.0,
    double dBlue = 0.0,
    double dHotPointSize = 4.0
  )
    : m_lDisplayListId( lDisplayListId ),
      m_dLineWidth( dLineWidth ),
      m_dPointSize( dPointSize ),
      m_bDashedLine( FALSE ),
      m_dHotPointSize( dHotPointSize )
  {
    m_sColor.Set( dRed, dGreen, dBlue );
  }

  // constructor
  SmDisplayList
  (
    GLuint             lDisplayListId,
    double             dLineWidth,
    double             dPointSize,
    const SmVector3d & rColor,
    double             dHotPointSize = 4.0 )
    : m_lDisplayListId( lDisplayListId ),
      m_dLineWidth( dLineWidth ),
      m_dPointSize( dPointSize ),
      m_bDashedLine( FALSE ),
      m_sColor( rColor ),
      m_dHotPointSize( dHotPointSize )
  {}

  // destructor
  ~SmDisplayList() { m_lDisplayListId = SM_BIG_ULONG ; }   


  // Simple Data Access
  GLuint     GetDisplayListId()  const            { return m_lDisplayListId ; }
  double     GetLineWidth()      const            { return m_dLineWidth ; }
  double     GetPointSize()      const            { return m_dPointSize ; }
  SmBoolean  GetDashedLine()     const            { return m_bDashedLine ; }
  SmVector3d GetColor()          const            { return m_sColor ; }
  double     GetHotPointSize()   const            { return m_dHotPointSize ; }
                                                  
  GLuint     SetDisplayListId( GLuint lDisplayListId )
  {
    GLuint lRtn = m_lDisplayListId;
    m_lDisplayListId = lDisplayListId;
    return(lRtn);
  }

  double     SetLineWidth( double dLineWidth )
  {
    double dRtn = m_dLineWidth;
    m_dLineWidth = dLineWidth;
    return(dRtn);
  }

  double     SetPointSize( double dPointSize )
  {
    double dRtn = m_dPointSize;
    m_dPointSize = dPointSize;
    return(dRtn);
  }

  SmBoolean  SetDashedLine( SmBoolean bDashedLine )
  {
    SmBoolean bRtn = m_bDashedLine;
    m_bDashedLine = bDashedLine;
    return(bRtn);
  }

  SmVector3d SetColor( double dRed, double dGreen, double dBlue )
  {
    SmVector3d vRtn = m_sColor;
    m_sColor.Set( dRed, dGreen, dBlue );
    return(vRtn);
  }

  SmVector3d SetColor( const SmVector3d &rColor )
  {
    SmVector3d vRtn = m_sColor;
    m_sColor = rColor;
    return(vRtn);
  }

  double     SetHotPointSize( double dHotPointSize )
  {
    double dRtn = m_dHotPointSize;
    m_dHotPointSize = dHotPointSize;
    return(dRtn);
  }

  // convenience Set Functions
  void SetLook
  ( 
    double dLineWidth,                 // in : new lineWidth
    double dPointSize,                 // in : new pointSize
    double dRed,                       // in : new color
    double dGreen,
    double dBlue,
    double dHotPointSize = 4.0         // in : new hot point size
  )       
  {
    m_dPointSize = dPointSize;
    m_dLineWidth = dLineWidth;
    m_sColor.Set( dRed, dGreen, dBlue );
    m_dHotPointSize = dHotPointSize;
  }

  void SetLook
  (
    double dLineWidth,                 // in : new lineWidth
    double dPointSize,                 // in : new pointSize
    SmVector3d &rColor,                // in : new color    
    double dHotPointSize = 4.0         // in : new hotPointSize
  )
  {
    m_dPointSize = dPointSize;
    m_dLineWidth = dLineWidth;
    m_sColor = rColor;
    m_dHotPointSize = dHotPointSize;
  }

  void Set
  (
    GLuint lDisplayListId,                 // in : new DisplayList ID
    double dLineWidth,                     // in : new lineWidth
    double dPointSize,                     // in : new pointSize
    const SmVector3d &rColor,              // in : new color    
    double dHotPointSize = 4.0             // in : new HotPointSize
  )
  {
    m_lDisplayListId = lDisplayListId;
    m_dPointSize = dPointSize;
    m_dLineWidth = dLineWidth;
    m_sColor = rColor;
    m_dHotPointSize = dHotPointSize;
  }
                                                  
  // equality operator
  SmBoolean operator==( const SmDisplayList & rDisplayList ) const
  {
    return(m_lDisplayListId == rDisplayList.m_lDisplayListId);
  }

  // assignment operator
  SmDisplayList& operator= ( const SmDisplayList& src )
  {
    if(&src == this) return(*this);
    m_lDisplayListId = src.m_lDisplayListId;
    m_dLineWidth = src.m_dLineWidth;
    m_dPointSize = src.m_dPointSize;
    m_bDashedLine = src.m_bDashedLine;
    m_sColor = src.m_sColor;
    m_dHotPointSize = src.m_dHotPointSize;
    return(*this);
  }

  const TCHAR * GetTypeString() const { return _T("SmDisplayList") ; }
} ; // end class SmDisplayList

/*******************************************************************//**
PURPOSE: Display properties for generating object graphics.
            These are values that compile into individual object display lists.

NOTES: contains: Draw Mode
                          Shade State
                          CrossHatch State
                          Hidden Curve State
                          Tessellation State
                          Application Colors
                          Line and Point sizes
***********************************************************************/
class SM_EXPORT SmDisplayParameters
{
public:
  ///////////////// Drawing Mode /////////////////////
  SmBoolean           m_bHiddenCurve ;              // [FALSE] TRUE = Draw hidden line image
  SmBoolean           m_bDrawWireFrame ;            // [TRUE ] TRUE = Draw all Edges and Vertices
  SmBoolean           m_bDrawFacets ;               // [FALSE] TRUE = Generate/Output polygon facets

  SmBoolean           m_bDoShading;                 // [TRUE ] TRUE = display polygon facets as shaded image
                                                    //         FALSE= display polygon facets as wires. 
  SmBoolean           m_bDraw3DField;               // [FALSE] TRUE = project graphics through 3d-to-3d space shaping map
                                                    //         FALSE= don't
  SmVolume           *m_p3DField;                   // [NULL]  3DField space shaping transformation   
                                                      
  SmTessellatorType   m_eTessellator;               // [SM_DM_SMLIB] Specify Nlib/openGL/SMLib tessellator
  SmBoolean           m_bUseEdgeTypeColors ;        // [TRUE ] TRUE = Select EdgeColor based on type (manifold, lamina, wire)
                                                    //         FALSE= Select EdgeColor based on attribute or owner color

  SmBoolean           m_bDrawCrossHatch ;           // [FALSE] TRUE = Draw UV IsoParameterCurve crosshatch lines
  SmBoolean           m_bVaryCrossHatchColor ;      // [FALSE] TRUE = Draw V IsoParameterCurvs in m_VaryCrossHatchColor
  SmBoolean           m_bDrawKnots;                 // [FALSE] TRUE = Draw edge knotPoints (place surface xhatch lines on knot boundaries) 
  SmBoolean           m_bDrawPolygon;               // [FALSE] TRUE = Draw volume->ControlMesh, surface->ControlNet and curve->ControlPolygon
  SmBoolean           m_bDrawControlPoints;         // [FALSE] TRUE = Draw volume->ControlPoints, surface->ControlPoints and curve->ControlPoints
  SmBoolean           m_bDrawNormals ;              // [FALSE] TRUE = Draw Surface midPoint Normal vector
  SmBoolean           m_bDrawSeams ;                // [FALSE] TRUE = Draw Seam Isoparameter Curves
  SmBoolean           m_bDrawSingularities ;        // [FALSE] TRUE = Draw point icon at Surface Singularities
  SmBoolean           m_bDrawFlatCorners ;          // [FALSE] TRUE = Draw graphics for FlatCorners
  SmBoolean           m_bDrawCurvature ;            // [FALSE] TRUE = Draw Curve Curvature combs
  SmBoolean           m_bDrawSpeed ;                // [FALSE] TRUE = Draw Curvature vectors with length of 1stDeriv vectors
  SmBoolean           m_bDrawDerivatives ;          // [FALSE] TRUE = Draw Curve 1st Derivative combs
  SmBoolean           m_bDrawPinCushion ;           // [FALSE] TRUE = Draw surface vector fields
  SmBoolean           m_bDrawParameterization ;     // [FALSE] TRUE = Draw Curve with sequence of sample points increasing in size 
                                                    //                and changing color from green to blue to show curve parameterization
  SmBoolean           m_bDrawLoopuses ;             // [FALSE] TRUE = Draw Loopuse graphics for every Face->Loopuse
  SmBoolean           m_bDrawNeighbors ;            // [FALSE] TRUE = When drawing a vertex, edge, or face, also draw 
                                                    //                topology objects connected to them.
  SmBoolean           m_bDrawMicro ;                // [FALSE] TRUE = When drawing a Vertex/Edge, Vertex/Face, or Edge/Face connection
                                                    //                also draw micro topology graphics.
  SmBoolean           m_bDrawHighCountCurves ;      // [FALSE] TRUE = When drawing a Brep, draw high control point count curves
                                                    //                in special color and thickness
                                                    //
  SmBoolean           m_bDrawTypedShells ;          // [FALSE] TRUE = Draw Brep Shells in different colors for bounding Infinite, void, and solid regions
  SmDrawModeType      m_eLastMode ;                 // Last value stored by smgfx_SetDrawingMode
                                                    // note: this value does not determine the draw
                                                    //       state - rather when it was set all the
                                                    //               drawing bits were set to a specific state,
                                                    //               since that time, any draw bit might have been altered.
                                                    //               This is saved to be kindof backward compatible with the
                                                    //               previous mode based interface style.

  ///////////////// DrawFaceuse ///////////////////////////////////////
  SmBoolean           m_bDrawFaceuse ;              // [FALSE] TRUE = DrawFace is being called from Faceuse::Draw or SmShell::Draw()
  const SmFaceuse   * m_pFaceuse ;                  // Shells and Faceuses call DrawFace which knows which FU to draw from this value
  double              m_dNormalGain ;               // [0.0] Gain applied to DrawFaceuse normals, 0.0 = guess from size of Shell

  ///////////////// MicroGraphics ///////////////////////////////////
  double              m_dMicroEdgeParam ;           // [SM_BIG_DOUBLE] != SM_BIG_DOUBLE - center micro graphics on this param point
  ULONG               m_lMicroEdgeSampleCount ;     // [200] - total number of Edge sample points clustered about m_dMicroEdgeParam

  ///////////////// Shading ///////////////////////////////////
  SmBoolean           m_bFlatShading   ;            // [FALSE] TRUE = When shading use flat shading, not Gouraud shading
  SmBoolean           m_bHiddenPolygon ;            // [FALSE] TRUE = When shading skip back facing polygons
  double              m_dLightShininess ;           // [50.0]  light shininess - sets specular behavior, [0-100]
  SmVector3d          m_sLightDir ;                 // [0,0,1] direction of light for shaded displays.  [0,0,1] is straight down the viewing vector.
  SmVector3d          m_sLightAmbient ;             // [.1, .1, .1] undirected light in the room 
  SmVector3d          m_sLightDiffuse ;             // [.65, .65, .65] light projected in the LightDir direction
  SmVector3d          m_sLightSpecular ;            // [.1, .1, .06] light reflected as shine
   
  //////////////  Cross Hatch /////////////////////////////////////
  ULONG               m_lCrossHatchUCount;          // [4] How many curves between the Knots in the U cross hatch
  ULONG               m_lCrossHatchVCount;          // [4] How many curves between the Knots in the V cross hatch
  ULONG               m_lCrossHatchWCount;          // [4] How many iso Surfaces to drwa in w direction
  double              m_dCrossHatchLineWidth;       // [1] Line width for the non-knot cross hatch curves
  double              m_dCrossHatchKnotLineWidth;   // [2] Line width for the cross hatch curves on knot values

  ///////////// Hidden Curve ///////////////////////////////
  ULONG               m_lHiddenCurveDash ;          // [1] 0 = visible, 1 - dash invisible, 2 - silhouette only
  SmAxis2Placement    m_vHCRView;                   // [Origin=(0,0,0), XAxis=(1,0,0), YAxis=(0,1,0)]
  SmBoolean           m_bPerspective;               // [FALSE] TRUE = Use Projection View not OrthoGraphic
  double              m_dHiddenCurveTolerance;      // 0.0 (if zero then set from brep->GetTol)
  SmBoolean           m_bDisplaySeams;              // [TRUE] Display seams on hidden Curve if true
  SmBoolean           m_bDisplaySmoothEdges;        // [TRUE] Display smooth edges on hidden Curve if true

  //////////////// Tessellation /////////////////
  double              m_dCrvTessAngle;              // [ 8.0 ] Angular tolerance used in tessellation in degrees
  double              m_dSrfTessAngle;              // [ 8.0 ] Angular tolerance used in tessellation in degrees
  double              m_dChordHeight;               // [ 0.01] Tolerance that controls curve and polygon tessellation
                                                    //         using chord height measurements.   
  double              m_dPixelTolerance;            // [25.0 ] Tolerance that controls surface and face tessellation
                                                    //         using chord height measurements for OpenGL this
                                                    //         is a pixel space distance (i.e. 25.0).
  double              m_dMax3DEdge;                 // [ 0.0 ] Expected max EdgeLength to tessellate. When negative,
                                                    //         compute dMax3DEdge based on box object BoundingBox size as
                                                    //         dMax3DEdge = -0.025 * BBox.Size * dMax3DEdge
  double              m_dMaxAspectRatio;            // [ 0.0 ] When negative, use viewBased surface tessellation, 
                                                    //         else use view independent surface tessellation 
  SmPolygonOutputType m_ePolygonOutputType;         // [SM_PO_TRIANGLES] Specify how SMLib tessellator outputs polygons
  SmBoolean           m_bViewBasedTess;             // [FALSE] TRUE = NOT USED YET 
  double              m_dGapLineWidth;                // [6.0] number of pixels used to draw lines           
  double              m_dGapPointSize;                // [8.0] number of pixels used to draw point icons           
  SmVector3d          m_GapPointColor ;               // [1,0,0] Color for gap display display
  SmVector3d          m_VertexEdgeGapColor ;          // [1,0,1] Color for VertexEdge gap lines     
  SmVector3d          m_VertexFaceGapColor ;          // [0,1,0] Color for VertexFace gap lines     
  SmVector3d          m_EdgeFaceGapColor ;            // [0,1,1] Color for EdgeFace gap lines  
  SmVector3d          m_EdgeEdgeGapColor ;            // [0,.5,1] Color for EdgeEdge gap lines
  SmVector3d          m_VertexFaceTrimCurveGapColor ; // [.7,.2,.1] Color for VertexUVTrimCurve gap lines
  SmVector3d          m_EdgeFaceTrimCurveGapColor ;   // [.2,.7,.1] Color for EdgeUVTrimCurve gap lines
  SmVector3d          m_InTolGapColor ;               // [1,.5,0] Color for GapFunction vectors segments in tolerance   
  SmVector3d          m_OutTolGapColor ;              // [.3,.3,.3] Color for GapFunction vectors segments out of tolerance   

  //////////// Application Colors /////////////////////////////
  SmVector3d          m_DefaultColor ;              // [0,0,0] normal object default color
  SmVector3d          m_DefaultShadingColor ;       // [.7,.7,.7] normal object default color used for Shaded Breps
  SmVector3d          m_SpineEdgeColor ;            // [0,.5,1] color for spine  edges when drawn from Breps
  SmVector3d          m_WireEdgeColor ;             // [0,1,0] color for wire   edges when drawn from Breps
  SmVector3d          m_LaminaEdgeColor ;           // [0,0,1] color for lamina edges when drawn from Breps
  SmVector3d          m_VaryCrossHatchColor ;       // [1,1,0] color for VDir crosshatch lines when m_bVaryCrossHatchColor == TRUE
  SmVector3d          m_CurveControlPolygonColor ;  // [1,0,0] color for curve   control polygons
  SmVector3d          m_SurfaceControlPolygonColor; // [0,0,1] color for surface control polygons
  SmVector3d          m_VolumeControlPolygonColor;  // [0,1,1] color for Volume  control polygons

  SmVector3d          m_SurfaceNormalColor ;        // [1,0,0] color for surface Normal vectors for normals pointing out of the screen
  SmVector3d          m_NegSurfaceNormalColor ;     // [0,0,1] color for surface Normal vectors for normals pointing into the screen
  SmVector3d          m_SurfaceNormalInfiniteColor; // [0,1,0] color for surface Normal vectors pointing into the infinite region
  SmVector3d          m_SurfaceNormalSolidColor ;   // [0,0,1] color for surface Normal vectors pointing into a solid region
  SmVector3d          m_SurfaceNormalVoidColor ;    // [1,0,0] color for surface Normal vectors pointing into a void region

  SmVector3d          m_KnotColor ;                 // [0,0,1] color for knots
  SmBoolean           m_bUseCurvature ;             // [TRUE] TRUE=use m_CurvatureColor, FALSE=use m_Curvature2Color
  SmVector3d          m_CurvatureColor ;            // [1,1,0] color for curvature combs
  SmVector3d          m_Curvature2Color ;           // [1,.65,0] color for curvature combs
  SmVector3d          m_SpeedColor ;                // [1,.5,0] color for speed combs
  SmVector3d          m_BaseSurfaceColor ;          // [.2,.2,.5] color for base surfaces of SmOffsetSurface objects
  SmVector3d          m_ExtendedSurfaceColor ;      // [.2,.5,.2] color for extended surfaces of SmOffsetSurface objects
  SmVector3d          m_NeighborColor ;             // [1,0,0] Color for neighbor Objects in DrawNeighbor functions
  SmVector3d          m_NeighborMateColor ;         // [0,1,0] Color for neighbor Mate Objects in DrawNeighbor functions
  SmVector3d          m_HighCountCurveColor ;       // [1,.5,.1] Color for drawing High ControlPoint Count Curves
  SmVector3d          m_ClassificationVertexColor ; // [1,0,0] Color for drawing CurveClassification vertex objects
  SmVector3d          m_ClassificationEdgeColor ;   // [0,1,0] Color for drawing CurveClassification edge objects
  SmVector3d          m_ClassificationFaceColor ;   // [0,1,1] Color for drawing CurveClassification face objects
  SmVector3d          m_InterruptColor ;            // [SM_BIG_DOUBLE..] when set to anything but SM_BIG_DOUBLE,
                                                    //      overrides normal object color selection
  SmVector3d          m_HotPointColor ;             // [1,0,0] Color for pickable Hot Point display
  SmVector3d          m_HotPointFillColor ;         // [1,1,0] Color for non pickable parts of the Hot Point display
  SmVector3d          m_HotPointSelectColor ;       // [1,0,0] Color for selected Hot Point display
  SmVector3d          m_HotPointHighlightColor ;    // [1,0,0] Color for highlighted Hot Point display 
  SmVector3d          m_TrackTargetColor ;          // [.2,.2,.7] Color to display current track mode icons
  SmVector3d          m_TrackInfoColor ;            // [.7,.7,.7] Color to display track mode orientation icons     
  SmVector3d          m_TestGeometryColor ;         // [0,.8,.1] Color to display Test Geometry as it is constructed and modified
  SmVector3d          m_OutputColor ;               // last color output to drawlist                                        
  SmVector3d          m_TmpColor ;                  // temp storage to cut down on construct/destruct calls                               
                                                                        
  /////////////// Line and Point Sizes ///////////////////////
  double              m_dLineWidth;                 // [2.0] number of pixels used to draw lines           
  double              m_dPointSize;                 // [4.0] number of pixels used to draw point icons           
  double              m_dHotPointSize;              // [6.0] number of pixels used to draw HotPoint icons
  SmBoolean           m_bDashedLines;               // [FALSE] TRUE = draw crosshatched curves dashed
  double              m_dHighCountCurveWidth;       // [6.0] number of pixels used to draw high control-point curves
  ULONG               m_lHighCountCurveLimit;       // [150] number of controlPoints in a curve to make it a HighCountCurve
  
  //////////////// Curvature, Derivative, and Parameterization Combs and PinCushions //////////////////////                                               
  ULONG               m_lSamplePointCount ;         // [ 25  ] Number of curve sample points                                                
  ULONG               m_lUPointCount ;              // [ 10  ] Number of surface U Sample Points
  ULONG               m_lVPointCount ;              // [ 10  ] Number of surface V Sample Points
  ULONG               m_lWPointCount ;              // [  5  ] Number of volume W Sample Points

  SmPinCushionType    m_ePinCushionType ;           // [SM_DM_UNIT_NORMAL] Select various vector fields
  double              m_dPinCushionScale ;          // [ 10.0] Scale applied to PinCushion Vectors
  double              m_dCurvatureScale ;           // [-20.0] Scale applied to curvature combs
  double              m_d3rdDerivScale ;            // [  1.0] Scale applied to 3rd derivative vectors

  // constructor
  SmDisplayParameters
    (SmDrawModeType eMode=SM_DM_WIREFRAME,
     SmBoolean bDoShading=TRUE) ;

  // equality operator
  SmBoolean operator==(const SmDisplayParameters&) const;

  // convenience predicates
  SmBoolean IsShaded()            const { return(m_bDrawFacets && m_bDoShading) ; }
  SmBoolean IsNLibFaceting()      const { return(m_bDrawFacets && m_eTessellator == SM_DM_NLIB) ; }
  SmBoolean IsSMLibFaceting()     const { return(m_bDrawFacets && m_eTessellator == SM_DM_SMLIB) ; }
  SmBoolean IsUntrimmedFaceting() const { return(m_bDrawFacets && m_eTessellator == SM_DM_UNTRIMMED_SURFACE) ; }

  SmColorRuleType GetShadedColorRule()   const
  {
    return(IsShaded() ? SM_CR_SHADING : SM_CR_STANDARD);
  }

  SmColorRuleType GetPropertyColorRule() const
  {
    return(m_bUseEdgeTypeColors ? SM_CR_OBJPROPERTY : SM_CR_STANDARD);
  }

  // list of things drawn in the Face/Surface outputGraphics() call
  SmBoolean IsDrawingFaceFeature() const
  {
    return (IsNLibFaceting()
         || m_bDrawCrossHatch
         || m_bDrawPolygon
         || m_bDrawControlPoints
         || m_bDrawNormals
         || m_bDrawSeams
         || m_bDrawSingularities
         || m_bDrawFlatCorners
         || m_bDrawPinCushion);
  }
                                                 
  const TCHAR * GetTypeString() const { return _T("SmDisplayParameters") ; }

} ; // end class SmDisplayParameters

/*******************************************************************//**
PURPOSE: Draw properties for generating output graphics

NOTES: contains: Global Draw State
                          View Orientation
                          Application Colors
                          Line and Point Sizes

***********************************************************************/
class SM_EXPORT SmViewParameters
{
public:
  /////////// global Draw State ////////////////////////////////
  SmBoolean        m_bGraphicsInitialized = FALSE ; // [FALSE] TRUE = ready to output graphcis.
                                                    //         FALSE= don't output graphics.
                                                       
  SmTArray<SmDisplayList> *m_pActiveLists = NULL ; // list of all active display lists
                                                   // gwc:note - this list stores SmDisplayList objects not pointers.
                                                   //  Application keeping track of these lists must avoid storing
                                                   //  pointers or references to the DisplayLists stored within m_pActiveLists.
                                                   //  Those references will become stale when the list changes. Applications
                                                   //  need to store m_pActiveList[ii].m_lDisplayListId values instead which
                                                   //  remain constant throughout list modification calls.
  ULONG            m_lNumOpened = SM_UNDEF_ULONG ; // number of open gl display lists
                                                   // used to prevent smgfx_Open from opening a 2nd display list
                                                   // before smgfx_Close is called to close the last one.
                                                  
  ///////////// View Orientation ///////////////////////////////                                                      
  SmVector3d       m_sMid;                         // [ 0,  0,  0] center of ortho viewing box
  SmVector3d       m_sSize;                        // [20, 20, 20] extent of ortho viewing box centered on m_sMid
  SmPoint3d        m_sRotationCenter;              // [0, 0, 0] center of mouse based rotations                                                          
  // constructor
  SmViewParameters()
   : m_bGraphicsInitialized(FALSE),    
     m_pActiveLists(NULL),
     m_lNumOpened(0),    
     m_sMid(0.0,0.0,0.0),
     m_sSize(20.0, 20.0, 20.0),
     m_sRotationCenter(0.0,0.0,0.0)
     { } 
     
   // queries
    
  const TCHAR * GetTypeString() const { return _T("SmViewParameters") ; }

} ; // end class SmViewParameters

/*******************************************************************//**
PURPOSE: Controls how and where polygons represented in 
   a SmTess object are output by the SmTess::OutputPolygons() method.

   The user should subclass this class and the OutputPolygon method
   to receive the polygons into his own functions.
         
NOTES: m_eOutputType specifies the default output options as:
   1.  Copy SmTess polygons into a PolyBrep model.
   2.  Send SmTess polygons to current draw stream through a sequence of OpenGL calls.
   3.  Send SmTess polygons to a SmGfxArray object suitable for rapid rendering.
***********************************************************************/
class SM_EXPORT SmPolygonOutputCallback 
{
protected:
  SmPolygonOutputType  m_eOutputType;               //          control OutputPolygon() output behavior, oneof:
             // SM_PO_CREATE_POLYBREP,              //          Output creates a 3-D SmPolyBrep in SmTess::m_p3DPolyBrep
             // SM_PO_TRIANGLES,                    // default: Output only triangles.
             // SM_PO_QUADRALATERALS,               //          Output quadralaterals when two adjacent 
             //                                     //            coplanar polygons are found that border with their longest edge.
             //                                     //            Otherwise output triangles when these conditions are not meet.
             // SM_PO_TRIANGLE_STRIPS,              //          Output triangle strips (see OpenGL) when possible (see m_lMinFanTriangles)
             // SM_PO_TRIANGLE_FANS,                //          Output triangle fans (see OpenGL) when possible.
             // SM_PO_STRIPS_OR_FANS,               //          Output strips when possible and then fans next.  
             // SM_PO_TRIANGLE_MESH,                //          Output a mesh of triangles (indexed set of arrays) for each face.
             // SM_PO_ALL_POLYGON_EDGES,            //          Output only the polygon edges
             // SM_PO_ALL_NON_PLANAR_EDGES,         //          Output only non-coplanar edges of a polygon
             // SM_PO_BOUNDARY_EDGES,               //          Output edges that correspond to trimmed surface boundaries
             // SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES //          Output boundary edges and silhouette edges of polygons.

  ULONG      m_lMinFanTriangles;     // Min number of triangles in legal fan
  ULONG      m_lNumVerticesOutput;   // total number of vertices output.
  ULONG      m_lPolygonsOutputCount; // total number of polygons output.

  SmBoolean  m_bGenerateNormals;     // TRUE = send out surface normals.
  SmBoolean  m_bPerspective;         // TRUE = doing an view operation in perspective.
  SmBoolean  m_bFlatShading;         // TRUE = Flat shading for polygons, FALSE = smooth shading
  SmBoolean  m_bShadedMode;          // TRUE = output colored polgons, FALSE = output polylines.
  SmBoolean  m_bHiddenLine;          // TRUE = generate a polygonal hidden line for m_vViewDirectionOrEye value.

  SmVector3d m_vViewDirectionOrEye;  // (perspective == TRUE) ? eye point : else view direction.
  double     m_dHCRShrinkFactor;     // Shrink Polygon amount,  when doing Z-Buffer HCR.  
                                     //   default:[0.05 for a 10x10x10 model]
  double     m_dHCRBackFacingDot;    // Min value for back facing dot product(ViewVector, PolygonNormal)
                                     // This causes the shrinkage to go awayfrom eye.  
                                     //  default:[0.2 works good for anglular tessellation tolerances < 30 degrees].  
  double     m_dHCRWireOffset;       // wireframe offset dist toward eye when doing Z-Buffer HCR

  ULONG      m_lMatID;               // Material ID for current Face
  ULONG      m_lSmooth;              // Smoothing Group for Current Face

public:
    // constructor
  SmPolygonOutputCallback( SmPolygonOutputType eOutType = SM_PO_TRIANGLES ) 
    : m_eOutputType( eOutType ),
      m_lMinFanTriangles( 2 ),
      m_lNumVerticesOutput( 0 ),
      m_lPolygonsOutputCount( 0 ),
      
      m_bGenerateNormals( TRUE ),
      m_bPerspective( FALSE ),
      m_bFlatShading( FALSE ),
      m_bShadedMode( TRUE ),
      m_bHiddenLine( FALSE ),
      
      m_vViewDirectionOrEye( 0, 0, 0 ),
      m_dHCRShrinkFactor( 0.05 ),
      m_dHCRBackFacingDot( 0.2 ),
      m_dHCRWireOffset( 0.05 ),
      
      m_lMatID( 0 ),
      m_lSmooth( 0 )
  {}
                                     
    // destructor
    virtual ~SmPolygonOutputCallback() { }

    // virtual methods

    // called by SmTess::OutputFacePolygons for every SmTess->Polygon
    virtual SmStatus OutputPolygon(ULONG           lPolygonType,       // in : 
                                   ULONG           lNumPoints,         // in : 
                                   SmPoint3d     * aPolygonPoints,     // in : 
                                   SmVector3d    * aPolygonNormals,    // in : 
                                   SmPoint2d     * aPolygonUVPoints,   // NotUsed: in : 
                                   SmSurface     * pSurface,           // NotUsed: in : 
                                   SmFace        * pFace,              // NotUsed: in : 
                                   SmGfxArraySet * pOptGfxSet=NULL) ;  // i/o: 

    // output mesh of polygons represented as lists of points, surfNormals, and polygon indices.
    virtual SmStatus OutputMesh(SmTArray<ULONG>      & rPolygonVertexCounts,  // in : 
                                SmTArray<ULONG>      & rPolygonVertexIndices, // in : 
                                SmTArray<SmPoint3d>  & rPolygon3DPoints,      // in : 
                                SmTArray<SmVector3d> & rSurfaceNormals,       // in : 
                                SmTArray<SmPoint2d>  & rPolygonUVPoints,      // NotUsed: in : 
                                SmSurface            * pSurface,              // NotUsed: in : 
                                SmFace               * pFace,                 // NotUsed: in : 
                                SmGfxArraySet        * pOptGfxSet=NULL) ;     // i/o: 

    virtual SmStatus OutputFaceWireframe(SmFace                * pFace,                 // NotUsed: in :
                                         const SmTArray<ULONG> & crEdgeVertexIndicies,  // in :
                                         SmTArray<SmPoint3d>   & cr3DPoints,            // in :
                                         SmGfxArraySet         * pOptGfxSet=NULL) ;     // in :
                                        
    virtual SmStatus CompletedPolygonOutput()                { return SM_SUCCESS; }

    // simple get data access
    SmBoolean           GetGenerateNormals()     const       { return m_bGenerateNormals; }
    ULONG               GetSmooth()              const       { return m_lSmooth; }
    ULONG               GetMatID()               const       { return m_lMatID; }
    ULONG               GetMinFanTriangles()     const       { return m_lMinFanTriangles; }
    SmPolygonOutputType GetOutputType()          const       { return m_eOutputType; }
    ULONG               GetPolygonsOutputCount() const       { return m_lPolygonsOutputCount; }
    ULONG               GetNumVerticesOutput()   const       { return m_lNumVerticesOutput; }

    SmVector3d GetViewDirection(const SmPoint3d & crPointToLookAt) const ;
    
    // simple set data access
    void SetOutputType
    (
      SmPolygonOutputType eOutputType // in : control OutputPolygon() behavior, oneof:
                                      // SM_PO_CREATE_POLYBREP,              // Output creates a 3-D SmPolyBrep
                                      // SM_PO_TRIANGLES,                    // Output only triangles.
                                      // SM_PO_QUADRALATERALS,               // Output quadralaterals when two adjacent 
                                      //                                     //   coplanar polygons are found that border with their longest edge.
                                      //                                     //   Otherwise output triangles when these conditions are not meet.
                                      // SM_PO_TRIANGLE_STRIPS,              // Output triangle strips (see OpenGL) when possible (see m_lMinFanTriangles)
                                      // SM_PO_TRIANGLE_FANS,                // Output triangle fans (see OpenGL) when possible.
                                      // SM_PO_STRIPS_OR_FANS,               // Output strips when possible and then fans next.  
                                      // SM_PO_TRIANGLE_MESH,                // Output a mesh of triangles (indexed set of arrays) for each face.
                                      // SM_PO_ALL_POLYGON_EDGES,            // Output only the polygon edges
                                      // SM_PO_ALL_NON_PLANAR_EDGES,         // Output only non-coplanar edges of a polygon
                                      // SM_PO_BOUNDARY_EDGES,               // Output edges that correspond to trimmed surface boundaries
                                      // SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES // Output boundary edges and silhouette edges of polygons.
    )
    {
      m_eOutputType = eOutputType;
    }

    void SetGenerateNormals(SmBoolean           bGenerateNormals) { m_bGenerateNormals    = bGenerateNormals; }
    void SetSmooth         (ULONG               lSmooth)          { m_lSmooth             = lSmooth; }
    void SetMatID          (ULONG               lMatID)           { m_lMatID              = lMatID; }
    void SetMinFanTriangles(ULONG               lMinFanTriangles) { m_lMinFanTriangles    = lMinFanTriangles; }
    void SetEyePoint       (const SmPoint3d   & crEyePoint)       { m_bPerspective        = TRUE; 
                                                                    m_vViewDirectionOrEye = crEyePoint; 
                                                                  }
    void SetViewDirection  (const SmVector3d  & crViewDirection)  { m_bPerspective        = FALSE; 
                                                                    m_vViewDirectionOrEye = crViewDirection; 
                                                                  }
    void SetGraphicsFlags  (SmBoolean           bShadedMode,      
                            SmBoolean           bFlatShading,     
                            SmBoolean           bHiddenLine)      { m_bShadedMode         = bShadedMode; 
                                                                    m_bFlatShading        = bFlatShading; 
                                                                    m_bHiddenLine         = bHiddenLine; 
                                                                  }
    void SetHCRTolerances  (double              dHCRShrinkFactor,    
                            double              dHCRBackFacingDot,
                            double              dHCRWireOffset);  

} ; // end class SmPolygonOutputCallback

/*******************************************************************//**
PURPOSE: This object creates SLA files for polygons
    by overloading the virtual OutputPolygon() method
    which is called by the SmTess::OutputPolygons() method.
    
NOTES: 
    1. inherits from SmPolygonOutputCallback the specification
       of how polygons represented in a SmTess object are output by 
       the SmTess::OutputPolygons() method.

    2. WE have changed to use FILE * and not ostream for m_pFout
       because ostream does not permit nonASCII characters to be used as
       directory/file names. Change back to stream to allow binary output
***********************************************************************/
class SM_EXPORT SmPolygonSLAOutput : public SmPolygonOutputCallback
{
protected:

  std::ostream * m_pFout;             // Stream used for writing SLA
  bool           m_bCleanupStream;
  ULONG          m_lMode;             // 0 - Default write all triangles into a single solid in SLA
                                      // 1 - Each face is a single solid in SLA file - for now this only
                                      //     works in ASCII SLA files
                                      //     
  SmSurface    * m_pLastSurface;      // Last surface pointer.
  ULONG          m_lSolidCounter;     // Counts solids and gives each a unique name in SLP file
  SmFileType     m_eFileType;         // Type of SLA file to write ASCII or BINARY
  SmTArray<SmPoint3d> m_sVertices;    // Used if binary mode to store triangles

public:
  SmPolygonSLAOutput
  (
    const TCHAR * cOutputFileName,
    SmFileType    eFileType,
    ULONG         lMode,
    SmBoolean   & rbFailure
  );

  SmPolygonSLAOutput
  (
    std::ostream & outputStream,
    SmFileType     eFileType,
    ULONG          lMode,
    SmBoolean    & rbFailure
  );

  virtual ~SmPolygonSLAOutput() { if (m_pFout && m_bCleanupStream) { delete(m_pFout); m_pFout = NULL; } }

  // called by SmTess::OutputFacePolygons which calls stream output methods for every SmTess->Polygon to build a SLA file.
  virtual SmStatus OutputPolygon(ULONG           lPolygonType,      // NotUsed: in : 
                                 ULONG           lNumPoints,        // in : 
                                 SmPoint3d     * aPolygonPoints,    // in : 
                                 SmVector3d    * aPolygonNormals,   // NotUsed: in : 
                                 SmPoint2d     * aPolygonUVPoints,  // NotUsed: in : 
                                 SmSurface     * pSurface,          // NotUsed: in : 
                                 SmFace        * pFace,             // in : 
                                 SmGfxArraySet * pOptGfxSet=NULL) ; // NotUsed: i/o:

  virtual SmStatus CompletedPolygonOutput();

} ; // end class SmPolygonSLAOutput

/*******************************************************************//**
PURPOSE: This object creates ProEngineer SLP files for polygons
    by overloading the virtual OutputPolygon() method
    which is called by the SmTess::OutputPolygons() method.

NOTES: 
    1. inherits from SmPolygonOutputCallback the specification
       of how polygons represented in a SmTess object are output by 
       the SmTess::OutputPolygons() method.
***********************************************************************/
class SM_EXPORT SmPolygonSLPOutput : public SmPolygonOutputCallback
{
protected:
 // inherited from SmPolygonOutputCallback
 //  SmPolygonOutputType  m_eOutputType;               // control OutputPolygon() output behavior, oneof:
 //             // SM_PO_CREATE_POLYBREP,              // Output creates a 3-D SmPolyBrep
 //             // SM_PO_TRIANGLES,                    // Output only triangles.
 //             // SM_PO_QUADRALATERALS,               // Output quadralaterals when two adjacent 
 //             //                                     //   coplanar polygons are found that border with their longest edge.
 //             //                                     //   Otherwise output triangles when these conditions are not meet.
 //             // SM_PO_TRIANGLE_STRIPS,              // Output triangle strips (see OpenGL) when possible (see m_lMinFanTriangles)
 //             // SM_PO_TRIANGLE_FANS,                // Output triangle fans (see OpenGL) when possible.
 //             // SM_PO_STRIPS_OR_FANS,               // Output strips when possible and then fans next.  
 //             // SM_PO_TRIANGLE_MESH,                // Output a mesh of triangles (indexed set of arrays) for each face.
 //             // SM_PO_ALL_POLYGON_EDGES,            // Output only the polygon edges
 //             // SM_PO_ALL_NON_PLANAR_EDGES,         // Output only non-coplanar edges of a polygon
 //             // SM_PO_BOUNDARY_EDGES,               // Output edges that correspond to trimmed surface boundaries
 //             // SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES // Output boundary edges and silhouette edges of polygons.
 // 
 //  ULONG      m_lMinFanTriangles;     // Min number of triangles in legal fan
 //  ULONG      m_lNumVerticesOutput;   // total number of vertices output.
 //  ULONG      m_lPolygonsOutputCount; // total number of polygons output.
 // 
 //  SmBoolean  m_bGenerateNormals;     // TRUE = send out surface normals.
 //  SmBoolean  m_bPerspective;         // TRUE = doing an view operation in perspective.
 //  SmBoolean  m_bFlatShading;         // TRUE = Flat shading for polygons, FALSE = smooth shading
 //  SmBoolean  m_bShadedMode;          // TRUE = output colored polgons, FALSE = output polylines.
 //  SmBoolean  m_bHiddenLine;          // TRUE = generate a polygonal hidden line for m_vViewDirectionOrEye value.
 // 
 //  SmVector3d m_vViewDirectionOrEye;  // (perspective == TRUE) ? eye point : else view direction.
 //  double     m_dHCRShrinkFactor;     // Shrink Polygon amount,  when doing Z-Buffer HCR.  
 //                                     //   default:[0.05 for a 10x10x10 model]
 //  double     m_dHCRBackFacingDot;    // Min value for back facing dot product(ViewVector, PolygonNormal)
 //                                     // This causes the shrinkage to go awayfrom eye.  
 //                                     //  default:[0.2 works good for anglular tessellation tolerances < 30 degrees].  
 //  double     m_dHCRWireOffset;       // wireframe offset dist toward eye when doing Z-Buffer HCR
 // 
 //  ULONG      m_lMatID;               // Material ID for current Face
 //  ULONG      m_lSmooth;              // Smoothing Group for Current Face

  std::ofstream  * m_pFout;             // Stream used for writing SLA
  ULONG       m_lMode;                  // 0 - Default as a single solid in SLP with a single color
                                        // 1 - Each face is a single solid in SLP file with a single color
                                        // 2 - Each Polygon, Tri-Strip, Tri-Fan is a single solid in 
                                        //     SPL file with its own color
                                        //     
  SmSurface * m_pLastSurface;           // Last surface pointer.
  ULONG       m_lSolidCounter;          // Counts solids and gives each a unique name in SLP file
                     

public:
  // constructor
  SmPolygonSLPOutput
  (
    const TCHAR       * cOutputFileName, // in : target file name
    SmPolygonOutputType eOutputType,     // in : oneof: 
                                         //      SM_PO_CREATE_POLYBREP, SM_PO_TRIANGLE_MESH,                   
                                         //      SM_PO_TRIANGLES,       SM_PO_ALL_POLYGON_EDGES,               
                                         //      SM_PO_QUADRALATERALS,  SM_PO_ALL_NON_PLANAR_EDGES, 
                                         //      
                                         //      SM_PO_TRIANGLE_STRIPS, SM_PO_BOUNDARY_EDGES,              
                                         //      SM_PO_TRIANGLE_FANS,   SM_PO_BOUNDARY_AND_SILHOUETTE_EDGES         
                                         //      SM_PO_STRIPS_OR_FANS,
    ULONG               lMode,           // in : 0 - Default as a single solid in SLP with a single color
                                         //      1 - Each face is a single solid in SLP file with a single color
                                         //      2 - Each Polygon, Tri-Strip, Tri-Fan is a single solid 
                                         //          in SPL file with its own color
    SmBoolean         & rbFailure        // out: FALSE = ok, TRUE = something wrong    
  );

   // destructor
  virtual ~SmPolygonSLPOutput()
  {
    if(m_pFout)
    {
      delete(m_pFout); m_pFout = NULL;
    }
    m_pLastSurface = NULL;
  }

  virtual SmStatus OutputPolygon(ULONG           lPolygonType,       // in :
                                 ULONG           lNumPoints,         // in :
                                 SmPoint3d     * aPolygonPoints,     // in :
                                 SmVector3d    * aPolygonNormals,    // in :
                                 SmPoint2d     * aPolygonUVPoints,   // NotUsed: in :
                                 SmSurface     * pSurface,           // NotUsed: in :
                                 SmFace        * pFace,              // NotUsed: in :
                                 SmGfxArraySet * pOptGfxSet=NULL) ;  // i/o:

  virtual SmStatus CompletedPolygonOutput();

  SmStatus OutputTriangle(const SmPoint3d  & crP0,
                          const SmPoint3d  & crP1,
                          const SmPoint3d  & crP2,
                          const SmVector3d & crN0,
                          const SmVector3d & crN1,
                          const SmVector3d & crN2) ;

} ; // end class SmPolygonSLPOutput

// CALLBACK TO WRITE TRIANGLES TO FILE
class SM_EXPORT SmPolygonFileOutput : public SmPolygonOutputCallback
{
public:
  ULONG m_lPolygonType     = SM_UNDEF_ULONG ;
  ULONG m_lNumberTriangles = SM_UNDEF_ULONG ;
  FILE * m_FilePtr         = NULL ;
  SmPolygonFileOutput() {};
  virtual ~SmPolygonFileOutput() {};

  virtual SmStatus OutputPolygon(ULONG           lPolygonType,         // NotUsed: in : 
                                 ULONG           lNumPoints,           // in : 
                                 SmPoint3d     * aPolygonPoints,       // in : 
                                 SmVector3d    * aPolygonNormals,      // in : 
                                 SmPoint2d     * aPolygonUVPoints,     // in : 
                                 SmSurface     * pSurface,             // NotUsed: in : 
                                 SmFace        * pFace,                // NotUsed: in : 
                                 SmGfxArraySet * pOptGfxSet = NULL) ;  // NotUsed: i/o: 

}; // end class SmPolygonFileOutput

/*******************************************************************//**
PURPOSE:  smgfx display interface api

NOTES:
***********************************************************************/
SM_EXPORT SmViewParameters    &smgfx_RefGlobalViewParameters() ;
SM_EXPORT SmDisplayParameters &smgfx_RefGlobalDisplayParameters() ;

#if defined(SM_GFX_CODE)

#ifndef NULL
#define NULL 0
#endif

// access GlobalDisplayParameters for reference or modification
SM_EXPORT        void smgfx_SetDisplayParameters      (SmDrawModeType eMode, SmDisplayParameters &rDisp) ;
SM_EXPORT inline void smgfx_GetGlobalDisplayParameters(SmDisplayParameters &rDisp) { rDisp = smgfx_RefGlobalDisplayParameters() ; }

// start/end display environment
SM_EXPORT void   smgfx_Initialize(const SmContext & crContext); // define lights, draw state, etc.
SM_EXPORT void   smgfx_Terminate();                             // free display lists
                    
// interactive debugging functions                    
SM_EXPORT void   sm_GraphicsLoop(SmBoolean bInteractiveFlag=FALSE) ; // TRUE = call the UI callback
                                                                     // TRUE  = enter CSMLibApp::Run2 message loop
                                                                     // UNSURE= process CSMLibApp::Run2 message queue and return
SM_EXPORT void   sm_GraphicsAddToBrepList(const SmObject *pObject) ;
SM_EXPORT void   sm_GraphicsBrepListClear() ;
                    

// display list management
SM_EXPORT SmDisplayList  smgfx_Open
(
  const SmVector3d & rColor=SmVector3d(0,0,0),
  double           * pOptLineWidth=NULL,
  double           * pOptPointSize=NULL,
  SmBoolean          bDashedLines=FALSE,
  SmGfxArraySet    * pOptGfxSet=NULL
) ;

SM_EXPORT SmDisplayList *smgfx_Close           (SmGfxArraySet * pOptGfxSet=NULL) ;  // end or decrement nested displayList count - return Id or NULL for nested closes
SM_EXPORT void           smgfx_Erase           (SmBoolean bClearUIPickLists=TRUE) ; // empty global displayList array - clear screen
SM_EXPORT int            smgfx_AnyDrawing      ();                                  // rtn: 1=displayLists stored in global displayList array, 0=none stored
                                                 
#define SMGFX_DRAWLIST(pASet, a) { smgfx_Open(smgfx_GetRuleColor(),NULL,NULL,FALSE,pASet) ; a ; smgfx_Close(pASet) ; }

// draw state management 
SM_EXPORT void      smgfx_SetLook        (double             dLineWidth,                 // eff: convenience function to set 3 params at once
                                          double             dPointSize,                 
                                          double             dRed,                       
                                          double             dGreen,                     
                                          double             dBlue,                      
                                          SmGfxArraySet    * pOptGfxSet=NULL);           
                                                                                         
SM_EXPORT double    smgfx_SetLook        (double             dLineWidth,                 // rtn: current chordHeight param value
                                          double             dPointSize,                 
                                          double             dRed,                       
                                          double             dGreen,                     
                                          double             dBlue,                      
                                          double             dChordHeight,               // in : defaults to 0.1
                                          SmGfxArraySet    * pOptGfxSet=NULL);           
                                         
SM_EXPORT void      smgfx_SetLook        (double             dLineWidth,
                                          double             dPointSize,
                                          const SmVector3d & rRGB,
                                          SmGfxArraySet    * pOptGfxSet=NULL);
                                         
SM_EXPORT void      smgfx_SetLook        (double             dLineWidth,                 // in : LineWidth in pixels  (commonly 1 to 3)
                                          double             dPointSize,                 // in : Point Size in pixels (commonly 2 to 5)
                                          SmBoolean          bChangeColor=TRUE,          // in : smgfx_ChangeColor(TRUE ) = inc color seq as: red - yellow - magenta - green - blue - cyan - red . . .
                                                                                         //    : smgfx_ChangeColor(FALSE) = Init Color sequence (to red)
                                          SmGfxArraySet    * pOptGfxSet=NULL) ;          // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                         
SM_EXPORT void      smgfx_GetLook        (double * dLineWidth, double * dPointSize, double *dRed, double *dGreen, double *dBlue) ;
SM_EXPORT double    smgfx_GetPointSize   ();                                                          // rtn: current value 
SM_EXPORT double    smgfx_SetPointSize   (double dPointSize, SmGfxArraySet * pOptGfxSet=NULL);         // eff: set new value, rtn last PointSize value
SM_EXPORT double    smgfx_GetLineWidth   ();                                                          // rtn: current value
SM_EXPORT double    smgfx_SetLineWidth   (double dLineWidth, SmGfxArraySet * pOptGfxSet=NULL);         // eff: set new value, rtn last LineWidth value
                    
SM_EXPORT double    smgfx_GetHotPointSize();                                // rtn: current value
SM_EXPORT double    smgfx_SetHotPointSize(double dHotPointSize,             // eff: set new value, rtn last HotPointSize value
                                          SmGfxArraySet * pOptGfxSet=NULL) ;
SM_EXPORT double    smgfx_GetGapPointSize();                                // rtn: current value 
SM_EXPORT double    smgfx_SetGapPointSize(double dGapPointSize,             // eff: set new value, rtn last GapPointSize value
                                          SmGfxArraySet * pOptGfxSet=NULL) ;
SM_EXPORT double    smgfx_GetGapLineWidth();                       // rtn: current value
SM_EXPORT double    smgfx_SetGapLineWidth(double dGapLineWidth, SmGfxArraySet * pOptGfxSet = NULL);   // eff: set new value, rtn last GapLineWidth value
                    
SM_EXPORT ULONG     smgfx_GetHighCountCurveLimit();                
                    
// eff: set new value, rtn last HighCountCurveLimit value
SM_EXPORT ULONG     smgfx_SetHighCountCurveLimit( ULONG lHighCountCurveLimit );         
                    
SM_EXPORT double    smgfx_GetHighCountCurveWidth();                
                    
// eff: set new value, rtn last HighCountCurveWidth value
SM_EXPORT double    smgfx_SetHighCountCurveWidth( double dHighCountCurveWidth );                

SM_EXPORT SmBoolean smgfx_GetDashedLines();                                                        
SM_EXPORT SmBoolean smgfx_SetDashedLines(SmBoolean bDashedLines, SmGfxArraySet * pOptGfxSet=NULL);  // eff: set new value, rtn last DashedLines value

SM_EXPORT inline void smgfx_SetRotationCenter( const SmPoint3d &sRotationCenter ) { smgfx_RefGlobalViewParameters().m_sRotationCenter = sRotationCenter; }

SM_EXPORT inline const SmPoint3d & smgfx_GetRotationCenter()                      { return(smgfx_RefGlobalViewParameters().m_sRotationCenter) ; }

SM_EXPORT SmBoolean   smgfx_IsOrthoView() ;

SM_EXPORT void        smgfx_SetViewVolume(double dXMid, double dXSize,  // set the ortho view box center and size
                                          double dYMid, double dYSize,
                                          double dZMid, double dZSize);

SM_EXPORT void        smgfx_ZoomBox(double dNewCentXDelta,              // move and scale the ortho view box
                                    double dNewCentYDelta,
                                    double dNewSizeDelta);

SM_EXPORT void        smgfx_ZoomWorldBox
                               (SmExtent3d &rBBox,                      // in : box to center on screen
                                double      dScale=0.9,                 // in : box view scale, .9 = box takes up about 90% of screen
                                SmPoint3d  *pCenter=NULL) ;             // in : normalized BBox pos centered in screen, NULL = [.5, .5, .5]
                                                                        //      ex: [.75,.5,.5] BBox is scooted to the left (the bbox pt to the right is centered)
                                                                        //      default:[NULL]

SM_EXPORT void        smgfx_ZoomIn();                                   // halve the ortho view box size
  

SM_EXPORT void        smgfx_ZoomOut();                                  // double the ortho view box size
 


// Basic Draw routines and associated callBacks
SM_EXPORT void smgfx_DrawPoint   (double dX,  double dY,  double dZ,  SmGfxArraySet *pOptGfxSet=NULL);
SM_EXPORT void smgfx_DrawHotPoint(double dX,  double dY,  double dZ,  SmGfxArraySet *pOptGfxSet=NULL);
SM_EXPORT void smgfx_DrawLine    (double dX1, double dY1, double dZ1, 
                                  double dX2, double dY2, double dZ2, SmGfxArraySet *pOptGfxSet=NULL);
SM_EXPORT void smgfx_DrawLine    (SmPoint3d *P0, SmPoint3d *P1,       SmGfxArraySet *pOptGfxSet=NULL);
SM_EXPORT void smgfx_DrawPolyline(double *pdPoints, long nNumPoints,  SmGfxArraySet *pOptGfxSet=NULL);
SM_EXPORT void smgfx_DrawComb    (double *pdVecs, double *pdBasePts, long nPts, 
                                  double dScale=1.0, 
                                  long lVecStrideInDoubles=3, long lBasePtStrideInDoubles=3,    SmGfxArraySet *pOptGfxSet=NULL) ;
SM_EXPORT void smgfx_DrawPlane   (const SmPoint3d &rPlanePoint, const SmVector3d &rPlaneNormal, SmGfxArraySet *pOptGfxSet=NULL) ;
                                 // also see: smgfx_DrawPlaneGrid
               
SM_EXPORT void smgfx_SetDrawLineCallBack    
(
  void (*dl) (double x, double y,  double z, double xx,double yy, double zz, SmGfxArraySet *pOptGfxSet)
);

SM_EXPORT void smgfx_SetDrawPointCallBack   (void (*dl) (double x, double y,  double z, SmGfxArraySet *pOptGfxSet));

SM_EXPORT void smgfx_SetDrawPolyLineCallBack(void (*dpl)(double *pts, long npts, SmGfxArraySet *pOptGfxSet));
                    
// Object Draw Routines
SM_EXPORT void      smgfx_DrawTriangle
(
  const SmPoint3d & crP1,
  const SmPoint3d & crP2,
  const SmPoint3d & crP3,
  const SmVector3d * cpNormal1 = NULL,
  const SmVector3d * cpNormal2 = NULL,
  const SmVector3d * cpNormal3 = NULL,
  SmGfxArraySet *pOptGfxSet=NULL
);

SM_EXPORT void      smgfx_DrawQuad
(
  const SmPoint3d & crP1,
  const SmPoint3d & crP2,
  const SmPoint3d & crP3,
  const SmPoint3d & crP4,
  const SmVector3d * cpNormal1 = NULL,
  const SmVector3d * cpNormal2 = NULL,
  const SmVector3d * cpNormal3 = NULL,
  const SmVector3d * cpNormal4 = NULL,
  SmGfxArraySet *pOptGfxSet=NULL
);

SM_EXPORT void      smgfx_DrawCircularArc
(
  const SmPoint3d  &crOrigin,        // in : arc origin
  const SmVector3d &crX,             // in : defines zero degree start for arc
  const SmVector3d &crY,             // in : specifies circle plane and 90 degree direction for arc
  double            dRadius,         // in : arc radius
  double            dStartAngleDeg,  // in : arc start point in degrees
  double            dEndAngleDeg,    // in : arc end point in degrees
  double            dMaxHeightError, // in : max arc height error allowed
                                     // note: number of polysegments drawn
                                     //       is determined by the radius and
                                     //       the dMaxHeightError
  SmGfxArraySet *pOptGfxSet=NULL
) ;

SM_EXPORT void      smgfx_DrawPlaneGrid
(
  const SmPoint3d & crOrigin,        // in : origin of the grid
  const SmVector3d & crVecX,             // in : unit vecX direction to draw
  const SmVector3d & crVecY,             // in : unit vecY direction to draw
  ULONG              lVecXCopyCount = 5, // in : number of VecX copies drawn on each side of Vec1
  ULONG              lVecYCopyCount = 5, // in : number of VecY copies drawn on each side of Vec2
  double             dVecXSpacing = 1.0, // in : space between vecX copies
  double             dVecYSpacing = 1.0, // in : space between vecY copies
  SmGfxArraySet     *pOptGfxSet=NULL
) ;

SM_EXPORT void      smgfx_DrawCylinderGrid
(
  const SmPoint3d  & crAxisPt,         // in : Point on Cyl Axis
  const SmVector3d & crAxisVec,        // in : unit vec in Cyl Axis direction
  const SmVector3d & crVecX,           // in : unit vec from Origin to theta=0 cylinder IsoLine
  double             dRadius,          // in : Cylinder radius
  ULONG              lThetaPieces = 12,// in : number arc pieces from 0 to 360
  ULONG              lArcCount = 5,    // in : number of arc copies drawn on each side of crAxisPt
  double             dArcSpacing = 1.0,// in : space between arc copies
  SmGfxArraySet     *pOptGfxSet=NULL
) ;

SM_EXPORT void      smgfx_DrawSphereGrid
(
  const SmPoint3d  & crOrigin,           // in : origin of the sphere
  const SmVector3d & crVecX,             // in : unit vec in Sphere X direction (pts to theta = 0 or 360)
  const SmVector3d & crVecZ,             // in : unit vec in Sphere Z direction (pts to Phi = 180)
  double             dRadius,            // in : Cylinder radius
  ULONG              lThetaPieces = 12,  // in : number of ConstTheta minor 1/2 circles drawn from 0 to 360, default:[12]
  ULONG              lZCircleCnt = 5,    // in : number of const Z circles drawn from Bot to Top, default:[5]
  SmGfxArraySet     *pOptGfxSet=NULL
) ;

SM_EXPORT void      smgfx_DrawTorusGrid
(
  const SmPoint3d  & crOrigin,            // in : origin of the Torus
  const SmVector3d & crVecX,              // in : unit vec in Torus X direction (pts to theta = 0 or 360)
  const SmVector3d & crVecZ,              // in : unit vec in Torus Z direction (Normal to torus plane)
  double             dMajRadius,          // in : Cylinder radius
  double             dMinRadius,          // in : Cylinder radius
  ULONG              lThetaPieces = 12,   // in : number ConstTheta minor circles drawn from 0 to 360
  ULONG              lZCircleCnt = 5,     // in : number of const Z circles drawn from Bot to Top, default:[5]
  SmGfxArraySet *pOptGfxSet=NULL
) ;

SM_EXPORT void      smgfx_DrawDiscGrid 
(
  const  SmPoint3d & crOrigin,       // in : origin of the grid
  const  SmVector3d & crVec1,        // in : disc normal direction to draw
  double dRadius,                    // in : disc radius
  ULONG  lRadiiCount = 12,           // in : number of radii drawn per 360 degrees.
  SmGfxArraySet *pOptGfxSet=NULL
) ;

// get application colors
SM_EXPORT inline const SmVector3d &smgfx_GetColor()                      { return( smgfx_RefGlobalDisplayParameters().m_InterruptColor ) ; } 
SM_EXPORT inline const SmVector3d &smgfx_GetDefaultColor()               { return( smgfx_RefGlobalDisplayParameters().m_DefaultColor ) ; }   

SM_EXPORT        const SmVector3d &smgfx_GetRuleColor                                    // rtn: Color appropriate for Object/Rule pair
(
  const SmObject *pObject=NULL,               // in : target Object or NULL=try InterruptColor then DefaultColor
  SmColorRuleType eColorRule=SM_CR_STANDARD   // in : ColorRule for selecting special case colors
);

SM_EXPORT inline const SmVector3d &smgfx_GetDefaultShadingColor()        { return( smgfx_RefGlobalDisplayParameters().m_DefaultShadingColor ) ; } 
SM_EXPORT inline const SmVector3d &smgfx_GetSpineEdgeColor()             { return( smgfx_RefGlobalDisplayParameters().m_SpineEdgeColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetWireEdgeColor()              { return( smgfx_RefGlobalDisplayParameters().m_WireEdgeColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetLaminaEdgeColor()            { return( smgfx_RefGlobalDisplayParameters().m_LaminaEdgeColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetVaryCrossHatchColor()        { return( smgfx_RefGlobalDisplayParameters().m_VaryCrossHatchColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetCurveControlPolygonColor()   { return( smgfx_RefGlobalDisplayParameters().m_CurveControlPolygonColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetSurfaceControlPolygonColor() { return( smgfx_RefGlobalDisplayParameters().m_SurfaceControlPolygonColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetVolumeControlPolygonColor()  { return( smgfx_RefGlobalDisplayParameters().m_VolumeControlPolygonColor ) ; }

SM_EXPORT inline const SmVector3d &smgfx_GetSurfaceNormalColor()         { return( smgfx_RefGlobalDisplayParameters().m_SurfaceNormalColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetNegSurfaceNormalColor()      { return( smgfx_RefGlobalDisplayParameters().m_NegSurfaceNormalColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetSurfaceNormalInfiniteColor() { return( smgfx_RefGlobalDisplayParameters().m_SurfaceNormalInfiniteColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetSurfaceNormalSolidColor()    { return( smgfx_RefGlobalDisplayParameters().m_SurfaceNormalSolidColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetSurfaceNormalVoidColor()     { return( smgfx_RefGlobalDisplayParameters().m_SurfaceNormalVoidColor ) ; }

SM_EXPORT inline const SmVector3d &smgfx_GetKnotColor()                  { return( smgfx_RefGlobalDisplayParameters().m_KnotColor ) ; }
SM_EXPORT inline const void       smgfx_SetUseCurvature(SmBoolean bUseCurvature) { smgfx_RefGlobalDisplayParameters().m_bUseCurvature = bUseCurvature ; }
SM_EXPORT inline const SmVector3d &smgfx_GetCurvatureColor()             { return( smgfx_RefGlobalDisplayParameters().m_CurvatureColor ) ; }
SM_EXPORT        const SmVector3d &smgfx_SetCurvatureColor(double dRed, double dGreen, double dBlue) ;
SM_EXPORT inline const SmVector3d &smgfx_GetCurvature2Color()            { return( smgfx_RefGlobalDisplayParameters().m_Curvature2Color ) ; }
SM_EXPORT        const SmVector3d &smgfx_SetCurvature2Color(double dRed, double dGreen, double dBlue) ;
SM_EXPORT inline const SmVector3d &smgfx_GetSpeedColor()                 { return( smgfx_RefGlobalDisplayParameters().m_SpeedColor ) ; }
SM_EXPORT        const SmVector3d &smgfx_SetSpeedColor(double dRed, double dGreen, double dBlue) ;
SM_EXPORT inline const SmVector3d &smgfx_GetBaseSurfaceColor()           { return( smgfx_RefGlobalDisplayParameters().m_BaseSurfaceColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetExtendedSurfaceColor()       { return( smgfx_RefGlobalDisplayParameters().m_ExtendedSurfaceColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetNeighborColor()              { return( smgfx_RefGlobalDisplayParameters().m_NeighborColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetNeighborMateColor()          { return( smgfx_RefGlobalDisplayParameters().m_NeighborMateColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetHighCountCurveColor()        { return( smgfx_RefGlobalDisplayParameters().m_HighCountCurveColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetHotPointColor()              { return( smgfx_RefGlobalDisplayParameters().m_HotPointColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetHotPointFillColor()          { return( smgfx_RefGlobalDisplayParameters().m_HotPointFillColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetHotPointSelectColor()        { return( smgfx_RefGlobalDisplayParameters().m_HotPointSelectColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetHotPointHighlightColor()     { return( smgfx_RefGlobalDisplayParameters().m_HotPointHighlightColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetTrackTargetColor()           { return( smgfx_RefGlobalDisplayParameters().m_TrackTargetColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetTrackInfoColor()             { return( smgfx_RefGlobalDisplayParameters().m_TrackInfoColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetTestGeometryColor()          { return( smgfx_RefGlobalDisplayParameters().m_TestGeometryColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetGapPointColor()              { return( smgfx_RefGlobalDisplayParameters().m_GapPointColor ) ; }

SM_EXPORT inline const SmVector3d &smgfx_GetVertexEdgeGapColor()         { return( smgfx_RefGlobalDisplayParameters().m_VertexEdgeGapColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetVertexFaceGapColor()         { return( smgfx_RefGlobalDisplayParameters().m_VertexFaceGapColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetEdgeFaceGapColor()           { return( smgfx_RefGlobalDisplayParameters().m_EdgeFaceGapColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetEdgeEdgeGapColor()           { return( smgfx_RefGlobalDisplayParameters().m_EdgeEdgeGapColor ) ; }
SM_EXPORT inline const SmVector3d &smgfx_GetVertexUVTrimCurveGapColor()  { return( smgfx_RefGlobalDisplayParameters().m_VertexFaceTrimCurveGapColor ) ; } 
SM_EXPORT inline const SmVector3d &smgfx_GetEdgeUVTrimCurveGapColor()    { return( smgfx_RefGlobalDisplayParameters().m_EdgeFaceTrimCurveGapColor ) ; }

SM_EXPORT inline const SmVector3d &smgfx_GetInTolGapColor()              { return( smgfx_RefGlobalDisplayParameters().m_InTolGapColor ) ; }
SM_EXPORT        const SmVector3d  smgfx_SetInTolGapColor(SmVector3d &rInTolGapColor) ;
SM_EXPORT inline const SmVector3d &smgfx_GetOutTolGapColor()             { return( smgfx_RefGlobalDisplayParameters().m_OutTolGapColor ) ; }
SM_EXPORT        const SmVector3d  smgfx_SetOutTolGapColor(SmVector3d &rOutTolGapColor) ;

SM_EXPORT        void              smgfx_GetCurveClassificationObjectColors
(
  SmVector3d &rVertexColor,
  SmVector3d &rEdgeColor,
  SmVector3d &rFaceColor
) ;

SM_EXPORT inline const SmVector3d &smgfx_GetVertexFaceTrimCurveGapColor(){ return( smgfx_RefGlobalDisplayParameters().m_VertexFaceTrimCurveGapColor ) ; } 
SM_EXPORT inline const SmVector3d &smgfx_GetEdgeFaceTrimCurveGapColor()  { return( smgfx_RefGlobalDisplayParameters().m_EdgeFaceTrimCurveGapColor ) ; } 
                          
// set, change, test current interrupt color - output color to the drawlist
SM_EXPORT SmVector3d &smgfx_SetColor   (double dRed, double dGreen, double dBlue, SmGfxArraySet * pOptGfxSet=NULL);
SM_EXPORT SmVector3d &smgfx_SetColor   (const SmVector3d &rRGB,                   SmGfxArraySet * pOptGfxSet=NULL);
SM_EXPORT SmVector3d &smgfx_ChangeColor(SmBoolean bChangeColor=TRUE,              SmGfxArraySet * pOptGfxSet=NULL);
SM_EXPORT SmVector3d &smgfx_ClearColor (SmGfxArraySet * pOptGfxSet=NULL);

// return TRUE = InterruptColor is set to any color other than (SM_BIG_DOUBLE,SM_BIG_DOUBLE,SM_BIG_DOUBLE)
SM_EXPORT SmBoolean smgfx_AnyInterruptColor() ;

// set current default color - don't output color to the drawlist
SM_EXPORT SmVector3d &smgfx_SetDefaultColor(double dRed, double dGreen, double dBlue);
SM_EXPORT SmVector3d &smgfx_SetDefaultColor(const SmVector3d &rRGB);

SM_EXPORT void smgfx_SetCurveClassificationObjectLook
(
  double dLineWidth,   
  double dPointSize,
  double dVertexRed, double dVertexGreen, double dVertexBlue,
  double dEdgeRed,   double dEdgeGreen,   double dEdgeBlue,
  double dFaceRed,   double dFaceGreen,   double dFaceBlue,
  SmGfxArraySet *pOptGfxSet=NULL
) ;

// application draw mode management
SM_EXPORT        void           smgfx_SetDrawingMode(SmDrawModeType eMode) ;
SM_EXPORT inline SmDrawModeType smgfx_GetDrawingMode()                      { return(smgfx_RefGlobalDisplayParameters().m_eLastMode) ; }
SM_EXPORT        void           smgfx_SetFaceuseNormalGain(double dNormalGain) ;
SM_EXPORT inline double         smgfx_SetFaceuseNormalGain()                { return(smgfx_RefGlobalDisplayParameters().m_dNormalGain) ; }
SM_EXPORT        void           smgfx_SetShadedMode(SmBoolean bDoShading);
SM_EXPORT inline SmBoolean      smgfx_GetShadedMode()                       { return smgfx_RefGlobalDisplayParameters().m_bDoShading ; }

SM_EXPORT        void      smgfx_SetShaded
(
  SmBoolean bShadedDisplay,
  SmBoolean bFlatShading,
  SmBoolean bHiddenPolygon
);

SM_EXPORT        void      smgfx_GetShaded
(
  SmBoolean & rbShadedDisplay,
  SmBoolean & rbFlatShading,
  SmBoolean & rbHiddenPolygon
);

SM_EXPORT inline SmBoolean smgfx_GetDraw3DField()                           { return(smgfx_RefGlobalDisplayParameters().m_bDraw3DField) ; }
SM_EXPORT inline void      smgfx_SetDraw3DField(SmBoolean bDraw3DField)     { smgfx_RefGlobalDisplayParameters().m_bDraw3DField = bDraw3DField ; }
SM_EXPORT inline SmVolume *smgfx_Get3DField()                               { return(smgfx_RefGlobalDisplayParameters().m_p3DField) ; }
SM_EXPORT inline void      smgfx_Set3DField(SmVolume *p3DField)             { smgfx_RefGlobalDisplayParameters().m_p3DField = p3DField ; } 

SM_EXPORT void      smgfx_SetLightDir(double  dX, double  dY, double  dZ) ;
SM_EXPORT void      smgfx_GetLightDir(double &dx, double &dy, double &dz) ;
SM_EXPORT void      smgfx_SetLightAmbient(double  dRed, double  dGreen, double  dBlue) ;
SM_EXPORT void      smgfx_GetLightAmbient(double &dRed, double &dGreen, double &dBlue) ;
SM_EXPORT void      smgfx_SetLightDiffuse(double  dRed, double  dGreen, double  dBlue) ;
SM_EXPORT void      smgfx_GetLightDiffuse(double &dRed, double &dGreen, double &dBlue) ;
SM_EXPORT void      smgfx_SetLightSpecular(double  dRed, double  dGreen, double  dBlue) ;
SM_EXPORT void      smgfx_GetLightSpecular(double &dRed, double &dGreen, double &dBlue) ;

SM_EXPORT void      smgfx_SetHiddenCurve
(
  SmBoolean bHiddenCurve, 
  ULONG lHiddenCurveDash,
  SmAxis2Placement crViewTransformation,
  SmBoolean bPerspective, 
  double dHiddenCurveTolerance,
  SmBoolean bDisplaySeams,
  SmBoolean bDisplaySmoothEdges
);

SM_EXPORT void      smgfx_SetHiddenCurve
(
  SmBoolean bHiddenCurve, 
  ULONG lHiddenCurveDash
);

SM_EXPORT void      smgfx_GetHiddenCurve
(
  SmBoolean & rbHiddenCurve,  
  ULONG & rlHiddenCurveDash, 
  SmAxis2Placement & rViewTransformation,
  SmBoolean & rbPerspective, 
  double  & rdHiddenCurveTolerance,
  SmBoolean & rbDisplaySeams,
  SmBoolean & rbDisplaySmoothEdges
);

SM_EXPORT void      smgfx_GetHiddenCurve
(
  SmBoolean & rbHiddenCurve,
  ULONG & lHiddenCurveDash 
);

SM_EXPORT void      smgfx_SetTessTolerances
(
  double dCrvTessAngle, 
  double dSrfTessAngle,
  double dChordHeight,
  double dPixelTolerance,
  double dMax3DEdge, 
  double dMaxAspectRatio, 
  SmPolygonOutputType lPolygonOutputType 
);

SM_EXPORT void      smgfx_SetTessTolerances
(
  double dCrvTessAngle,  
  double dSrfTessAngle
);

SM_EXPORT void      smgfx_GetTessTolerances
(
  double & rdCrvTessAngle,  
  double & rdSrfTessAngle,                                        
  double & rdChordHeight,
  double & rdPixelTolerance,
  double & rdMax3DEdge,  
  double & rdMaxAspectRatio,  
  SmPolygonOutputType & rlPolygonOutputType 
);

SM_EXPORT void      smgfx_GetTessTolerances
(
  double & rdCrvTessAngle,  
  double & rdSrfTessAngle
); 

SM_EXPORT double    smgfx_SetChordHeight   (double dChordHeight) ;  // return previous ChordHeight setting                                          

SM_EXPORT inline SmBoolean smgfx_GetDrawCrossHatch()                           { return smgfx_RefGlobalDisplayParameters().m_bDrawCrossHatch; }
SM_EXPORT inline void      smgfx_SetDrawCrossHatch(SmBoolean bDrawCrossHatch)  { smgfx_RefGlobalDisplayParameters().m_bDrawCrossHatch = bDrawCrossHatch ; }                                                                            
SM_EXPORT        void      smgfx_SetUVHatchCount(ULONG  lUHatch, ULONG  lVHatch);
SM_EXPORT        void      smgfx_GetUVHatchCount(ULONG &lUHatch, ULONG &lVHatch);
SM_EXPORT        void      smgfx_SetUVWHatchCount(ULONG  lUHatch, ULONG  lVHatch, ULONG  lWHatch);
SM_EXPORT        void      smgfx_GetUVWHatchCount(ULONG &lUHatch, ULONG &lVHatch, ULONG &lWHatch);

SM_EXPORT inline void      smgfx_SetDrawKnots(SmBoolean bDrawKnots)  { smgfx_RefGlobalDisplayParameters().m_bDrawKnots = bDrawKnots ; }
SM_EXPORT inline SmBoolean smgfx_GetDrawKnots()                      { return smgfx_RefGlobalDisplayParameters().m_bDrawKnots ; }

SM_EXPORT        void      smgfx_SetDrawFacets(SmBoolean bDrawFacets, SmTessellatorType eTessellatorType);
SM_EXPORT        SmBoolean smgfx_GetDrawFacets(SmTessellatorType *pOptTessellatorType);

SM_EXPORT inline SmBoolean smgfx_GetDrawPinCushion()                                 { return smgfx_RefGlobalDisplayParameters().m_bDrawPinCushion ; }
SM_EXPORT inline void      smgfx_SetDrawPinCushion(SmBoolean bDrawPinCushion)        { smgfx_RefGlobalDisplayParameters().m_bDrawPinCushion = bDrawPinCushion ; }
                                                                          
SM_EXPORT inline void      smgfx_SetPinCushionType(SmPinCushionType ePinCushionType) { smgfx_RefGlobalDisplayParameters().m_ePinCushionType = ePinCushionType ; }
SM_EXPORT inline SmPinCushionType smgfx_GetPinCushionType()                          { return smgfx_RefGlobalDisplayParameters().m_ePinCushionType ; }

SM_EXPORT inline void      smgfx_SetPinCushionScale(double dPinCushionScale)         { smgfx_RefGlobalDisplayParameters().m_dPinCushionScale = dPinCushionScale ; }
SM_EXPORT inline double    smgfx_GetPinCushionScale()                                { return smgfx_RefGlobalDisplayParameters().m_dPinCushionScale ; }
                                                                              
SM_EXPORT inline void      smgfx_SetDrawPolygon(SmBoolean bDrawPolygon)              { smgfx_RefGlobalDisplayParameters().m_bDrawPolygon = bDrawPolygon ; }
SM_EXPORT inline SmBoolean smgfx_GetDrawPolygon()                                    { return smgfx_RefGlobalDisplayParameters().m_bDrawPolygon ; }

SM_EXPORT inline void      smgfx_SetDrawControlPoints(SmBoolean bDrawControlPoints)  { smgfx_RefGlobalDisplayParameters().m_bDrawControlPoints = bDrawControlPoints ; }
SM_EXPORT inline SmBoolean smgfx_GetDrawControlPoints()                              { return smgfx_RefGlobalDisplayParameters().m_bDrawControlPoints ; }

SM_EXPORT        void smgfx_SetDrawCurvature(SmBoolean  bDrawCurvature, double  dScale=-25.0, ULONG lSamplePointCount=25) ;
SM_EXPORT        void smgfx_GetDrawCurvature(SmBoolean &bDrawCurvature, double &dScale, ULONG &lSamplePointCount) ;
                      
SM_EXPORT        void smgfx_SetDrawSpeed(SmBoolean  bDrawSpeed, double  dScale=-1.0) ;
SM_EXPORT        void smgfx_GetDrawSpeed(SmBoolean &bDrawSpeed, double &dScale) ;

SM_EXPORT inline void smgfx_Set3rdDerivScale(double  d3rdDerivScale=1.0)        { smgfx_RefGlobalDisplayParameters().m_d3rdDerivScale = d3rdDerivScale ; }
SM_EXPORT inline void smgfx_Get3rdDerivScale(double &d3rdDerivScale)            { d3rdDerivScale = smgfx_RefGlobalDisplayParameters().m_d3rdDerivScale ; }

SM_EXPORT        void smgfx_SetCombScales(double  dPinCushionScale=10.0, double  dCurvatureScale=-25, double  d3rdDerivScale=1.0) ;
SM_EXPORT        void smgfx_GetCombScales(double &dPinCushionScale, double &dCurvatureScale, double &d3rdDerivScale) ;

SM_EXPORT inline void smgfx_SetCurveSampleCount  (ULONG  lUSampleCount)         { smgfx_RefGlobalDisplayParameters().m_lSamplePointCount = lUSampleCount ; }
SM_EXPORT inline void smgfx_SetSurfaceSampleCount(ULONG  lUSampleCount, 
                                                  ULONG  lVSampleCount)         { smgfx_RefGlobalDisplayParameters().m_lUPointCount      = lUSampleCount ;
                                                                                  smgfx_RefGlobalDisplayParameters().m_lVPointCount      = lVSampleCount ;
                                                                                }
SM_EXPORT inline void smgfx_SetVolumeSampleCount (ULONG  lUSampleCount, 
                                           ULONG  lVSampleCount, 
                                           ULONG  lWSampleCount)                { smgfx_RefGlobalDisplayParameters().m_lUPointCount      = lUSampleCount ;
                                                                                  smgfx_RefGlobalDisplayParameters().m_lVPointCount      = lVSampleCount ;
                                                                                  smgfx_RefGlobalDisplayParameters().m_lWPointCount      = lWSampleCount ;
                                                                                }

SM_EXPORT inline void smgfx_GetCurveSampleCount  (ULONG &lUSampleCount)         { lUSampleCount = smgfx_RefGlobalDisplayParameters().m_lSamplePointCount ; }
SM_EXPORT inline void smgfx_GetSurfaceSampleCount(ULONG &lUSampleCount, 
                                                  ULONG &lVSampleCount)         { lUSampleCount = smgfx_RefGlobalDisplayParameters().m_lUPointCount      ;
                                                                                  lVSampleCount = smgfx_RefGlobalDisplayParameters().m_lVPointCount      ;
                                                                                }
SM_EXPORT inline void smgfx_GetVolumeSampleCount (ULONG &lUSampleCount, 
                                                  ULONG &lVSampleCount, 
                                                  ULONG &lWSampleCount)         { lUSampleCount = smgfx_RefGlobalDisplayParameters().m_lUPointCount      ;
                                                                                  lVSampleCount = smgfx_RefGlobalDisplayParameters().m_lVPointCount      ;
                                                                                  lWSampleCount = smgfx_RefGlobalDisplayParameters().m_lWPointCount      ;
                                                                                }

SM_EXPORT void      smgfx_SetDrawHighCountCurves(SmBoolean bDrawHighCountCurves, 
                                                 ULONG  lHighCountCurveLimit = 150,
                                                 double dHighCountCurveWidth = 6.0, 
                                                 double dRed=1.0, double dGreen=.5, double dBlue=.1) ;
SM_EXPORT SmBoolean smgfx_GetDrawHighCountCurves() ;

// SM_EXPORT        void      smgfx_SetDrawGaps(SmBoolean bDrawGaps) ;
// SM_EXPORT inline SmBoolean smgfx_GetDrawGaps()                                  { return(smgfx_RefGlobalDisplayParameters().m_bDrawHighCountCurves) ; }

#else   // no SM_GFX_CODE - stub functions to help prog_test compile with fewer #ifdef commands
SM_EXPORT inline void smgfx_Initialize(const SmContext &) { smgfx_RefGlobalViewParameters().m_bGraphicsInitialized = TRUE ; }  
SM_EXPORT inline void smgfx_Terminate()                             { smgfx_RefGlobalViewParameters().m_bGraphicsInitialized = FALSE ; }
SM_EXPORT inline void sm_GraphicsLoop(SmBoolean = FALSE)             { }
SM_EXPORT inline void smgfx_Erase(SmBoolean = TRUE) { }

#define SMGFX_DRAWLIST(pASet, a)                                    { }

SM_EXPORT inline void   smgfx_SetLook(double, double, double, double, double, SmGfxArraySet * = NULL)         { }
SM_EXPORT inline double smgfx_SetLook(double, double, double, double, double, double, SmGfxArraySet * = NULL) { return 0.0 ; }
SM_EXPORT inline void   smgfx_SetLook(double, double, const SmVector3d &, SmGfxArraySet * = NULL)             { }
SM_EXPORT inline void   smgfx_SetLook(double, double, SmBoolean = FALSE, SmGfxArraySet * = NULL)              { }
SM_EXPORT inline void   smgfx_GetLook(double*, double*, double*, double*, double*)                                    { }

SM_EXPORT inline double smgfx_GetPointSize()                                                   { return 0.0 ; }             
SM_EXPORT inline double smgfx_SetPointSize(double, SmGfxArraySet * = NULL)             { return 0.0 ; }             
SM_EXPORT inline double smgfx_GetLineWidth()                                                   { return 0.0 ; }
SM_EXPORT inline double smgfx_SetLineWidth(double, SmGfxArraySet * = NULL)              { return 0.0 ; }             

SM_EXPORT inline SmBoolean smgfx_SetDashedLines(SmBoolean, SmGfxArraySet * = NULL)      { return TRUE ; }
SM_EXPORT inline void      smgfx_SetRotationCenter(const SmPoint3d &)           { }                          
SM_EXPORT inline void      smgfx_ZoomBox(double, double, double)                               { }
SM_EXPORT inline void      smgfx_ZoomWorldBox(SmExtent3d &, double=0.9, SmPoint3d * =NULL, SmBoolean=FALSE) { }
SM_EXPORT inline void      smgfx_ZoomOut()                                                     { }    
                      
SM_EXPORT inline void      smgfx_DrawComb
(
  double*, double*, long, double = 1.0, 
  long = 3, 
  long = 3, 
  SmGfxArraySet * = NULL
)                       
{ }

SM_EXPORT inline void      smgfx_DrawCircularArc
(
  const SmPoint3d&, const SmVector3d&, 
  const SmVector3d&, 
  double, double, double, double, 
  SmGfxArraySet* = NULL
)                
{ }

SM_EXPORT inline const SmVector3d &smgfx_GetColor()                                                         { return(smgfx_RefGlobalDisplayParameters().m_TmpColor) ; }

SM_EXPORT inline SmVector3d &smgfx_SetColor   (const SmVector3d &, SmGfxArraySet * = NULL)      { return(smgfx_RefGlobalDisplayParameters().m_TmpColor) ; }
SM_EXPORT inline SmVector3d &smgfx_SetColor   (double, double, double, SmGfxArraySet * = NULL)      { return(smgfx_RefGlobalDisplayParameters().m_TmpColor) ; } 
SM_EXPORT inline SmVector3d &smgfx_ChangeColor(SmBoolean = TRUE, SmGfxArraySet * = NULL) { return(smgfx_RefGlobalDisplayParameters().m_TmpColor) ; }

SM_EXPORT inline void smgfx_SetCurveClassificationObjectLook
(
  double, double, double, 
  double, double, double, 
  double, double, double, 
  double, double, 
  SmGfxArraySet * = NULL 
) 
{ }
                                                                                                                   
SM_EXPORT inline void smgfx_SetDisplayParameters(SmDrawModeType, SmDisplayParameters &) { }
SM_EXPORT inline void smgfx_SetDrawingMode(SmDrawModeType)                             { } 
SM_EXPORT inline void smgfx_SetNormalGain(double)                                { }                         
SM_EXPORT inline void smgfx_SetHiddenCurve(SmBoolean, ULONG)                                 { }
SM_EXPORT inline void smgfx_SetUVHatchCount(ULONG, ULONG)                                    { }

SM_EXPORT inline void smgfx_SetDrawHighCountCurves
(
  SmBoolean, 
  ULONG = 150, 
  double = 6.0, 
  double = 1.0, 
  double = .5, double = .1
)        
{ }

SM_EXPORT inline void smgfx_SetDrawLineCallBack(void (*)(double, double, double, double, double, double, SmGfxArraySet *)) { }
SM_EXPORT inline void smgfx_SetDrawPointCallBack(void (*)(double, double, double, SmGfxArraySet *)) { }
SM_EXPORT inline void smgfx_SetDrawPolyLineCallBack(void (*)(double *, long, SmGfxArraySet *)) { }

#endif // no SM_GFX_CODE
#endif // !__SMGFX_EXTERN_H__
