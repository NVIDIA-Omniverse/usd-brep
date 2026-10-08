// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmGraphicsExtern.cpp
* PURPOSE: Platform specific interface to graphics.  See SmGraphicsOutput.cpp
*    for the generic OpenGL interface.
**********************************************************************/

#include "StdAfx.h"

#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>
#include <SmTArray.h>
#include <SmCoreTypes.h>
#include <SmGraphicsVertexArray.h>
#include <SmDatabaseIO.h>


//// is this available under unix ?
//#include <ios>

static void ByteSwapFloat( float *pF )
{
  union
  {
    float theFloat;
    char  theBytes[sizeof( float )];
  } src, dst;

  src.theFloat = *pF;
  dst.theBytes[0] = src.theBytes[3];
  dst.theBytes[1] = src.theBytes[2];
  dst.theBytes[2] = src.theBytes[1];
  dst.theBytes[3] = src.theBytes[0];
  *pF = dst.theFloat;

} // end ByteSwapFloat


// global variables to control interface display
// Drawing Mode
// Shading State
// CrossHatch State
// HiddenCurve State
// Tessellation State
static SmDisplayParameters s_Disp ;
static SmViewParameters    s_View ;

/*******************************************************************//**
PURPOSE: Access to global SmDisplayParameters s_Disp and s_View objects

NOTES:
***********************************************************************/
SmDisplayParameters &smgfx_RefGlobalDisplayParameters() { return(s_Disp) ; }
SmViewParameters    &smgfx_RefGlobalViewParameters()    { return(s_View) ; }

/*******************************************************************//**
PURPOSE: SmDisplayParameters Default Constructor

NOTES:
***********************************************************************/
SmDisplayParameters::SmDisplayParameters
  (SmDrawModeType eMode,    // in : old school style to set many drawBits at once
   SmBoolean bDoShading)    // in : TRUE = output facetted polygons shaded
                            //      FALSE= output facetted polygons outlined
 : /////////// Drawing Mode ////////////////////
   m_bHiddenCurve(FALSE),
   m_bDrawWireFrame(TRUE),
   m_bDrawFacets(FALSE),
   m_bDoShading(bDoShading),
   m_bDraw3DField(FALSE),
   m_p3DField(NULL),
   m_eTessellator(SM_DM_SMLIB),
   m_bUseEdgeTypeColors(TRUE),
              
   m_bDrawCrossHatch(FALSE),
   m_bVaryCrossHatchColor(FALSE),
   m_bDrawKnots(FALSE),              
   m_bDrawPolygon(FALSE),            
   m_bDrawControlPoints(FALSE),
   m_bDrawNormals(FALSE),  
   m_bDrawSeams(FALSE),
   m_bDrawSingularities(FALSE),
   m_bDrawFlatCorners(FALSE),
   m_bDrawCurvature(FALSE),
   m_bDrawSpeed(FALSE),
   m_bDrawDerivatives(FALSE),
   m_bDrawPinCushion(FALSE),
   m_bDrawParameterization(FALSE),
   m_bDrawLoopuses(FALSE),
   m_bDrawNeighbors(FALSE), 
   m_bDrawMicro(FALSE),
   m_bDrawHighCountCurves(FALSE),

  ///////////////// DrawShells ///////////////////////////////////////
  m_bDrawFaceuse(FALSE),      // TRUE = DrawFace is being called from Faceuse::Draw or SmShell::Draw()
  m_pFaceuse(NULL),           // Shells and Faceuses call DrawFace which knows which FU to draw from this value
  m_dNormalGain(0.0),         // Gain applied to DrawFaceuse normals, 0.0 = guess from size of Shell

   ////////////////////// MicroGraphics ////////////////////
   m_dMicroEdgeParam(SM_BIG_DOUBLE),
   m_lMicroEdgeSampleCount(200),

   ////////////////////// Shading ////////////////////
   m_bFlatShading(FALSE),         
   m_bHiddenPolygon(FALSE), 
   m_sLightDir(0.0, 0.0, 1.0),     // dir[0,0,1] = straight down the viewing axis 
   m_sLightAmbient(0.1, 0.1, 0.1),    // undirected light color and brightness
   m_sLightDiffuse(0.65, 0.65, 0.65), // light color and brightness directed in m_sLightDir direction    
   m_sLightSpecular(0.1, 0.1, 0.06),  // light reflected off of surface normal 
  ////////////////////// Cross Hatch ////////////////////
   m_lCrossHatchUCount(4),      
   m_lCrossHatchVCount(4),       
   m_lCrossHatchWCount(4),
   m_dCrossHatchLineWidth(1),    
   m_dCrossHatchKnotLineWidth(2),
                            
   ////////////////////// Hidden Curve ////////////////////
   m_lHiddenCurveDash(1),       
   m_vHCRView(),                
   m_bPerspective(FALSE),
   m_dHiddenCurveTolerance(0.0),
   m_bDisplaySeams(TRUE),
   m_bDisplaySmoothEdges(TRUE),
                            
   ////////////////////// Tessellation ////////////////////
   m_dCrvTessAngle(8.0),   
   m_dSrfTessAngle(8.0),   
   m_dChordHeight( 0.01), 
   m_dPixelTolerance (25.0),
   m_dMax3DEdge(0.0),              
   m_dMaxAspectRatio( 0.0),         
   m_ePolygonOutputType(SM_PO_TRIANGLES),      
   m_bViewBasedTess(FALSE),
 
   //////////// Gap Display /////////////////////////////
   m_dGapLineWidth(6),
   m_dGapPointSize(8),
   m_GapPointColor(1,0,0), 
   m_VertexEdgeGapColor(1,0,1),
   m_VertexFaceGapColor(0,1,0),
   m_EdgeFaceGapColor(0,1,1),
   m_EdgeEdgeGapColor(0,.5,1),
   m_VertexFaceTrimCurveGapColor(.7,.2,.1),
   m_EdgeFaceTrimCurveGapColor(.2,.7,.1),
   m_InTolGapColor(1,.5,0),
   m_OutTolGapColor(.3,.3,.3),  
    
   ////////////////////// application colors ////////////////////
   m_DefaultColor(0,0,0),                
   m_DefaultShadingColor(.7,.7,.7),      
   m_SpineEdgeColor(0,.5,1),      
   m_WireEdgeColor(0,1,0),               
   m_LaminaEdgeColor(0,0,1),
   m_VaryCrossHatchColor(1,1,0),             
   m_CurveControlPolygonColor(1,0,0),    
   m_SurfaceControlPolygonColor(0,0,1),  
   m_VolumeControlPolygonColor(0,1,1),

   m_SurfaceNormalColor        (1,0,0),          
   m_NegSurfaceNormalColor     (0,0,1),
   m_SurfaceNormalInfiniteColor(0,1,0),
   m_SurfaceNormalSolidColor   (0,0,1),
   m_SurfaceNormalVoidColor    (1,0,0),    
      
   m_KnotColor(0,0,1),
   m_bUseCurvature(TRUE),
   m_CurvatureColor(1,1,0),
   m_Curvature2Color(1,.65,0),
   m_SpeedColor(1,.5,0),
   m_BaseSurfaceColor(.2,.2,.5), 
   m_ExtendedSurfaceColor(.2,.5,.2),
   m_NeighborColor(1,0,0),
   m_NeighborMateColor(0,1,0), 
   m_HighCountCurveColor(1,.5,.1),
   m_ClassificationVertexColor(1,0,0),
   m_ClassificationEdgeColor(0,1,0), 
   m_ClassificationFaceColor(0,1,1),                 
   m_InterruptColor(SM_BIG_DOUBLE,       
                    SM_BIG_DOUBLE,       
                    SM_BIG_DOUBLE),
   m_HotPointColor(1,0,0),
   m_HotPointFillColor(1,.7,0),
   m_HotPointSelectColor(1,0,0),
   m_HotPointHighlightColor(1,0,0),
   m_TrackTargetColor(0,.8,.8),
   m_TrackInfoColor(.6,.6,.6),
   m_TestGeometryColor(0,.8,.1),
   m_OutputColor(0,0,0),
   m_TmpColor(0,0,0), 
   
   ////////////////////// Line and Point sizes ////////////////////
   m_dLineWidth(2.0),  
   m_dPointSize(4.0),  
   m_dHotPointSize(6.0),
   m_bDashedLines(FALSE),
   m_dHighCountCurveWidth(6.0),  
   m_lHighCountCurveLimit(150),

   //////////////// Curvature and Derivative Combs and PinCushions //////////////////////                                               
   m_lSamplePointCount(25),                                               
   m_lUPointCount(10),
   m_lVPointCount(10),
   m_lWPointCount(5),

   m_ePinCushionType(SM_DM_UNIT_NORMAL),
   m_dPinCushionScale(1.0),
   m_dCurvatureScale(-20.0),
   m_d3rdDerivScale(1.0)
             
{ 
   //////////////////////// LastMode ////////////////////////////////
#ifdef SM_GFX_CODE
   smgfx_SetDisplayParameters(eMode, *this) ;
#endif // SM_GFX_CODE
   m_eLastMode = eMode ;

} // end SmDisplayParameters::SmDisplayParameters default constructor

/*******************************************************************//**
PURPOSE: Equality operator for Display Properties

NOTES: 
***********************************************************************/
SmBoolean SmDisplayParameters::operator==
  (const SmDisplayParameters& crOther) 
 const
{
  if (   
      /////////// Drawing Mode ////////////////////
       m_bHiddenCurve            == crOther.m_bHiddenCurve     
      && m_bDrawWireFrame        == crOther.m_bDrawWireFrame
      && m_bDrawFacets           == crOther.m_bDrawFacets 
      && m_bDoShading            == crOther.m_bDoShading
      && m_bDraw3DField          == crOther.m_bDraw3DField
      && m_p3DField              == crOther.m_p3DField
      && m_eTessellator          == crOther.m_eTessellator
      && m_bUseEdgeTypeColors    == crOther.m_bUseEdgeTypeColors
                                
      && m_bDrawCrossHatch       == crOther.m_bDrawCrossHatch
      && m_bVaryCrossHatchColor  == crOther.m_bVaryCrossHatchColor
      && m_bDrawKnots            == crOther.m_bDrawKnots  
      && m_bDrawPolygon          == crOther.m_bDrawPolygon
      && m_bDrawControlPoints    == crOther.m_bDrawControlPoints
      && m_bDrawNormals          == crOther.m_bDrawNormals
      && m_bDrawSeams            == crOther.m_bDrawSeams        
      && m_bDrawSingularities    == crOther.m_bDrawSingularities
      && m_bDrawFlatCorners      == crOther.m_bDrawFlatCorners
      && m_bDrawCurvature        == crOther.m_bDrawCurvature
      && m_bDrawSpeed            == crOther.m_bDrawSpeed
      && m_bDrawDerivatives      == crOther.m_bDrawDerivatives
      && m_bDrawPinCushion       == crOther.m_bDrawPinCushion
      && m_bDrawParameterization == crOther.m_bDrawParameterization
      && m_bDrawLoopuses         == crOther.m_bDrawLoopuses
      && m_bDrawNeighbors        == crOther.m_bDrawNeighbors
      && m_bDrawMicro            == crOther.m_bDrawMicro
      && m_bDrawHighCountCurves  == crOther.m_bDrawHighCountCurves

      ///////////////// DrawShells ///////////////////////////////////////
      && m_bDrawFaceuse          == crOther.m_bDrawFaceuse
      && m_pFaceuse              == crOther.m_pFaceuse
      && m_dNormalGain           == crOther.m_dNormalGain

      ////////////////////// MicroGraphics ////////////////////
      && m_dMicroEdgeParam       == crOther.m_dMicroEdgeParam
      && m_lMicroEdgeSampleCount == crOther.m_lMicroEdgeSampleCount

      ////////////////////// Shading ////////////////////
      && m_bFlatShading          == crOther.m_bFlatShading     
      && m_bHiddenPolygon        == crOther.m_bHiddenPolygon   
      && m_sLightDir             == crOther.m_sLightDir
      && m_sLightAmbient         == crOther.m_sLightAmbient
      && m_sLightDiffuse         == crOther.m_sLightDiffuse
      && m_sLightSpecular        == crOther.m_sLightSpecular
                                                           
      ////////////////////// Cross Hatch ////////////////////
      && m_lCrossHatchUCount     == crOther.m_lCrossHatchUCount
      && m_lCrossHatchVCount     == crOther.m_lCrossHatchVCount
      && m_lCrossHatchWCount     == crOther.m_lCrossHatchWCount
      && SM_ARE_SAME(m_dCrossHatchLineWidth,crOther.m_dCrossHatchLineWidth) 
      && SM_ARE_SAME(m_dCrossHatchKnotLineWidth,crOther.m_dCrossHatchKnotLineWidth)    
                                                           
      ////////////////////// Hidden Curve ////////////////////
      && m_lHiddenCurveDash      == crOther.m_lHiddenCurveDash 
      && m_vHCRView              == crOther.m_vHCRView         
      && m_bPerspective          == crOther.m_bPerspective 
      && SM_ARE_SAME(m_dHiddenCurveTolerance,crOther.m_dHiddenCurveTolerance) 
      && m_bDisplaySeams         == crOther.m_bDisplaySeams  
      && m_bDisplaySmoothEdges   == crOther.m_bDisplaySmoothEdges  

      ////////////////////// Tessellation ////////////////////
      && SM_ARE_SAME(m_dCrvTessAngle,   crOther.m_dCrvTessAngle) 
      && SM_ARE_SAME(m_dSrfTessAngle,   crOther.m_dSrfTessAngle) 
      && SM_ARE_SAME(m_dChordHeight,    crOther.m_dChordHeight) 
      && SM_ARE_SAME(m_dPixelTolerance, crOther.m_dPixelTolerance) 
      && SM_ARE_SAME(m_dMax3DEdge,      crOther.m_dMax3DEdge) 
      && SM_ARE_SAME(m_dMaxAspectRatio, crOther.m_dMaxAspectRatio) 
      && m_ePolygonOutputType         ==        crOther.m_ePolygonOutputType
      && m_bViewBasedTess             ==        crOther.m_bViewBasedTess 
      
      //////////// Gap Display /////////////////////////////
      && m_dGapLineWidth               == crOther.m_dGapLineWidth
      && m_dGapPointSize               == crOther.m_dGapPointSize
      && m_GapPointColor               == crOther.m_GapPointColor     
      && m_VertexEdgeGapColor          == crOther.m_VertexEdgeGapColor
      && m_VertexFaceGapColor          == crOther.m_VertexFaceGapColor
      && m_EdgeFaceGapColor            == crOther.m_EdgeFaceGapColor
      && m_EdgeEdgeGapColor            == crOther.m_EdgeEdgeGapColor
      && m_VertexFaceTrimCurveGapColor == crOther.m_VertexFaceTrimCurveGapColor 
      && m_EdgeFaceTrimCurveGapColor   == crOther.m_EdgeFaceTrimCurveGapColor 
      && m_InTolGapColor               == crOther.m_InTolGapColor 
      && m_OutTolGapColor              == crOther.m_OutTolGapColor

      ////////////////////// application colors ////////////////////
      && m_DefaultColor               == crOther.m_DefaultColor              
      && m_DefaultShadingColor        == crOther.m_DefaultShadingColor       
      && m_SpineEdgeColor             == crOther.m_SpineEdgeColor       
      && m_WireEdgeColor              == crOther.m_WireEdgeColor             
      && m_LaminaEdgeColor            == crOther.m_LaminaEdgeColor 
      && m_VaryCrossHatchColor        == crOther.m_VaryCrossHatchColor          
      && m_CurveControlPolygonColor   == crOther.m_CurveControlPolygonColor  
      && m_SurfaceControlPolygonColor == crOther.m_SurfaceControlPolygonColor
      && m_VolumeControlPolygonColor  == crOther.m_VolumeControlPolygonColor

      && m_SurfaceNormalColor         == crOther.m_SurfaceNormalColor        
      && m_NegSurfaceNormalColor      == crOther.m_NegSurfaceNormalColor     
      && m_SurfaceNormalInfiniteColor == crOther.m_SurfaceNormalInfiniteColor
      && m_SurfaceNormalSolidColor    == crOther.m_SurfaceNormalSolidColor   
      && m_SurfaceNormalVoidColor     == crOther.m_SurfaceNormalVoidColor        

      && m_KnotColor                  == crOther.m_KnotColor
      && m_bUseCurvature              == crOther.m_bUseCurvature
      && m_CurvatureColor             == crOther.m_CurvatureColor           
      && m_Curvature2Color            == crOther.m_Curvature2Color
      && m_SpeedColor                 == crOther.m_SpeedColor
      && m_BaseSurfaceColor           == crOther.m_BaseSurfaceColor         
      && m_ExtendedSurfaceColor       == crOther.m_ExtendedSurfaceColor     
      && m_NeighborColor              == crOther.m_NeighborColor            
      && m_NeighborMateColor          == crOther.m_NeighborMateColor        
      && m_HighCountCurveColor        == crOther.m_HighCountCurveColor      
      && m_ClassificationVertexColor  == crOther.m_ClassificationVertexColor
      && m_ClassificationEdgeColor    == crOther.m_ClassificationEdgeColor  
      && m_ClassificationFaceColor    == crOther.m_ClassificationFaceColor  
      && m_InterruptColor             == crOther.m_InterruptColor
      && m_HotPointColor              == crOther.m_HotPointColor
      && m_HotPointFillColor          == crOther.m_HotPointFillColor
      && m_HotPointSelectColor        == crOther.m_HotPointSelectColor
      && m_TrackTargetColor           == crOther.m_TrackTargetColor
      && m_TrackInfoColor             == crOther.m_TrackInfoColor  

      
      ////////////////////// Line and Point sizes ////////////////////
      && m_dLineWidth                 == crOther.m_dLineWidth 
      && m_dPointSize                 == crOther.m_dPointSize 
      && m_dHotPointSize              == crOther.m_dHotPointSize
      && m_bDashedLines               == crOther.m_bDashedLines
      && m_dHighCountCurveWidth       == crOther.m_dHighCountCurveWidth
      && m_lHighCountCurveLimit       == crOther.m_lHighCountCurveLimit

      //////////////// Curvature and Derivative Combs and PinCushions //////////////////////                                               
      && m_lSamplePointCount          == crOther.m_lSamplePointCount                                               
      && m_lUPointCount               == crOther.m_lUPointCount
      && m_lVPointCount               == crOther.m_lVPointCount
      && m_lWPointCount               == crOther.m_lWPointCount

      && m_ePinCushionType            == crOther.m_ePinCushionType
      && m_dPinCushionScale           == crOther.m_dPinCushionScale
      && m_dCurvatureScale            == crOther.m_dCurvatureScale
      && m_d3rdDerivScale             == crOther.m_d3rdDerivScale
     )
    {
      return TRUE;
    }
  return FALSE;

} // end SmDisplayParameters::operator==

/*******************************************************************//**
PURPOSE: Get the view direction used if looking at the given point.

NOTES: 
***********************************************************************/
SmVector3d SmPolygonOutputCallback::GetViewDirection
  (const SmPoint3d & crPointToLookAt) 
 const
{
  if (m_bPerspective) 
    {
      SmVector3d sViewDir = crPointToLookAt - m_vViewDirectionOrEye;
      if (sViewDir.LengthSquared() < SM_EFF_ZERO_SQ) { SE(SM_ERR); return sViewDir; }
      SE(sViewDir.Unitize());
      return sViewDir;
    }
  return m_vViewDirectionOrEye;

} // end SmPolygonOutputCallback::GetViewDirection

/*******************************************************************//**
PURPOSE: Output wireframe representation of a face's polygons.

NOTES: 
***********************************************************************/
SmStatus SmPolygonOutputCallback::OutputFaceWireframe
 (SmFace                * pFace,                // NotUsed: in : 
  const SmTArray<ULONG> & crEdgeVertexIndicies, // in : Indices into the cr3DPoints array that
                                                //      correspond to start and end points of lines.
                                                //      [0] - start of first edge, [1] - end of first edge
  SmTArray<SmPoint3d>   & cr3DPoints,           // in : 3D points extracted from the polygonal mesh.
  SmGfxArraySet         * pOptGfxSet)           // in : 
{
  SM_REF2(pFace, pOptGfxSet) ;
  // locals
  ULONG ii ;

  // Move the points toward the viewer a little to get them 
  // in front of polygons.
  for (ii=0; ii<cr3DPoints.GetSize(); ii++) 
    {
      SmVector3d sViewVector = GetViewDirection(cr3DPoints[ii]);
      cr3DPoints[ii] = cr3DPoints[ii] - sViewVector * m_dHCRWireOffset;
    }

  // for every
  for (ii=0; ii<crEdgeVertexIndicies.GetSize(); ii++) 
    {
#ifdef SM_GFX_CODE
      ULONG lStart = crEdgeVertexIndicies[ii];
      ii++;
      ULONG lEnd = crEdgeVertexIndicies[ii];
      const SmPoint3d & rSt = cr3DPoints[lStart];
      const SmPoint3d & rEnd = cr3DPoints[lEnd];
      smgfx_OutputLine(rSt.x,rSt.y,rSt.z,rEnd.x,rEnd.y,rEnd.z, pOptGfxSet);
#endif // SM_GFX_CODE
    }

  // all done
  return SM_SUCCESS;

} // end SmPolygonOutputCallback::OutputFaceWireframe

/*******************************************************************//**
PURPOSE: Set HCR Tolerances used in Z-buffer HCR through OpenGL.

NOTES: 
***********************************************************************/
void SmPolygonOutputCallback::SetHCRTolerances
  (double dHCRShrinkFactor,
   double dHCRBackFacingDot,
   double dHCRWireOffset)
{
  m_dHCRShrinkFactor = dHCRShrinkFactor;
  m_dHCRBackFacingDot = dHCRBackFacingDot;
  m_dHCRWireOffset = dHCRWireOffset;

} // end SmPolygonOutputCallback::SetHCRTolerances

/*******************************************************************//**
PURPOSE: massage polygon data depending on draw parameters and 
            pass the tuned polygons on to be output to the graphics engine.

NOTES:
  0. Makes no changes to the rendering pipeline's state values.
  1. move polygon vertices to improve z buffering for hidden line drawing.
  2. make sure all vertices have a common vertex normal for flat shading.
  3. output vertices as a polyline or a sequence of lines as needed for wireframe polygons.

  if(!m_bHiddenLine)
    {
      if(m_bFlatShading) { output polygons with one computed and saved normal vector for all vertices ; }
      else               { output polygons with input data ; }
    }
  else
    {
      if(m_bShadedMode)  { move vertices small amount in normal direction to help z buffering ;
                           if(m_bFlatShading) { output modified polygons with one new normal vector ; }
                           else               { output modified polygons with input normal vectors ; }
                         }
      else               { output polygons as a polyline ; }
    }
***********************************************************************/
SmStatus SmPolygonOutputCallback::OutputPolygon
  (ULONG           lPolygonType,             // in : 0 - Triangle,
                                             //      1 - 4-Sided Planar Polygon
                                             //      2 - N-Sided Planar Polygons
                                             //      3 - Triangle Fan
                                             //      4 - Triangle Strip       
   ULONG           lNumPoints,               // in : number of vertices
   SmPoint3d     * sPoints,                  // in : list of vertex positions 
   SmVector3d    * sNormals,                 // in : list of assocatiated normals for each vertex
   SmPoint2d     * aPolygonUVPoints,         // NotUsed: in : 
   SmSurface     * pSurface,                 // NotUsed: in : surface which generated polygon or NULL
   SmFace        * pFace,                    // NotUsed: in : face owning surface or NULL
   SmGfxArraySet * pOptGfxSet)               // out, opt: container for vertex data, if not Null.
                                             //      Default: Null
{
  SM_REF3(aPolygonUVPoints, pSurface, pFace) ;
  // Accumulate counts, if we're actually outputting to graphics.
  if ( pOptGfxSet == NULL )
    {
      m_lNumVerticesOutput += lNumPoints;  // not correct since vertices are re-used
      //  if (lNumPoints != 3) SER(SM_ERR);
      m_lPolygonsOutputCount ++;
    }

#ifdef SM_GFX_CODE
  // hiddenline - move polygons a small amount to get a better result in the z buffer
  if (m_bHiddenLine) 
    { 
      if (m_bShadedMode) 
        {
          // move vertices in their normal direction a small amount
          for (ULONG i=0; i<lNumPoints; i++) 
            {
              SmVector3d sViewVector = GetViewDirection(sPoints[i]);
              double dOffsetDist     = m_dHCRShrinkFactor;
              if (sViewVector.Dot(sNormals[i]) > m_dHCRBackFacingDot) 
                {
                  dOffsetDist = - dOffsetDist;
                }
              sPoints[i] = sPoints[i] - sNormals[i] * dOffsetDist;
            }

#ifdef FIX_ME
          // move vertices in their normal direction a small amount
          //  after computing normal direction as cross product
          //  of vectors from vertex to vertex neighbors.
          for (ULONG i=2; i<lNumPoints; i++) 
            {
              // get 2 vectors from vertex to vertex neighbors
              SmPoint3d sCorner = sPoints[i-2];
              double dOffsetDist = m_dHCRShrinkFactor;
              if (lPolygonType == 3) 
                { // Triangle Fan
                  sCorner = sPoints[0];
                }
              SmVector3d sVec1 = sPoints[i-1] - sCorner;
              SmVector3d sVec2 = sPoints[i] - sCorner;

              // compute unit-normal vector
              SmVector3d sNorm = sVec1 * sVec2;
              if (sNorm.LengthSquared() > SM_EFF_ZERO_SQ) 
                {
                  sNorm.Unitize();
                }

              // invert normal direction for some corners in a triangle strip
              //   - TODO: might need to do something for other strips and fans as well.
              if(   lPolygonType == 4
                 && (i % 2 == 1)) 
                {
                  sNorm = - sNorm;
                }

              // move vertices a small amount in normal direction - help the z buffer
              sPoints[i-2] = sPoints[i-2] - sNorm * dOffsetDist;
              sPoints[i-1] = sPoints[i-1] - sNorm * dOffsetDist;
              sPoints[i] = sPoints[i] - sNorm * dOffsetDist;
              
              if (i==2) 
                {
                  sPoints[i-2] = sPoints[i-2] - sNorm * 2.0 * dOffsetDist;
                  sPoints[i-1] = sPoints[i-1] - sNorm * 2.0 * dOffsetDist;
                }
            }
#endif // FIX_ME
        } // end shaded branch
      else // not shaded - output as a polyline and if needed as a polygonFan
        {
          smgfx_OutputPolyline(&sPoints[0].x,lNumPoints);
          for (ULONG i=2; i<lNumPoints; i++) 
            {
              SmPoint3d sLast = sPoints[i-2];
              if (lPolygonType == 3) 
                { // PolygonFan
                  sLast = sPoints[0];
                }
              SmPoint3d sFirst = sPoints[i];
              smgfx_OutputLine(sLast.x,sLast.y,sLast.z,sFirst.x,sFirst.y,sFirst.z, pOptGfxSet);
            }
        } // end not shaded branch
    } // end hiddenline branch
#else
  SM_REF3(lPolygonType, sPoints, sNormals);
#endif // SM_GFX_CODE

  // all done
  return SM_SUCCESS;

} // end SmPolygonOutputCallback::OutputPolygon

/*******************************************************************//**
PURPOSE: This methods sends out polygons for an individual face
    in a mesh structure.  Basically it is a list of points and surface
    normals which are indexed by a list of which ones belong to which
    polygons.

NOTES: outputs all the polygons in the input arrays one at a time
***********************************************************************/
SmStatus SmPolygonOutputCallback::OutputMesh
  (SmTArray<ULONG>      & rPolygonVertexCount,   // in : This array will contain the count for the number
                                                 //      of points in each polygon.  Typically this will be
                                                 //      3.  The polygon vertices are accessed using the
                                                 //      following indices.  For example if the first polygon
                                                 //      has 3 vertices and the second has 4.  The first 3 
                                                 //      elements of the indices array will index the points
                                                 //      used for the first polygon and the next four will be 
                                                 //      for the second polygon.    
   SmTArray<ULONG>      & rPolygonVertexIndices, // in : Index for the vertices of each polygon.  
   SmTArray<SmPoint3d>  & rPolygon3DPoints,      // in : 
   SmTArray<SmVector3d> & rSurfaceNormals,       // in : 
   SmTArray<SmPoint2d>  & rPolygonUVPoints,      // NotUsed: in : 
   SmSurface            * pSurface,              // NotUsed: in : 
   SmFace               * pFace,                 // NotUsed: in : 
   SmGfxArraySet        * pOptGfxSet)            // NotUsed: i/o: container for vertex data, if not Null.
                                                 //    Default: Null
{  
  SM_REF4(rPolygonUVPoints, pSurface, pFace, pOptGfxSet) ;
  // locals                                            
  ULONG                lCurrentIndex = 0;

  // for every polygon
  for (ULONG ii=0; ii<rPolygonVertexCount.GetSize(); ii++) 
    {
      ULONG lNumVertex   = rPolygonVertexCount[ii];
      ULONG lPolygonType =   (lNumVertex == 3) ? 0
                           : (lNumVertex == 4) ? 1
                           : (lNumVertex  > 4) ? 2
                           : 99 ;

      // quit for not triangle nor quad polygons
      if(lPolygonType == 99) 
        { SER(SM_ERR); }

      // for every polygon vertex
      for (ULONG jj=0; jj<lNumVertex; jj++) 
        {
          // error - current Index exceeds rPolygonVertexIndices size
          if (lCurrentIndex >= rPolygonVertexIndices.GetSize()) 
            { SER(SM_ERR); }

          // get global index value for next polygon vertex
          ULONG lVertexIndex = rPolygonVertexIndices[lCurrentIndex];

          // increment local vertex index value for next iteration
          lCurrentIndex ++;

          // error - global index value out of bounds for input data arrays
          if(   lVertexIndex >= rPolygon3DPoints.GetSize() 
             || lVertexIndex >= rSurfaceNormals.GetSize()) 
           { SER(SM_ERR); }
        } // end iter every vertex for this polygon

      m_lPolygonsOutputCount ++;
   
    } // end iter ii, every polygone

  // all done
  return SM_SUCCESS;

} // end SmPolygonOutputCallback::OutputMesh

/*******************************************************************//**
PURPOSE: Constructor for SLP output type - opens output file

NOTES: We have changed this to FILE * from using ostream, since that did 
   not permit non-ASCII characters for file/directory names
   We changed back to ostream to permit binary output
***********************************************************************/
SmPolygonSLPOutput::SmPolygonSLPOutput
  (const TCHAR       * cOutputFileName, // in : target file name
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
   SmBoolean         & rbFailure)       // out: FALSE = ok, TRUE = something wrong
 : m_lMode(lMode), 
   m_pLastSurface(NULL),
   m_lSolidCounter(1) 
{
  m_eOutputType = eOutputType;
  
  m_pFout = new std::ofstream;
  std::ofstream & sFout = *m_pFout;

  sFout.open(cOutputFileName, SM_IOS::out );
  sFout.setf(SM_IOS::fixed);
  sFout.precision(8);

  if (!sFout.good()) 
    {
      rbFailure = TRUE;
      SE(SM_ERR); // Unable to open this file for some reason.
      return;
    }

  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("solid data__%06ld\n"),m_lSolidCounter);
  sFout << sBuff;

} // end SmPolygonSLPOutput::SmPolygonSLPOutput constructor

/*******************************************************************//**
PURPOSE: Destructor for SLA output type - closes file

NOTES: 
***********************************************************************/
SmStatus SmPolygonSLPOutput::CompletedPolygonOutput()
{
    std::ofstream & sFout = *m_pFout;
    TCHAR sBuff[SM_TBLOCK_SIZE];
    smos_sprintf(sBuff,_T("endsolid data__%06ld\n"),m_lSolidCounter);
    sFout << sBuff;

    return SM_SUCCESS;

} // end SmPolygonSLPOutput::CompletedPolygonOutput

/*******************************************************************//**
PURPOSE: Output a polygon to the SLA file.

NOTES: 
***********************************************************************/
SmStatus SmPolygonSLPOutput::OutputPolygon
  (ULONG           lPolygonType,     // in :
   ULONG           lNumPoints,       // in :
   SmPoint3d     * sPoints,          // in :
   SmVector3d    * sNormals,         // in :
   SmPoint2d     * sUVPoints,        // NotUsed: in :
   SmSurface     * pSurface,         // NotUsed: in :
   SmFace        * pFace,            // NotUsed: in :
   SmGfxArraySet * pOptGfxSet)       // i/o:
{
  SM_REF3(sUVPoints, pFace, pOptGfxSet) ;
  static ULONG lCount = 0;

  std::ofstream & sFout = *m_pFout;

  if (m_lMode == 2 || (m_lMode == 1 && pSurface != m_pLastSurface)) 
    {
            TCHAR sBuff[SM_TBLOCK_SIZE];
            if (m_pLastSurface != NULL) 
              { // Close out last solid
                smos_sprintf(sBuff,_T("endsolid data__%06ld\n"),m_lSolidCounter);
                sFout << sBuff;
              }
            m_lSolidCounter ++;
            smos_sprintf(sBuff,_T("solid data__%06ld\n"),m_lSolidCounter);
            sFout << sBuff;
            lCount ++;
            SmVector3d sColor(0.8,0.8,0.8);
            if (lCount % 6 == 0) sColor.Set(1.0,0.3,0.3);
            if (lCount % 6 == 1) sColor.Set(0.3,0.3,1.0);
            if (lCount % 6 == 2) sColor.Set(0.3,1.0,1.0);
            if (lCount % 6 == 3) sColor.Set(1.0,0.3,0.3);
            if (lCount % 6 == 4) sColor.Set(0.3,1.0,0.3);
            if (lCount % 6 == 5) sColor.Set(1.0,1.0,0.3);           
            sFout << "  color " << sColor.x << " " << sColor.y << " " << sColor.z << std::endl ;
    }
  m_pLastSurface = pSurface;


  if (lPolygonType == 0) 
    { // Triangle
      if (lNumPoints != 3) SER(SM_ERR);
      SmVector3d sVec1 = sPoints[1] - sPoints[0];
      SmVector3d sVec2 = sPoints[2] - sPoints[1];
      SmVector3d sNorm = sVec1 * sVec2;
      if (sNorm.Dot(sNormals[0]) < -SM_EFF_ZERO_SQRT) 
        {
          SE(SM_ERR);
        }
      SER(OutputTriangle(sPoints[0],sPoints[1],sPoints[2],
                         sNormals[0],sNormals[1],sNormals[2]));
    }
  if (lPolygonType == 1) 
    { // Quadralateral
      if (lNumPoints != 4) SER(SM_ERR);

      sFout << "  facet" << std::endl ;
      sFout << "      normal " << sNormals[0].x << " " << sNormals[0].y << " " << sNormals[0].z << std::endl;
      sFout << "      normal " << sNormals[1].x << " " << sNormals[1].y << " " << sNormals[1].z << std::endl;
      sFout << "      normal " << sNormals[2].x << " " << sNormals[2].y << " " << sNormals[2].z << std::endl;
      sFout << "      normal " << sNormals[3].x << " " << sNormals[3].y << " " << sNormals[3].z << std::endl;
      sFout << "    outerloop" << std::endl;
      sFout << "      vertex " << sPoints[0].x << " " << sPoints[0].y << " " << sPoints[0].z << std::endl;
      sFout << "      vertex " << sPoints[1].x << " " << sPoints[1].y << " " << sPoints[1].z << std::endl;
      sFout << "      vertex " << sPoints[2].x << " " << sPoints[2].y << " " << sPoints[2].z << std::endl;
      sFout << "      vertex " << sPoints[3].x << " " << sPoints[2].y << " " << sPoints[3].z << std::endl;
      sFout << "    endloop" << std::endl;
      sFout << "  endfacet" << std::endl;
  }

  if (lPolygonType == 2) 
    { // N-sided polygon
      SER(SM_ERR); // Right now we don't handle N-sided stuff here
    }

  if (lPolygonType == 3) 
    { // Triangle Fan
      for (ULONG j=2; j<lNumPoints; j++) 
        {
          SER(OutputTriangle(sPoints[0],sPoints[j-1],sPoints[j],
              sNormals[0],sNormals[j-1],sNormals[j]));
        }
    }

  if (lPolygonType == 4) 
    { // Triangle Strip
      for (ULONG j=2; j<lNumPoints; j++) 
        {
          if (j%2 == 0) 
            {
              SER(OutputTriangle(sPoints[j-2],sPoints[j-1],sPoints[j],
                  sNormals[j-2],sNormals[j-1],sNormals[j]));
            }
          else 
            {
              SER(OutputTriangle(sPoints[j-2],sPoints[j],sPoints[j-1],
                  sNormals[j-2],sNormals[j],sNormals[j-1]));
            }
        }
    }

  return SM_SUCCESS;

} // end SmPolygonSLPOutput::OutputPolygon

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmStatus SmPolygonSLPOutput::OutputTriangle
 (const SmPoint3d & crP0,
  const SmPoint3d & crP1,
  const SmPoint3d & crP2,
  const SmVector3d & crN0,
  const SmVector3d & crN1,
  const SmVector3d & crN2)
{
    std::ofstream & sFout = *m_pFout;

  // Check right hand rule to make sure polygon has area.
  SmVector3d sV1 = crP1 - crP0;
  SmVector3d sV2 = crP2 - crP0;
  SmVector3d sV3 = crP2 - crP1;
  SmVector3d sCross = sV1 * sV2;
  // Gives 8 decimal places for tolerance
  double dTol = (SM_EFF_ZERO_SQRT * (1.0 + crP0.GetMaxDimension())) / 1000.0;
  if (sV1.Length() < dTol || sV2.Length() < dTol || sV3.Length() < dTol) {
      return SM_SUCCESS; // Polygon is too small because of coincident 
      // end points.
  }

  sFout << "  facet" << std::endl;
  sFout << "      normal " << crN0.x << " " << crN0.y << " " << crN0.z << std::endl;
  sFout << "      normal " << crN1.x << " " << crN1.y << " " << crN1.z << std::endl;
  sFout << "      normal " << crN2.x << " " << crN2.y << " " << crN2.z << std::endl;
  sFout << "    outerloop" << std::endl;
  sFout << "      vertex " << crP0.x << " " << crP0.y << " " << crP0.z << std::endl;
  sFout << "      vertex " << crP1.x << " " << crP1.y << " " << crP1.z << std::endl;
  sFout << "      vertex " << crP2.x << " " << crP2.y << " " << crP2.z << std::endl;
  sFout << "    endloop" << std::endl;
  sFout << "  endfacet" << std::endl;

  return SM_SUCCESS;

} // end SmPolygonSLPOutput::OutputTriangle

/*******************************************************************//**
PURPOSE: Constructor for SLA output type - opens output file

NOTES: 
***********************************************************************/
SmPolygonSLAOutput::SmPolygonSLAOutput
  (const TCHAR * cOutputFileName,
   SmFileType    eFileType,
   ULONG         lMode,            // 0 - all triangles in single solid in SLA file
                                   // 1 - each face creates separate solid in SLA file
   SmBoolean   & rbFailure)
 : m_lMode(lMode), 
   m_pLastSurface(NULL),
   m_lSolidCounter(1), 
   m_eFileType(eFileType)  
{
  m_eOutputType      = SM_PO_TRIANGLES; // Only triangles for SLA
  m_bGenerateNormals = FALSE; // Don't bother with normals
  
  m_pFout = new std::ofstream;
  std::ofstream & sFout = *(std::ofstream*)m_pFout;

  rbFailure = FALSE;
  // Output Brep to File
  if (eFileType == SM_ASCII) 
    {
      sFout.open(cOutputFileName, SM_IOS::out );
      sFout.setf(SM_IOS::fixed);
      sFout.precision(14);
    }
  else 
    {   // SM_BINARY
      sFout.open(cOutputFileName, SM_IOS::out | SM_IOS_BINARY);
    }

  if ( m_pFout == NULL) 
    {
      rbFailure = TRUE;
      SE(SM_ERR); // Unable to open this file for some reason.
      return;
    }

  char sBuff[SM_TBLOCK_SIZE];

  if (m_eFileType == SM_ASCII) 
    {
      smos_sprintf_char(sBuff, "solid data__%06ld created by SMLib\n", m_lSolidCounter);
      sFout << sBuff;
    }
  else 
    {
      // Must be 80 characters
      for (ULONG i=0; i<80; i++) { sBuff[i] = ' '; }
      smos_sprintf_char(sBuff, "%s" , "STL Binary file created by SMLib");
      sFout.write(sBuff,sizeof(char)*80);
    }

} // end SmPolygonSLAOutput::SmPolygonSLAOutput constructor

/*******************************************************************//**
PURPOSE: Constructor for SLA output type - opens output file

NOTES:
***********************************************************************/
SmPolygonSLAOutput::SmPolygonSLAOutput
 (std::ostream & outputStream,
  SmFileType     eFileType,
  ULONG          lMode,            // 0 - all triangles in single solid in SLA file
                                   // 1 - each face creates separate solid in SLA file
  SmBoolean    & rbFailure)
 : m_lMode(lMode),
   m_pLastSurface(NULL),
   m_lSolidCounter(1),
   m_eFileType(eFileType)
{
  m_bCleanupStream   = false;
  m_eOutputType      = SM_PO_TRIANGLES; // Only triangles for SLA
  m_bGenerateNormals = FALSE; // Don't bother with normals

  m_pFout   = & outputStream;
  rbFailure = FALSE;

  char sBuff[SM_TBLOCK_SIZE];

  if (m_eFileType == SM_ASCII) 
    {
      smos_sprintf_char(sBuff, "solid data__%06ld created by SMLib\n", m_lSolidCounter);
      outputStream << sBuff;
    } 
  else 
    {
      // Must be 80 characters
      for (ULONG i = 0; i<80; i++) { sBuff[i] = ' '; }
      smos_sprintf_char(sBuff, "%s", "STL Binary file created by SMLib");
      outputStream.write(sBuff, sizeof(char) * 80);
    }
} // end SmPolygonSLAOutput::SmPolygonSLAOutput

/*******************************************************************//**
PURPOSE: Destructor for SLA output type - closes file

NOTES: 
***********************************************************************/
SmStatus SmPolygonSLAOutput::CompletedPolygonOutput()
{
  std::ostream & sFout = *m_pFout;

  if (m_eFileType != SM_ASCII) // SM_BINARY
    {
      // Now that we know how many polygons - write each one out in binary format.
      if (m_sVertices.GetSize() != m_lPolygonsOutputCount * 3) 
        { 
          SER(SM_ERR);
        }

      bool          bBigEndian = SmDatabaseIO::IsBigEndian();
      std::uint32_t lPolyCount = (std::uint32_t) m_lPolygonsOutputCount;
      if ( bBigEndian )
      { 
          std::uint32_t temp = SmDatabaseIO::ByteSwap32( lPolyCount );
          lPolyCount = temp;
      }

      sFout.write((char*)&lPolyCount,sizeof(std::uint32_t));

      for (ULONG i=0; i<m_lPolygonsOutputCount; i++) 
        {
          SmPoint3d sPnts[3];
          sPnts[0] = m_sVertices[i*3];
          sPnts[1] = m_sVertices[i*3+1];
          sPnts[2] = m_sVertices[i*3+2];
          SmVector3d sV1 = sPnts[1] - sPnts[0];
          SmVector3d sV2 = sPnts[2] - sPnts[0];
          if (sV1.LengthSquared() < SM_EFF_ZERO_SQ) 
            { SE(SM_ERR);   }
          SER(sV1.Unitize());
          if (sV2.LengthSquared() < SM_EFF_ZERO_SQ) 
            { SE(SM_ERR);   }
          SER(sV2.Unitize());
          SmVector3d sCross = sV1 * sV2;
          double dLength = sCross.LengthSquared();
          if (dLength < SM_EFF_ZERO_SQ) 
            {
              SE(SM_ERR);
            }
          else 
            {
              SER(sCross.Unitize());
            }
          float sNorm[3];
          sNorm[0] = (float)sCross.x;
          sNorm[1] = (float)sCross.y;
          sNorm[2] = (float)sCross.z;

          if ( bBigEndian )
          {
              ByteSwapFloat( &sNorm[0] );
              ByteSwapFloat( &sNorm[1] );
              ByteSwapFloat( &sNorm[2] );
          }
          sFout.write((char*)sNorm,sizeof(float)*3);
          for (ULONG j=0; j<3; j++) 
            {
              float sPt[3];
              sPt[0] = (float)sPnts[j].x;
              sPt[1] = (float)sPnts[j].y;
              sPt[2] = (float)sPnts[j].z;

              if ( bBigEndian )
              {
                  ByteSwapFloat( &sPt[0] );
                  ByteSwapFloat( &sPt[1] );
                  ByteSwapFloat( &sPt[2] );
              }
              sFout.write((char*)sPt,sizeof(float)*3);
            }
          char sBlank[2];
          sBlank[0] = ' ';
          sBlank[1] = ' ';
          sFout.write(sBlank,sizeof(char)*2);
        }
    }
  else 
    {
      sFout << "endsolid STL file created by SMLib" << std::endl ;
    }

  std::ofstream * pFileOut = dynamic_cast<std::ofstream*>( &sFout );

  if ( pFileOut )
  { pFileOut->close(); }

  return SM_SUCCESS;

} // end SmPolygonSLAOutput::CompletedPolygonOutput

/*******************************************************************//**
PURPOSE: Output a polygon to the SLA file.

NOTES: 
***********************************************************************/
SmStatus SmPolygonSLAOutput::OutputPolygon
  (ULONG           lPolygonType,   // NotUsed: in : 
   ULONG           lNumPoints,     // in : 
   SmPoint3d     * sPoints,        // in : 
   SmVector3d    * sNormals,       // NotUsed: in : 
   SmPoint2d     * sUVPoints,      // NotUsed: in : 
   SmSurface     * pSurface,       // NotUsed: in : 
   SmFace        * pFace,          // in : 
   SmGfxArraySet * pOptGfxSet)     // NotUsed: i/o:
{
  SM_REF5(lPolygonType, sNormals, sUVPoints, pFace, pOptGfxSet) ;
  if (lNumPoints != 3) return SM_ERR;

  std::ostream & sFout = *m_pFout;

  if (m_eFileType == SM_ASCII && (m_lMode == 1 && pSurface != m_pLastSurface)) 
    {
      char sBuff[SM_TBLOCK_SIZE];
      if (m_pLastSurface != NULL) 
        { // Close out last solid
          smos_sprintf_char(sBuff, "endsolid data__%06ld\n", m_lSolidCounter);
          sFout << sBuff << std::endl ;
          m_lSolidCounter ++;
          smos_sprintf_char(sBuff, "solid data__%06ld\n", m_lSolidCounter);
          sFout << sBuff << std::endl ;
        }
    }

  m_pLastSurface = pSurface;

  // Check right hand rule to make sure polygon has area.
  SmVector3d sV1 = sPoints[1] - sPoints[0];
  SmVector3d sV2 = sPoints[2] - sPoints[0];
  SmVector3d sV3 = sPoints[2] - sPoints[1];
  SmVector3d sCross = sV1 * sV2;
  double dTol = SM_EFF_ZERO_SQRT * (1.0 + sPoints[0].GetMaxDimension());
  if (sV1.Length() < dTol || sV2.Length() < dTol || sV3.Length() < dTol) 
    {
      return SM_SUCCESS; // Polygon is too small because of coincident 
      // end points.
    }
  SER(sV1.Unitize());
  SER(sV2.Unitize());
  double dLength = sCross.Length();
  if (dLength < SM_EFF_ZERO) 
    {
      SER(SM_ERR); // Polygon is too small - will have a topology problem
      // because vertices don't align.
    }
  SER(sCross.Unitize());
  m_lPolygonsOutputCount ++;
  if (m_eFileType == SM_ASCII) 
    {
      sFout << " facet normal " << sCross.x << " " << sCross.y << " " << sCross.z << std::endl;
      sFout << "  outer loop" << std::endl;
      for (ULONG i=0; i<3; i++) 
        {
          SmPoint3d sPnt = sPoints[i];
          sFout << "   vertex " << sPnt.x << " " << sPnt.y << " " << sPnt.z << std::endl;
        }
      sFout << "  endloop" << std::endl;
      sFout << " endfacet" << std::endl;
    }
  else 
    {
      m_sVertices.Add(sPoints[0]);
      m_sVertices.Add(sPoints[1]);
      m_sVertices.Add(sPoints[2]);
    }
  return SM_SUCCESS;

} // end SmPolygonSLAOutput::OutputPolygon


/*******************************************************************//**
PURPOSE: Output a polygon to a text file.

NOTES:
***********************************************************************/
SmStatus SmPolygonFileOutput::OutputPolygon
 (ULONG           s_lPolygonType,  // NotUsed: in : 
  ULONG           lNumPoints,      // in : 
  SmPoint3d     * sPoints,         // in : 
  SmVector3d    * sNormals,        // in : 
  SmPoint2d     * sUVPoints,       // in : 
  SmSurface     * pSurface,        // NotUsed: in : 
  SmFace        * pFace,           // NotUsed: in : 
  SmGfxArraySet * pOptGfxSet)      // NotUsed: i/o: 
{
  SM_REF4(s_lPolygonType, pSurface, pFace, pOptGfxSet) ;
  fprintf( m_FilePtr, "Triangle [%ld] ************\n", m_lNumberTriangles );

  for(ULONG ii = 0; ii < lNumPoints; ii++)
    fprintf( m_FilePtr,
             "(%lf,%lf,%lf)(%lf,%lf,%lf)(%lf,%lf)\n",
             sPoints[ii].x, sPoints[ii].y, sPoints[ii].z,
             sNormals[ii].x, sNormals[ii].y, sNormals[ii].z,
             sUVPoints[ii].x, sUVPoints[ii].y );

  m_lNumberTriangles++;

  return SM_SUCCESS;
} // end SmPolygonFileOutput::OutputPolygon


#ifdef SM_GFX_CODE


  #include <SmVector3d.h>

////THIS IS WHERE ALL THE GRAPHICS DISPLAY DEFAULTS ARE SET

///////////////// SHADING ///////////////////////////////////

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetShadedMode
 (SmBoolean bShadedDisplay)
{
  s_Disp.m_bDoShading = bShadedDisplay;

} // end smgfx_SetShadedMode

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetShaded
 (SmBoolean bShadedDisplay,
  SmBoolean bFlatShading,
  SmBoolean bHiddenPolygon)
{
  s_Disp.m_bDoShading     = bShadedDisplay;
  s_Disp.m_bFlatShading   = bFlatShading;
  s_Disp.m_bHiddenPolygon = bHiddenPolygon;

} // end smgfx_SetShaded

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetShaded
  (SmBoolean & rbShadedDisplay,
   SmBoolean & rbFlatShading,
   SmBoolean & rbHiddenPolygon)
{
  rbShadedDisplay = s_Disp.m_bDoShading;
  rbFlatShading   = s_Disp.m_bFlatShading;
  rbHiddenPolygon = s_Disp.m_bHiddenPolygon;

} // end smgfx_GetShaded

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetLightDir(double dX, double dY, double dZ)
{
  double dSize = sqrt(dX*dX + dY*dY + dZ*dZ) ;
  if(dSize > SM_EFF_ZERO) { s_Disp.m_sLightDir[0] = dX/dSize ;
                            s_Disp.m_sLightDir[1] = dY/dSize ;
                            s_Disp.m_sLightDir[2] = dZ/dSize ;
                          }
  else                    { s_Disp.m_sLightDir[0] = dX ;
                            s_Disp.m_sLightDir[1] = dY ;
                            s_Disp.m_sLightDir[2] = dZ ;
                          }


} // end smgfx_SetLightDir

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetLightDir
  (double  &dX,
   double  &dY,
   double  &dZ)
{
  dX = s_Disp.m_sLightDir[0] ;
  dY = s_Disp.m_sLightDir[1] ;
  dZ = s_Disp.m_sLightDir[2] ;

} // end smgfx_GetLightDir

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetLightAmbient(double dRed, double dGreen, double dBlue)
{
  double dMax = smos_3Max(dRed, dGreen, dBlue) ;
  if(dMax > 1.0) { s_Disp.m_sLightAmbient[0] = dRed/dMax ;
                   s_Disp.m_sLightAmbient[1] = dGreen/dMax ;
                   s_Disp.m_sLightAmbient[2] = dBlue/dMax ;
                 }
  else           { s_Disp.m_sLightAmbient[0] = dRed ;
                   s_Disp.m_sLightAmbient[1] = dGreen ;
                   s_Disp.m_sLightAmbient[2] = dBlue ;
                 }


} // end smgfx_SetLightAmbient

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetLightAmbient
  (double  &dRed,
   double  &dGreen,
   double  &dBlue)
{
  dRed   = s_Disp.m_sLightAmbient[0] ;
  dGreen = s_Disp.m_sLightAmbient[1] ;
  dBlue  = s_Disp.m_sLightAmbient[2] ;

} // end smgfx_GetLightAmbient

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetLightDiffuse(double dRed, double dGreen, double dBlue)
{
  double dMax = smos_3Max(dRed, dGreen, dBlue) ;
  if(dMax > 1.0) { s_Disp.m_sLightDiffuse[0] = dRed/dMax ;
                   s_Disp.m_sLightDiffuse[1] = dGreen/dMax ;
                   s_Disp.m_sLightDiffuse[2] = dBlue/dMax ;
                 }
  else           { s_Disp.m_sLightDiffuse[0] = dRed ;
                   s_Disp.m_sLightDiffuse[1] = dGreen ;
                   s_Disp.m_sLightDiffuse[2] = dBlue ;
                 }


} // end smgfx_SetLightDiffuse

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetLightDiffuse
  (double  &dRed,
   double  &dGreen,
   double  &dBlue)
{
  dRed   = s_Disp.m_sLightDiffuse[0] ;
  dGreen = s_Disp.m_sLightDiffuse[1] ;
  dBlue  = s_Disp.m_sLightDiffuse[2] ;

} // end smgfx_GetLightDiffuse

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetLightSpecular(double dRed, double dGreen, double dBlue)
{
  double dMax = smos_3Max(dRed, dGreen, dBlue) ;
  if(dMax > 1.0) { s_Disp.m_sLightSpecular[0] = dRed/dMax ;
                   s_Disp.m_sLightSpecular[1] = dGreen/dMax ;
                   s_Disp.m_sLightSpecular[2] = dBlue/dMax ;
                 }
  else           { s_Disp.m_sLightSpecular[0] = dRed ;
                   s_Disp.m_sLightSpecular[1] = dGreen ;
                   s_Disp.m_sLightSpecular[2] = dBlue ;
                 }


} // end smgfx_SetLightSpecular

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetLightSpecular
  (double  &dRed,
   double  &dGreen,
   double  &dBlue)
{
  dRed   = s_Disp.m_sLightSpecular[0] ;
  dGreen = s_Disp.m_sLightSpecular[1] ;
  dBlue  = s_Disp.m_sLightSpecular[2] ;

} // end smgfx_GetLightSpecular

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetDrawFacets
  (SmBoolean bDrawFacets,                // in : 
   SmTessellatorType eTessellatorType)   // in : oneof SM_DM_NLIB
                                         //            SM_DM_SMLIB
{
  s_Disp.m_bDrawFacets  = bDrawFacets ;
  s_Disp.m_eTessellator = eTessellatorType ;

} // end smgfx_SetDrawFacets

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmBoolean smgfx_GetDrawFacets
 (SmTessellatorType *pOptTessellatorType)   
{ 
  if(pOptTessellatorType) { *pOptTessellatorType = s_Disp.m_eTessellator ; }
  return s_Disp.m_bDrawFacets; 

} // end smgfx_GetDrawFacets

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetFaceuseNormalGain
  (double dNormalGain)                   // in : 
{
  s_Disp.m_dNormalGain  = dNormalGain ;

} // end smgfx_SetDrawFacets

/*******************************************************************//**
PURPOSE: Convenience Function to set multiple displayParameter
            global displayParmeter draw bits simultaneously.

NOTES: 
  This function simplifies setting combinations of display bits. 
  However any combination of display bits is allowed
  and can be built not using this function.
***********************************************************************/
void smgfx_SetDrawingMode
  (SmDrawModeType eMode)
{
  smgfx_SetDisplayParameters(eMode, s_Disp) ;
  s_Disp.m_eLastMode = eMode ;

} // end smgfx_SetDrawingMode

/*******************************************************************//**
PURPOSE: Convenience Function to set multiple displayParameter
            draw bits simultaneously

NOTES: 
  This function simplifies setting combinations of display bits. 
  However any combination of display bits is allowed
  and can be built not using this function.
***********************************************************************/
void smgfx_SetDisplayParameters
  (SmDrawModeType        eMode,
   SmDisplayParameters & rDisp)
{
  // ignore some display features:
  //   rDisp.m_bUseEdgeTypeColors
  //   rDisp.m_bDoShading
  //   rDisp.m_bDraw3DField
  //   rDisp.m_p3DField

  // disable the rest
  rDisp.m_bHiddenCurve          = FALSE ;
  rDisp.m_bDrawWireFrame        = FALSE ;
  rDisp.m_bDrawCrossHatch       = FALSE ;
  rDisp.m_bVaryCrossHatchColor  = FALSE ;
  rDisp.m_bDrawFacets           = FALSE ;
  rDisp.m_bDrawNormals          = FALSE ;
  rDisp.m_bDrawSeams            = FALSE ;
  rDisp.m_bDrawSingularities    = FALSE ;
  rDisp.m_bDrawFlatCorners      = FALSE ;
  rDisp.m_bDrawKnots            = FALSE ;
  rDisp.m_bDrawPolygon          = FALSE ;
  rDisp.m_bDrawControlPoints    = FALSE ;
  rDisp.m_bDrawCurvature        = FALSE ;
  rDisp.m_bDrawSpeed            = FALSE ;
  rDisp.m_bDrawDerivatives      = FALSE ;
  rDisp.m_bDrawPinCushion       = FALSE ;
  rDisp.m_bDrawParameterization = FALSE ;
  rDisp.m_bDrawLoopuses         = FALSE ;
  rDisp.m_bDrawNeighbors        = FALSE ;
  rDisp.m_bDrawMicro            = FALSE ;
  rDisp.m_bDrawHighCountCurves  = FALSE ;
  rDisp.m_bDrawTypedShells      = FALSE ;

  // turn on desired displays
  switch(eMode)
    {
      case SM_DM_HIDDENLINE      : rDisp.m_bHiddenCurve     = TRUE; break ;
      case SM_DM_WIREFRAME       : rDisp.m_bDrawWireFrame   = TRUE; break ;
      case SM_DM_CROSSHATCH      : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawCrossHatch       = TRUE; break ;
      case SM_DM_FACETS_NLIB     : rDisp.m_bDrawFacets      = TRUE; rDisp.m_eTessellator          = SM_DM_NLIB; break ;
      case SM_DM_FACETS_SMLIB    : rDisp.m_bDrawFacets      = TRUE; rDisp.m_eTessellator          = SM_DM_SMLIB; break ;
      case SM_DM_NORMALS         : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawNormals          = TRUE; break ;
      case SM_DM_SEAMS           : rDisp.m_bDrawSeams         = TRUE; break ;
      case SM_DM_SINGULARITIES   : rDisp.m_bDrawSingularities = TRUE; break ;
      case SM_DM_FLATCORNERS     : rDisp.m_bDrawFlatCorners = TRUE; break ;
      case SM_DM_KNOTS           : rDisp.m_bDrawCrossHatch  = TRUE; rDisp.m_bDrawKnots            = TRUE; break ;
      case SM_DM_CONTROL_NET     : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawPolygon          = TRUE; break ;
      case SM_DM_CONTROL_POINTS  : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawControlPoints    = TRUE; break ;
      case SM_DM_CURVATRUE       : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawCurvature        = TRUE; break ;
      case SM_DM_SPEED           : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawSpeed            = TRUE; break ;
      case SM_DM_DERIVATIVES     : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawDerivatives      = TRUE; break ;
      case SM_DM_PINCUSHION      : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawPinCushion       = TRUE; break ;
      case SM_DM_PARAMETERIZATION: rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawParameterization = TRUE; break ;
      case SM_DM_LOOPUSES        : rDisp.m_bDrawLoopuses    = TRUE; break ;
      case SM_DM_NEIGHBORS       : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawCrossHatch       = TRUE; 
                                   rDisp.m_bDrawNeighbors   = TRUE; rDisp.m_bDrawParameterization = TRUE; 
                                   rDisp.m_bDrawNormals     = TRUE;                                       break ;
      case SM_DM_FACEUSENEIGHBORS: rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawCrossHatch       = TRUE; break ;
      case SM_DM_HIGHCOUNTCURVES : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawHighCountCurves  = TRUE; break ;
      case SM_DM_BREP_SHELLS     : rDisp.m_bDrawWireFrame   = TRUE; rDisp.m_bDrawTypedShells      = TRUE; break ;
      case SM_DM_CURRENT:          break;    
    } // end switch on eMode

} // end smgfx_SetDisplayParameters

///////////// HIDDEN CURVE ///////////////////////////////

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetHiddenCurve
 (SmBoolean        bHiddenCurve, 
  ULONG            lHiddenCurveDash,  
  SmAxis2Placement vHCRView,
  SmBoolean        bPerspective,
  double           dHiddenCurveTolerance,
  SmBoolean        bDisplaySeams,
  SmBoolean        bDisplaySmoothEdges)
{
  s_Disp.m_bHiddenCurve     = bHiddenCurve;
  s_Disp.m_lHiddenCurveDash = lHiddenCurveDash;
  s_Disp.m_vHCRView         = vHCRView;
  s_Disp.m_bPerspective     = bPerspective;
  s_Disp.m_dHiddenCurveTolerance = dHiddenCurveTolerance;
  s_Disp.m_bDisplaySeams    = bDisplaySeams;
  s_Disp.m_bDisplaySmoothEdges = bDisplaySmoothEdges;

} // end smgfx_SetHiddenCurve

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetHiddenCurve
 (SmBoolean bHiddenCurve, 
  ULONG     lHiddenCurveDash) 
{
  s_Disp.m_bHiddenCurve     = bHiddenCurve;
  s_Disp.m_lHiddenCurveDash = lHiddenCurveDash;

} // end smgfx_SetHiddenCurve

/*******************************************************************//**
PURPOSE: load global hidden curve parameter values into output args

NOTES: 
***********************************************************************/
void smgfx_GetHiddenCurve
  (SmBoolean        & rbHiddenCurve,          // out: TRUE = show hidden curves in SmBrep::OutputGraphics
   ULONG            & rlHiddenCurveDash,      // out:
   SmAxis2Placement & rvHCRView,              // out:
   SmBoolean        & rbPerspective,          // out
   double           & rdHiddenCurveTolerance, // out:
   SmBoolean        & rdDisplaySeams,         // out
   SmBoolean        & rdDisplaySmoothEdges)   // out
{
  rbHiddenCurve           = s_Disp.m_bHiddenCurve;
  rlHiddenCurveDash       = s_Disp.m_lHiddenCurveDash;
  rvHCRView               = s_Disp.m_vHCRView;
  rbPerspective           = s_Disp.m_bPerspective;
  rdHiddenCurveTolerance  = s_Disp.m_dHiddenCurveTolerance;
  rdDisplaySeams          = s_Disp.m_bDisplaySeams;
  rdDisplaySmoothEdges    = s_Disp.m_bDisplaySmoothEdges;

} // end smgfx_GetHiddenCurve

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetHiddenCurve
  (SmBoolean & rbHiddenCurve,          // out:
   ULONG     & rlHiddenCurveDash)      // out:
{
  rbHiddenCurve       = s_Disp.m_bHiddenCurve;
  rlHiddenCurveDash   = s_Disp.m_lHiddenCurveDash;

} // end smgfx_GetHiddenCurve

//////////////// Tessellation /////////////////

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetTessTolerances
  (double              dCrvTessAngle,      // in :
   double              dSrfTessAngle,      // in :
   double              dChordHeight,       // in :
   double              dPixelTolerance,    // in :
   double              dMax3DEdge,         // in :
   double              dMaxAspectRatio,    // in :
   SmPolygonOutputType lPolygonOutputType) // in :
{
  s_Disp.m_dCrvTessAngle = dCrvTessAngle;
  s_Disp.m_dSrfTessAngle = dSrfTessAngle;
  s_Disp.m_dChordHeight  = dChordHeight;
  s_Disp.m_dPixelTolerance  = dPixelTolerance;
  s_Disp.m_dMax3DEdge    = dMax3DEdge;
  s_Disp.m_dMaxAspectRatio = smos_Fabs(dMaxAspectRatio);
  if (dMaxAspectRatio < 0.0) 
    {
      s_Disp.m_bViewBasedTess = TRUE;
    }
  s_Disp.m_ePolygonOutputType = lPolygonOutputType; 

} // end smgfx_SetTessTolerances

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
double smgfx_SetChordHeight
 (double dChordHeight)   // in :
{
  double sRtn           = s_Disp.m_dChordHeight ;
  s_Disp.m_dChordHeight = dChordHeight;
  
  return sRtn ;

} // end smgfx_SetChordHeight

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetTessTolerances
  (double dCrvTessAngle, 
   double dSrfTessAngle)
{
  s_Disp.m_dCrvTessAngle = dCrvTessAngle;
  s_Disp.m_dSrfTessAngle = dSrfTessAngle;

} // end smgfx_SetTessTolerances

/*******************************************************************//**
PURPOSE: Load global tessellation parameter values into output args

NOTES: 
***********************************************************************/
void smgfx_GetTessTolerances
  (double              & rdCrvTessAngle,      // out:
   double              & rdSrfTessAngle,      // out:
   double              & rdChordHeight,       // out:
   double              & rdPixelTolerance,    // out:
   double              & rdMax3DEdge,         // out:
   double              & rdMaxAspectRatio,    // out:
   SmPolygonOutputType & rePolygonOutputType) // out:
{                                             
  rdCrvTessAngle      = s_Disp.m_dCrvTessAngle;
  rdSrfTessAngle      = s_Disp.m_dSrfTessAngle;
  rdChordHeight       = s_Disp.m_dChordHeight;
  rdPixelTolerance    = s_Disp.m_dPixelTolerance;
  rdMax3DEdge         = s_Disp.m_dMax3DEdge;
  rdMaxAspectRatio    = s_Disp.m_dMaxAspectRatio;
  rePolygonOutputType = s_Disp.m_ePolygonOutputType;

} // end smgfx_GetTessTolerances

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetTessTolerances
  (double & rdCrvTessAngle, 
   double & rdSrfTessAngle)
{
  rdCrvTessAngle   = s_Disp.m_dCrvTessAngle;
  rdSrfTessAngle   = s_Disp.m_dSrfTessAngle;

} // end smgfx_GetTessTolerances

//////////////  Cross Hatch /////////////////////////////////////

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetUVHatchCount
  (ULONG lUHatch, 
   ULONG lVHatch)
{
  s_Disp.m_lCrossHatchUCount = lUHatch;
  s_Disp.m_lCrossHatchVCount = lVHatch;

} // end smgfx_SetUVHatchCount

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetUVHatchCount
  (ULONG & rlUHatch,                // out: get number of U dir hatch lines 
   ULONG & rlVHatch)                // out: get number of V dir hatch lines
{
  rlUHatch = s_Disp.m_lCrossHatchUCount;
  rlVHatch = s_Disp.m_lCrossHatchVCount;

} // end smgfx_GetUVHatchCount

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetUVWHatchCount
  (ULONG lUHatch,                  // in : set number of U dir hatch lines
   ULONG lVHatch,                  // in : set number of U dir hatch lines
   ULONG lWHatch)                  // in : set number of W dir hatch lines
{
  s_Disp.m_lCrossHatchUCount = lUHatch;
  s_Disp.m_lCrossHatchVCount = lVHatch;
  s_Disp.m_lCrossHatchWCount = lWHatch;

} // end smgfx_SetUVWHatchCount

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetUVWHatchCount
  (ULONG & rlUHatch,                // out: get number of U dir hatch lines 
   ULONG & rlVHatch,                // out: get number of U dir hatch lines 
   ULONG & rlWHatch)                // out: get number of W dir hatch lines
{
  rlUHatch = s_Disp.m_lCrossHatchUCount;
  rlVHatch = s_Disp.m_lCrossHatchVCount;
  rlWHatch = s_Disp.m_lCrossHatchWCount;

} // end smgfx_GetUVWHatchCount

///////////////// DRAWING MODE /////////////////////

//////////////  KNOTS ////////////////////////////////////

//////////////  PIN CUSHION ////////////////////////////////////

////////////// Draw Polygon and Control Points //////////////////////////////

////////////// Draw Curvature and sample counts //////////////////////////////

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetDrawCurvature(SmBoolean bDrawCurvature, double dCurvatureScale, ULONG lSamplePointCount) 
{ 
  s_Disp.m_bDrawCurvature    = bDrawCurvature ;
  s_Disp.m_dCurvatureScale   = dCurvatureScale ;
  s_Disp.m_lSamplePointCount = lSamplePointCount ; 

} // end smgfx_SetDrawCurvature

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetDrawCurvature(SmBoolean &bDrawCurvature, double &dCurvatureScale, ULONG &lSamplePointCount) 
{
  bDrawCurvature    = s_Disp.m_bDrawCurvature ; 
  dCurvatureScale   = s_Disp.m_dCurvatureScale ;
  lSamplePointCount = s_Disp.m_lSamplePointCount ; 

} // end smgfx_GetDrawCurvature

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetDrawSpeed(SmBoolean bDrawSpeed, double dSpeedScale) 
{ 
  s_Disp.m_bDrawSpeed      = bDrawSpeed ;
  s_Disp.m_dCurvatureScale = dSpeedScale ;

} // end smgfx_SetDrawSpeed

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetDrawSpeed(SmBoolean &bDrawSpeed, double &dSpeedScale) 
{
  bDrawSpeed  = s_Disp.m_bDrawSpeed ; 
  dSpeedScale = s_Disp.m_dCurvatureScale ;

} // end smgfx_GetDrawSpeed

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetCombScales
 (double  dPinCushionScale,  // in : default:[ 10.0]
  double  dCurvatureScale,   // in : default:[-25.0]
  double  d3rdDerivScale)    // in : default:[  1.0]
{ 
  s_Disp.m_dPinCushionScale = dPinCushionScale ;
  s_Disp.m_dCurvatureScale  = dCurvatureScale ;
  s_Disp.m_d3rdDerivScale   = d3rdDerivScale ;

} // end smgfx_SetCombScales

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetCombScales
 (double &dPinCushionScale, 
  double &dCurvatureScale, 
  double &d3rdDerivScale)
{
  dPinCushionScale = s_Disp.m_dPinCushionScale ;
  dCurvatureScale  = s_Disp.m_dCurvatureScale ;
  d3rdDerivScale   = s_Disp.m_d3rdDerivScale ;

} // end smgfx_GetCombScales

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_SetDrawHighCountCurves
  (SmBoolean bDrawHighCountCurves,    // in :       
   ULONG  lHighCountCurveLimit,       // in : default:[150]
   double dHighCountCurveWidth,       // in : default:[6.0]
   double dRed,                       // in : default:[1.0]
   double dGreen,                     // in : default:[0.5]
   double dBlue)                      // in : default:[0.1]
{ 
  s_Disp.m_lHighCountCurveLimit  = lHighCountCurveLimit ;
  s_Disp.m_bDrawHighCountCurves  = bDrawHighCountCurves ;
  s_Disp.m_dHighCountCurveWidth  = dHighCountCurveWidth ;
  s_Disp.m_HighCountCurveColor.Set(dRed, dGreen, dBlue) ;

} // end smgfx_SetDrawHighCountCurves

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmBoolean smgfx_GetDrawHighCountCurves()
{ 
  return(s_Disp.m_bDrawHighCountCurves) ;

} // end smgfx_GetDrawHighCountCurves

/*******************************************************************//**
PURPOSE: Set CurveClassification object colors

NOTES: 
***********************************************************************/
void smgfx_SetCurveClassificationObjectLook
  ( double dLineWidth,          // in : LineWidth in pixels  (commonly 1 to 3)
    double dPointSize,          // in : Point Size in pixels (commonly 1 to 5)
    double dVertexRed, double dVertexGreen, double dVertexBlue,
    double dEdgeRed,   double dEdgeGreen,   double dEdgeBlue,
    double dFaceRed,   double dFaceGreen,   double dFaceBlue,
    SmGfxArraySet *pOptGfxSet)
{
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_SetPointSize(dPointSize, pOptGfxSet) ;
  s_Disp.m_ClassificationVertexColor.Set(dVertexRed, dVertexGreen, dVertexBlue) ;
  s_Disp.m_ClassificationEdgeColor  .Set(dEdgeRed,   dEdgeGreen,   dEdgeBlue  ) ;  
  s_Disp.m_ClassificationFaceColor  .Set(dFaceRed,   dFaceGreen,   dFaceBlue  ) ;  

} // end smgfx_SetCurveClassificationObjectLook

/*******************************************************************//**
PURPOSE: Get CurveClassification object colors

NOTES: 
***********************************************************************/
void smgfx_GetCurveClassificationObjectColors
  (SmVector3d &rVertexColor,
   SmVector3d &rEdgeColor,
   SmVector3d &rFaceColor)
{
  rVertexColor = s_Disp.m_ClassificationVertexColor ;
  rEdgeColor   = s_Disp.m_ClassificationEdgeColor ;
  rFaceColor   = s_Disp.m_ClassificationFaceColor ;

} // end smgfx_GetCurveClassificationObjectColors

///////////////////////////////////////////////////////////////////////////
                                                 

/*******************************************************************//**
PURPOSE: return TRUE when top matrix of the perspective matrix stack
 is an ortho viewing matrix.  OpenGL does not support this
 query so this is a hack function that looks for a patter in
 the matrix.  In general it
 may have problems.

NOTES: 
  ortho matrix is of the form
   [ 2/(right-left)       0              0      tx]
   [      0         2/(top-bottom)       0      ty]
   [      0               0        2/(far-near) tz]
   [      0               0              0      -1]

   tx = - (right+left)/(right-left)
   ty = - (top+bottom)/(top-bottom)
   tz = - (far+near)  /(far-near)
***********************************************************************/
SmBoolean smgfx_IsOrthoView() 
{
  return(1) ;

} // end smgfx_IsOrthoView

/*******************************************************************//**
PURPOSE: set the ortho viewing box center and extent parameters.

NOTES: 
***********************************************************************/
void smgfx_SetViewVolume
  (double dXMid, double dXSize,    // in : new s_View.m_sMid and s_View.m_sSize x components
   double dYMid, double dYSize,    // in : new s_View.m_sMid and s_View.m_sSize y components
   double dZMid, double dZSize)    // in : new s_View.m_sMid and s_View.m_sSize z components
{
  s_View.m_sMid.x  = dXMid;
  s_View.m_sMid.y  = dYMid;
  s_View.m_sMid.z  = dZMid;
  s_View.m_sSize.x = dXSize;
  s_View.m_sSize.y = dYSize;
  s_View.m_sSize.z = dZSize;

} // end smgfx_SetViewVolume

/*******************************************************************//**
PURPOSE: Increase the ortho projection viewing box size by factor of
            two.

NOTES: 
***********************************************************************/
void smgfx_ZoomOut()
{
  // increase the ortho projection viewing box size by a factor of two
  s_View.m_sSize = s_View.m_sSize * 2.0;

} // end smgfx_ZoomOut

/*******************************************************************//**
PURPOSE: reduce the ortho projection viewing box size by factor of
            two.

NOTES: 
***********************************************************************/
void smgfx_ZoomIn()
{
  // decrease the ortho projection viewing box size by a factor of two
  s_View.m_sSize = s_View.m_sSize / 2.0;

} // end smgfx_ZoomIn

/*******************************************************************//**
PURPOSE: Move and scale the ortho viewing box.

NOTES: 
    the values will be interpreted as center being zero and 
    going to 1 in each direction 
    0,0,0.5     - will zoom to box 1/4 size of original with same center
    0.5,0.5,0.5 - will zoom to upper right quadrent
    0,0,2.0     - will zoom out 
***********************************************************************/
void smgfx_ZoomBox
  (double dNewCentXDelta,       // in : normalized ortho box horizontal offset amount in ortho box width units
   double dNewCentYDelta,       // in : normalized ortho box vertical   offset amount in ortho box height units
   double dNewSizeDelta)        // in : scale amount for ortho view box
{
  // display the ortho box center
  s_View.m_sMid.x = s_View.m_sMid.x + dNewCentXDelta * s_View.m_sSize.x;
  s_View.m_sMid.y = s_View.m_sMid.y + dNewCentYDelta * s_View.m_sSize.y;

  // scale the ortho box size
  s_View.m_sSize  = s_View.m_sSize * dNewSizeDelta;

} // end smgfx_ZoomBox

/*******************************************************************//**
PURPOSE: Size and move the view box to center on the given world
  box.

NOTES: 
    
***********************************************************************/
void smgfx_ZoomWorldBox
 (SmExtent3d &rBBox,         // in : box to center on screen                                                                    
  double      dScale,        // in : box view scale, .9 = box takes up about 90% of screen,
                             //      default:[.9]                                      
  SmPoint3d  *pCenter)       // in : normalized BBox pos centered in screen, NULL = [.5, .5, .5]                                
                             //      ex: [.75,.5,.5] BBox is scooted to the left (the bbox pt to the right is centered)
                             //      default:[NULL]                                                                             
{
  // from void SmView::OnViewZoomall() 

  // ensure global bounding box is  set
  if(rBBox.HasNegativeVolume())
    { return ; }

  SmVector3d sViewBBoxSize = rBBox.GetSize();
  SmPoint3d sNormCenter =    pCenter 
                           ? *pCenter
                           : SmPoint3d(.5, .5, .5) ;
  SmPoint3d sViewCenter = rBBox.Evaluate(sNormCenter.x, sNormCenter.y, sNormCenter.z);

  // load SmView ortho box parameters
  double dXMid  = sViewCenter.x;
  double dYMid  = sViewCenter.y;
  double dXSize = (smos_Max(sViewBBoxSize.x, sViewBBoxSize.y)/(2*dScale));
  double dYSize = dXSize ;
  if (dXSize > 300000.0)
    {
      dXSize = 100.0;
      dYSize = 100.0;
      dXMid = 0.0;
      dYMid = 0.0;
    }
#ifdef SM_GFX_CODE

  // side effect: load smgfx SmViewParameters s_View global with ortho box sizes
  smgfx_SetViewVolume(dXMid, dXSize, dYMid, dYSize, 0, 7000.0);
#endif // SM_GFX_CODE 

} // end smgfx_ZoomWorldBox

/*******************************************************************//**
PURPOSE: When using OpenGl (pOptGfxSet == NULL), manage the display list.  
  Open a new display list to accumulate all subsequent gl commands until 
  the matching smgfx_Close() call. Places the DisplayList ID value in the
  file global s_View.m_pActiveLists. Does nothing if a display list 
  is already open.

  When using OpenGL ES (pOptGfxSet != NULL) manage the SmGfxArraySet. 
  Sets the display state stored in pOptGfxSet and returns.  
  OpenGL ES does not use display lists.

NOTES: OpenGL   : Stores in the returned SmDisplayList:
       OpenGL ES: Stores in the SmGfxArraySet: 
          m_lDisplayListId = SM_BIG_ULONG  (an uninitialized value)
          m_dLineWidth     = s_Disp.m_dLineWidth
          m_dPointSize     = s_Disp.m_dPointSize
          m_sColor         = rColor
          m_dHotPointSize  = A magic Number, currently 4 but it might change

RETURNS ---
   for OpenGL   : copy of new or currently open display list object with valid DisplayListID
   for OpenGL ES: display list with DisplayListID = SM_BIG_ULONG
***********************************************************************/
SmDisplayList smgfx_Open              // rtn: return by value not by reference and adds a copy to SmTArray<SmDisplayList> m_pActiveLists
  (const SmVector3d & rColor,         // in : Color stored with new SmDisplayList
                                      //      default:[(0,0,0)]
   double           * pOptLineWidth,  // in : line width to use and store, NULL=use smgfx_GetLineWidth()
                                      //      default:[NULL]
   double           * pOptPointSize,  // in : point size to use and store, NULL=use smgfx_GetPointSize()
                                      //      default:[NULL]
   SmBoolean          bDashedLines,   // in : FALSE=dashed lines turned off before calling display, TRUE=turned on
                                      //      default:[FALSE]      Gets display state set
   SmGfxArraySet    * pOptGfxSet)     // i/o: Gets display state member values set.
                                      //      used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                      //      NULL to ignore. 
                                      //      default:[NULL]
{
  // return value
  SmDisplayList sDisplayList(SM_BIG_ULONG,
                             pOptLineWidth ? *pOptLineWidth : smgfx_GetLineWidth(),
                             pOptPointSize ? *pOptPointSize : smgfx_GetPointSize(),
                             rColor,
                             smgfx_GetHotPointSize()) ;
  sDisplayList.m_bDashedLine = bDashedLines ;

  // when given pOptGfxSet - just set the pOptGfxSet draw list and return
  if(pOptGfxSet)
    {
      // GWC: For nested DisplayLists - output current state
      //      These calls won't affect a system of nonNested Open Calls .
      pOptGfxSet->SetLineWidth((float)sDisplayList.GetLineWidth()) ;
      pOptGfxSet->SetPointSize((float)sDisplayList.GetPointSize()) ;
      pOptGfxSet->SetDashedLines(sDisplayList.GetDashedLine()) ;
      pOptGfxSet->SetRed  ( (float)sDisplayList.GetColor().x) ;
      pOptGfxSet->SetGreen( (float)sDisplayList.GetColor().y) ;
      pOptGfxSet->SetBlue ( (float)sDisplayList.GetColor().z) ;

      // all done
      return sDisplayList ; // return value not reference

    }

  // reset open DisplayList count when its too large
  if(s_View.m_lNumOpened > 10000) 
    { s_View.m_lNumOpened = 0 ; }

  // check state - graphics must be initialized
  if(!s_View.m_bGraphicsInitialized) 
    { return(sDisplayList) ; }  // return value not reference

  // check state - global ActiveLists must be allocated  // RCLxx moved
  if(!s_View.m_pActiveLists) 
    { return(sDisplayList) ; }  // return value not reference

  // fix state problems - if number of m_pAcitveLists is empty make sure m_lNumOpened is set to Zero 
  if(s_View.m_pActiveLists->GetSize() < 1) 
    { s_View.m_lNumOpened = 0 ; }          // RCLxx

  // increment open displayList count
  s_View.m_lNumOpened++;

#ifdef SM_DEBUG_GRAPHICS
    TCHAR sBuff[SM_TBLOCK_SIZE];
    smos_sprintf(sBuff,_T("\nIncrement DisplayList Depth to [%d]"),s_View.m_lNumOpened);
    smos_WriteBuffer(sBuff);
#endif // SM_DEBUG_GRAPHICS 
  
  // GWC: For nested DisplayLists - output current state
  //      These calls won't affect a system of nonNested Open Calls .
  smgfx_OutputLineWidth(sDisplayList.GetLineWidth()) ;
  smgfx_OutputPointSize(sDisplayList.GetPointSize()) ;
  smgfx_OutputDashedLines(sDisplayList.GetDashedLine()) ;
  smgfx_OutputColor(sDisplayList.GetColor()) ;

  // no work - display list already open - return it - it's the last entry to m_pActiveLists
  if (s_View.m_lNumOpened > 1) 
    { 
      return(s_View.m_pActiveLists->GetLast()) ; // return value not reference
    }

  // For a non-nested DisplayList the above calls are not part of the
  // DisplayList stored function sequence.

  // Before outputting the displayList graphics in OpenGL 
  // make sure to run the 4 output commands,
  //      smgfx_OutputPointSize(rDisplayList.GetPointSize()) ;
  //      smgfx_OutputLineWidth(rDisplayList.GetLineWidth()) ;
  //      smgfx_OutputDashedLines(rDisplayList.GetDashedLine()) ;
  //      smgfx_OutputColor(rDisplayList.GetColor()) ;
  // above to get the colors and sizes desired for this display list.
  // This is done so that the size and color can be changed
  // at run-time without having to rebuild the displayList itself.

  
  // all done
  return(sDisplayList) ;  // return value not reference

} // end smgfx_Open

/*******************************************************************//**
PURPOSE:  Closes the currently opened display list or decrements
             the open nesting count and redraws the scene.

NOTES: 
  returns: ptr to s_View.m_pActiveLists[last] when s_View.m_bGraphicsInitialized
           else returns NULL.
           
    s_View.m_pActiveLists[last] is a closed DisplayList when this call
    drops the open nesting count to zero, otherwise it's the open active
    DisplayList.
***********************************************************************/
SmDisplayList *smgfx_Close           // rtn: ref to s_View.m_pActiveLists[last] or NULL when confused
 (SmGfxArraySet    * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                     //      NULL to ignore. default:[NULL]
{
  // init return value
  SmDisplayList *pDisplayList = NULL ;

  // when using pOptGfxSet - just return NULL - no use of DisplayLists
  if(pOptGfxSet)
    {
      return(NULL) ; 
    }

  // check state
  if(s_View.m_lNumOpened == 0) { return(pDisplayList) ; }

  // decrement the count of open gl display lists
  s_View.m_lNumOpened--;

#ifdef SM_DEBUG_GRAPHICS
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\nDecrement DisplayList Depth to [%d]"),s_View.m_lNumOpened);
  smos_WriteBuffer(sBuff);
#endif // SM_DEBUG_GRAPHICS

  // close the list when this call matches an outer most Open() call
  if(s_View.m_lNumOpened == 0) 
    {
      // handle to run code when displayList is closed - no actions for OpenGL
    }

  // fetch the last display list on the s_View.m_pActiveLists when available
  if(!s_View.m_bGraphicsInitialized) 
    { return(pDisplayList) ; }
  else
    { 
      ULONG lLast = s_View.m_pActiveLists->GetSize()-1 ;

      // only return a display list when the outer most Open() is closed
      pDisplayList = &((s_View.m_pActiveLists->GetDataArray())[lLast]) ;
    }

  // all done
  return(pDisplayList) ;

} // end smgfx_Close

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_DrawPoint
  (double          dX,         // in : X of point[xyz]
   double          dY,         // in : Y of point[xyz] 
   double          dZ,         // in : Z of point[xyz] 
   SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                               //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;
  smgfx_OutputPointSize(s_Disp.m_dPointSize, pOptGfxSet);

  smgfx_OutputPoint(dX,dY,dZ, pOptGfxSet);

//    smgfx_RefreshColor();

} // end smgfx_DrawPoint

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_DrawPlane
  (const SmPoint3d  &rPlanePoint,    // in : Point on plane
   const SmVector3d &rPlaneNormal,   // in : Normal to plane
   SmGfxArraySet    *pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                     //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;

  smgfx_OutputPointSize  (s_Disp.m_dPointSize, pOptGfxSet);
  smgfx_OutputDashedLines(s_Disp.m_bDashedLines, pOptGfxSet) ;
  smgfx_OutputLineWidth  (s_Disp.m_dLineWidth, pOptGfxSet);

  // plane point
  smgfx_OutputPoint(rPlanePoint.x, rPlanePoint.y, rPlanePoint.z, pOptGfxSet);

  // plane normal
  smgfx_OutputLine(rPlanePoint.x, rPlanePoint.y, rPlanePoint.z,
                   rPlanePoint.x+rPlaneNormal.x, 
                   rPlanePoint.y+rPlaneNormal.y, 
                   rPlanePoint.z+rPlaneNormal.z, pOptGfxSet);

  // check input
  double dNormSizeSquared = rPlaneNormal.LengthSquared() ;
  if(dNormSizeSquared < SM_EFF_ZERO_SQ * 100)
    { return ; }

  // get orthogonal vectors
  SmVector3d sX, sY, sZ ;
  rPlaneNormal.MakeUnitOrthoVectors(NULL, sZ, sX, sY) ;

  // some plane graphics
  ULONG ii ;
  double daPoints[15] ;
  for(ii=0;ii<3;ii++)
    {
      double dScale = (ii+1) * (ii+1) * (ii+1) ;
      SmPoint3d *pTgt0 = (SmPoint3d *)daPoints ;
      SmPoint3d *pTgt1 = (SmPoint3d *)(daPoints+3) ;
      *pTgt0 = rPlanePoint + dScale * (sX + sY) ; 
      *pTgt1 = *pTgt0 - 2 * dScale * sX ; pTgt0++ ; pTgt1++ ;
      *pTgt1 = *pTgt0 - 2 * dScale * sY ; pTgt0++ ; pTgt1++ ;
      *pTgt1 = *pTgt0 + 2 * dScale * sX ; pTgt0++ ; pTgt1++ ;
      *pTgt1 = *pTgt0 + 2 * dScale * sY ; pTgt0++ ; pTgt1++ ;

      smgfx_OutputPolyline(daPoints, 5, 3, pOptGfxSet) ;
    } // end iter plane graphics

  smgfx_OutputDashedLines(FALSE, pOptGfxSet) ;

} // end smgfx_DrawPlane 

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_DrawHotPoint
  (double         dX,         // in : X of point[xyz]
   double         dY,         // in : Y of point[xyz]
   double         dZ,         // in : Z of point[xyz]
   SmGfxArraySet *pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                              //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;
  smgfx_OutputPointSize(s_Disp.m_dHotPointSize, pOptGfxSet);

  smgfx_OutputPoint(dX, dY, dZ, pOptGfxSet);

} // end smgfx_DrawHotPoint

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_GetLook
 (double * dLineWidth, // out:
  double * dPointSize, // out:
  double * dRed,       // out:
  double * dGreen,     // out:
  double * dBlue)      // out:
{
  SmVector3d sColor = smgfx_GetColor() ;
  *dLineWidth = smgfx_GetLineWidth() ;
  *dPointSize = smgfx_GetPointSize() ;
  *dRed       = sColor.x ;
  *dGreen     = sColor.y ;
  *dBlue      = sColor.z ;

} // end smgfx_GetPointSize

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
double smgfx_GetPointSize()
{
  return s_Disp.m_dPointSize ;

} // end smgfx_GetPointSize

/*******************************************************************//**
PURPOSE:  Set Global PointSize value

NOTES:  Outputs the PointSize and returns last PointSize value
***********************************************************************/
double smgfx_SetPointSize
 (double dPointSize,
  SmGfxArraySet *pOptGfxSet)  // in : used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                              //      NULL to ignore. default:[NULL]
{
  SM_REF2(dPointSize, pOptGfxSet);
  double dLastPointSize = 2.0 ;

  return(dLastPointSize) ;

} // end smgfx_SetPointSize

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
double smgfx_GetGapPointSize()
{
  return s_Disp.m_dGapPointSize ;
}

/*******************************************************************//**
PURPOSE:  Set Global GapPointSize value

NOTES:  Outputs the GapPointSize and returns last PointSize value
***********************************************************************/
double smgfx_SetGapPointSize
 (double          dGapPointSize,
  SmGfxArraySet * pOptGfxSet)
{
  SM_REF2(dGapPointSize, pOptGfxSet);
  double dLastGapPointSize = 2.0 ;


  return(dLastGapPointSize) ;

} // end smgfx_SetGapPointSize

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
double smgfx_GetGapLineWidth()
{
  return s_Disp.m_dGapLineWidth ;

} // end smgfx_GetGapLineWidth

/*******************************************************************//**
PURPOSE:  Set Global GapLineWidth value

NOTES:  Outputs the GapLineWidth and returns last LineWidth value
***********************************************************************/
double smgfx_SetGapLineWidth(double dGapLineWidth, SmGfxArraySet * pOptGfxSet)
{
  SM_REF2(dGapLineWidth, pOptGfxSet);
  double dLastGapLineWidth = 2.0 ;


  return(dLastGapLineWidth) ;

} // end smgfx_SetGapLineWidth

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
double smgfx_GetHotPointSize()
{
  return s_Disp.m_dHotPointSize ;
}

/*******************************************************************//**
PURPOSE:  Set Global HotPointSize value

NOTES:  Outputs the HotPointSize and returns last PointSize value
***********************************************************************/
double smgfx_SetHotPointSize(double dHotPointSize, SmGfxArraySet * pOptGfxSet)
{
  SM_REF2(dHotPointSize, pOptGfxSet);
  double dLastPointSize = 2.0 ;

  return(dLastPointSize) ;
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
double smgfx_GetLineWidth()
{ 
  return s_Disp.m_dLineWidth;

} // end smgfx_GetLineWidth

/*******************************************************************//**
PURPOSE:   Set Global LineWidth value

NOTES:  Outputs the LineWidth and returns last LineWidth value

***********************************************************************/
double smgfx_SetLineWidth(double dLineWidth, SmGfxArraySet * pOptGfxSet)
{
  SM_REF2(dLineWidth, pOptGfxSet);
  double dLastLineWidth = 1.0 ;


  return(dLastLineWidth) ;

} // end smgfx_SetLineWidth

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
ULONG smgfx_GetHighCountCurveLimit()
{ 
  return s_Disp.m_lHighCountCurveLimit;

} // end smgfx_GetHighCountCurveLimit

/*******************************************************************//**
PURPOSE:   Set Global LineWidth value

NOTES:  Outputs the LineWidth and returns last LineWidth value

***********************************************************************/
ULONG smgfx_SetHighCountCurveLimit(ULONG lHighCountCurveLimit)
{
  SM_REF1(lHighCountCurveLimit);
  ULONG lLastHighCountCurveLimit = 150 ;


  return(lLastHighCountCurveLimit) ;

} // end smgfx_SetHighCountCurveLimit

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
double smgfx_GetHighCountCurveWidth()
{ 
  return s_Disp.m_dHighCountCurveWidth;

} // end smgfx_GetHighCountCurveWidth

/*******************************************************************//**
PURPOSE:   Set Global LineWidth value

NOTES:  Outputs the LineWidth and returns last LineWidth value

***********************************************************************/
double smgfx_SetHighCountCurveWidth(double dHighCountCurveWidth)
{
  SM_REF1(dHighCountCurveWidth);
  double dLastHighCountCurveWidth = 6.0 ;


  return(dLastHighCountCurveWidth) ;

} // end smgfx_SetHighCountCurveWidth

/*******************************************************************//**
PURPOSE: Compute and retrieve color for given Object/ColorRule pair.

NOTES:
  When given an Object return Color for that Object/ColorRule pair
  else if InterruptColor is Set, return InterruptColor
  else return DefaultColor.
***********************************************************************/
const SmVector3d &smgfx_GetRuleColor
  (const SmObject *pObject,       // in : target object
   SmColorRuleType eColorRule)    // in : Color rule to apply
{
  return(  pObject                   ? pObject->GetObjectColor(eColorRule)
         : smgfx_AnyInterruptColor() ? smgfx_GetColor() 
         :                             smgfx_GetDefaultColor()) ;

} // end smgfx_GetRuleColor

/*******************************************************************//**
PURPOSE: convenience to set 3 draw state parameters with 1 call

NOTES:
***********************************************************************/
void smgfx_SetLook             // eff: convenience to set 3 things at once
  (double dLineWidth,          // in : LineWidth in pixels  (commonly 1 to 3)
   double dPointSize,          // in : Point Size in pixels (commonly 1 to 5)
   double dRed,                // in : red   color element from 0.0 to 1.0
   double dGreen,              // in : green color element from 0.0 to 1.0
   double dBlue,               // in : blue  color element from 0.0 to 1.0
   SmGfxArraySet * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                               //      NULL to ignore. default:[NULL]
{
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_SetPointSize(dPointSize, pOptGfxSet) ;
  smgfx_SetColor(dRed, dGreen, dBlue, pOptGfxSet) ;

} // end smgfx_SetLook

/*******************************************************************//**
PURPOSE: convenience to set 4 draw state parameters with 1 call

NOTES:
***********************************************************************/
double smgfx_SetLook           // eff: convenience to set 3 things at once
  (double dLineWidth,          // in : LineWidth in pixels  (commonly 1 to 3)
   double dPointSize,          // in : Point Size in pixels (commonly 1 to 5)
   double dRed,                // in : red   color element from 0.0 to 1.0
   double dGreen,              // in : green color element from 0.0 to 1.0
   double dBlue,               // in : blue  color element from 0.0 to 1.0
   double dChordHeight,        // in : Chordheight for curve and surface tessellations - default:[.1]
   SmGfxArraySet * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                               //      NULL to ignore. default:[NULL]
{
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_SetPointSize(dPointSize, pOptGfxSet) ;
  smgfx_SetColor(dRed, dGreen, dBlue, pOptGfxSet) ;
  return(smgfx_SetChordHeight(dChordHeight)) ;

} // end smgfx_SetLook

/*******************************************************************//**
PURPOSE: convenience to set 3 draw state parameters with 1 call

NOTES:
***********************************************************************/
void smgfx_SetLook             // eff: convenience to set 3 things at once      
  (double dLineWidth,          // in : LineWidth in pixels  (commonly 1 to 3)
   double dPointSize,          // in : Point Size in pixels (commonly 1 to 5)
   const SmVector3d &rRGB,     // in : red   color element from 0.0 to 1.0   
                               //      green color element from 0.0 to 1.0   
                               //      blue  color element from 0.0 to 1.0   
   SmGfxArraySet * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                               //      NULL to ignore. default:[NULL]
{                                                                           
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_SetPointSize(dPointSize, pOptGfxSet) ;
  smgfx_SetColor(rRGB, pOptGfxSet) ;

} // end smgfx_SetLook

/*******************************************************************//**
PURPOSE: convenience to set 3 draw state parameters with 1 call
            and clear the interupt color so that objects can
            be drawn with their attribute, property or default colors

NOTES:
***********************************************************************/
void smgfx_SetLook                // eff: convenience to set 3 things at once      
  (double    dLineWidth,          // in : LineWidth in pixels  (commonly 1 to 3)
   double    dPointSize,          // in : Point Size in pixels (commonly 2 to 5)
   SmBoolean bChangeColor,        // in : smgfx_ChangeColor(TRUE ) = increment color seq as: red - yellow - magenta - green - blue - cyan - red . . .
                                  //      smgfx_ChangeColor(FALSE) = init Color sequence (to red)
                                  //      default:[TRUE]
   SmGfxArraySet  * pOptGfxSet)   // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                  //      NULL to ignore. default:[NULL]
{                                                                           
  smgfx_SetLineWidth(dLineWidth,   pOptGfxSet) ;
  smgfx_SetPointSize(dPointSize,   pOptGfxSet) ;
  smgfx_ChangeColor (bChangeColor, pOptGfxSet) ; 

} // end smgfx_SetLook

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmBoolean smgfx_GetDashedLines()
{
  return(s_Disp.m_bDashedLines) ;
}

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmBoolean smgfx_SetDashedLines
 (SmBoolean bDashedLines,
  SmGfxArraySet * pOptGfxSet)
{
  SM_REF2(bDashedLines, pOptGfxSet);
  SmBoolean bLastDashedLines = FALSE ;


  return(bLastDashedLines) ;

} // end smgfx_SetDashedLines

/*******************************************************************//**
PURPOSE: Set Default InTol Gap function Color

NOTES: 
***********************************************************************/
const SmVector3d smgfx_SetInTolGapColor(SmVector3d &rInTolGapCol) 
{
  SmVector3d sRtnColor = smgfx_GetInTolGapColor() ;
  s_Disp.m_InTolGapColor = rInTolGapCol ;
  return( sRtnColor ) ; 
}

/*******************************************************************//**
PURPOSE: Set Default OutTol Gap function Color

NOTES: 
***********************************************************************/
const SmVector3d smgfx_SetOutTolGapColor(SmVector3d &rOutTolGapCol) 
{
  SmVector3d sRtnColor = smgfx_GetOutTolGapColor() ;
  s_Disp.m_OutTolGapColor = rOutTolGapCol ;
  return( sRtnColor ) ; 
} // end smgfx_SetOutTolGapColor

/*******************************************************************//**
PURPOSE: Set Default SurfaceCurvature Color for curvature comb reports
            pointing into the screen

NOTES: returns previous default color
***********************************************************************/
const SmVector3d &smgfx_SetCurvatureColor
  (double dRed, 
   double dGreen, 
   double dBlue) 
{
  // remember old set color
  s_Disp.m_TmpColor = s_Disp.m_InterruptColor ;

  // set new color
  s_Disp.m_CurvatureColor.x = dRed ;
  s_Disp.m_CurvatureColor.y = dGreen ;
  s_Disp.m_CurvatureColor.z = dBlue ;

  // all done
  return(s_Disp.m_TmpColor) ;
}

/*******************************************************************//**
PURPOSE: Set Default SurfaceCurvature2 Color for curvature comb reports
            pointing into the screen

NOTES: returns previous default color
***********************************************************************/
const SmVector3d &smgfx_SetCurvature2Color
  (double dRed, 
   double dGreen, 
   double dBlue) 
{
  // remember old set color
  s_Disp.m_TmpColor = s_Disp.m_InterruptColor ;

  // set new color
  s_Disp.m_Curvature2Color.x = dRed ;
  s_Disp.m_Curvature2Color.y = dGreen ;
  s_Disp.m_Curvature2Color.z = dBlue ;

  // all done
  return(s_Disp.m_TmpColor) ;
}

/*******************************************************************//**
PURPOSE: Set Default SurfaceSpeed Color for Speed comb reports

NOTES: returns previous default color
***********************************************************************/
const SmVector3d &smgfx_SetSpeedColor
  (double dRed, 
   double dGreen, 
   double dBlue) 
{
  // remember old set color
  s_Disp.m_TmpColor = s_Disp.m_InterruptColor ;

  // set new color
  s_Disp.m_SpeedColor.x = dRed ;
  s_Disp.m_SpeedColor.y = dGreen ;
  s_Disp.m_SpeedColor.z = dBlue ;

  // all done
  return(s_Disp.m_TmpColor) ;
}

/*******************************************************************//**
PURPOSE: Set Current Interrupt Color, return reference to
            temporary color whose value is the last SetColor.

NOTES: The temporary color changes all the time so 
            store its values if you plan on using it later.
       
    SmVector3d sLastColor = smgfx_SetColor(r,g,b) ;
***********************************************************************/
SmVector3d &smgfx_SetColor
 (double dRed, 
  double dGreen, 
  double dBlue,
  SmGfxArraySet * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                              //      NULL to ignore. default:[NULL]
    
{
  // remember old set color
  s_Disp.m_TmpColor = s_Disp.m_InterruptColor ;

  // set new color
  s_Disp.m_InterruptColor.x = dRed ;
  s_Disp.m_InterruptColor.y = dGreen ;
  s_Disp.m_InterruptColor.z = dBlue ;

  // output interrupt or default color
  s_Disp.m_OutputColor =   smgfx_AnyInterruptColor()
                         ? s_Disp.m_InterruptColor
                         : s_Disp.m_DefaultColor ;

  smgfx_OutputColor(s_Disp.m_OutputColor, pOptGfxSet) ; 

  // all done
  return(s_Disp.m_TmpColor) ;

} // end smgfx_SetColor

/*******************************************************************//**
PURPOSE: Set Current Interrupt Color, return reference to
            temporary color whose value is the last SetColor.

NOTES: The temporary color changes all the time so 
            store its values if you plan on using it later.
       
    SmVector3d sLastColor = smgfx_SetColor(sNewColor) ;

***********************************************************************/
SmVector3d &smgfx_SetColor
  (const SmVector3d & rRGB,
   SmGfxArraySet    * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                  //      NULL to ignore. default:[NULL]
 
{
  return(smgfx_SetColor(rRGB.x, rRGB.y, rRGB.z, pOptGfxSet)) ;

} // end smgfx_SetColor

/*******************************************************************//**
PURPOSE: Change InterruptColor - output new InterruptColor to display list
            - return temporary reference to previous SetColor() color.

NOTES: The temporary color changes all the time so 
            store its values if you plan on using it later.
       
            SmVector3d sLastColor = smgfx_SetColor(sNewColor) ;
***********************************************************************/
SmVector3d &smgfx_ChangeColor
  (SmBoolean       bChangeColor,   // in : TRUE  = increment color seq as: red - yellow - magenta - green - blue - cyan - red . . .
                                   //      FALSE = init color sequence (to red)
                                   //      default:[TRUE]
   SmGfxArraySet * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
{
  static ULONG lCount = 0;
  if(bChangeColor)  { lCount = (lCount+1) % 6 ; }
  else              { lCount = 0 ; }
  switch(lCount)
    { default:
      case 0 : return(smgfx_SetColor(1, 0, 0, pOptGfxSet)) ;
      case 1 : return(smgfx_SetColor(1, 0, 1, pOptGfxSet)) ;
      case 2 : return(smgfx_SetColor(0, 1, 0, pOptGfxSet)) ;
      case 3 : return(smgfx_SetColor(0, 0, 1, pOptGfxSet)) ;
      case 4 : return(smgfx_SetColor(0, 1, 1, pOptGfxSet)) ;
      case 5 : return(smgfx_SetColor(1,.5, 0, pOptGfxSet)) ;
    }

} // end smgfx_ChangeColor

/*******************************************************************//**
PURPOSE: Clear InterruptColor - output DefaultColor to displayList
            - return temporary reference to previous SetColor() color.

NOTES: 
   Clearing the interrupt color changes the behavior of
   smgfx_OutputObjectColor() calls used within OutputGraphics functions
   to select colors for Objects being drawn.  Objects drawn with an
   unCleared interrupt color use that color.  When the interrupt color
   is cleared, the color used is taken from either a color attribute
   or the default color.

   The temporary color changes all the time so 
   store its values if you plan on using it later.
       
   SmVector3d sLastColor = smgfx_SetColor(sNewColor) ;

***********************************************************************/
SmVector3d &smgfx_ClearColor
 (SmGfxArraySet * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                              //      NULL to ignore. default:[NULL]

{
  return(smgfx_SetColor(SM_BIG_DOUBLE, SM_BIG_DOUBLE, SM_BIG_DOUBLE, pOptGfxSet)) ;

} // end smgfx_ClearColor

/*******************************************************************//**
PURPOSE: Set Current Default Color, return reference to
            temporary color whose value is the last SetColor.

NOTES: Changing the default color does not output a color
       to the displayList.  Its something like adding a color attribute 
       to an object.  The calls affect won't be seen until the next 
       smgfx_OutputObjectColor() call made by an OutputGraphics
       method; the default color will be used if no Interrupt 
       nor attribute color  can be found.
***********************************************************************/
SmVector3d &smgfx_SetDefaultColor
  (double dRed, 
   double dGreen, 
   double dBlue) 
{
  s_Disp.m_TmpColor = s_Disp.m_DefaultColor ;

  s_Disp.m_DefaultColor.x = dRed ;
  s_Disp.m_DefaultColor.y = dGreen ;
  s_Disp.m_DefaultColor.z = dBlue ;

  return(s_Disp.m_TmpColor) ;

} // end smgfx_SetDefaultColor

/*******************************************************************//**
PURPOSE: Set Current Default Color, return reference to
            temporary color whose value is the last SetColor.

NOTES: The temporary color changes all the time so 
            store its values if you plan on using it later.
       
    SmVector3d sLastDefaultColor = smgfx_SetDefaultColor(sNewColor) ;

***********************************************************************/
SmVector3d &smgfx_SetDefaultColor
 (const SmVector3d &rRGB) 
{
  s_Disp.m_TmpColor = s_Disp.m_DefaultColor ;

  s_Disp.m_DefaultColor = rRGB ;

  return(s_Disp.m_TmpColor) ;

} // end smgfx_SetDefaultColor

// 
// /*******************************************************************//**
// PURPOSE:  OUtput last color saved in m_OutputColor
// 
// NOTES:  Makes no change to the colorStack
// ***********************************************************************/
// void smgfx_RefreshColor()
// {
// // gwc:this is not a healthy function - it's used at the bottom of the
// // draw functions to output color overriding the color choices made
// // in the OutputGraphics functions.  I've disabled this for now.
//   smgfx_OutputColor(s_Disp.m_OutputColor) ;
// }

/*******************************************************************//**
PURPOSE: Return TRUE if the interrupt color is set
            and should be used to override the normal object 
            color section, else return FALSE

NOTES: 
***********************************************************************/
SmBoolean smgfx_AnyInterruptColor() 
{
  return(  (s_Disp.m_InterruptColor.x == SM_BIG_DOUBLE)
         ? FALSE
         : TRUE) ;

} // end smgfx_AnyInterruptColor

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_DrawLine
 (double dX1, double dY1, double dZ1, // in : Start point(xyz)
  double dX2, double dY2, double dZ2, // in : End   point(xyz)
  SmGfxArraySet *pOptGfxSet)          // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                      //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;
  smgfx_OutputDashedLines(s_Disp.m_bDashedLines, pOptGfxSet) ;
  smgfx_OutputLineWidth(s_Disp.m_dLineWidth, pOptGfxSet);

  smgfx_OutputLine(dX1,dY1,dZ1, dX2,dY2,dZ2, pOptGfxSet);
      
  smgfx_OutputDashedLines(FALSE, pOptGfxSet) ; 

} // end smgfx_DrawLine

/*******************************************************************//**
PURPOSE:  Draw line between 2 points

NOTES: 
***********************************************************************/
void smgfx_DrawLine
  (SmPoint3d     * P0,         // in : Start point
   SmPoint3d     * P1,         // in : End   point
   SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                               //      NULL to ignore. default:[NULL]
                    
{
  if (!s_View.m_bGraphicsInitialized) return;
  smgfx_OutputDashedLines(s_Disp.m_bDashedLines, pOptGfxSet) ;
  smgfx_OutputLineWidth(s_Disp.m_dLineWidth, pOptGfxSet);

  smgfx_OutputLine(P0->x,P0->y,P0->z, P1->x,P1->y,P1->z, pOptGfxSet);
  
  smgfx_OutputDashedLines(FALSE, pOptGfxSet) ;

} // end smgfx_DrawLine

/*******************************************************************//**
PURPOSE: Draws a sequenced of points as a dashed line

NOTES:  npts = number of 3d points in array *pts = [xyz0, xyz1, ... xyzN-1]
***********************************************************************/
void smgfx_DrawPolyline
 (double        * pts,       // in : Polyline's ordered point list
  long            npts,      // in : Number of points in polyline
  SmGfxArraySet * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                             //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;
  smgfx_OutputDashedLines(s_Disp.m_bDashedLines, pOptGfxSet) ;
  smgfx_OutputLineWidth(s_Disp.m_dLineWidth, pOptGfxSet);

  smgfx_OutputPolyline(pts, npts, 3, pOptGfxSet);

  smgfx_OutputDashedLines(FALSE, pOptGfxSet) ;  

} // end smgfx_DrawPolyline

/*******************************************************************//**
PURPOSE: Draws a sequence of vecs with given base pts as a set of
  tines on a comb

NOTES: 
***********************************************************************/
void smgfx_DrawComb
 (double *pdVecs,                   // in : vecs packed into double array as:[V0 .. V1 .. V2 ... Vn],    sized:[npts * lVecStrideInDoubles]          
  double *pdBasePts,                // in : basePts packed into double array as:[p0 .. P1 .. P2 ... Pn], sized:[npts * lBasePtStrideInDoubles]       
  long    nPts,                     // in : number of points to plot from pts array                                                                  
  double  dScale,                   // in : scale applied to every vector,                                                                           
                                    //      default:[1.0]                                                                                            
  long    lVecStrideInDoubles,      // in : number of doubles between start of each vec in the *vecs array                                           
                                    //      ex: 3 = packed 3d vectors                                                                                
                                    //      ex: 6 = plot every other 3d vector.                                                                      
                                    //      default:[3]                                                                                              
  long    lBasePtStrideInDoubles,   // in : number of doubles between start of each basePt in the *basePts array                                     
                                    //      default:[3] 
  SmGfxArraySet *pOptGfxSet)         // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                    //      NULL to ignore. default:[NULL]
{                                                                                                                                  

  if (!s_View.m_bGraphicsInitialized) return;

  smgfx_OutputComb(pdVecs, pdBasePts, nPts, dScale, lVecStrideInDoubles, lBasePtStrideInDoubles, pOptGfxSet);

} // end smgfx_DrawComb

#ifdef USING_DEBUG_UI
static SmBoolean bFirst = TRUE;
#endif // USING_DEBUG_UI

/*******************************************************************//**
PURPOSE: Deletes all display lists stored in s_View.m_pActiveLists and
            redraws scene.

NOTES:
            For debugging purposes also calls
            sm_GraphicsBrepListClear() when bClearUIPickLists is
            TRUE to clear the UI brep display list.

            Otherwise there will be undisplayed Breps that can
            be picked.

***********************************************************************/
void smgfx_Erase
  (SmBoolean bClearUIPickLists) // in : TRUE : clear UI pick lists
                                //      FALSE: don't
                                //     default:[TRUE] 
{
  SM_REF1(bClearUIPickLists);
#ifdef USING_DEBUG_UI
  if (bFirst) 
    {
      // set the UI message loop callback method
      smos_Disp.m_InvokeUICallback();
      bFirst = FALSE;
    }

#endif // USING_DEBUG_UI


} // end smgfx_Erase

/*******************************************************************//**
PURPOSE: return 1 if any display lists are stored in s_View.m_pActiveLists,
            else return 0.

NOTES: 
***********************************************************************/
int smgfx_AnyDrawing() 
{
  return(0);

} // end smgfx_AnyDrawing

/*******************************************************************//**
PURPOSE: mark graphics as initialized.

NOTES: 
***********************************************************************/
void smgfx_Initialize(const SmContext & crContext)
{
  SM_REF1(crContext);
  s_View.m_bGraphicsInitialized = TRUE;

} // end smgfx_Initialize

/*******************************************************************//**
PURPOSE:  delete display lists and the display list array.

NOTES: 
***********************************************************************/
void smgfx_Terminate()
{
  smgfx_Erase(FALSE);


  s_View.m_bGraphicsInitialized = FALSE;

} // end smgfx_Terminate

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_DrawTriangle
 (const SmPoint3d  & crP1,      // in : Vertex 1 of triangle
  const SmPoint3d  & crP2,      // in : Vertex 2 of triangle
  const SmPoint3d  & crP3,      // in : Vertex 3 of triangle
  const SmVector3d * cpNormal1, // in : Triangle Surface Normal at Vertex 1
  const SmVector3d * cpNormal2, // in : Triangle Surface Normal at Vertex 2
  const SmVector3d * cpNormal3, // in : Triangle Surface Normal at Vertex 3
  SmGfxArraySet    * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;

// gwc:CHANGE_COLOR - shouldn't need to adjust triangle colors
//    smgfx_RefreshColor();
//    if (s_Disp.m_dRed < 0.1 && s_Disp.m_dGreen < 0.1 && s_Disp.m_dBlue < 0.1) {
//        smgfx_OutputColor(0.7,0.7,0.7);
//    }
  smgfx_OutputTriangle(crP1, crP2, crP3, cpNormal1, cpNormal2, cpNormal3, pOptGfxSet);

} // end smgfx_DrawTriangle

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_DrawQuad
 (const SmPoint3d  & crP1,      // in : Vertex 1 of triangle
  const SmPoint3d  & crP2,      // in : Vertex 2 of triangle
  const SmPoint3d  & crP3,      // in : Vertex 3 of triangle
  const SmPoint3d  & crP4,      // in : Vertex 3 of triangle
  const SmVector3d * cpNormal1, // in : Triangle Surface Normal at Vertex 1
  const SmVector3d * cpNormal2, // in : Triangle Surface Normal at Vertex 2
  const SmVector3d * cpNormal3, // in : Triangle Surface Normal at Vertex 3
  const SmVector3d * cpNormal4, // in : Triangle Surface Normal at Vertex 3
  SmGfxArraySet    * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;

// gwc:CHANGE_COLOR - shouldn't need to adjust triangle colors
//    smgfx_RefreshColor();
//    if (s_Disp.m_dRed < 0.1 && s_Disp.m_dGreen < 0.1 && s_Disp.m_dBlue < 0.1) {
//        smgfx_OutputColor(0.7,0.7,0.7);
//    }
  smgfx_OutputQuad(crP1, crP2, crP3, crP4, cpNormal1, cpNormal2, cpNormal3, cpNormal4, pOptGfxSet);

} // end smgfx_DrawQuad

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_DrawCircularArc
  (const SmPoint3d  & crOrigin,        // in : arc origin
   const SmVector3d & crX,             // in : defines zero degree start for arc
   const SmVector3d & crY,             // in : specifies circle plane and 90 degree direction for arc
   double             dRadius,         // in : arc radius
   double             dStartAngleDeg,  // in : arc start point in degrees
   double             dEndAngleDeg,    // in : arc end point in degrees
   double             dMaxHeightError, // in : max arc height error allowed
                                       // note: number of polysegments drawn
                                       //       is determined by the radius and
                                       //       the dMaxHieghtError
   SmGfxArraySet    * pOptGfxSet)       // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;

  smgfx_OutputCircularArc(crOrigin, crX, crY, dRadius, dStartAngleDeg, dEndAngleDeg, dMaxHeightError, pOptGfxSet);

} // end smgfx_DrawCircularArc

/*******************************************************************//**
PURPOSE: Draw Plane grid

NOTES: 
***********************************************************************/
void smgfx_DrawPlaneGrid
 (const SmPoint3d  & crOrigin,       // in : origin of the grid
  const SmVector3d & crVec1,         // in : unit vec1 direction to draw
  const SmVector3d & crVec2,         // in : unit vec2 direction to draw
  ULONG              lVec1CopyCount, // in : number of Vec1 copies drawn on each side of Vec1
  ULONG              lVec2CopyCount, // in : number of Vec2 copies drawn on each side of Vec2
  double             dVec1Spacing,   // in : space between vec1 copies
  double             dVec2Spacing,   // in : space between vec2 copies
  SmGfxArraySet    * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                     //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;

  // locals
  SmPoint3d sP1, sP2 ;

  ULONG ii ; 
  ULONG icnt1 = 2 * lVec1CopyCount + 1 ;
  ULONG icnt2 = 2 * lVec2CopyCount + 1 ;

  // Vec1 copies
  sP1 = crOrigin - lVec1CopyCount * dVec1Spacing * crVec2 - lVec2CopyCount * dVec2Spacing * crVec1 ;
  sP2 = crOrigin - lVec1CopyCount * dVec1Spacing * crVec2 + lVec2CopyCount * dVec2Spacing * crVec1 ;
  for(ii=0;ii<icnt1;ii++)
    {
      smgfx_DrawLine(&sP1, &sP2, pOptGfxSet) ;
      sP1 += dVec1Spacing * crVec2 ;
      sP2 += dVec1Spacing * crVec2 ;
    }

  // Vec2 copies
  sP1 = crOrigin - lVec2CopyCount * dVec2Spacing * crVec1 - lVec1CopyCount * dVec1Spacing * crVec2 ;
  sP2 = crOrigin - lVec2CopyCount * dVec2Spacing * crVec1 + lVec1CopyCount * dVec1Spacing * crVec2 ;
  for(ii=0;ii<icnt2;ii++)
    {
      smgfx_DrawLine(&sP1, &sP2, pOptGfxSet) ;
      sP1 += dVec2Spacing * crVec1 ;
      sP2 += dVec2Spacing * crVec1 ;
    }

} // end smgfx_DrawPlaneGrid

/*******************************************************************//**
PURPOSE: Draw Cylinder grid

NOTES: 
***********************************************************************/
void smgfx_DrawCylinderGrid
 (const SmPoint3d  & crAxisPt,         // in : Point on Cyl Axis
  const SmVector3d & crAxisVec,        // in : unit vec in Cyl Axis direction
  const SmVector3d & crVecX,           // in : unit vec from Origin to theta=0 cylinder IsoLine
  double             dRadius,          // in : Cylinder radius
  ULONG              lThetaPieces,     // in : number arc pieces from 0 to 360, default:[12]
  ULONG              lArcCount,        // in : number of arc copies drawn on each side of crAxisPt, default:[5]
  double             dArcSpacing,      // in : space between arc copies, default:[1]
  SmGfxArraySet    * pOptGfxSet)       // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;

  // locals
  SmPoint3d sP1, sP2 ;
  ULONG ii ;
  double dArcIncDeg = 360.0 / (lArcCount > 0 ? lArcCount : 12) ; 
  double dArcAngDeg = 0.0 ;
  SmVector3d sVecY = crAxisVec * crVecX ;

  // pick MaxHeightError
  double dMaxHeightError = dRadius / 100.0 ;  // about 20 segments per circle

  // Draw isoLines
  for(ii=0;ii<lThetaPieces;ii++,dArcAngDeg += dArcIncDeg)
    {
      double dCos = smos_CosDeg(dArcAngDeg) ;
      double dSin = smos_SinDeg(dArcAngDeg) ;

      sP1 = crAxisPt + dCos * dRadius * crVecX + dSin * dRadius * sVecY - lArcCount * dArcSpacing * crAxisVec ;
      sP2 = sP1 + 2.0 * lArcCount * dArcSpacing * crAxisVec ;

      smgfx_DrawLine(&sP1, &sP2, pOptGfxSet) ;
    }

  // Draw IsoCircles
  ULONG lArcCnt = 2 * lArcCount + 1 ;
  sP1 = crAxisPt - lArcCount * dArcSpacing * crAxisVec ; // center of 1st circle to draw

  for(ii=0;ii<lArcCnt;ii++)
    {
      smgfx_DrawCircularArc(sP1,             // in : arc origin
                            crVecX,          // in : defines zero degree start for arc
                            sVecY,           // in : specifies circle plane and 90 degree direction for arc
                            dRadius,         // in : arc radius
                            0.0,             // in : arc start point in degrees
                            360.0,           // in : arc end point in degrees
                            dMaxHeightError, // in : max arc height error allowed
                            pOptGfxSet) ;

      sP1 += dArcSpacing * crAxisVec ;
    }

} // end smgfx_DrawCylinderGrid

/*******************************************************************//**
PURPOSE: Draw Sphere grid

NOTES: 
***********************************************************************/
void smgfx_DrawSphereGrid
 (const SmPoint3d  & crOrigin,         // in : origin of the sphere
  const SmVector3d & crVecX,           // in : unit vec in Sphere X direction (pts to theta = 0 or 360)
  const SmVector3d & crVecZ,           // in : unit vec in Sphere Z direction (pts to Phi = 180)
  double             dRadius,          // in : Cylinder radius
  ULONG              lThetaPieces,     // in : number of const theta minor 1/2 circles drawn from 0 to 360, default:[12]
  ULONG              lZCircleCnt,      // in : number of const Z circles drawn from Bot to Top, default:[5]
  SmGfxArraySet    * pOptGfxSet)       // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;

  // locals
  ULONG      ii ;
  SmVector3d sVecY = crVecZ * crVecX ; 
  SmVector3d sThisY ;
  double     dArcIncDeg = 360.0 / (lThetaPieces > 0 ? lThetaPieces : 12) ; 
  double     dArcAngDeg = 0.0 ;

  // pick MaxHeightError
  double dMaxHeightError = dRadius / 100.0 ;  // about 20 segments per circle

  // Draw constant theta IsoArcs
  for(ii=0;ii<lThetaPieces;ii++,dArcAngDeg += dArcIncDeg)
    {
      double dCos = smos_CosDeg(dArcAngDeg) ;
      double dSin = smos_SinDeg(dArcAngDeg) ;

      sThisY = dCos * crVecX + dSin * sVecY ;

      smgfx_DrawCircularArc(crOrigin,        // in : arc origin
                            -crVecZ,         // in : defines zero degree start for arc
                            sThisY,          // in : specifies circle plane and 90 degree direction for arc
                            dRadius,         // in : arc radius
                            0.0,             // in : arc start point in degrees
                            180.0,           // in : arc end point in degrees
                            dMaxHeightError, // in : max arc height error allowed
                            pOptGfxSet) ;
    }

  // Draw constant Z IsoCircles
  if(lZCircleCnt == 0) lZCircleCnt = 5 ;

  double    dZInc   = 2 * dRadius / (lZCircleCnt+1) ;
  SmPoint3d sCenter = crOrigin - (dRadius - dZInc) * crVecZ ;

  for(ii=0;ii<lZCircleCnt;ii++, sCenter += dZInc * crVecZ)
    {
      // dThisRadius = Sqrt( Radius**2 - ZLength**2)
      double dThisRadius = smos_Sqrt(dRadius*dRadius - (dRadius-ii*dZInc)*(dRadius-ii*dZInc)) ;
      dMaxHeightError    = dThisRadius / 100.0 ;
      smgfx_DrawCircularArc(sCenter,         // in : arc origin
                            crVecX,          // in : defines zero degree start for arc
                            sVecY,           // in : specifies circle plane and 90 degree direction for arc
                            dThisRadius,     // in : arc radius
                            0.0,             // in : arc start point in degrees
                            180.0,           // in : arc end point in degrees
                            dMaxHeightError, // in : max arc height error allowed
                            pOptGfxSet) ;
    }

} // end smgfx_DrawSphereGrid

/*******************************************************************//**
PURPOSE: Draw Torus grid

NOTES: 
***********************************************************************/
void smgfx_DrawTorusGrid
 (const SmPoint3d  & crOrigin,       // in : origin of the Torus
  const SmVector3d & crVecX,         // in : unit vec in Torus X direction (pts to theta = 0 or 360)
  const SmVector3d & crVecZ,         // in : unit vec in Torus Z direction (Normal to torus plane)
  double             dMajRadius,     // in : Cylinder radius
  double             dMinRadius,     // in : Cylinder radius
  ULONG              lThetaPieces,   // in : number ConstTheta minor circles drawn from 0 to 360
  ULONG              lZCircleCnt,    // in : number of const Z circles drawn from Bot to Top, default:[5]
  SmGfxArraySet    * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                     //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;

  // locals
  ULONG ii ;
  SmVector3d sVecY = crVecZ * crVecX ; 
  SmVector3d sThisX, sCenter ;
  

  double dArcIncDeg = 360.0 / (lThetaPieces > 0 ? lThetaPieces : 12) ; 
  double dArcAngDeg = 0.0 ;

  // pick MaxHeightError
  double dMaxHeightError = dMinRadius / 100.0 ;  // about 20 segments per circle

  // Draw constant theta minor circles
  for(ii=0;ii<lThetaPieces;ii++,dArcAngDeg += dArcIncDeg)
    {
      double dCos = smos_CosDeg(dArcAngDeg) ;
      double dSin = smos_SinDeg(dArcAngDeg) ;

      sThisX  = dCos * crVecX + dSin * sVecY ;
      sCenter = crOrigin + dMajRadius * sThisX ; 

      smgfx_DrawCircularArc(sCenter,         // in : arc origin
                            sThisX,          // in : defines zero degree start for arc
                            crVecZ,          // in : specifies circle plane and 90 degree direction for arc
                            dMinRadius,      // in : arc radius
                            0.0,             // in : arc start point in degrees
                            360.0,           // in : arc end point in degrees
                            dMaxHeightError, // in : max arc height error allowed
                            pOptGfxSet) ;
    }

  // Draw constant Z IsoCircles
  if(lZCircleCnt == 0) lZCircleCnt = 5 ;

  double    dZInc   = 2 * dMinRadius / (lZCircleCnt+1) ;
  sCenter = crOrigin - (dMinRadius - dZInc) * crVecZ ;

  for(ii=0;ii<lZCircleCnt;ii++, sCenter += dZInc * crVecZ)
    {
      // Inside  Circle Radius = MajRadius - Sqrt( MinRadius**2 - ZLength**2)
      // Outside Circle Radius = MajRadius + Sqrt( MinRadius**2 - ZLength**2)
      double dThisRadius = smos_Sqrt(dMinRadius*dMinRadius - (dMinRadius-ii*dZInc)*(dMinRadius-ii*dZInc)) ;
      
      // inside circle 
      dMaxHeightError    = (dMajRadius - dThisRadius) / 100.0 ;
      smgfx_DrawCircularArc(sCenter,                  // in : arc origin
                            crVecX,                   // in : defines zero degree start for arc
                            sVecY,                    // in : specifies circle plane and 90 degree direction for arc
                            dMajRadius - dThisRadius, // in : arc radius
                            0.0,                      // in : arc start point in degrees
                            360.0,                    // in : arc end point in degrees
                            dMaxHeightError,          // in : max arc height error allowed
                            pOptGfxSet) ;

      // outside circle 
      dMaxHeightError    = (dMajRadius + dThisRadius) / 100.0 ;
      smgfx_DrawCircularArc(sCenter,                  // in : arc origin
                            crVecX,                   // in : defines zero degree start for arc
                            sVecY,                    // in : specifies circle plane and 90 degree direction for arc
                            dMajRadius + dThisRadius, // in : arc radius
                            0.0,                      // in : arc start point in degrees
                            360.0,                    // in : arc end point in degrees
                            dMaxHeightError,          // in : max arc height error allowed
                            pOptGfxSet) ;
    }

} // end smgfx_DrawTorusGrid

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void smgfx_DrawDiscGrid 
 (const  SmPoint3d  & crOrigin,      // in : origin of the grid
  const  SmVector3d & crVec1,        // in : disc normal direction to draw
  double              dRadius,       // in : disc radius
  ULONG               lRadiiCount,   // in : number of radii drawn per 360 degrees.
  SmGfxArraySet     * pOptGfxSet)    // in : used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                     //      NULL to ignore. default:[NULL]
{
  if (!s_View.m_bGraphicsInitialized) return;

  // locals
  SmPoint3d sP1, sP2 ;
  SmVector3d sVec, sX, sY ;

  // get orthogonal vectors to crVec1
  crVec1.MakeUnitOrthoVectors(NULL, sVec, sX, sY) ; 

  // for every radii
  ULONG ii ;
  double dAng = 0.0, dAngInc = 2.0 * SM_PI / lRadiiCount ;
  for(ii=0;ii<lRadiiCount;ii++, dAng+=dAngInc)
    {
      double c = cos(dAng) ;
      double s = sin(dAng) ;
      sVec = c * sX + s * sY ;

      sP1 = crOrigin + dRadius* 1.0/8.0 * sVec ;
      sP2 = crOrigin + dRadius* 7.0/8.0 * sVec ;

      smgfx_DrawLine(&sP1, &sP2, pOptGfxSet) ;

      sP1 = crOrigin + dRadius* 9.0/8.0 * sVec ;
      sP2 = crOrigin + dRadius*12.0/8.0 * sVec ;

      smgfx_DrawLine(&sP1, &sP2, pOptGfxSet) ;

    } // end iter every radii

  // output a circle
  smgfx_OutputCircularArc(crOrigin, sX, sY, dRadius, 0.0, 360.0, dRadius/50.0, pOptGfxSet) ;

} // end smgfx_DrawDiscGrid

/*******************************************************************//**
PURPOSE: 

NOTES: 
 To Use: debug into this routine
         put break on return within if (FALSE){}
         After stepping into sm_GraphicsLoop do one 'Step' then
         set  next statement to smos_Disp.m_InvokeUICallBack
         tell debug to go (continue)
         Object appears in graphics...you can rotate, scale etc
         use 'View->DebugExit' to return to here (return)
         See comments below
***********************************************************************/
void sm_GraphicsLoop
  (SmBoolean bInteractiveFlag)  // in : FALSE = don't do interactive grahics ;                        
                                //      TRUE  = do interactive graphics after clearing the message queue
                                //      default:[FALSE]
{
  if(bInteractiveFlag) 
    { 
      smos_InvokeUICallback() ; // pass control to the callback registered with smos_SetUICallback()
      return ;
    }

} // end sm_GraphicsLoop

/*******************************************************************//***
PURPOSE -- Add a copy of a Brep, surface, curve, to the UI interface 
           UserBreps, UserSurfaces, or UserCurves array
           for debug picking purposes

USAGE NOTE --- Called by Draw() methods when bAddToUIPickList is TRUE. Does nothing
               unless a host application has registered smos_SetBrepListCallback();
               nothing in this repository does.
************************************************************************/
void sm_GraphicsAddToBrepList
  (const SmObject *pObject)          // in : target brep, surface, or curvbe
{
  if(pObject)
    {
      smos_InvokeBrepListCallback(pObject) ;  // callback registered with smos_SetBrepListCallback()
    }

} // end sm_GraphicsAddToBrepList

/*******************************************************************//***
PURPOSE -- Clear the UI interface UserBreps array
           for debug picking purposes

USAGE NOTE ---
************************************************************************/
void sm_GraphicsBrepListClear
  ()
{                                       
  // clear the host application's pick lists
  //    sm_GraphicsBrepListClear() 
  //       invokes smos_InvokeBrepListClearCallback() { if (s_pfBrepListClearFun) { s_pfBrepListClearFun(); } }
  //          which calls the callback registered with smos_SetBrepListClearCallback()
   smos_InvokeBrepListClearCallback() ;  // callback registered with smos_SetBrepListClearCallback()
                                        

} // end sm_GraphicsBrepListClear // end sm_GraphicsBrepListClear

////////////////////////////////////////////////////////////////////

#endif // SM_GFX_CODE

//*******************************************************************//********
// The DebugUI gives you the ability to create an OpenGL window separate from
// your application and draw geometry in it.  You can also pan, zoom, rotate,
// save the current view, and other stuff in it.  This window uses direct windows
// API calls and does not depend on MFC.  Below describes how to setup and use
// the debug window.
//
//  **** How To Set Up Debug Window:
//
//   1) Create a DLL project with the following software included in it.
//      DebugUI.cpp, DisplayDialog.cpp, smCurveTest.cpp,
//      smFilletTest.cpp, smiges_Disp.m_test.cpp, smMergeTest.cpp, smoffset_test.cpp,
//      smSurfaceTest.cpp, smSweepTest.cpp, smtess_Disp.m_test.cpp, SmView.cpp, 
//      FileioDialog.cpp, Stdafx.cpp, TestingDialog.cpp
//
//   2) Somewhere in your existing code's initilization functions where you
//      have access to the process instance handle (HINSTANCE), include 
//      SmInitUI.h and declare an instance of the SmInitUI class with the
//      HINSTANCE as follows:
//         #ifdef _DEBUG
//         #include <SmInitUI.h>
//         #endif
//         .....
//         void sm_initilization(HINSTANCE hInstance)
//         {
//         #ifdef _DEBUG
//             SmInitUI initui(hInstance);
//         #endif
//         }
//      
//    3) To open the graphics window call smos_Disp.m_InvokeUICallback somewhere.
//       We typically do it inside of smgfx_Erase.  You may remove the comments
//       in that code as follows to do the same"
//
//     static SmBoolean bFirst = TRUE;
//     void smgfx_Erase()
//     {
//     if (bFirst) {
//         smos_Disp.m_InvokeUICallback();
//         bFirst = FALSE;
//     }
//
//
// **** How to Use the Debug Window:
//
//    1) You will need to compile debug (_DEBUG) with graphics enabled in smos_Disp.m_config.h.
//       Make sure SM_NO_GFX_OUTPUT_CODE and SM_NO_GFX_CODE are not defined in
//       your C++ options.
//       
//         #ifdef _DEBUG
//         #if !defined(SM_NO_GFX_OUTPUT_CODE)
//         #define SM_GFX_OUTPUT_CODE 1
//         #if !defined(SM_NO_GFX_CODE)
//         #define SM_GFX_CODE 1
//         #endif
//         #endif
//         #endif
//
//    2) To open the debugger window the first time you can call smgfx_Erase().  Draw 
//       something and Then to go into the debugger window 'step into' sm_GraphicsLoop().  
//
//       #ifdef SM_DEBUG_CODE
//            if (bDebugMe) {
//                smgfx_Erase();
//                smgfx_SetColor(1,0,0);
//                pCurve->Draw();
//                smgfx_SetColor(0,0,0);
//                pBrep->Draw();
//                sm_GraphicsLoop();    <--  Step into this function
//            }
//       #endif
//
//    3) Step one time after you are in sm_GraphicsLoop.  NOTE you must step one time 
//       after you are in sm_GraphicsLoop this is very important.  After that 
//       use 'set next statement' in the debugger to the smos_Disp.m_InvokeUICallback.
//       You can use step over or continue to get into the debugger loop.
//       We often set a break at the return statement.
//        
//       void sm_GraphicsLoop(SmBoolean bDebugMe = FALSE)
//       {
//           if (FALSE) {                  <-- Step to Here First -- NOTE This is very important
//               smos_Disp.m_InvokeUICallback();  <-- 'Set Next Statement' to here next
//               return;                   <-- Set a break here to catch you when exiting debug window.
//           }
//       }
//      
//    4) Your debug graphics window should now be open and active.  You can test this 
//       by moving the cursor over the menu's.  When finished you can use 'debug exit'
//       Short cut keys:  'Shift - Left Mouse' starts zoom.  You can zoom in if you
//       click hold and drag the mouse from the upper right to the lower left of your
//       screen.  You can zoom out by starting on the lower left and going to your 
//       upper right.  'Ctrl - Left Mouse' starts the rotate.  'Ctrl & Shift - Left Mouse'
//       starts the Pan mode.




