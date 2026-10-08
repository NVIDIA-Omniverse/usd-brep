// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmGraphicsOutput.cpp
* PURPOSE: Low level core interface to graphics.
**********************************************************************/

 
#include "StdAfx.h"

#include <SmGraphicsOutput.h>
#include <SmVolume.h>

#ifdef SM_GFX_OUTPUT_CODE




/*******************************************************************//**
PURPOSE: UGLY non-multithreaded work around for bug found but not fixed
in smgfx_OutputColor() and smgfx_GetOutputColor() that sometimes makes
the LastColor value returned by those functions invalid.

NOTES: see test and note left in smgfx_OutputColor.
***********************************************************************/
static SmVector3d sm_LastColor(0,0,0) ;   // local copy of the glOpen GL_CURRENT_COLOR

/*******************************************************************//**
PURPOSE: Optional Graphics Callback Mechanism

NOTES: 
***********************************************************************/

// If you would like to use the graphics callbacks to draw lines and polylines,
// define SM_GRAPHICS_CALLBACKS=1 in the consuming build.
//
// Behavioral note: the callback dispatch sites below are gated with
// `#if SM_GRAPHICS_CALLBACKS` (value), not `#ifdef` (defined-ness). Because the
// macro is always defined here (to 0 by default), an `#ifdef` test would always
// be true and compile the dispatch in regardless of the value, which defeats the
// opt-in. Consumers that set DrawLineCallBack/DrawPointCallBack/DrawPolyLineCallBack
// must build with SM_GRAPHICS_CALLBACKS=1 for those callbacks to take effect.
#ifndef SM_GRAPHICS_CALLBACKS
  #define SM_GRAPHICS_CALLBACKS 0
#endif

void (*DrawLineCallBack)(double x,  double y,  double z,
                         double xx, double yy, double zz,
                         SmGfxArraySet *pOptGfxSet) = NULL;  // note: the VC 2013 compiler does not allow default arguments on
                                                             //       pointers to functions.  Defaulting pOptGfxSet to NULL used
                                                             //       to be accepted on previous compiler versions. No longer.  
                                                             // i/o: pOptGfxSet: used for OpenGL ES 2.0. 
                                                             //      When given, accumulate GfxVertexArrays else make GL calls.
                                                             //      NULL to ignore. pOptGfxSet used to default to NULL. No Longer.

void (*DrawPointCallBack)(double x, double y, double z, SmGfxArraySet *pOptGfxSet) = NULL;

void (*DrawPolyLineCallBack)(double *pts, long npts, SmGfxArraySet *pOptGfxSet) = NULL;

// here you set a pointer to the users line drawing routine.

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
void smgfx_SetDrawLineCallBack(void (*dl)(double x,  double y,  double z,
                                          double xx, double yy, double zz,
                                          SmGfxArraySet *pOptGfxSet))   // note: the VC 2013 compiler does not allow default arguments on
                                                             //       pointers to functions.  Defaulting pOptGfxSet to NULL used
                                                             //       to be accepted on previous compiler versions. No longer.  
                                                             // i/o: pOptGfxSet: used for OpenGL ES 2.0. 
                                                             //      When given, accumulate GfxVertexArrays else make GL calls.
                                                             //      NULL to ignore. pOptGfxSet used to default to NULL. No Longer.
{
  DrawLineCallBack = dl;
}

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
void smgfx_SetDrawPointCallBack(void (*dp)(double x,double y, double z,
                                           SmGfxArraySet *pOptGfxSet))   // note: the VC 2013 compiler does not allow default arguments on
                                                             //       pointers to functions.  Defaulting pOptGfxSet to NULL used
                                                             //       to be accepted on previous compiler versions. No longer.  
                                                             // i/o: pOptGfxSet: used for OpenGL ES 2.0. 
                                                             //      When given, accumulate GfxVertexArrays else make GL calls.
                                                             //      NULL to ignore. pOptGfxSet used to default to NULL. No Longer.
{
  DrawPointCallBack = dp;
}

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
void smgfx_SetDrawPolyLineCallBack(void (*dpl)(double *pts, long npts,
                                               SmGfxArraySet *pOptGfxSet))  // note: the VC 2013 compiler does not allow default arguments on
                                                             //       pointers to functions.  Defaulting pOptGfxSet to NULL used
                                                             //       to be accepted on previous compiler versions. No longer.  
                                                             // i/o: pOptGfxSet: used for OpenGL ES 2.0. 
                                                             //      When given, accumulate GfxVertexArrays else make GL calls.
                                                             //      NULL to ignore. pOptGfxSet used to default to NULL. No Longer.
{
  DrawPolyLineCallBack = dpl;
}


/*******************************************************************//**
PURPOSE: This function sends a point to the graphics.

NOTES: 
***********************************************************************/
void smgfx_OutputPoint
 (double          dX,        // in : X of point[xyz]                                                        
  double          dY,        // in : Y of point[xyz]                                                        
  double          dZ,        // in : Z of point[xyz]                                                        
  SmGfxArraySet * pOptGfxSet) // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                             //      NULL to ignore. default:[NULL]                                         
{
  if(pOptGfxSet)
    {
      SmGfxVertexArray * pGfxVertexArray = pOptGfxSet->NewGfxVertexArray(SM_GV_P, SM_GV_POINT) ;
      pGfxVertexArray->Add( (float)dX, (float)dY, (float)dZ) ;
      return ; 
    }

#if SM_GRAPHICS_CALLBACKS
  if(DrawPointCallBack)
  { // Call user callback function
    (*DrawPointCallBack) (dX, dY, dZ, pOptGfxSet);
    return;
  }
#endif // SM_GRAPHICS_CALLBACKS

}

/*******************************************************************//**
PURPOSE: This function sends a point size change to the graphics

NOTES: 
***********************************************************************/
double smgfx_GetOutputPointSize
 (SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]                                         
{
  double dLastPointSize = 2.0 ;

  if(pOptGfxSet) 
    {
      dLastPointSize = pOptGfxSet->GetPointSize() ;
    }
  else
    {
    }
  return(dLastPointSize) ;

} // end smgfx_GetOutputPointSize

/*******************************************************************//**
PURPOSE: This function sends a line width change to the graphics.

NOTES: 
***********************************************************************/
double smgfx_GetOutputLineWidth
 (SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]                                         
{
  double dLastLineWidth = 1.0 ;

  if(pOptGfxSet) 
    {
      dLastLineWidth = pOptGfxSet->GetLineWidth() ;
    }
  else
    {
    }

  return(dLastLineWidth) ;

} // end smgfx_GetOutputLineWidth

/*******************************************************************//**
PURPOSE: Set the dashed lines on or off.

NOTES: 
***********************************************************************/
SmBoolean smgfx_GetOutputDashedLines
 (SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]                                         
{
  int sLastStipple = 0xFFFF ;

  //double dLastLineWidth = 1.0 ;

  if(pOptGfxSet) 
    {
      sLastStipple = pOptGfxSet->GetStipple() ;
    }
  else
    {
    }

  return(sLastStipple == 0x00FF ? TRUE : FALSE) ;

} // end smgfx_GetOutputDashedLines

/*******************************************************************//**
PURPOSE: This function returns GL_CURRENT_COLOR.

NOTES:
***********************************************************************/
SmVector3d smgfx_GetOutputColor
 (SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]                                         
{
  SmVector3d sLastColor[2] ;
  sLastColor[0].x = sLastColor[0].y = sLastColor[0].z = 0.0 ;

  if(pOptGfxSet) 
    {
      sLastColor[0] = pOptGfxSet->GetColor() ;
    }
  else
    {
    }

  // return red green blue vector
  return sLastColor[0] ;

} // end smgfx_GetOutputColor

/*******************************************************************//**
PURPOSE: This function sends a point size change to the graphics

NOTES: 
***********************************************************************/
double smgfx_OutputPointSize
 (double          dPointSize,
  SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]                                         

{
  double dLastPointSize = 2.0 ;

  if(pOptGfxSet)
    {
      dLastPointSize = (double)pOptGfxSet->SetPointSize((float)dPointSize) ;
    }
  else
    {
    }

  return(dLastPointSize) ;

} // end smgfx_OutputPointSize

/*******************************************************************//**
PURPOSE: This function sends a line width change to the graphics.

NOTES: 
***********************************************************************/
double smgfx_OutputLineWidth
 (double          dLineWidth,
  SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]                                         
{
  double dLastLineWidth = 1.0 ;

  if(pOptGfxSet)
    {
      dLastLineWidth = (double)pOptGfxSet->SetLineWidth((float)dLineWidth) ;
    }
  else
    {
    }

  return(dLastLineWidth) ;

} // end smgfx_OutputLineWidth

/*******************************************************************//**
PURPOSE: Set the dashed lines on or off.

NOTES: 
***********************************************************************/
SmBoolean smgfx_OutputDashedLines
 (SmBoolean bDoDashedLines,
  SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]                                         
{
  int sLastStipple = 0xFFFF ;

  if(pOptGfxSet)
    {
      if (bDoDashedLines) { sLastStipple = pOptGfxSet->SetDashedLines(TRUE) ; }
      else                { sLastStipple = pOptGfxSet->SetDashedLines(FALSE) ; }
    }
  else
    {
    }

  return(sLastStipple == 0x00FF ? TRUE : FALSE) ;

} // end smgfx_OutputDashedLines


/*******************************************************************//**
PURPOSE: output the object->DrawColor selected by SmObject::GetObjectColor() 
            to the currently open drawList.

NOTES: 1. Returns last color to simplify restoring state if desired.
       2. This call does not modify any of the global colors.
***********************************************************************/
SmVector3d smgfx_OutputObjectColor
  (const SmObject *pObject,    // in : target object
   SmColorRuleType eColorRule, // in : target color rule
                               //      oneof  SM_CR_STANDARD
                               //             SM_CR_SHADING
                               //             SM_CR_OBJPROPERTY
                               //             SM_CR_CURVECONTROLPOLYGON,  
                               //             SM_CR_SURFACECONTROLPOLYGON,
                               //             SM_CR_VOLUMECONTROLPOLYGON,
                               //             SM_CR_SURFACENORMAL
                               //             SM_CR_NEG_SURFACENORMAL
                               //             SM_CR_SURFACENORMAL_INFINITE,
                               //             SM_CR_SURFACENORMAL_SOLID,   
                               //             SM_CR_SURFACENORMAL_VOID,    
                               //             SM_CR_CURVATURE
                               //             SM_CR_CURVATURE2
                               //             SM_CR_SPEED
                               //             SM_CR_KNOT
                               //             SM_CR_VARYCROSSHATCH
                               //             SM_CR_HIGHCOUNTCURVE
                               //             SM_CR_BASESURFACE
                               //             SM_CR_EXTENDEDSURFACE
                               //             SM_CR_NEIGHBOR,   
                               //             SM_CR_NEIGHBORMATE
                               //             SM_CR_HOTPOINT
                               //             SM_CR_HOTPOINTFILL
                               //             SM_CR_HOTPOINTSELECT
                               //             SM_CR_HOTPOINTHIGHLIGHT
                               //             SM_CR_TRACKTARGET
                               //             SM_CR_TRACKINFO  
                               //             SM_CR_TESTGEOMETRY
                               //             SM_CR_GAPPOINT,      
                               //             SM_CR_VERTEX_EDGEGAP,
                               //             SM_CR_VERTEX_FACEGAP,
                               //             SM_CR_EDGE_FACEGAP,   
                               //             SM_CR_IN_TOL_GAP,
                               //             SM_CR_OUT_TOL_GAP, 
                               //      default:[SM_CR_STANDARD]
  SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                               //      NULL to ignore. default:[NULL]                                         
{
  // use smgfx_OutputColor() to make glColor3d(dRed,dGreen,dBlue) call 
  //  and fetch LastColor return value
  return(smgfx_OutputColor(pObject->GetObjectColor(eColorRule), pOptGfxSet)) ;

} // end smgfx_OutputObjectColor

/*******************************************************************//**
PURPOSE: This function sends a color change to the graphics.
            Returns last color to simplify restoring state if desired.

NOTES:
***********************************************************************/
SmVector3d smgfx_OutputColor
  (double         dRed,        // in : new red color    from 0.0 to 1.0
   double         dGreen,      // in : new green color  from 0.0 to 1.0
   double         dBlue,       // in : new blue color   from 0.0 to 1.0
  SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                               //      NULL to ignore. default:[NULL]                                         
{
  SmVector3d sLastColor[2] ;
  sLastColor[0].x = sLastColor[0].y = sLastColor[0].z = 0.0 ;

  SM_ASSERT_MSG(    0.0 <= dRed   && dRed   <= 1.0 
                 && 0.0 <= dGreen && dGreen <= 1.0 
                 && 0.0 <= dBlue  && dBlue  <= 1.0, _T("smgfx_OutputColor:: bad input: Bad Color Coordinate Value sent to smgfx_OutputColor")) ;

  if(pOptGfxSet)
    {
      sLastColor[0] = pOptGfxSet->SetColor((float)dRed, (float)dGreen, (float)dBlue) ;
    }
  else
    {
    }

  return sLastColor[0] ;

} // end smgfx_OutputColor

/*******************************************************************//**
PURPOSE: This function sends a color change to the graphics.
            Returns last color to simplify restoring state if desired.

NOTES: 
***********************************************************************/

SmVector3d smgfx_OutputColor
 (const SmVector3d & rColor,       // in : [red, green, blue] values from 0.0 to 1.0
  SmGfxArraySet    * pOptGfxSet)   // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]                                         
{
  // use smgfx_OutputColor() to make glColor3d(dRed,dGreen,dBlue) call 
  //  and fetch LastColor return value
  return (smgfx_OutputColor(rColor.x, rColor.y, rColor.z, pOptGfxSet));

} // end smgfx_OutputColor

/*******************************************************************//**
PURPOSE: Change Output Color - output newColor to display list

NOTES: 
***********************************************************************/
SmVector3d smgfx_ChangeOutputColor
 (SmGfxArraySet    * pOptGfxSet)    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]                                         
{
  static ULONG lCount = 0;
  lCount ++;
  if (lCount % 6 == 0) 
    return( smgfx_OutputColor(1,0,1,pOptGfxSet) ) ;
  if (lCount % 6 == 1) 
    return( smgfx_OutputColor(0,0,1,pOptGfxSet) ) ;
  if (lCount % 6 == 2) 
    return( smgfx_OutputColor(0,1,1,pOptGfxSet) ) ;
  if (lCount % 6 == 3) 
    return( smgfx_OutputColor(1,0,0,pOptGfxSet) ) ;
  if (lCount % 6 == 4) 
    return( smgfx_OutputColor(0,1,0,pOptGfxSet) ) ;
  if (lCount % 6 == 5) 
    return( smgfx_OutputColor(1,1,0,pOptGfxSet) ) ;
                       
  return( smgfx_OutputColor(0,0,0,pOptGfxSet) ) ;

} // end smgfx_ChangeOutputColor

/*******************************************************************//**
PURPOSE: This function sends a line out to the graphics.

NOTES: 
***********************************************************************/
void smgfx_OutputLine
 (double dX1, double dY1, double dZ1,  // in : line start point
  double dX2, double dY2, double dZ2,  // in : line end point
  SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]                                         

{
#if SM_GRAPHICS_CALLBACKS
  if(DrawLineCallBack)
  { // Call user callback function
    (*DrawLineCallBack) (dX1, dY1, dZ1, dX2, dY2, dZ2, pOptGfxSet);
    return;
  }
#endif // SM_GRAPHICS_CALLBACKS

  double dPnts[6];
  dPnts[0] = dX1;    
  dPnts[1] = dY1;    
  dPnts[2] = dZ1;
  dPnts[3] = dX2;
  dPnts[4] = dY2;
  dPnts[5] = dZ2;
  smgfx_OutputPolyline(dPnts, 2, 3, pOptGfxSet);

} // end smgfx_OutputLine

/*******************************************************************//**
PURPOSE: This function sends a polyline out to the graphics.

NOTES:
***********************************************************************/
void smgfx_OutputPolyline
  (double       * pts,                 // in : pts packed into double array as:[p0 .. P1 .. P2 ... Pn]
                                       //      sized:[npts * lStrideInDoubles]
   long           npts,                // in : number of points to plot from pts array
   long           lStrideInDoubles,    // in : number of doubles between
                                       //      start of each pt in the *pts array
                                       //      ex: 3 = packed 3d vectors
                                       //      ex: 6 = plot every other 3d vector.
                                       //      default:[3]
  SmGfxArraySet * pOptGfxSet)           // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                       //      NULL to ignore. default:[NULL]                                         
{
  // size of pts array
  long icnt = npts * lStrideInDoubles ;

  if(pOptGfxSet)
    {
      SmGfxVertexArray * pGfxVertexArray = pOptGfxSet->NewGfxVertexArray(SM_GV_P, SM_GV_LINE) ;
      for (long i3=0; i3<icnt; i3+=lStrideInDoubles) 
        {
          pGfxVertexArray->Add( (float)pts[i3], (float)pts[i3+1], (float)pts[i3+2]);
        }
      return ; 
    }

#if SM_GRAPHICS_CALLBACKS
  if (DrawPolyLineCallBack) 
    { // Call user callback function
      (*DrawPolyLineCallBack) (pts, npts, pOptGfxSet);
      return ;
    }
#endif // SM_GRAPHICS_CALLBACKS


} // end smgfx_OutputPolyline

/*******************************************************************//**
PURPOSE: This function sends a sequence of lines out to the graphics.

NOTES: Each line[i] is drawn from basePts[i] to basePts[i]+vecs[i]
***********************************************************************/
void smgfx_OutputComb
  (double *vecs,                   // in : vecs packed into double array as:[V0 .. V1 .. V2 ... Vn],    sized:[npts * lVecStrideInDoubles]
   double *basePts,                // in : basePts packed into double array as:[p0 .. P1 .. P2 ... Pn], sized:[npts * lBasePtStrideInDoubles]
   long    npts,                   // in : number of points to plot from pts array
   double  dScale,                 // in : scale applied to every vector,
                                   //      default:[1.0]
   long    lVecStrideInDoubles,    // in : number of doubles between start of each vec in the *vecs array
                                   //      ex: 3 = packed 3d vectors
                                   //      ex: 6 = plot every other 3d vector.
                                   //      default:[3]
   long    lBasePtStrideInDoubles, // in : number of doubles between start of each basePt in the *basePts array
                                   //      default:[3]
   SmGfxArraySet * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]                                         
{
  long iv = 0, ib = 0 ;
  long icnt = npts * lVecStrideInDoubles ;

  if(pOptGfxSet)
    {
      for(iv=0,ib=0;iv<icnt;iv+=lVecStrideInDoubles, ib+=lBasePtStrideInDoubles)
        {
          smgfx_OutputLine(basePts[ib], 
                           basePts[ib+1], 
                           basePts[ib+2],
                           basePts[ib]   + dScale * vecs[iv],
                           basePts[ib+1] + dScale * vecs[iv+1],
                           basePts[ib+2] + dScale * vecs[iv+2],
                           pOptGfxSet) ;
        } // end iter every comb
      return ; 
    }
  
#if SM_GRAPHICS_CALLBACKS
  if (DrawPolyLineCallBack) 
    { // Call user callback function
      double dTine[6] ;
      for(iv=0;iv<icnt;iv+=lVecStrideInDoubles, ib+=lBasePtStrideInDoubles)
        {
         dTine[0] = basePts[ib] ; 
         dTine[1] = basePts[ib+1] ;
         dTine[2] = basePts[ib+2] ;
         dTine[3] = basePts[ib]   + dScale * vecs[iv] ;  
         dTine[4] = basePts[ib+1] + dScale * vecs[iv+1] ;
         dTine[5] = basePts[ib+2] + dScale * vecs[iv+2] ;
         (*DrawPolyLineCallBack) (dTine, 2, pOptGfxSet);
        }
      return ;
    }
#endif // SM_GRAPHICS_CALLBACKS


} // end smgfx_OutputComb

/*******************************************************************//**
PURPOSE: This function sends a triangle out to the graphics.

NOTES:
***********************************************************************/
void smgfx_OutputTriangle
  (const SmPoint3d & crP1,
   const SmPoint3d & crP2,
   const SmPoint3d & crP3,
   const SmVector3d * cpNormal1,
   const SmVector3d * cpNormal2,
   const SmVector3d * cpNormal3,
   SmGfxArraySet    * pOptGfxSet)      // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                      //      NULL to ignore. default:[NULL]                                         
{
#ifdef SM_DEBUG_CODE
  // check that normals are oriented for a counter clockwise polygon
  SmVector3d sNorm = (crP2 - crP1) * (crP3 - crP1) ; 
  SM_ASSERT(sNorm.Dot(*cpNormal1) >= 0.0) ;
  SM_ASSERT(sNorm.Dot(*cpNormal2) >= 0.0) ;
  SM_ASSERT(sNorm.Dot(*cpNormal3) >= 0.0) ;
#endif // SM_DEBUG_CODE

  if(pOptGfxSet)
    {
      SmGfxVertexArray * pGfxVertexArray = pOptGfxSet->NewGfxVertexArray(SM_GV_PN, SM_GV_SIMPLE_MESH) ;

      pGfxVertexArray->Add( (float)crP1.x, (float)crP1.y, (float)crP1.z, (float)cpNormal1->x, (float)cpNormal1->y, (float)cpNormal1->x) ;
      pGfxVertexArray->Add( (float)crP2.x, (float)crP2.y, (float)crP2.z, (float)cpNormal2->x, (float)cpNormal2->y, (float)cpNormal2->x) ;
      pGfxVertexArray->Add( (float)crP3.x, (float)crP3.y, (float)crP3.z, (float)cpNormal3->x, (float)cpNormal3->y, (float)cpNormal3->x) ;
      return ; 
    }


} // end smgfx_OutputTriangle

/*******************************************************************//**
PURPOSE: This function sends a quad out to the graphics.

NOTES:
***********************************************************************/
void smgfx_OutputQuad
  (const SmPoint3d & crP1,
   const SmPoint3d & crP2,
   const SmPoint3d & crP3,
   const SmPoint3d & crP4,
   const SmVector3d * cpNormal1,
   const SmVector3d * cpNormal2,
   const SmVector3d * cpNormal3,
   const SmVector3d * cpNormal4,
   SmGfxArraySet    * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                      //      NULL to ignore. default:[NULL]                                         
{
#ifdef SM_DEBUG_CODE
  // check that normals are oriented for a counter clockwise polygon
  SmVector3d sNorm = (crP2 - crP1) * (crP3 - crP1) ; 
  SM_ASSERT(sNorm.Dot(*cpNormal1) >= 0.0) ;
  SM_ASSERT(sNorm.Dot(*cpNormal2) >= 0.0) ;
  SM_ASSERT(sNorm.Dot(*cpNormal3) >= 0.0) ;
  SM_ASSERT(sNorm.Dot(*cpNormal4) >= 0.0) ;
#endif // SM_DEBUG_CODE

  if(pOptGfxSet)
    {
      SmGfxVertexArray * pGfxVertexArray = pOptGfxSet->NewGfxVertexArray(SM_GV_PN, SM_GV_SIMPLE_MESH) ;

      pGfxVertexArray->Add( (float)crP1.x, (float)crP1.y, (float)crP1.z, (float)cpNormal1->x, (float)cpNormal1->y, (float)cpNormal1->x) ;
      pGfxVertexArray->Add( (float)crP2.x, (float)crP2.y, (float)crP2.z, (float)cpNormal2->x, (float)cpNormal2->y, (float)cpNormal2->x) ;
      pGfxVertexArray->Add( (float)crP3.x, (float)crP3.y, (float)crP3.z, (float)cpNormal3->x, (float)cpNormal3->y, (float)cpNormal3->x) ;
      pGfxVertexArray->Add( (float)crP4.x, (float)crP4.y, (float)crP4.z, (float)cpNormal4->x, (float)cpNormal4->y, (float)cpNormal4->x) ;
      return ; 
    }


} // end smgfx_OutputQuad

/*******************************************************************//**
PURPOSE: This function sends a polyline circular arc approximation
            out to the graphics.

NOTES:
***********************************************************************/
void smgfx_OutputCircularArc
  (const SmPoint3d  & crOrigin,         // in : arc origin
   const SmVector3d & crX,              // in : defines zero degree start for arc
   const SmVector3d & crY,              // in : specifies circle plane and 90 degree direction for arc
   double             dRadius,          // in : arc radius
   double             dStartAngleDeg,   // in : arc start point in degrees
   double             dEndAngleDeg,     // in : arc end point in degrees
   double             dMaxHeightError,  // in : max arc height error allowed
   SmGfxArraySet    * pOptGfxSet)        // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                        //      NULL to ignore. default:[NULL]                                         
                                        // note: number of polysegments drawn
                                        //       is determined by the radius and
                                        //       the dMaxHieghtError
{
  // check for bad inputs
  if(dRadius         <= SM_EFF_ZERO) return ;
  if(dMaxHeightError <= SM_EFF_ZERO) return ;
  if(dMaxHeightError >= dRadius - SM_EFF_ZERO) return ;

  // watch out for zero vectors
  if(crX.LengthSquared() < SM_EFF_ZERO_SQ) return ;

  // locals
  SmVector3d sVecX = crX ; sVecX.Unitize() ;
  SmVector3d sVecY = crY - sVecX.Dot(crY) * sVecX ; 

  // watch out for zero vectors
  if(sVecY.LengthSquared() < SM_EFF_ZERO_SQ) return ;
  sVecY.Unitize() ;

  // get the arc increment size that yields an error = dMaxHeightError
  double dHalfAngle = acos(1.0 - (dMaxHeightError / dRadius)) ;

  // make sure dEndAngleDeg is greater than dStartAngleDeg
  while(dStartAngleDeg > dEndAngleDeg) { dEndAngleDeg += 360.0 ; }

  // get number of increments to output
  double dStartAngleRad = dStartAngleDeg * SM_PI / 180.0 ;
  double dEndAngleRad   = dEndAngleDeg   * SM_PI / 180.0 ;
  int lcnt = (int)(ceil((dEndAngleRad - dStartAngleRad) / (2*dHalfAngle))) + 1 ;
  if(lcnt > 64) lcnt = 64 ;
 
  // get angle increment
  SmPoint3d sPoint ;
  double dAngleInc = (dEndAngleRad - dStartAngleRad) / lcnt ;
  double dA        = dStartAngleRad ;

  sVecX *= dRadius ;
  sVecY *= dRadius ;

  if(pOptGfxSet)
    {
      SmGfxVertexArray * pGfxVertexArray = pOptGfxSet->NewGfxVertexArray(SM_GV_P, SM_GV_LINE) ;

      for (int i=0; i<=lcnt; i++, dA+=dAngleInc) 
        {
          sPoint = crOrigin + cos(dA) * sVecX + sin(dA) * sVecY ;
          pGfxVertexArray->Add( (float)sPoint.x, (float)sPoint.y, (float)sPoint.z);
        }

      return ;
    }


} // end smgfx_OutputCircularArc

#else // no SM_GFX_OUTPUT_CODE

#if defined(SM_GFX_CODE)
// Draw-callback setters stay available for SM_GFX_CODE builds that omit the
// OpenGL output layer. Nothing invokes the callbacks without that layer.
void smgfx_SetDrawLineCallBack(void (*)(double, double, double, double, double, double, SmGfxArraySet *)) { }
void smgfx_SetDrawPointCallBack(void (*)(double, double, double, SmGfxArraySet *)) { }
void smgfx_SetDrawPolyLineCallBack(void (*)(double *, long, SmGfxArraySet *)) { }
#endif // SM_GFX_CODE

#include <SmAxis2Placement.h>

static SmVector3d smgfx_StubColor()
{
  return SmVector3d(0.0, 0.0, 0.0) ;
}

void smgfx_OutputInitialize() { }
void smgfx_OutputSetShadingMode(SmBoolean) { }
void smgfx_OutputSetShader(SmBoolean, SmBoolean) { }
void smgfx_OutputLightShininess(double) { }
void smgfx_OutputLightDir(double, double, double) { }
void smgfx_OutputLightAmbient(double, double, double) { }
void smgfx_OutputLightDiffuse(double, double, double) { }
void smgfx_OutputLightSpecular(double, double, double) { }

double smgfx_GetOutputPointSize(SmGfxArraySet *) { return 0.0 ; }
double smgfx_GetOutputLineWidth(SmGfxArraySet *) { return 0.0 ; }
SmBoolean smgfx_GetOutputDashedLines(SmGfxArraySet *) { return FALSE ; }
SmVector3d smgfx_GetOutputColor(SmGfxArraySet *) { return smgfx_StubColor() ; }

SmVector3d smgfx_OutputObjectColor(const SmObject *, SmColorRuleType, SmGfxArraySet *) { return smgfx_StubColor() ; }
double smgfx_OutputPointSize(double, SmGfxArraySet *) { return 0.0 ; }
double smgfx_OutputLineWidth(double, SmGfxArraySet *) { return 0.0 ; }
SmBoolean smgfx_OutputDashedLines(SmBoolean, SmGfxArraySet *) { return FALSE ; }
SmVector3d smgfx_OutputColor(double, double, double, SmGfxArraySet *) { return smgfx_StubColor() ; }
SmVector3d smgfx_OutputColor(const SmVector3d &, SmGfxArraySet *) { return smgfx_StubColor() ; }
SmVector3d smgfx_ChangeOutputColor(SmGfxArraySet *) { return smgfx_StubColor() ; }
void smgfx_OutputPolyline(double *, long, long, SmGfxArraySet *) { }

#endif // no SM_GFX_OUTPUT_CODE
