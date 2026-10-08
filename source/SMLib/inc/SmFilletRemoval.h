// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFilletRemoval.h 
* PURPOSE: Header file for SmFilletRemoval.
* 07-Jul-08 - bd - initial version
**********************************************************************/

#ifndef __SMFILLETREMOVAL_H__
#define __SMFILLETREMOVAL_H__

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMVECTOR2D_H__
#include <SmVector2d.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMMESSAGES_H__
#include <SmMessages.h>
#endif

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMOS_MATH_H__
#include <SmMath.h>
#endif

class SmFace;
class SmEdge;
class SmVertex;
class SmBSplineCurve;

/*******************************************************************//**
PURPOSE: Abstract base class for removing a Face from a Brep, and patching the hole.

NOTES: Currently implemented only for Fillet faces.
***********************************************************************/

class SM_EXPORT SmFaceRemoval
{
protected:
  SmFace *m_pFaceToDelete;
  double  m_d3dTol;

protected:
  SmFaceRemoval() {}

public:
  // Constructor:
  SmFaceRemoval( SmFace *pFaceToDelete, double d3dTolerance )
      : m_pFaceToDelete( pFaceToDelete ),
        m_d3dTol( d3dTolerance )
  {
    SM_ASSERT( pFaceToDelete != NULL );
    SM_ASSERT( d3dTolerance  > SM_EFF_ZERO );
  }

  // Destructor:
  virtual ~SmFaceRemoval() {}

  // Methods:
  virtual SmStatus RemoveFace() = 0;

protected:
  // A utility routine:
  SmStatus MoveVtxAndEdgeToAdjacentFace( SmVertex *pVtxToMove, SmEdge *pEdgeToMove, SmFace *pFromFace );

}; // end base class SmFaceRemoval


/*******************************************************************//**
PURPOSE: Helper class for class SmFilletRemoval:
   store information for each corner of the Fillet Face.

NOTES:
  At each corner, there is:
  - one original vertex;
  - one side edge coming into the vertex;
  - one side (tangent) face and one end face.
  The new vertex will be merged with the one on the opposite corner.

  If the curve of the side edge is long enough to reach where the new
  vertex will be (which is a srf/srf/srf intersection of the end face
  and the two side faces), then the parameter of that intersection
  will be recorded in m_dNewSideEdgeParam.  Otherwise, the curve will
  be extended along the intersection of the tangent face and the end face,
  and the extension piece will be recorded in the three curves (one 3d,
  and one in each surface's parameter space).  So, if the curves are
  non-null, use them, otherwise just extend the Edge's parameter range
  to m_dNewSideEdgeParam.

  In the usual case, there are three Edges incident on the corner vertex.
  There can be more than three however.  In that case, both vertices and
  the end edge at that end are retained.  The vertex with more than three
  edges, and the end edge, are moved from the Fillet face to the tangent
  face.  This case is flagged by this class's NoSideEdge() method.
***********************************************************************/
class SM_EXPORT SmFilletCornerInfo
{
public:
  SmVertex *m_pVertex;

  SmPoint2d m_sUVFilSurf;
  SmPoint2d m_sUVTanSurfOld;
  SmPoint2d m_sUVEndSurfOld;
  SmPoint2d m_sUVTanSurfNew;

  // Side edge curve extension: if Null, use m_dNewSideEdgeParam.
  SmBSplineCurve *m_pNewSideEdgeExtension3d;
  SmBSplineCurve *m_pNewSideEdgeExtEndFaceUV;
  SmBSplineCurve *m_pNewSideEdgeExtTanFaceUV;

  SmEdge   *m_pSideEdge;
  SmBoolean m_bHighEndOfSideEdge;
  double    m_dNewSideEdgeParam; // param of srf/srf/srf int pt on side edge curve.

  // Constructor and destructor.
  SmFilletCornerInfo();
  ~SmFilletCornerInfo();

  SmBoolean NoSideEdge() { return m_pSideEdge == NULL; }

  
}; // end helper class SmFilletCornerInfo

/*******************************************************************//**
PURPOSE: Derived class for removing a Fillet Face from a Brep, and patching the hole.

NOTES:
   d3DTol is for checking the results of srf/srf/srf intersection,
     and for passing to PointBasedSurfaceIntersect().

  Assumptions about the input Face:
  - four-sided Face,
  - two tangent Edges (opposite each other),
  - two non-tangent Edges.
  If any of these is violated, the RemoveFace() method will return SM_ERR
  and the Brep will not be modified.

  Terminology:
  - 'Ends'  : these correspond to the two non-tangent Edges.
  - 'Sides' : these correspond to the two tangent Edges.
***********************************************************************/

class SM_EXPORT SmFilletRemoval: public SmFaceRemoval
{
protected:
  SmEdge * m_aTanEdges[2];
  SmFace * m_aTanFaces[2];
  SmEdge * m_aEndEdges[2];
  SmFace * m_aEndFaces[2];

  SmSurfParamType m_eRailParam;    // u or v runs along the length of the fillet.
  SmBoolean       m_bEndsSwapped;  // if True, then [0] end is high, [1] is low.
  SmBoolean       m_bSidesSwapped; // same for the two sides.

  SmPoint3d       m_aNewVtxPts[2];
  double          m_aNewVtxTols[2];
  SmPoint2d       m_aUVEndSurfNew[2];
  double          m_dNewEdgeDeviation;
  SmBSplineCurve *m_pNewEdgeCurve3d;
  SmBSplineCurve *m_aNewEdgeCurveUVs[2];

  // Four CornerInfo objects:
  //  1st subscript: the two Fillet ends, 0 and 1;
  //  2nd subscript: Fillet tangent sides 0 and 1.
  //    e.g. CornerInfo[ which end ][ which side ]
  SmFilletCornerInfo m_aCornerInfo[2][2];

  // Methods:


public:
  // Constructor:
  SmFilletRemoval( SmFace *pFaceToDelete, double d3dTolerance );

  // Destructor:
  virtual ~SmFilletRemoval();

  virtual SmStatus RemoveFace();

protected:
  SmStatus CollectFilletTopology();
  SmStatus CalcNewCornerVertices(  ULONG lWhichEnd );
  SmStatus FindSideEdgeExtensions( ULONG lWhichEnd, ULONG lWhichSide );
  SmStatus IntersectTangentSurfaces();
  SmStatus DoRemoval();

}; // end derived class SmFilletRemoval

#endif // !__SMFILLETREMOVAL_H__

