// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPolyDecimate.h
* PURPOSE: Header file for SmPolyDecimate
**********************************************************************/

#ifndef __SMPOLYDECIMATE_H__
#define __SMPOLYDECIMATE_H__

#ifndef __SMPOLY_H__
#include <SmPoly.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SmBrepCache_H__
#include <SmBrepCache.h>
#endif

#ifndef __SMATTRIBUTE_H__
#include <SmAttribute.h>
#endif

#include <SmGraphicsExtern.h>

#ifndef __SMMATRIX_H__
#include <SmMatrix.h>
#endif

#ifndef __SMVECTORND_H__
#include <SmVectorNd.h>
#endif

/*******************************************************************//**
PURPOSE: values assigned to every vertex during a polyBrep decimation.

NOTES: SM_AI_VERTEX_DECIMATION_VALUES  - attribute id
***********************************************************************/
class SM_EXPORT SmDecimateVertexAttr : public SmAttribute
{
  friend class SmPolyDecimate ;
protected:
  // inherited from SmAttribute
  //   ULONG m_lAttributeID;                  // unique id for each attribute type - not each attribute.
  //   SmAttributeBehaviorType m_eBehavior;   // oneof SM_AB_COPY                  - one geom obj per attrib obj   - auto deleted
  //                                          //       SM_AB_REFERENCE             - many geom objs per attrib obj - auto deleted
  //                                          //       SM_AB_STANDALONE_COPY       - same as COPY      - not deleted if user_count goes to zero
  //                                          //       SM_AB_STANDALONE_REFERENCE  - same as REFERENCE - not deleted if user_count goes to zero
  //                                          //       SM_AB_TEMP                  - Not Copied, Not Persistent
  //   ULONG                   m_lMark;       // Used to mark attributes during various traversal operations.
  //                                          //   [not persistent - not written to and read from file]
  //   SmTArray<SmAObject*>    m_vUsers;      // list of all geometry objects connected to this attribute

  ULONG              m_lSortIndex ;            //      
  SmPolyVertexClass  m_eVertClass ;            // oneof: SM_PVC_NOT_COMPUTED // Not yet computed                                  
                                               //        SM_PVC_UNKNOWN      // We don't know, vert connected to a spine edge                                    
                                               //        SM_PVC_SIMPLE       // Simple interior vertex                            
                                               //        SM_PVC_BOUNDARY     // Boundary of mesh - has lamina edges               
                                               //        SM_PVC_INTERIOR     // Interior point between two feature edges          
                                               //        SM_PVC_CORNER       // Interior point between three or more feature edges
  double             m_dVertexDistance ;       // a weighted chordheight distance from vertex to baseline between 
                                               //  its feature/boundary edges. Calculated in FindDistanceToEdge as:
                                               //        vert->IsBoundary && m_dBoundaryEdgeWeight <= 0.0 ? SM_BIG_DOUBLE
                                               //      : vert->isBoundary                                 ? chordheight/m_dBoundaryEdgeWeight + m_dMaxFaceError
                                               //      : vert->IsInterior && m_dInteriorEdgeWeight <= 0.0 ? SM_BIG_DOUBLE
                                               //      : vert->IsInterior                                 ? chordheight/m_dInteriorEdgeWeight + m_dMaxFaceError
                                               //      : SM_ERR
  double             m_dMaxFaceError ;         // Max(Face[i]->Error)
                                               // Face[i]->Error = measure of max distance from decimationFace to existing polyBrep
                                               //   as computed in CheckErrorBounds()
  SmVector3d         m_vAveragePlaneNormal ;   // Sum_i(Face[i]->Normal * Face[i]->Area) / Sum_i(Face[i]->Area)
                                               //   where Face[i] are the set of faces attached to this vertex                         

public:
  // constructor
  SmDecimateVertexAttr
  (
    ULONG                   lAttributeID,
    SmAttributeBehaviorType eBehavior = SM_AB_COPY
  ) 
    : SmAttribute( lAttributeID, eBehavior ),
    m_lSortIndex( 0 ),
    m_eVertClass( SM_PVC_NOT_COMPUTED ),
    m_dVertexDistance( -SM_BIG_DOUBLE ),
    m_dMaxFaceError( 0.0 ),
    m_vAveragePlaneNormal( 0, 0, 0 )
  {}

  // virtual copy
  virtual SmAttribute * MakeCopy( const SmContext & crContext ) const;

  // destructor
  virtual ~SmDecimateVertexAttr()
  {
    m_lSortIndex = SM_UNDEF_ULONG;
    m_eVertClass = SM_PVC_NOT_COMPUTED;
    m_dVertexDistance = -SM_BIG_DOUBLE;
    m_dMaxFaceError = -SM_BIG_DOUBLE;
    m_vAveragePlaneNormal.SetUninitialized();
  }

  

  // simple data access
  virtual ULONG          GetNumLongElements()          const { return 1 ; }
  virtual ULONG          GetNumDoubleElements()        const { return 4 ; }
  virtual ULONG          GetNumCharacterElements()     const { return 0 ; }
  virtual const long   * GetLongElementsAddress()      const { return (const long *)&m_eVertClass ; }
  virtual const double * GetDoubleElementsAddress()    const { return (double*)&m_dVertexDistance ; }
  virtual const char   * GetCharacterElementsAddress() const { return NULL ; }

  double                 GetVertexDistance()           const { return m_dVertexDistance ; }

  // get memory used and allocated
  virtual ULONG GetMemoryUsed( ULONG &rlMemoryAllocated ) const
  {
    ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed( lAllocated );
    rlMemoryAllocated = sizeof( this ) + lAllocated - sizeof( SmTArray<SmAObject*> );
    return(sizeof( this ) + lUsed - sizeof( SmTArray<SmAObject*> ));
  }

  // pretty print
  virtual void Dump() const ;

} ; // end class SmDecimateVertexAttr

/*******************************************************************//**
PURPOSE: This attribute resides on an edge and contains the quadric
   that encodes the planes of the original polygons of the vertex or
   decendents thereof.

NOTES: SM_AI_QUADRIC_VERTEX_VALUES - attribute id
***********************************************************************/
class SM_EXPORT SmQuadricVertexAttr : public SmAttribute
{
  friend class SmPolyDecimate;
protected:
  // Add fields necessary to do quadric decimation
  SmVector3d m_Q[3];     // Quadratic Error Matrix = Sum(Area/3 * (Normalt.Normal)),  Normal = [a b c], the triangle normal vector     
  SmVector3d m_b;        // b vector               = Sum(Area/3 * -(Normal . Center-Origin) * Normal) 
  double     m_c;        // c value                = Sum(Area/3 * (Center-Origin . Center-Origin)) 

  // optimal point = -(Inv(m_Q) * m_b)        in SmQuadricVertexAttr::ComputeOptimalPosition()
  // quadric error = NewVert . m_b + m_c      in SmQuadricVertexAttr::ComputeError()
      
public:

  // constructor
  SmQuadricVertexAttr
  (
    ULONG                   lAttributeID,
    SmAttributeBehaviorType eBehavior = SM_AB_COPY
  )
    : SmAttribute( lAttributeID, eBehavior )
  {
    m_Q[0].Set( 0, 0, 0 );
    m_Q[1].Set( 0, 0, 0 );
    m_Q[2].Set( 0, 0, 0 );
    m_b.Set( 0, 0, 0 );
    m_c = 0.0;
  }

  // virtual copy function
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const;

  // destructor
  virtual ~SmQuadricVertexAttr()  { }

  SmQuadricVertexAttr& operator=( SmQuadricVertexAttr const &obj )
  {
      if ( &obj == this ) return *this;
      m_Q[0] = obj.m_Q[0];
      m_Q[1] = obj.m_Q[1];
      m_Q[2] = obj.m_Q[2];
      m_b = obj.m_b;
      m_c = obj.m_c;
      return *this;
  }

  // accumulate triangle weights into m_Q, m_b, and m_c values
  //    m_Q matrix += Area/3 * (Normal.Normalt),  Normal = [a b c], the triangle unit normal vector   
  //    m_b vector += Area/3 * -(Normal . Center-Origin) * Normal                                     
  //    m_c vector += Area/3 * (Center-Origin . Center-Origin)                                        
  SmStatus AddWeightedTriangle
  (
    const SmVector3d & crFaceNormal,        ///< [in ]:target triangle unit normal  <br>
    const SmPoint3d & crCenter,             ///< [in ]:target triangle center       <br> 
    double dArea                            ///< [in ]:target triangle area         <br>
  );

  // combine two Quadric error forms into a one as [result.vals = this->vals + other.vals] 
  SmStatus Union
  (
    const SmQuadricVertexAttr & crOther,    ///< [in ]:other  of result.vals = this->vals + other.vals   <br>
    SmQuadricVertexAttr       & rResult     ///< [out]: result of result.vals = this->vals + other.vals  <br>
  ) const;

  // compute optimal point for current error form data as [OptimalPt = -(Inv(m_Q) * m_b)]
  SmStatus ComputeOptimalPosition
  (
    SmPoint3d & rOptimal,                   ///< [out]: computed optimal point = -(Inv(m_Q) * m_b)   <br>
    SmBoolean & bFailsToCompute             ///< [out]: TRUE = m_Q is not invertible - no result     <br>
                                            ///<      : FALSE= result is good                        <br>
  ) const;

  // compute error for current error data as [error = NewVert . m_b + m_c]
  SmStatus ComputeError
  (
    const SmPoint3d & crNewVertex,          ///< [in ]: target vertex                   <br>
    double          & rdError               ///< [out]: error = NewVert . m_b + m_c     <br>
  ) const;
  
  // get memory used and allocated
  virtual ULONG GetMemoryUsed( ULONG & rlMemoryAllocated ) const
  {
    ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed( lAllocated );
    rlMemoryAllocated = sizeof( this ) + lAllocated - sizeof( SmTArray<SmAObject*> );
    return(sizeof( this ) + lUsed - sizeof( SmTArray<SmAObject*> ));
  }

  // pretty print
  virtual void Dump() const;

} ; // end class SmQuadricVertexAttr

/*******************************************************************//**
PURPOSE: This attribute resides on the edge during decimation and
   tells the error and resulting point for the new vertex upon 
   contraction of the edge.

NOTES:  ID = SM_AI_QUADRIC_EDGE_VALUES
***********************************************************************/
class SM_EXPORT SmQuadricEdgeAttr : public SmAttribute
{
  friend class SmPolyDecimate ;
protected:
  // Add fields necessary to do quadric decimation
  SmPoint3d  m_vNewPoint ;    // New point computed somehow - from quadric,
                              // from average, by taking one point
  double     m_dError ;       // Error associated with this point
  ULONG      m_lSortIndex ;   // Help in sorting

public:
  // constructor
  SmQuadricEdgeAttr
  (
    ULONG lAttributeID,
    SmAttributeBehaviorType eBehavior = SM_AB_COPY
  )
    : SmAttribute( lAttributeID, eBehavior ),
    m_vNewPoint( 0, 0, 0 ),
    m_dError( 0.0 )
  {}

  // virtual copy
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const ;

  // destructor
  virtual ~SmQuadricEdgeAttr()                         { }

  // simple data access
  SmPoint3d GetPoint() const                           { return m_vNewPoint ; }
  double    GetError() const                           { return m_dError ; }
  ULONG     GetSortIndex() const                       { return m_lSortIndex ; }

  void      SetPoint    (const SmPoint3d & crNewPoint) { m_vNewPoint = crNewPoint ; }
  void      SetError    (double dError)                { m_dError = dError ; }
  void      SetSortIndex(ULONG lSortIndex)             { m_lSortIndex = lSortIndex ; }

  // get memory used and allocated
  virtual ULONG GetMemoryUsed( ULONG &rlMemoryAllocated ) const
  {
    ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed( lAllocated );
    rlMemoryAllocated = sizeof( this ) + lAllocated - sizeof( SmTArray<SmAObject*> );
    return(sizeof( this ) + lUsed - sizeof( SmTArray<SmAObject*> ));
  }

  // pretty print
  virtual void Dump() const ;

} ; // end class SmQuadricEdgeAttr

/*******************************************************************//**
PURPOSE: Represents a polygon decimator. 

NOTES: The decimation process shall find and remove vertices
                that satisfy the decimation criteria which include: 
 (1) each vertex to be removed is surrounded by a complete cycle
     of triangles (each edge that uses the vertex is used by either one
     (non-manifold) or two triangles, 
 (2) the distance from the vertex to the average plane of the 
     surrounding triangles meets the minimum distance criteria, and 
 (3) the 'hole' after vertex-removal shall be retriangulated by a 
     local mesh which satisfiy the given error bound constrains 
     (i.e. the polygons after decimation should lie between a
          +offset and a -offset of the original polygons).
  GWC: Does filling the hole insert new vertices?

  GWC: I don't know what algorithm is described above.  I read DoQuadricDecimation
       which works differently.
  DoQuadricDecimation
  (1) Assign every edge an error value (I'm unclear what and how that error value means)
  (2) Cleverly sort the edges so that the smallest edges are targeted first.
  (3) Remove one edge at a time with a call to SqueezeEdge.
      SqueezeEdge conceptually removes an edge by squeezing the edge
       and it's end vertices to zero length and the edge and one vertex is 
       removed along with the faces (and their edges) connected to edge  
       which become degenerate as the edge shrinks to zero length.
       The remaining geometry is hooked together.
  (4) stop removing edges when m_dPercentOfReduction or the m_dMaximumError
       stopping criteria are met.

***********************************************************************/
class SM_EXPORT SmPolyDecimate
{
protected:
  SmPolyBrep * m_pPolyBrep ;            // Target polybrep
  double       m_dPercentOfReduction ;  // user input decimation stopping criteria;
                                        //      desired percentage of head vertices to remove from m_pPolyBrep, e.g. 0.2 = 20%.
                                        //      0.0 to ignore.
  double       m_dMaximumError ;        // user input decimation stopping crieria;
                                        //      maximum allowed accumulated error at a vertex before it 
                                        //      can no longer be moved during decimation. When all
                                        //      vertex accumulated errors are larger than this, decimation stops.
                                        //      0.0 to ignore.
                                        //      Vertex (and edge) accumulated error are defined and implemented
                                        //      attribute classes SmQuadricVertexAttr and SmQuadricEdgeAttr.
  double       m_dMinFeatureAngle ;     // Minimum dihedral angle in degrees (nonPlanarity) between two adjacent  
                                        //      triangles which is used in determining 'feature edge'
  double       m_dInteriorEdgeWeight ;  // Value used to weight InteriorEdge chordheight distances.
                                        //        Smaller distance edges are decimated before larger distance edges.
                                        //      0.0 or negative: distance = SM_BIG_DOUBLE, i.e. no interior edges will be removed.
                                        //      positive: distance = dChordHeight/m_dInteriorEdgeWeight + pAttr->m_dMaxFaceError.
                                        //      ex. 1.0 = chord height will be used in the same way as the vertex
                                        //                distance to a normal plane for simple vertices.
                                        //      ex. 0.5 = chord height will be scaled by 2.0 before
                                        //                being compared to normal plane distances.
                                        //      An interior edge may be detected using the MinFeatureAngle or
                                        //      using surface normals or UV coordinates.  
                                        //      It is basically an edge between two surfaces.
  double       m_dBoundaryEdgeWeight ;  // Value used to weight BoundaryEdge chordheight distances.
                                        //        Smaller distance edges are decimated before larger distance edges.
                                        //      0.0 or negative: distance = SM_BIG_DOUBLE, i.e. no boundary edges will be removed.
                                        //      positive: distance = dChordHeight/m_dBoundaryEdgeWeight + pAttr->m_dMaxFaceError.
  SmBoolean    m_bUseEdgeLengthAsError ;// TRUE  = decimation removes edges based on edge length, stopping 
                                        //              at either the percentage of reduction or the m_dMaximumError.
                                        //      FALSE = decimation removes edges based on quadric error + weight * perimeter.
                                        //        note: This only applies to quadric decimation.
  SmVector3d * m_pEyeOrViewVector ;     // orientation data for view-based decimation, 
                                        //      NULL = no view-based decimation.
  SmBoolean    m_bParallelProjection ;  // TRUE  = m_pEyeOrViewVector represents the view vector,
                                        // FALSE = m_pEyeOrViewVector represents the eye position

public:
  // constructor
  SmPolyDecimate
  (
    SmPolyBrep * pPolyBrep,                    ///< [in ]: target polyBrep to decmiate                                                   <br>
    double       dPercentOfReduction  =0.0,    ///< [in ]: Percent of edges to remove, 0.0 to ignore, default:[0.0]                      <br>
    double       dMaximumError        =0.0,    ///< [in ]: Max error allowed to move a vertex, 0.0 to ignore, default:[0.0]              <br>
    SmBoolean    bUseEdgeLengthAsError=FALSE,  ///< [in ]: TRUE:Error=EdgeLength, FALSE:Error=QuadricError, default:[FALSE]              <br>
    double       dMinFeatureAngDeg    =30.0,   ///< [in ]: min dihedral ang between faces for a feature edge, default:[30]               <br>
    double       dInteriorEdgeWeight  =1.0,    ///< [in ]: Weight interior edges to favor or avoid decimating them over boundary edges.  <br>
                                               ///<      : 0.0 or neg: Don't decimate Interior edges                                     <br>
                                               ///<      : smaller distance edges are decimated before larger distance edges.            <br>
                                               ///<      : pos: distance = dChordHeight/m_dInteriorEdgeWeight + pAttr->m_dMaxFaceError.  <br>
                                               ///<      : values from 0 to 1 favor decimatiing interior edges first                     <br>
                                               ///<      : values larger than 1 avoid decimating interior edges.                         <br>
    double       dBoundaryEdgeWeight  =1.0     ///< [in ]: Weight boundary edges to favor or avoid decimating them over interior edges.  <br>
                                               ///<      : 0.0 or neg: Don't decimate Boundary edges                                     <br>
                                               ///<      : pos: distance = dChordHeight/m_dInteriorEdgeWeight + pAttr->m_dMaxFaceError.  <br>
                                               ///<      : values from 0 to 1 favor decimatiing Boundary edges first                     <br>
                                               ///<      : values larger than 1 avoid decimating Boundary edges.                         <br>
  );

  SmStatus CheckErrorBounds
  (
    SmPolyFace  * pNewFace,                    ///< [in ]: target face to evaluate and modify with a SM_AI_POLY_ERRORBOUNDS attribute    <br>
    SmBrepCache * pPolyCache,                  ///< [in ]: contains vertices/edges/faces to which NewFace sample points are projected    <br>
    SmBoolean  & rbWithinErrBounds             ///< [out]: TRUE: m_dMaximumError == 0.0 or                                               <br>
                                               ///<      : min/max NewFace sample distances to PolyCache objects < +/-m_dMaximumError    <br>
  ) ;

  SmStatus ComputeVertexAttributes
  (
    SmPolyVertex          * pVert,
    SmDecimateVertexAttr *& rpNewAttr
  ) ;

  SmQuadricEdgeAttr * GetOrCreateQuadricEdgeAttr
  (
    SmPolyEdge         * pEdge,                ///< [in ]: target edge                       <br>
    SmQuadricEdgeAttr *& rpNewEdgeAttr         ///< [out]: attribute attached to pEdge       <br>
  ) ;

  SmStatus DoMinimumEdgeLengthDecimation
  (
    double dMinEdgeLength,
    double dOptMinAspectRatio=1.0/15.0
  ) ;

  SmStatus DoMeasuredDecimation() ;

  // very fast decimate using edge contraction to reduce polygon count
  SmStatus DoQuadricDecimation
  (
    SmVector3d * pOptEyeOrViewVector=NULL,       ///< [in ]: orientation data for view-based decimation,             <br>
                                                 ///<      : NULL = no view-based decimation.                        <br>
    SmBoolean    bOptIsParallelProjection = TRUE ///< [in ]:TRUE  = m_pEyeOrViewVector represents the view vector,   <br>
                                                 ///<      : FALSE = m_pEyeOrViewVector represents the eye position  <br>
  );                                          

  SmStatus DivideAndDecimate
  (
    SmTriangleBag      * pTriangleBag,
    const SmExtent3d   & crBBox,
    SmPolygonSLAOutput & rSLP,
    ULONG                lTargetSize
  ) ;

  SmStatus DoLargeDecimation
  (
    const TCHAR * cInputSTLFileName, 
    SmFileType    eInputFileType,
    const TCHAR * cOutputSTLFileName,
    SmFileType    eOutputFileType,  
    ULONG         lTargetSize
  ) ;

  SmStatus FillHole
  (
    SmPolyBrep                      * pPolyBrep,                  ///< [in ]:Hole filled in this brep                                        <br>
    SmPolyVertex                    * pVertToBeRemoved,           ///< [in ]:Vertex to be removed that will leave hole.                      <br>
    SmDecimateVertexAttr            * pAttr,                      ///< [in ]:Decimation attributes of this vertex.                           <br>
    SmBoolean                       & rbSuccess,                  ///< [out]: Did we successfully fill the hole                              <br>
    SmTArray<SmPolyFace*>           & rOrigPolys,                 ///< [in ]:Original polygons,                                              <br>
    SmTArray<SmPolyFace*>           & rNewPolys,                  ///< [in ]:New Polygons created to fill the hole                           <br>
    SmTArray<SmPolyVertex*>         & rSurvivingLoopVertices,     ///< [in ]:Vertices which will survive the destruction of the rNewPolys    <br>
    SmTArray<SmDecimateVertexAttr*> & rSurvivingLoopAttr          ///< [out]: Attributes corresponding to rSurvivingLoopVertices             <br>
  ) ;

  SmStatus FindDistanceToEdge
  (
    SmPolyVertex * pVert,                ///< [in ]: target vertex 
    double       & rdDistanceToEdge      ///< [out]: vert->IsBoundary && m_dBoundaryEdgeWeight <= 0.0 ? SM_BIG_DOUBLE                                       <br>
                                         ///<      : vert->isBoundary                                 ? chordheight/m_dBoundaryEdgeWeight + m_dMaxFaceError <br>
                                         ///<      : vert->IsInterior && m_dInteriorEdgeWeight <= 0.0 ? SM_BIG_DOUBLE                                       <br>
                                         ///<      : vert->IsInterior                                 ? chordheight/m_dInteriorEdgeWeight + m_dMaxFaceError <br>
                                         ///<      : SM_ERR ? SM_BIG_DOUBLE                                                                                 <br>
  ) ;

  SmPolyVertexClass FindVertexClass
  (
    SmPolyVertex          * cpPolyVertex,                   ///< [in ]: target vertex to classify                                         <br>
    SmTArray<SmPolyEdge*> * pFeatureOrBoundaryEdges = NULL, ///< [out]: opt list of lamina/feature/silhouette edges connected to vertex.  <br>
                                                            ///<      : NULL to ignore, default:[NULL]                                    <br>
    SmBoolean             * pbNewDVAttr = NULL              ///< [out]: TRUE if a new DecimateVertexAttr was created                      <br>
  ) ;

  SmStatus FindVertexAveragePlaneDistance                   ///< [in ]: target vertex                                                     <br>
  (                                                         ///< [out]: projected distance to average plane of all faces about vertex     <br>
    SmPolyVertex * pVert,                                   ///<      : augmented with a MaxFaceError value                               <br>
    double       & rdDistToCenter

  ) ;

  SmQuadricEdgeAttr * FindAttributeOnAnyEdge(SmPolyEdge * pEdge) const ;

  void                RemoveAttributeOnAnyEdge(SmPolyEdge *pEdge) const ;

  ULONG               FindAllAttributesOnAnyEdge
  (
    SmPolyEdge * pEdge,                                    ///< [in ]: target PolyEdge                        <br>
    SmTArray<SmQuadricEdgeAttr *> *pOptEdgeAttrs = NULL    ///< [out]: optional list of all found attributes  <br>
  ) const ;

  static SmStatus GetTriangularFaceData
  (
    const SmPolyFace * cpPolyFace,               ///< [in ]:target polyface                                                           <br>
    SmVector3d       & rFaceNormal,              ///< [out]: normal = unitized( cross(Edge0Dir, -Edge2Dir) ) (degNormal = [0,0,1])    <br>
    SmPoint3d        & rCenter,                  ///< [out]: center = (Pt0 + Pt1 + Pt2) / 3.0                                         <br>
    double           & rdArea                    ///< [out]: area   = (dBaseLength * dHeight) / 2.0; (degArea = 0.0)                  <br>
  ) ;

  void SetMaximumError   (double dMaximumError)       { m_dMaximumError = dMaximumError ; }

  void SetMinFeatureAngle(double dMinFeatureAngle)    { m_dMinFeatureAngle = dMinFeatureAngle ; }

  SmStatus SplitFace
  (
    SmPolyFace            * pFace,
    SmTArray<SmPolyFace*> & rNewFaces
  ) ;

  SmStatus SqueezeEdge
  (
    SmPolyEdge                   * pEdgeToBeRemoved,      ///< [in ]: target edge                                                        <br>
    SmQuadricEdgeAttr            * pAttr,                 ///< [in ]: its SM_AI_QUADRIC_EDGE_VALUES attribute containing                 <br>
                                                          ///<      : the point loc to be given to the SqueezedEdge->SurvivingVertex     <br>
    SmBoolean                    & rbSuccess,             ///< [out]: TRUE = edge removed, FALSE = not removed                           <br>
    SmTArray<SmPolyEdge*>        & rMovedEdges,           ///< [out]: list of edges moved                                                <br>
    SmTArray<SmQuadricEdgeAttr*> & rMovedEdgesAttrs       ///< [out]: their associated attributes                                        <br>
  ) ;
  
  SmBoolean TestFeatureEdge   (const SmPolyEdge * cpPolyEdge) ;

  SmBoolean TestSilhouetteEdge(const SmPolyEdge * cpPolyEdge) ;

  SmStatus ValidateQuadric() ;

} ; // end class SmPolyDecimate


#endif // !__SMPOLYDECIMATE_H__
