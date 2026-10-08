// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTopologySolver.h
* PURPOSE: Header file for SmTopologySolver object.
**********************************************************************/

#ifndef __SMTOPOLOGYSOLVER_H__
#define __SMTOPOLOGYSOLVER_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMGLOBALSOLVER_H__
#include <SmGlobalSolver.h>
#endif

#ifndef __SMMAPPTRTOPTR_H__
#include <SmMapPtrToPtr.h>
#endif


class SmTopologySolver;
class SmShape;

/*******************************************************************//**
PURPOSE: These classes provides the interface for local solutions of
    for two objects.  

NOTES: These classes are used only by SmTopologySolver.
***********************************************************************/
class SmTopoLocalSolver
{
  protected:
    SmTopologySolver & m_rTopologySolver ;  // pointer to owner object 
    SmBoolean          m_bSwapOrder ;       // TRUE = object order passed to SolveIt() are reversed
    const SmShape    * m_pShape ;           // Shape being queried - used to skip 
                                            // sibling members of composite objects.

  public:
    // constructor/destructor
    SmTopoLocalSolver(SmTopologySolver & rTopologySolver,
                      SmBoolean          bSwapOrder,
                      const SmShape      * pShape)
                     : m_rTopologySolver( rTopologySolver ),
                       m_bSwapOrder( bSwapOrder ),
                       m_pShape( pShape )
                     { }

    virtual ~SmTopoLocalSolver() { m_pShape = NULL; }

    SmTopologySolver &GetTopologySolver() { return m_rTopologySolver ; }

    // Solve function
    virtual SmStatus SolveIt( const SmObject * /*pObj1*/, const SmObject * /*pObj2*/ )
    {
      SE( SM_ERR ); return SM_ERR;
    }

} ; // end class SmTopoLocalSolver

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSPoint : public SmTopoLocalSolver
{
  protected:
    SmPoint3d         m_vPoint;
    const SmVertex  * m_pPointVertex;    // is this always nonNULL so it can be used to fetch a SrcZoneTol3d value?
                                         // if not, this needs to store a SrcZoneTol3d for obj assoc with m_vPoint

  public:
    SmTLSPoint
    ( 
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape 
    )
      : SmTopoLocalSolver( rTopologySolver, bSwapOrder, pShape ),
      m_pPointVertex( NULL )
    {}

    virtual ~SmTLSPoint()   { m_pPointVertex = NULL ; }

    void SetPoint( const SmPoint3d & crPoint )
    {
      m_vPoint = crPoint;
    }

    void SetPointVertex( SmVertex * pPointVertex )
    {
      m_pPointVertex = pPointVertex;
    }

    virtual SmStatus SolveIt( const SmObject * /*pObj1*/, const SmObject * /*pObj2*/ )
    {
      SE( SM_ERR ); return SM_ERR;
    }

} ; // end class SmTLSPoint


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSPointVertex : public  SmTLSPoint
{
  private:
  public:
    SmTLSPointVertex
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSPoint( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSPointVertex() {}

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSPointVertex

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSVertexVertex : public SmTLSPointVertex
{
  private:
  public:
    SmTLSVertexVertex
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSPointVertex( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSVertexVertex() {}

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSVertexVertex

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSPointCurve : public SmTLSPoint
{
    friend class SmTLSPointFace;
    friend class SmTLSCurveVertex;

  protected:
    const SmObject * m_pCurveOwner;     //      
    SmExtent1d       m_vInterval;       // limit interval for target curve 
    const SmCurve  * m_pSurfaceCurve;   //      

  public:
    SmTLSPointCurve
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSPoint( rTopologySolver, bSwapOrder, pShape ),
      m_pCurveOwner( NULL ),
      m_pSurfaceCurve( NULL )
    {}

    virtual ~SmTLSPointCurve()
    {
      m_pCurveOwner = NULL;
      m_pSurfaceCurve = NULL;
    }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSPointCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSPointEdge : public SmTLSPointCurve
{
  private:
  public:
    SmTLSPointEdge
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSPointCurve( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSPointEdge() {}
    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSPointEdge


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSVertexEdge : public SmTLSPointEdge
{
   private:
   public:
     SmTLSVertexEdge
     ( 
       SmTopologySolver & rTopologySolver,
       SmBoolean          bSwapOrder,
       const SmShape    * pShape 
     )
       : SmTLSPointEdge( rTopologySolver, bSwapOrder, pShape )
     {}

    virtual ~SmTLSVertexEdge() { }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSVertexEdge

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSPointFace : public SmTLSPoint
{
  private:
  public:
    SmTLSPointFace
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSPoint( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSPointFace() { }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSPointFace

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSVertexFace : public SmTLSPointFace
{
  private:
  public:
    SmTLSVertexFace
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSPointFace( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSVertexFace() { }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSVertexFace

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSCurve : public SmTopoLocalSolver
{
    friend class SmTLSCurveFace;

  protected:
    const SmCurve    * m_pCurve;
    SmExtent1d         m_vInterval;
    const SmEdge     * m_pCurveEdge;     // is one of these always required to be NonNULL?
    const SmFace     * m_pCurveFace;     // if so they can be used to find sSrcZoneTol3d - otherwise store a ZoneTol3d val.
    const SmCurve    * m_pSurfaceCurve;
  
  public:
    SmTLSCurve
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTopoLocalSolver( rTopologySolver, bSwapOrder, pShape ),
      m_pCurve( NULL ),
      m_pCurveEdge( NULL ),
      m_pCurveFace( NULL ),
      m_pSurfaceCurve( NULL )
    {}

    virtual ~SmTLSCurve()
    {
      m_pCurve = NULL;
      m_pCurveEdge = NULL;
      m_pCurveFace = NULL;
      m_pSurfaceCurve = NULL;
    }


    void SetCurve( const SmCurve * pCurve, const SmExtent1d & crInterval )
    {
      m_pCurve = pCurve;
      m_vInterval = crInterval;
    }

    void SetCurveEdge( SmEdge * pCurveEdge )
    {
      m_pCurveEdge = pCurveEdge;
    }

    virtual SmStatus SolveIt( const SmObject * /*pObj1*/, const SmObject * /*pObj2*/ )
    {
      SE( SM_ERR ); return SM_ERR;
    }

} ; // end class SmTLSCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSCurveVertex : public SmTLSCurve
{
  private:
  public:
    SmTLSCurveVertex
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSCurve( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSCurveVertex() { }

    virtual SmStatus SolveIt(const SmObject *pObj1,   // NotUsed: in :
                             const SmObject *pObj2);  // in :

} ; // end class SmTLSCurveVertex

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSEdgeVertex : public SmTLSCurveVertex
{
  private:
  public:
    SmTLSEdgeVertex
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSCurveVertex( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSEdgeVertex() { }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSEdgeVertex

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSCurveCurve : public SmTLSCurve
{
    friend class SmTLSCurveEdge;
    friend class SmTLSCurveFace;

  private:   
    const SmCurve * m_pCurve2;
    SmExtent1d      m_vInterval2;
    const SmEdge  * m_pCurveEdge2;
    const SmFace  * m_pCurveFace2;
    const SmCurve * m_pSurfaceCurve2;
  
  public:
    SmTLSCurveCurve
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSCurve( rTopologySolver, bSwapOrder, pShape ),
      m_pCurve2( NULL ),
      m_pCurveEdge2( NULL ),
      m_pCurveFace2( NULL ),
      m_pSurfaceCurve2( NULL )
    {}

    virtual ~SmTLSCurveCurve()
    {
      m_pCurve2 = NULL;
      m_pCurveEdge2 = NULL;
      m_pCurveFace2 = NULL;
      m_pSurfaceCurve2 = NULL;
    }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSCurveCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSCurveEdge : public SmTLSCurveCurve
{
  private:
  public:
    SmTLSCurveEdge
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSCurveCurve( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSCurveEdge() { }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSCurveEdge

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSEdgeEdge : public SmTLSCurveEdge
{
  private:
  public:
    SmTLSEdgeEdge
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSCurveEdge( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSEdgeEdge() { }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSEdgeEdge

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSCurveFace : public SmTLSCurve
{
    friend class SmTLSFaceFace;
  private:
  public:
    SmTLSCurveFace
    (
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape
    )
      : SmTLSCurve( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSCurveFace() { }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSCurveFace

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSEdgeFace : public SmTLSCurveFace
{
  private:
  public:
    SmTLSEdgeFace
    ( 
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape 
    )
      : SmTLSCurveFace( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSEdgeFace() { }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSEdgeFace

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmTLSFaceFace : public SmTopoLocalSolver
{
  private:
  public:
    SmTLSFaceFace
    ( 
      SmTopologySolver & rTopologySolver,
      SmBoolean          bSwapOrder,
      const SmShape    * pShape 
    )
      : SmTopoLocalSolver( rTopologySolver, bSwapOrder, pShape )
    {}

    virtual ~SmTLSFaceFace() { }

    virtual SmStatus SolveIt(const SmObject *pObj1, const SmObject *pObj2);

} ; // end class SmTLSFaceFace

/*******************************************************************//**
PURPOSE: The topology solver class provides an interface and mechansim
    to solve global problems using topology objects.

NOTES: 
***********************************************************************/
class SM_EXPORT SmTopologySolver : public SmGlobalSolver
{
  friend class SmTree;
  friend class SmTLSPointVertex;
  friend class SmTLSPointCurve;
  friend class SmTLSPointEdge;
  friend class SmTLSPointFace;
  friend class SmTLSCurveVertex;
  friend class SmTLSCurveEdge;
  friend class SmTLSCurveCurve;
  friend class SmTLSCurveFace;
  friend class SmTLSFaceFace;

private:
  SmTopoLocalSolver      * m_pLocalSolver;      // contains derived SolveIt() function

public:
  // constructor
  SmTopologySolver
  (
    SmSolverOperationType   eSolverOperation,
    SmSolutionRequestedType eSolutionRequested,
    double                  d3dTolerance,
    double                  dBestAnswerSoFarSq,
    const SmVector3d      * cpOptVectors,
    SmSolutionArray       & rSolutions
  ) ;
  
  // destructor
  virtual ~SmTopologySolver() { m_pLocalSolver = NULL ; }
  
  virtual SmStatus LocalSolve
  (
    SmTreeNode * apBranch[SM_GS_MAX_TREES],       // in : pair of treeNodes to compare
    SmBoolean  & rbNeedsMoreSubdivision           // out: TRUE  = Nodes have children that need to be processed
                                                  //    : FALSE = Bounding boxes are disjoint or
                                                  //    :         Nodes have no more children
  );
  
  SmBoolean MayContainAnswer
  (
    const SmExtent3d & crBox1, 
    const SmExtent3d & crBox2
  ) const;
  
  static SmStatus BrepPointSolve
  (
    const SmBrep            * cpBrep,             // in : Brep to query
    const SmPoint3d         & crPoint,            // in : Query point
    SmSolverOperationType     eSolverOperation,   //    : oneof: SM_SO_INTERSECT, SM_SO_NORMALIZE, SM_SO_RAYFIRE,
                                                  //    :        SM_SO_MINIMIZE, SM_SO_PROJECTED_MINIMIZE, SM_SO_DIRECTED_MINIMIZE,
                                                  //    :        SM_SO_MAXIMIZE, SM_SO_PROJECTED_MAXIMIZE, SM_SO_DIRECTED_MAXIMIZE,
                                                  //    :        SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_3D_SIGNED_DIRECTED_MINIMIZE.
    SmSolutionRequestedType   eSolutionRequested, //    : oneof: SM_SR_SINGLE = Only produce one solution - the best
                                                  //    : SM_SR_ALL    = Find all function satisfying solutions to given tol
    double                    d3dTolerance,       // in : passed along to SmTopologySolver, max valid sol size                
    double                    dBestAnswerSoFarSq, // in : passed along to SmTopologySolver,                     
    const SmVector3d        * cpOptVectors,       // in : used for some eSolverOperations - see SmSolverOperationType for more
    SmSolutionArray         & rSolutions          // out: One SmSolution object per problem solution
  );
  
  // get all Curve/Brep intersections
  static SmStatus BrepCurveSolve
  (
    const SmBrep          * cpBrep,               // in : target Brep
    const SmCurve         & crCurve,              // in : target Curve
    const SmExtent1d      & crInterval,           // in : target Curve interval
    SmSolverOperationType   eSolverOperation,     // in : Oneof the listed values above
    SmSolutionRequestedType eSolutionRequested,   // in : Oneof: SM_SR_SINGLE, SM_SR_ALL, SM_SR_NODES
    double                  d3dTolerance,         // in : size distance for solutions
    double                  dBestAnswerSoFarSq,   // in : Cull value for min/max operations
    const SmVector3d      * cpOptVectors,         // in :
    SmSolutionArray       & rSolutions            // out: Solutions list. This list is reset
                                                  //    : before being loaded in this call.
  );
  
  static SmStatus BrepBrepSolve
  (
    const SmBrep          * cpBrep1, 
    const SmBrep          * cpBrep2, 
    SmSolverOperationType   eSolverOperation,
    SmSolutionRequestedType eSolutionRequested,
    double                  d3dTolerance,
    double                  dBestAnswerSoFarSq,
    const SmVector3d      * cpOptVectors,
    SmSolutionArray       & rSolutions
  );
  
  static SmStatus ShapePointSolve
  (
    const SmShape         * cpShape,                  // in : List of topology objects to test for point
    const SmPoint3d       & crPoint,                  // in : Point to be tested
    SmSolverOperationType   eSolverOperation,         // in : Operation to apply to point/shape combination
    SmSolutionRequestedType eSolutionRequested,       // in : oneof: SM_SR_SINGLE
                                                      //    : SM_SR_ALL
    double                  d3dTolerance,             // in : Geometric Tolerance for operation
    double                  dBestAnswerSoFarSq,       // in : limiting value for min/max operations
                                                      //    : Set to Large (or Small) value to ignore.
    const SmVector3d      * cpOptVectors,             // in : Extra values needed by some operations.
    SmSolutionArray       & rSolutions                // out: List of solutions
  );
  
  static SmStatus ShapeCurveSolve
  (
    const SmShape         * cpShape, 
    const SmCurve         & crCurve,
    const SmExtent1d      & crInterval,
    SmSolverOperationType   eSolverOperation,
    SmSolutionRequestedType eSolutionRequested,
    double                  d3dTolerance,
    double                  dBestAnswerSoFarSq,
    const SmVector3d      * cpOptVectors,
    SmSolutionArray       & rSolutions
  );
  
  static SmStatus ShapeShapeSolve
  (
    const SmShape         * cpShape1, 
    const SmShape         * cpShape2, 
    SmSolverOperationType   eSolverOperation,
    SmSolutionRequestedType eSolutionRequested,
    double                  d3dTolerance,
    double                  dBestAnswerSoFarSq,
    const SmVector3d      * cpOptVectors,
    SmSolutionArray       & rSolutions
  );
  
  SmStatus SolveTrees
  (
    SmTree & rTree1,                   // in : 1st target tree
    SmTree & rTree2,                   // in : 2nd target tree
    SmTopoLocalSolver & rLocalSolve    // in : Local Solver to place in m_pLocalSolver
  );
  
  SmStatus SolveTwoTrees
  (
    SmTree & rTree1,
    SmTree & rTree2,
    SmTopoLocalSolver & rLocalSolve
  );
  
  void GetBestAnswerSoFar(double & rdBestAnwser, double * & rpdBestAnswer);
  
  // any node in tree may have data in it
  virtual SmBoolean AreReadyForLocalSolve( SmTreeNode * /*apBranch*/[SM_GS_MAX_TREES] ) const
  {
    return TRUE;
  }
  
  
  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmTopologySolver, SmGlobalSolver, SmTopologySolver_TYPE) ;
  
} ; // end class SmTopologySolver



#endif // !__SMTOPOLOGYSOLVER_H__


