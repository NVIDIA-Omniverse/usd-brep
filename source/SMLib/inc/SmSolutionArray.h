// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSolutionArray.h
* PURPOSE: Header file for SmSolutionArray.
**********************************************************************/

#ifndef __SMSOLUTIONARRAY_H__
#define __SMSOLUTIONARRAY_H__

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

class SmTreeNode;

#define SM_SA_MAX_TREES 5

/*******************************************************************//**
PURPOSE: This enum defines the type of solution output by various
    solvers.

NOTES: 
***********************************************************************/
enum SmSolutionType 
{   SM_ST_NODES_ONLY,        // Output just contains nodes of tree
    SM_ST_SINGLE_VALUE,      // Output is a single point/parameter
    SM_ST_RANGE_OF_VALUES,   // Output is a range of values between two parameters
    SM_ST_POLY_SINGLE_VALUE, // Output is a PolySolver Point Solution 
    SM_ST_UNKNOWN
} ;

/*******************************************************************//**
PURPOSE: This object contains the specific values for the solution of
    a domain problem.  A single solution end typically represents a single
    point in a solution space.  For example, if it represents a point of
    intersection between a curve and a surface the parameters of the curve
    and surface at which the intersection occurs will be in m_adParameters.
    The relative accuracy of the intersection will be in m_dSolutionValue.
    The SmSolutionEnd object is primarily used as part of a SmSolution
    object.  

NOTES: SmSolutionEnd may not have virtual functions or be derived
       from a class that has virtual functions because 
       SmSolutionArray manages arrays of SmSolution (which contains
       SmSolutionEnd objects) objects with smos_Calloc, smos_MemCpy,
       and smos_MemSet calls.
***********************************************************************/
class SM_EXPORT SmSolutionEnd
{ 
public:
  double m_dSolutionValue;        // The value of the solution being represented.
                                  //    Typically this is some sort of distance.  
  double m_adParameters           // The DomainPoint of the solution being represented 
            [SM_SA_MAX_TREES*2];  //    - typically the concatenation of curve and surface 
                                  //    DomainPoints as follows:
                                  //  Curve/Point problem solutions - [0] = Curve Param
                                  //
                                  //  Surf/Point  problem solutions - [0] = Surface1 U Param
                                  //                                  [1] = Surface1 V Param
                                  //
                                  //  Curve/Curve problem solutions - [0] = Curve1 Param
                                  //                                  [1] = Curve2 Param
                                  //
                                  //  Curve/Surf  problem solutions - [0] = Curve Param
                                  //                                  [1] = Surface1 U Param
                                  //                                  [2] = Surface1 V Param
                                  //
                                  //  Surf/Surf   problem solutions - [0] = Surface1 U Param
                                  //                                  [1] = Surface1 V Param
                                  //                                  [2] = Surface2 U Param
                                  //                                  [3] = Surface2 V Param
                                  //
                                  //  Surf/Surf/Surf prob solutions - [0] = Surface1 U Param
                                  //                                  [1] = Surface1 V Param
                                  //                                  [2] = Surface2 U Param
                                  //                                  [3] = Surface2 V Param
                                  //                                  [4] = Surface3 U Param
                                  //                                  [5] = Surface3 V Param
                                  //   
                                  //  By convention the solution parameter values are assigned to 
                                  //  geometry parameter values in the order that the geometry
                                  //  was specified by the originating method.  The primary exception
                                  //  to this rule is for curve( or edge)/surface(or face) solutions.  
                                  //  In this case the curve(or edge) parameter is always
                                  //  listed first because it is often required to sort points 
                                  //  along the single parameter of a curve or edge.  
  // constructor
  SmSolutionEnd()                 : m_dSolutionValue(0.0)
                                  { 
                                    // okay to use smos_MemSet on base (double) objects.
                                    smos_MemSet(&m_adParameters, 0, sizeof(double)*SM_SA_MAX_TREES*2) ; 
                                  }

  // assignment operator
  SmSolutionEnd & operator=(const SmSolutionEnd &crSolutionEnd ) ;

  // equality operator - checks all m_adParameter values - even the unused ones - set those to be 0.0
  SmBoolean operator==(const SmSolutionEnd &crOther) const;

  // destructor
  ~SmSolutionEnd()                { ReSet() ; }

  void ReSet()                    { m_dSolutionValue = 0.0 ;
                                    // okay to use smos_MemSet on base (double) objects.
                                    smos_MemSet(&m_adParameters, 0, sizeof(double)*SM_SA_MAX_TREES*2) ;
                                  }
  // simple m_adParameters data access
  double   operator[] (ULONG lIndex) const { SM_ASSERT_BREAK(lIndex < SM_SA_MAX_TREES*2);
                                             return m_adParameters[lIndex];
                                           }
  double & operator[] (ULONG lIndex)       { SM_ASSERT_BREAK(lIndex < SM_SA_MAX_TREES*2);
                                             return m_adParameters[lIndex];
                                           }

  const TCHAR * GetTypeString() const { return _T("SmSolutionEnd") ; }

} ; // end class SmSolutionEnd

/*******************************************************************//**
PURPOSE: The solution contains data which describes the results of
   an operation - typically produced by a solver of some sort.  The solution
   is general enough to contain answers of operations between points, 
   curves, surfaces, vertices, edges, faces, and breps.  Global solver
   operations typically produce an array of solutions (SmSolutionArray).
   Local solver operations typically iterate to a single solution
   (SmSolution).   

NOTES: 

Example:
   The minimization of the distance between a curve and a surface 
   would have the following values in the m_vStart:
     SER(rSurface->GlobalCurveSolve(...,rCurve,...,sSolutions));
     SmSolution & rSol = sSolutions[0];
     double dDistanceBetweenCurveAndSurface = rSol.m_vStart.m_vSolutionValue;
     double dCurveParameter    = rSol.m_vStart[0];
     double dSurfaceUParameter = rSol.m_vStart[1];
     double dSurfaceVParameter = rSol.m_vStart[2];
     SmPoint2d sUVOfSurface(dSurfaceUParameter,dSurfaceVParameter);
     SmPoint3d sPointOnSurface;
     SER(rSurface->EvaluatePoint(sUVOfSurface,sPointOnSurface));
     SmPoint3d sPointOnCurve;
     SER(rCurve->EvaluatePoint(dCurveParameter,sPointOnCurve));
     sPointOnSurface.DistanceBetween(sPointOnCurve) is equal to 
     dDistanceBetweenCurveAndSurface.

 This class cannot have virtual methods since SmSolutionArray
 manages arrays of SmSolutions using smos_MemSet().
***********************************************************************/
class SM_EXPORT SmSolution 
{
public:
    SmSolutionType   m_eSolutionType;  // What type of solution is this, single or range of values
                                       // a range of values is where the solution has the same value
                                       // over a specific range.  
                                       // SM_ST_SINGLE_VALUE: single valued solution, only m_vStart 
                                       //                      contains valid values.
                                       //
                                       // SM_ST_RANGE_OF_VALUES: solutions exist over a range of values
                                       //                      from m_vStart to m_vEnd. An example of 
                                       //                      this is a coincidence between two curves.
                                       //
                                       // SM_ST_POLY_SINGLE_VALUE: single valued solution output by the
                                       //                          PolySolver.  Use of m_vStart[0,1,2,3,4,5]
                                       //                          is to store positions not parameters.
                                       //
    ULONG            m_lNumVariables;  // How many total variables in this solution.  This defines how
                                       // many total parameters exist in m_vStart and/or m_vEnd.  For
                                       // example, if we are itersecting a curve and a surface the number
                                       // of variables is 3 - m_vStart[0] - curve parameter of the intersection
                                       // m_vStart[1] - U parameter of the surface, m_vStart[2] - V parameter
                                       // of the surface.  In most cases you will know the number of variables
                                       // based on the types of input given.
                                       // Exception: SmLSI3LocalSolver::LocalSolve() returns 9 variables
                                       //            the first 3 are the standard curve and surface parameters, 
                                       //            the next 6 are solution surface pnt and normal values 
                                       //            to save having to recompute them.

    SmSolutionEnd    m_vStart;         // Defines solution value parameters of single value or start
                                       // parameters of a range of values.  The variables are typically
                                       // ordered the same as given in the input method except in the
                                       // case of a one manifold (edge or curve) and a two manifold (face or
                                       // surface).  In this case the curve parameters go first.  
                                       // 
                                       // 
    SmSolutionEnd    m_vEnd;           // End value of the range of values.  A range of values will occur typically
                                       // if there is a coincidence or contiguous set of points where a valid
                                       // solution exists.  An example is the coincidence between two curves along
                                       // a portion of each curve. In this case, m_vStart will contain the 
                                       // lower parameter of the first object (i.e. curve/edge) and m_vEnd will
                                       // contain the higher parameter value on the first object.  
                                       // 
    ULONG            m_lNumObjects;    // How many object in this solution.  This is typically one or
                                       // two and may be three.  The corresponding objects are in the
                                       // m_apObjects array below.  The object are given in the order in which
                                       // they are passed into the original method except in the case where a
                                       // one dimensional and two dimensional object occur.  This order corresponds
                                       // the the order the parameters occur in m_vStart and m_vEnd.
                                       // 
    SmObject   * m_apObjects[SM_SA_MAX_TREES]; // Objects used in generation of this solution.  These
                                       // objects are typically curves or surfaces or objects which own them
                                       // (i.e. edges or faces).
    SmTreeNode * m_apNodes[SM_SA_MAX_TREES];   // Tree nodes as used in curve, surface, trimmed surface, and
                                       // brep caches where solution exists.  Most applications will never need
                                       // to use these values so they can be ignored for the most part. 
                                       // NULL = solution generated by without using the spatial decomposition trees.
                                       // note: there is no m_lNumNodes stored value because that will be a constant
                                       //       for each solver.

    // constructor
    SmSolution() : m_eSolutionType( SM_ST_UNKNOWN ),
      m_lNumVariables( 0 ),
      m_lNumObjects( 0 )
    { // okay to use smos_MemSet on base (pointer) objects.
      smos_MemSet( &m_apObjects, 0, sizeof( SmObject * )  *SM_SA_MAX_TREES );
      smos_MemSet( &m_apNodes, 0, sizeof( SmTreeNode * )*SM_SA_MAX_TREES );
    }

    // copy constructor
    SmSolution(const SmSolution & crSrc) ;

    // assignment operator
    SmSolution & operator=(const SmSolution &crSolution ) ;

    // equality operator
    SmBoolean operator==(const SmSolution &crOther) const;

    // destructor
    ~SmSolution()                    { ReSet() ; }

    void ReSet()
    {
      m_eSolutionType = SM_ST_UNKNOWN;
      m_lNumVariables = 0;
      m_lNumObjects = 0;
      // okay to use smos_MemSet on base (pointer) objects.
      smos_MemSet( &m_apObjects, 0, sizeof( SmObject * )  *SM_SA_MAX_TREES );
      smos_MemSet( &m_apNodes, 0, sizeof( SmTreeNode * )*SM_SA_MAX_TREES );
    }

    //
    long ClassifySolutionEnd
    (
      const SmSolution & crOtherSolution,          ///< [in ]: Solution to compare to thisSolution                 <br>
      SmBoolean          bCompareStart             ///< [in ]: TRUE = compare OtherSolution.Start to thisSolution  <br>
                                                   ///<      : FALSE= compare OtherSolution.End   to thisSolution  <br>
    ) const;

    // get solution object's object and param indices 
    SmStatus GetIndex
    (
      const SmObject * pObject,                    ///< [in ]: target object to query                               <br>
      SmBoolean        bObjectFlag,                ///< [in ]: TRUE = get index for given pObject                   <br>
                                                   ///<      : FALSE= get index for 1st object that is not pObject  <br>
      ULONG          & lObjectIndex,               ///< [out]: Index of requested object                            <br>
      ULONG          & lParamIndex                 ///< [out]: Index of 1st solution parameter for requested object <br>
    ) const ;

    // get solution point for the ith solution object
    SmStatus GetPoint
    (
      ULONG       lIndex,                          ///< [in ]: object index to query                          <br>
      SmPoint3d & rSolPoint,                       ///< [out]: Solution Point for the lIndex object           <br>
      SmPoint3d * pOptSolParam = NULL,             ///< [out]: Soloution Parameter for the lIndex object,     <br>
                                                   ///<      : when SolObjects is                             <br>
                                                   ///<      :   SmVertex or SmPoint  : set to: [0 0 0]       <br>
                                                   ///<      :   SmEdge   or SmCurve  :         [u 0 0]       <br>
                                                   ///<      :   SmFace   or SmSurface:         [u v 0]       <br>
                                                   ///<      :   SmVolume             :         [u v w]       <br>
                                                   ///<      : NULL to ignore                                 <br>
                                                   ///<      : default:[NULL]                                 <br>
      SmBoolean   bGetEnd = FALSE                  ///< [in ]: if True, get the End of a Range solution.      <br>
                                                   ///<      : If not a range solution, just return the Start.<br>
    ) const;

    SmStatus GetPoint
    (
      const SmObject * pObject,                    ///< [in ]: target object to query                             <br>
      SmBoolean        bObjectFlag,                ///< [in ]: TRUE = get point on given pObject                  <br>
                                                   ///<      : FALSE= get point on 1st object that is not pObject <br>
      SmPoint3d      & rSolPoint,                  ///< [out]: Solution Point for the lIndex object               <br>
      SmPoint3d      * pOptSolParam = NULL,        ///< [out]: Solution Parameter for the lIndex object,          <br>
                                                   ///<      : when SolObjects is                                 <br>
                                                   ///<      :   SmVertex or SmPoint  : set to: [0 0 0]           <br>
                                                   ///<      :   SmEdge   or SmCurve  :         [u 0 0]           <br>
                                                   ///<      :   SmFace   or SmSurface:         [u v 0]           <br>
                                                   ///<      :   SmVolume             :         [u v w]           <br>
                                                   ///<      : NULL to ignore                                     <br>
                                                   ///<      : default:[NULL]                                     <br>
      SmBoolean        bGetEnd = FALSE             ///< [in ]: if True, get the End of a Range solution.          <br>
                                                   ///<      : If not a range solution, just return the Start.    <br>
                                                   ///<      : default:[FALSE]                                    <br>
    ) const;

    SmStatus GetObjectIndexForParameterIndex
    (
      ULONG   lParameterIndex,                     ///< [in ]: Target Parameter Index                             <br>
      ULONG & rlObjectIndex,                       ///< [out]: Index of Object associated with ParameterIndex     <br>
      ULONG & rlDomainDir                          ///< [out]: Domain index associated with ParameterIndex        <br>
                                                   ///<      : 0 = s or u for curves, surfaces, and volumes       <br>
                                                   ///<      : 1 = v for surfaces and volumes                     <br>
                                                   ///<      : 2 = w for volumes                                  <br>
    ) const ; 

    SmStatus GetParameterInterval
    (
      ULONG        lParameterIndex,                ///< [in ]: Target Parameter Index                               <br>
      SmExtent1d & rIvl                            ///< [out]: interval associated with lParameterIndex             <br>
    ) const ; 

    double GetObjectTolerance(ULONG lObjectIndex) const ;

    // predicates
    SmBoolean IsEqual(const SmSolution &crOtherSolution) const ;

    // Pretty Display/Print SmSolution contents
    SmDisplayList * Draw(ULONG l1stObject1stParamIndex=0) const ;

    void  Dump() const ;

} ; // end class SmSolution

/*******************************************************************//**
PURPOSE: The solution array is a dynamic container class which contains
    SmSolution objects.  It is the primary class used to contain 
    results of globals solver operations.  Most global solver operations can 
    produce many possible answers depending upon the configuration of the 
    underlying geometry.  

NOTES:  This is basically an instance of the SmTArray template
    and could be one again.  Left this way because its not worth
    changing.
***********************************************************************/
class SM_EXPORT SmSolutionArray : public SmObject
{
  // Implementation
protected:
  ULONG        m_lSize;          // # of elements (upperBound - 1) 
  ULONG        m_lMaxSize;       // max allocated
  SmSolution * m_pData;          // pointer to an array of SmSolution objects 
  SmBoolean    m_bIsBorrowed;    // TRUE = data array borrowed, FALSE = data is internal  
                                 //   destructor deletes m_pData when m_bIsBorrowed==FALSE

  void SetAtGrow(ULONG nIndex, const SmSolution & crNewSolution);

public:

  // empty constructor
  SmSolutionArray()
  {
    m_cpContext = NULL;
    m_lSize = 0;
    m_lMaxSize = 0;
    m_pData = NULL;
    m_bIsBorrowed = FALSE;
  }

  // Constructor for borrowed arrays
  SmSolutionArray
  (
    ULONG        nDataSize,              ///< [in ]: Size of OptBorrowedData      <br>
    SmSolution * pOptBorrowedData=NULL   ///< [in ]: the borrowed array           <br>
  );

  // copy constructor
  SmSolutionArray(const SmSolutionArray & crSrc) ;

  // assignment operator: deep copy
  SmSolutionArray & operator=(const SmSolutionArray & crSrc) ;

  // equality operator
  SmBoolean operator==(const SmSolutionArray &crOther) const;

  // destructor
  virtual ~SmSolutionArray();

  // Attributes
  ULONG GetSize() const                   { return m_lSize; }    // number of SmSolutions currently held
  ULONG GetDataSize() const               { return m_lMaxSize; } // number of slots allocated that can hold SmSolutions
  void  SetSize(ULONG nNewSize);          // alloc memory and set both m_lMaxSize and m_lSize
  void  SetDataSize(ULONG nNewDataSize);  // alloc memory and set m_lMaxSize
  void  SetSizeValue(ULONG nNewSize) ;    // just set m_lSize (ensure m_lMaxSize >= m_lSize)

  void  ReSet()                           // set m_lSize = 0, but do not free memory
  {
    if(m_pData && m_lSize > 0)
    {
      smos_MemSet( m_pData, 0, m_lSize * sizeof( SmSolution ) );
    }
    m_lSize = 0;
  }

  void  RemoveAll()                       { SetSize(0); }       // set m_lSize = 0, free all memory

  // Accessing elements
  SmSolution   GetAt(ULONG nIndex) const;
  void         SetAt(ULONG nIndex, const SmSolution & crNewSolution);    
  SmSolution * GetDataArray() const       { return m_pData; }
  SmSolution & operator[] (ULONG lIndex)  { SM_ASSERT_BREAK(lIndex < m_lSize); return m_pData[lIndex]; }

  // Potentially growing the array

  // copy crNewSolution data into array object
  ULONG Add(const SmSolution & crNewSolution);      
  ULONG Append(const SmSolutionArray& crSource);
  ULONG AddSortedSolution(const SmSolution & crNewSolution, SmSortKeyType eSortKey);
  void  Copy(const SmSolutionArray& crSource);
  void  InsertAt(ULONG nIndex, const SmSolution & crNewSolution, ULONG nCount=1);
  void  InsertAt(ULONG nStartIndex, const SmSolutionArray & crArrayToInsert);
  void  RemoveAt(ULONG nIndex, ULONG nCount=1);
  void  RemoveDuplicateSolutions();
  void  Swap(ULONG nIndex1, ULONG nIndex2);

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmSolutionArray,SmObject,SmSolutionArray_TYPE);

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                          <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                      <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                  <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                            <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< NotUsed: [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]<br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                <br>
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  static SmBoolean SelfTest() ;

  SmDisplayList * Draw() const ;
    
} ; // end class SmSolutionArray


#endif // __SMSOLUTIONARRAY_H__

