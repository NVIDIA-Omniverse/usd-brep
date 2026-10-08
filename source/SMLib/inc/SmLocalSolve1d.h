// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmLocalSolve1d.h
* PURPOSE: Definition of runtime class SmLocalSolve1d to solve one 
*    dimensional functions by Newton-Ralphson like iterations when
*    we are relatively near an answer.
**********************************************************************/

#ifndef __SMLOCALSOLVE1D_H__
#define __SMLOCALSOLVE1D_H__

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

/*******************************************************************//**
PURPOSE: This enum defines what type of algorithm is being used in
    the local solver.

NOTES: 
***********************************************************************/
enum SmLocalSolverAlgorithmType 
{ 
  SM_SA_NEWTON,
  SM_SA_BISECTION,
  SM_SA_BRENT_MAXIMIZE,
  SM_SA_BRENT_MINIMIZE
} ;


/*******************************************************************//**
PURPOSE: This enum defines reasons that the numerical algorithms
    in the local solvers may have terminated.

NOTES: 
***********************************************************************/
enum SmTerminationReasonType 
{
  SM_TR_UNDEFINED,
  SM_TR_FOUND_ANSWER_CONVERGED,
  SM_TR_FOUND_ANSWER_CLOSE,
  SM_TR_OUT_OF_BOUNDS,
  SM_TR_BAD_JACOBIAN,
  SM_TR_UNABLE_TO_CONVERGE,
  SM_TR_STILL_ITERATING
};

/*******************************************************************//**
PURPOSE: This is the pure virtual object which supports evaluation
   Simply subclass this object.  Add your fields and implement
   the virtual Evaluate Function.

NOTES: 
***********************************************************************/
class SM_EXPORT SmEvalFunctionObject 
{
public:
  // constructor
  SmEvalFunctionObject()          { }

  // destructor
  virtual ~SmEvalFunctionObject() { }

  // Note that we should look for an answer in the evaluate
  // because it probably has a better Idea of the geometry of
  // the situation and the tolerance will be more meaningfull
  virtual SmStatus Evaluate(double      dT,                   // in : target T value to query
                            double    & rdFOfT,               // out: F(T)     = Function value for given dT value
                            double    & rdFPrimeOfT,          // out: dF(T)/dT = Function derivative value for given dT value
                            SmBoolean & rbFoundAnswer,        // out: TRUE = Function value is within tolerance of zero, FALSE=Not
                            SmBoolean   bSignalErrors=TRUE) ; // in : TRUE = signal errors, FALSE=errors anticipated, don't signal, default:[TRUE]

} ; // end class SmEvalFunctionObject

/*******************************************************************//**
PURPOSE: 

NOTES:
****************************************************************/
inline SmStatus SmEvalFunctionObject::Evaluate
  (double      ,     // in : target T value to query                                
   double    & ,     // out: F(T)     = Function value for given dT value           
   double    & ,     // out: dF(T)/dT = Function derivative value for given dT value
   SmBoolean & ,     // out: TRUE = Function value is within tolerance of zero, FALSE= Not      
   SmBoolean   )     // in : bSignalErrors: TRUE = signal errors, FALSE=errors anticipated, don't signal, default:[TRUE]                                                
{ SE(SM_ERR); 
  return SM_ERR; 
}

/*******************************************************************//**
PURPOSE: This class is used internally during solving to store
    iteration value.

NOTES:
 static class object - don't add virtual methods to SmIterationValue.  
 Virtual methods conflict with the methods in SmTArray<SmIterationValue>
 that use memset() to clear memory - with SmIterationValue virtual methods the
 virtual pointer tables get corrupted by SmTArray<SmIterationValue>::ReSet() 
 calls and the like.
***********************************************************************/
class SmIterationValue
{  
public:
  double m_dT         = SM_UNDEF_DOUBLE ;  // parameter value, t     
  double m_dFOfT      = SM_UNDEF_DOUBLE ;  // Function value,  f(t)  
  double m_dFPrimeOfT = SM_UNDEF_DOUBLE ;  // 1st Derivative,  f'(t).     
  double m_dPrevStep  = SM_UNDEF_DOUBLE ;  // the step used to get to this m_dT previous step.

  // To satisfy what is needed to be in SmTArray:
  inline int operator==( const SmIterationValue & crIV2 )
  {
    SM_REF1(crIV2) ; 
    return FALSE;
  } // end operator==
};

/*******************************************************************//**
 'Queue' of SmIterationValue objects.

 This is like a Queue, in that items are pushed on one end
 and come off the other.  But it's not a true Queue in that
 callers cannot read or pop from the back end, and it mostly
 looks at all values inside it.  (This makes this implementation
 much simpler.)
 It's really just an SmTArray, but each new addition (after it's
 full) replaces the oldest entry (like a Queue), but does so by
 moving the 'head' along in the array, and so avoiding having to
 slide all entries down on each addition.
**********************************************************************/

SM_TARRAY_TEMPLATE_PREDECLARATION(SmIterationValue) ;

class SmIterQueue1d
{
private:
  SmTArray< SmIterationValue > m_vArray;

  ULONG m_lSize;     // Size the queue.  Same as m_vArray.GetSize(); here for convenience.
  ULONG m_lCount;    // count of items in the queue.
  ULONG m_lNextIdx;  // index of the next available space.
  SmBoolean m_bFull; // m_lCount == size of array: no empty spaces.

  inline SmIterationValue & operator[]  (ULONG lIndex)
    {
      // Just modulo m_lSize.
      ULONG lMyIdx = lIndex % m_lSize;
      return m_vArray[ lMyIdx ];
    }

public:

  // Constructor:
  SmIterQueue1d( ULONG lSize )
  {
      m_vArray.SetSize( lSize );
      m_lSize = lSize;
      m_lNextIdx = m_lCount = 0;
      m_bFull = FALSE;
  }

  void Add( const SmIterationValue &sIterVal )
  {
      m_vArray[ m_lNextIdx ] = sIterVal;

      m_lNextIdx++;
      if ( m_lNextIdx >= m_lSize )
      {
          m_lNextIdx = 0;
          m_bFull    = TRUE;
      }
      if ( m_lCount < m_lSize ) { m_lCount++; }
  }

  SmBoolean IsOscillating( ULONG lStep = 2 );

}; // end class SmIterQueue1d


/*******************************************************************//**
PURPOSE: This object provides functionality for solving one dimensional
    equations using Newton iteration with some modifications.

NOTES: 
***********************************************************************/
class SM_EXPORT SmLocalSolve1d
{
protected:
  SmBoolean                    m_bUseInterval = FALSE ; // TRUE  = limit solution to given interval
                                                        // FALSE = don't
  SmExtent1d                   m_vInterval;             // problem interval: assumes function is continuous in this interval
  SmLocalSolverAlgorithmType   m_eLocalSolverType;      // oneof: SM_SA_NEWTON,      [Default]      
                                                        //        SM_SA_BISECTION,     
                                                        //        SM_SA_BRENT_MAXIMIZE,
                                                        //        SM_SA_BRENT_MINIMIZE. 
  SmEvalFunctionObject       & m_rFunctionEvaluator;    // different Evaluate functions solve different problems
                                                        // oneof: SmCurvePropertyEFO     - find zero values for variety of curve properties
                                                        //        SmFindClippedRadiusEFO - find the offset radius of SmConstantDistanceFS 
                                                        //                                 when clipping by surface-domain occurred
                                                        //        SmFindPCExtremaEFO     - Find CurvePoint so that Gap is zero or
                                                        //                                 Gap/CurveTangent are perpendicular.
                                                        //        SmFindSilSingEFO       - Find Silhouette Point on Curve for given viewing angle 
  double                       m_dDesiredAccuracy    = SM_UNDEF_DOUBLE ; // default:[SM_EFF_ZERO]
  double                       m_dAcceptableAccuracy = SM_UNDEF_DOUBLE ; // Value passed into SolveIt()
  ULONG                        m_lMaxIter            = SM_UNDEF_ULONG ;  // Max number of iterations before quitting NR iteration.     
  ULONG                        m_lMaxBoundaryHits    = SM_UNDEF_ULONG ;  // Max number of boundary hits before quitting NR iteration.     
  SmBoolean                    m_bIsPeriodic         = FALSE ;           // TRUE = Curve is closed     

  // following is used when we have a root bracketed
  // and we need to keep iteration within this bracket
public:
  SmBoolean m_bHaveLowerBracket = FALSE ; // in SolveIt - TRUE once a zero crossing lower bound is found     
  double    m_dLowerBracketF    = SM_UNDEF_DOUBLE ; //  Lower Bound Func value    
  double    m_dLowerBracketT    = SM_UNDEF_DOUBLE ; //  Lower Bound Param value
  SmBoolean m_bHaveUpperBracket = FALSE ; // in SolveIt - TRUE once a zero crossing upper bound is found     
  double    m_dUpperBracketF    = SM_UNDEF_DOUBLE ; //  Upper Bound Func value                                         
  double    m_dUpperBracketT    = SM_UNDEF_DOUBLE ; //  Upper Bound Param value                                        

  // the following data is used during processing
  // and can be retireved after output
  SmIterationValue m_v2Prev;     // two iterations ago
  SmIterationValue m_v1Prev;     // one iteration ago
  SmIterationValue m_vCurr;      // current iteration
  ULONG            m_lIter = 0 ; // Current iteration count.

  double m_dFoundAccuracy = SM_UNDEF_DOUBLE ;
  ULONG m_lLowerBoundHits = SM_UNDEF_ULONG ;
  ULONG m_lUpperBoundHits = SM_UNDEF_ULONG ;
  SmTerminationReasonType m_eTerminationReason = SM_TR_UNDEFINED ;

public:

  // Constructors
  SmLocalSolve1d(SmEvalFunctionObject & rFunctionEvaluator);
  SmLocalSolve1d(SmEvalFunctionObject & rFunctionEvaluator, 
                 const SmExtent1d     & crInterval, 
                 SmBoolean              bIsPeriodic = FALSE);

  // destructor
  ~SmLocalSolve1d() { }

  // Post Solve query to find out what happened
  double                  GetFoundAccuracy()     const { return m_dFoundAccuracy; }
  SmTerminationReasonType GetTerminationReason() const { return m_eTerminationReason; } 
  ULONG                   GetLowerBoundHits()    const { return m_lLowerBoundHits; }
  ULONG                   GetUpperBoundHits()    const { return m_lUpperBoundHits; }

  void     SetSolutionInterval(const SmExtent1d & crInterval)              { m_vInterval = crInterval; }
  void     SetSolverAlgorithm (SmLocalSolverAlgorithmType eLocalSolverAlg) { m_eLocalSolverType = eLocalSolverAlg; }
  SmStatus SetDesiredAccuracy (double dDesiredAccuracy)                    { m_dDesiredAccuracy = dDesiredAccuracy; 
                                                                             return SM_SUCCESS; 
                                                                           }
  // Attempt to find solution
  SmStatus SolveIt(double      dGuessT, 
                   double      dAcceptableAccuracy, 
                   SmBoolean & rbFoundSolution, 
                   double    & rdFoundT);
  
  SmStatus SolveByBisection(ULONG    lMaxDivisionLevel,
                            double   dLowerT,    double   dUpperT,
                            double   dLowerFOfT, double   dUpperFOfT,
                            double & rdFoundT,   double & rdFoundFOfT);

  SmStatus SolveByBrent(double     ax, 
                        double     bx, 
                        double     cx,
                        double     tol, 
                        SmBoolean  bMaximize,
                        double   & rdFoundT, 
                        double   & rdFoundFOfT);

  // Internal methods.
private:
  SmStatus SolveOscillating( double      dGuessT, 
                             SmBoolean & rbFoundSolution, 
                             double    & rdFoundT );

} ; // end class SmLocalSolve1d

/*******************************************************************//**
PURPOSE:  SmLocalSolve1d copy constructor

NOTES:
***********************************************************************/
inline SmLocalSolve1d::SmLocalSolve1d
  (SmEvalFunctionObject & rFunctionEvaluator)  // in : Object to Copy
 : m_bUseInterval      (FALSE), 
   m_eLocalSolverType  (SM_SA_NEWTON), 
   m_rFunctionEvaluator(rFunctionEvaluator), 
   m_dDesiredAccuracy  (SM_EFF_ZERO),
   m_lMaxIter          (100), 
   m_bIsPeriodic       (FALSE)
{

} // end SmLocalSolve1d::SmLocalSolve1d constructor

/*******************************************************************//**
PURPOSE:  SmLocalSolve1d constructor

NOTES:
***********************************************************************/
inline SmLocalSolve1d::SmLocalSolve1d
  (SmEvalFunctionObject & rFunctionEvaluator,   // in : contains callBack Evaluate() function
                                                //      oneof: SmCurvePropertyEFO
                                                //             SmFindClippedRadiusEFO
                                                //             SmFindPCExtremaEFO
                                                //             SmFindSilSingEFO
   const SmExtent1d     & crInterval,           // in : problem interval 
   SmBoolean              bIsPeriodic)          // in : TRUE = curve is closed
 : m_bUseInterval      (TRUE), 
   m_vInterval         (crInterval), 
   m_eLocalSolverType  (SM_SA_NEWTON), 
   m_rFunctionEvaluator(rFunctionEvaluator), 
   m_dDesiredAccuracy  (SM_EFF_ZERO), 
   m_lMaxIter          (100), 
   m_bIsPeriodic       (bIsPeriodic)
{

} // end SmLocalSolve1d::SmLocalSolve1d constructor


#endif // !__SMLOCALSOLVE1D_H__


