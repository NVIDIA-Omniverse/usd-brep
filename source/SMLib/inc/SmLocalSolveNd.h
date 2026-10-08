// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmLocalSolveNd.h
* PURPOSE: Definition of runtime class SmLocalSolveNd to solve N 
*    dimensional functions by Newton-Ralphson like iterations when
*    we are relatively near an answer.
*
* Contains
*   class SmEvalNFunctionsObject - pure virtual equation set representation used by 
*   class SmLocalSolveNd         - generic equation set, multi-variable, Newton-Raphson Solver
**********************************************************************/

#ifndef __SMLOCALSOLVEND_H__
#define __SMLOCALSOLVEND_H__

#ifndef __SMEXTENTNd_H__
#include <SmExtentNd.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMMATRIX_H__
#include <SmMatrix.h>
#endif


/*******************************************************************//**
PURPOSE: This enum is used to specify how to handle cases where
   one or more of the solution parameters go out of the specified range
   during the process of solving the equation.

NOTES: 
***********************************************************************/
enum SmBoundaryHandlerType {
  SM_BH_TOTAL_BOUNDARY_HITS,   // exit Newton-Raphson if sequence of 
                               //   next-guesses cause any combination 
                               //   of parameters to exceed their bounds 
                               //   more than a max-count of times. 
  SM_BH_HITS_ALL_BOUNDARIES,   // exit Newton-Raphson if next guess is 
                               //   out of all parameter bounds at once
  SM_BH_IGNORE_BOUNDARY_HITS   // allow Newton-Raphson to run without counting
                               //   the number of times its next-guess value
                               //   walks outside of the given parameter bounds.
};

#include <SmLocalSolve1d.h>

/*******************************************************************//**
PURPOSE: This is a pure virtual object used to apply a generic
   multi-variable Newton-Raphson solver implemented in
   SmLocalSolveNd::SolveIt() to a wide
   variety of equation sets.
   
   Derived classes from this class represent the specific 
   equation sets to be solved.

   SolveIt() communicates with derived objects of this class
   through the callback function, Evaluate(), which must 
   compute and return the set of equation values and the 
   jacobi matrix (1st derivative matrix) given a set of 
   problem parameter values.

NOTES: 
   To apply Newton_Raphson to solve a set of equations, derive a
   a class from SmEvalNFunctionsObject, add any required members,
   and implement the Evaluate function.  Then in the calling subroutine

   1. Construct a Derived SmEvalNFunctionsObjects class representing
      the equations to be solved.
   2. Construct a SmLocalSolveNd object using the constructed equation set.
   3. Control the Newton-Raphson solve behavior by setting the
       SmLocalSolveNd object's behavior bits.
        (see class SmLocalSolveNd)
   4. Initialize an array of Guess parameter values.
       Newton-Raphson only works well when given good nearby guesses.
   5. Call SmLocalSolveNd::SolveIt() to run the Newton-Raphson algorithm
      over the desired equation set starting at the given guess.
      
EXAMPLE: Surface/Surface/Surface intersections

    // Construct a surf/surf/surf equation set evaluator (SmSSSIntersectENFO works in 6 DOFs)
    SmSSSIntersectENFO sEvalFun(*this,crOtherSurface,crOtherSurface2,SM_EFF_ZERO*SM_EFF_ZERO);
    
    // Construct the Newton-Raphson Solver class with the surf/surf/surf equation set
    SmLocalSolveNd sLS(sEvalFun,&sIntervals,&sPeriodicities);  // note: sIntervals and sPeriodicities must be NULL or presized:[6]

    // set the solver's exit behavior for solutions that run outside the boundary
    sLS.SetBoundaryHandler(SM_BH_TOTAL_BOUNDARY_HITS,8);

    // Make the Newton-Raphson solve call
    SER(sLS.SolveIt(sGuessT,SM_EFF_ZERO_SQRT,bFoundSolution,sSolutionVector)); // note: sGuessT and sSolutionVector must be presized:[6]
 
***********************************************************************/
class SM_EXPORT SmEvalNFunctionsObject 
{
public:
  // note: the number of equations being solved is not stored.  It is
  //       passed to the solver indirectly as the size of the GuessVector array
  //       passed to the SmLocalSizeNd::SolveIt() method which calls the
  //       Evaluate() method of this class with same size crX and pOptJacobian arrays.

  double m_dSolutionSize ;  // Quality value to be computed in the Evaluate() functions.
                            //   default:[0.0] 
  double m_dScaledZero ;    // Tolerance value to test for convergence.
                            //   default:[1.0]

                            // The quality of a solution may or may not be 
                            // measured by the functions being zeroed.  
                            // Storing a SolutionSize and a ScaledZero
                            // value allows each derived class to define its 
                            // own quality measure which can be checked in 
                            // SolveIt for convergence and by the 
                            // calling functions to test the quality
                            // of the solution using a common quality value.
                            // The default values for m_dSolutionSize and m_dScaledZero
                            // are picked so that funcs that don't set these values
                            // will continue to converge in the SolveIt() routine. 

  SmEvalNFunctionsObject() : m_dSolutionSize(0.0),
                             m_dScaledZero  (1.0)             { }
  virtual ~SmEvalNFunctionsObject()                           { }
  double    GetSolutionSize() const                           { return m_dSolutionSize ; }
  double    GetScaledZero()   const                           { return m_dScaledZero ;   }
  SmBoolean IsConverged()     const                           { return m_dSolutionSize < m_dScaledZero ; }
  virtual SmBoolean HasIntervals()                            { return FALSE ; }
  virtual SmStatus  SetIntervals(const SmExtentNd          *, // in : cpIntervals,
                                 const SmTArray<SmBoolean> *) // in : cpPeriodicities )  
                                                              { SER(SM_ERR) ; 
                                                                return SM_ERR ;
                                                              }
                                  
  // a callback function used by SmLocalSolveNd::SolveIt()
  // to evaluate the set of equations being zeroed by its
  // Newton-Raphson algorithm and to update the m_dSolutionSize 
  // value.  The SolveIt() method calls this function repeatedly,
  // each time it updates its current best-guess parameter values,
  // until it finds a solution (all equation values are equal to zero)
  // or it gives up (fails to converge.) This method has to be able
  // to compute the pOptJacobian matrix.
  virtual SmStatus Evaluate(const SmTArray<double> & crX,             // in : x of Ax=F,                 Sized:[N] N = GuessVector size passed to SmLocalSizeNd::SolveIt()]     
                            SmTArray<double>       & rF,              // out: F of Ax=F function values, Sized:[N] N = GuessVector size passed to SmLocalSizeNd::SolveIt()]                    
                            SmMatrix               * pOptJacobian,    // out: Partial derivatives of the functions, ordered:
                                                                      //      [dF0/dx0 dF0/dx1 ... dF0/dxn, dF1/dx0 ... dF1/dxn, ...]
                                                                      //      NULL   =skip Jacobian computations - not needed this call
                                                                      //      NotNULL=set  Jacobian values       - needed this call 
                            SmBoolean              & rbFoundAnswer)   // out: Not always used, when used
                                                                      //      TRUE = converged (F members are within tolerance of 0.0
                                                                      //      FALSE= Not Used or Not Converged
                                                                      { // Base Class NR Solver Evaluate() function just returns an error.
                                                                        SM_REF4(crX, rF, pOptJacobian, rbFoundAnswer) ;
                                                                        SE(SM_ERR); return SM_ERR; 
                                                                      }

  // The following method will compute the Jacobian.
  // Base method finds finite diff Jacobian approx by tweaking each parameter and looking for the effects.
  //   1. use: In the derived Evaluate() method, optionally call this method
  //           to set the Evaluate's required output, pOptJacobian values.
  //           This method is not called by the General NewtonRaphson loop.
  //   2. Implement: Implement this derived method in any derived class if the 
  //                 class has explicit eqns for the jacobian to be called by
  //                 the derived Evaluate() method.
  //   Or just implement the jacobian computation directly into Evaluate() and
  //   don't bother implementing this method.
  SmStatus ComputeJacobian(SmTArray<double>       & crX,                    // in : current parameter values, SizeOnInput:[N] N = GuessVector size passed to SmLocalSizeNd::SolveIt()]
                           const SmTArray<double> & rF,                     // in : current equation values,  SizeOnInput:[N] N = GuessVector size passed to SmLocalSizeNd::SolveIt()]
                           SmMatrix               & rJacobian,              // out: jacobian finite differences, SizedOnInput:[N][N]
                           SmTArray<SmExtent1d>   * pOptIntervals = NULL ); // in : optional domains (intervals), sized:[N]

} ; // end class SmEvalNFunctionsObject 

/*******************************************************************//**
PURPOSE: This SmLocalSolveNd object provides a multi-variable 
  Newton-Raphson algorithm to solve a set of equations.  
  
  A problem has a set of problem parameter values and a set of 
  linear or non-linear equations.  The problem is solved when 
  a set of parameter values is found that makes all the equation 
  values exactly equal to zero.

  This solver can be used for any set of equations by deriving a 
  class from the SmEvalNFunctionsObject and implementing an Evaluate()
  callback function which computes the equation values given a set of
  parameter values.

NOTES:
   To apply Newton_Raphson to solve a set of equations, derive a
   a class from SmEvalNFunctionsObject, add any required members,
   and implement the Evaluate function.  Then in the calling subroutine

   1. Construct a Derived SmEvalNFunctionsObjects class representing
      the equations to be solved.
   2. Construct a SmLocalSolveNd object using the constructed equation set.
   3. Control the Newton-Raphson solve behavior by setting the
       SmLocalSolveNd object's behavior bits.
        (see class SmLocalSolveNd)
   4. Initialize an array of Guess parameter values.
       Newton-Raphson only works well when given good nearby guesses.
   5. Call SmLocalSolveNd::SolveIt() to run the Newton-Raphson algorithm
      over the desired equation set starting at the given guess.
      
EXAMPLE: Surface/Surface/Surface intersections

    // Construct a surf/surf/surf equation set evaluator (SmSSSIntersectENFO works in 6 DOFs)
    SmSSSIntersectENFO sEvalFun(*this,crOtherSurface,crOtherSurface2,SM_EFF_ZERO*SM_EFF_ZERO);
    
    // Construct the Newton-Raphson Solver class with the surf/surf/surf equation set
    SmLocalSolveNd sLS(sEvalFun,&sIntervals,&sPeriodicities); // note: sIntervals and sPeriodicities must be NULL or presized:[6]

    // set the solver's exit behavior for solutions that run outside the boundary
    sLS.SetBoundaryHandler(SM_BH_TOTAL_BOUNDARY_HITS,8); 

    // Make the Newton-Raphson solve call
    SER(sLS.SolveIt(sGuessT,SM_EFF_ZERO_SQRT,bFoundSolution,sSolutionVector)); // note: sGuessT and sSolutionVector must be presized:[6]

METHOD  --- Find param values p0, p1, .. pn to make a set of
            equations, Fn, go to zero as
    
            FA(p0,..pn) = 0 
            . . .
            FN(p0,..pn) = 0

            The basic Newton-Raphson algorithm is modified to prevent chasing
            bad steps. Flags within the SmLocalsSolveND object
            control which actions are taken.  These preventative measures include:

            Compute next param values
            limit next guesses to stay within interval limits
            test next guess, check sum of all fabs(function values).
              when next guess is good - return a success
              when next guess is very small and Sum of Fn is large - return a failure
              when next guess is worse than last guess 
                 - shorten step and try again for next step
                 - when step gets very short (< .001) negate step direction and keep trying.
                 - when negative step gets very short without finding a good guess - return a failure
              when next guess is better than last guess - carry on with iteration
              when good next guess can't be found - return a failure


BASIC Newton-Raphson 
           Taylor Series to first order for the Fn equations are

           0 = FA(p0+d0,..pn+dn) = FA(p0,..) + FA0(p0,..)*d0 + FA1(p0,..)*d1 + ...
           . . .
           0 = FN(p0+d0,..pn+dn) = FN(p0,..) + FN0(p0,..)*d0 + FN1(p0,..)*d1 + ...

               where FA0 = d(FA)/dp0, 
                     FA1 = d(FA)/dp1,
                     ...
                     FN0 = d(FN)/dp0
                     ...
                     FNn = d(FN)/dpn
        
           To find parameters to make all Fn zero, solve above eqns for all di.
           Rewriting the above equations in matrix form yields,

              [FA0 FA1 ... FAn]   [d0 ]    [-FA]
              [     ...       ]   [...]  = [...]
              [FN0 FN1 ... FNn]   [dn ]    [-FB]

              'jacobi matrix' * 'deltas' = 'FunRHS'
 
           update parameter values with the new di values as
           p0 += d0
           ...
           pn += dn
 
           iterate until all the FA,..,FN equations are close to zero.        
 
***********************************************************************/
class SM_EXPORT SmLocalSolveNd
{
protected:
  // note1: the size of the problem is not stored and only set indirectly by the
  // size of the GuessVector passed to the SmLocalSolveNd::SolveIt() method call.

  SmLocalSolverAlgorithmType  m_eLocalSolverType;      // oneof SM_SA_NEWTON,       
                                                       //       SM_SA_BISECTION,    
                                                       //       SM_SA_BRENT_MAXIMIZE
                                                       //       SM_SA_BRENT_MINIMIZE

  SmEvalNFunctionsObject    & m_rFunctionEvaluator;    // provides the evaluate function
  double                      m_dDesiredAccuracy;      // very tight tol used for stopping
  double                      m_dAcceptableAccuracy;   // looser tol used to accept answers that don't completely converge
  ULONG                       m_lMaximumIterations;    // stopping criteria for failures
  const SmExtentNd          * m_cpIntervals;           // NULL or sized:[DOF COUNT of contained SmEvalNFunctionsObject]
                                                       //  bounds on problem parameters.
                                                       //  Set m_cpIntervals[i].SetUnbounded() for an unbounded interval whose prob params don't get clamped
  const SmTArray<SmBoolean> * m_cpPeriodicities;       // NULL or sized:[DOF COUNT of contained SmEvalNFunctionsObject]
                                                       //  NULL = No SmEvalNFunctionsObject DOFs are constrained
                                                       //  NotNULL: elem[i] == TRUE = DOFi is periodic in m_cpIntervals[i]
                                                       //           elem[i] == FALSE= DOFi is clamped to m_cpIntervals[i]
                                                       //           Set m_cpIntervals[i].SetUnbounded() for an unbounded interval
  SmBoundaryHandlerType       m_eBoundaryHandler;      // oneof SM_BH_TOTAL_BOUNDARY_HITS,
                                                       //       SM_BH_HITS_ALL_BOUNDARIES,
                                                       //       SM_BH_IGNORE_BOUNDARY_HITS
  ULONG                       m_lMaxBoundaryHits;      // stopping criteria
  SmBoolean                   m_bStepCuttingEnabled;   // TRUE=cut and/or negate size step when failing

public:
  // the following data is used during processing
  // and can be retireved after output
  ULONG                       m_lCurrentIteration;     // Current iteration count.
  SmTArray<double>          * m_pSolutionVector;       // ptr to user supplied memory to store solution
  double                      m_dFoundAccuracy = 0.0;  // 
  SmTerminationReasonType     m_eTerminationReason;    // oneof SM_TR_FOUND_ANSWER_CONVERGED,
                                                       //       SM_TR_FOUND_ANSWER_CLOSE,    
                                                       //       SM_TR_OUT_OF_BOUNDS,         
                                                       //       SM_TR_BAD_JACOBIAN,          
                                                       //       SM_TR_UNABLE_TO_CONVERGE     
public:
  // Initilization 
  SmLocalSolveNd(SmEvalNFunctionsObject    & rEvalFun,                  // in : Define Eqns to set to Zero (defines the DOF COUNT)
                 const SmExtentNd          * cpIntervals = NULL,        // in : NULL or sized:[N], N = this rEvalFun DOF count
                                                                        //      bounds on problem parameters.
                                                                        //      Set m_cpIntervals[i].SetUnbounded() for ivl whose prob params don't get clamped
                 const SmTArray<SmBoolean> * cpPeriodicities = NULL ) ; // in : NULL or sized:[N], N = this rEvalFun DOF count
                                                                        //      ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                                                        //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped

  ~SmLocalSolveNd() { }

  // Access
  const SmExtentNd *      GetIntervals        ()              { return m_cpIntervals; }
                                                              
  // Post Solve query to find out what happened               
  double                  GetFoundAccuracy    () const        { return m_dFoundAccuracy; }
  SmTerminationReasonType GetTerminationReason() const        { return m_eTerminationReason; } 

  void SetStepCuttingEnabled(SmBoolean bStepCuttingEnabled)   { m_bStepCuttingEnabled = bStepCuttingEnabled; }
  void SetDesiredAccuracy   (double dDesiredAccuracy)         { m_dDesiredAccuracy = dDesiredAccuracy; }
  void SetAcceptableAccuracy(double dAcceptableAccuracy)      { m_dAcceptableAccuracy = dAcceptableAccuracy; }
  void SetMaximumIterations (ULONG lMaximumIterations)        { m_lMaximumIterations = lMaximumIterations; }
  void SetBoundaryHandler   (SmBoundaryHandlerType eBHandler, 
                             ULONG lMaxBoundaryHits = 3)      { m_eBoundaryHandler=eBHandler; 
                                                                m_lMaxBoundaryHits=lMaxBoundaryHits;
                                                              }
  // Attempt to find solution
  SmStatus SolveIt(const SmTArray<double> & dGuessVector,         // in : Sized:[N], N sets this problem size
                   double                   dAcceptableAccuracy,  // in :
                   SmBoolean              & rbFoundSolution,      // out:
                   SmTArray<double>       & rSolutionVector);     // out: Sized:[N], N sets this problem size

  SmStatus ComputeJacobian(SmTArray<double>       & crX, 
                           const SmTArray<double> & rF, 
                           SmMatrix               & rJacobian) const;

} ; // end class SmLocalSolveNd 

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmLocalSolveNd::SmLocalSolveNd
  (SmEvalNFunctionsObject    & rFunctionEvaluator, 
   const SmExtentNd          * cpIntervals, 
   const SmTArray<SmBoolean> * cpPeriodicities)
 : m_eLocalSolverType   (SM_SA_NEWTON), 
   m_rFunctionEvaluator (rFunctionEvaluator),
   m_dDesiredAccuracy   (SM_EFF_ZERO/10.0), 
   m_dAcceptableAccuracy(SM_EFF_ZERO),
   m_lMaximumIterations (100), 
   m_cpIntervals        (cpIntervals), 
   m_cpPeriodicities    (cpPeriodicities),    
   m_eBoundaryHandler   (SM_BH_HITS_ALL_BOUNDARIES), 
   m_lMaxBoundaryHits   (3),
   m_bStepCuttingEnabled(FALSE),
   m_pSolutionVector    (NULL)
{
}

#endif // !__SMLOCALSOLVEND_H__


