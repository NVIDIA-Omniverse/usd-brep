// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmIntegrator.h
* PURPOSE: Perform numerical integration of a function of one
*    variable over a given parametric range.  Note that these algrithms
*    are taken from Numerical Recipies in C.
**********************************************************************/

#ifndef __SMINTEGRATOR_H__
#define __SMINTEGRATOR_H__

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#define SM_KMAXX 8           // Max row number in extrapolation
#define SM_IMAXX (SM_KMAXX+1)   

/*******************************************************************//**
PURPOSE: This type defines which type of integration algorithm is to
    be used when doing integration.

NOTES: Please note that SM_IA_ROMBERG is usually much faster.
***********************************************************************/
enum SmIntegratorAlgorithmType {
    SM_IA_ROMBERG,
    SM_IA_SIMPSONS
};


/*******************************************************************//**
PURPOSE:  This is the pure virtual object which supports evaluation
  Simply subclass this object.  Add your fields and implement
  the virtual Evaluate Function 

NOTES: 
***********************************************************************/
class
SM_EXPORT 
SmIntegFuncEvalObj
{
public:
    SmIntegFuncEvalObj() {}
    virtual ~SmIntegFuncEvalObj() {}
    // Note that we should look for an answer in the evaluate
    // because it probably has a better Idea of the geometry of
    // the situation and the tolerance will be more meaningfull
    virtual SmStatus Evaluate(double dT, double & rdResult) const;
};

inline SmStatus SmIntegFuncEvalObj::Evaluate(double , double & ) const
    { SE(SM_ERR); return SM_ERR; }

/*******************************************************************//**
PURPOSE: This object performs numerical integration using one of two
    algorithms (Romberg or Simpsons).  Romberg is typically faster.

NOTES: To integrate a function, subclass the SmIntegFuncEvalObj
    and put the evaluator for the function in that subclass.
***********************************************************************/
class
SM_EXPORT 
SmIntegrator
{
protected:
    const SmIntegFuncEvalObj & m_crFunctionEvaluator;
    double                     m_dS;
    double                     m_dA;                   // integral interval start
    double                     m_dB;                   // integral interval end
    SmIntegratorAlgorithmType  m_eIntegAlgorithm;
    double                     m_dDesiredAccuracy;
    ULONG                      m_dRecursionLevel;

public:
    // Initilization 
    SmIntegrator(const SmIntegFuncEvalObj & crFunctionEvaluator);
    ~SmIntegrator() {}

    // Attempt to find solution
    SmStatus IntegrateIt(double                    dA,               // in :
                         double                    dB,               // in :
                         SmIntegratorAlgorithmType eIntegAlgorithm,  // NotUsed: in :
                         double                    dDesiredAccuracy, // in :
                         double                  & rdResult) ;       // out:
    SmStatus PolynomialInterpolation(double * dXA,
                                     double * dYA,
                                     ULONG    lN, 
                                     double   dX,
                                     double & rdY,
                                     double & rdDY);

protected:
    SmStatus Trapazoid(double dA, double dB, ULONG lN, double & rdResult);
    SmStatus QRomberg (double & rdResult);
    SmStatus QSimpsons(double & rdResult);

};

inline SmIntegrator::SmIntegrator(const SmIntegFuncEvalObj & crFunctionEvaluator)
:  m_crFunctionEvaluator(crFunctionEvaluator), m_dS(0.0), m_dRecursionLevel(0)
{
}

// Evaluator for the ODE integrator
class
SM_EXPORT 
SmODEIntegFuncEvalObj
{
public:
    SmODEIntegFuncEvalObj() {}
    virtual ~SmODEIntegFuncEvalObj() {}
    // Note that we should look for an answer in the evaluate
    // because it probably has a better Idea of the geometry of
    // the situation and the tolerance will be more meaningfull
    virtual SmStatus Evaluate(double dT, 
                              SmTArray<double> & rYValues,
                              SmTArray<double> & rDyDxValues) const;
};

inline SmStatus SmODEIntegFuncEvalObj::Evaluate(double , SmTArray<double> &,
                                                SmTArray<double> & ) const
    { SE(SM_ERR); return SM_ERR; }

class
SM_EXPORT
SmODEIntegrator
{
protected:
    ULONG            m_lRecursionDepth;
    ULONG            m_lMaxStep;
    double           m_dTiny;
    SmTArray<double> dym;
    SmTArray<double> dyt;
    SmTArray<double> yt;
    SmTArray<double> yerr;
    SmTArray<double> ytemp;
    SmTArray<double> ak2;
    SmTArray<double> ak3;
    SmTArray<double> ak4;
    SmTArray<double> ak5;
    SmTArray<double> ak6;
    SmTArray<double> ytemp2;
    SmTArray<double> yscal2;
    SmTArray<double> yar;
    SmTArray<double> dydxar;
    SmTArray<double> xar;
    SmTArray<double*> d;
    SmTArray<double> car;
    SmTArray<double> ddata;
    SmTArray<double> err;
    SmTArray<double> ysav;
    SmTArray<double> yseq;
    const SmODEIntegFuncEvalObj & m_crFunctionEvaluator;
    ULONG            first;
    ULONG            kmax;
    ULONG            kopt;
    double           epsold;
    double           xnew;
    double           a[SM_IMAXX+1];
    double           alf[SM_KMAXX+1][SM_KMAXX+1];

public:
    // Initilization 
    SmODEIntegrator(const SmODEIntegFuncEvalObj & crFunctionEvaluator,
                    ULONG lMaxStep=10000,
                    double dTiny=SM_EFF_ZERO_SQ);

    ~SmODEIntegrator() {}

    SmStatus RungeKuttaAdvance(SmTArray<double> & y,
                               SmTArray<double> & dydx,
                               double x,
                               double h,
                               SmTArray<double> & yout);

    SmStatus RungeKutta5thOrderStep(SmTArray<double> & y,
                                    SmTArray<double> & dydx,
                                    double & x,
                                    double htry,
                                    double eps,
                                    SmTArray<double> & yscal,
                                    double & hdid,
                                    double & hnext);

    SmStatus RungeKuttaCashKarpAdvance(SmTArray<double> & y,
                                       SmTArray<double> & dydx,
                                       double x,
                                       double h,
                                       SmTArray<double> & yout,
                                       SmTArray<double> & yerr);
    
    SmStatus RungeKuttaODEIntegrate(SmTArray<double> & ystart,
                                       double x1,
                                       double x2,
                                       double eps,
                                       double h1,
                                       double hmin,
                                       ULONG & nok,
                                       ULONG & nbad,
                                       SmTArray<double> * pOptYScale = NULL);

    SmStatus ModifiedMidpointStep(SmTArray<double> & y,
                                       SmTArray<double> & dydx,
                                       double xs,
                                       double htot,
                                       ULONG nstep,
                                       SmTArray<double> & yout);


   SmStatus BulirschStoerStep(SmTArray<double> & y,
                                    SmTArray<double> & dydx,
                                    double & x,
                                    double htry,
                                    double eps,
                                    SmTArray<double> & yscal,
                                    double & hdid,
                                    double & hnext);

   SmStatus PolynomialExtrapolation(ULONG iest,
                                    double xext,
                                    SmTArray<double> & yest,
                                    SmTArray<double> & yz,
                                    SmTArray<double> & dy);


};

#endif // !__SMINTEGRATOR_H__


