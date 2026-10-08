// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmIntegrator.cpp
* PURPOSE:  Methods for integrator object.  Note that these algrithms
*    are taken from Numerical Recipies in C.
**********************************************************************/

#include "StdAfx.h"

#include <SmIntegrator.h>


// JMAX limits the number of refinements in the algorithms
#define JMAX 20         
#define JMAXP JMAX+1

// K is the number of points used in the extrapolation for
// Romberg's algorithm
#define K 4

// Define stuff used by ODE integration
#define SAFETY 0.9
#define PGROW -0.2
#define PSHRNK -0.25
// ERRCON equals (5/SAFTEY) raised to the power (1/PGROW)
#define ERRCON 1.89e-4
#define MAXSTP 10000
#define TINY 1.0e-30

// Define stuff used by BS Step
#define SAFE1 0.25
#define SAFE2 0.7
#define REDMAX SM_ZONE_TOL_3D     // Maximum step size reduction factor
#define REDMIN 0.7        // Minimum step size reduction factor
#define SCALMX 0.1        // 1/SCALMX is the maximum factor by which
                          // a step size can be increased

class SmODEEval : public SmODEIntegFuncEvalObj
{
protected:
    const SmIntegFuncEvalObj & m_crEval;
public:
    SmODEEval(const SmIntegFuncEvalObj & crEval) : m_crEval(crEval) {}
    virtual SmStatus Evaluate(double dT,                               // in :
                              SmTArray<double> & rYValues,             // NotUsed: out:
                              SmTArray<double> & rDyDxValues) const;   // out:

};

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
SmStatus SmODEEval::Evaluate
 (double             dT,           // in :
  SmTArray<double> & rYValues,     // NotUsed: out:
  SmTArray<double> & rDyDxValues)  // out:
 const
{
  SM_REF1(rYValues) ; 
    double dResult;
    SER(m_crEval.Evaluate(dT,dResult));
    rDyDxValues[1] = dResult;
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Given a function of one variable, an integration range, 
    an integration approximation algorithm, and a desired accuracy,
    compute the resultant of the integration.

NOTES: The desired accuracy may sometimes be unachievable 
    with a given step size.
***********************************************************************/
SmStatus SmIntegrator::IntegrateIt
 (double                     dA,                // in : 
  double                     dB,                // in : 
  SmIntegratorAlgorithmType  eIntegAlgorithm,   // NotUsed: in : 
  double                     dDesiredAccuracy,  // in : 
  double                   & rdResult)          // out: 
{
  SM_REF1(eIntegAlgorithm) ;
    if (smos_Fabs(dA-dB) < SM_EFF_ZERO) {
        rdResult = 0;
        return SM_SUCCESS;
    }
#if 0
    m_dS = 0.0;
    m_dA = dA;
    m_dB = dB;

    m_eIntegAlgorithm = eIntegAlgorithm;
    m_dDesiredAccuracy = dDesiredAccuracy;
    if (eIntegAlgorithm == SM_IA_SIMPSONS) {
        SER(QSimpsons(rdResult));
    }
    else if (eIntegAlgorithm == SM_IA_ROMBERG) {
        SER(QRomberg(rdResult));
    }
    else { SER(SM_ERR); }
#endif
//#if 0
    const SmODEEval sEval(m_crFunctionEvaluator);
    // Try the ODE Integrator
    SmTArray<double> dY;
    dY.Add(0.0);
    SmODEIntegrator sODE(sEval);
    ULONG lOK, lBad;
    if (sODE.RungeKuttaODEIntegrate(dY,dA,dB,dDesiredAccuracy,
        (dB-dA)/10.0,(dB-dA)/100000000.0,lOK,lBad) == SM_SUCCESS) {
        double dODEResult = dY[0];
        rdResult = dODEResult;
    }
    else {
        rdResult = 0.0;
    }
//#endif
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Interpolate a polynomial and return the value of Y and Y' at
   the given X.  Given the arrays dXA, sYA which contain lN elements,
   and a value of X, return the value of Y and an error estimate DY.
   It utilizes Neville's algorithm as described in Numerical Recipies
   in C.

NOTES: Note that lN must be less than 1000.  This routine
   uses indexing starting from 1.
***********************************************************************/
SmStatus SmIntegrator::PolynomialInterpolation(double *dXA, double *dYA, 
                                              ULONG lN, double dX, 
                                              double & rdY, double & rdDY)
{
    SM_ASSERT(lN <= 1000);
    ULONG ns = 1;

    double dif = smos_Fabs(dX-dXA[0]);
    double c[1000];
    double d[1000];
    // First find the index ns of the closest table entry
    for (ULONG i=1; i<=lN; i++) {
        double dift;
        if ( (dift=smos_Fabs(dX-dXA[i])) < dif) {
            ns = i;
            dif = dift;
        }
        c[i] = dYA[i];  // Initialize the tables
        d[i] = dYA[i];
    }

    rdY = dYA[ns--];  // Initial approximation
    // Update things
    for (ULONG m=1; m<lN; m++) {
        for (ULONG ii=1; ii<=lN-m; ii++) {
            double ho = dXA[ii] - dX;
            double hp = dXA[ii+m] - dX;
            double w = c[ii+1]-d[ii];
            double den;
            // This error can occur if two input XA's are identical
            if ( (den=ho-hp) == 0.0) SER(SM_ERR);
            den = w / den;
            d[ii] = hp * den;
            c[ii] = ho * den;
        }
        if (2*ns < (lN-m)) {
            rdDY = c[ns+1]; }
        else {
            rdDY = d[ns--];    }
        rdY += rdDY;
    }

    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: This routine computes the N'th stage refinement of an extended
    trapazoidal rule.  N must start at 1 and increase by 1 each successive
    refinement.

NOTES: 
***********************************************************************/
SmStatus SmIntegrator::Trapazoid(double dA, double dB, ULONG lN, 
                                 double & rdResult)
{
    if (lN==1) { // Compute simple average of end points for initial value
        double dResultA, dResultB;
        SER(m_crFunctionEvaluator.Evaluate(dA,dResultA));
        SER(m_crFunctionEvaluator.Evaluate(dB,dResultB));
        m_dS = 0.5 * (dB-dA) * (dResultA + dResultB);
        rdResult = m_dS;
        return SM_SUCCESS;
    }

    ULONG it=1;
    for (ULONG j=1; j+1<lN; j++) it <<= 1; // note: can't say lN-1
    double tnm = it;
    double del = (dB-dA)/tnm; // Spacing of points to be added
    double x = dA + 0.5*del;
    double sum=0.0;
    for (ULONG jj=1; jj<=it; jj++, x+=del) {
        double dResult;
        SER(m_crFunctionEvaluator.Evaluate(x,dResult));
        sum += dResult;
    }
    m_dS = 0.5 * (m_dS + (dB-dA) * sum / tnm);
    rdResult = m_dS;
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Perform integration of function over domain using
   Romberg's algorithm.

NOTES: This algorithm works for smooth functions with no
   singularities.  It is much faster then Simpson's method.
***********************************************************************/
SmStatus SmIntegrator::QRomberg(double & rdResult)
{
    // s and h store successive trapezoidal approximations and their
    // relative step sizes
    double s[JMAXP+2], h[JMAXP+2];
    double aOrig = m_dA;
    double bOrig = m_dB;
    double a = m_dA;
    double b = m_dB;

    // Note that we will not utilize the 0th element to make
    // our arrays go from 1 to N instead of 0 to N
    h[1] = 1.0;
    for (ULONG j=1; j<=JMAX; j++) {
        SER(Trapazoid(a,b,j,s[j]));
        if (j >= K) {
            double ss, dss;
            SER(PolynomialInterpolation(&h[j-K],&s[j-K],K,0.0,ss,dss));
            if (smos_Fabs(dss) < SM_EFF_ZERO + m_dDesiredAccuracy*smos_Fabs(ss)) {
                rdResult = ss;
                return SM_SUCCESS;
            }
        }
        s[j+1] = s[j];
        h[j+1] = 0.25 * h[j];
        // Try subdivision
        if (j==6 && m_dRecursionLevel < 15) {
            m_dRecursionLevel ++;
            double dOrigAcc = m_dDesiredAccuracy;
            m_dDesiredAccuracy = m_dDesiredAccuracy / 2.0;
            double dMid = (a+b)/2.0;
            m_dB = dMid;
            double dResult1, dResult2;
            SER(QRomberg(dResult1));
            m_dA = dMid;
            m_dB = bOrig;
            SER(QRomberg(dResult2));
            rdResult = dResult1 + dResult2;
            m_dA = aOrig;
            m_dB = bOrig;
            m_dDesiredAccuracy = dOrigAcc;
            m_dRecursionLevel --;
            return SM_SUCCESS;
        }
    }
    SER(SM_ERR);
    return SM_ERR;
}

/*******************************************************************//**
PURPOSE: Perform integration of function over domain using
   Romberg's algorithm.

NOTES: 
***********************************************************************/
SmStatus SmIntegrator::QSimpsons(double & rdResult)
{
    double a = m_dA;
    double b = m_dB;
    double ost = -1.0e30;
    double os = -1.0e30;
    for (ULONG j=1;j<=JMAX;j++) {
        double st;
        SER(Trapazoid(a,b,j,st));
        double s = (4.0*st-ost)/3.0;
        if (smos_Fabs(s-os) < m_dDesiredAccuracy*smos_Fabs(os)) {
            rdResult = s;
            return SM_SUCCESS;
        }
        if (s == 0.0 && os == 0.0 && j > 6) {
            rdResult = s;
            return SM_SUCCESS;
        }
        os = s;
        ost = st;
    }
    SER(SM_ERR);
    return SM_ERR;
}


/*******************************************************************//**
PURPOSE: Constructor for SmODEIntegrator

NOTES: 
***********************************************************************/
SmODEIntegrator::SmODEIntegrator
  (const SmODEIntegFuncEvalObj & crFunctionEvaluator,
   ULONG lMaxStep,
   double dTiny)
  : m_lMaxStep(lMaxStep), 
    m_dTiny(dTiny),
    m_crFunctionEvaluator(crFunctionEvaluator), 
    first(1), 
    epsold(-1)
{
}

/*******************************************************************//**
PURPOSE: This is the method that advances the Runge-Kutta method 
    by a delta value.

NOTES: Use 1-n range instead of 0-n range in arrays
***********************************************************************/
SmStatus SmODEIntegrator::RungeKuttaAdvance(SmTArray<double> & y,
                                            SmTArray<double> & dydx,
                                            double x,
                                            double h,
                                            SmTArray<double> & yout)
{
    double hh = h * 0.5;
    double h6 = h / 6.0;
    double xh = x+hh;
    double n = y.GetSize() - 1;

    // Set up and empty arrays
    dym.ReSet(); dyt.ReSet(); yt.ReSet();

    if (dym.GetSize() < y.GetSize()) {
        dym.SetSize(y.GetSize());
        dyt.SetSize(y.GetSize());
        yt.SetSize(y.GetSize());
    }

    ULONG i;

    for (i=1;i<=n;i++) yt[i] = y[i]+hh*dydx[i];
    SER(m_crFunctionEvaluator.Evaluate(xh,yt,dyt));
    for (i=1;i<=n;i++) yt[i] = y[i]+hh*dyt[i];
    SER(m_crFunctionEvaluator.Evaluate(xh,yt,dym));
    for (i=1;i<=n;i++) {
        yt[i] = y[i] + h*dym[i];
        dym[i] += dyt[i];
    }
    SER(m_crFunctionEvaluator.Evaluate(x+h,yt,dyt));
    for (i=1;i<=n;i++) {
        yout[i]=y[i]+h6*(dydx[i]+dyt[i]+2.0*dym[i]);
    }
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Do a Runge-Kutta 5th order step

NOTES: 
***********************************************************************/
SmStatus SmODEIntegrator::RungeKutta5thOrderStep(SmTArray<double> & y,
                                                 SmTArray<double> & dydx,
                                                 double & x,
                                                 double htry,
                                                 double eps,
                                                 SmTArray<double> & yscal,
                                                 double & hdid,
                                                 double & hnext)
{
    ULONG i;
    double errmax, h, htemp, xnew2;

    // Set up and empty arrays
    yerr.ReSet(); ytemp.ReSet(); 

    if (yerr.GetSize() < y.GetSize()) {
        yerr.SetSize(y.GetSize());
        ytemp.SetSize(y.GetSize());
    }


    double n = y.GetSize() - 1;
    h = htry;

    for (;;) {
        SER(RungeKuttaCashKarpAdvance(y,dydx,x,h,ytemp,yerr));
        errmax = 0.0;
        for (i=1;i<=n;i++) {
            if (smos_Fabs(yscal[i]) < SM_EFF_ZERO) yscal[i] = SM_EFF_ZERO;
            errmax = smos_Max(errmax,smos_Fabs(yerr[i]/yscal[i]));
        }
        if ( eps <= errmax * SM_EFF_ZERO )
            { return SM_ERR_INVALID_INPUT; }
        errmax /= eps;
        if (errmax <= 1.0) break;
        htemp=SAFETY*h*smos_Pow(errmax,PSHRNK);
        // Truncation error to large reduce step size
        h=(h >= 0.0 ? smos_Max(htemp,0.1*h) : smos_Min(htemp,0.1*h));
        // No more than a factor of 10
        xnew2=x+h;
        if (SM_ARE_SAME(xnew2,x)) return SM_ERR;
    }
    if (errmax > ERRCON) hnext = SAFETY*h*smos_Pow(errmax,PGROW);
    else hnext = 5.0*h;
    hdid = h;
    x += h;
    for (i=1;i<=n;i++) y[i] = ytemp[i];
    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Advance the values using a Cash-Karp Runge-Kutta technique.

NOTES: 
***********************************************************************/
SmStatus SmODEIntegrator::RungeKuttaCashKarpAdvance(SmTArray<double> & y,
                                                    SmTArray<double> & dydx,
                                                    double x,
                                                    double h,
                                                    SmTArray<double> & yout,
                                                    SmTArray<double> & yerrout)
{
    ULONG i;
    double a2=0.2, a3=0.3, a4=0.6, a5=1.0, a6=0.875, b21=0.2;
    double b31=3.0/40.0, b32=9.0/40.0, b41=0.3, b42=-0.9, b43=1.2;
    double b51= -11.0/54.0, b52=2.5, b53= -70.0/27.0, b54=35.0/27.0;
    double b61=1631.0/55296.0, b62=175.0/512.0, b63=575.0/13824.0;
    double b64=44275.0/110592.0, b65=253.0/4096.0, c1=37.0/378.0;
    double c3=250.0/621.0, c4=125.0/594.0, c6=512.0/1771.0;
    double dc5=-277.00/14336.0;

    double dc1=c1-2825.0/27648.0;
    double dc3=c3-18575.0/48384.0;
    double dc4=c4-13525.0/55296.0;
    double dc6=c6-0.25;

    ak2.ReSet();
    ak3.ReSet();
    ak4.ReSet();
    ak5.ReSet();
    ak6.ReSet();
    ytemp2.ReSet();

    if (y.GetSize() < 1) SER(SM_ERR);

    ULONG n=y.GetSize()-1;

    if (ak2.GetSize() < y.GetSize()) {
        ak2.SetSize(y.GetSize());
        ak3.SetSize(y.GetSize());
        ak4.SetSize(y.GetSize());
        ak5.SetSize(y.GetSize());
        ak6.SetSize(y.GetSize());
        ytemp2.SetSize(y.GetSize());
    }

    // First step
    for (i=1;i<=n;i++) { 
        ytemp[i]=y[i]+b21*h*dydx[i]; 
    }
    SER(m_crFunctionEvaluator.Evaluate(x+a2*h,ytemp,ak2));

    // Second step
    for (i=1;i<=n;i++) {
        ytemp[i] = y[i]+h*(b31*dydx[i]+b32*ak2[i]);
    }
    SER(m_crFunctionEvaluator.Evaluate(x+a3*h,ytemp,ak3));

    // Third step
    for (i=1;i<=n;i++) {
        ytemp[i] = y[i]+h*(b41*dydx[i]+b42*ak2[i]+b43*ak3[i]);
    }
    SER(m_crFunctionEvaluator.Evaluate(x+a4*h,ytemp,ak4));

    // Fourth step
    for (i=1;i<=n;i++) {
        ytemp[i] = y[i]+h*(b51*dydx[i]+b52*ak2[i]+b53*ak3[i]+b54*ak4[i]);
    }
    SER(m_crFunctionEvaluator.Evaluate(x+a5*h,ytemp,ak5));

    // Fifth step
    for (i=1;i<=n;i++) {
        ytemp[i] = y[i]+h*(b61*dydx[i]+b62*ak2[i]+b63*ak3[i]+b64*ak4[i]+b65*ak5[i]);
    }
    SER(m_crFunctionEvaluator.Evaluate(x+a6*h,ytemp,ak6));

    // Sixth step
    for (i=1;i<=n;i++) {
        yout[i] = y[i]+h*(c1*dydx[i]+c3*ak3[i]+c4*ak4[i]+c6*ak6[i]);
    }

    // Estimate error between fourth and fith order methods
    for (i=1;i<=n;i++) {
        yerrout[i]=h*(dc1*dydx[i]+dc3*ak3[i]+dc4*ak4[i]+dc5*ak5[i]+dc6*ak6[i]);
    }

    return SM_SUCCESS;
}
    
/*******************************************************************//**
PURPOSE: This is the top level method to do RungeKutta ODE Integration.
    You will need to supply an evaluator buy subclassing the 
    SmODEIntegFuncEvalObj class and implementing corresponding evaluate
    method.

NOTES: 
***********************************************************************/
SmStatus SmODEIntegrator::RungeKuttaODEIntegrate(SmTArray<double> & ystart,
                                                 double x1,
                                                 double x2,
                                                 double eps,
                                                 double h1,
                                                 double hmin,
                                                 ULONG & nok,
                                                 ULONG & nbad,
                                                 SmTArray<double> * pYScale //
                                                 // Y Scale is used to give some control
                                                 // over the tolerances.  
                                                 )
{
    if (SM_ARE_SAME(x1,x2) || SM_ARE_SAME(h1,SM_EFF_ZERO)) {
        return SM_SUCCESS;
    }

    m_lRecursionDepth = 0;

    if (FALSE) {
        ULONG nstp, i;
        double x,hnext,hdid,h;
        
        ULONG nvar = ystart.GetSize();
        yscal2.SetSize(ystart.GetSize()+1);
        yar.SetSize(ystart.GetSize()+1);
        dydxar.SetSize(ystart.GetSize()+1);
        
        if (pYScale) {
            yscal2.ReSet();
            yscal2.Add(0.0);
            yscal2.Append(*pYScale);
        }

        x=x1;
        h=smos_Sign(h1,x2-x1);
        nok = nbad = 0;
        
        for (i=1;i<=nvar;i++) yar[i] = ystart[i-1];
        
        for (nstp=1;nstp<=m_lMaxStep;nstp++) {
            m_lRecursionDepth ++;
            SER(m_crFunctionEvaluator.Evaluate(x,yar,dydxar));
            if (!pYScale) {
                for (i=1;i<=nvar;i++) {
                    // Scaling used to monitor accuracy.  
                    yscal2[i] = smos_Fabs(yar[i]) + smos_Fabs(dydxar[i]*h) + m_dTiny;
                }
            }
            if ((x+h-x2)*(x+h-x1) > 0.0) h=x2-x; // If step size overshoots, decrease.
            if (RungeKutta5thOrderStep(yar,dydxar,x,h,eps,yscal2,hdid,hnext) != SM_SUCCESS) {
                return SM_ERR;
            }
            m_lRecursionDepth --;

//            if (BulirschStoerStep(y,dydx,x,h,eps,yscal2,hdid,hnext) != SM_SUCCESS) {
//                return SM_ERR;
//            }
            
            if (hdid == h) ++nok; else ++nbad;
            if ((x-x2)*(x2-x1) >= 0.0) { // Are we done?
                for (i=1;i<=nvar;i++) ystart[i-1] = yar[i];
    smos_WriteBuffer(_T("\n\n\nRunge Kutta\n"));
    yar.Dump();
//                
    if (m_lRecursionDepth == 0) break;
    return SM_SUCCESS;
            }
            if (smos_Fabs(hnext) <= hmin) SER(SM_ERR);
            h = hnext;
        }
    }

    if (TRUE) {
        ULONG nstp, i;
        double x,hnext,hdid,h;
        
        ULONG nvar = ystart.GetSize();
        yscal2.ReSet();
        yar.ReSet();
        dydxar.ReSet();
        yscal2.SetSize(ystart.GetSize()+1);
        yar.SetSize(ystart.GetSize()+1);
        dydxar.SetSize(ystart.GetSize()+1);

        x=x1;
        h=smos_Sign(h1,x2-x1);
        nok = nbad = 0;
        
        for (i=1;i<=nvar;i++) yar[i] = 0.0; // ystart[i-1];
        
        for (nstp=1;nstp<=m_lMaxStep;nstp++) {
            SER(m_crFunctionEvaluator.Evaluate(x,yar,dydxar));
            for (i=1;i<=nvar;i++) {
                // Scaling used to monitor accuracy.  
                yscal2[i] = smos_Fabs(yar[i]) + smos_Fabs(dydxar[i]*h) + m_dTiny;
                if (pYScale) {
                    yscal2[i] = smos_Max(yscal2[i],(*pYScale)[i-1]);
                }
            }
            if ((x+h-x2)*(x+h-x1) > 0.0) h=x2-x; // If step size overshoots, decrease.
//            if (BulirschStoerStep(yar,dydxar,x,h,eps,yscal2,hdid,hnext) != SM_SUCCESS) {
//                return SM_ERR;
//            }
            if (RungeKutta5thOrderStep(yar,dydxar,x,h,eps,yscal2,hdid,hnext) != SM_SUCCESS) {
                return SM_ERR;
            }
            if (hdid == h) ++nok; else ++nbad;
            if ((x-x2)*(x2-x1) >= 0.0) { // Are we done?
                for (i=1;i<=nvar;i++) ystart[i-1] = yar[i];
                return SM_SUCCESS;
            }
            if (smos_Fabs(hnext) <= hmin) SER(SM_ERR);
            h = hnext;
        }
        SE(SM_ERR);
    }
    



    SE(SM_ERR);
    return SM_ERR;
}



/*******************************************************************//**
PURPOSE: This method performs a modified midpoint step.  It advances
    y(x) from a point x to a point x + H by a sequence of n substeps each
    size h.

NOTES: 
***********************************************************************/
SmStatus SmODEIntegrator::ModifiedMidpointStep(SmTArray<double> & y,
                                               SmTArray<double> & dydx,
                                               double xs,
                                               double htot,
                                               ULONG nstep,
                                               SmTArray<double> & yout)
{
    ULONG n, i;
    double x, swap, h2, h;

    SmTArray<double> & ym = dym;
    SmTArray<double> & yn = dyt;
    if (y.GetSize() < 1) SER(SM_ERR);

    ULONG nvar = y.GetSize()-1;

    ym.ReSet();
    yn.ReSet();
    if (ym.GetSize() < nvar+1) {
        ym.SetSize(nvar+1);
        yn.SetSize(nvar+1);
    }

    h = htot/nstep;
    for (i=1;i<=nvar;i++) {
        ym[i] = y[i];
        yn[i] = y[i]+h*dydx[i];
    }
    x = xs + h;
    SER(m_crFunctionEvaluator.Evaluate(x,yn,yout));
    h2=2.0*h;
    for (n=2;n<=nstep;n++) {
        for (i=1;i<=nvar;i++) {
            swap = ym[i] + h2*yout[i];
            ym[i]=yn[i];
            yn[i]=swap;
        }
        x += h;
        SER(m_crFunctionEvaluator.Evaluate(x,yn,yout));
    }
    for (i=1;i<=nvar;i++) {
        yout[i] = 0.5*(ym[i]+yn[i]+h*yout[i]);
    }

    return SM_SUCCESS;
}


/*******************************************************************//**
PURPOSE: Use polynomial extrapolation to evaluate nv functions at x=0
    by fitting a polynomial to a sequence of estimates with progressively
    smaller values of x=xest, and corresponding function vectors
    yext[1..nv].  Extrapolated function values are output as yz[1..nv],
    and their estimated error is output as dy[1..nv].

NOTES: 
***********************************************************************/
SmStatus SmODEIntegrator::PolynomialExtrapolation(ULONG iest,
                                                  double xest,
                                                  SmTArray<double> & yest,
                                                  SmTArray<double> & yz,
                                                  SmTArray<double> & dy)
{
    ULONG k1, j;
    double q, f2, f1, delta;

    ULONG nv = yz.GetSize() - 1;
    if (car.GetSize() != nv+1) {
        car.SetSize(nv+1);
    }

    xar[iest] = xest;

    for (j=1; j<=nv; j++) dy[j] = yz[j] = yest[j];
    if (iest==1) {
        for (j=1; j<=nv; j++) d[j][1] = yest[j];
    }
    else {
        for (j=1; j<=nv; j++) car[j] = yest[j];
        for (k1=1;k1<iest;k1++) {
            delta = 1.0 / (xar[iest-k1]-xest);
            f1=xest*delta;
            f2=xar[iest-k1]*delta;
            for (j=1; j<=nv; j++) {
                q=d[j][k1];
                d[j][k1]=dy[j];
                delta=car[j]-q;
                dy[j]=f1*delta;
                car[j]=f2*delta;
                yz[j] += dy[j];
            }
        }
        for (j=1; j<=nv; j++) d[j][iest]=dy[j];
    }
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: This method takes a Bulirsch-Stoer Step.

NOTES: 
***********************************************************************/
SmStatus SmODEIntegrator::BulirschStoerStep(SmTArray<double> & y,
                                    SmTArray<double> & dydx,
                                    double & xx,
                                    double htry,
                                    double eps,
                                    SmTArray<double> & yscal,
                                    double & hdid,
                                    double & hnext)
{
    ULONG ii, iq, k, kk, km = 0;
    double eps1, errmax = 0.0, fact, h, red = 0.0, scale = 0.0, work, wrkmin, xest;
    ULONG nseq[SM_IMAXX+1];
    nseq[0] = 0;
    nseq[1] = 2;
    nseq[2] = 4;
    nseq[3] = 6;
    nseq[4] = 8;
    nseq[5] = 10;
    nseq[6] = 12;
    nseq[0] = 14;
    nseq[0] = 16;
    nseq[0] = 18;

    ULONG reduct, exitflag=0;
    
    if (y.GetSize() < 1) SER(SM_ERR);
    ULONG nv = y.GetSize()-1;
    
    if (d.GetSize() == 0) {
        d.SetSize(nv+1);
        ddata.SetSize((nv+1)*(SM_KMAXX+1));
        for (ii=0; ii<=nv; ii++) {
            d[ii] = &ddata.GetDataArray()[ii*(SM_KMAXX+1)];
        }
        err.SetSize(SM_KMAXX+1);
        xar.SetSize(SM_KMAXX+1);
        yerr.SetSize(nv+1);
        ysav.SetSize(nv+1);
        yseq.SetSize(nv+1);
    }
    
    if (eps != epsold) {
        hnext = xnew = -1.0e29;
        eps1=SAFE1*eps;
        a[1]=nseq[1]+1;
        for (k=1;k<=SM_KMAXX;k++) a[k+1] = a[k]+nseq[k+1];
        for (iq=2;iq<=SM_KMAXX;iq++) {
            for (k=1; k<iq; k++) {
                alf[k][iq]=smos_Pow(eps1,(a[k+1]-a[iq+1])/
                    ((a[iq+1]-a[1]+1.0)*(2*k+1)));
            }
        }
        epsold=eps;
        for (kopt=2;kopt<SM_KMAXX;kopt++) {
            if (a[kopt+1] > a[kopt]*alf[kopt-1][kopt]) break;
        }
        kmax = kopt;
    }
    
    h=htry;
    for ( ii=1; ii<=nv; ii++ ) ysav[ii] = y[ii];
    if (xx != xnew || h != hnext) {
        first = 1;
        kopt=kmax;
    }
    reduct=0;
    for (;;) {
        for (k=1;k<=kmax;k++) {
            xnew=xx+h;
            if (xnew == xx) SER(SM_ERR);
            SER(ModifiedMidpointStep(ysav,dydx,xx,h,nseq[k],yseq));
            xest=h/nseq[k];
            xest = xest*xest;
            SER(PolynomialExtrapolation(k,xest,yseq,y,yerr));
            if (k!=1) {
                errmax=TINY;
                for ( ii=1; ii<=nv; ii++ ) {
                    errmax = smos_Max(errmax,smos_Fabs(yerr[ii]/yscal[ii]));
                }
                if ( eps <= errmax * SM_EFF_ZERO )
                    { return SM_ERR_INVALID_INPUT; }
                errmax /= eps;
                km=k-1;
                err[km]=smos_Pow(errmax/SAFE1,1.0/(2*km+1));
            }
            if (k != 1 && (k >= kopt-1 || first)) {
                if (errmax < 1.0) {
                    exitflag = 1;
                    break;
                }
                if (k == kmax || k == kopt+1) {
                    red = SAFE2/err[km];
                    break;
                }
                else if (k == kopt && alf[kopt-1][kopt] < err[km]) {
                    red = 1.0/err[km];
                    break;
                }
                else if (kopt == kmax && alf[km][kmax-1] < err[km]) {
                    red=alf[km][kmax-1]*SAFE2/err[km];
                    break;
                }
                else if (alf[km][kopt] < err[km]) {
                    red=alf[km][kopt-1]/err[km];
                    break;
                }
            }
        }
        if (exitflag) break;
        red=smos_Min(red,REDMIN);
        red=smos_Max(red,REDMAX);
        h *= red;
        reduct=1;
    }
    xx=xnew;
    hdid = h;
    first=0;
    wrkmin=1.0e35;
    for (kk=1;kk<=km;kk++) {
        fact=smos_Max(err[kk],SCALMX);
        work=fact*a[kk+1];
        if (work<wrkmin) {
            scale=fact;
            wrkmin=work;
            kopt=kk+1;
        }
    }
    hnext=h/scale;
    if (kopt >= k && kopt != kmax && !reduct) {
        fact = smos_Max(scale/alf[kopt-1][kopt],SCALMX);
        if (a[kopt+1]*fact <= wrkmin) {
            hnext=h/fact;
            kopt++;
        }
    }
    
    return SM_SUCCESS;
}
