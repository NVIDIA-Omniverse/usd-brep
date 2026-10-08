// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmHyperbola.h
* PURPOSE: Header file for Hyperbola curve.
**********************************************************************/

#ifndef __SMHYPERBOLA_H__
#define __SMHYPERBOLA_H__

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

/*******************************************************************//**
PURPOSE: The SmHyperbola class defines a Hyperbola.

NOTES:
  Hyperpola(s) =   Origin 
                 + dSemiAxis      * CosineH(s) * XAxis
                 + dSemiImageAxis * SineH(s)   * YAxis
     where s is defined over the real numbers
             and is in hyperbolic radians with practical -5.0 < s <= 5.0
     Reminder:  s  |  cosh(s) |  sinh(s)
              -----+----------+----------
               0   |     1.0  |     0.0
               1.0 |     1.54 |     1.17
               2.0 |     3.76 |     3.62
               4.0 |    27.31 |    27.28
               8.0 | 1,490.48 | 1,490.48
              16.0 |   4.0E06 |   4.0E06
              32.0 |   3.9E13 |   3.9E13
***********************************************************************/
class SM_EXPORT SmHyperbola : public SmBSplineCurve
{
protected:
  SmAxis2Placement    m_vPosition;
  SmExtent1d          m_vAnalDomain;    // hyperbolic radians - practical limits
                                        // 
  double              m_dSemiAxis;      // Length of semi axis
  double              m_dSemiImageAxis; // Length of semi imaginary axis
  
public:
  // constructor
  SmHyperbola(const SmPoint3d  & crCenter,
              const SmVector3d & crXAxis,
              const SmVector3d & crYAxis,
              const SmExtent1d & crAnalDomain,
              double             dSemiAxis,
              double             dSemiImageAxis,
              ULONG              lDimension = 3,
              const SmContext  * cpContext = NULL) ; // in : must be given for automatic variables, optional for
                                                     //      stack variables built with overloaded new.
  // empty constructor for I/O
  SmHyperbola() { }
  
  // copy constructor
  SmHyperbola(const SmHyperbola & crHyperbola);
  
  // destructor
  virtual ~SmHyperbola() { }
  
  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const;
  
  // Select a different section of the curve
  virtual SmStatus AdjustSTEPInterval(const SmExtent1d & crNewSTEPInterval);
  
  virtual SmStatus Copy(const SmContext & crContext,
                        SmCurve *& rpNewCurve) const;
  
  // todo: 
  //      virtual SmStatus ConvertTFromSTEPToNURBS(double dSTEPParam,
  //                                               double & rdNURBSParam) const;
  //      
  //      virtual SmStatus ConvertTFromNURBSToSTEP(double dNURBSParam,
  //                                               double & rdSTEPParam) const;
  
  // create a step parameterized curve                          
  static SmStatus CreateCanonical(const SmContext & crContext,
                                  const SmAxis2Placement & crOrigin,
                                  double dSemiAxis,
                                  double dSemiImageAxis,
                                  SmHyperbola *& rpNewHyperbola);
  
  virtual SmStatus EvaluateSTEP(double dSTEPParameter,              // in :
                                ULONG lNumDerivatives,              // in :
                                SmBoolean bFromLeft,                // NotUsed: in : if P is on interval boundary
                                                                    //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                                                    //      FALSE = evaluate P in lower interval where P is on the right of the interval
                                SmVector3d aPointAndDerivatives[],  // out:
                                SmZoneTol3d dZoneTol3d=0.0) const;  // NotUsed: in : 
  
  SmStatus GetCanonical(SmAxis2Placement & rOrigin,
                        double & rdSemiAxis,
                        double & rdSemiImageAxis) const;
  
  virtual SmExtent1d GetSTEPInterval() const { return m_vAnalDomain; }
  virtual SmExtent1d GetMaxAnalyticDomain() const { SmExtent1d sDom( -6, 6 ); return sDom; } // see above
  
  virtual SmStatus GlobalPointSolveSTEP(const SmExtent1d    & crInterval,            // in : step domain
                                        SmSolverOperationType eSolverOperation,      // in :
                                        const SmPoint3d     & crTestPoint,           // in :
                                        double                dDistanceTolerance,    // in :
                                        const double        * cpdOptTargetDistance,  // in :
                                        const SmVector3d    * cpOptVectors,          // NotUsed: in :
                                        SmSolutionRequestedType eSolutionRequested,  // in :
                                        SmSolutionArray     & rSolutions);           // out: stepdomain parameters
  
  virtual SmBoolean IsBounded() const;  // TRUE=finite (FALSE=infinite) parameter range
  
  virtual SmBoolean IsAnalytic() const { return TRUE; }

  virtual SmBoolean IsArc(ULONG, double, SmAxis2Placement&, double&, double&, double&) const { return FALSE; }  
  
  virtual SmStatus MakeNurb();
  
  virtual SmStatus ReverseParameterization(const SmExtent1d & crOldInterval,
                                           SmExtent1d & rNewInterval);
  
  virtual SmStatus STEPInversion(const SmPoint3d      & crPointOnCurve,        // in :
                                 double               & rdAnalyticParameter,   // out:    
                                 SmCurveLocationType  * pOptLoc=NULL,          // out:
                                 SmZoneTol3d            dZoneTol=0.0) const;   // NotUsed: in :
  
  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling
  
  // get Curve Memory size - plus its attributes
  virtual ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,      // out: bigger size of all allocated memory in bytes
                              SmMarkType eMarkType=SM_MT_NOMARK) // in : uses without increment eMarkType value
                             const ;
  
  virtual SmStatus WriteToDB (SmDatabaseIO & rDB,                      // in : target output stream
                              ULONG          lDBVersionNumber) const ; // in : database version to get proper sequence of writes  
  
  static  SmStatus ReadFromDB(SM_TYPE           lType,              // NotUsed: in : Object type to be read
                              SmDatabaseIO    & rDB,                // in : target output stream
                              ULONG             lDim,               // in : curve image space dim, 2 or 3                                                                                    
                              const SmContext & crContext,          // in : context for new object construction
                              SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                                                    //      NotNULL on input = pointer to an empty object to be filled by this routine
                              ULONG             lDBVersionNumber) ; // in : database version to get proper sequence of writes
  
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmHyperbola,SmBSplineCurve,SmHyperbola_TYPE);
  
} ; // end SmHyperbola

#endif // !__SMHYPERBOLA_H__

