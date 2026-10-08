// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmParabola.h
* PURPOSE: Header file for Parabola curve.
**********************************************************************/

#ifndef __SMPARABOLA_H__
#define __SMPARABOLA_H__

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
PURPOSE: The SmParabola class defines a Parabola.

NOTES:
  Parabola(s) =   Origin
                + FocalDist * s*s * XAxis
                + FocalDist * 2*s * YAxis
  where s is defined over all real numbers                    
***********************************************************************/
class SM_EXPORT SmParabola : public SmBSplineCurve
{
protected:
  SmAxis2Placement    m_vPosition;
  SmExtent1d          m_vAnalDomain;
  double              m_dFocalDist;
  
public:
  // constructor
  SmParabola(const SmPoint3d  & crCenter,
             const SmVector3d & crXAxis,
             const SmVector3d & crYAxis,
             const SmExtent1d & crAnalDomain,
             double             dFocalDist,
             ULONG              lDimension = 3,
             const SmContext  * cpContext = NULL) ; // in : must be given for automatic variables, optional for
                                                    //      stack variables built with overloaded new.
  // empty constructor for I/O
  SmParabola() { }
  
  // copy constructor
  SmParabola(const SmParabola & crParabola);
  
  // destructor
  virtual ~SmParabola() { }
  
  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const;
  
  // Select a different section of the curve
   SmStatus AdjustSTEPInterval(const SmExtent1d & crNewSTEPInterval);
  
  virtual SmStatus Copy(const SmContext & crContext,
                        SmCurve *& rpNewCurve) const;
  
  // create a step parameterized curve                          
  static SmStatus CreateCanonical(const SmContext & crContext,
                                  const SmAxis2Placement & crOrigin,
                                  double dFocalDist,
                                  SmParabola *& rpNewParabola);
  
  virtual SmStatus EvaluateSTEP
      (double dSTEPParameter,             // in :
       ULONG lNumDerivatives,             // in :
       SmBoolean bFromLeft,               // NotUsed: in : if P is on interval boundary
                                          //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                          //      FALSE = evaluate P in lower interval where P is on the right of the interval
       SmVector3d aPointAndDerivatives[], // out: 
       SmZoneTol3d dZoneTol3d=0.0) const; // NotUsed: in :
  
  SmStatus GetCanonical(SmAxis2Placement & rOrigin,
                        double & rdFocalDist) const;
  
  virtual SmExtent1d GetSTEPInterval()      const { return m_vAnalDomain; }
  virtual SmExtent1d GetMaxAnalyticDomain() const { SmExtent1d sDom( -100, 100 ); return sDom; }
  
  virtual SmStatus GlobalPointSolveSTEP(const SmExtent1d    & crInterval,            // in : step domain
                                        SmSolverOperationType eSolverOperation,      // in :
                                        const SmPoint3d     & crTestPoint,           // in :
                                        double                dDistanceTolerance,    // in :
                                        const double        * cpdOptTargetDistance,  // in :
                                        const SmVector3d    * cpOptVectors,          // NotUsed: in :
                                        SmSolutionRequestedType eSolutionRequested,  // in :
                                        SmSolutionArray     & rSolutions);           // out: step domain parameters
  
  virtual SmBoolean IsBounded() const;   // TRUE=finite (FALSE=infinite) parameter range
  
  virtual SmBoolean IsAnalytic() const { return TRUE; }

  virtual SmBoolean IsArc(ULONG, double, SmAxis2Placement&, double&, double&, double&)           const { return FALSE; }  
  
  virtual SmStatus MakeNurb();
  
  virtual SmStatus ReverseParameterization(const SmExtent1d & crOldInterval,
                                           SmExtent1d & rNewInterval);
  
  virtual SmStatus STEPInversion(const SmPoint3d      & crPointOnCurve,         // in : Target Point                         
                                 double               & rdAnalyticParameter,    // out: STEPParameter closest to Target Point, range:m_vAnalDomain or positive  
                                 SmCurveLocationType  * pOptLoc=NULL,           // out: Point's classification to curve, NULL to ignore, default:[NULL]
                                 SmZoneTol3d            dZoneTol3d=0.0) const ; // NotUsed: in : Unused
  
  // todo: add these functions
  //      virtual SmStatus ConvertTFromSTEPToNURBS(double dSTEPParam,
  //                                               double & rdNURBSParam) const;
  //      
  //      virtual SmStatus ConvertTFromNURBSToSTEP(double dNURBSParam,
  //                                               double & rdSTEPParam) const;
  
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
  SM_COMMON(SmParabola,SmBSplineCurve,SmParabola_TYPE);
  
} ; // end SmParabola

#endif // !__SMPARABOLA_H__
