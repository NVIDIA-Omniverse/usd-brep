// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSLC.h
* PURPOSE: Header file for SmSlc class.
**********************************************************************/

#ifndef __SMSLC_H__
#define __SMSLC_H__

#include <SmSmlibAll.h>


// Description:
// The objective is to take slices across a brep for values of z
// (small increments) and obtain a series of point loops that represent
// outer (ccw) and inner (cw) boundaries...wrt the z axis.
// The curves are obtained by pBrep->CreatePlanarSectionCurves
// These curves come in random order, and randomly reversed in direction.
// One continuous (closed) curve may come in any number of segments.
// These curves all need connecting up (head-tail) into loops. Common points
// between interior start and end segments can be removed.
// The resulting segments can be tessellated into point sets. Where three or more
// points are colinear the interior one(s) can be removed.
// Once the point-loops are cleaned up, we have to decide which are outer and which
// are inner boundaries. We compute their 2d XY min-max boxes and use 
// x ranges ( or y ranges) to 'sort' from largest to smallest.
// We then compare boxes (inside, overlap, separate) to build a nested loop, confirming
// with a boundary check.
// We finally write out each boundary: outer0, inner01, inner02...outer1,inner11 ...etc
//
// The point data is kept in arrays (SmTarray <>) of Xvals and Yvals, and these remain
// unchanged. Initially each curve segment forms start-end Seg0[] and Seg1[] pointers,
// but as segments gets joined-up we form a new list 'Loop'.
// An array of pointers in 'Loop' is kept for all the loops. 
// If a loop is reversed, the order of 'loop' gets reversed  and NOT the Xvals, Yvals.
// The start of each loop is in LoopStart[], where loop0 = LoopStart[n], 
//  and loop1 = LoopStart[n+1] - 1;

// Note: The format of SLC files is not consistent.
// This is designed to the format used in the fused deposition 
// modeling (FDM) process from Stratasys, Inc.
// referenced  in http:// www-rp.me.vt.edu/bohn/rp/SLC.html



class SM_EXPORT SmSLC
{
protected:
    // Tolerances used in SLC creation
    double m_dSameTol; // use for point-equal tol in x and y
    double m_dApproxTol;    // used for sectioning
    double m_dChordHeightTolerance;
    double m_dAngleToleranceDeg;
    double m_zPlaneTol;       // significant difference in z plane
    double m_dColinearTol;
    double m_zTol;
    double m_zCut; //  0.010 =  standard SLC z increment                             
    
public:   
    SmSLC() 
    { // constructor set defaults
        m_dSameTol = 1.0e-3; // use for point-equal tol in x and y
        m_dApproxTol = SM_ZONE_TOL_3D;    // used for sectioning        
        m_dChordHeightTolerance = 0.25;
        m_dAngleToleranceDeg = 5.0;
        m_zPlaneTol = 0.04;       // significant difference in z plane
        m_dColinearTol = SM_ZONE_TOL_3D/10.0;
        m_zTol = 0.025;
        m_zCut =  0.75; //  0.010 =  standard SLC z increment    
    };

    int WriteFile(const SmBrep* pBrep, const TCHAR *cOutputFileName);

    int WriteFile
    (
      const SmBrep* pBrep, 
      const TCHAR *cOutputFileName, 
      double ChordHeightTolerance, 
      double AngleToleranceDeg, 
      double zCut
    );

    // put and get all tolerances
    void PutTolerances
    (
      double Same,
      double Approx,
      double Chord,
      double Angle,
      double Plane,
      double Colinear,
      double zTol,
      double Cut
    )
    {
      m_dSameTol = Same;
      m_dApproxTol = Approx;
      m_dChordHeightTolerance = Chord;
      m_dAngleToleranceDeg = Angle;
      m_zPlaneTol = Plane;
      m_dColinearTol = Colinear;
      m_zTol = zTol;
      m_zCut = Cut;
    }

    void GetTolerances
    (
      double &Same,
      double &Approx,
      double &Chord,
      double &Angle,
      double &Plane,
      double &Colinear,
      double &zTol,
      double &Cut
    )
    {
      Same = m_dSameTol;
      Approx = m_dApproxTol;
      Chord = m_dChordHeightTolerance;
      Angle = m_dAngleToleranceDeg;
      Plane = m_zPlaneTol;
      Colinear = m_dColinearTol;
      zTol = m_zTol;
      Cut = m_zCut;
    }

    //put and get important tolerances
    double GetCut() const{ return (m_zCut);}
    void PutCut(double Cut) { m_zCut = Cut;}
    double GetChordHeightTol() const { return (m_dChordHeightTolerance);}
    void PutChordHeightTol(double Cht) { m_dChordHeightTolerance = Cht;}
    double GetAngleTolDeg() const { return (m_dAngleToleranceDeg);}
    void PutAngleTolDeg(double Cht) { m_dAngleToleranceDeg = Cht;}


private:
    void SortZ
    (
      const SmBrep *pBrep, 
      SmTArray <double> &zlist
    );

    void SortLoops
    (
      FILE *pFile, 
      double z, 
      int Nseg, 
      SmTArray <double> &Xvals, 
      SmTArray <double> &Yvals, 
      SmTArray <int> &Seg0, 
      SmTArray <int> &Seg1
    );
    
    // Determine loop nesting and output loops in order (outer, in, in , outer,in) etc.
    void OutputLoops
    (
      FILE *pFile, 
      double z,
      SmTArray <double> &Xvals, 
      SmTArray <double> &Yvals, 
      SmTArray <int> &LoopStart, 
      SmTArray <int> &Loop
    );
    
    void RemoveDuplicates
    (
      int Nseg, 
      SmTArray <double> &Xvals, 
      SmTArray <double> &Yvals, 
      SmTArray <int> &Seg0, 
      SmTArray <int> &Seg1
    );

    int  Clockwise
    (
      int NPts, 
      SmTArray <double> &Xvals, 
      SmTArray <double> &Yvals,
      SmTArray <int> &Loop, 
      int loop0, 
      int loop1
    );
    
    // Confirm that loop i is an outer loop to j
    int OuterConfirm
    (
      int i, 
      int j, 
      SmTArray <double> &Xvals, 
      SmTArray <double> &Yvals, 
      SmTArray <int> &LoopStart, 
      SmTArray <int> &Loop
    );
    
    // Add up angles between points in the plane to +- 360 to test whether
    // X, Y is inside or outside boundary loop ( Loop(loop0, loop1)).
    int BoundaryInOut
    (
      double X, 
      double Y,  
      SmTArray <double> &Xvals, 
      SmTArray <double> &Yvals,
      SmTArray <int> &Loop, 
      int loop0, 
      int loop1
    );
    
    void RevLoop
    (
      SmTArray <int> &Loop, 
      int loop0, 
      int loop1
    );
    
    // return 1 if 0 inside 1
    // return 2 if 1 inside 0
    // return 3 if overlap 0,1
    // return 4 if separate 0,1
    int BoxOverlap
    (
      double Xm0, 
      double XM0, 
      double Ym0, 
      double YM0, 
      double Xm1, 
      double XM1, 
      double Ym1, 
      double YM1
    );
    
    
    // If 3(or more) 2D points are colinear, remove the middle one(s)
    void RemoveColinear
    (
      ULONG nPts, 
      SmPoint3d *pPts
    );
    
    // return the next Z value 
    int NextZ
    (
      double *Z, 
      SmTArray <double> &zlist
    );
    
    // return TRUE i 2 points are 'equal'
    SmBoolean PointEqual
    (
      double X0, 
      double Y0, 
      double X1, 
      double Y1
    );

} ; // end class SmSLC


#endif // __SMSLC_H__
