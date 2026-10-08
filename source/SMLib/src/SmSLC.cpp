// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSLC.cpp
* PURPOSE: Source file for SmSLC class methods.
**********************************************************************/

/*******************************************************************//**
PURPOSE: Write out a brep in SLC format

NOTES: 
***********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <SmSLC.h>
#include <SmSmlibAll.h>



int SmSLC:: WriteFile(const SmBrep* pBrep,  const TCHAR *cOutputFileName, 
        double ChordHeightTolerance, double AngleToleranceDeg, double zCut)
{
    PutCut(zCut);    
    PutChordHeightTol(ChordHeightTolerance);
    PutAngleTolDeg(AngleToleranceDeg);
    return (WriteFile(pBrep, cOutputFileName));   
}

int SmSLC::WriteFile(const SmBrep* pBrep, const TCHAR *cOutputFileName)
                         
{
    const SmContext *cpContext = pBrep->GetContext() ; SM_ASSERT(cpContext != NULL) ;
    FILE *pFile = NULL;
    if (cOutputFileName) {
        pFile = SM_FOPEN(cOutputFileName, _T("w"));
        if (pFile == NULL)
           return (1);
        SM_FPRINTF(pFile, _T("Slice V0\n"));
        SM_FPRINTF(pFile, _T("FILE(%s)\n"), cOutputFileName);
        SM_FPRINTF(pFile, _T("SLICES()\n"));
    }
    
    
    SmExtent3d sBBox;    
    
    
    SER(pBrep->CalculateBoundingBox(sBBox)); // Get box of brep
    //    sBBox.ExpandAbsolute(0.1);
    
    //     sBBox.Draw();
    
    SmPoint3d sPlanePoint = sBBox.Evaluate(0.5, 0.5, 0.0);
    SmVector3d PlaneNormal(0.0, 0.0, 1.0);
    
    // a 'z-list' is built to section z space according to min-max boxes
    // of face in the z direction. The idea is to capture highly sculptured
    // narrow z-ranges for extra sections
    SmTArray <double> zlist;
    SortZ(pBrep, zlist);
    // zlist.Dump();
    
    // return the next Z value
    double Z = zlist[0];
    double Znext = Z;
    int last = 0;
    do 
    {
        if (last == 1)
            break;
        if (NextZ(&Znext, zlist) ==0)
            last = 1;
        double zDelta = Znext - Z;
        sPlanePoint.z = Z;
        
        SmTArray < SmCurve*> s3DCurves;  // Holds 3D model space curves
        SmTArray < SmCurve*> sUVCurves;  // Holds 2D parameter space curves
        
        SmTArray < double> Xvals;
        SmTArray < double> Yvals;
        SmTArray < int> Seg0;
        SmTArray < int> Seg1;
        int Npt = 0;       
        
        
        // Create the planar section curves from the brep and the plane
        SER(pBrep->CreatePlanarSectionCurves(*cpContext, 
                                             sPlanePoint, 
                                             PlaneNormal,
                                             SM_CAST_APPROXTOL3D_PTR(&m_dApproxTol), 
                                             NULL, 
                                             &s3DCurves, 
                                             &sUVCurves, 
                                             NULL));
        
        // Draw the resulting curve segments
        int Nseg = s3DCurves.GetSize();
        if (Nseg > 0)
        {
            if (pFile) SM_FPRINTF(pFile, _T("Z %lf %lf\n"), sPlanePoint.z, zDelta);    
            
            int Mseg = 0;
            
            for (int j = 0; j < Nseg; j++)
            {
                SmCurve *pCurve = s3DCurves[j];
                //     pCurve->Draw();
                
                //SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve, pCurve);
                SmExtent1d cInterval = pCurve->GetNaturalInterval();
                SmPoint3d sData[300];
                SmTArray < SmPoint3d> sPoints(300, sData);
                if (pCurve->IsDegenerate())
                {
                    SmPoint3d sPnt;              
                    SER(pCurve->EvaluatePoint(cInterval.GetMin(), sPnt));
                    sPoints.Add(sPnt);
                }
                else 
                {
                    SER(pCurve->Tessellate(cInterval, m_dChordHeightTolerance, m_dAngleToleranceDeg,
                        3, NULL, &sPoints));
                }
                SmPoint3d *pPts = sPoints.GetDataArray();
                ULONG nPts = sPoints.GetSize();
                if (nPts>1)
                {
                    RemoveColinear(nPts, pPts);
                    Seg0.Add(Npt);
                    
                    for (ULONG k = 0; k < nPts; k++)
                    {
                        if (pPts[k].x < 1.0e22)
                        {
                            // ie not removed colinear
                            Xvals.Add(pPts[k].x);
                            Yvals.Add(pPts[k].y);
                            Npt++;
                        }
                    }
                    Seg1.Add(Npt - 1);
                    Mseg++;
                }
                
                // Delete the curves once we are done
                SM_ASSERT(s3DCurves[j] != NULL) ; delete s3DCurves[j] ; s3DCurves[j] = NULL ;
                SM_ASSERT(sUVCurves[j] != NULL) ; delete sUVCurves[j] ; sUVCurves[j] = NULL ;
            }
            SortLoops(pFile, sPlanePoint.z, Mseg, Xvals, Yvals, Seg0, Seg1);
        } // end if nseg >0
        Z = Znext;
    } while (last == 0);
    
    if (pFile) 
    {   
        SM_FPRINTF(pFile, _T("END\n"));
        fclose(pFile);
    }

    //  pBrep->Draw();
    //  pBrep->Dump();
    
    return (0);
}

// Compute z range from brep min-max box
// OR : from brep, get the min-max boxes of every face, and add the min.z and
// max.z values into an ascending zlist with no duplication or close values.

void SmSLC::SortZ(const SmBrep *pBrep, SmTArray <double> &zlist)

{   
    // Commented out works from face breps...
    //    SmTArray < SmFace*> sFaces;
    //    pBrep->GetFaces(sFaces);
    //    for (ULONG k = 0; k < sFaces.GetSize(); k++)
    //    {
    //        SmFace *pFace = sFaces[k];
    
    SmExtent3d sBBox;
    //      pFace->CalculateBoundingBox(sBBox); // Get box of each face
    pBrep->CalculateBoundingBox(sBBox); // work from brep of brep     
    double dParam = 0.0;
    SmPoint3d sP1 = sBBox.Evaluate(dParam, dParam, dParam);
    dParam = 1.0;
    SmPoint3d sP2 = sBBox.Evaluate(dParam, dParam, dParam); 
    
    //        if (fabs(sP2.z - sP1.z) < m_zPlaneTol)
    //            continue; // ignore faces in the z plane
    
    // insert into ascending  ZList if unique
    int ifound = -1;
    
    double dZ;
    ULONG idx, jdx = 0;
    for (idx = 0; idx < zlist.GetSize(); idx++)
    {
        dZ = zlist[idx];
        if (fabs(dZ -  sP1.z) < m_zPlaneTol)
        {
            // not unique, so
            ifound =idx; // keep larger of (from below)
            if (sP1.z > dZ)
                zlist[idx] = sP1.z;
        }
        else if (dZ > sP1.z)
        { 
            jdx = idx; 
            break;
        }
    }
    if (jdx == 0)
        jdx = idx;
    if (ifound < 0)
        zlist.InsertAt(jdx, sP1.z, 1); // unique, insert
    
    // insert into ascending  ZList if unique
    ifound = -1;
    jdx = 0;
    for (idx = 0; idx < zlist.GetSize(); idx++)
    {
        dZ = zlist[idx];
        
        if (fabs(dZ -  sP2.z) < m_zPlaneTol)
        {
            // not unique, so
            ifound =idx; // keep smaller of (from above)
            if (sP2.z < dZ)
                zlist[idx] = sP2.z;
        }
        
        else if (dZ > sP2.z)
        { 
            jdx = idx; 
            break; 
        }
    }
    if (jdx == 0)
        jdx = idx;
    if (ifound < 0)
        zlist.InsertAt(jdx, sP2.z, 1); // unique, insert
    //   }
    
    return;
}

SmBoolean SmSLC::PointEqual(double X0, double Y0, double X1, double Y1)
{
    if ((fabs(X0 - X1) < m_dSameTol) &&(fabs(Y0 - Y1) < m_dSameTol))
        return TRUE;
    return (FALSE);
}



// Top and tail link up point data into loops, removing duplicate points
void SmSLC::SortLoops(FILE *pFile, double z, int Nseg, 
                         SmTArray <double> &Xvals, SmTArray <double> &Yvals, 
                         SmTArray <int> &Seg0, SmTArray <int> &Seg1)
{
    int j;
    RemoveDuplicates(Nseg, Xvals, Yvals, Seg0, Seg1);
    
    SmTArray <int> Loop;  // list of point numbers into Xval,Yval
    SmTArray <int> LoopStart;
    
     // try to build loops based on first/next unused segment
    for (int Iseg = 0; Iseg < Nseg; Iseg++)
    {            
        // are there any unused loop starts
        int Ist = Seg0[Iseg];
        if (Ist > -1)
        { // first unused segment
            int Iend = Seg1[Iseg];
            double X0 = Xvals[Ist];  // save values to match for loop end
            double Y0 = Yvals[Ist];
            LoopStart.Add(Loop.GetSize()); // save start of loop
            
            for (int i = Ist; i <= Iend; i++)
                Loop.Add(i);// add in first segment

            Seg0[Iseg] = -1; // do not reuse 1st segment
            double X1 = Xvals[Iend]; // get end values
            double Y1 = Yvals[Iend];
            int looking = 1;
            while (looking) 
            {
                if (PointEqual(X0, Y0, X1, Y1)) // we have a loop or forced closure
                {
                    // start = end so found a loop
                    looking = 0;                                
                    continue; // to end of looking
                }
                else    // have not yet a closed loop
                {
                    for (j = 1; j < Nseg; j++) // skip the first seg
                    {
                        int Jst  = Seg0[j]; // look at remaining segments
                        int Jend = Seg1[j];
                        if (Jst < 0)
                            continue; // segment has already been used
                        if (PointEqual(X1, Y1, Xvals[Jst], Yvals[Jst])) // previous end = this start
                        {               
                            for (int  i = Jst + 1; i <= Jend; i++) // skip 1st (duplicate) point
                                Loop.Add(i);                 
                            Seg0[j] = -1;      // do not reuse this segment
                            X1 = Xvals[Jend];  // redefine new end point
                            Y1 = Yvals[Jend];
                            break;  // jump out to test if new end will match to X0,Y0
                        }
                        
                        if (PointEqual(X1, Y1, Xvals[Jend], Yvals[Jend])) // previous end = this end
                        {  // as above but in reverse             
                            for (int  i = Jend - 1; i >= Jst; i--)
                                Loop.Add(i);   
                            Seg0[j] = -1;
                            X1 = Xvals[Jst];
                            Y1 = Yvals[Jst];
                            break;
                        } // end if
                    } // end for
                    
                    if (j >= Nseg)
                    {
                        // points are not closed, so force an end             
                        X0 = X1;
                        Y0 = Y1;
                    } // end for
                } // end else
            } // end whilelooking
        }   //  if (Ist)                
    } // for Iseg
    
    LoopStart.Add(Loop.GetSize());
    OutputLoops(pFile, z, Xvals, Yvals, LoopStart, Loop);
    
    return;
}

// remove duplicate point sets coming from common edge between faces showing up twice
void SmSLC::RemoveDuplicates(int Nseg, 
                         SmTArray <double> &Xvals, SmTArray <double> &Yvals, 
                         SmTArray <int> &Seg0, SmTArray <int> &Seg1)
{
    int Iseg, Jseg; 
    for (Iseg = 0; Iseg < Nseg - 1; Iseg++)
    {            
        int ir0 = Seg0[Iseg];
        if (ir0 < 0)
            continue;
        ir0 = Seg1[Iseg] - ir0;
        for (Jseg = Iseg + 1; Jseg <Nseg; Jseg++)
        {
            int ir1 =  Seg0[Jseg];
            if (ir1 < 0)
                continue;
            ir1 = Seg1[Jseg] - ir1;
            if (ir1 != ir0)
                continue; // different number of points in each list
            int npt;
            int I0 = Seg0[Iseg];
            int J0 = Seg0[Jseg];
            int J1 = Seg1[Jseg];
            for (npt = 0; npt <= ir1; npt++)
            {
                double X0 = Xvals[I0];            
                double Y0 = Yvals[I0]; 
                double X1 = Xvals[J0];
                double Y1 = Yvals[J0];
                if (!PointEqual(X0, Y0, X1, Y1))
                {
                    double X2 = Xvals[J1]; // test reverse order duplicate
                    double Y2 = Yvals[J1];
                    if (!PointEqual(X0, Y0, X2, Y2))
                        break;
                }
                I0++;
                J0++;
                J1--;
            }
            if (npt > ir1) // we have duplicate lists
                Seg0[Jseg] = -1; // mark the 2nd
        }
    }
    return;
}


// Add up angles between points in the plane to +- 360
int  SmSLC::Clockwise(int NPts, 
                         SmTArray <double> &Xvals, SmTArray <double> &Yvals,
                         SmTArray <int> &Loop, int loop0, int loop1)
{
    NL_VECTOR V0;
    NL_VECTOR V1;
    NL_POINT   P0, P1, P2;
    NL_REAL ang;
    NL_REAL Tang = 0.0;
    
    if (NPts < 2)
        return (0);    
    double X0 = Xvals[Loop[loop0]];
    double Y0 = Yvals[Loop[loop0]];
    double X1 = Xvals[Loop[loop1]];
    double Y1 = Yvals[Loop[loop1]];    
    if (!PointEqual(X0, Y0, X1, Y1))
        return (0);// pointset not closed
    
    int j = loop0;
    int nsub = Loop[j];
    P0.x = Xvals[nsub];
    P0.y = Yvals[nsub];
    P0.z = 0.0;
    j++;
    
    nsub = Loop[j];
    P1.x = Xvals[nsub];
    P1.y = Yvals[nsub];
    P1.z = 0.0;
    N_VectorDiff (P1, P0, &V0)
        ;
    
    while (j < NPts - 1) 
    {
        j++;
        nsub = Loop[j];
        P2.x = Xvals[nsub];
        P2.y = Yvals[nsub];
        P2.z = 0.0;
        
        N_VectorDiff (P2, P1, &V1)
            ;
        N_VectorDirectedAngle(V0, V1, &ang);
        if (ang > 359.99)
            ang = 0.0;
        else if (ang > 180.0)
            ang = ang - 360.0;
        Tang += ang;
        
        P0.x = P1.x;
        P0.y = P1.y;
        P1.x = P2.x;
        P1.y = P2.y;
        V0.x = V1.x;
        V0.y = V1.y;
    }
    j = 1;
    nsub = Loop[j];
    P2.x = Xvals[nsub];
    P2.y = Yvals[nsub];
    P2.z = 0.0;
    
    N_VectorDiff (P2, P1, &V1)
        ;
    N_VectorDirectedAngle(V0, V1, &ang);
    if (ang > 359.99)
        ang = 0.0;
    else if (ang > 180.0)
        ang = ang - 360.0;
    Tang += ang;
    if (Tang  < -359.0)
        return (1); // cw (inner)
    if (Tang > 359.0)
        return (-1); // ccw  (outer)
    return (0);  // open set
}

// Add up angles between points in the plane to +- 360 to test whether
// X, Y is inside or outside boundary loop ( Loop(loop0, loop1)).
// Note: Just because one box is inside the other does not neccessarily
// mean that the loops are inside (consider a solid 'C' and an 'o')

int SmSLC::BoundaryInOut(double X, double Y,  
                         SmTArray <double> &Xvals, SmTArray <double> &Yvals,
                         SmTArray <int> &Loop, int loop0, int loop1)
{
    NL_VECTOR V0;
    NL_VECTOR V1;
    NL_POINT   P0, P1, P2;
    NL_REAL ang;
    NL_REAL Tang = 0.0;
    
    int NPts = loop1 - loop0 + 1;
    if (NPts < 2)
        return (0);
    double X0 = Xvals[Loop[loop0]];
    double Y0 = Yvals[Loop[loop0]];
    double X1 = Xvals[Loop[loop1]];
    double Y1 = Yvals[Loop[loop1]];    
    if (!PointEqual(X0, Y0, X1, Y1))
        return (0);// pointset not closed   
    
    P0.x = X;
    P0.y = Y;
    P1.z = 0.0;
    
    int j = loop0;
    int nsub = Loop[j];
    P1.x = Xvals[nsub];
    P1.y = Yvals[nsub];
    P1.z = 0.0;
    N_VectorDiff (P1, P0, &V0)
        ;
    
    while (++j < NPts) 
    {
        nsub = Loop[j];
        P2.x = Xvals[nsub];
        P2.y = Yvals[nsub];
        P2.z = 0.0;
        
        N_VectorDiff (P2, P0, &V1)
            ;
        N_VectorDirectedAngle(V0, V1, &ang);
        if (ang > 359.99)
            ang = 0.0;
        else if (ang > 180.0)
            ang = ang - 360.0;
        Tang += ang;
        
        P1.x = P2.x;
        P1.y = P2.y;
        V0.x = V1.x;
        V0.y = V1.y;
    }
    
    Tang =  fabs(Tang);
    if (Tang < 10)
        return (0);  // loop is inner
    if (Tang > 350.0)
        return (1);  // loop is outer
    return (0);      // loop is open 
}

// Reverse a loop
void SmSLC::RevLoop(SmTArray <int> &Loop, int loop0, int loop1)
{
    int Npts = loop1 - loop0 + 1;
    int ii = loop0;
    int jj = loop1;
    int imax = Npts/2;
    for (int i = 0; i < imax; i++)
    {
        int jx = Loop[ii];
        Loop[ii] = Loop[jj];
        Loop[jj] = jx;
        jj--;
        ii++;
    }
    return;
}

// If 3 (or more) points are colinear, remove the middle one(s)
void SmSLC::RemoveColinear(ULONG nPts, SmPoint3d *pPts)

{
    NL_POINT  A, B, C;
    if (nPts < 3)
        return;
    
    A.z = B.z = C.z = 0.0;
    A.x = pPts[0].x;
    A.y = pPts[0].y;
    ULONG j = 1;
    B.x = pPts[j].x;
    B.y = pPts[j].y;
    ULONG k = 2;
    while (k < nPts) 
    {
        C.x = pPts[k].x;
        C.y = pPts[k].y;
        if (N_PtsAreColinear(A, B, C, m_dColinearTol))
        {
            // middle of 3 colinear points
            pPts[j].x = 1.0e23; // high value use as flag to bypass           
        }
        else 
        {   
            A.x = B.x;
            A.y = B.y;
        }
        B.x = C.x;
        B.y = C.y;
        j = k;
        k++;
    }
    
    return;
}


// Return the next Z value 
// Normally we would just increment Z by zCut
// However also pay attention to z breaks(in zlist) and end
int SmSLC::NextZ(double *Z, SmTArray <double> &zlist)

{   
    int i, imax;
    imax = zlist.GetSize();
    double Zp = zlist[0];
    double Zn;
    if (*Z <(Zp - m_zTol))
    {
        *Z = Zp;
        return (1); 
    } // first time in
    for (i = 1; i < imax; i++)
    {
        Zn = zlist[i];
        if (*Z >= Zn)
        {
            Zp = Zn;
        }
        else
        {
            double Zinc =(Zn - Zp)/3.0; // for narrow section, take 1/3
            if (Zinc > m_zCut)
                Zinc = m_zCut; // unless GT zCut
            *Z = *Z + Zinc;
            if (*Z >(Zn - m_zTol))
                *Z = Zn; // near end so stretch to end         
            return (1);
        }
    }
    return (0);
}
    
// Determine loop nesting and output loops in order (outer, in, in , outer,in) etc.
void SmSLC::OutputLoops(FILE *pFile, double z,
                         SmTArray <double> &Xvals, SmTArray <double> &Yvals, 
                         SmTArray <int> &LoopStart, SmTArray <int> &Loop)
{
    int i, k1, l, ii, jj, mm, nn;
    SmTArray < double> Xmin;
    SmTArray < double> Xmax;    
    SmTArray < double> Ymin;
    SmTArray < double> Ymax;

    SM_REF1(z);
    
    int numloops= LoopStart.GetSize() - 1;
    
    if (pFile) SM_FPRINTF(pFile, _T("#Number of boundaries = %d\n"), numloops);
    if (numloops < 1)
        return;
    SmTArray <int> Outer;     
    for (i = 0; i < numloops; i++)
        Outer.Add(1); // initially set all as outer boundaries
    
    if (numloops > 1) // Build min-max boxes
    {
        for (i = 0; i < numloops; i++)
        {
            int k = LoopStart[i];
            l = LoopStart[i + 1];
            // SM_FPRINTF(pFile, _T("Loop %d  from %d to %d \n", i, k, l));
            for (int j = k; j < l; j++)
            {
                int kk = Loop[j];
                double X = Xvals[kk];
                double Y = Yvals[kk];
                // SM_FPRINTF(pFile, _T("X= %lf Y= %lf %d\n", X, Y, kk));
                if (j == k) // minmax box init off 1st case
                {
                    Xmin.Add(X);
                    Xmax.Add(X);
                    Ymin.Add(Y);
                    Ymax.Add(Y);
                }
                else 
                {
                    if (X < Xmin[i])
                        Xmin[i]= X;
                    else if (X > Xmax[i])
                        Xmax[i]= X;
                    if (Y < Ymin[i])
                        Ymin[i]= Y;
                    else if (Y > Ymax[i])
                        Ymax[i]= Y;
                }
            }
            // SM_FPRINTF(pFile, _T("X range %lf,%lf  Yrange %lf %lf \n"), Xmin[i], Xmax[i], Ymin[i], Ymax[i]);
        }
        // Built min-max boxes
        
        // Order based on box (greater of) x or y range....largest to smallest
        // since we know overall outer must be outer 
        // ( but innerest may be inner or outer, 
        //   depending on how many nested loops there are )
        // brute force sort
        SmTArray <int> Order;
        SmTArray <double> Range;
        double Tx = 0.0;
        double Ty = 0.0;
        for (i = 0; i < numloops; i++)
        {
            Tx +=(Xmax[i] - Xmin[i]);
            Ty +=(Ymax[i] - Ymin[i]);
        }
        for (i = 0; i < numloops; i++)
        {
            double score = Tx > Ty ?(Xmax[i] - Xmin[i]) :(Ymax[i] - Ymin[i]);
            Range.Add(score);
            Order.Add(i);
            // SM_FPRINTF(pFile, _T(" %d score = %lf,"), i, score);
        }
        // SM_FPRINTF(pFile, _T("\n"));
        int j = 0;
        int k = 0;
        for (jj = 0; jj < numloops; jj++)
        {
            // just a count for number of times
            double dMax = -1.0e15;
            for (i = 0; i < numloops; i++)
            {
                // find the next max 
                if (Range[i] > dMax)
                {
                    j = i; 
                    dMax = Range[i];
                }
            }
            Order[k] = j;
            Range[j] = -1.0e16;
            k++;
        }
        //        for (k=0; k<numloops; k++){
        //            SM_FPRINTF(pFile, _T(" %d = %d,"), k, Order[k]);
        //        }
        // SM_FPRINTF(pFile, _T("\n"));       
        
        // The following depends on the order from largest to smallest
        // to resolve all multiple-nesting issues
        for (ii = 0; ii < numloops - 1; ii++) // nothing left to compare last one with
        {
            i = Order[ii];  // largest range to smallest
            if (Outer[i] < 1)
                continue; // dont retest already inner boundaries
            double Xm0 = Xmin[i];
            double XM0 = Xmax[i];
            double Ym0 = Ymin[i];
            double YM0 = Ymax[i];
            for (jj = ii + 1; jj < numloops; jj++)
            {
                j = Order[jj];
                double Xm1 = Xmin[j];
                double XM1 = Xmax[j];
                double Ym1 = Ymin[j];
                double YM1 = Ymax[j];
                int olap = BoxOverlap(Xm0, XM0, Ym0, YM0, Xm1, XM1, Ym1, YM1);
                if (olap == 1 && OuterConfirm(j, i, Xvals, Yvals, LoopStart, Loop))
                {
                    Outer[i] = -j; // 0 (i) is an inner to j
                    Outer[j]++;
                }
                if (olap == 2 && OuterConfirm(i, j, Xvals, Yvals, LoopStart, Loop))
                {
                    Outer[j] = -i; // 1 (j) is an inner to i
                    Outer[i]++;
                }
                
                
                // SM_FPRINTF(pFile, _T("i=%d. j=%d olap = %d, Outer[i] = %d Outer[j]=%d\n"),
                //       i, j, olap, Outer[i], Outer[j]);
            }
        }
    }
    
    // write out final loops in order
    // prefered order = outer1, (all its inners), outer2, (all its inners) etc
    
    // sort out rotation directon (outer = ccw, inner = cw)
    for (nn = 0; nn < numloops; nn++)
    {
        int loop0 = LoopStart[nn];
        int loop1 = LoopStart[nn + 1] - 1;
        int Cw = Clockwise(loop1 - loop0 +1, Xvals, Yvals, Loop, loop0, loop1);               
        if ((Cw == 1 && Outer[nn] > 0) ||(Cw == -1 && Outer[nn] <= 0))
        {           
            RevLoop(Loop, loop0, loop1);
            Cw = -Cw;
        }              
    }
    
#ifdef SM_DEBUG_CODE
    SmBoolean bDrawPoints = (pFile) ? FALSE : TRUE;
#endif 

    // get next outer
    for (nn = 0; nn < numloops; nn++)
    {
        // find its matching inners
        if (Outer[nn] > 0 && Outer[nn] < 99999)
        {
            int loop0 = LoopStart[nn];
            int loop1 = LoopStart[nn + 1] - 1;                
            if (pFile) SM_FPRINTF(pFile, _T("#Loop of %d points  (cw=%d)\n"), loop1 - loop0 + 1, -1);                
            for (int k = loop0; k <= loop1; k++)
            {
                int kk = Loop[k];
                if (pFile) SM_FPRINTF(pFile, _T("%lf %lf\n"), Xvals[kk], Yvals[kk]); 
#ifdef SM_DEBUG_CODE             
            if (bDrawPoints){
                SmPoint3d P;
                P.x = Xvals[kk];
                P.y = Yvals[kk];                  
                P.z = z;
               P.Draw();
            } 
#endif                
            }

            if (pFile) SM_FPRINTF(pFile, _T("C\n"));
            for (mm = 0; mm < numloops; mm++)
            {
                if (-Outer[mm] == nn)
                {
                    int loop2 = LoopStart[mm];
                    int loop3 = LoopStart[mm + 1] - 1;                
                    if (pFile) SM_FPRINTF(pFile, _T("Loop of %d points  (cw=%d)\n"), loop3 - loop2 + 1, 1);                
                    for (int k = loop2; k <= loop3; k++)
                    {
                        k1 = Loop[k];
                        if (pFile) SM_FPRINTF(pFile, _T("%lf %lf\n"), Xvals[k1], Yvals[k1]); 
                        
#ifdef SM_DEBUG_CODE                                            
                        if (bDrawPoints){
                         SmPoint3d P;
                         P.x = Xvals[k1];
                         P.y = Yvals[k1];                  
                         P.z = z;
                         P.Draw(); 
                        }
#endif
                        
                    }
                    Outer[mm] = 999999;
                }
            }
        }
    }
    
    return;
}


// Confirm that loop i is an outer loop to j
int SmSLC::OuterConfirm(int i, int j, SmTArray <double> &Xvals, SmTArray <double> &Yvals, 
                         SmTArray <int> &LoopStart, SmTArray <int> &Loop)

{   
    // find a point on loop j (X0,Y0)
    // confirm that all loop points in loop i surround j
    double X0 = Xvals[Loop[LoopStart[j]]]; 
    double Y0 = Yvals[Loop[LoopStart[j]]];
    int loop0 = LoopStart[i];
    int loop1 = LoopStart[i + 1] - 1;
    
    // test whether X0,Y0 is inside loop (1 if so, else 0)
    int InOut = BoundaryInOut(X0, Y0, Xvals, Yvals, Loop, loop0, loop1);
    
    return (InOut);
}

int SmSLC::BoxOverlap(double Xm0, double XM0, double Ym0, double YM0, 
                double Xm1, double XM1, double Ym1, double YM1)
{
    // return 1 if 0 inside 1
    // return 2 if 1 inside 0
    // return 3 if overlap 0,1
    // return 4 if separate 0,1    
    // completely separate
    
    if (Xm0 >= XM1)
        return (4);
    if (XM0 <= Xm1)
        return (4);
    if (Ym0 >= YM1)
        return (4);
    if (YM0 <= Ym1)
        return (4);
    if (Xm0 > Xm1 && XM0 < XM1 &&
        Ym0 > Ym1 && YM0 < YM1)
        return (1);
    if (Xm1 > Xm0 && XM1 < XM0 &&
        Ym1 > Ym0 && YM1 < YM0)
        return (2);
    return (3);
}

