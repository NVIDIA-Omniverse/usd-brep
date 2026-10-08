// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* NL_Math.c : NLib Math Function Definitions                                */
/*****************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

/* global state NLIBCHKBYPASS can speed up computation, by bypassing numeric range (log) checks */
NL_INDEX NLIBCHKBYPASS = 0;

/* file local defintions */

NL_REAL ST_pythag( NL_REAL, NL_REAL );
NL_FLAG ST_EvalIso( NL_REAL, NL_POINT * );

/* thread local storage for ST_EvalIso communication */
NL_PRIVATE_TLS NL_INDEX ST_EvalIsoUvSwitch;
NL_PRIVATE_TLS NL_REAL ST_EvalIsoGloUv;
NL_PRIVATE_TLS  NL_FLAG   (*ST_EvalIsoGloS)(NL_REAL, NL_REAL, NL_POINT *);
#define ST_SIGN(a,b)     ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define ST_SHFT(a,b,c,d) (a)=(b); (b)=(c); (c)=(d);


/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the combination of two control
     points,  i.e. it  computes  Cw = alpha*Aw + beta*Bw. A  typical 
     calling example is:

       NL_REAL    alpha, beta;
       NL_CPOINT  Aw, Bw, Cw;
       ...
       (get alpha, beta, Aw and Bw);
       ...
       N_Combine2CPts(alpha,Aw,beta,Bw,&Cw);


   ACCESS:
   
     alpha , input  ,  First parameter
     Aw    , input  ,  First control point
     beta  , input  ,  Second parameter
     Bw    , input  ,  Second control point
     Cw    , output ,  Combination of A and B



   RETURN CODES:

     None

   ***********************************************************************/
NL_VOID N_Combine2CPts
 ( NL_REAL    alpha,  // in :  alpha of Cw = alpha * Aw + beta * Bw
   NL_CPOINT  Aw,     // in :  Aw    of Cw = alpha * Aw + beta * Bw
   NL_REAL    beta,   // in :  beta  of Cw = alpha * Aw + beta * Bw
   NL_CPOINT  Bw,     // in :  Bw    of Cw = alpha * Aw + beta * Bw
   NL_CPOINT *Cw )    // out:  Cw    of Cw = alpha * Aw + beta * Bw
{
    Cw->x = alpha * Aw.x + beta * Bw.x;
    Cw->y = alpha * Aw.y + beta * Bw.y;

    if( Aw.z NEQ NL_NOZ AND Bw.z NEQ NL_NOZ )
        Cw->z = alpha * Aw.z + beta * Bw.z;

    else if( Aw.z NEQ NL_NOZ )
        Cw->z = alpha *Aw.z;

    else if( Bw.z NEQ NL_NOZ )
        Cw->z = beta *Bw.z;

    else
        Cw->z = NL_NOZ;

    if( Aw.w NEQ NL_NOW AND Bw.w NEQ NL_NOW )
        Cw->w = alpha * Aw.w + beta * Bw.w;
    else
        Cw->w = NL_NOW;
} /* end N_Combine2CPts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the vector  combination of two 
     control  points,  i.e. it  computes  Cw  =  alpha^{x,y,z}*Aw  + 
     beta^{x,y,z}*Bw, where alpha and beta are 3-D vectors. Only the 
     wx, wy and wz components change, the weights remain intact. The 
     weight  of the  output point is  the same as  that of the first 
     control point. A typical calling example is:
     
       NL_VECTOR  alpha, beta;
       NL_CPOINT  Aw, Bw, Cw;
       ...
       (get alpha, beta, Aw and Bw);
       ...
       N_VectorCombineCPts(alpha,Aw,beta,Bw,&Cw);


   ACCESS:
   
     alpha , input  ,  Vector parameter
     Aw    , input  ,  First control point
     beta  , input  ,  Vector parameter
     Bw    , input  ,  Second control point
     Cw    , output ,  Combination of A and B



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorCombineCPts( NL_VECTOR alpha, NL_CPOINT Aw, NL_VECTOR beta, NL_CPOINT Bw, NL_CPOINT *Cw )
{

    Cw->x = alpha.x * Aw.x + beta.x * Bw.x;
    Cw->y = alpha.y * Aw.y + beta.y * Bw.y;

    if( Aw.z NEQ NL_NOZ AND Bw.z NEQ NL_NOZ )
        Cw->z = alpha.z * Aw.z + beta.z * Bw.z;

    else if( Aw.z NEQ NL_NOZ )
        Cw->z = alpha.z *Aw.z;

    else if( Bw.z NEQ NL_NOZ )
        Cw->z = beta.z *Bw.z;

    else
        Cw->z = NL_NOZ;

    if( Aw.w NEQ NL_NOW AND Bw.w NEQ NL_NOW )
        Cw->w = Aw.w;
    else
        Cw->w = NL_NOW;
} /* end N_VectorCombineCPts */

/*******************************************************************//**


   DESCRIPTION:

     This  arithmetic routine computes the combination of two points, 
     i.e. it computes C = alpha*A + beta*B. A typical calling example
     is:

       NL_REAL   alpha, beta;
       NL_POINT  A, B, C;
       ...
       (get alpha, beta, A and B);
       ...
       N_Combine2Pts(alpha,A,beta,B,&C);


   ACCESS:
   
     alpha , input  ,  First parameter
     A     , input  ,  First point
     beta  , input  ,  Second parameter
     B     , input  ,  Second point
     C     , output ,  Combination of A and B



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Combine2Pts( NL_REAL alpha, NL_POINT A, NL_REAL beta, NL_POINT B, NL_POINT *C )
{

    C->x = alpha * A.x + beta * B.x;
    C->y = alpha * A.y + beta * B.y;
    C->z = alpha * A.z + beta * B.z;
} /* end N_Combine2Pts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine  computes the vector combination of two 
     points, i.e. it computes  C = alpha^{x,y,z}*A + beta^{x,y,z}*B,
     where alpha and beta are 3-D vectors. A typical calling example
     is:
 
       NL_VECTOR  alpha, beta;
       NL_POINT   A, B, C;
       ...
       (get alpha, beta, A and B);
       ...
       N_VectorCombinePts(alpha,A,beta,B,&C);


   ACCESS:
   
     alpha , input  ,  Vector parameter
     A     , input  ,  First point
     beta  , input  ,  Vector parameter
     B     , input  ,  Second point
     C     , output ,  Combination of A and B



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorCombinePts( NL_VECTOR alpha, NL_POINT A, NL_VECTOR beta, NL_POINT B, NL_POINT *C )
{

    C->x = alpha.x * A.x + beta.x * B.x;
    C->y = alpha.y * A.y + beta.y * B.y;
    C->z = alpha.z * A.z + beta.z * B.z;
} /* end N_VectorCombinePts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine translates the combination of two points, 
     i.e.  it  computes  D = C + alpha*A + beta*B. A  typical  calling 
     example is:

       NL_POINT  A, B, C, D;
       NL_REAL   alpha, beta;
       ...
       (get alpha, beta, A, B and C);
       ...
       N_TranslateSum2Pts(C,alpha,A,beta,B,&D);


   ACCESS:
   
     C     , input  ,  Translation point
     alpha , input  ,  First parameter
     A     , input  ,  First point
     beta  , input  ,  Second parameter
     B     , input  ,  Second point
     D     , output ,  Translate of combination of A and B



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_TranslateSum2Pts( NL_POINT C, NL_REAL alpha, NL_POINT A, NL_REAL beta, NL_POINT B, NL_POINT *D )
{

    D->x = C.x + alpha * A.x + beta * B.x;
    D->y = C.y + alpha * A.y + beta * B.y;
    D->z = C.z + alpha * A.z + beta * B.z;
} /* end N_TranslateSum2Pts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine translates the combination of two control 
     points, i.e. it  computes Dw = Cw + alpha*Aw + beta*Bw. A typical
     calling example is:

       NL_CPOINT  Aw, Bw, Cw, Dw;
       NL_REAL    alpha, beta;
       ...
       (get alpha, beta, Aw, Bw and Cw);
       ...
       N_TranslateSum2CPts(Cw,alpha,Aw,beta,Bw,&Dw);

     IT IS ASSUMED THAT  Aw, Bw AND Cw ARE  COMPATIBLE, I.E.  HAVE THE
     SAME DIMENSION AND RATIONALITY.


   ACCESS:
   
     Cw    , input  ,  Translation control point
     alpha , input  ,  First parameter
     Aw    , input  ,  First control point
     beta  , input  ,  Second parameter
     Bw    , input  ,  Second control point
     Dw    , output ,  Translate of combination of Aw and Bw



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_TranslateSum2CPts
 (NL_CPOINT Cw,     /* in : Cw    of Dw = Cw + alpha*Aw + beta*Bw */
  NL_REAL   alpha,  /* in : alpha of Dw = Cw + alpha*Aw + beta*Bw */
  NL_CPOINT Aw,     /* in : Aw    of Dw = Cw + alpha*Aw + beta*Bw */
  NL_REAL   beta,   /* in : beta  of Dw = Cw + alpha*Aw + beta*Bw */
  NL_CPOINT Bw,     /* in : Bw    of Dw = Cw + alpha*Aw + beta*Bw */
  NL_CPOINT *Dw )   /* out: Dw    of Dw = Cw + alpha*Aw + beta*Bw */
{

    Dw->x = Cw.x + alpha * Aw.x + beta * Bw.x;
    Dw->y = Cw.y + alpha * Aw.y + beta * Bw.y;

    if( Aw.z NEQ NL_NOZ AND Bw.z NEQ NL_NOZ AND Cw.z NEQ NL_NOZ )
    {
        Dw->z = Cw.z + alpha * Aw.z + beta * Bw.z;
    }
    else
    {
        Dw->z = NL_NOZ;
    }

    if( Aw.w NEQ NL_NOW AND Bw.w NEQ NL_NOW AND Cw.w NEQ NL_NOW )
    {
        Dw->w = Cw.w + alpha * Aw.w + beta * Bw.w;
    }
    else
    {
        Dw->w = NL_NOW;
    }
} /* end N_TranslateSum2CPts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the combination of four points, 
     i.e.  it  computes E = alf*A + bet*B + gam*C + del*D. A  typical  
     calling example is:

       NL_REAL   alf, bet, gam, del;
       NL_POINT  A, B, C, D, E;
       ...
       (get alf, bet, gam, del, A, B, C and D);
       ...
       N_Combine4Pts(alf,A,bet,B,gam,C,del,D,&E);


   ACCESS:
   
     alf , input  ,  First parameter
     A   , input  ,  First point
     bet , input  ,  Second parameter
     B   , input  ,  Second point
     gam , input  ,  Third parameter
     C   , input  ,  Third point
     del , input  ,  Fourth parameter
     D   , input  ,  Fourth point
     E   , output ,  Combination of A, B, C and D



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Combine4Pts( NL_REAL alf, NL_POINT A, NL_REAL bet, NL_POINT B, NL_REAL gam, NL_POINT C, NL_REAL del, NL_POINT D, NL_POINT *E )
{

    E->x = alf * A.x + bet * B.x + gam * C.x + del * D.x;
    E->y = alf * A.y + bet * B.y + gam * C.y + del * D.y;
    E->z = alf * A.z + bet * B.z + gam * C.z + del * D.z;
} /* end N_Combine4Pts */


/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the combination of four control 
     points, i.e. it computes Ew = alf*Aw + bet*Bw + gam*Cw + del*Dw. 
     A typical calling example is:

       NL_REAL    alf, bet, gam, del;
       NL_CPOINT  Aw, Bw, Cw, Dw, Ew;
       ...
       (get alf, bet, gam, del, Aw, Bw, Cw and Dw);
       ...
       N_Combine4CPts(alf,Aw,bet,Bw,gam,Cw,del,Dw,&Ew);

     IT IS ASSUMED THAT  Aw, Bw, Cw AND Dw ARE  COMPATIBLE, I.E. THEY 
     HAVE THE SAME DIMENSION AND RATIONALITY.


   ACCESS:
   
     alf , input  ,  First parameter
     Aw  , input  ,  First control point
     bet , input  ,  Second parameter
     Bw  , input  ,  Second control point
     gam , input  ,  Third parameter
     Cw  , input  ,  Third control point
     del , input  ,  Fourth parameter
     Dw  , input  ,  Fourth control point
     Ew  , output ,  Combination of Aw, Bw, Cw and Dw


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Combine4CPts( NL_REAL alf, NL_CPOINT Aw, NL_REAL bet, NL_CPOINT Bw, NL_REAL gam, NL_CPOINT Cw, NL_REAL del, NL_CPOINT Dw, NL_CPOINT *Ew )
{

    Ew->x = alf * Aw.x + bet * Bw.x + gam * Cw.x + del * Dw.x;
    Ew->y = alf * Aw.y + bet * Bw.y + gam * Cw.y + del * Dw.y;

    if( Aw.z NEQ NL_NOZ AND Bw.z NEQ NL_NOZ AND Cw.z NEQ NL_NOZ AND Dw.z NEQ NL_NOZ )
    {
        Ew->z = alf * Aw.z + bet * Bw.z + gam * Cw.z + del * Dw.z;
    }
    else
    {
        Ew->z = NL_NOZ;
    }

    if( Aw.w NEQ NL_NOW AND Bw.w NEQ NL_NOW AND Cw.w NEQ NL_NOW AND Dw.w NEQ NL_NOW )
    {
        Ew->w = alf * Aw.w + bet * Bw.w + gam * Cw.w + del * Dw.w;
    }
    else
    {
        Ew->w = NL_NOW;
    }
} /* end N_Combine4CPts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine initializes a control point, i.e. it sets
     Bw = Aw. A typical calling example is:

       NL_CPOINT  Aw, Bw;
       ...
       (get Aw);
       ....
       N_CopyCPt(Aw,&Bw);


   ACCESS:
   
     Aw  , input  ,  Control point
     Bw  , output ,  Initialized control point



   RETURN CODES:

     None

   ***********************************************************************/
NL_VOID N_CopyCPt
 (NL_CPOINT  Aw,   /* in : From CPt */
  NL_CPOINT *Bw )  /* out: to CPt */
{
    Bw->x = Aw.x;
    Bw->y = Aw.y;
    Bw->z = Aw.z;
    Bw->w = Aw.w;
} /* end N_CopyCPt */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine initializes a point, i.e. it sets B = A.
     A typical calling example is:

       NL_POINT  A, B;
       ...
       (get A);
       ...
       N_CopyPt(A,&B);
 

   ACCESS:
   
     A  , input  ,  Point
     B  , output ,  Initialized point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CopyPt( NL_POINT A, NL_POINT *B )
{

    B->x = A.x;
    B->y = A.y;
    B->z = A.z;
} /* end N_CopyPt */


/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes a Euclidean point from the 
     weighted (homogeneous) point. A typical calling example is:

       NL_CPOINT  Pw;
       NL_POINT   P;
       ...
       (get Pw);
       ...
       N_CPtToPtEuclid(Pw,&P);


   ACCESS:
   
     Pw  , input  ,  Homogeneous point
     P   , output ,  Euclidean point



   RETURN CODES:

     None

   ***********************************************************************/
NL_VOID N_CPtToPtEuclid(NL_CPOINT Pw, NL_POINT* P)
{
  if (Pw.w NEQ NL_NOW)
    {
      NL_REAL wReciprocal = 1.0 / Pw.w;
      P->x = Pw.x * wReciprocal; 
      P->y = Pw.y * wReciprocal; 
  
      if (Pw.z NEQ NL_NOZ)
          P->z = Pw.z * wReciprocal; 
      else
          P->z = 0.0;
    }
  else
    {
      P->x = Pw.x;
      P->y = Pw.y;
  
      if (Pw.z NEQ NL_NOZ)
          P->z = Pw.z;
      else
          P->z = 0.0;
    }
} /* end N_CPtToPtEuclid */



/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine maps a control point to Euclidean space
     and  returns the  Euclidean point  along with the corresponding 
     weight. A typical calling example is:

       NL_CPOINT  Pw;
       NL_POINT   P;
       NL_REAL    w;
       ...
       (get Pw);
       ...
       N_CPtToPtAndW(Pw,&P,&w);


   ACCESS:
   
     Pw , input  ,  Homogeneous point
     P  , output ,  Euclidean point
     w  , output ,  Weight of Pw 



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtToPtAndW( NL_CPOINT Pw, NL_POINT *P, NL_REAL *w )
{

    N_CPtToPtEuclid( Pw, P );
    N_CPtGetW( Pw, w );
} /* end N_CPtToPtAndW */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes a weighted control point given
     a Euclidean point and a weight. A typical calling example is:

       NL_POINT   P;
       NL_REAL    w;
       NL_CPOINT  Pw;
       ...
       (get P and w);
       ...
       N_Weight(P,w,&Pw);


   ACCESS:
   
     P  , input  ,  Euclidean point
     w  , input  ,  Weight
     Pw , output ,  Homogeneous point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Weight
  ( NL_POINT   P,     /* in : euclidean point                     */ 
    NL_REAL    w,     /* in : weight or NL_NOW                    */ 
    NL_CPOINT *Pw )   /* out: euclidean/homogeneous control point */
{
    if( w EQ NL_NOW )
    {
        Pw->x = P.x;
        Pw->y = P.y;
        Pw->z = P.z;
        Pw->w = NL_NOW;
    }
    else
    {
        Pw->x = w * P.x;
        Pw->y = w * P.y;
        Pw->z = w * P.z;
        Pw->w = w;
    }
} /* end N_Weight */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine updates a control point by adding a term
     to it, i.e. it computes Bw = Bw + alpha*Aw. This routine is used
     to compute the  sum of the  blend of a  set of control points. A
     typical calling example is:

       NL_REAL    alpha;
       NL_CPOINT  Aw; 
       NL_CPOINT  Bw;
       ...
       (get alpha and Aw);
       ...
       N_VectorBlendCPt(alpha,Aw,&Bw);


   ACCESS:
   
     alpha , input  ,  Parameter
     Aw    , input  ,  Control point
     Bw    , output ,  Updated control point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorBlendCPt
 (NL_REAL    alpha,     /* in : alpha of Bw = Bw + alpha*Aw */
  const NL_CPOINT&  Aw, /* in : Aw    of Bw = Bw + alpha*Aw */
  NL_CPOINT *Bw )       /* out: Bw    of Bw = Bw + alpha*Aw */
{
    Bw->x = Bw->x + alpha * Aw.x;
    Bw->y = Bw->y + alpha * Aw.y;

    if( Aw.z NEQ NL_NOZ )
        Bw->z = Bw->z + alpha * Aw.z;
    else
        Bw->z = NL_NOZ;

    if( Aw.w NEQ NL_NOW )
        Bw->w = Bw->w + alpha * Aw.w;
    else
        Bw->w = NL_NOW;
} /* end N_VectorBlendCPt */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine scales a control point, i.e. it computes 
     Bw = alpha*Aw. A typical calling example is:

       NL_REAL    alpha;
       NL_CPOINT  Aw, Bw;
       ...
       (get alpha and Aw);
       ...
       N_ScaleCPt(alpha,Aw,&Bw);


   ACCESS:
   
     alpha , input  ,  Parameter
     Aw    , input  ,  Control point
     Bw    , output ,  Scaled control point


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ScaleCPt
 (NL_REAL    alpha,  /* in : alpha of Bw = alpha*Aw */
  NL_CPOINT  Aw,     /* in : Aw    of Bw = alpha*Aw */
  NL_CPOINT *Bw )    /* out: Bw    of Bw = alpha*Aw */
{

    Bw->x = alpha * Aw.x;
    Bw->y = alpha * Aw.y;

    if( Aw.z NEQ NL_NOZ )
        Bw->z = alpha * Aw.z;
    else
        Bw->z = NL_NOZ;

    if( Aw.w NEQ NL_NOW )
        Bw->w = alpha * Aw.w;
    else
        Bw->w = NL_NOW;
} /* end N_ScaleCPt */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine scales the first three coordinates of a 
     control point. A typical calling example is:

       NL_REAL    alpha;
       NL_CPOINT  Aw, Bw;
       ...
       (get alpha and Aw);
       ...
       N_ScaleCPtXYZ(alpha,Aw,&Bw);


   ACCESS:
   
     alpha , input  ,  Parameter
     Aw    , input  ,  Control point
     Bw    , output ,  Scaled control point


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ScaleCPtXYZ( NL_REAL alpha, NL_CPOINT Aw, NL_CPOINT *Bw )
{

    Bw->x = alpha * Aw.x;
    Bw->y = alpha * Aw.y;

    if( Aw.z NEQ NL_NOZ )
        Bw->z = alpha * Aw.z;
    else
        Bw->z = NL_NOZ;
    Bw->w = Aw.w;
} /* end N_ScaleCPtXYZ */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine updates a point by adding a term to it, 
     i.e.  it  computes  B = B + alpha*A. This  routine  is used  to 
     compute the  sum of  the blend  of a  set of  points. A typical 
     calling example is:

       NL_REAL   alpha; 
       NL_POINT  A, B;
       ...
       (get alpha and A);
       ...
       N_VectorBlendPt(alpha,A,&B);


   ACCESS:
   
     alpha , input  ,  Parameter
     A     , input  ,  Point
     B     , output ,  Updated  point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorBlendPt( NL_REAL alpha, NL_POINT A, NL_POINT *B )
{

    B->x += alpha * A.x;
    B->y += alpha * A.y;
    B->z += alpha * A.z;
} /* end N_VectorBlendPt */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine scales a point, i.e. it computes 
     B = alpha*A. A typical calling example is:

       NL_REAL   alpha;
       NL_POINT  A, B;
       ...
       (get alpha and A);
       ...
       N_ScalePt(alpha,A,&B);


   ACCESS:
   
     alpha , input  ,  Parameter
     A     , input  ,  Point
     B     , output ,  Scaled point


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ScalePt( NL_REAL alpha, NL_POINT A, NL_POINT *B )
{

    B->x = alpha * A.x;
    B->y = alpha * A.y;
    B->z = alpha * A.z;
} /* end N_ScalePt */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine transforms a control point given a general
     4x4 transformation  matrix. The matrix multiplication is performed
     from the left, i.e.

                     |a00 ... a02 tx| |Pw.x|   |Qw.x|
                     |            ty| |Pw.y| = |Qw.y|
                     |a20 ... a22 tz| |Pw.z|   |Qw.z|
                     |px  py  pz  1 | |Pw.w|   |Qw.w|

     The 4x4 matrix is the  traditional general  transformation matrix, 
     i.e. the upper left 3x3 contains rotation and scaling information,
     the  last  column  contains  transformation  whereas the  last row 
     contains projection parameters. A typical calling example is:

       NL_CPOINT   Pw, Qw;
       NL_RMATRIX  rma;
       ...
       (get Pw and define rma);
       ...
       N_TransformCPt(Pw,&rma,&Qw);


   ACCESS:
   
     Pw  , input  ,  Homogeneous point
     rma , input  ,  4x4 real matrix
     Qw  , output ,  Image of Pw


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_TransformCPt( NL_CPOINT Pw, NL_RMATRIX *rma, NL_CPOINT *Qw )
{

    NL_REAL ** RM, f;

    NL_CPOINT Rw;

    /* Get local notation */

    N_GetRealMatrixPtr( rma, &RM );

    /* Locate weighted control point in Euclidean space */

    if( Pw.w NEQ NL_NOW )
    {
        Pw.x /= Pw.w;
        Pw.y /= Pw.w;

        if( Pw.z NEQ NL_NOZ )
            Pw.z /= Pw.w;
    }

    /* Transform control point */

    if( Pw.z NEQ NL_NOZ )
    {
        f = 1.0 / (Pw.x * RM[3][0] + Pw.y * RM[3][1] + Pw.z * RM[3][2] + RM[3][3]);

        Rw.x = f * (Pw.x * RM[0][0] + Pw.y * RM[0][1] + Pw.z * RM[0][2] + RM[0][3]);
        Rw.y = f * (Pw.x * RM[1][0] + Pw.y * RM[1][1] + Pw.z * RM[1][2] + RM[1][3]);
        Rw.z = f * (Pw.x * RM[2][0] + Pw.y * RM[2][1] + Pw.z * RM[2][2] + RM[2][3]);
    }
    else
    {
        f = 1.0 / (Pw.x * RM[3][0] + Pw.y * RM[3][1] + RM[3][3]);

        Rw.x = f * (Pw.x * RM[0][0] + Pw.y * RM[0][1] + RM[0][3]);
        Rw.y = f * (Pw.x * RM[1][0] + Pw.y * RM[1][1] + RM[1][3]);
        Rw.z = NL_NOZ;
    }

    /* Weight transformed control point if neccesary */

    if( Pw.w NEQ NL_NOW )
    {
        Rw.x *= Pw.w;
        Rw.y *= Pw.w;

        if( Pw.z NEQ NL_NOZ )
            Rw.z *= Pw.w;
        Rw.w = Pw.w;
    }
    else
    {
        Rw.w = NL_NOW;
    }

    /* Get output */

    Qw->x = Rw.x;
    Qw->y = Rw.y;
    Qw->z = Rw.z;
    Qw->w = Rw.w;
} /* end N_TransformCPt */

/*******************************************************************//**


   DESCRIPTION:

     This  arithmetic routine  transforms a point given  a general  4x4 
     transformation matrix. The matrix multiplication is performed from 
     the left, i.e.

                     |a00 ... a02 tx| |P.x|   |Q.x|
                     |            ty| |P.y| = |Q.y|
                     |a20 ... a22 tz| |P.z|   |Q.z|
                     |px  py  pz  1 | | 1 |   | 1 |

     The 4x4 matrix is the  traditional general  transformation matrix, 
     i.e. the upper left 3x3 contains rotation and scaling information,
     the  last  column  contains  transformation  whereas the  last row 
     contains projection parameters. A typical calling example is:

       NL_POINT    P, Q;
       NL_RMATRIX  rma;
       ...
       (get P and define rma);
       ...
       N_TransformPt(P,&rma,&Q);


   ACCESS:
   
     P   , input  ,  Point
     rma , input  ,  4x4 real matrix
     Q   , output ,  Image of P


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_TransformPt( NL_POINT P, NL_RMATRIX *rma, NL_POINT *Q )
{

    NL_REAL ** RM, f;

    /* Get local notation */

    N_GetRealMatrixPtr( rma, &RM );

    /* Transform point */

    f = 1.0 / (P.x * RM[3][0] + P.y * RM[3][1] + P.z * RM[3][2] + RM[3][3]);

    Q->x = f * (P.x * RM[0][0] + P.y * RM[0][1] + P.z * RM[0][2] + RM[0][3]);
    Q->y = f * (P.x * RM[1][0] + P.y * RM[1][1] + P.z * RM[1][2] + RM[1][3]);
    Q->z = f * (P.x * RM[2][0] + P.y * RM[2][1] + P.z * RM[2][2] + RM[2][3]);
} /* end N_TransformPt */

/*******************************************************************//**


   DESCRIPTION:

     This  arithmetic routine  translates a control point by a given 
     vector, i.e. it computes Qw = Pw + T. A typical calling example
     is:

       NL_VECTOR  T;
       NL_CPOINT  Pw, Qw;
       ...
       (get T and Pw);
       ...
       N_TranslateCPt(Pw,T,&Qw);


   ACCESS:
   
     Pw  , input  ,  Homogeneous point
     T   , input  ,  Translation vector
     Qw  , output ,  Translate of Pw



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_TranslateCPt( NL_CPOINT Pw, NL_VECTOR T, NL_CPOINT *Qw )
{

    /* Locate weighted control point in Euclidean space */

    if( Pw.w NEQ NL_NOW )
    {
        Pw.x /= Pw.w;
        Pw.y /= Pw.w;

        if( Pw.z NEQ NL_NOZ )
            Pw.z /= Pw.w;
    }

    /* Translate control point */

    Qw->x = Pw.x + T.x;
    Qw->y = Pw.y + T.y;

    if( Pw.z NEQ NL_NOZ )
        Qw->z = Pw.z + T.z;
    else
        Qw->z = NL_NOZ;

    /* Weight transformed control point if neccesary */

    if( Pw.w NEQ NL_NOW )
    {
        Qw->x *= Pw.w;
        Qw->y *= Pw.w;

        if( Pw.z NEQ NL_NOZ )
            Qw->z *= Pw.w;
        Qw->w = Pw.w;
    }
    else
    {
        Qw->w = NL_NOW;
    }
} /* end N_TranslateCPt */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine scales a control point by a given vector-
     valued  factor with respect to a point. A typical calling example
     is:

       NL_VECTOR  f;
       NL_POINT   C;
       NL_CPOINT  Pw, Qw;
       ...
       (get f, C and Pw);
       ...
       N_ScaleCPtWithPtAndVector(Pw,C,f,&Qw);


   ACCESS:
   
     Pw  , input  ,  Homogeneous point
     C   , input  ,  Center of scaling
     f   , input  ,  Vector-valued scale factor
     Qw  , output ,  Scale of Pw



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ScaleCPtWithPtAndVector( NL_CPOINT Pw, NL_POINT C, NL_VECTOR f, NL_CPOINT *Qw )
{

    NL_REAL w;

    NL_VECTOR g;

    NL_CPOINT Cw;

    /* Get g and Cw */

    g.x = 1.0 - f.x;
    g.y = 1.0 - f.y;
    g.z = 1.0 - f.z;

    if( Pw.w NEQ NL_NOW )
        w = Pw.w;
    else
        w = 1.0;

    Cw.x = w * C.x;
    Cw.y = w * C.y;
    Cw.z = w * C.z;
    Cw.w = w;

    /* Scale control point */

    Qw->x = f.x * Pw.x + g.x * Cw.x;
    Qw->y = f.y * Pw.y + g.y * Cw.y;

    if( Pw.z NEQ NL_NOZ )
        Qw->z = f.z * Pw.z + g.z * Cw.z;
    else
        Qw->z = NL_NOZ;

    if( Pw.w NEQ NL_NOW )
        Qw->w = Pw.w;
    else
        Qw->w = NL_NOW;
} /* end N_ScaleCPtWithPtAndVector */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine converts a point to a control point object 
NL_NOW     by setting the weight to NL_NOW. A typical calling example is:

       NL_POINT   P;
       NL_CPOINT  Pw;
       ...
       (get P);
       ...
       N_PtToCPt(P,&Pw);


   ACCESS:
   
     P  , input  ,  Point object
     Pw , output ,  Control point converted from P



   RETURN CODES:

     None

   ***********************************************************************/
NL_VOID N_PtToCPt( NL_POINT P, NL_CPOINT *Pw )
{
  Pw->x = P.x;
  Pw->y = P.y;
  Pw->z = P.z;
  Pw->w = NL_NOW;

} /* end N_PtToCPt */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine extracts the weight of a control point. A
     typical calling example is:

       NL_CPOINT  Pw;
       NL_REAL    w;
       ...
       (get Pw);
       ...
       N_CPtGetW(Pw,&w);


   ACCESS:
   
     Pw , input  ,  Control point
     w  , output ,  Weight



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtGetW( NL_CPOINT Pw, NL_REAL *w )
{

    *w = Pw.w;
} /* end N_CPtGetW */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine extracts the coordinates of a pont. A
     typical calling example is:

       NL_POINT  P;
       NL_REAL   x, y, z;
       ...
       (get P);
       ...
       N_PtToXYZ(P,&x,&y,&z);


   ACCESS:
   
     P      , input  ,  Point
     x,y,z  , output ,  Coordinates of P



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PtToXYZ( NL_POINT P, NL_REAL *x, NL_REAL *y, NL_REAL *z )
{

    *x = P.x;
    *y = P.y;
    *z = P.z;
} /* end N_PtToXYZ */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine creates a point object given the x, y and
     z coordinates. A typical calling example is:

       NL_REAL   x, y, z;
       NL_POINT  P;
       ...
       (get x, y and z);
       ...
       N_PtFromXYZ(x,y,z,&P);    


   ACCESS:
   
     x,y,z  , input  ,  Coordinates of P
     P      , output ,  Point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PtFromXYZ( NL_REAL x, NL_REAL y, NL_REAL z, NL_POINT *P )
{

    P->x = x;
    P->y = y;
    P->z = z;
} /* end N_PtFromXYZ */

/*******************************************************************//**


   DESCRIPTION:

     This  arithmetic routine extracts the coordinates of a control 
     pont. A typical calling example is:

       NL_CPOINT  Pw;
       NL_REAL    wx, wy, wz, w;
       ...
       (get Pw);
       ...
       N_CPtToWxWyWz(Pw,&wx,&wy,&wz,&w);


   ACCESS:
   
     Pw          , input  ,  Control point
     wx,wy,wz,w  , output ,  Coordinates of Pw



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtToWxWyWz
 (NL_CPOINT Pw, /* in : Tgt Control point */
  NL_REAL  *wx, /* out: x of Cpt[x y z w] */
  NL_REAL  *wy, /* out: y of Cpt[x y z w] */
  NL_REAL  *wz, /* out: z of Cpt[x y z w] */
  NL_REAL  *w ) /* out: w of Cpt[x y z w] */
{

    *wx = Pw.x;
    *wy = Pw.y;
    *wz = Pw.z;
    *w = Pw.w;
} /* end N_CPtToWxWyWz */


/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine creates a control point object given the 
     wx, wy, wz and w coordinates. A typical calling example is:

     NL_REAL    wx, wy, wz, w;
     NL_CPOINT  Pw;
     ...
     (get wx, wy, wz and w);
     ...
     N_CPtFromWxWyWz(wx,wy,wz,w,&Pw);


   ACCESS:
   
     wx,wy,wz,w  , input  ,  Coordinates of Pw
     Pw          , output ,  Control point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtFromWxWyWz
 (NL_REAL wx,     /* in :  x  of CPt[x y z w] */
  NL_REAL wy,     /* in :  y  of CPt[x y z w] */
  NL_REAL wz,     /* in :  z  of CPt[x y z w] */
  NL_REAL w,      /* in :  w  of CPt[x y z w] */
  NL_CPOINT *Pw ) /* out: CPt of CPt[x y z w] */
{

    Pw->x = wx;
    Pw->y = wy;
    Pw->z = wz;
    Pw->w = w;
} /* end N_CPtFromWxWyWz */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine sets the x component of a control point.
     A typical calling example is:

       NL_REAL    wx;
       NL_CPOINT  Pw;
       ...
       (get wx);
       ...
       N_CPtSetX(wx,&Pw);


   ACCESS:
   
     wx , input  ,  X component of Pw
     Pw , output ,  Control point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtSetX( NL_REAL wx, NL_CPOINT *Pw )
{

    Pw->x = wx;
} /* end N_CPtSetX */


/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine sets the y component of a control point.
     A typical calling example is:

       NL_REAL    wy;
       NL_CPOINT  Pw;
       ...
       (get wy);
       ...
       N_CPtSetY(wy,&Pw);


   ACCESS:
   
     wy , input  ,  Y component of Pw
     Pw , output ,  Control point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtSetY( NL_REAL wy, NL_CPOINT *Pw )
{

    Pw->y = wy;
} /* end N_CPtSetY */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine sets the z component of a control point.
     A typical calling example is:

       NL_REAL    wz;
       NL_CPOINT  Pw;
       ...
       (get wz);
       ...
       N_CPtSetZ(wz,&Pw);


   ACCESS:
   
     wz , input  ,  Z component of Pw
     Pw , output ,  Control point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtSetZ( NL_REAL wz, NL_CPOINT *Pw )
{

    Pw->z = wz;
} /* end N_CPtSetZ */


/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine sets the w component of a control point.
     A typical calling example is:

       NL_REAL    w;
       NL_CPOINT  Pw;
       ...
       (get w);
       ...
       N_CPtSetW(w,&Pw);


   ACCESS:
   
     w  , input  ,  W component of Pw
     Pw , output ,  Control point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtSetW( NL_REAL w, NL_CPOINT *Pw )
{

    Pw->w = w;
} /* end N_CPtSetW */


/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the sum of two points, i.e. it 
     computes C = A + B. A typical calling example is:

       NL_POINT  A, B, C;
       ...
       (get A and B);
       ...
       N_Sum2Pts(A,B,&C);


   ACCESS:
   
     A     , input  ,  First point
     B     , input  ,  Second point
     C     , output ,  Sum of A and B



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Sum2Pts( NL_POINT A, NL_POINT B, NL_POINT *C )
{

    C->x = A.x + B.x;
    C->y = A.y + B.y;
    C->z = A.z + B.z;
} /* end N_Sum2Pts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the sum of two control points, 
     i.e. it computes Cw = Aw + Bw. A typical calling example is:

       NL_CPOINT  Aw, Bw, Cw;
       ...
       (get Aw and Bw);
       ...
       N_Sum2CPts(Aw,Bw,&Cw);


   ACCESS:
   
     Aw  , input  ,  First control point
     Bw  , input  ,  Second control point
     Cw  , output ,  Sum of Aw and Bw



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Sum2CPts( NL_CPOINT Aw, NL_CPOINT Bw, NL_CPOINT *Cw )
{

    Cw->x = Aw.x + Bw.x;
    Cw->y = Aw.y + Bw.y;

    if( Aw.z NEQ NL_NOZ AND Bw.z NEQ NL_NOZ )
        Cw->z = Aw.z + Bw.z;

    else if( Aw.z NEQ NL_NOZ )
        Cw->z = Aw.z;

    else if( Bw.z NEQ NL_NOZ )
        Cw->z = Bw.z;

    else
        Cw->z = NL_NOZ;

    if( Aw.w NEQ NL_NOW AND Bw.w NEQ NL_NOW )
        Cw->w = Aw.w + Bw.w;
    else
        Cw->w = NL_NOW;
} /* end N_Sum2CPts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the sum of a control point and a
     point,  i.e.  it  computes  Cw = Aw + B.  Only  the  first  three 
     coordinates  are  used to  compute  the  sum. A  typical  calling 
     example is:

       NL_CPOINT  Aw, Cw;
       NL_POINT   B;
       ...
       (get Aw and B);
       ...
       N_SumCPtAndPt(Aw,B,&Cw);


   ACCESS:
   
     Aw  , input  ,  Control point
     B   , input  ,  Point
     Cw  , output ,  Sum of Aw and B



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SumCPtAndPt( NL_CPOINT Aw, NL_POINT B, NL_CPOINT *Cw )
{

    Cw->x = Aw.x + B.x;
    Cw->y = Aw.y + B.y;

    if( Aw.z NEQ NL_NOZ )
        Cw->z = Aw.z + B.z;
    else
        Cw->z = NL_NOZ;
    Cw->w = Aw.w;
} /* end N_SumCPtAndPt */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the difference of two points, i.e. 
     it computes C = A - B. A typical calling example is:

       NL_POINT  A, B, C;
       ....
       (get A and B);
       ...
       N_Diff2Pts(A,B,&C);


   ACCESS:
   
     A     , input  ,  First point
     B     , input  ,  Second point
     C     , output ,  Difference of A and B



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Diff2Pts( NL_POINT A, NL_POINT B, NL_POINT *C )
{

    C->x = A.x - B.x;
    C->y = A.y - B.y;
    C->z = A.z - B.z;
} /* end N_Diff2Pts */

/*******************************************************************//**


   DESCRIPTION:

     This  arithmetic routine  computes the difference of two control 
     points, i.e. it computes Cw = Aw - Bw. A typical calling example
     is:

       NL_CPOINT  Aw, Bw, Cw;
       ...
       (get Aw and Bw);
       ...
       N_Diff2CPts(Aw,Bw,&Cw);


   ACCESS:
   
     Aw  , input  ,  First control point
     Bw  , input  ,  Second control point
     Cw  , output ,  Difference of Aw and Bw


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Diff2CPts
 (NL_CPOINT Aw,    // in : Aw of Cw = Aw - Bw
  NL_CPOINT Bw,    // in : Bw of Cw = Aw - Bw
  NL_CPOINT *Cw )  // out: Cw of Cw = Aw - Bw  (special cases: NL_NOZ and NL_NOW)
{
  Cw->x = Aw.x - Bw.x;
  Cw->y = Aw.y - Bw.y;
  
  if( Aw.z NEQ NL_NOZ AND Bw.z NEQ NL_NOZ )
      Cw->z = Aw.z - Bw.z;
  
  else if( Aw.z NEQ NL_NOZ )
      Cw->z = Aw.z;
  
  else if( Bw.z NEQ NL_NOZ )
      Cw->z = -Bw.z;
  
  else
      Cw->z = NL_NOZ;
  
  if( Aw.w NEQ NL_NOW AND Bw.w NEQ NL_NOW )
      Cw->w = Aw.w - Bw.w;
  else
      Cw->w = NL_NOW;

} /* end N_Diff2CPts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the difference of a control point 
     and a point, i.e. it computes Cw = Aw - B. Only  the  first  three 
     coordinates are used to compute the difference. A typical  calling 
     example is:

       NL_CPOINT  Aw, Cw;
       NL_POINT   B;
       ...
       (get Aw and B);
       ...
       N_DiffCPtPt(Aw,B,&Cw);


   ACCESS:
   
     Aw  , input  ,  Control point
     B   , input  ,  Point
     Cw  , output ,  Difference of Aw and B



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DiffCPtPt( NL_CPOINT Aw, NL_POINT B, NL_CPOINT *Cw )
{

    Cw->x = Aw.x - B.x;
    Cw->y = Aw.y - B.y;

    if( Aw.z NEQ NL_NOZ )
        Cw->z = Aw.z - B.z;
    else
        Cw->z = NL_NOZ;
    Cw->w = Aw.w;
} /* end N_DiffCPtPt */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the dot product of two points, 
     i.e. it computes dot = A*B. A typical calling example is:

       NL_POINT  A, B;
       NL_REAL   dot;
       ...
       (get A and B);
       ...
       N_Dot2Pts(A,B,&dot);


   ACCESS:
   
     A   , input  ,  First point
     B   , input  ,  Second point
     dot , output ,  Dot product


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Dot2Pts( NL_POINT A, NL_POINT B, NL_REAL *dot )
{

    *dot = A.x * B.x + A.y * B.y + A.z * B.z;
} /* end N_Dot2Pts */



/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the dot product of two control 
     points, i.e. it computes dw = Aw*Bw. The product is computed in
     homogeneous space. A typical calling example is:

       NL_CPOINT  Aw, Bw;
       NL_REAL    dw;
       ...
       (get Aw and Bw);
       ...
       N_Dot2CPtsIn4D(Aw,Bw,&dw);


   ACCESS:
   
     Aw , input  ,  First control point
     Bw , input  ,  Second control point
     dw , output ,  Dot product


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Dot2CPtsIn4D( NL_CPOINT Aw, NL_CPOINT Bw, NL_REAL *dw )
{

    *dw = Aw.x * Bw.x + Aw.y * Bw.y;

    if( Aw.z NEQ NL_NOZ AND Bw.z NEQ NL_NOZ )
        *dw += Aw.z*Bw.z;

    if( Aw.w NEQ NL_NOW AND Bw.w NEQ NL_NOW )
        *dw += Aw.w*Bw.w;

    else if( Aw.w NEQ NL_NOW )
        *dw += Aw.w;

    else if( Bw.w NEQ NL_NOW )
        *dw += Bw.w;
} /* end N_Dot2CPtsIn4D */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine  computes the dot product of  two control 
     points  as  follows: the  first  three  coordinates  are  used to 
     compute  the dot  product of  the <wx,wy,wz>  components, and the 
     fourth coordinates are used to compute the product of the weights. 
     A typical calling example is:

       NL_CPOINT  Aw, Bw;
       NL_REAL    dot, dow;
       ...
       (get Aw and Bw);
       ...
       N_Dot2CPts(Aw,Bw,&dot,&dow);


   ACCESS:
   
     Aw  , input  ,  First control point
     Bw  , input  ,  Second control point
     dot , output ,  Dot product of wx,wy,wz components
     dow , output ,  Product of weights


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Dot2CPts( NL_CPOINT Aw, NL_CPOINT Bw, NL_REAL *dot, NL_REAL *dow )
{

    *dot = Aw.x * Bw.x + Aw.y * Bw.y;

    if( Aw.z NEQ NL_NOZ AND Bw.z NEQ NL_NOZ )
        *dot += Aw.z*Bw.z;

    if( Aw.w NEQ NL_NOW AND Bw.w NEQ NL_NOW )
        *dow  = Aw.w*Bw.w;

    else if( Aw.w NEQ NL_NOW )
        *dow = Aw.w;

    else if( Bw.w NEQ NL_NOW )
        *dow = Bw.w;

    else
        *dow = NL_NOW;
} /* end N_Dot2CPts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine  computes the cross product of two control 
     points  as  follows: the  first  three  coordinates  are  used  to 
     compute the cross  product of the  <wx,wy,wz>  components, and the 
     fourth coordinates are used to compute the product of the weights. 
     A typical calling example is:

       NL_CPOINT  Aw, Bw, Cw;
       ...
       (get Aw and Bw);
       ...
       N_Cross2CPts(Aw,Bw,&Cw);


   ACCESS:
   
     Aw  , input  ,  First control point
     Bw  , input  ,  Second control point
     Cw  , output ,  Aw x Bw


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Cross2CPts( NL_CPOINT Aw, NL_CPOINT Bw, NL_CPOINT *Cw )
{
    if( Aw.z EQ NL_NOZ )
        Aw.z = 0.0;

    if( Bw.z EQ NL_NOZ )
        Bw.z = 0.0;

    if( Aw.w NEQ NL_NOW OR Bw.w NEQ NL_NOW )
    {
        if( Aw.w EQ NL_NOW )
            Aw.w = 1.0;

        if( Bw.w EQ NL_NOW )
            Bw.w = 1.0;
    }

    Cw->x = Aw.y * Bw.z - Aw.z * Bw.y;
    Cw->y = Aw.z * Bw.x - Aw.x * Bw.z;
    Cw->z = Aw.x * Bw.y - Aw.y * Bw.x;

    if( Aw.w NEQ NL_NOW OR Bw.w NEQ NL_NOW )
    {
        Cw->w = Aw.w * Bw.w;
    }
    else
    {
        Cw->w = NL_NOW;
    }
} /* end N_Cross2CPts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine  computes the cross product of two vectors
     A typical calling example is:

       NL_POINT  Aw, Bw, Cw;
       ...
       (get Aw and Bw);
       ...
       N_Cross2CPts(Aw,Bw,&Cw);


   ACCESS:
   
     Aw  , input  ,  First vector
     Bw  , input  ,  Second vector
     Cw  , output ,  Aw x Bw


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Cross2Pts( NL_POINT Aw, NL_POINT Bw, NL_POINT *Cw )
{
    if( Aw.z EQ NL_NOZ )
        Aw.z = 0.0;

    if( Bw.z EQ NL_NOZ )
        Bw.z = 0.0;

    Cw->x = Aw.y * Bw.z - Aw.z * Bw.y;
    Cw->y = Aw.z * Bw.x - Aw.x * Bw.z;
    Cw->z = Aw.x * Bw.y - Aw.y * Bw.x;
} /* end N_Cross2Pts */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the magnitude of a point, i.e. it 
     computes |A|. A typical calling example is:

       NL_POINT  A;
       NL_REAL   mag;
       ...
       (get A);
       ...
       N_PtMagnitude(A,&mag);


   ACCESS:
   
     A   , input  ,  Point
     mag , output ,  |A|


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PtMagnitude( NL_POINT A, NL_REAL *mag )
{

    NL_REAL d;

    N_Dot2Pts( A, A, &d );

    *mag = sqrt( d );
} /* end N_PtMagnitude */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes the magnitude of a control point, 
     i.e. it computes |Aw|. A typical calling example is:

       NL_CPOINT  Aw;
       NL_REAL   mag;
       ...
       (get Aw);
       ...
       N_CPtMagnitude(Aw,&mag);


   ACCESS:
   
     Aw  , input  ,  Control point
     mag , output ,  |Aw|


   RETURN CODES:

     None

   ***********************************************************************/
NL_VOID N_CPtMagnitude
 (NL_CPOINT  Aw,  // in : target vector
  NL_REAL  * mag) // out: mag = sqrt(Dot(Aw,Aw))
{

    NL_REAL dw;

    N_Dot2CPtsIn4D( Aw, Aw, &dw );

    *mag = sqrt( dw );
} /* end N_CPtMagnitude */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine computes rational derivative given control
     point, non rational derivative, and  the derivative of the weight. 
     More  precisely, given  Pw=(xw,yw,zw,w),  D=(xd,yd,zd), and wd, it
     computes

       Dw=(wd*x+xd*w,wd*y+yd*w,wd*z+zd*w,wd)
         =(d(w*x)/dt,d(w*y)/dt,d(w*z)/dt,dw/dt)

     where Dw is the rational derivative. A typical calling example is:

       NL_CPOINT   Pw;
       NL_VECTOR   D;
       NL_REAL     wd;
       NL_CVECTOR  Dw;
       ...
       (get Pw, D, and wd);
       ...
       N_CPtRatDeriv(Pw,D,wd,&Dw);


   ACCESS:
   
     Pw , input  ,  Control point
     D  , input  ,  Derivative vector in Euclidean space
     wd , input  ,  Derivative of w
     Dw , output ,  Rational derivative


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtRatDeriv( NL_CPOINT Pw, NL_VECTOR D, NL_REAL wd, NL_CVECTOR *Dw )
{

    NL_POINT P;

    NL_REAL w;

    N_CPtToPtEuclid( Pw, &P );
    N_CPtGetW( Pw, &w );

    Dw->x = wd * P.x + D.x * w;
    Dw->y = wd * P.y + D.y * w;

    if( Pw.z NEQ NL_NOZ )
        Dw->z = wd * P.z + D.z * w;
    else
        Dw->z = NL_NOZ;
    Dw->w = wd;
} /* end N_CPtRatDeriv */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine extracts the Euclidean coordinates of a
     control point. A typical calling example is:

       NL_CPOINT  Pw;
       NL_REAL    x, y, z;
       ...
       (get Pw);
       ...
       N_CPtToXYZ(Pw,&x,&y,&z);


   ACCESS:
   
     Pw     , input  ,  Control point
     x,y,z  , output ,  Euclidean coordinates of Pw



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtToXYZ( NL_CPOINT Pw, NL_REAL *x, NL_REAL *y, NL_REAL *z )
{

    NL_REAL w;

    *x = Pw.x;
    *y = Pw.y;

    if( Pw.z EQ NL_NOZ )
        *z = 0.0;
    else
        *z = Pw.z;

    if( Pw.w NEQ NL_NOW )
    {
        w = Pw.w;
        *x /= w;
        *y /= w;
        *z /= w;
    }
} /* end N_CPtToXYZ */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine maps a weighted (homogeneous) control point 
     to a Euclidean control point. That is,

NL_NOW       (xw,yw,zw  ,w) ---> (x,y,z   ,NL_NOW)  or
       (xw,yw,NL_NOZ,w) ---> (x,y,NL_NOZ,NL_NOW) 

     A typical calling example is:

       NL_CPOINT  Pw, Qw;
       ...
       (get Pw);
       ...
       N_CPtToPtNoW(Pw,&Qw);


   ACCESS:
   
     Pw  , input  ,  Homogeneous control point
     Qw  , output ,  Euclidean control point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtToPtNoW(NL_CPOINT Pw, NL_CPOINT* Qw)
{
    if (Pw.w NEQ NL_NOW)
    {
        NL_REAL wReciprocal = 1.0 / Pw.w;
        Qw->x = Pw.x * wReciprocal;
        Qw->y = Pw.y * wReciprocal; 

        if (Pw.z NEQ NL_NOZ)
            Qw->z = Pw.z * wReciprocal;
        else
            Qw->z = NL_NOZ;
        Qw->w = NL_NOW;
    }
    else
    {
        Qw->x = Pw.x;
        Qw->y = Pw.y;

        if (Pw.z NEQ NL_NOZ)
            Qw->z = Pw.z;
        else
            Qw->z = NL_NOZ;
        Qw->w = Pw.w;
    }
} /* end N_CPtToPtNoW */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine extracts the z coordinate of a control 
     point. A typical calling example is:

       NL_CPOINT  Pw;
       NL_REAL    z;
       ...
       (get Pw);
       ...
       N_CPtGetZ(Pw,&z);


   ACCESS:
   
     Pw , input  ,  Control point
     z  , output ,  Z-coordinate



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtGetZ( NL_CPOINT Pw, NL_REAL *z )
{

    *z = Pw.z;
} /* end N_CPtGetZ */

/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine re-weights a control point. A typical 
     calling example is:

       NL_CPOINT  Pw, Qw;
       NL_REAL    w;
       ...
       (get Pw and w);
       ...
       N_CPtResetW(Pw,w,&Qw);
       N_CPtResetW(Pw,w,&Pw);


   ACCESS:
   
     Pw , input  ,  Homogeneous point
     w  , input  ,  New weight of Pw 
     Qw , output ,  Re-weighted control point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtResetW( NL_CPOINT Pw, NL_REAL w, NL_CPOINT *Qw )
{

    NL_POINT P;

    N_CPtToPtEuclid( Pw, &P );
    N_Weight( P, w, Qw );
} /* end N_CPtResetW */


/*******************************************************************//**


   DESCRIPTION:

     This arithmetic routine maps a control point to the w = 0 hyper-
     space, i.e. it  drops the  fourth  coordinate. A typical calling 
     example is:

       NL_CPOINT  Pw;
       NL_POINT   P;
       ...
       (get Pw);
       ...
       N_CPtToPt(Pw,&P);


   ACCESS:
   
     Pw  , input  ,  Homogeneous point
     P   , output ,  Euclidean point



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtToPt( NL_CPOINT Pw, NL_POINT *P )
{

    P->x = Pw.x;
    P->y = Pw.y;

    if( Pw.z NEQ NL_NOZ )
        P->z = Pw.z;
    else
        P->z = 0.0;
} /* end N_CPtToPt */


/*******************************************************************//**


   DESCRIPTION:

     This math routine solves a quadratic equation of the form

             a*x^2 + b*x + c = 0

     Either zero, one or two solutions are returned. The roots are
     x1 = q/a and x2 = c/q where q = -0.5*(b+sgn(b)*sqrt(b*b-4*a*c))
     A typical calling example is:

       NL_INTEGER  n;
       NL_REAL     a, b, c, r1, r2;
       ...
       (get a, b and c );
       ...
       N_SolveQuadraticEq(a,b,c,&r1,&r2,&n);

     NOTE: ONE ROOT AND NL_ZERO COEFFICIENTS ARE CHECKED USING THE GLOBAL
     TOLERANCES "NL_ZDTL" AND "NL_ZCTL" APPLIED TO THE EQUATION.


   ACCESS:
   
     a,b,c , input  ,  Coefficients
     r1,r2 , output ,  Roots (r1<=r2)
     n     , output ,  Number of roots returned (0,1 or 2)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SolveQuadraticEq( NL_REAL a, NL_REAL b, NL_REAL c, NL_REAL *r1, NL_REAL *r2, NL_INTEGER *n )
{
    NL_PRIVATE NL_STRING rname = _T("N_SolveQuadraticEq");

    NL_FLAG error = NL_NO;

    /*  NL_REAL  max; Not used if there is no normalization  */

    NL_REAL dis, q;

    /***********************/
    /* Check special cases */
    /***********************/

    *n = 0;

    /* Revision Jan 03  RMB
       Based on comments on the quadratic equation 
       in Numerical Recipes in C, page 184, "If either
       a or c (or both) are small, then the roots will
       involve the subtraction of b from a very nearly 
       equal quantity (the discriminant); you will get
       that root very inaccurately."
       Based on a bug, normalizing can make a or c even 
       smaller hence the problem gets worse!!  
       So do NOT normalize.
    */
    /* Normalize coefficients */
    /*
                            max = fabs(a);
      if (fabs(b) GT max )
      max = fabs(b);
      if (fabs(c) GT max )
      max = fabs(c);
    
      if (N_FloatOpIsBad(a,max,NL_DIVISION) )
      NL_ERROR(NL_NUM_ERR);
      if (N_FloatOpIsBad(b,max,NL_DIVISION) )
      NL_ERROR(NL_NUM_ERR);
      if (N_FloatOpIsBad(c,max,NL_DIVISION) )
      NL_ERROR(NL_NUM_ERR);
    
      a = a/max;
      b = b/max;
      c = c/max;
    */

    /* a=b=0 --> c=0 */

    if( fabs( a )LT NL_ZCTL AND fabs( b )LT NL_ZCTL )
        NL_OUT;

    /* a=c=0 --> b*x=0 */

    if( fabs( a )LT NL_ZCTL AND fabs( c )LT NL_ZCTL )
    {
        *r1 = 0.0;
        *r2 = 0.0;
        *n = 1;

        NL_OUT;
    }

    /* b=c=0 --> a*x*x=0 */

    if( fabs( b )LT NL_ZCTL AND fabs( c )LT NL_ZCTL )
    {
        *r1 = 0.0;
        *r2 = 0.0;
        *n = 1;

        NL_OUT;
    }

    /* a=0 --> b*x+c=0 */

    if( fabs( a )LT NL_ZCTL )
    {
        if( N_FloatOpIsBad( -c, b, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        *r1 = -c / b;
        *r2 = *r1;
        *n = 1;

        NL_OUT;
    }

    /* b=0 --> a*x*x+c=0 */

    if( fabs( b )LT NL_ZCTL )
    {
        if( N_FloatOpIsBad( -c, a, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        dis = -c / a;

        if( dis LT 0.0 )
            NL_OUT;

        dis = sqrt( dis );
        *r1 = -dis;
        *r2 = dis;
        *n = 2;

        NL_OUT;
    }

    /* c=0 --> a*x*x+b*x=0 */

    if( fabs( c )LT NL_ZCTL )
    {
        if( N_FloatOpIsBad( -b, a, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        *r1 = 0.0;
        *r2 = -b / a;
        *n = 2;

        if( *r1 GT *r2 )
            N_SwapReals( r1, r2 );

        NL_OUT;
    }

    /**********************************/
    /* Continue with the regular case */
    /**********************************/

    /* Get discriminant */

    dis = b * b - 4.0 *a * c;

    /* No solution */

    if( dis LT - NL_ZDTL )
        NL_OUT;

    /* One root */

    if( fabs( dis )LT NL_ZDTL )
    {
        if( N_FloatOpIsBad( -b, 2.0 *a, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        *r1 = -b / (2.0 *a);
        *r2 = *r1;
        *n = 1;

        NL_OUT;
    }

    /* Two roots */

    dis = sqrt( dis );
    q = -0.5 *( b + NL_SIGN( b ) * dis );

    if( N_FloatOpIsBad( q, a, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    if( N_FloatOpIsBad( c, q, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    *r1 = q / a;
    *r2 = c / q;
    *n = 2;

    if( *r1 GT *r2 )
        N_SwapReals( r1, r2 );

    /* Exit */

    EXIT:

    return (error);
} /* end N_SolveQuadraticEq */

/*******************************************************************//**


   DESCRIPTION:

     This math routine checks if floating point  operations result in
     underflow or  overflow. The operations  are assumed to be in the
     form of x op y, where op is one of the four basic arithmetic
     operations. A typical calling example is:

       NL_REAL  x, y;
       ...
       (get x and y);
       ...
       if( N_FloatOpIsBad(x,y,NL_DIVISION) )  NL_ERROR(NL_NUM_ERR);


   ACCESS:
   
     x  , input  ,  First operand
     y  , input  ,  Second operand
     op , input  ,  Type of arithmetic operation: 
                      NL_ADDITION  
                      NL_SUBTRACTION 
                      NL_MULTIPLICATION 
                      NL_DIVISION.


   RETURN CODES:

     NL_TRUE : Error, operation CANNOT be performed
     NL_FALSE: No error

   ***********************************************************************/

NL_BOOLEAN N_FloatOpIsBad( NL_REAL x, NL_REAL y, NL_FLAG op )
{
    /* No longer used:   NL_REAL lx, ly; */

    /* NLIBCHKBYPASS can speed up computation, by bypassing numeric range (log) checks */
    /* However, this has been changed to avoid taking logs:
     * they were expensive enough to cause significant,
     * noticeable performance problems.
    */
    if( NLIBCHKBYPASS > 0 )
      { return NL_FALSE; }

    switch( op )
    {
        case NL_MULTIPLICATION:
            if( x EQ 0.0 )
              { return NL_FALSE; }

            if( y EQ 0.0 )
              { return NL_FALSE; }

            if( x LT 0.0 )
              { x = -x; }

            if( y LT 0.0 )
              { y = -y; }

            /* Do not use logs: too expensive.
             * Note also, just saying NL_LBIGD takes a log...
             *  lx = log( x );
             *  ly = log( y );
             *
             *  / * Check overflow and underflow * /
             *
             *  if( (lx + ly)GE NL_LBIGD OR( lx + ly )LE NL_LSMAD )
             *    { return NL_TRUE; }
             *  else
             *    { return NL_FALSE; }
             */

            /* If they're on opposite sides of 1.0, we're ok. */
            if ( x >= 1.0 && y <= 1.0 )
              { return NL_FALSE; }
            if ( x <= 1.0 && y >= 1.0 )
              { return NL_FALSE; }

            /* Now both are either < 1.0 or > 1.0. */
            /* Method: we're ok if x*y < NL_BIGD and x*y > NL_SMAD.
             * Rearrange those inequalities to avoid actually performing
             * x*y, and being sure that no operation used can overflow.
             */
            if ( x >= 1.0 ) {
                /* Check overflow. */
                if ( y <= NL_BIGD / x ) /* ok to divide: x >= 1.0 */
                  { return NL_FALSE; }
                if ( x <= NL_BIGD / y ) /* y is also >= 1.0. */
                  { return NL_FALSE; }
            }
            else if ( x <= 1.0 )
            {
                /* Check underflow. */
                /* Note, x and y are both > 0.0, and they couldn't be < NL_SMAD, */
                /* so NL_SMAD / y must be <= 1.0, i.e., it won't overflow.      */
                if ( x > NL_SMAD / y )
                  { return NL_FALSE; }
            }

            return NL_TRUE;
            break;

        case NL_DIVISION:
            if( y EQ 0.0 )
              { return NL_TRUE; }

            if( x EQ 0.0 )
              { return NL_FALSE; }

            if( x LT 0.0 )
              { x = -x; }

            if( y LT 0.0 )
              { y = -y; }

            /* Do not use logs: see note above.
             *  lx = log( x );
             *  ly = log( y );
             *
             *  / * Check overflow and underflow * /
             *
             *  if( lx GE( ly + NL_LBIGD )OR lx LE( ly + NL_LSMAD ) )
             *    { return NL_TRUE; }
             *  else
             *    { return NL_FALSE; }
             */

            /* Method: see notes above. */
            /* We're ok if x/y < NL_BIGD and x/y > NL_BIGD. */
            if ( y <= 1.0 && x < NL_BIGD * y )  /* Checks overflow */
              { return NL_FALSE; }
            if ( y >= 1.0 && x > NL_SMAD * y )  /* Checks underflow */
              { return NL_FALSE; }

            return NL_TRUE;
            break;

        case NL_ADDITION:
            if( NL_SIGN( x ) NEQ NL_SIGN( y ) )
              { return NL_FALSE; }

            if( x LT 0.0 )
              { x = -x; }

            if( y LT 0.0 )
              { y = -y; }

            if( y LT( NL_BIGD - x ) ) /* Check for overflow */
              { return NL_FALSE; }
            else
              { return NL_TRUE; }
            break;

        case NL_SUBTRACTION:
            if( NL_SIGN( x )*NL_SIGN( y ) GT 0.0 )
              { return NL_FALSE; }

            if( x LT 0.0 )
              { x = -x; }

            if( y GT 0.0 )
              { y = -y; }

            if( y GT( x - NL_BIGD ) ) /* Check for overflow */
              { return NL_FALSE; }
            else
              { return NL_TRUE; }

            break;

        default:

            return NL_TRUE;
    }
} /* end N_FloatOpIsBad */

/*******************************************************************//**


   DESCRIPTION:

     This allows the user to set a private global (static) for bypassing error
     checking for numeric range tests that involve 2 slow 'log' computations.
     It may be temporarily set on or off

   ACCESS:
   
     OnOff , input  ,  1 = On  = bypass error checking
                       0 = Off = do error checking (default)
                       typical call: 
                           M_NoErrChk(1);
                           do intensive calculations
                           M_NoErrChk(0);

   RETURN CODES:

     NL_NONE

   ***********************************************************************/

NL_VOID M_NoErrChk( NL_INDEX OnOff )
{
    NLIBCHKBYPASS = OnOff;
} /* end M_NoErrChk  */

/*******************************************************************//**


   DESCRIPTION:

     This math routine swaps two integer numbers. A typical calling
     example is:

       NL_INTEGER  i, j;
       ...
       (get i and j);
       ...
       N_SwapIntegers(&i,&j);


   ACCESS:
   
     i , input  ,  First number
     j , input  ,  Second number


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SwapIntegers( NL_INTEGER *i, NL_INTEGER *j )
{

    NL_INTEGER tmp;

    tmp = *i;
    *i = *j;
    *j = tmp;
} /* end N_SwapIntegers */

/*******************************************************************//**


   DESCRIPTION:

     This math routine swaps two real numbers. A typical calling 
     example is:

       NL_REAL  a, b;
       ...
       (get a and b);
       ...
       N_SwapReals(&a,&b);


   ACCESS:
   
     a , input  ,  First number
     b , input  ,  Second number


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SwapReals( NL_REAL *a, NL_REAL *b )
{

    NL_REAL tmp;

    tmp = *a;
    *a = *b;
    *b = tmp;
} /* end N_SwapReals */

/*******************************************************************//**


   DESCRIPTION:

     This math routine sorts a real array using Shellsort. A typical 
     calling example is:

       NL_REAL   *a;
       NL_INDEX  n;
       ...
       (get array a);
       ...
       N_ShellSortReal(a,n);


   ACCESS:
   
     a , in/out ,  Real array
     n , input  ,  Highest index in a


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ShellSortReal( NL_REAL *a, NL_INDEX n )
{

    NL_INDEX i, j, k;

    NL_REAL b;

    k = n + 1;

    while( k GT 1 )
    {
        if( k GE 5 )
            k = (5 * k - 1) / 11;
        else
            k = 1;

        for ( i = n - k; i >= 0; i-- )
        {
            b = a[i];

            for ( j = i + k; j <= n && b > a[j]; j += k )
                a[j - k] = a[j];
            a[j - k] = b;
        }
    }
} /* end N_ShellSortReal */

/*******************************************************************//**


   DESCRIPTION:

     This math routine searches a real array using insertion search. A 
     typical calling example is:

       NL_REAL   *a, t;
       NL_INDEX  n, k;
       ...
       (get array a);
       ...
       N_SearchInsertReal(a,n,t,&k);


   ACCESS:
   
     a , input  ,  Real array
     n , input  ,  Highest index in a
     t , input  ,  Element to search for
     k , output ,  Index of element found:
                     >= 0 : t is found
                     <  0 : t is not found


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SearchInsertReal( NL_REAL *a, NL_INDEX n, NL_REAL t, NL_INDEX *k )
{

    NL_INDEX l, m, r;

    NL_REAL f;

    l = 0;
    r = n;
    *k = -1;

    while( r GE l )
    {
        f = (t - a[l]) / (a[r] - a[l]);
        m = (NL_INDEX)( (NL_REAL)l + f * ((NL_REAL)r - (NL_REAL)l) );

        if( fabs( a[m] - t )LT NL_PTOL )
        {
            *k = m;
            break;
        }

        if( t LT a[m] )
            r = m - 1;
        else
            l = m + 1;
    }
} /* end N_SearchInsertReal */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the minimum or the maximum of a 1-D
     integer array. A typical calling example is:

       NL_INTEGER  *i, minmax;
       NL_INDEX    n;
       ...
       (get array i);
       ...
       minmax = N_MinMax1dIntArray(i,n,NL_MINIMUM);


   ACCESS:
   
     i   , input ,  Integer array
     n   , input ,  Highest index in i
     flg , input ,  Flag:
                      NL_MINIMUM: compute the minimum
                      NL_MAXIMUM: compute the maximum


   RETURN CODES:

     minmax : Minimum or the maximum of the array

   ***********************************************************************/

NL_INTEGER N_MinMax1dIntArray( NL_INTEGER *i, NL_INDEX n, NL_FLAG flg )
{

    NL_INTEGER minmax;

    NL_INDEX k;

    minmax = i[0];

    switch( flg )
    {
        case NL_MINIMUM:
            for ( k = 1; k <= n; k++ )
            {
                if( i[k]LT minmax )
                    minmax = i[k];
            }
            break;

        case NL_MAXIMUM:
            for ( k = 1; k <= n; k++ )
            {
                if( i[k]GT minmax )
                    minmax = i[k];
            }
            break;
    }

    return minmax;
} /* end N_MinMax1dIntArray */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the minimum or the maximum of a 2-D
     integer array. A typical calling example is:

       NL_INTEGER  **i, minmax;
       NL_INDEX    m, n;
       ...
       (get array i);
       ...
       minmax = N_MinMax2dIntArray(i,m,n,NL_MINIMUM);


   ACCESS:
   
     i   , input ,  Integer array
     m,n , input ,  Highest indexes in i
     flg , input ,  Flag:
                      NL_MINIMUM: compute the minimum
                      NL_MAXIMUM: compute the maximum


   RETURN CODES:

     minmax : Minimum or the maximum of the array

   ***********************************************************************/

NL_INTEGER N_MinMax2dIntArray( NL_INTEGER ** i, NL_INDEX m, NL_INDEX n, NL_FLAG flg )
{

    NL_INTEGER minmax;

    NL_INDEX k, l;

    minmax = i[0][0];

    switch( flg )
    {
        case NL_MINIMUM:
            for ( k = 0; k <= m; k++ )
            {
                for ( l = 0; l <= n; l++ )
                {
                    if( i[k][l]LT minmax )
                        minmax = i[k][l];
                }
            }
            break;

        case NL_MAXIMUM:
            for ( k = 0; k <= m; k++ )
            {
                for ( l = 0; l <= n; l++ )
                {
                    if( i[k][l]GT minmax )
                        minmax = i[k][l];
                }
            }
            break;
    }

    return minmax;
} /* end N_MinMax2dIntArray */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the minimum or the maximum of a 1-D
     real array. A typical calling example is:

       NL_REAL   *r, minmax;
       NL_INDEX  n;
       ...
       (get array r);
       ...
       minmax = N_MinMax1dRealArray(r,n,NL_MINIMUM);


   ACCESS:
   
     r   , input ,  Real array
     n   , input ,  Highest index in r
     flg , input ,  Flag:
                      NL_MINIMUM: compute the minimum
                      NL_MAXIMUM: compute the maximum


   RETURN CODES:

     minmax : Minimum or the maximum of the array

   ***********************************************************************/

NL_REAL N_MinMax1dRealArray( NL_REAL *r, NL_INDEX n, NL_FLAG flg )
{

    NL_REAL minmax;

    NL_INDEX k;

    minmax = r[0];

    switch( flg )
    {
        case NL_MINIMUM:
            for ( k = 1; k <= n; k++ )
            {
                if( r[k]LT minmax )
                    minmax = r[k];
            }
            break;

        case NL_MAXIMUM:
            for ( k = 1; k <= n; k++ )
            {
                if( r[k]GT minmax )
                    minmax = r[k];
            }
            break;
    }

    return minmax;
} /* end N_MinMax1dRealArray */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the minimum or the maximum of a 2-D
     real array. A typical calling example is:

       NL_REAL    **r, minmax;
       NL_INDEX   m, n;
       ...
       (get array r);
       ...
       minmax = N_MinMax2dRealArray(r,m,n,NL_MINIMUM);


   ACCESS:
   
     r   , input ,  Real array
     m,n , input ,  Highest indexes in r
     flg , input ,  Flag:
                      NL_MINIMUM: compute the minimum
                      NL_MAXIMUM: compute the maximum


   RETURN CODES:

     minmax : Minimum or the maximum of the array

   ***********************************************************************/

NL_REAL N_MinMax2dRealArray( NL_REAL ** r, NL_INDEX m, NL_INDEX n, NL_FLAG flg )
{

    NL_REAL minmax;

    NL_INDEX k, l;

    minmax = r[0][0];

    switch( flg )
    {
        case NL_MINIMUM:
            for ( k = 0; k <= m; k++ )
            {
                for ( l = 0; l <= n; l++ )
                {
                    if( r[k][l]LT minmax )
                        minmax = r[k][l];
                }
            }
            break;

        case NL_MAXIMUM:
            for ( k = 0; k <= m; k++ )
            {
                for ( l = 0; l <= n; l++ )
                {
                    if( r[k][l]GT minmax )
                        minmax = r[k][l];
                }
            }
            break;
    }

    return minmax;
} /* end N_MinMax2dRealArray */

/*******************************************************************//**


   DESCRIPTION:

     This math routine swaps two points. A typical calling example is:

       NL_POINT  P, Q;
       ...
       (get P and Q);
       ...
       N_SwapPts(&P,&Q);


   ACCESS:
   
     P  , input  ,  First point
     Q  , input  ,  Second point


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SwapPts( NL_POINT *P, NL_POINT *Q )
{

    NL_POINT A;

    N_CopyPt( *P, &A );
    N_CopyPt( *Q, P );
    N_CopyPt( A, Q );
} /* end N_SwapPts */

/*******************************************************************//**


   DESCRIPTION:

     This math routine swaps two control points. A typical calling 
     example is:

       NL_CPOINT  Pw, Qw;
       ...
       (get Pw and Qw);
       ...
       N_SwapCPts(&Pw,&Qw);


   ACCESS:
   
     Pw  , input  ,  First control point
     Qw  , input  ,  Second control point


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SwapCPts( NL_CPOINT *Pw, NL_CPOINT *Qw )
{

    NL_CPOINT Aw;

    N_CopyCPt( *Pw, &Aw );
    N_CopyCPt( *Qw, Pw );
    N_CopyCPt( Aw, Qw );
} /* end N_SwapCPts */

/*******************************************************************//**


   DESCRIPTION:

     This math routine  computes the i-th  row in the Pascal triangle
     from the  previous row IN PLACE, i.e. it overwrites the elements
     in the array. Because of the symmetry of the triangle, only half
     of the  elements need to be  computed. A typical calling example 
     is:

       NL_INTEGER  *row;
       NL_INDEX    i;
       ...
       (allocate memory for row);
       ...
       N_PascalTriIndex(row,i);

     IT IS  ASSUMED THAT  ENOUGH MEMORY  IS  ALLOCATED IN THE CALLING 
     ROUTINE TO HOLD ALL THE ENTRIES OF THE I-TH ROW.


   ACCESS:
   
     row , in/out ,  Array to hold the elements of the i-th row  
     i   , input  ,  Index of the i-th row


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PascalTriIndex( NL_INTEGER *row, NL_INDEX i )
{

    NL_INTEGER tmp1, tmp2;

    NL_INDEX j, r;

    /* Special cases for the first two rows */

    if( i EQ 0 )
    {
        row[0] = 1;
        return;
    }

    if( i EQ 1 )
    {
        row[0] = 1;
        row[1] = 1;
        return;
    }

    /* Compute the i-th row from the previous one */

    r = i / 2;
    tmp2 = 1;

    for ( j = 1; j <= r; j++ )
    {
        tmp1 = row[j];
        row[j] = row[j] + tmp2;
        row[i - j] = row[j];
        tmp2 = tmp1;
    }

    row[i] = 1;
} /* end N_PascalTriIndex */


/*******************************************************************//**


   DESCRIPTION:

     This  math  routine computes   the Pascal triangle up to a given
     row index. Because of the symmetry of the triangle, only half of 
     the elements need to be  computed. A typical calling example is:

       NL_INTEGER  **tri;
       NL_INDEX    row;
       ...
       (allocate memory for tri);
       ...
       N_PascalTriRow(tri,row);

     IT IS ASSUMED THAT ENOUGH MEMORY  IS ALLOCATED IN  THE 2-D ARRAY 
     tri TO HOLD ALL THE ENTRIES OF THE TRIANGLE.


   ACCESS:
   
     tri , output ,  Pascal triangle
     row , input  ,  Highest row index

   USER NOTE:

     This routine has been replaced by a global array called
     NL_PascalTri.


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PascalTriRow( NL_INTEGER ** tri, NL_INDEX row )
{

    NL_INTEGER tmp1, tmp2;

    NL_INDEX j, k, r;

    /* Special cases for the first two rows */

    tri[0][0] = 1;

    if( row EQ 0 )
        return;

    tri[1][0] = 1;
    tri[1][1] = 1;

    if( row EQ 1 )
        return;

    /* Compute the full triangle */

    for ( k = 2; k <= row; k++ )
    {
        tri[k][0] = 1;
        r = k / 2;
        tmp2 = 1;

        for ( j = 1; j <= r; j++ )
        {
            tmp1 = tri[k - 1][j];
            tri[k][j] = tri[k - 1][j] + tmp2;
            tri[k][k - j] = tri[k][j];
            tmp2 = tmp1;
        }
        tri[k][k] = 1;
    }
} /* end N_PascalTriRow */

/*******************************************************************//**


   DESCRIPTION:

     This  math  routine  initializes  an integer  matrix to NULL. It 
     is used to  check if memory  allocation is  needed, i.e. if  the 
     matrix  is  initialized  to NULL, memory  is  allocated to store
     matrix elements. Otherwise  it is assumed that memory allocation
     is not necessary. A typical calling example is:

       NL_IMATRIX  ima;
       ...
       N_InitIntMatrix(&ima);
     

   ACCESS:
   
     ima , in/out ,  Integer matrix


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_InitIntMatrix( NL_IMATRIX *ima )
{

    ima->n = -1;
    ima->m = -1;
    ima->IM = NULL;
    ima->bw = 0;
} /* end N_InitIntMatrix */


/*******************************************************************//**


   DESCRIPTION:

     This  math routine  initializes  a real matrix to NULL. It is used 
     to  check if memory  allocation is needed, i.e. if  the matrix  is  
     initialized to NULL, memory is allocated to store matrix elements. 
     Otherwise  it is assumed that memory allocation is  not necessary.
     A typical calling example is:
  
       NL_RMATRIX  rma;
       ...
       N_InitRealMatrix(&rma);
     

   ACCESS:
   
     rma , in/out ,  Real matrix


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_InitRealMatrix( NL_RMATRIX *rma )
{

    rma->n = -1;
    rma->m = -1;
    rma->RM = NULL;
    rma->bw = 0;
} /* end N_InitRealMatrix */


/*******************************************************************//**


   DESCRIPTION:

     This  math routine initializes  a point matrix to NULL. It is used 
     to check if  memory  allocation is needed, i.e. if  the matrix  is  
     initialized to NULL, memory is allocated to store matrix elements. 
     Otherwise it is assumed that memory allocation is not necessary. A
     typical calling example is:

       NL_PMATRIX  pma;
       ...
       N_InitPtMatrix(&pma);
     

   ACCESS:
   
     pma , in/out ,  Point matrix


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_InitPtMatrix( NL_PMATRIX *pma )
{

    pma->n = -1;
    pma->m = -1;
    pma->PM = NULL;
    pma->bw = 0;
} /* end N_InitPtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  initializes a control point matrix to NULL. It 
     is  used  to check if  memory allocation  is  needed, i.e. if  the 
     matrix is initialized to NULL, memory is allocated to store matrix 
     elements. Otherwise  it is  assumed that  memory allocation is not 
     necessary. A typical calling example is:

       NL_CMATRIX  cma;
       ...
       N_InitCPtMatrix(&cma);
     

   ACCESS:
   
     cma , in/out ,  Control point matrix


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_InitCPtMatrix( NL_CMATRIX *cma )
{

    cma->n = -1;
    cma->m = -1;
    cma->CM = NULL;
    cma->bw = 0;
} /* end N_InitCPtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine makes structure member definitions to create
     an integer matrix. A typical calling example is:

       NL_IMATRIX     ima;
       NL_INDEX       n, m, bw;
       NL_INTEGER     **IM;
       NL_MATRIXTYPE  mtp;
       ...
       (get n, m, bw, mtp and allocate memory for IM);
       ...
       N_CreateIntMatrix(&ima,n,m,IM,mtp,bw);
     

   ACCESS:
   
     ima  , in/out ,  Integer matrix
     n,m  , input  ,  Highest indexes
     IM   , input  ,  2-D array of elements
     mtp  , input  ,  Matrix type:
                       - NL_MT_FULL
                       - NL_MT_LOWERLEFT
                       - NL_MT_UPPERRIGHT
                       - NL_MT_BANDED
     bw   , input  ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CreateIntMatrix( NL_IMATRIX *ima, NL_INDEX n, NL_INDEX m, NL_INTEGER ** IM, NL_MATRIXTYPE mtp, NL_INDEX bw )
{

    ima->n = n;
    ima->m = m;
    ima->IM = IM;
    ima->mtp = mtp;
    ima->bw = bw;
} /* end N_CreateIntMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine makes structure member definitions to create
     a real matrix. A typical calling example is:

       NL_RMATRIX     rma;
       NL_INDEX       n, m, bw;
       NL_REAL        **RM;
       NL_MATRIXTYPE  mtp;
       ...
       (get n, m, bw, mtp and allocate memory for RM);
       ...
       N_CreateRealMatrix(&rma,n,m,RM,mtp,bw);
     

   ACCESS:
   
     rma  , in/out ,  Real matrix
     n,m  , input  ,  Highest indexes
     RM   , input  ,  2-D array of elements
     mtp  , input  ,  Matrix type:
                       - NL_MT_FULL
                       - NL_MT_LOWERLEFT
                       - NL_MT_UPPERRIGHT
                       - NL_MT_BANDED
     bw   , input  ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CreateRealMatrix( NL_RMATRIX *rma, NL_INDEX n, NL_INDEX m, NL_REAL ** RM, NL_MATRIXTYPE mtp, NL_INDEX bw )
{

    rma->n = n;
    rma->m = m;
    rma->RM = RM;
    rma->mtp = mtp;
    rma->bw = bw;
} /* end N_CreateRealMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine makes structure member definitions to create
     a point matrix. A typical calling example is:

       NL_PMATRIX     pma;
       NL_INDEX       n, m, bw;
       NL_POINT       **PM;
       NL_MATRIXTYPE  mtp;
       ...
       (get n, m, bw, mtp and allocate memory for PM);
       ...
       N_CreatePtMatrix(&pma,n,m,PM,mtp,bw);
     

   ACCESS:
   
     pma  , in/out ,  Point matrix
     n,m  , input  ,  Highest indexes
     PM   , input  ,  2-D array of elements
     mtp  , input  ,  Matrix type:
                       - NL_MT_FULL
                       - NL_MT_LOWERLEFT
                       - NL_MT_UPPERRIGHT
                       - NL_MT_BANDED
     bw   , input  ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CreatePtMatrix( NL_PMATRIX *pma, NL_INDEX n, NL_INDEX m, NL_POINT ** PM, NL_MATRIXTYPE mtp, NL_INDEX bw )
{

    pma->n = n;
    pma->m = m;
    pma->PM = PM;
    pma->mtp = mtp;
    pma->bw = bw;
} /* end N_CreatePtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine makes structure member definitions to create
     a control point matrix. A typical calling example is:

       NL_CMATRIX     cma;
       NL_INDEX       n, m, bw;
       NL_CPOINT      **CM;
       NL_MATRIXTYPE  mtp;
       ...
       (get n, m, bw, mtp and allocate memory for CM);
       ...
       N_CreateCPtMatrix(&cma,n,m,CM,mtp,bw);
     

   ACCESS:
   
     cma  , in/out ,  Control point matrix
     n,m  , input  ,  Highest indexes
     CM   , input  ,  2-D array of elements
     mtp  , input  ,  Matrix type:
                       - NL_MT_FULL
                       - NL_MT_LOWERLEFT
                       - NL_MT_UPPERRIGHT
                       - NL_MT_BANDED
     bw   , input  ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CreateCPtMatrix( NL_CMATRIX *cma, NL_INDEX n, NL_INDEX m, NL_CPOINT ** CM, NL_MATRIXTYPE mtp, NL_INDEX bw )
{

    cma->n = n;
    cma->m = m;
    cma->CM = CM;
    cma->mtp = mtp;
    cma->bw = bw;
} /* end N_CreateCPtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine sets the  indexes, the matrix  type and the 
     bandwidth of an integer matrix. A typical calling example is:

       NL_IMATRIX     ima;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       ...
       (get n, m, bw and mtp);
       ...
       N_IntMatrixDefine(&ima,n,m,mtp,bw);
     

   ACCESS:
   
     ima  , in/out ,  Integer matrix
     n,m  , input  ,  Highest indexes
     mtp  , input  ,  Matrix type:
                       - NL_MT_FULL
                       - NL_MT_LOWERLEFT
                       - NL_MT_UPPERRIGHT
                       - NL_MT_BANDED
     bw   , input  ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_IntMatrixDefine( NL_IMATRIX *ima, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw )
{

    ima->n = n;
    ima->m = m;
    ima->mtp = mtp;
    ima->bw = bw;
} /* end N_IntMatrixDefine */

/*******************************************************************//**


   DESCRIPTION:

     This math routine sets the indexes, the matrix type and the 
     bandwidth of a real matrix. A typical calling example is:

       NL_RMATRIX     rma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       ...
       (get n, m, bw and mtp);
       ...
       N_RealMatrixDefine(&rma,n,m,mtp,bw);
     

   ACCESS:
   
     rma  , in/out ,  Real matrix
     n,m  , input  ,  Highest indexes
     mtp  , input  ,  Matrix type:
                       - NL_MT_FULL
                       - NL_MT_LOWERLEFT
                       - NL_MT_UPPERRIGHT
                       - NL_MT_BANDED
     bw   , input  ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_RealMatrixDefine( NL_RMATRIX *rma, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw )
{

    rma->n = n;
    rma->m = m;
    rma->mtp = mtp;
    rma->bw = bw;
} /* end N_RealMatrixDefine */

/*******************************************************************//**


   DESCRIPTION:

     This math routine sets the indexes, the matrix type and the 
     bandwidth of a point matrix. A typical calling example is:

       NL_PMATRIX     pma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       ...
       (get n, m, bw and mtp);
       ...
       N_PtMatrixDefine(&pma,n,m,mtp,bw);
     

   ACCESS:
   
     pma  , in/out ,  Point matrix
     n,m  , input  ,  Highest indexes
     mtp  , input  ,  Matrix type:
                       - NL_MT_FULL
                       - NL_MT_LOWERLEFT
                       - NL_MT_UPPERRIGHT
                       - NL_MT_BANDED
     bw   , input  ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PtMatrixDefine( NL_PMATRIX *pma, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw )
{

    pma->n = n;
    pma->m = m;
    pma->mtp = mtp;
    pma->bw = bw;
} /* end N_PtMatrixDefine */

/*******************************************************************//**


   DESCRIPTION:

     This  math  routine  sets the  indexes, the  matrix  type  and the 
     bandwidth of a control point matrix. A typical calling example is:

       NL_CMATRIX     cma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       ...
       (get n, m, bw and mtp);
       ...
       N_CPtMatrixDefine(&cma,n,m,mtp,bw);
     

   ACCESS:
   
     cma  , in/out ,  Control point matrix
     n,m  , input  ,  Highest indexes
     mtp  , input  ,  Matrix type:
                       - NL_MT_FULL
                       - NL_MT_LOWERLEFT
                       - NL_MT_UPPERRIGHT
                       - NL_MT_BANDED
     bw   , input  ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CPtMatrixDefine( NL_CMATRIX *cma, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw )
{

    cma->n = n;
    cma->m = m;
    cma->mtp = mtp;
    cma->bw = bw;
} /* end N_CPtMatrixDefine */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the members of an integer matrix object. A
     typical calling example is as follows:

       NL_IMATRIX     ima;
       NL_INDEX       n, m, bw;
       NL_INTEGER     **IM;
       NL_MATRIXTYPE  mtp;
       ...
       (define ima);
       ...
       N_GetIntMatrixData(&ima,&n,&m,&IM,&mtp,&bw);


   ACCESS:
   
     ima  , input  ,  Integer matrix
     n,m  , output ,  Highest indexes
     IM   , output ,  2-D array of elements
     mtp  , output ,  Matrix type
     bw   , output ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetIntMatrixData( NL_IMATRIX *ima, NL_INDEX *n, NL_INDEX *m, NL_INTEGER *** IM, NL_MATRIXTYPE *mtp, NL_INDEX *bw )
{

    *n = ima->n;
    *m = ima->m;
    *IM = ima->IM;
    *mtp = ima->mtp;
    *bw = ima->bw;
} /* end N_GetIntMatrixData */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the members of a real matrix object. A
     typical calling example is as follows:

       NL_RMATRIX     rma;
       NL_INDEX       n, m, bw;
       NL_REAL        **RM;
       NL_MATRIXTYPE  mtp;
       ...
       (define rma);
       ...
       N_GetRealMatrixData(&rma,&n,&m,&RM,&mtp,&bw);


   ACCESS:
   
     rma  , input  ,  Real matrix
     n,m  , output ,  Highest indexes
     RM   , output ,  2-D array of elements
     mtp  , output ,  Matrix type
     bw   , output ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetRealMatrixData( NL_RMATRIX *rma, NL_INDEX *n, NL_INDEX *m, NL_REAL *** RM, NL_MATRIXTYPE *mtp, NL_INDEX *bw )
{

    *n = rma->n;
    *m = rma->m;
    *RM = rma->RM;
    *mtp = rma->mtp;
    *bw = rma->bw;
} /* end N_GetRealMatrixData */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the members of a point matrix object. A
     typical calling example is as follows:

       NL_PMATRIX     pma;
       NL_INDEX       n, m, bw;
       NL_POINT       **PM;
       NL_MATRIXTYPE  mtp;
       ...
       (define pma);
       ...
       N_GetPtMatrixData(&pma,&n,&m,&PM,&mtp,&bw);


   ACCESS:
   
     pma  , input  ,  Point matrix
     n,m  , output ,  Highest indexes
     PM   , output ,  2-D array of elements
     mtp  , output ,  Matrix type
     bw   , output ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetPtMatrixData( NL_PMATRIX *pma, NL_INDEX *n, NL_INDEX *m, NL_POINT *** PM, NL_MATRIXTYPE *mtp, NL_INDEX *bw )
{

    *n = pma->n;
    *m = pma->m;
    *PM = pma->PM;
    *mtp = pma->mtp;
    *bw = pma->bw;
} /* end N_GetPtMatrixData */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the members of a control point matrix 
     object. A typical calling example is as follows:

       NL_CMATRIX     cma;
       NL_INDEX       n, m, bw;
       NL_CPOINT      **CM;
       NL_MATRIXTYPE  mtp;
       ...
       (define cma);
       ...
       N_GetCPtMatrixData(&cma,&n,&m,&CM,&mtp,&bw);


   ACCESS:
   
     cma  , input  ,  Control point matrix
     n,m  , output ,  Highest indexes
     CM   , output ,  2-D array of elements
     mtp  , output ,  Matrix type
     bw   , output ,  Bandwidth


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetCPtMatrixData( NL_CMATRIX *cma, NL_INDEX *n, NL_INDEX *m, NL_CPOINT *** CM, NL_MATRIXTYPE *mtp, NL_INDEX *bw )
{

    *n = cma->n;
    *m = cma->m;
    *CM = cma->CM;
    *mtp = cma->mtp;
    *bw = cma->bw;
} /* end N_GetCPtMatrixData */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the highest indexes of an integer matrix 
     object. A typical calling example is as follows:

       NL_IMATRIX  ima;
       NL_INDEX    n, m;
       ...
       (define ima);
       ...
       N_GetMaxIndexIntMatrix(&ima,&n,&m);


   ACCESS:
   
     ima  , input  ,  Integer matrix
     n,m  , output ,  Highest indexes


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetMaxIndexIntMatrix( NL_IMATRIX *ima, NL_INDEX *n, NL_INDEX *m )
{

    *n = ima->n;
    *m = ima->m;
} /* end N_GetMaxIndexIntMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the highest indexes of a real matrix object. 
     A typical calling example is as follows:

       NL_RMATRIX  rma;
       NL_INDEX    n, m;
       ...
       (define rma);
       ...
       N_GetMaxIndexRealMatrix(&rma,&n,&m);


   ACCESS:
   
     rma  , input  ,  Real matrix
     n,m  , output ,  Highest indexes


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetMaxIndexRealMatrix( NL_RMATRIX *rma, NL_INDEX *n, NL_INDEX *m )
{

    *n = rma->n;
    *m = rma->m;
} /* end N_GetMaxIndexRealMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the highest indexes of a point matrix 
     object. A typical calling example is as follows:

       NL_PMATRIX  pma;
       NL_INDEX    n, m;
       ...
       (define pma);
       ...
       N_GetMaxIndexPtMatrix(&pma,&n,&m);


   ACCESS:
   
     pma  , input  ,  Point matrix
     n,m  , output ,  Highest indexes


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetMaxIndexPtMatrix( NL_PMATRIX *pma, NL_INDEX *n, NL_INDEX *m )
{

    *n = pma->n;
    *m = pma->m;
} /* end N_GetMaxIndexPtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the highest indexes of a control point 
     matrix object. A typical calling example is as follows:

       NL_CMATRIX  cma;
       NL_INDEX    n, m;
       ...
       (define cma);
       ...
       N_GetMaxIndexCPtMatrix(&cma,&n,&m);


   ACCESS:
   
     cma  , input  ,  Control point matrix
     n,m  , output ,  Highest indexes


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetMaxIndexCPtMatrix( NL_CMATRIX *cma, NL_INDEX *n, NL_INDEX *m )
{

    *n = cma->n;
    *m = cma->m;
} /* end N_GetMaxIndexCPtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the matrix pointer of an integer matrix 
     object. A typical calling example is as follows:

       NL_IMATRIX  ima;
       NL_INTEGER  **IM;
       ...
       (define ima);
       ...
       N_GetIntMatrixPtr(&ima,&IM);


   ACCESS:
   
     ima  , input  ,  Integer matrix
     IM   , output ,  2-D array of elements


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetIntMatrixPtr( NL_IMATRIX *ima, NL_INTEGER *** IM )
{

    *IM = ima->IM;
} /* end N_GetIntMatrixPtr */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the matrix pointer of a real matrix object. 
     A typical calling example is as follows:

       NL_RMATRIX  rma;
       NL_REAL     **RM;
       ...
       (define rma);
       ...
       N_GetRealMatrixPtr(&rma,&RM);


   ACCESS:
   
     rma  , input  ,  Real matrix
     RM   , output ,  2-D array of elements


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetRealMatrixPtr( NL_RMATRIX *rma, NL_REAL *** RM )
{

    *RM = rma->RM;
} /* end N_GetRealMatrixPtr */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the matrix pointer of a point matrix object. 
     A typical calling example is as follows:

       NL_PMATRIX  pma;
       NL_POINT    **PM;
       ...
       (define pma);
       ...
       N_GetPtMatrixPtr(&pma,&PM);


   ACCESS:
   
     pma  , input  ,  Point matrix
     PM   , output ,  2-D array of elements


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetPtMatrixPtr( NL_PMATRIX *pma, NL_POINT *** PM )
{

    *PM = pma->PM;
} /* end N_GetPtMatrixPtr */

/*******************************************************************//**


   DESCRIPTION:

     This math routine gets the matrix pointer of a control point 
     matrix object. A typical calling example is as follows:

       NL_CMATRIX  cma;
       NL_CPOINT   **CM;
       ...
       (define cma);
       ...
       N_GetCPtMatrixPtr(&cma,&CM);


   ACCESS:
   
     cma  , input  ,  Point matrix
     CM   , output ,  2-D array of elements


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetCPtMatrixPtr( NL_CMATRIX *cma, NL_CPOINT *** CM )
{

    *CM = cma->CM;
} /* end N_GetCPtMatrixPtr */

/*******************************************************************//**


   DESCRIPTION:

     This math routine prints real matrix data to the standard output. 
     A typical calling example is as follows:

       NL_RMATRIX  rma;
       ...
       (get rma);
       ...
       N_PrintRealMatrix(&rma);


   ACCESS:
   
     rma , input ,  Real matrix


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PrintRealMatrix( NL_RMATRIX *rma )
{

    NL_INDEX i, j, n, m, bw;

    NL_REAL ** RM;

    NL_MATRIXTYPE mtp;

    /* Get local notation */

    N_GetRealMatrixData( rma, &n, &m, &RM, &mtp, &bw );

    /* Print matrix data */

    N_FPRINTF( stdout, _T("%ld %ld\n"), n, m );

    switch( mtp )
    {
        case NL_MT_FULL:
            N_FPRINTF( stdout, _T("full      \n") );
            break;

        case NL_MT_LOWERLEFT:
            N_FPRINTF( stdout, _T("lowerleft \n") );
            break;

        case NL_MT_UPPERRIGHT:
            N_FPRINTF( stdout, _T("upperright\n") );
            break;

        case NL_MT_BANDED:
            N_FPRINTF( stdout, _T("banded    \n") );
            break;
    }
    N_FPRINTF( stdout, _T("%ld\n"), bw );

    switch( mtp )
    {
        case NL_MT_FULL:
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
                    N_FPRINTF( stdout, _T("%lf "), RM[i][j] );
                }
                N_FPRINTF( stdout, _T("\n") );
            }
            break;

        case NL_MT_LOWERLEFT:
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= i; j++ )
                {
                    N_FPRINTF( stdout, _T("%lf "), RM[i][j] );
                }
                N_FPRINTF( stdout, _T("\n") );
            }
            break;

        case NL_MT_UPPERRIGHT:
            for ( i = 0; i <= n; i++ )
            {
                for ( j = i; j <= m; j++ )
                {
                    N_FPRINTF( stdout, _T("%lf "), RM[i][j] );
                }
                N_FPRINTF( stdout, _T("\n") );
            }
            break;

        case NL_MT_BANDED:
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j < bw; j++ )
                {
                    N_FPRINTF( stdout, _T("%lf "), RM[i][j] );
                }
                N_FPRINTF( stdout, _T("\n") );
            }
            break;
    }
} /* end N_PrintRealMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine checks if an integer matrix is initialized to
     NULL. If yes, NL_TRUE is returned.  Otherwise, NL_FALSE  is returned. 
     The routine is used to check if memory  allocation is needed. A
     typical calling example is:

       NL_IMATRIX  ima;
       ...
       (initialize ima or allocate memory for ima);
       ...
       if( N_IntMatrixIsNULL(&ima) ) --> allocate memory;
     

   ACCESS:
   
     ima , input ,  Integer matrix


   RETURN CODES:

     NL_TRUE:  Matrix is initialized to NULL (need memory)
     NL_FALSE: Matrix is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

/* NL_BOOLEAN  N_IntMatrixIsNULL */
NL_BOOLEAN N_IntMatrixIsNULL( NL_IMATRIX *ima )
{

    NL_INDEX n, m, bw;

    NL_INTEGER ** IM;

    NL_MATRIXTYPE mtp;

    /* Get local notation */

    N_GetIntMatrixData( ima, &n, &m, &IM, &mtp, &bw );

    /* Check initialization */

    if( n LT 0 OR m LT 0 OR IM EQ NULL OR bw LE 0 )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_IntMatrixIsNULL */

/*******************************************************************//**


   DESCRIPTION:

     This math routine  checks if a real matrix is initialized to NULL.
     If  yes,  NL_TRUE  is  returned.  Otherwise,  NL_FALSE  is returned. The 
     routine is used to check if memory allocation is needed. A typical
     calling example is:

       NL_RMATRIX  rma;
       ...
       (initialize rma or allocate memory for rma);
       ...
       if( N_RealMatrixIsNULL(&rma) )  --> allocate memory;
     

   ACCESS:
   
     rma , input ,  Real matrix


   RETURN CODES:

     NL_TRUE:  Matrix is initialized to NULL (need memory)
     NL_FALSE: Matrix is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

/* NL_BOOLEAN  N_RealMatrixIsNULL */
NL_BOOLEAN N_RealMatrixIsNULL( NL_RMATRIX *rma )
{

    NL_INDEX n, m, bw;

    NL_REAL ** RM;

    NL_MATRIXTYPE mtp;

    /* Get local notation */

    N_GetRealMatrixData( rma, &n, &m, &RM, &mtp, &bw );

    /* Check initialization */

    if( n LT 0 OR m LT 0 OR RM EQ NULL OR bw LE 0 )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_RealMatrixIsNULL */

/*******************************************************************//**


   DESCRIPTION:

     This math routine checks if a point matrix is initialized to NULL. 
     If  yes,  NL_TRUE  is  returned. Otherwise,  NL_FALSE  is  returned. The 
     routine is used to check if memory allocation is needed. A typical
     calling example is:

       NL_PMATRIX  pma;
       ...
       (initialize pma or allocate memory for pma);
       ...
       if( N_PtMatrixIsNULL(&pma) )  --> allocate memory;
     

   ACCESS:
   
     pma , input ,  Point matrix


   RETURN CODES:

     NL_TRUE:  Matrix is initialized to NULL (need memory)
     NL_FALSE: Matrix is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

NL_BOOLEAN N_PtMatrixIsNULL( NL_PMATRIX *pma )
{

    NL_INDEX n, m, bw;

    NL_POINT ** PM;

    NL_MATRIXTYPE mtp;

    /* Get local notation */

    N_GetPtMatrixData( pma, &n, &m, &PM, &mtp, &bw );

    /* Check initialization */

    if( n LT 0 OR m LT 0 OR PM EQ NULL OR bw LE 0 )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_PtMatrixIsNULL */

/*******************************************************************//**


   DESCRIPTION:

     This math routine checks if a control point matrix is initialized 
     to NULL. If yes, NL_TRUE is returned. Otherwise, NL_FALSE  is returned. 
     The routine is used  to check if  memory allocation is  needed. A
     typical calling example is:
 
       NL_CMATRIX  cma;
       ...
       (initialize cma or allocate memory for cma);
       ...
       if( M_iscma(&cma) )  --> allocate memory;
     

   ACCESS:
   
     cma , input ,  Control point matrix


   RETURN CODES:

     NL_TRUE:  Matrix is initialized to NULL (need memory)
     NL_FALSE: Matrix is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

NL_BOOLEAN N_CPtMatrixIsNULL( NL_CMATRIX *cma )
{

    NL_INDEX n, m, bw;

    NL_CPOINT ** CM;

    NL_MATRIXTYPE mtp;

    /* Get local notation */

    N_GetCPtMatrixData( cma, &n, &m, &CM, &mtp, &bw );

    /* Check initialization */

    if( n LT 0 OR m LT 0 OR CM EQ NULL OR bw LE 0 )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_CPtMatrixIsNULL */

/*******************************************************************//**


   DESCRIPTION:

     This math routine creates a hash table for reals. More precisely,
     it allocates memory for the  table and sets all pointers to NULL.
     A typical calling example is:

       NL_REAL    **H;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get n);
       ...
       N_InitRealHash(&H,n,&S);


   ACCESS:
   
     H , in/out ,  Hash table (array of pointers)
     n , input  ,  Highest index in H
     S , input  ,  H's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_InitRealHash( NL_REAL *** H, NL_INDEX n, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL ** G;

    /* Allocate memory and initialize */

    G = N_AllocRealPtr1dArray( n, S );

    if( G EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
        G[i] = NULL;

    *H = G;

    /* Exit */

    EXIT:

    return (error);
} /* end N_InitRealHash */

/*******************************************************************//**


   DESCRIPTION:

     This math routine updates a hash  table by adding an elemnt to it. 
     More precisely, given a hash table H, and a real u (us<=u<=ue). It 
     hashes u into a position, say H[i], and does the following:

       (1) if H[i]  = NULL, u is placed into H[i][0]
       (2) if H[i] != NULL, the  routine checks if u is already at some
           position, say  H[i][k]. If it is, u is ignored. If not, u is
           placed into a  position, say  H[i][l], so that  all elements 
           H[i][0],...,H[i][r], r highest index, are maintained in sort
           order.

     A typical calling example is:

       NL_REAL    **H, us, u, ue;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get n);
       ...
       N_UpdateRealHash(H,n,us,u,ue,&m,&S);

     THE HASH TABLE H MUST  BE CREATED IN THE CALLING  ROUTINE. USE THE
     ROUTINE "N_InitRealHash.c" TO INITIALIZE (CREATE) THE TABLE.


   ACCESS:
   
     H  , in/out ,  Hash table (array of pointers)
     n  , input  ,  Highest index in H
     us , input  ,  Left real bound
     u  , input  ,  Real to be hashed (us<=u<=ue)
     ue , input  ,  Right real bound
     m  , output ,  Counter; incremented if elements are added
     S  , input  ,  H's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_UpdateRealHash( NL_REAL ** H, NL_INDEX n, NL_REAL us, NL_REAL u, NL_REAL ue, NL_INDEX *m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_UpdateRealHash");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, s;

    NL_REAL fac;

    /* Hash u into some index */

    if( u LT us OR u GT ue )
        NL_ERROR( NL_INP_ERR );

    fac = (u - us) / (ue - us);
    i = (NL_INDEX)( fac * n );

    /* If no element at i, add u */

    if( H[i]EQ NULL )
    {
        H[i] = N_AllocReal1dArray( 1, S );

        if( H[i]EQ NULL )
            NL_QUIT;

        H[i][0] = u;
        H[i][1] = NL_BIGD;
        (*m)++;

        NL_OUT;
    }

    /***********************************************/
    /* Slot is already occupied -> avoid collision */
    /***********************************************/

    /* Find highest index in array at H[i] and check for coincidence */

    k = 0;

    while( H[i][k]NEQ NL_BIGD )
    {
        if( fabs( H[i][k] - u )LT NL_PTOL )
            NL_OUT;

        k++;
    }

    /* u must be added -> reallocate memory */

    error = N_Realloc1dRealArray( &H[i], k, k + 1, S );

    if( error EQ NL_YES )
        NL_OUT;

    /* Add u while maintaining elements in sort order */

    for ( s = 0; s < k; s++ )
    {
        if( u LT H[i][s] )
            break;
    }

    for ( j = k; j > s; j-- )
        H[i][j] = H[i][j - 1];

    H[i][s] = u;
    H[i][k + 1] = NL_BIGD;

    (*m)++;

    /* Exit */

    EXIT:

    return (error);
} /* end N_UpdateRealHash */

/*******************************************************************//**


   DESCRIPTION:

     This math routine retrieves all elements from a hash table for 
     reals. A typical calling example is:

       NL_REAL   **H, *u;
       NL_INDEX  n;
       ...
       (get H);
       ...
       N_RetrieveRealHash(H,n,u);

     MEMORY TO STORE THE ELEMENTS IN  u  MUST  BE ALLOCATED  IN THE 
     CALLING ROUTINE.


   ACCESS:
   
     H , input  ,  Hash table (array of pointers)
     n , input  ,  Highest index in H
     u , output ,  Elements in H 


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_RetrieveRealHash( NL_REAL ** H, NL_INDEX n, NL_REAL *u )
{

    NL_INDEX i, j, k;

    /* Retrieve elements */

    k = 0;

    for ( i = 0; i <= n; i++ )
    {
        if( H[i]NEQ NULL )
        {
            j = 0;

            while( H[i][j]NEQ NL_BIGD )
                u[k++] = H[i][j++];
        }
    }
} /* end N_RetrieveRealHash */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     Given an integer matrix object, this routine  allocates memory to 
     store matrix elements. A typical calling example is:
 
       NL_IMATRIX     ima;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STACKS      S;
       ...
       N_SetIntMatrix(&ima,n,m,mtp,bw,&S);
 
     Since the declaration  "NL_IMATRIX ima"  defines  the data type  and 
     allocates memory, memory is needed to store matrix elements only.
 
 
   ACCESS:
   
     ima , in/out ,  Integer matrix
     n,m , input  ,  Highest indexes
     mtp , input  ,  Matrix type :
                      - NL_MT_FULL
                      - NL_MT_LOWERLEFT
                      - NL_MT_UPPERRIGHT
                      - NL_MT_BANDED
     bw  , input  ,  Bandwidth
     S   , input  ,  Memory stacks pointer
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_SetIntMatrix( NL_IMATRIX *ima, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STACKS *S )
{

    NL_INTEGER ** IM;

    /* Allocate memory */

    if( mtp EQ NL_MT_BANDED )
    {
        if( bw EQ 1 )
            IM = N_AllocInt2dArray( n, bw, S );
        else
            IM = N_AllocInt2dArray( n, bw - 1, S );
    }
    else
    {
        IM = N_AllocInt2dArray( n, m, S );
    }

    if( IM EQ NULL )
        return (1);

    /* Build matrix structure */

    N_CreateIntMatrix( ima, n, m, IM, mtp, bw );

    /* Exit */

    return (0);
} /* end N_SetIntMatrix */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     Given a real  matrix object, this  routine  allocates  memory to 
     store matrix elements. A typical calling example is:
 
       NL_RMATRIX     rma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STACKS      S;
       ...
       N_SetRealMatrix(&rma,n,m,mtp,bw,&S);
 
     Since the declaration  "NL_RMATRIX rma"  defines  the data type  and 
     allocates memory, memory is needed to store matrix elements only.
 
 
   ACCESS:
   
     rma , in/out ,  Real matrix
     n,m , input  ,  Highest indexes
     mtp , input  ,  Matrix type :
                      - NL_MT_FULL
                      - NL_MT_LOWERLEFT
                      - NL_MT_UPPERRIGHT
                      - NL_MT_BANDED
     bw  , input  ,  Bandwidth
     S   , input  ,  Memory stacks pointer
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_SetRealMatrix
 (NL_RMATRIX   *rma,  /* i/o: target real matrix */
  NL_INDEX      n,    /* in : highest 1st index */
  NL_INDEX      m,    /* in : highest 2nd index */
  NL_MATRIXTYPE mtp,  /* in : Matrix Type: NL_MT_FULL      */
                      /*                   NL_MT_LOWERLEFT */
                      /*                   NL_MT_UPPERRIGH */
                      /*                   NL_MT_BANDED    */
  NL_INDEX      bw,   /* in : Bandwidth */
  NL_STACKS    *S )   /* in : Memory stacks pointer */
{

    NL_REAL ** RM;

    /* Allocate memory */

    if( mtp EQ NL_MT_BANDED )
    {
        if( bw EQ 1 )
            RM = N_AllocReal2dArray( n, bw, S );
        else
            RM = N_AllocReal2dArray( n, bw - 1, S );
    }
    else
    {
        RM = N_AllocReal2dArray( n, m, S );
    }

    if( RM EQ NULL )
        return (1);

    /* Build matrix structure */

    N_CreateRealMatrix( rma, n, m, RM, mtp, bw );

    /* Exit */

    return (0);
} /* end N_SetRealMatrix */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     Given a point matrix object, this  routine  allocates  memory to 
     store matrix elements. A typical calling example is:
 
       NL_PMATRIX     pma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STACKS      S;
       ...
       N_SetPtMatrix(&pma,n,m,mtp,bw,&S);
 
     Since the declaration  "NL_PMATRIX pma"  defines  the data type  and 
     allocates memory, memory is needed to store matrix elements only.
 
 
   ACCESS:
   
     pma , in/out ,  Point matrix
     n,m , input  ,  Highest indexes
     mtp , input  ,  Matrix type :
                      - NL_MT_FULL
                      - NL_MT_LOWERLEFT
                      - NL_MT_UPPERRIGHT
                      - NL_MT_BANDED
     bw  , input  ,  Bandwidth
     S   , input  ,  Memory stacks pointer
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_SetPtMatrix( NL_PMATRIX *pma, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STACKS *S )
{

    NL_POINT ** PM;

    /* Allocate memory */

    if( mtp EQ NL_MT_BANDED )
    {
        if( bw EQ 1 )
            PM = N_AllocPt2dArray( n, bw, S );
        else
            PM = N_AllocPt2dArray( n, bw - 1, S );
    }
    else
    {
        PM = N_AllocPt2dArray( n, m, S );
    }

    if( PM EQ NULL )
        return (1);

    /* Build matrix structure */

    N_CreatePtMatrix( pma, n, m, PM, mtp, bw );

    /* Exit */

    return (0);
} /* end N_SetPtMatrix */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     Given a  control  point  matrix object, this  routine  allocates  
     memory to store matrix elements. A typical calling example is:
 
       NL_CMATRIX     cma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STACKS      S;
       ...
       N_SetCPtMatrix(&cma,n,m,mtp,bw,&S);
 
     Since the declaration  "NL_CMATRIX cma"  defines  the data type  and 
     allocates memory, memory is needed to store matrix elements only.
 
 
   ACCESS:
   
     cma , in/out ,  Control point matrix
     n,m , input  ,  Highest indexes
     mtp , input  ,  Matrix type :
                      - NL_MT_FULL
                      - NL_MT_LOWERLEFT
                      - NL_MT_UPPERRIGHT
                      - NL_MT_BANDED
     bw  , input  ,  Bandwidth
     S   , input  ,  Memory stacks pointer
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_SetCPtMatrix( NL_CMATRIX *cma, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STACKS *S )
{

    NL_CPOINT ** CM;

    /* Allocate memory */

    if( bw EQ NL_MT_BANDED )
    {
        if( bw EQ 1 )
            CM = N_AllocCPt2dArray( n, bw, S );
        else
            CM = N_AllocCPt2dArray( n, bw - 1, S );
    }
    else
    {
        CM = N_AllocCPt2dArray( n, m, S );
    }

    if( CM EQ NULL )
        return (1);

    /* Build matrix structure */

    N_CreateCPtMatrix( cma, n, m, CM, mtp, bw );

    /* Exit */

    return (0);
} /* end N_SetCPtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine  checks if memory is needed to store an integer
     matrix. If the  matrix is initiaized to the NULL matrix  (via the
     N_InitIntMatrix()  routine), memory is  allocated. If  not, the  routine 
     checkes if enough memory is available. A typical calling  example
     is as follows:

       NL_IMATRIX     ima;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STRING      rname;
       NL_STACKS      S;
       ...
       (get n, m, bw and rname);
       ...
       N_InitIntMatrix(&ima);
       N_CheckMemIntMatrix(&ima,n,m,mtp,bw,rname,&S);

     IT IS ASSUMED THAT MEMORY TO STORE THE MATRIX STRUCTURE ITSELF IS 
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     ima   , in/out ,  Integer matrix
     n,m   , input  ,  Highest indexes
     mtp   , input  ,  Matrix type :
                        - NL_MT_FULL
                        - NL_MT_LOWERLEFT
                        - NL_MT_UPPERRIGHT
                        - NL_MT_BANDED
     bw    , input  ,  Bandwidth
     rname , input  ,  Routine name where memory check is needed
     S     , input  ,  Memory stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CheckMemIntMatrix( NL_IMATRIX *ima, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STRING rname, NL_STACKS *S )
{

    NL_FLAG error;

    /* See if memory is needed */

    if( N_IntMatrixIsNULL( ima ) )
    {
        error = N_SetIntMatrix( ima, n, m, mtp, bw, S );

        if( error EQ NL_YES )
            return (1);
    }
    else
    {
        error = N_CheckIntMatStorage( ima, n, m, rname );

        if( error EQ NL_YES )
            return (1);

        N_IntMatrixDefine( ima, n, m, mtp, bw );
    }

    /* Exit */

    return (0);
} /* end N_CheckMemIntMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine  checks if  memory is  needed  to  store a real
     matrix. If the  matrix is initiaized to the NULL matrix  (via the
     N_InitRealMatrix()  routine), memory is  allocated. If  not, the  routine 
     checkes if enough memory is available. A typical calling  example
     is as follows:

       NL_RMATRIX     rma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STRING      rname;
       NL_STACKS      S;
       ...
       (get n, m, bw and rname);
       ...
       N_InitRealMatrix(&rma);
       N_CheckMemRealMatrix(&rma,n,m,mtp,bw,rname,&S);

     IT IS ASSUMED THAT MEMORY TO STORE THE MATRIX STRUCTURE ITSELF IS 
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     rma   , in/out ,  Real matrix
     n,m   , input  ,  Highest indexes
     mtp   , input  ,  Matrix type :
                        - NL_MT_FULL
                        - NL_MT_LOWERLEFT
                        - NL_MT_UPPERRIGHT
                        - NL_MT_BANDED
     bw    , input  ,  Bandwidth
     rname , input  ,  Routine name where memory check is needed
     S     , input  ,  Memory stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CheckMemRealMatrix( NL_RMATRIX *rma, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STRING rname, NL_STACKS *S )
{

    NL_FLAG error;

    /* See if memory is needed */

    if( N_RealMatrixIsNULL( rma ) )
    {
        error = N_SetRealMatrix( rma, n, m, mtp, bw, S );

        if( error EQ NL_YES )
            return (1);
    }
    else
    {
        error = N_CheckRealMatStorage( rma, n, m, rname );

        if( error EQ NL_YES )
            return (1);

        N_RealMatrixDefine( rma, n, m, mtp, bw );
    }

    /* Exit */

    return (0);
} /* end N_CheckMemRealMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  checks  if memory is  needed to store a point
     matrix. If the  matrix is initiaized to the NULL matrix  (via the
     N_InitPtMatrix()  routine), memory is  allocated. If  not, the  routine 
     checkes if enough memory is available. A typical calling  example
     is as follows:

       NL_PMATRIX     pma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STRING      rname;
       NL_STACKS      S;
       ...
       (get n, m, bw and rname);
       ...
       N_InitPtMatrix(&pma);
       N_CheckMemPtMatrix(&pma,n,m,mtp,bw,rname,&S);

     IT IS ASSUMED THAT MEMORY TO STORE THE MATRIX STRUCTURE ITSELF IS 
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     pma   , in/out ,  Point matrix
     n,m   , input  ,  Highest indexes
     mtp   , input  ,  Matrix type :
                        - NL_MT_FULL
                        - NL_MT_LOWERLEFT
                        - NL_MT_UPPERRIGHT
                        - NL_MT_BANDED
     bw    , input  ,  Bandwidth
     rname , input  ,  Routine name where memory check is needed
     S     , input  ,  Memory stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CheckMemPtMatrix( NL_PMATRIX *pma, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STRING rname, NL_STACKS *S )
{

    NL_FLAG error;

    /* See if memory is needed */

    if( N_PtMatrixIsNULL( pma ) )
    {
        error = N_SetPtMatrix( pma, n, m, mtp, bw, S );

        if( error EQ NL_YES )
            return (1);
    }
    else
    {
        error = N_CheckPtMatStorage( pma, n, m, rname );

        if( error EQ NL_YES )
            return (1);

        N_PtMatrixDefine( pma, n, m, mtp, bw );
    }

    /* Exit */

    return (0);
} /* end N_CheckMemPtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math  routine  checks if memory is needed to store a control 
     point matrix. If the matrix is initiaized to the NULL matrix (via 
     the N_InitCPtMatrix() routine), memory is allocated. If not, the routine 
     checkes if enough memory is available. A typical calling  example
     is as follows:

       NL_CMATRIX     cma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STRING      rname;
       NL_STACKS      S;
       ...
       (get n, m, bw and rname);
       ...
       N_InitCPtMatrix(&cma);
       N_CheckMemCPtMatrix(&cma,n,m,mtp,bw,rname,&S);

     IT IS ASSUMED THAT MEMORY TO STORE THE MATRIX STRUCTURE ITSELF IS 
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     cma   , in/out ,  Control point matrix
     n,m   , input  ,  Highest indexes
     mtp   , input  ,  Matrix type :
                        - NL_MT_FULL
                        - NL_MT_LOWERLEFT
                        - NL_MT_UPPERRIGHT
                        - NL_MT_BANDED
     bw    , input  ,  Bandwidth
     rname , input  ,  Routine name where memory check is needed
     S     , input  ,  Memory stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CheckMemCPtMatrix( NL_CMATRIX *cma, NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STRING rname, NL_STACKS *S )
{

    NL_FLAG error;

    /* See if memory is needed */

    if( N_CPtMatrixIsNULL( cma ) )
    {
        error = N_SetCPtMatrix( cma, n, m, mtp, bw, S );

        if( error EQ NL_YES )
            return (1);
    }
    else
    {
        error = N_CheckCPtMatStorage( cma, n, m, rname );

        if( error EQ NL_YES )
            return (1);

        N_CPtMatrixDefine( cma, n, m, mtp, bw );
    }

    /* Exit */

    return (0);
} /* end N_CheckMemCPtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the products of two real matrices. The 
     output matrix  is either "NL_MT_LOWERLEFT" or  "NL_MT_UPPERRIGHT"  or "NL_MT_FULL". 
     If the output matrix is initiaized to  the  NULL  matrix (via the 
     N_InitRealMatrix() routine), memory  is  allocated. If  not, the  routine  
     checks if enough memory is available. A typical  calling  example
     is as follows:

       NL_RMATRIX  rma, rmb, rmc;
       NL_STACKS   S;
       ...
       (get rma and rmb);
       ...
       N_InitRealMatrix(&rmc);
       N_RealMatrixMultiply(&rma,&rmb,&rmc,&S);

     IT IS ASSUMED THAT  MEMORY TO  STORE  THE  MATRIX  STRUCTURES  IS 
     ALLOCATED IN THE CALLING ROUTINE. BANDED MATRICES ARE  CONSIDERED
     NL_FULL IN THIS ROUTINE, I.E. SPECIAL INDEXING IS NOT NL_USED.


   ACCESS:
   
     rma   , input  ,  Real matrix
     rmb   , input  ,  Real matrix
     rmc   , output ,  Product matrix, rmc = rma*rmb
     S     , input  ,  rmc's stack pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixMultiply( NL_RMATRIX *rma, NL_RMATRIX *rmb, NL_RMATRIX *rmc, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixMultiply");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, lo, hi, na, ma, nb, mb;

    NL_REAL ** A, ** B, ** C;

    NL_MATRIXTYPE mta, mtb, mtc;

    /* Get local notation */

    N_GetRealMatrixData( rma, &na, &ma, &A, &mta, &i );
    N_GetRealMatrixData( rmb, &nb, &mb, &B, &mtb, &j );

    /* Check dimensions and matrix type */

    if( ma NEQ nb )
        NL_ERROR( NL_IND_ERR );

    if( mta EQ NL_MT_BANDED OR mtb EQ NL_MT_BANDED )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_LOWERLEFT )
        mtc = NL_MT_LOWERLEFT;

    else if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_UPPERRIGHT )
        mtc = NL_MT_UPPERRIGHT;

    else
        mtc = NL_MT_FULL;

    error = N_CheckMemRealMatrix( rmc, na, mb, mtc, na, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rmc, &C );

    /* Compute matrix product */

    if( mta EQ NL_MT_FULL AND mtb EQ NL_MT_FULL )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= mb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = 0; k <= ma; k++ )
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_FULL AND mtb EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= mb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = j; k <= ma; k++ )
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_FULL AND mtb EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= mb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = 0; k <= j; k++ )
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_FULL )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= mb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = 0; k <= i; k++ )
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= i; j++ )
            {
                C[i][j] = 0.0;

                for ( k = j; k <= i; k++ )
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= mb; j++ )
            {
                hi = NL_MIN( i, j );
                C[i][j] = 0.0;

                for ( k = 0; k <= hi; k++ )
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_FULL )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= mb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = i; k <= ma; k++ )
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= mb; j++ )
            {
                lo = NL_MAX( i, j );
                C[i][j] = 0.0;

                for ( k = lo; k <= ma; k++ )
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = i; j <= mb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = i; k <= j; k++ )
                    C[i][j] += A[i][k] * B[k][j];
            }
        }
        NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixMultiply */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the product  of a real matrix  and the
     transpose of  another real  matrix. The  output matrix  is either 
     "NL_MT_LOWERLEFT"  or  "NL_MT_UPPERRIGHT" or "NL_MT_FULL". If the  output matrix is 
     initiaized to the NULL matrix (via the  N_InitRealMatrix routine), memory  
     is  allocated. If not, the  routine  checks if  enough  memory is 
     available. A typical calling example is as follows:

       NL_RMATRIX  rma, rmb, rmc;
       NL_STACKS   S;
       ...
       (get rma and rmb);
       ...
       N_InitRealMatrix(&rmc);
       N_RealMatrixMultiplyTranspose(&rma,&rmb,&rmc,&S);

     IT IS ASSUMED THAT  MEMORY TO  STORE  THE  MATRIX  STRUCTURES  IS 
     ALLOCATED IN THE CALLING ROUTINE. BANDED MATRICES ARE  CONSIDERED
     NL_FULL IN THIS ROUTINE, I.E. SPECIAL INDEXING IS NOT NL_USED.


   ACCESS:
   
     rma  , input  ,  Real matrix
     rmb  , input  ,  Real matrix
     rmc  , output ,  Product  matrix,  rmc = rma*rmb^T,  where  rmb^T
                      denotes the transpose of rmb
     S    , input  ,  rmc's stack pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixMultiplyTranspose( NL_RMATRIX *rma, NL_RMATRIX *rmb, NL_RMATRIX *rmc, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixMultiplyTranspose");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, lo, hi, na, ma, nb, mb;

    NL_REAL ** A, ** B, ** C;

    NL_MATRIXTYPE mta, mtb, mtc;

    /* Get local notation */

    N_GetRealMatrixData( rma, &na, &ma, &A, &mta, &i );
    N_GetRealMatrixData( rmb, &nb, &mb, &B, &mtb, &j );

    /* Check dimensions and matrix type */

    if( ma NEQ mb )
        NL_ERROR( NL_IND_ERR );

    if( mta EQ NL_MT_BANDED OR mtb EQ NL_MT_BANDED )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_UPPERRIGHT )
        mtc = NL_MT_LOWERLEFT;

    else if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_LOWERLEFT )
        mtc = NL_MT_UPPERRIGHT;

    else
        mtc = NL_MT_FULL;

    error = N_CheckMemRealMatrix( rmc, na, nb, mtc, na, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rmc, &C );

    /* Compute matrix product */

    if( mta EQ NL_MT_FULL AND mtb EQ NL_MT_FULL )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = 0; k <= ma; k++ )
                    C[i][j] += A[i][k] * B[j][k];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_FULL AND mtb EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = 0; k <= j; k++ )
                    C[i][j] += A[i][k] * B[j][k];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_FULL AND mtb EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = j; k <= ma; k++ )
                    C[i][j] += A[i][k] * B[j][k];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_FULL )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = 0; k <= i; k++ )
                    C[i][j] += A[i][k] * B[j][k];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                hi = NL_MIN( i, j );
                C[i][j] = 0.0;

                for ( k = 0; k <= hi; k++ )
                    C[i][j] += A[i][k] * B[j][k];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= i; j++ )
            {
                C[i][j] = 0.0;

                for ( k = j; k <= i; k++ )
                    C[i][j] += A[i][k] * B[j][k];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_FULL )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = i; k <= ma; k++ )
                    C[i][j] += A[i][k] * B[j][k];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = i; j <= nb; j++ )
            {
                C[i][j] = 0.0;

                for ( k = i; k <= j; k++ )
                    C[i][j] += A[i][k] * B[j][k];
            }
        }
        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                lo = NL_MAX( i, j );
                C[i][j] = 0.0;

                for ( k = lo; k <= ma; k++ )
                    C[i][j] += A[i][k] * B[j][k];
            }
        }
        NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixMultiplyTranspose */

/*******************************************************************//**


   DESCRIPTION:

     This math routine  computes the product of a real matrix and its
     transpose. The output  matrix is a "full" matrix. If the  output 
     matrix is initialized to  NULL, memory is allocated. If not, the 
     routine checks if enough  memory is passed in. A typical calling 
     example is as follows:

       NL_RMATRIX  rma, rmb;
       NL_STACKS   SB;
       ...
       (get rma);
       ...
       N_InitRealMatrix(&rmb);
       N_RealMatrixTransposeMultiply(&rma,&rmb,&SB);

     IT IS ASSUMED THAT  MEMORY TO  STORE  THE  MATRIX  STRUCTURES IS 
     ALLOCATED IN THE CALLING ROUTINE. BANDED MATRICES ARE CONSIDERED
     NL_FULL IN THIS ROUTINE, I.E. NL_NO SPECIAL INDEXING IS NL_USED.


   ACCESS:
   
     rma  , input  ,  Real matrix
     rmb  , output ,  Product  matrix;  rmb = rma*rma^T,  where rma^T
                      denotes the transpose of rma
     SB   , input  ,  rmb's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixTransposeMultiply( NL_RMATRIX *rma, NL_RMATRIX *rmb, NL_STACKS *SB )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixTransposeMultiply");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, lo, hi, n, m;

    NL_REAL ** A, ** B;

    NL_MATRIXTYPE mtp;

    /* Get local notation and check input */

    N_GetRealMatrixData( rma, &n, &m, &A, &mtp, &i );

    if( mtp EQ NL_MT_BANDED )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( rmb, n, n, NL_MT_FULL, n, rname, SB );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rmb, &B );

    /* Compute matrix product */

    if( mtp EQ NL_MT_FULL )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= n; j++ )
            {
                B[i][j] = 0.0;

                for ( k = 0; k <= m; k++ )
                    B[i][j] += A[i][k] * A[j][k];
            }
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= n; j++ )
            {
                hi = NL_MIN( i, j );
                B[i][j] = 0.0;

                for ( k = 0; k <= hi; k++ )
                    B[i][j] += A[i][k] * A[j][k];
            }
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= n; j++ )
            {
                lo = NL_MAX( i, j );
                B[i][j] = 0.0;

                for ( k = lo; k <= m; k++ )
                    B[i][j] += A[i][k] * A[j][k];
            }
        }

        NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixTransposeMultiply */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the transpose of a real matrix. If the 
     output  matrix  is  initiaized  to   the  NULL  matrix  (via  the 
     N_InitRealMatrix() routine), memory  is  allocated. If  not, the  routine  
     checks if enough memory is available. If the output matrix is the
     same as the imput matrix, transposition is  done in place and the
     original  matrix is  destroyed. A  typical calling  example is as 
     follows:

       NL_RMATRIX  rma, rmb;
       NL_STACKS   SA, SB;
       ...
       (get rma);
       ...
       N_InitRealMatrix(&rmb);
       N_RealMatrixTranspose(&rma,&rmb,&SA,&SB);
       N_RealMatrixTranspose(&rma,&rma,&SA,&SA);

     IT IS ASSUMED THAT  MEMORY TO  STORE  THE  MATRIX  STRUCTURES  IS 
     ALLOCATED IN THE CALLING ROUTINE. 


   ACCESS:
   
     rma , input  ,  Real matrix
     rmb , output ,  Transpose of rma
     SA  , input  ,  rma's stack
     SB  , input  ,  rmb's stack 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixTranspose( NL_RMATRIX *rma, NL_RMATRIX *rmb, NL_STACKS *SA, NL_STACKS *SB )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixTranspose");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh, n, m, bw, sbw;

    NL_REAL ** A, ** B;

    NL_MATRIXTYPE mta, mtb;

    /* Get local notation */

    N_GetRealMatrixData( rma, &n, &m, &A, &mta, &bw );

    sbw = bw / 2;

    /* See if memory is needed */

    if( mta EQ NL_MT_LOWERLEFT )
        mtb = NL_MT_UPPERRIGHT;

    else if( mta EQ NL_MT_UPPERRIGHT )
        mtb = NL_MT_LOWERLEFT;

    else if( mta EQ NL_MT_BANDED )
        mtb = NL_MT_BANDED;

    else
        mtb = NL_MT_FULL;

    if( rma EQ rmb )
    {
        if( mta EQ NL_MT_LOWERLEFT OR mta EQ NL_MT_UPPERRIGHT )
        {
            B = A;
        }
        else
        {
            B = N_AllocReal2dArray( m, n, SB );

            if( B EQ NULL )
                NL_QUIT;
        }

        N_CreateRealMatrix( rma, m, n, B, mtb, bw );
    }
    else
    {
        error = N_CheckMemRealMatrix( rmb, m, n, mtb, bw, rname, SB );

        if( error EQ NL_YES )
            NL_OUT;

        N_GetRealMatrixPtr( rmb, &B );
    }

    /* Compute transpose of matrix */

    if( mta EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = i; j <= m; j++ )
            {
                B[i][j] = A[j][i];
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT )
    {
        for ( j = 0; j <= m; j++ )
        {
            for ( i = j; i <= n; i++ )
            {
                B[i][j] = A[j][i];
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_BANDED )
    {
        for ( i = 0; i <= n; i++ )
        {
            jl = NL_MAX( 0, i - sbw );
            jh = NL_MIN( n, i + sbw );

            for ( j = jl; j <= jh; j++ )
            {
                B[j][i - j + sbw] = A[i][j - i + sbw];
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_FULL )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
            {
                B[j][i] = A[i][j];
            }
        }

        NL_OUT;
    }

    /* If in-place transposition, kill memory */

    if( rma EQ rmb AND mta EQ NL_MT_FULL )
    {
        N_FreeReal2dArray( A, SA );
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixTranspose */

/*******************************************************************//**


   DESCRIPTION:

     This math routine  computes the  product  of a real  matrix and a
     point array, i.e. it computes Q = rma * P, where Q is a new array 
     of points. A typical calling example is as follows:

       NL_RMATRIX  rma;
       NL_POINT    *P, *Q;
       ...
       (get P, rma and allocate memory for Q);
       ...
       N_RealMatrixMultiplyPtArray(&rma,P,Q);

     IT IS ASSUMED THAT MEMORY TO  STORE Q IS ALLOCATED IN THE CALLING
     ROUTINE.


   ACCESS:
   
     rma , input  ,  Real matrix
     P   , input  ,  Array of input points
     Q   , output ,  Array of new points


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixMultiplyPtArray( NL_RMATRIX *rma, NL_POINT *P, NL_POINT *Q )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh, n, m, bw, sbw;

    NL_REAL ** A;

    NL_MATRIXTYPE mtp;

    /* Get local notation */

    N_GetRealMatrixData( rma, &n, &m, &A, &mtp, &bw );

    sbw = bw / 2;

    /* Compute new points */

    if( mtp EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyPt( NL_ZERO, &Q[i] );

            for ( j = 0; j <= i; j++ )
            {
                N_VectorBlendPt( A[i][j], P[j], &Q[i] );
            }
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyPt( NL_ZERO, &Q[i] );

            for ( j = i; j <= m; j++ )
            {
                N_VectorBlendPt( A[i][j], P[j], &Q[i] );
            }
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_FULL )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyPt( NL_ZERO, &Q[i] );

            for ( j = 0; j <= m; j++ )
            {
                N_VectorBlendPt( A[i][j], P[j], &Q[i] );
            }
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_BANDED )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyPt( NL_ZERO, &Q[i] );

            jl = NL_MAX( 0, i - sbw );
            jh = NL_MIN( n, i + sbw );

            for ( j = jl; j <= jh; j++ )
            {
                N_VectorBlendPt( A[i][j - i + sbw], P[j], &Q[i] );
            }
        }

        NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixMultiplyPtArray */

/*******************************************************************//**


   DESCRIPTION:

     This math  routine  computes the  product of a  real matrix and a
     control point  array, i.e. it computes Qw = rma * Pw, where Qw is 
     a new  array of  control points. A typical  calling example is as 
     follows:

       NL_RMATRIX  rma;
       NL_CPOINT   *Pw, *Qw;
       ...
       (get Pw, rma and allocate memory for Qw);
       ...
       N_RealMatrixMultiplyCPtArray(&rma,Pw,Qw);

     IT IS ASSUMED THAT MEMORY TO STORE Qw IS ALLOCATED IN THE CALLING
     ROUTINE.


   ACCESS:
   
     rma , input  ,  Real matrix
     Pw  , input  ,  Array of input control points
     Qw  , output ,  Array of new control points


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixMultiplyCPtArray( NL_RMATRIX *rma, NL_CPOINT *Pw, NL_CPOINT *Qw )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh, n, m, bw, sbw;

    NL_REAL ** A;

    NL_MATRIXTYPE mtp;

    /* Get local notation */

    N_GetRealMatrixData( rma, &n, &m, &A, &mtp, &bw );

    sbw = bw / 2;

    /* Compute new control points */

    if( mtp EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyCPt( NL_CZERO, &Qw[i] );

            for ( j = 0; j <= i; j++ )
            {
                N_VectorBlendCPt( A[i][j], Pw[j], &Qw[i] );
            }
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyCPt( NL_CZERO, &Qw[i] );

            for ( j = i; j <= m; j++ )
            {
                N_VectorBlendCPt( A[i][j], Pw[j], &Qw[i] );
            }
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_FULL )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyCPt( NL_CZERO, &Qw[i] );

            for ( j = 0; j <= m; j++ )
            {
                N_VectorBlendCPt( A[i][j], Pw[j], &Qw[i] );
            }
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_BANDED )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_CopyCPt( NL_CZERO, &Qw[i] );

            jl = NL_MAX( 0, i - sbw );
            jh = NL_MIN( n, i + sbw );

            for ( j = jl; j <= jh; j++ )
            {
                N_VectorBlendCPt( A[i][j - i + sbw], Pw[j], &Qw[i] );
            }
        }

        NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixMultiplyCPtArray */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  computes  the  product Qw = rma * Pw * rmb^T,
     where rma and rmb are real matrices, Pw is a 2-D array of control
     points, rmb^T denotes the  transpose of rmb, and Qw is the output
     2-D  array of  control  points. A  typical  calling example is as 
     follows:

       NL_RMATRIX  rma, rmb;
       NL_CPOINT   **Pw, **Qw;
       ...
       (get Pw, rma, rmb and allocate memory for Qw);
       ...
       N_RealMatrixMultiplyRealMatrixTranspose(&rma,Pw,&rmb,Qw);

     IT IS ASSUMED THAT MEMORY TO STORE Qw IS ALLOCATED IN THE CALLING
     ROUTINE. 


   ACCESS:
   
     rma , input  ,  Square real matrix
     Pw  , input  ,  2-D array of input control points
     rmb , input  ,  Square real matrix
     Qw  , output ,  2-D array of new control points


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixMultiplyRealMatrixTranspose( NL_RMATRIX *rma, NL_CPOINT ** Pw, NL_RMATRIX *rmb, NL_CPOINT ** Qw )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixMultiplyRealMatrixTranspose");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, na, ma, nb, mb;

    NL_REAL ** A, ** B;

    NL_CPOINT Aw;

    NL_MATRIXTYPE mta, mtb;

    /* Get local notation */

    N_GetRealMatrixData( rma, &na, &ma, &A, &mta, &i );
    N_GetRealMatrixData( rmb, &nb, &mb, &B, &mtb, &j );

    /* Check indexes and matrix types */

    if( na NEQ ma )
        NL_ERROR( NL_IND_ERR );

    if( nb NEQ mb )
        NL_ERROR( NL_IND_ERR );

    if( mta EQ NL_MT_BANDED OR mtb EQ NL_MT_BANDED )
        NL_ERROR( NL_INP_ERR );

    /* Compute new control points */

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                N_CopyCPt( NL_CZERO, &Qw[i][j] );

                for ( k = 0; k <= j; k++ )
                {
                    N_CopyCPt( NL_CZERO, &Aw );

                    for ( l = 0; l <= i; l++ )
                    {
                        N_VectorBlendCPt( A[i][l], Pw[l][k], &Aw );
                    }
                    N_VectorBlendCPt( B[j][k], Aw, &Qw[i][j] );
                }
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                N_CopyCPt( NL_CZERO, &Qw[i][j] );

                for ( k = j; k <= nb; k++ )
                {
                    N_CopyCPt( NL_CZERO, &Aw );

                    for ( l = 0; l <= i; l++ )
                    {
                        N_VectorBlendCPt( A[i][l], Pw[l][k], &Aw );
                    }
                    N_VectorBlendCPt( B[j][k], Aw, &Qw[i][j] );
                }
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_LOWERLEFT AND mtb EQ NL_MT_FULL )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                N_CopyCPt( NL_CZERO, &Qw[i][j] );

                for ( k = 0; k <= nb; k++ )
                {
                    N_CopyCPt( NL_CZERO, &Aw );

                    for ( l = 0; l <= i; l++ )
                    {
                        N_VectorBlendCPt( A[i][l], Pw[l][k], &Aw );
                    }
                    N_VectorBlendCPt( B[j][k], Aw, &Qw[i][j] );
                }
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                N_CopyCPt( NL_CZERO, &Qw[i][j] );

                for ( k = 0; k <= j; k++ )
                {
                    N_CopyCPt( NL_CZERO, &Aw );

                    for ( l = i; l <= na; l++ )
                    {
                        N_VectorBlendCPt( A[i][l], Pw[l][k], &Aw );
                    }
                    N_VectorBlendCPt( B[j][k], Aw, &Qw[i][j] );
                }
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                N_CopyCPt( NL_CZERO, &Qw[i][j] );

                for ( k = j; k <= nb; k++ )
                {
                    N_CopyCPt( NL_CZERO, &Aw );

                    for ( l = i; l <= na; l++ )
                    {
                        N_VectorBlendCPt( A[i][l], Pw[l][k], &Aw );
                    }
                    N_VectorBlendCPt( B[j][k], Aw, &Qw[i][j] );
                }
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_UPPERRIGHT AND mtb EQ NL_MT_FULL )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                N_CopyCPt( NL_CZERO, &Qw[i][j] );

                for ( k = 0; k <= nb; k++ )
                {
                    N_CopyCPt( NL_CZERO, &Aw );

                    for ( l = i; l <= na; l++ )
                    {
                        N_VectorBlendCPt( A[i][l], Pw[l][k], &Aw );
                    }
                    N_VectorBlendCPt( B[j][k], Aw, &Qw[i][j] );
                }
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_FULL AND mtb EQ NL_MT_LOWERLEFT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                N_CopyCPt( NL_CZERO, &Qw[i][j] );

                for ( k = 0; k <= j; k++ )
                {
                    N_CopyCPt( NL_CZERO, &Aw );

                    for ( l = 0; l <= na; l++ )
                    {
                        N_VectorBlendCPt( A[i][l], Pw[l][k], &Aw );
                    }
                    N_VectorBlendCPt( B[j][k], Aw, &Qw[i][j] );
                }
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_FULL AND mtb EQ NL_MT_UPPERRIGHT )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                N_CopyCPt( NL_CZERO, &Qw[i][j] );

                for ( k = j; k <= nb; k++ )
                {
                    N_CopyCPt( NL_CZERO, &Aw );

                    for ( l = 0; l <= na; l++ )
                    {
                        N_VectorBlendCPt( A[i][l], Pw[l][k], &Aw );
                    }
                    N_VectorBlendCPt( B[j][k], Aw, &Qw[i][j] );
                }
            }
        }

        NL_OUT;
    }

    if( mta EQ NL_MT_FULL AND mtb EQ NL_MT_FULL )
    {
        for ( i = 0; i <= na; i++ )
        {
            for ( j = 0; j <= nb; j++ )
            {
                N_CopyCPt( NL_CZERO, &Qw[i][j] );

                for ( k = 0; k <= nb; k++ )
                {
                    N_CopyCPt( NL_CZERO, &Aw );

                    for ( l = 0; l <= na; l++ )
                    {
                        N_VectorBlendCPt( A[i][l], Pw[l][k], &Aw );
                    }
                    N_VectorBlendCPt( B[j][k], Aw, &Qw[i][j] );
                }
            }
        }

        NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixMultiplyRealMatrixTranspose */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes a point on a polynomial curve given in
     power basis form. A typical calling example is as follows:

       NL_CPOINT     *aw;
       NL_INDEX      n;
       NL_PARAMETER  u;
       NL_POINT      C;
       ...
       (get aw array, n and u);
       ...
       N_PowerBasisCrvEvalPts(aw,n,u,&C);

    The routine uses Horner's method for fast computation.


   ACCESS:
   
     aw , input  ,  Control vector array
     n  , input  ,  Highest index in aw
     u  , input  ,  Parameter at which curve is to be evaluated
     C  , output ,  Point on the curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_VOID N_PowerBasisCrvEvalPts( NL_CPOINT *aw, NL_INDEX n, NL_PARAMETER u, NL_POINT *C )
{

    NL_INDEX i;

    NL_CPOINT Cw;

    /* Compute point using Horner's method */

    N_CopyCPt( aw[n], &Cw );

    for ( i = n - 1; i >= 0; i-- )
    {
        N_Combine2CPts( u, Cw, 1.0, aw[i], &Cw );
    }
    N_CPtToPtEuclid( Cw, C );
} /* end N_PowerBasisCrvEvalPts */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes a point on a polynomial surface given 
     in power basis form. A typical calling example is as follows:

       NL_CPOINT     **aw;
       NL_INDEX      n, m;
       NL_PARAMETER  u, v;
       NL_POINT      S;
       ...
       (get aw array, n, m, u and v);
       ...
       N_PowerBasisSrfEvalPts(aw,n,m,u,v,&S);

    The routine uses Horner's method for fast computation.


   ACCESS:
   
     aw   , input  ,  Control vector array
     n,m  , input  ,  Highest indexes in aw
     u,v  , input  ,  Parameters at which surface is to be evaluated
     C    , output ,  Point on the surface


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_VOID N_PowerBasisSrfEvalPts( NL_CPOINT ** aw, NL_INDEX n, NL_INDEX m, NL_PARAMETER u, NL_PARAMETER v, NL_POINT *S )
{

    NL_INDEX i, j;

    NL_CPOINT Sa, Sw;

    /* Compute point using Horner's method */

    N_CopyCPt( aw[n][m], &Sw );

    for ( j = m - 1; j >= 0; j-- )
    {
        N_Combine2CPts( v, Sw, 1.0, aw[n][j], &Sw );
    }

    for ( i = n - 1; i >= 0; i-- )
    {
        N_CopyCPt( aw[i][m], &Sa );

        for ( j = m - 1; j >= 0; j-- )
        {
            N_Combine2CPts( v, Sa, 1.0, aw[i][j], &Sa );
        }
        N_Combine2CPts( u, Sw, 1.0, Sa, &Sw );
    }
    N_CPtToPtEuclid( Sw, S );
} /* end N_PowerBasisSrfEvalPts */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the value and the derivatives of a 
     univariate polynomial. The polynomial is assumed to be in the
     following form:

       p(u) = a[n]*u^n + a[n-1]*u^n-1 + ... + a[1]*u + a[0]

     A typical calling example is as follows:

       NL_REAL       *a, *ad;
       NL_INDEX      n, der;
       NL_PARAMETER  u;
       ...
       (get array a; n, u and der; and allocate memory for ad);
       ...
       N_PowerBasisEvalDerivs(a,n,u,der,ad);

    MEMORY FOR ad MUST BE ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     a   , input  ,  Array of polynomial coefficients
     n   , input  ,  Highest index in a
     u   , input  ,  Parameter  at  which  derivatives  are  to  be 
                     computed
     der , input  ,  Highest derivatives required
     ad  , output ,  Derivatives; ad[k] is the k-th derivative 


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PowerBasisEvalDerivs( NL_REAL *a, NL_INDEX n, NL_PARAMETER u, NL_INDEX der, NL_REAL *ad )
{

    NL_INDEX i, j, k;

    NL_REAL fact;

    /* Initialize */

    ad[0] = a[n];

    for ( i = 1; i <= der; i++ )
        ad[i] = 0.0;

    /* Evaluate polynomial and its derivatives */

    for ( i = n - 1; i >= 0; i-- )
    {
        k = NL_MIN( der, n - i );

        for ( j = k; j >= 1; j-- )
            ad[j] = ad[j] * u + ad[j - 1];

        ad[0] = ad[0] * u + a[i];
    }

    /* Scale derivatives by the factorials */

    fact = 1.0;

    for ( i = 2; i <= der; i++ )
    {
        fact = fact * i;
        ad[i] = ad[i] * fact;
    }
} /* end N_PowerBasisEvalDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This math  routine finds a root of a polynomial using Newton's
     method with a start parameter. The polynomial is assumed to be 
     in the following form:

       p(u) = a[n]*u^n + a[n-1]*u^n-1 + ... + a[1]*u + a[0]

     A typical calling example is as follows:

       NL_REAL       *a, E;
       NL_INDEX      n;
       NL_PARAMETER  u, u0, ul, ur;
       ...
       (get a, u0, ul, ur and E);
       ...
       N_PowerBasisRootNewton(a,n,u0,ul,ur,E,&u);


   ACCESS:
   
     a     , input  ,  Array of polynomial coefficients
     n     , input  ,  Highest index in a
     u0    , input  ,  Guess parameter
     ul,ur , input  ,  Parameter interval root must be in
     E     , input  ,  Error tolerance
     u     , output ,  Root, i.e. p(u)=0.0


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR     

   ***********************************************************************/

NL_FLAG N_PowerBasisRootNewton( NL_REAL *a, NL_INDEX n, NL_PARAMETER u0, NL_PARAMETER ul, NL_PARAMETER ur, NL_REAL E, NL_REAL *u )
{
    NL_PRIVATE NL_STRING rname = _T("N_PowerBasisRootNewton");

    NL_FLAG error = NL_NO;

    NL_INDEX k;

    NL_REAL ad[2];

    NL_PARAMETER uold, unew;

    /* Iterate till limit is reached or root is found */

    k = 0;
    unew = u0;

    while( k LT NL_ITLIM )
    {
        N_PowerBasisEvalDerivs( a, n, unew, 1, ad );

        if( fabs( ad[0] )LT E )
            break;

        if( N_FloatOpIsBad( ad[0], ad[1], NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        uold = unew;
        unew = uold - (ad[0] / ad[1]);

        if( unew LT ul )
            unew = ul;

        if( unew GT ur )
            unew = ur;

        if( fabs( (unew - uold) * ad[1] )LT E )
            break;

        k++;
    }

    /* If no convergence -> error. Otherwise output parameter */

    if( k GE NL_ITLIM )
        NL_ERROR( NL_CON_ERR );

    *u = unew;

    EXIT:

    return (error);
} /* end N_PowerBasisRootNewton */

/*******************************************************************//**


   DESCRIPTION:

     This  math  routine  computes  the  LU  decomposition  of a square 
     matrix. The matrix has to be either "NL_MT_FULL" or "NL_MT_BANDED". The banded 
     matrix stores  only  the  non-zero  elements. For example, a 4 x 4 
     banded matrix with bandwidth 3 is stored as follows:

         B[0][0] | B[0][1]  B[0][2]  X        X       |
                 | B[1][0]  B[1][1]  B[1][2]  X       |
                 | X        B[2][0]  B[2][1]  B[2][2] |
                 | X        X        B[3][0]  B[3][1] | B[3][2]

     where B denotes an (n+1 x bw) array of elemnets (n is the highest
     index in the full matrix and bw is the bandwidth), and X  denotes
     elements in the original matrix that are not stored.  B[0][0] and
     B[3][2]  are  unused  in  the  example  above. A  typical calling  
     example is as follows:

       NL_RMATRIX  rma;
       ...
       (get rma);
       ...
       N_RealMatrixLuDecompose(&rma);

     THE  DECOMPOSITION IS  DONE IN-PLACE, I.E. THE ORIGINAL  MATRIX IS
     DESTROYED. THIS ROUTINE DOES NOT DO PIVOTING!


   ACCESS:
   
     rma   , in/out ,  Real matrix


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixLuDecompose( NL_RMATRIX *rma )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixLuDecompose");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jh, k, kl, n, m, bw, sbw;

    NL_REAL ** A, sum;

    NL_MATRIXTYPE mtp;

    /* Get local notation */

    N_GetRealMatrixData( rma, &n, &m, &A, &mtp, &bw );

    /* Check dimensions and matrix type */

    if( m NEQ n )
        NL_ERROR( NL_IND_ERR );

    if( mtp NEQ NL_MT_FULL AND mtp NEQ NL_MT_BANDED )
        NL_ERROR( NL_INP_ERR );

    sbw = bw / 2;

    /* Decompose matrix */

    if( mtp EQ NL_MT_FULL )
    {
        for ( i = 0; i <= n; i++ )
        {
            for ( j = i; j <= n; j++ )
            {
                sum = 0.0;

                for ( k = 0; k < i; k++ )
                    sum += A[i][k] * A[k][j];
                A[i][j] -= sum;
            }

            for ( j = i + 1; j <= n; j++ )
            {
                sum = 0.0;

                for ( k = 0; k < i; k++ )
                    sum += A[j][k] * A[k][i];

                if( fabs( A[i][i] )LT NL_LUDT )
                    NL_ERROR( NL_NUM_ERR );

                A[j][i] = (A[j][i] - sum) / A[i][i];
            }
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_BANDED )
    {
        for ( i = 0; i <= n; i++ )
        {
            jh = NL_MIN( n, i + sbw );

            for ( j = i; j <= jh; j++ )
            {
                kl = NL_MAX( 0, j - sbw );
                sum = 0.0;

                for ( k = kl; k < i; k++ )
                    sum += A[i][k - i + sbw] * A[k][j - k + sbw];
                A[i][j - i + sbw] -= sum;
            }

            for ( j = i + 1; j <= jh; j++ )
            {
                kl = NL_MAX( 0, j - sbw );
                sum = 0.0;

                for ( k = kl; k < i; k++ )
                    sum += A[j][k - j + sbw] * A[k][i - k + sbw];

                if( fabs( A[i][sbw] )LT NL_LUDT )
                    NL_ERROR( NL_NUM_ERR );

                A[j][i - j + sbw] = (A[j][i - j + sbw] - sum) / A[i][sbw];
            }
        }

        NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixLuDecompose */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the LU decomposition of a square matrix 
     using  Crout's  method with  partial pivoting. The  matrix must be 
     square and full. A typical calling example is as follows:

       NL_INDEX    *ind;
       NL_RMATRIX  rma;
       ...
       (get rma and allocate memory for ind);
       ...
       N_RealMatrixLuDecomposePivot(&rma,ind);

     THE  DECOMPOSITION IS  DONE IN-PLACE, I.E. THE ORIGINAL  MATRIX IS
     DESTROYED. THIS ROUTINE COMPUTES THE LU DECOMPOSITION OF A ROWWISE
     PERMUTATION OF rma.


   ACCESS:
   
     rma  , in/out ,  Real matrix
     ind  , output ,  Index array; must have memory to hold ind[0...n],
                      where n is the highest index of rma


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixLuDecomposePivot( NL_RMATRIX *rma, NL_INDEX *ind )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixLuDecomposePivot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, imax, n, m, bw;

    NL_REAL ** A, *v, sum, temp, dum, big;

    NL_MATRIXTYPE mtp;

    NL_STACKS SL;

    /* Initialize */

    N_InitNurbs( &SL );

    /* Get local notation and check input */

    N_GetRealMatrixData( rma, &n, &m, &A, &mtp, &bw );

    if( m NEQ n )
        NL_ERROR( NL_IND_ERR );

    if( mtp NEQ NL_MT_FULL )
        NL_ERROR( NL_INP_ERR );

    /********************/
    /* Decompose matrix */
    /********************/

    v = N_AllocReal1dArray( n, &SL );

    if( v EQ NULL )
        NL_QUIT;

    /* Get scaling info */

    for ( i = 0; i <= n; i++ )
    {
        big = 0.0;

        for ( j = 0; j <= n; j++ )
        {
            temp = fabs( A[i][j] );

            if( temp GT big )
                big = temp;
        }

        if( big LT NL_LUDT )
            NL_ERROR( NL_NUM_ERR );

        v[i] = 1.0 / big;
    }

    /* For each column do */

    for ( j = 0; j <= n; j++ )
    {
        imax = j;

        for ( i = 0; i < j; i++ )
        {
            sum = A[i][j];

            for ( k = 0; k < i; k++ )
                sum -= A[i][k] * A[k][j];
            A[i][j] = sum;
        }

        big = 0.0;

        for ( i = j; i <= n; i++ )
        {
            sum = A[i][j];

            for ( k = 0; k < j; k++ )
                sum -= A[i][k] * A[k][j];
            A[i][j] = sum;

            dum = v[i] * fabs( sum );

            if( dum GE big )
            {
                big = dum;
                imax = i;
            }
        }

        /* Exchange rows if necessary */

        if( j NEQ imax )
        {
            for ( k = 0; k <= n; k++ )
                N_SwapReals( &A[imax][k], &A[j][k] );

            v[imax] = v[j];
        }

        ind[j] = imax;

        /* Divide by the pivot */

        if( fabs( A[j][j] )LT NL_LUDT )
            NL_ERROR( NL_NUM_ERR );

        if( j LT n )
        {
            dum = 1.0 / A[j][j];

            for ( i = j + 1; i <= n; i++ )
                A[i][j] *= dum;
        }
    }

    /* Exit */

    N_EndNurbs( &SL );

    EXIT:

    return (error);
} /* end N_RealMatrixLuDecomposePivot */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  computes  the solution  of a system of linear 
     equations using forward elimination and backward substitution. It
     assumes an LU decomposed matrix as the  coefficient matrix, and a
     rigth hand side vector of NL_POINT or NL_CPOINT  elements. The solution 
     vector consists of NL_CPOINT elements. To facilitate with both NL_POINT
     and  NL_CPOINT  entities,  the  calling  mechanism  assumes  pointer 
     conversion. It works as follows: 

       NL_RMATRIX  rma;
       NL_POINT    *P;
       NL_CPOINT   *Pw, *Qw;
       NL_STACKS   SG;
       ...
       (get rma, point/control point array P/Pw, and allocate memory 
        for Qw);
       ...
       N_RealMatrixForBack(&rma,(NL_VOID *)P ,NL_EPOINT,Qw,&SG);
       N_RealMatrixForBack(&rma,(NL_VOID *)Pw,NL_HPOINT,Qw,&SG);

     IT IS  ASSUMED THAT  rma IS  AN LU  DECOMPOSED MATRIX, P/Pw IS AN 
     ARRAY OF NL_POINT/NL_CPOINT ITEMS, AND THAT MEMORY FOR Qw IS  ALLOCATED 
     IN THE CALLING ROUTINE.


   ACCESS:
   
     rma , input  ,  LU decomposed coefficient matrix
     A   , input  ,  NL_VOID pointer representing either NL_POINT or NL_CPOINT
     ptp , input  ,  Flag:
                       NL_EPOINT: Euclidean point pointer passed in
                       NL_HPOINT: Homogeneous point pointer passed in  
     Qw  , output ,  Solution vector of NL_CPOINT entities
     SG  , input  ,  Global stacks pointer


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixForBack( NL_RMATRIX *rma, NL_VOID *A, NL_FLAG ptp, NL_CPOINT *Qw, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixForBack");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh, n, m, bw, sbw;

    NL_REAL ** B, fact;

    NL_POINT *P, *Z, S, *Q;

    NL_CPOINT *Pw, *Zw, Sw;

    NL_MATRIXTYPE mtp;

    /* Get local notation */

    N_GetRealMatrixData( rma, &n, &m, &B, &mtp, &bw );

    /* Check dimensions and matrix type */

    if( m NEQ n )
        NL_ERROR( NL_IND_ERR );

    if( mtp NEQ NL_MT_FULL AND mtp NEQ NL_MT_BANDED )
        NL_ERROR( NL_INP_ERR );

    sbw = bw / 2;

    /* Compute solution vector */

    switch( ptp )
    {
        case NL_EPOINT:

            P = (NL_POINT *)A;

            Z = N_AllocPt1dArray( 2 * n + 1, SG );

            if( Z EQ NULL )
                NL_QUIT;

            Q = &Z[n + 1];

            if( mtp EQ NL_MT_FULL )
            {
                for ( i = 0; i <= n; i++ )
                {
                    N_CopyPt( NL_ZERO, &S );

                    for ( j = 0; j < i; j++ )
                    {
                        N_VectorBlendPt( B[i][j], Z[j], &S );
                    }
                    N_Diff2Pts( P[i], S, &Z[i] );
                }

                for ( i = n; i >= 0; i-- )
                {
                    if( N_FloatOpIsBad( 1.0, B[i][i], NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    fact = 1.0 / B[i][i];

                    N_CopyPt( NL_ZERO, &S );

                    for ( j = i + 1; j <= n; j++ )
                    {
                        N_VectorBlendPt( B[i][j], Q[j], &S );
                    }
                    N_Combine2Pts( fact, Z[i], -fact, S, &Q[i] );
                    N_PtToCPt( Q[i], &Qw[i] );
                }

                N_FreePt1dArray( Z, SG );

                NL_OUT;
            }

            if( mtp EQ NL_MT_BANDED )
            {
                for ( i = 0; i <= n; i++ )
                {
                    N_CopyPt( NL_ZERO, &S );

                    jl = NL_MAX( 0, i - sbw );

                    for ( j = jl; j < i; j++ )
                    {
                        N_VectorBlendPt( B[i][j - i + sbw], Z[j], &S );
                    }
                    N_Diff2Pts( P[i], S, &Z[i] );
                }

                for ( i = n; i >= 0; i-- )
                {
                    if( N_FloatOpIsBad( 1.0, B[i][sbw], NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    fact = 1.0 / B[i][sbw];

                    N_CopyPt( NL_ZERO, &S );

                    jh = NL_MIN( n, i + sbw );

                    for ( j = i + 1; j <= jh; j++ )
                    {
                        N_VectorBlendPt( B[i][j - i + sbw], Q[j], &S );
                    }
                    N_Combine2Pts( fact, Z[i], -fact, S, &Q[i] );
                    N_PtToCPt( Q[i], &Qw[i] );
                }

                N_FreePt1dArray( Z, SG );

                NL_OUT;
            }
            break;

        case NL_HPOINT:

            Pw = (NL_CPOINT *)A;

            Zw = N_AllocCPt1dArray( n, SG );

            if( Zw EQ NULL )
                NL_QUIT;

            if( mtp EQ NL_MT_FULL )
            {
                for ( i = 0; i <= n; i++ )
                {
                    N_CopyCPt( NL_CZERO, &Sw );

                    for ( j = 0; j < i; j++ )
                    {
                        N_VectorBlendCPt( B[i][j], Zw[j], &Sw );
                    }
                    N_Diff2CPts( Pw[i], Sw, &Zw[i] );
                }

                for ( i = n; i >= 0; i-- )
                {
                    if( N_FloatOpIsBad( 1.0, B[i][i], NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    fact = 1.0 / B[i][i];

                    N_CopyCPt( NL_CZERO, &Sw );

                    for ( j = i + 1; j <= n; j++ )
                    {
                        N_VectorBlendCPt( B[i][j], Qw[j], &Sw );
                    }
                    N_Combine2CPts( fact, Zw[i], -fact, Sw, &Qw[i] );
                }

                N_FreeCPt1dArray( Zw, SG );

                NL_OUT;
            }

            if( mtp EQ NL_MT_BANDED )
            {
                for ( i = 0; i <= n; i++ )
                {
                    N_CopyCPt( NL_CZERO, &Sw );

                    jl = NL_MAX( 0, i - sbw );

                    for ( j = jl; j < i; j++ )
                    {
                        N_VectorBlendCPt( B[i][j - i + sbw], Zw[j], &Sw );
                    }
                    N_Diff2CPts( Pw[i], Sw, &Zw[i] );
                }

                for ( i = n; i >= 0; i-- )
                {
                    if( N_FloatOpIsBad( 1.0, B[i][sbw], NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    fact = 1.0 / B[i][sbw];

                    N_CopyCPt( NL_CZERO, &Sw );

                    jh = NL_MIN( n, i + sbw );

                    for ( j = i + 1; j <= jh; j++ )
                    {
                        N_VectorBlendCPt( B[i][j - i + sbw], Qw[j], &Sw );
                    }
                    N_Combine2CPts( fact, Zw[i], -fact, Sw, &Qw[i] );
                }

                N_FreeCPt1dArray( Zw, SG );

                NL_OUT;
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixForBack */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  computes  the solution  of a system of linear 
     equations using forward elimination and backward substitution. It
     assumes an LU decomposed matrix as the  coefficient matrix, and a
     rigth  hand  side  vector of  NL_REAL elements. The  solution vector 
     consists of NL_REAL elements as well. A typical  calling example is:

       NL_RMATRIX  rma;
       NL_REAL     *y, *x;
       ...
       (get right hand side vector y);
       ...
       N_RealMatrixRightForBack(&rma,y,x);

     IT IS ASSUMED THAT rma IS AN LU DECOMPOSED MATRIX AND THAT MEMORY
     FOR x IS ALLOCATED IN THE  CALLING  ROUTINE. THE MATRIX TYPE MUST 
     BE EITHER "NL_MT_FULL" OR "NL_MT_BANDED".


   ACCESS:
   
     rma , input  ,  LU  decomposed  coefficient   matrix  ("NL_MT_FULL"  or "NL_MT_BANDED")
     y   , input  ,  Right hand side vector
     x   , output ,  Solution vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixRightForBack( NL_RMATRIX *rma, NL_REAL *y, NL_REAL *x )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixRightForBack");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, jl, jh, n, m, bw, sbw;

    NL_REAL ** A, *z, sum;

    NL_MATRIXTYPE mtp;

    NL_STACKS SL;

    /* Start NURBS environment */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_GetRealMatrixData( rma, &n, &m, &A, &mtp, &bw );

    /* Check dimensions and matrix type */

    if( m NEQ n )
        NL_ERROR( NL_IND_ERR );

    if( mtp NEQ NL_MT_FULL AND mtp NEQ NL_MT_BANDED )
        NL_ERROR( NL_INP_ERR );

    sbw = bw / 2;

    /* Compute solution vector */

    z = N_AllocReal1dArray( n, &SL );

    if( z EQ NULL )
        NL_QUIT;

    if( mtp EQ NL_MT_FULL )
    {
        for ( i = 0; i <= n; i++ )
        {
            sum = 0.0;

            for ( j = 0; j < i; j++ )
                sum += A[i][j] * z[j];
            z[i] = y[i] - sum;
        }

        for ( i = n; i >= 0; i-- )
        {
            sum = 0.0;

            for ( j = i + 1; j <= n; j++ )
                sum += A[i][j] * x[j];

            if( N_FloatOpIsBad( z[i] - sum, A[i][i], NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            x[i] = (z[i] - sum) / A[i][i];
        }

        NL_OUT;
    }

    if( mtp EQ NL_MT_BANDED )
    {
        for ( i = 0; i <= n; i++ )
        {
            sum = 0.0;
            jl = NL_MAX( 0, i - sbw );

            for ( j = jl; j < i; j++ )
                sum += A[i][j - i + sbw] * z[j];
            z[i] = y[i] - sum;
        }

        for ( i = n; i >= 0; i-- )
        {
            sum = 0.0;
            jh = NL_MIN( n, i + sbw );

            for ( j = i + 1; j <= jh; j++ )
                sum += A[i][j - i + sbw] * x[j];

            if( N_FloatOpIsBad( z[i] - sum, A[i][sbw], NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            x[i] = (z[i] - sum) / A[i][sbw];
        }

        NL_OUT;
    }

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_RealMatrixRightForBack */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  computes  the solution  of a system of linear 
     equations using forward elimination and backward substitution. It
     assumes an LU  decomposed matrix obtained using partial pivoting, 
     and a right hand side vector b. The solution is stored in b, i.e. 
     the input is destroyed. A typical  calling example is:

       NL_RMATRIX  rma;
       NL_INDEX    *ind;
       NL_REAL     *b;
       ...
       (LU decompose rma and get b);
       ...
       N_RealMatrixForBackPivot(&rma,ind,b);

     THE MATRIX TYPE MUST BE "NL_MT_FULL" AND MEMORY FOR b MUST BE ALLOCATED 
     IN THE  CALLING  ROUTINE


   ACCESS:
   
     rma , input  ,  LU decomposed coefficient matrix
     ind , input  ,  Permutation vector obtained from LU decomposition
     b   , in/out ,  Right hand side vector/solution vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixForBackPivot( NL_RMATRIX *rma, NL_INDEX *ind, NL_REAL *b )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixForBackPivot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, ii, ip, n, m, bw;

    NL_REAL ** A, sum;

    NL_MATRIXTYPE mtp;

    /* Get local notation and check input */

    N_GetRealMatrixData( rma, &n, &m, &A, &mtp, &bw );

    if( m NEQ n )
        NL_ERROR( NL_IND_ERR );

    if( mtp NEQ NL_MT_FULL )
        NL_ERROR( NL_INP_ERR );

    ii = -1;

    /* Compute solution vector */

    for ( i = 0; i <= n; i++ )
    {
        ip = ind[i];
        sum = b[ip];
        b[ip] = b[i];

        if( ii GE 0 )
        {
            for ( j = ii; j < i; j++ )
                sum -= A[i][j] * b[j];
        }
        else
        {
            if( fabs( sum )GT NL_LUDT )
                ii = i;
        }

        b[i] = sum;
    }

    for ( i = n; i >= 0; i-- )
    {
        sum = b[i];

        for ( j = i + 1; j <= n; j++ )
            sum -= A[i][j] * b[j];

        if( N_FloatOpIsBad( sum, A[i][i], NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        b[i] = sum / A[i][i];
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixForBackPivot */

/*******************************************************************//**


   DESCRIPTION:

     This math  routine computes the  inverse of a real matrix. If the 
     output  matrix  is  initiaized  to   the  NULL  matrix  (via  the 
     N_InitRealMatrix() routine), memory  is  allocated. If  not, the  routine  
     checks if enough memory is available. A  typical calling  example 
     is as follows:

       NL_RMATRIX  rma, rmb;
       NL_STACKS   SB;
       ...
       (get rma);
       ...
       N_InitRealMatrix(&rmb);
       N_RealMatrixInverse(&rma,NL_YES,&rmb,&SB);

     IT IS ASSUMED THAT  MEMORY TO  STORE  THE  MATRIX  STRUCTURES  IS 
     ALLOCATED IN THE  CALLING ROUTINE. THIS  ROUTINE DOES NOT PERFORM
     PIVOTING.


   ACCESS:
   
     rma , input  ,  Real matrix
     lud , input  ,  Flag:
                       NL_YES: rma is LU-decomposed
                       NL_NO : rma is not LU-decomposed
     rmb , output ,  Inverse of rma
     SB  , input  ,  rmb's stack 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixInverse( NL_RMATRIX *rma, NL_FLAG lud, NL_RMATRIX *rmb, NL_STACKS *SB )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixInverse");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_REAL ** A, ** B, *x, *y;

    NL_MATRIXTYPE mta;

    /* Get local notation */

    N_GetRealMatrixData( rma, &n, &m, &A, &mta, &i );

    /* Check matrix type */

    if( n NEQ m )
        NL_ERROR( NL_IND_ERR );

    if( mta NEQ NL_MT_FULL AND mta NEQ NL_MT_BANDED )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( rmb, n, m, NL_MT_FULL, n, rname, SB );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rmb, &B );

    /* LU decompose matrix if needed */

    if( lud EQ NL_NO )
    {
        error = N_RealMatrixLuDecompose( rma );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Compute the inverse */

    x = N_AllocReal1dArray( n, SB );

    if( x EQ NULL )
        NL_QUIT;

    y = N_AllocReal1dArray( n, SB );

    if( y EQ NULL )
        NL_QUIT;

    y[0] = 1.0;

    for ( i = 1; i <= n; i++ )
        y[i] = 0.0;

    error = N_RealMatrixRightForBack( rma, y, x );

    if( error EQ NL_YES )
        NL_OUT;

    for ( i = 0; i <= n; i++ )
        B[i][0] = x[i];

    for ( i = 1; i <= n; i++ )
    {
        y[i - 1] = 0.0;
        y[i] = 1.0;

        error = N_RealMatrixRightForBack( rma, y, x );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 0; j <= n; j++ )
            B[j][i] = x[j];
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixInverse */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the inverse of a real matrix using LU
     decomposition and  forward/backward  substitution  with  partial 
     pivoting. If the output matrix is initiaized to NULL, memory  is  
     allocated. If not, the routine checks if enough memory is passed
     in. A typical calling example is as follows:

       NL_RMATRIX  rma, rmb;
       NL_STACKS   SB;
       ...
       (get rma);
       ...
       N_InitRealMatrix(&rmb);
       N_RealMatrixInversePivot(&rma,&rmb,&SB);

     IT IS ASSUMED THAT  MEMORY TO STORE  THE  MATRIX  STRUCTURES  IS 
     ALLOCATED IN THE CALLING  ROUTINE. THE INPUT MATRIX IS LU-DECOM-
     POSED IN PLACE, I.E. IT IS DESTROYED. IT MUST BE  SAVED ON INPUT
     IF THIS IS NOT DESIRED!


   ACCESS:
   
     rma , in/out ,  Real matrix
     rmb , output ,  Inverse of rma
     SB  , input  ,  rmb's stack 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixInversePivot( NL_RMATRIX *rma, NL_RMATRIX *rmb, NL_STACKS *SB )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixInversePivot");

    NL_FLAG error = NL_NO;

    NL_INDEX *ind, i, j, n, m;

    NL_REAL ** B, *col;

    NL_MATRIXTYPE mtp;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation and check input */

    N_GetRealMatrixData( rma, &n, &m, &B, &mtp, &i );

    if( n NEQ m )
        NL_ERROR( NL_IND_ERR );

    if( mtp NEQ NL_MT_FULL )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( rmb, n, m, NL_MT_FULL, n, rname, SB );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rmb, &B );

    /* LU decompose matrix */

    ind = N_AllocInt1dArray( n, &SL );

    if( ind EQ NULL )
        NL_QUIT;

    error = N_RealMatrixLuDecomposePivot( rma, ind );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the inverse */

    col = N_AllocReal1dArray( n, &SL );

    if( col EQ NULL )
        NL_QUIT;

    for ( j = 0; j <= n; j++ )
    {
        for ( i = 0; i <= n; i++ )
            col[i] = 0.0;
        col[j] = 1.0;

        error = N_RealMatrixForBackPivot( rma, ind, col );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 0; i <= n; i++ )
            B[i][j] = col[i];
    }

    /* Exit */

    N_EndNurbs( &SL );

    EXIT:

    return (error);
} /* end N_RealMatrixInversePivot */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the least squares solution to  a  linear
     system of m+1 equations in n+1 unknowns, m > n.  A typical  calling
     example is as follows:

       NL_REAL     *rhs, *sol;
       NL_RMATRIX  rma;
       ...
       (get rma, allocate memory for rhs and sol, and load rhs)
       ...
       N_RealMatrixLstSqSolve(&rma,rhs,sol);


   ACCESS:
   
     rma  , input  ,  Real matrix with m+1 rows and n+1 columns (m>n)
     rhs  , input  ,  The right hand side of the system with at least m+1 elements. 
     sol  , output ,  The solution.  Must be allocated  in  the  calling
                      routine with at least n+1 elements


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixLstSqSolve
 (NL_RMATRIX *rma,  /* in : NL_RMATRIX *A of Ax=b, with m+1 rows and n+1 columns (m>n) */
  NL_REAL    *rhs,  /* in : NL_REAL    *b of Ax=b, sized:[m+1] */
  NL_REAL    *sol ) /* out: NL_REAL    *x of Ax=b, sized:[n+1] */
{                           
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixLstSqSolve");

    NL_FLAG error = NL_NO;

    NL_INDEX ii, n, m, kk, *ind;

    NL_REAL ** RM;

    NL_RMATRIX rmt, rmta;

    NL_MATRIXTYPE mtp;

    NL_STACKS SL;

    /* Initialize */

    N_InitNurbs( &SL );

    /* Get local notation and check input */

    N_GetRealMatrixData( rma, &m, &n, &RM, &mtp, &kk );

    if( m LE n )
        NL_ERROR( NL_IND_ERR );

    /* Get transpose of rma and multiply the transpose with */
    /* the original to get the square (n+1)x(n+1) system    */

    N_InitRealMatrix( &rmt );
    error = N_RealMatrixTranspose( rma, &rmt, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &rmta );
    error = N_RealMatrixMultiply( &rmt, rma, &rmta, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Multiply rmt with rhs to get the new rhs with n+1 elements */

    for ( ii = 0; ii <= n; ii++ )
    {
        sol[ii] = 0.0;

        for ( kk = 0; kk <= m; kk++ )
            sol[ii] += (RM[kk][ii] * rhs[kk]);
    }

    /* Decompose matrix */

    ind = N_AllocInt1dArray( n, &SL );

    if( ind EQ NULL )
        NL_QUIT;

    error = N_RealMatrixLuDecomposePivot( &rmta, ind );

    if( error EQ NL_YES )
        NL_OUT;

    /* Do forward/backward substitution to get solution */

    error = N_RealMatrixForBackPivot( &rmta, ind, sol );

    if( error EQ NL_YES )
        NL_OUT;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_RealMatrixLstSqSolve */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  computes  the solution  of a system of  linear 
     equations.  It assumes the coefficient matrix has been  decomposed
     using NL_SVD (Single Value Decomposition). The right hand side vector
     may consist of NL_POINT, NL_CPOINT, or NL_REAL elements.  The solution vec-
     tor consists of NL_CPOINT elements if the right hand side  is  either
     NL_POINT or NL_CPOINT type,  and NL_REAL if the right hand side is of  type
     NL_REAL.  To facilitate with NL_POINT,  NL_CPOINT  and  NL_REAL entities,  the 
     calling mechanism assumes pointer conversion. It works as follows: 

       NL_REAL     **U, *W, **V;
       NL_REAL     *X, *R;
       NL_POINT    *P;
       NL_CPOINT   *Pw, *Qw;
       ...
       (get NL_SVD arrays U, W, and V; load right hand side: P, Pw or R,
        and allocate the solution vector: Qw or X);
       ...
       N_SingleValueDecomposeSolve(U,W,V,me,nu,(NL_VOID *)P ,NL_EPOINT,NL_YES,(NL_VOID *)Qw);
       N_SingleValueDecomposeSolve(U,W,V,me,nu,(NL_VOID *)Pw,NL_HPOINT,NL_NO ,(NL_VOID *)Qw);
       N_SingleValueDecomposeSolve(U,W,V,me,nu,(NL_VOID *)R ,NL_RVALUE,NL_YES,(NL_VOID *)X );

     This routine assumes that U is a full matrix with me+1 rows  and
     nu+1 columns. See the book: "Numerical Recipes in C", Chapter 2,
     for a description of the algorithm and the meaning of arrays: U,
     W and V.


   ACCESS:
   
     U    , input  ,  A factor of the NL_SVD. me+1 rows, nu+1 columns
     W    , in/out ,  A factor of the NL_SVD. A vector with nu+1 elements.
                      These values may be changed if esvf = NL_YES
     V    , input  ,  A factor of the NL_SVD.  Full matrix with nu+1  rows
                      and columns
     me   , input  ,  me+1 is the number of equations
     nu   , input  ,  nu+1 is the number of unknowns (nu <= me)
     rhs  , input  ,  Right hand side vector (me+1 elements).  Elements
                      may be of type NL_POINT, NL_CPOINT or NL_REAL.  The  array
                      must be type cast as (NL_VOID *)
     type , input  ,  Flag:
                       NL_EPOINT: Right hand side is type NL_POINT
                       NL_HPOINT: Right hand side is type NL_CPOINT
                       NL_RVALUE: Right hand side is type NL_REAL
     esvf , input  ,  Flag:
                       NL_YES: Single values (in W) will be edited in this
                            routine
                       NL_NO : Do not edit single values in this  routine.
                            Assumes the caller has edited and  set  the
                            desired values to zero
     sol  , output ,  Solution vector of NL_CPOINT entities if type=NL_EPOINT
                      or type=NL_HPOINT,  or  NL_REAL  entities if type=NL_REAL. 
                      Memory for this array must be  allocated  in  the
                      calling routine (with high index nu)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SingleValueDecomposeSolve( NL_REAL ** U, NL_REAL *W, NL_REAL ** V, NL_INDEX me, NL_INDEX nu, NL_VOID *rhs, NL_FLAG type, NL_FLAG esvf, NL_VOID *sol )
{

    NL_PRIVATE NL_STRING rname = _T("N_SingleValueDecomposeSolve");

    NL_PRIVATE NL_REAL SVDTOL = 6.e-8;

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj;

    NL_REAL *R, *X, wmax, *rtemp, dd;

    NL_POINT *P, *ptemp, PP;

    NL_CPOINT *Pw, *Qw, *ctemp, CC;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error, and then edit single values */

    if( nu GT me )
        NL_ERROR( NL_INP_ERR );

    if( esvf EQ NL_YES )
    {
        wmax = 0.0;

        for ( ii = 0; ii <= nu; ii++ )
            if( W[ii]GT wmax )
                wmax = W[ii];

        if( wmax LE SVDTOL )
            NL_ERROR( NL_NUM_ERR );

        dd = wmax * me * SVDTOL;

        for ( ii = 0; ii <= nu; ii++ )
            if( W[ii]LE dd )
                W[ii] = 0.0;
    }

    /* Now solve the system. Switch on type of right hand side */

    switch( type )
    {
        case NL_RVALUE:

            rtemp = N_AllocReal1dArray( me, &SL );

            if( rtemp EQ NULL )
                NL_QUIT;

            R = (NL_REAL *)rhs;
            X = (NL_REAL *)sol;

            for ( ii = 0; ii <= nu; ii++ )
            {
                dd = 0.0;

                if( W[ii]NEQ 0.0 )
                {
                    for ( jj = 0; jj <= me; jj++ )
                        dd += U[jj][ii] * R[jj];

                    if( N_FloatOpIsBad( dd, W[ii], NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    dd /= W[ii];
                }
                rtemp[ii] = dd;
            }

            for ( ii = 0; ii <= nu; ii++ )
            {
                dd = 0.0;

                for ( jj = 0; jj <= nu; jj++ )
                    dd += V[ii][jj] * rtemp[jj];
                X[ii] = dd;
            }

            break;

        case NL_EPOINT:

            ptemp = N_AllocPt1dArray( me, &SL );

            if( ptemp EQ NULL )
                NL_QUIT;

            P = (NL_POINT *)rhs;
            Qw = (NL_CPOINT *)sol;

            for ( ii = 0; ii <= nu; ii++ )
            {
                N_CopyPt( NL_ZERO, &PP );

                if( W[ii]NEQ 0.0 )
                {
                    for ( jj = 0; jj <= me; jj++ )
                        N_VectorBlendPt( U[jj][ii], P[jj], &PP );

                    if( N_FloatOpIsBad( 1.0, W[ii], NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    N_ScalePt( 1.0 / W[ii], PP, &PP );
                }
                N_CopyPt( PP, &ptemp[ii] );
            }

            for ( ii = 0; ii <= nu; ii++ )
            {
                N_CopyPt( NL_ZERO, &PP );

                for ( jj = 0; jj <= nu; jj++ )
                    N_VectorBlendPt( V[ii][jj], ptemp[jj], &PP );
                N_PtToCPt( PP, &Qw[ii] );
            }

            break;

        case NL_HPOINT:

            ctemp = N_AllocCPt1dArray( me, &SL );

            if( ctemp EQ NULL )
                NL_QUIT;

            Pw = (NL_CPOINT *)rhs;
            Qw = (NL_CPOINT *)sol;

            for ( ii = 0; ii <= nu; ii++ )
            {
                N_CopyCPt( NL_CZERO, &CC );

                if( W[ii]NEQ 0.0 )
                {
                    for ( jj = 0; jj <= me; jj++ )
                        N_VectorBlendCPt( U[jj][ii], Pw[jj], &CC );

                    if( N_FloatOpIsBad( 1.0, W[ii], NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    N_ScaleCPt( 1.0 / W[ii], CC, &CC );
                }
                N_CopyCPt( CC, &ctemp[ii] );
            }

            for ( ii = 0; ii <= nu; ii++ )
            {
                N_CopyCPt( NL_CZERO, &CC );

                for ( jj = 0; jj <= nu; jj++ )
                    N_VectorBlendCPt( V[ii][jj], ctemp[jj], &CC );
                N_CopyCPt( CC, &Qw[ii] );
            }

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SingleValueDecomposeSolve */

/*******************************************************************//**


   DESCRIPTION:

     This math routine performs a single value decomposition on a (m+1)x
     (n+1) matrix, A ,  where m >= n.  That is: A = U x W x T(V),  where 
     T(V) is the transpose of V. A is overwritten by U. A is stored as a 
     full (m+1)x(n+1) matrix. A typical calling example is as follows:

       NL_REAL     **A, *W, **V;
       ...
       (allocate memory for A, W and V; load A, and make a copy of A if
        if it is to be saved);
       ...
       N_RealMatrixSingleValueDecompose(A,W,V,m,n);

     This routine assumes that A is a full matrix with  m+1  rows  and
     n+1 columns.  See the book: "Numerical Recipes in C",  Chapter 2,
     for a description of the algorithm and the meaning of arrays:  A,
     W and V.


   ACCESS:
   
     A  , in/out ,  The matrix with m+1 rows and n+1 columns. This mem-
                    memory is overwritten by U
     W  , output ,  The single values vector with n+1 elements.  Memory
                    for this array must be  allocated  in  the  calling 
                    routine
     V  , output ,  The orthogal matrix with n+1 rows and n+1  columns.
                    Memory for this array  must  be  allocated  in  the 
                    calling routine
     m  , input  ,  m+1 is column length of A (number of equations)
     n  , input  ,  n+1 is row length of A (number of unknowns). m >= n
                    must hold


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixSingleValueDecompose( NL_REAL ** A, NL_REAL *W, NL_REAL ** V, NL_INDEX m, NL_INDEX n )
{

    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixSingleValueDecompose");

    NL_PRIVATE NL_INTEGER MXITS = 30;

    NL_FLAG error = NL_NO;

    NL_INDEX i1, j1, flag, its, kk, ll, nm = 0, jj;

    NL_REAL c, f, h, s, x, y, z, *rv1, anorm = 0.0, g = 0.0, scale = 0.0;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for error */

    if( n GT m )
        NL_ERROR( NL_INP_ERR );

    /* Householder reduction to bidiagonal form */

    rv1 = N_AllocReal1dArray( n, &SL );

    if( rv1 EQ NULL )
        NL_QUIT;

    for ( i1 = 0; i1 <= n; i1++ )
    {
        ll = i1 + 1;
        rv1[i1] = scale * g;
        g = s = scale = 0.0;

        if( i1 <= m )
        {
            for ( kk = i1; kk <= m; kk++ )
                scale += fabs( A[kk][i1] );

            if( scale NEQ 0.0 )
            {
                for ( kk = i1; kk <= m; kk++ )
                {
                    A[kk][i1] /= scale;
                    s += A[kk][i1] * A[kk][i1];
                }

                f = A[i1][i1];
                g = -ST_SIGN( sqrt( s ), f );
                h = f * g - s;
                A[i1][i1] = f - g;

                if( i1 NEQ n )
                {
                    for ( j1 = ll; j1 <= n; j1++ )
                    {
                        s = 0.0;

                        for ( kk = i1; kk <= m; kk++ )
                            s += A[kk][i1] * A[kk][j1];
                        f = s / h;

                        for ( kk = i1; kk <= m; kk++ )
                            A[kk][j1] += f * A[kk][i1];
                    }
                }

                for ( kk = i1; kk <= m; kk++ )
                    A[kk][i1] *= scale;
            }
        }

        W[i1] = scale * g;
        g = s = scale = 0.0;

        if( i1 LE m AND i1 NEQ n )
        {
            for ( kk = ll; kk <= n; kk++ )
                scale += fabs( A[i1][kk] );

            if( scale NEQ 0.0 )
            {
                for ( kk = ll; kk <= n; kk++ )
                {
                    A[i1][kk] /= scale;
                    s += A[i1][kk] * A[i1][kk];
                }

                f = A[i1][ll];
                g = -ST_SIGN( sqrt( s ), f );
                h = f * g - s;
                A[i1][ll] = f - g;

                for ( kk = ll; kk <= n; kk++ )
                    rv1[kk] = A[i1][kk] / h;

                if( i1 NEQ m )
                {
                    for ( j1 = ll; j1 <= m; j1++ )
                    {
                        s = 0.0;

                        for ( kk = ll; kk <= n; kk++ )
                            s += A[j1][kk] * A[i1][kk];

                        for ( kk = ll; kk <= n; kk++ )
                            A[j1][kk] += s * rv1[kk];
                    }
                }

                for ( kk = ll; kk <= n; kk++ )
                    A[i1][kk] *= scale;
            }
        }

        anorm = NL_MAX( anorm, (fabs( W[i1] ) + fabs( rv1[i1] )) );
    }

    /* Accumulation of right hand transformations */

    for ( i1 = n, ll = i1; i1 >= 0; i1-- )
    {
        if( i1 LT n )
        {
            if( g NEQ 0.0 )
            {
                for ( j1 = ll; j1 <= n; j1++ )
                    V[j1][i1] = (A[i1][j1] / A[i1][ll]) / g;

                for ( j1 = ll; j1 <= n; j1++ )
                {
                    s = 0.0;

                    for ( kk = ll; kk <= n; kk++ )
                        s += A[i1][kk] * V[kk][j1];

                    for ( kk = ll; kk <= n; kk++ )
                        V[kk][j1] += s * V[kk][i1];
                }
            }

            for ( j1 = ll; j1 <= n; j1++ )
                V[i1][j1] = V[j1][i1] = 0.0;
        }

        V[i1][i1] = 1.0;
        g = rv1[i1];
        ll = i1;
    }

    /* Accumulation of left hand transformations */

    for ( i1 = n; i1 >= 0; i1-- )
    {
        ll = i1 + 1;
        g = W[i1];

        if( i1 LT n )
            for ( j1 = ll; j1 <= n; j1++ )
                A[i1][j1] = 0.0;

        if( g NEQ 0.0 )
        {
            g = 1.0 / g;

            if( i1 NEQ n )
            {
                for ( j1 = ll; j1 <= n; j1++ )
                {
                    s = 0.0;

                    for ( kk = ll; kk <= m; kk++ )
                        s += A[kk][i1] * A[kk][j1];
                    f = (s / A[i1][i1]) * g;

                    for ( kk = i1; kk <= m; kk++ )
                        A[kk][j1] += f * A[kk][i1];
                }
            }

            for ( j1 = i1; j1 <= m; j1++ )
                A[j1][i1] *= g;
        }
        else
        {
            for ( j1 = i1; j1 <= m; j1++ )
                A[j1][i1] = 0.0;
        }

        A[i1][i1] += 1.0;
    }

    /* Diagonalization of the bidiagonal form */

    for ( kk = n; kk >= 0; kk-- )            /* loop over singular values */
    {
        for ( its = 1; its <= MXITS; its++ ) /* loop over allowed iterations */
        {
            flag = 1;

            for ( ll = kk; ll >= 0; ll-- ) /* test for splitting */
            {
                nm = ll - 1;               /* note that rv1[0] is always 0 */

                if( fabs( rv1[ll] ) + anorm EQ anorm )
                {
                    flag = 0;
                    break;
                }

                if( fabs( W[nm] ) + anorm EQ anorm )
                    break;
            }

            if( flag NEQ 0 )
            { /* cancellation of rv1[ll] if ll > 1 */
                c = 0.0;
                s = 1.0;

                for ( i1 = ll; i1 <= kk; i1++ )
                {
                    f = s * rv1[i1];

                    if( fabs( f ) + anorm NEQ anorm )
                    {
                        g = W[i1];
                        h = sqrt( f * f + g * g );
                        W[i1] = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = -f * h;

                        for ( j1 = 0; j1 <= m; j1++ )
                        {
                            y = A[j1][nm];
                            z = A[j1][i1];
                            A[j1][nm] = y * c + z * s;
                            A[j1][i1] = z * c - y * s;
                        }
                    }
                }
            }

            z = W[kk];

            if( ll EQ kk )
            {     /* Convergence. */
                if( z LT 0.0 )
                { /* Singular value is made nonnegative */
                    W[kk] = -z;

                    for ( j1 = 0; j1 <= n; j1++ )
                        V[j1][kk] = -V[j1][kk];
                }
                break;
            }

            if( its EQ MXITS )
                NL_ERROR( NL_CON_ERR );

            x = W[ll]; /* shift from bottom 2-by-2 minor */
            nm = kk - 1;
            y = W[nm];
            g = rv1[nm];
            h = rv1[kk];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 *h * y);
            g = sqrt( f * f + 1.0 );
            f = ((x - z) * (x + z) + h * ((y / (f + ST_SIGN( g, f ))) - h)) / x;

            c = s = 1.0; /* next QR transformation: */

            for ( j1 = ll; j1 <= nm; j1++ )
            {
                i1 = j1 + 1;
                g = rv1[i1];
                y = W[i1];
                h = s * g;
                g = c * g;
                z = sqrt( f * f + h * h );
                rv1[j1] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y = y * c;

                for ( jj = 0; jj <= n; jj++ )
                {
                    x = V[jj][j1];
                    z = V[jj][i1];
                    V[jj][j1] = x * c + z * s;
                    V[jj][i1] = z * c - x * s;
                }

                z = sqrt( f * f + h * h );
                W[j1] = z; /* rotation can be arbitrary if z=0 */

                if( z NEQ 0.0 )
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }

                f = (c * g) + (s * y);
                x = (c * y) - (s * g);

                for ( jj = 0; jj <= m; jj++ )
                {
                    y = A[jj][j1];
                    z = A[jj][i1];
                    A[jj][j1] = y * c + z * s;
                    A[jj][i1] = z * c - y * s;
                }
            }

            rv1[ll] = 0.0;
            rv1[kk] = f;
            W[kk] = x;
        }
    }

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_RealMatrixSingleValueDecompose */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  computes  the solution  of a system of linear 
     equations using forward elimination and backward substitution. It
     assumes an LU  decomposed matrix obtained using partial pivoting, 
     and a right hand side vector with NL_POINT-type elements.  The solu-
     tion vector is of type NL_CPOINT. A typical calling example is:

       NL_RMATRIX  rma;
       NL_INDEX    *ind;
       NL_POINT    *rhs;
       NL_CPOINT   *sol
       ...
       (LU decompose rma and get rhs vector);
       ...
       N_RealMatrixRightForBackPivot(&rma,ind,rhs,sol);

     THE MATRIX TYPE MUST BE "NL_MT_FULL" AND MEMORY FOR sol MUST BE  ALLOC-
     ATED IN THE CALLING ROUTINE


   ACCESS:
   
     rma , input  ,  LU decomposed coefficient matrix
     ind , input  ,  Permutation vector obtained from LU  decomposition
     rhs , input  ,  Right hand side vector of type NL_POINT
     sol , output ,  Solution vector of type NL_CPOINT (allocated in call-
                     ing routine)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixRightForBackPivot( NL_RMATRIX *rma, NL_INDEX *ind, NL_POINT *rhs, NL_CPOINT *sol )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixRightForBackPivot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, ii, ip, n, m, bw, kk;

    NL_REAL ** A, sum, ** b;

    NL_MATRIXTYPE mtp;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation and check input */

    N_GetRealMatrixData( rma, &n, &m, &A, &mtp, &bw );

    if( m NEQ n )
        NL_ERROR( NL_IND_ERR );

    if( mtp NEQ NL_MT_FULL )
        NL_ERROR( NL_INP_ERR );

    /* Transfer rhs points to b[][3] array for easy processing */

    b = N_AllocReal2dArray( n, 2, &SL );

    if( b EQ NULL )
        NL_QUIT;

    for ( kk = 0; kk <= n; kk++ )
        N_PtToXYZ( rhs[kk], &b[kk][0], &b[kk][1], &b[kk][2] );

    /* Compute solution vector */

    for ( kk = 0; kk < 3; kk++ )
    {
        ii = -1;

        for ( i = 0; i <= n; i++ )
        {
            ip = ind[i];
            sum = b[ip][kk];
            b[ip][kk] = b[i][kk];

            if( ii GE 0 )
            {
                for ( j = ii; j < i; j++ )
                    sum -= A[i][j] * b[j][kk];
            }
            else
            {
                if( fabs( sum )GT NL_LUDT )
                    ii = i;
            }

            b[i][kk] = sum;
        }

        for ( i = n; i >= 0; i-- )
        {
            sum = b[i][kk];

            for ( j = i + 1; j <= n; j++ )
                sum -= A[i][j] * b[j][kk];

            if( N_FloatOpIsBad( sum, A[i][i], NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            b[i][kk] = sum / A[i][i];
        }
    }

    /* Load control point solution vector */

    for ( kk = 0; kk <= n; kk++ )
        N_CPtFromWxWyWz( b[kk][0], b[kk][1], b[kk][2], NL_NOW, &sol[kk] );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_RealMatrixRightForBackPivot */

/*******************************************************************//**


   DESCRIPTION:

     This math routine sorts a real array using Shellsort. It also out-
     puts an index array relating old and new indexes.  A typical call-
     ing example is:

       NL_REAL   *a;
       NL_INDEX  n, *ind;
       ...
       (get array a);
       ...
       N_SortRealArray(a,n,ind);


   ACCESS:
   
     a   , in/out ,  Real array
     n   , input  ,  Highest index in a and ind
     ind , output ,  Index array.  The output element a[i] is equal  to
                     the input element a[ind[i]]. Memory for this array
                     must be allocated in the calling routine


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SortRealArray( NL_REAL *a, NL_INDEX n, NL_INDEX *ind )
{

    NL_INDEX i, j, k, bi;

    NL_REAL b;

    for ( i = 0; i <= n; i++ )
        ind[i] = i;

    k = n + 1;

    while( k GT 1 )
    {
        if( k GE 5 )
            k = (5 * k - 1) / 11;
        else
            k = 1;

        for ( i = n - k; i >= 0; i-- )
        {
            b = a[i];
            bi = ind[i];

            for ( j = i + k; j <= n && b > a[j]; j += k )
            {
                a[j - k] = a[j];
                ind[j - k] = ind[j];
            }
            a[j - k] = b;
            ind[j - k] = bi;
        }
    }
} /* end N_SortRealArray */

/*******************************************************************//**


   DESCRIPTION:

     This math routine sorts an index array using Shellsort. It also out-
     puts an index array relating old and new indexes.  A typical calling
     example is:

       NL_INDEX   *a;
       NL_INDEX   n, *ind;
       ...
       (get array a);
       ...
       N_SortIndexArrayMap(a,n,ind);


   ACCESS:
   
     a   , in/out ,  Array of type NL_INDEX
     n   , input  ,  Highest index in a and ind
     ind , output ,  Index array.  The output element a[i] is equal  to
                     the input element a[ind[i]]. Memory for this array
                     must be allocated in the calling routine


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SortIndexArrayMap( NL_INDEX *a, NL_INDEX n, NL_INDEX *ind )
{

    NL_INDEX i, j, k, b, bi;

    for ( i = 0; i <= n; i++ )
        ind[i] = i;

    k = n + 1;

    while( k GT 1 )
    {
        if( k GE 5 )
            k = (5 * k - 1) / 11;
        else
            k = 1;

        for ( i = n - k; i >= 0; i-- )
        {
            b = a[i];
            bi = ind[i];

            for ( j = i + k; j <= n && b > a[j]; j += k )
            {
                a[j - k] = a[j];
                ind[j - k] = ind[j];
            }
            a[j - k] = b;
            ind[j - k] = bi;
        }
    }
} /* end N_SortIndexArrayMap */

/*******************************************************************//**


   DESCRIPTION:

     This math routine sorts an index array using Shellsort.  A typical
     calling example is:

       NL_INDEX   *a;
       NL_INDEX   n;
       ...
       (get array a);
       ...
       N_SortIndexArray(a,n);


   ACCESS:
   
     a   , in/out ,  Array of type NL_INDEX
     n   , input  ,  Highest index in a


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SortIndexArray( NL_INDEX *a, NL_INDEX n )
{

    NL_INDEX i, j, k, b;

    k = n + 1;

    while( k GT 1 )
    {
        if( k GE 5 )
            k = (5 * k - 1) / 11;
        else
            k = 1;

        for ( i = n - k; i >= 0; i-- )
        {
            b = a[i];

            for ( j = i + k; j <= n && b > a[j]; j += k )
            {
                a[j - k] = a[j];
            }
            a[j - k] = b;
        }
    }
} /* end N_SortIndexArray */

/*******************************************************************//**


   DESCRIPTION:

     This math routine minimizes a function f(x) of 1 variable.  It uses
     Brent's method (a combination of parabolic interpolation and Golden
     section search).  For a detailed discussion of this algorithm, see:

       Numerical Recipes in C
       by W. Press, B. Flannery, S. Teukolsky and W. Vetterling
       Cambridge University Press  ,  1988

     This routine requires  that the minimum point already be bracketed,
     i.e., there exists xa,xb,xc satisfying:

       xa < xb < xc   and   f(xa) > f(xb) < f(xc).

     A typical calling example is as follows:

       NL_REAL    xa, xb, xc, fb, xtol, ftol, xmin, fmin;
       ...
       (find xa,xb,xc; evaluate fb, choose xtol,ftol, and define
        the NL_REAL function f)
       ...
       N_FuncFindMinima(xa,xb,xc,fb,f,xtol,ftol,&xmin,&fmin);


   ACCESS

     xa,xb , input  ,  The initial bracketing abscissa
     xc
     fb    , input  ,  The function value fb = f(xb)
     f     , input  ,  Pointer to the function to be minimized
     xtol  , input  ,  A tolerance (convergence criterion).  The  length
                       of the abscissa interval will not  be  made  less 
                       than  2*xtol*(midpoint of interval).  This yields
                       a "fractional" accuracy of xtol.  xtol should not
                       be set smaller than the square  root  of  machine
                       accuracy. This is about 3.0e-8 when using 64 bits 
                       for double-precision (NL_REAL)
     ftol  , input  ,  Absolute tolerance for measuring  convergence  to
                       the function minimum.  Iteration stops if f(x) <=
                       ftol.  This is useful if the known minimum should
                       be zero.  If this is not known,  set ftol to  the
                       smallest possible value
     xmin  , output ,  The abscissa of the local minimum
     fmin  , output ,  fmin = f(xmin).


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FuncFindMinima( NL_REAL xa, NL_REAL xb, NL_REAL xc, NL_REAL fb, NL_REAL( *f)(NL_REAL), NL_REAL xtol, NL_REAL ftol, NL_REAL *xmin, NL_REAL *fmin )
{

    NL_PRIVATE NL_INTEGER MXITS = 100;

    NL_INDEX its;

    NL_REAL a, b, gold, u, v, w, x, fu, fv, fw, fx, xm, p, q, r, e, etemp, d = 0.0, zeps, tol1, tol2;

    gold = 0.3819660; /*  The "golden mean" number.  */

    zeps = 1.0e-14;   /*  Determined this with a great deal of  */
    /*  experimentation. Don't change without */
    /*  doing same.                           */

    e = 0.0;

    if( xa LT xc ) /*  a and b must be in  */
    {              /*  ascending order.    */
        a = xa;
        b = xc;
    }
    else
    {
        a = xc;
        b = xa;
    }

    x = w = v = xb;                      /*  initialization  */
    fw = fv = fx = fb;

    for ( its = 1; its <= MXITS; its++ ) /*  the big loop  */
    {
        xm = 0.5 *( a + b );
        tol1 = xtol * fabs( x ) + zeps;
        tol2 = 2.0 *tol1;

        if( fabs( x - xm )LE tol2 - 0.5 *( b - a ) )
        {               /*  No point in making the  */
            *xmin = x;
            *fmin = fx; /*  interval any smaller.   */
            return;     /*  Stop here.              */
        }

        if( fabs( e )GT tol1 )
        { /*  construct a trial parabolic fit  */
            r = (x - w) * (fx - fv);
            q = (x - v) * (fx - fw);
            p = q * (x - v) - r * (x - w);
            q = 2.0 *( q - r );

            if( q GT 0.0 )
                p = -p;
            q = fabs( q );
            etemp = e;
            e = d;

            if( fabs( p )GE fabs( 0.5 *q *etemp )OR p LE q *( a - x )OR p GE q *( b - x ) )
            { /* Parabola no good; take golden section step instead. */
                if( x GE xm )
                    e = a - x;
                else
                    e = b - x;

                d = gold * e;
            }
            else
            { /* Take the parabolic step.  */
                d = p / q;
                u = x + d;

                if( u - a LT tol2 OR b - u LT tol2 )
                    d = ST_SIGN( tol1, xm - x );
            }
        }
        else
        { /*  Just take a golden section step.  */
            if( x GE xm )
                e = a - x;
            else
                e = b - x;

            d = gold * e;
        }

        if( fabs( d )GE tol1 )
            u = x + d;                  /* u is the new */
        else
            u = x + ST_SIGN( tol1, d ); /* candidate.   */

        fu = (*f)( u );

        if( fu LE ftol )
        { /*  Convergence  */
            *xmin = u;
            *fmin = fu;
            return;
        }

        if( fu LE fx )             /*  Now do bookkeeping to rein  */
        {                          /*  in the interval and update  */
            if( u GE x )
                a = x;             /*  the internal abscissa and   */
            else
                b = x;             /*  their corresponding         */
            ST_SHFT( v, w, x, u ); /*  function values.            */
            ST_SHFT( fv, fw, fx, fu );
        }
        else
        {
            if( u LT x )
                a = u;
            else
                b = u;

            if( fu LE fw OR w EQ x )
            {
                v = w;
                w = u;
                fv = fw;
                fw = fu;
            }
            else if( fu LE fv OR v EQ x OR v EQ w )
            {
                v = u;
                fv = fu;
            }
        }
    }           /* end of big loop */

    *xmin = x;
    *fmin = fx; /* Too many iterations if we get    */
    /* to here; but assign best values. */
    return;
} /* end N_FuncFindMinima */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  computes  (n+1)x(m+1)  approximately  evenly  spaced
     points on a surface. Optionally, the corresponding parameter values
     as well as the points may be returned.  The  surface  is given pro-
     cedurally,  i.e.  it is passed in as a function pointer.  A typical 
     calling example:

       NL_PARAMETER  u0, u1, v0, v1;
       NL_INDEX      n, m;
       NL_REAL       *u, *v, tol;
       NL_POINT      **P;
       ...
       (get u0,..., tol; get memory for P, u and v);
       (define S(u,v) as NL_FLAG S(NL_REAL u, NL_REAL v, NL_POINT *P));
       ...
       N_EvenSpacePtsSrf(S,u0,u1,v0,v1,n,m,tol,P   ,u   ,v   ); or
       N_EvenSpacePtsSrf(S,u0,u1,v0,v1,n,m,tol,P   ,NULL,NULL); or
       N_EvenSpacePtsSrf(S,u0,u1,v0,v1,n,m,tol,NULL,u   ,v   ); 


   ACCESS:
   
     S     , input  ,  The surface, S(u,v)
     u0,u1 , input  ,  The  n+1 points will be taken at parameter values
                       between u0 and u1
     v0,v1 , input  ,  The  m+1 points will be taken at parameter values
                       between v0 and v1
     n,m   , input  ,  (n+1)x(m+1) points are  to be generated (n,m > 1)
     tol   , input  ,  Tolerance used to place points equally along iso-
                       parametric  lines (see M_EVENSPACEPTSCRV). A  good default
                       is  1% of the  diagonal of the  surface's min-max
                       box.
     P     , output ,  Point array:
                         != NULL: (n+1)x(m+1) points generated
                          = NULL: no points are computed
                       MEMORY FOR  P  MUST BE  ALLOCATED IN  THE CALLING
                       ROUTINE TO HOLD UP TO P[n][m]!
     u,v   , output ,  Parameter arrays:
                         != NULL: (n+1)/(m+1) parameters are computed
                          = NULL: no parameters are computed
                       MEMORY  FOR  u  AND  v  MUST BE  ALLOCATED IN THE 
                       CALLING ROUTINE TO HOLD UP TO u[n] AND v[m]!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_EvenSpacePtsSrf( NL_FLAG(*S)( NL_REAL, NL_REAL, NL_POINT * ), NL_PARAMETER u0, NL_PARAMETER u1, NL_PARAMETER v0, NL_PARAMETER v1, NL_INDEX n, NL_INDEX m, NL_REAL tol, NL_POINT ** P, NL_PARAMETER *u, NL_PARAMETER *v )
{
    NL_PRIVATE NL_STRING rname = _T("N_EvenSpacePtsSrf");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_REAL *t, *ua, *va, *ue, *ve, du, dv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Set globals and check for errors */

    ST_EvalIsoGloS = S;

    if( n LE 1 OR m LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( u NEQ NULL AND v EQ NULL )
        NL_ERROR( NL_INP_ERR );

    if( v NEQ NULL AND u EQ NULL )
        NL_ERROR( NL_INP_ERR );

    /* Get local memory */

    t = N_AllocReal1dArray( NL_MAX( n, m ), &SL );

    if( t EQ NULL )
        NL_QUIT;

    ua = N_AllocReal1dArray( 2 * (n + m + 2), &SL );

    if( ua EQ NULL )
        NL_QUIT;

    va = &ua[n + 1];
    ue = &va[m + 1];
    ve = &ue[n + 1];

    /* Get average v parameters using uniform u's */

    ST_EvalIsoUvSwitch = 2;

    du = (u1 - u0) / n;

    for ( j = 0; j <= m; j++ )
        va[j] = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        if( i LT n )
            ST_EvalIsoGloUv = u0 + i * du;
        else
            ST_EvalIsoGloUv = u1;

        error = N_EvenSpacePtsCrv( ST_EvalIso, v0, v1, m, tol, NULL, t );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 1; j < m; j++ )
            va[j] += t[j];
    }

    va[0] = v0;
    va[m] = v1;

    for ( j = 1; j < m; j++ )
        va[j] /= (NL_REAL)n + 1.0;

    /* Get average u parameters using uniform v's */

    ST_EvalIsoUvSwitch = 1;

    dv = (v1 - v0) / m;

    for ( i = 0; i <= n; i++ )
        ua[i] = 0.0;

    for ( j = 0; j <= m; j++ )
    {
        if( j LT m )
            ST_EvalIsoGloUv = v0 + j * dv;
        else
            ST_EvalIsoGloUv = v1;

        error = N_EvenSpacePtsCrv( ST_EvalIso, u0, u1, n, tol, NULL, t );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 1; i < n; i++ )
            ua[i] += t[i];
    }

    ua[0] = u0;
    ua[n] = u1;

    for ( i = 1; i < n; i++ )
        ua[i] /= (NL_REAL)m + 1.0;

    /* Get equally spaced points in v-direction using average u's */

    ST_EvalIsoUvSwitch = 2;

    for ( j = 0; j <= m; j++ )
        ve[j] = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        ST_EvalIsoGloUv = ua[i];

        error = N_EvenSpacePtsCrv( ST_EvalIso, v0, v1, m, tol, NULL, t );

        if( error EQ NL_YES )
            NL_OUT;

        for ( j = 1; j < m; j++ )
            ve[j] += t[j];
    }

    ve[0] = v0;
    ve[m] = v1;

    for ( j = 1; j < m; j++ )
        ve[j] /= (NL_REAL)n + 1.0;

    /* Get equally spaced points in u-direction using average v's */

    ST_EvalIsoUvSwitch = 1;

    for ( i = 0; i <= n; i++ )
        ue[i] = 0.0;

    for ( j = 0; j <= m; j++ )
    {
        ST_EvalIsoGloUv = va[j];

        error = N_EvenSpacePtsCrv( ST_EvalIso, u0, u1, n, tol, NULL, t );

        if( error EQ NL_YES )
            NL_OUT;

        for ( i = 1; i < n; i++ )
            ue[i] += t[i];
    }

    ue[0] = u0;
    ue[n] = u1;

    for ( i = 1; i < n; i++ )
        ue[i] /= (NL_REAL)m + 1.0;

    /* Output parameters and/or points */

    if( u NEQ NULL OR v NEQ NULL )
    {
        for ( i = 0; i <= n; i++ )
            u[i] = ue[i];

        for ( j = 0; j <= m; j++ )
            v[j] = ve[j];
    }

    if( P NEQ NULL )
    {
        for ( i = 0; i <= n; i++ )
            for ( j = 0; j <= m; j++ )
            {
                error = S( ue[i], ve[j], &P[i][j] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_EvenSpacePtsSrf */

/*****************   Function  ST_EvalIso()  ******************/

NL_FLAG ST_EvalIso( NL_REAL s, NL_POINT *P )
{

    NL_FLAG error;

    if( ST_EvalIsoUvSwitch EQ 1 )
        error = ST_EvalIsoGloS( s, ST_EvalIsoGloUv, P );
    else
        error = ST_EvalIsoGloS( ST_EvalIsoGloUv, s, P );

    return (error);
}

/*******************************************************************//**


   DESCRIPTION:

     Given curve, C(u), this routine computes n+1 points on C(u),  which
     are  approximately  evenly  spaced.  Optionally,  the corresponding
     u-parameter  values as  well as  the points  may be  returned.  The
     curve is given procedurally,  i.e.  it is passed in as  a  function
     pointer. The iteration limit NL_ITLIM is defined as a global parameter
     in "globals.h". If NL_ITLIM is exceeded, the currently  best  solution  
     is returned. A typical calling example is:

       NL_PARAMETER  u0, u1;
       NL_INDEX      n;
       NL_REAL       tol, *u;
       NL_POINT      *P;
       ...
       (get u0, u1, tol, n, and get memory for P and u);
       (define C(u) as NL_FLAG C(NL_REAL u, NL_POINT *P));
       ...
       N_EvenSpacePtsCrv(C,u0,u1,n,tol,P   ,u   ); or
       N_EvenSpacePtsCrv(C,u0,u1,n,tol,P   ,NULL); or
       N_EvenSpacePtsCrv(C,u0,u1,n,tol,NULL,u   ); 


   ACCESS:
   
     C     , input  ,  Curve, C(u)
     u0,u1 , input  ,  The n+1 points will be taken at parameter values
                       between u0 and u1.
     n     , input  ,  n+1 points are to be generated ( n > 1 ).
     tol   , input  ,  Let aver be the average distance  between neigh-
                       boring points on the curve,  and  let maxdev  be
                       the  maximum deviation of  any  two  neighboring
                       points  from  the average.  Then  maxdev/aver <=
                       tol.
     P     , output ,  The n+1 approximately evenly spaced points. This 
                       memory must be allocated in the calling routine.
                       if (P == NULL), then this is not used.
     u     , output ,  If (u == NULL),  then  this   is  not  used  (no
                       parameters  returned). If (u != NULL),  then  it
                       is assumed that u is an array of length at least 
                       n+1.  The   parameter   values  of  C(u),  which 
                       correspond to the points in  P, are  returned in 
                       this array.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_EvenSpacePtsCrv( NL_FLAG(*C)( NL_REAL, NL_POINT * ), NL_PARAMETER u0, NL_PARAMETER u1, NL_INDEX n, NL_REAL tol, NL_POINT *P, NL_PARAMETER *u )
{
    NL_PRIVATE NL_STRING rname = _T("N_EvenSpacePtsCrv");

    NL_FLAG error = NL_NO;

    NL_INDEX i, k, its;

    NL_REAL *t, *oldt, *s, *temp, aver, dt, d, num, den, ztol;

    NL_POINT *Q;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Check for error in input */

    if( n LE 1 )
        NL_ERROR( NL_INP_ERR );

    ztol = NL_MIN( NL_MTOL, NL_PTOL );

    /* Allocate arrays and initialize the t-array */

    Q = N_AllocPt1dArray( n, &S );

    if( Q EQ NULL )
        NL_QUIT;

    t = N_AllocReal1dArray( 3 * (n + 1), &S );

    if( t EQ NULL )
        NL_QUIT;

    oldt = &t[n + 1];
    s = &oldt[n + 1];

    oldt[0] = u0;
    oldt[n] = u1;
    t[0] = u0;
    t[n] = u1;
    dt = (u1 - u0) / n;

    for ( i = 1; i < n; i++ )
        t[i] = u0 + i * dt;

    /* Initialize start point */

    error = C( u0, &Q[0] );

    if( error EQ NL_YES )
        NL_OUT;

    /* Iteratively compute distances, and linearly interpolate */
    /* to improve them                                         */

    s[0] = 0.0;

    for ( its = 1; its <= NL_ITLIM; its++ )
    {
        /* Compute points and distances */

        for ( i = 1; i <= n; i++ )
        {
            error = C( t[i], &Q[i] );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( Q[i - 1], Q[i], &d );
            s[i] = s[i - 1] + d;
        }

        aver = s[n] / n;

        if( s[n]LE ztol ) /* degenerate curve */
            i = n + 1;
        else
        {
            /* Compute deviations */

            for ( i = 1; i <= n; i++ )
            {
                d = fabs( s[i] - s[i - 1] - aver );

                if( N_FloatOpIsBad( d, aver, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );

                if( d / aver GT tol )
                    break;
            }
        }

        /* Convergence              -> Load u array and return. */
        /* Exceeded iteration limit -> Return best result.      */

        if( i GT n OR its GE NL_ITLIM )
        {
            if( u NEQ NULL )
            {
                for ( k = 0; k <= n; k++ )
                    u[k] = t[k];
            }

            if( P NEQ NULL )
            {
                for ( k = 0; k <= n; k++ )
                {
                    N_CopyPt( Q[k], &P[k] );
                }
            }

            if( its GE NL_ITLIM AND i LE n )
                NL_ERROR( NL_CON_ERR );

            NL_OUT;
        }

        /* Recompute t-values using linear interpolation */

        temp = t;
        t = oldt;
        oldt = temp;

        k = 1;

        for ( i = 1; i < n; i++ )
        {
            d = i * aver;

            while( d GT s[k] )
                k++;
            num = oldt[k] - oldt[k - 1];
            den = s[k] - s[k - 1];

            if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            t[i] = (num / den) * (d - s[k - 1]) + oldt[k - 1];
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_EvenSpacePtsCrv */

/*******************************************************************//**


   DESCRIPTION:

     This math routine computes the inverse of a real matrix using the
     method of Single Value Decompostion. A typical calling example is
     as follows:

       NL_REAL     **R, **RI;
       ...
       (compute R and allocate memory RI);
       ...
       N_Real2dArrayInverseSVD(R,n,RI);


   ACCESS:
   
     R  , input  ,  Real square matrix
     n  , input  ,  R and RI are (n+1)x(n+1) square matrices
     RI , output ,  Inverse of R. This memory must be allocated in the
                    calling routine


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Real2dArrayInverseSVD( NL_REAL ** R, NL_INDEX n, NL_REAL ** RI )
{

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj;

    NL_REAL *W, ** V, ** U, *x, *y;

    NL_STACKS SL;

    N_InitNurbs( &SL );
    /* Allocate work memory */

    W = N_AllocReal1dArray( n, &SL );
    V = N_AllocReal2dArray( n, n, &SL );

    if( W EQ NULL OR V EQ NULL )
        NL_QUIT;

    U = N_AllocReal2dArray( n, n, &SL );

    if( U EQ NULL )
        NL_QUIT;

    x = N_AllocReal1dArray( 2 * n + 2, &SL );

    if( x EQ NULL )
        NL_QUIT;

    y = &x[n + 1];

    /* Compute single value decompostion of R */

    for ( ii = 0; ii <= n; ii++ )
        for ( jj = 0; jj <= n; jj++ )
            U[ii][jj] = R[ii][jj];

    error = N_RealMatrixSingleValueDecompose( U, W, V, n, n );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the inverse */

    y[0] = 1.0;

    for ( ii = 1; ii <= n; ii++ )
        y[ii] = 0.0;

    error = N_SingleValueDecomposeSolve( U, W, V, n, n, (NL_VOID *)y, NL_RVALUE, NL_YES, (NL_VOID *)x );

    if( error EQ NL_YES )
        NL_OUT;

    for ( ii = 0; ii <= n; ii++ )
        RI[ii][0] = x[ii];

    for ( ii = 1; ii <= n; ii++ )
    {
        y[ii - 1] = 0.0;
        y[ii] = 1.0;

        error = N_SingleValueDecomposeSolve( U, W, V, n, n, (NL_VOID *)y, NL_RVALUE, NL_NO, (NL_VOID *)x );

        if( error EQ NL_YES )
            NL_OUT;

        for ( jj = 0; jj <= n; jj++ )
            RI[jj][ii] = x[jj];
    }

    /* Exit */

    EXIT:
    N_EndNurbs( &SL );

    return (error);
} /* end N_Real2dArrayInverseSVD */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  computes the  inverse of a real  matrix  using 
     single value decomposition. If the output matrix is initialized to 
     NULL, memory  is  allocated. If not, the routine  checks if enough 
     memory is passed in. A typical calling example is as follows:

       NL_RMATRIX  rma, rmb;
       NL_STACKS   SB;
       ...
       (get rma);
       ...
       N_InitRealMatrix(&rmb);
       N_RealMatrixInverseSVD(&rma,&rmb,&SB);

     IT IS  ASSUMED THAT  MEMORY  TO STORE  THE  MATRIX  STRUCTURES  IS 
     ALLOCATED IN THE CALLING  ROUTINE.


   ACCESS:
   
     rma , input  ,  Real matrix
     rmb , output ,  Inverse of rma
     SB  , input  ,  rmb's stack 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixInverseSVD( NL_RMATRIX *rma, NL_RMATRIX *rmb, NL_STACKS *SB )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixInverseSVD");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, bw;

    NL_REAL ** A, ** B;

    NL_MATRIXTYPE mtp;

    /* Get local notation and check input */

    N_GetRealMatrixData( rma, &n, &m, &A, &mtp, &bw );

    if( n NEQ m )
        NL_ERROR( NL_IND_ERR );

    if( mtp NEQ NL_MT_FULL )
        NL_ERROR( NL_INP_ERR );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( rmb, n, m, NL_MT_FULL, n, rname, SB );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rmb, &B );

    /* Get inverse */

    error = N_Real2dArrayInverseSVD( A, n, B );

    if( error EQ NL_YES )
        NL_OUT;

    /* Exit */

    EXIT:

    return (error);
} /* end N_RealMatrixInverseSVD */

/*******************************************************************//**


   DESCRIPTION:

     This math routine generates a random number between 0 and RAND_MAX.
     A typical calling example is:

       NL_INTEGER  rn;
       ...
       rn = N_GenerateRandomNumber();


   ACCESS:
   
     None


   RETURN CODES:

     rn: random number in the range of 0 to RAND_MAX

   ***********************************************************************/

NL_INTEGER N_GenerateRandomNumber( NL_VOID )
{

    static unsigned long int next = 7;

    NL_INTEGER rn;

    next = next * 1103515245 + 12345;
    rn = (NL_INTEGER)( next / 65536 ) % RAND_MAX;

    return (rn);
} /* end N_GenerateRandomNumber */

/*******************************************************************//**


   DESCRIPTION:

     This math routine generates a random sequence of flips and flops.
     A typical calling example is:

       NL_BOOLEAN  ff;
       ...
       ff = N_RandomFlipsFlops();


   ACCESS:
   
     None


   RETURN CODES:

     ff: random flip or flop

   ***********************************************************************/

NL_BOOLEAN N_RandomFlipsFlops( NL_VOID )
{

    NL_INTEGER rn;

    rn = N_GenerateRandomNumber();

    if( rn % 2 )
        return NL_FLOP;
    else
        return NL_FLIP;
} /* end N_RandomFlipsFlops */

/*******************************************************************//**


   DESCRIPTION:

     Given a sorted  integer array, this routine finds an element in the
     array. The method uses interpolation search to achieve a log(log N) 
     performance. A typical calling example:

       NL_INDEX  *u, t, m, j;
       ... 
       (get u and t);
       ...
       N_FindElemIntArray(u,m,t,&j);


   ACCESS:
   
     u   , input  ,  Sorted integer array
     m   , input  ,  Highest index in u
     t   , input  ,  Integer value 
     j   , output ,  Index of array element t is equal to, i.e., u[j]=t.
                     If t is not in u, j = -1 is returned!


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FindElemIntArray( NL_INDEX *u, NL_INDEX m, NL_INDEX t, NL_INDEX *j )
{

    NL_INDEX i = 0, inc, l, r;

    NL_REAL num, den, f;

    /* Find element */

    *j = -1;

    if( t GE u[0]OR t LE u[m] )
    {
        l = 0;
        r = m;

        while( r GE l )
        {
            if( t LT u[l] )
            {
                *j = -1;
                break;
            }

            if( t GT u[r] )
            {
                *j = -1;
                break;
            }

            num = (NL_REAL)t - u[l];
            den = (NL_REAL)u[r] - (NL_REAL)u[l];

            if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            {
                inc = 0;
            }
            else
            {
                f = num / den;
                inc = (NL_INDEX)( f * ((NL_REAL)r - (NL_REAL)l) );
                i = l + inc;
            }

            if( t EQ u[i] )
            {
                *j = i;
                break;
            }

            if( t LT u[i] )
                r = i - 1;
            else
                l = i + 1;
        }
    }
} /* end N_FindElemIntArray */

/*******************************************************************//**


   DESCRIPTION:

     Given a sorted  integer array, this  routine finds  the interval a 
     given value is in. The method uses interpolation search to achieve 
     an approximate log(log N) performance. A typical calling example:

       NL_INDEX  *u, t, m, j;
       ... 
       (get u and t);
       ...
       N_FindIntervalIntArray(u,m,t,NL_LEFT,&j);


   ACCESS:
   
     u   , input  ,  Sorted integer array
     m   , input  ,  Highest index in u
     t   , input  ,  Integer value 
     flg , input  ,  Flag:
                       NL_LEFT : t is in [u[j],u[j+1]) 
                       NL_RIGHT: t is in (u[j],u[j+1]] 
                       NL_NONE : t is in (u[j],u[j+1]) 
     j   , output ,  Left  index of  interval t  is in. If t is outside
                     (u[0],u[m]), j = -1 is returned!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FindIntervalIntArray( NL_INDEX *u, NL_INDEX m, NL_INDEX t, NL_FLAG flg, NL_INDEX *j )
{
    NL_PRIVATE NL_STRING rname = _T("N_FindIntervalIntArray");

    NL_FLAG error = NL_NO;

    NL_INDEX i, inc, l, r;

    NL_REAL f;

    /* Check value */

    if( t LT u[0]OR t GT u[m] )
    {
        *j = -1;
        NL_OUT;
    }

    /* Find interval */

    l = 0;
    r = m;
    f = ( (NL_REAL)t - (NL_REAL)u[l] ) / ( (NL_REAL)u[r] - (NL_REAL)u[l] );
    i = l + (NL_INDEX)( f * ((NL_REAL)r - (NL_REAL)l) );

    switch( flg )
    {
        case NL_LEFT: /* t must be in [u[j],u[j+1]) */
            if( t EQ u[m] )
            {
                *j = -1;
                NL_OUT;
            }

            if( t GE u[0]AND t LT u[1] )
            {
                *j = 0;
                NL_OUT;
            }

            if( t GE u[m - 1]AND t LT u[m] )
            {
                *j = m - 1;
                NL_OUT;
            }

            while( t LT u[i]OR t GE u[i + 1] )
            {
                if( t LT u[i] )
                    r = i;
                else
                    l = i;
                f = ( (NL_REAL)t - (NL_REAL)u[l] ) / ( (NL_REAL)u[r] - (NL_REAL)u[l] );
                inc = (NL_INDEX)( f * ((NL_REAL)r - (NL_REAL)l) );

                if( inc EQ 0 AND r GT( l + 1 ) )
                    inc = 1;

                i = l + inc;
            }
            break;

        case NL_RIGHT: /* t must be in (u[j],u[j+1]] */
            if( t EQ u[0] )
            {
                *j = -1;
                NL_OUT;
            }

            if( t GT u[0]AND t LE u[1] )
            {
                *j = 0;
                NL_OUT;
            }

            if( t GT u[m - 1]AND t LE u[m] )
            {
                *j = m - 1;
                NL_OUT;
            }

            while( t LE u[i]OR t GT u[i + 1] )
            {
                if( t LT u[i] )
                    r = i;
                else
                    l = i;
                f = ( (NL_REAL)t - (NL_REAL)u[l] ) / ( (NL_REAL)u[r] - (NL_REAL)u[l] );
                inc = (NL_INDEX)( f * ((NL_REAL)r - (NL_REAL)l) );

                if( inc EQ 0 AND r GT( l + 1 ) )
                    inc = 1;

                i = l + inc;
            }
            break;

        case NL_NONE: /* t must be in (u[j],u[j+1]) */
            if( t EQ u[0] )
            {
                *j = -1;
                NL_OUT;
            }

            if( t EQ u[m] )
            {
                *j = -1;
                NL_OUT;
            }

            if( t GT u[0]AND t LT u[1] )
            {
                *j = 0;
                NL_OUT;
            }

            if( t GT u[m - 1]AND t LT u[m] )
            {
                *j = m - 1;
                NL_OUT;
            }

            while( t LE u[i]OR t GE u[i + 1] )
            {
                if( t EQ u[i]OR t EQ u[i + 1] )
                {
                    *j = -1;
                    NL_OUT;
                }

                if( t LT u[i] )
                    r = i;
                else
                    l = i;
                f = ((NL_REAL)t - (NL_REAL)u[l] ) / ( (NL_REAL)u[r] - (NL_REAL)u[l] );
                inc = (NL_INDEX)( f * ((NL_REAL)r - (NL_REAL)l) );

                if( inc EQ 0 AND r GT( l + 1 ) )
                    inc = 1;

                i = l + inc;
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    *j = i;

    /* Exit */

    EXIT:

    return (error);
} /* end N_FindIntervalIntArray */

/*******************************************************************//**


   DESCRIPTION:

     Given a sorted real array, this routine finds the interval a given 
     value is in. The method  uses interpolation  search to  achieve an  
     approximate log(log N) performance. A typical calling example is:

       NL_REAL   *u, t;
       NL_INDEX  m, j;
       ... 
       (get u and t);
       ...
       N_FindIntervalRealArray(u,m,t,NL_LEFT,&j);


   ACCESS:
   
     u   , input  ,  Sorted real array
     m   , input  ,  Highest index in u
     t   , input  ,  Real value 
     flg , input  ,  Flag:
                       NL_LEFT : t is in [u[j],u[j+1]) 
                       NL_RIGHT: t is in (u[j],u[j+1]] 
                       NL_NONE : t is in (u[j],u[j+1]) 
     j   , output ,  Left  index of  interval t  is in. If t is outside
                     (u[0],u[m]), j = -1 is returned!


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FindIntervalRealArray( NL_REAL *u, NL_INDEX m, NL_REAL t, NL_FLAG flg, NL_INDEX *j )
{
    NL_PRIVATE NL_STRING rname = _T("N_FindIntervalRealArray");

    NL_FLAG error = NL_NO;

    NL_INDEX i, inc, l, r;

    NL_REAL f;

    /* Check value */

    if( t LT u[0]OR t GT u[m] )
    {
        *j = -1;
        NL_OUT;
    }

    /* Find interval */

    l = 0;
    r = m;
    f = (t - u[l]) / (u[r] - u[l]);
    i = l + (NL_INDEX)( f * ((NL_REAL)r - (NL_REAL)l) );

    switch( flg )
    {
        case NL_LEFT: /* t must be in [u[j],u[j+1]) */
            if( t EQ u[m] )
            {
                *j = -1;
                NL_OUT;
            }

            if( t GE u[0]AND t LT u[1] )
            {
                *j = 0;
                NL_OUT;
            }

            if( t GE u[m - 1]AND t LT u[m] )
            {
                *j = m - 1;
                NL_OUT;
            }

            while( t LT u[i]OR t GE u[i + 1] )
            {
                if( t LT u[i] )
                    r = i;
                else
                    l = i;
                f = (t - u[l]) / (u[r] - u[l]);
                inc = (NL_INDEX)( f * ((NL_REAL)r - (NL_REAL)l) );

                if( inc EQ 0 AND r GT( l + 1 ) )
                    inc = 1;

                i = l + inc;
            }
            break;

        case NL_RIGHT: /* t must be in (u[j],u[j+1]] */
            if( t EQ u[0] )
            {
                *j = -1;
                NL_OUT;
            }

            if( t GT u[0]AND t LE u[1] )
            {
                *j = 0;
                NL_OUT;
            }

            if( t GT u[m - 1]AND t LE u[m] )
            {
                *j = m - 1;
                NL_OUT;
            }

            while( t LE u[i]OR t GT u[i + 1] )
            {
                if( t LT u[i] )
                    r = i;
                else
                    l = i;
                f = (t - u[l]) / (u[r] - u[l]);
                inc = (NL_INDEX)( f * ((NL_REAL)r - (NL_REAL)l) );

                if( inc EQ 0 AND r GT( l + 1 ) )
                    inc = 1;

                i = l + inc;
            }
            break;

        case NL_NONE: /* t must be in (u[j],u[j+1]) */
            if( t EQ u[0] )
            {
                *j = -1;
                NL_OUT;
            }

            if( t EQ u[m] )
            {
                *j = -1;
                NL_OUT;
            }

            if( t GT u[0]AND t LT u[1] )
            {
                *j = 0;
                NL_OUT;
            }

            if( t GT u[m - 1]AND t LT u[m] )
            {
                *j = m - 1;
                NL_OUT;
            }

            while( t LE u[i]OR t GE u[i + 1] )
            {
                if( t EQ u[i]OR t EQ u[i + 1] )
                {
                    *j = -1;
                    NL_OUT;
                }

                if( t LT u[i] )
                    r = i;
                else
                    l = i;
                f = (t - u[l]) / (u[r] - u[l]);
                inc = (NL_INDEX)( f * ((NL_REAL)r - (NL_REAL)l) );

                if( inc EQ 0 AND r GT( l + 1 ) )
                    inc = 1;

                i = l + inc;
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    *j = i;

    /* Exit */

    EXIT:

    return (error);
} /* end N_FindIntervalRealArray */

/*******************************************************************//**


   DESCRIPTION:

     This math routine  computes a tangent vector via the 3-point Bessel
     method. A typical calling example is as follows:

       NL_POINT   P[3];
       NL_VECTOR  T;
       NL_REAL    u[3]
       ...
       (load 3 points into P, and optionally, parameters into u);
       ...

       N_TangentVectorBessel(P,1,u,NL_YES,&T);


   ACCESS:
   
     P    , input  ,  Three points
     idx  , input  ,  Index of the point at which to compute the tangent
                      ( idx = 0, 1 or 2 )
     u    , input  ,  Parameter values at 3 points (optional):
                         = NULL : this function will compute parameters
                        != NULL : use the values in u
     nflg , input  ,  Flag:
                         = NL_YES : normalize the tangent vector
                         =  NL_NO : do not normalize the tangent
     T    , output ,  The tangent vector at P[idx]


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TangentVectorBessel( NL_POINT *P, NL_INDEX idx, NL_REAL *u, NL_FLAG nflg, NL_VECTOR *T )
{
    NL_PRIVATE NL_STRING rname = _T("N_TangentVectorBessel");

    NL_FLAG error = NL_NO;

    NL_REAL uu[3], oma, alf, du, du1;

    NL_VECTOR S[2], V;

    /* Compute the chords */

    N_VectorDiff( P[1], P[0], &S[0] );
    N_VectorDiff( P[2], P[1], &S[1] );

    /* Compute the tangents */

    if( u EQ NULL )
    {
        error = N_FitCalcCrvParamValues( (NL_VOID *)P, 2, NL_EPOINT, NL_CHORDLENGTH, uu );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        uu[0] = u[0];
        uu[1] = u[1];
        uu[2] = u[2];
    }

    du = uu[1] - uu[0];
    du1 = uu[2] - uu[1];

    if( N_FloatOpIsBad( du, du + du1, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    alf = du / (du + du1);
    oma = 1.0 - alf;

    if( N_FloatOpIsBad( oma, du, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    if( N_FloatOpIsBad( alf, du1, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    oma = oma / du;
    alf = alf / du1;

    N_VectorCombine( oma, S[0], alf, S[1], &V );

    if( idx EQ 1 )
    {
        N_VectorCopy( V, T );
    }
    else
    {
        if( idx EQ 0 )
            du = uu[1] - uu[0];
        else
            du = uu[2] - uu[1];

        if( N_FloatOpIsBad( 2.0, du, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        alf = 2.0 / du;

        N_VectorCombine( alf, S[(idx + 1) / 2], -1.0, V, T );
    }

    if( nflg EQ NL_YES )
    {
        error = N_VectorNormalizeRef( T );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_TangentVectorBessel */

/*******************************************************************//**


   DESCRIPTION:

     This math routine  computes  a tangent vector via the 5-point Akima
     method. A typical calling example is as follows:

       NL_POINT   P[5];
       NL_VECTOR  T;
       ...
       (load 5 points into P);
       ...

       N_TangentVectorAkima(P,2,NL_YES,&T);


   ACCESS:
   
     P    , input  ,  Five points
     idx  , input  ,  Index of the point at which to compute the tangent
                      ( idx = 0, 1, 2, 3 or 4 )
     nflg , input  ,  Flag:
                         = NL_YES : normalize the tangent vector
                         =  NL_NO : do not normalize the tangent
     T    , output ,  The tangent vector at P[idx]


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TangentVectorAkima( NL_POINT *P, NL_INDEX idx, NL_FLAG nflg, NL_VECTOR *T )
{

    NL_FLAG error = NL_NO;

    NL_INDEX ii;

    NL_REAL oma, alf, a, b;

    NL_VECTOR S[8], V;

    /* Compute the chords */

    for ( ii = 1; ii <= 4; ii++ )
        N_VectorDiff( P[ii], P[ii - 1], &S[ii + 1] );

    if( idx LT 2 )
    {
        N_VectorCombine( 2.0, S[2], -1.0, S[3], &S[1] );
        N_VectorCombine( 2.0, S[1], -1.0, S[2], &S[0] );
    }

    if( idx GT 2 )
    {
        N_VectorCombine( 2.0, S[5], -1.0, S[4], &S[6] );
        N_VectorCombine( 2.0, S[6], -1.0, S[5], &S[7] );
    }

    /* Compute the tangents */

    N_VectorCross( S[idx], S[idx + 1], &V );
    N_VectorMagnitude( V, &a );

    N_VectorCross( S[idx + 2], S[idx + 3], &V );
    N_VectorMagnitude( V, &b );

    if( (a + b)GT NL_LTOL )
        alf = a / (a + b);
    else
        alf = 0.5;

    oma = 1.0 - alf;

    N_VectorCombine( oma, S[idx + 1], alf, S[idx + 2], T );

    if( nflg EQ NL_YES )
    {
        error = N_VectorNormalizeRef( T );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_TangentVectorAkima */

/*******************************************************************//**


   DESCRIPTION:

     This math routine  computes the eigenvalues and the eigenvectors of
     a matrix.  First it  reduces the  matrix into a  tridiagonal matrix 
     with Householder  reduction. Then  it computes  the eigenvalues and 
     the eigenvectors via QL  decomposition with implicit shifts. If the 
     output matrix is initiaized  to NULL, memory  is allocated. If not, 
     the routine checks if enough memory is passed in. A typical calling 
     example is as follows:

       NL_RMATRIX  rma, rmb;
       NL_REAL     *v;
       NL_STACKS   SB;
       ...
       (get rma, and allocate memory for v);
       ...
       N_InitRealMatrix(&rmb);
       N_RealMatrixEigenValuesVectors(&rma,v,&rmb,&SB);

     IT IS ASSUMED THAT MEMORIES TO STORE "v" AND THE  MATRIX  STRUCTURE
     "rmb" ARE ALLOCATED IN THE CALLING  ROUTINE. 


   ACCESS:
   
     rma , input  ,  Real matrix
     v   , output ,  Eigenvalues
     rmb , output ,  Matrix of  eigenvectors; the NL_NORMALIZED eigenvector
                     corresponding to  v[k] is stored on the k-th column 
                     of rmb
     SB  , input  ,  rmb's stack 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_RealMatrixEigenValuesVectors( NL_RMATRIX *rma, NL_REAL *v, NL_RMATRIX *rmb, NL_STACKS *SB )
{
    NL_PRIVATE NL_STRING rname = _T("N_RealMatrixEigenValuesVectors");

    NL_PRIVATE NL_INDEX itl = 30;

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, n, m, iter;

    NL_REAL ** A, ** B, ** T, *d, *e, f, g, h, hh, scale, dd, c, b, p, r, s;

    NL_MATRIXTYPE mtp;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation and check input */

    N_GetRealMatrixData( rma, &n, &m, &T, &mtp, &i );

    if( n NEQ m )
        NL_ERROR( NL_IND_ERR );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( rmb, n, n, NL_MT_FULL, n, rname, SB );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rmb, &B );

    d = N_AllocReal1dArray( n, &SL );

    if( d EQ NULL )
        NL_QUIT;

    e = N_AllocReal1dArray( n, &SL );

    if( e EQ NULL )
        NL_QUIT;

    A = N_AllocReal2dArray( n, n, &SL );

    if( A EQ NULL )
        NL_QUIT;

    /* Preserve input data */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= n; j++ )
            A[i][j] = T[i][j];
    }

    /* Reduce matrix to tridiagonal form */

    for ( i = n; i >= 1; i-- )
    {
        l = i - 1;
        h = scale = 0.0;

        if( l GT 0 )
        {
            for ( k = 0; k <= l; k++ )
                scale += fabs( A[i][k] );

            if( scale LE NL_SMAD )
            {
                e[i] = A[i][l];
            }
            else
            {
                for ( k = 0; k <= l; k++ )
                {
                    if( N_FloatOpIsBad( A[i][k], scale, NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    A[i][k] /= scale;
                    h += A[i][k] * A[i][k];
                }

                f = A[i][l];

                if( f GE 0.0 )
                    g = -sqrt( h );
                else
                    g = sqrt( h );

                e[i] = scale * g;
                h -= f * g;
                A[i][l] = f - g;

                f = 0.0;

                for ( j = 0; j <= l; j++ )
                {
                    if( N_FloatOpIsBad( A[i][j], h, NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    A[j][i] = A[i][j] / h;

                    g = 0;

                    for ( k = 0; k <= j; k++ )
                        g += A[j][k] * A[i][k];

                    for ( k = j + 1; k <= l; k++ )
                        g += A[k][j] * A[i][k];

                    if( N_FloatOpIsBad( g, h, NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    e[j] = g / h;
                    f += e[j] * A[i][j];
                }

                if( N_FloatOpIsBad( f, h + h, NL_DIVISION ) )
                    NL_ERROR( NL_NUM_ERR );
                hh = f / (h + h);

                for ( j = 0; j <= l; j++ )
                {
                    f = A[i][j];
                    e[j] = g = e[j] - hh * f;

                    for ( k = 0; k <= j; k++ )
                        A[j][k] -= (f * e[k] + g * A[i][k]);
                }
            }
        }
        else
        {
            e[i] = A[i][l];
        }

        d[i] = h;
    }

    d[0] = e[0] = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        l = i - 1;

        if( d[i] )
        {
            for ( j = 0; j <= l; j++ )
            {
                g = 0.0;

                for ( k = 0; k <= l; k++ )
                    g += A[i][k] * A[k][j];

                for ( k = 0; k <= l; k++ )
                    A[k][j] -= g * A[k][i];
            }
        }

        d[i] = A[i][i];
        A[i][i] = 1.0;

        for ( j = 0; j <= l; j++ )
            A[j][i] = A[i][j] = 0.0;
    }

    /* Got tridiagonal matrix; d[0..n] contains the diagonal elements,    */
    /* whereas e[0..n] has the sugdiagonal elements. Use these to compute */
    /* the eigenvalues and the eigenvectors.                              */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= n; j++ )
            B[i][j] = A[i][j];
    }

    for ( i = 1; i <= n; i++ )
        e[i - 1] = e[i];
    e[n] = 0.0;

    for ( l = 0; l <= n; l++ )
    {
        iter = 0;

        do
        {
            for ( m = l; m <= n - 1; m++ )
            {
                dd = fabs( d[m] ) + fabs( d[m + 1] );

                if( (NL_REAL)( fabs( e[m] ) + dd )EQ dd )
                    break;
            }

            if( m NEQ l )
            {
                if( iter++GE itl )
                    NL_ERROR( NL_CON_ERR );

                g = (d[l + 1] - d[l]) / (2.0 *e[l]);
                r = ST_pythag( g, 1.0 );
                g = d[m] - d[l] + e[l] / (g + NL_SIAB( r, g ));

                s = c = 1.0;
                p = 0.0;

                for ( i = m - 1; i >= l; i-- )
                {
                    f = s * e[i];
                    b = c * e[i];
                    e[i + 1] = (r = ST_pythag( f, g ));

                    if( r LE NL_SMAD )
                    {
                        d[i + 1] -= p;
                        e[m] = 0.0;
                        break;
                    }

                    if( N_FloatOpIsBad( f, r, NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    s = f / r;

                    if( N_FloatOpIsBad( g, r, NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );
                    c = g / r;

                    g = d[i + 1] - p;
                    r = (d[i] - g) * s + 2.0 *c * b;
                    d[i + 1] = g + (p = s * r);
                    g = c * r - b;

                    for ( k = 0; k <= n; k++ )
                    {
                        f = B[k][i + 1];

                        B[k][i + 1] = s * B[k][i] + c * f;
                        B[k][i] = c * B[k][i] - s * f;
                    }
                }

                if( r LE NL_SMAD AND i GE l )
                    continue;

                d[l] -= p;
                e[l] = g;
                e[m] = 0.0;
            }
        } while ( m NEQ l );

        v[l] = d[l];
    }

    /* Exit */

    N_EndNurbs( &SL );

    EXIT:

    return (error);
} /* end N_RealMatrixEigenValuesVectors */

/*********************************************************************/
/* ST_pythag: Compute sqrt(a^2+b^2) without under/over-flow           */
/*********************************************************************/

NL_REAL ST_pythag( NL_REAL a, NL_REAL b )
{

    NL_REAL ava, avb, fac, res;

    ava = fabs( a );
    avb = fabs( b );

    if( ava GT avb )
    {
        if( N_FloatOpIsBad( avb, ava, NL_DIVISION ) )
            return 0.0;

        fac = avb / ava;
        res = ava * (sqrt( 1.0 + fac * fac ));

        return res;
    }
    else
    {
        if( N_FloatOpIsBad( ava, avb, NL_DIVISION ) )
            return 0.0;

        fac = ava / avb;
        res = avb * (sqrt( 1.0 + fac * fac ));

        return res;
    }
}

/*******************************************************************//**


   DESCRIPTION:

     Given a  sorted  real array, this  routine finds  clusters  in the 
     array, i.e. it finds groups of numbers whose interval span is less
     than a given distance. A typical calling example is:

       NL_REAL    *t, *s, **u;
       NL_INDEX   *ud, k, l, cdt;
       NL_STACKS  SG;
       ... 
       (get array t);
       ...
       N_FindClustersRealArray(t,k,cdt,NL_PARAMETERS,&s,&l,&u,&ud,&SG);
       N_FindClustersRealArray(t,k,cdt,NL_CLUSTERS,&s,&l,&u,&ud,&SG);
       N_FindClustersRealArray(t,k,cdt,NL_BOTH,&s,&l,&u,&ud,&SG);

     MEMORY TO STORE THE ARRAYS s, u AND ud IS ALLOCATED IN THE CALLING
     ROUTINE.


   ACCESS:
   
     t   , input  ,  Sorted real array
     k   , input  ,  Highest index in t
     cdt , input  ,  Cluster  distance, i.e.  all  the  elements in the
                     cluster are in an interval of length cdt
     cfl , input  ,  Flag:
                       NL_PARAMETERS: return parameters  representing each
                                   cluster, i.e. the average of cluster
                                   elements
                       NL_CLUSTERS: return each element of every cluster
                       NL_BOTH: return both
     s   , output ,  cluster representatives
     l   , output ,  highest index in s and in ud
     u   , output ,  clusters:
                       u[0][0..ud[0]]: elements of first cluster
                       ...
                       u[l][0..ud[l]]: elements of last cluster
     ud  , output ,  highest indexes in u
     SG  , input  ,  stack of s, u, and ud


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FindClustersRealArray( NL_REAL *t, NL_INDEX k, NL_REAL cdt, NL_FLAG cfl, NL_REAL ** s, NL_INDEX *l, NL_REAL *** u, NL_INDEX ** ud, NL_STACKS *SG )
{

    NL_FLAG error = NL_NO;

    NL_INDEX *nl, i, j, is, ie, ib, r, a;

    NL_REAL ** ul, *sl, dmax, d, sum, avp, avn;

    /* Allocate memories */

    sl = N_AllocReal1dArray( k, SG );

    if( sl EQ NULL )
        NL_QUIT;

    ul = N_AllocRealPtr1dArray( k, SG );

    if( ul EQ NULL )
        NL_QUIT;

    nl = N_AllocInt1dArray( k, SG );

    if( nl EQ NULL )
        NL_QUIT;

    /* Find clusters */

    is = 0;
    ie = k;
    ib = -1;
    r = -1;

    while( ib NEQ ie )
    {
        if( is EQ ie )
        {
            r++;

            if( cfl EQ NL_PARAMETERS OR cfl EQ NL_BOTH )
            {
                sl[r] = t[ie];
            }

            if( cfl EQ NL_CLUSTERS OR cfl EQ NL_BOTH )
            {
                ul[r] = N_AllocReal1dArray( 0, SG );

                if( ul[r]EQ NULL )
                    NL_QUIT;

                ul[r][0] = t[ie];
                nl[r] = 0;
            }

            ib = ie;
        }
        else
        {
            sum = 0.0;
            avp = 0.0;
            ib = ie;

            for ( i = is; i <= ie; i++ )
            {
                sum += t[i];
                avn = sum / ((NL_REAL)i - (NL_REAL)is + 1.0);
                dmax = 0.0;

                for ( j = is; j <= i; j++ )
                {
                    d = fabs( t[i] - avn );

                    if( d GT dmax )
                        dmax = d;
                }

                if( dmax GT cdt )
                {
                    r++;

                    if( cfl EQ NL_PARAMETERS OR cfl EQ NL_BOTH )
                    {
                        sl[r] = avp;
                    }

                    if( cfl EQ NL_CLUSTERS OR cfl EQ NL_BOTH )
                    {
                        ul[r] = N_AllocReal1dArray( i - is, SG );

                        if( ul[r]EQ NULL )
                            NL_QUIT;

                        a = -1;

                        for ( j = is; j < i; j++ )
                            ul[r][++a] = t[j];
                        nl[r] = a;
                    }

                    is = i;
                    ib = i - 1;
                    break;
                }
                else if( dmax LE cdt AND i EQ ie )
                {
                    r++;

                    if( cfl EQ NL_PARAMETERS OR cfl EQ NL_BOTH )
                    {
                        sl[r] = avn;
                    }

                    if( cfl EQ NL_CLUSTERS OR cfl EQ NL_BOTH )
                    {
                        ul[r] = N_AllocReal1dArray( i - is, SG );

                        if( ul[r]EQ NULL )
                            NL_QUIT;

                        a = -1;

                        for ( j = is; j <= i; j++ )
                            ul[r][++a] = t[j];
                        nl[r] = a;
                    }

                    ib = ie;
                }
                else
                {
                    avp = avn;
                }
            }
        }
    }

    /* Get the output */

    if( cfl EQ NL_PARAMETERS OR cfl EQ NL_BOTH )
        *s = sl;

    if( cfl EQ NL_CLUSTERS OR cfl EQ NL_BOTH )
    {
        *u = ul;
        *ud = nl;
    }
    *l = r;

    /* Exit */

    EXIT:

    return (error);
} /* end N_FindClustersRealArray */

/*******************************************************************//**


   DESCRIPTION:

     This math routine sorts a real and integer array pair (a[i],b[i]) 
     using Shellsort. That is, the array a is sorted while the corres-
     ponding  elements from  b are swapped  whenever elements of a are 
     swapped. An  illustrative  example is to  sort a real array while
     keeping  track of the  corresponding  indexes. A  typical calling 
     example is:

       NL_REAL   *a;
       NL_INDEX  *b;
       NL_INDEX  n;
       ...
       (get arrays a and b);
       ...
       N_SortRealIndexArrays(a,b,n);


   ACCESS:
   
     a , in/out ,  Real array to be sorted
     b , in/out ,  Index array to be associated with a
     n , input  ,  Highest index in a and b


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SortRealIndexArrays( NL_REAL *a, NL_INDEX *b, NL_INDEX n )
{

    NL_INDEX i, j, k, d;

    NL_REAL c;

    k = n + 1;

    while( k GT 1 )
    {
        if( k GE 5 )
            k = (5 * k - 1) / 11;
        else
            k = 1;

        for ( i = n - k; i >= 0; i-- )
        {
            c = a[i];
            d = b[i];

            for ( j = i + k; j <= n && c > a[j]; j += k )
            {
                a[j - k] = a[j];
                b[j - k] = b[j];
            }
            a[j - k] = c;
            b[j - k] = d;
        }
    }
} /* end N_SortRealIndexArrays */

/*******************************************************************//**


   DESCRIPTION:

     This  math routine  sorts a  real array  pair  (a[i],b[i])  using 
     Shellsort. That is, the array a is sorted while the corresponding
     elements from  b are swapped whenever elements of  a are swapped.
     An illustrative  example is to  sort distances  of points  from a 
     curve  while  keeping track  of the  corresponding  parameters. A 
     typical calling example is:

       NL_REAL   *a, *b;
       NL_INDEX  n;
       ...
       (get arrays a and b);
       ...
       N_SortRealRealArrays(a,b,n);


   ACCESS:
   
     a , in/out ,  Real array to be sorted
     b , in/out ,  Real array to be associated with a
     n , input  ,  Highest index in a and b


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SortRealRealArrays( NL_REAL *a, NL_REAL *b, NL_INDEX n )
{

    NL_INDEX i, j, k;

    NL_REAL c, d;

    k = n + 1;

    while( k GT 1 )
    {
        if( k GE 5 )
            k = (5 * k - 1) / 11;
        else
            k = 1;

        for ( i = n - k; i >= 0; i-- )
        {
            c = a[i];
            d = b[i];

            for ( j = i + k; j <= n && c > a[j]; j += k )
            {
                a[j - k] = a[j];
                b[j - k] = b[j];
            }
            a[j - k] = c;
            b[j - k] = d;
        }
    }
} /* end N_SortRealRealArrays */

/**********************************************************************/
/* M_TridiagonalSystem                                                */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     Solve the tridiagonal system of n equations for P0 to Pn-1 given 
     a[], b[], c[] and Q0 to Qn-1. The system of equations are:

     b0  c0   0  ...                        P0     Q0
     a1  b1  c1   0  ...                    P1     Q1
      0  a2  b2  c2   0  ...                P2     Q2
          ....                                  =

               ...  0  an-2  bn-2  cn-2     Pn-2   Qn-2
                       ...   an-1  cn-1     Pn-1   Qn-1

     
     NL_REAL     a[], b[], c[];
     NL_POINT  Q[];
     NL_INDEX     n;
       ...
     set a[i], b[i] and c[i] for i = 0 to n-1
     get Q[i] for i=0 to n-1
     allocate memory for P[]
       ...
     status = M_TridiagonalSystem (a, b, c, Q, n, P)

     IT IS ASSUMED THAT MEMORY TO STORE THE P ARRAY IS 
     ALLOCATED IN THE CALLING  ROUTINE. 

   ACCESS:
   
     a, b, c , input  ,  NL_REAL arrays
     Q         input     NL_POINT array
     n         input     number of equations
     P         output    NL_POINT array, the solution of the system


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG M_TridiagonalSystem( NL_REAL *a, NL_REAL *b, NL_REAL *c, NL_POINT *Q, NL_INDEX n, NL_POINT *P )
{

    /* Based on Numerical Recipes in C */

    NL_FLAG error = NL_NO;
    NL_INDEX j;
    NL_REAL bet, *gam, OneOverBet;

    NL_STACKS SL;
    N_InitNurbs( &SL );

    gam = N_AllocReal1dArray( n, &SL );

    if( gam == NULL )
        NL_QUIT;

    bet = b[0];

    if( bet == 0.0 )
        NL_QUIT;
    OneOverBet = 1.0 / bet;

    N_ScalePt( 1.0 / bet, Q[0], &P[0] );

    /* Decomposition and forward substitution */
    for ( j = 1; j < n; j++ )
    {
        gam[j] = c[j - 1] * OneOverBet;
        bet = b[j] - a[j] * gam[j];

        if( bet == 0.0 )
            NL_QUIT;
        OneOverBet = 1.0 / bet;
        N_Combine2Pts( OneOverBet, Q[j], -a[j] * OneOverBet, Q[j - 1], &P[j - 1] );
    }

    /* Back substitution */
    for ( j = (n - 2); j > 1; j-- )
    {
        N_Combine2Pts( 1.0, P[j], -gam[j + 1], P[j + 1], &P[j] );
    }

    /* Exit */
    EXIT:
    return error;
}

/**********************************************************************/
/* M_CorneredTridiagonalSystem                                        */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     Solve the cornered tridiagonal system of n equations for P0 to Pn-1 
     given a[], b[], c[], alpha, beta and Q0 to Qn-1. The system of 
     equations are:

     b0  c0   0  ...         ... 0 beta     P0     Q0
     a1  b1  c1   0  ...             0      P1     Q1
      0  a2  b2  c2   0  ...                P2     Q2
          ....                                  =

               ...  0  an-2  bn-2  cn-2     Pn-2   Qn-2
     alpha 0 ...       ...   an-1  cn-1     Pn-1   Qn-1

     
       NL_REAL     a[], b[], c[], alpha, beta
       NL_POINT  Q[];
       NL_INDEX     n;
       ...
       set alpha, beta, a[i], b[i] and c[i] for i = 0 to n-1
       get Q[i] for i=0 to n-1
       allocate memory for P[]
       ...
       status = M_CorneredTridiagonalSystem (a, b, c, alpha, beta, Q, n, P)

     IT IS ASSUMED THAT MEMORY TO STORE THE P ARRAY IS 
     ALLOCATED IN THE CALLING  ROUTINE. 

   ACCESS:
   
     a, b, c ,      input  ,  NL_REAL arrays
     alpha, beta    input     NL_REAL
     Q              input     NL_POINT array
     n              input     number of equations
     P              output    NL_POINT array, the solution of the system


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG M_CorneredTridiagonalSystem( NL_REAL *a, NL_REAL *b, NL_REAL *c, NL_REAL alpha, NL_REAL beta, NL_POINT *Q, NL_INDEX n, NL_POINT *P )
{

    /* Based on Numerical Recipes in C */

    NL_FLAG error = NL_NO;
    int i;
    NL_REAL gamma, *bb, BetaOverGamma;
    NL_POINT *R, *S, F;

    NL_STACKS SL;
    N_InitNurbs( &SL );

    if( n <= 2 )
        NL_QUIT;

    bb = N_AllocReal1dArray( n, &SL );

    if( bb == NULL )
        NL_QUIT;

    R = N_AllocPt1dArray( n, &SL );

    if( R == NULL )
        NL_QUIT;

    S = N_AllocPt1dArray( n, &SL );

    if( S == NULL )
        NL_QUIT;

    gamma = -b[0];

    if( gamma == 0.0 )
        NL_QUIT;
    BetaOverGamma = beta / gamma;

    /* set up the diagonal of the modified tridiagonal system */
    bb[0] = b[0] - gamma;
    bb[n - 1] = b[n - 1] - alpha * BetaOverGamma;

    for ( i = 1; i < (n - 1); i++ )
        bb[i] = b[i];

    error = M_TridiagonalSystem( a, bb, c, Q, n, P );

    if( error )
        NL_QUIT;

    R[0].x = R[0].y = R[0].z = gamma;
    R[n - 1].x = R[n - 1].y = R[n - 1].z = alpha;

    for ( i = 1; i < (n - 1); i++ )
        R[i].x = R[i].y = R[i].z = 0.0;

    error = M_TridiagonalSystem( a, bb, c, R, n, S );

    if( error )
        NL_QUIT;

    F.x = (P[0].x + BetaOverGamma * P[n - 1].x) / (1.0 + S[0].x + BetaOverGamma * S[n - 1].x);
    F.y = (P[0].y + BetaOverGamma * P[n - 1].y) / (1.0 + S[0].y + BetaOverGamma * S[n - 1].y);
    F.z = (P[0].z + BetaOverGamma * P[n - 1].z) / (1.0 + S[0].z + BetaOverGamma * S[n - 1].z);

    for ( i = 0; i < n; i++ )
    {
        P[i].x -= F.x * S[i].x;
        P[i].y -= F.y * S[i].y;
        P[i].z -= F.z * S[i].z;
    }

    /* Exit */
    EXIT:
    return error;
}
