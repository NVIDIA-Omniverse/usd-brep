// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* NL_Geom.c : NLib Geometry Function Definitions                            */
/*****************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>


/* file local declarations */

NL_REAL ST_havers( NL_REAL );
NL_REAL ST_mmxaps( NL_REAL );

/* Thread Local Storage for ST_mmxaps */
NL_REAL *ST_mmxaps_xp;
NL_REAL *ST_mmxaps_yp;
NL_INDEX ST_mmxaps_gn;

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the dot product of two given
     vectors. A typical calling example is:

       NL_VECTOR  a, b;
       NL_REAL    dot;
       ...
       N_VectorDot(a,b,&dot);


   ACCESS:
   
     a,b  , input  ,  Given vectors
     dot  , output ,  Dot product



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorDot( NL_VECTOR a, NL_VECTOR b, NL_REAL *dot )
{
    *dot = a.x * b.x + a.y * b.y + a.z * b.z;
} /* end N_VectorDot */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the dot product of two given
     vectors. Both vectors are passed in by reference. A typical
     calling example is:

       NL_VECTOR  a, b;
       NL_REAL    dot;
       ...
       N_VectorDot(&a,&b,&dot);


   ACCESS:
   
     a,b  , input  ,  Given vectors
     dot  , output ,  Dot product



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorDotRef( NL_VECTOR *a, NL_VECTOR *b, NL_REAL *dot )
{
    *dot = a->x * b->x + a->y * b->y + a->z * b->z;
} /* end N_VectorDotRef */


/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the cross product of two given
     vectors. A typical calling example is:

       NL_VECTOR  a, b, c;
       ...
       N_VectorCross(a,b,&c);


   ACCESS:
   
     a,b  , input  ,  Given vectors
     c    , output ,  Cross product



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorCross( NL_VECTOR a, NL_VECTOR b, NL_VECTOR *c )
{
    c->x = a.y * b.z - a.z * b.y;
    c->y = a.z * b.x - a.x * b.z;
    c->z = a.x * b.y - a.y * b.x;
} /* end N_VectorCross */


/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the cross product of two given
     vectors.  Both are passed in  by reference. A typical calling 
     example is:

       NL_VECTOR  a, b, c;
       ...
       N_VectorCross(&a,&b,&c);


   ACCESS:
   
     a,b  , input  ,  Given vectors
     c    , output ,  Cross product



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorCrossRef( NL_VECTOR *a, NL_VECTOR *b, NL_VECTOR *c )
{
    c->x = a->y * b->z - a->z * b->y;
    c->y = a->z * b->x - a->x * b->z;
    c->z = a->x * b->y - a->y * b->x;
} /* end N_VectorCrossRef */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine scales a given vector by a given factor, 
     i.e. it computes b = alpha*a. A typical calling example is:

       NL_VECTOR  a, b;
       NL_REAL    alpha;
       ...
       (get a and alpha);
       ...
       N_VectorScale(a,alpha,&b);


   ACCESS:
   
     a     , input  ,  Given vector
     alpha , input  ,  Scaling factor
     b     , output ,  Scaled vactor



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorScale( NL_VECTOR a, NL_REAL alpha, NL_VECTOR *b )
{
    b->x = alpha * a.x;
    b->y = alpha * a.y;
    b->z = alpha * a.z;
} /* end N_VectorScale */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine reverses the direction of a given vector. A
     typical calling example is:

       NL_VECTOR  a, b;
       ...
       (get a);
       ...
       N_VectorReverse(a,&b);


   ACCESS:
   
     a  , input  ,  Given vector
     b  , output ,  Reverse of a


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorReverse( NL_VECTOR a, NL_VECTOR *b )
{
    b->x = -a.x;
    b->y = -a.y;
    b->z = -a.z;
} /* end N_VectorReverse */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine reverses the  direction of a given vector in
     place, ie the original components are destroyed. A typical calling 
     example is:

       NL_VECTOR  a;
       ...
       (get a);
       ...
       N_VectorReverseInPlace(&a);


   ACCESS:
   
     a  , in/out ,  Given vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorReverseInPlace( NL_VECTOR *a )
{
    a->x = -a->x;
    a->y = -a->y;
    a->z = -a->z;
} /* end N_VectorReverseInPlace */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine reverses the direction of a given vector wrt
     another vector. That is, given vectors a and b. If the dot product
     of a  and b is  negative,  the  direction  of b is  reversed. This 
     version reverses b in place. A typical calling example:

       NL_VECTOR  a, b;
       ...
       (get a and b);
       ...
       N_VectorReverseVectorInPlace(a,&b);


   ACCESS:
   
     a  , input  ,  Given vector
     b  , input  ,  Vector to be reversed


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorReverseVectorInPlace( NL_VECTOR a, NL_VECTOR *b )
{
    NL_REAL dot;

    N_VectorDot( a, *b, &dot );

    if( dot LT 0.0 )
        N_VectorReverseInPlace( b );
} /* end N_VectorReverseVectorInPlace */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine reverses the direction of a given vector wrt
     another vector. That is, given vectors a and b. If the dot product
     of a and b is  negative, the direction of b is reversed. A typical 
     calling example is:

       NL_VECTOR  a, b, c;
       ...
       (get a and b);
       ...
       N_VectorReverseVector(a,b,&c);


   ACCESS:
   
     a  , input  ,  Given vector
     b  , input  ,  Vector to be reversed
     c  , output ,  Reversed b 


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorReverseVector( NL_VECTOR a, NL_VECTOR b, NL_VECTOR *c )
{
    NL_REAL dot;

    N_VectorDot( a, b, &dot );

    if( dot LT 0.0 )
        N_VectorReverse( b, c );
    else
        N_VectorCopy( b, c );
} /* end N_VectorReverseVector */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine scales a given vector by a given factor, 
     i.e.  it  computes  b = alpha*a. The  vector  is  passed in by
     reference. A typical calling example is:

       NL_VECTOR  a, b;
       NL_REAL    alpha;
       ...
       (get a and alpha);
       ...
       N_VectorScaleRef(&a,alpha,&b);


   ACCESS:
   
     a     , input  ,  Given vector
     alpha , input  ,  Scaling factor
     b     , output ,  Scaled vactor



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorScaleRef( NL_VECTOR *a, NL_REAL alpha, NL_VECTOR *b )
{
    b->x = alpha * a->x;
    b->y = alpha * a->y;
    b->z = alpha * a->z;
} /* end N_VectorScaleRef */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the sum of two given vectors,
     i.e. it computes c = a+b. A typical calling example is:

       NL_VECTOR  a, b, c;
       ...
       (get a and b);
       ...
       N_VectorSum(a,b,&c);


   ACCESS:
   
     a,b  , input  ,  Given vectors
     c    , output ,  Sum of a and b



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorSum( NL_VECTOR a, NL_VECTOR b, NL_VECTOR *c )
{
    c->x = a.x + b.x;
    c->y = a.y + b.y;
    c->z = a.z + b.z;
} /* end N_VectorSum */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry routine computes the combination of two given 
     vectors, i.e. it computes c = alf*a+bet*b. A typical calling 
     example is:

       NL_VECTOR  a, b, c;
       NL_REAL    alf, bet;
       ...
       (get a, b, alf and bet);
       ...
       N_VectorCombine(alf,a,bet,b,&c);


   ACCESS:
   
     alf,bet , input  ,  Combination parameters
     a,b     , input  ,  Given vectors
     c       , output ,  Combination of a and b



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorCombine( NL_REAL alf, NL_VECTOR a, NL_REAL bet, NL_VECTOR b, NL_VECTOR *c )
{
    c->x = alf * a.x + bet * b.x;
    c->y = alf * a.y + bet * b.y;
    c->z = alf * a.z + bet * b.z;
} /* end N_VectorCombine */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry  routine  computes  the  difference of  two given 
     vectors, i.e. it computes c = a-b. A typical calling example is:

       NL_VECTOR  a, b, c;
       ...
       (get a and b);
       ...
       N_VectorDiff(a,b,&c);


   ACCESS:
   
     a,b  , input  ,  Given vectors
     c    , output ,  Difference of a and b



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorDiff
 ( NL_VECTOR a,    /* a of c = a - b         */ 
   NL_VECTOR b,    /* b of c = a - b         */ 
   NL_VECTOR *c )  /* c = vector from b to a */ 
{
    c->x = a.x - b.x;
    c->y = a.y - b.y;
    c->z = a.z - b.z;
} /* end N_VectorDiff */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine  computes the difference vector between two 
     control points in Euclidean space, i.e. it first maps the control
     points  to Euclidean  space, and computes  the difference between 
     the Euclidean points. A typical calling example is:

       NL_CPOINT  Pw, Qw;
       NL_VECTOR  V;
       ...
       (get Pw and Qw);
       ...
       N_VectorDiffCPts(Pw,Qw,&V);


   ACCESS:
   
     Pw  , input  ,  First control point
     Qw  , input  ,  Second control point
     V   , output ,  Difference vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorDiffCPts( NL_CPOINT Pw, NL_CPOINT Qw, NL_VECTOR *V )
{
    NL_POINT P, Q;

    /* Map to Euclidean space */

    N_CPtToPtEuclid( Pw, &P );
    N_CPtToPtEuclid( Qw, &Q );

    /* Compute difference vector */

    N_VectorDiff( P, Q, V );
} /* end N_VectorDiffCPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes a  direction vector given  the end
     points, i.e. it computes d = Pe-Ps. A typical calling example is:
 
       NL_POINT   Ps, Pe;
       NL_VECTOR  d;
       ...
       (get Ps and Pe);
       ...
       N_VectorDir(Ps,Pe,&d);


   ACCESS:
   
     Ps,Pe , input  ,  Start and end points
     d     , output ,  Direction vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorDir
 (NL_POINT Ps,    /* in : Ps  of Vec = Pe - Ps */
  NL_POINT Pe,    /* in : Pe  of Vec = Pe - Ps */
  NL_VECTOR *d )  /* out: Vec of Vec = Pe - Ps */
{
    d->x = Pe.x - Ps.x;
    d->y = Pe.y - Ps.y;
    d->z = Pe.z - Ps.z;
} /* end N_VectorDir */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine creates a NL_VECTOR data type from <x,y,z>
     data. A typical calling example is:

       NL_REAL    x, y, z;
       NL_VECTOR  a;
       ...
       (get x, y and z);
       ...
       N_VectorCreate(x,y,z,&a);


   ACCESS:
   
     x,y,z , input  ,  Coordinates of vector
     a     , output ,  NL_VECTOR entity


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorCreate( NL_REAL x, NL_REAL y, NL_REAL z, NL_VECTOR *a )
{
    a->x = x;
    a->y = y;
    a->z = z;
} /* end N_VectorCreate */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes a copy of a given vector, i.e. it  
     computes b = a. A typical calling example is:

       NL_VECTOR  a, b;
       ...
       (get a);
       ...
       N_VectorCopy(a,&b);


   ACCESS:
   
     a , input  ,  Given vector
     b , output ,  Copy vector



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorCopy( NL_VECTOR a, NL_VECTOR *b )
{
    b->x = a.x;
    b->y = a.y;
    b->z = a.z;
} /* end N_VectorCopy */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the mixed product of three given
     vectors, i.e. it  computes  abc = (a x b)c. A  typical  calling 
     example is:

       NL_VECTOR  a, b, c;
       NL_REAL    mix;
       ...
       (get a, b and c);
       ...
       N_VectorMixMultiply(a,b,c,&mix);


   ACCESS:
   
     a,b,c  , input  ,  Given vectors
     mix    , output ,  Mixed product



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorMixMultiply( NL_VECTOR a, NL_VECTOR b, NL_VECTOR c, NL_REAL *mix )
{
    NL_VECTOR d;

    N_VectorCross( a, b, &d );
    N_VectorDot( d, c, mix );
} /* end N_VectorMixMultiply */


/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the magnitude of a given vector.
     A typical calling example is:

       NL_VECTOR  a;
       NL_REAL    mag;
       ...
       (get a);
       ...
       N_VectorMagnitude(a,&mag);


   ACCESS:
   
     a   , input  ,  Given vector
     mag , output ,  Vector magnitude



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorMagnitude( NL_VECTOR a, NL_REAL *mag )
{
    NL_REAL d;

    N_VectorDot( a, a, &d );
    *mag = sqrt( d );
} /* end N_VectorMagnitude */


/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the magnitude of a given vector
     passed in by reference. A typical calling example is:

       NL_VECTOR  a;
       NL_REAL    mag;
       ...
       (get a);
       ...
       N_VectorMagnitudeRef(&a,&mag);


   ACCESS:
   
     a   , input  ,  Given vector
     mag , output ,  Vector magnitude



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorMagnitudeRef( NL_VECTOR *a, NL_REAL *mag )
{
    NL_REAL d;

    /* let d = a*a */
    N_VectorDotRef( a, a, &d );
    *mag = sqrt( d );
} /* end N_VectorMagnitudeRef */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine normalizes a given vector. A typical calling
     example is:

       NL_VECTOR  a, b;
       NL_REAL    mag;
       ...
       (get a);
       ...
       N_VectorNormalize(a,&b,&mag);


   ACCESS:
   
     a   , input  ,  Given vector
     b   , output ,  Normalized vector
     mag , output ,  Magnitude of given vector 


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VectorNormalize
 (NL_VECTOR  a,    /* in : Given vector */
  NL_VECTOR *b,    /* out: Normalized vector */
  NL_REAL   *mag ) /* out: Magnitude of given vector */
{
    NL_PRIVATE NL_STRING rname = _T("N_VectorNormalize");

    NL_REAL d, invd;

    N_VectorMagnitude( a, &d );

    if( N_FloatOpIsBad( 1.0, d, NL_DIVISION ) )
    {
        b->x = b->y = b->z = 0.0;
        *mag = 0.0;
        N_ErrSet( NL_GEO_ERR, rname );

        return (1);
    }

    invd = 1.0 / d;
    N_VectorScale( a, invd, b );
    *mag = d;

    return (0);
} /* end N_VectorNormalize */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine normalizes a given vector in place, i.e. it
     is passed in by reference. A typical calling example is:

       NL_VECTOR  a;
       ...
       N_VectorNormalizeRef(&a);


   ACCESS:
   
     a   , in/out ,  Given vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VectorNormalizeRef( NL_VECTOR *a )
{
    NL_PRIVATE NL_STRING rname = _T("N_VectorNormalizeRef");

    NL_REAL d, invd;

    /* let d = sqrt(vector_dot_product(a,a)) */
    N_VectorMagnitudeRef( a, &d );

    if( N_FloatOpIsBad( 1.0, d, NL_DIVISION ) )
    {
        N_ErrSet( NL_GEO_ERR, rname );
        return (1);
    }

    invd = 1.0 / d;

    /* scale a vector by letting a = invd * a   */
    N_VectorScaleRef( a, invd, a );

    return (0);
} /* end N_VectorNormalizeRef */


/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes a vector that is perpendicular 
     to a given  vector in  2-D. The  perpendicular is computed by
     rotating the input vector counterclockwise. A typical calling
     example is:

       NL_VECTOR  a, b;
       ...
       (get a);
       ...
       N_VectorPerpendicular(a,&b);


   ACCESS:
   
     a , input  ,  Given vector
     b , output ,  Perpendicular



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorPerpendicular
  ( NL_VECTOR a,    /* in : 2d vector (z == 0.0)              */ 
    NL_VECTOR *b )  /* out: CCW 90 degree rotation (z == 0.0) */ 
{
    b->x = -a.y;
    b->y = a.x;
    b->z = 0.0;
} /* end N_VectorPerpendicular */

/*******************************************************************//**


   DESCRIPTION:

     Given a line by its start point P and its direction vector V.
     This geometry routine computes a point along <P,V> at a given
     parameter,  i.e. it  computes  Q=P+alpha*V. A typical calling 
     example is:

       NL_POINT   P, Q;
       NL_REAL    alpha;
       NL_VECTOR  V;
       ...
       (get P, V and alpha);
       ...
       N_VectorPtAlongVector(P,alpha,V,&Q);


   ACCESS:
   
     P     , input  ,  Start point
     alpha , input  ,  Parameter
     V     , input  ,  Direction vector
     Q     , output ,  Point on the line



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VectorPtAlongVector
 ( NL_POINT   P,      /* in : P     of  Q = P + alpha * V */ 
   NL_REAL    alpha,  /* in : alpha of  Q = P + alpha * V */ 
   NL_VECTOR  V,      /* in : V     of  Q = P + alpha * V */ 
   NL_POINT * Q )     /* out: Q     of  Q = P + alpha * V */ 
{
    Q->x = P.x + alpha * V.x;
    Q->y = P.y + alpha * V.y;
    Q->z = P.z + alpha * V.z;
} /* end N_VectorPtAlongVector */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine  checks if two  vectors point to the same 
     direction, i.e. if their dot product is non-negative. A typical 
     calling example is:

       NL_VECTOR  a, b;
       ...
       (get a and b);
       ...
       if( N_VectorsAreParallel(a,b) )  --> a and b point to the same direction;


   ACCESS:
   
     a,b , input ,  Given vectors


   RETURN CODES:

     NL_TRUE : a and b point to the same direction
     NL_FALSE: a and b point to opposite directions

   ***********************************************************************/

NL_BOOLEAN N_VectorsAreParallel( NL_VECTOR a, NL_VECTOR b )
{
    NL_REAL dot;

    /* Check direction */

    N_VectorDot( a, b, &dot );

    if( dot GE 0.0 )
        return NL_TRUE;
    else
        return NL_FALSE;
} /* end N_VectorsAreParallel */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the cosine of the angle between two
     vectors. A typical calling example is:

       NL_VECTOR  a, b;
       NL_REAL    cos;
       ...
       N_VectorsCosAngle(a,b,&cos);


   ACCESS:
   
     a,b  , input  ,  Given vectors
     cos  , output ,  Cosine of the angle between the vectors



   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VectorsCosAngle( NL_VECTOR a, NL_VECTOR b, NL_REAL *cos )
{
    NL_PRIVATE NL_STRING rname = _T("N_VectorsCosAngle");

    NL_REAL dot, ma, mb, den;

    dot = a.x * b.x + a.y * b.y + a.z * b.z;
    ma = a.x * a.x + a.y * a.y + a.z * a.z;
    mb = b.x * b.x + b.y * b.y + b.z * b.z;

    den = sqrt( ma * mb );

    if( N_FloatOpIsBad( dot, den, NL_DIVISION ) )
    {
        N_ErrSet( NL_GEO_ERR, rname );
        return (1);
    }

    *cos = dot / den;

    if( *cos GT 1.0 )
        *cos = 1.0;

    if( *cos LT - 1.0 )
        *cos = -1.0;

    return (0);
} /* end N_VectorsCosAngle */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This geometry routine computes the angle, IN DEGREES, between two 
     vectors. A typical calling example is:
 
       NL_VECTOR  a, b;
       NL_REAL    alf;
       ...
       N_VectorsAngle(a,b,&alf);
 
 
   ACCESS:
   
     a,b  , input  ,  Given vectors
     alf  , output ,  Angle between the vectors in degrees,
                      From 0 to + 180 degrees. 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_VectorsAngle( NL_VECTOR a, NL_VECTOR b, NL_REAL *alf )
{
    NL_PRIVATE NL_STRING rname = _T("N_VectorsAngle");

    NL_REAL dot, ma, mb, arg, rad;

    N_VectorDot( a, b, &dot );
    N_VectorMagnitude( a, &ma );
    N_VectorMagnitude( b, &mb );

    if( N_FloatOpIsBad( dot, ma *mb, NL_DIVISION ) )
    {
        N_ErrSet( NL_GEO_ERR, rname );
        return (1);
    }

    arg = dot / (ma * mb);

    if( arg GT 1.0 )
        arg = 1.0;

    if( arg LT - 1.0 )
        arg = -1.0;

    rad = acos( arg );
    *alf = 180.0 *(rad / NL_PI);

    return (0);
} /* end N_VectorsAngle */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This  geometry routine  computes the  directed  angle, IN  DEGREES, 
     between a reference vector and an arbitrary vector in 2-D. That is, 
     the angle ranges from 0 to 360  degrees. A typical calling example:
 
       NL_VECTOR  R, V;
       NL_REAL    alf;
       ...
       N_VectorDirectedAngle(R,V,&alf);
 
 
   ACCESS:
   
     R   , input  ,  Reference vector
     V   , input  ,  Arbitrary vector
     alf , output ,  Angle between R and V
 
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_VectorDirectedAngle( NL_VECTOR R, NL_VECTOR V, NL_REAL *alf )
{
    NL_PRIVATE NL_STRING rname = _T("N_VectorDirectedAngle");

    NL_REAL dot, mr, mv, arg, rad, ang;

    NL_VECTOR N;

    N_VectorDot( R, V, &dot );
    N_VectorMagnitude( R, &mr );
    N_VectorMagnitude( V, &mv );

    if( N_FloatOpIsBad( dot, mr *mv, NL_DIVISION ) )
    {
        N_ErrSet( NL_GEO_ERR, rname );
        return (1);
    }

    arg = dot / (mr * mv);

    if( arg GT 1.0 )
        arg = 1.0;

    if( arg LT - 1.0 )
        arg = -1.0;

    rad = acos( arg );
    ang = 180.0 *(rad / NL_PI);

    N_VectorPerpendicular( R, &N );
    N_VectorDot( V, N, &dot );

    if( dot LT 0.0 )
        ang = 360.0 - ang;

    *alf = ang;

    return (0);
} /* end N_VectorDirectedAngle */

/*******************************************************************//**


   DESCRIPTION:

     This geometry  routine  intersects  two lines. The  least-squares  
     intersection  is computed  which is  the true intersection if the 
     lines happen to lie on the  same plane. A typical calling example
     is:

       NL_LINESEG  ls1, ls2;
       NL_POINT    P;
       NL_REAL     t1, t2;
       NL_FLAG     flg;
       ...
       (get lines ls1 and ls2);
       ...
       N_IsectLineLine(ls1,ls2,&P,&t1,&t2,&flg);


   ACCESS:
   
     ls1,ls2 , input  ,  Given lines
     P       , output ,  Intersection point
     t1,t2   , output ,  Parameters corresponding to the intersection, 
     flg     , output ,  Flag = 
                           NL_TRUE : Lines intersect
                           NL_FALSE: Lines  do  not intersect (parallel or
                                  bounded)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsectLineLine( NL_LINESEG ls1, NL_LINESEG ls2, NL_POINT *P, NL_REAL *t1, NL_REAL *t2, NL_FLAG *flg )
{
    NL_PRIVATE NL_STRING rname = _T("N_IsectLineLine");

    NL_FLAG error, bd1, bd2;

    NL_VECTOR P21, V1, V2;

    NL_POINT P1, P2;

    NL_REAL a[2][2], b[2], alpha, beta, numa, numb, den, m1, m2, u1, u2;

    /* Normalize input vectors */

    N_LineGetData( &ls1, &P1, &V1, &bd1 );
    N_LineGetData( &ls2, &P2, &V2, &bd2 );

    error = N_VectorNormalize( V1, &V1, &m1 );

    if( error EQ NL_YES )
        return (1);

    error = N_VectorNormalize( V2, &V2, &m2 );

    if( error EQ NL_YES )
        return (1);

    /* Compute intersection */

    N_VectorDir( P1, P2, &P21 );

    N_VectorDot( V1, V1, &a[0][0] );
    N_VectorDot( V1, V2, &a[0][1] );
    N_VectorDot( V2, V2, &a[1][1] );

    N_VectorDot( V1, P21, &b[0] );
    N_VectorDot( V2, P21, &b[1] );

    den = a[0][0] * a[1][1] - a[0][1] * a[0][1];

    if( fabs( den )LT NL_ZCTL ) /* Check parallelism */  /* gwc: was NL_LTOL - but this is not an aangle - maybe NL_ZCTL is not right */
    {
        *flg = NL_FALSE;
        return (0);
    }

    numa = b[0] * a[1][1] - b[1] * a[0][1];
    numb = b[0] * a[0][1] - b[1] * a[0][0];

    if( N_FloatOpIsBad( numa, den, NL_DIVISION )OR N_FloatOpIsBad( numb, den, NL_DIVISION ) )
    {
        N_ErrSet( NL_NUM_ERR, rname );
        return (1);
    }
    alpha = numa / den;
    beta = numb / den;

    if( N_FloatOpIsBad( alpha, m1, NL_DIVISION )OR N_FloatOpIsBad( beta, m2, NL_DIVISION ) )
    {
        N_ErrSet( NL_NUM_ERR, rname );
        return (1);
    }
    u1 = alpha / m1;
    u2 = beta / m2;

    /* Check if lines are bounded */

    if( bd1 EQ NL_TRUE )
    {
        if( u1 LT 0.0 OR u1 GT 1.0 )
        {
            *flg = NL_FALSE;
            return (0);
        }
    }

    if( bd2 EQ NL_TRUE )
    {
        if( u2 LT 0.0 OR u2 GT 1.0 )
        {
            *flg = NL_FALSE;
            return (0);
        }
    }

    /* Compute the intersection point */

    N_VectorPtAlongVector( P1, alpha, V1, &P1 );
    N_VectorPtAlongVector( P2, beta, V2, &P2 );
    N_Combine2Pts( 0.5, P1, 0.5, P2, P );

    /* Exit */

    *t1 = u1;
    *t2 = u2;
    *flg = NL_TRUE;

    return (0);
} /* end N_IsectLineLine */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine intersects two line segments. A typical 
     calling example is:

       NL_POINT  P1, P2, Q1, Q2, R;
       NL_FLAG   its;
       ...
       (get P1, P2, Q1 and Q2);
       ...
       N_IsectLineSegs(P1,P2,Q1,Q2,&R,&its);


   ACCESS:
   
     P1,P2 , input  ,  Line segment <P1,P2>
     Q1,Q2 , input  ,  Line segment <Q1,Q2>
     R     , output ,  Intersection point
     its   , output ,  Flag = 
                        NL_TRUE : Line segments intersect
                        NL_FALSE: Line segments do not intersect


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsectLineSegs( NL_POINT P1, NL_POINT P2, NL_POINT Q1, NL_POINT Q2, NL_POINT *R, NL_FLAG *its )
{
    NL_PRIVATE NL_STRING rname = _T("N_IsectLineSegs");

    NL_FLAG error = NL_NO;

    NL_POINT S, T;

    NL_REAL p1x, p1y, p2x, p2y, q1x, q1y, q2x, q2y, a, b, c, d, t, s;

    /* Get coordinates and initialize */

    *its = NL_TRUE;

    N_PtToXYZ( P1, &p1x, &p1y, &a );
    N_PtToXYZ( P2, &p2x, &p2y, &b );
    N_PtToXYZ( Q1, &q1x, &q1y, &c );
    N_PtToXYZ( Q2, &q2x, &q2y, &d );

    /* Check if intersection exists */

    a = (q1x - p1x) * (p2y - p1y) - (q1y - p1y) * (p2x - p1x);  
    b = (q2x - p1x) * (p2y - p1y) - (q2y - p1y) * (p2x - p1x);
    c = (p1x - q1x) * (q2y - q1y) - (p1y - q1y) * (q2x - q1x);
    d = (p2x - q1x) * (q2y - q1y) - (p2y - q1y) * (q2x - q1x);

    if( a *b GT 0.0 )
    {
        *its = NL_FALSE;
        NL_OUT;
    }

    if( c *d GT 0.0 )
    {
        *its = NL_FALSE;
        NL_OUT;
    }

    if( fabs( a - b )LT NL_ZCTL )  /* gwc: was NL_LTOL, but this is not an angle - switched to NL_ZCTL which may have to be reviewed */
    {
        *its = NL_FALSE;
        NL_OUT;
    }

    /* Compute intersection */

    if( N_FloatOpIsBad( -c, a - b, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    if( N_FloatOpIsBad( a, a - b, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    t = -c / (a - b);
    s = a / (a - b);

    N_Combine2Pts( 1.0 - t, P1, t, P2, &S );
    N_Combine2Pts( 1.0 - s, Q1, s, Q2, &T );
    N_Combine2Pts( 0.5, S, 0.5, T, R );

    /* Exit */

    EXIT:

    return (error);
} /* end N_IsectLineSegs */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine checks if two 2-D line segments 

           <P1(x1,y1),P2(x2,y2)>  and  <P3(x3,y3),P4(x4,y4)>

     intersect or not. A typical calling example is:

       NL_REAL  x1, y1, x2, y2, x3, y3, x4, y4, tol;
       ...
       (get x1,...,tol);
       ...
       if( N_LineSegs2dAreIntersecting(x1,y1,x2,y2,x3,y3,x4,y4,tol) ) --> intersection;

     THIS ROUTINE DOES NOT HANDLE SPECIAL CASES, SUCH AS COLLINEAR LINE
     SEGMENTS, FOR EFFICIENCY REASONS.
     

   ACCESS:
   
     x1,y1,x2,y2 , input ,  Coordinates of first line's end points
     x3,y3,x4,y4 , input ,  Coordinates of second line's end points
     tol         , input ,  Parameter tolerance


   RETURN CODES:

     NL_YES: Segments intersect
     NL_NO : Segments DO NOT intersect

   ***********************************************************************/

NL_BOOLEAN N_LineSegs2dAreIntersecting( NL_REAL x1, NL_REAL y1, NL_REAL x2, NL_REAL y2, NL_REAL x3, NL_REAL y3, NL_REAL x4, NL_REAL y4, NL_REAL tol )
{
    NL_REAL ax, ay, bx, by, cx, cy, den, numa, numb, denp, denm;

    /* Do bounding box check */

    if( NL_MAX( x3, x4 ) + tol LT NL_MIN( x1, x2 ) - tol )
        return NL_NO;

    if( NL_MAX( x1, x2 ) + tol LT NL_MIN( x3, x4 ) - tol )
        return NL_NO;

    if( NL_MAX( y3, y4 ) + tol LT NL_MIN( y1, y2 ) - tol )
        return NL_NO;

    if( NL_MAX( y1, y2 ) + tol LT NL_MIN( y3, y4 ) - tol )
        return NL_NO;

    /* Check intersection */

    ax = x2 - x1;
    ay = y2 - y1;
    bx = x3 - x4;
    by = y3 - y4;
    cx = x1 - x3;
    cy = y1 - y3;

    den = ay * bx - ax * by;

    if( fabs( den )LT NL_LTOL )
        return NL_NO;

    numa = by * cx - bx * cy;
    denp = den + tol;
    denm = den - tol;

    if( den GT 0.0 )
    {
        if( numa LT - tol OR numa GT denp )
            return NL_NO;
    }
    else
    {
        if( numa GT tol OR numa LT denm )
            return NL_NO;
    }

    numb = ax * cy - ay * cx;

    if( den GT 0.0 )
    {
        if( numb LT - tol OR numb GT denp )
            return NL_NO;
    }
    else
    {
        if( numb GT tol OR numb LT denm )
            return NL_NO;
    }

    return NL_YES;
} /* end N_LineSegs2dAreIntersecting */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry  routine  intersects  a  line  with  a  plane. The 
     intersection point is computed using vector operations. A typical
     calling example is:

       NL_LINESEG  lsg;
       NL_PLANE    pln;
       NL_POINT    P;
       NL_REAL     t;
       NL_FLAG     flg;
       ...
       (get plane and line);
       ...
       N_IsectLinePlane(lsg,pln,&P,&t,&flg);


   ACCESS:
   
     lsg , input  ,  Given line
     pln , input  ,  Given plane
     P   , output ,  Intersection point
     t   , output ,  Parameter  corresponding  to  the   intersection, 
     flg , output ,  Flag = 
                       NL_TRUE : Line and plane intersect
                       NL_FALSE: Line  and  plane do not  intersect (line
                               and  plane   are  parallel  or  line is 
                               bounded).


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsectLinePlane( NL_LINESEG lsg, NL_PLANE pln, NL_POINT *P, NL_REAL *t, NL_FLAG *flg )
{
    NL_PRIVATE NL_STRING rname = _T("N_IsectLinePlane");

    NL_FLAG error, bnd;

    NL_VECTOR PL, V, N;

    NL_POINT Q, R;

    NL_REAL alpha, num, den, mV, mN;

    /* Normalize input vectors */

    N_LineGetData( &lsg, &Q, &V, &bnd );
    N_PlaneGetData( &pln, &R, &N );

    error = N_VectorNormalize( V, &V, &mV );

    if( error EQ NL_YES )
        return (1);

    error = N_VectorNormalize( N, &N, &mN );

    if( error EQ NL_YES )
        return (1);

    /* Compute intersection */

    N_VectorDir( Q, R, &PL );

    N_VectorDot( V, N, &den );

    if( fabs( den )LT NL_LTOL ) /* Check for parallelism */
    {
        *flg = NL_FALSE;
        return (0);
    }
    N_VectorDot( PL, N, &num );

    if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
    {
        N_ErrSet( NL_NUM_ERR, rname );
        return (1);
    }
    alpha = num / den;

    if( N_FloatOpIsBad( alpha, mV, NL_DIVISION ) )
    {
        N_ErrSet( NL_NUM_ERR, rname );
        return (1);
    }
    *t = alpha / mV;

    /* Check if line is bounded */

    if( bnd EQ NL_TRUE )
    {
        if( *t LT 0.0 OR *t GT 1.0 )
        {
            *flg = NL_FALSE;
            return (0);
        }
    }

    /* Compute the intersection point and exit */

    N_VectorPtAlongVector( Q, alpha, V, P );

    *flg = NL_TRUE;

    return (0);
} /* end N_IsectLinePlane */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the intersection line of two planes. 
     A typical calling example is:

       NL_PLANE   pl1, pl2;
       NL_POINT   P;
       NL_VECTOR  V;
       NL_FLAG    flg;
       ...
       (get the planes);
       ...
       N_IsectPlanePlane(pl1,pl2,&P,&V,&flg);


   ACCESS:
   
     pl1 , input  ,  Plane 1 
     pl2 , input  ,  Plane 2
     P   , output ,  Point on line of intersection
     V   , output ,  Direction vector of line of intersection
     flg , output ,  Flag = 
                       NL_TRUE : Planes intersect
                       NL_FALSE: Planes do not intersect (parallel)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsectPlanePlane( NL_PLANE pl1, NL_PLANE pl2, NL_POINT *P, NL_VECTOR *V, NL_FLAG *flg )
{
    NL_FLAG error;

    NL_REAL x0, y0, z0, f, g, h, det, fac, ad, db, dc, a1, b1, c1, d1, a2, b2, c2, d2;

    NL_POINT P1, P2;

    NL_VECTOR N1, N2;

    /* Get planes' normalized implicit equations */

    N_PlaneGetData( &pl1, &P1, &N1 );
    N_PlaneGetData( &pl2, &P2, &N2 );

    error = N_PlanePtNormalToImplicit( P1, N1, &a1, &b1, &c1, &d1 );

    if( error EQ NL_YES )
        return (1);

    error = N_PlanePtNormalToImplicit( P2, N2, &a2, &b2, &c2, &d2 );

    if( error EQ NL_YES )
        return (1);

    /* Compute intersection line */

    f = b1 * c2 - b2 * c1;
    g = c1 * a2 - c2 * a1;
    h = a1 * b2 - a2 * b1;

    det = f * f + g * g + h * h;   /* angle between normals squared */

    if( fabs( det )LT NL_ZCTL )   /* gwc: was NL_LTOL but since this was an angle squred was changed to ZCTL when LTOL was made larger */
    {
        *flg = NL_FALSE;
        return (0);
    }
    fac = 1.0 / det;

    dc = d1 * c2 - c1 * d2;
    db = d1 * b2 - b1 * d2;
    ad = a1 * d2 - a2 * d1;

    x0 = fac * (g * dc - h * db);
    y0 = -fac * (f * dc + h * ad);
    z0 = fac * (f * db + g * ad);

    /* Define line and exit */

    N_PtFromXYZ( x0, y0, z0, P );
    N_PtFromXYZ( f, g, h, V );

    *flg = NL_TRUE;

    return (0);
} /* end N_IsectPlanePlane */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine projects a point onto a finite or infinite
     line. A typical calling example is:

       NL_LINESEG  lsg;
       NL_POINT    P, Q;
       NL_REAL     t;
       NL_FLAG     flg;
       ...
       (define line and get P);
       ...
       N_ProjectPtLine(lsg,P,&Q,&t,&flg);


   ACCESS:
   
     lsg , input  ,  Given line
     P   , input  ,  Given point
     Q   , output ,  Projection point
     t   , output ,  Parameter corresponding to the  projection
     flg , output ,  Flag = 
                       NL_TRUE : Point is projected
                       NL_FALSE: Projection is  out of the line segment



   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ProjectPtLine( NL_LINESEG lsg, NL_POINT P, NL_POINT *Q, NL_REAL *t, NL_FLAG *flg )
{
    NL_FLAG error;

    NL_PLANE nor;

    /* Get the plane normal to the line */

    N_CreatePlanePtNormal( &nor, P, lsg.V );

    /* Intersect plane with line */

    error = N_IsectLinePlane( lsg, nor, Q, t, flg );

    /* Exit */

    return (error);
} /* end N_ProjectPtLine */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This geometry  routine projects a  point onto a line segment and
     returns the projection as well as the corresponding parameter. A 
     typical calling example is:
 
       NL_POINT  P, A, B, Q;
       NL_REAL   t;
       ...
       (get P, A and B);
       ...
       N_ProjectPtLineParam(P,A,B,&Q,&t);
 
 
   ACCESS:
   
     P   , input  ,  Given point
     A,B , input  ,  End points of line segment
     Q   , output ,  Projection of P onto AB
     t   , output ,  Parameter corresponding to Q, i.e.  Q=A+t*(B-A).
                     t=0 corresponds to A and t=1 is assigned to B!
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_ProjectPtLineParam( NL_POINT P, NL_POINT A, NL_POINT B, NL_POINT *Q, NL_REAL *t )
{
    NL_PRIVATE NL_STRING rname = _T("N_ProjectPtLineParam");

    NL_VECTOR BA, PA, T;

    NL_REAL dot, len, dis;

    /* Check special cases */

    N_DistPtPt( A, B, &dis );

    if( dis LT NL_MTOL )
    {
        N_VectorSum( A, B, &T );
        N_VectorScale( T, 0.5, Q );

        *t = 0.0;

        return (0);
    }

    N_DistPtPt( P, A, &dis );

    if( dis LT NL_MTOL )
    {
        N_VectorCopy( A, Q );

        *t = 0.0;

        return (0);
    }

    N_DistPtPt( P, B, &dis );

    if( dis LT NL_MTOL )
    {
        N_VectorCopy( B, Q );

        *t = 1.0;

        return (0);
    }

    /* Get general case */

    N_VectorDiff( B, A, &BA );
    N_VectorDiff( P, A, &PA );
    N_VectorDot( BA, BA, &len );
    N_VectorDot( PA, BA, &dot );

    len = sqrt( len );

    if( N_FloatOpIsBad( dot, len, NL_DIVISION ) )
    {
        N_ErrSet( NL_NUM_ERR, rname );
        return (1);
    }

    dis = dot / len;

    if( fabs( dis )LT NL_MTOL )
    {
        N_VectorCopy( A, Q );

        *t = 0.0;
    }
    else if( fabs( dis - len )LT NL_MTOL )
    {
        N_VectorCopy( B, Q );

        *t = 1.0;
    }
    else
    {
        *t = dis / len;

        N_VectorPtAlongVector( A, *t, BA, Q );
    }

    return (0);
} /* end N_ProjectPtLineParam */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine projects a point onto a plane. A typical
     calling example is:

       NL_PLANE  pln;
       NL_POINT  P, Q;
       ...
       (define plane and get P);
       ...
       N_ProjectPtPlane(pln,P,&Q);


   ACCESS:
   
     pln , input  ,  Given plane
     P   , input  ,  Given point
     Q   , output ,  Projection point


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ProjectPtPlane( NL_PLANE pln, NL_POINT P, NL_POINT *Q )
{
    NL_FLAG error, flg;

    NL_LINESEG lsg;

    NL_REAL t;

    NL_VECTOR N;

    /* Get line perpendicular to plane */

    N_PlaneGetNormal( &pln, &N );
    N_CreateLineStartDirVector( &lsg, P, N, NL_UNBOUNDED );

    /* Intersect plane with line */

    error = N_IsectLinePlane( lsg, pln, Q, &t, &flg );

    /* Exit */

    return (error);
} /* end N_ProjectPtPlane */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry  routine  projects a point  onto the  mid-plane of a
     quadrilateral defined by the four corners. The projection is in the 
     direction of the plane normal. The routine also assigns  parameters 
     that do not necessarily correspond to the bi-linear surface parame-
     ters,  however, they are  adequate  approximations  for  subsequent 
     surface constructions. A typical calling example is:

       NL_FLAG  pfl;
       NL_POINT P00, P10, P01, P11, Q;
       NL_REAL  ul, ur, vb, vt, u, v, tol, dis;
       ...
       (define P00,...,P11, get P and tol)
       ...
       N_ProjectPtQuad(P00,P10,P01,P11,P,tol,ul,ur,vb,vt,&Q,&u,&v,&dis,&pfl);


   ACCESS:
   
     P00,P10,P01,P11 , input  ,  Corners of the quadrilateral
     P               , input  ,  Point to be projected
     tol             , input  ,  Point coincidence tolerance
     ul,ur,vb,vt     , input  ,  Parameter bounds
     Q               , output ,  Projection of P
     u,v             , output ,  Parameters assigned to Q
     dis             , output ,  Distance between P and Q
     pfl             , output ,  Parameter flag:
                                   NL_YES: Parameter assigned to Q
                                   NL_NO : No parameter assigned to Q


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ProjectPtQuad( NL_POINT P00, NL_POINT P10, NL_POINT P01, NL_POINT P11, NL_POINT P, NL_REAL tol, NL_REAL ul, NL_REAL ur, NL_REAL vb, NL_REAL vt, NL_POINT *Q, NL_REAL *u, NL_REAL *v, NL_REAL *dis, NL_FLAG *pfl )
{
    NL_PRIVATE NL_STRING rname = _T("N_ProjectPtQuad");

    NL_FLAG flg, error = NL_NO;

    NL_REAL a, b, c, d, tol2, t1, t2, dnu, dnv, ddu, ddv, sgu, sgv, ud, vd, dot;

    NL_POINT U, V, Iu, Iv, G;

    NL_VECTOR Wub, Wut, Wvl, Wvr, A, B, C, D, N;

    NL_LINESEG lnu, lnv, ln1, ln2;

    NL_PLANE pln;

    /* Check input parameters */

    *pfl = NL_YES;
    ud = ur - ul;
    vd = vt - vb;

    if( ud LT NL_PTOL OR vd LT NL_PTOL )
        NL_ERROR( NL_INP_ERR );

    /******************************/
    /* Check for degenerate cases */
    /******************************/

    tol2 = tol * tol;

    N_DistSqPtPt( P00, P10, &a );
    N_DistSqPtPt( P10, P11, &b );
    N_DistSqPtPt( P11, P01, &c );
    N_DistSqPtPt( P01, P00, &d );

    /* The quadrilateral is a point */

    if( a LT tol2 AND b LT tol2 AND c LT tol2 AND d LT tol2 )
    {
        *u = 0.5 *(ul + ur);
        *v = 0.5 *(vb + vt);

        N_VectorCopy( P00, Q );

        NL_OUT;
    }

    /* The quadrilateral is a line segment */

    if( a LT tol2 AND b GT tol2 AND c LT tol2 AND d GT tol2 )
    {
        N_CreateLinePtPt( &lnv, P00, P01, NL_UNBOUNDED );

        error = N_ProjectPtLine( lnv, P, Q, &t2, &flg );

        if( error EQ NL_YES )
            NL_OUT;

        if( flg EQ NL_FALSE )
        {
            *pfl = NL_NO;
            NL_OUT;
        }

        if( t2 LT 0.0 )
            sgv = -1.0;
        else
            sgv = 1.0;

        N_DistPtPt( P00, *Q, &dnv );
        N_DistPtPt( P00, P01, &ddv );

        *u = ul;
        *v = vb + sgv * (dnv / ddv) * vd;

        NL_OUT;
    }

    if( a GT tol2 AND b LT tol2 AND c GT tol2 AND d LT tol2 )
    {
        N_CreateLinePtPt( &lnu, P00, P10, NL_UNBOUNDED );

        error = N_ProjectPtLine( lnu, P, Q, &t1, &flg );

        if( error EQ NL_YES )
            NL_OUT;

        if( flg EQ NL_FALSE )
        {
            *pfl = NL_NO;
            NL_OUT;
        }

        if( t1 LT 0.0 )
            sgu = -1.0;
        else
            sgu = 1.0;

        N_DistPtPt( P00, *Q, &dnu );
        N_DistPtPt( P00, P10, &ddu );

        *u = ul + sgu * (dnu / ddu) * ud;
        *v = vb;

        NL_OUT;
    }

    /* The quadrilateral is a triangle */

    if( a LT tol2 OR b LT tol2 OR c LT tol2 OR d LT tol2 )
    {
        if( a LT tol2 )
        {
            N_VectorDiff( P01, P00, &Wvl );
            N_VectorDiff( P11, P10, &Wvr );
            N_VectorCross( Wvl, Wvr, &N );
            N_CreatePlanePtNormal( &pln, P00, N );

            error = N_ProjectPtPlane( pln, P, Q );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( *Q, P00, &dot );

            if( dot LT tol )
            {
                *u = ul;
                *v = vb;
                NL_OUT;
            }

            N_VectorDiff( P11, P01, &A );
            N_CreateLinePtPt( &lnu, P00, *Q, NL_UNBOUNDED );
            N_CreateLineStartDirVector( &lnv, *Q, A, NL_UNBOUNDED );
            N_CreateLinePtPt( &ln1, P01, P11, NL_UNBOUNDED );
            N_CreateLinePtPt( &ln2, P00, P01, NL_UNBOUNDED );

            error = N_IsectLineLine( lnu, ln1, &U, &t1, &t2, &flg );

            if( error EQ NL_YES )
                NL_OUT;

            if( flg EQ NL_FALSE )
            {
                *u = NL_BIGD;
            }
            else
            {
                if( t2 LT 0.0 )
                    sgu = -1.0;
                else
                    sgu = 1.0;

                N_DistPtPt( P01, U, &dnu );
                N_DistPtPt( P01, P11, &ddu );

                *u = ul + sgu * (dnu / ddu) * ud;
            }

            error = N_IsectLineLine( lnv, ln2, &V, &t1, &t2, &flg );

            if( error EQ NL_YES )
                NL_OUT;

            if( flg EQ NL_FALSE )
            {
                *pfl = NL_NO;
                NL_OUT;
            }

            if( t2 LT 0.0 )
                sgv = -1.0;
            else
                sgv = 1.0;

            N_DistPtPt( P00, V, &dnv );
            N_DistPtPt( P00, P01, &ddv );

            *v = vb + sgv * (dnv / ddv) * vd;
        }
        else if( b LT tol2 )
        {
            N_VectorDiff( P00, P10, &Wub );
            N_VectorDiff( P01, P11, &Wut );
            N_VectorCross( Wub, Wut, &N );
            N_CreatePlanePtNormal( &pln, P10, N );

            error = N_ProjectPtPlane( pln, P, Q );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( *Q, P10, &dot );

            if( dot LT tol )
            {
                *u = ur;
                *v = vb;
                NL_OUT;
            }

            N_VectorDiff( P01, P00, &A );
            N_CreateLineStartDirVector( &lnu, *Q, A, NL_UNBOUNDED );
            N_CreateLinePtPt( &lnv, P10, *Q, NL_UNBOUNDED );
            N_CreateLinePtPt( &ln1, P00, P10, NL_UNBOUNDED );
            N_CreateLinePtPt( &ln2, P00, P01, NL_UNBOUNDED );

            error = N_IsectLineLine( lnu, ln1, &U, &t1, &t2, &flg );

            if( error EQ NL_YES )
                NL_OUT;

            if( flg EQ NL_FALSE )
            {
                *pfl = NL_NO;
                NL_OUT;
            }

            if( t2 LT 0.0 )
                sgu = -1.0;
            else
                sgu = 1.0;

            N_DistPtPt( P00, U, &dnu );
            N_DistPtPt( P00, P10, &ddu );

            *u = ul + sgu * (dnu / ddu) * ud;

            error = N_IsectLineLine( lnv, ln2, &V, &t1, &t2, &flg );

            if( error EQ NL_YES )
                NL_OUT;

            if( flg EQ NL_FALSE )
            {
                *v = NL_BIGD;
            }
            else
            {
                if( t2 LT 0.0 )
                    sgv = -1.0;
                else
                    sgv = 1.0;

                N_DistPtPt( P00, V, &dnv );
                N_DistPtPt( P00, P01, &ddv );

                *v = vb + sgv * (dnv / ddv) * vd;
            }
        }
        else if( c LT tol2 )
        {
            N_VectorDiff( P00, P01, &Wvl );
            N_VectorDiff( P10, P11, &Wvr );
            N_VectorCross( Wvl, Wvr, &N );
            N_CreatePlanePtNormal( &pln, P01, N );

            error = N_ProjectPtPlane( pln, P, Q );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( *Q, P01, &dot );

            if( dot LT tol )
            {
                *u = ul;
                *v = vt;
                NL_OUT;
            }

            N_VectorDiff( P10, P00, &A );
            N_CreateLinePtPt( &lnu, P01, *Q, NL_UNBOUNDED );
            N_CreateLineStartDirVector( &lnv, *Q, A, NL_UNBOUNDED );
            N_CreateLinePtPt( &ln1, P00, P10, NL_UNBOUNDED );
            N_CreateLinePtPt( &ln2, P00, P01, NL_UNBOUNDED );

            error = N_IsectLineLine( lnu, ln1, &U, &t1, &t2, &flg );

            if( error EQ NL_YES )
                NL_OUT;

            if( flg EQ NL_FALSE )
            {
                *u = NL_BIGD;
            }
            else
            {
                if( t2 LT 0.0 )
                    sgu = -1.0;
                else
                    sgu = 1.0;

                N_DistPtPt( P00, U, &dnu );
                N_DistPtPt( P00, P10, &ddu );

                *u = ul + sgu * (dnu / ddu) * ud;
            }

            error = N_IsectLineLine( lnv, ln2, &V, &t1, &t2, &flg );

            if( error EQ NL_YES )
                NL_OUT;

            if( flg EQ NL_FALSE )
            {
                *pfl = NL_NO;
                NL_OUT;
            }

            if( t2 LT 0.0 )
                sgv = -1.0;
            else
                sgv = 1.0;

            N_DistPtPt( P00, V, &dnv );
            N_DistPtPt( P00, P01, &ddv );

            *v = vb + sgv * (dnv / ddv) * vd;
        }
        else if( d LT tol2 )
        {
            N_VectorDiff( P10, P00, &Wub );
            N_VectorDiff( P11, P01, &Wut );
            N_VectorCross( Wub, Wut, &N );
            N_CreatePlanePtNormal( &pln, P00, N );

            error = N_ProjectPtPlane( pln, P, Q );

            if( error EQ NL_YES )
                NL_OUT;

            N_DistPtPt( *Q, P00, &dot );

            if( dot LT tol )
            {
                *u = ul;
                *v = vb;
                NL_OUT;
            }

            N_VectorDiff( P11, P10, &A );
            N_CreateLineStartDirVector( &lnu, *Q, A, NL_UNBOUNDED );
            N_CreateLinePtPt( &lnv, P00, *Q, NL_UNBOUNDED );
            N_CreateLinePtPt( &ln1, P00, P10, NL_UNBOUNDED );
            N_CreateLinePtPt( &ln2, P10, P11, NL_UNBOUNDED );

            error = N_IsectLineLine( lnu, ln1, &U, &t1, &t2, &flg );

            if( error EQ NL_YES )
                NL_OUT;

            if( flg EQ NL_FALSE )
            {
                *pfl = NL_NO;
                NL_OUT;
            }

            if( t2 LT 0.0 )
                sgu = -1.0;
            else
                sgu = 1.0;

            N_DistPtPt( P00, U, &dnu );
            N_DistPtPt( P00, P10, &ddu );

            *u = ul + sgu * (dnu / ddu) * ud;

            error = N_IsectLineLine( lnv, ln2, &V, &t1, &t2, &flg );

            if( error EQ NL_YES )
                NL_OUT;

            if( flg EQ NL_FALSE )
            {
                *v = NL_BIGD;
            }
            else
            {
                if( t2 LT 0.0 )
                    sgv = -1.0;
                else
                    sgv = 1.0;

                N_DistPtPt( P10, V, &dnv );
                N_DistPtPt( P10, P11, &ddv );

                *v = vb + sgv * (dnv / ddv) * vd;
            }
        }

        NL_OUT;
    }

    /***************************/
    /* Handle the general case */
    /***************************/

    N_Combine4Pts( 0.25, P00, 0.25, P10, 0.25, P01, 0.25, P11, &G );

    N_VectorCombine( 0.5, P00, 0.5, P10, &A );
    N_VectorCombine( 0.5, P10, 0.5, P11, &B );
    N_VectorDiff( A, G, &C );
    N_VectorDiff( B, G, &D );
    N_VectorCross( C, D, &N );

    N_CreatePlanePtNormal( &pln, G, N );

    error = N_ProjectPtPlane( pln, P00, &P00 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_ProjectPtPlane( pln, P10, &P10 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_ProjectPtPlane( pln, P01, &P01 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_ProjectPtPlane( pln, P11, &P11 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_ProjectPtPlane( pln, P, Q );

    if( error EQ NL_YES )
        NL_OUT;

    N_CreateLinePtPt( &ln1, P00, P01, NL_UNBOUNDED );
    N_CreateLinePtPt( &ln2, P10, P11, NL_UNBOUNDED );

    error = N_IsectLineLine( ln1, ln2, &Iu, &t1, &t2, &flg );

    if( error EQ NL_YES )
        NL_OUT;

    if( flg EQ NL_TRUE )
    {
        N_CreateLinePtPt( &lnu, Iu, *Q, NL_UNBOUNDED );
    }
    else
    {
        N_VectorDiff( P01, P00, &A );
        N_CreateLineStartDirVector( &lnu, *Q, A, NL_UNBOUNDED );
    }

    N_CreateLinePtPt( &ln1, P00, P10, NL_UNBOUNDED );
    N_CreateLinePtPt( &ln2, P01, P11, NL_UNBOUNDED );

    error = N_IsectLineLine( ln1, ln2, &Iv, &t1, &t2, &flg );

    if( error EQ NL_YES )
        NL_OUT;

    if( flg EQ NL_TRUE )
    {
        N_CreateLinePtPt( &lnv, Iv, *Q, NL_UNBOUNDED );
    }
    else
    {
        N_VectorDiff( P10, P00, &A );
        N_CreateLineStartDirVector( &lnv, *Q, A, NL_UNBOUNDED );
    }

    N_CreateLinePtPt( &ln1, P00, P10, NL_UNBOUNDED );

    error = N_IsectLineLine( lnu, ln1, &U, &t1, &t2, &flg );

    if( error EQ NL_YES )
        NL_OUT;

    if( flg EQ NL_TRUE )
    {
        if( t2 LT 0.0 )
            sgu = -1.0;
        else
            sgu = 1.0;

        N_DistPtPt( P00, U, &dnu );
        N_DistPtPt( P00, P10, &ddu );

        *u = ul + sgu * (dnu / ddu) * ud;
    }
    else
    {
        *u = NL_BIGD;
    }

    N_CreateLinePtPt( &ln2, P00, P01, NL_UNBOUNDED );

    error = N_IsectLineLine( lnv, ln2, &V, &t1, &t2, &flg );

    if( error EQ NL_YES )
        NL_OUT;

    if( flg EQ NL_TRUE )
    {
        if( t2 LT 0.0 )
            sgv = -1.0;
        else
            sgv = 1.0;

        N_DistPtPt( P00, V, &dnv );
        N_DistPtPt( P00, P01, &ddv );

        *v = vb + sgv * (dnv / ddv) * vd;
    }
    else
    {
        *v = NL_BIGD;
    }

    /* Exit */

    EXIT:

    N_DistPtPt( P, *Q, dis );

    return (error);
} /* end N_ProjectPtQuad */

/*******************************************************************//**


   DESCRIPTION:

     Given a parametrized polygon, i.e. each vertex is associated with a
     parameter, and a point, this geometry routine  computes the closest 
     projection  of  this  point  onto  the  polygon, the  corresponding 
     parameter, and the shortest distance. The best uses of this routine
     are to find start parameters for point projection, or to get appro-
     ximate parameters for shaping. A typical calling example is:

       NL_POINT  *P, Q, R;
       NL_REAL   *u, t, d;
       NL_INDEX  n;
       ...
       (get arrays P and u, and point Q);
       ...
       N_ProjectPtClosePolyPt(P,u,n,Q,&R,&t,&d)


   ACCESS:
   
     P   , input  ,  Polygon vertices
     u   , input  ,  Parameters corresponding to P[i]
     n   , input  ,  Highest index in P and u
     Q   , input  ,  Point to be projected
     R   , output ,  Closest point on the polygon
     t   , output ,  Parameter corresponding to R
     d   , output ,  Shortest distance, i.e. dist(Q,R)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_ProjectPtClosePolyPt( NL_POINT *P, NL_REAL *u, NL_INDEX n, NL_POINT Q, NL_POINT *R, NL_REAL *t, NL_REAL *d )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL dis, d1, d2, dv, dl, s, pv = 0.0, pl = 0.0;

    NL_POINT T, Rv = { 0,0,0 }, Rl = { 0,0,0 };

    /* Project point onto each leg and get approximate parameter */

    dl = NL_BIGD;

    for ( i = 1; i <= n; i++ )
    {
        /* Project point */

        error = N_ProjectPtLineParam( Q, P[i - 1], P[i], &T, &s );

        if( error EQ NL_YES )
            NL_OUT;

        /* Find parameter */

        N_DistPtPt( T, P[i - 1], &d1 );
        N_DistPtPt( T, P[i], &d2 );

        if( d1 LT NL_MTOL )
        {
            *t = u[i - 1];
            *d = d1;
            N_VectorCopy( T, R );
            NL_OUT;
        }
        else if( d2 LT NL_MTOL )
        {
            *t = u[i];
            *d = d2;
            N_VectorCopy( T, R );
            NL_OUT;
        }
        else if( s GT 0.0 AND s LT 1.0 )
        {
            N_DistPtPt( Q, T, &dis );

            if( dis LT dl )
            {
                dl = dis;
                pl = u[i - 1] + s * (u[i] - u[i - 1]);

                N_VectorCopy( T, &Rl );
            }
        }
    }

    /* Check with the vertices */

    dv = NL_BIGD;

    for ( i = 0; i <= n; i++ )
    {
        N_DistPtPt( Q, P[i], &dis );

        if( dis LT dv )
        {
            dv = dis;
            pv = u[i];
            N_VectorCopy( P[i], &Rv );
        }
    }

    /* Get the output */

    if( dv LT dl )
    {
        *t = pv;
        *d = dv;
        N_VectorCopy( Rv, R );
    }
    else
    {
        *t = pl;
        *d = dl;
        N_VectorCopy( Rl, R );
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_ProjectPtClosePolyPt */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the distance between two points.
     A typical calling example is:

       NL_POINT  P, Q;
       NL_REAL   d;
       ... 
       (get P and Q);
       ...
       N_DistPtPt(P,Q,&d);


   ACCESS:
   
     P  , input  ,  First point
     Q  , input  ,  Second point
     d  , output ,  Distance


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistPtPt( NL_POINT P, NL_POINT Q, NL_REAL *d )
{
    NL_REAL disc;

    disc = (P.x - Q.x) * (P.x - Q.x) + (P.y - Q.y) * (P.y - Q.y) + (P.z - Q.z) * (P.z - Q.z);

    *d = sqrt( disc );
} /* end N_DistPtPt */



/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the distance between two points in
     2-D. A typical calling example is:

       NL_POINT  P, Q;
       NL_REAL   d;
       ... 
       (get P and Q);
       ...
       N_DistPtPt2d(P,Q,&d);


   ACCESS:
   
     P  , input  ,  First 2-D point
     Q  , input  ,  Second 2-D point
     d  , output ,  Distance


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistPtPt2d( NL_POINT P, NL_POINT Q, NL_REAL *d )
{
    NL_REAL disc;

    disc = (P.x - Q.x) * (P.x - Q.x) + (P.y - Q.y) * (P.y - Q.y);

    *d = sqrt( disc );
} /* end N_DistPtPt2d */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the square of the distance between
     two points. A typical calling example is:

       NL_POINT  P, Q;
       NL_REAL   dsq;
       ... 
       (get P and Q);
       ...
       N_DistSqPtPt(P,Q,&dsq);


   ACCESS:
   
     P    , input  ,  First point
     Q    , input  ,  Second point
     dsq  , output ,  Squared distance


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistSqPtPt( NL_POINT P, NL_POINT Q, NL_REAL *dsq )
{
    *dsq = (P.x - Q.x) * (P.x - Q.x) + (P.y - Q.y) * (P.y - Q.y) + (P.z - Q.z) * (P.z - Q.z);
} /* end N_DistSqPtPt */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the square of the distance between
     two points in 2-D. A typical calling example is:

       NL_POINT  P, Q;
       NL_REAL   dsq;
       ... 
       (get P and Q);
       ...
       N_DistSqPtPt2d(P,Q,&dsq);


   ACCESS:
   
     P    , input  ,  First point in 2-D
     Q    , input  ,  Second point in 2-D
     dsq  , output ,  Squared distance


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistSqPtPt2d( NL_POINT P, NL_POINT Q, NL_REAL *dsq )
{
    *dsq = (P.x - Q.x) * (P.x - Q.x) + (P.y - Q.y) * (P.y - Q.y);
} /* end N_DistSqPtPt2d */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine checks if two points are identical. A typical 
     calling example is:

       NL_POINT  P, Q;
       ...
       (get points);
       ...
       if( N_PtsAreEqual(P,Q) )  --> points are identical
     

   ACCESS:
   
     P  , input ,  Point
     Q  , input ,  Point


   RETURN CODES:

     NL_TRUE : Points are identical
     NL_FALSE: Points are NOT identical

   ***********************************************************************/

NL_BOOLEAN N_PtsAreEqual( NL_POINT P, NL_POINT Q )
{
    if( P.x EQ Q.x AND P.y EQ Q.y AND P.z EQ Q.z )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_PtsAreEqual */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This geometry routine computes the distance between a point P and
     a line segment AB. If the projection of P falls  between A and B,
     the  perpendicular distance  from P to AB is  computed. Otherwise 
     the closest of  d(P,A) and d(P,B) is  returned. A typical calling
     example is:
 
       NL_POINT  P, A, B;
       NL_REAL   d;
       ...
       (get P, A and B);
       ...
       N_DistPtLineSeg(P,A,B,&d);
 
 
   ACCESS:
   
     P   , input  ,  Given point
     A,B , input  ,  End points of line segment
     d   , output ,  Distance between P and AB
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_DistPtLineSeg
 ( NL_POINT P,  /* in : point                                */ 
   NL_POINT A,  /* in : start line segment                   */ 
   NL_POINT B,  /* in : end line segment                     */ 
   NL_REAL *d ) /* out: min dist between point P and line AB */ 
{
    NL_PRIVATE NL_STRING rname = _T("N_DistPtLineSeg");

    NL_VECTOR BA, PA, BP, T;

    NL_REAL dotap, dotbp, dotab, num, den;

    /* Check special case */

    N_DistPtPt( A, B, &dotab );

    if( dotab LT NL_MTOL )
    {
        N_VectorSum( A, B, &T );
        N_VectorScale( T, 0.5, &T );
        N_DistPtPt( P, T, d );

        return (0);
    }

    /* Compute some quantities */

    N_VectorDiff( B, A, &BA );
    N_VectorDiff( P, A, &PA );
    N_VectorDiff( B, P, &BP );

    N_VectorDot( PA, BA, &dotap );
    N_VectorDot( BP, BA, &dotbp );

    /* Compute distance for different positions of P */

    if( dotap GE 0.0 AND dotbp GE 0.0 )
    {
        N_VectorCross( PA, BA, &T );
        N_VectorMagnitude( T, &num );

        den = dotab;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
        {
            N_ErrSet( NL_NUM_ERR, rname );
            return (1);
        }

        *d = num / den;
    }
    else if( dotap LT 0.0 )
    {
        N_DistPtPt( P, A, d );
    }
    else if( dotbp LT 0.0 )
    {
        N_DistPtPt( P, B, d );
    }

    return (0);
} /* end N_DistPtLineSeg */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This geometry routine computes the distance between a point Q and
     a line segment AB. If the projection of Q falls  between A and B,
     the  perpendicular distance  from Q to AB is  computed. Otherwise 
     -1.0 is  returned. A typical calling example is:
 
       NL_POINT  Q, A, B;
       NL_REAL   d;
       ...
       (get Q, A and B);
       ...
       N_DistPerpPtLineSeg(Q,A,B,&d);
 
 
   ACCESS:
   
     Q   , input  ,  Given point
     A,B , input  ,  End points of line segment
     d   , output ,  Distance between Q and AB
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_DistPerpPtLineSeg( NL_POINT Q, NL_POINT A, NL_POINT B, NL_REAL *d )
{
    NL_PRIVATE NL_STRING rname = _T("N_DistPerpPtLineSeg");

    NL_VECTOR BA, QA, BQ, T;

    NL_REAL dotaq, dotbq, dotab, num, den;

    /* Check special case */

    N_DistPtPt( A, B, &dotab );

    if( dotab LT NL_MTOL )
    {
        N_VectorSum( A, B, &T );
        N_VectorScale( T, 0.5, &T );
        N_DistPtPt( Q, T, d );

        return (0);
    }

    /* Compute some quantities */

    N_VectorDiff( B, A, &BA );
    N_VectorDiff( Q, A, &QA );
    N_VectorDiff( B, Q, &BQ );

    N_VectorDot( QA, BA, &dotaq );
    N_VectorDot( BQ, BA, &dotbq );

    /* Compute the distance */

    if( dotaq LT 0.0 OR dotbq LT 0.0 )
    {
        *d = -1.0;
        return (0);
    }

    N_VectorDot( BA, BA, &dotab );
    N_VectorCross( QA, BA, &T );
    N_VectorMagnitude( T, &num );

    if( dotab LT 0.0 )
        dotab = 0.0;
    den = sqrt( dotab );

    if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
    {
        N_ErrSet( NL_NUM_ERR, rname );
        return (1);
    }

    *d = num / den;

    return (0);
} /* end N_DistPerpPtLineSeg */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the length of a polygon. A typical 
     calling example is:

       NL_INDEX  n;
       NL_POINT  *P;
       NL_REAL   len;
       ... 
       (get array P);
       ...
       N_DistPolygon(P,n,&len);


   ACCESS:
   
     P   , input  ,  Point array
     n   , input  ,  Highest index in P
     len , output ,  Length of polygon


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistPolygon( NL_POINT *P, NL_INDEX n, NL_REAL *len )
{
    NL_INDEX i;

    NL_REAL d;

    *len = 0.0;

    for ( i = 1; i <= n; i++ )
    {
        N_DistPtPt( P[i - 1], P[i], &d );
        *len += d;
    }
} /* end N_DistPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the length of a control polygon. A 
     typical calling example is:

       NL_INDEX   n;
       NL_CPOINT  *Pw;
       NL_REAL    len;
       ... 
       (get array Pw);
       ...
       N_DistCPolygon(Pw,n,&len);


   ACCESS:
   
     Pw  , input  ,  Control point array
     n   , input  ,  Highest index in Pw
     len , output ,  Length of control polygon


   RETURN CODES:

     None

   ***********************************************************************/
NL_VOID N_DistCPolygon( NL_CPOINT *Pw, NL_INDEX n, NL_REAL *len )
{
    NL_INDEX i;

    NL_REAL d;

    *len = 0.0;

    for ( i = 1; i <= n; i++ )
    {
        N_DistCptCpt( Pw[i - 1], Pw[i], &d );
        *len += d;
    }
} /* end N_DistCPolygon */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the distance between two control 
     points. A typical calling example is:

       NL_CPOINT  Pw, Qw;
       NL_REAL    d;
       ...
       (get Pw and Qw);
       ...
       N_DistCptCpt(Pw,Qw,&d);


   ACCESS:
   
     Pw  , input  ,  First control point
     Qw  , input  ,  Second control point
     d   , output ,  Distance


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistCptCpt( NL_CPOINT Pw, NL_CPOINT Qw, NL_REAL *d )
{
    NL_POINT P, Q;

    /* Map to the Euclidean space */

    N_CPtToPtEuclid( Pw, &P );
    N_CPtToPtEuclid( Qw, &Q );

    /* Compute distance */

    N_DistPtPt( P, Q, d );
} /* end N_DistCptCpt */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the homogenous distance between 
     two control points. A typical calling example is:

       NL_CPOINT  Pw, Qw;
       NL_REAL    dw;
       ...
       (get Pw and Qw);
       ...
       N_DistCptCptHomo(Pw,Qw,&dw);


   ACCESS:
   
     Pw  , input  ,  First control point
     Qw  , input  ,  Second control point
     dw  , output ,  Distance


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistCptCptHomo( NL_CPOINT Pw, NL_CPOINT Qw, NL_REAL *dw )
{
    NL_REAL disc;
    disc = (Pw.x - Qw.x) * (Pw.x - Qw.x) + (Pw.y - Qw.y) * (Pw.y - Qw.y);

    if( Pw.z NEQ NL_NOZ AND Qw.z NEQ NL_NOZ )
        disc += (Pw.z - Qw.z) * (Pw.z - Qw.z);

    if( Pw.w NEQ NL_NOW AND Qw.w NEQ NL_NOW )
        disc += (Pw.w - Qw.w) * (Pw.w - Qw.w);

    *dw = sqrt( disc );
} /* end N_DistCptCptHomo */


/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the distance of a point from an
     infinite line. A typical calling example is:

       NL_LINESEG  lsg;
       NL_POINT    P;
       NL_REAL     d;
       ...
       (define line and get P);
       ...
       N_DistPtInfLine(lsg,P,&d);


   ACCESS:
   
     lsg , input  ,  Given line
     P   , input  ,  Given point
     d   , output ,  Distance of point from line


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_DistPtInfLine( NL_LINESEG lsg, NL_POINT P, NL_REAL *d )
{
    NL_FLAG error, flg;

    NL_POINT Q;

    NL_REAL t;

    /* Project point onto the line */

    N_LineSetBoundingFlag( &lsg, NL_UNBOUNDED );

    error = N_ProjectPtLine( lsg, P, &Q, &t, &flg );

    if( error EQ NL_YES )
        return (1);

    /* Compute distance */

    N_DistPtPt( P, Q, d );

    /* Exit */

    return (0);
} /* end N_DistPtInfLine */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the distance of a point from a
     plane. A typical calling example is:

       NL_PLANE  pln;
       NL_POINT  P;
       NL_REAL   d;
       ...
       (define plane and get P);
       ...
       N_DistPtPlane(pln,P,&d);


   ACCESS:
   
     pln , input  ,  Given plane
     P   , input  ,  Given point
     d   , output ,  Distance of point from plane


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_DistPtPlane( NL_PLANE pln, NL_POINT P, NL_REAL *d )
{
    NL_FLAG error;

    NL_POINT Q;

    /* Project point onto the plane */

    error = N_ProjectPtPlane( pln, P, &Q );

    if( error EQ NL_YES )
        return (1);

    /* Compute distance */

    N_DistPtPt( P, Q, d );

    /* Exit */

    return (0);
} /* end N_DistPtPlane */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the SIGNED distance of a point from 
     a plane given in the following form:

        a*x + b*y + c*z + d = 0

     where  (a,b,c) are normalized, i.e. they represent the unit normal 
     to the plane. A typical calling example is:

       NL_POINT  P;
       NL_REAL   a, b, c, d, dist;
       ...
       (define plane and get P);
       ...
       N_DistSignedPtPlane(a,b,c,d,P,&dist);


   ACCESS:
   
     a,b,c,d , input  ,  Plane's normalized implicit equation
     P       , input  ,  Given point
     dist    , output ,  SIGNED distance of point from plane


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistSignedPtPlane( NL_REAL a, NL_REAL b, NL_REAL c, NL_REAL d, NL_POINT P, NL_REAL *dist )
{
    NL_REAL x, y, z;

    /* Compute distance */

    N_PtToXYZ( P, &x, &y, &z );

    *dist = a * x + b * y + c * z + d;
} /* end N_DistSignedPtPlane */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the extended bounding box of a line
     segment, i.e. it computes the bounding box of the line and offsets
     it by a given distance. A typical calling example is:

       NL_POINT      P, Q;
       NL_REAL       r;
       NL_MINMAXBOX  box;
       ...
       (get P and Q);
       ...
       N_LineCalcBBox(P,Q,r,&box);


   ACCESS:
   
     P,Q , input  ,  Start and end points of line
     r   , input  ,  Offset distance
     box , output ,  Bounding box


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_LineCalcBBox( NL_POINT P, NL_POINT Q, NL_REAL r, NL_MINMAXBOX *box )
{
    NL_REAL xl, xr, yb, yt, zn, zf, xp, yp, zp, xq, yq, zq;

    N_PtToXYZ( P, &xp, &yp, &zp );
    N_PtToXYZ( Q, &xq, &yq, &zq );

    xl = xr = xp;
    yb = yt = yp;
    zn = zf = zp;

    if( xq LT xl )
        xl = xq;

    if( xq GT xr )
        xr = xq;

    if( yq LT yb )
        yb = yq;

    if( yq GT yt )
        yt = yq;

    if( zq LT zn )
        zn = zq;

    if( zq GT zf )
        zf = zq;

    xl -= r;
    xr += r;
    yb -= r;
    yt += r;
    zn -= r;
    zf += r;

    N_BBoxDefine( box, xl, xr, yb, yt, zn, zf );
} /* end N_LineCalcBBox */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine sets the bounding flag of a line. A typical 
     calling example is:

       NL_LINESEG  lsg;
       ...
       (get lsg);
       ...
       N_LineSetBoundingFlag(&lsg,NL_UNBOUNDED);


   ACCESS:
   
     lsg , in/out ,  Line segment
     flg , input  ,  Flag:
                       NL_UNBOUNDED: Line is unbounded
                         NL_BOUNDED: Line is bounded



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_LineSetBoundingFlag( NL_LINESEG *lsg, NL_FLAG flg )
{
    lsg->bounded = flg;
} /* end N_LineSetBoundingFlag */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine gets the direction vector of a line. A 
     typical calling example is:

       NL_LINESEG  lsg;
       NL_VECTOR   V;
       ...
       (get lsg);
       ...
       N_LineGetDir(&lsg,&V);


   ACCESS:
   
     lsg , input  ,  Line segment
     V   , output ,  Direction vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_LineGetDir( NL_LINESEG *lsg, NL_VECTOR *V )
{
    *V = lsg->V;
} /* end N_LineGetDir */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine breaks a line object down to its components. 
     A typical calling example is:

       NL_LINESEG  lsg;
       NL_POINT    P;
       NL_VECTOR   V;
       NL_FLAG     b;
       ...
       (get lsg);
       ...
       N_LineGetData(&lsg,&P,&V,&b);


   ACCESS:
   
     lsg  , input  ,  Line
     P    , output ,  Start point
     V    , output ,  Direction vetor
     b    , output ,  Flag:
                        NL_BOUNDED
                        NL_UNBOUNDED


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_LineGetData( NL_LINESEG *lsg, NL_POINT *P, NL_VECTOR *V, NL_FLAG *b )
{
    *P = lsg->P;
    *V = lsg->V;
    *b = lsg->bounded;
} /* end N_LineGetData */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the maximum extent of a 1-D point
     array. A typical calling example is:

       NL_POINT  *P;
       NL_INDEX  n;
       NL_REAL   diag;
       ...
       (get array P);
       ...
       N_Pts1dGetMaxExtent(P,n,&diag);


   ACCESS:
   
     P    , input  ,  Point array
     n    , input  ,  Highest index in P
     diag , output ,  Diagonal of bounding box (maximum extent) 


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Pts1dGetMaxExtent( NL_POINT *P, NL_INDEX n, NL_REAL *diag )
{
    NL_EPOLYGON ppl;

    NL_MINMAXBOX box;

    /* Compute min-max box's diagonal */

    N_EPolygonFromPts( &ppl, n, P );
    N_PolygonGetBBox( &ppl, &box );
    N_BBoxGetDiagonal( &box, diag );
} /* end N_Pts1dGetMaxExtent */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the maximum extent of a 2-D point
     array. A typical calling example is:

       NL_POINT  **P;
       NL_INDEX  n, m;
       NL_REAL   diag;
       ...
       (get array P);
       ...
       N_Pts2dGetMaxExtent(P,n,m,&diag);


   ACCESS:
   
     P    , input  ,  Point array
     n,m  , input  ,  Highest indexes in P
     diag , output ,  Diagonal of bounding box (maximum extent) 


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Pts2dGetMaxExtent( NL_POINT ** P, NL_INDEX n, NL_INDEX m, NL_REAL *diag )
{
    NL_ENET ntl;

    NL_MINMAXBOX box;

    /* Compute min-max box's diagonal */

    N_ENetFromPts( &ntl, n, m, P );
    N_ENetGetBBox( &ntl, &box );
    N_BBoxGetDiagonal( &box, diag );
} /* end N_Pts2dGetMaxExtent */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the length of the diagonal of a 
     box. A typical calling example is:

       NL_MINMAXBOX  box;
       NL_REAL       diag;
       ...
       (get box);
       ...
       N_BBoxGetDiagonal(&box,&diag);


   ACCESS:
   
     box  , input  ,  Box
     diag , output ,  Length of the diagonal


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BBoxGetDiagonal( NL_MINMAXBOX *box, NL_REAL *diag )
{
    NL_REAL xl, xr, yb, yt, zn, zf;

    NL_POINT Pll, Pur;

    /* Get local notation */

    N_GetBBoxData( box, &xl, &xr, &yb, &yt, &zn, &zf );

    /* Compute diagonal */

    N_PtFromXYZ( xl, yb, zn, &Pll );
    N_PtFromXYZ( xr, yt, zf, &Pur );
    N_DistPtPt( Pll, Pur, diag );
} /* end N_BBoxGetDiagonal */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the center of a box. A typical 
     calling example is:

       NL_MINMAXBOX  box;
       NL_POINT      C;
       ...
       (get box);
       ...
       N_BBoxGetCenter(&box,&C);


   ACCESS:
   
     box , input  ,  Box
     C   , output ,  Center of box


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BBoxGetCenter( NL_MINMAXBOX *box, NL_POINT *C )
{
    NL_REAL xl, xr, yb, yt, zn, zf;

    NL_POINT Pll, Pur;

    /* Get local notation */

    N_GetBBoxData( box, &xl, &xr, &yb, &yt, &zn, &zf );

    /* Compute center */

    N_PtFromXYZ( xl, yb, zn, &Pll );
    N_PtFromXYZ( xr, yt, zf, &Pur );
    N_Combine2Pts( 0.5, Pll, 0.5, Pur, C );
} /* end N_BBoxGetCenter */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the lengths of a box in x-, y- and 
     z-directions. A typical calling example is:

       NL_MINMAXBOX  box;
       NL_REAL       xd, yd, zd;
       ...
       (get box);
       ...
       N_BBoxGetDimensions(&box,&xd,&yd,&zd);


   ACCESS:
   
     box      , input  ,  Box
     xd,yd,zd , output ,  Dimensions in the principal directions


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BBoxGetDimensions( NL_MINMAXBOX *box, NL_REAL *xd, NL_REAL *yd, NL_REAL *zd )
{
    NL_REAL xl, xr, yb, yt, zn, zf;

    /* Get local notation */

    N_GetBBoxData( box, &xl, &xr, &yb, &yt, &zn, &zf );

    /* Compute the dimensions */

    *xd = fabs( xr - xl );
    *yd = fabs( yt - yb );
    *zd = fabs( zf - zn );
} /* end N_BBoxGetDimensions */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the area of overlap of two 2-D 
     boxes. A typical calling example is:

       NL_REAL  xl, xr, yb, yt, ul, ur, vb, vt, tol, A;
       ...
       (get box data);
       ...
       N_BBox2dCalcOverlap(xl,xr,yb,yt,ul,ur,vb,vt,tol,&A);


   ACCESS:
   
     xl,xr,yb,yt , input  ,  Box
     ul,ur,vb,vt , input  ,  Box
     tol         , input  ,  Line coincidence tolerance
     A           , output ,  Area of overlap:
                               >= 0.0 : boxes overlap
                               <  0.0 : boxes do not overlap


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BBox2dCalcOverlap( NL_REAL xl, NL_REAL xr, NL_REAL yb, NL_REAL yt, NL_REAL ul, NL_REAL ur, NL_REAL vb, NL_REAL vt, NL_REAL tol, NL_REAL *A )
{
    NL_REAL w, h;

    /* Compute area */

    w = NL_MIN( xr, ur ) - NL_MAX( xl, ul );
    h = NL_MIN( yt, vt ) - NL_MAX( yb, vb );

    if( w LT - tol OR h LT - tol )
        *A = -1.0;

    else if( w LT tol OR h LT tol )
        *A = 0.0;

    else
        *A = w * h;
} /* end N_BBox2dCalcOverlap */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine makes a line object given the two end 
     points of the line. A typical calling example is:

       NL_LINESEG  lsg;
       NL_POINT    Ps, Pe;
       ...
       (get Ps and Pe);
       ...
       N_CreateLinePtPt(&lsg,Ps,Pe,NL_UNBOUNDED);


   ACCESS:
   
     lsg , output ,  Line segment
     Ps  , input  ,  Start point
     Pe  , input  ,  End point
     flg , input  ,  Flag:
                       NL_UNBOUNDED: Line is unbounded
                         NL_BOUNDED: Line is bounded by <Ps,Pe>



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CreateLinePtPt( NL_LINESEG *lsg, NL_POINT Ps, NL_POINT Pe, NL_FLAG flg )
{
    NL_VECTOR V;

    /* Copy vector data into line structure */

    N_VectorDir( Ps, Pe, &V );
    N_CreateLineStartDirVector( lsg, Ps, V, flg );
} /* end N_CreateLinePtPt */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine makes a line object given the start point
     and a direction vector. A typical example is:

       NL_LINESEG  lsg;
       NL_POINT    P;
       NL_VECTOR   V;
       ...
       (get P and V);
       ...
       N_CreateLineStartDirVector(&lsg,P,V,NL_BOUNDED);


   ACCESS:
   
     lsg , output ,  Line segment
     P   , input  ,  Start point
     V   , input  ,  Direction vector
     flg , input  ,  Flag:
                       NL_UNBOUNDED: Line is unbounded
                         NL_BOUNDED: Line is bounded by <P,P+V>


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CreateLineStartDirVector( NL_LINESEG *lsg, NL_POINT P, NL_VECTOR V, NL_FLAG flg )
{
    lsg->P = P;
    lsg->V = V;
    lsg->bounded = flg;
} /* end N_CreateLineStartDirVector */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine makes a plane object given a point and the
     normal vector. A typical calling example is:

       NL_PLANE   pln;
       NL_POINT   P;
       NL_VECTOR  N;
       ...
       (get P and N);
       ...
       N_CreatePlanePtNormal(&pln,P,N);


   ACCESS:
   
     pln , output ,  Plane
     P   , input  ,  Point lying on the plane
     N   , input  ,  Normal vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CreatePlanePtNormal
 (NL_PLANE *pln, /* out: Plane */
  NL_POINT  P,   /* in : Point lying on the plane */
  NL_VECTOR N )  /* in : Normal vector */
{
    pln->P = P;
    pln->N = N;
} /* end N_CreatePlanePtNormal */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine makes an interval object given the left and 
     right values. A typical calling example is:

       NL_INTERVAL  I;
       NL_REAL      ul, ur;
       ...
       (get ul and ur);
       ...
       N_CreateInterval(&I,ul,ur);


   ACCESS:
   
     I  , output ,  Interval
     ul , input  ,  Left end
     ur , input  ,  Right end



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CreateInterval( NL_INTERVAL *I, NL_REAL ul, NL_REAL ur )
{
    I->ul = ul;
    I->ur = ur;
} /* end N_CreateInterval */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine breaks an interval object down to its 
     components. A typical calling example is:

       NL_INTERVAL  I;
       NL_REAL      ul, ur;
       ...
       (get I);
       ...
       N_IntervalGetData(&I,&ul,&ur);


   ACCESS:
   
     I  , input  ,  Interval
     ul , output ,  Left end
     ur , output ,  Right end



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_IntervalGetData( NL_INTERVAL *I, NL_REAL *ul, NL_REAL *ur )
{
    *ul = I->ul;
    *ur = I->ur;
} /* end N_IntervalGetData */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine makes a rectangle object given the left, 
     right, bottom and top values. A typical calling example is:

       NL_RECTANGLE  R;
       NL_REAL       ul, ur, vb, vt;
       ...
       (get ul, ur, vb and vt);
       ...
       N_CreateRectangle(&R,ul,ur,vb,vt);


   ACCESS:
   
     R  , output ,  Rectangle
     ul , input  ,  Left 
     ur , input  ,  Right
     vb , input  ,  Bottom
     vt , input  ,  Top



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CreateRectangle( NL_RECTANGLE *R, NL_REAL ul, NL_REAL ur, NL_REAL vb, NL_REAL vt )
{
    R->ul = ul;
    R->ur = ur;
    R->vb = vb;
    R->vt = vt;
} /* end N_CreateRectangle */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine breaks a rectangle object down to its
     components. A typical calling example is:

       NL_RECTANGLE  R;
       NL_REAL       ul, ur, vb, vt;
       ...
       (get R);
       ...
       N_RectangleGetData(&R,&ul,&ur,&vb,&vt);


   ACCESS:
   
     R  , input  ,  Rectangle
     ul , output ,  Left 
     ur , output ,  Right
     vb , output ,  Bottom
     vt , output ,  Top



   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_RectangleGetData( NL_RECTANGLE *R, NL_REAL *ul, NL_REAL *ur, NL_REAL *vb, NL_REAL *vt )
{
    *ul = R->ul;
    *ur = R->ur;
    *vb = R->vb;
    *vt = R->vt;
} /* end N_RectangleGetData */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine breaks a min-max box object down to its 
     components. A typical calling example is:

       NL_MINMAXBOX  box;
       NL_REAL       xl, xr, yb, yt, zn, zf;
       ...
       (get box);
       ...
       N_GetBBoxData(&box,&xl,&xr,&yb,&yt,&zn,&zf);


   ACCESS:
   
     box   , input  ,  Min-max box
     xl,xr , output ,  Left and right
     yb,yt , output ,  Bottom and top
     zn,zf , output ,  Near and far


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_GetBBoxData( NL_MINMAXBOX *box, NL_REAL *xl, NL_REAL *xr, NL_REAL *yb, NL_REAL *yt, NL_REAL *zn, NL_REAL *zf )
{
    *xl = box->xl;
    *xr = box->xr;
    *yb = box->yb;
    *yt = box->yt;
    *zn = box->zn;
    *zf = box->zf;
} /* end N_GetBBoxData */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine defines a bounding box. A typical calling 
     example is:

       NL_MINMAXBOX  box;
       NL_REAL       xl, xr, yb, yt, zn, zf;
       ...
       (get xl, xy, yb, yt, zn and zf);
       ...
       N_BBoxDefine(&box,xl,xr,yb,yt,zn,zf);


   ACCESS:
   
     box   , output ,  Min-max box
     xl,xr , input  ,  Left and right
     yb,yt , input  ,  Bottom and top
     zn,zf , input  ,  Near and far


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_BBoxDefine( NL_MINMAXBOX *box, NL_REAL xl, NL_REAL xr, NL_REAL yb, NL_REAL yt, NL_REAL zn, NL_REAL zf )
{
    box->xl = xl;
    box->xr = xr;
    box->yb = yb;
    box->yt = yt;
    box->zn = zn;
    box->zf = zf;
} /* end N_BBoxDefine */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine breaks a plane object down to its components. 
     A typical calling example is:

       NL_PLANE   pln;
       NL_POINT   P;
       NL_VECTOR  N;
       ...
       (get pln);
       ...
       N_PlaneGetData(&pln,&P,&N);


   ACCESS:
   
     pln  , input  ,  Plane
     P    , output ,  Point on plane
     N    , output ,  Normal to plane


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PlaneGetData( NL_PLANE *pln, NL_POINT *P, NL_VECTOR *N )
{
    *P = pln->P;
    *N = pln->N;
} /* end N_PlaneGetData */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine gets the normal vector of a plane. A typical 
     calling example is:

       NL_PLANE   pln;
       NL_VECTOR  N;
       ...
       (get pln);
       ...
       N_PlaneGetNormal(&pln,&N);


   ACCESS:
   
     pln , input  ,  Plane
     N   , output ,  Normal


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_PlaneGetNormal( NL_PLANE *pln, NL_VECTOR *N )
{
    *N = pln->N;
} /* end N_PlaneGetNormal */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine  computes a general rotation matrix  given 
     the axis of rotation  and the rotation  angle. A typical calling
     example is:
  
       NL_POINT    P;
       NL_VECTOR   V;
       NL_REAL     al;
       NL_RMATRIX  rma;
       NL_STACKS   S;
       ...
       (get P, V and al);
       ...
       N_InitRealMatrix(&rma);
       N_CreateRotationMatrixAboutAxis(P,V,al,&rma,&S);

     If rma is initialized to the NULL matrix, memory is allocated in
     the  routine. If not, it is  assumed that  memory allocation has 
     been done. However, the routine checks for  the proper amount by
     looking  at  the  matrix  indexes (which  are  assumed to be set 
     correctly).  


   ACCESS:
   
     P,V  , input  ,  Point and direction vector of axis of rotation
     al   , input  ,  Angle of rotation
     rma  , output ,  Rotation matrix
     S    , input  ,  rma's stack



   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateRotationMatrixAboutAxis( NL_POINT P, NL_VECTOR V, NL_REAL al, NL_RMATRIX *rma, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateRotationMatrixAboutAxis");

    NL_FLAG error;

    NL_REAL ** RM, cosal, sinal, omcos, vx2, vy2, vz2, xv, yv, zv, vxy, vyz, vzx, rad, xp, yp, zp;

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( rma, 3, 3, NL_MT_FULL, 3, rname, S );

    if( error EQ NL_YES )
        return (1);

    N_GetRealMatrixPtr( rma, &RM );

    /* Normalize input vector */

    error = N_VectorNormalizeRef( &V );

    if( error EQ NL_YES )
        return (1);

    /* Get some parameters */

    N_PtToXYZ( P, &xp, &yp, &zp );
    N_PtToXYZ( V, &xv, &yv, &zv );

    rad = (al / 180.0) * NL_PI;
    cosal = cos( rad );
    sinal = sin( rad );
    omcos = 1.0 - cosal;

    vx2 = xv * xv;
    vy2 = yv * yv;
    vz2 = zv * zv;
    vxy = xv * yv;
    vyz = yv * zv;
    vzx = zv * xv;

    /* Compute the rotation matrix */

    RM[0][0] = vx2 + cosal * (1.0 - vx2);
    RM[0][1] = vxy * omcos - zv * sinal;
    RM[0][2] = vzx * omcos + yv * sinal;
    RM[0][3] = xp - (xp * RM[0][0] + yp * RM[0][1] + zp * RM[0][2]);

    RM[1][0] = vxy * omcos + zv * sinal;
    RM[1][1] = vy2 + cosal * (1.0 - vy2);
    RM[1][2] = vyz * omcos - xv * sinal;
    RM[1][3] = yp - (xp * RM[1][0] + yp * RM[1][1] + zp * RM[1][2]);

    RM[2][0] = vzx * omcos - yv * sinal;
    RM[2][1] = vyz * omcos + xv * sinal;
    RM[2][2] = vz2 + cosal * (1.0 - vz2);
    RM[2][3] = zp - (xp * RM[2][0] + yp * RM[2][1] + zp * RM[2][2]);

    RM[3][0] = 0.0;
    RM[3][1] = 0.0;
    RM[3][2] = 0.0;
    RM[3][3] = 1.0;

    /* Exit */

    return (0);
} /* end N_CreateRotationMatrixAboutAxis */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes a transformation matrix that maps 
     a given  orthonormal  coordinate frame  into another. The matrix
     multiplication is performed from the left, i.e.

           |O1.x P1.x Q1.x R1.x|   |O2.x P2.x Q2.x R2.x|
       [A] |O1.y P1.y Q1.y R1.y| = |O2.y P2.y Q2.y R2.y| 
           |O1.z P1.z Q1.z R1.z|   |O2.z P2.z Q2.z R2.z|
           | 1    1    1    1  |   | 1    1    1    1  |

     where A  is a  4x4  matrix,  O1  and  O2  are  the  origins,  and 
     (P1,Q1,R1)  and  (P2,Q2,R2) are  the end points of the respective 
     axes. A typical calling example is:
  
       NL_POINT    O1, O2;
       NL_VECTOR   X1, Y1, Z1, X2, Y2, Z2;
       NL_RMATRIX  rma;
       NL_STACKS   SG;
       ...
       (get O1, X1, ..., Z2);
       ...
       N_InitRealMatrix(&rma);
       N_CreateTransformMatrixFromAxes(O1,X1,Y1,Z1,O2,X2,Y2,Z2,&rma,&SG);

     If rma is initialized to the NULL matrix, memory  is allocated in
     the  routine. If not, it is  assumed that  memory  allocation has 
     been done. However, the routine checks for  the proper  amount by
     looking at the matrix indexes.


   ACCESS:
   
     O1,X1,Y1,Z1 , input  ,  Given orthonormal coordinate frame (O1 is
                             the  origin  and <X1,Y1,Z1>  are the unit
                             axis vectors)
     O2,X2,Y2,Z2 , input  ,  New orthonormal  coordinate  frame (O2 is 
                             the new origin and <X2,Y2,Z2> are the new
                             unit axis directions)
     rma         , output ,  Transformation matrix
     SG          , input  ,  rma's stack



   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateTransformMatrixFromAxes( NL_POINT O1, NL_VECTOR X1, NL_VECTOR Y1, NL_VECTOR Z1, NL_POINT O2, NL_VECTOR X2, NL_VECTOR Y2, NL_VECTOR Z2, NL_RMATRIX *rma, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateTransformMatrixFromAxes");

    NL_FLAG error = NL_NO;

    NL_REAL ** R1, ** RI, ** T1, ** T2, x, y, z;

    NL_RMATRIX rm1, rmi, rt1, rt2, rmw;

    NL_STACKS SL;

    /* Initialize NURBS */

    N_InitNurbs( &SL );

    /* Check and allocate memory */

    error = N_CheckMemRealMatrix( rma, 3, 3, NL_MT_FULL, 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* allocate memory for rm1 elements */
    error = N_SetRealMatrix( &rm1, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* allocate memory for rmi elements */
    error = N_SetRealMatrix( &rmi, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* allocate memory for rt1 elements */
    error = N_SetRealMatrix( &rt1, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* allocate memory for rt2 elements */
    error = N_SetRealMatrix( &rt2, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* allocate memory for rmw elements */
    error = N_SetRealMatrix( &rmw, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* locals: pointers to element arrays */
    N_GetRealMatrixPtr( &rm1, &R1 );
    N_GetRealMatrixPtr( &rmi, &RI );
    N_GetRealMatrixPtr( &rt1, &T1 );
    N_GetRealMatrixPtr( &rt2, &T2 );

    /* Build matrices */

    /* copy X1, Y1, Z1 values into rotation matrix R1 (rm1) (inverse rotation) */
    N_PtToXYZ( X1, &R1[0][0], &R1[0][1], &R1[0][2] );
    R1[0][3] = 0.0;
    N_PtToXYZ( Y1, &R1[1][0], &R1[1][1], &R1[1][2] );
    R1[1][3] = 0.0;
    N_PtToXYZ( Z1, &R1[2][0], &R1[2][1], &R1[2][2] );
    R1[2][3] = 0.0;
    R1[3][0] = 0.0;
    R1[3][1] = 0.0;
    R1[3][2] = 0.0;
    R1[3][3] = 1.0;

    /* copy X2, Y2, Z2 values into rotation matrix RI (rmi) (forward rotation) */
    N_PtToXYZ( X2, &RI[0][0], &RI[1][0], &RI[2][0] );
    RI[0][3] = 0.0;
    N_PtToXYZ( Y2, &RI[0][1], &RI[1][1], &RI[2][1] );
    RI[1][3] = 0.0;
    N_PtToXYZ( Z2, &RI[0][2], &RI[1][2], &RI[2][2] );
    RI[2][3] = 0.0;
    RI[3][0] = 0.0;
    RI[3][1] = 0.0;
    RI[3][2] = 0.0;
    RI[3][3] = 1.0;

    /* copy -O1 into transform matrix T1 (rt1)  (inverse translation) */
    N_PtToXYZ( O1, &x, &y, &z );
    T1[0][0] = 1.0;
    T1[0][1] = 0.0;
    T1[0][2] = 0.0;
    T1[0][3] = -x;
    T1[1][0] = 0.0;
    T1[1][1] = 1.0;
    T1[1][2] = 0.0;
    T1[1][3] = -y;
    T1[2][0] = 0.0;
    T1[2][1] = 0.0;
    T1[2][2] = 1.0;
    T1[2][3] = -z;
    T1[3][0] = 0.0;
    T1[3][1] = 0.0;
    T1[3][2] = 0.0;
    T1[3][3] = 1.0;

    /* copy +O2 into transform matrix T2 (rt2) (forward translation) */
    N_PtToXYZ( O2, &x, &y, &z );
    T2[0][0] = 1.0;
    T2[0][1] = 0.0;
    T2[0][2] = 0.0;
    T2[0][3] = x;
    T2[1][0] = 0.0;
    T2[1][1] = 1.0;
    T2[1][2] = 0.0;
    T2[1][3] = y;
    T2[2][0] = 0.0;
    T2[2][1] = 0.0;
    T2[2][2] = 1.0;
    T2[2][3] = z;
    T2[3][0] = 0.0;
    T2[3][1] = 0.0;
    T2[3][2] = 0.0;
    T2[3][3] = 1.0;

    /* Compute matrix products */

    /* let rmw = rt2 * rmi */
    error = N_RealMatrixMultiply( &rt2, &rmi, &rmw, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* let rt2 = rmw * rm1 */
    error = N_RealMatrixMultiply( &rmw, &rm1, &rt2, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* let rma = rt2 * rt1 = rt2 * rmi * rm1 * rt1                                           */
    /*     rma := inverse_translate => inverse_rotate => forward_rotate => forward_translate */
    error = N_RealMatrixMultiply( &rt2, &rt1, rma, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateTransformMatrixFromAxes */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes a translation matrix that maps one 
     point  into another. The matrix  multiplication is performed from 
     the left, i.e.

               |O1.x|   |O2.x|
           [A] |O1.y| = |O2.y| 
               |O1.z|   |O2.z|
               | 1  |   | 1  |

     where  A  is  a 4x4 matrix, and O1 and O2 are the given points. A 
     typical calling example is:
  
       NL_POINT    O1, O2;
       NL_RMATRIX  rma;
       NL_STACKS   SG;
       ...
       (get O1 and O2);
       ...
       N_InitRealMatrix(&rma);
       N_CreateTranslationMatrixFromPts(O1,O2,&rma,&SG);

     If rma is initialized to the NULL matrix, memory  is allocated in
     the  routine. If not, it is  assumed that  memory  allocation has 
     been done. However, the routine checks for  the proper  amount by
     looking at the matrix indexes.


   ACCESS:
   
     O1  , input  ,  Point to be transformed
     O2  , input  ,  Map of O1
     rma , output ,  Transformation matrix
     SG  , input  ,  rma's stack



   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateTranslationMatrixFromPts( NL_POINT O1, NL_POINT O2, NL_RMATRIX *rma, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateTranslationMatrixFromPts");

    NL_FLAG error = NL_NO;

    NL_REAL ** T, x1, y1, z1, x2, y2, z2;

    /* Check or allocate memory */

    error = N_CheckMemRealMatrix( rma, 3, 3, NL_MT_FULL, 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rma, &T );

    /* Build matrix */

    N_PtToXYZ( O1, &x1, &y1, &z1 );
    N_PtToXYZ( O2, &x2, &y2, &z2 );

    T[0][0] = 1.0;
    T[0][1] = 0.0;
    T[0][2] = 0.0;
    T[0][3] = x2 - x1;
    T[1][0] = 0.0;
    T[1][1] = 1.0;
    T[1][2] = 0.0;
    T[1][3] = y2 - y1;
    T[2][0] = 0.0;
    T[2][1] = 0.0;
    T[2][2] = 1.0;
    T[2][3] = z2 - z1;
    T[3][0] = 0.0;
    T[3][1] = 0.0;
    T[3][2] = 0.0;
    T[3][3] = 1.0;

    /* Exit */

    EXIT:

    return (error);
} /* end N_CreateTranslationMatrixFromPts */

/*******************************************************************//**


   DESCRIPTION:

     Given two  2-D  orientations  <P,V> and  <Q,W>, where  P and  Q are 
     position  vectors  and V  and W  are UNIT  direction  vectors. This 
     routine computes the  unique transformation  matrix that maps <P,V>
     into <Q,W>. The matrix  multiplication is performed from the left:

               |P.x|   |Q.x|
           [A] |P.y| = |Q.y| 
               |P.z|   |Q.z|
               | 1 |   | 1 |

     where  A  is a  4x4  matrix, and  P and  Q are the given  points. A 
     typical calling example is:
  
       NL_POINT    P, Q;
       NL_VECTOR   V, W;
       NL_RMATRIX  rma;
       NL_STACKS   SG;
       ...
       (get P, Q, V and W);
       ...
       N_InitRealMatrix(&rma);
       N_CreateTransformMatrixFromVectors(P,V,Q,W,&rma,&SG);

     If  rma is initialized to NULL, memory is allocated in the routine. 
     If  not, it is  assumed  that  memory  allocation  has  been  done. 
     However, the routine checks for the proper amount by looking at the 
     matrix indexes.


   ACCESS:
   
     <P,V> , input  ,  Point and UNIT vector to be transformed
     <Q,W> , input  ,  Image of <P,V>
     rma   , output ,  4x4 transformation matrix
     SG    , input  ,  rma's stack



   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateTransformMatrixFromVectors( NL_POINT P, NL_VECTOR V, NL_POINT Q, NL_VECTOR W, NL_RMATRIX *rma, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateTransformMatrixFromVectors");

    NL_FLAG error = NL_NO;

    NL_REAL ** T, x1, y1, x2, y2, z, vx, vy, wx, wy, a, b, c, d;

    /* Check or allocate memory */

    error = N_CheckMemRealMatrix( rma, 3, 3, NL_MT_FULL, 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( rma, &T );

    /* Build matrix */

    N_PtToXYZ( P, &x1, &y1, &z );
    N_PtToXYZ( Q, &x2, &y2, &z );
    N_PtToXYZ( V, &vx, &vy, &z );
    N_PtToXYZ( W, &wx, &wy, &z );

    a = vx * wx + vy * wy;
    b = vy * wx - vx * wy;
    c = -a * x1 - b * y1 + x2;
    d = b * x1 - a * y1 + y2;

    T[0][0] = a;
    T[0][1] = b;
    T[0][2] = 0.0;
    T[0][3] = c;
    T[1][0] = -b;
    T[1][1] = a;
    T[1][2] = 0.0;
    T[1][3] = d;
    T[2][0] = 0.0;
    T[2][1] = 0.0;
    T[2][2] = 0.0;
    T[2][3] = 0.0;
    T[3][0] = 0.0;
    T[3][1] = 0.0;
    T[3][2] = 0.0;
    T[3][3] = 1.0;

    /* Exit */

    EXIT:

    return (error);
} /* end N_CreateTransformMatrixFromVectors */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the image of a given point under the
     following axial transformations:

       NL_PINCH: {x|y|z} coordinate is scaled according to s({x|y|z})
         NL_PRIVATE  NL_STRING  rname = "N_TransformPtWithShapeFuncAndScale");
NL_TAPER: P is scaled according to s({x|y|z})
       NL_TWIST: P is rotated according to s({x|y|z})
       NL_SHEAR: {x|y|z} coordinate is translated according to s({x|y|z})

     where  {x|y|z} means  x or y or z  coordinate, and  s({x|y|z}) is a 
     function of  either  x or  y  or z. Also, if  a point  P is  scaled 
     according to, say, s(y), then  only its x and z coordinates change.
     The scaling  function is  given as a  B-spline  function. A typical 
     calling example is:
  
       NL_POINT  P;
       NL_REAL   a;
       NL_CFUN   cfn;
       ...
       (get P, cfn and a);
       ...
       N_TransformPtWithShapeFuncAndScale(P,&cfn,a,NL_TAPER,NL_XDIR,NL_YCRD);

     The  transformation is performed  in-place, i.e. the original point 
     is destroyed.


   ACCESS:
   
     P   , in/out ,  Point to be transformed
     cfn , input  ,  Shape function
     a   , input  ,  Scaling factor (amplitude)
     tra , input  ,  Flag:
                       NL_PINCH: point is pinched
                       NL_TAPER: point is tapered
                       NL_TWIST: point is twisted
                       NL_SHEAR: point is sheared
     dir , input  ,  Flag:
                       NL_XDIR: shape in the x-direction
                       NL_YDIR: shape in the y-direction
                       NL_ZDIR: shape in the z-direction
     cor , input  ,  Flag:
                       NL_XCRD: change x coordinate
                       NL_YCRD: change y coordinate
                       NL_ZCRD: change z coordinate
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TransformPtWithShapeFuncAndScale( NL_POINT *P, NL_CFUN *cfn, NL_REAL a, NL_FLAG tra, NL_FLAG dir, NL_FLAG cor )
{
    NL_PRIVATE NL_STRING rname = _T("N_TransformPtWithShapeFuncAndScale");

    NL_FLAG error = NL_NO;

    NL_REAL x, y, z, f, alf, w;

    /* Extract coordinates and check flags */

    N_PtToXYZ( *P, &x, &y, &z );

    switch( dir )
    {
        case NL_XDIR:

            error = N_CFuncEval( cfn, x, NL_LEFT, &f );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_YDIR:

            error = N_CFuncEval( cfn, y, NL_LEFT, &f );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        case NL_ZDIR:

            error = N_CFuncEval( cfn, z, NL_LEFT, &f );

            if( error EQ NL_YES )
                NL_OUT;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    switch( cor )
    {
        case NL_XCRD:
            break;

        case NL_YCRD:
            break;

        case NL_ZCRD:
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Transform point */

    switch( tra )
    {
        case NL_PINCH:
            switch( dir )
            {
                case NL_XDIR:
                    switch( cor )
                    {
                        case NL_YCRD:
                            y *= a * f;
                            break;

                        case NL_ZCRD:
                            z *= a * f;
                            break;
                    }
                    break;

                case NL_YDIR:
                    switch( cor )
                    {
                        case NL_XCRD:
                            x *= a * f;
                            break;

                        case NL_ZCRD:
                            z *= a * f;
                            break;
                    }
                    break;

                case NL_ZDIR:
                    switch( cor )
                    {
                        case NL_XCRD:
                            x *= a * f;
                            break;

                        case NL_YCRD:
                            y *= a * f;
                            break;
                    }
                    break;
            }
            break;

        case NL_TAPER:
            switch( dir )
            {
                case NL_XDIR:

                    y *= a * f;
                    z *= a * f;
                    break;

                case NL_YDIR:

                    x *= a * f;
                    z *= a * f;
                    break;

                case NL_ZDIR:

                    x *= a * f;
                    y *= a * f;
                    break;
            }
            break;

        case NL_TWIST:

            alf = NL_PI * a * f;

            switch( dir )
            {
                case NL_XDIR:
                    w = y;
                    y = cos( alf ) * w - sin( alf ) * z;
                    z = sin( alf ) * w + cos( alf ) * z;
                    break;

                case NL_YDIR:

                    w = x;
                    x = cos( alf ) * w + sin( alf ) * z;
                    z = -sin( alf ) * w + cos( alf ) * z;
                    break;

                case NL_ZDIR:

                    w = x;
                    x = cos( alf ) * w - sin( alf ) * y;
                    y = sin( alf ) * w + cos( alf ) * y;
                    break;
            }
            break;

        case NL_SHEAR:
            switch( dir )
            {
                case NL_XDIR:
                    switch( cor )
                    {
                        case NL_YCRD:
                            y += a * f;
                            break;

                        case NL_ZCRD:
                            z += a * f;
                            break;
                    }
                    break;

                case NL_YDIR:
                    switch( cor )
                    {
                        case NL_XCRD:
                            x += a * f;
                            break;

                        case NL_ZCRD:
                            z += a * f;
                            break;
                    }
                    break;

                case NL_ZDIR:
                    switch( cor )
                    {
                        case NL_XCRD:
                            x += a * f;
                            break;

                        case NL_YCRD:
                            y += a * f;
                            break;
                    }
                    break;
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    N_PtFromXYZ( x, y, z, P );

    /* Exit */

    EXIT:

    return (error);
} /* end N_TransformPtWithShapeFuncAndScale */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes a  transformation matrix that takes
     a general  direction  vector  into one of the  principal axes. The
     transformation  is the  concatenation of two  rotations. A typical 
     calling example is:
  
       NL_VECTOR   V;
       NL_RMATRIX  rma;
       NL_STACKS   SG;
       ...
       (get V);
       ...
       N_InitRealMatrix(&rma);
       N_CreateTransformMatrixFromVector(V,NL_XDIR,&rma,&SG);

     If rma is initialized to the NULL matrix,  memory is  allocated in
     the routine. If not, it is assumed that memory allocation has been 
     done. However, the routine checks for the proper amount by looking  
     at the matrix indexes (which are assumed to be set correctly).  


   ACCESS:
   
     V    , input  ,  Direction  vector to be  transformed into the x-, 
                      y- or z-axis
     afl  , input  ,  Flag:
                        NL_XDIR: transform into the x-axis
                        NL_YDIR: transform into the y-axis
                        NL_ZDIR: transform into the z-axis
     rma  , output ,  Transformation matrix
     SG   , input  ,  rma's stack



   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateTransformMatrixFromVector( NL_VECTOR V, NL_FLAG afl, NL_RMATRIX *rma, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateTransformMatrixFromVector");

    NL_FLAG error = NL_NO;

    NL_REAL ** R1, ** R2, cosal, sinal, cosbe, sinbe, alf, bet, x, y, z, mag;

    NL_VECTOR U;

    NL_RMATRIX rm1, rm2;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* See if memory is needed */

    error = N_CheckMemRealMatrix( rma, 3, 3, NL_MT_FULL, 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SetRealMatrix( &rm1, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SetRealMatrix( &rm2, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &rm1, &R1 );
    N_GetRealMatrixPtr( &rm2, &R2 );

    /* Get angular coordinates */

    N_PtToXYZ( V, &x, &y, &z );
    N_VectorCreate( x, y, 0.0, &U );
    N_VectorMagnitude( U, &mag );

    error = N_VectorsAngle( NL_UNITZ, V, &alf );

    if( error EQ NL_YES )
        NL_OUT;

    bet = 0.0;

    if( mag GT NL_MTOL )
    {
        error = N_VectorsAngle( NL_UNITX, U, &bet );

        if( error EQ NL_YES )
            NL_OUT;

        N_VectorDot( NL_UNITY, U, &z );

        if( z LT 0.0 )
            bet = 360.0 - bet;
    }

    alf *= NL_RAD;
    bet *= NL_RAD;

    /* Get the rotation matrixes */

    cosal = cos( alf );
    sinal = sin( alf );
    cosbe = cos( bet );
    sinbe = sin( bet );

    switch( afl )
    {
        case NL_XDIR:

            R1[0][0] = cosbe;
            R1[0][1] = sinbe;
            R1[0][2] = 0.0;
            R1[0][3] = 0.0;
            R1[1][0] = -sinbe;
            R1[1][1] = cosbe;
            R1[1][2] = 0.0;
            R1[1][3] = 0.0;
            R1[2][0] = 0.0;
            R1[2][1] = 0.0;
            R1[2][2] = 1.0;
            R1[2][3] = 0.0;
            R1[3][0] = 0.0;
            R1[3][1] = 0.0;
            R1[3][2] = 0.0;
            R1[3][3] = 1.0;

            R2[0][0] = sinal;
            R2[0][1] = 0.0;
            R2[0][2] = cosal;
            R2[0][3] = 0.0;
            R2[1][0] = 0.0;
            ;
            R2[1][1] = 1.0;
            R2[1][2] = 0.0;
            R2[1][3] = 0.0;
            R2[2][0] = -cosal;
            R2[2][1] = 0.0;
            R2[2][2] = sinal;
            R2[2][3] = 0.0;
            R2[3][0] = 0.0;
            R2[3][1] = 0.0;
            R2[3][2] = 0.0;
            R2[3][3] = 1.0;

            break;

        case NL_YDIR:

            R1[0][0] = sinbe;
            R1[0][1] = -cosbe;
            R1[0][2] = 0.0;
            R1[0][3] = 0.0;
            R1[1][0] = cosbe;
            R1[1][1] = sinbe;
            R1[1][2] = 0.0;
            R1[1][3] = 0.0;
            R1[2][0] = 0.0;
            R1[2][1] = 0.0;
            R1[2][2] = 1.0;
            R1[2][3] = 0.0;
            R1[3][0] = 0.0;
            R1[3][1] = 0.0;
            R1[3][2] = 0.0;
            R1[3][3] = 1.0;

            R2[0][0] = 1.0;
            R2[0][1] = 0.0;
            R2[0][2] = 0.0;
            R2[0][3] = 0.0;
            R2[1][0] = 0.0;
            R2[1][1] = sinal;
            R2[1][2] = cosal;
            R2[1][3] = 0.0;
            R2[2][0] = 0.0;
            R2[2][1] = -cosal;
            R2[2][2] = sinal;
            R2[2][3] = 0.0;
            R2[3][0] = 0.0;
            R2[3][1] = 0.0;
            R2[3][2] = 0.0;
            R2[3][3] = 1.0;

            break;

        case NL_ZDIR:

            R1[0][0] = cosbe;
            R1[0][1] = sinbe;
            R1[0][2] = 0.0;
            R1[0][3] = 0.0;
            R1[1][0] = -sinbe;
            R1[1][1] = cosbe;
            R1[1][2] = 0.0;
            R1[1][3] = 0.0;
            R1[2][0] = 0.0;
            R1[2][1] = 0.0;
            R1[2][2] = 1.0;
            R1[2][3] = 0.0;
            R1[3][0] = 0.0;
            R1[3][1] = 0.0;
            R1[3][2] = 0.0;
            R1[3][3] = 1.0;

            R2[0][0] = cosal;
            R2[0][1] = 0.0;
            R2[0][2] = -sinal;
            R2[0][3] = 0.0;
            R2[1][0] = 0.0;
            ;
            R2[1][1] = 1.0;
            R2[1][2] = 0.0;
            R2[1][3] = 0.0;
            R2[2][0] = sinal;
            R2[2][1] = 0.0;
            R2[2][2] = cosal;
            R2[2][3] = 0.0;
            R2[3][0] = 0.0;
            R2[3][1] = 0.0;
            R2[3][2] = 0.0;
            R2[3][3] = 1.0;

            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Get transformation matrix */

    error = N_RealMatrixMultiply( &rm2, &rm1, rma, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateTransformMatrixFromVector */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry routine tests if a  point lies on the left or the 
     right of a  given line. The test  is meaningful only in 2-D. The
     sides are defined by looking along the direction vector defining
     the line. A typical calling example is:

       NL_LINESEG  lsg;
       NL_POINT    Q;
       ...
       switch( N_PtGetSideOfLine(&lsg,Q) )
       {
         case NL_LEFT : ...
         case NL_RIGHT: ...
       }


   ACCESS:
   
     lsg  , input  ,  (Directed) line
     Q    , input  ,  Point 


   RETURN CODES:

     NL_LEFT : Q lies on the left or on
     NL_RIGHT: Q lies on the right

   ***********************************************************************/

NL_FLAG N_PtGetSideOfLine( NL_LINESEG *lsg, NL_POINT Q )
{

    NL_FLAG b;

    NL_POINT P;

    NL_VECTOR V, D, C;

    /* Do side test */

    N_LineGetData( lsg, &P, &V, &b );
    N_VectorDiff( Q, P, &D );
    N_VectorCross( V, D, &C );

    if( C.z GE 0.0 )
        return NL_LEFT;
    else
        return NL_RIGHT;
} /* end N_PtGetSideOfLine */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine tests if a  point lies  on the left or the 
     right of a  given plane. The  right side of  the plane is facing
     toward the viewer (the normal vector is directed at the viewer),
     whereas the left side is facing away  from the viewer. A typical 
     calling example is:

       NL_PLANE  pln;
       NL_POINT  Q;
       ...
       switch( N_PtGetSideOfPlane(&pln,Q) )
       {
         case NL_LEFT : ...
         case NL_RIGHT: ...
       }


   ACCESS:
   
     pln  , input  ,  Plane
     Q    , input  ,  Point 


   RETURN CODES:

     NL_LEFT : Q lies on the left 
     NL_RIGHT: Q lies on the right or on

   ***********************************************************************/

NL_FLAG N_PtGetSideOfPlane( NL_PLANE *pln, NL_POINT Q )
{

    NL_POINT P;

    NL_VECTOR N, D;

    NL_REAL dot;

    /* Do side test */

    N_PlaneGetData( pln, &P, &N );
    N_VectorDiff( Q, P, &D );
    N_VectorDot( N, D, &dot );

    if( dot GE 0.0 )
        return NL_RIGHT;
    else
        return NL_LEFT;
} /* end N_PtGetSideOfPlane */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry routine  converts a  rectangle  object into two 
     intervals. A typical calling example is:

       NL_RECTANGLE  R;
       NL_INTERVAL   I, J;
       ...
       (get R);
       ...
       N_RectToTwoIntervals(&R,&I,&J);

     The intervals I and J must be declared in the calling routine.


   ACCESS:
   
     R , input  ,  Rectangle
     I , output ,  Interval [left,right]
     J , output ,  Interval [bottom,top]


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_RectToTwoIntervals( NL_RECTANGLE *R, NL_INTERVAL *I, NL_INTERVAL *J )
{

    I->ul = R->ul;
    I->ur = R->ur;

    J->ul = R->vb;
    J->ur = R->vt;
} /* end N_RectToTwoIntervals */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the best-fit plane to a given set of
     3D points.  The coefficients of  the normalized plane equation  are 
     returned, i.e.

         a*x + b*y + c*z + d = 0

     where (a,b,c) are the  coordinates of the unit normal to the plane.
     A typical calling example is:
  
       NL_POINT  *P;
       NL_INDEX  n;
       NL_REAL   a, b, c, d;
       ...
       (get points P);
       ...
       N_FitPlanePts(P,n,&a,&b,&c,&d);


   ACCESS:
   
     P        , input  ,  Points in 3-D
     n        , input  ,  Highest index in P
     a,b,c,d  , output ,  Coefficients of normalized plane's equation


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitPlanePts( NL_POINT *P, NL_INDEX n, NL_REAL *a, NL_REAL *b, NL_REAL *c, NL_REAL *d )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_FitPlanePts"); */

    NL_FLAG error = NL_NO;

    NL_POINT C ;

    NL_VECTOR N ;

    NL_REAL era, erm ;

    NL_STACKS SL;

    /* Initialize system */

    N_InitNurbs( &SL );

    /* get best fit Plane(C,N) */
    error = N_PlaneFit3dPts(P, n, &C, &N, &era, &erm) ;

    if( error EQ NL_YES )
        NL_OUT;

    /* set output */
    *a = N.x ; 
    *b = N.y ;
    *c = N.z ; 
    *d = - (*a * C.x + *b * C.y + *c * C.z) ;

#ifdef OBSOLETE

    NL_FLAG dir, error = NL_NO;

    NL_INDEX i;

    NL_REAL ** m, x, y, z, aa, bb, cc, fac, xmin, xmax, ymin, ymax, zmin, zmax, xd, yd, zd, ** V, W[3], *R, X[3];

    /* Get form of plane equation */

    xmin = xmax = P[0].x;
    ymin = ymax = P[0].y;
    zmin = zmax = P[0].z;

    for ( i = 1; i <= n; i++ )
    {
        if( P[i].x LT xmin )
            xmin = P[i].x;

        if( P[i].x GT xmax )
            xmax = P[i].x;

        if( P[i].y LT ymin )
            ymin = P[i].y;

        if( P[i].y GT ymax )
            ymax = P[i].y;

        if( P[i].z LT zmin )
            zmin = P[i].z;

        if( P[i].z GT zmax )
            zmax = P[i].z;
    }

    xd = fabs( xmax - xmin );
    yd = fabs( ymax - ymin );
    zd = fabs( zmax - zmin );

    if( xd LE yd AND xd LE zd )
        dir = 1;

    else if( yd LE xd AND yd LE zd )
        dir = 2;

    else
        dir = 3;

    /* Get initial matrix */

    m = N_AllocReal2dArray( n, 2, &SL );

    if( m EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( P[i], &x, &y, &z );

        m[i][0] = 1.0;

        if( dir EQ 1 )
        {
            m[i][1] = y;
            m[i][2] = z;
        }
        else if( dir EQ 2 )
        {
            m[i][1] = x;
            m[i][2] = z;
        }
        else
        {
            m[i][1] = x;
            m[i][2] = y;
        }
    }

    /* Now solve with Single Value Decomposition */

    V = N_AllocReal2dArray( 2, 2, &SL );

    if( V EQ NULL )
        NL_QUIT;

    error = N_RealMatrixSingleValueDecompose( m, W, V, n, 2 );

    if( error EQ NL_YES )
        NL_OUT;

    R = N_AllocReal1dArray( n, &SL );

    if( R EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        if( dir EQ 1 )
            R[i] = P[i].x;

        else if( dir EQ 2 )
            R[i] = P[i].y;

        else
            R[i] = P[i].z;
    }

    error = N_SingleValueDecomposeSolve( m, W, V, n, 2, (NL_VOID *)R, NL_RVALUE, NL_YES, (NL_VOID *)X );

    if( error EQ NL_YES )
        NL_OUT;

    aa = X[0];
    bb = X[1];
    cc = X[2];

    /* Compute coefficients */

    fac = sqrt( bb * bb + cc * cc + 1.0 );

    if( N_FloatOpIsBad( 1.0, fac, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );
    fac = 1.0 / fac;
    *d = fac * aa;

    if( dir EQ 1 )
    {
        *a = -fac;
        *b = fac * bb;
        *c = fac * cc;
    }
    else if( dir EQ 2 )
    {
        *a = fac * bb;
        *b = -fac;
        *c = fac * cc;
    }
    else
    {
        *a = fac * bb;
        *b = fac * cc;
        *c = -fac;
    }

#endif /* OBSOLETE */

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_FitPlanePts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes a plane which fits a  given  set  of
     sequencially ordered 3D points,  i.e. a 3D polyline.  The center of
     gravity and a least squares normal vector are computed. The coefficients
     of the  normalized  plane  equation  can  be  obtained  by  calling 
     N_PlanePtNormalToImplicit.  This  function returns a plane regardless of whether the
     points lie in a unique plane or not;  i.e.  it  is  in some sense a 
     "best fit" plane to the set of points.  A  typical  calling example 
     is:
  
       NL_POINT   *P;
       NL_INDEX   n;
       NL_POINT   CG;
       NL_VECTOR  N;
       ...
       (get points P);
       ...
       N_PlaneFitOrdered3dPts(P,n,&CG,&N);


   ACCESS:
   
     P   , input  ,  Points in 3-D
     n   , input  ,  Highest index in P (n > 1 must hold)
     CG  , output ,  Center of gravity
     N   , output ,  Average normal vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PlaneFitOrdered3dPts( NL_POINT *P, NL_INDEX n, NL_POINT *CG, NL_VECTOR *N )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_PlaneFitOrdered3dPts"); */

    NL_FLAG error = NL_NO;

    NL_REAL era, erm ;

    NL_STACKS SL;

    /* Initialize system */

    N_InitNurbs( &SL );

    /* get best fit Plane(C,N) */
    error = N_PlaneFit3dPts(P, n, CG, N, &era, &erm) ;

    if( error EQ NL_YES )
        NL_OUT;

#ifdef OBSOLETE

    NL_FLAG error = NL_NO;

    NL_INDEX ii, count1, count2;

    NL_REAL ang, mag;

    NL_POINT PP;

    NL_VECTOR NN1, NN2, V1, V2, VN;

    if( n LE 1 )
        NL_ERROR( NL_INP_ERR );

    /* Walk thru the points computing center of gravity and */
    /* summing the normal vectors computed at each interior */
    /* polyline point from vector cross products.           */

    /* For numerical reasons, "almost collinear" points are */
    /* not used if not necessary.                           */

    count1 = count2 = 0;
    N_CopyPt( NL_ZERO, &PP );
    N_CopyPt( NL_ZERO, &NN1 );
    N_CopyPt( NL_ZERO, &NN2 );

    for ( ii = 0; ii <= n; ii++ )
    {
        N_Sum2Pts( PP, P[ii], &PP );

        if( ii GT 0 AND ii LT n )
        {
            N_VectorDir( P[ii - 1], P[ii], &V1 );
            N_VectorDir( P[ii], P[ii + 1], &V2 );

            error = N_VectorsAngle( V1, V2, &ang );

            N_VectorCross( V1, V2, &VN );
            error = N_VectorNormalize( VN, &VN, &mag );

            if( error EQ NL_YES OR ang LT 1.0 OR ang GT 179. )
            {
                if( error EQ NL_NO )
                {
                    N_VectorSum( NN2, VN, &NN2 );
                    count2 += 1;
                }
            }
            else
            {
                N_VectorSum( NN1, VN, &NN1 );
                count1 += 1;
            }
        }
    }

    if( count1 + count2 EQ 0 )
        NL_ERROR( NL_INP_ERR );

    N_ScalePt( 1.0 / (n + 1), PP, CG );

    if( count1 GT 0 )
        N_VectorScale( NN1, 1.0 / count1, N );
    else
        N_VectorScale( NN2, 1.0 / count2, N );

#endif /* OBSOLETE */

    /* Exit */

    EXIT:

    return (error);
} /* end N_PlaneFitOrdered3dPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry  routine  computes a  best fitting plane to a set of 
     points in 3-D. A point (C) and a unit normal vector (N) are returned. 
     The method  first finds  the center of  gravity (C) of the points, and 
     then finds the eigenvector (N) associated with the smallest eigenvalue
     of the matrix, M = Atranspose * A, Where Ai = Pi - C. The 
     average as well as the maximum errors are also returned. A typical 
     calling example is:
       
       NL_POINT  *P, C;
       NL_VECTOR  N;
       NL_INDEX   n;
       NL_REAL    era, erm;
       ...
       (get points P);
       ...
       N_PlaneFit3dPts(P,n,&C,&N,&era,&erm);


   ACCESS:
   
     P    , input  ,  Points
     n    , input  ,  Highest index in P
     C    , output ,  Point on the plane
     N    , output ,  Unit normal vector of plane
     era  , output ,  Average error
     erm  , output ,  Maximum error


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PlaneFit3dPts( NL_POINT *P, NL_INDEX n, NL_POINT *C, NL_VECTOR *N, NL_REAL *era, NL_REAL *erm )
{
    NL_PRIVATE NL_STRING rname = _T("N_PlaneFit3dPts");

    NL_FLAG error = NL_NO;

    NL_INDEX i, im;

    NL_REAL ** A, ** B, v[3], xc, yc, zc, x, y, z, d, da, dm, inv, vm;

    NL_POINT CC, CM;

    NL_VECTOR NN;

    NL_RMATRIX rma, rmb, rmat, rmata;

    NL_PLANE pln;

    NL_STACKS SL;

    /* Initialize */

    N_InitNurbs( &SL );

    /* Get various entities */

    inv = ((NL_REAL)n + 1.0);

    if( N_FloatOpIsBad( 1.0, inv, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );
    inv = 1.0 / inv;

    /* Compute center of gravity */

    N_VectorCreate( 0.0, 0.0, 0.0, &CM );

    for ( i = 0; i <= n; i++ )
        N_VectorSum( P[i], CM, &CM );
    N_VectorScale( CM, inv, &CC );

    N_PtToXYZ( CC, &xc, &yc, &zc );

    /* Get matrix */

    error = N_SetRealMatrix( &rma, n, 2, NL_MT_FULL, 0, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &rma, &A );

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( P[i], &x, &y, &z );

        A[i][0] = x - xc;
        A[i][1] = y - yc;
        A[i][2] = z - zc;
    }

    /* Get (A^T)A */

    N_InitRealMatrix( &rmat );
    error = N_RealMatrixTranspose( &rma, &rmat, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &rmata );
    error = N_RealMatrixMultiply( &rmat, &rma, &rmata, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Solve eigenvector problem */

    N_InitRealMatrix( &rmb );
    error = N_RealMatrixEigenValuesVectors( &rmata, v, &rmb, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &rmb, &B );

    vm = NL_BIGD;
    im = 0;

    for ( i = 0; i <= 2; i++ )
    {
        if( v[i]LT vm )
        {
            vm = v[i];
            im = i;
        }
    }

    N_VectorCreate( B[0][im], B[1][im], B[2][im], &NN );
    N_CreatePlanePtNormal( &pln, CC, NN );

    /* Get the errors */

    da = 0.0;
    dm = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        error = N_DistPtPlane( pln, P[i], &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT dm )
            dm = d;
        da += d;
    }

    da *= inv;

    /* Get the output */

    N_VectorCopy( CC, C );
    N_VectorCopy( NN, N );

    *era = da;
    *erm = dm;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_PlaneFit3dPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine reflects a control  point  through  a  given
     Euclidean plane. That is, the Euclidean point corresponding to the
     control point is reflected through the plane,  and  the  resulting
     point is weighted with the same weight.  A typical calling example 
     is:

       NL_CPOINT   Pw, Qw;
       NL_PLANE    pln;
       ...
       (get Pw and define pln);
       ...
       N_CptReflect(Pw,pln,&Qw);


   ACCESS:
   
     Pw  , input  ,  Homogeneous point to be reflected
     pln , input  ,  The plane
     Qw  , output ,  Reflection of Pw


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CptReflect
 (NL_CPOINT  Pw,  /* in : Homogeneous point to be reflecte */
  NL_PLANE   pln, /* in : The reflection plane */
  NL_CPOINT *Qw ) /* out: Reflection of Pw */
{

    NL_REAL NOZ_TOL = 1.0e-6;

    NL_FLAG error = NL_NO;

    NL_REAL x, y, z, w, wz, vz, dd;

    NL_POINT P, Q, R;

    /* Reflect point */

    N_CPtToPtEuclid( Pw, &P );
    error = N_ProjectPtPlane( pln, P, &Q );

    if( error EQ NL_YES )
        return (1);
    N_VectorDir( P, Q, &R );
    N_Sum2Pts( Q, R, &Q );

    /* Make control point out of Q */

    N_PtToXYZ( Q, &x, &y, &z );
    N_CPtToWxWyWz( Pw, &dd, &dd, &wz, &w );

    if( w NEQ NL_NOW )
    {
        x *= w;
        y *= w;
        z *= w;
    }

    N_PlaneGetNormal( &pln, &R );
    error = N_VectorNormalize( R, &R, &dd );

    if( error EQ NL_YES )
        return (1);
    N_PtToXYZ( R, &dd, &dd, &vz );

    if( wz EQ NL_NOZ AND fabs( vz )LE NOZ_TOL )
        z = NL_NOZ;

    N_CPtFromWxWyWz( x, y, z, w, Qw );

    return (0);
} /* end N_CptReflect */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine checks if three control points are collinear,
     i.e. if they represent G^1  continuity in  homogeneous or Euclidean
     space. The control points are arranged topologically as Pw-Qw-Rw. A
     typical calling example is:

       NL_CPOINT  Pw, Qw, Rw;
       ...
       (get Pw, Qw and Rw);
       ...
       if( N_CPtsAreColinear(Pw,Qw,Rw,NL_THREED) )  --> points are collinear;


   ACCESS:
   
     Pw   , input  ,  First control point
     Qw   , input  ,  Second control point
     Rw   , input  ,  Third control point
     dim  , input  ,  Flag:
                        NL_THREED: check Euclidean data
                        NL_FOURD : check homogeneous data


   RETURN CODES:

     NL_YES: Points are collinear
     NL_NO : Points are not collinear

   ***********************************************************************/

NL_BOOLEAN N_CPtsAreColinear( NL_CPOINT Pw, NL_CPOINT Qw, NL_CPOINT Rw, NL_FLAG dim )
{

    NL_FLAG error;

    NL_REAL a00, a01, a11, d;

    NL_POINT P, Q, R;

    NL_VECTOR V1, V2;

    /* Check collinearity */

    if( dim EQ NL_THREED )
    {
        N_CPtToPtEuclid( Pw, &P );
        N_CPtToPtEuclid( Qw, &Q );
        N_CPtToPtEuclid( Rw, &R );

        N_DistPtLineSeg( Q, P, R, &d );

        if( d LE 2.0 *NL_MTOL )
            return NL_YES;
        else
            return NL_NO;
    }
    else if( dim EQ NL_FOURD )
    {
        N_CPtToPt( Pw, &P );
        N_CPtToPt( Qw, &Q );
        N_CPtToPt( Rw, &R );

        N_VectorDiff( Q, P, &V1 );
        N_VectorDiff( R, Q, &V2 );

        error = N_VectorNormalizeRef( &V1 );

        if( error EQ NL_YES )
            return NL_NO;

        error = N_VectorNormalizeRef( &V2 );

        if( error EQ NL_YES )
            return NL_NO;

        N_VectorDot( V1, V1, &a00 );  /* gwc: should be 1.0 due to N_VectorNormalizeRef call above */
        N_VectorDot( V1, V2, &a01 );  /* gwc: cos(angle between V1 and V2) */
        N_VectorDot( V2, V2, &a11 );  /* gwc: should be 1.0 due to N_VectorNormalizeRef call above */

        d = a00 * a11 - a01 * a01;    /* gwc: a sloppy way of checking (1 - cos(ang) * cos(ang)) < NL_LTol */

        if( fabs( d )LT NL_ZCTL )     /* gwc: because of the cos**2 term this is not really an angle - changed from NL_LTOL to NL_ZCTL when NL_LTOL was changed */
            return NL_YES;
        else
            return NL_NO;
    }
    else
    {
        return NL_NO;
    }
} /* end N_CPtsAreColinear */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This geometry  routine  computes a  new  weight to  ensure that the
     control  points Aw, Bw and Cw  lie on the same line  in  4-D. Their
     Euclidean images A, B, and C must be collinear. The new  weight for 
     Aw is  computed so  that Aw is on  the line of  <Bw,Cw>. A  typical 
     calling example is:
 
       NL_CPOINT  Aw, Bw, Cw;
       NL_REAL    wa;
       ...
       (get Aw, Bw and Cw);
       ...
       N_CPtCalcWeightColinear(Aw,Bw,Cw,&wa);
 
     Aw, Bw AND Cw ARE ARRANGED TOPOLOGICALLY AS Aw-Bw-Cw!
 
 
   ACCESS:
   
     Aw  , input  ,  First control point
     Bw  , input  ,  Second control point
     Cw  , input  ,  Third control point
     wa  , output ,  New weight of Aw 
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_CPtCalcWeightColinear( NL_CPOINT Aw, NL_CPOINT Bw, NL_CPOINT Cw, NL_REAL *wa )
{
    NL_PRIVATE NL_STRING rname = _T("N_CPtCalcWeightColinear");

    NL_FLAG error;

    NL_REAL a[2][2], b[2], num, den, alf, wl;

    NL_VECTOR P21, V1, V2;

    NL_POINT P1, P2, A, B;

    /* Check collinearity */

    if( NOT N_CPtsAreColinear( Aw, Bw, Cw, NL_THREED ) )
    {
        N_ErrSet( NL_INP_ERR, rname );
        return (1);
    }

    if( N_CPtsAreColinear( Aw, Bw, Cw, NL_FOURD ) )
    {
        N_CPtGetW( Aw, wa );
        return (0);
    }

    /* Get some 3-D vectors */

    N_VectorCopy( NL_ZERO, &P1 );
    N_CPtToPt( Cw, &P2 );
    N_CPtToPtEuclid( Aw, &A );
    N_VectorCopy( A, &V1 );
    N_CPtToPt( Bw, &B );
    N_VectorDiff( B, P2, &V2 );

    /* Compute intersection */

    error = N_VectorNormalizeRef( &V1 );

    if( error EQ NL_YES )
        return (1);

    error = N_VectorNormalizeRef( &V2 );

    if( error EQ NL_YES )
        return (1);

    N_VectorDir( P1, P2, &P21 );

    N_VectorDot( V1, V1, &a[0][0] );   /* gwc: should be 1.0 due to N_VectorNormalizeRef call above */
    N_VectorDot( V1, V2, &a[0][1] );   /* gwc: cos(angle between V1 and V2) */
    N_VectorDot( V2, V2, &a[1][1] );   /* gwc: should be 1.0 due to N_VectorNormalizeRef call above */

    N_VectorDot( V1, P21, &b[0] );     
    N_VectorDot( V2, P21, &b[1] );

    den = a[0][0] * a[1][1] - a[0][1] * a[0][1]; /* gwc: a sloppy way of checking (1 - cos(ang) * cos(ang)) < NL_LTol */

    if( fabs( den )LT NL_ZCTL )    /* gwc: because of the cos**2 term this is not really an angle - changed from NL_LTOL to NL_ZCTL when NL_LTOL was changed */
    {
        N_ErrSet( NL_INP_ERR, rname );
        return (1);
    }

    num = b[0] * a[1][1] - b[1] * a[0][1];

    if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
    {
        N_ErrSet( NL_NUM_ERR, rname );
        return (1);
    }

    alf = num / den;

    N_PtMagnitude( A, &den );

    if( N_FloatOpIsBad( alf, den, NL_DIVISION ) )
    {
        N_ErrSet( NL_NUM_ERR, rname );
        return (1);
    }

    wl = alf / den;

    if( wl LT NL_WMIN )
    {
        N_ErrSet( NL_WEI_ERR, rname );
        return (1);
    }
    else
    {
        *wa = wl;
    }

    return (0);
} /* end N_CPtCalcWeightColinear */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine converts the implicit  equation of a plane to 
     point-normal form, i.e. given

         a*x + b*y + c*z + d = 0

     it converts this equation to <P,N>, where P is a point on the plane
     and N is the UNIT normal.  A typical calling example is:
  
       NL_REAL    a, b, c, d;
       NL_POINT   P;
       NL_VECTOR  N;
       ...
       (get a, b, c and d);
       ...
       N_PlaneImplicitToPtNormal(a,b,c,d,&P,&N);


   ACCESS:
   
     a,b,c,d  , input  ,  Coefficients of plane's equation
     P        , output ,  Point on the plane
     N        , output ,  Unit normal to the plane



   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PlaneImplicitToPtNormal( NL_REAL a, NL_REAL b, NL_REAL c, NL_REAL d, NL_POINT *P, NL_VECTOR *N )
{
    NL_PRIVATE NL_STRING rname = _T("N_PlaneImplicitToPtNormal");

    NL_FLAG error = NL_NO;

    NL_REAL x, y, z, num;

    /* Get unit normal */

    N_PtFromXYZ( a, b, c, N );

    error = N_VectorNormalizeRef( N );

    if( error EQ NL_YES )
        NL_OUT;

    /* Choose a suitable point */

    if( fabs( a )GT fabs( b )AND fabs( a )GT fabs( c ) )
    {
        num = -(b * b + c * c + d);

        if( N_FloatOpIsBad( num, a, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        x = num / a;
        y = b;
        z = c;
    }
    else if( fabs( b )GT fabs( a )AND fabs( b )GT fabs( c ) )
    {
        num = -(a * a + c * c + d);

        if( N_FloatOpIsBad( num, b, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        x = a;
        y = num / b;
        z = c;
    }
    else
    {
        num = -(a * a + b * b + d);

        if( N_FloatOpIsBad( num, c, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        x = a;
        y = b;
        z = num / c;
    }

    N_PtFromXYZ( x, y, z, P );

    /* Exit */

    EXIT:

    return (error);
} /* end N_PlaneImplicitToPtNormal */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine converts the  point-normal definition of a 
     plane to implicit form, i.e. given <P,N>, it computes

         a*x + b*y + c*z + d = 0

     where  (a,b,c,d) are the  coefficients of  the  NORMLIZED  plane
     equation.  A typical calling example is:
  
       NL_POINT   P;
       NL_VECTOR  N;
       NL_REAL    a, b, c, d;
       ...
       (get P and N);
       ...
       N_PlanePtNormalToImplicit(P,N,&a,&b,&c,&d);


   ACCESS:
   
     P        , input  ,  Point on the plane
     N        , input  ,  Normal to the plane
     a,b,c,d  , output ,  Coefficients of normalized plane's equation


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PlanePtNormalToImplicit( NL_POINT P, NL_VECTOR N, NL_REAL *a, NL_REAL *b, NL_REAL *c, NL_REAL *d )
{

    NL_FLAG error = NL_NO;

    NL_REAL dot;

    /* Get coefficients */

    error = N_VectorNormalizeRef( &N );

    if( error EQ NL_YES )
        NL_OUT;

    N_PtToXYZ( N, a, b, c );
    N_VectorDot( P, N, &dot );
    *d = -dot;

    /* Exit */

    EXIT:

    return (error);
} /* end N_PlanePtNormalToImplicit */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the length of a control polygon in
     homogeneous space. A typical calling example is:

       NL_INDEX   n;
       NL_CPOINT  *Pw;
       NL_REAL    lw;
       ... 
       (get array Pw);
       ...
       N_DistCPolygonHomo(Pw,n,&lw);


   ACCESS:
   
     Pw  , input  ,  Control point array
     n   , input  ,  Highest index in Pw
     lw  , output ,  Length of control polygon in 4-D


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistCPolygonHomo( NL_CPOINT *Pw, NL_INDEX n, NL_REAL *lw )
{

    NL_INDEX i;

    NL_REAL dw;

    *lw = 0.0;

    for ( i = 1; i <= n; i++ )
    {
        N_DistCptCptHomo( Pw[i - 1], Pw[i], &dw );
        *lw += dw;
    }
} /* end N_DistCPolygonHomo */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This geometry routine computes the distance between a point P and a
     circular arc. If the  projection of P falls between the sweep angle
     of the  arc, the distance from  the arc is returned. Otherwise  the 
     predefined  NL_UNDEFINED  is  returned. If the  arc is a straight line 
     segment, the distance of P from the line (defined by Ps and Pe)  is
     computed. A typical calling example is:
 
       NL_POINT  P, C, Ps, Pe;
       NL_REAL   r, alf, bet, d;
       ...
       (get P, C, Ps, Pe, r, alf and bet);
       ...
       N_DistPtArc(P,C,r,alf,bet,Ps,Pe,&d);

 
   ACCESS:
   
     P       , input  ,  Given point
     C,r     , input  ,  Center and radius of  circular arc. IF THE  ARC 
                         IS A STRAIGHT LINE, r = NL_INFINITE IS PASSED  IN!
     alf,bet , input  ,  Start and end angles. IT  IS  ASSUMED  THAT THE
                         ANGLES  REPRESENT THE SMALLEST  ARC, I.E.,  THE 
                         ONE THAT SWEEPS LESS THAN 180 DEGREES!
     Ps,Pe   , input  ,  Start and end points of the arc
     d       , output ,  Distance between P and the arc
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_DistPtArc( NL_POINT P, NL_POINT C, NL_REAL r, NL_REAL alf, NL_REAL bet, NL_POINT Ps, NL_POINT Pe, NL_REAL *d )
{

    NL_FLAG error;

    NL_VECTOR R, V;

    NL_REAL ang, dis, dif, ds, de;

    /* See if segment is a line */

    if( r EQ NL_INFINITE )  /* gwc: N_BezCenterRadiusFrom3CPts sets r to NL_INFINITE when given 3 colinear points to examine */
    {
        error = N_DistPerpPtLineSeg( P, Ps, Pe, &dis );

        if( error EQ NL_YES )
            return (1);

        if( dis GE 0.0 )
            *d = dis;
        else
            *d = NL_UNDEFINED;

        return (0);
    }

    /* Check special case */

    N_DistPtPt( P, Ps, &ds );
    N_DistPtPt( P, Pe, &de );
    N_DistPtPt( C, P, &dis );

    dif = fabs( dis - r );

    if( ds LE NL_MTOL OR de LE NL_MTOL )
    {
        *d = dif;
        return (0);
    }

    /* Process circle case now */

    N_VectorCreate( 1.0, 0.0, 0.0, &R );
    N_VectorDiff( P, C, &V );
    N_VectorDirectedAngle( R, V, &ang );

    if( fabs( alf - bet )LE 180.0 )
    {
        if( alf LT bet )
        {
            if( alf LE ang AND ang LE bet )
                *d = dif;
            else
                *d = NL_UNDEFINED;
        }
        else if( alf GT bet )
        {
            if( alf GE ang AND ang GE bet )
                *d = dif;
            else
                *d = NL_UNDEFINED;
        }
        else
        {
            *d = NL_UNDEFINED;
        }
    }
    else if( fabs( alf - bet )GT 180.0 )
    {
        if( alf LT 180.0 AND bet GT 180.0 )
        {
            if( (0.0 LE ang AND ang LE alf)OR( bet LE ang AND ang LE 360.0 ) )
                *d = dif;
            else
                *d = NL_UNDEFINED;
        }
        else if( alf GT 180.0 AND bet LT 180.0 )
        {
            if( (0.0 LE ang AND ang LE bet)OR( alf LE ang AND ang LE 360.0 ) )
                *d = dif;
            else
                *d = NL_UNDEFINED;
        }
        else
        {
            *d = NL_UNDEFINED;
        }
    }
    else
    {
        *d = NL_UNDEFINED;
    }

    return (0);
} /* end N_DistPtArc */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This geometry routine computes the maximum  distance between a line 
     segment and a circular arc. If the  arc is a straight line segment, 
     the maximum distance of the end points from  this line is returned. 
     The distance  between the  arc and the  line segment is  defined as 
     follows:

       d(line_seg,circ_arc) = NL_MAX[d(P_i,C_i)]

     where P_i is any  point on the  line segment  contained in  the arc
     wedge, C_i is any point on  the circular arc, and P_iC_i are radial
     lines. A typical  calling example is:
 
       NL_POINT  P, Q, C, Ps, Pe;
       NL_REAL   r, alf, bet;
       ...
       (get P, Q, C, Ps, Pe, r, alf and bet);
       ...
       N_DistMaxLineArc(C,r,alf,bet,Ps,Pe,P,Q,&d);

 
   ACCESS:
   
     C,r     , input  ,  Center and radius of  circular arc. IF THE  ARC 
                         IS A STRAIGHT LINE, r = NL_INFINITE IS PASSED  IN!
     alf,bet , input  ,  Start and end angles. IT  IS  ASSUMED  THAT THE
                         ANGLES  REPRESENT THE SMALLEST  ARC, I.E.,  THE 
                         ONE THAT SWEEPS LESS THAN 180 DEGREES!
     Ps,Pe   , input  ,  Start and end points of the arc
     P,Q     , input  ,  End points of line segment
     d       , output ,  Maximum distance between l<P,Q> and the arc
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_DistMaxLineArc( NL_POINT C, NL_REAL r, NL_REAL alf, NL_REAL bet, NL_POINT Ps, NL_POINT Pe, NL_POINT P, NL_POINT Q, NL_REAL *d )
{

    NL_FLAG error, ifl;

    NL_REAL d1, d2, d3, t, t1, t2;

    NL_POINT R, M;

    NL_LINESEG lpq, lcs, lce;

    /* See if segment is a line */

    if( r EQ NL_INFINITE )
    {
        error = N_DistPerpPtLineSeg( P, Ps, Pe, &d1 );

        if( error EQ NL_YES )
            return (1);

        error = N_DistPerpPtLineSeg( Q, Ps, Pe, &d2 );

        if( error EQ NL_YES )
            return (1);

        if( d1 LT 0.0 OR d2 LT 0.0 )
            *d = NL_UNDEFINED;
        else
            *d = NL_MAX( d1, d2 );

        return (0);
    }

    /* Classify points wrt to the circle wedge */

    error = N_DistPtArc( P, C, r, alf, bet, Ps, Pe, &d1 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtArc( Q, C, r, alf, bet, Ps, Pe, &d2 );

    if( error EQ NL_YES )
        NL_OUT;

    if( d1 EQ NL_UNDEFINED AND d2 EQ NL_UNDEFINED )
    {
        *d = NL_UNDEFINED;
        NL_OUT;
    }

    /* Split the line segment if necessary */

    N_CreateLinePtPt( &lpq, P, Q, NL_BOUNDED );
    N_CreateLinePtPt( &lcs, C, Ps, NL_UNBOUNDED );
    N_CreateLinePtPt( &lce, C, Pe, NL_UNBOUNDED );

    if( d1 NEQ NL_UNDEFINED AND d2 EQ NL_UNDEFINED )
    {
        error = N_IsectLineLine( lpq, lcs, &R, &t1, &t2, &ifl );

        if( error EQ NL_YES )
            NL_OUT;

        if( ifl EQ NL_TRUE )
            N_DistPtPt( R, P, &d3 );
        else
            d3 = NL_BIGD;

        if( ifl EQ NL_FALSE OR d3 LT NL_MTOL )
        {
            error = N_IsectLineLine( lpq, lce, &R, &t1, &t2, &ifl );

            if( error EQ NL_YES )
                NL_OUT;

            if( ifl EQ NL_FALSE )
            {
                *d = NL_UNDEFINED;
                NL_OUT;
            }
        }

        N_VectorCopy( R, &Q );
    }

    if( d1 EQ NL_UNDEFINED AND d2 NEQ NL_UNDEFINED )
    {
        error = N_IsectLineLine( lpq, lcs, &R, &t1, &t2, &ifl );

        if( error EQ NL_YES )
            NL_OUT;

        if( ifl EQ NL_TRUE )
            N_DistPtPt( R, Q, &d3 );
        else
            d3 = NL_BIGD;

        if( ifl EQ NL_FALSE OR d3 LT NL_MTOL )
        {
            error = N_IsectLineLine( lpq, lce, &R, &t1, &t2, &ifl );

            if( error EQ NL_YES )
                NL_OUT;

            if( ifl EQ NL_FALSE )
            {
                *d = NL_UNDEFINED;
                NL_OUT;
            }
        }

        N_VectorCopy( R, &P );
    }

    /* Process circle case now */

    N_DistPtPt( C, P, &d1 );
    N_DistPtPt( C, Q, &d2 );

    d1 = fabs( d1 - r );
    d2 = fabs( d2 - r );
    *d = NL_MAX( d1, d2 );

    error = N_ProjectPtLineParam( C, P, Q, &M, &t );

    if( error EQ NL_YES )
        return (1);

    if( t GE 0.0 AND t LE 1.0 )
    {
        N_DistPtPt( C, M, &d3 );
        d3 = fabs( d3 - r );
    }
    else
    {
        d3 = -1.0;
    }

    *d = NL_MAX( *d, d3 );

    /* Exit */

    EXIT:

    return (0);
} /* end N_DistMaxLineArc */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the SIGNED distance of a point from 
     a line given in the following form:

        a*x + b*y + c = 0

     where  (a,b)  are normalized, i.e. they represent  the unit normal 
     to the plane. A typical calling example is:

       NL_POINT  P;
       NL_REAL   a, b, c, dist;
       ...
       (define line and get P);
       ...
       N_DistSignedPtLine(a,b,c,P,&dist);


   ACCESS:
   
     a,b,c , input  ,  Line's normalized implicit equation
     P     , input  ,  Given point
     dist  , output ,  SIGNED distance of point from line


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_DistSignedPtLine( NL_REAL a, NL_REAL b, NL_REAL c, NL_POINT P, NL_REAL *dist )
{

    NL_REAL x, y, z;

    /* Compute distance */

    N_PtToXYZ( P, &x, &y, &z );

    *dist = a * x + b * y + c;
} /* end N_DistSignedPtLine */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine converts the implicit equation of a 2-D line 
     to point-vector form, i.e. given

         a*x + b*y + c = 0

     it converts this equation to <P,V>, where P is a point on the line
     and V is the UNIT directiion vector.  A typical calling example:
  
       NL_REAL    a, b, c;
       NL_POINT   P;
       NL_VECTOR  V;
       ...
       (get a, b and c);
       ...
       N_LineImplicitToPtVector(a,b,c,&P,&V);


   ACCESS:
   
     a,b,c  , input  ,  Coefficients of line's equation
     P      , output ,  Point on the line
     V      , output ,  Unit direction vector


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_LineImplicitToPtVector( NL_REAL a, NL_REAL b, NL_REAL c, NL_POINT *P, NL_VECTOR *V )
{
    NL_PRIVATE NL_STRING rname = _T("N_LineImplicitToPtVector");

    NL_FLAG error = NL_NO;

    NL_REAL x, y, num;

    /* Get unit direction vector */

    N_PtFromXYZ( b, -a, 0.0, V );

    error = N_VectorNormalizeRef( V );

    if( error EQ NL_YES )
        NL_OUT;

    /* Choose a suitable point */

    if( fabs( a )GT fabs( b ) )
    {
        num = -(b * b + c);

        if( N_FloatOpIsBad( num, a, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        x = num / a;
        y = b;
    }
    else
    {
        num = -(a * a + c);

        if( N_FloatOpIsBad( num, b, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        x = a;
        y = num / b;
    }

    N_PtFromXYZ( x, y, 0.0, P );

    /* Exit */

    EXIT:

    return (error);
} /* end N_LineImplicitToPtVector */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine converts the  point-vector definition of a 
     2-D line to implicit form, i.e. given <P,V>, it computes

         a*x + b*y + c = 0

     where  (a,b,c)  are  the  coefficients  of  the  NORMLIZED  line
     equation.  A typical calling example is:
  
       NL_POINT   P;
       NL_VECTOR  V;
       NL_REAL    a, b, c;
       ...
       (get P and V);
       ...
       N_LinePtVectorToImplicit(P,V,&a,&b,&c);


   ACCESS:
   
     P      , input  ,  Point on the line
     V      , input  ,  Direction vector of the line
     a,b,c  , output ,  Coefficients of normalized line's equation


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_LinePtVectorToImplicit( NL_POINT P, NL_VECTOR V, NL_REAL *a, NL_REAL *b, NL_REAL *c )
{

    NL_FLAG error = NL_NO;

    NL_REAL dot, vx, vy, vz;

    NL_VECTOR N;

    /* Get coefficients */

    N_PtToXYZ( V, &vx, &vy, &vz );
    N_PtFromXYZ( -vy, vx, 0.0, &N );

    error = N_VectorNormalizeRef( &N );

    if( error EQ NL_YES )
        NL_OUT;

    P.z = 0.0;
    N_VectorDot( P, N, &dot );

    N_PtToXYZ( N, a, b, c );
    *c = -dot;

    /* Exit */

    EXIT:

    return (error);
} /* end N_LinePtVectorToImplicit */

/*******************************************************************//**

   DESCRIPTION:

     This geometry routine computes a least-squares line to a given set 
     of 2-D  points. The coefficients of the normalized line's equation  
     are returned, i.e.

         a*x + b*y + c = 0

     where (a,b) are the coordinates of the unit  normal to the line. A
     typical calling example is:
  
       NL_POINT  *P;
       NL_INDEX  n;
       NL_REAL   a, b, c;
       ...
       (get points P);
       ...
       N_LineFit2dPts(P,n,&a,&b,&c);


   ACCESS:
   
     P      , input  ,  Points in 2-D
     n      , input  ,  Highest index in P
     a,b,c  , output ,  Coefficients of normalized line's equation


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_LineFit2dPts( NL_POINT *P, NL_INDEX n, NL_REAL *a, NL_REAL *b, NL_REAL *c )
{
    NL_PRIVATE NL_STRING rname = _T("N_LineFit2dPts");

    /* locals */
    NL_STACKS SL;
    NL_FLAG error = NL_NO;
    NL_POINT  Q ;                                /* Q of desired Line(t) = Q + t*V */
    NL_INDEX  ii, jj ;                           /* iterators */
    NL_INDEX  ld = 1 ;                           /* Index for 2d space, equals 1 = Dim - 1 */
                                                
    /* memory for Least Squares*/     
    NL_INDEX    lMinIndex ;
    NL_REAL     cnt, SVDVal ;          
    NL_POINT   *Y ;                              /* Yi = Pi - Q */
    NL_REAL    *MW ;                             /* eigenvalues computed by N_RealMatrixSingleValueDecompose */
                                                 /*         MW   = W  of SVD M = MU * W * MV_transpose  */
    NL_RMATRIX  rmMU, rmMV ;                     /* initial rmMU = Matrix M of min( E(Q,V) = Sum_i ( Y_itranspose * M * Y_i ) */
                                                 /* final   rmMU = MU of SVD M = MU * W * MV_transpose */
                                                 /*         rmMV = MV of SVD M = MU * W * MV_transpose */

    /* Initialize system */
    N_InitNurbs( &SL );

    /* allocate array memory for Least Squares*/
    Y  = N_AllocPt1dArray( n, &SL );
    MW = N_AllocReal1dArray( ld, &SL );
    N_SetRealMatrix( &rmMU, ld, ld, NL_MT_FULL, ld, &SL );
    N_SetRealMatrix( &rmMV, ld, ld, NL_MT_FULL, ld, &SL );

    /* use least squares to find the line -  see notes */
    /* Let LinePt = 1/m Sum_i(pt_i) */
    /* Let LineVec = eigenvector associated with smallest eigenvalue of the matrix M */
    /*      where M = Sum_i((Yi_trans*Yi)*I - Yi * Yi_trans) */
    /*           Yi = Pi-Q */

    /* get Q = Pt Center */
    Q.x = Q.y = Q.z = 0.0 ;

    cnt = (NL_REAL)n + 1.0;
    if( N_FloatOpIsBad( 1.0, cnt, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    for(ii=0;ii<=n;ii++)
      {
        Q.x += P[ii].x ; 
        Q.y += P[ii].y ; 
      }
    Q.x /= cnt ; 
    Q.y /= cnt ; 

    /* Build Y vecs */
    for(ii=0;ii<=n;ii++)
      {
        Y[ii].x = P[ii].x - Q.x ;
        Y[ii].y = P[ii].y - Q.y ;
      }

    /* init Matrix M */
    for(ii=0;ii<=ld;ii++)
      { for(jj=0;jj<=ld;jj++)
          { rmMU.RM[ii][jj] = 0.0 ; }
      }

    /* Build Matrix M = Sum_i((Yi_trans*Yi)*I - Yi * Yi_trans */
    for(ii=0;ii<=n;ii++)
      {
        NL_REAL YdotY = Y[ii].x * Y[ii].x + Y[ii].y * Y[ii].y ;

        rmMU.RM[0][0] += YdotY - Y[ii].x * Y[ii].x ;
        rmMU.RM[0][1] +=       - Y[ii].x * Y[ii].y ;
                     
        rmMU.RM[1][0] +=       - Y[ii].y * Y[ii].x ;
        rmMU.RM[1][1] += YdotY - Y[ii].y * Y[ii].y ;
      }

    /* use SVD to decompose M */
    N_RealMatrixSingleValueDecompose( rmMU.RM, MW, rmMV.RM, ld, ld );  

    /* find eigenvector associated with min eigenvalue */    
    lMinIndex = 0 ;
    SVDVal    = MW[0] ;

    for(ii=1;ii<=ld;ii++)
      {
       if(SVDVal > MW[ii]) { lMinIndex = ii ; SVDVal = MW[ii] ; }
      }

    /* set output, N = [a, b] vector perp to V */
    *a = -rmMU.RM[1][lMinIndex] ;
    *b =  rmMU.RM[0][lMinIndex] ;

    /* set output, C = - N dot Q */
    *c = - (*a * Q.x + *b * Q.y) ;

#ifdef OBSOLETE

    NL_PRIVATE NL_STRING rname = _T("N_LineFit2dPts");

    NL_FLAG dir, error = NL_NO;

    NL_INDEX i, j;

    NL_REAL ** m, x, y, aa, bb, fac, xmin, xmax, ymin, ymax, xd, yd;

    NL_RMATRIX M, Mt, MtM, MtMi, A;

    NL_STACKS SL;

    /* Initialize system */

    N_InitNurbs( &SL );

    /* Get form of line equation */

    xmin = xmax = P[0].x;
    ymin = ymax = P[0].y;

    for ( i = 1; i <= n; i++ )
    {
        if( P[i].x LT xmin )
            xmin = P[i].x;

        if( P[i].x GT xmax )
            xmax = P[i].x;

        if( P[i].y LT ymin )
            ymin = P[i].y;

        if( P[i].y GT ymax )
            ymax = P[i].y;
    }

    xd = fabs( xmax - xmin );
    yd = fabs( ymax - ymin );

    if( xd LE yd )
        dir = 1;
    else
        dir = 2;

    /* Get initial matrix */

    error = N_SetRealMatrix( &M, n, 1, NL_MT_FULL, 0, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &M, &m );

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( P[i], &x, &y, &fac );

        m[i][0] = 1.0;

        if( dir EQ 1 )
            m[i][1] = y;
        else
            m[i][1] = x;
    }

    /* Get (MtM)^{-1}Mt */

    N_InitRealMatrix( &Mt );
    error = N_RealMatrixTranspose( &M, &Mt, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &MtM );
    error = N_RealMatrixMultiply( &Mt, &M, &MtM, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &MtMi );
    error = N_RealMatrixInverse( &MtM, NL_NO, &MtMi, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &A );
    error = N_RealMatrixMultiply( &MtMi, &Mt, &A, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &A, &m );

    /* Compute coefficients */

    aa = bb = 0.0;

    for ( j = 0; j <= n; j++ )
    {
        N_PtToXYZ( P[j], &x, &y, &fac );

        if( dir EQ 1 )
        {
            aa += m[0][j] * x;
            bb += m[1][j] * x;
        }
        else
        {
            aa += m[0][j] * y;
            bb += m[1][j] * y;
        }
    }

    fac = sqrt( bb * bb + 1.0 );

    if( N_FloatOpIsBad( 1.0, fac, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );
    fac = 1.0 / fac;
    *c = fac * aa;

    if( dir EQ 1 )
    {
        *a = -fac;
        *b = fac * bb;
    }
    else
    {
        *a = fac * bb;
        *b = -fac;
    }

#endif /* OBSOLETE */

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_LineFit2dPts */

/*******************************************************************//**

   DESCRIPTION:

     This geometry routine computes a least-squares line to a given set 
     of 3D points. The point-vector form <Q,V> of the line is returned. 
     Q = Point Cloud Center of Gravity
     V = unit-direction of Line(t) = Q + tV that minimizes
     E(Q,V) = Sum_i ( (P_i - Q) - ((P_i - Q) dot V)*V )**2 )

     A typical calling example is:
  
       NL_POINT   *P, *Q;
       NL_VECTOR  V;
       NL_INDEX  n;
       ...
       (get points P);
       ...
       N_LineFit3dPts(P,n,&Q,&V);


   ACCESS:
   
     P      , input  ,  Points in 3-D
     n      , input  ,  Highest index in P
     Q, V   , output ,  Point and direction vector of line


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_LineFit3dPts( NL_POINT *P, NL_INDEX n, NL_POINT *Q, NL_VECTOR *V )
{
    NL_PRIVATE NL_STRING rname = _T("N_LineFit3dPts");

#ifdef OBSOLETE 
    NL_FLAG flg, error = NL_NO;

    NL_INDEX i;

    NL_REAL x, y, z, xmin, xmax, ymin, ymax, zmin, zmax, xd, yd, zd, a_xy, b_xy, c_xy, a_yz, b_yz, c_yz, a_zx, b_zx, c_zx;

    NL_POINT *P_xy, *P_yz, *P_zx, R_xy, R_yz, R_zx;

    NL_VECTOR D_xy, D_yz, D_zx, N_xy, N_yz, N_zx;

    NL_PLANE pl_xy, pl_yz, pl_zx;
#endif /* obsolete */

    /* locals */
    NL_STACKS SL;
    NL_FLAG error = NL_NO;
    NL_INDEX  ii, jj ;                           /* iterators */
    NL_INDEX  ld = 2 ;                           /* Index for 3d space3, equals 2 = Dim - 1 */
                                                
    /* memory for Least Squares*/     
    NL_INDEX    lMinIndex ;
    NL_REAL     cnt, SVDVal ;          
    NL_POINT   *Y ;                              /* Yi = Pi - Q */
    NL_REAL    *MW ;                             /* eigenvalues computed by N_RealMatrixSingleValueDecompose */
                                                 /*         MW   = W  of SVD M = MU * W * MV_transpose  */
    NL_RMATRIX  rmMU, rmMV ;                     /* initial rmMU = Matrix M of min( E(Q,V) = Sum_i ( Y_itranspose * M * Y_i ) */
                                                 /* final   rmMU = MU of SVD M = MU * W * MV_transpose */
                                                 /*         rmMV = MV of SVD M = MU * W * MV_transpose */

    /* Initialize system */
    N_InitNurbs( &SL );

    /* allocate array memory for Least Squares*/
    Y  = N_AllocPt1dArray( n, &SL );
    MW = N_AllocReal1dArray( ld, &SL );
    N_SetRealMatrix( &rmMU,        ld, ld, NL_MT_FULL, ld, &SL );
    N_SetRealMatrix( &rmMV,        ld, ld, NL_MT_FULL, ld, &SL );

    /* use least squares to find the line -  see notes */
    /* Let LinePt = 1/m Sum_i(pt_i) */
    /* Let LineVec = eigenvector associated with smallest eigenvalue of the matrix M */
    /*      where M = Sum_i((Yi_trans*Yi)*I - Yi * Yi_trans) */
    /*           Yi = Pi-Q */

    /* get Q = Pt Center */
    Q->x = Q->y = Q->z = 0.0 ;

    cnt = (NL_REAL)n + 1.0;
    if( N_FloatOpIsBad( 1.0, cnt, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );

    for(ii=0;ii<=n;ii++)
      {
        N_VectorSum(P[ii], *Q, Q) ; 
      }
    N_VectorScale(*Q, 1.0/cnt, Q) ;

    /* Build Y vecs */
    for(ii=0;ii<=n;ii++)
      {
        N_VectorDiff(P[ii],*Q,Y+ii) ;
      }

    /* init Matrix M */
    for(ii=0;ii<=ld;ii++)
      { for(jj=0;jj<=ld;jj++)
          { rmMU.RM[ii][jj] = 0.0 ; }
      }

    /* Build Matrix M = Sum_i((Yi_trans*Yi)*I - Yi * Yi_trans */
    for(ii=0;ii<=n;ii++)
      {
        NL_REAL YdotY ;
        N_VectorDot(Y[ii],Y[ii], &YdotY) ;
        rmMU.RM[0][0] += YdotY - Y[ii].x * Y[ii].x ;
        rmMU.RM[0][1] +=       - Y[ii].x * Y[ii].y ;
        rmMU.RM[0][2] +=       - Y[ii].x * Y[ii].z ;
                     
        rmMU.RM[1][0] +=       - Y[ii].y * Y[ii].x ;
        rmMU.RM[1][1] += YdotY - Y[ii].y * Y[ii].y ;
        rmMU.RM[1][2] +=       - Y[ii].y * Y[ii].z ;
                     
        rmMU.RM[2][0] +=       - Y[ii].z * Y[ii].x ;
        rmMU.RM[2][1] +=       - Y[ii].z * Y[ii].y ;
        rmMU.RM[2][2] += YdotY - Y[ii].z * Y[ii].z ;
      }

    /* use SVD to decompose M */
    N_RealMatrixSingleValueDecompose( rmMU.RM, MW, rmMV.RM, ld, ld );  

    /* find eigenvector associated with min eigenvalue */    
    lMinIndex = 0 ;
    SVDVal    = MW[0] ;

    for(ii=1;ii<=ld;ii++)
      {
       if(SVDVal > MW[ii]) { lMinIndex = ii ; SVDVal = MW[ii] ; }
      }

    /* Set output vec Q */
    V->x = rmMU.RM[0][lMinIndex] ;
    V->y = rmMU.RM[1][lMinIndex] ;
    V->z = rmMU.RM[2][lMinIndex] ;


#ifdef OBSOLETE

    /* Get min-max box */

    xmin = xmax = P[0].x;
    ymin = ymax = P[0].y;
    zmin = zmax = P[0].z;

    for ( i = 1; i <= n; i++ )
    {
        if( P[i].x LT xmin )
            xmin = P[i].x;

        if( P[i].x GT xmax )
            xmax = P[i].x;

        if( P[i].y LT ymin )
            ymin = P[i].y;

        if( P[i].y GT ymax )
            ymax = P[i].y;

        if( P[i].z LT zmin )
            zmin = P[i].z;

        if( P[i].z GT zmax )
            zmax = P[i].z;
    }

    xd = fabs( xmax - xmin );
    yd = fabs( ymax - ymin );
    zd = fabs( zmax - zmin );

    /* Project onto the <x,y> and <z,x> planes */

    if( xd GE yd AND xd GE zd )
    {
        /* Project to principal planes */

        P_xy = N_AllocPt1dArray( n, &SL );

        if( P_xy EQ NULL )
            NL_QUIT;

        P_zx = N_AllocPt1dArray( n, &SL );

        if( P_zx EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= n; i++ )
        {
            P_xy[i].x = P[i].x;
            P_zx[i].x = P[i].x;
            P_xy[i].y = P[i].y;
            P_zx[i].y = P[i].z;
            P_xy[i].z = 0.0;
            P_zx[i].z = 0.0;
        }

        /* Get least-squares lines */

        error = N_LineFit2dPts( P_xy, n, &a_xy, &b_xy, &c_xy );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_LineFit2dPts( P_zx, n, &a_zx, &b_zx, &c_zx );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get planes perpendicular to principal planes along these lines */

        error = N_LineImplicitToPtVector( a_xy, b_xy, c_xy, &R_xy, &D_xy );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_LineImplicitToPtVector( a_zx, b_zx, c_zx, &R_zx, &D_zx );

        if( error EQ NL_YES )
            NL_OUT;

        N_PtToXYZ( R_zx, &x, &y, &z );
        N_PtFromXYZ( x, 0.0, y, &R_zx );

        N_PtToXYZ( D_xy, &x, &y, &z );
        N_PtFromXYZ( -y, x, 0.0, &N_xy );

        N_PtToXYZ( D_zx, &x, &y, &z );
        N_PtFromXYZ( -y, 0.0, x, &N_zx );

        /* Intersect planes */

        N_CreatePlanePtNormal( &pl_xy, R_xy, N_xy );
        N_CreatePlanePtNormal( &pl_zx, R_zx, N_zx );

        error = N_IsectPlanePlane( pl_xy, pl_zx, Q, V, &flg );

        if( error EQ NL_YES )
            NL_OUT;

        if( flg EQ NL_FALSE )
            NL_ERROR( NL_NUM_ERR );

        NL_OUT;
    }

    /* Project onto the <x,y> and <y,z> planes */

    if( yd GE xd AND yd GE zd )
    {
        /* Project to principal planes */

        P_xy = N_AllocPt1dArray( n, &SL );

        if( P_xy EQ NULL )
            NL_QUIT;

        P_yz = N_AllocPt1dArray( n, &SL );

        if( P_yz EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= n; i++ )
        {
            P_xy[i].x = P[i].x;
            P_yz[i].x = P[i].y;
            P_xy[i].y = P[i].y;
            P_yz[i].y = P[i].z;
            P_xy[i].z = 0.0;
            P_yz[i].z = 0.0;
        }

        /* Get least-squares lines */

        error = N_LineFit2dPts( P_xy, n, &a_xy, &b_xy, &c_xy );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_LineFit2dPts( P_yz, n, &a_yz, &b_yz, &c_yz );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get planes perpendicular to principal planes along these lines */

        error = N_LineImplicitToPtVector( a_xy, b_xy, c_xy, &R_xy, &D_xy );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_LineImplicitToPtVector( a_yz, b_yz, c_yz, &R_yz, &D_yz );

        if( error EQ NL_YES )
            NL_OUT;

        N_PtToXYZ( R_yz, &x, &y, &z );
        N_PtFromXYZ( 0.0, x, y, &R_yz );

        N_PtToXYZ( D_xy, &x, &y, &z );
        N_PtFromXYZ( -y, x, 0.0, &N_xy );

        N_PtToXYZ( D_yz, &x, &y, &z );
        N_PtFromXYZ( 0.0, -y, x, &N_yz );

        /* Intersect planes */

        N_CreatePlanePtNormal( &pl_xy, R_xy, N_xy );
        N_CreatePlanePtNormal( &pl_yz, R_yz, N_yz );

        error = N_IsectPlanePlane( pl_xy, pl_yz, Q, V, &flg );

        if( error EQ NL_YES )
            NL_OUT;

        if( flg EQ NL_FALSE )
            NL_ERROR( NL_NUM_ERR );

        NL_OUT;
    }

    /* Project onto the <y,z> and <z,x> planes */

    if( zd GE xd AND zd GE yd )
    {
        /* Project to principal planes */

        P_yz = N_AllocPt1dArray( n, &SL );

        if( P_yz EQ NULL )
            NL_QUIT;

        P_zx = N_AllocPt1dArray( n, &SL );

        if( P_zx EQ NULL )
            NL_QUIT;

        for ( i = 0; i <= n; i++ )
        {
            P_yz[i].x = P[i].y;
            P_zx[i].x = P[i].x;
            P_yz[i].y = P[i].z;
            P_zx[i].y = P[i].z;
            P_yz[i].z = 0.0;
            P_zx[i].z = 0.0;
        }

        /* Get least-squares lines */

        error = N_LineFit2dPts( P_yz, n, &a_yz, &b_yz, &c_yz );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_LineFit2dPts( P_zx, n, &a_zx, &b_zx, &c_zx );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get planes perpendicular to principal planes along these lines */

        error = N_LineImplicitToPtVector( a_yz, b_yz, c_yz, &R_yz, &D_yz );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_LineImplicitToPtVector( a_zx, b_zx, c_zx, &R_zx, &D_zx );

        if( error EQ NL_YES )
            NL_OUT;

        N_PtToXYZ( R_yz, &x, &y, &z );
        N_PtFromXYZ( 0.0, x, y, &R_yz );

        N_PtToXYZ( R_zx, &x, &y, &z );
        N_PtFromXYZ( x, 0.0, y, &R_zx );

        N_PtToXYZ( D_yz, &x, &y, &z );
        N_PtFromXYZ( 0.0, -y, x, &N_yz );

        N_PtToXYZ( D_zx, &x, &y, &z );
        N_PtFromXYZ( -y, 0.0, x, &N_zx );

        /* Intersect planes */

        N_CreatePlanePtNormal( &pl_yz, R_yz, N_yz );
        N_CreatePlanePtNormal( &pl_zx, R_zx, N_zx );

        error = N_IsectPlanePlane( pl_yz, pl_zx, Q, V, &flg );

        if( error EQ NL_YES )
            NL_OUT;

        if( flg EQ NL_FALSE )
            NL_ERROR( NL_NUM_ERR );

        NL_OUT;
    }

#endif /* obsolete method */

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_LineFit3dPts */

/*******************************************************************//**

   DESCRIPTION:

     This geometry  routine  computes a  best fitting line  to a set of 
     points in 2-D or  in 3-D. A point and a unit  direction vector are 
     returned. The method first finds the center of gravity (C) of the 
     points,  followed by a least squares computation of the V. 
     
     A  typical calling example is:
  
       NL_POINT   *P, C;
       NL_VECTOR  V;
       NL_INDEX   n;
       NL_REAL    era, erm;
       ...
       (get points P);
       ...
       N_LineFitPts(P,n,&C,&V,&era,&erm);


   ACCESS:
   
     P    , input  ,  Points
     n    , input  ,  Highest index in P
     C    , output ,  Point on line
     V    , output ,  Unit direction vector of line
     era  , output ,  Average error
     erm  , output ,  Maximum error


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_LineFitPts( NL_POINT *P, NL_INDEX n, NL_POINT *C, NL_VECTOR *V, NL_REAL *era, NL_REAL *erm )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_LineFitPts"); */

    NL_FLAG error = NL_NO;

    NL_INDEX ii ;

    NL_REAL dm, da, d ;

    NL_FLAG bIs2d = 0 ;

    NL_LINESEG lsg;

    /* check for 2d pts */
    if(P[0].z == NL_NOZ)
      {
        /* temporarily move all NL_NOZ values to z = 0 */
        bIs2d = 1 ;
        for(ii=0;ii<=n;ii++) { P[ii].z = 0.0 ; }
      }

    /* pass the call along */
    error = N_LineFit3dPts(P, n, C, V) ;

    /* check for 2d pts */
    if(bIs2d)
      {
        /* restore NL_NOZ values  */
        for(ii=0;ii<=n;ii++) { P[ii].z = NL_NOZ ; }
        C->z = NL_NOZ ;
        V->z = NL_NOZ ;
      }

    /* Get the errors */

    N_CreateLineStartDirVector( &lsg, *C, *V, NL_UNBOUNDED );

    da =  0.0;
    dm = -1.0;

    for ( ii = 0; ii <= n; ii++ )
      {
        error = N_DistPtInfLine( lsg, P[ii], &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT dm )
            dm = d;
        da += d;
      }

    da /= (NL_REAL)n + 1.0 ;

    /* Get the output */
    *era = da;
    *erm = dm;

#ifdef OBSOLETE

    NL_INDEX i, il;

    NL_REAL d, da, dm, inv;

    NL_POINT CC, CM;

    NL_VECTOR VV, VM, VD, VF;

    NL_LINESEG lsg;

    /* Get various entities */

    inv = (NL_REAL)(n + 1);

    if( N_FloatOpIsBad( 1.0, inv, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );
    inv = 1.0 / inv;

    /* Compute center of gravity */

    N_VectorCreate( 0.0, 0.0, 0.0, &CM );

    for ( i = 0; i <= n; i++ )
        N_VectorSum( P[i], CM, &CM );
    N_VectorScale( CM, inv, &CC );

    /* Get reference vector */

    il = -1;
    dm = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        N_DistSqPtPt2d( CC, P[i], &d );

        if( d GT dm )
        {
            dm = d;
            il = i;
        }
    }

    N_VectorDiff( P[il], CC, &VF );

    /* Get average direction vector */

    N_VectorCreate( 0.0, 0.0, 0.0, &VM );

    for ( i = 0; i <= n; i++ )
    {
        N_VectorDiff( P[i], CC, &VD );
        N_VectorDot( VD, VF, &d );

        if( d LT 0.0 )
            N_VectorReverseInPlace( &VD );
        N_VectorSum( VD, VM, &VM );
    }

    N_VectorScale( VM, inv, &VV );

    error = N_VectorNormalizeRef( &VV );

    if( error EQ NL_YES )
        NL_OUT;

    N_CreateLineStartDirVector( &lsg, CC, VV, NL_UNBOUNDED );

    /* Get the errors */

    da = 0.0;
    dm = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        error = N_DistPtInfLine( lsg, P[i], &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT dm )
            dm = d;
        da += d;
    }

    da *= inv;

    /* Get the output */

    N_VectorCopy( CC, C );
    N_VectorCopy( VV, V );

    *era = da;
    *erm = dm;

#endif /* OBSOLETE */

    /* Exit */

    EXIT:

    return (error);

} /* end N_LineFitPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine rotates a 2-D point set with respect to an
     arbitrary center. A typical calling example is:
  
       NL_POINT  *P, *Q, O;
       NL_REAL   alf;
       NL_INDEX  n;
       ...
       (get P, O, alf and memory for Q);
       ...
       N_Rotate2dPts(P,n,O,alf,Q);

     MEMORY FOR Q MUST BE ALLOCATED IN THE CALLING ROUTINE!


   ACCESS:
   
     P   , input  ,  Point set
     n   , input  ,  Highest index in P
     O   , input  ,  Center of rotation
     alf , input  ,  Angle of rotation in degrees
     Q   , output ,  Rotated points


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Rotate2dPts( NL_POINT *P, NL_INDEX n, NL_POINT O, NL_REAL alf, NL_POINT *Q )
{

    NL_INDEX i;

    NL_REAL rad, sinal, cosal, f1, f2;

    rad = (alf * NL_PI) / 180.0;
    sinal = sin( rad );
    cosal = cos( rad );
    f1 = O.x * (1.0 - cosal) + O.y * sinal;
    f2 = O.y * (1.0 - cosal) - O.x * sinal;

    for ( i = 0; i <= n; i++ )
    {
        Q[i].x = P[i].x * cosal - P[i].y * sinal + f1;
        Q[i].y = P[i].x * sinal + P[i].y * cosal + f2;
        Q[i].z = 0.0;
    }
} /* end N_Rotate2dPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine scales a 2-D point set with respect to an 
     arbitrary point. A typical calling example is:
  
       NL_POINT   *P, *Q, O;
       NL_VECTOR  S;
       NL_INDEX   n;
       ...
       (get P, O, S and memory for Q);
       ...
       N_Scale2dPts(P,n,O,S,Q);

     MEMORY FOR Q MUST BE ALLOCATED IN THE CALLING ROUTINE!


   ACCESS:
   
     P  , input  ,  Point set
     n  , input  ,  Highest index in P
     O  , input  ,  Center of scaling
     S  , input  ,  Scaling vector
     Q  , output ,  Scaled points


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Scale2dPts( NL_POINT *P, NL_INDEX n, NL_POINT O, NL_VECTOR S, NL_POINT *Q )
{

    NL_INDEX i;

    NL_REAL f1, f2;

    f1 = O.x * (1.0 - S.x);
    f2 = O.y * (1.0 - S.y);

    for ( i = 0; i <= n; i++ )
    {
        Q[i].x = P[i].x * S.x + f1;
        Q[i].y = P[i].y * S.y + f2;
        Q[i].z = 0.0;
    }
} /* end N_Scale2dPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine shears a 2-D point set given a shear vector. 
     A typical calling example is:
  
       NL_POINT   *P, *Q;
       NL_VECTOR  SH;
       NL_INDEX   n;
       ...
       (get P, SH and memory for Q);
       ...
       N_Shear2dPts(P,n,SH,Q);

     MEMORY FOR Q MUST BE ALLOCATED IN THE CALLING ROUTINE!


   ACCESS:
   
     P  , input  ,  Point set
     n  , input  ,  Highest index in P
     SH , input  ,  Shear vector
     Q  , output ,  Sheared points


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Shear2dPts( NL_POINT *P, NL_INDEX n, NL_VECTOR SH, NL_POINT *Q )
{

    NL_INDEX i;

    for ( i = 0; i <= n; i++ )
    {
        Q[i].x = P[i].x + SH.x * P[i].y;
        Q[i].y = SH.y * P[i].x + P[i].y;
        Q[i].z = 0.0;
    }
} /* end N_Shear2dPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine translates a 2-D point set given a translation 
     vector. A typical calling example is:
  
       NL_POINT   *P, *Q;
       NL_VECTOR  T;
       NL_INDEX   n;
       ...
       (get P, T and memory for Q);
       ...
       N_Translate2dPts(P,n,T,Q);

     MEMORY FOR Q MUST BE ALLOCATED IN THE CALLING ROUTINE!


   ACCESS:
   
     P  , input  ,  Point set
     n  , input  ,  Highest index in P
     T  , input  ,  Translation vector
     Q  , output ,  Translated points


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Translate2dPts( NL_POINT *P, NL_INDEX n, NL_VECTOR T, NL_POINT *Q )
{

    NL_INDEX i;

    for ( i = 0; i <= n; i++ )
    {
        Q[i].x = P[i].x + T.x;
        Q[i].y = P[i].y + T.y;
        Q[i].z = 0.0;
    }
} /* end N_Translate2dPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes a transformation matrix that maps a
     given tetrahedron to its image.

              [A] Pi = Qi,


                     |a00 ... a02 tx|
              [A] =  |            ty| 
                     |a20 ... a22 tz| 
                     | 0   0   0  1 | 


     where A is a 4x4 matrix. A typical calling example is:
  
       NL_POINT    P1,P2,P3,P4,Q1,Q2,Q3,Q4;

       NL_RMATRIX  rma;
       NL_STACKS   SG;
       ...
       (get P1, P2, ...  Q3, Q4);
       ...
       N_InitRealMatrix(&rma);
       N_TetraCalcMatrix(P1,P2,P3,P4,Q1,Q2,Q3,Q4,&rma,&SG);

     If rma is initialized to the NULL matrix,  memory  is allocated in
     the  routine.  If not, it is  assumed that  memory  allocation has
     been done.  However, the routine checks for  the proper  amount by
     looking at the matrix indexes.

     This  function  was  contributed by Art Hammerschmidt of Pratt and
     Whitney Canada, Inc.


   ACCESS:
   
     P1,P2,P3,P4 , input  ,  Corner points of a tetrahedron
     Q1,Q2,Q3,Q4 , input  ,  Image of corner points of tetrahedron
     rma         , output ,  Transformation matrix
     SG          , input  ,  rma's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_TetraCalcMatrix( NL_POINT P1, NL_POINT P2, NL_POINT P3, NL_POINT P4, NL_POINT Q1, NL_POINT Q2, NL_POINT Q3, NL_POINT Q4, NL_RMATRIX *rma, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_TetraCalcMatrix");

    NL_FLAG error = NL_NO;

    NL_REAL ** R, ** A, *X;

    NL_RMATRIX rm1;

    NL_INDEX *ind, i, j, n;

    NL_STACKS SL;

    /* Initialize NURBS */

    N_InitNurbs( &SL );

    /* Check and allocate memory */

    error = N_CheckMemRealMatrix( rma, 3, 3, NL_MT_FULL, 3, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    n = 11;

    error = N_SetRealMatrix( &rm1, n, n, NL_MT_FULL, 11, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &rm1, &R );
    N_GetRealMatrixPtr( rma, &A );

    /* Initialize matrix R */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= n; j++ )
            R[i][j] = 0.0;
    }

    /* Build matrices R[][] * X[] = Y[]  */

    N_PtToXYZ( P1, &R[0][0], &R[0][1], &R[0][2] );     /* A00  => Q1x  */
    R[0][3] = 1.0;

    N_PtToXYZ( P2, &R[1][0], &R[1][1], &R[1][2] );     /* A01  => Q2x  */
    R[1][3] = 1.0;

    N_PtToXYZ( P3, &R[2][0], &R[2][1], &R[2][2] );     /* A02  => Q3x  */
    R[2][3] = 1.0;

    N_PtToXYZ( P4, &R[3][0], &R[3][1], &R[3][2] );     /* A03  => Q4x  */
    R[3][3] = 1.0;

    N_PtToXYZ( P1, &R[4][4], &R[4][5], &R[4][6] );     /* A10  => Q1y  */
    R[4][7] = 1.0;

    N_PtToXYZ( P2, &R[5][4], &R[5][5], &R[5][6] );     /* A11  => Q2y  */
    R[5][7] = 1.0;

    N_PtToXYZ( P3, &R[6][4], &R[6][5], &R[6][6] );     /* A12  => Q3y  */
    R[6][7] = 1.0;

    N_PtToXYZ( P4, &R[7][4], &R[7][5], &R[7][6] );     /* A13  => Q4y  */
    R[7][7] = 1.0;

    N_PtToXYZ( P1, &R[8][8], &R[8][9], &R[8][10] );    /* A20  => Q1z  */
    R[8][11] = 1.0;

    N_PtToXYZ( P2, &R[9][8], &R[9][9], &R[9][10] );    /* A21  => Q2z  */
    R[9][11] = 1.0;

    N_PtToXYZ( P3, &R[10][8], &R[10][9], &R[10][10] ); /* A22  => Q3z  */
    R[10][11] = 1.0;

    N_PtToXYZ( P4, &R[11][8], &R[11][9], &R[11][10] ); /* A23  => Q4z  */
    R[11][11] = 1.0;

    /* LU decompose matrix */

    ind = N_AllocInt1dArray( n, &SL );

    if( ind EQ NULL )
        NL_QUIT;

    error = N_RealMatrixLuDecomposePivot( &rm1, ind );

    if( error EQ NL_YES )
        NL_OUT;

    X = N_AllocReal1dArray( n, &SL );

    if( X EQ NULL )
        NL_QUIT;

    N_PtToXYZ( Q1, &X[0], &X[4], &X[8] );
    N_PtToXYZ( Q2, &X[1], &X[5], &X[9] );
    N_PtToXYZ( Q3, &X[2], &X[6], &X[10] );
    N_PtToXYZ( Q4, &X[3], &X[7], &X[11] );

    /* solution vector X */

    error = N_RealMatrixForBackPivot( &rm1, ind, X );

    if( error EQ NL_YES )
        NL_OUT;

    /* build output tranformation matrix A[][]  */

    A[0][0] = X[0];
    A[0][1] = X[1];
    A[0][2] = X[2];
    A[0][3] = X[3];
    A[1][0] = X[4];
    A[1][1] = X[5];
    A[1][2] = X[6];
    A[1][3] = X[7];
    A[2][0] = X[8];
    A[2][1] = X[9];
    A[2][2] = X[10];
    A[2][3] = X[11];
    A[3][0] = 0.0;
    A[3][1] = 0.0;
    A[3][2] = 0.0;
    A[3][3] = 1.0;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_TetraCalcMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine  computes the maximum area plane to a grid of
     3D points. That is,  it finds a  plane on which the  convex hull of 
     the projections of the points form  the maximum area. If the points
     lie on a symmetric object such as a sphere or a cylinder, no unique
     plane can be found, i.e. this routine works best with general point
     sets. A typical calling example is:
  
       NL_POINT   **P;
       NL_INDEX   n, m;
       NL_REAL    tol;
       NL_VECTOR  N;
       ...
       (get points P and tol);
       ...
       N_MaxAreaPlane3dPts(P,n,m,tol,&N);

     IF THE ROUTINE FAILS TO CONVERGE,  NL_CON_ERR ALONG WITH THE BEST SOL-
     UTION ARE RETURNED  (may  be  the  case  where  no  unique solution 
     exists). 


   ACCESS:
   
     P    , input  ,  Points in 3-D
     n,m  , input  ,  Highest indexes in P
     tol  , input  ,  Angular  tolerance; the  iteration  stops when the
                      angle between consecutive iterates is <tol (tol is
                      given IN DEGREES!)
     N    , output ,  Unit normal to maximum area plane  (best  solution 
                      if no convergence)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_MaxAreaPlane3dPts( NL_POINT ** P, NL_INDEX n, NL_INDEX m, NL_REAL tol, NL_VECTOR *N )
{
    NL_PRIVATE NL_STRING rname = _T("N_MaxAreaPlane3dPts");

    NL_INDEX itl = 20;

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k;

    NL_REAL ** M, alf;

    NL_VECTOR D, Vb, Vd, Vl, Wbd, Wdl;

    NL_RMATRIX a, at, ata;

    NL_STACKS SL;

    /* Initialize system */

    N_InitNurbs( &SL );

    /* Get initial matrix */

    k = 2 * n * m - 1;

    error = N_SetRealMatrix( &a, k, 2, NL_MT_FULL, 0, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &a, &M );

    k = -1;

    for ( i = 0; i < n; i++ )
    {
        for ( j = 0; j < m; j++ )
        {
            N_VectorDiff( P[i + 1][j], P[i][j], &Vb );
            N_VectorDiff( P[i + 1][j + 1], P[i][j], &Vd );
            N_VectorDiff( P[i][j + 1], P[i][j], &Vl );
            N_VectorCross( Vb, Vd, &Wbd );
            N_VectorCross( Vd, Vl, &Wdl );
            N_VectorScale( Wbd, 0.5, &Wbd );
            N_VectorScale( Wdl, 0.5, &Wdl );

            M[k + 1][0] = Wbd.x;
            M[k + 1][1] = Wbd.y;
            M[k + 1][2] = Wbd.z;
            M[k + 2][0] = Wdl.x;
            M[k + 2][1] = Wdl.y;
            M[k + 2][2] = Wdl.z;

            k += 2;
        }
    }

    /* Get (a^t)a */

    N_InitRealMatrix( &at );
    error = N_RealMatrixTranspose( &a, &at, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &ata );
    error = N_RealMatrixMultiply( &at, &a, &ata, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &ata, &M );

    /* Compute maximum eigenvector */

    D.x = 1.0;
    D.y = 1.0;
    D.z = 1.0;
    k = 0;

    while( k LT itl )
    {
        /* Normalize vector */

        error = N_VectorNormalizeRef( &D );

        if( error EQ NL_YES )
            NL_OUT;

        /* Get new vector */

        N->x = M[0][0] * D.x + M[0][1] * D.y + M[0][2] * D.z;
        N->y = M[1][0] * D.x + M[1][1] * D.y + M[1][2] * D.z;
        N->z = M[2][0] * D.x + M[2][1] * D.y + M[2][2] * D.z;

        /* Check angle */

        error = N_VectorsAngle( D, *N, &alf );

        if( error EQ NL_YES )
            NL_OUT;

        if( alf LT tol )
            break;

        D.x = N->x;
        D.y = N->y;
        D.z = N->z;
        k++;
    }

    error = N_VectorNormalizeRef( N );

    if( error EQ NL_YES )
        NL_OUT;

    if( k GE itl )
        NL_ERROR( NL_CON_ERR );

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_MaxAreaPlane3dPts */

/*******************************************************************//**


   DESCRIPTION:

     Given an unordered 2D point set. This geometry routine computes the
     k   nearest  neighbors of  a subset  of  points. That is,  for each 
     selected  indexes, it  finds the  k immediate  neighbors. A typical 
     calling example:

       NL_POINT   *P;
       NL_INDEX   *ind, **cln, n, m, k;
       NL_REAL    **cld, tol, sfg;
       NL_STACKS  SG;
       ...
       (get points, indexes, tol, sfg and k);
       ...
       N_PairSelected2dPts(P,n,ind,m,k,tol,sfg,&cln,&cld,&SG);

     The data is interpreted as follows:

       ind[0..m]         : indexes for which k neighbors are to be found
       P[ind[0..m]]      : points for which k neighbors are to be found
       cln[0..m][0..k-1] : indexes of closest neighbors
       cln[i][0..k-1]    : indexes of closest neighbors of P[ind[i]]
       cld[i][0..k-1]    : distances from closest neighbors of P[ind[i]]


   ACCESS:
   
     P   , input  ,  Unordered 2-D points
     n   , input  ,  Highest index in P
     ind , input  ,  Index array
     m   , input  ,  Highest index in ind
     k   , input  ,  Find the first k nearest points (k < n!)
     tol , input  ,  Point coincidence tolerance
     sfg , input  ,  Scale size of uniform grid data structure:
                       = 1.0: default size appropriate for uniform point
                              distribution
                       > 1.0: appropriate for data with holes and isles;
                              1.5 seems to give good performance for the
                              majority of practical data sets
     cln , output ,  Closest neighbor indexes
     cld , output ,  Closest neighbor distances
     SG  , input  ,  cln's and cld's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_PairSelected2dPts( NL_POINT *P, NL_INDEX n, NL_INDEX *ind, NL_INDEX m, NL_INDEX k, NL_REAL tol, NL_REAL sfg, NL_INDEX *** cln, NL_REAL *** cld, NL_STACKS *SG )
{

    NL_PRIVATE NL_STRING rname = _T("N_PairSelected2dPts");

    NL_PRIVATE NL_INDEX ces = 15;

    NL_FLAG ** vst, flg, error = NL_NO;

    NL_INDEX *** cell, ** top, ** clo, ** tm, *ib, i, j, l, r, s, t, u, ic, jc, il, ih, jl, jh, ji, ip, itp, xres, yres;

    NL_REAL ** dlo, *db, x, y, z, xl, xr, yb, yt, xd, yd, x1, x2, y1, y2, dsh2, dis2, dmin2, tol2, rtp, size;

    NL_STACKS SL;

    /* Initialize system */

    N_InitNurbs( &SL );

    if( k GT n )
        NL_ERROR( NL_INP_ERR );

    /* Get min-max box */

    N_PtToXYZ( P[0], &x, &y, &z );

    xl = xr = x;
    yb = yt = y;

    for ( i = 1; i <= n; i++ )
    {
        N_PtToXYZ( P[i], &x, &y, &z );

        if( x LT xl )
            xl = x;

        if( x GT xr )
            xr = x;

        if( y LT yb )
            yb = y;

        if( y GT yt )
            yt = y;
    }

    /* Get cells to cover point set */

    xl -= tol;
    xr += tol;
    yb -= tol;
    yt += tol;

    xd = fabs( xr - xl );
    yd = fabs( yt - yb );

    size = sfg * sqrt( (xd * yd) / ((NL_REAL)n + (NL_REAL)1) );
    xres = (NL_INDEX)(xd / size);
    yres = (NL_INDEX)(yd / size);

    top = N_AllocInt2dArray( xres, yres, &SL );

    if( top EQ NULL )
        NL_QUIT;

    tm = N_AllocInt2dArray( xres, yres, &SL );

    if( tm EQ NULL )
        NL_QUIT;

    cell = N_AllocIntPtr2dArray( xres, yres, &SL );

    if( cell EQ NULL )
        NL_QUIT;

    vst = N_AllocFlag2dArray( xres, yres, &SL );

    if( vst EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= xres; i++ )
    {
        for ( j = 0; j <= yres; j++ )
        {
            top[i][j] = -1;
            tm[i][j] = ces;

            cell[i][j] = N_AllocInt1dArray( ces, &SL );

            if( cell[i][j]EQ NULL )
                NL_QUIT;
        }
    }

    /* Put each point in a cell discarding coincident points */

    tol2 = tol * tol;

    for ( i = 0; i <= n; i++ )
    {
        /* Find cell indexes */

        N_PtToXYZ( P[i], &x, &y, &z );

        ic = (NL_INDEX)((x - xl) / size);
        jc = (NL_INDEX)((y - yb) / size);

        /* Check for coincident points */

        flg = NL_NO;

        for ( j = 0; j <= top[ic][jc]; j++ )
        {
            l = cell[ic][jc][j];

            N_DistSqPtPt2d( P[i], P[l], &dis2 );

            if( dis2 LT tol2 )
            {
                flg = NL_YES;
                break;
            }
        }

        /* Put point in the cell */

        if( flg EQ NL_NO )
        {
            l = top[ic][jc] + 1;

            if( l GT tm[ic][jc] )
            {
                error = N_Realloc1dIntArray( &cell[ic][jc], tm[ic][jc], tm[ic][jc] + ces, &SL );

                if( error EQ NL_YES )
                    NL_OUT;

                tm[ic][jc] += ces;
            }

            cell[ic][jc][l] = i;
            top[ic][jc]++;
        }
    }

    /* Get k-bucket and output memory */

    db = N_AllocReal1dArray( k - 1, &SL );

    if( db EQ NULL )
        NL_QUIT;

    ib = N_AllocInt1dArray( k - 1, &SL );

    if( ib EQ NULL )
        NL_QUIT;

    clo = N_AllocInt2dArray( m, k - 1, SG );

    if( clo EQ NULL )
        NL_QUIT;

    dlo = N_AllocReal2dArray( m, k - 1, SG );

    if( dlo EQ NULL )
        NL_QUIT;

    /* Now find k neighbors */

    il = 0;
    ih = xres;
    jl = 0;
    jh = yres;

    for ( r = 0; r <= m; r++ )
    {
        s = ind[r];

        for ( i = il; i <= ih; i++ )
        {
            for ( j = jl; j <= jh; j++ )
                vst[i][j] = NL_NO;
        }

        /* Get cell indexes */

        N_PtToXYZ( P[s], &x, &y, &z );

        ic = (NL_INDEX)((x - xl) / size);
        jc = (NL_INDEX)((y - yb) / size);

        x1 = xl + ic * size;
        x2 = x1 + size;
        y1 = yb + jc * size;
        y2 = y1 + size;

        /* Get shortest distance from point to cell walls */

        dsh2 = (x - x1) * (x - x1);

        if( (x2 - x) * (x2 - x)LT dsh2 )
            dsh2 = (x2 - x) * (x2 - x);

        if( (y - y1) * (y - y1)LT dsh2 )
            dsh2 = (y - y1) * (y - y1);

        if( (y2 - y) * (y2 - y)LT dsh2 )
            dsh2 = (y2 - y) * (y2 - y);

        /* Initialize k-buckets */

        for ( i = 0; i <= k - 1; i++ )
        {
            db[i] = NL_BIGD;
            ib[i] = -1;
        }

        dmin2 = NL_BIGD;

        /* Now search for nearest neighbors */

        l = 0;

        while( dmin2 GT dsh2 )
        {
            il = NL_MAX( 0, ic - l );
            jl = NL_MAX( 0, jc - l );
            ih = NL_MIN( xres, ic + l );
            jh = NL_MIN( yres, jc + l );

            for ( i = il; i <= ih; i++ )
            {
                if( i EQ il OR i EQ ih )
                    ji = 1;
                else
                    ji = jh - jl;

                for ( j = jl; j <= jh; j += ji )
                {
                    if( vst[i][j]EQ NL_YES )
                        continue;

                    /* Check distances for all points in the cell */

                    for ( t = 0; t <= top[i][j]; t++ )
                    {
                        ip = cell[i][j][t];

                        if( ip EQ s )
                            continue;

                        N_DistSqPtPt2d( P[s], P[ip], &dis2 );

                        if( dis2 LT db[k - 1] )
                        {
                            db[k - 1] = dis2;
                            ib[k - 1] = ip;

                            /* Resort bucket */

                            for ( u = k - 1; u >= 1; u-- )
                            {
                                if( db[u]GT db[u - 1] )
                                    break;

                                rtp = db[u];
                                itp = ib[u];
                                db[u] = db[u - 1];
                                ib[u] = ib[u - 1];
                                db[u - 1] = rtp;
                                ib[u - 1] = itp;
                            }
                        }
                    }

                    vst[i][j] = NL_YES;
                }
            }

            dmin2 = db[k - 1];
            dsh2 += size * size;
            l++;
        }

        /* Save nearest neighbors' indexes and distances */

        for ( t = 0; t <= k - 1; t++ )
        {
            dlo[r][t] = sqrt( db[t] );
            clo[r][t] = ib[t];
        }
    }

    /* Get output */

    *cln = clo;
    *cld = dlo;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_PairSelected2dPts */

/*******************************************************************//**


   DESCRIPTION:

     Given an unordered set of points lying on or near a 3-D curve. This
     geometry  routine orders the  points so  that they  can be used for 
     fitting or  approximation. It is assumed  that the point set sweeps
     out a curve that does not bend more than 180 degrees anywhere along 
     its path, and that the points are reasonably uniformly distributed.
     If the unordered set contains duplicate points, or points  that map
     onto the same point on the  least-squares line, the routine removes
     the offending point(s) and represents them with one that is closest
     to the nearest non-offending neighbor. A typical calling example:

       NL_POINT   *P, *Q;
       NL_INDEX   n, m;
       NL_REAL    tol;
       NL_STACKS  SG;
       ...
       (get points);
       ...
       N_Order3dPts(P,n,tol,&Q,&m,&SG);

     MEMORY FOR Q IS ALLOCATED INSIDE THEE ROUTINE!


   ACCESS:
   
     P   , input  ,  Unordered points
     n   , input  ,  Highest index in P
     tol , input  ,  Point coincidence tolerance
     Q   , output ,  Ordered poins
     m   , output ,  Highest index in Q
     SG  , input  ,  Q's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Order3dPts( NL_POINT *P, NL_INDEX n, NL_REAL tol, NL_POINT ** Q, NL_INDEX *m, NL_STACKS *SG )
{

    NL_FLAG flg, error;

    NL_INDEX *ind, i, j, k, l, im, in;

    NL_REAL *t, u, d, dmin;

    NL_POINT *D, A, R;

    NL_VECTOR V;

    NL_LINESEG lsg;

    NL_STACKS SL;

    /* Initialize system */

    N_InitNurbs( &SL );

    /* Get least-squares line */

    error = N_LineFit3dPts( P, n, &R, &V );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &V );

    if( error EQ NL_YES )
        NL_OUT;

    /* Project points onto least-squares line */

    t = N_AllocReal1dArray( n, &SL );

    if( t EQ NULL )
        NL_QUIT;

    ind = N_AllocInt1dArray( n, &SL );

    if( ind EQ NULL )
        NL_QUIT;

    N_CreateLineStartDirVector( &lsg, R, V, NL_UNBOUNDED );

    for ( i = 0; i <= n; i++ )
    {
        error = N_ProjectPtLine( lsg, P[i], &A, &t[i], &flg );

        if( error EQ NL_YES )
            NL_OUT;

        ind[i] = i;
    }

    /* Sort parameters and indexes */

    k = n + 1;

    while( k GT 1 )
    {
        if( k GE 5 )
            k = (5 * k - 1) / 11;
        else
            k = 1;

        for ( i = n - k; i >= 0; i-- )
        {
            u = t[i];
            l = ind[i];

            for ( j = i + k; j <= n && u > t[j]; j += k )
            {
                t[j - k] = t[j];
                ind[j - k] = ind[j];
            }

            t[j - k] = u;
            ind[j - k] = l;
        }
    }

    /* Order and purge points */

    D = N_AllocPt1dArray( n, SG );

    if( D EQ NULL )
        NL_QUIT;

    k = 0;
    l = -1;

    while( k LE n )
    {
        j = k;
        l++;

        while( k LE n AND fabs( t[k] - t[k + 1] )LT tol )
            k++;

        if( k GT j )
        {
            dmin = NL_BIGD;
            im = 0;

            if( l EQ 0 )
                in = k + 1;
            else
                in = j - 1;

            for ( i = j; i <= k; i++ )
            {
                N_DistPtPt( P[ind[i]], P[ind[in]], &d );

                if( d LT dmin )
                {
                    d = dmin;
                    im = i;
                }
            }

            N_VectorCopy( P[ind[im]], &D[l] );
        }
        else
        {
            N_VectorCopy( P[ind[k]], &D[l] );
        }

        k++;
    }

    /* Get output */

    *Q = D;
    *m = l;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_Order3dPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the min-max box of a 1-D point
     array. A typical calling example is:

       NL_POINT      *P;
       NL_INDEX      n;
       NL_MINMAXBOX  box;
       ...
       (get array P);
       ...
       N_Pts1dCalcBBox(P,n,&box);


   ACCESS:
   
     P    , input  ,  Point array
     n    , input  ,  Highest index in P
     box  , output ,  Min-max box of P


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Pts1dCalcBBox( NL_POINT *P, NL_INDEX n, NL_MINMAXBOX *box )
{

    NL_EPOLYGON ppl;

    /* Get min-max box */

    N_EPolygonFromPts( &ppl, n, P );
    N_PolygonGetBBox( &ppl, box );
} /* end N_Pts1dCalcBBox */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the min-max box of a 2-D point
     array. A typical calling example is:

       NL_POINT      **P;
       NL_INDEX      n, m;
       NL_MINMAXBOX  box;
       ...
       (get array P);
       ...
       N_Pts2dCalcBBox(P,n,m,&box);


   ACCESS:
   
     P    , input  ,  Point array
     n,m  , input  ,  Highest indexes in P
     box  , output ,  Min-max box of P


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_Pts2dCalcBBox( NL_POINT ** P, NL_INDEX n, NL_INDEX m, NL_MINMAXBOX *box )
{

    NL_ENET ntl;

    /* Get min-max box */

    N_ENetFromPts( &ntl, n, m, P );
    N_ENetGetBBox( &ntl, box );
} /* end N_Pts2dCalcBBox */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes derivatives of a circle given in the
     traditional trigonometric form:
 
       C(u) = O + R*cos(ang)*X + R*sin(ang)*Y

     where O is the center, X and Y are orthogonal  vectors spanning the
     plane of the  circle, R is the radius and ang is the sweep angle. A 
     typical calling example is:

       NL_POINT   O;
       NL_VECTOR  *D, X, Y;
       NL_REAL    R, ang;
       NL_INDEX   k;
       ... 
       (get defining data and allocate memory for D);
       ...
       N_CalcCircArcDerivs(O,X,Y,R,ang,k,D);

     MEMORY FOR D  MUST BE ALLOCATED IN THE  CALLING ROUTINE TO HOLD ALL 
     THE DERIVATIVES D[0], D[1],..., D[K]! X AND Y MUST BE UNIT AXES!!


   ACCESS:
   
     O,X,Y  , input  ,  Center and orthogonal UNIT axes of circle
     R      , input  ,  Radius of circle
     ang    , input  ,  Sweep angle
     k      , input  ,  Highest index of derivatives:
                          D[0]: point on the circle
                          D[1]: first derivative
                          ...
                          D[k]: k-th derivative
     D      , output ,  Derivatives of circle. Note that D[i]=D[i+4]!


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CalcCircArcDerivs( NL_POINT O, NL_VECTOR X, NL_VECTOR Y, NL_REAL R, NL_REAL ang, NL_INDEX k, NL_VECTOR *D )
{

    NL_INDEX i;

    NL_REAL rad, alf = 0.0, bet = 0.0, gam, del;

    /* Get point and derivatives */

    rad = (NL_PI * ang) / 180.0;
    gam = R * cos( rad );
    del = R * sin( rad );

    N_TranslateSum2Pts( O, gam, X, del, Y, &D[0] );

    for ( i = 1; i <= k; i++ )
    {
        switch( i % 4 )
        {
            case 1:
                alf = -del;
                bet = gam;
                break;

            case 2:
                alf = -gam;
                bet = -del;
                break;

            case 3:
                alf = del;
                bet = -gam;
                break;

            case 0:
                alf = gam;
                bet = del;
                break;
        }

        N_Combine2Pts( alf, X, bet, Y, &D[i] );
    }
} /* end N_CalcCircArcDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine  computes the  area of a spherical polygon in
     spherical degrees, i.e. the full sphere has an area of 4*NL_PI radian,
     that is, 720  degrees. The  vertices of the  polygon are  listed in 
     counterclockwise  order,  and  the  coordinates  are  those of  the 
     spherical  coordinates of latitude and longitude. A typical calling 
     example is:
  
       NL_REAL   *u, *v, AA;
       NL_INDEX  n;
       ...
       (get coordinates of vertices);
       ...
       N_AreaSpherePolygon(u,v,n,&AA);


   ACCESS:
   
     u,v , input  ,  Coordinates of  vertices listed in counterclockwise
                     order. The  spherical vertices have typical coordi-
                     nates 0<=u<=180 and 0<=v<=360
     n   , input  ,  Highest index in (u,v)
     AA  , output ,  Area of polygon in spherical degrees


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AreaSpherePolygon( NL_REAL *u, NL_REAL *v, NL_INDEX n, NL_REAL *AA )
{

    NL_PRIVATE NL_REAL DEG = 57.2957795130823208;

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_REAL *ul, *vl, bet1, bet2 = 0.0, lam1, lam2 = 0.0, cosb1;
    NL_REAL cosb2 = 0.0, hava, sum, exc;
    NL_REAL fac, A, B, C, S, T, HPI, ang, a1, a2, a3, a4;

    NL_STACKS SL;

    /* Initialize system */

    N_InitNurbs( &SL );

    /* Convert coordinates */

    HPI = 0.5 *NL_PI;
    fac = NL_PI / 180.0;

    ul = N_AllocReal1dArray( n, &SL );

    if( ul EQ NULL )
        NL_QUIT;

    vl = N_AllocReal1dArray( n, &SL );

    if( vl EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= n; i++ )
    {
        ul[i] = 90.0 - u[i];
        ul[i] *= fac;
        vl[i] = fac * v[i];
    }

    /* Compute the area */

    sum = 0.0;

    for ( i = 0; i <= n; i++ )
    {
        j = i + 1;

        if( i EQ 0 )
        {
            lam1 = vl[i];
            bet1 = ul[i];
            lam2 = vl[i + 1];
            bet2 = ul[i + 1];
            cosb1 = cos( bet1 );
            cosb2 = cos( bet2 );
        }
        else
        {
            j = (i + 1) % (n + 1);

            lam1 = lam2;
            bet1 = bet2;
            lam2 = vl[j];
            bet2 = ul[j];
            cosb1 = cosb2;
            cosb2 = cos( bet2 );
        }

        if( lam1 NEQ lam2 )
        {
            A = ST_havers( bet2 - bet1 );
            B = ST_havers( lam2 - lam1 );

            hava = A + cosb1 * cosb2 * B;

            A = 2 * asin( sqrt( hava ) );
            B = HPI - bet2;
            C = HPI - bet1;
            S = 0.5 *(A + B + C);

            a1 = 0.5 *S;
            a2 = 0.5 *(S - A);
            a3 = 0.5 *(S - B);
            a4 = 0.5 *(S - C);

            T = tan( a1 ) * tan( a2 ) * tan( a3 ) * tan( a4 );
            ang = sqrt( fabs( T ) );
            exc = DEG * fabs( 4.0 *atan( ang ) );

            if( lam2 LT lam1 )
                exc = -exc;

            sum += exc;
        }
    }

    /* Get the output */

    *AA = fabs( sum );

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_AreaSpherePolygon */

/**********************************************************************/
/* ST_havers: Evaluate Haversine function                              */
/**********************************************************************/

NL_REAL ST_havers( NL_REAL a )
{

    return (0.5 *(1.0 - cos( a )));
}

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine  computes  the  area of a  spherical patch in
     spherical degrees, i.e. the full sphere has an area of 4*NL_PI radian,
     that is, 720 degrees. The patch is given by the start and end sweep
     angles. A typical calling example is:
  
       NL_REAL  us, ue, vs, ve, AA;
       ...
       (get sweep angles);
       ...
       N_AreaSpherePatch(us,ue,vs,ve,&AA);


   ACCESS:
   
     us,ue , input  ,  Sweep  angles from  the north pole  to the  south 
                       pole. Must  satisfy 0<=us < ue<=180. us=0  is the
                       north pole, and ue=180 is the south pole.
     vs,ve , input  ,  Sweep  angles around  the  equator. Must  satisfy 
                       vs<ve and ve-vs<=360.
     AA    , output ,  Area of rectangular spherical patch


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AreaSpherePatch( NL_REAL us, NL_REAL ue, NL_REAL vs, NL_REAL ve, NL_REAL *AA )
{
    NL_PRIVATE NL_STRING rname = _T("N_AreaSpherePatch");

    NL_FLAG error = NL_NO;

    NL_REAL u[4], v[4], xs, xe, ys, ye, xt, yt, sum, B;

    /* Check error */

    if( us GE ue OR vs GE ve )
        NL_ERROR( NL_INP_ERR );

    if( us LT 0.0 OR ue GT 180.0 )
        NL_ERROR( NL_INP_ERR );

    if( ve - vs GT 360.0 )
        NL_ERROR( NL_INP_ERR );

    /* Compute the area */

    xs = us;
    xe = xs;
    xt = ue;
    ys = 0.0;
    ye = ys;
    yt = ve - vs;

    sum = 0.0;

    while( xe LT xt )
    {
        xe = NL_MIN( xt, xs + 90.0 );

        while( ye LT yt )
        {
            ye = NL_MIN( yt, ys + 90.0 );

            u[0] = xs;
            v[0] = ys;
            u[1] = xe;
            v[1] = ys;
            u[2] = xe;
            v[2] = ye;
            u[3] = xs;
            v[3] = ye;

            error = N_AreaSpherePolygon( u, v, 3, &B );

            if( error EQ NL_YES )
                NL_OUT;

            sum += B;
            ys = ye;
        }

        xs = xe;
        ys = 0.0;
        ye = ys;
    }

    /* Get the output */

    *AA = sum;

    /* Exit */

    EXIT:

    return (error);
} /* end N_AreaSpherePatch */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine checks if three points are collinear, i.e. if 
     they form a  degenerate triangle. The three points are input in any
     order. A typical calling example is:

       NL_POINT  A, B, C;
       NL_REAL   tol;
       ...
       (get A, B, C and tol);
       ...
       if( N_PtsAreColinear(A,B,C,tol) )  --> points are collinear;

     The tolerance "tol" is used to check the distance of the projection
     of one of the  points to the longest  side of the triangle <A,B,C>.
     A good default for tol is NL_MTOL!


   ACCESS:
   
     A,B,C , input  ,  Vertices of the triangle
     tol   , input  ,  Tolerance; use  NL_MTOL if no specific  tolerance is
                       needed or available


   RETURN CODES:

     NL_YES: Points are collinear
     NL_NO : Points are not collinear

   ***********************************************************************/

/* NL_BOOLEAN  N_PtsAreColinear */
NL_BOOLEAN N_PtsAreColinear( NL_POINT A, NL_POINT B, NL_POINT C, NL_REAL tol )
{

    NL_FLAG error;

    NL_REAL dab, dbc, dca, d, tol2;

    /* Check collinearity */

    N_DistSqPtPt( A, B, &dab );
    N_DistSqPtPt( B, C, &dbc );
    N_DistSqPtPt( C, A, &dca );

    tol2 = tol * tol;

    if( dab LT tol2 OR dbc LT tol2 OR dca LT tol2 )
        return NL_YES;

    if( dab GT dbc AND dab GT dca )
    {
        error = N_DistPerpPtLineSeg( C, A, B, &d );

        if( error EQ NL_YES )
            return NL_NO;

        if( d LT tol )
            return NL_YES;
        else
            return NL_NO;
    }
    else if( dbc GT dca AND dbc GT dab )
    {
        error = N_DistPerpPtLineSeg( A, B, C, &d );

        if( error EQ NL_YES )
            return NL_NO;

        if( d LT tol )
            return NL_YES;
        else
            return NL_NO;
    }

    if( dca GT dab AND dca GT dbc )
    {
        error = N_DistPerpPtLineSeg( B, C, A, &d );

        if( error EQ NL_YES )
            return NL_NO;

        if( d LT tol )
            return NL_YES;
        else
            return NL_NO;
    }

    return NL_NO;
} /* end N_PtsAreColinear */

/*******************************************************************//**


   DESCRIPTION:

     This  geometry routine  computes the  center and  the  radius of a 
     circle defined by  three non-collinear  points. If the  points are
     collinear,  the  indicator flag "cfl" is set to NL_FALSE. Both 2-D as
     well as  3-D cases are  handled, i.e. the  circle does not have to 
     lie in the <x,y> plane. A typical calling example is:

       NL_POINT  A, B, C, D;
       NL_REAL   r;
       NL_FLAG   cfl;
       ...
       (get points A, B, and C);
       ...
       N_CalcCircCenterAndRadius(A,B,C,&D,&r,&cfl);


   ACCESS:
   
     A,B,C , input  ,  Points on the circle
     D     , output ,  Center of circle
     r     , output ,  Radius of circle
     cfl   , output ,  Flag:
                         NL_TRUE : circle computed
                         NL_FALSE: no circle computed, e.g. the points are
                                collinear


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CalcCircCenterAndRadius( NL_POINT A, NL_POINT B, NL_POINT C, NL_POINT *D, NL_REAL *r, NL_FLAG *cfl )
{

    NL_FLAG ifl, error = NL_NO;

    NL_REAL x, y, za, zb, zc, fac, rr, t1, t2, ra, rb, rc;

    NL_POINT Mab, Mbc, Mca, G, P, Pa, Pb, Pc, CC;

    NL_VECTOR Vab, Vbc, Vca, Wab, Wbc, Wca, Na, Nb, Nc, N, V;

    NL_LINESEG lab, lbc, lca;

    NL_PLANE pab, pbc, pca, pln;

    /* See if points are collinear */

    *cfl = NL_FALSE;

    if( N_PtsAreColinear( A, B, C, NL_MTOL ) )
        NL_OUT;

    N_DistPtPt( A, B, &za );
    N_DistPtPt( B, C, &zb );
    N_DistPtPt( C, A, &zc );

    if( za LT NL_MTOL OR zb LT NL_MTOL OR zc LT NL_MTOL )
        NL_OUT;

    fac = 1.0 / 3.0;

    /* Get some points and vectors */

    N_VectorCombine( 0.5, A, 0.5, B, &Mab );
    N_VectorDiff( B, A, &Vab );
    N_VectorCombine( 0.5, B, 0.5, C, &Mbc );
    N_VectorDiff( C, B, &Vbc );
    N_VectorCombine( 0.5, C, 0.5, A, &Mca );
    N_VectorDiff( A, C, &Vca );

    /* Get perpendicular bisectors */

    N_PtToXYZ( A, &x, &y, &za );
    N_PtToXYZ( B, &x, &y, &zb );
    N_PtToXYZ( C, &x, &y, &zc );

    if( fabs( za )LT NL_MTOL AND fabs( zb )LT NL_MTOL AND fabs( zc )LT NL_MTOL )
    {
        /* Points are in the <x,y> plane */

        N_VectorPerpendicular( Vab, &Wab );
        N_VectorPerpendicular( Vbc, &Wbc );
        N_VectorPerpendicular( Vca, &Wca );

        N_CreateLineStartDirVector( &lab, Mab, Wab, NL_UNBOUNDED );
        N_CreateLineStartDirVector( &lbc, Mbc, Wbc, NL_UNBOUNDED );
        N_CreateLineStartDirVector( &lca, Mca, Wca, NL_UNBOUNDED );
    }
    else
    {
        /* Points are NOT in the <x,y> plane */

        N_VectorCross( Vab, Vbc, &Nb );
        N_VectorCross( Vbc, Vca, &Nc );
        N_VectorCross( Vca, Vab, &Na );

        N_VectorCreate( 0.0, 0.0, 0.0, &N );
        N_VectorCreate( 0.0, 0.0, 0.0, &G );

        N_VectorSum( A, B, &G );
        N_VectorSum( C, G, &G );

        N_VectorSum( Na, Nb, &N );
        N_VectorSum( Nc, N, &N );

        N_VectorScale( G, fac, &G );
        N_VectorScale( N, fac, &N );

        N_CreatePlanePtNormal( &pln, G, N );
        N_CreatePlanePtNormal( &pab, Mab, Vab );
        N_CreatePlanePtNormal( &pbc, Mbc, Vbc );
        N_CreatePlanePtNormal( &pca, Mca, Vca );

        error = N_IsectPlanePlane( pln, pab, &P, &V, &ifl );

        if( error EQ NL_YES OR ifl EQ NL_FALSE )
            NL_OUT;
        N_CreateLineStartDirVector( &lab, P, V, NL_UNBOUNDED );

        error = N_IsectPlanePlane( pln, pbc, &P, &V, &ifl );

        if( error EQ NL_YES OR ifl EQ NL_FALSE )
            NL_OUT;
        N_CreateLineStartDirVector( &lbc, P, V, NL_UNBOUNDED );

        error = N_IsectPlanePlane( pln, pca, &P, &V, &ifl );

        if( error EQ NL_YES OR ifl EQ NL_FALSE )
            NL_OUT;
        N_CreateLineStartDirVector( &lca, P, V, NL_UNBOUNDED );
    }

    /* Intersect the three bisectors */

    error = N_IsectLineLine( lab, lbc, &Pb, &t1, &t2, &ifl );

    if( error EQ NL_YES OR ifl EQ NL_FALSE )
        NL_OUT;

    error = N_IsectLineLine( lbc, lca, &Pc, &t1, &t2, &ifl );

    if( error EQ NL_YES OR ifl EQ NL_FALSE )
        NL_OUT;

    error = N_IsectLineLine( lca, lab, &Pa, &t1, &t2, &ifl );

    if( error EQ NL_YES OR ifl EQ NL_FALSE )
        NL_OUT;

    /* Get the center and the radius */

    N_VectorCreate( 0.0, 0.0, 0.0, &CC );
    N_VectorSum( Pa, Pb, &CC );
    N_VectorSum( Pc, CC, &CC );
    N_VectorScale( CC, fac, &CC );

    N_DistPtPt( CC, A, &ra );
    N_DistPtPt( CC, B, &rb );
    N_DistPtPt( CC, C, &rc );

    rr = fac * (ra + rb + rc);

    /* Create the output */

    N_VectorCopy( CC, D );

    *r = rr;
    *cfl = NL_TRUE;

    /* Exit */

    EXIT:

    return (error);
} /* end N_CalcCircCenterAndRadius */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine  computes a least-squares circle to a set of 
     2-D  points. The  center and the  radius  are returned. The method
     minimizies the  geometric  error in the least-squares  sense, i.e.

         Sum_i [r^2 - (x_i-cx)^2 - (y_i-cy)^2]^2 = min

     is computed, where r is the radius and (cx,cy) denotes the center.
     A relative  tolerance of  10^{-5} is used to  improve the  initial
     approximation via  Gauss-Newton iterations. If  Gauss-Newton fails
     after some number of iterations, as in the case of collinear data,
     the best  circle is  returned and  the error flag is  cleared. The  
     average  and  the  maximum  errors  are also  returned. A  typical 
     calling example is:
  
       NL_POINT  *PP;
       NL_INDEX  n;
       NL_REAL   tol, cx, cy, rr, era, erm;
       NL_FLAG   cfl
       ...
       (get points PP and tolerance tol);
       ...
       N_CreateCircFrom2dPts(PP,n,tol,NL_AVERAGE,&cx,&cy,&rr,&era,&erm,&cfl);

     In order to  avoid numerical  errors, the  routine  checks if  the 
     points are  collinear, i.e., if the maximum distance of the points
     from the best fitting line is less than "tol".


   ACCESS:
   
     PP    , input  ,  Points in 2-D
     n     , input  ,  Highest index in PP
     tol   , input  ,  Tolerance to check collinearity
     etp   , input  ,  Flag:
                         NL_MAXIMUM: tol is maximum deviation
                         NL_AVERAGE: tol is average deviation
     cx,cy , output ,  Center of circle
     rr    , output ,  Radius of circle
     era   , output ,  Average absolute error
     erm   , output ,  Maximum absolute error
     cfl   , output ,  Flag:
                         NL_TRUE : circle computed
                         NL_FALSE: no  cicle is computed, e.g. the  points 
                                are collinear or degenerate


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CreateCircFrom2dPts
 ( NL_POINT * PP,   /* in : Points in 2-D                                 */
   NL_INDEX   n,    /* in : Highest index in PP                           */
   NL_REAL    tol,  /* in : Tolerance to check collinearity to radius     */
   NL_FLAG    etp,  /* in : NL_MAXIMUM: tol is maximum deviation          */
                    /*      NL_AVERAGE: tol is average deviation          */
   NL_REAL  * cx,   /* out: x of Circle Center                            */
   NL_REAL  * cy,   /* out: y of Circle Center                            */
   NL_REAL  * rr,   /* out: Radius of circle                              */
   NL_REAL  * era,  /* out: Average absolute error                        */
   NL_REAL  * erm,  /* out: Maximum absolute error                        */
   NL_FLAG  * cfl ) /* out: NL_TRUE : circle computed                     */    
                    /*      NL_FALSE: points are collinear or degenerate, */ 
                    /*                no circle computed                  */
{
    NL_PRIVATE NL_STRING rname = _T("N_CreateCircFrom2dPts");

    NL_PRIVATE NL_INDEX itlm = 30;

    NL_FLAG error = NL_NO;

    NL_INDEX ind[3], i, itc;

    NL_REAL ** RM, ** JM, *rhs, sol[3], A, B, C, D, E, F, G, H, x, y, z, xo, yo, ro, dis, ri, eavp, eavn, eavo, x2, y2, xy, x2py2, fac, xc, yc, rc, inv, da, dao, dm, dmo;

    NL_POINT CC;

    NL_VECTOR VV;

    NL_RMATRIX rma, rmj;

    NL_STACKS SL;

    /* Initialize system */

    N_InitNurbs( &SL );

    /* See if points are collinear */

    *cfl = NL_FALSE;

    error = N_LineFitPts( PP, n, &CC, &VV, era, erm );

    if( error EQ NL_YES )
        NL_OUT;

    switch( etp )
    {
        case NL_MAXIMUM:
            if( *erm LT tol )
                NL_OUT;
            break;

        case NL_AVERAGE:
            if( *era LT tol )
                NL_OUT;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Handle special case of n=2 */

    if( n EQ 2 )
    {
        error = N_CalcCircCenterAndRadius( PP[0], PP[1], PP[2], &CC, rr, cfl );

        if( error EQ NL_YES OR *cfl EQ NL_FALSE )
            NL_OUT;

        N_PtToXYZ( CC, cx, cy, &z );
        *era = 0.0;
        *erm = 0.0;

        NL_OUT;
    }

    /* Get various entities */

    A = B = C = D = E = F = G = H = 0.0;

    fac = (NL_REAL)n + 1.0;

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( PP[i], &x, &y, &z );

        x2 = x * x;
        y2 = y * y;
        xy = x * y;
        x2py2 = x2 + y2;

        A += x;
        B += y;
        C += x2;
        D += y2;
        E += xy;
        G += x * x2py2;
        H += y * x2py2;
    }

    F = C + D;

    /* Solve system of equations */

    error = N_SetRealMatrix( &rma, 2, 2, NL_MT_FULL, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &rma, &RM );

    RM[0][0] = 2.0 *C;
    RM[0][1] = 2.0 *E;
    RM[0][2] = A;
    sol[0] = G;
    RM[1][0] = 2.0 *E;
    RM[1][1] = 2.0 *D;
    RM[1][2] = B;
    sol[1] = H;
    RM[2][0] = 2.0 *A;
    RM[2][1] = 2.0 *B;
    RM[2][2] = fac;
    sol[2] = F;

    error = N_RealMatrixLuDecomposePivot( &rma, ind );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixForBackPivot( &rma, ind, sol );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get initial center and radius and error measures */

    xc = sol[0];
    yc = sol[1];
    rc = sqrt( xc * xc + yc * yc + sol[2] );

    eavp = 0.0;
    dm = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( PP[i], &x, &y, &z );

        ri = sqrt( (x - xc) * (x - xc) + (y - yc) * (y - yc) );
        eavp += fabs( rc - ri );

        if( fabs( rc - ri )GT dm )
            dm = fabs( rc - ri );
    }

    if( N_FloatOpIsBad( eavp, fac, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );
    eavp /= fac;

    xo = xc;
    yo = yc;
    ro = rc;
    dao = eavp;
    dmo = dm;

    if( eavp LT NL_MTOL )
    {
        *cx = xo;
        *cy = yo;
        *rr = ro;
        *era = eavp;
        *erm = dm;
        *cfl = NL_TRUE;

        NL_OUT;
    }

    if( N_FloatOpIsBad( eavp, rc, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );
    eavp /= rc;
    eavo = eavp;

    /************************************************/
    /* Improve circle fit by Gauss-Newton iteration */
    /************************************************/

    /* Set up least-squares problem */

    error = N_SetRealMatrix( &rmj, n, 2, NL_MT_FULL, 2, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &rmj, &JM );

    rhs = N_AllocReal1dArray( n, &SL );

    if( rhs EQ NULL )
        NL_QUIT;

    itc = 0;

    while( itc LE itlm )
    {
        /* Set up Jacobian and the right hand side */

        for ( i = 0; i <= n; i++ )
        {
            N_PtToXYZ( PP[i], &x, &y, &z );
            ri = sqrt( (x - xc) * (x - xc) + (y - yc) * (y - yc) );

            if( N_FloatOpIsBad( 1.0, ri, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            inv = 1.0 / ri;

            JM[i][0] = (xc - x) * inv;
            JM[i][1] = (yc - y) * inv;
            JM[i][2] = -1.0;
            rhs[i] = rc - ri;
        }

        /* Solve by least-squares */

        error = N_RealMatrixLstSqSolve( &rmj, rhs, sol );

        if( error EQ NL_YES )
        {
            N_ErrClear();
            error = NL_NO;
            break;
        }

        xc += sol[0];
        yc += sol[1];
        rc += sol[2];

        /* Recompute the average error */

        eavn = 0.0;
        dm = -1.0;

        for ( i = 0; i <= n; i++ )
        {
            N_PtToXYZ( PP[i], &x, &y, &z );

            ri = sqrt( (x - xc) * (x - xc) + (y - yc) * (y - yc) );
            eavn += fabs( rc - ri );

            if( fabs( rc - ri )GT dm )
                dm = fabs( rc - ri );
        }

        if( N_FloatOpIsBad( eavn, fac, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        eavn /= fac;

        da = eavn;

        if( N_FloatOpIsBad( eavn, rc, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        eavn /= rc;

        /* Check for convergence and save best result */

        dis = sqrt( sol[0] * sol[0] + sol[1] * sol[1] + sol[2] * sol[2] );

        if( N_FloatOpIsBad( dis, rc, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        dis /= rc;

        /* Best result is with the smallest average error */

        if( eavn LT eavo )
        {
            xo = xc;
            yo = yc;
            ro = rc;
            dao = da;
            dmo = dm;
            eavo = eavn;
        }

        /* If average error does not change, out */

        if( fabs( eavp - eavn )LT tol )
            break;

        /* If center does not move, out */

        if( dis LT tol )
            break;

        eavp = eavn;
        itc++;
    }

    /* Get the output */

    *cx = xo;
    *cy = yo;
    *rr = ro;
    *era = dao;
    *erm = dmo;
    *cfl = NL_TRUE;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_CreateCircFrom2dPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine  computes a least-squares sphere to a set of 
     3-D  points. The  center and the  radius  are returned. The method
     minimizies the  geometric  error in the least-squares  sense, i.e.

       Sum_i [r^2 - (x_i-cx)^2 - (y_i-cy)^2 - (z_i-cz)^2]^2 = min

     is computed, where  r is the radius and  (cx,cy,cz) is the center.
     A relative  tolerance of  10^{-5} is used to  improve the  initial
       NL_PRIVATE  NL_STRING  rname = "N_SphereFit3dPts");
approximation via  Gauss-Newton iterations. If  Gauss-Newton fails
     after some number of iterations, as in the case of  coplanar data,
     the  best sphere is  returned and  the error flag is  cleared. The 
     average  as well  as the  maximum absolute  radial errors are also
     returned. A typical calling example is:
  
       NL_POINT  *PP;
       NL_INDEX  n;
       NL_REAL   tol, cx, cy, cz, rr, era, erm;
       NL_FLAG   sfl;
       ...
       (get points PP and tolerance tol);
       ...
       N_SphereFit3dPts(PP,n,tol,NL_AVERAGE,&cx,&cy,&cz,&rr,&era,&erm,&sfl);

     In order to  avoid numerical  errors, the  routine  checks if  the 
     points are  coplanar, i.e., if the  maximum distance of the points
     from the best fitting plane is less than "tol".


   ACCESS:
   
     PP       , input  ,  Points in 3-D
     n        , input  ,  Highest index in PP
     tol      , input  ,  Tolerance to check coplanarity
     etp      , input  ,  Flag:
                            NL_MAXIMUM: tol is maximum deviation
                            NL_AVERAGE: tol is average deviation
     cx,cy,cz , output ,  Center of sphere
     rr       , output ,  Radius of sphere
     era,erm  , output ,  Absolute average and maximum radial errors
     sfl      , output ,  Flag:
                            NL_TRUE : sphere computed
                            NL_FALSE: no sphere is computed, eg the points 
                                   are coplanar or degenerate


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SphereFit3dPts( NL_POINT *PP, NL_INDEX n, NL_REAL tol, NL_FLAG etp, NL_REAL *cx, NL_REAL *cy, NL_REAL *cz, NL_REAL *rr, NL_REAL *era, NL_REAL *erm, NL_FLAG *sfl )
{
    NL_PRIVATE NL_STRING rname = _T("N_SphereFit3dPts");

    NL_PRIVATE NL_INDEX itlm = 30;

    NL_FLAG error = NL_NO;

    NL_INDEX ind[4], i, itc;

    NL_REAL ** RM, ** JM, *rhs, sol[4], A, B, C, D, E, F, G, H, I, J, K, L, M, x, y, z, xo, yo, zo, ro, dis, ri, eavp, eavn, eavo, x2, y2, z2, xy, xz, yz, x2y2z2, f, xc, yc, zc, rc, inv, da, dm, dao, dmo;

    NL_POINT CC;

    NL_VECTOR NN;

    NL_RMATRIX rma, rmj;

    NL_STACKS SL;

    /* Initialize system */

    N_InitNurbs( &SL );

    /* See if points are coplanar */

    *sfl = NL_FALSE;

    error = N_PlaneFit3dPts( PP, n, &CC, &NN, era, erm );

    if( error EQ NL_YES )
        NL_OUT;

    switch( etp )
    {
        case NL_MAXIMUM:
            if( *erm LT tol )
                NL_OUT;
            break;

        case NL_AVERAGE:
            if( *era LT tol )
                NL_OUT;
            break;

        default:
            NL_ERROR( NL_CAL_ERR );
    }

    /* Handle special case of n=3 */

    if( n EQ 3 )
    {
        error = N_SphereCenterRadius( PP[0], PP[1], PP[2], PP[3], &CC, rr, sfl );

        if( error EQ NL_YES OR *sfl EQ NL_FALSE )
            NL_OUT;

        N_PtToXYZ( CC, cx, cy, cz );
        *era = 0.0;
        *erm = 0.0;

        NL_OUT;
    }

    /* Get various entities */

    A = B = C = D = E = F = G = H = I = J = K = L = 0.0;
    f = (NL_REAL)n + 1.0;

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( PP[i], &x, &y, &z );

        x2 = x * x;
        y2 = y * y;
        z2 = z * z;
        xy = x * y;
        xz = x * z;
        yz = y * z;
        x2y2z2 = x2 + y2 + z2;

        A += x;
        B += y;
        C += z;
        D += x2;
        E += y2;
        F += z2;
        G += xy;
        H += xz;
        I += yz;
        J += x * x2y2z2;
        K += y * x2y2z2;
        L += z * x2y2z2;
    }

    M = D + E + F;

    /* Solve system of equations */

    error = N_SetRealMatrix( &rma, 3, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &rma, &RM );

    RM[0][0] = 2.0 *D;
    RM[0][1] = 2.0 *G;
    RM[0][2] = 2.0 *H;
    RM[0][3] = A;
    sol[0] = J;
    RM[1][0] = 2.0 *G;
    RM[1][1] = 2.0 *E;
    RM[1][2] = 2.0 *I;
    RM[1][3] = B;
    sol[1] = K;
    RM[2][0] = 2.0 *H;
    RM[2][1] = 2.0 *I;
    RM[2][2] = 2.0 *F;
    RM[2][3] = C;
    sol[2] = L;
    RM[3][0] = 2.0 *A;
    RM[3][1] = 2.0 *B;
    RM[3][2] = 2.0 *C;
    RM[3][3] = f;
    sol[3] = M;

    error = N_RealMatrixLuDecomposePivot( &rma, ind );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixForBackPivot( &rma, ind, sol );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get initial center and radius */

    xc = sol[0];
    yc = sol[1];
    zc = sol[2];
    rc = sqrt( xc * xc + yc * yc + zc * zc + sol[3] );

    /************************************************/
    /* Improve sphere fit by Gauss-Newton iteration */
    /************************************************/

    /* Get average error */

    xo = xc;
    yo = yc;
    zo = zc;
    ro = rc;

    eavp = 0.0;
    dm = -1.0;

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( PP[i], &x, &y, &z );

        ri = sqrt( (x - xc) * (x - xc) + (y - yc) * (y - yc) + (z - zc) * (z - zc) );
        eavp += fabs( rc - ri );

        if( fabs( rc - ri )GT dm )
            dm = fabs( rc - ri );
    }

    if( N_FloatOpIsBad( eavp, f, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );
    eavp /= f;

    dao = eavp;
    dmo = dm;

    if( eavp LT NL_MTOL )
    {
        *cx = xo;
        *cy = yo;
        *cz = zo;
        *era = eavp;
        *erm = dm;
        *rr = ro;
        *sfl = NL_TRUE;

        NL_OUT;
    }

    if( N_FloatOpIsBad( eavp, rc, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );
    eavp /= rc;

    eavo = eavp;

    /* Set up least-squares problem */

    error = N_SetRealMatrix( &rmj, n, 3, NL_MT_FULL, 3, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &rmj, &JM );

    rhs = N_AllocReal1dArray( n, &SL );

    if( rhs EQ NULL )
        NL_QUIT;

    itc = 0;

    while( itc LE itlm )
    {
        /* Set up Jacobian and the right hand side */

        for ( i = 0; i <= n; i++ )
        {
            N_PtToXYZ( PP[i], &x, &y, &z );
            ri = sqrt( (x - xc) * (x - xc) + (y - yc) * (y - yc) + (z - zc) * (z - zc) );

            if( N_FloatOpIsBad( 1.0, ri, NL_DIVISION ) )
                NL_ERROR( NL_NUM_ERR );
            inv = 1.0 / ri;

            JM[i][0] = (xc - x) * inv;
            JM[i][1] = (yc - y) * inv;
            JM[i][2] = (zc - z) * inv;
            JM[i][3] = -1.0;
            rhs[i] = rc - ri;
        }

        /* Solve by least-squares */

        error = N_RealMatrixLstSqSolve( &rmj, rhs, sol );

        if( error EQ NL_YES )
        {
            N_ErrClear();
            error = NL_NO;
            break;
        }

        xc += sol[0];
        yc += sol[1];
        zc += sol[2];
        rc += sol[3];

        /* Recompute the average error */

        eavn = 0.0;
        dm = -1.0;

        for ( i = 0; i <= n; i++ )
        {
            N_PtToXYZ( PP[i], &x, &y, &z );

            ri = sqrt( (x - xc) * (x - xc) + (y - yc) * (y - yc) + (z - zc) * (z - zc) );
            eavn += fabs( rc - ri );

            if( fabs( rc - ri )GT dm )
                dm = fabs( rc - ri );
        }

        if( N_FloatOpIsBad( eavn, f, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        eavn /= f;

        da = eavn;

        if( N_FloatOpIsBad( eavn, rc, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        eavn /= rc;

        /* Check for convergence and save best result */

        dis = sqrt( sol[0] * sol[0] + sol[1] * sol[1] + sol[2] * sol[2] + sol[3] * sol[3] );

        if( N_FloatOpIsBad( dis, rc, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );
        dis /= rc;

        /* Best result is with the smallest average error */

        if( eavn LT eavo )
        {
            xo = xc;
            yo = yc;
            zo = zc;
            ro = rc;
            dao = da;
            dmo = dm;
            eavo = eavn;
        }

        /* If average error does not change, out */

        if( fabs( eavp - eavn )LT tol )
            break;

        /* If center does not move, out */

        if( dis LT tol )
            break;

        eavp = eavn;
        itc++;
    }

    /* Get the output */

    *cx = xo;
    *cy = yo;
    *cz = zo;
    *rr = ro;
    *era = dao;
    *erm = dmo;
    *sfl = NL_TRUE;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SphereFit3dPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes a bounding  rectangle of a 2-D point
     set. The method  minimizes a  univariate function that computes the
     area of the  bounding box as a  function of  the rotation  angle. A  
     typical calling example is:
  
       NL_POINT   *P, R1, R2, R3, R4;
       NL_REAL    tpc, alf;
       NL_INDEX   n;
       ...
       (get points P and tpc);
       ...
       N_BoundRect2dPts(P,n,NL_YES,tpc,&R1,&R2,&R3,&R4,&alf);


   ACCESS:
   
     P            , input  ,  Points
     n            , input  ,  Highest index in P
     cen          , input  ,  Flag:
                                NL_YES: points  are   centered  around  the 
                                     global origin
                                NL_NO:  center points around the origin
     tpc          , input  ,  Tolerance  expressed in  percentage, e.g., 
                              tpc = 0.1  gives a  bounding  box  that is
                              about 1 degree  off the  absolute minimum.
                              In general, the  tolerance is set as "tpc"
                              times the  estimated  minimum  box area. A
                              good  default is  anywhere between 0.1 and
                              0.01.
     R1,R2,R3,R4  , output ,  Vertices of bounding  rectangle  listed in
                              counterclockwise order
     alf          , output ,  Rotation  angle  between the  bounding box 
                              and the  rectangle <R1,R2,R3,R4>. That is,
                              the best bounding rectangle is obtained by
                              rotating the  point set by alf, and compu-
                              ting  the  bounding  box  of  the  rotated 
                              points.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_BoundRect2dPts( NL_POINT *P, NL_INDEX n, NL_FLAG cen, NL_REAL tpc, NL_POINT *R1, NL_POINT *R2, NL_POINT *R3, NL_POINT *R4, NL_REAL *alf )
{
    NL_PRIVATE NL_STRING rname = _T("N_BoundRect2dPts");

    NL_FLAG error = NL_NO;

    NL_INDEX i, imin, kl, kr;

    NL_REAL rot[12], RA[12], Amin, rmin, x, y, z, xc, yc, xl, xr, yb, yt, inv, n1r, ang, ca, sa, rtol;

    NL_VECTOR V;

    NL_STACKS SL;

    /* Initialize */

    N_InitNurbs( &SL );

    /* Get some entities */

    n1r = (NL_REAL)n + 1.0;

    if( N_FloatOpIsBad( 1.0, n1r, NL_DIVISION ) )
        NL_ERROR( NL_NUM_ERR );
    inv = 1.0 / n1r;

    ST_mmxaps_xp = N_AllocReal1dArray( n, &SL );

    if( ST_mmxaps_xp EQ NULL )
        NL_QUIT;

    ST_mmxaps_yp = N_AllocReal1dArray( n, &SL );

    if( ST_mmxaps_yp EQ NULL )
        NL_QUIT;

    ST_mmxaps_gn = n;

    /* Center points around the global origin */

    xc = yc = 0.0;

    if( cen EQ NL_NO )
    {
        for ( i = 0; i <= n; i++ )
        {
            N_PtToXYZ( P[i], &ST_mmxaps_xp[i], &ST_mmxaps_yp[i], &z );

            xc += ST_mmxaps_xp[i];
            yc += ST_mmxaps_yp[i];
        }

        xc *= inv;
        yc *= inv;

        for ( i = 0; i <= n; i++ )
        {
            ST_mmxaps_xp[i] -= xc;
            ST_mmxaps_yp[i] -= yc;
        }
    }
    else
    {
        for ( i = 0; i <= n; i++ )
            N_PtToXYZ( P[i], &ST_mmxaps_xp[i], &ST_mmxaps_yp[i], &z );
    }

    /* Find the minimum angle of rotation: (1) find an approximate minimum */

    rot[1] = 0.0;
    rot[2] = 10.0;
    rot[3] = 20.0;
    rot[4] = 30.0;
    rot[5] = 40.0;
    rot[6] = 50.0;
    rot[7] = 60.0;
    rot[8] = 70.0;
    rot[9] = 80.0;
    rot[10] = 90.0;

    Amin = NL_BIGD;
    imin = 1;

    for ( i = 1; i <= 10; i++ )
    {
        RA[i] = ST_mmxaps( rot[i] );

        if( RA[i]LT Amin )
        {
            Amin = RA[i];
            imin = i;
        }
    }

    /* Find the minimum angle of rotation: (2) bracket the minimum */

    kl = kr = -1;

    for ( i = imin - 1; i >= 1; i-- )
    {
        if( RA[i]GT Amin )
        {
            kl = i;
            break;
        }
    }

    if( kl EQ - 1 )
    {
        rot[0] = -10.0;

        RA[0] = ST_mmxaps( rot[0] );

        if( RA[0]GT Amin )
            kl = 0;
    }

    if( kl GT - 1 )
    {
        for ( i = imin + 1; i <= 10; i++ )
        {
            if( RA[i]GT Amin )
            {
                kr = i;
                break;
            }
        }

        if( kr EQ - 1 )
        {
            rot[11] = 100.0;

            RA[11] = ST_mmxaps( rot[11] );

            if( RA[11]GT Amin )
                kr = 11;
        }
    }

    /* Find the minimum angle of rotation: (3) compute the minimum */

    rtol = tpc * Amin;

    if( kl EQ - 1 OR kr EQ - 1 )
    {
        rmin = rot[imin];
    }
    else
    {
        N_FuncFindMinima( rot[kl], rot[imin], rot[kr], Amin, ST_mmxaps, rtol, 0.0, &rmin, &ca );
    }

    /* Rotate the point set with the minimum angle and get the bounding box */

    ang = rmin * NL_RAD;
    ca = cos( ang );
    sa = sin( ang );

    xl = xr = ST_mmxaps_xp[0] * ca - ST_mmxaps_yp[0] * sa;
    yb = yt = ST_mmxaps_xp[0] * sa + ST_mmxaps_yp[0] * ca;

    for ( i = 1; i <= n; i++ )
    {
        x = ST_mmxaps_xp[i] * ca - ST_mmxaps_yp[i] * sa;
        y = ST_mmxaps_xp[i] * sa + ST_mmxaps_yp[i] * ca;

        if( x LT xl )
            xl = x;

        if( x GT xr )
            xr = x;

        if( y LT yb )
            yb = y;

        if( y GT yt )
            yt = y;
    }

    /* Now rotate back to get the vertices of the output box */

    x = xl * ca + yb * sa;
    y = -xl * sa + yb * ca;
    N_VectorCreate( x, y, 0.0, R1 );

    x = xr * ca + yb * sa;
    y = -xr * sa + yb * ca;
    N_VectorCreate( x, y, 0.0, R2 );

    x = xr * ca + yt * sa;
    y = -xr * sa + yt * ca;
    N_VectorCreate( x, y, 0.0, R3 );

    x = xl * ca + yt * sa;
    y = -xl * sa + yt * ca;
    N_VectorCreate( x, y, 0.0, R4 );

    /* Translate the box back if centering was necessary */

    if( cen EQ NL_NO )
    {
        N_VectorCreate( xc, yc, 0.0, &V );
        N_VectorSum( V, *R1, R1 );
        N_VectorSum( V, *R2, R2 );
        N_VectorSum( V, *R3, R3 );
        N_VectorSum( V, *R4, R4 );
    }

    *alf = rmin;

    /* Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_BoundRect2dPts */

/**********************************************************************/
/* ST_mmxaps: Area of min-max box of rotated point set                 */
/**********************************************************************/

NL_REAL ST_mmxaps( NL_REAL alf )
{

    NL_INDEX i;

    NL_REAL xl, xr, yb, yt, ca, sa, ang, x, y;

    if( alf EQ 0.0 OR alf EQ 90.0 )
    {
        xl = xr = ST_mmxaps_xp[0];
        yb = yt = ST_mmxaps_yp[0];

        for ( i = 1; i <= ST_mmxaps_gn; i++ )
        {
            if( ST_mmxaps_xp[i]LT xl )
                xl = ST_mmxaps_xp[i];

            if( ST_mmxaps_xp[i]GT xr )
                xr = ST_mmxaps_xp[i];

            if( ST_mmxaps_yp[i]LT yb )
                yb = ST_mmxaps_yp[i];

            if( ST_mmxaps_yp[i]GT yt )
                yt = ST_mmxaps_yp[i];
        }
    }
    else
    {
        ang = alf * NL_RAD;
        ca = cos( ang );
        sa = sin( ang );

        xl = xr = ST_mmxaps_xp[0] * ca - ST_mmxaps_yp[0] * sa;
        yb = yt = ST_mmxaps_xp[0] * sa + ST_mmxaps_yp[0] * ca;

        for ( i = 1; i <= ST_mmxaps_gn; i++ )
        {
            x = ST_mmxaps_xp[i] * ca - ST_mmxaps_yp[i] * sa;
            y = ST_mmxaps_xp[i] * sa + ST_mmxaps_yp[i] * ca;

            if( x LT xl )
                xl = x;

            if( x GT xr )
                xr = x;

            if( y LT yb )
                yb = y;

            if( y GT yt )
                yt = y;
        }
    }

    return ((xr - xl) * (yt - yb));
}

/*******************************************************************//**


   DESCRIPTION:

     This  geometry routine  computes the  center and  the  radius of a 
     sphere  defined by  four  non-coplanar  points. If the  points are
     coplanar,  the  indicator  flag "sfl" is set  to  NL_FALSE. A typical 
     calling example is:

       NL_POINT  A, B, C, D, E;
       NL_REAL   r;
       NL_FLAG   sfl;
       ...
       (get points A, B, C, and D);
       ...
       N_SphereCenterRadius(A,B,C,D,&E,&r,&sfl);


   ACCESS:
   
     A,B,C,D , input  ,  Points on the sphere
     E       , output ,  Center of sphere
     r       , output ,  Radius of sphere
     sfl     , output ,  Flag:
                           NL_TRUE : sphere computed
                           NL_FALSE: no  sphere  computed, e.g. the points 
                                  are coplanar


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SphereCenterRadius( NL_POINT A, NL_POINT B, NL_POINT C, NL_POINT D, NL_POINT *E, NL_REAL *r, NL_FLAG *sfl )
{

    NL_FLAG ifl, cfl, error;

    NL_REAL ra, rb, rc, rd, t1, t2, rr, third, sixth;

    NL_POINT Oabc, Oabd, Obcd, Oacd, P, PS, CC;

    NL_VECTOR Nabc, Nabd, Nbcd, Nacd, V1, V2, V3, N1, N2, N3, VS;

    NL_LINESEG labc, labd, lbcd, lacd;

    NL_PLANE pabc, pabd, pbcd, pacd;

    /* Get the four planes of the tetrahedron <A,B,C,D>  */

    *sfl = NL_FALSE;
    third = 1.0 / 3.0;
    sixth = 1.0 / 6.0;

    error = N_CalcCircCenterAndRadius( A, B, C, &Oabc, &ra, &cfl );

    if( error EQ NL_YES OR cfl EQ NL_FALSE )
        NL_OUT;

    error = N_CalcCircCenterAndRadius( A, B, D, &Oabd, &rb, &cfl );

    if( error EQ NL_YES OR cfl EQ NL_FALSE )
        NL_OUT;

    error = N_CalcCircCenterAndRadius( B, C, D, &Obcd, &rc, &cfl );

    if( error EQ NL_YES OR cfl EQ NL_FALSE )
        NL_OUT;

    error = N_CalcCircCenterAndRadius( A, C, D, &Oacd, &rd, &cfl );

    if( error EQ NL_YES OR cfl EQ NL_FALSE )
        NL_OUT;

    N_VectorDiff( B, A, &V1 );
    N_VectorDiff( C, B, &V2 );
    N_VectorDiff( A, C, &V3 );
    N_VectorCross( V1, V2, &N1 );
    N_VectorCross( V2, V3, &N2 );
    N_VectorCross( V3, V1, &N3 );

    error = N_VectorNormalizeRef( &N1 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &N2 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &N3 );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCreate( 0.0, 0.0, 0.0, &VS );
    N_VectorSum( VS, N1, &VS );
    N_VectorSum( VS, N2, &VS );
    N_VectorSum( VS, N3, &VS );
    N_VectorScale( VS, third, &Nabc );

    N_VectorDiff( B, A, &V1 );
    N_VectorDiff( D, B, &V2 );
    N_VectorDiff( A, D, &V3 );
    N_VectorCross( V1, V2, &N1 );
    N_VectorCross( V2, V3, &N2 );
    N_VectorCross( V3, V1, &N3 );

    error = N_VectorNormalizeRef( &N1 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &N2 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &N3 );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCreate( 0.0, 0.0, 0.0, &VS );
    N_VectorSum( VS, N1, &VS );
    N_VectorSum( VS, N2, &VS );
    N_VectorSum( VS, N3, &VS );
    N_VectorScale( VS, third, &Nabd );

    N_VectorDiff( C, B, &V1 );
    N_VectorDiff( D, C, &V2 );
    N_VectorDiff( B, D, &V3 );
    N_VectorCross( V1, V2, &N1 );
    N_VectorCross( V2, V3, &N2 );
    N_VectorCross( V3, V1, &N3 );

    error = N_VectorNormalizeRef( &N1 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &N2 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &N3 );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCreate( 0.0, 0.0, 0.0, &VS );
    N_VectorSum( VS, N1, &VS );
    N_VectorSum( VS, N2, &VS );
    N_VectorSum( VS, N3, &VS );
    N_VectorScale( VS, third, &Nbcd );

    N_VectorDiff( C, A, &V1 );
    N_VectorDiff( D, C, &V2 );
    N_VectorDiff( A, D, &V3 );
    N_VectorCross( V1, V2, &N1 );
    N_VectorCross( V2, V3, &N2 );
    N_VectorCross( V3, V1, &N3 );

    error = N_VectorNormalizeRef( &N1 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &N2 );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VectorNormalizeRef( &N3 );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCreate( 0.0, 0.0, 0.0, &VS );
    N_VectorSum( VS, N1, &VS );
    N_VectorSum( VS, N2, &VS );
    N_VectorSum( VS, N3, &VS );
    N_VectorScale( VS, third, &Nacd );

    N_CreatePlanePtNormal( &pabc, Oabc, Nabc );
    N_CreatePlanePtNormal( &pabd, Oabd, Nabd );
    N_CreatePlanePtNormal( &pbcd, Obcd, Nbcd );
    N_CreatePlanePtNormal( &pacd, Oacd, Nacd );

    /* Check coplanarity */

    error = N_DistPtPlane( pabc, D, &rd );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtPlane( pabd, C, &rc );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtPlane( pbcd, A, &ra );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_DistPtPlane( pacd, B, &rb );

    if( error EQ NL_YES )
        NL_OUT;

    if( ra LT NL_MTOL OR rb LT NL_MTOL OR rc LT NL_MTOL OR rd LT NL_MTOL )
        NL_OUT;

    /* Get center and radius */

    N_CreateLineStartDirVector( &labc, Oabc, Nabc, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &labd, Oabd, Nabd, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &lbcd, Obcd, Nbcd, NL_UNBOUNDED );
    N_CreateLineStartDirVector( &lacd, Oacd, Nacd, NL_UNBOUNDED );

    N_VectorCreate( 0.0, 0.0, 0.0, &PS );

    error = N_IsectLineLine( labc, labd, &P, &t1, &t2, &ifl );

    if( error EQ NL_YES OR ifl EQ NL_FALSE )
        NL_OUT;
    N_VectorSum( PS, P, &PS );

    error = N_IsectLineLine( labc, lbcd, &P, &t1, &t2, &ifl );

    if( error EQ NL_YES OR ifl EQ NL_FALSE )
        NL_OUT;
    N_VectorSum( PS, P, &PS );

    error = N_IsectLineLine( labc, lacd, &P, &t1, &t2, &ifl );

    if( error EQ NL_YES OR ifl EQ NL_FALSE )
        NL_OUT;
    N_VectorSum( PS, P, &PS );

    error = N_IsectLineLine( labd, lbcd, &P, &t1, &t2, &ifl );

    if( error EQ NL_YES OR ifl EQ NL_FALSE )
        NL_OUT;
    N_VectorSum( PS, P, &PS );

    error = N_IsectLineLine( labd, lacd, &P, &t1, &t2, &ifl );

    if( error EQ NL_YES OR ifl EQ NL_FALSE )
        NL_OUT;
    N_VectorSum( PS, P, &PS );

    error = N_IsectLineLine( lbcd, lacd, &P, &t1, &t2, &ifl );

    if( error EQ NL_YES OR ifl EQ NL_FALSE )
        NL_OUT;
    N_VectorSum( PS, P, &PS );

    N_VectorScale( PS, sixth, &CC );
    N_DistPtPt( CC, A, &ra );
    N_DistPtPt( CC, B, &rb );
    N_DistPtPt( CC, C, &rc );
    N_DistPtPt( CC, D, &rd );

    rr = 0.25 *(ra + rb + rc + rd);

    /* Create the output */

    N_VectorCopy( CC, E );

    *r = rr;
    *sfl = NL_TRUE;

    /* Exit */

    EXIT:

    return (error);
} /* end N_SphereCenterRadius */
