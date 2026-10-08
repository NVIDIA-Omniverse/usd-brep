// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**************************************************************************/
/* FuncsBasic.c : Basic Function Definitions that act on NL_CFUN objects  */
/**************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>



/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if a curve function structure has enough
     memory to store given control values and knots. A  typical calling
     example is:

       NL_CFUN    cfn;
       NL_INDEX   nf, mk;
       NL_STRING  rname;
       ...
       (define cfn and get nf, mk and rname);
       ...
       N_CrvIsFuncSized(&cfn,nf,mk,rname);


   ACCESS:
   
     cfn   , input ,  NURBS curve structure
     nf    , input ,  Expected highest index in fu
     mk    , input ,  Expected highest index in U
     rname , input ,  Routine name error is checked in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvIsFuncSized( NL_CFUN *cfn, NL_INDEX nf, NL_INDEX mk, NL_STRING rname )
{
    NL_FLAG error = NL_NO;
    NL_INDEX n, m;

    /* Get local notation */
    N_CFuncGetArraySizes( cfn, &n, &m );

    /* Check storage */
    if( n LT nf OR m LT mk )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */
    return (error);
} /* end N_CrvIsFuncSized */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store members of a curve
     value structure. Proper  error check is performed in case 
     memory allocation fails. A typical calling example is:

       NL_CVALUE  *cvl;
       NL_STACKS  S;
       ...
       cvl = N_AllocCValue(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     cvl  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CVALUE *N_AllocCValue( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCValue");

    NL_CVALUE *cvl;

    NL_CVLNODE *cvd;

    /* Allocate memory for the structure */

    cvl = (NL_CVALUE *)N_Malloc( sizeof( NL_CVALUE ) );

    if( cvl EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    cvd = (NL_CVLNODE *)N_Malloc( sizeof( NL_CVLNODE ) );

    if( cvd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cvl );
        cvl = NULL;
        return NULL;
    }

    cvd->ptr = cvl;
    cvd->next = S->cvl;
    S->cvl = cvd;

    /* Exit */

    return cvl;
} /* end N_AllocCValue */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to store  a curve  value object. 
     Proper error check is performed in case memory allocation fails. 
     A typical calling example is:

       NL_CVALUE  *cvs;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get n);
       ...
       cvs = N_AllocCValueAndArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in control value array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     cvs  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CVALUE *N_AllocCValueAndArray( NL_INDEX n, NL_STACKS *S )
{
    NL_REAL *fu;

    NL_CVALUE *cvs;

    /* Allocate memory */

    cvs = N_AllocCValue( S );

    if( cvs EQ NULL )
        return NULL;

    fu = N_AllocReal1dArray( n, S );

    if( fu EQ NULL )
        return NULL;

    /* Build the object */

    N_CValueFromArray( cvs, fu, n );

    /* Exit */

    return cvs;
} /* end N_AllocCValueAndArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine deallocates  memory that stores  members of a 
     control value structure. Given a control value pointer, the routine
     searches  for the  pointer  on the  memory  stack. It it  is found, 
     memory is deallocated. If not, the  routine does nothing. A typical
     calling example is:

       NL_CVALUE  *cvl;
       NL_STACKS   S;
       ...
       N_FreeCValue(cvl,&S);


   ACCESS:
   
     cvl , input  ,  Control value pointer 
     S   , input  ,  cvl's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeCValue( NL_CVALUE *cvl, NL_STACKS *S )
{
    NL_CVLNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->cvl NEQ NULL )
    {
        prev = S->cvl;
        curr = S->cvl;

        while( curr NEQ NULL AND curr->ptr NEQ cvl )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->cvl = NULL;
            }
            else /* More than one node */
            {
                S->cvl = S->cvl->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
        }
    }
} /* end N_FreeCValue */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine makes  proper pointer assignments to define a 
     curve function control value  object from a set of function values.  
     Memory for the control value structure is allocated in the  calling 
     routine; only the pointer is passed down. A typical calling example 
     is:

       NL_CVALUE  cvl;
       NL_REAL    *fu;
       NL_INDEX   n;
       ...
       (allocate memory for fu);
       ...
       N_CValueFromArray(&cvl,fu,n);


   ACCESS:
   
     cvl , in/out ,  Control value structure
     fu  , input  ,  Function values
     n   , input  ,  Highest index in fu


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CValueFromArray( NL_CVALUE *cvl, NL_REAL *fu, NL_INDEX n )
{
    cvl->n = n;
    cvl->fu = fu;
} /* end N_CValueFromArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store an array of curve functions 
     each  defined by the same set of parameters  <n,p,m>. Proper error 
     check  is  performed in  case  memory allocation fails. A  typical 
     calling example is:

       NL_CFUN    **cf2;
       NL_INDEX   n, m, k;
       NL_DEGREE  p;
       NL_STACKS  S;
       ...
       (get n, p, m and k);
       ...
       cf2 = N_AllocCrvFuncArray(n,p,m,k,&S);


   ACCESS:
   
     n  , input  ,  Highest index in control value arrays
     p  , input  ,  Degree of each function
     m  , input  ,  Highest index in knot vector arrays
     k  , input  ,  Highest index of function  array cf2[0],...,cf2[k];
                    cf2[i], 0<=i<=k, is a pointer to the i-th function.
     S  , input  ,  Memory stacks pointer


   RETURN CODES:

     cf2  : Pointer to array of functions if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CFUN ** N_AllocCrvFuncArray( NL_INDEX n, NL_DEGREE p, NL_INDEX m, NL_INDEX k, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCrvFuncArray");

    NL_INDEX i;

    NL_CFUN ** cf2;

    NL_CF2NODE *f2d;

    /* Allocate memory for function pointer arrays */

    cf2 = (NL_CFUN ** )N_Malloc( (k + 1) * sizeof( NL_CFUN * ) );

    if( cf2 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Allocate memory for each function in the array */

    for ( i = 0; i <= k; i++ )
    {
        cf2[i] = N_AllocCrvFunc( n, p, m, S );

        if( cf2[i]EQ NULL )
        {
            N_Free( cf2 );
            cf2 = NULL;
            return NULL;
        }
    }

    /* Put pointer on memory stack */

    f2d = (NL_CF2NODE *)N_Malloc( sizeof( NL_CF2NODE ) );

    if( f2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cf2 );
        cf2 = NULL;
        return NULL;
    }

    f2d->ptr = cf2;
    f2d->next = S->cf2;
    S->cf2 = f2d;

    /* Exit */

    return cf2;
} /* end N_AllocCrvFuncArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a curve function defined by 
     the usual parameters <n,p,m>. Proper error checks are performed in 
     case memory allocations fail. A typical calling example is:

       NL_CFUN    *cfn;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       NL_STACKS  S;
       ...
       (get n, p and m);
       ...
       cfn = N_AllocCrvFunc(n,p,m,&S);


   ACCESS:
   
     n  , input  ,  Highest index in control value array
     p  , input  ,  Degree
     m  , input  ,  Highest index in knot vector array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     cfn  : Pointer to funcion if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CFUN *N_AllocCrvFunc( NL_INDEX n, NL_DEGREE p, NL_INDEX m, NL_STACKS *S )
{
    NL_CVALUE *cvs;

    NL_KNOTVECTOR *knt;

    NL_CFUN *cfn;

    /* Allocate memory */

    cvs = N_AllocCValueAndArray( n, S );

    if( cvs EQ NULL )
        return NULL;

    knt = N_AllocKnotVectorAndArray( m, S );

    if( knt EQ NULL )
        return NULL;

    cfn = N_AllocCrvFuncStack( S );

    if( cfn EQ NULL )
        return NULL;

    /* Build curve function structure */

    N_CFuncFromKnotVector( cfn, cvs, p, knt );

    /* Exit */

    return cfn;
} /* end N_AllocCrvFunc */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store members of a curve function
     structure. Error  check is  performed in  case  memory  allocation 
     fails. A typical calling example is:

       NL_CFUN    *cfs;
       NL_STACKS  S;
       ...
       cfs = N_AllocCrvFuncStack(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     cfs  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CFUN *N_AllocCrvFuncStack( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCrvFuncStack");

    NL_CFUN *cfs;

    NL_CFNNODE *cfd;

    /* Allocate memory for the structure */

    cfs = (NL_CFUN *)N_Malloc( sizeof( NL_CFUN ) );

    if( cfs EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    cfd = (NL_CFNNODE *)N_Malloc( sizeof( NL_CFNNODE ) );

    if( cfd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cfs );
        cfs = NULL;
        return NULL;
    }

    cfd->ptr = cfs;
    cfd->next = S->cfn;
    S->cfn = cfd;

    /* Exit */

    return cfs;
} /* end N_AllocCrvFuncStack */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine   deallocates  memory  that stores  a  curve 
     function, i.e. control value structure, control values, knot vector 
     structure and knots. IT DOES  NOT DEALLOCATE MEMORY THAT STORES THE 
     NL_CURVE NL_FUNCTION STRUCTURE ITSELF. A typical calling example is:

       NL_CFUN    *cfn;
       NL_STACKS  S;
       ...
       N_FreeCrvFunc(cfn,&S);


   ACCESS:
   
     cfn , input  ,  Curve function pointer
     S   , input  ,  cfn's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeCrvFunc( NL_CFUN *cfn, NL_STACKS *S )
{
    NL_DEGREE p;

    NL_REAL *fu, *U;

    NL_CVALUE *cvl;

    NL_KNOTVECTOR *knt;

    /* Get locals */

    N_CFuncGetArrayAndKnotVector( cfn, &cvl, &p, &knt );
    N_CrvFuncCntrlValKnots( cfn, &fu, &U );

    /* Kill curve function constituents */

    N_FreeCValue( cvl, S );
    N_FreeReal1dArray( fu, S );
    N_FreeKnotVector( knt, S );
    N_FreeReal1dArray( U, S );
} /* end N_FreeCrvFunc */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine makes  proper pointer  assignments  to define
     a curve function object from control value and knot vector objects. 
     Memory  is  allocated in  the  calling  routine; only  pointers are 
     passed down. A typical calling example is:

       NL_CFUN        cfn;
       NL_CVALUE      cvl;
       NL_DEGREE      p;
       NL_KNOTVECTOR  knt;
       ...
       (define cvl and knt);
       ...
       N_CFuncFromKnotVector(&cfn,&cvl,p,&knt);


   ACCESS:
   
     cfn , in/out ,  NURBS curve function
     cvl , input  ,  Control value object
     p   , input  ,  Degree
     knt , input  ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CFuncFromKnotVector( NL_CFUN *cfn, NL_CVALUE *cvl, NL_DEGREE p, NL_KNOTVECTOR *knt )
{
    cfn->cvl = cvl;
    cfn->p = p;
    cfn->knt = knt;
} /* end N_CFuncFromKnotVector */

/*******************************************************************//**


   DESCRIPTION:

     This routine generates a curve  function  object given function 
     values  and knots. It  allocates  memory to  store the  control  
     value  and  knot  vector  objects,  and  makes  proper  pointer  
     assignments to create the curve function object. Memory for the  
     curve function structure is  allocated in  the calling routine. 
     A typical calling example is:

       NL_CFUN    cfn;
       NL_REAL    *fu, *U;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       NL_STACKS  S;
       ...
       (allocate memory for fu and U);
       ...
       N_CFuncFromKnots(&cfn,fu,n,p,U,m,&S);


   ACCESS:
   
     cfn , in/out ,  NURBS curve function
     fu  , input  ,  Function values
     n   , input  ,  Highest index in fu
     p   , input  ,  Degree
     U   , input  ,  Knots
     m   , input  ,  Highest index in U
     S   , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CFuncFromKnots( NL_CFUN *cfn, NL_REAL *fu, NL_INDEX n, NL_DEGREE p, NL_REAL *U, NL_INDEX m, NL_STACKS *S )
{
    NL_CVALUE *cvl;

    NL_KNOTVECTOR *knt;

    /* Allocate memory */

    cvl = N_AllocCValue( S );

    if( cvl EQ NULL )
        return (1);

    knt = N_AllocKnotVector( S );

    if( knt EQ NULL )
        return (1);

    /* Make pointer assignments */

    N_CValueFromArray( cvl, fu, n );
    N_KnotVectorFromRealArray( knt, U, m );
    N_CFuncFromKnotVector( cfn, cvl, p, knt );

    /* Exit */

    return (0);
} /* end N_CFuncFromKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a  curve function  is initialized 
     to NULL. If yes, NL_TRUE is returned. Otherwise, NL_FALSE is returned.  
     This routine is used to check if memory allocation is needed.  A 
     typical calling example is:

       NL_CFUN  cfn;
       ...
       if( N_CrvAreFuncArraysNULL(&cfn) )  --> allocate memory;
     

   ACCESS:
   
     cfn , input ,  NURBS curve function


   RETURN CODES:

     NL_TRUE:  Function is initialized to NULL (need memory)
     NL_FALSE: Function is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

NL_BOOLEAN N_CrvAreFuncArraysNULL( NL_CFUN *cfn )
{
    NL_DEGREE p;

    NL_CVALUE *cvl;

    NL_KNOTVECTOR *knt;

    /* Get local notation */

    N_CFuncGetArrayAndKnotVector( cfn, &cvl, &p, &knt );

    /* Check initialization */

    if( cvl EQ NULL OR p EQ - 1 OR knt EQ NULL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_CrvAreFuncArraysNULL */


/*******************************************************************//**


   DESCRIPTION:

     Given a  curve  function object, this routine  allocates memory to 
     store control values and knots. A typical calling example is:

       NL_CFUN    cfn;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       NL_STACKS  S;
       ...
       (get n, p and m);
       ...
       N_AllocCFuncArrays(&cfn,n,p,m,&S);
 
     Since  the  declaration  "NL_CFUN cfn"  defines  the  data  type  and 
     allocates memory, memory is needed to store only the control value 
     and knot vector objects.


   ACCESS:
   
     cfn , in/out ,  NURBS curve function
     n   , input  ,  Highest index in fu
     p   , input  ,  Degree
     m   , input  ,  Highest index in U
     S   , input  ,  cfn's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AllocCFuncArrays( NL_CFUN *cfn, NL_INDEX n, NL_DEGREE p, NL_INDEX m, NL_STACKS *S )
{
    NL_CVALUE *cvl;

    NL_KNOTVECTOR *knt;

    /* Allocate memory */

    cvl = N_AllocCValueAndArray( n, S );

    if( cvl EQ NULL )
        return (1);

    knt = N_AllocKnotVectorAndArray( m, S );

    if( knt EQ NULL )
        return (1);

    /* Build curve function structure */

    N_CFuncFromKnotVector( cfn, cvl, p, knt );

    /* Exit */

    return (0);
} /* end N_AllocCFuncArrays */



/*******************************************************************//**


   DESCRIPTION:

     This  utility routine  initializes a  curve function  structure by 
     setting pointers to NULL and the degree to -1. It is used to check 
     if  memory allocation  is needed, i.e.  if the  curve  function is 
     initialized  to  NULL,  memory is  allocated to  hold the  control 
     value  and  knot  vector  objects. Otherwise  it is  assumed  that 
     memory has already been allocated. A typical calling example is:

       NL_CFUN  cfn;
       ...
       N_CFuncInitArrays(&cfn);
     

   ACCESS:
   
     cfn , in/out ,  NURBS curve function


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CFuncInitArrays( NL_CFUN *cfn )
{
    cfn->cvl = NULL;
    cfn->p = -1;
    cfn->knt = NULL;
} /* end N_CFuncInitArrays */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets parameters to complete curve function 
     definition. A typical calling example is:

       NL_CFUN    cfn;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       ...
       N_CFuncSetSizeIndices(&cfn,n,p,m);


   ACCESS:
   
     cfn , in/out ,  NURBS curve function
     n   , input  ,  Highest index in fu array
     p   , input  ,  Degree
     m   , input  ,  Highest index in knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CFuncSetSizeIndices( NL_CFUN *cfn, NL_INDEX n, NL_DEGREE p, NL_INDEX m )
{
    cfn->cvl->n = n;
    cfn->p = p;
    cfn->knt->m = m;
} /* end N_CFuncSetSizeIndices */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine  checks if memory is  needed to store a curve
     function. If the function is initialized to NULL (via N_CFuncInitArrays()),
     memory is allocated. If  not, the routine checkes if enough memory 
     is available. A typical calling example is:

       NL_CFUN    cfn;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       NL_STRING  rname;
       NL_STACKS  S;
       ...
       (get n, p, m and rname);
       ...
       N_CFuncInitArrays(&cfn);
       N_CFuncSizeArrays(&cfn,n,p,m,rname,&S);

     IT IS  ASSUMED THAT MEMORY TO STORE THE  NL_CURVE  NL_FUNCTION STRUCTURE 
     IS  ALLOCATED  IN  THE  CALLING  ROUTINE. THIS  ROUTINE RESETS THE
     HIGHEST INDEXES TO n AND m.


   ACCESS:
   
     cfn   , in/out ,  NURBS curve function 
     n,p,m , input  ,  Usual parameters
     rname , input  ,  Routine name
     S     , input  ,  cfn's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CFuncSizeArrays( NL_CFUN *cfn, NL_INDEX n, NL_DEGREE p, NL_INDEX m, NL_STRING rname, NL_STACKS *S )
{
    NL_FLAG error;

    /* See if memory is needed */

    if( N_CrvAreFuncArraysNULL( cfn ) )
    {
        error = N_AllocCFuncArrays( cfn, n, p, m, S );

        if( error EQ NL_YES )
            return (1);
    }
    else
    {
        error = N_CrvIsFuncSized( cfn, n, m, rname );

        if( error EQ NL_YES )
            return (1);

        N_CFuncSetSizeIndices( cfn, n, p, m );
    }

    /* Exit */

    return (0);
} /* end N_CFuncSizeArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine compacts control value and knot vector arrays. 
     That is, given a curve function with  control value and knot vector 
     arrays larger than required. This routine redefines these arrays to
     the  appropriate sizes  making the  curve function  definition more 
     memory efficient. A typical calling example is:

       NL_CFUN    cfn;
       NL_STACKS  SG;
       ...
       (define cfn);
       ...
       N_CrvFuncCompact(&cfn,&SG);

     SG  MUST BE  cfn's STACK, I.E.  ALL MEMORY  ALLOCATED FOR  cfn MUST 
     RESIDE ON SG.


   ACCESS:
   
     cfn , in/out ,  Curve function
     SG  , input  ,  cfn's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncCompact( NL_CFUN *cfn, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, n, m;

    NL_REAL *U, *V, *fu, *fv;

    /* Get function data */

    N_CFuncGetArraySizes( cfn, &n, &m );
    N_CrvFuncCntrlValKnots( cfn, &fu, &U );

    /* Allocate memory for new control value and knot vector arrays */

    fv = N_AllocReal1dArray( n, SG );

    if( fv EQ NULL )
        NL_QUIT;

    V = N_AllocReal1dArray( m, SG );

    if( V EQ NULL )
        NL_QUIT;

    /* Copy control values and knots */

    for ( i = 0; i <= n; i++ )
        fv[i] = fu[i];

    for ( i = 0; i <= m; i++ )
        V[i] = U[i];

    /* Redefine function and kill old memory */

    N_CrvFuncSetPtrs( cfn, fv, V );

    N_FreeReal1dArray( fu, SG );
    N_FreeReal1dArray( U, SG );

    /* Exit */

    EXIT:

    return (error);
} /* end N_CrvFuncCompact */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine copies a given function into another function.
     A typical calling example is:

       NL_CFUN    cfnP, cfnQ;
       NL_STACKS  S;
       ...
       (define cfnP);
       ...
       N_CFuncInitArrays(&cfnQ);
       N_CrvFuncCopy(&cfnP,&cfnQ,&S);

     If the  function is  initialized to  NULL, memory is  allocated for 
     cfnQ. Otherwise, it is assumed that memory is already available.


   ACCESS:
   
     cfnP , input  ,  Function to be copied
     cfnQ , output ,  Copied function
     S    , input  ,  cfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CrvFuncCopy( NL_CFUN *cfnP, NL_CFUN *cfnQ, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_CrvFuncCopy");

    NL_FLAG error;

    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_REAL *fp, *fq, *UP, *UQ;

    /* Get local notation */

    N_CFuncGetData( cfnP, &n, &fp, &p, &m, &UP );

    /* See if memory is needed */

    error = N_CFuncSizeArrays( cfnQ, n, p, m, rname, S );

    if( error EQ NL_YES )
        return (1);

    N_CrvFuncCntrlValKnots( cfnQ, &fq, &UQ );

    /* Copy the function */

    for ( i = 0; i <= n; i++ )
        fq[i] = fp[i];

    for ( i = 0; i <= m; i++ )
        UQ[i] = UP[i];

    /* Exit */

    return (0);
} /* end N_CrvFuncCopy */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine prints curve function definition data out to 
     the standard output device. The print is arranged as follows:

           n       --> highest index in control value array
           p       --> degree

           <stop>  --> hit any key to continue

           f0      -->
           f1      -->
           .       --> 
           .       --> control values
           .       -->
           fn      -->

           <stop>   --> hit any key to continue

           u0       -->
           u1       -->
           .        -->
           .        --> knots
           .        -->
           um       -->

     A typical calling example is:

       NL_CFUN  cfn;
       ...
       N_CrvFuncPrint(&cfn);


   ACCESS:
   
     cfn , input ,  Curve fuction to be printed


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvFuncPrint( NL_CFUN *cfn )
{
    NL_INDEX i, n, m;

    NL_DEGREE p;

    NL_REAL *U, *fu;

    /* Get local notation */

    N_CFuncGetData( cfn, &n, &fu, &p, &m, &U );

    /* Print function data */

    N_FPRINTF( stdout, _T("%ld\n"), n );
    N_FPRINTF( stdout, _T("%hd\n"), p );

    NL_PAUSE;

    for ( i = 0; i <= n; i++ )
        N_FPRINTF( stdout, _T("%18.16f\n"), fu[i] );

    NL_PAUSE;

    for ( i = 0; i <= m; i++ )
        N_FPRINTF( stdout, _T("%18.16f\n"), U[i] );
} /* end N_CrvFuncPrint */

/*******************************************************************//**


   DESCRIPTION:

     Given a curve function, this  routine scales the knot vector to a 
     given  interval. That  is, U[0] <= U[1] <= ... <= U[m] is  mapped
     to  a <= ... <=  U[p+1]' <= ... <=  U[m-p-1]'  <= ... <= b, where
     I=[a,b] is the given  interval. A typical calling example is: 

       NL_CFUN      cfn;
       NL_INTERVAL  I;
       ...
       (get new interval I);
       ...
       N_CrvFuncReparam(&cfn,I);

     The knot vector is rescaled IN-PLACE, i.e. the original knots are
     destroyed.


   ACCESS:
   
     cfn , in/out ,  Function
     I   , input  ,  Parameter interval


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvFuncReparam( NL_CFUN *cfn, NL_INTERVAL I )
{
    NL_INDEX i, m;

    NL_DEGREE p;

    NL_REAL *U, fac, a, b, u0;

    /* Get local notation */

    N_CFuncGetDegree( cfn, &p );
    N_CFuncGetKnots( cfn, &m, &U );
    N_IntervalGetData( &I, &a, &b );

    /* Compute new knots */

    if( a NEQ U[0]OR b NEQ U[m] )
    {
        u0 = U[0];
        fac = (b - a) / (U[m] - U[0]);

        for ( i = 0; i <= p; i++ )
            U[i] = a;

        for ( i = p + 1; i <= m - p - 1; i++ )
            U[i] = fac * (U[i] - u0) + a;

        for ( i = m - p; i <= m; i++ )
            U[i] = b;
    }
} /* end N_CrvFuncReparam */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control values from a curve function 
     object. A typical calling example is:

       NL_CFUN   cfn;
       NL_INDEX  n;
       NL_REAL   *fu;
       ...
       N_CrvFuncCntrlVal(&cfn,&n,&fu);


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     n   , output ,  Highest index in fu
     fu  , output ,  Control value array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvFuncCntrlVal( NL_CFUN *cfn, NL_INDEX *n, NL_REAL ** fu )
{
    *n = cfn->cvl->n;
    *fu = cfn->cvl->fu;
} /* end N_CrvFuncCntrlVal */


/*******************************************************************//**


   DESCRIPTION:

     This utility  routine detaches the control value and the knot 
     vector objects from a given curve function. A typical calling 
     example is:
   
       NL_CFUN        cfn;
       NL_CVALUE      *cvl;
       NL_DEGREE      p;
       NL_KNOTVECTOR  *knt;
       ...
       N_CFuncGetArrayAndKnotVector(&cfn,&cvl,&p,&knt);


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     cvl , output ,  Control value object
     p   , output ,  Degree
     knt , output ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CFuncGetArrayAndKnotVector( NL_CFUN *cfn, NL_CVALUE ** cvl, NL_DEGREE *p, NL_KNOTVECTOR ** knt )
{
    *cvl = cfn->cvl;
    *p = cfn->p;
    *knt = cfn->knt;
} /* end N_CFuncGetArrayAndKnotVector */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control values, knot vector and knots 
     from a curve function object. A typical calling example is:

       NL_CFUN        cfn;
       NL_REAL        *fu;
       NL_KNOTVECTOR  *knt;
       NL_REAL        *U;
       ...
       N_CrvFuncCntrlValKnotVector(&cfn,&fu,&knt,&U);


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     fu  , output ,  Control value array
     knt , output ,  Knot vector object
     U   , output ,  Knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvFuncCntrlValKnotVector( NL_CFUN *cfn, NL_REAL ** fu, NL_KNOTVECTOR ** knt, NL_REAL ** U )
{
    *fu = cfn->cvl->fu;
    *knt = cfn->knt;
    *U = cfn->knt->U;
} /* end N_CrvFuncCntrlValKnotVector */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the highest indexes of a curve function 
     definition. A typical calling example is:

       NL_CFUN   cfn;
       NL_INDEX  n, m;
       ...
       N_CFuncGetArraySizes(&cfn,&n,&m);


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     n   , output ,  Highest index in fu
     m   , output ,  Highest index in U


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CFuncGetArraySizes( NL_CFUN *cfn, NL_INDEX *n, NL_INDEX *m )
{
    *n = cfn->cvl->n;
    *m = cfn->knt->m;
} /* end N_CFuncGetArraySizes */


/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This utility routine gets curve function knot vector information.
     A typical calling example is:
 
       NL_CFUN   cfn;
       NL_INDEX  m;
       NL_REAL   *U;
       ...
       N_CFuncGetKnots(&cfn,&m,&U);
 
 
   ACCESS:
   
     cfn , input  ,  NURBS curve function
     m   , output ,  Highest index in U
     U   , output ,  Knot vector
 
 
   RETURN CODES:
 
     None
 
   ***********************************************************************/

NL_VOID N_CFuncGetKnots( NL_CFUN *cfn, NL_INDEX *m, NL_REAL ** U )
{
    *m = cfn->knt->m;
    *U = cfn->knt->U;
} /* end N_CFuncGetKnots */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the knot vector object from a curve 
     function. A typical calling example is:

       NL_CFUN        cfn;
       NL_KNOTVECTOR  *knt;
       ...
       N_CFuncGetKnotVector(&cfn,&knt);


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     knt , output ,  Knot vector object


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CFuncGetKnotVector( NL_CFUN *cfn, NL_KNOTVECTOR ** knt )
{
    *knt = cfn->knt;
} /* end N_CFuncGetKnotVector */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the degree of a curve function. A typical 
     calling example is:

       NL_CFUN    cfn;
       NL_DEGREE  *p;
       ...
       N_CFuncGetDegree(&cfn,&p);


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     p   , output ,  Degree


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CFuncGetDegree( NL_CFUN *cfn, NL_DEGREE *p )
{
    *p = cfn->p;
} /* end N_CFuncGetDegree */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control values and knots from a curve 
     function object. A typical calling example is:

       NL_CFUN  cfn;
       NL_REAL  *fu, *U;
       ...
       N_CrvFuncCntrlValKnots(&cfn,&fu,&U);


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     fu  , output ,  Control value array
     U   , output ,  Knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvFuncCntrlValKnots( NL_CFUN *cfn, NL_REAL ** fu, NL_REAL ** U )
{
    *fu = cfn->cvl->fu;
    *U = cfn->knt->U;
} /* end N_CrvFuncCntrlValKnots */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine breaks a curve function object down to its 
     components, i.e. indexes, control value array and knot vector. A  
     typical calling example is:

       NL_CFUN    cfn;
       NL_INDEX   n, m;
       NL_DEGREE  p;
       NL_REAL    *fu, *U;
       ...
       N_CFuncGetData(&cfn,&n,&fu,&p,&m,&U);


   ACCESS:
   
     cfn , input  ,  NURBS curve function
     n   , output ,  Highest index in fu
     fu  , output ,  Control values
     p   , output ,  Degree
     m   , output ,  Highest index in U
     U   , output ,  Knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CFuncGetData( NL_CFUN *cfn, NL_INDEX *n, NL_REAL ** fu, NL_DEGREE *p, NL_INDEX *m, NL_REAL ** U )
{
    *n = cfn->cvl->n;
    *fu = cfn->cvl->fu;
    *p = cfn->p;
    *m = cfn->knt->m;
    *U = cfn->knt->U;
} /* end N_CFuncGetData */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets control value and knot vector pointers
     of a curve function object. A typical calling example is:

       NL_CFUN  cfn;
       NL_REAL  *fu, *U;
       ...
       N_CrvFuncSetPtrs(&cfn,fu,U);


   ACCESS:
   
     cfn , in/out ,  Curve function
     fu  , input  ,  Control value array
     U   , input  ,  Knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CrvFuncSetPtrs( NL_CFUN *cfn, NL_REAL *fu, NL_REAL *U )
{
    cfn->cvl->fu = fu;
    cfn->knt->U = U;
} /* end N_CrvFuncSetPtrs */

/*******************************************************************//**


   DESCRIPTION:

     This routine  computes a point on a  curve function by evaluating 
     all  non-vanishing   basis  functions  and  multiplying  them  by 
     appropriate values. Discontinuous functions can also be evaluated 
     by passing a NL_LEFT/NL_RIGHT flag. A typical calling example is:

       NL_CFUN       cfn;
       NL_PARAMETER  u;
       NL_REAL       f;
       ...
       (define cfn and get u);
       ...
       N_CFuncEval(&cfn,u,NL_LEFT,&f);


   ACCESS:
   
     cfn , input  ,  Curve function
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                       NL_RIGHT: u is in (u[j],u[j+1]]
     f   , output ,  Function value


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CFuncEval( NL_CFUN *cfn, NL_PARAMETER u, NL_FLAG flg, NL_REAL *f )
{
    NL_PRIVATE NL_STRING rname = _T("N_CFuncEval");

    NL_FLAG error = NL_NO;

    NL_INDEX i, spn;

    NL_DEGREE p;

    NL_REAL *N, *fu;

    NL_KNOTVECTOR *knt;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvFuncCntrlValKnots( cfn, &fu, &N );
    N_CFuncGetDegree( cfn, &p );
    N_CFuncGetKnotVector( cfn, &knt );

    /* Check parameter */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute non-vanishing B-splines */

    N = N_AllocReal1dArray( p, &S );

    if( N EQ NULL )
        NL_QUIT;

    error = N_BasisEval( knt, p, u, flg, N, &spn );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the function value */

    *f = 0.0;

    for ( i = 0; i <= p; i++ )
    {
        *f += fu[spn - p + i] * N[i];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CFuncEval */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes derivatives of a curve function by evaluating 
     all  non-vanishing  basis  functions  and  their  derivatives,  and 
     multiplying them by appropriate values. Discontinuous functions can 
     also  be  handled by  passing a  NL_LEFT/NL_RIGHT flag. A typical calling
     example is:

       NL_CFUN       cfn;
       NL_PARAMETER  u;
       NL_INDEX      der;
       NL_REAL       *fd;
       ...
       (define cfn, get u, der and allocate memory for fd);
       ...
       N_CFuncDerivs(&cfn,u,NL_LEFT,der,fd);


   ACCESS:
   
     cfn , input  ,  Curve function
     u   , input  ,  Parameter value 
     flg , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1])
                              (NL_RIGHT DERIVATIVES REQUIRED)
                       NL_RIGHT: u is in (u[j],u[j+1]]
                              (NL_LEFT DERIVATIVES REQUIRED) 
     der , input  ,  Highest derivative required
     fd  , output ,  Function derivatives; fd[k] is the k-th derivative.
                     MEMORY  FOR fd  MUST BE  ALLOCATED  IN  THE CALLING
                     ROUTINE.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CFuncDerivs( NL_CFUN *cfn, NL_PARAMETER u, NL_FLAG flg, NL_INDEX der, NL_REAL *fd )
{
    NL_PRIVATE NL_STRING rname = _T("N_CFuncDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX j, k, spn;

    NL_DEGREE p;

    NL_REAL ** ND, *fu, *U;

    NL_KNOTVECTOR *knt;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_CrvFuncCntrlValKnots( cfn, &fu, &U );
    N_CFuncGetDegree( cfn, &p );
    N_CFuncGetKnotVector( cfn, &knt );

    /* Check parameter */

    error = N_KnotVectorIsParamOutOfBounds( knt, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute derivatives */

    ND = N_AllocReal2dArray( der, p, &S );

    if( ND EQ NULL )
        NL_QUIT;

    error = N_BasisDerivs( knt, p, u, flg, der, ND, &spn );

    if( error EQ NL_YES )
        NL_OUT;

    for ( k = 0; k <= der; k++ )
    {
        fd[k] = 0.0;

        for ( j = spn - p; j <= spn; j++ )
        {
            fd[k] += fu[j] * ND[k][j - spn + p];
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_CFuncDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This error  routine checks  if a surface  function  structure has  
     sufficient  memory to  store control  values and knots. A typical 
     calling example is:

       NL_SFUN    sfn;
       NL_INDEX   nf, mf, rk, sk;
       NL_STRING  rname;
       ...
       (define sfn, get nf, mf, rk, sk and rname);
       ...
       N_SrfIsFuncSized(&sfn,nf,mf,rk,sk,rname);


   ACCESS:
   
     sfn    , input ,  Surface function
     nf,mf  , input ,  Expected highest indexes in control value array
     rk,sk  , input ,  Expected highest indexes in knot vector arrays
     rname  , input ,  Routine name where error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfIsFuncSized( NL_SFUN *sfn, NL_INDEX nf, NL_INDEX mf, NL_INDEX rk, NL_INDEX sk, NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX n, m, r, s;

    /* Get local notation */

    N_SFuncGetArraySizes( sfn, &n, &m, &r, &s );

    /* Check storage */

    if( n LT nf OR m LT mf OR r LT rk OR s LT sk )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */

    return (error);
} /* end N_SrfIsFuncSized */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store members of a surface
     value  structure. Proper  error check is  performed in case 
     memory allocation fails. A typical calling example is:

       NL_SVALUE  *svl;
       NL_STACKS  S;
       ...
       svl = N_AllocSValue(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     svl  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_SVALUE *N_AllocSValue( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocSValue");

    NL_SVALUE *svl;

    NL_SVLNODE *svd;

    /* Allocate memory for the structure */

    svl = (NL_SVALUE *)N_Malloc( sizeof( NL_SVALUE ) );

    if( svl EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    svd = (NL_SVLNODE *)N_Malloc( sizeof( NL_SVLNODE ) );

    if( svd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( svl );
        svl = NULL;
        return NULL;
    }

    svd->ptr = svl;
    svd->next = S->svl;
    S->svl = svd;

    /* Exit */

    return svl;
} /* end N_AllocSValue */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to store a surface value object. 
     Proper error check is performed in case memory allocation fails. 
     A typical calling example is:

       NL_SVALUE  *svs;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get n and m);
       ...
       svs = N_AllocSValueAndArray(n,m,&S);


   ACCESS:
   
     n,m , input  ,  Highest indexes in control value array
     S   , input  ,  Memory stack pointer


   RETURN CODES:

     svs  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_SVALUE *N_AllocSValueAndArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_REAL ** fuv;

    NL_SVALUE *svs;

    /* Allocate memory */

    svs = N_AllocSValue( S );

    if( svs EQ NULL )
        return NULL;

    fuv = N_AllocReal2dArray( n, m, S );

    if( fuv EQ NULL )
        return NULL;

    /* Build the object */

    N_SValueFromArray( svs, fuv, n, m );

    /* Exit */

    return svs;
} /* end N_AllocSValueAndArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine deallocates  memory that stores  members of a 
     control value structure. Given a control value pointer, the routine
     searches  for the  pointer  on the  memory  stack. It it  is found, 
     memory is deallocated. If not, the  routine does nothing. A typical
     calling example is:

       NL_SVALUE  *svl;
       NL_STACKS   S;
       ...
       N_FreeSValue(svl,&S);


   ACCESS:
   
     svl , input  ,  Control value pointer 
     S   , input  ,  svl's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeSValue( NL_SVALUE *svl, NL_STACKS *S )
{
    NL_SVLNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->svl NEQ NULL )
    {
        prev = S->svl;
        curr = S->svl;

        while( curr NEQ NULL AND curr->ptr NEQ svl )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->svl = NULL;
            }
            else /* More than one node */
            {
                S->svl = S->svl->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
        }
    }
} /* end N_FreeSValue */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes proper  pointer  assignments to define a
     surface  function  control  value  object  from a  set of  function 
     values. Memory for the  control value structure is allocated in the  
     calling routine; only the pointer is passed down. A typical calling  
     example is:

       NL_SVALUE  svl;
       NL_REAL    **fuv;
       NL_INDEX   n, m;
       ...
       (allocate memory for fuv);
       ...
       N_SValueFromArray(&svl,fuv,n,m);


   ACCESS:
   
     svl , in/out ,  Control value structure
     fuv , input  ,  Function values
     n,m , input  ,  Highest indexes in fuv


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SValueFromArray( NL_SVALUE *svl, NL_REAL ** fuv, NL_INDEX n, NL_INDEX m )
{
    svl->n = n;
    svl->m = m;
    svl->fuv = fuv;
} /* end N_SValueFromArray */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  allocates  memory  to  store a  2-D array of surface
     functions each defined by the same set of parameters <n,m,p,q,r,s>. 
     Proper error check is performed in  case  memory allocations  fail. 
     A typical calling example is:

       NL_SFUN     ***sf3;
       NL_INDEX    n, m, r, s, k, l;
       NL_DEGREE   p, q; 
       NL_STACKS   S;
       ...
       (get n, m, p, q, r, s, k and l);
       ...
       sf3 = N_Alloc2dArraySrfFunc(n,m,p,q,r,s,k,l,&S);


   ACCESS:
   
     n,m  , input  ,  Highest indexes in control value arrays
     p,q  , input  ,  Degrees in u- and v-directions
     r,s  , input  ,  Highest indexes in knot vector arrays
     k,l  , input  ,  Highest  indexes in function array  sf3[0][0],...,
                      sf3[k][l];  sf3[i][j] is a  pointer to the (i,j)th
                      function. 
     S    , input  ,  Memory stacks pointer


   RETURN CODES:

     sf3  : Pointer to 2-D array of functions if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_SFUN *** N_Alloc2dArraySrfFunc( NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_INDEX r, NL_INDEX s, NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Alloc2dArraySrfFunc");

    NL_INDEX i, j;

    NL_SFUN *** sf3, ** sf2;

    NL_SF2NODE *v2d;

    NL_SF3NODE *v3d;

    /* Allocate memory for function pointer arrays */

    sf3 = (NL_SFUN *** )N_Malloc( (k + 1) * sizeof( NL_SFUN ** ) );

    if( sf3 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    sf2 = (NL_SFUN ** )N_Malloc( (k + 1) * (l + 1) * sizeof( NL_SFUN * ) );

    if( sf2 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( sf3 );
        sf3 = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    j = 0;

    for ( i = 0; i <= k; i++ )
    {
        sf3[i] = &sf2[j];
        j = j + l + 1;
    }

    /* Allocate memory for the individual functions */

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
            sf3[i][j] = N_AllocSrfFunc( n, m, p, q, r, s, S );

            if( sf3[i][j]EQ NULL )
            {
                N_Free( sf3 );
                sf3 = NULL;
                N_Free( sf2 );
                sf2 = NULL;
                return NULL;
            }
        }
    }

    /* Put pointers on memory stacks */

    v2d = (NL_SF2NODE *)N_Malloc( sizeof( NL_SF2NODE ) );

    if( v2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( sf3 );
        sf3 = NULL;
        N_Free( sf2 );
        sf2 = NULL;
        return NULL;
    }

    v3d = (NL_SF3NODE *)N_Malloc( sizeof( NL_SF3NODE ) );

    if( v3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( sf3 );
        sf3 = NULL;
        N_Free( sf2 );
        sf2 = NULL;
        N_Free( v2d );
        v2d = NULL;
        return NULL;
    }

    v2d->ptr = sf2;
    v2d->next = S->sf2;
    S->sf2 = v2d;

    v3d->ptr = sf3;
    v3d->next = S->sf3;
    S->sf3 = v3d;

    /* Exit */

    return sf3;
} /* end N_Alloc2dArraySrfFunc */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a surface  function defined 
     by the  usual parameters  <n,m,p,q,r,s>. Proper  error  checks are 
     performed  in  case  memory  allocations  fail. A  typical calling 
     example is:

       NL_SFUN    *sfn;
       NL_INDEX   n, m, r, s;
       NL_DEGREE  p, q;
       NL_STACKS  S;
       ...
       (get n, m, p, q, r and s);
       ...
       sfn = N_AllocSrfFunc(n,m,p,q,r,s,&S);


   ACCESS:
   
     n,m  , input  ,  Highest indexes in control value array
     p,q  , input  ,  Degrees
     r,s  , input  ,  Highest indexes in knot vector arrays
     S    , input  ,  Memory stack pointer


   RETURN CODES:

     sfn  : Pointer to funcion if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_SFUN *N_AllocSrfFunc( NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_INDEX r, NL_INDEX s, NL_STACKS *S )
{
    NL_SVALUE *svs;

    NL_KNOTVECTOR *knu, *knv;

    NL_SFUN *sfn;

    /* Allocate memory */

    svs = N_AllocSValueAndArray( n, m, S );

    if( svs EQ NULL )
        return NULL;

    knu = N_AllocKnotVectorAndArray( r, S );

    if( knu EQ NULL )
        return NULL;

    knv = N_AllocKnotVectorAndArray( s, S );

    if( knv EQ NULL )
        return NULL;

    sfn = N_AllocSrfFuncData( S );

    if( sfn EQ NULL )
        return NULL;

    /* Build surface function structure */

    N_SFuncFromKnotVectors( sfn, svs, p, q, knu, knv );

    /* Exit */

    return sfn;
} /* end N_AllocSrfFunc */

/*******************************************************************//**


   DESCRIPTION:

     This routine  allocates memory for  members of  a surface function
     structure. Error  check is  performed in  case  memory  allocation 
     fails. A typical calling example is:

       NL_SFUN    *sfs;
       NL_STACKS  S;
       ...
       sfs = N_AllocSrfFuncData(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     sfs  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_SFUN *N_AllocSrfFuncData( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocSrfFuncData");

    NL_SFUN *sfs;

    NL_SFNNODE *sfd;

    /* Allocate memory for the structure */

    sfs = (NL_SFUN *)N_Malloc( sizeof( NL_SFUN ) );

    if( sfs EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    sfd = (NL_SFNNODE *)N_Malloc( sizeof( NL_SFNNODE ) );

    if( sfd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( sfs );
        sfs = NULL;
        return NULL;
    }

    sfd->ptr = sfs;
    sfd->next = S->sfn;
    S->sfn = sfd;

    /* Exit */

    return sfs;
} /* end N_AllocSrfFuncData */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  deallocates  memory  that  stores surface 
     function data, i.e. control value structure, control values, knot  
     vector structures  and knots. IT DOES NOT  DEALLOCATE MEMORY THAT 
     STORES THE NL_FUNCTION STRUCTURE ITSELF. A typical calling example:

       NL_SFUN    *sfn;
       NL_STACKS   S;
       ...
       N_FreeSrfFuncData(sfn,&S);


   ACCESS:
   
     sfn , input  ,  Function pointer
     S   , input  ,  sfn's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeSrfFuncData( NL_SFUN *sfn, NL_STACKS *S )
{
    NL_DEGREE p, q;

    NL_REAL ** fuv, *U, *V;

    NL_SVALUE *svl;

    NL_KNOTVECTOR *knu, *knv;

    /* Get locals */

    N_SFuncGetArrayAndKnotVectors( sfn, &svl, &p, &q, &knu, &knv );
    N_SFuncGetKnots( sfn, &fuv, &U, &V );

    /* Kill function constituents */

    N_FreeSValue( svl, S );
    N_FreeReal2dArray( fuv, S );
    N_FreeKnotVector( knu, S );
    N_FreeReal1dArray( U, S );
    N_FreeKnotVector( knv, S );
    N_FreeReal1dArray( V, S );
} /* end N_FreeSrfFuncData */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine makes  proper pointer  assignments to define
     a  surface function  object  from  control  value and  knot vector 
     objects. Memory is allocated in the calling routine; only pointers 
     are passed down. A typical calling example is:

       NL_SFUN        sfn;
       NL_SVALUE      svl;
       NL_DEGREE      p, q;
       NL_KNOTVECTOR  knu, knv;
       ...
       (define svl, knu and knv);
       ...
       N_SFuncFromKnotVectors(&sfn,&svl,p,q,&knu,&knv);


   ACCESS:
   
     sfn     , in/out ,  NURBS surface function
     svl     , input  ,  Control value object
     p,q     , input  ,  Degrees
     knu,knv , input  ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SFuncFromKnotVectors( NL_SFUN *sfn, NL_SVALUE *svl, NL_DEGREE p, NL_DEGREE q, NL_KNOTVECTOR *knu, NL_KNOTVECTOR *knv )
{
    sfn->svl = svl;
    sfn->p = p;
    sfn->q = q;
    sfn->knu = knu;
    sfn->knv = knv;
} /* end N_SFuncFromKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  generates a surface  function object  given function 
     values  and knots. It  allocates memory  to store the control value  
     and knot  vector  objects, and makes  proper pointer assignments to 
     create the surface function object. Memory for the surface function  
     structure is allocated  in the  calling routine. A  typical calling 
     example is:

       NL_SFUN    sfn;
       NL_REAL    **fuv, *U, *V;
       NL_INDEX   n, m, r, s;
       NL_DEGREE  p, q;
       NL_STACKS  S;
       ...
       (allocate memory for fuv, U and V);
       ...
       N_SFuncFromKnots(&sfn,fuv,n,m,p,q,U,V,r,s,&S);


   ACCESS:
   
     sfn , in/out ,  NURBS surface function
     fuv , input  ,  Function values
     n,m , input  ,  Highest indexes in fuv
     p,q , input  ,  Degrees
     U,V , input  ,  Knot vectors
     r,s , input  ,  Highest indexes in U and V, respectively
     S   , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SFuncFromKnots( NL_SFUN *sfn, NL_REAL ** fuv, NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_REAL *U, NL_REAL *V, NL_INDEX r, NL_INDEX s, NL_STACKS *S )
{
    NL_SVALUE *svl;

    NL_KNOTVECTOR *knu, *knv;

    /* Allocate memory */

    svl = N_AllocSValue( S );

    if( svl EQ NULL )
        return (1);

    knu = N_AllocKnotVector( S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVector( S );

    if( knv EQ NULL )
        return (1);

    /* Make pointer assignments */

    N_SValueFromArray( svl, fuv, n, m );
    N_KnotVectorFromRealArray( knu, U, r );
    N_KnotVectorFromRealArray( knv, V, s );
    N_SFuncFromKnotVectors( sfn, svl, p, q, knu, knv );

    /* Exit */

    return (0);
} /* end N_SFuncFromKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a surface function is initialized to 
     the NULL  function. If yes,  NL_TRUE is returned. Otherwise,  NL_FALSE is  
     returned. This  routine  is  used  to  check  if  memory allocation 
     is needed. A typical calling example is:

       NL_SFUN  sfn;
       ...
       if( N_SrfAreFuncArraysNULL(&sfn) )  --> allocate memory;
     

   ACCESS:
   
     sfn , input ,  Surface function


   RETURN CODES:

     NL_TRUE:  Function is initialized to NULL (need memory)
     NL_FALSE: Function is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

NL_BOOLEAN N_SrfAreFuncArraysNULL( NL_SFUN *sfn )
{
    NL_DEGREE p, q;

    NL_SVALUE *svl;

    NL_KNOTVECTOR *knu, *knv;

    /* Get local notation */

    N_SFuncGetArrayAndKnotVectors( sfn, &svl, &p, &q, &knu, &knv );

    /* Check initialization */

    if( svl EQ NULL OR p EQ - 1 OR q EQ - 1 OR knu EQ NULL OR knv EQ NULL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_SrfAreFuncArraysNULL */

/*******************************************************************//**


   DESCRIPTION:

     Given a surface function object, this routine allocates memory to 
     store control points and knots. A typical calling example is:

       NL_SFUN    sfn;
       NL_STACKS  S;
       ...
       N_AllocSFuncArrays(&sfn,n,m,p,q,r,s,&S);

     where  <n,p,r>  and  <m,q,s>  are the  usual surface  parameters. 
     Since  the  declaration  "NL_SFUN  sfn"  defines  the data  type and 
     allocates memory,  memory is needed to store only the control net 
     and knot vector objects.


   ACCESS:
   
     sfn , in/out ,  Surface function structure
     n,m , input  ,  Highest indexes in control value array
     p,q , input  ,  Degrees of the function
     r,s , input  ,  Highest indexes in knot vector arrays
     S   , input  ,  sfn's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AllocSFuncArrays( NL_SFUN *sfn, NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_INDEX r, NL_INDEX s, NL_STACKS *S )
{
    NL_SVALUE *svl;

    NL_KNOTVECTOR *knu, *knv;

    /* Allocate memory */

    svl = N_AllocSValueAndArray( n, m, S );

    if( svl EQ NULL )
        return (1);

    knu = N_AllocKnotVectorAndArray( r, S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVectorAndArray( s, S );

    if( knv EQ NULL )
        return (1);

    /* Build function structure */

    N_SFuncFromKnotVectors( sfn, svl, p, q, knu, knv );

    /* Exit */

    return (0);
} /* end N_AllocSFuncArrays */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine initializes a surface function structure by  
     setting pointers to NULL  and the degrees  to -1. It is  used to  
     check if memory allocation is needed, ie if the surface function 
     is initialized to NULL, memory is allocated to hold the  control 
     value and knot vectors objects.  Otherwise  it is  assumed  that 
     memory has already been allocated. A typical calling example is:

       NL_SFUN  sfn;
       ...
       N_SFuncInitArrays(&sfn);
     

   ACCESS:
   
     sfn , in/out ,  Surface function


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SFuncInitArrays( NL_SFUN *sfn )
{
    sfn->svl = NULL;
    sfn->p = -1;
    sfn->q = -1;
    sfn->knu = NULL;
    sfn->knv = NULL;
} /* end N_SFuncInitArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets parameters to complete surface function
     definition. A typical calling example is:

       NL_SFUN    sfn;
       NL_INDEX   n, m, r, s;
       NL_DEGREE  p, q;
       ...
       N_SFuncSetSizeIndices(&sfn,n,m,p,q,r,s);


   ACCESS:
   
     sfn , in/out ,  Surface function
     n,m , input  ,  Highest indexes in control value array
     p,q , input  ,  Degrees
     r,s , input  ,  Highest indexes in knot vector arrays


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SFuncSetSizeIndices( NL_SFUN *sfn, NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_INDEX r, NL_INDEX s )
{
    sfn->svl->n = n;
    sfn->svl->m = m;
    sfn->p = p;
    sfn->q = q;
    sfn->knu->m = r;
    sfn->knv->m = s;
} /* end N_SFuncSetSizeIndices */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if memory is needed to store a surface
     function. If  the  function  is  initiaized  to  NULL,  memory  is  
     allocated. If  not,  the  routine  checkes  if  enough  memory  is 
     available. A typical calling example is:

       NL_SFUN     sfn;
       NL_INDEX    n, m, r, s;
       NL_DEGREE   p, q;
       NL_STRING   rname;
       NL_STACKS   S;
       ...
       (get n, m, p, q, r, s and rname);
       ...
       N_SFuncSizeArrays(&sfn,n,m,p,q,r,s,rname,&S);

     IT IS  ASSUMED THAT MEMORY TO STORE THE NL_SURFACE NL_FUNCTION STRUCTURE 
     ITSELF IS ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     sfn         , in/out ,  Surface function to be created
     n,m,p,q,r,s , input  ,  Usual surface parameters
     rname       , input  ,  Routine name
     S           , input  ,  sfn's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SFuncSizeArrays( NL_SFUN *sfn, NL_INDEX n, NL_INDEX m, NL_DEGREE p, NL_DEGREE q, NL_INDEX r, NL_INDEX s, NL_STRING rname, NL_STACKS *S )
{
    NL_FLAG error;

    /* See if memory is needed */

    if( N_SrfAreFuncArraysNULL( sfn ) )
    {
        error = N_AllocSFuncArrays( sfn, n, m, p, q, r, s, S );

        if( error EQ 1 )
            return (1);
    }
    else
    {
        error = N_SrfIsFuncSized( sfn, n, m, r, s, rname );

        if( error EQ 1 )
            return (1);

        N_SFuncSetSizeIndices( sfn, n, m, p, q, r, s );
    }

    /* Exit */

    return (0);
} /* end N_SFuncSizeArrays */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine compacts  control  value and  knot  vector 
     arrays. That is, given a surface function with control value  and 
     knot vector arrays  larger than  required. This routine redefines  
     these  arrays to the appropriate  sizes which  makes the function  
     definition more memory efficient. A typical calling example is:

       NL_SFUN    sfn;
       NL_STACKS  SG;
       ...
       (define sfn);
       ...
       N_SrfFuncCompact(&sfn,&SG);

     S MUST BE  sfn's STACK, I.E. ALL MEMORY ALLOCATED FOR sfn MUST BE 
     ON S.


   ACCESS:
   
     sfn , in/out ,  Surface function
     SG  , input  ,  sfn's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncCompact( NL_SFUN *sfn, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_REAL ** fuv, ** fst, *U, *V, *S, *T;

    /* Get function data */

    N_SFuncGetArraySizes( sfn, &n, &m, &r, &s );
    N_SFuncGetKnots( sfn, &fuv, &U, &V );

    /* Allocate memory for new control value and knot vector arrays */

    fst = N_AllocReal2dArray( n, m, SG );

    if( fst EQ NULL )
        NL_QUIT;

    S = N_AllocReal1dArray( r, SG );

    if( S EQ NULL )
        NL_QUIT;

    T = N_AllocReal1dArray( s, SG );

    if( T EQ NULL )
        NL_QUIT;

    /* Copy control values and knots */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
            fst[i][j] = fuv[i][j];
    }

    for ( i = 0; i <= r; i++ )
        S[i] = U[i];

    for ( j = 0; j <= s; j++ )
        T[j] = V[j];

    /* Redefine function and kill old memory */

    N_SrfFuncSetPtrs( sfn, fst, S, T );

    N_FreeReal2dArray( fuv, SG );
    N_FreeReal1dArray( U, SG );
    N_FreeReal1dArray( V, SG );

    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfFuncCompact */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine copies a given surface  function into a new 
     function. A typical calling example is:

       NL_SFUN    sfnP, sfnQ;
       NL_STACKS  S;
       ...
       (define sfnP);
       ...
       N_SFuncInitArrays(&sfnQ);
       N_SrfFuncCopy(&sfnP,&sfnQ,&S);

     If the function is initialized to NULL, memory will be allocated 
     for  sfnQ.  Otherwise,  it is  assumed  that  memory  is already 
     available.


   ACCESS:
   
     sfnP , input  ,  Function to be copied
     sfnQ , output ,  Copied function
     S    , input  ,  sfnQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncCopy( NL_SFUN *sfnP, NL_SFUN *sfnQ, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncCopy");

    NL_FLAG error;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL ** fp, ** fq, *UP, *VP, *UQ, *VQ;

    /* Get local notation */

    N_SFuncGetComponents( sfnP, &n, &m, &fp, &p, &q, &r, &s, &UP, &VP );

    /* See if memory is needed */

    error = N_SFuncSizeArrays( sfnQ, n, m, p, q, r, s, rname, S );

    if( error EQ NL_YES )
        return (1);

    N_SFuncGetKnots( sfnQ, &fq, &UQ, &VQ );

    /* Copy the function */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
            fq[i][j] = fp[i][j];
    }

    for ( i = 0; i <= r; i++ )
        UQ[i] = UP[i];

    for ( j = 0; j <= s; j++ )
        VQ[j] = VP[j];

    /* Exit */

    return (0);
} /* end N_SrfFuncCopy */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine prints surface function definition data to the 
     standard output device. The print is arranged as follows:

           n m     --> highest indexes in control value array
           p q     --> degrees    

           <stop>  --> hit any key to continue
  
           f00     -->
           f01     -->
           .       --> 
           .       --> 
           .       -->
           f0m     -->
           f10     --> control values
           f11     -->
           .       -->
           .       -->
           .       --> 
           fnm     -->

           <stop>  --> hit any key to continue

           u0      -->
           u1      -->
           .       -->
           .       --> u-knots
           .       -->
           ur      -->

           <stop>  --> hit any key to continue

           v0      -->
           v1      -->
           .       -->
           .       --> v-knots
           .       -->
           vs      -->

     This  routine allows the  programmer to quickly check a functions's
     control values  and knots, as  well as the  appropriate  indexes. A
     typical calling example is:

       NL_SFUN  sfn;
       ...
       N_SrfFuncPrint(&sfn);


   ACCESS:
   
     sfn  , input  ,  Surface function to be printed


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfFuncPrint( NL_SFUN *sfn )
{
    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL ** fuv, *U, *V;

    /* Get local notation */

    N_SFuncGetComponents( sfn, &n, &m, &fuv, &p, &q, &r, &s, &U, &V );

    /* Print surface data */

    N_FPRINTF( stdout, _T("%ld %ld\n"), n, m );
    N_FPRINTF( stdout, _T("%hd %hd\n"), p, q );
    NL_PAUSE;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
            N_FPRINTF( stdout, _T("%18.16f\n"), fuv[i][j] );
        NL_PAUSE;
    }
    NL_PAUSE;

    for ( i = 0; i <= r; i++ )
        N_FPRINTF( stdout, _T("%18.16f\n"), U[i] );
    NL_PAUSE;

    for ( j = 0; j <= s; j++ )
        N_FPRINTF( stdout, _T("%18.16f\n"), V[j] );
} /* end N_SrfFuncPrint */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine breaks a surface function object down to its 
     components, i.e. indexes, control  value array and knot vectors. A  
     typical calling example is:

       NL_SFUN    sfn;
       NL_INDEX   n, m, r, s;
       NL_DEGREE  p, q;
       NL_REAL    **fuv, *U, *V;
       ...
       N_SFuncGetComponents(&sfn,&n,&m,&fuv,&p,&q,&r,&s,&U,&V);


   ACCESS:
   
     sfn , input  ,  NURBS surface function
     n,m , output ,  Highest indexes in fuv
     fuv , output ,  Control values
     p,q , output ,  Degrees
     r,s , output ,  Highest indexes in U and V, respectively
     U,V , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SFuncGetComponents( NL_SFUN *sfn, NL_INDEX *n, NL_INDEX *m, NL_REAL *** fuv, NL_DEGREE *p, NL_DEGREE *q, NL_INDEX *r, NL_INDEX *s, NL_REAL ** U, NL_REAL ** V )
{
    *n = sfn->svl->n;
    *m = sfn->svl->m;
    *fuv = sfn->svl->fuv;
    *p = sfn->p;
    *q = sfn->q;
    *r = sfn->knu->m;
    *s = sfn->knv->m;
    *U = sfn->knu->U;
    *V = sfn->knv->U;
} /* end N_SFuncGetComponents */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control values and knots from a surface 
     function object. A typical calling example is:

       NL_SFUN  sfn;
       NL_REAL  **fu, *U, *V;
       ...
       N_SFuncGetKnots(&sfn,&fuv,&U,&V);


   ACCESS:
   
     sfn , input  ,  NURBS surface function
     fuv , output ,  Control value array
     U,V , output ,  Knot vector arrays


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SFuncGetKnots( NL_SFUN *sfn, NL_REAL *** fuv, NL_REAL ** U, NL_REAL ** V )
{
    *fuv = sfn->svl->fuv;
    *U = sfn->knu->U;
    *V = sfn->knv->U;
} /* end N_SFuncGetKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control values from a surface function 
     object. A typical calling example is:

       NL_SFUN   sfn;
       NL_INDEX  n, m;
       NL_REAL   **fuv;
       ...
       N_SrfFuncCntrlVal(&sfn,&n,&m,&fuv);


   ACCESS:
   
     sfn , input  ,  NURBS surface function
     n,m , output ,  Highest indexes in fuv
     fuv , output ,  Control value array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfFuncCntrlVal( NL_SFUN *sfn, NL_INDEX *n, NL_INDEX *m, NL_REAL *** fuv )
{
    *n = sfn->svl->n;
    *m = sfn->svl->m;
    *fuv = sfn->svl->fuv;
} /* end N_SrfFuncCntrlVal */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets knot vectors info from surface function 
     object. A typical calling example is:

       NL_SFUN   sfn;
       NL_INDEX  r, s;
       NL_REAL   *U, *V;
       ...
       N_SrfFuncGetKnots(&sfn,&r,&s,&U,&V);


   ACCESS:
   
     sfn , input  ,  Function
     r,s , output ,  Highest indexes in U and V
     U,V , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfFuncGetKnots( NL_SFUN *sfn, NL_INDEX *r, NL_INDEX *s, NL_REAL ** U, NL_REAL ** V )
{
    *r = sfn->knu->m;
    *s = sfn->knv->m;
    *U = sfn->knu->U;
    *V = sfn->knv->U;
} /* end N_SrfFuncGetKnots */

/*******************************************************************//**


   DESCRIPTION:

     Given a surface function, this routine scales the knot vectors to 
     a given rectangle. A typical calling example is:

       NL_SFUN       sfn;
       NL_RECTANGLE  R;
       ...
       (get rectangle R);
       ...
       N_SrfFuncScale(&sfn,R,NL_UDIR);

     The  knot vectors  are rescaled IN-PLACE, i.e. the original knots 
     are destroyed.


   ACCESS:
   
     sfn , in/out ,  Function
     R   , input  ,  Parameter rectangle
     dir , input  ,  Flag:
                       NL_UDIR : Rescale u-knot vector
                       NL_VDIR : Rescale v-knot vector
                       NL_UVDIR: Rescale both knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfFuncScale( NL_SFUN *sfn, NL_RECTANGLE R, NL_FLAG dir )
{
    NL_INDEX i, j, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, fac, a, b, c, d, u0, v0;

    /* Get local notation */

    N_SFuncGetDegrees( sfn, &p, &q );
    N_SrfFuncGetKnots( sfn, &r, &s, &U, &V );
    N_RectangleGetData( &R, &a, &b, &c, &d );

    /* Compute new knots */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        if( a NEQ U[0]OR b NEQ U[r] )
        {
            u0 = U[0];
            fac = (b - a) / (U[r] - U[0]);

            for ( i = 0; i <= p; i++ )
                U[i] = a;

            for ( i = p + 1; i <= r - p - 1; i++ )
                U[i] = fac * (U[i] - u0) + a;

            for ( i = r - p; i <= r; i++ )
                U[i] = b;
        }
    }

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        if( c NEQ V[0]OR d NEQ V[s] )
        {
            v0 = V[0];
            fac = (d - c) / (V[s] - V[0]);

            for ( j = 0; j <= q; j++ )
                V[j] = c;

            for ( j = q + 1; j <= s - q - 1; j++ )
                V[j] = fac * (V[j] - v0) + c;

            for ( j = s - q; j <= s; j++ )
                V[j] = d;
        }
    }
} /* end N_SrfFuncScale */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the knot vector objects from a surface 
     function. A typical calling example is:

       NL_SFUN        sfn;
       NL_KNOTVECTOR  *knu, *knv;
       ...
       N_SFuncGetKnotVectors(&sfn,&knu,&knv);


   ACCESS:
   
     sfn     , input  ,  NURBS surface function
     knu,knv , output ,  Knot vector objects


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SFuncGetKnotVectors( NL_SFUN *sfn, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv )
{
    *knu = sfn->knu;
    *knv = sfn->knv;
} /* end N_SFuncGetKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the degree of a surface function. A 
     typical calling example is:

       NL_SFUN    sfn;
       NL_DEGREE  *p, *q;
       ...
       N_SFuncGetDegrees(&sfn,&p,&q);


   ACCESS:
   
     sfn , input  ,  NURBS surface function
     p,q , output ,  Degrees


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SFuncGetDegrees( NL_SFUN *sfn, NL_DEGREE *p, NL_DEGREE *q )
{
    *p = sfn->p;
    *q = sfn->q;
} /* end N_SFuncGetDegrees */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the highest indexes in a surface function 
     object. A typical calling example is:

       NL_SFUN   sfn;
       NL_INDEX  n, m, r, s;
       ...
       N_SFuncGetArraySizes(&sfn,&n,&m,&r,&s);


   ACCESS:
   
     sfn , input  ,  Surface function
     n,m , output ,  Highest indexes in fuv
     r,s , output ,  Highest indexes in U and V


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SFuncGetArraySizes( NL_SFUN *sfn, NL_INDEX *n, NL_INDEX *m, NL_INDEX *r, NL_INDEX *s )
{
    *n = sfn->svl->n;
    *m = sfn->svl->m;
    *r = sfn->knu->m;
    *s = sfn->knv->m;
} /* end N_SFuncGetArraySizes */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine detaches the control value and the knot vector
     objects from a given surface function. A typical calling example:

       NL_SFUN        sfn;
       NL_SVALUE      *svl;
       NL_DEGREE      p, q;
       NL_KNOTVECTOR  *knu, *knv;
       ...
       N_SFuncGetArrayAndKnotVectors(&sfn,&svl,&p,&q,&knu,&knv);


   ACCESS:
   
     sfn     , input  ,  Surface function
     svl     , output ,  Control value
     p,q     , output ,  Degrees
     knu,knv , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SFuncGetArrayAndKnotVectors( NL_SFUN *sfn, NL_SVALUE ** svl, NL_DEGREE *p, NL_DEGREE *q, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv )
{
    *svl = sfn->svl;
    *p = sfn->p;
    *q = sfn->q;
    *knu = sfn->knu;
    *knv = sfn->knv;
} /* end N_SFuncGetArrayAndKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets control value and knot vector pointers 
     of a surface function object. A typical calling example is:

       NL_SFUN  sfn;
       NL_REAL  **fuv, *U, *V;
       ...
       N_SrfFuncSetPtrs(&sfn,fuv,U,V);


   ACCESS:
   
     sfn , in/out ,  Surface function
     fuv , input  ,  Control value array
     U   , input  ,  U-knot vector array
     V   , input  ,  V-knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfFuncSetPtrs( NL_SFUN *sfn, NL_REAL ** fuv, NL_REAL *U, NL_REAL *V )
{
    sfn->svl->fuv = fuv;
    sfn->knu->U = U;
    sfn->knv->U = V;
} /* end N_SrfFuncSetPtrs */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a point on a surface function by evaluating 
     all  non-vanishing   basis  functions  and  multiplying  them  by 
     appropriate values. Discontinuous functions can also be evaluated 
     by passing NL_LEFT/NL_RIGHT flags. A typical calling example is:

       NL_SFUN       sfn;
       NL_PARAMETER  u, v;
       NL_REAL       F;
       ...
       (define sfn, get u and v);
       ...
       N_SrfFuncEvalPt(&sfn,u,v,NL_LEFT,NL_RIGHT,&F);


   ACCESS:
   
     sfn     , input  ,  Surface function
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : u is in [u[j],u[j+1])
                           NL_RIGHT: u is in (u[j],u[j+1]]
     F       , output ,  Function value


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfFuncEvalPt( NL_SFUN *sfn, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_REAL *F )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfFuncEvalPt");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, usp, vsp, tmp;

    NL_DEGREE p, q;

    NL_REAL ** fuv, *NU, *NV, *tu;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SFuncGetKnots( sfn, &fuv, &NU, &NV );
    N_SFuncGetDegrees( sfn, &p, &q );
    N_SFuncGetKnotVectors( sfn, &knu, &knv );

    /* Check parameters */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute non-vanishing B-splines */

    NU = N_AllocReal1dArray( p, &S );

    if( NU EQ NULL )
        NL_QUIT;

    NV = N_AllocReal1dArray( q, &S );

    if( NV EQ NULL )
        NL_QUIT;

    error = N_BasisEval( knu, p, u, ufl, NU, &usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEval( knv, q, v, vfl, NV, &vsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the function value */

    tu = N_AllocReal1dArray( p, &S );

    if( tu EQ NULL )
        NL_QUIT;

    for ( i = 0; i <= p; i++ )
    {
        tmp = usp - p + i;
        tu[i] = 0.0;

        for ( j = 0; j <= q; j++ )
        {
            tu[i] += NV[j] * fuv[tmp][vsp - q + j];
        }
    }

    *F = 0.0;

    for ( i = 0; i <= p; i++ )
    {
        *F += NU[i] * tu[i];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfFuncEvalPt */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  computes  all derivatives  of a  surface function by 
     evaluating all non-vanishing basis functions and their derivatives, 
     and evaluating  the derivative surface. Discontinuous functions can 
     also be  handled by passing  NL_LEFT/NL_RIGHT flags.  Memory to store the
     derivatives  must be  allocated in  the calling  routine. A typical 
     calling example is:

       NL_SFUN       sfn;
       NL_PARAMETER  u, v;
       NL_INDEX      udr, vdr;
       NL_REAL       **FD;
       ...
       (define sfn, get u, v, udr vdr, and allocate memory for FD);
       ...
       N_SFuncDerivs(&sfn,u,v,NL_LEFT,NL_RIGHT,udr,vdr,FD);


   ACCESS:
   
     sfn     , input  ,  Surface function
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                                  (NL_RIGHT DERIVATIVES REQUIRED)
                           NL_RIGHT: t is in (t[j],t[j+1]]
                                  (NL_LEFT DERIVATIVES REQUIRED)
                           (t is either u or v)
     udr,vdr , input  ,  Highest derivatives required
     FD      , output ,  Derivatives; FD[k][l] is  the  k-th  derivative  
                         in the u-direction  and the l-th derivative  in  
                         the  v-direction. FD MUST HAVE ROOM TO STORE UP 
                         TO FD[udr][vdr].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SFuncDerivs( NL_SFUN *sfn, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_INDEX udr, NL_INDEX vdr, NL_REAL ** FD )
{
    NL_PRIVATE NL_STRING rname = _T("N_SFuncDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, usp, vsp, dru, drv, tmp;

    NL_DEGREE p, q;

    NL_REAL ** fuv, ** DU, ** DV, *tu;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SFuncGetKnots( sfn, &fuv, &tu, &tu );
    N_SFuncGetDegrees( sfn, &p, &q );
    N_SFuncGetKnotVectors( sfn, &knu, &knv );

    /* Check parameters */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute non-vanishing B-splines and their derivatives */

    dru = NL_MIN( udr, p );
    DU = N_AllocReal2dArray( dru, p, &S );

    if( DU EQ NULL )
        NL_QUIT;

    drv = NL_MIN( vdr, q );
    DV = N_AllocReal2dArray( drv, q, &S );

    if( DV EQ NULL )
        NL_QUIT;

    error = N_BasisDerivs( knu, p, u, ufl, dru, DU, &usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisDerivs( knv, q, v, vfl, drv, DV, &vsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Initialize derivative matrix */

    for ( k = 0; k <= udr; k++ )
    {
        for ( l = 0; l <= vdr; l++ )
        {
            FD[k][l] = 0.0;
        }
    }

    /* Compute derivatives */

    tu = N_AllocReal1dArray( p, &S );

    if( tu EQ NULL )
        NL_QUIT;

    for ( l = 0; l <= drv; l++ )
    {
        for ( i = 0; i <= p; i++ )
        {
            tmp = usp - p + i;
            tu[i] = 0.0;

            for ( j = 0; j <= q; j++ )
            {
                tu[i] += DV[l][j] * fuv[tmp][vsp - q + j];
            }
        }

        for ( k = 0; k <= dru; k++ )
        {
            for ( i = 0; i <= p; i++ )
            {
                FD[k][l] += DU[k][i] * tu[i];
            }
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SFuncDerivs */
