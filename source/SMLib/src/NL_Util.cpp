// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* NL_Util.c : Utility Function Definitions                                  */
/*****************************************************************************/

#include "StdAfx.h"



/* Optionally Run with VLD */
#ifdef CHECK_MEMORY
#include <wchar.h>
#include "C:\Program Files (x86)\Visual Leak Detector\include\vld.h"
static int nInitEnd = 0;
#endif

#include <nurbs.h>
#include <NL_Globals.h>
#include <memory.h>

#ifdef SM_USE_TBB_SCALABLE
#include "tbb/scalable_allocator.h"
#endif

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine initializes the NURBS programming environment. The
     current version sets the  stack pointers to NULL so that memory
     allocation becomes possible. A typical calling example is:
 
       NL_STACKS  S;
       ...
       N_InitNurbs(&S);
 
 
   ACCESS:
   
     S  , input  ,  Memory stack pointer
 
 
   RETURN CODES:
 
     None
 
   ***********************************************************************/

NL_VOID N_InitNurbs( NL_STACKS *S )
{

#ifdef CHECK_MEMORY
    nInitEnd++;
    {
        FILE* fp = N_FileOpen( _T("C:\\SMLib\\Bugs\\0203.Elemtech\\CountInitEnd.txt"), _T("a") );
        N_FPRINTF( fp, _T("nInitEnd = %d\n"), nInitEnd );
        N_FileClose( fp );
    }
#endif

    S->f1d = NULL;
    S->f2d = NULL;
    S->i1d = NULL;
    S->i2d = NULL;
    S->i3d = NULL;
    S->r1d = NULL;
    S->r2d = NULL;
    S->r3d = NULL;
    S->r4d = NULL;
    S->p1d = NULL;
    S->p2d = NULL;
    S->p3d = NULL;
    S->p4d = NULL;
    S->c1d = NULL;
    S->c2d = NULL;
    S->c3d = NULL;
    S->c4d = NULL;
    S->cur = NULL;
    S->cu2 = NULL;
    S->cu3 = NULL;
    S->cu4 = NULL;
    S->sur = NULL;
    S->vol = NULL;
    S->su2 = NULL;
    S->su3 = NULL;
    S->pol = NULL;
    S->ppl = NULL;
    S->pp2 = NULL;
    S->net = NULL;
    S->msh = NULL;
    S->knt = NULL;
    S->kn2 = NULL;
    S->cvl = NULL;
    S->svl = NULL;
    S->vvl = NULL;
    S->cfn = NULL;
    S->cf2 = NULL;
    S->sfn = NULL;
    S->sf2 = NULL;
    S->sf3 = NULL;
    S->ima = NULL;
    S->rma = NULL;
    S->pma = NULL;
    S->cma = NULL;
}

/*******************************************************************//**
   DESCRIPTION:

     This routine ends the NURBS programming environment. The current
     version  scans  through  the  linked lists of  memory stacks and
     deallocates  dynamically  allocated  memory. A  typical  calling
     example is:

       NL_STACKS  S;
       ...
       N_EndNurbs(&S);


   ACCESS:

     S  , input  ,  Memory stacks pointer


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_EndNurbs(NL_STACKS* S)
{
    NL_F1DNODE* f1p;

    NL_F2DNODE* f2p;

    NL_I1DNODE* i1p;

    NL_I2DNODE* i2p;

    NL_I3DNODE* i3p;

    NL_R1DNODE* r1p;

    NL_R2DNODE* r2p;

    NL_R3DNODE* r3p;

    NL_R4DNODE* r4p;

    NL_P1DNODE* p1p;

    NL_P2DNODE* p2p;

    NL_P3DNODE* p3p;

    NL_P4DNODE* p4p;

    NL_C1DNODE* c1p;

    NL_C2DNODE* c2p;

    NL_C3DNODE* c3p;

    NL_C4DNODE* c4p;

    NL_CURNODE* cup;

    NL_CU2NODE* u2p;

    NL_CU3NODE* u3p;

    NL_CU4NODE* u4p;

    NL_SURNODE* sup;

    NL_VOLNODE* vop;

    NL_SU2NODE* s2p;

    NL_SU3NODE* s3p;

    NL_POLNODE* pop;

    NL_PPLNODE* ppp;

    NL_PP2NODE* pp2p;

    NL_NETNODE* nep;

    NL_MESHNODE* mep;

    NL_KNTNODE* knp;

    NL_KN2NODE* k2p;

    NL_CVLNODE* cvp;

    NL_SVLNODE* svp;

    NL_VVLNODE* vvp;

    NL_CFNNODE* cfp;

    NL_CF2NODE* w2p;

    NL_SFNNODE* sfp;

    NL_SF2NODE* v2p;

    NL_SF3NODE* v3p;

    NL_IMANODE* imp;

    NL_RMANODE* rmp;

    NL_PMANODE* pmp;

    NL_CMANODE* cmp;

#ifdef CHECK_MEMORY
    nInitEnd--;
    {
        FILE* fp = N_FileOpen(_T("C:\\SMLib\\Bugs\\0203.Elemtech\\NINIT.txt"), _T("a"));
        N_FPRINTF(fp, _T("nInitEnd = %d\n"), nInitEnd);
        if (nInitEnd > 0)
            N_FPRINTF(fp, _T("MISSED END NURBS\n"));
        N_FileClose(fp);
    }
#endif

    /***********************************************************/
    /* Scan through each stack, free memory and kill the stack */
    /***********************************************************/

    /* Free flag arrays */

    while (S->f1d NEQ NULL)
    {
        f1p = S->f1d;
        S->f1d = f1p->next;
        N_Free(f1p->ptr);
        f1p->ptr = NULL;
        N_Free(f1p);
        f1p = NULL;
    }

    /* Free 2-D flag arrays */

    while (S->f2d NEQ NULL)
    {
        f2p = S->f2d;
        S->f2d = f2p->next;
        N_Free(f2p->ptr);
        f2p->ptr = NULL;
        N_Free(f2p);
        f2p = NULL;
    }

    /* Free integer arrays */

    while (S->i1d NEQ NULL)
    {
        i1p = S->i1d;
        S->i1d = i1p->next;
        N_Free(i1p->ptr);
        i1p->ptr = NULL;
        N_Free(i1p);
        i1p = NULL;
    }

    /* Free 2-D integer arrays */

    while (S->i2d NEQ NULL)
    {
        i2p = S->i2d;
        S->i2d = i2p->next;
        N_Free(i2p->ptr);
        i2p->ptr = NULL;
        N_Free(i2p);
        i2p = NULL;
    }

    /* Free 3-D integer arrays */

    while (S->i3d NEQ NULL)
    {
        i3p = S->i3d;
        S->i3d = i3p->next;
        N_Free(i3p->ptr);
        i3p->ptr = NULL;
        N_Free(i3p);
        i3p = NULL;
    }

    /* Free real arrays */

    while (S->r1d NEQ NULL)
    {
        r1p = S->r1d;
        S->r1d = r1p->next;
        N_Free(r1p->ptr);
        r1p->ptr = NULL;
        N_Free(r1p);
        r1p = NULL;
    }

    /* Free 2-D real arrays */

    while (S->r2d NEQ NULL)
    {
        r2p = S->r2d;
        S->r2d = r2p->next;
        N_Free(r2p->ptr);
        r2p->ptr = NULL;
        N_Free(r2p);
        r2p = NULL;
    }

    /* Free 3-D real arrays */

    while (S->r3d NEQ NULL)
    {
        r3p = S->r3d;
        S->r3d = r3p->next;
        N_Free(r3p->ptr);
        r3p->ptr = NULL;
        N_Free(r3p);
        r3p = NULL;
    }

    /* Free 4-D real arrays */

    while (S->r4d NEQ NULL)
    {
        r4p = S->r4d;
        S->r4d = r4p->next;
        N_Free(r4p->ptr);
        r4p->ptr = NULL;
        N_Free(r4p);
        r4p = NULL;
    }

    /* Free point arrays */

    while (S->p1d NEQ NULL)
    {
        p1p = S->p1d;
        S->p1d = p1p->next;
        N_Free(p1p->ptr);
        p1p->ptr = NULL;
        N_Free(p1p);
        p1p = NULL;
    }

    /* Free 2-D point arrays */

    while (S->p2d NEQ NULL)
    {
        p2p = S->p2d;
        S->p2d = p2p->next;
        N_Free(p2p->ptr);
        p2p->ptr = NULL;
        N_Free(p2p);
        p2p = NULL;
    }

    /* Free 3-D point arrays */

    while (S->p3d NEQ NULL)
    {
        p3p = S->p3d;
        S->p3d = p3p->next;
        N_Free(p3p->ptr);
        p3p->ptr = NULL;
        N_Free(p3p);
        p3p = NULL;
    }

    /* Free 4-D point arrays */

    while (S->p4d NEQ NULL)
    {
        p4p = S->p4d;
        S->p4d = p4p->next;
        N_Free(p4p->ptr);
        p4p->ptr = NULL;
        N_Free(p4p);
        p4p = NULL;
    }

    /* Free control point arrays */

    while (S->c1d NEQ NULL)
    {
        c1p = S->c1d;
        S->c1d = c1p->next;
        N_Free(c1p->ptr);
        c1p->ptr = NULL;
        N_Free(c1p);
        c1p = NULL;
    }

    /* Free 2-D control point arrays */

    while (S->c2d NEQ NULL)
    {
        c2p = S->c2d;
        S->c2d = c2p->next;
        N_Free(c2p->ptr);
        c2p->ptr = NULL;
        N_Free(c2p);
        c2p = NULL;
    }

    /* Free 3-D control point arrays */

    while (S->c3d NEQ NULL)
    {
        c3p = S->c3d;
        S->c3d = c3p->next;
        N_Free(c3p->ptr);
        c3p->ptr = NULL;
        N_Free(c3p);
        c3p = NULL;
    }

    /* Free 4-D control point arrays */

    while (S->c4d NEQ NULL)
    {
        c4p = S->c4d;
        S->c4d = c4p->next;
        N_Free(c4p->ptr);
        c4p->ptr = NULL;
        N_Free(c4p);
        c4p = NULL;
    }

    /* Free curves */

    while (S->cur NEQ NULL)
    {
        cup = S->cur;
        S->cur = cup->next;
        N_Free(cup->ptr);
        cup->ptr = NULL;
        N_Free(cup);
        cup = NULL;
    }

    /* Free 1-D curve pointers */

    while (S->cu2 NEQ NULL)
    {
        u2p = S->cu2;
        S->cu2 = u2p->next;
        N_Free(u2p->ptr);
        u2p->ptr = NULL;
        N_Free(u2p);
        u2p = NULL;
    }

    /* Free 2-D curve pointers */

    while (S->cu3 NEQ NULL)
    {
        u3p = S->cu3;
        S->cu3 = u3p->next;
        N_Free(u3p->ptr);
        u3p->ptr = NULL;
        N_Free(u3p);
        u3p = NULL;
    }

    /* Free 3-D curve pointers */

    while (S->cu4 NEQ NULL)
    {
        u4p = S->cu4;
        S->cu4 = u4p->next;
        N_Free(u4p->ptr);
        u4p->ptr = NULL;
        N_Free(u4p);
        u4p = NULL;
    }

    /* Free surfaces */

    while (S->sur NEQ NULL)
    {
        sup = S->sur;
        S->sur = sup->next;
        N_Free(sup->ptr);
        sup->ptr = NULL;
        N_Free(sup);
        sup = NULL;
    }

    /* Free volumes */

    while (S->vol NEQ NULL)
    {
        vop = S->vol;
        S->vol = vop->next;
        N_Free(vop->ptr);
        vop->ptr = NULL;
        N_Free(vop);
        vop = NULL;
    }

    /* Free 1-D surface pointers */

    while (S->su2 NEQ NULL)
    {
        s2p = S->su2;
        S->su2 = s2p->next;
        N_Free(s2p->ptr);
        s2p->ptr = NULL;
        N_Free(s2p);
        s2p = NULL;
    }

    /* Free 2-D surface pointers */

    while (S->su3 NEQ NULL)
    {
        s3p = S->su3;
        S->su3 = s3p->next;
        N_Free(s3p->ptr);
        s3p->ptr = NULL;
        N_Free(s3p);
        s3p = NULL;
    }

    /* Free control polygons */

    while (S->pol NEQ NULL)
    {
        pop = S->pol;
        S->pol = pop->next;
        N_Free(pop->ptr);
        pop->ptr = NULL;
        N_Free(pop);
        pop = NULL;
    }

    /* Free point polygons */

    while (S->ppl NEQ NULL)
    {
        ppp = S->ppl;
        S->ppl = ppp->next;
        N_Free(ppp->ptr);
        ppp->ptr = NULL;
        N_Free(ppp);
        ppp = NULL;
    }

    /* Free 1-D arrays of point polygons */

    while (S->pp2 NEQ NULL)
    {
        pp2p = S->pp2;
        S->pp2 = pp2p->next;
        N_Free(pp2p->ptr);
        pp2p->ptr = NULL;
        N_Free(pp2p);
        pp2p = NULL;
    }

    /* Free control nets */

    while (S->net NEQ NULL)
    {
        nep = S->net;
        S->net = nep->next;
        N_Free(nep->ptr);
        nep->ptr = NULL;
        N_Free(nep);
        nep = NULL;
    }

    /* Free control meshes */

    while (S->msh NEQ NULL)
    {
        mep = S->msh;
        S->msh = mep->next;
        N_Free(mep->ptr);
        mep->ptr = NULL;
        N_Free(mep);
        mep = NULL;
    }

    /* Free knot vectors */

    while (S->knt NEQ NULL)
    {
        knp = S->knt;
        S->knt = knp->next;
        N_Free(knp->ptr);
        knp->ptr = NULL;
        N_Free(knp);
        knp = NULL;
    }

    /* Free 1-D array of knot vectors */

    while (S->kn2 NEQ NULL)
    {
        k2p = S->kn2;
        S->kn2 = k2p->next;
        N_Free(k2p->ptr);
        k2p->ptr = NULL;
        N_Free(k2p);
        k2p = NULL;
    }

    /* Free curve values */

    while (S->cvl NEQ NULL)
    {
        cvp = S->cvl;
        S->cvl = cvp->next;
        N_Free(cvp->ptr);
        cvp->ptr = NULL;
        N_Free(cvp);
        cvp = NULL;
    }

    /* Free surface values */

    while (S->svl NEQ NULL)
    {
        svp = S->svl;
        S->svl = svp->next;
        N_Free(svp->ptr);
        svp->ptr = NULL;
        N_Free(svp);
        svp = NULL;
    }

    /* Free volume values */

    while (S->vvl NEQ NULL)
    {
        vvp = S->vvl;
        S->vvl = vvp->next;
        N_Free(vvp->ptr);
        vvp->ptr = NULL;
        N_Free(vvp);
        vvp = NULL;
    }

    /* Free curve functions */

    while (S->cfn NEQ NULL)
    {
        cfp = S->cfn;
        S->cfn = cfp->next;
        N_Free(cfp->ptr);
        cfp->ptr = NULL;
        N_Free(cfp);
        cfp = NULL;
    }

    /* Free 1-D curve function pointers */

    while (S->cf2 NEQ NULL)
    {
        w2p = S->cf2;
        S->cf2 = w2p->next;
        N_Free(w2p->ptr);
        w2p->ptr = NULL;
        N_Free(w2p);
        w2p = NULL;
    }

    /* Free surface functions */

    while (S->sfn NEQ NULL)
    {
        sfp = S->sfn;
        S->sfn = sfp->next;
        N_Free(sfp->ptr);
        sfp->ptr = NULL;
        N_Free(sfp);
        sfp = NULL;
    }

    /* Free 1-D surface function pointers */

    while (S->sf2 NEQ NULL)
    {
        v2p = S->sf2;
        S->sf2 = v2p->next;
        N_Free(v2p->ptr);
        v2p->ptr = NULL;
        N_Free(v2p);
        v2p = NULL;
    }

    /* Free 2-D surface function pointers */

    while (S->sf3 NEQ NULL)
    {
        v3p = S->sf3;
        S->sf3 = v3p->next;
        N_Free(v3p->ptr);
        v3p->ptr = NULL;
        N_Free(v3p);
        v3p = NULL;
    }

    /* Free integer matrices */

    while (S->ima NEQ NULL)
    {
        imp = S->ima;
        S->ima = imp->next;
        N_Free(imp->ptr);
        imp->ptr = NULL;
        N_Free(imp);
        imp = NULL;
    }

    /* Free real matrices */

    while (S->rma NEQ NULL)
    {
        rmp = S->rma;
        S->rma = rmp->next;
        N_Free(rmp->ptr);
        rmp->ptr = NULL;
        N_Free(rmp);
        rmp = NULL;
    }

    /* Free point matrices */

    while (S->pma NEQ NULL)
    {
        pmp = S->pma;
        S->pma = pmp->next;
        N_Free(pmp->ptr);
        pmp->ptr = NULL;
        N_Free(pmp);
        pmp = NULL;
    }

    /* Free control point matrices */

    while (S->cma NEQ NULL)
    {
        cmp = S->cma;
        S->cma = cmp->next;
        N_Free(cmp->ptr);
        cmp->ptr = NULL;
        N_Free(cmp);
        cmp = NULL;
    }

}      /* end N_EndNurbs */

/*******************************************************************//**
 
   DESCRIPTION:
 
     Pool based interface for N_Malloc - allocate memory block
     without initialization. Same as malloc() 

       NL_STACKS  S;
       N_InitNurbs(&S);

       void *ptr = N_Malloc(size)
 
   ACCESS:
   
      size    , input  , Size in bytes to allocate

 
   RETURN CODES:
 
     NL_STO_ERR when pool failed to allocate.
 
   ***********************************************************************/
void *N_Malloc( NL_INTEGER size)              /* in : desired memory size in bytes   */
{
    NL_PRIVATE NL_STRING rname = _T("N_Malloc");

    /*  init output */
    void *ret = NULL;

    /* get memory with malloc() */
#ifdef SM_USE_TBB_SCALABLE
    ret = (void *)scalable_malloc( (size_t)size );
#else
    ret = (void*)malloc((size_t)size);
#endif

    /*  respond to alloc failures - check state */

    if( ret == NULL )
    {
        N_ErrSet( NL_STO_ERR, rname );
    }

    /*  all done */
    return ret;
} /*  end N_Malloc */

/*******************************************************************//**
 
   DESCRIPTION:
 
     Pool based interface for calloc - allocate memory block
     and initialize it with 0 values. Same as calloc()

       NL_STACKS  S;
       N_InitNurbs(&S);

       void *ptr = N_Calloc(num, size)
 
   ACCESS:
   
       num,     input  , number of contiguous objects to allocate
       size,    input  , size in bytes of each object

 
   RETURN CODES:
 
     NL_STO_ERR when pool failed to allocate.
 
   ***********************************************************************/
void *N_Calloc( NL_INTEGER num,           /*  in : num of memory blocks to allocate */
                NL_INTEGER size)          /*  in : size of each block in bytes */
{
    NL_PRIVATE NL_STRING rname = _T("N_Calloc");

    /*  init output */
    void *ret = NULL;

    /* get memory with Calloc() */
#ifdef SM_USE_TBB_SCALABLE
    ret = (void*)scalable_calloc((size_t)num, (size_t)size);
#else
    ret = (void*)calloc((size_t)num, (size_t)size);
#endif

    /*  respond to alloc failures - check state */

    if( ret == NULL )
    {
        N_ErrSet( NL_STO_ERR, rname );
    }

    /*  all done */
    return ret;
} /*  end N_Calloc */

/*******************************************************************//**
 
   DESCRIPTION:
 
     Pool based interface for realloc - reallocate memory block
     while preserving old values. Same as realloc()

       NL_STACKS  S;
       N_InitNurbs(&S);

       void *ptr = N_Realloc(MemPtr, size )
 
   ACCESS:
   
       MemPtr,  input  , pointer to previously allocated memory block
       size,    input  , size in bytes of desired new block size

 
   RETURN CODES:
 
     NL_STO_ERR when pool failed to reallocate.
 
   ***********************************************************************/
void *N_Realloc( void *MemPtr,                  /*  in : pointer to existing block */
                 NL_INTEGER size )              /*  in : desired new size for existing block */
{
    NL_PRIVATE NL_STRING rname = _T("N_Realloc");

    /*  init output */
    void *ret = NULL;

    /*  - use realloc() call */
#ifdef SM_USE_TBB_SCALABLE
    ret = (void*)scalable_realloc(MemPtr, (size_t)size);
#else
    ret = (void*)realloc(MemPtr, (size_t)size);
#endif

    /*  respond to alloc failures -  check state */

    if( ret == NULL )
    {
        N_ErrSet( NL_STO_ERR, rname );
    }

    /*  all done */
    return ret;
} /*  end N_Realloc */

/*******************************************************************//**
 
   DESCRIPTION:
 
     Pool based interface for free - free memory block
     aallocated with N_Malloc, N_Calloc, and N_Realloc. 
     Same as free().

       NL_STACKS  S;
       N_InitNurbs(&S);

       N_Free(MemPtr); MemPtr = NULL;
 
   ACCESS:
   
       MemPtr,  input  , ptr to memory block to free
 
   RETURN CODES:
 
     none.
 
   ***********************************************************************/
void N_Free( void *MemPtr ) /*  in : pointer to memory block to be freed */
{
    /* use Free() */
#ifdef SM_USE_TBB_SCALABLE
    scalable_free(MemPtr);
#else
    free(MemPtr);
#endif

} /* end N_Free */

/*******************************************************************//**
 
   DESCRIPTION:
 
     Pool based interface for memset - set block of memory to given value
     Same as memset().

       NL_STACKS  S;
       N_InitNurbs(&S);

       N_MemSet(MemPtr, Val, SizeInBytes) ;

   ACCESS:
   
       MemPtr,      input  , ptr to memory block to set
       Val,         input  , char value to assign to values
       SizeInBytes, input, size of MemPtr block in bytes

 
   RETURN CODES:
 
     none.
 
   ***********************************************************************/
void N_MemSet
  (void *ptr,              /* in : pointer to memory block to be set */
   NL_INTEGER val,         /* in : set value                         */
   NL_INTEGER sizeInBytes) /* in : size in bytes of memory block     */
{
    /*  use memset */
    memset( ptr, val, sizeInBytes );

} /* end N_MemSet */


/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a flag array of given size.
     Proper  error check is  performed in case memory allocation fails. 
     A typical calling example is:

       NL_FLAG   *f;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get n);
       ...
       f = N_AllocFlag1dArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     f    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_FLAG *N_AllocFlag1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocFlag1dArray");

    NL_FLAG *f;

    NL_F1DNODE *f1d;

    /* Allocate memory for the array */

    f = (NL_FLAG *)N_Malloc( (n + 1) * sizeof( NL_FLAG ) );

    if( f EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    f1d = (NL_F1DNODE *)N_Malloc( sizeof( NL_F1DNODE ) );

    if( f1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( f );
        f = NULL;
        return NULL;
    }

    f1d->ptr = f;
    f1d->next = S->f1d;
    S->f1d = f1d;

    /* Exit */

    return f;
} /* end N_AllocFlag1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a  2-D flag array of given 
     size. Proper error check is  performed in case memory allocations 
     fail. A typical calling example is:

       NL_FLAG     **f;
       NL_INDEX    n, m;
       NL_STACKS   S;
       ...
       (get n and m);
       ...
       f = N_AllocFlag2dArray(n,m,&S);


   ACCESS:
   
     n,m  , input  ,  Highest indexes in array
     S    , input  ,  Memory stack pointer


   RETURN CODES:

     f    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_FLAG ** N_AllocFlag2dArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocFlag2dArray");

    NL_INDEX k, l;

    NL_FLAG ** f, *g;

    NL_F1DNODE *f1d;

    NL_F2DNODE *f2d;

    /* Allocate memories for the array */

    f = (NL_FLAG ** )N_Malloc( (n + 1) * sizeof( NL_FLAG * ) );

    if( f EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    g = (NL_FLAG *)N_Malloc( (n + 1) * (m + 1) * sizeof( NL_FLAG ) );

    if( g EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( f );
        f = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        f[k] = &g[l];
        l = l + m + 1;
    }

    /* Put pointers on memory stacks */

    f1d = (NL_F1DNODE *)N_Malloc( sizeof( NL_F1DNODE ) );

    if( f1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( f );
        f = NULL;
        N_Free( g );
        g = NULL;
        return NULL;
    }

    f2d = (NL_F2DNODE *)N_Malloc( sizeof( NL_F2DNODE ) );

    if( f2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( f );
        f = NULL;
        N_Free( g );
        g = NULL;
        N_Free( f1d );
        f1d = NULL;
        return NULL;
    }

    f1d->ptr = g;
    f1d->next = S->f1d;
    S->f1d = f1d;

    f2d->ptr = f;
    f2d->next = S->f2d;
    S->f2d = f2d;

    /* Exit */

    return f;
} /* end N_AllocFlag2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine  allocates  memory to  store a 1-D array  of flag
     pointers.  Proper error  check is  performed  in  case  memory 
     allocations fail. A typical calling example is:

       NL_FLAG    **f;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get n);
       ...
       f = N_AllocFlagPtr1dArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stacks pointer


   RETURN CODES:

     f    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_FLAG ** N_AllocFlagPtr1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocFlagPtr1dArray");

    NL_FLAG ** f;

    NL_F2DNODE *f2d;

    /* Allocate memory */

    f = (NL_FLAG ** )N_Malloc( (n + 1) * sizeof( NL_FLAG * ) );

    if( f EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stacks */

    f2d = (NL_F2DNODE *)N_Malloc( sizeof( NL_F2DNODE ) );

    if( f2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( f );
        f = NULL;
        return NULL;
    }

    f2d->ptr = f;
    f2d->next = S->f2d;
    S->f2d = f2d;

    /* Exit */

    return f;
}

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store an integer array of given
     size. Proper  error check is performed in case memory allocation 
     fails. A typical calling example is:

       NL_INTEGER  *i;
       NL_INDEX    n;
       NL_STACKS   S;
       ...
       (get n);
       ...
       i = N_AllocInt1dArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     i    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_INTEGER *N_AllocInt1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocInt1dArray");

    NL_INTEGER *i;

    NL_I1DNODE *i1d;

    /* Allocate memory for the array */

    i = (NL_INTEGER *)N_Malloc( (n + 1) * sizeof( NL_INTEGER ) );

    if( i EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    i1d = (NL_I1DNODE *)N_Malloc( sizeof( NL_I1DNODE ) );

    if( i1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( i );
        i = NULL;
        return NULL;
    }

    i1d->ptr = i;
    i1d->next = S->i1d;
    S->i1d = i1d;

    /* Exit */

    return i;
} /* end N_AllocInt1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a  2-D integer array of 
     given  size. Proper  error check is  performed  in case memory 
     allocations fail. A typical calling example is:

       NL_INTEGER  **i;
       NL_INDEX    n, m;
       NL_STACKS   S;
       ...
       (get n and m);
       ...
       i = N_AllocInt2dArray(n,m,&S);


   ACCESS:
   
     n,m  , input  ,  Highest indexes in array
     S    , input  ,  Memory stack pointer


   RETURN CODES:

     i    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_INTEGER ** N_AllocInt2dArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocInt2dArray");

    NL_INDEX k, l;

    NL_INTEGER ** i, *j;

    NL_I1DNODE *i1d;

    NL_I2DNODE *i2d;

    /* Allocate memories for the array */

    i = (NL_INTEGER ** )N_Malloc( (n + 1) * sizeof( NL_INTEGER * ) );

    if( i EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    j = (NL_INTEGER *)N_Malloc( (n + 1) * (m + 1) * sizeof( NL_INTEGER ) );

    if( j EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( i );
        i = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        i[k] = &j[l];
        l = l + m + 1;
    }

    /* Put pointers on memory stacks */

    i1d = (NL_I1DNODE *)N_Malloc( sizeof( NL_I1DNODE ) );

    if( i1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( i );
        i = NULL;
        N_Free( j );
        j = NULL;
        return NULL;
    }

    i2d = (NL_I2DNODE *)N_Malloc( sizeof( NL_I2DNODE ) );

    if( i2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( i );
        i = NULL;
        N_Free( j );
        j = NULL;
        N_Free( i1d );
        i1d = NULL;
        return NULL;
    }

    i1d->ptr = j;
    i1d->next = S->i1d;
    S->i1d = i1d;

    i2d->ptr = i;
    i2d->next = S->i2d;
    S->i2d = i2d;

    /* Exit */

    return i;
} /* end N_AllocInt2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory to store a 1-D array of integer 
     pointers.  Proper error  check is  performed  in  case  memory 
     allocations fail. A typical calling example is:

       NL_INTEGER  **i;
       NL_INDEX    n;
       NL_STACKS   S;
       ...
       (get n);
       ...
       i = N_AllocIntPtr1dArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stacks pointer


   RETURN CODES:

     i    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_INTEGER ** N_AllocIntPtr1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocIntPtr1dArray");

    NL_INTEGER ** i;

    NL_I2DNODE *i2d;

    /* Allocate memory */

    i = (NL_INTEGER ** )N_Malloc( (n + 1) * sizeof( NL_INTEGER * ) );

    if( i EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stacks */

    i2d = (NL_I2DNODE *)N_Malloc( sizeof( NL_I2DNODE ) );

    if( i2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( i );
        i = NULL;
        return NULL;
    }

    i2d->ptr = i;
    i2d->next = S->i2d;
    S->i2d = i2d;

    /* Exit */

    return i;
} /* end N_AllocIntPtr1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory to store a 2-D array of integer 
     pointers.  Proper error  check is  performed  in  case  memory 
     allocations fail. A typical calling example is:

       NL_INTEGER  ***i;
       NL_INDEX    n, m;
       NL_STACKS   S;
       ...
       (get n and m);
       ...
       i = N_AllocIntPtr2dArray(n,m,&S);


   ACCESS:
   
     n,m , input  ,  Highest indexes in array
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     i    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_INTEGER *** N_AllocIntPtr2dArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocIntPtr2dArray");

    NL_INDEX k, l;

    NL_INTEGER *** i, ** j;

    NL_I2DNODE *i2d;

    NL_I3DNODE *i3d;

    /* Allocate memory */

    i = (NL_INTEGER *** )N_Malloc( (n + 1) * sizeof( NL_INTEGER ** ) );

    if( i EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    j = (NL_INTEGER ** )N_Malloc( (n + 1) * (m + 1) * sizeof( NL_INTEGER * ) );

    if( j EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( i );
        i = NULL;
        return NULL;
    }

    /* Make pointer asignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        i[k] = &j[l];
        l = l + m + 1;
    }

    /* Put pointers on memory stacks */

    i2d = (NL_I2DNODE *)N_Malloc( sizeof( NL_I2DNODE ) );

    if( i2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( i );
        i = NULL;
        N_Free( j );
        j = NULL;
        return NULL;
    }

    i3d = (NL_I3DNODE *)N_Malloc( sizeof( NL_I3DNODE ) );

    if( i3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( i );
        i = NULL;
        N_Free( j );
        j = NULL;
        N_Free( i2d );
        i2d = NULL;
        return NULL;
    }

    i2d->ptr = j;
    i2d->next = S->i2d;
    S->i2d = i2d;

    i3d->ptr = i;
    i3d->next = S->i3d;
    S->i3d = i3d;

    /* Exit */

    return i;
} /* end N_AllocIntPtr2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to  store a real  array of given
     size. Proper  error check is performed in case memory allocation 
     fails. A typical calling example is:

       NL_REAL    *r;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get n);
       ...
       r = N_AllocReal1dArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     r    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_REAL *N_AllocReal1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocReal1dArray");

    NL_REAL *r;

    NL_R1DNODE *r1d;

    /* Allocate memory for the array */

    r = (NL_REAL *)N_Malloc( (n + 1) * sizeof( NL_REAL ) );

    if( r EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    r1d = (NL_R1DNODE *)N_Malloc( sizeof( NL_R1DNODE ) );

    if( r1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        return NULL;
    }

    r1d->ptr = r;
    r1d->next = S->r1d;
    S->r1d = r1d;

    /* Exit */

    return r;
} /* end N_AllocReal1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory to store a 2-D real array of given
     size. Proper  error check is performed in case memory allocations 
     fail. A typical calling example is:

       NL_REAL    **r;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get n and m);
       ...
       r = N_AllocReal2dArray(n,m,&S);


   ACCESS:
   
     n,m  , input  ,  Highest indexes in array
     S    , input  ,  Memory stacks pointer


   RETURN CODES:

     r    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_REAL ** N_AllocReal2dArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocReal2dArray");
    NL_INDEX k, l;

    NL_REAL ** r, *s;

    NL_R1DNODE *r1d;

    NL_R2DNODE *r2d;

    /* Allocate memories for the array */

    r = (NL_REAL ** )N_Malloc( (n + 1) * sizeof( NL_REAL * ) );

    if( r EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    s = (NL_REAL *)N_Malloc( (n + 1) * (m + 1) * sizeof( NL_REAL ) );

    if( s EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        r[k] = &s[l];
        l = l + m + 1;
    }

    /* Put pointers on memory stacks */

    r1d = (NL_R1DNODE *)N_Malloc( sizeof( NL_R1DNODE ) );

    if( r1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        return NULL;
    }

    r2d = (NL_R2DNODE *)N_Malloc( sizeof( NL_R2DNODE ) );

    if( r2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( r1d );
        r1d = NULL;
        return NULL;
    }

    r1d->ptr = s;
    r1d->next = S->r1d;
    S->r1d = r1d;

    r2d->ptr = r;
    r2d->next = S->r2d;
    S->r2d = r2d;

    /* Exit */

    return r;
} /* end N_AllocReal2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory to store a 3-D real array of given
     size. Proper  error check is performed in case memory allocations 
     fail. A typical calling example is:

       NL_REAL    ***r;
       NL_INDEX   i1, i2, i3;
       NL_STACKS  S;
       ...
       (get i1, i2, and i3);
       ...
       r = N_AllocReal3dArray(i1,i2,i3,&S);


   ACCESS:
   
     i1,i2,i3 , input  ,  Highest indexes in array
     S        , input  ,  Memory stacks pointer


   RETURN CODES:

     r    : Pointer to 3-D array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

/* NL_REAL  ***N_AllocReal3dArray */
NL_REAL *** N_AllocReal3dArray( NL_INDEX i1, NL_INDEX i2, NL_INDEX i3, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocReal3dArray");
    NL_INDEX i, j, l2, l3;

    NL_REAL *** r, ** s, *t;

    NL_R1DNODE *r1d;

    NL_R2DNODE *r2d;

    NL_R3DNODE *r3d;

    /* Allocate memory for the array */

    r = (NL_REAL *** )N_Malloc( (i1 + 1) * sizeof( NL_REAL ** ) );

    if( r EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    s = (NL_REAL ** )N_Malloc( (i1 + 1) * (i2 + 1) * sizeof( NL_REAL * ) );

    if( s EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        return NULL;
    }

    t = (NL_REAL *)N_Malloc( (i1 + 1) * (i2 + 1) * (i3 + 1) * sizeof( NL_REAL ) );

    if( t EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l2 = l3 = 0;

    for ( i = 0; i <= i1; i++ )
    {
        r[i] = &s[l2];

        for ( j = 0; j <= i2; j++ )
        {
            s[l2 + j] = &t[l3];
            l3 = l3 + i3 + 1;
        }
        l2 = l2 + i2 + 1;
    }

    /* Put pointers on memory stacks */

    r1d = (NL_R1DNODE *)N_Malloc( sizeof( NL_R1DNODE ) );

    if( r1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( t );
        t = NULL;
        return NULL;
    }

    r2d = (NL_R2DNODE *)N_Malloc( sizeof( NL_R2DNODE ) );

    if( r2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( t );
        t = NULL;
        N_Free( r1d );
        r1d = NULL;
        return NULL;
    }

    r3d = (NL_R3DNODE *)N_Malloc( sizeof( NL_R3DNODE ) );

    if( r3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( t );
        t = NULL;
        N_Free( r1d );
        r1d = NULL;
        N_Free( r2d );
        r2d = NULL;
        return NULL;
    }

    r1d->ptr = t;
    r1d->next = S->r1d;
    S->r1d = r1d;

    r2d->ptr = s;
    r2d->next = S->r2d;
    S->r2d = r2d;

    r3d->ptr = r;
    r3d->next = S->r3d;
    S->r3d = r3d;

    /* Exit */

    return r;
} /* end N_AllocReal3dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory to store a 1-D array of real 
     pointers.  Proper error  check is performed  in case memory 
     allocations fail. A typical calling example is:

       NL_REAL    **r;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get n);
       ...
       r = N_AllocRealPtr1dArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stacks pointer


   RETURN CODES:

     r    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

/* NL_REAL  **N_AllocRealPtr1dArray */
NL_REAL ** N_AllocRealPtr1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocRealPtr1dArray");

    NL_REAL ** r;

    NL_R2DNODE *r2d;

    /* Allocate memory */

    r = (NL_REAL ** )N_Malloc( (n + 1) * sizeof( NL_REAL * ) );

    if( r EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stacks */

    r2d = (NL_R2DNODE *)N_Malloc( sizeof( NL_R2DNODE ) );

    if( r2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        return NULL;
    }

    r2d->ptr = r;
    r2d->next = S->r2d;
    S->r2d = r2d;

    /* Exit */

    return r;
} /* end N_AllocRealPtr1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  allocates  memory to  store a 2-D array of real 
     pointers.  Proper error  check is  performed  in  case  memory 
     allocations fail. A typical calling example is:

       NL_REAL    ***r;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get n and m);
       ...
       r = N_AllocRealPtr2dArray(n,m,&S);


   ACCESS:
   
     n,m , input  ,  Highest indexes in array
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     r    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_REAL *** N_AllocRealPtr2dArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocRealPtr2dArray");

    NL_INDEX k, l;

    NL_REAL *** r, ** s;

    NL_R2DNODE *r2d;

    NL_R3DNODE *r3d;

    /* Allocate memory */

    r = (NL_REAL *** )N_Malloc( (n + 1) * sizeof( NL_REAL ** ) );

    if( r EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    s = (NL_REAL ** )N_Malloc( (n + 1) * (m + 1) * sizeof( NL_REAL * ) );

    if( s EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        return NULL;
    }

    /* Make pointer asignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        r[k] = &s[l];
        l = l + m + 1;
    }

    /* Put pointers on memory stacks */

    r2d = (NL_R2DNODE *)N_Malloc( sizeof( NL_R2DNODE ) );

    if( r2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        return NULL;
    }

    r3d = (NL_R3DNODE *)N_Malloc( sizeof( NL_R3DNODE ) );

    if( r3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( r2d );
        r2d = NULL;
        return NULL;
    }

    r2d->ptr = s;
    r2d->next = S->r2d;
    S->r2d = r2d;

    r3d->ptr = r;
    r3d->next = S->r3d;
    S->r3d = r3d;

    /* Exit */

    return r;
}

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to store a 1-D array of NL_REAL 
     pointers.  Proper error  check is  performed  in  case  memory 
     allocations fail. A typical calling example is:

       NL_REAL    ***r;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get n);
       ...
       r = N_AllocRealPtr3dArray(n,&S);


   ACCESS:
   
     n , input  ,  Highest index in array
     S , input  ,  Memory stacks pointer


   RETURN CODES:

     r    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_REAL *** N_AllocRealPtr3dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocRealPtr3dArray");

    NL_REAL *** r;

    NL_R3DNODE *r3d;

    /* Allocate memory */

    r = (NL_REAL *** )N_Malloc( (n + 1) * sizeof( NL_REAL ** ) );

    if( r EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stacks */

    r3d = (NL_R3DNODE *)N_Malloc( sizeof( NL_R3DNODE ) );

    if( r3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        return NULL;
    }

    r3d->ptr = r;
    r3d->next = S->r3d;
    S->r3d = r3d;

    /* Exit */

    return r;
} /* end N_AllocRealPtr3dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory to store a 4-D real array of given
     size. Proper  error check is performed in case memory allocations 
     fail. A typical calling example is:

       NL_REAL    ****r;
       NL_INDEX   i1, i2, i3, i4;
       NL_STACKS  S;
       ...
       (get i1, i2, i3 and i4);
       ...
       r = N_AllocReal4dArray(i1,i2,i3,i4,&S);


   ACCESS:
   
     i1,i2,i3,i4 , input  ,  Highest indexes in array
     S           , input  ,  Memory stacks pointer


   RETURN CODES:

     r    : Pointer to 4-D array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_REAL **** N_AllocReal4dArray( NL_INDEX i1, NL_INDEX i2, NL_INDEX i3, NL_INDEX i4, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocReal4dArray");
    NL_INDEX i, j, k, l2, l3, l4;

    NL_REAL **** r, *** s, ** t, *u;

    NL_R1DNODE *r1d;

    NL_R2DNODE *r2d;

    NL_R3DNODE *r3d;

    NL_R4DNODE *r4d;

    /* Allocate memories for the array */

    r = (NL_REAL **** )N_Malloc( (i1 + 1) * sizeof( NL_REAL *** ) );

    if( r EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    s = (NL_REAL *** )N_Malloc( (i1 + 1) * (i2 + 1) * sizeof( NL_REAL ** ) );

    if( s EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        return NULL;
    }

    t = (NL_REAL ** )N_Malloc( (i1 + 1) * (i2 + 1) * (i3 + 1) * sizeof( NL_REAL * ) );

    if( t EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        return NULL;
    }

    u = (NL_REAL *)N_Malloc( (i1 + 1) * (i2 + 1) * (i3 + 1) * (i4 + 1) * sizeof( NL_REAL ) );

    if( u EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( t );
        t = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l2 = l3 = l4 = 0;

    for ( i = 0; i <= i1; i++ )
    {
        r[i] = &s[l2];

        for ( j = 0; j <= i2; j++ )
        {
            s[l2 + j] = &t[l3];

            for ( k = 0; k <= i3; k++ )
            {
                t[l3 + k] = &u[l4];
                l4 = l4 + i4 + 1;
            }
            l3 = l3 + i3 + 1;
        }
        l2 = l2 + i2 + 1;
    }

    /* Put pointers on memory stacks */

    r1d = (NL_R1DNODE *)N_Malloc( sizeof( NL_R1DNODE ) );

    if( r1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( t );
        t = NULL;
        N_Free( u );
        u = NULL;
        return NULL;
    }

    r2d = (NL_R2DNODE *)N_Malloc( sizeof( NL_R2DNODE ) );

    if( r2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( t );
        t = NULL;
        N_Free( u );
        u = NULL;
        N_Free( r1d );
        r1d = NULL;
        return NULL;
    }

    r3d = (NL_R3DNODE *)N_Malloc( sizeof( NL_R3DNODE ) );

    if( r3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( t );
        t = NULL;
        N_Free( u );
        u = NULL;
        N_Free( r1d );
        r1d = NULL;
        N_Free( r2d );
        r2d = NULL;
        return NULL;
    }

    r4d = (NL_R4DNODE *)N_Malloc( sizeof( NL_R4DNODE ) );

    if( r4d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( r );
        r = NULL;
        N_Free( s );
        s = NULL;
        N_Free( t );
        t = NULL;
        N_Free( u );
        u = NULL;
        N_Free( r1d );
        r1d = NULL;
        N_Free( r2d );
        r2d = NULL;
        N_Free( r3d );
        r3d = NULL;
        return NULL;
    }

    r1d->ptr = u;
    r1d->next = S->r1d;
    S->r1d = r1d;

    r2d->ptr = t;
    r2d->next = S->r2d;
    S->r2d = r2d;

    r3d->ptr = s;
    r3d->next = S->r3d;
    S->r3d = r3d;

    r4d->ptr = r;
    r4d->next = S->r4d;
    S->r4d = r4d;

    /* Exit */

    return r;
} /* end N_AllocReal4dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine  allocates memory  to store a  point array of given
     size. Proper  error check is performed in case memory allocation 
     fails. A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get n);
       ...
       P = N_AllocPt1dArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stacks pointer


   RETURN CODES:

     P    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

/* NL_POINT  *N_AllocPt1dArray */
NL_POINT *N_AllocPt1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocPt1dArray");

    NL_POINT *P;

    NL_P1DNODE *p1d;

    /* Allocate memory for the array */

    P = (NL_POINT *)N_Malloc( (n + 1) * sizeof( NL_POINT ) );

    if( P EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    p1d = (NL_P1DNODE *)N_Malloc( sizeof( NL_P1DNODE ) );

    if( p1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        return NULL;
    }

    p1d->ptr = P;
    p1d->next = S->p1d;
    S->p1d = p1d;

    /* Exit */

    return P;
} /* end N_AllocPt1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a 2-D point array of given
     size. Proper  error check is performed in case memory allocations 
     fail. A typical calling example is:

       NL_POINT   **P;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get n and m);
       ...
       P = N_AllocPt2dArray(n,m,&S);


   ACCESS:
   
     n,m  , input  ,  Highest indexes in array
     S    , input  ,  Memory stacks pointer


   RETURN CODES:

     P    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

/* NL_POINT  **N_AllocPt2dArray */
NL_POINT ** N_AllocPt2dArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocPt2dArray");

    NL_INDEX k, l;

    NL_POINT ** P, *Q;

    NL_P1DNODE *p1d;

    NL_P2DNODE *p2d;

    /* Allocate memories for the array */

    P = (NL_POINT ** )N_Malloc( (n + 1) * sizeof( NL_POINT * ) );

    if( P EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    Q = (NL_POINT *)N_Malloc( (n + 1) * (m + 1) * sizeof( NL_POINT ) );

    if( Q EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        P[k] = &Q[l];
        l = l + m + 1;
    }

    /* Put pointers on memory stacks */

    p1d = (NL_P1DNODE *)N_Malloc( sizeof( NL_P1DNODE ) );

    if( p1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        N_Free( Q );
        Q = NULL;
        return NULL;
    }

    p2d = (NL_P2DNODE *)N_Malloc( sizeof( NL_P2DNODE ) );

    if( p2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        N_Free( Q );
        Q = NULL;
        N_Free( p1d );
        p1d = NULL;
        return NULL;
    }

    p1d->ptr = Q;
    p1d->next = S->p1d;
    S->p1d = p1d;

    p2d->ptr = P;
    p2d->next = S->p2d;
    S->p2d = p2d;

    /* Exit */

    return P;
} /* end N_AllocPt2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a 3-D point array of given
     size. Proper  error check is performed in case memory allocations 
     fail. A typical calling example is:

       NL_POINT   ***P;
       NL_INDEX   m,n,o;
       NL_STACKS  S;
       ...
       (get m,n and o);
       ...
       P = N_AllocPt2dArray(m,n,o,&S);


   ACCESS:
   
     m,n,o  , input  ,  Highest indexes in array
     S      , input  ,  Memory stacks pointer


   RETURN CODES:

     P    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_POINT *** N_AllocPt3dArray( NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocPt3dArray");

    NL_INDEX k, l;

    NL_POINT *** P, ** Q, *R;

    NL_P1DNODE *p1d;

    NL_P2DNODE *p2d;

    NL_P3DNODE *p3d;

    /* Allocate memories for the array */

    P = (NL_POINT *** )N_Malloc( (m + 1) * sizeof( NL_POINT ** ) );

    if( P EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    Q = (NL_POINT ** )N_Malloc( (m + 1) * (n + 1) * sizeof( NL_POINT * ) );

    if( Q EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        return NULL;
    }

    R = (NL_POINT *)N_Malloc( (m + 1) * (n + 1) * (o + 1) * sizeof( NL_POINT ) );

    if( Q EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l = 0;

    for ( k = 0; k <= m; k++ )
    {
        P[k] = &Q[l];
        l = l + n + 1;
    }

    l = 0;

    for ( k = 0; k < (m + 1) * (n + 1); k++ )
    {
        Q[k] = &R[l];
        l = l + o + 1;
    }

    /* Put pointers on memory stacks */

    p1d = (NL_P1DNODE *)N_Malloc( sizeof( NL_P1DNODE ) );

    if( p1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        N_Free( Q );
        Q = NULL;
        N_Free( R );
        R = NULL;
        return NULL;
    }

    p2d = (NL_P2DNODE *)N_Malloc( sizeof( NL_P2DNODE ) );

    if( p2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        N_Free( Q );
        Q = NULL;
        N_Free( R );
        R = NULL;
        N_Free( p1d );
        p1d = NULL;
        return NULL;
    }

    p3d = (NL_P3DNODE *)N_Malloc( sizeof( NL_P3DNODE ) );

    if( p3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        N_Free( Q );
        Q = NULL;
        N_Free( R );
        R = NULL;
        N_Free( p1d );
        p1d = NULL;
        N_Free( p2d );
        p2d = NULL;
        return NULL;
    }

    p1d->ptr = R;
    p1d->next = S->p1d;
    S->p1d = p1d;

    p2d->ptr = Q;
    p2d->next = S->p2d;
    S->p2d = p2d;

    p3d->ptr = P;
    p3d->next = S->p3d;
    S->p3d = p3d;

    /* Exit */

    return P;
} /* end N_AllocPt3dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a control point array of 
     given  size. Proper  error  check is  performed in  case memory 
     allocation fails. A typical calling example is:

       NL_CPOINT  *Pw;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get n);
       ...
       Pw = N_AllocCPt1dArray(n,&S);


   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     Pw   : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

/* NL_CPOINT  *N_AllocCPt1dArray */
NL_CPOINT *N_AllocCPt1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCPt1dArray");

    NL_CPOINT *Pw;

    NL_C1DNODE *c1d;

    /* Allocate memory for the array */

    Pw = (NL_CPOINT *)N_Malloc( (n + 1) * sizeof( NL_CPOINT ) );

    if( Pw EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    c1d = (NL_C1DNODE *)N_Malloc( sizeof( NL_C1DNODE ) );

    if( c1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        return NULL;
    }

    c1d->ptr = Pw;
    c1d->next = S->c1d;
    S->c1d = c1d;

    /* Exit */

    return Pw;
} /* end N_AllocCPt1dArray */



/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a 2-D control point array 
     of given  size. Proper error check is  performed in  case memory 
     allocations fail. A typical calling example is:

       NL_CPOINT  **Pw;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get n and m);
       ...
       Pw = N_AllocCPt2dArray(n,m,&S);


   ACCESS:
   
     n,m  , input  ,  Highest indexes in array
     S    , input  ,  Memory stacks pointer


   RETURN CODES:

     Pw   : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

/* NL_CPOINT  **N_AllocCPt2dArray */
NL_CPOINT ** N_AllocCPt2dArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCPt2dArray");

    NL_INDEX k, l;

    NL_CPOINT ** Pw, *Qw;

    NL_C1DNODE *c1d;

    NL_C2DNODE *c2d;

    /* Allocate memory for the array */

    Pw = (NL_CPOINT ** )N_Malloc( (n + 1) * sizeof( NL_CPOINT * ) );

    if( Pw EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    Qw = (NL_CPOINT *)N_Malloc( (n + 1) * (m + 1) * sizeof( NL_CPOINT ) );

    if( Qw EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        Pw[k] = &Qw[l];
        l = l + m + 1;
    }

    /* Put pointers on memory stacks */

    c1d = (NL_C1DNODE *)N_Malloc( sizeof( NL_C1DNODE ) );

    if( c1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        N_Free( Qw );
        Qw = NULL;
        return NULL;
    }

    c2d = (NL_C2DNODE *)N_Malloc( sizeof( NL_C2DNODE ) );

    if( c2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        N_Free( Qw );
        Qw = NULL;
        N_Free( c1d );
        c1d = NULL;
        return NULL;
    }

    c1d->ptr = Qw;
    c1d->next = S->c1d;
    S->c1d = c1d;

    c2d->ptr = Pw;
    c2d->next = S->c2d;
    S->c2d = c2d;

    /* Exit */

    return Pw;
} /* end N_AllocCPt2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a 3-D control point array 
     of given  size. Proper error check is  performed in  case memory 
     allocations fail. A typical calling example is:

       NL_CPOINT  **Pw;
       NL_INDEX   m, n, o;
       NL_STACKS  S;
       ...
       (get m, n, and o);
       ...
       Pw = N_AllocCPt2dArray(m,n,o,&S);


   ACCESS:
   
     m,n,o  , input  ,  Highest indexes in array
     S      , input  ,  Memory stacks pointer


   RETURN CODES:

     Pw   : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CPOINT *** N_AllocCPt3dArray( NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCPt3dArray");

    NL_INDEX k, l;

    NL_CPOINT *** Pw, ** Qw, *Rw;

    NL_C1DNODE *c1d;

    NL_C2DNODE *c2d;

    NL_C3DNODE *c3d;

    /* Allocate memory for the array */

    Pw = (NL_CPOINT *** )N_Malloc( (m + 1) * sizeof( NL_CPOINT ** ) );

    if( Pw EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    Qw = (NL_CPOINT ** )N_Malloc( (m + 1) * (n + 1) * sizeof( NL_CPOINT * ) );

    if( Qw EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        return NULL;
    }

    Rw = (NL_CPOINT *)N_Malloc( (m + 1) * (n + 1) * (o + 1) * sizeof( NL_CPOINT ) );

    if( Rw EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        N_Free( Qw );
        Qw = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l = 0;

    for ( k = 0; k <= m; k++ )
    {
        Pw[k] = &Qw[l];
        l = l + n + 1;
    }

    l = 0;

    for ( k = 0; k < (m + 1) * (n + 1); k++ )
    {
        Qw[k] = &Rw[l];
        l = l + o + 1;
    }

    /* Put pointers on memory stacks */

    c1d = (NL_C1DNODE *)N_Malloc( sizeof( NL_C1DNODE ) );

    if( c1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        N_Free( Qw );
        Qw = NULL;
        N_Free( Rw );
        Rw = NULL;
        return NULL;
    }

    c2d = (NL_C2DNODE *)N_Malloc( sizeof( NL_C2DNODE ) );

    if( c2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        N_Free( Qw );
        Qw = NULL;
        N_Free( Rw );
        Rw = NULL;
        N_Free( c1d );
        c1d = NULL;
        return NULL;
    }

    c3d = (NL_C3DNODE *)N_Malloc( sizeof( NL_C3DNODE ) );

    if( c3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        N_Free( Qw );
        Qw = NULL;
        N_Free( Rw );
        Rw = NULL;
        N_Free( c1d );
        c1d = NULL;
        N_Free( c2d );
        c2d = NULL;
        return NULL;
    }

    c1d->ptr = Rw;
    c1d->next = S->c1d;
    S->c1d = c1d;

    c2d->ptr = Qw;
    c2d->next = S->c2d;
    S->c2d = c2d;

    c3d->ptr = Pw;
    c3d->next = S->c3d;
    S->c3d = c3d;

    /* Exit */

    return Pw;
} /* end N_AllocCPt3dArray */

/*******************************************************************//**


   DESCRIPTION:

     Given an array of points defined by the x, y and z  coordinate 
     arrays. This routine converts this input to  point input, i.e.  
     intstead of using <x[i],y[i],z[i]>, the more object based P[i]  
     is used, where P[i] is the ith point object. A typical calling
     example is:

       NL_POINT   *P;
       NL_REAL    *x, *y, *z;
       NL_INDEX   n;
       NL_STACKS  S;
       ...
       (get memory for x, y and z, and define coordinate values);
       ...
       P = N_XYZToPtArray(x,y,z,n,&S);


   ACCESS:
   
     x,y,z , input  ,  Point coordinates
     n     , input  ,  Highest index in <x,y,z>
     S     , input  ,  Memory stack pointer


   RETURN CODES:

     P    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_POINT *N_XYZToPtArray( NL_REAL *x, NL_REAL *y, NL_REAL *z, NL_INDEX n, NL_STACKS *S )
{
    NL_INDEX i;

    NL_POINT *P;

    /* Allocate memory for the array */

    P = N_AllocPt1dArray( n, S );

    if( P EQ NULL )
        return NULL;

    /* Fill in the array */

    for ( i = 0; i <= n; i++ )
    {
        N_PtFromXYZ( x[i], y[i], z[i], &P[i] );
    }

    /* Exit */

    return P;
} /* end N_XYZToPtArray */

/*******************************************************************//**


   DESCRIPTION:

     Given a 2-D array of  points in 3-D defined  by the x,  y and z 
     coordinate  arrays. This  routine converts  this input to point 
     input, i.e. intstead  of  using  <x[i][j],y[i][j],z[i][j]>, the
     more object based P[i][j] is used where P[i][j] is the (i,j)-th 
     point object. A typical calling example is:

       NL_POINT   **P;
       NL_REAL    **x, **y, **z;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get memory for x, y and z, and define coordinate values);
       ...
       P = N_XYZTo2dPtArray(x,y,z,n,m,&S); 


   ACCESS:
   
     x,y,z , input  ,  Point coordinates
     n,m   , input  ,  Highest indexes in <x,y,z>
     S     , input  ,  Memory stack pointer


   RETURN CODES:

     P    : Pointer to point array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_POINT ** N_XYZTo2dPtArray( NL_REAL ** x, NL_REAL ** y, NL_REAL ** z, NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_INDEX i, j;

    NL_POINT ** P;

    /* Allocate memory for the point array */

    P = N_AllocPt2dArray( n, m, S );

    if( P EQ NULL )
        return NULL;

    /* Fill in the array */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_PtFromXYZ( x[i][j], y[i][j], z[i][j], &P[i][j] );
        }
    }

    /* Exit */

    return P;
} /* end N_XYZTo2dPtArray */

/*******************************************************************//**


   DESCRIPTION:

     Given an  array of control points either in 2-D or in 3-D defined 
     by the wx, wy, wz and w coordinate arrays. This  routine converts 
     this  input  to  control  point  input.  Two  dimensional and non 
     rational control points are dealt with by setting the appropriate
     coordinates  to NL_NOZ  or  NL_NOW, respectively. A  typical  calling
     example is:

       NL_CPOINT  *Pw;
       NL_REAL    *wx, *wy, *wz, *w;
       NL_STACKS  S;
       ...
       (get memory for wx, wy, wz and w, and set their values);
       ...
       Pw = N_XYZToCPtArray(wx,wy,wz,w,n,&S);


   ACCESS:
   
     wx,wy,wz,w , input  ,  Control point coordinates
     n          , input  ,  Highest index in <wx,wy,wz,w>
     S          , input  ,  Memory stacks pointer


   RETURN CODES:

     Pw   : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/
NL_CPOINT *N_XYZToCPtArray( NL_REAL *wx, NL_REAL *wy, NL_REAL *wz, NL_REAL *w, NL_INDEX n, NL_STACKS *S )
{
  NL_INDEX i;
  
  NL_CPOINT *Pw;
  
  /* Allocate memory for the array */
  
  Pw = N_AllocCPt1dArray( n, S );
  
  if( Pw EQ NULL )
      return NULL;
  
  /* Fill in the array */
  
  for ( i = 0; i <= n; i++ )
  {
      N_CPtFromWxWyWz( wx[i], wy[i], wz[i], w[i], &Pw[i] );
  }
  
  /* Exit */
  
  return Pw;
} /* end N_XYZToCPtArray */

/*******************************************************************//**


   DESCRIPTION:

     Given a 2-D array of control points defined by the wx, wy, wz and 
     w coordinate arrays. This routine  converts this input to control  
     point  input. It can be restricted to non rational by setting the 
     w values to NL_NOW. A typical calling example is:

       NL_CPOINT  **Pw;
       NL_REAL    **wx, **wy, **wz, **w;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get memory for wx, wy, wz and w, and set their values);
       ...
       Pw = N_XYZTo2dCPtArray(wx,wy,wz,w,n,m,&S);


   ACCESS:
   
     wx,wy,wz,w , input  ,  Control point coordinates
     n,m        , input  ,  Highest indexes in <wx,wy,wz,w>
     S          , input  ,  Memory stacks pointer


   RETURN CODES:

     Pw   : Pointer to control point array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CPOINT ** N_XYZTo2dCPtArray( NL_REAL ** wx, NL_REAL ** wy, NL_REAL ** wz, NL_REAL ** w, NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_INDEX i, j;

    NL_CPOINT ** Pw;

    /* Allocate memory for the 2-D array */

    Pw = N_AllocCPt2dArray( n, m, S );

    if( Pw EQ NULL )
        return NULL;

    /* Fill in the control point array */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtFromWxWyWz( wx[i][j], wy[i][j], wz[i][j], w[i][j], &Pw[i][j] );
        }
    }

    /* Exit */

    return Pw;
} /* end N_XYZTo2dCPtArray */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store an integer matrix. Proper 
     error checks  are performed  in case memory  allocations fail. A
     typical calling example is:

       NL_IMATRIX     *ima;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STACKS      S;
       ...
       (get n, m, mtp and bw);
       ...
       ima = N_AllocIntMatrix(n,m,mtp,bw,&S);


   ACCESS:
   
     n,m , input  ,  Highest row and column indexes
     mtp , input  ,  Matrix type: 
                       - NL_MT_FULL 
                       - NL_MT_LOWERLEFT 
                       - NL_MT_UPPERRIGHT 
                       - NL_MT_BANDED
     bw  , input  ,  Bandwidth
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     ima  : Pointer to matrix if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_IMATRIX *N_AllocIntMatrix( NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocIntMatrix");

    NL_INTEGER ** IM;

    NL_IMATRIX *ima;

    NL_IMANODE *imd;

    /* Allocate memory to store matrix elements */

    if( mtp EQ NL_MT_BANDED )
    {
        IM = N_AllocInt2dArray( n, bw - 1, S );
    }
    else
    {
        IM = N_AllocInt2dArray( n, m, S );
    }

    if( IM EQ NULL )
        return NULL;

    /* Allocate memory for matrix structure */

    ima = (NL_IMATRIX *)N_Malloc( sizeof( NL_IMATRIX ) );

    if( ima EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    imd = (NL_IMANODE *)N_Malloc( sizeof( NL_IMANODE ) );

    if( imd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( ima );
        ima = NULL;
        return NULL;
    }

    imd->ptr = ima;
    imd->next = S->ima;
    S->ima = imd;

    /* Build matrix structure */

    N_CreateIntMatrix( ima, n, m, IM, mtp, bw );

    /* Exit */

    return ima;
} /* end N_AllocIntMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to  store a real matrix. Proper 
     error checks are performed in case memory allocations fail. A
     typical calling example is:

       NL_RMATRIX     *rma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STACKS      S;
       ...
       (get n, m, mtp and bw);
       ...
       rma = N_AllocRealMatrix(n,m,mtp,bw,&S);


   ACCESS:
   
     n,m , input  ,  Highest row and column indexes
     mtp , input  ,  Matrix type: 
                       - NL_MT_FULL 
                       - NL_MT_LOWERLEFT 
                       - NL_MT_UPPERRIGHT 
                       - NL_MT_BANDED
     bw  , input  ,  Bandwidth
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     rma  : Pointer to matrix if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_RMATRIX *N_AllocRealMatrix( NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocRealMatrix");

    NL_REAL ** RM;

    NL_RMATRIX *rma;

    NL_RMANODE *rmd;

    /* Allocate memory to store matrix elements */

    if( mtp EQ NL_MT_BANDED )
    {
        RM = N_AllocReal2dArray( n, bw - 1, S );
    }
    else
    {
        RM = N_AllocReal2dArray( n, m, S );
    }

    if( RM EQ NULL )
        return NULL;

    /* Allocate memory for matrix structure */

    rma = (NL_RMATRIX *)N_Malloc( sizeof( NL_RMATRIX ) );

    if( rma EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    rmd = (NL_RMANODE *)N_Malloc( sizeof( NL_RMANODE ) );

    if( rmd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( rma );
        rma = NULL;
        return NULL;
    }

    rmd->ptr = rma;
    rmd->next = S->rma;
    S->rma = rmd;

    /* Build matrix structure */

    N_CreateRealMatrix( rma, n, m, RM, mtp, bw );

    /* Exit */

    return rma;
} /* end N_AllocRealMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store a point matrix. Proper 
     error checks are performed in case memory allocations fail. A
     typical calling example is:

       NL_PMATRIX     *pma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STACKS      S;
       ...
       (get n, m, mtp and bw);
       ...
       pma = N_AllocPtMatrix(n,m,mtp,bw,&S);


   ACCESS:
   
     n,m , input  ,  Highest row and column indexes
     mtp , input  ,  Matrix type: 
                       - NL_MT_FULL 
                       - NL_MT_LOWERLEFT 
                       - NL_MT_UPPERRIGHT 
                       - NL_MT_BANDED
     bw  , input  ,  Bandwidth
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     pma  : Pointer to matrix if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_PMATRIX *N_AllocPtMatrix( NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocPtMatrix");

    NL_POINT ** PM;

    NL_PMATRIX *pma;

    NL_PMANODE *pmd;

    /* Allocate memory to store matrix elements */

    if( mtp EQ NL_MT_BANDED )
    {
        PM = N_AllocPt2dArray( n, bw - 1, S );
    }
    else
    {
        PM = N_AllocPt2dArray( n, m, S );
    }

    if( PM EQ NULL )
        return NULL;

    /* Allocate memory for matrix structure */

    pma = (NL_PMATRIX *)N_Malloc( sizeof( NL_PMATRIX ) );

    if( pma EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    pmd = (NL_PMANODE *)N_Malloc( sizeof( NL_PMANODE ) );

    if( pmd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( pma );
        pma = NULL;
        return NULL;
    }

    pmd->ptr = pma;
    pmd->next = S->pma;
    S->pma = pmd;

    /* Build matrix structure */

    N_CreatePtMatrix( pma, n, m, PM, mtp, bw );

    /* Exit */

    return pma;
} /* end N_AllocPtMatrix */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to  store a control  point matrix. 
     Proper error checks are performed in case memory allocations fail.
     A typical calling example is:

       NL_CMATRIX     *cma;
       NL_INDEX       n, m, bw;
       NL_MATRIXTYPE  mtp;
       NL_STACKS      S;
       ...
       (get n, m, mtp and bw);
       ...
       cma = N_AllocCPtMatrix(n,m,mtp,bw,&S);


   ACCESS:
   
     n,m , input  ,  Highest row and column indexes
     mtp , input  ,  Matrix type: 
                       - NL_MT_FULL 
                       - NL_MT_LOWERLEFT 
                       - NL_MT_UPPERRIGHT 
                       - NL_MT_BANDED
     bw  , input  ,  Bandwidth
     S   , input  ,  Memory stacks pointer


   RETURN CODES:

     cma  : Pointer to matrix if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CMATRIX *N_AllocCPtMatrix( NL_INDEX n, NL_INDEX m, NL_MATRIXTYPE mtp, NL_INDEX bw, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCPtMatrix");

    NL_CPOINT ** CM;

    NL_CMATRIX *cma;

    NL_CMANODE *cmd;

    /* Allocate memory to store matrix elements */

    if( mtp EQ NL_MT_BANDED )
    {
        CM = N_AllocCPt2dArray( n, bw - 1, S );
    }
    else
    {
        CM = N_AllocCPt2dArray( n, m, S );
    }

    if( CM EQ NULL )
        return NULL;

    /* Allocate memory for matrix structure */

    cma = (NL_CMATRIX *)N_Malloc( sizeof( NL_CMATRIX ) );

    if( cma EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    cmd = (NL_CMANODE *)N_Malloc( sizeof( NL_CMANODE ) );

    if( cmd EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( cma );
        cma = NULL;
        return NULL;
    }

    cmd->ptr = cma;
    cmd->next = S->cma;
    S->cma = cmd;

    /* Build matrix structure */

    N_CreateCPtMatrix( cma, n, m, CM, mtp, bw );

    /* Exit */

    return cma;
} /* end N_AllocCPtMatrix */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine allocates  memory to store a 1-D array of control 
     point pointers. Proper error check is performed in case memory 
     allocations fail. A typical calling example is:
 
       NL_CPOINT   **Pw;
       NL_INDEX    n;
       NL_STACKS   S;
       ...
       (get n);
       ...
       Pw = N_AllocCPtPtr1dArray(n,&S);
 
 
   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stacks pointer
 
 
   RETURN CODES:
 
     Pw   : Pointer to array if no error
     NULL : Memory allocation fails
 
   ***********************************************************************/

NL_CPOINT ** N_AllocCPtPtr1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCPtPtr1dArray");

    NL_CPOINT ** Pw;

    NL_C2DNODE *c2d;

    /* Allocate memory */

    Pw = (NL_CPOINT ** )N_Malloc( (n + 1) * sizeof( NL_CPOINT * ) );

    if( Pw EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stacks */

    c2d = (NL_C2DNODE *)N_Malloc( sizeof( NL_C2DNODE ) );

    if( c2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        return NULL;
    }

    c2d->ptr = Pw;
    c2d->next = S->c2d;
    S->c2d = c2d;

    /* Exit */

    return Pw;
}

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine allocates memory to  store a 2-D array of control 
     point pointers. Proper error check is performed in case memory 
     allocations fail. A typical calling example is:
 
       NL_CPOINT   ***Pw;
       NL_INDEX    n, m;
       NL_STACKS   S;
       ...
       (get n and m);
       ...
       Pw = N_AllocCPtPtr2dArray(n,m,&S);
 
 
   ACCESS:
   
     n,m , input  ,  Highest indexes in array
     S   , input  ,  Memory stacks pointer
 
 
   RETURN CODES:
 
     Pw   : Pointer to array if no error
     NULL : Memory allocation fails
 
   ***********************************************************************/

NL_CPOINT *** N_AllocCPtPtr2dArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCPtPtr2dArray");

    NL_INDEX k, l;

    NL_CPOINT *** Pw, ** Qw;

    NL_C2DNODE *c2d;

    NL_C3DNODE *c3d;

    /* Allocate memory */

    Pw = (NL_CPOINT *** )N_Malloc( (n + 1) * sizeof( NL_CPOINT ** ) );

    if( Pw EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    Qw = (NL_CPOINT ** )N_Malloc( (n + 1) * (m + 1) * sizeof( NL_CPOINT * ) );

    if( Qw EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        return NULL;
    }

    /* Make pointer asignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        Pw[k] = &Qw[l];
        l = l + m + 1;
    }

    /* Put pointers on memory stacks */

    c2d = (NL_C2DNODE *)N_Malloc( sizeof( NL_C2DNODE ) );

    if( c2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        N_Free( Qw );
        Qw = NULL;
        return NULL;
    }

    c3d = (NL_C3DNODE *)N_Malloc( sizeof( NL_C3DNODE ) );

    if( c3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( Pw );
        Pw = NULL;
        N_Free( Qw );
        Qw = NULL;
        N_Free( c2d );
        c2d = NULL;
        return NULL;
    }

    c2d->ptr = Qw;
    c2d->next = S->c2d;
    S->c2d = c2d;

    c3d->ptr = Pw;
    c3d->next = S->c3d;
    S->c3d = c3d;

    /* Exit */

    return Pw;
}

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine allocates  memory to  store a 1-D array of  point 
     pointers.  Proper error  check is  performed  in  case  memory 
     allocations fail. A typical calling example is:
 
       NL_POINT    **P;
       NL_INDEX    n;
       NL_STACKS   S;
       ...
       (get n);
       ...
       P = N_AllocPtPtr1dArray(n,&S);
 
 
   ACCESS:
   
     n  , input  ,  Highest index in array
     S  , input  ,  Memory stacks pointer
 
 
   RETURN CODES:
 
     P    : Pointer to array if no error
     NULL : Memory allocation fails
 
   ***********************************************************************/

/* NL_POINT  **N_AllocPtPtr1dArray */
NL_POINT ** N_AllocPtPtr1dArray( NL_INDEX n, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocPtPtr1dArray");

    NL_POINT ** P;

    NL_P2DNODE *p2d;

    /* Allocate memory */

    P = (NL_POINT ** )N_Malloc( (n + 1) * sizeof( NL_POINT * ) );

    if( P EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stacks */

    p2d = (NL_P2DNODE *)N_Malloc( sizeof( NL_P2DNODE ) );

    if( p2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        return NULL;
    }

    p2d->ptr = P;
    p2d->next = S->p2d;
    S->p2d = p2d;

    /* Exit */

    return P;
} /* end N_AllocPtPtr1dArray */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine allocates  memory to  store a 2-D array of  point 
     pointers.  Proper error  check is  performed  in  case  memory 
     allocations fail. A typical calling example is:
 
       NL_POINT    ***P;
       NL_INDEX    n, m;
       NL_STACKS   S;
       ...
       (get n and m);
       ...
       P = N_AllocPtPtr2dArray(n,m,&S);
 
 
   ACCESS:
   
     n,m , input  ,  Highest indexes in array
     S   , input  ,  Memory stacks pointer
 
 
   RETURN CODES:
 
     P    : Pointer to array if no error
     NULL : Memory allocation fails
 
   ***********************************************************************/

NL_POINT *** N_AllocPtPtr2dArray( NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocPtPtr2dArray");

    NL_INDEX k, l;

    NL_POINT *** P, ** Q;

    NL_P2DNODE *p2d;

    NL_P3DNODE *p3d;

    /* Allocate memory */

    P = (NL_POINT *** )N_Malloc( (n + 1) * sizeof( NL_POINT ** ) );

    if( P EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    Q = (NL_POINT ** )N_Malloc( (n + 1) * (m + 1) * sizeof( NL_POINT * ) );

    if( Q EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        return NULL;
    }

    /* Make pointer asignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        P[k] = &Q[l];
        l = l + m + 1;
    }

    /* Put pointers on memory stacks */

    p2d = (NL_P2DNODE *)N_Malloc( sizeof( NL_P2DNODE ) );

    if( p2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        N_Free( Q );
        Q = NULL;
        return NULL;
    }

    p3d = (NL_P3DNODE *)N_Malloc( sizeof( NL_P3DNODE ) );

    if( p3d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        N_Free( Q );
        Q = NULL;
        N_Free( p2d );
        p2d = NULL;
        return NULL;
    }

    p2d->ptr = Q;
    p2d->next = S->p2d;
    S->p2d = p2d;

    p3d->ptr = P;
    p3d->next = S->p3d;
    S->p3d = p3d;

    /* Exit */

    return P;
} /* end N_AllocPtPtr2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  allocates memory  to store  a 2-D  point  array of 
     variable column size. That is, the array looks like this:

       P[0][0],...,P[0][m[0]]
       P[1][0],...,P[1][m[1]]
       ...
       P[n][0],...,P[n][m[n]]

     Proper  error check is performed in case memory allocations fail. 
     A typical calling example is:

       NL_POINT   **P;
       NL_INDEX   n, *m;
       NL_STACKS  S;
       ...
       (get n and set up array m);
       ...
       P = N_AllocVariable2dPtArray(n,m,&S);


   ACCESS:
   
     n  , input  ,  Highest row index
     m  , input  ,  Highest column indexes
     S  , input  ,  Memory stacks pointer


   RETURN CODES:

     P    : Pointer to array if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_POINT ** N_AllocVariable2dPtArray( NL_INDEX n, NL_INDEX *m, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocVariable2dPtArray");

    NL_INDEX k, l, r;

    NL_POINT ** P, *Q;

    NL_P1DNODE *p1d;

    NL_P2DNODE *p2d;

    /* Get total number of entries */

    r = 0;

    for ( k = 0; k <= n; k++ )
        r = r + m[k] + 1;

    /* Allocate memories for the array */

    P = (NL_POINT ** )N_Malloc( (n + 1) * sizeof( NL_POINT * ) );

    if( P EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    Q = (NL_POINT *)N_Malloc( r * sizeof( NL_POINT ) );

    if( Q EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        return NULL;
    }

    /* Make pointer assignments */

    l = 0;

    for ( k = 0; k <= n; k++ )
    {
        P[k] = &Q[l];
        l = l + m[k] + 1;
    }

    /* Put pointers on memory stacks */

    p1d = (NL_P1DNODE *)N_Malloc( sizeof( NL_P1DNODE ) );

    if( p1d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        N_Free( Q );
        Q = NULL;
        return NULL;
    }

    p2d = (NL_P2DNODE *)N_Malloc( sizeof( NL_P2DNODE ) );

    if( p2d EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( P );
        P = NULL;
        N_Free( Q );
        Q = NULL;
        N_Free( p1d );
        p1d = NULL;
        return NULL;
    }

    p1d->ptr = Q;
    p1d->next = S->p1d;
    S->p1d = p1d;

    p2d->ptr = P;
    p2d->next = S->p2d;
    S->p2d = p2d;

    /* Exit */

    return P;
} /* end N_AllocVariable2dPtArray */

/*******************************************************************//**


   DESCRIPTION:

     Opens and returns a file stream that can be used by other NLib 
     functions with N_FPRINTF() and fscanf() calls.

     One feature of fopen(), N_FPRINTF(), fscanf(), fclose() is that
     N_FPRINTF() and fscanf() will only work on FILE streams opened
     in the same DLL or dynamic library from which they are being called.

     This function is a simple wrapper for fopen() so that applications
     can open FILE streams from NLib which can be passed to other
     NLIb I/O functions.

   ACCESS:
   
     fname , input  ,  filename string to be opened
     mode  , input  ,  file mode string specifying how to open the file
                       "r" = open existing file for read
                       "w" = open new file for write
     fptr , input  ,  FILE stream pointer opened with N_FileOpen().

     FILE *fptr = N_FileOpen(fname, mode);
     . . .
     N_FileClose(fptr)
 

   RETURNS:

     non-NULL : no error
     NULL     : an error

   ***********************************************************************/
FILE *N_FileOpen( const TCHAR* fname, const TCHAR* mode )
{
    NL_PRIVATE NL_STRING rname = _T("N_FileOpen");
    FILE *fptr;

/* Open file */
#ifdef VS8

    fopen_s( &fptr, fname, mode );

#else

    fptr = N_FOPEN( fname, mode );

#endif

    if( fptr EQ NULL )
        N_ErrSet(NL_FIL_ERR,rname);

    /* all done */
    return (fptr);

} /* end N_FileOpen */

/*******************************************************************//**


   DESCRIPTION:

     Writes a string to the specified FILE pointer.

     One feature of fopen(), N_FPRINTF(), fscanf(), fclose() is that
     N_FPRINTF() and fscanf() will only work on FILE streams opened
     in the same DLL or dynamic library from which they are being called.

     This function is a simple wrapper for N_FPRINTF() so that applications
     can open FILE streams from NLib which can be passed to other
     NLIb I/O functions.

   ACCESS:
   
     fptr  , input  ,  FILE stream pointer opened with N_FileOpen()
     string, input  ,  string to write to file

     TCHAR sBuf[256]

     FILE *fPtr = N_FileOpen(_T("Filename"), "w") ;
     . . .
     SM_SPRINTF(sBuf,_T("Fomatted output string with args %d"), lArg) ;
     N_FileWrite(fptr, sBuf);
     . . .
     N_FileClose(fptr);
 

   RETURNS:

     non-NULL : no error
     NULL     : an error

   ***********************************************************************/
NL_VOID N_FileWrite( FILE *fptr, NL_STRING string )
{
    /* Write to file */
    N_FPRINTF( fptr, _T("%s"), string );
} /* end N_FileWrite */

/*******************************************************************//**


   DESCRIPTION:

     Closes  a file stream that has been opened with a N_FileOpen() call. 
     Functions with N_FPRINTF() and fscanf() calls.

     One feature of fopen(), N_FPRINTF(), fscanf(), fclose() is that
     N_FPRINTF() and fscanf() will only work on FILE streams opened
     in the same DLL or dynamic library from which they are being called.

     This function is a simple wrapper for fclose() so that applications
     can close FILE streams from NLib which were opened with N_FileOpen().

   ACCESS:
   
     fname , input  ,  filename string to be opened
     mode  , input  ,  file mode string specifying how to open the file
                       "r" = open existing file for read
                       "w" = open new file for write
     fptr , input  ,  FILE stream pointer opened with N_FileOpen().

     FILE *fptr = N_FileOpen(fname, mode);
     . . .
     N_FileClose(fptr)
 

   RETURNS:

     NL_VOID

   ***********************************************************************/
NL_VOID N_FileClose( FILE *fptr )
{
    /* close file */
    fclose( fptr );
} /* end N_FileClose */

/*******************************************************************//**


   DESCRIPTION:

     moves the file pointer forward by lBytes in
     a FILE ptr opened by N_FileOpen().

   ACCESS:
   
     fname , input  ,  filename string to be opened
     mode  , input  ,  file mode string specifying how to open the file
                       "r" = open existing file for read
                       "w" = open new file for write
     fptr  , input  ,  FILE stream pointer opened with N_FileOpen().
     bytes , input  , number of bytes to move the file pointer forward.


     FILE *fptr = N_FileOpen(fname, mode);

     N_FileOffset(fptr, bytes);

     N_FileClose(fptr);
 

   RETURNS:

     NL_VOID

   ***********************************************************************/

NL_VOID N_FileOffset( FILE *fptr, NL_INTEGER bytes )
{
    /* Do the File offset */
    NL_INTEGER i;

    for ( i = 0; i < bytes; i++ )
    {
        fgetc( fptr );
    }
} /* end N_FileOffset */

/* Read and return index from open file */
NL_INDEX N_FileReadIndex( FILE *fptr )
{
    NL_INDEX n = 0;
	NL_INDEX nRtn = N_FSCANF(fptr, _T("%ld"), &n);
    return (nRtn != EOF ? n : nRtn);
}

/* Write index to open file */
NL_VOID N_FileWriteIndex( FILE *fptr, NL_INDEX n )
{
    /* Write to file */
    N_FPRINTF( fptr, _T("%ld\n"), n );
} /* end N_FileWriteIndex */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  reads in 1-D  NL_POINT  data  from a file and  
     creates a 1-D point array. The data file is assumed to be arranged 
     as follows:

           n         --> highest index in point array
           x0 y0 z0  -->
           x1 y1 z1  -->
           .         --> <x,y,z> components
           .         --> 
           .         -->
           xn yn zn  -->

     THIS  ROUTINE  ALLOCATES MEMORY  LOCALLY, ONLY A POINTER IS PASSED 
     IN. A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   n;
       TCHAR* fname;
       NL_STACKS  S;
       ...
       (get file name fname);
       ...
       N_Read1dPtFromFile(&P,&n,fname,&S);


   ACCESS:
   
     P     , output ,  Pointer to 1-D array
     n     , output ,  Highest index in P
     fname , input  ,  Name of the data file
     S     , input  ,  P's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Read1dPtFromFile( NL_POINT ** P, NL_INDEX *n, TCHAR* fname, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Read1dPtFromFile");

    NL_FLAG error = NL_NO;

    NL_INDEX i, k;

    NL_REAL x, y, z;

    NL_POINT *Q;

    FILE *fptr;

    /* Open file */

    fptr = N_FileOpen( fname, _T("r") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Allocate memory */

	if (EOF == N_FSCANF(fptr, _T("%ld"), &k)) { NL_ERROR(NL_SCN_ERR); }

    Q = NULL;

    if( k >= 0 )
    {
        Q = N_AllocPt1dArray( k, S );

        if( Q EQ NULL )
            NL_QUIT;
    }

    /* Read in data */

    for(i = 0; i <= k; i++)
    {
      if(EOF == N_FSCANF( fptr, _T( "%lf%lf%lf" ), &x, &y, &z ))
      {
        NL_ERROR( NL_SCN_ERR );
      }
      N_PtFromXYZ( x, y, z, &Q[i] );
    }

    *P = Q;
    *n = k;

    /* Exit */

    EXIT:
    if( fptr NEQ NULL )
        N_FileClose( fptr );

    return (error);
} /* end N_Read1dPtFromFile */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine writes 1-D NL_POINT data to a file. The data 
     file is assumed to be arranged as follows:

           n         --> highest index in point array
           x0 y0 z0  -->
           x1 y1 z1  -->
           .         --> <x,y,z> components
           .         --> 
           .         -->
           xn yn zn  -->

     A typical calling example is:

       NL_POINT   *P;
       NL_INDEX   n;
       TCHAR*  fname;
       ...
       (get array P);
       ...
       N_Write1dPtToFile(P,n,fname);


   ACCESS:
   
     P     , input ,  1-D point array
     n     , input ,  Highest index in P
     fname , input ,  Name of the data file
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Write1dPtToFile( NL_POINT *P, NL_INDEX n, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_Write1dPtToFile");

    NL_FLAG error = NL_NO;

    NL_INDEX i;

    NL_REAL x, y, z;

    FILE *fptr;

    /* Open file */

    fptr = N_FileOpen( fname, _T("w") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Write data out */

    N_FPRINTF( fptr, _T("%ld\n"), n );

    for ( i = 0; i <= n; i++ )
    {
        N_PtToXYZ( P[i], &x, &y, &z );
        N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f\n"), x, y, z );
    }

    /* Exit */

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_Write1dPtToFile */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  reads in 2-D  NL_POINT  data  from a file and  
     creates a 2-D point array. The data file is assumed to be arranged 
     as follows:

           n m          --> highest indexes in point array
           x00 y00 z00  -->
           x01 y01 z01  -->
           ...          -->
           x0m y0m z0m  --> <x,y,z> components
           x10 y10 z10  -->
           ...          --> 
           xnm ynm znm  -->

     THIS  ROUTINE  ALLOCATES MEMORY  LOCALLY, ONLY A POINTER IS PASSED 
     IN. A typical calling example is:

       NL_POINT   **P;
       NL_INDEX   n, m;
       NL_STRING  fname;
       NL_STACKS  S;
       ...
       (get file name fname);
       ...
       N_Read2dPtFromFile(&P,&n,&m,fname,&S);


   ACCESS:
   
     P     , output ,  Pointer to 2-D array
     n,m   , output ,  Highest indexes in P
     fname , input  ,  Name of the data file
     S     , input  ,  P's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Read2dPtFromFile( NL_POINT *** P, NL_INDEX *n, NL_INDEX *m, TCHAR* fname, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_Read2dPtFromFile");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l;

    NL_REAL x, y, z;

    NL_POINT ** Q;

    FILE *fptr;

    /* Open file */

    fptr = N_FileOpen( fname, _T("r") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Allocate memory */

	if (EOF == N_FSCANF(fptr, _T("%ld%ld"), &k, &l)) { NL_ERROR(NL_SCN_ERR); }

    if( k < 0 || l < 0 )
        NL_OUT;

    Q = N_AllocPt2dArray( k, l, S );

    if( Q EQ NULL )
        NL_QUIT;

    /* Read in data */

    for ( i = 0; i <= k; i++ )
    {
        for ( j = 0; j <= l; j++ )
        {
			if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf"), &x, &y, &z)) { NL_ERROR(NL_SCN_ERR); }
            N_PtFromXYZ( x, y, z, &Q[i][j] );
        }
    }

    *P = Q;
    *n = k;
    *m = l;

    /* Exit */

    EXIT:
    if( fptr NEQ NULL )
        N_FileClose( fptr );

    return (error);
} /* end N_Read2dPtFromFile */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine writes 2-D NL_POINT data to a file. The data 
     file is assumed to be arranged as follows:

           n m          --> highest indexes in point array
           x00 y00 z00  -->
           x01 y01 z01  -->
           ...          -->
           x0m y0m z0m  --> <x,y,z> components
           x10 y10 z10  -->
           ...          --> 
           xnm ynm znm  -->

     A typical calling example is:

       NL_POINT   **P;
       NL_INDEX   n, m;
       NL_STRING  fname;
       ...
       (get array P);
       ...
       N_Write2dPtToFile(P,n,m,fname);


   ACCESS:
   
     P     , input ,  2-D point array
     n,m   , input ,  Highest indexes in P
     fname , input ,  Name of the data file
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Write2dPtToFile( NL_POINT ** P, NL_INDEX n, NL_INDEX m, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_Write2dPtToFile");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j;

    NL_REAL x, y, z;

    FILE *fptr;

    /* Open file */

    fptr = N_FileOpen( fname, _T("w") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Write data out */

    N_FPRINTF( fptr, _T("%ld %ld\n"), n, m );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_PtToXYZ( P[i][j], &x, &y, &z );
            N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f\n"), x, y, z );
        }
    }

    /* Exit */

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_Write2dPtToFile */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine  deallocates  memory  that stores a  2-D flag 
     array. Given a  pointer to  the array, the routine searches for the 
     pointer on the memory stack. If it is found, memory is deallocated. 
     If not, the routine does nothing. A typical calling example is:

       NL_FLAG    **f2d;
       NL_STACKS  S;
       ...
       N_FreeFlag2dArray(f2d,&S);


   ACCESS:
   
     f2d , input  ,  2-D flag array pointer
     S   , input  ,  f2d's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeFlag2dArray( NL_FLAG ** f2d, NL_STACKS *S )
{
    NL_F2DNODE *prev, *curr;

    NL_FLAG *f1d;

    /* Traverse memory stack to find pointer */

    if( S->f2d NEQ NULL )
    {
        prev = S->f2d;
        curr = S->f2d;

        while( curr NEQ NULL AND curr->ptr NEQ f2d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->f2d = NULL;
            }
            else /* More than one node */
            {
                S->f2d = S->f2d->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            f1d = f2d[0];
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
            N_FreeFlag1dArray( f1d, S );
        }
    }
} /* end N_FreeFlag2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine  deallocates memory that stores a flag array. 
     Given a pointer to the array, the routine searches for the pointer 
     on the  memory  stack. If it  is found, memory  is deallocated. If 
     not, the routine does nothing. A typical calling example is:

       NL_FLAG    *f1d;
       NL_STACKS  S;
       ...
       N_FreeFlag1dArray(f1d,&S);


   ACCESS:
   
     f1d , input  ,  Flag array pointer
     S   , input  ,  f1d's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeFlag1dArray( NL_FLAG *f1d, NL_STACKS *S )
{
    NL_F1DNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->f1d NEQ NULL )
    {
        prev = S->f1d;
        curr = S->f1d;

        while( curr NEQ NULL AND curr->ptr NEQ f1d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->f1d = NULL;
            }
            else /* More than one node */
            {
                S->f1d = S->f1d->next;
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
} /* end N_FreeFlag1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine  deallocates memory that  stores an integer
     array. Given a  pointer to the  array, the  routine searches for 
     the  pointer on  the  memory  stack. If it  is found, memory  is 
     deallocated. If not, the routine does nothing. A typical calling
     example is:

       NL_INTEGER  *i1d;
       NL_STACKS   S;
       ...
       N_FreeInt1dArray(i1d,&S);


   ACCESS:
   
     i1d , input  ,  Integer array pointer
     S   , input  ,  i1d's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeInt1dArray( NL_INTEGER *i1d, NL_STACKS *S )
{
    NL_I1DNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->i1d NEQ NULL )
    {
        prev = S->i1d;
        curr = S->i1d;

        while( curr NEQ NULL AND curr->ptr NEQ i1d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->i1d = NULL;
            }
            else /* More than one node */
            {
                S->i1d = S->i1d->next;
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
} /* end N_FreeInt1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine deallocates  memory that stores a 2-D integer
     array. Given a  pointer to  the array, the routine searches for the 
     pointer on the memory stack. If it is found, memory is deallocated. 
     If not, the routine does nothing. A typical calling example is:

       NL_INTEGER  **i2d;
       NL_STACKS   S;
       ...
       N_FreeInt2dArray(i2d,&S);


   ACCESS:
   
     i2d , input  ,  2-D integer array pointer
     S   , input  ,  i2d's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeInt2dArray( NL_INTEGER ** i2d, NL_STACKS *S )
{
    NL_I2DNODE *prev, *curr;

    NL_INTEGER *i1d;

    /* Traverse memory stack to find pointer */

    if( S->i2d NEQ NULL )
    {
        prev = S->i2d;
        curr = S->i2d;

        while( curr NEQ NULL AND curr->ptr NEQ i2d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->i2d = NULL;
            }
            else /* More than one node */
            {
                S->i2d = S->i2d->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            i1d = i2d[0];
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
            N_FreeInt1dArray( i1d, S );
        }
    }
} /* end N_FreeInt2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine  deallocates  memory that stores an  array of
     integer  pointers. Given  a  pointer to  the  array,  the  routine 
     searches  for the pointer  on the  memory  stack. If it  is found, 
     memory is deallocated. If not, the routine does nothing. A typical 
     calling example is:

       NL_INTEGER  **i1p;
       NL_STACKS   S;
       ...
       N_FreeIntPtr1dArray(i1p,&S);


   ACCESS:
   
     i1p , input  ,  Array of integer pointers
     S   , input  ,  i1p's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeIntPtr1dArray( NL_INTEGER ** i1p, NL_STACKS *S )
{
    NL_I2DNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->i2d NEQ NULL )
    {
        prev = S->i2d;
        curr = S->i2d;

        while( curr NEQ NULL AND curr->ptr NEQ i1p )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->i2d = NULL;
            }
            else /* More than one node */
            {
                S->i2d = S->i2d->next;
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
} /* end N_FreeIntPtr1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine deallocates memory that stores a 2-D array of
     integer  pointers. Given  a  pointer  to  the  array, the  routine 
     searches  for the  pointer on  the memory  stack. If  it is found, 
     memory is deallocated. If not, the routine does nothing. A typical 
     calling example is:

       NL_INTEGER  ***i2p;
       NL_STACKS   S;
       ...
       N_FreeIntPtr2dArray(i2p,&S);


   ACCESS:
   
     i2p , input  ,  2-D array of integer pointers
     S   , input  ,  i2p's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeIntPtr2dArray( NL_INTEGER *** i2p, NL_STACKS *S )
{
    NL_I3DNODE *prev, *curr;

    NL_INTEGER ** i1p;

    /* Traverse memory stack to find pointer */

    if( S->i3d NEQ NULL )
    {
        prev = S->i3d;
        curr = S->i3d;

        while( curr NEQ NULL AND curr->ptr NEQ i2p )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->i3d = NULL;
            }
            else /* More than one node */
            {
                S->i3d = S->i3d->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            i1p = i2p[0];
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
            N_FreeIntPtr1dArray( i1p, S );
        }
    }
} /* end N_FreeIntPtr2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  deallocates  memory  that  stores a real  
     array. Given a  pointer to the  array, the  routine searches for 
     the  pointer on  the  memory  stack. It it  is found, memory  is 
     deallocated. If not, the routine does nothing. A typical calling
     example is:

       NL_REAL    *r1d;
       NL_STACKS  S;
       ...
       N_FreeReal1dArray(r1d,&S);


   ACCESS:
   
     r1d , input  ,  Real array pointer
     S   , input  ,  r1d's stack


   RETURN CODES:

     None

   ***********************************************************************/
/* NL_VOID N_FreeReal1dArray */
NL_VOID N_FreeReal1dArray( NL_REAL *r1d, NL_STACKS *S )
{
    NL_R1DNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->r1d NEQ NULL )
    {
        prev = S->r1d;
        curr = S->r1d;

        while( curr NEQ NULL AND curr->ptr NEQ r1d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->r1d = NULL;
            }
            else /* More than one node */
            {
                S->r1d = S->r1d->next;
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
} /* end N_FreeReal1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  deallocates  memory that  stores a 2-D real  
     array. Given a  pointer to  the array, the routine searches for the 
     pointer on the memory stack. It it is found, memory is deallocated. 
     If not, the routine does nothing. A typical calling example is:

       NL_REAL    **r2d;
       NL_STACKS  S;
       ...
       N_FreeReal2dArray(r2d,&S);


   ACCESS:
   
     r2d , input  ,  2-D real array pointer
     S   , input  ,  r2d's stack


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_FreeReal2dArray */
NL_VOID N_FreeReal2dArray( NL_REAL ** r2d, NL_STACKS *S )
{
    NL_R2DNODE *prev, *curr;

    NL_REAL *r1d;

    /* Traverse memory stack to find pointer */

    if( S->r2d NEQ NULL )
    {
        prev = S->r2d;
        curr = S->r2d;

        while( curr NEQ NULL AND curr->ptr NEQ r2d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->r2d = NULL;
            }
            else /* More than one node */
            {
                S->r2d = S->r2d->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            r1d = r2d[0];
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
            N_FreeReal1dArray( r1d, S );
        }
    }
} /* end N_FreeReal2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  deallocates  memory that  stores a 3-D real  
     array. Given a  pointer to  the array, the routine searches for the 
     pointer on the memory stack. It it is found, memory is deallocated. 
     If not, the routine does nothing. A typical calling example is:

       NL_REAL    ***r3d;
       NL_STACKS  S;
       ...
       N_FreeReal3dArray(r3d,&S);


   ACCESS:
   
     r3d , input  ,  3-D real array pointer
     S   , input  ,  r3d's stack


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_FreeReal3dArray */
NL_VOID N_FreeReal3dArray( NL_REAL *** r3d, NL_STACKS *S )
{
    NL_R3DNODE *prev, *curr;

    NL_REAL ** r2d;

    /* Traverse memory stack to find pointer */

    if( S->r3d NEQ NULL )
    {
        prev = S->r3d;
        curr = S->r3d;

        while( curr NEQ NULL AND curr->ptr NEQ r3d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->r3d = NULL;
            }
            else /* More than one node */
            {
                S->r3d = S->r3d->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            r2d = r3d[0];
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
            N_FreeReal2dArray( r2d, S );
        }
    }
} /* end N_FreeReal3dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  deallocates  memory that  stores a point  
     array. Given a  pointer to the  array, the  routine searches for 
     the  pointer on  the  memory  stack. If it  is found, memory  is 
     deallocated. If not, the routine does nothing. A typical calling
     example is:

       NL_POINT   *p1d;
       NL_STACKS  S;
       ...
       N_FreePt1dArray(p1d,&S);


   ACCESS:
   
     p1d , input  ,  Point array pointer
     S   , input  ,  p1d's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreePt1dArray( NL_POINT *p1d, NL_STACKS *S )
{
    NL_P1DNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->p1d NEQ NULL )
    {
        prev = S->p1d;
        curr = S->p1d;

        while( curr NEQ NULL AND curr->ptr NEQ p1d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->p1d = NULL;
            }
            else /* More than one node */
            {
                S->p1d = S->p1d->next;
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
} /* end N_FreePt1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  deallocates memory that  stores a 2-D point
     array. Given a  pointer to  the array, the routine searches for the 
     pointer on the memory stack. If it is found, memory is deallocated. 
     If not, the routine does nothing. A typical calling example is:

       NL_POINT   **p2d;
       NL_STACKS  S;
       ...
       N_FreePt2dArray(p2d,&S);


   ACCESS:
   
     p2d , input  ,  2-D point array pointer
     S   , input  ,  p2d's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreePt2dArray( NL_POINT ** p2d, NL_STACKS *S )
{
    NL_P2DNODE *prev, *curr;

    NL_POINT *p1d;

    /* Traverse memory stack to find pointer */

    if( S->p2d NEQ NULL )
    {
        prev = S->p2d;
        curr = S->p2d;

        while( curr NEQ NULL AND curr->ptr NEQ p2d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->p2d = NULL;
            }
            else /* More than one node */
            {
                S->p2d = S->p2d->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            p1d = p2d[0];
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
            N_FreePt1dArray( p1d, S );
        }
    }
} /* end N_FreePt2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine deallocates memory that  stores a control  
     point array. Given a pointer to the array, the routine searches 
     for the  pointer on  the  memory  stack. It it is found, memory 
     is  deallocated. If  not, the  routine does  nothing. A typical
     calling example is:

       NL_CPOINT  *c1d;
       NL_STACKS  S;
       ...
       N_FreeCPt1dArray(c1d,&S);


   ACCESS:
   
     c1d , input  ,  Control point array pointer
     S   , input  ,  c1d's stack


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_FreeCPt1dArray */
NL_VOID N_FreeCPt1dArray( NL_CPOINT *c1d, NL_STACKS *S )
{
    NL_C1DNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->c1d NEQ NULL )
    {
        prev = S->c1d;
        curr = S->c1d;

        while( curr NEQ NULL AND curr->ptr NEQ c1d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->c1d = NULL;
            }
            else /* More than one node */
            {
                S->c1d = S->c1d->next;
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
} /* end N_FreeCPt1dArray */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine deallocates memory that stores a 2-D control  
     point  array. Given a  pointer to  the array, the routine searches 
     for the pointer on  the  memory  stack. It it is found,  memory is 
     deallocated. If not, the routine  does nothing. A typical  calling
     example is:

       NL_CPOINT  **c2d;
       NL_STACKS  S;
       ... 
       N_FreeCPt2dArray(c2d,&S);


   ACCESS:
   
     c2d , input  ,  2-D control point array pointer
     S   , input  ,  c2d's stack


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_FreeCPt2dArray */
NL_VOID N_FreeCPt2dArray( NL_CPOINT ** c2d, NL_STACKS *S )
{
    NL_C2DNODE *prev, *curr;

    NL_CPOINT *c1d;

    /* Traverse memory stack to find pointer */

    if( S->c2d NEQ NULL )
    {
        prev = S->c2d;
        curr = S->c2d;

        while( curr NEQ NULL AND curr->ptr NEQ c2d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->c2d = NULL;
            }
            else /* More than one node */
            {
                S->c2d = S->c2d->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            c1d = c2d[0];
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
            N_FreeCPt1dArray( c1d, S );
        }
    }
} /* end N_FreeCPt2dArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limit of a 
     flag array. It either extends the limit to  hold more elements, or 
     compacts the array to a smaller size. A typical calling example:

       NL_FLAG    *f1d;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       N_Realloc1dFlagArray(&f1d,n,m,&S);


   ACCESS:
   
     f1d , input  ,  Flag array pointer
     n   , input  ,  Highest index in f1d
     m   , input  ,  New highest index
     S   , input  ,  f1d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc1dFlagArray( NL_FLAG ** f1d, NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_FLAG *g1d, *h;

    NL_INDEX k, kl;

    /* Allocate new memory */

    g1d = N_AllocFlag1dArray( m, S );

    if( g1d EQ NULL )
        NL_QUIT;

    /* Copy f1d to g1d */

    h = *f1d;
    kl = NL_MIN( n, m );

    for ( k = 0; k <= kl; k++ )
        g1d[k] = h[k];

    /* Kill old memory and reassign pointer */

    N_FreeFlag1dArray( h, S );
    *f1d = g1d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc1dFlagArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limits of 
     a  2-D  flag  array. It  either  extends the limits to  hold more 
     elements, or  compacts the  array to  a smaller  size. A  typical 
     calling example is:

       NL_FLAG    **f2d;
       NL_INDEX   n, m, k, l;
       NL_STACKS  S;
       ...
       N_Realloc2dFlagArray(&f2d,n,m,k,l,&S);


   ACCESS:
   
     f2d , input  ,  Flag array pointer
     n,m , input  ,  Highest indexes in f2d
     k,l , input  ,  New highest indexes
     S   , input  ,  f2d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc2dFlagArray( NL_FLAG *** f2d, NL_INDEX n, NL_INDEX m, NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_FLAG ** g2d, ** h;

    NL_INDEX r, s, rl, sl;

    /* Allocate new memory */

    g2d = N_AllocFlag2dArray( k, l, S );

    if( g2d EQ NULL )
        NL_QUIT;

    /* Copy f2d to g2d */

    h = *f2d;
    rl = NL_MIN( k, n );
    sl = NL_MIN( l, m );

    for ( r = 0; r <= rl; r++ )
    {
        for ( s = 0; s <= sl; s++ )
            g2d[r][s] = h[r][s];
    }

    /* Kill old memory and reassign pointer */

    N_FreeFlag2dArray( h, S );
    *f2d = g2d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc2dFlagArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limit of an 
     integer array. It  either extends the limit to  hold more elements,
     or compacts the array to a smaller size. A typical calling example:

       NL_INTEGER  *i1d;
       NL_INDEX    n, m;
       NL_STACKS   S;
       ...
       N_Realloc1dIntArray(&i1d,n,m,&S);


   ACCESS:
   
     i1d , input  ,  Integer array pointer
     n   , input  ,  Highest index in i1d
     m   , input  ,  New highest index
     S   , input  ,  i1d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc1dIntArray( NL_INTEGER ** i1d, NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_INTEGER *j1d, *i;

    NL_INDEX k, kl;

    /* Allocate new memory */

    j1d = N_AllocInt1dArray( m, S );

    if( j1d EQ NULL )
        NL_QUIT;

    /* Copy i1d to j1d */

    i = *i1d;
    kl = NL_MIN( n, m );

    for ( k = 0; k <= kl; k++ )
        j1d[k] = i[k];

    /* Kill old memory and reassign pointer */

    N_FreeInt1dArray( i, S );
    *i1d = j1d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc1dIntArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limits of 
     a 1-D array of integer pointers. It either  extends the limits to  
     hold more elements, or  compacts the array to  a smaller  size. A  
     typical calling example is:

       NL_INTEGER  **i1p;
       NL_INDEX    n, m;
       NL_STACKS   S;
       ...
       N_Realloc1dIntPtrArray(&i1p,n,m,&S);


   ACCESS:
   
     i1p , input  ,  1-D array of integer pointer
     n   , input  ,  Highest index in i1p
     m   , input  ,  New highest index
     S   , input  ,  i1p's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc1dIntPtrArray( NL_INTEGER *** i1p, NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_INTEGER ** j1p, ** i;

    NL_INDEX k, kl;

    /* Allocate new memory */

    j1p = N_AllocIntPtr1dArray( m, S );

    if( j1p EQ NULL )
        NL_QUIT;

    /* Copy i1p to j1p */

    i = *i1p;
    kl = NL_MIN( n, m );

    for ( k = 0; k <= kl; k++ )
        j1p[k] = i[k];

    /* Kill old memory and reassign pointer */

    N_FreeIntPtr1dArray( i, S );
    *i1p = j1p;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc1dIntPtrArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limits of 
     a 2-D integer array. It  either  extends the limits to  hold more 
     elements, or  compacts the  array to  a smaller  size. A  typical 
     calling example is:

       NL_INTEGER  **i2d;
       NL_INDEX    n, m, k, l;
       NL_STACKS   S;
       ...
       N_Realloc2dIntArray(&i2d,n,m,k,l,&S);


   ACCESS:
   
     i2d , input  ,  Integer array pointer
     n,m , input  ,  Highest indexes in i2d
     k,l , input  ,  New highest indexes
     S   , input  ,  i2d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc2dIntArray( NL_INTEGER *** i2d, NL_INDEX n, NL_INDEX m, NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_INTEGER ** j2d, ** i;

    NL_INDEX r, s, rl, sl;

    /* Allocate new memory */

    j2d = N_AllocInt2dArray( k, l, S );

    if( j2d EQ NULL )
        NL_QUIT;

    /* Copy i2d to j2d */

    i = *i2d;
    rl = NL_MIN( k, n );
    sl = NL_MIN( l, m );

    for ( r = 0; r <= rl; r++ )
    {
        for ( s = 0; s <= sl; s++ )
            j2d[r][s] = i[r][s];
    }

    /* Kill old memory and reassign pointer */

    N_FreeInt2dArray( i, S );
    *i2d = j2d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc2dIntArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limits of 
     a 2-D array of integer pointers. It either  extends the limits to  
     hold more elements, or  compacts the array to  a smaller  size. A  
     typical calling example is:

       NL_INTEGER  ***i2p;
       NL_INDEX    n, m, k, l;
       NL_STACKS   S;
       ...
       N_Realloc2dIntPtrArray(&i2p,n,m,k,l,&S);


   ACCESS:
   
     i2p , input  ,  2-D array of integer pointer
     n,m , input  ,  Highest indexes in i2p
     k,l , input  ,  New highest indexes
     S   , input  ,  i2p's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc2dIntPtrArray( NL_INTEGER **** i2p, NL_INDEX n, NL_INDEX m, NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_INTEGER *** j2p, *** i;

    NL_INDEX r, s, rl, sl;

    /* Allocate new memory */

    j2p = N_AllocIntPtr2dArray( k, l, S );

    if( j2p EQ NULL )
        NL_QUIT;

    /* Copy i2p to j2p */

    i = *i2p;
    rl = NL_MIN( k, n );
    sl = NL_MIN( l, m );

    for ( r = 0; r <= rl; r++ )
    {
        for ( s = 0; s <= sl; s++ )
            j2p[r][s] = i[r][s];
    }

    /* Kill old memory and reassign pointer */

    N_FreeIntPtr2dArray( i, S );
    *i2p = j2p;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc2dIntPtrArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to  change the limit of a 
     real  array. It  either  extends the  limit to  hold more elements,
     or compacts the array to a smaller size. A typical calling example:

       NL_REAL    *r1d;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       N_Realloc1dRealArray(&r1d,n,m,&S);


   ACCESS:
   
     r1d , input  ,  Real array pointer
     n   , input  ,  Highest index in r1d
     m   , input  ,  New highest index
     S   , input  ,  r1d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc1dRealArray( NL_REAL ** r1d, NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_REAL *s1d, *r;

    NL_INDEX k, kl;

    /* Allocate new memory */

    s1d = N_AllocReal1dArray( m, S );

    if( s1d EQ NULL )
        NL_QUIT;

    /* Copy r1d to s1d */

    r = *r1d;
    kl = NL_MIN( n, m );

    for ( k = 0; k <= kl; k++ )
        s1d[k] = r[k];

    /* Kill old memory and reassign pointer */

    N_FreeReal1dArray( r, S );
    *r1d = s1d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc1dRealArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limits of 
     a 2-D real  array. It  either  extends  the  limits to  hold more 
     elements, or  compacts the  array to  a smaller  size. A  typical 
     calling example is:

       NL_REAL    **r2d;
       NL_INDEX   n, m, k, l;
       NL_STACKS  S;
       ...
       N_Realloc2dRealArray(&r2d,n,m,k,l,&S);


   ACCESS:
   
     r2d , input  ,  Real array pointer
     n,m , input  ,  Highest indexes in r2d
     k,l , input  ,  New highest indexes
     S   , input  ,  r2d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc2dRealArray( NL_REAL *** r2d, NL_INDEX n, NL_INDEX m, NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_REAL ** s2d, ** rp;

    NL_INDEX r, s, rl, sl;

    /* Allocate new memory */

    s2d = N_AllocReal2dArray( k, l, S );

    if( s2d EQ NULL )
        NL_QUIT;

    /* Copy r2d to s2d */

    rp = *r2d;
    rl = NL_MIN( k, n );
    sl = NL_MIN( l, m );

    for ( r = 0; r <= rl; r++ )
    {
        for ( s = 0; s <= sl; s++ )
            s2d[r][s] = rp[r][s];
    }

    /* Kill old memory and reassign pointer */

    N_FreeReal2dArray( rp, S );
    *r2d = s2d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc2dRealArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change  the limit of a 
     point  array. It  either extends  the limit to  hold more elements,
     or compacts the array to a smaller size. A typical calling example:

       NL_POINT   *p1d;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       N_Realloc1dPtArray(&p1d,n,m,&S);


   ACCESS:
   
     p1d , input  ,  Point array pointer
     n   , input  ,  Highest index in p1d
     m   , input  ,  New highest index
     S   , input  ,  p1d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc1dPtArray( NL_POINT ** p1d, NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_POINT *q1d, *p;

    NL_INDEX k, kl;

    /* Allocate new memory */

    q1d = N_AllocPt1dArray( m, S );

    if( q1d EQ NULL )
        NL_QUIT;

    /* Copy p1d to q1d */

    p = *p1d;
    kl = NL_MIN( n, m );

    for ( k = 0; k <= kl; k++ )
        N_CopyPt( p[k], &q1d[k] );

    /* Kill old memory and reassign pointer */

    N_FreePt1dArray( p, S );
    *p1d = q1d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc1dPtArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limits of 
     a 2-D point  array. It  either  extends  the limits to  hold more 
     elements, or  compacts the  array to  a smaller  size. A  typical 
     calling example is:

       NL_POINT   **p2d;
       NL_INDEX   n, m, k, l;
       NL_STACKS  S;
       ...
       N_Realloc2dPtArray(&p2d,n,m,k,l,&S);


   ACCESS:
   
     p2d , input  ,  Point array pointer
     n,m , input  ,  Highest indexes in p2d
     k,l , input  ,  New highest indexes
     S   , input  ,  p2d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc2dPtArray( NL_POINT *** p2d, NL_INDEX n, NL_INDEX m, NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_POINT ** q2d, ** p;

    NL_INDEX r, s, rl, sl;

    /* Allocate new memory */

    q2d = N_AllocPt2dArray( k, l, S );

    if( q2d EQ NULL )
        NL_QUIT;

    /* Copy p2d to q2d */

    p = *p2d;
    rl = NL_MIN( k, n );
    sl = NL_MIN( l, m );

    for ( r = 0; r <= rl; r++ )
    {
        for ( s = 0; s <= sl; s++ )
            N_CopyPt( p[r][s], &q2d[r][s] );
    }

    /* Kill old memory and reassign pointer */

    N_FreePt2dArray( p, S );
    *p2d = q2d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc2dPtArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limit of a 
     control point  array. It  either  extends the  limit to  hold more 
     elements, or  compacts  the  array  to a  smaller  size. A typical 
     calling example is:

       NL_CPOINT  *c1d;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       N_Realloc1dCPtArray(&c1d,n,m,&S);


   ACCESS:
   
     c1d , input  ,  Control point array pointer
     n   , input  ,  Highest index in c1d
     m   , input  ,  New highest index
     S   , input  ,  c1d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc1dCPtArray( NL_CPOINT ** c1d, NL_INDEX n, NL_INDEX m, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_CPOINT *d1d, *c;

    NL_INDEX k, kl;

    /* Allocate new memory */

    d1d = N_AllocCPt1dArray( m, S );

    if( d1d EQ NULL )
        NL_QUIT;

    /* Copy c1d to d1d */

    c = *c1d;
    kl = NL_MIN( n, m );

    for ( k = 0; k <= kl; k++ )
        N_CopyCPt( c[k], &d1d[k] );

    /* Kill old memory and reassign pointer */

    N_FreeCPt1dArray( c, S );
    *c1d = d1d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc1dCPtArray */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine reallocates  memory to change the limits of 
     a 2-D control point array. It either extends  the limits to  hold 
     more elements, or compacts the array to a smaller size. A typical 
     calling example is:

       NL_CPOINT  **c2d;
       NL_INDEX   n, m, k, l;
       NL_STACKS  S;
       ...
       N_Realloc2dCPtArray(&c2d,n,m,k,l,&S);


   ACCESS:
   
     c2d , input  ,  Control point array pointer
     n,m , input  ,  Highest indexes in c2d
     k,l , input  ,  New highest indexes
     S   , input  ,  c2d's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_Realloc2dCPtArray( NL_CPOINT *** c2d, NL_INDEX n, NL_INDEX m, NL_INDEX k, NL_INDEX l, NL_STACKS *S )
{
    NL_FLAG error = NL_NO;

    NL_CPOINT ** d2d, ** c;

    NL_INDEX r, s, rl, sl;

    /* Allocate new memory */

    d2d = N_AllocCPt2dArray( k, l, S );

    if( d2d EQ NULL )
        NL_QUIT;

    /* Copy c2d to d2d */

    c = *c2d;
    rl = NL_MIN( k, n );
    sl = NL_MIN( l, m );

    for ( r = 0; r <= rl; r++ )
    {
        for ( s = 0; s <= sl; s++ )
            N_CopyCPt( c[r][s], &d2d[r][s] );
    }

    /* Kill old memory and reassign pointer */

    N_FreeCPt2dArray( c, S );
    *c2d = d2d;

    /* Exit */

    EXIT:

    return (error);
} /* end N_Realloc2dCPtArray */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This routine shows a pictorial view of the memory stack. For each
     pointer entry a star "*" is  printed, and the  list is closed  by 
     the "|" character signifying the NULL pointer. A  typical calling 
     example is:
 
       NL_STACKS  S;
       ...
       N_PrintStack(&S);
 
 
   ACCESS:
   
     S  , input  ,  Memory stacks pointer
 
 
   RETURN CODES:
 
     None
 
   ***********************************************************************/

NL_VOID N_PrintStack( NL_STACKS *S )
{
    NL_F1DNODE *f1p;

    NL_F2DNODE *f2p;

    NL_I1DNODE *i1p;

    NL_I2DNODE *i2p;

    NL_I3DNODE *i3p;

    NL_R1DNODE *r1p;

    NL_R2DNODE *r2p;

    NL_R3DNODE *r3p;

    NL_R4DNODE *r4p;

    NL_P1DNODE *p1p;

    NL_P2DNODE *p2p;

    NL_P3DNODE *p3p;

    NL_P4DNODE *p4p;

    NL_C1DNODE *c1p;

    NL_C2DNODE *c2p;

    NL_C3DNODE *c3p;

    NL_C4DNODE *c4p;

    NL_CURNODE *cup;

    NL_CU2NODE *u2p;

    NL_CU3NODE *u3p;

    NL_CU4NODE *u4p;

    NL_SURNODE *sup;

    NL_VOLNODE *vop;

    NL_SU2NODE *s2p;

    NL_SU3NODE *s3p;

    NL_POLNODE *pop;

    NL_PPLNODE *ppp;

    NL_PP2NODE *pp2p;

    NL_NETNODE *nep;

    NL_MESHNODE *mep;

    NL_KNTNODE *knp;

    NL_KN2NODE *k2p;

    NL_CVLNODE *cvp;

    NL_SVLNODE *svp;

    NL_VVLNODE *vvp;

    NL_IMANODE *imp;

    NL_RMANODE *rmp;

    NL_PMANODE *pmp;

    NL_CMANODE *cmp;

    f1p = S->f1d;
    N_FPRINTF( stdout, _T("f1d: ") );

    while( f1p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        f1p = f1p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    f2p = S->f2d;
    N_FPRINTF( stdout, _T("f2d: ") );

    while( f2p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        f2p = f2p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    i1p = S->i1d;
    N_FPRINTF( stdout, _T("i1d: ") );

    while( i1p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        i1p = i1p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    i2p = S->i2d;
    N_FPRINTF( stdout, _T("i2d: ") );

    while( i2p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        i2p = i2p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    i3p = S->i3d;
    N_FPRINTF( stdout, _T("i3d: ") );

    while( i3p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        i3p = i3p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    r1p = S->r1d;
    N_FPRINTF( stdout, _T("r1d: ") );

    while( r1p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        r1p = r1p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    r2p = S->r2d;
    N_FPRINTF( stdout, _T("r2d: ") );

    while( r2p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        r2p = r2p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    r3p = S->r3d;
    N_FPRINTF( stdout, _T("r3d: ") );

    while( r3p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        r3p = r3p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    r4p = S->r4d;
    N_FPRINTF( stdout, _T("r4d: ") );

    while( r4p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        r4p = r4p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    p1p = S->p1d;
    N_FPRINTF( stdout, _T("p1d: ") );

    while( p1p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        p1p = p1p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    p2p = S->p2d;
    N_FPRINTF( stdout, _T("p2d: ") );

    while( p2p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        p2p = p2p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    p3p = S->p3d;
    N_FPRINTF( stdout, _T("p3d: ") );

    while( p3p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        p3p = p3p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    p4p = S->p4d;
    N_FPRINTF( stdout, _T("p4d: ") );

    while( p4p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        p4p = p4p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    c1p = S->c1d;
    N_FPRINTF( stdout, _T("c1d: ") );

    while( c1p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        c1p = c1p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    c2p = S->c2d;
    N_FPRINTF( stdout, _T("c2d: ") );

    while( c2p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        c2p = c2p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    c3p = S->c3d;
    N_FPRINTF( stdout, _T("c3d: ") );

    while( c3p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        c3p = c3p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    c4p = S->c4d;
    N_FPRINTF( stdout, _T("c4d: ") );

    while( c4p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        c4p = c4p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    cup = S->cur;
    N_FPRINTF( stdout, _T("cur: ") );

    while( cup NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        cup = cup->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    u2p = S->cu2;
    N_FPRINTF( stdout, _T("cu2: ") );

    while( u2p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        u2p = u2p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    u3p = S->cu3;
    N_FPRINTF( stdout, _T("cu3: ") );

    while( u3p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        u3p = u3p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    u4p = S->cu4;
    N_FPRINTF( stdout, _T("cu4: ") );

    while( u4p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        u4p = u4p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    sup = S->sur;
    N_FPRINTF( stdout, _T("sur: ") );

    while( sup NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        sup = sup->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    vop = S->vol;
    N_FPRINTF( stdout, _T("vol: ") );

    while( vop NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        vop = vop->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    s2p = S->su2;
    N_FPRINTF( stdout, _T("su2: ") );

    while( s2p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        s2p = s2p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    s3p = S->su3;
    N_FPRINTF( stdout, _T("su3: ") );

    while( s3p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        s3p = s3p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    pop = S->pol;
    N_FPRINTF( stdout, _T("pol: ") );

    while( pop NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        pop = pop->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    ppp = S->ppl;
    N_FPRINTF( stdout, _T("ppl: ") );

    while( ppp NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        ppp = ppp->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    pp2p = S->pp2;
    N_FPRINTF( stdout, _T("pp2: ") );

    while( pp2p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        pp2p = pp2p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    nep = S->net;
    N_FPRINTF( stdout, _T("net: ") );

    while( nep NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        nep = nep->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    mep = S->msh;
    N_FPRINTF( stdout, _T("msh: ") );

    while( mep NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        mep = mep->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    knp = S->knt;
    N_FPRINTF( stdout, _T("knt: ") );

    while( knp NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        knp = knp->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    k2p = S->kn2;
    N_FPRINTF( stdout, _T("kn2: ") );

    while( k2p NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        k2p = k2p->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    cvp = S->cvl;
    N_FPRINTF( stdout, _T("cvl: ") );

    while( cvp NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        cvp = cvp->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    svp = S->svl;
    N_FPRINTF( stdout, _T("svl: ") );

    while( svp NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        svp = svp->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    vvp = S->vvl;
    N_FPRINTF( stdout, _T("vvl: ") );

    while( vvp NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        vvp = vvp->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    imp = S->ima;
    N_FPRINTF( stdout, _T("ima: ") );

    while( imp NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        imp = imp->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    rmp = S->rma;
    N_FPRINTF( stdout, _T("rma: ") );

    while( rmp NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        rmp = rmp->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    pmp = S->pma;
    N_FPRINTF( stdout, _T("pma: ") );

    while( pmp NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        pmp = pmp->next;
    }
    N_FPRINTF( stdout, _T("|\n") );

    cmp = S->cma;
    N_FPRINTF( stdout, _T("cma: ") );

    while( cmp NEQ NULL )
    {
        N_FPRINTF( stdout, _T("*") );
        cmp = cmp->next;
    }
    N_FPRINTF( stdout, _T("|\n") );
} /* end N_PrintStack */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine initializes a temporary global curve pro-
     jector structure by setting pointers to NULL  and  nks/nip  to 
     -1. It is used to check if memory allocation is  needed,  i.e. 
     if the structure is initialized to NULL, memory is  allocated, 
     otherwise  it  is  assumed  that memory has already been allo-
     cated. A typical calling example is:

       NL_GCPTEMP  Tdata;
       ...
       N_CrvProjectionInitArrays(&Tdata);
     

   ACCESS:
   
     Tdata , in/out ,  Temporary global curve projector data


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_CrvProjectionInitArrays */
NL_VOID N_CrvProjectionInitArrays( NL_GCPTEMP *Tdata )
{
    Tdata->nks = -1;
    Tdata->nip = -1;

    Tdata->box = NULL;
    Tdata->cog = NULL;
    Tdata->pts = NULL;
    Tdata->us = NULL;
    Tdata->pflg = NULL;
} /* end N_CrvProjectionInitArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if two control points are equal or not. 
     A typical calling example is:

       NL_CPOINT  Pw, Qw;
       ...
       if( N_CPtsAreEqual(Pw,Qw) )  --> control points are equal;
     

   ACCESS:
   
     Pw , input ,  Control point
     Qw , input ,  Control point


   RETURN CODES:

     NL_TRUE : Control points are equal
     NL_FALSE: Control points are NOT equal

   ***********************************************************************/

/* NL_BOOLEAN  N_CPtsAreEqual */
NL_BOOLEAN N_CPtsAreEqual( NL_CPOINT Pw, NL_CPOINT Qw )
{
    NL_REAL d;

    NL_CPOINT Rw;

    N_Diff2CPts( Pw, Qw, &Rw );
    N_CPtMagnitude( Rw, &d );

    if( d LT NL_MTOL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_CPtsAreEqual */

/*******************************************************************//**


   DESCRIPTION:

     This routine creates an NL_IGES file and writes lines, curves and sur-
     faces to it as 110, 126 and 128 Entities. The NL_IGES file is named as 
     specified in the argument list.  Color of each entity can be speci-
     fied. The Start and Global sections of the NL_IGES file contain a min-
     imum amount of information;  this can be subsequently edited as de-
     sired. A typical calling example is:

       NL_POINT    **lines;
       NL_SURFACE  **surs;
       NL_CURVE    **curs;
       NL_INDEX    nl, nc, ns, *lcols, *ccols, *scols;
       NL_REAL     tol;
       NL_STRING   fname;
       ...
       (set file name, fname; load lines, curves and surfaces in arrays,
        lines, curs, and surs; and colors in lcols, ccols, and scols);
       ...
       N_WriteLineCrvSrfIges(lines,nl,lcols,curs,nc,ccols,surs,ns,tol,fname);


   ACCESS:
   
     lines , input  ,  Endpoints of the lines to be written to the  NL_IGES
                       file.  lines[i][0]  and  lines[i][1] are the end-
                       points of the i-th line
     nl    , input  ,  High index of lines (there are nl+1 lines).  nl<0
                       indicates no lines are input
     lcols , input  ,  Line colors (nl+1 color indexes).  See  NL_IGES  for
                       color values (or set any values 1 to 9 if not im-
                       portant).
     curs  , input  ,  Pointers to the NURBS curves to be written to the
                       NL_IGES file. curs[i] is a pointer to the i-th curve
     nc    , input  ,  High  index  of  curve  pointers  (there are nc+1
                       pointers in curs).  nc<0 indicates no  curves are 
                       input
     ccols , input  ,  Curve colors (nc+1 color indexes)
     surs  , input  ,  Pointers to the NURBS surfaces to be  written  to 
                       the NL_IGES file. surs[i] is a pointer to  the  i-th 
                       surface
     ns    , input  ,  High index  of surface  pointers  (there are ns+1
                       pointers in surs). ns<0 indicates no surfaces are 
                       input
     scols , input  ,  Surface colors (ns+1 color indexes)
     tol   , input  ,  Minimum model resolution tolerance  (Parameter 19
                       of the Global Section)
     fname , output ,  Name of the NL_IGES file (may  not  be  more than 30
                       characters long)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_WriteLineCrvSrfIges( NL_POINT ** lines, NL_INDEX nl, NL_INDEX *lcols, NL_CURVE ** curs, NL_INDEX nc, NL_INDEX *ccols, NL_SURFACE ** surs, NL_INDEX ns, NL_REAL tol, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_WriteLineCrvSrfIges");
    NL_PRIVATE NL_STRING blank = _T(" ");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, r, s, ii, jj, kk, npl, dline, pline;

    NL_INTEGER rat, closed, planar, uclo, vclo;

    NL_DEGREE p, q;

    NL_CPOINT *Pw, ** Qw;

    NL_REAL *U, *V, x, y, z, w, maxc, d1, d2, d3, d4, d5, d6;

    NL_MINMAXBOX box;

    NL_POINT norm;

    FILE *fptr = NULL;

    /* Check for input error and open file */

    if( nc LT 0 AND nl LT 0 AND ns LT 0 )
        NL_ERROR( NL_INP_ERR );
    fptr = N_FileOpen( fname, _T("w") );

    if( fptr EQ NULL )
        NL_ERROR( NL_FIL_ERR );

    /* Write Start Section */

    N_FPRINTF( fptr, _T("Nurbs lines/curves/surfaces written out using the%23sS%7d\n"), blank, 1 );

    /* Write Global Section */

    maxc = 0.0;

    for ( ii = 0; ii <= nc; ii++ )
    {
        N_CrvGetBBox( curs[ii], &box );
        N_GetBBoxData( &box, &d1, &d2, &d3, &d4, &d5, &d6 );

        if( fabs( d1 )GT maxc )
            maxc = fabs( d1 );

        if( fabs( d2 )GT maxc )
            maxc = fabs( d2 );

        if( fabs( d3 )GT maxc )
            maxc = fabs( d3 );

        if( fabs( d4 )GT maxc )
            maxc = fabs( d4 );

        if( fabs( d5 )GT maxc )
            maxc = fabs( d5 );

        if( fabs( d6 )GT maxc )
            maxc = fabs( d6 );
    }

    for ( ii = 0; ii <= nl; ii++ )
    {
        if( fabs( lines[ii][0].x )GT maxc )
            maxc = fabs( lines[ii][0].x );

        if( fabs( lines[ii][0].y )GT maxc )
            maxc = fabs( lines[ii][0].y );

        if( fabs( lines[ii][0].z )GT maxc )
            maxc = fabs( lines[ii][0].z );

        if( fabs( lines[ii][1].x )GT maxc )
            maxc = fabs( lines[ii][1].x );

        if( fabs( lines[ii][1].y )GT maxc )
            maxc = fabs( lines[ii][1].y );

        if( fabs( lines[ii][1].z )GT maxc )
            maxc = fabs( lines[ii][1].z );
    }

    for ( ii = 0; ii <= ns; ii++ )
    {
        N_SrfGetBBox( surs[ii], &box );
        N_GetBBoxData( &box, &d1, &d2, &d3, &d4, &d5, &d6 );

        if( fabs( d1 )GT maxc )
            maxc = fabs( d1 );

        if( fabs( d2 )GT maxc )
            maxc = fabs( d2 );

        if( fabs( d3 )GT maxc )
            maxc = fabs( d3 );

        if( fabs( d4 )GT maxc )
            maxc = fabs( d4 );

        if( fabs( d5 )GT maxc )
            maxc = fabs( d5 );

        if( fabs( d6 )GT maxc )
            maxc = fabs( d6 );
    }

    maxc = 1.001 *maxc;

    N_FPRINTF( fptr, _T(",,8HNlib 8.0,30H%30s,1H ,1H ,%17sG%7d\n"), fname, blank, 1 );
    N_FPRINTF( fptr, _T("%3d,%3d,%2d,%3d,%3d,1H , %1.1f,1,2HIN,%37sG%7d\n"), 32, 38, 7, 308, 15, 1.0, blank, 2 );
    N_FPRINTF( fptr, _T("1, %5.3f,13H             , %18.13lf,%26sG%7d\n"), 0.001, tol, blank, 3 );
    N_FPRINTF( fptr, _T("%22.10lf,1H ,1H ,%3d,0,%35sG%7d\n"), maxc, 9, blank, 4 );
    N_FPRINTF( fptr, _T("13H             ;%55sG%7d\n"), blank, 5 );

    /* Write the Directory Entries */

    pline = 1;
    dline = 1;

    /*  First for the lines  */

    for ( ii = 0; ii <= nl; ii++ )
    {
        npl = 3; /* Number of lines in Parameter Data Section */

        /* Write the Directory Entry for this line */

        N_FPRINTF( fptr, _T("%8d%8ld%8d%8d%8d%8d%8d%8d00000000D%7ld\n"), 110, pline, 0, 1, 4, 0, 0, 0, dline );
        dline += 1;
        N_FPRINTF( fptr, _T("%8d%8d%8ld%8ld%8d%8d%8dNURBSURF%8ldD%7ld\n"), 110, 1, lcols[ii], npl, 0, 0, 0, ii + 1, dline );
        dline += 1;

        pline += npl;
    }

    /*  Now for the curves  */

    for ( ii = 0; ii <= nc; ii++ )
    {
        /* Compute number of lines in Parameter Data Section */

        N_CrvGetArraySizes( curs[ii], &n, &m );

        npl = 3;            /* integer data, trim bounds & unit normal */
        npl += (m / 3 + 1); /* knots */
        npl += (n / 4 + 1); /* weights */
        npl += (n + 1);     /* control points */

        /* Write the Directory Entry for this curve */

        N_FPRINTF( fptr, _T("%8d%8ld%8d%8d%8d%8d%8d%8d00000000D%7ld\n"), 126, pline, 0, 1, 4, 0, 0, 0, dline );
        dline += 1;
        N_FPRINTF( fptr, _T("%8d%8d%8ld%8ld%8d%8d%8dNURBSURF%8ldD%7ld\n"), 126, 1, ccols[ii], npl, 0, 0, 0, ii + 1, dline );
        dline += 1;

        pline += npl;
    }

    /* Now for the surfaces */

    for ( ii = 0; ii <= ns; ii++ )
    {
        /* Compute number of lines in Parameter Data Section */

        N_SrfGetArraySizes( surs[ii], &n, &m, &r, &s );

        npl = 3;                        /* integer data & trim bounds */
        npl += (r / 3 + 1);             /* u knots */
        npl += (s / 3 + 1);             /* v knots */
        npl += ((m + 1) * (n / 4 + 1)); /* weights */
        npl += ((m + 1) * (n + 1));     /* control points */

        /* Write the Directory Entry for this surface */

        N_FPRINTF( fptr, _T("%8d%8ld%8d%8d%8d%8d%8d%8d00000000D%7ld\n"), 128, pline, 0, 1, 4, 0, 0, 0, dline );
        dline += 1;
        N_FPRINTF( fptr, _T("%8d%8d%8ld%8ld%8d%8d%8dNURBSURF%8ldD%7ld\n"), 128, 1, (ii + 2) % 8, npl, 0, 0, 0, ii + 1, dline );
        dline += 1;

        pline += npl;
    }

    /* Write the Parameter Data Section */

    pline = 1;

    /*  First for the lines  */

    for ( ii = 0; ii <= nl; ii++ )
    {
        /* Write the integer data */

        N_FPRINTF( fptr, _T("%3d,%61s%7ldP%7ld\n"), 110, blank, 2 * ii + 1, pline );
        pline += 1;

        /* Write the endpoints */

        N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), lines[ii][0].x, lines[ii][0].y, lines[ii][0].z, 2 * ii + 1, pline );

        pline += 1;

        N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf; %7ldP%7ld\n"), lines[ii][1].x, lines[ii][1].y, lines[ii][1].z, 2 * ii + 1, pline );

        pline += 1;
    }

    /*  Now for the curves  */

    for ( ii = 0; ii <= nc; ii++ )
    {
        /* Get local notation */

        N_CrvGetCPtsDegreeAndKnots( curs[ii], &n, &Pw, &p, &m, &U );

        /* Compute the Properties */

        if( N_CrvIsClosed( curs[ii] ) )
            closed = 1;
        else
            closed = 0;

        rat = 1;

        if( N_IsCrvRat( curs[ii] ) )
        {
            for ( jj = 0; jj <= n; jj++ )
            {
                N_CPtGetW( Pw[jj], &w );

                if( w NEQ 1.0 )
                {
                    rat = 0;
                    break;
                }
            }
        }

        N_CopyPt( NL_ZERO, &norm );

        if( N_CrvIs3d( curs[ii] ) )
            planar = 0;     /* just flag as planar */
        else                /* if xy-planar        */
        {                   /* in xy-plane */
            if( N_CrvIsLine( curs[ii], tol ) )
                planar = 0; /* plane not unique */
            else
            {
                planar = 1;
                N_PtFromXYZ( 0.0, 0.0, 1.0, &norm );
            }
        }

        /* Write the integer data */

        N_FPRINTF( fptr, _T("%3d,%5ld,%3d,%2ld,%2ld,%2ld,%2d,%39s%7ldP%7ld\n"), 126, n, p, planar, closed, rat, 0, blank, 2 * (ii + nl) + 3, pline );
        pline += 1;

        /* Write the knots */

        for ( jj = 0; jj <= m; jj += 3 )
        {
            if( jj EQ m )
                N_FPRINTF( fptr, _T("%19.12lf,%44s %7ldP%7ld\n"), U[jj], blank, 2 * (ii + nl) + 3, pline );

            else if( jj EQ m - 1 )
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), U[jj], U[jj + 1], blank, 2 *( ii + nl ) + 3, pline );

            else
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), U[jj], U[jj + 1], U[jj + 2], 2 *( ii + nl ) + 3, pline );

            pline += 1;
        }

        /* Write the weights */

        if( rat EQ 1 )
        { /* non-rational */
            for ( jj = 0; jj <= n; jj += 4 )
            {
                if( jj EQ n )
                    N_FPRINTF( fptr, _T("%13lf,%50s %7ldP%7ld\n"), 1.0, blank, 2 * (ii + nl) + 3, pline );

                else if( jj EQ n - 1 )
                    N_FPRINTF( fptr, _T("%13lf , %13lf,%34s %7ldP%7ld\n"), 1.0, 1.0, blank, 2 *( ii + nl ) + 3, pline );

                else if( jj EQ n - 2 )
                    N_FPRINTF( fptr, _T("%13lf , %13lf , %13lf,%18s %7ldP%7ld\n"), 1.0, 1.0, 1.0, blank, 2 *( ii + nl ) + 3, pline );

                else
                    N_FPRINTF( fptr, _T("%13lf , %13lf , %13lf , %13lf,   %7ldP%7ld\n"), 1.0, 1.0, 1.0, 1.0, 2 *( ii + nl ) + 3, pline );

                pline += 1;
            }
        }
        else
        { /* rational */
            for ( jj = 0; jj <= n; jj += 4 )
            {
                if( jj EQ n )
                {
                    N_CPtGetW( Pw[jj], &d1 );
                    N_FPRINTF( fptr, _T("%13.7lf,%50s %7ldP%7ld\n"), d1, blank, 2 * (ii + nl) + 3, pline );
                }
                else if( jj EQ n - 1 )
                {
                    N_CPtGetW( Pw[jj], &d1 );
                    N_CPtGetW( Pw[jj + 1], &d2 );
                    N_FPRINTF( fptr, _T("%13.7lf , %13.7lf,%34s %7ldP%7ld\n"), d1, d2, blank, 2 * (ii + nl) + 3, pline );
                }
                else if( jj EQ n - 2 )
                {
                    N_CPtGetW( Pw[jj], &d1 );
                    N_CPtGetW( Pw[jj + 1], &d2 );
                    N_CPtGetW( Pw[jj + 2], &d3 );
                    N_FPRINTF( fptr, _T("%13.7lf , %13.7lf , %13.7lf,%18s %7ldP%7ld\n"), d1, d2, d3, blank, 2 * (ii + nl) + 3, pline );
                }
                else
                {
                    N_CPtGetW( Pw[jj], &d1 );
                    N_CPtGetW( Pw[jj + 1], &d2 );
                    N_CPtGetW( Pw[jj + 2], &d3 );
                    N_CPtGetW( Pw[jj + 3], &d4 );
                    N_FPRINTF( fptr, _T("%13.7lf , %13.7lf , %13.7lf , %13.7lf,   %7ldP%7ld\n"), d1, d2, d3, d4, 2 * (ii + nl) + 3, pline );
                }

                pline += 1;
            }
        }

        /* Write the Euclidean control points */

        for ( jj = 0; jj <= n; jj++ )
        {
            N_CPtToXYZ( Pw[jj], &x, &y, &z );
            N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), x, y, z, 2 * (ii + nl) + 3, pline );

            pline += 1;
        }

        /* Write the trim bounds */

        N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), U[0], U[m], blank, 2 * (ii + nl) + 3, pline );
        pline += 1;

        /* Write the unit normal */

        N_PtToXYZ( norm, &x, &y, &z );
        N_FPRINTF( fptr, _T("%15lf , %15lf , %15lf;%12s %7ldP%7ld\n"), x, y, z, blank, 2 * (ii + nl) + 3, pline );
        pline += 1;
    }

    /* Now for the surfaces */

    for ( ii = 0; ii <= ns; ii++ )
    {
        /* Get local notation */

        N_SrfGetCPtsDegreesAndKnots( surs[ii], &n, &m, &Qw, &p, &q, &r, &s, &U, &V );

        /* Compute the Properties */

        if( N_SrfIsClosed( surs[ii], NL_UDIR ) )
            uclo = 1;
        else
            uclo = 0;

        if( N_SrfIsClosed( surs[ii], NL_VDIR ) )
            vclo = 1;
        else
            vclo = 0;

        rat = 1;

        if( N_IsSrfRat( surs[ii] ) )
        {
            for ( jj = 0; jj <= n; jj++ )
            {
                for ( kk = 0; kk <= m; kk++ )
                {
                    N_CPtGetW( Qw[jj][kk], &w );

                    if( w NEQ 1.0 )
                    {
                        rat = 0;
                        break;
                    }
                }

                if( rat EQ 0 )
                    break;
            }
        }

        /* Write the integer data */

        N_FPRINTF( fptr, _T("%3d,%5ld,%5ld,%3d,%3d,%2ld,%2ld,%2ld,%2d,%2d,%26s%7ldP%7ld\n"), 128, n, m, p, q, uclo, vclo, rat, 0, 0, blank, 2 * (ii + nl + nc) + 5, pline );
        pline += 1;

        /* Write the u-knots */

        for ( jj = 0; jj <= r; jj += 3 )
        {
            if( jj EQ r )
                N_FPRINTF( fptr, _T("%19.12lf,%44s %7ldP%7ld\n"), U[jj], blank, 2 * (ii + nl + nc) + 5, pline );

            else if( jj EQ r - 1 )
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), U[jj], U[jj + 1], blank, 2 *( ii + nl + nc ) + 5, pline );

            else
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), U[jj], U[jj + 1], U[jj + 2], 2 *( ii + nl + nc ) + 5, pline );

            pline += 1;
        }

        /* Write the v-knots */

        for ( jj = 0; jj <= s; jj += 3 )
        {
            if( jj EQ s )
                N_FPRINTF( fptr, _T("%19.12lf,%44s %7ldP%7ld\n"), V[jj], blank, 2 * (ii + nl + nc) + 5, pline );

            else if( jj EQ s - 1 )
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), V[jj], V[jj + 1], blank, 2 *( ii + nl + nc ) + 5, pline );

            else
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), V[jj], V[jj + 1], V[jj + 2], 2 *( ii + nl + nc ) + 5, pline );

            pline += 1;
        }

        /* Write the weights */

        if( rat EQ 1 )
        { /* non-rational */
            for ( jj = 0; jj <= m; jj++ )
                for ( kk = 0; kk <= n; kk += 4 )
                {
                    if( kk EQ n )
                        N_FPRINTF( fptr, _T("%13lf,%50s %7ldP%7ld\n"), 1.0, blank, 2 * (ii + nl + nc) + 5, pline );

                    else if( kk EQ n - 1 )
                        N_FPRINTF( fptr, _T("%13lf , %13lf,%34s %7ldP%7ld\n"), 1.0, 1.0, blank, 2 *( ii + nl + nc ) + 5, pline );

                    else if( kk EQ n - 2 )
                        N_FPRINTF( fptr, _T("%13lf , %13lf , %13lf,%18s %7ldP%7ld\n"), 1.0, 1.0, 1.0, blank, 2 *( ii + nl + nc ) + 5, pline );

                    else
                        N_FPRINTF( fptr, _T("%13lf , %13lf , %13lf , %13lf,   %7ldP%7ld\n"), 1.0, 1.0, 1.0, 1.0, 2 *( ii + nl + nc ) + 5, pline );

                    pline += 1;
                }
        }
        else
        { /* rational */
            for ( jj = 0; jj <= m; jj++ )
                for ( kk = 0; kk <= n; kk += 4 )
                {
                    if( kk EQ n )
                    {
                        N_CPtGetW( Qw[kk][jj], &d1 );
                        N_FPRINTF( fptr, _T("%13.7lf,%50s %7ldP%7ld\n"), d1, blank, 2 * (ii + nl + nc) + 5, pline );
                    }
                    else if( kk EQ n - 1 )
                    {
                        N_CPtGetW( Qw[kk][jj], &d1 );
                        N_CPtGetW( Qw[kk + 1][jj], &d2 );
                        N_FPRINTF( fptr, _T("%13.7lf , %13.7lf,%34s %7ldP%7ld\n"), d1, d2, blank, 2 * (ii + nl + nc) + 5, pline );
                    }
                    else if( kk EQ n - 2 )
                    {
                        N_CPtGetW( Qw[kk][jj], &d1 );
                        N_CPtGetW( Qw[kk + 1][jj], &d2 );
                        N_CPtGetW( Qw[kk + 2][jj], &d3 );
                        N_FPRINTF( fptr, _T("%13.7lf , %13.7lf , %13.7lf,%18s %7ldP%7ld\n"), d1, d2, d3, blank, 2 * (ii + nl + nc) + 5, pline );
                    }
                    else
                    {
                        N_CPtGetW( Qw[kk][jj], &d1 );
                        N_CPtGetW( Qw[kk + 1][jj], &d2 );
                        N_CPtGetW( Qw[kk + 2][jj], &d3 );
                        N_CPtGetW( Qw[kk + 3][jj], &d4 );
                        N_FPRINTF( fptr, _T("%13.7lf , %13.7lf , %13.7lf , %13.7lf,   %7ldP%7ld\n"), d1, d2, d3, d4, 2 * (ii + nl + nc) + 5, pline );
                    }

                    pline += 1;
                }
        }

        /* Write the Euclidean control points */

        for ( jj = 0; jj <= m; jj++ )
            for ( kk = 0; kk <= n; kk++ )
            {
                N_CPtToXYZ( Qw[kk][jj], &x, &y, &z );
                N_FPRINTF( fptr, _T("%19.12lf , %19.12lf , %19.12lf, %7ldP%7ld\n"), x, y, z, 2 * (ii + nl + nc) + 5, pline );

                pline += 1;
            }

        /* Write the u,v trim bounds */

        N_FPRINTF( fptr, _T("%19.12lf , %19.12lf,%22s %7ldP%7ld\n"), U[0], U[r], blank, 2 * (ii + nl + nc) + 5, pline );
        pline += 1;
        N_FPRINTF( fptr, _T("%19.12lf , %19.12lf;%22s %7ldP%7ld\n"), V[0], V[s], blank, 2 * (ii + nl + nc) + 5, pline );
        pline += 1;
    }

    /* Write the Terminate Section */

    N_FPRINTF( fptr, _T("S%7dG%7dD%7ldP%7ld%40sT%7d\n"), 1, 5, dline - 1, pline - 1, blank, 1 );

    /* Exit */

    EXIT:

    N_FileClose( fptr );

    return (error);
} /* end N_WriteLineCrvSrfIges */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates an  NL_IGES file for visualizing a point 
     set by marking each point with a star or a cube. The NL_IGES file  is 
     named as specified in the argument list. A typical calling example 
     is:

       NL_POINT  *P;
       NL_INDEX   pcl, mark;
       NL_REAL    tol, size;
       NL_STRING  fname;
       ...
       (get file name, P, pcl, size, mark and tol);
       ...
       N_WritePtsIges(P,k,tol,pcl,mark,size,fname);


   ACCESS:
   
     P     , input  ,  Point array
     k     , input  ,  High index in P
     tol   , input  ,  Minimum model resolution tolerance (Parameter 19
                       of the Global Section)
     pcl   , input  ,  Point color.  See NL_IGES for  color values (or set 
                       any values 1 to 9 if not important).
     mark  , input  ,  Mark type
                         1: star
                         2: cube
     size  , input  ,  Mark size
     fname , output ,  Name of the NL_IGES file (may  not  be more than 30
                       characters long)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_WritePtsIges( NL_POINT *P, NL_INDEX k, NL_REAL tol, NL_INDEX pcl, NL_INDEX mark, NL_REAL size, TCHAR* fname )
{
    NL_PRIVATE NL_STRING rname = _T("N_WritePtsIges");

    NL_FLAG error = NL_NO;

    NL_INDEX *col, i, nq;

    NL_REAL x, y, z;

    NL_POINT ** Q;

    NL_STACKS SL;

    /* Open Nlib */

    N_InitNurbs( &SL );

    /* Get memory */

    if( mark EQ 1 )
        nq = 3 * (k + 1);

    else if( mark EQ 2 )
        nq = 12 *( k + 1 );

    else
        NL_ERROR( NL_INP_ERR );

    Q = N_AllocPt2dArray( nq, 1, &SL );

    if( Q EQ NULL )
        NL_QUIT;

    col = N_AllocInt1dArray( nq, &SL );

    if( col EQ NULL )
        NL_QUIT;

    /* Load line segments */

    nq = -1;

    if( mark EQ 1 )
    {
        for ( i = 0; i <= k; i++ )
        {
            N_PtToXYZ( P[i], &x, &y, &z );

            nq++;
            N_PtFromXYZ( x - size, y, z, &Q[nq][0] );
            N_PtFromXYZ( x + size, y, z, &Q[nq][1] );
            col[nq] = pcl;

            nq++;
            N_PtFromXYZ( x, y - size, z, &Q[nq][0] );
            N_PtFromXYZ( x, y + size, z, &Q[nq][1] );
            col[nq] = pcl;

            nq++;
            N_PtFromXYZ( x, y, z - size, &Q[nq][0] );
            N_PtFromXYZ( x, y, z + size, &Q[nq][1] );
            col[nq] = pcl;
        }
    }
    else if( mark EQ 2 )
    {
        for ( i = 0; i <= k; i++ )
        {
            N_PtToXYZ( P[i], &x, &y, &z );

            nq++;
            N_PtFromXYZ( x - size, y - size, z - size, &Q[nq][0] );
            N_PtFromXYZ( x + size, y - size, z - size, &Q[nq][1] );
            col[nq] = pcl;
            nq++;
            N_PtFromXYZ( x + size, y - size, z - size, &Q[nq][0] );
            N_PtFromXYZ( x + size, y + size, z - size, &Q[nq][1] );
            col[nq] = pcl;
            nq++;
            N_PtFromXYZ( x + size, y + size, z - size, &Q[nq][0] );
            N_PtFromXYZ( x - size, y + size, z - size, &Q[nq][1] );
            col[nq] = pcl;
            nq++;
            N_PtFromXYZ( x - size, y + size, z - size, &Q[nq][0] );
            N_PtFromXYZ( x - size, y - size, z - size, &Q[nq][1] );
            col[nq] = pcl;

            nq++;
            N_PtFromXYZ( x - size, y - size, z + size, &Q[nq][0] );
            N_PtFromXYZ( x + size, y - size, z + size, &Q[nq][1] );
            col[nq] = pcl;
            nq++;
            N_PtFromXYZ( x + size, y - size, z + size, &Q[nq][0] );
            N_PtFromXYZ( x + size, y + size, z + size, &Q[nq][1] );
            col[nq] = pcl;
            nq++;
            N_PtFromXYZ( x + size, y + size, z + size, &Q[nq][0] );
            N_PtFromXYZ( x - size, y + size, z + size, &Q[nq][1] );
            col[nq] = pcl;
            nq++;
            N_PtFromXYZ( x - size, y + size, z + size, &Q[nq][0] );
            N_PtFromXYZ( x - size, y - size, z + size, &Q[nq][1] );
            col[nq] = pcl;

            nq++;
            N_PtFromXYZ( x - size, y - size, z - size, &Q[nq][0] );
            N_PtFromXYZ( x - size, y - size, z + size, &Q[nq][1] );
            col[nq] = pcl;
            nq++;
            N_PtFromXYZ( x + size, y - size, z - size, &Q[nq][0] );
            N_PtFromXYZ( x + size, y - size, z + size, &Q[nq][1] );
            col[nq] = pcl;
            nq++;
            N_PtFromXYZ( x + size, y + size, z - size, &Q[nq][0] );
            N_PtFromXYZ( x + size, y + size, z + size, &Q[nq][1] );
            col[nq] = pcl;
            nq++;
            N_PtFromXYZ( x - size, y + size, z - size, &Q[nq][0] );
            N_PtFromXYZ( x - size, y + size, z + size, &Q[nq][1] );
            col[nq] = pcl;
        }
    }

    /* Create NL_IGES file */

    error = N_WriteLineCrvSrfIges( Q, nq, col, NULL, -1, NULL, NULL, -1, tol, fname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Close Nlib */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_WritePtsIges */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates an NL_IGES file for surface visualization
     with  or  without the  control  net. The  NL_IGES  file  is  named  as 
     specified in the argument list. A typical calling example is:

       NL_SURFACE  **surs;
       NL_INDEX    lcl, ns;
       NL_REAL     tol;
       NL_STRING   fname;
       ...
       (get file name, surs, scl, lcl, tol);
       ...
       N_WriteSrfIges(surs,ns,tol,NL_YES,NL_NO,lcl,fname);


   ACCESS:
   
     surs  , input  ,  Pointers to the NURBS  surfaces to be written  to
                       the NL_IGES file. surs[i] is  a pointer  to the i-th
                       surface
     ns    , input  ,  High index of surface  pointers  (there are ns+1
                       pointers in surs)
     tol   , input  ,  Minimum model resolution tolerance  (Parameter 19
                       of the Global Section)
     sfl   , input  ,  Flag:
                         NL_YES: output surface
                         NL_NO : do not output surface
     nfl   , input  ,  Flag:
                         NL_YES: output control net
                         NL_NO : do not output control net
     lcl   , input  ,  Color index for control nets
     fname , output ,  Name of the NL_IGES file (may  not  be more than 30
                       characters long)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_WriteSrfIges( NL_SURFACE ** surs, NL_INDEX ns, NL_REAL tol, NL_FLAG sfl, NL_FLAG nfl, NL_INDEX lcl, TCHAR* fname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX *col = NULL, i, j, k, nl, np, n, m, r, s, nh, mh;

    NL_POINT ** P;

    NL_CPOINT ** Pw;

    NL_SURFACE ** surl;

    NL_STACKS SL;

    /* Open Nlib */

    N_InitNurbs( &SL );

    /* See if surface is to be output */

    if( sfl EQ NL_YES )
    {
        surl = surs;
        nl = ns;
    }
    else
    {
        surl = NULL;
        nl = -1;
    }

    /* See if net is to be output */

    if( nfl EQ NL_NO )
    {
        P = NULL;
        np = -1;
    }
    else
    {
        /* Get number of lines */

        np = 0;

        for ( k = 0; k <= ns; k++ )
        {
            N_SrfGetArraySizes( surs[k], &n, &m, &r, &s );

            np += m * (n + 1) + n * (m + 1) + 1;
        }

        P = N_AllocPt2dArray( np, 1, &SL );

        if( P EQ NULL )
            NL_QUIT;

        col = N_AllocInt1dArray( np, &SL );

        if( col EQ NULL )
            NL_QUIT;

        /* Load line segments */

        np = -1;

        for ( k = 0; k <= ns; k++ )
        {
            N_SrfGetCPts( surs[k], &n, &m, &Pw );

            if( N_SrfIsClosed( surs[k], NL_VDIR ) )
                mh = m - 1;
            else
                mh = m;

            for ( j = 0; j <= mh; j++ )
            {
                for ( i = 1; i <= n; i++ )
                {
                    np++;
                    N_CPtToPtEuclid( Pw[i - 1][j], &P[np][0] );
                    N_CPtToPtEuclid( Pw[i][j], &P[np][1] );
                    col[np] = lcl;
                }
            }

            if( N_SrfIsClosed( surs[k], NL_UDIR ) )
                nh = n - 1;
            else
                nh = n;

            for ( i = 0; i <= nh; i++ )
            {
                for ( j = 1; j <= m; j++ )
                {
                    np++;
                    N_CPtToPtEuclid( Pw[i][j - 1], &P[np][0] );
                    N_CPtToPtEuclid( Pw[i][j], &P[np][1] );
                    col[np] = lcl;
                }
            }
        }
    }

    /* Create NL_IGES file */

    error = N_WriteLineCrvSrfIges( P, np, col, NULL, -1, NULL, surl, nl, tol, fname );

    if( error EQ NL_YES )
        NL_OUT;

    /* Close Nlib */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_WriteSrfIges */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine  checks if a 1-D point set is co-circular or 
     not. It fits a circle to the points and checks either the maximum 
     or the average  error. The best fitting circle  along with a flag
     indicator are returned. A typical calling example is:

       NL_FLAG    cir;
       NL_INDEX   n;
       NL_REAL    tol, r;
       NL_POINT   *P, C;
       ...
       (get P and choose tol);
       ...
       N_IsPtSet1dCircular(P,n,tol,NL_AVERAGE,&C,&r,&cir)
     

   ACCESS:
   
     P   , input  ,  Point set
     n   , input  ,  Highest index in P
     tol , input  ,  Tolerance to check co-circularity to radius
     tfl , input  ,  Flag:
                       NL_MAXIMUM: tol is maximum deviation
                       NL_AVERAGE: tol is average deviation
     C   , output ,  Center of the circle
     r   , output ,  Radius of the circle
     cir , output ,  Flag:
                       NL_TRUE : points are co-circular
                       NL_FALSE: points are NOT co-circular


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsPtSet1dCircular
 ( NL_POINT * P,    /* in : Point Set                                   */
   NL_INDEX   n,    /* in : Highest index in P                          */
   NL_REAL    tol,  /* in : Tolerance to check co-circularity to radius */
   NL_FLAG    tfl,  /* in : NL_MAXIMUM: tol is maximum deviation        */
                    /*      NL_AVERAGE: tol is average deviation        */
   NL_POINT * C,    /* out: Center of the circle                        */
   NL_REAL  * r,    /* out: Radius of the circle                        */
   NL_FLAG  * cir ) /* out: NL_TRUE : points are co-circular            */
                    /*      NL_FALSE: points are NOT co-circular        */
{                   
    NL_FLAG cfl, error = NL_NO;

    NL_REAL cx, cy, era, erm;

    /* Check co-circularity */

    *cir = NL_FALSE;

    /* get a best fit circle to the given point set */
    error = N_CreateCircFrom2dPts( P,          /* in : Points in 2-D                                 */
                                   n,          /* in : Highest index in PP                           */
                                   NL_MTOL,    /* in : Tolerance to check collinearity to radius     */
                                   NL_AVERAGE, /* in : NL_MAXIMUM: tol is maximum deviation          */
                                               /*      NL_AVERAGE: tol is average deviation          */
                                   &cx,        /* out: x of Circle Center                            */
                                   &cy,        /* out: y of Circle Center                            */
                                   r,          /* out: Radius of circle                              */
                                   &era,       /* out: Average absolute error                        */
                                   &erm,       /* out: Maximum absolute error                        */
                                   &cfl );     /* out: NL_TRUE : circle computed                     */
                                               /*      NL_FALSE: points are collinear or degenerate, */
                                               /*                no circle computed                  */

    if( error EQ NL_YES OR cfl EQ NL_FALSE )
        NL_OUT;

    switch( tfl )
    {
        case NL_MAXIMUM:
            if( erm LT tol )
                *cir = NL_TRUE;
            break;

        case NL_AVERAGE:
            if( era LT tol )
                *cir = NL_TRUE;
            break;

        default:
            *cir = NL_FALSE;
    }

    N_PtFromXYZ( cx, cy, 0.0, C );

    /* Exit */

    EXIT:

    return (error);
} /* end N_IsPtSet1dCircular */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine  checks if a 1-D point  set  is collinear or 
     not. It fits a line to the points and checks either the maximum or
     the  average  error. The  best  fitting  line  along  with  a flag 
     indicator are returned. A typical calling example is:

       NL_FLAG    lin;
       NL_INDEX   n;
       NL_REAL    tol;
       NL_VECTOR  V;
       NL_POINT   *P, C;
       ...
       (get P and choose tol);
       ...
       N_IsPtSet1dColinear(P,n,tol,NL_AVERAGE,&C,&V,&lin)
     

   ACCESS:
   
     P   , input  ,  Point set
     n   , input  ,  Highest index in P
     tol , input  ,  Tolerance to check collinearity
     tfl , input  ,  Flag:
                       NL_MAXIMUM: tol is maximum deviation
                       NL_AVERAGE: tol is average deviation
     C   , output ,  Point on the best fit line
     V   , output ,  Direction vector of line
     lin , output ,  Flag:
                       NL_TRUE : points are collinear
                       NL_FALSE: points are NOT collinear


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsPtSet1dColinear( NL_POINT *P, NL_INDEX n, NL_REAL tol, NL_FLAG tfl, NL_POINT *C, NL_VECTOR *V, NL_FLAG *lin )
{
    NL_FLAG error = NL_NO;

    NL_REAL era, erm;

    /* Check collinearity */

    *lin = NL_FALSE;

    error = N_LineFitPts( P, n, C, V, &era, &erm );

    if( error EQ NL_YES )
        NL_OUT;

    switch( tfl )
    {
        case NL_MAXIMUM:
            if( erm LT tol )
                *lin = NL_TRUE;
            break;

        case NL_AVERAGE:
            if( era LT tol )
                *lin = NL_TRUE;
            break;

        default:
            *lin = NL_FALSE;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_IsPtSet1dColinear */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine  checks if  a 1-D point  set  is coplanar  or 
     not. It fits a plane to the points and checks either the maximum or
     the  average  error. The  best  fitting  plane  along  with  a flag 
     indicator are returned. A typical calling example is:

       NL_FLAG    pln;
       NL_INDEX   n;
       NL_REAL    tol;
       NL_VECTOR  N;
       NL_POINT   *P, C;
       ...
       (get P and choose tol);
       ...
       N_IsPtSet1dCoplanar(P,n,tol,NL_AVERAGE,&C,&N,&pln)
     

   ACCESS:
   
     P   , input  ,  Point set
     n   , input  ,  Highest index in P
     tol , input  ,  Tolerance to check collinearity
     tfl , input  ,  Flag:
                       NL_MAXIMUM: tol is maximum deviation
                       NL_AVERAGE: tol is average deviation
     C   , output ,  Point on the best fit plane
     N   , output ,  Normal vector of plane
     pln , output ,  Flag:
                       NL_TRUE : points are coplanar
                       NL_FALSE: points are NOT coplanar


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsPtSet1dCoplanar( NL_POINT *P, NL_INDEX n, NL_REAL tol, NL_FLAG tfl, NL_POINT *C, NL_VECTOR *N, NL_FLAG *pln )
{
    NL_FLAG error = NL_NO;

    NL_REAL era, erm;

    /* Check coplanarity */

    *pln = NL_FALSE;

    error = N_PlaneFit3dPts( P, n, C, N, &era, &erm );

    if( error EQ NL_YES )
        NL_OUT;

    switch( tfl )
    {
        case NL_MAXIMUM:
            if( erm LT tol )
                *pln = NL_TRUE;
            break;

        case NL_AVERAGE:
            if( era LT tol )
                *pln = NL_TRUE;
            break;

        default:
            *pln = NL_FALSE;
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_IsPtSet1dCoplanar */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if a 1-D point set is co-spherical or 
     not. It fits a sphere to the points and checks either the maximum 
     or the average  error. The best fitting sphere  along with a flag
     indicator are returned. A typical calling example is:

       NL_FLAG    sph;
       NL_INDEX   n;
       NL_REAL    tol, r;
       NL_POINT   *P, C;
       ...
       (get P and choose tol);
       ...
       N_IsPtSet1dSpherical(P,n,tol,NL_AVERAGE,&C,&r,&sph)
     

   ACCESS:
   
     P   , input  ,  Point set
     n   , input  ,  Highest index in P
     tol , input  ,  Tolerance to check co-sphericity
     tfl , input  ,  Flag:
                       NL_MAXIMUM: tol is maximum deviation
                       NL_AVERAGE: tol is average deviation
     C   , output ,  Center of the sphere
     r   , output ,  Radius of the sphere
     sph , output ,  Flag:
                       NL_TRUE : points are co-spherical
                       NL_FALSE: points are NOT co-spherical


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_IsPtSet1dSpherical( NL_POINT *P, NL_INDEX n, NL_REAL tol, NL_FLAG tfl, NL_POINT *C, NL_REAL *r, NL_FLAG *sph )
{
    NL_FLAG sfl, error = NL_NO;

    NL_REAL cx, cy, cz, era, erm;

    /* Check co-sphericity */

    *sph = NL_FALSE;

    error = N_SphereFit3dPts( P, n, NL_MTOL, NL_AVERAGE, &cx, &cy, &cz, r, &era, &erm, &sfl );

    if( error EQ NL_YES OR sfl EQ NL_FALSE )
        NL_OUT;

    switch( tfl )
    {
        case NL_MAXIMUM:
            if( erm LT tol )
                *sph = NL_TRUE;
            break;

        case NL_AVERAGE:
            if( era LT tol )
                *sph = NL_TRUE;
            break;

        default:
            *sph = NL_FALSE;
    }

    N_PtFromXYZ( cx, cy, cz, C );

    /* Exit */

    EXIT:

    return (error);
} /* end N_IsPtSet1dSpherical */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine deallocates memory that stores a 3-D control  
     point  array. Given a  pointer to  the array, the routine searches 
     for the pointer on  the  memory  stack. It it is found,  memory is 
     deallocated. If not, the routine  does nothing. A typical  calling
     example is:

       NL_CPOINT  **c3d;
       NL_STACKS  S;
       ... 
       N_FreeCPt2dArray(c3d,&S);


   ACCESS:
   
     c3d , input  ,  3-D control point array pointer
     S   , input  ,  c3d's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeCPt3dArray( NL_CPOINT *** c3d, NL_STACKS *S )
{
    NL_C3DNODE *prev, *curr;

    NL_CPOINT ** c2d;

    /* Traverse memory stack to find pointer */

    if( S->c3d NEQ NULL )
    {
        prev = S->c3d;
        curr = S->c3d;

        while( curr NEQ NULL AND curr->ptr NEQ c3d )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->c3d = NULL;
            }
            else /* More than one node */
            {
                S->c3d = S->c3d->next;
            }
        }
        else                /* Not the first node */
        if( curr NEQ NULL ) /* Node found         */
        {
            prev->next = curr->next;
        }

        if( curr NEQ NULL ) /* Release memory     */
        {
            c2d = c3d[0];
            N_Free( curr->ptr );
            curr->ptr = NULL;
            N_Free( curr );
            curr = NULL;
            N_FreeCPt2dArray( c2d, S );
        }
    }
} /* end N_FreeCPt3dArray */

/*******************************************************************//**


   DESCRIPTION:

     This error routine sets the error handle given an  error number and 
     the name of the routine where the error occurred. A typical calling
     example is:

       NL_INTEGER  err;
       NL_STRING   rname;
       ...
       (get err and rname);
       ...
       N_ErrSet(err,rname);


   ACCESS:
   
     err   , input  ,  Error number
     rname , input  ,  Routine name


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ErrSet( NL_INTEGER err, const TCHAR * rname )
{
    NL_ERROR.eno = err;
    NL_ERROR.fna = (NL_STRING)rname;
} /* end N_ErrSet */

/*******************************************************************//**


   DESCRIPTION:

     This error routine clears the error handle;  i.e.  it  resets it to
     the initial "No Error" condition. A typical calling example is:

       N_ErrClear();


   ACCESS:
   
     No arguments


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_ErrClear( NL_VOID )
{
    NL_ERROR.eno = 0;
    NL_ERROR.fna = _T(" ");
} /* end N_ErrClear */

/*******************************************************************//**


   DESCRIPTION:

     This error routine  checks if an integer matrix structure has 
     sufficient memory to store matrix elements. A typical calling
     example is:

       NL_IMATRIX  ima;
       NL_INDEX    nr, mc;
       NL_STRING   rname;
       ...
       (define ima, get nr, mc and rname);
       ...
       N_CheckIntMatStorage(&ima,nr,mc,rname);


   ACCESS:
   
     ima   , input ,  Integer matrix
     nr    , input ,  Expected highest index in rows
     mc    , input ,  Expected highest index in columns
     rname , input ,  Name of routine where check is made


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CheckIntMatStorage( NL_IMATRIX *ima, NL_INDEX nr, NL_INDEX mc, NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    /* Get local notation */

    N_GetMaxIndexIntMatrix( ima, &n, &m );

    /* Check storage */

    if( n LT nr OR m LT mc )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */

    return (error);
} /* end N_CheckIntMatStorage */

/*******************************************************************//**


   DESCRIPTION:

     This  error  routine  checks if a  real matrix  structure has 
     sufficient memory to store matrix elements. A typical calling
     example is:

       NL_RMATRIX  rma;
       NL_INDEX    nr, mc;
       NL_STRING   rname;
       ...
       (define rma, get nr, mc and rname);
       ...
       N_CheckRealMatStorage(&rma,nr,mc,rname);


   ACCESS:
   
     rma   , input ,  Real matrix
     nr    , input ,  Expected highest index in rows
     mc    , input ,  Expected highest index in columns
     rname , input ,  Name of routine where check is made


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CheckRealMatStorage( NL_RMATRIX *rma, NL_INDEX nr, NL_INDEX mc, NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    /* Get local notation */

    N_GetMaxIndexRealMatrix( rma, &n, &m );

    /* Check storage */

    if( n LT nr OR m LT mc )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */

    return (error);
} /* end N_CheckRealMatStorage */

/*******************************************************************//**


   DESCRIPTION:

     This error  routine checks if a  point matrix  structure  has 
     sufficient memory to store matrix elements. A typical calling
     example is:

       NL_PMATRIX  pma;
       NL_INDEX    nr, mc;
       NL_STRING   rname;
       ...
       (define pma, nr, mc and rname);
       ...
       N_CheckPtMatStorage(&pma,nr,mc,rname);


   ACCESS:
   
     pma   , input ,  Point matrix
     nr    , input ,  Expected highest index in rows
     mc    , input ,  Expected highest index in columns
     rname , input ,  Name of routine where check is made


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CheckPtMatStorage( NL_PMATRIX *pma, NL_INDEX nr, NL_INDEX mc, NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    /* Get local notation */

    N_GetMaxIndexPtMatrix( pma, &n, &m );

    /* Check storage */

    if( n LT nr OR m LT mc )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */

    return (error);
} /* end N_CheckPtMatStorage */

/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if a control point matrix structure has 
     sufficient  memory to  store matrix  elements. A  typical calling
     example is:

       NL_CMATRIX  cma;
       NL_INDEX    nr, mc;
       NL_STRING   rname;
       ...
       (define cma, get nr, mc and rname);
       ...
       N_CheckCPtMatStorage(&cma,nr,mc,rname);


   ACCESS:
   
     cma   , input ,  Control point matrix
     nr    , input ,  Expected highest index in rows
     mc    , input ,  Expected highest index in columns
     rname , input ,  Name of routine where check is made


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CheckCPtMatStorage( NL_CMATRIX *cma, NL_INDEX nr, NL_INDEX mc, NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX n, m;

    /* Get local notation */

    N_GetMaxIndexCPtMatrix( cma, &n, &m );

    /* Check storage */

    if( n LT nr OR m LT mc )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */

    return (error);
} /* end N_CheckCPtMatStorage */

/*******************************************************************//**


   DESCRIPTION:

     This error routine returns the type of error. A typical calling 
     example is:

       NL_INTEGER  etp;

       etp = N_ErrGetType();


   ACCESS:
   
     No arguments


   RETURN CODES:

     Returns the error type, e.g. NL_CUR_ERR

   ***********************************************************************/

NL_INTEGER N_ErrGetType( NL_VOID )
{
    return (NL_ERROR.eno);
}
