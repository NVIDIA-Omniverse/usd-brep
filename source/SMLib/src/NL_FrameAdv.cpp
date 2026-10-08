// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/********************************************************************************/
/* FrameAdv.c : Advanced Function Definitions for NL_CPOLYGON, NL_CNET, NL_CMESH objects */
/********************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates  memory to store  members of a control
     net structure. Proper error check is performed in case memory 
     allocation fails. A typical calling example is:

       NL_CMESH    *mesh;
       NL_STACKS  S;
       ...
       mesh = N_AllocCNet(&S);


   ACCESS:
   
     S  , input  ,  Memory stack pointer


   RETURN CODES:

     mesh  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_CMESH *N_AllocCMesh( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocCNet");

    NL_CMESH *mesh;

    NL_MESHNODE *med;

    /* Allocate memory for the structure */

    mesh = (NL_CMESH *)N_Malloc( sizeof( NL_CMESH ) );

    if( mesh EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    med = (NL_MESHNODE *)N_Malloc( sizeof( NL_MESHNODE ) );

    if( med EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( mesh );
        mesh = NULL;
        return NULL;
    }

    med->ptr = mesh;
    med->next = S->msh;
    S->msh = med;

    /* Exit */

    return mesh;
}                        /* end S_CMesh Alloc*/ /* end N_AllocateCMESH */

/*******************************************************************//**


   DESCRIPTION:

     This  routine allocates  memory to store  a control  mesh object. 
     Proper error check is performed in case memory allocation fails.
  
       NL_CNET    *net;
       NL_INDEX   n, m;
       NL_STACKS  S;
       ...
       (get m, n, and o);
       ...
       net = N_AllocCMeshAndArrays(m,n,o,&S);


   ACCESS:
   
     m,n,o , input  ,  Highest indexes in control net array 
     S     , input  ,  Memory stack pointer


   RETURN CODES:

     mesh  : Pointer to structure if no error
     NULL  : Memory allocation fails

   ***********************************************************************/

/* NL_CNET  *N_AllocCNetAndArrays */
NL_CMESH *N_AllocCMeshAndArrays( NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_STACKS *S )
{
    NL_CPOINT *** Pw;

    NL_CMESH *mesh;

    /* Allocate memory */

    mesh = N_AllocCMesh( S );

    if( mesh EQ NULL )
        return NULL;

    Pw = N_AllocCPt3dArray( m, n, o, S );

    if( Pw EQ NULL )
        return NULL;

    /* Build the object */

    N_CMeshFromCPts( mesh, Pw, m, n, o );

    /* Exit */

    return mesh;
} /* end N_AllocCMeshAndArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes proper pointer assignments to define
     a control net object  from a set  of control points. Memory for
     the net structure is allocated in the calling routine; only the
     pointer is passed down. A typical calling example is:

       NL_CNET    net;
       NL_CPOINT  **Pw;
       NL_INDEX   n, m;
       ...
       (allocate memory for Pw);
       ...
       N_CNetFromCPts(&net,Pw,n,m);


   ACCESS:
   
     net , in/out ,  Control net
     Pw  , input  ,  Control points
     n,m , input  ,  Highest indexes in Pw


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_CMeshFromCPts( NL_CMESH *mesh, NL_CPOINT *** Pw, NL_INDEX m, NL_INDEX n, NL_INDEX o )
{
    mesh->m = m;
    mesh->n = n;
    mesh->o = o;
    mesh->Pw = Pw;
} /* end N_CMeshFromCPts */

/*******************************************************************//**


   DESCRIPTION:

     This routine generates a control mesh object given the wx, wy, wz 
     and w components of  the control points. It allocates  memory to  
     store the control points, and makes proper pointer  assignments. 
     The  volume  is  restricted  to  non-rational  by  setting  the 
     w coordinate values to NL_NOW. All other memory allocation is done 
     in the calling routine. A typical calling example is:

       NL_CMESH   mesh;
       NL_REAL    **wx, **wy, **wz, **w;
       NL_INDEX   m, n, o;
       NL_STACKS  S;
       ...
       (allocate memory for wx, wy, wz and w);
       ...
       N_CNetFromCPtCoords(&mesh,wx,wy,wz,w,m,n,o,&S); 


   ACCESS:
   
     mesh       , in/out ,  Control mesh
     wx,wy,wz,w , input  ,  wx, wy, wz and w components
     m,n,o      , input  ,  Highest indexes in <wx,wy,wz,w>
     S          , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_CMeshFromCPtCoords( NL_CMESH *mesh, NL_REAL *** wx, NL_REAL *** wy, NL_REAL *** wz, NL_REAL *** w, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_STACKS *S )
{
    NL_INDEX i, j, k;

    NL_CPOINT *** Pw;

    /* Allocate memory for the point array */

    Pw = N_AllocCPt3dArray( m, n, o, S );

    if( Pw EQ NULL )
        return (1);

    /* Fill in point array */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_CPtFromWxWyWz( wx[i][j][k], wy[i][j][k], wz[i][j][k], w[i][j][k], &Pw[i][j][k] );
            }
        }
    }

    /* Make pointer assignments */

    N_CMeshFromCPts( mesh, Pw, m, n, o );

    /* Exit */

    return (0);
} /* end N_CMeshFromCPtCoords */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine breaks a control mesh object down to its 
     components. A typical calling example is:

       NL_CMESH   mesh;
       NL_INDEX   m,n,o;
       NL_CPOINT  **Pw;
       ...
       N_CMeshGetCPts(&mesh,&m,&n,&o,&Pw);


   ACCESS:
   
     mesh  , input  ,  Control mesh
     m,n,o , output ,  Highest indexes in Pw
     Pw    , output ,  Control points


   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_CNetGetCPts */
NL_VOID N_CMeshGetCPts( NL_CMESH *mesh, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_CPOINT **** Pw )
{
    *m = mesh->m;
    *n = mesh->n;
    *o = mesh->o;
    *Pw = mesh->Pw;
} /* end N_CMeshGetCPts */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine deallocates  memory that stores members of a 
     control mesh  structure. Given a  control mesh pointer, the  routine
     searches for the  pointer  on the  memory stack.  It it is  found,
     memory is deallocated. If not, the routine does nothing. A typical
     calling example is:

       NL_CMESH    *mesh;
       NL_STACKS  S;
       ...
       N_FreeCNet(mesh,&S);


   ACCESS:
   
     mesh , input  ,  Control mesh pointer 
     S   , input  ,  mesh's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeCMesh( NL_CMESH *mesh, NL_STACKS *S )
{
    NL_MESHNODE *prev, *curr;

    /* Traverse memory stack to find pointer */

    if( S->msh NEQ NULL )
    {
        prev = S->msh;
        curr = S->msh;

        while( curr NEQ NULL AND curr->ptr NEQ mesh )
        {
            prev = curr;
            curr = curr->next;
        }

        if( prev EQ curr )           /* First node         */
        {
            if( curr->next EQ NULL ) /* One node only      */
            {
                S->msh = NULL;
            }
            else /* More than one node */
            {
                S->msh = S->msh->next;
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
} /* end N_FreeCMesh */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine makes a point mesh object given the vertices 
     of the mesh. A typical calling example is:

       NL_EMESH  msh;
       NL_INDEX  m, n, o;
       NL_POINT  ***P;
       ...
       (get m,n,o and array P);
       ...
       N_ENetFromPts(&msh,n,m,P);


   ACCESS:
   
     msh   , output ,  Point mesh object
     m,n,o , input  ,  Highest indexes in P
     P     , input  ,  Vertices of point mesh



   RETURN CODES:

     None

   ***********************************************************************/

/* NL_VOID  N_ENetFromPts */
NL_VOID N_EMeshFromPts( NL_EMESH *msh, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_POINT *** P )
{
    msh->m = m;
    msh->n = n;
    msh->o = o;
    msh->P = P;
} /* end N_EMeshFromPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine breaks a mesh object down to its components. 
     A typical calling example is:

       NL_EMESH   msh;
       NL_INDEX   m, n, o;
       NL_POINT   ***P;
       ...
       (get msh);
       ...
       N_EMeshGetPts(&msh,&m,&n,&o,&P);


   ACCESS:
   
     msh    , input  ,  Mesh
     m,n,o  , output ,  Highest indexes in P
     P      , output ,  Vertex array of mesh


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_EMeshGetPts( NL_EMESH *msh, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_POINT **** P )
{
    *m = msh->m;
    *n = msh->n;
    *o = msh->o;
    *P = msh->P;
} /* end N_EMeshGetPts */

/*******************************************************************//**


   DESCRIPTION:

     This geometry routine computes the bounding box of a point mesh. A
     typical calling example is:

       NL_EMESH      msh;
       NL_MINMAXBOX  box;
       ...
       (get msh);
       ...
       N_EMeshGetBBox(&msh,&box);


   ACCESS:
   
     msh , input  ,  Point mesh
     box , output ,  Bounding box


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_EMeshGetBBox( NL_EMESH *msh, NL_MINMAXBOX *box )
{
    NL_INDEX i, j, k, m, n, o;

    NL_POINT *** P;

    NL_REAL x, y, z, xl, xr, yb, yt, zn, zf;

    /* Get local notation */

    N_EMeshGetPts( msh, &m, &n, &o, &P );

    /* Compute min-max box */

    N_PtToXYZ( P[0][0][0], &x, &y, &z );

    xl = xr = x;
    yb = yt = y;
    zn = zf = z;

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_PtToXYZ( P[i][j][k], &x, &y, &z );

                if( x LT xl )
                    xl = x;

                if( x GT xr )
                    xr = x;

                if( y LT yb )
                    yb = y;

                if( y GT yt )
                    yt = y;

                if( z LT zn )
                    zn = z;

                if( z GT zf )
                    zf = z;
            }
        }
    }

    N_BBoxDefine( box, xl, xr, yb, yt, zn, zf );
} /* end N_EMeshGetBBox */
#endif // NLIB_UNUSED
