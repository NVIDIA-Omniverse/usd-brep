// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**************************************************************************/
/* FuncsAdv.h: Advanced Function Declarations that act on NL_CFUN objects */
/**************************************************************************/

#ifndef _FUNCSADV_H

#define _FUNCSADV_H

/***************************/
/* Advanced NL_CFUN functions */
/***************************/

GW_EXPORT NL_FLAG N_CrvFuncEvalRatBasis( NL_CFUN *, NL_INDEX, NL_PARAMETER, NL_FLAG, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvFuncEvalDerivsAtKnot( NL_CFUN *, NL_INDEX, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_REAL * );
GW_EXPORT NL_FLAG N_CrvFuncEvalInvertPt( NL_CFUN *, NL_REAL, NL_PARAMETER, NL_REAL, NL_INTEGER, NL_PARAMETER * );
GW_EXPORT NL_FLAG N_MapKnotsBetweenCrvFuncAndKnotVector( NL_CFUN *, NL_KNOTVECTOR *, NL_DEGREE, NL_KNOTVECTOR *, NL_KNOTVECTOR * );

/**********************************/
/* Advanced NL_CFUN and NL_CVALUE Tools */
/**********************************/

GW_EXPORT NL_FLAG N_CrvFuncRefine( NL_CFUN *, NL_KNOTVECTOR *, NL_CFUN *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncDegreeElevate( NL_CFUN *, NL_INDEX, NL_CFUN *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncInsertKnot( NL_CFUN *, NL_PARAMETER, NL_INDEX, NL_CFUN *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncDecompose( NL_CFUN *, NL_CFUN ***, NL_INDEX *, NL_STACKS * );

/***********************************************/
/* Advanced NL_CFUN and NL_CVALUE Symbolic operators */
/***********************************************/

GW_EXPORT NL_FLAG N_CrvFuncMultiplyCrvFunc( NL_CFUN *, NL_CFUN *, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncMultiplyCrv( NL_CFUN *, NL_CURVE *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncSumDiffCrvFunc( NL_CFUN *, NL_CFUN *, NL_FLAG, NL_FLAG, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncDeriv( NL_CFUN *, NL_INDEX, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncDerivKnot( NL_CFUN *, NL_INDEX, NL_FLAG, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncHigherDerivKnot( NL_CFUN *, NL_INDEX, NL_FLAG, NL_INDEX, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncMultiplyCrv4d( NL_CFUN *, NL_CURVE *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_VOID N_CrvFuncMultiplyConstant( NL_REAL, NL_CFUN * );

/***************************/
/* advanced NL_SFUN functions */
/***************************/

GW_EXPORT NL_FLAG N_SrfFuncEvalRatBasis( NL_SFUN *, NL_INDEX, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_REAL * );
GW_EXPORT NL_FLAG N_SrfFuncDerivFuncAtKnot( NL_SFUN *, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_FLAG, NL_FLAG, NL_REAL * );
GW_EXPORT NL_FLAG N_SFuncEvalGrid( NL_SFUN *, NL_PARAMETER *, NL_PARAMETER *, NL_INDEX, NL_INDEX, NL_FLAG, NL_FLAG, NL_REAL ** );

/***************************/
/* advanced NL_SFUN tools     */
/***************************/

GW_EXPORT NL_FLAG N_SrfFuncRefine( NL_SFUN *, NL_KNOTVECTOR *, NL_FLAG, NL_SFUN *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncRemoveKnot( NL_SFUN *, NL_PARAMETER, NL_INDEX, NL_FLAG, NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncDegreeElevate( NL_SFUN *, NL_INDEX, NL_FLAG, NL_SFUN *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncInsertKnot( NL_SFUN *, NL_PARAMETER, NL_INDEX, NL_FLAG, NL_SFUN *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncDecompose( NL_SFUN *, NL_SFUN ****, NL_INDEX *, NL_INDEX *, NL_STACKS * );

/************************************/
/* advanced NL_SFUN Symbolic operators */
/************************************/

GW_EXPORT NL_VOID N_SrfFuncMultiplyConstant( NL_REAL, NL_SFUN * );
GW_EXPORT NL_FLAG N_SrfFuncMultiplySrfFunc( NL_SFUN *, NL_SFUN *, NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncMultiplySrf( NL_SFUN *, NL_SURFACE *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncSumDiffSrfFunc( NL_SFUN *, NL_SFUN *, NL_FLAG, NL_FLAG, NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncDerivFunc( NL_SFUN *, NL_INDEX, NL_INDEX, NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncFuncDerivFuncAtKnot( NL_SFUN *, NL_INDEX, NL_FLAG, NL_FLAG, NL_SFUN *, NL_STACKS * );

/***********************/
/* NL_VFUN Error routines */
/***********************/

GW_EXPORT NL_FLAG N_VolumeIsFuncSized( NL_VFUN *vfn, NL_INDEX mf, NL_INDEX nf, NL_INDEX of, NL_INDEX rk, 
                                      NL_INDEX sk, NL_INDEX tk, NL_STRING rname );

/***************************/
/* NL_VVALUE Utility routines */
/***************************/

GW_EXPORT NL_VVALUE *N_AllocVValue( NL_STACKS * );
GW_EXPORT NL_VVALUE *N_AllocVValueAndArray( NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_STACKS *S );
GW_EXPORT NL_VOID N_FreeVValue( NL_VVALUE *vvl, NL_STACKS *S );
GW_EXPORT NL_VOID N_VValueFromArray( NL_VVALUE *, NL_REAL ***, NL_INDEX, NL_INDEX, NL_INDEX );

/***************************/
/* NL_VFUN Utility routines   */
/***************************/

GW_EXPORT NL_VOID N_VFuncFromKnotVectors( NL_VFUN *vfn, NL_VVALUE *vvl, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, 
                                         NL_KNOTVECTOR *knu, NL_KNOTVECTOR *knv, NL_KNOTVECTOR *knw );

GW_EXPORT NL_FLAG N_VFuncFromKnots( NL_VFUN *vfn, NL_REAL *** fuvw, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, 
                                    NL_DEGREE r, NL_REAL *U, NL_REAL *V, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STACKS *S );

GW_EXPORT NL_FLAG N_VFuncSizeArrays( NL_VFUN *vfn, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, 
                                    NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STRING rname, NL_STACKS *S );

/* simple NL_VFUN data access */

GW_EXPORT NL_VOID N_VFuncGetArraySizes( NL_VFUN *vfn, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_INDEX *ir, 
                                       NL_INDEX *is, NL_INDEX *it );
GW_EXPORT NL_VOID N_VFuncSetSizeIndices( NL_VFUN *vfn, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, 
                                        NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it );
GW_EXPORT NL_VOID N_VFuncGetComponents( NL_VFUN *vfn, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_REAL **** fuvw, 
                                       NL_DEGREE *p, NL_DEGREE *q, NL_DEGREE *r, NL_INDEX *ir, NL_INDEX *is, 
                                       NL_INDEX *it, NL_REAL ** U, NL_REAL ** V, NL_REAL ** W );
GW_EXPORT NL_VOID N_VFuncGetKnots( NL_VFUN *vfn, NL_REAL **** fuvw, NL_REAL ** U, NL_REAL ** V, NL_REAL ** W );
GW_EXPORT NL_VOID N_VFuncGetDegrees( NL_VFUN *vfn, NL_DEGREE *p, NL_DEGREE *q, NL_DEGREE *r );

GW_EXPORT NL_VOID N_VFuncInitArrays( NL_VFUN *vfn );
GW_EXPORT NL_BOOLEAN N_VFuncAreArraysNULL( NL_VFUN *vfn );
GW_EXPORT NL_VOID N_VFuncGetKnotVectors( NL_VFUN *vfn, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv, NL_KNOTVECTOR ** knw );
GW_EXPORT NL_VOID N_VFuncGetArrayAndKnotVectors( NL_VFUN *vfn, NL_VVALUE ** vvl, NL_DEGREE *p, NL_DEGREE *q, 
                                                NL_DEGREE *r, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv, NL_KNOTVECTOR ** knw );
GW_EXPORT NL_FLAG N_AllocVFuncArrays( NL_VFUN *vfn, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, 
                                     NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STACKS *S );

/* advanced NL_VFUN functions */

GW_EXPORT NL_FLAG N_VFuncEval( NL_VFUN *vfn, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, 
                              NL_FLAG vfl, NL_FLAG wfl, NL_REAL *F );
GW_EXPORT NL_FLAG N_VFuncDerivs( NL_VFUN *vfn, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, 
                                NL_FLAG vfl, NL_FLAG wfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL *** FD );

#endif /* _FUNCSADV_H */
