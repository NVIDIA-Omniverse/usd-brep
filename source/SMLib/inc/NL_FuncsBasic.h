// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**************************************************************************/
/* FuncsBasic.h: Basic Function Declarations that act on NL_CFUN objects  */
/**************************************************************************/

#ifndef _FUNCSBASIC_H

#define _FUNCSBASIC_H

/****************************************/
/* NL_CFUN Error routines */
/****************************************/

GW_EXPORT NL_FLAG N_CrvIsFuncSized( NL_CFUN *, NL_INDEX, NL_INDEX, NL_STRING );

/***************************/
/* NL_CVALUE Utility routines */
/***************************/

GW_EXPORT NL_CVALUE *N_AllocCValue( NL_STACKS * );
GW_EXPORT NL_CVALUE *N_AllocCValueAndArray( NL_INDEX, NL_STACKS * );
GW_EXPORT NL_VOID N_FreeCValue( NL_CVALUE *, NL_STACKS * );
GW_EXPORT NL_VOID N_CValueFromArray( NL_CVALUE *, NL_REAL *, NL_INDEX );

/***************************/
/* NL_CFUN Utility routines   */
/***************************/

GW_EXPORT NL_CFUN ** N_AllocCrvFuncArray( NL_INDEX, NL_DEGREE, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_CFUN *N_AllocCrvFunc( NL_INDEX, NL_DEGREE, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_CFUN *N_AllocCrvFuncStack( NL_STACKS * );
GW_EXPORT NL_VOID N_FreeCrvFunc( NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_VOID N_CFuncFromKnotVector( NL_CFUN *, NL_CVALUE *, NL_DEGREE, NL_KNOTVECTOR * );
GW_EXPORT NL_FLAG N_CFuncFromKnots( NL_CFUN *, NL_REAL *, NL_INDEX, NL_DEGREE, NL_REAL *, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_BOOLEAN N_CrvAreFuncArraysNULL( NL_CFUN * );
GW_EXPORT NL_FLAG N_AllocCFuncArrays( NL_CFUN *, NL_INDEX, NL_DEGREE, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_VOID N_CFuncInitArrays( NL_CFUN * );
GW_EXPORT NL_VOID N_CFuncSetSizeIndices( NL_CFUN *, NL_INDEX, NL_DEGREE, NL_INDEX );
GW_EXPORT NL_FLAG N_CFuncSizeArrays( NL_CFUN *, NL_INDEX, NL_DEGREE, NL_INDEX, NL_STRING, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncCompact( NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvFuncCopy( NL_CFUN *, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_VOID N_CrvFuncPrint( NL_CFUN * );

/* simple NL_CFUN data access */

GW_EXPORT NL_VOID N_CrvFuncReparam( NL_CFUN *, NL_INTERVAL );
GW_EXPORT NL_VOID N_CrvFuncCntrlVal( NL_CFUN *, NL_INDEX *, NL_REAL ** );
GW_EXPORT NL_VOID N_CFuncGetArrayAndKnotVector( NL_CFUN *, NL_CVALUE **, NL_DEGREE *, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_CrvFuncCntrlValKnotVector( NL_CFUN *, NL_REAL **, NL_KNOTVECTOR **, NL_REAL ** );
GW_EXPORT NL_VOID N_CFuncGetArraySizes( NL_CFUN *, NL_INDEX *, NL_INDEX * );
GW_EXPORT NL_VOID N_CFuncGetKnots( NL_CFUN *, NL_INDEX *, NL_REAL ** );
GW_EXPORT NL_VOID N_CFuncGetKnotVector( NL_CFUN *, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_CFuncGetDegree( NL_CFUN *, NL_DEGREE * );
GW_EXPORT NL_VOID N_CrvFuncCntrlValKnots( NL_CFUN *, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_CFuncGetData( NL_CFUN *, NL_INDEX *, NL_REAL **, NL_DEGREE *, NL_INDEX *, NL_REAL ** );
GW_EXPORT NL_VOID N_CrvFuncSetPtrs( NL_CFUN *, NL_REAL *, NL_REAL * );
/*      GW_EXPORT NL_VOID     N_CFuncGetKnots( NL_CFUN *, NL_INDEX *, NL_REAL ** );    */

/* Basic NL_CFUN functions */

GW_EXPORT NL_FLAG N_CFuncEval( NL_CFUN *, NL_PARAMETER, NL_FLAG, NL_REAL * );
GW_EXPORT NL_FLAG N_CFuncDerivs( NL_CFUN *, NL_PARAMETER, NL_FLAG, NL_INDEX, NL_REAL * );

/***********************/
/* NL_SFUN Error routines */
/***********************/

GW_EXPORT NL_FLAG N_SrfIsFuncSized( NL_SFUN *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_STRING );

/***************************/
/* NL_SVALUE Utility routines */
/***************************/

GW_EXPORT NL_SVALUE *N_AllocSValue( NL_STACKS * );
GW_EXPORT NL_SVALUE *N_AllocSValueAndArray( NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_VOID N_FreeSValue( NL_SVALUE *, NL_STACKS * );
GW_EXPORT NL_VOID N_SValueFromArray( NL_SVALUE *, NL_REAL **, NL_INDEX, NL_INDEX );

/***************************/
/* NL_SFUN Utility routines   */
/***************************/

GW_EXPORT NL_SFUN *** N_Alloc2dArraySrfFunc( NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_SFUN *N_AllocSrfFunc( NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_SFUN *N_AllocSrfFuncData( NL_STACKS * );
GW_EXPORT NL_VOID N_FreeSrfFuncData( NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_VOID N_SFuncFromKnotVectors( NL_SFUN *, NL_SVALUE *, NL_DEGREE, NL_DEGREE, NL_KNOTVECTOR *, NL_KNOTVECTOR * );
GW_EXPORT NL_FLAG N_SFuncFromKnots( NL_SFUN *, NL_REAL **, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_REAL *, NL_REAL *, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_BOOLEAN N_SrfAreFuncArraysNULL( NL_SFUN * );
GW_EXPORT NL_FLAG N_AllocSFuncArrays( NL_SFUN *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_VOID N_SFuncInitArrays( NL_SFUN * );
GW_EXPORT NL_VOID N_SFuncSetSizeIndices( NL_SFUN *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX );
GW_EXPORT NL_FLAG N_SFuncSizeArrays( NL_SFUN *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_STRING, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncCompact( NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFuncCopy( NL_SFUN *, NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_VOID N_SrfFuncPrint( NL_SFUN * );

/* simple NL_SFUN data access */

GW_EXPORT NL_VOID N_SFuncGetComponents( NL_SFUN *, NL_INDEX *, NL_INDEX *, NL_REAL ***, NL_DEGREE *, NL_DEGREE *, NL_INDEX *, NL_INDEX *, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_SFuncGetKnots( NL_SFUN *, NL_REAL ***, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_SrfFuncCntrlVal( NL_SFUN *, NL_INDEX *, NL_INDEX *, NL_REAL *** );
GW_EXPORT NL_VOID N_SrfFuncGetKnots( NL_SFUN *, NL_INDEX *, NL_INDEX *, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_SrfFuncScale( NL_SFUN *, NL_RECTANGLE, NL_FLAG );
GW_EXPORT NL_VOID N_SFuncGetKnotVectors( NL_SFUN *, NL_KNOTVECTOR **, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_SFuncGetDegrees( NL_SFUN *, NL_DEGREE *, NL_DEGREE * );
GW_EXPORT NL_VOID N_SFuncGetArraySizes( NL_SFUN *, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_INDEX * );
GW_EXPORT NL_VOID N_SFuncGetArrayAndKnotVectors( NL_SFUN *, NL_SVALUE **, NL_DEGREE *, NL_DEGREE *, NL_KNOTVECTOR **, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_SrfFuncSetPtrs( NL_SFUN *, NL_REAL **, NL_REAL *, NL_REAL * );

/* basic NL_SFUN functions */

GW_EXPORT NL_FLAG N_SrfFuncEvalPt( NL_SFUN *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_REAL * );
GW_EXPORT NL_FLAG N_SFuncDerivs( NL_SFUN *, NL_PARAMETER, NL_PARAMETER, NL_FLAG, NL_FLAG, NL_INDEX, NL_INDEX, NL_REAL ** );

#endif /* _FUNCSBASIC_H */
