// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvBasic.h: Basic Function Declarations that act on NL_CURVE objects    */
/**********************************************************************/

#ifndef _CRVBASIC_H
#define _CRVBASIC_H

/******************/
/* Error routines */
/******************/

GW_EXPORT NL_FLAG N_CrvCountsAreValid( NL_CURVE *, NL_STRING );
GW_EXPORT NL_FLAG N_CrvWeightsAreValid( NL_CURVE *, NL_STRING );
/* linux warning GW_EXPORT NL_FLAG N_CrvIsValid( NL_CURVE *, const  NL_STRING ); */
GW_EXPORT NL_FLAG N_CrvIsValid( NL_CURVE *, const  TCHAR * );
GW_EXPORT NL_FLAG N_CrvIsNotReversed( NL_CURVE *, NL_FLAG );
GW_EXPORT NL_FLAG N_CrvIsSized( NL_CURVE *, NL_INDEX, NL_INDEX, const NL_STRING );
GW_EXPORT NL_FLAG N_CrvHasEqualCPts( NL_CURVE *, NL_REAL, NL_INDEX * );
GW_EXPORT NL_FLAG N_CrvReplaceEqualCPts( NL_CURVE *, NL_REAL );

/********************/
/* Utility routines */
/********************/

GW_EXPORT NL_CURVE *N_AllocCrv( NL_STACKS * );
GW_EXPORT NL_CURVE *N_AllocCrvAndArrays( NL_INDEX, NL_DEGREE, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_CURVE ** N_Alloc1dArrayCrvs( NL_INDEX, NL_DEGREE, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_CURVE ** N_AllocArrayCrvPtrs( NL_INDEX, NL_STACKS * );
GW_EXPORT NL_CURVE ** N_AllocArrayCrvPtrsAndData( NL_INDEX, NL_FLAG, NL_STACKS * );
GW_EXPORT NL_CURVE *** N_AllocArrayRealCrvPtrs( NL_INDEX, NL_STACKS * );
GW_EXPORT NL_CURVE **** N_AllocArrayTripleCrvPtrs( NL_INDEX, NL_STACKS * );
GW_EXPORT NL_CURVE *** N_Alloc2dArrayCrvPtrs( NL_INDEX, NL_INDEX, NL_STACKS *S );

GW_EXPORT NL_BOOLEAN N_CrvAreArraysNULL( NL_CURVE * );
GW_EXPORT NL_VOID    N_CrvInitArrays( NL_CURVE * );
GW_EXPORT NL_VOID    N_CrvSetSizeIndices( NL_CURVE *, NL_INDEX, NL_DEGREE, NL_INDEX );
GW_EXPORT NL_FLAG    N_AllocCrvArrays( NL_CURVE *, NL_INDEX, NL_DEGREE, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_FLAG    N_CrvSizeArrays( NL_CURVE *, NL_INDEX, NL_DEGREE, NL_INDEX, NL_STRING, NL_STACKS * );
GW_EXPORT NL_FLAG    N_CrvCopy( NL_CURVE *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_VOID    N_FreeCrv( NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_VOID    N_FreeCrvStruct( NL_CURVE *, NL_STACKS * );
                     
GW_EXPORT NL_VOID    N_CrvFromCPolygonAndKnotVector( NL_CURVE *, NL_CPOLYGON *, NL_DEGREE, NL_KNOTVECTOR * );
GW_EXPORT NL_FLAG    N_CrvFromCPtsAndKnots( NL_CURVE *, NL_CPOINT *, NL_INDEX, NL_DEGREE, NL_REAL *, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_FLAG    N_CrvFromCPtCoordsAndKnots( NL_CURVE *, NL_REAL *, NL_REAL *, NL_REAL *, NL_REAL *, NL_INDEX, NL_DEGREE, NL_REAL *, NL_INDEX, NL_STACKS * );
                     
GW_EXPORT NL_FLAG    N_CreateCrvFromPowerBasis( NL_CURVE *, NL_CPOINT *, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_STACKS * );
GW_EXPORT NL_FLAG    N_CreateCrvFromNumAndDenom( NL_CURVE *, NL_CFUN *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG    N_CreateCrvFromCoordFuncs( NL_CFUN *, NL_CFUN *, NL_CFUN *, NL_CFUN *, NL_CURVE *, NL_STACKS * );

/**********************/
/* Simple data access */
/**********************/

GW_EXPORT NL_VOID N_CrvDetachPolygonKnot( NL_CURVE *, NL_CPOLYGON **, NL_DEGREE *, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_CrvGetConicData( NL_FLAG, NL_REAL, NL_REAL *, NL_INDEX *, NL_DEGREE *, NL_INDEX * );
GW_EXPORT NL_VOID N_CrvGetCPtsDegreeAndKnots( NL_CURVE *, NL_INDEX *, NL_CPOINT **, NL_DEGREE *, NL_INDEX *, NL_REAL ** );
GW_EXPORT NL_VOID N_CrvGetArraySizes( NL_CURVE *, NL_INDEX *, NL_INDEX * );
GW_EXPORT NL_VOID N_CrvGetCPts( NL_CURVE *, NL_INDEX *, NL_CPOINT ** );
GW_EXPORT NL_VOID N_CrvGetCPtsAndKnots( NL_CURVE *, NL_CPOINT **, NL_REAL ** );
GW_EXPORT NL_VOID N_CrvGetCPtsKnotVectorAndKnots( NL_CURVE *, NL_CPOINT **, NL_KNOTVECTOR **, NL_REAL ** );
GW_EXPORT NL_VOID N_CrvSetCPtsAndKnots( NL_CURVE *, NL_CPOINT *, NL_REAL * );
GW_EXPORT NL_VOID N_CrvGetKnots( NL_CURVE *, NL_INDEX *, NL_REAL ** );
GW_EXPORT NL_VOID N_CrvGetParamBounds( NL_CURVE const*, NL_PARAMETER *, NL_PARAMETER * );
GW_EXPORT NL_VOID N_CrvGetKnotVector( NL_CURVE *, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_CrvGetDegree( NL_CURVE *, NL_DEGREE * );
GW_EXPORT NL_FLAG N_CrvGetEPolygon( NL_CURVE *, NL_INDEX, NL_INDEX, NL_EPOLYGON *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvGetNumAndDenom( NL_CURVE *, NL_CURVE *, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvGetCoordFuncs( NL_CURVE *, NL_CFUN *, NL_CFUN *, NL_CFUN *, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvGetDenomCrvFunc( NL_CURVE *, NL_CFUN *, NL_STACKS * );

/*****************/
/* modifications */
/*****************/

GW_EXPORT NL_VOID N_CrvClampKnot( NL_CURVE *, NL_PARAMETER * );
GW_EXPORT NL_FLAG N_CrvCompress( NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvExpand( NL_CURVE *, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_VOID N_Crv3dTo2d( NL_CURVE * );
GW_EXPORT NL_VOID N_Crv2dTo3d( NL_CURVE * );
GW_EXPORT NL_VOID N_CrvRatToNonRat( NL_CURVE * );
GW_EXPORT NL_VOID N_CrvNonRatToRat( NL_CURVE * );
GW_EXPORT NL_VOID N_CrvsMakeRatCompatible( NL_CURVE **, NL_INDEX );
GW_EXPORT NL_VOID N_CrvReparamToInterval( NL_CURVE *, NL_INTERVAL );
GW_EXPORT NL_VOID N_CrvMakeNonRat( NL_CURVE * );
GW_EXPORT NL_VOID N_CrvPruneRat( NL_CURVE *, NL_FLAG );

/**************/
/* Predicates */
/**************/

GW_EXPORT NL_BOOLEAN N_IsCrvRat( NL_CURVE * );
GW_EXPORT NL_BOOLEAN N_CrvIs3d( NL_CURVE * );
GW_EXPORT NL_BOOLEAN N_CrvIsClosed( NL_CURVE * );
GW_EXPORT NL_BOOLEAN N_CrvIsDegen( NL_CURVE * );
GW_EXPORT NL_BOOLEAN N_CrvIsLine( NL_CURVE *, NL_REAL );
GW_EXPORT NL_BOOLEAN N_CrvsAreCoincident( NL_CURVE *, NL_CURVE *, NL_REAL, NL_STACKS * );
GW_EXPORT NL_BOOLEAN N_CrvsAreEqual( NL_CURVE *, NL_CURVE *, NL_REAL, NL_REAL);
GW_EXPORT NL_BOOLEAN N_CrvsAreCombatible( NL_CURVE **, NL_INDEX );
GW_EXPORT NL_BOOLEAN N_CrvAreWeightsEqual( NL_CURVE * );
GW_EXPORT NL_BOOLEAN N_Crv4dIsDegen( NL_CURVE * );
GW_EXPORT NL_BOOLEAN N_CrvIsInZ0Plane( NL_CURVE * );
GW_EXPORT NL_FLAG    N_CrvIsPlanar( NL_CURVE *, NL_REAL, NL_POINT *, NL_VECTOR *, NL_FLAG *, NL_STACKS * );

/***************/
/* Persistence */
/***************/

GW_EXPORT NL_FLAG N_CrvWriteToFile( NL_CURVE *, TCHAR* );
GW_EXPORT NL_FLAG N_CrvArrayWriteToFile( NL_CURVE **, NL_INDEX, TCHAR* );
GW_EXPORT NL_FLAG N_CrvPrint( NL_CURVE * );
GW_EXPORT NL_FLAG N_CrvReadFromFile( NL_CURVE *, const TCHAR*, NL_FLAG, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvArrayReadFromFile( NL_CURVE ***, NL_INDEX *, const TCHAR*, NL_FLAG, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvReadFromFilePtr( NL_CURVE *, FILE *, NL_FLAG, NL_STACKS * );
GW_EXPORT NL_FLAG N_CrvWriteToFilePtr( NL_CURVE *, FILE * );
GW_EXPORT NL_FLAG N_CrvWriteIgesCPolygon( NL_CURVE **, NL_INDEX, NL_REAL, NL_FLAG, NL_INDEX *, NL_FLAG, NL_INDEX, TCHAR* );
GW_EXPORT NL_FLAG N_CrvWriteIges( NL_CURVE **, NL_INDEX, NL_REAL, TCHAR* );

/****************************************************************/
/* A struct that pretends to be an evaluated point on a curve   */
/****************************************************************/

GW_EXPORT NL_VOID    N_crvJetInit( NL_CURVEJET *, NL_CURVE * );
GW_EXPORT NL_BOOLEAN N_crvJetUnset( NL_CURVEJET * );
GW_EXPORT NL_VOID    N_crvJetReset( NL_CURVEJET * );
GW_EXPORT NL_VOID    N_crvJetSetParam( NL_CURVEJET *, NL_REAL );
GW_EXPORT NL_POINT  *N_crvJetPos( NL_CURVEJET * );
GW_EXPORT NL_VECTOR *N_crvJetDer1( NL_CURVEJET * );
GW_EXPORT NL_VECTOR *N_crvJetDer2( NL_CURVEJET * );
GW_EXPORT NL_VOID    N_crvJetPosCopy( NL_CURVEJET *, NL_POINT * );
GW_EXPORT NL_VOID    N_crvJetDer1Copy( NL_CURVEJET *, NL_VECTOR * );
GW_EXPORT NL_VOID    N_crvJetDer2Copy( NL_CURVEJET *, NL_VECTOR * );
GW_EXPORT NL_BOOLEAN N_crvJetCurvature( NL_CURVEJET *, NL_VECTOR * );
GW_EXPORT NL_BOOLEAN N_crvJetRelax( NL_CURVEJET *, NL_POINT *, NL_GCPTEMP *, NL_STACKS * );

#endif /* _CRVBASIC_H */
