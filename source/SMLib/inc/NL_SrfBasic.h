// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**************************************************************************/
/* SrfBasic.h: Basic Function Declarations that act on NL_SURFACE objects */
/**************************************************************************/

#ifndef _SRFBASIC_H
#define _SRFBASIC_H

/******************/
/* Error routines */
/******************/

GW_EXPORT NL_FLAG N_SrfCountsAreValid( NL_SURFACE *, NL_STRING );
GW_EXPORT NL_FLAG N_SrfWeightsAreValid( NL_SURFACE *, NL_STRING );
GW_EXPORT NL_FLAG N_SrfIsValid( NL_SURFACE *, const TCHAR * );
GW_EXPORT NL_FLAG N_SrfIsSized( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_STRING );
GW_EXPORT NL_FLAG N_SrfHasEqualCPts( NL_SURFACE *, NL_REAL, NL_INDEX *, NL_INDEX * );

/********************/
/* Utility routines */
/********************/

GW_EXPORT NL_SURFACE *N_AllocSrf( NL_STACKS * );
GW_EXPORT NL_SURFACE *N_AllocSrfAndArrays( NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_SURFACE ** N_AllocArraySrfPtrs( NL_INDEX, NL_STACKS * );
GW_EXPORT NL_SURFACE ** N_AllocArraySrfPtrsInit( NL_INDEX, NL_FLAG, NL_STACKS * );
GW_EXPORT NL_SURFACE *** N_Alloc2dArraySrfPtrs( NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_SURFACE *** N_Alloc2dArraySrfPtrsParameters( NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_BOOLEAN N_SrfAreArraysNULL( NL_SURFACE * );
GW_EXPORT NL_VOID N_SrfInitArrays( NL_SURFACE * );
GW_EXPORT NL_VOID N_SrfSetSizeIndices( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX );
GW_EXPORT NL_FLAG N_AllocSrfArrays( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfSizeArrays( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_STRING, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfCopy( NL_SURFACE *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_VOID N_FreeSrf( NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_VOID N_FreeSrfStruct( NL_SURFACE *, NL_STACKS * );

GW_EXPORT NL_VOID N_SrfFromCNetAndKnotVectors( NL_SURFACE *, NL_CNET *, NL_DEGREE, NL_DEGREE, NL_KNOTVECTOR *, NL_KNOTVECTOR * );
GW_EXPORT NL_FLAG N_SrfFromCPtsAndKnots( NL_SURFACE *, NL_CPOINT **, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_REAL *, 
                                         NL_REAL *, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfFromCPtCoordsAndKnots( NL_SURFACE *, NL_REAL **, NL_REAL **, NL_REAL **, NL_REAL **, NL_INDEX, NL_INDEX, 
                                             NL_DEGREE, NL_DEGREE, NL_REAL *, NL_REAL *, NL_INDEX, NL_INDEX, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSrfFromCoordFuncs( NL_SFUN *, NL_SFUN *, NL_SFUN *, NL_SFUN *, NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSplineSrf( NL_SURFACE *, NL_CPOINT **, NL_INDEX, NL_INDEX, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER, 
                                    NL_PARAMETER, NL_STACKS * );
GW_EXPORT NL_FLAG N_CreateSrfFromNumAndDen( NL_SURFACE *, NL_SFUN *, NL_SURFACE *, NL_STACKS * );

/**********************/
/* Simple data access */
/**********************/

GW_EXPORT NL_VOID N_SrfGetNetAndKnotVectors( NL_SURFACE *, NL_CNET **, NL_DEGREE *, NL_DEGREE *, NL_KNOTVECTOR **, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_SrfGetCPtsDegreesAndKnots( const NL_SURFACE *, NL_INDEX *, NL_INDEX *, NL_CPOINT ***, NL_DEGREE *, NL_DEGREE *, 
                                              NL_INDEX *, NL_INDEX *, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_SrfGetCPts( NL_SURFACE *, NL_INDEX *, NL_INDEX *, NL_CPOINT *** );
GW_EXPORT NL_VOID N_SrfGetArraySizes( NL_SURFACE *, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_INDEX * );
GW_EXPORT NL_VOID N_SrfGetCPtsAndKnots( NL_SURFACE *, NL_CPOINT ***, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_SrfGetCPtsKnotVectorAndKnots( NL_SURFACE *, NL_CPOINT ***, NL_KNOTVECTOR **, NL_KNOTVECTOR **, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_SrfSetCPtsAndKnots( NL_SURFACE *, NL_CPOINT **, NL_REAL *, NL_REAL * );
GW_EXPORT NL_VOID N_SrfGetKnots( NL_SURFACE *, NL_INDEX *, NL_INDEX *, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_SrfGetParameterBounds( NL_SURFACE *, NL_PARAMETER *, NL_PARAMETER *, NL_PARAMETER *, NL_PARAMETER * );
GW_EXPORT NL_VOID N_SrfGetKnotVectors( NL_SURFACE *, NL_KNOTVECTOR **, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_SrfGetDegrees( NL_SURFACE *, NL_DEGREE *, NL_DEGREE * );
GW_EXPORT NL_FLAG N_SrfGetDenominatorFunc( NL_SURFACE *, NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfGetENet( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_ENET *, NL_STACKS * );

GW_EXPORT NL_FLAG N_SrfGetCoordFuncs( NL_SURFACE *, NL_SFUN *, NL_SFUN *, NL_SFUN *, NL_SFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfGetNumAndDen( NL_SURFACE *, NL_SURFACE *, NL_SFUN *, NL_STACKS * );

/*****************/
/* modifications */
/*****************/

GW_EXPORT NL_FLAG N_SrfCompress( NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_VOID N_ClampSrfAtParams( NL_SURFACE *, NL_PARAMETER *, NL_PARAMETER * );
GW_EXPORT NL_FLAG N_SwapUV( NL_SURFACE *, NL_STACKS * );
GW_EXPORT NL_VOID N_SrfRatToNonRat( NL_SURFACE * );
GW_EXPORT NL_VOID N_SrfNonRatToRat( NL_SURFACE * );
GW_EXPORT NL_VOID N_MakeSrfsRatCompatible( NL_SURFACE **, NL_INDEX );
GW_EXPORT NL_VOID N_SrfReparamToInterval( NL_SURFACE *, NL_RECTANGLE, NL_FLAG );
GW_EXPORT NL_VOID N_SrfMakeNonRat( NL_SURFACE * );
GW_EXPORT NL_VOID N_SrfPruneRat( NL_SURFACE * );
GW_EXPORT NL_FLAG N_ExtendPeriodicSurface( NL_SURFACE *, NL_FLAG, NL_FLAG, NL_STACKS * );

/**************/
/* Predicates */
/**************/

GW_EXPORT NL_BOOLEAN N_IsSrfRat( NL_SURFACE * );
GW_EXPORT NL_BOOLEAN N_SrfIsDegen( NL_SURFACE * );
GW_EXPORT NL_BOOLEAN N_SrfIsClosed( NL_SURFACE *, NL_FLAG );
GW_EXPORT NL_BOOLEAN N_1dPtSetIsClosed( NL_VOID *, NL_INDEX, NL_FLAG, NL_REAL );
GW_EXPORT NL_BOOLEAN N_2dPtSetIsClosed( NL_VOID **, NL_INDEX, NL_INDEX, NL_FLAG, NL_FLAG, NL_REAL );
GW_EXPORT NL_BOOLEAN N_SrfsAreCoincident( NL_SURFACE *, NL_SURFACE *, NL_REAL, NL_STACKS * );
GW_EXPORT NL_BOOLEAN N_SrfsAreEqual( NL_SURFACE *, NL_SURFACE *, NL_REAL, NL_REAL );
GW_EXPORT NL_BOOLEAN N_SrfIsSingular( NL_SURFACE *, NL_FLAG );
GW_EXPORT NL_FLAG N_SrfIsFlat( NL_SURFACE *, NL_REAL, NL_FLAG *, NL_FLAG * );
GW_EXPORT NL_BOOLEAN N_SrfAreWeightsEqual( NL_SURFACE * );
GW_EXPORT NL_FLAG N_SrfIsFlatCheap( NL_SURFACE *, NL_REAL, NL_FLAG *, NL_FLAG * );

/***************/
/* Persistence */
/***************/

GW_EXPORT NL_FLAG N_CreateSrfFromDataFile( NL_SURFACE *, TCHAR*, NL_FLAG, NL_STACKS * );
GW_EXPORT NL_FLAG N_WriteSrf( NL_SURFACE *, TCHAR* );
GW_EXPORT NL_FLAG N_WriteSrfArray( NL_SURFACE **, NL_INDEX, TCHAR* );
GW_EXPORT NL_FLAG N_PrintSrfData( NL_SURFACE * );
GW_EXPORT NL_FLAG N_WriteSrfToIgesFile( NL_SURFACE **, NL_INDEX, TCHAR* );
GW_EXPORT NL_FLAG N_SrfReadFromFile( NL_SURFACE *, FILE *, NL_FLAG, NL_STACKS * );
GW_EXPORT NL_FLAG N_ReadSrfArray( NL_SURFACE *** srfs, NL_INDEX *, TCHAR*, NL_FLAG, NL_STACKS * );
GW_EXPORT NL_FLAG N_SrfWriteToFile( NL_SURFACE *, FILE * );

/****************************************************************/
/* A struct that pretends to be an evaluated point on a surface */
/****************************************************************/

GW_EXPORT NL_VOID N_surfJetInit( NL_SURFJET *, NL_SURFACE * );
GW_EXPORT NL_BOOLEAN N_surfJetUnset( NL_SURFJET * );
GW_EXPORT NL_VOID N_surfJetReset( NL_SURFJET * );
GW_EXPORT NL_VOID N_surfJetSetParam( NL_SURFJET *, NL_REAL, NL_REAL );
GW_EXPORT NL_POINT *N_surfJetPos( NL_SURFJET * );
GW_EXPORT NL_VECTOR *N_surfJetDer_u( NL_SURFJET * );
GW_EXPORT NL_VECTOR *N_surfJetDer_v( NL_SURFJET * );
GW_EXPORT NL_VECTOR *N_surfJetDer_uu( NL_SURFJET * );
GW_EXPORT NL_VECTOR *N_surfJetDer_uv( NL_SURFJET * );
GW_EXPORT NL_VECTOR *N_surfJetDer_vv( NL_SURFJET * );
GW_EXPORT NL_BOOLEAN N_surfJetNormal( NL_SURFJET *, NL_VECTOR * );
GW_EXPORT NL_VOID N_surfJetPosCopy( NL_SURFJET *, NL_POINT * );
GW_EXPORT NL_VOID N_surfJetDer_uCopy( NL_SURFJET *, NL_VECTOR * );
GW_EXPORT NL_VOID N_surfJetDer_vCopy( NL_SURFJET *, NL_VECTOR * );
GW_EXPORT NL_VOID N_surfJetDer_uuCopy( NL_SURFJET *, NL_VECTOR * );
GW_EXPORT NL_VOID N_surfJetDer_uvCopy( NL_SURFJET *, NL_VECTOR * );
GW_EXPORT NL_VOID N_surfJetDer_vvCopy( NL_SURFJET *, NL_VECTOR * );
GW_EXPORT NL_BOOLEAN N_surfJetPrinCurvature( NL_SURFJET *, NL_REAL *, NL_VECTOR *, NL_REAL *, NL_VECTOR * );
GW_EXPORT NL_BOOLEAN N_surfJetRelax( NL_SURFJET *, NL_POINT * );

#endif /* _SRFBASIC_H */
