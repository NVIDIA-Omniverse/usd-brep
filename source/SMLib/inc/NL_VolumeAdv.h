// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/***********************************************************************/
/* VolumeAdv.h: Functions Declarations that act on NL_VOLUME objects   */
/***********************************************************************/

#ifndef _VOLUMEADV_H
#define _VOLUMEADV_H

/******************/
/* Error routines */
/******************/

GW_EXPORT NL_FLAG N_VolumeAreCountsValid( NL_VOLUME *vol, NL_STRING rname );
GW_EXPORT NL_FLAG N_VolumeAreWeightsValid( NL_VOLUME *vol, NL_STRING rname );
GW_EXPORT NL_FLAG N_VolumeIsValid( NL_VOLUME *vol, const TCHAR * );
GW_EXPORT NL_FLAG N_VolumeHasEqualCPts( NL_VOLUME *vol, NL_REAL Tol, NL_INDEX *Nu, NL_INDEX *Nv, NL_INDEX *Nw );
GW_EXPORT NL_FLAG N_VolumeIsSized( NL_VOLUME *vol, NL_INDEX np, NL_INDEX mp, NL_INDEX op, NL_INDEX rk, NL_INDEX sk, 
                                   NL_INDEX tk, NL_STRING rname );

/************************/
/* Constructor routines */
/************************/

GW_EXPORT NL_FLAG N_VolumeConstruct( NL_INDEX, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_DEGREE, 
                                     NL_REAL *, NL_REAL *, NL_REAL *, NL_REAL ***, NL_POINT ***, NL_PARAMETER ru[2], 
                                     NL_PARAMETER rv[2], NL_PARAMETER rw[2], NL_VOLUME *, NL_STACKS *);

/********************/
/* Utility routines */
/********************/

GW_EXPORT NL_VOLUME *N_AllocVolume( NL_STACKS *S );
GW_EXPORT NL_VOLUME *N_AllocVolumeAndArrays( NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, 
                                             NL_INDEX is, NL_INDEX it, NL_STACKS *S );
GW_EXPORT NL_BOOLEAN N_AreVolumeArraysNULL( NL_VOLUME *vol );
GW_EXPORT NL_VOID N_VolumeInitArrays( NL_VOLUME *vol );
GW_EXPORT NL_FLAG N_AllocVolumeArrays( NL_VOLUME *vol, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, 
                                       NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STACKS *S );
GW_EXPORT NL_VOID N_VolumeSetSizeIndices( NL_VOLUME *vol, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, 
                                          NL_INDEX ir, NL_INDEX is, NL_INDEX it );
GW_EXPORT NL_FLAG N_VolumeSizeArrays( NL_VOLUME *vol, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, 
                                      NL_INDEX is, NL_INDEX it, NL_STRING rname, NL_STACKS *S );
GW_EXPORT NL_FLAG N_VolumeCopy( NL_VOLUME *volP, NL_VOLUME *volQ, NL_STACKS *S );
GW_EXPORT NL_VOID N_FreeVolume( NL_VOLUME *vol, NL_STACKS *S );

GW_EXPORT NL_VOID N_VolumeFromCMeshAndKnotVectors( NL_VOLUME *vol, NL_CMESH *mesh, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, 
                                                   NL_KNOTVECTOR *knu, NL_KNOTVECTOR *knv, NL_KNOTVECTOR *knw );
GW_EXPORT NL_FLAG N_VolumeFromCPtsAndKnots( NL_VOLUME *vol, NL_CPOINT *** Pw, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, 
                                            NL_DEGREE r, NL_REAL *U, NL_REAL *V, NL_REAL *W, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STACKS *S );
GW_EXPORT NL_FLAG N_VolumeReadFromFile( NL_VOLUME *vol, FILE *fptr, NL_FLAG chk, NL_STACKS *S );
GW_EXPORT NL_FLAG N_VolumeWriteToFile( NL_VOLUME *vol, FILE *fptr );
GW_EXPORT NL_VOID N_VolumePruneRat( NL_VOLUME *vol );
GW_EXPORT NL_VOID N_VolumeMakeNonRat( NL_VOLUME *vol );

/**********************/
/* Simple data access */
/**********************/

GW_EXPORT NL_FLAG N_VolumeGetDenominatorFunc( NL_VOLUME *, NL_VFUN *, NL_STACKS * );
GW_EXPORT NL_VOID N_VolumeGetCPtsAndKnots( NL_VOLUME *, NL_CPOINT ****, NL_REAL **, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_VolumeGetMeshAndKnotVectors( NL_VOLUME *, NL_CMESH **, NL_DEGREE *, NL_DEGREE *, NL_DEGREE *, 
                                                 NL_KNOTVECTOR **, NL_KNOTVECTOR **, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_VolumeGetCPtsDegreesAndKnots( NL_VOLUME *, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_CPOINT ****, NL_DEGREE *, NL_DEGREE *, 
                                                  NL_DEGREE *, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_REAL **, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_VolumeGetArraySizes( NL_VOLUME *, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_INDEX * );
GW_EXPORT NL_VOID N_VolumeGetDegrees( NL_VOLUME *, NL_DEGREE *, NL_DEGREE *, NL_DEGREE * );
GW_EXPORT NL_VOID N_VolumeGetKnotVectors( NL_VOLUME *, NL_KNOTVECTOR **, NL_KNOTVECTOR **, NL_KNOTVECTOR ** );
GW_EXPORT NL_VOID N_VolumeGetParamBounds( NL_VOLUME *, NL_PARAMETER *, NL_PARAMETER *, NL_PARAMETER *, NL_PARAMETER *, NL_PARAMETER *, NL_PARAMETER * );
GW_EXPORT NL_VOID N_VolumeGetKnots( NL_VOLUME *, NL_INDEX *, NL_INDEX *, NL_INDEX *, NL_REAL **, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_VOID N_VolumeGetCPts( NL_VOLUME *, NL_INDEX *, NL_INDEX *n, NL_INDEX *o, NL_CPOINT **** Pw );
GW_EXPORT NL_VOID N_VolumeGetCPtsKnotVectorAndKnots( NL_VOLUME *, NL_CPOINT ****, NL_KNOTVECTOR **, NL_KNOTVECTOR **, NL_KNOTVECTOR **, 
                                                     NL_REAL **, NL_REAL **, NL_REAL ** );
GW_EXPORT NL_FLAG N_VolumeGetEMesh( NL_VOLUME *, NL_INDEX ku, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_EMESH *, NL_STACKS * );
GW_EXPORT NL_FLAG N_VolumeGetBBox( NL_VOLUME *, NL_MINMAXBOX * );
GW_EXPORT NL_VOID N_VolumeSetCPtsAndKnots( NL_VOLUME *, NL_CPOINT ***, NL_REAL *, NL_REAL *, NL_REAL * );

/*****************/
/* modifications */
/*****************/

GW_EXPORT NL_FLAG N_VolumeCompress( NL_VOLUME *, NL_STACKS * );

/**************/
/* Predicates */
/**************/

GW_EXPORT NL_BOOLEAN N_VolumeIsRat( NL_VOLUME *vol );
GW_EXPORT NL_BOOLEAN N_VolumeIsDegen( NL_VOLUME *vol );
GW_EXPORT NL_BOOLEAN N_VolumeAreWeightsEqual( NL_VOLUME *vol );
GW_EXPORT NL_BOOLEAN N_VolumesAreEqual( NL_VOLUME *, NL_VOLUME *, NL_REAL, NL_REAL );

/***************/
/* Persistence */
/***************/

/******************/
/* NURB Functions */
/******************/

GW_EXPORT NL_FLAG N_VolumeEval( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_POINT *S );
GW_EXPORT NL_FLAG N_VolumeDerivs( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, 
                                  NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_POINT *** SD );
GW_EXPORT NL_FLAG N_VolumeBasisDerivs( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, 
                                       NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL ****** BDer, NL_INDEX *usp, NL_INDEX *vsp, NL_INDEX *wsp );
GW_EXPORT NL_FLAG N_VolumeBasisIEval( NL_VOLUME *vol, NL_INDEX i, NL_INDEX j, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, 
                                      NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_REAL *R );
GW_EXPORT NL_FLAG N_VolumeRatBasisDerivs( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, 
                                          NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL ****** RD, NL_INDEX *usp, NL_INDEX *vsp, NL_INDEX *wsp );
GW_EXPORT NL_FLAG N_VolumeRatBasisIDerivs( NL_VOLUME *vol, NL_INDEX i, NL_INDEX j, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, 
                                           NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL *** RD );
GW_EXPORT NL_FLAG N_VolumeRatBasisIEval( NL_VOLUME *vol, NL_INDEX i, NL_INDEX j, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, 
                                         NL_FLAG vfl, NL_FLAG wfl, NL_REAL *R );
GW_EXPORT NL_FLAG N_VolumeNonRatBasisDerivs( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, 
                                             NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL ****** BDer, NL_INDEX *usp, NL_INDEX *vsp, NL_INDEX *wsp );
GW_EXPORT NL_FLAG N_VolumeEvalGrid( NL_VOLUME *vol, NL_PARAMETER *u, NL_PARAMETER *v, NL_PARAMETER *w, NL_INDEX mm, NL_INDEX nn, NL_INDEX oo, 
                                    NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_POINT *** S );
GW_EXPORT NL_VOID N_VolumeTransform( NL_VOLUME *vol, NL_RMATRIX *rma );
GW_EXPORT NL_VOID N_VolumeScale( NL_VOLUME *vol, NL_POINT C, NL_VECTOR f );
GW_EXPORT NL_VOID N_VolumeTranslate( NL_VOLUME *vol, NL_VECTOR T );
GW_EXPORT NL_FLAG N_VolumeRotateAtPoint( NL_VOLUME *vol, NL_POINT P, NL_VECTOR V, NL_REAL al );
GW_EXPORT NL_VOID N_VolumeReparam( NL_VOLUME *vol, NL_PARAMETER us, NL_PARAMETER ue, NL_PARAMETER vs, NL_PARAMETER ve, NL_PARAMETER ws, NL_PARAMETER we );
GW_EXPORT NL_FLAG N_VolumeInsertKnot( NL_VOLUME *volP, NL_PARAMETER t, NL_INDEX mt, NL_FLAG dir, NL_VOLUME *volQ, NL_STACKS *SP, NL_STACKS *SQ );
GW_EXPORT NL_FLAG N_VolumeMakeIsoSurface( NL_VOLUME *vol, NL_PARAMETER t, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SC );
GW_EXPORT NL_FLAG N_VolumeMakeIsoCurve( NL_VOLUME *vol, NL_PARAMETER t1, NL_PARAMETER t2, NL_FLAG dir, NL_CURVE *cur, NL_STACKS *SC );
GW_EXPORT NL_FLAG N_VolumeElevateDegree( NL_VOLUME *volP, NL_INDEX t, NL_FLAG dir, NL_VOLUME *volQ, NL_STACKS *SP, NL_STACKS *SQ );
GW_EXPORT NL_FLAG N_VolumeMakeBoundaryCurves( NL_VOLUME *vol, NL_CURVE *curU00, NL_CURVE *curU01, NL_CURVE *curU11, NL_CURVE *curU10, 
                                              NL_CURVE *curV00, NL_CURVE *curV01, NL_CURVE *curV11, NL_CURVE *curV10, NL_CURVE *curW00, 
                                              NL_CURVE *curW01, NL_CURVE *curW11, NL_CURVE *curW10, NL_STACKS *SC );
GW_EXPORT NL_FLAG N_VolumeMakeBoundarySurfaces( NL_VOLUME *vol, NL_SURFACE *surU0, NL_SURFACE *surU1, NL_SURFACE *surV0, 
                                                NL_SURFACE *surV1, NL_SURFACE *surW0, NL_SURFACE *surW1, NL_STACKS *SC );

#endif /* _VOLUMEADV_H */
