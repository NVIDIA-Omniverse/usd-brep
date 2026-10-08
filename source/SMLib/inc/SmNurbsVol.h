// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmNurbsVol.h
* PURPOSE: Header file for local copies of nurb functions.
**********************************************************************/

#ifndef __Sm_Nurbs_VOL_H__
#define __Sm_Nurbs_VOL_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __Sm_Nurbs_CRV_H__
#include <SmNurbsCrv.h>
#endif

class SmVolume ;

SM_EXPORT SmStatus sm_FindMeshExtrema
(
  const gw_VOLUME * pVol,          ///< [in] : Target Volume                                                                     <br>
  const SmPoint3d & rTestPoint,    ///< [in] : Target Point                                                                      <br>
  ULONG lNumToFind,                ///< [in] : Number of controlPoints to find and place in output arrays                        <br>
  double * adMinDistSq,            ///< [out]: array of TargetPoint/ControlPoint distances sorted by distance sized:[lNumToFind] <br>
  ULONG alMinUVIndex[10][2],       ///< [out]: associated controlPoint [iu, iv] indices for every adMinDistSq entry              <br>
  double * adMaxDistSq,            ///< [out]: array of TargetPoint/ControlPoint distances sorted by distance sized:[lNumToFind] <br>
  ULONG alMaxUVIndex[10][2],       ///< [out]: associated controlPoint [iu, iv] indices for every adMaxDistSq entry              <br>
  ULONG & rlNumMinFound,           ///< [out]: number of MinDist ControlPoints found [rlNumMinFound <= lNumToFind]               <br>
  ULONG & rlNumMaxFound            ///< [out]: number of MaxDist ControlPoints found [rlNumMinFound <= lNumToFind]               <br>
);

SM_EXPORT SmStatus sm_ComputePartialMeshConstants
(
  const gw_VOLUME *cpVol,           ///< [in] : Volume to query                                                     <br>
  ULONG lU0Span,                    ///< [in] : Min U Span to query                                                 <br>
  ULONG lV0Span,                    ///< [in] : Min V Span to query                                                 <br>
  ULONG LW0SPan,                    ///< [in] : Min W Span to query                                                 <br>
  ULONG lU1Span,                    ///< [in] : Max U Span to query                                                 <br>
  ULONG lV1Span,                    ///< [in] : Max V Span to query                                                 <br>
  ULONG LW1SPan,                    ///< [in] : Max W Span to query                                                 <br>
  SmExtent3d  * pBoundingBox,       ///< [out]: Bounding Box                                                        <br>
  SmPseudoBox * pPseudoBox,         ///< [out]: Pseudo Box                                                          <br>
  SmPoint3d & rU0V0W0,              ///< [out]: specified span corner point                                         <br>
  SmPoint3d & rU1V0W0,              ///< [out]: specified span corner point                                         <br>
  SmPoint3d & rU0V1W0,              ///< [out]: specified span corner point                                         <br>
  SmPoint3d & rU1V1W0,              ///< [out]: specified span corner point                                         <br>
  SmPoint3d & rU0V0W1,              ///< [out]: specified span corner point                                         <br>
  SmPoint3d & rU1V0W1,              ///< [out]: specified span corner point                                         <br>
  SmPoint3d & rU0V1W1,              ///< [out]: specified span corner point                                         <br>
  SmPoint3d & rU1V1W1,              ///< [out]: specified span corner point                                         <br>
  double * pdUChordHeight,          ///< [out]: Max U Chord height along any single row of constant vw CPoints      <br>
  double * pdVChordHeight,          ///< [out]: Max V Chord height along any single row of constant uw CPoints      <br>
  double * pdWChordHeight,          ///< [out]: Max W Chord height along any single row of constant uv CPoints      <br>
  double * pdUAngleDeg,             ///< [out]: Max U tangAngle Change along any single row of constant vw CPoints  <br>
  double * pdVAngleDeg,             ///< [out]: Max V tangAngle Change along any single row of constant uw CPoints  <br>
  double * pdWAngleDeg              ///< [out]: Max W tangAngle Change along any single row of constant uv CPoints  <br>
);

SM_EXPORT SmStatus sm_ComputeMeshConstants
(
  const SmVolume   * pVolume,         ///< [in] :      <br>
  const SmExtent3d * pUVDomain,       ///< [in] :      <br>
  const gw_VOLUME  * cpVol,           ///< [in] :      <br>
  double           * pdUChordHeight,  ///< [out]:      <br>
  double           * pdVChordHeight,  ///< [out]:      <br>
  double           * pdWChordHeight,  ///< [out]:      <br>
  double           * pdUAngleDeg,     ///< [out]:      <br>
  double           * pdVAngleDeg,     ///< [out]:      <br>
  double           * pdWAngleDeg      ///< [out]:      <br>
);

SM_EXPORT SmStatus sm_CopyNurbVolume
(
  const gw_VOLUME *cpFrom,            ///< [in] :      <br>
  gw_VOLUME *pTo                      ///< [out]:      <br>
);

SM_EXPORT ULONG sm_ComputeNurbVolumeSize(const gw_VOLUME *cpVolume) ;

SM_EXPORT ULONG sm_ComputeNurbVolumeSize
(
  gw_INDEX  lUCPointHighestIndex,     ///< [in] : max U ControlPoint index value = U ControlPoint_count - 1    <br>
  gw_INDEX  lVCPointHighestIndex,     ///< [in] : max V ControlPoint index value = V ControlPoint_count - 1    <br>
  gw_INDEX  lWCPointHighestIndex,     ///< [in] : max W ControlPoint index value = V ControlPoint_count - 1    <br>
  gw_INDEX  lUKnotsHighestIndex,      ///< [in] : max U knot index value = U knot_count - 1                    <br>
  gw_INDEX  lVKnotsHighestIndex,      ///< [in] : max V knot index value = V knot_count - 1                    <br>
  gw_INDEX  lWKnotsHighestIndex       ///< [in] : max W knot index value = W knot_count - 1                    <br>
);

SM_EXPORT void sm_InitNurbVolumeMemory
(
  gw_VOLUME* pVolumeMemory,           ///< [in] : pointer to volume memory block to init, sized:[sm_ComputeNurbVolumeSize()]  <br>
  gw_INDEX  lUCPointHighestIndex,     ///< [in] : max U ControlPoint index value = U ControlPoint_count - 1                   <br>
  gw_INDEX  lVCPointHighestIndex,     ///< [in] : max V ControlPoint index value = V ControlPoint_count - 1                   <br>
  gw_INDEX  lWCPointHighestIndex,     ///< [in] : max W ControlPoint index value = V ControlPoint_count - 1                   <br>
  gw_DEGREE lUDegree,                 ///< [in] : U dir degree                                                                <br>
  gw_DEGREE lVDegree,                 ///< [in] : V dir degree                                                                <br>
  gw_DEGREE lWDegree,                 ///< [in] : W dir degree                                                                <br>
  gw_INDEX  lUKnotsHighestIndex,      ///< [in] : max U knot index value = U knot_count - 1                                   <br>
  gw_INDEX  lVKnotsHighestIndex,      ///< [in] : max V knot index value = V knot_count - 1                                   <br>
  gw_INDEX  lWKnotsHighestIndex       ///< [in] : max W knot index value = W knot_count - 1                                   <br>
);

SM_EXPORT char *sm_AllocateBlockOfNurbVolumes
(
  gw_INDEX  lNumberOfVolumes,         ///< [in] :     <br>
  gw_INDEX  lUCPointHighestIndex,     ///< [in] :     <br>
  gw_INDEX  lVCPointHighestIndex,     ///< [in] :     <br>
  gw_INDEX  lWCPointHighestIndex,     ///< [in] :     <br>
  gw_DEGREE lUDegree,                 ///< [in] :     <br>
  gw_DEGREE lVDegree,                 ///< [in] :     <br>
  gw_DEGREE lWDegree,                 ///< [in] :     <br>
  gw_INDEX  lUKnotsHighestIndex,      ///< [in] :     <br>
  gw_INDEX  lVKnotsHighestIndex,      ///< [in] :     <br>
  gw_INDEX  lWKnotsHighestIndex,      ///< [in] :     <br>
  SmTArray<void*> & rVolumes          ///< [out]:     <br>
);

SM_EXPORT gw_VOLUME * sm_AllocateNurbVolume
(
  gw_INDEX  lUCPointHighestIndex,      ///< [in] : max U ControlPoint index value = U ControlPoint_count - 1   <br>
  gw_INDEX  lVCPointHighestIndex,      ///< [in] : max V ControlPoint index value = V ControlPoint_count - 1   <br>
  gw_INDEX  lWCPointHighestIndex,      ///< [in] : max W ControlPoint index value = W ControlPoint_count - 1   <br>
  gw_DEGREE lUDegree,                  ///< [in] : U dir degree                                                <br>
  gw_DEGREE lVDegree,                  ///< [in] : V dir degree                                                <br>
  gw_DEGREE lWDegree,                  ///< [in] : W dir degree                                                <br>
  gw_INDEX  lUKnotsHighestIndex,       ///< [in] : max U knot index value = U knot_count - 1                   <br>
  gw_INDEX  lVKnotsHighestIndex,       ///< [in] : max V knot index value = V knot_count - 1                   <br>
  gw_INDEX  lWKnotsHighestIndex        ///< [in] : max W knot index value = W knot_count - 1                   <br>
);

SM_EXPORT gw_VOLUME * sm_AllocateAndCopyNurbVolume
(
  const gw_VOLUME *cpSrcVol            ///< [in] : target volume to copy <br>
);

#endif // !__Sm_Nurbs_VOL_H__


