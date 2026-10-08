// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSmlibAll.h
* PURPOSE:  Header file which includes almost everything
*    Note that it does not include some of the IGES stuff.
**********************************************************************/

#ifndef __SMTSLIB_ALL_H__
#define __SMTSLIB_ALL_H__

#ifndef __SMASSEMBLY_H__
#include <SmAssembly.h>
#endif // !__SMASSEMBLY_H__

#ifndef __SMASSEMBLYINSTANCE_H__
#include <SmAssemblyInstance.h>
#endif // !__SMASSEMBLYINSTANCE_H__


#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMBREPDATA_H__
#include <SmBrepData.h>
#endif

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

// Remove Composites
// #ifndef __SMCEDGE_H__
// #include <SmCEdge.h>
// #endif

// Remove Composites
// #ifndef __SMCFACE_H__
// #include <SmCFace.h>
// #endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifndef __SMVERTEXUSE_H__
#include <SmVertexuse.h>
#endif

#ifndef __SMLINE_H__
#include <SmLine.h>
#endif

#ifndef __SMCIRCLE_H__
#include <SmCircle.h>
#endif

#ifndef __SMCONE_H__
#include <SmCone.h>
#endif

#ifndef __SMPLANE_H__
#include <SmPlane.h>
#endif

#ifndef __SMSPHERE_H__
#include <SmSphere.h>
#endif

#ifndef __SMCYLINDER_H__
#include <SmCylinder.h>
#endif

#ifndef __SMTORUS_H__
#include <SmTorus.h>
#endif

#ifndef __SMSURFOFREVOLUTION_H__
#include <SmSurfOfRevolution.h>
#endif

#ifndef __SMSURFOFEXTRUSION_H__
#include <SmSurfOfExtrusion.h>
#endif

#ifndef __SMVOLUME_ALL_H__
#include <SmVolumeAll.h>
#endif

#ifndef __SMBREPCACHE_H__
#include <SmBrepCache.h>
#endif

#ifndef __SMCACHEMGRBREP_H__
#include <SmCacheMgrBrep.h>
#endif

#ifndef __SMCACHEMGRTSRF_H__
#include <SmCacheMgrTSrf.h>
#endif

#ifndef __SMCURVEBOUNDEDSURFACE_H__
#include <SmCurveBoundedSurface.h>
#endif

#ifndef __SMCURVECLASSIFICATION_H__
#include <SmCurveClass.h>
#endif

#ifndef __SMPOINTCLASSIFICATION_H__
#include <SmPointClass.h>
#endif

#ifndef __SMTRIMSRFCACHE_H__
#include <SmTrimSrfCache.h>
#endif

#ifndef __SMTOPOLOGYSOLVER_H__
#include <SmTopologySolver.h>
#endif

#ifndef __SMVOLUME_H__
  #include <SmVolume.h>
  #include <SmBSplineVolume.h>
#endif

#ifndef __SMPRIMITIVECREATION_H__
#include <SmPrimitiveCreation.h>
#endif

#ifndef __SMASSERT_VALID_H__
#include <SmAssertArray.h>     
#endif 

#endif // !__SMTSLIB_ALL_H__
