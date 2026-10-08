// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmApiTessellationParams.h
* PURPOSE: Tessellation quality controls and their shared defaults.
*          Self-contained, so code that only needs the parameters does not
*          pull in the kernel headers of SmApiBrep.h.
**********************************************************************/

#ifndef __SmApiTessellationParams_H__
#define __SmApiTessellationParams_H__

/// Default tessellation quality, shared by the C++ API, the Python bindings and hdUsdBrep.
namespace SmTessellationDefaults
{
  constexpr double kChordHeightTol     = 0.0;   ///< 0 = ignore; absolute world units, so scale-dependent
  constexpr double kCurveAngleTolDeg   = 25.0;  ///< 0 = ignore
  constexpr double kSurfaceAngleTolDeg = 25.0;  ///< 0 = ignore
  constexpr double kMaxEdgeLength      = 0.0;   ///< 0 = ignore; edge-length subdivision target in world units, not a
                                                ///< hard cap (final triangle edges can be somewhat longer); negative
                                                ///< selects a model-relative target of 0.025*|value|*bbox-diagonal
  constexpr double kMaxAspectRatio     = 0.0;   ///< 0 = ignore
}

/// Tessellation quality controls; a default-constructed instance is the standard quality.
/// All enabled constraints contribute; refinement required by any applicable constraint
/// is retained. Each is disabled by 0.0.
struct SmTessellationParams
{
  double dChordHeightTol     = SmTessellationDefaults::kChordHeightTol;
  double dCurveAngleTolDeg   = SmTessellationDefaults::kCurveAngleTolDeg;
  double dSurfaceAngleTolDeg = SmTessellationDefaults::kSurfaceAngleTolDeg;
  double dMaxEdgeLength      = SmTessellationDefaults::kMaxEdgeLength;
  double dMaxAspectRatio     = SmTessellationDefaults::kMaxAspectRatio;
};

#endif // __SmApiTessellationParams_H__
