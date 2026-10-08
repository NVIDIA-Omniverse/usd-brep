// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef HYDRA_USD_BREP_TEST_H
#define HYDRA_USD_BREP_TEST_H

// Include after HdUsdBrepImagingHeaders.h, which must come first.
#include "pxr/imaging/hd/primvarsSchema.h"

#include <stdexcept>
#include <string>

inline void Check(bool ok, const std::string& message)
{
    if (!ok)
    {
        throw std::runtime_error(message);
    }
}

template <class Handle>
Handle Require(Handle handle, const char* message)
{
    Check(bool(handle), message);
    return handle;
}

// Value of a Vec3f-array primvar on a Hydra prim.
inline PXR_NS::VtVec3fArray PrimvarVec3fArray(const PXR_NS::HdContainerDataSourceHandle& prim, const char* name)
{
    auto value = PXR_NS::HdPrimvarsSchema::GetFromParent(prim).GetPrimvar(PXR_NS::TfToken(name)).GetPrimvarValue();
    Check(bool(value), std::string("missing primvar ") + name);
    return value->GetValue(0).Get<PXR_NS::VtVec3fArray>();
}

void TestNativeInstances(const char* filename);

void TestDisplayAndAnimation(const char* filename);

void TestDoubleSidedAndOpacity(const char* filename);

void TestGeomSubsetChildren(const char* filename);

#endif
