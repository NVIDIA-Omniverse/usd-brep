// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <iosfwd>

// Shared by the standalone example and SM_API_test. Throws on a failed kernel
// call or numerical check. Creates and destroys its own geometry and context.
void RunSTEPDerivativesExample(std::ostream& output);
