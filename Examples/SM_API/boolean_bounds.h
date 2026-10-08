// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <iosfwd>

// Shared by the standalone example and SM_API_test. Throws on failed API calls
// or correctness checks; reports timings without asserting a speed threshold.
void RunBooleanBoundsExample(std::ostream& output);
