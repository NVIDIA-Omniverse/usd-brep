// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "StdAfx.h"
#include "SM_API_test.h"
#include "boolean_bounds.h"

#include <exception>
#include <iostream>
#include <sstream>

SmStatus TestSmBooleanBoundsExample()
{
    try
    {
        // Run the actual example, including all three bounds policies and
        // its analytic/invariance checks. Never assert a timing threshold.
        // The example's report (including timings) is discarded to keep the
        // suite output clean; failures still surface through the exception.
        std::ostringstream discardedReport;
        RunBooleanBoundsExample(discardedReport);
        return SM_SUCCESS;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Boolean/bounds example: " << error.what() << '\n';
        return SM_ERR;
    }
}
