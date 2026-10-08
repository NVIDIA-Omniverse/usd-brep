// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "StdAfx.h"
#include "SM_API_test.h"
#include "step_derivatives.h"

#include <exception>
#include <iostream>
#include <sstream>

SmStatus TestSmSTEPDerivativesExample()
{
    try
    {
        std::ostringstream discardedReport;
        RunSTEPDerivativesExample(discardedReport);
        return SM_SUCCESS;
    }
    catch (const std::exception& error)
    {
        std::cerr << "STEP derivative example: " << error.what() << '\n';
        return SM_ERR;
    }
}
