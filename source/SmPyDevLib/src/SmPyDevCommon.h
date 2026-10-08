// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __SmPyDevCommon_H__
#define __SmPyDevCommon_H__

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_dev_topology(py::module_& m);
void bind_dev_assert_valid(py::module_& m);
void bind_dev_dump(py::module_& m);
void bind_dev_draw(py::module_& m);
void bind_dev_tests(py::module_& m);
void bind_dev_user_tests(py::module_& m);

#endif // __SmPyDevCommon_H__
