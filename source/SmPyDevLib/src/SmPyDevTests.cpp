// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Facade over the dev-only _smlib_tests module for prog_test suite execution.

#include "SmPyDevCommon.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void bind_dev_tests(py::module_& m)
{
    auto sub = m.def_submodule(
        "tests",
        "In-process prog_test suite runner (requires ``_smlib_tests`` build).");

    sub.def(
        "available",
        []() {
            try {
                py::module_::import("_smlib_tests");
                return true;
            } catch (const py::error_already_set&) {
                PyErr_Clear();
                return false;
            }
        },
        "``True`` when the ``_smlib_tests`` dev module is importable.");

    sub.def(
        "list_prog_test_suites",
        []() {
            py::module_ tests = py::module_::import("_smlib_tests");
            return tests.attr("list_prog_test_suites")();
        },
        "List supported ``prog_test`` suite names.");

    sub.def(
        "run_prog_test_suite",
        [](const std::string& name, const std::string& working_directory, bool do_graphics) {
            py::module_ tests = py::module_::import("_smlib_tests");
            return tests.attr("run_prog_test_suite")(name, working_directory, do_graphics);
        },
        py::arg("name"),
        py::arg("working_directory") = "",
        py::arg("do_graphics") = false,
        "Run a ``prog_test`` suite in-process and return a result dict.");

    sub.def(
        "debug_runtime_info",
        []() {
            py::module_ tests = py::module_::import("_smlib_tests");
            return tests.attr("debug_runtime_info")();
        },
        "Return compile flags and loaded library paths for debugger setup.");
}
