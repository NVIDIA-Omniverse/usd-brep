// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Developer-only Python bindings for SMLib debugger workflows.  This module
// intentionally stays separate from the stable _omni_solid API surface.
//
// Import _omni_solid first so Brep / Face / Edge / Vertex / Curve / Surface
// handles from the stable module cast correctly into dev bindings.

#include "SmPyDevCommon.h"
#include "SmPyCommon.h"

#include <SmApiGeneral.h>

#if defined(__GNUC__) && !defined(_WIN32)
#define SMPYLIB_DEV_OPNEW_ATTR __attribute__((used, visibility("default")))
#else
#define SMPYLIB_DEV_OPNEW_ATTR
#endif

// pybind11 references operator new(size_t) for registered kernel types.  Mirror
// the shims in SmPyMain.cpp so _smlib_dev can dlopen without _omni_solid.
SMPYLIB_DEV_OPNEW_ATTR void* SmBrep::operator new(size_t sz)        { return smos_Malloc(sz); }
SMPYLIB_DEV_OPNEW_ATTR void* SmPolyBrep::operator new(size_t sz)    { return smos_Malloc(sz); }
SMPYLIB_DEV_OPNEW_ATTR void* SmPolyVertex::operator new(size_t sz)  { return smos_Malloc(sz); }
SMPYLIB_DEV_OPNEW_ATTR void* SmPolyEdge::operator new(size_t sz)    { return smos_Malloc(sz); }
SMPYLIB_DEV_OPNEW_ATTR void* SmPolyFace::operator new(size_t sz)    { return smos_Malloc(sz); }

PYBIND11_MODULE(_smlib_dev, m)
{
    m.doc() =
        "Developer-only Python bindings for SMLib debugger workflows.\n"
        "\n"
        "This module exposes kernel-internal topology (loopuses, edgeuses),\n"
        "AssertValid reports, kernel Dump() text, kernel Draw() extraction,\n"
        "viewport draw helpers, and prog_test facades.  It is intended for\n"
        "the SMLib Python GUI and kernel developers, not for production\n"
        "modeling scripts.\n"
        "\n"
        "Import ``_omni_solid`` (``import _omni_solid as sm``) for stable\n"
        "modeling operations; use ``_smlib_dev`` only when you need its kernel\n"
        "debugging tools.\n"
        "\n"
        "Submodules\n"
        "----------\n"
        "  draw        - ``extract_draw_batches``, ``viewport_draw_*``\n"
        "  tests       - ``list_prog_test_suites``, ``run_prog_test_suite``\n"
        "                (forwards to ``_smlib_tests`` when built)\n"
        "\n"
        "Topology types (dev-only): ``Loop``, ``Loopuse``, ``Edgeuse``.\n"
        "``assert_valid`` runs ``sm_AssertValid``; debug SMLib pretty-prints failures.\n"
        "``dump`` runs kernel ``Dump()``; debug SMLib echoes the text to stderr.\n"
        "``user_test`` / ``user_test_example`` are kernel-direct C++ entry points\n"
        "for GUI playback (edit ``SmPyDevUserTests.cpp`` and rebuild).\n"
        "\n"
        "Object lifetime matches ``_omni_solid``: handles are kernel-owned.";

    py::module_::import("_omni_solid");

    if (SmApiGetOrCreateContext() == nullptr) {
        SmApiCreateContext();
    }

    bind_dev_topology(m);
    bind_dev_assert_valid(m);
    bind_dev_dump(m);
    bind_dev_draw(m);
    bind_dev_tests(m);
    bind_dev_user_tests(m);
}
