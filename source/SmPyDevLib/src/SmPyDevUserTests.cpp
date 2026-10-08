// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// User-managed kernel-direct tests for the Python GUI.
// Invoked as smdev.user_test().
//
// Edit user_test() below, rebuild _smlib_dev (./repo.sh build), then pick
// test_4 in tools/smlib_gui/user_test.py and press Run.
// Return a Brep so the GUI can tessellate it. nullptr is valid for dump-only
// experiments. Copy from user_test_example() when you want a starting point.

#include "SmAssertArray.h"
#include "SmPyDevCommon.h"
#include "SmPyCommon.h"

#include <SmApiGeneral.h>

namespace
{

SmBrep* make_box(double origin_x, double origin_y, double origin_z,
                        double length, double width, double height)
{
    SmContext* context = SmApiGetOrCreateContext();
    if (context == nullptr) {
        throw std::runtime_error("SMLib context is not available");
    }

    SmBrep* box = new (*context) SmBrep();
    SmPoint3d origin(origin_x, origin_y, origin_z);
    SmAxis2Placement frame(
        origin,
        SmVector3d(1.0, 0.0, 0.0),
        SmVector3d(0.0, 1.0, 0.0));
    SmPrimitiveCreation creator(box->GetInfiniteRegion());
    CHECK_STATUS(creator.CreateBox(length, width, height, frame));
    return box;
}

} // namespace

// Kernel-direct analog of tools/smlib_gui/user_test.py test_2.
SmBrep* user_test_example()
{
    SmBrep* box = make_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
    SmApiDraw(box);
    SM_DUMP_AND_ASSERT_VALID(box);
    return box;
}

// Ad-hoc kernel-direct scratch pad. Rebuild _smlib_dev after editing.
SmBrep* user_test()
{
    return nullptr;
}

void bind_dev_user_tests(py::module_& m)
{
    m.def(
        "kernel_create_box",
        [](py::tuple origin, double length, double width, double height) {
            SmPoint3d o = to_point(origin);
            return make_box(o.x, o.y, o.z, length, width, height);
        },
        py::return_value_policy::reference,
        py::arg("origin"), py::arg("length"), py::arg("width"), py::arg("height"),
        "Kernel-direct ``SmPrimitiveCreation::CreateBox`` without SM_API input\n"
        "validation.\n"
        "\n"
        "Unlike ``_omni_solid.create_box``, degenerate dimensions (e.g. a zero\n"
        "height) are passed straight to the kernel, which builds an invalid\n"
        "Brep. Use it to exercise validators such as ``assert_valid``.\n"
        "\n"
        "Returns:\n"
        "    Brep: the kernel-built box.\n"
        "\n"
        "See Also:\n"
        "    _omni_solid.create_box: validated public constructor.");

    m.def(
        "user_test_example",
        &user_test_example,
        py::return_value_policy::reference,
        "Kernel-direct analog of GUI ``user_test.py`` test_2.\n"
        "\n"
        "Creates a 10x10x10 box at the origin with ``SmPrimitiveCreation``,\n"
        "then runs kernel ``Dump()`` and ``sm_AssertValid``.\n"
        "\n"
        "Returns:\n"
        "    Brep: the box, so the GUI can tessellate it.\n"
        "\n"
        "Notes:\n"
        "    This is a copy-paste template for ``user_test()``. It does not\n"
        "    go through ``_omni_solid.create_box``.\n"
        "\n"
        "See Also:\n"
        "    user_test: ad-hoc kernel-direct stub called from GUI test_4.");

    m.def(
        "user_test",
        &user_test,
        py::return_value_policy::reference,
        "Ad-hoc kernel-direct user test.\n"
        "\n"
        "Edit ``source/SmPyDevLib/src/SmPyDevUserTests.cpp``, rebuild\n"
        "``_smlib_dev``, then pick GUI user test 4 and press Run.\n"
        "\n"
        "Returns:\n"
        "    Brep or ``None``. Assign the result in Python so the GUI can\n"
        "    tessellate it: ``result = smdev.user_test()``.\n"
        "\n"
        "See Also:\n"
        "    user_test_example: kernel-direct analog of GUI test_2.");
}
