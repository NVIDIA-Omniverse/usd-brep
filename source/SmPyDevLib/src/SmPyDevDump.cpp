// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Developer-only Dump binding: call kernel Dump() on a Brep or selected
// Face / Edge / Vertex / Loop / Loopuse / Edgeuse (and Curve / Surface).
// Debug SMLib echoes Dump() through smos_WriteBuffer to stderr.

#include "SmPyCommon.h"
#include "SmPyDevCommon.h"

namespace
{

void dump_kernel_object(py::object object, bool abbreviated)
{
    const SmBoolean abbrev = abbreviated ? TRUE : FALSE;
    if (py::isinstance<SmBrep>(object))
    {
        SmBrep* brep = object.cast<SmBrep*>();
        if (brep == nullptr)
        {
            throw py::value_error("Cannot dump a null SMLib object");
        }
        brep->Dump();
        return;
    }
    if (py::isinstance<SmFace>(object))
    {
        SmFace* face = object.cast<SmFace*>();
        if (face == nullptr)
        {
            throw py::value_error("Cannot dump a null SMLib object");
        }
        face->Dump(abbrev);
        return;
    }
    if (py::isinstance<SmEdge>(object))
    {
        SmEdge* edge = object.cast<SmEdge*>();
        if (edge == nullptr)
        {
            throw py::value_error("Cannot dump a null SMLib object");
        }
        edge->Dump(abbrev);
        return;
    }
    if (py::isinstance<SmVertex>(object))
    {
        SmVertex* vertex = object.cast<SmVertex*>();
        if (vertex == nullptr)
        {
            throw py::value_error("Cannot dump a null SMLib object");
        }
        vertex->Dump(abbrev);
        return;
    }
    if (py::isinstance<SmEdgeuse>(object))
    {
        SmEdgeuse* edgeuse = object.cast<SmEdgeuse*>();
        if (edgeuse == nullptr)
        {
            throw py::value_error("Cannot dump a null SMLib object");
        }
        edgeuse->Dump();
        return;
    }
    if (py::isinstance<SmLoop>(object))
    {
        SmLoop* loop = object.cast<SmLoop*>();
        if (loop == nullptr)
        {
            throw py::value_error("Cannot dump a null SMLib object");
        }
        loop->Dump(abbrev);
        return;
    }
    if (py::isinstance<SmLoopuse>(object))
    {
        SmLoopuse* loopuse = object.cast<SmLoopuse*>();
        if (loopuse == nullptr)
        {
            throw py::value_error("Cannot dump a null SMLib object");
        }
        loopuse->Dump(abbrev);
        return;
    }
    if (py::isinstance<SmCurve>(object))
    {
        SmCurve* curve = object.cast<SmCurve*>();
        if (curve == nullptr)
        {
            throw py::value_error("Cannot dump a null SMLib object");
        }
        curve->Dump(abbrev);
        return;
    }
    if (py::isinstance<SmSurface>(object))
    {
        SmSurface* surface = object.cast<SmSurface*>();
        if (surface == nullptr)
        {
            throw py::value_error("Cannot dump a null SMLib object");
        }
        surface->Dump(abbrev);
        return;
    }
    throw py::type_error(
        "dump expects a Brep, Face, Edge, Vertex, Edgeuse, Loop, Loopuse, "
        "Curve, or Surface"
    );
}

void dump(py::object object, bool abbreviated)
{
    dump_kernel_object(object, abbreviated);
}

} // namespace

void bind_dev_dump(py::module_& m)
{
    m.def(
        "dump",
        [](py::object object, bool abbreviated)
        {
            dump(object, abbreviated);
        },
        py::arg("object"),
        py::arg("abbreviated") = false,
        "Run kernel ``Dump()`` on a Brep or selected topology.\n"
        "\n"
        "Args:\n"
        "    object: Brep, Face, Edge, Vertex, Edgeuse, Loop, Loopuse,\n"
        "        Curve, or Surface to dump.\n"
        "    abbreviated: If ``True``, ask types that support it for a\n"
        "        shorter dump. Defaults to\n"
        "        ``False`` (full dump). Ignored for ``Brep`` and ``Edgeuse``.\n"
        "\n"
        "Returns:\n"
        "    None. Kernel dump text is written by ``smos_WriteBuffer``.\n"
        "    Debug SMLib builds echo that text to the terminal (stderr).\n"
        "    Release builds intentionally do not echo Dump().\n"
        "\n"
        "Notes:\n"
        "    This calls the existing SMLib ``Dump()`` pretty-printers. It\n"
        "    does not inspect topology from Python. Launch the Python GUI\n"
        "    against a debug build to see dump text in the terminal.\n"
        "\n"
        "See Also:\n"
        "    assert_valid: Kernel ``sm_AssertValid`` for the same objects.\n"
        "    topology_pick_ray: Pick Face / Edge / Vertex handles to dump.\n"
        "\n"
        "Wraps: Dump"
    );
}
