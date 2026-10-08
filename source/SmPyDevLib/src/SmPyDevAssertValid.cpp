// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Developer-only AssertValid binding: call SmObject::AssertValid() on a Brep
// or selected Face / Edge / Vertex / Loop / Loopuse / Edgeuse (and Curve /
// Surface), then format the resulting SmAssertArray here.
//
// This deliberately bypasses sm_AssertValid(), which returns TRUE without
// running any checks outside SM_DEBUG_CODE. The AssertValid() methods
// themselves are compiled in every configuration, so calling them directly
// makes the result meaningful in release builds too. The kernel's own
// SmAssertArray::Dump() pretty-printer is not used for the same reason: it
// writes through smos_WriteBuffer, which only reaches a terminal in debug
// SMLib. Formatting the reports here keeps debug and release output identical.

#include "SmPyCommon.h"
#include "SmPyDevCommon.h"

#include <sstream>

namespace
{

SmObject* sm_object_from_python(py::object object)
{
    if (py::isinstance<SmObject>(object))
    {
        SmObject* obj = object.cast<SmObject*>();
        if (obj == nullptr)
        {
            throw py::value_error("Cannot run AssertValid on a null SMLib object");
        }
        return obj;
    }

    // Face / Edge / Vertex / Loop / Loopuse / Edgeuse are bound without an
    // SmObject pybind base, but they inherit SmObject in C++.
    if (py::isinstance<SmFace>(object))
    {
        SmFace* face = object.cast<SmFace*>();
        if (face == nullptr)
        {
            throw py::value_error("Cannot run AssertValid on a null SMLib object");
        }
        return face;
    }
    if (py::isinstance<SmEdge>(object))
    {
        SmEdge* edge = object.cast<SmEdge*>();
        if (edge == nullptr)
        {
            throw py::value_error("Cannot run AssertValid on a null SMLib object");
        }
        return edge;
    }
    if (py::isinstance<SmVertex>(object))
    {
        SmVertex* vertex = object.cast<SmVertex*>();
        if (vertex == nullptr)
        {
            throw py::value_error("Cannot run AssertValid on a null SMLib object");
        }
        return vertex;
    }
    if (py::isinstance<SmEdgeuse>(object))
    {
        SmEdgeuse* edgeuse = object.cast<SmEdgeuse*>();
        if (edgeuse == nullptr)
        {
            throw py::value_error("Cannot run AssertValid on a null SMLib object");
        }
        return edgeuse;
    }
    if (py::isinstance<SmLoop>(object))
    {
        SmLoop* loop = object.cast<SmLoop*>();
        if (loop == nullptr)
        {
            throw py::value_error("Cannot run AssertValid on a null SMLib object");
        }
        return loop;
    }
    if (py::isinstance<SmLoopuse>(object))
    {
        SmLoopuse* loopuse = object.cast<SmLoopuse*>();
        if (loopuse == nullptr)
        {
            throw py::value_error("Cannot run AssertValid on a null SMLib object");
        }
        return loopuse;
    }
    // Curve / Surface currently inherit an SmObject pybind base, so the first
    // guard usually catches them. Keep explicit branches so a later binding
    // change (as with Face / Edge / Vertex) still dispatches correctly.
    if (py::isinstance<SmCurve>(object))
    {
        SmCurve* curve = object.cast<SmCurve*>();
        if (curve == nullptr)
        {
            throw py::value_error("Cannot run AssertValid on a null SMLib object");
        }
        return curve;
    }
    if (py::isinstance<SmSurface>(object))
    {
        SmSurface* surface = object.cast<SmSurface*>();
        if (surface == nullptr)
        {
            throw py::value_error("Cannot run AssertValid on a null SMLib object");
        }
        return surface;
    }

    throw py::type_error(
        "assert_valid expects a Brep, Face, Edge, Vertex, Edgeuse, Loop, "
        "Loopuse, Curve, or Surface"
    );
}

SmAssertTestLevel parse_test_level(int level)
{
    switch (level)
    {
        case 0:
            return SM_LEVEL_0;
        case 1:
            return SM_LEVEL_1;
        case 2:
            return SM_LEVEL_2;
        default:
            throw py::value_error("level must be 0, 1, or 2");
    }
}

// GetAssertTypeString() right-pads to a fixed width for column alignment.
std::string trim_trailing(std::string text)
{
    const std::size_t end = text.find_last_not_of(' ');
    text.erase(end == std::string::npos ? 0 : end + 1);
    return text;
}

// Mirrors the fields SmAssertReport::Dump() prints, with two differences: the
// file/line labels only exist as members under SM_DEBUG_CODE, and the class
// label is the owner's rather than the reporting class's, because the kernel's
// SM_TYPE-to-string mapper is not declared in any public header.
std::string format_report(const SmAssertReport& report, ULONG index)
{
    TCHAR message[SM_TBLOCK_SIZE];
    report.FormatLogMessage(message, SM_TBLOCK_SIZE);

    std::ostringstream out;
    out << "  [" << index << "] " << (report.m_bOK ? "PASSED" : "FAILED") << " "
        << trim_trailing(smos_FromTChar(report.GetAssertTypeString())) << " "
        << smos_FromTChar(report.m_sOwnerTypeString) << "::Test #" << report.GetTestIndex() << " : "
        << smos_FromTChar(report.m_pName) << " - " << smos_FromTChar(message) << " Owner["
        << smos_FromTChar(report.m_sOwnerTypeString) << "=" << report.GetOwner() << "]";

    if (report.GetOther() != nullptr)
    {
        out << ", Other[" << smos_FromTChar(report.m_sOtherTypeString) << "=" << report.GetOther() << "]";
    }
    return out.str();
}

struct AssertValidRun
{
    bool ok = true;
    std::vector<std::string> reports;
    // Context for the stderr echo only, so len(reports) stays a report count.
    std::string header;
};

AssertValidRun assert_valid(py::object object, int level, bool walk)
{
    SmObject* obj = sm_object_from_python(object);
    const SmAssertTestLevel test_level = parse_test_level(level);

    SmAssertArray failures;
    const SmBoolean ok = obj->AssertValid(&failures, test_level, walk ? SM_WALK : SM_NO_WALK, nullptr);

    AssertValidRun run;
    run.ok = ok ? true : false;
    run.reports.reserve(failures.GetSize());
    for (ULONG ii = 0; ii < failures.GetSize(); ++ii)
    {
        const SmAssertReport* report = failures.GetAt(ii);
        if (report != nullptr)
        {
            run.reports.push_back(format_report(*report, ii));
        }
    }

    if (!run.ok)
    {
        std::ostringstream header;
        header << "smdev.assert_valid: " << smos_FromTChar(obj->GetClassString()) << "="
               << static_cast<const void*>(obj)
               << " failed AssertValid (level=" << level << ", walk=" << (walk ? "True" : "False") << ") with "
               << run.reports.size() << " report(s)";
        run.header = header.str();
    }
    return run;
}

void echo_run(const AssertValidRun& run)
{
    if (run.header.empty() && run.reports.empty())
    {
        return;
    }
    py::object stream = py::module_::import("sys").attr("stderr");
    if (!run.header.empty())
    {
        stream.attr("write")(run.header + "\n");
    }
    for (const std::string& line : run.reports)
    {
        stream.attr("write")(line + "\n");
    }
    stream.attr("flush")();
}

} // namespace

void bind_dev_assert_valid(py::module_& m)
{
    m.def(
        "assert_valid",
        [](py::object object, int level, bool walk)
        {
            AssertValidRun run = assert_valid(object, level, walk);
            echo_run(run);
            return run.ok;
        },
        py::arg("object"),
        py::arg("level") = 2,
        py::arg("walk") = true,
        "Run kernel ``AssertValid`` on a Brep or selected topology.\n"
        "\n"
        "Args:\n"
        "    object: Brep, Face, Edge, Vertex, Edgeuse, Loop, Loopuse,\n"
        "        Curve, or Surface to check.\n"
        "    level: Test depth. ``0`` is pointer/structure checks, ``1``\n"
        "        adds geometry checks, ``2`` adds slow checks. Defaults to\n"
        "        ``2``. Level ``2`` with\n"
        "        ``walk=True`` is expensive on a large Brep in any build\n"
        "        configuration.\n"
        "    walk: If ``True`` (default), also validate topology-graph\n"
        "        descendants. If ``False``, check only the given object.\n"
        "\n"
        "Returns:\n"
        "    bool: ``True`` when every check passed. Failed reports are\n"
        "    written to ``sys.stderr``; use ``assert_valid_reports`` to\n"
        "    receive them instead.\n"
        "\n"
        "Notes:\n"
        "    Checks run in debug *and* release builds. This calls\n"
        "    ``SmObject::AssertValid`` directly rather than\n"
        "    ``sm_AssertValid``, which returns ``True`` without testing\n"
        "    anything outside ``SM_DEBUG_CODE``, and it formats the\n"
        "    reports here rather than through ``SmAssertArray::Dump``, whose\n"
        "    ``smos_WriteBuffer`` output is debug-only. Report text therefore\n"
        "    matches in both configurations, except that the originating\n"
        "    file/line labels only exist in debug SMLib.\n"
        "\n"
        "See Also:\n"
        "    assert_valid_reports: Same checks, returning the report text.\n"
        "    dump: Kernel ``Dump()`` pretty-printer for the same objects.\n"
        "    topology_pick_ray: Pick Face / Edge / Vertex handles to check.\n"
        "\n"
        "Wraps: SmObject::AssertValid"
    );

    m.def(
        "assert_valid_reports",
        [](py::object object, int level, bool walk)
        {
            AssertValidRun run = assert_valid(object, level, walk);
            return py::make_tuple(run.ok, py::cast(run.reports));
        },
        py::arg("object"),
        py::arg("level") = 2,
        py::arg("walk") = true,
        "Run kernel ``AssertValid`` and return the formatted failure reports.\n"
        "\n"
        "Args:\n"
        "    object: Brep, Face, Edge, Vertex, Edgeuse, Loop, Loopuse,\n"
        "        Curve, or Surface to check.\n"
        "    level: Test depth, as for ``assert_valid``.\n"
        "    walk: Whether to validate topology-graph descendants, as for\n"
        "        ``assert_valid``.\n"
        "\n"
        "Returns:\n"
        "    tuple: ``(ok, reports)``. ``ok`` is ``True`` when every check\n"
        "    passed; ``reports`` is a list of formatted ``SmAssertArray``\n"
        "    lines, empty when ``ok``. Nothing is written to stderr, so the\n"
        "    caller owns presentation.\n"
        "\n"
        "See Also:\n"
        "    assert_valid: Same checks, returning only the bool.\n"
        "\n"
        "Wraps: SmObject::AssertValid"
    );
}
