// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiGeneral.h (SmApiDraw overloads for in-kernel dev-viewport
// visualization of Brep / PolyBrep / Face / Edge / curve / point arrays).
//
// All ``viewport_draw_*`` functions are no-ops unless the kernel's dev
// viewport has been enabled via ``smGet_DoGraphics()``; in normal headless
// Python use they do nothing and return success.  They exist for parity with
// the C API and for use from the developer test harness.

#include "SmPyCommon.h"

#include <SmConfig.h>
#include <SmExtent2d.h>
#include <SmApiGeneral.h>
#include <SmGraphicsVertexArray.h>

#include <stdexcept>

namespace {

#ifdef SM_GFX_OUTPUT_CODE

const char* smpy_draw_type_name(SmGfxVertexDrawType draw_type)
{
    switch (draw_type) {
    case SM_GV_POINT:       return "point";
    case SM_GV_LINE:        return "line";
    case SM_GV_SIMPLE_MESH: return "simple_mesh";
    case SM_GV_INDEX_MESH:  return "index_mesh";
    case SM_GV_UNKNOWN:
    default:                return "unknown";
    }
}

const char* smpy_vertex_data_type_name(SmGfxVertexDataType data_type)
{
    switch (data_type) {
    case SM_GV_P:        return "position";
    case SM_GV_PN:       return "position_normal";
    case SM_GV_COLORED:  return "position_normal_color";
    case SM_GV_TEXTURED: return "position_normal_uv";
    default:             return "unknown";
    }
}

py::tuple smpy_gfx_point_tuple(float x, float y, float z)
{
    return py::make_tuple(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
}

py::tuple smpy_gfx_uv_tuple(float u, float v)
{
    return py::make_tuple(static_cast<double>(u), static_cast<double>(v));
}

py::dict smpy_gfx_vertex_array_to_dict(SmGfxVertexArray* array, const std::string& object_type,
                                       const std::string& draw_variant, ULONG batch_index)
{
    py::dict batch;
    if (!array) {
        return batch;
    }

    py::list positions;
    py::list normals;
    py::list colors;
    py::list uvs;

    switch (array->GetVertexDataType()) {
    case SM_GV_P: {
        SmTArray<SmGfxVertex>* vertices = array->GetVertexArray();
        if (vertices) {
            for (ULONG ii = 0; ii < vertices->GetSize(); ++ii) {
                const SmGfxVertex& vertex = vertices->GetAt(ii);
                positions.append(smpy_gfx_point_tuple(vertex.m_fX, vertex.m_fY, vertex.m_fZ));
            }
        }
        break;
    }
    case SM_GV_PN: {
        SmTArray<SmGfxNVertex>* vertices = array->GetNVertexArray();
        if (vertices) {
            for (ULONG ii = 0; ii < vertices->GetSize(); ++ii) {
                const SmGfxNVertex& vertex = vertices->GetAt(ii);
                positions.append(smpy_gfx_point_tuple(vertex.m_fX, vertex.m_fY, vertex.m_fZ));
                normals.append(smpy_gfx_point_tuple(vertex.m_fNX, vertex.m_fNY, vertex.m_fNZ));
            }
        }
        break;
    }
    case SM_GV_COLORED: {
        SmTArray<SmGfxColoredVertex>* vertices = array->GetColoredVertexArray();
        if (vertices) {
            for (ULONG ii = 0; ii < vertices->GetSize(); ++ii) {
                const SmGfxColoredVertex& vertex = vertices->GetAt(ii);
                positions.append(smpy_gfx_point_tuple(vertex.m_fX, vertex.m_fY, vertex.m_fZ));
                normals.append(smpy_gfx_point_tuple(vertex.m_fNX, vertex.m_fNY, vertex.m_fNZ));
                colors.append(smpy_gfx_point_tuple(vertex.m_fR, vertex.m_fG, vertex.m_fB));
            }
        }
        break;
    }
    case SM_GV_TEXTURED: {
        SmTArray<SmGfxTexturedVertex>* vertices = array->GetTexturedVertexAarray();
        if (vertices) {
            for (ULONG ii = 0; ii < vertices->GetSize(); ++ii) {
                const SmGfxTexturedVertex& vertex = vertices->GetAt(ii);
                positions.append(smpy_gfx_point_tuple(vertex.m_fX, vertex.m_fY, vertex.m_fZ));
                normals.append(smpy_gfx_point_tuple(vertex.m_fNX, vertex.m_fNY, vertex.m_fNZ));
                uvs.append(smpy_gfx_uv_tuple(vertex.m_fU, vertex.m_fV));
            }
        }
        break;
    }
    default:
        break;
    }

    py::list indices;
    SmTArray<ULONG>* index_array = array->GetIndexArray();
    if (index_array) {
        for (ULONG ii = 0; ii < index_array->GetSize(); ++ii) {
            indices.append(index_array->GetAt(ii));
        }
    }

    const int stipple = array->GetStipple();
    batch["primitive"] = smpy_draw_type_name(array->GetVertexDrawType());
    batch["draw_type"] = smpy_draw_type_name(array->GetVertexDrawType());
    batch["data_type"] = smpy_vertex_data_type_name(array->GetVertexDataType());
    batch["positions"] = positions;
    if (normals.size() > 0) {
        batch["normals"] = normals;
    }
    if (colors.size() > 0) {
        batch["colors"] = colors;
    }
    if (uvs.size() > 0) {
        batch["uvs"] = uvs;
    }
    if (indices.size() > 0) {
        batch["indices"] = indices;
    }
    batch["color"] = smpy_gfx_point_tuple(array->GetRed(), array->GetGreen(), array->GetBlue());
    batch["point_size"] = static_cast<double>(array->GetPointSize());
    batch["line_width"] = static_cast<double>(array->GetLineWidth());
    batch["stipple"] = stipple;
    batch["dashed"] = (stipple != 0xffff);

    py::dict metadata;
    metadata["object_type"] = object_type;
    metadata["draw_variant"] = draw_variant;
    metadata["batch_index"] = batch_index;
    batch["metadata"] = metadata;
    return batch;
}

py::list smpy_gfx_array_set_to_batches(SmGfxArraySet& array_set, const std::string& object_type,
                                       const std::string& draw_variant)
{
    py::list batches;
    for (ULONG ii = 0; ii < array_set.GetSize(); ++ii) {
        py::dict batch = smpy_gfx_vertex_array_to_dict(array_set.GetAt(ii), object_type, draw_variant, ii);
        if (batch && batch.contains("positions")) {
            batches.append(batch);
        }
    }
    return batches;
}

void smpy_extract_default_draw(SmBrep* object, SmGfxArraySet& array_set)
{
    object->Draw(FALSE, &array_set);
}

void smpy_extract_default_draw(SmFace* object, SmGfxArraySet& array_set)
{
    object->Draw(&array_set);
}

void smpy_extract_default_draw(SmEdge* object, SmGfxArraySet& array_set)
{
    object->Draw(&array_set);
}

void smpy_extract_default_draw(SmVertex* object, SmGfxArraySet& array_set)
{
    object->Draw(&array_set);
}

void smpy_extract_default_draw(SmEdgeuse* object, SmGfxArraySet& array_set)
{
    object->Draw(1.0, TRUE, &array_set);
}

void smpy_extract_default_draw(SmLoop* object, SmGfxArraySet& array_set)
{
    object->Draw(3, FALSE, static_cast<SmPlane**>(NULL), TRUE, &array_set);
}

void smpy_extract_default_draw(SmLoopuse* object, SmGfxArraySet& array_set)
{
    object->Draw(3, FALSE, static_cast<SmPlane**>(NULL), TRUE, &array_set);
}

void smpy_extract_default_draw(SmCurve* object, SmGfxArraySet& array_set)
{
    object->Draw(static_cast<const SmExtent1d*>(NULL), FALSE, static_cast<SmPlane*>(NULL), &array_set);
}

void smpy_extract_default_draw(SmSurface* object, SmGfxArraySet& array_set)
{
    object->Draw(FALSE, &array_set);
}

void smpy_extract_draw_variant(SmBrep* object, SmGfxArraySet& array_set, const std::string& draw_variant)
{
    if (draw_variant != "default") {
        throw py::value_error("Brep draw extraction supports only the default variant");
    }
    smpy_extract_default_draw(object, array_set);
}

void smpy_extract_draw_variant(SmFace* object, SmGfxArraySet& array_set, const std::string& draw_variant)
{
    if (draw_variant == "default") {
        smpy_extract_default_draw(object, array_set);
        return;
    }
    if (draw_variant == "draw_uv") {
        object->DrawUV(8, 8, FALSE, static_cast<const SmExtent2d*>(NULL), FALSE, &array_set);
        return;
    }
    throw py::value_error("Face draw extraction supports variants: default, draw_uv");
}

void smpy_extract_draw_variant(SmEdge* object, SmGfxArraySet& array_set, const std::string& draw_variant)
{
    if (draw_variant != "default") {
        throw py::value_error("Edge draw extraction supports only the default variant");
    }
    smpy_extract_default_draw(object, array_set);
}

void smpy_extract_draw_variant(SmVertex* object, SmGfxArraySet& array_set, const std::string& draw_variant)
{
    if (draw_variant != "default") {
        throw py::value_error("Vertex draw extraction supports only the default variant");
    }
    smpy_extract_default_draw(object, array_set);
}

void smpy_extract_draw_variant(SmEdgeuse* object, SmGfxArraySet& array_set, const std::string& draw_variant)
{
    if (draw_variant != "default") {
        throw py::value_error("Edgeuse draw extraction supports only the default variant");
    }
    smpy_extract_default_draw(object, array_set);
}

void smpy_extract_draw_variant(SmLoop* object, SmGfxArraySet& array_set, const std::string& draw_variant)
{
    if (draw_variant != "default") {
        throw py::value_error("Loop draw extraction supports only the default variant");
    }
    smpy_extract_default_draw(object, array_set);
}

void smpy_extract_draw_variant(SmLoopuse* object, SmGfxArraySet& array_set, const std::string& draw_variant)
{
    if (draw_variant != "default") {
        throw py::value_error("Loopuse draw extraction supports only the default variant");
    }
    smpy_extract_default_draw(object, array_set);
}

void smpy_extract_draw_variant(SmCurve* object, SmGfxArraySet& array_set, const std::string& draw_variant)
{
    if (draw_variant != "default") {
        throw py::value_error("Curve draw extraction supports only the default variant");
    }
    smpy_extract_default_draw(object, array_set);
}

void smpy_extract_draw_variant(SmSurface* object, SmGfxArraySet& array_set, const std::string& draw_variant)
{
    if (draw_variant == "default") {
        smpy_extract_default_draw(object, array_set);
        return;
    }
    if (draw_variant == "draw_uv") {
        object->DrawUV(8, 8, FALSE, static_cast<const SmExtent2d*>(NULL), FALSE, &array_set);
        return;
    }
    throw py::value_error("Surface draw extraction supports variants: default, draw_uv");
}

template <typename T>
py::list smpy_extract_draw_batches_for(py::object object, const std::string& object_type,
                                       const std::string& draw_variant)
{
    T* typed_object = object.cast<T*>();
    if (!typed_object) {
        throw py::value_error("Cannot extract draw batches from a null SMLib object");
    }
    SmGfxArraySet array_set(0.0f, 0.0f, 0.0f);
    smpy_extract_draw_variant(typed_object, array_set, draw_variant);
    return smpy_gfx_array_set_to_batches(array_set, object_type, draw_variant);
}

#endif // SM_GFX_OUTPUT_CODE

py::list smpy_extract_draw_batches(py::object object, const std::string& draw_variant)
{
#ifndef SM_GFX_OUTPUT_CODE
    (void)object;
    (void)draw_variant;
    throw std::runtime_error(
        "kernel Draw extraction is unavailable because SMLib was built without SM_GFX_OUTPUT_CODE");
#else
    if (py::isinstance<SmBrep>(object)) {
        return smpy_extract_draw_batches_for<SmBrep>(object, "Brep", draw_variant);
    }
    if (py::isinstance<SmFace>(object)) {
        return smpy_extract_draw_batches_for<SmFace>(object, "Face", draw_variant);
    }
    if (py::isinstance<SmEdge>(object)) {
        return smpy_extract_draw_batches_for<SmEdge>(object, "Edge", draw_variant);
    }
    if (py::isinstance<SmVertex>(object)) {
        return smpy_extract_draw_batches_for<SmVertex>(object, "Vertex", draw_variant);
    }
    if (py::isinstance<SmEdgeuse>(object)) {
        return smpy_extract_draw_batches_for<SmEdgeuse>(object, "Edgeuse", draw_variant);
    }
    if (py::isinstance<SmLoop>(object)) {
        return smpy_extract_draw_batches_for<SmLoop>(object, "Loop", draw_variant);
    }
    if (py::isinstance<SmLoopuse>(object)) {
        return smpy_extract_draw_batches_for<SmLoopuse>(object, "Loopuse", draw_variant);
    }
    if (py::isinstance<SmCurve>(object)) {
        return smpy_extract_draw_batches_for<SmCurve>(object, "Curve", draw_variant);
    }
    if (py::isinstance<SmSurface>(object)) {
        return smpy_extract_draw_batches_for<SmSurface>(object, "Surface", draw_variant);
    }
    throw py::type_error(
        "extract_draw_batches expects a Brep, Face, Edge, Vertex, Edgeuse, Loop, Loopuse, Curve, or Surface");
#endif
}

} // namespace

void bind_viewport_draw(BoundModule& m)
{
    m.def(
        "extract_draw_batches",
        [](py::object object, const std::string& variant) {
            return smpy_extract_draw_batches(object, variant);
        },
        py::arg("object"), py::arg("variant") = "default",
        "Extract kernel ``Draw()`` geometry into Python-renderable batches.\n"
        "\n"
        "Args:\n"
        "    object: Brep, Face, Edge, Vertex, Edgeuse, Loop, Loopuse,\n"
        "        Curve, or Surface whose default kernel ``Draw()`` method\n"
        "        should be captured.\n"
        "    variant: Draw variant to use. ``\"default\"`` calls the object's\n"
        "        default kernel ``Draw()`` method. ``\"draw_uv\"`` calls\n"
        "        ``DrawUV()`` for Face and Surface objects.\n"
        "\n"
        "Returns:\n"
        "    list[dict[str, object]]: copied draw batches. Each batch stores\n"
        "    ``primitive`` (``\"point\"``, ``\"line\"``, ``\"simple_mesh\"``, or\n"
        "    ``\"index_mesh\"``), ``positions``, optional ``normals`` /\n"
        "    ``colors`` / ``uvs`` / ``indices``, display state (``color``,\n"
        "    ``point_size``, ``line_width``, ``stipple``, ``dashed``), and\n"
        "    source metadata.\n"
        "\n"
        "Raises:\n"
        "    RuntimeError: SMLib was built without ``SM_GFX_OUTPUT_CODE``,\n"
        "        so kernel Draw extraction cannot produce batches.\n"
        "\n"
        "Notes:\n"
        "    This uses the ``SmGfxArraySet`` output side channel on the\n"
        "    object's own ``Draw()`` method. It does not call ``SmApiDraw``\n"
        "    or the kernel developer viewport, so it works in normal Python\n"
        "    sessions where ``viewport_draw_*`` would no-op. A build that\n"
        "    defines ``SM_NO_GFX_OUTPUT_CODE`` raises ``RuntimeError``\n"
        "    instead of returning an empty list.\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_brep: Legacy dev-viewport draw helper.\n"
        "    tessellate: Main shaded Brep display path.\n"
        "\n"
        "Wraps: Draw(..., SmGfxArraySet*)");

    m.def(
        "viewport_draw_brep",
        [](SmBrep* obj, py::object color, bool clear_first, bool center) {
            std::unique_ptr<SmVector3d> holder;
            CHECK_STATUS(SmApiDraw(obj, optional_color(color, holder), clear_first ? TRUE : FALSE,
                                   center ? TRUE : FALSE));
        },
        py::arg("brep"), py::arg("color") = py::none(), py::arg("clear_first") = false,
        py::arg("center") = false,
        "Draw a Brep in the kernel's dev viewport.\n"
        "\n"
        "Args:\n"
        "    brep: Brep to draw.\n"
        "    color: Optional RGB color as ``(r, g, b)`` with components in\n"
        "        ``[0.0, 1.0]``. ``None`` (default) uses black.\n"
        "    clear_first: If ``True``, clear the viewport before drawing.\n"
        "        Defaults to ``False``.\n"
        "    center: If ``True``, recenter the view on the object after\n"
        "        drawing. Defaults to ``False``.\n"
        "\n"
        "Notes:\n"
        "    No-op unless the kernel's dev viewport is enabled. In a\n"
        "    normal headless Python session this returns success without\n"
        "    rendering anything. Intended for developer harnesses, not\n"
        "    production visualization - use ``tessellate`` plus an\n"
        "    external renderer for that.\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_face: Draw a single face.\n"
        "    viewport_draw_polybrep: Draw a tessellated mesh instead.\n"
        "    tessellate: Convert to a mesh for external rendering.\n"
        "\n"
        "Wraps: SmApiDraw(SmBrep*)");

    m.def(
        "viewport_draw_face",
        [](SmFace* obj, py::object color, bool clear_first, bool center) {
            std::unique_ptr<SmVector3d> holder;
            CHECK_STATUS(SmApiDraw(obj, optional_color(color, holder), clear_first ? TRUE : FALSE,
                                   center ? TRUE : FALSE));
        },
        py::arg("face"), py::arg("color") = py::none(), py::arg("clear_first") = false,
        py::arg("center") = false,
        "Draw a single face in the kernel's dev viewport.\n"
        "\n"
        "Args:\n"
        "    face: Face to draw. Typically obtained from\n"
        "        ``Brep.faces()`` or ``find_face``.\n"
        "    color: Optional RGB color as ``(r, g, b)``. ``None`` uses\n"
        "        black.\n"
        "    clear_first: Clear the viewport first. Defaults to ``False``.\n"
        "    center: Recenter the view on the object. Defaults to ``False``.\n"
        "\n"
        "Notes:\n"
        "    No-op unless the kernel's dev viewport is enabled (see\n"
        "    ``viewport_draw_brep``).\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_brep: Draw the parent Brep.\n"
        "    viewport_draw_curve: Draw an isolated curve.\n"
        "\n"
        "Wraps: SmApiDraw(SmFace*)");

    m.def(
        "viewport_draw_curve",
        [](SmCurve* obj, py::object color, bool clear_first, bool center) {
            std::unique_ptr<SmVector3d> holder;
            CHECK_STATUS(SmApiDraw(obj, optional_color(color, holder), clear_first ? TRUE : FALSE,
                                   center ? TRUE : FALSE));
        },
        py::arg("curve"), py::arg("color") = py::none(), py::arg("clear_first") = false,
        py::arg("center") = false,
        "Draw a curve in the kernel's dev viewport.\n"
        "\n"
        "Args:\n"
        "    curve: Curve to draw.\n"
        "    color: Optional RGB color as ``(r, g, b)``. ``None`` uses\n"
        "        black.\n"
        "    clear_first: Clear the viewport first. Defaults to ``False``.\n"
        "    center: Recenter the view on the object. Defaults to ``False``.\n"
        "\n"
        "Notes:\n"
        "    No-op unless the kernel's dev viewport is enabled (see\n"
        "    ``viewport_draw_brep``).\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_curves: Draw a list of curves in one call.\n"
        "    viewport_draw_surface: Draw the underlying surface of a face.\n"
        "\n"
        "Wraps: SmApiDraw(SmCurve*)");

    m.def(
        "viewport_draw_surface",
        [](SmSurface* obj, py::object color, bool clear_first, bool center) {
            std::unique_ptr<SmVector3d> holder;
            CHECK_STATUS(SmApiDraw(obj, optional_color(color, holder), clear_first ? TRUE : FALSE,
                                   center ? TRUE : FALSE));
        },
        py::arg("surface"), py::arg("color") = py::none(), py::arg("clear_first") = false,
        py::arg("center") = false,
        "Draw an untrimmed surface in the kernel's dev viewport.\n"
        "\n"
        "Args:\n"
        "    surface: Surface to draw.\n"
        "    color: Optional RGB color as ``(r, g, b)``. ``None`` uses\n"
        "        black.\n"
        "    clear_first: Clear the viewport first. Defaults to ``False``.\n"
        "    center: Recenter the view on the object. Defaults to ``False``.\n"
        "\n"
        "Notes:\n"
        "    No-op unless the kernel's dev viewport is enabled (see\n"
        "    ``viewport_draw_brep``). Draws the full untrimmed surface;\n"
        "    use ``viewport_draw_face`` to honor face trim curves.\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_face: Trimmed-face variant.\n"
        "\n"
        "Wraps: SmApiDraw(SmSurface*)");

    m.def(
        "viewport_draw_curves",
        [](std::vector<SmCurve*>& curves, py::object color, bool clear_first, bool center) {
            SmTArray<SmCurve*> arr;
            for (auto* c : curves) arr.Add(c);
            std::unique_ptr<SmVector3d> holder;
            CHECK_STATUS(SmApiDraw(arr, optional_color(color, holder), clear_first ? TRUE : FALSE,
                                   center ? TRUE : FALSE));
        },
        py::arg("curves"), py::arg("color") = py::none(), py::arg("clear_first") = false,
        py::arg("center") = false,
        "Draw a list of curves in the kernel's dev viewport in one call.\n"
        "\n"
        "Args:\n"
        "    curves: Curves to draw.\n"
        "    color: Optional RGB color as ``(r, g, b)`` applied to all\n"
        "        curves. ``None`` uses black.\n"
        "    clear_first: Clear the viewport first. Defaults to ``False``.\n"
        "    center: Recenter the view on the object. Defaults to ``False``.\n"
        "\n"
        "Notes:\n"
        "    No-op unless the kernel's dev viewport is enabled (see\n"
        "    ``viewport_draw_brep``).\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_curve: Single-curve variant.\n"
        "\n"
        "Wraps: SmApiDraw(SmTArray<SmCurve*>&)");

    m.def(
        "viewport_draw_point",
        [](py::tuple base, py::object color, bool clear_first, bool center) {
            SmPoint3d b = to_point(base);
            std::unique_ptr<SmVector3d> holder;
            CHECK_STATUS(SmApiDraw(b, optional_color(color, holder), clear_first ? TRUE : FALSE,
                                   center ? TRUE : FALSE));
        },
        py::arg("base"), py::arg("color") = py::none(), py::arg("clear_first") = false,
        py::arg("center") = false,
        "Draw a single 3D point in the kernel's dev viewport.\n"
        "\n"
        "Args:\n"
        "    base: Point location as ``(x, y, z)``.\n"
        "    color: Optional RGB color as ``(r, g, b)``. ``None`` uses\n"
        "        black.\n"
        "    clear_first: Clear the viewport first. Defaults to ``False``.\n"
        "    center: Recenter the view on the object. Defaults to ``False``.\n"
        "\n"
        "Notes:\n"
        "    No-op unless the kernel's dev viewport is enabled (see\n"
        "    ``viewport_draw_brep``).\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_points: Batch-draw a list of points.\n"
        "    viewport_draw_vector: Draw a point with a direction arrow.\n"
        "\n"
        "Wraps: SmApiDraw(SmPoint3d&)");

    m.def(
        "viewport_draw_points",
        [](std::vector<py::tuple>& points, py::object color, bool clear_first, bool center) {
            SmTArray<SmPoint3d> arr;
            fill_points_array(points, arr);
            std::unique_ptr<SmVector3d> holder;
            CHECK_STATUS(SmApiDraw(arr, optional_color(color, holder), clear_first ? TRUE : FALSE,
                                   center ? TRUE : FALSE));
        },
        py::arg("points"), py::arg("color") = py::none(), py::arg("clear_first") = false,
        py::arg("center") = false,
        "Draw a list of 3D points in the kernel's dev viewport in one call.\n"
        "\n"
        "Args:\n"
        "    points: Points as a list of ``(x, y, z)`` tuples.\n"
        "    color: Optional RGB color as ``(r, g, b)`` applied to all\n"
        "        points. ``None`` uses black.\n"
        "    clear_first: Clear the viewport first. Defaults to ``False``.\n"
        "    center: Recenter the view on the object. Defaults to ``False``.\n"
        "\n"
        "Notes:\n"
        "    No-op unless the kernel's dev viewport is enabled (see\n"
        "    ``viewport_draw_brep``).\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_point: Single-point variant.\n"
        "\n"
        "Wraps: SmApiDraw(SmTArray<SmPoint3d>&)");

    m.def(
        "viewport_draw_vector",
        [](py::tuple base, py::tuple direction, py::object color, bool clear_first, bool center) {
            SmPoint3d b = to_point(base);
            SmVector3d d = to_vec(direction);
            std::unique_ptr<SmVector3d> holder;
            CHECK_STATUS(SmApiDraw(b, d, optional_color(color, holder), clear_first ? TRUE : FALSE,
                                   center ? TRUE : FALSE));
        },
        py::arg("base"), py::arg("direction"), py::arg("color") = py::none(),
        py::arg("clear_first") = false, py::arg("center") = false,
        "Draw a 3D vector arrow from a base point in the kernel's dev viewport.\n"
        "\n"
        "Args:\n"
        "    base: Tail of the arrow as ``(x, y, z)``.\n"
        "    direction: Arrow direction and length as ``(dx, dy, dz)``.\n"
        "        Magnitude is honored - the arrow tip is at\n"
        "        ``base + direction``.\n"
        "    color: Optional RGB color as ``(r, g, b)``. ``None`` uses\n"
        "        black.\n"
        "    clear_first: Clear the viewport first. Defaults to ``False``.\n"
        "    center: Recenter the view on the object. Defaults to ``False``.\n"
        "\n"
        "Notes:\n"
        "    No-op unless the kernel's dev viewport is enabled (see\n"
        "    ``viewport_draw_brep``). Useful for visualizing surface\n"
        "    normals or ray directions during debugging.\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_point: Draw just the base point.\n"
        "\n"
        "Wraps: SmApiDraw(SmPoint3d&, SmVector3d&)");

    m.def(
        "viewport_draw_polybrep",
        [](SmPolyBrep* obj, py::object color, bool clear_first, bool center) {
            std::unique_ptr<SmVector3d> holder;
            CHECK_STATUS(SmApiDraw(obj, optional_color(color, holder), clear_first ? TRUE : FALSE,
                                   center ? TRUE : FALSE));
        },
        py::arg("poly_brep"), py::arg("color") = py::none(), py::arg("clear_first") = false,
        py::arg("center") = false,
        "Draw a polygon mesh (``PolyBrep``) in the kernel's dev viewport.\n"
        "\n"
        "Args:\n"
        "    poly_brep: Mesh to draw.\n"
        "    color: Optional RGB color as ``(r, g, b)``. ``None`` uses\n"
        "        black.\n"
        "    clear_first: Clear the viewport first. Defaults to ``False``.\n"
        "    center: Recenter the view on the object. Defaults to ``False``.\n"
        "\n"
        "Notes:\n"
        "    No-op unless the kernel's dev viewport is enabled (see\n"
        "    ``viewport_draw_brep``).\n"
        "\n"
        "See Also:\n"
        "    viewport_draw_brep: Exact-NURBS Brep counterpart.\n"
        "    tessellate: Produce a ``PolyBrep`` from a Brep first.\n"
        "\n"
        "Wraps: SmApiDraw(SmPolyBrep*)");
}

void bind_dev_draw(py::module_& m)
{
    auto sub_draw = m.def_submodule(
        "draw",
        "Kernel Draw() extraction and legacy dev-viewport draw helpers.");
    BoundModule bm_draw { m, sub_draw };
    bind_viewport_draw(bm_draw);
}
