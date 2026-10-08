// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Bridges: SmApiPrimitives.h (SmApiCreate{Box,Sphere,Cone,Cylinder,Torus,Plane,
// Pyramid,...} and partial/skin/swung/blend primitive creators).

#include "SmPyCommon.h"

#include <SmApiPrimitives.h>

void bind_primitives(BoundModule& m)
{
    m.def("create_box", [](py::tuple origin, double l, double w, double h) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateBox(o, l, w, h, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("length"), py::arg("width"), py::arg("height"),
       "Create a six-sided box or cube solid.\n"
       "\n"
       "Args:\n"
       "    origin: Position at the lower-left corner of the solid as ``(x, y, z)``.\n"
       "    length: Distance along the X axis. Must be positive.\n"
       "    width: Distance along the Y axis. Must be positive.\n"
       "    height: Distance along the Z axis. Must be positive.\n"
       "\n"
       "Returns:\n"
       "    Brep: axis-aligned box (6 planar faces, 12 edges, 8 vertices).\n"
       "\n"
       "Wraps: SmApiCreateBox");

    m.def("create_sphere", [](py::tuple origin, double radius) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateSphere(o, radius, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius"),
       "Create a solid sphere with poles.\n"
       "\n"
       "Args:\n"
       "    origin: Center of the sphere as ``(x, y, z)``.\n"
       "    radius: Radius of the sphere. Must be non-zero; a negative radius\n"
       "        gives the same sphere with a reversed (inside-out) parameterization.\n"
       "\n"
       "Returns:\n"
       "    Brep: closed manifold solid.\n"
       "\n"
       "Notes:\n"
       "    The standard NURBS sphere has degenerate poles at top and bottom\n"
       "    and a seam edge where the periodic surface wraps.\n"
       "\n"
       "See Also:\n"
       "    create_sphere_no_pole: Sphere built from patches that avoid pole singularities.\n"
       "    create_partial_sphere: Spherical sector between two angles.\n"
       "\n"
       "Wraps: SmApiCreateSphere");

    m.def("create_cone", [](py::tuple origin, double rBase, double rTop, double h) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateCone(o, rBase, rTop, h, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius_base"), py::arg("radius_top"), py::arg("height"),
       "Create a solid cone.\n"
       "\n"
       "Args:\n"
       "    origin: Position of the center of the base as ``(x, y, z)``.\n"
       "    radius_base: Radius at the base (Z = 0). Must be non-negative.\n"
       "    radius_top: Radius at the top (Z = ``height``). Must be non-negative.\n"
       "        Use 0 for a pointed cone; positive values produce a frustum\n"
       "        (truncated cone).\n"
       "    height: Distance between base and top, along the Z axis. Must be positive.\n"
       "\n"
       "Returns:\n"
       "    Brep: closed manifold solid (axis aligned along Z).\n"
       "\n"
       "See Also:\n"
       "    create_partial_cone: Conical sector between two angles.\n"
       "    create_cone_elliptic_ends: Cone with tilted elliptic end cross-sections.\n"
       "\n"
       "Wraps: SmApiCreateCone");

    m.def("create_cylinder", [](py::tuple origin, double radius, double height) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateCylinder(o, radius, height, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius"), py::arg("height"),
       "Create a solid cylinder.\n"
       "\n"
       "Args:\n"
       "    origin: Position of the center of the base as ``(x, y, z)``.\n"
       "    radius: Radius of the cylinder. Must be positive.\n"
       "    height: Distance between base and top, along the Z axis. Must be positive.\n"
       "\n"
       "Returns:\n"
       "    Brep: capped manifold solid (axis aligned along Z).\n"
       "\n"
       "See Also:\n"
       "    create_partial_cylinder: Cylindrical sector between two angles.\n"
       "    create_cylindrical_box: Hollow cylinder sector defined by inside/outside radii.\n"
       "\n"
       "Wraps: SmApiCreateCylinder");

    m.def("create_torus", [](py::tuple origin, double rMajor, double rMinor) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateTorus(o, rMajor, rMinor, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius_major"), py::arg("radius_minor"),
       "Create a solid torus.\n"
       "\n"
       "Args:\n"
       "    origin: Position of the center of the torus as ``(x, y, z)``.\n"
       "    radius_major: Distance from the torus center to the tube center.\n"
       "        Must be positive.\n"
       "    radius_minor: Radius of the tube cross-section. Must be positive.\n"
       "\n"
       "Returns:\n"
       "    Brep: closed manifold solid (major circle in the XY plane,\n"
       "    tube circling around Z).\n"
       "\n"
       "Notes:\n"
       "    For a standard torus ``radius_major`` should be greater than\n"
       "    ``radius_minor``. Equal radii give a self-intersecting horn torus;\n"
       "    ``radius_minor > radius_major`` gives a spindle torus.\n"
       "\n"
       "See Also:\n"
       "    create_partial_torus: Toroidal sector between two angles.\n"
       "\n"
       "Wraps: SmApiCreateTorus");

    m.def("create_plane", [](py::tuple origin, double length, double width) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreatePlane(o, length, width, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("length"), py::arg("width"),
       "Create a planar rectangular face (sheet body, not a solid).\n"
       "\n"
       "Args:\n"
       "    origin: Position of the (minU, minV) corner as ``(x, y, z)``.\n"
       "    length: Length along the X axis. Must be positive.\n"
       "    width: Width along the Y axis. Must be positive.\n"
       "\n"
       "Returns:\n"
       "    Brep: single-face sheet body in the XY plane (1 face, 4 edges,\n"
       "    4 vertices). ``is_manifold_solid`` returns ``False``.\n"
       "\n"
       "See Also:\n"
       "    create_planar_circle: Circular sheet body in the XY plane.\n"
       "\n"
       "Wraps: SmApiCreatePlane");

    m.def("create_planar_circle", [](py::tuple origin, double radius) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreatePlanarCircle(o, radius, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius"),
       "Create a circular planar face (sheet body, not a solid).\n"
       "\n"
       "Args:\n"
       "    origin: Center of the circular face as ``(x, y, z)``.\n"
       "    radius: Radius of the circle. Must be positive.\n"
       "\n"
       "Returns:\n"
       "    Brep: single-face sheet body in the XY plane.\n"
       "\n"
       "See Also:\n"
       "    create_plane: Rectangular sheet body in the XY plane.\n"
       "\n"
       "Wraps: SmApiCreatePlanarCircle");

    m.def("create_pyramid", [](py::tuple origin, double length, double height) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreatePyramid(o, length, height, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("length"), py::arg("height"),
       "Create a pyramid with a square base and an apex point.\n"
       "\n"
       "Args:\n"
       "    origin: Position of the center of the base as ``(x, y, z)``.\n"
       "    length: Length and width of the square base. Must be positive.\n"
       "    height: Height from base to apex, along the Z axis. Must be positive.\n"
       "\n"
       "Returns:\n"
       "    Brep: closed manifold solid (square base in the XY plane, apex on the Z axis).\n"
       "\n"
       "See Also:\n"
       "    create_cone: Pointed solid with a circular base (use ``radius_top=0``).\n"
       "\n"
       "Wraps: SmApiCreatePyramid");

    m.def("create_cylindrical_box", [](py::tuple origin, double length,
                                       double insideR, double outsideR,
                                       double startDeg, double endDeg) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateCylindricalBox(o, length, insideR, outsideR, startDeg, endDeg, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("length"), py::arg("inside_radius"),
       py::arg("outside_radius"), py::arg("start_angle_deg"), py::arg("end_angle_deg"),
       "Create a cylindrical box (hollow cylinder sector).\n"
       "\n"
       "Args:\n"
       "    origin: Position of the origin on the Z axis at the base as ``(x, y, z)``.\n"
       "    length: Length along the Z axis. Must be positive.\n"
       "    inside_radius: Inside radius from the Z axis. Must be positive.\n"
       "    outside_radius: Outside radius from the Z axis. Must be greater than\n"
       "        ``inside_radius``.\n"
       "    start_angle_deg: Start angle in the XY plane from the X axis, in degrees.\n"
       "        Must be in ``[-360, 360]``.\n"
       "    end_angle_deg: End angle in the XY plane from the X axis, in degrees.\n"
       "        Must be greater than ``start_angle_deg`` and at most 360 degrees\n"
       "        beyond it.\n"
       "\n"
       "Returns:\n"
       "    Brep: closed manifold solid (wedge-shaped section of a hollow cylinder,\n"
       "    axis aligned along Z).\n"
       "\n"
       "See Also:\n"
       "    create_cylinder: Solid cylinder.\n"
       "    create_partial_cylinder: Solid cylinder sector (no inside hole).\n"
       "\n"
       "Wraps: SmApiCreateCylindricalBox");

    m.def("create_partial_sphere", [](py::tuple origin, double radius,
                                      double startDeg, double endDeg) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreatePartialSphere(o, radius, startDeg, endDeg, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius"), py::arg("start_angle_deg"), py::arg("end_angle_deg"),
       "Create a spherical wedge (azimuthal sector swept about the Z axis).\n"
       "\n"
       "Args:\n"
       "    origin: Center of the sphere as ``(x, y, z)``.\n"
       "    radius: Radius of the sphere. Must be positive.\n"
       "    start_angle_deg: Start of the azimuthal sweep, measured in the XY\n"
       "        plane from the X axis, in degrees.\n"
       "    end_angle_deg: End of the azimuthal sweep, measured in the XY plane\n"
       "        from the X axis, in degrees. Must be greater than ``start_angle_deg``.\n"
       "\n"
       "Returns:\n"
       "    Brep: spherical wedge (lune) bounded by the two meridian half-disks at\n"
       "    ``start_angle_deg`` and ``end_angle_deg``, sweeping about the Z axis.\n"
       "\n"
       "Notes:\n"
       "    The angular sweep is azimuthal (longitude-like, about Z), NOT\n"
       "    latitudinal. ``create_partial_sphere(o, r, 0, 180)`` produces a\n"
       "    hemisphere split along the YZ plane (an orange-wedge half), not the\n"
       "    upper equatorial cap. Implementation: builds a pole-to-pole meridian\n"
       "    half-circle at ``start_angle_deg`` and rotationally sweeps it about\n"
       "    Z by ``end_angle_deg - start_angle_deg``.\n"
       "\n"
       "See Also:\n"
       "    create_sphere: Full sphere.\n"
       "    create_sphere_no_pole: Full sphere without polar singularities.\n"
       "\n"
       "Wraps: SmApiCreatePartialSphere");

    m.def("create_sphere_no_pole", [](py::tuple center, double radius) {
        SmVector3d c = to_vec(center);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateSphereNoPole(c, radius, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("center"), py::arg("radius"),
       "Create a sphere without singularities at the poles.\n"
       "\n"
       "Args:\n"
       "    center: Center of the sphere as ``(x, y, z)``.\n"
       "    radius: Radius of the sphere. Must be non-zero; a negative radius\n"
       "        gives the same sphere with a reversed (inside-out) parameterization.\n"
       "\n"
       "Returns:\n"
       "    Brep: closed manifold solid.\n"
       "\n"
       "Notes:\n"
       "    Constructs the sphere from patches that avoid the degenerate pole\n"
       "    vertices of the standard NURBS sphere.\n"
       "\n"
       "See Also:\n"
       "    create_sphere: Standard NURBS sphere (with poles).\n"
       "    create_partial_sphere: Spherical sector between two angles.\n"
       "\n"
       "Wraps: SmApiCreateSphereNoPole");

    m.def("create_partial_cone", [](py::tuple origin, double rBase, double rTop,
                                    double h, double startDeg, double endDeg) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreatePartialCone(o, rBase, rTop, h, startDeg, endDeg, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius_base"), py::arg("radius_top"),
       py::arg("height"), py::arg("start_angle_deg"), py::arg("end_angle_deg"),
       "Create a partial cone with start and end angles.\n"
       "\n"
       "Args:\n"
       "    origin: Position of the center of the base as ``(x, y, z)``.\n"
       "    radius_base: Radius at the base. Must be non-negative.\n"
       "    radius_top: Radius at the top. Must be non-negative. Use 0 for a\n"
       "        pointed apex.\n"
       "    height: Height of the cone, along the Z axis. Must be positive.\n"
       "    start_angle_deg: Start angle in the XY plane from the X axis, in degrees.\n"
       "    end_angle_deg: End angle in the XY plane from the X axis, in degrees.\n"
       "        Must be greater than ``start_angle_deg``.\n"
       "\n"
       "Returns:\n"
       "    Brep: conical sector between ``start_angle_deg`` and ``end_angle_deg``.\n"
       "\n"
       "See Also:\n"
       "    create_cone: Full cone.\n"
       "    create_partial_cylinder: Cylindrical sector (equal base and top radii).\n"
       "\n"
       "Wraps: SmApiCreatePartialCone");

    m.def("create_cone_elliptic_ends", [](double frontR, double rearR, double h,
                                          double frontTiltAboutX, double rearTiltAboutX,
                                          double frontTiltAboutY, double rearTiltAboutY,
                                          bool endCaps) {
        SmBrep* r = nullptr;
        // Note: SMLib kernel parameters are dFront/RearYTiltDeg (rotation
        // about X) and dFront/RearXTiltDeg (rotation about Y).  The Python
        // names reflect the rotation axis directly to avoid ambiguity.
        CHECK_STATUS(SmApiCreateConeEllipticEnds(frontR, rearR, h,
                     frontTiltAboutX, rearTiltAboutX, frontTiltAboutY, rearTiltAboutY,
                     endCaps ? TRUE : FALSE, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("front_radius"), py::arg("rear_radius"), py::arg("height"),
       py::arg("front_tilt_about_x_deg") = 0.0, py::arg("rear_tilt_about_x_deg") = 0.0,
       py::arg("front_tilt_about_y_deg") = 0.0, py::arg("rear_tilt_about_y_deg") = 0.0,
       py::arg("end_caps") = true,
       "Create a cone with elliptic ends and independent end tilts.\n"
       "\n"
       "Args:\n"
       "    front_radius: Radius at the front end (before tilt). Must be positive.\n"
       "    rear_radius: Radius at the rear end (before tilt). Must be positive.\n"
       "    height: Distance between the centers of the two end ellipses.\n"
       "        Must be positive.\n"
       "    front_tilt_about_x_deg: Front end rotation about the X axis, in degrees.\n"
       "    rear_tilt_about_x_deg: Rear end rotation about the X axis, in degrees.\n"
       "    front_tilt_about_y_deg: Front end rotation about the Y axis, in degrees.\n"
       "    rear_tilt_about_y_deg: Rear end rotation about the Y axis, in degrees.\n"
       "    end_caps: If ``True`` (default), close off the ends with cap faces.\n"
       "\n"
       "Returns:\n"
       "    Brep: cone with tilted elliptic end cross-sections.\n"
       "\n"
       "Notes:\n"
       "    Stability: this API is currently known to crash inside the kernel\n"
       "    for some inputs (tracking ticket TBD; see source for details).\n"
       "\n"
       "    Naming differs from the C kernel: the SMLib parameters are\n"
       "    ``dFront/RearYTiltDeg`` (rotation about X) and ``dFront/RearXTiltDeg``\n"
       "    (rotation about Y), where the ``X``/``Y`` letter refers to the\n"
       "    *direction the end tips toward*. The Python names use the rotation\n"
       "    axis directly to avoid that ambiguity.\n"
       "\n"
       "See Also:\n"
       "    create_cone: Standard cone with circular cross-sections.\n"
       "\n"
       "Wraps: SmApiCreateConeEllipticEnds");

    m.def("create_partial_cylinder", [](py::tuple origin, double radius, double height,
                                        double startDeg, double endDeg) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreatePartialCylinder(o, radius, height, startDeg, endDeg, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius"), py::arg("height"),
       py::arg("start_angle_deg"), py::arg("end_angle_deg"),
       "Create a partial cylinder with start and end angles.\n"
       "\n"
       "Args:\n"
       "    origin: Position of the center of the base as ``(x, y, z)``.\n"
       "    radius: Radius of the cylinder. Must be positive.\n"
       "    height: Height of the cylinder, along the Z axis. Must be positive.\n"
       "    start_angle_deg: Start angle in the XY plane from the X axis, in degrees.\n"
       "    end_angle_deg: End angle in the XY plane from the X axis, in degrees.\n"
       "        Must be greater than ``start_angle_deg``.\n"
       "\n"
       "Returns:\n"
       "    Brep: cylindrical sector between ``start_angle_deg`` and ``end_angle_deg``.\n"
       "\n"
       "Notes:\n"
       "    Internally delegates to ``SmApiCreatePartialCone`` with equal base\n"
       "    and top radii.\n"
       "\n"
       "See Also:\n"
       "    create_cylinder: Full cylinder.\n"
       "    create_cylindrical_box: Hollow cylinder sector with inside/outside radii.\n"
       "    create_partial_cone: Conical sector with independent base/top radii.\n"
       "\n"
       "Wraps: SmApiCreatePartialCylinder");

    m.def("create_partial_torus", [](py::tuple origin, double rMajor, double rMinor,
                                     double startDeg, double endDeg) {
        SmVector3d o = to_vec(origin);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreatePartialTorus(o, rMajor, rMinor, startDeg, endDeg, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("origin"), py::arg("radius_major"), py::arg("radius_minor"),
       py::arg("start_angle_deg"), py::arg("end_angle_deg"),
       "Create a partial torus with start and end angles.\n"
       "\n"
       "Args:\n"
       "    origin: Position of the center of the torus as ``(x, y, z)``.\n"
       "    radius_major: Major radius (center to tube center). Must be positive.\n"
       "    radius_minor: Minor radius (tube cross-section). Must be positive.\n"
       "    start_angle_deg: Start angle in the XY plane from the X axis, in degrees.\n"
       "    end_angle_deg: End angle in the XY plane from the X axis, in degrees.\n"
       "        Must be greater than ``start_angle_deg``.\n"
       "\n"
       "Returns:\n"
       "    Brep: toroidal sector between ``start_angle_deg`` and ``end_angle_deg``.\n"
       "\n"
       "See Also:\n"
       "    create_torus: Full torus.\n"
       "\n"
       "Wraps: SmApiCreatePartialTorus");

    m.def("create_swung_primitive", [](std::vector<SmCurve*>& xyCurves,
                                       std::vector<SmCurve*>& xzCurves,
                                       double scale) {
        SmTArray<SmCurve*> xy, xz;
        for (auto* c : xyCurves) xy.Add(c);
        for (auto* c : xzCurves) xz.Add(c);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateSwungPrimitive(xy, xz, scale, r));
        return r;
    }, py::return_value_policy::reference,
       py::arg("xy_curves"), py::arg("xz_curves"), py::arg("scale") = 1.0,
       "Create a swung solid from XY and XZ profile curves.\n"
       "\n"
       "Args:\n"
       "    xy_curves: Curves defining the XY profile.\n"
       "    xz_curves: Curves defining the XZ profile.\n"
       "    scale: Scale factor (default 1.0). Must be positive.\n"
       "\n"
       "Returns:\n"
       "    Brep: swung solid.\n"
       "\n"
       "See Also:\n"
       "    create_skin_primitive: Lofted solid from cross-section profile curves.\n"
       "\n"
       "Wraps: SmApiCreateSwungPrimitive");

    m.def("create_blend_primitive",
          [](SmBrep* brep, SmEdge* edge1, SmFace* face1, SmEdge* edge2, SmFace* face2, bool same_dir_curves) {
              SmFace* blend_face = nullptr;
              CHECK_STATUS(SmApiCreateBlendPrimitive(brep, edge1, face1, edge2, face2,
                                                     same_dir_curves ? TRUE : FALSE, blend_face));
              return blend_face;
          },
          py::return_value_policy::reference,
          py::arg("brep"), py::arg("edge1"), py::arg("face1"), py::arg("edge2"), py::arg("face2"),
          py::arg("same_dir_curves") = true,
          "Create a G2 blend surface between two boundary edges on existing faces.\n"
          "\n"
          "Args:\n"
          "    brep: Brep that will own the resulting blend face.\n"
          "    edge1: First boundary edge on ``face1``.\n"
          "    face1: Target surface at the start of the blend.\n"
          "    edge2: Second boundary edge on ``face2``.\n"
          "    face2: Target surface at the end of the blend.\n"
          "    same_dir_curves: If ``True`` (default), blend between the start of\n"
          "        ``edge1``'s curve and the start of ``edge2``'s curve. If ``False``,\n"
          "        blend from the start of ``edge1``'s curve to the end of ``edge2``'s\n"
          "        curve.\n"
          "\n"
          "Returns:\n"
          "    Face: resulting blend face inserted into ``brep``.\n"
          "\n"
          "Notes:\n"
          "    \"Blend\" here means a continuity-controlled connecting patch (G2 by\n"
          "    default in the kernel), distinct from edge-rounding fillets in the\n"
          "    ``fillet_*`` family. Obtain ``edge`` and ``face`` handles from stable\n"
          "    topology queries (``brep.faces()``, ``face.edges()``, ``edge.faces()``).\n"
          "\n"
          "Wraps: SmApiCreateBlendPrimitive");

    m.def("create_skin_primitive", [](const std::vector<unsigned long>& curves_per_profile,
                                      const std::vector<SmCurve*>& curves3d, unsigned long degree, bool cap_ends) {
        SmTArray<ULONG> counts;
        for (auto c : curves_per_profile) counts.Add(static_cast<ULONG>(c));
        SmTArray<const SmCurve*> arr;
        for (auto* c : curves3d) arr.Add(c);
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateSkinPrimitive(counts, arr, static_cast<ULONG>(degree),
                                              cap_ends ? TRUE : FALSE, r, nullptr, nullptr, nullptr));
        return r;
    }, py::return_value_policy::reference,
       py::arg("curves_per_profile"), py::arg("curves"), py::arg("degree") = 1,
       py::arg("cap_ends") = true,
       "Pure: returns new Brep; inputs unchanged.\n"
       "Create a lofted solid (skin) from a set of cross-section profile curves.\n"
       "\n"
       "Args:\n"
       "    curves_per_profile: Number of curves in each cross-section profile.\n"
       "        At least two positive counts are required, and\n"
       "        ``sum(curves_per_profile)`` must equal ``len(curves)``.\n"
       "    curves: All profile curve segments concatenated, in profile order.\n"
       "    degree: Degree in the loft direction. ``1`` = linear (default),\n"
       "        ``2`` = quadratic, ``3`` = cubic. Must be 1, 2, or 3.\n"
       "    cap_ends: If ``True`` (default), cap the first and last profiles\n"
       "        to close the solid. Intermediate profiles guide the loft and do\n"
       "        not create transverse faces.\n"
       "\n"
       "Returns:\n"
       "    Brep: lofted solid.\n"
       "\n"
       "Notes:\n"
       "    Input Curves remain valid and unchanged after both successful and failed calls.\n"
       "    Planar profiles may contain matching outer and inner loops.\n"
       "\n"
       "See Also:\n"
       "    create_skin_from_faces: Loft between boundary loops of existing faces.\n"
       "    create_swung_primitive: Surface of revolution from XY/XZ profiles.\n"
       "\n"
       "Wraps: SmApiCreateSkinPrimitive");

    m.def("create_skin_from_faces", [](std::vector<SmFace*>& faces, unsigned long degree) {
        SmTArray<SmFace*> in_faces;
        for (auto* f : faces) in_faces.Add(f);
        SmTArray<SmFace*> out_faces;
        SmBrep* r = nullptr;
        CHECK_STATUS(SmApiCreateSkinFromFaces(in_faces, static_cast<ULONG>(degree), out_faces, r));
        std::vector<SmFace*> vf;
        for (ULONG i = 0; i < out_faces.GetSize(); i++) vf.push_back(out_faces[i]);
        return py::make_tuple(vf, r);
    }, py::return_value_policy::reference,
       py::arg("faces"), py::arg("degree") = 1,
       "Create a lofted solid between matching loops on a sequence of faces.\n"
       "\n"
       "Args:\n"
       "    faces: Sequence of faces whose boundary loops should be lofted between.\n"
       "    degree: Degree in the loft direction. ``1`` = linear (default),\n"
       "        ``2`` = quadratic, ``3`` = cubic. Must be 1, 2, or 3.\n"
       "\n"
       "Returns:\n"
       "    tuple: ``(new_faces, brep)`` where ``new_faces`` is the list of\n"
       "    resulting lofted faces and ``brep`` is the lofted solid.\n"
       "\n"
       "See Also:\n"
       "    create_skin_primitive: Loft from raw cross-section curves instead of faces.\n"
       "\n"
       "Wraps: SmApiCreateSkinFromFaces");
}
