<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Operation Index

Quick reference for all SMLib operation guides organized by category.

## SM_API Operations

| Guide | Python Functions | C API Header | Description |
|---|---|---|---|
| [primitives](primitives.md) | `create_box`, `create_sphere`, `create_cone`, `create_cylinder`, `create_torus`, `create_plane`, `create_planar_circle`, `create_pyramid`, `create_cylindrical_box`, `create_partial_sphere`, `create_sphere_no_pole`, `create_partial_cone`, `create_partial_cylinder`, `create_partial_torus`, `create_swung_primitive`, `create_blend_primitive` | SmApiPrimitives.h | Create solid primitives and sheet bodies |
| [booleans](booleans.md) | `boolean`, `boolean_union`, `boolean_difference`, `boolean_intersection`, `boolean_merge`, `merge_breps`, `boolean_with_options`, `boolean_with_curves`, `boolean_2d`, `boolean_lists`, `evaluate_csg_tree`, `non_manifold_boolean`, `piecewise_merge` | SmApiBrep.h | Combine solids via set operations |
| [sweeps](sweeps.md) | `linear_sweep`, `rotational_sweep`, `draft_sweep`, `pipe_sweep`, `taper_extrude`, `sweep_along_planar_path`, `linear_sweep_with_repetitions`, `rotational_sweep_with_repetitions`, `curve_sweep`, `curve_sweep_from_faces`, `curve_sweep_from_edges`, `non_manifold_linear_sweep`, `non_manifold_rotational_sweep` | SmApiBrep.h | Create solids by sweeping profiles |
| [fillets](fillets.md) | `circular_fillet`, `chamfer_fillet`, `fillet_edges`, `fillet_edges_per_edge`, `variable_radius_fillet`, `remove_fillet`, `surface_surface_fillet`, `fillet_preview`, `set_bevel_corners` | SmApiFillets.h, SmApiBrep.h | Round or bevel edges |
| [curves](curves.md) | `create_line_segment`, `create_line`, `create_circle`, `create_arc`, `create_helix`, `create_ellipse`, `create_rectangle`, `create_regular_polygon`, `create_curve`, `create_canonical_curve`, `create_curve_approx_points`, `create_curve_interp_points`, `offset_curve`, `evaluate_curve`, `evaluate_continuity`, `drop_curve_to_surface`, `convert_to_lines_and_arcs`, `project_curve_to_surface`, `lift_uv_curve`, `make_curves_compatible`, `order_curves`, `remove_curve_knots` | SmApiCurves.h | Create and simplify curves for profiles and paths |
| [surfaces](surfaces.md) | `create_surface_from_points`, `create_surface_from_corner_points`, `create_surface_from_ordered_points`, `create_surface_from_random_points`, `create_ruled_surface`, `create_surface_revolution`, `create_offset_surface`, `create_extrude_surface`, `create_sweep_surface`, `create_skin_surface`, `create_planar_faces` | SmApiSurfaces.h, SmApiTrimmedSurfaces.h | Create standalone NURBS surfaces and planar Brep faces |
| [offset](offset.md) | `offset_brep`, `shell_brep`, `create_offset_profile` | SmApiBrep.h, SmApiPrimitives.h | Offset surfaces, hollow out solids, offset planar curve loops |
| [stitching](stitching.md) | `stitch_into_solid`, `stitch_into_shell`, `unify_normals`, `stitch_advanced`, `stitch_brep`, `stitch_brep_with_face`, `simple_face_stitch` | SmApiBrep.h | Join disconnected faces into solids/shells |
| [intersectors](intersectors.md) | `intersect_breps`, `intersect_brep_with_plane`, `intersect_curves`, `intersect_surfaces`, `intersect_faces`, `intersect_curve_surface`, `intersect_curve_brep`, `intersect_curve_face` | SmApiIntersectors.h | Compute geometric intersections |
| [transforms](transforms.md) | `translate`, `rotate`, `scale`, `scale_about_point`, `transform` | SmApiGeneral.h | Reposition and resize geometry |
| [concurrency](concurrency.md) | `SmApiCreateContext`, `SmApiGetOrCreateContext` | SmApiGeneral.h | Context lifetime, object-owned caches, and process-per-worker concurrency (operations run single-threaded in the shipped build) |
| [queries](queries.md) | `is_manifold_solid`, `compute_volume`, `brep_bounding_box`, `brep_copy`, `face_bounding_box`, `face_area`, `edge_bounding_box`, `edge_length`, `vertex_point`, `brep_closest_point`, `brep_closest_point_all`, `curve_closest_point`, `curve_closest_point_all`, `surface_closest_point`, `get_closest_point`, `get_edges_via_api`, `get_faces_via_api`, `find_edge`, `find_face`, `evaluate_curve`, `evaluate_surface_point`, `evaluate_surface_normal`, `evaluate_surface_derivatives`; methods `Brep.{info,bounding_box,center,volume,is_manifold_solid,copy,face_count,edge_count,vertex_count}`, `Face.{bounding_box,area,surface,uv_domain,edges,vertices,outer_loop_edges,boundary_loops,boundary_outer_loop,point_at_uv,normal_at_uv,outward_normal_at_uv,brep}`, `Edge.{bounding_box,length,curve,start_vertex,end_vertex,vertices,other_vertex,faces,point_at,brep}`, `Vertex.{point,edges,faces,brep}`, `Curve.{parameter_range,start_point,end_point,evaluate,length,is_closed,is_planar}`, `Surface.{u_domain,v_domain,uv_domain,evaluate,normal,derivatives,is_planar,is_periodic_u,is_periodic_v}` | SmApiQueries.h, SmApiSurfaces.h, SmApiCurves.h | Validate, measure, evaluate stable topology queries |
| [dev_topology](dev_topology.md) | `_smlib_dev`: `topology_pick_ray`, `edgeuses`, `loopuses`, `loops`, `edgeuse_of_face`, `loop_of_face`, `face_loops`, `assert_valid`, `dump`, `user_test`, `user_test_example`, `draw.extract_draw_batches`, `tests.*`; types `Loop`, `Loopuse`, `Edgeuse` | SmTopologySolver.h, SmDraw, `_smlib_tests` | Debugger topology pick, loopuse/edgeuse walks, AssertValid, Dump, kernel-direct user tests, Draw extraction, in-process prog_test |
| [cut_project_trim](cut_project_trim.md) | `cut`, `project_brep_onto_plane`, `project_and_trim`, `create_silhouette_curves`, `project_curve`, `trim_surface_with_3d_curves`, `trim_project_parallel` | SmApiBrep.h, SmApiTrimmedSurfaces.h | Cut, project, and trim operations |
| [tessellation](tessellation.md) | `tessellate`, `tessellate_boundaries` | SmApiBrep.h | Convert BReps to polygon meshes or sample boundary edges |
| [polygon_booleans](polygon_booleans.md) | `poly_boolean`, `poly_boolean_union`, `poly_boolean_difference`, `poly_boolean_intersection`, `poly_boolean_merge` | SmApiPolygons.h | Boolean operations on polygon meshes |
| [polybrep](polybrep.md) | `PolyBrep`, `PolyFace`, `PolyEdge`, `PolyVertex`, `poly_brep_is_manifold_via_api`, `poly_brep_volume_via_api` | SmPoly*, SmApiPolygons.h | Inspect and measure tessellated / imported meshes |
| [heal_import](heal_import.md) | `heal_brep` | SmApiHeal.h | Attempt supported BRep repairs |
| [interactive_performance](interactive_performance.md) | `mass_properties`, `volume`, `bounding_box`, `tessellate`, `translate`, `rotate`, `transform`, `boolean_difference` | SmApiQueries.h, SmApiBrep.h, SmApiGeneral.h | Preview-vs-exact API selection and cache reuse: property preview levers, tessellation quality knobs, rigid-transform reuse, repeated cutters (not kernel tessellation cost or context/concurrency) |

## SM_API_USD Operations

| Guide | Python Functions | C API Header | Description |
|---|---|---|---|
| [usd_import_export](usd_import_export.md) | `usd.ensure_plugin_registered`, `usd.import_breps`, `usd.import_brep`, `usd.export_brep`, `usd.export_breps`, `usd.append_breps`, `usd.import_meshes`, `usd.import_mesh`, `usd.export_mesh`, `usd.export_meshes`, `usd.append_meshes` | SmApiUsd.h | Round-trip BRep (`BrepArray`) and polygon meshes (`UsdGeomMesh` ↔ `PolyBrep`) |

## See Also

- **[`.agents/docs/errors.md`](../docs/errors.md)** — decoding `RuntimeError` from any binding: every `SM_ERR_*` code with typical causes and remedies, reading the kernel trace, operation-specific guidance for `pipe_sweep`, booleans, fillets, and tessellation.

## Typical Workflow Order

1. **Create** — primitives, curves, surfaces
2. **Shape** — sweeps, booleans, cut, trim
3. **Refine** — fillets, offset/shell
4. **Validate** — is_manifold_solid, compute_volume, closest_point queries
5. **Export** — USD export, tessellation (`tessellate` → `PolyBrep`)

## Python Binding Coverage

The `_omni_solid` Python module provides near-complete coverage of stable SM_API modeling operations. **Tessellation** (`tessellate` / `SmApiTessellate`) and **polygon mesh booleans** (`poly_boolean*` / `SmApiPolyBoolean*`) are wrapped; **`PolyBrep`** supports topology traversal, bounding box, mass properties, plane section, and ray intersection (see **[polybrep](polybrep.md)**).

Debugger-only surface (loopuse/edgeuse walks, topology pick, kernel Draw, in-process `prog_test`) lives in a separate **`_smlib_dev`** module — see **[dev_topology](dev_topology.md)**.

The stable `_omni_solid` module exposes:

- **Enums:** `BooleanOp`, `BooleanOp2D`, `PolyBooleanOp`, `FilletXSect`, `TrimType`, `ContinuityType`
- **Types:** `Vec3`, `Vec2`, `Brep`, `PolyBrep`, `PolyFace`, `PolyEdge`, `PolyVertex`, `Face`, `Edge`, `Vertex`, `Curve`, `BSplineCurve`, `Line`, `Circle`, `Surface`, `Object`, `Axis2Placement`
- **Topology:** `brep.faces()`, `brep.edges()`, `brep.vertices()`; on `PolyBrep`: `get_faces()`, `get_edges()`, `get_vertices()`, `get_points()` (see **[polybrep](polybrep.md)**)
- **Properties on geometric types:**
  - `Brep`: `.info()`, `.bounding_box(tight=False)`, `.center()`, `.volume(relative_accuracy=1e-3)`, `.is_manifold_solid()`, `.copy()`, `.face_count()`, `.edge_count()`, `.vertex_count()`
  - `Face`: `.bounding_box(tight=False)`, `.area(relative_accuracy=1e-3)`
  - `Edge`: `.bounding_box(tight=False)`, `.length(desired_accuracy=1e-6)`
  - `Vertex`: `.point()`
  - `Curve`: `.parameter_range()`, `.start_point()`, `.end_point()`, `.evaluate(t)`, `.length(desired_accuracy=1e-6)`, `.is_closed(tolerance=0.0)`, `.is_planar(tolerance=SM_EFF_ZERO)`
  - `Surface`: `.u_domain()`, `.v_domain()`, `.uv_domain()`, `.evaluate(u, v)`, `.normal(u, v)`, `.derivatives(u, v)`, `.is_planar(tolerance=SM_EFF_ZERO)`, `.is_periodic_u()`, `.is_periodic_v()`

  Brep / Face / Edge / Vertex methods wrap `SmApiQueries.h` entry points and are also exposed as flat aliases under `sm.queries.*` (`brep_bounding_box`, `face_area`, `edge_length`, `vertex_point`, `brep_copy`, etc.). Surface evaluators wrap `SmApiSurfaces.h` (`SmApiEvaluateSurfacePoint` / `Normal` / `Derivatives`) and have flat aliases `sm.evaluate_surface_point` / `_normal` / `_derivatives`. The remaining Curve / Surface predicates (`is_planar`, `is_periodic_*`, `parameter_range`, `length`, ...) are method-only — they call the kernel directly. See **[queries](queries.md)**.

- **Topology accessors on stable geometric types** (walk the BRep graph without loopuse/edgeuse):
  - `Face`: `.surface()`, `.uv_domain()`, `.edges()`, `.vertices()`, `.outer_loop_edges()`, `.boundary_loops()`, `.boundary_outer_loop()`, `.point_at_uv(u, v)`, `.normal_at_uv(u, v)`, `.outward_normal_at_uv(u, v)`, `.brep()`
  - `Edge`: `.curve()`, `.start_vertex()`, `.end_vertex()`, `.vertices()`, `.other_vertex(v)`, `.faces()`, `.point_at(t)`, `.brep()`
  - `Vertex`: `.edges()`, `.faces()`, `.brep()`

  Face boundary methods expose ordered edge occurrences and direction bits without stable use objects. For direct `Loop` / `Loopuse` / `Edgeuse` objects, `topology_pick_ray`, and kernel Draw extraction, see **[dev_topology](dev_topology.md)**.

- **Type stubs (`.pyi`):** `source/SmPyLib/stubs/usd_brep/__init__.pyi` and `source/SmPyLib/stubs/usd_brep/usd.pyi` for stable API; `source/SmPyDevLib/stubs/smlib_dev/__init__.pyi` for debugger API.
