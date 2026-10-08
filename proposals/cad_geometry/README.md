# Boundary Representation Geometry in OpenUSD

Copyright &copy; 2026, Alliance for OpenUSD (AOUSD), Geometry Working Group - version 0.1 (DRAFT)

## Contents

- [Introduction](#introduction)
- [Motivation](#motivation)
  - [USD geometry today](#usd-geometry-today)
  - [The expanding ecosystem](#the-expanding-ecosystem)
- [Problem statement](#problem-statement)
  - [Two distinct kinds of geometry](#two-distinct-kinds-of-geometry)
  - [Why this matters now](#why-this-matters-now)
- [Industry use cases](#industry-use-cases)
  - [Hero use case 1: Measurement and clash detection in facility planning](#hero-use-case-1-measurement-and-clash-detection-in-facility-planning)
  - [Hero use case 2: Consumer-controlled data quality](#hero-use-case-2-consumer-controlled-data-quality)
  - [Additional use cases by domain](#additional-use-cases-by-domain)
  - [Architecture, Engineering, Construction, and Operations (AECO)](#architecture-engineering-construction-and-operations-aeco)
  - [Manufacturing and Product Engineering (MFG)](#manufacturing-and-product-engineering-mfg)
  - [Robotics and Simulation](#robotics-and-simulation)
- [Existing mechanisms in USD](#existing-mechanisms-in-usd)
  - [UsdGeomMesh and subdivision surfaces](#usdgeommesh-and-subdivision-surfaces)
  - [UsdGeomNurbsCurves and UsdGeomNurbsPatch](#usdgeomcurves-and-usdgeomnurbspatch)
  - [UsdVol and the opaque-data precedent](#usdvol-and-the-opaque-data-precedent)
  - [Analytic primitives](#analytic-primitives)
- [Design considerations](#design-considerations)
  - [Principles](#principles)
  - [Open questions for discussion](#open-questions-for-discussion)
  - [Likely direction](#likely-direction)
  - [Risks](#risks)
- [Relationship to other proposals](#relationship-to-other-proposals)
- [Next steps](#next-steps)
- [Appendix A: BRep Glossary](#appendix-a-brep-glossary)
- [Appendix B: AI-Assisted Drafting](#appendix-b-ai-assisted-drafting)

---

## Introduction

USD has no mechanism for representing mathematically exact geometry.
Meshes approximate. Subdivision surfaces approximate. NURBS patches come
closest but have only limited trimming support, no topological
connectivity, and single-precision control vertices. Analytic primitives
are exact but limited to a handful of canonical shapes.

The dominant interchange representation for exact geometry in industry
is the **boundary representation (Brep)**: a solid described by its
bounding surfaces, edges, and vertices with explicit topological
relationships. Manufacturing, AEC, simulation, and robotics workflows
depend on Breps, and none of USD's current types can express them.

As USD moves into these industries, that gap is becoming a bottleneck.
This document makes the case for adding Brep support. It separates the
concern of **boundary-representation geometry** from the **display
geometry** USD already handles well, and lays out requirements that any
proposed Brep schema should satisfy. It does not propose a schema
itself; the companion proposal from the AOUSD Geometry Working Group
presents the technical design.

**Expected outcome.** Shared agreement on four points: (1) why Brep
support is needed, (2) what workflows it unblocks, (3) why existing USD
mechanisms cannot fill the role, and (4) what design principles should
guide the schema work.

## Motivation

### USD geometry today

USD ships four families of geometric primitives, all designed with visual
content in mind:

- **Polygonal meshes** (`UsdGeomMesh`) approximate curved surfaces with
  planar facets. Tessellation density controls fidelity, trading accuracy
  for performance. Meshes are the dominant geometry type in USD and in
  real-time rendering generally.

- **Subdivision surfaces** (also `UsdGeomMesh`, with a subdivision scheme)
  produce a smooth limit surface at render time. The smoothness hides the
  faceted control cage, but the limit surface is still defined by that
  cage-not by the mathematical shape the author intended.

- **NURBS curves and patches** (`UsdGeomNurbsCurves`, `UsdGeomNurbsPatch`)
  are the closest USD comes to parametric geometry. In practice they are
  under-specified for solid modeling: control vertices are single
  precision (`point3f`), and there is no way to express that two patches
  share an edge or form a closed shell. Trim curves are defined in the
  schema (following the RenderMan `RiTrimCurve` encoding) but have
  limited tool support in practice.

- **Analytic primitives** (`UsdGeomSphere`, `UsdGeomCylinder`,
  `UsdGeomCone`, `UsdGeomCapsule`, `UsdGeomCube`) are exact shapes, but
  the catalog is small and fixed. They carry no trimming, no
  parameterization metadata, and cannot be composed into solids.

For film, games, and most visualization workflows these primitives are
well suited. For industrial workflows that depend on mathematical
exactness, they are not.

### The expanding ecosystem

USD grew up in visual effects, where meshes and subdivision surfaces
are the native language. As USD expands into industrial sectors—CAD,
BIM, simulation, robotics, metrology—it encounters disciplines where
mathematical exactness is foundational. A cylinder in a CAD model is a
parametric surface with a known center, radius, and axis; a hole has a
canonical diameter; an assembly carries constraint relationships
between exact entities.

Today, industrial platforms are adopting USD primarily as a
visualization and collaboration layer. Siemens Teamcenter converts JT
files to USD meshes for photorealistic digital twins. PTC connects
Windchill to Omniverse for real-time rendering of Creo data. Trimble
exports SketchUp models as USDZ for 3D workflows. In every case, the
entry point is tessellated geometry—exact Brep data stays behind in
native formats because USD has no way to carry it.

This tessellation boundary is the core problem. The exact geometry is
the authoritative representation in these domains, and meshes are
derived artifacts. When the exact form is stripped at the USD
boundary, downstream consumers lose the ability to measure, simulate
against, or manufacture from the original design intent.

## Problem statement

### Two distinct kinds of geometry

Industrial workflows require two fundamentally different kinds of
geometry. USD currently supports only one:

| | **Display geometry** | **Exact geometry (Brep)** |
|---|---|---|
| **Purpose** | Visualization, rendering, real-time interaction | Design, simulation, measurement, manufacturing |
| **Representation** | Meshes, subdivision surfaces | Boundary representations (Breps), parametric surfaces, analytic shapes with trimming |
| **Precision** | Approximate - controlled by tessellation density | Mathematically exact within tolerance |
| **Topology** | Implicit - inferred from mesh connectivity | Explicit - surfaces know their neighbors, edges know their faces |
| **Derivation** | Derived from exact geometry (or authored directly) | Authoritative source |

These are not competing representations-they are complementary. A single
object in an industrial scene typically needs both: exact geometry for
engineering operations and display geometry for visualization. One is the
source; the other is a derived view that can be regenerated at whatever
resolution the consumer needs.

### Why this matters now

Several converging trends make this gap increasingly costly:

1. **Industrial platforms are converging on USD—but only for
   visualization.** When CAD or BIM data enters USD today, it is
   tessellated at the boundary: exact geometry stays in native
   formats and only mesh proxies reach the scene. AOUSD membership
   now spans CAD, AEC, and geospatial companies, yet none carry
   exact geometry through USD—they need to see what Brep support
   could enable before investing beyond visualization.

2. **Cross-discipline collaboration demands geometric fidelity.** An
   architect's BIM model, a mechanical engineer's CAD assembly, and a
   roboticist's simulation environment must all survive the round-trip
   through USD without degradation. Mesh-only pipelines break that
   promise at every discipline boundary.

3. **"Derive on demand" is replacing pre-tessellation.** Rather than
   choosing a tessellation density at export time and shipping large
   mesh files, workflows increasingly prefer to ship the compact exact
   geometry and tessellate at the point of consumption, where the target
   resolution and use case are known. This also inverts control:
   consumers choose the visual quality they need rather than inheriting
   whatever the exporter chose, and USD's ability to aggregate data
   from diverse sources extends to the geometry itself.

Today, each industrial adopter works around the gap independently:
embedding STEP files as opaque payloads, encoding Brep data in
`customData` dictionaries, or maintaining parallel pipelines where the
"real" geometry lives outside USD and a mesh proxy lives inside. Every
one of these workarounds forfeits composition, layering, and sparse
overrides-the capabilities that make USD worth adopting in the first
place-for the most important geometry in the scene.

## Industry use cases

The use cases below are drawn from the AOUSD Geometry Working Group's
cross-industry survey ("CAD and BIM Use Cases," v0.2, December 2024,
Thorsten Hertel and Alex Fuchs). Two cross-cutting scenarios appear most
frequently across domains and are developed first; the domain-specific
sections that follow elaborate on these and introduce additional
scenarios. In each case, the workflow either breaks outright or degrades
significantly when only mesh geometry is available.

### Hero use case 1: Measurement and clash detection in facility planning

A facility planner—assembling a factory floor, a warehouse, a
construction site, or a hospital wing—aggregates geometry from dozens of
sources into a single USD stage. The equipment suppliers publish
tessellated meshes; the planner has no access to the original CAD
models.

With only meshes, two problems arise:

- **Measurement is unreliable.** The planner needs the clearance between
  a robot base and its mounting surface, the center-to-center distance
  between bolt holes, or the gap between an HVAC duct and a structural
  beam. Tessellated geometry lacks parametric identity: a cylinder has
  no center axis, a hole has no canonical diameter, and the precision of
  any measurement is bounded by the tessellation tolerance—which the
  planner did not choose and may not know.

- **Clash detection is ambiguous.** When the measured distance between
  objects is on the order of the tessellation tolerance, it is impossible
  to distinguish a real intersection from a mesh artifact. The WG use
  cases document puts it directly: "If measured distance between objects
  is in the ballpark of the tolerance used for tessellation, it is
  unsafe to exclude a collision." Resolving these "soft clashes"
  requires either re-tessellating at prohibitive density or switching to
  exact geometry for a deterministic answer.

If a Brep were available alongside the mesh in the USD stage, the
planner could measure and check clearances against the authoritative
representation without returning to each supplier's native CAD system. This scenario recurs across AECO (clash detection across
disciplines, construction simulation), manufacturing (robot collision
checking, equipment layout), and digital twins (facility monitoring and
planning).

### Hero use case 2: Consumer-controlled data quality

Today, the person who exports geometry to USD chooses a tessellation
density—and that choice is final. Every downstream consumer receives the
same mesh, regardless of their actual needs.

This is a poor fit for multi-consumer workflows:

- A **visualization artist** wants to suppress small features (drill
  holes, threads) and apply texture maps. A coarse mesh suffices.
- A **structural engineer** needs full geometric fidelity for a
  close-up FEA mesh with boundary-layer refinement. The visualization
  mesh is too coarse.
- A **machining planner** needs the exact parametric surface for
  toolpath generation. No mesh is precise enough.

With a Brep in the stage, each consumer derives a fit-for-purpose
representation—mesh, analytic surface, or raw
parametric data—without modifying the source and without coordinating
with the original author. The WG use cases document frames this as the
"ability to work with one file, describing the part exactly, but being
able to derive use case dependent representation (meshes) of the part
with no data loss and reversibility."

This pattern appears wherever one authoring step feeds multiple
downstream consumers: product design feeding both rendering and
simulation, BIM models serving both visualization and code compliance,
and digital twins serving both monitoring dashboards and maintenance
planning.

### Additional use cases by domain

The hero use cases recur across domains. The WG survey identifies
additional scenarios that do not reduce to measurement or
re-tessellation:

- **AECO — reference model exchange.** Designers at different firms
  exchange building or infrastructure models for coordination.
  Exact geometry preserves dimensional accuracy across the exchange
  boundary; mesh-only exchange leaves tessellation tolerance unknown.
  Clash detection and construction simulation are instances of the
  facility-planning scenario above.

- **MFG — constraint data.** CAD assemblies encode mates, alignments,
  and kinematic constraints that reference exact geometric entities.
  A robot arm with joint constraints defined against exact surfaces
  can be automatically rigged; the same arm exported as mesh loses
  the constraint references entirely.

- **MFG — Product Manufacturing Information (PMI).** A threaded hole
  may be stored as a plain cylinder plus PMI metadata. Applications
  that operate on exact geometry can reconstruct the thread;
  visualization tools can texture-map the corresponding Brep faces.
  Both require the exact surface to be present and addressable.

- **Robotics — simulation preprocessing.** FEA, CFD, and multi-body
  dynamics each require domain-specific meshing from exact boundaries.
  A pre-tessellated mesh created for visualization cannot satisfy
  these requirements; shipping exact geometry eliminates the
  round-trip through the source CAD system.

- **Cross-domain — wireframe visualization and IP protection.** Not
  every review needs photorealistic rendering. Wireframe views
  derived from Brep edges can present simplified representations,
  emphasize structural features such as holes or fillets, or expose
  internal detail that a shaded surface hides. Because wire edges
  are first-class entities in the Brep topology, the wireframe is
  already present—no extra tessellation step is required. This mode
  also serves IP protection: stripping the analytic surfaces and
  retaining only the wireframe is a common first step when CAD users
  share geometry outside their organization. A Brep-aware USD stage
  can support both full-fidelity and wireframe-only representations
  of the same asset, controlled by the publisher.

## Existing mechanisms in USD

A natural question is whether USD's existing geometry types can be
stretched to carry boundary-representation geometry. None of them can,
but the reasons are instructive.

### UsdGeomMesh and subdivision surfaces

Meshes and subdivision surfaces approximate curved geometry with planar
polygons. For industrial use the limitations are fundamental: meshes
cannot be refined without the source geometry, they carry no solid
topology (adjacency, orientation, containment), and parametric
identity—center axes, radii, fillet profiles—dissolves into
undifferentiated triangles. Mesh-to-Brep reconstruction remains an
active research area but is not production-ready.

### UsdGeomNurbsCurves and UsdGeomNurbsPatch

USD's NURBS primitives can represent parametric surfaces in principle,
but fall short in practice:

- **Single precision.** Control points are `float3`; CAD kernels use
  double precision. The accumulated loss can exceed machining and FEA
  tolerances.
- **Limited trimming.** `UsdGeomNurbsPatch` defines trim curves
  (via legacy `RiTrimCurve` encoding), but there is no mechanism for
  expressing that trimmed patches form a closed solid.
- **No topological connectivity.** Individual patches are isolated;
  USD cannot express shared edges, closed shells, or solid regions.
- **No sub-surface composition.** Faces, edges, and vertices cannot be
  individually addressed for sparse overrides or `GeomSubset`
  assignment.

### UsdVol and the opaque-data precedent

USD volumes (`UsdVolOpenVBDAsset`) reference external OpenVDB files.
The data is opaque: it cannot be queried, overridden, or composed
through USD. Applying this pattern to Brep data—wrapping STEP files
or kernel blobs as opaque payloads—would surrender the capabilities
that justify adopting USD.

STEP (ISO 10303) is an established interchange format, but it is a
serialization format designed for file exchange on disk. It lacks the
in-memory data model needed for scene composition and cannot
participate in USD's layering, referencing, or override mechanisms.
It also has technical gaps: no support for non-manifold models (common
in simulation workflows) and no mechanism for attaching per-face or
per-region metadata. Embedding STEP as an opaque payload would inherit
these limitations while forfeiting USD's core strengths. Native
representation is what enables measurement, interference checking, and
simulation against exact geometry within USD.

### Analytic primitives

USD's analytic shapes (`UsdGeomSphere`, `UsdGeomCylinder`, etc.) are
mathematically exact but limited to a fixed catalog of canonical
forms. They lack trimming, topological connectivity, and composition
into solids—the defining features of a boundary representation.

## Design considerations

What follows distills the problem statement and use cases into
principles and open questions for the schema work. Many of these reflect
lessons the Geometry Working Group learned while developing its draft
Brep schema.

### Principles

1. **Native, not opaque.** Brep geometry must live in USD properties
   that participate in composition, layering, sparse overrides, and
   metadata attachment. The UsdVol pattern (opaque external files) is
   explicitly not the model.

2. **Coexistence with meshes.** Brep and Mesh representations of the
   same object should be relatable-so applications can choose which to
   operate on-but neither should require the other. How they relate
   (USD relationships, naming conventions, a companion API schema) is
   important but separable from the core Brep schema.

3. **Double precision for geometric data.** Control vertices, knot
   vectors, weights, and analytic parameters must be double precision.
   This is widely regarded as a prerequisite for industrial adoption.
   Display data
   (colors, texture coordinates) can remain single precision.

4. **Prim count discipline.** Industrial assemblies contain millions of
   topological entities. One prim per face or edge would be
   prohibitively expensive. The schema should pack topology within a
   manageable number of prims while still supporting per-element
   properties (e.g., via `GeomSubset`).

5. **Additive, not disruptive.** Applications that do not need Brep
   support should not be affected. The
   [USD Profiles](https://github.com/PixarAnimationStudios/OpenUSD-proposals/tree/main/proposals/profiles)
   proposal provides the mechanism for declaring which geometry
   capabilities an application supports.

6. **Grounded in proven theory.** The Radial Edge Data Model
   (Weiler, 1986) and the PRC/STEP geometry catalogs represent decades
   of industrial validation. A new USD schema should build on that
   foundation, not reinvent it.

7. **Ship early, iterate.** The full scope of CAD geometry is vast. A
   minimal first version-likely NURBS-based Breps covering the most
   common industrial cases-with a clear extension path is more
   valuable than a comprehensive design that ships late.

### Open questions for discussion

- **New domain or existing domain?** `UsdSolid` vs. extending `UsdGeom`.
  A new domain avoids burdening existing consumers; extending `UsdGeom`
  preserves interoperability. The WG draft uses `UsdSolid`.

- **Brep-to-Mesh relationship.** How should a Brep prim relate to its
  derived mesh(es)? Options include USD relationships, naming
  conventions, or a companion API schema.

- **Manifold and non-manifold.** Should the first version support both?
  Non-manifold models matter for simulation (e.g., mid-surface
  idealization) but add complexity.

- **Assembly vs. body.** Arrays of Breps in one prim (`BrepArray`)
  reduce prim count; one-prim-per-Brep simplifies referencing.

- **Tessellation responsibility.** USD-level plugin vs. application-level
  kernel tessellation. A built-in tessellator provides consistency but
  introduces a large dependency.

### Likely direction

The Geometry Working Group has already developed a detailed draft Brep
schema built on the Radial Edge Data Model. It addresses many of the
considerations above:

- A `UsdSolidBrepAPI` multiple-apply schema packs all topology and geometry
  for a single Brep into one API instance, applied to a `UsdSolidBrepArray`
  IsA schema.
- Geometry types are derived from the PRC specification (ISO 14739-1:2014),
  providing a standards-based catalog of curves, surfaces, and volumes.
- Double precision is used for geometric data.
- Both manifold and non-manifold Breps are supported.
- Per-element properties are assignable via `GeomSubset`.

The companion proposal presents this schema in detail. Nothing in the
present document endorses or precludes a specific schema design; the
goal is to establish requirements against which *any* proposed schema
can be evaluated.

### Risks

- **Scope creep.** Exact geometry covers a wide territory — Brep
  topology and a broad catalog of curve, surface, and volume types
  (analytic primitives, NURBS, blends, offsets, volumes, and more,
  per standards such as PRC ISO 14739-1:2014). The companion schema
  proposal targets the full catalog but presents detailed schema
  designs incrementally, beginning with NURBS curves and surfaces in
  the current revision and adding the remaining types in subsequent
  revisions.

- **Kernel dependency.** Tessellation, Booleans, and measurement require
  geometry kernels (Parasolid, ACIS, Open Cascade). The schema should
  not mandate a specific kernel, but the ecosystem needs at least one
  open-source tessellation path.

- **Evaluation cost.** Brep data is more compact than high-resolution
  meshes but more expensive to evaluate. The schema should let
  applications choose between the Brep and a cached mesh.

- **Adoption chicken-and-egg.** Reference implementations (exporters, a
  tessellator, Hydra adapters) and early-adopter engagement are
  essential.

- **Core-team alignment.** Brep support introduces requirements from
  domains Pixar has not historically served. Close collaboration
  between the WG and Pixar's team is needed.

## Relationship to other proposals

- **Geometry Working Group Brep schema** (`proposals/UsdSolid/README.md`
  in this repository). The companion schema proposal contains the
  technical design; this document provides the motivation and
  requirements. They are meant to be reviewed together but each can
  stand on its own.

- **USD Profiles** ([OpenUSD-proposals: profiles](https://github.com/PixarAnimationStudios/OpenUSD-proposals/tree/main/proposals/profiles)).
  Profiles let applications declare which USD capabilities they
  support. A Brep-capable profile would let tools advertise exact-
  geometry support without imposing it on tools that do not need it.

- **Identifier Separation of Concerns** ([OpenUSD-proposals #105](https://github.com/PixarAnimationStudios/OpenUSD-proposals/pull/105)).
  That proposal uses a similar "frame the problem first, then evaluate
  solutions" methodology. The two proposals are substantively
  independent.

- **BRep Glossary** ([Appendix A](#appendix-a-brep-glossary)).
  The Geometry Working Group's shared glossary, including entity
  mappings across major geometry kernels. This proposal follows its
  conventions.

## Next steps

1. **Review this problem statement.** Gather feedback on framing, use
   case coverage, and design principles from the broader OpenUSD
   community.

2. **Evaluate the Brep schema against these requirements.** Once the
   problem statement is accepted, review the Geometry Working Group's
   schema proposal through the lens of the principles and open
   questions above.

3. **Build a reference implementation.** A minimal package-example
   schema in `extras/`, a basic tessellator, a STEP-to-USD
   converter-demonstrates that the schema is implementable and
   useful. The Gaussian Splats landing (OpenUSD PR #3716) is a good
   precedent for shipping schema, Hydra adapter, and conversion tools
   together.

4. **Engage early adopters.** Validate the schema against real
   production data from CAD vendors, BIM tool developers, and
   simulation platforms.

5. **Ship and iterate.** An imperfect first version that ships is more
   valuable than a comprehensive design that doesn't. The industry is
   already working around USD's limitations; the sooner there is a
   standard path, the sooner the workarounds can converge.

## Appendix A: BRep Glossary

### BRep

A BRep (Boundary Representation) is a data model used to precisely
represent 3D objects by defining their boundaries with geometry and
topology. Rather than representing the entire volume of an object, a
BRep defines only the boundary surfaces, edges, and vertices that
separate regions of space. A BRep is composed of regions. BReps can
represent isolated points, wires, surface sheets, and solids with
well-defined interiors and exteriors.

### Curve

A geometric entity that maps points on a closed interval of the real
line into 3D space. In a BRep, curves provide the underlying geometry
for edges, with each edge referencing a specific bounded portion of a
curve defined by parameter range. Common curve types include lines,
circles, ellipses, and NURBS. The same curve may be shared by multiple
edges that reference different parameter ranges.

### Edge

A topological entity representing a bounded portion of a curve. Each
edge references its underlying geometric curve and the parameter range
that defines which portion of the curve is used, along with start and
end vertices. The curve subset defined by the edge may not be
self-intersecting.

### Edgeuse

A structure indicating how a loop uses each edge comprising it. Each
edgeuse is owned by a loop and references a single edge, with an
orientation flag indicating traversal direction relative to the
underlying curve. Each edgeuse also has an associated trim curve that
maps the edge into the face's surface parameter space. Edgeuses are
conceptually related to "coedges," "fins," "links," or "winged edges"
in various BRep implementations.

### Face

A bounded and connected set of points on a single G1-continuous
surface. Faces must be bounded by loops, with one outer loop defining
the external boundary and optional inner loops representing holes or
voids within the face.

### Faceuse

A structure indicating how a shell uses a face. Each faceuse is owned
by a shell and references a single face, with an orientation flag
indicating whether the shell uses the face in the same direction as the
surface normal. Faceuses are conceptually related to "sides" in various
BRep implementations.

### Geometry

The geometric shape data that defines the precise locations and forms of
boundaries in a BRep. Geometry includes points (vertex locations),
curves (edge shapes), and surfaces (face shapes). Geometry provides the
"shape" aspect of a BRep, while topology provides the "connectivity"
aspect.

### Loop

A closed sequence of connected edges, where each edge is referenced
indirectly by an edgeuse. Loops serve as boundaries for faces. An outer
loop defines the external boundary of a face, while inner loops
represent holes or voids. Loop orientation (winding order) is
significant for determining which side of the boundary is interior.

### Manifold

A topological property of a BRep where each point on the boundary has a
well-defined local neighborhood. In manifold BReps, each edge is shared
by exactly two faces. Non-manifold BReps may have edges shared by more
than two faces (spine edges), edges connected to only one face (laminar
edges), or vertices where multiple disconnected faces meet.

### Region

A connected subset of 3D space that classifies points as being either
"in" or "out." Regions are owned by BReps and composed of shells. The
first shell of a region defines the outer boundary; additional shells
represent internal boundaries. Some BRep implementations refer to
regions as "lumps."

### Shell

A maximal connected set of faces and wires. Shells are owned by regions
and organize the boundary topology. A shell can be closed (forming a
complete boundary) or open (with free edges).

### Solid

A region of 3D space with a well-defined interior and volume, bounded by
one or more closed shells. The outermost shell defines the exterior
boundary; inner shells represent voids or cavities. For a solid to be
well-defined, its boundary shells must be watertight.

### Surface

A continuous mapping from 2D parameter space (u, v) to 3D space (x, y,
z). In a BRep, surfaces provide the underlying geometry for faces.
Common surface types include planes, cones, spheres, cylinders, tori,
Bezier surfaces, and NURBS surfaces.

### Topology

The connectivity and relationship data that defines how boundary
elements are organized and connected in a BRep. Topology includes the
structural entities: BReps, regions, shells, faces, loops, edges, and
vertices, along with the "use" structures (faceuses, edgeuses) that
define orientations and connections.

### Trim Curve

A trim curve maps points in 2D space to surface parameters (u, v). Trim
curves provide the 2D geometry associated with edgeuses, defining how
edges bound faces in the face's surface parameter space. Trim curves
enable faces to represent bounded regions of surfaces, allowing complex
trimmed shapes beyond simple rectangular parameter regions.

### Vertex

A topological entity representing a point in 3D space. Vertices mark
the endpoints of one or more edges. In practice, vertices may
incorporate geometric tolerances.

### Wire

A connected set of edges forming a one-dimensional topological
structure. Unlike loops that bound faces, wires exist independently as
standalone curve features in a BRep model.

### Entity mapping across geometry kernels

| USD (AOUSD) | Parasolid | ACIS | Granite (PTC) | CGM (CATIA) | ShapeManager (Autodesk) | Open CASCADE |
|:--|:--|:--|:--|:--|:--|:--|
| BRep | Body | Body | — | Body | Body | TopoDS_Compound |
| Region | Lump | Lump | — | Lump | Lump | TopoDS_Solid |
| Shell | Shell | Shell | — | Shell | Shell | TopoDS_Shell |
| Face | Face | Face | Face | Face | Face | TopoDS_Face |
| Loop | Loop | Loop | Loop | Loop | Loop | TopoDS_Wire |
| Edge | Edge | Edge | Edge | Edge | Edge | TopoDS_Edge |
| Edgeuse | Fin | Coedge | — | — | Coedge | TopExp_Explorer |
| Faceuse | Side | Side | — | — | Side | Orientation |
| Vertex | Vertex | Vertex | Vertex | Vertex | Vertex | TopoDS_Vertex |

## Appendix B: AI-Assisted Drafting

Portions of this document were drafted with the assistance of an AI
language model (Claude, Anthropic) under author direction. The document
structure follows the methodology established in
[PR #105](https://github.com/PixarAnimationStudios/OpenUSD-proposals/pull/105)
(Identifier Separation of Concerns): frame the problem, survey existing
mechanisms, establish design principles, then evaluate solutions. The AI
helped with:

- Structuring the document following that methodology
- Distilling the Geometry Working Group's use cases document into
  problem-statement form
- Surveying existing USD geometry mechanisms against the requirements
  identified here
- Turning author outlines and technical notes into structured prose

All technical claims, design positions, and editorial judgments were
reviewed and approved by the author. This disclosure is made in the
interest of transparency.
