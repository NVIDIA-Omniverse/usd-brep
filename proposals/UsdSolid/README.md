![Status:Draft](https://img.shields.io/badge/Draft-blue)

# Solid Models in USD

This proposal introduces a Boundary Representation (Brep) schema to OpenUSD, enabling precise solid model geometry for CAD, CAM, CAE, and related industrial workflows.
The proposed schema implements the Radial Edge Data Model and supports manifold solids, non-manifold solids, sheet bodies, and wireframes.
It is designed to complement existing USD geometry types, bringing exact-geometry capabilities required by manufacturing, robotics, architecture, engineering, and construction (AEC) pipelines that cannot be met by mesh or subdivision surface representations alone.

---

## Contents

- [Introduction](#introduction)
  - [Proposed approach](#proposed-approach)
- [Glossary](#glossary)
- [Design](#design)
  - [Shape](#shape)
  - [Topology and "use"](#topology-and-use)
  - [Brep](#brep)
  - [USD implementation](#usd-implementation)
    - [UsdSolidBrepArray](#usdsolidbreparray)
    - [Instancing of Brep models](#instancing-of-brep-models)
    - [Brep geometry in USD](#brep-geometry-in-usd)
    - [Geometry type extensions](#geometry-type-extensions)
    - [Modeling Breps on a UsdStage](#modeling-breps-on-a-usdstage)
    - [Trimming curves](#trimming-curves)
  - [Flexible design possibilities](#flexible-design-possibilities)
    - [One Brep per BrepArray](#one-brep-per-breparray)
    - [One Assembly per BrepArray](#one-assembly-per-breparray)
    - [One Model per BrepArray](#one-model-per-breparray)
  - [Other implementations considered](#other-implementations-considered)
    - [One UsdPrim per geometry object](#one-usdprim-per-geometry-object)
    - [One UsdPrim per Brep](#one-usdprim-per-brep)
    - [Breps as an Applied API](#breps-as-an-applied-api)
    - [Breps as a black box](#breps-as-a-black-box)
  - [Assemblies](#assemblies)
  - [Tolerance](#tolerance)
  - [Validation](#validation)
    - [Rules and requirements](#rules-and-requirements)
  - [Risks](#risks)
  - [Open questions](#open-questions)
- [Schema](#schema)
- [Examples](#examples)
  - [Cube](#cube)
  - [Non-manifold cubes](#non-manifold-cubes)
  - [Cube with internal void](#cube-with-internal-void)
  - [BrepArray with multiple Breps, individual colors](#breparray-with-multiple-breps-individual-colors)
  - [Brep with materials applied to faces](#brep-with-materials-applied-to-faces)
- [References](#references)

---

## Introduction

The companion [problem statement](../cad_geometry/README.md) establishes why OpenUSD needs native Brep support and what design principles should guide the schema work. This proposal presents the concrete schema design: a robust and flexible Brep representation for OpenUSD, developed by the AOUSD Geometry Working Group. Community feedback, exploratory datasets, and downstream integration experiments are encouraged.

This proposal does not impose new requirements on OpenUSD stakeholders who do not work with Brep data; applications that do not need Brep support are unaffected. The broader question of how applications declare and discover which USD capabilities they support is being explored in the [USD Profiles](https://github.com/PixarAnimationStudios/OpenUSD-proposals/tree/main/proposals/profiles) proposal.

### Proposed approach

The schema implements a solid-model Boundary Representation (Brep) built on the Radial Edge Data Model ([Weiler, 1986](https://webserver2.tecgraf.puc-rio.br/~lfm/teses/KevinWeiler-Doutorado-1986.pdf)). For background on why this model was chosen over alternatives, see the companion problem statement's [design principles](../cad_geometry/README.md#principles).

In support of the model, this proposal also introduces additional curve, surface, and volume geometry types derived from the Product Representation Compact (PRC) format (ISO 14739-1:2014). The [Shape](#shape) section below catalogs the curve and surface primitives _UsdSolid_ is intended to support. Detailed designs of the additional geometry types are not yet included in this document and will be added in a subsequent revision.

## Glossary

For definitions of BRep, Curve, Edge, Edgeuse, Face, Faceuse, Geometry, Loop, Manifold, Region, Shell, Solid, Surface, Topology, Trim Curve, Vertex, and Wire, see [Appendix A: BRep Glossary](../cad_geometry/README.md#appendix-a-brep-glossary) in the companion problem statement. The following additional terms are specific to this proposal's schema:

**BrepArray** -- A `UsdSolidBrepArray` prim that holds one or more Breps in a packed, flat representation. Each Brep's topology, geometry, and metadata are concatenated into shared arrays and delineated by per-Brep counts and offsets. See [USD implementation](#usd-implementation).

**Wire edge** -- A `wireEdge` is a topologically standalone edge that is not part of any face loop. Wire edges carry the same curve geometry as loop-bounding edges but exist independently as one-dimensional features within a shell. They are represented in the schema by `wireEdge:*` arrays.

## Design

The following subsections introduce the Brep data model concepts that inform the schema design. Readers already familiar with Brep topology from the companion [problem statement](../cad_geometry/README.md) may wish to skip to [USD implementation](#usd-implementation).

Solid models rigorously partition space into regions by connecting sets of surfaces into region boundaries. Regions are the set of points that can be connected by curves of any shape that do not cross boundaries. The boundaries between regions must be watertight to prevent the points of each region from being connectable to one another. Manifold solid objects partition space into one solid and one or more void regions, classifying every point in space as either inside or outside the solid. A solid is manifold if for all points on the boundary there exists a neighborhood that is homeomorphic to a two-dimensional disk. A Brep that is not manifold is called "non-manifold." Non-manifold objects can partition space into one to any number of regions, where every point in space classifies to one of the model's regions.

In geometric modeling, where mathematical approximations of shape are common, gaps between adjacent surfaces are unavoidable. In the Radial Edge Data Model, the connections of adjacent surfaces are explicit objects that can fill these gaps and create the necessary partition of space.

Several Brep models were considered as options for the base of the _UsdSolid_ schema. The Radial Edge Data Model was chosen because, in addition to standard manifold modeling, it offers a robust representation of non-manifold modeling. Weiler's model was the first complete non-manifold Brep to explicitly represent topological adjacencies (Lee, 1999). The topology models in PRC, STEP, Parasolid, and others map into the proposed Radial Edge Data Model. Concepts from both PRC and STEP are used in this proposal, including all of the Brep geometry types in PRC and the volumes concept from STEP. As in PRC and STEP, this design supports wire-frame models.

The proposed model is composed of three parts: shapes, topology objects, and special connectivity objects called "uses." Because limiting _UsdPrim_ count is good practice in general and essential in large-scale scenes, the _UsdSolid_ design utilizes a _UsdSolidBrepAPI_ multiple-apply schema that can be applied to a _UsdSolidBrepArray_ IsA schema. Each instance of the _UsdSolidBrepAPI_ contains all the shape, topology, and connectivity data of a single Brep, plus metadata such as material bindings and a local transform.

### Shape

In practice, Breps are a large class of shape functions built on parametric mappings.
Today, this includes NURBs curves and surfaces, analytics, and is extendable to helices, offsets and more.
USD currently supports the standard NURBS curves, _UsdGeomNurbsCurves_, and NURBS surfaces, _UsdGeomNurbsPatch_.
Further, USD has analytic surfaces _UsdGeomCone, UsdGeomCylinder, UsdGeomPlane, UsdGeomSphere_.

The existing USD surface primitives have some issues relative to their use with Breps.
First, the NURBS curves and surfaces do not support double precision control vertices.
This is necessary for USD to be accepted as a reference CAD format, so the proposed schema includes double precision geometry.
Second, the _UsdGeomCylinder_ surface includes its end caps.
Also the _UsdGeomCone_ surface often includes an end cap in practice, but does not address caps in the documentation.
For full flexibility, solid models require that the analytics do not include end caps.
Third, the parameterization of the sphere, cone, cylinder, plane and volume are currently unspecified.
In order to trim them properly for solid modeling the analytics will need parameterizations and double precision.
Last, the CAD community uses a larger set of analytic surfaces than currently supported in USD.

In this proposal, each geometry type is defined by a set of attributes that reside within the _UsdSolidBrepAPI_ schema. The geometry is packed within like types, as in the _UsdGeomNurbsCurves_ class. Where necessary, the geometry will have double-precision attributes — for example, NURBS curves have double-precision control vertices, weights, and knots. Analytic geometry data will include both the analytic definition and the parameterization — for example, a sphere will have a radius and also a frame of reference that defines the parameterized surface origin, orientation, beginning, and end. The PRC parameterizations are recommended for analytic geometries. Trimming curves are also part of the _UsdSolidBrepAPI_ schema, packed as the 3D curves are.

For a complete set of geometry, the target is to meet the PRC standard (ISO 14739-1:2014). Achieving this will require an extensive list of attributes for the _UsdSolidBrepAPI_. The following curves and surfaces are needed to match the PRC specification. _Volume_, _CurveInVolume_, and _SurfInVolume_ types are also included for anticipated future uses. New geometry will be added to the _UsdSolidBrepAPI_ schema.


| Curves | Surfaces | Volumes |
| --- | --- | --- |
| Blend02Boundary | Blend01 | Volume |
| Circle | Blend02 | |
| Composite | Blend03 | |
| OnSurf | Cylindrical | |
| Ellipse | Offset | |
| Equation | Ruled | |
| Helix01 | Revolution | |
| Hyperbola | Extrusion | |
| Intersection | FromCurves | |
| Line | Torus | |
| Offset | Transform | |
| Parabola | SurfInVolume | |
| PolyLine | | |
| Transform | | |
| ProjectedCurve | | |
| CurveInVolume | | |



### Topology and "use"


When a closed set of curves lay on a surface, it can be used to trim the surface to that set of boundary curves and this results in a trimmed surface.
If two surfaces share the same segment of a boundary, this is called an edge and the two surfaces are neighbors.
If one has a way to keep track of which surfaces share an edge, then they have added a topology to our modeling system.


A trimmed surface, together with the information about its neighbors is referred to as a face.
A face must have an outer boundary and it may have many inner boundaries or "holes."
A shell is a collection of connected faces.


If a shell is closed then you have a solid.
A solid has an outer closed shell and possibly many inner shells that define cavities in the solid.
A "region" encloses space from a closed outer shell or between two closed shells (one inside the other) and has volume.
The outer region is the infinite region, so a closed box is represented by two regions - the outer infinite region, and the region within the box.
If there is a box within the box ( a 'thick' box) then there are 3 regions.
The inner box may 'float in space' without being attached to the outer box; it may not be physically possible, but it is topologically acceptable.


In a closed solid each boundary edge is shared by two neighboring faces (or connected to one face twice, called a seam edge), so this is referred to as a boundary representation or "Brep".
The first implementations of a Brep model were manifold, that is they allowed for only two faces to share an edge, hence the name "twin-edge boundary representation" or Brep.
If more than two faces can share an Edge, the topology is non-manifold.


It turns out that the manifold restriction that an edge may have only two neighbors is very limiting.
Any time you intersect two surfaces, if you look in the neighborhood of a point on the intersection curve, there appear to be four neighboring surfaces at that point.
That's what is meant by being "non-manifold".
A manifold edge is restricted to having two neighboring surfaces and a non-manifold edge may have more than two surfaces that share that edge.


Each boundary of a face consists of a closed loop of edges.
In the topology structure each loop has a list of edges, and the same edge may be used by two or more faces and that's what "EdgeUse" refers to.
The term edgeuse has real importance since there could be many uses of an edge.


If one constructs a box (it defines a region) and then adds another box right next to it, then they have another region.
Since they share the same face, you can see why "FaceUses" are necessary.
The face is used on one region, and the face is also used in the other region.


### Brep

The key idea of Brep modeling is that simple trimmed-shapes connect together through their boundaries to form complex geometry models just as a set of small glass pieces welded together along their edges form a stain glass window.


Shapes are points, curves, and surfaces each of which is a simply connected point set within a 3D space.
Shapes can be infinite (planes, cylinders, lines and such) or finite (Bspline curves and surfaces) but have no sense of boundaries.
For every shape there is a simple topology object that adds trimming to the shape so that it can be connected into a Brep model.
The simple topology objects are vertices, edges, faces, and regions.
Important combinations of simple topology objects forming key boundaries within a geometry model are also represented explicitly; a loop is any closed sequence of connected edges used to bound a face and a shell is any set of faces connected edge-to-edge to bound a region.


A topology object is "used" each time it connects to the geometry model to form a boundary.
In general, all of the simple topology objects participate in a hierarchy of boundaries: regions are bounded by faces (gathered into connected shells), faces are bounded by edges (gathered into connected loops), and edges are bounded by vertices.


Boundaries are used to establish neighbor relations.
Topology objects don't connect directly to their neighbors; rather neighbor relationships are created when two topology objects both use a common boundary.


A geometry model represented as a hierarchy of boundary connections is referred to as a boundary representation or "Brep" model.


Each use of a topology object (that can be used more than one time in one model) is represented by a specialized use object.
Each use of a face to bound a region is represented by a Faceuse.
Each use of a loop to bound a face is represented by a Loopuse.
Each use of an edge to bound a face is represented by an Edgeuse, and each use of a vertex to bound an edge is represented by a Vertexuse.
There are no Shelluse objects because shells are used just once per model to bound one region and the shell object can act as its own Shelluse object.


A connection between an object and a boundary is represented as a connection between their use objects.


The following diagram shows the Brep object model.

![alt_text](images/image0.png "Brep Object Model")



### USD implementation

To make the USD implementation as lightweight as possible, yet fully featured, we propose using a single concrete IsA schema as an array of Breps and single apply APIs to add geometry to the array.
The _UsdSolidBrepArray_ is a flattened format that describes all the necessary connectivity to build the Brep directed graph, with topology and "use" objects; and standardizes the application of select metadata.
Each geometry type, e.g. _NURBS_ curves or surfaces, are singly apply API schemas.
Since each type of geometry is optional (a given Brep may have only _NURBS_ and no analytics), this will minimize the number of default valued attributes.

In solid modeling a Brep is not a monolithic object; each object within the Brep has its own instance and may have unique properties.
For example, material properties may be assigned to Faces, ID tags for any or all objects, etc.
A minimal set of metadata for the components within a Brep are included in the schema.

Any Brep model must be modified to optimize its performance in OpenUSD.
The set of modifications necessary are
1. Flattened representation
    1. The schema must represent all topology with arrays of indices
    1. Entities smaller than a Brep (Face, Edge, etc.) don't exist as `Prims`
1. Geometries are applied schemas
1. Design that allows compacted Breps (multiple Breps in one `Prim`)

This set of modifications could be applied to other Brep models.
The Radial Edge Data Model was chosen because of its neutral position in the CAD industry and its natural representation of general bodies as well as the usual manifold bodies, wire frames, etc.

#### UsdSolidBrepArray

The _UsdSolidBrepArray_ derives from _UsdGeomGprim_ with attributes to define the Brep topology, "uses", and count of each per Brep.
_UsdSolidBrepArray_ derives from _UsdGeomGprim_ so that it can have the properties _Extent_ and _Visibility_, and have _XformOps_ applied.
It follows the rules of all geometric primitives, such as no nested _Gprims_.

The flat format of Brep connectivity is a concise representation of the Brep that creates only a small perturbation of the USD format, a single schema to represent an entire Brep model.
With this model, creating one or more Breps in USD requires one _BrepArray_ to define the connectivity and metadata, with curves and surfaces applied.
In some usecases we expect that a collections of Breps will have one Brep per _BrepArray_.

#### Instancing of Brep models

In this proposal, whole _UsdSolidBrepArray_ can be referenced to create multiple instances of a set of Breps.

#### Brep geometry in USD

There are 4 types of geometry stored along with the _UsdSolidBrepArray_.
The simplest is the vertex location, which is stored as a point3d.
An edge needs a curve and a face needs a surface to have shape, so curves and surfaces are applied APIs, where owning Edges and Faces indicate which geometry gives them shape.
UVTrimCurves are the fourth geometry object, also an applied API.
They are the projection of the edge curves onto the face surfaces.

#### Geometry type extensions

Not yet included at this stage of the proposal is the _USD_ implementation of the new geometry types.
The definitions of the PRC geometry types listed above can be found in the[ PRC specification.](https://docs.techsoft3d.com/exchange/2024/_downloads/a7028c5c324de43fc7d5083bfa100c2a/SC2N570-PRC-WD.pdf)
Having reviewed the definitions, we see no issues with the potential implementation.
Care will be taken to ensure proper architecture.
Each geometry type definition will be another applied API.
The attributes will be compact definitions of the parameterized shape, allowing multiple geometries of one type to be defined within the finite set of attributes.

#### Modeling Breps on a UsdStage

Efficient modeling or editing a Brep directly on a UsdStage is not a feature of the current schema, but there are clear steps to take to support this.
Creating schemas for the 11 topology and use objects in the Brep model will allow the entire directed graph structure of the Brep to be represented in USD, which will be well-suited for live editing.

#### Trimming curves

Optional trim curves can be included similarly to curve and surface geometry.
The edgeuse defines the connection between a given edge and face; the edgeuse stores an index for the associated trimming curve applied API.
Trim curves are optional in this Brep model because the edge curve defines the model truth, but trim curves are useful for, e.g., speeding up tessellation algorithms.


### Flexible design possibilities

The _UsdSolidBrepArray_ is deliberately an _array_ of Breps rather than a single Brep, but this does not imply it is an assembly or a scenegraph hierarchy. A _UsdSolidBrepArray_ is a flexible list of Brep parts; the semantics of CAD assembly relationships (mates, joints, kinematic constraints) are intentionally out of scope for this proposal (see [Assemblies](#assemblies)).

The array form supports a range of authoring paradigms. For most users, a 1:1 mapping between _UsdPrim_ and Brep is the natural choice and leverages OpenUSD's referencing, instancing, and layering directly. When prim-count reduction or uniform material treatment matters, packing a set of rigidly connected Breps into a single _UsdSolidBrepArray_ is also valid. These are not competing designs; they are endpoints on a spectrum the schema permits.

Sparse overrides over Brep variants are a promising avenue for future work: an override layer could represent the delta between a source Brep and a variant without reserializing the full topology. This is not pursued in the current proposal but is preserved as a design possibility (see Open Questions).

The UsdSolidBrepArray enables many possible design paradigms.
We enumerate some of the choices here.

#### One Brep per BrepArray

The first design we present draws an equivalency between a Brep and a _UsdPrim_.
Standard _USD_ hierarchies can be built to represent a CAD model.

This design is well suited to building a _USD_ representation of a CAD model similar to how it would be structured in the native design software.
A _USD_ model in this style of a large assembly containing a vast number of constraints would be challenging to simulate.
A single model can have multiple assembly representations depending on the designers use case.
An animator creating marketing material will rig a model differently than an engineer writing manufacturing documentation.
When constraints are applied throughout this model, the hierarchy is not as useful.

#### One Assembly per BrepArray

Next, one might consider a design where each _UsdSolidBrepArray_ contains a set of Breps forming a rigidly connected CAD assembly.
Consider a car model represented in this way, where an entire door is packed into one _UsdSolidBrepArray_.
Adding constraints to the door hinge enables simulation or modeling of the door opening and closing.

This simplified model reduces the number of constraints that need to be simulated.

#### One Model per BrepArray

Last, a user could pack an entire model into a single _UsdSolidBrepArray_ _UsdPrim_.
This design is well suited to content delivery due to its highly packed nature.
The single _UsdPrim_ design minimizes the cost of stage traversal.

### Other implementations considered

Several designs were implemented prior to the one presented.
We discuss them below.

#### One UsdPrim per geometry object

The central design question in this section — whether to further decompose the Brep across prims — hinges on what OpenUSD is being asked to do with CAD data. The aim of this proposal is to support the _use_ of designs authored in CAD modelers (visualization, clash detection, simulation, measurement, downstream tessellation), not CAD _authoring_ in USD itself. Without a compelling use case for live CAD design on a _UsdStage_, the proliferation of prims that per-geometry-object decomposition would require cannot be justified.

The first design attempted created individual prims for each brep and its curves and surfaces.
This design followed the _UsdGeom_ schema design where _UsdGeomNurbsPatch_ exists for individual surfaces.
It was also thought that being able to interact with individual curves and surfaces may be useful.

Practice found that this was an untenable design.
A surface model of a car was used to test the design.
Surfaces with geometric proximity and like materials were stitched into Breps, then imported to _USD_.
A typical model would have 500 Breps, each with 100 geometry _UsdPrim_.
Representing each model with 50,000 _UsdPrim_ is not practical, so a new design that packed geometry into the Brep was created.

#### One UsdPrim per Brep

The second design eliminated the surface and curve prims.
Instead, all of the geometry information was moved into the _Usd_ Brep schema, packing geometry based on type.
This design improved performance and shrunk file size by 1/3.

The proposed design is capable of everything the one-_UsdPrim_-per-Brep design is, but adds the capability of packing multiple Breps into a single _UsdPrim_.

#### Breps as an Applied API

In this design the _BrepArray_ was a strongly typed container derived from _Gprim_.
Each Brep was added to the _BrepArray_ through an application of a multiple apply API.
This design had an advantage over the proposed design in instancing of individual Breps.
Utilizing _Connections_, a single Brep in this _BrepArray_ could be referenced in another _BrepArray_.

What this design lacked was coherence with USD's style.
Using _Connections_ to instance a Brep could be considered an abuse.
Effective instancing required unique _XformOps_ for each Brep applied to the _BrepArray_, forcing a new means to apply _XformOps_ to subsets of a _UsdPrim_.
Further, _GeomSubset_ wasn't an option for applying material properties to Breps or Faces, so a new material binding scheme was required.


This schema was shelved because it deviated too far from _USD_ norms.

#### Breps as a black box

One proposed design was to treat Breps as a black box, from the OpenUSD perspective, a la Volumes.
The advantage here lies in the common use case of tessellation.
It is possible to reuse existing standards and technology, such as STEP and open geometry kernels, to reduce the OpenUSD Brep implementation to a file reference, then import to OpenUSD mesh representations of Breps ad hoc.

The AOUSD Geometry Working Group found this design too limiting.
The rationale for rejecting this approach -- and for not simply adopting STEP -- is discussed in detail in the companion [problem statement](../cad_geometry/README.md).
In summary: including the whole Brep model topology and geometry allows for per-face and per-region property assignment, USD-native composition and overrides, and future assembly design where constraints are assigned between different Brep bodies.


### Assemblies

Most non-trivial CAD models will be assemblies of parts.
However, CAD assemblies are outside the scope of this proposal as we aim to define only the base geometric and topologic representation of a Brep.


Significant care will be necessary when defining how CAD assemblies will be represented in OpenUSD.
First, the concept of _kind_ already exists so the name _assembly_ will be overloaded.
The existing _kind_ is used for picking; applying that label to CAD assemblies would cause confusion and conflicts in the scene.
Second, it is not clear whether CAD assemblies will require a new schema or be a best-practices guide utilizing existing instancing tools.
Last, the CAD assembly structure should work with constraints imposed by external modeling tools, further enabling world simulation.


### Tolerance

A valid Brep will have a single tolerance number that it conforms to.
Any two topologically connected geometric entities will have a maximum gap size less than the given tolerance.
This includes trim curves, which must be within tolerance to both the surface and projected curve.
Degenerate geometry is not allowed, where degeneracy is measured against tolerance.
All unconnected topologic entities must have a minimum  gap greater than tolerance.
The specific rules are enumerated in the [Rules and requirements](#rules-and-requirements) section below.


### Validation


Validation is essential to ensure that authored Breps are well-formed.
We record in this proposal the rules that we expect authored Breps to conform to.
Given the complex nature of geometric modeling, not all of the constraints are verifiable within OpenUSD.

For the proposed model, it is straight-forward to create a topology verification tool.
One can walk the topology graph of the Brep to confirm that, e.g., every edge has a start and end vertex.
Some geometric data is verifiable as well, such as confirming that a NURBS curve has a coherent order, knot vector, weights and control vertices.

But some geometric requirements will be outside the scope of OpenUSD.
Without a complete geometric engine, testing for, e.g., self-intersecting curves is not possible.
As adoption of this schema grows, we hope to find that 3rd party geometry modeling libraries are interested in taking on the challenge of validating Usd Breps.


#### Rules and requirements

Here we record the rules and requirements of a valid Usd Brep model.
These rules will be included in the schema prior to publishing.

1. Brep
    1. All self-intersections are marked with appropriate topology, including:
        1. Any face-face intersections have an edge and/or vertex
        1. Any edge-edge intersections have a vertex
1. Regions
    1. All regions are separated by closed shells
1. Shells
    1. A shell may contain either
        1. A vertex
        1. WireEdges and vertices
        1. or Facesuses and (optional) wireEdges and vertices
1. Faceuses
    1. The "same" orientated faceuse is on the positive-normal side of the associated surface
1. Faces
    1. The face range must be a subset of the surface range
    1. No sliver faces (a face is a sliver if it is contained in a pipe with radius = tolerance)
    1. No faces with area less than tolerance^2
    1. A face has a single outer loop (seam edges are required)
    1. The first loop listed is the outer loop.
1. Loops
    1. A loop may contain either
        1. A vertex (degenerate inner loop on a face)
        1. or one or more edgeuses
1. Edges
    1. The edge range must be a subset of the curve range
    1. No edges contained in a sphere of radius = tolerance
    1. The orientation of the edge is the same as its curve
    1. The curve runs from the start vertex to the end vertex (possibly through either vertex)
1. Surfaces
    1. No sliver surfaces (a surface is a sliver if it is contained in a pipe with radius = tolerance)
    1. No surface with area less than tolerance^2
1. Curves
    1. No edges contained in a sphere of radius = tolerance
    1. Curve orientation is with the parameterization
1. Trim Curves
    1. An edge or wireedge type edgeuse must be represented with NURBS
    1. The control points need not be constrained to lie on the surface, but the curve must
1. Range
    1. For any range _[a, b]_ it must be that _b > a_
    1. The range on periodic geometry must have length <= period



### Risks

Several risks warrant attention as this schema moves toward adoption:

- **Validation complexity.** Full geometric validation — for example self-intersection detection, tangency checks, and tolerance enforcement across a Brep — requires a geometry kernel and is outside the scope of OpenUSD. The core representation can be validated topologically (see [Validation](#validation)), but geometric validity will depend on third-party libraries. Producers authoring invalid Breps risk silent downstream failures.
- **Geometry kernel dependencies for rendering.** Tessellation of Breps for rendering requires a geometry engine. OpenUSD itself does not ship one. A successful deployment depends on an ecosystem path where kernels (open source or commercial) consume UsdSolid data, which introduces a supply-chain consideration for adopters.
- **Performance at scale.** Large industrial assemblies may contain thousands of Breps and tens of thousands of faces. The packed _UsdSolidBrepArray_ design is motivated in part by prim-count concerns, but composition, layering, and instancing performance at scale remain to be characterized with real datasets.
- **Ecosystem readiness.** DCCs, viewers, and engines will need to add Brep support before the schema delivers end-user value. The schema's early success will depend on coordinated adoption rather than any single implementation.
- **Data quality dependence on exporters.** Because Brep authoring happens outside OpenUSD, the quality of UsdSolid data will track the quality of the exporter. Consumers should expect variance until exporters mature, and the WG should consider publishing reference datasets and a conformance suite.

The companion problem statement identifies additional ecosystem-level risks including adoption sequencing and core-team alignment.

### Open questions

The following design and deployment questions remain open. They are recorded here to support discussion during review and to inform follow-up proposals; they are not blockers to landing the core schema.

- **CAD assembly representation.** Assemblies are out of scope for this proposal (see [Assemblies](#assemblies)). Whether they warrant a new schema or a best-practices guide built on existing instancing and constraint tools is open. Interaction with the existing _kind_ metadata — already used for model-hierarchy roles such as `assembly` and `component` — needs careful treatment to avoid overloading terminology.
- **Brep↔Mesh correlation.** Relating a Brep to its tessellated derivatives is deferred (see the companion [problem statement](../cad_geometry/README.md)). One candidate approach raised by Steve Ghee: because gprims are generally derived from Breps in an N:1 relationship, a gprim could back-reference its source Brep(s) by identifier or path via an applied API schema. At runtime, these cross-references would let a Brep discover which gprims represent it (avoiding redundant tessellation) and let a picked gprim trace back to its source. Supporting an array of references would handle the case where a single gprim is derived from multiple Breps. This mechanism is best designed as a follow-up once the core schema is deployed and real-world datasets are available.
- **Constraint systems for assembly simulation.** Mates, joints, and kinematic constraints are foundational to industrial simulation and digital-twin workflows. Whether constraints belong in UsdSolid, in a separate schema, or in an OpenUSD-wide constraint proposal is unresolved.
- **Migration from STEP, PRC, and Parasolid.** Detailed mappings from widely used Brep interchange formats into UsdSolid are needed. The radial edge data model is general enough to accommodate them, but practical exporters will surface decisions about tolerance propagation, PMI (product manufacturing information) carriage, and metadata preservation.
- **Sparse overrides for Brep variants.** As noted in [Flexible design possibilities](#flexible-design-possibilities), sparse overrides that represent the delta between a source Brep and a variant are an appealing avenue but not pursued here.

## Schema

> **Implementation note:** The library and class names in the schema use a `prelimUsdSolid` / `PrelimUsdSolid` prefix and set `skipCodeGeneration = true`. This follows the OpenUSD convention for work-in-progress schemas that are not yet built into the SDK. When the schema is accepted for inclusion in OpenUSD the prefix will be stripped and code generation enabled, following the same lifecycle used by other preliminary schemas (e.g., the Gaussian splat schema progressed from `prelimUsdRenderGaussian` to `UsdRenderGaussian`).

**schema.usda**

The full schema definition lives alongside this document as a standalone file: [`schema.usda`](./schema.usda).


## Examples

The following examples demonstrate the _UsdSolidBrepArray_ schema at increasing levels of complexity. Each builds on the last, so the commentary highlights what changes rather than restating the full schema.

> **Reading tip:** every top-level attribute on a _BrepArray_ corresponds directly to the schema in the [Schema](#schema) section. The array sizes and index relationships follow the rules in the `doc` strings there; the commentary below highlights what each example is meant to demonstrate rather than restating the schema.

### Cube

A minimal manifold Brep: a unit cube with 8 vertices, 12 edges, 6 faces, 6 loops, 2 shells, and 2 regions (one solid, one void). This example establishes the baseline topology and attribute layout that all subsequent examples extend.

![Cube](images/cube.png "Cube")

**CubeIds.usda**
<details>
  <summary> Click to expand </summary>

```
#usda 1.0

def Xform "World"
{
    def BrepArray "Cube" (
        prepend apiSchemas = ["BrepPointAPI:vertexPoint", "BrepCurve3dNurbAPI:edge3dNurb", "BrepSurfaceNurbAPI"]
    )
    {
        uniform point3d[] brep:edge3dNurb:curve3d:nurb:controlVertices = [(1, 0, 1), (1, 1, 1), (0, 1, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 1, 0), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1), (1, 0, 1), (0, 1, 0), (0, 1, 1), (0, 1, 0), (1, 1, 0)]
        uniform double[] brep:edge3dNurb:curve3d:nurb:knots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:vertexCount = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform double[] brep:edge3dNurb:curve3d:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform double3[] brep:extent = [(0, 0, 0), (1, 1, 1)]
        uniform double[] brep:intersectTol3d = [0.00002]
        uniform uint[] brep:regionCount = [2]
        uniform point3d[] brep:surface:nurb:controlVertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 0, 1), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (1, 0, 0), (1, 0, 1), (0, 1, 0), (1, 1, 0), (0, 1, 1), (1, 1, 1)]
        uniform double[] brep:surface:nurb:uKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:surface:nurb:uOrder = [2, 2, 2, 2, 2, 2]
        uniform uint[] brep:surface:nurb:uVertexCount = [2, 2, 2, 2, 2, 2]
        uniform double[] brep:surface:nurb:vKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:surface:nurb:vOrder = [2, 2, 2, 2, 2, 2]
        uniform uint[] brep:surface:nurb:vVertexCount = [2, 2, 2, 2, 2, 2]
        uniform double[] brep:surface:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform point3d[] brep:vertexPoint:point:position = [(1, 1, 1), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 1, 1), (0, 1, 0), (1, 1, 0)]
        uniform token[] edge:curveType = ["BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI"]
        uniform double[] edge:range = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
        uniform int2[] edge:vertexIndices = [(4, 0), (5, 0), (2, 6), (1, 5), (3, 7), (7, 0), (2, 1), (2, 3), (3, 4), (1, 4), (6, 5), (6, 7)]
        uniform uint[] edgeuse:edgeIndex = [2, 11, 4, 7, 3, 9, 0, 1, 6, 3, 10, 2, 8, 4, 5, 0, 6, 7, 8, 9, 10, 1, 5, 11]
        uniform uint[] edgeuse:nextRadialEUIndex = [11, 23, 13, 17, 9, 19, 15, 21, 16, 4, 20, 0, 18, 2, 22, 6, 8, 3, 12, 5, 10, 7, 14, 1]
        uniform token[] edgeuse:orientationType = ["same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite"]
        uniform token[] edgeuse:thisRadialEntryType = ["topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "bottomEntry", "bottomEntry"]
        uniform float3[] extent = [(0, 0, 0), (1, 1, 1)]
        uniform uint[] face:loopCount = [1, 1, 1, 1, 1, 1]
        uniform double2[] face:range = [(0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1)]
        uniform token[] face:surfaceType = ["BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI"]
        uniform token[] face:trimType = ["general", "general", "general", "general", "general", "general"]
        uniform uint[] faceuse:faceIndex = [5, 4, 2, 0, 3, 1, 5, 1, 4, 0, 3, 2]
        uniform token[] faceuse:orientationType = ["same", "same", "same", "same", "same", "same", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite"]
        uniform uint[] loop:edgeuseCount = [4, 4, 4, 4, 4, 4]
        uniform uint[] loop:vertexIndex = [9999999, 9999999, 9999999, 9999999, 9999999, 9999999]
        uniform uint[] region:shellCount = [1, 1]
        uniform token[] region:type = ["voidRegion", "solidRegion"]
        uniform uint[] shell:faceuseCount = [6, 6]
        uniform token[] shell:pointType = ["none", "none"]
        uniform uint[] shell:wireEdgeCount = [0, 0]
        uniform token[] vertex:pointType = ["BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI"]
        uniform token[] wireEdge:curveType = []
        uniform double[] wireEdge:range = []
        uniform int2[] wireEdge:vertexIndices = []
    }
}
```
</details>
 

### Non-manifold cubes

What changes vs. [Cube](#cube): two cubes that share a single face, producing a non-manifold Brep. This example demonstrates that the schema represents _every_ partition of space as an explicit _Region_, not just the inside/outside pair of a manifold solid.

Two cubes sharing a face have 3 regions: the infinite region outside the Brep, and 1 region inside each cube. The model contains 11 faces, as one is shared between the cubes.

In the wireframe model image below, the non-manifold edges are shown in blue. An edge is non-manifold when it does not connect to exactly 2 faces (or 1 face twice for closed surfaces).


![NonManifold Cubes](images/cube2.png "Non-manifold cubes")


**nonManifoldCubes.usda**

<details>
  <summary> Click to expand </summary>

```
#usda 1.0

def Xform "World"
{
    def BrepArray "nonManifoldCubes" (
        prepend apiSchemas = ["BrepPointAPI:vertexPoint", "BrepCurve3dNurbAPI:edge3dNurb", "BrepSurfaceNurbAPI"]
    )
    {
        uniform point3d[] brep:edge3dNurb:curve3d:nurb:controlVertices = [(1, 0, 1), (1, 1, 1), (0, 1, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 1, 0), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1), (1, 0, 1), (0, 1, 0), (0, 1, 1), (0, 1, 0), (1, 1, 0), (1, 0, 1), (2, 0, 1), (2, 0, 1), (2, 1, 1), (1, 1, 1), (2, 1, 1), (2, 1, 0), (2, 1, 1), (1, 1, 0), (2, 1, 0), (2, 0, 0), (2, 1, 0), (1, 0, 0), (2, 0, 0), (2, 0, 0), (2, 0, 1)]
        uniform double[] brep:edge3dNurb:curve3d:nurb:knots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:vertexCount = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform double[] brep:edge3dNurb:curve3d:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform double3[] brep:extent = [(0, 0, 0), (2, 1, 1)]
        uniform double[] brep:intersectTol3d = [0.00002]
        uniform uint[] brep:regionCount = [3]
        uniform point3d[] brep:surface:nurb:controlVertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 0, 1), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (1, 0, 0), (1, 0, 1), (0, 1, 0), (1, 1, 0), (0, 1, 1), (1, 1, 1), (1, 0, 1), (1, 1, 1), (2, 0, 1), (2, 1, 1), (1, 1, 0), (2, 1, 0), (1, 1, 1), (2, 1, 1), (1, 0, 0), (2, 0, 0), (1, 1, 0), (2, 1, 0), (2, 0, 0), (2, 0, 1), (2, 1, 0), (2, 1, 1), (1, 0, 0), (1, 0, 1), (2, 0, 0), (2, 0, 1)]
        uniform double[] brep:surface:nurb:uKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:surface:nurb:uOrder = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform uint[] brep:surface:nurb:uVertexCount = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform double[] brep:surface:nurb:vKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:surface:nurb:vOrder = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform uint[] brep:surface:nurb:vVertexCount = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform double[] brep:surface:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform point3d[] brep:vertexPoint:point:position = [(1, 1, 1), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 1, 1), (0, 1, 0), (1, 1, 0), (2, 0, 1), (2, 1, 1), (2, 1, 0), (2, 0, 0)]
        uniform token[] edge:curveType = ["BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI"]
        uniform double[] edge:range = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
        uniform int2[] edge:vertexIndices = [(4, 0), (5, 0), (2, 6), (1, 5), (3, 7), (7, 0), (2, 1), (2, 3), (3, 4), (1, 4), (6, 5), (6, 7), (4, 8), (8, 9), (0, 9), (10, 9), (7, 10), (11, 10), (3, 11), (11, 8)]
        uniform uint[] edgeuse:edgeIndex = [2, 11, 4, 7, 3, 9, 0, 1, 6, 3, 10, 2, 8, 4, 5, 0, 6, 7, 8, 9, 10, 1, 5, 11, 0, 12, 13, 14, 5, 14, 15, 16, 4, 16, 17, 18, 19, 17, 15, 13, 8, 18, 19, 12]
        uniform uint[] edgeuse:nextRadialEUIndex = [11, 23, 32, 17, 9, 19, 15, 21, 16, 4, 20, 0, 40, 2, 22, 24, 8, 3, 12, 5, 10, 7, 28, 1, 6, 43, 39, 29, 14, 27, 38, 33, 13, 31, 37, 41, 42, 34, 30, 26, 18, 35, 36, 25]
        uniform token[] edgeuse:orientationType = ["same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "opposite", "same", "same", "opposite"]
        uniform token[] edgeuse:thisRadialEntryType = ["topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "bottomEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry"]
        uniform float3[] extent = [(0, 0, 0), (2, 1, 1)]
        uniform uint[] face:loopCount = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform double2[] face:range = [(0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1)]
        uniform token[] face:surfaceType = ["BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI"]
        uniform token[] face:trimType = ["general", "general", "general", "general", "general", "general", "general", "general", "general", "general", "general"]
        uniform uint[] faceuse:faceIndex = [5, 4, 2, 0, 1, 6, 7, 8, 9, 10, 5, 1, 4, 0, 3, 2, 10, 8, 7, 6, 9, 3]
        uniform token[] faceuse:orientationType = ["same", "same", "same", "same", "same", "same", "same", "same", "same", "same", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite", "same"]
        uniform uint[] loop:edgeuseCount = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
        uniform uint[] loop:vertexIndex = [9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999]
        uniform uint[] region:shellCount = [1, 1, 1]
        uniform token[] region:type = ["voidRegion", "solidRegion", "solidRegion"]
        uniform uint[] shell:faceuseCount = [10, 6, 6]
        uniform token[] shell:pointType = ["none", "none", "none"]
        uniform uint[] shell:wireEdgeCount = [0, 0, 0]
        uniform token[] vertex:pointType = ["BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI"]
        uniform token[] wireEdge:curveType = []
        uniform double[] wireEdge:range = []
        uniform int2[] wireEdge:vertexIndices = []
    }
}
```

</details>
 

### Cube with internal void

What changes vs. [Cube](#cube): a manifold cube that contains a spherical void — a hollow pocket inside the solid. This example demonstrates how the schema represents interior cavities using multiple shells on a region.

The Brep has three regions: the infinite void outside, the solid cube, and the interior void. The solid region is bounded by _two_ shells: an outer shell of 6 cube faces and an inner shell of a single spherical face. `region:shellCount` encodes this as `[1, 2, 1]` — the two one-shell regions are the infinite void and the interior void, and the two-shell middle entry is the solid. Per the [Design](#design) section, the outer shell of a region is listed first; any subsequent shells are inner shells defining cavities.

![Cube With Internal Void](images/cubevoid.png "Cube With Internal Void")


**cubeWithVoid.usda**
<details>
  <summary> Click to expand </summary>

```
#usda 1.0

def Xform "World"
{
    def BrepArray "cubeWithVoid" (
        prepend apiSchemas = ["BrepPointAPI:vertexPoint", "BrepCurve3dNurbAPI:edge3dNurb", "BrepSurfaceNurbAPI"]
    )
    {
        uniform point3d[] brep:edge3dNurb:curve3d:nurb:controlVertices = [(1, 0, 1), (1, 1, 1), (0, 1, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 1, 0), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1), (1, 0, 1), (0, 1, 0), (0, 1, 1), (0, 1, 0), (1, 1, 0), (0.5, 0.5, 0.3), (0.7, 0.5, 0.3), (0.7, 0.5, 0.5), (0.7, 0.5, 0.7), (0.5, 0.5, 0.7)]
        uniform double[] brep:edge3dNurb:curve3d:nurb:knots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0.5, 0.5, 1, 1, 1]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:vertexCount = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 5]
        uniform double[] brep:edge3dNurb:curve3d:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.7071067811865476, 1, 0.7071067811865476, 1]
        uniform double3[] brep:extent = [(0, 0, 0), (1, 1, 1)]
        uniform double[] brep:intersectTol3d = [0.00002]
        uniform uint[] brep:regionCount = [3]
        uniform point3d[] brep:surface:nurb:controlVertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 0, 1), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (1, 0, 0), (1, 0, 1), (0, 1, 0), (1, 1, 0), (0, 1, 1), (1, 1, 1), (0.5, 0.5, 0.3), (0.7, 0.5, 0.3), (0.7, 0.5, 0.5), (0.7, 0.5, 0.7), (0.5, 0.5, 0.7), (0.5, 0.5, 0.3), (0.7, 0.7, 0.3), (0.7, 0.7, 0.5), (0.7, 0.7, 0.7), (0.5, 0.5, 0.7), (0.5, 0.5, 0.3), (0.5, 0.7, 0.3), (0.5, 0.7, 0.5), (0.5, 0.7, 0.7), (0.5, 0.5, 0.7), (0.5, 0.5, 0.3), (0.3, 0.7, 0.3), (0.3, 0.7, 0.5), (0.3, 0.7, 0.7), (0.5, 0.5, 0.7), (0.5, 0.5, 0.3), (0.3, 0.5, 0.3), (0.3, 0.5, 0.5), (0.3, 0.5, 0.7), (0.5, 0.5, 0.7), (0.5, 0.5, 0.3), (0.3, 0.3, 0.3), (0.3, 0.3, 0.5), (0.3, 0.3, 0.7), (0.5, 0.5, 0.7), (0.5, 0.5, 0.3), (0.5, 0.3, 0.3), (0.5, 0.3, 0.5), (0.5, 0.3, 0.7), (0.5, 0.5, 0.7), (0.5, 0.5, 0.3), (0.7, 0.3, 0.3), (0.7, 0.3, 0.5), (0.7, 0.3, 0.7), (0.5, 0.5, 0.7), (0.5, 0.5, 0.3), (0.7, 0.5, 0.3), (0.7, 0.5, 0.5), (0.7, 0.5, 0.7), (0.5, 0.5, 0.7)]
        uniform double[] brep:surface:nurb:uKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1]
        uniform uint[] brep:surface:nurb:uOrder = [2, 2, 2, 2, 2, 2, 3]
        uniform uint[] brep:surface:nurb:uVertexCount = [2, 2, 2, 2, 2, 2, 9]
        uniform double[] brep:surface:nurb:vKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0.5, 0.5, 1, 1, 1]
        uniform uint[] brep:surface:nurb:vOrder = [2, 2, 2, 2, 2, 2, 3]
        uniform uint[] brep:surface:nurb:vVertexCount = [2, 2, 2, 2, 2, 2, 5]
        uniform double[] brep:surface:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.7071067811865476, 1, 0.7071067811865476, 1, 0.7071067811865476, 0.5, 0.7071067811865476, 0.5, 0.7071067811865476, 1, 0.7071067811865476, 1, 0.7071067811865476, 1, 0.7071067811865476, 0.5, 0.7071067811865476, 0.5, 0.7071067811865476, 1, 0.7071067811865476, 1, 0.7071067811865476, 1, 0.7071067811865476, 0.5, 0.7071067811865476, 0.5, 0.7071067811865476, 1, 0.7071067811865476, 1, 0.7071067811865476, 1]
        uniform point3d[] brep:vertexPoint:point:position = [(1, 1, 1), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 1, 1), (0, 1, 0), (1, 1, 0), (0.5, 0.5, 0.7), (0.5, 0.5, 0.3)]
        uniform token[] edge:curveType = ["BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI"]
        uniform double[] edge:range = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
        uniform int2[] edge:vertexIndices = [(4, 0), (5, 0), (2, 6), (1, 5), (3, 7), (7, 0), (2, 1), (2, 3), (3, 4), (1, 4), (6, 5), (6, 7), (9, 8)]
        uniform uint[] edgeuse:edgeIndex = [2, 11, 4, 7, 3, 9, 0, 1, 6, 3, 10, 2, 8, 4, 5, 0, 6, 7, 8, 9, 10, 1, 5, 11, 12, 12]
        uniform uint[] edgeuse:nextRadialEUIndex = [11, 23, 13, 17, 9, 19, 15, 21, 16, 4, 20, 0, 18, 2, 22, 6, 8, 3, 12, 5, 10, 7, 14, 1, 25, 24]
        uniform token[] edgeuse:orientationType = ["same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "opposite", "same"]
        uniform token[] edgeuse:thisRadialEntryType = ["topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "bottomEntry", "bottomEntry", "bottomEntry", "topEntry"]
        uniform float3[] extent = [(0, 0, 0), (1, 1, 1)]
        uniform uint[] face:loopCount = [1, 1, 1, 1, 1, 1, 1]
        uniform double2[] face:range = [(0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1)]
        uniform token[] face:surfaceType = ["BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI"]
        uniform token[] face:trimType = ["general", "general", "general", "general", "general", "general", "general"]
        uniform uint[] faceuse:faceIndex = [5, 4, 2, 0, 3, 1, 5, 1, 4, 0, 3, 2, 6, 6]
        uniform token[] faceuse:orientationType = ["same", "same", "same", "same", "same", "same", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite", "same", "opposite"]
        uniform uint[] loop:edgeuseCount = [4, 4, 4, 4, 4, 4, 2]
        uniform uint[] loop:vertexIndex = [9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999]
        uniform uint[] region:shellCount = [1, 2, 1]
        uniform token[] region:type = ["voidRegion", "solidRegion", "voidRegion"]
        uniform uint[] shell:faceuseCount = [6, 6, 1, 1]
        uniform token[] shell:pointType = ["none", "none", "none", "none"]
        uniform uint[] shell:wireEdgeCount = [0, 0, 0, 0]
        uniform token[] vertex:pointType = ["BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI"]
        uniform token[] wireEdge:curveType = []
        uniform double[] wireEdge:range = []
        uniform int2[] wireEdge:vertexIndices = []
    }
}
```

</details>

### BrepArray with multiple Breps, individual colors

What changes vs. the preceding examples: a single _BrepArray_ prim that packs two Breps, with distinct materials bound to each. This example demonstrates the array semantics described in [Flexible design possibilities](#flexible-design-possibilities) — a prim can hold more than one Brep — and the use of _GeomSubset_ to bind materials to individual Breps within that prim.

The two cubes are geometrically independent (they do not share topology, unlike [Non-manifold cubes](#non-manifold-cubes)). All per-Brep arrays (`brep:regionCount`, `brep:*Extent`) are length 2; the topology arrays concatenate the two Breps' objects in order, and the per-Brep counts (`brep:*Count`) describe the split.

![BrepArray](images/BrepArray.png "BrepArray with multiple Breps")


**cubeBrepArray.usda**

<details>
  <summary> Click to expand </summary>

```
#usda 1.0

def Xform "World"
{
    def BrepArray "brepArray" (
        prepend apiSchemas = ["BrepPointAPI:vertexPoint", "BrepCurve3dNurbAPI:edge3dNurb", "BrepSurfaceNurbAPI"]
    )
    {
       uniform point3d[] brep:edge3dNurb:curve3d:nurb:controlVertices = [(1, 0, 1), (1, 1, 1), (0, 1, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 1, 0), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1), (1, 0, 1), (0, 1, 0), (0, 1, 1), (0, 1, 0), (1, 1, 0), (3, 0, 1), (3, 1, 1), (2, 1, 1), (3, 1, 1), (2, 0, 0), (2, 1, 0), (2, 0, 1), (2, 1, 1), (3, 0, 0), (3, 1, 0), (3, 1, 0), (3, 1, 1), (2, 0, 0), (2, 0, 1), (2, 0, 0), (3, 0, 0), (3, 0, 0), (3, 0, 1), (2, 0, 1), (3, 0, 1), (2, 1, 0), (2, 1, 1), (2, 1, 0), (3, 1, 0)]
        uniform double[] brep:edge3dNurb:curve3d:nurb:knots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:vertexCount = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform double[] brep:edge3dNurb:curve3d:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform double3[] brep:extent = [(0, 0, 0), (1, 1, 1), (2, 0, 0), (3, 1, 1)]
        uniform double[] brep:intersectTol3d = [0.00002, 0.00002]
        uniform uint[] brep:regionCount = [2, 2]
        uniform point3d[] brep:surface:nurb:controlVertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 0, 1), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (1, 0, 0), (1, 0, 1), (0, 1, 0), (1, 1, 0), (0, 1, 1), (1, 1, 1), (2, 0, 0), (3, 0, 0), (2, 1, 0), (3, 1, 0), (2, 0, 1), (2, 1, 1), (3, 0, 1), (3, 1, 1), (2, 0, 0), (2, 1, 0), (2, 0, 1), (2, 1, 1), (3, 0, 0), (3, 0, 1), (3, 1, 0), (3, 1, 1), (2, 0, 0), (2, 0, 1), (3, 0, 0), (3, 0, 1), (2, 1, 0), (3, 1, 0), (2, 1, 1), (3, 1, 1)]
        uniform double[] brep:surface:nurb:uKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:surface:nurb:uOrder = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform uint[] brep:surface:nurb:uVertexCount = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform double[] brep:surface:nurb:vKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:surface:nurb:vOrder = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform uint[] brep:surface:nurb:vVertexCount = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform double[] brep:surface:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform point3d[] brep:vertexPoint:point:position = [(1, 1, 1), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 1, 1), (0, 1, 0), (1, 1, 0), (3, 1, 1), (2, 0, 1), (2, 0, 0), (3, 0, 0), (3, 0, 1), (2, 1, 1), (2, 1, 0), (3, 1, 0)]
        uniform token[] edge:curveType = ["BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI"]
        uniform double[] edge:range = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
        uniform int2[] edge:vertexIndices = [(4, 0), (5, 0), (2, 6), (1, 5), (3, 7), (7, 0), (2, 1), (2, 3), (3, 4), (1, 4), (6, 5), (6, 7), (12, 8), (13, 8), (10, 14), (9, 13), (11, 15), (15, 8), (10, 9), (10, 11), (11, 12), (9, 12), (14, 13), (14, 15)]
        uniform uint[] edgeuse:edgeIndex = [2, 11, 4, 7, 3, 9, 0, 1, 6, 3, 10, 2, 8, 4, 5, 0, 6, 7, 8, 9, 10, 1, 5, 11, 14, 23, 16, 19, 15, 21, 12, 13, 18, 15, 22, 14, 20, 16, 17, 12, 18, 19, 20, 21, 22, 13, 17, 23]
        uniform uint[] edgeuse:nextRadialEUIndex = [11, 23, 13, 17, 9, 19, 15, 21, 16, 4, 20, 0, 18, 2, 22, 6, 8, 3, 12, 5, 10, 7, 14, 1, 35, 47, 37, 41, 33, 43, 39, 45, 40, 28, 44, 24, 42, 26, 46, 30, 32, 27, 36, 29, 34, 31, 38, 25]
        uniform token[] edgeuse:orientationType = ["same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite"]
        uniform token[] edgeuse:thisRadialEntryType = ["topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "bottomEntry", "bottomEntry"]
        uniform float3[] extent = [(0, 0, 0), (3, 1, 1)]
        uniform uint[] face:loopCount = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform double2[] face:range = [(0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1)]
        uniform token[] face:surfaceType = ["BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI"]
        uniform token[] face:trimType = ["general", "general", "general", "general", "general", "general", "general", "general", "general", "general", "general", "general"]
        uniform uint[] faceuse:faceIndex = [5, 4, 2, 0, 3, 1, 5, 1, 4, 0, 3, 2, 11, 10, 8, 6, 9, 7, 11, 7, 10, 6, 9, 8]
        uniform token[] faceuse:orientationType = ["same", "same", "same", "same", "same", "same", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite", "same", "same", "same", "same", "same", "same", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite"]
        uniform uint[] loop:edgeuseCount = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
        uniform uint[] loop:vertexIndex = [9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999, 9999999]
        uniform uint[] region:shellCount = [1, 1, 1, 1]
        uniform token[] region:type = ["voidRegion", "solidRegion", "voidRegion", "solidRegion"]
        uniform uint[] shell:faceuseCount = [6, 6, 6, 6]
        uniform token[] shell:pointType = ["none", "none", "none", "none"]
        uniform uint[] shell:wireEdgeCount = [0, 0, 0, 0]
        uniform token[] vertex:pointType = ["BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI"]
        uniform token[] wireEdge:curveType = []
        uniform double[] wireEdge:range = []
        uniform int2[] wireEdge:vertexIndices = []

        def GeomSubset "subset_0" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {
            uniform token elementType = "brep"
            uniform int[] indices = [0]
            rel material:binding = </World/Looks/Black>
        }

        def GeomSubset "subset_1" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {
            uniform token elementType = "brep"
            uniform int[] indices = [1]
            rel material:binding = </World/Looks/Green>
        }
    }

    def "Looks"
    {
        def Material "Black"
        {
            token outputs:surface.connect = </World/Looks/Black/PBRShader.outputs:surface>
            def Shader "PBRShader"
            {
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = (0, 0, 0)
                token outputs:surface
            }
        }
        def Material "Green"
        {
            token outputs:surface.connect = </World/Looks/Green/PBRShader.outputs:surface>
            def Shader "PBRShader"
            {
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = (0.463, 0.725, 0)
                token outputs:surface
            }
        }
    }
     }
```
</details>

### Brep with materials applied to faces

What changes vs. [BrepArray with multiple Breps, individual colors](#breparray-with-multiple-breps-individual-colors): material bindings are assigned _per face_ within a single Brep, rather than per Brep within a _BrepArray_. This demonstrates finer-grained material control: _UsdGeomSubset_ with `elementType = "face"` selects face indices on the Brep and binds a material to each subset.

When tessellated and authored as a _UsdGeomMesh_, _UsdGeomSubset_ is used to assign appropriate material bindings.

![Brep Face Materials](images/FaceMaterials.png "Brep with materials per face")


**CubeFaceMaterials.usda**

<details>
  <summary> Click to expand </summary>

```
#usda 1.0

def Xform "World"
{
    def BrepArray "CubeMats" (
        prepend apiSchemas = ["BrepPointAPI:vertexPoint", "BrepCurve3dNurbAPI:edge3dNurb", "BrepSurfaceNurbAPI"]
    )
    {
        uniform point3d[] brep:edge3dNurb:curve3d:nurb:controlVertices = [(1, 0, 1), (1, 1, 1), (0, 1, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 1, 0), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1), (1, 0, 1), (0, 1, 0), (0, 1, 1), (0, 1, 0), (1, 1, 0)]
        uniform double[] brep:edge3dNurb:curve3d:nurb:knots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:order = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform uint[] brep:edge3dNurb:curve3d:nurb:vertexCount = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        uniform double[] brep:edge3dNurb:curve3d:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform double3[] brep:extent = [(0, 0, 0), (1, 1, 1)]
        uniform double[] brep:intersectTol3d = [0.00002]
        uniform uint[] brep:regionCount = [2]
        uniform point3d[] brep:surface:nurb:controlVertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 1), (1, 1, 1), (0, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 0), (1, 0, 1), (1, 1, 0), (1, 1, 1), (0, 0, 0), (0, 0, 1), (1, 0, 0), (1, 0, 1), (0, 1, 0), (1, 1, 0), (0, 1, 1), (1, 1, 1)]
        uniform double[] brep:surface:nurb:uKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:surface:nurb:uOrder = [2, 2, 2, 2, 2, 2]
        uniform uint[] brep:surface:nurb:uVertexCount = [2, 2, 2, 2, 2, 2]
        uniform double[] brep:surface:nurb:vKnots = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
        uniform uint[] brep:surface:nurb:vOrder = [2, 2, 2, 2, 2, 2]
        uniform uint[] brep:surface:nurb:vVertexCount = [2, 2, 2, 2, 2, 2]
        uniform double[] brep:surface:nurb:weights = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        uniform point3d[] brep:vertexPoint:point:position = [(1, 1, 1), (0, 0, 1), (0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 1, 1), (0, 1, 0), (1, 1, 0)]
        uniform token[] edge:curveType = ["BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI", "BrepCurve3dNurbAPI"]
        uniform double[] edge:range = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
        uniform int2[] edge:vertexIndices = [(4, 0), (5, 0), (2, 6), (1, 5), (3, 7), (7, 0), (2, 1), (2, 3), (3, 4), (1, 4), (6, 5), (6, 7)]
        uniform uint[] edgeuse:edgeIndex = [2, 11, 4, 7, 3, 9, 0, 1, 6, 3, 10, 2, 8, 4, 5, 0, 6, 7, 8, 9, 10, 1, 5, 11]
        uniform uint[] edgeuse:nextRadialEUIndex = [11, 23, 13, 17, 9, 19, 15, 21, 16, 4, 20, 0, 18, 2, 22, 6, 8, 3, 12, 5, 10, 7, 14, 1]
        uniform token[] edgeuse:orientationType = ["same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite", "opposite", "same", "same", "opposite", "opposite", "same", "same", "opposite", "same", "same", "opposite", "opposite"]
        uniform token[] edgeuse:thisRadialEntryType = ["topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "bottomEntry", "bottomEntry", "topEntry", "topEntry", "bottomEntry", "topEntry", "topEntry", "topEntry", "topEntry", "bottomEntry", "bottomEntry", "bottomEntry", "bottomEntry"]
        uniform float3[] extent = [(0, 0, 0), (1, 1, 1)]
        uniform uint[] face:loopCount = [1, 1, 1, 1, 1, 1]
        uniform double2[] face:range = [(0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1), (0, 0), (1, 1)]
        uniform token[] face:surfaceType = ["BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI", "BrepSurfaceNurbAPI"]
        uniform token[] face:trimType = ["general", "general", "general", "general", "general", "general"]
        uniform uint[] faceuse:faceIndex = [5, 4, 2, 0, 3, 1, 5, 1, 4, 0, 3, 2]
        uniform token[] faceuse:orientationType = ["same", "same", "same", "same", "same", "same", "opposite", "opposite", "opposite", "opposite", "opposite", "opposite"]
        uniform uint[] loop:edgeuseCount = [4, 4, 4, 4, 4, 4]
        uniform uint[] loop:vertexIndex = [9999999, 9999999, 9999999, 9999999, 9999999, 9999999]
        rel material:binding = None
        uniform uint[] region:shellCount = [1, 1]
        uniform token[] region:type = ["voidRegion", "solidRegion"]
        uniform uint[] shell:faceuseCount = [6, 6]
        uniform token[] shell:pointType = ["none", "none"]
        uniform uint[] shell:wireEdgeCount = [0, 0]
        uniform token[] vertex:pointType = ["BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI", "BrepPointAPI"]
        uniform token[] wireEdge:curveType = []
        uniform double[] wireEdge:range = []
        uniform int2[] wireEdge:vertexIndices = []

        def GeomSubset "subset_0" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {
            uniform token elementType = "face"
            uniform int[] indices = [0, 1, 2]
            rel material:binding = </World/Looks/Black>
        }

        def GeomSubset "subset_1" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {
            uniform token elementType = "face"
            uniform int[] indices = [3, 4, 5]
            rel material:binding = </World/Looks/Green>
        }
    }

    def "Looks"
    {
        def Material "Black"
        {
            token outputs:surface.connect = </World/Looks/Black/PBRShader.outputs:surface>
            def Shader "PBRShader"
            {
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = (0, 0, 0)
                token outputs:surface
            }
        }
        def Material "Green"
        {
            token outputs:surface.connect = </World/Looks/Green/PBRShader.outputs:surface>
            def Shader "PBRShader"
            {
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = (0.463, 0.725, 0)
                token outputs:surface
            }
        }
    }
}

```
</details>

## References

- Lee, K., 1999. _Principles of CAD/CAM/CAE Systems._ Addison-Wesley Longman Publishing Co., Inc., 582 pp.
- Weiler, K., 1986. _Topological Structures for Geometric Modeling._ PhD thesis, Rensselaer Polytechnic Institute. [Full text](https://webserver2.tecgraf.puc-rio.br/~lfm/teses/KevinWeiler-Doutorado-1986.pdf).
- ISO 14739-1:2014. _Document management — 3D use of Product Representation Compact (PRC) format — Part 1: PRC 10001._ International Organization for Standardization.
- ISO 10303 (STEP). _Industrial automation systems and integration — Product data representation and exchange._ International Organization for Standardization.
- Hertel, T., and Fuchs, A., 2024. _AOUSD Geometry WG CAD and BIM use cases_ (v0.2, internal working document). AOUSD Geometry Working Group.
