# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Bake a BRep to a GLB for the docs viewer: a mesh node per B-rep face and a LINES node with the B-rep edges.

``usd.tessellate_file`` carries a BrepArray's face GeomSubsets onto the triangles, so each face first gets a
subset of its own. Exact areas, edge lengths and adjacency come from the kernel; glTF ``extras`` hold them.
"""
import json
import struct
import tempfile
from array import array
from pathlib import Path

import usd_brep as sm
from pxr import Usd, UsdGeom, UsdShade

UNITS = {1.0: "m", 0.01: "cm", 0.001: "mm"}


def to_gltf(point, scale=1.0):
    """Model space (Z-up) to glTF space (Y-up)."""
    x, y, z = (c * scale for c in point)
    return x, z, -y


class Glb:
    """Minimal glTF 2.0 binary writer: one node per mesh, one buffer."""

    def __init__(self):
        self.blob = bytearray()
        self.doc = {"asset": {"version": "2.0", "generator": "usd-brep docs brep_glb.py"}, "scene": 0,
                    "scenes": [{"nodes": []}], "nodes": [], "meshes": [], "accessors": [], "bufferViews": [],
                    "materials": [{"doubleSided": True, "pbrMetallicRoughness": {
                        "baseColorFactor": [0.30, 0.33, 0.38, 1.0], "metallicFactor": 0.0, "roughnessFactor": 0.55}}]}

    def _accessor(self, data, kind, target):
        self.blob.extend(bytes(-len(self.blob) % 4))
        self.doc["bufferViews"].append({"buffer": 0, "byteOffset": len(self.blob),
                                        "byteLength": len(data) * data.itemsize, "target": target})
        self.blob.extend(data.tobytes())
        accessor = {"bufferView": len(self.doc["bufferViews"]) - 1, "type": kind,
                    "componentType": 5126 if data.typecode == "f" else 5125,
                    "count": len(data) // 3 if kind == "VEC3" else len(data)}
        if kind == "VEC3":
            accessor["min"] = [min(data[i::3]) for i in range(3)]
            accessor["max"] = [max(data[i::3]) for i in range(3)]
        self.doc["accessors"].append(accessor)
        return len(self.doc["accessors"]) - 1

    def node(self, name, positions, indices, normals=None, extras=None):
        """Shaded triangles when given normals, else lines."""
        primitive = {"attributes": {"POSITION": self._accessor(positions, "VEC3", 34962)},
                     "indices": self._accessor(indices, "SCALAR", 34963), "mode": 1}
        if normals is not None:
            primitive["attributes"]["NORMAL"] = self._accessor(normals, "VEC3", 34962)
            primitive.update(mode=4, material=0)
        self.doc["meshes"].append({"name": name, "primitives": [primitive]})
        self.doc["nodes"].append({"name": name, "mesh": len(self.doc["meshes"]) - 1, "extras": extras})
        self.doc["scenes"][0]["nodes"].append(len(self.doc["nodes"]) - 1)

    def write(self, path, extras):
        self.blob.extend(bytes(-len(self.blob) % 4))
        self.doc["scenes"][0]["extras"] = extras
        text = json.dumps({**self.doc, "buffers": [{"byteLength": len(self.blob)}]}, separators=(",", ":")).encode()
        text += b" " * (-len(text) % 4)
        Path(path).write_bytes(struct.pack("<4sII", b"glTF", 2, 28 + len(text) + len(self.blob))
                               + struct.pack("<I4s", len(text), b"JSON") + text
                               + struct.pack("<I4s", len(self.blob), b"BIN\0") + bytes(self.blob))


def mark_faces(path):
    """Bind each B-rep face of the BrepArray in `path` to marker material f<k>; return the surface types."""
    stage = Usd.Stage.Open(path)
    brep = next(p for p in stage.Traverse() if p.GetTypeName() == "BrepArray")
    types = [str(t).removeprefix("BrepSurface").removesuffix("API") for t in brep.GetAttribute("face:surfaceType").Get()]
    for k in range(len(types)):
        subset = UsdGeom.Subset.Define(stage, brep.GetPath().AppendChild(f"brepFace_{k}"))
        subset.CreateElementTypeAttr(UsdGeom.Tokens.face)
        subset.CreateIndicesAttr([k])
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(UsdShade.Material.Define(stage, f"/BrepFaceMarkers/f{k}"))
    stage.Save()
    return types


def faces_of(mesh, meters):
    """{B-rep face: (positions, normals, indices)}: the mesh's triangles by face, welded, in glTF space."""
    face_of = {}
    for subset in UsdGeom.Subset.GetAllGeomSubsets(mesh):
        marker = UsdShade.MaterialBindingAPI(subset.GetPrim()).GetDirectBinding().GetMaterialPath().name
        face_of.update(dict.fromkeys(subset.GetIndicesAttr().Get(), int(marker[1:])))
    points, indices = mesh.GetPointsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
    normals = UsdGeom.PrimvarsAPI(mesh).GetPrimvar("normals").ComputeFlattened()  # one per corner
    faces = {}
    for triangle in range(len(indices) // 3):
        positions, out_normals, out_indices, weld = faces.setdefault(face_of[triangle], (array("f"), array("f"), array("I"), {}))
        for corner in range(3 * triangle, 3 * triangle + 3):
            p, n = to_gltf(points[indices[corner]], meters), to_gltf(normals[corner])
            key = tuple(round(x, 7) for x in (*p, *n))
            if key not in weld:
                weld[key] = len(positions) // 3
                positions.extend(p)
                out_normals.extend(n)
            out_indices.append(weld[key])
    return {face: data[:3] for face, data in faces.items()}


def mesh_area(positions, indices):
    total = 0.0
    for t in range(0, len(indices), 3):
        a, b, c = (positions[3 * i:3 * i + 3] for i in indices[t:t + 3])
        u, v = [b[j] - a[j] for j in range(3)], [c[j] - a[j] for j in range(3)]
        total += 0.5 * sum((u[(j + 1) % 3] * v[(j + 2) % 3] - u[(j + 2) % 3] * v[(j + 1) % 3]) ** 2
                           for j in range(3)) ** 0.5
    return total


def bake(brep, path, angle=10.0):
    """Write `brep` to `path` as a GLB, tessellated to `angle` degrees of surface angle tolerance."""
    glb = Glb()
    with tempfile.TemporaryDirectory(prefix="brep-mesh-", ignore_cleanup_errors=True) as tmp:
        source, baked = str(Path(tmp, "brep.usda")), str(Path(tmp, "baked.usda"))
        sm.usd.export_brep(brep, source)
        types = mark_faces(source)
        (result,) = sm.usd.tessellate_file(source, baked, chord_height_tolerance=0.0, surface_angle_tolerance_deg=angle)
        if not result.ok:
            raise RuntimeError(f"tessellate_file: {result.status_name} {result.message}")
        stage = Usd.Stage.Open(baked)
        meters = UsdGeom.GetStageMetersPerUnit(stage)
        exact = [face.area() for face in brep.faces()]  # in the USD face order
        for k, (positions, normals, indices) in sorted(faces_of(UsdGeom.Mesh(stage.GetPrimAtPath(result.mesh_path)), meters).items()):
            glb.node(f"face_{k}", positions, indices, normals, {
                "face": k, "surface": types[k], "exact": round(exact[k], 6),
                "area": round(mesh_area(positions, indices) / meters ** 2, 6)})

    points, segments, edge_of_segment, edges = array("f"), array("I"), [], []
    samples, counts, handles = sm.tessellate_boundaries(brep, angle_tolerance_deg=5.0)
    solid, start = brep.is_manifold_solid(), 0
    for count, edge in zip(counts, handles):
        base = len(points) // 3
        for p in samples[start:start + count]:
            points.extend(to_gltf(p, meters))
        for j in range(count - 1):
            segments.extend((base + j, base + j + 1))
            edge_of_segment.append(len(edges))
        # On a manifold solid every edge has two uses, so a single adjacent face means a seam.
        adjacent = len(edge.faces())
        edges.append({"length": round(edge.length(), 4), "faces": adjacent, "seam": solid and adjacent == 1})
        start += count
    glb.node("edges", points, segments, extras={"edges": edges, "edgeOfSegment": edge_of_segment})
    glb.write(path, {"faces": len(types), "edges": len(edges), "vertices": brep.vertex_count(),
                     "units": UNITS.get(meters, "units")})
