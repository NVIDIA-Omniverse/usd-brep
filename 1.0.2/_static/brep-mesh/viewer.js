// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Viewer for the docs' GLBs, B-reps converted to meshes by brep_glb.py (a node per B-rep face). Faces and B-rep edges
// are pickable. "three" resolves through the import map that brep_mesh.py puts on the page.
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { LineSegments2 } from "three/addons/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "three/addons/lines/LineSegmentsGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

// Colors come from CSS custom properties on the viewer (--brep-view/-edge/-pick, set from the docs theme).
// Re-read when the theme switches.
const palette = (card) => {
  const style = getComputedStyle(card);
  const read = (name) => new THREE.Color(style.getPropertyValue(`--brep-${name}`).trim());
  return { view: read("view"), edge: read("edge"), pick: read("pick") };
};
const refreshers = [];
const refreshAll = () => refreshers.forEach((refresh) => refresh());
new MutationObserver(refreshAll).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme", "class"] });
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", refreshAll);
const fmt = (x) => x.toLocaleString(undefined, { maximumSignificantDigits: 4 });
const SURFACE = { Nurb: "NURBS" };
const surface = (type) => `${SURFACE[type] ?? type.toLowerCase()} surface`;
const edgeText = (length, faces, seam, units) =>
  `length ${fmt(length)} ${units} · ${seam ? "seam" : `${faces} adjacent face${faces === 1 ? "" : "s"}`}`;
const MODE_TIPS = {
  shaded: "Show the B-rep's faces",
  both: "Show the faces and the B-rep edges: the true boundary curves, not mesh edges",
  edges: "Show only the B-rep edges",
  xray: "See through the faces: hidden B-rep edges are dashed",
};
const OFFSET = { polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1 }; // edges on a surface win

// One WebGL context for the whole page (browsers allow ~16): each viewer renders with it, then copies the
// frame into its own 2D canvas.
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
const environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
const loader = new GLTFLoader();

// Parse each model when its card nears the viewport.
const observer = new IntersectionObserver((entries) => {
  for (const entry of entries.filter((e) => e.isIntersecting)) {
    observer.unobserve(entry.target);
    setUp(entry.target);
  }
}, { rootMargin: "200px" });
document.querySelectorAll(".brep-mesh").forEach((card) => observer.observe(card));

// One solid: a mesh per B-rep face and a LINES node with the edges (extras: edges, edgeOfSegment).
function solid(root, summary) {
  const faces = [];
  let lines = null;
  root.traverse((o) => {
    if (o.isMesh) faces.push(o);
    else if (o.isLineSegments) lines = o;
  });
  // Own material per face so a highlight tints one face.
  faces.forEach((face) => (face.material = Object.assign(face.material.clone(), OFFSET)));
  const { edges, edgeOfSegment } = lines.userData, { index, attributes } = lines.geometry, u = summary.units;
  const segments = new Float32Array(index.count * 3);
  index.array.forEach((v, i) => segments.set(attributes.position.array.subarray(3 * v, 3 * v + 3), 3 * i));
  lines.removeFromParent();
  return {
    faces, segments, edgeOfSegment,
    summary: `${summary.faces} faces · ${summary.edges} edges · ${summary.vertices} vertices · hover to inspect`,
    faceAt: ({ object: face }) => {
      const d = face.userData;
      const area = d.exact === undefined ? `mesh area ${fmt(d.area)} ${u}²`
        : `area ${fmt(d.exact)} ${u}² (mesh ${((d.area / d.exact - 1) * 100).toFixed(2)}%)`;
      return { id: `f${face.id}`, text: `Face ${d.face} · ${surface(d.surface)} · ${area}`,
               highlight: (color) => face.material.emissive.copy(color).multiplyScalar(0.45),
               clear: () => face.material.emissive.setRGB(0, 0, 0) };
    },
    edgeText: (e) => `Edge ${e} · ${edgeText(edges[e].length, edges[e].faces, edges[e].seam, u)}`,
  };
}

async function setUp(card) {
  const canvas = card.querySelector("canvas"), info = card.querySelector(".info");
  const context = canvas.getContext("2d");
  let colors = palette(card);
  const scene = new THREE.Scene();
  scene.background = colors.view;
  scene.environment = environment;
  const camera = new THREE.PerspectiveCamera(30, 1, 0.001, 1000);
  const controls = new OrbitControls(camera, canvas);

  const gltf = await loader.loadAsync(card.dataset.src).catch((error) => console.error(card.dataset.src, error));
  if (!gltf) return (info.textContent = "Could not load this model.");
  const root = gltf.scene;
  scene.add(root);
  const model = solid(root, root.userData);
  const { faces, segments, edgeOfSegment } = model;

  // transparent only to be drawn after the faces, which X-ray makes translucent.
  const edges = new LineSegments2(new LineSegmentsGeometry().setPositions(segments),
                                  new LineMaterial({ linewidth: 1.6, vertexColors: true, transparent: true }));
  edges.renderOrder = 2;
  root.add(edges);
  const edgeColors = new Float32Array(segments.length);
  edges.geometry.setColors(edgeColors);
  const paint = ([from, to], color) => {
    for (let i = 6 * from; i < 6 * to; i += 3) color.toArray(edgeColors, i);
    edges.geometry.attributes.instanceColorStart.data.needsUpdate = true;
  };
  // X-ray: depth-only copies of the faces decide what is hidden; hidden edges are drawn again, dashed and dimmer.
  const hiddenEdges = new LineSegments2(edges.geometry, new LineMaterial({
    linewidth: 1.2, vertexColors: true, transparent: true, opacity: 0.5, dashed: true, depthFunc: THREE.GreaterDepth }));
  hiddenEdges.renderOrder = 3;
  root.add(hiddenEdges);
  const depthOnly = new THREE.MeshBasicMaterial({ colorWrite: false, ...OFFSET });
  const occluders = faces.map((f) => {
    const occluder = new THREE.Mesh(f.geometry, depthOnly);
    occluder.renderOrder = -1;
    f.add(occluder);
    return occluder;
  });

  const box = new THREE.Box3();
  faces.forEach((f) => box.expandByObject(f));
  const center = box.getCenter(new THREE.Vector3()), radius = box.getSize(new THREE.Vector3()).length() / 2;
  Object.assign(hiddenEdges.material, { dashSize: radius * 0.03, gapSize: radius * 0.02 });
  hiddenEdges.computeLineDistances();
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(new THREE.Vector3(1, 0.8, 1.4).normalize(), radius / Math.sin(THREE.MathUtils.degToRad(15)));
  Object.assign(camera, { near: radius / 100, far: radius * 100 });
  controls.update();

  const render = () => {
    const w = canvas.clientWidth, h = canvas.clientHeight, ratio = devicePixelRatio;
    renderer.setPixelRatio(ratio);
    renderer.setSize(w, h, false);
    renderer.render(scene, camera);
    if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) {
      Object.assign(canvas, { width: Math.round(w * ratio), height: Math.round(h * ratio) });
    }
    context.drawImage(renderer.domElement, 0, 0, canvas.width, canvas.height);
  };
  const resize = () => {
    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
    render();
  };
  controls.addEventListener("change", render);
  new ResizeObserver(resize).observe(canvas);
  resize();

  // An edge's segments are consecutive.
  const edgeAt = (s) => {
    const e = edgeOfSegment[s];
    let from = s, to = s + 1;
    while (from > 0 && edgeOfSegment[from - 1] === e) from--;
    while (to < edgeOfSegment.length && edgeOfSegment[to] === e) to++;
    return { id: `e${e}`, text: model.edgeText(e), range: [from, to] };
  };
  let picked = null; // { id, text, highlight?, clear? } for a face, { id, text, range } for an edge
  const pick = (next) => {
    if (picked?.id === next?.id) return;
    picked?.clear?.();
    if (picked?.range) paint(picked.range, colors.edge);
    picked = next;
    picked?.highlight?.(colors.pick);
    if (picked?.range) paint(picked.range, colors.pick);
    info.textContent = picked?.text ?? model.summary;
    render();
  };
  paint([0, edgeOfSegment.length], colors.edge);
  info.textContent = model.summary;
  refreshers.push(() => {
    colors = palette(card);
    scene.background = colors.view;
    paint([0, edgeOfSegment.length], colors.edge);
    picked?.highlight?.(colors.pick);
    if (picked?.range) paint(picked.range, colors.pick);
    render();
  });

  // Edges are picked in screen space: three's 3D line raycast took ~70 ms a hover on 500k segments.
  // The segment ends are projected once per camera move: x, y in CSS pixels and the view depth.
  const screen = new Float32Array(segments.length), toClip = new THREE.Matrix4();
  let projected = false;
  const project = () => {
    const e = toClip.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse).elements;
    const w = canvas.clientWidth / 2, h = canvas.clientHeight / 2;
    for (let i = 0; i < segments.length; i += 3) {
      const x = segments[i], y = segments[i + 1], z = segments[i + 2], depth = e[3] * x + e[7] * y + e[11] * z + e[15];
      screen[i] = depth > 0 ? ((e[0] * x + e[4] * y + e[8] * z + e[12]) / depth + 1) * w : NaN; // behind the camera: never hit
      screen[i + 1] = (1 - (e[1] * x + e[5] * y + e[9] * z + e[13]) / depth) * h;
      screen[i + 2] = depth;
    }
    projected = true;
  };
  controls.addEventListener("change", () => (projected = false));
  new ResizeObserver(() => (projected = false)).observe(canvas);
  // The nearest segment within 6 px of (x, y) and in front of maxDepth, or -1.
  const edgeNear = (x, y, maxDepth) => {
    if (!projected) project();
    let best = -1;
    for (let s = 0, i = 0; s < edgeOfSegment.length; s++, i += 6) {
      const x0 = screen[i], y0 = screen[i + 1], x1 = screen[i + 3], y1 = screen[i + 4];
      if (Math.min(x0, x1) > x + 6 || Math.max(x0, x1) < x - 6 || Math.min(y0, y1) > y + 6 || Math.max(y0, y1) < y - 6) continue;
      const dx = x1 - x0, dy = y1 - y0, t = Math.min(1, Math.max(0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy || 1)));
      const depth = screen[i + 2] + t * (screen[i + 5] - screen[i + 2]);
      if ((x0 + t * dx - x) ** 2 + (y0 + t * dy - y) ** 2 <= 36 && depth < maxDepth) [best, maxDepth] = [s, depth];
    }
    return best;
  };

  const raycaster = new THREE.Raycaster(), forward = new THREE.Vector3();
  const hover = (clientX, clientY) => {
    const r = canvas.getBoundingClientRect();
    raycaster.setFromCamera(new THREE.Vector2(((clientX - r.left) / r.width) * 2 - 1, 1 - ((clientY - r.top) / r.height) * 2), camera);
    const face = raycaster.intersectObjects(faces.filter((f) => f.visible), false)[0];
    // Edges have no depth of their own here: drop hits behind the nearest face, except in X-ray.
    const maxDepth = mode === "xray" || !face ? Infinity
      : (face.distance + radius * 0.02) * raycaster.ray.direction.dot(camera.getWorldDirection(forward));
    const edge = edges.visible ? edgeNear(clientX - r.left, clientY - r.top, maxDepth) : -1;
    pick(edge >= 0 ? edgeAt(edge) : face ? model.faceAt(face) : null);
  };
  // At most one pick per frame.
  let pointer = null;
  canvas.addEventListener("pointermove", (e) => {
    if (e.buttons) return;
    if (!pointer) requestAnimationFrame(() => { if (pointer) hover(...pointer); pointer = null; });
    pointer = [e.clientX, e.clientY];
  });
  canvas.addEventListener("pointerleave", () => { pointer = null; pick(null); });

  // Display mode: the page's buttons, plus X-ray after "Edges".
  let mode = "both";
  const xrayButton = Object.assign(document.createElement("button"), { textContent: "X-ray" });
  xrayButton.dataset.mode = "xray";
  card.querySelector('button[data-mode="edges"]')?.after(xrayButton);
  card.querySelectorAll("button[data-mode]").forEach((b) => (b.title ||= MODE_TIPS[b.dataset.mode] ?? ""));
  // Edge width in pixels; hidden edges stay a little thinner.
  const width = Object.assign(document.createElement("input"), { type: "range", min: 0.5, max: 5, step: 0.1, value: edges.material.linewidth });
  const widthLabel = Object.assign(document.createElement("label"), { className: "edge-width", title: "Width of the B-rep edge lines" });
  widthLabel.append("Edge width", width);
  xrayButton.after(widthLabel);
  width.addEventListener("input", () => {
    edges.material.linewidth = Number(width.value);
    hiddenEdges.material.linewidth = 0.75 * width.value;
    render();
  });
  const apply = () => {
    const xray = mode === "xray";
    faces.forEach((f) => {
      f.visible = mode !== "edges";
      // X-ray blends every layer of faces; the occluders supply the depth the edges are tested against.
      Object.assign(f.material, { transparent: xray, opacity: xray ? 0.25 : 1, depthWrite: !xray, depthTest: !xray });
      f.material.needsUpdate = true;
    });
    occluders.forEach((o) => (o.visible = xray));
    edges.visible = mode !== "shaded";
    width.disabled = !edges.visible;
    hiddenEdges.visible = xray;
    card.querySelectorAll("button[data-mode]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.mode === mode)));
    pick(null);
    render();
  };
  card.querySelectorAll("button[data-mode]").forEach((b) => b.addEventListener("click", () => { mode = b.dataset.mode; apply(); }));
  apply();

  // Full screen, Esc to leave. Where the browser refuses (an iframe, no user gesture), fill the window instead.
  const maximize = Object.assign(document.createElement("button"), { className: "maximize" });
  card.querySelector(".bar").append(maximize);
  const maximized = () => document.fullscreenElement === card || card.classList.contains("expanded");
  const label = () => {
    maximize.textContent = maximized() ? "Exit full screen" : "Full screen";
    maximize.title = maximized() ? "Back to the page (Esc)" : "Fill the screen with this model (Esc to exit)";
    maximize.setAttribute("aria-pressed", String(maximized()));
  };
  const toggleMaximized = async () => {
    if (document.fullscreenElement === card) await document.exitFullscreen();
    else if (card.classList.contains("expanded")) card.classList.remove("expanded");
    else {
      try {
        await card.requestFullscreen();
      } catch {
        card.classList.add("expanded");
      }
    }
    label();
  };
  maximize.addEventListener("click", toggleMaximized);
  document.addEventListener("fullscreenchange", label);
  document.addEventListener("keydown", (e) => e.key === "Escape" && card.classList.contains("expanded") && toggleMaximized());
  label();
}
