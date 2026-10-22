# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Sphinx directive for interactive USD BRep models, baked to GLB during the build (``brep_glb.py``).

- ``.. brep-mesh::`` with Python as content: the BRep the code leaves in ``model``; the code is shown under it.
- ``.. brep-mesh:: script.py`` with Python as content: the content runs after the script's definitions.
- ``.. brep-mesh:: part.smb``: a native SMLib file.

Options: ``:name:``, the GLB's name, and ``:angle:``, the surface angle tolerance in degrees (default 12).
The viewer's script and stylesheet go on the pages that use it, with an import map that loads three.js from a CDN.
"""
import hashlib
import json
import os
import runpy
from pathlib import Path

from docutils import nodes
from docutils.parsers.rst import directives
from sphinx.util.docutils import SphinxDirective

STATIC = Path(__file__).resolve().parent / "static"
THREE = "https://cdn.jsdelivr.net/npm/three@0.186.1/"
IMPORT_MAP = json.dumps({"imports": {"three": THREE + "build/three.module.min.js", "three/addons/": THREE + "examples/jsm/"}})
BUTTONS = "".join(f'<button data-mode="{mode}">{label}</button>'
                  for mode, label in (("shaded", "Shaded"), ("both", "Shaded + edges"), ("edges", "Edges")))


def baked_models(doctreedir):
    # Next to the doctrees, so incremental builds keep them; copied into _static/brep-mesh/models.
    return Path(doctreedir) / "brep_mesh" / "brep-mesh" / "models"


class BrepMesh(SphinxDirective):
    optional_arguments = 1
    has_content = True
    option_spec = {"name": directives.unchanged, "angle": float}

    def model(self, source, code):
        """The BRep to bake."""
        if not source:
            scope = {}
            exec(code, scope)
            return scope["model"]
        relative, path = self.env.relfn2path(source)
        self.env.note_dependency(relative)
        if Path(path).suffix == ".smb":
            import usd_brep

            return usd_brep.read_brep_from_file(str(path))
        scope = runpy.run_path(str(path))
        exec(code, scope)
        return scope["model"]

    def run(self):
        from brep_glb import bake  # imports usd_brep and pxr: only once a page bakes a model

        source, code = (self.arguments or [""])[0], "\n".join(self.content)
        # Named by default after its page and what it shows, not its line: editing the page around it leaves no
        # stale GLB behind, and two views of one source don't overwrite each other.
        key = hashlib.sha256(f"{self.env.docname}\n{source}\n{code}\n{self.options.get('angle')}".encode())
        name = self.options.get("name", f"{Path(source).stem or 'model'}_{key.hexdigest()[:8]}")
        bake(self.model(source, code), baked_models(self.env.doctreedir) / f"{name}.glb", self.options.get("angle", 12.0))
        src = "../" * self.env.docname.count("/") + f"_static/brep-mesh/models/{name}.glb"
        viewer = nodes.raw("", f'<div class="brep-mesh" data-src="{src}"><canvas></canvas>'
                               f'<div class="bar">{BUTTONS}</div><div class="info"></div></div>', format="html")
        if code and not source:
            return [viewer, nodes.literal_block(code, code, language="python")]
        return [viewer]


def add_static(app):
    baked_models(app.doctreedir).mkdir(parents=True, exist_ok=True)
    app.config.html_static_path += [str(STATIC), str(baked_models(app.doctreedir).parents[1])]


def add_assets(app, pagename, templatename, context, doctree):
    # Called per page: files added here go on this page only.
    if doctree and any('class="brep-mesh"' in raw.astext() for raw in doctree.findall(nodes.raw)):
        app.add_css_file("brep-mesh/viewer.css")
        app.add_js_file(None, body=IMPORT_MAP, type="importmap")
        app.add_js_file("brep-mesh/viewer.js", type="module")


def setup(app):
    # A build tree's usd_brep doesn't find the omniSolid schema by itself; repo test sets the same path.
    os.environ.setdefault("OMNISOLID_PLUGIN_PATH", str(STATIC.parents[2] / "_build" / "schema" / "omniSolid" / "resources"))
    app.add_directive("brep-mesh", BrepMesh)
    app.connect("builder-inited", add_static)
    app.connect("html-page-context", add_assets)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
