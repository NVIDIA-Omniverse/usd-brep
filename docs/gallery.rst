Gallery
=======

Interactive USD BRep models, converted to meshes for the browser. Drag to rotate, scroll to zoom,
right-drag to pan. Hover a face or an edge to inspect it: faces report their surface type and exact
area, edges their length.

Built with the Python API
-------------------------

Each model is the output of the code under it.

CSG: intersect, then subtract
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. brep-mesh::
   :name: csg

   import usd_brep as sm

   box = sm.create_box((-5, -5, -5), 10, 10, 10)
   body = sm.boolean_intersection(box, sm.create_sphere((0, 0, 0), 6.5))
   # Unequal radii, so the holes never meet tangentially.
   for axis, radius in [(None, 3), ((1, 0, 0), 2.5), ((0, 1, 0), 2)]:
       hole = sm.create_cylinder((0, 0, -8), radius, 16)
       if axis:
           sm.rotate(hole, (0, 0, 0), axis, 90)
       body = sm.boolean_difference(body, hole)
   model = body

Plate with holes: boolean difference
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. brep-mesh::
   :name: plate

   import usd_brep as sm

   plate = sm.create_box((0, 0, 0), 40, 24, 4)
   for x, y in [(5, 5), (35, 5), (5, 19), (35, 19)]:
       hole = sm.create_cylinder((x, y, -1), 1.6, 6)
       plate = sm.boolean_difference(plate, hole)
   bore = sm.create_cylinder((20, 12, -1), 6, 6)
   model = sm.boolean_difference(plate, bore)

Bracket: union, then fillet the inside edge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. brep-mesh::
   :name: bracket

   import usd_brep as sm

   base = sm.create_box((0, 0, 0), 20, 12, 3)
   wall = sm.create_box((0, 0, 0), 3, 12, 16)
   bracket = sm.boolean_union(base, wall)

   def inside_corner(edge):  # runs along y at x = z = 3
       (x0, _, z0), (x1, _, z1) = edge.bounding_box(tight=True)
       return max(abs(v - 3) for v in (x0, z0, x1, z1)) < 1e-6

   edges = [e for e in bracket.edges() if inside_corner(e)]
   sm.fillet_edges(bracket, edges, radius=2.0)
   model = bracket

Rounded box: fillet every edge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. brep-mesh::
   :name: rounded_box

   import usd_brep as sm

   model = sm.create_box((0, 0, 0), 20, 12, 8)
   sm.circular_fillet(model, 2.0)

Tray: shell a box, removing the top face
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. brep-mesh::
   :name: tray

   import usd_brep as sm

   box = sm.create_box((0, 0, 0), 20, 14, 8)
   top = next(f for f in box.faces()
              if f.bounding_box(tight=True)[0][2] == 8)
   model = sm.shell_brep(box, -1.0, create_solid=True,
                         faces_to_shell=[top])

Pulley: revolve a closed profile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. brep-mesh::
   :name: pulley

   import usd_brep as sm

   # Closed (radius, height) profile with a V groove, in the xz plane.
   rz = [(2, 0), (10, 0), (10, 3), (8.5, 4), (10, 5), (10, 8), (2, 8),
         (2, 0)]
   profile = [sm.create_line_segment((r0, 0, z0), (r1, 0, z1))
              for (r0, z0), (r1, z1) in zip(rz, rz[1:])]
   model = sm.rotational_sweep(profile, (0, 0, 0), (0, 0, 1), 360,
                               cap_ends=True)

Spring: pipe sweep along a helix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. brep-mesh::
   :name: spring
   :angle: 20

   import usd_brep as sm

   helix = sm.create_helix((0, 0, 0), height=10, radius_start=2,
                           radius_end=2, turns=5)
   model = sm.pipe_sweep(helix, radius=0.3)

Test parts
----------

Parts from the repository's test data, ``TestFiles/pt_TestFiles``.

.. grid:: 1 1 2 2
   :gutter: 3

   .. grid-item-card:: LShape II

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Defeature/LShape_II.smb

      ``Defeature/LShape_II.smb`` · 25 faces

   .. grid-item-card:: Platte

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Defeature/Platte.smb

      ``Defeature/Platte.smb`` · 122 faces

   .. grid-item-card:: Defillet

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Defeature/defillet.smb

      ``Defeature/defillet.smb`` · 9 faces

   .. grid-item-card:: Extrusion

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Defeature/extrusion.smb

      ``Defeature/extrusion.smb`` · 11 faces

   .. grid-item-card:: Box diff 3cyls

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Fillet/box_diff_3cyls.smb

      ``Fillet/box_diff_3cyls.smb`` · 13 faces

   .. grid-item-card:: Box w slot

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Fillet/box_w_slot.smb

      ``Fillet/box_w_slot.smb`` · 27 faces

   .. grid-item-card:: Cliff 1

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Fillet/cliff_1.smb

      ``Fillet/cliff_1.smb`` · 43 faces

   .. grid-item-card:: Crown

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Fillet/crown.smb

      ``Fillet/crown.smb`` · 13 faces

   .. grid-item-card:: Cyl on box w tan hole2

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Fillet/cyl_on_box_w_tan_hole2.smb

      ``Fillet/cyl_on_box_w_tan_hole2.smb`` · 13 faces

   .. grid-item-card:: Rollover corner

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Fillet/rollover_corner.smb

      ``Fillet/rollover_corner.smb`` · 8 faces

   .. grid-item-card:: Boolean trees result

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Solids/boolean_trees_result.smb

      ``Solids/boolean_trees_result.smb`` · 15 faces

   .. grid-item-card:: Extrusion2

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Solids/extrusion2.smb

      ``Solids/extrusion2.smb`` · 26 faces

   .. grid-item-card:: Offset bracket2

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Solids/offset_bracket2.smb

      ``Solids/offset_bracket2.smb`` · 59 faces

   .. grid-item-card:: Revolution 360

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Solids/revolution_360.smb

      ``Solids/revolution_360.smb`` · 29 faces

   .. grid-item-card:: Revolution mold

      .. brep-mesh:: ../TestFiles/pt_TestFiles/Solids/revolution_mold.smb

      ``Solids/revolution_mold.smb`` · 33 faces
