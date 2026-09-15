GENERIC CONSTRUCTION PIECES — 27 FBX MODELS

Scale: one Unity unit = one meter. Reference Brick_2x3 has a 0.2 x 0.3
footprint, a 0.12 body height, and 0.1425 total height including studs.
Stud center spacing: 0.1. Stud diameter: 0.06. Stud height: 0.0225.
Plate and tile body height: 0.04. Edge bevel: 0.0015, three segments.

Each FBX contains one triangulated mesh object, one neutral plastic material,
and UV0 coordinates. Bricks and plates have hollow undersides, with support
tubes on two-wide pieces and posts on single-row pieces. Geometry is welded
with Boolean unions; every exported mesh is checked for manifold edges.
Pivots are centered on the bottom face. FBX is exported Y-up, -Z-forward,
with meter units and baked axis conversion. Use Unity's file unit conversion
and Scale Factor 1. No textures, animations, colliders, or LODs are included.
Import normals to preserve the rounded-edge shading.

The manifest gives dimensions in the source X/Y/Z axes (width/length/height),
triangle counts and FBX re-import verification results. Unity axes are
width/height/length. Roof prism and plain triangular prism intentionally
share the same basic shape, as in the reference sheet.

preview.png is rendered from the actual meshes, not generated concept art.
generate_models.py regenerates the pack using Blender 4.2:
  blender -b -t 4 --python generate_models.py

These are original generic modeling assets. This package is not legal
clearance of any particular construction-toy part or brand.
