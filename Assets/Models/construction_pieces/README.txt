GENERIC CONSTRUCTION PIECES — 27 FBX MODELS

Scale: one Unity unit = one meter. Reference Brick_2x3 has a 0.2 x 0.3
footprint, a 0.12 body height, and 0.1425 total height including studs.
Stud center spacing: 0.1. Stud diameter: 0.06. Stud height: 0.0225.
Plate and tile body height: 0.04. Edge chamfer: 0.0015, one segment.

Each FBX contains one triangulated mesh object, one neutral plastic material,
and UV0 coordinates. Bricks and plates have hollow undersides, with support
tubes on two-wide pieces and posts on single-row pieces. Pieces are built for
instanced rendering in large numbers: 44 triangles for a tile or plain block,
184 for a 1x1 brick, 836 for the 2x3 and 2236 for the 2x8 (12992 in total).
Studs are 16-segment, tubes 12-segment, posts 8-segment and the plain cylinder
24-segment. Every mesh is a set of closed, consistently wound shells — studs,
tubes and posts overlap the body instead of being Boolean-unioned into it — so
the top faces stay single fans and no hidden seam vertices exist. Edges carry a
single chamfer whose corner normals blend the two faces it joins, which shades
as a rounded edge while the large faces stay perfectly flat; the round parts
have smooth radial normals. Import normals to keep this shading. UV0 is a box
projection (cylindrical on the round parts) at one tile per stud pitch; islands
overlap, so generate lightmap UVs in the importer if you need them.
Pivots are centered on the bottom face. FBX is exported Y-up, -Z-forward,
with meter units and baked axis conversion. Use Unity's file unit conversion
and Scale Factor 1. No textures, animations, colliders, or LODs are included.

The manifest gives dimensions in the source X/Y/Z axes (width/length/height),
triangle, vertex and shell counts and FBX re-import verification results.
Unity axes are width/height/length. Roof prism and plain triangular prism
intentionally share the same basic shape, as in the reference sheet; their
sloped ridges and corners lose under a millimeter to the chamfer.

The pack is the catalog generic-construction-27-v1 of the brick-assembly
JSON format (Assets/Phys/BRICK_ASSEMBLY.md): piece IDs are the file names,
footprints are width x length in stud units, a placement positions the
bottom-centre pivot. The format's zero rotation has Ramp and Wedge rising
toward +Z; as generated here they rise toward -Z (the profiles are extruded
along the source +Y length axis, which imports as Unity -Z; verified with the
import check), so a loader turns those two by 180 degrees about Y until the
generator's profiles are mirrored.

preview.png is rendered from the actual meshes, not generated concept art.
generate_models.py regenerates the pack using Blender 5.2 (segment counts and
the chamfer width are the constants at the top of the script):
  blender -b -t 4 --python generate_models.py

These are original generic modeling assets. This package is not legal
clearance of any particular construction-toy part or brand.
