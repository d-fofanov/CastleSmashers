# Constructor block 2x3

Generic six-stud construction-toy brick with beveled body/stud edges,
an open underside and two hollow support tubes. Stylized dimensions, not a
manufacturing or certified fit model.

- 1316 triangles (limit: 1500), 672 geometric vertices.
- Dimensions in Unity units (metres): X=0.199159664, Y=0.142436975, Z=0.3.
- FBX is Y up with centimetre file units; use Unity's default file-unit conversion.
- Pivot at the centre of the bottom face; identity object scale.
- One FBX geometry, one mesh object, no separate stud/body/tube objects.
- Explicit per-corner unit normals, with smooth round sides and sharp rims.
- Uncolored mesh: no vertex colors, material assignments, MTL or textures.
- Assign your own material in Unity. No UVs are supplied.
- Nine closed, consistently oriented mesh shells. Studs and tubes slightly
  overlap the body; this is a visual game asset, not a boolean-unioned print solid.
- Import the FBX into Unity and assign your material. Set Normals to Import.
  No scene or physics
  changes are included.
- Regenerate with Tools/generate_constructor_block.py; numpy and Pillow are
  needed only for the preview. Preview is rendered from the exported geometry.
