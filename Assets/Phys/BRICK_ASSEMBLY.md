# Brick assembly — JSON models built from the construction-piece pack

A `brick-assembly` document describes a shape built from the 27 pieces of `Assets/Models/construction_pieces` (catalog
`generic-construction-27-v1`) as explicit, deterministic placements, so that a castle can be exchanged between agents
and rebuilt in Unity exactly. This is version 1 of the format (after *Brick Assembly Text Specification, version 1*).
Nothing in the project reads it yet — the castle demo plans its castles procedurally (`BrickCastle`, see
[README.md](README.md#castle-demo)) — so this document is the contract for the loader that will; the last section maps
the format onto the solver's conventions.

## Units and axes

Unity axes: X right, Y up, Z forward. Positions are in grid units, 1 grid unit = 0.1 Unity units (`gridToUnity`), the
stud pitch. A position places the piece's pivot, the centre of its bottom face. The FBX models are already sized in
Unity units (metres) with that pivot, so a placement converts its coordinates and leaves the model unscaled — never scale
the asset by 0.1 as well.

| Measurement | Grid units | Unity units |
|---|---|---|
| Stud spacing (pitch) | 1 | 0.1 |
| Brick body height | 1.2 | 0.12 |
| Plate / tile body height | 0.4 | 0.04 |
| Stud height | 0.225 | 0.0225 |
| 2 x 3 brick footprint | 2 x 3 | 0.2 x 0.3 |

Pieces stack on body heights, studs excluded: a brick resting on a brick starts at y = 1.2, and the lower brick's studs
sit in the hollow underside of the upper one. All catalog dimensions below are grid units.

## Rotations and transforms

`rotation` is `[x, y, z]` in degrees, applied Z first, then X, then Y: `R = Ry · Rx · Rz` on column vectors, which is
Unity's own Euler convention (`Quaternion.Euler(x, y, z)`; `quaternion.EulerZXY(radians)` in Unity.Mathematics) with
Unity's axis rotation matrices — +90° about Y maps local +Z to +X. Compose rotations as matrices or quaternions, never
by adding Euler angles.

```
assemblyPoint = position + R · localPoint
worldPosition = parentPosition + parentRotation · localPosition
worldRotation = parentRotation · localRotation
```

Points and positions share the grid units. Stud-aligned construction uses multiples of 90°; other angles are allowed for
decorative arrangements. Version 1 has no per-piece scale.

## Piece catalog

IDs are the FBX file names without the extension. The footprint is local X x Z (width x length); the height is the body
without studs and, for sloped pieces, its highest point.

| Piece ID | Footprint | Body height | Shape |
|---|---|---|---|
| `Brick_WxL`: 1x1, 1x2, 1x3, 1x4, 1x6, 1x8, 2x2, 2x3, 2x4, 2x6, 2x8 | W x L | 1.2 | studs on top, hollow underside |
| `Plate_WxL`: 1x1, 1x2, 1x4, 2x2, 2x4 | W x L | 0.4 | studs on top, hollow underside |
| `Tile_WxL`: 1x1, 1x2, 2x2, 2x4 | W x L | 0.4 | smooth top, solid |
| `Ramp` | 2 x 2 | 1.2 | slope rising toward +Z, from 0.2 at the low edge |
| `Wedge` | 2 x 3 | 0.8 | slope rising toward +Z, from 0.15 at the low edge |
| `Roof_Prism`, `Plain_Triangular_Prism` | 2 x 2 | 1.2 | centred ridge parallel to X (the same shape twice) |
| `Plain_Cube` | 1 x 1 | 1 | solid |
| `Plain_Rectangular_Block` | 2 x 3 | 1.2 | solid |
| `Plain_Cylinder` | diameter 1.5 | 1.5 | axis Y |

**Zero-rotation orientation.** Rectangular pieces have their width along X and their length along Z; `Ramp` and `Wedge`
rise toward +Z; both triangular prisms have a centred ridge parallel to X; the cylinder's axis is Y. Imported FBX
geometry must be normalised to these conventions — dimensions alone cannot tell a ramp rising toward +Z from one rising
toward −Z. The pack meets them as imported, with every pivot at the bottom-face centre, so a loader applies the
document's rotation to the model as it comes (checked in Unity on the vertices of each piece's top plane: the high edge
of the imported `Ramp` lies at z = +0.1, of the `Wedge` at z = +0.15). `generate_models.py` builds the two slopes rising
toward its −Y, because the FBX export and Unity's import turn the generator's +Y length axis into Unity −Z — keep that
in mind when adding directional pieces. The sloped meshes lose under a millimetre of height to the edge chamfer
(`mesh_manifest.json`: Ramp 0.1193 m, prisms 0.1188 m, Wedge 0.0797 m); the catalog heights are the nominal ones.

## The assembly document

| Root field | Required | Value |
|---|---|---|
| `format` | yes | `"brick-assembly"` |
| `version` | yes | `1` |
| `catalog` | yes | `"generic-construction-27-v1"` |
| `name` | yes | display name |
| `units` | yes | `{ "gridToUnity": 0.1, "axes": "unity" }` — the only values valid for this catalog |
| `parts` | yes | root parts; an empty array if the assembly consists only of module instances |
| `palette` | no | named colours `"key": "#RRGGBB"`, referenced by `color`; applies inside modules too |
| `modules` | no | reusable groups, see below |
| `instances` | no | root placements of modules |

```json
{
  "format": "brick-assembly",
  "version": 1,
  "catalog": "generic-construction-27-v1",
  "name": "Small bridge",
  "units": { "gridToUnity": 0.1, "axes": "unity" },
  "palette": { "supports": "#687783", "deck": "#C58B42" },
  "parts": [
    { "id": "left_support",  "piece": "Brick_2x2", "position": [-3, 0, 0],  "rotation": [0, 0, 0],  "color": "supports" },
    { "id": "right_support", "piece": "Brick_2x2", "position": [3, 0, 0],   "rotation": [0, 0, 0],  "color": "supports" },
    { "id": "bridge_deck",   "piece": "Brick_2x8", "position": [0, 1.2, 0], "rotation": [0, 90, 0], "color": "deck" }
  ]
}
```

The deck, turned 90° about Y, spans eight grid units along X (x = −4 .. 4) and rests on the tops of the two supports
(y = 1.2).

| Part field | Required | Value |
|---|---|---|
| `id` | yes | string, unique within its containing scope (the root or one module); slash-free, so that expanded instance paths stay unambiguous |
| `piece` | yes | a catalog ID |
| `position` | yes | three finite numbers, grid units: the pivot |
| `rotation` | no | three finite numbers, degrees; default `[0, 0, 0]` |
| `color` | no | a palette key or `#RRGGBB`; default `#A0A0A0`; a missing palette key is an error |

## Reusable modules

A module holds local `parts` and, optionally, nested `instances`; an instance names a module and places it with a
`position` and an optional `rotation` (default `[0, 0, 0]`). The root `palette` applies throughout. The fragment below
adds two pillars of two bricks each to a complete document (the bridge above would then carry its deck at y = 2.4):

```json
"modules": {
  "pillar": {
    "parts": [
      { "id": "bottom", "piece": "Brick_2x2", "position": [0, 0, 0] },
      { "id": "top",    "piece": "Brick_2x2", "position": [0, 1.2, 0] }
    ]
  }
},
"instances": [
  { "id": "pillar_left",  "module": "pillar", "position": [-3, 0, 0], "rotation": [0, 0, 0] },
  { "id": "pillar_right", "module": "pillar", "position": [3, 0, 0],  "rotation": [0, 0, 0] }
]
```

Instances expand recursively with the transform composition above (parent position and rotation applied to every local
placement); an expanded part's ID is its path, `pillar_left/bottom`. Part and instance IDs are unique within their
containing scope, module names document-wide. Unknown module references and circular references are errors. Version 1
defines no module scale and no per-instance colour override.

## Reconstruction

A receiving agent — or the loader — reconstructs the document exactly: pieces loaded by catalog ID, normalised to the
axes and centre-bottom pivots above, grid coordinates converted with `gridToUnity`, instances expanded recursively,
rotations composed as matrices or quaternions. Every placement, colour and ID is preserved: nothing moves pieces, fills
gaps, resizes geometry or substitutes an unavailable asset. Unknown piece IDs and invalid transforms are errors;
unsupported pieces and unintended intersections are reported separately, without modifying the assembly.

## Validation and limits

Reject: malformed JSON; unsupported `format`, `version` or `catalog`; invalid vectors (anything but three finite
numbers); duplicate IDs within a scope; unresolved piece, module or palette references; circular module dependencies;
for this catalog, a `gridToUnity` other than 0.1 or `axes` other than `unity`. Findings about geometric support and
collisions are diagnostics, not permission to alter placements.

Visual reconstruction is distinct from physical feasibility: the format fixes the geometry and guarantees no mechanical
connection — nothing holds on a smooth tile or on the solid decorative pieces. Studs meant to sit in the underside
cavity of the piece above make the bounding boxes intersect; that is not an unintended overlap.

## In this project

* **Bodies.** The castle demo's rules for the 2 x 3 brick (`BrickCastle.BrickSpec`) generalise to every piece: a part is
  one box body of the solver, `footprint · 0.1 · s` wide and long and `h · s + margin` tall (`h` the body height in
  metres, `margin` the 1 cm collision margin), so that resting contacts — which settle exactly one margin deep — leave
  the models neither gapped nor overlapping; the box centre is `pivot + R · (0, (h · s − margin) / 2, 0)`, and the model
  is drawn through `AvbdGpuRenderer.MeshRanges` with a mesh offset of `(0, −(h · s − margin) / 2, 0)`. Studs never enter
  a box; sloped and round pieces get their bounding box (the solver is boxes only). `s` is the solver scale
  (`BrickScale`, 5 in the demo: the toy is simulated at five times its model size so that the penalty ramp holds it, see
  the README), so a grid coordinate becomes `x · 0.1 · s` solver metres.
* **Snaps.** The castle's four-joints-per-overlap snap (`BrickCastle.AddSnapJoints`) is meaningful only where the lower
  piece has studs — on bricks and plates, at the body-height pitch. Whatever rests on a tile, a ramp, a prism, the
  cylinder or a plain piece is held by friction alone, as the format warns.
* **Colours.** `color` (a palette key resolved to `#RRGGBB`, or the default grey) becomes the body's RGBA8 tint
  (`AvbdGpuRenderer.SetTints`).
* **IDs.** The loader keeps the expanded ID paths next to the body indices it returns, so that tests and diagnostics can
  name a piece.
