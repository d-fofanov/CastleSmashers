# Brick assembly — JSON models built from the construction-piece pack

A `brick-assembly` document describes a shape built from the 27 pieces of `Assets/Models/construction_pieces` (catalog
`generic-construction-27-v1`) as explicit, deterministic placements, so that a castle can be exchanged between agents
and rebuilt in Unity exactly. This is version 1 of the format (after *Brick Assembly Text Specification, version 1*).
`BrickAssembly` (`Phys.AvbdGpu.Scenes`) reads it and the preview demo (`Preview.unity`, see
[README.md](README.md#preview-demo)) shows every document of `Assets/Resources/Castles` on the solver; the last section
says how the format maps onto the solver's conventions.

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
| `occupancy` | no | the occupancy map: per stud cell the top of the highest piece, and the posts where figures stand (see below) |

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

## Occupancy map

A castle is also a place where figures stand: on its walls, in its courtyard. The optional root field `occupancy` says
where, without anyone having to recover it from thousands of placements: a grid over the footprint, one cell per stud,
holding the top of the highest piece over the cell in grid units, and the posts of the garrison.

```json
"occupancy": {
  "origin": [1, 1],
  "size": [38, 38],
  "rows": [
    [6, 6, 6, 6, 0, 0, 0, 0, 6, 6, 6, 6],
    [6, 6, 6, 6, 0, 0, 0, 0, 6, 6, 6, 6]
  ],
  "posts": [
    [3, 6, 2, 180],
    [36, 6, 2, 0]
  ]
}
```

| Field | Required | Value |
|---|---|---|
| `origin` | yes | two integers: the grid cell (x, z) whose min corner is the first cell of the first row; the map covers `[origin, origin + size)` |
| `size` | yes | two positive integers: cells along x, cells along z |
| `rows` | yes | exactly `size[1]` arrays of exactly `size[0]` finite numbers ≥ 0: the top of the highest body over the cell `(origin[0] + i, origin[1] + j)` in grid units, studs excluded; `0` = nothing stands there |
| `posts` | no | `[x, y, z, yaw]` in grid units and degrees: where a figure stands — `y` is the surface under its feet, `yaw` the heading about +Y with 0 facing +Z (the format's rotation convention); `[x, y, z]` faces +Z |

A piece covers every cell whose centre lies within its body's bounding box on the ground plane (a `Brick_2x3` at x = 1
covers cells 0 and 1, not 2; a turned or lying piece takes its bounding box), and a cell's top is the highest such box.
The map holds tops only: the passage under a lintel reads as solid, which is acceptable for what the map is for (where
nothing stands, where a shot may start). A document with the section is taken as written — an author may leave a
gateway open or add posts by hand; a document without it gets the map derived from its parts by the same rule, and
posts derived from the map:

* **Wall posts** stand on a cell at least three courses high (3.6) that is level within a plate (0.45) with its two
  neighbours along the wall (a merlon next to it disqualifies; when the cell inward is level too, the post stands half a
  cell back from the edge), and whose walk outward from the map's centre (along the dominant axis) steps down within
  the wall's thickness (8 cells, room for a parapet in front of the walk) and meets nothing as high again before the
  border. Posts closest to the outer edge come first, then those behind an equally high outer wall (an inner ward),
  spread around the centre by angle at least four cells apart; they face outward.
* **Ground posts** stand on a cell below wall height whose 3 x 3 neighbourhood is level — the ground, a pavement or a
  raised courtyard floor — enclosed in all four axis directions by cells three courses above it; those nearest a wall
  come first, facing it.

Reject: `size` that is not two positive integers, a row count or length that does not match `size`, a top that is not
a finite non-negative number, a post with fewer than three or more than four numbers.

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

`Assets/Phys/AvbdGpu/Scenes/BrickAssembly.cs` holds the catalog (`PieceCatalog`: the 27 pieces with their footprint, body
height, studs on top and whether the underside grips studs), the reader (`BrickAssembly.Parse`: a small JSON reader, the
validation above, module expansion into `Parts` with their ID paths, composed rotations and resolved colours, the bounds),
a writer for the castle planner's layouts (`BrickAssembly.WriteLayout`, behind `Phys / Export Outpost as Brick
Assembly`) and `AssemblyBuilder`, which turns an assembly into solver bodies; `PreviewDemo` drives it, `BrickAssemblyTests`
and `PreviewSmokeTests` check it. `AssemblyOccupancy.cs` is the occupancy map: parsed from the section, derived from the
parts (`FromParts`), its posts derived (`DerivePosts`), written back into a document (`Splice`, behind `Phys / Write
Occupancy`, which refreshes every document of `Resources/Castles`); the siege demo places the garrison on its posts and
skips shots that would start inside a wall (`AssemblyOccupancyTests`).

* **Bodies.** `AssemblyBuilder.Build` adds one box per part (grouped so that parts sharing a piece and a mesh offset are one
  draw range of `AvbdGpuRenderer.MeshRanges`; `AssemblyBodies` maps parts to bodies and back). The castle demo's rules for
  the 2 x 3 brick (`BrickCastle.BrickSpec`) generalise to every piece: the box is the body without the studs,
  `footprint · 0.1 · s` across and `h · s` tall, grown by the 1 cm collision `margin` along the body axis that faces down
  (`AssemblyBuilder.DownAxis`: −y for an upright piece, +z for a cylinder lying along +z), so that resting contacts — which
  settle exactly one margin deep — leave the models neither gapped nor overlapping, and shrunk by `Clearance` (1.25 mm per
  side of the model, real bricks' 1.25 % of the pitch) on the two other axes, so that side-by-side boxes do not touch and
  pass loads the snaps are not built to take. The box centre is `pivot + R · c` with `c = (0, h · s / 2, 0) + d · margin / 2`
  (`d` the down axis), the mesh offset `−c`. Sloped and round pieces get their bounding box (the solver is boxes only).
  `s` is the solver scale (`BrickScale`, 5 in the demo: the toy is simulated at five times its model size so that the
  penalty ramp holds it, see the README), so a grid coordinate becomes `x · 0.1 · s` solver metres.
* **Snaps.** `AssemblyBuilder.AddSnapJoints` is the castle's four-joints-per-overlap snap: joints at the inset corners of
  every overlap of at least half a stud in which an upright, stud-aligned piece rests exactly one body height (within 0.02
  studs) on a studded top — bricks and plates — and world joints for the gripping pieces on the ground; a piece a stud
  height above a top is not nested and gets no joint. Tiles, slopes and prisms grip studs, the plain pieces do not, and
  nothing holds on a smooth top: whatever rests there is held by friction alone, as the format warns.
* **Diagnostics.** `AssemblyBuilder.Diagnose` reports, without touching a placement, the pieces resting on nothing, the
  pieces resting on less than half their footprint (a piece a stud height above a top counts as resting on the studs) and
  the intersecting pairs; the demo's HUD shows the counts and the first offending IDs.
* **Colours and IDs.** `color` (a palette key resolved to `#RRGGBB`, or the default grey) becomes the body's RGBA8 tint
  (`AvbdGpuRenderer.SetTints`); the expanded ID paths stay next to the bodies, so tests and diagnostics name pieces.
* **Physical feasibility.** The format places pieces; whether they stand is the physics' verdict, and a design that stacks
  the same brick straight up course after course does not (columns two studs thick lean and topple, rings of four columns
  never interlock, decks span hollows on nothing). `Tools/rebond_castle.py` turns such a document into one that holds
  without changing its look: it expands every course of upright bricks into stud cells and tiles them again, cell for
  cell and colour for colour, with a running bond, alternating corner and junction ownership and the best support the
  course below offers; the hidden supports a particular design needs (cross walls under a deck, a post, a bracket under
  a banner) are added as data at the top of the script. See the README's preview demo section for the citadel.
  `Tools/generate_trees.py` goes the other way round: it designs the six trees of `Assets/Resources/Trees` as stud cells
  course by course under a corbel rule (a course reaches at most one stud beyond the one below, and only next to a cell
  resting on it; the overhanging cells get pieces that keep three quarters of their footprint on the course below) and
  tiles them with the same course tiler, so that a crown of a thousand pieces holds on its trunk; the castle demo plants
  them around the castle asleep.
