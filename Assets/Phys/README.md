# Phys.AvbdGpu — Augmented Vertex Block Descent on the GPU

3D rigid-box physics in compute shaders: frictional contacts, ball-socket joints with angular locks and fracture
(torque, or directional snap limits), springs, ignore-collision links. The algorithm is Augmented Vertex Block Descent (Giles, Diaz, Yuksel, SIGGRAPH 2025)
exactly as in the author's reference implementation `avbd-demo3d`; the whole step — broadphase, narrowphase with
persisted manifolds, graph colouring, colour-batched primal sweeps, dual updates, velocities — runs on the GPU and
nothing is read back for rendering. See [ALGORITHMS.md](ALGORITHMS.md) for the pipeline.

* **Runtime** (`Phys.AvbdGpu`) — `AvbdGpuWorld` (bodies, joints, springs, links, `Step()`), buffers, command-buffer
  recording, seven `.compute` files (`Resources/AvbdGpu`).
* **Reference** (`Phys.AvbdRef`) — line-for-line C# port of `avbd-demo3d` (the test oracle).
* **Scenes** (`Phys.AvbdGpu.Scenes`) — the 14 reference scenes and 3 GPU benchmark scenes behind an `ISceneBuilder`
  interface that both solvers implement; `BrickCastle`, a castle planned on the stud grid of the construction brick.
* **Presentation** (`Phys.AvbdGpu.Presentation`) — `AvbdGpuRenderer`: instanced draws straight from the solver buffers
  (the collision boxes, or any mesh for a range of bodies), per-body tints, shadows, GPU-written contact / joint debug lines.
* **Demo** (`Phys.Demo`) — `Demo.unity` / `DemoBootstrap` (the catalog scenes) and `Castle.unity` / `CastleDemo` (the brick
  castle) on the shared `DemoBase` (scene keys, HUD, drag, shooting, player flags), `DemoCamera`.
* **Tests** — EditMode: reference behaviour, kernel checks, GPU vs reference comparisons, invariants, castle layout,
  performance; PlayMode: demo and castle smoke tests.

## Layout

```
Assets/Phys/AvbdGpu/Runtime            AvbdGpuWorld, AvbdGpuPipeline, AvbdGpuBuffers, AvbdGpuKernels, AvbdGpuTypes, AvbdGpuConstants
Assets/Phys/AvbdGpu/Runtime/Resources  AvbdCommon.hlsl, AvbdUtil, AvbdScan, AvbdBroadphase, AvbdNarrowphase, AvbdConstraints,
                                       AvbdColoring, AvbdSolver, AvbdDebug (.compute)
Assets/Phys/AvbdGpu/Reference          RefMath, RefBodies, RefJoint (+Spring), RefManifold, RefCollide, RefSolver, RefSceneBuilder
Assets/Phys/AvbdGpu/Scenes             AvbdScenes (catalog + ISceneBuilder), BrickCastle (brick, layout, castle plans, snap joints)
Assets/Phys/AvbdGpu/Presentation       AvbdGpuRenderer, Resources/AvbdGpu/AvbdBox.shader (+ AvbdBodyInstancing.hlsl), AvbdLines.shader
Assets/Phys/AvbdGpu/Tests/Editor       ReferenceTests, GpuKernelTests, GpuVsReferenceTests, InvariantTests, BrickCastleTests, PerformanceTests, DiagnosticTests
Assets/Phys/AvbdGpu/Tests/Runtime      DemoSmokeTests, CastleSmokeTests
Assets/Phys/Demo                       Demo.unity, Castle.unity, DemoBase, DemoBootstrap, CastleDemo, DemoCamera, Editor/BuildDemo
Assets/Models/ConstructorBlock2x3      the 2 x 3 construction brick (FBX, Tools/generate_constructor_block.py)
```

## Using the solver

```csharp
var world = new AvbdGpuWorld(AvbdGpuConfig.ForBodies(65536));   // capacities; buffers are allocated once
int ground = world.AddBody(new float3(100, 1, 100), 0f, 0.5f, float3.zero, quaternion.identity, float3.zero); // density 0 = static
int box = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 4, 0), quaternion.identity, float3.zero);
int joint = world.AddJointIndexed(-1, box, new float3(0, 6, 0), float3.zero, 5000f, 0f);   // bodyA -1: world anchor
// snap joint: hard, breaks at 300 N of shear across body B's +y, 50 N of pull along it, or 1 cm of separation
world.AddJointIndexed(ground, box, new float3(0, 0.5f, 0), new float3(0, -0.5f, 0), float.PositiveInfinity, 0f, float.PositiveInfinity, 300f, 50f, 0.01f, snapAxis: 2);
world.Params.Iterations = 10;
world.Step();                          // uploads new bodies, runs the step on the GPU, no synchronisation
world.ReadbackPoses = true;            // optional asynchronous pose readback (1-2 frames old) for Pick / gameplay
world.GetPosesSync(out var pos, out var rot);   // synchronous readback (tests, tools)
```

`AvbdScenes.Build(world, AvbdScenes.Pyramid)` builds a catalog scene into any `ISceneBuilder`. Draw with
`new AvbdGpuRenderer(world).Render()` once per frame; `renderer.MeshRanges` draws a body range with a mesh instead of its
box (`Mesh`, `Scale`, `Offset` of the model pivot in body space), `renderer.SetTints(rgba, start, count)` colours bodies
(RGBA8, alpha 0 = hash palette), `renderer.Shadows` casts and receives the main light's shadows.

## Parameters (`AvbdGpuParams`, defaults = reference)

| Parameter | Default | Meaning |
|---|---|---|
| `Dt`, `Substeps`, `Iterations` | 1/60, 1, 10 | step, substeps (each a full step incl. collision detection), primal/dual sweeps |
| `Gravity` | (0, −10, 0) | vector; the adaptive initial guess uses its direction |
| `Alpha` | 0.99 | stabilisation: fraction of the start-of-step constraint error ignored (Eq. 18) |
| `BetaLin`, `BetaAng` | 1e4, 100 | penalty ramp per unit of linear / angular error and iteration (Eq. 16) |
| `Gamma` | 0.999 | warm-start decay of penalties and multipliers (Eq. 19) |
| `PostStabilize` | off | extra position-only iteration with α = 0 (2D reference option) |
| `RotatedInertia` | off | `R I Rᵀ` (paper Eq. 8) instead of the reference's body-frame diagonal |
| `CellSize` | 0 = auto | grid cell size (auto: 2 × mean extent of the dynamic bodies) |
| `ColorRounds` | 8 | Jones–Plassmann rounds per step |

Constants (`AvbdGpuConstants` / `AvbdCommon.hlsl`): penalty bounds 1 .. 1e10, collision margin 0.01, stick
threshold 1e-5, hard stiffness ≥ 1e30 (`float.PositiveInfinity` accepted), 8 contacts per manifold, 32 colours.

Joint fracture: the reference's `fracture` breaks a joint when its angular multiplier (torque) exceeds it. The snap limits
(`fractureLateral`, `fractureTension`, `breakDistance`, off at infinity; an extension, mirrored in the reference) act on
the linear multiplier and the anchors: the joint breaks when the pull of body B away from A along the snap axis (a body-B
axis, `snapAxis` ±1 x / ±2 y / ±3 z) exceeds `fractureTension`, when the part across the axis exceeds `fractureLateral`,
or when the anchors are `breakDistance` apart; compression along the axis never breaks it. Jointed bodies do not collide
until the joint breaks.

Capacities (`AvbdGpuConfig.ForBodies(n)`): bodies n, joints max(n, 4096), springs n/4, manifolds 4n, contacts 16n,
pairs 10n, grid entries 8n, hash 8n (power of two), cells 2n. Appends beyond a capacity are dropped and flagged in
`Stats.OverflowFlags` (1 pairs, 2 manifolds, 4 contacts, 8 grid entries, 16 large bodies, 32 unsorted cells); the demo
HUD shows the flags in red. 65 536 bodies take ~270 MB of GPU memory.

## Demo

Open `Assets/Phys/Demo/Demo.unity` and press Play, or build with `Phys / Build Demo Player`
(`Unity.exe -batchmode -quit -executeMethod Phys.Demo.Editor.BuildDemo.Build -buildPath C:/out/Demo.exe`).

Keys: `1-0` scene, `,` `.` previous / next scene (17 scenes: the 14 reference scenes, then pyramid 22k, pile 50k,
pyramid 74k), `R` reset, `Space` pause, `N` step, `F1` contact crosses (red sliding, green sticking), `F2` colour mode
(palette / graph colour / uniform), `F3` post-stabilise, `F4` rotated inertia, `F5` joint lines, `+/-` iterations,
`[ ]` substeps, `B` / `Enter` shoot a box, `G` gravity on/off, `H` hide HUD. Left drag pulls a body with a soft world
joint (5000 N/m), right drag orbits, middle drag pans, wheel / `Q` `E` zoom, `W A S D` orbit.

Player flags: `-avbd-scene n`, `-avbd-screenshot file [-avbd-frames n]` (screenshot then quit),
`-avbd-bench [-avbd-frames n]` (average frame time of the second half of the run logged, then quit),
`-avbd-yaw deg -avbd-pitch deg -avbd-distance m` (camera), `-avbd-shoot n` (fire a box at frame n).

## Castle demo

`Assets/Phys/Demo/Castle.unity` (`Phys / Build Castle Player`, or `-executeMethod Phys.Demo.Editor.BuildDemo.Build
-buildScene Castle`) builds a castle out of the construction brick `Assets/Models/ConstructorBlock2x3`: every brick is a box
body of the solver, drawn with the brick model through `AvbdGpuRenderer.MeshRanges`. Ten castles on keys `1` .. `0`, all
planned by `BrickCastle` on the stud grid (`CastlePlan.Presets`):

| Key | Name | Bricks | Side (studs) | Towers | Walls | Gatehouse |
|---|---|---|---|---|---|---|
| 1 | Outpost | 1 001 | 38 | 10 x 10, 8 courses | 5 courses | 10 courses |
| 2 | Fort | 1 346 | 38 | 10 x 10, 11 | 6 | 11 |
| 3 | Small castle | 2 282 | 50 | 10 x 10, 15 | 8 | 12 |
| 4 | Castle | 3 386 | 62 | 10 x 10, 19 | 10 | 14 |
| 5 | Large castle | 4 699 | 74 | 10 x 10, 24 | 12 | 17 |
| 6 | Fortress | 7 313 | 94 | 14 x 14, 28 | 14, 6 x 6 mid towers | 17 |
| 7 | Citadel | 10 643 | 106 | 14 x 14, 30 | 12, double, 6 x 6 mid towers | 18 |
| 8 | Stronghold | 17 129 | 126 | 18 x 18, 36 | 16, double, 18 x 18 bastions | 22, double depth |
| 9 | Great fortress | 24 733 | 150 | 18 x 18, 42 | 20, double, 18 x 18 bastions | 26, double depth |
| 0 | Royal citadel | 34 477 | 170 | 22 x 22, 50 | 24, double, 18 x 18 bastions | 30, double depth |

* curtain walls five studs thick in English bond (a stretcher row and a header row per course, swapped every course),
  doubled side by side in the big plans; hollow corner towers and a keep (rings whose two course patterns are rotated
  copies of each other); a six-deep gatehouse whose passage is closed by three corbel courses (every overhanging stretcher
  keeps two of its three studs supported), with a turret at each end; a mid-wall tower or bastion on the plain walls of
  the larger plans; header merlons on every top; a staircase up the west wall. Dry-stacked structures lean once they
  are much taller than they are wide, so the plans grow in footprint and thickness rather than height (every
  free-standing element stays under about 3.5 times its width tall); the substep count drops from 3 to 2 and 1 for the
  castles above 8 000 and 20 000 bricks. `Tools/castle_counts.py` reproduces the generator's brick counts and the
  elements' aspect ratios for tuning new plans without Unity.
* The brick's studs nest in the hollow underside of the brick above, so the collision box is the body without the
  studs, made one collision margin (1 cm) taller: resting contacts settle exactly that deep, and the models then stack
  with no gap and no overlap.
* `BrickScale` (default 5) is solver metres per model metre and `BrickMass` (0.25 kg) the mass: at 1 x 0.6 x 1.5 m and
  a fraction of a kilogram a brick is in the regime the reference's penalty ramp is tuned for (`PenaltyMin` 1, `Beta`
  1e4); at the model's true 0.2 x 0.12 x 0.3 m the stacks sink visibly before the penalties catch up. The mass is kept
  low so that the snapped gatehouse, which bends as a beam over its passage, stays below the snap limits. Gravity stays
  10 m/s², so the toy moves in slow motion; 240 settle steps run before the castle is shown.
* `J` snaps the bricks together: four hard ball-socket joints at the inset corners of every brick-on-brick overlap (and
  of every ground-course footprint, to the world). A snap connection breaks when it is pushed sideways by more than
  `SnapFractureLateral` (300 N), pulled apart along the studs by more than `SnapFractureTension` (50 N) — the limits are
  split over the four joints — or when the bricks separate by half the stud height (5.4 cm at scale 5); the weight of the
  bricks above (compression) never breaks it. A cannonball into the gatehouse breaks a few thousand of the 23 k joints
  and blows the hit section out; the rest stays snapped. Jointed bodies do not collide until a joint breaks (the reference's rule),
  and the angular lock assumes equal orientations, hence four points per overlap rather than one lock.

Keys as the main demo plus `J` snap on/off, `F6` collision boxes, `F7` shadows; `B` / `Enter` fires a 30 kg cannonball
at 24 model m/s (× √5 in the solver). Flags: `-avbd-scene 0..2`, `-avbd-snap`, and the screenshot / bench / camera
flags above.

## Measured behaviour

Numbers from the EditMode suite and the player on an RTX 4070 Laptop GPU (i7-14700HX, D3D12), plugged in. On battery
this GPU is capped at ~250 MHz / 17 W and everything below is 3-5x slower.

* GPU vs the reference (reference run in the GPU's pair convention, 10 iterations): a box dropped on the ground
  1e-7 after 120 steps, spring 0, stack ratio 2e-5, static friction 9e-5, dynamic friction 3e-5, bridge 1e-4, rope
  2e-3, heavy rope (1000 : 1) 1e-3, stack 6e-3, breakable 3e-3, soft body ~0, pyramid 1e-2 after 30 steps. The
  remaining difference is the Gauss-Seidel order: the stiff spring chain (k ratio 1000) is 0.9 m apart at 10
  iterations and 1e-4 at 100 iterations, where both converge to the same solution.
* The narrowphase reproduces the reference `collide()` on random OBB pairs (contact count, feature keys, points
  within 1e-3; face and edge manifolds); the broadphase produces exactly the brute-force AABB pair set (owner-cell
  rule, large-body list, link filter). Two runs are bitwise identical.
* A resting box carries its weight in the contact multipliers (sum of -lambda_n = m g within 10 %), its normal
  penalty ramps and persists across steps and its contacts stick; the 16-row pyramid, the 1-2-4-8 stack and the
  bridge stand; the breakable chain fractures.

Step times (solver only, `PerformanceTests`, GPU synchronised once after 60-120 steps) and player frame times
(`-avbd-bench`: one step + rendering + HUD per frame, 1280 x 720):

| Scene | Bodies | Manifolds / contacts | Step, 10 iterations | Step, 4 iterations | Player frame, 10 iterations |
|---|---|---|---|---|---|
| Pyramid (16 rows) | 137 | 283 / 1.2 k | 1.47 ms | | 3.5 ms (290 fps) |
| Breakable | 19 | 10 / 36 | | | 3.1 ms (328 fps) |
| Pyramid 22k | 22 141 | 85 k / 340 k | 6.3 ms | 3.9 ms | 8.3 ms (120 fps) |
| Pile 50k (falling block) | 50 001 | 52 k / 200 k | 3.9 ms | 3.4 ms | 4.6 ms (219 fps) |
| Pyramid 74k | 73 811 | 305 k / 1.19 M | | 17.4 ms | 37 ms (27 fps) |

Small scenes are bound by the ~130 indirect dispatches of a step (about 1.5 ms); large piles by the contact traffic
of the primal sweeps (every iteration re-reads every contact from both bodies), so the iteration count is the main
knob — the paper uses 4 for its large piles. CPU time per step is 0.1-1 ms (parameter upload, command buffer
submission).

## Known limits

* Boxes only (the narrowphase is `collide.cpp`); no sleeping; discrete contacts at `x⁻`, so very fast small bodies
  can tunnel (the reference behaves the same); no body removal (joints can be removed, their slots are reused).
* Small scenes cost ~1–2 ms per step regardless of size: every iteration is `colours + 3` dispatches. The active
  colour count adapts to the scene to keep that low.
* Determinism holds on one GPU/driver; other GPUs round differently.
* Bodies spanning more than 64 cells bypass the grid through a 256-entry list (brute force against every body);
  more than 256 such bodies overflow (flag 16).
* The feature keys of the reference include the clipped polygon's vertex index, so warm starting can miss a contact
  after a re-ordering of the clip polygon — reproduced as is.

## Tests

```
.\RunTests.ps1                                  # EditMode + PlayMode (editor must not have the project open)
.\RunTests.ps1 -Platform EditMode -Filter Phys.AvbdGpu.Tests.GpuVsReferenceTests
.\RunTests.ps1 -Platform EditMode -Filter Phys.AvbdGpu.Tests.PerformanceTests   # step times (excluded by default)
.\RunTests.ps1 -Platform EditMode -Filter Phys.AvbdGpu.Tests.SnapFractureTests  # snap limits, GPU and reference
.\RunTests.ps1 -Platform PlayMode -Filter Phys.AvbdGpu.Tests.CastleSmokeTests   # the smallest and largest castles stand, cannonball, snaps break
```

The runner forces D3D12 (`-GraphicsApi ""` for the editor default). `DiagnosticTests` only log traces and are
excluded from the default runs, like `PerformanceTests`.
