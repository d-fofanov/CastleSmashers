# Phys.AvbdGpu — Augmented Vertex Block Descent on the GPU

3D rigid-box physics in compute shaders: frictional contacts, ball-socket joints with angular locks and fracture,
springs, ignore-collision links. The algorithm is Augmented Vertex Block Descent (Giles, Diaz, Yuksel, SIGGRAPH 2025)
exactly as in the author's reference implementation `avbd-demo3d`; the whole step — broadphase, narrowphase with
persisted manifolds, graph colouring, colour-batched primal sweeps, dual updates, velocities — runs on the GPU and
nothing is read back for rendering. See [ALGORITHMS.md](ALGORITHMS.md) for the pipeline.

* **Runtime** (`Phys.AvbdGpu`) — `AvbdGpuWorld` (bodies, joints, springs, links, `Step()`), buffers, command-buffer
  recording, seven `.compute` files (`Resources/AvbdGpu`).
* **Reference** (`Phys.AvbdRef`) — line-for-line C# port of `avbd-demo3d` (the test oracle).
* **Scenes** (`Phys.AvbdGpu.Scenes`) — the 14 reference scenes and 3 GPU benchmark scenes behind an `ISceneBuilder`
  interface that both solvers implement.
* **Presentation** (`Phys.AvbdGpu.Presentation`) — `AvbdGpuRenderer`: one instanced draw straight from the solver
  buffers, GPU-written contact / joint debug lines.
* **Demo** (`Phys.Demo`) — `Demo.unity`, `DemoBootstrap` (scene keys, HUD, drag, shooting), `DemoCamera`.
* **Tests** — EditMode: reference behaviour, kernel checks, GPU vs reference comparisons, invariants, performance;
  PlayMode: demo smoke test.

## Layout

```
Assets/Phys/AvbdGpu/Runtime            AvbdGpuWorld, AvbdGpuPipeline, AvbdGpuBuffers, AvbdGpuKernels, AvbdGpuTypes, AvbdGpuConstants
Assets/Phys/AvbdGpu/Runtime/Resources  AvbdCommon.hlsl, AvbdUtil, AvbdScan, AvbdBroadphase, AvbdNarrowphase, AvbdConstraints,
                                       AvbdColoring, AvbdSolver, AvbdDebug (.compute)
Assets/Phys/AvbdGpu/Reference          RefMath, RefBodies, RefJoint (+Spring), RefManifold, RefCollide, RefSolver, RefSceneBuilder
Assets/Phys/AvbdGpu/Scenes             AvbdScenes (catalog + ISceneBuilder)
Assets/Phys/AvbdGpu/Presentation       AvbdGpuRenderer, Resources/AvbdGpu/AvbdBox.shader, AvbdLines.shader
Assets/Phys/AvbdGpu/Tests/Editor       ReferenceTests, GpuKernelTests, GpuVsReferenceTests, InvariantTests, PerformanceTests, DiagnosticTests
Assets/Phys/AvbdGpu/Tests/Runtime      DemoSmokeTests
Assets/Phys/Demo                       Demo.unity, DemoBootstrap, DemoCamera, Editor/BuildDemo
```

## Using the solver

```csharp
var world = new AvbdGpuWorld(AvbdGpuConfig.ForBodies(65536));   // capacities; buffers are allocated once
int ground = world.AddBody(new float3(100, 1, 100), 0f, 0.5f, float3.zero, quaternion.identity, float3.zero); // density 0 = static
int box = world.AddBody(new float3(1, 1, 1), 1f, 0.5f, new float3(0, 4, 0), quaternion.identity, float3.zero);
int joint = world.AddJointIndexed(-1, box, new float3(0, 6, 0), float3.zero, 5000f, 0f);   // bodyA -1: world anchor
world.Params.Iterations = 10;
world.Step();                          // uploads new bodies, runs the step on the GPU, no synchronisation
world.ReadbackPoses = true;            // optional asynchronous pose readback (1-2 frames old) for Pick / gameplay
world.GetPosesSync(out var pos, out var rot);   // synchronous readback (tests, tools)
```

`AvbdScenes.Build(world, AvbdScenes.Pyramid)` builds a catalog scene into any `ISceneBuilder`. Draw with
`new AvbdGpuRenderer(world).Render()` once per frame.

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
`-avbd-bench [-avbd-frames n]` (average frame time of the second half of the run logged, then quit).

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
```

The runner forces D3D12 (`-GraphicsApi ""` for the editor default). `DiagnosticTests` only log traces and are
excluded from the default runs, like `PerformanceTests`.
