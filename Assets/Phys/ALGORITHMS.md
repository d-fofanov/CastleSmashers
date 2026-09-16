# How the GPU solver works

Augmented Vertex Block Descent (Giles, Diaz, Yuksel, SIGGRAPH 2025) for 3D rigid boxes, with the whole step running
in compute shaders. The numerics are those of the author's reference `avbd-demo3d`; this document describes how the
serial reference was turned into a GPU pipeline. Section references are to the paper.

## The step as energy minimisation

Implicit Euler is the minimiser of `½ ‖x − y‖²_M/h² + Σ E(x)` where `y = x + h v + h² g` is the inertial pose. VBD
minimises it one body at a time (block descent): the 6 degrees of freedom of a body (position, rotation) take one
Newton step of the local energy while every other body is fixed. The local system is 6 × 6, assembled from the
body's inertia and from every constraint that touches it, and solved with an LDLᵀ factorisation (`solve6`).

Hard constraints (contacts, hard joints) are augmented Lagrangian terms `½ k C² + λ C`: a penalty `k` that ramps
up with the residual (Eq. 16, `k += β|C|`) and a multiplier `λ` that is updated after every sweep (Eq. 11,
`λ = clamp(k C + λ, bounds)`). Contacts bound `λ_n ≤ 0` and keep the tangential force inside the friction cone
`|λ_t| ≤ μ|λ_n|`; joints fracture when `|λ_ang|` exceeds the threshold (and, an extension for snapped bricks, when
the linear multiplier pulls body B away from A along a body-B axis, shears across it, or the anchors separate past
per-joint limits). Both variables are warm started from the
previous step with a decay `γ` (Eq. 19). A stabilisation parameter `α` removes only a fraction of the constraint
error that existed at the start of the step (Eq. 18), which keeps error correction from injecting momentum.

Contacts use a truncated Taylor series of the constraint about the start-of-step pose `x⁻` (Sec. 4): the normal,
the tangents and the contact points are computed once per step, and the constraint value during the iterations is
`C = C0 (1 − α) + J (x − x⁻)`.

## Pipeline

Every kernel is recorded once into a `CommandBuffer` (twice: the two manifold pools alternate between the recordings,
step 12); every count-dependent dispatch is indirect, so the buffers are re-recorded only when the iteration count, the
post-stabilisation flag, the active colour count, the terrain or the sleeping settings change. Per-step constants (dt, gravity, α, β, γ, counts, the sleep thresholds) live in one constant
buffer.

0. **CPU wakes** (`WakeList`): the bodies the application changed since the last step (spawned, re-driven, re-flagged,
   jointed, or the neighbours of a retired body, found through last step's constraint lists) get their sleep words
   cleared, so that their contacts are recomputed below. Then the **hot list** (`HotFlag` → scan → `HotScatter`): the
   bodies every per-body pass of the step runs over — alive, not the terrain slot, and active (dynamic, alive and awake)
   or not yet in the sleeping grid (a body that fell asleep after the last rebuild is still inserted into the hot grid,
   and simply skipped by the passes that only concern the awake). A flag per body and an exclusive scan give the list in
   index order; the bodies in the sleeping grid have no thread anywhere until a wake appends them. The **active joint
   and spring lists** (`JointList`) follow: every active hot body walks its link list and appends each joint or spring
   once (from body B when B is active, else from A; world-anchored joints hang on B through a sentinel link), so the
   constraint index space of the step is the manifolds, then these joints, then these springs, and no pass ever visits
   the joints of a sleeping structure. Then the **kinematic orientations** (`DriveKinematic`): bodies with a heading take
   `RotateY(yaw)` from their drive, bodies that align to their velocity take `LookRotation(v⁻, up)` (the previous step's
   velocity, while faster than 1 m/s). This runs before collision detection so that the step's contacts see the
   orientation the body will keep.
1. **World AABBs** (`BodyAabb`) of the hot bodies at `x⁻`, expanded by the collision margin. Bodies spanning more than
   64 grid cells (the 100 m grounds) go to a small *large-body list* that every hot body tests brute force instead of
   the grid. Retired slots and the terrain slot are never hot: they enter neither a grid nor the list.
2. **Hot grid** (`GridCount` → scan → `GridScatter` → `GridSortCell`): every hot body is inserted into every cell its
   AABB overlaps (counting sort with atomics and an exclusive scan); each bucket is then sorted by body index. The cell
   size defaults to twice the mean extent of the dynamic bodies. The **sleeping grid** is the same structure over the
   inactive bodies (asleep, or static and small), rebuilt by its own command buffer (`AvbdSleepGrid.compute`) every
   `SleepGridRebuildSteps` steps when `SleepGridRebuildMin` bodies (or a 64th of the grid) fell asleep or woke since the
   last rebuild — a gate kernel decides and writes zero dispatch arguments otherwise. Its bodies carry bit 29 of their
   sleep word; every wake overwrites the word, so a woken body's entries are skipped (stale) and it is found in the hot
   grid instead, while a body that fell asleep since the rebuild (pending) is still in the hot grid. A sleeper never
   moves, so its entries stay current between rebuilds.
3. **Pairs** (`PairGen`): an *active* body walks its cells in both grids and takes every pair with an inactive body
   (static or asleep) and, with another active body, the pair in which it has the lower index; inactive bodies stay in
   the grids to be found but walk nothing, and a pair without an active body is never emitted (the manifolds of sleeping
   bodies are carried instead, below). A pair is emitted only from the cell that contains the minimum corner of the two
   AABBs' intersection (*owner-cell rule*), so a pair overlapping many cells is emitted exactly once and hash aliasing
   can only add candidates that the AABB test rejects. Pairs joined by a joint (unless broken), a spring or an
   ignore-collision link are filtered through a sorted per-body link list. An active large body walks the sleeping grid
   itself (up to 4096 cells; the flag 16 reports more).
4. **Narrowphase** (`Collide`): the reference's OBB test — SAT over 15 axes with an edge preference tolerance, face
   clipping (incident face against the reference face's side planes, up to 8 points, feature keys from the reference
   axis, incident axis and vertex index) or closest points of the two support edges. The previous step's manifold of
   the same pair is found in an open-addressing hash table keyed by the pair, and failing that in the **cold store**
   (below: the pair's manifold from when its bodies fell asleep, valid when a body of it woke this step); per feature key
   the penalty, multiplier, stick flag and, for sticking contacts, the old anchor points are carried over, then the
   warm-start decay is applied, and a cold manifold used this way is marked dead. Contacts are appended to a pool (one
   atomic per manifold), the header to the manifold list, the pair to the new hash table. Bodies that report events get
   the kind of the other body or-ed into their event word, and an impact counted when the manifold is new and the other
   body is a projectile moving faster than 2 m/s.
   **Terrain** (`CollideTerrain`, one thread per body, appending to the same pools): the world's heightfield is a body
   slot — static, at the identity pose, in no grid cell — so a terrain manifold is an ordinary manifold with the dynamic
   body as A and the slot as B, and nothing downstream knows the difference. The body is skipped when its AABB's bottom
   is above a max-height mip (8 x 8-cell blocks) over its footprint; otherwise its 26 lattice points (corners, edge
   midpoints, face centres, in that order) are sampled against the surface (bilinear height, the samples' central
   differences interpolated for the normal, flat beyond the border), their signed distance to the local tangent plane
   taken, and the at most 8 deepest touching points selected (ties to the lower index, so a face resting flat keeps its
   corners). One normal per manifold, the normalised sum of the selected points' surface normals; each contact's terrain
   point is the lattice point projected along that normal onto the local tangent plane, so its tangential `C0` is zero
   like a clipped box point's, and its feature key is the lattice index, which the warm start finds again step after
   step (a resting box keeps eight sticking anchors on the terrain as on the ground box). The selection runs in fixed-trip
   unrolled loops over a register array of distances and re-samples the chosen points, FXC having no data-dependent
   indexing of local arrays. `AvbdTerrain.hlsl` mirrors `Heightfield.cs` and `RefCollide.CollideTerrain` operation for
   operation; the GPU and the CPU reference agree to rounding.
   **Touches** (`WakeTouch` → `WakeApply` → `WakeClear`): every constraint joining a sleeping body to an awake one that
   is not resting marks the sleeper (a bit at its index) or, when the awake body is faster than `WakeSpeed`, its whole
   island (a bit at the island's representative); every marked body and every body whose island is marked has its
   asleep bit cleared before anything is solved (see *Sleeping* below). The two passes over every body run only in steps
   that set a mark; a woken sleeper of the sleeping grid is appended to the hot list.
   **Second round** (`PairGenWoken` → `Collide` → `CollideTerrain` → `JointListWoken`): the bodies the touches woke were
   inactive at pair generation, so they walk both grids now and take every pair with an inactive body and, with another
   woken body, the one where they have the lower index (a body that was active in the first round emitted its pair with
   them already; the woken bodies of the sleeping grid keep their grid bit for the step, so they still find each other
   there). Their contacts warm start from the cold store, their terrain contacts likewise, and their joints and springs
   join the active lists (the ones no body that was active at the start of the step brought already).
5. **Joints** (`PrepareJoints`, over the active joint list): `C0` of the ball-socket and the angular lock at `x⁻`, decay,
   penalty capped at the material stiffness; a joint without an active body is not in the list and stays frozen with its
   bodies.
6. **Constraint lists** (`ConsCount` → scan → `ConsFill` → `ConsSort`): a CSR list per dynamic body of the manifolds,
   joints and springs of the step touching it (the carried manifolds give the sleeping bodies theirs: the wakes of a
   retired body walk them). Each list of an active body is sorted by a canonical key (the neighbour index for manifolds,
   the CPU-assigned index for joints and springs), so the accumulation order of the local system is deterministic; the
   lists of sleeping bodies stay unsorted.
   **Islands** (`LabelRound` × R, over the hot list): every dynamic body carries a label, the index of a representative
   body of its island; a round replaces it by the minimum of its own label, the label of its label (pointer jumping) and
   its dynamic neighbours' labels (Jacobi, ping-pong buffers), so a component converges to the least index in it in about
   log₂(diameter) rounds — four rounds per step converge a 100-hop wall within two steps. Labels only merge, so every
   `SleepRelabelSteps` the awake bodies restart from their own index and bodies that came apart become separate islands
   again. A sleeping body keeps the label it fell asleep with (its connectivity is frozen, and it runs no round; a
   neighbour reads its label from the authoritative buffer whichever side of the ping-pong the round is on).
7. **Colouring** (`ColorInvalidate` → `ColorRound` × R → `ColorFinalize` → `ColorScan` → `ColorScatter`): bodies that
   share a constraint must not be updated in the same parallel pass; sleeping bodies are coloured like static ones (they
   are not solved and constrain no colour). Last step's colours are kept unless a
   higher-priority neighbour (hashed index) has the same one; uncoloured bodies then take the smallest colour that is
   free among coloured neighbours whenever every uncoloured neighbour has lower priority (Jones–Plassmann greedy, a
   few rounds). Whatever is still uncoloured joins the *overflow group*, which is updated Jacobi style with double
   buffered positions — always correct, only slower to converge. The colour count in use is read back asynchronously
   and the number of recorded colour passes adapts to it.
8. **Predict**: inertial pose `y = x + h v + h² (g + a)`, where `a` is the body's drive acceleration (below), the VBD
   adaptive initial guess (gravity weighted by the measured acceleration, the drive added whole), `x⁻`.
9. **Iterations**: for every colour a `Primal` dispatch updates its bodies in place (they never read each other), the
   overflow group writes to a second buffer and is committed; then `Dual` updates every manifold and joint in
   parallel (a carried manifold or a frozen joint, without an active body, is skipped); the last iteration ends with
   `Velocity` (`v = (x − x⁻)/h`, `ω = 2 (q q⁻⁻¹)ₓᵧᵤ/h`; sleeping bodies keep their zero velocities). With
   post-stabilisation on, the main iterations use α = 1 and one extra position-only iteration uses α = 0. A body with
   locked rotation solves only the 3 × 3 linear block of its system (infinite angular inertia: the angular update and the
   cross terms vanish) and keeps `ω = 0`.
10. **Rest and sleep** (`SleepTimer` → `RestSpread` × `SleepHops` → `RestSleep`, over the hot list): every awake body
    compares its pose with its rest anchor and either restarts its rest counter and the anchor (moved further than
    `SleepDistance` or `SleepAngle`) or counts the step; the counters are then propagated as a minimum over each body's
    awake dynamic neighbours (a sleeping neighbour counts as rested: it slept because its neighbourhood rested, and
    anything moving against it wakes it first), one round per hop, and a body whose neighbourhood minimum reached
    `SleepTime` falls asleep (asleep bit, velocities zeroed, its sleep generation advanced, counted as pending for the
    sleeping grid).
11. **Freeze** (`FreezeCount` → two scans → `FreezeCopy` → `FreezeFinish`, only in steps where a body fell asleep): every
    manifold of the step whose bodies are both inactive now (alive: a retired body takes its manifolds with it) is
    appended to the cold store with its contacts and the generations of its bodies, in the order of the step's list
    (the scans give the manifolds and the contacts their places, so the contacts of the pool lie in manifold order),
    and hashed by pair. It is not recreated next step — no pair without an active body is — so it leaves the step.
12. The counters are copied to a stats buffer for the asynchronous readback. The manifold, contact and hash buffers
    are not copied anywhere: the step is recorded twice, once reading pool 0 and writing pool 1 and once the other way
    round, and the world executes the two recordings alternately, so the next step reads this step's manifolds from
    where they were written.

Rendering never touches the CPU: one `RenderMeshPrimitives` cube draw reads the pose and definition buffers by
instance id; contact crosses and joint lines are written by small kernels into vertex buffers and drawn with
`RenderPrimitivesIndirect`.

## Driven bodies, pools and events

Three extensions carry units and projectiles on the same solver; none of them touches the numerics of a body that does
not use them, so the reference comparisons hold as before (the drives and the rotation lock are mirrored in the reference).
The terrain (above) is a fourth: a world without one records no terrain pass and pays nothing.

* **Drives** are accelerations of the inertial pose, exactly where gravity enters: a constant force, a force of constant
  magnitude towards a fixed point (evaluated at `x⁻`), or a motor `F = clamp(m (v_target − v⁻) / h, F_max)` on a mask of
  axes. A free body under a drive therefore follows the implicit Euler trajectory of the total acceleration, and a body
  in contact feels the drive through its inertia term like any external force. The motor is a proportional controller
  with the gain `m / h`, so it runs `F h / m` below its target under a steady load `F`. The drive records are CPU-owned
  and re-uploaded as one range whenever any of them changes (the unit pool every frame, ~30 KB).
* **Locked rotation** (units, arrows) replaces the 6 × 6 solve by the linear 3 × 3 block; the orientation is then either
  frozen or set kinematically at the start of the step from the drive's yaw or from the velocity. Setting it before the
  broadphase keeps the contacts consistent with the orientation used in the solve; the velocity used is the previous
  step's, which is also what makes the step deterministic.
* **Retired slots** (`FLAG_DEAD`) count as static everywhere (`isStatic`), get an empty AABB, sit in no cell and no large
  list, are never coloured, are skipped by `Predict` / `Velocity` and collapse to a point in the vertex shader. Spawning
  into a slot uploads the definition and the drive from the CPU (their owner) and lets a small kernel write the
  GPU-owned state (pose, velocities, colour = uncoloured, events = 0) from a record buffer, one dispatch per batch. A
  slot must not be reused in the step it was retired in: a retired body pairs with nothing, so after one step the
  previous-manifold buffers hold nothing of it and the newcomer cannot warm start from the old body's contacts;
  `BodyPool` keeps retired slots back until the world has stepped. Nothing is compacted, so body indices stay stable
  and pools are contiguous ranges (one mesh range, one event readback each).
* **Events** are or / add atomics on a per-body word (kinds touched, impacts), which are order-independent, so the
  step stays bitwise reproducible; the words are sticky until the slot is respawned and are read back asynchronously
  per pool range. Whether a projectile is spent (touched anything: its drive is switched off, and it is retired after a
  cooldown) and whether a unit was hit (an impact by a projectile faster than 2 m/s) both come from them; the CPU never
  inspects contacts.
* **Ballistics**: implicit Euler falls `g t h / 2` further than the parabola after a time `t` (`x_n = x_0 + v_0 t + g
  h² n (n + 1) / 2`), so the launch velocity of a projectile gets `+ g h / 2` upward and the discrete trajectory passes
  through the aim point exactly (`SiegeTests`: closest approach 2e-3 m).

A D3D12 note from the sleeping work: `BuildArgs` writing the indirect argument slots 0..6 as one contiguous run of
`Store3`s made the kernel do nothing at all (no error, no output — presumably the stores were coalesced into something
the driver rejected); the slot of the second pair round is therefore written in phase 1, next to the pair count,
where the stores are not contiguous.

## Sleeping

A sleeping body is frozen: it keeps its pose, its zero velocities and — through the cold store and the skipped joint
preparation — the exact penalties, multipliers and sticking anchors it fell asleep with, so no warm-start decay happens
while it sleeps and a sleeping castle stops creeping. It costs nothing while it sleeps: it is in no list the step runs
over (the hot list, the active joint list, the constraint index space), produces no pairs and sits in the sleeping
grid to be found. An awake body at the boundary of a sleeping region has its manifold with the sleeper recomputed
every step like any other (the sleeper is inactive, so the awake side emits the pair), and is solved against the
frozen neighbour exactly as against a static body: a manifold whose `C0` is the true error at `x⁻` and whose multiplier
carries the resting load. Only a manifold *without* an active body leaves the step, into the cold store.

**The cold store** keeps such manifolds — contacts, penalties, multipliers, stick anchors — with the *sleep generation*
of each body (a counter that advances whenever the body falls asleep, and when its slot is retired). A cold manifold is
valid for a body while the generation still matches and the body is static, asleep, or *fresh* (bit 28: it woke this
step, by a touch or the application): the round that gives a fresh body its contacts finds the pair in the cold hash,
warm starts from it and marks it dead, so nothing is ever warm started from a state of an earlier sleep, and a body
that wakes, moves off and comes back starts its contacts cold like any new pair. The pool is append-only; the store is
compacted — dead entries squeezed out in place, chunk by chunk through a scratch, the hash rebuilt — by its own command
buffer every `ColdCompactSteps` steps once a quarter of it was thawed or it is three quarters full (a gate on the GPU;
a compaction changes no result, the pool's order is not observable). With more frozen manifolds than
`MaxColdManifolds` (or contacts than `MaxColdContacts`) the surplus is dropped with the overflow flags 2 / 4: those
bodies wake with cold contacts and settle again.

Bodies rest independently — a body rests when it has stayed within the thresholds of a *rest anchor* for `SleepTime`, a
criterion that tolerates the sub-millimetre jitter of a resting pile but catches creep and a slow topple — but sleep
by neighbourhood: a body falls asleep only when every body within `SleepHops` contacts or joints of it rests as well.
This is deliberately not the island (the whole connected component): a 22 000-body pyramid always has a few bodies
creeping somewhere, and an island rule would keep all of them awake, while the neighbourhood rule keeps a two-hop ring
around each creeper awake and lets the rest sleep. The island is used for waking instead, where it matters:

* A constraint between a sleeping body and an awake one that is *not resting* wakes the sleeper in the same step,
  before the constraint lists and the solve. When the awake body is faster than `WakeSpeed` (an impact, a projectile, a
  walking unit) the whole island of the touched body wakes at once — a cannonball meets a wall that gives way, not a
  frozen one, and the shock propagates through the entire structure within the step as it does in an awake scene — and
  the island is held awake for `WakeHold` (its rest counters are set back), long enough for whatever lost its support
  to start falling and stay awake on its own motion. A slow toucher wakes the touched body alone, with its rest counter
  intact: if the touch does not move it, it goes back to sleep as soon as its neighbourhood allows; if it does, its
  motion resets the counter and, next step, wakes its own sleeping neighbours, so a slow push travels through a sleeping
  pile one body per step. A resting toucher wakes nothing: a pile creeping below the thresholds leans on its frozen
  neighbours instead of keeping them awake.
* A body woken by a touch was asleep at pair generation; the second narrowphase round gives it its contacts, warm
  started from the cold store, so it is solved on fresh manifolds like every other body (bit 30 of the sleep word marks
  it for that round and for the joint lists).
* The application wakes what it changes: spawns (sleep word and label reset), changed drives, flags, joints and
  springs added, removed or re-anchored, ignore links, whatever overlaps a retired body (found in both grids: its
  manifolds may be in the cold store, which no list indexes), and everything when the terrain or gravity changes
  (`WakeAll`, an upload of fresh words for the bodies on the GPU). A body woken this way is awake at pair generation,
  so its contacts are recomputed with the normal warm-start decay from the cold store, and `WakeTouch` then wakes what
  it touches.
* The application can build asleep (`SleepRange`): bodies added since the last step go up with the asleep bit, a rest
  counter no sleep time exceeds and a common island label, and the sleeping grid is rebuilt before their first step
  (their AABBs come from the rebuild), so they never enter the hot list, the hot grid or the pools — a world larger
  than its active capacity is built this way. They have no cold manifolds: woken, they start their contacts cold.

Islands are maintained incrementally by the label rounds above and only ever merge between relabels, so debris that
left an island shares its label for up to `SleepRelabelSteps` (a touch on it wakes the old island meanwhile). Labels
that have not converged when a group falls asleep, or during the steps after a relabel, split an island into label
groups: a touch wakes the touched group, and the neighbouring groups act as static for that one step (their `C0` is
still exact) before `WakeTouch` wakes them through the now awake boundary. Sleeping is off in the reference, and off in
every GPU test that compares against it: with it on, resting bodies stop where the reference keeps creeping.

## Determinism

Atomics are only used where order does not matter (counts, slot allocation, or-ing wake bits, minima); buckets and
per-body constraint lists are sorted afterwards, the colouring uses hashed priorities, same-colour bodies never read
each other, and the island and rest propagations are Jacobi rounds over ping-pong buffers (every body writes its own
entry from last round's values). Two runs of the same scene on the same GPU and driver are bitwise identical
(`GpuKernelTests.RunsAreBitwiseDeterministic`, `SleepTests.SleepingIsBitwiseDeterministic`).

## Differences from the reference

* Pairs are keyed with the lower body index as A (the C++ demo iterates its newest-first list, making the newer body
  A). The contact basis and the clipping reference face follow the order, so the reference can be run in the GPU's
  convention (`Solver.LowIndexFirst`) for like-for-like comparisons.
* The Gauss–Seidel order is the colour order, not creation order. Both orders converge to the same implicit Euler
  solution; at low iteration counts stiff chains differ (springs with a 1000 : 1 stiffness ratio: 0.9 m at 10
  iterations, 1e-4 at 100).
* Gravity is a vector (Unity is Y-up, the reference Z-up); the adaptive initial guess weights the measured
  acceleration along the gravity direction.
* Options that the 3D reference does not have: post-stabilisation (from the 2D reference) and the rotated inertia
  `R I Rᵀ` (paper Eq. 8) instead of the body-frame diagonal.
