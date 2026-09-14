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

Every kernel is recorded once into a `CommandBuffer`; every count-dependent dispatch is indirect, so the buffer is
re-recorded only when the iteration count, the post-stabilisation flag or the active colour count changes. Per-step
constants (dt, gravity, α, β, γ, counts) live in one constant buffer.

1. **World AABBs** (`BodyAabb`) at `x⁻`, expanded by the collision margin. Bodies spanning more than 64 grid cells
   (the 100 m grounds) go to a small *large-body list* that every body tests brute force instead of the grid.
2. **Hashed uniform grid** (`GridCount` → scan → `GridScatter` → `GridSortCell`): every body is inserted into every
   cell its AABB overlaps (counting sort with atomics and an exclusive scan); each bucket is then sorted by body index.
   The cell size defaults to twice the mean extent of the dynamic bodies.
3. **Pairs** (`PairGen`): a body walks its cells and considers bucket entries with a higher index. A pair is emitted
   only from the cell that contains the minimum corner of the two AABBs' intersection (*owner-cell rule*), so a pair
   overlapping many cells is emitted exactly once and hash aliasing can only add candidates that the AABB test
   rejects. Static–static pairs are skipped, and pairs joined by a joint (unless broken), a spring or an
   ignore-collision link are filtered through a sorted per-body link list.
4. **Narrowphase** (`Collide`): the reference's OBB test — SAT over 15 axes with an edge preference tolerance, face
   clipping (incident face against the reference face's side planes, up to 8 points, feature keys from the reference
   axis, incident axis and vertex index) or closest points of the two support edges. The previous step's manifold of
   the same pair is found in an open-addressing hash table keyed by the pair; per feature key the penalty, multiplier,
   stick flag and, for sticking contacts, the old anchor points are carried over, then the warm-start decay is applied.
   Contacts are appended to a pool (one atomic per manifold), the header to the manifold list, the pair to the new
   hash table.
5. **Joints** (`PrepareJoints`): `C0` of the ball-socket and the angular lock at `x⁻`, decay, penalty capped at the
   material stiffness.
6. **Constraint lists** (`ConsCount` → scan → `ConsFill` → `ConsSort`): a CSR list per dynamic body of the manifolds,
   joints and springs touching it. Each list is sorted by a canonical key (the neighbour index for manifolds, the
   CPU-assigned index for joints and springs), so the accumulation order of the local system is deterministic.
7. **Colouring** (`ColorInvalidate` → `ColorRound` × R → `ColorFinalize` → `ColorScan` → `ColorScatter`): bodies that
   share a constraint must not be updated in the same parallel pass. Last step's colours are kept unless a
   higher-priority neighbour (hashed index) has the same one; uncoloured bodies then take the smallest colour that is
   free among coloured neighbours whenever every uncoloured neighbour has lower priority (Jones–Plassmann greedy, a
   few rounds). Whatever is still uncoloured joins the *overflow group*, which is updated Jacobi style with double
   buffered positions — always correct, only slower to converge. The colour count in use is read back asynchronously
   and the number of recorded colour passes adapts to it.
8. **Predict**: inertial pose, VBD adaptive initial guess (gravity weighted by the measured acceleration), `x⁻`.
9. **Iterations**: for every colour a `Primal` dispatch updates its bodies in place (they never read each other), the
   overflow group writes to a second buffer and is committed; then `Dual` updates every manifold and joint in
   parallel; the last iteration ends with `Velocity` (`v = (x − x⁻)/h`, `ω = 2 (q q⁻⁻¹)ₓᵧᵤ/h`). With
   post-stabilisation on, the main iterations use α = 1 and one extra position-only iteration uses α = 0.
10. The manifold, contact and hash buffers are copied to their "previous" twins (`CopyBuffer`) and the counters are
    copied to a stats buffer for the asynchronous readback.

Rendering never touches the CPU: one `RenderMeshPrimitives` cube draw reads the pose and definition buffers by
instance id; contact crosses and joint lines are written by small kernels into vertex buffers and drawn with
`RenderPrimitivesIndirect`.

## Determinism

Atomics are only used where order does not matter (counts, slot allocation); buckets and per-body constraint lists
are sorted afterwards, the colouring uses hashed priorities, and same-colour bodies never read each other. Two runs
of the same scene on the same GPU and driver are bitwise identical (`GpuKernelTests.RunsAreBitwiseDeterministic`).

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
