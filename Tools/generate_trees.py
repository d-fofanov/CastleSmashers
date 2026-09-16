"""Generates the trees of Assets/Resources/Trees as brick-assembly documents (Assets/Phys/BRICK_ASSEMBLY.md): six kinds of
tree, one to three thousand pieces each, that the castle demo scatters around the castle and the preview demo shows on their
own.

A tree is designed as stud cells course by course (trunk, knots, crown, tiers), the way the citadel re-bonding tool sees a
castle, and then tiled into bricks. What keeps it standing on the solver is the corbel rule applied to every course: a
course may reach at most one stud beyond the course below, and only next to a cell that rests on it, so that the crown's
underside is an inverted cone that widens one stud per course out of the trunk and nothing ever hangs in the air (the knots
of a trunk step one stud per course the same way, and end below the crown: a stub the crown grew over would be a lever).
The overhanging cells of a course are tiled first, with 1 x 4 and 2 x 4 pieces that reach inward to keep three quarters of
their footprint on the course below (a half-hanging piece tips under the course above and pops); the rest of the course is
tiled by the re-bonding tool's course tiler (a running bond whose seams avoid the seams below) with bricks of two studs at
most inside the crown, so that a crown of a given piece count weighs a third of what it would in 2 x 4s (the crown's weight,
over the few contacts of the trunk top, is what makes a tree sink), and a single brick stacked on another single is dropped
(a column of singles topples). Trunks are rectangular (a round one leaves singles under the crown), 8 x 8 under a broad
crown, and every crown is centred on its trunk: a dry stack takes no tension, and an off-centre crown leans it over. Every
piece is then coloured from a smooth noise, lighter towards the top and the outside of the crown, so that the foliage reads
as leaves and the trunk as bark. The pine grows its underside as wide as the rule allows (an octagonal cone) and stacks
tiers on it, each two studs in from the last, with Ramps on their rims and a Roof_Prism on top; the cypress ends in a prism
too. Generating takes a few seconds; the six documents are checked in.

    python Tools/generate_trees.py [Assets/Resources/Trees]
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rebond_castle as rb   # noqa: E402  the course tiler (running bond, support costs) and the document writer

COURSE = rb.COURSE
FORMAT = dict(format="brick-assembly", version=1, catalog="generic-construction-27-v1", units={"gridToUnity": 0.1, "axes": "unity"})
BARK_LENGTHS = [4, 3, 2, 1]                     # the trunk: bricks up to four studs long
LEAF_LENGTHS = [2, 1]                           # the crown's inside: two studs at most
RIM_SHAPES = [(1, 4), (2, 2), (2, 3), (2, 4)]   # width x length of the pieces that cover overhanging cells (three quarters resting)
RIM_SUPPORT = 0.75                              # a corbel overhangs a quarter of its stone: half-hanging pieces tip and pop


# ------------------------------------------------------------------------------------------------ cells
def nbrs4(c):
    x, z = c
    return ((x + 1, z), (x - 1, z), (x, z + 1), (x, z - 1))


def disc(cx, cz, rx, rz=None, wobble=()):
    """Cells whose centre lies in the ellipse of semi-axes rx, rz around (cx, cz); `wobble` = [(amplitude, lobes, phase)]
    modulates the radius with the angle (a crown is never quite round)."""
    rz = rx if rz is None else rz
    cells = set()
    if rx <= 0 or rz <= 0: return cells
    for x in range(int(math.floor(cx - rx - 1)), int(math.ceil(cx + rx + 1))):
        for z in range(int(math.floor(cz - rz - 1)), int(math.ceil(cz + rz + 1))):
            dx, dz = x + 0.5 - cx, z + 0.5 - cz
            f = 1.0
            if wobble:
                a = math.atan2(dz, dx)
                f += sum(amp * math.sin(lobes * a + phase) for amp, lobes, phase in wobble)
            if (dx / (rx * f)) ** 2 + (dz / (rz * f)) ** 2 <= 1.0: cells.add((x, z))
    return cells


def square(half, cut=0):
    """Cells of the square spanning [-half, half) on both axes, with `cut` cells chamfered off every corner."""
    cells = set()
    for x in range(-half, half):
        for z in range(-half, half):
            ex, ez = min(x + half, half - 1 - x), min(z + half, half - 1 - z)   # distance to the nearest edges
            if ex + ez < cut: continue
            cells.add((x, z))
    return cells


def block(x0, z0, w, l):
    """Cells of the w x l block with its min corner at (x0, z0)."""
    return {(x, z) for x in range(x0, x0 + w) for z in range(z0, z0 + l)}


def ellipsoid(t, c0, c1, lobes, material, jitter=0.4, wobble=None, squash=(1.0, 1.0)):
    """Adds the union of ellipsoidal lobes [(x, z, centre course, radius, half-height in courses)] to the courses c0 .. c1."""
    for c in range(c0, c1 + 1):
        for (lx, lz, lc, R, H) in lobes:
            f = 1 - ((c - lc) / H) ** 2
            if f <= 0: continue
            r = R * math.sqrt(f)
            w = wobble(c) if wobble else ()
            t.add(c, disc(lx + t.rng.uniform(-jitter, jitter), lz + t.rng.uniform(-jitter, jitter), r, r * t.rng.uniform(*squash), wobble=w), material)


# ------------------------------------------------------------------------------------------------ the model
class Tree:
    """Courses of cells with a material each ('bark', 'leaf'), slopes and prisms placed as they are, and the corbel rule
    applied to the courses from the ground up as they are tiled."""

    def __init__(self, name, key, palette, seed):
        self.name, self.key, self.palette = name, key, palette
        self.rng = random.Random(seed)
        self.courses = []     # course -> {(x, z): material}
        self.extras = []      # (course, piece, x, z, yaw, colour): slopes and prisms placed after finish()
        self.tiled = []       # course -> [(piece, x, z, yaw, material, cells)] once finished
        self.dropped = 0

    def course(self, c):
        while len(self.courses) <= c: self.courses.append({})
        return self.courses[c]

    def add(self, c, cells, material):
        """Adds cells to a course (an earlier material wins where they overlap)."""
        course = self.course(c)
        for cell in cells:
            if cell not in course: course[cell] = material

    def extra(self, c, piece, x, z, yaw, colour):
        """A slope or prism placed as it is, after finish(): on cells the course below kept and the course above never had."""
        self.extras.append((c, piece, x, z, yaw, colour))

    def finish(self):
        """Tiles the courses from the ground up under the corbel rule: a cell that does not rest on the course below needs an
        orthogonal neighbour that does and a rim piece to hold it (two cells flanking a convex corner share one supporter: one of
        them goes), and a single brick stacked on another single goes too (a column of singles topples). What fails is dropped
        course by course, so that every course is clamped against what the course below finally keeps."""
        below, tops, singles_below = set(), {}, set()
        self.tiled, self.dropped = [], 0
        for c, course in enumerate(self.courses):
            while True:
                if c > 0:
                    bad = [u for u in course if u not in below and not any(v in below and v in course for v in nbrs4(u))]
                    if bad:
                        for u in bad: del course[u]
                        self.dropped += len(bad)
                        continue
                pieces, failed = tile_course(course, below if c > 0 else set(course), tops, c % 2)
                if failed is not None:
                    del course[failed]
                    self.dropped += 1
                    continue
                weak = [cells[0] for (piece, x, z, yaw, material, cells) in pieces if piece == "Brick_1x1" and cells[0] in singles_below]
                if weak:
                    for u in weak: del course[u]
                    self.dropped += len(weak)
                    continue
                break
            self.tiled.append(pieces)
            below = set(course)
            tops = {cell: k for k, (piece, x, z, yaw, material, cells) in enumerate(pieces) for cell in cells}
            singles_below = {cells[0] for (piece, x, z, yaw, material, cells) in pieces if piece == "Brick_1x1"}
        return self


# ------------------------------------------------------------------------------------------------ tiling
def rim_pieces(course, below):
    """Covers every cell that does not rest on the course below with a piece reaching inward: three quarters of it on the course
    below (a 1 x 4 or 2 x 4 with its outer stud row hanging, a corner cell of a smaller piece), covering the most overhanging
    cells, then the largest; the piece takes the material of the overhanging cell it was chosen for (a crown eats a little into
    the trunk top it grows out of). The cells with the fewest supporters go first and no piece may take the last supporter of
    another overhanging cell. Returns [(w, l, x0, z0, material)], the cells taken and the first cell no piece can hold (or None)."""
    unsupported = {u for u in course if u not in below}
    supporters = {u: [v for v in nbrs4(u) if v in below and v in course] for u in unsupported}
    order = sorted(unsupported, key=lambda u: (len(supporters[u]), -(u[0] * u[0] + u[1] * u[1]), u))
    taken = set()
    pieces = []
    for u in order:
        if u in taken: continue
        best = None
        for (w, l) in RIM_SHAPES:
            for (pw, pl) in ((w, l), (l, w)):
                for x0 in range(u[0] - pw + 1, u[0] + 1):
                    for z0 in range(u[1] - pl + 1, u[1] + 1):
                        cells = [(x, z) for x in range(x0, x0 + pw) for z in range(z0, z0 + pl)]
                        if any(c not in course or c in taken for c in cells): continue
                        support = sum(1 for c in cells if c in below) / len(cells)
                        if support < RIM_SUPPORT - 1e-9: continue
                        cellset = set(cells)
                        stranded = any(w_ not in taken and w_ not in cellset and all(v in taken or v in cellset for v in supporters[w_])
                                       for w_ in unsupported if w_ != u)
                        if stranded: continue
                        covered = sum(1 for c in cells if c in unsupported)
                        score = (covered, len(cells), support, -x0, -z0)
                        if best is None or score > best[0]: best = (score, pw, pl, x0, z0, cells)
        if best is None: return pieces, taken, u
        _, pw, pl, x0, z0, cells = best
        taken.update(cells)
        pieces.append((pw, pl, x0, z0, course[u]))
    return pieces, taken, None


def brick_of(w, l):
    """The catalog brick covering w cells along x and l along z, with its yaw."""
    return (f"Brick_{w}x{l}", 0) if w <= l else (f"Brick_{l}x{w}", 90)


def tile_course(course, below, tops, parity):
    """Tiles one course: the overhanging cells first, then the running bond per material (`tops`: the cells of the course below
    keyed to the piece there, whose seams the bond avoids). Returns [(piece, x, z, yaw, material, cells)] and the first cell no
    rim piece can hold (or None)."""
    rim, taken, failed = rim_pieces(course, below)
    if failed is not None: return [], failed
    pieces = []
    for (pw, pl, x0, z0, material) in rim:
        piece, yaw = brick_of(pw, pl)
        cells = [(x, z) for x in range(x0, x0 + pw) for z in range(z0, z0 + pl)]
        pieces.append((piece, x0 + pw / 2, z0 + pl / 2, yaw, material, cells))
    for material, lengths in (("bark", BARK_LENGTHS), ("leaf", LEAF_LENGTHS)):
        rest = {cell: (material, material) for cell in course if cell not in taken and course[cell] == material}
        if not rest: continue
        rb.LENGTHS = lengths
        tiler = rb.Course(0.0, rest, {cell: tops.get(cell, "ground") for cell in rest})
        for (piece, x, z, yaw, mat, _structure, cells) in tiler.tile(parity):
            pieces.append((piece, x, z, yaw, mat, cells))
    return pieces, None


def tile_tree(tree):
    """The parts of a finished tree with their ids, positions and colours (material names, coloured later), and per-course
    statistics (course, cells, pieces)."""
    parts = []
    counters = {}
    stats = []
    for c, pieces in enumerate(tree.tiled):
        y = round(c * COURSE, 3)
        for (piece, x, z, yaw, material, cells) in pieces:
            structure = "trunk" if material == "bark" else "crown"
            counters[structure] = counters.get(structure, 0) + 1
            pid = f"{structure}/c{c}_{counters[structure]}"
            part = dict(id=pid, piece=piece, position=[x, y, z], color=material, cells=cells, course=c)
            if yaw: part["rotation"] = [0, yaw, 0]
            parts.append(part)
        stats.append((c, len(tree.courses[c]), len(pieces)))
    for (c, piece, x, z, yaw, colour) in tree.extras:
        counters[piece] = counters.get(piece, 0) + 1
        part = dict(id=f"{piece.lower()}s/c{c}_{counters[piece]}", piece=piece, position=[x, round(c * COURSE, 3), z], color=colour, course=c)
        if yaw: part["rotation"] = [0, yaw, 0]
        parts.append(part)
    return parts, stats


# ------------------------------------------------------------------------------------------------ colours
def value_noise(x, y, z, seed, wavelength):
    """Smooth noise in [0, 1]: trilinear interpolation of hashed lattice values."""
    def lattice(i, j, k):
        h = (i * 73856093) ^ (j * 19349663) ^ (k * 83492791) ^ (seed * 2654435761)
        h = ((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF
        return ((h ^ (h >> 16)) & 0xFFFF) / 65535.0
    fx, fy, fz = x / wavelength, y / wavelength, z / wavelength
    ix, iy, iz = math.floor(fx), math.floor(fy), math.floor(fz)
    def smooth(u): return u * u * (3 - 2 * u)
    tx, ty, tz = smooth(fx - ix), smooth(fy - iy), smooth(fz - iz)
    v = 0.0
    for dx in (0, 1):
        for dy in (0, 1):
            for dz in (0, 1):
                w = (tx if dx else 1 - tx) * (ty if dy else 1 - ty) * (tz if dz else 1 - tz)
                v += w * lattice(ix + dx, iy + dy, iz + dz)
    return v


def colour_parts(tree, parts, seed):
    """Palette keys per piece: foliage in three shades, lighter towards the top and the outside of the crown; bark in two, with
    the dark marks of a birch where the palette has them."""
    crown = [p for p in parts if p["color"] == "leaf"]
    c_lo, c_hi, radius = 0, 1, {}
    if crown:
        c_lo, c_hi = min(p["course"] for p in crown), max(p["course"] for p in crown)
        for p in crown: radius[p["course"]] = max(radius.get(p["course"], 0.0), math.hypot(p["position"][0], p["position"][2]))
    marks = "bark_dark" in tree.palette
    rng = random.Random(seed)
    for p in parts:
        x, y, z = p["position"]
        n = value_noise(x, y * 2.5, z, seed, 4.5)
        if p["color"] == "leaf":
            up = (p["course"] - c_lo) / max(c_hi - c_lo, 1)
            out = math.hypot(x, z) / max(radius.get(p["course"], 1.0), 1.0)
            v = 0.55 * n + 0.28 * up + 0.22 * out
            p["color"] = "leaf_dark" if v < 0.42 else "leaf" if v < 0.66 else "leaf_light"
        elif p["color"] == "bark":
            p["color"] = "bark_dark" if marks and rng.random() < 0.18 else "bark_light" if n > 0.58 else "bark"
    return parts


# ------------------------------------------------------------------------------------------------ the trees
def slope_ring(tree, c, colour):
    """After finish(): Ramps on the rim of a tier, on the cells of course c - 1 that course c left free (the tier top around the
    next tier), wherever a 2 x 2 of them fits with its outer edge on the rim, rising towards the trunk: the +x side rises towards
    -x (yaw 270), the -x side towards +x (90), the +z side towards -z (180), the -z side towards +z (0)."""
    below = set(tree.courses[c - 1])
    ring = below - set(tree.courses[c])
    placed = set()
    for (x, z) in sorted(ring, key=lambda u: (-(u[0] * u[0] + u[1] * u[1]), u)):
        cells = {(x, z), (x + 1, z), (x, z + 1), (x + 1, z + 1)}
        if not cells <= ring or cells & placed: continue
        cx, cz = x + 1, z + 1
        if abs(cx) >= abs(cz): outward, yaw = (((x + 2, z), (x + 2, z + 1)), 270) if cx > 0 else (((x - 1, z), (x - 1, z + 1)), 90)
        else: outward, yaw = (((x, z + 2), (x + 1, z + 2)), 180) if cz > 0 else (((x, z - 1), (x + 1, z - 1)), 0)
        if any(u in below for u in outward): continue   # not on the rim of that side
        placed |= cells
        tree.extra(c, "Ramp", x + 1, z + 1, yaw, colour)


def trunk(t, half, courses):
    """A square trunk of the given half-width straight out of the ground: a flared foot would carry nothing while the trunk
    above it sinks its centimetre under the crown, and the pieces bridging the two would tear their snaps."""
    for c in range(0, courses): t.add(c, square(half), "bark")


def limbs(t, directions, starts, length, half):
    """Limb stubs three studs wide and four long leaving a trunk of the given half-width along the axes given as (dx, dz), one
    stud further out per course for `length` courses: bark showing under the crown, whose cone grows over them. A crown must
    stay centred on its trunk (a dry stack takes no tension), so the stubs carry no lobes of their own."""
    for (dx, dz), s in zip(directions, starts):
        for k in range(length + 1):
            reach = half - 3 + k             # the block's inner end starts three studs inside the trunk face
            if dx: cells = block(reach if dx > 0 else -reach - 4, -1, 4, 3)
            else: cells = block(-1, reach if dz > 0 else -reach - 4, 3, 4)
            t.add(s + k, cells, "bark")


def oak(seed):
    t = Tree("Oak", "oak", dict(bark="#5A4030", bark_light="#6E5240", leaf_dark="#2E5A2A", leaf="#3F7A33", leaf_light="#5C9A40"), seed)
    rng = t.rng
    trunk(t, 4, 13)
    limbs(t, [(1, 0), (-1, 0), (0, 1), (0, -1)], [rng.randint(6, 7) for _ in range(4)], 3, 4)   # knots two courses under the crown
    ellipsoid(t, 13, 40, [(0.0, 0.0, 21, 11.5, 10.0)], "leaf", jitter=0.25, wobble=lambda c: [(0.06, 3, c * 0.9), (0.04, 5, c * 0.4)])
    return t.finish()


def erode(cells):
    """The cells whose four neighbours are all in the set (one stud in from the boundary)."""
    return {c for c in cells if all(v in cells for v in nbrs4(c))}


def pine(seed):
    t = Tree("Pine", "pine", dict(bark="#4E3826", bark_light="#61482F", leaf_dark="#1E4A32", leaf="#2A6440", leaf_light="#3D7F4E"), seed)
    trunk_top, cone = 6, 9
    trunk(t, 4, trunk_top + 3)
    for k in range(cone): t.add(trunk_top + k, square(13), "leaf")           # the underside: as wide as the corbel rule lets it grow
    t.finish()                                                               # (an octagonal cone), so the tiers start from what it kept
    tier, c, rings = set(t.courses[trunk_top + cone - 1]), trunk_top + cone, []
    while len(tier) >= 16:                                                   # tiers of three courses, each two studs in from the last,
        for k in range(3):                                                   # their rims bevelled with ramps
            t.add(c, tier, "leaf")
            c += 1
        tier = erode(erode(tier))
        rings.append(c)
        t.add(c, tier, "leaf")
        c += 1
    t.add(c, square(1), "leaf")                                              # the tip
    t.finish()
    for ring in rings: slope_ring(t, ring, "leaf")
    if t.courses[c]: t.extra(c + 1, "Roof_Prism", 0, 0, 0, "leaf_light")
    return t


def cypress(seed):
    t = Tree("Cypress", "cypress", dict(bark="#4A3A2C", bark_light="#5C4A38", leaf_dark="#1B3D2A", leaf="#26523A", leaf_light="#356A48"), seed)
    trunk(t, 3, 3)
    H, R = 40, 7.4
    ellipsoid(t, 2, H, [(0.0, 0.0, H / 2, R, H / 2)], "leaf", jitter=0.3, wobble=lambda c: [(0.08, 4, c * 0.7), (0.05, 7, c * 1.3)], squash=(0.9, 1.0))
    top = len(t.courses)
    t.add(top, square(1), "leaf")
    t.finish()
    if t.courses[top]: t.extra(top + 1, "Roof_Prism", 0, 0, 90, "leaf_light")
    return t


def elm(seed):
    t = Tree("Elm", "elm", dict(bark="#55453A", bark_light="#6A5848", leaf_dark="#5C6E1E", leaf="#7C8F2A", leaf_light="#A3B03A"), seed)
    rng = t.rng
    trunk_top = 8
    trunk(t, 4, trunk_top + 4)
    R, flare, top = 13.0, 11, 7                                              # the vase: nearly a stud per course out to R, then a flat dome
    for k in range(flare + top + 1):
        c = trunk_top + k
        r = 4.0 + k * (R - 4.0) / flare if k <= flare else R * math.sqrt(max(1 - ((k - flare) / (top + 0.5)) ** 2, 0.0))
        t.add(c, disc(rng.uniform(-0.25, 0.25), rng.uniform(-0.25, 0.25), r, r * rng.uniform(0.94, 1.0), wobble=[(0.06, 5, c * 0.8), (0.04, 3, c * 1.7)]), "leaf")
    return t.finish()


def birch(seed):
    t = Tree("Birch", "birch", dict(bark="#E4E2DA", bark_light="#F2F0E8", bark_dark="#3A3A38", leaf_dark="#5E8A2E", leaf="#7FAA3A", leaf_light="#A6C94E"), seed)
    rng = t.rng
    trunk_top = 12
    trunk(t, 4, trunk_top)
    for c in range(trunk_top, trunk_top + 4): t.add(c, square(3), "bark")   # the trunk goes on into the crown, narrower
    a = rng.uniform(0, math.pi)                                              # two side lobes opposite each other: the crown stays centred
    lobes = [(0.0, 0.0, 21, 8.3, 13.0), (3.5 * math.cos(a), 3.5 * math.sin(a), 19, 6.2, 7.0), (-3.5 * math.cos(a), -3.5 * math.sin(a), 19, 6.2, 7.0)]
    ellipsoid(t, trunk_top, 40, lobes, "leaf", jitter=0.25, wobble=lambda c: [(0.08, 4, c * 1.1), (0.05, 6, c * 0.5)])
    return t.finish()


def maple(seed):
    t = Tree("Maple", "maple", dict(bark="#4F3B2E", bark_light="#65503F", leaf_dark="#9A3A1C", leaf="#C8581E", leaf_light="#E8892A"), seed)
    rng = t.rng
    trunk_top = 10
    trunk(t, 4, trunk_top)
    limbs(t, [(1, 0), (-1, 0), (0, 1), (0, -1)], [rng.randint(4, 5) for _ in range(4)], 2, 4)   # knots under the crown
    ellipsoid(t, trunk_top, 32, [(0.0, 0.0, 18, 12.0, 9.0)], "leaf", jitter=0.25, wobble=lambda c: [(0.07, 3, c * 0.7), (0.05, 6, c * 0.3)], squash=(0.92, 1.0))
    return t.finish()


TREES = [oak, pine, cypress, elm, birch, maple]


# ------------------------------------------------------------------------------------------------ checks and output
def check(parts):
    """What the loader's diagnostics would report: pieces resting on nothing or on less than half their cells; the extremes."""
    by_course = {}
    for p in parts:
        if "cells" in p: by_course.setdefault(p["course"], {}).update({cell: p["id"] for cell in p["cells"]})
    floating = poor = 0
    for p in parts:
        if "cells" not in p or p["course"] == 0: continue
        below = by_course.get(p["course"] - 1, {})
        supported = sum(1 for cell in p["cells"] if cell in below)
        if supported == 0: floating += 1
        elif supported * 2 < len(p["cells"]): poor += 1
    cells = sum(len(p["cells"]) for p in parts if "cells" in p)
    xs = [p["position"][0] for p in parts]; zs = [p["position"][2] for p in parts]
    return dict(floating=floating, poor=poor, cells=cells, courses=max(p["course"] for p in parts) + 1,
                width=max(xs) - min(xs) + 2, depth=max(zs) - min(zs) + 2)


def document(tree, parts):
    out = []
    for p in parts:
        q = dict(id=p["id"], piece=p["piece"], position=p["position"], color=p["color"])
        if p.get("rotation"): q["rotation"] = p["rotation"]
        out.append(q)
    return dict(FORMAT, name=tree.name, palette=tree.palette, parts=out)


if __name__ == "__main__":
    folder = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Assets", "Resources", "Trees")
    os.makedirs(folder, exist_ok=True)
    for k, make in enumerate(TREES):
        tree = make(seed=101 + k)
        parts, stats = tile_tree(tree)
        colour_parts(tree, parts, seed=7 + k)
        info = check(parts)
        path = os.path.join(folder, tree.key + ".json")
        rb.write(document(tree, parts), path)
        pieces = {}
        for p in parts: pieces[p["piece"]] = pieces.get(p["piece"], 0) + 1
        top = ", ".join(f"{n} {name}" for name, n in sorted(pieces.items(), key=lambda kv: -kv[1])[:5])
        print(f"{tree.name:8s} {len(parts):5d} pieces, {info['cells']} cells, {info['courses']} courses ({info['courses'] * COURSE * 0.5:.1f} m at scale 5), "
              f"{info['width']:.0f} x {info['depth']:.0f} studs; floating {info['floating']}, poorly supported {info['poor']}, dropped {tree.dropped}: {top} -> {os.path.relpath(path)}")
