"""Re-bonds the masonry of a brick-assembly document (Assets/Phys/BRICK_ASSEMBLY.md) so that the castle holds together on
the solver, without changing what it looks like.

An agent's design usually stacks the same brick straight up course after course: walls become independent columns two
studs thick, rings become four columns that never interlock, decks span hollows on nothing. The look of the castle is the
set of stud cells each course fills and their colours; the seams between bricks are not. So every course of upright
bricks is expanded into its cells and tiled again, cell for cell and colour for colour, choosing the bricks so that
their seams avoid the seams of the course below (a running bond), so that corners and wall-to-tower junctions change
their owner every course (the interlock of a masonry ring), and so that each brick rests on as much of the course below
as it can. Plates, tiles, slopes, cylinders and bricks that do not stand upright on the grid are kept as they are.

The Emerald Crown Citadel additionally gets the structure its design implies but does not build: two cross walls inside
the hollow keep under its roof deck (the front one with an opening behind the door), a post inside each side hall under
the middle of its roof, two more beams under the bridge deck, and a plate bracket under every banner (the banner stands
on it and leans on the wall; its gold crest stands on the bracket in front of the banner). Lintels over the gate and the
doors come out of the re-tiling by themselves. Everything visible keeps its place and colour, except that the crests
move down onto their brackets and the gatehouse banner moves down by a fifth of a stud.

    python Tools/rebond_castle.py Tools/castles/emerald_crown_citadel.design.json Assets/Resources/Castles/emerald_crown_citadel.json
"""
import collections
import json
import math
import sys

# ------------------------------------------------------------------------------------------------ the catalog
BRICKS = {f"Brick_{w}x{l}": (w, l) for w, l in [(1, 1), (1, 2), (1, 3), (1, 4), (1, 6), (1, 8), (2, 2), (2, 3), (2, 4), (2, 6), (2, 8)]}
FOOTPRINT = dict(BRICKS)
FOOTPRINT.update({f"Plate_{w}x{l}": (w, l) for w, l in [(1, 1), (1, 2), (1, 4), (2, 2), (2, 4)]})
FOOTPRINT.update({f"Tile_{w}x{l}": (w, l) for w, l in [(1, 1), (1, 2), (2, 2), (2, 4)]})
FOOTPRINT.update({"Ramp": (2, 2), "Roof_Prism": (2, 2), "Wedge": (2, 3), "Plain_Cube": (1, 1), "Plain_Rectangular_Block": (2, 3),
                  "Plain_Cylinder": (1.5, 1.5), "Plain_Triangular_Prism": (2, 2)})
HEIGHT = {p: 1.2 for p in BRICKS}
HEIGHT.update({p: 0.4 for p in FOOTPRINT if p.startswith("Plate_") or p.startswith("Tile_")})
HEIGHT.update({"Ramp": 1.2, "Roof_Prism": 1.2, "Wedge": 0.8, "Plain_Cube": 1, "Plain_Rectangular_Block": 1.2, "Plain_Cylinder": 1.5,
               "Plain_Triangular_Prism": 1.2})
COURSE = 1.2
PLATE = 0.4
LENGTHS = [8, 6, 4, 3, 2, 1]   # 1-wide and 2-wide bricks (a 2 x 1 is a Brick_1x2 turned across the strip)
IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


# ------------------------------------------------------------------------------------------------ expansion
def mat_mul(a, b): return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def mat_vec(a, v): return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def rotation(e):
    """[x, y, z] degrees, applied Z first, then X, then Y (Unity's Euler convention)."""
    x, y, z = [math.radians(v) for v in e]
    c, s = math.cos, math.sin
    rx = [[1, 0, 0], [0, c(x), -s(x)], [0, s(x), c(x)]]
    ry = [[c(y), 0, s(y)], [0, 1, 0], [-s(y), 0, c(y)]]
    rz = [[c(z), -s(z), 0], [s(z), c(z), 0], [0, 0, 1]]
    return mat_mul(ry, mat_mul(rx, rz))


def euler_of(m):
    """The [x, y, z] Euler angles (multiples of 90) of a rotation made of quarter turns."""
    for x in (0, 90, 180, -90):
        for y in (0, 90, 180, -90):
            for z in (0, 90, 180, -90):
                r = rotation([x, y, z])
                if all(abs(r[i][j] - m[i][j]) < 1e-6 for i in range(3) for j in range(3)):
                    return [x, y, z]
    raise SystemExit("a rotation that is not made of quarter turns")


def expand(doc):
    """Every part of the document in world space: id path, piece, position, rotation matrix, colour."""
    mods = doc.get("modules") or {}
    parts = []

    def walk(scope, prefix, pos, rot):
        for p in scope.get("parts", []):
            lp = mat_vec(rot, p["position"])
            parts.append(dict(id=prefix + p["id"], piece=p["piece"], pos=[pos[i] + lp[i] for i in range(3)],
                              rot=mat_mul(rot, rotation(p.get("rotation", [0, 0, 0]))), color=p.get("color")))
        for inst in scope.get("instances") or []:
            lp = mat_vec(rot, inst["position"])
            walk(mods[inst["module"]], prefix + inst["id"] + "/", [pos[i] + lp[i] for i in range(3)],
                 mat_mul(rot, rotation(inst.get("rotation", [0, 0, 0]))))
    walk(doc, "", [0, 0, 0], IDENTITY)
    return parts


def upright_yaw(rot):
    """0 or 90 for a piece standing upright, turned by a multiple of 90 degrees about Y; None otherwise."""
    if abs(rot[1][1] - 1) > 1e-6: return None
    zx = rot[0][2]   # world x component of local +Z
    if abs(abs(zx) - 1) < 1e-6: return 90
    if abs(zx) < 1e-6: return 0
    return None


def footprint_cells(piece, pos, yaw):
    """Integer stud cells [(cx, cz)] under an upright piece, or None if it is not aligned to the grid."""
    w, l = FOOTPRINT[piece]
    if yaw == 90: w, l = l, w
    x0, z0 = pos[0] - w / 2, pos[2] - l / 2
    if w != int(w) or l != int(l) or abs(x0 - round(x0)) > 1e-6 or abs(z0 - round(z0)) > 1e-6: return None
    x0, z0 = int(round(x0)), int(round(z0))
    return [(x, z) for x in range(x0, x0 + int(w)) for z in range(z0, z0 + int(l))]


def key_y(y): return int(round(y * 1000))


# ------------------------------------------------------------------------------------------------ tiling
def dp_tile(length, seam_cost, piece_cost):
    """Tiles a strip of `length` cells with pieces of LENGTHS, minimising pieces + seam costs + piece costs.
    seam_cost(p): the cost of a seam before cell p (0 < p < length); piece_cost(start, length): the cost of a piece."""
    inf = float("inf")
    best = [inf] * (length + 1)
    choice = [0] * (length + 1)
    best[0] = 0
    for i in range(1, length + 1):
        for ln in LENGTHS:
            if ln > i or best[i - ln] == inf: continue
            cost = best[i - ln] + 1 + piece_cost(i - ln, ln) + (seam_cost(i) if i < length else 0)
            if cost < best[i] - 1e-9: best[i], choice[i] = cost, ln
    pieces, i = [], length
    while i > 0:
        pieces.append((i - choice[i], choice[i]))
        i -= choice[i]
    return pieces[::-1]


class Course:
    """The cells of one course of upright bricks, tiled again into runs two studs wide, then one stud wide."""

    def __init__(self, y, cells, tops):
        self.y = y
        self.cells = cells   # (cx, cz) -> (colour, structure)
        self.tops = tops     # (cx, cz) -> id of the surface lying exactly under the course there (missing = nothing)
        self.free = set(cells)
        self.out = []

    def cell(self, axis, line, p): return (line, p) if axis == "z" else (p, line)

    def run(self, axis, lines, first):
        """Length of the same-colour run of free cells along `axis` from `first` over all `lines`."""
        n, color = 0, None
        while True:
            cs = [self.cell(axis, ln, first + n) for ln in lines]
            if not all(c in self.free for c in cs): break
            cols = {self.cells[c][0] for c in cs}
            if len(cols) != 1 or (color is not None and cols != {color}): break
            color = cols.pop()
            n += 1
        return n

    def is_start(self, axis, lines, first):
        prev = [self.cell(axis, ln, first - 1) for ln in lines]
        here = self.cells[self.cell(axis, lines[0], first)][0]
        return not all(c in self.free and self.cells[c][0] == here for c in prev)

    def extent(self, axis, c):
        """Length of the same-colour run of free cells along `axis` through cell c."""
        color = self.cells[c][0]
        n = 1
        for d in (1, -1):
            q = (c[0], c[1] + d) if axis == "z" else (c[0] + d, c[1])
            while q in self.free and self.cells[q][0] == color:
                n += 1
                q = (q[0], q[1] + d) if axis == "z" else (q[0] + d, q[1])
        return n

    def longest_run(self, axis, width):
        """The longest run of `width` adjacent free lines of one colour along `axis`: (first, length, lines). A two-wide run
        must be at least three long, or an isolated 2 x 2 block: the cross-section of a wall is two long and is not a run."""
        best = None
        other = "x" if axis == "z" else "z"
        for (cx, cz) in self.free:
            a, first = (cx, cz) if axis == "z" else (cz, cx)
            lines = tuple(range(a, a + width))
            if not self.is_start(axis, lines, first): continue
            n = self.run(axis, lines, first)
            if n == 0: continue
            if width == 2 and n < 3 and not (n == 2 and all(self.extent(other, self.cell(axis, ln, first + q)) <= 2 for ln in lines for q in range(2))): continue
            if best is None or n > best[1] or (n == best[1] and (a, first) < (best[2][0], best[0])): best = (first, n, lines)
        return best

    def tile_run(self, axis, lines, first, length):
        """Tiles the run with the DP: seams avoid the seams of the course below, pieces rest on as much as they can."""
        def below(p): return frozenset(self.tops.get(self.cell(axis, ln, first + p)) for ln in lines)

        def seam_cost(p):
            cost = 0
            if below(p - 1) != below(p): cost += 3                      # a seam of the course below runs right here
            if p >= 2 and below(p - 2) != below(p - 1): cost += 1       # or one stud away
            if p + 1 < length and below(p) != below(p + 1): cost += 1
            return cost

        def piece_cost(start, ln):
            supported = sum(1 for q in range(start, start + ln) for ln_ in lines if self.tops.get(self.cell(axis, ln_, first + q)) is not None)
            frac = supported / (ln * len(lines))
            return 10 if frac == 0 else (2 if frac < 0.5 else 0)

        color, structure = self.cells[self.cell(axis, lines[0], first)]
        for start, ln in dp_tile(length, seam_cost, piece_cost):
            cells = [self.cell(axis, ln_, first + start + q) for ln_ in lines for q in range(ln)]
            for c in cells: self.free.discard(c)
            if len(lines) == 2:
                if ln == 1: piece, yaw = "Brick_1x2", (90 if axis == "z" else 0)   # a 2 x 1: a 1x2 turned across the strip
                else: piece, yaw = f"Brick_2x{ln}", (0 if axis == "z" else 90)
            else:
                piece, yaw = f"Brick_1x{ln}", (0 if axis == "z" else 90)
            xs, zs = [c[0] for c in cells], [c[1] for c in cells]
            self.out.append((piece, (min(xs) + max(xs) + 1) / 2, (min(zs) + max(zs) + 1) / 2, yaw, color, structure, cells))

    def tile_with(self, first_axis):
        """Two-wide runs first, `first_axis` before the other, then one-wide runs; the longest run first within each pass."""
        self.free = set(self.cells)
        self.out = []
        for axis in ((first_axis, "x" if first_axis == "z" else "z")):
            while True:
                r = self.longest_run(axis, 2)
                if r is None: break
                self.tile_run(axis, r[2], r[0], r[1])
        while self.free:
            best = None
            for axis in ("z", "x"):
                r = self.longest_run(axis, 1)
                if r is not None and (best is None or r[1] > best[1][1]): best = (axis, r)
            axis, (first, n, lines) = best
            self.tile_run(axis, lines, first, n)
        return self.out

    def quality(self, out):
        """Lower is better: pieces resting on nothing or on less than half their cells, seams straight above seams, piece count."""
        owner = {}
        for k, (piece, x, z, yaw, color, structure, cells) in enumerate(out):
            for c in cells: owner[c] = k
        floating = poor = shared = 0
        for (piece, x, z, yaw, color, structure, cells) in out:
            supported = sum(1 for c in cells if self.tops.get(c) is not None)
            if supported == 0: floating += 1
            elif supported * 2 < len(cells): poor += 1
        for (x, z), k in owner.items():
            for nb in ((x + 1, z), (x, z + 1)):
                if nb in owner and owner[nb] != k and self.tops.get((x, z)) != self.tops.get(nb): shared += 1
        return 10 * floating + 2 * poor + shared + 0.1 * len(out)

    def tile(self, parity):
        """Corners and junctions change owner every course (the parity's axis goes first), unless the other order leaves
        fewer pieces hanging or fewer seams above seams (a lintel course)."""
        natural = "z" if parity == 0 else "x"
        a = self.tile_with(natural)
        qa = self.quality(a)
        b = self.tile_with("x" if natural == "z" else "z")
        qb = self.quality(b)
        if qb < qa - 1e-9:
            self.out = b
            return b
        self.out = a
        return a


# ------------------------------------------------------------------------------------------------ the citadel's fixes
class Fixes:
    """Extra brick cells (hidden structure), brick cells reserved for plates (the brackets), extra plate parts, moved parts."""

    def __init__(self):
        self.cells = []        # (y, cx, cz, colour, structure)
        self.reserved = set()  # (key_y, cx, cz)
        self.parts = []        # brick-assembly parts to add as they are
        self.moved = {}        # id -> new position

    def wall(self, y0, courses, xs, zs, color, structure, skip=lambda y, x, z: False):
        for k in range(courses):
            y = round(y0 + k * COURSE, 3)
            for x in xs:
                for z in zs:
                    if not skip(y, x, z): self.cells.append((y, x, z, color, structure))

    def bracket(self, y, x, z_face, outward, structure, color):
        """A Plate_2x4 along z at course `y`, one stud proud of the wall face at `z_face` (the wall lies on the other side of
        `outward`), reaching one stud into the hollow behind the two-stud wall; two Plate_2x2 fill the rest of the course."""
        inner = z_face - outward * 1       # the plate is centred one stud inside the face: covers face - 1 .. face + 3 (into the wall)
        self.parts.append(dict(id=f"{structure}/bracket", piece="Plate_2x4", position=[x, y, inner], color=color))
        for k in (1, 2):
            self.parts.append(dict(id=f"{structure}/bracket_fill{k}", piece="Plate_2x2", position=[x, round(y + PLATE * k, 3), inner], color=color))
        for cx in (x - 1, x):
            for cz in (inner - 1, inner):
                self.reserved.add((key_y(y), cx, cz))


def citadel_fixes(parts):
    f = Fixes()
    base = 1.6                                  # the island's top, where every building stands
    kx, kz = 0, 5                               # the keep's centre
    # the keep: two cross walls under the roof deck, the front one open behind the door for the first four courses
    f.wall(base, 18, range(kx - 8, kx + 8), [kz - 3, kz - 2], "stone", "central_keep/cross_front",
           skip=lambda y, x, z: y < base + 4 * COURSE - 1e-6 and kx - 2 <= x < kx + 2)
    f.wall(base, 18, range(kx - 8, kx + 8), [kz + 1, kz + 2], "stone", "central_keep/cross_back")
    # a post inside each side hall under the middle plate of its roof deck
    for hx in (-15, 15):
        f.wall(base, 6, [hx - 1, hx], [kz - 2, kz - 1, kz, kz + 1], "stone", f"side_hall_{hx}/post")
    # the bridge: two more beams under the middle of its deck
    f.wall(0, 1, range(-2, 2), range(-32, -24), "stone_dark", "approach_bridge/beam")
    # the towers' banners hang on the outward face from world 4.4 to 8.4: a bracket at course 4.0 (top 4.4); the crest stands on it
    for tx, tz, name in ((-24, -20, "front_tower_-24"), (24, -20, "front_tower_24"), (-24, 20, "rear_spire_-24"), (24, 20, "rear_spire_24")):
        outward = -1 if tz < 0 else 1
        y = round(base + 2 * COURSE, 3)
        f.bracket(y, tx, tz + outward * 4, outward, f"{name}/banner", "stone")
        f.moved[f"{name}/tower_shell_000/p0077"] = [tx, round(y + PLATE + 0.5, 3), tz + outward * 4.8]
    # the keep's banners on its front face (world z = -3) from 3.2 to 7.2: brackets at course 2.8; the crests are found by position
    for px in (-5, 5):
        f.bracket(round(base + COURSE, 3), px, kz - 8, -1, f"central_keep/banner_{px}", "stone")
    for p in parts:
        if p["piece"] == "Tile_1x1" and p["id"].startswith("central_keep/") and abs(p["pos"][1] - (base + 3.6)) < 1e-6:
            f.moved[p["id"]] = [p["pos"][0], round(base + COURSE + PLATE + 0.5, 3), p["pos"][2]]
    # the gatehouse banner on its front face (world z = -24) from 10.6 to 12.6: a bracket at course 10.0, the banner 0.2 lower
    y = round(base + 7 * COURSE, 3)
    f.bracket(y, 0, -24, -1, "main_gatehouse/banner", "stone")
    f.moved["main_gatehouse/p0050"] = [0, round(y + PLATE + 1, 3), -24.4]
    return f


# ------------------------------------------------------------------------------------------------ the document
def rebond(doc, fixes):
    parts = expand(doc)
    f = fixes(parts) if fixes else Fixes()

    # upright bricks on the grid are re-tiled; everything else is kept (moved where the fixes say so)
    courses = collections.defaultdict(dict)    # key_y -> {(cx, cz): (colour, structure)}
    kept = []
    for p in parts:
        yaw = upright_yaw(p["rot"]) if p["piece"] in BRICKS else None
        cells = footprint_cells(p["piece"], p["pos"], yaw) if yaw is not None else None
        if cells is None:
            kept.append(p)
            continue
        structure = p["id"].split("/")[0]
        for c in cells:
            courses[key_y(p["pos"][1])][c] = (p["color"], structure)
    for (y, cx, cz, color, structure) in f.cells:
        if (cx, cz) in courses[key_y(y)]: raise SystemExit(f"extra cell ({cx}, {cz}) at y={y} is already filled")
        courses[key_y(y)][(cx, cz)] = (color, structure)
    for (ky, cx, cz) in f.reserved:
        if (cx, cz) not in courses[ky]: raise SystemExit(f"reserved cell ({cx}, {cz}) at y={ky / 1000} is not a brick cell")
        del courses[ky][(cx, cz)]

    # the surfaces a course can rest on: the ground, the kept pieces' tops and, course by course, the new bricks
    tops = collections.defaultdict(dict)       # key_y -> {(cx, cz): id}

    def add_top(part_id, piece, pos, yaw, top_y):
        w, l = FOOTPRINT[piece]
        if yaw == 90: w, l = l, w
        x0, z0 = pos[0] - w / 2, pos[2] - l / 2
        for x in range(int(math.floor(x0 + 1e-6)), int(math.ceil(x0 + w - 1e-6))):
            for z in range(int(math.floor(z0 + 1e-6)), int(math.ceil(z0 + l - 1e-6))):
                tops[key_y(top_y)][(x, z)] = part_id

    out = []
    for p in kept:
        pos = f.moved.get(p["id"], p["pos"])
        part = dict(id=p["id"], piece=p["piece"], position=list(pos), color=p["color"])
        rot = euler_of(p["rot"])
        if rot != [0, 0, 0]: part["rotation"] = rot
        out.append(part)
        yaw = upright_yaw(p["rot"])
        if yaw is not None: add_top(p["id"], p["piece"], pos, yaw, pos[1] + HEIGHT[p["piece"]])
    for ep in f.parts:
        out.append(dict(ep))
        add_top(ep["id"], ep["piece"], ep["position"], 90 if ep.get("rotation") else 0, ep["position"][1] + HEIGHT[ep["piece"]])
    stats = []
    for ky in sorted(courses):
        cells = courses[ky]
        if not cells: continue
        y = ky / 1000
        top_here = dict(tops[ky])
        if ky == 0:
            for c in cells: top_here[c] = "ground"
        parity = int(round((y - 1.6) / COURSE)) % 2
        course = Course(y, cells, top_here)
        counter = collections.Counter()
        for (piece, x, z, yaw, color, structure, cs) in course.tile(parity):
            counter[structure] += 1
            pid = f"{structure}/y{y:g}_{counter[structure]}"
            part = dict(id=pid, piece=piece, position=[x, y, z], color=color)
            if yaw: part["rotation"] = [0, yaw, 0]
            out.append(part)
            for c in cs: tops[key_y(y + COURSE)][c] = pid
        stats.append((y, len(cells), sum(counter.values())))
    result = dict(format=doc["format"], version=doc["version"], catalog=doc["catalog"], name=doc["name"], units=doc["units"], palette=doc["palette"], parts=out)
    return result, dict(parts_in=len(parts), parts_out=len(out), kept=len(kept), added=len(f.parts), moved=len(f.moved), courses=stats)


def number(v):
    return str(int(round(v))) if abs(v - round(v)) < 1e-9 else f"{v:.6f}".rstrip("0")


def write(doc, path):
    lines = ["{", f'  "format": "{doc["format"]}",', f'  "version": {doc["version"]},', f'  "catalog": "{doc["catalog"]}",',
             f'  "name": "{doc["name"]}",', '  "units": { "gridToUnity": 0.1, "axes": "unity" },',
             '  "palette": { ' + ", ".join(f'"{k}": "{v}"' for k, v in doc["palette"].items()) + " },", '  "parts": [']
    for i, p in enumerate(doc["parts"]):
        s = f'    {{ "id": "{p["id"]}", "piece": "{p["piece"]}", "position": [{", ".join(number(v) for v in p["position"])}]'
        if p.get("rotation"): s += f', "rotation": [{", ".join(str(v) for v in p["rotation"])}]'
        if p.get("color"): s += f', "color": "{p["color"]}"'
        lines.append(s + " }" + ("," if i + 1 < len(doc["parts"]) else ""))
    lines += ["  ]", "}", ""]
    open(path, "w", encoding="utf-8", newline="\n").write("\n".join(lines))


if __name__ == "__main__":
    src, dst = sys.argv[1], sys.argv[2]
    document = json.load(open(src, encoding="utf-8"))
    result, info = rebond(document, citadel_fixes if "emerald" in src.lower() else None)
    write(result, dst)
    print(f"{src} -> {dst}: {info['parts_in']} parts in, {info['parts_out']} out ({info['kept']} kept, {info['added']} added, {info['moved']} moved)")
    for y, n, t in info["courses"]: print(f"  y={y:g}: {n} cells -> {t} bricks")
