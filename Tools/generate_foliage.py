"""Generates the foliage of Assets/Resources/Foliage as brick-assembly documents (Assets/Phys/BRICK_ASSEMBLY.md): five kinds
of grass, five bushes, five rocks, a moss cushion, a fallen log, a stump and two plants, meant to be scattered around the
castle like its trees and shown on their own by the preview demo.

Each model is authored as a height field of plate courses (0.4 grid units) over the stud grid, so that every stud rests on
the stud below and nothing overhangs, and built the way the trees are built: every span of three plate courses aligned to
the brick grid is a course of bricks, the one or two plate courses left above it are plates that carry the stepped
surface, and every course is tiled by the re-bonding tool's course tiler (a running bond whose seams avoid the seams of
the course below) with pieces up to four studs long. The tiler's two-wide runs strand single studs at the steps of a round
outline: a single is merged into the piece beside it wherever the pack has the size, and a single never stands on a single
(a column of singles topples, and two loose plates on the corner of a brick are what a settling stack sheds first): a
column that would end in two single plates is raised a plate to join the bricks beside it, or cut below its top plate.
Every piece is coloured from a hash of its place, lighter where its top is in the open, with the accents of each model on
the small top plates (berries, blossom, seed heads, moss); the cut ends and the bark stripes of the log are materials the
tiler keeps apart. The silhouettes are those of the first, plate-only version of these models (stacks of 1 x 1 and 1 x 2
plates, 100 to 580 a model, that stood asleep but came apart when touched: a thrown stone toppled the reed tufts into
pieces), give or take the raised and cut studs; the succulent's diagonal leaves are two studs wide instead of chains of
single plates touching at their corners.

    python Tools/generate_foliage.py [Assets/Resources/Foliage]
"""
import json
import math
import os
import sys
import uuid
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rebond_castle as rb   # noqa: E402  the course tiler (running bond, support costs)

ROOT = Path(__file__).resolve().parents[1]
BRICK_LENGTHS = [4, 3, 2, 1]   # bricks up to four studs long
PLATE_LENGTHS = [4, 2, 1]      # the pack has no 1 x 3 plate


# ------------------------------------------------------------------------------------------------ the shapes
def dome(x, z, cx, cz, rx, rz, height, power=0.65):
    return height * max(0, 1 - ((x - cx) / rx) ** 2 - ((z - cz) / rz) ** 2) ** power


def shape(kind, x, z):
    """Height in plate courses at the point (x, z) of the stud grid."""
    r, a = math.hypot(x, z), math.atan2(z, x)
    if kind.startswith("grass_"):
        if kind == "grass_meadow":
            centers = [(-2, -1, 6), (0, 0, 9), (2, 1, 7), (-1, 2, 5), (2, -2, 5)]
        elif kind == "grass_tall_reeds":
            centers = [(-2, 0, 14), (0, 1, 17), (2, 0, 12), (0, -2, 15)]
        elif kind == "grass_sedge":
            centers = [(-3, 0, 5), (-1, 0, 10), (1, 0, 10), (3, 0, 5), (0, 2, 6)]
        elif kind == "grass_dry_tussock":
            centers = [(-2, -2, 7), (-2, 2, 6), (2, -2, 6), (2, 2, 8), (0, 0, 11)]
        else:
            centers = [(-3, -1, 4), (-1, -1, 7), (1, 0, 9), (3, 1, 11), (1, 2, 6)]
        return max(dome(x, z, cx, cz, 1.35, 1.45, h, 0.5) for cx, cz, h in centers)
    if kind == "bush_boxwood":
        return dome(x, z, 0, 0, 6, 5, 15, 0.35)
    if kind == "bush_juniper":
        return max(dome(x, z, -3, 0, 4, 3.5, 10), dome(x, z, 1, 0, 4, 4, 18), dome(x, z, 4, 1, 3, 3, 11))
    if kind == "bush_berry":
        return max(dome(x, z, cx, cz, 3.8, 3.8, h) for cx, cz, h in [(-3, -1, 12), (2, -2, 15), (0, 2, 17), (4, 2, 10)])
    if kind == "bush_flowering":
        return max(dome(x, z, cx, cz, 3.6, 3.6, h) for cx, cz, h in [(-3, 0, 10), (0, -2, 13), (3, 0, 11), (0, 3, 9)])
    if kind == "bush_golden_spirea":
        return dome(x, z, 0, 0, 7, 5, 12, 0.8) * (0.85 + 0.15 * math.cos(5 * a))
    if kind == "rock_granite_boulder":
        return dome(x, z, -0.5, 0, 7, 6, 19, 0.6) * (0.94 + 0.06 * math.sin(3 * a))
    if kind == "rock_slate_outcrop":
        return max(dome(x, z, -3, 0, 4, 5, 22, 0.25), dome(x, z, 3, 1, 3, 4, 13, 0.3))
    if kind == "rock_sandstone_stack":
        q = max(abs(x) / 6, abs(z) / 5)
        return max(0, 18 * (1 - q ** 3)) * (0.85 + 0.15 * math.cos(z))
    if kind == "rock_mossy_boulder":
        return max(dome(x, z, -2, 0, 5, 6, 16), dome(x, z, 3, 1, 4, 4, 11))
    if kind == "rock_river_cluster":
        return max(dome(x, z, cx, cz, rx, rz, h) for cx, cz, rx, rz, h in [(-4, -2, 4, 3, 10), (3, -2, 4, 4, 14), (0, 4, 4, 3, 8)])
    if kind == "ground_moss_cushion":
        return max(dome(x, z, cx, cz, rx, rz, h) for cx, cz, rx, rz, h in [(-4, 0, 6, 5, 6), (3, 1, 6, 6, 8), (0, -4, 5, 4, 5)])
    if kind == "wood_fallen_log":
        if abs(x) > 9 or abs(z) > 3.4: return 0
        return 4 + 10 * math.sqrt(max(0, 1 - (z / 3.4) ** 2)) - (1 if abs(x) > 7 else 0)
    if kind == "wood_weathered_stump":
        if r > 6: return 0
        roots = max(0, 1 - r / 6) * (6 + 4 * math.cos(5 * a) ** 2)
        trunk = (17 + 2 * math.sin(3 * a)) if r < 3.6 else 0
        if r < 2.1: trunk = 10   # recessed heartwood, with a supported floor
        return max(roots, trunk)
    if kind == "plant_succulent_rosette":
        # A crown and four leaves along the diagonals, two studs wide, tapering outward over a low web of small leaves.
        best = max(dome(x, z, 0, 0, 2.5, 2.5, 17), max(0, 1 - r / 8) * 3)
        for i in range(4):
            angle = math.pi / 4 + i * math.pi / 2
            u = x * math.cos(angle) + z * math.sin(angle)
            v = -x * math.sin(angle) + z * math.cos(angle)
            if 0.5 < u < 8 and abs(v) < 1.0: best = max(best, (1 - u / 8) * 21)
        return best
    if kind == "plant_fern_clump":
        # Six toothed fronds radiating from a low crown.
        best = dome(x, z, 0, 0, 2.5, 2.5, 12)
        for i in range(6):
            angle = i * math.pi / 3
            u = x * math.cos(angle) + z * math.sin(angle)
            v = -x * math.sin(angle) + z * math.cos(angle)
            width = 0.65 + 1.1 * math.sin(u * 2) ** 2
            if 0 < u < 9 and abs(v) < width:
                best = max(best, (12 - u) * (1 - 0.35 * abs(v) / width))
        return best
    raise ValueError(kind)


def material(kind, cell):
    """The regions whose colour boundaries the tiler must keep: the cut ends and the bark stripes of the log."""
    x, z = cell
    if kind == "wood_fallen_log":
        if abs(x + 0.5) > 7: return "end"
        if z % 3 == 0: return "stripe"
    return "body"


GREEN = {"dark": "#25452B", "mid": "#42743A", "light": "#79A653", "accent": "#A3BD67"}
STONE = {"dark": "#4D555C", "mid": "#747F86", "light": "#A2AAAC", "accent": "#C0C7BF"}
MODELS = [
    ("grass_meadow", "Meadow Grass", GREEN),
    ("grass_tall_reeds", "Tall Reed Grass", {**GREEN, "accent": "#B99A59"}),
    ("grass_sedge", "Blue Sedge", {"dark": "#2A5653", "mid": "#4E8176", "light": "#81ADA0", "accent": "#AEC9B2"}),
    ("grass_dry_tussock", "Dry Tussock Grass", {"dark": "#766035", "mid": "#AB8A49", "light": "#D1B568", "accent": "#E9D598"}),
    ("grass_windswept", "Windswept Grass", {**GREEN, "light": "#92B94E"}),
    ("bush_boxwood", "Round Boxwood Bush", GREEN),
    ("bush_juniper", "Juniper Bush", {"dark": "#233F3D", "mid": "#3B655E", "light": "#628B7E", "accent": "#8EA8A2"}),
    ("bush_berry", "Red Berry Bush", {**GREEN, "accent": "#BE3945"}),
    ("bush_flowering", "Flowering Heather Bush", {**GREEN, "accent": "#CB8CBD"}),
    ("bush_golden_spirea", "Golden Spirea Bush", {"dark": "#576932", "mid": "#8F9F3F", "light": "#C5CC61", "accent": "#E2D97F"}),
    ("rock_granite_boulder", "Granite Boulder", STONE),
    ("rock_slate_outcrop", "Slate Outcrop", {"dark": "#343B4D", "mid": "#555F78", "light": "#78859B", "accent": "#9CA7BA"}),
    ("rock_sandstone_stack", "Layered Sandstone", {"dark": "#956144", "mid": "#B78156", "light": "#D5A273", "accent": "#E7C196"}),
    ("rock_mossy_boulder", "Mossy Boulder", {**STONE, "accent": "#618345"}),
    ("rock_river_cluster", "River Stone Cluster", {"dark": "#59605F", "mid": "#87928B", "light": "#ADB8AB", "accent": "#D1D3BB"}),
    ("ground_moss_cushion", "Moss Cushion", {"dark": "#36522A", "mid": "#608337", "light": "#8EAD4C", "accent": "#B9C96E"}),
    ("wood_fallen_log", "Fallen Log", {"dark": "#453025", "mid": "#694837", "light": "#96704A", "accent": "#C5A576"}),
    ("wood_weathered_stump", "Weathered Stump", {"dark": "#43352C", "mid": "#715440", "light": "#98795A", "accent": "#C0A17C"}),
    ("plant_succulent_rosette", "Succulent Rosette", {"dark": "#345E53", "mid": "#588775", "light": "#85AD8D", "accent": "#B7C598"}),
    ("plant_fern_clump", "Fern Clump", {"dark": "#234D32", "mid": "#3C7F43", "light": "#76A451", "accent": "#A5C172"}),
]


def heights(kind):
    """The height field: cell -> plate courses (the shape sampled at the cell centres, a stud of at least 0.8 courses kept)."""
    raw = {(x, z): shape(kind, x + 0.5, z + 0.5) for x in range(-12, 12) for z in range(-12, 12)}
    return {c: max(1, int(h)) for c, h in raw.items() if h >= 0.8}


# ------------------------------------------------------------------------------------------------ tiling
def tile(cells, tops, parity, plate):
    """Tiles one course ({cell: material}) against the surfaces below ({cell: id}) with the re-bonding tool's course tiler:
    [(piece, x, z, yaw, material, cells)]."""
    rb.LENGTHS = PLATE_LENGTHS if plate else BRICK_LENGTHS
    course = rb.Course(0.0, {c: (m, m) for c, m in cells.items()}, {c: tops.get(c, "ground") for c in cells})
    return [(piece.replace("Brick_", "Plate_") if plate else piece, x, z, yaw, m, cs) for (piece, x, z, yaw, m, _s, cs) in course.tile(parity)]


def piece_of(cells, m, plate):
    """The catalog piece covering a rectangle of cells: (piece, x, z, yaw, material, cells), or None if the pack has no such size."""
    xs, zs = [c[0] for c in cells], [c[1] for c in cells]
    w, l = max(xs) - min(xs) + 1, max(zs) - min(zs) + 1
    size = (min(w, l), max(w, l))
    if size not in ((1, 1), (1, 2), (1, 4), (2, 2), (2, 4)) and (plate or size not in ((1, 3), (2, 3))): return None
    return (("Plate_" if plate else "Brick_") + f"{size[0]}x{size[1]}", (min(xs) + max(xs) + 1) / 2, (min(zs) + max(zs) + 1) / 2, 90 if w > l else 0, m, sorted(cells))


def join(p, c, n, plate):
    """Pieces replacing the piece p and the single at cell c, next to p's cell n: the line of p through n along the direction
    c - n gets c (a line of five bricks becomes three and two), the lines of p before and after it stay; None when the pack
    cannot cover one of them without a single."""
    m, cells = p[4], p[5]
    axis = 0 if c[1] == n[1] else 1   # the direction of c from n
    other = 1 - axis
    lines = {}
    for cell in cells: lines.setdefault(cell[other], []).append(cell)
    out = []
    line = sorted(lines[n[other]] + [c])
    if len(line) == 5:
        if plate: return None
        out += [line[:3], line[3:]]
    elif piece_of(line, m, plate) is None or len(line) < 2: return None
    else: out.append(line)
    for side in (lambda k: k < n[other], lambda k: k > n[other]):
        rest = [cell for k, ln in lines.items() if side(k) for cell in ln]
        if not rest: continue
        if len(rest) < 2 or piece_of(rest, m, plate) is None: return None
        out.append(rest)
    return [piece_of(cells, m, plate) for cells in out]


def repair(pieces, plate):
    """Merges every single-stud piece it can into a neighbouring piece of its material (see join): the tiler's two-wide runs
    strand single cells at the steps of a round outline, and a single is a weak point."""
    pieces = list(pieces)
    owner = {c: i for i, p in enumerate(pieces) for c in p[5]}
    changed = True
    while changed:
        changed = False
        for i, p in enumerate(pieces):
            if p is None or len(p[5]) != 1: continue
            c = p[5][0]
            for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
                j = owner.get(n)
                if j is None or j == i or pieces[j][4] != p[4]: continue
                new = join(pieces[j], c, n, plate)
                if new is None: continue
                pieces[i] = pieces[j] = None
                for q in new:
                    pieces.append(q)
                    for cell in q[5]: owner[cell] = len(pieces) - 1
                changed = True
                break
    return [p for p in pieces if p is not None]


def build(kind):
    """The pieces of a model: [(piece, x, y, z, yaw, courses tall, material, cells)] with y in plate courses, and the height field
    as built: a single-stud piece the tiler leaves is merged into a neighbour where the pack allows, and a single never stands on
    a single (a column of singles topples, and two loose plates on a corner are what a settling stack sheds first): a column that
    would end in two single plates is raised to the brick level next to it, where it joins the bricks, or cut below its top
    plate; a single brick never gets another."""
    H = heights(kind)
    parts, tops, singles, changed = [], {}, {}, 0   # tops: cell -> id of the surface there; singles: courses of single-stud pieces ending there
    k = 0
    while 3 * k < max(H.values()):
        level, level_tops, level_singles, raised = [], dict(tops), dict(singles), False
        for (plate, y) in [(False, 3 * k), (True, 3 * k), (True, 3 * k + 1), (True, 3 * k + 2)]:
            t = 1 if plate else 3
            while not raised:
                cells = {c: material(kind, c) for c, h in H.items() if (y < h < 3 * k + 3 if plate else h >= 3 * k + 3)}
                if not cells: break
                pieces = repair(tile(cells, level_tops, y % 2, plate), plate)
                weak = [cs[0] for (_p, _x, _z, _yaw, _m, cs) in pieces if len(cs) == 1 and level_singles.get(cs[0], 0) > 0]
                if weak:
                    for c in weak:
                        if plate and any(H.get(n, 0) >= 3 * k + 3 for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1))):
                            H[c] = 3 * k + 3   # flush with the bricks beside it: the level is built again with the cell among them
                            raised = True
                        else: H[c] = y
                    changed += len(weak)
                    continue
                for (piece, x, z, yaw, m, cs) in pieces:
                    pid = f"t{len(parts) + len(level)}"
                    level.append((piece, x, y, z, yaw, t, m, cs))
                    for c in cs:
                        level_tops[c] = pid
                        level_singles[c] = level_singles.get(c, 0) + t if len(cs) == 1 else 0
                break
        if raised: continue
        parts += level
        tops, singles = level_tops, level_singles
        k += 1
    return parts, H, changed


# ------------------------------------------------------------------------------------------------ colours
def colour(kind, H, part):
    piece, x, y, z, yaw, t, m, cells = part
    x0, z0 = min(c[0] for c in cells), min(c[1] for c in cells)
    w, l = max(c[0] for c in cells) - x0 + 1, max(c[1] for c in cells) - z0 + 1
    top = any(H[c] == y + t for c in cells)   # a piece with its top in the open (the surface is what shows)
    h = ((x0 * 73856093) ^ (z0 * 19349663) ^ (y * 83492791)) & 0xFFFFFFFF
    noise = ((((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF) >> 8) % 17   # a hash of the place, 0 .. 16
    if kind == "wood_fallen_log":
        if m == "end": return "accent" if (int(abs(z0 + l / 2)) + y // 3) % 3 else "light"
        return "dark" if m == "stripe" else "mid"
    if kind == "wood_weathered_stump": return "accent" if top and math.hypot(x0 + w / 2, z0 + l / 2) < 3 else ("dark" if noise < 5 else "mid")
    if kind == "rock_sandstone_stack": return ["dark", "mid", "light", "mid", "accent"][y // 2 % 5]   # the strata
    if kind == "rock_mossy_boulder" and top and t == 1 and noise < 11: return "accent"                          # moss on the top plates
    if top and t == 1 and len(cells) <= 2 and kind in ("bush_berry", "bush_flowering") and noise < 5: return "accent"   # berries and blossom: small plates
    if top and t == 1 and kind == "grass_tall_reeds" and noise < 12: return "accent"                                 # seed heads
    if top: return "light" if noise < 12 else "mid"
    return "dark" if noise < 4 else "mid"


# ------------------------------------------------------------------------------------------------ checks and output
def validate(doc, count):
    """The document as written: the format, the pieces of the pack, every part on the plate grid, no two overlapping, everything
    resting on the course below or the ground."""
    assert doc["format"] == "brick-assembly" and doc["version"] == 1
    assert doc["catalog"] == "generic-construction-27-v1"
    assert doc["units"] == {"gridToUnity": 0.1, "axes": "unity"}
    assert len(doc["parts"]) == count
    assert len({p["id"] for p in doc["parts"]}) == count
    occupied = set()
    for p in doc["parts"]:
        assert "/" not in p["id"] and p["color"] in doc["palette"]
        assert (ROOT / "Assets/Models/construction_pieces" / (p["piece"] + ".fbx")).is_file()
        assert all(math.isfinite(v) for v in p["position"])
        w, l = map(int, p["piece"].split("_")[1].split("x"))
        if p.get("rotation") == [0, 90, 0]: w, l = l, w
        px, py, pz = p["position"]
        x, z, y = round(px - w / 2), round(pz - l / 2), round(py / 0.4)
        assert abs(py - y * 0.4) < 1e-8 and abs(px - w / 2 - x) < 1e-8 and abs(pz - l / 2 - z) < 1e-8
        t = 3 if p["piece"].startswith("Brick_") else 1
        cells = {(x + i, y + j, z + k) for i in range(w) for j in range(t) for k in range(l)}
        assert not cells & occupied, f"Overlapping body: {p['id']}"
        occupied |= cells
    assert min(y for x, y, z in occupied) == 0
    assert all(y == 0 or (x, y - 1, z) in occupied for x, y, z in occupied), "Unsupported cell"


def document(kind, name, palette, parts, H):
    out = []
    for i, part in enumerate(sorted(parts, key=lambda p: (p[2], p[1], p[3]))):
        piece, x, y, z, yaw, t, m, cells = part
        q = {"id": f"p{i + 1:04}", "piece": piece, "position": [x, round(y * 0.4, 4), z], "color": colour(kind, H, part)}
        if yaw: q["rotation"] = [0, yaw, 0]
        out.append(q)
    return {"format": "brick-assembly", "version": 1, "catalog": "generic-construction-27-v1", "name": name, "units": {"gridToUnity": 0.1, "axes": "unity"}, "palette": palette, "parts": out}


def write(doc, path):
    """One explicit part per line, matching the other resource assemblies."""
    header = json.dumps({k: v for k, v in doc.items() if k != "parts"}, indent=2)
    text = header[:-2] + ',\n  "parts": [\n' + ',\n'.join('    ' + json.dumps(p) for p in doc["parts"]) + '\n  ]\n}\n'
    path.write_text(text, encoding="utf-8")


def meta(path, key, folder=False):
    """A .meta with a stable GUID, unless Unity wrote one already."""
    path = Path(str(path) + ".meta")
    if path.exists(): return
    importer = "folderAsset: yes\nDefaultImporter:" if folder else "TextScriptImporter:"
    path.write_text(f"fileFormatVersion: 2\nguid: {uuid.uuid5(uuid.NAMESPACE_URL, 'EngineGPU/Foliage' + key).hex}\n{importer}\n  externalObjects: {{}}\n  userData: \n  assetBundleName: \n  assetBundleVariant: \n", encoding="utf-8")


def main(folder):
    folder.mkdir(parents=True, exist_ok=True)
    meta(folder, "", folder=True)
    for key, name, palette in MODELS:
        parts, H, adjusted = build(key)
        doc = document(key, name, palette, parts, H)
        validate(doc, len(parts))
        path = folder / (key + ".json")
        write(doc, path)
        validate(json.loads(path.read_text(encoding="utf-8")), len(parts))
        meta(path, "/" + key)
        mix = {}
        for p in doc["parts"]: mix[p["piece"]] = mix.get(p["piece"], 0) + 1
        top = ", ".join(f"{n} {piece}" for piece, n in sorted(mix.items(), key=lambda kv: -kv[1])[:4])
        xs, zs = [c[0] for c in H], [c[1] for c in H]
        print(f"{key:24s} {len(parts):4d} pieces, {len(H):3d} studs, {max(H.values()) * 0.4:4.1f} tall, {max(xs) - min(xs) + 1:2d} x {max(zs) - min(zs) + 1:2d}; "
              f"{adjusted} studs raised or cut a plate: {top}")


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "Assets/Resources/Foliage")
