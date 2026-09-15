"""Generic construction pieces: 27 low-poly FBX models.

Every piece is assembled from explicit vertex rings and closed shells (studs, tubes and posts
overlap the hollow body) instead of Boolean unions and a bevel modifier. Edges get a single
chamfer whose corner normals blend the faces it joins, so they still shade as rounded while the
large faces stay flat. Triangles, normals and UVs are all explicit; nothing is left to modifiers.

    blender -b -t 4 --python generate_models.py

PIECES_OUT=<dir> writes the pack somewhere else (for checks) instead of next to this script.
"""
import bpy, bmesh, math, json, os
from collections import Counter, defaultdict
from mathutils import Vector

OUT = os.environ.get('PIECES_OUT') or os.path.dirname(os.path.abspath(__file__))
P = .1       # stud pitch
H = .12      # brick body height (plates and tiles: .04)
SH = .0225   # stud height
WALL = .015  # wall and ceiling thickness of the hollow underside
C = .0015    # edge chamfer: the pack's rounded-edge width
STUD_R, TUBE_R, TUBE_RI, POST_R = .03, .04, .03, .012
N_STUD, N_TUBE, N_POST, N_CYL = 16, 12, 8, 24   # segments of the round parts
UV = 1 / P   # UV0 tiles once per stud pitch
BUDGET = 2500
UP, DOWN = Vector((0, 0, 1)), Vector((0, 0, -1))
SIDES = [Vector(n) for n in [(1, 0, 0)] * 2 + [(0, 1, 0)] * 2 + [(-1, 0, 0)] * 2 + [(0, -1, 0)] * 2]


class Geo:
    """Triangle soup with a normal and a UV per corner."""
    def __init__(self): self.verts, self.tris = [], []
    def vertex(self, p): self.verts.append(Vector(p)); return len(self.verts) - 1
    def tri(self, ids, normals, uvs): self.tris.append((tuple(ids), tuple(Vector(n).normalized() for n in normals), tuple(uvs)))


class Ring:
    """A closed vertex loop, counter-clockwise seen from +Z, with a shading normal per vertex and,
    for round parts, a cylindrical UV per vertex; u_wrap closes the seam of the last segment."""
    def __init__(self, geo, points, normals, uvs=None, u_wrap=None):
        self.geo, self.ids, self.n, self.uv, self.u_wrap = geo, [geo.vertex(p) for p in points], list(normals), uvs, u_wrap
    def __len__(self): return len(self.ids)


def box_uv(p, n):
    """Planar projection along the dominant axis of n, ties preferring Z then X."""
    a = abs(n.x), abs(n.y), abs(n.z)
    if a[2] >= max(a[0], a[1]) - 1e-6: return (p.x * UV, p.y * UV)
    if a[0] >= a[1] - 1e-6: return (p.y * UV, p.z * UV)
    return (p.x * UV, p.z * UV)

def face_normal(verts, ids):
    a, b, c = (verts[i] for i in ids[:3])
    return (b - a).cross(c - a).normalized()

def polygon(geo, ids, normals, uv_normal):
    """Fan-triangulated convex polygon; box UVs projected along uv_normal."""
    uvs = [box_uv(geo.verts[i], uv_normal) for i in ids]
    for k in range(1, len(ids) - 1):
        geo.tri((ids[0], ids[k], ids[k + 1]), (normals[0], normals[k], normals[k + 1]), (uvs[0], uvs[k], uvs[k + 1]))

def bridge(a, b, normal=None, cyl=False):
    """Quads between two rings of equal length. (lower, upper) faces outward, (upper, lower) inward;
    for concentric rings at one height (inner, outer) faces down and (outer, inner) faces up.
    Corner normals come from the rings (smooth strips) unless a flat normal is given."""
    geo = a.geo
    for i in range(len(a)):
        j = (i + 1) % len(a)
        ids = (a.ids[i], a.ids[j], b.ids[j], b.ids[i])
        normals = (a.n[i], a.n[j], b.n[j], b.n[i]) if normal is None else (normal,) * 4
        if cyl:
            uvs = (a.uv[i], (a.u_wrap, a.uv[0][1]) if j == 0 else a.uv[j], (b.u_wrap, b.uv[0][1]) if j == 0 else b.uv[j], b.uv[i])
        else:
            uvs = tuple(box_uv(geo.verts[k], face_normal(geo.verts, ids)) for k in ids)
        geo.tri(ids[:3], normals[:3], uvs[:3])
        geo.tri((ids[0], ids[2], ids[3]), (normals[0], normals[2], normals[3]), (uvs[0], uvs[2], uvs[3]))

def cap(ring, up):
    """Flat fan over a ring, facing +Z or -Z."""
    ids = ring.ids if up else [ring.ids[0]] + ring.ids[:0:-1]
    polygon(ring.geo, ids, [UP if up else DOWN] * len(ids), UP if up else DOWN)

def rect_ring(geo, a, b, z, normals):
    """Octagon: a rectangle of half-extents a, b with its corners cut by C (the vertical edge chamfer)."""
    pts = [(a, -b + C), (a, b - C), (a - C, b), (-a + C, b), (-a, b - C), (-a, -b + C), (-a + C, -b), (a - C, -b)]
    return Ring(geo, [(x, y, z) for x, y in pts], normals)

def circle_ring(geo, cx, cy, z, r, n, normal, u_step):
    """normal: 'out' (radial), 'in' (-radial) or a fixed vector; u_step: UV advance per segment."""
    pts, normals, uvs = [], [], []
    for i in range(n):
        d = Vector((math.cos(i * math.tau / n), math.sin(i * math.tau / n), 0))
        pts.append((cx + r * d.x, cy + r * d.y, z))
        normals.append(d if normal == 'out' else -d if normal == 'in' else Vector(normal))
        uvs.append((i * u_step, z * UV))
    return Ring(geo, pts, normals, uvs, n * u_step)


def hollow_body(geo, w, l, h):
    a, b = w * P / 2, l * P / 2
    r0 = rect_ring(geo, a - C, b - C, 0, [DOWN] * 8)
    r1 = rect_ring(geo, a, b, C, SIDES)
    r2 = rect_ring(geo, a, b, h - C, SIDES)
    r3 = rect_ring(geo, a - C, b - C, h, [UP] * 8)
    bridge(r0, r1); bridge(r1, r2); bridge(r2, r3); cap(r3, True)
    c0 = rect_ring(geo, a - WALL, b - WALL, 0, [-n for n in SIDES])
    c1 = rect_ring(geo, a - WALL, b - WALL, h - WALL, [-n for n in SIDES])
    bridge(c0, r0, normal=DOWN)   # bottom rim
    bridge(c1, c0)                # cavity walls face inward
    cap(c1, False)                # ceiling faces down into the cavity

def stud(geo, cx, cy, z0):
    """Closed shell standing on the face at height z0, its base cap hidden inside the body."""
    u = math.tau * STUD_R / N_STUD * UV
    base = circle_ring(geo, cx, cy, z0 - .002, STUD_R, N_STUD, 'out', u)
    rim = circle_ring(geo, cx, cy, z0 + SH - C, STUD_R, N_STUD, 'out', u)
    top = circle_ring(geo, cx, cy, z0 + SH, STUD_R - C, N_STUD, UP, u)
    cap(base, False); bridge(base, rim, cyl=True); bridge(rim, top, cyl=True); cap(top, True)

def tube(geo, cx, cy, zt):
    u = math.tau * TUBE_R / N_TUBE * UV
    ob, ot = (circle_ring(geo, cx, cy, z, TUBE_R, N_TUBE, 'out', u) for z in (0, zt))
    ib, it = (circle_ring(geo, cx, cy, z, TUBE_RI, N_TUBE, 'in', u) for z in (0, zt))
    bridge(ob, ot, cyl=True); bridge(it, ib, cyl=True); bridge(ib, ob, normal=DOWN); bridge(ot, it, normal=UP)

def post(geo, cx, cy, zt):
    u = math.tau * POST_R / N_POST * UV
    b, t = (circle_ring(geo, cx, cy, z, POST_R, N_POST, 'out', u) for z in (0, zt))
    cap(b, False); bridge(b, t, cyl=True); cap(t, True)

def brick(w, l, h):
    geo = Geo(); hollow_body(geo, w, l, h)
    for x in range(w):
        for y in range(l): stud(geo, (x - (w - 1) / 2) * P, (y - (l - 1) / 2) * P, h)
    # Support tubes on two-wide pieces, solid posts on single rows; both end inside the ceiling.
    zt = h - WALL + .003
    for j in range(l - 1):
        y = (j - (l - 2) / 2) * P
        if w == 2: tube(geo, 0, y, zt)
        elif l > 1: post(geo, 0, y, zt)
    return geo

def cylinder(r, h):
    geo = Geo(); u = math.tau * r / N_CYL * UV
    r0 = circle_ring(geo, 0, 0, 0, r - C, N_CYL, DOWN, u)
    r1 = circle_ring(geo, 0, 0, C, r, N_CYL, 'out', u)
    r2 = circle_ring(geo, 0, 0, h - C, r, N_CYL, 'out', u)
    r3 = circle_ring(geo, 0, 0, h, r - C, N_CYL, UP, u)
    cap(r0, False); bridge(r0, r1, cyl=True); bridge(r1, r2, cyl=True); bridge(r2, r3, cyl=True); cap(r3, True)
    return geo


def prism_solid(profile, w):
    """Convex polyhedron: a (y, z) profile, counter-clockwise seen from +X, extruded over x in [-w/2, w/2].
    Faces are vertex index lists, counter-clockwise seen from outside."""
    n = len(profile)
    verts = [Vector((x, y, z)) for x in (-w / 2, w / 2) for y, z in profile]
    faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))] + [[i, (i + 1) % n, (i + 1) % n + n, i + n] for i in range(n)]
    return verts, faces

def box_solid(w, l, h): return prism_solid([(-l / 2, 0), (l / 2, 0), (l / 2, h), (-l / 2, h)], w)

def chamfered(solid):
    """Single-segment chamfer of a convex polyhedron: each face shrinks by C inside its plane, quads
    join the shrunk faces along the edges and a polygon closes each vertex. Every new vertex keeps
    the normal of its face, so the strips blend between the faces they connect."""
    verts, faces = solid
    geo = Geo()
    normals = [face_normal(verts, f) for f in faces]
    corner, owner = {}, {}
    for fi, f in enumerate(faces):
        for k, v in enumerate(f):
            p, q, r = verts[f[k - 1]], verts[v], verts[f[(k + 1) % len(f)]]
            i1, i2 = normals[fi].cross((q - p).normalized()), normals[fi].cross((r - q).normalized())   # inward edge normals
            corner[fi, v] = geo.vertex(q + (i1 + i2) * (C / (1 + i1.dot(i2))))
            owner[v, f[(k + 1) % len(f)]] = fi
    for fi, f in enumerate(faces): polygon(geo, [corner[fi, v] for v in f], [normals[fi]] * len(f), normals[fi])
    for (u, v), f1 in owner.items():
        if u < v:
            f2 = owner[v, u]
            polygon(geo, [corner[f1, u], corner[f2, u], corner[f2, v], corner[f1, v]], [normals[f1], normals[f2], normals[f2], normals[f1]], normals[f1] + normals[f2])
    for v in range(len(verts)):
        around = [next(fi for fi, f in enumerate(faces) if v in f)]
        while True:
            f = faces[around[-1]]; nxt = owner[f[(f.index(v) + 1) % len(f)], v]
            if nxt == around[0]: break
            around.append(nxt)
        around.reverse()
        polygon(geo, [corner[fi, v] for fi in around], [normals[fi] for fi in around], sum((normals[fi] for fi in around), Vector()))
    return geo


def check(name, geo):
    """Closed, consistently wound shells with positive volume and no degenerate triangles."""
    edges = Counter((a, b) for ids, _, _ in geo.tris for a, b in zip(ids, ids[1:] + ids[:1]))
    assert all(n == 1 for n in edges.values()) and all((b, a) in edges for a, b in edges), name + ': open or non-manifold shell'
    parent = list(range(len(geo.verts)))
    def find(i):
        while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for ids, _, _ in geo.tris: parent[find(ids[1])] = find(ids[0]); parent[find(ids[2])] = find(ids[0])
    volume = defaultdict(float)
    for ids, _, _ in geo.tris:
        a, b, c = (geo.verts[i] for i in ids)
        assert (b - a).cross(c - a).length > 1e-12, name + ': degenerate triangle'
        volume[find(ids[0])] += a.dot(b.cross(c)) / 6
    assert all(v > 0 for v in volume.values()), name + ': inverted shell'
    assert len(geo.tris) <= BUDGET, (name, len(geo.tris))
    return len(volume)


bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'; scene.unit_settings.scale_length = 1
def material(name, color, roughness):
    m = bpy.data.materials.new(name); m.diffuse_color = color
    if m.node_tree is None: m.use_nodes = True   # Blender 5 materials are node-based from the start
    bs = m.node_tree.nodes['Principled BSDF']; bs.inputs['Base Color'].default_value = color; bs.inputs['Roughness'].default_value = roughness
    return m
mat = material('Neutral_Plastic', (.46, .5, .53, 1), .3)

def active(o):
    bpy.ops.object.select_all(action='DESELECT'); o.select_set(True); bpy.context.view_layer.objects.active = o

items, stats = [], []
def finish(name, geo):
    shells = check(name, geo)
    me = bpy.data.meshes.new(name + '_Mesh')
    me.from_pydata([tuple(v) for v in geo.verts], [], [ids for ids, _, _ in geo.tris])
    assert not me.validate(verbose=True), name
    assert all(me.loops[3 * i + k].vertex_index == ids[k] for i, (ids, _, _) in enumerate(geo.tris) for k in range(3)), name
    me.shade_smooth()
    uv = me.uv_layers.new(name='UVMap')
    for i, (_, _, uvs) in enumerate(geo.tris):
        for k in range(3): uv.data[3 * i + k].uv = uvs[k]
    me.normals_split_custom_set([n for _, normals, _ in geo.tris for n in normals])
    worst = max(math.degrees(me.corner_normals[3 * i + k].vector.angle(normals[k])) for i, (_, normals, _) in enumerate(geo.tris) for k in range(3))
    assert worst < .5, (name, worst)
    me.materials.append(mat)
    o = bpy.data.objects.new(name, me); scene.collection.objects.link(o); active(o)
    dims = list(o.dimensions)
    bpy.ops.export_scene.fbx(filepath=OUT + '/' + name + '.fbx', use_selection=True, object_types={'MESH'}, apply_unit_scale=True, apply_scale_options='FBX_SCALE_UNITS', axis_forward='-Z', axis_up='Y', bake_space_transform=True, use_mesh_modifiers=False, mesh_smooth_type='OFF', add_leaf_bones=False, bake_anim=False, path_mode='AUTO')
    stats.append(dict(name=name, dimensions_xyz_m=dims, triangles=len(me.polygons), vertices=len(me.vertices), shells=shells))
    items.append(o); o.hide_render = True; o.hide_set(True)

for w, l in [(1, 1), (1, 2), (1, 3), (1, 4), (1, 6), (1, 8), (2, 2), (2, 3), (2, 4), (2, 6), (2, 8)]: finish(f'Brick_{w}x{l}', brick(w, l, H))
for w, l in [(1, 1), (1, 2), (1, 4), (2, 2), (2, 4)]: finish(f'Plate_{w}x{l}', brick(w, l, .04))
for w, l in [(1, 1), (1, 2), (2, 2), (2, 4)]: finish(f'Tile_{w}x{l}', chamfered(box_solid(w * P, l * P, .04)))
# The slopes rise toward -Y here: the FBX export (-Z forward, Y up) and Unity's import turn +Y into Unity -Z, and the
# brick-assembly catalog (Assets/Phys/BRICK_ASSEMBLY.md) has Ramp and Wedge rising toward +Z at zero rotation.
finish('Ramp', chamfered(prism_solid([(-.1, 0), (.1, 0), (.1, .02), (-.1, .12)], .2)))
finish('Roof_Prism', chamfered(prism_solid([(-.1, 0), (.1, 0), (0, .12)], .2)))
finish('Wedge', chamfered(prism_solid([(-.15, 0), (.15, 0), (.15, .015), (-.15, .08)], .2)))
finish('Plain_Cube', chamfered(box_solid(.1, .1, .1)))
finish('Plain_Rectangular_Block', chamfered(box_solid(.2, .3, .12)))
finish('Plain_Cylinder', cylinder(.075, .15))
finish('Plain_Triangular_Prism', chamfered(prism_solid([(-.1, 0), (.1, 0), (0, .12)], .2)))

# Re-import every delivered FBX: one mesh, the same size and triangle count, closed edges and the custom normals intact.
for entry in stats:
    before = set(bpy.data.objects)
    bpy.ops.import_scene.fbx(filepath=OUT + '/' + entry['name'] + '.fbx', use_custom_normals=True)
    new = list(set(bpy.data.objects) - before); meshes = [o for o in new if o.type == 'MESH']; assert len(meshes) == 1
    me = meshes[0].data
    dims = sorted(meshes[0].dimensions); expected = sorted(entry['dimensions_xyz_m'])
    assert max(abs(a - b) for a, b in zip(dims, expected)) < 1e-5, (entry['name'], dims, expected)
    assert len(me.polygons) == entry['triangles'] and len(me.uv_layers) == 1 and len(me.materials) == 1, entry['name']
    bm = bmesh.new(); bm.from_mesh(me); entry['non_manifold_edges'] = sum(not e.is_manifold for e in bm.edges); bm.free()
    assert entry['non_manifold_edges'] == 0, entry['name']
    # Flat corners on the big faces, blended corners on the chamfer strips: both must survive the round trip.
    flat = sum(me.corner_normals[l].vector.angle(me.polygons[l // 3].normal) < math.radians(.5) for l in range(len(me.loops)))
    assert 0 < flat < len(me.loops), (entry['name'], flat, len(me.loops))
    entry['fbx_roundtrip_verified'] = True
    for o in new: bpy.data.objects.remove(o, do_unlink=True)
json.dump(stats, open(OUT + '/mesh_manifest.json', 'w'), indent=2)

# Render an actual-mesh contact sheet with a fixed orthographic view.
for idx, o in enumerate(items):
    o.hide_render = False; o.hide_set(False)
    row = idx // 6; col = idx % 6
    o.rotation_euler[2] = math.radians(-35)
    o.location = (col * .85, -row * .72, 0)
    # labels lie flat on the ground, readable from camera
    bpy.ops.object.text_add(location=(col * .85, -row * .72 - .28, .003))
    t = bpy.context.object; t.data.body = o.name.replace('_', ' '); t.data.align_x = 'CENTER'; t.data.size = .045
    t.data.materials.append(bpy.data.materials.get('Label') or material('Label', (.02, .03, .04, 1), .6))
bpy.ops.mesh.primitive_plane_add(size=200, location=(0, 0, -.006)); ground = bpy.context.object
ground.data.materials.append(material('Background', (.88, .9, .92, 1), .6))
bpy.ops.object.camera_add(location=(2.12, -5.5, 7.5)); cam = bpy.context.object; target = Vector((2.12, -1.5, 0)); cam.rotation_euler = (target - cam.location).to_track_quat('-Z', 'Y').to_euler(); cam.data.type = 'ORTHO'; cam.data.ortho_scale = 5.6; scene.camera = cam
bpy.ops.object.light_add(type='AREA', location=(1, -1, 6)); bpy.context.object.data.energy = 1100; bpy.context.object.data.shape = 'DISK'; bpy.context.object.data.size = 5
scene.world.color = (.4, .4, .4); scene.render.engine = 'CYCLES'; scene.cycles.samples = 24
scene.render.resolution_x = 1800; scene.render.resolution_y = 1450; scene.render.resolution_percentage = 100
scene.render.filepath = OUT + '/preview.png'; bpy.ops.render.render(write_still=True)
print('DONE: %d FBX exports and roundtrip checks, %d triangles in total' % (len(stats), sum(s['triangles'] for s in stats)))
