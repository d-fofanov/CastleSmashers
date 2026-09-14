"""Generate a single-mesh FBX construction brick, 0.3 metres long.

Geometry/export use the Python standard library; preview needs numpy and Pillow.
"""
from pathlib import Path
import math
from collections import Counter

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'Assets/Models/ConstructorBlock2x3'
OUT.mkdir(parents=True, exist_ok=True)
vertices, faces, groups = [], [], []

def ring(points):
    ids = list(range(len(vertices), len(vertices) + len(points)))
    vertices.extend(points)
    return ids

def triangle(a, b, c):
    faces.append((a, b, c))

def bridge(a, b):
    for i in range(len(a)):
        j = (i + 1) % len(a)
        triangle(a[i], b[i], b[j])
        triangle(a[i], b[j], a[j])

def cap(a, upward):
    for i in range(1, len(a) - 1):
        triangle(a[0], a[i+1], a[i]) if upward else triangle(a[0], a[i], a[i+1])

def rectangle(x, z, y, bevel=.18):
    return ring([(a,y,b) for a,b in [(-x+bevel,-z),(x-bevel,-z),(x,-z+bevel),
                (x,z-bevel),(x-bevel,z),(-x+bevel,z),(-x,z-bevel),(-x,-z+bevel)]])

def circle(x, z, y, radius):
    return ring([(x+radius*math.cos(i*math.tau/24),y,z+radius*math.sin(i*math.tau/24)) for i in range(24)])

groups.append(('Beveled_hollow_body',len(faces)))
outer = [rectangle(7.78,11.78,0), rectangle(7.9,11.9,.12),
         rectangle(7.9,11.9,9.45), rectangle(7.75,11.75,9.6)]
for a,b in zip(outer,outer[1:]): bridge(a,b)
cap(outer[-1],True)
inner_low,inner_high = rectangle(6.4,10.4,0),rectangle(6.4,10.4,8.1)
bridge(inner_high,inner_low)
bridge(inner_low,outer[0])
cap(inner_high,False)

for x in (-4,4):
    for z in (-8,0,8):
        groups.append((f'Stud_{x}_{z}',len(faces)))
        rings = [circle(x,z,9.56,2.4),circle(x,z,11.08,2.4),circle(x,z,11.3,2.18)]
        for a,b in zip(rings,rings[1:]): bridge(a,b)
        cap(rings[0],False)
        cap(rings[-1],True)

for z in (-4,4):
    groups.append((f'Underside_tube_{z}',len(faces)))
    low,high = circle(0,z,0,3.25),circle(0,z,8.12,3.25)
    ilow,ihigh = circle(0,z,0,2.4),circle(0,z,8.12,2.4)
    bridge(low,high)
    bridge(ihigh,ilow)
    bridge(high,ihigh)
    bridge(ilow,low)

# Export explicit triangles: the budget remains valid after engine triangulation.
assert len(faces) <= 1500
edges = Counter(tuple(sorted((a,b))) for f in faces for a,b in zip(f,f[1:]+f[:1]))
assert all(count == 2 for count in edges.values()), 'Open or non-manifold shell'
directed = Counter((a,b) for f in faces for a,b in zip(f,f[1:]+f[:1]))
assert all(directed[(a,b)] == directed[(b,a)] for a,b in directed), 'Inconsistent winding'
def subtract(a,b): return tuple(x-y for x,y in zip(a,b))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def normalized(a):
    length = math.sqrt(dot(a,a))
    assert length > 1e-12
    return tuple(x/length for x in a)

face_normals, adjacent = [], [[] for _ in vertices]
for index,f in enumerate(faces):
    a,b,c = [vertices[i] for i in f]
    u,v = subtract(b,a),subtract(c,a)
    normal = normalized((u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]))
    face_normals.append(normal)
    for corner,vi in enumerate(f):
        u = normalized(subtract(vertices[f[(corner+1)%3]],vertices[vi]))
        v = normalized(subtract(vertices[f[(corner+2)%3]],vertices[vi]))
        adjacent[vi].append((index,math.acos(max(-1,min(1,dot(u,v))))))
normals = []
for index,f in enumerate(faces):
    for vi in f:
        # Keep the planar body and narrow bevel strips flat. Averaging bevel
        # corners across long triangles introduces diagonal shading gradients.
        if index < groups[1][1]:
            normals.append(face_normals[index])
            continue
        contributors = [(face_normals[j],weight) for j,weight in adjacent[vi]
                        if dot(face_normals[index],face_normals[j]) > math.cos(math.radians(35))]
        normals.append(normalized(tuple(sum(n[axis]*w for n,w in contributors) for axis in range(3))))

# FBX stores centimetres (UnitScaleFactor=1), producing a 0.3 m long mesh
# with Unity's default file-unit conversion and identity object scaling.
scaled_vertices = [tuple(c*30/23.8 for c in p) for p in vertices]
def array(values): return ','.join(format(v,'.12g') for v in values)
# Autodesk/Unity's ASCII reader needs definition fields on separate lines.
# Inline ObjectType blocks can silently produce a Transform-only prefab even
# when tolerant third-party readers successfully load the mesh.
fbx = '''; FBX 7.4.0 project file
FBXHeaderExtension: {
    FBXHeaderVersion: 1003
    FBXVersion: 7400
    Creator: "ConstructorBlockGenerator"
}
GlobalSettings: {
    Version: 1000
    Properties70: {
        P: "UpAxis", "int", "Integer", "",1
        P: "UpAxisSign", "int", "Integer", "",1
        P: "FrontAxis", "int", "Integer", "",2
        P: "FrontAxisSign", "int", "Integer", "",1
        P: "CoordAxis", "int", "Integer", "",0
        P: "CoordAxisSign", "int", "Integer", "",1
        P: "OriginalUpAxis", "int", "Integer", "",1
        P: "OriginalUpAxisSign", "int", "Integer", "",1
        P: "UnitScaleFactor", "double", "Number", "",1
        P: "OriginalUnitScaleFactor", "double", "Number", "",1
    }
}
Documents: {
    Count: 1
    Document: 100, "Scene", "Scene" {
        RootNode: 0
    }
}
Definitions: {
    Version: 100
    Count: 2
    ObjectType: "Geometry" {
        Count: 1
    }
    ObjectType: "Model" {
        Count: 1
    }
}
Objects: {
    Geometry: 1000, "Geometry::ConstructorBlock2x3", "Mesh" {
        GeometryVersion: 124
'''
fbx += f'        Vertices: *{len(vertices)*3} {{\n            a: {array(c for p in scaled_vertices for c in p)}\n        }}\n'
fbx += f'        PolygonVertexIndex: *{len(faces)*3} {{\n            a: {array(c for a,b,d in faces for c in (a,b,-d-1))}\n        }}\n'
fbx += '''        LayerElementNormal: 0 {
            Version: 101
            Name: "Normals"
            MappingInformationType: "ByPolygonVertex"
            ReferenceInformationType: "Direct"
'''
fbx += f'            Normals: *{len(normals)*3} {{\n                a: {array(c for n in normals for c in n)}\n            }}\n        }}\n'
fbx += '''        Layer: 0 {
            Version: 100
            LayerElement: {
                Type: "LayerElementNormal"
                TypedIndex: 0
            }
        }
    }
    Model: 2000, "Model::ConstructorBlock2x3", "Mesh" {
        Version: 232
        Properties70: {
            P: "Lcl Translation", "Lcl Translation", "", "A",0,0,0
            P: "Lcl Rotation", "Lcl Rotation", "", "A",0,0,0
            P: "Lcl Scaling", "Lcl Scaling", "", "A",1,1,1
        }
        Shading: T
        Culling: "CullingOff"
    }
}
Connections: {
    C: "OO",1000,2000
    C: "OO",2000,0
}
'''
(OUT/'ConstructorBlock2x3.fbx').write_text(fbx)
(OUT/'README.md').write_text(f'''# Constructor block 2x3

Generic six-stud construction-toy brick with beveled body/stud edges,
an open underside and two hollow support tubes. Stylized dimensions, not a
manufacturing or certified fit model.

- {len(faces)} triangles (limit: 1500), {len(vertices)} geometric vertices.
- Dimensions in Unity units (metres): X=0.199159664, Y=0.142436975, Z=0.3.
- FBX is Y up with centimetre file units; use Unity's default file-unit conversion.
- Pivot at the centre of the bottom face; identity object scale.
- One FBX geometry, one mesh object, no separate stud/body/tube objects.
- Explicit per-corner unit normals, with smooth round sides and sharp rims.
- Uncolored mesh: no vertex colors, material assignments, MTL or textures.
- Assign your own material in Unity. No UVs are supplied.
- Nine closed, consistently oriented mesh shells. Studs and tubes slightly
  overlap the body; this is a visual game asset, not a boolean-unioned print solid.
- Import the FBX into Unity and assign your material. Set Normals to Import.
  No scene or physics
  changes are included.
- Regenerate with Tools/generate_constructor_block.py; numpy and Pillow are
  needed only for the preview. Preview is rendered from the exported geometry.
''')
print(f'Exported {len(vertices)} vertices, {len(faces)} triangles; closed-shell topology and winding verified.')

def preview():
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    # Read the deliverable back, so the preview also checks the exported mesh.
    import re
    exported = (OUT/'ConstructorBlock2x3.fbx').read_text()
    def read_array(name):
        match = re.search(r'\b'+name+r': \*\d+\s*\{\s*a: ([^}]+)', exported)
        return [float(n) for n in match.group(1).split(',')]
    v = np.array(read_array('Vertices')).reshape(-1,3)*23.8/30
    indices = [int(n) for n in read_array('PolygonVertexIndex')]
    fs = [[a,b,-c-1] for a,b,c in zip(indices[::3],indices[1::3],indices[2::3])]
    width,height = 1600,1000
    canvas = np.empty((height,width,3),dtype=np.uint8)
    canvas[:] = (234,238,243)
    depth = np.full((height,width),-np.inf)
    for centre,eye,scale in [((475,510),(1.15,1.1,1.4),24),((1230,520),(-1,-1.5,1.4),17)]:
        forward = np.array(eye,dtype=float); forward /= np.linalg.norm(forward)
        right = np.cross([0,1,0],forward); right /= np.linalg.norm(right)
        up = np.cross(forward,right)
        p = v - [0,5.65,0]
        screen = np.column_stack((p@right*scale+centre[0],-p@up*scale+centre[1],p@forward))
        light = forward*.7 + up*.8 - right*.5; light /= np.linalg.norm(light)
        for f in fs:
            a,b,c = v[f]
            normal = np.cross(b-a,c-a); normal /= np.linalg.norm(normal)
            if normal@forward <= 0: continue
            q = screen[f]
            xmin,xmax = max(0,int(q[:,0].min())),min(width-1,int(q[:,0].max())+1)
            ymin,ymax = max(0,int(q[:,1].min())),min(height-1,int(q[:,1].max())+1)
            xx,yy = np.meshgrid(np.arange(xmin,xmax+1)+.5,np.arange(ymin,ymax+1)+.5)
            den = (q[1,1]-q[2,1])*(q[0,0]-q[2,0])+(q[2,0]-q[1,0])*(q[0,1]-q[2,1])
            if abs(den)<1e-9: continue
            w0 = ((q[1,1]-q[2,1])*(xx-q[2,0])+(q[2,0]-q[1,0])*(yy-q[2,1]))/den
            w1 = ((q[2,1]-q[0,1])*(xx-q[2,0])+(q[0,0]-q[2,0])*(yy-q[2,1]))/den
            w2 = 1-w0-w1
            zz = w0*q[0,2]+w1*q[1,2]+w2*q[2,2]
            region = depth[ymin:ymax+1,xmin:xmax+1]
            mask = (w0>=-1e-8)&(w1>=-1e-8)&(w2>=-1e-8)&(zz>region)
            half = light+forward; half/=np.linalg.norm(half)
            shade = .36+.64*max(0,normal@light)
            color = np.clip(np.array([180,183,188])*shade+max(0,normal@half)**55*65,0,255).astype(np.uint8)
            region[mask] = zz[mask]
            canvas[ymin:ymax+1,xmin:xmax+1][mask] = color
    im = Image.fromarray(canvas)
    draw = ImageDraw.Draw(im)
    font_path = 'C:/Windows/Fonts/segoeui.ttf'
    title = ImageFont.truetype(font_path,42)
    small = ImageFont.truetype(font_path,24)
    draw.text((65,48),'CONSTRUCTOR BLOCK / 2 x 3',font=title,fill='#202b3b')
    draw.text((67,112),f'{len(faces):,} triangles  |  Uncolored mesh  |  Six studs',font=small,fill='#526074')
    draw.text((320,875),'TOP / THREE-QUARTER',font=small,fill='#526074')
    draw.text((1100,820),'HOLLOW UNDERSIDE',font=small,fill='#526074')
    im.resize((1280,800),Image.Resampling.LANCZOS).save(OUT/'ConstructorBlock2x3-preview.png')

if __name__ == '__main__':
    preview()
