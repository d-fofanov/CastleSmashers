"""Generate uncolored single-mesh toy figure and arrow FBX assets."""
from pathlib import Path
import math
import re
import uuid
from collections import Counter

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = (ROOT/'Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx').read_text()

def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def cross(a,b): return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def unit(v):
    l=math.sqrt(dot(v,v))
    return tuple(x/l for x in v)

class Mesh:
    def __init__(self): self.v=[]; self.f=[]; self.smooth=[]
    def ring(self,points):
        ids=list(range(len(self.v),len(self.v)+len(points))); self.v.extend(points); return ids
    def tri(self,a,b,c,smooth=False): self.f.append((a,b,c)); self.smooth.append(smooth)
    def bridge(self,a,b,smooth=False):
        for i in range(len(a)):
            j=(i+1)%len(a)
            self.tri(a[i],b[i],b[j],smooth); self.tri(a[i],b[j],a[j],smooth)
    def cap(self,r,up):
        for i in range(1,len(r)-1):
            self.tri(r[0],r[i+1],r[i]) if up else self.tri(r[0],r[i],r[i+1])
    def box(self,x,z,y0,y1,w0,w1,depth,bevel=.003):
        def r(y,w,d,c):
            return self.ring([(x+a,y,z+b) for a,b in [(-w+c,-d),(w-c,-d),(w,-d+c),(w,d-c),(w-c,d),(-w+c,d),(-w,d-c),(-w,-d+c)]])
        rs=[r(y0,w0-bevel,depth-bevel,bevel),r(y0+bevel,w0,depth,bevel),r(y1-bevel,w1,depth,bevel),r(y1,w1-bevel,depth-bevel,bevel)]
        for a,b in zip(rs,rs[1:]): self.bridge(a,b)
        self.cap(rs[0],False); self.cap(rs[-1],True)
    def cylinder(self,start,end,radius,segments=16,bevel=.003):
        axis=unit(sub(end,start)); basis=unit(cross(axis,(0,0,1))); other=cross(basis,axis)
        length=math.sqrt(dot(sub(end,start),sub(end,start)))
        rs=[]
        for t,r in [(0,radius-bevel),(bevel,radius),(length-bevel,radius),(length,radius-bevel)]:
            rs.append(self.ring([tuple(start[k]+axis[k]*t+r*(basis[k]*math.cos(i*math.tau/segments)+other[k]*math.sin(i*math.tau/segments)) for k in range(3)) for i in range(segments)]))
        for a,b in zip(rs,rs[1:]): self.bridge(a,b,True)
        self.cap(rs[0],False); self.cap(rs[-1],True)
    def hand(self,x,y,z):
        # C-shaped clip hand, open downwards, with a hexagonal cross-section.
        rs=[]
        for i in range(13):
            a=math.radians(-45+i*270/12)
            radial=(math.cos(a),math.sin(a),0)
            rs.append(self.ring([(x+.021*radial[0]+.008*math.cos(j*math.tau/6)*radial[0],y+.021*radial[1]+.008*math.cos(j*math.tau/6)*radial[1],z+.008*math.sin(j*math.tau/6)) for j in range(6)]))
        # This sweep uses the opposite orientation to the vertical rings.
        start=len(self.f)
        for a,b in zip(rs,rs[1:]): self.bridge(a,b,True)
        self.cap(rs[0],False); self.cap(rs[-1],True)
        # Orient the closed shell using its signed volume.
        volume=sum(dot(self.v[a],cross(self.v[b],self.v[c])) for a,b,c in self.f[start:])
        if volume<0: self.f[start:]=[(a,c,b) for a,b,c in self.f[start:]]
    def export(self,name):
        assert len(self.f)<=1500,(name,len(self.f))
        edges=Counter(tuple(sorted((a,b))) for f in self.f for a,b in zip(f,f[1:]+f[:1]))
        assert all(n==2 for n in edges.values())
        face_normals=[unit(cross(sub(self.v[b],self.v[a]),sub(self.v[c],self.v[a]))) for a,b,c in self.f]
        adjacent=[[] for _ in self.v]
        for index,f in enumerate(self.f):
            for corner,vi in enumerate(f):
                a=unit(sub(self.v[f[(corner+1)%3]],self.v[vi])); b=unit(sub(self.v[f[(corner+2)%3]],self.v[vi]))
                adjacent[vi].append((index,math.acos(max(-1,min(1,dot(a,b))))))
        normals=[]
        for index,f in enumerate(self.f):
            for vi in f:
                if not self.smooth[index]: normals.append(face_normals[index]); continue
                ns=[(face_normals[j],w) for j,w in adjacent[vi] if self.smooth[j] and dot(face_normals[j],face_normals[index])>math.cos(math.radians(40))]
                normals.append(unit(tuple(sum(n[k]*w for n,w in ns) for k in range(3))))
        text=TEMPLATE.replace('ConstructorBlock2x3',name)
        def replace_array(key,values):
            nonlocal text
            values=list(values)
            text=re.sub(r'\b'+key+r': \*\d+\s*\{\s*a: [^}]+}',key+': *'+str(len(values))+' {\n            a: '+','.join(format(v,'.12g') for v in values)+'\n        }',text)
        replace_array('Vertices',(c*100 for v in self.v for c in v))
        replace_array('PolygonVertexIndex',(i for a,b,c in self.f for i in (a,b,-c-1)))
        replace_array('Normals',(c for n in normals for c in n))
        out=ROOT/'Assets/Models'/name; out.mkdir(parents=True,exist_ok=True)
        (out/(name+'.fbx')).write_text(text)
        meta=out/(name+'.fbx.meta')
        if not meta.exists():
            source=(ROOT/'Assets/Models/ConstructorBlock2x3/ConstructorBlock2x3.fbx.meta').read_text()
            source=re.sub(r'guid: [a-f0-9]+','guid: '+uuid.uuid4().hex,source,count=1)
            meta.write_text(source)
        size=[max(p[k] for p in self.v)-min(p[k] for p in self.v) for k in range(3)]
        (out/'README.md').write_text(f'# {name}\n\nOne static mesh object, {len(self.f)} triangles, explicit normals. No colors, materials, textures or rig. Assign your material in Unity.\n\nDimensions X/Y/Z: {size[0]:.6f} / {size[1]:.6f} / {size[2]:.6f} metres. Y up, identity transform. '+('Figure faces +Z, pivot at foot level. Separate closed shells are combined in one mesh; not boolean-unioned.\n' if name=='ConstructorFigure' else 'Arrow points +Z, pivot at the centre of its base footprint.\n'))
        print(name,len(self.f),'triangles, dimensions',size)

figure=Mesh()
for x in (-.041,.041):
    figure.box(x,.012,0,.047,.034,.034,.051)
    figure.box(x,-.002,.041,.18,.032,.032,.034)
figure.box(0,0,.174,.207,.077,.077,.04)
figure.box(0,0,.204,.334,.065,.088,.044)
figure.cylinder((0,.33,0),(0,.354,0),.028,12,.002)
figure.cylinder((0,.348,0),(0,.455,0),.057,20,.006)
figure.cylinder((0,.452,0),(0,.48,0),.027,12,.003)
for sign in (-1,1):
    figure.cylinder((sign*.088,.31,0),(sign*.136,.221,.005),.027,12,.005)
    figure.hand(sign*.148,.197,.008)
figure.export('ConstructorFigure')

arrow=Mesh()
# Extruded seven-corner arrow; concave cap triangulation is explicit.
poly=[(-.025,-.15),(.025,-.15),(.025,.015),(.075,.015),(0,.15),(-.075,.015),(-.025,.015)]
low=arrow.ring([(x,0,z) for x,z in poly]); high=arrow.ring([(x,.018,z) for x,z in poly])
arrow.bridge(low,high)
for a,b,c in [(0,1,2),(0,2,6),(6,2,4),(2,3,4),(6,4,5)]:
    arrow.tri(low[a],low[b],low[c]); arrow.tri(high[a],high[c],high[b])
arrow.export('ConstructorArrow')
