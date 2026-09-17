"""Generate neutral constructor-toy FBX models and previews with Blender.

Run: blender --background --python Tools/generate_constructor_fantasy.py
Coordinates in the builders are Unity metres: X right, Y up, +Z forward.
Exports use the project's existing ASCII FBX template and importer settings.
"""
from pathlib import Path
import math
import json
import random
import re
import uuid
from collections import Counter
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'Assets/Models/ConstructorFantasy'
OUT.mkdir(parents=True, exist_ok=True)
TEMPLATE = (ROOT / 'Assets/Models/ConstructorFigure/ConstructorFigure.fbx').read_text()
META = (ROOT / 'Assets/Models/ConstructorFigure/ConstructorFigure.fbx.meta').read_text()
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
parts = []
assets = []

def xyz(p): return Vector((p[0], -p[2], p[1]))

def finish(obj, bevel=0):
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        mod = obj.modifiers.new('Molded edge', 'BEVEL')
        mod.width = bevel
        mod.segments = 1
        bpy.ops.object.modifier_apply(modifier=mod.name)
    parts.append(obj)
    return obj

def box(p, size, bevel=.003):
    bpy.ops.mesh.primitive_cube_add(size=1, location=xyz(p))
    obj = bpy.context.object
    obj.dimensions = (size[0], size[2], size[1])
    return finish(obj, bevel)

def rod(a, b, r, n=10, r2=None, bevel=0):
    a, b = xyz(a), xyz(b)
    bpy.ops.mesh.primitive_cone_add(vertices=n, radius1=r, radius2=r if r2 is None else r2,
                                  depth=(b-a).length, location=(a+b)/2)
    obj = bpy.context.object
    obj.rotation_euler = (b-a).to_track_quat('Z', 'Y').to_euler()
    return finish(obj, bevel)

def beam(a, b, width, depth=None):
    obj = box(tuple((a[i]+b[i])/2 for i in range(3)), (width, math.dist(a,b), depth or width))
    obj.rotation_euler = (xyz(b)-xyz(a)).to_track_quat('Z','Y').to_euler()
    return obj

def ico(p, size, seed=0, subdivisions=1):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=subdivisions, radius=1, location=xyz(p))
    obj = bpy.context.object
    rng = random.Random(seed)
    for v in obj.data.vertices:
        v.co *= rng.uniform(.86, 1.12)
    obj.scale = (size[0], size[2], size[1])
    return finish(obj)

def stud(x,y,z,r=.023): rod((x,y,z),(x,y+.018,z),r,12,bevel=.002)

def raw_mesh(name, vertices, faces):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata([xyz(v) for v in vertices], [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name,mesh)
    bpy.context.collection.objects.link(obj)
    parts.append(obj)
    return obj

def prism(points, thickness):
    # Extruded convex X/Y polygon, centred about Z=0.
    n=len(points)
    vertices=[(x,y,z) for z in (-thickness/2,thickness/2) for x,y in points]
    faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]
    faces += [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    return raw_mesh('Wedge',vertices,faces)

def export(name, note, hierarchy=None):
    global parts
    if hierarchy:
        for part in hierarchy:
            for piece in part['objects']:
                group=piece.vertex_groups.new(name=part['name'])
                group.add(list(range(len(piece.data.vertices))),1.0,'REPLACE')
    bpy.ops.object.select_all(action='DESELECT')
    for obj in parts: obj.select_set(True)
    bpy.context.view_layer.objects.active=parts[0]
    bpy.ops.object.join()
    obj=bpy.context.object
    obj.name=name
    bpy.context.scene.cursor.location=(0,0,0)
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
    # Recalculate each closed shell before triangulation, including custom wedges.
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')
    mod=obj.modifiers.new('Explicit triangles','TRIANGULATE')
    bpy.ops.object.modifier_apply(modifier=mod.name)
    vertices=[(v.co.x,v.co.z,-v.co.y) for v in obj.data.vertices]
    faces=[tuple(p.vertices) for p in obj.data.polygons]
    edges=Counter(tuple(sorted((a,b))) for f in faces for a,b in zip(f,f[1:]+f[:1]))
    assert all(n==2 for n in edges.values()), (name,'open mesh')
    directed=Counter((a,b) for f in faces for a,b in zip(f,f[1:]+f[:1]))
    assert all(directed[(a,b)]==directed[(b,a)] for a,b in directed), (name,'winding')
    assert all(p.area>1e-12 for p in obj.data.polygons), (name,'degenerate face')
    assert len(faces)<=4000, (name,'triangle budget',len(faces))
    text=TEMPLATE.replace('ConstructorFigure',name)
    arrays={'Vertices':[c*100 for v in vertices for c in v],
            'PolygonVertexIndex':[i for a,b,c in faces for i in (a,b,-c-1)],
            'Normals':[c for p in obj.data.polygons for _ in p.vertices for c in (p.normal.x,p.normal.z,-p.normal.y)]}
    for key, values in arrays.items():
        text=re.sub(r'\b'+key+r': \*\d+\s*\{\s*a: [^}]+}',
                    key+': *'+str(len(values))+' {\n            a: '+','.join(format(v,'.10g') for v in values)+'\n        }',text)
    if hierarchy:
        # Keep the assembled preview, but emit four independently pivoted FBX meshes.
        # File coordinates and node translations are centimetres, as in the template.
        header=text.split('Definitions: {')[0]
        model_template=text.split('    Model: 2000,')[1].split('\n}\nConnections:')[0]
        root='    Model: 2000,'+model_template.replace('"Mesh" {','"Null" {',1)
        objects=[root]
        connections=['    C: "OO",2000,0']
        pivots={p['name']:p['pivot'] for p in hierarchy}
        ids={p['name']:2010+i for i,p in enumerate(hierarchy)}
        for index,part in enumerate(hierarchy):
            group_index=obj.vertex_groups[part['name']].index
            selected=[v.index for v in obj.data.vertices if any(g.group==group_index for g in v.groups)]
            selected_set=set(selected)
            polygons=[p for p in obj.data.polygons if set(p.vertices)<=selected_set]
            remap={old:new for new,old in enumerate(selected)}
            geometry=text.split('Objects: {\n')[1].split('    Model: 2000,')[0]
            geometry=geometry.replace('Geometry: 1000,','Geometry: '+str(1010+index)+',',1).replace('Geometry::'+name,'Geometry::'+part['name'])
            arrays={'Vertices':[(vertices[i][k]-part['pivot'][k])*100 for i in selected for k in range(3)],
                    'PolygonVertexIndex':[n for p in polygons for a,b,c in [tuple(remap[i] for i in p.vertices)] for n in (a,b,-c-1)],
                    'Normals':[c for p in polygons for _ in p.vertices for c in (p.normal.x,p.normal.z,-p.normal.y)]}
            for key,values in arrays.items():
                geometry=re.sub(r'\b'+key+r': \*\d+\s*\{\s*a: [^}]+}',key+': *'+str(len(values))+' {\n            a: '+','.join(format(v,'.10g') for v in values)+'\n        }',geometry)
            parent_pivot=pivots.get(part['parent'],(0,0,0))
            translation=[part['pivot'][k]-parent_pivot[k] for k in range(3)]
            model='    Model: '+str(ids[part['name']])+','+model_template.replace('Model::'+name,'Model::'+part['name'])
            model=model.replace('"A",0,0,0','"A",'+','.join(format(v*100,'.10g') for v in translation),1)
            objects.extend([geometry,model])
            connections.extend([f'    C: "OO",{1010+index},{ids[part["name"]]}',
                                f'    C: "OO",{ids[part["name"]]},{ids.get(part["parent"],2000)}'])
            part['triangles']=len(polygons)
            part['local_position']=translation
        assert sum(p['triangles'] for p in hierarchy)==len(faces)
        text=header+'''Definitions: {
    Version: 100
    Count: 9
    ObjectType: "Geometry" {
        Count: 4
    }
    ObjectType: "Model" {
        Count: 5
    }
}
Objects: {
'''+ '\n'.join(objects)+'\n}\nConnections: {\n'+'\n'.join(connections)+'\n}\n'
    (OUT/(name+'.fbx')).write_text(text)
    meta=OUT/(name+'.fbx.meta')
    if not meta.exists(): meta.write_text(re.sub(r'guid: [a-f0-9]+','guid: '+uuid.uuid4().hex,META,count=1))
    bounds=[[min(v[k] for v in vertices),max(v[k] for v in vertices)] for k in range(3)]
    assets.append((obj,dict(name=name,triangles=len(faces),bounds=bounds,
                            dimensions=[b-a for a,b in bounds],note=note)))
    if hierarchy:
        assets[-1][1]['parts']=[{k:v for k,v in p.items() if k!='objects'} for p in hierarchy]
    parts=[]
    obj.hide_render=True
    print('EXPORTED',name,len(faces),flush=True)

# Trebuchet: studded runners, A-frame, axle, asymmetric throwing arm,
# hanging counterweight and a suspended sling. Four meshes with mechanical pivots.
for x in (-.25,.25):
    box((x,.055,0),(.105,.11,.92),.008)
    for z in (-.36,-.24,0,.24,.36): stud(x,.11,z)
    for z in (-.31,.31): beam((x,.13,z),(x,.65,0),.065)
for z in (-.29,.29): box((0,.10,z),(.60,.06,.075),.006)
rod((-.34,.65,0),(.34,.65,0),.044,12,bevel=.004)
for x in (-.345,.345): rod((x-.018,.65,0),(x+.018,.65,0),.067,12,bevel=.004)
trebuchet_parts=[dict(name='Base',parent=None,pivot=(0,0,0),objects=list(parts))]
part_start=len(parts)
beam((0,.47,-.28),(0,1.05,.65),.062,.075)
rod((-.09,.47,-.28),(.09,.47,-.28),.027)
trebuchet_parts.append(dict(name='ActiveBeam',parent='Base',pivot=(0,.65,0),objects=parts[part_start:]))
part_start=len(parts)
for x in (-.073,.073): beam((x,.47,-.28),(x,.30,-.28),.023)
box((0,.255,-.28),(.235,.17,.19),.012)
for x in (-.06,.06): stud(x,.34,-.28,.025)
trebuchet_parts.append(dict(name='HangingLoad',parent='ActiveBeam',pivot=(0,.47,-.28),objects=parts[part_start:]))
part_start=len(parts)
for x in (-.055,.055): rod((0,1.05,.65),(x,.72,.79),.009,6)
box((0,.704,.79),(.155,.027,.15),.006)
for x in (-.068,.068): box((x,.73,.79),(.018,.037,.15),.003)
box((0,.73,.857),(.145,.037,.016),.003)
trebuchet_parts.append(dict(name='ProjectileBasin',parent='ActiveBeam',pivot=(0,1.05,.65),objects=parts[part_start:]))
export('ConstructorTrebuchet','Four pivoted meshes: Base → ActiveBeam → {HangingLoad, ProjectileBasin}. Rotate each moving part about local X. Base pivot (0,0,0); beam axle (0,0.65,0); load hinge (0,0.47,-0.28); basin suspension (0,1.05,0.65), all in assembled metres. Load includes hanger straps; basin includes suspension ropes. Local positions relative to beam: load (0,-0.18,-0.28), basin (0,0.40,0.65). Zero rotations reproduce the rest pose. Counter-rotate the hanging parts to maintain their world orientation as the beam moves, or animate them independently to swing. Throws toward +Z; accepts rocks around 0.10 m. No baked animation or physics.',trebuchet_parts)

for i, size in enumerate([(.054,.047,.052),(.062,.041,.048),(.047,.057,.044),(.058,.050,.060)],1):
    rock=ico((0,0,0),size,100+i,2 if i==4 else 1)
    export(f'ConstructorRock{i:02d}','Centred projectile pivot. Faceted molded boulder; sized for the trebuchet sling.')

# Bow lies in Y/Z. The handle is vertical and its centre is the attachment pivot.
points=[(-.17,-.007),(-.145,.016),(-.115,.043),(-.075,.052),(-.033,.015),
        (0,0),(.033,.015),(.075,.052),(.115,.043),(.145,.016),(.17,-.007)]
for (y,z),(yy,zz) in zip(points,points[1:]):
    beam((0,y,z),(0,yy,zz),.014,.019)
rod((0,-.034,0),(0,.034,0),.0085,10)
for y in (-.024,-.012,0,.012,.024): rod((0,y-.0015,0),(0,y+.0015,0),.0092,10)
rod((0,-.17,-.008),(0,.17,-.008),.0018,6)
export('ConstructorBow','Grip-centre pivot, Y-up, shoots +Z. 0.34 m tall; 0.017 m grip fits the original figure clip opening. Suggested hand placement (±0.148, 0.197, 0.008) relative to figure.')

for variant in ('Arrow','HeavyArrow'):
    rod((0,0,-.135),(0,0,.093),.0037,8)
    rod((0,0,.090),(0,0,.14),.016 if variant=='Arrow' else .022,4,r2=0)
    for angle in (0,math.tau/3,2*math.tau/3):
        obj=raw_mesh('Fletching',[(x,y,z) for x in (-.0012,.0012) for y,z in [(0,-.126),(.019,-.124),(.013,-.076),(0,-.09)]],
                     [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)])
        obj.rotation_euler.y=angle
    rod((0,0,-.14),(0,0,-.127),.0055,8)
    export('ConstructorBow'+variant,'Centre pivot, points +Z. 0.28 m long, three solid molded fins; fits ConstructorBow. Static mesh.')

# Reuse the reference figure silhouette and exact scale, then add wizard parts.
source=TEMPLATE
def values(key): return [float(x) for x in re.search(r'\b'+key+r': \*\d+\s*\{\s*a: ([^}]+)',source).group(1).split(',')]
v=values('Vertices'); indices=[int(x) for x in values('PolygonVertexIndex')]
raw_mesh('Figure',[(v[i]/100,v[i+1]/100,v[i+2]/100) for i in range(0,len(v),3)],
         [(indices[i],indices[i+1],-indices[i+2]-1) for i in range(0,len(indices),3)])
rod((0,.052,0),(0,.205,0),.094,8,r2=.072) # molded robe skirt
rod((0,.44,0),(0,.457,0),.091,12,bevel=.003)
rod((0,.453,0),(0,.608,-.012),.063,10,r2=.023)
rod((0,.608,-.012),(.032,.656,-.018),.023,8,r2=0)
beard=prism([(-.043,.366),(0,.265),(.043,.366),(.031,.39),(-.031,.39)],.020)
beard.location=xyz((0,0,.060))
rod((-.148,.025,.008),(-.148,.493,.008),.010,10)
ico((-.148,.522,.008),(.033,.047,.033),2)
for x in (-.022,.022):
    rod((x,.397,.055),(x,.397,.062),.005,8)
export('ConstructorMage','Original 0.48 m figure body plus robe, beard, pointed hat and staff. Faces +Z; foot-centre pivot. Neutral single mesh, no rig; approximately 0.66 m tall with hat.')

# The projectile and bursts deliberately contain no color, textures or materials.
ico((0,0,.04),(.052,.052,.073),8)
for i in range(5):
    a=i*math.tau/5
    x,y=.032*math.cos(a),.032*math.sin(a)
    rod((x,y,.024),(x*.5,y*.5,-.16-(i%2)*.025),.021,5,r2=0)
export('ConstructorSpellProjectile','Centre pivot; travels +Z with a tapered trail toward -Z. Assign any opaque, transparent or emissive material in Unity.')

ico((0,0,0),(.086,.086,.086),44)
for i in range(12):
    a=i*math.tau/12
    direction=Vector((math.cos(a),.35*math.sin(a*3),math.sin(a))).normalized()
    rod(tuple(direction*.055),tuple(direction*(.19+(i%3)*.018)),.031,5,r2=0)
export('ConstructorSpellExplosion01','Radial starburst with chunky rays; impact-centre pivot. Neutral material-ready static effect.')

bpy.ops.mesh.primitive_torus_add(major_segments=16,minor_segments=4,location=(0,0,0),major_radius=.17,minor_radius=.019)
finish(bpy.context.object)
for i in range(8):
    a=i*math.tau/8
    p=(.12*math.cos(a),.025,.12*math.sin(a))
    rod(p,(p[0]*1.3,.10+(i%3)*.018,p[2]*1.3),.028,5,r2=0)
ico((0,.026,0),(.065,.05,.065),9)
export('ConstructorSpellExplosion02','Horizontal shock ring with crystal fragments; impact-centre pivot, Y-up. Neutral material-ready static effect.')

for i in range(7):
    a=i*math.tau/7
    ico((.088*math.cos(a),.02+.018*(i%2),.088*math.sin(a)),(.060,.067,.060),30+i)
ico((0,.085,0),(.092,.10,.092),70,2)
for i in range(5):
    a=i*math.tau/5
    ico((.15*math.cos(a),.12+.025*(i%2),.15*math.sin(a)),(.020,.026,.020),80+i)
export('ConstructorSpellExplosion03','Faceted puff cloud with flying fragments; impact-centre pivot. Neutral material-ready static effect.')

(OUT/'manifest.json').write_text(json.dumps([info for _,info in assets],indent=2))
rows='\n'.join('| '+i['name']+' | '+str(i['triangles'])+' | '+' × '.join(f'{d:.3f}' for d in i['dimensions'])+' |' for _,i in assets)
notes='\n\n'.join('**'+i['name']+'** — '+i['note'] for _,i in assets)
(OUT/'README.md').write_text('''# Constructor fantasy models

13 low-poly, neutral FBX assets matching ConstructorFigure (0.48 m body height).
The trebuchet has four hierarchical meshes with animation pivots; each other
FBX has one mesh. All have explicit flat normals and an identity root transform.
Molded bevels, studs and geometric silhouettes match construction-toy pieces.
Y-up, +Z forward, dimensions in metres after Unity file-unit conversion.
No textures, UVs, vertex colors, embedded materials, rig, animation or colliders.
Assign a Unity material to the renderer; the spell assets can share a tintable
material. Effects are static meshes intended to be spawned, scaled and faded
by your gameplay/VFX code. Overlapping closed shells are intentional; these
are visual models, not boolean-unioned manufacturing solids.

| Model | Triangles | X × Y × Z (m) |
| --- | ---: | --- |
'''+rows+'\n\n'+notes+'''

Regenerate using Blender 5.2 in background mode with
`Tools/generate_constructor_fantasy.py`. The generator validates closed edges,
consistent winding, nondegenerate faces and a 4,000 triangle ceiling per asset.
Previews show the actual exported geometry
with neutral studio shading; individual preview scales vary for readability.
''')

# Render the actual export meshes using a fast neutral studio view.
scene=bpy.context.scene
scene.render.engine='BLENDER_WORKBENCH'
scene.display.shading.light='STUDIO'
scene.display.shading.studiolight_rotate_z=.4
scene.display.shading.color_type='SINGLE'
scene.display.shading.single_color=(.67,.69,.72)
scene.display.shading.show_shadows=True
scene.display.shading.show_cavity=True
scene.display.shading.cavity_type='BOTH'
scene.display.shading.curvature_ridge_factor=1.3
scene.display.shading.curvature_valley_factor=1.0
scene.display.shading.background_type='WORLD'
scene.world.color=(.065,.078,.095)
scene.render.image_settings.file_format='PNG'
scene.render.resolution_percentage=100
bpy.ops.object.camera_add()
camera=bpy.context.object
scene.camera=camera
camera.data.type='ORTHO'
def aim(centre,direction,scale):
    camera.location=Vector(centre)+Vector(direction)*4
    camera.rotation_euler=(Vector(centre)-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.ortho_scale=scale
scene.render.resolution_x=700
scene.render.resolution_y=700
for obj,info in assets:
    obj.hide_render=False
    centre=sum((Vector(corner) for corner in obj.bound_box),Vector())/8
    aim(centre,(1,-1.6,.95),max(obj.dimensions)*1.42)
    scene.render.filepath=str(OUT/(obj.name+'-preview.png'))
    bpy.ops.render.render(write_still=True)
    obj.hide_render=True

# A front-facing contact sheet with the models rotated for a three-quarter view.
for index,(obj,info) in enumerate(assets):
    obj.hide_render=False
    obj.rotation_euler=(-Vector((1,-1.6,.95))).to_track_quat('-Z','Y').inverted().to_euler()
    bpy.context.view_layer.update()
    corners=[obj.matrix_world@Vector(v) for v in obj.bound_box]
    span=max(max(v[k] for v in corners)-min(v[k] for v in corners) for k in (0,1))
    obj.scale *= 1.45/span
    bpy.context.view_layer.update()
    corners=[obj.matrix_world@Vector(v) for v in obj.bound_box]
    centre=sum(corners,Vector())/8
    col=index%4; row=index//4
    obj.location+=Vector((col*2.15,-row*2.25,0))-centre
    bpy.ops.object.text_add(location=(col*2.15-1,-row*2.25-1.02,.8))
    text=bpy.context.object
    text.data.body=info['name'].replace('Constructor','').replace('SpellExplosion','Spell burst ')
    text.data.size=.13
    text.data.extrude=0
    text.color=(.8,.8,.8,1)
aim((3.18,-3.4,0),(0,0,1),9.25)
scene.render.resolution_x=1600
scene.render.resolution_y=1700
scene.render.filepath=str(OUT/'ConstructorFantasy-preview.png')
bpy.ops.render.render(write_still=True)
print('COMPLETE: 13 assets, previews, manifest and documentation.',flush=True)
