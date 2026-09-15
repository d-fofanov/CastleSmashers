import bpy, math, json, os
from mathutils import Vector
OUT=os.path.dirname(os.path.abspath(__file__))
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene
scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=1
P=.1; H=.12; SH=.0225; WALL=.015
mat=bpy.data.materials.new('Neutral_Plastic'); mat.diffuse_color=(.46,.5,.53,1); mat.use_nodes=True
bs=mat.node_tree.nodes.get('Principled BSDF'); bs.inputs['Base Color'].default_value=(.46,.5,.53,1); bs.inputs['Roughness'].default_value=.3

def active(o):
 bpy.ops.object.select_all(action='DESELECT'); o.select_set(True); bpy.context.view_layer.objects.active=o

def box(w,l,h,z=0):
 bpy.ops.mesh.primitive_cube_add(size=1,location=(0,0,z+h/2)); o=bpy.context.object; o.dimensions=(w,l,h); bpy.ops.object.transform_apply(location=False,rotation=False,scale=True); return o

def cyl(r,h,x=0,y=0,z=0):
 bpy.ops.mesh.primitive_cylinder_add(vertices=32,radius=r,depth=h,location=(x,y,z+h/2)); return bpy.context.object

def boolean(o,b,mode):
 active(o); m=o.modifiers.new(mode,'BOOLEAN'); m.operation=mode; m.solver='EXACT'; m.object=b; bpy.ops.object.modifier_apply(modifier=m.name); bpy.data.objects.remove(b,do_unlink=True)

def brick(w,l,h):
 o=box(w*P,l*P,h)
 boolean(o,box(w*P-2*WALL,l*P-2*WALL,h-WALL+.01,-.01),'DIFFERENCE')
 for x in range(w):
  for y in range(l): boolean(o,cyl(.03,SH+.002,(x-(w-1)/2)*P,(y-(l-1)/2)*P,h-.002),'UNION')
 # Support tubes on two-wide pieces, solid support posts on single rows.
 if w==2:
  for j in range(l-1):
   y=(j-(l-2)/2)*P
   tube=cyl(.04,h-WALL+.003,0,y,.003)
   boolean(tube,cyl(.03,h+.02,0,y,-.005),'DIFFERENCE'); boolean(o,tube,'UNION')
 elif l>1:
  for j in range(l-1): boolean(o,cyl(.012,h-WALL+.003,0,(j-(l-2)/2)*P,.003),'UNION')
 return o

def prism(kind):
 w=.2; l=.2; h=.12
 # Profiles on yz plane, extruded across x.
 if kind=='Ramp': profile=[(-l/2,0),(l/2,0),(l/2,h),(-l/2,.02)]
 elif kind=='Wedge':
  l=.3; h=.08; profile=[(-l/2,0),(l/2,0),(l/2,h),(-l/2,.015)]
 else: profile=[(-l/2,0),(l/2,0),(0,h)]
 n=len(profile); v=[(x,y,z) for x in [-w/2,w/2] for y,z in profile]
 faces=[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
 mesh=bpy.data.meshes.new(kind); mesh.from_pydata(v,[],faces); mesh.update(); o=bpy.data.objects.new(kind,mesh); scene.collection.objects.link(o)
 active(o); bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT'); bpy.ops.mesh.normals_make_consistent(inside=False); bpy.ops.object.mode_set(mode='OBJECT'); return o

items=[]; stats=[]
def finish(o,name):
 active(o); o.name=name; o.data.name=name+'_Mesh'
 m=o.modifiers.new('Rounded_Edges','BEVEL'); m.width=.0015; m.segments=3; m.limit_method='ANGLE'; bpy.ops.object.modifier_apply(modifier=m.name)
 for p in o.data.polygons: p.use_smooth=True
 m=o.modifiers.new('Weighted_Normals','WEIGHTED_NORMAL'); m.keep_sharp=True; m.weight=50; bpy.ops.object.modifier_apply(modifier=m.name)
 bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT'); bpy.ops.uv.smart_project(angle_limit=math.radians(66),island_margin=.02); bpy.ops.object.mode_set(mode='OBJECT')
 # Set origin at center of bottom without moving geometry.
 scene.cursor.location=(0,0,0); bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
 o.data.materials.append(mat)
 m=o.modifiers.new('Triangulated','TRIANGULATE'); bpy.ops.object.modifier_apply(modifier=m.name)
 import bmesh
 bm=bmesh.new(); bm.from_mesh(o.data); bad=sum(not e.is_manifold for e in bm.edges); vol=bm.calc_volume(signed=True); bm.free()
 assert bad==0,(name,bad)
 assert vol>0,(name,vol)
 dims=list(o.dimensions)
 bpy.ops.export_scene.fbx(filepath=OUT+'/'+name+'.fbx',use_selection=True,object_types={'MESH'},apply_unit_scale=True,apply_scale_options='FBX_SCALE_UNITS',axis_forward='-Z',axis_up='Y',bake_space_transform=True,use_mesh_modifiers=True,mesh_smooth_type='OFF',add_leaf_bones=False,bake_anim=False,path_mode='AUTO')
 stats.append(dict(name=name,dimensions_xyz_m=dims,triangles=len(o.data.polygons),vertices=len(o.data.vertices),non_manifold_edges=bad))
 items.append(o); o.hide_render=True; o.hide_set(True)

for w,l in [(1,1),(1,2),(1,3),(1,4),(1,6),(1,8),(2,2),(2,3),(2,4),(2,6),(2,8)]: finish(brick(w,l,H),f'Brick_{w}x{l}')
for w,l in [(1,1),(1,2),(1,4),(2,2),(2,4)]: finish(brick(w,l,.04),f'Plate_{w}x{l}')
for w,l in [(1,1),(1,2),(2,2),(2,4)]: finish(box(w*P,l*P,.04),f'Tile_{w}x{l}')
for kind in ['Ramp','Roof_Prism','Wedge']: finish(prism(kind),kind)
finish(box(.1,.1,.1),'Plain_Cube'); finish(box(.2,.3,.12),'Plain_Rectangular_Block'); finish(cyl(.075,.15),'Plain_Cylinder'); finish(prism('Triangular_Prism'),'Plain_Triangular_Prism')
# Re-import every delivered FBX and check one mesh and scale preservation.
for entry in stats:
 before=set(bpy.data.objects)
 bpy.ops.import_scene.fbx(filepath=OUT+'/'+entry['name']+'.fbx')
 new=list(set(bpy.data.objects)-before); meshes=[o for o in new if o.type=='MESH']; assert len(meshes)==1
 dims=sorted(meshes[0].dimensions); expected=sorted(entry['dimensions_xyz_m']); assert max(abs(a-b) for a,b in zip(dims,expected))<1e-5,(entry['name'],dims,expected)
 entry['fbx_roundtrip_verified']=True
 for o in new: bpy.data.objects.remove(o,do_unlink=True)
json.dump(stats,open(OUT+'/mesh_manifest.json','w'),indent=2)
# Render an actual-mesh contact sheet with a fixed orthographic view.
for idx,o in enumerate(items):
 o.hide_render=False; o.hide_set(False)
 row=idx//6; col=idx%6
 o.rotation_euler[2]=math.radians(-35)
 o.location=(col*.85,-row*.72,0)
 # labels lie flat on the ground, readable from camera
 bpy.ops.object.text_add(location=(col*.85,-row*.72-.28,.003))
 t=bpy.context.object; t.data.body=o.name.replace('_',' '); t.data.align_x='CENTER'; t.data.size=.045
 black=bpy.data.materials.get('Label')
 if not black:
  black=bpy.data.materials.new('Label'); black.diffuse_color=(.02,.03,.04,1)
 t.data.materials.append(black)
bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,-.006)); ground=bpy.context.object
gm=bpy.data.materials.new('Background'); gm.diffuse_color=(.88,.9,.92,1); ground.data.materials.append(gm)
bpy.ops.object.camera_add(location=(2.12,-5.5,7.5)); cam=bpy.context.object; target=Vector((2.12,-1.5,0)); cam.rotation_euler=(target-cam.location).to_track_quat('-Z','Y').to_euler(); cam.data.type='ORTHO'; cam.data.ortho_scale=5.6; scene.camera=cam
bpy.ops.object.light_add(type='AREA',location=(1,-1,6)); bpy.context.object.data.energy=1100; bpy.context.object.data.shape='DISK'; bpy.context.object.data.size=5
scene.world.color=(.4,.4,.4); scene.render.engine='CYCLES'; scene.cycles.samples=24
scene.render.resolution_x=1800; scene.render.resolution_y=1450; scene.render.resolution_percentage=100
scene.render.filepath=OUT+'/preview.png'; bpy.ops.render.render(write_still=True)
print('DONE: 27 FBX exports and roundtrip checks')
