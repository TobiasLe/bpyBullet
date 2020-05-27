import bpy
import bpybullet
import bmesh


# create a sphere:
mesh = bpy.data.meshes.new("sphere")
bpy_sphere = bpy.data.objects.new("sphere", mesh)
bpy.context.scene.collection.objects.link(bpy_sphere)
bm = bmesh.new()
bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, diameter=0.5)
bm.to_mesh(mesh)
bm.free()

# create a cube:
mesh = bpy.data.meshes.new("cube")
bpy_cube = bpy.data.objects.new("cube", mesh)
bpy.context.scene.collection.objects.link(bpy_cube)
bm = bmesh.new()
bmesh.ops.create_cube(bm, size=1)
bm.to_mesh(mesh)
bm.free()

# position sphere above cube
bpy_sphere.location = (0, 0, 5)
bpy_cube.location = (0, 0, 0)

# set up physics simulation
bullet_world = bpybullet.BulletWorld()

physics_sphere = bullet_world.add_as_sphere(bpy_sphere, mass=1)
physics_sphere.change_dynamics(restitution=0.7)  # makes it bouncy

physics_cube = bullet_world.add_as_mesh(bpy_cube, mass=0)  # mass=0 --> will not be affected by gravity
physics_cube.change_dynamics(restitution=1)

# run simulation and set the result as keyframes:
bullet_world.simulate(n_steps=2000)
bullet_world.set_keyframes(sub_step=8)  # will only set every 8th frame of the simulation as frame in blender

# run this script with: blender --python sphere_above_cube.py
