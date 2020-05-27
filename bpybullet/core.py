import pybullet
import bmesh
import numpy as np


class PhysicsObject:
    def __init__(self, bpy_object, bullet_object_id, collision_shape_id=None):
        self.bpy_object = bpy_object
        self.bullet_object_id = bullet_object_id
        self.collision_shape_id = collision_shape_id
        self.xyz_trajectory = None
        self.euler_angles_trajectory = None

    def change_dynamics(self, **kwargs):
        pybullet.changeDynamics(self.bullet_object_id, linkIndex=-1, **kwargs)


class BulletWorld:
    def __init__(self, gravity=(0, 0, -9.81)):
        self.physics_client_id = pybullet.connect(pybullet.DIRECT)
        pybullet.setGravity(*gravity, self.physics_client_id)
        self.objects = {}
        self.n_steps = 0
        self.step = 0

    def add_as_mesh(self, bpy_object, mass):
        mesh = bmesh.new()
        mesh.from_mesh(bpy_object.data)
        bmesh.ops.triangulate(mesh, faces=mesh.faces[:])

        collision_shape_id = pybullet.createCollisionShape(pybullet.GEOM_MESH,
                                                           vertices=[v.co for v in mesh.verts],
                                                           indices=[v.index for face in mesh.faces for v in face.verts])
        bullet_object_id = pybullet.createMultiBody(baseMass=mass,
                                                    baseCollisionShapeIndex=collision_shape_id,
                                                    basePosition=bpy_object.location,
                                                    baseOrientation=pybullet.getQuaternionFromEuler(bpy_object.rotation_euler))
        physics_object = PhysicsObject(bpy_object, bullet_object_id, collision_shape_id)
        self.objects[bpy_object.name] = physics_object
        return physics_object

    def add_as_sphere(self, bpy_object, mass, radius=None):
        if radius is None:
            mesh = bmesh.new()
            mesh.from_mesh(bpy_object.data)
            radius = max([np.linalg.norm(v.co) for v in mesh.verts])

        collision_shape_id = pybullet.createCollisionShape(pybullet.GEOM_SPHERE, radius=radius)
        bullet_object_id = pybullet.createMultiBody(baseMass=mass,
                                                    baseCollisionShapeIndex=collision_shape_id,
                                                    basePosition=bpy_object.location,
                                                    baseOrientation=pybullet.getQuaternionFromEuler(bpy_object.rotation_euler))

        physics_object = PhysicsObject(bpy_object, bullet_object_id, collision_shape_id)
        self.objects[bpy_object.name] = physics_object
        return physics_object

    def preallocate_trajectories(self, n_steps):
        for obj in self.objects.values():
            obj.xyz_trajectory = np.zeros((n_steps, 3))
            obj.euler_angles_trajectory = np.zeros((n_steps, 3))
        self.step = 0
        self.n_steps = n_steps

    def step_simulation(self):
        pybullet.stepSimulation(self.physics_client_id)
        self.step += 1

    def write_step_to_trajectories(self):
        for obj in self.objects.values():
            position, orientation = pybullet.getBasePositionAndOrientation(obj.bullet_object_id)
            obj.xyz_trajectory[self.step] = position
            obj.euler_angles_trajectory[self.step] = pybullet.getEulerFromQuaternion(orientation)

    def simulate(self, n_steps):
        self.preallocate_trajectories(n_steps)

        for i in range(n_steps):
            self.step_simulation()
            self.write_step_to_trajectories()

    def set_keyframes(self, sub_step=1):
        for i in range(int(self.n_steps/sub_step)):
            for obj in self.objects.values():
                obj.bpy_object.location = obj.xyz_trajectory[i*sub_step]
                obj.bpy_object.keyframe_insert(data_path="location", frame=i)

                obj.bpy_object.rotation_euler = obj.euler_angles_trajectory[i*sub_step]
                obj.bpy_object.keyframe_insert(data_path="rotation_euler", frame=i)

                for fcurve in obj.bpy_object.animation_data.action.fcurves:
                    kf = fcurve.keyframe_points[-1]
                    kf.interpolation = 'CONSTANT'
