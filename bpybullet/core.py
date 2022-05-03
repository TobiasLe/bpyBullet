import pybullet
import bmesh
import numpy as np
import bpy


class PhysicsObject:
    def __init__(self, bpy_object, bullet_object_id, collision_shape_id=None):
        self.bpy_object = bpy_object
        self.bullet_object_id = bullet_object_id
        self.collision_shape_id = collision_shape_id
        self.xyz_trajectory = None
        self.euler_angles_trajectory = None
        self.contacts = []

    def change_dynamics(self, **kwargs):
        pybullet.changeDynamics(self.bullet_object_id, linkIndex=-1, **kwargs)

    def reset_base_position_and_orientation(self, position=None, orientation=None):
        if position is None:
            position = self.bpy_object.location
        if orientation is None:
            orientation = pybullet.getQuaternionFromEuler(self.bpy_object.rotation_euler)
        pybullet.resetBasePositionAndOrientation(self.bullet_object_id, position, orientation)

    def get_collision_partner_id(self, collision):
        if self.bullet_object_id == collision.body_unique_id_a:
            partner_id = collision.body_unique_id_b
        elif self.bullet_object_id == collision.body_unique_id_b:
            partner_id = collision.body_unique_id_a
        else:
            raise ValueError("not a collision of this object!")
        return partner_id


class ContactPoint:
    def __init__(self, contact_info, step=None):
        self.contact_flag = contact_info[0]
        self.body_unique_id_a = contact_info[1]
        self.body_unique_id_b = contact_info[2]
        self.link_index_a = contact_info[3]
        self.link_index_b = contact_info[4]
        self.position_on_a = contact_info[5]
        self.position_on_b = contact_info[6]
        self.contact_normal_on_b = contact_info[7]
        self.contact_distance = contact_info[8]
        self.normal_force = contact_info[9]
        self.lateral_friction1 = contact_info[10]
        self.lateral_friction_dir_1 = contact_info[11]
        self.lateral_friction2 = contact_info[12]
        self.lateral_friction_dir_2 = contact_info[13]

        self.step = step


class BulletWorld:
    def __init__(self, gravity=(0, 0, -9.81)):
        self.physics_client_id = pybullet.connect(pybullet.DIRECT)
        pybullet.setGravity(*gravity, self.physics_client_id)
        self.objects = []
        self.objects_by_bpy_name = {}
        self.objects_by_bullet_id = {}
        self.n_steps = 0
        self.step = 0

    def add_physics_object(self, physics_object):
        self.objects.append(physics_object)
        self.objects_by_bpy_name[physics_object.bpy_object.name] = physics_object
        self.objects_by_bullet_id[physics_object.bullet_object_id] = physics_object

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
                                                    baseOrientation=pybullet.getQuaternionFromEuler(
                                                        bpy_object.rotation_euler))
        physics_object = PhysicsObject(bpy_object, bullet_object_id, collision_shape_id)
        self.add_physics_object(physics_object)
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
                                                    baseOrientation=pybullet.getQuaternionFromEuler(
                                                        bpy_object.rotation_euler))

        physics_object = PhysicsObject(bpy_object, bullet_object_id, collision_shape_id)
        self.add_physics_object(physics_object)
        return physics_object

    def preallocate_trajectories(self, n_steps):
        for obj in self.objects:
            obj.xyz_trajectory = np.zeros((n_steps + 1, 3))
            obj.euler_angles_trajectory = np.zeros((n_steps + 1, 3))
        self.step = 0
        self.n_steps = n_steps

    def step_simulation(self):
        pybullet.stepSimulation(self.physics_client_id)
        self.step += 1

    def write_step_to_trajectories(self):
        for obj in self.objects:
            position, orientation = pybullet.getBasePositionAndOrientation(obj.bullet_object_id)
            obj.xyz_trajectory[self.step] = position
            obj.euler_angles_trajectory[self.step] = pybullet.getEulerFromQuaternion(orientation)

    def record_contacts(self, contact_force_record_threshold=0, max_n_contacts=float("inf")):
        contacts = [ContactPoint(contact, self.step) for contact in
                    pybullet.getContactPoints(physicsClientId=self.physics_client_id)
                    if contact[9] > contact_force_record_threshold]
        for contact in contacts:
            for id in (contact.body_unique_id_a, contact.body_unique_id_b):
                obj_contacts = self.objects_by_bullet_id[id].contacts
                if len(obj_contacts) < max_n_contacts:
                    obj_contacts.append(contact)

    def simulate(self, n_steps, record_contacts=False, contact_force_record_threshold=0):
        self.preallocate_trajectories(n_steps)

        for i in range(n_steps):
            self.step_simulation()
            self.write_step_to_trajectories()

            if record_contacts:
                self.record_contacts(contact_force_record_threshold)

    def set_keyframes(self, sub_step=1, as_samples=False):
        n_frames = int(self.n_steps / sub_step) + 1
        frames = np.arange(n_frames)
        for obj in self.objects:
            obj.bpy_object.animation_data_create()
            action = bpy.data.actions.new(obj.bpy_object.name + "_act")
            obj.bpy_object.animation_data.action = action
            for i in range(3):
                fcurve = action.fcurves.new("location", index=i)
                fcurve.keyframe_points.add(n_frames)
                frame_data = np.column_stack((frames, obj.xyz_trajectory[::sub_step, i]))
                fcurve.keyframe_points.foreach_set("co", frame_data.flatten())
                if as_samples:
                    fcurve.convert_to_samples(0, n_frames)

            for i in range(3):
                fcurve = action.fcurves.new("rotation_euler", index=i)
                fcurve.keyframe_points.add(n_frames)
                frame_data = np.column_stack((frames, obj.euler_angles_trajectory[::sub_step, i]))
                fcurve.keyframe_points.foreach_set("co", frame_data.flatten())
                if as_samples:
                    fcurve.convert_to_samples(0, n_frames)
