import coal
import numpy as np

sphere = coal.Sphere(0.5)
sphere_obj = coal.CollisionObject(sphere)

M_sphere = coal.Transform3s.Identity()
M_sphere.setTranslation(np.array([-0.6, 0.0, 0.0]))
sphere_obj.setTransform(M_sphere)

box = coal.Box(np.array([0.5, 0.5, 0.5]))
box_obj = coal.CollisionObject(box)

M_box = coal.Transform3s.Identity()
M_box.setTranslation(np.array([-0.6, 0.0, 0.0]))
box_obj.setTransform(M_box)

collision_manager = coal.DynamicAABBTreeCollisionManager()
collision_manager.registerObject(sphere_obj)
collision_manager.registerObject(box_obj)

assert collision_manager.size() == 2

collision_manager.setup()

# Perform collision detection
callback = coal.CollisionCallBackDefault()
collision_manager.collide(sphere_obj, callback)

assert callback.data.result.numContacts() == 1
