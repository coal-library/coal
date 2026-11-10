import unittest

import coal
import numpy as np
from test_case import TestCase


class TestMainAPI(TestCase):
    def test_collision(self):
        capsule = coal.Capsule(1.0, 2.0)
        M1 = coal.Transform3s()
        M2 = coal.Transform3s(np.eye(3), np.array([3, 0, 0]))

        req = coal.CollisionRequest()
        res = coal.CollisionResult()

        self.assertTrue(not coal.collide(capsule, M1, capsule, M2, req, res))

    def test_distance(self):
        capsule = coal.Capsule(1.0, 2.0)
        M1 = coal.Transform3s()
        M2 = coal.Transform3s(np.eye(3), np.array([3, 0, 0]))

        req = coal.DistanceRequest()
        res = coal.DistanceResult()

        self.assertTrue(coal.distance(capsule, M1, capsule, M2, req, res) > 0)

    def test_collision_object_type_tags(self):
        # CollisionObject caches the geometry's type tags rather than
        # forwarding to it, so the setter has to refresh them.
        box = coal.Box(1.0, 2.0, 3.0)
        obj = coal.CollisionObject(box)
        self.assertEqual(obj.getObjectType(), coal.OBJECT_TYPE.OT_GEOM)
        self.assertEqual(obj.getNodeType(), coal.NODE_TYPE.GEOM_BOX)

        # compute_local_aabb is optional in both bindings.
        sphere = coal.Sphere(1.0)
        obj.setCollisionGeometry(sphere)
        self.assertEqual(obj.getObjectType(), coal.OBJECT_TYPE.OT_GEOM)
        self.assertEqual(obj.getNodeType(), coal.NODE_TYPE.GEOM_SPHERE)

        capsule = coal.Capsule(1.0, 2.0)
        obj.setCollisionGeometry(capsule, compute_local_aabb=False)
        self.assertEqual(obj.getObjectType(), coal.OBJECT_TYPE.OT_GEOM)
        self.assertEqual(obj.getNodeType(), coal.NODE_TYPE.GEOM_CAPSULE)


if __name__ == "__main__":
    unittest.main()
