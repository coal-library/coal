import unittest
from test_case import TestCase
import coal

import numpy as np


def tetahedron():
    pts = coal.StdVec_Vec3s()
    pts.append(np.array((0, 0, 0)))
    pts.append(np.array((0, 1, 0)))
    pts.append(np.array((1, 0, 0)))
    pts.append(np.array((0, 0, 1)))
    tri = coal.StdVec_Triangle()
    tri.append(coal.Triangle(0, 1, 2))
    tri.append(coal.Triangle(0, 1, 3))
    tri.append(coal.Triangle(0, 2, 3))
    tri.append(coal.Triangle(1, 2, 3))
    return coal.Convex(pts, tri)


class TestMainAPI(TestCase):
    def test_convex_halfspace(self):
        convex = tetahedron()
        halfspace = coal.Halfspace(np.array((0, 0, 1)), 0)

        req = coal.CollisionRequest()
        res = coal.CollisionResult()

        M1 = coal.Transform3s()
        M2 = coal.Transform3s(np.eye(3), np.array([0, 0, -0.1]))
        res.clear()
        coal.collide(convex, M1, halfspace, M2, req, res)
        self.assertFalse(coal.collide(convex, M1, halfspace, M2, req, res))

        M1 = coal.Transform3s()
        M2 = coal.Transform3s(np.eye(3), np.array([0, 0, 0.1]))
        res.clear()
        self.assertTrue(coal.collide(convex, M1, halfspace, M2, req, res))

        M1 = coal.Transform3s()
        M2 = coal.Transform3s(np.eye(3), np.array([0, 0, 2]))
        res.clear()
        self.assertTrue(coal.collide(convex, M1, halfspace, M2, req, res))


if __name__ == "__main__":
    unittest.main()
