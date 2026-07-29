import unittest
from test_case import TestCase
import coal

import pickle
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


class TestGeometryPickling(TestCase):
    def pickling(self, obj):
        with open("save.p", "wb") as f:
            pickle.dump(obj, f)
        with open("save.p", "rb") as f:
            obj2 = pickle.load(f)

        self.assertTrue(obj == obj2)

    def test_all_shapes(self):
        box = coal.Box(1.0, 2.0, 3.0)
        self.pickling(box)

        trans = coal.Transform3s(np.array((0, 0, 0)))
        self.pickling(trans)

        sphere = coal.Sphere(1.0)
        self.pickling(sphere)

        ellipsoid = coal.Ellipsoid(1.0, 2.0, 3.0)
        self.pickling(ellipsoid)

        convex = tetahedron()
        self.pickling(convex)

        capsule = coal.Capsule(1.0, 2.0)
        self.pickling(capsule)

        cylinder = coal.Cylinder(1.0, 2.0)
        self.pickling(cylinder)

        plane = coal.Plane(np.array([0.0, 0.0, 1.0]), 2.0)
        self.pickling(plane)

        half_space = coal.Halfspace(np.array([0.0, 0.0, 1.0]), 2.0)
        self.pickling(half_space)


if __name__ == "__main__":
    unittest.main()
