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

    def test_std_vectors(self):
        vec = coal.StdVec_Vec3s()
        vec.append(np.array((0.0, 0.0, 0.0)))
        vec.append(np.array((1.0, 0.0, 0.0)))
        self.pickling(vec)

        tri = coal.StdVec_Triangle()
        tri.append(coal.Triangle(0, 1, 2))
        tri.append(coal.Triangle(0, 1, 3))
        self.pickling(tri)

        tri16 = coal.StdVec_Triangle16()
        tri16.append(coal.Triangle16(0, 1, 2))
        self.pickling(tri16)

        collision_request = coal.CollisionRequest()
        self.pickling(collision_request)

        collision_request_vec = coal.StdVec_CollisionRequest()
        collision_request_vec.append(coal.CollisionRequest())
        self.pickling(collision_request_vec)

        collision_result = coal.CollisionResult()
        self.pickling(collision_result)

        collision_result_vec = coal.StdVec_CollisionResult()
        collision_result_vec.append(coal.CollisionResult())
        self.pickling(collision_result_vec)

        distance_request = coal.DistanceRequest()
        self.pickling(distance_request)

        distance_request_vec = coal.StdVec_DistanceRequest()
        distance_request_vec.append(coal.DistanceRequest())
        self.pickling(distance_request_vec)

        distance_result = coal.DistanceResult()
        self.pickling(distance_result)

        distance_result_vec = coal.StdVec_DistanceResult()
        distance_result_vec.append(coal.DistanceResult())
        self.pickling(distance_result_vec)

        contact = coal.Contact()
        self.pickling(contact)

        contact_vec = coal.StdVec_Contact()
        contact_vec.append(coal.Contact())
        self.pickling(contact_vec)

        patch = coal.ContactPatch()
        self.pickling(patch)

        patch_vec = coal.StdVec_ContactPatch()
        patch_vec.append(coal.ContactPatch())
        self.pickling(patch_vec)

        patch_request = coal.ContactPatchRequest()
        self.pickling(patch_request)

        patch_request_vec = coal.StdVec_ContactPatchRequest()
        patch_request_vec.append(coal.ContactPatchRequest())
        self.pickling(patch_request_vec)

        patch_result = coal.ContactPatchResult()
        self.pickling(patch_result)

        patch_result_vec = coal.StdVec_ContactPatchResult()
        patch_result_vec.append(coal.ContactPatchResult())
        self.pickling(patch_result_vec)


if __name__ == "__main__":
    unittest.main()
