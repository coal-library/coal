import copy
import unittest

import coal
import numpy as np
from test_case import TestCase


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


def colliding_spheres():
    request = coal.CollisionRequest()
    result = coal.CollisionResult()
    coal.collide(
        coal.Sphere(1.0),
        coal.Transform3s.Identity(),
        coal.Sphere(1.0),
        coal.Transform3s(np.array([1.0, 0.0, 0.0])),
        request,
        result,
    )
    return result


class TestCopy(TestCase):
    def assertIndependentCopies(self, obj, read, edit):
        """`copy()`, `copy.copy` and `copy.deepcopy` return an object of the same
        class, equal to `obj`, that can be edited without changing `obj`."""
        copiers = {
            "copy()": lambda o: o.copy(),
            "copy.copy": copy.copy,
            "copy.deepcopy": copy.deepcopy,
        }
        for name, copier in copiers.items():
            with self.subTest(cls=type(obj).__name__, copier=name):
                before = copy.deepcopy(read(obj))
                other = copier(obj)
                self.assertIs(type(other), type(obj))
                self.assertIsNot(other, obj)
                if type(obj).__eq__ is not object.__eq__:
                    self.assertTrue(other == obj)
                edit(other)
                self.assertFalse(np.array_equal(read(other), before))
                self.assertTrue(np.array_equal(read(obj), before))

    def test_requests(self):
        def set_security_margin(request):
            request.security_margin = 0.5

        self.assertIndependentCopies(
            coal.CollisionRequest(),
            lambda r: r.security_margin,
            set_security_margin,
        )

        def set_max_num_patch(request):
            request.max_num_patch = 42

        self.assertIndependentCopies(
            coal.ContactPatchRequest(),
            lambda r: r.max_num_patch,
            set_max_num_patch,
        )

        def set_abs_err(request):
            request.abs_err = 0.5

        self.assertIndependentCopies(
            coal.DistanceRequest(), lambda r: r.abs_err, set_abs_err
        )

    def test_results(self):
        result = colliding_spheres()
        self.assertTrue(result.isCollision())
        self.assertIndependentCopies(
            result, lambda r: r.numContacts(), lambda r: r.clear()
        )

        def set_penetration_depth(contact):
            contact.penetration_depth = 42.0

        self.assertIndependentCopies(
            result.getContact(0),
            lambda c: c.penetration_depth,
            set_penetration_depth,
        )

    def test_shapes(self):
        def set_radius(sphere):
            sphere.radius = 2.0

        self.assertIndependentCopies(coal.Sphere(1.0), lambda s: s.radius, set_radius)

        def set_half_side(box):
            box.halfSide = np.array([4.0, 5.0, 6.0])

        self.assertIndependentCopies(
            coal.Box(1.0, 2.0, 3.0), lambda b: b.halfSide, set_half_side
        )

    def test_convex(self):
        # copy() and copy.copy use the copy constructor, which shares the points of
        # a Convex. copy.deepcopy and clone() deep copy them.
        convex = tetahedron()
        for copier in (lambda o: o.copy(), copy.copy):
            other = copier(convex)
            self.assertIs(type(other), type(convex))
            self.assertTrue(other == convex)
            other.points(0)[:] = [5.0, 5.0, 5.0]
            self.assertTrue(np.array_equal(convex.points(0), [5.0, 5.0, 5.0]))
            convex.points(0)[:] = [0.0, 0.0, 0.0]

        for copier in (copy.deepcopy, lambda o: o.clone()):
            other = copier(convex)
            self.assertIs(type(other), type(convex))
            self.assertTrue(other == convex)
            other.points(0)[:] = [5.0, 5.0, 5.0]
            self.assertTrue(np.array_equal(convex.points(0), [0.0, 0.0, 0.0]))

    def test_transform(self):
        def set_translation(transform):
            transform.setTranslation(np.array([1.0, 2.0, 3.0]))

        self.assertIndependentCopies(
            coal.Transform3s.Identity(),
            lambda t: t.getTranslation(),
            set_translation,
        )

    def test_aabb(self):
        def set_min(aabb):
            aabb.min_ = np.array([-5.0, -5.0, -5.0])

        self.assertIndependentCopies(
            coal.AABB(np.zeros(3), np.ones(3)), lambda a: a.min_, set_min
        )

    def test_std_vectors(self):
        points = coal.StdVec_Vec3s()
        points.append(np.zeros(3))
        self.assertIndependentCopies(points, len, lambda v: v.append(np.ones(3)))

        contacts = colliding_spheres().getContacts()
        self.assertIndependentCopies(contacts, len, lambda v: v.clear())

    def test_bvh(self):
        bvh = coal.BVHModelOBBRSS()
        bvh.beginModel(1, 3)
        bvh.addTriangle(np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))
        bvh.endModel()

        def move_vertex(model):
            model.vertices()[0] = [5.0, 5.0, 5.0]

        self.assertIndependentCopies(bvh, lambda m: m.vertices(), move_vertex)


if __name__ == "__main__":
    unittest.main()
