import unittest
from test_case import TestCase
import coal
import numpy as np


class TestGeometricShapes(TestCase):
    def test_capsule(self):
        capsule = coal.Capsule(1.0, 2.0)
        self.assertIsInstance(capsule, coal.Capsule)
        self.assertIsInstance(capsule, coal.ShapeBase)
        self.assertIsInstance(capsule, coal.CollisionGeometry)
        self.assertEqual(capsule.getNodeType(), coal.NODE_TYPE.GEOM_CAPSULE)
        self.assertEqual(capsule.radius, 1.0)
        self.assertEqual(capsule.halfLength, 1.0)
        capsule.radius = 3.0
        capsule.halfLength = 4.0
        self.assertEqual(capsule.radius, 3.0)
        self.assertEqual(capsule.halfLength, 4.0)
        com = capsule.computeCOM()
        self.assertApprox(com, np.zeros(3))
        V = capsule.computeVolume()
        V_cylinder = capsule.radius * capsule.radius * np.pi * 2.0 * capsule.halfLength
        V_sphere = 4.0 * np.pi / 3 * capsule.radius**3
        V_ref = V_cylinder + V_sphere
        self.assertApprox(V, V_ref)
        I0 = capsule.computeMomentofInertia()
        Iz_cylinder = V_cylinder * capsule.radius**2 / 2.0
        Iz_sphere = 0.4 * V_sphere * capsule.radius * capsule.radius
        Iz_ref = Iz_cylinder + Iz_sphere
        Ix_cylinder = (
            V_cylinder * (3 * capsule.radius**2 + 4 * capsule.halfLength**2) / 12.0
        )
        V_hemi = 0.5 * V_sphere  # volume of hemisphere
        I0x_hemi = 0.5 * Iz_sphere  # inertia of hemisphere w.r.t. origin
        com_hemi = 3.0 * capsule.radius / 8.0  # CoM of hemisphere w.r.t. origin
        Icx_hemi = (
            I0x_hemi - V_hemi * com_hemi * com_hemi
        )  # inertia of hemisphere w.r.t. CoM
        Ix_hemi = (
            Icx_hemi + V_hemi * (capsule.halfLength + com_hemi) ** 2
        )  # inertia of hemisphere w.r.t. tip of cylinder
        Ix_ref = Ix_cylinder + 2 * Ix_hemi  # total inertia of capsule
        I0_ref = np.diag([Ix_ref, Ix_ref, Iz_ref])
        self.assertApprox(I0, I0_ref)
        Ic = capsule.computeMomentofInertiaRelatedToCOM()
        self.assertApprox(Ic, I0_ref)

    def test_box1(self):
        box = coal.Box(np.array([1.0, 2.0, 3.0]))
        self.assertIsInstance(box, coal.Box)
        self.assertIsInstance(box, coal.ShapeBase)
        self.assertIsInstance(box, coal.CollisionGeometry)
        self.assertEqual(box.getNodeType(), coal.NODE_TYPE.GEOM_BOX)
        self.assertTrue(np.array_equal(box.halfSide, np.array([0.5, 1.0, 1.5])))
        box.halfSide = np.array([4.0, 5.0, 6.0])
        self.assertTrue(np.array_equal(box.halfSide, np.array([4.0, 5.0, 6.0])))
        com = box.computeCOM()
        self.assertApprox(com, np.zeros(3))
        V = box.computeVolume()
        x = float(2 * box.halfSide[0])
        y = float(2 * box.halfSide[1])
        z = float(2 * box.halfSide[2])
        V_ref = x * y * z
        self.assertApprox(V, V_ref)
        I0 = box.computeMomentofInertia()
        Ix = V_ref * (y * y + z * z) / 12.0
        Iy = V_ref * (x * x + z * z) / 12.0
        Iz = V_ref * (y * y + x * x) / 12.0
        I0_ref = np.diag([Ix, Iy, Iz])
        self.assertApprox(I0, I0_ref)
        Ic = box.computeMomentofInertiaRelatedToCOM()
        self.assertApprox(Ic, I0_ref)

    def test_box2(self):
        box = coal.Box(1.0, 2.0, 3)
        self.assertIsInstance(box, coal.Box)
        self.assertIsInstance(box, coal.ShapeBase)
        self.assertIsInstance(box, coal.CollisionGeometry)
        self.assertEqual(box.getNodeType(), coal.NODE_TYPE.GEOM_BOX)
        self.assertEqual(box.halfSide[0], 0.5)
        self.assertEqual(box.halfSide[1], 1.0)
        self.assertEqual(box.halfSide[2], 1.5)

    def test_sphere(self):
        sphere = coal.Sphere(1.0)
        self.assertIsInstance(sphere, coal.Sphere)
        self.assertIsInstance(sphere, coal.ShapeBase)
        self.assertIsInstance(sphere, coal.CollisionGeometry)
        self.assertEqual(sphere.getNodeType(), coal.NODE_TYPE.GEOM_SPHERE)
        self.assertEqual(sphere.radius, 1.0)
        sphere.radius = 2.0
        self.assertEqual(sphere.radius, 2.0)
        com = sphere.computeCOM()
        self.assertApprox(com, np.zeros(3))
        V = sphere.computeVolume()
        V_ref = 4.0 * np.pi / 3 * sphere.radius**3
        self.assertApprox(V, V_ref)
        I0 = sphere.computeMomentofInertia()
        I0_ref = 0.4 * V_ref * sphere.radius * sphere.radius * np.identity(3)
        self.assertApprox(I0, I0_ref)
        Ic = sphere.computeMomentofInertiaRelatedToCOM()
        self.assertApprox(Ic, I0_ref)

    def test_cylinder(self):
        cylinder = coal.Cylinder(1.0, 2.0)
        self.assertIsInstance(cylinder, coal.Cylinder)
        self.assertIsInstance(cylinder, coal.ShapeBase)
        self.assertIsInstance(cylinder, coal.CollisionGeometry)
        self.assertEqual(cylinder.getNodeType(), coal.NODE_TYPE.GEOM_CYLINDER)
        self.assertEqual(cylinder.radius, 1.0)
        self.assertEqual(cylinder.halfLength, 1.0)
        cylinder.radius = 3.0
        cylinder.halfLength = 4.0
        self.assertEqual(cylinder.radius, 3.0)
        self.assertEqual(cylinder.halfLength, 4.0)
        com = cylinder.computeCOM()
        self.assertApprox(com, np.zeros(3))
        V = cylinder.computeVolume()
        V_ref = cylinder.radius * cylinder.radius * np.pi * 2.0 * cylinder.halfLength
        self.assertApprox(V, V_ref)
        I0 = cylinder.computeMomentofInertia()
        Ix_ref = V_ref * (3 * cylinder.radius**2 + 4 * cylinder.halfLength**2) / 12.0
        Iz_ref = V_ref * cylinder.radius**2 / 2.0
        I0_ref = np.diag([Ix_ref, Ix_ref, Iz_ref])
        self.assertApprox(I0, I0_ref)
        Ic = cylinder.computeMomentofInertiaRelatedToCOM()
        self.assertApprox(Ic, I0_ref)

    def test_cone(self):
        cone = coal.Cone(1.0, 2.0)
        self.assertIsInstance(cone, coal.Cone)
        self.assertIsInstance(cone, coal.ShapeBase)
        self.assertIsInstance(cone, coal.CollisionGeometry)
        self.assertEqual(cone.getNodeType(), coal.NODE_TYPE.GEOM_CONE)
        self.assertEqual(cone.radius, 1.0)
        self.assertEqual(cone.halfLength, 1.0)
        cone.radius = 3.0
        cone.halfLength = 4.0
        self.assertEqual(cone.radius, 3.0)
        self.assertEqual(cone.halfLength, 4.0)
        com = cone.computeCOM()
        self.assertApprox(com, np.array([0.0, 0.0, -0.5 * cone.halfLength]))
        V = cone.computeVolume()
        V_ref = np.pi * cone.radius**2 * 2.0 * cone.halfLength / 3.0
        self.assertApprox(V, V_ref)
        I0 = cone.computeMomentofInertia()
        Ix_ref = V_ref * (3.0 / 20.0 * cone.radius**2 + 0.4 * cone.halfLength**2)
        Iz_ref = 0.3 * V_ref * cone.radius**2
        I0_ref = np.diag([Ix_ref, Ix_ref, Iz_ref])
        self.assertApprox(I0, I0_ref)
        Ic = cone.computeMomentofInertiaRelatedToCOM()
        Icx_ref = V_ref * 3.0 / 20.0 * (cone.radius**2 + cone.halfLength**2)
        Ic_ref = np.diag([Icx_ref, Icx_ref, Iz_ref])
        self.assertApprox(Ic, Ic_ref)

    def test_BVH(self):
        bvh = coal.BVHModelOBBRSS()
        self.assertEqual(bvh.num_vertices, 0)
        self.assertEqual(bvh.vertices().shape, (0, 3))

    def test_convex(self):
        verts = coal.StdVec_Vec3s()
        faces = coal.StdVec_Triangle()
        verts.extend(
            [
                np.array([0, 0, 0]),
                np.array([0, 1, 0]),
                np.array([1, 0, 0]),
            ]
        )
        faces.append(coal.Triangle(0, 1, 2))
        coal.Convex(verts, faces)

        verts.append(np.array([0, 0, 1]))
        try:
            coal.Convex.convexHull(verts, False, None)
            qhullAvailable = True
        except Exception as e:
            self.assertIn(
                "Library built without qhull. Cannot build object of this type.", str(e)
            )
            qhullAvailable = False

        if qhullAvailable:
            coal.Convex.convexHull(verts, False, "")
            coal.Convex.convexHull(verts, True, "")

            try:
                coal.Convex.convexHull(verts[:3], False, None)
            except Exception as e:
                self.assertIn(
                    "You shouldn't use this function with less than 4 points.", str(e)
                )


if __name__ == "__main__":
    unittest.main()
