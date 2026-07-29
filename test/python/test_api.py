import unittest
from test_case import TestCase
import coal

import numpy as np


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


if __name__ == "__main__":
    unittest.main()
