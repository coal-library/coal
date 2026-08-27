import unittest
import numpy as np


class TestCase(unittest.TestCase):
    def assertApprox(self, a, b, epsilon=1e-6):
        return self.assertTrue(np.allclose(a, b, epsilon), "%s !~= %s" % (a, b))
