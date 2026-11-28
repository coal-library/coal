import unittest

import coal
import numpy as np
from test_case import TestCase

# Grid of GRID_CELLS x GRID_CELLS squares, two triangles each, in the plane
# z = 0 over [-GRID_HALF_WIDTH, GRID_HALF_WIDTH]^2 (metres).
GRID_CELLS = 10
GRID_HALF_WIDTH = 0.4
# Box side (metres): the box holds the whole grid, so every mesh BV overlaps it
# and every triangle is in contact.
BOX_SIDE = 1.0
# Contacts asked for: more than the grid has triangles, so the request is never
# satisfied early and the traversal visits every leaf.
MAX_CONTACTS = 1000


def grid_mesh():
    step = 2 * GRID_HALF_WIDTH / GRID_CELLS

    def at(i, j):
        return np.array([-GRID_HALF_WIDTH + i * step, -GRID_HALF_WIDTH + j * step, 0.0])

    mesh = coal.BVHModelOBBRSS()
    mesh.beginModel(2 * GRID_CELLS * GRID_CELLS, 6 * GRID_CELLS * GRID_CELLS)
    for i in range(GRID_CELLS):
        for j in range(GRID_CELLS):
            mesh.addTriangle(at(i, j), at(i + 1, j), at(i + 1, j + 1))
            mesh.addTriangle(at(i, j), at(i + 1, j + 1), at(i, j + 1))
    mesh.endModel()
    return mesh


def grid_collides_with_box(mesh):
    box = coal.Box(BOX_SIDE, BOX_SIDE, BOX_SIDE)
    req = coal.CollisionRequest()
    req.num_max_contacts = MAX_CONTACTS
    res = coal.CollisionResult()
    coal.collide(mesh, coal.Transform3s(), box, coal.Transform3s(), req, res)
    return res.isCollision()


def counted(n):
    """The count expected for n traversal tests: n when Coal counts, else 0."""
    return n if coal.traversalStatsEnabled() else 0


class TestTraversalStats(TestCase):
    def test_mesh_box_collide_counts_every_triangle(self):
        # One leaf test per triangle, one BV test per inner node of the mesh's
        # binary tree (n leaves, n - 1 inner nodes).
        mesh = grid_mesh()
        n_tris = mesh.num_tris
        self.assertEqual(n_tris, 2 * GRID_CELLS * GRID_CELLS)

        coal.resetTraversalStats()
        self.assertTrue(grid_collides_with_box(mesh))

        stats = coal.traversalStats()
        self.assertEqual(stats.leaf_tests, counted(n_tris))
        self.assertEqual(stats.bv_tests, counted(n_tris - 1))


if __name__ == "__main__":
    unittest.main()
