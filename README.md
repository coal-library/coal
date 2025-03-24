# Coal — An extension of the Flexible Collision Library

<p align="center">
  <a href="https://gitlab.laas.fr/coal-library/coal/commits/devel/"><img src="https://gitlab.laas.fr/coal-library/coal/badges/devel/pipeline.svg" alt="Pipeline status"/></a>
  <a href="https://gepettoweb.laas.fr/doc/coal-library/coal/devel/doxygen-html/"><img src="https://img.shields.io/badge/docs-online-brightgreen" alt="Documentation"/></a>
  <a href="https://gepettoweb.laas.fr/doc/coal-library/coal/devel/coverage/"><img src="https://gepgitlab.laas.fr/coal-library/badges/devel/coverage.svg?job=doc-coverage" alt="Coverage report"/></a>
  <a href="https://anaconda.org/conda-forge/coal"><img src="https://img.shields.io/conda/dn/conda-forge/coal.svg" alt="Conda Downloads"/></a>
  <a href="https://anaconda.org/conda-forge/coal"><img src="https://img.shields.io/conda/vn/conda-forge/coal.svg" alt="Conda Version"/></a>
  <a href="https://badge.fury.io/py/coal"><img src="https://badge.fury.io/py/coal.svg" alt="PyPI version"></a>
  <a href="https://github.com/psf/black"><img alt="black" src="https://img.shields.io/badge/code%20style-black-000000.svg"></a>
  <a href="https://github.com/astral-sh/ruff"><img alt="ruff" src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json"></a>
</p>

[FCL](https://github.com/flexible-collision-library/fcl) was forked in 2015, creating a new project called HPP-FCL.
Since then, a large part of the code has been rewritten or removed (unused and untested code), and new features have been introduced (see below).
Due to these major changes, it was decided in 2024 to rename the HPP-FCL project to **Coal**.

If you use **Coal** in your projects and research papers, we would appreciate it if you would [cite it](https://raw.githubusercontent.com/coal-library/coal/devel/CITATION.bib).

## New features

Compared to the original [FCL](https://github.com/flexible-collision-library/fcl) library, the main new features are:
- dedicated and efficient implementations of the GJK and the EPA algorithms (we do not rely on [libccd](https://github.com/danfis/libccd))
- the support of safety margins for collision detection
- an accelerated version of collision detection *à la Nesterov*, which leads to increased performance (up to a factor of 2). More details are available in this [paper](https://hal.archives-ouvertes.fr/hal-03662157/)
- the computation of a lower bound of the distance between two objects when collision checking is performed, and no collision is found
- the implementation of Python bindings for easy code prototyping
- the support of new geometries such as height fields, capsules, ellipsoids, etc.
- enhance reliability with the fix of a myriad of bugs
- efficient computation of **contact points** and **contact patches** between objects
- full support of object serialization via Boost.Serialization

Note: the broad phase was reintroduced by [Justin Carpentier](https://github.com/jcarpent) in 2022, based on the FCL version 0.7.0.

This project is now used in several robotics frameworks such as [Pinocchio](https://github.com/stack-of-tasks/pinocchio), an open-source library which implements efficient and versatile rigid-body dynamics algorithms, the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc), an open-source library for Motion and Manipulation Planning. **Coal** has recently also been used to develop [Simple](https://github.com/Simple-Robotics/Simple), a new (differentiable) and efficient simulator for robotics and beyond.

## A high-performance library

Unlike the original FCL library, Coal implements the well-established [GJK algorithm](https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm) and [its variants](https://hal.archives-ouvertes.fr/hal-03662157/) for collision detection and distance computation. These implementations lead to state-of-the-art performance, as shown in the figures below.

On the one hand, we have benchmarked Coal against major state-of-the-art software alternatives:

1. the [Bullet simulator](https://github.com/bulletphysics/bullet3),
2. the original [FCL library](https://github.com/flexible-collision-library/fcl) (used in the [Drake framework]()),
3. the [libccd library](https://github.com/danfis/libccd) (used in [MuJoCo](http://mujoco.org/)).

The results are depicted in the following figure, which notably shows that the accelerated variants of GJK largely outperform by a large margin (from 5x up to 15x times faster).
Please notice that the y-axis is in log scale.

<p align="center">
  <img src="./doc/images/coal-vs-the-rest-of-the-world.png" width="600" alt="Coal vs the rest of the world" align="center"/>
</p>

On the other hand, why do we care about dedicated collision detection solvers like GJK for the narrow phase? Why can't we simply formulate the collision detection problem as a quadratic problem and call an off-the-shelf optimization solver like [ProxQP](https://github.com/Simple-Robotics/proxsuite))? Here is why:

<p align="center">
  <img src="./doc/images/coal-performances.jpg" width="600" alt="Coal vs generic QP solvers" align="center"/>
</p>

One can observe that GJK-based approaches largely outperform solutions based on classic optimization solvers (e.g., QP solver like [ProxQP](https://github.com/Simple-Robotics/proxsuite)), notably for large geometries composed of tens or hundreds of vertices.

## Open-source projects relying on Coal

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives.
- [IfcOpenShell](https://github.com/IfcOpenShell/IfcOpenShell) Open source IFC library and geometry engine.
- [Crocoddyl](https://github.com/loco-3d/crocoddyl) A software to realize model predictive control for complex robotics platforms.
- [TSID](https://github.com/stack-of-tasks/tsid/) A software that implements a Task Space Inverse Dynamics QP.
- [HPP](https://humanoid-path-planner.github.io/hpp-doc/) A SDK that implements motion planners for humanoids and other robots.
- [Jiminy](https://github.com/duburcqa/jiminy) A simulator based on Pinocchio.
- [ocs2](https://github.com/leggedrobotics/ocs2) A toolbox for Optimal Control for Switched Systems (OCS2)

## Installation

### Conda

Coal can be installed from the [conda-forge channel](https://anaconda.org/conda-forge/coal):

```bash
conda install coal -c conda-forge
```

## Build

You can find build instruction [here](./development/build.md).

## C++ example
Both the C++ library and the python bindings can be installed as simply as `conda -c conda-forge install coal`.
The `.so` library, include files and python bindings will then be installed under `$CONDA_PREFIX/lib`, `$CONDA_PREFIX/include` and `$CONDA_PREFIX/lib/python3.XX/site-packages`.

Here is an example of using Coal in C++:
```cpp
#include "coal/math/transform.h"
#include "coal/mesh_loader/loader.h"
#include "coal/BVH/BVH_model.h"
#include "coal/collision.h"
#include "coal/collision_data.h"
#include <iostream>
#include <memory>

// Function to load a convex mesh from a `.obj`, `.stl` or `.dae` file.
//
// This function imports the object inside the file as a BVHModel, i.e. a point cloud
// which is hierarchically transformed into a tree of bounding volumes.
// The leaves of this tree are the individual points of the point cloud
// stored in the `.obj` file.
// This BVH can then be used for collision detection.
//
// For better computational efficiency, we sometimes prefer to work with
// the convex hull of the point cloud. This insures that the underlying object
// is convex and thus very fast collision detection algorithms such as
// GJK or EPA can be called with this object.
// Consequently, after creating the BVH structure from the point cloud, this function
// also computes its convex hull.
std::shared_ptr<coal::ConvexBase> loadConvexMesh(const std::string& file_name) {
  coal::NODE_TYPE bv_type = coal::BV_AABB;
  coal::MeshLoader loader(bv_type);
  coal::BVHModelPtr_t bvh = loader.load(file_name);
  bvh->buildConvexHull(true, "Qt");
  return bvh->convex;
}

int main() {
  // Create the coal shapes.
  // Coal supports many primitive shapes: boxes, spheres, capsules, cylinders, ellipsoids, cones, planes,
  // halfspace and convex meshes (i.e. convex hulls of clouds of points).
  // It also supports BVHs (bounding volumes hierarchies), height-fields and octrees.
  std::shared_ptr<coal::Ellipsoid> shape1 = std::make_shared<coal::Ellipsoid>(0.7, 1.0, 0.8);
  std::shared_ptr<coal::ConvexBase> shape2 = loadConvexMesh("../path/to/mesh/file.obj");

  // Define the shapes' placement in 3D space
  coal::Transform3s T1;
  T1.setQuatRotation(coal::Quaternion3f::UnitRandom());
  T1.setTranslation(coal::Vec3s::Random());
  coal::Transform3s T2 = coal::Transform3s::Identity();
  T2.setQuatRotation(coal::Quaternion3f::UnitRandom());
  T2.setTranslation(coal::Vec3s::Random());

  // Define collision requests and results.
  //
  // The collision request allows to set parameters for the collision pair.
  // For example, we can set a positive or negative security margin.
  // If the distance between the shapes is less than the security margin, the shapes
  // will be considered in collision.
  // Setting a positive security margin can be usefull in motion planning,
  // i.e to prevent shapes from getting too close to one another.
  // In physics simulation, allowing a negative security margin may be usefull to stabilize the simulation.
  coal::CollisionRequest col_req;
  col_req.security_margin = 1e-1;
  // A collision result stores the result of the collision test (signed distance between the shapes,
  // witness points location, normal etc.)
  coal::CollisionResult col_res;

  // Collision call
  coal::collide(shape1.get(), T1, shape2.get(), T2, col_req, col_res);

  // We can access the collision result once it has been populated
  std::cout << "Collision? " << col_res.isCollision() << "\n";
  if (col_res.isCollision()) {
    coal::Contact contact = col_res.getContact(0);
    // The penetration depth does **not** take into account the security margin.
    // Consequently, the penetration depth is the true signed distance which separates the shapes.
    // To have the distance which takes into account the security margin, we can simply add the two together.
    std::cout << "Penetration depth: " << contact.penetration_depth << "\n";
    std::cout << "Distance between the shapes including the security margin: " << contact.penetration_depth + col_req.security_margin << "\n";
    std::cout << "Witness point on shape1: " << contact.nearest_points[0].transpose() << "\n";
    std::cout << "Witness point on shape2: " << contact.nearest_points[1].transpose() << "\n";
    std::cout << "Normal: " << contact.normal.transpose() << "\n";
  }

  // Before calling another collision test, it is important to clear the previous results stored in the collision result.
  col_res.clear();

  return 0;
}
```

## Python example
Here is the C++ example from above translated in python using the python bindings of Coal:
```python
import numpy as np
import coal
# Optional:
# The Pinocchio library is a rigid body algorithms library and has a handy SE3 module.
# It can be installed as simply as `conda -c conda-forge install pinocchio`.
# Installing pinocchio also installs coal.
import pinocchio as pin

def loadConvexMesh(file_name: str):
    loader = coal.MeshLoader()
    bvh: coal.BVHModelBase = loader.load(file_name)
    bvh.buildConvexHull(True, "Qt")
    return bvh.convex

if __name__ == "__main__":
    # Create coal shapes
    shape1 = coal.Ellipsoid(0.7, 1.0, 0.8)
    shape2 = loadConvexMesh("../path/to/mesh/file.obj")

    # Define the shapes' placement in 3D space
    T1 = coal.Transform3s()
    T1.setTranslation(pin.SE3.Random().translation)
    T1.setRotation(pin.SE3.Random().rotation)
    T2 = coal.Transform3s();
    # Using np arrays also works
    T1.setTranslation(np.random.rand(3))
    T2.setRotation(pin.SE3.Random().rotation)

    # Define collision requests and results
    col_req = coal.CollisionRequest()
    col_res = coal.CollisionResult()

    # Collision call
    coal.collide(shape1, T1, shape2, T2, col_req, col_res)

    # Accessing the collision result once it has been populated
    print("Is collision? ", {col_res.isCollision()})
    if col_res.isCollision():
        contact: coal.Contact = col_res.getContact(0)
        print("Penetration depth: ", contact.penetration_depth)
        print("Distance between the shapes including the security margin: ", contact.penetration_depth + col_req.security_margin)
        print("Witness point shape1: ", contact.getNearestPoint1())
        print("Witness point shape2: ", contact.getNearestPoint2())
        print("Normal: ", contact.normal)

    # Before running another collision call, it is important to clear the old one
    col_res.clear()
```

## Acknowledgments

The development of **Coal** is actively supported by the [Gepetto team](http://projects.laas.fr/gepetto/) [@LAAS-CNRS](http://www.laas.fr), the [Willow team](https://www.di.ens.fr/willow/) [@INRIA](http://www.inria.fr) and, to some extent, [Eureka Robotics](https://eurekarobotics.com/).
