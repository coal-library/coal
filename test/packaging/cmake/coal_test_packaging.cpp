#include <coal/math/transform.h>
#include <coal/mesh_loader/loader.h>
#include <coal/BVH/BVH_model.h>
#include <coal/collision.h>
#include <coal/collision_data.h>
#include <hpp/fcl/broadphase/broadphase.h>
#include <iostream>
#include <memory>

int main() {
  // Create the coal shapes.
  // Coal supports many primitive shapes: boxes, spheres, capsules, cylinders,
  // ellipsoids, cones, planes, halfspace and convex meshes (i.e. convex hulls
  // of clouds of points). It also supports BVHs (bounding volumes hierarchies),
  // height-fields and octrees.
  auto shape1 = std::make_shared<coal::Ellipsoid>(0.7, 1.0, 0.8);
  auto shape2 = std::make_shared<coal::Ellipsoid>(0.7, 1.0, 0.8);

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
  // If the distance between the shapes is less than the security margin, the
  // shapes will be considered in collision. Setting a positive security margin
  // can be usefull in motion planning, i.e to prevent shapes from getting too
  // close to one another. In physics simulation, allowing a negative security
  // margin may be usefull to stabilize the simulation.
  coal::CollisionRequest col_req;
  col_req.security_margin = 1e-1;
  // A collision result stores the result of the collision test (signed distance
  // between the shapes, witness points location, normal etc.)
  coal::CollisionResult col_res;

  // Collision call
  coal::collide(shape1.get(), T1, shape2.get(), T2, col_req, col_res);

  // We can access the collision result once it has been populated
  std::cout << "Collision? " << col_res.isCollision() << "\n";
  if (col_res.isCollision()) {
    coal::Contact contact = col_res.getContact(0);
    // The penetration depth does **not** take into account the security margin.
    // Consequently, the penetration depth is the true signed distance which
    // separates the shapes. To have the distance which takes into account the
    // security margin, we can simply add the two together.
    std::cout << "Penetration depth: " << contact.penetration_depth << "\n";
    std::cout << "Distance between the shapes including the security margin: "
              << contact.penetration_depth + col_req.security_margin << "\n";
    std::cout << "Witness point on shape1: "
              << contact.nearest_points[0].transpose() << "\n";
    std::cout << "Witness point on shape2: "
              << contact.nearest_points[1].transpose() << "\n";
    std::cout << "Normal: " << contact.normal.transpose() << "\n";
  }

  // Before calling another collision test, it is important to clear the
  // previous results stored in the collision result.
  col_res.clear();

  return 0;
}