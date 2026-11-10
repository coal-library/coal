//
// Copyright (c) 2026 INRIA
//

#include <benchmark/benchmark.h>

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "coal/data_types.h"
#include "coal/internal/shape_shape_func.h"
#include "coal/math/transform.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes.h"
#include "utility.h"

using namespace coal;

namespace {

const std::size_t num_transforms = 10000;
const Scalar radius1 = Scalar(1.0);
const Scalar radius2 = Scalar(0.5);
const Scalar height = Scalar(2.0);
const Scalar plane_offset1 = Scalar(1.0);
const Scalar plane_offset2 = Scalar(2.0);
const Vec3s plane_normal(Scalar(1.0), Scalar(0.0), Scalar(0.0));
const Vec3s box_side(Scalar(1.0), Scalar(0.5), Scalar(0.25));
const Vec3s ellipsoid_radii(Scalar(0.5), Scalar(0.3), Scalar(0.2));
const Vec3s triangle_a(Scalar(0.0), Scalar(0.0), Scalar(0.0));
const Vec3s triangle_b(Scalar(1.0), Scalar(0.0), Scalar(0.0));
const Vec3s triangle_c(Scalar(0.0), Scalar(1.0), Scalar(0.0));
// Be sure to not generate shape in collision
const Scalar closest_distance(5.);
const Scalar farest_distance(10.);

// One static instance per shape. Only the Sphere-Sphere pair needs two spheres:
// `get_sphere2` is the sphere shared by the other pairs.
const Sphere& get_sphere1() {
  static const Sphere sphere(radius1);
  return sphere;
}

const Sphere& get_sphere2() {
  static const Sphere sphere(radius2);
  return sphere;
}

const Box& get_box() {
  static const Box box(box_side);
  return box;
}

const Capsule& get_capsule() {
  static const Capsule capsule(radius2, height);
  return capsule;
}

const Cone& get_cone() {
  static const Cone cone(radius2, height);
  return cone;
}

const Cylinder& get_cylinder() {
  static const Cylinder cylinder(radius2, height);
  return cylinder;
}

const Ellipsoid& get_ellipsoid() {
  static const Ellipsoid ellipsoid(ellipsoid_radii);
  return ellipsoid;
}

const TriangleP& get_triangle() {
  static const TriangleP triangle(triangle_a, triangle_b, triangle_c);
  return triangle;
}

const Plane& get_plane1() {
  static const Plane plane(plane_normal, plane_offset1);
  return plane;
}

const Plane& get_plane2() {
  static const Plane plane(plane_normal, plane_offset2);
  return plane;
}

const Halfspace& get_halfspace() {
  static const Halfspace halfspace(plane_normal, plane_offset1);
  return halfspace;
}

/// Transforms of the second shape, with the first shape at the origin.
const std::vector<Transform3s>& get_transforms() {
  static const std::vector<Transform3s> transforms = [] {
    std::vector<Transform3s> ret_transforms;
    Scalar extents_6d[] = {closest_distance, closest_distance, closest_distance,
                           farest_distance,  farest_distance,  farest_distance};
    generateRandomTransforms(extents_6d, ret_transforms, num_transforms);
    // Don't rotate to avoid plane or halfspace collision.
    // This should not have any performance impact.
    for (auto& t : ret_transforms) {
      t.rotation().setIdentity();
    }
    return ret_transforms;
  }();
  return transforms;
}

/// Runs the benchmark, calling `compute_distance` for each transform of the
/// second shape, the first shape being at the origin.
/// `compute_distance` takes the two shapes, their transforms, the narrowphase
/// solver, the witness points and the normal, and returns the distance.
template <typename S1, typename S2, typename ComputeDistance>
void run(benchmark::State& state, const S1& o1, const S2& o2,
         const ComputeDistance& compute_distance) {
  const GJKSolver nsolver;
  const std::vector<Transform3s>& transforms = get_transforms();

  const Transform3s tf1 = Transform3s::Identity();

  Vec3s p1, p2, normal;
  Scalar sum = Scalar(0.0);

  for (auto _ : state) {
    for (const Transform3s& tf2 : transforms) {
      sum += compute_distance(o1, tf1, o2, tf2, nsolver, p1, p2, normal);
    }
  }

  benchmark::DoNotOptimize(sum);
  state.SetItemsProcessed(state.iterations() *
                          static_cast<std::int64_t>(num_transforms));
}

/// Distance by GJK only, i.e. the generic implementation, without the
/// specialization for the pair of shapes.
template <typename S1, typename S2>
Scalar gjk_distance(const S1& o1, const Transform3s& tf1, const S2& o2,
                    const Transform3s& tf2, const GJKSolver& nsolver, Vec3s& p1,
                    Vec3s& p2, Vec3s& normal) {
  return nsolver.shapeDistance<S1, S2>(o1, tf1, o2, tf2, false, p1, p2, normal);
}

/// Distance by the specialized implementation for the pair of shapes.
template <typename S1, typename S2>
Scalar specialized_distance(const S1& o1, const Transform3s& tf1, const S2& o2,
                            const Transform3s& tf2, const GJKSolver& nsolver,
                            Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  return internal::ShapeShapeDistance<S1, S2>(&o1, tf1, &o2, tf2, &nsolver,
                                              false, p1, p2, normal);
}

// Pairs with both the generic GJK implementation and a specialization.
void sphere_sphere_gjk(benchmark::State& state) {
  run(state, get_sphere1(), get_sphere2(), gjk_distance<Sphere, Sphere>);
}

void sphere_sphere_specialized(benchmark::State& state) {
  run(state, get_sphere1(), get_sphere2(),
      specialized_distance<Sphere, Sphere>);
}

void box_sphere_gjk(benchmark::State& state) {
  run(state, get_box(), get_sphere2(), gjk_distance<Box, Sphere>);
}

void box_sphere_specialized(benchmark::State& state) {
  run(state, get_box(), get_sphere2(), specialized_distance<Box, Sphere>);
}

void box_box_gjk(benchmark::State& state) {
  run(state, get_box(), get_box(), gjk_distance<Box, Box>);
}

void box_capsule_gjk(benchmark::State& state) {
  run(state, get_box(), get_capsule(), gjk_distance<Box, Capsule>);
}

void box_capsule_specialized(benchmark::State& state) {
  run(state, get_box(), get_capsule(), specialized_distance<Box, Capsule>);
}

void capsule_capsule_gjk(benchmark::State& state) {
  run(state, get_capsule(), get_capsule(), gjk_distance<Capsule, Capsule>);
}

void capsule_capsule_specialized(benchmark::State& state) {
  run(state, get_capsule(), get_capsule(),
      specialized_distance<Capsule, Capsule>);
}

void cylinder_cylinder_gjk(benchmark::State& state) {
  run(state, get_cylinder(), get_cylinder(), gjk_distance<Cylinder, Cylinder>);
}

void cylinder_sphere_gjk(benchmark::State& state) {
  run(state, get_cylinder(), get_sphere2(), gjk_distance<Cylinder, Sphere>);
}

void cylinder_sphere_specialized(benchmark::State& state) {
  run(state, get_cylinder(), get_sphere2(),
      specialized_distance<Cylinder, Sphere>);
}

void capsule_sphere_gjk(benchmark::State& state) {
  run(state, get_capsule(), get_sphere2(), gjk_distance<Capsule, Sphere>);
}

void capsule_sphere_specialized(benchmark::State& state) {
  run(state, get_capsule(), get_sphere2(),
      specialized_distance<Capsule, Sphere>);
}

void triangle_triangle_gjk(benchmark::State& state) {
  run(state, get_triangle(), get_triangle(),
      gjk_distance<TriangleP, TriangleP>);
}

void triangle_triangle_specialized(benchmark::State& state) {
  run(state, get_triangle(), get_triangle(),
      specialized_distance<TriangleP, TriangleP>);
}

void sphere_triangle_gjk(benchmark::State& state) {
  run(state, get_sphere2(), get_triangle(), gjk_distance<Sphere, TriangleP>);
}

void sphere_triangle_specialized(benchmark::State& state) {
  run(state, get_sphere2(), get_triangle(),
      specialized_distance<Sphere, TriangleP>);
}

// Pairs with only the generic GJK implementation, without specialization for
// the pair of shapes.
void ellipsoid_triangle_gjk(benchmark::State& state) {
  run(state, get_ellipsoid(), get_triangle(),
      gjk_distance<Ellipsoid, TriangleP>);
}

void cone_ellipsoid_gjk(benchmark::State& state) {
  run(state, get_cone(), get_ellipsoid(), gjk_distance<Cone, Ellipsoid>);
}

void cone_cone_gjk(benchmark::State& state) {
  run(state, get_cone(), get_cone(), gjk_distance<Cone, Cone>);
}

void cone_cylinder_gjk(benchmark::State& state) {
  run(state, get_cone(), get_cylinder(), gjk_distance<Cone, Cylinder>);
}

void cone_triangle_gjk(benchmark::State& state) {
  run(state, get_cone(), get_triangle(), gjk_distance<Cone, TriangleP>);
}

void ellipsoid_ellipsoid_gjk(benchmark::State& state) {
  run(state, get_ellipsoid(), get_ellipsoid(),
      gjk_distance<Ellipsoid, Ellipsoid>);
}

void ellipsoid_sphere_gjk(benchmark::State& state) {
  run(state, get_ellipsoid(), get_sphere2(), gjk_distance<Ellipsoid, Sphere>);
}

void capsule_ellipsoid_gjk(benchmark::State& state) {
  run(state, get_capsule(), get_ellipsoid(), gjk_distance<Capsule, Ellipsoid>);
}

void capsule_cylinder_gjk(benchmark::State& state) {
  run(state, get_capsule(), get_cylinder(), gjk_distance<Capsule, Cylinder>);
}

void capsule_triangle_gjk(benchmark::State& state) {
  run(state, get_capsule(), get_triangle(), gjk_distance<Capsule, TriangleP>);
}

void capsule_cone_gjk(benchmark::State& state) {
  run(state, get_capsule(), get_cone(), gjk_distance<Capsule, Cone>);
}

void cylinder_ellipsoid_gjk(benchmark::State& state) {
  run(state, get_cylinder(), get_ellipsoid(),
      gjk_distance<Cylinder, Ellipsoid>);
}

void cylinder_triangle_gjk(benchmark::State& state) {
  run(state, get_cylinder(), get_triangle(), gjk_distance<Cylinder, TriangleP>);
}

void cone_sphere_gjk(benchmark::State& state) {
  run(state, get_cone(), get_sphere2(), gjk_distance<Cone, Sphere>);
}

void box_triangle_gjk(benchmark::State& state) {
  run(state, get_box(), get_triangle(), gjk_distance<Box, TriangleP>);
}

void box_ellipsoid_gjk(benchmark::State& state) {
  run(state, get_box(), get_ellipsoid(), gjk_distance<Box, Ellipsoid>);
}

void box_cylinder_gjk(benchmark::State& state) {
  run(state, get_box(), get_cylinder(), gjk_distance<Box, Cylinder>);
}

void box_cone_gjk(benchmark::State& state) {
  run(state, get_box(), get_cone(), gjk_distance<Box, Cone>);
}

// Pairs with only the specialized implementation: Plane and Halfspace cannot
// be handled by GJK.
void plane_plane_specialized(benchmark::State& state) {
  run(state, get_plane1(), get_plane2(), specialized_distance<Plane, Plane>);
}

void plane_box_specialized(benchmark::State& state) {
  run(state, get_plane1(), get_box(), specialized_distance<Plane, Box>);
}

void plane_triangle_specialized(benchmark::State& state) {
  run(state, get_plane1(), get_triangle(),
      specialized_distance<Plane, TriangleP>);
}

void plane_capsule_specialized(benchmark::State& state) {
  run(state, get_plane1(), get_capsule(), specialized_distance<Plane, Capsule>);
}

void plane_cone_specialized(benchmark::State& state) {
  run(state, get_plane1(), get_cone(), specialized_distance<Plane, Cone>);
}

void plane_cylinder_specialized(benchmark::State& state) {
  run(state, get_plane1(), get_cylinder(),
      specialized_distance<Plane, Cylinder>);
}

void plane_sphere_specialized(benchmark::State& state) {
  run(state, get_plane1(), get_sphere2(), specialized_distance<Plane, Sphere>);
}

void plane_ellipsoid_specialized(benchmark::State& state) {
  run(state, get_plane1(), get_ellipsoid(),
      specialized_distance<Plane, Ellipsoid>);
}

void halfspace_plane_specialized(benchmark::State& state) {
  run(state, get_halfspace(), get_plane2(),
      specialized_distance<Halfspace, Plane>);
}

void halfspace_box_specialized(benchmark::State& state) {
  run(state, get_halfspace(), get_box(), specialized_distance<Halfspace, Box>);
}

void halfspace_capsule_specialized(benchmark::State& state) {
  run(state, get_halfspace(), get_capsule(),
      specialized_distance<Halfspace, Capsule>);
}

void halfspace_cone_specialized(benchmark::State& state) {
  run(state, get_halfspace(), get_cone(),
      specialized_distance<Halfspace, Cone>);
}

void halfspace_cylinder_specialized(benchmark::State& state) {
  run(state, get_halfspace(), get_cylinder(),
      specialized_distance<Halfspace, Cylinder>);
}

void halfspace_sphere_specialized(benchmark::State& state) {
  run(state, get_halfspace(), get_sphere2(),
      specialized_distance<Halfspace, Sphere>);
}

void halfspace_ellipsoid_specialized(benchmark::State& state) {
  run(state, get_halfspace(), get_ellipsoid(),
      specialized_distance<Halfspace, Ellipsoid>);
}

void halfspace_triangle_specialized(benchmark::State& state) {
  run(state, get_halfspace(), get_triangle(),
      specialized_distance<Halfspace, TriangleP>);
}

}  // namespace

// GJK
BENCHMARK(box_box_gjk);
BENCHMARK(box_capsule_gjk);
BENCHMARK(box_cone_gjk);
BENCHMARK(box_cylinder_gjk);
BENCHMARK(box_ellipsoid_gjk);
BENCHMARK(box_sphere_gjk);
BENCHMARK(box_triangle_gjk);

BENCHMARK(capsule_capsule_gjk);
BENCHMARK(capsule_cone_gjk);
BENCHMARK(capsule_cylinder_gjk);
BENCHMARK(capsule_ellipsoid_gjk);
BENCHMARK(capsule_sphere_gjk);
BENCHMARK(capsule_triangle_gjk);

BENCHMARK(cone_cone_gjk);
BENCHMARK(cone_cylinder_gjk);
BENCHMARK(cone_ellipsoid_gjk);
BENCHMARK(cone_sphere_gjk);
BENCHMARK(cone_triangle_gjk);

BENCHMARK(cylinder_cylinder_gjk);
BENCHMARK(cylinder_ellipsoid_gjk);
BENCHMARK(cylinder_sphere_gjk);
BENCHMARK(cylinder_triangle_gjk);

BENCHMARK(ellipsoid_ellipsoid_gjk);
BENCHMARK(ellipsoid_sphere_gjk);
BENCHMARK(ellipsoid_triangle_gjk);

BENCHMARK(sphere_sphere_gjk);
BENCHMARK(sphere_triangle_gjk);

BENCHMARK(triangle_triangle_gjk);

//  Specialized
BENCHMARK(box_capsule_specialized);
BENCHMARK(box_sphere_specialized);

BENCHMARK(capsule_capsule_specialized);
BENCHMARK(capsule_sphere_specialized);

BENCHMARK(cylinder_sphere_specialized);

BENCHMARK(sphere_sphere_specialized);
BENCHMARK(sphere_triangle_specialized);

BENCHMARK(triangle_triangle_specialized);

// Plane
BENCHMARK(plane_plane_specialized);
BENCHMARK(plane_box_specialized);
BENCHMARK(plane_triangle_specialized);
BENCHMARK(plane_capsule_specialized);
BENCHMARK(plane_cone_specialized);
BENCHMARK(plane_cylinder_specialized);
BENCHMARK(plane_sphere_specialized);
BENCHMARK(plane_ellipsoid_specialized);

// Halfspace
BENCHMARK(halfspace_plane_specialized);
BENCHMARK(halfspace_box_specialized);
BENCHMARK(halfspace_capsule_specialized);
BENCHMARK(halfspace_cone_specialized);
BENCHMARK(halfspace_cylinder_specialized);
BENCHMARK(halfspace_sphere_specialized);
BENCHMARK(halfspace_ellipsoid_specialized);
BENCHMARK(halfspace_triangle_specialized);

BENCHMARK_MAIN();
