/// Copyright 2025 INRIA

#include "coal/fwd.hh"
#include "coal/narrowphase/gjk.h"

#include "fwd.h"

using namespace coal;
using coal::details::EPA;
using coal::details::GJK;
using coal::details::MinkowskiDiff;
using coal::details::SupportOptions;

using namespace nb::literals;

void exposeGJK(nb::module_& m) {
  nb::enum_<GJK::Status>(m, "GJKStatus")
      .value("Failed", GJK::Status::Failed)
      .value("DidNotRun", GJK::Status::DidNotRun)
      .value("NoCollision", GJK::Status::NoCollision)
      .value("NoCollisionEarlyStopped", GJK::Status::NoCollisionEarlyStopped)
      .value("CollisionWithPenetrationInformation",
             GJK::Status::CollisionWithPenetrationInformation)
      .value("Collision", GJK::Status::Collision)
      .export_values();

  nb::class_<MinkowskiDiff>(m, "MinkowskiDiff")
      .def(nb::init<>())
      .def("set",
           [](MinkowskiDiff& diff, const ShapeBase* shape1,
              const ShapeBase* shape2, bool flag) {
             if (flag) {
               diff.set<SupportOptions::WithSweptSphere>(shape1, shape2);
             } else {
               diff.set<SupportOptions::NoSweptSphere>(shape1, shape2);
             }
           })
      .def("set",
           [](MinkowskiDiff& diff, const ShapeBase* shape1,
              const ShapeBase* shape2, const Transform3s& transform1,
              const Transform3s& transform2, bool flag) {
             if (flag) {
               diff.set<SupportOptions::WithSweptSphere>(
                   shape1, shape2, transform1, transform2);
             } else {
               diff.set<SupportOptions::NoSweptSphere>(shape1, shape2,
                                                       transform1, transform2);
             }
           })
      .def("support0",
           [](MinkowskiDiff& self, const Vec3s& dir, int& hint,
              bool compute_swept_sphere_support = false) {
             if (compute_swept_sphere_support) {
               self.support0<SupportOptions::WithSweptSphere>(dir, hint);
             } else {
               self.support0<SupportOptions::NoSweptSphere>(dir, hint);
             }
           })
      .def("support1",
           [](MinkowskiDiff& self, const Vec3s& dir, int& hint,
              bool compute_swept_sphere_support = false) {
             if (compute_swept_sphere_support) {
               self.support1<SupportOptions::WithSweptSphere>(dir, hint);
             } else {
               self.support1<SupportOptions::NoSweptSphere>(dir, hint);
             }
           })
      .def("support", &MinkowskiDiff::support)
      .DEF_RW_CLASS_ATTRIB(MinkowskiDiff, swept_sphere_radius)
      .DEF_RW_CLASS_ATTRIB(MinkowskiDiff, normalize_support_direction);

  nb::enum_<GJKVariant>(m, "GJKVariant")
      .value("DefaultGJK", GJKVariant::DefaultGJK)
      .value("PolyakAcceleration", GJKVariant::PolyakAcceleration)
      .value("NesterovAcceleration", GJKVariant::NesterovAcceleration)
      .export_values();

  nb::enum_<GJKInitialGuess>(m, "GJKInitialGuess")
      .value("DefaultGuess", GJKInitialGuess::DefaultGuess)
      .value("CachedGuess", GJKInitialGuess::CachedGuess)
      .value("BoundingVolumeGuess", GJKInitialGuess::BoundingVolumeGuess)
      .export_values();

  nb::enum_<GJKConvergenceCriterion>(m, "GJKConvergenceCriterion")
      .value("Default", GJKConvergenceCriterion::Default)
      .value("DualityGap", GJKConvergenceCriterion::DualityGap)
      .value("Hybrid", GJKConvergenceCriterion::Hybrid)
      .export_values();

  nb::enum_<GJKConvergenceCriterionType>(m, "GJKConvergenceCriterionType")
      .value("Absolute", GJKConvergenceCriterionType::Absolute)
      .value("Relative", GJKConvergenceCriterionType::Relative)
      .export_values();

  nb::class_<GJK>(m, "GJK")
      .def(nb::init<unsigned int, Scalar>(), "max_iterations_"_a, "tolerance"_a)
      .DEF_RW_CLASS_ATTRIB(GJK, distance)
      .DEF_RW_CLASS_ATTRIB(GJK, ray)
      .DEF_RW_CLASS_ATTRIB(GJK, support_hint)
      .DEF_RW_CLASS_ATTRIB(GJK, gjk_variant)
      .DEF_RW_CLASS_ATTRIB(GJK, convergence_criterion)
      .DEF_RW_CLASS_ATTRIB(GJK, convergence_criterion_type)
      .DEF_CLASS_FUNC(GJK, reset)
      .DEF_CLASS_FUNC(GJK, evaluate)
      .DEF_CLASS_FUNC(GJK, getTolerance)
      .DEF_CLASS_FUNC(GJK, getNumMaxIterations)
      .DEF_CLASS_FUNC(GJK, getNumIterations)
      .DEF_CLASS_FUNC(GJK, getNumIterationsMomentumStopped)
      .DEF_CLASS_FUNC(GJK, hasClosestPoints)
      .DEF_CLASS_FUNC(GJK, getWitnessPointsAndNormal)
      .DEF_CLASS_FUNC(GJK, setDistanceEarlyBreak)
      .DEF_CLASS_FUNC(GJK, getGuessFromSimplex);
}