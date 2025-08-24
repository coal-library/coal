/// Copyright 2025 INRIA

#include "coal/fwd.hh"
#include "coal/contact_patch.h"
#include "coal/serialization/collision_data.h"

#include "serializable.hh"

#include "fwd.h"

using namespace coal;
using namespace nb::literals;

void exposeContactPatchAPI(nb::module_& m) {
  nb::enum_<ContactPatch::PatchDirection>(m, "ContactPatchDirection")
      .value("DEFAULT", ContactPatch::PatchDirection::DEFAULT)
      .value("INVERTED", ContactPatch::PatchDirection::INVERTED)
      .export_values();

  nb::class_<ContactPatch>(m, "ContactPatch")
      .def(nb::init<size_t>(), "preallocated_size"_a = 12)
      .DEF_RW_CLASS_ATTRIB(ContactPatch, tf)
      .DEF_RW_CLASS_ATTRIB(ContactPatch, direction)
      .DEF_RW_CLASS_ATTRIB(ContactPatch, penetration_depth)
      .DEF_CLASS_FUNC(ContactPatch, size)
      .DEF_CLASS_FUNC(ContactPatch, getNormal)
      .DEF_CLASS_FUNC(ContactPatch, addPoint)
      .DEF_CLASS_FUNC(ContactPatch, getPoint)
      .DEF_CLASS_FUNC(ContactPatch, getPointShape1)
      .DEF_CLASS_FUNC(ContactPatch, getPointShape2)
      .DEF_CLASS_FUNC(ContactPatch, clear)
      .DEF_CLASS_FUNC(ContactPatch, isSame);

  nb::bind_vector<std::vector<ContactPatch>>(m, "StdVec_ContactPatch");

  nb::class_<ContactPatchRequest>(m, "ContactPatchRequest")
      .def(nb::init<size_t, size_t, Scalar>(), "max_num_patch"_a = 1,
           "num_samples_curved_shapes"_a = 12,
           "patch_tolerance"_a = Scalar(1e-3))
      .def(nb::init<const CollisionRequest&, size_t, Scalar>(),
           "collision_request"_a, "num_samples_curved_shapes"_a = 12,
           "patch_tolerance"_a = Scalar(1e-3))
      .DEF_RW_CLASS_ATTRIB(ContactPatchRequest, max_num_patch)
      .DEF_CLASS_FUNC(ContactPatchRequest, getNumSamplesCurvedShapes)
      .DEF_CLASS_FUNC(ContactPatchRequest, setNumSamplesCurvedShapes)
      .DEF_CLASS_FUNC(ContactPatchRequest, getPatchTolerance)
      .DEF_CLASS_FUNC(ContactPatchRequest, setPatchTolerance);

  nb::bind_vector<std::vector<ContactPatchRequest>>(
      m, "StdVec_ContactPatchRequest");

  nb::class_<ContactPatchResult>(m, "ContactPatchResult")
      .def(nb::init<ContactPatchRequest>(), "request"_a = 12)
      .DEF_CLASS_FUNC(ContactPatchResult, numContactPatches)
      .DEF_CLASS_FUNC(ContactPatchResult, getUnusedContactPatch)
      .def("getContactPatch", &ContactPatchResult::getContactPatch,
           nb::rv_policy::copy)
      .DEF_CLASS_FUNC(ContactPatchResult, clear)
      .DEF_CLASS_FUNC(ContactPatchResult, set)
      .DEF_CLASS_FUNC(ContactPatchResult, check);

  nb::bind_vector<std::vector<ContactPatchResult>>(m,
                                                   "StdVec_ContactPatchResult");

  nb::class_<ComputeContactPatch>(m, "ComputeContactPatch")
      .def(nb::init<const CollisionGeometry*, const CollisionGeometry*>(),
           "o1"_a, "o2"_a)
      .def("__call__",
           [](const ComputeContactPatch& self, const Transform3s& t1,
              const Transform3s& t2, const CollisionResult& collision_result,
              const ContactPatchRequest& request, ContactPatchResult& result) {
             self.operator()(t1, t2, collision_result, request, result);
           });
}