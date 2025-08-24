//
// Copyright (c) 2024 INRIA
//

#ifndef COAL_SERIALIZATION_CONTACT_PATCH_H
#define COAL_SERIALIZATION_CONTACT_PATCH_H

#include "coal/collision_data.h"
#include "coal/serialization/fwd.h"
#include "coal/serialization/transform.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::ContactPatch& contact_patch,
               const unsigned int /*version*/) {
  using namespace coal;
  typedef Eigen::Matrix<Scalar, 2, Eigen::Dynamic> PolygonPoints;

  size_t patch_size = contact_patch.size();
  ar& make_nvp("patch_size", patch_size);
  if (patch_size > 0) {
    if (Archive::is_loading::value) {
      contact_patch.points().resize(patch_size);
    }
    Eigen::Map<PolygonPoints> points_map(
        reinterpret_cast<Scalar*>(contact_patch.points().data()), 2,
        static_cast<Eigen::Index>(patch_size));
    ar& make_nvp("points", points_map);
  }

  ar& make_nvp("penetration_depth", contact_patch.penetration_depth);
  ar& make_nvp("direction", contact_patch.direction);
  ar& make_nvp("tf", contact_patch.tf);
}

template <class Archive>
void serialize(Archive& ar, coal::ContactPatchRequest& request,
               const unsigned int /*version*/) {
  using namespace coal;

  ar& make_nvp("max_num_patch", request.max_num_patch);

  size_t num_samples_curved_shapes = request.getNumSamplesCurvedShapes();
  Scalar patch_tolerance = request.getPatchTolerance();
  ar& make_nvp("num_samples_curved_shapes", num_samples_curved_shapes);
  ar& make_nvp("patch_tolerance", num_samples_curved_shapes);

  if (Archive::is_loading::value) {
    request.setNumSamplesCurvedShapes(num_samples_curved_shapes);
    request.setPatchTolerance(patch_tolerance);
  }
}

template <class Archive>
void serialize(Archive& ar, coal::ContactPatchResult& result,
               const unsigned int /*version*/) {
  using namespace coal;

  size_t num_patches = result.numContactPatches();
  ar& make_nvp("num_patches", num_patches);

  std::vector<ContactPatch> patches;
  patches.resize(num_patches);
  if (Archive::is_loading::value) {
    ar& make_nvp("patches", patches);

    const ContactPatchRequest request(num_patches);
    result.set(request);
    for (size_t i = 0; i < num_patches; ++i) {
      ContactPatch& patch = result.getUnusedContactPatch();
      patch = patches[i];
    }
  } else {
    patches.clear();
    for (size_t i = 0; i < num_patches; ++i) {
      patches.emplace_back(result.getContactPatch(i));
    }
    ar& make_nvp("patches", patches);
  }
}

}  // namespace serialization
}  // namespace boost

#endif  // COAL_SERIALIZATION_CONTACT_PATCH_H
