//
// Copyright (c) 2024 INRIA
//

#ifndef COAL_SERIALIZATION_kIOS_H
#define COAL_SERIALIZATION_kIOS_H

#include "coal/BV/kIOS.h"

#include "coal/serialization/OBB.h"
#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::kIOS& bv, const unsigned int version) {
  split_free(ar, bv, version);
}

template <class Archive>
void save(Archive& ar, const coal::kIOS& bv, const unsigned int /*version*/) {
  // Number of spheres in kIOS is never larger than kIOS::kios_max_num_spheres
  ar& make_nvp("num_spheres", bv.num_spheres);

  std::array<coal::Vec3s, coal::kIOS::max_num_spheres> centers{};
  std::array<coal::Scalar, coal::kIOS::max_num_spheres> radii;
  for (std::size_t i = 0; i < coal::kIOS::max_num_spheres; ++i) {
    centers[i] = bv.spheres[i].o;
    radii[i] = bv.spheres[i].r;
  }
  ar& make_nvp("centers", make_array(centers.data(), centers.size()));
  ar& make_nvp("radii", make_array(radii.data(), radii.size()));

  ar& make_nvp("obb", bv.obb);
}

template <class Archive>
void load(Archive& ar, coal::kIOS& bv, const unsigned int /*version*/) {
  ar >> make_nvp("num_spheres", bv.num_spheres);

  std::array<coal::Vec3s, coal::kIOS::max_num_spheres> centers;
  std::array<coal::Scalar, coal::kIOS::max_num_spheres> radii;
  ar >> make_nvp("centers", make_array(centers.data(), centers.size()));
  ar >> make_nvp("radii", make_array(radii.data(), radii.size()));
  for (std::size_t i = 0; i < coal::kIOS::max_num_spheres; ++i) {
    bv.spheres[i].o = centers[i];
    bv.spheres[i].r = radii[i];
  }

  ar >> make_nvp("obb", bv.obb);
}

}  // namespace serialization
}  // namespace boost

#endif  // COAL_SERIALIZATION_kIOS_H
