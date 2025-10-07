//
// Copyright (c) 2021-2024 INRIA
//

#ifndef COAL_SERIALIZATION_HFIELD_H
#define COAL_SERIALIZATION_HFIELD_H

#include "coal/hfield.h"

#include "coal/serialization/fwd.h"
#include "coal/serialization/OBBRSS.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::HFNodeBase& node,
               const unsigned int /*version*/) {
  ar& make_nvp("first_child", node.first_child);
  ar& make_nvp("x_id", node.x_id);
  ar& make_nvp("x_size", node.x_size);
  ar& make_nvp("y_id", node.y_id);
  ar& make_nvp("y_size", node.y_size);
  ar& make_nvp("max_height", node.max_height);
  ar& make_nvp("contact_active_faces", node.contact_active_faces);
}

template <class Archive, typename BV>
void serialize(Archive& ar, coal::HFNode<BV>& node,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::HFNodeBase>(node));
  ar& make_nvp("bv", node.bv);
}

namespace internal {
template <typename BV>
struct HeightFieldAccessor : coal::HeightField<BV> {
  typedef coal::HeightField<BV> Base;
  using Base::bvs;
  using Base::heights;
  using Base::max_height;
  using Base::min_height;
  using Base::num_bvs;
  using Base::x_dim;
  using Base::x_grid;
  using Base::y_dim;
  using Base::y_grid;
};
}  // namespace internal

template <class Archive, typename BV>
void serialize(Archive& ar, coal::HeightField<BV>& hf_model,
               const unsigned int /*version*/) {
  ar& make_nvp(
      "base",
      boost::serialization::base_object<coal::CollisionGeometry>(hf_model));

  typedef internal::HeightFieldAccessor<BV> Accessor;
  Accessor& access = reinterpret_cast<Accessor&>(hf_model);

  ar& make_nvp("x_dim", access.x_dim);
  ar& make_nvp("y_dim", access.y_dim);
  ar& make_nvp("heights", access.heights);
  ar& make_nvp("min_height", access.min_height);
  ar& make_nvp("max_height", access.max_height);
  ar& make_nvp("x_grid", access.x_grid);
  ar& make_nvp("y_grid", access.y_grid);

  ar& make_nvp("bvs", access.bvs);
  ar& make_nvp("num_bvs", access.num_bvs);
}
}  // namespace serialization
}  // namespace boost

COAL_SERIALIZATION_DECLARE_EXPORT(::coal::HeightField<::coal::AABB>)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::HeightField<::coal::OBB>)
COAL_SERIALIZATION_DECLARE_EXPORT(::coal::HeightField<::coal::OBBRSS>)

#endif  // ifndef COAL_SERIALIZATION_HFIELD_H
