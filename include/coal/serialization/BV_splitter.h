//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_BV_SPLITTER_H
#define COAL_SERIALIZATION_BV_SPLITTER_H

#include "coal/internal/BV_splitter.h"

#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

namespace internal {
template <typename BV>
struct BVSplitterAccessor : coal::BVSplitter<BV> {
  typedef coal::BVSplitter<BV> Base;
  using Base::split_axis;
  using Base::split_method;
  using Base::split_value;
  using Base::split_vector;
  using Base::tri_indices;
  using Base::type;
  using Base::vertices;
};
}  // namespace internal

template <class Archive, typename BV>
void save(Archive& ar, const coal::BVSplitter<BV>& splitter_,
          const unsigned int /*version*/) {
  using namespace coal;
  typedef internal::BVSplitterAccessor<BV> Accessor;
  const Accessor& splitter = reinterpret_cast<const Accessor&>(splitter_);

  ar& make_nvp("split_axis", splitter.split_axis);
  ar& make_nvp("split_vector", splitter.split_vector);
  ar& make_nvp("split_value", splitter.split_value);
  ar& make_nvp("type", splitter.type);
  ar& make_nvp("split_method", splitter.split_method);
}

template <class Archive, typename BV>
void load(Archive& ar, coal::BVSplitter<BV>& splitter_,
          const unsigned int /*version*/) {
  using namespace coal;
  typedef internal::BVSplitterAccessor<BV> Accessor;
  Accessor& splitter = reinterpret_cast<Accessor&>(splitter_);

  ar >> make_nvp("split_axis", splitter.split_axis);
  ar >> make_nvp("split_vector", splitter.split_vector);
  ar >> make_nvp("split_value", splitter.split_value);
  ar >> make_nvp("type", splitter.type);
  ar >> make_nvp("split_method", splitter.split_method);

  splitter.vertices = NULL;
  splitter.tri_indices = NULL;
}
}  // namespace serialization
}  // namespace boost

#endif  // ifndef COAL_SERIALIZATION_BV_SPLITTER_H
