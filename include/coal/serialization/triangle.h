//
// Copyright (c) 2021-2022 INRIA
//

#ifndef COAL_SERIALIZATION_TRIANGLE_H
#define COAL_SERIALIZATION_TRIANGLE_H

#include "coal/data_types.h"
#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive, typename Integer>
void serialize(Archive &ar, coal::TriangleTpl<Integer> &triangle,
               const unsigned int /*version*/) {
  ar &make_nvp("p0", triangle[0]);
  ar &make_nvp("p1", triangle[1]);
  ar &make_nvp("p2", triangle[2]);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef COAL_SERIALIZATION_TRIANGLE_H
