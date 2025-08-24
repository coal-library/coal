//
// Copyright (c) 2022 INRIA
//

#ifndef COAL_SERIALIZATION_QUADRILATERAL_H
#define COAL_SERIALIZATION_QUADRILATERAL_H

#include "coal/data_types.h"
#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive, typename Integer>
void serialize(Archive &ar, coal::QuadrilateralTpl<Integer> &quadrilateral,
               const unsigned int /*version*/) {
  ar &make_nvp("p0", quadrilateral[0]);
  ar &make_nvp("p1", quadrilateral[1]);
  ar &make_nvp("p2", quadrilateral[2]);
  ar &make_nvp("p3", quadrilateral[3]);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef COAL_SERIALIZATION_QUADRILATERAL_H
