//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE COAL_ALLOCA
#include <boost/test/included/unit_test.hpp>

#include "coal/data_types.h"
#include "coal/alloca.h"

using namespace coal;

BOOST_AUTO_TEST_CASE(copy_std_vector) {
  for (std::size_t i = 0; i < 10; ++i) {
    std::vector<Vec2s> vec;
    for (std::size_t i = 0; i < 20; ++i) {
      vec.emplace_back(Vec2s::Random());
    }

    // temporary alloca copy
    Vec2s* tmp_vec_copy = COAL_ALLOCA_TYPED_PTR(Vec2s, vec.size());
    for (std::size_t i = 0; i < vec.size(); ++i) {
      tmp_vec_copy[i] = vec[i];
    }

    // copy from alloca tmp
    std::vector<Vec2s> vec_copy;
    for (std::size_t i = 0; i < vec.size(); ++i) {
      vec_copy.emplace_back(tmp_vec_copy[i]);
    }

    // check original and copy match
    BOOST_CHECK(vec_copy.size() == vec.size());
    for (std::size_t i = 0; i < vec_copy.size(); ++i) {
      BOOST_CHECK(vec[i] == vec_copy[i]);
    }
  }
}
