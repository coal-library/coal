//
// Copyright (c) 2026 INRIA
//

#define BOOST_TEST_MODULE COAL_ARRAY_VIEW
#include <boost/test/included/unit_test.hpp>

#include "coal/container/array_view.h"
#include "coal/data_types.h"

using namespace coal;

BOOST_AUTO_TEST_CASE(constructor) {
  {
    ArrayView<Scalar> view;
    BOOST_CHECK(!view.isValid());
  }

  {
    std::vector<Scalar> vec = {3.14, 1.12, -8.3};
    ArrayView<Scalar> view(vec.data(), 0);
    BOOST_CHECK(view.isValid());
    BOOST_CHECK(view.size() == 0);
    BOOST_CHECK(view.data() == vec.data());
  }
}

BOOST_AUTO_TEST_CASE(std_vector_view) {
  std::vector<Scalar> vec = {3.14, 1.12, -8.3};
  ArrayView<Scalar> view(vec.data(), vec.size());

  BOOST_CHECK(view.isValid());
  BOOST_CHECK(view.size() == vec.size());

  for (std::size_t i = 0; i < view.size(); ++i) {
    BOOST_CHECK(view[i] == vec[i]);
  }
}
