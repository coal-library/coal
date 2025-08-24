//
// Copyright (c) 2017-2021 CNRS INRIA
//

/*
 Code adapted from Pinocchio and
 https://gist.githubusercontent.com/mtao/5798888/raw/5be9fa9b66336c166dba3a92c0e5b69ffdb81501/eigen_boost_serialization.hpp
 Copyright (c) 2015 Michael Tao
*/

#ifndef COAL_SERIALIZATION_EIGEN_H
#define COAL_SERIALIZATION_EIGEN_H

#ifdef COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
#ifdef HPP_FCL_SKIP_EIGEN_BOOST_SERIALIZATION
#define COAL_SKIP_EIGEN_BOOST_SERIALIZATION
#endif
#endif

#ifndef COAL_SKIP_EIGEN_BOOST_SERIALIZATION

#include <Eigen/Dense>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/array.hpp>

// Workaround a bug in GCC >= 7 and C++17
// ref. https://gitlab.com/libeigen/eigen/-/issues/1676
#ifdef __GNUC__
#if __GNUC__ >= 7 && __cplusplus >= 201703L
namespace boost {
namespace serialization {
struct U;
}
}  // namespace boost
namespace Eigen {
namespace internal {
template <>
struct traits<boost::serialization::U> {
  enum { Flags = 0 };
};
}  // namespace internal
}  // namespace Eigen
#endif
#endif

// There is the same kind of bug with CL when building
// nanobind bindings.
// This doesn't happen when building Boost.Python bindings.
// We only activate the workaround when C++17 is enable since
// it's mandatory to build nanobind.
#if defined(_MSVC_LANG) && _MSVC_LANG >= 201703L
namespace boost {
namespace archive {
class xml_iarchive;
class text_iarchive;
class binary_iarchive;
}  // namespace archive
}  // namespace boost
namespace Eigen {
namespace internal {
template <>
struct traits<boost::archive::xml_iarchive> {
  enum { Flags = 0 };
};
template <>
struct traits<boost::archive::text_iarchive> {
  enum { Flags = 0 };
};
template <>
struct traits<boost::archive::binary_iarchive> {
  enum { Flags = 0 };
};
}  // namespace internal
}  // namespace Eigen
#endif

namespace boost {
namespace serialization {

template <class Archive, typename S, int Rows, int Cols, int Options,
          int MaxRows, int MaxCols>
void save(Archive& ar,
          const Eigen::Matrix<S, Rows, Cols, Options, MaxRows, MaxCols>& m,
          const unsigned int /*version*/) {
  Eigen::DenseIndex rows(m.rows()), cols(m.cols());
  if (Rows == Eigen::Dynamic) ar& BOOST_SERIALIZATION_NVP(rows);
  if (Cols == Eigen::Dynamic) ar& BOOST_SERIALIZATION_NVP(cols);
  ar& make_nvp("data", make_array(m.data(), (size_t)m.size()));
}

template <class Archive, typename S, int Rows, int Cols, int Options,
          int MaxRows, int MaxCols>
void load(Archive& ar,
          Eigen::Matrix<S, Rows, Cols, Options, MaxRows, MaxCols>& m,
          const unsigned int /*version*/) {
  Eigen::DenseIndex rows = Rows, cols = Cols;
  if (Rows == Eigen::Dynamic) ar >> BOOST_SERIALIZATION_NVP(rows);
  if (Cols == Eigen::Dynamic) ar >> BOOST_SERIALIZATION_NVP(cols);
  m.resize(rows, cols);
  ar >> make_nvp("data", make_array(m.data(), (size_t)m.size()));
}

template <class Archive, typename S, int Rows, int Cols, int Options,
          int MaxRows, int MaxCols>
void serialize(Archive& ar,
               Eigen::Matrix<S, Rows, Cols, Options, MaxRows, MaxCols>& m,
               const unsigned int version) {
  split_free(ar, m, version);
}

template <class Archive, typename PlainObjectBase, int MapOptions,
          typename StrideType>
void save(Archive& ar,
          const Eigen::Map<PlainObjectBase, MapOptions, StrideType>& m,
          const unsigned int /*version*/) {
  Eigen::DenseIndex rows(m.rows()), cols(m.cols());
  if (PlainObjectBase::RowsAtCompileTime == Eigen::Dynamic)
    ar& BOOST_SERIALIZATION_NVP(rows);
  if (PlainObjectBase::ColsAtCompileTime == Eigen::Dynamic)
    ar& BOOST_SERIALIZATION_NVP(cols);
  ar& make_nvp("data", make_array(m.data(), (size_t)m.size()));
}

template <class Archive, typename PlainObjectBase, int MapOptions,
          typename StrideType>
void load(Archive& ar, Eigen::Map<PlainObjectBase, MapOptions, StrideType>& m,
          const unsigned int /*version*/) {
  Eigen::DenseIndex rows = PlainObjectBase::RowsAtCompileTime,
                    cols = PlainObjectBase::ColsAtCompileTime;
  if (PlainObjectBase::RowsAtCompileTime == Eigen::Dynamic)
    ar >> BOOST_SERIALIZATION_NVP(rows);
  if (PlainObjectBase::ColsAtCompileTime == Eigen::Dynamic)
    ar >> BOOST_SERIALIZATION_NVP(cols);
  m.resize(rows, cols);
  ar >> make_nvp("data", make_array(m.data(), (size_t)m.size()));
}

template <class Archive, typename PlainObjectBase, int MapOptions,
          typename StrideType>
void serialize(Archive& ar,
               Eigen::Map<PlainObjectBase, MapOptions, StrideType>& m,
               const unsigned int version) {
  split_free(ar, m, version);
}

}  // namespace serialization
}  // namespace boost
//
#endif  // ifned COAL_SKIP_EIGEN_BOOST_SERIALIZATION

#endif  // ifndef COAL_SERIALIZATION_EIGEN_H
