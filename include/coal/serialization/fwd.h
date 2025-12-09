//
// Copyright (c) 2021-2024 INRIA
//

#ifndef COAL_SERIALIZATION_FWD_H
#define COAL_SERIALIZATION_FWD_H

#include <type_traits>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/export.hpp>

#include "coal/fwd.hh"
#include "coal/serialization/eigen.h"

#define COAL_SERIALIZATION_SPLIT(Type)                                   \
  template <class Archive>                                               \
  void serialize(Archive& ar, Type& value, const unsigned int version) { \
    split_free(ar, value, version);                                      \
  }

#define COAL_SERIALIZATION_DECLARE_EXPORT(T) \
  BOOST_CLASS_EXPORT_KEY(T)                  \
  namespace boost {                          \
  namespace archive {                        \
  namespace detail {                         \
  namespace extra_detail {                   \
  template <>                                \
  struct init_guid<T> {                      \
    static guid_initializer<T> const& g;     \
  };                                         \
  }                                          \
  }                                          \
  }                                          \
  }                                          \
  /**/

#define COAL_SERIALIZATION_DEFINE_EXPORT(T)             \
  namespace boost {                                     \
  namespace archive {                                   \
  namespace detail {                                    \
  namespace extra_detail {                              \
  guid_initializer<T> const& init_guid<T>::g =          \
      ::boost::serialization::singleton<                \
          guid_initializer<T> >::get_mutable_instance() \
          .export_guid();                               \
  }                                                     \
  }                                                     \
  }                                                     \
  }                                                     \
  /**/

namespace coal {
namespace serialization {
namespace detail {

template <class Derived, class Base>
struct init_cast_register {};

template <class Derived, class Base>
struct cast_register_initializer {
  void init(std::true_type) const {
    boost::serialization::void_cast_register<Derived, Base>();
  }

  void init(std::false_type) const {}

  cast_register_initializer const& init() const {
#ifndef _WIN32
    COAL_COMPILER_DIAGNOSTIC_PUSH
    _Pragma("GCC diagnostic ignored \"-Wconversion\"")
#endif
        BOOST_STATIC_WARNING((std::is_base_of<Base, Derived>::value));
#ifndef _WIN32
    COAL_COMPILER_DIAGNOSTIC_POP
#endif
    init(std::is_base_of<Base, Derived>());
    return *this;
  }
};

}  // namespace detail

template <typename T>
struct register_type {
  template <class Archive>
  static void on(Archive& /*ar*/) {}
};
}  // namespace serialization
}  // namespace coal

#define COAL_SERIALIZATION_CAST_REGISTER(Derived, Base)                      \
  namespace coal {                                                           \
  namespace serialization {                                                  \
  namespace detail {                                                         \
  template <>                                                                \
  struct init_cast_register<Derived, Base> {                                 \
    static cast_register_initializer<Derived, Base> const& g;                \
  };                                                                         \
  cast_register_initializer<Derived, Base> const& init_cast_register<        \
      Derived, Base>::g =                                                    \
      ::boost::serialization::singleton<                                     \
          cast_register_initializer<Derived, Base> >::get_mutable_instance() \
          .init();                                                           \
  }                                                                          \
  }                                                                          \
  }

#endif  // ifndef COAL_SERIALIZATION_FWD_H
