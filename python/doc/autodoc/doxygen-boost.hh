#ifndef DOXYGEN_BOOST_DOC_HH
#define DOXYGEN_BOOST_DOC_HH

#ifndef DOXYGEN_DOC_HH
#error "You should have included doxygen.hh first."
#endif  // DOXYGEN_DOC_HH

#include <boost/python.hpp>

namespace doxygen {

namespace visitor {

template <typename function_type,
          typename policy_type = boost::python::default_call_policies>
struct member_func_impl : boost::python::def_visitor<
                              member_func_impl<function_type, policy_type> > {
  member_func_impl(const char* n, const function_type& f)
      : name(n), function(f) {}

  member_func_impl(const char* n, const function_type& f, policy_type p)
      : name(n), function(f), policy(p) {}

  template <class classT>
  inline void visit(classT& c) const {
    // Either a boost::python::keyword<N> object or a void_ object
    call(c, member_func_args(function));
  }

  template <class classT, std::size_t nkeywords>
  inline void call(
      classT& c, const boost::python::detail::keywords<nkeywords>& args) const {
    c.def(name, function, member_func_doc(function), args, policy);
  }

  template <class classT>
  inline void call(classT& c, const void_&) const {
    c.def(name, function, member_func_doc(function), policy);
  }

  const char* name;
  const function_type& function;
  policy_type policy;
};

// TODO surprisingly, this does not work when defined here but it works when
// defined after the generated files are included.
template <typename function_type>
inline member_func_impl<function_type> member_func(
    const char* name, const function_type& function) {
  return member_func_impl<function_type>(name, function);
}

template <typename function_type, typename policy_type>
inline member_func_impl<function_type, policy_type> member_func(
    const char* name, const function_type& function,
    const policy_type& policy) {
  return member_func_impl<function_type, policy_type>(name, function, policy);
}

#define DOXYGEN_DOC_DECLARE_INIT_VISITOR(z, nargs, unused)                    \
  template <typename Class BOOST_PP_COMMA_IF(nargs)                           \
                BOOST_PP_ENUM_PARAMS(nargs, class Arg)>                       \
  struct init_##nargs##_impl                                                  \
      : boost::python::def_visitor<                                           \
            init_##nargs##_impl<Class BOOST_PP_COMMA_IF(nargs)                \
                                    BOOST_PP_ENUM_PARAMS(nargs, Arg)> > {     \
    typedef constructor_##nargs##_impl<Class BOOST_PP_COMMA_IF(nargs)         \
                                           BOOST_PP_ENUM_PARAMS(nargs, Arg)>  \
        constructor;                                                          \
    typedef boost::python::init<BOOST_PP_ENUM_PARAMS(nargs, Arg)> init_base;  \
                                                                              \
    template <class classT>                                                   \
    inline void visit(classT& c) const {                                      \
      call(c, constructor::args());                                           \
    }                                                                         \
                                                                              \
    template <class classT>                                                   \
    void call(classT& c,                                                      \
              const boost::python::detail::keywords<nargs + 1>& args) const { \
      c.def(init_base(constructor::doc(), args));                             \
    }                                                                         \
                                                                              \
    template <class classT>                                                   \
    void call(classT& c, const void_&) const {                                \
      c.def(init_base(constructor::doc()));                                   \
    }                                                                         \
  };                                                                          \
                                                                              \
  template <typename Class BOOST_PP_COMMA_IF(nargs)                           \
                BOOST_PP_ENUM_PARAMS(nargs, class Arg)>                       \
  inline init_##nargs##_impl<Class BOOST_PP_COMMA_IF(nargs)                   \
                                 BOOST_PP_ENUM_PARAMS(nargs, Arg)>            \
  init() {                                                                    \
    return init_##nargs##_impl<Class BOOST_PP_COMMA_IF(nargs)                 \
                                   BOOST_PP_ENUM_PARAMS(nargs, Arg)>();       \
  }

BOOST_PP_REPEAT(DOXYGEN_DOC_MAX_NUMBER_OF_ARGUMENTS_IN_CONSTRUCTOR,
                DOXYGEN_DOC_DECLARE_INIT_VISITOR, ~)
#undef DOXYGEN_DOC_DECLARE_INIT_VISITOR

}  // namespace visitor

template <typename Func>
void def(const char* name, Func func) {
  boost::python::def(name, func, member_func_doc(func));
}

}  // namespace doxygen

#endif  // DOXYGEN_BOOST_DOC_HH
