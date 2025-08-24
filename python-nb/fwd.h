/// Copyright 2025 INRIA

#ifndef COAL_PYTHON_NB_FWD_H
#define COAL_PYTHON_NB_FWD_H

#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/bind_vector.h>

#include <nanoeigenpy/fwd.hpp>

namespace nb = nanobind;

#define DEF_RW_CLASS_ATTRIB(cls_name, name) def_rw(#name, &cls_name::name)
#define DEF_RO_CLASS_ATTRIB(cls_name, name) def_ro(#name, &cls_name::name)
#define DEF_CLASS_FUNC(cls_name, name) def(#name, &cls_name::name)

#endif  // ifndef COAL_PYTHON_NB_FWD_H
