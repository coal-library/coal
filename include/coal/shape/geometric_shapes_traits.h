/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2015-2022, CNRS, Inria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef COAL_GEOMETRIC_SHAPES_TRAITS_H
#define COAL_GEOMETRIC_SHAPES_TRAITS_H

#include "coal/shape/geometric_shapes.h"

namespace coal {

struct shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = false,
    IsInflatable = false,
    HasInflatedSupportFunction = false,
    IsStrictlyConvex = false
  };
};

template <typename Shape>
struct shape_traits : shape_traits_base {};

template <>
struct shape_traits<TriangleP> : shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = false,
    IsInflatable = true,
    HasInflatedSupportFunction = false,
    IsStrictlyConvex = false
  };
};

template <>
struct shape_traits<Box> : shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = false,
    IsInflatable = true,
    HasInflatedSupportFunction = false,
    IsStrictlyConvex = false
  };
};

template <>
struct shape_traits<Sphere> : shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = false,
    IsInflatable = true,
    HasInflatedSupportFunction = false,
    IsStrictlyConvex = true
  };
};

template <>
struct shape_traits<Ellipsoid> : shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = false,
    IsInflatable = true,
    HasInflatedSupportFunction = false,
    IsStrictlyConvex = true
  };
};

template <>
struct shape_traits<Capsule> : shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = false,
    IsInflatable = true,
    HasInflatedSupportFunction = false,
    IsStrictlyConvex = false
  };
};

template <>
struct shape_traits<Cone> : shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = false,
    IsInflatable = true,
    HasInflatedSupportFunction = false,
    IsStrictlyConvex = false
  };
};

template <>
struct shape_traits<Cylinder> : shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = false,
    IsInflatable = true,
    HasInflatedSupportFunction = false,
    IsStrictlyConvex = false
  };
};

template <typename IndexType>
struct shape_traits<ConvexBaseTpl<IndexType>> : shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = true,
    IsInflatable = true,
    HasInflatedSupportFunction = true,
    IsStrictlyConvex = false
  };
};

template <>
struct shape_traits<Halfspace> : shape_traits_base {
  enum {
    NeedNormalizedDir = false,
    NeedNesterovNormalizeHeuristic = false,
    IsInflatable = true,
    HasInflatedSupportFunction = false,
    IsStrictlyConvex = false
  };
};

}  // namespace coal

#endif  // ifndef COAL_GEOMETRIC_SHAPES_TRAITS_H
