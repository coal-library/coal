/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

/** \author Jia Pan */

#ifndef COAL_BV_H
#define COAL_BV_H

#include "coal/BV/kDOP.h"
#include "coal/BV/AABB.h"
#include "coal/BV/OBB.h"
#include "coal/BV/RSS.h"
#include "coal/BV/OBBRSS.h"
#include "coal/BV/kIOS.h"
#include "coal/math/transform.h"

/** @brief Main namespace */
namespace coal {

/// @cond IGNORE
namespace details {

/// @brief Convert a bounding volume of type BV1 in configuration tf1 to a
/// bounding volume of type BV2 in I configuration.
template <typename BV1, typename BV2>
struct Converter {
  static void convert(const BV1& bv1, const Transform3s& tf1, BV2& bv2);
  static void convert(const BV1& bv1, BV2& bv2);
};

/// @brief Convert from AABB to AABB, not very tight but is fast.
template <>
struct Converter<AABB, AABB> {
  static void convert(const AABB& bv1, const Transform3s& tf1, AABB& bv2) {
    const Vec3s& center = bv1.center();
    Scalar r = (bv1.max_ - bv1.min_).norm() * Scalar(0.5);
    const Vec3s center2 = tf1.transform(center);
    bv2.min_ = center2 - Vec3s::Constant(r);
    bv2.max_ = center2 + Vec3s::Constant(r);
  }

  static void convert(const AABB& bv1, AABB& bv2) { bv2 = bv1; }
};

template <>
struct Converter<AABB, OBB> {
  static void convert(const AABB& bv1, const Transform3s& tf1, OBB& bv2) {
    bv2.To = tf1.transform(bv1.center());
    bv2.extent.noalias() = (bv1.max_ - bv1.min_) * 0.5;
    bv2.axes = tf1.getRotation();
  }

  static void convert(const AABB& bv1, OBB& bv2) {
    bv2.To = bv1.center();
    bv2.extent.noalias() = (bv1.max_ - bv1.min_) * 0.5;
    bv2.axes.setIdentity();
  }
};

template <>
struct Converter<OBB, OBB> {
  static void convert(const OBB& bv1, const Transform3s& tf1, OBB& bv2) {
    bv2.extent = bv1.extent;
    bv2.To = tf1.transform(bv1.To);
    bv2.axes.noalias() = tf1.getRotation() * bv1.axes;
  }

  static void convert(const OBB& bv1, OBB& bv2) { bv2 = bv1; }
};

template <>
struct Converter<OBBRSS, OBB> {
  static void convert(const OBBRSS& bv1, const Transform3s& tf1, OBB& bv2) {
    Converter<OBB, OBB>::convert(bv1.obb, tf1, bv2);
  }

  static void convert(const OBBRSS& bv1, OBB& bv2) {
    Converter<OBB, OBB>::convert(bv1.obb, bv2);
  }
};

template <>
struct Converter<RSS, OBB> {
  static void convert(const RSS& bv1, const Transform3s& tf1, OBB& bv2) {
    const Scalar half = Scalar(0.5);
    bv2.extent = Vec3s(bv1.length[0] * half + bv1.radius,
                       bv1.length[1] * half + bv1.radius, bv1.radius);
    bv2.To = tf1.transform(bv1.Tr);
    bv2.axes.noalias() = tf1.getRotation() * bv1.axes;
  }

  static void convert(const RSS& bv1, OBB& bv2) {
    const Scalar half = Scalar(0.5);
    bv2.extent = Vec3s(bv1.length[0] * half + bv1.radius,
                       bv1.length[1] * half + bv1.radius, bv1.radius);
    bv2.To = bv1.Tr;
    bv2.axes = bv1.axes;
  }
};

template <typename BV1>
struct Converter<BV1, AABB> {
  static void convert(const BV1& bv1, const Transform3s& tf1, AABB& bv2) {
    const Vec3s& center = bv1.center();
    const Scalar half = Scalar(0.5);
    Scalar r = Vec3s(bv1.width(), bv1.height(), bv1.depth()).norm() * half;
    const Vec3s center2 = tf1.transform(center);
    bv2.min_ = center2 - Vec3s::Constant(r);
    bv2.max_ = center2 + Vec3s::Constant(r);
  }

  static void convert(const BV1& bv1, AABB& bv2) {
    const Vec3s& center = bv1.center();
    const Scalar half = Scalar(0.5);
    Scalar r = Vec3s(bv1.width(), bv1.height(), bv1.depth()).norm() * half;
    bv2.min_ = center - Vec3s::Constant(r);
    bv2.max_ = center + Vec3s::Constant(r);
  }
};

template <typename BV1>
struct Converter<BV1, OBB> {
  static void convert(const BV1& bv1, const Transform3s& tf1, OBB& bv2) {
    AABB bv;
    Converter<BV1, AABB>::convert(bv1, bv);
    Converter<AABB, OBB>::convert(bv, tf1, bv2);
  }

  static void convert(const BV1& bv1, OBB& bv2) {
    AABB bv;
    Converter<BV1, AABB>::convert(bv1, bv);
    Converter<AABB, OBB>::convert(bv, bv2);
  }
};

template <>
struct Converter<OBB, RSS> {
  static void convert(const OBB& bv1, const Transform3s& tf1, RSS& bv2) {
    bv2.Tr = tf1.transform(bv1.To);
    bv2.axes.noalias() = tf1.getRotation() * bv1.axes;

    bv2.radius = bv1.extent[2];
    bv2.length[0] = 2 * (bv1.extent[0] - bv2.radius);
    bv2.length[1] = 2 * (bv1.extent[1] - bv2.radius);
  }

  static void convert(const OBB& bv1, RSS& bv2) {
    bv2.Tr = bv1.To;
    bv2.axes = bv1.axes;

    bv2.radius = bv1.extent[2];
    bv2.length[0] = 2 * (bv1.extent[0] - bv2.radius);
    bv2.length[1] = 2 * (bv1.extent[1] - bv2.radius);
  }
};

template <>
struct Converter<RSS, RSS> {
  static void convert(const RSS& bv1, const Transform3s& tf1, RSS& bv2) {
    bv2.Tr = tf1.transform(bv1.Tr);
    bv2.axes.noalias() = tf1.getRotation() * bv1.axes;

    bv2.radius = bv1.radius;
    bv2.length[0] = bv1.length[0];
    bv2.length[1] = bv1.length[1];
  }

  static void convert(const RSS& bv1, RSS& bv2) { bv2 = bv1; }
};

template <>
struct Converter<OBBRSS, RSS> {
  static void convert(const OBBRSS& bv1, const Transform3s& tf1, RSS& bv2) {
    Converter<RSS, RSS>::convert(bv1.rss, tf1, bv2);
  }

  static void convert(const OBBRSS& bv1, RSS& bv2) {
    Converter<RSS, RSS>::convert(bv1.rss, bv2);
  }
};

template <>
struct Converter<AABB, RSS> {
  static void convert(const AABB& bv1, const Transform3s& tf1, RSS& bv2) {
    bv2.Tr = tf1.transform(bv1.center());

    /// Sort the AABB edges so that AABB extents are ordered.
    Scalar d[3] = {bv1.width(), bv1.height(), bv1.depth()};
    Eigen::DenseIndex id[3] = {0, 1, 2};

    for (Eigen::DenseIndex i = 1; i < 3; ++i) {
      for (Eigen::DenseIndex j = i; j > 0; --j) {
        if (d[j] > d[j - 1]) {
          {
            Scalar tmp = d[j];
            d[j] = d[j - 1];
            d[j - 1] = tmp;
          }
          {
            Eigen::DenseIndex tmp = id[j];
            id[j] = id[j - 1];
            id[j - 1] = tmp;
          }
        }
      }
    }

    const Vec3s extent = (bv1.max_ - bv1.min_) * 0.5;
    bv2.radius = extent[id[2]];
    bv2.length[0] = (extent[id[0]] - bv2.radius) * 2;
    bv2.length[1] = (extent[id[1]] - bv2.radius) * 2;

    const Matrix3s& R = tf1.getRotation();
    const bool left_hand = (id[0] == (id[1] + 1) % 3);
    if (left_hand)
      bv2.axes.col(0) = -R.col(id[0]);
    else
      bv2.axes.col(0) = R.col(id[0]);
    bv2.axes.col(1) = R.col(id[1]);
    bv2.axes.col(2) = R.col(id[2]);
  }

  static void convert(const AABB& bv1, RSS& bv2) {
    convert(bv1, Transform3s(), bv2);
  }
};

template <>
struct Converter<AABB, OBBRSS> {
  static void convert(const AABB& bv1, const Transform3s& tf1, OBBRSS& bv2) {
    Converter<AABB, OBB>::convert(bv1, tf1, bv2.obb);
    Converter<AABB, RSS>::convert(bv1, tf1, bv2.rss);
  }

  static void convert(const AABB& bv1, OBBRSS& bv2) {
    Converter<AABB, OBB>::convert(bv1, bv2.obb);
    Converter<AABB, RSS>::convert(bv1, bv2.rss);
  }
};

}  // namespace details

/// @endcond

/// @brief Convert a bounding volume of type BV1 in configuration tf1 to
/// bounding volume of type BV2 in identity configuration.
template <typename BV1, typename BV2>
static inline void convertBV(const BV1& bv1, const Transform3s& tf1, BV2& bv2) {
  details::Converter<BV1, BV2>::convert(bv1, tf1, bv2);
}

/// @brief Convert a bounding volume of type BV1 to bounding volume of type BV2
/// in identity configuration.
template <typename BV1, typename BV2>
static inline void convertBV(const BV1& bv1, BV2& bv2) {
  details::Converter<BV1, BV2>::convert(bv1, bv2);
}

}  // namespace coal

#endif
