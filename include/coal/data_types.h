/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2023, Inria
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

#ifndef COAL_DATA_TYPES_H
#define COAL_DATA_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "coal/config.hh"
#include "coal/deprecated.hh"

#ifdef COAL_HAS_OCTOMAP
#define OCTOMAP_VERSION_AT_LEAST(x, y, z) \
  (OCTOMAP_MAJOR_VERSION > x ||           \
   (OCTOMAP_MAJOR_VERSION >= x &&         \
    (OCTOMAP_MINOR_VERSION > y ||         \
     (OCTOMAP_MINOR_VERSION >= y && OCTOMAP_PATCH_VERSION >= z))))

#define OCTOMAP_VERSION_AT_MOST(x, y, z) \
  (OCTOMAP_MAJOR_VERSION < x ||          \
   (OCTOMAP_MAJOR_VERSION <= x &&        \
    (OCTOMAP_MINOR_VERSION < y ||        \
     (OCTOMAP_MINOR_VERSION <= y && OCTOMAP_PATCH_VERSION <= z))))
#endif  // COAL_HAS_OCTOMAP

namespace coal {
#ifdef COAL_USE_FLOAT_PRECISION
COAL_DEPRECATED typedef float CoalScalar;
typedef float Scalar;
#else
COAL_DEPRECATED typedef double CoalScalar;
typedef double Scalar;
#endif
typedef Eigen::Matrix<Scalar, 3, 1> Vec3s;
typedef Eigen::Matrix<Scalar, 2, 1> Vec2s;
typedef Eigen::Matrix<Scalar, 6, 1> Vec6s;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecXs;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3s;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor> MatrixX3s;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 2, Eigen::RowMajor> MatrixX2s;
typedef Eigen::Matrix<Eigen::DenseIndex, Eigen::Dynamic, 3, Eigen::RowMajor>
    Matrixx3i;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;
typedef Eigen::Vector2i support_func_guess_t;

typedef double SolverScalar;
typedef Eigen::Matrix<SolverScalar, 3, 1> Vec3ps;

#ifdef COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
// We keep the FCL_REAL typedef and the Vec[..]f typedefs for backward
// compatibility.
typedef Scalar FCL_REAL;
typedef Vec3s Vec3f;
typedef Vec2s Vec2f;
typedef Vec6s Vec6f;
typedef VecXs VecXf;
typedef Matrix3s Matrix3f;
typedef MatrixX3s Matrixx3f;
typedef MatrixX2s Matrixx2f;
typedef MatrixXs MatrixXf;
#endif

/// @brief Initial guess to use for the GJK algorithm
/// DefaultGuess: Vec3s(1, 0, 0)
/// CachedGuess: previous vector found by GJK or guess cached by the user
/// BoundingVolumeGuess: guess using the centers of the shapes' AABB
/// WARNING: to use BoundingVolumeGuess, computeLocalAABB must have been called
/// on the two shapes.
enum GJKInitialGuess { DefaultGuess, CachedGuess, BoundingVolumeGuess };

/// @brief Variant to use for the GJK algorithm
enum GJKVariant { DefaultGJK, PolyakAcceleration, NesterovAcceleration };

/// @brief Which convergence criterion is used to stop the algorithm (when the
/// shapes are not in collision). (default) VDB: Van den Bergen (A Fast and
/// Robust GJK Implementation, 1999) DG: duality-gap, as used in the Frank-Wolfe
/// and the vanilla 1988 GJK algorithms Hybrid: a mix between VDB and DG.
enum GJKConvergenceCriterion { Default, DualityGap, Hybrid };

/// @brief Wether the convergence criterion is scaled on the norm of the
/// solution or not
enum GJKConvergenceCriterionType { Relative, Absolute };

/// @brief Triangle with 3 indices for points
template <typename _IndexType>
class TriangleTpl {
 public:
  // clang-format off
  COAL_DEPRECATED_MESSAGE(Use IndexType instead.) typedef _IndexType index_type;
  // clang-format on
  typedef _IndexType IndexType;
  typedef int size_type;

  template <typename OtherIndexType>
  friend class TriangleTpl;

  /// @brief Default constructor
  TriangleTpl() {}

  /// @brief Copy constructor
  TriangleTpl(const TriangleTpl& other) { *this = other; }

  /// @brief Create a triangle with given vertex indices
  TriangleTpl(IndexType p1, IndexType p2, IndexType p3) { set(p1, p2, p3); }

  /// @brief Copy constructor from another vertex index type.
  template <typename OtherIndexType>
  TriangleTpl(const TriangleTpl<OtherIndexType>& other) {
    *this = other;
  }

  /// @brief Copy operator
  TriangleTpl& operator=(const TriangleTpl& other) {
    this->set(other.vids[0], other.vids[1], other.vids[2]);
    return *this;
  }

  /// @brief Copy operator from another index type.
  template <typename OtherIndexType>
  TriangleTpl& operator=(const TriangleTpl<OtherIndexType>& other) {
    *this = other.template cast<OtherIndexType>();
    return *this;
  }

  template <typename OtherIndexType>
  TriangleTpl<OtherIndexType> cast() const {
    TriangleTpl<OtherIndexType> res;
    res.set(OtherIndexType(this->vids[0]), OtherIndexType(this->vids[1]),
            OtherIndexType(this->vids[2]));
    return res;
  }

  /// @brief Set the vertex indices of the triangle
  inline void set(IndexType p1, IndexType p2, IndexType p3) {
    vids[0] = p1;
    vids[1] = p2;
    vids[2] = p3;
  }

  /// @brief Access the triangle index
  inline IndexType operator[](IndexType i) const { return vids[i]; }

  inline IndexType& operator[](IndexType i) { return vids[i]; }

  static inline size_type size() { return 3; }

  bool operator==(const TriangleTpl& other) const {
    return vids[0] == other.vids[0] && vids[1] == other.vids[1] &&
           vids[2] == other.vids[2];
  }

  bool operator!=(const TriangleTpl& other) const { return !(*this == other); }

  bool isValid() const {
    return vids[0] != (std::numeric_limits<IndexType>::max)() &&
           vids[1] != (std::numeric_limits<IndexType>::max)() &&
           vids[2] != (std::numeric_limits<IndexType>::max)();
  }

 protected:
  /// @brief indices for each vertex of triangle
  IndexType vids[3] = {(std::numeric_limits<IndexType>::max)(),
                       (std::numeric_limits<IndexType>::max)(),
                       (std::numeric_limits<IndexType>::max)()};
};

typedef TriangleTpl<std::uint16_t> Triangle16;
//
typedef TriangleTpl<std::uint32_t> Triangle32;
//
COAL_DEPRECATED_MESSAGE(Use Triangle32 instead.)
typedef Triangle32 Triangle;

/// @brief Quadrilateral with 4 indices for points
template <typename _IndexType>
struct QuadrilateralTpl {
  // clang-format off
  COAL_DEPRECATED_MESSAGE(Use IndexType instead.) typedef _IndexType index_type;
  // clang-format on
  typedef _IndexType IndexType;
  typedef int size_type;

  /// @brief Default constructor
  QuadrilateralTpl() {}

  /// @brief Copy constructor
  QuadrilateralTpl(const QuadrilateralTpl& other) { *this = other; }

  /// @brief Copy constructor from another vertex index type.
  template <typename OtherIndexType>
  QuadrilateralTpl(const QuadrilateralTpl<OtherIndexType>& other) {
    *this = other;
  }

  /// @brief Copy operator
  QuadrilateralTpl& operator=(const QuadrilateralTpl& other) {
    this->set(other.vids[0], other.vids[1], other.vids[2], other.vids[3]);
    return *this;
  }

  /// @brief Copy operator from another index type.
  template <typename OtherIndexType>
  QuadrilateralTpl& operator=(const QuadrilateralTpl<OtherIndexType>& other) {
    *this = other.template cast<OtherIndexType>();
    return *this;
  }

  template <typename OtherIndexType>
  QuadrilateralTpl<OtherIndexType> cast() const {
    QuadrilateralTpl<OtherIndexType> res;
    res.set(OtherIndexType(this->vids[0]), OtherIndexType(this->vids[1]),
            OtherIndexType(this->vids[2]), OtherIndexType(this->vids[3]));
    return res;
  }

  QuadrilateralTpl(IndexType p0, IndexType p1, IndexType p2, IndexType p3) {
    set(p0, p1, p2, p3);
  }

  /// @brief Set the vertex indices of the quadrilateral
  inline void set(IndexType p0, IndexType p1, IndexType p2, IndexType p3) {
    vids[0] = p0;
    vids[1] = p1;
    vids[2] = p2;
    vids[3] = p3;
  }

  /// @access the quadrilateral index
  inline IndexType operator[](IndexType i) const { return vids[i]; }

  inline IndexType& operator[](IndexType i) { return vids[i]; }

  static inline size_type size() { return 4; }

  bool operator==(const QuadrilateralTpl& other) const {
    return vids[0] == other.vids[0] && vids[1] == other.vids[1] &&
           vids[2] == other.vids[2] && vids[3] == other.vids[3];
  }

  bool operator!=(const QuadrilateralTpl& other) const {
    return !(*this == other);
  }

 protected:
  IndexType vids[4];
};

typedef QuadrilateralTpl<std::uint16_t> Quadrilateral16;
//
typedef QuadrilateralTpl<std::uint32_t> Quadrilateral32;
//
COAL_DEPRECATED_MESSAGE(Use Quadrilateral32 instead.)
typedef Quadrilateral32 Quadrilateral;

}  // namespace coal

#endif
