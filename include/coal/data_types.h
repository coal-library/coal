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
#ifdef COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
// We keep the FCL_REAL typedef and the Vec[..]f typedefs for backward
// compatibility.
typedef double FCL_REAL;
typedef Eigen::Matrix<FCL_REAL, 3, 1> Vec3f;
typedef Eigen::Matrix<FCL_REAL, 2, 1> Vec2f;
typedef Eigen::Matrix<FCL_REAL, 6, 1> Vec6f;
typedef Eigen::Matrix<FCL_REAL, Eigen::Dynamic, 1> VecXf;
typedef Eigen::Matrix<FCL_REAL, 3, 3> Matrix3f;
typedef Eigen::Matrix<FCL_REAL, Eigen::Dynamic, 3, Eigen::RowMajor> Matrixx3f;
typedef Eigen::Matrix<FCL_REAL, Eigen::Dynamic, 2, Eigen::RowMajor> Matrixx2f;
typedef Eigen::Matrix<FCL_REAL, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;
#endif

typedef double CoalScalar;
typedef Eigen::Matrix<CoalScalar, 3, 1> Vec3s;
typedef Eigen::Matrix<CoalScalar, 2, 1> Vec2s;
typedef Eigen::Matrix<CoalScalar, 6, 1> Vec6s;
typedef Eigen::Matrix<CoalScalar, Eigen::Dynamic, 1> VecXs;
typedef Eigen::Matrix<CoalScalar, 3, 3> Matrix3s;
typedef Eigen::Matrix<CoalScalar, Eigen::Dynamic, 3, Eigen::RowMajor> MatrixX3s;
typedef Eigen::Matrix<CoalScalar, Eigen::Dynamic, 2, Eigen::RowMajor> MatrixX2s;
typedef Eigen::Matrix<Eigen::DenseIndex, Eigen::Dynamic, 3, Eigen::RowMajor>
    Matrixx3i;
typedef Eigen::Matrix<CoalScalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;
typedef Eigen::Vector2i support_func_guess_t;

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
class COAL_DLLAPI Triangle {
 public:
  typedef std::size_t index_type;
  typedef int size_type;

  /// @brief Default constructor
  Triangle() {}

  /// @brief Create a triangle with given vertex indices
  Triangle(index_type p1, index_type p2, index_type p3) { set(p1, p2, p3); }

  /// @brief Set the vertex indices of the triangle
  inline void set(index_type p1, index_type p2, index_type p3) {
    vids[0] = p1;
    vids[1] = p2;
    vids[2] = p3;
  }

  /// @brief Access the triangle index
  inline index_type operator[](index_type i) const { return vids[i]; }

  inline index_type& operator[](index_type i) { return vids[i]; }

  static inline size_type size() { return 3; }

  bool operator==(const Triangle& other) const {
    return vids[0] == other.vids[0] && vids[1] == other.vids[1] &&
           vids[2] == other.vids[2];
  }

  bool operator!=(const Triangle& other) const { return !(*this == other); }

  bool isValid() const {
    return vids[0] != (std::numeric_limits<index_type>::max)() &&
           vids[1] != (std::numeric_limits<index_type>::max)() &&
           vids[2] != (std::numeric_limits<index_type>::max)();
  }

 private:
  /// @brief indices for each vertex of triangle
  index_type vids[3] = {(std::numeric_limits<index_type>::max)(),
                        (std::numeric_limits<index_type>::max)(),
                        (std::numeric_limits<index_type>::max)()};
};

/// @brief Quadrilateral with 4 indices for points
struct COAL_DLLAPI Quadrilateral {
  typedef std::size_t index_type;
  typedef int size_type;

  Quadrilateral() {}

  Quadrilateral(index_type p0, index_type p1, index_type p2, index_type p3) {
    set(p0, p1, p2, p3);
  }

  /// @brief Set the vertex indices of the quadrilateral
  inline void set(index_type p0, index_type p1, index_type p2, index_type p3) {
    vids[0] = p0;
    vids[1] = p1;
    vids[2] = p2;
    vids[3] = p3;
  }

  /// @access the quadrilateral index
  inline index_type operator[](index_type i) const { return vids[i]; }

  inline index_type& operator[](index_type i) { return vids[i]; }

  static inline size_type size() { return 4; }

  bool operator==(const Quadrilateral& other) const {
    return vids[0] == other.vids[0] && vids[1] == other.vids[1] &&
           vids[2] == other.vids[2] && vids[3] == other.vids[3];
  }

  bool operator!=(const Quadrilateral& other) const {
    return !(*this == other);
  }

 private:
  index_type vids[4];
};

}  // namespace coal

#endif
