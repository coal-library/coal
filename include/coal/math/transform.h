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

#ifndef COAL_TRANSFORM_H
#define COAL_TRANSFORM_H

#include "coal/fwd.hh"
#include "coal/data_types.h"

namespace coal {

COAL_DEPRECATED typedef Eigen::Quaternion<Scalar> Quaternion3f;
typedef Eigen::Quaternion<Scalar> Quats;

static inline std::ostream& operator<<(std::ostream& o, const Quats& q) {
  o << "(" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << ")";
  return o;
}

/// @brief Simple transform class used locally by InterpMotion
class COAL_DLLAPI Transform3s {
  /// @brief Matrix cache
  Matrix3s R;

  /// @brief Translation vector
  Vec3s T;

 public:
  /// @brief Default transform is no movement
  Transform3s() {
    setIdentity();  // set matrix_set true
  }

  static Transform3s Identity() { return Transform3s(); }

  /// @brief Construct transform from rotation and translation
  template <typename Matrixx3Like, typename Vector3Like>
  Transform3s(const Eigen::MatrixBase<Matrixx3Like>& R_,
              const Eigen::MatrixBase<Vector3Like>& T_)
      : R(R_), T(T_) {}

  /// @brief Construct transform from rotation and translation
  template <typename Vector3Like>
  Transform3s(const Quats& q_, const Eigen::MatrixBase<Vector3Like>& T_)
      : R(q_.toRotationMatrix()), T(T_) {}

  /// @brief Construct transform from rotation
  Transform3s(const Matrix3s& R_) : R(R_), T(Vec3s::Zero()) {}

  /// @brief Construct transform from rotation
  Transform3s(const Quats& q_) : R(q_), T(Vec3s::Zero()) {}

  /// @brief Construct transform from translation
  Transform3s(const Vec3s& T_) : R(Matrix3s::Identity()), T(T_) {}

  /// @brief Construct transform from other transform
  Transform3s(const Transform3s& tf) : R(tf.R), T(tf.T) {}

  /// @brief operator =
  Transform3s& operator=(const Transform3s& tf) {
    R = tf.R;
    T = tf.T;
    return *this;
  }

  /// @brief get translation
  inline const Vec3s& getTranslation() const { return T; }

  /// @brief get translation
  inline const Vec3s& translation() const { return T; }

  /// @brief get translation
  inline Vec3s& translation() { return T; }

  /// @brief get rotation
  inline const Matrix3s& getRotation() const { return R; }

  /// @brief get rotation
  inline const Matrix3s& rotation() const { return R; }

  /// @brief get rotation
  inline Matrix3s& rotation() { return R; }

  /// @brief get quaternion
  inline Quats getQuatRotation() const { return Quats(R); }

  /// @brief set transform from rotation and translation
  template <typename Matrix3Like, typename Vector3Like>
  inline void setTransform(const Eigen::MatrixBase<Matrix3Like>& R_,
                           const Eigen::MatrixBase<Vector3Like>& T_) {
    R.noalias() = R_;
    T.noalias() = T_;
  }

  /// @brief set transform from rotation and translation
  inline void setTransform(const Quats& q_, const Vec3s& T_) {
    R = q_.toRotationMatrix();
    T = T_;
  }

  /// @brief set transform from rotation
  template <typename Derived>
  inline void setRotation(const Eigen::MatrixBase<Derived>& R_) {
    R.noalias() = R_;
  }

  /// @brief set transform from translation
  template <typename Derived>
  inline void setTranslation(const Eigen::MatrixBase<Derived>& T_) {
    T.noalias() = T_;
  }

  /// @brief set transform from rotation
  inline void setQuatRotation(const Quats& q_) { R = q_.toRotationMatrix(); }

  /// @brief transform a given vector by the transform
  template <typename Derived>
  inline Vec3s transform(const Eigen::MatrixBase<Derived>& v) const {
    return R * v + T;
  }

  /// @brief transform a given vector by the inverse of the transform
  template <typename Derived>
  inline Vec3s inverseTransform(const Eigen::MatrixBase<Derived>& v) const {
    return R.transpose() * (v - T);
  }

  /// @brief inverse transform
  inline Transform3s& inverseInPlace() {
    R.transposeInPlace();
    T = -R * T;
    return *this;
  }

  /// @brief inverse transform
  inline Transform3s inverse() {
    return Transform3s(R.transpose(), -R.transpose() * T);
  }

  /// @brief inverse the transform and multiply with another
  inline Transform3s inverseTimes(const Transform3s& other) const {
    return Transform3s(R.transpose() * other.R, R.transpose() * (other.T - T));
  }

  /// @brief multiply with another transform
  inline const Transform3s& operator*=(const Transform3s& other) {
    T += R * other.T;
    R *= other.R;
    return *this;
  }

  /// @brief multiply with another transform
  inline Transform3s operator*(const Transform3s& other) const {
    return Transform3s(R * other.R, R * other.T + T);
  }

  /// @brief check whether the transform is identity
  inline bool isIdentity(
      const Scalar& prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
    return R.isIdentity(prec) && T.isZero(prec);
  }

  /// @brief set the transform to be identity transform
  inline void setIdentity() {
    R.setIdentity();
    T.setZero();
  }

  /// @brief return a random transform
  static Transform3s Random() {
    Transform3s tf = Transform3s();
    tf.setRandom();
    return tf;
  }

  /// @brief set the transform to a random transform
  inline void setRandom();

  bool operator==(const Transform3s& other) const {
    return (R == other.getRotation()) && (T == other.getTranslation());
  }

  bool operator!=(const Transform3s& other) const { return !(*this == other); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename Derived>
inline Quats fromAxisAngle(const Eigen::MatrixBase<Derived>& axis,
                           Scalar angle) {
  return Quats(Eigen::AngleAxis<Scalar>(angle, axis));
}

/// @brief Uniformly random quaternion sphere.
/// Code taken from Pinocchio (https://github.com/stack-of-tasks/pinocchio).
inline Quats uniformRandomQuaternion() {
  // Rotational part
  const Scalar u1 = (Scalar)rand() / Scalar(RAND_MAX);
  const Scalar u2 = (Scalar)rand() / Scalar(RAND_MAX);
  const Scalar u3 = (Scalar)rand() / Scalar(RAND_MAX);

  const Scalar mult1 = std::sqrt(Scalar(1.0) - u1);
  const Scalar mult2 = std::sqrt(u1);

  static const Scalar PI_value = static_cast<Scalar>(EIGEN_PI);
  Scalar s2 = std::sin(2 * PI_value * u2);
  Scalar c2 = std::cos(2 * PI_value * u2);
  Scalar s3 = std::sin(2 * PI_value * u3);
  Scalar c3 = std::cos(2 * PI_value * u3);

  Quats q;
  q.w() = mult1 * s2;
  q.x() = mult1 * c2;
  q.y() = mult2 * s3;
  q.z() = mult2 * c3;
  return q;
}

inline void Transform3s::setRandom() {
  const Quats q = uniformRandomQuaternion();
  this->rotation() = q.matrix();
  this->translation().setRandom();
}

/// @brief Construct othonormal basis from vector.
/// The z-axis is the normalized input vector.
inline Matrix3s constructOrthonormalBasisFromVector(const Vec3s& vec) {
  Matrix3s basis = Matrix3s::Zero();
  basis.col(2) = vec.normalized();
  basis.col(1) = -vec.unitOrthogonal();
  basis.col(0) = basis.col(1).cross(vec);
  return basis;
}

}  // namespace coal

#endif
