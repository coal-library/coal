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

#ifndef TEST_COAL_UTILITY_H
#define TEST_COAL_UTILITY_H

#include "coal/math/transform.h"
#include "coal/collision_data.h"
#include "coal/collision_object.h"
#include "coal/broadphase/default_broadphase_callbacks.h"
#include "coal/shape/convex.h"

#ifdef COAL_HAS_OCTOMAP
#include "coal/octree.h"
#endif

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#define EIGEN_VECTOR_IS_APPROX(Va, Vb, precision)                            \
  BOOST_CHECK_MESSAGE(((Va) - (Vb)).isZero(precision),                       \
                      "check " #Va ".isApprox(" #Vb ") failed at precision " \
                          << precision << " [\n"                             \
                          << (Va).transpose() << "\n!=\n"                    \
                          << (Vb).transpose() << "\n]")

#define EIGEN_MATRIX_IS_APPROX(Va, Vb, precision)                            \
  BOOST_CHECK_MESSAGE(((Va) - (Vb)).isZero(precision),                       \
                      "check " #Va ".isApprox(" #Vb ") failed at precision " \
                          << precision << " [\n"                             \
                          << (Va) << "\n!=\n"                                \
                          << (Vb) << "\n]")

#define Scalar_IS_APPROX(Va, Vb, precision)                                  \
  BOOST_CHECK_MESSAGE(std::abs((Va) - (Vb)) < precision,                     \
                      "check " #Va ".isApprox(" #Vb ") failed at precision " \
                          << precision << " [\n"                             \
                          << Va << "\n!=\n"                                  \
                          << Vb << "\n]")

namespace octomap {
#ifdef COAL_HAS_OCTOMAP
typedef coal::shared_ptr<octomap::OcTree> OcTreePtr_t;
#endif
}  // namespace octomap
namespace coal {

class BenchTimer {
 public:
  BenchTimer();
  ~BenchTimer();

  void start();                       ///< start timer
  void stop();                        ///< stop the timer
  double getElapsedTime();            ///< get elapsed time in milli-second
  double getElapsedTimeInSec();       ///< get elapsed time in second (same as
                                      ///< getElapsedTime)
  double getElapsedTimeInMilliSec();  ///< get elapsed time in milli-second
  double getElapsedTimeInMicroSec();  ///< get elapsed time in micro-second

 private:
  double startTimeInMicroSec;  ///< starting time in micro-second
  double endTimeInMicroSec;    ///< ending time in micro-second
  int stopped;                 ///< stop flag
#ifdef _WIN32
  LARGE_INTEGER frequency;  ///< ticks per second
  LARGE_INTEGER startCount;
  LARGE_INTEGER endCount;
#else
  timeval startCount;
  timeval endCount;
#endif
};

struct TStruct {
  std::vector<double> records;
  double overall_time;

  TStruct() { overall_time = 0; }

  void push_back(double t) {
    records.push_back(t);
    overall_time += t;
  }
};

extern const Eigen::IOFormat vfmt;
extern const Eigen::IOFormat pyfmt;
typedef Eigen::AngleAxis<Scalar> AngleAxis;
extern const Vec3s UnitX;
extern const Vec3s UnitY;
extern const Vec3s UnitZ;

/// @brief Load an obj mesh file
void loadOBJFile(const char* filename, std::vector<Vec3s>& points,
                 std::vector<Triangle32>& triangles);

void saveOBJFile(const char* filename, std::vector<Vec3s>& points,
                 std::vector<Triangle32>& triangles);

#ifdef COAL_HAS_OCTOMAP
coal::OcTree loadOctreeFile(const std::string& filename,
                            const Scalar& resolution);
#endif

/// @brief Generate one random transform whose translation is constrained by
/// extents and rotation without constraints. The translation is (x, y, z), and
/// extents[0] <= x <= extents[3], extents[1] <= y <= extents[4], extents[2] <=
/// z <= extents[5]
void generateRandomTransform(Scalar extents[6], Transform3s& transform);

/// @brief Generate n random transforms whose translations are constrained by
/// extents.
void generateRandomTransforms(Scalar extents[6],
                              std::vector<Transform3s>& transforms,
                              std::size_t n);

/// @brief Generate n random transforms whose translations are constrained by
/// extents. Also generate another transforms2 which have additional random
/// translation & rotation to the transforms generated.
void generateRandomTransforms(Scalar extents[6], Scalar delta_trans[3],
                              Scalar delta_rot,
                              std::vector<Transform3s>& transforms,
                              std::vector<Transform3s>& transforms2,
                              std::size_t n);

/// @ brief Structure for minimum distance between two meshes and the
/// corresponding nearest point pair
struct DistanceRes {
  Scalar distance;
  Vec3s p1;
  Vec3s p2;
};

/// @brief Default collision callback for two objects o1 and o2 in broad phase.
/// return value means whether the broad phase can stop now.
bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2,
                              void* cdata);

/// @brief Default distance callback for two objects o1 and o2 in broad phase.
/// return value means whether the broad phase can stop now. also return dist,
/// i.e. the bmin distance till now
bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2,
                             void* cdata, Scalar& dist);

std::string getNodeTypeName(NODE_TYPE node_type);

Quats makeQuat(Scalar w, Scalar x, Scalar y, Scalar z);

std::ostream& operator<<(std::ostream& os, const Transform3s& tf);

/// Get the argument --nb-run from argv
std::size_t getNbRun(const int& argc, char const* const* argv,
                     std::size_t defaultValue);

void generateEnvironments(std::vector<CollisionObject*>& env, Scalar env_scale,
                          std::size_t n);

void generateEnvironmentsMesh(std::vector<CollisionObject*>& env,
                              Scalar env_scale, std::size_t n);

/// @brief Constructs a box with halfsides (l, w, d), centered around 0.
/// The box is 2*l wide on the x-axis, 2*w wide on the y-axis and 2*d wide on
/// the z-axis.
ConvexTpl<Quadrilateral32> buildBox(Scalar l, Scalar w, Scalar d);

/// @brief We give an ellipsoid as input. The output is a 20 faces polytope
/// which vertices belong to the original ellipsoid surface. The procedure is
/// simple: we construct a icosahedron, see
/// https://sinestesia.co/blog/tutorials/python-icospheres/ . We then apply an
/// ellipsoid tranformation to each vertex of the icosahedron.
ConvexTpl<Triangle32> constructPolytopeFromEllipsoid(
    const Ellipsoid& ellipsoid);

Box makeRandomBox(Scalar min_size, Scalar max_size);

Sphere makeRandomSphere(Scalar min_size, Scalar max_size);

Ellipsoid makeRandomEllipsoid(Scalar min_size, Scalar max_size);

Capsule makeRandomCapsule(std::array<Scalar, 2> min_size,
                          std::array<Scalar, 2> max_size);

Cone makeRandomCone(std::array<Scalar, 2> min_size,
                    std::array<Scalar, 2> max_size);

Cylinder makeRandomCylinder(std::array<Scalar, 2> min_size,
                            std::array<Scalar, 2> max_size);

ConvexTpl<Triangle32> makeRandomConvex(Scalar min_size, Scalar max_size);

Plane makeRandomPlane(Scalar min_size, Scalar max_size);

Halfspace makeRandomHalfspace(Scalar min_size, Scalar max_size);

std::shared_ptr<ShapeBase> makeRandomGeometry(NODE_TYPE node_type);

}  // namespace coal

#endif
