/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021-2024, INRIA
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

#ifndef COAL_GJK_H
#define COAL_GJK_H

#include <vector>

#include "coal/narrowphase/minkowski_difference.h"

namespace coal {

namespace details {

/// @brief class for GJK algorithm
///
/// @note The computations are performed in the frame of the first shape.
struct COAL_DLLAPI GJK {
  struct COAL_DLLAPI SimplexV {
    /// @brief support vector for shape 0 and 1.
    Vec3ps w0, w1;
    /// @brief support vector (i.e., the furthest point on the shape along the
    /// support direction)
    Vec3ps w;
  };

  typedef unsigned char vertex_id_t;

  /// @brief A simplex is a set of up to 4 vertices.
  /// Its rank is the number of vertices it contains.
  /// @note This data structure does **not** own the vertices it refers to.
  /// To be efficient, the constructor of `GJK` creates storage for 4 vertices.
  /// Since GJK does not need any more storage, it reuses these vertices
  /// throughout the algorithm by using multiple instance of this `Simplex`
  /// class.
  struct COAL_DLLAPI Simplex {
    /// @brief simplex vertex
    SimplexV* vertex[4];
    /// @brief size of simplex (number of vertices)
    vertex_id_t rank;

    Simplex() {}

    void reset() {
      rank = 0;
      for (size_t i = 0; i < 4; ++i) vertex[i] = nullptr;
    }
  };

  /// @brief Status of the GJK algorithm:
  /// DidNotRun: GJK has not been run.
  /// Failed: GJK did not converge (it exceeded the maximum number of
  /// iterations).
  /// NoCollisionEarlyStopped: GJK found a separating hyperplane and exited
  ///     before converting. The shapes are not in collision.
  /// NoCollision: GJK converged and the shapes are not in collision.
  /// Collision: GJK converged and the shapes are in collision.
  /// Failed: GJK did not converge.
  enum Status {
    DidNotRun,
    Failed,
    NoCollisionEarlyStopped,
    NoCollision,
    CollisionWithPenetrationInformation,
    Collision
  };

 public:
  SolverScalar distance_upper_bound;
  Status status;
  GJKVariant gjk_variant;
  GJKConvergenceCriterion convergence_criterion;
  GJKConvergenceCriterionType convergence_criterion_type;

  MinkowskiDiff const* shape;
  Vec3ps ray;
  support_func_guess_t support_hint;
  /// @brief The distance between the two shapes, computed by GJK.
  /// If the distance is below GJK's threshold, the shapes are in collision in
  /// the eyes of GJK. If `distance_upper_bound` is set to a value lower than
  /// infinity, GJK will early stop as soon as it finds `distance` to be greater
  /// than `distance_upper_bound`.
  SolverScalar distance;
  Simplex* simplex;  // Pointer to the result of the last run of GJK.

 private:
  // max_iteration and tolerance are made private
  // because they are meant to be set by the `reset` function.
  size_t max_iterations;
  SolverScalar tolerance;

  SimplexV store_v[4];
  SimplexV* free_v[4];
  vertex_id_t nfree;
  vertex_id_t current;
  Simplex simplices[2];
  size_t iterations;
  size_t iterations_momentum_stop;

 public:
  /// \param max_iterations_ number of iteration before GJK returns failure.
  /// \param tolerance_ precision of the algorithm.
  ///
  /// The tolerance argument is useful for continuous shapes and for polyhedron
  /// with some vertices closer than this threshold.
  ///
  /// Suggested values are 100 iterations and a tolerance of 1e-6.
  GJK(size_t max_iterations_, SolverScalar tolerance_)
      : max_iterations(max_iterations_), tolerance(tolerance_) {
    COAL_ASSERT(tolerance_ > 0, "Tolerance must be positive.",
                std::invalid_argument);
    initialize();
  }

  /// @brief resets the GJK algorithm, preparing it for a new run.
  /// Other than the maximum number of iterations and the tolerance,
  /// this function does **not** modify the parameters of the GJK algorithm.
  void reset(size_t max_iterations_, SolverScalar tolerance_);

  /// @brief GJK algorithm, given the initial value guess
  Status evaluate(
      const MinkowskiDiff& shape, const Vec3ps& guess,
      const support_func_guess_t& supportHint = support_func_guess_t::Zero());

  /// @brief apply the support function along a direction, the result is return
  /// in sv
  inline void getSupport(const Vec3ps& d, SimplexV& sv,
                         support_func_guess_t& hint) const {
    Vec3s w0, w1;
    shape->support(d.cast<Scalar>(), w0, w1, hint);
    sv.w0 = w0.cast<SolverScalar>();
    sv.w1 = w1.cast<SolverScalar>();
    sv.w = sv.w0 - sv.w1;
  }

  /// @brief whether the simplex enclose the origin
  bool encloseOrigin();

  /// @brief get the underlying simplex using in GJK, can be used for cache in
  /// next iteration
  inline Simplex* getSimplex() const { return simplex; }

  /// Tells whether the closest points are available.
  bool hasClosestPoints() const { return distance < distance_upper_bound; }

  /// Get the witness points on each object, and the corresponding normal.
  /// @param[in] shape is the Minkowski difference of the two shapes.
  /// @param[out] w0 is the witness point on shape0.
  /// @param[out] w1 is the witness point on shape1.
  /// @param[out] normal is the normal of the separating plane found by
  /// GJK. It points from shape0 to shape1.
  void getWitnessPointsAndNormal(const MinkowskiDiff& shape, Vec3ps& w0,
                                 Vec3ps& w1, Vec3ps& normal) const;

  /// @brief get the guess from current simplex
  Vec3ps getGuessFromSimplex() const;

  /// @brief Distance threshold for early break.
  /// GJK stops when it proved the distance is more than this threshold.
  /// @note The closest points will be erroneous in this case.
  ///       If you want the closest points, set this to infinity (the default).
  void setDistanceEarlyBreak(const SolverScalar& dup) {
    distance_upper_bound = dup;
  }

  /// @brief Convergence check used to stop GJK when shapes are not in
  /// collision.
  bool checkConvergence(const Vec3ps& w, const SolverScalar& rl,
                        SolverScalar& alpha, const SolverScalar& omega) const;

  /// @brief Get the max number of iterations of GJK.
  size_t getNumMaxIterations() const { return max_iterations; }

  /// @brief Get the tolerance of GJK.
  SolverScalar getTolerance() const { return tolerance; }

  /// @brief Get the number of iterations of the last run of GJK.
  size_t getNumIterations() const { return iterations; }

  /// @brief Get GJK number of iterations before momentum stops.
  /// Only usefull if the Nesterov or Polyak acceleration activated.
  size_t getNumIterationsMomentumStopped() const {
    return iterations_momentum_stop;
  }

 private:
  /// @brief Initializes the GJK algorithm.
  /// This function should only be called by the constructor.
  /// Otherwise use \ref reset.
  void initialize();

  /// @brief discard one vertex from the simplex
  inline void removeVertex(Simplex& simplex);

  /// @brief append one vertex to the simplex
  inline void appendVertex(Simplex& simplex, const Vec3ps& v,
                           support_func_guess_t& hint);

  /// @brief Project origin (0) onto line a-b
  /// For a detailed explanation of how to efficiently project onto a simplex,
  /// check out Ericson's book, page 403:
  /// https://realtimecollisiondetection.net/ To sum up, a simplex has a voronoi
  /// region for each feature it has (vertex, edge, face). We find the voronoi
  /// region in which the origin lies and stop as soon as we find it; we then
  /// project onto it and return the result. We start by voronoi regions
  /// generated by vertices then move on to edges then faces. Checking voronoi
  /// regions is done using simple dot products. Moreover, edges voronoi checks
  /// reuse computations of vertices voronoi checks. The same goes for faces
  /// which reuse checks from edges.
  /// Finally, in addition to the voronoi procedure, checks relying on the order
  /// of construction
  /// of the simplex are added. To know more about these, visit
  /// https://caseymuratori.com/blog_0003.
  bool projectLineOrigin(const Simplex& current, Simplex& next);

  /// @brief Project origin (0) onto triangle a-b-c
  /// See \ref projectLineOrigin for an explanation on simplex projections.
  bool projectTriangleOrigin(const Simplex& current, Simplex& next);

  /// @brief Project origin (0) onto tetrahedron a-b-c-d
  /// See \ref projectLineOrigin for an explanation on simplex projections.
  bool projectTetrahedraOrigin(const Simplex& current, Simplex& next);
};

/// @brief class for EPA algorithm
struct COAL_DLLAPI EPA {
  typedef GJK::SimplexV SimplexVertex;
  struct COAL_DLLAPI SimplexFace {
    Vec3ps n;
    SolverScalar d;
    bool ignore;          // If the origin does not project inside the face, we
                          // ignore this face.
    size_t vertex_id[3];  // Index of vertex in sv_store.
    SimplexFace* adjacent_faces[3];  // A face has three adjacent faces.
    SimplexFace* prev_face;          // The previous face in the list.
    SimplexFace* next_face;          // The next face in the list.
    size_t adjacent_edge[3];         // Each face has 3 edges: `0`, `1` and `2`.
                              // The i-th adjacent face is bound (to this face)
                              // along its `adjacent_edge[i]`-th edge
                              // (with 0 <= i <= 2).
    size_t pass;

    SimplexFace() : n(Vec3ps::Zero()), ignore(false) {};
  };

  /// @brief The simplex list of EPA is a linked list of faces.
  /// Note: EPA's linked list does **not** own any memory.
  /// The memory it refers to is contiguous and owned by a std::vector.
  struct COAL_DLLAPI SimplexFaceList {
    SimplexFace* root;
    size_t count;
    SimplexFaceList() : root(nullptr), count(0) {}

    void reset() {
      root = nullptr;
      count = 0;
    }

    void append(SimplexFace* face) {
      face->prev_face = nullptr;
      face->next_face = root;
      if (root != nullptr) root->prev_face = face;
      root = face;
      ++count;
    }

    void remove(SimplexFace* face) {
      if (face->next_face != nullptr)
        face->next_face->prev_face = face->prev_face;
      if (face->prev_face != nullptr)
        face->prev_face->next_face = face->next_face;
      if (face == root) root = face->next_face;
      --count;
    }
  };

  /// @brief We bind the face `fa` along its edge `ea` to the face `fb` along
  /// its edge `fb`.
  static inline void bind(SimplexFace* fa, size_t ea, SimplexFace* fb,
                          size_t eb) {
    assert(ea == 0 || ea == 1 || ea == 2);
    assert(eb == 0 || eb == 1 || eb == 2);
    fa->adjacent_edge[ea] = eb;
    fa->adjacent_faces[ea] = fb;
    fb->adjacent_edge[eb] = ea;
    fb->adjacent_faces[eb] = fa;
  }

  struct COAL_DLLAPI SimplexHorizon {
    SimplexFace* current_face;  // current face in the horizon
    SimplexFace* first_face;    // first face in the horizon
    size_t num_faces;           // number of faces in the horizon
    SimplexHorizon()
        : current_face(nullptr), first_face(nullptr), num_faces(0) {}
  };

  enum Status {
    DidNotRun = -1,
    Failed = 0,
    Valid = 1,
    AccuracyReached = 1 << 1 | Valid,
    Degenerated = 1 << 1 | Failed,
    NonConvex = 2 << 1 | Failed,
    InvalidHull = 3 << 1 | Failed,
    OutOfFaces = 4 << 1 | Failed,
    OutOfVertices = 5 << 1 | Failed,
    FallBack = 6 << 1 | Failed
  };

 public:
  Status status;
  GJK::Simplex result;
  Vec3ps normal;
  support_func_guess_t support_hint;
  SolverScalar depth;
  SimplexFace* closest_face;

 private:
  // max_iteration and tolerance are made private
  // because they are meant to be set by the `reset` function.
  size_t max_iterations;
  SolverScalar tolerance;

  std::vector<SimplexVertex> sv_store;
  std::vector<SimplexFace> fc_store;
  SimplexFaceList hull, stock;
  size_t num_vertices;  // number of vertices in polytpoe constructed by EPA
  size_t iterations;

 public:
  EPA(size_t max_iterations_, SolverScalar tolerance_)
      : max_iterations(max_iterations_), tolerance(tolerance_) {
    initialize();
  }

  /// @brief Copy constructor of EPA.
  /// Mostly needed for the copy constructor of `GJKSolver`.
  EPA(const EPA& other)
      : max_iterations(other.max_iterations),
        tolerance(other.tolerance),
        sv_store(other.sv_store),
        fc_store(other.fc_store) {
    initialize();
  }

  /// @brief Get the max number of iterations of EPA.
  size_t getNumMaxIterations() const { return max_iterations; }

  /// @brief Get the max number of vertices of EPA.
  size_t getNumMaxVertices() const { return sv_store.size(); }

  /// @brief Get the max number of faces of EPA.
  size_t getNumMaxFaces() const { return fc_store.size(); }

  /// @brief Get the tolerance of EPA.
  SolverScalar getTolerance() const { return tolerance; }

  /// @brief Get the number of iterations of the last run of EPA.
  size_t getNumIterations() const { return iterations; }

  /// @brief Get the number of vertices in the polytope of the last run of EPA.
  size_t getNumVertices() const { return num_vertices; }

  /// @brief Get the number of faces in the polytope of the last run of EPA.
  size_t getNumFaces() const { return hull.count; }

  /// @brief resets the EPA algorithm, preparing it for a new run.
  /// It potentially reallocates memory for the vertices and faces
  /// if the passed parameters are bigger than the previous ones.
  /// This function does **not** modify the parameters of the EPA algorithm,
  /// i.e. the maximum number of iterations and the tolerance.
  /// @note calling this function destroys the previous state of EPA.
  /// In the future, we may want to copy it instead, i.e. when EPA will
  /// be (properly) warm-startable.
  void reset(size_t max_iterations, SolverScalar tolerance);

  /// \return a Status which can be demangled using (status & Valid) or
  ///         (status & Failed). The other values provide a more detailled
  ///         status
  Status evaluate(GJK& gjk, const Vec3ps& guess);

  /// Get the witness points on each object, and the corresponding normal.
  /// @param[in] shape is the Minkowski difference of the two shapes.
  /// @param[out] w0 is the witness point on shape0.
  /// @param[out] w1 is the witness point on shape1.
  /// @param[in] normal is the normal found by EPA. It points from shape0 to
  /// shape1. The normal is used to correct the witness points on the shapes if
  /// the shapes have a non-zero swept-sphere radius.
  void getWitnessPointsAndNormal(const MinkowskiDiff& shape, Vec3ps& w0,
                                 Vec3ps& w1, Vec3ps& normal) const;

 private:
  /// @brief Allocates memory for the EPA algorithm.
  /// This function should only be called by the constructor.
  /// Otherwise use \ref reset.
  void initialize();

  bool getEdgeDist(SimplexFace* face, const SimplexVertex& a,
                   const SimplexVertex& b, SolverScalar& dist);

  /// @brief Add a new face to the polytope.
  /// This function sets the `ignore` flag to `true` if the origin does not
  /// project inside the face.
  SimplexFace* newFace(size_t id_a, size_t id_b, size_t id_vertex,
                       bool force = false);

  /// @brief Find the best polytope face to split
  SimplexFace* findClosestFace();

  /// @brief the goal is to add a face connecting vertex w and face edge f[e]
  bool expand(size_t pass, const SimplexVertex& w, SimplexFace* f, size_t e,
              SimplexHorizon& horizon);

  // @brief Use this function to debug expand if needed.
  // void PrintExpandLooping(const SimplexFace* f, const SimplexVertex& w);
};

}  // namespace details

}  // namespace coal

#endif
