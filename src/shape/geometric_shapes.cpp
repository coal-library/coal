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

#include "coal/shape/geometric_shapes.h"
#include "coal/shape/geometric_shapes_utility.h"

#ifdef COAL_HAS_QHULL
#include <libqhullcpp/QhullError.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullLinkedList.h>
#include <libqhullcpp/QhullVertex.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/QhullRidge.h>
#include <libqhullcpp/Qhull.h>

using orgQhull::Qhull;
using orgQhull::QhullFacet;
using orgQhull::QhullPoint;
using orgQhull::QhullRidgeSet;
using orgQhull::QhullVertexList;
using orgQhull::QhullVertexSet;
#endif

namespace coal {

template <typename IndexType>
ConvexBaseTpl<IndexType>* ConvexBaseTpl<IndexType>::convexHull(
    std::shared_ptr<std::vector<Vec3s>>& pts, unsigned int num_points,
    bool keepTriangles, const char* qhullCommand) {
  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  return ConvexBaseTpl<IndexType>::convexHull(pts->data(), num_points,
                                              keepTriangles, qhullCommand);
  COAL_COMPILER_DIAGNOSTIC_POP
}
template COAL_DLLAPI ConvexBaseTpl<Triangle16::IndexType>*
ConvexBaseTpl<Triangle16::IndexType>::convexHull(
    std::shared_ptr<std::vector<Vec3s>>& pts, unsigned int num_points,
    bool keepTriangles, const char* qhullCommand);
template COAL_DLLAPI ConvexBaseTpl<Triangle32::IndexType>*
ConvexBaseTpl<Triangle32::IndexType>::convexHull(
    std::shared_ptr<std::vector<Vec3s>>& pts, unsigned int num_points,
    bool keepTriangles, const char* qhullCommand);

template <typename IndexType>
ConvexBaseTpl<IndexType>* ConvexBaseTpl<IndexType>::convexHull(
    const Vec3s* pts, unsigned int num_points, bool keepTriangles,
    const char* qhullCommand) {
#ifdef COAL_HAS_QHULL
  if (num_points <= 3) {
    COAL_THROW_PRETTY(
        "You shouldn't use this function with less than"
        " 4 points.",
        std::invalid_argument);
  }
  assert(pts[0].data() + 3 == pts[1].data());

  Qhull qh;
  const char* command =
      qhullCommand ? qhullCommand : (keepTriangles ? "Qt" : "");

  // TODO: add a ifdef not double precision here
  using Vec3d = Eigen::Vector3d;
  std::vector<Vec3d> qhull_pts;
  qhull_pts.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    qhull_pts.push_back(pts[i].template cast<double>());
  }
  qh.runQhull("", 3, static_cast<int>(num_points), qhull_pts[0].data(),
              command);

  if (qh.qhullStatus() != qh_ERRnone) {
    if (qh.hasQhullMessage()) std::cerr << qh.qhullMessage() << std::endl;
    COAL_THROW_PRETTY("Qhull failed", std::logic_error);
  }

  typedef int size_type;

  // Map index in pts to index in vertices. -1 means not used
  std::vector<int> pts_to_vertices(num_points, -1);

  // Initialize the vertices
  size_t nvertex = static_cast<size_t>(qh.vertexCount());
  if (nvertex >= size_t(std::numeric_limits<IndexType>::max())) {
    COAL_THROW_PRETTY("nvertex >= std::numeric_limits<IndexType>::max()",
                      std::runtime_error);
  }
  std::shared_ptr<std::vector<Vec3s>> vertices(
      new std::vector<Vec3s>(size_t(nvertex)));
  QhullVertexList vertexList(qh.vertexList());
  size_t i_vertex = 0;
  for (QhullVertexList::const_iterator v = vertexList.begin();
       v != vertexList.end(); ++v) {
    QhullPoint pt((*v).point());
    pts_to_vertices[(size_t)pt.id()] = (int)i_vertex;
    (*vertices)[i_vertex] = Vec3s(Scalar(pt[0]), Scalar(pt[1]), Scalar(pt[2]));
    ++i_vertex;
  }
  assert(i_vertex == nvertex);

  ConvexTpl<TriangleTpl<IndexType>>* convex_tri(NULL);
  ConvexBaseTpl<IndexType>* convex(NULL);
  if (keepTriangles)
    convex = convex_tri = new ConvexTpl<TriangleTpl<IndexType>>();
  else
    convex = new ConvexBaseTpl<IndexType>;
  convex->initialize(vertices, static_cast<unsigned int>(nvertex));

  // Build the neighbors
  convex->neighbors.reset(new std::vector<Neighbors>(size_t(nvertex)));
  std::vector<std::set<IndexType>> nneighbors(static_cast<size_t>(nvertex));
  if (keepTriangles) {
    convex_tri->num_polygons = static_cast<unsigned int>(qh.facetCount());
    convex_tri->polygons.reset(
        new std::vector<TriangleTpl<IndexType>>(convex_tri->num_polygons));
    convex_tri->computeCenter();
  }

  unsigned int c_nneighbors = 0;
  unsigned int i_polygon = 0;

  // TODO: make sure number of vertices < size of IndexType

  // Compute the neighbors from the edges of the faces.
  for (QhullFacet facet = qh.beginFacet(); facet != qh.endFacet();
       facet = facet.next()) {
    if (facet.isSimplicial()) {
      // In 3D, simplicial faces have 3 vertices. We mark them as neighbors.
      QhullVertexSet f_vertices(facet.vertices());
      IndexType n = static_cast<IndexType>(f_vertices.count());
      assert(n == 3);
      TriangleTpl<IndexType> tri(
          static_cast<IndexType>(
              pts_to_vertices[static_cast<size_t>(f_vertices[0].point().id())]),
          static_cast<IndexType>(
              pts_to_vertices[static_cast<size_t>(f_vertices[1].point().id())]),
          static_cast<IndexType>(pts_to_vertices[static_cast<size_t>(
              f_vertices[2].point().id())]));
      if (keepTriangles) {
        reorderTriangle(convex_tri, tri);
        (*convex_tri->polygons)[i_polygon++] = tri;
      }
      for (IndexType j = 0; j < static_cast<IndexType>(n); ++j) {
        IndexType i = (j == 0) ? n - 1 : j - 1;
        IndexType k = (j == n - 1) ? 0 : j + 1;
        // Update neighbors of pj;
        if (nneighbors[tri[j]].insert(tri[i]).second) c_nneighbors++;
        if (nneighbors[tri[j]].insert(tri[k]).second) c_nneighbors++;
      }
    } else {
      if (keepTriangles) {  // TODO I think there is a memory leak here.
        COAL_THROW_PRETTY(
            "You requested to keep triangles so you "
            "must pass option \"Qt\" to qhull via the qhull command argument.",
            std::invalid_argument);
      }
      // Non-simplicial faces have more than 3 vertices and contains a list of
      // rigdes. Ridges are (3-1)D simplex (i.e. one edge). We mark the two
      // vertices of each ridge as neighbors.
      QhullRidgeSet f_ridges(facet.ridges());
      for (size_type j = 0; j < f_ridges.count(); ++j) {
        assert(f_ridges[j].vertices().count() == 2);
        int pi = pts_to_vertices[static_cast<size_t>(
                f_ridges[j].vertices()[0].point().id())],
            pj = pts_to_vertices[static_cast<size_t>(
                f_ridges[j].vertices()[1].point().id())];
        // Update neighbors of pi and pj;
        if (nneighbors[static_cast<size_t>(pj)]
                .insert(static_cast<IndexType>(pi))
                .second)
          c_nneighbors++;
        if (nneighbors[static_cast<size_t>(pi)]
                .insert(static_cast<IndexType>(pj))
                .second)
          c_nneighbors++;
      }
    }
  }
  assert(!keepTriangles || static_cast<int>(i_polygon) == qh.facetCount());

  // Build the double representation (free in this case because qhull has
  // alreday run)
  convex->buildDoubleDescriptionFromQHullResult(qh);

  // Fill the neighbor attribute of the returned object.
  convex->nneighbors_.reset(new std::vector<IndexType>(c_nneighbors));
  std::vector<Neighbors>& neighbors_ = *(convex->neighbors);
  std::vector<IndexType>& nneighbors_ = *(convex->nneighbors_);
  IndexType begin_id = 0;
  for (size_t i = 0; i < static_cast<size_t>(nvertex); ++i) {
    Neighbors& n = neighbors_[i];
    if (nneighbors[i].size() >= (std::numeric_limits<unsigned char>::max)())
      COAL_THROW_PRETTY("Too many neighbors.", std::logic_error);
    n.count = (unsigned char)nneighbors[i].size();
    n.begin_id = begin_id;
    IndexType j = 0;
    for (IndexType idx : nneighbors[i]) {
      nneighbors_[n.begin_id + j] = idx;
      j++;
    }
    begin_id += n.count;
  }

  // Now that the neighbors are computed, we can call the
  // `buildSupportWarmStart` function.
  convex->buildSupportWarmStart();
  return convex;
#else
  COAL_THROW_PRETTY(
      "Library built without qhull. Cannot build object of this type.",
      std::logic_error);
  COAL_UNUSED_VARIABLE(pts);
  COAL_UNUSED_VARIABLE(num_points);
  COAL_UNUSED_VARIABLE(keepTriangles);
  COAL_UNUSED_VARIABLE(qhullCommand);
#endif
}
template COAL_DLLAPI ConvexBaseTpl<Triangle16::IndexType>*
ConvexBaseTpl<Triangle16::IndexType>::convexHull(const Vec3s* pts,
                                                 unsigned int num_points,
                                                 bool keepTriangles,
                                                 const char* qhullCommand);
template COAL_DLLAPI ConvexBaseTpl<Triangle32::IndexType>*
ConvexBaseTpl<Triangle32::IndexType>::convexHull(const Vec3s* pts,
                                                 unsigned int num_points,
                                                 bool keepTriangles,
                                                 const char* qhullCommand);

#ifdef COAL_HAS_QHULL
template <typename IndexType>
void ConvexBaseTpl<IndexType>::buildDoubleDescription() {
  if (num_points <= 3) {
    COAL_THROW_PRETTY(
        "You shouldn't use this function with a convex less than"
        " 4 points.",
        std::invalid_argument);
  }

  Qhull qh;
  const char* command = "Qt";
  using Vec3d = Eigen::Vector3d;
  std::vector<Vec3d> qhull_pts;
  qhull_pts.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    qhull_pts.push_back((*points)[i].template cast<double>());
  }
  qh.runQhull("", 3, static_cast<int>(num_points), qhull_pts[0].data(),
              command);

  if (qh.qhullStatus() != qh_ERRnone) {
    if (qh.hasQhullMessage()) std::cerr << qh.qhullMessage() << std::endl;
    COAL_THROW_PRETTY("Qhull failed", std::logic_error);
  }

  buildDoubleDescriptionFromQHullResult(qh);
}
template void COAL_DLLAPI
ConvexBaseTpl<Triangle16::IndexType>::buildDoubleDescription();
template void COAL_DLLAPI
ConvexBaseTpl<Triangle32::IndexType>::buildDoubleDescription();

template <typename IndexType>
void ConvexBaseTpl<IndexType>::buildDoubleDescriptionFromQHullResult(
    const Qhull& qh) {
  num_normals_and_offsets = static_cast<unsigned int>(qh.facetCount());
  normals.reset(new std::vector<Vec3s>(num_normals_and_offsets));
  std::vector<Vec3s>& normals_ = *normals;
  offsets.reset(new std::vector<Scalar>(num_normals_and_offsets));
  std::vector<Scalar>& offsets_ = *offsets;
  unsigned int i_normal = 0;
  for (QhullFacet facet = qh.beginFacet(); facet != qh.endFacet();
       facet = facet.next()) {
    const orgQhull::QhullHyperplane& plane = facet.hyperplane();
    normals_[i_normal] =
        Vec3s(Scalar(plane.coordinates()[0]), Scalar(plane.coordinates()[1]),
              Scalar(plane.coordinates()[2]));
    offsets_[i_normal] = Scalar(plane.offset());
    i_normal++;
  }
  assert(static_cast<int>(i_normal) == qh.facetCount());
}
template void COAL_DLLAPI
ConvexBaseTpl<Triangle16::IndexType>::buildDoubleDescriptionFromQHullResult(
    const Qhull& qh);
template void COAL_DLLAPI
ConvexBaseTpl<Triangle32::IndexType>::buildDoubleDescriptionFromQHullResult(
    const Qhull& qh);
#endif

void Halfspace::unitNormalTest() {
  Scalar l = n.norm();
  if (l > 0) {
    Scalar inv_l = Scalar(1) / l;
    n *= inv_l;
    d *= inv_l;
  } else {
    n << 1, 0, 0;
    d = 0;
  }
}

void Plane::unitNormalTest() {
  Scalar l = n.norm();
  if (l > 0) {
    Scalar inv_l = Scalar(1) / l;
    n *= inv_l;
    d *= inv_l;
  } else {
    n << 1, 0, 0;
    d = 0;
  }
}

void Box::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Sphere::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = radius;
}

void Ellipsoid::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Capsule::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Cone::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Cylinder::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Halfspace::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void Plane::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

void TriangleP::computeLocalAABB() {
  computeBV<AABB>(*this, Transform3s(), aabb_local);
  const Scalar ssr = this->getSweptSphereRadius();
  if (ssr > 0) {
    aabb_local.min_ -= Vec3s::Constant(ssr);
    aabb_local.max_ += Vec3s::Constant(ssr);
  }
  aabb_center = aabb_local.center();
  aabb_radius = (aabb_local.min_ - aabb_center).norm();
}

}  // namespace coal
