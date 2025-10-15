#include "coal/shape/convex.h"

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

// Reorders `tri` such that the dot product between the normal of triangle and
// the vector `triangle barycentre - convex_tri.center` is positive.
void reorderTriangle(const Convex<Triangle>* convex_tri, Triangle& tri) {
  Vec3s p0, p1, p2;
  p0 = (*(convex_tri->points))[tri[0]];
  p1 = (*(convex_tri->points))[tri[1]];
  p2 = (*(convex_tri->points))[tri[2]];

  Vec3s barycentre_tri, center_barycenter;
  barycentre_tri = (p0 + p1 + p2) / 3;
  center_barycenter = barycentre_tri - convex_tri->center;

  Vec3s edge_tri1, edge_tri2, n_tri;
  edge_tri1 = p1 - p0;
  edge_tri2 = p2 - p1;
  n_tri = edge_tri1.cross(edge_tri2);

  if (center_barycenter.dot(n_tri) < 0) {
    tri.set(tri[1], tri[0], tri[2]);
  }
}

ConvexBase* ConvexBase::convexHull(std::shared_ptr<std::vector<Vec3s>>& pts,
                                   unsigned int num_points, bool keepTriangles,
                                   const char* qhullCommand) {
  COAL_COMPILER_DIAGNOSTIC_PUSH
  COAL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  return ConvexBase::convexHull(pts->data(), num_points, keepTriangles,
                                qhullCommand);
  COAL_COMPILER_DIAGNOSTIC_POP
}

ConvexBase* ConvexBase::convexHull(const Vec3s* pts, unsigned int num_points,
                                   bool keepTriangles,
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
  qh.runQhull("", 3, static_cast<int>(num_points), pts[0].data(), command);

  if (qh.qhullStatus() != qh_ERRnone) {
    if (qh.hasQhullMessage()) std::cerr << qh.qhullMessage() << std::endl;
    COAL_THROW_PRETTY("Qhull failed", std::logic_error);
  }

  typedef std::size_t index_type;
  typedef int size_type;

  // Map index in pts to index in vertices. -1 means not used
  std::vector<int> pts_to_vertices(num_points, -1);

  // Initialize the vertices
  size_t nvertex = static_cast<size_t>(qh.vertexCount());
  std::shared_ptr<std::vector<Vec3s>> vertices(
      new std::vector<Vec3s>(size_t(nvertex)));
  QhullVertexList vertexList(qh.vertexList());
  size_t i_vertex = 0;
  for (QhullVertexList::const_iterator v = vertexList.begin();
       v != vertexList.end(); ++v) {
    QhullPoint pt((*v).point());
    pts_to_vertices[(size_t)pt.id()] = (int)i_vertex;
    (*vertices)[i_vertex] = Vec3s(pt[0], pt[1], pt[2]);
    ++i_vertex;
  }
  assert(i_vertex == nvertex);

  Convex<Triangle>* convex_tri(NULL);
  ConvexBase* convex(NULL);
  if (keepTriangles)
    convex = convex_tri = new Convex<Triangle>();
  else
    convex = new ConvexBase;
  convex->initialize(vertices, static_cast<unsigned int>(nvertex));

  // Build the neighbors
  convex->neighbors.reset(new std::vector<Neighbors>(size_t(nvertex)));
  std::vector<std::set<index_type>> nneighbors(static_cast<size_t>(nvertex));
  if (keepTriangles) {
    convex_tri->num_polygons = static_cast<unsigned int>(qh.facetCount());
    convex_tri->polygons.reset(
        new std::vector<Triangle>(convex_tri->num_polygons));
    convex_tri->computeCenter();
  }

  unsigned int c_nneighbors = 0;
  unsigned int i_polygon = 0;

  // Compute the neighbors from the edges of the faces.
  for (QhullFacet facet = qh.beginFacet(); facet != qh.endFacet();
       facet = facet.next()) {
    if (facet.isSimplicial()) {
      // In 3D, simplicial faces have 3 vertices. We mark them as neighbors.
      QhullVertexSet f_vertices(facet.vertices());
      size_t n = static_cast<size_t>(f_vertices.count());
      assert(n == 3);
      Triangle tri(
          static_cast<size_t>(
              pts_to_vertices[static_cast<size_t>(f_vertices[0].point().id())]),
          static_cast<size_t>(
              pts_to_vertices[static_cast<size_t>(f_vertices[1].point().id())]),
          static_cast<size_t>(pts_to_vertices[static_cast<size_t>(
              f_vertices[2].point().id())]));
      if (keepTriangles) {
        reorderTriangle(convex_tri, tri);
        (*convex_tri->polygons)[i_polygon++] = tri;
      }
      for (size_t j = 0; j < n; ++j) {
        size_t i = (j == 0) ? n - 1 : j - 1;
        size_t k = (j == n - 1) ? 0 : j + 1;
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
                .insert(static_cast<size_t>(pi))
                .second)
          c_nneighbors++;
        if (nneighbors[static_cast<size_t>(pi)]
                .insert(static_cast<size_t>(pj))
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
  convex->nneighbors_.reset(new std::vector<unsigned int>(c_nneighbors));
  unsigned int* p_nneighbors = convex->nneighbors_->data();
  std::vector<Neighbors>& neighbors_ = *(convex->neighbors);
  for (size_t i = 0; i < static_cast<size_t>(nvertex); ++i) {
    Neighbors& n = neighbors_[i];
    if (nneighbors[i].size() >= (std::numeric_limits<unsigned char>::max)())
      COAL_THROW_PRETTY("Too many neighbors.", std::logic_error);
    n.count_ = (unsigned char)nneighbors[i].size();
    n.n_ = p_nneighbors;
    p_nneighbors =
        std::copy(nneighbors[i].begin(), nneighbors[i].end(), p_nneighbors);
  }
  assert(p_nneighbors == convex->nneighbors_->data() + c_nneighbors);

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

#ifdef COAL_HAS_QHULL
void ConvexBase::buildDoubleDescription() {
  if (num_points <= 3) {
    COAL_THROW_PRETTY(
        "You shouldn't use this function with a convex less than"
        " 4 points.",
        std::invalid_argument);
  }

  Qhull qh;
  const char* command = "Qt";
  qh.runQhull("", 3, static_cast<int>(num_points), (*points)[0].data(),
              command);

  if (qh.qhullStatus() != qh_ERRnone) {
    if (qh.hasQhullMessage()) std::cerr << qh.qhullMessage() << std::endl;
    COAL_THROW_PRETTY("Qhull failed", std::logic_error);
  }

  buildDoubleDescriptionFromQHullResult(qh);
}

void ConvexBase::buildDoubleDescriptionFromQHullResult(const Qhull& qh) {
  num_normals_and_offsets = static_cast<unsigned int>(qh.facetCount());
  normals.reset(new std::vector<Vec3s>(num_normals_and_offsets));
  std::vector<Vec3s>& normals_ = *normals;
  offsets.reset(new std::vector<double>(num_normals_and_offsets));
  std::vector<double>& offsets_ = *offsets;
  unsigned int i_normal = 0;
  for (QhullFacet facet = qh.beginFacet(); facet != qh.endFacet();
       facet = facet.next()) {
    const orgQhull::QhullHyperplane& plane = facet.hyperplane();
    normals_[i_normal] = Vec3s(plane.coordinates()[0], plane.coordinates()[1],
                               plane.coordinates()[2]);
    offsets_[i_normal] = plane.offset();
    i_normal++;
  }
  assert(static_cast<int>(i_normal) == qh.facetCount());
}
#endif

}  // namespace coal
