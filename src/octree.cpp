/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020-2023, INRIA
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

#include "coal/octree.h"

#include <array>

namespace coal {
namespace internal {
struct Neighbors {
  char value;
  Neighbors() : value(0) {}
  bool minusX() const { return value & 0x1; }
  bool plusX() const { return value & 0x2; }
  bool minusY() const { return value & 0x4; }
  bool plusY() const { return value & 0x8; }
  bool minusZ() const { return value & 0x10; }
  bool plusZ() const { return value & 0x20; }
  void hasNeighboordMinusX() { value |= 0x1; }
  void hasNeighboordPlusX() { value |= 0x2; }
  void hasNeighboordMinusY() { value |= 0x4; }
  void hasNeighboordPlusY() { value |= 0x8; }
  void hasNeighboordMinusZ() { value |= 0x10; }
  void hasNeighboordPlusZ() { value |= 0x20; }
};  // struct neighbors

void computeNeighbors(const std::vector<Vec6s>& boxes,
                      std::vector<Neighbors>& neighbors) {
  typedef std::vector<Vec6s> VectorVec6s;
  Scalar fixedSize = -1;
  Scalar e = Scalar(1e-8);
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    const Vec6s& box(boxes[i]);
    Neighbors& n(neighbors[i]);
    Scalar x(box[0]);
    Scalar y(box[1]);
    Scalar z(box[2]);
    Scalar s(box[3]);
    if (fixedSize == -1)
      fixedSize = s;
    else
      assert(s == fixedSize);

    for (VectorVec6s::const_iterator it = boxes.begin(); it != boxes.end();
         ++it) {
      const Vec6s& otherBox = *it;
      Scalar xo(otherBox[0]);
      Scalar yo(otherBox[1]);
      Scalar zo(otherBox[2]);
      // if (fabs(x-xo) < e && fabs(y-yo) < e && fabs(z-zo) < e){
      //   continue;
      // }
      if ((fabs(x - xo - s) < e) && (fabs(y - yo) < e) && (fabs(z - zo) < e)) {
        n.hasNeighboordMinusX();
      } else if ((fabs(x - xo + s) < e) && (fabs(y - yo) < e) &&
                 (fabs(z - zo) < e)) {
        n.hasNeighboordPlusX();
      } else if ((fabs(x - xo) < e) && (fabs(y - yo - s) < e) &&
                 (fabs(z - zo) < e)) {
        n.hasNeighboordMinusY();
      } else if ((fabs(x - xo) < e) && (fabs(y - yo + s) < e) &&
                 (fabs(z - zo) < e)) {
        n.hasNeighboordPlusY();
      } else if ((fabs(x - xo) < e) && (fabs(y - yo) < e) &&
                 (fabs(z - zo - s) < e)) {
        n.hasNeighboordMinusZ();
      } else if ((fabs(x - xo) < e) && (fabs(y - yo) < e) &&
                 (fabs(z - zo + s) < e)) {
        n.hasNeighboordPlusZ();
      }
    }
  }
}

}  // namespace internal

void OcTree::exportAsObjFile(const std::string& filename) const {
  std::vector<Vec6s> boxes(this->toBoxes());
  std::vector<internal::Neighbors> neighbors(boxes.size());
  internal::computeNeighbors(boxes, neighbors);
  // compute list of vertices and faces

  typedef std::vector<Vec3s> VectorVec3s;
  std::vector<Vec3s> vertices;

  typedef std::array<std::size_t, 4> Array4;
  typedef std::vector<Array4> VectorArray4;
  std::vector<Array4> faces;

  for (std::size_t i = 0; i < boxes.size(); ++i) {
    const Vec6s& box(boxes[i]);
    internal::Neighbors& n(neighbors[i]);

    Scalar x(box[0]);
    Scalar y(box[1]);
    Scalar z(box[2]);
    Scalar size(box[3]);

    const Scalar half = Scalar(0.5);
    vertices.push_back(
        Vec3s(x - half * size, y - half * size, z - half * size));
    vertices.push_back(
        Vec3s(x + half * size, y - half * size, z - half * size));
    vertices.push_back(
        Vec3s(x - half * size, y + half * size, z - half * size));
    vertices.push_back(
        Vec3s(x + half * size, y + half * size, z - half * size));
    vertices.push_back(
        Vec3s(x - half * size, y - half * size, z + half * size));
    vertices.push_back(
        Vec3s(x + half * size, y - half * size, z + half * size));
    vertices.push_back(
        Vec3s(x - half * size, y + half * size, z + half * size));
    vertices.push_back(
        Vec3s(x + half * size, y + half * size, z + half * size));

    // Add face only if box has no neighbor with the same face
    if (!n.minusX()) {
      Array4 a = {{8 * i + 1, 8 * i + 5, 8 * i + 7, 8 * i + 3}};
      faces.push_back(a);
    }
    if (!n.plusX()) {
      Array4 a = {{8 * i + 2, 8 * i + 4, 8 * i + 8, 8 * i + 6}};
      faces.push_back(a);
    }
    if (!n.minusY()) {
      Array4 a = {{8 * i + 1, 8 * i + 2, 8 * i + 6, 8 * i + 5}};
      faces.push_back(a);
    }
    if (!n.plusY()) {
      Array4 a = {{8 * i + 4, 8 * i + 3, 8 * i + 7, 8 * i + 8}};
      faces.push_back(a);
    }
    if (!n.minusZ()) {
      Array4 a = {{8 * i + 1, 8 * i + 2, 8 * i + 4, 8 * i + 3}};
      faces.push_back(a);
    }
    if (!n.plusZ()) {
      Array4 a = {{8 * i + 5, 8 * i + 6, 8 * i + 8, 8 * i + 7}};
      faces.push_back(a);
    }
  }
  // write obj in a file
  std::ofstream os;
  os.open(filename);
  if (!os.is_open())
    COAL_THROW_PRETTY(
        (std::string("failed to open file \"") + filename + std::string("\""))
            .c_str(),
        std::runtime_error);
  // write vertices
  os << "# list of vertices\n";
  for (VectorVec3s::const_iterator it = vertices.begin(); it != vertices.end();
       ++it) {
    const Vec3s& v = *it;
    os << "v " << v[0] << " " << v[1] << " " << v[2] << '\n';
  }
  os << "\n# list of faces\n";
  for (VectorArray4::const_iterator it = faces.begin(); it != faces.end();
       ++it) {
    const Array4& f = *it;
    os << "f " << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << '\n';
  }
}

OcTreePtr_t makeOctree(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 3>& point_cloud,
    const Scalar resolution) {
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 3> InputType;
  typedef InputType::ConstRowXpr RowType;

  shared_ptr<octomap::OcTree> octree(new octomap::OcTree(resolution));
  for (Eigen::DenseIndex row_id = 0; row_id < point_cloud.rows(); ++row_id) {
    RowType row = point_cloud.row(row_id);
    octree->updateNode(row[0], row[1], row[2], true, true);
  }
  octree->updateInnerOccupancy();

  return OcTreePtr_t(new OcTree(octree));
}
}  // namespace coal
