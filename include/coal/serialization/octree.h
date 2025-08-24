//
// Copyright (c) 2023-2024 INRIA
//

#ifndef COAL_SERIALIZATION_OCTREE_H
#define COAL_SERIALIZATION_OCTREE_H

#include <sstream>
#include <iostream>

#include <boost/serialization/string.hpp>

#include "coal/octree.h"
#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

namespace internal {
struct OcTreeAccessor : coal::OcTree {
  typedef coal::OcTree Base;
  using Base::default_occupancy;
  using Base::free_threshold;
  using Base::occupancy_threshold;
  using Base::tree;
};
}  // namespace internal

template <class Archive>
void save_construct_data(Archive &ar, const coal::OcTree *octree_ptr,
                         const unsigned int /*version*/) {
  const coal::Scalar resolution = octree_ptr->getResolution();
  ar << make_nvp("resolution", resolution);
}

template <class Archive>
void save(Archive &ar, const coal::OcTree &octree,
          const unsigned int /*version*/) {
  typedef internal::OcTreeAccessor Accessor;
  const Accessor &access = reinterpret_cast<const Accessor &>(octree);

  std::ostringstream stream;
  access.tree->write(stream);
  const std::string stream_str = stream.str();
  auto size = stream_str.size();
  // We can't directly serialize stream_str because it contains binary data.
  // This create a bug on Windows with text_archive
  ar << make_nvp("tree_data_size", size);
  ar << make_nvp("tree_data",
                 make_array(stream_str.c_str(), stream_str.size()));

  ar << make_nvp("base", base_object<coal::CollisionGeometry>(octree));
  ar << make_nvp("default_occupancy", access.default_occupancy);
  ar << make_nvp("occupancy_threshold", access.occupancy_threshold);
  ar << make_nvp("free_threshold", access.free_threshold);
}

template <class Archive>
void load_construct_data(Archive &ar, coal::OcTree *octree_ptr,
                         const unsigned int /*version*/) {
  coal::Scalar resolution;
  ar >> make_nvp("resolution", resolution);
  new (octree_ptr) coal::OcTree(resolution);
}

template <class Archive>
void load(Archive &ar, coal::OcTree &octree, const unsigned int /*version*/) {
  typedef internal::OcTreeAccessor Accessor;
  Accessor &access = reinterpret_cast<Accessor &>(octree);

  std::size_t tree_data_size;
  ar >> make_nvp("tree_data_size", tree_data_size);

  std::string stream_str;
  stream_str.resize(tree_data_size);
  /// TODO use stream_str.data in C++17
  assert(tree_data_size > 0 && "tree_data_size should be greater than 0");
  ar >> make_nvp("tree_data", make_array(&stream_str[0], tree_data_size));
  std::istringstream stream(stream_str);

  octomap::AbstractOcTree *new_tree = octomap::AbstractOcTree::read(stream);
  access.tree = std::shared_ptr<const octomap::OcTree>(
      dynamic_cast<octomap::OcTree *>(new_tree));

  ar >> make_nvp("base", base_object<coal::CollisionGeometry>(octree));
  ar >> make_nvp("default_occupancy", access.default_occupancy);
  ar >> make_nvp("occupancy_threshold", access.occupancy_threshold);
  ar >> make_nvp("free_threshold", access.free_threshold);
}

template <class Archive>
void serialize(Archive &ar, coal::OcTree &octree, const unsigned int version) {
  split_free(ar, octree, version);
}

}  // namespace serialization
}  // namespace boost

COAL_SERIALIZATION_DECLARE_EXPORT(::coal::OcTree)

#endif  // ifndef COAL_SERIALIZATION_OCTREE_H
